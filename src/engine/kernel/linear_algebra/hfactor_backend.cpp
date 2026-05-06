// ═══════════════════════════════════════════════════════════════════════════
// HFactorBackend — implementation
//
// Wraps the vendored HiGHS HFactor (see
// src/engine/kernel/linear_algebra/highs_factor/) in an API that mirrors
// `hacdcpf::engine::SparseLUFactor`.
//
// Phase 2 of U.7.118.  Phase 3 will plug this into the dual-simplex driver
// behind a new `FactorBackendKind::HFactorPort` enum value.
// ═══════════════════════════════════════════════════════════════════════════

#include "hacdcpf/engine/kernel/linear_algebra/hfactor_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// HiGHS headers (vendored copies).
#include "util/HFactor.h"
#include "util/HVector.h"

namespace hacdcpf::engine {

struct HFactorBackend::Impl {
  HFactor f;

  // CSC copy of the bound A matrix in HighsInt format (HFactor::setupGeneral
  // takes `const HighsInt*` pointers and stores them by reference, so we keep
  // ownership here).
  std::vector<HighsInt> a_start;
  std::vector<HighsInt> a_index;
  std::vector<double>   a_value;

  // Mutable basic_index (HFactor permutes it during build()).
  std::vector<HighsInt> basic_index;

  // Cached dimensions (for ftran/btran sanity checks).
  HighsInt num_row = 0;
  HighsInt num_col = 0;

  // Set by the most-recent factorize() to indicate the bound matrix is
  // ready for build/ftran/btran/update.
  bool setup_done = false;
};

HFactorBackend::HFactorBackend() : p_(std::make_unique<Impl>()) {}
HFactorBackend::~HFactorBackend() = default;

HFactorBackend::HFactorBackend(HFactorBackend&&) noexcept = default;
HFactorBackend& HFactorBackend::operator=(HFactorBackend&&) noexcept = default;

// ────────────────────────────────────────────────────────────────────────────
// Helper: pack a dense double[size] into an HVector (count + index + array +
// packIndex/packValue).  Values whose magnitude is below `tiny` are dropped.
// ────────────────────────────────────────────────────────────────────────────
namespace {

constexpr double kPackTiny = 1e-30;

void pack_dense_into_hvector(const double* src, HighsInt n, HVector& v) {
  v.setup(n);
  v.clear();
  v.array.assign(src, src + n);
  v.count = 0;
  v.index.resize(static_cast<size_t>(n));
  for (HighsInt i = 0; i < n; ++i) {
    if (std::fabs(src[i]) > kPackTiny) {
      v.index[static_cast<size_t>(v.count++)] = i;
    }
  }
  // Provide pack values too — HFactor::updateFT iterates packIndex/packValue.
  v.packFlag = true;
  v.pack();
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────────
// factorize
// ────────────────────────────────────────────────────────────────────────────
bool HFactorBackend::factorize(const Eigen::SparseMatrix<double>& A,
                               const int* basic_index,
                               int n_basic) {
  valid = false;
  n_updates = 0;
  rank_deficiency = 0;
  refactor_hint_ = 0;

  if (n_basic <= 0 || basic_index == nullptr) return false;
  if (A.rows() <= 0 || A.cols() <= 0) return false;

  // Eigen sparse must be column-major and compressed.
  Eigen::SparseMatrix<double> Acm(A);
  Acm.makeCompressed();

  const HighsInt num_row = static_cast<HighsInt>(Acm.rows());
  const HighsInt num_col = static_cast<HighsInt>(Acm.cols());
  const HighsInt nnz     = static_cast<HighsInt>(Acm.nonZeros());

  // Copy Eigen CSC (StorageIndex int) into HighsInt (== int on this build,
  // but stay defensive in case HIGHSINT64 ever turns on).
  p_->a_start.assign(num_col + 1, 0);
  for (HighsInt j = 0; j <= num_col; ++j) {
    p_->a_start[static_cast<size_t>(j)] =
        static_cast<HighsInt>(Acm.outerIndexPtr()[j]);
  }
  p_->a_index.resize(static_cast<size_t>(nnz));
  for (HighsInt k = 0; k < nnz; ++k) {
    p_->a_index[static_cast<size_t>(k)] =
        static_cast<HighsInt>(Acm.innerIndexPtr()[k]);
  }
  p_->a_value.assign(Acm.valuePtr(), Acm.valuePtr() + nnz);

  // basic_index is mutable for HFactor (it permutes during build).
  p_->basic_index.resize(static_cast<size_t>(n_basic));
  for (int i = 0; i < n_basic; ++i) {
    const int col = basic_index[i];
    if (col < 0 || col >= static_cast<int>(num_col)) return false;
    p_->basic_index[static_cast<size_t>(i)] = static_cast<HighsInt>(col);
  }

  p_->num_row = num_row;
  p_->num_col = num_col;

  // setupGeneral: pass num_basic == n_basic (== num_row in normal usage).
  // We use the original-HFactor logic and the Forrest-Tomlin update method,
  // matching HiGHS' simplex defaults.
  p_->f.setupGeneral(
      /*num_col*/ num_col,
      /*num_row*/ num_row,
      /*num_basic*/ static_cast<HighsInt>(n_basic),
      p_->a_start.data(),
      p_->a_index.data(),
      p_->a_value.data(),
      p_->basic_index.data(),
      kDefaultPivotThreshold,
      kDefaultPivotTolerance,
      kHighsDebugLevelMin,
      /*log_options*/ nullptr,
      /*use_original_HFactor_logic*/ true,
      /*update_method*/ kUpdateMethodFt);

  p_->setup_done = true;

  const HighsInt rd = p_->f.build(/*factor_timer_clock_pointer*/ nullptr);
  rank_deficiency = static_cast<int>(rd);
  if (rd != 0) {
    valid = false;
    return false;
  }

  m = static_cast<int>(num_row);
  valid = true;
  return true;
}

// ────────────────────────────────────────────────────────────────────────────
// FTRAN — dense in/out.
// HFactor's ftranCall(std::vector<double>&) overload moves the buffer in,
// so we copy into a temporary vector.
// ────────────────────────────────────────────────────────────────────────────
void HFactorBackend::ftran(const double* rhs, double* result) const {
  if (!valid) return;
  std::vector<double> buf(static_cast<size_t>(m));
  std::memcpy(buf.data(), rhs, static_cast<size_t>(m) * sizeof(double));
  // ftranCall(std::vector<double>&) is non-const because it uses the internal
  // rhs_ HVector workspace.  Cast away constness on the stored HFactor — the
  // operation is logically const w.r.t. the factorization.
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
  std::memcpy(result, buf.data(), static_cast<size_t>(m) * sizeof(double));
}

void HFactorBackend::btran(const double* rhs, double* result) const {
  if (!valid) return;
  std::vector<double> buf(static_cast<size_t>(m));
  std::memcpy(buf.data(), rhs, static_cast<size_t>(m) * sizeof(double));
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
  std::memcpy(result, buf.data(), static_cast<size_t>(m) * sizeof(double));
}

// ────────────────────────────────────────────────────────────────────────────
// update — wraps HFactor::update(aq, ep, &iRow, &hint).
//
// `a_q_after_ftran` must be B^{-1} * a_q (the FTRAN-image of the entering
// column).  `btran_e_p` must be B^{-T} * e_{pivot_row} (the BTRAN-image of
// the unit pivot-row vector).  Both are dense length-m arrays.
// ────────────────────────────────────────────────────────────────────────────
bool HFactorBackend::update(int pivot_row,
                            const double* a_q_after_ftran,
                            const double* btran_e_p) {
  if (!valid) return false;
  if (pivot_row < 0 || pivot_row >= m) return false;

  HVector aq, ep;
  pack_dense_into_hvector(a_q_after_ftran, static_cast<HighsInt>(m), aq);
  pack_dense_into_hvector(btran_e_p,        static_cast<HighsInt>(m), ep);

  HighsInt iRow = static_cast<HighsInt>(pivot_row);
  HighsInt hint = 0;
  p_->f.update(&aq, &ep, &iRow, &hint);
  refactor_hint_ = static_cast<int>(hint);

  ++n_updates;
  return true;
}

void HFactorBackend::reset_update_tracking() noexcept {
  n_updates = 0;
  refactor_hint_ = 0;
}

}  // namespace hacdcpf::engine

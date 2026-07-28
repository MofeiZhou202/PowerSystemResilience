// ═══════════════════════════════════════════════════════════════════════════
// HFactorBackend — implementation
//
// Wraps the vendored HiGHS HFactor (see
// src/engine/kernel/linear_algebra/highs_factor/) in an API that mirrors
// `mipsolvers::engine::SparseLUFactor`.
//
// Phase 2 of U.7.118.  Phase 3 will plug this into the dual-simplex driver
// behind a new `FactorBackendKind::HFactorPort` enum value.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

// HiGHS headers (vendored copies).
#include "util/HFactor.h"
#include "util/HVector.h"

namespace mipsolvers::engine {

struct HFactorBackend::Impl {
  struct UpdatePack {
    std::vector<HighsInt> index;
    std::vector<double> value;
    std::uint64_t serial = 0;
    bool valid = false;
  };

  HFactor f;

  // CSC copy of the bound A matrix in HighsInt format (HFactor::setupGeneral
  // takes `const HighsInt*` pointers and stores them by reference, so we keep
  // ownership here).
  std::vector<HighsInt> a_start;
  std::vector<HighsInt> a_index;
  std::vector<double>   a_value;

  // Mutable basic_index (HFactor permutes it during build()).
  std::vector<HighsInt> basic_index;

  // HFactor stores basis variables by its pivot-row ordering.  The public
  // backend contract stores them in the caller's basis-position ordering.
  // external_to_internal[p] is the HFactor position of caller position p.
  std::vector<HighsInt> external_to_internal;

  // Cached dimensions (for ftran/btran sanity checks).
  HighsInt num_row = 0;
  HighsInt num_col = 0;

  // Set by the most-recent factorize() to indicate the bound matrix is
  // ready for build/ftran/btran/update.
  bool setup_done = false;

  // Reusable scratch buffer for ftran/btran to avoid per-call heap allocation
  // in the dual-simplex inner loop.  Sized to num_row after each factorize().
  mutable std::vector<double> solve_buf;
  mutable UpdatePack aq_pack;
  mutable UpdatePack ep_pack;
  mutable std::vector<double> aq_rhs;
  mutable std::vector<double> ep_rhs;
  mutable std::vector<double> aq_solution_internal;
  std::uint64_t factor_serial = 0;
};

HFactorBackend::HFactorBackend() : p_(std::make_unique<Impl>()) {}
HFactorBackend::~HFactorBackend() = default;

HFactorBackend::HFactorBackend(HFactorBackend&&) noexcept = default;
HFactorBackend& HFactorBackend::operator=(HFactorBackend&&) noexcept = default;

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
  ++p_->factor_serial;
  p_->aq_pack.valid = false;
  p_->ep_pack.valid = false;

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
  // This standalone HFactor port supports Forrest-Tomlin updates. The MPF/PF
  // solve branches are deliberately disabled in the vendored factor source;
  // selecting them makes the update representation inconsistent with FTRAN.
  const HighsInt update_method = kUpdateMethodFt;
  // Pivot tolerance (min absolute pivot for rank determination) is
  // env-overridable for diagnosis: the default 1e-10 declares mildly
  // ill-conditioned SCUC bases rank-deficient where UMFPACK still succeeds.
  double pivot_tol = kDefaultPivotTolerance;
  if (const char* e = std::getenv("MIPSOLVERS_HFACTOR_PIVOT_TOL")) {
    const double v = std::atof(e);
    if (v >= 0.0 && v <= 1.0) pivot_tol = v;
  }
  p_->f.setupGeneral(
      /*num_col*/ num_col,
      /*num_row*/ num_row,
      /*num_basic*/ static_cast<HighsInt>(n_basic),
      p_->a_start.data(),
      p_->a_index.data(),
      p_->a_value.data(),
      p_->basic_index.data(),
      kDefaultPivotThreshold,
      pivot_tol,
      kHighsDebugLevelMin,
      /*log_options*/ nullptr,
      /*use_original_HFactor_logic*/ true,
      /*update_method*/ update_method);

  p_->setup_done = true;

  const HighsInt rd = p_->f.build(/*factor_timer_clock_pointer*/ nullptr);
  rank_deficiency = static_cast<int>(rd);
  if (rd != 0) {
    // Export the no-pivot rows/vars for HiGHS-style basis repair by the
    // caller (replace the row's basic with its logical column).
    no_pivot_rows_.assign(p_->f.row_with_no_pivot.begin(),
                          p_->f.row_with_no_pivot.end());
    no_pivot_vars_.assign(p_->f.var_with_no_pivot.begin(),
                          p_->f.var_with_no_pivot.end());
    p_->external_to_internal.clear();
    valid = false;
    return false;
  }
  no_pivot_rows_.clear();
  no_pivot_vars_.clear();

  // buildFinish() permutes basic_index into HFactor's pivot-row order.  Keep
  // that permutation private: every public solve/update remains expressed in
  // the exact basis-position order supplied by the caller.
  std::vector<int> external_position_by_col(static_cast<std::size_t>(num_col),
                                            -1);
  for (int external = 0; external < n_basic; ++external) {
    const int col = basic_index[external];
    if (external_position_by_col[static_cast<std::size_t>(col)] >= 0) {
      p_->external_to_internal.clear();
      return false;
    }
    external_position_by_col[static_cast<std::size_t>(col)] = external;
  }
  p_->external_to_internal.assign(static_cast<std::size_t>(n_basic), -1);
  for (int internal = 0; internal < n_basic; ++internal) {
    const int col =
        static_cast<int>(p_->basic_index[static_cast<std::size_t>(internal)]);
    if (col < 0 || col >= static_cast<int>(num_col)) return false;
    const int external =
        external_position_by_col[static_cast<std::size_t>(col)];
    if (external < 0 ||
        p_->external_to_internal[static_cast<std::size_t>(external)] >= 0) {
      p_->external_to_internal.clear();
      return false;
    }
    p_->external_to_internal[static_cast<std::size_t>(external)] = internal;
  }

  p_->solve_buf.assign(static_cast<size_t>(num_row), 0.0);

  m = static_cast<int>(num_row);
  valid = true;
  return true;
}

bool HFactorBackend::factorize_with_logicals(
    const Eigen::SparseMatrix<double>& A,
    const int* basic_index,
    int n_basic,
    const std::vector<int>& logical_col_by_row,
    std::vector<int>& repaired_basis) {
  const int num_row = static_cast<int>(A.rows());
  const int num_col = static_cast<int>(A.cols());
  repaired_basis.clear();
  if (n_basic != num_row || basic_index == nullptr ||
      static_cast<int>(logical_col_by_row.size()) != num_row) {
    valid = false;
    return false;
  }

  std::vector<int> logical_row_by_col(static_cast<std::size_t>(num_col), -1);
  for (int row = 0; row < num_row; ++row) {
    const int col = logical_col_by_row[static_cast<std::size_t>(row)];
    if (col < 0 || col >= num_col ||
        logical_row_by_col[static_cast<std::size_t>(col)] >= 0) {
      valid = false;
      return false;
    }
    logical_row_by_col[static_cast<std::size_t>(col)] = row;
  }

  std::vector<int> hfactor_basis(static_cast<std::size_t>(n_basic), -1);
  for (int pos = 0; pos < n_basic; ++pos) {
    const int col = basic_index[pos];
    if (col < 0 || col >= num_col) {
      valid = false;
      return false;
    }
    const int logical_row =
        logical_row_by_col[static_cast<std::size_t>(col)];
    hfactor_basis[static_cast<std::size_t>(pos)] =
        logical_row >= 0 ? num_col + logical_row : col;
  }

  // This is intentionally parallel to factorize(), except that HFactor is
  // allowed to complete a deficient input basis with implicit logicals.
  valid = false;
  n_updates = 0;
  rank_deficiency = 0;
  refactor_hint_ = 0;
  no_pivot_rows_.clear();
  no_pivot_vars_.clear();

  using HighsInt = ::HighsInt;
  const HighsInt h_num_row = static_cast<HighsInt>(num_row);
  const HighsInt h_num_col = static_cast<HighsInt>(num_col);
  if (num_row <= 0 || num_col <= 0 || !A.isCompressed()) return false;

  p_->a_start.resize(static_cast<std::size_t>(num_col) + 1);
  p_->a_index.resize(static_cast<std::size_t>(A.nonZeros()));
  p_->a_value.resize(static_cast<std::size_t>(A.nonZeros()));
  for (int j = 0; j <= num_col; ++j)
    p_->a_start[static_cast<std::size_t>(j)] =
        static_cast<HighsInt>(A.outerIndexPtr()[j]);
  for (Eigen::Index k = 0; k < A.nonZeros(); ++k) {
    p_->a_index[static_cast<std::size_t>(k)] =
        static_cast<HighsInt>(A.innerIndexPtr()[k]);
    p_->a_value[static_cast<std::size_t>(k)] = A.valuePtr()[k];
  }
  p_->basic_index.resize(static_cast<std::size_t>(n_basic));
  for (int i = 0; i < n_basic; ++i)
    p_->basic_index[static_cast<std::size_t>(i)] =
        static_cast<HighsInt>(hfactor_basis[static_cast<std::size_t>(i)]);

  p_->num_row = h_num_row;
  p_->num_col = h_num_col;
  const HighsInt update_method = kUpdateMethodFt;
  p_->f.setupGeneral(h_num_col, h_num_row, static_cast<HighsInt>(n_basic),
                     p_->a_start.data(), p_->a_index.data(),
                     p_->a_value.data(), p_->basic_index.data(),
                     kDefaultPivotThreshold, kDefaultPivotTolerance,
                     kHighsDebugLevelMin, nullptr, true, update_method);
  p_->setup_done = true;

  const HighsInt rd = p_->f.build(nullptr);
  if (rd < 0) return false;
  rank_deficiency = static_cast<int>(rd);
  no_pivot_rows_.assign(p_->f.row_with_no_pivot.begin(),
                        p_->f.row_with_no_pivot.end());
  no_pivot_vars_.assign(p_->f.var_with_no_pivot.begin(),
                        p_->f.var_with_no_pivot.end());

  // HFactor's post-build basic_index is in an internal pivot-row order and
  // must never become the caller's basis.  Use its rank diagnostics only:
  // replace the reported singular input positions, leaving every other
  // caller position untouched.
  repaired_basis.assign(basic_index, basic_index + n_basic);
  std::vector<char> repaired_position(static_cast<std::size_t>(n_basic), 0);
  for (int k = 0; k < rank_deficiency; ++k) {
    if (k >= static_cast<int>(no_pivot_rows_.size()) ||
        k >= static_cast<int>(no_pivot_vars_.size())) {
      return false;
    }
    const int missing_row = no_pivot_rows_[static_cast<std::size_t>(k)];
    const int missing_var = no_pivot_vars_[static_cast<std::size_t>(k)];
    if (missing_row < 0 || missing_row >= num_row) return false;
    const auto position =
        std::find(hfactor_basis.begin(), hfactor_basis.end(), missing_var);
    if (position == hfactor_basis.end()) return false;
    const int external =
        static_cast<int>(std::distance(hfactor_basis.begin(), position));
    if (repaired_position[static_cast<std::size_t>(external)]) return false;
    repaired_position[static_cast<std::size_t>(external)] = 1;
    repaired_basis[static_cast<std::size_t>(external)] =
        logical_col_by_row[static_cast<std::size_t>(missing_row)];
  }
  std::vector<char> seen_repaired(static_cast<std::size_t>(num_col), 0);
  for (int col : repaired_basis) {
    if (col < 0 || col >= num_col ||
        seen_repaired[static_cast<std::size_t>(col)]) {
      return false;
    }
    seen_repaired[static_cast<std::size_t>(col)] = 1;
  }

  const int repair_count = rank_deficiency;
  const std::vector<int> repair_rows = no_pivot_rows_;
  const std::vector<int> repair_vars = no_pivot_vars_;

  // The caller's logical columns may be Ruiz-scaled multiples of e_i, while
  // HFactor's implicit logical is exactly e_i.  They have identical rank but
  // different numerical values, so rebuild once in the real column space.
  if (!factorize(A, repaired_basis.data(), n_basic)) return false;
  rank_deficiency = repair_count;
  no_pivot_rows_ = repair_rows;
  no_pivot_vars_ = repair_vars;
  return true;
}

// ────────────────────────────────────────────────────────────────────────────
// FTRAN — dense in/out.
// HFactor's ftranCall(std::vector<double>&) overload moves the buffer in,
// so we copy into a temporary vector.
// ────────────────────────────────────────────────────────────────────────────
void HFactorBackend::ftran(const double* rhs, double* result) const {
  if (!valid) return;
  // Use pre-allocated scratch buffer to avoid per-call heap allocation.
  std::vector<double>& buf = p_->solve_buf;
  std::memcpy(buf.data(), rhs, static_cast<size_t>(m) * sizeof(double));
  // ftranCall(std::vector<double>&) is non-const because it uses the internal
  // rhs_ HVector workspace.  Cast away constness on the stored HFactor — the
  // operation is logically const w.r.t. the factorization.
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(
        p_->external_to_internal[static_cast<std::size_t>(external)]);
    result[external] = buf[static_cast<std::size_t>(internal)];
  }
}

void HFactorBackend::ftran_for_update(const double* rhs, double* result) const {
  if (!valid) return;
  HVector vector;
  vector.setup(static_cast<HighsInt>(m));
  vector.packFlag = true;
  for (int row = 0; row < m; ++row) {
    const double value = rhs[row];
    vector.array[static_cast<std::size_t>(row)] = value;
    if (value != 0.0) {
      vector.index[static_cast<std::size_t>(vector.count++)] = row;
    }
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(vector, 1.0, nullptr);
  p_->aq_rhs.assign(rhs, rhs + m);
  p_->aq_solution_internal = vector.array;
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(
        p_->external_to_internal[static_cast<std::size_t>(external)]);
    result[external] = vector.array[static_cast<std::size_t>(internal)];
  }
  p_->aq_pack.index.assign(vector.packIndex.begin(),
                           vector.packIndex.begin() + vector.packCount);
  p_->aq_pack.value.assign(vector.packValue.begin(),
                           vector.packValue.begin() + vector.packCount);
  p_->aq_pack.serial = p_->factor_serial;
  p_->aq_pack.valid = true;
}

void HFactorBackend::btran(const double* rhs, double* result) const {
  if (!valid) return;
  // Use pre-allocated scratch buffer to avoid per-call heap allocation.
  std::vector<double>& buf = p_->solve_buf;
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(
        p_->external_to_internal[static_cast<std::size_t>(external)]);
    buf[static_cast<std::size_t>(internal)] =
        rhs[external];
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
  std::memcpy(result, buf.data(), static_cast<size_t>(m) * sizeof(double));
}

void HFactorBackend::btran_for_update(const double* rhs, double* result) const {
  if (!valid) return;
  HVector vector;
  vector.setup(static_cast<HighsInt>(m));
  vector.packFlag = true;
  for (int external = 0; external < m; ++external) {
    const double value = rhs[external];
    const HighsInt internal =
        p_->external_to_internal[static_cast<std::size_t>(external)];
    vector.array[static_cast<std::size_t>(internal)] = value;
    if (value != 0.0) {
      vector.index[static_cast<std::size_t>(vector.count++)] = internal;
    }
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(vector, 1.0, nullptr);
  p_->ep_rhs.assign(rhs, rhs + m);
  std::memcpy(result, vector.array.data(), static_cast<size_t>(m) * sizeof(double));
  p_->ep_pack.index.assign(vector.packIndex.begin(),
                           vector.packIndex.begin() + vector.packCount);
  p_->ep_pack.value.assign(vector.packValue.begin(),
                           vector.packValue.begin() + vector.packCount);
  p_->ep_pack.serial = p_->factor_serial;
  p_->ep_pack.valid = true;
}

// ────────────────────────────────────────────────────────────────────────────
// update — apply HFactor's Forrest-Tomlin exchange in its pivot-row space.
//
// `a_q_after_ftran` must be B^{-1} * a_q (the FTRAN-image of the entering
// column).  `btran_e_p` must be B^{-T} * e_{pivot_row} (the BTRAN-image of
// the unit pivot-row vector).  Both are dense length-m arrays.
// ────────────────────────────────────────────────────────────────────────────
bool HFactorBackend::update(int pivot_row,
                            int entering_col,
                            const double* a_q_after_ftran,
                            const double* btran_e_p) {
  if (!valid) return false;
  if (pivot_row < 0 || pivot_row >= m) return false;
  if (entering_col < 0 || entering_col >= p_->num_col) return false;
  if (a_q_after_ftran == nullptr || btran_e_p == nullptr) return false;
  if (!p_->aq_pack.valid || p_->aq_pack.serial != p_->factor_serial) {
    if (static_cast<int>(p_->aq_rhs.size()) != m) return false;
    const std::vector<double> rhs = p_->aq_rhs;
    std::vector<double> refreshed(static_cast<std::size_t>(m));
    ftran_for_update(rhs.data(), refreshed.data());
  }
  if (!p_->ep_pack.valid || p_->ep_pack.serial != p_->factor_serial) {
    if (static_cast<int>(p_->ep_rhs.size()) != m) return false;
    const std::vector<double> rhs = p_->ep_rhs;
    std::vector<double> refreshed(static_cast<std::size_t>(m));
    btran_for_update(rhs.data(), refreshed.data());
  }

  const HighsInt internal_pivot =
      p_->external_to_internal[static_cast<std::size_t>(pivot_row)];
  if (internal_pivot < 0 || internal_pivot >= m) return false;

  HVector aq;
  HVector ep;
  aq.setup(static_cast<HighsInt>(m));
  ep.setup(static_cast<HighsInt>(m));
  for (int external = 0; external < m; ++external) {
    if (!std::isfinite(a_q_after_ftran[external])) return false;
    const HighsInt internal =
        p_->external_to_internal[static_cast<std::size_t>(external)];
    aq.array[static_cast<std::size_t>(internal)] =
        p_->aq_solution_internal[static_cast<std::size_t>(internal)];
  }
  for (int row = 0; row < m; ++row) {
    const double value = btran_e_p[row];
    if (!std::isfinite(value)) return false;
    ep.array[static_cast<std::size_t>(row)] = value;
  }
  const double pivot = aq.array[static_cast<std::size_t>(internal_pivot)];
  if (pivot == 0.0 || !std::isfinite(1.0 / pivot)) return false;
  aq.packCount = static_cast<HighsInt>(p_->aq_pack.index.size());
  ep.packCount = static_cast<HighsInt>(p_->ep_pack.index.size());
  std::copy(p_->aq_pack.index.begin(), p_->aq_pack.index.end(),
            aq.packIndex.begin());
  std::copy(p_->aq_pack.value.begin(), p_->aq_pack.value.end(),
            aq.packValue.begin());
  std::copy(p_->ep_pack.index.begin(), p_->ep_pack.index.end(),
            ep.packIndex.begin());
  std::copy(p_->ep_pack.value.begin(), p_->ep_pack.value.end(),
            ep.packValue.begin());

  p_->basic_index[static_cast<std::size_t>(internal_pivot)] =
      static_cast<HighsInt>(entering_col);
  HighsInt update_row = internal_pivot;
  HighsInt hint = 0;
  p_->f.update(&aq, &ep, &update_row, &hint);

  ++n_updates;
  ++p_->factor_serial;
  p_->aq_pack.valid = false;
  p_->ep_pack.valid = false;
  p_->aq_rhs.clear();
  p_->ep_rhs.clear();
  p_->aq_solution_internal.clear();
  refactor_hint_ = static_cast<int>(hint);
  return true;
}

void HFactorBackend::reset_update_tracking() noexcept {
  n_updates = 0;
  refactor_hint_ = 0;
}

}  // namespace mipsolvers::engine

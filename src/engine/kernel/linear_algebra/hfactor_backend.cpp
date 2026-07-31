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

namespace {
// FTRAN/BTRAN engage HFactor's hyper-sparse solve strategy only for bases at
// least this large.  Small systems keep the exact dense solve path: hyper-
// sparsity does not pay off there, and the dense solve is numerically steadier
// on ill-conditioned small probes (the adversarial 1e6-scaled NETLIB cases,
// where the hyper-sparse rounding otherwise tips dual Phase I over an
// invariant).  Env-tunable via MIPSOLVERS_HFACTOR_HYPERSPARSE_MINROWS.
int hyper_sparse_min_rows() {
  static const int v = [] {
    const char* e = std::getenv("MIPSOLVERS_HFACTOR_HYPERSPARSE_MINROWS");
    if (e) {
      const int x = std::atoi(e);
      if (x >= 0) return x;
    }
    return 2000;
  }();
  return v;
}
}  // namespace

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

  // HFactor stores basis variables by its pivot-row ordering.  The public
  // backend contract stores them in the caller's basis-position ordering.
  // external_to_internal[p] is the HFactor position of caller position p.
  std::vector<HighsInt> external_to_internal;
  std::vector<HighsInt> internal_to_external;

  // Cached dimensions (for ftran/btran sanity checks).
  HighsInt num_row = 0;
  HighsInt num_col = 0;

  // Set by the most-recent factorize() to indicate the bound matrix is
  // ready for build/ftran/btran/update.
  bool setup_done = false;

  // Reusable scratch buffer for ftran/btran to avoid per-call heap allocation
  // in the dual-simplex inner loop.  Sized to num_row after each factorize().
  mutable std::vector<double> solve_buf;
  // Reusable HVectors so the plain ftran/btran can drive HFactor's hyper-sparse
  // solve path (real RHS density) without per-call HVector setup.
  mutable HVector solve_vec_ftran;
  mutable HVector solve_vec_btran;
  // Pivotal solves must retain their HFactor pack data until update(). Reuse
  // the complete HVectors directly: reconstructing them from copied dense and
  // packed buffers allocated and initialized several O(m) arrays per pivot.
  mutable HVector update_vec_aq;
  mutable HVector update_vec_ep;
  mutable std::uint64_t aq_capture_serial = 0;
  mutable std::uint64_t ep_capture_serial = 0;
  mutable bool aq_capture_valid = false;
  mutable bool ep_capture_valid = false;
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
  p_->aq_capture_valid = false;
  p_->ep_capture_valid = false;

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
  p_->internal_to_external.assign(static_cast<std::size_t>(n_basic), -1);
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
    p_->internal_to_external[static_cast<std::size_t>(internal)] = external;
  }

  p_->solve_buf.assign(static_cast<size_t>(num_row), 0.0);
  p_->solve_vec_ftran.setup(static_cast<HighsInt>(num_row));
  p_->solve_vec_btran.setup(static_cast<HighsInt>(num_row));
  p_->update_vec_aq.setup(static_cast<HighsInt>(num_row));
  p_->update_vec_ep.setup(static_cast<HighsInt>(num_row));

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

  // Fast path: a healthy simplex basis is full rank in the real column
  // space, so one real-space build both proves that and produces the working
  // factor. The implicit-logical diagnostic build below exists only to
  // identify repairable singular positions, so pay for it only after a
  // real-space build reports deficiency.
  if (factorize(A, basic_index, n_basic)) {
    repaired_basis.assign(basic_index, basic_index + n_basic);
    return true;
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
void HFactorBackend::ftran(const double* rhs, double* result,
                           const std::vector<int>* rhs_pattern,
                           std::vector<int>* result_pattern) const {
  if (result_pattern != nullptr) result_pattern->clear();
  if (!valid) return;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  if (m < hyper_sparse_min_rows()) {
    // Small system: exact dense path (byte-identical to pre-hyper-sparse).
    std::vector<double>& buf = p_->solve_buf;
    std::memcpy(buf.data(), rhs, static_cast<size_t>(m) * sizeof(double));
    nc.ftranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(
          p_->external_to_internal[static_cast<std::size_t>(external)]);
      result[external] = buf[static_cast<std::size_t>(internal)];
      if (result_pattern != nullptr && result[external] != 0.0) {
        result_pattern->push_back(external);
      }
    }
    return;
  }
  // Large system: hyper-sparse FTRAN via a reused HVector.  Overwrite every
  // array entry (so no stale solution leaks in) and record the nonzero pattern;
  // the real RHS density selects sparse vs dense internally.  Same permutation
  // convention as the dense path (no input remap, output remapped).
  HVector& vector = p_->solve_vec_ftran;
  vector.clear();
  vector.packFlag = false;
  if (rhs_pattern != nullptr) {
    for (int row : *rhs_pattern) {
      if (row < 0 || row >= m) continue;
      const double value = rhs[row];
      vector.array[static_cast<std::size_t>(row)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = row;
      }
    }
  } else {
    for (int row = 0; row < m; ++row) {
      const double value = rhs[row];
      vector.array[static_cast<std::size_t>(row)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = row;
      }
    }
  }
  const double density =
      static_cast<double>(vector.count) / static_cast<double>(m);
  nc.ftranCall(vector, density, nullptr);
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(
        p_->external_to_internal[static_cast<std::size_t>(external)]);
    result[external] = vector.array[static_cast<std::size_t>(internal)];
  }
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
      result_pattern->push_back(static_cast<int>(
          p_->internal_to_external[static_cast<std::size_t>(internal)]));
    }
  }
}

void HFactorBackend::ftran_for_update(
    const double* rhs, double* result, const std::vector<int>* rhs_pattern,
    std::vector<int>* result_pattern) const {
  if (result_pattern != nullptr) result_pattern->clear();
  if (!valid) return;
  HVector& vector = p_->update_vec_aq;
  vector.clear();
  vector.packFlag = true;
  if (rhs_pattern != nullptr) {
    for (int row : *rhs_pattern) {
      if (row < 0 || row >= m) continue;
      const double value = rhs[row];
      vector.array[static_cast<std::size_t>(row)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = row;
      }
    }
  } else {
    for (int row = 0; row < m; ++row) {
      const double value = rhs[row];
      vector.array[static_cast<std::size_t>(row)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = row;
      }
    }
  }
  // Pass the true RHS density so HFactor selects its hyper-sparse FTRAN
  // strategy on sparse right-hand sides (SCUC pivotal columns are sparse);
  // the solve result is identical to the dense strategy, only faster.  Small
  // bases force density 1.0 (dense) for numerical steadiness (size gate).
  const double density =
      m >= hyper_sparse_min_rows()
          ? static_cast<double>(vector.count) / static_cast<double>(m)
          : 1.0;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(vector, density, nullptr);
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(
        p_->external_to_internal[static_cast<std::size_t>(external)]);
    result[external] = vector.array[static_cast<std::size_t>(internal)];
  }
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
      result_pattern->push_back(static_cast<int>(
          p_->internal_to_external[static_cast<std::size_t>(internal)]));
    }
  }
  p_->aq_capture_serial = p_->factor_serial;
  p_->aq_capture_valid = true;
}

void HFactorBackend::btran(const double* rhs, double* result,
                           const std::vector<int>* rhs_pattern,
                           std::vector<int>* result_pattern) const {
  if (result_pattern != nullptr) result_pattern->clear();
  if (!valid) return;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  if (m < hyper_sparse_min_rows()) {
    // Small system: exact dense path (byte-identical to pre-hyper-sparse).
    std::vector<double>& buf = p_->solve_buf;
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(
          p_->external_to_internal[static_cast<std::size_t>(external)]);
      buf[static_cast<std::size_t>(internal)] = rhs[external];
    }
    nc.btranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
    std::memcpy(result, buf.data(), static_cast<size_t>(m) * sizeof(double));
    if (result_pattern != nullptr) {
      for (int row = 0; row < m; ++row) {
        if (result[row] != 0.0) result_pattern->push_back(row);
      }
    }
    return;
  }
  // Large system: hyper-sparse BTRAN via a reused HVector.  Same permutation
  // convention as the dense path (input remapped external->internal, output not
  // remapped).  Overwrite every array entry so no stale solution leaks in.
  HVector& vector = p_->solve_vec_btran;
  vector.clear();
  vector.packFlag = false;
  if (rhs_pattern != nullptr) {
    for (int external : *rhs_pattern) {
      if (external < 0 || external >= m) continue;
      const int internal = static_cast<int>(
          p_->external_to_internal[static_cast<std::size_t>(external)]);
      const double value = rhs[external];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  } else {
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(
          p_->external_to_internal[static_cast<std::size_t>(external)]);
      const double value = rhs[external];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  }
  const double density =
      static_cast<double>(vector.count) / static_cast<double>(m);
  nc.btranCall(vector, density, nullptr);
  std::memcpy(result, vector.array.data(),
              static_cast<size_t>(m) * sizeof(double));
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      result_pattern->push_back(
          static_cast<int>(vector.index[static_cast<std::size_t>(k)]));
    }
  }
}

void HFactorBackend::btran_for_update(
    const double* rhs, double* result, const std::vector<int>* rhs_pattern,
    std::vector<int>* result_pattern) const {
  if (result_pattern != nullptr) result_pattern->clear();
  if (!valid) return;
  HVector& vector = p_->update_vec_ep;
  vector.clear();
  vector.packFlag = true;
  if (rhs_pattern != nullptr) {
    for (int external : *rhs_pattern) {
      if (external < 0 || external >= m) continue;
      const double value = rhs[external];
      const HighsInt internal =
          p_->external_to_internal[static_cast<std::size_t>(external)];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  } else {
    for (int external = 0; external < m; ++external) {
      const double value = rhs[external];
      const HighsInt internal =
          p_->external_to_internal[static_cast<std::size_t>(external)];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  }
  // Pass the true RHS density so HFactor selects its hyper-sparse BTRAN
  // strategy.  CHUZR's BTRAN RHS is a unit vector (count==1) — the canonical
  // hyper-sparse case — where this is a large win; result is identical.  Small
  // bases force density 1.0 (dense) for numerical steadiness (size gate).
  const double density =
      m >= hyper_sparse_min_rows()
          ? static_cast<double>(vector.count) / static_cast<double>(m)
          : 1.0;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(vector, density, nullptr);
  std::memcpy(result, vector.array.data(), static_cast<size_t>(m) * sizeof(double));
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      result_pattern->push_back(
          static_cast<int>(vector.index[static_cast<std::size_t>(k)]));
    }
  }
  p_->ep_capture_serial = p_->factor_serial;
  p_->ep_capture_valid = true;
}

bool HFactorBackend::ftran_indexed(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<int>& result_index, std::vector<double>& result_value,
    std::vector<int>& result_lookup, bool capture_update) const {
  result_index.clear();
  result_value.clear();
  result_lookup.clear();
  if (!valid || rhs_index.size() != rhs_value.size()) return false;
  HVector& vector = capture_update ? p_->update_vec_aq : p_->solve_vec_ftran;
  vector.clear();
  vector.packFlag = capture_update;
  for (std::size_t k = 0; k < rhs_index.size(); ++k) {
    const int row = rhs_index[k];
    const double entry = rhs_value[k];
    if (row < 0 || row >= m || entry == 0.0) continue;
    vector.array[static_cast<std::size_t>(row)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = row;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(vector,
               static_cast<double>(vector.count) / std::max(1, m), nullptr);
  if (vector.count < 0) {
    if (capture_update) p_->aq_capture_valid = false;
    return false;
  }
  result_index.reserve(
      static_cast<std::size_t>(std::max<HighsInt>(0, vector.count)));
  result_value.reserve(result_index.capacity());
  // result_lookup stays empty: IndexedVector::at() builds it lazily, and the
  // hot pivot loops only iterate the packed support. Finiteness is checked
  // here so callers do not need a second pass over the exported values.
  bool all_finite = true;
  for (HighsInt k = 0; k < vector.count; ++k) {
    const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
    const double entry = vector.array[static_cast<std::size_t>(internal)];
    if (entry == 0.0) continue;
    all_finite = all_finite && std::isfinite(entry);
    result_index.push_back(static_cast<int>(
        p_->internal_to_external[static_cast<std::size_t>(internal)]));
    result_value.push_back(entry);
  }
  if (!all_finite) {
    if (capture_update) p_->aq_capture_valid = false;
    return false;
  }
  if (capture_update) {
    p_->aq_capture_serial = p_->factor_serial;
    p_->aq_capture_valid = true;
  }
  return true;
}

bool HFactorBackend::btran_indexed(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<int>& result_index, std::vector<double>& result_value,
    std::vector<int>& result_lookup, bool capture_update) const {
  result_index.clear();
  result_value.clear();
  result_lookup.clear();
  if (!valid || rhs_index.size() != rhs_value.size()) return false;
  HVector& vector = capture_update ? p_->update_vec_ep : p_->solve_vec_btran;
  vector.clear();
  vector.packFlag = capture_update;
  for (std::size_t k = 0; k < rhs_index.size(); ++k) {
    const int external = rhs_index[k];
    const double entry = rhs_value[k];
    if (external < 0 || external >= m || entry == 0.0) continue;
    const HighsInt internal =
        p_->external_to_internal[static_cast<std::size_t>(external)];
    vector.array[static_cast<std::size_t>(internal)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = internal;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(vector,
               static_cast<double>(vector.count) / std::max(1, m), nullptr);
  if (vector.count < 0) {
    if (capture_update) p_->ep_capture_valid = false;
    return false;
  }
  result_index.reserve(
      static_cast<std::size_t>(std::max<HighsInt>(0, vector.count)));
  result_value.reserve(result_index.capacity());
  // result_lookup stays empty: IndexedVector::at() builds it lazily, and the
  // hot pivot loops only iterate the packed support. Finiteness is checked
  // here so callers do not need a second pass over the exported values.
  bool all_finite = true;
  for (HighsInt k = 0; k < vector.count; ++k) {
    const HighsInt row = vector.index[static_cast<std::size_t>(k)];
    const double entry = vector.array[static_cast<std::size_t>(row)];
    if (entry == 0.0) continue;
    all_finite = all_finite && std::isfinite(entry);
    result_index.push_back(static_cast<int>(row));
    result_value.push_back(entry);
  }
  if (!all_finite) {
    if (capture_update) p_->ep_capture_valid = false;
    return false;
  }
  if (capture_update) {
    p_->ep_capture_serial = p_->factor_serial;
    p_->ep_capture_valid = true;
  }
  return true;
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
  if (a_q_after_ftran == nullptr || btran_e_p == nullptr) return false;
  return update_captured(pivot_row, entering_col);
}

bool HFactorBackend::update_captured(int pivot_row, int entering_col) {
  if (!valid) return false;
  if (pivot_row < 0 || pivot_row >= m) return false;
  if (entering_col < 0 || entering_col >= p_->num_col) return false;
  if (!p_->aq_capture_valid ||
      p_->aq_capture_serial != p_->factor_serial ||
      !p_->ep_capture_valid ||
      p_->ep_capture_serial != p_->factor_serial) {
    return false;
  }

  const HighsInt internal_pivot =
      p_->external_to_internal[static_cast<std::size_t>(pivot_row)];
  if (internal_pivot < 0 || internal_pivot >= m) return false;

  HVector& aq = p_->update_vec_aq;
  HVector& ep = p_->update_vec_ep;
  const double pivot = aq.array[static_cast<std::size_t>(internal_pivot)];
  if (pivot == 0.0 || !std::isfinite(1.0 / pivot)) return false;

  p_->basic_index[static_cast<std::size_t>(internal_pivot)] =
      static_cast<HighsInt>(entering_col);
  HighsInt update_row = internal_pivot;
  HighsInt hint = 0;
  p_->f.update(&aq, &ep, &update_row, &hint);

  ++n_updates;
  ++p_->factor_serial;
  p_->aq_capture_valid = false;
  p_->ep_capture_valid = false;
  refactor_hint_ = static_cast<int>(hint);
  return true;
}

void HFactorBackend::reset_update_tracking() noexcept {
  n_updates = 0;
  refactor_hint_ = 0;
}

}  // namespace mipsolvers::engine

// ═══════════════════════════════════════════════════════════════════════════
// HFactorBackend — implementation
//
// Wraps the vendored HiGHS HFactor (see
// src/engine/kernel/linear_algebra/highs_factor/) in an API that mirrors
// `mipsolvers::engine::SparseLUFactor`.
// ═══════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <type_traits>

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

// HiGHS-style running mean of the SOLVE RESULT density (HConst.h
// kRunningAverageMultiplier = 0.05).  HFactor selects its dense skip-scan solve
// when expected_density exceeds the kHyper*{L,U} thresholds (0.10-0.15).  The
// bridge previously fed the RHS density (~1/m for a unit CHUZR/PRICE RHS),
// which forced solveHyper even when the RESULT was dense (d2q06c row_ep/pivotal
// row ~40%).  Feeding the running result density restores the HiGHS kernel
// choice; the solve result is unchanged, only the internal strategy differs.
inline void update_solve_density_mean(double& mean, HighsInt result_count,
                                      int num_row) {
  const double local =
      static_cast<double>(std::max<HighsInt>(0, result_count)) /
      static_cast<double>(std::max(1, num_row));
  mean = 0.95 * mean + 0.05 * local;
}
}  // namespace

struct HFactorBackend::Impl {
  HFactor f;

  // Fallback CSC index conversion for storage widths other than the native
  // 32-bit and HighsInt paths. Values are always shared with the source.
  std::vector<HighsInt> a_start;
  std::vector<HighsInt> a_index;

  // Identity of the compressed Eigen buffers currently mirrored above.
  // StandardFormLP matrices are immutable while a factor is live, so pointer
  // identity is enough to avoid re-copying all nnz at every INVERT.
  const void* source_outer = nullptr;
  const void* source_inner = nullptr;
  const double* source_value = nullptr;
  std::size_t source_index_size = 0;
  Eigen::Index source_rows = 0;
  Eigen::Index source_cols = 0;
  Eigen::Index source_nnz = 0;
  std::uint64_t matrix_copies = 0;
  const HighsInt* bound_start = nullptr;
  const HighsInt* bound_index = nullptr;
  const std::int32_t* bound_start32 = nullptr;
  const std::int32_t* bound_index32 = nullptr;
  const double* bound_value = nullptr;

  template <typename SparseMatrix>
  void bind_compressed_matrix(const SparseMatrix& A) {
    using StorageIndex = typename SparseMatrix::StorageIndex;
    const bool unchanged =
        A.isCompressed() && source_outer == A.outerIndexPtr() &&
        source_inner == A.innerIndexPtr() && source_value == A.valuePtr() &&
        source_index_size == sizeof(StorageIndex) &&
        source_rows == A.rows() && source_cols == A.cols() &&
        source_nnz == A.nonZeros();
    if (unchanged) return;

    const HighsInt num_col = static_cast<HighsInt>(A.cols());
    const HighsInt nnz = static_cast<HighsInt>(A.nonZeros());
    if constexpr (std::is_same_v<StorageIndex, HighsInt>) {
      a_start.clear();
      a_index.clear();
      bound_start = A.outerIndexPtr();
      bound_index = A.innerIndexPtr();
      bound_start32 = nullptr;
      bound_index32 = nullptr;
      bound_value = A.valuePtr();
    } else if constexpr (std::is_same_v<StorageIndex, std::int32_t>) {
      a_start.clear();
      a_index.clear();
      bound_start = nullptr;
      bound_index = nullptr;
      bound_start32 = A.outerIndexPtr();
      bound_index32 = A.innerIndexPtr();
      bound_value = A.valuePtr();
    } else {
      a_start.resize(static_cast<std::size_t>(num_col) + 1);
      for (HighsInt j = 0; j <= num_col; ++j) {
        a_start[static_cast<std::size_t>(j)] =
            static_cast<HighsInt>(A.outerIndexPtr()[j]);
      }
      a_index.resize(static_cast<std::size_t>(nnz));
      for (HighsInt k = 0; k < nnz; ++k) {
        a_index[static_cast<std::size_t>(k)] =
            static_cast<HighsInt>(A.innerIndexPtr()[k]);
      }
      bound_start = a_start.data();
      bound_index = a_index.data();
      bound_start32 = nullptr;
      bound_index32 = nullptr;
      bound_value = A.valuePtr();
      ++matrix_copies;
    }

    source_outer = A.outerIndexPtr();
    source_inner = A.innerIndexPtr();
    source_value = A.valuePtr();
    source_index_size = sizeof(StorageIndex);
    source_rows = A.rows();
    source_cols = A.cols();
    source_nnz = A.nonZeros();
  }

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
  // HiGHS-style running means of each solve's RESULT density, consulted only
  // for m >= hyper_sparse_min_rows().  Start at 0 (cold: hyper-sparse) and warm
  // toward the observed result density so dense-result bases (d2q06c) take the
  // skip-scan kernel.  Persist across rebuilds within a solve like HiGHS.
  mutable double density_mean_ftran = 0.0;
  mutable double density_mean_btran = 0.0;
  mutable double density_mean_aq = 0.0;
  mutable double density_mean_ep = 0.0;
  mutable bool profile_indexed_solves = false;
  mutable double profiled_indexed_solve_time_sec = 0.0;
  mutable double profiled_indexed_export_time_sec = 0.0;
  // BTRAN-only export subset (row-space result, NO internal<->external convert,
  // pure pack). FTRAN-export-with-conversion = export_time - export_btran_time.
  mutable double profiled_indexed_export_btran_time_sec = 0.0;
  mutable double profiled_indexed_solve_synthetic_tick = 0.0;
  mutable std::uint64_t profiled_indexed_solve_count = 0;
  mutable std::uint64_t aq_capture_serial = 0;
  mutable std::uint64_t ep_capture_serial = 0;
  mutable std::uint64_t ftran_workspace_serial = 0;
  mutable bool aq_capture_valid = false;
  mutable bool ep_capture_valid = false;
  mutable bool ftran_workspace_valid = false;
  mutable std::uint64_t dense_solves = 0;
  mutable std::uint64_t indexed_solves = 0;
  std::uint64_t factor_serial = 0;
  // Caller basis position (external) -> HFactor internal position, and back.
  HighsInt in_pos(int external) const {
    return external_to_internal[static_cast<std::size_t>(external)];
  }
  HighsInt out_pos(HighsInt internal) const {
    return internal_to_external[static_cast<std::size_t>(internal)];
  }
};

HFactorBackend::HFactorBackend() : p_(std::make_unique<Impl>()) {}
HFactorBackend::~HFactorBackend() = default;

HFactorBackend::HFactorBackend(HFactorBackend&&) noexcept = default;
HFactorBackend& HFactorBackend::operator=(HFactorBackend&&) noexcept = default;

std::uint64_t HFactorBackend::matrix_copy_count() const noexcept {
  return p_->matrix_copies;
}

std::uint64_t HFactorBackend::dense_solve_count() const noexcept {
  return p_->dense_solves;
}

std::uint64_t HFactorBackend::indexed_solve_count() const noexcept {
  return p_->indexed_solves;
}

void HFactorBackend::set_indexed_solve_profiling(bool enabled) noexcept {
  p_->profile_indexed_solves = enabled;
  p_->profiled_indexed_solve_time_sec = 0.0;
  p_->profiled_indexed_export_time_sec = 0.0;
  p_->profiled_indexed_export_btran_time_sec = 0.0;
  p_->profiled_indexed_solve_synthetic_tick = 0.0;
  p_->profiled_indexed_solve_count = 0;
}

double HFactorBackend::profiled_indexed_solve_time_sec() const noexcept {
  return p_->profiled_indexed_solve_time_sec;
}

double HFactorBackend::profiled_indexed_export_time_sec() const noexcept {
  return p_->profiled_indexed_export_time_sec;
}

double HFactorBackend::profiled_indexed_export_btran_time_sec() const noexcept {
  return p_->profiled_indexed_export_btran_time_sec;
}

double HFactorBackend::profiled_indexed_solve_synthetic_tick() const noexcept {
  return p_->profiled_indexed_solve_synthetic_tick;
}

std::uint64_t HFactorBackend::profiled_indexed_solve_count() const noexcept {
  return p_->profiled_indexed_solve_count;
}

double HFactorBackend::build_synthetic_tick() const noexcept {
  return p_->f.build_synthetic_tick;
}

bool HFactorBackend::strengthen_pivot_threshold() noexcept {
  if (pivot_threshold_ >= kMaxPivotThreshold) return false;
  pivot_threshold_ = kMaxPivotThreshold;
  return true;
}

// ────────────────────────────────────────────────────────────────────────────
// factorize
// ────────────────────────────────────────────────────────────────────────────
template <typename SparseMatrix>
bool HFactorBackend::factorize_impl(const SparseMatrix& A,
                                    const int* basic_index,
                                    int n_basic) {
  valid = false;
  n_updates = 0;
  rank_deficiency = 0;
  refactor_hint_ = 0;
  ++p_->factor_serial;
  p_->aq_capture_valid = false;
  p_->ep_capture_valid = false;
  p_->ftran_workspace_valid = false;

  if (n_basic <= 0 || basic_index == nullptr) return false;
  if (A.rows() <= 0 || A.cols() <= 0) return false;

  // HFactor is built with 64-bit HighsInt while Eigen uses 32-bit storage
  // indices. Convert once per immutable matrix, then reuse the converted CSC
  // across basis rebuilds. The slow uncompressed case gets a local compressed
  // copy and intentionally cannot hit the pointer-identity cache next time.
  SparseMatrix compressed;
  const SparseMatrix* matrix = &A;
  if (!A.isCompressed()) {
    compressed = A;
    compressed.makeCompressed();
    matrix = &compressed;
  }
  const HighsInt num_row = static_cast<HighsInt>(matrix->rows());
  const HighsInt num_col = static_cast<HighsInt>(matrix->cols());
  p_->bind_compressed_matrix(*matrix);

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
  const double pivot_threshold = pivot_threshold_;
  // Pivot tolerance (min absolute pivot for rank determination) is
  // env-overridable for diagnosis: the default 1e-10 declares mildly
  // ill-conditioned SCUC bases rank-deficient where UMFPACK still succeeds.
  double pivot_tol = kDefaultPivotTolerance;
  if (const char* e = std::getenv("MIPSOLVERS_HFACTOR_PIVOT_TOL")) {
    const double v = std::atof(e);
    if (v >= 0.0 && v <= 1.0) pivot_tol = v;
  }
  if (p_->bound_start32 != nullptr) {
    p_->f.setupGeneral32(
        num_col, num_row, static_cast<HighsInt>(n_basic), p_->bound_start32,
        p_->bound_index32, p_->bound_value, p_->basic_index.data(),
        pivot_threshold, pivot_tol, kHighsDebugLevelMin, nullptr, true,
        update_method);
  } else {
    p_->f.setupGeneral(
        num_col, num_row, static_cast<HighsInt>(n_basic), p_->bound_start,
        p_->bound_index, p_->bound_value, p_->basic_index.data(),
        pivot_threshold, pivot_tol, kHighsDebugLevelMin, nullptr, true,
        update_method);
  }

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

template <typename SparseMatrix>
bool HFactorBackend::factorize_with_logicals_impl(
    const SparseMatrix& A, const int* basic_index, int n_basic,
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
  if (factorize_impl(A, basic_index, n_basic)) {
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

  p_->bind_compressed_matrix(A);
  p_->basic_index.resize(static_cast<std::size_t>(n_basic));
  for (int i = 0; i < n_basic; ++i)
    p_->basic_index[static_cast<std::size_t>(i)] =
        static_cast<HighsInt>(hfactor_basis[static_cast<std::size_t>(i)]);

  p_->num_row = h_num_row;
  p_->num_col = h_num_col;
  const HighsInt update_method = kUpdateMethodFt;
  const double pivot_threshold = pivot_threshold_;
  if (p_->bound_start32 != nullptr) {
    p_->f.setupGeneral32(
        h_num_col, h_num_row, static_cast<HighsInt>(n_basic),
        p_->bound_start32, p_->bound_index32, p_->bound_value,
        p_->basic_index.data(), pivot_threshold,
        kDefaultPivotTolerance, kHighsDebugLevelMin, nullptr, true,
        update_method);
  } else {
    p_->f.setupGeneral(
        h_num_col, h_num_row, static_cast<HighsInt>(n_basic), p_->bound_start,
        p_->bound_index, p_->bound_value, p_->basic_index.data(),
        pivot_threshold, kDefaultPivotTolerance, kHighsDebugLevelMin,
        nullptr, true, update_method);
  }
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
  if (!factorize_impl(A, repaired_basis.data(), n_basic)) return false;
  rank_deficiency = repair_count;
  no_pivot_rows_ = repair_rows;
  no_pivot_vars_ = repair_vars;
  return true;
}

bool HFactorBackend::factorize(const Eigen::SparseMatrix<double>& A,
                               const int* basic_index, int n_basic) {
  return factorize_impl(A, basic_index, n_basic);
}

bool HFactorBackend::factorize(const HFactorSparseMatrix64& A,
                               const int* basic_index, int n_basic) {
  return factorize_impl(A, basic_index, n_basic);
}

bool HFactorBackend::factorize(const StandardColumnMatrix& A,
                               const int* basic_index, int n_basic) {
  if (const auto* narrow = A.narrow_matrix()) {
    return factorize_impl(*narrow, basic_index, n_basic);
  }
  return factorize_impl(*A.wide_matrix(), basic_index, n_basic);
}

bool HFactorBackend::factorize_with_logicals(
    const Eigen::SparseMatrix<double>& A, const int* basic_index, int n_basic,
    const std::vector<int>& logical_col_by_row,
    std::vector<int>& repaired_basis) {
  return factorize_with_logicals_impl(A, basic_index, n_basic,
                                      logical_col_by_row, repaired_basis);
}

bool HFactorBackend::factorize_with_logicals(
    const HFactorSparseMatrix64& A, const int* basic_index, int n_basic,
    const std::vector<int>& logical_col_by_row,
    std::vector<int>& repaired_basis) {
  return factorize_with_logicals_impl(A, basic_index, n_basic,
                                      logical_col_by_row, repaired_basis);
}

bool HFactorBackend::factorize_with_logicals(
    const StandardColumnMatrix& A, const int* basic_index, int n_basic,
    const std::vector<int>& logical_col_by_row,
    std::vector<int>& repaired_basis) {
  if (const auto* narrow = A.narrow_matrix()) {
    return factorize_with_logicals_impl(*narrow, basic_index, n_basic,
                                        logical_col_by_row, repaired_basis);
  }
  return factorize_with_logicals_impl(*A.wide_matrix(), basic_index, n_basic,
                                      logical_col_by_row, repaired_basis);
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
  ++p_->dense_solves;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  if (m < hyper_sparse_min_rows()) {
    // Small system: exact dense path (byte-identical to pre-hyper-sparse).
    std::vector<double>& buf = p_->solve_buf;
    std::memcpy(buf.data(), rhs, static_cast<size_t>(m) * sizeof(double));
    nc.ftranCall(buf, /*factor_timer_clock_pointer*/ nullptr);
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(p_->in_pos(external));
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
  ++p_->ftran_workspace_serial;
  p_->ftran_workspace_valid = false;
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
  nc.ftranCall(vector, p_->density_mean_ftran, nullptr);
  update_solve_density_mean(p_->density_mean_ftran, vector.count, m);
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(p_->in_pos(external));
    result[external] = vector.array[static_cast<std::size_t>(internal)];
  }
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
      result_pattern->push_back(static_cast<int>(p_->out_pos(internal)));
    }
  }
}

void HFactorBackend::ftran_for_update(
    const double* rhs, double* result, const std::vector<int>* rhs_pattern,
    std::vector<int>* result_pattern) const {
  if (result_pattern != nullptr) result_pattern->clear();
  if (!valid) return;
  ++p_->dense_solves;
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
  // For m >= hyper_sparse_min_rows() feed the running result density so a dense
  // pivotal column (col_aq) takes the skip-scan kernel; small bases force
  // density 1.0 (dense) for numerical steadiness (size gate).  The solve result
  // is identical to either strategy.
  const double density =
      m >= hyper_sparse_min_rows() ? p_->density_mean_aq : 1.0;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.ftranCall(vector, density, nullptr);
  if (m >= hyper_sparse_min_rows())
    update_solve_density_mean(p_->density_mean_aq, vector.count, m);
  for (int external = 0; external < m; ++external) {
    const int internal = static_cast<int>(p_->in_pos(external));
    result[external] = vector.array[static_cast<std::size_t>(internal)];
  }
  if (result_pattern != nullptr && vector.count >= 0) {
    result_pattern->reserve(static_cast<std::size_t>(vector.count));
    for (HighsInt k = 0; k < vector.count; ++k) {
      const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
      result_pattern->push_back(static_cast<int>(p_->out_pos(internal)));
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
  ++p_->dense_solves;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  if (m < hyper_sparse_min_rows()) {
    // Small system: exact dense path (byte-identical to pre-hyper-sparse).
    std::vector<double>& buf = p_->solve_buf;
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(p_->in_pos(external));
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
      const int internal = static_cast<int>(p_->in_pos(external));
      const double value = rhs[external];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  } else {
    for (int external = 0; external < m; ++external) {
      const int internal = static_cast<int>(p_->in_pos(external));
      const double value = rhs[external];
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  }
  nc.btranCall(vector, p_->density_mean_btran, nullptr);
  update_solve_density_mean(p_->density_mean_btran, vector.count, m);
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
  ++p_->dense_solves;
  HVector& vector = p_->update_vec_ep;
  vector.clear();
  vector.packFlag = true;
  if (rhs_pattern != nullptr) {
    for (int external : *rhs_pattern) {
      if (external < 0 || external >= m) continue;
      const double value = rhs[external];
      const HighsInt internal = p_->in_pos(external);
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  } else {
    for (int external = 0; external < m; ++external) {
      const double value = rhs[external];
      const HighsInt internal = p_->in_pos(external);
      vector.array[static_cast<std::size_t>(internal)] = value;
      if (value != 0.0) {
        vector.index[static_cast<std::size_t>(vector.count++)] = internal;
      }
    }
  }
  // For m >= hyper_sparse_min_rows() feed the running result density (a dense
  // pivotal row takes the skip-scan kernel); small bases force density 1.0
  // (dense) for numerical steadiness (size gate).  Result is identical either
  // way, and CHUZR's unit BTRAN RHS keeps the running mean low whenever the
  // pivotal rows stay sparse.
  const double density =
      m >= hyper_sparse_min_rows() ? p_->density_mean_ep : 1.0;
  HFactor& nc = const_cast<HFactor&>(p_->f);
  nc.btranCall(vector, density, nullptr);
  if (m >= hyper_sparse_min_rows())
    update_solve_density_mean(p_->density_mean_ep, vector.count, m);
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
  return ftran_indexed_impl(rhs_index, rhs_value, &result_index, &result_value,
                            &result_lookup, capture_update, false);
}

bool HFactorBackend::ftran_indexed_resident(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    ResidentVectorView& result) const {
  result = {};
  if (!ftran_indexed_impl(rhs_index, rhs_value, nullptr, nullptr, nullptr, true,
                          true)) {
    return false;
  }
  result = ResidentVectorView(
      this, p_->factor_serial, &p_->factor_serial, &p_->aq_capture_valid,
      p_->update_vec_aq.index.data(), p_->update_vec_aq.array.data(),
      static_cast<int>(p_->update_vec_aq.count), m,
      sizeof(HighsInt) == sizeof(std::int64_t),
      p_->external_to_internal.data(), p_->internal_to_external.data());
  return result.valid();
}

bool HFactorBackend::ftran_indexed_scratch_resident(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    ResidentVectorView& result) const {
  result = {};
  if (!ftran_indexed_impl(rhs_index, rhs_value, nullptr, nullptr, nullptr, false,
                          true)) {
    return false;
  }
  p_->ftran_workspace_valid = true;
  result = ResidentVectorView(
      this, p_->ftran_workspace_serial, &p_->ftran_workspace_serial,
      &p_->ftran_workspace_valid, p_->solve_vec_ftran.index.data(),
      p_->solve_vec_ftran.array.data(),
      static_cast<int>(p_->solve_vec_ftran.count), m,
      sizeof(HighsInt) == sizeof(std::int64_t), p_->external_to_internal.data(),
      p_->internal_to_external.data());
  return result.valid();
}

bool HFactorBackend::ftran_indexed_impl(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<int>* result_index, std::vector<double>* result_value,
    std::vector<int>* result_lookup, bool capture_update,
    bool resident_only) const {
  if (result_index != nullptr) result_index->clear();
  if (result_value != nullptr) result_value->clear();
  if (result_lookup != nullptr) result_lookup->clear();
  if (!valid || rhs_index.size() != rhs_value.size()) return false;
  ++p_->indexed_solves;
  HVector& vector = capture_update ? p_->update_vec_aq : p_->solve_vec_ftran;
  if (!capture_update) {
    ++p_->ftran_workspace_serial;
    p_->ftran_workspace_valid = false;
  }
  double& density_mean =
      capture_update ? p_->density_mean_aq : p_->density_mean_ftran;
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
  const auto solve_start =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  // max(RHS density, running result density): never downgrades a dense RHS to
  // hyper-sparse (preserves the prior choice + numerical steadiness) and adds
  // the dense skip-scan kernel when the RESULT is dense (d2q06c row_ep/col_aq
  // ~40%) even at m below the size gate.  No size gate here: the indexed hot
  // path never had one, and max() only ever ADDS dense selections.
  const double expected_density = std::max(
      static_cast<double>(vector.count) / std::max(1, m), density_mean);
  nc.ftranCall(vector, expected_density, nullptr);
  const auto solve_end =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  update_solve_density_mean(density_mean, vector.count, m);
  auto record_profile = [&]() {
    if (!p_->profile_indexed_solves) return;
    const auto tp_now = std::chrono::steady_clock::now();
    p_->profiled_indexed_solve_time_sec +=
        std::chrono::duration<double>(tp_now - solve_start).count();
    if (!resident_only) {
      p_->profiled_indexed_export_time_sec +=
          std::chrono::duration<double>(tp_now - solve_end).count();
    }
    p_->profiled_indexed_solve_synthetic_tick += vector.synthetic_tick;
    ++p_->profiled_indexed_solve_count;
  };
  if (vector.count < 0) {
    if (capture_update) p_->aq_capture_valid = false;
    record_profile();
    return false;
  }
  if (!resident_only) {
    result_index->reserve(
        static_cast<std::size_t>(std::max<HighsInt>(0, vector.count)));
    result_value->reserve(result_index->capacity());
  }
  // result_lookup stays empty: IndexedVector::at() builds it lazily, and the
  // hot pivot loops only iterate the packed support. Finiteness is checked
  // here so callers do not need a second pass over the exported values.
  bool all_finite = true;
  const HighsInt solved_count = vector.count;
  HighsInt resident_count = 0;
  for (HighsInt k = 0; k < solved_count; ++k) {
    const HighsInt internal = vector.index[static_cast<std::size_t>(k)];
    const double entry = vector.array[static_cast<std::size_t>(internal)];
    if (entry == 0.0) continue;
    all_finite = all_finite && std::isfinite(entry);
    if (resident_only) {
      vector.index[static_cast<std::size_t>(resident_count++)] = internal;
    } else {
      result_index->push_back(static_cast<int>(p_->out_pos(internal)));
      result_value->push_back(entry);
    }
  }
  if (resident_only) vector.count = resident_count;
  if (!all_finite) {
    if (capture_update) p_->aq_capture_valid = false;
    record_profile();
    return false;
  }
  if (capture_update) {
    p_->aq_capture_serial = p_->factor_serial;
    p_->aq_capture_valid = true;
  }
  record_profile();
  return true;
}

bool HFactorBackend::ftran_indexed_at_captured_pattern(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<double>& result_value) const {
  result_value.clear();
  if (!valid || rhs_index.size() != rhs_value.size() ||
      !p_->aq_capture_valid || p_->aq_capture_serial != p_->factor_serial) {
    return false;
  }
  ++p_->indexed_solves;
  HVector& vector = p_->solve_vec_ftran;
  ++p_->ftran_workspace_serial;
  p_->ftran_workspace_valid = false;
  vector.clear();
  vector.packFlag = false;
  for (std::size_t k = 0; k < rhs_index.size(); ++k) {
    const int row = rhs_index[k];
    const double entry = rhs_value[k];
    if (row < 0 || row >= m || entry == 0.0) continue;
    vector.array[static_cast<std::size_t>(row)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = row;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  const auto solve_start =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  const double expected_density = std::max(
      static_cast<double>(vector.count) / std::max(1, m),
      p_->density_mean_ftran);
  nc.ftranCall(vector, expected_density, nullptr);
  const auto solve_end =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  update_solve_density_mean(p_->density_mean_ftran, vector.count, m);
  auto record_profile = [&]() {
    if (!p_->profile_indexed_solves) return;
    const auto tp_now = std::chrono::steady_clock::now();
    p_->profiled_indexed_solve_time_sec +=
        std::chrono::duration<double>(tp_now - solve_start).count();
    p_->profiled_indexed_export_time_sec +=
        std::chrono::duration<double>(tp_now - solve_end).count();
    p_->profiled_indexed_solve_synthetic_tick += vector.synthetic_tick;
    ++p_->profiled_indexed_solve_count;
  };
  if (vector.count < 0) {
    record_profile();
    return false;
  }
  for (HighsInt k = 0; k < vector.count; ++k) {
    const HighsInt row = vector.index[static_cast<std::size_t>(k)];
    if (!std::isfinite(vector.array[static_cast<std::size_t>(row)])) {
      record_profile();
      return false;
    }
  }

  const HVector& captured = p_->update_vec_aq;
  result_value.reserve(
      static_cast<std::size_t>(std::max<HighsInt>(0, captured.count)));
  for (HighsInt k = 0; k < captured.count; ++k) {
    const HighsInt internal = captured.index[static_cast<std::size_t>(k)];
    if (captured.array[static_cast<std::size_t>(internal)] == 0.0) continue;
    result_value.push_back(vector.array[static_cast<std::size_t>(internal)]);
  }
  record_profile();
  return true;
}

bool HFactorBackend::ftran_resident_ep_at_captured_aq_pattern(
    const ResidentVectorView& rhs,
    std::vector<double>& result_value) const {
  result_value.clear();
  if (!rhs.valid() || rhs.owner_ != this || !valid ||
      !p_->aq_capture_valid || p_->aq_capture_serial != p_->factor_serial) {
    return false;
  }
  ++p_->indexed_solves;
  HVector& vector = p_->solve_vec_ftran;
  ++p_->ftran_workspace_serial;
  p_->ftran_workspace_valid = false;
  vector.clear();
  vector.packFlag = false;
  // The BTRAN result is already in external row space, which is also FTRAN's
  // RHS space. Preserve its HVector support order and avoid a Native packed
  // round trip (derivation: native_presolve_lp_2026-08-18.md section 8.17).
  const HVector& resident_ep = p_->update_vec_ep;
  for (HighsInt k = 0; k < resident_ep.count; ++k) {
    const HighsInt row = resident_ep.index[static_cast<std::size_t>(k)];
    const double entry = resident_ep.array[static_cast<std::size_t>(row)];
    if (entry == 0.0) continue;
    vector.array[static_cast<std::size_t>(row)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = row;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  const auto solve_start =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  const double expected_density = std::max(
      static_cast<double>(vector.count) / std::max(1, m),
      p_->density_mean_ftran);
  nc.ftranCall(vector, expected_density, nullptr);
  const auto solve_end =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  update_solve_density_mean(p_->density_mean_ftran, vector.count, m);
  auto record_profile = [&]() {
    if (!p_->profile_indexed_solves) return;
    const auto now = std::chrono::steady_clock::now();
    p_->profiled_indexed_solve_time_sec +=
        std::chrono::duration<double>(now - solve_start).count();
    p_->profiled_indexed_export_time_sec +=
        std::chrono::duration<double>(now - solve_end).count();
    p_->profiled_indexed_solve_synthetic_tick += vector.synthetic_tick;
    ++p_->profiled_indexed_solve_count;
  };
  if (vector.count < 0) {
    record_profile();
    return false;
  }
  for (HighsInt k = 0; k < vector.count; ++k) {
    const HighsInt row = vector.index[static_cast<std::size_t>(k)];
    if (!std::isfinite(vector.array[static_cast<std::size_t>(row)])) {
      record_profile();
      return false;
    }
  }

  const HVector& captured_aq = p_->update_vec_aq;
  result_value.reserve(
      static_cast<std::size_t>(std::max<HighsInt>(0, captured_aq.count)));
  for (HighsInt k = 0; k < captured_aq.count; ++k) {
    const HighsInt internal =
        captured_aq.index[static_cast<std::size_t>(k)];
    if (captured_aq.array[static_cast<std::size_t>(internal)] == 0.0) continue;
    result_value.push_back(vector.array[static_cast<std::size_t>(internal)]);
  }
  record_profile();
  return true;
}

bool HFactorBackend::ftran_resident_ep_at_captured_aq_pattern(
    const ResidentVectorView& rhs, ResidentVectorView& result) const {
  result = {};
  if (!rhs.valid() || rhs.owner_ != this || !valid ||
      !p_->aq_capture_valid || p_->aq_capture_serial != p_->factor_serial) {
    return false;
  }
  ++p_->indexed_solves;
  HVector& vector = p_->solve_vec_ftran;
  ++p_->ftran_workspace_serial;
  p_->ftran_workspace_valid = false;
  vector.clear();
  vector.packFlag = false;
  const HVector& resident_ep = p_->update_vec_ep;
  for (HighsInt k = 0; k < resident_ep.count; ++k) {
    const HighsInt row = resident_ep.index[static_cast<std::size_t>(k)];
    const double entry = resident_ep.array[static_cast<std::size_t>(row)];
    if (entry == 0.0) continue;
    vector.array[static_cast<std::size_t>(row)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = row;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  const auto solve_start =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  const double expected_density = std::max(
      static_cast<double>(vector.count) / std::max(1, m),
      p_->density_mean_ftran);
  nc.ftranCall(vector, expected_density, nullptr);
  update_solve_density_mean(p_->density_mean_ftran, vector.count, m);
  auto record_profile = [&]() {
    if (!p_->profile_indexed_solves) return;
    const auto now = std::chrono::steady_clock::now();
    p_->profiled_indexed_solve_time_sec +=
        std::chrono::duration<double>(now - solve_start).count();
    p_->profiled_indexed_solve_synthetic_tick += vector.synthetic_tick;
    ++p_->profiled_indexed_solve_count;
  };
  if (vector.count < 0) {
    record_profile();
    return false;
  }
  for (HighsInt k = 0; k < vector.count; ++k) {
    const HighsInt row = vector.index[static_cast<std::size_t>(k)];
    if (!std::isfinite(vector.array[static_cast<std::size_t>(row)])) {
      record_profile();
      return false;
    }
  }

  const HVector& captured_aq = p_->update_vec_aq;
  p_->ftran_workspace_valid = true;
  result = ResidentVectorView(
      this, p_->ftran_workspace_serial, &p_->ftran_workspace_serial,
      &p_->ftran_workspace_valid, captured_aq.index.data(), vector.array.data(),
      static_cast<int>(captured_aq.count), m,
      sizeof(HighsInt) == sizeof(std::int64_t), p_->external_to_internal.data(),
      p_->internal_to_external.data());
  record_profile();
  return result.valid();
}

bool HFactorBackend::captured_aq_value(int external_row, double& out) const {
  if (!valid || external_row < 0 || external_row >= m ||
      !p_->aq_capture_valid || p_->aq_capture_serial != p_->factor_serial) {
    return false;
  }
  const HighsInt internal = p_->in_pos(external_row);
  if (internal < 0 || internal >= m) return false;
  out = p_->update_vec_aq.array[static_cast<std::size_t>(internal)];
  return true;
}

bool HFactorBackend::btran_indexed(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<int>& result_index, std::vector<double>& result_value,
    std::vector<int>& result_lookup, bool capture_update) const {
  return btran_indexed_impl(rhs_index, rhs_value, &result_index, &result_value,
                            &result_lookup, capture_update, false);
}

bool HFactorBackend::btran_indexed_resident(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    ResidentVectorView& result) const {
  result = {};
  if (!btran_indexed_impl(rhs_index, rhs_value, nullptr, nullptr, nullptr, true,
                          true)) {
    return false;
  }
  result = ResidentVectorView(
      this, p_->factor_serial, &p_->factor_serial, &p_->ep_capture_valid,
      p_->update_vec_ep.index.data(), p_->update_vec_ep.array.data(),
      static_cast<int>(p_->update_vec_ep.count), m,
      sizeof(HighsInt) == sizeof(std::int64_t));
  return result.valid();
}

bool HFactorBackend::btran_indexed_impl(
    const std::vector<int>& rhs_index, const std::vector<double>& rhs_value,
    std::vector<int>* result_index, std::vector<double>* result_value,
    std::vector<int>* result_lookup, bool capture_update,
    bool resident_only) const {
  if (result_index != nullptr) result_index->clear();
  if (result_value != nullptr) result_value->clear();
  if (result_lookup != nullptr) result_lookup->clear();
  if (!valid || rhs_index.size() != rhs_value.size()) return false;
  ++p_->indexed_solves;
  HVector& vector = capture_update ? p_->update_vec_ep : p_->solve_vec_btran;
  double& density_mean =
      capture_update ? p_->density_mean_ep : p_->density_mean_btran;
  vector.clear();
  vector.packFlag = capture_update;
  for (std::size_t k = 0; k < rhs_index.size(); ++k) {
    const int external = rhs_index[k];
    const double entry = rhs_value[k];
    if (external < 0 || external >= m || entry == 0.0) continue;
    const HighsInt internal = p_->in_pos(external);
    vector.array[static_cast<std::size_t>(internal)] = entry;
    vector.index[static_cast<std::size_t>(vector.count++)] = internal;
  }
  HFactor& nc = const_cast<HFactor&>(p_->f);
  const auto solve_start =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  const double expected_density = std::max(
      static_cast<double>(vector.count) / std::max(1, m), density_mean);
  nc.btranCall(vector, expected_density, nullptr);
  const auto solve_end =
      p_->profile_indexed_solves ? std::chrono::steady_clock::now()
                                 : std::chrono::steady_clock::time_point{};
  update_solve_density_mean(density_mean, vector.count, m);
  auto record_profile = [&]() {
    if (!p_->profile_indexed_solves) return;
    const auto tp_now = std::chrono::steady_clock::now();
    p_->profiled_indexed_solve_time_sec +=
        std::chrono::duration<double>(tp_now - solve_start).count();
    if (!resident_only) {
      const double export_sec =
          std::chrono::duration<double>(tp_now - solve_end).count();
      p_->profiled_indexed_export_time_sec += export_sec;
      p_->profiled_indexed_export_btran_time_sec += export_sec;
    }
    p_->profiled_indexed_solve_synthetic_tick += vector.synthetic_tick;
    ++p_->profiled_indexed_solve_count;
  };
  if (vector.count < 0) {
    if (capture_update) p_->ep_capture_valid = false;
    record_profile();
    return false;
  }
  if (!resident_only) {
    result_index->reserve(
        static_cast<std::size_t>(std::max<HighsInt>(0, vector.count)));
    result_value->reserve(result_index->capacity());
  }
  // result_lookup stays empty: IndexedVector::at() builds it lazily, and the
  // hot pivot loops only iterate the packed support. Finiteness is checked
  // here so callers do not need a second pass over the exported values.
  bool all_finite = true;
  const HighsInt solved_count = vector.count;
  HighsInt resident_count = 0;
  for (HighsInt k = 0; k < solved_count; ++k) {
    const HighsInt row = vector.index[static_cast<std::size_t>(k)];
    const double entry = vector.array[static_cast<std::size_t>(row)];
    if (entry == 0.0) continue;
    all_finite = all_finite && std::isfinite(entry);
    if (resident_only) {
      vector.index[static_cast<std::size_t>(resident_count++)] = row;
    } else {
      result_index->push_back(static_cast<int>(row));
      result_value->push_back(entry);
    }
  }
  if (resident_only) vector.count = resident_count;
  if (!all_finite) {
    if (capture_update) p_->ep_capture_valid = false;
    record_profile();
    return false;
  }
  if (capture_update) {
    p_->ep_capture_serial = p_->factor_serial;
    p_->ep_capture_valid = true;
  }
  record_profile();
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

  const HighsInt internal_pivot = p_->in_pos(pivot_row);
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
  p_->ftran_workspace_valid = false;
  refactor_hint_ = static_cast<int>(hint);
  return true;
}

void HFactorBackend::reset_update_tracking() noexcept {
  n_updates = 0;
  refactor_hint_ = 0;
}

}  // namespace mipsolvers::engine

// High-performance Mehrotra Predictor-Corrector IPM for LP
//
// Solves: min c'x s.t. A x <= b, Aeq x = beq, lb <= x <= ub
//
// Bounded-variable formulation: handles bounds directly (no upper-bound slacks).
// Uses banded Cholesky for narrow-banded normal equations, sparse LDLT otherwise.

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>

#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"

#if defined(HACDCPF_HAVE_ACCELERATE) && defined(__APPLE__)
#define MIPSOLVERS_USE_ACCELERATE 1
#include <Accelerate/Accelerate.h>
#else
#define MIPSOLVERS_USE_ACCELERATE 0
#endif

// ARM NEON SIMD for compilers that provide arm_neon.h.
#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__)
#include <arm_neon.h>
#define USE_NEON 1
#else
#define USE_NEON 0
#endif

#if defined(_MSC_VER)
#define MIPSOLVERS_RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
#define MIPSOLVERS_RESTRICT __restrict__
#else
#define MIPSOLVERS_RESTRICT
#endif

// OpenMP for the big embarrassingly-parallel kernel loops (SpMV, element-wise
// builds).  Threshold-guarded at runtime so small problems stay serial.
// Only loops with provably disjoint per-iteration outputs are parallelized.
#if defined(MIPSOLVERS_USE_OPENMP)
#include <omp.h>
#define MIPSOLVERS_OMP_STR_(x) #x
#define MIPSOLVERS_OMP_STR(x) MIPSOLVERS_OMP_STR_(x)
#define MIPSOLVERS_OMP_PARALLEL_IF(cond) \
  _Pragma(MIPSOLVERS_OMP_STR(omp parallel for if(cond)))
#else
#define MIPSOLVERS_OMP_PARALLEL_IF(cond)
#endif
#define MIPSOLVERS_OMP_THRESHOLD 4096  // parallelize only loops larger than this

namespace mipsolvers::engine {

// Apple Accelerate sparse Cholesky cache — persists across IPM solve calls
// to reuse symbolic factorization when the sparsity pattern is unchanged
// (e.g. repeated node LP solves in B&C with same constraint matrix).
struct AccelSparseCache {
#if MIPSOLVERS_USE_ACCELERATE
  SparseOpaqueSymbolicFactorization symbolic{};
  std::vector<long> col_starts;
  int cached_m = 0;
  int cached_nnz = 0;
  bool valid = false;

  ~AccelSparseCache() {
    if (valid) SparseCleanup(symbolic);
  }
#endif
};

namespace {

constexpr double kBig = 1e20;
constexpr double kTau = 0.9995;
constexpr double kMinVal = 1e-14;
constexpr int kBandedThreshold = 128;  // use banded Cholesky when bandwidth <= this
constexpr size_t kDenseScatterThreshold = 4'000'000;  // switch to dense BLAS when scatter > 4M
constexpr size_t kDenseMaxBytes = 256 * 1024 * 1024;  // memory gate for the dense path
constexpr size_t kMaxScatterEntries = 100'000'000;  // scatter map cap (~1.6GB)

// =====================================================================
// SIMD-optimized vector operations (ARM NEON)
// =====================================================================

#if USE_NEON

// y[j] = a[j] * b[j] + c[j]  (fused multiply-add)
inline void simd_fma(const double* MIPSOLVERS_RESTRICT a, const double* MIPSOLVERS_RESTRICT b,
                     const double* MIPSOLVERS_RESTRICT c, double* MIPSOLVERS_RESTRICT y, int n) {
  int j = 0;
  for (; j + 4 <= n; j += 4) {
    float64x2_t a0 = vld1q_f64(a + j);
    float64x2_t a1 = vld1q_f64(a + j + 2);
    float64x2_t b0 = vld1q_f64(b + j);
    float64x2_t b1 = vld1q_f64(b + j + 2);
    float64x2_t c0 = vld1q_f64(c + j);
    float64x2_t c1 = vld1q_f64(c + j + 2);
    float64x2_t r0 = vfmaq_f64(c0, a0, b0);  // c + a*b
    float64x2_t r1 = vfmaq_f64(c1, a1, b1);
    vst1q_f64(y + j, r0);
    vst1q_f64(y + j + 2, r1);
  }
  for (; j < n; ++j) y[j] = a[j] * b[j] + c[j];
}

// y[j] += a * x[j]
inline void simd_axpy(double a, const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y, int n) {
  float64x2_t va = vdupq_n_f64(a);
  int j = 0;
  for (; j + 4 <= n; j += 4) {
    float64x2_t x0 = vld1q_f64(x + j);
    float64x2_t x1 = vld1q_f64(x + j + 2);
    float64x2_t y0 = vld1q_f64(y + j);
    float64x2_t y1 = vld1q_f64(y + j + 2);
    y0 = vfmaq_f64(y0, va, x0);
    y1 = vfmaq_f64(y1, va, x1);
    vst1q_f64(y + j, y0);
    vst1q_f64(y + j + 2, y1);
  }
  for (; j < n; ++j) y[j] += a * x[j];
}

// y[j] = max(y[j] + a * x[j], minval)
inline void simd_axpy_max(double a, const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y,
                           double minval, int n) {
  float64x2_t va = vdupq_n_f64(a);
  float64x2_t vmin = vdupq_n_f64(minval);
  int j = 0;
  for (; j + 4 <= n; j += 4) {
    float64x2_t x0 = vld1q_f64(x + j);
    float64x2_t x1 = vld1q_f64(x + j + 2);
    float64x2_t y0 = vld1q_f64(y + j);
    float64x2_t y1 = vld1q_f64(y + j + 2);
    y0 = vmaxq_f64(vfmaq_f64(y0, va, x0), vmin);
    y1 = vmaxq_f64(vfmaq_f64(y1, va, x1), vmin);
    vst1q_f64(y + j, y0);
    vst1q_f64(y + j + 2, y1);
  }
  for (; j < n; ++j) y[j] = std::max(y[j] + a * x[j], minval);
}

// y[j] = max(a[j] - b[j], minval)
inline void simd_sub_max(const double* MIPSOLVERS_RESTRICT a, const double* MIPSOLVERS_RESTRICT b,
                         double* MIPSOLVERS_RESTRICT y, double minval, int n) {
  float64x2_t vmin = vdupq_n_f64(minval);
  int j = 0;
  for (; j + 4 <= n; j += 4) {
    float64x2_t a0 = vld1q_f64(a + j);
    float64x2_t a1 = vld1q_f64(a + j + 2);
    float64x2_t b0 = vld1q_f64(b + j);
    float64x2_t b1 = vld1q_f64(b + j + 2);
    float64x2_t r0 = vmaxq_f64(vsubq_f64(a0, b0), vmin);
    float64x2_t r1 = vmaxq_f64(vsubq_f64(a1, b1), vmin);
    vst1q_f64(y + j, r0);
    vst1q_f64(y + j + 2, r1);
  }
  for (; j < n; ++j) y[j] = std::max(a[j] - b[j], minval);
}

// y[j] = max(a[j] - b[j], minval)
inline void simd_rsub_max(const double* MIPSOLVERS_RESTRICT a, const double* MIPSOLVERS_RESTRICT b,
                          double* MIPSOLVERS_RESTRICT y, double minval, int n) {
  float64x2_t vmin = vdupq_n_f64(minval);
  int j = 0;
  for (; j + 4 <= n; j += 4) {
    float64x2_t a0 = vld1q_f64(a + j);
    float64x2_t a1 = vld1q_f64(a + j + 2);
    float64x2_t b0 = vld1q_f64(b + j);
    float64x2_t b1 = vld1q_f64(b + j + 2);
    float64x2_t r0 = vmaxq_f64(vsubq_f64(b0, a0), vmin);
    float64x2_t r1 = vmaxq_f64(vsubq_f64(b1, a1), vmin);
    vst1q_f64(y + j, r0);
    vst1q_f64(y + j + 2, r1);
  }
  for (; j < n; ++j) y[j] = std::max(b[j] - a[j], minval);
}

#else
// Scalar fallbacks
inline void simd_fma(const double* a, const double* b, const double* c, double* y, int n) {
  for (int j = 0; j < n; ++j) y[j] = a[j] * b[j] + c[j];
}
inline void simd_axpy(double a, const double* x, double* y, int n) {
  for (int j = 0; j < n; ++j) y[j] += a * x[j];
}
inline void simd_axpy_max(double a, const double* x, double* y, double minval, int n) {
  for (int j = 0; j < n; ++j) y[j] = std::max(y[j] + a * x[j], minval);
}
inline void simd_sub_max(const double* a, const double* b, double* y, double minval, int n) {
  for (int j = 0; j < n; ++j) y[j] = std::max(a[j] - b[j], minval);
}
inline void simd_rsub_max(const double* a, const double* b, double* y, double minval, int n) {
  for (int j = 0; j < n; ++j) y[j] = std::max(b[j] - a[j], minval);
}
#endif

// Banded scatter entry: offset into banded storage + precomputed A product
struct BandScatterEntry { int offset; double a_prod; };

// In-place banded Cholesky factorization (lower triangle, column-major band storage)
// band layout: band[k * m + col] = L[col+k, col] for k = 0..bw
// Returns false if not positive definite.
inline bool banded_chol_factor(double* MIPSOLVERS_RESTRICT band, int m, int bw) {
  for (int j = 0; j < m; ++j) {
    // Compute L[j,j]
    double djj = band[j];  // band[0*m + j]
    const int s_start = std::max(0, j - bw);
    for (int s = s_start; s < j; ++s) {
      double v = band[(j - s) * m + s];
      djj -= v * v;
    }
    if (djj <= 0.0) return false;
    djj = std::sqrt(djj);
    band[j] = djj;
    double inv_djj = 1.0 / djj;

    // Compute L[j+k, j] for k = 1..min(bw, m-1-j)
    int kmax = std::min(bw, m - 1 - j);
    for (int k = 1; k <= kmax; ++k) {
      double& Lij = band[k * m + j];  // L[j+k, j]
      // Inner loop: s from max(0, j+k-bw) to j-1
      // This is the tightened bound that eliminates the if(d1<=bw) check
      const int s_lo = std::max(0, j + k - bw);
      for (int s = s_lo; s < j; ++s) {
        Lij -= band[(j + k - s) * m + s] * band[(j - s) * m + s];
      }
      Lij *= inv_djj;
    }
  }
  return true;
}

// Solve L * L^T * x = b in-place (b is overwritten with x)
inline void banded_chol_solve(const double* MIPSOLVERS_RESTRICT band, int m, int bw, double* MIPSOLVERS_RESTRICT b) {
  // Forward: L y = b
  for (int j = 0; j < m; ++j) {
    for (int k = std::max(0, j - bw); k < j; ++k) {
      b[j] -= band[(j - k) * m + k] * b[k];
    }
    b[j] /= band[j];
  }
  // Backward: L^T x = y
  for (int j = m - 1; j >= 0; --j) {
    int kmax = std::min(bw, m - 1 - j);
    for (int k = 1; k <= kmax; ++k) {
      b[j] -= band[k * m + j] * b[j + k];
    }
    b[j] /= band[j];
  }
}

// y = N * x with N symmetric banded, stored as band[(row-col)*m + col]
// (lower triangle, row >= col, row-col <= bw).  Used by iterative
// refinement against the pristine (unfactorized) band storage.
inline void banded_sym_matvec(const double* MIPSOLVERS_RESTRICT band, int m, int bw,
                              const double* MIPSOLVERS_RESTRICT x,
                              double* MIPSOLVERS_RESTRICT y) {
  std::fill(y, y + m, 0.0);
  for (int j = 0; j < m; ++j) {
    const double xj = x[j];
    y[j] += band[j] * xj;  // diagonal
    const int kmax = std::min(bw, m - 1 - j);
    for (int k = 1; k <= kmax; ++k) {
      const double v = band[k * m + j];  // N[j+k, j] == N[j, j+k]
      y[j + k] += v * xj;
      y[j] += v * x[j + k];
    }
  }
}

// =====================================================================
// Ruiz equilibration
// =====================================================================
// Alternating row/column infinity-norm scaling of the constraint system
// [A; Aeq] (inequality rows first, then equality rows).  Returns scaled
// copies plus cumulative diagonal scalings dr (rows) and dc (columns).
// The scaled LP  min (Dc c)'x̂  s.t.  (Dr A Dc) x̂ + ŝ = Dr b, ŝ >= 0
// has the same feasible set as the original under
//   x = Dc·x̂,  y_orig = Dr·ŷ,  z_orig = ẑ / Dc (box multipliers).
// Slack coefficients stay 1, so no slack handling anywhere changes.
struct RuizScaling {
  Eigen::SparseMatrix<double> A;    // Dr·A·Dc
  Eigen::SparseMatrix<double> Aeq;  // Dr·Aeq·Dc
  std::vector<double> dr;           // row scales, size mi + me
  std::vector<double> dc;           // column scales, size n
  bool active = false;
};

RuizScaling ruiz_equilibrate(const Eigen::SparseMatrix<double>& A,
                             const Eigen::SparseMatrix<double>& Aeq,
                             int n, int rounds) {
  RuizScaling rs;
  const int mi = static_cast<int>(A.rows());
  const int me = static_cast<int>(Aeq.rows());
  const int na = static_cast<int>(A.outerSize());
  const int ne = static_cast<int>(Aeq.outerSize());
  if (rounds <= 0 || n <= 0 || (mi + me) <= 0) return rs;

  rs.active = true;
  rs.A = A;
  rs.Aeq = Aeq;
  rs.dr.assign(static_cast<size_t>(mi + me), 1.0);
  rs.dc.assign(static_cast<size_t>(n), 1.0);

  std::vector<double> row_max(static_cast<size_t>(mi + me));
  std::vector<double> row_factor(static_cast<size_t>(mi + me));
  std::vector<double> col_max(static_cast<size_t>(n));
  for (int round = 0; round < rounds; ++round) {
    // Row equilibration: dr_i *= 1/sqrt(max_j |a_ij|)
    std::fill(row_max.begin(), row_max.end(), 0.0);
    for (int j = 0; j < na; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.A, j); it; ++it)
        row_max[it.row()] = std::max(row_max[it.row()], std::abs(it.value()));
    for (int j = 0; j < ne; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.Aeq, j); it; ++it)
        row_max[mi + it.row()] = std::max(row_max[mi + it.row()], std::abs(it.value()));
    // Accumulate once per row (per-entry accumulation would square dr).
    for (int i = 0; i < mi + me; ++i) {
      row_factor[i] = (row_max[i] > 0.0) ? 1.0 / std::sqrt(row_max[i]) : 1.0;
      rs.dr[i] *= row_factor[i];
    }
    for (int j = 0; j < na; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.A, j); it; ++it)
        it.valueRef() *= row_factor[it.row()];
    for (int j = 0; j < ne; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.Aeq, j); it; ++it)
        it.valueRef() *= row_factor[mi + it.row()];
    // Column equilibration: dc_j *= 1/sqrt(max_i |a_ij|)
    std::fill(col_max.begin(), col_max.end(), 0.0);
    for (int j = 0; j < na; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.A, j); it; ++it)
        col_max[j] = std::max(col_max[j], std::abs(it.value()));
    for (int j = 0; j < ne; ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(rs.Aeq, j); it; ++it)
        col_max[j] = std::max(col_max[j], std::abs(it.value()));
    for (int j = 0; j < n; ++j) {
      const double cmax = col_max[j];
      if (cmax <= 0.0) continue;
      const double f = 1.0 / std::sqrt(cmax);
      if (j < na)
        for (Eigen::SparseMatrix<double>::InnerIterator it(rs.A, j); it; ++it)
          it.valueRef() *= f;
      if (j < ne)
        for (Eigen::SparseMatrix<double>::InnerIterator it(rs.Aeq, j); it; ++it)
          it.valueRef() *= f;
      rs.dc[j] *= f;
    }
  }
  return rs;
}

}  // namespace

NativeIPMLPAdapter::NativeIPMLPAdapter(IPMLPOptions opt)
    : opt_(std::move(opt)), accel_cache_(std::make_unique<AccelSparseCache>()) {}
NativeIPMLPAdapter::~NativeIPMLPAdapter() = default;
std::string NativeIPMLPAdapter::name() const { return "NativeIPMLP"; }
bool NativeIPMLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeIPMLPAdapter::solve_lp(const LPModel& prob) const {
  static const Eigen::VectorXd empty;
  return solve_lp(prob, empty);
}

SolveResult NativeIPMLPAdapter::solve_lp(const LPModel& prob, const Eigen::VectorXd& x0) const {
  // Scaling fallback: solve with the configured Ruiz rounds first; if it
  // fails to converge, retry without scaling.  Equilibration helps most
  // problems (e.g. NETLIB afiro) but stalls some degenerate ones
  // (e.g. stocfor1) — retrying unscaled is the standard robustness answer.
  SolveResult res = solve_lp_impl(prob, x0, opt_.ruiz_rounds);
  if (!res.stats.success && opt_.ruiz_rounds > 0) {
    SolveResult raw = solve_lp_impl(prob, x0, 0);
    if (raw.stats.success) res = std::move(raw);
  }
  return res;
}

SolveResult NativeIPMLPAdapter::solve_lp_impl(const LPModel& prob, const Eigen::VectorXd& x0, int ruiz_rounds) const {
  const bool has_warm_start = (x0.size() == prob.c.size());
  const bool ipm_verbose_env = (std::getenv("MIPSOLVERS_IPM_VERBOSE") != nullptr);
  SolveResult out;
  out.stats.solver_name = name();
  const auto t0 = std::chrono::steady_clock::now();

  const int n_orig = static_cast<int>(prob.c.size());
  if (n_orig == 0) {
    out.stats.success = true;
    out.stats.status = "Empty";
    out.x = Eigen::VectorXd();
    return out;
  }

  const int sense_sign = (prob.sense == Sense::Maximize) ? -1 : 1;
  const int mi = static_cast<int>(prob.A.rows());
  const int me = static_cast<int>(prob.Aeq.rows());
  const int m = mi + me;
  const int nn = n_orig + mi;  // original vars + inequality slacks

  // === One-sided (G-type) row normalization ===
  // The IPM's slack form A x + s = b requires a finite upper bound on every
  // inequality row.  Rows of the form A x >= lhs (b = +inf) are negated to
  // -A x <= -lhs (L-type); constraint_duals for flipped rows are negated
  // back at extraction.  `lp` is the effective problem for all constraint
  // data below (identical to prob when no one-sided rows exist).
  std::vector<char> flip_row(static_cast<size_t>(mi), 0);
  bool any_flip = false;
  for (int i = 0; i < mi; ++i) {
    const double lhs_i = lp_row_lhs_or_neg_inf(prob, i);
    if (!std::isfinite(prob.b[i]) && std::isfinite(lhs_i)) {
      flip_row[static_cast<size_t>(i)] = 1;
      any_flip = true;
    }
  }
  LPModel norm_lp;
  if (any_flip) {
    norm_lp = prob;
    for (int k = 0; k < norm_lp.A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(norm_lp.A, k); it; ++it)
        if (flip_row[static_cast<size_t>(it.row())]) it.valueRef() = -it.value();
    for (int i = 0; i < mi; ++i) {
      if (!flip_row[static_cast<size_t>(i)]) continue;
      norm_lp.b[i] = -lp_row_lhs_or_neg_inf(prob, i);
      if (norm_lp.row_lhs.size() == norm_lp.A.rows())
        norm_lp.row_lhs[i] = -std::numeric_limits<double>::infinity();
    }
  }
  const LPModel& lp = any_flip ? norm_lp : prob;

  // === Ruiz equilibration (honors the caller's round count) ===
  // Scales A/Aeq/b/c and finite variable bounds; slacks keep coefficient 1.
  // Outputs are unscaled at extraction: x = Dc·x̂, y = Dr·ŷ, z = ẑ/Dc.
  const RuizScaling scal =
      (ruiz_rounds > 0 && m > 0)
          ? ruiz_equilibrate(lp.A, lp.Aeq, n_orig, ruiz_rounds)
          : RuizScaling{};

  // === Bounds — use double flags (0.0/1.0) for branchless arithmetic ===
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)prob.vars.size()) ? prob.vars[j].lb : -kBig;
    double hi = (j < (int)prob.vars.size()) ? prob.vars[j].ub : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
    if (scal.active) {
      const double inv_d = 1.0 / scal.dc[static_cast<size_t>(j)];
      if (flb[j]) lo *= inv_d;
      if (fub[j]) hi *= inv_d;
    }
    lb[j] = lo; ub[j] = hi;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(lp, i);
    // Both b_i and lhs_i scale with dr_i, so the slack range scales once.
    const double dri = scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0;
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(lp.b[i])
            ? std::max(0.0, dri * (lp.b[i] - lhs))
            : kBig;
    flb[n_orig + i] = 1.0;
    fub[n_orig + i] = (ub[n_orig + i] < kBig - 1) ? 1.0 : 0.0;
  }

  // === Detect fixed variables (lb ≈ ub) ===
  // Fixed vars are removed from the barrier formulation: clear flb/fub so they
  // don't generate ill-conditioned barrier terms (gl→0, zl→∞).  Their primal
  // value is locked at lb, theta set to 0 to remove them from the normal
  // equations, and they are excluded from the dual convergence check.
  std::vector<bool> is_fixed(nn, false);
  int n_fixed_vars = 0;
  for (int j = 0; j < n_orig; ++j) {
    if (flb[j] && fub[j] && ub[j] - lb[j] < 1e-9) {
      is_fixed[j] = true;
      ++n_fixed_vars;
      flb[j] = 0.0;
      fub[j] = 0.0;
    }
  }

  // === Cost vector (with sense) ===
  std::vector<double> c(nn, 0.0);
  for (int j = 0; j < n_orig; ++j)
    c[j] = sense_sign * prob.c(j) *
           (scal.active ? scal.dc[static_cast<size_t>(j)] : 1.0);

  std::vector<double> b(m);
  for (int i = 0; i < mi; ++i)
    b[i] = (scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0) * lp.b(i);
  for (int i = 0; i < me; ++i)
    b[mi + i] = (scal.active ? scal.dr[static_cast<size_t>(mi + i)] : 1.0) * lp.beq(i);

  // CSC pointers — from the scaled copies when Ruiz is active, else direct
  const Eigen::SparseMatrix<double>& A_mat = scal.active ? scal.A : lp.A;
  const Eigen::SparseMatrix<double>& Aeq_mat = scal.active ? scal.Aeq : lp.Aeq;
  const int* A_o = A_mat.outerIndexPtr();
  const int* A_i = A_mat.innerIndexPtr();
  const double* A_v = A_mat.valuePtr();
  const int* Aeq_o = me > 0 ? Aeq_mat.outerIndexPtr() : nullptr;
  const int* Aeq_i = me > 0 ? Aeq_mat.innerIndexPtr() : nullptr;
  const double* Aeq_v = me > 0 ? Aeq_mat.valuePtr() : nullptr;

  // Build CSR for A (inequality) — needed for forward SpMV (sequential y writes)
  std::vector<int> A_rp, A_ci, Aeq_rp, Aeq_ci;
  std::vector<double> A_rv, Aeq_rv;
  auto build_csr = [](const Eigen::SparseMatrix<double>& M,
                       std::vector<int>& rp, std::vector<int>& ci, std::vector<double>& rv) {
    int rows = static_cast<int>(M.rows());
    rp.assign(rows + 1, 0);
    const int* Mo = M.outerIndexPtr();
    const int* Mi = M.innerIndexPtr();
    const double* Mv = M.valuePtr();
    int cols = static_cast<int>(M.cols());
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p)
        rp[Mi[p] + 1]++;
    for (int i = 0; i < rows; ++i) rp[i + 1] += rp[i];
    int nnz = rp[rows];
    ci.resize(nnz); rv.resize(nnz);
    std::vector<int> pos(rp.begin(), rp.begin() + rows);
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p) {
        int r = Mi[p];
        int q = pos[r]++;
        ci[q] = k; rv[q] = Mv[p];
      }
  };
  if (mi > 0) build_csr(A_mat, A_rp, A_ci, A_rv);
  if (me > 0) build_csr(Aeq_mat, Aeq_rp, Aeq_ci, Aeq_rv);

  // === SpMV operations ===
  // y -= Ae * x  (CSR-based: sequential y writes)
  auto ae_mul_sub = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(mi > MIPSOLVERS_OMP_THRESHOLD)
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] -= s;
    }
    MIPSOLVERS_OMP_PARALLEL_IF(me > MIPSOLVERS_OMP_THRESHOLD)
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] -= s;
    }
  };

  // y = Ae' * w  (CSC-based: gather from w, natural for CSC)
  auto aet_mul = [&](const double* MIPSOLVERS_RESTRICT w, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(n_orig > MIPSOLVERS_OMP_THRESHOLD)
    for (int j = 0; j < n_orig; ++j) {
      double s = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p)
        s += A_v[p] * w[A_i[p]];
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
          s += Aeq_v[p] * w[mi + Aeq_i[p]];
      y[j] = s;
    }
    for (int i = 0; i < mi; ++i)
      y[n_orig + i] = w[i];
  };

  // Merged: r_p = b - Ae*x, r_d = c - Ae'*y (single CSC pass)
  auto compute_residuals = [&](const double* MIPSOLVERS_RESTRICT xv, const double* MIPSOLVERS_RESTRICT yv,
                               double* MIPSOLVERS_RESTRICT rp, double* MIPSOLVERS_RESTRICT rd) {
    std::memcpy(rp, b.data(), sizeof(double) * m);
    std::memcpy(rd, c.data(), sizeof(double) * nn);
    for (int j = 0; j < n_orig; ++j) {
      double xj = xv[j];
      double dj = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
        int i = A_i[p];
        double aij = A_v[p];
        rp[i] -= aij * xj;
        dj += aij * yv[i];
      }
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
          int i = Aeq_i[p];
          double aij = Aeq_v[p];
          rp[mi + i] -= aij * xj;
          dj += aij * yv[mi + i];
        }
      rd[j] -= dj;
    }
    for (int i = 0; i < mi; ++i) {
      rp[i] -= xv[n_orig + i];
      rd[n_orig + i] -= yv[i];
    }
  };

  // === Compute bandwidth of N = AeΘAe' from A's and Aeq's structure ===
  int bandwidth = 0;
  // For original columns j: rows touched = {A's rows for col j} ∪ {mi + Aeq's rows for col j}
  for (int j = 0; j < n_orig; ++j) {
    int rmin = m, rmax = -1;
    for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
      rmin = std::min(rmin, A_i[p]);
      rmax = std::max(rmax, A_i[p]);
    }
    if (me > 0) {
      for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
        int r = mi + Aeq_i[p];
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
      }
    }
    if (rmax >= 0) bandwidth = std::max(bandwidth, rmax - rmin);
  }
  // Slack columns: each touches exactly 1 row, so bandwidth contribution = 0

  const bool use_banded = (bandwidth <= kBandedThreshold);
  bool use_dense = false;  // set below when scatter table would exceed kDenseScatterThreshold

  // === Banded storage: band[(row-col)*m + col] for row >= col, row-col <= bw ===
  const int bw = bandwidth;
  std::vector<double> band_storage;
  std::vector<BandScatterEntry> band_scatter;
  std::vector<int> band_scatter_col_start;

  // === Sparse path — normal equations N = Ae·Θ·Ae' ===
  Eigen::SparseMatrix<double> N_sparse;
#if MIPSOLVERS_HAVE_CHOLMOD
  // CHOLMOD is the preferred sparse Cholesky backend (supernodal BLAS-3).
  CholmodLDLT cholmod_ldlt;
#endif
  // True when the CHOLMOD backend owns the sparse normal-equations path;
  // false when CHOLMOD is not compiled or analyze failed, in which case
  // the platform backend below (Accelerate / Eigen) handles factorization.
  bool cholmod_ok = false;
#if MIPSOLVERS_USE_ACCELERATE
  std::vector<long> accel_col_starts;
  SparseOpaqueSymbolicFactorization accel_symbolic{};
  SparseOpaqueFactorization_Double accel_numeric{};
  bool accel_numeric_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt;
#endif
  // Scatter entries for sparse path
  struct SparseScatterEntry { int ni; double a_prod; };
  std::vector<SparseScatterEntry> sparse_scatter;
  std::vector<int> sparse_scatter_col_start;
  std::vector<int> sparse_diag_offsets;
  int sparse_n_nnz = 0;

  // Dense BLAS path (activated when A is too dense for the scatter approach)
  Eigen::MatrixXd Ae_dense, Ae_sqrt, N_dense;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_dense;

  if (use_banded) {
    band_storage.resize(static_cast<size_t>(bw + 1) * m, 0.0);

    // Build banded scatter map for original variable columns
    // Column j of Ae touches: A's rows for col j (at rows 0..mi-1)
    //                         plus Aeq's rows for col j (at rows mi..m-1)
    // Tight reserve bound: per column, each row pairs only with rows
    // inside the band (r1 - r2 <= bw), i.e. at most min(nz, bw+1) partners
    // per row.  Reserving nz^2 would over-allocate — a single dense column
    // (nz = 1e5) would request 1e10 entries -> bad_alloc.
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      const size_t nz =
          static_cast<size_t>(A_o[j + 1] - A_o[j]) +
          (me > 0 ? static_cast<size_t>(Aeq_o[j + 1] - Aeq_o[j]) : 0);
      total += nz * std::min(nz, static_cast<size_t>(bw) + 1);
    }
    band_scatter.reserve(total);
    band_scatter_col_start.resize(n_orig + 1);
    band_scatter_col_start[0] = 0;

    // Temporary buffer for rows/values of column j in Ae
    std::vector<int> col_rows;
    std::vector<double> col_vals;

    for (int j = 0; j < n_orig; ++j) {
      col_rows.clear();
      col_vals.clear();
      for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
        col_rows.push_back(A_i[p]);
        col_vals.push_back(A_v[p]);
      }
      if (me > 0) {
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
          col_rows.push_back(mi + Aeq_i[p]);
          col_vals.push_back(Aeq_v[p]);
        }
      }
      int nz = static_cast<int>(col_rows.size());
      for (int pi = 0; pi < nz; ++pi) {
        int r1 = col_rows[pi];
        double a1 = col_vals[pi];
        for (int pk = 0; pk < nz; ++pk) {
          int r2 = col_rows[pk];
          if (r1 < r2) continue;  // lower triangle only
          double a2 = col_vals[pk];
          int d = r1 - r2;
          if (d <= bw) {
            band_scatter.push_back({d * m + r2, a1 * a2});
          }
        }
      }
      band_scatter_col_start[j + 1] = static_cast<int>(band_scatter.size());
    }
  } else {
    // Estimate scatter table size before committing to the sparse path.
    // For dense A (e.g. random LP benchmarks), each column has O(m) non-zeros,
    // making the scatter O(n*m^2) entries — catastrophic in both memory and time.
    // Switch to a dense BLAS path in that case.
    {
      size_t estimated_scatter = 0;
      for (int j = 0; j < n_orig; ++j) {
        size_t nz_j = static_cast<size_t>(A_o[j + 1] - A_o[j]);
        if (me > 0) nz_j += static_cast<size_t>(Aeq_o[j + 1] - Aeq_o[j]);
        estimated_scatter += nz_j * nz_j;
      }
      // Memory feasibility gate: the dense path allocates m x nn (Ae_dense,
      // Ae_sqrt) plus m x m (N_dense).  Huge sparse LPs (n ~ 1e6) always
      // exceed the scatter threshold, which would request terabytes of
      // dense storage — stay sparse unless the dominant m x nn matrix
      // fits a bounded budget.
      const size_t dense_bytes =
          sizeof(double) * static_cast<size_t>(m) * static_cast<size_t>(nn);
      use_dense = (estimated_scatter > kDenseScatterThreshold) &&
                  (dense_bytes <= kDenseMaxBytes);
      if (!use_dense && estimated_scatter > kMaxScatterEntries) {
        // Neither path is memory-feasible: the dense matrices exceed the
        // byte budget and the sparse scatter map (Theta(estimated_scatter)
        // entries, also a proxy for nnz of N = Ae*Ae') exceeds its cap.
        // Fail cleanly instead of dying in reserve() with bad_alloc.
        out.stats.status = "ProblemTooLarge";
        out.stats.success = false;
        return out;
      }
    }

    if (use_dense) {
      // Dense BLAS path: build Ae_dense (m x nn) once.  Each IPM iteration computes
      //   N = Ae_sqrt * Ae_sqrt'  where Ae_sqrt[:,k] = sqrt(theta[k]) * Ae[:,k]
      // via Eigen (backed by BLAS/Accelerate), then factorizes with dense LDLT.
      // Cost per iter: O(m*nn) scale + O(m^2*nn) syrk + O(m^3/3) factorization.
      Ae_dense.resize(m, nn);
      Ae_dense.setZero();
      for (int j = 0; j < n_orig; ++j) {
        for (int p = A_o[j]; p < A_o[j + 1]; ++p)
          Ae_dense(A_i[p], j) = A_v[p];
        if (me > 0)
          for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
            Ae_dense(mi + Aeq_i[p], j) = Aeq_v[p];
      }
      for (int i = 0; i < mi; ++i)
        Ae_dense(i, n_orig + i) = 1.0;
      Ae_sqrt.resize(m, nn);
      N_dense.resize(m, m);
    } else {
      // Sparse LDLT path — build Ae and compute N_sparse = Ae * Ae'
      using T = Eigen::Triplet<double>;
      std::vector<T> trips;
      trips.reserve(prob.A.nonZeros() + prob.Aeq.nonZeros() + mi);
      for (int k = 0; k < prob.A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, k); it; ++it)
          trips.emplace_back(it.row(), it.col(), it.value());
      for (int i = 0; i < mi; ++i)
        trips.emplace_back(i, n_orig + i, 1.0);
      for (int k = 0; k < prob.Aeq.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Aeq, k); it; ++it)
          trips.emplace_back(mi + it.row(), it.col(), it.value());
      Eigen::SparseMatrix<double> Ae(m, nn);
      Ae.setFromTriplets(trips.begin(), trips.end());
      Ae.makeCompressed();
      const int* Ao = Ae.outerIndexPtr();
      const int* Ai = Ae.innerIndexPtr();
      const double* Av = Ae.valuePtr();

      N_sparse = Ae * Ae.transpose();
      for (int i = 0; i < m; ++i) N_sparse.coeffRef(i, i) += 1e-10;
      // Store only lower triangle: CHOLMOD requires this, and Eigen SimplicialLDLT
      // with Eigen::Lower also reads only the lower triangle.
      N_sparse = N_sparse.triangularView<Eigen::Lower>();
      N_sparse.makeCompressed();
      sparse_n_nnz = static_cast<int>(N_sparse.nonZeros());

      const int* No = N_sparse.outerIndexPtr();
      const int* Ni = N_sparse.innerIndexPtr();
      size_t total = 0;
      for (int j = 0; j < n_orig; ++j) {
        int nz = Ao[j + 1] - Ao[j];
        total += static_cast<size_t>(nz) * nz;
      }
      sparse_scatter.reserve(total);
      sparse_scatter_col_start.resize(n_orig + 1);
      sparse_scatter_col_start[0] = 0;
      for (int j = 0; j < n_orig; ++j) {
        for (int pi = Ao[j]; pi < Ao[j + 1]; ++pi)
          for (int pk = Ao[j]; pk < Ao[j + 1]; ++pk) {
            int r1 = Ai[pi], r2 = Ai[pk];
            const int* pos = std::lower_bound(Ni + No[r2], Ni + No[r2 + 1], r1);
            if (pos != Ni + No[r2 + 1] && *pos == r1)
              sparse_scatter.push_back({static_cast<int>(pos - Ni), Av[pi] * Av[pk]});
          }
        sparse_scatter_col_start[j + 1] = static_cast<int>(sparse_scatter.size());
      }
      sparse_diag_offsets.resize(m);
      for (int i = 0; i < m; ++i) {
        const int* pos = std::lower_bound(Ni + No[i], Ni + No[i + 1], i);
        sparse_diag_offsets[i] = static_cast<int>(pos - Ni);
      }
#if MIPSOLVERS_HAVE_CHOLMOD && !MIPSOLVERS_USE_ACCELERATE
      // CHOLMOD symbolic analysis — once per sparsity pattern; the numeric
      // factorization later re-reads the aliased N_sparse.valuePtr().
      // Priority: Accelerate on macOS (measured ~2.7x faster factorization
      // than CHOLMOD on the same BLAS), then CHOLMOD, then Eigen.
      cholmod_ok = cholmod_ldlt.analyze(m, N_sparse.outerIndexPtr(),
                                        N_sparse.innerIndexPtr(),
                                        N_sparse.valuePtr(), sparse_n_nnz);
#endif
#if !MIPSOLVERS_USE_ACCELERATE
      if (!cholmod_ok) ldlt.analyzePattern(N_sparse);
#endif
    }
  }

#if MIPSOLVERS_USE_ACCELERATE
  // Apple Accelerate sparse Cholesky setup (after N_sparse is built).
  // Helper lambdas to create Apple Sparse wrappers around N_sparse data.
  auto make_apple_structure = [&]() {
    SparseMatrixStructure s{};
    s.rowCount = m;
    s.columnCount = m;
    s.columnStarts = accel_col_starts.data();
    s.rowIndices = N_sparse.innerIndexPtr();
    s.attributes.transpose = false;
    s.attributes.triangle = SparseLowerTriangle;
    s.attributes.kind = SparseSymmetric;
    s.attributes._reserved = 0;
    s.attributes._allocatedBySparse = false;
    s.blockSize = 1;
    return s;
  };

  auto make_apple_matrix = [&]() {
    SparseMatrix_Double mat{};
    mat.structure = make_apple_structure();
    mat.data = N_sparse.valuePtr();
    return mat;
  };

  if (!use_banded && !use_dense && !cholmod_ok) {
    // Convert Eigen CSC column starts (int) to Apple format (long)
    const int* No = N_sparse.outerIndexPtr();
    accel_col_starts.resize(m + 1);
    for (int i = 0; i <= m; ++i)
      accel_col_starts[i] = static_cast<long>(No[i]);

    // Create or reuse symbolic factorization
    if (accel_cache_ && accel_cache_->valid &&
        accel_cache_->cached_m == m &&
        accel_cache_->cached_nnz == sparse_n_nnz) {
      accel_symbolic = accel_cache_->symbolic;
    } else {
      accel_symbolic = SparseFactor(SparseFactorizationCholesky, make_apple_structure());
      if (accel_cache_) {
        if (accel_cache_->valid) SparseCleanup(accel_cache_->symbolic);
        accel_cache_->symbolic = accel_symbolic;
        accel_cache_->col_starts = accel_col_starts;
        accel_cache_->cached_m = m;
        accel_cache_->cached_nnz = sparse_n_nnz;
        accel_cache_->valid = true;
      }
    }
  }
#endif

  // === Initialization ===
  std::vector<double> xv(nn), yv(m);
  std::vector<double> gl(nn), gu(nn), zl(nn), zu(nn);
  double* x_d = xv.data();
  double* y_d = yv.data();
  double* gl_d = gl.data();
  double* gu_d = gu.data();
  double* zl_d = zl.data();
  double* zu_d = zu.data();

  // ae_mul for init (CSR-based)
  auto ae_mul = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    MIPSOLVERS_OMP_PARALLEL_IF(mi > MIPSOLVERS_OMP_THRESHOLD)
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] = s;
    }
    MIPSOLVERS_OMP_PARALLEL_IF(me > MIPSOLVERS_OMP_THRESHOLD)
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] = s;
    }
  };

  // Initial point from least-squares (cold) or warm-start (from x0)
  {
    if (has_warm_start) {
      // Warm-start: use provided x0 as initial primal point.
      // Set original variables from x0, compute slacks from constraints.
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -kBig;
        double hi = fub[j] ? ub[j] : kBig;
        // Caller-provided x0 is in original coordinates: x̂0 = x0 / Dc.
        const double x0j =
            scal.active ? x0[j] / scal.dc[static_cast<size_t>(j)] : x0[j];
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0j, lo + 1e-6, hi - 1e-6);
        }
      }
      // Compute inequality slacks: s_i = b_i - A_i * x  (must be > 0)
      for (int i = 0; i < mi; ++i) {
        double ax = 0.0;
        for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
          ax += A_rv[p] * x_d[A_ci[p]];
        // Interior slack, finite even for G-type rows (b = +inf) and for
        // inits violating the row's two-sided range — otherwise the barrier
        // gap gu = ub - s goes negative/inf and NaNs the normal equations.
        double s_init = b[i] - ax;
        if (!std::isfinite(s_init)) s_init = 1.0;
        x_d[n_orig + i] =
            std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
      }
      // Dual initial: y = 0 (least-squares dual was tested but adds ~1ms
      // factorization overhead with no iteration reduction — primal warm-start
      // already drives convergence).
      std::fill(yv.begin(), yv.end(), 0.0);
    } else if (use_dense) {
      // Dense mode init: N = Ae_dense * Ae_dense' (Theta=I), solve for x and y.
      N_dense.noalias() = Ae_dense * Ae_dense.transpose();
      N_dense.diagonal().array() += 1e-10;
      ldlt_dense.compute(N_dense);
      if (ldlt_dense.info() == Eigen::Success) {
        Eigen::VectorXd tmp = ldlt_dense.solve(
            Eigen::Map<const Eigen::VectorXd>(b.data(), m));
        aet_mul(tmp.data(), x_d);
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        Eigen::VectorXd yy = ldlt_dense.solve(
            Eigen::Map<const Eigen::VectorXd>(tmp2.data(), m));
        std::memcpy(y_d, yy.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
    } else if (use_banded) {
      std::fill(band_storage.begin(), band_storage.end(), 0.0);
      for (int j = 0; j < n_orig; ++j)
        for (int si = band_scatter_col_start[j]; si < band_scatter_col_start[j + 1]; ++si)
          band_storage[band_scatter[si].offset] += band_scatter[si].a_prod;
      for (int i = 0; i < mi; ++i) band_storage[i] += 1.0;
      for (int i = 0; i < m; ++i) band_storage[i] += 1e-10;

      std::vector<double> bw_init(band_storage);
      if (banded_chol_factor(bw_init.data(), m, bw)) {
        std::memcpy(y_d, b.data(), sizeof(double) * m);
        banded_chol_solve(bw_init.data(), m, bw, y_d);
        aet_mul(y_d, x_d);

        ae_mul(c.data(), y_d);
        banded_chol_solve(bw_init.data(), m, bw, y_d);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
    } else {
#if MIPSOLVERS_HAVE_CHOLMOD
      if (cholmod_ok) {
        // N_sparse currently holds the theta=1 fill (N = Ae*Ae' + reg).
        if (cholmod_ldlt.factorize(N_sparse.valuePtr())) {
          // x_init = Ae' * (N \ b)
          std::memcpy(y_d, b.data(), sizeof(double) * m);
          cholmod_ldlt.solve(y_d, y_d);
          aet_mul(y_d, x_d);
          // y_init = N \ (Ae * c)
          std::vector<double> tmp2(m);
          ae_mul(c.data(), tmp2.data());
          cholmod_ldlt.solve(tmp2.data(), tmp2.data());
          std::memcpy(y_d, tmp2.data(), sizeof(double) * m);
        } else {
          std::fill(xv.begin(), xv.end(), 0.5);
          std::fill(yv.begin(), yv.end(), 0.0);
        }
      } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
      {
      SparseOpaqueFactorization_Double init_fac = SparseFactor(accel_symbolic, make_apple_matrix());
      if (init_fac.status == SparseStatusOK) {
        // x_init = Ae' * (N \ b)
        std::memcpy(y_d, b.data(), sizeof(double) * m);
        DenseVector_Double rhs1{};
        rhs1.count = m;
        rhs1.data = y_d;
        SparseSolve(init_fac, rhs1);
        aet_mul(y_d, x_d);
        // y_init = N \ (Ae * c)
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        DenseVector_Double rhs2{};
        rhs2.count = m;
        rhs2.data = tmp2.data();
        SparseSolve(init_fac, rhs2);
        std::memcpy(y_d, tmp2.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
      SparseCleanup(init_fac);
      }
#else
      {
      ldlt.factorize(N_sparse);
      if (ldlt.info() == Eigen::Success) {
        Eigen::Map<Eigen::VectorXd> bmap(b.data(), m);
        Eigen::VectorXd tmp = ldlt.solve(bmap);
        aet_mul(tmp.data(), x_d);
        std::vector<double> tmp2(m);
        ae_mul(c.data(), tmp2.data());
        Eigen::Map<Eigen::VectorXd> t2map(tmp2.data(), m);
        Eigen::VectorXd yy = ldlt.solve(t2map);
        std::memcpy(y_d, yy.data(), sizeof(double) * m);
      } else {
        std::fill(xv.begin(), xv.end(), 0.5);
        std::fill(yv.begin(), yv.end(), 0.0);
      }
      }
#endif
    }

    // Lock fixed variables at their exact value before general clamping.
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) x_d[j] = lb[j];
    }

    // If the least-squares init produced non-finite iterates or
    // astronomically large ones (near-singular initial normal matrix —
    // e.g. problems with many one-sided rows), fall back to the plain
    // interior point instead of propagating inf/huge values into the
    // barrier (comp_sum/mu or pf = inf at iteration 0).
    bool init_ok = true;
    for (int j = 0; j < nn && init_ok; ++j) {
      const double xj = x_d[j];
      init_ok = std::isfinite(xj) && std::abs(xj) < 1e10;
    }
    for (int i = 0; i < m && init_ok; ++i) {
      const double yi = y_d[i];
      init_ok = std::isfinite(yi) && std::abs(yi) < 1e10;
    }
    if (!init_ok) {
      std::fill(xv.begin(), xv.end(), 0.5);
      std::fill(yv.begin(), yv.end(), 0.0);
    }

    // Clamp original variables strictly inside their bounds, then recompute
    // slacks.  For one-sided variables the interior point must be built from
    // the *bounded* side only: the old `hi = x + 10` (lb-only case) fell below
    // lb whenever the least-squares init was negative, seeding x < lb and
    // leaving a permanent bound violation that the barrier gl = max(x-lb, eps)
    // silently hid (observed on afiro variable 24: x=-50, lb=0).
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) continue;  // already locked
      if (flb[j] && fub[j]) {
        const double range = ub[j] - lb[j];
        x_d[j] = std::clamp(x_d[j], lb[j] + 0.01 * range, ub[j] - 0.01 * range);
      } else if (flb[j]) {
        x_d[j] = std::max(x_d[j], lb[j] + 1.0);
      } else if (fub[j]) {
        x_d[j] = std::min(x_d[j], ub[j] - 1.0);
      }
      // free variables: keep the least-squares value as-is
    }
    // Recompute inequality slack variables: s_i = b_i - A_i * x (must be > 0)
    for (int i = 0; i < mi; ++i) {
      double ax = 0.0;
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        ax += A_rv[p] * x_d[A_ci[p]];
      // Interior slack, finite even for G-type rows (b = +inf) and for
      // inits violating the row's two-sided range (see warm-start path).
      double s_init = b[i] - ax;
      if (!std::isfinite(s_init)) s_init = 1.0;
      x_d[n_orig + i] =
          std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
    }
    for (int j = 0; j < nn; ++j) {
      gl_d[j] = flb[j] ? std::max(x_d[j] - lb[j], 1e-4) : kBig;
      gu_d[j] = fub[j] ? std::max(ub[j] - x_d[j], 1e-4) : kBig;
    }
    // Mehrotra-style dual start (Mehrotra 1992).  Setting zl=zu=1 arbitrarily
    // leaves the initial dual residual rd = c - Ae'y - flb*zl + fub*zu huge
    // (df ~ 1e7 on SCUC / scaled LPs) — the dominant cause of divergence.
    // Instead choose zl - zu = rc := c - Ae'y so rd starts near zero, then
    // shift both toward strict positivity and center the products g*z.
    {
      std::vector<double> mh_rp(m), mh_rc(nn);
      compute_residuals(x_d, y_d, mh_rp.data(), mh_rc.data());  // mh_rc = c - Ae'y
      double zmin = 0.0;
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) { zl_d[j] = 0.0; zu_d[j] = 0.0; continue; }
        const double rcj = mh_rc[j];
        double zl0 = 0.0, zu0 = 0.0;
        if (flb[j] && fub[j]) {
          zl0 = std::max(rcj, 0.0);
          zu0 = std::max(-rcj, 0.0);
        } else if (flb[j]) {
          zl0 = rcj;
        } else if (fub[j]) {
          zu0 = -rcj;
        }
        zl_d[j] = zl0;
        zu_d[j] = zu0;
        if (flb[j]) zmin = std::min(zmin, zl0);
        if (fub[j]) zmin = std::min(zmin, zu0);
      }
      const double dz = std::max(-1.5 * zmin, 0.0);
      double gz = 0.0, gsum = 0.0;
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) continue;
        if (flb[j] && gl_d[j] < kBig) { gz += gl_d[j] * (zl_d[j] + dz); gsum += gl_d[j]; }
        if (fub[j] && gu_d[j] < kBig) { gz += gu_d[j] * (zu_d[j] + dz); gsum += gu_d[j]; }
      }
      const double dzc = dz + ((gsum > 1e-30) ? 0.5 * gz / gsum : 1.0);
      for (int j = 0; j < nn; ++j) {
        if (j < n_orig && is_fixed[j]) continue;
        if (flb[j]) zl_d[j] = std::max(zl_d[j] + dzc, 1e-6);
        if (fub[j]) zu_d[j] = std::max(zu_d[j] + dzc, 1e-6);
      }
    }
  }

  // === IPM iteration ===
  std::vector<double> theta(nn), dx(nn), dy(m), dzl(nn), dzu(nn);
  std::vector<double> dx_aff(nn), dy_aff(m), dzl_aff(nn), dzu_aff(nn);
  std::vector<double> rhs(m), tmp_n(nn), Atdy(nn);
  std::vector<double> r_p(m), r_d(nn);
  std::vector<double> inv_gl(nn), inv_gu(nn);  // precomputed reciprocals

  double* theta_d = theta.data();
  double* dx_d = dx.data();
  double* dy_d = dy.data();
  double* dx_aff_d = dx_aff.data();
  double* dy_aff_d = dy_aff.data();
  double* dzl_aff_d = dzl_aff.data();
  double* dzu_aff_d = dzu_aff.data();
  double* rhs_d = rhs.data();
  double* tmp_d = tmp_n.data();
  double* Atdy_d = Atdy.data();
  double* r_p_d = r_p.data();
  double* r_d_d = r_d.data();
  double* inv_gl_d = inv_gl.data();
  double* inv_gu_d = inv_gu.data();
  const double* flb_d = flb.data();
  const double* fub_d = fub.data();
  const double* lb_d = lb.data();
  const double* ub_d = ub.data();

  // Primal-dual regularization (IP-PMM style).  reg is the diagonal added to
  // the normal equations (dual reg delta) and, via 1/(d+reg), also bounds
  // theta (primal reg rho).  It starts at reg_floor and is refreshed each
  // iteration to track the barrier scale mu (see the loop), so it stabilizes
  // the near-singular degenerate-LP normal equations early then vanishes as
  // mu -> 0 to keep the solution accurate.
  const double reg_floor = (n_fixed_vars > n_orig / 4) ? 1e-6 : 1e-10;
  double reg = reg_floor;
  const int max_iter = std::min(opt_.max_iter, 200);
  bool converged = false;

  double t_resid = 0, t_setup = 0, t_pred = 0, t_corr = 0, t_update = 0;
  double t_fill = 0, t_factor = 0;
  double t_init_overhead = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  auto tnow = []() { return std::chrono::steady_clock::now(); };

  // Working copy of banded Cholesky factor
  std::vector<double> band_work;
  if (use_banded) band_work.resize(band_storage.size());

  // Lambda: fill banded N from θ, factorize
  auto fill_and_factor_banded = [&]() -> bool {
    auto t_ff = tnow();
    double* bs = band_storage.data();
    const size_t bs_sz = band_storage.size();
    std::memset(bs, 0, sizeof(double) * bs_sz);

    // Original variable columns
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = band_scatter_col_start[j]; si < band_scatter_col_start[j + 1]; ++si)
        bs[band_scatter[si].offset] += th * band_scatter[si].a_prod;
    }
    // Slack columns: identity → theta of slack goes to diagonal
    for (int i = 0; i < mi; ++i)
      bs[i] += theta_d[n_orig + i];
    // Regularization
    for (int i = 0; i < m; ++i) bs[i] += reg;

    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();

    // Copy to working buffer and factorize
    std::memcpy(band_work.data(), bs, sizeof(double) * bs_sz);
    bool ok = banded_chol_factor(band_work.data(), m, bw);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ok;
  };

  auto fill_and_factor_dense = [&]() -> bool {
    auto t_ff = tnow();
    // Ae_sqrt[:,k] = sqrt(theta[k]) * Ae_dense[:,k]
    for (int k = 0; k < nn; ++k)
      Ae_sqrt.col(k) = Ae_dense.col(k) * std::sqrt(theta_d[k]);
    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();
    // N = Ae_sqrt * Ae_sqrt'  (symmetric rank-nn update = Ae * diag(theta) * Ae')
#if MIPSOLVERS_USE_ACCELERATE
    // Use Accelerate cblas_dsyrk for full BLAS performance (>100 GFLOPS on M4).
    // Fills only the lower triangle; Eigen::LDLT<MatrixXd> reads only lower by default.
    N_dense.setZero();
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    cblas_dsyrk(CblasColMajor, CblasLower, CblasNoTrans,
                m, nn, 1.0, Ae_sqrt.data(), m, 0.0, N_dense.data(), m);
#pragma clang diagnostic pop
#else
    N_dense.noalias() = Ae_sqrt * Ae_sqrt.transpose();
#endif
    N_dense.diagonal().array() += reg;
    ldlt_dense.compute(N_dense);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ldlt_dense.info() == Eigen::Success;
  };

  auto fill_and_factor_sparse = [&]() -> bool {
    auto t_ff = tnow();
    double* Nv = N_sparse.valuePtr();
    std::memset(Nv, 0, sizeof(double) * sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = sparse_scatter_col_start[j]; si < sparse_scatter_col_start[j + 1]; ++si)
        Nv[sparse_scatter[si].ni] += th * sparse_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i)
      Nv[sparse_diag_offsets[i]] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) Nv[sparse_diag_offsets[i]] += reg;
    auto t_ff2 = tnow();
    t_fill += std::chrono::duration<double, std::milli>(t_ff2 - t_ff).count();
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cholmod_ok) {
      const bool ok = cholmod_ldlt.factorize(N_sparse.valuePtr());
      t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
      return ok;
    }
#endif
#if MIPSOLVERS_USE_ACCELERATE
    SparseMatrix_Double apple_N = make_apple_matrix();
    if (accel_numeric_valid) {
      SparseRefactor(apple_N, &accel_numeric);
    } else {
      accel_numeric = SparseFactor(accel_symbolic, apple_N);
      accel_numeric_valid = true;
    }
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return accel_numeric.status == SparseStatusOK;
#else
    ldlt.factorize(N_sparse);
    t_factor += std::chrono::duration<double, std::milli>(tnow() - t_ff2).count();
    return ldlt.info() == Eigen::Success;
#endif
  };

  // Lambda: solve normal equations, with conditional iterative refinement
  // (mirrors the KKT path): after each solve, the residual r = rhs - N·dy
  // is checked against the pristine matrix; up to 2 correction solves with
  // the existing factorization run when ||r||∞ > 1e-12·max(1, ||rhs||∞).
  // Cost: one matvec per linear solve — negligible next to factorization.
  std::vector<double> ir_ndy(m), ir_r(m), ir_corr(m);
  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    auto raw_solve = [&](const double* rhs, double* dy) {
      if (use_banded) {
        std::memcpy(dy, rhs, sizeof(double) * m);
        banded_chol_solve(band_work.data(), m, bw, dy);
      } else if (use_dense) {
        Eigen::Map<const Eigen::VectorXd> rhs_map(rhs, m);
        Eigen::Map<Eigen::VectorXd> dy_map(dy, m);
        dy_map = ldlt_dense.solve(rhs_map);
      } else {
#if MIPSOLVERS_HAVE_CHOLMOD
        if (cholmod_ok) {
          if (!cholmod_ldlt.solve(rhs, dy)) {
            // Poison with NaN so the finiteness guard aborts the loop
            // (same pattern as the UMFPACK failure handling).
            std::fill(dy, dy + m, std::numeric_limits<double>::quiet_NaN());
          }
          return;
        }
#endif
#if MIPSOLVERS_USE_ACCELERATE
        std::memcpy(dy, rhs, sizeof(double) * m);
        DenseVector_Double xb{};
        xb.count = m;
        xb.data = dy;
        SparseSolve(accel_numeric, xb);
#else
        Eigen::Map<const Eigen::VectorXd> rhs_map(rhs, m);
        Eigen::Map<Eigen::VectorXd> dy_map(dy, m);
        dy_map = ldlt.solve(rhs_map);
#endif
      }
    };
    raw_solve(rhs_buf, dy_buf);
    if (m == 0) return;
    double rhs_norm = 0.0;
    for (int i = 0; i < m; ++i)
      rhs_norm = std::max(rhs_norm, std::abs(rhs_buf[i]));
    const double ir_tol = 1e-12 * std::max(1.0, rhs_norm);
    for (int ref = 0; ref < 2; ++ref) {
      // ndy = N * dy against the pristine matrix
      if (use_banded) {
        banded_sym_matvec(band_storage.data(), m, bw, dy_buf, ir_ndy.data());
      } else {
        Eigen::Map<const Eigen::VectorXd> dy_map(dy_buf, m);
        Eigen::Map<Eigen::VectorXd> ndy_map(ir_ndy.data(), m);
        if (use_dense) {
          ndy_map = N_dense.selfadjointView<Eigen::Lower>() * dy_map;
        } else {
          ndy_map = N_sparse.selfadjointView<Eigen::Lower>() * dy_map;
        }
      }
      double r_norm = 0.0;
      for (int i = 0; i < m; ++i) {
        ir_r[i] = rhs_buf[i] - ir_ndy[i];
        r_norm = std::max(r_norm, std::abs(ir_r[i]));
      }
      if (r_norm <= ir_tol) break;
      raw_solve(ir_r.data(), ir_corr.data());
      for (int i = 0; i < m; ++i) dy_buf[i] += ir_corr[i];
    }
  };

  for (int iter = 0; iter < max_iter; ++iter) {
    if (opt_.time_limit_sec > 0.0 && std::isfinite(opt_.time_limit_sec)) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= opt_.time_limit_sec) {
        out.stats.success = false;
        out.stats.iterations = iter;
        out.stats.status = "Time limit";
        break;
      }
    }
    auto t_s = tnow();

    // Residuals: r_p = b - Ae*x, r_d = c - Ae'*y (merged single pass)
    compute_residuals(x_d, y_d, r_p_d, r_d_d);

    // Merge: r_d adjustment + complementarity + norms — single pass over nn
    int n_compl = 0;
    double comp_sum = 0.0, pfeas = 0.0, dfeas = 0.0;
    for (int j = 0; j < nn; ++j) {
      r_d_d[j] -= flb_d[j] * zl_d[j];
      r_d_d[j] += fub_d[j] * zu_d[j];
      comp_sum += flb_d[j] * gl_d[j] * zl_d[j] + fub_d[j] * gu_d[j] * zu_d[j];
      n_compl += static_cast<int>(flb_d[j]) + static_cast<int>(fub_d[j]);
      double ad = std::abs(r_d_d[j]);
      if (ad > dfeas && !(j < n_orig && is_fixed[j])) dfeas = ad;
    }
    for (int i = 0; i < m; ++i) {
      double ap = std::abs(r_p_d[i]);
      if (ap > pfeas) pfeas = ap;
    }
    double mu = (n_compl > 0) ? comp_sum / n_compl : 0.0;

    if ((opt_.verbose || ipm_verbose_env) && (iter < 3 || iter % 5 == 0))
      fprintf(stderr, "IPM-LP %3d: pf=%.2e df=%.2e mu=%.2e\n", iter, pfeas, dfeas, mu);

    if (pfeas < opt_.tol_primal && dfeas < opt_.tol_dual && mu < opt_.tol_gap) {
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.primal_feas = pfeas;
      out.stats.dual_feas = dfeas;
      out.stats.complementarity = mu;
      out.stats.status = "Optimal";
      converged = true;
      break;
    }
    t_resid += std::chrono::duration<double, std::milli>(tnow() - t_s).count();

    // Build Θ, precompute inv_gl, inv_gu, and factorize
    auto t_su = tnow();
    // Refresh the primal-dual regularization to track the barrier scale mu.
    // reg damps the (otherwise unbounded on degenerate LPs) Newton step early
    // then vanishes as mu -> 0.  Applied as primal reg in theta = 1/(d+reg) —
    // bounding theta at 1/reg instead of the old hard 1e14 cap that amplified
    // dx = theta*(Ae'dy) into 1e25+ garbage — and as dual reg on the
    // normal-equations diagonal (added inside fill_and_factor).
    reg = std::clamp(1e-6 * mu, reg_floor, 1e-2);
    MIPSOLVERS_OMP_PARALLEL_IF(nn > MIPSOLVERS_OMP_THRESHOLD)
    for (int j = 0; j < nn; ++j) {
      if (j < n_orig && is_fixed[j]) {
        inv_gl_d[j] = 0.0;
        inv_gu_d[j] = 0.0;
        theta_d[j] = 0.0;  // remove fixed var from normal equations
        continue;
      }
      double igl = flb_d[j] / gl_d[j];  // 0 if no lb (flb=0, gl=kBig)
      double igu = fub_d[j] / gu_d[j];
      inv_gl_d[j] = igl;
      inv_gu_d[j] = igu;
      double d = zl_d[j] * igl + zu_d[j] * igu;
      theta_d[j] = 1.0 / (d + reg);
    }

    bool factor_ok = use_banded ? fill_and_factor_banded()
                                : (use_dense ? fill_and_factor_dense() : fill_and_factor_sparse());
    // Dynamic regularization retry: if Cholesky fails (ill-conditioned normal
    // equations from near-parallel constraints), increase diagonal perturbation.
    if (!factor_ok) {
      double dyn_reg = std::max(reg * 1e4, 1e-8);
      for (int retry = 0; retry < 4 && !factor_ok; ++retry) {
        if (use_banded) {
          // Restore the pristine matrix before perturbing: the failed
          // attempt left band_work partially factorized, so adding dyn_reg
          // without this refill would factorize garbage.  band_storage
          // still holds the untouched fill from fill_and_factor_banded().
          std::memcpy(band_work.data(), band_storage.data(),
                      sizeof(double) * band_storage.size());
          double* bw_data = band_work.data();
          for (int i = 0; i < m; ++i) bw_data[i] += dyn_reg;
          factor_ok = banded_chol_factor(bw_data, m, bw);
        } else if (use_dense) {
          N_dense.diagonal().array() += dyn_reg;
          ldlt_dense.compute(N_dense);
          factor_ok = (ldlt_dense.info() == Eigen::Success);
        } else {
          // Re-fill sparse normal equations with extra diagonal reg
          double* Nv = N_sparse.valuePtr();
          for (int i = 0; i < m; ++i) Nv[sparse_diag_offsets[i]] += dyn_reg;
#if MIPSOLVERS_HAVE_CHOLMOD
          if (cholmod_ok) {
            factor_ok = cholmod_ldlt.factorize(N_sparse.valuePtr());
          } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
          {
          SparseMatrix_Double apple_N = make_apple_matrix();
          if (accel_numeric_valid) {
            SparseRefactor(apple_N, &accel_numeric);
          } else {
            accel_numeric = SparseFactor(accel_symbolic, apple_N);
            accel_numeric_valid = true;
          }
          factor_ok = (accel_numeric.status == SparseStatusOK);
          }
#else
          {
          ldlt.factorize(N_sparse);
          factor_ok = (ldlt.info() == Eigen::Success);
          }
#endif
        }
        if (!factor_ok) dyn_reg *= 100.0;
      }
    }
    if (!factor_ok) {
      out.stats.status = "Cholesky failed";
      out.stats.iterations = iter;
      break;
    }
    t_setup += std::chrono::duration<double, std::milli>(tnow() - t_su).count();

    // ---- Predictor (affine, σ=0) ----
    auto t_p = tnow();

    // ξ_aff = -r_d - z_l + z_u  →  tmp_n = Θ * ξ_aff
    for (int j = 0; j < nn; ++j) {
      double xi = -r_d_d[j] - flb_d[j] * zl_d[j] + fub_d[j] * zu_d[j];
      tmp_d[j] = theta_d[j] * xi;
    }

    // rhs = r_p - Ae*tmp_n
    std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
    ae_mul_sub(tmp_d, rhs_d);
    solve_normal(rhs_d, dy_aff_d);

    // Δx_aff = Θ(Ae'Δy_aff) + tmp_n
    aet_mul(dy_aff_d, Atdy_d);
    simd_fma(theta_d, Atdy_d, tmp_d, dx_aff_d, nn);  // dx_aff = theta * Atdy + tmp

    // Δz_aff from complementarity
    for (int j = 0; j < nn; ++j) {
      dzl_aff_d[j] = flb_d[j] * (-zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_aff_d[j]);
      dzu_aff_d[j] = fub_d[j] * (-zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_aff_d[j]);
    }

    // Affine step sizes
    double ap_aff = 1.0, ad_aff = 1.0;
    for (int j = 0; j < nn; ++j) {
      if (dx_aff_d[j] < -kMinVal && flb_d[j]) {
        double a = -kTau * gl_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dx_aff_d[j] > kMinVal && fub_d[j]) {
        double a = kTau * gu_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dzl_aff_d[j] < -kMinVal) {
        double a = -kTau * zl_d[j] / dzl_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
      if (dzu_aff_d[j] < -kMinVal) {
        double a = -kTau * zu_d[j] / dzu_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
    }
    ap_aff = std::max(ap_aff, kMinVal);
    ad_aff = std::max(ad_aff, kMinVal);

    // Centering parameter
    double mu_aff = 0.0;
    for (int j = 0; j < nn; ++j) {
      mu_aff += flb_d[j] * (gl_d[j] + ap_aff * dx_aff_d[j]) * (zl_d[j] + ad_aff * dzl_aff_d[j]);
      mu_aff += fub_d[j] * (gu_d[j] - ap_aff * dx_aff_d[j]) * (zu_d[j] + ad_aff * dzu_aff_d[j]);
    }
    mu_aff = (n_compl > 0) ? mu_aff / n_compl : 0.0;
    // n_compl == 0 (no finite bounds anywhere) gives mu = mu_aff = 0, so
    // mu_aff / mu would be 0/0 = NaN.  With no complementarity there is
    // nothing to center — take a pure affine step instead of letting NaN
    // propagate into the corrector rhs and the returned solution.
    double sigma = (mu > 0.0) ? std::min(std::pow(mu_aff / mu, 3.0), 0.5) : 0.0;
    double sigma_mu = sigma * mu;

    t_pred += std::chrono::duration<double, std::milli>(tnow() - t_p).count();

    // Conservative corrector skip: only when affine step is near-optimal
    // Skip when: excellent affine step (both > 0.9) AND sigma is tiny (< 0.01)
    double ap, ad;
    auto t_c = tnow();
    const bool skip_corrector = (ap_aff > 0.9 && ad_aff > 0.9 && sigma < 0.02);
    
    if (skip_corrector) {
      // Use affine direction directly (with very small centering)
      ap = ap_aff;
      ad = ad_aff;
      std::memcpy(dx_d, dx_aff_d, sizeof(double) * nn);
      std::memcpy(dy_d, dy_aff_d, sizeof(double) * m);
      std::memcpy(dzl.data(), dzl_aff_d, sizeof(double) * nn);
      std::memcpy(dzu.data(), dzu_aff_d, sizeof(double) * nn);
    } else {
      // ---- Corrector (centering + Mehrotra second-order) ----
      for (int j = 0; j < nn; ++j) {
        double xi = -r_d_d[j];
        xi += flb_d[j] * (sigma_mu * inv_gl_d[j] - zl_d[j] - dzl_aff_d[j] * dx_aff_d[j] * inv_gl_d[j]);
        xi -= fub_d[j] * (sigma_mu * inv_gu_d[j] - zu_d[j] + dzu_aff_d[j] * dx_aff_d[j] * inv_gu_d[j]);
        tmp_d[j] = theta_d[j] * xi;
      }
      std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
      ae_mul_sub(tmp_d, rhs_d);
      solve_normal(rhs_d, dy_d);

      aet_mul(dy_d, Atdy_d);
      simd_fma(theta_d, Atdy_d, tmp_d, dx_d, nn);  // dx = theta * Atdy + tmp

      for (int j = 0; j < nn; ++j) {
        dzl[j] = flb_d[j] * ((sigma_mu - dzl_aff_d[j] * dx_aff_d[j]) * inv_gl_d[j] - zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_d[j]);
        dzu[j] = fub_d[j] * ((sigma_mu + dzu_aff_d[j] * dx_aff_d[j]) * inv_gu_d[j] - zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_d[j]);
      }

      // Corrector step sizes
      ap = 1.0;
      ad = 1.0;
      for (int j = 0; j < nn; ++j) {
        if (dx_d[j] < -kMinVal && flb_d[j]) {
          double a = -kTau * gl_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dx_d[j] > kMinVal && fub_d[j]) {
          double a = kTau * gu_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dzl[j] < -kMinVal) {
          double a = -kTau * zl_d[j] / dzl[j];
          if (a < ad) ad = a;
        }
        if (dzu[j] < -kMinVal) {
          double a = -kTau * zu_d[j] / dzu[j];
          if (a < ad) ad = a;
        }
      }
      ap = std::max(ap, kMinVal);
      ad = std::max(ad, kMinVal);
    }

    t_corr += std::chrono::duration<double, std::milli>(tnow() - t_c).count();

    // Finiteness guard: a failed solve or NaN in the search direction must
    // abort the loop instead of polluting the iterate and returning a NaN
    // "solution" after MaxIter.  dzl/dzu derive from dx, so checking the
    // two solve outputs dx/dy covers every NaN source upstream.
    bool step_finite = true;
    for (int j = 0; j < nn; ++j)
      if (!std::isfinite(dx_d[j])) { step_finite = false; break; }
    if (step_finite)
      for (int i = 0; i < m; ++i)
        if (!std::isfinite(dy_d[i])) { step_finite = false; break; }
    if (!step_finite) {
      out.stats.status = "NumericalError";
      out.stats.iterations = iter;
      break;
    }

    // ---- Update (SIMD-accelerated) ----
    auto t_u = tnow();
    simd_axpy(ap, dx_d, x_d, nn);                   // x += ap * dx
    simd_axpy(ad, dy_d, y_d, m);                    // y += ad * dy
    simd_axpy_max(ad, dzl.data(), zl_d, kMinVal, nn);  // zl = max(zl + ad*dzl, kMin)
    simd_axpy_max(ad, dzu.data(), zu_d, kMinVal, nn);  // zu = max(zu + ad*dzu, kMin)
    simd_sub_max(x_d, lb_d, gl_d, kMinVal, nn);    // gl = max(x - lb, kMin)
    simd_rsub_max(x_d, ub_d, gu_d, kMinVal, nn);   // gu = max(ub - x, kMin)
    // Re-lock fixed variables (prevent drift from numerical arithmetic)
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) {
        x_d[j] = lb[j];
        gl_d[j] = kBig;
        gu_d[j] = kBig;
      }
    }
    t_update += std::chrono::duration<double, std::milli>(tnow() - t_u).count();
  }

  if (opt_.verbose) {
    auto t_end = std::chrono::steady_clock::now();
    printf("  IPM timing: total=%.3f ms (m=%d, nn=%d, bw=%d, %s)\n",
           std::chrono::duration<double, std::milli>(t_end - t0).count(), m, nn, bandwidth,
           use_banded ? "BANDED" : (use_dense ? "DENSE" : "SPARSE"));
    printf("    init=%.3f resid=%.3f setup=%.3f pred=%.3f corr=%.3f update=%.3f\n",
           t_init_overhead, t_resid, t_setup, t_pred, t_corr, t_update);
    printf("    setup_sub: fill=%.3f factor=%.3f\n", t_fill, t_factor);
  }

  // === Extract solution ===
#if MIPSOLVERS_USE_ACCELERATE
  if (!use_banded && !use_dense && accel_numeric_valid) {
    SparseCleanup(accel_numeric);
  }
#endif
  if (!converged && out.stats.status.empty()) {
    out.stats.status = "MaxIter";
    out.stats.success = false;
  }
  out.x.resize(n_orig);
  for (int j = 0; j < n_orig; ++j)
    out.x(j) = scal.active ? x_d[j] * scal.dc[static_cast<size_t>(j)] : x_d[j];
  out.stats.objective = prob.c.dot(out.x);
  // Original-space feasibility audit.  A scaled (Ruiz) solve can converge in
  // scaled coordinates yet map back to an infeasible original-space point
  // (observed on afiro).  Reject such false-optima so solve_lp's unscaled retry
  // can recover, and so "Optimal" is never reported for a violated solution.
  if (out.stats.success) {
    double bnd_viol = 0.0;
    for (int j = 0; j < n_orig; ++j) {
      const double lbj = prob.vars[static_cast<size_t>(j)].lb;
      const double ubj = prob.vars[static_cast<size_t>(j)].ub;
      if (lbj > -1e19) bnd_viol = std::max(bnd_viol, lbj - out.x(j));
      if (ubj < 1e19) bnd_viol = std::max(bnd_viol, out.x(j) - ubj);
    }
    double row_viol = 0.0;
    if (prob.A.rows() > 0) {
      const Eigen::VectorXd ax = prob.A * out.x;
      for (int i = 0; i < ax.size(); ++i)
        row_viol = std::max(row_viol, ax(i) - prob.b(i));
    }
    if (prob.Aeq.rows() > 0) {
      const Eigen::VectorXd ae = prob.Aeq * out.x;
      for (int i = 0; i < ae.size(); ++i)
        row_viol = std::max(row_viol, std::abs(ae(i) - prob.beq(i)));
    }
    double scale = 1.0;
    if (prob.b.size()) scale = std::max(scale, prob.b.cwiseAbs().maxCoeff());
    if (prob.beq.size()) scale = std::max(scale, prob.beq.cwiseAbs().maxCoeff());
    if (out.x.size()) scale = std::max(scale, out.x.cwiseAbs().maxCoeff());
    const double audit_tol = 1e-6 * scale;
    if (std::max(bnd_viol, row_viol) > audit_tol) {
      out.stats.success = false;
      out.stats.status = "Residual audit rejected";
    }
  }
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i) {
      // G-type rows were negated to L-type at input: flip the dual sign back
      // (KKT multiplier of -A x <= -lhs is the negative of A x >= lhs).
      const double row_sign =
          (any_flip && i < mi && flip_row[static_cast<size_t>(i)]) ? -1.0 : 1.0;
      out.constraint_duals(i) = row_sign * sense_sign * y_d[i] *
                                (scal.active ? scal.dr[static_cast<size_t>(i)] : 1.0);
    }
  }
  // Export bound multipliers for original variables (used by IPM→simplex crossover).
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = scal.active ? zl_d[j] / scal.dc[static_cast<size_t>(j)]
                                     : zl_d[j];
    out.box_dual_ub(j) = scal.active ? zu_d[j] / scal.dc[static_cast<size_t>(j)]
                                     : zu_d[j];
  }
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);
  return out;
}

SolveResult NativeIPMLPAdapter::solve_node_lp(const LPModel& base_lp,
                                               const Eigen::VectorXd& node_lb,
                                               const Eigen::VectorXd& node_ub) const {
  // Use cached path if available
  if (cached_state_) {
    static const Eigen::VectorXd empty;
    return solve_cached_node_lp(node_lb, node_ub, empty);
  }
  LPModel node_lp = base_lp;
  const int n = static_cast<int>(node_lp.vars.size());
  for (int i = 0; i < n; ++i) {
    node_lp.vars[i].lb = node_lb[i];
    node_lp.vars[i].ub = node_ub[i];
  }
  return solve_lp(node_lp);
}

SolveResult NativeIPMLPAdapter::solve_node_lp(const LPModel& base_lp,
                                               const Eigen::VectorXd& node_lb,
                                               const Eigen::VectorXd& node_ub,
                                               const Eigen::VectorXd& x0) const {
  // Use cached path if available
  if (cached_state_) {
    return solve_cached_node_lp(node_lb, node_ub, x0);
  }
  LPModel node_lp = base_lp;
  const int n = static_cast<int>(node_lp.vars.size());
  for (int i = 0; i < n; ++i) {
    node_lp.vars[i].lb = node_lb[i];
    node_lp.vars[i].ub = node_ub[i];
  }
  return solve_lp(node_lp, x0);
}

// ======================================================================
// CachedState: persistent infrastructure for repeated node LP solves
// ======================================================================

struct NativeIPMLPAdapter::CachedState {
  // Problem dimensions
  int n_orig = 0;  // original variables
  int mi = 0;      // inequality constraints
  int me = 0;      // equality constraints
  int m = 0;       // mi + me
  int nn = 0;      // n_orig + mi (with slacks)
  int sense_sign = 1;

  // Cost vector (with sense applied)
  std::vector<double> c;

  // RHS
  std::vector<double> b;

  // CSC pointers (borrowed from Eigen sparse — must keep base_lp alive)
  const int* A_o = nullptr;
  const int* A_i = nullptr;
  const double* A_v = nullptr;
  const int* Aeq_o = nullptr;
  const int* Aeq_i = nullptr;
  const double* Aeq_v = nullptr;

  // CSR representation
  std::vector<int> A_rp, A_ci, Aeq_rp, Aeq_ci;
  std::vector<double> A_rv, Aeq_rv;

  // Bandwidth and banded/sparse choice
  int bandwidth = 0;
  bool use_banded = false;

  // Banded path
  std::vector<double> band_storage;
  std::vector<BandScatterEntry> band_scatter;
  std::vector<int> band_scatter_col_start;

  // Sparse path
  Eigen::SparseMatrix<double> N_sparse;
  struct SparseScatterEntry { int ni; double a_prod; };
  std::vector<SparseScatterEntry> sparse_scatter;
  std::vector<int> sparse_scatter_col_start;
  std::vector<int> sparse_diag_offsets;
  int sparse_n_nnz = 0;

#if MIPSOLVERS_USE_ACCELERATE
  std::vector<long> accel_col_starts;
  SparseOpaqueSymbolicFactorization accel_symbolic{};
  bool accel_symbolic_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt;
#endif

  // Warm-start dual from previous solve (carried across node LPs)
  mutable std::vector<double> last_dual;
  // Box multipliers from previous solve
  mutable std::vector<double> last_zl, last_zu;

  // Copy of base LP (owns the Eigen sparse matrices whose pointers we borrow)
  LPModel base_lp_copy;

  // One-sided (G-type) rows negated to L-type at prepare time (same
  // normalization as solve_lp): constraint_duals for these rows are negated
  // back at extraction.
  std::vector<char> flip_row;

  // Ruiz scaling (inactive when opt_.ruiz_rounds == 0).  When active, the
  // A_o/A_i/A_v and Aeq_* pointers reference the owned scaled matrices
  // below, and c/b are stored scaled.  Node bounds are scaled by 1/dc per
  // solve; outputs are unscaled: x = Dc·x̂, y = Dr·ŷ, z = ẑ/Dc.
  bool scaling_active = false;
  std::vector<double> scal_dr, scal_dc;
  Eigen::SparseMatrix<double> A_scaled, Aeq_scaled;

#if MIPSOLVERS_HAVE_CHOLMOD
  // CHOLMOD backend for the sparse normal-equations path (preferred over
  // Accelerate/Eigen when cholmod_ok is true).  mutable like the other
  // cache scratch state (factorize/solve run from const adapter methods).
  mutable CholmodLDLT cholmod;
#endif
  bool cholmod_ok = false;

  ~CachedState() {
#if MIPSOLVERS_USE_ACCELERATE
    if (accel_symbolic_valid) SparseCleanup(accel_symbolic);
#endif
  }
};

void NativeIPMLPAdapter::prepare_for_node_solves(const LPModel& base_lp) {
  auto cs = std::make_unique<CachedState>();

  // Keep a copy so CSC pointers remain valid
  cs->base_lp_copy = base_lp;

  // One-sided (G-type) row normalization (same as solve_lp): rows of the
  // form A x >= lhs (b = +inf) are negated to -A x <= -lhs so every row has
  // a finite upper bound; duals are negated back at extraction.
  {
    const int mi0 = static_cast<int>(cs->base_lp_copy.A.rows());
    cs->flip_row.assign(static_cast<size_t>(mi0), 0);
    bool any_flip = false;
    for (int i = 0; i < mi0; ++i) {
      const double lhs_i = lp_row_lhs_or_neg_inf(cs->base_lp_copy, i);
      if (!std::isfinite(cs->base_lp_copy.b[i]) && std::isfinite(lhs_i)) {
        cs->flip_row[static_cast<size_t>(i)] = 1;
        any_flip = true;
      }
    }
    if (any_flip) {
      for (int k = 0; k < cs->base_lp_copy.A.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(cs->base_lp_copy.A, k);
             it; ++it)
          if (cs->flip_row[static_cast<size_t>(it.row())])
            it.valueRef() = -it.value();
      for (int i = 0; i < mi0; ++i) {
        if (!cs->flip_row[static_cast<size_t>(i)]) continue;
        cs->base_lp_copy.b[i] = -lp_row_lhs_or_neg_inf(base_lp, i);
        if (cs->base_lp_copy.row_lhs.size() == cs->base_lp_copy.A.rows())
          cs->base_lp_copy.row_lhs[i] =
              -std::numeric_limits<double>::infinity();
      }
    } else {
      cs->flip_row.clear();
    }
  }

  const auto& lp = cs->base_lp_copy;

  cs->n_orig = static_cast<int>(lp.c.size());
  if (cs->n_orig == 0) { cached_state_ = std::move(cs); return; }

  cs->sense_sign = (lp.sense == Sense::Maximize) ? -1 : 1;
  cs->mi = static_cast<int>(lp.A.rows());
  cs->me = static_cast<int>(lp.Aeq.rows());
  cs->m = cs->mi + cs->me;
  cs->nn = cs->n_orig + cs->mi;

  const int n_orig = cs->n_orig;
  const int mi = cs->mi;
  const int me = cs->me;
  const int m = cs->m;
  const int nn = cs->nn;

  // Ruiz equilibration (same option as solve_lp).  Scaled matrices and the
  // dr/dc vectors move into the cache; all downstream structures (CSR,
  // scatter maps, symbolic factorization) are built from the scaled data.
  {
    RuizScaling scal =
        (opt_.ruiz_rounds > 0 && m > 0)
            ? ruiz_equilibrate(lp.A, lp.Aeq, n_orig, opt_.ruiz_rounds)
            : RuizScaling{};
    cs->scaling_active = scal.active;
    if (scal.active) {
      cs->A_scaled = std::move(scal.A);
      cs->Aeq_scaled = std::move(scal.Aeq);
      cs->scal_dr = std::move(scal.dr);
      cs->scal_dc = std::move(scal.dc);
    }
  }
  const Eigen::SparseMatrix<double>& A_mat =
      cs->scaling_active ? cs->A_scaled : lp.A;
  const Eigen::SparseMatrix<double>& Aeq_mat =
      cs->scaling_active ? cs->Aeq_scaled : lp.Aeq;

  // Cost
  cs->c.resize(nn, 0.0);
  for (int j = 0; j < n_orig; ++j)
    cs->c[j] = cs->sense_sign * lp.c(j) *
               (cs->scaling_active ? cs->scal_dc[static_cast<size_t>(j)] : 1.0);

  // RHS
  cs->b.resize(m);
  for (int i = 0; i < mi; ++i)
    cs->b[i] = (cs->scaling_active ? cs->scal_dr[static_cast<size_t>(i)] : 1.0) * lp.b(i);
  for (int i = 0; i < me; ++i)
    cs->b[mi + i] = (cs->scaling_active ? cs->scal_dr[static_cast<size_t>(mi + i)] : 1.0) * lp.beq(i);

  // CSC pointers from our copy (scaled matrices when Ruiz is active)
  cs->A_o = A_mat.outerIndexPtr();
  cs->A_i = A_mat.innerIndexPtr();
  cs->A_v = A_mat.valuePtr();
  cs->Aeq_o = me > 0 ? Aeq_mat.outerIndexPtr() : nullptr;
  cs->Aeq_i = me > 0 ? Aeq_mat.innerIndexPtr() : nullptr;
  cs->Aeq_v = me > 0 ? Aeq_mat.valuePtr() : nullptr;

  // Build CSR
  auto build_csr = [](const Eigen::SparseMatrix<double>& M,
                       std::vector<int>& rp, std::vector<int>& ci, std::vector<double>& rv) {
    int rows = static_cast<int>(M.rows());
    rp.assign(rows + 1, 0);
    const int* Mo = M.outerIndexPtr();
    const int* Mi = M.innerIndexPtr();
    const double* Mv = M.valuePtr();
    int cols = static_cast<int>(M.cols());
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p)
        rp[Mi[p] + 1]++;
    for (int i = 0; i < rows; ++i) rp[i + 1] += rp[i];
    int nnz = rp[rows];
    ci.resize(nnz); rv.resize(nnz);
    std::vector<int> pos(rp.begin(), rp.begin() + rows);
    for (int k = 0; k < cols; ++k)
      for (int p = Mo[k]; p < Mo[k + 1]; ++p) {
        int r = Mi[p];
        int q = pos[r]++;
        ci[q] = k; rv[q] = Mv[p];
      }
  };
  if (mi > 0) build_csr(A_mat, cs->A_rp, cs->A_ci, cs->A_rv);
  if (me > 0) build_csr(Aeq_mat, cs->Aeq_rp, cs->Aeq_ci, cs->Aeq_rv);

  // Bandwidth
  for (int j = 0; j < n_orig; ++j) {
    int rmin = m, rmax = -1;
    for (int p = cs->A_o[j]; p < cs->A_o[j + 1]; ++p) {
      rmin = std::min(rmin, cs->A_i[p]);
      rmax = std::max(rmax, cs->A_i[p]);
    }
    if (me > 0) {
      for (int p = cs->Aeq_o[j]; p < cs->Aeq_o[j + 1]; ++p) {
        int r = mi + cs->Aeq_i[p];
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
      }
    }
    if (rmax >= 0) cs->bandwidth = std::max(cs->bandwidth, rmax - rmin);
  }
  cs->use_banded = (cs->bandwidth <= kBandedThreshold);

  if (cs->use_banded) {
    const int bw = cs->bandwidth;
    cs->band_storage.resize(static_cast<size_t>(bw + 1) * m, 0.0);

    // Tight reserve bound (same as solve_lp): rows pair only within the
    // band, so nz^2 would over-allocate and a single dense column would
    // request ~1e10 entries -> bad_alloc.
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      const size_t nz =
          static_cast<size_t>(cs->A_o[j + 1] - cs->A_o[j]) +
          (me > 0 ? static_cast<size_t>(cs->Aeq_o[j + 1] - cs->Aeq_o[j]) : 0);
      total += nz * std::min(nz, static_cast<size_t>(bw) + 1);
    }
    if (total > kMaxScatterEntries) {
      // Scatter map too large — skip caching; node LPs fall back to the
      // full solve_lp path (which has its own memory-feasibility gates).
      cached_state_.reset();
      return;
    }
    cs->band_scatter.reserve(total);
    cs->band_scatter_col_start.resize(n_orig + 1);
    cs->band_scatter_col_start[0] = 0;

    std::vector<int> col_rows;
    std::vector<double> col_vals;
    for (int j = 0; j < n_orig; ++j) {
      col_rows.clear(); col_vals.clear();
      for (int p = cs->A_o[j]; p < cs->A_o[j + 1]; ++p) {
        col_rows.push_back(cs->A_i[p]);
        col_vals.push_back(cs->A_v[p]);
      }
      if (me > 0) {
        for (int p = cs->Aeq_o[j]; p < cs->Aeq_o[j + 1]; ++p) {
          col_rows.push_back(mi + cs->Aeq_i[p]);
          col_vals.push_back(cs->Aeq_v[p]);
        }
      }
      int nz = static_cast<int>(col_rows.size());
      for (int pi = 0; pi < nz; ++pi)
        for (int pk = 0; pk < nz; ++pk) {
          int r1 = col_rows[pi], r2 = col_rows[pk];
          if (r1 < r2) continue;
          int d = r1 - r2;
          if (d <= bw)
            cs->band_scatter.push_back({d * m + r2, col_vals[pi] * col_vals[pk]});
        }
      cs->band_scatter_col_start[j + 1] = static_cast<int>(cs->band_scatter.size());
    }
  } else {
    // Estimate the scatter map size (Theta(sum_j nz_j^2), also an upper
    // proxy for nnz of N = Ae*Ae') before building anything: a single
    // dense column (nz = 1e5) implies ~1e10 entries -> bad_alloc.  Skip
    // caching in that case; node LPs fall back to the full solve_lp path.
    {
      size_t scatter_est = 0;
      for (int j = 0; j < n_orig; ++j) {
        const size_t nz =
            static_cast<size_t>(cs->A_o[j + 1] - cs->A_o[j]) +
            (me > 0 ? static_cast<size_t>(cs->Aeq_o[j + 1] - cs->Aeq_o[j]) : 0);
        scatter_est += nz * nz;
      }
      if (scatter_est > kMaxScatterEntries) {
        cached_state_.reset();
        return;
      }
    }
    // Sparse path: build Ae, N_sparse, scatter maps, symbolic factorization
    using T = Eigen::Triplet<double>;
    std::vector<T> trips;
    trips.reserve(A_mat.nonZeros() + Aeq_mat.nonZeros() + mi);
    for (int k = 0; k < A_mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(A_mat, k); it; ++it)
        trips.emplace_back(it.row(), it.col(), it.value());
    for (int i = 0; i < mi; ++i)
      trips.emplace_back(i, n_orig + i, 1.0);
    for (int k = 0; k < Aeq_mat.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq_mat, k); it; ++it)
        trips.emplace_back(mi + it.row(), it.col(), it.value());
    Eigen::SparseMatrix<double> Ae(m, nn);
    Ae.setFromTriplets(trips.begin(), trips.end());
    Ae.makeCompressed();
    const int* Ao = Ae.outerIndexPtr();
    const int* Ai = Ae.innerIndexPtr();
    const double* Av = Ae.valuePtr();

    cs->N_sparse = Ae * Ae.transpose();
    for (int i = 0; i < m; ++i) cs->N_sparse.coeffRef(i, i) += 1e-10;
    cs->N_sparse = cs->N_sparse.triangularView<Eigen::Lower>();
    cs->N_sparse.makeCompressed();
    cs->sparse_n_nnz = static_cast<int>(cs->N_sparse.nonZeros());

    const int* No = cs->N_sparse.outerIndexPtr();
    const int* Ni = cs->N_sparse.innerIndexPtr();
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      int nz = Ao[j + 1] - Ao[j];
      total += static_cast<size_t>(nz) * nz;
    }
    cs->sparse_scatter.reserve(total);
    cs->sparse_scatter_col_start.resize(n_orig + 1);
    cs->sparse_scatter_col_start[0] = 0;
    for (int j = 0; j < n_orig; ++j) {
      for (int pi = Ao[j]; pi < Ao[j + 1]; ++pi)
        for (int pk = Ao[j]; pk < Ao[j + 1]; ++pk) {
          int r1 = Ai[pi], r2 = Ai[pk];
          const int* pos = std::lower_bound(Ni + No[r2], Ni + No[r2 + 1], r1);
          if (pos != Ni + No[r2 + 1] && *pos == r1)
            cs->sparse_scatter.push_back({static_cast<int>(pos - Ni), Av[pi] * Av[pk]});
        }
      cs->sparse_scatter_col_start[j + 1] = static_cast<int>(cs->sparse_scatter.size());
    }
    cs->sparse_diag_offsets.resize(m);
    for (int i = 0; i < m; ++i) {
      const int* pos = std::lower_bound(Ni + No[i], Ni + No[i + 1], i);
      cs->sparse_diag_offsets[i] = static_cast<int>(pos - Ni);
    }

#if MIPSOLVERS_HAVE_CHOLMOD && !MIPSOLVERS_USE_ACCELERATE
    // CHOLMOD symbolic analysis — once per cache lifetime; numeric
    // factorization re-reads the (per-solve refilled) N_local values, which
    // share this exact pattern.  Priority: Accelerate > CHOLMOD > Eigen
    // (Accelerate measured ~1.9x faster on Apple Silicon).
    cs->cholmod_ok = cs->cholmod.analyze(
        m, cs->N_sparse.outerIndexPtr(), cs->N_sparse.innerIndexPtr(),
        cs->N_sparse.valuePtr(), cs->sparse_n_nnz);
#endif
#if MIPSOLVERS_USE_ACCELERATE
    if (!cs->cholmod_ok) {
    const int* No2 = cs->N_sparse.outerIndexPtr();
    cs->accel_col_starts.resize(m + 1);
    for (int i = 0; i <= m; ++i)
      cs->accel_col_starts[i] = static_cast<long>(No2[i]);

    SparseMatrixStructure s{};
    s.rowCount = m;
    s.columnCount = m;
    s.columnStarts = cs->accel_col_starts.data();
    s.rowIndices = cs->N_sparse.innerIndexPtr();
    s.attributes.transpose = false;
    s.attributes.triangle = SparseLowerTriangle;
    s.attributes.kind = SparseSymmetric;
    s.attributes._reserved = 0;
    s.attributes._allocatedBySparse = false;
    s.blockSize = 1;
    cs->accel_symbolic = SparseFactor(SparseFactorizationCholesky, s);
    cs->accel_symbolic_valid = true;
    }
#else
    if (!cs->cholmod_ok) cs->ldlt.analyzePattern(cs->N_sparse);
#endif
  }

  // Compute initial least-squares dual: y0 = (A*A^T + reg*I)^{-1} * A*c.
  // One factorize+solve (~8ms) that saves 30+ iterations on every first node LP.
  if (m > 0 && !cs->use_banded) {
    // Fill N with theta=1 (i.e. N = A*A^T + reg*I)
    double* Nv = cs->N_sparse.valuePtr();
    std::memset(Nv, 0, sizeof(double) * cs->sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      for (int si = cs->sparse_scatter_col_start[j]; si < cs->sparse_scatter_col_start[j + 1]; ++si)
        Nv[cs->sparse_scatter[si].ni] += cs->sparse_scatter[si].a_prod;  // theta=1
    }
    for (int i = 0; i < mi; ++i)
      Nv[cs->sparse_diag_offsets[i]] += 1.0;  // slack columns with theta=1
    for (int i = 0; i < m; ++i)
      Nv[cs->sparse_diag_offsets[i]] += 1e-10;  // regularization

    // RHS = A * c (including slack cost = 0)
    cs->last_dual.resize(m, 0.0);
    for (int i = 0; i < mi; ++i) {
      double s = 0.0;
      for (int p = cs->A_rp[i]; p < cs->A_rp[i + 1]; ++p)
        s += cs->A_rv[p] * cs->c[cs->A_ci[p]];
      cs->last_dual[i] = s;
    }
    for (int i = 0; i < me; ++i) {
      double s = 0.0;
      for (int p = cs->Aeq_rp[i]; p < cs->Aeq_rp[i + 1]; ++p)
        s += cs->Aeq_rv[p] * cs->c[cs->Aeq_ci[p]];
      cs->last_dual[mi + i] = s;
    }

    // Factorize and solve
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cs->cholmod_ok) {
      if (cs->cholmod.factorize(cs->N_sparse.valuePtr()))
        cs->cholmod.solve(cs->last_dual.data(), cs->last_dual.data());
    } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
    {
    SparseMatrixStructure ds{};
    ds.rowCount = m;
    ds.columnCount = m;
    ds.columnStarts = cs->accel_col_starts.data();
    ds.rowIndices = cs->N_sparse.innerIndexPtr();
    ds.attributes.transpose = false;
    ds.attributes.triangle = SparseLowerTriangle;
    ds.attributes.kind = SparseSymmetric;
    ds.attributes._reserved = 0;
    ds.attributes._allocatedBySparse = false;
    ds.blockSize = 1;
    SparseMatrix_Double apple_N{};
    apple_N.structure = ds;
    apple_N.data = cs->N_sparse.valuePtr();
    auto num_f = SparseFactor(cs->accel_symbolic, apple_N);
    if (num_f.status == SparseStatusOK) {
      DenseVector_Double dv{};
      dv.count = m;
      dv.data = cs->last_dual.data();
      SparseSolve(num_f, dv);
      SparseCleanup(num_f);
    }
    }
#else
    {
    cs->ldlt.factorize(cs->N_sparse);
    if (cs->ldlt.info() == Eigen::Success) {
      Eigen::Map<Eigen::VectorXd> y_map(cs->last_dual.data(), m);
      y_map = cs->ldlt.solve(y_map.eval());
    }
    }
#endif
  }

  cached_state_ = std::move(cs);
}

void NativeIPMLPAdapter::update_cached_cost(const Eigen::VectorXd& new_c) {
  if (!cached_state_ || cached_state_->n_orig == 0) return;
  auto& cs = *cached_state_;
  const int n_orig = cs.n_orig;
  // new_c is in the original LP convention; the cache stores Dc-scaled cost.
  for (int j = 0; j < n_orig && j < static_cast<int>(new_c.size()); ++j)
    cs.c[j] = cs.sense_sign * new_c[j] *
              (cs.scaling_active ? cs.scal_dc[static_cast<size_t>(j)] : 1.0);
  // Slack costs (indices n_orig..nn-1) remain 0.
}

SolveResult NativeIPMLPAdapter::solve_cached_node_lp(
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x0) const {
  SolveResult out;
  out.stats.solver_name = name();
  const auto t0 = std::chrono::steady_clock::now();

  if (!cached_state_ || cached_state_->n_orig == 0) {
    out.stats.status = "NoCachedState";
    if (opt_.verbose)
      fprintf(stderr, "[IPM-CACHE] solve_cached: NO cached state!\n");
    return out;
  }

  if (opt_.verbose)
    fprintf(stderr, "[IPM-CACHE] solve_cached: n_orig=%d m=%d bw=%d %s ws=%d\n",
            cached_state_->n_orig, cached_state_->m, cached_state_->bandwidth,
            cached_state_->use_banded ? "BANDED" : "SPARSE",
            (int)(x0.size() == cached_state_->n_orig));

  const auto& cs = *cached_state_;
  const int n_orig = cs.n_orig;
  const int mi = cs.mi;
  const int me = cs.me;
  const int m = cs.m;
  const int nn = cs.nn;
  const int sense_sign = cs.sense_sign;
  const bool has_warm_start = (x0.size() == n_orig);
  const bool use_banded = cs.use_banded;
  const int bw = cs.bandwidth;

  // CSC pointers (from cached copy)
  const int* A_o = cs.A_o;
  const int* A_i = cs.A_i;
  const double* A_v = cs.A_v;
  const int* Aeq_o = cs.Aeq_o;
  const int* Aeq_i = cs.Aeq_i;
  const double* Aeq_v = cs.Aeq_v;

  // Bounds — only thing that changes per node.  Node bounds arrive in
  // original coordinates; the cache holds the Ruiz-scaled problem, so
  // finite bounds are scaled by 1/dc here (flags from original bounds).
  const bool scaling = cs.scaling_active;
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)node_lb.size()) ? node_lb[j] : -kBig;
    double hi = (j < (int)node_ub.size()) ? node_ub[j] : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
    if (scaling) {
      const double inv_d = 1.0 / cs.scal_dc[static_cast<size_t>(j)];
      if (flb[j]) lo *= inv_d;
      if (fub[j]) hi *= inv_d;
    }
    lb[j] = lo; ub[j] = hi;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(cs.base_lp_copy, i);
    // cs.b is stored scaled; scale (b - lhs) by the same dr_i.
    const double dri = scaling ? cs.scal_dr[static_cast<size_t>(i)] : 1.0;
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(cs.base_lp_copy.b[i])
            ? std::max(0.0, dri * (cs.base_lp_copy.b[i] - lhs))
            : kBig;
    flb[n_orig + i] = 1.0;
    fub[n_orig + i] = (ub[n_orig + i] < kBig - 1) ? 1.0 : 0.0;
  }

  // Detect fixed variables (lb ≈ ub) and remove from barrier formulation.
  std::vector<bool> is_fixed(nn, false);
  int n_fixed_vars = 0;
  for (int j = 0; j < n_orig; ++j) {
    if (flb[j] && fub[j] && ub[j] - lb[j] < 1e-9) {
      is_fixed[j] = true;
      ++n_fixed_vars;
      flb[j] = 0.0;
      fub[j] = 0.0;
    }
  }

  // Alias cached data
  const auto& c_vec = cs.c;
  const auto& b_vec = cs.b;
  const auto& A_rp = cs.A_rp;
  const auto& A_ci = cs.A_ci;
  const auto& A_rv = cs.A_rv;
  const auto& Aeq_rp = cs.Aeq_rp;
  const auto& Aeq_ci = cs.Aeq_ci;
  const auto& Aeq_rv = cs.Aeq_rv;

  // SpMV lambdas (use cached CSR/CSC)
  auto ae_mul_sub = [&](const double* MIPSOLVERS_RESTRICT x, double* MIPSOLVERS_RESTRICT y) {
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] -= s;
    }
    for (int k = 0; k < me; ++k) {
      double s = 0.0;
      for (int p = Aeq_rp[k]; p < Aeq_rp[k + 1]; ++p)
        s += Aeq_rv[p] * x[Aeq_ci[p]];
      y[mi + k] -= s;
    }
  };

  auto aet_mul = [&](const double* MIPSOLVERS_RESTRICT w, double* MIPSOLVERS_RESTRICT y) {
    for (int j = 0; j < n_orig; ++j) {
      double s = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p)
        s += A_v[p] * w[A_i[p]];
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p)
          s += Aeq_v[p] * w[mi + Aeq_i[p]];
      y[j] = s;
    }
    for (int i = 0; i < mi; ++i) y[n_orig + i] = w[i];
  };

  auto compute_residuals = [&](const double* MIPSOLVERS_RESTRICT xv, const double* MIPSOLVERS_RESTRICT yv,
                               double* MIPSOLVERS_RESTRICT rp, double* MIPSOLVERS_RESTRICT rd) {
    std::memcpy(rp, b_vec.data(), sizeof(double) * m);
    std::memcpy(rd, c_vec.data(), sizeof(double) * nn);
    for (int j = 0; j < n_orig; ++j) {
      double xj = xv[j];
      double dj = 0.0;
      for (int p = A_o[j]; p < A_o[j + 1]; ++p) {
        int i = A_i[p];
        double aij = A_v[p];
        rp[i] -= aij * xj;
        dj += aij * yv[i];
      }
      if (me > 0)
        for (int p = Aeq_o[j]; p < Aeq_o[j + 1]; ++p) {
          int i = Aeq_i[p];
          double aij = Aeq_v[p];
          rp[mi + i] -= aij * xj;
          dj += aij * yv[mi + i];
        }
      rd[j] -= dj;
    }
    for (int i = 0; i < mi; ++i) {
      rp[i] -= xv[n_orig + i];
      rd[n_orig + i] -= yv[i];
    }
  };

  // === Initialization ===
  std::vector<double> xv(nn), yv(m);
  std::vector<double> gl(nn), gu(nn), zl(nn), zu(nn);
  double* x_d = xv.data();
  double* y_d = yv.data();
  double* gl_d = gl.data();
  double* gu_d = gu.data();
  double* zl_d = zl.data();
  double* zu_d = zu.data();

  {
    if (has_warm_start) {
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -kBig;
        double hi = fub[j] ? ub[j] : kBig;
        // Caller-provided x0 is in original coordinates: x̂0 = x0 / Dc.
        const double x0j =
            scaling ? x0[j] / cs.scal_dc[static_cast<size_t>(j)] : x0[j];
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0j, lo + 1e-6, hi - 1e-6);
        }
      }
    } else {
      for (int j = 0; j < n_orig; ++j) {
        double lo = flb[j] ? lb[j] : -10.0;
        double hi = fub[j] ? ub[j] : 10.0;
        x_d[j] = 0.5 * (lo + hi);
      }
    }

    // Compute inequality slacks: s_i = b_i - A_i * x (must be > 0)
    for (int i = 0; i < mi; ++i) {
      double ax = 0.0;
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        ax += A_rv[p] * x_d[A_ci[p]];
      x_d[n_orig + i] = [&] {
        // Interior slack, finite even for G-type rows (b = +inf) and for
        // inits violating the row's two-sided range (see solve_lp).
        double s_init = b_vec[i] - ax;
        if (!std::isfinite(s_init)) s_init = 1.0;
        return std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
      }();
    }

    // Dual warm-start: reuse previous solve's dual if available.
    // This dramatically reduces iteration count (from ~60 to ~15-20).
    if (static_cast<int>(cs.last_dual.size()) == m) {
      std::memcpy(yv.data(), cs.last_dual.data(), sizeof(double) * m);
    } else {
      std::fill(yv.begin(), yv.end(), 0.0);
    }
  }

  // Lock fixed variables at their exact value before general clamping.
  for (int j = 0; j < n_orig; ++j) {
    if (is_fixed[j]) x_d[j] = lb[j];
  }

  // Clamp original variables to bounds, then recompute slacks.
  for (int j = 0; j < n_orig; ++j) {
    if (is_fixed[j]) continue;  // already locked
    double lo = flb[j] ? lb[j] : (x_d[j] - 10.0);
    double hi = fub[j] ? ub[j] : (x_d[j] + 10.0);
    double range = hi - lo;
    x_d[j] = std::clamp(x_d[j], lo + 0.01 * range, hi - 0.01 * range);
    if (x_d[j] <= lo || x_d[j] >= hi) x_d[j] = 0.5 * (lo + hi);
  }
  for (int i = 0; i < mi; ++i) {
    double ax = 0.0;
    for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
      ax += A_rv[p] * x_d[A_ci[p]];
    x_d[n_orig + i] = [&] {
      // Interior slack, finite even for G-type rows (b = +inf) and for
      // inits violating the row's two-sided range (see solve_lp).
      double s_init = b_vec[i] - ax;
      if (!std::isfinite(s_init)) s_init = 1.0;
      return std::clamp(s_init, 1e-4, std::max(1e-4, ub[n_orig + i] - 1e-4));
    }();
  }
  // Bound slacks and initial multipliers.
  // If we have previous zl/zu, use them (scaled to current gap sizes).
  const bool have_prev_z = (static_cast<int>(cs.last_zl.size()) == nn &&
                            static_cast<int>(cs.last_zu.size()) == nn);
  for (int j = 0; j < nn; ++j) {
    gl_d[j] = flb[j] ? std::max(x_d[j] - lb[j], 1e-4) : kBig;
    gu_d[j] = fub[j] ? std::max(ub[j] - x_d[j], 1e-4) : kBig;
    if (have_prev_z) {
      // Scale previous multiplier to maintain approx complementarity:
      // zl * gl ≈ mu_target. Use zl_prev but cap to prevent extreme values.
      zl_d[j] = flb[j] ? std::clamp(cs.last_zl[j], 1e-6, 1e4) : 0.0;
      zu_d[j] = fub[j] ? std::clamp(cs.last_zu[j], 1e-6, 1e4) : 0.0;
    } else {
      zl_d[j] = flb[j] ? 1.0 : 0.0;
      zu_d[j] = fub[j] ? 1.0 : 0.0;
    }
  }

  // === IPM iteration (same loop as solve_lp but using cached structures) ===
  std::vector<double> theta(nn), dx(nn), dy(m), dzl(nn), dzu(nn);
  std::vector<double> dx_aff(nn), dy_aff(m), dzl_aff(nn), dzu_aff(nn);
  std::vector<double> rhs(m), tmp_n(nn), Atdy(nn);
  std::vector<double> r_p(m), r_d(nn);
  std::vector<double> inv_gl(nn), inv_gu(nn);

  double* theta_d = theta.data();
  double* dx_d = dx.data();
  double* dy_d = dy.data();
  double* dx_aff_d = dx_aff.data();
  double* dy_aff_d = dy_aff.data();
  double* dzl_aff_d = dzl_aff.data();
  double* dzu_aff_d = dzu_aff.data();
  double* rhs_d = rhs.data();
  double* tmp_d = tmp_n.data();
  double* Atdy_d = Atdy.data();
  double* r_p_d = r_p.data();
  double* r_d_d = r_d.data();
  double* inv_gl_d = inv_gl.data();
  double* inv_gu_d = inv_gu.data();
  const double* flb_d = flb.data();
  const double* fub_d = fub.data();
  const double* lb_d = lb.data();
  const double* ub_d = ub.data();

  const double reg = (n_fixed_vars > n_orig / 4) ? 1e-6 : 1e-10;
  const int max_iter = opt_.max_iter;
  bool converged = false;

  // Working copies for Cholesky
  std::vector<double> band_work;
  if (use_banded) band_work.resize(cs.band_storage.size());

  // Mutable copy of band_storage for fill
  std::vector<double> band_storage_local;
  if (use_banded) band_storage_local.resize(cs.band_storage.size());

  // For sparse path, we need mutable N_sparse values
  Eigen::SparseMatrix<double> N_local;
#if MIPSOLVERS_USE_ACCELERATE
  std::vector<long> accel_col_starts_local;
  SparseOpaqueFactorization_Double accel_numeric{};
  bool accel_numeric_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt_local;
#endif
  if (!use_banded) {
    N_local = cs.N_sparse;  // copy structure + values (values will be overwritten)
#if MIPSOLVERS_USE_ACCELERATE
    accel_col_starts_local = cs.accel_col_starts;
#else
    ldlt_local.analyzePattern(N_local);
#endif
  }

  // Fill + factor lambdas using cached scatter maps
  auto fill_and_factor_banded = [&]() -> bool {
    double* bs = band_storage_local.data();
    std::memset(bs, 0, sizeof(double) * band_storage_local.size());
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = cs.band_scatter_col_start[j]; si < cs.band_scatter_col_start[j + 1]; ++si)
        bs[cs.band_scatter[si].offset] += th * cs.band_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i) bs[i] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) bs[i] += reg;
    std::memcpy(band_work.data(), bs, sizeof(double) * band_storage_local.size());
    return banded_chol_factor(band_work.data(), m, bw);
  };

  auto fill_and_factor_sparse = [&]() -> bool {
    double* Nv = N_local.valuePtr();
    std::memset(Nv, 0, sizeof(double) * cs.sparse_n_nnz);
    for (int j = 0; j < n_orig; ++j) {
      const double th = theta_d[j];
      for (int si = cs.sparse_scatter_col_start[j]; si < cs.sparse_scatter_col_start[j + 1]; ++si)
        Nv[cs.sparse_scatter[si].ni] += th * cs.sparse_scatter[si].a_prod;
    }
    for (int i = 0; i < mi; ++i) Nv[cs.sparse_diag_offsets[i]] += theta_d[n_orig + i];
    for (int i = 0; i < m; ++i) Nv[cs.sparse_diag_offsets[i]] += reg;
#if MIPSOLVERS_HAVE_CHOLMOD
    if (cs.cholmod_ok) {
      return cs.cholmod.factorize(N_local.valuePtr());
    }
#endif
#if MIPSOLVERS_USE_ACCELERATE
    SparseMatrixStructure s_struct{};
    s_struct.rowCount = m;
    s_struct.columnCount = m;
    s_struct.columnStarts = accel_col_starts_local.data();
    s_struct.rowIndices = N_local.innerIndexPtr();
    s_struct.attributes.transpose = false;
    s_struct.attributes.triangle = SparseLowerTriangle;
    s_struct.attributes.kind = SparseSymmetric;
    s_struct.attributes._reserved = 0;
    s_struct.attributes._allocatedBySparse = false;
    s_struct.blockSize = 1;
    SparseMatrix_Double apple_N{};
    apple_N.structure = s_struct;
    apple_N.data = N_local.valuePtr();
    if (accel_numeric_valid) {
      SparseRefactor(apple_N, &accel_numeric);
    } else {
      accel_numeric = SparseFactor(cs.accel_symbolic, apple_N);
      accel_numeric_valid = true;
    }
    return accel_numeric.status == SparseStatusOK;
#else
    ldlt_local.factorize(N_local);
    return ldlt_local.info() == Eigen::Success;
#endif
  };

  // Solve normal equations with conditional iterative refinement (same
  // scheme as the non-cached path: residual against the pristine matrix,
  // up to 2 correction solves with the existing factorization).
  std::vector<double> ir_ndy(m), ir_r(m), ir_corr(m);
  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    auto raw_solve = [&](const double* rhs, double* dy) {
      if (use_banded) {
        std::memcpy(dy, rhs, sizeof(double) * m);
        banded_chol_solve(band_work.data(), m, bw, dy);
      } else {
#if MIPSOLVERS_HAVE_CHOLMOD
        if (cs.cholmod_ok) {
          if (!cs.cholmod.solve(rhs, dy)) {
            // Poison with NaN so the finiteness guard aborts the loop.
            std::fill(dy, dy + m, std::numeric_limits<double>::quiet_NaN());
          }
          return;
        }
#endif
#if MIPSOLVERS_USE_ACCELERATE
        std::memcpy(dy, rhs, sizeof(double) * m);
        DenseVector_Double xb{};
        xb.count = m;
        xb.data = dy;
        SparseSolve(accel_numeric, xb);
#else
        Eigen::Map<const Eigen::VectorXd> rhs_map(rhs, m);
        Eigen::Map<Eigen::VectorXd> dy_map(dy, m);
        dy_map = ldlt_local.solve(rhs_map);
#endif
      }
    };
    raw_solve(rhs_buf, dy_buf);
    if (m == 0) return;
    double rhs_norm = 0.0;
    for (int i = 0; i < m; ++i)
      rhs_norm = std::max(rhs_norm, std::abs(rhs_buf[i]));
    const double ir_tol = 1e-12 * std::max(1.0, rhs_norm);
    for (int ref = 0; ref < 2; ++ref) {
      // ndy = N * dy against the pristine matrix
      if (use_banded) {
        banded_sym_matvec(band_storage_local.data(), m, bw, dy_buf, ir_ndy.data());
      } else {
        Eigen::Map<const Eigen::VectorXd> dy_map(dy_buf, m);
        Eigen::Map<Eigen::VectorXd> ndy_map(ir_ndy.data(), m);
        ndy_map = N_local.selfadjointView<Eigen::Lower>() * dy_map;
      }
      double r_norm = 0.0;
      for (int i = 0; i < m; ++i) {
        ir_r[i] = rhs_buf[i] - ir_ndy[i];
        r_norm = std::max(r_norm, std::abs(ir_r[i]));
      }
      if (r_norm <= ir_tol) break;
      raw_solve(ir_r.data(), ir_corr.data());
      for (int i = 0; i < m; ++i) dy_buf[i] += ir_corr[i];
    }
  };

  double last_pfeas = 0.0, last_dfeas = 0.0, last_mu = 0.0;
  for (int iter = 0; iter < max_iter; ++iter) {
    if (opt_.time_limit_sec > 0.0 && std::isfinite(opt_.time_limit_sec)) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= opt_.time_limit_sec) {
        out.stats.success = false;
        out.stats.iterations = iter;
        out.stats.status = "Time limit";
        out.stats.primal_feas = last_pfeas;
        out.stats.dual_feas = last_dfeas;
        out.stats.complementarity = last_mu;
        break;
      }
    }
    // Residuals
    compute_residuals(x_d, y_d, r_p_d, r_d_d);

    int n_compl = 0;
    double comp_sum = 0.0, pfeas = 0.0, dfeas = 0.0;
    for (int j = 0; j < nn; ++j) {
      r_d_d[j] -= flb_d[j] * zl_d[j];
      r_d_d[j] += fub_d[j] * zu_d[j];
      comp_sum += flb_d[j] * gl_d[j] * zl_d[j] + fub_d[j] * gu_d[j] * zu_d[j];
      n_compl += static_cast<int>(flb_d[j]) + static_cast<int>(fub_d[j]);
      double ad = std::abs(r_d_d[j]);
      if (ad > dfeas && !(j < n_orig && is_fixed[j])) dfeas = ad;
    }
    for (int i = 0; i < m; ++i) {
      double ap = std::abs(r_p_d[i]);
      if (ap > pfeas) pfeas = ap;
    }
    double mu = (n_compl > 0) ? comp_sum / n_compl : 0.0;
    last_pfeas = pfeas; last_dfeas = dfeas; last_mu = mu;

    if (pfeas < opt_.tol_primal && dfeas < opt_.tol_dual && mu < opt_.tol_gap) {
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.primal_feas = pfeas;
      out.stats.dual_feas = dfeas;
      out.stats.complementarity = mu;
      out.stats.status = "Optimal";
      converged = true;
      break;
    }

    // Theta + factorize
    for (int j = 0; j < nn; ++j) {
      if (j < n_orig && is_fixed[j]) {
        inv_gl_d[j] = 0.0;
        inv_gu_d[j] = 0.0;
        theta_d[j] = 0.0;  // remove fixed var from normal equations
        continue;
      }
      double igl = flb_d[j] / gl_d[j];
      double igu = fub_d[j] / gu_d[j];
      inv_gl_d[j] = igl;
      inv_gu_d[j] = igu;
      double d = zl_d[j] * igl + zu_d[j] * igu;
      theta_d[j] = (d > kMinVal) ? 1.0 / d : 1e14;
    }

    bool factor_ok = use_banded ? fill_and_factor_banded() : fill_and_factor_sparse();
    if (!factor_ok) {
      // Dynamic regularization: retry with progressively larger reg
      double dyn_reg = std::max(reg * 1e4, 1e-6);
      for (int retry = 0; retry < 5 && !factor_ok; ++retry) {
        // Add extra regularization to diagonal
        if (use_banded) {
          // Restore the pristine matrix before perturbing (same fix as the
          // non-cached path): band_work was left partially factorized by
          // the failed attempt; band_storage_local holds the fresh fill.
          std::memcpy(band_work.data(), band_storage_local.data(),
                      sizeof(double) * band_storage_local.size());
          double* bs = band_work.data();
          for (int i = 0; i < m; ++i) bs[i] += dyn_reg;
          factor_ok = banded_chol_factor(band_work.data(), m, bw);
        } else {
          double* Nv = N_local.valuePtr();
          for (int i = 0; i < m; ++i) Nv[cs.sparse_diag_offsets[i]] += dyn_reg;
#if MIPSOLVERS_HAVE_CHOLMOD
          if (cs.cholmod_ok) {
            factor_ok = cs.cholmod.factorize(N_local.valuePtr());
          } else
#endif
#if MIPSOLVERS_USE_ACCELERATE
          {
          SparseMatrixStructure s_struct{};
          s_struct.rowCount = m;
          s_struct.columnCount = m;
          s_struct.columnStarts = accel_col_starts_local.data();
          s_struct.rowIndices = N_local.innerIndexPtr();
          s_struct.attributes.transpose = false;
          s_struct.attributes.triangle = SparseLowerTriangle;
          s_struct.blockSize = 1;
          SparseMatrix_Double s_mat{s_struct, N_local.valuePtr()};
          if (accel_numeric_valid) SparseCleanup(accel_numeric);
          accel_numeric = SparseFactor(cs.accel_symbolic, s_mat);
          accel_numeric_valid = (accel_numeric.status == SparseStatusOK);
          factor_ok = accel_numeric_valid;
          }
#else
          {
          ldlt_local.factorize(N_local);
          factor_ok = (ldlt_local.info() == Eigen::Success);
          }
#endif
        }
        dyn_reg *= 10.0;
      }
      if (!factor_ok) {
        out.stats.status = "Cholesky failed";
        out.stats.iterations = iter;
        break;
      }
    }

    // Predictor
    for (int j = 0; j < nn; ++j) {
      double xi = -r_d_d[j] - flb_d[j] * zl_d[j] + fub_d[j] * zu_d[j];
      tmp_d[j] = theta_d[j] * xi;
    }
    std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
    ae_mul_sub(tmp_d, rhs_d);
    solve_normal(rhs_d, dy_aff_d);
    aet_mul(dy_aff_d, Atdy_d);
    simd_fma(theta_d, Atdy_d, tmp_d, dx_aff_d, nn);

    for (int j = 0; j < nn; ++j) {
      dzl_aff_d[j] = flb_d[j] * (-zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_aff_d[j]);
      dzu_aff_d[j] = fub_d[j] * (-zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_aff_d[j]);
    }

    double ap_aff = 1.0, ad_aff = 1.0;
    for (int j = 0; j < nn; ++j) {
      if (dx_aff_d[j] < -kMinVal && flb_d[j]) {
        double a = -kTau * gl_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dx_aff_d[j] > kMinVal && fub_d[j]) {
        double a = kTau * gu_d[j] / dx_aff_d[j];
        if (a < ap_aff) ap_aff = a;
      }
      if (dzl_aff_d[j] < -kMinVal) {
        double a = -kTau * zl_d[j] / dzl_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
      if (dzu_aff_d[j] < -kMinVal) {
        double a = -kTau * zu_d[j] / dzu_aff_d[j];
        if (a < ad_aff) ad_aff = a;
      }
    }
    ap_aff = std::max(ap_aff, kMinVal);
    ad_aff = std::max(ad_aff, kMinVal);

    double mu_aff = 0.0;
    for (int j = 0; j < nn; ++j) {
      mu_aff += flb_d[j] * (gl_d[j] + ap_aff * dx_aff_d[j]) * (zl_d[j] + ad_aff * dzl_aff_d[j]);
      mu_aff += fub_d[j] * (gu_d[j] - ap_aff * dx_aff_d[j]) * (zu_d[j] + ad_aff * dzu_aff_d[j]);
    }
    mu_aff = (n_compl > 0) ? mu_aff / n_compl : 0.0;
    // n_compl == 0 (no finite bounds anywhere) gives mu = mu_aff = 0, so
    // mu_aff / mu would be 0/0 = NaN.  With no complementarity there is
    // nothing to center — take a pure affine step instead of letting NaN
    // propagate into the corrector rhs and the returned solution.
    double sigma = (mu > 0.0) ? std::min(std::pow(mu_aff / mu, 3.0), 0.5) : 0.0;
    double sigma_mu = sigma * mu;

    double ap, ad;
    const bool skip_corrector = (ap_aff > 0.9 && ad_aff > 0.9 && sigma < 0.02);

    if (skip_corrector) {
      ap = ap_aff;
      ad = ad_aff;
      std::memcpy(dx_d, dx_aff_d, sizeof(double) * nn);
      std::memcpy(dy_d, dy_aff_d, sizeof(double) * m);
      std::memcpy(dzl.data(), dzl_aff_d, sizeof(double) * nn);
      std::memcpy(dzu.data(), dzu_aff_d, sizeof(double) * nn);
    } else {
      // Corrector
      for (int j = 0; j < nn; ++j) {
        double xi = -r_d_d[j];
        xi += flb_d[j] * (sigma_mu * inv_gl_d[j] - zl_d[j] - dzl_aff_d[j] * dx_aff_d[j] * inv_gl_d[j]);
        xi -= fub_d[j] * (sigma_mu * inv_gu_d[j] - zu_d[j] + dzu_aff_d[j] * dx_aff_d[j] * inv_gu_d[j]);
        tmp_d[j] = theta_d[j] * xi;
      }
      std::memcpy(rhs_d, r_p_d, sizeof(double) * m);
      ae_mul_sub(tmp_d, rhs_d);
      solve_normal(rhs_d, dy_d);
      aet_mul(dy_d, Atdy_d);
      simd_fma(theta_d, Atdy_d, tmp_d, dx_d, nn);

      for (int j = 0; j < nn; ++j) {
        dzl[j] = flb_d[j] * ((sigma_mu - dzl_aff_d[j] * dx_aff_d[j]) * inv_gl_d[j] - zl_d[j] - zl_d[j] * inv_gl_d[j] * dx_d[j]);
        dzu[j] = fub_d[j] * ((sigma_mu + dzu_aff_d[j] * dx_aff_d[j]) * inv_gu_d[j] - zu_d[j] + zu_d[j] * inv_gu_d[j] * dx_d[j]);
      }

      ap = 1.0; ad = 1.0;
      for (int j = 0; j < nn; ++j) {
        if (dx_d[j] < -kMinVal && flb_d[j]) {
          double a = -kTau * gl_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dx_d[j] > kMinVal && fub_d[j]) {
          double a = kTau * gu_d[j] / dx_d[j];
          if (a < ap) ap = a;
        }
        if (dzl[j] < -kMinVal) {
          double a = -kTau * zl_d[j] / dzl[j];
          if (a < ad) ad = a;
        }
        if (dzu[j] < -kMinVal) {
          double a = -kTau * zu_d[j] / dzu[j];
          if (a < ad) ad = a;
        }
      }
      ap = std::max(ap, kMinVal);
      ad = std::max(ad, kMinVal);
    }

    // Finiteness guard (same as the non-cached path): abort on a NaN
    // search direction instead of returning a NaN "solution" after
    // MaxIter.  dzl/dzu derive from dx, so dx/dy cover all NaN sources.
    bool step_finite = true;
    for (int j = 0; j < nn; ++j)
      if (!std::isfinite(dx_d[j])) { step_finite = false; break; }
    if (step_finite)
      for (int i = 0; i < m; ++i)
        if (!std::isfinite(dy_d[i])) { step_finite = false; break; }
    if (!step_finite) {
      out.stats.status = "NumericalError";
      out.stats.iterations = iter;
      break;
    }

    // Update
    simd_axpy(ap, dx_d, x_d, nn);
    simd_axpy(ad, dy_d, y_d, m);
    simd_axpy_max(ad, dzl.data(), zl_d, kMinVal, nn);
    simd_axpy_max(ad, dzu.data(), zu_d, kMinVal, nn);
    simd_sub_max(x_d, lb_d, gl_d, kMinVal, nn);
    simd_rsub_max(x_d, ub_d, gu_d, kMinVal, nn);
    // Re-lock fixed variables (prevent drift from numerical arithmetic)
    for (int j = 0; j < n_orig; ++j) {
      if (is_fixed[j]) {
        x_d[j] = lb[j];
        gl_d[j] = kBig;
        gu_d[j] = kBig;
      }
    }
  }

  // Cleanup
#if MIPSOLVERS_USE_ACCELERATE
  if (!use_banded && accel_numeric_valid) SparseCleanup(accel_numeric);
#endif

  if (!converged && out.stats.status.empty()) {
    out.stats.status = "MaxIter";
    out.stats.success = false;
    out.stats.iterations = max_iter;
    out.stats.primal_feas = last_pfeas;
    out.stats.dual_feas = last_dfeas;
    out.stats.complementarity = last_mu;
  }
  out.x.resize(n_orig);
  for (int j = 0; j < n_orig; ++j)
    out.x(j) = scaling ? x_d[j] * cs.scal_dc[static_cast<size_t>(j)] : x_d[j];
  out.stats.objective = cs.base_lp_copy.c.dot(out.x);
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i) {
      // G-type rows were negated to L-type at prepare time: flip back.
      const double row_sign =
          (!cs.flip_row.empty() && i < mi &&
           cs.flip_row[static_cast<size_t>(i)] != 0) ? -1.0 : 1.0;
      out.constraint_duals(i) = row_sign * sense_sign * y_d[i] *
                                (scaling ? cs.scal_dr[static_cast<size_t>(i)] : 1.0);
    }
  }
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = scaling ? zl_d[j] / cs.scal_dc[static_cast<size_t>(j)]
                                 : zl_d[j];
    out.box_dual_ub(j) = scaling ? zu_d[j] / cs.scal_dc[static_cast<size_t>(j)]
                                 : zu_d[j];
  }
  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);

  // Save converged dual for warm-starting subsequent node LPs.
  if (out.stats.success) {
    cs.last_dual.resize(m);
    std::memcpy(cs.last_dual.data(), y_d, sizeof(double) * m);
    cs.last_zl.resize(nn);
    cs.last_zu.resize(nn);
    std::memcpy(cs.last_zl.data(), zl_d, sizeof(double) * nn);
    std::memcpy(cs.last_zu.data(), zu_d, sizeof(double) * nn);
  }

  if (opt_.verbose)
    fprintf(stderr, "[IPM-CACHE] done: iters=%d success=%d time=%.1fms status=%s\n",
            out.stats.iterations, (int)out.stats.success,
            out.stats.runtime_sec * 1000.0, out.stats.status.c_str());
  return out;
}

}  // namespace mipsolvers::engine

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
#include <cstring>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>

#if defined(HACDCPF_HAVE_ACCELERATE) && defined(__APPLE__)
#include <Accelerate/Accelerate.h>
#endif

// ARM NEON SIMD for Apple M-series and ARM64
#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#define USE_NEON 1
#else
#define USE_NEON 0
#endif

namespace mipsolvers::engine {

// Apple Accelerate sparse Cholesky cache — persists across IPM solve calls
// to reuse symbolic factorization when the sparsity pattern is unchanged
// (e.g. repeated node LP solves in B&C with same constraint matrix).
struct AccelSparseCache {
#if defined(HACDCPF_HAVE_ACCELERATE)
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

// =====================================================================
// SIMD-optimized vector operations (ARM NEON)
// =====================================================================

#if USE_NEON

// y[j] = a[j] * b[j] + c[j]  (fused multiply-add)
inline void simd_fma(const double* __restrict__ a, const double* __restrict__ b,
                     const double* __restrict__ c, double* __restrict__ y, int n) {
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
inline void simd_axpy(double a, const double* __restrict__ x, double* __restrict__ y, int n) {
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
inline void simd_axpy_max(double a, const double* __restrict__ x, double* __restrict__ y, 
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
inline void simd_sub_max(const double* __restrict__ a, const double* __restrict__ b,
                         double* __restrict__ y, double minval, int n) {
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
inline void simd_rsub_max(const double* __restrict__ a, const double* __restrict__ b,
                          double* __restrict__ y, double minval, int n) {
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
inline bool banded_chol_factor(double* __restrict__ band, int m, int bw) {
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
inline void banded_chol_solve(const double* __restrict__ band, int m, int bw, double* __restrict__ b) {
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
  const bool has_warm_start = (x0.size() == prob.c.size());
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

  // === Bounds — use double flags (0.0/1.0) for branchless arithmetic ===
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)prob.vars.size()) ? prob.vars[j].lb : -kBig;
    double hi = (j < (int)prob.vars.size()) ? prob.vars[j].ub : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    lb[j] = lo; ub[j] = hi;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(prob, i);
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(prob.b[i])
            ? std::max(0.0, prob.b[i] - lhs)
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
  for (int j = 0; j < n_orig; ++j) c[j] = sense_sign * prob.c(j);

  std::vector<double> b(m);
  for (int i = 0; i < mi; ++i) b[i] = prob.b(i);
  for (int i = 0; i < me; ++i) b[mi + i] = prob.beq(i);

  // CSC pointers — direct from Eigen sparse
  const int* A_o = prob.A.outerIndexPtr();
  const int* A_i = prob.A.innerIndexPtr();
  const double* A_v = prob.A.valuePtr();
  const int* Aeq_o = me > 0 ? prob.Aeq.outerIndexPtr() : nullptr;
  const int* Aeq_i = me > 0 ? prob.Aeq.innerIndexPtr() : nullptr;
  const double* Aeq_v = me > 0 ? prob.Aeq.valuePtr() : nullptr;

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
  if (mi > 0) build_csr(prob.A, A_rp, A_ci, A_rv);
  if (me > 0) build_csr(prob.Aeq, Aeq_rp, Aeq_ci, Aeq_rv);

  // === SpMV operations ===
  // y -= Ae * x  (CSR-based: sequential y writes)
  auto ae_mul_sub = [&](const double* __restrict__ x, double* __restrict__ y) {
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

  // y = Ae' * w  (CSC-based: gather from w, natural for CSC)
  auto aet_mul = [&](const double* __restrict__ w, double* __restrict__ y) {
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
  auto compute_residuals = [&](const double* __restrict__ xv, const double* __restrict__ yv,
                               double* __restrict__ rp, double* __restrict__ rd) {
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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
    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      int nz_a = A_o[j + 1] - A_o[j];
      int nz_eq = (me > 0) ? (Aeq_o[j + 1] - Aeq_o[j]) : 0;
      int nz = nz_a + nz_eq;
      total += static_cast<size_t>(nz) * nz;
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
      use_dense = (estimated_scatter > kDenseScatterThreshold);
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
#if !defined(HACDCPF_HAVE_ACCELERATE)
      ldlt.analyzePattern(N_sparse);
#endif
    }
  }

#if defined(HACDCPF_HAVE_ACCELERATE)
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

  if (!use_banded && !use_dense) {
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
  auto ae_mul = [&](const double* __restrict__ x, double* __restrict__ y) {
    for (int i = 0; i < mi; ++i) {
      double s = x[n_orig + i];
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        s += A_rv[p] * x[A_ci[p]];
      y[i] = s;
    }
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
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0[j], lo + 1e-6, hi - 1e-6);
        }
      }
      // Compute inequality slacks: s_i = b_i - A_i * x  (must be > 0)
      for (int i = 0; i < mi; ++i) {
        double ax = 0.0;
        for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
          ax += A_rv[p] * x_d[A_ci[p]];
        x_d[n_orig + i] = std::max(b[i] - ax, 1e-4);
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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
#else
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
#endif
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
    // Recompute inequality slack variables: s_i = b_i - A_i * x (must be > 0)
    for (int i = 0; i < mi; ++i) {
      double ax = 0.0;
      for (int p = A_rp[i]; p < A_rp[i + 1]; ++p)
        ax += A_rv[p] * x_d[A_ci[p]];
      x_d[n_orig + i] = std::max(b[i] - ax, 1e-4);
    }
    for (int j = 0; j < nn; ++j) {
      gl_d[j] = flb[j] ? std::max(x_d[j] - lb[j], 1e-4) : kBig;
      gu_d[j] = fub[j] ? std::max(ub[j] - x_d[j], 1e-4) : kBig;
      zl_d[j] = flb[j] ? 1.0 : 0.0;
      zu_d[j] = fub[j] ? 1.0 : 0.0;
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

  const double reg = (n_fixed_vars > n_orig / 4) ? 1e-6 : 1e-10;
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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

  // Lambda: solve normal equations
  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    if (use_banded) {
      std::memcpy(dy_buf, rhs_buf, sizeof(double) * m);
      banded_chol_solve(band_work.data(), m, bw, dy_buf);
    } else if (use_dense) {
      Eigen::Map<const Eigen::VectorXd> rhs_map(rhs_buf, m);
      Eigen::Map<Eigen::VectorXd> dy_map(dy_buf, m);
      dy_map = ldlt_dense.solve(rhs_map);
    } else {
#if defined(HACDCPF_HAVE_ACCELERATE)
      std::memcpy(dy_buf, rhs_buf, sizeof(double) * m);
      DenseVector_Double xb{};
      xb.count = m;
      xb.data = dy_buf;
      SparseSolve(accel_numeric, xb);
#else
      Eigen::Map<Eigen::VectorXd> rhs_map(rhs_buf, m);
      Eigen::Map<Eigen::VectorXd> dy_map(dy_buf, m);
      dy_map = ldlt.solve(rhs_map);
#endif
    }
  };

  for (int iter = 0; iter < max_iter; ++iter) {
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

    if (opt_.verbose && (iter < 3 || iter % 5 == 0))
      printf("IPM-LP %3d: pf=%.2e df=%.2e mu=%.2e\n", iter, pfeas, dfeas, mu);

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
      theta_d[j] = (d > kMinVal) ? 1.0 / d : 1e14;
    }

    bool factor_ok = use_banded ? fill_and_factor_banded()
                                : (use_dense ? fill_and_factor_dense() : fill_and_factor_sparse());
    // Dynamic regularization retry: if Cholesky fails (ill-conditioned normal
    // equations from near-parallel constraints), increase diagonal perturbation.
    if (!factor_ok) {
      double dyn_reg = std::max(reg * 1e4, 1e-8);
      for (int retry = 0; retry < 4 && !factor_ok; ++retry) {
        if (use_banded) {
          // Refill banded from theta, then add dynamic reg to diagonal
          fill_and_factor_banded();  // fills band_storage + copies to band_work
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
#if defined(HACDCPF_HAVE_ACCELERATE)
          SparseMatrix_Double apple_N = make_apple_matrix();
          if (accel_numeric_valid) {
            SparseRefactor(apple_N, &accel_numeric);
          } else {
            accel_numeric = SparseFactor(accel_symbolic, apple_N);
            accel_numeric_valid = true;
          }
          factor_ok = (accel_numeric.status == SparseStatusOK);
#else
          ldlt.factorize(N_sparse);
          factor_ok = (ldlt.info() == Eigen::Success);
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
    double sigma = std::min(std::pow(mu_aff / mu, 3.0), 0.5);
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
#if defined(HACDCPF_HAVE_ACCELERATE)
  if (!use_banded && !use_dense && accel_numeric_valid) {
    SparseCleanup(accel_numeric);
  }
#endif
  if (!converged && out.stats.status.empty()) {
    out.stats.status = "MaxIter";
    out.stats.success = false;
  }
  out.x.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) out.x(j) = x_d[j];
  out.stats.objective = prob.c.dot(out.x);
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i)
      out.constraint_duals(i) = sense_sign * y_d[i];
  }
  // Export bound multipliers for original variables (used by IPM→simplex crossover).
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = zl_d[j];
    out.box_dual_ub(j) = zu_d[j];
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

#if defined(HACDCPF_HAVE_ACCELERATE)
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

  ~CachedState() {
#if defined(HACDCPF_HAVE_ACCELERATE)
    if (accel_symbolic_valid) SparseCleanup(accel_symbolic);
#endif
  }
};

void NativeIPMLPAdapter::prepare_for_node_solves(const LPModel& base_lp) {
  auto cs = std::make_unique<CachedState>();

  // Keep a copy so CSC pointers remain valid
  cs->base_lp_copy = base_lp;
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

  // Cost
  cs->c.resize(nn, 0.0);
  for (int j = 0; j < n_orig; ++j) cs->c[j] = cs->sense_sign * lp.c(j);

  // RHS
  cs->b.resize(m);
  for (int i = 0; i < mi; ++i) cs->b[i] = lp.b(i);
  for (int i = 0; i < me; ++i) cs->b[mi + i] = lp.beq(i);

  // CSC pointers from our copy
  cs->A_o = lp.A.outerIndexPtr();
  cs->A_i = lp.A.innerIndexPtr();
  cs->A_v = lp.A.valuePtr();
  cs->Aeq_o = me > 0 ? lp.Aeq.outerIndexPtr() : nullptr;
  cs->Aeq_i = me > 0 ? lp.Aeq.innerIndexPtr() : nullptr;
  cs->Aeq_v = me > 0 ? lp.Aeq.valuePtr() : nullptr;

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
  if (mi > 0) build_csr(lp.A, cs->A_rp, cs->A_ci, cs->A_rv);
  if (me > 0) build_csr(lp.Aeq, cs->Aeq_rp, cs->Aeq_ci, cs->Aeq_rv);

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

    size_t total = 0;
    for (int j = 0; j < n_orig; ++j) {
      int nz_a = cs->A_o[j + 1] - cs->A_o[j];
      int nz_eq = (me > 0) ? (cs->Aeq_o[j + 1] - cs->Aeq_o[j]) : 0;
      int nz = nz_a + nz_eq;
      total += static_cast<size_t>(nz) * nz;
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
    // Sparse path: build Ae, N_sparse, scatter maps, symbolic factorization
    using T = Eigen::Triplet<double>;
    std::vector<T> trips;
    trips.reserve(lp.A.nonZeros() + lp.Aeq.nonZeros() + mi);
    for (int k = 0; k < lp.A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, k); it; ++it)
        trips.emplace_back(it.row(), it.col(), it.value());
    for (int i = 0; i < mi; ++i)
      trips.emplace_back(i, n_orig + i, 1.0);
    for (int k = 0; k < lp.Aeq.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, k); it; ++it)
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

#if defined(HACDCPF_HAVE_ACCELERATE)
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
#else
    cs->ldlt.analyzePattern(cs->N_sparse);
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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
#else
    cs->ldlt.factorize(cs->N_sparse);
    if (cs->ldlt.info() == Eigen::Success) {
      Eigen::Map<Eigen::VectorXd> y_map(cs->last_dual.data(), m);
      y_map = cs->ldlt.solve(y_map.eval());
    }
#endif
  }

  cached_state_ = std::move(cs);
}

void NativeIPMLPAdapter::update_cached_cost(const Eigen::VectorXd& new_c) {
  if (!cached_state_ || cached_state_->n_orig == 0) return;
  auto& cs = *cached_state_;
  const int n_orig = cs.n_orig;
  for (int j = 0; j < n_orig && j < static_cast<int>(new_c.size()); ++j)
    cs.c[j] = cs.sense_sign * new_c[j];
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

  // Bounds — only thing that changes per node
  std::vector<double> lb(nn), ub(nn), flb(nn), fub(nn);
  for (int j = 0; j < n_orig; ++j) {
    double lo = (j < (int)node_lb.size()) ? node_lb[j] : -kBig;
    double hi = (j < (int)node_ub.size()) ? node_ub[j] : kBig;
    if (lo < -kBig + 1) lo = -kBig;
    if (hi > kBig - 1) hi = kBig;
    lb[j] = lo; ub[j] = hi;
    flb[j] = (lo > -kBig + 1) ? 1.0 : 0.0;
    fub[j] = (hi < kBig - 1) ? 1.0 : 0.0;
  }
  for (int i = 0; i < mi; ++i) {
    lb[n_orig + i] = 0.0;
    const double lhs = lp_row_lhs_or_neg_inf(cs.base_lp_copy, i);
    ub[n_orig + i] =
        std::isfinite(lhs) && std::isfinite(cs.base_lp_copy.b[i])
            ? std::max(0.0, cs.base_lp_copy.b[i] - lhs)
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
  auto ae_mul_sub = [&](const double* __restrict__ x, double* __restrict__ y) {
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

  auto aet_mul = [&](const double* __restrict__ w, double* __restrict__ y) {
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

  auto compute_residuals = [&](const double* __restrict__ xv, const double* __restrict__ yv,
                               double* __restrict__ rp, double* __restrict__ rd) {
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
        if (hi - lo < 2e-6) {
          x_d[j] = 0.5 * (lo + hi);
        } else {
          x_d[j] = std::clamp(x0[j], lo + 1e-6, hi - 1e-6);
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
      x_d[n_orig + i] = std::max(b_vec[i] - ax, 1e-4);
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
    x_d[n_orig + i] = std::max(b_vec[i] - ax, 1e-4);
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
#if defined(HACDCPF_HAVE_ACCELERATE)
  std::vector<long> accel_col_starts_local;
  SparseOpaqueFactorization_Double accel_numeric{};
  bool accel_numeric_valid = false;
#else
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower, Eigen::AMDOrdering<int>> ldlt_local;
#endif
  if (!use_banded) {
    N_local = cs.N_sparse;  // copy structure + values (values will be overwritten)
#if defined(HACDCPF_HAVE_ACCELERATE)
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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

  auto solve_normal = [&](double* rhs_buf, double* dy_buf) {
    if (use_banded) {
      std::memcpy(dy_buf, rhs_buf, sizeof(double) * m);
      banded_chol_solve(band_work.data(), m, bw, dy_buf);
    } else {
#if defined(HACDCPF_HAVE_ACCELERATE)
      std::memcpy(dy_buf, rhs_buf, sizeof(double) * m);
      DenseVector_Double xb{};
      xb.count = m;
      xb.data = dy_buf;
      SparseSolve(accel_numeric, xb);
#else
      Eigen::Map<Eigen::VectorXd> rhs_map(rhs_buf, m);
      Eigen::Map<Eigen::VectorXd> dy_map(dy_buf, m);
      dy_map = ldlt_local.solve(rhs_map);
#endif
    }
  };

  double last_pfeas = 0.0, last_dfeas = 0.0, last_mu = 0.0;
  for (int iter = 0; iter < max_iter; ++iter) {
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
          double* bs = band_work.data();
          for (int i = 0; i < m; ++i) bs[i] += dyn_reg;
          factor_ok = banded_chol_factor(band_work.data(), m, bw);
        } else {
          double* Nv = N_local.valuePtr();
          for (int i = 0; i < m; ++i) Nv[cs.sparse_diag_offsets[i]] += dyn_reg;
#if defined(HACDCPF_HAVE_ACCELERATE)
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
#else
          ldlt_local.factorize(N_local);
          factor_ok = (ldlt_local.info() == Eigen::Success);
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
    double sigma = std::min(std::pow(mu_aff / mu, 3.0), 0.5);
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
#if defined(HACDCPF_HAVE_ACCELERATE)
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
  for (int j = 0; j < n_orig; ++j) out.x(j) = x_d[j];
  out.stats.objective = cs.base_lp_copy.c.dot(out.x);
  if (m > 0) {
    out.constraint_duals.resize(m);
    for (int i = 0; i < m; ++i)
      out.constraint_duals(i) = sense_sign * y_d[i];
  }
  out.box_dual_lb.resize(n_orig);
  out.box_dual_ub.resize(n_orig);
  for (int j = 0; j < n_orig; ++j) {
    out.box_dual_lb(j) = zl_d[j];
    out.box_dual_ub(j) = zu_d[j];
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

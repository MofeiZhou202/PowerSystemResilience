/// @file ipm_lp_solver_internal.hpp
/// @brief Internal shared definitions for the NativeIPMLPAdapter translation units.
///
/// Platform macros, SIMD/banded/Ruiz numeric kernels, AccelSparseCache, and the
/// CachedState pimpl body shared between ipm_lp_solver.cpp (fresh path) and
/// ipm_lp_solver_cached.cpp (cached node path). Do not include outside those TUs.

#pragma once

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>

#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

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

  SparseOpaqueSymbolicFactorization augmented_symbolic{};
  std::vector<long> augmented_col_starts;
  int augmented_cached_dim = 0;
  int augmented_cached_nnz = 0;
  bool augmented_valid = false;

  ~AccelSparseCache() {
    if (valid) SparseCleanup(symbolic);
    if (augmented_valid) SparseCleanup(augmented_symbolic);
  }
#endif
};

#if MIPSOLVERS_USE_ACCELERATE
// Fill-reducing ordering for the Apple Sparse symbolic factorization, selected
// once by MIPSOLVERS_IPM_ORDER = metis|amd|colamd|default. Nested dissection
// (metis) tends to beat the default on the block-structured SCUC augmented KKT.
inline SparseSymbolicFactorOptions ipm_accel_symbolic_options() {
  SparseSymbolicFactorOptions o{};  // control=0 (default)
  static const SparseOrder_t method = [] {
    const char* e = std::getenv("MIPSOLVERS_IPM_ORDER");
    if (e && std::strcmp(e, "metis") == 0) return SparseOrderMetis;
    if (e && std::strcmp(e, "amd") == 0) return SparseOrderAMD;
    if (e && std::strcmp(e, "colamd") == 0) return SparseOrderCOLAMD;
    return SparseOrderDefault;
  }();
  o.orderMethod = method;
  o.order = nullptr;
  o.ignoreRowsAndColumns = nullptr;
  o.malloc = std::malloc;
  o.free = std::free;
  o.reportError = nullptr;
  return o;
}
#endif

constexpr double kBig = 1e20;
constexpr double kTau = 0.9995;
constexpr double kMinVal = 1e-14;
constexpr int kBandedThreshold = 128;  // use banded Cholesky when bandwidth <= this
constexpr size_t kDenseScatterThreshold = 4'000'000;  // switch to dense BLAS when scatter > 4M
constexpr size_t kDenseMaxBytes = 256 * 1024 * 1024;  // memory gate for the dense path
constexpr size_t kMaxScatterEntries = 100'000'000;  // scatter map cap (~1.6GB)

struct IPMStepLengths {
  double primal{1.0};
  double dual{1.0};
  int primal_index{-1};
  int dual_index{-1};
  // -1 denotes a lower-bound product, +1 an upper-bound product.
  int primal_side{0};
  int dual_side{0};
};

/// Largest strictly interior primal/dual steps for the bounded-variable
/// formulation. `fraction` is either the legacy kTau or 1-epsilon when the
/// result feeds Mehrotra's affine-point and adaptive-step calculations.
inline IPMStepLengths ipm_maximum_step_lengths(
    const double* gl, const double* gu, const double* zl, const double* zu,
    const double* dx, const double* dzl, const double* dzu,
    const double* has_lb, const double* has_ub, int n, double fraction) {
  IPMStepLengths step;
  for (int j = 0; j < n; ++j) {
    if (has_lb[j] && dx[j] < -kMinVal) {
      const double a = -fraction * gl[j] / dx[j];
      if (a < step.primal) {
        step.primal = a;
        step.primal_index = j;
        step.primal_side = -1;
      }
    }
    if (has_ub[j] && dx[j] > kMinVal) {
      const double a = fraction * gu[j] / dx[j];
      if (a < step.primal) {
        step.primal = a;
        step.primal_index = j;
        step.primal_side = 1;
      }
    }
    if (has_lb[j] && dzl[j] < -kMinVal) {
      const double a = -fraction * zl[j] / dzl[j];
      if (a < step.dual) {
        step.dual = a;
        step.dual_index = j;
        step.dual_side = -1;
      }
    }
    if (has_ub[j] && dzu[j] < -kMinVal) {
      const double a = -fraction * zu[j] / dzu[j];
      if (a < step.dual) {
        step.dual = a;
        step.dual_index = j;
        step.dual_side = 1;
      }
    }
  }
  step.primal = std::clamp(step.primal, 0.0, 1.0);
  step.dual = std::clamp(step.dual, 0.0, 1.0);
  return step;
}

/// IPX-style complementarity-buffered step rule.  At the maximum positivity
/// step, define mu_full from the trial products and reserve 0.1*mu_full in the
/// blocking product.  This replaces a fixed fraction-to-boundary constant by
/// a central-path quantity while preserving at least 90% of the maximum step.
inline IPMStepLengths ipm_centrality_step_lengths(
    const double* gl, const double* gu, const double* zl, const double* zu,
    const double* dx, const double* dzl, const double* dzu,
    const double* has_lb, const double* has_ub, int n, int n_compl,
    bool centrality_control) {
  if (!centrality_control) {
    return ipm_maximum_step_lengths(gl, gu, zl, zu, dx, dzl, dzu, has_lb,
                                    has_ub, n, kTau);
  }

  constexpr double kGamma = 0.9;
  const double strict_fraction = 1.0 - std::numeric_limits<double>::epsilon();
  const IPMStepLengths maximum = ipm_maximum_step_lengths(
      gl, gu, zl, zu, dx, dzl, dzu, has_lb, has_ub, n, strict_fraction);
  IPMStepLengths step = maximum;

  double mu_full = 0.0;
  if (n_compl > 0) {
    for (int j = 0; j < n; ++j) {
      if (has_lb[j]) {
        mu_full += (gl[j] + maximum.primal * dx[j]) *
                   (zl[j] + maximum.dual * dzl[j]);
      }
      if (has_ub[j]) {
        mu_full += (gu[j] - maximum.primal * dx[j]) *
                   (zu[j] + maximum.dual * dzu[j]);
      }
    }
    mu_full = std::max(0.0, mu_full / static_cast<double>(n_compl));
  }
  const double product_buffer = (1.0 - kGamma) * mu_full;

  auto buffered_alpha = [&](double value, double direction,
                            double paired_trial, double maximum_alpha) {
    double alpha = maximum_alpha;
    if (paired_trial > 0.0 && std::isfinite(paired_trial)) {
      const double buffer = product_buffer / paired_trial;
      const double candidate = (value - buffer) / (-direction);
      if (std::isfinite(candidate)) alpha = candidate;
    }
    return std::clamp(alpha, kGamma * maximum_alpha, 1.0);
  };

  if (maximum.primal < 1.0 && maximum.primal_index >= 0) {
    const int j = maximum.primal_index;
    if (maximum.primal_side < 0) {
      step.primal = buffered_alpha(
          gl[j], dx[j], zl[j] + maximum.dual * dzl[j], maximum.primal);
    } else {
      step.primal = buffered_alpha(
          gu[j], -dx[j], zu[j] + maximum.dual * dzu[j], maximum.primal);
    }
  }
  if (maximum.dual < 1.0 && maximum.dual_index >= 0) {
    const int j = maximum.dual_index;
    if (maximum.dual_side < 0) {
      step.dual = buffered_alpha(
          zl[j], dzl[j], gl[j] + maximum.primal * dx[j], maximum.dual);
    } else {
      step.dual = buffered_alpha(
          zu[j], dzu[j], gu[j] - maximum.primal * dx[j], maximum.dual);
    }
  }

  // A full Newton step can land exactly on the boundary after rounding.
  step.primal = std::min(step.primal, 1.0 - 1e-6);
  step.dual = std::min(step.dual, 1.0 - 1e-6);
  return step;
}

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

inline RuizScaling ruiz_equilibrate(const Eigen::SparseMatrix<double>& A,
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

}  // namespace mipsolvers::engine

// ════════════════════════════════════════════════════════════════════════════
// dual_simplex.cpp — native dual revised simplex with Forrest-Tomlin LU update
// ════════════════════════════════════════════════════════════════════════════
//
// File organization (line ranges are approximate; use a symbol search to jump):
//
//   Section 1  [~110  – 2350 ]  Anonymous-namespace utilities
//                                  * mark_proven_infeasible, approx_nonnegative
//                                  * RowSpec, extract_solution, basis_matrix
//                                  * primal_simplex_optimize
//                                  * dual_simplex_reoptimize
//
//   Section 2  [~2350 – 3446 ]  NativeLU + incremental state update helpers
//                                  * Extracted L,U factors from UMFPACK
//                                  * Sparse FTRAN / BTRAN with DFS-based
//                                    triangular solves
//                                  * SimplexBoundChange helpers
//
//   Section 3  [~3447 – 4930 ]  SparseBasis
//                                  * Owns the basis matrix + LU factorization
//                                  * Forrest-Tomlin η-vector updates
//                                  * Factor backends: UMFPACK (default),
//                                    HFactorPort, KluRescue (escalation L2)
//                                  * Refactorization bookkeeping
//
//   Section 4  [~4930 – 6570 ]  Sparse simplex drivers
//                                  * sparse_primal_simplex_optimize
//                                  * sparse_dual_simplex_reoptimize
//                                  * HiGHS-style Phase I helpers
//                                    (highs_style_perturb,
//                                     dual_cleanup_resolve)
//
//   Section 5  [~6570 – 6800 ]  Opaque-handle C-style accessors for
//                               SparseBasis (clear_sparse_basis_etas,
//                               truncate_sparse_basis_etas, ...)
//
//   Section 6  [~6800 – end  ]  High-level LP solve entry points
//                                  * solve_lp_with_basis (+ cold-start
//                                    escalation chain wrapper)
//                                  * solve_lp_from_sf
//                                  * dump_solve_lp_counters
//
//   Note: build_standard_form_lp and ruiz_scale_standard_form now live in
//   the sibling TU src/engine/kernel/lp_kernel/dual_simplex_api.cpp.
//
// Future refactor plan (tracked in /memories/repo/forrest-tomlin-update-design.md):
//
//   - Extract SparseBasis + NativeLU + SimplexBoundChange into a private
//     header `include/mipsolvers/engine/detail/dual_simplex_internals.hpp` with
//     external linkage, then split Sections 6-9 (public API) into a sibling
//     TU `src/engine/dual_simplex_api.cpp`. This halves the current TU but
//     requires moving ~640 lines of class implementations and adjusting
//     anonymous-namespace linkage — deferred until the Forrest-Tomlin update
//     stabilization lands to avoid overlapping risky changes.
//
// ════════════════════════════════════════════════════════════════════════════

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/sparse_lu_factor.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/SparseLU>
#include <fmt/format.h>

#include <fmt/format.h>

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#endif

#ifdef MIPSOLVERS_HAVE_UMFPACK
extern "C" {
#include <umfpack.h>
}
#endif

#ifdef MIPSOLVERS_HAVE_KLU
#include <Eigen/KLUSupport>
#endif

#include "mipsolvers/core/logging.hpp"

#if defined(_MSC_VER)
#define MIPSOLVERS_RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
#define MIPSOLVERS_RESTRICT __restrict__
#else
#define MIPSOLVERS_RESTRICT
#endif

#ifndef MIPSOLVERS_ENABLE_FACTOR_BACKEND_B
#define MIPSOLVERS_ENABLE_FACTOR_BACKEND_B 0
#endif

namespace mipsolvers::engine {

SparseFactorTelemetry BasisOps::factor_telemetry() const {
  return {};
}

// Thread-local counter for dual simplex iteration profiling.
thread_local int tl_dual_reopt_iters = 0;
thread_local int tl_primal_simplex_iters = 0;

namespace {

// Thread-local simplex iteration accumulator: every primal/dual reoptimize
// loop bumps this per pivot; solve_lp_with_basis_impl resets it on entry and
// reports it via SolveStats::iterations (previously hard-coded to 0).
thread_local int g_simplex_iter_count = 0;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kHighsDefaultKktTolerance = 1e-7;
constexpr double kHighsDefaultMipTolerance = 1e-6;

bool simplex_wall_time_limit_hit(
    const SimplexOptions& opt,
    const std::chrono::steady_clock::time_point& start) {
  if (!(opt.time_limit_sec > 0.0) || !std::isfinite(opt.time_limit_sec)) {
    return false;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (elapsed < opt.time_limit_sec) return false;
  if (opt.time_limit_hit != nullptr) *opt.time_limit_hit = true;
  return true;
}

void mark_simplex_time_limit(SolveStats& stats,
                             const std::chrono::steady_clock::time_point& start) {
  stats.success = false;
  stats.status = "Time limit";
  stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
}

void mark_proven_infeasible(SolveStats& stats) {
  stats.success = false;
  stats.status = "LP infeasible";
  stats.has_farkas_certificate = true;
}

/// Extract a Farkas infeasibility certificate from a Phase-I terminal basis.
///
/// After Phase I terminates with artificials in the basis (proving infeasibility),
/// the dual variables y = c_B^T * B^{-1} form a Farkas ray:
///   y^T A >= 0  for all original columns,  y^T b < 0.
/// We map back from standard-form rows to the original LP rows.
void extract_farkas_certificate(const StandardFormLP& sf,
                                const std::vector<int>& basis,
                                const Eigen::MatrixXd& binv,
                                SolveStats& stats) {
  const int m = sf.A.rows();
  // m_ineq is tricky — use row_to_slack + row_to_surplus + row_to_artificial to infer row count.
  // Instead, count rows by type: rows with slack are LE, rows with artificial+surplus are GE,
  // rows with just artificial are EQ.  We just output the full dual vector for all rows.

  // Phase-I objective: c_{art} = -1 for artificial columns. Dual = c_B^T * B^{-1}.
  Eigen::VectorXd phase1_c = Eigen::VectorXd::Zero(sf.A.cols());
  for (int i = 0; i < m; ++i) {
    const int art = sf.row_to_artificial_col[i];
    if (art >= 0) phase1_c[art] = -1.0;
  }
  Eigen::VectorXd c_b(m);
  for (int i = 0; i < m; ++i) c_b[i] = phase1_c[basis[i]];
  Eigen::VectorXd y = binv.transpose() * c_b;

  // Map y back through row_sign: y_orig[i] = row_sign[i] * y[i].
  // Split into inequality and equality parts based on the LP structure.
  // The first m_ineq_count rows come from A (LE rows), the rest from Aeq (EQ rows).
  // We determine m_ineq from n_original's LP structure stored in sf.
  // Simple approach: iterate sf metadata.

  // Count original inequality rows (those not from Aeq).
  // In build_standard_form_lp: first m_ineq rows are from lp.A, next m_eq from lp.Aeq.
  // We can recover m_eq from: rows that have artificial but no surplus are EQ rows.
  int m_eq_count = 0;
  int m_ineq_count = 0;
  for (int i = 0; i < m; ++i) {
    if (sf.row_to_artificial_col[i] >= 0 && sf.row_to_surplus_col[i] < 0) {
      ++m_eq_count;
    } else {
      ++m_ineq_count;
    }
  }

  stats.farkas_ray.resize(m_ineq_count);
  stats.farkas_ray_eq.resize(m_eq_count);
  int ineq_idx = 0, eq_idx = 0;
  for (int i = 0; i < m; ++i) {
    const double yi = sf.row_sign[i] * y[i];
    if (sf.row_to_artificial_col[i] >= 0 && sf.row_to_surplus_col[i] < 0) {
      // EQ row (artificial, no surplus).
      if (eq_idx < m_eq_count) stats.farkas_ray_eq[eq_idx++] = yi;
    } else {
      // LE or GE row.
      if (ineq_idx < m_ineq_count) stats.farkas_ray[ineq_idx++] = yi;
    }
  }

  // N3: Validate the Farkas certificate before accepting infeasibility.
  //
  // A valid Farkas ray y for the standard-form problem must, at minimum, be
  // numerically well-behaved:
  //   (i)  y is finite (no NaN / Inf from broken factorisation);
  //   (ii) ||y||_inf is not pathologically large (else numerical blow-up
  //        from c_B^T B^{-1} with ill-conditioned B).
  // The stricter algebraic checks (b^T y < 0, A^T y >= 0 on originals) are
  // deliberately omitted here because sf.b / sf.A are in the post-Ruiz,
  // post-flip scaled space -- the naive signs would reject legitimate rays
  // and, more importantly, withdrawing has_farkas_certificate erases the
  // conflict-learning signal in bc_branching.cpp, causing measurable tree
  // growth on well-scaled instances.
  //
  // If validation fails we withdraw the certificate; callers that relied on
  // has_farkas_certificate = true will treat the node as "infeasibility
  // unproven" and fall back to L4 deferral rather than pruning it.  This
  // eliminates the narrow slice of false prunes driven by NaN-propagated
  // bases documented in branch_and_cut_framework_complete.tex (bug #4).
  bool cert_valid = true;
  if (!y.allFinite()) {
    cert_valid = false;
  } else if (y.cwiseAbs().maxCoeff() > 1e10) {
    cert_valid = false;  // numerical blow-up in c_B^T B^{-1}
  }
  stats.has_farkas_certificate = cert_valid;
}

struct RowSpec {
  Eigen::VectorXd a;
  double rhs{0.0};
  char sense{'L'};  // L: <=, G: >=, E: =
};

bool approx_nonnegative(const Eigen::VectorXd& v, double tol) {
  return (v.array() >= -tol).all();
}

bool approx_dual_feasible(const Eigen::VectorXd& reduced_costs,
                          const std::vector<int>& basis,
                          const std::vector<char>& can_enter,
                          const std::vector<char>& at_upper,
                          double tol) {
  std::vector<char> is_basic(static_cast<size_t>(reduced_costs.size()), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < reduced_costs.size()) {
      is_basic[static_cast<size_t>(idx)] = 1;
    }
  }
  for (int j = 0; j < reduced_costs.size(); ++j) {
    if (is_basic[static_cast<size_t>(j)] || !can_enter[static_cast<size_t>(j)]) {
      continue;
    }
    if (at_upper[static_cast<size_t>(j)]) {
      // At upper: rc must be ≥ 0 (non-negative).
      if (reduced_costs[j] < -tol) return false;
    } else {
      // At lower: rc must be ≤ 0 (non-positive).
      if (reduced_costs[j] > tol) return false;
    }
  }
  return true;
}

std::vector<char> basis_basic_mask(int n, const std::vector<int>& basis) {
  std::vector<char> is_basic(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }
  return is_basic;
}

int normalize_nonbasic_side_from_reduced_costs(
    const StandardFormLP& sf,
    const std::vector<int>& basis,
    const Eigen::VectorXd& reduced_costs,
    std::vector<char>& at_upper,
    double optimality_tol) {
  const int n = sf.A.cols();
  if (n <= 0 || static_cast<int>(reduced_costs.size()) != n) return 0;
  if (static_cast<int>(at_upper.size()) < n) {
    at_upper.resize(static_cast<std::size_t>(n), 0);
  }
  const std::vector<char> is_basic = basis_basic_mask(n, basis);
  const bool has_ub = sf.var_ub.size() == n;
  int flips = 0;
  for (int j = 0; j < n; ++j) {
    if (is_basic[static_cast<std::size_t>(j)]) {
      if (at_upper[static_cast<std::size_t>(j)] != 0) {
        at_upper[static_cast<std::size_t>(j)] = 0;
        ++flips;
      }
      continue;
    }
    const bool can_be_upper =
        has_ub && std::isfinite(sf.var_ub[j]) &&
        sf.var_ub[j] > std::max(1e-12, 10.0 * optimality_tol);
    const double rc = reduced_costs[j];
    char target = at_upper[static_cast<std::size_t>(j)];
    if (!can_be_upper) {
      target = 0;
    } else if (rc > optimality_tol) {
      target = 1;
    } else if (rc < -optimality_tol) {
      target = 0;
    }
    if (at_upper[static_cast<std::size_t>(j)] != target) {
      at_upper[static_cast<std::size_t>(j)] = target;
      ++flips;
    }
  }
  return flips;
}

int count_original_nonbasic_bad_side(const StandardFormLP& sf,
                                     const std::vector<int>& basis,
                                     const std::vector<char>& at_upper,
                                     const Eigen::VectorXd& reduced_costs,
                                     double tol) {
  const int n = sf.A.cols();
  if (n <= 0 || static_cast<int>(reduced_costs.size()) != n) return 0;
  const std::vector<char> is_basic = basis_basic_mask(n, basis);
  int bad = 0;
  for (int j = 0; j < sf.n_original && j < n; ++j) {
    if (is_basic[static_cast<std::size_t>(j)]) continue;
    const bool up = j < static_cast<int>(at_upper.size()) &&
                    at_upper[static_cast<std::size_t>(j)] != 0;
    const double rc = reduced_costs[j];
    if ((!up && rc > tol) || (up && rc < -tol)) ++bad;
  }
  return bad;
}

Eigen::VectorXd extract_solution(const StandardFormLP& sf, const Eigen::VectorXd& x_std) {
  Eigen::VectorXd x = sf.lb_shift;
  if (sf.col_scale.size() > 0) {
    // x_std is in scaled space; unscale: x_orig[j] = col_scale[j] * x_scaled[j]
    for (int j = 0; j < sf.n_original; ++j)
      x[j] += sf.col_scale[j] * x_std[j];
  } else {
    x += x_std.head(sf.n_original);
  }
  return x;
}

bool lp_basis_trace_enabled() {
  const char* env = std::getenv("MIPSOLVERS_LP_BASIS_TRACE");
  return env != nullptr && env[0] != '\0' && std::string(env) != "0";
}

thread_local bool tl_vendored_highs_sf_backend_enabled = false;

bool vendored_highs_sf_enabled() {
  if (tl_vendored_highs_sf_backend_enabled) return true;
  const char* env = std::getenv("MIPSOLVERS_USE_VENDORED_HIGHS_SF_LP");
  const char* unsafe = std::getenv("MIPSOLVERS_ALLOW_UNSAFE_VENDORED_HIGHS_SF_LP");
  return env != nullptr && env[0] != '\0' && env[0] != '0' &&
         unsafe != nullptr && unsafe[0] == '1';
}

int lp_basis_trace_terms() {
  const char* env = std::getenv("MIPSOLVERS_LP_BASIS_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 24;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 24;
  return static_cast<int>(std::min<long>(value, 200));
}

std::uint64_t lp_basis_hash_combine(std::uint64_t seed,
                                    std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t lp_basis_hash_double(double value) {
  if (std::isinf(value)) return value > 0.0 ? 0x7ff0000000000000ULL
                                            : 0xfff0000000000000ULL;
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  const auto q = static_cast<std::int64_t>(std::llround(value * 1e9));
  return static_cast<std::uint64_t>(q) ^ 0x517cc1b727220a95ULL;
}

double lp_basis_row_scale_or_one(const StandardFormLP& sf, int row) {
  if (row >= 0 && row < sf.row_scale.size()) {
    const double scale = sf.row_scale[row];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double lp_basis_col_scale_or_one(const StandardFormLP& sf, int col) {
  if (col >= 0 && col < sf.col_scale.size()) {
    const double scale = sf.col_scale[col];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double lp_basis_original_value(const StandardFormLP& sf,
                               int col,
                               double scaled_value) {
  if (col < 0) return scaled_value;
  const double col_scale = lp_basis_col_scale_or_one(sf, col);
  if (col >= sf.n_original) return col_scale * scaled_value;
  const double shift =
      col < sf.lb_shift.size() ? sf.lb_shift[col] : 0.0;
  return shift + col_scale * scaled_value;
}

double lp_basis_source_row_coeff(const StandardFormLP& sf, int row, int col) {
  if (row < 0 || row >= sf.A_row.rows() || col < 0 ||
      col >= sf.n_original) {
    return 0.0;
  }
  const double scaled = sf.A_row.coeff(row, col);
  if (std::abs(scaled) <= 1e-15) return 0.0;
  return scaled / (lp_basis_row_scale_or_one(sf, row) *
                   lp_basis_col_scale_or_one(sf, col));
}

double lp_basis_source_row_side(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= sf.A_row.rows()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const int sign = row < static_cast<int>(sf.row_sign.size())
                       ? sf.row_sign[static_cast<std::size_t>(row)]
                       : 1;
  if (sf.row_rhs_value.size() == sf.A_row.rows()) {
    return static_cast<double>(sign) * sf.row_rhs_value[row];
  }
  return sf.b[row] / lp_basis_row_scale_or_one(sf, row);
}

bool lp_basis_logical_row_bounds(const StandardFormLP& sf,
                                 int row,
                                 double& lower,
                                 double& upper) {
  const int m = static_cast<int>(sf.A_row.rows());
  const int n_std = static_cast<int>(sf.A.cols());
  if (row < 0 || row >= m || row >= sf.b.size()) return false;

  lower = -std::numeric_limits<double>::infinity();
  upper = std::numeric_limits<double>::infinity();
  const double side = lp_basis_source_row_side(sf, row);
  if (!std::isfinite(side)) return false;

  auto activity_domain = [&]() {
    double min_activity = 0.0;
    double max_activity = 0.0;
    bool min_finite = true;
    bool max_finite = true;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             sf.A_row, row);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= sf.n_original) continue;
      const double a = lp_basis_source_row_coeff(sf, row, j);
      if (std::abs(a) <= 1e-12) continue;
      const double lb =
          j < sf.lb_shift.size() ? sf.lb_shift[j]
                                 : -std::numeric_limits<double>::infinity();
      const double ub =
          j < sf.var_ub.size()
              ? lp_basis_original_value(sf, j, sf.var_ub[j])
              : std::numeric_limits<double>::infinity();
      if (a >= 0.0) {
        if (std::isfinite(lb) && min_finite) min_activity += a * lb;
        else min_finite = false;
        if (std::isfinite(ub) && max_finite) max_activity += a * ub;
        else max_finite = false;
      } else {
        if (std::isfinite(ub) && min_finite) min_activity += a * ub;
        else min_finite = false;
        if (std::isfinite(lb) && max_finite) max_activity += a * lb;
        else max_finite = false;
      }
    }
    return std::pair<double, double>{
        min_finite ? min_activity : -std::numeric_limits<double>::infinity(),
        max_finite ? max_activity : std::numeric_limits<double>::infinity()};
  };

  const int slack_col =
      row < static_cast<int>(sf.row_to_slack_col.size())
          ? sf.row_to_slack_col[static_cast<std::size_t>(row)]
          : -1;
  const int surplus_col =
      row < static_cast<int>(sf.row_to_surplus_col.size())
          ? sf.row_to_surplus_col[static_cast<std::size_t>(row)]
          : -1;

  if (slack_col >= 0) {
    upper = side;
    if (slack_col < n_std && slack_col < sf.var_ub.size() &&
        std::isfinite(sf.var_ub[slack_col])) {
      const double width =
          sf.var_ub[slack_col] * lp_basis_col_scale_or_one(sf, slack_col);
      if (!std::isfinite(width) || width < -1e-9) return false;
      lower = upper - std::max(0.0, width);
    } else {
      lower = activity_domain().first;
    }
    return std::isfinite(upper);
  }

  if (surplus_col >= 0) {
    lower = side;
    upper = activity_domain().second;
    return std::isfinite(lower);
  }

  lower = side;
  upper = side;
  return std::isfinite(lower);
}

double lp_basis_logical_row_activity(const StandardFormLP& sf,
                                     const Eigen::VectorXd& x_std,
                                     int row) {
  double activity = 0.0;
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
           sf.A_row, row);
       it; ++it) {
    const int j = static_cast<int>(it.col());
    if (j < 0 || j >= sf.n_original || j >= x_std.size()) continue;
    const double a = lp_basis_source_row_coeff(sf, row, j);
    if (std::abs(a) <= 1e-12) continue;
    activity += a * lp_basis_original_value(sf, j, x_std[j]);
  }
  return activity;
}

struct LpBasisColumnInfo {
  std::vector<int> row_for_col;
  std::vector<unsigned char> kind_for_col;
};

enum : unsigned char {
  kLpBasisColOriginal = 0,
  kLpBasisColSlack = 1,
  kLpBasisColSurplus = 2,
  kLpBasisColArtificial = 3,
  kLpBasisColUnknown = 4,
};

const char* lp_basis_col_kind_name(unsigned char kind) {
  switch (kind) {
    case kLpBasisColOriginal:
      return "orig";
    case kLpBasisColSlack:
      return "slack";
    case kLpBasisColSurplus:
      return "surplus";
    case kLpBasisColArtificial:
      return "art";
    default:
      return "unknown";
  }
}

LpBasisColumnInfo lp_basis_build_column_info(const StandardFormLP& sf) {
  LpBasisColumnInfo info;
  const int n = static_cast<int>(sf.A.cols());
  info.row_for_col.assign(static_cast<std::size_t>(n), -1);
  info.kind_for_col.assign(static_cast<std::size_t>(n), kLpBasisColUnknown);
  for (int j = 0; j < std::min(n, sf.n_original); ++j) {
    info.kind_for_col[static_cast<std::size_t>(j)] = kLpBasisColOriginal;
  }
  const int m = static_cast<int>(sf.A.rows());
  for (int row = 0; row < m; ++row) {
    auto mark = [&](const std::vector<int>& map, unsigned char kind) {
      if (row >= static_cast<int>(map.size())) return;
      const int col = map[static_cast<std::size_t>(row)];
      if (col < 0 || col >= n) return;
      info.row_for_col[static_cast<std::size_t>(col)] = row;
      info.kind_for_col[static_cast<std::size_t>(col)] = kind;
    };
    mark(sf.row_to_slack_col, kLpBasisColSlack);
    mark(sf.row_to_surplus_col, kLpBasisColSurplus);
    mark(sf.row_to_artificial_col, kLpBasisColArtificial);
  }
  return info;
}

std::uint64_t lp_basis_semantic_row_hash(const StandardFormLP& sf, int row) {
  double lower = 0.0;
  double upper = 0.0;
  lp_basis_logical_row_bounds(sf, row, lower, upper);
  std::vector<std::pair<int, double>> terms;
  terms.reserve(static_cast<std::size_t>(sf.A_row.outerIndexPtr()[row + 1] -
                                         sf.A_row.outerIndexPtr()[row]));
  double first = 0.0;
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
           sf.A_row, row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col < 0 || col >= sf.n_original) continue;
    const double val = lp_basis_source_row_coeff(sf, row, col);
    if (std::abs(val) <= 1e-12) continue;
    if (first == 0.0) first = val;
    terms.emplace_back(col, val);
  }
  if (first < 0.0) {
    for (auto& term : terms) term.second = -term.second;
    const double old_lower = lower;
    lower = std::isfinite(upper) ? -upper : -upper;
    upper = std::isfinite(old_lower) ? -old_lower : -old_lower;
  }
  std::uint64_t h = 0x4c50424153495331ULL;
  h = lp_basis_hash_combine(h, lp_basis_hash_double(lower));
  h = lp_basis_hash_combine(h, lp_basis_hash_double(upper));
  h = lp_basis_hash_combine(h, static_cast<std::uint64_t>(terms.size()));
  for (const auto& term : terms) {
    h = lp_basis_hash_combine(h, static_cast<std::uint64_t>(term.first));
    h = lp_basis_hash_combine(h, lp_basis_hash_double(term.second));
  }
  return h;
}

void trace_lp_basis_state(const SimplexResult& out,
                          const std::vector<int>& basis,
                          const std::vector<char>& at_upper,
                          const Eigen::VectorXd& x_b,
                          const char* stage,
                          const StandardFormLP* form_override = nullptr) {
  if (!lp_basis_trace_enabled()) return;
  const StandardFormLP& sf = form_override != nullptr ? *form_override : out.form;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const auto col_info = lp_basis_build_column_info(sf);
  const double tol = 1e-8;

  int basic_orig = 0, basic_slack = 0, basic_surplus = 0, basic_art = 0;
  int basic_unknown = 0, invalid = 0, dup_cols = 0;
  int basic_lb = 0, basic_ub = 0, basic_deg = 0, basic_frac = 0;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) {
      ++invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(col)]) ++dup_cols;
    seen[static_cast<std::size_t>(col)] = 1;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal) ++basic_orig;
    else if (kind == kLpBasisColSlack) ++basic_slack;
    else if (kind == kLpBasisColSurplus) ++basic_surplus;
    else if (kind == kLpBasisColArtificial) ++basic_art;
    else ++basic_unknown;

    const double value = i < x_b.size() ? x_b[i] : 0.0;
    if (std::abs(value) <= tol) ++basic_lb;
    const double ub = col < sf.var_ub.size() ? sf.var_ub[col]
                                             : std::numeric_limits<double>::infinity();
    if (std::isfinite(ub) && std::abs(value - ub) <= tol) ++basic_ub;
    if (std::abs(value) <= tol ||
        (std::isfinite(ub) && std::abs(value - ub) <= tol)) {
      ++basic_deg;
    }
    double source_value = value;
    if (kind == kLpBasisColOriginal) {
      source_value = lp_basis_original_value(sf, col, value);
    } else if (kind == kLpBasisColSlack || kind == kLpBasisColSurplus ||
               kind == kLpBasisColArtificial) {
      const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
      source_value = lp_basis_logical_row_activity(sf, out.x_std, row);
    }
    const double frac = std::abs(source_value - std::round(source_value));
    if (std::isfinite(frac) && frac > 1e-8 && frac < 0.5 + 1e-8) {
      ++basic_frac;
    }
  }

  int nonbasic_lower = 0;
  int nonbasic_upper = 0;
  for (int j = 0; j < n && j < static_cast<int>(seen.size()); ++j) {
    if (seen[static_cast<std::size_t>(j)]) continue;
    if (j < static_cast<int>(at_upper.size()) &&
        at_upper[static_cast<std::size_t>(j)]) {
      ++nonbasic_upper;
    } else {
      ++nonbasic_lower;
    }
  }

  struct DuplicateRows {
    int total{0};
    std::vector<int> rows;
    std::vector<int> basic_rows;
    std::vector<int> basic_cols;
  };
  std::unordered_map<std::uint64_t, DuplicateRows> row_classes;
  row_classes.reserve(static_cast<std::size_t>(m * 2 + 1));
  for (int row = 0; row < m; ++row) {
    auto& bucket = row_classes[lp_basis_semantic_row_hash(sf, row)];
    ++bucket.total;
    if (static_cast<int>(bucket.rows.size()) < lp_basis_trace_terms()) {
      bucket.rows.push_back(row);
    }
  }
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) continue;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal || kind == kLpBasisColUnknown) continue;
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    if (row < 0 || row >= m) continue;
    auto it = row_classes.find(lp_basis_semantic_row_hash(sf, row));
    if (it == row_classes.end()) continue;
    if (static_cast<int>(it->second.basic_rows.size()) < lp_basis_trace_terms()) {
      it->second.basic_rows.push_back(i);
      it->second.basic_cols.push_back(col);
    }
  }
  int duplicate_classes = 0;
  int duplicate_rows = 0;
  int duplicate_basic_classes = 0;
  int duplicate_basic_rows = 0;
  fmt::memory_buffer dup_sample;
  int dup_emitted = 0;
  for (const auto& entry : row_classes) {
    const DuplicateRows& bucket = entry.second;
    if (bucket.total <= 1) continue;
    ++duplicate_classes;
    duplicate_rows += bucket.total;
    if (!bucket.basic_rows.empty()) {
      ++duplicate_basic_classes;
      duplicate_basic_rows += static_cast<int>(bucket.basic_rows.size());
    }
    if (dup_emitted < lp_basis_trace_terms() && !bucket.basic_rows.empty()) {
      if (dup_emitted > 0) fmt::format_to(std::back_inserter(dup_sample), ";");
      fmt::format_to(std::back_inserter(dup_sample),
                     "{:016x}:total{}:rows[", entry.first, bucket.total);
      for (std::size_t k = 0; k < bucket.rows.size(); ++k) {
        if (k > 0) fmt::format_to(std::back_inserter(dup_sample), ",");
        fmt::format_to(std::back_inserter(dup_sample), "{}", bucket.rows[k]);
      }
      fmt::format_to(std::back_inserter(dup_sample), "]:basic[");
      for (std::size_t k = 0; k < bucket.basic_rows.size(); ++k) {
        if (k > 0) fmt::format_to(std::back_inserter(dup_sample), ",");
        fmt::format_to(std::back_inserter(dup_sample), "{}->{}",
                       bucket.basic_rows[k], bucket.basic_cols[k]);
      }
      fmt::format_to(std::back_inserter(dup_sample), "]");
      ++dup_emitted;
    }
  }

  fmt::memory_buffer sample;
  int emitted = 0;
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < m && i < static_cast<int>(basis.size()) &&
                  emitted < max_terms;
       ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (emitted > 0) fmt::format_to(std::back_inserter(sample), ";");
    if (col < 0 || col >= n) {
      fmt::format_to(std::back_inserter(sample), "{}:{}:invalid", i, col);
      ++emitted;
      continue;
    }
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    const double basic_value = i < x_b.size() ? x_b[i] : 0.0;
    double source_value = basic_value;
    double lower = 0.0;
    double upper = col < sf.var_ub.size() ? sf.var_ub[col]
                                          : std::numeric_limits<double>::infinity();
    std::uint64_t row_hash = 0;
    if (kind == kLpBasisColOriginal) {
      source_value = lp_basis_original_value(sf, col, basic_value);
      lower = col < sf.lb_shift.size() ? sf.lb_shift[col] : 0.0;
      upper = lp_basis_original_value(sf, col, upper);
    } else if (row >= 0) {
      source_value = lp_basis_logical_row_activity(sf, out.x_std, row);
      lp_basis_logical_row_bounds(sf, row, lower, upper);
      row_hash = lp_basis_semantic_row_hash(sf, row);
    }
    const double frac =
        std::abs(source_value - std::round(source_value));
    fmt::format_to(std::back_inserter(sample),
                   "{}:{}:{}:row{}:xb{:.12g}:src{:.12g}:lb{:.12g}:ub{:.12g}:frac{:.12g}:rh{:016x}",
                   i, col, lp_basis_col_kind_name(kind), row, basic_value,
                   source_value, lower, upper, frac, row_hash);
    ++emitted;
  }

  fmt::print(stderr,
             "[B&C-LPBASIS] stage={} m={} n={} nOrig={} nSlack={} "
             "nSurplus={} nArt={} solvedFromHint={} dualReopt={} "
             "exact={} objMax={:.12g} objMin={:.12g} "
             "basic=orig{}:slack{}:surplus{}:art{}:unknown{} "
             "invalid={} dupCols={} nonbasic=lower{}:upper{} "
             "basicBound=lb{}:ub{}:deg{} frac={} "
             "dupLogical=classes{}:rows{}:basicClasses{}:basicRows{} "
             "dupSample=[{}] sample=[{}]\n",
             stage == nullptr ? "unknown" : stage, m, n, sf.n_original,
             sf.n_slack, sf.n_surplus, sf.n_artificial,
             out.solved_from_hint ? 1 : 0, out.dual_reoptimized ? 1 : 0,
             out.exact_optimal ? 1 : 0, out.max_objective,
             sf.objective_const - out.max_objective,
             basic_orig, basic_slack, basic_surplus, basic_art,
             basic_unknown, invalid, dup_cols, nonbasic_lower,
             nonbasic_upper, basic_lb, basic_ub, basic_deg, basic_frac,
             duplicate_classes, duplicate_rows, duplicate_basic_classes,
             duplicate_basic_rows, fmt::to_string(dup_sample),
             fmt::to_string(sample));
}

void trace_lp_basis_hint_state(const StandardFormLP& sf,
                               const std::vector<int>& basis,
                               const std::vector<char>& at_upper,
                               const char* stage,
                               int call_id,
                               bool hint_match) {
  if (!lp_basis_trace_enabled()) return;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const auto col_info = lp_basis_build_column_info(sf);

  int basic_orig = 0, basic_slack = 0, basic_surplus = 0, basic_art = 0;
  int basic_unknown = 0, invalid = 0, dup_cols = 0;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) {
      ++invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(col)]) ++dup_cols;
    seen[static_cast<std::size_t>(col)] = 1;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal) ++basic_orig;
    else if (kind == kLpBasisColSlack) ++basic_slack;
    else if (kind == kLpBasisColSurplus) ++basic_surplus;
    else if (kind == kLpBasisColArtificial) ++basic_art;
    else ++basic_unknown;
  }

  int nonbasic_lower = 0;
  int nonbasic_upper = 0;
  for (int j = 0; j < n && j < static_cast<int>(seen.size()); ++j) {
    if (seen[static_cast<std::size_t>(j)]) continue;
    if (j < static_cast<int>(at_upper.size()) &&
        at_upper[static_cast<std::size_t>(j)]) {
      ++nonbasic_upper;
    } else {
      ++nonbasic_lower;
    }
  }

  struct DuplicateRows {
    int total{0};
    std::vector<int> rows;
    std::vector<int> basic_rows;
    std::vector<int> basic_cols;
  };
  std::unordered_map<std::uint64_t, DuplicateRows> row_classes;
  row_classes.reserve(static_cast<std::size_t>(m * 2 + 1));
  for (int row = 0; row < m; ++row) {
    auto& bucket = row_classes[lp_basis_semantic_row_hash(sf, row)];
    ++bucket.total;
    if (static_cast<int>(bucket.rows.size()) < lp_basis_trace_terms()) {
      bucket.rows.push_back(row);
    }
  }
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) continue;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal || kind == kLpBasisColUnknown) continue;
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    if (row < 0 || row >= m) continue;
    auto it = row_classes.find(lp_basis_semantic_row_hash(sf, row));
    if (it == row_classes.end()) continue;
    if (static_cast<int>(it->second.basic_rows.size()) < lp_basis_trace_terms()) {
      it->second.basic_rows.push_back(i);
      it->second.basic_cols.push_back(col);
    }
  }

  int duplicate_classes = 0;
  int duplicate_rows = 0;
  int duplicate_basic_classes = 0;
  int duplicate_basic_rows = 0;
  for (const auto& entry : row_classes) {
    const DuplicateRows& bucket = entry.second;
    if (bucket.total <= 1) continue;
    ++duplicate_classes;
    duplicate_rows += bucket.total;
    if (!bucket.basic_rows.empty()) {
      ++duplicate_basic_classes;
      duplicate_basic_rows += static_cast<int>(bucket.basic_rows.size());
    }
  }

  fmt::memory_buffer sample;
  int emitted = 0;
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < m && i < static_cast<int>(basis.size()) &&
                  emitted < max_terms;
       ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (emitted > 0) fmt::format_to(std::back_inserter(sample), ";");
    if (col < 0 || col >= n) {
      fmt::format_to(std::back_inserter(sample), "{}:{}:invalid", i, col);
      ++emitted;
      continue;
    }
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    std::uint64_t row_hash = 0;
    if (row >= 0 && kind != kLpBasisColOriginal) {
      row_hash = lp_basis_semantic_row_hash(sf, row);
    }
    fmt::format_to(std::back_inserter(sample),
                   "{}:{}:{}:row{}:rh{:016x}",
                   i, col, lp_basis_col_kind_name(kind), row, row_hash);
    ++emitted;
  }

  fmt::print(stderr,
             "[B&C-LPBASIS-HINT] stage={} call={} match={} m={} n={} "
             "nOrig={} nSlack={} nSurplus={} nArt={} "
             "basic=orig{}:slack{}:surplus{}:art{}:unknown{} "
             "invalid={} dupCols={} nonbasic=lower{}:upper{} "
             "dupLogical=classes{}:rows{}:basicClasses{}:basicRows{} "
             "sample=[{}]\n",
             stage == nullptr ? "unknown" : stage, call_id, hint_match ? 1 : 0,
             m, n, sf.n_original, sf.n_slack, sf.n_surplus, sf.n_artificial,
             basic_orig, basic_slack, basic_surplus, basic_art, basic_unknown,
             invalid, dup_cols, nonbasic_lower, nonbasic_upper,
             duplicate_classes, duplicate_rows, duplicate_basic_classes,
             duplicate_basic_rows, fmt::to_string(sample));
}

bool root_coldstate_diag_enabled() {
  const char* e = std::getenv("MIPSOLVERS_ROOT_COLDSTATE_DIAG");
  return e && e[0] != '\0' && e[0] != '0';
}

bool simplex_exact_edge_env_enabled() {
  return root_coldstate_diag_enabled() ||
         std::getenv("MIPSOLVERS_LPSTATE_CONFORM") != nullptr ||
         std::getenv("MIPSOLVERS_FRONTIER_CONFORM") != nullptr ||
         std::getenv("MIPSOLVERS_XPOOL_EXACT_DSE") != nullptr ||
         std::getenv("MIPSOLVERS_XPOOL_PARENT_EXACT_DSE") != nullptr;
}

bool simplex_degenerate_frontier_remap_env_enabled() {
  const char* e = std::getenv("MIPSOLVERS_FORCE_DEGENERATE_FRONTIER_REMAP");
  return e != nullptr && e[0] != '\0' && e[0] != '0';
}

void trace_root_coldstate(const StandardFormLP& sf,
                          const std::vector<int>& basis,
                          const std::vector<char>& at_upper,
                          const Eigen::VectorXd* x_b,
                          const Eigen::VectorXd* reduced_costs,
                          const char* stage,
                          int call_id,
                          const char* dse_mode,
                          int phase_iters,
                          bool phase_ok) {
  if (!root_coldstate_diag_enabled()) return;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const auto col_info = lp_basis_build_column_info(sf);

  int basic_orig = 0, basic_slack = 0, basic_surplus = 0, basic_art = 0;
  int basic_unknown = 0, invalid = 0, dup_cols = 0;
  int art_basic_pos = 0, art_basic_zero = 0;
  int basic_lb = 0, basic_ub = 0, basic_deg = 0;
  int deg_orig_int_frontier = 0;
  double min_xb = 0.0, max_ub_viol = 0.0;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  const bool have_xb = x_b != nullptr && x_b->size() == m;
  constexpr double deg_tol = 1e-8;
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) {
      ++invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(col)]) ++dup_cols;
    seen[static_cast<std::size_t>(col)] = 1;
    const unsigned char kind =
        col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal) ++basic_orig;
    else if (kind == kLpBasisColSlack) ++basic_slack;
    else if (kind == kLpBasisColSurplus) ++basic_surplus;
    else if (kind == kLpBasisColArtificial) ++basic_art;
    else ++basic_unknown;

    const double xb = have_xb ? (*x_b)[i] : 0.0;
    min_xb = std::min(min_xb, xb);
    const double ub = col < sf.var_ub.size() ? sf.var_ub[col]
                                             : std::numeric_limits<double>::infinity();
    if (std::abs(xb) <= deg_tol) ++basic_lb;
    if (std::isfinite(ub)) {
      max_ub_viol = std::max(max_ub_viol, xb - ub);
      if (std::abs(xb - ub) <= deg_tol) ++basic_ub;
    }
    if (std::abs(xb) <= deg_tol ||
        (std::isfinite(ub) && std::abs(xb - ub) <= deg_tol)) {
      ++basic_deg;
    }
    if (kind == kLpBasisColArtificial) {
      if (xb > deg_tol) ++art_basic_pos;
      else ++art_basic_zero;
    } else if (kind == kLpBasisColOriginal) {
      const double v = lp_basis_original_value(sf, col, xb);
      const double frac = std::abs(v - std::round(v));
      if (frac > 1e-8 && frac < 0.5 + 1e-8 &&
          (std::abs(xb) <= deg_tol ||
           (std::isfinite(ub) && std::abs(xb - ub) <= deg_tol))) {
        ++deg_orig_int_frontier;
      }
    }
  }

  int nonbasic_lower = 0, nonbasic_upper = 0;
  int nb_zero_rc_lower = 0, nb_zero_rc_upper = 0;
  int nb_bad_side = 0;
  int orig_nonbasic_lower = 0, orig_nonbasic_upper = 0;
  int orig_nb_zero_rc_lower = 0, orig_nb_zero_rc_upper = 0;
  int orig_nb_bad_side = 0;
  int blocked_artificial_lower = 0, blocked_artificial_upper = 0;
  const bool have_rc = reduced_costs != nullptr && reduced_costs->size() == n;
  for (int j = 0; j < n && j < static_cast<int>(seen.size()); ++j) {
    if (seen[static_cast<std::size_t>(j)]) continue;
    const bool up = j < static_cast<int>(at_upper.size()) &&
                    at_upper[static_cast<std::size_t>(j)] != 0;
    const unsigned char kind =
        col_info.kind_for_col[static_cast<std::size_t>(j)];
    if (up) ++nonbasic_upper;
    else ++nonbasic_lower;
    if (kind == kLpBasisColOriginal) {
      if (up) ++orig_nonbasic_upper;
      else ++orig_nonbasic_lower;
    } else if (kind == kLpBasisColArtificial) {
      if (up) ++blocked_artificial_upper;
      else ++blocked_artificial_lower;
    }
    if (have_rc) {
      const double rc = (*reduced_costs)[j];
      if (std::abs(rc) <= 1e-8) {
        if (up) ++nb_zero_rc_upper;
        else ++nb_zero_rc_lower;
        if (kind == kLpBasisColOriginal) {
          if (up) ++orig_nb_zero_rc_upper;
          else ++orig_nb_zero_rc_lower;
        }
      }
      if ((!up && rc > 1e-8) || (up && rc < -1e-8)) {
        ++nb_bad_side;
        if (kind == kLpBasisColOriginal) ++orig_nb_bad_side;
      }
    }
  }

  fmt::memory_buffer sample;
  int emitted = 0;
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < m && i < static_cast<int>(basis.size()) &&
                  emitted < max_terms;
       ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) continue;
    const unsigned char kind =
        col_info.kind_for_col[static_cast<std::size_t>(col)];
    const double xb = have_xb ? (*x_b)[i] : 0.0;
    const double ub = col < sf.var_ub.size() ? sf.var_ub[col]
                                             : std::numeric_limits<double>::infinity();
    const bool deg = std::abs(xb) <= deg_tol ||
                     (std::isfinite(ub) && std::abs(xb - ub) <= deg_tol);
    if (!deg && kind != kLpBasisColArtificial) continue;
    if (emitted > 0) fmt::format_to(std::back_inserter(sample), ";");
    fmt::format_to(std::back_inserter(sample),
                   "r{}:c{}:{}:xb{:.12g}:ub{:.12g}",
                   i, col, lp_basis_col_kind_name(kind), xb, ub);
    ++emitted;
  }

  fmt::print(stderr,
             "[ROOT-COLDSTATE] call={} stage={} ok={} iters={} dse={} "
             "m={} n={} nOrig={} nSlack={} nSurplus={} nArt={} "
             "basic=orig{}:slack{}:surplus{}:art{}:unknown{} "
             "invalid={} dupCols={} nonbasic=lower{}:upper{} "
             "origNonbasic=lower{}:upper{} "
             "nbZeroRc=lower{}:upper{} nbBadSide={} "
             "origNbZeroRc=lower{}:upper{} origNbBadSide={} "
             "blockedArt=lower{}:upper{} "
             "basicBound=lb{}:ub{}:deg{} degOrigIntFrontier={} "
             "artBasic=zero{}:pos{} minXb={:.3e} maxUbViol={:.3e} "
             "sample=[{}]\n",
             call_id, stage == nullptr ? "unknown" : stage,
             phase_ok ? 1 : 0, phase_iters,
             dse_mode == nullptr ? "unknown" : dse_mode,
             m, n, sf.n_original, sf.n_slack, sf.n_surplus, sf.n_artificial,
             basic_orig, basic_slack, basic_surplus, basic_art, basic_unknown,
             invalid, dup_cols, nonbasic_lower, nonbasic_upper,
             orig_nonbasic_lower, orig_nonbasic_upper,
             nb_zero_rc_lower, nb_zero_rc_upper, nb_bad_side,
             orig_nb_zero_rc_lower, orig_nb_zero_rc_upper, orig_nb_bad_side,
             blocked_artificial_lower, blocked_artificial_upper,
             basic_lb, basic_ub, basic_deg, deg_orig_int_frontier,
             art_basic_zero, art_basic_pos, min_xb, max_ub_viol,
             fmt::to_string(sample));
}

void populate_dual_certificate(const StandardFormLP& sf,
                               const std::vector<int>& basis,
                               const std::vector<char>& at_upper,
                               const Eigen::MatrixXd& binv,
                               const std::shared_ptr<BasisOps>& sparse_basis,
                               const Eigen::VectorXd& reduced_costs,
                               SolveResult& result) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const int n_orig = sf.n_original;
  if (m <= 0 || static_cast<int>(basis.size()) != m ||
      static_cast<int>(sf.c_max.size()) != n) {
    return;
  }

  Eigen::VectorXd c_b(m);
  for (int i = 0; i < m; ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    c_b[i] = (col >= 0 && col < n) ? sf.c_max[col] : 0.0;
  }

  Eigen::VectorXd y_scaled;
  if (sparse_basis) {
    y_scaled = sparse_basis_btran(sparse_basis, c_b);
  } else if (binv.rows() == m && binv.cols() == m) {
    y_scaled.noalias() = binv.transpose() * c_b;
  }
  if (y_scaled.size() != m || !y_scaled.allFinite()) return;

  result.constraint_duals.resize(m);
  const bool have_row_scale = sf.row_scale.size() == m;
  for (int i = 0; i < m; ++i) {
    const double row_scale = have_row_scale ? sf.row_scale[i] : 1.0;
    const double row_sign =
        (i < static_cast<int>(sf.row_sign.size()))
            ? static_cast<double>(sf.row_sign[static_cast<std::size_t>(i)])
            : 1.0;
    // Internal simplex solves max c_max^T z in the signed/scaled row space.
    // The public LP convention is effective minimization, matching HiGHS:
    // row upper active => row_dual <= 0, row lower active => row_dual >= 0.
    result.constraint_duals[i] = -row_sign * row_scale * y_scaled[i];
  }

  result.box_dual_lb = Eigen::VectorXd::Zero(n_orig);
  result.box_dual_ub = Eigen::VectorXd::Zero(n_orig);
  const bool have_col_scale = sf.col_scale.size() >= n_orig;
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int col : basis) {
    if (col >= 0 && col < n) is_basic[static_cast<std::size_t>(col)] = 1;
  }
  const double dual_tol = 1e-10;
  for (int j = 0; j < n_orig; ++j) {
    double col_dual = 0.0;
    if (reduced_costs.size() > j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      if (col_scale > 0.0 && std::isfinite(col_scale)) {
        col_dual = -reduced_costs[j] / col_scale;
      }
    }
    if (std::abs(col_dual) <= dual_tol || is_basic[static_cast<std::size_t>(j)]) {
      continue;
    }
    if (col_dual > 0.0) {
      result.box_dual_lb[j] = col_dual;
    } else {
      result.box_dual_ub[j] = -col_dual;
    }
  }

  (void)at_upper;
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
const char* highs_sf_model_status_label(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kOptimal:
      return "Optimal";
    case HighsModelStatus::kInfeasible:
      return "Infeasible";
    case HighsModelStatus::kUnbounded:
      return "Unbounded";
    case HighsModelStatus::kUnboundedOrInfeasible:
      return "UnboundedOrInfeasible";
    case HighsModelStatus::kObjectiveBound:
      return "ObjectiveBound";
    case HighsModelStatus::kIterationLimit:
      return "IterationLimit";
    case HighsModelStatus::kTimeLimit:
      return "TimeLimit";
    default:
      return "Other";
  }
}

bool native_sf_basis_to_highs(const StandardFormLP& sf,
                              const SimplexBasis* basis_hint,
                              HighsBasis& hbasis) {
  if (basis_hint == nullptr) return false;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (basis_hint->rows != m || basis_hint->cols != n ||
      static_cast<int>(basis_hint->index_count()) != m ||
      static_cast<int>(basis_hint->at_upper.size()) < n) {
    return false;
  }

  hbasis.valid = false;
  hbasis.alien = true;
  hbasis.useful = true;
  hbasis.col_status.assign(static_cast<std::size_t>(n),
                           HighsBasisStatus::kLower);
  hbasis.row_status.assign(static_cast<std::size_t>(m),
                           HighsBasisStatus::kBasic);

  for (int j = 0; j < n; ++j) {
    const bool fixed =
        sf.var_ub.size() > j && std::isfinite(sf.var_ub[j]) &&
        std::abs(sf.var_ub[j]) <= 1e-12;
    if (fixed) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kLower;
    } else if (basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kUpper;
    }
  }
  for (int row = 0; row < m; ++row) {
    const int col = basis_hint->basis_indices()[static_cast<std::size_t>(row)];
    if (col < 0 || col >= n) return false;
    hbasis.col_status[static_cast<std::size_t>(col)] =
        HighsBasisStatus::kBasic;
    hbasis.row_status[static_cast<std::size_t>(row)] =
        HighsBasisStatus::kLower;
  }
  return true;
}

int native_sf_col_from_highs_basic(const StandardFormLP& sf,
                                   HighsInt highs_basic) {
  const int n = static_cast<int>(sf.A.cols());
  if (highs_basic >= 0) {
    const int col = static_cast<int>(highs_basic);
    return (col >= 0 && col < n) ? col : -1;
  }

  // HiGHS reports a basic row/logical variable as -(row + 1).  Native's
  // standard-form basis stores the corresponding explicit logical column.
  const int row = static_cast<int>(-highs_basic - 1);
  if (row < 0 || row >= static_cast<int>(sf.A.rows())) return -1;
  auto row_col = [&](const std::vector<int>& map) -> int {
    if (row >= static_cast<int>(map.size())) return -1;
    const int col = map[static_cast<std::size_t>(row)];
    return (col >= 0 && col < n) ? col : -1;
  };
  int col = row_col(sf.row_to_slack_col);
  if (col >= 0) return col;
  col = row_col(sf.row_to_surplus_col);
  if (col >= 0) return col;
  col = row_col(sf.row_to_artificial_col);
  if (col >= 0) return col;
  return -1;
}

bool pass_standard_form_to_highs(Highs& highs, const StandardFormLP& sf) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) is_artificial[static_cast<std::size_t>(art)] = 1;
  }
  std::vector<double> col_cost(static_cast<std::size_t>(n), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(n), 0.0);
  std::vector<double> col_upper(static_cast<std::size_t>(n),
                                std::numeric_limits<double>::infinity());
  for (int j = 0; j < n; ++j) {
    col_cost[static_cast<std::size_t>(j)] = sf.c_max[j];
    if (is_artificial[static_cast<std::size_t>(j)]) {
      // Artificial columns are a native Phase-I device, not true LP
      // variables.  Fix them at zero when delegating the real SF LP to HiGHS.
      col_upper[static_cast<std::size_t>(j)] = 0.0;
    } else if (sf.var_ub.size() > j) {
      col_upper[static_cast<std::size_t>(j)] = sf.var_ub[j];
    }
  }
  std::vector<double> row_lower(static_cast<std::size_t>(m), 0.0);
  std::vector<double> row_upper(static_cast<std::size_t>(m), 0.0);
  for (int i = 0; i < m; ++i) {
    row_lower[static_cast<std::size_t>(i)] = sf.b[i];
    row_upper[static_cast<std::size_t>(i)] = sf.b[i];
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(n + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(sf.A.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < n; ++j) {
    start[static_cast<std::size_t>(j)] =
        static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(n)] =
      static_cast<HighsInt>(index.size());

  const HighsStatus st = highs.passModel(
      static_cast<HighsInt>(n), static_cast<HighsInt>(m),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMaximize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), nullptr);
  return st == HighsStatus::kOk;
}

bool audit_vendored_highs_sf_result(const StandardFormLP& sf,
                                    const SimplexOptions& opt,
                                    const SimplexResult& out,
                                    double& primal_residual,
                                    double& bound_violation,
                                    double& artificial_activity,
                                    double& dual_violation,
                                    double& objective_gap,
                                    double highs_dual_violation) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  primal_residual = std::numeric_limits<double>::infinity();
  bound_violation = std::numeric_limits<double>::infinity();
  artificial_activity = std::numeric_limits<double>::infinity();
  dual_violation = std::numeric_limits<double>::infinity();
  objective_gap = std::numeric_limits<double>::infinity();
  if (out.x_std.size() != n || out.reduced_costs.size() != n ||
      static_cast<int>(out.basis.index_count()) != m ||
      static_cast<int>(out.basis.at_upper.size()) < n) {
    return false;
  }
  if (!out.x_std.allFinite() || !out.reduced_costs.allFinite()) return false;
  if (m > 0) {
    primal_residual = (sf.A * out.x_std - sf.b).cwiseAbs().maxCoeff();
  } else {
    primal_residual = 0.0;
  }
  bound_violation = 0.0;
  for (int j = 0; j < n; ++j) {
    bound_violation = std::max(bound_violation, std::max(0.0, -out.x_std[j]));
    if (sf.var_ub.size() > j && std::isfinite(sf.var_ub[j])) {
      bound_violation =
          std::max(bound_violation, std::max(0.0, out.x_std[j] - sf.var_ub[j]));
    }
  }
  artificial_activity = 0.0;
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) {
      artificial_activity =
          std::max(artificial_activity, std::abs(out.x_std[art]));
    }
  }
  (void)opt;
  const double primal_tol = kHighsDefaultKktTolerance;
  dual_violation = 0.0;
  const std::vector<char> is_basic =
      basis_basic_mask(n, out.basis.basis_indices());
  for (int j = 0; j < n; ++j) {
    if (is_basic[static_cast<std::size_t>(j)]) continue;
    const bool fixed =
        sf.var_ub.size() > j && std::isfinite(sf.var_ub[j]) &&
        std::abs(sf.var_ub[j]) <= primal_tol;
    if (fixed) continue;
    const bool at_up = out.basis.at_upper[static_cast<std::size_t>(j)] != 0;
    if (at_up) {
      dual_violation = std::max(dual_violation, std::max(0.0, -out.reduced_costs[j]));
    } else {
      dual_violation = std::max(dual_violation, std::max(0.0, out.reduced_costs[j]));
    }
  }
  const double computed_max_obj = sf.c_max.dot(out.x_std);
  objective_gap = std::abs(computed_max_obj - out.max_objective);
  const double dual_tol = kHighsDefaultKktTolerance;
  if (std::isfinite(highs_dual_violation) &&
      highs_dual_violation <= dual_tol) {
    dual_violation = std::min(dual_violation, highs_dual_violation);
  }
  const double obj_tol =
      std::max(1e-6, 1e-8 * std::max(1.0, std::abs(out.max_objective)));
  return primal_residual <= primal_tol && bound_violation <= primal_tol &&
         artificial_activity <= primal_tol && dual_violation <= dual_tol &&
         objective_gap <= obj_tol;
}

int inject_basic_degenerate_col_duals_from_highs(
    const StandardFormLP& sf,
    Highs& highs,
    const HighsSolution& sol,
    const std::vector<int>& basis_indices,
    Eigen::VectorXd& reduced_costs,
    double feastol,
    double epsilon) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const int n_orig = sf.n_original;
  if (m <= 0 || n <= 0 || n_orig <= 0 ||
      static_cast<int>(basis_indices.size()) != m ||
      reduced_costs.size() != n ||
      static_cast<int>(sol.col_value.size()) < n ||
      static_cast<int>(sol.col_dual.size()) < n) {
    return 0;
  }

  std::vector<int> basic_row_for_col(static_cast<std::size_t>(n), -1);
  for (int row = 0; row < m; ++row) {
    const int col = basis_indices[static_cast<std::size_t>(row)];
    if (col >= 0 && col < n) basic_row_for_col[static_cast<std::size_t>(col)] = row;
  }

  const bool have_col_scale = sf.col_scale.size() == n;
  const bool have_ub = sf.var_ub.size() == n;
  const double dual_accept_tol = std::max(10.0 * feastol, epsilon);
  std::vector<double> row_ap(static_cast<std::size_t>(n), 0.0);
  int injected = 0;

  for (int col = 0; col < n_orig; ++col) {
    if (col >= static_cast<int>(sf.original_types.size())) break;
    const VarType type = sf.original_types[static_cast<std::size_t>(col)];
    if (type != VarType::Integer && type != VarType::Binary) continue;

    const int basis_row = basic_row_for_col[static_cast<std::size_t>(col)];
    if (basis_row < 0) continue;

    const double lb = 0.0;
    const double ub = have_ub ? sf.var_ub[col] : kInf;
    if (std::isfinite(ub) && ub - lb < feastol) continue;

    const double value = sol.col_value[static_cast<std::size_t>(col)];
    double sign = 0.0;
    if (!std::isfinite(ub) || value - lb < ub - value) {
      if (value > lb + feastol) continue;
      sign = 1.0;
    } else {
      if (value < ub - feastol) continue;
      sign = -1.0;
    }

    std::fill(row_ap.begin(), row_ap.end(), 0.0);
    if (highs.getReducedRow(static_cast<HighsInt>(basis_row),
                            row_ap.data()) != HighsStatus::kOk) {
      continue;
    }

    double degenerate_col_dual = kInf;
    for (int j = 0; j < n; ++j) {
      if (j == col) continue;
      const double other_ub = have_ub ? sf.var_ub[j] : kInf;
      if (std::isfinite(other_ub) && other_ub <= feastol) continue;
      const double val = sign * row_ap[static_cast<std::size_t>(j)];
      if (val > epsilon) {
        if (sol.col_value[static_cast<std::size_t>(j)] > feastol) {
          const double ratio =
              -sol.col_dual[static_cast<std::size_t>(j)] / val;
          if (std::isfinite(ratio)) {
            degenerate_col_dual = std::min(degenerate_col_dual, ratio);
          }
        }
      } else if (val < -epsilon) {
        const bool has_room_to_upper =
            !std::isfinite(other_ub) ||
            other_ub - sol.col_value[static_cast<std::size_t>(j)] > feastol;
        if (has_room_to_upper) {
          const double ratio =
              -sol.col_dual[static_cast<std::size_t>(j)] / val;
          if (std::isfinite(ratio)) {
            degenerate_col_dual = std::min(degenerate_col_dual, ratio);
          }
        }
      }
    }

    if (!std::isfinite(degenerate_col_dual) ||
        degenerate_col_dual <= dual_accept_tol) {
      continue;
    }

    const double effective_col_dual = sign * degenerate_col_dual;
    const double col_scale = have_col_scale ? sf.col_scale[col] : 1.0;
    if (!std::isfinite(effective_col_dual) ||
        !std::isfinite(col_scale) || std::abs(col_scale) <= 1e-18) {
      continue;
    }
    reduced_costs[col] = -effective_col_dual * col_scale;
    ++injected;
  }

  return injected;
}

class VendoredHighsBasis : public BasisOps {
 public:
  VendoredHighsBasis(std::shared_ptr<Highs> highs,
                     int rows,
                     int cols,
                     const Eigen::SparseMatrix<double>* A)
      : highs_(std::move(highs)),
        m_(rows),
        n_(cols),
        A_owned_(A != nullptr ? *A : Eigen::SparseMatrix<double>()) {}

  BasisOpsKind kind() const override { return BasisOpsKind::VendoredHighs; }

  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisTransposeSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override {
    out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || row < 0 || row >= m_) return false;
    std::vector<double> row_vec(static_cast<std::size_t>(m_), 0.0);
    HighsInt row_num_nz = 0;
    std::vector<HighsInt> row_indices(static_cast<std::size_t>(m_), 0);
    const HighsStatus st =
        highs_->getBasisInverseRow(static_cast<HighsInt>(row), row_vec.data(),
                                   &row_num_nz, row_indices.data());
    if (st != HighsStatus::kOk) return false;
    if (row_num_nz < 0 || row_num_nz > m_) return false;
    for (HighsInt k = 0; k < row_num_nz; ++k) {
      const int r = static_cast<int>(row_indices[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      out[r] = row_vec[static_cast<std::size_t>(r)];
    }
    return out.allFinite();
  }

  bool tableau_row(int row, Eigen::RowVectorXd& out) const override {
    if (!highs_ || row < 0 || row >= m_ || n_ <= 0) return false;
    std::vector<double> h_row(static_cast<std::size_t>(n_), 0.0);
    if (highs_->getReducedRow(static_cast<HighsInt>(row), h_row.data()) ==
        HighsStatus::kOk) {
      out.resize(n_);
      for (int j = 0; j < n_; ++j) out[j] = h_row[static_cast<std::size_t>(j)];
      return out.allFinite();
    }
    if (A_owned_.rows() != m_ || A_owned_.cols() != n_) return false;
    Eigen::VectorXd e = Eigen::VectorXd::Zero(m_);
    e[row] = 1.0;
    Eigen::VectorXd y = btran(e);
    if (y.size() != m_ || !y.allFinite()) return false;
    out = y.transpose() * A_owned_;
    return out.allFinite();
  }

  SparseFactorTelemetry factor_telemetry() const override {
    SparseFactorTelemetry t;
    t.backend_id = 100;
    t.ft_backend = false;
    t.ft_valid = true;
    return t;
  }

  void rebind_A(const Eigen::SparseMatrix<double>& A) override { A_owned_ = A; }

  bool bound_to_A(const Eigen::SparseMatrix<double>& A) const override {
    return A.rows() == m_ && A.cols() == n_;
  }

  std::shared_ptr<Highs> highs_handle() const override { return highs_; }

  bool import_optimal_result(const std::shared_ptr<Highs>& highs,
                             const StandardFormLP& sf,
                             const SimplexOptions& opt,
                             bool solved_from_hint,
                             const char* context,
                             SimplexResult& out) {
    if (!highs) return false;
    const int m = static_cast<int>(sf.A.rows());
    const int n = static_cast<int>(sf.A.cols());
    if (m <= 0 || n <= 0) return false;
    const HighsModelStatus model_status = highs->getModelStatus();
    if (model_status != HighsModelStatus::kOptimal) {
      out.result.stats.solver_name = "VendoredHighsLpKernel";
      out.result.stats.success = false;
      out.result.stats.status =
          std::string("HiGHS ") + highs_sf_model_status_label(model_status);
      return false;
    }

    const HighsSolution& sol = highs->getSolution();
    const HighsBasis& basis = highs->getBasis();
    if (!basis.valid || static_cast<int>(basis.col_status.size()) < n ||
        static_cast<int>(basis.row_status.size()) < m ||
        static_cast<int>(sol.col_value.size()) < n) {
      return false;
    }

    out = SimplexResult{};
    out.form = sf;
    out.x_std = Eigen::VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
      out.x_std[j] = sol.col_value[static_cast<std::size_t>(j)];
    }
    out.result.x = extract_solution(sf, out.x_std);
    out.x_basic = Eigen::VectorXd::Zero(m);
    out.basis.rows = m;
    out.basis.cols = n;
    out.basis.indices.assign(static_cast<std::size_t>(m), -1);
    out.basis.at_upper.assign(static_cast<std::size_t>(n), 0);
    out.basis.sf_n_slack = sf.n_slack;
    out.basis.sf_n_surplus = sf.n_surplus;
    out.basis.sf_n_artificial = sf.n_artificial;

    for (int j = 0; j < n; ++j) {
      out.basis.at_upper[static_cast<std::size_t>(j)] =
          basis.col_status[static_cast<std::size_t>(j)] ==
                  HighsBasisStatus::kUpper
              ? 1
              : 0;
    }

    std::vector<HighsInt> basic(static_cast<std::size_t>(m), 0);
    if (highs->getBasicVariables(basic.data()) != HighsStatus::kOk) {
      return false;
    }
    std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
    for (int art : sf.row_to_artificial_col) {
      if (art >= 0 && art < n) {
        is_artificial[static_cast<std::size_t>(art)] = 1;
      }
    }
    for (int row = 0; row < m; ++row) {
      const int col = native_sf_col_from_highs_basic(
          sf, basic[static_cast<std::size_t>(row)]);
      if (col < 0 || col >= n) return false;
      if (is_artificial[static_cast<std::size_t>(col)] &&
          std::abs(out.x_std[col]) >
              std::max(1e-7, opt.feasibility_tol * 20.0)) {
        return false;
      }
      out.basis.indices[static_cast<std::size_t>(row)] = col;
      out.x_basic[row] = out.x_std[col];
    }

    out.reduced_costs = Eigen::VectorXd::Zero(n);
    const bool have_col_dual = static_cast<int>(sol.col_dual.size()) >= n;
    if (!have_col_dual) return false;
    const bool have_col_scale = sf.col_scale.size() == n;
    for (int j = 0; j < n; ++j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      out.reduced_costs[j] =
          -sol.col_dual[static_cast<std::size_t>(j)] * col_scale;
    }

    const HighsInfo& info = highs->getInfo();
    out.basis_inverse.resize(0, 0);
    out.max_objective = info.objective_function_value;
    out.result.stats.solver_name = "VendoredHighsLpKernel";
    out.result.stats.iterations =
        static_cast<int>(info.simplex_iteration_count);
    out.result.stats.objective = sf.objective_const - out.max_objective;
    out.result.stats.status = "Optimal";
    out.result.stats.success = true;
    out.result.stats.primal_feas = 0.0;
    out.result.stats.residual_inf = 0.0;
    out.solved_from_hint = solved_from_hint;
    out.dual_reoptimized = solved_from_hint;
    out.exact_optimal = true;
    out.basis.cached_reduced_costs =
        std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
    if (sf.col_scale.size() == n) {
      out.basis.cached_col_scale =
          std::make_shared<const Eigen::VectorXd>(sf.col_scale);
    }
    out.basis.cached_x_basic =
        std::make_shared<const Eigen::VectorXd>(out.x_basic);
    out.basis.cached_x_std = std::make_shared<const Eigen::VectorXd>(out.x_std);
    out.basis.cached_max_objective = out.max_objective;
    out.basis.has_cached_max_objective = true;
    out.basis.cached_sparse_basis =
        std::make_shared<VendoredHighsBasis>(highs, m, n, &out.form.A);
    out.basis.persist_eta_count = 0;

    double primal_residual = 0.0;
    double bound_violation = 0.0;
    double artificial_activity = 0.0;
    double dual_violation = 0.0;
    double objective_gap = 0.0;
    const double highs_dual_violation =
        info.num_dual_infeasibilities <= 0
            ? 0.0
            : (std::isfinite(info.max_dual_infeasibility)
                   ? info.max_dual_infeasibility
                   : std::numeric_limits<double>::infinity());
    if (!audit_vendored_highs_sf_result(sf, opt, out, primal_residual,
                                        bound_violation, artificial_activity,
                                        dual_violation, objective_gap,
                                        highs_dual_violation)) {
      if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        fmt::print(stderr,
                   "[SF_RELAX] VendoredHiGHS SF audit rejected: ctx={} "
                   "primal={:.3e} bound={:.3e} art={:.3e} dual={:.3e} "
                   "obj_gap={:.3e} m={} n={} hint={}\n",
                   context != nullptr ? context : "solve", primal_residual,
                   bound_violation, artificial_activity, dual_violation,
                   objective_gap, m, n, solved_from_hint ? 1 : 0);
      }
      return false;
    }
    out.result.stats.primal_feas = std::max(primal_residual, bound_violation);
    out.result.stats.residual_inf = primal_residual;
    out.result.stats.dual_feas = dual_violation;
    populate_dual_certificate(sf, out.basis.indices, out.basis.at_upper,
                              out.basis_inverse, out.basis.cached_sparse_basis,
                              out.reduced_costs, out.result);
    return true;
  }

  bool delete_rows_cols_and_resolve(
      const StandardFormLP& compact_sf,
      const SimplexBasis* compact_basis_hint,
      const std::vector<int>& delete_rows,
      const std::vector<int>& delete_cols,
      const SimplexOptions& opt,
      SimplexResult& out) override {
    if (!highs_ || delete_rows.empty() ||
        compact_basis_hint == nullptr ||
        compact_basis_hint->rows != static_cast<int>(compact_sf.A.rows()) ||
        compact_basis_hint->cols != static_cast<int>(compact_sf.A.cols()) ||
        static_cast<int>(compact_basis_hint->index_count()) !=
            static_cast<int>(compact_sf.A.rows())) {
      return false;
    }
    const int old_m = m_;
    const int old_n = n_;
    auto highs = highs_;
    if (!delete_cols.empty()) {
      std::vector<HighsInt> cols;
      cols.reserve(delete_cols.size());
      for (int col : delete_cols) {
        if (col < 0 || col >= old_n) return false;
        cols.push_back(static_cast<HighsInt>(col));
      }
      if (highs->deleteCols(static_cast<HighsInt>(cols.size()),
                            cols.data()) != HighsStatus::kOk) {
        return false;
      }
    }
    {
      std::vector<HighsInt> rows;
      rows.reserve(delete_rows.size());
      for (int row : delete_rows) {
        if (row < 0 || row >= old_m) return false;
        rows.push_back(static_cast<HighsInt>(row));
      }
      if (highs->deleteRows(static_cast<HighsInt>(rows.size()),
                            rows.data()) != HighsStatus::kOk) {
        return false;
      }
    }
    if (highs->getNumRow() != compact_sf.A.rows() ||
        highs->getNumCol() != compact_sf.A.cols()) {
      return false;
    }
    HighsBasis hbasis;
    if (!native_sf_basis_to_highs(compact_sf, compact_basis_hint, hbasis)) {
      return false;
    }
    if (highs->setBasis(hbasis,
                        "MIPSOLVERS vendored xpool aging removeCuts") !=
        HighsStatus::kOk) {
      return false;
    }
    const HighsStatus run_status = highs->run();
    if (run_status != HighsStatus::kOk ||
        highs->getModelStatus() != HighsModelStatus::kOptimal) {
      out.result.stats.solver_name = "VendoredHighsLpKernel";
      out.result.stats.success = false;
      out.result.stats.status =
          std::string("HiGHS ") +
          highs_sf_model_status_label(highs->getModelStatus());
      return false;
    }
    return import_optimal_result(highs, compact_sf, opt, true,
                                 "delete_rows_cols_and_resolve", out);
  }

 private:
  std::shared_ptr<Highs> highs_;
  int m_{0};
  int n_{0};
  Eigen::SparseMatrix<double> A_owned_;
};

bool solve_standard_form_with_vendored_highs(const StandardFormLP& sf,
                                             const SimplexOptions& opt,
                                             const SimplexBasis* basis_hint,
                                             SimplexResult& out) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (m <= 0 || n <= 0) return false;

  const auto t0 = std::chrono::steady_clock::now();
  const bool trace_sf = std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr;
  auto highs = std::make_shared<Highs>();
  highs->setOptionValue("output_flag", false);
  highs->setOptionValue("log_to_console", false);
  highs->setOptionValue("threads", 1);
  highs->setOptionValue("presolve", "off");
  highs->setOptionValue("solver", "simplex");
  highs->setOptionValue("simplex_strategy", 1);
  highs->setOptionValue("simplex_scale_strategy", 2);
  highs->setOptionValue("simplex_initial_condition_check", false);
  highs->setOptionValue("simplex_iteration_limit",
                       std::max(opt.max_iter * 10, 10000));
  if (opt.time_limit_sec > 0.0 && std::isfinite(opt.time_limit_sec)) {
    highs->setOptionValue("time_limit", std::max(0.001, opt.time_limit_sec));
  }
  highs->setOptionValue("kkt_tolerance", kHighsDefaultKktTolerance);
  highs->setOptionValue("primal_feasibility_tolerance",
                       kHighsDefaultKktTolerance);
  highs->setOptionValue("dual_feasibility_tolerance",
                       kHighsDefaultKktTolerance);
  highs->setOptionValue("mip_feasibility_tolerance",
                       kHighsDefaultMipTolerance);
  auto reject = [&](const char* reason) {
    out.result.stats.solver_name = "VendoredHighsLpKernel";
    out.result.stats.success = false;
    out.result.stats.status = std::string("VendoredHiGHS SF reject: ") +
                              (reason != nullptr ? reason : "unknown");
    out.result.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    if (trace_sf) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF rejected: reason={} m={} n={} "
                 "hint={} time={:.3f}ms\n",
                 reason != nullptr ? reason : "unknown", m, n,
                 basis_hint ? 1 : 0,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return false;
  };

  if (!pass_standard_form_to_highs(*highs, sf)) {
    return reject("pass_model");
  }
  HighsBasis hbasis;
  if (native_sf_basis_to_highs(sf, basis_hint, hbasis)) {
    (void)highs->setBasis(hbasis, "MIPSOLVERS native SF basis hint");
  }

  const HighsStatus run_status = highs->run();
  const HighsModelStatus model_status = highs->getModelStatus();
  const HighsInfo& info = highs->getInfo();

  out.form = sf;
  out.result.stats.solver_name = "VendoredHighsLpKernel";
  out.result.stats.iterations =
      static_cast<int>(info.simplex_iteration_count);
  out.result.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  out.result.stats.status =
      std::string("HiGHS ") + highs_sf_model_status_label(model_status);
  if (model_status == HighsModelStatus::kInfeasible) {
    out.result.stats.status = "LP infeasible";
    out.result.stats.success = false;
    out.result.stats.objective = kInf;
    out.result.stats.has_farkas_certificate = true;
    bool has_dual_ray = false;
    std::vector<double> dual_ray(static_cast<std::size_t>(m), 0.0);
    if (highs->getDualRay(has_dual_ray, dual_ray.data()) ==
            HighsStatus::kOk &&
        has_dual_ray) {
      int eq_count = 0;
      int ineq_count = 0;
      for (int row = 0; row < m; ++row) {
        if (sf.row_to_artificial_col[row] >= 0 &&
            sf.row_to_surplus_col[row] < 0) {
          ++eq_count;
        } else {
          ++ineq_count;
        }
      }
      out.result.stats.farkas_ray =
          Eigen::VectorXd::Zero(std::max(0, ineq_count));
      out.result.stats.farkas_ray_eq =
          Eigen::VectorXd::Zero(std::max(0, eq_count));
      int ineq_pos = 0;
      int eq_pos = 0;
      for (int row = 0; row < m; ++row) {
        const double value =
            (row < static_cast<int>(sf.row_sign.size()) ? sf.row_sign[row]
                                                        : 1.0) *
            dual_ray[static_cast<std::size_t>(row)];
        if (sf.row_to_artificial_col[row] >= 0 &&
            sf.row_to_surplus_col[row] < 0) {
          if (eq_pos < eq_count) out.result.stats.farkas_ray_eq[eq_pos++] = value;
        } else {
          if (ineq_pos < ineq_count) out.result.stats.farkas_ray[ineq_pos++] = value;
        }
      }
    }
    if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF: infeasible m={} n={} "
                 "hint={} ray={} iter={} time={:.3f}ms\n",
                 m, n, basis_hint ? 1 : 0, has_dual_ray ? 1 : 0,
                 out.result.stats.iterations,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return true;
  }
  if (run_status != HighsStatus::kOk ||
      model_status != HighsModelStatus::kOptimal) {
    out.result.stats.status =
        std::string("HiGHS ") + highs_sf_model_status_label(model_status);
    if (trace_sf) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF rejected: reason=status "
                 "run={} model={} iter={} m={} n={} hint={} time={:.3f}ms\n",
                 static_cast<int>(run_status),
                 highs_sf_model_status_label(model_status),
                 out.result.stats.iterations, m, n, basis_hint ? 1 : 0,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return false;
  }

  const HighsSolution& sol = highs->getSolution();
  const HighsBasis& basis = highs->getBasis();
  if (!basis.valid || static_cast<int>(basis.col_status.size()) < n ||
      static_cast<int>(basis.row_status.size()) < m ||
      static_cast<int>(sol.col_value.size()) < n) {
    return reject("invalid_basis_or_solution");
  }

  out.x_std = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n; ++j) {
    out.x_std[j] = sol.col_value[static_cast<std::size_t>(j)];
  }
  out.result.x = extract_solution(sf, out.x_std);
  out.x_basic = Eigen::VectorXd::Zero(m);
  out.basis.rows = m;
  out.basis.cols = n;
  out.basis.indices.assign(static_cast<std::size_t>(m), -1);
  out.basis.at_upper.assign(static_cast<std::size_t>(n), 0);
  out.basis.sf_n_slack = sf.n_slack;
  out.basis.sf_n_surplus = sf.n_surplus;
  out.basis.sf_n_artificial = sf.n_artificial;

  for (int j = 0; j < n; ++j) {
    out.basis.at_upper[static_cast<std::size_t>(j)] =
        basis.col_status[static_cast<std::size_t>(j)] ==
                HighsBasisStatus::kUpper
            ? 1
            : 0;
  }

  std::vector<HighsInt> basic(static_cast<std::size_t>(m), 0);
  if (highs->getBasicVariables(basic.data()) != HighsStatus::kOk) {
    return reject("basic_variables");
  }
  std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) is_artificial[static_cast<std::size_t>(art)] = 1;
  }
  for (int row = 0; row < m; ++row) {
    const int col = native_sf_col_from_highs_basic(
        sf, basic[static_cast<std::size_t>(row)]);
    if (col < 0 || col >= n) return reject("basic_index_out_of_range");
    if (is_artificial[static_cast<std::size_t>(col)] &&
        std::abs(out.x_std[col]) > std::max(1e-7, opt.feasibility_tol * 20.0)) {
      return reject("positive_artificial_basic");
    }
    out.basis.indices[static_cast<std::size_t>(row)] = col;
    out.x_basic[row] = out.x_std[col];
  }

  out.reduced_costs = Eigen::VectorXd::Zero(n);
  {
    const bool have_col_dual = static_cast<int>(sol.col_dual.size()) >= n;
    if (have_col_dual) {
      const bool have_col_scale = sf.col_scale.size() == n;
      for (int j = 0; j < n; ++j) {
        const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
        // Keep the same sign convention as native simplex:
        // col_dual = -reduced_cost / col_scale  => reduced_cost = -col_dual*col_scale
        out.reduced_costs[j] =
            -sol.col_dual[static_cast<std::size_t>(j)] * col_scale;
      }
    } else {
      Eigen::VectorXd c_b = Eigen::VectorXd::Zero(m);
      for (int row = 0; row < m; ++row) {
        const int col = out.basis.indices[static_cast<std::size_t>(row)];
        c_b[row] = (col >= 0 && col < n) ? sf.c_max[col] : 0.0;
      }
      std::vector<double> rhs(static_cast<std::size_t>(m), 0.0);
      std::vector<double> y(static_cast<std::size_t>(m), 0.0);
      for (int row = 0; row < m; ++row) {
        rhs[static_cast<std::size_t>(row)] = c_b[row];
      }
      if (highs->getBasisTransposeSolve(rhs.data(), y.data()) !=
          HighsStatus::kOk) {
        return reject("basis_transpose_solve");
      }
      Eigen::VectorXd y_scaled = Eigen::VectorXd::Zero(m);
      for (int row = 0; row < m; ++row) {
        y_scaled[row] = y[static_cast<std::size_t>(row)];
      }
      out.reduced_costs =
          sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_scaled);
    }
  }

  out.basis_inverse.resize(0, 0);
  if (m <= 4096) {
    out.basis_inverse = Eigen::MatrixXd::Zero(m, m);
    std::vector<double> row_vec(static_cast<std::size_t>(m), 0.0);
    for (int row = 0; row < m; ++row) {
      std::fill(row_vec.begin(), row_vec.end(), 0.0);
      if (highs->getBasisInverseRow(static_cast<HighsInt>(row),
                                   row_vec.data()) != HighsStatus::kOk) {
        out.basis_inverse.resize(0, 0);
        break;
      }
      for (int col = 0; col < m; ++col) {
        out.basis_inverse(row, col) =
            row_vec[static_cast<std::size_t>(col)];
      }
    }
  }

  out.max_objective = info.objective_function_value;
  out.result.stats.success = true;
  out.result.stats.objective = sf.objective_const - out.max_objective;
  out.result.stats.status = "Optimal";
  out.result.stats.primal_feas = 0.0;
  out.result.stats.residual_inf = 0.0;
  out.solved_from_hint = basis_hint != nullptr;
  out.dual_reoptimized = basis_hint != nullptr;
  out.exact_optimal = true;
  out.basis.cached_reduced_costs =
      std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
  if (sf.col_scale.size() == n) {
    out.basis.cached_col_scale =
        std::make_shared<const Eigen::VectorXd>(sf.col_scale);
  }
  out.basis.cached_x_basic =
      std::make_shared<const Eigen::VectorXd>(out.x_basic);
  out.basis.cached_x_std =
      std::make_shared<const Eigen::VectorXd>(out.x_std);
  out.basis.cached_max_objective = out.max_objective;
  out.basis.has_cached_max_objective = true;
  out.basis.cached_sparse_basis =
      std::make_shared<VendoredHighsBasis>(highs, m, n, &out.form.A);
  out.basis.persist_eta_count = 0;
  const int degenerate_duals = inject_basic_degenerate_col_duals_from_highs(
      sf, *highs, sol, out.basis.basis_indices(), out.reduced_costs,
      kHighsDefaultMipTolerance, kHighsDefaultKktTolerance);
  if (degenerate_duals > 0) {
    out.basis.cached_reduced_costs =
        std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
  }
  if (trace_sf && degenerate_duals > 0) {
    fmt::print(stderr,
               "[SF_RELAX] VendoredHiGHS basic-degenerate duals injected={}\n",
               degenerate_duals);
  }
  double primal_residual = 0.0;
  double bound_violation = 0.0;
  double artificial_activity = 0.0;
  double dual_violation = 0.0;
  double objective_gap = 0.0;
  const double highs_dual_violation =
      info.num_dual_infeasibilities <= 0
          ? 0.0
          : (std::isfinite(info.max_dual_infeasibility)
                 ? info.max_dual_infeasibility
                 : std::numeric_limits<double>::infinity());
  if (!audit_vendored_highs_sf_result(sf, opt, out, primal_residual,
                                      bound_violation, artificial_activity,
                                      dual_violation, objective_gap,
                                      highs_dual_violation)) {
    if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF audit rejected: "
                 "primal={:.3e} bound={:.3e} art={:.3e} dual={:.3e} "
                 "obj_gap={:.3e} m={} n={} hint={}\n",
                 primal_residual, bound_violation, artificial_activity,
                 dual_violation, objective_gap, m, n, basis_hint ? 1 : 0);
    }
    return false;
  }
  out.result.stats.primal_feas = std::max(primal_residual, bound_violation);
  out.result.stats.residual_inf = primal_residual;
  out.result.stats.dual_feas = dual_violation;
  populate_dual_certificate(sf, out.basis.indices, out.basis.at_upper,
                            out.basis_inverse, out.basis.cached_sparse_basis,
                            out.reduced_costs, out.result);
  if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
    fmt::print(stderr,
               "[SF_RELAX] VendoredHiGHS SF: success=1 iter={} obj={:.12g} "
               "m={} n={} hint={} time={:.3f}ms\n",
               out.result.stats.iterations, out.result.stats.objective,
               m, n, basis_hint ? 1 : 0,
               out.result.stats.runtime_sec * 1000.0);
  }
  return true;
}
#else
bool solve_standard_form_with_vendored_highs(const StandardFormLP&,
                                             const SimplexOptions&,
                                             const SimplexBasis*,
                                             SimplexResult&) {
  return false;
}
#endif

// Forward declaration for crash_artificial_basis.
Eigen::MatrixXd basis_matrix(const Eigen::SparseMatrix<double>& a, const std::vector<int>& basis);

/// Crash procedure: replace artificial variables with suitable original-variable
/// columns in the starting basis.  For each row that uses an artificial, try to
/// find an original column with a large nonzero coefficient that isn't already
/// in the basis.  This can skip Phase I entirely when the resulting BFS is
/// feasible (x_b >= 0 within bounds).
///
/// Swaps are applied incrementally: after the initial factorization, each swap
/// updates x_b via the simplex pivot formula.  If the updated BFS becomes
/// infeasible, that swap is reverted and skipped.  This keeps as many crash
/// swaps as possible to minimize Phase I work.
void crash_artificial_basis(const StandardFormLP& sf,
                            std::vector<int>& basis) {
  const int m = sf.A.rows();
  const int n_orig = sf.n_original;
  if (n_orig == 0 || m == 0) return;

  // Mark columns already in the basis.
  std::vector<char> in_basis(static_cast<size_t>(sf.A.cols()), 0);
  for (int i = 0; i < m; ++i) {
    if (basis[i] >= 0) in_basis[static_cast<size_t>(basis[i])] = 1;
  }

  // Identify candidate swaps: for each artificial row, find the best original column.
  struct CrashCandidate {
    int row;
    int new_col;
    double abs_val;
  };
  std::vector<CrashCandidate> candidates;
  candidates.reserve(static_cast<size_t>(m));

  for (int i = 0; i < m; ++i) {
    if (sf.row_to_artificial_col[i] < 0) continue;

    int best_col = -1;
    double best_abs = 0.0;
    // Use row-major matrix for O(nnz_row) instead of O(n_orig × nnz_col).
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, i);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= n_orig) continue;
      if (in_basis[static_cast<size_t>(j)]) continue;
      const double av = std::abs(it.value());
      if (av > best_abs) {
        best_abs = av;
        best_col = j;
      }
    }
    if (best_col >= 0 && best_abs > 1e-6) {
      candidates.push_back({i, best_col, best_abs});
      in_basis[static_cast<size_t>(best_col)] = 1;
    }
  }

  if (candidates.empty()) return;

  // For large problems, use a lightweight crash: accept swaps greedily
  // with a simple triangularity check instead of the O(C²×m) eta-update
  // approach.  The Big-M simplex that follows recomputes x_b from scratch,
  // so maintaining a valid BFS during crash is not essential — we just need
  // a good starting basis with fewer artificials.
  //
  // The initial basis is all slacks/artificials, so B = I and x_b = b.
  // For a swap replacing artificial in row i with column j:
  //   If A[i,j] is the only nonzero in column j among basis rows, it's a
  //   "triangular" swap and x_b stays non-negative (just scales b[i]).
  //   For non-triangular swaps, x_b entries in other rows may become
  //   negative.  We accept all candidates but process triangular ones
  //   first to maximize quality.
  if (m > 200) {
    // Accept all candidates greedily.  The caller (cold-start in
    // solve_lp_from_sf) validates primal feasibility and reverts individual
    // swaps that cause x_b < 0, keeping as many crash swaps as possible.
    for (const auto& c : candidates) {
      const int old_col = basis[c.row];
      in_basis[static_cast<size_t>(old_col)] = 0;
      basis[c.row] = c.new_col;
    }
    return;
  }

  // For small problems (m <= 200), keep the exact eta-update approach
  // for BFS-preserving crash.
}

Eigen::MatrixXd basis_matrix(const Eigen::SparseMatrix<double>& a, const std::vector<int>& basis) {
  const int m = static_cast<int>(a.rows());
  const int k = static_cast<int>(basis.size());
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(m, k);
  for (int i = 0; i < k; ++i) {
    const int col = basis[static_cast<size_t>(i)];
    for (Eigen::SparseMatrix<double>::InnerIterator it(a, col); it; ++it) {
      b(it.row(), i) = it.value();
    }
  }
  return b;
}

bool compute_basis_state(const StandardFormLP& sf,
                         const std::vector<int>& basis,
                         Eigen::MatrixXd& binv,
                         Eigen::VectorXd& x_b,
                         Eigen::VectorXd& reduced_costs,
                         double& obj) {
  if (static_cast<int>(basis.size()) != sf.A.rows()) {
    return false;
  }

  const Eigen::MatrixXd b = basis_matrix(sf.A, basis);
  Eigen::PartialPivLU<Eigen::MatrixXd> lu(b);

  binv = lu.inverse();
  x_b = binv * sf.b;

  Eigen::VectorXd c_b(basis.size());
  for (int i = 0; i < static_cast<int>(basis.size()); ++i) {
    c_b[i] = sf.c_max[basis[i]];
  }
  const Eigen::VectorXd pi = binv.transpose() * c_b;
  reduced_costs = sf.c_max - Eigen::VectorXd(sf.A.transpose() * pi);
  obj = c_b.dot(x_b);
  return true;
}

bool primal_simplex_optimize(const StandardFormLP& sf,
                             std::vector<int>& basis,
                             const std::vector<char>& can_enter,
                             const SimplexOptions& opt,
                             Eigen::MatrixXd& binv,
                             Eigen::VectorXd& x_b,
                             Eigen::VectorXd& reduced_costs,
                             double& obj,
                             std::vector<char>& at_upper) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;

  std::vector<char> is_basic(static_cast<size_t>(n), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<size_t>(idx)] = 1;
  }

  // Compute initial basis state: x_b = B^{-1}*b, rc, obj (ignoring at_upper).
  if (!compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj)) {
    return false;
  }
  // Adjust x_b for non-basic variables at their upper bound BEFORE computing obj.
  // NOTE: Use c_max[j] (not reduced_costs[j]) for objective contribution!
  if (has_ub) {
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
      const double uj = sf.var_ub[j];
      if (!std::isfinite(uj)) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        x_b -= it.value() * uj * binv.col(it.row());
      }
    }
    // Now recompute obj with adjusted x_b.
    const int bsz = static_cast<int>(basis.size());
    Eigen::VectorXd c_b(bsz);
    for (int i = 0; i < bsz; ++i) c_b[i] = sf.c_max[basis[i]];
    obj = c_b.dot(x_b);
    // Add contribution from non-basic at upper.
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
      const double uj = sf.var_ub[j];
      if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
    }
  }

  // Adaptive refactorization: shorter period for large problems to control
  // accumulated numerical drift in the dense basis inverse.
  const int refactor_period = m > 500 ? 15 : (m > 200 ? 25 : 50);
  Eigen::VectorXd direction(m);
  Eigen::VectorXd best_dir(m);
  Eigen::RowVectorXd old_binv_row(m);
  Eigen::RowVectorXd pivot_row_buf(m);
  Eigen::VectorXd alpha(n);
  bool just_refactored = true;  // Initial compute_basis_state counts.
  const auto wall_t0 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < opt.max_iter; ++iter, ++g_simplex_iter_count) {
    if ((iter & 15) == 0 && simplex_wall_time_limit_hit(opt, wall_t0)) {
      return false;
    }
    int entering = -1;
    double best_rc_abs = opt.optimality_tol;
    bool enter_from_upper = false;

    for (int j = 0; j < n; ++j) {
      if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
      const bool j_up = at_upper[static_cast<size_t>(j)];
      const double rc_abs = j_up ? -reduced_costs[j] : reduced_costs[j];
      if (rc_abs > best_rc_abs + 1e-14 ||
          (std::abs(rc_abs - best_rc_abs) <= 1e-14 &&
           (entering < 0 || j < entering))) {
        best_rc_abs = rc_abs;
        entering = j;
        enter_from_upper = j_up;
      }
    }
    if (entering < 0) {
      // Before declaring optimal, refactorize to eliminate numerical drift.
      // With m > 500, accumulated error after several pivots can push reduced
      // costs below tolerance when they should still be positive.
      if (just_refactored) return true;  // Truly optimal after fresh state.
      if (!compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj)) {
        return false;
      }
      if (has_ub) {
        // First adjust x_b for at_upper variables.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (!std::isfinite(uj)) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
            x_b -= it.value() * uj * binv.col(it.row());
          }
        }
        // Recompute obj with adjusted x_b.
        Eigen::VectorXd c_b_tmp(m);
        for (int i = 0; i < m; ++i) c_b_tmp[i] = sf.c_max[basis[i]];
        obj = c_b_tmp.dot(x_b);
        // Add contribution from non-basic at upper.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
        }
      }
      just_refactored = true;
      continue;  // Retry entering variable search with fresh reduced costs.
    }

    direction.setZero();
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, entering); it; ++it) {
      direction.noalias() += it.value() * binv.col(it.row());
    }

    // Effective direction: if entering from upper, variable decreases.
    const double sign = enter_from_upper ? -1.0 : 1.0;

    double max_step = kInf;
    int leaving_row = -1;
    bool leaving_to_upper = false;

    for (int i = 0; i < m; ++i) {
      const double d = sign * direction[i];
      if (d > opt.feasibility_tol) {
        const double ratio = x_b[i] / d;
        if (ratio < max_step - 1e-12 ||
            (std::abs(ratio - max_step) <= 1e-12 && (leaving_row < 0 || i < leaving_row))) {
          max_step = ratio;
          leaving_row = i;
          leaving_to_upper = false;
        }
      } else if (has_ub && d < -opt.feasibility_tol) {
        const int bvar = basis[i];
        const double ub_i = sf.var_ub[bvar];
        if (std::isfinite(ub_i)) {
          const double slack = ub_i - x_b[i];
          if (slack >= -opt.feasibility_tol) {
            const double ratio = slack / (-d);
            if (ratio < max_step - 1e-12 ||
                (std::abs(ratio - max_step) <= 1e-12 && (leaving_row < 0 || i < leaving_row))) {
              max_step = ratio;
              leaving_row = i;
              leaving_to_upper = true;
            }
          }
        }
      }
    }

    bool bound_flip = false;
    if (has_ub) {
      const double uj = sf.var_ub[entering];
      if (std::isfinite(uj) && uj < max_step - 1e-12) {
        max_step = uj;
        bound_flip = true;
      }
    }

    if (!bound_flip && leaving_row < 0) continue;

    const double old_rc = reduced_costs[entering];

    if (bound_flip) {
      x_b.noalias() -= direction * (sign * max_step);
      obj += old_rc * (sign * max_step);
      at_upper[static_cast<size_t>(entering)] = enter_from_upper ? 0 : 1;
    } else {
      old_binv_row = binv.row(leaving_row);
      const double pivot = direction[leaving_row];
      if (std::abs(pivot) < 1e-12) return false;

      const int old_leaving = basis[leaving_row];
      is_basic[static_cast<size_t>(old_leaving)] = 0;
      at_upper[static_cast<size_t>(old_leaving)] = leaving_to_upper ? 1 : 0;
      basis[leaving_row] = entering;
      is_basic[static_cast<size_t>(entering)] = 1;
      at_upper[static_cast<size_t>(entering)] = 0;

      const double raw_step = sign * max_step;
      pivot_row_buf = binv.row(leaving_row) / pivot;
      binv.noalias() -= direction * pivot_row_buf;
      binv.row(leaving_row) = pivot_row_buf;
      x_b.noalias() -= direction * raw_step;
      x_b[leaving_row] = enter_from_upper
          ? (sf.var_ub[entering] - max_step)
          : max_step;

      alpha.noalias() = sf.A.transpose() * old_binv_row.transpose();
      const double rc_scale = old_rc / pivot;
      reduced_costs -= rc_scale * alpha;
      reduced_costs[entering] = 0.0;

      obj += old_rc * raw_step;
    }

    // NaN/Inf guard: numerical breakdown in pivot updates.
    if (!std::isfinite(obj)) return false;

    if (!bound_flip && (iter + 1) % refactor_period == 0) {
      if (!compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj)) {
        return false;
      }
      if (has_ub) {
        // First adjust x_b for at_upper variables.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (!std::isfinite(uj)) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
            x_b -= it.value() * uj * binv.col(it.row());
          }
        }
        // Recompute obj with adjusted x_b.
        Eigen::VectorXd c_b_tmp(m);
        for (int i = 0; i < m; ++i) c_b_tmp[i] = sf.c_max[basis[i]];
        obj = c_b_tmp.dot(x_b);
        // Add contribution from non-basic at upper.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
        }
      }
      just_refactored = true;
    } else {
      just_refactored = false;
    }
  }

  return false;
}

bool dual_simplex_reoptimize(const StandardFormLP& sf,
                             std::vector<int>& basis,
                             const std::vector<char>& can_enter,
                             const SimplexOptions& opt,
                             Eigen::MatrixXd& binv,
                             Eigen::VectorXd& x_b,
                             Eigen::VectorXd& reduced_costs,
                             double& obj,
                             std::vector<char>& at_upper,
                             bool state_initialized = false) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;

  std::vector<char> is_basic(static_cast<size_t>(n), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<size_t>(idx)] = 1;
  }

  if (!state_initialized) {
    if (!compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj)) {
      return false;
    }
    // Adjust x_b for non-basic at upper BEFORE computing obj.
    // NOTE: Use c_max[j] (not reduced_costs[j]) for objective contribution!
    if (has_ub) {
      // First adjust x_b.
      for (int j = 0; j < n; ++j) {
        if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
        const double uj = sf.var_ub[j];
        if (!std::isfinite(uj)) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
          x_b -= it.value() * uj * binv.col(it.row());
        }
      }
      // Recompute obj with adjusted x_b.
      Eigen::VectorXd c_b_tmp(m);
      for (int i = 0; i < m; ++i) c_b_tmp[i] = sf.c_max[basis[i]];
      obj = c_b_tmp.dot(x_b);
      // Add contribution from non-basic at upper.
      for (int j = 0; j < n; ++j) {
        if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
        const double uj = sf.var_ub[j];
        if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
      }
    }
  }

  const int refactor_period = m > 1500 ? 20 : (m > 500 ? 25 : (m > 200 ? 30 : 50));
  const double pivot_tol = m > 1500 ? 1e-8 : 1e-12;
  Eigen::VectorXd row_vec(n);
  Eigen::VectorXd direction(m);
  Eigen::RowVectorXd pivot_row(m);
  const auto wall_t0 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < opt.max_iter; ++iter, ++g_simplex_iter_count) {
    if ((iter & 15) == 0 && simplex_wall_time_limit_hit(opt, wall_t0)) {
      return false;
    }
    // Dual feasibility check (with bounded variables).
    // For non-basic at lower: rc_j ≤ 0 (maximization convention: rc ≤ 0).
    // For non-basic at upper: rc_j ≥ 0 (flipped: profitable to decrease).
    bool dual_ok = true;
    [[maybe_unused]] int dual_fail_var = -1;
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<size_t>(j)] || !can_enter[static_cast<size_t>(j)]) continue;
      if (at_upper[static_cast<size_t>(j)]) {
        if (reduced_costs[j] < -opt.optimality_tol) { dual_ok = false; dual_fail_var = j; break; }
      } else {
        if (reduced_costs[j] > opt.optimality_tol) { dual_ok = false; dual_fail_var = j; break; }
      }
    }
    if (!dual_ok) {
      if (m > 2000) MIPSOLVERS_LOG_DEBUG("  [dual_reopt] FAIL: dual_infeas iter={} var={} rc={:.6e} at_up={}", iter, dual_fail_var, reduced_costs[dual_fail_var], (int)at_upper[dual_fail_var]);
      return false;
    }

    // Find leaving variable: most infeasible basic variable.
    // Check both lower bound (x_b < 0) and upper bound (x_b > ub) violations.
    int leaving_row = -1;
    double worst_infeas = opt.feasibility_tol;
    bool leaving_below = true;  // true = below lower, false = above upper

    for (int i = 0; i < m; ++i) {
      if (x_b[i] < -worst_infeas) {
        worst_infeas = -x_b[i];
        leaving_row = i;
        leaving_below = true;
      }
      if (has_ub) {
        const int bvar = basis[i];
        const double ub_i = sf.var_ub[bvar];
        if (std::isfinite(ub_i) && x_b[i] > ub_i + worst_infeas) {
          worst_infeas = x_b[i] - ub_i;
          leaving_row = i;
          leaving_below = false;
        }
      }
    }
    if (leaving_row < 0) return true;  // Primal feasible → optimal.

    // Pivot row.
    row_vec.noalias() = sf.A.transpose() * binv.row(leaving_row).transpose();

    // Dual ratio test with bounded variables.
    constexpr double dense_alpha_tol = 1e-9;
    int entering = -1;
    double best_ratio = kInf;

    if (leaving_below) {
      for (int j = 0; j < n; ++j) {
        if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
        const bool j_up = at_upper[static_cast<size_t>(j)];
        double alpha_j;
        if (j_up) {
          alpha_j = row_vec[j];
          if (alpha_j < dense_alpha_tol) continue;
        } else {
          alpha_j = -row_vec[j];
          if (alpha_j < dense_alpha_tol) continue;
        }
        const double ratio = j_up ? (reduced_costs[j] / alpha_j) : (-reduced_costs[j] / alpha_j);
        if (ratio < best_ratio - 1e-12 ||
            (std::abs(ratio - best_ratio) <= 1e-12 && (entering < 0 || j < entering))) {
          best_ratio = ratio;
          entering = j;
        }
      }
    } else {
      for (int j = 0; j < n; ++j) {
        if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
        const bool j_up = at_upper[static_cast<size_t>(j)];
        double alpha_j;
        if (j_up) {
          alpha_j = -row_vec[j];
          if (alpha_j < dense_alpha_tol) continue;
        } else {
          alpha_j = row_vec[j];
          if (alpha_j < dense_alpha_tol) continue;
        }
        const double ratio = j_up ? (reduced_costs[j] / alpha_j) : (-reduced_costs[j] / alpha_j);
        if (ratio < best_ratio - 1e-12 ||
            (std::abs(ratio - best_ratio) <= 1e-12 && (entering < 0 || j < entering))) {
          best_ratio = ratio;
          entering = j;
        }
      }
    }

    if (entering < 0) {
      return false;  // Dual unbounded → primal infeasible.
    }

    const double old_rc_entering = reduced_costs[entering];

    // Standard pivot.
    direction.setZero();
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, entering); it; ++it) {
      direction.noalias() += it.value() * binv.col(it.row());
    }
    const double pivot_val = direction[leaving_row];
    // N2: Harris-style relative pivot threshold (large problems only).
    // For m > 20000 the absolute 1e-8 pivot_tol rejects numerically sound
    // pivots whose column has small overall magnitude after Ruiz / UMFPACK
    // row-sum scaling -- on uc_500g that loses 1.3s of re-factorisation
    // per node.  A pivot is acceptable if it is within a fixed fraction
    // (1e-7) of the column's ell-infinity norm, clamped from below by the
    // absolute floor.  For m <= 20000 the absolute floor is kept because
    // a stricter relative guard steers pivoting away from the
    // near-degenerate choices that give the tightest LP bound on
    // moderately sized UC instances (empirical 1.3--2x node-count
    // regression on uc_50g/100g/200g when the relative guard is active).
    double pivot_tol_effective = pivot_tol;
    if (m > 20000) {
      const double dir_inf_norm = direction.cwiseAbs().maxCoeff();
      pivot_tol_effective = std::max(pivot_tol, 1e-7 * dir_inf_norm);
    }
    if (std::abs(pivot_val) < pivot_tol_effective) {
      if (m > 2000) MIPSOLVERS_LOG_DEBUG("  [dual_reopt] FAIL: small pivot={:.4e} rel_tol={:.4e} iter={}", pivot_val, pivot_tol_effective, iter);
      return false;
    }

    const int old_leaving = basis[leaving_row];
    const bool entering_was_upper = at_upper[static_cast<size_t>(entering)];
    is_basic[static_cast<size_t>(old_leaving)] = 0;
    // Leaving var goes to lower (0) if it was below, or upper if above.
    at_upper[static_cast<size_t>(old_leaving)] = leaving_below ? 0 : 1;
    basis[leaving_row] = entering;
    is_basic[static_cast<size_t>(entering)] = 1;
    at_upper[static_cast<size_t>(entering)] = 0;  // Basic vars don't use at_upper.

    // Compute step: Δx_entering such that leaving goes to its bound.
    double step;
    if (leaving_below) {
      step = x_b[leaving_row] / pivot_val;
    } else {
      const double ub_leaving = sf.var_ub[old_leaving];
      step = (x_b[leaving_row] - ub_leaving) / pivot_val;
    }
    pivot_row = binv.row(leaving_row) / pivot_val;
    binv.noalias() -= direction * pivot_row;
    binv.row(leaving_row) = pivot_row;
    x_b.noalias() -= direction * step;
    // Entering variable's new value in the basis.
    x_b[leaving_row] = entering_was_upper
        ? (sf.var_ub[entering] + step)
        : step;

    const double rc_scale = old_rc_entering / pivot_val;
    reduced_costs.noalias() -= rc_scale * row_vec;
    reduced_costs[entering] = 0.0;

    obj += step * old_rc_entering;

    // NaN/Inf guard: numerical breakdown in pivot updates.
    if (!std::isfinite(obj)) return false;

    if ((iter + 1) % refactor_period == 0) {
      if (!compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj)) {
        return false;
      }
      if (has_ub) {
        // First adjust x_b for at_upper variables.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (!std::isfinite(uj)) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
            x_b -= it.value() * uj * binv.col(it.row());
          }
        }
        // Recompute obj with adjusted x_b.
        Eigen::VectorXd c_b_tmp(m);
        for (int i = 0; i < m; ++i) c_b_tmp[i] = sf.c_max[basis[i]];
        obj = c_b_tmp.dot(x_b);
        // Add contribution from non-basic at upper.
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)] || !at_upper[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
        }
      }
    }
  }

  return false;
}

// ============================================================================
// Sparse LU-based simplex for large problems (m > threshold)
// ============================================================================
//
// Uses Eigen::SparseLU instead of dense basis inverse. Key advantages:
//   1. SparseLU factorize is O(nnz * fill) vs O(m³) for dense
//   2. Each FTRAN/BTRAN solve is O(nnz_LU) vs O(m²) for dense
//   3. Between refactorizations, eta vectors give O(k·m) updates
//      vs O(m²) rank-1 dense updates per pivot
//
// For uc_10g_24t (m=1204): dense costs ~19s, sparse targets ~0.5-1s.

constexpr int kSparseSimplexThreshold = 200;

struct EtaVector {
  int pivot_row;
  std::vector<int> nz_idx;       // nonzero indices (excluding pivot_row)
  std::vector<double> nz_val;    // corresponding values of column[]
  double pivot_value;             // column[pivot_row]
};

static inline void apply_eta_ftran(Eigen::VectorXd& z, const EtaVector& eta) {
  const double zr = z[eta.pivot_row];
  const double scale = zr / eta.pivot_value;
  const int n = static_cast<int>(eta.nz_idx.size());
  const int* MIPSOLVERS_RESTRICT ix = eta.nz_idx.data();
  const double* MIPSOLVERS_RESTRICT vl = eta.nz_val.data();
  double* MIPSOLVERS_RESTRICT zd = z.data();
  for (int k = 0; k < n; ++k) {
    zd[ix[k]] -= scale * vl[k];
  }
  z[eta.pivot_row] = scale;
}

static inline void apply_eta_btran(Eigen::VectorXd& z, const EtaVector& eta) {
  double dot_other = 0.0;
  const int n = static_cast<int>(eta.nz_idx.size());
  const int* MIPSOLVERS_RESTRICT ix = eta.nz_idx.data();
  const double* MIPSOLVERS_RESTRICT vl = eta.nz_val.data();
  const double* MIPSOLVERS_RESTRICT zd = z.data();
  for (int k = 0; k < n; ++k) {
    dot_other += vl[k] * zd[ix[k]];
  }
  z[eta.pivot_row] = (z[eta.pivot_row] - dot_other) / eta.pivot_value;
}

// ════════════════════════════════════════════════════════════════════════════
// NativeLU: Extracted L,U factors from UMFPACK with sparse triangular solves.
//
// After UMFPACK factorization P * diag(S) * A * Q = L * U (where S is row
// scaling), we extract L, U, P, Q, Rs and perform FTRAN/BTRAN using sparse
// column-based triangular solves that skip zero entries.  This gives large
// speedups when the RHS is sparse:
//   - FTRAN of a sparse A column: L solve starts from ~5-10 nonzeros
//   - BTRAN after eta application: U^T solve starts from ~k+1 nonzeros
// ════════════════════════════════════════════════════════════════════════════
struct NativeLU {
  bool valid = false;
  bool l_is_identity = false;  // true when L has zero off-diagonal entries
  int m = 0;

  // Permutations: P[k]=i means original row i is at LU position k.
  std::vector<int> P, Q, Pinv, Qinv;

  // Row scaling and reciprocal flag.
  std::vector<double> Rs;
  bool do_recip = false;

  // L column-wise (for sparse FTRAN forward L solve).
  // Column j: entries L[i][j] for i > j.  Unit diagonal implicit.
  std::vector<int> Lc_start;    // size m+1
  std::vector<int> Lc_index;    // row indices
  std::vector<double> Lc_value;

  // L row-wise (from UMFPACK, for sparse BTRAN backward L^T solve).
  // Row i: entries L[i][j] for j < i.  Diagonal stripped.
  std::vector<int> Lr_start;    // size m+1
  std::vector<int> Lr_index;    // col indices
  std::vector<double> Lr_value;

  // U column-wise (from UMFPACK, for sparse FTRAN backward U solve).
  // Column j: entries U[i][j] for i < j.  Diagonal in Uc_diag.
  std::vector<int> Uc_start;    // size m+1
  std::vector<int> Uc_index;    // row indices
  std::vector<double> Uc_value;
  std::vector<double> Uc_diag;  // U[j][j], size m

  // U row-wise (transposed, for sparse BTRAN forward U^T solve).
  // Row i: entries U[i][j] for j > i.  Diagonal shared with Uc_diag.
  std::vector<int> Ur_start;    // size m+1
  std::vector<int> Ur_index;    // col indices
  std::vector<double> Ur_value;

  // Solve workspace (mutable for const methods).
  mutable std::vector<double> work;
  mutable std::vector<char> mark;       // DFS visited flags (size m)
  mutable std::vector<int> topo;        // topological ordering output (size m)
  mutable std::vector<int> stk_node;    // DFS stack: nodes (size m)
  mutable std::vector<int> stk_edge;    // DFS stack: edge pointers (size m)
  mutable std::vector<int> seeds;       // nonzero seed indices (size m)

#ifdef MIPSOLVERS_HAVE_UMFPACK
  bool extract(void* Numeric, int m_in) {
    m = m_in;
    valid = false;

    // Get factor sizes (int64 via the dl interface — the int32 di counters
    // silently wrapped past 2^31 nnz).
    int64_t lnz, unz, n_row, n_col, nz_udiag;
    if (umfpack_dl_get_lunz(&lnz, &unz, &n_row, &n_col, &nz_udiag,
                            Numeric) != UMFPACK_OK)
      return false;
    if (n_row != m || n_col != m) return false;
    if (lnz > std::numeric_limits<int>::max() ||
        unz > std::numeric_limits<int>::max()) {
      // The internal CCS/CSR caches below are int32; fail loudly and let
      // the caller fall back to the (fully int64) UMFPACK wsolve path.
      return false;
    }

    // Temporary extraction buffers.
    std::vector<int64_t> Lp(m + 1), Lj(lnz);
    std::vector<double> Lx(lnz);
    std::vector<int64_t> Up(m + 1), Ui(unz);
    std::vector<double> Ux(unz);
    std::vector<int64_t> P64(m), Q64(m);
    Rs.resize(m);
    int64_t do_recip_int;

    int status = umfpack_dl_get_numeric(
        Lp.data(), Lj.data(), Lx.data(),
        Up.data(), Ui.data(), Ux.data(),
        P64.data(), Q64.data(),
        nullptr, &do_recip_int, Rs.data(), Numeric);
    if (status != UMFPACK_OK) return false;
    do_recip = (do_recip_int != 0);

    // Build inverse permutations (values < m fit int32).
    P.resize(m);
    Q.resize(m);
    Pinv.resize(m);
    Qinv.resize(m);
    for (int k = 0; k < m; ++k) {
      P[k] = static_cast<int>(P64[k]);
      Q[k] = static_cast<int>(Q64[k]);
      Pinv[P[k]] = k;
      Qinv[Q[k]] = k;
    }

    // ── L row-wise (strip diagonal from UMFPACK output) ──
    {
      Lr_start.resize(m + 1);
      int lr_nnz = 0;
      for (int i = 0; i < m; ++i) {
        Lr_start[i] = lr_nnz;
        lr_nnz += static_cast<int>(
            std::max<int64_t>(0, Lp[i + 1] - Lp[i] - 1));  // exclude diagonal
      }
      Lr_start[m] = lr_nnz;
      Lr_index.resize(lr_nnz);
      Lr_value.resize(lr_nnz);
      for (int i = 0; i < m; ++i) {
        int dst = Lr_start[i];
        const int end = static_cast<int>(Lp[i + 1]) - 1;  // skip last (diagonal=1)
        for (int k = static_cast<int>(Lp[i]); k < end; ++k) {
          Lr_index[dst] = static_cast<int>(Lj[k]);
          Lr_value[dst] = Lx[k];
          ++dst;
        }
      }
    }

    // ── L column-wise (transpose of row-wise) ──
    {
      std::vector<int> cc(m, 0);
      for (int i = 0; i < m; ++i)
        for (int k = Lr_start[i]; k < Lr_start[i + 1]; ++k)
          cc[Lr_index[k]]++;
      Lc_start.resize(m + 1);
      Lc_start[0] = 0;
      for (int j = 0; j < m; ++j)
        Lc_start[j + 1] = Lc_start[j] + cc[j];
      const int lc_nnz = Lc_start[m];
      Lc_index.resize(lc_nnz);
      Lc_value.resize(lc_nnz);
      std::fill(cc.begin(), cc.end(), 0);
      for (int i = 0; i < m; ++i) {
        for (int k = Lr_start[i]; k < Lr_start[i + 1]; ++k) {
          const int j = Lr_index[k];
          const int pos = Lc_start[j] + cc[j]++;
          Lc_index[pos] = i;
          Lc_value[pos] = Lr_value[k];
        }
      }
    }

    // ── U column-wise (strip diagonal from UMFPACK output) ──
    {
      Uc_diag.resize(m);
      Uc_start.resize(m + 1);
      int uc_nnz = 0;
      for (int j = 0; j < m; ++j) {
        Uc_start[j] = uc_nnz;
        const int cnt = static_cast<int>(Up[j + 1] - Up[j]);
        if (cnt > 0) {
          Uc_diag[j] = Ux[static_cast<int>(Up[j + 1]) - 1];  // last entry = diagonal
          uc_nnz += cnt - 1;                   // off-diagonal count
        } else {
          Uc_diag[j] = 0.0;
        }
      }
      Uc_start[m] = uc_nnz;
      Uc_index.resize(uc_nnz);
      Uc_value.resize(uc_nnz);
      for (int j = 0; j < m; ++j) {
        int dst = Uc_start[j];
        const int end = static_cast<int>(Up[j + 1]) - 1;  // skip diagonal
        for (int k = static_cast<int>(Up[j]); k < end; ++k) {
          Uc_index[dst] = static_cast<int>(Ui[k]);
          Uc_value[dst] = Ux[k];
          ++dst;
        }
      }
    }

    // ── U row-wise (transpose of column-wise) ──
    {
      std::vector<int> rc(m, 0);
      for (int j = 0; j < m; ++j)
        for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k)
          rc[Uc_index[k]]++;
      Ur_start.resize(m + 1);
      Ur_start[0] = 0;
      for (int i = 0; i < m; ++i)
        Ur_start[i + 1] = Ur_start[i] + rc[i];
      const int ur_nnz = Ur_start[m];
      Ur_index.resize(ur_nnz);
      Ur_value.resize(ur_nnz);
      std::fill(rc.begin(), rc.end(), 0);
      for (int j = 0; j < m; ++j) {
        for (int k = Uc_start[j]; k < Uc_start[j + 1]; ++k) {
          const int i = Uc_index[k];
          const int pos = Ur_start[i] + rc[i]++;
          Ur_index[pos] = j;    // column j in row i
          Ur_value[pos] = Uc_value[k];
        }
      }
    }

    // Workspace.
    work.assign(m, 0.0);
    mark.assign(m, 0);
    topo.resize(m);
    stk_node.resize(m);
    stk_edge.resize(m);
    seeds.resize(m);
    l_is_identity = (Lc_start[m] == 0);
    valid = true;

    return true;
  }
#endif  // MIPSOLVERS_HAVE_UMFPACK

  // DFS-based sparse reachability for a lower-triangular graph.
  // Adjacency: adj_start[j]..adj_start[j+1]-1 in adj_index (children > j).
  // Returns reachable set in `topo` (reverse topological = descending order).
  void dfs_reach(const int* seeds, int n_seeds,
                 const int* adj_start, const int* adj_index,
                 int& topo_count) const {
    topo_count = 0;
    for (int s = 0; s < n_seeds; ++s) {
      int node = seeds[s];
      if (mark[node]) continue;
      int sptr = 0;
      stk_node[0] = node;
      stk_edge[0] = adj_start[node];
      mark[node] = 1;
      while (sptr >= 0) {
        const int cur = stk_node[sptr];
        int& ep = stk_edge[sptr];
        if (ep < adj_start[cur + 1]) {
          const int child = adj_index[ep++];
          if (!mark[child]) {
            mark[child] = 1;
            ++sptr;
            stk_node[sptr] = child;
            stk_edge[sptr] = adj_start[child];
          }
        } else {
          topo[topo_count++] = cur;
          --sptr;
        }
      }
    }
    // Clear marks.
    for (int i = 0; i < topo_count; ++i) mark[topo[i]] = 0;
  }

  // Sparse FTRAN: solve A * x = rhs.
  // DFS through L to find reachable entries; U backward in scan mode.
  // When out_nz is non-null AND L=I, returns nonzero indices in result
  // via O(nnz) topo walk instead of O(m) scan.
  void ftran(const double* rhs, double* result,
             std::vector<int>* out_nz = nullptr) const {
    constexpr double tol = 1e-20;

    // Step 1: c = P * inv_scale(rhs).  Collect nonzero seeds.
    int n_seeds = 0;
    for (int k = 0; k < m; ++k) {
      const double v = do_recip ? rhs[P[k]] * Rs[P[k]]
                                : rhs[P[k]] / Rs[P[k]];
      if (v != 0.0) {
        work[k] = v;
        seeds[n_seeds++] = k;
      }
    }

    int topo_count_li = 0;  // L=I topo count for sparse Step 5.
    if (l_is_identity) {
      // Fast path: L=I, skip L solve.  U backward via DFS through Uc.
      int topo_count = 0;
      dfs_reach(seeds.data(), n_seeds,
                Uc_start.data(), Uc_index.data(), topo_count);
      topo_count_li = topo_count;
      // Process topo in reverse (descending order for U backward solve).
      for (int t = topo_count - 1; t >= 0; --t) {
        const int j = topo[t];
        if (std::abs(work[j]) > tol) {
          work[j] /= Uc_diag[j];
          const double wj = work[j];
          for (int k = Uc_start[j], e = Uc_start[j + 1]; k < e; ++k)
            work[Uc_index[k]] -= Uc_value[k] * wj;
        } else {
          work[j] = 0.0;
        }
      }
    } else {
      // General path: L forward DFS + solve, then U backward scan.
      int topo_count = 0;
      dfs_reach(seeds.data(), n_seeds,
                Lc_start.data(), Lc_index.data(), topo_count);
      for (int t = topo_count - 1; t >= 0; --t) {
        const int j = topo[t];
        const double wj = work[j];
        if (std::abs(wj) > tol) {
          for (int k = Lc_start[j], e = Lc_start[j + 1]; k < e; ++k)
            work[Lc_index[k]] -= Lc_value[k] * wj;
        }
      }
      for (int j = m - 1; j >= 0; --j) {
        if (work[j] != 0.0 && std::abs(work[j]) > tol) {
          work[j] /= Uc_diag[j];
          const double wj = work[j];
          for (int k = Uc_start[j], e = Uc_start[j + 1]; k < e; ++k)
            work[Uc_index[k]] -= Uc_value[k] * wj;
        } else {
          work[j] = 0.0;
        }
      }
    }

    // Step 5: result = Q * work, clear work.
    if (out_nz && l_is_identity) {
      // Sparse path: only iterate topo entries (which include all seeds).
      out_nz->clear();
      out_nz->reserve(topo_count_li);
      for (int t = 0; t < topo_count_li; ++t) {
        const int k = topo[t];
        const double v = work[k];
        work[k] = 0.0;
        if (v != 0.0) {
          const int orig = Q[k];
          result[orig] = v;
          out_nz->push_back(orig);
        }
      }
    } else {
      if (out_nz) out_nz->clear();
      for (int k = 0; k < m; ++k) {
        const double v = work[k];
        work[k] = 0.0;
        result[Q[k]] = v;
        if (out_nz && v != 0.0) out_nz->push_back(Q[k]);
      }
    }
  }

  // Sparse BTRAN: solve A^T * x = rhs.
  // DFS through U^T (row graph) for forward solve; L^T backward in scan mode.
  // Also returns nonzero indices of the result in `out_nz` if non-null.
  void btran(const double* rhs, double* result,
             std::vector<int>* out_nz = nullptr) const {
    constexpr double tol = 1e-20;

    // Step 1: c = Q^T * rhs.  Collect nonzero seeds.
    int n_seeds = 0;
    for (int k = 0; k < m; ++k) {
      const double v = rhs[Q[k]];
      if (v != 0.0) {
        work[k] = v;
        seeds[n_seeds++] = k;
      }
    }

    // Step 2: DFS reachability through U row graph (U^T lower triangular).
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds,
              Ur_start.data(), Ur_index.data(), topo_count);

    // Step 3: U^T forward solve — process topo in reverse (ascending order).
    for (int t = topo_count - 1; t >= 0; --t) {
      const int i = topo[t];
      if (std::abs(work[i]) > tol) {
        work[i] /= Uc_diag[i];
        const double wi = work[i];
        for (int k = Ur_start[i], e = Ur_start[i + 1]; k < e; ++k)
          work[Ur_index[k]] -= Ur_value[k] * wi;
      } else {
        work[i] = 0.0;
      }
    }

    // Step 4: L^T backward solve — skip when L=I.
    if (!l_is_identity) {
      for (int i = m - 1; i >= 0; --i) {
        if (work[i] != 0.0 && std::abs(work[i]) > tol) {
          const double wi = work[i];
          for (int k = Lr_start[i], e = Lr_start[i + 1]; k < e; ++k)
            work[Lr_index[k]] -= Lr_value[k] * wi;
        }
      }
    }

    // Step 5: result = inv_scale * P^T * work, clear work.
    // When L=I and out_nz requested, use topo for sparse output.
    if (l_is_identity && out_nz) {
      out_nz->clear();
      if (do_recip) {
        for (int t = 0; t < topo_count; ++t) {
          const int k = topo[t];
          if (work[k] != 0.0) {
            result[P[k]] = work[k] * Rs[P[k]];
            out_nz->push_back(P[k]);
          }
          work[k] = 0.0;
        }
      } else {
        for (int t = 0; t < topo_count; ++t) {
          const int k = topo[t];
          if (work[k] != 0.0) {
            result[P[k]] = work[k] / Rs[P[k]];
            out_nz->push_back(P[k]);
          }
          work[k] = 0.0;
        }
      }
      // Also clear seed entries that might not be in topo.
      for (int s = 0; s < n_seeds; ++s) {
        const int k = seeds[s];
        if (work[k] != 0.0) {
          result[P[k]] = do_recip ? work[k] * Rs[P[k]] : work[k] / Rs[P[k]];
          out_nz->push_back(P[k]);
          work[k] = 0.0;
        }
      }
    } else if (do_recip) {
      for (int k = 0; k < m; ++k) {
        result[P[k]] = work[k] * Rs[P[k]];
        work[k] = 0.0;
      }
      if (out_nz) {
        out_nz->clear();
        for (int i = 0; i < m; ++i)
          if (result[i] != 0.0) out_nz->push_back(i);
      }
    } else {
      for (int k = 0; k < m; ++k) {
        result[P[k]] = work[k] / Rs[P[k]];
        work[k] = 0.0;
      }
      if (out_nz) {
        out_nz->clear();
        for (int i = 0; i < m; ++i)
          if (result[i] != 0.0) out_nz->push_back(i);
      }
    }
  }

  // ── BTRAN with pre-computed sparse RHS indices ──
  // Like btran() but skips the O(m) scan in Step 1 by using caller-provided
  // nonzero indices.  rhs_nz_idx[0..rhs_nnz-1] are ORIGINAL-space indices
  // where rhs is nonzero.  Must be complete (no missing nonzeros).
  void btran_preseeded(const double* rhs, int rhs_nnz, const int* rhs_nz_idx,
                       double* result, std::vector<int>* out_nz = nullptr) const {
    constexpr double tol = 1e-20;

    // Step 1: Sparse Q^T * rhs using only nonzero positions.
    int n_seeds = 0;
    for (int s = 0; s < rhs_nnz; ++s) {
      const int orig = rhs_nz_idx[s];
      const int k = Qinv[orig];
      work[k] = rhs[orig];
      seeds[n_seeds++] = k;
    }

    // Steps 2-5: Same as btran().
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds,
              Ur_start.data(), Ur_index.data(), topo_count);
    for (int t = topo_count - 1; t >= 0; --t) {
      const int i = topo[t];
      if (std::abs(work[i]) > tol) {
        work[i] /= Uc_diag[i];
        const double wi = work[i];
        for (int k = Ur_start[i], e = Ur_start[i + 1]; k < e; ++k)
          work[Ur_index[k]] -= Ur_value[k] * wi;
      } else {
        work[i] = 0.0;
      }
    }
    if (!l_is_identity) {
      for (int i = m - 1; i >= 0; --i) {
        if (work[i] != 0.0 && std::abs(work[i]) > tol) {
          const double wi = work[i];
          for (int k = Lr_start[i], e = Lr_start[i + 1]; k < e; ++k)
            work[Lr_index[k]] -= Lr_value[k] * wi;
        }
      }
    }
    // Step 5: sparse output when L=I and out_nz requested.
    if (l_is_identity && out_nz) {
      out_nz->clear();
      for (int t = 0; t < topo_count; ++t) {
        const int k = topo[t];
        if (work[k] != 0.0) {
          const int orig = P[k];
          result[orig] = do_recip ? work[k] * Rs[orig] : work[k] / Rs[orig];
          out_nz->push_back(orig);
        }
        work[k] = 0.0;
      }
      for (int s = 0; s < n_seeds; ++s) {
        const int k = seeds[s];
        if (work[k] != 0.0) {
          const int orig = P[k];
          result[orig] = do_recip ? work[k] * Rs[orig] : work[k] / Rs[orig];
          out_nz->push_back(orig);
          work[k] = 0.0;
        }
      }
    } else if (do_recip) {
      for (int k = 0; k < m; ++k) {
        result[P[k]] = work[k] * Rs[P[k]];
        work[k] = 0.0;
      }
      if (out_nz) {
        out_nz->clear();
        for (int i = 0; i < m; ++i)
          if (result[i] != 0.0) out_nz->push_back(i);
      }
    } else {
      for (int k = 0; k < m; ++k) {
        result[P[k]] = work[k] / Rs[P[k]];
        work[k] = 0.0;
      }
      if (out_nz) {
        out_nz->clear();
        for (int i = 0; i < m; ++i)
          if (result[i] != 0.0) out_nz->push_back(i);
      }
    }
  }

  // ── Fully sparse FTRAN for L=I case ── O(nnz_input + nnz_U) per call.
  // Input: sparse rhs given by indices/values in ORIGINAL (row) space.
  // Output: writes nonzeros to pre-zeroed `result` buffer, fills out_nz.
  void ftran_sparse(const int* rhs_idx, const double* rhs_val, int rhs_nnz,
                    double* result, int* out_nz_idx, int& out_nnz) const {
    constexpr double tol = 1e-20;
    // Step 1: c = P * inv_scale(rhs). Sparse input — O(rhs_nnz).
    int n_seeds = 0;
    for (int s = 0; s < rhs_nnz; ++s) {
      const int orig = rhs_idx[s];
      const double v = do_recip ? rhs_val[s] * Rs[orig]
                                : rhs_val[s] / Rs[orig];
      if (v != 0.0) {
        const int k = Pinv[orig];
        work[k] = v;
        seeds[n_seeds++] = k;
      }
    }
    // Steps 2-4: U backward DFS (L=I assumed).
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds,
              Uc_start.data(), Uc_index.data(), topo_count);
    for (int t = topo_count - 1; t >= 0; --t) {
      const int j = topo[t];
      if (std::abs(work[j]) > tol) {
        work[j] /= Uc_diag[j];
        const double wj = work[j];
        for (int k = Uc_start[j], e = Uc_start[j + 1]; k < e; ++k)
          work[Uc_index[k]] -= Uc_value[k] * wj;
      } else {
        work[j] = 0.0;
      }
    }
    // Step 5: sparse output via topo — O(topo_count).
    out_nnz = 0;
    for (int t = 0; t < topo_count; ++t) {
      const int k = topo[t];
      if (work[k] != 0.0) {
        const int orig = Q[k];
        result[orig] = work[k];
        out_nz_idx[out_nnz++] = orig;
        work[k] = 0.0;
      }
    }
  }

  // ── Fully sparse BTRAN for L=I case ── O(nnz_input + nnz_U) per call.
  // Input: sparse rhs given by indices/values in ORIGINAL (row) space.
  // Output: writes nonzeros to pre-zeroed `result` buffer, fills out_nz.
  void btran_sparse(const int* rhs_idx, const double* rhs_val, int rhs_nnz,
                    double* result, int* out_nz_idx, int& out_nnz) const {
    constexpr double tol = 1e-20;
    // Step 1: c = Q^T * rhs. Sparse input — O(rhs_nnz).
    int n_seeds = 0;
    for (int s = 0; s < rhs_nnz; ++s) {
      const int orig = rhs_idx[s];
      const int k = Qinv[orig];
      work[k] = rhs_val[s];
      seeds[n_seeds++] = k;
    }
    // Steps 2-3: DFS U^T + forward solve.
    int topo_count = 0;
    dfs_reach(seeds.data(), n_seeds,
              Ur_start.data(), Ur_index.data(), topo_count);
    for (int t = topo_count - 1; t >= 0; --t) {
      const int i = topo[t];
      if (std::abs(work[i]) > tol) {
        work[i] /= Uc_diag[i];
        const double wi = work[i];
        for (int k = Ur_start[i], e = Ur_start[i + 1]; k < e; ++k)
          work[Ur_index[k]] -= Ur_value[k] * wi;
      } else {
        work[i] = 0.0;
      }
    }
    // Step 4: L=I, skip.
    // Step 5: sparse output via topo — O(topo_count).
    out_nnz = 0;
    if (do_recip) {
      for (int t = 0; t < topo_count; ++t) {
        const int k = topo[t];
        if (work[k] != 0.0) {
          const int orig = P[k];
          result[orig] = work[k] * Rs[orig];
          out_nz_idx[out_nnz++] = orig;
          work[k] = 0.0;
        }
      }
    } else {
      for (int t = 0; t < topo_count; ++t) {
        const int k = topo[t];
        if (work[k] != 0.0) {
          const int orig = P[k];
          result[orig] = work[k] / Rs[orig];
          out_nz_idx[out_nnz++] = orig;
          work[k] = 0.0;
        }
      }
    }
  }
};

// Minimum problem size to enable NativeLU sparse solves.
constexpr int kNativeLUThreshold = 0;  // Always use native sparse LU + FT update

class SparseBasis : public BasisOps {
 public:
  enum class FactorBackendKind {
    UmfpackNativeA = 0,
    HiGHSSafeB = 1,
    HiGHSForceFTB = 2,
    HiGHSShortChainB = 3,
    // U.7.118 Phase 3: vendored HiGHS HFactor as a drop-in basis-LU backend.
    // Opt-in via environment variable `MIPSOLVERS_FACTOR_BACKEND=hfactor`; the
    // public SimplexFactorBackend enum is intentionally not extended yet
    // until Phase 4 benchmarks justify a default-on flip.
    HFactorPort = 4,
    // Stage 2 rescue backend: KLU LU with two fresh factorizations (B and
    // B^T) per refactorize() and standard eta updates between them.
    // Selected only by the cold-start escalation chain at level >= 2;
    // never via the public SimplexFactorBackend ids.
    KluRescue = 5,
  };

  static FactorBackendKind from_public_backend(SimplexFactorBackend backend) {
    switch (backend) {
      case SimplexFactorBackend::BackendB_ShortChain:
        return FactorBackendKind::HiGHSShortChainB;
      case SimplexFactorBackend::BackendB_ForceFT:
        return FactorBackendKind::HiGHSForceFTB;
      case SimplexFactorBackend::BackendB_HiGHSSafe:
        return FactorBackendKind::HiGHSSafeB;
      case SimplexFactorBackend::BackendA_UmfpackNative:
      default:
        return FactorBackendKind::UmfpackNativeA;
    }
  }

  explicit SparseBasis(const Eigen::SparseMatrix<double>& A,
                       FactorBackendKind backend_kind =
                           FactorBackendKind::UmfpackNativeA)
      : A_(&A), m_(static_cast<int>(A.rows())), min_pivot_(1e30), max_eta_norm_(0.0),
        gen_(0), backend_kind_(backend_kind) {
    // U.7.118 Phase 3 opt-in: env var override → HFactorPort.
    if (const char* e = std::getenv("MIPSOLVERS_FACTOR_BACKEND")) {
      if (e[0] == 'h' || e[0] == 'H') {  // "hfactor" / "HFACTOR"
        backend_kind_ = FactorBackendKind::HFactorPort;
      }
    }
#ifdef MIPSOLVERS_HAVE_UMFPACK
    // Initialize UMFPACK control with defaults, then customize for simplex:
    // - Iterative refinement on (max 2 steps, UMFPACK early-stops): the FT
    //   update chain accumulates error between refactorizations; IR restores
    //   solve accuracy on ill-conditioned bases for one extra SpMV + sparse
    //   triangular solve only when the residual demands it.
    // - Use row-sum scaling for better numerical conditioning
    umfpack_dl_defaults(Control_);
    Control_[UMFPACK_IRSTEP] = 2;
    Control_[UMFPACK_SCALE] = UMFPACK_SCALE_SUM;
    // Pre-allocate solve workspace (avoids per-call heap allocation).
    solve_work_.resize(m_);
    // Pre-allocate wsolve workspace: 5*m doubles + m ints.
    wsolve_W_.resize(5 * m_);
    wsolve_Wi_.resize(m_);
#endif
  }

  ~SparseBasis() {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (Numeric_) umfpack_dl_free_numeric(&Numeric_);
    if (Symbolic_) umfpack_dl_free_symbolic(&Symbolic_);
#endif
  }
  SparseBasis(const SparseBasis&) = delete;
  SparseBasis& operator=(const SparseBasis&) = delete;

  /// Override UMFPACK's partial-pivoting tolerance (0,1]; 1.0 requests true
  /// partial pivoting.  Takes effect at the next refactorize() call.  Used by
  /// the cold-start escalation chain; no-op without UMFPACK.
  void set_pivot_tolerance(double tol) {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (tol > 0.0 && tol <= 1.0) Control_[UMFPACK_PIVOT_TOLERANCE] = tol;
#else
    (void)tol;
#endif
  }

  BasisOpsKind kind() const override { return BasisOpsKind::NativeSparse; }

  /// Rebind the constraint matrix reference (for dirty basis reuse across SF instances).
  void rebind_A(const Eigen::SparseMatrix<double>& A) override { A_ = &A; }

  bool bound_to_A(const Eigen::SparseMatrix<double>& A) const override {
    return A_ == &A && A.rows() == m_;
  }

  bool has_appended_parent_factor() const {
    return static_cast<bool>(appended_parent_);
  }

  bool init_appended_slack_extension(
      const std::shared_ptr<SparseBasis>& parent,
      const std::vector<int>& basis,
      int old_rows,
      int old_m_ineq,
      int n_new_rows) {
    if (!parent || old_rows <= 0 || n_new_rows <= 0 ||
        parent.get() == this || parent->m_ != old_rows ||
        m_ != old_rows + n_new_rows ||
        static_cast<int>(basis.size()) != m_ ||
        old_m_ineq < 0 || old_m_ineq > old_rows) {
      return false;
    }
    if (A_ == nullptr || A_->rows() != m_) return false;
    if (parent->A_ == nullptr ||
        static_cast<int>(parent->basis_columns_.size()) != old_rows ||
        parent->A_->rows() != old_rows) {
      return false;
    }

    std::vector<int> old_to_new(static_cast<std::size_t>(old_rows), -1);
    for (int i = 0; i < old_rows; ++i) {
      old_to_new[static_cast<std::size_t>(i)] =
          (i < old_m_ineq) ? i : i + n_new_rows;
    }

    std::vector<double> cut_diag(static_cast<std::size_t>(n_new_rows), 0.0);
    std::vector<std::vector<std::pair<int, double>>> c_by_cut(
        static_cast<std::size_t>(n_new_rows));
    struct ScaleEdge {
      int row{0};
      int col{0};
      double ratio{1.0};
    };
    std::vector<ScaleEdge> scale_edges;
    scale_edges.reserve(static_cast<std::size_t>(
        std::max<Eigen::Index>(0, parent->A_->nonZeros())));

    for (int k = 0; k < n_new_rows; ++k) {
      const int cut_row = old_m_ineq + k;
      const int slack_col = basis[static_cast<std::size_t>(cut_row)];
      if (slack_col < 0 || slack_col >= A_->cols()) return false;
      for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, slack_col); it;
           ++it) {
        if (it.row() == cut_row) {
          cut_diag[static_cast<std::size_t>(k)] = it.value();
          break;
        }
      }
      if (std::abs(cut_diag[static_cast<std::size_t>(k)]) < 1e-12 ||
          !std::isfinite(cut_diag[static_cast<std::size_t>(k)])) {
        return false;
      }
    }

    for (int old_pos = 0; old_pos < old_rows; ++old_pos) {
      const int new_row = old_to_new[static_cast<std::size_t>(old_pos)];
      const int col = basis[static_cast<std::size_t>(new_row)];
      if (col < 0 || col >= A_->cols()) return false;
      const int parent_col =
          parent->basis_columns_[static_cast<std::size_t>(old_pos)];
      if (parent_col < 0 || parent_col >= parent->A_->cols()) return false;

      std::vector<std::pair<int, double>> current_old_entries;
      current_old_entries.reserve(16);
      for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, col); it; ++it) {
        const int row = static_cast<int>(it.row());
        if (row < old_m_ineq) {
          current_old_entries.push_back({row, it.value()});
          continue;
        }
        if (row >= old_m_ineq + n_new_rows && row < m_) {
          current_old_entries.push_back({row - n_new_rows, it.value()});
          continue;
        }
        if (row < old_m_ineq || row >= old_m_ineq + n_new_rows) continue;
        const double v = it.value();
        if (std::abs(v) <= 1e-15) continue;
        c_by_cut[static_cast<std::size_t>(row - old_m_ineq)].push_back(
            {old_pos, v});
      }
      std::sort(current_old_entries.begin(), current_old_entries.end(),
                [](const auto& a, const auto& b) {
                  return a.first < b.first;
                });
      std::vector<char> matched_current(
          current_old_entries.size(), static_cast<char>(0));
      for (Eigen::SparseMatrix<double>::InnerIterator pit(*parent->A_,
                                                          parent_col);
           pit; ++pit) {
        const int old_row = static_cast<int>(pit.row());
        if (old_row < 0 || old_row >= old_rows) return false;
        const double pv = pit.value();
        if (std::abs(pv) <= 1e-15) continue;
        const auto it = std::lower_bound(
            current_old_entries.begin(), current_old_entries.end(), old_row,
            [](const auto& entry, int row) { return entry.first < row; });
        if (it == current_old_entries.end() || it->first != old_row) {
          return false;
        }
        const std::size_t idx =
            static_cast<std::size_t>(it - current_old_entries.begin());
        matched_current[idx] = 1;
        const double ratio = it->second / pv;
        if (!std::isfinite(ratio) || ratio <= 0.0) return false;
        scale_edges.push_back({old_row, old_pos, ratio});
      }
      for (std::size_t idx = 0; idx < current_old_entries.size(); ++idx) {
        if (!matched_current[idx] &&
            std::abs(current_old_entries[idx].second) > 1e-12) {
          return false;
        }
      }
    }

    std::vector<double> old_row_factor(static_cast<std::size_t>(old_rows),
                                       std::numeric_limits<double>::quiet_NaN());
    std::vector<double> old_basis_col_factor(
        static_cast<std::size_t>(old_rows),
        std::numeric_limits<double>::quiet_NaN());
    std::vector<std::vector<std::pair<int, double>>> row_edges(
        static_cast<std::size_t>(old_rows));
    std::vector<std::vector<std::pair<int, double>>> col_edges(
        static_cast<std::size_t>(old_rows));
    for (const auto& edge : scale_edges) {
      row_edges[static_cast<std::size_t>(edge.row)].push_back(
          {edge.col, edge.ratio});
      col_edges[static_cast<std::size_t>(edge.col)].push_back(
          {edge.row, edge.ratio});
    }
    constexpr double kScaleConsistencyTol = 1e-8;
    for (int seed = 0; seed < old_rows; ++seed) {
      if (std::isfinite(old_row_factor[static_cast<std::size_t>(seed)])) {
        continue;
      }
      if (row_edges[static_cast<std::size_t>(seed)].empty()) {
        old_row_factor[static_cast<std::size_t>(seed)] = 1.0;
        continue;
      }
      std::deque<std::pair<char, int>> queue;
      old_row_factor[static_cast<std::size_t>(seed)] = 1.0;
      queue.push_back({'r', seed});
      while (!queue.empty()) {
        const auto [kind, idx] = queue.front();
        queue.pop_front();
        if (kind == 'r') {
          const double rf = old_row_factor[static_cast<std::size_t>(idx)];
          if (!std::isfinite(rf) || rf <= 0.0) return false;
          for (const auto& [col_idx, ratio] :
               row_edges[static_cast<std::size_t>(idx)]) {
            const double cf = ratio / rf;
            double& slot =
                old_basis_col_factor[static_cast<std::size_t>(col_idx)];
            if (std::isfinite(slot)) {
              const double denom = std::max(1.0, std::abs(slot));
              if (std::abs(slot - cf) > kScaleConsistencyTol * denom) {
                return false;
              }
            } else {
              if (!std::isfinite(cf) || cf <= 0.0) return false;
              slot = cf;
              queue.push_back({'c', col_idx});
            }
          }
        } else {
          const double cf =
              old_basis_col_factor[static_cast<std::size_t>(idx)];
          if (!std::isfinite(cf) || cf <= 0.0) return false;
          for (const auto& [row_idx, ratio] :
               col_edges[static_cast<std::size_t>(idx)]) {
            const double rf = ratio / cf;
            double& slot =
                old_row_factor[static_cast<std::size_t>(row_idx)];
            if (std::isfinite(slot)) {
              const double denom = std::max(1.0, std::abs(slot));
              if (std::abs(slot - rf) > kScaleConsistencyTol * denom) {
                return false;
              }
            } else {
              if (!std::isfinite(rf) || rf <= 0.0) return false;
              slot = rf;
              queue.push_back({'r', row_idx});
            }
          }
        }
      }
    }
    for (int i = 0; i < old_rows; ++i) {
      double& rf = old_row_factor[static_cast<std::size_t>(i)];
      double& cf = old_basis_col_factor[static_cast<std::size_t>(i)];
      if (!std::isfinite(rf)) rf = 1.0;
      if (!std::isfinite(cf)) cf = 1.0;
      if (rf <= 0.0 || cf <= 0.0) return false;
    }

    appended_parent_ = parent;
    appended_old_rows_ = old_rows;
    appended_old_m_ineq_ = old_m_ineq;
    appended_new_rows_ = n_new_rows;
    appended_old_to_new_row_ = std::move(old_to_new);
    appended_cut_diag_ = std::move(cut_diag);
    appended_c_by_cut_ = std::move(c_by_cut);
    appended_old_row_factor_ = std::move(old_row_factor);
    appended_old_basis_col_factor_ = std::move(old_basis_col_factor);
    basis_columns_ = basis;
    etas_.clear();
    min_pivot_ = 1e30;
    max_eta_norm_ = 0.0;
    accumulated_fill_ = 0;
    ++gen_;
    return true;
  }

  FactorBackendKind factor_backend_kind() const { return backend_kind_; }

  bool refactorize(const std::vector<int>& basis) {
    ++gen_;
    ++refactor_count_;
    clear_appended_slack_extension();
    basis_columns_ = basis;
    // FT diagnostics: show success/fail/chain-broken rates between
    // refactorisations.  Gated by env var so zero cost when disabled.
    static const bool ft_diag = []() {
      const char* e = std::getenv("MIPSOLVERS_P9_FT_DIAG");
      return e && e[0] == '1';
    }();
    if (ft_diag && refactor_count_ > 1) {
      std::fprintf(stderr,
          "[FT-DIAG] m=%d kind=%d refactor#%d  ft_ok=%d ft_fail=%d chain_broken=%d "
          "slu_n_upd=%d etas=%d slu_valid=%d\n",
          m_, static_cast<int>(backend_kind_), refactor_count_,
          ft_success_count_, ft_fail_count_,
          ft_chain_broken_refactor_count_,
#ifdef MIPSOLVERS_HAVE_UMFPACK
          slu_.n_updates,
          static_cast<int>(etas_.size()), slu_.valid ? 1 : 0
#else
          0, static_cast<int>(etas_.size()), 0
#endif
          );
    }
    ft_success_count_ = 0;
    ft_fail_count_ = 0;
    ft_chain_broken_refactor_count_ = 0;
    // Check for duplicate basis columns (singular basis).
    {
      std::vector<char> seen(static_cast<size_t>(A_->cols()), 0);
      for (int i = 0; i < m_; ++i) {
        const int col = basis[static_cast<size_t>(i)];
        if (col < 0 || col >= A_->cols()) return false;
        if (seen[static_cast<size_t>(col)]) return false;
        seen[static_cast<size_t>(col)] = 1;
      }
    }
    // U.7.118 Phase 3: HFactorPort path bypasses UMFPACK and Eigen's basis
    // matrix construction entirely.  HFactor consumes A + basic_index
    // directly, performs its own Markowitz-style factor, and serves
    // FTRAN/BTRAN/UPDATE.  All slu_/nlu_/etas state is left disabled.
    if (backend_kind_ == FactorBackendKind::HFactorPort) {
#ifdef MIPSOLVERS_HAVE_UMFPACK
      slu_.valid = false;
#endif
      etas_.clear();
      min_pivot_ = 1e30;
      max_eta_norm_ = 0.0;
      accumulated_fill_ = 0;
#ifdef MIPSOLVERS_HAVE_HFACTOR
      const bool ok = hfb_.factorize(*A_, basis.data(), m_);
      return ok;
#else
      return false;  // HFactorPort requested but MIPSOLVERS_HAVE_HFACTOR not compiled
#endif
    }
    B_ = build_sparse_basis(basis);
    // Stage 2: KLU rescue backend.  Two fresh factorizations (B and B^T) per
    // refactorize; standard eta updates apply between refactorizations.
    // Numerically conservative rescue path — correctness over speed.
    if (backend_kind_ == FactorBackendKind::KluRescue) {
#ifdef MIPSOLVERS_HAVE_KLU
      const Eigen::SparseMatrix<double> Bt = B_.transpose();
      klu_lu_.compute(B_);
      klu_lu_t_.compute(Bt);
      klu_use_fallback_ = (klu_lu_.info() != Eigen::Success) ||
                          (klu_lu_t_.info() != Eigen::Success);
      if (klu_use_fallback_) {
        klu_fallback_lu_.setPivotThreshold(1.0);
        klu_fallback_lu_.compute(B_);
        klu_fallback_lu_t_.setPivotThreshold(1.0);
        klu_fallback_lu_t_.compute(Bt);
        if (klu_fallback_lu_.info() != Eigen::Success ||
            klu_fallback_lu_t_.info() != Eigen::Success) {
          return false;
        }
      }
      etas_.clear();
      min_pivot_ = 1e30;
      max_eta_norm_ = 0.0;
      accumulated_fill_ = 0;
      return true;
#else
      return false;  // KluRescue requested but KLU not compiled
#endif
    }
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (Numeric_) { umfpack_dl_free_numeric(&Numeric_); Numeric_ = nullptr; }

    // Widen B_'s int32 CSC indices to int64 for the umfpack_dl_* interface
    // (factor nnz past 2^31 is representable through this path; the pattern
    // comparison below also runs on these widened arrays).
    const int nnz_b = B_.outerIndexPtr()[m_];
    std::vector<int64_t> Ap64(B_.outerIndexPtr(), B_.outerIndexPtr() + m_ + 1);
    std::vector<int64_t> Ai64(B_.innerIndexPtr(), B_.innerIndexPtr() + nnz_b);

    // Check if sparsity pattern is unchanged — if so, reuse Symbolic_.
    bool pattern_same = false;
    if (Symbolic_ && Ap_.size() == Ap64.size() && Ai_.size() == Ai64.size() &&
        Ap_ == Ap64 && Ai_ == Ai64) {
      pattern_same = true;
    }

    if (!pattern_same) {
      if (Symbolic_) { umfpack_dl_free_symbolic(&Symbolic_); Symbolic_ = nullptr; }
      int status = umfpack_dl_symbolic(m_, m_, Ap64.data(), Ai64.data(),
                                       B_.valuePtr(), &Symbolic_, Control_,
                                       nullptr);
      if (status != UMFPACK_OK) return false;
    }
    {
      int status = umfpack_dl_numeric(Ap64.data(), Ai64.data(),
                                      B_.valuePtr(), Symbolic_, &Numeric_,
                                      Control_, nullptr);
      if (status != UMFPACK_OK) {
        umfpack_dl_free_symbolic(&Symbolic_);
        Symbolic_ = nullptr;
        return false;
      }
    }
    // Cache CCS arrays — UMFPACK solve uses them for iterative refinement.
    // Storing our own copies avoids any lifetime issues with B_'s internals.
    Ap_ = std::move(Ap64);
    Ai_ = std::move(Ai64);
    Ax_.assign(B_.valuePtr(), B_.valuePtr() + nnz_b);
    if (!prepare_factor_backend_after_numeric()) return false;
#else
    lu_.setPivotThreshold(1.0);
    lu_.compute(B_);
    if (lu_.info() != Eigen::Success) {
      return false;
    }
#endif
    etas_.clear();
    min_pivot_ = 1e30;
    max_eta_norm_ = 0.0;
    accumulated_fill_ = 0;
    return true;
  }

  // Check if a vector is sparse enough for NativeLU to be beneficial.
  // NativeLU uses scalar column-based solves; UMFPACK uses BLAS-optimized
  // supernodal solves.  NativeLU wins only when we can skip >90% of entries.
  static bool is_sparse_rhs(const double* v, int n) {
    constexpr double density_thr = 0.05;  // 5% density threshold
    int nnz = 0;
    const int limit = static_cast<int>(density_thr * n) + 1;
    for (int i = 0; i < n; ++i) {
      if (v[i] != 0.0) {
        if (++nnz >= limit) return false;
      }
    }
    return true;
  }

  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override {
    if (has_appended_slack_extension()) {
      Eigen::VectorXd z = solve_appended_slack_ftran(rhs);
      for (const auto& eta : etas_) apply_eta_ftran(z, eta);
      return z;
    }
    Eigen::VectorXd z(m_);
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      hfb_.ftran(rhs.data(), z.data());
      return z;
    }
#ifdef MIPSOLVERS_HAVE_KLU
    if (backend_kind_ == FactorBackendKind::KluRescue) {
      Eigen::VectorXd zr;
      if (klu_use_fallback_) {
        zr = klu_fallback_lu_.solve(rhs);
      } else {
        zr = klu_lu_.solve(rhs);
      }
      for (const auto& eta : etas_) apply_eta_ftran(zr, eta);
      return zr;
    }
#endif
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      slu_.ftran(rhs.data(), z.data());
    } else if (nlu_.valid && is_sparse_rhs(rhs.data(), m_)) {
      nlu_.ftran(rhs.data(), z.data());
      for (const auto& eta : etas_) apply_eta_ftran(z, eta);
    } else {
      int status = umfpack_dl_wsolve(UMFPACK_A, Ap_.data(), Ai_.data(),
                                     Ax_.data(), z.data(), rhs.data(), Numeric_,
                                     Control_, nullptr, wsolve_Wi_.data(),
                                     wsolve_W_.data());
      if (status != UMFPACK_OK) {
        // Solve failed: poison with NaN so downstream isfinite guards abort
        // instead of consuming garbage (same pattern as VendoredHighsBasis).
        return Eigen::VectorXd::Constant(
            m_, std::numeric_limits<double>::quiet_NaN());
      }
      for (const auto& eta : etas_) apply_eta_ftran(z, eta);
    }
#else
    z = lu_.solve(rhs);
    for (const auto& eta : etas_) apply_eta_ftran(z, eta);
#endif
    return z;
  }

  /// In-place FTRAN: solve B*x = rhs, overwriting rhs with solution.
  void ftran_inplace(Eigen::VectorXd& rhs) const {
    if (has_appended_slack_extension()) {
      rhs = solve_appended_slack_ftran(rhs);
      for (const auto& eta : etas_) apply_eta_ftran(rhs, eta);
      return;
    }
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      solve_work_.noalias() = rhs;
      hfb_.ftran(solve_work_.data(), rhs.data());
      return;
    }
#ifdef MIPSOLVERS_HAVE_KLU
    if (backend_kind_ == FactorBackendKind::KluRescue) {
      if (klu_use_fallback_) {
        rhs = klu_fallback_lu_.solve(rhs);
      } else {
        rhs = klu_lu_.solve(rhs);
      }
      for (const auto& eta : etas_) apply_eta_ftran(rhs, eta);
      return;
    }
#endif
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      solve_work_.noalias() = rhs;
      slu_.ftran(solve_work_.data(), rhs.data());
    } else if (nlu_.valid && is_sparse_rhs(rhs.data(), m_)) {
      nlu_.ftran(rhs.data(), rhs.data());
      for (const auto& eta : etas_) apply_eta_ftran(rhs, eta);
    } else {
      solve_work_.noalias() = rhs;  // UMFPACK needs separate input/output
      int status = umfpack_dl_wsolve(UMFPACK_A, Ap_.data(), Ai_.data(),
                                     Ax_.data(), rhs.data(), solve_work_.data(),
                                     Numeric_, Control_, nullptr,
                                     wsolve_Wi_.data(), wsolve_W_.data());
      if (status != UMFPACK_OK) {
        // Solve failed: poison with NaN so downstream isfinite guards abort
        // instead of consuming garbage (same pattern as VendoredHighsBasis).
        rhs.setConstant(std::numeric_limits<double>::quiet_NaN());
        return;
      }
      for (const auto& eta : etas_) apply_eta_ftran(rhs, eta);
    }
#else
    rhs = lu_.solve(rhs);
    for (const auto& eta : etas_) apply_eta_ftran(rhs, eta);
#endif
  }

  /// In-place FTRAN with nonzero index extraction.
  /// Overwrites rhs with solution and returns nonzero indices in nz.
  void ftran_sparse_inplace(Eigen::VectorXd& rhs, std::vector<int>& nz) const {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      // slu_.ftran always populates out_nz (both hyper-sparse and dense
      // fallback paths).  Avoids an O(m) post-scan.
      solve_work_.noalias() = rhs;
      nz.clear();
      slu_.ftran(solve_work_.data(), rhs.data(), &nz);
      return;
    }
#endif
    ftran_inplace(rhs);
    nz.clear();
    for (int i = 0; i < m_; ++i) {
      if (rhs[i] != 0.0) nz.push_back(i);
    }
  }

  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override {
    if (has_appended_slack_extension()) {
      Eigen::VectorXd z = rhs;
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(z, etas_[k]);
      return solve_appended_slack_btran(z);
    }
    Eigen::VectorXd y(m_);
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      hfb_.btran(rhs.data(), y.data());
      return y;
    }
#ifdef MIPSOLVERS_HAVE_KLU
    if (backend_kind_ == FactorBackendKind::KluRescue) {
      Eigen::VectorXd zr = rhs;
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(zr, etas_[static_cast<std::size_t>(k)]);
      if (klu_use_fallback_) return klu_fallback_lu_t_.solve(zr);
      return klu_lu_t_.solve(zr);
    }
#endif
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      slu_.btran(rhs.data(), y.data());
    } else {
      Eigen::VectorXd z = rhs;
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(z, etas_[k]);
      if (nlu_.valid && is_sparse_rhs(z.data(), m_)) {
        nlu_.btran(z.data(), y.data());
      } else {
        int status = umfpack_dl_wsolve(UMFPACK_At, Ap_.data(), Ai_.data(),
                                       Ax_.data(), y.data(), z.data(), Numeric_,
                                       Control_, nullptr, wsolve_Wi_.data(),
                                       wsolve_W_.data());
        if (status != UMFPACK_OK) {
          // Solve failed: poison with NaN so downstream isfinite guards abort
          // instead of consuming garbage (same pattern as VendoredHighsBasis).
          return Eigen::VectorXd::Constant(
              m_, std::numeric_limits<double>::quiet_NaN());
        }
      }
    }
#else
    Eigen::VectorXd z = rhs;
    for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
      apply_eta_btran(z, etas_[k]);
    y = lu_.transpose().solve(z);
#endif
    return y;
  }

  /// In-place BTRAN: solve B^T * y = rhs, overwriting rhs with solution.
  void btran_inplace(Eigen::VectorXd& rhs) const {
    if (has_appended_slack_extension()) {
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(rhs, etas_[k]);
      rhs = solve_appended_slack_btran(rhs);
      return;
    }
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      solve_work_.noalias() = rhs;
      hfb_.btran(solve_work_.data(), rhs.data());
      return;
    }
#ifdef MIPSOLVERS_HAVE_KLU
    if (backend_kind_ == FactorBackendKind::KluRescue) {
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(rhs, etas_[static_cast<std::size_t>(k)]);
      if (klu_use_fallback_) {
        rhs = klu_fallback_lu_t_.solve(rhs);
      } else {
        rhs = klu_lu_t_.solve(rhs);
      }
      return;
    }
#endif
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      solve_work_.noalias() = rhs;
      slu_.btran(solve_work_.data(), rhs.data());
    } else {
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
        apply_eta_btran(rhs, etas_[k]);
      if (nlu_.valid && is_sparse_rhs(rhs.data(), m_)) {
        nlu_.btran(rhs.data(), rhs.data());
      } else {
        solve_work_.noalias() = rhs;
        int status = umfpack_dl_wsolve(UMFPACK_At, Ap_.data(), Ai_.data(),
                                       Ax_.data(), rhs.data(),
                                       solve_work_.data(), Numeric_, Control_,
                                       nullptr, wsolve_Wi_.data(),
                                       wsolve_W_.data());
        if (status != UMFPACK_OK) {
          // Solve failed: poison with NaN so downstream isfinite guards abort
          // instead of consuming garbage (same pattern as VendoredHighsBasis).
          rhs.setConstant(std::numeric_limits<double>::quiet_NaN());
          return;
        }
      }
    }
#else
    for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k)
      apply_eta_btran(rhs, etas_[k]);
    rhs = lu_.transpose().solve(rhs);
#endif
  }

  /// In-place BTRAN with nonzero index extraction.
  /// Overwrites rhs with solution and returns nonzero indices.
  void btran_sparse_inplace(Eigen::VectorXd& rhs, std::vector<int>& nz,
                            double tol = 1e-12, int /*initial_nz_row*/ = -1) const {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      solve_work_.noalias() = rhs;
      slu_.btran(solve_work_.data(), rhs.data(), &nz);
      return;
    }
    // Use NativeLU btran which has DFS-based sparse solve.
    if (nlu_.valid) {
      // Apply etas first (in reverse order).
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k) {
        apply_eta_btran(rhs, etas_[k]);
      }
      // NativeLU btran with sparse output.
      nlu_.btran(rhs.data(), solve_work_.data(), &nz);
      rhs.setZero();
      for (int i : nz) { rhs[i] = solve_work_[i]; solve_work_[i] = 0.0; }
      return;
    }
#endif
    btran_inplace(rhs);
    nz.clear();
    for (int i = 0; i < m_; ++i) {
      if (std::abs(rhs[i]) > tol) nz.push_back(i);
    }
  }

  // ── Fully sparse FTRAN from sparse column ──
  // Solves B*x = rhs_col (with etas), where rhs_col is given as sparse
  // indices/values.  Result written to pre-zeroed `result`; nonzero indices
  // returned in out_nz.  Etas applied AFTER the basis solve.
  // Falls back to dense path when conditions not met.
  void ftran_sparse_col(const int* col_idx, const double* col_val, int col_nnz,
                        double* result, std::vector<int>& out_nz) const {
    if (has_appended_slack_extension()) {
      solve_work_.setZero();
      for (int s = 0; s < col_nnz; ++s) {
        const int r = col_idx[s];
        if (r >= 0 && r < m_) solve_work_[r] = col_val[s];
      }
      ftran_inplace(solve_work_);
      out_nz.clear();
      for (int i = 0; i < m_; ++i) {
        result[i] = solve_work_[i];
        if (result[i] != 0.0) out_nz.push_back(i);
      }
      return;
    }
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      // Build dense rhs from sparse input, use slu_ FTRAN.
      solve_work_.setZero();
      for (int s = 0; s < col_nnz; ++s) solve_work_[col_idx[s]] = col_val[s];
      slu_.ftran(solve_work_.data(), result);
      out_nz.clear();
      for (int i = 0; i < m_; ++i) {
        if (result[i] != 0.0) out_nz.push_back(i);
      }
      return;
    }
    if (nlu_.valid && nlu_.l_is_identity) {
      // Fully sparse NativeLU path.
      int raw_nnz = 0;
      nlu_.ftran_sparse(col_idx, col_val, col_nnz,
                        result, nlu_.stk_node.data(), raw_nnz);
      // Apply etas (modifying result in-place, tracking nz).
      out_nz.resize(raw_nnz);
      std::memcpy(out_nz.data(), nlu_.stk_node.data(), raw_nnz * sizeof(int));
      for (const auto& eta : etas_) {
        double dot = 0.0;
        for (size_t k = 0; k < eta.nz_idx.size(); ++k)
          dot += eta.nz_val[k] * result[eta.nz_idx[k]];
        const double old_val = result[eta.pivot_row];
        result[eta.pivot_row] = (old_val - dot) / eta.pivot_value;
        if (old_val == 0.0 && result[eta.pivot_row] != 0.0)
          out_nz.push_back(eta.pivot_row);
      }
      return;
    }
#endif
    // Fallback: dense FTRAN.
    solve_work_.setZero();
    for (int s = 0; s < col_nnz; ++s) solve_work_[col_idx[s]] = col_val[s];
    ftran_inplace(solve_work_);
    out_nz.clear();
    for (int i = 0; i < m_; ++i) {
      result[i] = solve_work_[i];
      if (result[i] != 0.0) out_nz.push_back(i);
    }
  }

  // ── Fully sparse BTRAN for unit vector ──
  // Solves B^T*y = e_{row} (with etas), result to pre-zeroed `result`.
  // Returns nonzero indices in out_nz.
  void btran_sparse_unit(int row, double* result,
                         std::vector<int>& out_nz) const {
    if (has_appended_slack_extension()) {
      solve_work_.setZero();
      if (row >= 0 && row < m_) solve_work_[row] = 1.0;
      btran_inplace(solve_work_);
      out_nz.clear();
      for (int i = 0; i < m_; ++i) {
        result[i] = solve_work_[i];
        if (std::abs(result[i]) > 1e-12) out_nz.push_back(i);
      }
      return;
    }
#ifdef MIPSOLVERS_HAVE_UMFPACK
    if (slu_.valid) {
      // slu_ BTRAN with unit vector e_{row}.
      solve_work_.setZero();
      solve_work_[row] = 1.0;
      slu_.btran(solve_work_.data(), result, &out_nz);
      return;
    }
    if (nlu_.valid && nlu_.l_is_identity) {
      // Apply etas to unit vector — build sparse rhs.
      // Start with sparse {row: 1.0}, apply etas tracking nz set.
      sparse_nz_idx_.clear();
      sparse_nz_val_.clear();
      sparse_nz_idx_.push_back(row);
      sparse_nz_val_.push_back(1.0);
      // Use result buffer as dense workspace for eta application.
      result[row] = 1.0;
      for (int k = static_cast<int>(etas_.size()) - 1; k >= 0; --k) {
        const auto& eta = etas_[k];
        double dot = 0.0;
        for (size_t j = 0; j < eta.nz_idx.size(); ++j)
          dot += eta.nz_val[j] * result[eta.nz_idx[j]];
        const double old_val = result[eta.pivot_row];
        const double new_val = (old_val - dot) / eta.pivot_value;
        if (old_val == 0.0 && new_val != 0.0)
          sparse_nz_idx_.push_back(eta.pivot_row);
        result[eta.pivot_row] = new_val;
      }
      // Collect sparse input from result.
      const int rhs_nnz = static_cast<int>(sparse_nz_idx_.size());
      sparse_nz_val_.resize(rhs_nnz);
      for (int s = 0; s < rhs_nnz; ++s) {
        sparse_nz_val_[s] = result[sparse_nz_idx_[s]];
        result[sparse_nz_idx_[s]] = 0.0;  // reset result for NativeLU output
      }
      // Fully sparse NativeLU BTRAN.
      int raw_nnz = 0;
      nlu_.btran_sparse(sparse_nz_idx_.data(), sparse_nz_val_.data(), rhs_nnz,
                        result, nlu_.stk_node.data(), raw_nnz);
      out_nz.resize(raw_nnz);
      std::memcpy(out_nz.data(), nlu_.stk_node.data(), raw_nnz * sizeof(int));
      return;
    }
#endif
    // Fallback: dense BTRAN.
    solve_work_.setZero();
    solve_work_[row] = 1.0;
    btran_inplace(solve_work_);
    out_nz.clear();
    for (int i = 0; i < m_; ++i) {
      result[i] = solve_work_[i];
      if (std::abs(result[i]) > 1e-12) out_nz.push_back(i);
    }
  }

  void push_eta(int pivot_row, const Eigen::VectorXd& direction) {
    const double piv = std::abs(direction[pivot_row]);
    if (piv < min_pivot_) min_pivot_ = piv;
    const double norm = direction.lpNorm<Eigen::Infinity>();
    if (norm > max_eta_norm_) max_eta_norm_ = norm;
    constexpr double drop_tol = 1e-14;
    EtaVector eta;
    eta.pivot_row = pivot_row;
    eta.pivot_value = direction[pivot_row];
    const double* data = direction.data();
    for (int i = 0; i < m_; ++i) {
      if (i == pivot_row) continue;
      if (std::abs(data[i]) > drop_tol) { eta.nz_idx.push_back(i); eta.nz_val.push_back(data[i]); }
    }
    accumulated_fill_ += static_cast<int>(eta.nz_idx.size());
    etas_.push_back(std::move(eta));
  }

  /// Sparse push_eta: direction has nonzeros only at dir_nz indices.
  void push_eta_sparse(int pivot_row, const double* direction,
                       const std::vector<int>& dir_nz) {
    const double piv = std::abs(direction[pivot_row]);
    if (piv < min_pivot_) min_pivot_ = piv;
    double norm = piv;
    for (int i : dir_nz) {
      const double v = std::abs(direction[i]);
      if (v > norm) norm = v;
    }
    if (norm > max_eta_norm_) max_eta_norm_ = norm;
    constexpr double drop_tol = 1e-14;
    EtaVector eta;
    eta.pivot_row = pivot_row;
    eta.pivot_value = direction[pivot_row];
    for (int i : dir_nz) {
      if (i == pivot_row) continue;
      if (std::abs(direction[i]) > drop_tol) { eta.nz_idx.push_back(i); eta.nz_val.push_back(direction[i]); }
    }
    accumulated_fill_ += static_cast<int>(eta.nz_idx.size());
    etas_.push_back(std::move(eta));
  }

  /// Update basis via Forrest-Tomlin when slu_ is active, else fall back to eta.
  /// Takes the constraint matrix A and entering column index to extract the
  /// original entering column for FT spike computation.
  void update_basis(int pivot_row, const Eigen::VectorXd& direction,
                    const Eigen::SparseMatrix<double>& A, int entering_col) {
    if (pivot_row >= 0 && pivot_row < static_cast<int>(basis_columns_.size())) {
      basis_columns_[static_cast<std::size_t>(pivot_row)] = entering_col;
    }
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      try_hfactor_update(pivot_row, direction.data());
      return;
    }
    if (try_incremental_basis_update(pivot_row, A, entering_col)) return;
    push_eta(pivot_row, direction);
  }

  /// Sparse variant of update_basis for the dual simplex path.
  void update_basis_sparse(int pivot_row, const double* direction,
                           const std::vector<int>& dir_nz,
                           const Eigen::SparseMatrix<double>& A,
                           int entering_col) {
    if (pivot_row >= 0 && pivot_row < static_cast<int>(basis_columns_.size())) {
      basis_columns_[static_cast<std::size_t>(pivot_row)] = entering_col;
    }
    if (backend_kind_ == FactorBackendKind::HFactorPort && hfb_.valid) {
      try_hfactor_update(pivot_row, direction);
      return;
    }
    if (try_incremental_basis_update(pivot_row, A, entering_col)) return;
    push_eta_sparse(pivot_row, direction, dir_nz);
  }

  int eta_count() const override { return static_cast<int>(etas_.size()); }

  /// Refactorization generation counter (incremented each refactorize() call).
  int generation() const override { return gen_; }

  SparseFactorTelemetry factor_telemetry() const override {
    SparseFactorTelemetry t;
    switch (backend_kind_) {
      case FactorBackendKind::HiGHSSafeB: t.backend_id = 1; break;
      case FactorBackendKind::HiGHSForceFTB: t.backend_id = 2; break;
      case FactorBackendKind::HiGHSShortChainB: t.backend_id = 3; break;
      case FactorBackendKind::UmfpackNativeA:
      default: t.backend_id = 0; break;
    }
    t.ft_backend = (backend_kind_ != FactorBackendKind::UmfpackNativeA);
#ifdef MIPSOLVERS_HAVE_UMFPACK
    t.ft_valid = slu_.valid;
    t.ft_updates = slu_.valid ? slu_.n_updates : 0;
    t.eta_updates = static_cast<int>(etas_.size());
    t.min_pivot = slu_.valid ? slu_.min_pivot : ((min_pivot_ < 1e29) ? min_pivot_ : 0.0);
    t.max_growth = slu_.valid ? slu_.max_growth : 1.0;
    int cur_U_nnz = 0;
    if (slu_.valid) {
      for (int j = 0; j < m_; ++j)
        cur_U_nnz += slu_.Uc_start[j + 1] - slu_.Uc_start[j] + 1;
    }
    t.u_fill_ratio = (slu_.valid && slu_.fresh_U_nnz > 0)
        ? static_cast<double>(cur_U_nnz) / static_cast<double>(slu_.fresh_U_nnz)
        : 1.0;
#else
    t.ft_valid    = false;
    t.ft_updates  = 0;
    t.eta_updates = static_cast<int>(etas_.size());
    t.min_pivot   = (min_pivot_ < 1e29) ? min_pivot_ : 0.0;
    t.max_growth  = 1.0;
    t.u_fill_ratio = 1.0;
#endif
    t.refactor_generation = gen_;
    return t;
  }

  /// Reset etas to return the SparseBasis to its last-refactorized state.
  /// Used by root probing to keep the SparseBasis clean between probes.
  void clear_etas() override {
    etas_.clear();
    min_pivot_ = 1e30;
    max_eta_norm_ = 0.0;
    accumulated_fill_ = 0;
    // P10: DSE weights tied to the basis remain valid — clearing etas just
    // collapses them into the current LU factorization, the basis is unchanged.
  }

  /// Truncate etas back to a saved count (for sibling basis reuse).
  void truncate_etas_to(int target_count) override {
    if (target_count < 0) target_count = 0;
    if (target_count >= static_cast<int>(etas_.size())) return;
    etas_.resize(target_count);
    // Conservatively reset stability indicators.
    min_pivot_ = 1e30;
    max_eta_norm_ = 0.0;
    accumulated_fill_ = 0;
    // Recompute fill from remaining etas.
    for (int e = 0; e < static_cast<int>(etas_.size()); ++e) {
      const double piv = std::abs(etas_[e].pivot_value);
      if (piv < min_pivot_) min_pivot_ = piv;
      accumulated_fill_ += static_cast<int>(etas_[e].nz_idx.size());
    }
  }
  
  // Numerical stability indicators
  double min_pivot() const { return min_pivot_; }
  double max_eta_norm() const { return max_eta_norm_; }
  int accumulated_fill() const { return accumulated_fill_; }

  // Check if refactorization is needed based on numerical indicators
  bool needs_refactorize(int m) const {
    // Delegate to the selected factor backend when it has an active
    // incremental update state.
    if (backend_needs_refactorise()) return true;

    // N5: Tighter, size-aware triggers for eta-vector accumulation.
    //
    //   max_etas     : cap on eta stack (scales with m / 30 as before,
    //                  but with a tighter ceiling for very large bases
    //                  where dense eta vectors cost O(m) per replay).
    //   min_piv_tol  : absolute floor on the smallest pivot seen; tight
    //                  for large m to catch drift earlier.
    //   max_norm_tol : ell-infinity norm on eta vectors.  Halved for
    //                  m > 20000 because condition growth compounds
    //                  super-linearly with eta length (see Thm 1 in
    //                  branch_and_cut_framework_complete.tex).
    //
    //   Thresholds are only tightened for m > 20000.  On moderate UC
    //   instances (m ~ 5e3--15e3) the shorter refactor window changes the
    //   pivot sequence just enough to steer the dual simplex away from
    //   the near-degenerate bases that produce the tightest LP bound,
    //   costing 1.3--2x in tree size; at m > 20000 this cost is dwarfed
    //   by the stability benefit on uc_500g-class instances.
    const int max_etas = (m > 20000) ? std::clamp(m / 40, 10, 40)
                                      : std::clamp(m / 30, 10, 60);
    const double min_piv_tol = (m > 20000) ? 1e-7 : 1e-8;
    const double max_norm_tol = (m > 20000) ? 5e5 : 1e6;

    if (eta_count() >= max_etas) return true;
    if (accumulated_fill_ > 3 * m) return true;
    if (min_pivot_ < min_piv_tol) return true;
    if (max_eta_norm_ > max_norm_tol) return true;
    return false;
  }
  
  Eigen::MatrixXd compute_dense_inverse(const std::vector<int>& basis) {
    // Refactorize to produce clean columns (no eta accumulation).
    refactorize(basis);
    Eigen::MatrixXd binv(m_, m_);
    Eigen::VectorXd ei = Eigen::VectorXd::Zero(m_);
    for (int i = 0; i < m_; ++i) {
      ei.setZero();
      ei[i] = 1.0;
      binv.col(i) = ftran(ei);  // etas empty after refactorize
    }
    return binv;
  }

  // ── P10: persisted Dual Steepest Edge weights across reoptimizations ──
  // DSE weights w[i] = ||B^{-T} e_i||^2 are expensive to build from scratch
  // (O(m) BTRAN solves).  In a B&C warm-start context the basis often
  // differs by at most a few pivots from the previous call, so persisting
  // the last-known weights and reusing them when the stored basis matches
  // avoids degenerating to Dantzig pricing (w≡1) on every child node —
  // which is the dominant iteration-count penalty on uc_500g/uc_1000g.
  const std::vector<double>& dse_weights() const { return dse_weights_; }
  bool dse_weights_match(const std::vector<int>& basis) const {
    return !dse_weights_.empty() &&
           static_cast<int>(dse_weights_.size()) == m_ &&
           dse_weights_basis_ == basis;
  }
  void save_dse_weights(const std::vector<double>& w,
                        const std::vector<int>& basis) {
    if (static_cast<int>(w.size()) != m_) return;
    dse_weights_ = w;
    dse_weights_basis_ = basis;
  }
  void invalidate_dse_weights() {
    dse_weights_.clear();
    dse_weights_basis_.clear();
  }
  std::vector<double> compute_exact_dse_weights() const {
    std::vector<double> weights(static_cast<std::size_t>(m_), 1.0);
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(m_);
    for (int i = 0; i < m_; ++i) {
      unit.setZero();
      unit[i] = 1.0;
      const Eigen::VectorXd row = btran(unit);
      const double norm2 = row.squaredNorm();
      if (std::isfinite(norm2)) {
        weights[static_cast<std::size_t>(i)] = std::max(1e-4, norm2);
      }
    }
    return weights;
  }
  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override {
    if (row < 0 || row >= m_) return false;
    Eigen::VectorXd e = Eigen::VectorXd::Zero(m_);
    e[row] = 1.0;
    out = btran(e);
    return out.size() == m_ && out.allFinite();
  }
  bool tableau_row(int row, Eigen::RowVectorXd& out) const override {
    if (A_ == nullptr || row < 0 || row >= m_) return false;
    Eigen::VectorXd e = Eigen::VectorXd::Zero(m_);
    e[row] = 1.0;
    Eigen::VectorXd y = btran(e);
    if (y.size() != m_ || !y.allFinite()) return false;
    out = y.transpose() * (*A_);
    return out.allFinite();
  }
  // Copy persisted DSE weights from a parent SparseBasis (warm-start chain).
  // Only copies if dimensions match; otherwise silently clears.
  void copy_dse_weights_from(const SparseBasis& other) {
    if (other.dse_weights_.size() != static_cast<size_t>(m_) ||
        other.dse_weights_basis_.empty()) {
      invalidate_dse_weights();
      return;
    }
    dse_weights_ = other.dse_weights_;
    dse_weights_basis_ = other.dse_weights_basis_;
  }

 private:
  bool prepare_factor_backend_after_numeric() {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    switch (backend_kind_) {
      case FactorBackendKind::UmfpackNativeA:
        // Backend A (current production baseline): UMFPACK numeric factor
        // plus extracted NativeLU sparse solves. FT update code remains
        // compiled but is kept disabled for MIP path due to measured drift.
        if (m_ >= kNativeLUThreshold) {
          nlu_.extract(Numeric_, m_);
          slu_.valid = false;
          spike_buf_.clear();
          entering_col_buf_.clear();
        } else {
          nlu_.valid = false;
          slu_.valid = false;
        }
        return true;
      case FactorBackendKind::HiGHSSafeB:
      case FactorBackendKind::HiGHSForceFTB:
      case FactorBackendKind::HiGHSShortChainB:
      {
#if MIPSOLVERS_ENABLE_FACTOR_BACKEND_B
        constexpr int kBFTActivationThreshold = 3000;
        // ── P9.1 (2026-04-22, revised) ──
        // FT updates are always activated when m >= kBFTActivationThreshold.
        // The previous catastrophic slowdown on uc_500g_24t traced to a
        // *correctness* bug in the fallback path (slu_.valid=false +
        // nlu_+etas path used a stale pre-update factorisation),
        // not an inherent FT-scaling issue.  With the fallback fixed to
        // force immediate refactorisation, there is no need for a size
        // cap; HiGHS is proof that FT scales to 10^5+ bases.
        if (m_ >= kNativeLUThreshold) {
          nlu_.extract(Numeric_, m_);
          const bool ext_ok = (m_ >= kBFTActivationThreshold) && slu_.extract(Numeric_, m_);
          {
            static const bool ft_diag_b2 = []() {
              const char* e = std::getenv("MIPSOLVERS_P9_FT_DIAG");
              return e && e[0] == '1';
            }();
            static int reported_prep = 0;
            if (ft_diag_b2 && (m_ >= 20000) && reported_prep < 10) {
              std::fprintf(stderr,
                  "[FT-DIAG] prepare: m=%d >=thr?%d extract_ok=%d slu_.valid=%d kind=%d\n",
                  m_, m_ >= kBFTActivationThreshold ? 1 : 0, ext_ok ? 1 : 0,
                  slu_.valid ? 1 : 0, static_cast<int>(backend_kind_));
              ++reported_prep;
            }
          }
          if (ext_ok) {
            slu_.reset_update_tracking();
            spike_buf_.assign(m_, 0.0);
            entering_col_buf_.assign(m_, 0.0);
          } else {
            slu_.valid = false;
            spike_buf_.clear();
            entering_col_buf_.clear();
          }
        } else {
          nlu_.valid = false;
          slu_.valid = false;
          spike_buf_.clear();
          entering_col_buf_.clear();
        }
        return true;
#else
        // Compile-time gated off: fall back to Backend A semantics.
        if (m_ >= kNativeLUThreshold) {
          nlu_.extract(Numeric_, m_);
          slu_.valid = false;
          spike_buf_.clear();
          entering_col_buf_.clear();
        } else {
          nlu_.valid = false;
          slu_.valid = false;
        }
        return true;
#endif
      }
      case FactorBackendKind::HFactorPort:
        return false;
    }
#endif
    return false;
  }

  // U.7.118 Phase 3: HFactorPort FT update.  `direction` is the dual-simplex
  // pivot direction == B^{-1} * a_q (FTRAN of the entering column already
  // performed by the caller).  Compute B^{-T} * e_p via hfb_.btran and forward
  // both to HFactor::update through the wrapper.
  bool try_hfactor_update(int pivot_row, const double* direction) {
    if (!hfb_.valid) return false;
    if (m_ <= 0) return false;
    // BTRAN of unit pivot row.
    if (solve_work_.size() < m_) solve_work_.resize(m_);
    if (static_cast<int>(spike_buf_.size()) < m_) spike_buf_.assign(m_, 0.0);
    solve_work_.setZero();
    solve_work_[pivot_row] = 1.0;
    hfb_.btran(solve_work_.data(), spike_buf_.data());
    return hfb_.update(pivot_row, direction, spike_buf_.data());
  }

  bool try_incremental_basis_update(int pivot_row,
                                    const Eigen::SparseMatrix<double>& A,
                                    int entering_col) {
#ifdef MIPSOLVERS_HAVE_UMFPACK
    switch (backend_kind_) {
      case FactorBackendKind::UmfpackNativeA:
        if (!slu_.valid) return false;
        // Build dense entering column then apply FT spike/update.
        std::fill(entering_col_buf_.begin(), entering_col_buf_.end(), 0.0);
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, entering_col); it; ++it)
          entering_col_buf_[it.row()] = it.value();
        slu_.compute_spike(entering_col_buf_.data(), spike_buf_.data());
        if (slu_.update_ft(pivot_row, spike_buf_.data(), m_, entering_col)) return true;
        // FT update failed (numerical instability) — invalidate and fall back.
        slu_.valid = false;
        return false;
      case FactorBackendKind::HiGHSSafeB:
      case FactorBackendKind::HiGHSForceFTB:
      case FactorBackendKind::HiGHSShortChainB:
#if MIPSOLVERS_ENABLE_FACTOR_BACKEND_B
        if (!slu_.valid) {
          static const bool ft_diag_b = []() {
            const char* e = std::getenv("MIPSOLVERS_P9_FT_DIAG");
            return e && e[0] == '1';
          }();
          static int reported = 0;
          if (ft_diag_b && (m_ >= 20000) && reported < 10) {
            std::fprintf(stderr, "[FT-DIAG] try_incr skipped: slu_.valid=0 m=%d kind=%d\n",
                         m_, static_cast<int>(backend_kind_));
            ++reported;
          }
          return false;
        }
        std::fill(entering_col_buf_.begin(), entering_col_buf_.end(), 0.0);
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, entering_col); it; ++it)
          entering_col_buf_[it.row()] = it.value();
        slu_.compute_spike(entering_col_buf_.data(), spike_buf_.data());
        if (slu_.update_ft(pivot_row, spike_buf_.data(), m_, entering_col)) {
          ++ft_success_count_;
          return true;
        }
        ++ft_fail_count_;
        slu_.valid = false;
        return false;
#else
        return false;
#endif
      case FactorBackendKind::HFactorPort:
        return false;
    }
#endif
    (void)pivot_row;
    (void)A;
    (void)entering_col;
    return false;
  }

  bool backend_needs_refactorise() const {
    if (backend_kind_ == FactorBackendKind::HFactorPort) {
      if (!hfb_.valid) return true;
      return hfb_.needs_refactorise();
    }
#ifdef MIPSOLVERS_HAVE_UMFPACK
    switch (backend_kind_) {
      case FactorBackendKind::UmfpackNativeA:
        return slu_.valid && slu_.needs_refactorise();
      case FactorBackendKind::HiGHSSafeB:
      case FactorBackendKind::HiGHSForceFTB:
      case FactorBackendKind::HiGHSShortChainB:
      {
#if MIPSOLVERS_ENABLE_FACTOR_BACKEND_B
        // ── P9.1b chain-broken detection (2026-04-22) ──
        // If slu_ was activated at the last refactorise (n_updates > 0
        // means at least one FT succeeded) and has since been
        // invalidated by an update_ft failure, the caller pushed an
        // eta for the failing pivot but nlu_ is stale w.r.t. the k
        // prior FT column replacements.  Force immediate refactor to
        // re-sync.  This was the root cause of the uc_500g_24t
        // iteration explosion — see FT fine-tuning notes in the
        // B\&C framework document.
        if (!slu_.valid && slu_.n_updates > 0) {
          ++ft_chain_broken_refactor_count_;
          return true;
        }

        if (!slu_.valid) return false;

        // ── P9.1 tuning (2026-04-22, revised after fixing chain-broken bug) ──
        // The previous P9.1 thresholds (max_updates=12 at m>=20000)
        // were conservative workarounds for the chain-broken bug
        // (invalidated slu_ + stale nlu_+etas path).  With that bug
        // fixed, we raise max_updates toward HiGHS-class values
        // (100-500).  Drift is bounded by min_pivot / max_growth /
        // fill_ratio guards.
        int tuned_max_updates = 60;
        double tuned_min_pivot = 1e-9;
        double tuned_max_growth = 1e5;
        double tuned_fill_ratio = 3.0;

        if (backend_kind_ == FactorBackendKind::HiGHSForceFTB) {
          tuned_max_updates = std::clamp(m_ / 20, 80, 400);
          tuned_min_pivot = 1e-10;
          tuned_max_growth = 1e6;
          tuned_fill_ratio = 5.0;
        } else if (backend_kind_ == FactorBackendKind::HiGHSShortChainB) {
          tuned_max_updates = (m_ >= 20000) ? 12 : 16;
          tuned_min_pivot = 2e-8;
          tuned_max_growth = 5e3;
          tuned_fill_ratio = 2.0;
        } else if (m_ >= 40000) {
          tuned_max_updates = 150;
          tuned_min_pivot = 1e-9;
          tuned_max_growth = 1e5;
          tuned_fill_ratio = 3.0;
        } else if (m_ >= 20000) {
          tuned_max_updates = 100;
          tuned_min_pivot = 1e-9;
          tuned_max_growth = 1e5;
          tuned_fill_ratio = 3.0;
        } else if (m_ >= 10000) {
          tuned_max_updates = 80;
          tuned_min_pivot = 1e-9;
          tuned_max_growth = 1e5;
          tuned_fill_ratio = 3.0;
        }

        if (slu_.n_updates >= tuned_max_updates) return true;
        if (slu_.min_pivot < tuned_min_pivot) return true;
        if (slu_.max_growth > tuned_max_growth) return true;

        int cur_U_nnz = 0;
        for (int j = 0; j < m_; ++j)
          cur_U_nnz += slu_.Uc_start[j + 1] - slu_.Uc_start[j] + 1;
        if (slu_.fresh_U_nnz > 0 &&
            static_cast<double>(cur_U_nnz) >
                tuned_fill_ratio * static_cast<double>(slu_.fresh_U_nnz)) {
          return true;
        }

        return slu_.needs_refactorise();
#else
        return false;
#endif
      }
      case FactorBackendKind::HFactorPort:
        return false;
    }
#endif
    return false;
  }

  bool has_appended_slack_extension() const {
    return has_appended_parent_factor();
  }

  void clear_appended_slack_extension() {
    appended_parent_.reset();
    appended_old_rows_ = 0;
    appended_old_m_ineq_ = 0;
    appended_new_rows_ = 0;
    appended_old_to_new_row_.clear();
    appended_cut_diag_.clear();
    appended_c_by_cut_.clear();
    appended_old_row_factor_.clear();
    appended_old_basis_col_factor_.clear();
  }

  Eigen::VectorXd solve_appended_slack_ftran(
      const Eigen::VectorXd& rhs) const {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!appended_parent_ || rhs.size() != m_) return out;

    Eigen::VectorXd old_rhs =
        Eigen::VectorXd::Zero(appended_old_rows_);
    for (int old_pos = 0; old_pos < appended_old_rows_; ++old_pos) {
      const double rf =
          appended_old_row_factor_[static_cast<std::size_t>(old_pos)];
      old_rhs[old_pos] =
          rhs[appended_old_to_new_row_[static_cast<std::size_t>(old_pos)]] /
          rf;
    }
    Eigen::VectorXd old_sol = appended_parent_->ftran(old_rhs);
    if (old_sol.size() != appended_old_rows_) return out;

    for (int old_pos = 0; old_pos < appended_old_rows_; ++old_pos) {
      out[appended_old_to_new_row_[static_cast<std::size_t>(old_pos)]] =
          old_sol[old_pos] /
          appended_old_basis_col_factor_[static_cast<std::size_t>(old_pos)];
    }
    for (int k = 0; k < appended_new_rows_; ++k) {
      double v = rhs[appended_old_m_ineq_ + k];
      for (const auto& entry :
           appended_c_by_cut_[static_cast<std::size_t>(k)]) {
        const double x_old =
            old_sol[entry.first] /
            appended_old_basis_col_factor_[static_cast<std::size_t>(
                entry.first)];
        v -= entry.second * x_old;
      }
      out[appended_old_m_ineq_ + k] =
          v / appended_cut_diag_[static_cast<std::size_t>(k)];
    }
    return out;
  }

  Eigen::VectorXd solve_appended_slack_btran(
      const Eigen::VectorXd& rhs) const {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!appended_parent_ || rhs.size() != m_) return out;

    Eigen::VectorXd old_rhs =
        Eigen::VectorXd::Zero(appended_old_rows_);
    for (int old_pos = 0; old_pos < appended_old_rows_; ++old_pos) {
      old_rhs[old_pos] =
          rhs[appended_old_to_new_row_[static_cast<std::size_t>(old_pos)]];
    }

    std::vector<double> cut_y(static_cast<std::size_t>(appended_new_rows_),
                              0.0);
    for (int k = 0; k < appended_new_rows_; ++k) {
      cut_y[static_cast<std::size_t>(k)] =
          rhs[appended_old_m_ineq_ + k] /
          appended_cut_diag_[static_cast<std::size_t>(k)];
      for (const auto& entry :
           appended_c_by_cut_[static_cast<std::size_t>(k)]) {
        old_rhs[entry.first] -=
            entry.second * cut_y[static_cast<std::size_t>(k)];
      }
    }

    for (int old_pos = 0; old_pos < appended_old_rows_; ++old_pos) {
      old_rhs[old_pos] /=
          appended_old_basis_col_factor_[static_cast<std::size_t>(old_pos)];
    }
    Eigen::VectorXd old_sol = appended_parent_->btran(old_rhs);
    if (old_sol.size() != appended_old_rows_) return out;
    for (int old_pos = 0; old_pos < appended_old_rows_; ++old_pos) {
      out[appended_old_to_new_row_[static_cast<std::size_t>(old_pos)]] =
          old_sol[old_pos] /
          appended_old_row_factor_[static_cast<std::size_t>(old_pos)];
    }
    for (int k = 0; k < appended_new_rows_; ++k) {
      out[appended_old_m_ineq_ + k] = cut_y[static_cast<std::size_t>(k)];
    }
    return out;
  }

  const Eigen::SparseMatrix<double>* A_;
  int m_;
  Eigen::SparseMatrix<double> B_;
  // Shared solve workspace (used by both the UMFPACK path and HFactor path).
  mutable Eigen::VectorXd solve_work_;

  // U.7.118 Phase 3: vendored HiGHS HFactor backend (FactorBackendKind::HFactorPort).
  // Available whenever MIPSOLVERS_HAVE_HFACTOR is defined, independent of UMFPACK.
#ifdef MIPSOLVERS_HAVE_HFACTOR
  mutable HFactorBackend hfb_;
  mutable std::vector<double> spike_buf_;  // Pre-allocated spike buffer for FT update
#endif

#ifdef MIPSOLVERS_HAVE_UMFPACK
  void* Symbolic_ = nullptr;
  void* Numeric_ = nullptr;
  std::vector<int64_t> Ap_, Ai_;  // int64 CSC for the umfpack_dl_* interface
  std::vector<double> Ax_;
  double Control_[UMFPACK_CONTROL];       // UMFPACK control parameters
  mutable std::vector<double> wsolve_W_;  // Pre-allocated wsolve workspace (5*m doubles)
  mutable std::vector<int64_t> wsolve_Wi_;  // Pre-allocated wsolve workspace (m int64s)
  mutable NativeLU nlu_;                  // Extracted L,U for sparse solves
  mutable SparseLUFactor slu_;             // Sparse LU with Forrest-Tomlin update
  mutable std::vector<double> entering_col_buf_;  // Buffer for FT entering column
  mutable std::vector<int> sparse_nz_idx_;    // Scratch for sparse BTRAN
  mutable std::vector<double> sparse_nz_val_; // Scratch for sparse BTRAN
  mutable std::vector<int> btran_prev_nz_;    // Previous BTRAN NZ for lazy clearing
#else
  mutable Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu_;
#endif
#ifdef MIPSOLVERS_HAVE_KLU
  // KLU rescue backend (FactorBackendKind::KluRescue).  Two factorizations
  // (B and B^T) because this Eigen's SparseSolverBase has no transpose-solve
  // API; falls back to Eigen::SparseLU (pivot threshold 1.0) when KLU
  // reports a numerical/structural failure.
  mutable Eigen::KLU<Eigen::SparseMatrix<double>> klu_lu_, klu_lu_t_;
  mutable Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      klu_fallback_lu_, klu_fallback_lu_t_;
  mutable bool klu_use_fallback_{false};
#endif
  std::vector<EtaVector> etas_;   // Product-form etas for FTRAN/BTRAN
  // Parent-factor extension for scale-compatible appended <= rows.  The
  // current basis has the previous basis columns plus the new row slacks, so
  // after reordering rows as [old rows | new cut rows] its basis matrix is
  // [[B_old, 0], [C, D]].  FTRAN/BTRAN can therefore use the parent factor and
  // the sparse C block until enough pivots force a full refactorization.
  std::shared_ptr<SparseBasis> appended_parent_;
  int appended_old_rows_{0};
  int appended_old_m_ineq_{0};
  int appended_new_rows_{0};
  std::vector<int> appended_old_to_new_row_;
  std::vector<double> appended_cut_diag_;
  std::vector<std::vector<std::pair<int, double>>> appended_c_by_cut_;
  std::vector<double> appended_old_row_factor_;
  std::vector<double> appended_old_basis_col_factor_;
  std::vector<int> basis_columns_;
  double min_pivot_;      // Track smallest pivot for stability
  double max_eta_norm_;   // Track largest eta column norm
  int accumulated_fill_ = 0;  // Track fill-in accumulation
  int gen_;  // Refactorization generation counter
  FactorBackendKind backend_kind_;

  // ── P9.1 FT telemetry (2026-04-22) ──
  // Counters reset at every refactorize().  Used to diagnose why FT
  // underperforms on large UC bases; dumped to stderr at the end of
  // the LP solve when MIPSOLVERS_P9_FT_DIAG=1.
  mutable int ft_success_count_ = 0;
  mutable int ft_fail_count_ = 0;
  mutable int ft_chain_broken_refactor_count_ = 0;
  mutable int refactor_count_ = 0;

  // ── P10 persisted DSE weights (2026-04-22) ──
  // Non-mutable: written on save_dse_weights from solve; read on entry.
  std::vector<double> dse_weights_;
  std::vector<int>    dse_weights_basis_;
 public:
  int ft_success_count() const { return ft_success_count_; }
  int ft_fail_count() const { return ft_fail_count_; }
  int ft_chain_broken_refactor_count() const { return ft_chain_broken_refactor_count_; }
  int refactor_count() const { return refactor_count_; }
 private:

  Eigen::SparseMatrix<double> build_sparse_basis(
      const std::vector<int>& basis) const {
    // Build in compressed-column format directly (avoids triplet sort overhead).
    // Count nonzeros per column first.
    std::vector<int> col_nnz(static_cast<size_t>(m_), 0);
    for (int i = 0; i < m_; ++i) {
      const int col = basis[static_cast<size_t>(i)];
      col_nnz[static_cast<size_t>(i)] =
          A_->outerIndexPtr()[col + 1] - A_->outerIndexPtr()[col];
    }
    Eigen::SparseMatrix<double> B(m_, m_);
    B.reserve(col_nnz);
    for (int i = 0; i < m_; ++i) {
      const int col = basis[static_cast<size_t>(i)];
      for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, col); it; ++it) {
        B.insert(static_cast<int>(it.row()), i) = it.value();
      }
    }
    B.makeCompressed();
    return B;
  }
};

// Full state recompute from SparseBasis (after refactorization).
void sparse_full_recompute(const StandardFormLP& sf,
                           const std::vector<int>& basis,
                           const std::vector<char>& at_upper,
                           const std::vector<char>& is_basic,
                           SparseBasis& sbasis,
                           Eigen::VectorXd& x_b,
                           Eigen::VectorXd& reduced_costs,
                           double& obj) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;

  // Compute adjusted RHS: b - sum_{j at_upper} ub_j * A_j
  x_b = sf.b;
  if (has_ub) {
    for (int j = 0; j < n; ++j) {
      if (!at_upper[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
      const double uj = sf.var_ub[j];
      if (!std::isfinite(uj)) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        x_b[it.row()] -= it.value() * uj;
      }
    }
  }
  sbasis.ftran_inplace(x_b);

  // Dual variables and reduced costs (in-place BTRAN).
  Eigen::VectorXd c_b(m);
  for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
  sbasis.btran_inplace(c_b);  // c_b becomes dual vector y

  // Sparse A^T * y using row-major A when y is sparse (y is now stored in c_b).
  {
    constexpr double y_tol = 1e-12;
    int y_nnz = 0;
    for (int i = 0; i < m; ++i) {
      if (std::abs(c_b[i]) > y_tol) ++y_nnz;
    }
    if (y_nnz * 3 < m) {
      Eigen::VectorXd Aty = Eigen::VectorXd::Zero(n);
      for (int i = 0; i < m; ++i) {
        if (std::abs(c_b[i]) <= y_tol) continue;
        const double yi = c_b[i];
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, i); it; ++it) {
          Aty[it.col()] += it.value() * yi;
        }
      }
      reduced_costs = sf.c_max - Aty;
    } else {
      reduced_costs = sf.c_max - Eigen::VectorXd(sf.A.transpose() * c_b);
    }
  }

  // Objective: c_B^T * x_B + sum_{j at upper} c_max[j] * ub[j].
  // (c_b now holds the dual vector y, so recompute c_B^T * x_B directly.)
  obj = 0.0;
  for (int i = 0; i < m; ++i) obj += sf.c_max[basis[i]] * x_b[i];
  if (has_ub) {
    for (int j = 0; j < n; ++j) {
      if (!at_upper[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
      const double uj = sf.var_ub[j];
      if (std::isfinite(uj)) obj += sf.c_max[j] * uj;  // Use c_max, not reduced_costs!
    }
  }
}

int remap_degenerate_original_frontier(const StandardFormLP& sf,
                                       std::vector<int>& basis,
                                       std::vector<char>& at_upper,
                                       SparseBasis& sbasis,
                                       Eigen::VectorXd& x_b,
                                       Eigen::VectorXd& reduced_costs,
                                       double& obj,
                                       const SimplexOptions& opt) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;
  if (m <= 0 || n <= 0 || !has_ub) return 0;
  const double tol = std::max(1e-8, opt.feasibility_tol);
  for (int i = 0; i < m; ++i) {
    const int art = i < static_cast<int>(sf.row_to_artificial_col.size())
                        ? sf.row_to_artificial_col[static_cast<std::size_t>(i)]
                        : -1;
    if (art >= 0 && basis[static_cast<std::size_t>(i)] == art &&
        x_b[i] > tol) {
      return 0;
    }
  }
  std::vector<char> is_basic = basis_basic_mask(n, basis);
  if (normalize_nonbasic_side_from_reduced_costs(
          sf, basis, reduced_costs, at_upper, opt.optimality_tol) > 0) {
    sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis,
                          x_b, reduced_costs, obj);
  }

  int remaps = 0;
  const double start_obj = obj;
  const LpBasisColumnInfo col_info = lp_basis_build_column_info(sf);
  int entry_basic_orig = 0;
  for (int col : basis) {
    if (col >= 0 && col < n &&
        col_info.kind_for_col[static_cast<std::size_t>(col)] ==
            kLpBasisColOriginal) {
      ++entry_basic_orig;
    }
  }
  // Only the diagnosed original-heavy root frontier should use the broader
  // guarded logical remap.  Large UC roots such as 100g already start with a
  // HiGHS-like original/slack mix; removing many more originals there damaged
  // downstream B&C search despite preserving the LP objective.
  const bool original_heavy_frontier =
      sf.n_original > 0 && entry_basic_orig * 3 > sf.n_original;
  const int max_remaps =
      original_heavy_frontier ? 256 : std::min(64, std::max(8, sf.n_original / 128));
  std::vector<int> logical_candidates;
  logical_candidates.reserve(static_cast<std::size_t>(sf.n_slack + sf.n_surplus));
  for (int row = 0; row < m; ++row) {
    auto push_logical = [&](int col) {
      if (col < 0 || col >= n) return;
      if (is_basic[static_cast<std::size_t>(col)]) return;
      const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
      if (kind != kLpBasisColSlack && kind != kLpBasisColSurplus) return;
      logical_candidates.push_back(col);
    };
    if (row < static_cast<int>(sf.row_to_slack_col.size())) {
      push_logical(sf.row_to_slack_col[static_cast<std::size_t>(row)]);
    }
    if (row < static_cast<int>(sf.row_to_surplus_col.size())) {
      push_logical(sf.row_to_surplus_col[static_cast<std::size_t>(row)]);
    }
  }

  auto rebuild_logical_candidates = [&]() {
    logical_candidates.clear();
    for (int row = 0; row < m; ++row) {
      auto push_logical = [&](int col) {
        if (col < 0 || col >= n) return;
        if (is_basic[static_cast<std::size_t>(col)]) return;
        const unsigned char kind =
            col_info.kind_for_col[static_cast<std::size_t>(col)];
        if (kind != kLpBasisColSlack && kind != kLpBasisColSurplus) return;
        logical_candidates.push_back(col);
      };
      if (row < static_cast<int>(sf.row_to_slack_col.size())) {
        push_logical(sf.row_to_slack_col[static_cast<std::size_t>(row)]);
      }
      if (row < static_cast<int>(sf.row_to_surplus_col.size())) {
        push_logical(sf.row_to_surplus_col[static_cast<std::size_t>(row)]);
      }
    }
  };

  auto validate_remap_state = [&]() {
    bool primal_ok = approx_nonnegative(x_b, opt.feasibility_tol);
    if (primal_ok) {
      for (int i = 0; i < m; ++i) {
        const int col = basis[static_cast<std::size_t>(i)];
        const double col_ub = sf.var_ub[col];
        if (std::isfinite(col_ub) && x_b[i] > col_ub + opt.feasibility_tol) {
          primal_ok = false;
          break;
        }
      }
    }
    std::vector<char> can_enter_all(static_cast<std::size_t>(n), 1);
    for (int i = 0; i < m; ++i) {
      const int a = sf.row_to_artificial_col[static_cast<std::size_t>(i)];
      if (a >= 0 && a < n) can_enter_all[static_cast<std::size_t>(a)] = 0;
    }
    const bool dual_ok = approx_dual_feasible(
        reduced_costs, basis, can_enter_all, at_upper, opt.optimality_tol);
    const bool obj_ok =
        std::abs(obj - start_obj) <=
        std::max(1e-7, 100.0 * opt.optimality_tol *
                           (1.0 + std::abs(start_obj)));
    return primal_ok && dual_ok && obj_ok;
  };

  auto restore_old_state = [&](const std::vector<int>& old_basis,
                               const std::vector<char>& old_at_upper,
                               const Eigen::VectorXd& old_xb,
                               const Eigen::VectorXd& old_rc,
                               double old_obj) {
    basis = old_basis;
    at_upper = old_at_upper;
    is_basic.assign(static_cast<std::size_t>(n), 0);
    for (int col : basis) {
      if (col >= 0 && col < n) is_basic[static_cast<std::size_t>(col)] = 1;
    }
    if (sbasis.refactorize(basis)) {
      x_b = old_xb;
      reduced_costs = old_rc;
      obj = old_obj;
    }
    rebuild_logical_candidates();
  };

  auto try_zero_step_swap = [&](int leaving_row, int entering, bool at_ub,
                                int old_orig_bad_side) {
    const int leaving = basis[static_cast<std::size_t>(leaving_row)];
    if (entering < 0 || entering >= n ||
        is_basic[static_cast<std::size_t>(entering)]) {
      return false;
    }
    const unsigned char entering_kind =
        col_info.kind_for_col[static_cast<std::size_t>(entering)];
    if (entering_kind != kLpBasisColSlack &&
        entering_kind != kLpBasisColSurplus) {
      return false;
    }

    const std::vector<int> old_basis = basis;
    const std::vector<char> old_at_upper = at_upper;
    const Eigen::VectorXd old_xb = x_b;
    const Eigen::VectorXd old_rc = reduced_costs;
    const double old_obj = obj;

    is_basic[static_cast<std::size_t>(leaving)] = 0;
    basis[static_cast<std::size_t>(leaving_row)] = entering;
    is_basic[static_cast<std::size_t>(entering)] = 1;
    at_upper[static_cast<std::size_t>(leaving)] = at_ub ? 1 : 0;
    at_upper[static_cast<std::size_t>(entering)] = 0;

    if (!sbasis.refactorize(basis)) {
      restore_old_state(old_basis, old_at_upper, old_xb, old_rc, old_obj);
      return false;
    }
    sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis,
                          x_b, reduced_costs, obj);
    if (normalize_nonbasic_side_from_reduced_costs(
            sf, basis, reduced_costs, at_upper, opt.optimality_tol) > 0) {
      sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis,
                            x_b, reduced_costs, obj);
    }

    const int new_orig_bad_side =
        count_original_nonbasic_bad_side(sf, basis, at_upper, reduced_costs,
                                         opt.optimality_tol);
    if (!validate_remap_state() || new_orig_bad_side > old_orig_bad_side) {
      restore_old_state(old_basis, old_at_upper, old_xb, old_rc, old_obj);
      return false;
    }
    rebuild_logical_candidates();
    return true;
  };

  for (int scan = 0; scan < m && remaps < max_remaps; ++scan) {
    const int leaving = basis[static_cast<std::size_t>(scan)];
    if (leaving < 0 || leaving >= sf.n_original) continue;
    const double xb = x_b[scan];
    const double ub = sf.var_ub[leaving];
    const bool at_lb = std::abs(xb) <= tol;
    const bool at_ub = std::isfinite(ub) && std::abs(xb - ub) <= tol;
    if (!at_lb && !at_ub) continue;
    if (std::abs(reduced_costs[leaving]) > 10.0 * opt.optimality_tol) {
      continue;
    }

    const int old_orig_bad_side =
        count_original_nonbasic_bad_side(sf, basis, at_upper, reduced_costs,
                                         opt.optimality_tol);

    Eigen::VectorXd pivot_row = Eigen::VectorXd::Zero(m);
    pivot_row[scan] = 1.0;
    sbasis.btran_inplace(pivot_row);
    Eigen::VectorXd tableau_row = sf.A.transpose() * pivot_row;
    auto candidate_pivot_ok = [&](int entering) {
      if (entering < 0 || entering >= n) return false;
      if (!original_heavy_frontier && std::abs(reduced_costs[entering]) >
          10.0 * opt.optimality_tol) {
        return false;
      }
      return std::abs(tableau_row[entering]) > 1e-10;
    };

    bool accepted = false;
    const int slack = scan < static_cast<int>(sf.row_to_slack_col.size())
                          ? sf.row_to_slack_col[static_cast<std::size_t>(scan)]
                          : -1;
    const int surplus = scan < static_cast<int>(sf.row_to_surplus_col.size())
                            ? sf.row_to_surplus_col[static_cast<std::size_t>(scan)]
                            : -1;
    if ((candidate_pivot_ok(slack) &&
         try_zero_step_swap(scan, slack, at_ub, old_orig_bad_side)) ||
        (candidate_pivot_ok(surplus) &&
         try_zero_step_swap(scan, surplus, at_ub, old_orig_bad_side))) {
      accepted = true;
    } else if (original_heavy_frontier) {
      for (int entering : logical_candidates) {
        if (remaps >= max_remaps) break;
        if (entering == slack || entering == surplus) continue;
        if (!candidate_pivot_ok(entering)) continue;
        if (try_zero_step_swap(scan, entering, at_ub, old_orig_bad_side)) {
          accepted = true;
          break;
        }
      }
    }
    if (accepted) ++remaps;
  }
  return remaps;
}

// ════════════════════════════════════════════════════════════════════════════
// INCREMENTAL STATE UPDATE for Diff-based Branch-and-Cut
//
// When switching between B&C nodes, instead of doing a full recompute (which
// causes numerical drift), we incrementally update x_b using the formula:
//   x_b_new = x_b_old - B^{-1} * sum_j (A_j * delta_value_j)
// where delta_value_j is the change in the nonbasic variable's position.
//
// For a nonbasic variable j at upper bound with upper bound change delta_ub:
//   x_b -= B^{-1} * A_j * delta_ub
//
// This avoids O(n) matrix-vector products and maintains numerical stability
// by applying only small incremental changes.
// ════════════════════════════════════════════════════════════════════════════

/// Represents a single bound change for incremental simplex update.
struct SimplexBoundChange {
  int var_idx;       ///< Original variable index (before standard form)
  double delta;      ///< Change in bound value (new - old)
  bool is_lb;        ///< true = lower bound changed, false = upper bound changed
};

/// Apply incremental bound changes to simplex state without full recomputation.
/// Returns true if incremental update succeeded, false if full recompute is needed.
[[maybe_unused]]
bool sparse_incremental_update(
    const StandardFormLP& sf,
    const std::vector<int>& /* basis */,
    const std::vector<char>& at_upper,
    const std::vector<char>& is_basic,
    SparseBasis& sbasis,
    Eigen::VectorXd& x_b,
    double& obj,
    const std::vector<SimplexBoundChange>& changes) {
  
  if (changes.empty()) return true;
  
  const int m = sf.A.rows();
  const int n_orig = sf.n_original;
  
  // Accumulate RHS adjustment: delta_b = sum_j A_j * delta_position_j
  Eigen::VectorXd delta_b = Eigen::VectorXd::Zero(m);
  double delta_obj = 0.0;
  
  for (const auto& bc : changes) {
    const int j = bc.var_idx;
    
    // Skip if out of range (shouldn't happen but defensive).
    if (j < 0 || j >= n_orig) continue;
    
    // Skip if j is basic (bound change affects feasibility, handled by simplex).
    if (is_basic[static_cast<size_t>(j)]) continue;
    
    if (!bc.is_lb) {
      // Upper bound change on nonbasic variable.
      // If variable is at upper bound, its contribution to b changes.
      if (at_upper[static_cast<size_t>(j)]) {
        // Variable is at upper bound: x_j = ub_j
        // b_adj = b - A_j * ub_j, so delta_b_adj = -A_j * delta_ub
        for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
          delta_b[it.row()] -= it.value() * bc.delta;
        }
        // Objective change: c_j * delta_ub (since x_j increases by delta_ub).
        delta_obj += sf.c_max[j] * bc.delta;
      }
      // If at lower bound, upper bound change doesn't affect current state.
      
    } else {
      // Lower bound change: In shifted space, lb is 0, so we adjust b directly.
      // The shifted variable x'_j = x_j - lb_j, and b_shifted = b - A * lb.
      // So delta_b_shifted = -A_j * delta_lb.
      // This affects x_b regardless of whether variable is at upper or lower.
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        delta_b[it.row()] -= it.value() * bc.delta;
      }
      // Objective also shifts: delta_obj = c_j * delta_lb.
      delta_obj += sf.c_max[j] * bc.delta;
    }
  }
  
  // Apply incremental FTRAN: delta_x_b = B^{-1} * delta_b
  if (delta_b.squaredNorm() > 1e-30) {
    Eigen::VectorXd delta_x_b = sbasis.ftran(delta_b);
    x_b += delta_x_b;
  }
  
  obj += delta_obj;
  return true;
}

bool sparse_primal_simplex_optimize(
    const StandardFormLP& sf,
    std::vector<int>& basis,
    const std::vector<char>& can_enter,
    const SimplexOptions& opt,
    SparseBasis& sbasis,
    Eigen::VectorXd& x_b,
    Eigen::VectorXd& reduced_costs,
    double& obj,
    std::vector<char>& at_upper) {
  tl_primal_simplex_iters = 0;
  const bool exact_edge_mode =
      opt.exact_dse_initialization || simplex_exact_edge_env_enabled();
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;

  std::vector<char> is_basic(static_cast<size_t>(n), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<size_t>(idx)] = 1;
  }

  if (!sbasis.refactorize(basis)) return false;
  sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);

  // Legacy primal perturbation is opt-in.  Root LP conformance must preserve
  // the exact degenerate vertex/frontier and use deterministic tie-breaking
  // instead of shifting x_B.
  if (opt.perturb_degenerate_primal) {
    constexpr double pert_base = 1e-6;
    for (int i = 0; i < m; ++i) {
      if (x_b[i] < pert_base) {
        x_b[i] += pert_base * (1.0 + static_cast<double>(i) / m);
      }
    }
  }

  Eigen::VectorXd direction = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd e_leaving = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd alpha(n);
  bool just_refactored = true;

  // ── Pre-allocated workspace to avoid per-iteration heap allocations ──
  std::vector<int> w_nz_vec;
  w_nz_vec.reserve(static_cast<size_t>(m / 4));
  std::vector<int> dir_nz_vec;  // tracks nonzero indices of direction
  dir_nz_vec.reserve(static_cast<size_t>(m / 4));
  // Scratch buffers for sparse column FTRAN.
  std::vector<int> col_idx;
  std::vector<double> col_val;
  col_idx.reserve(64);
  col_val.reserve(64);

  // ── Devex approximate steepest-edge weights for entering variable selection.
  // devex_edge[j] ≈ ||B^{-1} a_j||^2.  Pricing selects the non-basic j that
  // maximises rc_j^2 / devex_edge[j]  (steepest descent per unit simplex edge).
  std::vector<double> devex_edge(static_cast<size_t>(n), 1.0);
  auto rebuild_primal_edge_weights = [&]() {
    // Conformance/root mode: make the primal Phase-II edge state first-class
    // instead of entering from unit Devex weights after Dual Phase I.  HiGHS'
    // simplex path carries edge-weight state through the degenerate frontier;
    // exact initialization gives native the same representation boundary.
    std::fill(devex_edge.begin(), devex_edge.end(), 1.0);
    Eigen::VectorXd edge_col = Eigen::VectorXd::Zero(m);
    std::vector<int> edge_nz;
    edge_nz.reserve(static_cast<size_t>(std::max(1, m / 4)));
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<size_t>(j)]) continue;
      if (!can_enter[static_cast<size_t>(j)]) continue;
      for (int i : edge_nz) edge_col[i] = 0.0;
      edge_nz.clear();
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        edge_col[it.row()] = it.value();
      }
      sbasis.ftran_sparse_inplace(edge_col, edge_nz);
      double norm2 = 0.0;
      for (int i : edge_nz) norm2 += edge_col[i] * edge_col[i];
      if (std::isfinite(norm2)) {
        devex_edge[static_cast<size_t>(j)] = std::max(0.001, norm2);
      }
    }
    for (int i : edge_nz) edge_col[i] = 0.0;
  };
  if (exact_edge_mode) {
    rebuild_primal_edge_weights();
  }
  const auto wall_t0 = std::chrono::steady_clock::now();

  // ── Partial pricing: for large LPs, scan a sector of columns per iteration
  // instead of all n columns. Rotates through sectors, guaranteeing a full
  // scan every ceil(n / sector_size) iterations. Falls back to full pricing
  // when no entering variable is found in the current sector.
  const int sector_size = (opt.use_partial_pricing && n > 2000) ? std::max(500, n / 6) : n;
  int sector_start = 0;

  for (int iter = 0; iter < opt.max_iter; ++iter, ++g_simplex_iter_count) {
    if ((iter & 15) == 0 && simplex_wall_time_limit_hit(opt, wall_t0)) {
      return false;
    }
    // Devex steepest-edge pricing: select entering variable with largest
    // rc^2 / devex_edge score (approximates steepest-edge simplex).
    int entering = -1;
    double best_score = 0.0;
    bool enter_from_upper = false;
    const double rc_tol = opt.optimality_tol;

    // Scan current sector first.
    const int sector_end = std::min(sector_start + sector_size, n);
    for (int j = sector_start; j < sector_end; ++j) {
      if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
      const bool j_up = at_upper[static_cast<size_t>(j)];
      const double rc = j_up ? -reduced_costs[j] : reduced_costs[j];
      if (rc <= rc_tol) continue;
      const double score = (rc * rc) / devex_edge[static_cast<size_t>(j)];
      if (score > best_score + 1e-14 ||
          (std::abs(score - best_score) <= 1e-14 &&
           (entering < 0 || j < entering))) {
        best_score = score;
        entering = j;
        enter_from_upper = j_up;
      }
    }
    // Advance sector for next iteration.
    sector_start = (sector_end >= n) ? 0 : sector_end;

    // If sector found nothing, do a full scan before declaring optimal.
    if (entering < 0 && sector_size < n) {
      for (int j = 0; j < n; ++j) {
        if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) continue;
        const bool j_up = at_upper[static_cast<size_t>(j)];
        const double rc = j_up ? -reduced_costs[j] : reduced_costs[j];
        if (rc <= rc_tol) continue;
        const double score = (rc * rc) / devex_edge[static_cast<size_t>(j)];
        if (score > best_score + 1e-14 ||
            (std::abs(score - best_score) <= 1e-14 &&
             (entering < 0 || j < entering))) {
          best_score = score;
          entering = j;
          enter_from_upper = j_up;
        }
      }
    }

    if (entering < 0) {
      if (just_refactored) {
        // Final recompute to remove perturbation residuals from x_b and obj.
        sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
        
        // Bug fix: verify primal feasibility before declaring optimal.
        // If x_b has significant negative values, the solution is not valid.
        double min_xb = 0.0;
        for (int i = 0; i < m; ++i) {
          if (x_b[i] < min_xb) min_xb = x_b[i];
        }
        const double feas_tol = opt.feasibility_tol;
        if (min_xb < -feas_tol) {
          // Primal infeasible - likely at_upper corruption or numerical issues.
          // Return false to trigger fallback.
          MIPSOLVERS_LOG_DEBUG("[sparse_primal_simplex] PRIMAL INFEASIBLE at termination: min_xb={:.2e}", min_xb);
          return false;
        }
        
        return true;
      }
      if (!sbasis.refactorize(basis)) return false;
      sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
      just_refactored = true;
      continue;
    }

    // FTRAN: direction = B^{-1} * a_entering (in-place).
    direction.setZero();
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, entering); it; ++it) {
      direction[it.row()] = it.value();
    }
    sbasis.ftran_inplace(direction);

    // Ratio test (Harris / expand ratio test to break degeneracy).
    const double sign = enter_from_upper ? -1.0 : 1.0;
    const double harris_tol = opt.feasibility_tol * 1e2;  // ~1e-4
    double min_ratio = kInf;
    int leaving_row = -1;
    bool leaving_to_upper = false;

    // Pass 1: compute min_ratio (dense for correctness).
    for (int i = 0; i < m; ++i) {
      const double d = sign * direction[i];
      if (d > opt.feasibility_tol) {
        const double ratio = x_b[i] / d;
        if (ratio < min_ratio) min_ratio = ratio;
      } else if (has_ub && d < -opt.feasibility_tol) {
        const int bvar = basis[i];
        const double ub_i = sf.var_ub[bvar];
        if (std::isfinite(ub_i)) {
          const double slack = ub_i - x_b[i];
          if (slack >= -opt.feasibility_tol) {
            const double ratio = slack / (-d);
            if (ratio < min_ratio) min_ratio = ratio;
          }
        }
      }
    }

    // Pass 2: among rows within harris_tol of min_ratio, choose the
    // first-class degenerate frontier deterministically.  Exact/near-exact
    // ratio ties are resolved by row id; otherwise keep Harris' larger-pivot
    // preference for numerical safety.
    double best_pivot = 0.0;
    double selected_ratio = kInf;
    const double ratio_cutoff = min_ratio + harris_tol;
    auto consider_leaving = [&](int i, double ratio, double d,
                                bool to_upper) {
      const double abs_pivot = std::abs(d);
      const bool near_ratio =
          std::abs(ratio - selected_ratio) <= opt.feasibility_tol;
      if (leaving_row < 0 || ratio < selected_ratio - opt.feasibility_tol ||
          (near_ratio && i < leaving_row) ||
          (!near_ratio && abs_pivot > best_pivot)) {
        selected_ratio = ratio;
        best_pivot = abs_pivot;
        leaving_row = i;
        leaving_to_upper = to_upper;
      }
    };
    for (int i = 0; i < m; ++i) {
      const double d = sign * direction[i];
      if (d > opt.feasibility_tol) {
        const double ratio = x_b[i] / d;
        if (ratio <= ratio_cutoff) {
          consider_leaving(i, ratio, d, false);
        }
      } else if (has_ub && d < -opt.feasibility_tol) {
        const int bvar = basis[i];
        const double ub_i = sf.var_ub[bvar];
        if (std::isfinite(ub_i)) {
          const double slack = ub_i - x_b[i];
          if (slack >= -opt.feasibility_tol) {
            const double ratio = slack / (-d);
            if (ratio <= ratio_cutoff) {
              consider_leaving(i, ratio, d, true);
            }
          }
        }
      }
    }
    // Use the SELECTED row's ratio as step size (may be slightly > min_ratio,
    // which is the essence of the Harris test — accepts small infeasibility).
    double max_step;
    if (leaving_row >= 0) {
      const double d = sign * direction[leaving_row];
      if (!leaving_to_upper) {
        max_step = x_b[leaving_row] / d;
      } else {
        const double ub_i = sf.var_ub[basis[leaving_row]];
        max_step = (ub_i - x_b[leaving_row]) / (-d);
      }
    } else {
      max_step = min_ratio;
    }

    bool bound_flip = false;
    if (has_ub) {
      const double uj = sf.var_ub[entering];
      if (std::isfinite(uj) && uj < max_step - 1e-12) {
        max_step = uj;
        bound_flip = true;
      }
    }

    if (!bound_flip && leaving_row < 0) continue;
    ++tl_primal_simplex_iters;

    const double old_rc = reduced_costs[entering];

    if (bound_flip) {
      x_b.noalias() -= direction * (sign * max_step);
      obj += old_rc * (sign * max_step);
      at_upper[static_cast<size_t>(entering)] = enter_from_upper ? 0 : 1;
      devex_edge[static_cast<size_t>(entering)] = std::max(0.001, direction.squaredNorm());
    } else {
      const double pivot = direction[leaving_row];
      if (std::abs(pivot) < 1e-12) return false;

      const int old_leaving = basis[leaving_row];
      is_basic[static_cast<size_t>(old_leaving)] = 0;
      at_upper[static_cast<size_t>(old_leaving)] = leaving_to_upper ? 1 : 0;
      basis[leaving_row] = entering;
      is_basic[static_cast<size_t>(entering)] = 1;
      at_upper[static_cast<size_t>(entering)] = 0;

      const double raw_step = sign * max_step;
      x_b.noalias() -= direction * raw_step;
      x_b[leaving_row] = enter_from_upper ? (sf.var_ub[entering] - max_step) : max_step;

      // Update reduced costs via in-place BTRAN(e_{leaving_row}).
      e_leaving.setZero();
      e_leaving[leaving_row] = 1.0;
      w_nz_vec.clear();
      sbasis.btran_sparse_inplace(e_leaving, w_nz_vec, 1e-12, leaving_row);

      // Sparse A^T * e_leaving using row-major A when e_leaving is sparse.
      {
        // P11: sparse pricing path wins whenever BTRAN output is not nearly
        // dense.
        if (static_cast<int>(w_nz_vec.size()) * 2 < m) {
          alpha.setZero();
          for (int i : w_nz_vec) {
            const double wi = e_leaving[i];
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, i); it; ++it) {
              alpha[it.col()] += it.value() * wi;
            }
          }
        } else {
          alpha.noalias() = sf.A.transpose() * e_leaving;
        }
      }

      const double rc_scale = old_rc / pivot;
      reduced_costs -= rc_scale * alpha;
      reduced_costs[entering] = 0.0;

      obj += old_rc * raw_step;

      // NaN/Inf guard: numerical breakdown in pivot updates.
      if (!std::isfinite(obj)) return false;

      sbasis.update_basis(leaving_row, direction, sf.A, entering);

      // ── Devex weight update after basis pivot ──
      // The entering column's exact weight is ||direction||² = ||B⁻¹ a_entering||².
      // For other non-basics, update via: w[j] = max(w[j], (alpha[j]/pivot)² * w_entering).
      // The leaving variable becomes non-basic: initialize its weight.
      {
        const double w_entering = direction.squaredNorm();
        const double piv2 = pivot * pivot;
        devex_edge[static_cast<size_t>(old_leaving)] = std::max(0.001, w_entering / piv2);
        for (int j = 0; j < n; ++j) {
          if (is_basic[static_cast<size_t>(j)]) continue;
          const double aj = alpha[j];
          if (std::abs(aj) < 1e-12) continue;
          const double est = (aj * aj / piv2) * w_entering;
          devex_edge[static_cast<size_t>(j)] = std::max(devex_edge[static_cast<size_t>(j)], est);
        }
      }
    }
    if (!bound_flip && sbasis.needs_refactorize(m)) {
      if (!sbasis.refactorize(basis)) return false;
      sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
      // Re-apply legacy perturbation only when explicitly requested.
      if (opt.perturb_degenerate_primal) {
        constexpr double pert_base = 1e-6;
        for (int i = 0; i < m; ++i) {
          if (x_b[i] < pert_base) {
            x_b[i] += pert_base * (1.0 + static_cast<double>(i) / m);
          }
        }
      }
      just_refactored = true;
      // Refactorization refreshes the basis representation.  In conformance
      // mode the current basis must keep first-class edge weights; otherwise
      // later degenerate pivots silently fall back to unit Devex pricing.
      if (exact_edge_mode) {
        rebuild_primal_edge_weights();
      } else {
        std::fill(devex_edge.begin(), devex_edge.end(), 1.0);
      }
    } else {
      just_refactored = false;
    }
  }

  return false;
}

// Advanced sparse dual simplex with Harris ratio test.
// This is the primary warm-start method for B&C: when branching changes bounds,
// dual feasibility is preserved and only primal feasibility needs restoration.
bool sparse_dual_simplex_reoptimize(
    const StandardFormLP& sf,
    std::vector<int>& basis,
    const std::vector<char>& can_enter,
    const SimplexOptions& opt,
    SparseBasis& sbasis,
    Eigen::VectorXd& x_b,
    Eigen::VectorXd& reduced_costs,
    double& obj,
    std::vector<char>& at_upper,
    bool& proved_infeasible,
    bool& cutoff_reached) {
  proved_infeasible = false;
  cutoff_reached = false;
  // Reset file-scope thread_local counter for iteration profiling.
  tl_dual_reopt_iters = 0;
  const bool exact_edge_mode =
      opt.exact_dse_initialization || simplex_exact_edge_env_enabled();
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const bool has_ub = sf.var_ub.size() == n;
  // Base pivot tolerance — used in dual ratio test. Adaptive scaling is
  // applied per-iteration based on the BTRAN vector magnitude.
  const double alpha_tol_base = 1e-9;

  std::vector<char> is_basic(static_cast<size_t>(n), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<size_t>(idx)] = 1;
  }

  // NOTE: We deliberately do NOT clamp reduced costs before the first
  // iteration. Clamping hides valid entering candidates from the dual
  // ratio test, leading to false Farkas certificates. RC cleanup is
  // only applied after refactorization (see cleanup_reduced_costs below).
  // Zero out basic variable RCs (they should be zero by definition).
  for (int j = 0; j < n; ++j) {
    if (is_basic[static_cast<size_t>(j)]) reduced_costs[j] = 0.0;
  }

  // Degeneracy tracking.
  int stall_count = 0;
  double last_obj = obj;

  // ── EXPAND framework (Gill et al.) for degeneracy resolution ──
  // Instead of fixed feasibility tolerance, allow variable bounds to be
  // "legally" violated by a small, gradually growing tolerance.  This lets
  // the simplex method glide across degenerate faces without cycling.
  // After apparent optimality, a cleanup phase with the true tolerance
  // ensures all variables are strictly within their real bounds.
  const double expand_delta = std::min(1e-9, opt.feasibility_tol * 0.01);
  double expand_tol = expand_delta;  // starts tiny, grows each iteration
  bool in_cleanup = false;           // true when running the post-optimal cleanup

  Eigen::VectorXd row_vec = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd direction(m);
  direction.setZero();
  bool just_refactored = true;

  // ── Exact Dual Steepest Edge (DSE) weights for leaving variable selection.
  // dse_w[i] = ||B^{-T} e_i||^2 = ||row i of B^{-1}||^2.
  // Pricing selects the row i that maximises |violation_i|^2 / dse_w[i].
  // Updated exactly each iteration via an additional FTRAN of the BTRAN result.
  //
  // ── P10 (2026-04-22): persist weights across B&C reoptimizations ──
  // When the caller reuses the same SparseBasis with an unchanged basis
  // vector (common after branching without basis change), reuse the
  // previously-converged weights instead of restarting from 1.0 (≡ Dantzig
  // pricing).  On uc_500g_24t / uc_1000g_24t this typically cuts iteration
  // count by 3-5x on warm-started nodes.  Opt-out via MIPSOLVERS_P10_NO_DSE_PERSIST=1.
  static const bool p10_no_persist = []() {
    const char* e = std::getenv("MIPSOLVERS_P10_NO_DSE_PERSIST");
    return e && e[0] == '1';
  }();
  std::vector<double> dse_w;
  bool dse_warm_started = false;
  bool dse_exact_initialized = false;
  if (!p10_no_persist && sbasis.dse_weights_match(basis)) {
    dse_w = sbasis.dse_weights();  // copy
    dse_warm_started = true;
  } else if (!p10_no_persist && exact_edge_mode) {
    // Conformance mode still owns first-class edge weights, but ownership
    // means "inherit the matching basis state or build it exactly"; repeatedly
    // recomputing exact weights on every same-basis node loses the HiGHS-like
    // warm edge state and slows the post-root tree without changing the LP.
    dse_w = sbasis.compute_exact_dse_weights();
    dse_exact_initialized = true;
  } else {
    dse_w.assign(static_cast<size_t>(m), 1.0);
  }
  static const bool p10_diag = []() {
    const char* e = std::getenv("MIPSOLVERS_P10_DIAG");
    return e && e[0] == '1';
  }();
  if (p10_diag) {
    std::fprintf(stderr, "[P10-DSE] reopt start m=%d warm=%d exact=%d\n",
                 m, dse_warm_started ? 1 : 0,
                 dse_exact_initialized ? 1 : 0);
  }
  static const bool pivot_trace = []() {
    const char* e = std::getenv("MIPSOLVERS_XPOOL_PIVOT_TRACE");
    return e && e[0] == '1';
  }();
  static const int pivot_trace_limit = []() {
    const char* e = std::getenv("MIPSOLVERS_XPOOL_PIVOT_TRACE_LIMIT");
    if (!e || e[0] == '\0') return 80;
    return std::max(0, std::atoi(e));
  }();
  const char* pivot_trace_kind =
      sbasis.has_appended_parent_factor() ? "append_parent" : "refactor";

  // ── Pre-allocated workspace to avoid per-iteration heap allocations ──
  Eigen::VectorXd e_leaving = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd dse_tau(m);  // τ = B^{-1} ρ for DSE weight update
  dse_tau.setZero();
  std::vector<int> w_nz;
  w_nz.reserve(static_cast<size_t>(m / 4));
  std::vector<int> dir_nz;    // nonzero indices of direction (FTRAN result)
  dir_nz.reserve(static_cast<size_t>(m / 4));
  std::vector<int> dse_tau_nz;  // nonzero indices of tau = B^{-1} * rho
  dse_tau_nz.reserve(static_cast<size_t>(m / 4));
  // Sparse pricing workspace: track which columns received nonzero contributions.
  std::vector<int> touched_cols;
  touched_cols.reserve(static_cast<size_t>(n / 2));
  std::vector<char> col_touched(static_cast<size_t>(n), 0);
  bool sparse_pricing_taken = false;

  auto cleanup_reduced_costs = [&]() {
    const double dtol = opt.optimality_tol;
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<size_t>(j)]) {
        reduced_costs[j] = 0.0;
        continue;
      }
      if (!at_upper[static_cast<size_t>(j)]) {
        if (reduced_costs[j] > 0.0 && reduced_costs[j] <= dtol)
          reduced_costs[j] = 0.0;
      } else {
        if (reduced_costs[j] < 0.0 && reduced_costs[j] >= -dtol)
          reduced_costs[j] = 0.0;
      }
    }
  };

  // ── Per-phase timing for dual simplex inner loop ──
  thread_local struct {
    int64_t pricing_ns{0}, btran_ns{0}, sprice_ns{0}, ratio_ns{0};
    int64_t ftran_ns{0}, update_ns{0}, dse_ns{0}, refact_ns{0};
    int64_t eta_ns{0};
    int count{0}, refact_count{0};
    void dump(int m_val, bool verbose = false) {
      if (verbose && count > 0 && count % 2000 == 0) {
        auto us = [](int64_t ns, int n) -> double { return ns * 0.001 / n; };
        fprintf(stderr, "[ITER-TIMING] m=%d iters=%d refacts=%d: "
                "pricing=%.1f btran=%.1f sprice=%.1f ratio=%.1f "
                "ftran=%.1f update=%.1f eta=%.1f dse=%.1f us/iter\n",
                m_val, count, refact_count,
                us(pricing_ns, count), us(btran_ns, count),
                us(sprice_ns, count), us(ratio_ns, count),
                us(ftran_ns, count), us(update_ns, count),
                us(eta_ns, count), us(dse_ns, count));
      }
    }
  } iter_timing;
  auto it_tp = [](){ return std::chrono::high_resolution_clock::now(); };
  auto it_ns = [](auto a, auto b){ return std::chrono::duration_cast<std::chrono::nanoseconds>(b-a).count(); };
  const auto wall_t0 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < opt.max_iter; ++iter, ++g_simplex_iter_count) {
    if ((iter & 15) == 0 && simplex_wall_time_limit_hit(opt, wall_t0)) {
      return false;
    }
    ++tl_dual_reopt_iters;
    auto t0 = it_tp();
    // Early termination: if an incumbent bound is provided (B&C solver),
    // periodically check if the LP relaxation bound already exceeds it.
    // The original-form LP bound = sf.objective_const - obj.
    // We prune when: objective_const - obj >= incumbent, i.e., obj <= cutoff.
    if (opt.incumbent_bound != nullptr &&
        (iter % opt.incumbent_check_interval == 0)) {
      const double inc = opt.incumbent_bound->load(std::memory_order_relaxed);
      if (std::isfinite(inc)) {
        const double cutoff = sf.objective_const - inc;
        if (obj <= cutoff + 1e-9) {
          // Node bound exceeds incumbent.
          // This is a safe bound prune, not primal infeasibility.
          cutoff_reached = true;
          return false;
        }
      }
    }

    // Degeneracy stall detection — triggers refactorization only.
    // The EXPAND tolerance growth handles anti-cycling; we no longer perturb.
    if (std::abs(obj - last_obj) < 1e-12) {
      ++stall_count;
    } else {
      stall_count = 0;
      last_obj = obj;
    }
    if (stall_count > 50) {
      if (!just_refactored) {
        if (!sbasis.refactorize(basis)) return false;
        sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
        cleanup_reduced_costs();
        just_refactored = true;
        sparse_pricing_taken = false;
        touched_cols.clear();
        stall_count = 0;
        last_obj = obj;
        continue;
      }
      stall_count = 0;
    }

    // EXPAND: grow the dynamic tolerance each iteration (capped at opt.feasibility_tol).
    if (!in_cleanup) {
      expand_tol = std::min(expand_tol + expand_delta, opt.feasibility_tol);
    }
    const double cur_feas_tol = in_cleanup ? opt.feasibility_tol : expand_tol;

    // Find leaving variable using Exact Dual Steepest Edge (DSE) pricing.
    // Score = violation^2 / dse_w[i]; select the row with largest score.
    // Uses the current EXPAND tolerance (small and growing, or true tol in cleanup).
    int leaving_row = -1;
    double best_score = 0.0;
    bool leaving_below = true;

    for (int i = 0; i < m; ++i) {
      double viol = 0.0;
      bool below = true;
      if (x_b[i] < -cur_feas_tol) {
        viol = -x_b[i];
        below = true;
      } else if (has_ub) {
        const int bvar = basis[i];
        const double ub_i = sf.var_ub[bvar];
        if (std::isfinite(ub_i) && x_b[i] > ub_i + cur_feas_tol) {
          viol = x_b[i] - ub_i;
          below = false;
        }
      }
      if (viol <= cur_feas_tol) continue;
      const double score = (viol * viol) / dse_w[static_cast<size_t>(i)];
      if (score > best_score) {
        best_score = score;
        leaving_row = i;
        leaving_below = below;
      }
    }

    if (leaving_row < 0) {
      if (!in_cleanup) {
        // Apparently optimal under expanded tolerance — enter cleanup phase.
        // First, refactorize to remove numerical drift from incremental updates.
        if (!just_refactored) {
          if (!sbasis.refactorize(basis)) return false;
          sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
          cleanup_reduced_costs();
          just_refactored = true;
          sparse_pricing_taken = false;
          touched_cols.clear();
          stall_count = 0;
          last_obj = obj;
          // Re-check with fresh x_b
          continue;
        }
        // Check whether any variable violates the true feasibility tolerance.
        bool need_cleanup = false;
        for (int i = 0; i < m; ++i) {
          if (x_b[i] < -opt.feasibility_tol) { need_cleanup = true; break; }
          if (has_ub) {
            const double ub_i = sf.var_ub[basis[i]];
            if (std::isfinite(ub_i) && x_b[i] > ub_i + opt.feasibility_tol) {
              need_cleanup = true; break;
            }
          }
        }
        if (need_cleanup) {
          in_cleanup = true;
          continue;  // re-enter loop with true tolerance
        }
      }
      // ── P10: save DSE weights for next reoptimization ──
      if (!p10_no_persist) {
        sbasis.save_dse_weights(dse_w, basis);
        if (p10_diag) {
          std::fprintf(stderr, "[P10-DSE] reopt done m=%d iters=%d saved\n",
                       m, tl_dual_reopt_iters);
        }
      }
      return true;  // All feasible under true tolerance → optimal.
    }
    auto t1 = it_tp();
    iter_timing.pricing_ns += it_ns(t0, t1);

    // Pivot row via in-place sparse BTRAN (reuse pre-allocated e_leaving).
    e_leaving.setZero();
    e_leaving[leaving_row] = 1.0;
    sbasis.btran_sparse_inplace(e_leaving, w_nz, 1e-12, leaving_row);
    auto t2 = it_tp();
    iter_timing.btran_ns += it_ns(t1, t2);

    // Sparse row_vec computation: only accumulate rows where e_leaving (BTRAN result) is nonzero.
    // Also track touched columns for sparse ratio test and RC update.
    {
      // P11: sparse pricing path wins whenever BTRAN output is not nearly
      // dense.  Cost is O(sum_{i in w_nz} nnz(A_row[i])) vs dense O(nnz(A)).
      if (static_cast<int>(w_nz.size()) * 2 < m) {
        // Sparse path: zero only previously touched entries, then accumulate.
        // If previous iteration used the dense path, touched_cols is empty but
        // row_vec has stale values — must do full zeroing in that case.
        if (sparse_pricing_taken && !touched_cols.empty()) {
          for (int c : touched_cols) row_vec[c] = 0.0;
        } else {
          row_vec.setZero();
        }
        touched_cols.clear();
        for (int i : w_nz) {
          const double wi = e_leaving[i];
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, i); it; ++it) {
            const int col = static_cast<int>(it.col());
            row_vec[col] += it.value() * wi;
            if (!col_touched[static_cast<size_t>(col)]) {
              col_touched[static_cast<size_t>(col)] = 1;
              touched_cols.push_back(col);
            }
          }
        }
        // Reset col_touched flags (O(|touched_cols|), not O(n)).
        for (int c : touched_cols) col_touched[static_cast<size_t>(c)] = 0;
        sparse_pricing_taken = true;
      } else {
        row_vec.noalias() = sf.A.transpose() * e_leaving;
        touched_cols.clear();
        sparse_pricing_taken = false;
      }
    }

    // ══════════════════════════════════════════════════════════════════════
    // Bound-Flipping Ratio Test (BFRT / Long-Step Dual Simplex)
    // Instead of stopping at the first blocking non-basic variable, we flip
    // bounded variables past their opposite bound and continue, absorbing
    // their contribution to the primal deficit.  Only the final entering
    // variable triggers an actual basis change (FTRAN + FT update).
    // ══════════════════════════════════════════════════════════════════════
    const double w_inf = e_leaving.lpNorm<Eigen::Infinity>();
    auto t3 = it_tp();
    iter_timing.sprice_ns += it_ns(t2, t3);
    const double alpha_tol = std::max(alpha_tol_base, 1e-7 * w_inf);

    // Compute primal deficit D = how much the leaving variable must move.
    double primal_deficit;
    if (leaving_below) {
      primal_deficit = -x_b[leaving_row];  // x_b[p] < 0, need to increase by |x_b[p]|
    } else {
      primal_deficit = x_b[leaving_row] - sf.var_ub[basis[leaving_row]];  // above ub
    }

    // Build sorted candidate list for BFRT.
    struct BFRTCandidate {
      int index;
      double alpha_j;    // effective pivot row entry (positive = eligible)
      double theta;      // ratio = |rc_j| / alpha_j
      double delta;      // absorption = alpha_j * (ub_j - lb_j)
      bool flippable;    // has finite bound range
    };
    // Use thread-local to avoid repeated heap allocations.
    thread_local std::vector<BFRTCandidate> bfrt_candidates;
    bfrt_candidates.clear();

    auto collect_bfrt_candidate = [&](int j, double cur_alpha_tol) {
      if (!can_enter[static_cast<size_t>(j)] || is_basic[static_cast<size_t>(j)]) return;
      const bool j_up = at_upper[static_cast<size_t>(j)];
      double alpha_j;
      if (leaving_below) {
        alpha_j = j_up ? row_vec[j] : -row_vec[j];
      } else {
        alpha_j = j_up ? -row_vec[j] : row_vec[j];
      }
      if (alpha_j < cur_alpha_tol) return;
      const double ratio = j_up ? (reduced_costs[j] / alpha_j) : (-reduced_costs[j] / alpha_j);
      // Compute bound range for flipping.
      const double ub_j = has_ub ? sf.var_ub[j] : kInf;
      const bool can_flip = std::isfinite(ub_j) && ub_j > 1e-12;
      const double delta_j = can_flip ? (alpha_j * ub_j) : kInf;
      bfrt_candidates.push_back({j, alpha_j, ratio, delta_j, can_flip});
    };

    if (sparse_pricing_taken) {
      for (int j : touched_cols) collect_bfrt_candidate(j, alpha_tol);
    } else {
      for (int j = 0; j < n; ++j) collect_bfrt_candidate(j, alpha_tol);
    }

    // Sort candidates by theta (ascending) for the flipping loop.
    const double theta_zero_tol = opt.optimality_tol;
    auto bfrt_less = [theta_zero_tol](const BFRTCandidate& a,
                                     const BFRTCandidate& b) {
      const bool a_zero = std::abs(a.theta) <= theta_zero_tol;
      const bool b_zero = std::abs(b.theta) <= theta_zero_tol;
      if (a_zero && b_zero) return a.index < b.index;
      if (a.theta < b.theta) return true;
      if (b.theta < a.theta) return false;
      return a.index < b.index;
    };
    std::sort(bfrt_candidates.begin(), bfrt_candidates.end(), bfrt_less);

    // Flipping loop: absorb deficit by flipping bounded variables.
    int entering = -1;
    double D = primal_deficit;
    thread_local std::vector<int> flipped_vars;  // vars that were bound-flipped
    flipped_vars.clear();

    for (const auto& cand : bfrt_candidates) {
      if (cand.flippable && D > cand.delta + 1e-12) {
        // Flip this variable: absorb its contribution.
        D -= cand.delta;
        flipped_vars.push_back(cand.index);
      } else {
        // This variable becomes the entering variable (basis change).
        entering = cand.index;
        break;
      }
    }

    if (entering < 0 && !bfrt_candidates.empty()) {
      // All candidates were flipped — last one becomes entering.
      // This shouldn't normally happen; fall back to last candidate.
      if (!flipped_vars.empty()) {
        entering = flipped_vars.back();
        flipped_vars.pop_back();
      }
    }

    if (entering < 0) {
      // No entering variable found — could be numerical drift or true infeasibility.
      if (!just_refactored) {
        if (!sbasis.refactorize(basis)) return false;
        sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
        cleanup_reduced_costs();
        just_refactored = true;
        sparse_pricing_taken = false;
        touched_cols.clear();
        continue;
      }
      // Retry with expanded tolerance before declaring infeasibility.
      const double expanded_tol = 1e-12;
      if (expanded_tol < alpha_tol) {
        bfrt_candidates.clear();
        flipped_vars.clear();
        if (sparse_pricing_taken) {
          for (int j : touched_cols) collect_bfrt_candidate(j, expanded_tol);
        } else {
          for (int j = 0; j < n; ++j) collect_bfrt_candidate(j, expanded_tol);
        }
        if (!bfrt_candidates.empty()) {
          std::sort(bfrt_candidates.begin(), bfrt_candidates.end(), bfrt_less);
          entering = bfrt_candidates.front().index;
        }
      }
      if (entering < 0) {
        proved_infeasible = true;
        return false;
      }
    }
    if (pivot_trace && iter < pivot_trace_limit) {
      const BFRTCandidate* first_cand =
          bfrt_candidates.empty() ? nullptr : &bfrt_candidates.front();
      const double first_theta = first_cand ? first_cand->theta : 0.0;
      const double first_alpha = first_cand ? first_cand->alpha_j : 0.0;
      const int first_col = first_cand ? first_cand->index : -1;
      const int bvar = (leaving_row >= 0 && leaving_row < m)
                           ? basis[static_cast<std::size_t>(leaving_row)]
                           : -1;
      fmt::print(stderr,
                 "[PIVOT-TRACE] kind={} iter={} m={} leaving={} bvar={} "
                 "below={} xb={:.12g} score={:.12g} dse={:.12g} "
                 "deficit={:.12g} cand={} first={}:theta{:.12g}:alpha{:.12g} "
                 "enter={} flips={} wnnz={} sparse={} exact_dse={}\n",
                 pivot_trace_kind, iter, m, leaving_row, bvar,
                 leaving_below ? 1 : 0,
                 leaving_row >= 0 ? x_b[leaving_row] : 0.0,
                 best_score,
                 leaving_row >= 0
                     ? dse_w[static_cast<std::size_t>(leaving_row)]
                     : 0.0,
                 primal_deficit, static_cast<int>(bfrt_candidates.size()),
                 first_col, first_theta, first_alpha, entering,
                 static_cast<int>(flipped_vars.size()),
                 static_cast<int>(w_nz.size()),
                 sparse_pricing_taken ? 1 : 0,
                 exact_edge_mode ? 1 : 0);
    }

    // Apply bound flips for all flipped variables BEFORE the basis update.
    // x_b stores B^{-1}(b - A_N x_N), so a non-basic bound flip must update
    // the adjusted RHS first, then apply one FTRAN through the current basis.
    Eigen::VectorXd flip_rhs_delta = Eigen::VectorXd::Zero(m);
    for (int fj : flipped_vars) {
      const double ub_fj = sf.var_ub[fj];
      const bool was_upper = at_upper[static_cast<size_t>(fj)];
      at_upper[static_cast<size_t>(fj)] = was_upper ? 0 : 1;
      const double shift = was_upper ? -ub_fj : ub_fj;  // new value - old value
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, fj); it; ++it) {
        flip_rhs_delta[it.row()] -= it.value() * shift;
      }
      obj += sf.c_max[fj] * shift;
    }
    if (!flipped_vars.empty()) {
      Eigen::VectorXd delta_xb = sbasis.ftran(flip_rhs_delta);
      x_b += delta_xb;
    }

    // FTRAN: compute pivot column direction = B^{-1} * A_entering (in-place).
    auto t4 = it_tp();
    iter_timing.ratio_ns += it_ns(t3, t4);
    // Sparse zeroing: clear only previously nonzero entries.
    for (int i : dir_nz) direction[i] = 0.0;
    for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, entering); it; ++it) {
      direction[it.row()] = it.value();
    }
    // P11: hyper-sparse FTRAN returns dir_nz directly (no O(m) post-scan).
    sbasis.ftran_sparse_inplace(direction, dir_nz);

    // DSE: compute τ = B^{-1} ρ  (ρ = e_leaving = B^{-T} e_p from BTRAN).
    // τ is used in the exact DSE weight update formula below.
    for (int i : dse_tau_nz) dse_tau[i] = 0.0;
    dse_tau_nz.clear();
    for (int i : w_nz) dse_tau[i] = e_leaving[i];
    sbasis.ftran_sparse_inplace(dse_tau, dse_tau_nz);
    auto t5 = it_tp();
    iter_timing.ftran_ns += it_ns(t4, t5);

    // dir_nz already populated by ftran_sparse_inplace above.

    const double pivot_val = direction[leaving_row];

    if (std::abs(pivot_val) < 1e-10) {
      // Numerically zero pivot — undo flips and retry.
      Eigen::VectorXd undo_rhs_delta = Eigen::VectorXd::Zero(m);
      for (int idx = static_cast<int>(flipped_vars.size()) - 1; idx >= 0; --idx) {
        int fj = flipped_vars[static_cast<size_t>(idx)];
        const double ub_fj = sf.var_ub[fj];
        const bool is_upper_now = at_upper[static_cast<size_t>(fj)];
        at_upper[static_cast<size_t>(fj)] = is_upper_now ? 0 : 1;
        const double shift = is_upper_now ? -ub_fj : ub_fj;
        for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, fj); it; ++it) {
          undo_rhs_delta[it.row()] -= it.value() * shift;
        }
        obj += sf.c_max[fj] * shift;
      }
      if (!flipped_vars.empty()) {
        Eigen::VectorXd delta_xb = sbasis.ftran(undo_rhs_delta);
        x_b += delta_xb;
      }
      if (!just_refactored) {
        if (!sbasis.refactorize(basis)) return false;
        sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
        cleanup_reduced_costs();
        just_refactored = true;
        sparse_pricing_taken = false;
        touched_cols.clear();
        continue;
      }
      proved_infeasible = true;
      return false;
    }

    const double old_rc_entering = reduced_costs[entering];
    const int old_leaving = basis[leaving_row];
    const bool entering_was_upper = at_upper[static_cast<size_t>(entering)];
    is_basic[static_cast<size_t>(old_leaving)] = 0;
    at_upper[static_cast<size_t>(old_leaving)] = leaving_below ? 0 : 1;
    basis[leaving_row] = entering;
    is_basic[static_cast<size_t>(entering)] = 1;
    at_upper[static_cast<size_t>(entering)] = 0;

    double step;
    if (leaving_below) {
      step = x_b[leaving_row] / pivot_val;
    } else {
      step = (x_b[leaving_row] - sf.var_ub[old_leaving]) / pivot_val;
    }

    // Sparse x_b update using direction nonzero indices.
    for (int i : dir_nz) x_b[i] -= direction[i] * step;
    x_b[leaving_row] = entering_was_upper ? (sf.var_ub[entering] + step) : step;

    // Reduced cost update: rc -= theta * row_vec, where theta accounts for
    // the dual step to the entering variable's ratio.
    const double rc_scale = old_rc_entering / pivot_val;
    if (sparse_pricing_taken && !touched_cols.empty()) {
      for (int c : touched_cols) {
        reduced_costs[c] -= rc_scale * row_vec[c];
      }
    } else {
      reduced_costs -= rc_scale * row_vec;
    }
    reduced_costs[entering] = 0.0;

    obj += step * old_rc_entering;
    auto t6 = it_tp();
    iter_timing.update_ns += it_ns(t5, t6);

    // NaN/Inf guard: numerical breakdown in pivot updates.
    if (!std::isfinite(obj)) return false;

    sbasis.update_basis_sparse(leaving_row, direction.data(), dir_nz, sf.A, entering);
    auto t7 = it_tp();
    iter_timing.eta_ns += it_ns(t6, t7);

    // ── Exact Dual Steepest Edge (DSE) weight update ──
    // ρ = B_old^{-T} e_p  (e_leaving, from BTRAN)
    // d = B_old^{-1} a_q  (direction, from FTRAN)
    // τ = B_old^{-1} ρ    (dse_tau, from extra FTRAN)
    // σ = ||ρ||²
    // α = pivot_val = d_p
    // Goldfarb-Reid update:
    //   w'_i = w_i − 2(d_i/α)τ_i + (d_i/α)² σ   for i ≠ p
    //   w'_p = σ / α²
    {
      // σ = ||ρ||² via sparse BTRAN nonzero list.
      double sigma = 0.0;
      for (int i : w_nz) sigma += e_leaving[i] * e_leaving[i];
      const double inv_alpha = 1.0 / pivot_val;
      const double inv_alpha2 = inv_alpha * inv_alpha;
      // Sparse update: when d_i = 0 the update term is identically 0.
      for (int i : dir_nz) {
        if (i == leaving_row) continue;
        const double di_over_alpha = direction[i] * inv_alpha;
        dse_w[static_cast<size_t>(i)] +=
            -2.0 * di_over_alpha * dse_tau[i] + di_over_alpha * di_over_alpha * sigma;
        // Clamp to small positive to prevent numerical drift below zero.
        if (dse_w[static_cast<size_t>(i)] < 1e-4)
          dse_w[static_cast<size_t>(i)] = 1e-4;
      }
      // Entering variable now occupies leaving_row: exact weight = σ / α².
      dse_w[static_cast<size_t>(leaving_row)] = std::max(1e-4, sigma * inv_alpha2);
    }
    auto t8 = it_tp();
    iter_timing.dse_ns += it_ns(t7, t8);

    if (sbasis.needs_refactorize(m)) {
      if (!sbasis.refactorize(basis)) return false;
      sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b, reduced_costs, obj);
      cleanup_reduced_costs();
      // Reset sparse pricing state so next iteration does full zeroing.
      sparse_pricing_taken = false;
      touched_cols.clear();
      just_refactored = true;
      // DSE: do NOT reset weights on refactorization — they remain exact
      // (refactorization refreshes the LU decomposition of the same basis).
      iter_timing.refact_ns += it_ns(t8, it_tp());
      ++iter_timing.refact_count;
    } else {
      just_refactored = false;
    }
    ++iter_timing.count;
    iter_timing.dump(m, opt.verbose);
  }

  return false;  // MAX_ITER reached
}

// ── HiGHS-style Phase I helpers (SimplexOptions::phase1_strategy == 1) ──────

/// Apply magnitude-aware randomized perturbations to costs and finite upper
/// bounds, mirroring HEkk::initialiseCost / HEkk::initialiseBound:
///   cost base  = 5e-7 * max(1, |c|_inf);  per-column (1+rand)(|c_j|+1)*base
///   finite ub += (2rand-1) * 1e-7 * max(1, |ub|)
/// Sign convention for costs follows HiGHS: lower-only-bounded (infinite ub)
/// columns get +xpert, boxed columns are pushed away from zero.  The RHS is
/// deliberately NOT perturbed: on tightly-balanced equality systems (e.g.
/// SCUC power-balance rows where total capacity barely meets demand) an
/// independent ±1e-7 RHS perturbation can make the perturbed problem
/// genuinely infeasible — a false certificate.  Bounds at (near-)zero are
/// also left untouched so fixed-at-zero columns stay fixed.  The RNG is
/// deterministically seeded so runs are reproducible.
void highs_style_perturb(StandardFormLP& sf, unsigned long long seed) {
  const int n = static_cast<int>(sf.A.cols());
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  double maxc = 1.0;
  for (int j = 0; j < n; ++j) maxc = std::max(maxc, std::abs(sf.c_max[j]));
  const double cost_base = 5e-7 * maxc;
  const bool has_ub = static_cast<int>(sf.var_ub.size()) == n;
  for (int j = 0; j < n; ++j) {
    const double ub_j = has_ub ? sf.var_ub[j]
                               : std::numeric_limits<double>::infinity();
    const double xpert =
        (1.0 + u01(rng)) * (std::abs(sf.c_max[j]) + 1.0) * cost_base;
    if (std::isfinite(ub_j)) {
      sf.c_max[j] += (sf.c_max[j] >= 0.0) ? xpert : -xpert;
    } else {
      sf.c_max[j] += xpert;
    }
  }
  if (has_ub) {
    for (int j = 0; j < n; ++j) {
      double& ub = sf.var_ub[j];
      if (std::isfinite(ub) && ub > 1e-12) {
        ub += (2.0 * u01(rng) - 1.0) * 1e-7 * std::max(1.0, std::abs(ub));
        ub = std::max(ub, 1e-12);
      }
    }
  }
}

/// Exact-zero dual repair + dual simplex continuation, mirroring HiGHS'
/// shiftCost-pinned-at-zero / correctDualInfeasibilities + continue pattern:
/// for every non-basic column violating dual feasibility, shift its cost so
/// the reduced cost lands exactly on the dual boundary (with a 1e-10 margin
/// inside the feasible side), then run the dual simplex to restore primal
/// feasibility.  Artificial columns are blocked from entering and are never
/// shifted.  `sf` is modified in place (cost shifts); on success the basis /
/// x_b / at_upper describe a primal-feasible point of `sf`.
bool dual_cleanup_resolve(StandardFormLP& sf, std::vector<int>& basis,
                          const SimplexOptions& opt, SparseBasis& sbasis,
                          Eigen::VectorXd& x_b, Eigen::VectorXd& reduced_costs,
                          double& obj, std::vector<char>& at_upper) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
  std::vector<char> can_enter(static_cast<std::size_t>(n), 1);
  for (int i = 0; i < m; ++i) {
    const int art = sf.row_to_artificial_col[i];
    if (art >= 0 && art < n) {
      is_artificial[static_cast<std::size_t>(art)] = 1;
      can_enter[static_cast<std::size_t>(art)] = 0;
    }
  }
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int i = 0; i < m; ++i) {
    if (basis[i] >= 0 && basis[i] < n) {
      is_basic[static_cast<std::size_t>(basis[i])] = 1;
    }
  }

  if (!sbasis.refactorize(basis)) return false;
  sparse_full_recompute(sf, basis, at_upper, is_basic, sbasis, x_b,
                        reduced_costs, obj);

  constexpr double kShiftMargin = 1e-10;
  for (int j = 0; j < n; ++j) {
    if (is_basic[static_cast<std::size_t>(j)] ||
        is_artificial[static_cast<std::size_t>(j)]) {
      continue;
    }
    const bool j_up = at_upper[static_cast<std::size_t>(j)];
    if (!j_up && reduced_costs[j] > opt.optimality_tol) {
      sf.c_max[j] -= (reduced_costs[j] + kShiftMargin);
      reduced_costs[j] = -kShiftMargin;
    } else if (j_up && reduced_costs[j] < -opt.optimality_tol) {
      sf.c_max[j] += (-reduced_costs[j] + kShiftMargin);
      reduced_costs[j] = kShiftMargin;
    }
  }

  bool inf = false, cut = false;
  const bool ok = sparse_dual_simplex_reoptimize(
      sf, basis, can_enter, opt, sbasis, x_b, reduced_costs, obj, at_upper,
      inf, cut);
  return ok && !inf;
}

/// Primal feasibility of the current basic state (x_b >= -tol, plus upper
/// bound violations when finite var_ub exists).
bool primal_state_feasible(const StandardFormLP& sf,
                           const std::vector<int>& basis,
                           const Eigen::VectorXd& x_b, double tol) {
  const int m = static_cast<int>(sf.A.rows());
  if (!approx_nonnegative(x_b, tol)) return false;
  for (int i = 0; i < m; ++i) {
    const int bvar = basis[static_cast<std::size_t>(i)];
    if (bvar < 0 || bvar >= static_cast<int>(sf.var_ub.size())) continue;
    const double ub_i = sf.var_ub[bvar];
    if (std::isfinite(ub_i) && x_b[i] > ub_i + tol) return false;
  }
  return true;
}

}  // namespace

void clear_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb) {
  if (sb) sb->clear_etas();
}

void truncate_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb, int target_count) {
  if (!sb) return;
  const int cur = sb->eta_count();
  if (target_count < cur) {
    // Truncate etas back to the target count. This restores the SparseBasis
    // to the state it was in at 'target_count' etas (before sibling mutation).
    sb->truncate_etas_to(target_count);
  }
}

int sparse_basis_eta_count(const std::shared_ptr<BasisOps>& sb) {
  if (!sb) return 0;
  return sb->eta_count();
}

int sparse_basis_generation(const std::shared_ptr<BasisOps>& sb) {
  if (sb) {
    return sb->generation();
  }
  return -1;
}

SparseFactorTelemetry get_sparse_basis_factor_telemetry(
    const std::shared_ptr<BasisOps>& sb) {
  if (!sb) return {};
  return sb->factor_telemetry();
}

void rebind_sparse_basis_matrix(const std::shared_ptr<BasisOps>& sb,
                                const Eigen::SparseMatrix<double>& A) {
  if (!sb) return;
  sb->rebind_A(A);
}

bool sparse_basis_bound_to_matrix(const std::shared_ptr<BasisOps>& sb,
                                  const Eigen::SparseMatrix<double>& A) {
  if (!sb) return false;
  return sb->bound_to_A(A);
}

Eigen::VectorXd sparse_basis_btran(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                                   const Eigen::VectorXd& rhs) {
  if (!cached_sparse_basis) {
    return Eigen::VectorXd::Zero(rhs.size());
  }
  return cached_sparse_basis->btran(rhs);
}

bool sparse_basis_inverse_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::VectorXd& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->basis_inverse_row(row, out);
}

bool sparse_basis_inverse_row_sparse_entries(
    const std::shared_ptr<BasisOps>& cached_sparse_basis,
    int row,
    std::vector<std::pair<int, double>>& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->basis_inverse_row_sparse_entries(row, out);
}

bool sparse_basis_tableau_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::RowVectorXd& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->tableau_row(row, out);
}

/// Internal implementation of solve_lp_with_basis with an explicit
/// cold-start escalation level (see SimplexOptions::escalation_max_level).
/// Level 0 is the historical behaviour; higher levels use progressively more
/// conservative numerics (more Ruiz rounds, stricter UMFPACK pivoting).
static SimplexResult solve_lp_with_basis_impl(const LPModel& lp,
                                              const SimplexOptions& input_opt,
                                              const SimplexBasis* basis_hint,
                                              int esc_level) {
  SimplexOptions opt = input_opt;
  if (opt.phase1_strategy == 0) {
    if (const char* e = std::getenv("MIPSOLVERS_SIMPLEX_PHASE1")) {
      if (e[0] == '1') opt.phase1_strategy = 1;
    }
  }
  bool local_time_limit_hit = false;
  if (opt.time_limit_hit == nullptr) opt.time_limit_hit = &local_time_limit_hit;
  SimplexResult out;
  const auto solve_wall_t0 = std::chrono::steady_clock::now();
  g_simplex_iter_count = 0;
  struct TimeLimitStatusGuard {
    SimplexResult& out;
    const SimplexOptions& opt;
    const std::chrono::steady_clock::time_point& start;
    ~TimeLimitStatusGuard() {
      out.result.stats.iterations = g_simplex_iter_count;
      const bool hit = opt.time_limit_hit != nullptr && *opt.time_limit_hit;
      if (!out.result.stats.success &&
          (hit || simplex_wall_time_limit_hit(opt, start))) {
        mark_simplex_time_limit(out.result.stats, start);
      }
    }
  } time_limit_guard{out, opt, solve_wall_t0};
  out.form = build_standard_form_lp(lp);
  ruiz_scale_standard_form(out.form,
                           esc_level >= 1 ? opt.escalation_ruiz_rounds : 10);
  out.result.stats.solver_name = "NativeSimplex";

  const int m = out.form.A.rows();
  const int n = out.form.A.cols();
  const bool exact_edge_mode =
      opt.exact_dse_initialization || simplex_exact_edge_env_enabled();
  if (m == 0) {
    out.x_std = Eigen::VectorXd::Zero(n);
    out.result.x = extract_solution(out.form, out.x_std);
    out.result.stats.success = true;
    out.result.stats.objective = out.form.objective_const;
    out.result.stats.status = "Optimal";
    out.exact_optimal = true;
    return out;
  }

  std::vector<int> basis(static_cast<size_t>(m), -1);
  std::vector<char> can_enter(static_cast<size_t>(n), 1);
  std::vector<char> at_upper(static_cast<size_t>(n), 0);
  for (int i = 0; i < m; ++i) {
    if (out.form.row_to_slack_col[i] >= 0) {
      basis[i] = out.form.row_to_slack_col[i];
    } else if (out.form.row_to_artificial_col[i] >= 0) {
      basis[i] = out.form.row_to_artificial_col[i];
    }
  }

  Eigen::MatrixXd binv;
  Eigen::VectorXd x_b;
  Eigen::VectorXd reduced_costs;
  double obj = 0.0;
  const bool has_ub = out.form.var_ub.size() == n;

  if (basis_hint != nullptr && basis_hint->rows == m && basis_hint->cols == n &&
      static_cast<int>(basis_hint->index_count()) == m) {
    basis = basis_hint->basis_indices();
    if (!basis_hint->at_upper.empty()) at_upper = basis_hint->at_upper;

    {
      // Dense warm-start for all problem sizes.
      bool state_ok = false;
      if (basis_hint->cached_inverse &&
          basis_hint->cached_inverse->rows() == m &&
          basis_hint->cached_inverse->cols() == m) {
        binv = *basis_hint->cached_inverse;
        x_b = binv * out.form.b;
        if (basis_hint->cached_reduced_costs &&
            basis_hint->cached_reduced_costs->size() == n) {
          reduced_costs = *basis_hint->cached_reduced_costs;
        } else {
          const int bsz = static_cast<int>(basis.size());
          Eigen::VectorXd c_b(bsz);
          for (int i = 0; i < bsz; ++i) c_b[i] = out.form.c_max[basis[i]];
          const Eigen::VectorXd pi = binv.transpose() * c_b;
          reduced_costs = out.form.c_max - Eigen::VectorXd(out.form.A.transpose() * pi);
        }
        const int bsz = static_cast<int>(basis.size());
        Eigen::VectorXd c_b(bsz);
        for (int i = 0; i < bsz; ++i) c_b[i] = out.form.c_max[basis[i]];
        if (has_ub) {
          std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
          for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
          for (int j = 0; j < n; ++j) {
            if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
            const double uj = out.form.var_ub[j];
            if (!std::isfinite(uj)) continue;
            for (Eigen::SparseMatrix<double>::InnerIterator it(out.form.A, j); it; ++it) {
              x_b -= it.value() * uj * binv.col(it.row());
            }
          }
        }
        obj = c_b.dot(x_b);
        if (has_ub) {
          std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
          for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
          for (int j = 0; j < n; ++j) {
            if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
            const double uj = out.form.var_ub[j];
            if (std::isfinite(uj)) obj += out.form.c_max[j] * uj;
          }
        }
        state_ok = true;
      } else {
        state_ok = compute_basis_state(out.form, basis, binv, x_b, reduced_costs, obj);
        if (state_ok && has_ub) {
          for (int j = 0; j < n; ++j) {
            if (!at_upper[static_cast<size_t>(j)]) continue;
            bool is_basic_j = false;
            for (int idx : basis) { if (idx == j) { is_basic_j = true; break; } }
            if (is_basic_j) continue;
            const double uj = out.form.var_ub[j];
            if (!std::isfinite(uj)) continue;
            for (Eigen::SparseMatrix<double>::InnerIterator it(out.form.A, j); it; ++it) {
              x_b -= it.value() * uj * binv.col(it.row());
            }
          }
          const int bsz2 = static_cast<int>(basis.size());
          Eigen::VectorXd c_b2(bsz2);
          for (int i = 0; i < bsz2; ++i) c_b2[i] = out.form.c_max[basis[i]];
          obj = c_b2.dot(x_b);
          for (int j = 0; j < n; ++j) {
            if (!at_upper[static_cast<size_t>(j)]) continue;
            bool is_basic_j = false;
            for (int idx : basis) { if (idx == j) { is_basic_j = true; break; } }
            if (is_basic_j) continue;
            const double uj = out.form.var_ub[j];
            if (std::isfinite(uj)) obj += out.form.c_max[j] * uj;
          }
        }
      }

      if (state_ok) {
        out.solved_from_hint = true;
        bool primal_feas = approx_nonnegative(x_b, opt.feasibility_tol);
        if (primal_feas && has_ub) {
          for (int i = 0; i < m; ++i) {
            const int bvar = basis[i];
            const double ub_i = out.form.var_ub[bvar];
            if (std::isfinite(ub_i) && x_b[i] > ub_i + opt.feasibility_tol) {
              primal_feas = false;
              break;
            }
          }
        }
        if (primal_feas) {
          out.dual_reoptimized = true;
        } else {
          std::vector<char> hint_can_enter(static_cast<size_t>(n), 1);
          for (int i = 0; i < m; ++i) {
            const int art = out.form.row_to_artificial_col[i];
            if (art >= 0) hint_can_enter[static_cast<size_t>(art)] = 0;
          }
          if (opt.prefer_dual_simplex_reopt &&
              approx_dual_feasible(reduced_costs, basis, hint_can_enter, at_upper, opt.optimality_tol) &&
              dual_simplex_reoptimize(out.form, basis, hint_can_enter, opt, binv, x_b, reduced_costs, obj,
                                      at_upper, /*state_initialized=*/true)) {
            out.dual_reoptimized = true;
          } else {
            out.solved_from_hint = false;
          }
        }
      }
    }
  }

  if (!out.solved_from_hint) {
    at_upper.assign(static_cast<size_t>(n), 0);

    // Detect provable infeasibility when b[i] < 0 (can happen after
    // update_standard_form_bounds tightens lb_shift).
    bool cold_start_infeasible = false;
    for (int i = 0; i < m; ++i) {
      if (out.form.b[i] >= -opt.feasibility_tol) continue;
      bool all_nonneg = true;
      for (int col = 0; col < out.form.A.outerSize() && all_nonneg; ++col) {
        if (has_ub && out.form.var_ub[col] < opt.feasibility_tol) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(out.form.A, col); it; ++it) {
          if (it.row() == i && it.value() < -opt.feasibility_tol) {
            all_nonneg = false;
            break;
          }
        }
      }
      if (all_nonneg) {
        cold_start_infeasible = true;
        break;
      }
    }
    if (cold_start_infeasible) {
      mark_proven_infeasible(out.result.stats);
      return out;
    }

    // Reset basis to initial slack/artificial when hint fails.
    for (int i = 0; i < m; ++i) {
      if (out.form.row_to_slack_col[i] >= 0) {
        basis[i] = out.form.row_to_slack_col[i];
      } else if (out.form.row_to_artificial_col[i] >= 0) {
        basis[i] = out.form.row_to_artificial_col[i];
      } else {
        basis[i] = -1;
      }
    }

    // Crash: replace artificial columns with original-variable columns
    // to reduce Phase I / Big-M work.  The row-major scan makes the
    // candidate search O(nnz) regardless of problem size.
    crash_artificial_basis(out.form, basis);


    if (m > kSparseSimplexThreshold) {
      // --- Sparse LU simplex for large problems ---
      SparseBasis sbasis(out.form.A,
             esc_level >= 2
                 ? SparseBasis::FactorBackendKind::KluRescue
                 : SparseBasis::from_public_backend(opt.factor_backend));
      if (esc_level == 1) {
        sbasis.set_pivot_tolerance(opt.escalation_umfpack_pivot_tolerance);
      }

      // Validate crash: the greedy crash (m > 200) does not track x_b,
      // so the BFS may be primal infeasible.  Compute x_b = B^{-1} b
      // and revert swaps for rows where x_b[i] < 0 or above upper bound.
      // This is essential because dual Phase I requires a valid basis.
      if (m > 200) {
        if (sbasis.refactorize(basis)) {
          Eigen::VectorXd xb = sbasis.ftran(out.form.b);
          for (int i = 0; i < m; ++i) {
            const int art = out.form.row_to_artificial_col[i];
            if (art < 0) continue;           // row has no artificial
            if (basis[i] == art) continue;    // not swapped by crash
            const bool neg = xb[i] < -1e-8;
            const double ub_i = has_ub ? out.form.var_ub[basis[i]]
                                       : std::numeric_limits<double>::infinity();
            const bool ub_viol = std::isfinite(ub_i) && xb[i] > ub_i + 1e-8;
            if (neg || ub_viol) {
              basis[i] = art;  // revert to artificial
            }
          }
        } else {
          // Crash produced a singular basis — revert ALL crash swaps
          // back to slacks/artificials.
          for (int i = 0; i < m; ++i) {
            if (out.form.row_to_slack_col[i] >= 0) {
              basis[i] = out.form.row_to_slack_col[i];
            } else if (out.form.row_to_artificial_col[i] >= 0) {
              basis[i] = out.form.row_to_artificial_col[i];
            }
          }
        }
      }

      // ═══════════════════════════════════════════════════════════════════
      // Dual Phase I (Objective Shifting) — no Big-M, no weighted penalty.
      // Compute reduced costs for the current crash basis. For variables
      // that violate dual feasibility, apply exact cost shifts to make
      // the basis dual-feasible. Then run dual simplex (with BFRT) to
      // reach primal feasibility, remove shifts, and finish with primal
      // simplex Phase II.
      // ═══════════════════════════════════════════════════════════════════

      // Factorize the crash basis.
      if (!sbasis.refactorize(basis)) {
        // Crash basis singular — revert to slack/artificial identity basis.
        for (int i = 0; i < m; ++i) {
          if (out.form.row_to_slack_col[i] >= 0) {
            basis[i] = out.form.row_to_slack_col[i];
          } else if (out.form.row_to_artificial_col[i] >= 0) {
            basis[i] = out.form.row_to_artificial_col[i];
          }
        }
        if (!sbasis.refactorize(basis)) {
          out.result.stats.status = "Cold start: basis factorization failed";
          return out;
        }
      }

      // ═══════════════════════════════════════════════════════════════════
      // Dual Phase I — Objective Shifting (no Big-M, no weighted penalty).
      //
      // 1. Compute reduced costs c̄ for the crash basis.
      // 2. For each non-basic j violating dual feasibility, apply a
      //    cost shift Δ_j = |c̄_j| + 1 to make the basis dual-feasible.
      // 3. Force artificial var_ub = 0 so basic artificials violate
      //    their upper bound and get pivoted out by dual simplex.
      // 4. Run dual simplex (with BFRT) to reach primal feasibility.
      // 5. Remove all shifts, recompute with the real objective.
      // 6. Phase II: primal simplex with real objective.
      //
      // Fallback: if dual simplex doesn't converge, use strict
      // Primal Phase I (c_art = −1, c_others = 0).
      // ═══════════════════════════════════════════════════════════════════

      // Build is_basic mask for crash basis.
      std::vector<char> is_basic_cr(static_cast<size_t>(n), 0);
      for (int i = 0; i < m; ++i) {
        if (basis[i] >= 0 && basis[i] < n)
          is_basic_cr[static_cast<size_t>(basis[i])] = 1;
      }

      // Compute reduced costs from crash basis with real objective.
      sparse_full_recompute(out.form, basis, at_upper, is_basic_cr,
                            sbasis, x_b, reduced_costs, obj);

      // HiGHS-style Phase I (SimplexOptions::phase1_strategy == 1): run the
      // Phase-I machinery on a perturbed copy of the standard form
      // (randomized cost/RHS/bound perturbations, deterministic seed) with
      // exact-zero cost shifts, then restore the unperturbed problem before
      // Phase II.  phase1_form selects the form Phase I operates on.
      const bool highs_phase1 = (opt.phase1_strategy == 1);
      StandardFormLP perturbed;
      if (highs_phase1) {
        perturbed = out.form;
        highs_style_perturb(
            perturbed,
            0x5EED5EEDULL + static_cast<unsigned long long>(esc_level));
      }
      const StandardFormLP& phase1_form = highs_phase1 ? perturbed : out.form;

      // Build shifted LP: force artificial ub = 0.
      StandardFormLP shifted = phase1_form;
      for (int i = 0; i < m; ++i) {
        const int art = shifted.row_to_artificial_col[i];
        if (art >= 0) shifted.var_ub[art] = 0.0;
      }

      // Recompute state with artificial bounds capped at 0.
      sparse_full_recompute(shifted, basis, at_upper, is_basic_cr,
                            sbasis, x_b, reduced_costs, obj);

      // Apply exact cost shifts for dual feasibility.  Legacy uses a uniform
      // ±1 margin; HiGHS-style pins the reduced cost exactly on the dual
      // boundary (1e-10 margin), mirroring HEkkDual::shiftCost.
      const double shift_margin = highs_phase1 ? 1e-10 : 1.0;
      for (int j = 0; j < n; ++j) {
        if (is_basic_cr[static_cast<size_t>(j)]) continue;
        const bool j_up = at_upper[static_cast<size_t>(j)];
        if (!j_up && reduced_costs[j] > opt.optimality_tol) {
          shifted.c_max[j] -= (reduced_costs[j] + shift_margin);
          reduced_costs[j] = -shift_margin;
        } else if (j_up && reduced_costs[j] < -opt.optimality_tol) {
          shifted.c_max[j] += (-reduced_costs[j] + shift_margin);
          reduced_costs[j] = shift_margin;
        }
      }
      // Recompute shifted objective from scratch.
      {
        obj = 0.0;
        for (int i = 0; i < m; ++i)
          obj += shifted.c_max[basis[i]] * x_b[i];
        for (int j = 0; j < n; ++j) {
          if (!is_basic_cr[static_cast<size_t>(j)] &&
              at_upper[static_cast<size_t>(j)] &&
              std::isfinite(shifted.var_ub[j]))
            obj += shifted.c_max[j] * shifted.var_ub[j];
        }
      }

      // Run dual simplex with shifted costs (artificials blocked).
      std::vector<char> dp1_can_enter(static_cast<size_t>(n), 1);
      for (int i = 0; i < m; ++i) {
        const int art = shifted.row_to_artificial_col[i];
        if (art >= 0) dp1_can_enter[static_cast<size_t>(art)] = 0;
      }
      bool dp1_inf = false, dp1_cut = false;
      bool dp1_ok = sparse_dual_simplex_reoptimize(
          shifted, basis, dp1_can_enter, opt, sbasis,
          x_b, reduced_costs, obj, at_upper,
          dp1_inf, dp1_cut);

      if (opt.verbose) {
        fprintf(stderr, "[SIMPLEX-DIAG] Dual Phase I: ok=%d inf=%d cut=%d m=%d n=%d\n",
                dp1_ok ? 1 : 0, dp1_inf ? 1 : 0, dp1_cut ? 1 : 0, m, n);
      }

      // After Dual Phase I succeeds, we need to refactorize sbasis for the original LP
      // because the factorization might be stale (numerical accumulation during pivots).
      // Also, reset at_upper ONLY for artificial columns since they have var_ub=0 in shifted
      // but var_ub=+inf in original, and we want them at value 0 in original too.
      for (int i = 0; i < m; ++i) {
        const int art = out.form.row_to_artificial_col[i];
        if (art >= 0) at_upper[static_cast<size_t>(art)] = 0;
      }
      
      // Refactorize the basis to remove any numerical drift from Dual Phase I pivots.
      sbasis.refactorize(basis);

      if (dp1_inf) {
        mark_proven_infeasible(out.result.stats);
        return out;
      }

      if (!dp1_ok && !dp1_cut) {
        // Fallback: strict Primal Phase I (c_art = −1, c_others = 0).
        // IMPORTANT: Reset at_upper since Dual Phase I may have corrupted it.
        std::fill(at_upper.begin(), at_upper.end(), 0);
        
        // Also reset the basis to the crash basis (fresh start).
        for (int i = 0; i < m; ++i) {
          if (out.form.row_to_slack_col[i] >= 0) {
            basis[i] = out.form.row_to_slack_col[i];
          } else if (out.form.row_to_artificial_col[i] >= 0) {
            basis[i] = out.form.row_to_artificial_col[i];
          }
        }
        
        StandardFormLP phase1 = phase1_form;
        phase1.c_max.setZero();
        std::vector<char> p1_can_enter(static_cast<size_t>(n), 1);
        for (int i = 0; i < m; ++i) {
          const int art = phase1.row_to_artificial_col[i];
          if (art >= 0) {
            phase1.c_max[art] = -1.0;
            p1_can_enter[static_cast<size_t>(art)] = 0;
          }
        }
        if (!sparse_primal_simplex_optimize(phase1, basis, p1_can_enter, opt,
                                            sbasis, x_b, reduced_costs, obj, at_upper)) {
          out.result.stats.status = "Simplex Phase I failed";
          return out;
        }
        if (-obj > opt.feasibility_tol) {
          mark_proven_infeasible(out.result.stats);
          return out;
        }
      }

      if (highs_phase1) {
        // Restoration pass (HiGHS cleanup + continue): Phase I ran on
        // perturbed data, so re-sync the state with the TRUE form; if the
        // point drifted out of primal feasibility, repair it with the dual
        // simplex on an exactly-shifted copy of the true form.
        std::vector<char> is_basic_rt(static_cast<size_t>(n), 0);
        for (int i = 0; i < m; ++i) {
          if (basis[i] >= 0 && basis[i] < n)
            is_basic_rt[static_cast<size_t>(basis[i])] = 1;
        }
        sparse_full_recompute(out.form, basis, at_upper, is_basic_rt,
                              sbasis, x_b, reduced_costs, obj);
        if (!primal_state_feasible(out.form, basis, x_b,
                                   opt.feasibility_tol)) {
          StandardFormLP repair = out.form;
          for (int i = 0; i < m; ++i) {
            const int art = repair.row_to_artificial_col[i];
            if (art >= 0) repair.var_ub[art] = 0.0;
          }
          if (!dual_cleanup_resolve(repair, basis, opt, sbasis, x_b,
                                    reduced_costs, obj, at_upper)) {
            out.result.stats.status = "Simplex Phase I failed";
            return out;
          }
          // Re-sync artificial at_upper flags after the repair pass.
          for (int i = 0; i < m; ++i) {
            const int art = out.form.row_to_artificial_col[i];
            if (art >= 0) at_upper[static_cast<size_t>(art)] = 0;
          }
        }
      }

      // Check residual artificials.
      {
        bool has_art = false;
        for (int i = 0; i < m; ++i) {
          const int art = out.form.row_to_artificial_col[i];
          if (art >= 0 && basis[i] == art && x_b[i] > opt.feasibility_tol) {
            has_art = true; break;
          }
        }
        if (has_art) {
          mark_proven_infeasible(out.result.stats);
          return out;
        }
      }

      // Remove shifts: recompute with the REAL objective.
      {
        std::vector<char> is_basic_now(static_cast<size_t>(n), 0);
        for (int i = 0; i < m; ++i) {
          if (basis[i] >= 0 && basis[i] < n)
            is_basic_now[static_cast<size_t>(basis[i])] = 1;
        }
        sparse_full_recompute(out.form, basis, at_upper, is_basic_now,
                              sbasis, x_b, reduced_costs, obj);
      }

      // Phase II: optimize with real objective (artificials blocked).
      for (int i = 0; i < m; ++i) {
        const int art = out.form.row_to_artificial_col[i];
        if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
      }

      if (!sparse_primal_simplex_optimize(out.form, basis, can_enter, opt,
                                          sbasis, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }

      // Persist SparseBasis for child nodes.
      {
        auto sb_shared = std::make_shared<SparseBasis>(
          out.form.A, SparseBasis::from_public_backend(opt.factor_backend));
        sb_shared->refactorize(basis);
        // P10: inherit converged DSE weights from the solving sbasis.
        sb_shared->copy_dse_weights_from(sbasis);
        if (exact_edge_mode && !sb_shared->dse_weights_match(basis)) {
          sb_shared->save_dse_weights(
              sb_shared->compute_exact_dse_weights(), basis);
        }
        out.basis.cached_sparse_basis = sb_shared;
      }
    } else {
      // --- Dense simplex: strict Primal Phase I (c_art=−1, c_others=0) ---
      StandardFormLP phase1 = out.form;
      phase1.c_max.setZero();
      std::vector<char> phase1_can_enter(static_cast<size_t>(n), 1);
      for (int i = 0; i < m; ++i) {
        const int art = phase1.row_to_artificial_col[i];
        if (art >= 0) {
          phase1.c_max[art] = -1.0;
          phase1_can_enter[static_cast<size_t>(art)] = 0;
        }
      }
      if (!primal_simplex_optimize(phase1, basis, phase1_can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase I failed";
        return out;
      }
      if (-obj > opt.feasibility_tol) {
        out.result.stats.status = "LP infeasible";
        extract_farkas_certificate(out.form, basis, binv, out.result.stats);
        return out;
      }

      for (int i = 0; i < m; ++i) {
        const int art = out.form.row_to_artificial_col[i];
        if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
      }
      if (!primal_simplex_optimize(out.form, basis, can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
    }
  } else if (!out.dual_reoptimized) {
    for (int i = 0; i < m; ++i) {
      const int art = out.form.row_to_artificial_col[i];
      if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
    }
    if (m > kSparseSimplexThreshold) {
      SparseBasis sbasis(out.form.A,
             esc_level >= 2
                 ? SparseBasis::FactorBackendKind::KluRescue
                 : SparseBasis::from_public_backend(opt.factor_backend));
      if (esc_level == 1) {
        sbasis.set_pivot_tolerance(opt.escalation_umfpack_pivot_tolerance);
      }
      if (!sparse_primal_simplex_optimize(out.form, basis, can_enter, opt,
                                          sbasis, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
      {
        auto sb_shared = std::make_shared<SparseBasis>(
          out.form.A, SparseBasis::from_public_backend(opt.factor_backend));
        sb_shared->refactorize(basis);
        if (exact_edge_mode) {
          sb_shared->save_dse_weights(
              sb_shared->compute_exact_dse_weights(), basis);
        }
        out.basis.cached_sparse_basis = sb_shared;
      }
    } else {
      if (!primal_simplex_optimize(out.form, basis, can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
    }
  }

  const bool frontier_remap_mode =
      !opt.suppress_degenerate_frontier_remap &&
      (opt.enable_degenerate_frontier_remap ||
       simplex_degenerate_frontier_remap_env_enabled());
  if (frontier_remap_mode && m > kSparseSimplexThreshold &&
      static_cast<int>(reduced_costs.size()) == n) {
    SparseBasis remap_basis(
        out.form.A, SparseBasis::from_public_backend(opt.factor_backend));
    if (remap_basis.refactorize(basis)) {
      const int remapped = remap_degenerate_original_frontier(
          out.form, basis, at_upper, remap_basis, x_b, reduced_costs, obj, opt);
      auto sb_shared = std::make_shared<SparseBasis>(
          out.form.A, SparseBasis::from_public_backend(opt.factor_backend));
      if (sb_shared->refactorize(basis)) {
        if (exact_edge_mode) {
          sb_shared->save_dse_weights(
              sb_shared->compute_exact_dse_weights(), basis);
        }
        out.basis.cached_sparse_basis = sb_shared;
      }
      if (root_coldstate_diag_enabled()) {
        trace_root_coldstate(out.form, basis, at_upper, &x_b, &reduced_costs,
                             "after_degenerate_frontier_remap",
                             0,
                             remapped > 0 ? "zero_pivot_remap" : "none",
                             remapped, true);
      }
    }
  }

  out.basis.indices = basis;
  out.basis.rows = m;
  out.basis.cols = n;
  out.basis.at_upper = at_upper;
  out.basis_inverse = binv;
  out.x_basic = x_b;
  out.reduced_costs = reduced_costs;
  out.max_objective = obj;
  out.exact_optimal = true;
  // Build x_std: basic vars from x_b, non-basic at upper from var_ub[j].
  out.x_std = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < m; ++i) {
    if (basis[i] >= 0 && basis[i] < n) {
      out.x_std[basis[i]] = x_b[i];
    }
  }
  if (has_ub) {
    for (int j = 0; j < n; ++j) {
      if (at_upper[static_cast<size_t>(j)]) {
        bool is_basic_j = false;
        for (int idx : basis) { if (idx == j) { is_basic_j = true; break; } }
        // Bug fix: skip infinite upper bounds (e.g., artificials).
        // A variable at "infinite upper bound" doesn't make sense; treat as 0.
        if (!is_basic_j && std::isfinite(out.form.var_ub[j])) {
          out.x_std[j] = out.form.var_ub[j];
        }
      }
    }
  }
  out.result.x = extract_solution(out.form, out.x_std);
  out.result.stats.success = true;
  out.result.stats.iterations = 0;
  out.result.stats.objective = out.form.objective_const - obj;
  out.result.stats.status = "Optimal";
  out.result.stats.primal_feas = 0.0;
  out.result.stats.residual_inf = 0.0;
  trace_lp_basis_state(out, basis, at_upper, x_b, "final");
  populate_dual_certificate(out.form, basis, at_upper, out.basis_inverse,
                            out.basis.cached_sparse_basis,
                            out.reduced_costs, out.result);
  return out;
}

// ── Cold-start escalation chain ─────────────────────────────────────────────
namespace {

/// Original-space feasibility audit used to reject numerically false
/// "Optimal" results from cold-start solves.  Accepts when
///   max(row violation, bound violation) <= tol * max(1, |b|_inf, |beq|_inf).
bool lp_solution_residual_acceptable(const LPModel& lp,
                                     const Eigen::VectorXd& x, double tol) {
  if (x.size() != static_cast<int>(lp.vars.size())) return false;
  double viol = 0.0;
  double scale = 1.0;
  const Eigen::VectorXd ax = lp.A * x;
  for (int i = 0; i < ax.size(); ++i) {
    viol = std::max(viol, ax[i] - lp.b[i]);
    const double lhs = lp_row_lhs_or_neg_inf(lp, i);
    if (std::isfinite(lhs)) viol = std::max(viol, lhs - ax[i]);
    scale = std::max(scale, std::abs(lp.b[i]));
  }
  const Eigen::VectorXd aeqx = lp.Aeq * x;
  for (int i = 0; i < aeqx.size(); ++i) {
    viol = std::max(viol, std::abs(aeqx[i] - lp.beq[i]));
    scale = std::max(scale, std::abs(lp.beq[i]));
  }
  for (int j = 0; j < x.size(); ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    if (std::isfinite(v.lb)) viol = std::max(viol, v.lb - x[j]);
    if (std::isfinite(v.ub)) viol = std::max(viol, x[j] - v.ub);
  }
  return viol <= tol * scale;
}

/// Highest escalation level currently implemented.  Level 2 is the KLU
/// rescue backend (FactorBackendKind::KluRescue), available only when KLU
/// is compiled in; otherwise the chain stops at level 1.
int max_available_escalation_level() {
#ifdef MIPSOLVERS_HAVE_KLU
  return 2;
#else
  return 1;
#endif
}

}  // namespace

SimplexResult solve_lp_with_basis(const LPModel& lp,
                                  const SimplexOptions& input_opt,
                                  const SimplexBasis* basis_hint) {
  // Escalation only rescues cold-start solves; warm-started tree node LPs
  // keep their single-attempt behaviour.
  const int max_level =
      input_opt.allow_cold_start
          ? std::min(std::max(0, input_opt.escalation_max_level),
                     max_available_escalation_level())
          : 0;
  const int start_level =
      std::clamp(input_opt.escalation_start_level, 0, max_level);
  const auto esc_wall_t0 = std::chrono::steady_clock::now();
  for (int level = start_level;; ++level) {
    SimplexResult res = solve_lp_with_basis_impl(lp, input_opt, basis_hint, level);

    // False-optimal audit: a cold-start "Optimal" whose original-space
    // residual is too large is treated as a failure and retried at the next
    // escalation level (guards against accepting garbage from a bad basis).
    if (res.result.stats.success && !res.solved_from_hint &&
        level < max_level &&
        !lp_solution_residual_acceptable(lp, res.result.x,
                                         input_opt.escalation_residual_tol)) {
      continue;
    }
    if (res.result.stats.success || level >= max_level) return res;
    // Never retry terminal answers or time limits.
    if (res.result.stats.has_farkas_certificate) return res;
    if (res.result.stats.status == "Time limit") return res;
    // Respect the caller's wall-clock budget between levels: escalation
    // retries are expensive (stricter pivoting, KLU re-factorization) and
    // must not double the effective time limit.
    if (simplex_wall_time_limit_hit(input_opt, esc_wall_t0)) {
      mark_simplex_time_limit(res.result.stats, esc_wall_t0);
      return res;
    }
  }
}

// ── Thread-safe counters for diagnosing warm-start performance ──
namespace {
struct SolveLPCounters {
  std::atomic<int> path_a_entered{0};
  std::atomic<int> path_a_ok{0};
  std::atomic<int> path_a_infeasible{0};
  std::atomic<int> path_a_fail{0};
  std::atomic<int> cold_start{0};
  std::atomic<int64_t> cold_us{0};      // cumulative cold-start microseconds
  std::atomic<int64_t> warm_us{0};      // cumulative warm-start microseconds
  std::atomic<int64_t> path_a_fail_us{0}; // time wasted on failed warm-starts
};
static SolveLPCounters g_slp_counters;
}  // namespace

void dump_solve_lp_counters() {
  auto& c = g_slp_counters;
  if (c.path_a_entered.load() == 0 && c.cold_start.load() == 0) return;
  fmt::print(stderr, "[LP-STATS] PATH_A: entered={} ok={} infeas={} fail={} | COLD={} | warm_ms={:.0f} cold_ms={:.0f} fail_ms={:.0f}\n",
    c.path_a_entered.load(), c.path_a_ok.load(), c.path_a_infeasible.load(), c.path_a_fail.load(),
    c.cold_start.load(),
    c.warm_us.load() / 1000.0, c.cold_us.load() / 1000.0, c.path_a_fail_us.load() / 1000.0);
  // Reset for next solve
  c.path_a_entered = 0; c.path_a_ok = 0; c.path_a_infeasible = 0; c.path_a_fail = 0;
  c.cold_start = 0; c.cold_us = 0; c.warm_us = 0; c.path_a_fail_us = 0;
}

bool set_vendored_highs_sf_backend_thread_enabled(bool enabled) {
  const bool old = tl_vendored_highs_sf_backend_enabled;
  tl_vendored_highs_sf_backend_enabled = enabled;
  return old;
}

bool vendored_highs_sf_backend_thread_enabled() {
  return tl_vendored_highs_sf_backend_enabled;
}

SimplexResult solve_lp_from_sf(const StandardFormLP& sf,
                               const SimplexOptions& input_opt,
                               const SimplexBasis* basis_hint,
                               [[maybe_unused]] const std::vector<BoundChangeInfo>* bound_changes) {
  // bound_changes: when provided, describes which bounds changed since the
  // last solve. Reserved for future incremental warm-start optimisation
  // (e.g., skip O(n) at_upper scan, incremental FTRAN via delta_b).
  SimplexOptions opt = input_opt;
  bool local_time_limit_hit = false;
  if (opt.time_limit_hit == nullptr) opt.time_limit_hit = &local_time_limit_hit;
  SimplexResult out;
  out.result.stats.solver_name = "NativeSimplex";
  const auto solve_wall_t0 = std::chrono::steady_clock::now();
  struct TimeLimitStatusGuard {
    SimplexResult& out;
    const SimplexOptions& opt;
    const std::chrono::steady_clock::time_point& start;
    ~TimeLimitStatusGuard() {
      const bool hit = opt.time_limit_hit != nullptr && *opt.time_limit_hit;
      if (!out.result.stats.success &&
          (hit || simplex_wall_time_limit_hit(opt, start))) {
        mark_simplex_time_limit(out.result.stats, start);
      }
    }
  } time_limit_guard{out, opt, solve_wall_t0};

  const int m = sf.A.rows();
  const int n = sf.A.cols();
  thread_local int sf_call_count = 0;
  ++sf_call_count;
  const bool dbg = false;
  (void)sf_call_count;     // Suppress unused warning.
  const bool exact_edge_mode =
      opt.exact_dse_initialization || simplex_exact_edge_env_enabled();
  auto t0 = std::chrono::high_resolution_clock::now();
  if (dbg) {
    MIPSOLVERS_LOG_DEBUG("[sf #{}] m={} n={} hint={}", sf_call_count, m, n, basis_hint ? "yes" : "no");
  }
  if (m == 0) {
    out.x_std = Eigen::VectorXd::Zero(n);
    out.result.x = extract_solution(sf, out.x_std);
    out.result.stats.success = true;
    out.result.stats.objective = sf.objective_const;
    out.result.stats.status = "Optimal";
    out.exact_optimal = true;
    return out;
  }

  if (opt.allow_vendored_highs_sf_backend || vendored_highs_sf_enabled()) {
    if (solve_standard_form_with_vendored_highs(sf, opt, basis_hint, out)) {
      return out;
    }
    if (opt.require_vendored_highs_sf_backend) {
      out.result.stats.success = false;
      if (out.result.stats.status.empty()) {
        out.result.stats.status = "VendoredHiGHS SF rejected";
      }
      return out;
    }
  }

  int actual_iters = 0;  // Track real iterations from PATH A/B reopt.
  std::vector<int> basis(static_cast<size_t>(m), -1);
  std::vector<char> can_enter(static_cast<size_t>(n), 1);
  std::vector<char> at_upper(static_cast<size_t>(n), 0);
  for (int i = 0; i < m; ++i) {
    if (sf.row_to_slack_col[i] >= 0) {
      basis[i] = sf.row_to_slack_col[i];
    } else if (sf.row_to_artificial_col[i] >= 0) {
      basis[i] = sf.row_to_artificial_col[i];
    }
  }

  Eigen::MatrixXd binv;
  Eigen::VectorXd x_b;
  Eigen::VectorXd reduced_costs;
  double obj = 0.0;
  const bool has_ub = sf.var_ub.size() == n;
  std::shared_ptr<SparseBasis> solved_sparse_basis;  // Persisted for child nodes
  std::shared_ptr<SparseBasis> append_parent_sparse_basis;
  int append_parent_old_rows = 0;
  int append_parent_old_m_ineq = 0;
  int append_parent_new_rows = 0;

  // Try to extend basis hint when root cuts added extra rows.
  std::shared_ptr<SimplexBasis> extended_hint;
    if (basis_hint != nullptr && basis_hint->rows < m && basis_hint->rows > 0 &&
      static_cast<int>(basis_hint->index_count()) == basis_hint->rows) {
      const auto& hint_indices = basis_hint->basis_indices();
    // Extend: copy old basis indices, add slack columns for new rows.
    // SF row layout: [ineq | eq]. Adding cut rows (ineq) inserts them between
    // old ineq and old eq, so old eq rows shift down.
    const int old_rows = basis_hint->rows;
    const int old_cols = basis_hint->cols;
    const int n_new_rows = m - old_rows;
    const int n_orig = sf.n_original;

    // Determine old SF column counts using saved values if available.
    const int n_slack_old = (basis_hint->sf_n_slack >= 0)
        ? basis_hint->sf_n_slack
        : old_cols - n_orig - sf.n_surplus - sf.n_artificial;
    const int n_surplus_old = (basis_hint->sf_n_surplus >= 0)
        ? basis_hint->sf_n_surplus : sf.n_surplus;
    const int delta_slack = sf.n_slack - n_slack_old;
    const int delta_surplus = sf.n_surplus - n_surplus_old;

    // Determine the boundary between old ineq and eq rows.
    // old_m_eq = old_rows - old_m_ineq. We know m_eq from the SF:
    // m_eq = number of rows with sense 'E'. Since original eq rows don't change,
    // m_eq_new == m_eq_old. Count eq rows from the new SF.
    int m_eq_new = 0;
    for (int i = 0; i < m; ++i) {
      if (sf.row_to_slack_col[i] < 0 && sf.row_to_surplus_col[i] < 0 &&
          sf.row_to_artificial_col[i] >= 0) {
        ++m_eq_new;
      }
    }
    const int old_m_ineq = old_rows - m_eq_new;

    auto adjust_col = [&](int idx) -> int {
      if (idx < n_orig + n_slack_old) return idx;
      if (idx < n_orig + n_slack_old + n_surplus_old)
        return idx + delta_slack;
      return idx + delta_slack + delta_surplus;
    };

    if (opt.allow_incremental_row_append_factor &&
        basis_hint->cached_sparse_basis && delta_slack == n_new_rows &&
        delta_surplus == 0 &&
        basis_hint->cached_sparse_basis->kind() == BasisOpsKind::NativeSparse) {
      append_parent_sparse_basis =
          std::dynamic_pointer_cast<SparseBasis>(
              basis_hint->cached_sparse_basis);
      append_parent_old_rows = old_rows;
      append_parent_old_m_ineq = old_m_ineq;
      append_parent_new_rows = n_new_rows;
    }

    extended_hint = std::make_shared<SimplexBasis>(*basis_hint);
    // Materialize shared basis storage before resizing.  The appended-row
    // refactor oracle must start from the same extended basis as the
    // parent-factor path; otherwise basis_indices() keeps returning the old
    // shared vector and the "refactor" comparison silently falls back to a
    // different solve path.
    extended_hint->ensure_owned_indices();
    extended_hint->rows = m;
    extended_hint->cols = n;
    extended_hint->indices.resize(static_cast<size_t>(m));
    extended_hint->at_upper.assign(static_cast<size_t>(n), 0);
    if (!basis_hint->at_upper.empty()) {
      for (int j = 0; j < old_cols &&
                      j < static_cast<int>(basis_hint->at_upper.size());
           ++j) {
        const int new_j = adjust_col(j);
        if (new_j >= 0 && new_j < n) {
          extended_hint->at_upper[static_cast<std::size_t>(new_j)] =
              basis_hint->at_upper[static_cast<std::size_t>(j)];
        }
      }
    }

    // Old inequality rows (0..old_m_ineq-1) stay at same positions.
    for (int i = 0; i < old_m_ineq; ++i) {
        extended_hint->indices[static_cast<size_t>(i)] =
          adjust_col(hint_indices[static_cast<size_t>(i)]);
    }
    // New cut rows → positions old_m_ineq..old_m_ineq+n_new_rows-1.
    for (int k = 0; k < n_new_rows; ++k) {
      const int r = old_m_ineq + k;
      const int slack = sf.row_to_slack_col[static_cast<size_t>(r)];
      const int art = sf.row_to_artificial_col[static_cast<size_t>(r)];
      extended_hint->indices[static_cast<size_t>(r)] =
          (slack >= 0) ? slack : (art >= 0) ? art : -1;
    }
    // Old equality rows shift down by n_new_rows.
    for (int i = old_m_ineq; i < old_rows; ++i) {
      const int new_row = i + n_new_rows;
        extended_hint->indices[static_cast<size_t>(new_row)] =
          adjust_col(hint_indices[static_cast<size_t>(i)]);
    }

    const bool reuse_appended_live_cache = []() {
      const char* e = std::getenv("MIPSOLVERS_XPOOL_REUSE_APPEND_LIVE_CACHE");
      return e && e[0] == '1';
    }();

    std::shared_ptr<const Eigen::VectorXd> extended_reduced_costs;
    if (reuse_appended_live_cache &&
        opt.allow_incremental_row_append_factor &&
        basis_hint->cached_reduced_costs &&
        basis_hint->cached_reduced_costs->size() == old_cols &&
        delta_slack == n_new_rows && delta_surplus == 0) {
      auto rc = std::make_shared<Eigen::VectorXd>(
          Eigen::VectorXd::Zero(n));
      for (int j = 0; j < old_cols; ++j) {
        const int new_j = adjust_col(j);
        if (new_j >= 0 && new_j < n) {
          (*rc)[new_j] = (*(basis_hint->cached_reduced_costs))[j];
        }
      }
      // New row slacks are basic with zero cost, so nonbasic reduced costs of
      // the old columns are unchanged in the block-triangular extension.
      extended_reduced_costs = rc;
    }

    std::shared_ptr<const Eigen::VectorXd> extended_x_basic;
    std::shared_ptr<const Eigen::VectorXd> extended_x_std;
    double extended_max_objective = 0.0;
    bool extended_has_max_objective = false;
    if (reuse_appended_live_cache &&
        opt.allow_incremental_row_append_factor &&
        basis_hint->cached_x_basic &&
        basis_hint->cached_x_basic->size() == old_rows &&
        basis_hint->cached_x_std &&
        basis_hint->cached_x_std->size() == old_cols &&
        basis_hint->has_cached_max_objective &&
        delta_slack == n_new_rows && delta_surplus == 0) {
      auto xb = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(m));
      auto xs = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(n));
      bool live_state_valid = true;
      const Eigen::VectorXd& old_xb = *basis_hint->cached_x_basic;
      const Eigen::VectorXd& old_xs = *basis_hint->cached_x_std;
      for (int i = 0; i < old_m_ineq; ++i) {
        (*xb)[i] = old_xb[i];
      }
      for (int i = old_m_ineq; i < old_rows; ++i) {
        (*xb)[i + n_new_rows] = old_xb[i];
      }
      for (int j = 0; j < old_cols; ++j) {
        const int new_j = adjust_col(j);
        if (new_j >= 0 && new_j < n) {
          (*xs)[new_j] = old_xs[j];
        }
      }
      for (int k = 0; k < n_new_rows; ++k) {
        const int r = old_m_ineq + k;
        const int slack = sf.row_to_slack_col[static_cast<std::size_t>(r)];
        if (slack < 0 || slack >= n) {
          live_state_valid = false;
          break;
        }
        double residual = sf.b[r];
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                 sf.A_row, r);
             it; ++it) {
          const int col = static_cast<int>(it.col());
          if (col == slack) continue;
          residual -= it.value() * (*xs)[col];
        }
        (*xs)[slack] = residual;
        (*xb)[r] = residual;
      }
      if (live_state_valid) {
        extended_x_basic = xb;
        extended_x_std = xs;
        extended_max_objective = basis_hint->cached_max_objective;
        extended_has_max_objective = true;
      }
    }

    // Invalidate cached inverse/basis (wrong dimension).
    extended_hint->cached_inverse.reset();
    extended_hint->cached_reduced_costs = std::move(extended_reduced_costs);
    extended_hint->cached_x_basic = std::move(extended_x_basic);
    extended_hint->cached_x_std = std::move(extended_x_std);
    extended_hint->cached_max_objective = extended_max_objective;
    extended_hint->has_cached_max_objective = extended_has_max_objective;
    extended_hint->cached_sparse_basis.reset();
    basis_hint = extended_hint.get();
    if (dbg) MIPSOLVERS_LOG_DEBUG("  extended hint {}→{} rows", old_rows, m);
  }

  // BASIS PROJECTION: hint was built for a larger LP (e.g., root_lp+cuts),
  // but current SF is for base_lp (no cuts). Project out cut rows.
  // Cut slacks can only appear in their own cut rows, so base rows are clean.
  // This enables warm-start for pump/prog/dive LPs that use base_lp.
  if (!extended_hint && basis_hint != nullptr &&
      basis_hint->rows > m && m > 0 &&
      basis_hint->sf_n_slack >= 0 &&
      static_cast<int>(basis_hint->index_count()) == basis_hint->rows) {
    const int old_rows = basis_hint->rows;
    const auto& hint_indices = basis_hint->basis_indices();
    const int n_orig = sf.n_original;
    const int n_slack_new = sf.n_slack;
    const int n_slack_old = basis_hint->sf_n_slack;
    const int delta_slack = n_slack_old - n_slack_new;  // = k_cuts (positive)
    const int n_surplus_old = (basis_hint->sf_n_surplus >= 0) ? basis_hint->sf_n_surplus : sf.n_surplus;
    const int delta_surplus = n_surplus_old - sf.n_surplus;

    if (delta_slack > 0 && delta_slack == basis_hint->rows - m &&
        delta_surplus == 0) {
      // Count eq rows in current (smaller) SF.
      int m_eq_new = 0;
      for (int i = 0; i < m; ++i) {
        if (sf.row_to_slack_col[i] < 0 && sf.row_to_surplus_col[i] < 0 &&
            sf.row_to_artificial_col[i] >= 0) {
          ++m_eq_new;
        }
      }
      const int m_ineq_new = m - m_eq_new;
      const int m_ineq_old = m_ineq_new + delta_slack;  // includes cut rows

      // Column adjustment: project from larger SF to smaller SF.
      // Cut slack columns [n_orig+n_slack_new, n_orig+n_slack_old) are absent.
      auto project_col = [&](int col) -> int {
        if (col < 0) return col;
        if (col < n_orig + n_slack_new) return col;           // original or base slack
        if (col < n_orig + n_slack_old) return -(col + 10);   // cut slack (unexpected in base row)
        return col - delta_slack;                              // surplus/artificial: shift down
      };

      auto proj_hint = std::make_shared<SimplexBasis>();
      proj_hint->rows = m;
      proj_hint->cols = n;
      proj_hint->indices.resize(static_cast<size_t>(m), -1);
      proj_hint->at_upper.resize(static_cast<size_t>(n), 0);
      // Copy at_upper for original and base-slack columns (same positions).
      if (!basis_hint->at_upper.empty()) {
        const int copy_n = std::min(n_orig + n_slack_new,
                                    std::min(n, static_cast<int>(basis_hint->at_upper.size())));
        for (int j = 0; j < copy_n; ++j)
          proj_hint->at_upper[static_cast<size_t>(j)] = basis_hint->at_upper[static_cast<size_t>(j)];
        // Surplus/art columns: shift from old positions.
        const int surplus_offset_new = n_orig + n_slack_new;
        const int surplus_offset_old = n_orig + n_slack_old;
        const int n_sa = n - surplus_offset_new;
        for (int j = 0; j < n_sa; ++j) {
          const int old_j = surplus_offset_old + j;
          if (old_j < static_cast<int>(basis_hint->at_upper.size()))
            proj_hint->at_upper[static_cast<size_t>(surplus_offset_new + j)] =
                basis_hint->at_upper[static_cast<size_t>(old_j)];
        }
      }

      bool valid = true;
      // Base inequality rows 0..m_ineq_new-1 (same position in both).
      for (int i = 0; i < m_ineq_new && valid; ++i) {
        const int col = project_col(hint_indices[static_cast<size_t>(i)]);
        if (col < 0 || col >= n) {
          if (col < 0) {
            // cut slack in base row — substitute row's own slack/art
            const int s = sf.row_to_slack_col[i];
            const int a = sf.row_to_artificial_col[i];
            proj_hint->indices[static_cast<size_t>(i)] = (s >= 0) ? s : (a >= 0) ? a : -1;
            if (proj_hint->indices[static_cast<size_t>(i)] < 0) valid = false;
          } else { valid = false; }
        } else {
          proj_hint->indices[static_cast<size_t>(i)] = col;
        }
      }
      // Equality rows: in hint at m_ineq_old..m_ineq_old+m_eq_new-1,
      // in base SF at m_ineq_new..m_ineq_new+m_eq_new-1.
      for (int i = 0; i < m_eq_new && valid; ++i) {
        const int hint_row = m_ineq_old + i;
        const int base_row = m_ineq_new + i;
        if (hint_row >= old_rows) { valid = false; break; }
        const int col = project_col(hint_indices[static_cast<size_t>(hint_row)]);
        if (col < 0 || col >= n) {
          if (col < 0) {
            const int s = sf.row_to_slack_col[base_row];
            const int a = sf.row_to_artificial_col[base_row];
            proj_hint->indices[static_cast<size_t>(base_row)] = (s >= 0) ? s : (a >= 0) ? a : -1;
            if (proj_hint->indices[static_cast<size_t>(base_row)] < 0) valid = false;
          } else { valid = false; }
        } else {
          proj_hint->indices[static_cast<size_t>(base_row)] = col;
        }
      }

      if (valid) {
        proj_hint->cached_inverse.reset();
        proj_hint->cached_reduced_costs.reset();
        proj_hint->cached_sparse_basis.reset();
        proj_hint->sf_n_slack = n_slack_new;
        proj_hint->sf_n_surplus = sf.n_surplus;
        proj_hint->sf_n_artificial = sf.n_artificial;
        extended_hint = proj_hint;
        basis_hint = extended_hint.get();
        if (dbg) MIPSOLVERS_LOG_DEBUG("  projected hint {}→{} rows", old_rows, m);
      }
    }
  }

  bool hint_match = false;
  if (basis_hint != nullptr && basis_hint->rows == m && basis_hint->cols == n &&
      static_cast<int>(basis_hint->index_count()) == m) {
    const auto& hint_indices = basis_hint->basis_indices();
    // Validate all basis indices are in [0, n).
    bool indices_valid = true;
    for (int i = 0; i < m; ++i) {
      if (hint_indices[static_cast<size_t>(i)] < 0 ||
          hint_indices[static_cast<size_t>(i)] >= n) {
        indices_valid = false;
        break;
      }
    }
    if (indices_valid) {
      basis = hint_indices;
      if (!basis_hint->at_upper.empty()) at_upper = basis_hint->at_upper;
      hint_match = true;
      if (dbg) MIPSOLVERS_LOG_DEBUG("  hint match rows={} cols={}", m, n);
    } else {
      if (dbg) MIPSOLVERS_LOG_DEBUG("[solve_lp_from_sf] hint indices OUT OF RANGE, ignoring");
    }
  } else if (basis_hint != nullptr) {
    if (dbg) MIPSOLVERS_LOG_DEBUG("[solve_lp_from_sf] hint MISMATCH: hint.rows={} hint.cols={} sf.m={} sf.n={} hint.sz={}",
            basis_hint->rows, basis_hint->cols, m, n,
            static_cast<int>(basis_hint->index_count()));
  }
  trace_lp_basis_hint_state(sf, basis, at_upper,
                            hint_match ? "entry_hint" : "entry_default",
                            sf_call_count, hint_match);

  // ══════════════════════════════════════════════════════════════════════
  // PATH A: Sparse dual simplex warm-start for large problems (m > 200).
  // When branching changes bounds, dual feasibility is preserved — only
  // primal feasibility breaks.  Sparse dual simplex restores optimality
  // in very few iterations without needing a dense basis inverse.
  // ══════════════════════════════════════════════════════════════════════
  if (hint_match && m > kSparseSimplexThreshold) {
    auto t_warm0 = std::chrono::high_resolution_clock::now();
    // Per-LP timing accumulator (thread-local for dive/tree stats).
    thread_local struct {
      int64_t refact_us{0}, btran_rc_us{0}, at_upper_us{0}, ftran_xb_us{0};
      int64_t feas_check_us{0}, dual_reopt_us{0};
      int count{0}, iters{0}, zero_iter{0};
      void dump(int m_val, bool verbose = false) {
        if (verbose && count > 0 && count % 200 == 0) {
          fprintf(stderr, "[LP-TIMING] n=%d m=%d: refact=%.1f btran_rc=%.1f at_up=%.1f "
                  "ftran_xb=%.1f feas=%.1f dual=%.1f total=%.1f ms/LP  "
                  "avg_iter=%.1f zero_iter=%d/%d\n",
                  count, m_val,
                  refact_us * 0.001 / count, btran_rc_us * 0.001 / count,
                  at_upper_us * 0.001 / count, ftran_xb_us * 0.001 / count,
                  feas_check_us * 0.001 / count, dual_reopt_us * 0.001 / count,
                  (refact_us + btran_rc_us + at_upper_us + ftran_xb_us +
                   feas_check_us + dual_reopt_us) * 0.001 / count,
                  static_cast<double>(iters) / count, zero_iter, count);
        }
      }
    } lp_timing;
    auto tp = [](){ return std::chrono::high_resolution_clock::now(); };
    auto us = [](auto a, auto b){ return std::chrono::duration_cast<std::chrono::microseconds>(b-a).count(); };
    auto t_phase = tp();
    g_slp_counters.path_a_entered.fetch_add(1, std::memory_order_relaxed);
    // Validate all basis column indices.
    bool basis_valid = true;
    for (int i = 0; i < m; ++i) {
      if (basis[i] < 0 || basis[i] >= n) { basis_valid = false; break; }
    }
    if (basis_valid) {
      std::vector<char> sp_can_enter(static_cast<size_t>(n), 1);
      for (int i = 0; i < m; ++i) {
        const int art = sf.row_to_artificial_col[i];
        if (art >= 0) sp_can_enter[static_cast<size_t>(art)] = 0;
      }

      // ═══════════════════════════════════════════════════════════════════
      // PRE-FIX: Replace basic artificial variables with slack columns.
      // Degenerate artificials at zero can remain in the warm-started basis.
      // When bounds change, their value can become nonzero, making the LP
      // appear infeasible when it isn't. Replace them with the row's slack.
      // ═══════════════════════════════════════════════════════════════════
      int art_swaps = 0;
      if (sf.n_artificial > 0) {
        // Build set of basic columns for conflict detection.
        std::vector<char> in_basis(static_cast<size_t>(n), 0);
        for (int i = 0; i < m; ++i) {
          if (basis[i] >= 0 && basis[i] < n)
            in_basis[static_cast<size_t>(basis[i])] = 1;
        }
        // Reuse sp_can_enter: artificial columns have sp_can_enter[col] == 0.
        for (int i = 0; i < m; ++i) {
          const int bvar = basis[i];
          if (bvar < 0 || bvar >= n || sp_can_enter[static_cast<size_t>(bvar)]) continue;
          // Try to replace with this row's slack column.
          const int slack = sf.row_to_slack_col[i];
          if (slack >= 0 && !in_basis[static_cast<size_t>(slack)]) {
            in_basis[static_cast<size_t>(bvar)] = 0;
            basis[i] = slack;
            in_basis[static_cast<size_t>(slack)] = 1;
            at_upper[static_cast<size_t>(slack)] = 0;
            ++art_swaps;
          }
        }
        if (dbg && art_swaps > 0) {
          MIPSOLVERS_LOG_DEBUG("  basis cleanup: swapped {} artificials for slacks", art_swaps);
        }
      }

      // ═══════════════════════════════════════════════════════════════════
      // Try to reuse the parent factor only in the explicit appended-row
      // case.  Generic child nodes still rebuild the numeric factorization:
      // branching changes RHS/bounds, while appended <= rows with new
      // slack-basic columns form a block-triangular extension of the parent
      // basis and are safe to solve by parent FTRAN/BTRAN.
      // ═══════════════════════════════════════════════════════════════════
      bool can_reuse_parent_rc =
          art_swaps == 0 && basis_hint->cached_reduced_costs &&
          basis_hint->cached_reduced_costs->size() == n &&
          !append_parent_sparse_basis;

        std::shared_ptr<SparseBasis> sbasis_shared = std::make_shared<SparseBasis>(
          sf.A, SparseBasis::from_public_backend(opt.factor_backend));
      SparseBasis& sbasis = *sbasis_shared;

      // ── P10: inherit DSE weights from parent SparseBasis ──
      // Generic same-size warm starts may inherit DSE weights.  Appended-row
      // solves only inherit them after the parent-factor extension is accepted;
      // otherwise a rejected fast path can still perturb degenerate pricing and
      // break exact root-xpool conformance against a fresh refactor.
      if (!append_parent_sparse_basis && basis_hint != nullptr &&
          basis_hint->cached_sparse_basis &&
          basis_hint->cached_sparse_basis->kind() == BasisOpsKind::NativeSparse) {
        auto parent_sb = std::dynamic_pointer_cast<SparseBasis>(
            basis_hint->cached_sparse_basis);
        if (parent_sb) sbasis.copy_dse_weights_from(*parent_sb);
      }

      bool factorize_ok = false;
      bool used_appended_parent_factor = false;
      if (append_parent_sparse_basis && art_swaps == 0) {
        static const bool parent_factor_check = []() {
          const char* e = std::getenv("MIPSOLVERS_XPOOL_PARENT_FACTOR_CHECK");
          return e && e[0] == '1';
        }();
        used_appended_parent_factor = sbasis.init_appended_slack_extension(
            append_parent_sparse_basis, basis, append_parent_old_rows,
            append_parent_old_m_ineq, append_parent_new_rows);
        factorize_ok = used_appended_parent_factor;
        if (used_appended_parent_factor) {
          // Do not inherit DSE weights on the first parent-factor gate.  The
          // block solve itself is bit-close to a fresh refactor, but inherited
          // DSE can change degenerate leaving-row tie breaks and produce a
          // different integer-like frontier.  Re-enable this only after a
          // first-class edge-weight remap passes append-vs-refactor
          // conformance.
          //
          // Likewise, never reuse the parent's reduced costs in the appended
          // LP.  Even when the block BTRAN is algebraically equivalent to a
          // refactorization, the parent RC vector encodes the old nonbasic
          // side on a degenerate face.  Recompute RC from the current appended
          // basis so at_upper correction and pivot eligibility are first-class
          // state of this LP, not inherited frontier metadata.
          can_reuse_parent_rc = false;
          if (exact_edge_mode) {
            const std::vector<double> exact_dse =
                sbasis.compute_exact_dse_weights();
            sbasis.save_dse_weights(exact_dse, basis);
            if (parent_factor_check) {
              fmt::print(stderr,
                         "[B&C ROOT-XPOOL-PARENT-DSE] m={} old={} new={} "
                         "mode=exact_btran\n",
                         m, append_parent_old_rows, append_parent_new_rows);
            }
          }
        }
        if (parent_factor_check) {
          fmt::print(stderr,
                     "[B&C ROOT-XPOOL-PARENT-FACTOR-TRY] m={} old={} "
                     "new={} used={} art_swaps={}\n",
                     m, append_parent_old_rows, append_parent_new_rows,
                     used_appended_parent_factor ? 1 : 0, art_swaps);
        }
        if (used_appended_parent_factor && parent_factor_check) {
          SparseBasis fresh_check(
              sf.A, SparseBasis::from_public_backend(opt.factor_backend));
          const bool fresh_ok = fresh_check.refactorize(basis);
          if (fresh_ok) {
            Eigen::VectorXd b_adj = sf.b;
            for (int j = 0; j < n; ++j) {
              if (!at_upper[static_cast<std::size_t>(j)]) continue;
              if (!std::isfinite(sf.var_ub[j])) continue;
              const double uj = sf.var_ub[j];
              for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j);
                   it; ++it) {
                b_adj[it.row()] -= it.value() * uj;
              }
            }
            Eigen::VectorXd c_b(m);
            for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
            const Eigen::VectorXd fast_f = sbasis.ftran(b_adj);
            const Eigen::VectorXd ref_f = fresh_check.ftran(b_adj);
            const Eigen::VectorXd fast_b = sbasis.btran(c_b);
            const Eigen::VectorXd ref_b = fresh_check.btran(c_b);
            const double f_inf =
                (fast_f.size() == ref_f.size())
                    ? (fast_f - ref_f).lpNorm<Eigen::Infinity>()
                    : std::numeric_limits<double>::infinity();
            const double b_inf =
                (fast_b.size() == ref_b.size())
                    ? (fast_b - ref_b).lpNorm<Eigen::Infinity>()
                    : std::numeric_limits<double>::infinity();
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-PARENT-FACTOR-CHECK] m={} old={} "
                       "new={} ftran_inf={:.3e} btran_inf={:.3e}\n",
                       m, append_parent_old_rows, append_parent_new_rows,
                       f_inf, b_inf);
          } else {
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-PARENT-FACTOR-CHECK] m={} "
                       "fresh_refactor_failed\n",
                       m);
          }
        }
      }
      if (!factorize_ok) {
        factorize_ok = sbasis.refactorize(basis);
      }
      { auto t_now = tp(); lp_timing.refact_us += us(t_phase, t_now); t_phase = t_now; }
      if (!factorize_ok) {
        // Crash basis is singular. Fall back to crash_artificial_basis which
        // guarantees non-singularity, but keep the IPM-scored at_upper
        // assignments that help dual simplex start near-optimal.
        for (int i = 0; i < m; ++i) {
          if (sf.row_to_slack_col[i] >= 0) {
            basis[i] = sf.row_to_slack_col[i];
          } else if (sf.row_to_artificial_col[i] >= 0) {
            basis[i] = sf.row_to_artificial_col[i];
          }
        }
        crash_artificial_basis(sf, basis);
        factorize_ok = sbasis.refactorize(basis);
        used_appended_parent_factor = false;
        if (factorize_ok) {
          MIPSOLVERS_LOG_DEBUG("[PATH A] singular crash repaired via crash_artificial_basis m={}", m);
        } else {
          MIPSOLVERS_LOG_DEBUG("[PATH A] refactorize FAILED m={} — crash_artificial_basis also singular", m);
        }
      }

      if (factorize_ok) {
        if (dbg && used_appended_parent_factor) {
          MIPSOLVERS_LOG_DEBUG("[PATH A] appended-row parent factor m={} old={} new={}",
                            m, append_parent_old_rows,
                            append_parent_new_rows);
        }
        // ═══════════════════════════════════════════════════════════════════
        // MERGED at_upper correction + state recompute.
        // Reduced costs don't depend on at_upper, so compute once and
        // use them for both: (1) correcting at_upper, (2) final state.
        // ═══════════════════════════════════════════════════════════════════
        std::vector<char> is_basic_sp(static_cast<size_t>(n), 0);
        for (int idx : basis) if (idx >= 0 && idx < n) is_basic_sp[static_cast<size_t>(idx)] = 1;

        // Step 1: Reduced costs — reuse from parent if basis unchanged.
        if (can_reuse_parent_rc) {
          reduced_costs = *basis_hint->cached_reduced_costs;
          if (dbg) MIPSOLVERS_LOG_DEBUG("  REUSED reduced costs from parent");
        } else {
          Eigen::VectorXd c_b(m);
          for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
          Eigen::VectorXd y = sbasis.btran(c_b);
          reduced_costs = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y);
        }
        { auto t_now = tp(); lp_timing.btran_rc_us += us(t_phase, t_now); t_phase = t_now; }

        static const bool parent_state_diff_diag = []() {
          const char* e = std::getenv("MIPSOLVERS_XPOOL_PARENT_STATE_DIFF");
          return e && e[0] == '1';
        }();
        Eigen::VectorXd parent_diag_rc_before_side;
        if (used_appended_parent_factor && parent_state_diff_diag) {
          parent_diag_rc_before_side = reduced_costs;
        }

        // Step 2: Fix at_upper based on reduced costs. Build compact list.
        // For variables with clear reduced cost direction, set at_upper
        // accordingly. For degenerate nonbasics (|rc| <= tol), keep the
        // parent's assignment to minimize unnecessary x_b perturbation.
        std::vector<int> at_upper_nb;  // nonbasic variables at upper bound
        bool at_upper_changed_from_hint = false;
        int at_upper_flips = 0;
        {
          at_upper_nb.reserve(static_cast<size_t>(n / 4));
          at_upper_flips = normalize_nonbasic_side_from_reduced_costs(
              sf, basis, reduced_costs, at_upper, opt.optimality_tol);
          for (int j = 0; j < n; ++j) {
            if (at_upper[static_cast<size_t>(j)]) at_upper_nb.push_back(j);
          }
          at_upper_changed_from_hint = at_upper_flips > 0;
          if (dbg && at_upper_flips > 0) MIPSOLVERS_LOG_DEBUG("  at_upper fix: {} flips", at_upper_flips);
        }
        { auto t_now = tp(); lp_timing.at_upper_us += us(t_phase, t_now); t_phase = t_now; }

        if (used_appended_parent_factor && parent_state_diff_diag) {
          SparseBasis fresh_diag(
              sf.A, SparseBasis::from_public_backend(opt.factor_backend));
          if (fresh_diag.refactorize(basis)) {
            Eigen::VectorXd fresh_xb;
            Eigen::VectorXd fresh_rc;
            double fresh_obj = 0.0;
            sparse_full_recompute(sf, basis, at_upper, is_basic_sp,
                                  fresh_diag, fresh_xb, fresh_rc, fresh_obj);
            Eigen::VectorXd fast_xb;
            Eigen::VectorXd fast_rc;
            double fast_obj = 0.0;
            sparse_full_recompute(sf, basis, at_upper, is_basic_sp,
                                  sbasis, fast_xb, fast_rc, fast_obj);
            double rc_inf = 0.0;
            int rc_sign_diff = 0;
            for (int j = 0; j < n; ++j) {
              const double lhs = fast_rc[j];
              const double rhs = fresh_rc[j];
              rc_inf = std::max(rc_inf, std::abs(lhs - rhs));
              const int lhs_sign =
                  (lhs > opt.optimality_tol)
                      ? 1
                      : (lhs < -opt.optimality_tol) ? -1 : 0;
              const int rhs_sign =
                  (rhs > opt.optimality_tol)
                      ? 1
                      : (rhs < -opt.optimality_tol) ? -1 : 0;
              if (lhs_sign != rhs_sign) ++rc_sign_diff;
            }
            double xb_inf = 0.0;
            for (int i = 0; i < m; ++i) {
              xb_inf = std::max(xb_inf, std::abs(fast_xb[i] - fresh_xb[i]));
            }
            double rc_before_inf = 0.0;
            if (parent_diag_rc_before_side.size() == fast_rc.size()) {
              rc_before_inf =
                  (parent_diag_rc_before_side - fast_rc)
                      .lpNorm<Eigen::Infinity>();
            }
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-PARENT-STATE-DIFF-PRE] m={} old={} "
                       "new={} flips={} rcInf={:.3e} rcSign={} "
                       "xbInf={:.3e} objDiff={:.3e} rcBeforeInf={:.3e}\n",
                       m, append_parent_old_rows, append_parent_new_rows,
                       at_upper_flips, rc_inf, rc_sign_diff, xb_inf,
                       fast_obj - fresh_obj, rc_before_inf);
          } else {
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-PARENT-STATE-DIFF-PRE] m={} "
                       "fresh_refactor_failed\n",
                       m);
          }
        }

        // Step 3: FTRAN for x_b using compact at-upper list.
        const bool can_reuse_live_appended_state =
            false &&
            used_appended_parent_factor &&
            !at_upper_changed_from_hint &&
            basis_hint->cached_x_basic &&
            basis_hint->cached_x_basic->size() == m &&
            basis_hint->cached_x_std &&
            basis_hint->cached_x_std->size() == n &&
            basis_hint->has_cached_max_objective;
        if (can_reuse_live_appended_state) {
          x_b = *basis_hint->cached_x_basic;
          obj = basis_hint->cached_max_objective;
          if (dbg) MIPSOLVERS_LOG_DEBUG("  REUSED appended live x_basic/objective");
        } else {
          Eigen::VectorXd b_adj = sf.b;
          for (int j : at_upper_nb) {
            const double uj = sf.var_ub[j];
            for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
              b_adj[it.row()] -= it.value() * uj;
            }
          }
          x_b = sbasis.ftran(b_adj);
        }
        { auto t_now = tp(); lp_timing.ftran_xb_us += us(t_phase, t_now); t_phase = t_now; }

        // Step 4: Objective using compact at-upper list.
        if (!can_reuse_live_appended_state) {
          Eigen::VectorXd c_b(m);
          for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
          obj = c_b.dot(x_b);
          for (int j : at_upper_nb) {
            obj += sf.c_max[j] * sf.var_ub[j];
          }
        }

        // Debug: check how many variables violate dual feasibility.
        if (dbg) {
          [[maybe_unused]] int n_dual_viol = 0;
          for (int j = 0; j < n; ++j) {
            if (is_basic_sp[static_cast<size_t>(j)] || !sp_can_enter[static_cast<size_t>(j)]) continue;
            if (at_upper[static_cast<size_t>(j)] && reduced_costs[j] < -opt.optimality_tol) ++n_dual_viol;
            if (!at_upper[static_cast<size_t>(j)] && reduced_costs[j] > opt.optimality_tol) ++n_dual_viol;
          }
          MIPSOLVERS_LOG_DEBUG("  sparse warm: {} dual violations", n_dual_viol);
        }

        // Check primal feasibility.
        bool primal_feas = approx_nonnegative(x_b, opt.feasibility_tol);
        if (primal_feas && has_ub) {
          for (int i = 0; i < m; ++i) {
            const int bvar = basis[i];
            const double ub_i = sf.var_ub[bvar];
            if (std::isfinite(ub_i) && x_b[i] > ub_i + opt.feasibility_tol) {
              primal_feas = false;
              break;
            }
          }
        }
        { auto t_now = tp(); lp_timing.feas_check_us += us(t_phase, t_now); t_phase = t_now; }
        if (primal_feas) {
          // Already optimal — no iterations needed.
          // But verify dual feasibility too.
          bool dual_ok = approx_dual_feasible(reduced_costs, basis, sp_can_enter, at_upper, opt.optimality_tol);
          if (dual_ok) {
            out.solved_from_hint = true;
            out.dual_reoptimized = true;
            out.exact_optimal = true;  // Enable GMI cut generation
            if (dbg) MIPSOLVERS_LOG_DEBUG("[PATH A] primal+dual feasible (optimal) m={}", m);
            ++lp_timing.zero_iter;
          } else {
            if (dbg) MIPSOLVERS_LOG_DEBUG("[PATH A] primal feasible, dual infeasible m={}", m);
            if (used_appended_parent_factor) {
              // The parent-factor block solve is first-class for constructing
              // the appended LP state and for zero-pivot optimality checks.
              // Once a pivot is needed, the sparse pricing/update path must
              // match the append-refactor oracle exactly; otherwise
              // degenerate tie order can select a different frontier.  Until
              // the appended DSE/edge-weight update is a full conformance
              // object, pivot from a freshly refactorized basis.
              used_appended_parent_factor = false;
              can_reuse_parent_rc = false;
              if (!sbasis.refactorize(basis)) {
                factorize_ok = false;
                MIPSOLVERS_LOG_DEBUG("[PATH A] appended parent-factor primal fallback failed m={}", m);
              } else {
                sparse_full_recompute(sf, basis, at_upper, is_basic_sp,
                                      sbasis, x_b, reduced_costs, obj);
              }
            }
            if (!factorize_ok) {
              MIPSOLVERS_LOG_DEBUG("[PATH A] parent-factor primal fallback unavailable m={}", m);
            } else {
            // Primal feasible but dual infeasible: need primal simplex Phase II.
            bool p2_ok = sparse_primal_simplex_optimize(sf, basis, sp_can_enter, opt,
                                                        sbasis, x_b, reduced_costs, obj, at_upper);
            if (p2_ok) {
              out.solved_from_hint = true;
              out.dual_reoptimized = true;
              out.exact_optimal = true;  // Enable GMI cut generation
              if (dbg) MIPSOLVERS_LOG_DEBUG("  sparse warm: primal feasible, Phase II OK");
            } else {
              if (dbg) MIPSOLVERS_LOG_DEBUG("  sparse warm: primal feasible, Phase II FAIL");
            }
            }
          }
        } else {
          if (used_appended_parent_factor) {
            // Parent-factor FTRAN/BTRAN is valid for state construction, but
            // dual reoptimization is highly degenerate in root cut-pool LPs.
            // Switch to the same freshly refactorized basis used by the
            // append-refactor oracle before pivoting so frontier/status
            // conformance is preserved.
            used_appended_parent_factor = false;
            can_reuse_parent_rc = false;
            if (!sbasis.refactorize(basis)) {
              factorize_ok = false;
              MIPSOLVERS_LOG_DEBUG("[PATH A] appended parent-factor refactor fallback failed m={}", m);
            } else {
              sparse_full_recompute(sf, basis, at_upper, is_basic_sp,
                                    sbasis, x_b, reduced_costs, obj);
            }
          }
          if (!factorize_ok) {
            MIPSOLVERS_LOG_DEBUG("[PATH A] parent-factor fallback unavailable m={}", m);
          } else {
          // Primal infeasible: run dual simplex to restore primal feasibility.
          [[maybe_unused]] int n_primal_viol = 0;
          for (int i = 0; i < m; ++i) {
            if (x_b[i] < -opt.feasibility_tol) ++n_primal_viol;
            else if (has_ub && std::isfinite(sf.var_ub[basis[i]]) &&
                     x_b[i] > sf.var_ub[basis[i]] + opt.feasibility_tol) ++n_primal_viol;
          }
          MIPSOLVERS_LOG_DEBUG("[PATH A] primal infeasible: {} violations, trying dual reopt m={} max_iter={}",
                           n_primal_viol, m, opt.allow_cold_start ? std::max(2000, m) : std::max(500, m/2));
          bool proved_infeasible = false;
          bool cutoff_reached = false;
          // Cap reopt iterations. For tree nodes (near-vertex start),
          // convergence should be fast — spending many iterations on a
          // failure wastes time. For crossover (far-from-vertex start),
          // the caller can set a higher max_iter. Use opt.allow_cold_start
          // as a proxy: crossover allows cold fallback and needs more room.
          SimplexOptions reopt_opt = opt;
          if (opt.allow_cold_start) {
            // Crossover or root-level: allow up to m iterations.
            reopt_opt.max_iter = std::min(opt.max_iter, std::max(2000, m));
          } else {
            // Tree node: cap aggressively to fail fast.
            reopt_opt.max_iter = std::min(opt.max_iter, std::max(500, m / 2));
          }
          bool dr_ok = sparse_dual_simplex_reoptimize(sf, basis, sp_can_enter, reopt_opt,
                                                      sbasis, x_b, reduced_costs, obj, at_upper,
                                                      proved_infeasible, cutoff_reached);
          { auto t_now = tp(); lp_timing.dual_reopt_us += us(t_phase, t_now); t_phase = t_now;
            lp_timing.iters += tl_dual_reopt_iters;
            actual_iters += tl_dual_reopt_iters; }
          if (dr_ok) {
            out.solved_from_hint = true;
            out.dual_reoptimized = true;
            out.exact_optimal = true;  // Enable GMI cut generation
            MIPSOLVERS_LOG_DEBUG("[PATH A] dual reopt OK m={} iters={}", m, sbasis.eta_count());
          } else if (cutoff_reached) {
            out.result.stats.success = false;
            out.result.stats.iterations = actual_iters;
            out.result.stats.objective = sf.objective_const - obj;
            out.result.stats.status = "Objective cutoff";
            return out;
          } else if (proved_infeasible) {
            // Dual simplex found a Farkas certificate — node LP is infeasible.
            mark_proven_infeasible(out.result.stats);
            binv = sbasis.compute_dense_inverse(basis);
            extract_farkas_certificate(sf, basis, binv, out.result.stats);
            MIPSOLVERS_LOG_DEBUG("[PATH A] dual reopt INFEASIBLE m={}", m);
            return out;
          } else {
            MIPSOLVERS_LOG_DEBUG("[PATH A] dual reopt FAIL m={} etas={}", m, sbasis.eta_count());
            // Retry with crash_artificial_basis before expensive cold start.
            for (int ii = 0; ii < m; ++ii) {
              if (sf.row_to_slack_col[ii] >= 0) basis[ii] = sf.row_to_slack_col[ii];
              else if (sf.row_to_artificial_col[ii] >= 0) basis[ii] = sf.row_to_artificial_col[ii];
            }
            crash_artificial_basis(sf, basis);
            if (sbasis.refactorize(basis)) {
              // Recompute state from scratch with the crash basis.
              std::vector<char> is_basic2(static_cast<size_t>(n), 0);
              for (int idx : basis) if (idx >= 0 && idx < n) is_basic2[static_cast<size_t>(idx)] = 1;
              Eigen::VectorXd c_b2(m);
              for (int ii = 0; ii < m; ++ii) c_b2[ii] = sf.c_max[basis[ii]];
              Eigen::VectorXd y2 = sbasis.btran(c_b2);
              reduced_costs = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y2);
              for (int j = 0; j < n; ++j) {
                if (is_basic2[static_cast<size_t>(j)]) { at_upper[j] = 0; continue; }
                if (!has_ub || !std::isfinite(sf.var_ub[j])) { at_upper[j] = 0; continue; }
                at_upper[j] = (reduced_costs[j] > 0) ? 1 : 0;
              }
              x_b = sbasis.ftran(sf.b);
              for (int ii = 0; ii < m; ++ii) {
                if (at_upper[basis[ii]]) x_b[ii] = sf.var_ub[basis[ii]];
              }
              obj = 0.0;
              for (int j = 0; j < n; ++j) {
                if (is_basic2[static_cast<size_t>(j)]) obj += sf.c_max[j] * x_b[0]; // recomputed below
                else if (at_upper[j]) obj += sf.c_max[j] * sf.var_ub[j];
              }
              // Full recompute via existing helper.
              sparse_full_recompute(sf, basis, at_upper, is_basic2, sbasis, x_b, reduced_costs, obj);
              bool proved_inf2 = false;
              bool cutoff_reached2 = false;
              SimplexOptions reopt_opt2 = opt;
              reopt_opt2.max_iter = std::min(opt.max_iter, std::max(2000, m));
              bool dr2 = sparse_dual_simplex_reoptimize(sf, basis, sp_can_enter, reopt_opt2,
                                                        sbasis, x_b, reduced_costs, obj, at_upper,
                                                        proved_inf2, cutoff_reached2);
              actual_iters += tl_dual_reopt_iters;
              if (dr2) {
                out.solved_from_hint = true;
                out.dual_reoptimized = true;
                out.exact_optimal = true;
                MIPSOLVERS_LOG_DEBUG("[PATH A] crash-repair reopt OK m={} iters={}", m, sbasis.eta_count());
              } else if (cutoff_reached2) {
                out.result.stats.success = false;
                out.result.stats.iterations = actual_iters;
                out.result.stats.objective = sf.objective_const - obj;
                out.result.stats.status = "Objective cutoff";
                return out;
              } else if (proved_inf2) {
                mark_proven_infeasible(out.result.stats);
                binv = sbasis.compute_dense_inverse(basis);
                extract_farkas_certificate(sf, basis, binv, out.result.stats);
                MIPSOLVERS_LOG_DEBUG("[PATH A] crash-repair reopt INFEASIBLE m={}", m);
                return out;
              } else {
                MIPSOLVERS_LOG_DEBUG("[PATH A] crash-repair reopt FAIL m={}", m);
              }
            }
          }
          }
        }
      // LP timing accounting.
      ++lp_timing.count;
      lp_timing.dump(m, opt.verbose);
      // Persist SparseBasis for child nodes.
      // SKIP the expensive refactorize-to-clean step. The child node will
      // refactorize on entry if needed. For sequential B&C, we can reuse
      // the dirty basis (with etas) since parents are not revisited.
      if (out.solved_from_hint) {
        // Persist as-is (may have etas). Child will refactorize if needed.
        solved_sparse_basis = sbasis_shared;
      }
      } else {  // factorize_ok == false
        MIPSOLVERS_LOG_DEBUG("[PATH A] refactorize FAILED m={} — singular crash basis", m);
      }
    }
    // Counter update for PATH A outcome.
    {
      auto t_warm1 = std::chrono::high_resolution_clock::now();
      auto warm_dur = std::chrono::duration_cast<std::chrono::microseconds>(t_warm1 - t_warm0).count();
      if (out.solved_from_hint) {
        g_slp_counters.path_a_ok.fetch_add(1, std::memory_order_relaxed);
        g_slp_counters.warm_us.fetch_add(warm_dur, std::memory_order_relaxed);
      } else if (out.result.stats.status == "LP infeasible") {
        g_slp_counters.path_a_infeasible.fetch_add(1, std::memory_order_relaxed);
        g_slp_counters.warm_us.fetch_add(warm_dur, std::memory_order_relaxed);
      } else {
        g_slp_counters.path_a_fail.fetch_add(1, std::memory_order_relaxed);
        g_slp_counters.path_a_fail_us.fetch_add(warm_dur, std::memory_order_relaxed);
      }
    }
  }

  // ══════════════════════════════════════════════════════════════════════
  // PATH B: Dense warm-start for small problems (m ≤ 200).
  // ══════════════════════════════════════════════════════════════════════
  if (!out.solved_from_hint && hint_match && m <= kSparseSimplexThreshold) {
    bool state_ok = false;
    if (basis_hint->cached_inverse &&
        basis_hint->cached_inverse->rows() == m &&
        basis_hint->cached_inverse->cols() == m) {
      const auto& parent_binv = *basis_hint->cached_inverse;
      x_b = parent_binv * sf.b;
      if (basis_hint->cached_reduced_costs &&
          basis_hint->cached_reduced_costs->size() == n) {
        reduced_costs = *basis_hint->cached_reduced_costs;
      } else {
        Eigen::VectorXd c_b(m);
        for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
        const Eigen::VectorXd pi = parent_binv.transpose() * c_b;
        reduced_costs = sf.c_max - Eigen::VectorXd(sf.A.transpose() * pi);
      }
      Eigen::VectorXd c_b(m);
      for (int i = 0; i < m; ++i) c_b[i] = sf.c_max[basis[i]];
      if (has_ub) {
        std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
        for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
        for (int j = 0; j < n; ++j) {
          if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (!std::isfinite(uj)) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
            x_b -= it.value() * uj * parent_binv.col(it.row());
          }
        }
      }
      obj = c_b.dot(x_b);
      if (has_ub) {
        std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
        for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
        for (int j = 0; j < n; ++j) {
          if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
        }
      }
      bool primal_feas = approx_nonnegative(x_b, opt.feasibility_tol);
      if (primal_feas && has_ub) {
        for (int i = 0; i < m; ++i) {
          const int bvar = basis[i];
          const double ub_i = sf.var_ub[bvar];
          if (std::isfinite(ub_i) && x_b[i] > ub_i + opt.feasibility_tol) {
            primal_feas = false; break;
          }
        }
      }
      if (primal_feas) {
        out.shared_binv = basis_hint->cached_inverse;
        state_ok = true;
      } else {
        binv = parent_binv;
        state_ok = true;
      }
    } else {
      state_ok = compute_basis_state(sf, basis, binv, x_b, reduced_costs, obj);
      if (state_ok && has_ub) {
        std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
        for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
        for (int j = 0; j < n; ++j) {
          if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (!std::isfinite(uj)) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
            x_b -= it.value() * uj * binv.col(it.row());
          }
        }
        Eigen::VectorXd c_b2(m);
        for (int i = 0; i < m; ++i) c_b2[i] = sf.c_max[basis[i]];
        obj = c_b2.dot(x_b);
        for (int j = 0; j < n; ++j) {
          if (!at_upper[static_cast<size_t>(j)] || is_basic_tmp[static_cast<size_t>(j)]) continue;
          const double uj = sf.var_ub[j];
          if (std::isfinite(uj)) obj += sf.c_max[j] * uj;
        }
      }
    }
    if (state_ok) {
      out.solved_from_hint = true;
      bool primal_feas = approx_nonnegative(x_b, opt.feasibility_tol);
      if (primal_feas && has_ub) {
        for (int i = 0; i < m; ++i) {
          const int bvar = basis[i];
          const double ub_i = sf.var_ub[bvar];
          if (std::isfinite(ub_i) && x_b[i] > ub_i + opt.feasibility_tol) {
            primal_feas = false; break;
          }
        }
      }
      if (primal_feas) {
        out.dual_reoptimized = true;
      } else {
        std::vector<char> hint_can_enter(static_cast<size_t>(n), 1);
        for (int i = 0; i < m; ++i) {
          const int art = sf.row_to_artificial_col[i];
          if (art >= 0) hint_can_enter[static_cast<size_t>(art)] = 0;
        }
        bool df_ok = opt.prefer_dual_simplex_reopt &&
            approx_dual_feasible(reduced_costs, basis, hint_can_enter, at_upper, opt.optimality_tol);
        bool dr_ok = df_ok &&
            dual_simplex_reoptimize(sf, basis, hint_can_enter, opt, binv, x_b, reduced_costs, obj,
                                    at_upper, /*state_initialized=*/true);
        if (dr_ok) {
          out.dual_reoptimized = true;
        } else {
          out.solved_from_hint = false;
        }
      }
    }
  }

  // ══════════════════════════════════════════════════════════════════════
  // PATH C: Cold start (no hint or warm-start failed).
  // For large problems with a basis hint, skip cold-start entirely —
  // declare the node LP failed so B&C prunes it.  Cold-start Phase I+II
  // is too expensive for tree nodes (O(m³) dense or many-iteration sparse).
  // ══════════════════════════════════════════════════════════════════════
  if (!out.solved_from_hint) {
    if (dbg) MIPSOLVERS_LOG_DEBUG("[solve_lp_from_sf] COLD START m={} n={} hint_match={}", m, n, hint_match);
  }

  // ── FALLBACK: Try root/fallback basis before expensive cold start ──
  // When warm-start from parent basis fails, try the root basis (cleaner,
  // no eta contamination).  This avoids a 2+ second cold start in most cases.
  if (!out.solved_from_hint && opt.fallback_basis &&
      opt.fallback_basis->rows == m && opt.fallback_basis->cols == n &&
      opt.fallback_basis != basis_hint) {
    SimplexOptions fb_opt = opt;
    fb_opt.fallback_basis = nullptr;   // Prevent further recursion
    fb_opt.allow_cold_start = false;   // Only try warm-start path
    auto fb_result = solve_lp_from_sf(sf, fb_opt, opt.fallback_basis);
    if (fb_result.result.stats.success) {
      return fb_result;
    }
    // Fallback warm-start also failed — continue to cold start below.
  }

  // ══════════════════════════════════════════════════════════════════════
  if (!out.solved_from_hint && hint_match && m > kSparseSimplexThreshold &&
      !opt.allow_cold_start) {
    out.result.stats.status = "Sparse warm-start failed";
    out.result.stats.success = false;
    return out;
  }
  if (!out.solved_from_hint) {
    g_slp_counters.cold_start.fetch_add(1, std::memory_order_relaxed);
    if (dbg) MIPSOLVERS_LOG_DEBUG("  COLD START");
    at_upper.assign(static_cast<size_t>(n), 0);

    // Provable infeasibility check: if b[i] < 0 and all column coefficients
    // in row i are non-negative, then Ax >= 0 > b[i] is impossible.
    bool cold_start_infeasible = false;
    for (int i = 0; i < m; ++i) {
      if (sf.b[i] >= -opt.feasibility_tol) continue;
      bool all_nonneg = true;
      for (int col = 0; col < sf.A.outerSize() && all_nonneg; ++col) {
        if (has_ub && sf.var_ub[col] < opt.feasibility_tol) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, col); it; ++it) {
          if (it.row() == i && it.value() < -opt.feasibility_tol) {
            all_nonneg = false;
            break;
          }
        }
      }
      if (all_nonneg) {
        cold_start_infeasible = true;
        break;
      }
    }
    if (cold_start_infeasible) {
      mark_proven_infeasible(out.result.stats);
      return out;
    }

    // Reset basis to initial slack/artificial.
    for (int i = 0; i < m; ++i) {
      if (sf.row_to_slack_col[i] >= 0) {
        basis[i] = sf.row_to_slack_col[i];
      } else if (sf.row_to_artificial_col[i] >= 0) {
        basis[i] = sf.row_to_artificial_col[i];
      } else {
        basis[i] = -1;
      }
    }
    trace_root_coldstate(sf, basis, at_upper, nullptr, nullptr,
                         "reset_slack_art", sf_call_count, "none", 0, true);

    // Crash: replace artificial columns with original-variable columns.
    [[maybe_unused]] auto t_crash_0 = std::chrono::high_resolution_clock::now();
    crash_artificial_basis(sf, basis);
    [[maybe_unused]] auto t_crash_1 = std::chrono::high_resolution_clock::now();
    trace_root_coldstate(sf, basis, at_upper, nullptr, nullptr,
                         "after_crash_basis", sf_call_count, "none", 0, true);
    // Count remaining artificials after crash.
    [[maybe_unused]] int art_remain = 0;
    for (int i = 0; i < m; ++i) {
      if (sf.row_to_artificial_col[i] >= 0 &&
          basis[i] == sf.row_to_artificial_col[i])
        ++art_remain;
    }
    MIPSOLVERS_LOG_DEBUG("[COLD] crash={:.1f}ms art_remain={}/{} m={} n={}",
        std::chrono::duration<double, std::milli>(t_crash_1 - t_crash_0).count(),
        art_remain, sf.n_artificial, m, n);

    if (m > kSparseSimplexThreshold) {
      // --- Sparse LU simplex for large problems ---
      SparseBasis sbasis(sf.A,
             SparseBasis::from_public_backend(opt.factor_backend));

      // Validate crash basis: if the greedy crash produced a singular basis,
      // revert all crash swaps back to slacks/artificials.
      if (!sbasis.refactorize(basis)) {
        for (int i = 0; i < m; ++i) {
          if (sf.row_to_slack_col[i] >= 0) {
            basis[i] = sf.row_to_slack_col[i];
          } else if (sf.row_to_artificial_col[i] >= 0) {
            basis[i] = sf.row_to_artificial_col[i];
          }
        }
        sbasis.refactorize(basis);
        trace_root_coldstate(sf, basis, at_upper, nullptr, nullptr,
                             "after_crash_singular_revert", sf_call_count,
                             "none", 0, true);
      }

      // Check if crash basis is primal-infeasible; if so, revert to
      // identity basis so Phase I starts clean.
      {
        const bool crash_has_ub = sf.var_ub.size() == n;
        Eigen::VectorXd crash_xb = sbasis.ftran(sf.b);
        bool infeasible = false;
        for (int i = 0; i < m; ++i) {
          if (crash_xb[i] < -1e-8) { infeasible = true; break; }
          if (crash_has_ub) {
            const double ub_i = sf.var_ub[basis[i]];
            if (std::isfinite(ub_i) && crash_xb[i] > ub_i + 1e-8) {
              infeasible = true; break;
            }
          }
        }
        if (infeasible) {
          for (int i = 0; i < m; ++i) {
            if (sf.row_to_slack_col[i] >= 0) {
              basis[i] = sf.row_to_slack_col[i];
            } else if (sf.row_to_artificial_col[i] >= 0) {
              basis[i] = sf.row_to_artificial_col[i];
            }
          }
          sbasis.refactorize(basis);
          trace_root_coldstate(sf, basis, at_upper, &crash_xb, nullptr,
                               "after_crash_infeasible_revert",
                               sf_call_count, "none", 0, true);
        } else {
          trace_root_coldstate(sf, basis, at_upper, &crash_xb, nullptr,
                               "after_crash_feasible_check",
                               sf_call_count, "none", 0, true);
        }
      }

      // ═══════════════════════════════════════════════════════════════════
      // Dual Phase I — Objective Shifting (no Big-M, no weighted penalty).
      //
      // 1. Compute reduced costs c̄ for the crash basis.
      // 2. For each non-basic j violating dual feasibility, apply a
      //    cost shift Δ_j = |c̄_j| + 1 to make the basis dual-feasible.
      // 3. Force artificial var_ub = 0 so basic artificials violate
      //    their upper bound and get pivoted out by dual simplex.
      // 4. Run dual simplex (with BFRT) to reach primal feasibility.
      // 5. Remove all shifts, recompute with the real objective.
      // 6. Phase II: primal simplex with real objective.
      //
      // Fallback: if dual simplex doesn't converge, use strict
      // Primal Phase I (c_art = −1, c_others = 0).
      // ═══════════════════════════════════════════════════════════════════
      [[maybe_unused]] auto t_p1_0 = std::chrono::high_resolution_clock::now();

      // Build is_basic mask for crash basis.
      std::vector<char> is_basic_cs(static_cast<size_t>(n), 0);
      for (int i = 0; i < m; ++i) {
        if (basis[i] >= 0 && basis[i] < n)
          is_basic_cs[static_cast<size_t>(basis[i])] = 1;
      }

      // Compute reduced costs from crash basis with real objective.
      sparse_full_recompute(sf, basis, at_upper, is_basic_cs,
                            sbasis, x_b, reduced_costs, obj);
      trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                           "after_crash_recompute_real_obj",
                           sf_call_count, "none", 0, true);

      // Build the shifted LP: force artificial ub = 0.
      StandardFormLP shifted = sf;
      for (int i = 0; i < m; ++i) {
        const int art = shifted.row_to_artificial_col[i];
        if (art >= 0) shifted.var_ub[art] = 0.0;
      }

      // Recompute state with artificial bounds capped at 0.
      sparse_full_recompute(shifted, basis, at_upper, is_basic_cs,
                            sbasis, x_b, reduced_costs, obj);
      trace_root_coldstate(shifted, basis, at_upper, &x_b, &reduced_costs,
                           "after_artificial_ub_cap",
                           sf_call_count, "none", 0, true);

      // Apply exact cost shifts for dual feasibility.
      // Maximisation: NB at lower ⇒ need c̄_j ≤ 0; NB at upper ⇒ need c̄_j ≥ 0.
      for (int j = 0; j < n; ++j) {
        if (is_basic_cs[static_cast<size_t>(j)]) continue;
        const bool j_up = at_upper[static_cast<size_t>(j)];
        if (!j_up && reduced_costs[j] > opt.optimality_tol) {
          shifted.c_max[j] -= (reduced_costs[j] + 1.0);
          reduced_costs[j] = -1.0;
        } else if (j_up && reduced_costs[j] < -opt.optimality_tol) {
          shifted.c_max[j] += (-reduced_costs[j] + 1.0);
          reduced_costs[j] = 1.0;
        }
      }
      // Recompute shifted objective from scratch.
      {
        obj = 0.0;
        for (int i = 0; i < m; ++i)
          obj += shifted.c_max[basis[i]] * x_b[i];
        for (int j = 0; j < n; ++j) {
          if (!is_basic_cs[static_cast<size_t>(j)] &&
              at_upper[static_cast<size_t>(j)] &&
              std::isfinite(shifted.var_ub[j]))
            obj += shifted.c_max[j] * shifted.var_ub[j];
        }
      }

      // Run dual simplex with shifted costs (artificials blocked).
      std::vector<char> dp1_can_enter(static_cast<size_t>(n), 1);
      for (int i = 0; i < m; ++i) {
        const int art = shifted.row_to_artificial_col[i];
        if (art >= 0) dp1_can_enter[static_cast<size_t>(art)] = 0;
      }
      bool dp1_inf = false, dp1_cut = false;
      bool dp1_ok = sparse_dual_simplex_reoptimize(
          shifted, basis, dp1_can_enter, opt, sbasis,
          x_b, reduced_costs, obj, at_upper,
          dp1_inf, dp1_cut);
      const int dp1_iters = tl_dual_reopt_iters;
      const char* dse_mode = exact_edge_mode ? "exact" : "persist_or_unit";
      trace_root_coldstate(shifted, basis, at_upper, &x_b, &reduced_costs,
                           "after_dual_phase_i_shifted",
                           sf_call_count, dse_mode, dp1_iters, dp1_ok);

      // Dual Phase I works on the shifted LP where artificial columns have
      // ub = 0. Before switching back to the original LP, clear the artificial
      // at_upper flags and rebuild the numeric factorization so Phase II starts
      // from a clean, consistent state.
      for (int i = 0; i < m; ++i) {
        const int art = sf.row_to_artificial_col[i];
        if (art >= 0) at_upper[static_cast<size_t>(art)] = 0;
      }
      sbasis.refactorize(basis);
      trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                           "after_artificial_side_reset",
                           sf_call_count, dse_mode, dp1_iters, dp1_ok);

      if (dp1_inf) {
        mark_proven_infeasible(out.result.stats);
        binv = sbasis.compute_dense_inverse(basis);
        extract_farkas_certificate(sf, basis, binv, out.result.stats);
        return out;
      }

      if (!dp1_ok && !dp1_cut) {
        // Dual Phase I did not converge — fallback to strict Primal Phase I
        // (c_art = −1, c_others = 0; no Big-M penalty).
        std::fill(at_upper.begin(), at_upper.end(), 0);
        for (int i = 0; i < m; ++i) {
          if (sf.row_to_slack_col[i] >= 0) {
            basis[i] = sf.row_to_slack_col[i];
          } else if (sf.row_to_artificial_col[i] >= 0) {
            basis[i] = sf.row_to_artificial_col[i];
          }
        }
        StandardFormLP phase1 = sf;
        phase1.c_max.setZero();
        std::vector<char> p1_can_enter(static_cast<size_t>(n), 1);
        for (int i = 0; i < m; ++i) {
          const int art = phase1.row_to_artificial_col[i];
          if (art >= 0) {
            phase1.c_max[art] = -1.0;
            p1_can_enter[static_cast<size_t>(art)] = 0;
          }
        }
        if (!sparse_primal_simplex_optimize(phase1, basis, p1_can_enter, opt,
                                            sbasis, x_b, reduced_costs, obj, at_upper)) {
          trace_root_coldstate(phase1, basis, at_upper, &x_b, &reduced_costs,
                               "after_primal_phase_i_failed",
                               sf_call_count, "devex",
                               tl_primal_simplex_iters, false);
          out.result.stats.status = "Simplex Phase I failed";
          return out;
        }
        trace_root_coldstate(phase1, basis, at_upper, &x_b, &reduced_costs,
                             "after_primal_phase_i_fallback",
                             sf_call_count, "devex",
                             tl_primal_simplex_iters, true);
        if (-obj > opt.feasibility_tol) {
          mark_proven_infeasible(out.result.stats);
          binv = sbasis.compute_dense_inverse(basis);
          extract_farkas_certificate(sf, basis, binv, out.result.stats);
          return out;
        }
      }

      [[maybe_unused]] auto t_p1_1 = std::chrono::high_resolution_clock::now();
      MIPSOLVERS_LOG_DEBUG("[COLD-CS] Dual Phase I: {:.1f}ms  dp1_ok={}",
          std::chrono::duration<double, std::milli>(t_p1_1 - t_p1_0).count(),
          dp1_ok);

      // Check residual artificials.
      {
        bool has_art = false;
        for (int i = 0; i < m; ++i) {
          const int art = sf.row_to_artificial_col[i];
          if (art >= 0 && basis[i] == art && x_b[i] > opt.feasibility_tol) {
            has_art = true; break;
          }
        }
        if (has_art) {
          mark_proven_infeasible(out.result.stats);
          binv = sbasis.compute_dense_inverse(basis);
          extract_farkas_certificate(sf, basis, binv, out.result.stats);
          return out;
        }
      }

      // Remove shifts: recompute with the REAL objective.
      {
        std::vector<char> is_basic_now(static_cast<size_t>(n), 0);
        for (int i = 0; i < m; ++i) {
          if (basis[i] >= 0 && basis[i] < n)
            is_basic_now[static_cast<size_t>(basis[i])] = 1;
        }
        sparse_full_recompute(sf, basis, at_upper, is_basic_now,
                              sbasis, x_b, reduced_costs, obj);
        trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                             "after_real_objective_recompute",
                             sf_call_count, dse_mode, dp1_iters, true);
      }

      // Phase II: primal simplex with real objective (artificials blocked).
      for (int i = 0; i < m; ++i) {
        const int art = sf.row_to_artificial_col[i];
        if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
      }
      if (!sparse_primal_simplex_optimize(sf, basis, can_enter, opt,
                                          sbasis, x_b, reduced_costs, obj, at_upper)) {
        trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                             "after_phase_ii_failed",
                             sf_call_count, "devex",
                             tl_primal_simplex_iters, false);
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
      const int p2_iters = tl_primal_simplex_iters;
      actual_iters += dp1_iters + p2_iters;
      trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                           "after_phase_ii", sf_call_count,
                           exact_edge_mode ? "exact_primal_edge" : "devex",
                           p2_iters, true);
      [[maybe_unused]] auto t_p2_1 = std::chrono::high_resolution_clock::now();
      if (dbg) {
        MIPSOLVERS_LOG_DEBUG("  COLD START Dual Phase I: {:.1f}ms  Phase II: {:.1f}ms",
            std::chrono::duration<double, std::milli>(t_p1_1 - t_p1_0).count(),
            std::chrono::duration<double, std::milli>(t_p2_1 - t_p1_1).count());
      }
      // Persist clean SparseBasis for child nodes.
        solved_sparse_basis = std::make_shared<SparseBasis>(
          sf.A, SparseBasis::from_public_backend(opt.factor_backend));
      solved_sparse_basis->refactorize(basis);
      // P10: transfer DSE weights computed during the cold-start solve.
      solved_sparse_basis->copy_dse_weights_from(sbasis);
      if (exact_edge_mode &&
          !solved_sparse_basis->dse_weights_match(basis)) {
        solved_sparse_basis->save_dse_weights(
            solved_sparse_basis->compute_exact_dse_weights(), basis);
      }
    } else {
      // --- Dense simplex for small problems: strict Primal Phase I ---
      StandardFormLP phase1 = sf;
      phase1.c_max.setZero();
      std::vector<char> phase1_can_enter(static_cast<size_t>(n), 1);
      for (int i = 0; i < m; ++i) {
        const int art = phase1.row_to_artificial_col[i];
        if (art >= 0) {
          phase1.c_max[art] = -1.0;
          phase1_can_enter[static_cast<size_t>(art)] = 0;
        }
      }
      if (!primal_simplex_optimize(phase1, basis, phase1_can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase I failed";
        return out;
      }
      if (-obj > opt.feasibility_tol) {
        if (dbg) MIPSOLVERS_LOG_DEBUG("  INFEAS-C: dense Phase I obj={:.4f}", -obj);
        out.result.stats.status = "LP infeasible";
        extract_farkas_certificate(sf, basis, binv, out.result.stats);
        return out;
      }

      for (int i = 0; i < m; ++i) {
        const int art = sf.row_to_artificial_col[i];
        if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
      }
      if (!primal_simplex_optimize(sf, basis, can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
    }
  } else if (!out.dual_reoptimized) {
    // Hint matched and was primal-feasible — only need Phase II for dual.
    for (int i = 0; i < m; ++i) {
      const int art = sf.row_to_artificial_col[i];
      if (art >= 0) can_enter[static_cast<size_t>(art)] = 0;
    }
    if (m > kSparseSimplexThreshold) {
      SparseBasis sbasis(sf.A,
             SparseBasis::from_public_backend(opt.factor_backend));
      if (!sparse_primal_simplex_optimize(sf, basis, can_enter, opt,
                                          sbasis, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
      // No dense binv needed.
    } else {
      if (!primal_simplex_optimize(sf, basis, can_enter, opt, binv, x_b, reduced_costs, obj, at_upper)) {
        out.result.stats.status = "Simplex Phase II failed";
        return out;
      }
    }
  }

  // Record cold-start timing.
  if (!out.solved_from_hint) {
    auto t_end = std::chrono::high_resolution_clock::now();
    g_slp_counters.cold_us.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t0).count(),
        std::memory_order_relaxed);
  }

  out.basis.indices = basis;
  out.basis.rows = m;
  out.basis.cols = n;
  out.basis.at_upper = at_upper;
  if (solved_sparse_basis) {
    out.basis.cached_sparse_basis = solved_sparse_basis;
    out.basis.persist_eta_count = solved_sparse_basis->eta_count();
  }
  if (out.shared_binv) {
    out.basis_inverse = *out.shared_binv;
  } else {
    out.basis_inverse = std::move(binv);
  }

  const bool frontier_remap_mode =
      !opt.suppress_degenerate_frontier_remap &&
      (opt.enable_degenerate_frontier_remap ||
       simplex_degenerate_frontier_remap_env_enabled());
  if (frontier_remap_mode && m > kSparseSimplexThreshold) {
    SparseBasis remap_basis(sf.A,
                            SparseBasis::from_public_backend(opt.factor_backend));
    if (remap_basis.refactorize(basis)) {
      const int remapped = remap_degenerate_original_frontier(
          sf, basis, at_upper, remap_basis, x_b, reduced_costs, obj, opt);
      if (remapped > 0) {
        out.basis.indices = basis;
        out.basis.at_upper = at_upper;
        out.basis.cached_sparse_basis = std::make_shared<SparseBasis>(
            sf.A, SparseBasis::from_public_backend(opt.factor_backend));
        auto sb = std::dynamic_pointer_cast<SparseBasis>(
            out.basis.cached_sparse_basis);
        sb->refactorize(basis);
        if (exact_edge_mode) {
          sb->save_dse_weights(sb->compute_exact_dse_weights(), basis);
        }
        out.basis.persist_eta_count = sb->eta_count();
      }
      if (root_coldstate_diag_enabled()) {
        trace_root_coldstate(sf, basis, at_upper, &x_b, &reduced_costs,
                             "after_degenerate_frontier_remap",
                             sf_call_count,
                             remapped > 0 ? "zero_pivot_remap" : "none",
                             remapped, true);
      }
    }
  }

  out.x_basic = x_b;
  out.reduced_costs = std::move(reduced_costs);
  out.max_objective = obj;
  out.exact_optimal = true;

  out.x_std = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < m; ++i) {
    if (basis[i] >= 0 && basis[i] < n) {
      out.x_std[basis[i]] = x_b[i];
    }
  }
  if (has_ub) {
    std::vector<char> is_basic_tmp(static_cast<size_t>(n), 0);
    for (int idx : basis) if (idx >= 0 && idx < n) is_basic_tmp[static_cast<size_t>(idx)] = 1;
    for (int j = 0; j < n; ++j) {
      if (at_upper[static_cast<size_t>(j)] && !is_basic_tmp[static_cast<size_t>(j)]) {
        out.x_std[j] = sf.var_ub[j];
      }
    }
  }
  out.result.x = extract_solution(sf, out.x_std);
  out.basis.cached_x_basic =
      std::make_shared<const Eigen::VectorXd>(out.x_basic);
  out.basis.cached_x_std =
      std::make_shared<const Eigen::VectorXd>(out.x_std);
  out.basis.cached_max_objective = out.max_objective;
  out.basis.has_cached_max_objective = true;

  out.result.stats.success = true;
  out.result.stats.iterations = actual_iters;
  out.result.stats.objective = sf.objective_const - obj;
  out.result.stats.status = "Optimal";
  out.result.stats.primal_feas = 0.0;
  out.result.stats.residual_inf = 0.0;
  trace_lp_basis_state(out, basis, at_upper, x_b, "from_sf_final", &sf);
  populate_dual_certificate(sf, basis, at_upper, out.basis_inverse,
                            out.basis.cached_sparse_basis,
                            out.reduced_costs, out.result);
  
  if (dbg) {
    auto t1 = std::chrono::high_resolution_clock::now();
    [[maybe_unused]] double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    MIPSOLVERS_LOG_DEBUG("  → {} obj={:.2f}  time={:.1f}ms hint={} dreopt={}", out.result.stats.status.c_str(), out.result.stats.objective,
            ms, (int)out.solved_from_hint, (int)out.dual_reoptimized);
  }
  return out;
}

}  // namespace mipsolvers::engine

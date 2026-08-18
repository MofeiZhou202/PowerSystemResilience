/// @file lp_presolve.cpp
/// @brief Native LP presolve — P1 zero-fill rules + postsolve stack.
///
/// Design document: docs/archive/native_presolve_lp_2026-08-18.md §4 (phase
/// P1).  Rule catalog and correctness arguments: Andersen & Andersen,
/// "Presolving in Linear Programming", Mathematical Programming 71 (1995),
/// §2.1-2.3.  The pass works on a merged single-block representation (Aeq
/// rows appended to A with lhs == rhs == beq) held as CSC + CSR adjacency
/// with row/column active flags — the in-repo adjacency template is
/// src/engine/solver/native/milp/bc/milp_presolve.cpp (§3.1) — and compacts
/// once after the fixpoint.  Sense is normalized to minimization internally
/// and restored on output.

#include "mipsolvers/engine/presolve/lp_presolve.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <map>
#include <type_traits>

namespace mipsolvers::engine {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Tolerances (design §5, risk R1).  Two envelopes:
//  - kInfeasTol = 1e-7 relative (HiGHS primal_feasibility_tolerance
//    magnitude): infeasibility is declared only when a violation exceeds
//    this envelope (double-tolerance confirmation, R1).
//  - kDeleteTol = 1e-8 relative: slacks accepted when *deleting* a
//    constraint.  These slacks resurface as original-model violations at
//    postsolve time, and the wiring's original-model audit accepts at
//    max(1e-10, 10*tol_primal) = 1e-7 relative to the global side scale;
//    keeping deletion slacks one decade inside that envelope guarantees the
//    audit cannot reject a correct postsolve on the tolerance boundary.
constexpr double kInfeasTol = 1e-7;
constexpr double kDeleteTol = 1e-8;
// Fixed-column detection, same scale as the IPM kernel's own fixed-variable
// detection (ipm_lp_solver.cpp, ub - lb < 1e-9).  Fixing at the midpoint is
// exact: rows are shifted by exactly the recovered value, so this slack
// never reaches the original-model audit.
constexpr double kFixedTol = 1e-9;
// Matrix zero detection: 1e-9 * max(1, max |a_ij| in the row) (§5 R1).
constexpr double kZeroTol = 1e-9;

// Row sides use the same no-bound sentinel convention as the original-model
// residual audit lp_solution_residual_acceptable: |side| >= 1e19 is no bound.
bool finite_row_side(double v) { return std::isfinite(v) && std::abs(v) < 1e19; }

/// Merged single-block working representation (design §3.1).
struct PresolveWork {
  int m = 0;
  int n = 0;
  std::vector<double> c;    // minimization convention
  std::vector<double> lb;   // -kInf when no lower bound
  std::vector<double> ub;   // +kInf when no upper bound
  std::vector<double> lhs;  // -kInf when no lower row side
  std::vector<double> rhs;  // +kInf when no upper row side
  // CSC over the merged row set.
  std::vector<int> csc_start;
  std::vector<int> csc_row;
  std::vector<double> csc_val;
  // CSR over the merged row set.
  std::vector<int> csr_start;
  std::vector<int> csr_col;
  std::vector<double> csr_val;
  std::vector<char> row_active;
  std::vector<char> col_active;
};

/// Build the merged representation.  Returns false on non-finite input
/// (NaN coefficient/cost/side) — the caller reports "invalid_input" instead
/// of reducing a model it cannot reason about (no silent plausible numbers).
bool build_work(const LPModel& lp, PresolveWork& w) {
  const int mi = static_cast<int>(lp.A.rows());
  const int me = static_cast<int>(lp.Aeq.rows());
  const int n = static_cast<int>(lp.c.size());
  // Shape consistency: a malformed model is not presolved (fail-loud via
  // "invalid_input"); the caller's direct path owns its validation.
  if (lp.b.size() != mi || lp.beq.size() != me ||
      (lp.row_lhs.size() != 0 && lp.row_lhs.size() != mi) ||
      (mi > 0 && lp.A.cols() != n) || (me > 0 && lp.Aeq.cols() != n)) {
    return false;
  }
  w.m = mi + me;
  w.n = n;
  w.c.resize(static_cast<std::size_t>(n));
  const double sense_sign = lp.sense == Sense::Maximize ? -1.0 : 1.0;
  for (int j = 0; j < n; ++j) {
    if (!std::isfinite(lp.c[j])) return false;
    w.c[static_cast<std::size_t>(j)] = sense_sign * lp.c[j];
  }
  w.lb.resize(static_cast<std::size_t>(n));
  w.ub.resize(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    const double lo = j < static_cast<int>(lp.vars.size())
                          ? lp.vars[static_cast<std::size_t>(j)].lb
                          : -kVariableNoBound;
    const double hi = j < static_cast<int>(lp.vars.size())
                          ? lp.vars[static_cast<std::size_t>(j)].ub
                          : kVariableNoBound;
    if (std::isnan(lo) || std::isnan(hi)) return false;
    w.lb[static_cast<std::size_t>(j)] =
        variable_has_finite_lower_bound(lo) ? lo : -kInf;
    w.ub[static_cast<std::size_t>(j)] =
        variable_has_finite_upper_bound(hi) ? hi : kInf;
  }
  w.lhs.resize(static_cast<std::size_t>(w.m));
  w.rhs.resize(static_cast<std::size_t>(w.m));
  for (int i = 0; i < mi; ++i) {
    const double lo = lp_row_lhs_or_neg_inf(lp, i);
    const double hi = lp.b[i];
    if (std::isnan(lo) || std::isnan(hi)) return false;
    w.lhs[static_cast<std::size_t>(i)] = finite_row_side(lo) ? lo : -kInf;
    w.rhs[static_cast<std::size_t>(i)] = finite_row_side(hi) ? hi : kInf;
  }
  for (int k = 0; k < me; ++k) {
    if (!std::isfinite(lp.beq[k])) return false;
    w.lhs[static_cast<std::size_t>(mi + k)] = lp.beq[k];
    w.rhs[static_cast<std::size_t>(mi + k)] = lp.beq[k];
  }

  // CSC: column j holds its A entries (rows 0..mi-1) then its Aeq entries
  // (rows mi..m-1).  Exact zeros are dropped so structural degrees below
  // count algebraically meaningful entries only.
  w.csc_start.assign(static_cast<std::size_t>(n) + 1, 0);
  w.csc_row.clear();
  w.csc_val.clear();
  w.csc_row.reserve(static_cast<std::size_t>(lp.A.nonZeros() +
                                             lp.Aeq.nonZeros()));
  w.csc_val.reserve(w.csc_row.capacity());
  for (int j = 0; j < n; ++j) {
    w.csc_start[static_cast<std::size_t>(j)] =
        static_cast<int>(w.csc_row.size());
    if (j < lp.A.outerSize()) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        if (it.value() == 0.0) continue;
        if (!std::isfinite(it.value())) return false;
        w.csc_row.push_back(static_cast<int>(it.row()));
        w.csc_val.push_back(it.value());
      }
    }
    if (j < lp.Aeq.outerSize()) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        if (it.value() == 0.0) continue;
        if (!std::isfinite(it.value())) return false;
        w.csc_row.push_back(mi + static_cast<int>(it.row()));
        w.csc_val.push_back(it.value());
      }
    }
  }
  w.csc_start[static_cast<std::size_t>(n)] =
      static_cast<int>(w.csc_row.size());

  // CSR from CSC (counting sort over rows).
  w.csr_start.assign(static_cast<std::size_t>(w.m) + 1, 0);
  for (std::size_t p = 0; p < w.csc_row.size(); ++p)
    ++w.csr_start[static_cast<std::size_t>(w.csc_row[p]) + 1];
  for (int i = 0; i < w.m; ++i)
    w.csr_start[static_cast<std::size_t>(i) + 1] +=
        w.csr_start[static_cast<std::size_t>(i)];
  const int nnz = w.csr_start[static_cast<std::size_t>(w.m)];
  w.csr_col.resize(static_cast<std::size_t>(nnz));
  w.csr_val.resize(static_cast<std::size_t>(nnz));
  {
    std::vector<int> pos(w.csr_start.begin(), w.csr_start.begin() + w.m);
    for (int j = 0; j < n; ++j)
      for (int p = w.csc_start[static_cast<std::size_t>(j)];
           p < w.csc_start[static_cast<std::size_t>(j) + 1]; ++p) {
        const int q = pos[static_cast<std::size_t>(w.csc_row[p])]++;
        w.csr_col[static_cast<std::size_t>(q)] = j;
        w.csr_val[static_cast<std::size_t>(q)] = w.csc_val[p];
      }
  }
  w.row_active.assign(static_cast<std::size_t>(w.m), 1);
  w.col_active.assign(static_cast<std::size_t>(n), 1);
  return true;
}

struct RuleCounts {
  long empty_rows = 0;
  long empty_cols = 0;
  long fixed_cols = 0;
  long redundant_rows = 0;
  long singleton_rows = 0;
  long implied_bound_tightenings = 0;
  long doubleton_equations = 0;
  long singleton_columns = 0;
  long free_column_substitutions = 0;
  bool p2_structure_sampled = false;
  long active_singleton_col_candidates = 0;
  long singleton_equality_col_candidates = 0;
  long singleton_ranged_col_candidates = 0;
  long doubleton_equality_row_candidates = 0;
  long free_equality_col_candidates = 0;
  long implied_free_equality_col_candidates = 0;
  long parallel_row_candidates = 0;
  long parallel_col_candidates = 0;
  long total() const {
    return empty_rows + empty_cols + fixed_cols + redundant_rows +
           singleton_rows + implied_bound_tightenings +
           doubleton_equations + singleton_columns +
           free_column_substitutions;
  }
};

/// Row-side tolerance scale: `factor` relative to the largest finite side
/// magnitude (§5 R1, HiGHS primal_feasibility_tolerance magnitude).
double row_tol(double lhs, double rhs, double factor) {
  double scale = 1.0;
  if (lhs > -kInf) scale = std::max(scale, std::abs(lhs));
  if (rhs < kInf) scale = std::max(scale, std::abs(rhs));
  return factor * scale;
}

enum class RuleOutcome { kOk, kInfeasible, kUnboundedCandidate };

/// Row pass: empty rows, singleton rows, redundant rows (A&A §2.1, §2.3).
RuleOutcome row_sweep(PresolveWork& w, RuleCounts& counts) {
  for (int i = 0; i < w.m; ++i) {
    if (!w.row_active[static_cast<std::size_t>(i)]) continue;
    const int rb = w.csr_start[static_cast<std::size_t>(i)];
    const int re = w.csr_start[static_cast<std::size_t>(i) + 1];
    const double lhs = w.lhs[static_cast<std::size_t>(i)];
    const double rhs = w.rhs[static_cast<std::size_t>(i)];
    const double tol_infeas = row_tol(lhs, rhs, kInfeasTol);
    const double tol_delete = row_tol(lhs, rhs, kDeleteTol);

    // Collect active entries and min/max activity in one pass (A&A §2.3:
    // activity bounds from the column bounds; an unbounded contributing
    // column makes the corresponding activity bound infinite).
    int degree = 0;
    int last_col = -1;
    double last_val = 0.0;
    double row_abs_max = 0.0;
    double min_act = 0.0;
    double max_act = 0.0;
    bool min_inf = false;  // min activity is -inf
    bool max_inf = false;  // max activity is +inf
    for (int p = rb; p < re; ++p) {
      const int j = w.csr_col[static_cast<std::size_t>(p)];
      if (!w.col_active[static_cast<std::size_t>(j)]) continue;
      const double a = w.csr_val[static_cast<std::size_t>(p)];
      ++degree;
      last_col = j;
      last_val = a;
      row_abs_max = std::max(row_abs_max, std::abs(a));
      const double lo = w.lb[static_cast<std::size_t>(j)];
      const double hi = w.ub[static_cast<std::size_t>(j)];
      if (a > 0.0) {
        if (lo == -kInf) min_inf = true; else min_act += a * lo;
        if (hi == kInf) max_inf = true; else max_act += a * hi;
      } else {
        if (hi == kInf) min_inf = true; else min_act += a * hi;
        if (lo == -kInf) max_inf = true; else max_act += a * lo;
      }
    }
    // Overflow guard: a finite-bound accumulation that left the finite
    // regime carries no information about the true activity bound (huge
    // opposite-sign terms may cancel).  Treat that side as unbounded —
    // conservative in both directions: it can neither prove infeasibility
    // nor justify a redundant-row deletion from that side (§5 R1).
    if (!std::isfinite(min_act)) min_inf = true;
    if (!std::isfinite(max_act)) max_inf = true;

    if (degree == 0) {
      // Empty row (A&A §2.1): the activity is identically zero.  A violation
      // beyond the infeasibility envelope proves infeasibility (§5 R1); zero
      // inside the tighter deletion envelope lets the row go (the residual
      // slack stays a decade below the original-model audit envelope).  In
      // the band between the two envelopes the row is kept — an ambiguous
      // row must survive to the reduced model rather than be resolved by
      // presolve on a tolerance boundary.
      if ((lhs > -kInf && lhs > tol_infeas) ||
          (rhs < kInf && rhs < -tol_infeas)) {
        return RuleOutcome::kInfeasible;
      }
      if ((lhs == -kInf || lhs <= tol_delete) &&
          (rhs == kInf || rhs >= -tol_delete)) {
        w.row_active[static_cast<std::size_t>(i)] = 0;
        ++counts.empty_rows;
      }
      continue;
    }

    if (degree == 1) {
      // Singleton row (A&A §2.3, eq. (2.4)): a_ij x_j must lie in
      // [lhs, rhs], which implies bounds on x_j whose direction depends on
      // the sign of a_ij.  The tightened column bounds make the row
      // redundant, so it is deleted; primal-only postsolve needs no record
      // because bound tightening is monotone.
      const int j = last_col;
      const double a = last_val;
      const double zero_tol = kZeroTol * std::max(1.0, row_abs_max);
      if (std::abs(a) > zero_tol) {
        double new_lb = -kInf;
        double new_ub = kInf;
        if (a > 0.0) {
          if (lhs > -kInf) new_lb = lhs / a;
          if (rhs < kInf) new_ub = rhs / a;
        } else {
          if (rhs < kInf) new_lb = rhs / a;
          if (lhs > -kInf) new_ub = lhs / a;
        }
        double& lo = w.lb[static_cast<std::size_t>(j)];
        double& hi = w.ub[static_cast<std::size_t>(j)];
        // An implied bound that excludes the current box by more than the
        // infeasibility envelope proves infeasibility (A&A §2.3).
        const bool lb_conflict =
            new_lb > -kInf && hi < kInf && new_lb > hi;
        const bool ub_conflict =
            new_ub < kInf && lo > -kInf && new_ub < lo;
        if ((lb_conflict && new_lb > hi + row_tol(hi, new_lb, kInfeasTol)) ||
            (ub_conflict && new_ub < lo - row_tol(lo, new_ub, kInfeasTol))) {
          return RuleOutcome::kInfeasible;
        }
        // A conflict inside the ambiguous band between the envelopes keeps
        // the row in the model instead of resolving it on a tolerance
        // boundary.
        const bool ambiguous =
            (lb_conflict && new_lb > hi + row_tol(hi, new_lb, kDeleteTol)) ||
            (ub_conflict && new_ub < lo - row_tol(lo, new_ub, kDeleteTol));
        if (!ambiguous) {
          // Apply every finite tighter implied bound, clamped to keep the
          // box valid; the row is then enforced exactly (up to one rounding
          // of rhs/a), so deleting it adds no audit-visible slack.  A bound
          // whose magnitude left the finite-side regime (|bound| >= 1e19,
          // the row/column no-bound sentinel convention) is not usable: if
          // such a bound would actually tighten the box, the row must stay
          // in the model — deleting it would silently drop the constraint.
          bool deletion_safe = true;
          if (new_lb > -kInf && finite_row_side(new_lb) && new_lb > lo) {
            lo = std::min(new_lb, hi);
          } else if (new_lb > -kInf && !finite_row_side(new_lb) &&
                     new_lb > lo) {
            deletion_safe = false;
          }
          if (new_ub < kInf && finite_row_side(new_ub) && new_ub < hi) {
            hi = std::max(new_ub, lo);
          } else if (new_ub < kInf && !finite_row_side(new_ub) &&
                     new_ub < hi) {
            deletion_safe = false;
          }
          if (deletion_safe) {
            w.row_active[static_cast<std::size_t>(i)] = 0;
            ++counts.singleton_rows;
            continue;
          }
          // Implied bound beyond the sentinel: keep the row and fall
          // through to the activity-based tests.
        }
        // Ambiguous singleton: keep the row and fall through to the
        // activity-based tests.
      }
      // A numerically zero singleton coefficient cannot produce a usable
      // implied bound; fall through to the activity-based tests.
    }

    // Redundant / infeasible row via activity bounds (A&A §2.3).
    // Infeasible: even the smallest attainable activity exceeds the upper
    // side, or the largest falls below the lower side — confirmed strictly
    // outside the infeasibility envelope (§5 R1).
    if (!min_inf && rhs < kInf && min_act > rhs + tol_infeas) {
      return RuleOutcome::kInfeasible;
    }
    if (!max_inf && lhs > -kInf && max_act < lhs - tol_infeas) {
      return RuleOutcome::kInfeasible;
    }
    // Redundant: the activity range lies inside the row sides, with only
    // the tighter deletion envelope of slack, so the deleted row can never
    // resurface above the original-model audit envelope at postsolve time.
    const bool lower_ok =
        (lhs == -kInf) || (!min_inf && min_act >= lhs - tol_delete);
    const bool upper_ok =
        (rhs == kInf) || (!max_inf && max_act <= rhs + tol_delete);
    if (lower_ok && upper_ok) {
      w.row_active[static_cast<std::size_t>(i)] = 0;
      ++counts.redundant_rows;
    }
  }
  return RuleOutcome::kOk;
}

/// P3 row-activity implied-bound propagation (A&A 1995 §3; Achterberg et
/// al. 2020 §6.4).  Finite sums plus counts of infinite contributions let
/// every residual activity [L_-j,U_-j] be obtained in O(1), hence O(nnz)
/// for the whole sweep rather than a quadratic scan per row.
RuleOutcome implied_bound_sweep(PresolveWork& w, RuleCounts& counts) {
  for (int i = 0; i < w.m; ++i) {
    if (!w.row_active[static_cast<std::size_t>(i)]) continue;
    const int rb = w.csr_start[static_cast<std::size_t>(i)];
    const int re = w.csr_start[static_cast<std::size_t>(i) + 1];
    double min_sum = 0.0;
    double max_sum = 0.0;
    int min_inf_count = 0;
    int max_inf_count = 0;
    bool min_overflow = false;
    bool max_overflow = false;

    for (int p = rb; p < re; ++p) {
      const int j = w.csr_col[static_cast<std::size_t>(p)];
      if (!w.col_active[static_cast<std::size_t>(j)]) continue;
      const double a = w.csr_val[static_cast<std::size_t>(p)];
      const double lo = w.lb[static_cast<std::size_t>(j)];
      const double hi = w.ub[static_cast<std::size_t>(j)];
      const double min_bound = a > 0.0 ? lo : hi;
      const double max_bound = a > 0.0 ? hi : lo;
      if (min_bound == -kInf || min_bound == kInf) {
        ++min_inf_count;
      } else {
        min_sum += a * min_bound;
        if (!std::isfinite(min_sum)) min_overflow = true;
      }
      if (max_bound == -kInf || max_bound == kInf) {
        ++max_inf_count;
      } else {
        max_sum += a * max_bound;
        if (!std::isfinite(max_sum)) max_overflow = true;
      }
    }

    const double lhs = w.lhs[static_cast<std::size_t>(i)];
    const double rhs = w.rhs[static_cast<std::size_t>(i)];
    for (int p = rb; p < re; ++p) {
      const int j = w.csr_col[static_cast<std::size_t>(p)];
      if (!w.col_active[static_cast<std::size_t>(j)]) continue;
      const double a = w.csr_val[static_cast<std::size_t>(p)];
      const double lo = w.lb[static_cast<std::size_t>(j)];
      const double hi = w.ub[static_cast<std::size_t>(j)];
      const double min_bound = a > 0.0 ? lo : hi;
      const double max_bound = a > 0.0 ? hi : lo;
      const bool own_min_inf = min_bound == -kInf || min_bound == kInf;
      const bool own_max_inf = max_bound == -kInf || max_bound == kInf;
      const bool residual_min_finite =
          !min_overflow && min_inf_count - static_cast<int>(own_min_inf) == 0;
      const bool residual_max_finite =
          !max_overflow && max_inf_count - static_cast<int>(own_max_inf) == 0;
      const double residual_min =
          residual_min_finite
              ? min_sum - (own_min_inf ? 0.0 : a * min_bound)
              : 0.0;
      const double residual_max =
          residual_max_finite
              ? max_sum - (own_max_inf ? 0.0 : a * max_bound)
              : 0.0;

      double candidate_lo = lo;
      double candidate_hi = hi;
      // Direct interval projection fixed in the P3 algorithm card:
      // a>0: lower=(lhs-U_-j)/a, upper=(rhs-L_-j)/a; a<0 swaps sides.
      if (a > 0.0) {
        if (lhs > -kInf && residual_max_finite)
          candidate_lo = std::max(candidate_lo, (lhs - residual_max) / a);
        if (rhs < kInf && residual_min_finite)
          candidate_hi = std::min(candidate_hi, (rhs - residual_min) / a);
      } else {
        if (rhs < kInf && residual_min_finite)
          candidate_lo = std::max(candidate_lo, (rhs - residual_min) / a);
        if (lhs > -kInf && residual_max_finite)
          candidate_hi = std::min(candidate_hi, (lhs - residual_max) / a);
      }
      if ((candidate_lo > -kInf && !finite_row_side(candidate_lo)) ||
          (candidate_hi < kInf && !finite_row_side(candidate_hi))) {
        continue;
      }
      const double conflict_tol =
          row_tol(candidate_lo, candidate_hi, kInfeasTol);
      if (candidate_lo > candidate_hi + conflict_tol)
        return RuleOutcome::kInfeasible;
      if (candidate_lo > candidate_hi) continue;

      double& current_lo = w.lb[static_cast<std::size_t>(j)];
      double& current_hi = w.ub[static_cast<std::size_t>(j)];
      const double lo_tol = kDeleteTol * std::max(1.0, std::abs(candidate_lo));
      const double hi_tol = kDeleteTol * std::max(1.0, std::abs(candidate_hi));
      if ((current_lo == -kInf && candidate_lo > -kInf) ||
          candidate_lo > current_lo + lo_tol) {
        current_lo = candidate_lo;
        ++counts.implied_bound_tightenings;
      }
      if ((current_hi == kInf && candidate_hi < kInf) ||
          candidate_hi < current_hi - hi_tol) {
        current_hi = candidate_hi;
        ++counts.implied_bound_tightenings;
      }
    }
  }
  return RuleOutcome::kOk;
}

// Substitution side-inflation cap (§6 P1 acceptance fix).  The wiring
// publishes the reduced solve at publication_tol_scale =
// clamp(0.9 * scale_orig/scale_reduced, 1e-4, 1.0) (see
// lp_presolve_publication_tol_scale); that mechanism only guarantees the
// original-model residual audit when the floor 1e-4 never binds ABOVE the
// true ratio, i.e. when scale_reduced <= 9e3 * scale_orig.
// Worse, the tightened target is attainable only while the required
// absolute precision (~1e-8 * scale_orig) stays decades above the linear
// algebra noise floor (~1e-13 * scale_reduced).  Capping substitution
// shifts at kSideShiftCap * scale_orig keeps both properties by
// construction; a column whose fixing would inflate a side past the cap
// simply stays in the reduced model (exactness is preserved either way —
// the postsolve stack just lacks that record).
constexpr double kSideShiftCap = 1e2;

// P2 substitution controls.  The pivot and fill-in constants match the
// HiGHS semantics recorded before implementation in design appendix A.2;
// their theoretical role is the Markowitz stability/complexity guard in
// A&A (1995) §2.4 and Suhl & Szymanski (1994).
constexpr double kSubstitutionPivotThreshold = 0.01;
constexpr long kSubstitutionMaxFillin = 10;
constexpr double kSubstitutionNnzGrowthLimit = 0.05;

/// Deterministic, mutable sparse adjacency used only while P2 substitutions
/// create fill-in.  P1 keeps the compact CSC/CSR scans; one conversion per
/// substitution sweep avoids imposing map overhead on the common P1 path.
struct MutableAdjacency {
  std::vector<std::map<int, double>> rows;
  std::vector<std::map<int, double>> cols;

  explicit MutableAdjacency(const PresolveWork& w)
      : rows(static_cast<std::size_t>(w.m)),
        cols(static_cast<std::size_t>(w.n)) {
    for (int j = 0; j < w.n; ++j) {
      if (!w.col_active[static_cast<std::size_t>(j)]) continue;
      for (int p = w.csc_start[static_cast<std::size_t>(j)];
           p < w.csc_start[static_cast<std::size_t>(j) + 1]; ++p) {
        const int i = w.csc_row[static_cast<std::size_t>(p)];
        if (!w.row_active[static_cast<std::size_t>(i)]) continue;
        const double v = w.csc_val[static_cast<std::size_t>(p)];
        rows[static_cast<std::size_t>(i)].emplace(j, v);
        cols[static_cast<std::size_t>(j)].emplace(i, v);
      }
    }
  }

  void set(int i, int j, double value) {
    auto& row = rows[static_cast<std::size_t>(i)];
    auto& col = cols[static_cast<std::size_t>(j)];
    if (value == 0.0) {
      row.erase(j);
      col.erase(i);
    } else {
      row[j] = value;
      col[i] = value;
    }
  }

  long nnz() const {
    long count = 0;
    for (const auto& row : rows) count += static_cast<long>(row.size());
    return count;
  }
};

long count_parallel_candidates(
    const std::vector<std::map<int, double>>& vectors) {
  using Signature = std::vector<std::pair<int, std::int64_t>>;
  std::map<Signature, long> groups;
  for (const auto& entries : vectors) {
    if (entries.empty()) continue;
    double max_abs = 0.0;
    for (const auto& [unused, value] : entries)
      max_abs = std::max(max_abs, std::abs(value));
    if (!(max_abs > 0.0) || !std::isfinite(max_abs)) continue;
    const double orientation = entries.begin()->second < 0.0 ? -1.0 : 1.0;
    Signature signature;
    signature.reserve(entries.size());
    for (const auto& [index, value] : entries) {
      // Diagnostic-only normalized hash (P4 pre-implementation survey).
      // Quantization at 1e-12 finds exact/near-exact parallel structure;
      // any future reduction must re-verify coefficients and sides before
      // adoption, per Achterberg et al. (2020) §5.
      const auto normalized = static_cast<std::int64_t>(
          std::llround(orientation * value / max_abs * 1e12));
      signature.emplace_back(index, normalized);
    }
    ++groups[std::move(signature)];
  }
  long candidates = 0;
  for (const auto& [unused, count] : groups)
    if (count > 1) candidates += count - 1;
  return candidates;
}

void commit_mutable_adjacency(PresolveWork& w, const MutableAdjacency& a) {
  w.csr_start.assign(static_cast<std::size_t>(w.m) + 1, 0);
  w.csr_col.clear();
  w.csr_val.clear();
  for (int i = 0; i < w.m; ++i) {
    w.csr_start[static_cast<std::size_t>(i)] =
        static_cast<int>(w.csr_col.size());
    for (const auto& [j, v] : a.rows[static_cast<std::size_t>(i)]) {
      w.csr_col.push_back(j);
      w.csr_val.push_back(v);
    }
  }
  w.csr_start[static_cast<std::size_t>(w.m)] =
      static_cast<int>(w.csr_col.size());

  w.csc_start.assign(static_cast<std::size_t>(w.n) + 1, 0);
  w.csc_row.clear();
  w.csc_val.clear();
  for (int j = 0; j < w.n; ++j) {
    w.csc_start[static_cast<std::size_t>(j)] =
        static_cast<int>(w.csc_row.size());
    for (const auto& [i, v] : a.cols[static_cast<std::size_t>(j)]) {
      w.csc_row.push_back(i);
      w.csc_val.push_back(v);
    }
  }
  w.csc_start[static_cast<std::size_t>(w.n)] =
      static_cast<int>(w.csc_row.size());
}

enum class SubstitutionOutcome { kRejected, kApplied, kInfeasible };

bool implied_free_in_equality(const PresolveWork& w,
                              const MutableAdjacency& a, int row, int col) {
  const double lo = w.lb[static_cast<std::size_t>(col)];
  const double hi = w.ub[static_cast<std::size_t>(col)];
  if (lo == -kInf && hi == kInf) return true;

  const auto& entries = a.rows[static_cast<std::size_t>(row)];
  const auto pivot_it = entries.find(col);
  if (pivot_it == entries.end() || pivot_it->second == 0.0) return false;
  const double pivot = pivot_it->second;
  double min_other = 0.0;
  double max_other = 0.0;
  for (const auto& [j, coef] : entries) {
    if (j == col) continue;
    const double jlo = w.lb[static_cast<std::size_t>(j)];
    const double jhi = w.ub[static_cast<std::size_t>(j)];
    const double low_term = coef > 0.0 ? jlo : jhi;
    const double high_term = coef > 0.0 ? jhi : jlo;
    if (low_term == -kInf || low_term == kInf || high_term == -kInf ||
        high_term == kInf) {
      return false;
    }
    min_other += coef * low_term;
    max_other += coef * high_term;
    if (!std::isfinite(min_other) || !std::isfinite(max_other)) return false;
  }
  double implied_lo = (w.rhs[static_cast<std::size_t>(row)] - max_other) / pivot;
  double implied_hi = (w.rhs[static_cast<std::size_t>(row)] - min_other) / pivot;
  if (pivot < 0.0) std::swap(implied_lo, implied_hi);
  const double tol = row_tol(lo, hi, kDeleteTol);
  return (lo == -kInf || implied_lo >= lo - tol) &&
         (hi == kInf || implied_hi <= hi + tol);
}

bool stable_substitution_pivot(const MutableAdjacency& a, int row, int col) {
  const auto& r = a.rows[static_cast<std::size_t>(row)];
  const auto& c = a.cols[static_cast<std::size_t>(col)];
  if (r.size() == 2 || c.size() == 2) return true;
  const double pivot = std::abs(r.at(col));
  double row_max = 0.0;
  double col_max = 0.0;
  for (const auto& [unused, v] : r) row_max = std::max(row_max, std::abs(v));
  for (const auto& [unused, v] : c) col_max = std::max(col_max, std::abs(v));
  return pivot >= kSubstitutionPivotThreshold * std::max(row_max, col_max);
}

SubstitutionOutcome substitute_singleton_equality_column(
    PresolveWork& w, MutableAdjacency& a, int row, int col,
    RuleCounts& counts, std::vector<LpPostsolveRecord>& stack,
    double& objective_offset_min, double side_cap) {
  const auto row_snapshot = a.rows[static_cast<std::size_t>(row)];
  const auto pivot_it = row_snapshot.find(col);
  if (pivot_it == row_snapshot.end() || pivot_it->second == 0.0 ||
      a.cols[static_cast<std::size_t>(col)].size() != 1) {
    return SubstitutionOutcome::kRejected;
  }
  const double pivot = pivot_it->second;
  const double equality_rhs = w.rhs[static_cast<std::size_t>(row)];
  const double offset = equality_rhs / pivot;
  if (!std::isfinite(offset)) return SubstitutionOutcome::kRejected;

  // Project the singleton column's box through
  //   a_j*x_j + s = rhs, x_j in [l_j,u_j]
  // to obtain rhs-max(a_j*l_j,a_j*u_j) <= s <=
  // rhs-min(a_j*l_j,a_j*u_j).  This is the bounded singleton-column form
  // of A&A (1995) §2.4, derived and fixed before implementation in the P2
  // mismatch record of the design document.
  const double lo = w.lb[static_cast<std::size_t>(col)];
  const double hi = w.ub[static_cast<std::size_t>(col)];
  double product_lo = -kInf;
  double product_hi = kInf;
  if (pivot > 0.0) {
    if (lo > -kInf) product_lo = pivot * lo;
    if (hi < kInf) product_hi = pivot * hi;
  } else {
    if (hi < kInf) product_lo = pivot * hi;
    if (lo > -kInf) product_hi = pivot * lo;
  }
  if ((product_lo > -kInf && !std::isfinite(product_lo)) ||
      (product_hi < kInf && !std::isfinite(product_hi))) {
    return SubstitutionOutcome::kRejected;
  }
  const double projected_lhs =
      product_hi == kInf ? -kInf : equality_rhs - product_hi;
  const double projected_rhs =
      product_lo == -kInf ? kInf : equality_rhs - product_lo;
  if ((projected_lhs > -kInf &&
       (!finite_row_side(projected_lhs) ||
        std::abs(projected_lhs) > side_cap)) ||
      (projected_rhs < kInf &&
       (!finite_row_side(projected_rhs) ||
        std::abs(projected_rhs) > side_cap))) {
    return SubstitutionOutcome::kRejected;
  }

  const double subst_cost = w.c[static_cast<std::size_t>(col)];
  const double next_offset = objective_offset_min + subst_cost * offset;
  if (!std::isfinite(next_offset)) return SubstitutionOutcome::kRejected;
  std::map<int, double> scales;
  for (const auto& [j, coef] : row_snapshot) {
    if (j == col) continue;
    const double scale = -coef / pivot;
    const double next_cost = w.c[static_cast<std::size_t>(j)] +
                             subst_cost * scale;
    if (!std::isfinite(scale) || !std::isfinite(next_cost))
      return SubstitutionOutcome::kRejected;
    scales.emplace(j, scale);
  }

  LpPostsolveFreeColSubstitution rec;
  rec.col = col;
  rec.rhs = equality_rhs;
  rec.pivot = pivot;
  for (const auto& [j, coef] : row_snapshot)
    if (j != col) rec.row_entries.emplace_back(j, coef);
  stack.emplace_back(std::move(rec));

  objective_offset_min = next_offset;
  for (const auto& [j, scale] : scales)
    w.c[static_cast<std::size_t>(j)] += subst_cost * scale;
  w.c[static_cast<std::size_t>(col)] = 0.0;
  a.set(row, col, 0.0);
  w.col_active[static_cast<std::size_t>(col)] = 0;
  w.lhs[static_cast<std::size_t>(row)] = projected_lhs;
  w.rhs[static_cast<std::size_t>(row)] = projected_rhs;
  if (projected_lhs == -kInf && projected_rhs == kInf) {
    for (const auto& [j, unused] : row_snapshot)
      if (j != col) a.set(row, j, 0.0);
    w.row_active[static_cast<std::size_t>(row)] = 0;
  }
  ++counts.singleton_columns;
  return SubstitutionOutcome::kApplied;
}

SubstitutionOutcome substitute_equality_column(
    PresolveWork& w, MutableAdjacency& a, int row, int subst_col,
    RuleCounts& counts, std::vector<LpPostsolveRecord>& stack,
    double& objective_offset_min, double side_cap, long nnz_limit,
    bool doubleton) {
  const auto row_snapshot = a.rows[static_cast<std::size_t>(row)];
  const auto pivot_it = row_snapshot.find(subst_col);
  if (pivot_it == row_snapshot.end() || pivot_it->second == 0.0) {
    return SubstitutionOutcome::kRejected;
  }
  const double pivot = pivot_it->second;
  const double rhs = w.rhs[static_cast<std::size_t>(row)];
  const double offset = rhs / pivot;
  if (!std::isfinite(offset)) return SubstitutionOutcome::kRejected;

  std::map<int, double> scales;
  for (const auto& [j, coef] : row_snapshot) {
    if (j == subst_col) continue;
    const double scale = -coef / pivot;
    if (!std::isfinite(scale)) return SubstitutionOutcome::kRejected;
    scales.emplace(j, scale);
  }

  // A doubleton equation also transfers the eliminated column's explicit
  // bounds to the staying column (A&A 1995 §2.4; design appendix A.1).
  if (doubleton) {
    const int stay_col = scales.begin()->first;
    const double scale = scales.begin()->second;
    double implied_lo = -kInf;
    double implied_hi = kInf;
    const double subst_lo = w.lb[static_cast<std::size_t>(subst_col)];
    const double subst_hi = w.ub[static_cast<std::size_t>(subst_col)];
    if (scale > 0.0) {
      if (subst_lo > -kInf) implied_lo = (subst_lo - offset) / scale;
      if (subst_hi < kInf) implied_hi = (subst_hi - offset) / scale;
    } else {
      if (subst_hi < kInf) implied_lo = (subst_hi - offset) / scale;
      if (subst_lo > -kInf) implied_hi = (subst_lo - offset) / scale;
    }
    double& stay_lo = w.lb[static_cast<std::size_t>(stay_col)];
    double& stay_hi = w.ub[static_cast<std::size_t>(stay_col)];
    const double candidate_lo = std::max(stay_lo, implied_lo);
    const double candidate_hi = std::min(stay_hi, implied_hi);
    if (candidate_lo > candidate_hi +
                               row_tol(candidate_lo, candidate_hi, kInfeasTol)) {
      return SubstitutionOutcome::kInfeasible;
    }
    if (candidate_lo > candidate_hi) return SubstitutionOutcome::kRejected;
    if ((candidate_lo > -kInf && !finite_row_side(candidate_lo)) ||
        (candidate_hi < kInf && !finite_row_side(candidate_hi))) {
      return SubstitutionOutcome::kRejected;
    }
    stay_lo = candidate_lo;
    stay_hi = candidate_hi;
  }

  std::vector<int> affected_rows;
  for (const auto& [i, unused] : a.cols[static_cast<std::size_t>(subst_col)])
    if (i != row) affected_rows.push_back(i);

  long projected_nnz = a.nnz() - static_cast<long>(row_snapshot.size());
  long fillin = 0;
  for (const int i : affected_rows) {
    const auto& target = a.rows[static_cast<std::size_t>(i)];
    const double multiplier = target.at(subst_col);
    long new_size = static_cast<long>(target.size()) - 1;
    for (const auto& [j, scale] : scales) {
      const auto old = target.find(j);
      const double value = (old == target.end() ? 0.0 : old->second) +
                           multiplier * scale;
      if (!std::isfinite(value)) return SubstitutionOutcome::kRejected;
      if (old == target.end() && value != 0.0) {
        ++new_size;
        ++fillin;
      } else if (old != target.end() && value == 0.0) {
        --new_size;
      }
    }
    const long old_size = static_cast<long>(target.size());
    const long row_limit = std::max(
        old_size + static_cast<long>(affected_rows.size()) - 1,
        2 * old_size);
    if (new_size > row_limit) return SubstitutionOutcome::kRejected;
    projected_nnz += new_size - old_size;

    const double shift = multiplier * offset;
    if (!std::isfinite(shift)) return SubstitutionOutcome::kRejected;
    const double lhs = w.lhs[static_cast<std::size_t>(i)];
    const double upper = w.rhs[static_cast<std::size_t>(i)];
    if ((lhs > -kInf && (!finite_row_side(lhs - shift) ||
                         std::abs(lhs - shift) > side_cap)) ||
        (upper < kInf && (!finite_row_side(upper - shift) ||
                          std::abs(upper - shift) > side_cap))) {
      return SubstitutionOutcome::kRejected;
    }
  }
  if (!doubleton && fillin > kSubstitutionMaxFillin)
    return SubstitutionOutcome::kRejected;
  if (projected_nnz > nnz_limit) return SubstitutionOutcome::kRejected;

  const double subst_cost = w.c[static_cast<std::size_t>(subst_col)];
  const double next_offset = objective_offset_min + subst_cost * offset;
  if (!std::isfinite(next_offset)) return SubstitutionOutcome::kRejected;
  for (const auto& [j, scale] : scales) {
    const double next_cost = w.c[static_cast<std::size_t>(j)] + subst_cost * scale;
    if (!std::isfinite(next_cost)) return SubstitutionOutcome::kRejected;
  }

  if (doubleton) {
    const auto [stay_col, scale] = *scales.begin();
    stack.push_back(LpPostsolveDoubletonEquation{
        subst_col, stay_col, pivot, -scale * pivot, rhs});
    ++counts.doubleton_equations;
  } else {
    LpPostsolveFreeColSubstitution rec;
    rec.col = subst_col;
    rec.rhs = rhs;
    rec.pivot = pivot;
    for (const auto& [j, coef] : row_snapshot)
      if (j != subst_col) rec.row_entries.emplace_back(j, coef);
    stack.emplace_back(std::move(rec));
    if (a.cols[static_cast<std::size_t>(subst_col)].size() == 1)
      ++counts.singleton_columns;
    else
      ++counts.free_column_substitutions;
  }

  objective_offset_min = next_offset;
  for (const auto& [j, scale] : scales)
    w.c[static_cast<std::size_t>(j)] += subst_cost * scale;
  w.c[static_cast<std::size_t>(subst_col)] = 0.0;

  for (const int i : affected_rows) {
    const double multiplier = a.rows[static_cast<std::size_t>(i)].at(subst_col);
    const double shift = multiplier * offset;
    if (w.lhs[static_cast<std::size_t>(i)] > -kInf)
      w.lhs[static_cast<std::size_t>(i)] -= shift;
    if (w.rhs[static_cast<std::size_t>(i)] < kInf)
      w.rhs[static_cast<std::size_t>(i)] -= shift;
    a.set(i, subst_col, 0.0);
    for (const auto& [j, scale] : scales) {
      const auto old = a.rows[static_cast<std::size_t>(i)].find(j);
      const double old_value =
          old == a.rows[static_cast<std::size_t>(i)].end() ? 0.0 : old->second;
      a.set(i, j, old_value + multiplier * scale);
    }
  }
  for (const auto& [j, unused] : row_snapshot) a.set(row, j, 0.0);
  w.row_active[static_cast<std::size_t>(row)] = 0;
  w.col_active[static_cast<std::size_t>(subst_col)] = 0;
  return SubstitutionOutcome::kApplied;
}

RuleOutcome substitution_sweep(
    PresolveWork& w, RuleCounts& counts,
    std::vector<LpPostsolveRecord>& stack, double& objective_offset_min,
    double side_cap, long nnz_limit,
    const std::chrono::steady_clock::time_point& deadline, bool& timed_out) {
  MutableAdjacency a(w);
  if (!counts.p2_structure_sampled) {
    counts.p2_structure_sampled = true;
    for (int j = 0; j < w.n; ++j) {
      if (!w.col_active[static_cast<std::size_t>(j)] ||
          a.cols[static_cast<std::size_t>(j)].size() != 1) {
        continue;
      }
      ++counts.active_singleton_col_candidates;
      const int i = a.cols[static_cast<std::size_t>(j)].begin()->first;
      if (w.lhs[static_cast<std::size_t>(i)] ==
          w.rhs[static_cast<std::size_t>(i)]) {
        ++counts.singleton_equality_col_candidates;
      } else if (w.lhs[static_cast<std::size_t>(i)] > -kInf &&
                 w.rhs[static_cast<std::size_t>(i)] < kInf) {
        ++counts.singleton_ranged_col_candidates;
      }
    }
    for (int i = 0; i < w.m; ++i) {
      if (!w.row_active[static_cast<std::size_t>(i)] ||
          w.lhs[static_cast<std::size_t>(i)] !=
              w.rhs[static_cast<std::size_t>(i)]) {
        continue;
      }
      if (a.rows[static_cast<std::size_t>(i)].size() == 2)
        ++counts.doubleton_equality_row_candidates;
      for (const auto& [j, unused] : a.rows[static_cast<std::size_t>(i)]) {
        if (w.lb[static_cast<std::size_t>(j)] == -kInf &&
            w.ub[static_cast<std::size_t>(j)] == kInf) {
          ++counts.free_equality_col_candidates;
        } else if (implied_free_in_equality(w, a, i, j)) {
          ++counts.implied_free_equality_col_candidates;
        }
      }
    }
    counts.parallel_row_candidates = count_parallel_candidates(a.rows);
    counts.parallel_col_candidates = count_parallel_candidates(a.cols);
  }
  bool changed = true;
  while (changed) {
    changed = false;
    if (std::chrono::steady_clock::now() > deadline) {
      timed_out = true;
      return RuleOutcome::kOk;
    }

    // A singleton equality column can be eliminated even with explicit
    // bounds by projecting those bounds onto the equality's remaining
    // activity (P2 mismatch derivation; A&A 1995 §2.4).
    for (int j = 0; j < w.n && !changed; ++j) {
      if (!w.col_active[static_cast<std::size_t>(j)] ||
          a.cols[static_cast<std::size_t>(j)].size() != 1) {
        continue;
      }
      const int i = a.cols[static_cast<std::size_t>(j)].begin()->first;
      if (!w.row_active[static_cast<std::size_t>(i)] ||
          w.lhs[static_cast<std::size_t>(i)] !=
              w.rhs[static_cast<std::size_t>(i)] ||
          a.rows[static_cast<std::size_t>(i)].size() < 2) {
        continue;
      }
      const auto outcome = substitute_singleton_equality_column(
          w, a, i, j, counts, stack, objective_offset_min, side_cap);
      if (outcome == SubstitutionOutcome::kInfeasible)
        return RuleOutcome::kInfeasible;
      changed = outcome == SubstitutionOutcome::kApplied;
    }
    if (changed) continue;

    // Doubleton equalities next (design appendix A.1): choose the sparser
    // column when coefficients are within a factor two, otherwise eliminate
    // through the larger pivot for stability.
    for (int i = 0; i < w.m && !changed; ++i) {
      if (!w.row_active[static_cast<std::size_t>(i)] ||
          w.lhs[static_cast<std::size_t>(i)] !=
              w.rhs[static_cast<std::size_t>(i)] ||
          a.rows[static_cast<std::size_t>(i)].size() != 2) {
        continue;
      }
      auto first = a.rows[static_cast<std::size_t>(i)].begin();
      auto second = std::next(first);
      int subst = first->first;
      const double ratio = std::max(std::abs(first->second),
                                    std::abs(second->second)) /
                           std::min(std::abs(first->second),
                                    std::abs(second->second));
      if (a.cols[static_cast<std::size_t>(first->first)].size() == 1 ||
          (ratio <= 2.0 &&
           a.cols[static_cast<std::size_t>(first->first)].size() <
               a.cols[static_cast<std::size_t>(second->first)].size()) ||
          (ratio > 2.0 && std::abs(first->second) > std::abs(second->second))) {
        subst = first->first;
      } else {
        subst = second->first;
      }
      const auto outcome = substitute_equality_column(
          w, a, i, subst, counts, stack, objective_offset_min, side_cap,
          nnz_limit, true);
      if (outcome == SubstitutionOutcome::kInfeasible)
        return RuleOutcome::kInfeasible;
      changed = outcome == SubstitutionOutcome::kApplied;
    }
    if (changed) continue;

    // General free/implied-free equality substitution (A&A 1995 §2.4;
    // design appendix A.2).  Equality rows are dual-implied-free by
    // definition; explicit or activity-certified implied freedom supplies
    // the primal condition.
    for (int i = 0; i < w.m && !changed; ++i) {
      if (!w.row_active[static_cast<std::size_t>(i)] ||
          w.lhs[static_cast<std::size_t>(i)] !=
              w.rhs[static_cast<std::size_t>(i)] ||
          a.rows[static_cast<std::size_t>(i)].size() < 3) {
        continue;
      }
      for (const auto& [j, unused] : a.rows[static_cast<std::size_t>(i)]) {
        if (!implied_free_in_equality(w, a, i, j) ||
            !stable_substitution_pivot(a, i, j)) {
          continue;
        }
        const auto outcome = substitute_equality_column(
            w, a, i, j, counts, stack, objective_offset_min, side_cap,
            nnz_limit, false);
        if (outcome == SubstitutionOutcome::kInfeasible)
          return RuleOutcome::kInfeasible;
        if (outcome == SubstitutionOutcome::kApplied) {
          changed = true;
          break;
        }
      }
    }
  }
  commit_mutable_adjacency(w, a);
  return RuleOutcome::kOk;
}

/// Column pass: empty columns and fixed columns (A&A §2.2).
RuleOutcome col_sweep(PresolveWork& w, RuleCounts& counts,
                      std::vector<LpPostsolveRecord>& stack,
                      double& objective_offset_min, double side_cap) {
  for (int j = 0; j < w.n; ++j) {
    if (!w.col_active[static_cast<std::size_t>(j)]) continue;
    const int cb = w.csc_start[static_cast<std::size_t>(j)];
    const int ce = w.csc_start[static_cast<std::size_t>(j) + 1];
    int degree = 0;
    for (int p = cb; p < ce; ++p)
      if (w.row_active[static_cast<std::size_t>(w.csc_row[p])]) ++degree;

    const double cj = w.c[static_cast<std::size_t>(j)];
    const double lo = w.lb[static_cast<std::size_t>(j)];
    const double hi = w.ub[static_cast<std::size_t>(j)];

    if (degree == 0) {
      // Empty column (A&A §2.2): unconstrained by any remaining row, so the
      // optimal value is a bound chosen by the cost sign.
      double value;
      if (cj == 0.0) {
        // Zero cost: any feasible point is optimal; take the lower bound,
        // else the upper bound, else 0 (all three are feasible).
        value = (lo > -kInf) ? lo : (hi < kInf ? hi : 0.0);
      } else if (cj > 0.0) {
        // Minimization with positive cost pushes x_j to its lower bound.
        if (lo == -kInf) return RuleOutcome::kUnboundedCandidate;
        value = lo;
      } else {
        if (hi == kInf) return RuleOutcome::kUnboundedCandidate;
        value = hi;
      }
      objective_offset_min += cj * value;
      stack.push_back(LpPostsolveFixedCol{j, value});
      w.col_active[static_cast<std::size_t>(j)] = 0;
      ++counts.empty_cols;
      continue;
    }

    // Fixed column (A&A §2.2): lb == ub forces x_j = lb; substitute the
    // value into every row (shifting the row sides), add c_j x_j to the
    // objective offset, and delete the column.
    if (lo > -kInf && hi < kInf &&
        hi - lo <= kFixedTol * std::max({1.0, std::abs(lo), std::abs(hi)})) {
      const double value = 0.5 * (lo + hi);
      // Sentinel/overflow guard (§5 R1, audit convention |side| >= 1e19 is
      // no bound): a substitution that pushes a finite row side out of the
      // finite-side regime would silently turn a bounded row into a
      // one-sided/free row in the reduced model AND in the original-model
      // residual audit, and a non-finite a*v would do the same through
      // +/-inf.  The side_cap clause additionally keeps the reduced side
      // scale within kSideShiftCap of the original one (see the constant's
      // comment).  Skip the reduction in those cases — the column stays
      // and the rows keep their exact sides.
      bool substitution_safe = true;
      for (int p = cb; p < ce; ++p) {
        const int i = w.csc_row[static_cast<std::size_t>(p)];
        if (!w.row_active[static_cast<std::size_t>(i)]) continue;
        const double av = w.csc_val[static_cast<std::size_t>(p)] * value;
        if (!std::isfinite(av)) {
          substitution_safe = false;
          break;
        }
        const double lhs = w.lhs[static_cast<std::size_t>(i)];
        const double rhs = w.rhs[static_cast<std::size_t>(i)];
        if ((lhs > -kInf && (!finite_row_side(lhs - av) ||
                             std::abs(lhs - av) > side_cap)) ||
            (rhs < kInf && (!finite_row_side(rhs - av) ||
                            std::abs(rhs - av) > side_cap))) {
          substitution_safe = false;
          break;
        }
      }
      if (substitution_safe) {
        objective_offset_min += cj * value;
        for (int p = cb; p < ce; ++p) {
          const int i = w.csc_row[static_cast<std::size_t>(p)];
          if (!w.row_active[static_cast<std::size_t>(i)]) continue;
          const double av = w.csc_val[static_cast<std::size_t>(p)] * value;
          double& lhs = w.lhs[static_cast<std::size_t>(i)];
          double& rhs = w.rhs[static_cast<std::size_t>(i)];
          if (lhs > -kInf) lhs -= av;
          if (rhs < kInf) rhs -= av;
        }
        stack.push_back(LpPostsolveFixedCol{j, value});
        w.col_active[static_cast<std::size_t>(j)] = 0;
        ++counts.fixed_cols;
      }
    }
  }
  return RuleOutcome::kOk;
}

}  // namespace

double lp_presolve_publication_tol_scale(const LPModel& orig,
                                         const LPModel& reduced) {
  // Global side scale, same definition as the original-model residual audits
  // (audit_ipm_lp_optimality / lp_solution_residual_acceptable): max over
  // finite inequality sides below the |side| >= 1e19 no-bound sentinel plus
  // every equality RHS, floored at 1.
  auto side_scale = [](const LPModel& m) {
    constexpr double kSideSentinel = 1e19;
    double s = 1.0;
    for (int i = 0; i < m.b.size(); ++i)
      if (std::abs(m.b[i]) < kSideSentinel)
        s = std::max(s, std::abs(m.b[i]));
    if (lp_has_row_lhs(m))
      for (int i = 0; i < m.row_lhs.size(); ++i)
        if (std::isfinite(m.row_lhs[i]) &&
            std::abs(m.row_lhs[i]) < kSideSentinel)
          s = std::max(s, std::abs(m.row_lhs[i]));
    // Equality RHS values are always real sides; unlike inequality bounds,
    // they have no |side| >= 1e19 "no bound" sentinel representation.
    for (int i = 0; i < m.beq.size(); ++i)
      s = std::max(s, std::abs(m.beq[i]));
    return s;
  };
  const double scale_orig = side_scale(orig);
  const double scale_reduced = side_scale(reduced);
  // Correctness invariant (native_presolve_lp_2026-08-18.md §6, P1 second
  // mismatch round): the wiring publishes the reduced solve at
  // scale * max(1e-10, 10*tol) relative to the REDUCED global side scale and
  // then audits the postsolved point at max(1e-10, 10*tol) relative to the
  // ORIGINAL global side scale.  P1 postsolve is exact for surviving
  // columns, so a surviving row carries the identical residual in both
  // models; the transfer is therefore guaranteed iff
  //     scale <= scale_orig / scale_reduced.
  // Deleted rows are independent of this scale: their deletion slack is
  // bounded by kDeleteTol = 1e-8 at presolve time, one decade inside the
  // audit envelope, whatever the reduced solve's tolerance.
  // The 0.9 margin against the exact boundary absorbs the last-ulp
  // difference (~1e-13 relative) between the kernel's reduced-model
  // residual evaluation and the wiring's original-model re-evaluation with
  // ~12 orders of safety, while tightening the reduced publication target
  // by at most ~0.05 decades versus a direct solve.  (The previous
  // unconditional one-decade factor demanded a 1e-8-relative publish on
  // models with zero side inflation — modszk1's normal-equation trajectory
  // stalls with primal residual pinned at 1.38e-8 under that target, and
  // the escalation ladder then burns seconds on augmented grinds, §6.)
  // No positive clamp floor is admissible here: the HiGHS reduction bridge
  // can produce a side ratio below the native kSideShiftCap domain, and any
  // floor above the ratio breaks the transfer invariant.  Both scales are in
  // [1, 1e19), so the quotient is finite and far above FP64 underflow.
  return std::min(0.9 * scale_orig / scale_reduced, 1.0);
}

LpPresolveConfig lp_presolve_config_from_env(LpPresolveConfig base) {
  if (const char* e = std::getenv("MIPSOLVERS_NATIVE_PRESOLVE")) {
    base.enabled = !(e[0] == '0' && e[1] == '\0');
  }
  if (const char* e = std::getenv("MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX")) {
    char* end = nullptr;
    const double v = std::strtod(e, &end);
    if (end != e) base.time_box_sec = v;
  }
  if (std::getenv("MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE")) base.verbose = true;
  return base;
}

Eigen::VectorXd postsolve_primal(const LpPresolveResult& result,
                                 const Eigen::VectorXd& x_reduced) {
  if (!result.use_reduced ||
      x_reduced.size() != result.reduced.c.size()) {
    return Eigen::VectorXd();
  }
  const int n = static_cast<int>(result.orig_to_reduced_col.size());
  Eigen::VectorXd x(n);
  for (int j = 0; j < n; ++j) {
    const int r = result.orig_to_reduced_col[static_cast<std::size_t>(j)];
    x[j] = r >= 0 ? x_reduced[r] : 0.0;
  }
  // Reverse-order undo of the reduction stack (§2.1 HighsPostsolveStack
  // protocol).  P1 records are all FixedCol; later phases add record types
  // whose undo reads other already-recovered components, which is why the
  // undo runs after the active-column fill.
  for (auto it = result.postsolve_stack.rbegin();
       it != result.postsolve_stack.rend(); ++it) {
    std::visit(
        [&x](const auto& rec) {
          using T = std::decay_t<decltype(rec)>;
          if constexpr (std::is_same_v<T, LpPostsolveFixedCol>) {
            x[rec.orig_col] = rec.value;
          } else if constexpr (
              std::is_same_v<T, LpPostsolveDoubletonEquation>) {
            x[rec.subst_col] =
                (rec.rhs - rec.coef_stay * x[rec.stay_col]) /
                rec.coef_subst;
          } else if constexpr (
              std::is_same_v<T, LpPostsolveFreeColSubstitution>) {
            double activity = 0.0;
            for (const auto& [j, coef] : rec.row_entries)
              activity += coef * x[j];
            x[rec.col] = (rec.rhs - activity) / rec.pivot;
          }
        },
        *it);
  }
  return x;
}

LpPresolveResult lp_presolve_run(const LPModel& lp,
                                 const LpPresolveConfig& cfg) {
  const auto t0 = std::chrono::steady_clock::now();
  LpPresolveResult out;
  // Row/col/nnz accounting mirrors highs_presolve_lp so the telemetry of
  // the two presolve arms is directly comparable (§2.4-A2 anchor).
  out.orig_rows =
      static_cast<long>(lp.A.rows()) + static_cast<long>(lp.Aeq.rows());
  out.orig_cols = static_cast<long>(lp.c.size());
  out.orig_nnz = lp.A.nonZeros() + lp.Aeq.nonZeros();
  out.status = "disabled";

  auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  if (!cfg.enabled) {
    out.presolve_ms = elapsed_ms();
    return out;
  }

  PresolveWork w;
  if (!build_work(lp, w)) {
    // Non-finite model data: no reduction is defensible; the caller solves
    // the original model and its audits decide (fail-loud, §5 R1).
    out.reduced_rows = out.orig_rows;
    out.reduced_cols = out.orig_cols;
    out.reduced_nnz = out.orig_nnz;
    out.status = "invalid_input";
    out.presolve_ms = elapsed_ms();
    return out;
  }

  RuleCounts counts;
  double objective_offset_min = 0.0;
  int rounds = 0;
  const double time_box_ms = 1000.0 * std::max(0.0, cfg.time_box_sec);
  // Original global side scale (same definition as the original-model
  // residual audit: max over finite sides, floored at 1).  Fixed-column
  // substitution shifts are capped at kSideShiftCap times this scale.
  double orig_side_scale = 1.0;
  for (int i = 0; i < w.m; ++i) {
    const double lhs = w.lhs[static_cast<std::size_t>(i)];
    const double rhs = w.rhs[static_cast<std::size_t>(i)];
    if (lhs > -kInf) orig_side_scale = std::max(orig_side_scale, std::abs(lhs));
    if (rhs < kInf) orig_side_scale = std::max(orig_side_scale, std::abs(rhs));
  }
  const double side_cap = kSideShiftCap * orig_side_scale;
  const long substitution_nnz_limit = static_cast<long>(std::ceil(
      (1.0 + kSubstitutionNnzGrowthLimit) * static_cast<double>(out.orig_nnz)));
  const auto deadline = t0 + std::chrono::duration_cast<
                                 std::chrono::steady_clock::duration>(
                                 std::chrono::duration<double>(
                                     std::max(0.0, cfg.time_box_sec)));

  // Fixpoint over the P1 rule set (§4): a round with no reduction ends the
  // loop.  Termination is guaranteed because every counted rule deletes a
  // row or a column, so there are at most m + n productive rounds.
  for (;;) {
    if (elapsed_ms() > time_box_ms) {
      // Time box exceeded (§5 R5): return the unreduced model; the caller
      // continues on its pre-existing solve path.
      out.reduced_rows = out.orig_rows;
      out.reduced_cols = out.orig_cols;
      out.reduced_nnz = out.orig_nnz;
      out.status = "time_box";
      out.presolve_ms = elapsed_ms();
      if (cfg.verbose) {
        std::fprintf(stderr,
                     "[NATIVE-PRESOLVE] %s: rows %ld->%ld cols %ld->%ld nnz "
                     "%ld->%ld use_reduced=%d %.1fms\n",
                     out.status.c_str(), out.orig_rows, out.reduced_rows,
                     out.orig_cols, out.reduced_cols, out.orig_nnz,
                     out.reduced_nnz, static_cast<int>(out.use_reduced),
                     out.presolve_ms);
      }
      return out;
    }
    ++rounds;
    const long before = counts.total();
    const RuleOutcome row_outcome = row_sweep(w, counts);
    if (row_outcome != RuleOutcome::kOk) {
      out.infeasible = row_outcome == RuleOutcome::kInfeasible;
      out.unbounded_candidate =
          row_outcome == RuleOutcome::kUnboundedCandidate;
      out.status = out.infeasible ? "infeasible" : "unbounded_candidate";
      out.presolve_ms = elapsed_ms();
      return out;
    }
    const RuleOutcome bound_outcome =
        cfg.propagate_bounds ? implied_bound_sweep(w, counts)
                             : RuleOutcome::kOk;
    if (bound_outcome != RuleOutcome::kOk) {
      out.infeasible = bound_outcome == RuleOutcome::kInfeasible;
      out.status = out.infeasible ? "infeasible" : "unbounded_candidate";
      out.presolve_ms = elapsed_ms();
      return out;
    }
    const RuleOutcome col_outcome =
        col_sweep(w, counts, out.postsolve_stack, objective_offset_min,
                  side_cap);
    if (col_outcome != RuleOutcome::kOk) {
      out.infeasible = col_outcome == RuleOutcome::kInfeasible;
      out.unbounded_candidate =
          col_outcome == RuleOutcome::kUnboundedCandidate;
      out.status = out.infeasible ? "infeasible" : "unbounded_candidate";
      out.presolve_ms = elapsed_ms();
      return out;
    }
    bool substitution_timed_out = false;
    const RuleOutcome substitution_outcome =
        cfg.substitutions
            ? substitution_sweep(w, counts, out.postsolve_stack,
                                 objective_offset_min, side_cap,
                                 substitution_nnz_limit, deadline,
                                 substitution_timed_out)
            : RuleOutcome::kOk;
    if (substitution_timed_out) {
      out.reduced_rows = out.orig_rows;
      out.reduced_cols = out.orig_cols;
      out.reduced_nnz = out.orig_nnz;
      out.status = "time_box";
      out.presolve_ms = elapsed_ms();
      if (cfg.verbose) {
        std::fprintf(stderr,
                     "[NATIVE-PRESOLVE] %s: rows %ld->%ld cols %ld->%ld nnz "
                     "%ld->%ld use_reduced=%d %.1fms\n",
                     out.status.c_str(), out.orig_rows, out.reduced_rows,
                     out.orig_cols, out.reduced_cols, out.orig_nnz,
                     out.reduced_nnz, static_cast<int>(out.use_reduced),
                     out.presolve_ms);
      }
      return out;
    }
    if (substitution_outcome != RuleOutcome::kOk) {
      out.infeasible = substitution_outcome == RuleOutcome::kInfeasible;
      out.status = out.infeasible ? "infeasible" : "unbounded_candidate";
      out.presolve_ms = elapsed_ms();
      return out;
    }
    if (counts.total() == before) break;
  }

  if (counts.total() == 0) {
    // No reduction: the caller's solve is bit-identical to the pre-presolve
    // path (gate G4, §4).  Dimensions are still reported so reduced-vs-
    // original ratios are well-defined telemetry.
    out.reduced_rows = out.orig_rows;
    out.reduced_cols = out.orig_cols;
    out.reduced_nnz = out.orig_nnz;
    out.status = "no_reduction";
    out.presolve_ms = elapsed_ms();
    if (cfg.verbose) {
      std::fprintf(stderr,
                   "[NATIVE-PRESOLVE] p2_candidates: singleton_cols=%ld "
                   "singleton_eq=%ld singleton_ranged=%ld doubleton_eq=%ld "
                   "free_eq_cols=%ld implied_free_eq_cols=%ld\n",
                   counts.active_singleton_col_candidates,
                   counts.singleton_equality_col_candidates,
                   counts.singleton_ranged_col_candidates,
                   counts.doubleton_equality_row_candidates,
                   counts.free_equality_col_candidates,
                   counts.implied_free_equality_col_candidates);
      std::fprintf(stderr,
                   "[NATIVE-PRESOLVE] p4_candidates: parallel_rows=%ld "
                   "parallel_cols=%ld\n",
                   counts.parallel_row_candidates,
                   counts.parallel_col_candidates);
      std::fprintf(stderr,
                   "[NATIVE-PRESOLVE] %s: rows %ld->%ld cols %ld->%ld nnz "
                   "%ld->%ld use_reduced=%d %.1fms\n",
                   out.status.c_str(), out.orig_rows, out.reduced_rows,
                   out.orig_cols, out.reduced_cols, out.orig_nnz,
                   out.reduced_nnz, static_cast<int>(out.use_reduced),
                   out.presolve_ms);
    }
    return out;
  }

  // Meager-reduction gate (§6, P1 second mismatch round).  The reduced
  // solve is speculative: the caller can always solve the original model
  // directly.  IPM factorization cost is superlinear in model size, so a
  // reduction below ~1% in BOTH rows and columns buys at most ~2% of the
  // direct solve time, while a numerically hostile reduced model costs at
  // least one wasted solve attempt of the same order as the direct solve —
  // and tiny deletions can flip a degenerate barrier trajectory into a
  // stalled basin (modszk1: 0.3% of rows removed, the reduced normal-
  // equation trajectory pins at mu=2.3e-7 where the original's dives to
  // 1e-9 — 4.0s vs 0.069s for a mathematically equivalent model).  The
  // expected value of solving such a reduction is negative; report it as
  // unused so the caller's solve is bit-identical to the pre-presolve path
  // (gate G4, §4), exactly like "no_reduction".
  {
    long active_rows = 0;
    long active_cols = 0;
    for (int i = 0; i < w.m; ++i)
      active_rows += w.row_active[static_cast<std::size_t>(i)];
    for (int j = 0; j < w.n; ++j)
      active_cols += w.col_active[static_cast<std::size_t>(j)];
    const double row_cut =
        1.0 - static_cast<double>(active_rows) / std::max(1, w.m);
    const double col_cut =
        1.0 - static_cast<double>(active_cols) / std::max(1, w.n);
    if (row_cut < 0.01 && col_cut < 0.01) {
      long active_nnz = 0;
      for (int j = 0; j < w.n; ++j) {
        if (!w.col_active[static_cast<std::size_t>(j)]) continue;
        for (int p = w.csc_start[static_cast<std::size_t>(j)];
             p < w.csc_start[static_cast<std::size_t>(j) + 1]; ++p)
          active_nnz += w.row_active[static_cast<std::size_t>(w.csc_row[p])];
      }
      out.reduced_rows = active_rows;
      out.reduced_cols = active_cols;
      out.reduced_nnz = active_nnz;
      out.status = "meager_reduction";
      out.presolve_ms = elapsed_ms();
      if (cfg.verbose) {
        std::fprintf(stderr,
                     "[NATIVE-PRESOLVE] %s: rows %ld->%ld cols %ld->%ld nnz "
                     "%ld->%ld use_reduced=%d %.1fms\n",
                     out.status.c_str(), out.orig_rows, out.reduced_rows,
                     out.orig_cols, out.reduced_cols, out.orig_nnz,
                     out.reduced_nnz, static_cast<int>(out.use_reduced),
                     out.presolve_ms);
      }
      return out;
    }
  }

  // === Compaction: rebuild a compressed LPModel from the active flags ===
  // Output convention: rows whose sides are exactly equal (lhs == rhs —
  // every former Aeq row keeps exact equality through fixed-column
  // substitution, which shifts both sides by the same amount) go back into
  // `Aeq`/`beq`; all other surviving rows live in `A` with double bounds in
  // row_lhs/b.  The IPM kernel's slack form requires this split: an
  // equality expressed as a ranged A-row gets a zero-width slack whose
  // barrier curvature blows up (theta ~ z/gu with gu -> 0), destroying the
  // normal-equation conditioning (P1 acceptance mismatch, §6).
  const double sense_sign = lp.sense == Sense::Maximize ? -1.0 : 1.0;
  out.orig_to_reduced_col.assign(static_cast<std::size_t>(w.n), -1);
  std::vector<int> red_to_orig;
  red_to_orig.reserve(static_cast<std::size_t>(w.n));
  for (int j = 0; j < w.n; ++j) {
    if (!w.col_active[static_cast<std::size_t>(j)]) continue;
    out.orig_to_reduced_col[static_cast<std::size_t>(j)] =
        static_cast<int>(red_to_orig.size());
    red_to_orig.push_back(j);
  }
  const int nred = static_cast<int>(red_to_orig.size());

  // Row classification: w.lhs is finite or -kInf, w.rhs finite or +kInf, so
  // lhs == rhs implies both finite — an exact equality row.
  std::vector<int> row_map_a(static_cast<std::size_t>(w.m), -1);
  std::vector<int> row_map_eq(static_cast<std::size_t>(w.m), -1);
  int mred = 0;
  int meq = 0;
  for (int i = 0; i < w.m; ++i) {
    if (!w.row_active[static_cast<std::size_t>(i)]) continue;
    if (w.lhs[static_cast<std::size_t>(i)] == w.rhs[static_cast<std::size_t>(i)])
      row_map_eq[static_cast<std::size_t>(i)] = meq++;
    else
      row_map_a[static_cast<std::size_t>(i)] = mred++;
  }

  LPModel& red = out.reduced;
  red.sense = lp.sense;
  red.c.resize(nred);
  red.vars.resize(static_cast<std::size_t>(nred));
  for (int r = 0; r < nred; ++r) {
    const int j = red_to_orig[static_cast<std::size_t>(r)];
    red.c[r] = sense_sign * w.c[static_cast<std::size_t>(j)];
    auto& v = red.vars[static_cast<std::size_t>(r)];
    if (j < static_cast<int>(lp.vars.size())) {
      v = lp.vars[static_cast<std::size_t>(j)];
    }
    const double lo = w.lb[static_cast<std::size_t>(j)];
    const double hi = w.ub[static_cast<std::size_t>(j)];
    v.lb = lo == -kInf ? -kVariableNoBound : lo;
    v.ub = hi == kInf ? kVariableNoBound : hi;
  }

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(w.csc_row.size());
  std::vector<Eigen::Triplet<double>> trips_eq;
  long reduced_nnz = 0;
  for (int r = 0; r < nred; ++r) {
    const int j = red_to_orig[static_cast<std::size_t>(r)];
    for (int p = w.csc_start[static_cast<std::size_t>(j)];
         p < w.csc_start[static_cast<std::size_t>(j) + 1]; ++p) {
      const int i = w.csc_row[static_cast<std::size_t>(p)];
      const int ra = row_map_a[static_cast<std::size_t>(i)];
      if (ra >= 0) {
        trips.emplace_back(ra, r, w.csc_val[static_cast<std::size_t>(p)]);
        ++reduced_nnz;
        continue;
      }
      const int re = row_map_eq[static_cast<std::size_t>(i)];
      if (re >= 0) {
        trips_eq.emplace_back(re, r, w.csc_val[static_cast<std::size_t>(p)]);
        ++reduced_nnz;
      }
    }
  }
  red.A.resize(mred, nred);
  red.A.setFromTriplets(trips.begin(), trips.end());
  red.A.makeCompressed();
  red.row_lhs.resize(mred);
  red.b.resize(mred);
  for (int i = 0; i < w.m; ++i) {
    const int ri = row_map_a[static_cast<std::size_t>(i)];
    if (ri < 0) continue;
    red.row_lhs[ri] = w.lhs[static_cast<std::size_t>(i)];
    red.b[ri] = w.rhs[static_cast<std::size_t>(i)];
  }
  red.Aeq.resize(meq, nred);
  red.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
  red.Aeq.makeCompressed();
  red.beq.resize(meq);
  for (int i = 0; i < w.m; ++i) {
    const int ri = row_map_eq[static_cast<std::size_t>(i)];
    if (ri < 0) continue;
    red.beq[ri] = w.rhs[static_cast<std::size_t>(i)];
  }

  // Offset convention: internally accumulated in the minimization sense;
  // reported in the original sense so c_orig' x_orig = c_red' x_red + offset.
  out.objective_offset = sense_sign * objective_offset_min;
  out.use_reduced = true;
  out.reduced_rows = mred + meq;
  out.reduced_cols = nred;
  out.reduced_nnz = reduced_nnz;
  out.status = "reduced";
  out.presolve_ms = elapsed_ms();

  if (cfg.verbose) {
    std::fprintf(stderr,
                 "[NATIVE-PRESOLVE] rules: rounds=%d empty_rows=%ld "
                 "empty_cols=%ld fixed_cols=%ld redundant_rows=%ld "
                 "singleton_rows=%ld implied_bounds=%ld doubleton_eq=%ld "
                 "singleton_cols=%ld "
                 "free_col_subst=%ld\n",
                 rounds, counts.empty_rows, counts.empty_cols,
                 counts.fixed_cols, counts.redundant_rows,
                 counts.singleton_rows, counts.implied_bound_tightenings,
                 counts.doubleton_equations, counts.singleton_columns,
                 counts.free_column_substitutions);
    std::fprintf(stderr,
                 "[NATIVE-PRESOLVE] p2_candidates: singleton_cols=%ld "
                 "singleton_eq=%ld singleton_ranged=%ld doubleton_eq=%ld "
                 "free_eq_cols=%ld implied_free_eq_cols=%ld\n",
                 counts.active_singleton_col_candidates,
                 counts.singleton_equality_col_candidates,
                 counts.singleton_ranged_col_candidates,
                 counts.doubleton_equality_row_candidates,
                 counts.free_equality_col_candidates,
                 counts.implied_free_equality_col_candidates);
    std::fprintf(stderr,
                 "[NATIVE-PRESOLVE] p4_candidates: parallel_rows=%ld "
                 "parallel_cols=%ld\n",
                 counts.parallel_row_candidates,
                 counts.parallel_col_candidates);
    std::fprintf(stderr,
                 "[NATIVE-PRESOLVE] %s: rows %ld->%ld cols %ld->%ld nnz "
                 "%ld->%ld use_reduced=%d %.1fms\n",
                 out.status.c_str(), out.orig_rows, out.reduced_rows,
                 out.orig_cols, out.reduced_cols, out.orig_nnz,
                 out.reduced_nnz, static_cast<int>(out.use_reduced),
                 out.presolve_ms);
  }
  return out;
}

}  // namespace mipsolvers::engine

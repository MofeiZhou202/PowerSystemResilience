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
#include <limits>
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
  long total() const {
    return empty_rows + empty_cols + fixed_cols + redundant_rows +
           singleton_rows;
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
  // finite row sides below the |side| >= 1e19 no-bound sentinel, floored
  // at 1.
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
  // The floor cannot violate the invariant in this wiring: kSideShiftCap
  // bounds scale_reduced <= 1e2 * scale_orig, so 0.9 * ratio >= 9e-3
  // always; the floor is purely defensive against future rule phases
  // changing that bound.
  return std::clamp(0.9 * scale_orig / scale_reduced, 1e-4, 1.0);
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
                 "singleton_rows=%ld\n",
                 rounds, counts.empty_rows, counts.empty_cols,
                 counts.fixed_cols, counts.redundant_rows,
                 counts.singleton_rows);
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

/// @file lp_presolve.hpp
/// @brief Native LP presolve — P1 zero-fill rules + P2 substitution rules
/// (doubleton equation / free column / singleton column) + postsolve stack.
///
/// Design document: docs/archive/native_presolve_lp_2026-08-18.md.
/// Rule catalog: Andersen & Andersen, "Presolving in Linear Programming",
/// Mathematical Programming 71 (1995), §2.1-2.3 (empty row/col, fixed col,
/// redundant row, singleton row) and §2.4 (substitution rules, implemented
/// per the HiGHS-semantics algorithm cards of appendix A).  The component
/// takes an LPModel and returns a reduced LPModel plus a postsolve stack
/// that recovers a primal solution of the original model, following the
/// per-reduction-record / reverse-undo protocol of HighsPostsolveStack
/// referenced in §2.1 of the design doc.
#pragma once

#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Configuration for the native LP presolve.
struct LpPresolveConfig {
  /// Master switch.  From P1 on, IPMLPOptions::presolve is the programmatic
  /// switch (§3.1) and the IPM wiring overlays the
  /// MIPSOLVERS_NATIVE_PRESOLVE* environment variables on top of it.
  bool enabled{false};
  /// Run P2 equality substitutions after the P1 zero-fill fixpoint.  Kept
  /// separately controllable for phase-level regression tests; production
  /// callers leave the theory-guided P2 rule set enabled.
  bool substitutions{true};
  /// Run P3 row-activity implied-bound propagation.  Production callers
  /// leave it enabled; phase-level tests can isolate earlier rule sets.
  bool propagate_bounds{true};
  /// Wall-clock time box for one presolve pass.  Exceeding the box returns
  /// the unreduced model with status "time_box" (risk R5, §5).
  double time_box_sec{2.0};
  /// Emit [NATIVE-PRESOLVE] per-rule counts and a summary line to stderr.
  bool verbose{false};
};

/// Overlay MIPSOLVERS_NATIVE_PRESOLVE* environment overrides on top of
/// @p base (same convention as highs_lp_presolve_config_from_env):
///   MIPSOLVERS_NATIVE_PRESOLVE         "0" -> force disable, anything else -> enable
///   MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX double -> time_box_sec
///   MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE present -> verbose
LpPresolveConfig lp_presolve_config_from_env(LpPresolveConfig base);

/// One postsolve record: column `orig_col` was fixed to `value` (A&A §2.2:
/// fixed-column substitution and zero-cost/unbounded-direction empty-column
/// fixing).  Undo restores the primal value directly; no other solution
/// component is involved, which is why primal-only postsolve needs nothing
/// else for the P1 rule set (§3.3: the IPM path has no basis postsolve).
struct LpPostsolveFixedCol {
  int orig_col{-1};
  double value{0.0};
};

/// One postsolve record: the doubleton equation
/// `coef_subst * x_subst + coef_stay * x_stay = rhs` (exact equality,
/// lhs == rhs) was used to eliminate column `subst_col` (A&A §2.4;
/// design appendix A.1).  Primal undo evaluates
/// `x_subst = (rhs - coef_stay * x_stay) / coef_subst`
/// (HighsPostsolveStack DoubletonEquation, primal-only subset — the IPM path
/// has no basis/dual postsolve, §3.3).  `rhs` is the row side at substitution
/// time (after every earlier substitution's equal shift of both sides).
struct LpPostsolveDoubletonEquation {
  int subst_col{-1};
  int stay_col{-1};
  double coef_subst{0.0};  ///< a_s: coefficient of the eliminated column.
  double coef_stay{0.0};   ///< a_t: coefficient of the remaining column.
  double rhs{0.0};         ///< Row side at substitution time.
};

/// One postsolve record: column `col` was eliminated through the (equality)
/// row whose live entries — snapshot taken BEFORE the row was deleted
/// (appendix A.4 trap 8) — are stored here excluding `col` itself (A&A §2.4;
/// design appendix A.2).  Primal undo evaluates
/// `x_col = (rhs - sum_j a_j * x_j) / pivot`
/// (HighsPostsolveStack FreeColSubstitution, primal-only subset).
struct LpPostsolveFreeColSubstitution {
  int col{-1};
  double rhs{0.0};    ///< Binding row side at substitution time.
  double pivot{0.0};  ///< a_rc: coefficient of `col` in the substitution row.
  /// (orig_col, coefficient) pairs of the substitution row, `col` excluded.
  std::vector<std::pair<int, double>> row_entries;
};

/// Postsolve stack entry.  P1 rules other than column fixing (row deletions,
/// bound tightenings) do not change the solution space and need no record;
/// P2 adds the two substitution records above (§3.1).
using LpPostsolveRecord =
    std::variant<LpPostsolveFixedCol, LpPostsolveDoubletonEquation,
                 LpPostsolveFreeColSubstitution>;

/// Outcome of one native LP presolve pass.  `reduced` is valid iff
/// `use_reduced` is true (same contract as HighsLpPresolveResult).
struct LpPresolveResult {
  bool use_reduced{false};  ///< Solve `reduced`, then postsolve the primal.
  bool infeasible{false};   ///< Presolve proved infeasibility.
  /// Presolve suspects unboundedness (an empty column with a nonzero cost in
  /// an unbounded improving direction, A&A §2.2).  Presolve never concludes
  /// unboundedness itself: use_reduced stays false and the solver decides
  /// authoritatively on the original model (§4, P1 definition).
  bool unbounded_candidate{false};
  LPModel reduced;          ///< Native reduced LP (valid iff use_reduced).
  /// Constant term the deleted fixed columns contribute to the objective, in
  /// the original sense convention: c_orig' x_orig = c_red' x_red + offset.
  double objective_offset{0.0};
  /// Per-reduction records in application order; undo in reverse (§2.1).
  std::vector<LpPostsolveRecord> postsolve_stack;
  /// Original column -> reduced column, -1 when deleted (recovered from the
  /// postsolve stack).  Size == original column count when use_reduced.
  std::vector<int> orig_to_reduced_col;
  double presolve_ms{0.0};  ///< Wall-clock time of the pass.
  long orig_rows{-1};
  long orig_cols{-1};
  long orig_nnz{-1};
  long reduced_rows{-1};
  long reduced_cols{-1};
  long reduced_nnz{-1};
  std::string status;       ///< Machine-readable pass status name.
};

/// Run one native presolve pass over @p lp.  Never throws.
///
/// Rule set (§4): iterate the zero-fill rules of A&A §2.1-2.3 (empty row/col,
/// fixed column, redundant row, singleton row) together with the P2
/// substitution rules of A&A §2.4 (doubleton equation, free/implied-free
/// column substitution, singleton-column substitution — appendix A algorithm
/// cards) to a fixpoint on an in-place entry-pool adjacency representation
/// with row/column active flags, then compact once into the reduced LPModel
/// (surviving exact-equality rows — lhs == rhs, which substitutions preserve
/// bitwise because both sides are shifted by the identical amount — go back
/// into `Aeq`/`beq`; all other rows live in `A` with double bounds in
/// `row_lhs`/`b`.  The split is required by the IPM kernel's slack form: an
/// equality expressed as a ranged A-row gets a zero-width slack whose barrier
/// curvature destroys the normal-equation conditioning, §6 P1 acceptance
/// fix).  With no reductions the pass reports
/// use_reduced=false so the caller's solve is bit-identical to the
/// pre-presolve path (gate G4, §4).  The same bit-identity holds when the
/// reduction is meager (< 1% in both rows and columns): the pass then
/// reports status "meager_reduction" with use_reduced=false — a reduction
/// that small has negative expected value for the speculative reduced
/// solve (§6, P1 second mismatch round).
LpPresolveResult lp_presolve_run(const LPModel& lp, const LpPresolveConfig& cfg);

/// Recover an original-space primal from a reduced-space primal.
///
/// Contract: `result.use_reduced` must be true and `x_reduced` must have one
/// entry per reduced column.  Active columns are filled through
/// `orig_to_reduced_col`, then the postsolve stack is undone in reverse
/// order (§2.1 HighsPostsolveStack protocol).  A contract violation returns
/// an empty vector; the caller must treat that as a postsolve failure and
/// fall back — never publish the empty vector as a solution.
Eigen::VectorXd postsolve_primal(const LpPresolveResult& result,
                                 const Eigen::VectorXd& x_reduced);

/// Publication tolerance scale for the reduced solve (relative to the
/// reduced model's global side scale) that guarantees the wiring's
/// original-model residual audit.  Returns
/// `min(0.9 * scale_orig/scale_reduced, 1.0)`; the invariant
/// `result <= scale_orig/scale_reduced` is what the audit transfer
/// requires.  See the implementation comment for the full derivation
/// (native_presolve_lp_2026-08-18.md §6, P1 second mismatch round).
double lp_presolve_publication_tol_scale(const LPModel& orig,
                                         const LPModel& reduced);

}  // namespace mipsolvers::engine

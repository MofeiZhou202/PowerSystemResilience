/// @file bc_cglp.hpp
/// @brief Lift-and-project disjunctive cut generation (CGLP).
///
/// The implementation builds and solves the disjunctive CGLP, validates
/// fallback cuts on both sides of the binary disjunction, applies efficacy
/// filtering, and deduplicates admitted root cuts. It remains experimental
/// and disabled by default because it has not passed the end-to-end MIPLIB
/// performance gate.
///
/// Theory (Balas, Ceria, Cornuéjols 1993):
///   Given the LP relaxation P = {x : Ax ≤ b, 0 ≤ x ≤ 1} and a fractional
///   point x* with x*_j ∈ (0,1), the 0-1 disjunction x_j ≤ 0 ∨ x_j ≥ 1
///   induces a set of valid inequalities α^T x ≥ β for conv(P ∩ {x_j = 0})
///   ∪ conv(P ∩ {x_j = 1}). The CGLP is the LP that produces the most
///   violated such inequality at x*.
///
/// Implemented scope:
///   * Single-variable 0-1 disjunction only (no general split disjunctions).
///   * Binary variables only; no general-integer lift.
///   * Root-node generation only; no tree-stage separation.
///   * Default OFF via BCOptions::enable_cglp_cuts.

#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"

namespace mipsolvers::engine::detail {

/// @brief Result of a single CGLP solve on one branching binary.
struct CGLPResult {
  /// True iff a usable cut `α^T x ≥ β` was extracted and validated.
  bool generated{false};
  /// Index of the branching binary variable the cut was derived from.
  int branch_var{-1};
  /// Sparse cut coefficients (length == number of vars in the LP).
  Eigen::SparseVector<double> alpha;
  /// Cut right-hand side.
  double beta{0.0};
  /// Observed violation at the incumbent LP point `x*`:
  ///     violation = β − α^T x*   (must be > 0 for a real cut).
  double violation{0.0};
  /// Violation divided by ‖α‖₂; used as the efficacy filter threshold.
  double efficacy{0.0};
  /// True iff the CGLP master LP solved successfully and produced the alpha
  /// vector (as opposed to falling back to the interpolation separator).
  bool cglp_lp_solved{false};
  /// True iff the interpolation fallback separator was invoked (the CGLP LP
  /// failed, produced degenerate alpha, or gave weak efficacy).
  bool used_fallback{false};
  /// One-line diagnostic; populated on both success and failure paths.
  const char* reason{""};
};

/// @brief Per-root-pass hit-rate statistics for CGLP cut generation.
///
/// Populated by add_cglp_cuts_root() via the optional stats_out pointer and
/// accumulated into BCStats by branch_and_cut.cpp across all root cut rounds.
struct CGLPPassStats {
  int n_candidates{0};      ///< Binary variables evaluated as candidates
  int n_cglp_lp_solved{0};  ///< Candidates where CGLP master LP solved cleanly
  int n_fallback_used{0};   ///< Candidates where interpolation fallback was used
  int n_accepted{0};        ///< Cuts admitted after all filters and dedup
  int n_dedup_rejected{0};  ///< Candidate cuts dropped by hash deduplication

  // Per-rejection-reason triage counters (populated only when cut not accepted).
  int n_rej_build_failed{0};
  int n_rej_cglp_lp_failed{0};
  int n_rej_degenerate_alpha{0};
  int n_rej_weak_cut{0};
  int n_rej_branch_validity{0};
};

/// @brief Input bundle for one root-level CGLP pass.
/// @note Lifetime of referenced LP/options must outlive the call.
struct CGLPContext {
  const LPModel& lp;          ///< Current relaxation (incl. accumulated cuts).
  const Eigen::VectorXd& x_star;  ///< Current LP solution to separate.
  const BCOptions& opt;       ///< CGLP options live under opt.enable_cglp_cuts.
};

/// @brief Canonicalized row used by the CGLP builder.
///
/// The CGLP implementation converts the source LP into a pure
/// inequality system `Mx <= d` (including finite variable bounds and
/// equality rows split into two inequalities). This representation is
/// used to assemble both disjunction-side dual blocks.
struct CGLPCanonicalRow {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
};

/// @brief Materialized CGLP master LP and index metadata.
///
/// Variable layout:
///   alpha[0..n-1], beta,
///   lambda0[0..R-1], gamma0,
///   lambda1[0..R-1], gamma1
/// where R is the number of canonical rows in `Mx <= d`.
struct CGLPModel {
  LPModel lp;
  std::vector<CGLPCanonicalRow> canonical_rows;

  int n_original{0};
  int branch_var{-1};

  int alpha_start{0};
  int beta_idx{-1};
  int lambda0_start{-1};
  int gamma0_idx{-1};
  int lambda1_start{-1};
  int gamma1_idx{-1};

  int canonical_row_count{0};
};

/// @brief Build a CGLP master LP for the binary disjunction
/// `x_j <= 0 OR x_j >= 1` around the fractional point in ctx.
///
/// This function constructs both side blocks, beta-link rows, and the
/// normalization row. `generate_cglp_cut()` owns solving and cut extraction.
///
/// @return `true` on successful model construction.
/// @return `false` if `j` is out of range, non-binary, or ctx dimensions
/// are inconsistent.
bool build_cglp_lp(const CGLPContext& ctx, int j, CGLPModel& out_model);

/// @brief Select CGLP candidate binaries from the current fractional LP
/// solution. Returns variable indices sorted by "separation utility"
/// (currently: raw fractionality |x_j - round(x_j)|, descending).
std::vector<int> select_cglp_candidates(const CGLPContext& ctx);

/// @brief Solve the CGLP for a single 0-1 disjunction on variable `j` and
/// return at most one cut.
///
/// Contract:
///   * If `generated` is true, then `α^T x* < β - opt.cglp_min_efficacy * ‖α‖₂`
///     at the provided `ctx.x_star`.
///   * The cut is valid for the original LP's integer hull,
///     i.e. every integer-feasible x of the outer MIP satisfies
///     `α^T x ≥ β` (no node-local bounds in α/β).
///   * If `generated` is false, all other fields are undefined except
///     `branch_var` and `reason`.
CGLPResult generate_cglp_cut(const CGLPContext& ctx, int j);

/// @brief Root-level CGLP pass: iterate candidates, collect generated cuts,
/// and append them to `lp` as new rows. Returns the number of cuts added.
///
/// If `stats_out` is non-null, pass-level hit-rate counters are written there
/// (useful for accumulation across rounds in BCStats).
int add_cglp_cuts_root(LPModel& lp,
                       const Eigen::VectorXd& x_star,
                       const BCOptions& opt,
                       CGLPPassStats* stats_out = nullptr);

}  // namespace mipsolvers::engine::detail

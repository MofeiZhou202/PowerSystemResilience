#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::native_dual {

enum class Status {
  Optimal,
  PrimalInfeasible,
  Unbounded,
  DualInfeasibleStart,
  IterationLimit,
  TimeLimit,
  ObjectiveCutoff,
  NumericalFailure,
  InvalidBasis,
};

struct Statistics {
  int iterations{0};
  int degenerate_dual_steps{0};
  int degenerate_primal_steps{0};
  int major_rebuilds{0};
  int reinversions{0};
  int initial_factor_reuses{0};
  int rank_repairs{0};
  int bound_flips{0};
  int iterative_refinements{0};
  int canonical_primal_corrections{0};
  int canonical_pivot_reinversions{0};
  int pivot_identity_refinements{0};
  int cost_shifts{0};
  int dual_start_cost_shifts{0};
  int dual_start_required_cost_shift_lower_bound{0};
  int dual_start_shift_threshold{0};
  int dual_start_adaptive_rejections{0};
  int cleanup_passes{0};
  int dual_phase_one_iterations{0};
  int dual_phase_two_iterations{0};
  int primal_phase_one_iterations{0};
  int primal_phase_two_iterations{0};
  int primal_cleanup_iterations{0};
  int phase_transitions{0};
  int cleanup_required{0};
  int cleanup_avoided{0};
  int transition_dual_infeasibility_count{0};
  // 0: not terminated in dual Phase I, 1: CHUZR found no leaving row,
  // 2: CHUZC found no entering column after a fresh rebuild,
  // 3: the original-bound dual defect reached zero and was rebuilt/audited.
  int dual_phase_one_terminal_reason{0};
  int dual_phase_one_terminal_leaving_row{-1};
  int dual_phase_one_terminal_leaving_side{0};
  int dual_phase_one_positive_candidates{0};
  int dual_phase_one_certified_candidates{0};
  int dual_phase_one_stable_candidates{0};
  int dse_reselections{0};
  int dse_initialization_solves{0};
  int dse_initialization_refinements{0};
  int certified_dse_btrans{0};
  int certified_dse_candidates{0};
  int certified_dse_rejections{0};
  int devex_frameworks{0};
  int devex_restarts{0};
  int cycles_detected{0};
  int taboo_changes{0};
  int taboo_rejections{0};
  int taboo_rows{0};
  int taboo_row_rejections{0};
  int taboo_row_releases{0};
  int harris_second_pass_candidates{0};
  int unstable_pivot_rejections{0};
  int stability_blocked_rows{0};
  int max_updates_between_rebuilds{0};
  int rebuild_update_limit{0};
  int rebuild_synthetic_work{0};
  int rebuild_numerical_trouble{0};
  int rebuild_possibly_optimal{0};
  int rebuild_possibly_infeasible{0};
  double max_primal_infeasibility{0.0};
  double max_dual_infeasibility{0.0};
  double max_primal_drift{0.0};
  double max_dual_drift{0.0};
  double max_objective_drift{0.0};
  double max_cost_perturbation{0.0};
  double max_cost_shift{0.0};
  double dse_initialization_time_sec{0.0};
  double certified_dse_time_sec{0.0};
  double dual_phase_one_time_sec{0.0};
  double dual_phase_two_time_sec{0.0};
  double primal_phase_one_time_sec{0.0};
  double primal_phase_two_time_sec{0.0};
  double cleanup_time_sec{0.0};
  double primal_cleanup_time_sec{0.0};
  double dual_phase_one_initial_objective{0.0};
  double dual_phase_one_final_objective{0.0};
  double transition_max_dual_infeasibility{0.0};
  double dual_phase_one_terminal_violation{0.0};
  double dual_phase_one_positive_capacity{0.0};
  double dual_phase_one_certified_capacity{0.0};
  double dual_phase_one_stable_capacity{0.0};
  double dual_phase_one_stable_capacity_error{0.0};
};

struct Result {
  Status status{Status::InvalidBasis};
  std::string message;
  Statistics statistics;
  std::vector<int> basis;
  std::vector<char> at_upper;
  std::vector<double> edge_weights;
  Eigen::VectorXd x_basic;
  Eigen::VectorXd reduced_costs;
  Eigen::VectorXd farkas_multiplier;
  double farkas_margin{0.0};
  double farkas_error_bound{0.0};
  bool has_farkas_certificate{false};
  Eigen::VectorXd primal_ray;
  bool has_unbounded_certificate{false};
  double max_objective{0.0};
  // Set on TimeLimit / IterationLimit results when the interrupted state was
  // re-certified against the ORIGINAL cost (stabilization deltas removed,
  // state reconstructed, dual feasibility audited).  When true, max_objective
  // is the exact original-cost dual objective of a dual-feasible point — a
  // rigorous bound on the LP optimum despite the unfinished solve, resting on
  // the same audit an Optimal result rests on.  When false, max_objective is
  // the working-cost objective and must not be used as a bound.
  bool dual_bound_certified{false};
  std::shared_ptr<BasisOps> basis_ops;
};

// Revised dual simplex Phase II for an inherited basis. The routine owns all
// numerical state transitions: rank repair, reinversion, CHUZR, CHUZC/BFRT,
// pivot verification and exact state reconstruction. It never cold-starts or
// calls another LP solver.
Result solve_phase2(const StandardFormLP& sf,
                    const SimplexOptions& options,
                    const SimplexBasis& basis_hint);

// Full warm-start controller. A valid inherited basis may repair sparse
// one-sided dual defects with cost shifts. It never discards a failed warm
// basis for a cold retry.
Result solve(const StandardFormLP& sf,
             const SimplexOptions& options,
             const SimplexBasis& basis_hint);

// Full cold/warm controller. A null hint constructs a logical basis, applies
// only rank-certified singleton crash substitutions, and runs the standard
// artificial-objective revised-primal Phase I before optimization.
Result solve(const StandardFormLP& sf,
             const SimplexOptions& options,
             const SimplexBasis* basis_hint);

const char* status_name(Status status);

}  // namespace mipsolvers::engine::native_dual

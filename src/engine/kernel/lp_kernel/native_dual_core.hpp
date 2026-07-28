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
  DualInfeasibleStart,
  IterationLimit,
  TimeLimit,
  ObjectiveCutoff,
  NumericalFailure,
  InvalidBasis,
};

struct Statistics {
  int iterations{0};
  int major_rebuilds{0};
  int reinversions{0};
  int rank_repairs{0};
  int bound_flips{0};
  int iterative_refinements{0};
  int canonical_primal_corrections{0};
  int canonical_pivot_reinversions{0};
  int pivot_identity_refinements{0};
  int cost_shifts{0};
  int cleanup_passes{0};
  int dse_reselections{0};
  int dse_initialization_solves{0};
  int dse_initialization_refinements{0};
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
  double max_objective{0.0};
  std::shared_ptr<BasisOps> basis_ops;
};

// Revised dual simplex Phase II for an inherited basis. The routine owns all
// numerical state transitions: rank repair, reinversion, CHUZR, CHUZC/BFRT,
// pivot verification and exact state reconstruction. It never cold-starts or
// calls another LP solver.
Result solve_phase2(const StandardFormLP& sf,
                    const SimplexOptions& options,
                    const SimplexBasis& basis_hint);

// Full warm-start controller. A valid inherited basis must admit a legitimate
// nonbasic bound-side assignment that is dual feasible. The solver never
// shifts costs or discards a failed warm basis for a cold retry.
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

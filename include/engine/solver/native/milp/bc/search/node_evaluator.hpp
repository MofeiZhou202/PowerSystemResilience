/// @file node_evaluator.hpp
/// @brief Node LP solver and child evaluation for branch-and-cut tree search.
///
/// NodeEvaluator encapsulates all logic for evaluating a single B&C node:
/// - Domain propagation from branching decisions
/// - LP solve at the node
/// - Reduced-cost variable fixing
/// - Rounding and local heuristics
/// - Generation of node-specific cuts
///
/// This component is shared between sequential and parallel search paths,
/// providing a uniform contract for child evaluation.

#pragma once

#include <vector>
#include <string>

#include <Eigen/Core>

#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/engine/solver/solver_adapter.hpp"
#include "hacdcpf/engine/detail/bc_types.hpp"
#include "hacdcpf/engine/detail/bc_pools.hpp"

namespace hacdcpf::engine::solver::native::milp::bc {

// Forward declarations
class BCSolveContext;

// Type aliases for convenience
using SolveResult = hacdcpf::engine::SolveResult;
using PoolSolution = hacdcpf::engine::detail::PoolSolution;

/// @brief Pseudocost update tracking for branching statistics.
struct PCUpdate {
  int branch_var{-1};
  bool is_up_branch{false};
  bool has_inference{false};
  double inference_count{0.0};
  bool has_gain{false};
  double gain{0.0};
  double cutoff_count{0};
  double conflict_score{0.0};
};

/// @brief Reduced cost fixing result.
struct FixingResult {
  int variables_fixed{0};
  int bounds_tightened{0};
  bool infeasible{false};
};

/// @struct ChildEvalResult
/// @brief Result of evaluating a single child node.
///
/// Encapsulates:
/// - Feasibility (node pruned or node LP solved)
/// - Updated child state (bounds, relaxation, basis)
/// - Pool updates (solutions, cuts, pseudocost observations)
/// - Work statistics (LP solves, cuts generated, etc.)
struct ChildEvalResult {
  /// Feasibility: false = node infeasible/pruned, true = LP solved
  bool feasible{false};

  /// Child node state after evaluation (updated bounds, solution, basis).
  detail::Node updated_child;

  /// New solutions found at this node.
  std::vector<PoolSolution> new_solutions;

  /// New cuts generated at this node.
  std::vector<detail::PoolCut> new_cuts;

  /// Pseudocost observations to update branching statistics.
  std::vector<PCUpdate> pseudocost_updates;

  /// Work statistics
  struct {
    int lp_solves{0};
    int cuts_added{0};
    int pool_cuts_separated{0};
    int conflict_tightenings{0};
  } counters;

  /// Reason for infeasibility (if feasible == false).
  std::string infeasibility_reason;
};

/// @class NodeEvaluator
/// @brief Evaluates a single B&C node (domain propagation + LP solve + cuts).
///
/// 8-Step Pipeline:
/// 1. Domain propagation with conflict learning
/// 2. Node LP solve (simplex or IPM with fallback)
/// 3. Incumbent pruning check
/// 4. Reduced-cost variable fixing
/// 5. Pool cut separation and re-solve
/// 6. Node-specific cut generation and re-solve
/// 7. Rounding heuristics and local search
/// 8. Pseudocost observation tracking
///
/// Usage:
/// @code
///   NodeEvaluator evaluator(ctx);
///   ChildEvalResult result = evaluator.evaluate(child_node, parent_bound);
///   if (!result.feasible) {
///     // Node pruned
///   } else {
///     // Use result.updated_child for further branching
///   }
/// @endcode
class NodeEvaluator {
 public:
  /// Construct evaluator with a solver context.
  /// @param context Reference to BCSolveContext (shared state).
  explicit NodeEvaluator(BCSolveContext& context);

  /// Evaluate a single child node.
  /// @param child          Node to evaluate (bounds from branching decision).
  /// @param is_up_branch   True if branching on x_j >= val, false for x_j <= val.
  /// @param branch_var     Variable index that was branched on.
  /// @param parent_bound   Bound from parent node (for dual progress tracking).
  /// @param incumbent_obj  Current best feasible objective (or infinity).
  /// @return ChildEvalResult with updated child state and pool updates.
  ChildEvalResult evaluate(
      detail::Node& child,
      bool is_up_branch,
      int branch_var,
      double parent_bound,
      double incumbent_obj
  );

 private:
  /// STEP 1: Domain propagation with conflict learning.
  /// @param child             Node with new bounds from branching.
  /// @param conflict_tightenings Output: count of tightenings from propagation.
  /// @return true if feasible after propagation, false if infeasible.
  bool propagate_domain(detail::Node& child, int& conflict_tightenings);

  /// STEP 2: Solve node LP (simplex or IPM with fallback).
  /// @param child   Node with bounds to solve LP over.
  /// @param result  Output: LP solution (x, objective, basis if simplex).
  /// @return true if LP solved successfully, false if infeasible/numerical failure.
  bool solve_node_lp(detail::Node& child, SolveResult& result);

  /// STEP 3: Check if incumbent objective prunes this node.
  /// @param node_bound    Current node lower bound.
  /// @param incumbent_obj  Best feasible objective.
  /// @return true if node should be pruned, false otherwise.
  bool check_incumbent_prune(double node_bound, double incumbent_obj);

  /// STEP 4: Reduced-cost variable fixing using simplex basis.
  /// @param child        Node with LP solution.
  /// @param lp_result    LP solve result with optional basis + reduced costs.
  /// @param incumbent_obj Best feasible objective for bounds.
  /// @return FixingResult with variable count + infeasibility flag.
  FixingResult apply_reduced_cost_fixing(
      detail::Node& child,
      const SolveResult& lp_result,
      double incumbent_obj
  );

  /// STEP 5: Separation and re-solve with violated cuts from global pool.
  /// @param child   Node with updated LP solution.
  /// @param result  Output: updated SolveResult after pool-cut re-solve.
  /// @return Number of pool cuts separated.
  int separate_pool_cuts(detail::Node& child, SolveResult& result);

  /// STEP 6: Generation and re-solve with node-specific cuts.
  /// @param child   Node with updated LP solution.
  /// @param result  Output: updated SolveResult after cut re-solve.
  /// @return Number of cuts generated.
  int generate_node_cuts(detail::Node& child, SolveResult& result);

  /// STEP 7: Rounding heuristics and local search.
  /// @param child     Node with LP solution.
  /// @param lp_result LP solve result for heuristic starting point.
  /// @return Vector of feasible solutions found (possibly empty).
  std::vector<PoolSolution> apply_rounding_heuristics(
      detail::Node& child,
      const SolveResult& lp_result
  );

  /// STEP 8: Compute pseudocost updates for branching statistics.
  /// @param branch_var Variable that was branched on.
  /// @param is_up_branch True for upper bound branch.
  /// @param gain Dual bound improvement (node.bound - parent.bound).
  /// @return Vector of pseudocost observations.
  std::vector<PCUpdate> compute_pseudocost_updates(
      int branch_var,
      bool is_up_branch,
      double gain
  );

  BCSolveContext* ctx_{nullptr};
};

}  // namespace hacdcpf::engine::solver::native::milp::bc

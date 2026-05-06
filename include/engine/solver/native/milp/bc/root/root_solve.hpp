/// @file root_solve.hpp
/// @brief Root LP solve and initialization for branch-and-cut.
///
/// The root solver handles:
/// - Root LP solve with initial basis/basis factorization
/// - Root-specific presolve and bound tightening
/// - Early termination checks (gap closed, infeasible)
/// - Setup for tree search (bounds, incumbent, basis caching)

#pragma once

#include <limits>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/engine/detail/bc_types.hpp"

namespace hacdcpf::engine::solver::native::milp::bc {

class BCSolveContext;

/// @struct RootPipelineResult
/// @brief Result of root solve phase (before tree search begins).
///
/// Contains:
/// - Root node with bounds, LP solution, basis
/// - Incumbent solution (if found by heuristics)
/// - Cut information (cuts added, pool state)
/// - Termination flag (if root solving reached optimality)
struct RootPipelineResult {
  /// Root node state after solve.
  detail::Node root_node;

  /// Feasible solution found during root phase (if any).
  std::vector<Eigen::VectorXd> root_solutions;

  /// Cuts added during root solve.
  int cuts_added{0};

  /// Root LP bound (lower bound in minimization).
  double root_bound{std::numeric_limits<double>::lowest()};

  /// Should tree search proceed? False if root closed gap or reached optimality.
  bool proceed_to_tree{true};

  /// Termination reason (if !proceed_to_tree).
  std::string termination_reason;
};

/// @class RootSolvePhase
/// @brief Executes root LP solve pipeline (initial bounds, cuts, heuristics).
class RootSolvePhase {
 public:
  /// Construct root solver with shared context.
  explicit RootSolvePhase(BCSolveContext& context);

  /// Run the complete root pipeline.
  /// Includes:
  /// - Root LP solve
  /// - Root cut rounds
  /// - Feasibility pump + progressive rounding
  /// - Early termination checks
  /// @return Result with root node state and decision to proceed to tree.
  RootPipelineResult solve();

 private:
  /// Solve the root LP relaxation.
  bool solve_root_lp(detail::Node& root_node);

  /// Generate and add cuts at the root.
  /// @param root_node Node to add cuts for.
  /// @return Number of cuts added.
  int generate_root_cuts(detail::Node& root_node);

  /// Run feasibility pump heuristic at root.
  std::vector<Eigen::VectorXd> run_feasibility_pump(
      const detail::Node& root_node);

  /// Run progressive rounding heuristic.
  std::vector<Eigen::VectorXd> run_progressive_rounding(
      const detail::Node& root_node);

  BCSolveContext* ctx_{nullptr};
};

}  // namespace hacdcpf::engine::solver::native::milp::bc

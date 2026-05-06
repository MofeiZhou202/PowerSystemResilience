/// @file sequential_search.hpp
/// @brief Sequential branch-and-cut search driver.
///
/// Implements the standard branch-and-cut loop:
/// 1. Initialize with root node from RootSolvePhase
/// 2. Iteratively: select node → evaluate → branch
/// 3. Termination: time/node/gap limit or queue exhausted

#pragma once

#include <memory>
#include <string>

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine::solver::native::milp::bc {

class BCSolveContext;
class NodeEvaluator;
class RootSolvePhase;

/// @struct SequentialSearchStats
/// @brief Statistics from sequential tree search.
struct SequentialSearchStats {
  int nodes_explored{0};
  int nodes_pruned{0};
  int solutions_found{0};
  double search_time_sec{0.0};
  std::string termination_reason;
};

/// @class SequentialSearchDriver
/// @brief Executes sequential (single-threaded) branch-and-cut search.
///
/// Usage:
/// @code
///   RootSolvePhase root_phase(ctx);
///   auto root_result = root_phase.solve();
///
///   SequentialSearchDriver search(ctx);
///   auto search_result = search.execute(root_result.root_node);
/// @endcode
class SequentialSearchDriver {
 public:
  /// Construct sequential search driver.
  explicit SequentialSearchDriver(BCSolveContext& context);

  /// Execute sequential branch-and-cut search.
  /// @param root_node Root node from root solve phase.
  /// @return Search statistics (termination reason, nodes explored, etc.).
  SequentialSearchStats execute(const detail::Node& root_node);

 private:
  /// Main search loop: select → evaluate → branch.
  void search_loop();

  /// Select next node from queue based on node selection strategy.
  /// @return true if a node was selected, false if queue exhausted.
  bool select_next_node(detail::Node& selected);

  /// Branch on fractional variable at current node.
  /// @param current Current node to branch from.
  /// @param branch_var Variable to branch on.
  void branch(const detail::Node& current, int branch_var);

  /// Check termination conditions (time, nodes, gap limit).
  /// @return true if search should terminate.
  bool should_terminate() const;

  /// Apply bound propagation and presolve at search startup.
  void initialize_search(const detail::Node& root);

  // ── Private Data ──────────────────────────────────────────────────────────
  BCSolveContext* ctx_{nullptr};
  std::unique_ptr<detail::NodeQueue> queue_;
};

}  // namespace mipsolvers::engine::solver::native::milp::bc

/// @file detail/bc/core/node_types.hpp
/// @brief Node and tree management types for branch-and-cut.
///
/// Extracted from bc_types.hpp (Phase 4.5 reorganization).
/// Defines the central Node structure and tree traversal infrastructure.

#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

/// @brief Positive infinity constant used throughout the B&C solver.
constexpr double kInf = std::numeric_limits<double>::infinity();

// ════════════════════════════════════════════════════════════════════════════
// Bound Modification and Conflict Recording
// ════════════════════════════════════════════════════════════════════════════

/// @brief Records a single bound modification (for undo/redo during tree traversal).
struct BoundChange {
  int var_idx;       ///< Variable index
  double old_val;    ///< Value before change (for reverting)
  double new_val;    ///< Value after change (for applying)
  bool is_lb;        ///< true = lower bound, false = upper bound
};

/// @brief Explicit branch-domain literal used for conflict learning.
struct BranchDomainLiteral {
  int var_idx{-1};
  double value{0.0};
  bool is_lb{false};
};

// ════════════════════════════════════════════════════════════════════════════
// Tree Node and Domain Management
// ════════════════════════════════════════════════════════════════════════════

/// @brief Tree node with parent pointer for LCA-based navigation.
/// @details Stores only bound changes relative to parent, not full lb/ub arrays.
struct TreeNode {
  int id{-1};
  int depth{0};
  TreeNode* parent{nullptr};

  /// Bound changes introduced at this node (usually just 1 from branching).
  std::vector<BoundChange> bound_changes;

  /// Inherited from parent; updated after solving this node's LP.
  std::shared_ptr<SimplexBasis> basis_hint;

  /// LP relaxation solution at this node.
  Eigen::VectorXd x_relax;

  /// Dual bound from LP relaxation.
  double bound{kInf};

  /// Whether this node has been solved (for incremental warm-start).
  bool solved{false};
};

/// @brief Manages the global domain state and efficient tree traversal.
/// @details Maintains current lb/ub arrays and handles node switching via LCA.
class TreeDomainManager {
 public:
  Eigen::VectorXd current_lb;
  Eigen::VectorXd current_ub;
  TreeNode* current_node{nullptr};

  /// @brief Initialize with root bounds.
  void initialize(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    current_lb = lb;
    current_ub = ub;
    current_node = nullptr;
  }

  /// @brief Switch domain state from current_node to target_node via LCA.
  int switch_to_node(TreeNode* target);

  /// @brief Get the bound changes needed to go from current to target.
  std::vector<BoundChange> get_changes_to(TreeNode* target) const;

 private:
  int apply_changes(TreeNode* node);
  int revert_changes(TreeNode* node);
  static TreeNode* find_lca(TreeNode* a, TreeNode* b);
};

/// @brief Arena allocator for TreeNode objects (pointer-stable via deque).
class TreeNodeArena {
  std::deque<TreeNode> storage_;
 public:
  /// @brief Allocate a new TreeNode with default-initialized fields.
  TreeNode* alloc() {
    storage_.emplace_back();
    return &storage_.back();
  }
  /// @brief Number of allocated nodes.
  std::size_t size() const { return storage_.size(); }
};

// ════════════════════════════════════════════════════════════════════════════
// Node Representation (Phase 4 forward-compatible format)
// ════════════════════════════════════════════════════════════════════════════

/// @brief Central Node structure for B&C tree node representation.
/// 
/// Contains all information about a branch-and-cut tree node:
/// - Bounds (lb, ub) from branching decisions
/// - LP relaxation solution (x_relax)
/// - Dual bound from LP solve
/// - Simplex basis for warm-start
/// - Branch reasons for conflict learning
/// - Scoring information from NLP solvers (for MINLP)
struct Node {
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd x_relax;
  Eigen::VectorXd x_seed;
  std::optional<Eigen::VectorXd> nlp_down_scores;
  std::optional<Eigen::VectorXd> nlp_up_scores;
  std::shared_ptr<SimplexBasis> basis_hint;
  bool has_live_factor_telemetry{false};
  SparseFactorTelemetry live_factor_telemetry{};
  int ipm_iterations{0};  ///< Number of IPM iterations for this node's LP solve
  std::vector<BranchDomainLiteral> branch_reasons;
  double bound{kInf};
  double estimate{kInf};
  int depth{0};

  /// Create a lightweight child node copying only lb/ub (skips x_relax/x_seed copy).
  Node branch_child() const {
    Node c;
    c.lb = lb;
    c.ub = ub;
    c.nlp_down_scores = nlp_down_scores;
    c.nlp_up_scores = nlp_up_scores;
    c.basis_hint = basis_hint;
    c.has_live_factor_telemetry = has_live_factor_telemetry;
    c.live_factor_telemetry = live_factor_telemetry;
    c.branch_reasons = branch_reasons;
    c.bound = bound;
    c.estimate = estimate;
    c.depth = depth + 1;
    return c;
  }
};

}  // namespace mipsolvers::engine::detail

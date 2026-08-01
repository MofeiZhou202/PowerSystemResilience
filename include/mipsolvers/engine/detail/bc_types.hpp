/// @file bc_types.hpp
/// @brief Core data types for the branch-and-cut solver.
///
/// Contains node representations, pseudo-cost tracking, queue structures,
/// thread workspace, and LP/cut request types used throughout the B&C modules.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/bc/enums.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {

/// @brief Positive infinity constant used throughout the B&C solver.
constexpr double kInf = std::numeric_limits<double>::infinity();

// ════════════════════════════════════════════════════════════════════════════
// Diff-based Tree Management for Memory Efficiency and Numerical Stability
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

/// @brief Learned conflict clause over branch-domain literals.
struct ConflictClause {
  std::vector<BranchDomainLiteral> literals;
  std::size_t hash{0};
};

/// @brief A local domain bound together with the branch-domain reason that
/// implied it during node propagation.
struct DomainReasonBound {
  BranchDomainLiteral bound;
  std::vector<BranchDomainLiteral> reason;
  int trail_pos{-1};
  int depth{0};
  BranchDomainLiteral source_conflict_literal;
  bool has_source_conflict_literal{false};
  std::vector<BranchDomainLiteral> source_conflict_clause;
  bool has_source_conflict_clause{false};
  int prev_bound_pos{-1};
  double prev_bound_value{0.0};
  bool has_proof_activity_audit{false};
  double proof_activity_margin{0.0};
  double proof_activity{0.0};
  double proof_required_activity{0.0};
};

/// @brief Ordered node-local domain trail entry.
///
/// This is the native counterpart of HiGHS' domchgstack_/prevboundval_ chain:
/// every local bound change knows the previous same-side bound active before it.
struct LocalDomainTrailEntry {
  BranchDomainLiteral bound;
  std::vector<BranchDomainLiteral> reason;
  int pos{-1};
  int depth{0};
  int prev_bound_pos{-1};
  double prev_bound_value{0.0};
  bool is_branch{false};
  BranchDomainLiteral source_conflict_literal;
  bool has_source_conflict_literal{false};
  std::vector<BranchDomainLiteral> source_conflict_clause;
  bool has_source_conflict_clause{false};
};

/// @brief Node-local binary implication proved under the node domain.
///
/// The implication is valid only in the subtree that inherits the node domain:
/// `(trigger_var = trigger_value_one) => implied_var bound implied_value`.
struct LocalBinaryImplication {
  int trigger_var{-1};
  bool trigger_value_one{false};
  int implied_var{-1};
  bool implied_is_lb{false};
  double implied_value{0.0};
  std::size_t hash{0};
};

/// @brief Conflict clause with an explicit subtree scope.
///
/// The residual clause is valid only when all scope literals are active in the
/// current node domain.  This mirrors HiGHS' local-domain reconvergence cuts:
/// the frontier is consumed only in the local trail segment that explains it.
struct ScopedConflictClause {
  std::vector<BranchDomainLiteral> scope_literals;
  std::vector<BranchDomainLiteral> residual_literals;
  std::size_t hash{0};
  int depth{0};
  bool has_target_bound{false};
  BranchDomainLiteral target_bound;
  BranchDomainLiteral source_conflict_literal;
  bool has_source_conflict_literal{false};
  bool has_proof_activity_audit{false};
  double proof_activity_margin{0.0};
  double proof_activity{0.0};
  double proof_required_activity{0.0};
};

/// @brief Globally consumable bound-lifting certificate.
///
/// A resolved local-trail proof states that whenever all frontier literals are
/// active, `target_bound` is valid.  This is the first-class counterpart of
/// HiGHS' reconvergence cut `flip(target) + frontier`: the conflict clause is
/// still published, but queued nodes can also consume the implication directly
/// without waiting for unit-clause simulation.
struct BoundLiftingCertificate {
  std::vector<BranchDomainLiteral> frontier_literals;
  BranchDomainLiteral target_bound;
  BranchDomainLiteral source_conflict_literal;
  bool has_source_conflict_literal{false};
  std::size_t hash{0};
  int depth{0};
  bool has_proof_activity_audit{false};
  double proof_activity_margin{0.0};
  double proof_activity{0.0};
  double proof_required_activity{0.0};
};

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
  /// @return The number of bound changes applied.
  int switch_to_node(TreeNode* target) {
    if (current_node == target) return 0;

    if (current_node == nullptr) {
      std::vector<TreeNode*> path;
      TreeNode* t = target;
      while (t != nullptr) {
        path.push_back(t);
        t = t->parent;
      }
      int changes = 0;
      for (auto it = path.rbegin(); it != path.rend(); ++it) {
        changes += apply_changes(*it);
      }
      current_node = target;
      return changes;
    }

    TreeNode* lca = find_lca(current_node, target);

    int revert_count = 0;
    TreeNode* temp = current_node;
    while (temp != lca && temp != nullptr) {
      revert_count += revert_changes(temp);
      temp = temp->parent;
    }

    std::vector<TreeNode*> forward_path;
    temp = target;
    while (temp != lca && temp != nullptr) {
      forward_path.push_back(temp);
      temp = temp->parent;
    }

    int apply_count = 0;
    for (auto it = forward_path.rbegin(); it != forward_path.rend(); ++it) {
      apply_count += apply_changes(*it);
    }

    current_node = target;
    return revert_count + apply_count;
  }

  /// @brief Get the bound changes needed to go from current to target.
  std::vector<BoundChange> get_changes_to(TreeNode* target) const {
    std::vector<BoundChange> result;
    if (current_node == target) return result;

    if (current_node == nullptr) {
      std::vector<TreeNode*> path;
      TreeNode* t = target;
      while (t != nullptr) {
        path.push_back(t);
        t = t->parent;
      }
      for (auto it = path.rbegin(); it != path.rend(); ++it) {
        for (const auto& bc : (*it)->bound_changes) {
          result.push_back(bc);
        }
      }
      return result;
    }

    TreeNode* lca = find_lca(current_node, target);

    TreeNode* temp = current_node;
    while (temp != lca && temp != nullptr) {
      for (auto it = temp->bound_changes.rbegin(); it != temp->bound_changes.rend(); ++it) {
        result.push_back({it->var_idx, it->new_val, it->old_val, it->is_lb});
      }
      temp = temp->parent;
    }

    std::vector<TreeNode*> forward_path;
    temp = target;
    while (temp != lca && temp != nullptr) {
      forward_path.push_back(temp);
      temp = temp->parent;
    }
    for (auto it = forward_path.rbegin(); it != forward_path.rend(); ++it) {
      for (const auto& bc : (*it)->bound_changes) {
        result.push_back(bc);
      }
    }

    return result;
  }

private:
  int apply_changes(TreeNode* node) {
    for (const auto& bc : node->bound_changes) {
      if (bc.is_lb) current_lb[bc.var_idx] = bc.new_val;
      else          current_ub[bc.var_idx] = bc.new_val;
    }
    return static_cast<int>(node->bound_changes.size());
  }

  int revert_changes(TreeNode* node) {
    for (auto it = node->bound_changes.rbegin(); it != node->bound_changes.rend(); ++it) {
      if (it->is_lb) current_lb[it->var_idx] = it->old_val;
      else          current_ub[it->var_idx] = it->old_val;
    }
    return static_cast<int>(node->bound_changes.size());
  }

  static TreeNode* find_lca(TreeNode* a, TreeNode* b) {
    if (!a) return b;
    if (!b) return a;
    while (a->depth > b->depth && a->parent) a = a->parent;
    while (b->depth > a->depth && b->parent) b = b->parent;
    while (a != b && a && b) {
      a = a->parent;
      b = b->parent;
    }
    return a;
  }
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

/// @brief Sparse cut row plus its validity scope.
struct PoolCut {
  Eigen::SparseVector<double> coeff;  ///< Sparse inequality coeff*x <= rhs
  double rhs;
  int age{0};              ///< Consecutive non-violations (purge when > max_age)
  double best_efficacy;    ///< Best violation/||coeff|| ever seen
  double norm{0.0};        ///< Cached ||coeff||
  std::size_t hash{0};     ///< Structural hash for fast duplicate rejection
  bool domain_only{false}; ///< Propagation-only row; do not inject into node LPs.
  std::uint64_t source_trace_id{0};  ///< Diagnostic transformed-cut source id.
  ValidityScope validity_scope{ValidityScope::GlobalCut};
};

/// @brief Legacy Node structure (kept for compatibility during transition).
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
  std::vector<DomainReasonBound> domain_reason_bounds;
  std::vector<LocalDomainTrailEntry> local_domain_trail;
  std::vector<int> local_branch_positions;
  std::vector<std::uint64_t> dynamic_probe_literal_keys;
  std::vector<LocalBinaryImplication> local_binary_implications;
  std::vector<std::vector<BranchDomainLiteral>> local_conflict_clauses;
  std::vector<ScopedConflictClause> scoped_conflict_clauses;
  std::vector<PoolCut> local_cuts;
  std::uint64_t domain_learning_epoch{0};
  std::uint64_t objective_artifact_epoch{0};
  std::uint64_t domain_closure_epoch{0};
  bool lp_refresh_needed{false};
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
    c.domain_reason_bounds = domain_reason_bounds;
    c.local_domain_trail = local_domain_trail;
    c.local_branch_positions = local_branch_positions;
    c.dynamic_probe_literal_keys = dynamic_probe_literal_keys;
    c.local_binary_implications = local_binary_implications;
    c.local_conflict_clauses = local_conflict_clauses;
    c.scoped_conflict_clauses = scoped_conflict_clauses;
    c.local_cuts = local_cuts;
    c.domain_learning_epoch = domain_learning_epoch;
    c.objective_artifact_epoch = objective_artifact_epoch;
    c.domain_closure_epoch = domain_closure_epoch;
    c.lp_refresh_needed = false;
    c.bound = bound;
    c.estimate = estimate;
    c.depth = depth + 1;
    return c;
  }
};

/// @brief Pseudo-cost estimates for branching variable selection.
/// Tracks sum, sum-of-squares, and count for Welford-style variance computation.
struct PseudoCost {
  double down_sum{0.0};
  double up_sum{0.0};
  double down_sq{0.0};  ///< Sum of squared observations (for variance)
  double up_sq{0.0};    ///< Sum of squared observations (for variance)
  double down_inference_sum{0.0};
  double up_inference_sum{0.0};
  double down_conflict_score{0.0};
  double up_conflict_score{0.0};
  int down_cnt{0};
  int up_cnt{0};
  int down_inference_cnt{0};
  int up_inference_cnt{0};
  int down_cutoff_cnt{0};
  int up_cutoff_cnt{0};

  /// @brief Average down-branch cost (default 1.0 if no observations).
  double down_avg() const { return (down_cnt > 0) ? (down_sum / static_cast<double>(down_cnt)) : 1.0; }
  /// @brief Average up-branch cost (default 1.0 if no observations).
  double up_avg() const { return (up_cnt > 0) ? (up_sum / static_cast<double>(up_cnt)) : 1.0; }

  /// @brief Variance of down-branch cost (0 if < 2 observations).
  double down_var() const {
    if (down_cnt < 2) return 0.0;
    double n = static_cast<double>(down_cnt);
    double mean = down_sum / n;
    return std::max(0.0, (down_sq - n * mean * mean) / (n - 1.0));
  }
  /// @brief Variance of up-branch cost (0 if < 2 observations).
  double up_var() const {
    if (up_cnt < 2) return 0.0;
    double n = static_cast<double>(up_cnt);
    double mean = up_sum / n;
    return std::max(0.0, (up_sq - n * mean * mean) / (n - 1.0));
  }

  /// @brief Lower confidence bound for down-branch (95% CI).
  /// Returns avg - 1.96 * sigma / sqrt(n), clamped to (0, avg].
  double down_lcb() const {
    if (down_cnt < 2) return down_avg();
    double avg = down_avg();
    double sigma = std::sqrt(down_var());
    double se = sigma / std::sqrt(static_cast<double>(down_cnt));
    return std::max(1e-6, avg - 1.96 * se);
  }
  /// @brief Lower confidence bound for up-branch (95% CI).
  double up_lcb() const {
    if (up_cnt < 2) return up_avg();
    double avg = up_avg();
    double sigma = std::sqrt(up_var());
    double se = sigma / std::sqrt(static_cast<double>(up_cnt));
    return std::max(1e-6, avg - 1.96 * se);
  }

  /// @brief Average number of propagated bound changes after a down branch.
  double down_inference_avg() const {
    return (down_inference_cnt > 0)
        ? (down_inference_sum / static_cast<double>(down_inference_cnt))
        : 0.0;
  }
  /// @brief Average number of propagated bound changes after an up branch.
  double up_inference_avg() const {
    return (up_inference_cnt > 0)
        ? (up_inference_sum / static_cast<double>(up_inference_cnt))
        : 0.0;
  }

  /// @brief Fraction of observed branches in this direction that ended in cutoff.
  double down_cutoff_ratio() const {
    return static_cast<double>(down_cutoff_cnt) /
           std::max(1.0, static_cast<double>(down_cutoff_cnt + down_cnt));
  }
  /// @brief Fraction of observed branches in this direction that ended in cutoff.
  double up_cutoff_ratio() const {
    return static_cast<double>(up_cutoff_cnt) /
           std::max(1.0, static_cast<double>(up_cutoff_cnt + up_cnt));
  }

  /// @brief Small multiplicative bonus from inferences/cutoffs/conflicts.
  static double history_multiplier(double inference_avg,
                                   double cutoff_ratio,
                                   double conflict_score) {
    return 1.0 + 0.05 * std::log1p(std::max(0.0, inference_avg))
               + 0.25 * std::max(0.0, cutoff_ratio)
               + 0.05 * std::log1p(std::max(0.0, conflict_score));
  }

  double down_history_multiplier() const {
    return history_multiplier(down_inference_avg(), down_cutoff_ratio(),
                              down_conflict_score);
  }
  double up_history_multiplier() const {
    return history_multiplier(up_inference_avg(), up_cutoff_ratio(),
                              up_conflict_score);
  }

  /// @brief Add a down-branch observation.
  void add_down(double gain) {
    down_sum += gain;
    down_sq += gain * gain;
    ++down_cnt;
  }
  /// @brief Add an up-branch observation.
  void add_up(double gain) {
    up_sum += gain;
    up_sq += gain * gain;
    ++up_cnt;
  }

  /// @brief Add a down-branch inference-count observation.
  void add_down_inference(double inference_count) {
    down_inference_sum += inference_count;
    ++down_inference_cnt;
  }
  /// @brief Add an up-branch inference-count observation.
  void add_up_inference(double inference_count) {
    up_inference_sum += inference_count;
    ++up_inference_cnt;
  }

  /// @brief Record that a branch direction ended in cutoff.
  void add_down_cutoff(int count = 1) { down_cutoff_cnt += count; }
  /// @brief Record that a branch direction ended in cutoff.
  void add_up_cutoff(int count = 1) { up_cutoff_cnt += count; }

  /// @brief Increase directional conflict score.
  void add_down_conflict(double score = 1.0) { down_conflict_score += score; }
  /// @brief Increase directional conflict score.
  void add_up_conflict(double score = 1.0) { up_conflict_score += score; }
};

/// Convert a child objective gain into the unit directional cost predicted by
/// pseudocost branching. Distance is measured before child propagation.
inline double normalized_pseudocost_gain(double gain, double distance) {
  if (!std::isfinite(gain) || gain <= 0.0) return 0.0;
  return gain / std::max(1e-9, std::abs(distance));
}

/// Evidence available for one branch direction at the current node.
/// Proof states stay discrete so failures and cutoffs cannot contaminate the
/// global pseudocost distribution with scale-dependent sentinel gains.
enum class BranchEvidenceState {
  Predicted,
  Measured,
  ExactOptimal,
  ProvenCutoff,
  UnknownFailure,
};

struct BranchDirectionalObservation {
  int var{-1};
  bool is_up{false};
  BranchEvidenceState state{BranchEvidenceState::Predicted};
  double gain{0.0};
};

/// One parent expansion's estimator calibration aggregates. Directional
/// entries compare history-only predictions with solved child bounds; node
/// entries compare the queued node estimate with the best realized child.
struct BranchEstimatorCalibration {
  std::uint64_t node_samples{0};
  double node_predicted_lift_sum{0.0};
  double node_realized_lift_sum{0.0};
  double node_abs_error_sum{0.0};
  double node_squared_error_sum{0.0};
  double node_predicted_sq_sum{0.0};
  double node_realized_sq_sum{0.0};
  double node_cross_sum{0.0};
  std::uint64_t directional_samples{0};
  double directional_predicted_gain_sum{0.0};
  double directional_realized_gain_sum{0.0};
  double directional_abs_error_sum{0.0};
  double directional_squared_error_sum{0.0};
  std::uint64_t directional_rank_samples{0};
  std::uint64_t directional_rank_concordant{0};
};

enum class BranchProbeReuseKind {
  None,
  Warm,
  Exact,
};

/// Dense global-column lookup backed by statistics for branchable columns only.
/// Slot zero is a harmless sentinel for non-branchable columns, allowing legacy
/// pc[col] call sites to retain their global indexing contract.
class CompactPseudoCostTable {
 public:
  CompactPseudoCostTable() = default;

  CompactPseudoCostTable(int num_cols,
                         const std::vector<int>& branchable_cols)
      : num_cols_(std::max(0, num_cols)),
        col_to_slot_(static_cast<std::size_t>(num_cols_), 0),
        values_(1) {
    values_.reserve(branchable_cols.size() + 1);
    active_cols_.reserve(branchable_cols.size());
    for (int col : branchable_cols) {
      if (col < 0 || col >= num_cols_ ||
          col_to_slot_[static_cast<std::size_t>(col)] != 0) {
        continue;
      }
      col_to_slot_[static_cast<std::size_t>(col)] =
          static_cast<int>(values_.size());
      values_.emplace_back();
      active_cols_.push_back(col);
    }
  }

  PseudoCost& operator[](int col) {
    return values_[slot(col)];
  }
  const PseudoCost& operator[](int col) const {
    return values_[slot(col)];
  }

  std::size_t size() const { return static_cast<std::size_t>(num_cols_); }
  std::size_t active_size() const { return active_cols_.size(); }
  const std::vector<int>& active_cols() const { return active_cols_; }
  std::size_t storage_bytes() const {
    return col_to_slot_.capacity() * sizeof(int) +
           active_cols_.capacity() * sizeof(int) +
           values_.capacity() * sizeof(PseudoCost);
  }

 private:
  std::size_t slot(int col) const {
    if (col < 0 || col >= num_cols_) return 0;
    return static_cast<std::size_t>(
        col_to_slot_[static_cast<std::size_t>(col)]);
  }

  int num_cols_{0};
  std::vector<int> col_to_slot_;
  std::vector<int> active_cols_;
  std::vector<PseudoCost> values_;
};

/// @brief Check if a variable has integer or binary type.
inline bool is_integer_type(const VariableMeta& v) {
  return v.type == VarType::Integer || v.type == VarType::Binary;
}

/// @brief Check if a value is integral within tolerance.
inline bool is_integral(double x, double tol) {
  return std::abs(x - std::round(x)) <= tol;
}

/// @brief Cut family identifiers for efficacy tracking (P2.1).
enum class CutFamily : int {
  Gomory = 0,
  MIR = 1,
  Cover = 2,
  Clique = 3,
  ZeroHalf = 4,
  Count = 5  // sentinel for array sizing
};

/// @brief Per-family cut statistics for adaptive generation (P2.1).
/// Tracks moving average efficacy over the last N rounds and disables
/// families that fall below min_family_efficacy (0.001).
struct CutFamilyStats {
  static constexpr int kHistorySize = 5;
  static constexpr double kMinEfficacy = 0.001;
  
  int rounds{0};                    ///< Number of rounds with this family
  int cuts_total{0};                ///< Total cuts generated
  int attempts{0};                  ///< Rounds in which this family was called
  int zero_rounds{0};               ///< Called rounds that admitted zero cuts
  int skipped_rounds{0};            ///< Rounds skipped by adaptive gating
  int budget_total{0};              ///< Total admission budget spent
  double efficacy_sum{0.0};         ///< Running sum of efficacy values
  double history[kHistorySize]{};   ///< Circular buffer of per-round avg efficacy
  int history_idx{0};               ///< Current index in circular buffer
  bool disabled{false};             ///< True if family is disabled due to low efficacy
  
  /// @brief Add a round's efficacy data and update moving average.
  void add_round(double avg_efficacy) {
    history[history_idx] = avg_efficacy;
    history_idx = (history_idx + 1) % kHistorySize;
    ++rounds;
    
    // Check if we should disable this family
    if (rounds >= kHistorySize) {
      double sum = 0.0;
      for (int i = 0; i < kHistorySize; ++i) sum += history[i];
      double mavg = sum / static_cast<double>(kHistorySize);
      disabled = (mavg < kMinEfficacy);
    }
  }
  
  /// @brief Check if this family should be skipped.
  bool should_skip() const { return disabled; }
  
  /// @brief Get the moving average efficacy (0 if not enough rounds).
  double moving_avg() const {
    if (rounds < kHistorySize) return 1.0;  // Be optimistic early
    double sum = 0.0;
    for (int i = 0; i < kHistorySize; ++i) sum += history[i];
    return sum / static_cast<double>(kHistorySize);
  }
};

/// @brief Container for all cut family statistics.
struct CutFamilyTracker {
  CutFamilyStats families[static_cast<int>(CutFamily::Count)];
  
  CutFamilyStats& get(CutFamily f) { return families[static_cast<int>(f)]; }
  const CutFamilyStats& get(CutFamily f) const { return families[static_cast<int>(f)]; }
  
  bool should_skip(CutFamily f) const { return families[static_cast<int>(f)].should_skip(); }
};

/// @brief Feasible solution stored in the solution pool.
struct PoolSolution {
  Eigen::VectorXd x;
  double obj;
};

/// @brief Priority queue item wrapping a Node with its queue key.
struct QueueItem {
  Node node;
  double key{0.0};
};

/// @brief Min-key comparator for QueueItem (priority queue is max-heap, so invert).
struct QueueItemMinKey {
  bool operator()(const QueueItem& a, const QueueItem& b) const {
    return a.key > b.key;
  }
};

/// @brief Request sent from explorer threads to the cut worker.
struct CutRequest {
  SimplexResult simplex_result;   ///< Moved from explorer (O(1) via Eigen move)
  Eigen::VectorXd x_relax;       ///< LP solution at node
  Eigen::VectorXd node_lb, node_ub;
  int node_depth{0};
};

/// @brief Pseudo-cost update record from a child node solve.
struct PCUpdate {
  int var;
  bool is_up;
  double branch_distance{1.0};
  bool has_gain{false};
  double gain{0.0};
  bool suppress_gain_update{false};
  bool has_inference{false};
  double inference_count{0.0};
  int cutoff_count{0};
  double conflict_score{0.0};

  void apply(PseudoCost& pseudocost) const {
    if (has_gain && !suppress_gain_update) {
      const double unit_gain =
          normalized_pseudocost_gain(gain, branch_distance);
      if (is_up) pseudocost.add_up(unit_gain);
      else pseudocost.add_down(unit_gain);
    }
    if (has_inference) {
      if (is_up) pseudocost.add_up_inference(inference_count);
      else pseudocost.add_down_inference(inference_count);
    }
    if (cutoff_count > 0) {
      if (is_up) pseudocost.add_up_cutoff(cutoff_count);
      else pseudocost.add_down_cutoff(cutoff_count);
    }
    if (conflict_score > 0.0) {
      if (is_up) pseudocost.add_up_conflict(conflict_score);
      else pseudocost.add_down_conflict(conflict_score);
    }
  }
};

/// @brief Result of processing a single child node in the B&C tree.
struct NodeResult {
  Node child_down, child_up;
  bool down_valid{false}, up_valid{false};
  std::vector<PCUpdate> pc_updates;
  std::vector<PoolSolution> solutions;
  std::vector<PoolCut> new_cuts;
  int lp_solves{0};
  int cuts_added{0};
  int pool_cuts_separated{0};
};

/// @brief Per-thread workspace for parallel B&C.
struct ThreadWorkspace {
  StandardFormLP node_sf;   ///< Per-thread copy of base_sf
};

/// @brief Result from solving an LP relaxation at a B&C node.
struct LPRelaxationResult {
  SolveResult primal;
  double dual_bound{kInf};
  std::shared_ptr<SimplexBasis> basis_hint;
  std::shared_ptr<SimplexResult> simplex;
  Eigen::VectorXd row_duals;
  // The LP is solved and certified, but a requested IPM-to-simplex crossover
  // did not produce a usable basis. Callers must route subsequent LPs to IPM
  // instead of silently cold-starting simplex from this basis-free result.
  bool requires_ipm_nodes{false};
};

/// @brief HiGHS-aligned persistent LP/basis state for the root B&C phase.
///
/// HiGHS keeps a single `Highs lpsolver` instance whose simplex basis and
/// factorization survive across cuts / pump / dive / RENS. We mimic that
/// pattern with a small object owning:
///   * `base_sf_`: the cut-free root `StandardFormLP` (Ruiz-scaled), built
///     once and reused by every phase that does not need cuts in the LP.
///   * `base_basis_`: a snapshot of the simplex basis taken **before** any
///     cut row was committed to the LP. This is the analog of HiGHS'
///     `firstrootbasis`. Dimensionally compatible with `base_sf_`, so any
///     phase reaching for `base_sf_` (pump, dive, RENS root) can warm-start
///     simplex without dimension-mismatch fallback to IPM.
///   * `cut_basis_`: the latest basis from the cut-augmented LP solve.
///     Updated every time the cut loop produces a new basis. Used by the
///     xpool re-solve loop and by tree nodes for warm starts.
///
/// This is a deliberately thin abstraction: the actual `LPModel` (which
/// grows as cuts are committed) is still owned by the legacy code path; the
/// persistent state focuses on the two pieces that drive the worst dimension
/// mismatches in the pump and post-cut phases.
class PersistentLPState {
 public:
  /// Build the cut-free root `StandardFormLP` from `base_lp` and Ruiz-scale it.
  /// Lazy: returns immediately on a second call.
  StandardFormLP& ensure_base_sf(const LPModel& base_lp) {
    if (!base_sf_built_) {
      base_sf_ = build_standard_form_lp(base_lp);
      ruiz_scale_standard_form(base_sf_);
      base_sf_built_ = true;
    }
    return base_sf_;
  }

  /// Capture the basis snapshot taken before any cut was added. Idempotent;
  /// later calls overwrite if `force` is true.
  void set_base_basis(std::shared_ptr<SimplexBasis> b, bool force = false) {
    if (force || !base_basis_) base_basis_ = std::move(b);
  }

  /// Capture / update the basis matching the cut-augmented LP. May be
  /// called repeatedly through the cut loop.
  void set_cut_basis(std::shared_ptr<SimplexBasis> b) {
    cut_basis_ = std::move(b);
  }

  bool has_base_sf()    const { return base_sf_built_; }
  bool has_base_basis() const { return static_cast<bool>(base_basis_); }
  bool has_cut_basis()  const { return static_cast<bool>(cut_basis_); }

  /// Drop the cached `base_sf_`. The next call to `ensure_base_sf` will
  /// rebuild it. Used when the underlying `base_lp` has been mutated by
  /// row additions (e.g., committed cuts that we are not yet representing
  /// as a separate cut layer in this state).
  void invalidate_base_sf() {
    base_sf_ = StandardFormLP{};
    base_sf_built_ = false;
  }

  StandardFormLP&       base_sf_mut() { return base_sf_; }
  const StandardFormLP& base_sf() const { return base_sf_; }
  std::shared_ptr<SimplexBasis> base_basis() const { return base_basis_; }
  std::shared_ptr<SimplexBasis> cut_basis()  const { return cut_basis_; }

  /// Returns a basis hint that is *dimensionally compatible* with `base_sf_`.
  /// Prefers `base_basis_` (cut-free). If only `cut_basis_` exists and
  /// happens to match `base_sf_` dimensions (e.g., no cuts were committed),
  /// returns it as a fallback. Otherwise returns nullptr — the caller must
  /// then either rebuild the basis or fall back to IPM.
  std::shared_ptr<SimplexBasis> basis_hint_for_base() const {
    if (base_basis_) return base_basis_;
    if (cut_basis_ && base_sf_built_ &&
        cut_basis_->rows == base_sf_.A.rows() &&
        cut_basis_->cols == base_sf_.A.cols()) {
      return cut_basis_;
    }
    return nullptr;
  }

 private:
  StandardFormLP base_sf_{};
  bool base_sf_built_{false};
  std::shared_ptr<SimplexBasis> base_basis_;
  std::shared_ptr<SimplexBasis> cut_basis_;
};

}  // namespace mipsolvers::engine::detail

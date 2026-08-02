/// @file bc_types.hpp
/// @brief Core data types for the branch-and-cut solver.
///
/// Contains node representations, pseudo-cost tracking, queue structures,
/// thread workspace, and LP/cut request types used throughout the B&C modules.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
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

/// Sparse queued-node domain relative to the queue's shared root domain.
/// Active nodes use dense lb/ub work vectors; queued nodes release those
/// vectors and retain one entry only for columns whose bound pair differs.
struct SparseNodeBound {
  int var_idx{-1};
  double lb{0.0};
  double ub{0.0};
};

struct NodeQueueDomainStorageStats {
  std::uint64_t compactions{0};
  std::uint64_t materializations{0};
  std::uint64_t dense_bound_values_released{0};
  std::uint64_t compact_entries_created{0};
  std::uint64_t current_compact_entries{0};
  std::uint64_t current_compact_nodes{0};
  std::uint64_t peak_compact_entries{0};
  std::uint64_t peak_compact_nodes{0};
  std::uint64_t compaction_failures{0};
  std::uint64_t materialization_failures{0};
};

/// Auditable counts for branch-child payload inheritance.  An inherited
/// element is counted only when it remains in shared immutable storage at
/// child construction; this deliberately does not estimate bytes or runtime.
struct NodePayloadSharingStats {
  std::uint64_t child_creations{0};
  std::uint64_t shared_payload_vectors{0};
  std::uint64_t shared_payload_elements{0};
};

/// Vector-like storage whose copies share immutable data until a mutation.
/// Read-only iteration and indexing deliberately do not detach, even when the
/// wrapper itself is non-const. Call write() for element-wise mutation.
template <typename T>
class CopyOnWriteVector {
 public:
  using value_type = T;
  using size_type = typename std::vector<T>::size_type;
  using const_iterator = typename std::vector<T>::const_iterator;

  CopyOnWriteVector() = default;
  CopyOnWriteVector(const CopyOnWriteVector&) = default;
  CopyOnWriteVector(CopyOnWriteVector&&) noexcept = default;
  CopyOnWriteVector& operator=(const CopyOnWriteVector&) = default;
  CopyOnWriteVector& operator=(CopyOnWriteVector&&) noexcept = default;

  explicit CopyOnWriteVector(std::vector<T> values)
      : values_(std::make_shared<std::vector<T>>(std::move(values))) {}

  CopyOnWriteVector& operator=(std::vector<T> values) {
    values_ = values.empty()
                  ? nullptr
                  : std::make_shared<std::vector<T>>(std::move(values));
    return *this;
  }

  bool empty() const noexcept { return !values_ || values_->empty(); }
  size_type size() const noexcept { return values_ ? values_->size() : 0; }

  const std::vector<T>& read() const noexcept {
    static const std::vector<T> empty_values;
    return values_ ? *values_ : empty_values;
  }

  std::vector<T>& write() {
    if (!values_) {
      values_ = std::make_shared<std::vector<T>>();
    } else if (values_.use_count() != 1) {
      values_ = std::make_shared<std::vector<T>>(*values_);
    }
    return *values_;
  }

  operator const std::vector<T>&() const noexcept { return read(); }
  const std::vector<T>* read_ptr() const noexcept { return &read(); }
  std::vector<T>* write_ptr() { return &write(); }

  const_iterator begin() const noexcept { return read().begin(); }
  const_iterator end() const noexcept { return read().end(); }
  const T& operator[](size_type index) const { return read()[index]; }
  const T& operator[](size_type index) { return read()[index]; }
  const T& back() const { return read().back(); }
  const T& back() { return read().back(); }
  T& mutable_back() { return write().back(); }

  void clear() {
    if (!empty()) write().clear();
  }

  void reserve(size_type count) {
    if (count > read().capacity()) write().reserve(count);
  }

  void resize(size_type count) {
    if (count != size()) write().resize(count);
  }

  void push_back(const T& value) { write().push_back(value); }
  void push_back(T&& value) { write().push_back(std::move(value)); }

  template <typename... Args>
  T& emplace_back(Args&&... args) {
    return write().emplace_back(std::forward<Args>(args)...);
  }

  bool shares_storage_with(const CopyOnWriteVector& other) const noexcept {
    return values_ && values_ == other.values_;
  }

  long storage_use_count() const noexcept {
    return values_ ? values_.use_count() : 0;
  }

 private:
  std::shared_ptr<std::vector<T>> values_;
};

/// @brief Node representation used by the serial and parallel native trees.
struct Node {
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  std::vector<SparseNodeBound> compact_domain;
  Eigen::Index compact_domain_size{0};
  Eigen::VectorXd x_relax;
  Eigen::VectorXd x_seed;
  std::vector<int> fractional_branchables;
  double fractional_branchables_tol{0.0};
  bool fractional_branchables_valid{false};
  std::optional<Eigen::VectorXd> nlp_down_scores;
  std::optional<Eigen::VectorXd> nlp_up_scores;
  std::shared_ptr<SimplexBasis> basis_hint;
  bool has_live_factor_telemetry{false};
  SparseFactorTelemetry live_factor_telemetry{};
  int ipm_iterations{0};  ///< Number of IPM iterations for this node's LP solve
  CopyOnWriteVector<BranchDomainLiteral> branch_reasons;
  CopyOnWriteVector<DomainReasonBound> domain_reason_bounds;
  CopyOnWriteVector<LocalDomainTrailEntry> local_domain_trail;
  CopyOnWriteVector<int> local_branch_positions;
  CopyOnWriteVector<std::uint64_t> dynamic_probe_literal_keys;
  CopyOnWriteVector<LocalBinaryImplication> local_binary_implications;
  CopyOnWriteVector<std::vector<BranchDomainLiteral>> local_conflict_clauses;
  CopyOnWriteVector<ScopedConflictClause> scoped_conflict_clauses;
  CopyOnWriteVector<PoolCut> local_cuts;
  std::uint64_t domain_learning_epoch{0};
  std::uint64_t objective_artifact_epoch{0};
  std::uint64_t domain_closure_epoch{0};
  bool lp_refresh_needed{false};
  double bound{kInf};
  double estimate{kInf};
  int depth{0};

  /// Create a lightweight child node copying only lb/ub (skips x_relax/x_seed copy).
  Node branch_child(NodePayloadSharingStats* sharing_stats = nullptr) const {
    Node c;
    c.lb = lb;
    c.ub = ub;
    c.compact_domain = compact_domain;
    c.compact_domain_size = compact_domain_size;
    c.fractional_branchables_valid = false;
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
    if (sharing_stats != nullptr) {
      ++sharing_stats->child_creations;
      const auto count_shared = [&](const auto& parent_payload,
                                    const auto& child_payload) {
        if (!parent_payload.empty() &&
            parent_payload.shares_storage_with(child_payload)) {
          ++sharing_stats->shared_payload_vectors;
          sharing_stats->shared_payload_elements +=
              static_cast<std::uint64_t>(parent_payload.size());
        }
      };
      count_shared(branch_reasons, c.branch_reasons);
      count_shared(domain_reason_bounds, c.domain_reason_bounds);
      count_shared(local_domain_trail, c.local_domain_trail);
      count_shared(local_branch_positions, c.local_branch_positions);
      count_shared(dynamic_probe_literal_keys, c.dynamic_probe_literal_keys);
      count_shared(local_binary_implications, c.local_binary_implications);
      count_shared(local_conflict_clauses, c.local_conflict_clauses);
      count_shared(scoped_conflict_clauses, c.scoped_conflict_clauses);
      count_shared(local_cuts, c.local_cuts);
    }
    return c;
  }
};

inline bool node_domain_is_compact(const Node& node) {
  return node.compact_domain_size > 0 && node.lb.size() == 0 &&
         node.ub.size() == 0;
}

inline bool compact_node_domain(Node& node, const Eigen::VectorXd& root_lb,
                                const Eigen::VectorXd& root_ub,
                                std::size_t* entries_out = nullptr) {
  if (entries_out != nullptr) *entries_out = 0;
  if (node_domain_is_compact(node)) {
    if (entries_out != nullptr) *entries_out = node.compact_domain.size();
    return true;
  }
  if (root_lb.size() <= 0 || root_lb.size() != root_ub.size() ||
      node.lb.size() != root_lb.size() || node.ub.size() != root_ub.size()) {
    return false;
  }

  std::vector<SparseNodeBound> compact;
  compact.reserve(node.local_domain_trail.size() + 2);
  for (Eigen::Index j = 0; j < root_lb.size(); ++j) {
    const double node_lb = node.lb[j] == 0.0 ? 0.0 : node.lb[j];
    const double node_ub = node.ub[j] == 0.0 ? 0.0 : node.ub[j];
    const double base_lb = root_lb[j] == 0.0 ? 0.0 : root_lb[j];
    const double base_ub = root_ub[j] == 0.0 ? 0.0 : root_ub[j];
    if (node_lb == base_lb && node_ub == base_ub) continue;
    compact.push_back(
        SparseNodeBound{static_cast<int>(j), node_lb, node_ub});
  }

  node.compact_domain = std::move(compact);
  node.compact_domain_size = root_lb.size();
  Eigen::VectorXd empty_lb;
  Eigen::VectorXd empty_ub;
  node.lb.swap(empty_lb);
  node.ub.swap(empty_ub);
  if (entries_out != nullptr) *entries_out = node.compact_domain.size();
  return true;
}

inline bool materialize_node_domain(Node& node,
                                    const Eigen::VectorXd& root_lb,
                                    const Eigen::VectorXd& root_ub) {
  if (!node_domain_is_compact(node)) {
    return node.lb.size() == root_lb.size() &&
           node.ub.size() == root_ub.size();
  }
  if (node.compact_domain_size != root_lb.size() ||
      root_lb.size() != root_ub.size()) {
    return false;
  }

  for (const auto& change : node.compact_domain) {
    if (change.var_idx < 0 || change.var_idx >= root_lb.size()) return false;
  }
  node.lb = root_lb;
  node.ub = root_ub;
  for (const auto& change : node.compact_domain) {
    node.lb[change.var_idx] = change.lb;
    node.ub[change.var_idx] = change.ub;
  }
  node.compact_domain.clear();
  node.compact_domain_size = 0;
  return true;
}

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

/// @brief Cut family identifiers for efficacy tracking.
enum class CutFamily : int {
  Gomory = 0,
  MIR = 1,
  Cover = 2,
  Clique = 3,
  ZeroHalf = 4,
  Count = 5  // sentinel for array sizing
};

/// @brief Per-family cut statistics for adaptive generation.
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

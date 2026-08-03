/// @file bc_utils_core.hpp
/// @brief Core B&C utility/type declarations (part 1 of the bc_utils.hpp split).
/// Included transitively via bc_utils.hpp; do not include directly.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"

namespace mipsolvers::engine::detail {

class CliqueTable;
class SharedConflictPool;
class BinaryImplicationGraph;
class VariableBoundTable;

struct CrashBasisRecoveryStats {
    int candidate_rows{0};
    int candidate_edges{0};
    int attempted_columns{0};
    int stable_pivot_rejections{0};
    int structural_matches{0};
    int selected_swaps{0};
    int rank_repairs{0};
    int crash_refactors{0};
    double mean_partition_score{0.0};
    double mean_row_dual_score{0.0};
    double mean_activity_score{0.0};
    double min_accepted_relative_pivot{0.0};
    double mean_accepted_relative_pivot{0.0};
    bool rank_valid{false};
    bool passes_screen{false};
};

// ── Utility functions (bc_utils.cpp) ─────────────────────────────────────

/// @brief Compute objective value (negated for maximization).
/// @param c Cost vector.
/// @param x Decision variables.
/// @param sense Minimize or Maximize.
/// @return c^T x for minimization, -c^T x for maximization.
double objective_value(const Eigen::VectorXd& c, const Eigen::VectorXd& x, Sense sense);

/// @brief Clamp each element of x to [lb, ub].
Eigen::VectorXd clamp_to_bounds(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& lb,
                                const Eigen::VectorXd& ub);

/// Shared tolerance for incumbent-bound pruning checks.
inline constexpr double kIncumbentPruneTol = 1e-12;

/// Shared feasibility tolerance for lightweight integer heuristics.
inline constexpr double kNodeFeasibilityTol = 1e-6;

/// Estimate-guided queue priority while keeping exact lower bounds separate.
inline double node_selection_priority(const Node& node) {
    if (!std::isfinite(node.bound)) {
        return kInf;
    }
    if (!std::isfinite(node.estimate)) {
        return node.bound;
    }
    return 0.5 * (node.bound + std::max(node.bound, node.estimate));
}

struct QueueLowerBoundAudit {
    double indexed_min{kInf};
    double computed_min{kInf};
    double abs_error{0.0};
    int indexed_nodes{0};
    int live_nodes{0};
    bool size_mismatch{false};
    bool finite_mismatch{false};
    bool ok{true};
};

bool bounds_consistent(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/// @brief Rebuild the ordered local domain trail from branch and reason bounds.
void rebuild_local_domain_trail(Node& node,
                                int n,
                                const Eigen::VectorXd* root_lb = nullptr,
                                const Eigen::VectorXd* root_ub = nullptr);

/// @brief Append a reason bound and keep its HiGHS-style local trail metadata.
void append_domain_reason_bound(Node& node,
                                DomainReasonBound rb,
                                int n,
                                const Eigen::VectorXd* root_lb = nullptr,
                                const Eigen::VectorXd* root_ub = nullptr);

/// @brief First-class local-trail resolution for row-dual target bounds.
///
/// This is the native analogue of HiGHS ConflictSet::resolveLinearLeq/Geq:
/// select only the local bound changes needed to close the row proof, relax or
/// drop redundant changes along the previous-bound chain, then resolve local
/// propagation artifacts backward to a reason-side frontier that can be
/// consumed by other queued nodes.
enum class DualProofResolutionStatus {
    Success,
    InvalidInput,
    ActivityFailed,
    MissingReason,
    ScopeBlocked,
    LiteralLimit
};

inline bool dual_proof_resolution_success(DualProofResolutionStatus status) {
    return status == DualProofResolutionStatus::Success;
}

inline bool dual_proof_resolution_missing_reason(
    DualProofResolutionStatus status) {
    return status == DualProofResolutionStatus::MissingReason;
}

inline bool dual_proof_resolution_scope_blocked(
    DualProofResolutionStatus status) {
    return status == DualProofResolutionStatus::ScopeBlocked ||
           status == DualProofResolutionStatus::LiteralLimit;
}

inline bool dual_proof_resolution_activity_failed(
    DualProofResolutionStatus status) {
    return status != DualProofResolutionStatus::Success &&
           !dual_proof_resolution_missing_reason(status) &&
           !dual_proof_resolution_scope_blocked(status);
}

struct DualProofTrailResolution {
    DualProofResolutionStatus status{DualProofResolutionStatus::InvalidInput};
    bool valid{false};
    bool cutoff_conflict{false};
    bool has_proved_target_bound{false};
    BranchDomainLiteral proved_target_bound;
    BranchDomainLiteral flipped_target;
    std::vector<BranchDomainLiteral> proof_frontier;
    std::vector<BranchDomainLiteral> resolved_frontier;
    std::vector<BranchDomainLiteral> clause;
    std::vector<std::vector<BranchDomainLiteral>> reconvergence_clauses;
    double proof_margin{0.0};
    double proof_budget{0.0};
    double proof_activity{0.0};
    double resolved_activity{0.0};
};

/// @brief First-class local-trail resolution for a row-dual cutoff proof.
///
/// This is the native analogue of HiGHS
/// `ConflictSet::conflictAnalysis(proofinds, proofvals, proofrhs)`: prove the
/// row cutoff from active local proof-side bounds, relax/drop redundant bounds
/// using the previous-bound chain, then resolve the selected local changes to a
/// reason-side frontier that can be inserted into the global conflict pool.
struct DualProofConflictResolution {
    DualProofResolutionStatus status{DualProofResolutionStatus::InvalidInput};
    bool valid{false};
    std::vector<BranchDomainLiteral> proof_frontier;
    std::vector<BranchDomainLiteral> resolved_frontier;
    std::vector<std::vector<BranchDomainLiteral>> conflict_clauses;
    double root_min_activity{0.0};
    double node_min_activity{0.0};
    double cutoff_rhs{0.0};
    double proof_margin{0.0};
    double proof_activity{0.0};
    double resolved_activity{0.0};
};

DualProofResolutionStatus resolve_dual_proof_target_bound_from_local_trail(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd* x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    const Eigen::SparseVector<double>& coeff,
    double rhs,
    DualProofTrailResolution& out,
    const Eigen::SparseVector<double>* priority_coeff = nullptr);

DualProofResolutionStatus resolve_dual_proof_conflict_from_local_trail(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    double int_tol,
    double lp_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const Eigen::SparseVector<double>& coeff,
    double rhs,
    int max_literals,
    DualProofConflictResolution& out);

/// Lightweight sequential search queue with separate priority and lower-bound views.
class NodeQueue {
 private:
    class DomainMaterializationGuard {
     public:
        explicit DomainMaterializationGuard(NodeQueue& owner);
        ~DomainMaterializationGuard() noexcept;

        DomainMaterializationGuard(const DomainMaterializationGuard&) = delete;
        DomainMaterializationGuard& operator=(
            const DomainMaterializationGuard&) = delete;

     private:
        NodeQueue& owner_;
    };

 public:
    explicit NodeQueue(NodeSelection mode) : mode_(mode) {}

    void configure_domain_signature(const std::vector<char>& branchable_cols,
                                    const Eigen::VectorXd& root_lb,
                                    const Eigen::VectorXd& root_ub,
                                    double tol = 1e-9) {
        DomainMaterializationGuard domain_guard(*this);
        root_domain_lb_ = root_lb;
        root_domain_ub_ = root_ub;
        domain_storage_enabled_ =
            root_lb.size() > 0 && root_lb.size() == root_ub.size();
        domain_signature_cols_.clear();
        domain_signature_cols_.reserve(branchable_cols.size());
        for (std::size_t j = 0; j < branchable_cols.size(); ++j) {
            if (branchable_cols[j] != 0) {
                domain_signature_cols_.push_back(static_cast<int>(j));
            }
        }
        domain_signature_size_ = root_lb.size();
        (void)tol;
        domain_signature_enabled_ =
            !domain_signature_cols_.empty() &&
            root_lb.size() == root_ub.size();
        signature_hash_to_ids_.clear();
        id_to_signature_hash_.clear();
        if (!domain_signature_enabled_) return;
        std::vector<std::int64_t> doomed;
        std::unordered_set<std::int64_t> doomed_set;
        std::vector<std::int64_t> ids;
        ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            ids.push_back(id);
        }
        std::sort(ids.begin(), ids.end());
        for (std::int64_t id : ids) {
            if (doomed_set.count(id) != 0) continue;
            const auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) continue;
            const auto signature_hash =
                make_domain_signature_hash(node_it->second);
            if (!signature_hash.has_value()) continue;
            auto& bucket = signature_hash_to_ids_[*signature_hash];
            auto existing = std::find_if(
                bucket.begin(), bucket.end(), [&](std::int64_t candidate_id) {
                    const auto candidate = nodes_.find(candidate_id);
                    return candidate != nodes_.end() &&
                           same_domain(candidate->second, node_it->second);
                });
            if (existing == bucket.end()) {
                bucket.push_back(id);
                id_to_signature_hash_[id] = *signature_hash;
                continue;
            }
            const auto keep_it = nodes_.find(*existing);
            if (keep_it != nodes_.end() &&
                keep_it->second.bound <=
                    node_it->second.bound +
                        signature_bound_tol(node_it->second.bound)) {
                doomed.push_back(id);
                doomed_set.insert(id);
            } else {
                doomed.push_back(*existing);
                doomed_set.insert(*existing);
                id_to_signature_hash_.erase(*existing);
                *existing = id;
                id_to_signature_hash_[id] = *signature_hash;
            }
        }
        for (std::int64_t id : doomed) erase_id(id);
    }

    const NodeQueueDomainStorageStats& domain_storage_stats() const {
        return domain_storage_stats_;
    }

    void push(Node node, bool has_incumbent) {
        if (mode_ == NodeSelection::DepthFirst) {
            push_dfs(std::move(node));
            return;
        }
        if (mode_ == NodeSelection::Hybrid && !has_incumbent) {
            push_dfs(std::move(node));
            return;
        }
        push_priority(std::move(node));
    }

    void push_dfs(Node node) {
        const auto id = store(std::move(node));
        if (id < 0) return;
        const double bound = nodes_.at(id).bound;
        dfs_.push_back(id);
        dfs_bounds_.emplace(bound, id);
    }

    void push_priority(Node node) {
        const auto id = store(std::move(node));
        if (id < 0) return;
        const Node& stored = nodes_.at(id);
        priority_.emplace(node_selection_priority(stored), id);
        bounds_.emplace(stored.bound, id);
    }

    bool pop(Node& out, bool has_incumbent) {
        if (mode_ == NodeSelection::DepthFirst) {
            return pop_dfs(out);
        }
        if (mode_ == NodeSelection::Hybrid && !has_incumbent && !dfs_.empty()) {
            hybrid_post_incumbent_active_ = false;
            hybrid_queue_leaves_ = 0;
            hybrid_last_lower_bound_leave_ = 0;
            return pop_dfs(out);
        }
        if (mode_ == NodeSelection::Hybrid && has_incumbent) {
            if (!hybrid_post_incumbent_active_) {
                hybrid_post_incumbent_active_ = true;
                hybrid_queue_leaves_ = 0;
                hybrid_last_lower_bound_leave_ = -kHybridBestBoundPeriod;
            }
            const double best_lb_before = lower_bound();
            const bool force_best_bound =
                hybrid_queue_leaves_ - hybrid_last_lower_bound_leave_ >=
                kHybridBestBoundPeriod;
            bool popped = false;
            if (force_best_bound) {
                popped = pop_bestbound(out);
            } else if (!priority_.empty()) {
                popped = pop_priority(out);
            } else {
                popped = pop_bestbound(out);
            }
            if (popped) {
                ++hybrid_queue_leaves_;
                const double lb_tol =
                    std::max(1e-9, 1e-12 *
                                      std::max(1.0, std::abs(best_lb_before)));
                if (force_best_bound ||
                    (std::isfinite(best_lb_before) &&
                     std::isfinite(out.bound) &&
                     out.bound <= best_lb_before + lb_tol)) {
                    hybrid_last_lower_bound_leave_ = hybrid_queue_leaves_;
                }
            }
            return popped;
        }
        if (!dfs_.empty()) {
            return pop_dfs(out);
        }
        return pop_priority(out);
    }

    bool pop_bestbound(Node& out) {
        const bool have_priority_bound = !bounds_.empty();
        const bool have_dfs_bound = !dfs_bounds_.empty();
        if (!have_priority_bound && !have_dfs_bound) {
            return false;
        }

        if (have_dfs_bound &&
            (!have_priority_bound || dfs_bounds_.begin()->first <= bounds_.begin()->first)) {
            return take_id(dfs_bounds_.begin()->second, out);
        }
        return take_id(bounds_.begin()->second, out);
    }

    double lower_bound() const {
        double lb = kInf;
        if (!bounds_.empty()) {
            lb = bounds_.begin()->first;
        }
        if (!dfs_bounds_.empty()) {
            lb = std::min(lb, dfs_bounds_.begin()->first);
        }
        return lb;
    }

    QueueLowerBoundAudit audit_lower_bound_index(double tol = 1e-9) const {
        QueueLowerBoundAudit audit;
        audit.indexed_min = lower_bound();
        audit.indexed_nodes = size();
        audit.live_nodes = static_cast<int>(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)id;
            audit.computed_min = std::min(audit.computed_min, node.bound);
        }

        const bool indexed_finite = std::isfinite(audit.indexed_min);
        const bool computed_finite = std::isfinite(audit.computed_min);
        audit.finite_mismatch = indexed_finite != computed_finite;
        if (indexed_finite && computed_finite) {
            audit.abs_error = std::abs(audit.indexed_min - audit.computed_min);
        } else {
            audit.abs_error = audit.finite_mismatch ? kInf : 0.0;
        }
        audit.size_mismatch = audit.indexed_nodes != audit.live_nodes;
        const double scale = indexed_finite && computed_finite
            ? std::max({1.0, std::abs(audit.indexed_min),
                        std::abs(audit.computed_min)})
            : 1.0;
        const double allowed = std::max(1e-9, tol) * scale;
        audit.ok = !audit.size_mismatch && !audit.finite_mismatch &&
                   audit.abs_error <= allowed;
        return audit;
    }

    int inherit_root_domain(
        const Eigen::VectorXd& root_lb,
        const Eigen::VectorXd& root_ub,
        const std::function<double(const Node&)>
            *domain_objective_lower_bound_cb = nullptr,
        bool has_incumbent = false,
        double incumbent_obj = kInf,
        double prune_tol = kIncumbentPruneTol,
        std::uint64_t* tightened_out = nullptr,
        std::uint64_t* refresh_marked_out = nullptr,
        std::uint64_t* domain_bound_lift_nodes_out = nullptr,
        double* domain_bound_lift_sum_out = nullptr,
        double* domain_bound_lift_max_out = nullptr) {
        if (tightened_out != nullptr) *tightened_out = 0;
        if (refresh_marked_out != nullptr) *refresh_marked_out = 0;
        if (domain_bound_lift_nodes_out != nullptr) {
            *domain_bound_lift_nodes_out = 0;
        }
        if (domain_bound_lift_sum_out != nullptr) {
            *domain_bound_lift_sum_out = 0.0;
        }
        if (domain_bound_lift_max_out != nullptr) {
            *domain_bound_lift_max_out = 0.0;
        }
        if (root_lb.size() != root_ub.size()) return 0;
        DomainMaterializationGuard domain_guard(*this);
        root_domain_lb_ = root_lb;
        root_domain_ub_ = root_ub;
        domain_storage_enabled_ = root_lb.size() > 0;
        if (nodes_.empty()) return 0;

        std::vector<std::int64_t> ids;
        ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            ids.push_back(id);
        }

        if (domain_signature_enabled_) {
            signature_hash_to_ids_.clear();
            id_to_signature_hash_.clear();
        }

        constexpr double tol = 1e-9;
        int pruned = 0;
        for (std::int64_t id : ids) {
            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) continue;
            Node& node = node_it->second;
            if (node.lb.size() != root_lb.size() ||
                node.ub.size() != root_ub.size()) {
                continue;
            }

            bool changed = false;
            std::uint64_t local_tightened = 0;
            for (int j = 0; j < root_lb.size(); ++j) {
                if (std::isfinite(root_lb[j]) &&
                    node.lb[j] < root_lb[j] - tol) {
                    node.lb[j] = root_lb[j];
                    changed = true;
                    ++local_tightened;
                }
                if (std::isfinite(root_ub[j]) &&
                    node.ub[j] > root_ub[j] + tol) {
                    node.ub[j] = root_ub[j];
                    changed = true;
                    ++local_tightened;
                }
            }

            if (!changed) {
                rebuild_local_domain_trail(
                    node, static_cast<int>(node.lb.size()), &root_lb, &root_ub);
                continue;
            }
            const auto index_membership = detach_queue_indices(id, node);
            if (!bounds_consistent(node.lb, node.ub)) {
                erase_detached_node(id);
                ++pruned;
                continue;
            }

            const double old_bound = node.bound;
            if (node.x_seed.size() == node.lb.size()) {
                node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
            }
            if (node.x_relax.size() == node.lb.size()) {
                node.x_relax = clamp_to_bounds(node.x_relax, node.lb, node.ub);
            }
            node.lp_refresh_needed = true;
            rebuild_local_domain_trail(node, static_cast<int>(node.lb.size()),
                                       &root_lb, &root_ub);
            if (tightened_out != nullptr) *tightened_out += local_tightened;
            if (refresh_marked_out != nullptr) ++(*refresh_marked_out);

            if (domain_objective_lower_bound_cb != nullptr) {
                const double domain_lb = (*domain_objective_lower_bound_cb)(node);
                if (std::isfinite(domain_lb) &&
                    (!std::isfinite(node.bound) || domain_lb > node.bound)) {
                    node.bound = domain_lb;
                    if (node.estimate < node.bound) node.estimate = node.bound;
                }
            }
            const double lift =
                std::isfinite(old_bound) && std::isfinite(node.bound)
                    ? std::max(0.0, node.bound - old_bound)
                    : 0.0;
            if (lift > 0.0) {
                if (domain_bound_lift_nodes_out != nullptr) {
                    ++(*domain_bound_lift_nodes_out);
                }
                if (domain_bound_lift_sum_out != nullptr) {
                    *domain_bound_lift_sum_out += lift;
                }
                if (domain_bound_lift_max_out != nullptr) {
                    *domain_bound_lift_max_out =
                        std::max(*domain_bound_lift_max_out, lift);
                }
            }

            if (has_incumbent && std::isfinite(incumbent_obj) &&
                std::isfinite(node.bound) &&
                node.bound >= incumbent_obj - prune_tol) {
                erase_detached_node(id);
                ++pruned;
                continue;
            }
            reattach_queue_indices(id, index_membership);
        }

        if (domain_signature_enabled_) {
            rebuild_domain_signatures();
        }
        return pruned;
    }

    bool empty() const { return priority_.empty() && dfs_.empty(); }

    int size() const {
        return static_cast<int>(priority_.size() + dfs_.size());
    }

    int priority_size() const {
        return static_cast<int>(priority_.size());
    }

    int dfs_size() const {
        return static_cast<int>(dfs_.size());
    }

    /// Discard the complete open frontier while retaining the configured root
    /// domain and cumulative storage telemetry. This is the queue transaction
    /// used by a proof-preserving root restart.
    int discard_all() noexcept {
        const int discarded = static_cast<int>(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)id;
            release_compact_storage(node);
        }
        nodes_.clear();
        priority_.clear();
        bounds_.clear();
        dfs_.clear();
        dfs_bounds_.clear();
        signature_hash_to_ids_.clear();
        id_to_signature_hash_.clear();
        hybrid_post_incumbent_active_ = false;
        hybrid_queue_leaves_ = 0;
        hybrid_last_lower_bound_leave_ = 0;
        return discarded;
    }

        int count_lp_refresh_needed_near_lower_bound(
            double bound_band = kInf,
            int max_scan = 4096) const {
            if (nodes_.empty()) return 0;

            const double queue_lb = lower_bound();
            const double threshold =
                std::isfinite(queue_lb) && std::isfinite(bound_band)
                    ? queue_lb + std::max(0.0, bound_band)
                    : kInf;

            int scanned = 0;
            int marked = 0;
            auto scan_source = [&](const std::multiset<QueueKey>& source) {
                for (auto it = source.begin(); it != source.end(); ++it) {
                    if (max_scan > 0 && scanned >= max_scan) break;
                    ++scanned;
                    if (std::isfinite(threshold) && it->first > threshold) break;
                    const auto node_it = nodes_.find(it->second);
                    if (node_it == nodes_.end()) continue;
                    if (node_it->second.lp_refresh_needed) ++marked;
                }
            };

            scan_source(bounds_);
            if (max_scan <= 0 || scanned < max_scan) {
                scan_source(dfs_bounds_);
            }
            return marked;
        }

        bool pop_lp_refresh_node(Node& out,
                                 double bound_band = kInf,
                                 int max_scan = 512,
                                 bool* was_dfs_out = nullptr) {
            if (nodes_.empty()) return false;

            const double queue_lb = lower_bound();
            const double threshold =
                std::isfinite(queue_lb) && std::isfinite(bound_band)
                    ? queue_lb + std::max(0.0, bound_band)
                    : kInf;

            std::int64_t best_id = -1;
            double best_bound = kInf;
            bool best_was_dfs = false;
            int scanned = 0;
            auto scan_source = [&](const std::multiset<QueueKey>& source,
                                   bool source_is_dfs) {
                for (auto it = source.begin(); it != source.end(); ++it) {
                    if (max_scan > 0 && scanned >= max_scan) break;
                    ++scanned;
                    const double bound = it->first;
                    if (std::isfinite(threshold) && bound > threshold) break;
                    const auto node_it = nodes_.find(it->second);
                    if (node_it == nodes_.end()) continue;
                    if (!node_it->second.lp_refresh_needed) continue;
                    if (bound < best_bound ||
                        (bound == best_bound && it->second < best_id)) {
                        best_bound = bound;
                        best_id = it->second;
                        best_was_dfs = source_is_dfs;
                    }
                }
            };

            scan_source(bounds_, false);
            if (max_scan <= 0 || scanned < max_scan) {
                scan_source(dfs_bounds_, true);
            }
            if (best_id < 0) return false;
            if (was_dfs_out != nullptr) *was_dfs_out = best_was_dfs;
            return take_id(best_id, out);
        }

		    int prune_conflicts(const ConflictPool& conflict_pool) {
		        DomainMaterializationGuard domain_guard(*this);
		        std::vector<std::int64_t> doomed;
		        doomed.reserve(nodes_.size());
		        for (const auto& [id, node] : nodes_) {
		            if (conflict_pool.has_conflict(node.lb, node.ub)) {
	                doomed.push_back(id);
	            }
	        }
	        for (std::int64_t id : doomed) {
	            erase_id(id);
		        }
	        return static_cast<int>(doomed.size());
		    }

	    int prune_by_incumbent(bool has_incumbent,
	                           double incumbent_obj,
	                           double prune_tol = kIncumbentPruneTol) {
	        if (!has_incumbent || !std::isfinite(incumbent_obj) ||
	            nodes_.empty()) {
	            return 0;
	        }
	        std::vector<std::int64_t> doomed;
	        doomed.reserve(nodes_.size());
	        for (const auto& [id, node] : nodes_) {
	            if (std::isfinite(node.bound) &&
	                node.bound >= incumbent_obj - prune_tol) {
	                doomed.push_back(id);
	            }
	        }
	        for (std::int64_t id : doomed) {
	            erase_id(id);
	        }
	        return static_cast<int>(doomed.size());
	    }

        int prune_by_lower_bound_limit(double lower_bound_limit,
                                       double prune_tol = kIncumbentPruneTol) {
            if (!std::isfinite(lower_bound_limit) || nodes_.empty()) {
                return 0;
            }
            std::vector<std::int64_t> doomed;
            doomed.reserve(nodes_.size());
            const double threshold = lower_bound_limit - std::max(0.0, prune_tol);
            for (const auto& [id, node] : nodes_) {
                if (std::isfinite(node.bound) && node.bound >= threshold) {
                    doomed.push_back(id);
                }
            }
            for (std::int64_t id : doomed) {
                erase_id(id);
            }
            return static_cast<int>(doomed.size());
        }

	    int prune_by_implications(const std::vector<VariableMeta>& vars,
			                              const BinaryImplicationGraph& implication_graph,
		                              const ConflictPool* conflict_pool = nullptr,
		                              bool has_incumbent = false,
		                              double incumbent_obj = kInf,
		                              double tol = 1e-9,
		                              std::uint64_t* tightened_out = nullptr,
		                              double bound_band = kInf,
		                              int max_nodes = 0,
                                  const Eigen::VectorXd* root_lb = nullptr,
                                  const Eigen::VectorXd* root_ub = nullptr);

	    int count_clause_hits(const std::vector<BranchDomainLiteral>& literals,
	                          bool require_unit_or_conflict = true,
	                          int max_hits = std::numeric_limits<int>::max(),
	                          double tol = 1e-9) const {
	        if (literals.empty()) return 0;
	        int hits = 0;
	        for (const auto& [id, node] : nodes_) {
	            (void)id;
	            int unsatisfied = 0;
	            for (const auto& literal : literals) {
	                if (literal.var_idx < 0 ||
	                    literal.var_idx >= queued_domain_size(node)) {
	                    unsatisfied = 2;
	                    break;
	                }
	                const bool active = literal.is_lb
	                    ? (queued_domain_lb(node, literal.var_idx) >=
	                       literal.value - tol)
	                    : (queued_domain_ub(node, literal.var_idx) <=
	                       literal.value + tol);
	                if (!active && ++unsatisfied > 1) break;
	            }
	            if (require_unit_or_conflict ? (unsatisfied <= 1)
	                                         : (unsatisfied == 0)) {
	                ++hits;
	                if (hits >= max_hits) return hits;
	            }
	        }
	        return hits;
	    }

		    int count_clause_hits_near_lower_bound(
		        const std::vector<BranchDomainLiteral>& literals,
		        double bound_band,
	        bool require_unit_or_conflict = true,
	        int max_hits = std::numeric_limits<int>::max(),
	        double tol = 1e-9) const {
	        if (literals.empty()) return 0;
	        const double queue_lb = lower_bound();
	        if (!std::isfinite(queue_lb)) return 0;
	        const double threshold =
	            queue_lb + std::max(0.0, bound_band) +
	            std::max(1e-9, tol) * std::max(1.0, std::abs(queue_lb));
	        int hits = 0;
	        for (const auto& [id, node] : nodes_) {
	            (void)id;
	            if (!std::isfinite(node.bound) || node.bound > threshold) {
	                continue;
	            }
	            int unsatisfied = 0;
	            for (const auto& literal : literals) {
	                if (literal.var_idx < 0 ||
	                    literal.var_idx >= queued_domain_size(node)) {
	                    unsatisfied = 2;
	                    break;
	                }
	                const bool active = literal.is_lb
	                    ? (queued_domain_lb(node, literal.var_idx) >=
	                       literal.value - tol)
	                    : (queued_domain_ub(node, literal.var_idx) <=
	                       literal.value + tol);
	                if (!active && ++unsatisfied > 1) break;
	            }
	            if (require_unit_or_conflict ? (unsatisfied <= 1)
	                                         : (unsatisfied == 0)) {
	                ++hits;
	                if (hits >= max_hits) return hits;
	            }
	        }
		        return hits;
		    }

		    int count_branch_literal_side(int var_idx, bool is_lb) const {
		        if (var_idx < 0) return 0;
		        int count = 0;
		        for (const auto& [id, node] : nodes_) {
		            (void)id;
		            for (const auto& lit : node.branch_reasons) {
		                if (lit.var_idx == var_idx && lit.is_lb == is_lb) {
		                    ++count;
		                    break;
		                }
		            }
		        }
		        return count;
		    }

        int propagate_bound_lifting_certificates(
            const std::vector<VariableMeta>& vars,
            const std::vector<BoundLiftingCertificate>& certificates,
            bool has_incumbent = false,
            double incumbent_obj = kInf,
            double prune_tol = kIncumbentPruneTol,
            std::uint64_t* tightened_out = nullptr,
            std::uint64_t* reasons_out = nullptr,
            std::uint64_t* refresh_marked_out = nullptr,
            double* frontier_bound_move_sum_out = nullptr,
            double* frontier_bound_move_max_out = nullptr,
            double bound_band = kInf,
            int max_nodes = 0,
            const Eigen::VectorXd* root_lb = nullptr,
            const Eigen::VectorXd* root_ub = nullptr,
            const std::function<double(const Node&)>
                *domain_objective_lower_bound_cb = nullptr,
            const std::function<bool(Node&,
                                     const std::vector<DomainReasonBound>&,
                                     std::uint64_t&,
                                     std::uint64_t&,
                                     bool&)>* post_update_cb = nullptr) {
            if (tightened_out != nullptr) *tightened_out = 0;
            if (reasons_out != nullptr) *reasons_out = 0;
            if (refresh_marked_out != nullptr) *refresh_marked_out = 0;
            if (frontier_bound_move_sum_out != nullptr) {
                *frontier_bound_move_sum_out = 0.0;
            }
            if (frontier_bound_move_max_out != nullptr) {
                *frontier_bound_move_max_out = 0.0;
            }
            if (certificates.empty() || nodes_.empty()) return 0;

            const int n = static_cast<int>(vars.size());
            if (n <= 0) return 0;
            DomainMaterializationGuard domain_guard(*this);
            const double tol = 1e-9;
            auto certificate_audit_ok =
                [&](const BoundLiftingCertificate& cert) -> bool {
                if (cert.target_bound.var_idx < 0 ||
                    cert.target_bound.var_idx >= n ||
                    !std::isfinite(cert.target_bound.value) ||
                    !cert.has_source_conflict_literal ||
                    cert.source_conflict_literal.var_idx < 0 ||
                    cert.source_conflict_literal.var_idx >= n ||
                    !std::isfinite(cert.source_conflict_literal.value) ||
                    !cert.has_proof_activity_audit ||
                    !std::isfinite(cert.proof_activity_margin) ||
                    cert.proof_activity_margin < -1e-8) {
                    return false;
                }
                const bool source_excludes_target =
                    cert.source_conflict_literal.var_idx ==
                        cert.target_bound.var_idx &&
                    cert.source_conflict_literal.is_lb !=
                        cert.target_bound.is_lb &&
                    (cert.target_bound.is_lb
                         ? cert.source_conflict_literal.value <
                               cert.target_bound.value - tol
                         : cert.source_conflict_literal.value >
                               cert.target_bound.value + tol);
                if (!source_excludes_target) return false;
                std::vector<double> lower(static_cast<std::size_t>(n), -kInf);
                std::vector<double> upper(static_cast<std::size_t>(n), kInf);
                for (const auto& lit : cert.frontier_literals) {
                    if (lit.var_idx < 0 || lit.var_idx >= n ||
                        !std::isfinite(lit.value) ||
                        lit.var_idx == cert.target_bound.var_idx) {
                        return false;
                    }
                    if (lit.is_lb) {
                        lower[static_cast<std::size_t>(lit.var_idx)] =
                            std::max(lower[static_cast<std::size_t>(lit.var_idx)],
                                     lit.value);
                    } else {
                        upper[static_cast<std::size_t>(lit.var_idx)] =
                            std::min(upper[static_cast<std::size_t>(lit.var_idx)],
                                     lit.value);
                    }
                    if (lower[static_cast<std::size_t>(lit.var_idx)] >
                        upper[static_cast<std::size_t>(lit.var_idx)] + tol) {
                        return false;
                    }
                }
                return true;
            };
            auto literal_active = [&](const BranchDomainLiteral& lit,
                                      const Eigen::VectorXd& lb,
                                      const Eigen::VectorXd& ub) {
                if (lit.var_idx < 0 || lit.var_idx >= n ||
                    lit.var_idx >= lb.size() || lit.var_idx >= ub.size()) {
                    return false;
                }
                return lit.is_lb ? (lb[lit.var_idx] >= lit.value - tol)
                                 : (ub[lit.var_idx] <= lit.value + tol);
            };
            auto flip_clause_literal =
                [&](const BranchDomainLiteral& lit,
                    BranchDomainLiteral& flipped) -> bool {
                if (lit.var_idx < 0 || lit.var_idx >= n ||
                    !std::isfinite(lit.value)) {
                    return false;
                }
                const bool integer_var =
                    is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
                if (lit.is_lb) {
                    flipped = BranchDomainLiteral{
                        lit.var_idx,
                        integer_var ? std::ceil(lit.value - tol) - 1.0
                                    : lit.value - std::max(1e-7, 10.0 * tol),
                        false};
                } else {
                    flipped = BranchDomainLiteral{
                        lit.var_idx,
                        integer_var ? std::floor(lit.value + tol) + 1.0
                                    : lit.value + std::max(1e-7, 10.0 * tol),
                        true};
                }
                return true;
            };
            auto duplicate_reason =
                [&](const Node& node,
                    const std::vector<DomainReasonBound>& pending,
                    const DomainReasonBound& rb) {
                const std::size_t reason_hash = conflict_clause_hash(rb.reason);
                for (const auto& existing : node.domain_reason_bounds) {
                    if (conflict_clause_hash(existing.reason) == reason_hash &&
                        domain_reason_bounds_equal(existing, rb)) {
                        return true;
                    }
                }
                for (const auto& existing : pending) {
                    if (conflict_clause_hash(existing.reason) == reason_hash &&
                        domain_reason_bounds_equal(existing, rb)) {
                        return true;
                    }
	                }
	                return false;
	            };

	            struct Update {
	                std::int64_t id{-1};
	                Eigen::VectorXd lb;
	                Eigen::VectorXd ub;
	                std::vector<DomainReasonBound> reasons;
	                std::uint64_t tightened{0};
	                double frontier_bound_move_sum{0.0};
	                double frontier_bound_move_max{0.0};
	            };

            std::vector<std::int64_t> candidate_ids;
            if (std::isfinite(bound_band) || max_nodes > 0) {
                std::set<std::int64_t> seen_ids;
                const double queue_lb = lower_bound();
                const double threshold =
                    std::isfinite(queue_lb) && std::isfinite(bound_band)
                        ? queue_lb + std::max(0.0, bound_band)
                        : kInf;
                auto collect_ids = [&](const std::multiset<QueueKey>& source) {
                    for (auto it = source.begin(); it != source.end(); ++it) {
                        if (std::isfinite(threshold) && it->first > threshold) break;
                        if (seen_ids.insert(it->second).second) {
                            candidate_ids.push_back(it->second);
                            if (max_nodes > 0 &&
                                static_cast<int>(candidate_ids.size()) >= max_nodes) {
                                break;
                            }
                        }
                    }
                };
                collect_ids(bounds_);
                if (max_nodes <= 0 ||
                    static_cast<int>(candidate_ids.size()) < max_nodes) {
                    collect_ids(dfs_bounds_);
                }
            } else {
                candidate_ids.reserve(nodes_.size());
                for (const auto& [id, node] : nodes_) {
                    (void)node;
                    candidate_ids.push_back(id);
                }
            }

            std::vector<std::int64_t> doomed;
            std::vector<Update> updates;
            doomed.reserve(candidate_ids.size());
            updates.reserve(candidate_ids.size());

            for (std::int64_t id : candidate_ids) {
                const auto node_find = nodes_.find(id);
                if (node_find == nodes_.end()) continue;
                const Node& node = node_find->second;
                if (node.lb.size() < n || node.ub.size() < n) continue;
                if (has_incumbent && std::isfinite(incumbent_obj) &&
                    std::isfinite(node.bound) &&
                    node.bound >= incumbent_obj - prune_tol) {
                    doomed.push_back(id);
                    continue;
                }

                Eigen::VectorXd lb = node.lb;
                Eigen::VectorXd ub = node.ub;
                std::vector<DomainReasonBound> reasons;
                std::uint64_t tightened = 0;
                bool prune = false;
                bool changed_this_pass = true;
                const std::size_t max_closure_passes = certificates.size() + 1;
                for (std::size_t pass = 0;
                     pass < max_closure_passes && changed_this_pass && !prune;
                     ++pass) {
                    changed_this_pass = false;
                    for (const auto& cert : certificates) {
                        if (!certificate_audit_ok(cert)) continue;
                        if (cert.target_bound.var_idx < 0 ||
                            cert.target_bound.var_idx >= n ||
                            !std::isfinite(cert.target_bound.value)) {
                            continue;
                        }
                        std::vector<BranchDomainLiteral> clause =
                            cert.frontier_literals;
                        BranchDomainLiteral source_lit =
                            cert.source_conflict_literal;
                        if (!cert.has_source_conflict_literal ||
                            source_lit.var_idx < 0 || source_lit.var_idx >= n ||
                            !std::isfinite(source_lit.value)) {
                            if (!flip_clause_literal(cert.target_bound,
                                                     source_lit)) {
                                continue;
                            }
                        }
                        clause.push_back(source_lit);

                        int missing_idx = -1;
                        int missing_count = 0;
                        std::vector<BranchDomainLiteral> reason;
                        reason.reserve(clause.size());
                        for (int ci = 0; ci < static_cast<int>(clause.size());
                             ++ci) {
                            const auto& lit =
                                clause[static_cast<std::size_t>(ci)];
                            if (literal_active(lit, lb, ub)) {
                                reason.push_back(lit);
                            } else {
                                missing_idx = ci;
                                if (++missing_count > 1) break;
                            }
                        }
                        if (missing_count == 0) {
                            prune = true;
                            break;
                        }
                        if (missing_count != 1 || missing_idx < 0) continue;

                        const BranchDomainLiteral& missing =
                            clause[static_cast<std::size_t>(missing_idx)];
                        const bool missing_is_source =
                            missing.var_idx == source_lit.var_idx &&
                            missing.is_lb == source_lit.is_lb &&
                            std::abs(missing.value - source_lit.value) <= tol;
                        BranchDomainLiteral implied = cert.target_bound;
                        if (!missing_is_source &&
                            !flip_clause_literal(missing, implied)) {
                            continue;
                        }
                        if (implied.var_idx < 0 || implied.var_idx >= n ||
                            !std::isfinite(implied.value)) {
                            continue;
                        }
                        canonicalize_branch_literals(reason);

                        DomainReasonBound rb;
                        rb.bound = implied;
                        rb.reason = reason;
                        rb.trail_pos = static_cast<int>(
                            node.branch_reasons.size() +
                            node.domain_reason_bounds.size() + reasons.size());
                        rb.depth = cert.depth;
                        rb.source_conflict_literal = missing;
                        rb.has_source_conflict_literal = true;
                        rb.has_proof_activity_audit =
                            cert.has_proof_activity_audit;
                        rb.proof_activity_margin =
                            cert.proof_activity_margin;
                        rb.proof_activity = cert.proof_activity;
                        rb.proof_required_activity =
                            cert.proof_required_activity;

                        const int j = implied.var_idx;
                        bool bound_changed = false;
                        if (implied.is_lb) {
                            if (implied.value > ub[j] + tol) {
                                prune = true;
                                break;
                            }
                            if (implied.value > lb[j] + tol) {
                                lb[j] = implied.value;
                                bound_changed = true;
                                ++tightened;
                            }
                        } else {
                            if (implied.value < lb[j] - tol) {
                                prune = true;
                                break;
                            }
                            if (implied.value < ub[j] - tol) {
                                ub[j] = implied.value;
                                bound_changed = true;
                                ++tightened;
                            }
                        }
                        bool reason_added = false;
                        if (bound_changed &&
                            !duplicate_reason(node, reasons, rb)) {
                            reasons.push_back(std::move(rb));
                            reason_added = true;
                        }
	                        if (bound_changed || reason_added) {
	                            changed_this_pass = true;
	                        }
	                    }
	                }

	                if (prune || !bounds_consistent(lb, ub)) {
	                    doomed.push_back(id);
	                } else if (tightened > 0 || !reasons.empty()) {
	                    double move_sum = 0.0;
	                    double move_max = 0.0;
	                    for (int j = 0; j < n; ++j) {
                        if (lb.size() > j && ub.size() > j &&
                            node.lb.size() > j && node.ub.size() > j) {
                            const double lb_move =
                                std::isfinite(lb[j]) && std::isfinite(node.lb[j])
                                    ? std::max(0.0, lb[j] - node.lb[j])
                                    : 0.0;
                            const double ub_move =
                                std::isfinite(ub[j]) && std::isfinite(node.ub[j])
                                    ? std::max(0.0, node.ub[j] - ub[j])
                                    : 0.0;
                            const double move = lb_move + ub_move;
                            if (move > 0.0) {
                                move_sum += move;
                                move_max = std::max(move_max, move);
                            }
                        }
	                    }
	                    updates.push_back(Update{id, std::move(lb), std::move(ub),
	                                             std::move(reasons), tightened,
	                                             move_sum, move_max});
	                }
	            }

            int pruned = 0;
            int post_update_pruned = 0;
            for (auto& upd : updates) {
                auto node_it = nodes_.find(upd.id);
                if (node_it == nodes_.end()) continue;
                Node& node = node_it->second;
                const auto index_membership =
                    detach_queue_indices(upd.id, node);
                const double old_bound = node.bound;
                bool refresh_marked = false;
                if (upd.tightened > 0) {
                    node.lb = std::move(upd.lb);
                    node.ub = std::move(upd.ub);
                    if (node.x_seed.size() == node.lb.size()) {
                        node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
                    }
                    node.lp_refresh_needed = true;
                    refresh_marked = true;
                    if (tightened_out != nullptr) *tightened_out += upd.tightened;
                    if (frontier_bound_move_sum_out != nullptr) {
                        *frontier_bound_move_sum_out +=
                            upd.frontier_bound_move_sum;
                    }
                    if (frontier_bound_move_max_out != nullptr) {
                        *frontier_bound_move_max_out =
                            std::max(*frontier_bound_move_max_out,
                                     upd.frontier_bound_move_max);
                    }
                }
                std::vector<DomainReasonBound> added_reason_bounds;
                added_reason_bounds.reserve(upd.reasons.size());
                std::uint64_t reasons_added = 0;
                for (auto& rb : upd.reasons) {
                    bool duplicate = false;
                    const std::size_t reason_hash = conflict_clause_hash(rb.reason);
                    for (const auto& existing : node.domain_reason_bounds) {
                        if (conflict_clause_hash(existing.reason) == reason_hash &&
                            domain_reason_bounds_equal(existing, rb)) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (duplicate) continue;
                    added_reason_bounds.push_back(rb);
                    append_domain_reason_bound(node, std::move(rb), n,
                                               root_lb, root_ub);
                    ++reasons_added;
                }
                if (reasons_added > 0) {
                    if (reasons_out != nullptr) *reasons_out += reasons_added;
                }
                bool post_changed = false;
                if (post_update_cb != nullptr &&
                    (upd.tightened > 0 || reasons_added > 0 ||
                     !added_reason_bounds.empty())) {
                    std::uint64_t post_tightened = 0;
                    std::uint64_t post_reasons = 0;
                    bool post_pruned = false;
                    const bool post_ok = (*post_update_cb)(
                        node, added_reason_bounds, post_tightened,
                        post_reasons, post_pruned);
                    if (!post_ok || post_pruned ||
                        !bounds_consistent(node.lb, node.ub)) {
                        erase_detached_node(upd.id);
                        ++post_update_pruned;
                        continue;
                    }
                    post_changed = post_tightened > 0 || post_reasons > 0;
                if (post_tightened > 0) {
                    if (node.x_seed.size() == node.lb.size()) {
                        node.x_seed = clamp_to_bounds(node.x_seed,
                                                      node.lb, node.ub);
                        }
                        node.lp_refresh_needed = true;
                        refresh_marked = true;
                        if (tightened_out != nullptr) {
                            *tightened_out += post_tightened;
                        }
                    }
                    if (post_reasons > 0 && reasons_out != nullptr) {
                        *reasons_out += post_reasons;
                    }
                }
                if (upd.tightened > 0 || reasons_added > 0 || post_changed) {
                    rebuild_local_domain_trail(node, n, root_lb, root_ub);
                }
		                if (domain_objective_lower_bound_cb != nullptr) {
		                    const double domain_lb =
		                        (*domain_objective_lower_bound_cb)(node);
		                    if (std::isfinite(domain_lb) &&
		                        (!std::isfinite(node.bound) ||
	                         domain_lb > node.bound)) {
                        node.bound = domain_lb;
                        if (node.estimate < node.bound) {
                            node.estimate = node.bound;
		                        }
                            refresh_marked = true;
		                    }
		                }
                const double lift =
                    std::isfinite(old_bound) && std::isfinite(node.bound)
                        ? std::max(0.0, node.bound - old_bound)
                        : 0.0;
                if (lift > 0.0) {
                    if (frontier_bound_move_sum_out != nullptr) {
                        *frontier_bound_move_sum_out += lift;
                    }
                    if (frontier_bound_move_max_out != nullptr) {
                        *frontier_bound_move_max_out =
                            std::max(*frontier_bound_move_max_out, lift);
                    }
                }
                if (refresh_marked && refresh_marked_out != nullptr) {
                    ++(*refresh_marked_out);
                }
		                if (!refresh_domain_signature_after_update(upd.id)) {
	                    erase_detached_node(upd.id);
	                    continue;
                }
                reattach_queue_indices(upd.id, index_membership);
            }
            for (std::int64_t id : doomed) {
                erase_id(id);
                ++pruned;
            }
            return pruned + post_update_pruned;
        }


	    template <typename PropagateFn>
	    int propagate_by_domain_object(
	        PropagateFn&& propagate_fn,
	        bool has_incumbent = false,
	        double incumbent_obj = kInf,
	        double prune_tol = kIncumbentPruneTol,
	        std::uint64_t* tightened_out = nullptr,
	        double bound_band = kInf,
	        int max_nodes = 0,
          const Eigen::VectorXd* root_lb = nullptr,
          const Eigen::VectorXd* root_ub = nullptr,
          const std::function<double(const Node&)>
              *domain_objective_lower_bound_cb = nullptr,
          std::uint64_t* domain_bound_lift_nodes_out = nullptr,
          double* domain_bound_lift_sum_out = nullptr,
          double* domain_bound_lift_max_out = nullptr) {
	        if (tightened_out != nullptr) *tightened_out = 0;
            if (domain_bound_lift_nodes_out != nullptr) {
                *domain_bound_lift_nodes_out = 0;
            }
            if (domain_bound_lift_sum_out != nullptr) {
                *domain_bound_lift_sum_out = 0.0;
            }
            if (domain_bound_lift_max_out != nullptr) {
                *domain_bound_lift_max_out = 0.0;
            }
		        if (nodes_.empty()) return 0;
		        DomainMaterializationGuard domain_guard(*this);

		        struct Update {
	            std::int64_t id{-1};
	            Eigen::VectorXd lb;
	            Eigen::VectorXd ub;
	            std::uint64_t tightened{0};
	        };

	        std::vector<std::int64_t> doomed;
	        std::vector<Update> updates;
	        doomed.reserve(nodes_.size());
	        updates.reserve(nodes_.size());

	        std::vector<std::int64_t> candidate_ids;
	        if (std::isfinite(bound_band) || max_nodes > 0) {
	            std::set<std::int64_t> seen_ids;
	            const double queue_lb = lower_bound();
	            const double threshold =
	                std::isfinite(queue_lb) && std::isfinite(bound_band)
	                    ? queue_lb + std::max(0.0, bound_band)
	                    : kInf;
	            auto collect_ids = [&](const std::multiset<QueueKey>& source) {
	                for (auto it = source.begin(); it != source.end(); ++it) {
	                    if (std::isfinite(threshold) && it->first > threshold) break;
	                    if (seen_ids.insert(it->second).second) {
	                        candidate_ids.push_back(it->second);
	                        if (max_nodes > 0 &&
	                            static_cast<int>(candidate_ids.size()) >= max_nodes) {
	                            break;
	                        }
	                    }
	                }
	            };
	            collect_ids(bounds_);
	            if (max_nodes <= 0 ||
	                static_cast<int>(candidate_ids.size()) < max_nodes) {
	                collect_ids(dfs_bounds_);
	            }
	        } else {
	            candidate_ids.reserve(nodes_.size());
	            for (const auto& [id, node] : nodes_) {
	                (void)node;
	                candidate_ids.push_back(id);
	            }
	        }

	        for (std::int64_t id : candidate_ids) {
	            const auto node_find = nodes_.find(id);
	            if (node_find == nodes_.end()) continue;
	            const Node& node = node_find->second;
	            if (has_incumbent && std::isfinite(incumbent_obj) &&
	                std::isfinite(node.bound) &&
	                node.bound >= incumbent_obj - prune_tol) {
	                doomed.push_back(id);
	                continue;
	            }

	            Eigen::VectorXd lb = node.lb;
	            Eigen::VectorXd ub = node.ub;
	            int tightened = 0;
	            int pruned = 0;
	            const bool ok = propagate_fn(node, lb, ub, tightened, pruned);
	            if (!ok || pruned > 0 || !bounds_consistent(lb, ub)) {
	                doomed.push_back(id);
	                continue;
	            }
	            if (tightened > 0) {
	                updates.push_back(Update{id, std::move(lb), std::move(ub),
	                                         static_cast<std::uint64_t>(tightened)});
	            }
	        }

		        for (const Update& upd : updates) {
		            auto node_it = nodes_.find(upd.id);
		            if (node_it == nodes_.end()) continue;
		            Node& node = node_it->second;
		            const auto index_membership =
		                detach_queue_indices(upd.id, node);
		            node.lb = upd.lb;
		            node.ub = upd.ub;
		            if (node.x_seed.size() == node.lb.size()) {
		                node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
		            }
		            node.lp_refresh_needed = true;
		            if (tightened_out != nullptr) *tightened_out += upd.tightened;
		            rebuild_local_domain_trail(
		                node, static_cast<int>(node.lb.size()), root_lb, root_ub);
                    if (domain_objective_lower_bound_cb != nullptr) {
                        const double old_bound = node.bound;
                        const double domain_lb =
                            (*domain_objective_lower_bound_cb)(node);
                        if (std::isfinite(domain_lb) &&
                            (!std::isfinite(node.bound) ||
                             domain_lb > node.bound)) {
                            node.bound = domain_lb;
                            if (node.estimate < node.bound) {
                                node.estimate = node.bound;
                            }
                            const double lift =
                                std::isfinite(old_bound)
                                    ? std::max(0.0, node.bound - old_bound)
                                    : 0.0;
                            if (lift > 0.0) {
                                if (domain_bound_lift_nodes_out != nullptr) {
                                    ++(*domain_bound_lift_nodes_out);
                                }
                                if (domain_bound_lift_sum_out != nullptr) {
                                    *domain_bound_lift_sum_out += lift;
                                }
                                if (domain_bound_lift_max_out != nullptr) {
                                    *domain_bound_lift_max_out =
                                        std::max(*domain_bound_lift_max_out,
                                                 lift);
                                }
                            }
                        }
                    }
		                    if (!refresh_domain_signature_after_update(upd.id)) {
		                        erase_detached_node(upd.id);
		                        continue;
		                    }
		            reattach_queue_indices(upd.id, index_membership);
		        }

	        for (std::int64_t id : doomed) {
	            erase_id(id);
	        }
	        return static_cast<int>(doomed.size());
	    }

	    template <typename PropagateFn>
	    int propagate_by_domain_object_with_reasons(
	        PropagateFn&& propagate_fn,
	        bool has_incumbent = false,
	        double incumbent_obj = kInf,
	        double prune_tol = kIncumbentPruneTol,
	        std::uint64_t* tightened_out = nullptr,
	        std::uint64_t* reasons_out = nullptr,
	        double bound_band = kInf,
	        int max_nodes = 0,
	        const Eigen::VectorXd* root_lb = nullptr,
	        const Eigen::VectorXd* root_ub = nullptr,
	        const std::function<double(const Node&)>
	            *domain_objective_lower_bound_cb = nullptr,
	        std::uint64_t* domain_bound_lift_nodes_out = nullptr,
	        double* domain_bound_lift_sum_out = nullptr,
	        double* domain_bound_lift_max_out = nullptr,
	        const std::function<bool(Node&,
	                                 const std::vector<DomainReasonBound>&,
	                                 std::uint64_t&,
	                                 std::uint64_t&,
	                                 bool&)>* post_update_cb = nullptr) {
	        if (tightened_out != nullptr) *tightened_out = 0;
	        if (reasons_out != nullptr) *reasons_out = 0;
            if (domain_bound_lift_nodes_out != nullptr) {
                *domain_bound_lift_nodes_out = 0;
            }
            if (domain_bound_lift_sum_out != nullptr) {
                *domain_bound_lift_sum_out = 0.0;
            }
            if (domain_bound_lift_max_out != nullptr) {
                *domain_bound_lift_max_out = 0.0;
            }
		        if (nodes_.empty()) return 0;
		        DomainMaterializationGuard domain_guard(*this);

		        struct Update {
	            std::int64_t id{-1};
	            Eigen::VectorXd lb;
	            Eigen::VectorXd ub;
	            std::vector<DomainReasonBound> reasons;
	            std::uint64_t tightened{0};
	        };

	        std::vector<std::int64_t> doomed;
	        std::vector<Update> updates;
	        doomed.reserve(nodes_.size());
	        updates.reserve(nodes_.size());

	        std::vector<std::int64_t> candidate_ids;
	        if (std::isfinite(bound_band) || max_nodes > 0) {
	            std::set<std::int64_t> seen_ids;
	            const double queue_lb = lower_bound();
	            const double threshold =
	                std::isfinite(queue_lb) && std::isfinite(bound_band)
	                    ? queue_lb + std::max(0.0, bound_band)
	                    : kInf;
	            auto collect_ids = [&](const std::multiset<QueueKey>& source) {
	                for (auto it = source.begin(); it != source.end(); ++it) {
	                    if (std::isfinite(threshold) && it->first > threshold) break;
	                    if (seen_ids.insert(it->second).second) {
	                        candidate_ids.push_back(it->second);
	                        if (max_nodes > 0 &&
	                            static_cast<int>(candidate_ids.size()) >= max_nodes) {
	                            break;
	                        }
	                    }
	                }
	            };
	            collect_ids(bounds_);
	            if (max_nodes <= 0 ||
	                static_cast<int>(candidate_ids.size()) < max_nodes) {
	                collect_ids(dfs_bounds_);
	            }
	        } else {
	            candidate_ids.reserve(nodes_.size());
	            for (const auto& [id, node] : nodes_) {
	                (void)node;
	                candidate_ids.push_back(id);
	            }
	        }

	        for (std::int64_t id : candidate_ids) {
	            const auto node_find = nodes_.find(id);
	            if (node_find == nodes_.end()) continue;
	            const Node& node = node_find->second;
	            if (has_incumbent && std::isfinite(incumbent_obj) &&
	                std::isfinite(node.bound) &&
	                node.bound >= incumbent_obj - prune_tol) {
	                doomed.push_back(id);
	                continue;
	            }

	            Eigen::VectorXd lb = node.lb;
	            Eigen::VectorXd ub = node.ub;
	            std::vector<DomainReasonBound> reasons;
	            int tightened = 0;
	            int pruned = 0;
	            const bool ok =
	                propagate_fn(node, lb, ub, reasons, tightened, pruned);
	            if (!ok || pruned > 0 || !bounds_consistent(lb, ub)) {
	                doomed.push_back(id);
	                continue;
	            }
	            if (tightened > 0 || !reasons.empty()) {
	                updates.push_back(Update{
	                    id, std::move(lb), std::move(ub), std::move(reasons),
	                    static_cast<std::uint64_t>(std::max(0, tightened))});
	            }
	        }

			        int post_update_pruned = 0;
				        for (Update& upd : updates) {
				            auto node_it = nodes_.find(upd.id);
				            if (node_it == nodes_.end()) continue;
				            Node& node = node_it->second;
				            const auto index_membership =
				                detach_queue_indices(upd.id, node);
				            bool touched_domain = false;
				            bool trail_changed = false;
				            if (upd.tightened > 0) {
			                node.lb = std::move(upd.lb);
		                node.ub = std::move(upd.ub);
		                if (node.x_seed.size() == node.lb.size()) {
		                    node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
		                }
		                touched_domain = true;
		                trail_changed = true;
		                node.lp_refresh_needed = true;
		                if (tightened_out != nullptr) *tightened_out += upd.tightened;
		            }
		            std::uint64_t reasons_added = 0;
		            std::vector<DomainReasonBound> added_reason_bounds;
		            if (post_update_cb != nullptr && !upd.reasons.empty()) {
		                added_reason_bounds.reserve(upd.reasons.size());
		            }
		            for (DomainReasonBound& rb : upd.reasons) {
		                bool duplicate = false;
		                for (const auto& existing : node.domain_reason_bounds) {
		                    if (conflict_clause_hash(existing.reason) ==
		                            conflict_clause_hash(rb.reason) &&
		                        domain_reason_bounds_equal(existing, rb)) {
	                        duplicate = true;
	                        break;
		                    }
		                }
		                if (!duplicate) {
		                    if (post_update_cb != nullptr) {
		                        added_reason_bounds.push_back(rb);
		                    }
			                    append_domain_reason_bound(
			                        node, std::move(rb), static_cast<int>(node.lb.size()),
			                        root_lb, root_ub);
			                    ++reasons_added;
			                    trail_changed = true;
			                }
		            }
	            if (reasons_added > 0 && reasons_out != nullptr) {
	                *reasons_out += reasons_added;
	            }
				            bool post_changed = false;
				            bool post_domain_tightened = false;
			            if (post_update_cb != nullptr &&
			                (touched_domain || reasons_added > 0 ||
			                 !added_reason_bounds.empty())) {
			                std::uint64_t post_tightened = 0;
			                std::uint64_t post_reasons = 0;
		                bool post_pruned = false;
			                const bool post_ok = (*post_update_cb)(
			                    node, added_reason_bounds, post_tightened,
			                    post_reasons, post_pruned);
			                if (!post_ok || post_pruned ||
			                    !bounds_consistent(node.lb, node.ub)) {
			                    erase_detached_node(upd.id);
			                    ++post_update_pruned;
			                    continue;
			                }
			                post_changed = post_tightened > 0 || post_reasons > 0;
			                post_domain_tightened = post_tightened > 0;
			                if (post_tightened > 0) {
				                    if (node.x_seed.size() == node.lb.size()) {
				                        node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
			                    }
			                    node.lp_refresh_needed = true;
			                    touched_domain = true;
			                    trail_changed = true;
			                    if (tightened_out != nullptr) {
			                        *tightened_out += post_tightened;
			                    }
			                }
			                if (post_reasons > 0 && reasons_out != nullptr) {
				                    *reasons_out += post_reasons;
				                    trail_changed = true;
				                }
				            }
				            if (trail_changed || post_changed) {
				                rebuild_local_domain_trail(
				                    node, static_cast<int>(node.lb.size()), root_lb, root_ub);
				            }
	                    if (domain_objective_lower_bound_cb != nullptr &&
	                        (touched_domain || post_domain_tightened)) {
                        const double old_bound = node.bound;
                        const double domain_lb =
                            (*domain_objective_lower_bound_cb)(node);
                        if (std::isfinite(domain_lb) &&
                            (!std::isfinite(node.bound) ||
                             domain_lb > node.bound)) {
                            node.bound = domain_lb;
                            if (node.estimate < node.bound) {
                                node.estimate = node.bound;
                            }
                            const double lift =
                                std::isfinite(old_bound)
                                    ? std::max(0.0, node.bound - old_bound)
                                    : 0.0;
                            if (lift > 0.0) {
                                if (domain_bound_lift_nodes_out != nullptr) {
                                    ++(*domain_bound_lift_nodes_out);
                                }
                                if (domain_bound_lift_sum_out != nullptr) {
                                    *domain_bound_lift_sum_out += lift;
                                }
                                if (domain_bound_lift_max_out != nullptr) {
                                    *domain_bound_lift_max_out =
                                        std::max(*domain_bound_lift_max_out,
                                                 lift);
                                }
                            }
                        }
                    }
                    if (!refresh_domain_signature_after_update(upd.id)) {
                        erase_detached_node(upd.id);
                        continue;
                    }
		            reattach_queue_indices(upd.id, index_membership);
			        }

		        for (std::int64_t id : doomed) {
		            erase_id(id);
		        }
		        return static_cast<int>(doomed.size()) + post_update_pruned;
		    }

	    template <typename ClosureFn>
	    int propagate_by_node_object(
	        ClosureFn&& closure_fn,
	        bool has_incumbent = false,
	        double incumbent_obj = kInf,
	        double prune_tol = kIncumbentPruneTol,
	        std::uint64_t* tightened_out = nullptr,
	        std::uint64_t* reasons_out = nullptr,
	        double bound_band = kInf,
	        int max_nodes = 0,
	        const std::function<double(const Node&)>
	            *domain_objective_lower_bound_cb = nullptr,
	        std::uint64_t* domain_bound_lift_nodes_out = nullptr,
	        double* domain_bound_lift_sum_out = nullptr,
	        double* domain_bound_lift_max_out = nullptr) {
	        if (tightened_out != nullptr) *tightened_out = 0;
	        if (reasons_out != nullptr) *reasons_out = 0;
	        if (domain_bound_lift_nodes_out != nullptr) {
	            *domain_bound_lift_nodes_out = 0;
	        }
	        if (domain_bound_lift_sum_out != nullptr) {
	            *domain_bound_lift_sum_out = 0.0;
	        }
	        if (domain_bound_lift_max_out != nullptr) {
	            *domain_bound_lift_max_out = 0.0;
	        }
		        if (nodes_.empty()) return 0;
		        DomainMaterializationGuard domain_guard(*this);

		        std::vector<std::int64_t> candidate_ids;
	        if (std::isfinite(bound_band) || max_nodes > 0) {
	            std::set<std::int64_t> seen_ids;
	            const double queue_lb = lower_bound();
	            const double threshold =
	                std::isfinite(queue_lb) && std::isfinite(bound_band)
	                    ? queue_lb + std::max(0.0, bound_band)
	                    : kInf;
	            auto collect_ids = [&](const std::multiset<QueueKey>& source) {
	                for (auto it = source.begin(); it != source.end(); ++it) {
	                    if (std::isfinite(threshold) && it->first > threshold) break;
	                    if (seen_ids.insert(it->second).second) {
	                        candidate_ids.push_back(it->second);
	                        if (max_nodes > 0 &&
	                            static_cast<int>(candidate_ids.size()) >= max_nodes) {
	                            break;
	                        }
	                    }
	                }
	            };
	            collect_ids(bounds_);
	            if (max_nodes <= 0 ||
	                static_cast<int>(candidate_ids.size()) < max_nodes) {
	                collect_ids(dfs_bounds_);
	            }
	        } else {
	            candidate_ids.reserve(nodes_.size());
	            for (const auto& [id, node] : nodes_) {
	                (void)node;
	                candidate_ids.push_back(id);
	            }
	        }

	        int pruned = 0;
	        for (std::int64_t id : candidate_ids) {
	            auto node_it = nodes_.find(id);
	            if (node_it == nodes_.end()) continue;
	            if (has_incumbent && std::isfinite(incumbent_obj) &&
	                std::isfinite(node_it->second.bound) &&
	                node_it->second.bound >= incumbent_obj - prune_tol) {
	                erase_id(id);
	                ++pruned;
	                continue;
	            }

	            Node& node = node_it->second;
	            const double old_bound = node.bound;
	            const double old_obj_domain_lb =
	                domain_objective_lower_bound_cb != nullptr
	                    ? (*domain_objective_lower_bound_cb)(node)
	                    : kInf;
	            const auto index_membership = detach_queue_indices(id, node);

	            std::uint64_t tightened = 0;
	            std::uint64_t reasons = 0;
	            bool closure_pruned = false;
	            const bool ok =
	                closure_fn(node, tightened, reasons, closure_pruned);
	            if (!ok || closure_pruned ||
	                !bounds_consistent(node.lb, node.ub)) {
	                erase_detached_node(id);
	                ++pruned;
	                continue;
	            }

	            double new_obj_domain_lb = kInf;
	            if (domain_objective_lower_bound_cb != nullptr) {
	                new_obj_domain_lb = (*domain_objective_lower_bound_cb)(node);
	                const double domain_lb = new_obj_domain_lb;
	                if (std::isfinite(domain_lb) &&
	                    (!std::isfinite(node.bound) || domain_lb > node.bound)) {
	                    node.bound = domain_lb;
	                    if (node.estimate < node.bound) node.estimate = node.bound;
	                }
	            }
	            if (has_incumbent && std::isfinite(incumbent_obj) &&
	                std::isfinite(node.bound) &&
	                node.bound >= incumbent_obj - prune_tol) {
	                erase_detached_node(id);
	                ++pruned;
	                continue;
	            }

	            if (tightened_out != nullptr) *tightened_out += tightened;
	            if (reasons_out != nullptr) *reasons_out += reasons;
	            const double lift =
	                std::isfinite(old_bound) && std::isfinite(node.bound)
	                    ? std::max(0.0, node.bound - old_bound)
	                    : 0.0;
	            const bool objective_lb_moved =
	                std::isfinite(old_obj_domain_lb) &&
	                std::isfinite(new_obj_domain_lb) &&
	                new_obj_domain_lb > old_obj_domain_lb + 1e-9;
	            if (lift > 0.0 || objective_lb_moved) {
	                if (domain_bound_lift_nodes_out != nullptr) {
	                    ++(*domain_bound_lift_nodes_out);
	                }
	                if (domain_bound_lift_sum_out != nullptr) {
	                    *domain_bound_lift_sum_out += lift;
	                }
	                if (domain_bound_lift_max_out != nullptr) {
	                    *domain_bound_lift_max_out =
	                        std::max(*domain_bound_lift_max_out, lift);
	                }
	            }
	            if (!refresh_domain_signature_after_update(id)) {
	                erase_detached_node(id);
	                continue;
	            }
	            reattach_queue_indices(id, index_membership);
	        }
	        return pruned;
	    }

 private:
    using QueueKey = std::pair<double, std::int64_t>;

    struct QueueIndexMembership {
        bool had_priority{false};
        bool had_bound{false};
        bool had_dfs_bound{false};
    };

    NodeSelection mode_;
    std::int64_t next_id_{0};
    std::unordered_map<std::int64_t, Node> nodes_;
    std::multiset<QueueKey> priority_;
    std::multiset<QueueKey> bounds_;
    std::vector<std::int64_t> dfs_;
    std::multiset<QueueKey> dfs_bounds_;
    bool domain_signature_enabled_{false};
    std::vector<int> domain_signature_cols_;
    Eigen::Index domain_signature_size_{0};
    Eigen::VectorXd root_domain_lb_;
    Eigen::VectorXd root_domain_ub_;
    bool domain_storage_enabled_{false};
    NodeQueueDomainStorageStats domain_storage_stats_;
    std::unordered_map<std::size_t, std::vector<std::int64_t>>
        signature_hash_to_ids_;
    std::unordered_map<std::int64_t, std::size_t> id_to_signature_hash_;
    bool hybrid_post_incumbent_active_{false};
    std::int64_t hybrid_queue_leaves_{0};
    std::int64_t hybrid_last_lower_bound_leave_{0};
    static constexpr std::int64_t kHybridBestBoundPeriod = 10;

    Eigen::Index queued_domain_size(const Node& node) const {
        if (node_domain_is_compact(node)) return node.compact_domain_size;
        return node.lb.size() == node.ub.size() ? node.lb.size() : 0;
    }

    double queued_domain_lb(const Node& node, Eigen::Index col) const {
        if (!node_domain_is_compact(node)) return node.lb[col];
        const auto it = std::lower_bound(
            node.compact_domain.begin(), node.compact_domain.end(), col,
            [](const SparseNodeBound& bound, Eigen::Index index) {
                return bound.var_idx < index;
            });
        return it != node.compact_domain.end() && it->var_idx == col
                   ? it->lb
                   : root_domain_lb_[col];
    }

    double queued_domain_ub(const Node& node, Eigen::Index col) const {
        if (!node_domain_is_compact(node)) return node.ub[col];
        const auto it = std::lower_bound(
            node.compact_domain.begin(), node.compact_domain.end(), col,
            [](const SparseNodeBound& bound, Eigen::Index index) {
                return bound.var_idx < index;
            });
        return it != node.compact_domain.end() && it->var_idx == col
                   ? it->ub
                   : root_domain_ub_[col];
    }

    bool compact_stored_node(Node& node) noexcept {
        if (!domain_storage_enabled_ || node_domain_is_compact(node)) return true;
        const std::uint64_t released =
            static_cast<std::uint64_t>(node.lb.size() + node.ub.size());
        std::size_t entries = 0;
        if (!compact_node_domain(node, root_domain_lb_, root_domain_ub_,
                                 &entries)) {
            ++domain_storage_stats_.compaction_failures;
            return false;
        }
        ++domain_storage_stats_.compactions;
        domain_storage_stats_.dense_bound_values_released += released;
        domain_storage_stats_.compact_entries_created += entries;
        ++domain_storage_stats_.current_compact_nodes;
        domain_storage_stats_.current_compact_entries += entries;
        domain_storage_stats_.peak_compact_nodes = std::max(
            domain_storage_stats_.peak_compact_nodes,
            domain_storage_stats_.current_compact_nodes);
        domain_storage_stats_.peak_compact_entries = std::max(
            domain_storage_stats_.peak_compact_entries,
            domain_storage_stats_.current_compact_entries);
        return true;
    }

    void release_compact_storage(const Node& node) noexcept {
        if (!node_domain_is_compact(node)) return;
        const auto entries =
            static_cast<std::uint64_t>(node.compact_domain.size());
        if (domain_storage_stats_.current_compact_nodes > 0) {
            --domain_storage_stats_.current_compact_nodes;
        }
        domain_storage_stats_.current_compact_entries =
            entries <= domain_storage_stats_.current_compact_entries
                ? domain_storage_stats_.current_compact_entries - entries
                : 0;
    }

    void materialize_stored_node(Node& node) {
        if (!node_domain_is_compact(node)) return;
        const auto entries =
            static_cast<std::uint64_t>(node.compact_domain.size());
        if (!materialize_node_domain(node, root_domain_lb_, root_domain_ub_)) {
            ++domain_storage_stats_.materialization_failures;
            throw std::logic_error(
                "queued node domain cannot be materialized from the root domain");
        }
        ++domain_storage_stats_.materializations;
        if (domain_storage_stats_.current_compact_nodes > 0) {
            --domain_storage_stats_.current_compact_nodes;
        }
        domain_storage_stats_.current_compact_entries =
            entries <= domain_storage_stats_.current_compact_entries
                ? domain_storage_stats_.current_compact_entries - entries
                : 0;
    }

    void materialize_all_domains() {
        for (auto& [id, node] : nodes_) {
            (void)id;
            materialize_stored_node(node);
        }
    }

    void compact_all_domains_noexcept() noexcept {
        if (!domain_storage_enabled_) return;
        for (auto& [id, node] : nodes_) {
            (void)id;
            if (!compact_stored_node(node)) {
                domain_storage_enabled_ = false;
                for (auto& [restore_id, restore_node] : nodes_) {
                    (void)restore_id;
                    if (!node_domain_is_compact(restore_node)) continue;
                    try {
                        materialize_stored_node(restore_node);
                    } catch (...) {
                        return;
                    }
                }
                return;
            }
        }
    }

    QueueIndexMembership detach_queue_indices(std::int64_t id,
                                               const Node& node) {
        QueueIndexMembership membership;
        const double priority = node_selection_priority(node);
        auto priority_it = priority_.find({priority, id});
        if (priority_it != priority_.end()) {
            priority_.erase(priority_it);
            membership.had_priority = true;
        }
        const double bound = node.bound;
        auto bound_it = bounds_.find({bound, id});
        if (bound_it != bounds_.end()) {
            bounds_.erase(bound_it);
            membership.had_bound = true;
        }
        auto dfs_bound_it = dfs_bounds_.find({bound, id});
        if (dfs_bound_it != dfs_bounds_.end()) {
            dfs_bounds_.erase(dfs_bound_it);
            membership.had_dfs_bound = true;
        }
        return membership;
    }

    void reattach_queue_indices(std::int64_t id,
                                const QueueIndexMembership& membership) {
        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) return;
        const Node& node = node_it->second;
        if (membership.had_priority) {
            priority_.emplace(node_selection_priority(node), id);
        }
        if (membership.had_bound) {
            bounds_.emplace(node.bound, id);
        }
        if (membership.had_dfs_bound) {
            dfs_bounds_.emplace(node.bound, id);
        }
    }

    void erase_detached_node(std::int64_t id) {
        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) return;
        dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());
        forget_domain_signature(id);
        release_compact_storage(node_it->second);
        nodes_.erase(node_it);
    }

    double signature_bound_tol(double bound) const {
        return std::max(1e-9, 1e-12 * std::max(1.0, std::abs(bound)));
    }

    std::optional<std::size_t> make_domain_signature_hash(
        const Node& node) const {
        if (!domain_signature_enabled_ ||
            queued_domain_size(node) != domain_signature_size_) {
            return std::nullopt;
        }
        std::size_t hash = 0xcbf29ce484222325ULL;
        auto combine = [&](std::size_t value) {
            hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) +
                    (hash >> 2);
        };
        for (int j : domain_signature_cols_) {
            if (j < 0 || j >= queued_domain_size(node)) continue;
            const double raw_lb = queued_domain_lb(node, j);
            const double raw_ub = queued_domain_ub(node, j);
            const double lb = raw_lb == 0.0 ? 0.0 : raw_lb;
            const double ub = raw_ub == 0.0 ? 0.0 : raw_ub;
            combine(std::hash<int>{}(j));
            combine(std::hash<double>{}(lb));
            combine(std::hash<double>{}(ub));
        }
        return hash;
    }

    bool same_domain(const Node& lhs, const Node& rhs) const {
        if (queued_domain_size(lhs) != queued_domain_size(rhs)) {
            return false;
        }
        for (int j : domain_signature_cols_) {
            if (j < 0 || j >= queued_domain_size(lhs)) continue;
            const double raw_lhs_lb = queued_domain_lb(lhs, j);
            const double raw_rhs_lb = queued_domain_lb(rhs, j);
            const double raw_lhs_ub = queued_domain_ub(lhs, j);
            const double raw_rhs_ub = queued_domain_ub(rhs, j);
            const double lhs_lb = raw_lhs_lb == 0.0 ? 0.0 : raw_lhs_lb;
            const double rhs_lb = raw_rhs_lb == 0.0 ? 0.0 : raw_rhs_lb;
            const double lhs_ub = raw_lhs_ub == 0.0 ? 0.0 : raw_lhs_ub;
            const double rhs_ub = raw_rhs_ub == 0.0 ? 0.0 : raw_rhs_ub;
            if (lhs_lb != rhs_lb || lhs_ub != rhs_ub) return false;
        }
        return true;
    }

    void forget_domain_signature(std::int64_t id) {
        auto hash_it = id_to_signature_hash_.find(id);
        if (hash_it == id_to_signature_hash_.end()) return;
        auto bucket_it = signature_hash_to_ids_.find(hash_it->second);
        if (bucket_it != signature_hash_to_ids_.end()) {
            auto& ids = bucket_it->second;
            ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
            if (ids.empty()) signature_hash_to_ids_.erase(bucket_it);
        }
        id_to_signature_hash_.erase(hash_it);
    }

    bool refresh_domain_signature_after_update(std::int64_t id) {
        if (!domain_signature_enabled_) return nodes_.find(id) != nodes_.end();
        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) return false;
        forget_domain_signature(id);
        auto signature_hash = make_domain_signature_hash(node_it->second);
        if (!signature_hash.has_value()) return true;
        auto bucket_it = signature_hash_to_ids_.find(*signature_hash);
        if (bucket_it != signature_hash_to_ids_.end()) {
            const std::vector<std::int64_t> candidates = bucket_it->second;
            for (std::int64_t existing_id : candidates) {
                if (existing_id == id) continue;
                auto existing_node = nodes_.find(existing_id);
                if (existing_node == nodes_.end() ||
                    !same_domain(existing_node->second, node_it->second)) {
                    continue;
                }
                if (existing_node->second.bound <=
                    node_it->second.bound +
                        signature_bound_tol(node_it->second.bound)) {
                    erase_id(id);
                    return false;
                }
                erase_id(existing_id);
                break;
            }
        }
        signature_hash_to_ids_[*signature_hash].push_back(id);
        id_to_signature_hash_[id] = *signature_hash;
        return true;
    }

    void rebuild_domain_signatures() {
        if (!domain_signature_enabled_) return;
        signature_hash_to_ids_.clear();
        id_to_signature_hash_.clear();
        std::vector<std::int64_t> ids;
        ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            ids.push_back(id);
        }
        for (std::int64_t id : ids) {
            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) continue;
            auto signature_hash = make_domain_signature_hash(node_it->second);
            if (!signature_hash.has_value()) continue;
            auto& bucket = signature_hash_to_ids_[*signature_hash];
            auto existing = std::find_if(
                bucket.begin(), bucket.end(), [&](std::int64_t candidate_id) {
                    const auto candidate = nodes_.find(candidate_id);
                    return candidate != nodes_.end() &&
                           same_domain(candidate->second, node_it->second);
                });
            if (existing == bucket.end()) {
                bucket.push_back(id);
                id_to_signature_hash_[id] = *signature_hash;
                continue;
            }
            auto existing_node = nodes_.find(*existing);
            if (existing_node != nodes_.end() &&
                existing_node->second.bound <=
                    node_it->second.bound + signature_bound_tol(node_it->second.bound)) {
                erase_id(id);
            } else {
                if (existing_node != nodes_.end()) {
                    erase_id(*existing);
                }
                signature_hash_to_ids_[*signature_hash].push_back(id);
                id_to_signature_hash_[id] = *signature_hash;
            }
        }
    }

    std::int64_t store(Node node) {
        std::optional<std::size_t> signature_hash;
        if (domain_signature_enabled_) {
            signature_hash = make_domain_signature_hash(node);
            if (signature_hash.has_value()) {
                auto bucket_it = signature_hash_to_ids_.find(*signature_hash);
                if (bucket_it != signature_hash_to_ids_.end()) {
                    const std::vector<std::int64_t> candidates = bucket_it->second;
                    for (std::int64_t existing_id : candidates) {
                        auto existing_node = nodes_.find(existing_id);
                        if (existing_node == nodes_.end() ||
                            !same_domain(existing_node->second, node)) {
                            continue;
                        }
                        if (existing_node->second.bound <=
                            node.bound + signature_bound_tol(node.bound)) {
                            return -1;
                        }
                        erase_id(existing_id);
                        break;
                    }
                }
            }
        }
        const auto id = next_id_++;
        if (domain_storage_enabled_ && !compact_stored_node(node)) {
            throw std::invalid_argument(
                "queued node domain does not match the configured root domain");
        }
        nodes_.emplace(id, std::move(node));
        if (signature_hash.has_value()) {
            signature_hash_to_ids_[*signature_hash].push_back(id);
            id_to_signature_hash_[id] = *signature_hash;
        }
        return id;
    }

    bool pop_dfs(Node& out) {
        if (dfs_.empty()) {
            return false;
        }

        const auto id = dfs_.back();

        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) {
            return false;
        }

        materialize_stored_node(node_it->second);
        dfs_.pop_back();

        const double bound = node_it->second.bound;
        auto dfs_bound_it = dfs_bounds_.find({bound, id});
        if (dfs_bound_it != dfs_bounds_.end()) {
            dfs_bounds_.erase(dfs_bound_it);
        }

        out = std::move(node_it->second);
        forget_domain_signature(id);
        nodes_.erase(node_it);
        return true;
    }

	    bool pop_priority(Node& out) {
        if (priority_.empty()) {
            return false;
        }

        auto it = priority_.begin();
        const auto [priority, id] = *it;

        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) {
            return false;
        }

        materialize_stored_node(node_it->second);
        priority_.erase(it);

        const double bound = node_it->second.bound;
        auto bound_it = bounds_.find({bound, id});
        if (bound_it != bounds_.end()) {
            bounds_.erase(bound_it);
        }

        out = std::move(node_it->second);
        forget_domain_signature(id);
        nodes_.erase(node_it);
        (void)priority;
	        return true;
	    }

	    void erase_id(std::int64_t id) {
	        auto node_it = nodes_.find(id);
	        if (node_it == nodes_.end()) {
	            return;
	        }
	        const double bound = node_it->second.bound;
	        auto prio_it = priority_.find({node_selection_priority(node_it->second), id});
	        if (prio_it != priority_.end()) {
	            priority_.erase(prio_it);
	        }
	        auto bound_it = bounds_.find({bound, id});
	        if (bound_it != bounds_.end()) {
	            bounds_.erase(bound_it);
	        }
	        auto dfs_bound_it = dfs_bounds_.find({bound, id});
	        if (dfs_bound_it != dfs_bounds_.end()) {
	            dfs_bounds_.erase(dfs_bound_it);
	        }
	        dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());
            forget_domain_signature(id);
	        release_compact_storage(node_it->second);
	        nodes_.erase(node_it);
	    }

        bool take_id(std::int64_t id, Node& out) {
            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) {
                return false;
            }
            materialize_stored_node(node_it->second);
            const double bound = node_it->second.bound;
            auto prio_it =
                priority_.find({node_selection_priority(node_it->second), id});
            if (prio_it != priority_.end()) {
                priority_.erase(prio_it);
            }
            auto bound_it = bounds_.find({bound, id});
            if (bound_it != bounds_.end()) {
                bounds_.erase(bound_it);
            }
            auto dfs_bound_it = dfs_bounds_.find({bound, id});
            if (dfs_bound_it != dfs_bounds_.end()) {
                dfs_bounds_.erase(dfs_bound_it);
            }
            dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());
            out = std::move(node_it->second);
            forget_domain_signature(id);
            nodes_.erase(node_it);
            return true;
        }
		};

inline NodeQueue::DomainMaterializationGuard::DomainMaterializationGuard(
    NodeQueue& owner)
    : owner_(owner) {
    owner_.materialize_all_domains();
}

inline NodeQueue::DomainMaterializationGuard::~DomainMaterializationGuard()
    noexcept {
    owner_.compact_all_domains_noexcept();
}

/// @brief Return true when the incumbent bound dominates the node bound.
bool incumbent_prunes_node(bool has_incumbent,
                           double incumbent_obj,
                           double node_bound,
                           double prune_tol = kIncumbentPruneTol);

/// @brief Round integer variables and clamp all variables to node bounds.
Eigen::VectorXd project_integer_solution(const std::vector<VariableMeta>& vars,
                                         const Eigen::VectorXd& x,
                                         const Eigen::VectorXd& lb,
                                         const Eigen::VectorXd& ub);

/// @brief Find indices of fractional integer variables.
/// @param[out] out Indices of fractional variables.
/// @return true if any fractional variables found.
bool fractional_indices(const std::vector<VariableMeta>& vars,
                        const Eigen::VectorXd& x,
                        double int_tol,
                        std::vector<int>& out);

/// @brief Check that lb[i] <= ub[i] for all i (within tolerance).
bool bounds_consistent(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/// @brief Apply node bounds by tightening variable metadata.
void apply_node_bounds(std::vector<VariableMeta>& vars,
                       const Eigen::VectorXd& lb,
                       const Eigen::VectorXd& ub);

}  // namespace mipsolvers::engine::detail

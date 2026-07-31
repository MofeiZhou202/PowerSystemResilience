/// @file bc_utils.hpp
/// @brief Declarations for utility, branching, cut generation, and relaxation functions.
///
/// Groups all non-class helper function declarations used by the B&C modules.
/// Implementations are in bc_utils.cpp, bc_branching.cpp, bc_cuts.cpp, bc_relaxation.cpp.

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
#include <string>
#include <unordered_map>
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
 public:
    explicit NodeQueue(NodeSelection mode) : mode_(mode) {}

    void configure_domain_signature(const std::vector<char>& branchable_cols,
                                    const Eigen::VectorXd& root_lb,
                                    const Eigen::VectorXd& root_ub,
                                    double tol = 1e-9) {
        domain_signature_cols_.clear();
        domain_signature_cols_.reserve(branchable_cols.size());
        for (std::size_t j = 0; j < branchable_cols.size(); ++j) {
            if (branchable_cols[j] != 0) {
                domain_signature_cols_.push_back(static_cast<int>(j));
            }
        }
        domain_signature_root_lb_ = root_lb;
        domain_signature_root_ub_ = root_ub;
        domain_signature_tol_ = std::max(1e-12, tol);
        domain_signature_enabled_ =
            !domain_signature_cols_.empty() &&
            root_lb.size() == root_ub.size();
        signature_to_id_.clear();
        id_to_signature_.clear();
        if (!domain_signature_enabled_) return;
        std::vector<std::int64_t> doomed;
        for (const auto& [id, node] : nodes_) {
            auto sig = make_domain_signature(node);
            if (!sig.has_value()) continue;
            const auto existing = signature_to_id_.find(*sig);
            if (existing == signature_to_id_.end()) {
                signature_to_id_.emplace(*sig, id);
                id_to_signature_.emplace(id, *sig);
                continue;
            }
            const auto keep_it = nodes_.find(existing->second);
            if (keep_it != nodes_.end() &&
                keep_it->second.bound <= node.bound + signature_bound_tol(node.bound)) {
                doomed.push_back(id);
            } else {
                doomed.push_back(existing->second);
                signature_to_id_[*sig] = id;
                id_to_signature_[id] = *sig;
            }
        }
        for (std::int64_t id : doomed) erase_id(id);
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
            auto it = dfs_bounds_.begin();
            const auto [bound, id] = *it;
            dfs_bounds_.erase(it);

            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) {
                return false;
            }

            auto prio_it = priority_.find({node_selection_priority(node_it->second), id});
            if (prio_it != priority_.end()) {
                priority_.erase(prio_it);
            }
            auto bound_it = bounds_.find({bound, id});
            if (bound_it != bounds_.end()) {
                bounds_.erase(bound_it);
            }
            dfs_.erase(std::remove(dfs_.begin(), dfs_.end(), id), dfs_.end());

            out = std::move(node_it->second);
            nodes_.erase(node_it);
            return true;
        }

        auto it = bounds_.begin();
        const auto [bound, id] = *it;
        bounds_.erase(it);

        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) {
            return false;
        }

        auto prio_it = priority_.find({node_selection_priority(node_it->second), id});
        if (prio_it != priority_.end()) {
            priority_.erase(prio_it);
        }

        out = std::move(node_it->second);
        nodes_.erase(node_it);
        (void)bound;
        return true;
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
        if (nodes_.empty() || root_lb.size() != root_ub.size()) return 0;

        std::vector<std::int64_t> ids;
        ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            ids.push_back(id);
        }

        if (domain_signature_enabled_) {
            domain_signature_root_lb_ = root_lb;
            domain_signature_root_ub_ = root_ub;
            signature_to_id_.clear();
            id_to_signature_.clear();
        }

        const double tol = domain_signature_enabled_
                               ? std::max(1e-12, domain_signature_tol_)
                               : 1e-9;
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
	                    literal.var_idx >= static_cast<int>(node.lb.size()) ||
	                    literal.var_idx >= static_cast<int>(node.ub.size())) {
	                    unsatisfied = 2;
	                    break;
	                }
	                const bool active = literal.is_lb
	                    ? (node.lb[literal.var_idx] >= literal.value - tol)
	                    : (node.ub[literal.var_idx] <= literal.value + tol);
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
	                    literal.var_idx >= static_cast<int>(node.lb.size()) ||
	                    literal.var_idx >= static_cast<int>(node.ub.size())) {
	                    unsatisfied = 2;
	                    break;
	                }
	                const bool active = literal.is_lb
	                    ? (node.lb[literal.var_idx] >= literal.value - tol)
	                    : (node.ub[literal.var_idx] <= literal.value + tol);
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
                    if (branch_literal_stronger_or_equal(existing.bound, rb.bound) &&
                        branch_literal_stronger_or_equal(rb.bound, existing.bound) &&
                        conflict_clause_hash(existing.reason) == reason_hash) {
                        return true;
                    }
                }
                for (const auto& existing : pending) {
                    if (branch_literal_stronger_or_equal(existing.bound, rb.bound) &&
                        branch_literal_stronger_or_equal(rb.bound, existing.bound) &&
                        conflict_clause_hash(existing.reason) == reason_hash) {
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
                        if (branch_literal_stronger_or_equal(existing.bound, rb.bound) &&
                            branch_literal_stronger_or_equal(rb.bound, existing.bound) &&
                            conflict_clause_hash(existing.reason) == reason_hash) {
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
		                    if (branch_literal_stronger_or_equal(existing.bound, rb.bound) &&
		                        branch_literal_stronger_or_equal(rb.bound, existing.bound) &&
	                        conflict_clause_hash(existing.reason) ==
	                            conflict_clause_hash(rb.reason)) {
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
    double domain_signature_tol_{1e-9};
    std::vector<int> domain_signature_cols_;
    Eigen::VectorXd domain_signature_root_lb_;
    Eigen::VectorXd domain_signature_root_ub_;
    std::unordered_map<std::string, std::int64_t> signature_to_id_;
    std::unordered_map<std::int64_t, std::string> id_to_signature_;
    bool hybrid_post_incumbent_active_{false};
    std::int64_t hybrid_queue_leaves_{0};
    std::int64_t hybrid_last_lower_bound_leave_{0};
    static constexpr std::int64_t kHybridBestBoundPeriod = 10;

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
        nodes_.erase(node_it);
    }

    double signature_bound_tol(double bound) const {
        return std::max(1e-9, 1e-12 * std::max(1.0, std::abs(bound)));
    }

    static std::int64_t quantize_lower_bound(double value, double tol) {
        if (!std::isfinite(value)) return std::numeric_limits<std::int64_t>::min();
        const long double q = std::ceil(static_cast<long double>(value) -
                                        static_cast<long double>(tol));
        const long double lo = static_cast<long double>(
            std::numeric_limits<std::int64_t>::min() + 1);
        const long double hi = static_cast<long double>(
            std::numeric_limits<std::int64_t>::max() - 1);
        return static_cast<std::int64_t>(std::max(lo, std::min(hi, q)));
    }

    static std::int64_t quantize_upper_bound(double value, double tol) {
        if (!std::isfinite(value)) return std::numeric_limits<std::int64_t>::max();
        const long double q = std::floor(static_cast<long double>(value) +
                                         static_cast<long double>(tol));
        const long double lo = static_cast<long double>(
            std::numeric_limits<std::int64_t>::min() + 1);
        const long double hi = static_cast<long double>(
            std::numeric_limits<std::int64_t>::max() - 1);
        return static_cast<std::int64_t>(std::max(lo, std::min(hi, q)));
    }

    template <typename T>
    static void append_signature_bytes(std::string& sig, const T& value) {
        sig.append(reinterpret_cast<const char*>(&value), sizeof(T));
    }

    std::optional<std::string> make_domain_signature(const Node& node) const {
        if (!domain_signature_enabled_ ||
            node.lb.size() != node.ub.size() ||
            node.lb.size() != domain_signature_root_lb_.size() ||
            node.ub.size() != domain_signature_root_ub_.size()) {
            return std::nullopt;
        }
        std::string sig;
        sig.reserve(domain_signature_cols_.size() * 18);
        constexpr char lower_tag = 'L';
        constexpr char upper_tag = 'U';
        for (int j : domain_signature_cols_) {
            if (j < 0 || j >= node.lb.size()) continue;
            const double root_lb = domain_signature_root_lb_[j];
            const double root_ub = domain_signature_root_ub_[j];
            const double lb = node.lb[j];
            const double ub = node.ub[j];
            const bool lower_active =
                std::isfinite(lb) &&
                (!std::isfinite(root_lb) || lb > root_lb + domain_signature_tol_);
            if (lower_active) {
                const std::int32_t col = static_cast<std::int32_t>(j);
                const std::int64_t val =
                    quantize_lower_bound(lb, domain_signature_tol_);
                append_signature_bytes(sig, col);
                sig.push_back(lower_tag);
                append_signature_bytes(sig, val);
            }
            const bool upper_active =
                std::isfinite(ub) &&
                (!std::isfinite(root_ub) || ub < root_ub - domain_signature_tol_);
            if (upper_active) {
                const std::int32_t col = static_cast<std::int32_t>(j);
                const std::int64_t val =
                    quantize_upper_bound(ub, domain_signature_tol_);
                append_signature_bytes(sig, col);
                sig.push_back(upper_tag);
                append_signature_bytes(sig, val);
            }
        }
        return sig;
    }

    void forget_domain_signature(std::int64_t id) {
        auto sig_it = id_to_signature_.find(id);
        if (sig_it == id_to_signature_.end()) return;
        auto owner_it = signature_to_id_.find(sig_it->second);
        if (owner_it != signature_to_id_.end() && owner_it->second == id) {
            signature_to_id_.erase(owner_it);
        }
        id_to_signature_.erase(sig_it);
    }

    bool refresh_domain_signature_after_update(std::int64_t id) {
        if (!domain_signature_enabled_) return nodes_.find(id) != nodes_.end();
        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) return false;
        forget_domain_signature(id);
        auto sig = make_domain_signature(node_it->second);
        if (!sig.has_value()) return true;
        auto existing = signature_to_id_.find(*sig);
        if (existing != signature_to_id_.end() && existing->second != id) {
            auto existing_node = nodes_.find(existing->second);
            if (existing_node != nodes_.end()) {
                if (existing_node->second.bound <=
                    node_it->second.bound + signature_bound_tol(node_it->second.bound)) {
                    erase_id(id);
                    return false;
                }
                erase_id(existing->second);
            } else {
                signature_to_id_.erase(existing);
            }
        }
        signature_to_id_[*sig] = id;
        id_to_signature_[id] = std::move(*sig);
        return true;
    }

    void rebuild_domain_signatures() {
        if (!domain_signature_enabled_) return;
        signature_to_id_.clear();
        id_to_signature_.clear();
        std::vector<std::int64_t> ids;
        ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            ids.push_back(id);
        }
        for (std::int64_t id : ids) {
            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) continue;
            auto sig = make_domain_signature(node_it->second);
            if (!sig.has_value()) continue;
            auto existing = signature_to_id_.find(*sig);
            if (existing == signature_to_id_.end()) {
                signature_to_id_[*sig] = id;
                id_to_signature_[id] = std::move(*sig);
                continue;
            }
            auto existing_node = nodes_.find(existing->second);
            if (existing_node != nodes_.end() &&
                existing_node->second.bound <=
                    node_it->second.bound + signature_bound_tol(node_it->second.bound)) {
                erase_id(id);
            } else {
                if (existing_node != nodes_.end()) {
                    erase_id(existing->second);
                }
                signature_to_id_[*sig] = id;
                id_to_signature_[id] = std::move(*sig);
            }
        }
    }

    std::int64_t store(Node node) {
        std::optional<std::string> sig;
        if (domain_signature_enabled_) {
            sig = make_domain_signature(node);
            if (sig.has_value()) {
                auto existing = signature_to_id_.find(*sig);
                if (existing != signature_to_id_.end()) {
                    auto existing_node = nodes_.find(existing->second);
                    if (existing_node != nodes_.end()) {
                        if (existing_node->second.bound <=
                            node.bound + signature_bound_tol(node.bound)) {
                            return -1;
                        }
                        erase_id(existing->second);
                    } else {
                        signature_to_id_.erase(existing);
                    }
                }
            }
        }
        const auto id = next_id_++;
        nodes_.emplace(id, std::move(node));
        if (sig.has_value()) {
            signature_to_id_[*sig] = id;
            id_to_signature_[id] = std::move(*sig);
        }
        return id;
    }

    bool pop_dfs(Node& out) {
        if (dfs_.empty()) {
            return false;
        }

        const auto id = dfs_.back();
        dfs_.pop_back();

        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) {
            return false;
        }

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
        priority_.erase(it);

        auto node_it = nodes_.find(id);
        if (node_it == nodes_.end()) {
            return false;
        }

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
	        nodes_.erase(node_it);
	    }

        bool take_id(std::int64_t id, Node& out) {
            auto node_it = nodes_.find(id);
            if (node_it == nodes_.end()) {
                return false;
            }
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

/// @brief Add rows (cuts) to an LP model's inequality system.
void add_rows_to_lp(LPModel& lp,
                     const std::vector<Eigen::VectorXd>& rows,
                     const std::vector<double>& rhs_vals);

/// @brief Add sparse rows (cuts) to an LP model's inequality system.
void add_sparse_rows_to_lp(LPModel& lp,
                            const std::vector<Eigen::SparseVector<double>>& rows,
                            const std::vector<double>& rhs_vals);

/// @brief Check if x satisfies all constraints of lp within tolerance.
bool satisfies_lp(const LPModel& lp, const Eigen::VectorXd& x, double tol);

/// @brief Compute an estimate for a node from fractional variables and pseudocost history.
double compute_node_estimate(const std::vector<VariableMeta>& vars,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound);

/// Compute a node estimate by scanning only precomputed branchable columns.
/// This avoids an O(number-of-all-columns) pass at every B&B node when a large
/// model has comparatively few integer variables.
double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound);

double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const CompactPseudoCostTable& pc,
                             double int_tol,
                             double node_bound);

/// @brief Append or strengthen a branch-domain reason on a node.
void append_branch_reason(Node& node,
                          int var_idx,
                          bool is_lb,
                          double value);

/// @brief Build a conservative binary no-good cut from a node domain.
/// Returns false if the reason trail is not a pure binary fixing conflict.
bool try_build_binary_conflict_cut(const LPModel& base_lp,
                                   const std::vector<BranchDomainLiteral>& reasons,
                                   PoolCut& out_cut,
                                   int max_literals = 64);

/// @brief Push a node onto the appropriate queue (PQ or DFS stack).
void push_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
               std::vector<Node>& dfs,
               const Node& node,
               NodeSelection mode,
               bool has_incumbent);

/// @brief Pop the next node from the queue.
/// @return true if a node was popped, false if queue is empty.
bool pop_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
              std::vector<Node>& dfs,
              NodeSelection mode,
              bool has_incumbent,
              Node& out);

/// @brief Compute the minimum lower bound across all live nodes.
double live_node_lower_bound(const std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
                             const std::vector<Node>& dfs);

/// @brief MILP presolve: bound tightening + variable fixing.
/// @return Number of variables fixed.
int milp_presolve(LPModel& lp, int max_rounds = 10);

// ── Branching functions (bc_branching.cpp) ───────────────────────────────

/// @brief Select branch variable by most-infeasible rule.
int choose_branch_var_most_infeasible(const std::vector<int>& cand,
                                      const Eigen::VectorXd& x);

/// @brief Select branch variable by pseudo-cost scoring (product rule).
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc);

/// @brief Select branch variable by pseudo-cost scoring with priority weighting.
/// Priority values > 0 give a soft multiplicative bonus to the pseudocost score.
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context);

/// @brief Select branch variable by pseudo-cost, static priority, and a dynamic prior callback.
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context);

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const std::vector<PseudoCost>& pc,
                                const std::vector<int>* priority = nullptr);

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const CompactPseudoCostTable& pc,
                                const std::vector<int>* priority = nullptr);

/// @brief Select branch variable using the configured strategy.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc);

/// @brief Select branch variable using the configured strategy with priority.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context);

/// @brief Select branch variable using static priorities and a dynamic prior callback.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context);

/// @brief Reliability branching: probe unreliable candidates with LP solves.
/// @details For candidates with insufficient pseudo-cost data (< reliability_limit
/// observations), solves child LPs to get real estimates. Only probes at depth <= 3.
/// @param cand Candidate fractional variable indices.
/// @param x Current LP relaxation solution.
/// @param pc Pseudo-cost table (updated in-place with probe results).
/// @param base_lp Base LP model.
/// @param base_sf Base standard-form LP for bound updates.
/// @param node_lb Current node lower bounds.
/// @param node_ub Current node upper bounds.
/// @param basis_hint Simplex basis for warm-starting probes.
/// @param simplex_opt Simplex solver options.
/// @param parent_bound Parent node's LP bound (for gain computation).
/// @param[in,out] lp_solves Counter incremented for each probe LP solve.
/// @param node_depth Current depth in the B&C tree.
/// @param reliability_limit Minimum observations before trusting pseudo-costs.
/// @param max_probes Maximum strong-branching probes per node.
/// @return Index of selected branching variable.
int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    std::vector<PseudoCost>& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit = 1,
    int max_probes = 4);

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    CompactPseudoCostTable& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit = 1,
    int max_probes = 4);

// ── Cut generation functions (bc_cuts.cpp) ───────────────────────────────

/// @brief Dispatch cut generation based on BCOptions::cuts.
/// @details Calls the appropriate cut family (GMI, MIR, cover, or all) within
/// a budget. Cuts are appended as new inequality rows to lp.
/// @param[in,out] lp LP model to add cuts to.
/// @param x Current fractional LP solution.
/// @param simplex Simplex result (needed for tableau-based cuts); nullptr to skip GMI.
/// @param opt B&C options controlling cut types and quality thresholds.
/// @param max_cuts Maximum cuts to generate.
/// @param sbasis Sparse basis factorization for BTRAN (optional, for GMI).
/// @param tracker Optional per-family efficacy tracker for adaptive cut generation (P2.1).
/// @return Number of cuts added to the LP.
int add_cuts(LPModel& lp,
             const Eigen::VectorXd& x,
             const SimplexResult* simplex,
             const BCOptions& opt,
             int max_cuts,
             const std::shared_ptr<BasisOps>& sbasis = nullptr,
             CutFamilyTracker* tracker = nullptr,
             const class CliqueTable* clique_table = nullptr);

/// @brief Generate Gomory Mixed-Integer cuts from the simplex basis.
/// @details Constructs GMI cuts from the bounded-form simplex tableau for each
/// fractional basic integer variable. Filters by density, efficacy, activity,
/// parallelism, and binary support ratio. Batch-inserts selected cuts.
/// @param[in,out] lp LP model to add cuts to.
/// @param x Current fractional LP solution.
/// @param simplex Simplex result with basis and tableau data.
/// @param opt B&C options (GMI quality thresholds).
/// @param max_cuts Maximum cuts to select.
/// @param sbasis Sparse basis factorization for BTRAN (optional).
/// @return Number of cuts added.
int add_basis_gomory_cuts(LPModel& lp,
                          const Eigen::VectorXd& x,
                          const SimplexResult& simplex,
                          const BCOptions& opt,
                          int max_cuts,
                          const std::shared_ptr<BasisOps>& sbasis = nullptr);

/// @brief Generate HiGHS-style transformed tableau CMIR cuts at the root.
/// @details Aggregates LP rows with a basis-inverse row, converts the
/// aggregate into the bounded shifted standard space, generates a CMIR cut,
/// untransforms through the exact standard-form row coefficients, and appends
/// only rows violated by the current original-space LP solution.
int add_transformed_tableau_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::shared_ptr<BasisOps>& sbasis = nullptr,
	    const std::vector<char>* implied_integer_cols = nullptr,
	    const BinaryImplicationGraph* implication_graph = nullptr,
	    const VariableBoundTable* variable_bound_table = nullptr,
	    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
	    double cut_generation_feastol = 1e-8,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
	    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt);

/// @brief Generate HiGHS-style transformed path aggregation CMIR cuts.
/// @details Starts from active row sides, projects through short row paths that
/// cancel continuous columns in transformed space, then uses the same bounded
/// transformed cut generator and exact untransform as transformed tableau cuts.
int add_transformed_path_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
	    const std::vector<char>* implied_integer_cols = nullptr,
	    const BinaryImplicationGraph* implication_graph = nullptr,
	    const VariableBoundTable* variable_bound_table = nullptr,
	    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
	    double cut_generation_feastol = 1e-8,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
	    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt,
	    const std::function<int(int)>* highs_path_randint = nullptr);

/// @brief Generate HiGHS-style mod-k cuts from transformed active rows.
/// @details Mirrors HighsModkSeparator: skip rows containing continuous
/// variables with nonzero transformed bound distance, build a GF(k) row system
/// from active integral rows, and add generated cuts directly to the cutpool.
int add_transformed_modk_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    const std::vector<char>* implied_integer_cols = nullptr,
    const BinaryImplicationGraph* implication_graph = nullptr,
    const VariableBoundTable* variable_bound_table = nullptr,
    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
    double cut_generation_feastol = 1e-8,
    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt);

// ── Relaxation functions (bc_relaxation.cpp) ─────────────────────────────

/// @brief Solve the LP relaxation at a B&C node.
/// @details Uses simplex (if opt.use_simplex_lp_nodes) or IPM fallback.
/// Returns primal solution, dual bound, and basis hint for warm-starting children.
/// @param lp LP model with node-specific bounds already applied.
/// @param x0 Optional starting point (for IPM path).
/// @param basis_hint Simplex basis from parent node (for warm-start).
/// @param opt B&C options.
/// @return Primal result, dual bound, and basis hint.
LPRelaxationResult solve_lp_relaxation(const LPModel& lp,
                                       const Eigen::VectorXd* x0,
                                       const SimplexBasis* basis_hint,
                                       const BCOptions& opt);

/// @brief Recover a crash basis from a primal point, optionally constrained by
/// IPM dual/complementarity signals and seeded from an already valid basis.
/// @details When seed_basis is provided, only rows currently using
/// slack/artificial columns are considered for structural swaps. This keeps a
/// known-valid basis intact and only repairs the newly introduced rows.
CrashBasisRecoveryStats recover_primal_activity_basis(const StandardFormLP& sf,
                                                      const Eigen::VectorXd& x_orig,
                                                      const SolveResult* ipm_res,
                                                      const SimplexBasis* seed_basis,
                                                      SimplexBasis& out_basis);

/// @brief Solve the NLP relaxation at a B&C node (for MINLP).
/// @param nlp_base Base NLP model (bounds updated per node).
/// @param lb Node lower bounds.
/// @param ub Node upper bounds.
/// @param x0 Optional starting point.
/// @param opt B&C options.
/// @return Primal solution and convergence info.
SolveResult solve_nlp_relaxation(const NLPModel& nlp_base,
                                 const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub,
                                 const Eigen::VectorXd* x0,
                                 const BCOptions& opt);

/// @brief Lightweight rounding heuristic for child nodes.
/// @return true if a new incumbent was found.
bool try_rounding_heuristic(const LPModel& base_lp,
                            const Eigen::VectorXd& x_relax,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub,
                            double int_tol,
                            bool& has_incumbent,
                            double& incumbent_obj,
                            Eigen::VectorXd& incumbent_x,
                            SolutionPool* sol_pool = nullptr);

/// @brief Check if x satisfies all constraints using explicit bounds (avoids LPModel copy).
/// @details Uses base_lp constraint matrices but checks bounds from node_lb/node_ub.
bool satisfies_with_bounds(const LPModel& lp,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& node_lb,
                           const Eigen::VectorXd& node_ub,
                           double tol);

/// @brief Fix integer variables whose reduced costs prove they can't improve the incumbent.
/// @details After solving LP at a node, for each non-basic integer variable j:
///   If j is at lower bound and rc[j] >= gap: fix j at lb (ub[j] = lb[j])
///   If j is at upper bound and |rc[j]| >= gap: fix j at ub (lb[j] = ub[j])
/// @return Number of variables fixed.
int reduced_cost_fixing(const std::vector<VariableMeta>& vars,
                        const Eigen::VectorXd& x_relax,
                        const Eigen::VectorXd& reduced_costs,
                        const std::vector<int>& basis_indices,
                        double node_bound,
                        double incumbent_obj,
                        double int_tol,
                        Eigen::VectorXd& node_lb,
                        Eigen::VectorXd& node_ub,
                        std::vector<BranchDomainLiteral>* forbidden_literals = nullptr,
                        const Eigen::VectorXd* sf_col_scale = nullptr,
                        std::vector<DomainReasonBound>* reason_bounds = nullptr,
                        const std::vector<BranchDomainLiteral>* reason_frontier = nullptr,
                        int reason_depth = 0,
                        int reason_trail_offset = 0);

/// @brief Full LP row-dual objective-cutoff proof row.
///
/// For the effective minimization LP and incumbent cutoff U, the row is
///     coeff^T x <= rhs
/// with
///     coeff = c - A^T y,   rhs = U - b^T y,
/// where y is recovered from the simplex basis row duals and mapped back to
/// the original LP rows. This is the native analogue of HiGHS
/// HighsLpRelaxation::computeDualProof().
struct DualProofRow {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
  double min_activity{0.0};
  double max_activity{0.0};
  double lp_activity{0.0};
  double expected_lp_gap{0.0};
  double actual_lp_gap{0.0};
  double gap_error{0.0};
  double max_stationarity_error{0.0};
  double max_scaled_stationarity_error{0.0};
  double max_cost_mapping_error{0.0};
  int nnz{0};
  int row_dual_nnz{0};
  int ignored_row_duals{0};
  int removed_bound_coefficients{0};
  int tightened_coefficients{0};
  bool used_solver_row_duals{false};
  bool valid{false};
  std::string reject_reason;
};

/// @brief Build a HiGHS-style full row-dual cutoff proof from a simplex result.
bool build_full_dual_proof_row(const LPModel& lp,
                               const SimplexResult& simplex,
                               const Eigen::VectorXd& root_lb,
                               const Eigen::VectorXd& root_ub,
                               const Eigen::VectorXd& x_lp,
                               double lp_objective,
                               double incumbent_obj,
                               DualProofRow& out,
                               double tol = 1e-8,
                               const StandardFormLP* form_override = nullptr);

/// @brief Reduced-cost-mass objective-cutoff proof row.
///
/// This is a stricter source row for conflict/reconvergence target selection:
/// it keeps only non-basic integer original columns whose reduced-cost
/// coefficient is active away from the global/root side.  All other
/// coefficients are shifted to the safe root-bound side, so the resulting
/// row remains a valid local objective-cutoff proof while exposing frontier
/// terms that carry actual reduced-cost mass.
bool build_reduced_cost_mass_dual_proof_row(
    const LPModel& lp,
    const SimplexResult& simplex,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_lp,
    double lp_objective,
    double incumbent_obj,
    DualProofRow& out,
    double tol = 1e-8,
    const StandardFormLP* form_override = nullptr,
    const std::vector<char>* implied_integer_cols = nullptr,
    const std::vector<char>* force_keep_cols = nullptr);

/// @brief Consume a cutoff dual proof as direct domain fixing.
///
/// For a valid proof row `coeff^T x <= rhs`, if the minimum activity over the
/// current local domain already exceeds `rhs`, the node is cutoff.  Otherwise
/// each coefficient gives the strongest one-variable bound that can still keep
/// the proof row feasible.  This is the proof-row analogue of HiGHS
/// reduced-cost/objective propagation: first obtain a real LP certificate, then
/// turn it into local bound changes before re-solving the node LP.
///
/// @return -1 if the domain is cutoff, otherwise the number of bounds changed.
int apply_dual_proof_domain_fixing(
    const std::vector<VariableMeta>& vars,
    const DualProofRow& proof,
    double int_tol,
    double lp_tol,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    std::vector<BoundChangeInfo>* changes_out = nullptr);

/// @brief Build a shortened reduced-cost cutoff conflict from the LP proof row.
///
/// The returned clause is a proof-cover version of
/// `branch_path + forbidden_literal`: using the current node reduced costs, it
/// keeps only branch-domain bounds whose local-bound activity is sufficient to
/// make the forbidden literal violate the incumbent cutoff row.  This is the
/// native analogue of HiGHS resolving a dual/objective proof against domain
/// reasons; if the proof cover cannot be certified, the function returns false
/// and callers should fall back to the conservative full-path clause.
bool build_reduced_cost_cutoff_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    double node_bound,
    double incumbent_obj,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const BranchDomainLiteral& forbidden,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin = nullptr,
    std::vector<std::vector<BranchDomainLiteral>>* reconvergence_clauses = nullptr,
    PoolCut* violated_local_proof_cover = nullptr,
    const DualProofRow* dual_proof = nullptr);

/// @brief Structured output for target-bound row-dual explanation.
///
/// HiGHS stores reconvergence cuts as `flip(target)` plus the resolved proof
/// frontier.  Keeping these fields separate lets callers admit true binary
/// implications only when the frontier itself resolves to one literal.
struct DualProofTargetExplanation {
  bool valid{false};
  bool cutoff_conflict{false};
  bool has_proved_target_bound{false};
  BranchDomainLiteral proved_target_bound;
  BranchDomainLiteral flipped_target;
  std::vector<BranchDomainLiteral> initial_frontier;
  std::vector<BranchDomainLiteral> resolved_frontier;
  std::vector<BranchDomainLiteral> clause;
  double proof_margin{0.0};
  double proof_budget{0.0};
  double frontier_lp_activity{0.0};
  double local_proof_cover_violation{0.0};
  int local_proof_cover_nnz{0};
  bool has_proof_activity_audit{false};
  double proof_activity{0.0};
  double proof_required_activity{0.0};
};

/// @brief Explain a generic target bound change with a full row-dual proof.
///
/// This is the native analogue of HiGHS
/// HighsDomain::conflictAnalyzeReconvergence(domchg, proof row): a local
/// target bound is proved from the objective-cutoff row using only prior
/// branch/propagation reasons, then emitted as a reason-side frontier plus the
/// flipped target literal.
bool build_dual_proof_target_bound_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin = nullptr,
    PoolCut* violated_local_proof_cover = nullptr,
    const DualProofRow* dual_proof = nullptr,
    DualProofTargetExplanation* explanation = nullptr,
    std::vector<PoolCut>* additional_local_proof_covers = nullptr,
    const DualProofRow* priority_proof = nullptr);

/// @brief Propagate bound changes through constraints to tighten other variable bounds.
/// @details Performs constraint-based bound tightening at tree nodes.
/// @return Number of bounds tightened.
int node_bound_propagation(const LPModel& lp,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds = 2);

/// @brief Overload accepting pre-computed row-major sparse matrices to avoid
/// per-call format conversion overhead in the B&C loop.
int node_bound_propagation(const LPModel& lp,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds = 2);

/// @brief Tracked variant: records every bound change as BoundChangeInfo.
/// Used for incremental standard-form updates (avoids full O(n) scan).
int node_bound_propagation_tracked(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    std::vector<BoundChangeInfo>& changes_out);

/// @brief Persistent binary implication graph built from root probing.
/// @details Stores implications of the form `(x_j = 0/1) => bound(k)`.
/// Propagation is event-driven: binary literals fixed by branching, clique
/// propagation, or row tightening are pushed into a queue, and all reachable
/// implications are applied until closure.
class BinaryImplicationGraph {
 public:
    struct Implication {
        int var_idx{-1};
        double value{0.0};
        bool is_lb{false};
    };

    BinaryImplicationGraph() = default;
    explicit BinaryImplicationGraph(int n_vars) { reset(n_vars); }

    void reset(int n_vars);
    bool empty() const { return num_implications_ == 0; }
    std::size_t size() const { return num_implications_; }

    bool add_implication(int trigger_var,
                                             bool trigger_value_one,
                                             int implied_var,
                                             bool implied_is_lb,
                                             double implied_value);

    std::pair<const Implication*, const Implication*> implications(int trigger_var,
                                                                   bool trigger_value_one) const {
        if (trigger_var < 0 || trigger_var >= n_vars_ || adjacency_.empty()) {
            return {nullptr, nullptr};
        }
        const auto& bucket = adjacency_[literal_index(trigger_var, trigger_value_one)];
        return {bucket.data(), bucket.data() + bucket.size()};
    }

    bool implies_binary_value(int trigger_var,
                              bool trigger_value_one,
                              int implied_var,
                              bool implied_value_one,
                              double tol = 1e-9) const {
        auto range = implications(trigger_var, trigger_value_one);
        for (const Implication* p = range.first; p != range.second; ++p) {
            if (p == nullptr || p->var_idx != implied_var) continue;
            if (implied_value_one && p->is_lb && p->value >= 1.0 - tol) {
                return true;
            }
            if (!implied_value_one && !p->is_lb && p->value <= tol) {
                return true;
            }
        }
        return false;
    }

    int propagate(const std::vector<VariableMeta>& vars,
	                                Eigen::VectorXd& lb,
	                                Eigen::VectorXd& ub,
	                                std::vector<BoundChangeInfo>* changes_out = nullptr,
	                                double tol = 1e-9) const;

 private:
    static std::size_t literal_index(int var_idx, bool value_one) {
        return static_cast<std::size_t>(2 * var_idx + (value_one ? 1 : 0));
    }

    int n_vars_{0};
    std::size_t num_implications_{0};
    std::vector<std::vector<Implication>> adjacency_;
};

/// @brief First-class variable-bound table, mirroring HiGHS' VUB/VLB store.
/// @details Stores source relations of the form
/// `x_col <= coef * y_trigger + constant` (VUB) and
/// `x_col >= coef * y_trigger + constant` (VLB), where `y_trigger` is a
/// binary trigger.  These objects are stronger source material than the
/// downstream implications obtained by evaluating the row at y=0 and y=1:
/// transformed tableau/path separation can substitute the whole affine bound.
class VariableBoundTable {
 public:
    struct VarBound {
        double coef{0.0};
        double constant{0.0};

        double min_value() const {
            return constant + std::min(coef, 0.0);
        }
        double max_value() const {
            return constant + std::max(coef, 0.0);
        }
    };

    struct Entry {
        int trigger_col{-1};
        VarBound bound;
    };

    struct SourceStats {
        std::uint64_t vub_attempts{0};
        std::uint64_t vub_accepted{0};
        std::uint64_t vub_replaced{0};
        std::uint64_t vlb_attempts{0};
        std::uint64_t vlb_accepted{0};
        std::uint64_t vlb_replaced{0};
        std::uint64_t mir_attempts{0};
        std::uint64_t mir_strengthened{0};
        std::uint64_t redundant{0};
    };

    VariableBoundTable() = default;
    explicit VariableBoundTable(int n_vars) { reset(n_vars); }

    void reset(int n_vars);
    bool empty() const { return num_varbounds_ == 0; }
    std::size_t size() const { return num_varbounds_; }
    int num_vars() const { return n_vars_; }
    const SourceStats& stats() const { return stats_; }

    const std::vector<Entry>& vubs(int col) const;
    const std::vector<Entry>& vlbs(int col) const;

    bool add_vub(int col, int trigger_col, double coef, double constant,
                 double col_upper_bound, bool col_is_integral);
    bool add_vlb(int col, int trigger_col, double coef, double constant,
                 double col_lower_bound, bool col_is_integral);

    std::uint64_t export_to_implication_graph(
        BinaryImplicationGraph& implication_graph,
        double tol = 1e-9) const;

 private:
    bool valid_indices(int col, int trigger_col) const {
        return col >= 0 && trigger_col >= 0 && col < n_vars_ &&
               trigger_col < n_vars_ && col != trigger_col;
    }

    static bool strengthen_var_bound(VarBound& bound, int multiplier);

    int n_vars_{0};
    std::size_t num_varbounds_{0};
    std::vector<std::vector<Entry>> vubs_;
    std::vector<std::vector<Entry>> vlbs_;
    SourceStats stats_;
};

inline int NodeQueue::prune_by_implications(
    const std::vector<VariableMeta>& vars,
    const BinaryImplicationGraph& implication_graph,
    const ConflictPool* conflict_pool,
    bool has_incumbent,
    double incumbent_obj,
	    double tol,
	    std::uint64_t* tightened_out,
	    double bound_band,
	    int max_nodes,
	    const Eigen::VectorXd* root_lb,
	    const Eigen::VectorXd* root_ub) {
    if (tightened_out != nullptr) *tightened_out = 0;
    if (implication_graph.empty()) return 0;
    std::vector<std::int64_t> doomed;
    doomed.reserve(nodes_.size());

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
        auto node_find = nodes_.find(id);
        if (node_find == nodes_.end()) continue;
        Node& node = node_find->second;
        if (has_incumbent &&
            incumbent_prunes_node(true, incumbent_obj, node.bound)) {
            doomed.push_back(id);
            continue;
        }
        Eigen::VectorXd propagated_lb = node.lb;
        Eigen::VectorXd propagated_ub = node.ub;
        std::vector<BoundChangeInfo> changes;
        implication_graph.propagate(vars, propagated_lb, propagated_ub,
                                    &changes, tol);
        if (!bounds_consistent(propagated_lb, propagated_ub) ||
            (conflict_pool != nullptr &&
             conflict_pool->has_conflict(propagated_lb, propagated_ub))) {
            doomed.push_back(id);
            continue;
        }
        // HiGHS keeps propagation objects live in the domain.  Do the same for
        // queued native nodes: root/global implications are valid in every
        // descendant, so carrying the tightened domain forward lets subsequent
        // certificate lower-bound passes and the eventual node LP see the same
        // first-class bounds instead of rediscovering them later.
        if (!changes.empty()) {
            const auto index_membership = detach_queue_indices(id, node);
            node.lb = std::move(propagated_lb);
            node.ub = std::move(propagated_ub);
            if (node.x_seed.size() == node.lb.size()) {
                node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
            }
            node.lp_refresh_needed = true;
            rebuild_local_domain_trail(
                node, static_cast<int>(node.lb.size()), root_lb, root_ub);
            if (!refresh_domain_signature_after_update(id)) {
                erase_detached_node(id);
                continue;
            }
            reattach_queue_indices(id, index_membership);
            if (tightened_out != nullptr) {
                *tightened_out += static_cast<std::uint64_t>(changes.size());
            }
        }
    }
    for (std::int64_t id : doomed) {
        erase_id(id);
    }
    return static_cast<int>(doomed.size());
}

/// @brief Variable-to-row index for event-driven bound propagation.
/// @details Instead of rescanning all rows after every bound change, the
/// propagator revisits only rows incident to variables whose bounds changed.
class RowPropagationIndex {
 public:
    void build(int n_vars,
               const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
               const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row);

    const std::vector<int>& ineq_rows_for(int var_idx) const {
        static const std::vector<int> kEmpty;
        if (var_idx < 0 || var_idx >= static_cast<int>(ineq_rows_by_var_.size())) return kEmpty;
        return ineq_rows_by_var_[static_cast<std::size_t>(var_idx)];
    }

    const std::vector<int>& eq_rows_for(int var_idx) const {
        static const std::vector<int> kEmpty;
        if (var_idx < 0 || var_idx >= static_cast<int>(eq_rows_by_var_.size())) return kEmpty;
        return eq_rows_by_var_[static_cast<std::size_t>(var_idx)];
    }

 private:
    std::vector<std::vector<int>> ineq_rows_by_var_;
    std::vector<std::vector<int>> eq_rows_by_var_;
};

struct PropagationProfile {
    std::uint64_t calls{0};
    std::uint64_t active_passes{0};
    std::uint64_t changed_var_visits{0};
    std::uint64_t ineq_rows_visited{0};
    std::uint64_t eq_rows_visited{0};
    std::uint64_t baseline_full_scan_rows{0};
    std::uint64_t bound_tightenings{0};
    /// Cumulative wall-time spent in propagation (nanoseconds).
    std::uint64_t wall_ns{0};
};

void reset_propagation_profile();
PropagationProfile get_propagation_profile();

/// @brief Derive binary-trigger implications from a learned conflict clause.
/// @details For a 2-literal conflict `(l_1 \land l_2)` infeasible, every
/// binary literal among `l_1,l_2` can trigger the negation of the other bound.
/// If both literals are binary, both directed implications are inserted.
bool add_binary_implications_from_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const std::vector<BranchDomainLiteral>& clause,
    BinaryImplicationGraph& implication_graph);

/// @brief Unified fixed-point node propagation engine.
/// @details Replays learned conflict clauses, then alternates clique-table
/// implications, cached binary implications, and one round of row-based bound
/// tightening until no new bound change is produced. This is the in-tree
/// domain propagation hot path used before every node LP solve.
bool propagate_node_domain(
        const LPModel& lp,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
        const RowPropagationIndex& row_index,
        Eigen::VectorXd& node_lb,
        Eigen::VectorXd& node_ub,
        int max_rounds,
        const ConflictPool* conflict_pool,
        const CliqueTable* clique_table,
        const BinaryImplicationGraph* implication_graph,
        std::vector<BoundChangeInfo>& changes_out,
        const std::vector<BranchDomainLiteral>& branch_reasons = {},
        std::vector<BranchDomainLiteral>* learned_conflict = nullptr,
        int* total_tightened = nullptr,
        std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
        const std::vector<DomainReasonBound>* existing_reason_bounds = nullptr);

bool propagate_node_domain(
        const LPModel& lp,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
        const RowPropagationIndex& row_index,
        Eigen::VectorXd& node_lb,
        Eigen::VectorXd& node_ub,
        int max_rounds,
        const SharedConflictPool* conflict_pool,
        const CliqueTable* clique_table,
        const BinaryImplicationGraph* implication_graph,
        std::vector<BoundChangeInfo>& changes_out,
        const std::vector<BranchDomainLiteral>& branch_reasons = {},
        std::vector<BranchDomainLiteral>* learned_conflict = nullptr,
        int* total_tightened = nullptr,
        std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
        const std::vector<DomainReasonBound>* existing_reason_bounds = nullptr);

}  // namespace mipsolvers::engine::detail

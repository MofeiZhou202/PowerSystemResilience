/// @file bc_parallel.cpp
/// @brief Parallel B&C thread functions: explorer_thread and cut_worker_thread.
///
/// Extracted from branch_and_cut.cpp for modularity. Contains the
/// heterogeneous parallel explorer (Diver/Prover/IPMDiver) and the
/// dedicated cut worker that generates GMI cuts from submitted LP results.

#include "mipsolvers/engine/detail/bc_parallel.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_pools.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"

namespace mipsolvers::engine {
using namespace detail;

namespace {
double gap_optimality_prune_tol_parallel(double limit, double lp_tol) {
  return std::max({1e-7, 10.0 * std::max(0.0, lp_tol),
                   1e-9 * std::max(1.0, std::abs(limit))});
}

void atomic_add_relaxed(std::atomic<double>& target, double value) {
  double current = target.load(std::memory_order_relaxed);
  while (!target.compare_exchange_weak(current, current + value,
                                       std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
  }
}
}  // namespace

void detail::explorer_thread(
    std::shared_ptr<const BcEnvOptions> environment,
    int thread_id,
    WorkerRole role,
    ThreadSafeNodeQueue& node_queue,
    CutRequestQueue& cut_queue,
    SharedCutPool& shared_cp,
    SharedConflictPool& shared_conflicts,
    SharedSolutionPool& shared_sp,
    SharedIncumbent& shared_inc,
	    const LPModel& base_lp,
	    const LPModel& pre_cut_lp,
	    const StandardFormLP& base_sf,
	    const std::vector<char>& branchable_cols,
	    const std::vector<int>& branch_priority,
	    const std::vector<int>& branch_original_cols,
	    int original_col_count,
	    const BCBranchingPriorFn* dynamic_branching_prior,
	    const CliqueTable& clique_table,
    const BinaryImplicationGraph& implication_graph,
    const SimplexOptions& simplex_opt,
    const BCOptions& opt,
    std::vector<PseudoCost>& pc,
    std::mutex& pc_mtx,
    AtomicBCStats& stats,
    std::atomic<bool>& should_stop,
    ParallelProgressEvent& progress_event,
    std::atomic<int>& active_explorers,
    ActiveNodeBounds& active_bounds,
    FallbackLogger& fallback_logger,
    std::shared_ptr<const SimplexBasis> root_basis,
    WorkStealingPool* steal_pool,
    SharedImplicationGraph* shared_impl_graph,
    DeterministicTurnToken* det_token,
    const std::atomic<double>* optimality_limit) {
  ScopedBcEnvOptions environment_scope(std::move(environment));

	  const int n = static_cast<int>(base_lp.vars.size());
	  const bool strict_highs_lp_contract =
	      uses_highs_lp_kernel(opt.lp_kernel_backend);
	  auto is_branchable_col = [&](int j) {
	    return j >= 0 && j < n &&
	           j < static_cast<int>(branchable_cols.size()) &&
	           branchable_cols[static_cast<std::size_t>(j)] != 0;
	  };
	  std::vector<int> branchable_indices;
	  branchable_indices.reserve(branchable_cols.size());
	  for (int j = 0; j < n && j < static_cast<int>(branchable_cols.size()); ++j) {
	    if (branchable_cols[static_cast<std::size_t>(j)] != 0) {
	      branchable_indices.push_back(j);
	    }
	  }
	  auto fractional_branchable_indices =
	      [&](const Eigen::VectorXd& x, const Eigen::VectorXd& lb,
	          const Eigen::VectorXd& ub, double int_tol,
	          std::vector<int>& out) -> bool {
	    out.clear();
	    const int limit =
	        std::min(n, static_cast<int>(std::min<std::size_t>(
	                        branchable_cols.size(),
	                        static_cast<std::size_t>(std::min(
	                            static_cast<int>(x.size()),
	                            std::min(static_cast<int>(lb.size()),
	                                     static_cast<int>(ub.size())))))));
	    const double split_tol = std::max(1e-9, int_tol);
	    for (int j : branchable_indices) {
	      if (j >= limit) break;
	      if (ub[j] - lb[j] <= split_tol) continue;
	      const double v = std::min(ub[j], std::max(lb[j], x[j]));
	      if (is_integral(v, int_tol)) continue;
	      const double down_ub = std::min(ub[j], std::floor(v));
	      const double up_lb = std::max(lb[j], std::ceil(v));
	      if (down_ub < lb[j] - split_tol || up_lb > ub[j] + split_tol) {
	        continue;
	      }
	      out.push_back(j);
	    }
	    return !out.empty();
	  };
  Eigen::VectorXd root_lb(n);
  Eigen::VectorXd root_ub(n);
  for (int j = 0; j < n; ++j) {
    root_lb[j] = base_lp.vars[static_cast<std::size_t>(j)].lb;
    root_ub[j] = base_lp.vars[static_cast<std::size_t>(j)].ub;
  }
  const int par_base_m = static_cast<int>(base_lp.A.rows()) +
                         static_cast<int>(base_lp.Aeq.rows());
  const int par_probe_reliability = opt.probe_reliability;
  const int par_probe_max = (par_base_m > 2000) ? std::max(1, opt.probe_max_candidates - 1) : opt.probe_max_candidates;
  const bool is_ipm_diver = (role == WorkerRole::IPMDiver);
  // Master switch for cross-thread conflict sharing. When false, we still
  // use shared_cp for root cuts / shared_inc for primal / shared_sp for
  // guided rounding, but conflict clauses, binary implications, and
  // conflict cuts are NOT propagated across threads — this eliminates the
  // sole correctness-hazard feedback loop that could make a multi-thread
  // run prune regions the sequential / Gurobi runs still visit.
  const bool use_shared_conflicts = opt.share_conflict_learning_across_threads;
  auto current_optimality_limit = [&]() -> double {
    return optimality_limit != nullptr
               ? optimality_limit->load(std::memory_order_acquire)
               : kInf;
  };
  auto node_exceeds_optimality_limit = [&](double bound) -> bool {
    if (opt.require_tree_exhaustion_certificate || !std::isfinite(bound)) {
      return false;
    }
    const double limit = current_optimality_limit();
    if (!std::isfinite(limit)) return false;
    return bound >= limit - gap_optimality_prune_tol_parallel(limit, opt.lp_tol);
  };

  struct ActiveNodeGuard {
    ActiveNodeBounds& bounds;
    std::atomic<int>& active;
    ParallelProgressEvent& progress;
    int worker;
    bool armed{false};
    ActiveNodeGuard(ActiveNodeBounds& b, std::atomic<int>& a,
                    ParallelProgressEvent& p, int w, double bound)
        : bounds(b), active(a), progress(p), worker(w), armed(true) {
      bounds.enter(worker, bound);
      active.fetch_add(1, std::memory_order_acq_rel);
      progress.notify();
    }
    ~ActiveNodeGuard() {
      if (armed) {
        bounds.leave(worker);
        active.fetch_sub(1, std::memory_order_acq_rel);
        progress.notify();
      }
    }
    ActiveNodeGuard(const ActiveNodeGuard&) = delete;
    ActiveNodeGuard& operator=(const ActiveNodeGuard&) = delete;
  };

  // IPM solver for IPMDiver role (fast ~30ms node LPs without cold-start).
  std::unique_ptr<NativeIPMLPAdapter> ipm_solver;
  if (is_ipm_diver) {
    IPMLPOptions ipm_opt;
    ipm_opt.max_iter = opt.ipm_node_max_iter;
    ipm_opt.tol_primal = std::max(1e-9, opt.lp_tol * opt.ipm_node_tol_multiplier);
    ipm_opt.tol_dual = std::max(1e-9, opt.lp_tol * opt.ipm_node_tol_multiplier);
    ipm_opt.tol_gap = std::max(1e-9, opt.lp_tol * opt.ipm_node_tol_multiplier);
    ipm_opt.verbose = false;
    ipm_solver = std::make_unique<NativeIPMLPAdapter>(ipm_opt);
    ipm_solver->prepare_for_node_solves(base_lp);
  }

  std::unique_ptr<NativeIPMLPAdapter> ipm_infeas_verifier;
  if (!is_ipm_diver) {
    IPMLPOptions ipm_opt;
    ipm_opt.max_iter = opt.ipm_node_max_iter;
    ipm_opt.tol_primal = 1e-6;
    ipm_opt.tol_dual = 1e-6;
    ipm_opt.tol_gap = 1e-6;
    ipm_opt.verbose = false;
    ipm_infeas_verifier = std::make_unique<NativeIPMLPAdapter>(ipm_opt);
    ipm_infeas_verifier->prepare_for_node_solves(base_lp);
  }

  // StandardFormLP only needed for simplex roles.
  StandardFormLP local_sf;
  if (!is_ipm_diver) {
    local_sf = base_sf;  // Per-thread copy
  }

  // Pre-compute row-major sparse matrices (per-thread, avoids repeated conversion).
  const Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_local = base_lp.A;
  const Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row_local = base_lp.Aeq;
  RowPropagationIndex row_index_local;
  row_index_local.build(n, A_row_local, Aeq_row_local);
  BinaryImplicationGraph local_implication_graph = implication_graph;

  // ── Cross-thread implication sharing (2026-Q2 improvement) ──
  // Thread-local publish buffer for batched flushing to the shared graph,
  // and a last-generation marker for the subscribe fast-path.
  const bool share_impl = (shared_impl_graph != nullptr) &&
                           opt.share_conflict_learning_across_threads &&
                           opt.share_implications_across_threads;
  std::vector<SharedImplicationGraph::Arc> publish_buffer;
  std::uint64_t last_impl_gen_seen =
      share_impl ? shared_impl_graph->generation() : 0;

  auto flush_publish_buffer = [&]() {
    if (!share_impl || publish_buffer.empty()) return;
    const std::size_t added = shared_impl_graph->publish(publish_buffer);
    if (added > 0) {
      stats.shared_implications_published.fetch_add(
          static_cast<std::uint64_t>(added), std::memory_order_relaxed);
    }
    publish_buffer.clear();
  };

  auto queue_publish_local_implication =
      [&](int trigger_var, bool trigger_value_one, int implied_var,
          bool implied_is_lb, double implied_value) {
    if (!share_impl) return;
    publish_buffer.push_back({trigger_var, trigger_value_one, implied_var,
                              implied_is_lb, implied_value});
    if (static_cast<int>(publish_buffer.size()) >=
        std::max(1, opt.implication_publish_batch_size)) {
      flush_publish_buffer();
    }
  };

  auto pull_shared_implications_if_any = [&]() {
    if (!share_impl) return;
    const std::uint64_t cur_gen = shared_impl_graph->generation();
    if (cur_gen == last_impl_gen_seen) return;  // lock-free fast path
    std::vector<SharedImplicationGraph::Arc> new_arcs;
    new_arcs.reserve(16);
    last_impl_gen_seen =
        shared_impl_graph->pull_since(last_impl_gen_seen, new_arcs);
    std::uint64_t consumed = 0;
    for (const auto& a : new_arcs) {
      if (local_implication_graph.add_implication(a.trigger_var,
                                                   a.trigger_value_one,
                                                   a.implied_var,
                                                   a.implied_is_lb,
                                                   a.implied_value)) {
        ++consumed;
      }
    }
    if (consumed > 0) {
      stats.shared_implications_consumed.fetch_add(consumed,
                                                    std::memory_order_relaxed);
    }
    stats.shared_implication_pulls.fetch_add(1, std::memory_order_relaxed);
  };

  // Per-thread FallbackManager for LP recovery.
  FallbackConfig fb_config;
  fb_config.l1_max_retries = opt.fallback_l1_retries;
  fb_config.l1_perturbation = opt.fallback_l1_perturbation;
  fb_config.l3_max_retries = opt.fallback_l3_retries;
  fb_config.l3_tol_multiplier = opt.fallback_l3_tol_mult;
  FallbackManager fallback_mgr(fb_config, fallback_logger, simplex_opt);

  // Per-thread SolverDispatcher routes LP solves through unified path.
  DispatcherConfig par_disp_config = DispatcherConfig::from_bc_options(opt);
  par_disp_config.allow_persistent_lp_state = true;
  SolverDispatcher par_dispatcher(par_disp_config, &fallback_mgr, &shared_inc.obj);
  if (root_basis) {
    par_dispatcher.set_root_basis(root_basis);
  }
  // Per-explorer node_id counter. Previously a function-local `static`,
  // which unintentionally persisted across solves in the same process and
  // polluted fallback-logging node IDs. The value is only used for fallback
  // diagnostics, so a fresh per-thread counter is sufficient.
  int node_id_gen = 0;
  // P7.3: collect per-child pseudo-cost updates and merge once per expanded
  // parent node (single lock acquisition) instead of locking inside each
  // process_child path.
  std::vector<PCUpdate> pc_updates_buffer;
  pc_updates_buffer.reserve(8);
  auto record_node_explored = [&] {
    stats.nodes_explored.fetch_add(1, std::memory_order_relaxed);
    progress_event.notify();
  };

  // ── Helper lambda: process a single child node ──
  // branch_var is passed explicitly so the lambda doesn't capture a changing 'j'.
  auto process_child = [&](Node& child, bool is_up, int branch_var,
                            double parent_bound,
                            const Eigen::VectorXd& parent_x_relax) -> bool {
    if (!bounds_consistent(child.lb, child.ub)) return false;

    int inference_count = 0;
    auto apply_branch_history = [&](bool has_gain, double gain,
                                    int cutoff_count,
                                    double conflict_score = 0.0) {
      PCUpdate upd;
      upd.var = branch_var;
      upd.is_up = is_up;
      upd.has_gain = has_gain;
      upd.gain = gain;
      upd.has_inference = true;
      upd.inference_count = static_cast<double>(inference_count);
      upd.cutoff_count = cutoff_count;
      upd.conflict_score = conflict_score;
      pc_updates_buffer.push_back(std::move(upd));
    };
    std::vector<BoundChangeInfo> prop_changes;
    std::vector<BranchDomainLiteral> learned_conflict;
    prop_changes.reserve(32);
    if (opt.enable_reduced_cost_proof_conflict_minimization) {
      child.domain_reason_bounds.reserve(child.domain_reason_bounds.size() + 32);
    }
    auto publish_binary_clause = [&](const std::vector<BranchDomainLiteral>& clause) {
      if (!share_impl || clause.size() != 2) return;
      auto decode = [&](const BranchDomainLiteral& lit,
                        int& vid, bool& value_one,
                        bool& neg_is_lb, double& neg_val) -> bool {
        vid = lit.var_idx;
        if (vid < 0 || vid >= static_cast<int>(base_lp.vars.size())) return false;
        if (base_lp.vars[static_cast<std::size_t>(vid)].type != VarType::Binary) {
          return false;
        }
        if (lit.is_lb && lit.value >= 1.0 - 1e-9) {
          value_one = true; neg_is_lb = false; neg_val = 0.0;
          return true;
        }
        if (!lit.is_lb && lit.value <= 1e-9) {
          value_one = false; neg_is_lb = true; neg_val = 1.0;
          return true;
        }
        return false;
      };
      int v0 = -1, v1 = -1;
      bool val0 = false, val1 = false;
      bool neg_lb0 = false, neg_lb1 = false;
      double neg_val0 = 0.0, neg_val1 = 0.0;
      if (decode(clause[0], v0, val0, neg_lb0, neg_val0) &&
          decode(clause[1], v1, val1, neg_lb1, neg_val1)) {
        queue_publish_local_implication(v0, val0, v1, neg_lb1, neg_val1);
        queue_publish_local_implication(v1, val1, v0, neg_lb0, neg_val0);
      }
    };
    auto learn_short_reconvergence_implications =
        [&](const std::vector<BranchDomainLiteral>& branch_reasons,
            const std::vector<DomainReasonBound>& reason_bounds) -> int {
	      if (!opt.enable_reduced_cost_proof_conflict_minimization ||
	          reason_bounds.empty()) {
	        return 0;
	      }
	      if (strict_highs_lp_contract) {
	        return 0;
	      }
      const int max_cut_literals =
          std::max(2, opt.reduced_cost_conflict_cut_max_literals);
      const int max_literals = std::min(32, max_cut_literals);
      const int max_frontier_clauses =
          std::min(4, std::max(1, opt.reduced_cost_conflict_max_per_node));
      struct FrontierCandidate {
        std::vector<BranchDomainLiteral> clause;
        double score{0.0};
        int queue_hits{0};
      };
      std::vector<FrontierCandidate> candidates;
      candidates.reserve(static_cast<std::size_t>(
          std::min<int>(reason_bounds.size(), max_frontier_clauses * 4)));
      BranchLiteralListSet seen;

      auto literal_pos = [&](const BranchDomainLiteral& lit) -> int {
        int best_pos = -1;
        for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
          if (branch_literal_stronger_or_equal(branch_reasons[static_cast<std::size_t>(i)],
                                               lit)) {
            best_pos = std::max(best_pos, i);
          }
        }
        for (const auto& rb : reason_bounds) {
          if (rb.trail_pos < 0) continue;
          if (branch_literal_stronger_or_equal(rb.bound, lit)) {
            best_pos = std::max(best_pos, rb.trail_pos);
          }
        }
        return best_pos;
      };

      auto find_reason_for_literal =
          [&](const BranchDomainLiteral& lit,
              int before_pos) -> const DomainReasonBound* {
        const DomainReasonBound* best = nullptr;
        for (const auto& rb : reason_bounds) {
          if (rb.trail_pos < 0 || rb.trail_pos >= before_pos) continue;
          if (!branch_literal_stronger_or_equal(rb.bound, lit)) continue;
          if (best == nullptr || rb.trail_pos > best->trail_pos) {
            best = &rb;
          }
        }
        return best;
      };

      auto uip_minimize_frontier =
          [&](std::vector<BranchDomainLiteral> frontier,
              int target_pos) {
        canonicalize_branch_literals(frontier);
        BranchLiteralListSet visited_frontiers;
        for (int iter = 0; iter < 16; ++iter) {
          if (!visited_frontiers.insert(frontier)) break;
          int best_idx = -1;
          int best_pos = -1;
          int resolvable_at_depth = 0;
          const int branch_depth_begin = static_cast<int>(branch_reasons.size());
          for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
            const int pos = literal_pos(frontier[static_cast<std::size_t>(i)]);
            if (pos < branch_depth_begin || pos >= target_pos) continue;
            const DomainReasonBound* reason =
                find_reason_for_literal(frontier[static_cast<std::size_t>(i)],
                                        target_pos);
            if (reason == nullptr || reason->reason.empty()) continue;
            ++resolvable_at_depth;
            if (pos > best_pos) {
              best_pos = pos;
              best_idx = i;
            }
          }
          if (best_idx < 0 || resolvable_at_depth <= 1) break;
          const DomainReasonBound* reason =
              find_reason_for_literal(frontier[static_cast<std::size_t>(best_idx)],
                                      target_pos);
          if (reason == nullptr) break;
          frontier.erase(frontier.begin() + best_idx);
          frontier.insert(frontier.end(), reason->reason.begin(), reason->reason.end());
          canonicalize_branch_literals(frontier);
          if (frontier.empty() ||
              static_cast<int>(frontier.size()) > max_literals) {
            break;
          }
        }
        return frontier;
      };

      auto flip_bound = [&](const BranchDomainLiteral& lit,
                            BranchDomainLiteral& out) -> bool {
        if (lit.var_idx < 0 || lit.var_idx >= n ||
            !std::isfinite(lit.value)) {
          return false;
        }
        const bool integer_var =
            is_integer_type(base_lp.vars[static_cast<std::size_t>(lit.var_idx)]);
        if (lit.is_lb) {
          out = BranchDomainLiteral{
              lit.var_idx,
              integer_var ? std::ceil(lit.value - 1e-9) - 1.0
                          : lit.value - 1e-7,
              false};
        } else {
          out = BranchDomainLiteral{
              lit.var_idx,
              integer_var ? std::floor(lit.value + 1e-9) + 1.0
                          : lit.value + 1e-7,
              true};
        }
        return true;
      };

      for (const auto& rb : reason_bounds) {
        if (rb.reason.empty()) continue;
        BranchDomainLiteral flipped;
        if (rb.has_source_conflict_literal) {
          flipped = rb.source_conflict_literal;
        } else if (!flip_bound(rb.bound, flipped)) {
          continue;
        }
        std::vector<BranchDomainLiteral> clause = uip_minimize_frontier(
            rb.reason,
            rb.trail_pos >= 0
                ? rb.trail_pos
                : static_cast<int>(branch_reasons.size() + reason_bounds.size()));
        clause.push_back(flipped);
        canonicalize_branch_literals(clause);
        if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
          continue;
        }
        if (!seen.insert(clause)) continue;
        const int short_limit = std::min(8, max_literals);
        const int queue_hits =
            static_cast<int>(clause.size()) <= short_limit
                ? node_queue.count_clause_hits(clause, true, 8)
                : 0;
        if (queue_hits <= 0) continue;
        FrontierCandidate cand;
        cand.score = 1000.0 * static_cast<double>(queue_hits) -
                     static_cast<double>(clause.size());
        cand.queue_hits = queue_hits;
        cand.clause = std::move(clause);
        candidates.push_back(std::move(cand));
      }

      if (candidates.empty()) return 0;
      std::sort(candidates.begin(), candidates.end(),
                [](const auto& a, const auto& b) {
                  if (a.score != b.score) return a.score > b.score;
                  if (a.clause.size() != b.clause.size()) {
                    return a.clause.size() < b.clause.size();
                  }
                  return conflict_clause_hash(a.clause) <
                         conflict_clause_hash(b.clause);
                });

      int learned = 0;
      for (const auto& candidate : candidates) {
        if (learned >= max_frontier_clauses) break;
        const auto& clause = candidate.clause;
        bool clause_added = false;
        if (use_shared_conflicts) {
          clause_added = shared_conflicts.add(clause);
        }
        const bool impl_added =
            add_binary_implications_from_conflict_clause(
                base_lp.vars, clause, local_implication_graph);
        if (impl_added) {
          stats.rc_binary_implications_learned.fetch_add(
              1, std::memory_order_relaxed);
        }
        publish_binary_clause(clause);
        clause_added |= impl_added;
        if (clause_added && static_cast<int>(clause.size()) <= max_cut_literals) {
          PoolCut conflict_cut;
          if (use_shared_conflicts &&
              try_build_binary_conflict_cut(base_lp, clause, conflict_cut,
                                            max_cut_literals) &&
              shared_cp.add(std::move(conflict_cut.coeff), conflict_cut.rhs)) {
            stats.cuts_added.fetch_add(1, std::memory_order_relaxed);
            stats.rc_conflict_cuts_added.fetch_add(1, std::memory_order_relaxed);
          }
        }
        if (!clause_added) continue;
        stats.rc_conflict_clauses_learned.fetch_add(
            1, std::memory_order_relaxed);
        stats.rc_proof_conflict_clauses_learned.fetch_add(
            1, std::memory_order_relaxed);
        stats.rc_proof_conflict_literals_before.fetch_add(
            static_cast<std::uint64_t>(clause.size()),
            std::memory_order_relaxed);
        stats.rc_proof_conflict_literals_after.fetch_add(
            static_cast<std::uint64_t>(clause.size()),
            std::memory_order_relaxed);
        ++learned;
      }
      return learned;
    };
    if (!propagate_node_domain(base_lp, A_row_local, Aeq_row_local,
                               row_index_local,
                               child.lb, child.ub,
                               opt.bound_propagation_rounds,
                               use_shared_conflicts ? &shared_conflicts : nullptr,
                               &clique_table, &local_implication_graph,
                               prop_changes,
                               child.branch_reasons,
                               &learned_conflict,
                               &inference_count,
                               opt.enable_reduced_cost_proof_conflict_minimization
                                   ? &child.domain_reason_bounds
                                   : nullptr)) {
      if (learned_conflict.empty()) learned_conflict = child.branch_reasons;
      if (!learned_conflict.empty()) {
        if (use_shared_conflicts) {
          shared_conflicts.add(learned_conflict);
        }
        add_binary_implications_from_conflict_clause(base_lp.vars,
                                                     learned_conflict,
                                                     local_implication_graph);
        publish_binary_clause(learned_conflict);
        PoolCut conflict_cut;
        if (use_shared_conflicts &&
            try_build_binary_conflict_cut(base_lp, learned_conflict,
                                          conflict_cut) &&
            shared_cp.add(std::move(conflict_cut.coeff), conflict_cut.rhs)) {
          stats.cuts_added.fetch_add(1, std::memory_order_relaxed);
        }
      }
      apply_branch_history(false, 0.0, 1, learned_conflict.empty() ? 0.0 : 1.0);
      return false;
    }
    const int short_reconv_learned =
        learn_short_reconvergence_implications(child.branch_reasons,
                                               child.domain_reason_bounds);
    if (short_reconv_learned > 0) {
      inference_count += short_reconv_learned;
    }

    SimplexResult simplex_res;  // Only valid for simplex path.

    if (is_ipm_diver) {
      // IPM path: solve on pre-cut LP (IPM Cholesky fails on cut-augmented LPs).
      auto ipm_result = ipm_solver->solve_node_lp(
          pre_cut_lp, child.lb, child.ub, child.x_seed);
      stats.lp_solves.fetch_add(1, std::memory_order_relaxed);
      if (!ipm_result.stats.success ||
          ipm_result.x.size() != n) {
        // IPM failed — defer node to simplex threads via global queue.
        // Inherit parent's x_relax so receiving thread can branch.
        child.x_relax = parent_x_relax;
        child.x_seed = parent_x_relax;
        node_queue.push_pq(child);
        return false;
      }
      child.x_relax = clamp_to_bounds(ipm_result.x, child.lb, child.ub);
      child.x_seed = child.x_relax;
      child.bound = std::max(parent_bound, ipm_result.stats.objective);
      // No simplex basis — child inherits parent's basis_hint.
    } else {
      // Simplex path: warm-start from parent basis.
      update_standard_form_bounds(local_sf, base_lp, child.lb, child.ub);

      bool deferred = false;
      const int nid = node_id_gen++;
      simplex_res = par_dispatcher.solve(
          local_sf, child.basis_hint.get(), SolveContext::NodeLP,
          nid, child.depth, n, deferred);
      stats.lp_solves.fetch_add(1, std::memory_order_relaxed);

        bool rescued_by_ipm = false;
      const auto failure_type = classify_lp_result(simplex_res, n);
      if (failure_type != LPFailureType::Success) {
        if (failure_type == LPFailureType::ObjectiveCutoff) {
          child.bound = std::max(parent_bound, simplex_res.result.stats.objective);
          const double gain = std::max(0.0, child.bound - parent_bound);
          apply_branch_history(true, gain, 1);
          return false;
        }
        const bool has_farkas = simplex_res.result.stats.has_farkas_certificate;
        if (failure_type == LPFailureType::Infeasible && has_farkas &&
            ipm_infeas_verifier) {
          auto ipm_res = ipm_infeas_verifier->solve_node_lp(
              base_lp, child.lb, child.ub, child.x_seed);
          stats.lp_solves.fetch_add(1, std::memory_order_relaxed);
          if (ipm_res.stats.success && ipm_res.x.size() == n) {
            child.x_relax = clamp_to_bounds(ipm_res.x, child.lb, child.ub);
            child.x_seed = child.x_relax;
            child.bound = std::max(parent_bound, ipm_res.stats.objective);
            rescued_by_ipm = true;
          }
        }
        if (!rescued_by_ipm && failure_type == LPFailureType::Infeasible) {
          fallback_logger.log(nid, child.depth,
                              LPFailureType::Infeasible, 5, "pruned");
          if (has_farkas) {
            if (learned_conflict.empty()) learned_conflict = child.branch_reasons;
            if (use_shared_conflicts) {
              shared_conflicts.add(learned_conflict);
            }
            add_binary_implications_from_conflict_clause(base_lp.vars,
                                                         learned_conflict,
                                                         local_implication_graph);
            // Publish the two directed arcs of a 2-literal binary conflict
            // to the shared implication graph for sibling explorers.
            publish_binary_clause(learned_conflict);
            PoolCut conflict_cut;
            if (use_shared_conflicts &&
                try_build_binary_conflict_cut(base_lp, learned_conflict,
                                              conflict_cut) &&
                shared_cp.add(std::move(conflict_cut.coeff), conflict_cut.rhs)) {
              stats.cuts_added.fetch_add(1, std::memory_order_relaxed);
            }
          }
          apply_branch_history(false, 0.0, 1, has_farkas ? 1.0 : 0.0);
        }
        if (!rescued_by_ipm && deferred) {
          // Inherit parent's x_relax so receiving thread can branch.
          child.x_relax = parent_x_relax;
          child.x_seed = parent_x_relax;
          node_queue.push_pq(child);
        }
        if (!rescued_by_ipm) {
          return false;
        }
      }

      if (!rescued_by_ipm) {
        child.x_relax = clamp_to_bounds(simplex_res.result.x, child.lb, child.ub);
        child.x_seed = child.x_relax;
        auto child_basis = std::make_shared<SimplexBasis>(std::move(simplex_res.basis));
        if (child.basis_hint) {
          child_basis->try_share_indices_from(*child.basis_hint);
        }
        child_basis->compact_indices_storage();
        child_basis->cached_reduced_costs =
            std::make_shared<const Eigen::VectorXd>(std::move(simplex_res.reduced_costs));
        if (simplex_res.form.col_scale.size() == simplex_res.form.A.cols()) {
          child_basis->cached_col_scale =
              std::make_shared<const Eigen::VectorXd>(simplex_res.form.col_scale);
        }
        child.basis_hint = child_basis;
        child.bound = std::max(parent_bound, simplex_res.result.stats.objective);
      }
    }

    // N-PAR3 fix: take a fresh atomic snapshot here rather than a dual read.
    // shared_inc.snapshot() reads obj then has in the same acquire sequence
    // (mirroring the reversed write order), giving a consistent pair without
    // holding any lock.
    SharedIncumbent::Snapshot snap0 = shared_inc.snapshot();
    bool   has_incumbent = snap0.has;  // mutable: refreshed after pool LP re-solve
	    double inc_obj       = snap0.obj;
	    const double gain = std::max(0.0, child.bound - parent_bound);
	    if (incumbent_prunes_node(has_incumbent, inc_obj, child.bound)) {
	      apply_branch_history(true, gain, 1);
	      return false;
	    }
	    if (node_exceeds_optimality_limit(child.bound)) {
	      stats.gap_suboptimal_queue_prunes.fetch_add(
	          1, std::memory_order_relaxed);
	      apply_branch_history(true, gain, 1);
	      return false;
	    }

    // Reduced-cost variable fixing (simplex roles only — IPM has no reduced costs).
    if (!is_ipm_diver && has_incumbent && simplex_res.reduced_costs.size() >= n) {
      std::vector<BranchDomainLiteral> rc_forbidden_literals;
      Eigen::VectorXd rc_proof_lb;
      Eigen::VectorXd rc_proof_ub;
      if (opt.enable_reduced_cost_conflict_learning) {
        rc_forbidden_literals.reserve(
            static_cast<std::size_t>(std::max(0, opt.reduced_cost_conflict_max_per_node)));
        if (opt.enable_reduced_cost_proof_conflict_minimization) {
          rc_proof_lb = child.lb;
          rc_proof_ub = child.ub;
        }
      }
      const int rc_fixed = reduced_cost_fixing(base_lp.vars, child.x_relax,
                          simplex_res.reduced_costs,
                          child.basis_hint->basis_indices(), child.bound,
                          inc_obj, opt.int_tol, child.lb, child.ub,
                          opt.enable_reduced_cost_conflict_learning
                              ? &rc_forbidden_literals
                              : nullptr,
                          child.basis_hint
                              ? child.basis_hint->cached_col_scale.get()
                              : nullptr);
      if (rc_fixed > 0) {
        stats.rc_fixings.fetch_add(static_cast<std::uint64_t>(rc_fixed),
                                   std::memory_order_relaxed);
        stats.rc_forbidden_literals.fetch_add(
            static_cast<std::uint64_t>(rc_forbidden_literals.size()),
            std::memory_order_relaxed);
      }
	      if (rc_fixed > 0 && opt.enable_reduced_cost_conflict_learning &&
	          !strict_highs_lp_contract &&
	          !rc_forbidden_literals.empty()) {
	        int learned = 0;
	        const int max_learn = std::max(0, opt.reduced_cost_conflict_max_per_node);
	        const int max_cut_literals =
	            std::max(2, opt.reduced_cost_conflict_cut_max_literals);
	        const int max_pool_literals =
	            std::max(max_cut_literals,
	                     opt.reduced_cost_conflict_pool_max_literals);
	        for (const auto& forbidden : rc_forbidden_literals) {
	          if (learned >= max_learn) break;
	          if (forbidden.var_idx < 0 || forbidden.var_idx >= n) continue;
	          std::vector<BranchDomainLiteral> clause;
	          const std::uint64_t full_clause_len =
	              static_cast<std::uint64_t>(child.branch_reasons.size() + 1);
	          bool proof_clause = false;
	          std::vector<std::vector<BranchDomainLiteral>> reconvergence_clauses;
	          if (opt.enable_reduced_cost_proof_conflict_minimization) {
	            const Eigen::VectorXd& proof_lb =
	                rc_proof_lb.size() == n ? rc_proof_lb : child.lb;
	            const Eigen::VectorXd& proof_ub =
	                rc_proof_ub.size() == n ? rc_proof_ub : child.ub;
	            double proof_margin = 0.0;
	            proof_clause = build_reduced_cost_cutoff_conflict_clause(
	                base_lp.vars, root_lb, root_ub, proof_lb, proof_ub,
	                child.x_relax, simplex_res.reduced_costs,
	                child.basis_hint->basis_indices(), child.bound, inc_obj,
	                opt.int_tol, child.branch_reasons,
	                child.domain_reason_bounds, forbidden,
	                max_pool_literals, clause, &proof_margin,
	                &reconvergence_clauses);
	            (void)proof_margin;
	          }
	          if (!proof_clause) {
	            clause = child.branch_reasons;
          bool merged = false;
          for (auto& lit : clause) {
            if (lit.var_idx != forbidden.var_idx || lit.is_lb != forbidden.is_lb) continue;
            if (lit.is_lb) lit.value = std::max(lit.value, forbidden.value);
            else lit.value = std::min(lit.value, forbidden.value);
            merged = true;
	            break;
	          }
	          if (!merged) clause.push_back(forbidden);
	          }
	          canonicalize_branch_literals(clause);
	          if (clause.empty() ||
	              static_cast<int>(clause.size()) > max_pool_literals) {
	            continue;
	          }

          std::vector<std::vector<BranchDomainLiteral>> clauses_to_learn;
          clauses_to_learn.push_back(clause);
          clauses_to_learn.insert(clauses_to_learn.end(),
                                  reconvergence_clauses.begin(),
                                  reconvergence_clauses.end());
          bool added = false;
          std::uint64_t learned_after_sum = 0;
          for (const auto& clause_to_learn : clauses_to_learn) {
            bool clause_added = false;
            if (use_shared_conflicts) {
              clause_added = shared_conflicts.add(clause_to_learn);
            }
            const bool impl_added = add_binary_implications_from_conflict_clause(
                base_lp.vars, clause_to_learn, local_implication_graph);
            if (impl_added) {
              stats.rc_binary_implications_learned.fetch_add(
                  1, std::memory_order_relaxed);
            }
            clause_added |= impl_added;
            publish_binary_clause(clause_to_learn);
            if (clause_added &&
                static_cast<int>(clause_to_learn.size()) <= max_cut_literals) {
              PoolCut conflict_cut;
              if (use_shared_conflicts &&
                  try_build_binary_conflict_cut(base_lp, clause_to_learn,
                                                conflict_cut,
                                                max_cut_literals) &&
                  (static_cast<int>(clause_to_learn.size()) <= 2 ||
                   sparse_dot(conflict_cut.coeff, child.x_relax) >
                       conflict_cut.rhs + 1e-7) &&
                  shared_cp.add(std::move(conflict_cut.coeff), conflict_cut.rhs)) {
              stats.cuts_added.fetch_add(1, std::memory_order_relaxed);
              stats.rc_conflict_cuts_added.fetch_add(1, std::memory_order_relaxed);
            }
          }
            if (clause_added) {
              added = true;
              learned_after_sum +=
                  static_cast<std::uint64_t>(clause_to_learn.size());
              stats.rc_conflict_clauses_learned.fetch_add(
                  1, std::memory_order_relaxed);
            }
          }
          if (added) {
            if (proof_clause) {
              stats.rc_proof_conflict_clauses_learned.fetch_add(
                  static_cast<std::uint64_t>(clauses_to_learn.size()),
                  std::memory_order_relaxed);
              stats.rc_proof_conflict_literals_before.fetch_add(
                  full_clause_len * static_cast<std::uint64_t>(clauses_to_learn.size()),
                  std::memory_order_relaxed);
              stats.rc_proof_conflict_literals_after.fetch_add(
                  learned_after_sum, std::memory_order_relaxed);
            }
            ++learned;
            inference_count += 1;
          }
        }
      }
      if (rc_fixed > 0 && opt.rc_fixing_followup_propagation) {
        std::vector<BoundChangeInfo> rc_prop_changes;
        std::vector<BranchDomainLiteral> rc_conflict;
        int rc_tightenings = 0;
        if (!propagate_node_domain(base_lp, A_row_local, Aeq_row_local,
                                   row_index_local,
                                   child.lb, child.ub,
                                   opt.bound_propagation_rounds,
                                   use_shared_conflicts ? &shared_conflicts : nullptr,
                                   &clique_table, &local_implication_graph,
                                   rc_prop_changes,
                                   child.branch_reasons,
                                   &rc_conflict,
                                   &rc_tightenings)) {
          apply_branch_history(true, gain, 1, 1.0);
          return false;
        }
      }
      if (!bounds_consistent(child.lb, child.ub)) {
        apply_branch_history(true, gain, 1);
        return false;
      }
    }

    // Pool cut separation (simplex roles only — IPM Cholesky can fail on augmented LPs).
    // P7.4: gate by pool_cut_scan_interval to halve violation-scan FLOPs on large LPs.
    const int scan_interval = std::max(1, opt.pool_cut_scan_interval);
    const bool do_pool_scan = (node_id_gen % scan_interval == 0);
    if (!is_ipm_diver && do_pool_scan
        && static_cast<int>(base_lp.A.rows()) <= opt.pool_cut_row_threshold) {
      auto violated_cuts = shared_cp.select_violated_and_age(
          child.x_relax, 1e-4, opt.cuts_per_round);
      if (!violated_cuts.empty()) {
        const int pool_budget = static_cast<int>(violated_cuts.size());
        LPModel pool_lp = base_lp;
        apply_node_bounds(pool_lp.vars, child.lb, child.ub);
        std::vector<Eigen::SparseVector<double>> pool_rows;
        std::vector<double> pool_rhs_vec;
        pool_rows.reserve(pool_budget);
        pool_rhs_vec.reserve(pool_budget);
        for (int pi = 0; pi < pool_budget; ++pi) {
          PoolCut pc_cut = std::move(violated_cuts[static_cast<std::size_t>(pi)]);
          pool_rows.push_back(std::move(pc_cut.coeff));
          pool_rhs_vec.push_back(pc_cut.rhs);
        }
        add_sparse_rows_to_lp(pool_lp, pool_rows, pool_rhs_vec);
        stats.pool_cuts_separated.fetch_add(pool_budget, std::memory_order_relaxed);

        StandardFormLP pool_sf = build_standard_form_lp(pool_lp);
        ruiz_scale_standard_form(pool_sf);
        auto pool_res = par_dispatcher.solve_no_fallback(pool_sf,
                                                         child.basis_hint.get(),
                                                         SolveContext::PoolCuts);
        stats.lp_solves.fetch_add(1, std::memory_order_relaxed);

        if (pool_res.result.stats.success && pool_res.result.x.size() == n) {
          child.x_relax = clamp_to_bounds(pool_res.result.x, child.lb, child.ub);
          child.x_seed = child.x_relax;
          child.bound = std::max(child.bound, pool_res.result.stats.objective);
          // Refresh snapshot after the pool LP re-solve; use atomic snapshot
          // to avoid the dual-read race (N-PAR3).
          { auto s = shared_inc.snapshot(); has_incumbent = s.has; inc_obj = s.obj; }
	          if (incumbent_prunes_node(has_incumbent, inc_obj, child.bound)) {
	            apply_branch_history(true, gain, 1);
	            return false;
	          }
	          if (node_exceeds_optimality_limit(child.bound)) {
	            stats.gap_suboptimal_queue_prunes.fetch_add(
	                1, std::memory_order_relaxed);
	            apply_branch_history(true, gain, 1);
	            return false;
	          }
	        }
	      }
	    }

    // Rounding heuristic — always attempt (satisfies_with_bounds is cheap).
    {
      const Eigen::VectorXd xr = project_integer_solution(base_lp.vars, child.x_relax,
                                                          child.lb, child.ub);
      if (satisfies_with_bounds(base_lp, xr, child.lb, child.ub, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, xr, base_lp.sense);
        shared_inc.try_update(xr, obj);
        shared_sp.add(xr, obj);
      }
    }

    // Guided rounding from solution pool.
    if (shared_sp.size() >= 3) {
      Eigen::VectorXd xg = shared_sp.guided_rounding(child.x_relax, base_lp.vars);
      xg = clamp_to_bounds(xg, child.lb, child.ub);
      if (satisfies_with_bounds(base_lp, xg, child.lb, child.ub, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, xg, base_lp.sense);
        shared_inc.try_update(xg, obj);
        shared_sp.add(xg, obj);
      }
    }

    // Submit to cut worker (simplex roles only — IPM has no basis for Gomory cuts).
    if (!is_ipm_diver && opt.cuts != CutType::None && child.depth <= opt.max_cut_depth) {
      CutRequest req;
      req.simplex_result = simplex_res;
      req.simplex_result.form = local_sf;
      req.x_relax = child.x_relax;
      req.node_lb = child.lb;
      req.node_ub = child.ub;
      req.node_depth = child.depth;
      if (cut_queue.try_push(std::move(req))) {
        stats.cut_requests_submitted.fetch_add(1, std::memory_order_relaxed);
      } else {
        stats.cut_requests_dropped.fetch_add(1, std::memory_order_relaxed);
      }
    }

    apply_branch_history(true, gain, 0);

    return true;
  };

  // P5.1: Per-worker local DFS stack with work stealing support.
  // When work stealing is enabled, we use the WorkStealingPool; otherwise fallback
  // to a simple local vector (no stealing).
  WorkStealingDeque* my_deque = nullptr;
  std::vector<Node> local_dfs_fallback;
  if (steal_pool && opt.enable_work_stealing) {
    my_deque = &steal_pool->get(thread_id);
  }
  const int kLocalDfsMax = std::max(1, opt.work_steal_deque_size);
  std::vector<Node> frontier_drain_buffer;

  while (!should_stop.load(std::memory_order_relaxed)) {
    // Absorb any implications learned by sibling explorers since the last
    // iteration (lock-free generation check; only paying shared_lock when new
    // arcs are available). Keeping this at loop top ensures every node LP we
    // solve sees the freshest global tightenings.
    pull_shared_implications_if_any();

    // Atomic incumbent snapshot for the whole iteration.  Avoids the
    // torn `has()` / `get_obj()` read that can make two threads reach
    // different prune/keep decisions on the same node.  Refreshed below
    // only where a fresher view is actually useful (e.g. after solving
    // the LP and before pushing children, because another worker may
    // have published a better incumbent in the meantime).
    SharedIncumbent::Snapshot inc_snap = shared_inc.snapshot();

    Node cur;
    bool got_node = false;

    // Deterministic-parallel: block until it is this worker's turn to
    // touch the shared queue.  LP solves in the body run in parallel;
    // only the pop (and the later child push) are serialized.
    if (det_token) det_token->wait_turn(thread_id);

    auto drain_local_dfs_to_proof_frontier = [&]() {
      if (!inc_snap.has || !std::isfinite(inc_snap.obj)) return;
      frontier_drain_buffer.clear();
      if (my_deque) {
        my_deque->drain(frontier_drain_buffer);
      } else if (!local_dfs_fallback.empty()) {
        frontier_drain_buffer.reserve(local_dfs_fallback.size());
        while (!local_dfs_fallback.empty()) {
          frontier_drain_buffer.push_back(std::move(local_dfs_fallback.back()));
          local_dfs_fallback.pop_back();
        }
	      }
	      const bool changed_frontier = !frontier_drain_buffer.empty();
	      for (Node& node : frontier_drain_buffer) {
	        if (node_exceeds_optimality_limit(node.bound)) {
	          stats.gap_suboptimal_queue_prunes.fetch_add(
	              1, std::memory_order_relaxed);
	          continue;
	        }
	        if (!incumbent_prunes_node(true, inc_snap.obj, node.bound)) {
	          node_queue.push_pq(std::move(node));
	        }
      }
      frontier_drain_buffer.clear();
      if (changed_frontier) progress_event.notify();
      node_queue.prune_by_incumbent(true, inc_snap.obj,
                                    std::max(1e-9, kIncumbentPruneTol *
                                                        std::max(1.0, std::abs(inc_snap.obj))));
    };
    drain_local_dfs_to_proof_frontier();

    if (inc_snap.has) {
      // Proof phase: after a finite incumbent exists, all workers consume the
      // same best-bound frontier. Keeping Diver-local DFS stacks here expands
      // stale subtrees and destroys the certificate schedule.
      got_node = node_queue.pop_bestbound(cur);
    } else if (role == WorkerRole::Diver || role == WorkerRole::IPMDiver) {
      // Diver/IPMDiver: prefer local DFS stack, then steal, then global queue.
      if (my_deque) {
        // P5.1: Use work-stealing deque.
        if (my_deque->pop(cur)) {
          got_node = true;
        } else if (steal_pool->try_steal(thread_id, cur)) {
          // Successfully stole from another worker.
          got_node = true;
        } else {
          got_node = node_queue.pop(cur, inc_snap.has);
        }
      } else {
        // Fallback: no work stealing.
        if (!local_dfs_fallback.empty()) {
          cur = std::move(local_dfs_fallback.back());
          local_dfs_fallback.pop_back();
          got_node = true;
        } else {
          got_node = node_queue.pop(cur, inc_snap.has);
        }
      }
    } else {
      // Prover: prefer best-bound from global queue.
      got_node = node_queue.pop_bestbound(cur);
    }

    // Release pop-phase turn.  The push-phase turn is re-acquired later
    // once this worker is ready to publish its children.
    if (det_token) det_token->advance();

    if (!got_node) {
      if (should_stop.load(std::memory_order_acquire)) break;
      // When idle, attempt crossover heuristic (replaces cut_worker crossover).
      if (shared_sp.size() >= 2) {
        auto [s0, s1] = shared_sp.top2();
        Eigen::VectorXd xc = s0.x;
        for (int i = 0; i < n; ++i) {
	          if (!is_branchable_col(i)) continue;
          xc[i] = std::round(s0.x[i]);
        }
        for (int i = 0; i < n; ++i) {
	          if (!is_branchable_col(i)) continue;
          if (!is_integral(s0.x[i], 1e-5)) {
            xc[i] = std::round(s1.x[i]);
          }
        }
        for (int i = 0; i < n; ++i) {
          xc[i] = std::min(base_lp.vars[i].ub, std::max(base_lp.vars[i].lb, xc[i]));
        }
        if (satisfies_lp(base_lp, xc, 1e-6)) {
          const double obj = objective_value(base_lp.c, xc, base_lp.sense);
          if (shared_inc.try_update(xc, obj)) {
            shared_sp.add(xc, obj);
            stats.crossover_incumbents.fetch_add(1, std::memory_order_relaxed);
          }
        }
      }
      continue;
    }

    ActiveNodeGuard active_guard(active_bounds, active_explorers,
                                 progress_event, thread_id, cur.bound);

	    if (incumbent_prunes_node(inc_snap.has, inc_snap.obj, cur.bound)) {
	      continue;
	    }
	    if (node_exceeds_optimality_limit(cur.bound)) {
	      stats.gap_suboptimal_queue_prunes.fetch_add(
	          1, std::memory_order_relaxed);
	      continue;
	    }
	      if (use_shared_conflicts && shared_conflicts.has_conflict(cur.lb, cur.ub)) {
	        continue;
	      }
    std::vector<int> frac;
		    if (!fractional_branchable_indices(cur.x_relax, cur.lb, cur.ub,
		                                       opt.int_tol, frac)) {
      if (satisfies_with_bounds(base_lp, cur.x_relax, cur.lb, cur.ub, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, cur.x_relax, base_lp.sense);
        shared_inc.try_update(cur.x_relax, obj);
        shared_sp.add(cur.x_relax, obj);
      }
      record_node_explored();
      continue;
    }

    BCBranchContext branch_context;
    branch_context.depth = cur.depth;
    branch_context.node_lb = cur.bound;
    branch_context.best_obj =
        inc_snap.has ? inc_snap.obj : std::numeric_limits<double>::infinity();
    branch_context.candidates = &frac;
    branch_context.lp_x = cur.x_relax.data();
    branch_context.lp_x_size = static_cast<std::size_t>(cur.x_relax.size());
    std::vector<int> original_candidates;
    Eigen::VectorXd original_lp_x;
    if (original_col_count > 0 &&
        branch_original_cols.size() == static_cast<std::size_t>(n)) {
      bool mapping_valid = true;
      original_candidates.reserve(frac.size());
      for (int candidate : frac) {
        if (candidate < 0 || candidate >= n) {
          mapping_valid = false;
          break;
        }
        const int original =
            branch_original_cols[static_cast<std::size_t>(candidate)];
        if (original < 0 || original >= original_col_count) {
          mapping_valid = false;
          break;
        }
        original_candidates.push_back(original);
      }
      if (mapping_valid) {
        original_lp_x = Eigen::VectorXd::Zero(original_col_count);
        for (int col = 0; col < n; ++col) {
          const int original =
              branch_original_cols[static_cast<std::size_t>(col)];
          if (original >= 0 && original < original_col_count) {
            original_lp_x[original] = cur.x_relax[col];
          }
        }
        branch_context.candidates = &original_candidates;
        branch_context.lp_x = original_lp_x.data();
        branch_context.lp_x_size =
            static_cast<std::size_t>(original_lp_x.size());
      }
    }
    const bool has_dynamic_prior =
        dynamic_branching_prior != nullptr &&
        static_cast<bool>(*dynamic_branching_prior);

    int j = -1;
    std::vector<BranchDirectionalObservation> selected_direction_observations;
    const bool compact_pc_path = n >= 100000;
    if (role == WorkerRole::Prover && cur.depth <= opt.max_plunge_depth &&
        !compact_pc_path) {
      // Prover: reliability branching (copy shared PCs, probe, merge back).
      std::vector<PseudoCost> local_pc;
      std::vector<PseudoCost> base_pc;  // baseline snapshot for delta-merge
      {
        std::lock_guard<std::mutex> lk(pc_mtx);
        local_pc = pc;
      }
      if (opt.pseudocost_delta_merge) {
        base_pc = local_pc;  // copy baseline for diffing after probing
      }
      int relia_lp = 0;
      j = choose_branch_var_reliability(
          frac, cur.x_relax, local_pc, base_lp, base_sf,
          cur.lb, cur.ub, cur.basis_hint.get(), simplex_opt,
          cur.bound, relia_lp, cur.depth,
          /*reliability_limit=*/par_probe_reliability,
          /*max_probes=*/par_probe_max, branch_priority,
          has_dynamic_prior ? *dynamic_branching_prior : BCBranchingPriorFn{},
          branch_context, &selected_direction_observations);
      stats.lp_solves.fetch_add(relia_lp, std::memory_order_relaxed);
      // Merge probing results back to shared PCs.
      if (opt.pseudocost_delta_merge) {
        // Delta-aggregate merge: publish only the *new* samples this thread
        // accumulated since it copied the baseline. This is correct under
        // concurrent aggregation because sums and counts are linear in
        // samples (SCIP parallel-UG style). See
        // docs/parallel_bc_sharing_improvements_2026q2.md §4.
        std::lock_guard<std::mutex> lk(pc_mtx);
        for (int i : branchable_indices) {
          const int d_cnt = local_pc[i].down_cnt - base_pc[i].down_cnt;
          if (d_cnt > 0) {
            pc[i].down_cnt += d_cnt;
            pc[i].down_sum += (local_pc[i].down_sum - base_pc[i].down_sum);
            pc[i].down_sq  += (local_pc[i].down_sq  - base_pc[i].down_sq);
          }
          const int d_dco = local_pc[i].down_cutoff_cnt - base_pc[i].down_cutoff_cnt;
          if (d_dco > 0) pc[i].down_cutoff_cnt += d_dco;
          const double d_dcs = local_pc[i].down_conflict_score - base_pc[i].down_conflict_score;
          if (d_dcs > 0.0) pc[i].down_conflict_score += d_dcs;
          const int u_cnt = local_pc[i].up_cnt - base_pc[i].up_cnt;
          if (u_cnt > 0) {
            pc[i].up_cnt += u_cnt;
            pc[i].up_sum += (local_pc[i].up_sum - base_pc[i].up_sum);
            pc[i].up_sq  += (local_pc[i].up_sq  - base_pc[i].up_sq);
          }
          const int u_uco = local_pc[i].up_cutoff_cnt - base_pc[i].up_cutoff_cnt;
          if (u_uco > 0) pc[i].up_cutoff_cnt += u_uco;
          const double u_ucs = local_pc[i].up_conflict_score - base_pc[i].up_conflict_score;
          if (u_ucs > 0.0) pc[i].up_conflict_score += u_ucs;
        }
      } else {
        std::lock_guard<std::mutex> lk(pc_mtx);
        for (int i : branchable_indices) {
          if (local_pc[i].down_cnt > pc[i].down_cnt) {
            pc[i].down_sum = local_pc[i].down_sum;
            pc[i].down_sq = local_pc[i].down_sq;
            pc[i].down_cnt = local_pc[i].down_cnt;
          }
          if (local_pc[i].down_cutoff_cnt > pc[i].down_cutoff_cnt) {
            pc[i].down_cutoff_cnt = local_pc[i].down_cutoff_cnt;
          }
          if (local_pc[i].down_conflict_score > pc[i].down_conflict_score) {
            pc[i].down_conflict_score = local_pc[i].down_conflict_score;
          }
          if (local_pc[i].up_cnt > pc[i].up_cnt) {
            pc[i].up_sum = local_pc[i].up_sum;
            pc[i].up_sq = local_pc[i].up_sq;
            pc[i].up_cnt = local_pc[i].up_cnt;
          }
          if (local_pc[i].up_cutoff_cnt > pc[i].up_cutoff_cnt) {
            pc[i].up_cutoff_cnt = local_pc[i].up_cutoff_cnt;
          }
          if (local_pc[i].up_conflict_score > pc[i].up_conflict_score) {
            pc[i].up_conflict_score = local_pc[i].up_conflict_score;
          }
        }
      }
    } else {
      // Diver / deep Prover: fast pseudocost branching.
      // Take a lightweight read of the pseudocosts without copying the full vector.
      // Only read entries for fractional variables.
      std::vector<PseudoCost> frac_pc(frac.size());
      {
        std::lock_guard<std::mutex> lk(pc_mtx);
        for (size_t fi = 0; fi < frac.size(); ++fi)
          frac_pc[fi] = pc[frac[fi]];
      }
      j = choose_branch_var_pseudocost_candidates(
          frac, cur.x_relax, frac_pc, branch_priority,
          has_dynamic_prior ? *dynamic_branching_prior : BCBranchingPriorFn{},
          branch_context);
    }
    if (j < 0) {
      record_node_explored();
      continue;
    }

    const double xj = cur.x_relax[j];
    const double parent_bound = cur.bound;
    const double parent_estimate = cur.estimate;
    const int parent_depth = cur.depth;
    Eigen::VectorXd parent_x_relax = std::move(cur.x_relax);
    pc_updates_buffer.clear();

    PseudoCost direction_pc;
    {
      std::lock_guard<std::mutex> lk(pc_mtx);
      direction_pc = pc[static_cast<std::size_t>(j)];
    }
    const double branch_fractionality = xj - std::floor(xj);
    const BranchDirectionalObservation* down_direction_observation = nullptr;
    const BranchDirectionalObservation* up_direction_observation = nullptr;
    for (const auto& observation : selected_direction_observations) {
      if (observation.var != j) continue;
      if (observation.is_up) up_direction_observation = &observation;
      else down_direction_observation = &observation;
    }
    const auto history_prediction_usable =
        [](const BranchDirectionalObservation* observation) {
          return observation == nullptr ||
                 observation->state == BranchEvidenceState::Predicted ||
                 observation->state == BranchEvidenceState::UnknownFailure;
        };
    const bool down_prediction_available =
        history_prediction_usable(down_direction_observation);
    const bool up_prediction_available =
        history_prediction_usable(up_direction_observation);
    const double predicted_down_gain = compute_directional_branch_score(
        direction_pc, branch_fractionality, false, nullptr);
    const double predicted_up_gain = compute_directional_branch_score(
        direction_pc, 1.0 - branch_fractionality, true, nullptr);
    const bool prefer_up = choose_up_branch_first(
        direction_pc, branch_fractionality, down_direction_observation,
        up_direction_observation, inc_snap.has);
    if (prefer_up) {
      stats.branch_direction_preferred_up.fetch_add(1,
                                                    std::memory_order_relaxed);
      stats.branch_direction_first_up.fetch_add(1, std::memory_order_relaxed);
    } else {
      stats.branch_direction_preferred_down.fetch_add(
          1, std::memory_order_relaxed);
      stats.branch_direction_first_down.fetch_add(1,
                                                  std::memory_order_relaxed);
    }

    Node child_down = cur.branch_child();
    child_down.ub[j] = std::min(child_down.ub[j], std::floor(xj));
      append_branch_reason(child_down, j, false, child_down.ub[j]);

    Node child_up = std::move(cur);
    child_up.x_relax.resize(0);
    child_up.x_seed.resize(0);
    child_up.ipm_iterations = 0;
    child_up.lp_refresh_needed = false;
    child_up.depth = parent_depth + 1;
    child_up.lb[j] = std::max(child_up.lb[j], std::ceil(xj));
      append_branch_reason(child_up, j, true, child_up.lb[j]);

    bool down_valid = false;
    bool up_valid = false;
    auto process_direction = [&](bool is_up, int ordinal) {
      Node& child = is_up ? child_up : child_down;
      bool& valid = is_up ? up_valid : down_valid;
      const std::size_t update_start = pc_updates_buffer.size();
      const auto incumbent_before = shared_inc.snapshot();
      valid = process_child(child, is_up, j, parent_bound, parent_x_relax);
      const auto incumbent_after = shared_inc.snapshot();
      const bool cutoff = std::any_of(
          pc_updates_buffer.begin() +
              static_cast<std::ptrdiff_t>(update_start),
          pc_updates_buffer.end(),
          [](const PCUpdate& update) { return update.cutoff_count > 0; });
      if (cutoff) {
        if (ordinal == 0) {
          stats.branch_first_child_cutoffs.fetch_add(
              1, std::memory_order_relaxed);
        } else {
          stats.branch_second_child_cutoffs.fetch_add(
              1, std::memory_order_relaxed);
        }
      }
      if (ordinal == 0 && incumbent_after.has &&
          (!incumbent_before.has || incumbent_after.obj < incumbent_before.obj)) {
        stats.branch_first_child_incumbent_updates.fetch_add(
            1, std::memory_order_relaxed);
      }
    };
    const std::array<bool, 2> direction_order{prefer_up, !prefer_up};
    for (int ordinal = 0; ordinal < 2; ++ordinal) {
      process_direction(direction_order[static_cast<std::size_t>(ordinal)],
                        ordinal);
    }
    const auto calibration = compute_branch_estimator_calibration(
        parent_bound, parent_estimate, down_prediction_available,
        predicted_down_gain, up_prediction_available, predicted_up_gain,
        down_valid, child_down.bound, up_valid, child_up.bound);
    stats.node_estimate_calibration_samples.fetch_add(
        calibration.node_samples, std::memory_order_relaxed);
    atomic_add_relaxed(stats.node_estimate_predicted_lift_sum,
                       calibration.node_predicted_lift_sum);
    atomic_add_relaxed(stats.node_estimate_realized_lift_sum,
                       calibration.node_realized_lift_sum);
    atomic_add_relaxed(stats.node_estimate_abs_error_sum,
                       calibration.node_abs_error_sum);
    atomic_add_relaxed(stats.node_estimate_squared_error_sum,
                       calibration.node_squared_error_sum);
    atomic_add_relaxed(stats.node_estimate_predicted_sq_sum,
                       calibration.node_predicted_sq_sum);
    atomic_add_relaxed(stats.node_estimate_realized_sq_sum,
                       calibration.node_realized_sq_sum);
    atomic_add_relaxed(stats.node_estimate_cross_sum,
                       calibration.node_cross_sum);
    stats.directional_calibration_samples.fetch_add(
        calibration.directional_samples, std::memory_order_relaxed);
    atomic_add_relaxed(stats.directional_predicted_gain_sum,
                       calibration.directional_predicted_gain_sum);
    atomic_add_relaxed(stats.directional_realized_gain_sum,
                       calibration.directional_realized_gain_sum);
    atomic_add_relaxed(stats.directional_abs_error_sum,
                       calibration.directional_abs_error_sum);
    atomic_add_relaxed(stats.directional_squared_error_sum,
                       calibration.directional_squared_error_sum);
    stats.directional_rank_samples.fetch_add(
        calibration.directional_rank_samples, std::memory_order_relaxed);
    stats.directional_rank_concordant.fetch_add(
        calibration.directional_rank_concordant, std::memory_order_relaxed);

    // Refresh incumbent snapshot: the LP solves above may have published
    // a better incumbent, and using the fresh value here tightens the
    // child prune decision without breaking determinism (both children
    // in this iteration see the same snapshot).
    inc_snap = shared_inc.snapshot();
	    if (inc_snap.has) {
	      node_queue.prune_by_incumbent(true, inc_snap.obj,
	                                    std::max(1e-9, kIncumbentPruneTol *
	                                                        std::max(1.0, std::abs(inc_snap.obj))));
	      if (down_valid && incumbent_prunes_node(true, inc_snap.obj, child_down.bound)) down_valid = false;
	      if (up_valid && incumbent_prunes_node(true, inc_snap.obj, child_up.bound)) up_valid = false;
	    }
	    const double child_opt_limit = current_optimality_limit();
	    if (!opt.require_tree_exhaustion_certificate &&
	        std::isfinite(child_opt_limit)) {
	      stats.gap_suboptimal_queue_prune_passes.fetch_add(
	          1, std::memory_order_relaxed);
	      const double opt_limit_tol =
	          gap_optimality_prune_tol_parallel(child_opt_limit, opt.lp_tol);
	      if (down_valid && std::isfinite(child_down.bound) &&
	          child_down.bound >= child_opt_limit - opt_limit_tol) {
	        down_valid = false;
	        stats.gap_suboptimal_queue_prunes.fetch_add(
	            1, std::memory_order_relaxed);
	      }
	      if (up_valid && std::isfinite(child_up.bound) &&
	          child_up.bound >= child_opt_limit - opt_limit_tol) {
	        up_valid = false;
	        stats.gap_suboptimal_queue_prunes.fetch_add(
	            1, std::memory_order_relaxed);
	      }
	    }

	    {
      std::lock_guard<std::mutex> lk(pc_mtx);
      for (const auto& upd : pc_updates_buffer) {
        if (upd.var >= 0 && upd.var < n) {
          upd.apply(pc[upd.var]);
        }
      }
      pc_updates_buffer.clear();
      if (down_valid) {
        child_down.estimate = compute_node_estimate(branchable_indices, child_down.x_relax,
                                                    pc, opt.int_tol, child_down.bound,
                                                    opt.node_estimate_aggregation);
      }
      if (up_valid) {
        child_up.estimate = compute_node_estimate(branchable_indices, child_up.x_relax,
                                                  pc, opt.int_tol, child_up.bound,
                                                  opt.node_estimate_aggregation);
      }
    }

    if (down_valid) child_down.x_seed.resize(0);
    if (up_valid) child_up.x_seed.resize(0);

    // Deterministic-parallel: re-acquire the token before publishing
    // children so push order is a deterministic function of the round-
    // robin worker rotation.
    if (det_token) det_token->wait_turn(thread_id);

    if (inc_snap.has) {
      // Proof phase: publish children directly to the shared best-bound queue.
      if (down_valid) node_queue.push_pq(std::move(child_down));
      if (up_valid) node_queue.push_pq(std::move(child_up));
    } else if (role == WorkerRole::Diver || role == WorkerRole::IPMDiver) {
      // Diver/IPMDiver: push DFS-preferred child to local stack, other to global PQ.
      // P5.1: Use work-stealing deque when available.
      auto push_local = [&](Node&& node) {
        if (my_deque) {
          if (!my_deque->push(std::move(node))) {
            // Deque full, spill the still-owned node to the global queue.
            node_queue.push_dfs(std::move(node));
          }
        } else {
          // Fallback without work stealing.
          if (static_cast<int>(local_dfs_fallback.size()) < kLocalDfsMax) {
            local_dfs_fallback.push_back(std::move(node));
          } else {
            node_queue.push_dfs(std::move(node));
          }
        }
      };

      if (down_valid && up_valid) {
        if (!prefer_up) {
          node_queue.push_pq(std::move(child_up));
          push_local(std::move(child_down));
        } else {
          node_queue.push_pq(std::move(child_down));
          push_local(std::move(child_up));
        }
      } else if (down_valid) {
        push_local(std::move(child_down));
      } else if (up_valid) {
        push_local(std::move(child_up));
      }
    } else {
      // Prover: push both children to global PQ (best-bound selection).
      if (down_valid && up_valid) {
        node_queue.push_pq(std::move(child_down));
        node_queue.push_pq(std::move(child_up));
      } else if (down_valid) {
        node_queue.push_pq(std::move(child_down));
      } else if (up_valid) {
        node_queue.push_pq(std::move(child_up));
      }
    }

    // Release push-phase turn.
    if (det_token) det_token->advance();

    record_node_explored();
  }

  // Flush any remaining locally-learned implications before this worker exits,
  // so nothing learned in the last batch is lost to sibling threads that may
  // still be running (cooperative finish path).
  flush_publish_buffer();

  // Deterministic-parallel: mark this worker as done so the turn token
  // skips us in its rotation and sibling threads do not block waiting
  // for a turn we will never take.
  if (det_token) det_token->mark_done(thread_id);
  progress_event.notify();
}

void detail::cut_worker_thread(
    std::shared_ptr<const BcEnvOptions> environment,
    CutRequestQueue& cut_queue,
    SharedCutPool& shared_cp,
    SharedSolutionPool& shared_sp,
    SharedIncumbent& shared_inc,
    const LPModel& base_lp,
    const BCOptions& opt,
    AtomicBCStats& stats,
    std::atomic<bool>& should_stop) {
  ScopedBcEnvOptions environment_scope(std::move(environment));

  const int n = static_cast<int>(base_lp.vars.size());

  while (!should_stop.load(std::memory_order_relaxed)) {
    auto maybe_req = cut_queue.pop(std::chrono::milliseconds(
        std::max(1, opt.cut_worker_queue_timeout_ms)));

    if (!maybe_req.has_value()) {
      if (should_stop.load(std::memory_order_acquire)) break;
      if (shared_sp.size() >= 2) {
        auto [s0, s1] = shared_sp.top2();
        Eigen::VectorXd xc = s0.x;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          xc[i] = std::round(s0.x[i]);
        }
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          if (!is_integral(s0.x[i], 1e-5)) {
            xc[i] = std::round(s1.x[i]);
          }
        }
        for (int i = 0; i < n; ++i) {
          xc[i] = std::min(base_lp.vars[i].ub, std::max(base_lp.vars[i].lb, xc[i]));
        }
        if (satisfies_lp(base_lp, xc, 1e-6)) {
          const double obj = objective_value(base_lp.c, xc, base_lp.sense);
          if (shared_inc.try_update(xc, obj)) {
            shared_sp.add(xc, obj);
            stats.crossover_incumbents.fetch_add(1, std::memory_order_relaxed);
            stats.cut_worker_incumbents.fetch_add(1, std::memory_order_relaxed);
          }
        }
      }
      continue;
    }

    CutRequest& req = *maybe_req;

    if (req.node_depth == 0) {
      LPModel node_lp = base_lp;
      apply_node_bounds(node_lp.vars, req.node_lb, req.node_ub);
      std::shared_ptr<BasisOps> req_sbasis = req.simplex_result.basis.cached_sparse_basis;
      const int added = add_transformed_tableau_cuts(
          node_lp, req.x_relax, req.simplex_result, opt, opt.cuts_per_round,
          req_sbasis, nullptr, nullptr, nullptr);
      if (added > 0) {
        stats.cuts_added.fetch_add(added, std::memory_order_relaxed);
        stats.cut_worker_cuts.fetch_add(added, std::memory_order_relaxed);

        const int orig_rows = static_cast<int>(base_lp.A.rows());
        const int total_rows = static_cast<int>(node_lp.A.rows());
        // Only root cuts are exported globally. Node-bounded Gomory cuts are
        // local to that subproblem and would be unsound in the shared pool.
        Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_view = node_lp.A;
        for (int r = orig_rows; r < total_rows; ++r) {
          Eigen::SparseVector<double> row_sparse(n);
          int nnz = 0;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_view, r); it; ++it) {
            if (it.col() < n) ++nnz;
          }
          row_sparse.reserve(nnz);
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_view, r); it; ++it) {
            if (it.col() < n) {
              row_sparse.insertBack(it.col()) = it.value();
            }
          }
          shared_cp.add(std::move(row_sparse), node_lp.b[r]);
        }
      }
    }

    if (shared_sp.size() > 0) {
      Eigen::VectorXd xr = shared_sp.guided_rounding(req.x_relax, base_lp.vars);
      xr = clamp_to_bounds(xr, req.node_lb, req.node_ub);
      LPModel check_lp = base_lp;
      apply_node_bounds(check_lp.vars, req.node_lb, req.node_ub);
      if (satisfies_lp(check_lp, xr, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, xr, base_lp.sense);
        if (shared_inc.try_update(xr, obj)) {
          shared_sp.add(xr, obj);
          stats.cut_worker_incumbents.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
  }
}

}  // namespace mipsolvers::engine

/// @file bc_concurrent_tree_state.hpp
/// @brief Shared state bundle for the legacy parallel branch-and-cut tree.

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {

struct ConcurrentTreeState {
  SharedCutPool shared_cp;
  SharedConflictPool shared_conflicts;
  SharedSolutionPool shared_sp;
  SharedIncumbent shared_inc;
  ThreadSafeNodeQueue shared_queue;
  CutRequestQueue cut_req_queue;
  AtomicBCStats atomic_stats;
  std::atomic<bool> should_stop{false};
  std::atomic<double> optimality_limit{kInf};
  std::atomic<int> active_explorers{0};
  ActiveNodeBounds active_bounds;
  std::mutex pc_mtx;
  std::vector<PseudoCost> pseudocosts;
  FallbackLogger fallback_logger;
  SimplexOptions par_simplex_opt;
  std::unique_ptr<WorkStealingPool> steal_pool;
  SharedImplicationGraph shared_impl_graph;
  std::unique_ptr<DeterministicTurnToken> det_token;
  std::vector<std::thread> explorers;
  std::thread cut_worker;
  bool use_cut_worker{false};
  int n_explorers{0};

  ConcurrentTreeState(const BCOptions& opt,
                      std::chrono::steady_clock::time_point t0,
                      int cut_coeff_size)
      : shared_cp(opt.cut_pool_max_size, opt.cut_pool_max_age, cut_coeff_size),
        shared_conflicts(
            opt.cut_pool_max_size,
            std::max(64, opt.reduced_cost_conflict_pool_max_literals)),
        shared_sp(opt.solution_pool_size),
        shared_queue(opt.node_sel),
        cut_req_queue(opt.cut_worker_queue_size),
        active_bounds(std::max(1, resolve_num_threads(opt))),
        pseudocosts(static_cast<std::size_t>(std::max(0, cut_coeff_size))),
        fallback_logger(t0) {
    par_simplex_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
    par_simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    par_simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    par_simplex_opt.allow_cold_start = false;
    par_simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
    par_simplex_opt.factor_backend =
        simplex_factor_backend_from_id(opt.simplex_factor_backend);
  }

  void shutdown_and_join() {
    should_stop.store(true, std::memory_order_release);
    shared_queue.request_shutdown();
    cut_req_queue.request_shutdown();
    for (auto& t : explorers) {
      if (t.joinable()) t.join();
    }
    if (cut_worker.joinable()) cut_worker.join();
  }
};

}  // namespace mipsolvers::engine::detail

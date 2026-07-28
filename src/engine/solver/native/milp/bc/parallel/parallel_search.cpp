/// @file parallel_search.cpp
/// @brief Implementation of ParallelSearchDriver (Phase 4.6).
///
/// Wraps the existing detail::explorer_thread / detail::cut_worker_thread
/// dispatch into a clean component interface driven by BCSolveContext.
///
/// Design principle: this file delegates to the battle-tested parallel
/// infrastructure in bc_parallel.cpp. Its sole responsibility is mapping
/// BCSolveContext state into the exact parameter set each thread function
/// expects, and collecting results when threads finish.

#include "mipsolvers/engine/solver/native/milp/bc/parallel/parallel_search.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "mipsolvers/engine/solver/native/milp/bc/parallel/shared_state.hpp"
#include "mipsolvers/engine/detail/bc_parallel.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::solver::native::milp::bc {

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

ParallelSearchDriver::ParallelSearchDriver(BCSolveContext& context)
    : ctx_(context) {}

// ─────────────────────────────────────────────────────────────────────────────
// Public execute()
// ─────────────────────────────────────────────────────────────────────────────

ParallelSearchStats ParallelSearchDriver::execute(
    const detail::Node& root_node,
    const ParallelWorkerConfig& config) {

  const auto t_start = std::chrono::steady_clock::now();

  // Reset local termination flag for this execute() call.
  should_stop_.store(false, std::memory_order_relaxed);
  active_explorers_.store(0, std::memory_order_relaxed);

  // Initialize shared communication queues.
  const BCOptions& opt = ctx_.options();
  node_queue_ = std::make_unique<detail::ThreadSafeNodeQueue>(opt.node_sel);
  cut_req_queue_ = std::make_unique<detail::CutRequestQueue>(opt.cut_worker_queue_size);
  shared_impl_graph_ = std::make_unique<detail::SharedImplicationGraph>();

  // Seed SharedIncumbent from context so threads start with the current best.
  if (ctx_.has_incumbent()) {
    shared_inc_.try_update(ctx_.get_incumbent_solution(),
                           ctx_.incumbent_objective());
  }

  // Deterministic token for serialized node pop/push (optional).
  if (config.deterministic_mode && config.n_explorers > 1) {
    det_token_ = std::make_unique<detail::DeterministicTurnToken>(config.n_explorers);
  }

  // Work-stealing pool for per-worker DFS deques.
  if (config.enable_work_stealing && config.n_explorers > 1) {
    steal_pool_ = std::make_unique<detail::WorkStealingPool>(
        config.n_explorers,
        /* max_deque_size= */ 256,
        config.random_seed);
  }

  // Launch all worker threads.
  launch_workers(root_node, config);

  // Monitor until context requests termination or all explorers finish.
  while (true) {
    const bool work_in_flight = active_explorers_.load(std::memory_order_relaxed) > 0;
    const bool global_queue_nonempty = node_queue_ && !node_queue_->empty();
    const bool local_work_nonempty = steal_pool_ && (steal_pool_->total_size() > 0);
    if (!work_in_flight && !global_queue_nonempty && !local_work_nonempty) {
      break;
    }

    // Propagate context terminate flag into local should_stop.
    if (ctx_.should_terminate()) {
      should_stop_.store(true, std::memory_order_release);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Signal cut worker and drain remaining requests.
  if (cut_req_queue_) cut_req_queue_->request_shutdown();
  should_stop_.store(true, std::memory_order_release);
  if (node_queue_) node_queue_->request_shutdown();

  ParallelSearchStats stats = join_workers();

  // Propagate best incumbent from shared_inc_ back to context.
  if (shared_inc_.has()) {
    ctx_.update_incumbent(shared_inc_.get_x(), shared_inc_.get_obj());
  }

  const auto t_end = std::chrono::steady_clock::now();
  stats.search_time_sec =
      std::chrono::duration<double>(t_end - t_start).count();

  // Pull stats from context atomics.
  stats.nodes_explored = static_cast<int>(ctx_.nodes_explored());
  stats.solutions_found = ctx_.has_incumbent() ? 1 : 0;
  stats.cut_worker_cuts = atomic_stats_.cut_worker_cuts.load();

  // Work-stealing summary.
  if (steal_pool_) {
    stats.work_steals = steal_pool_->get_total_steals();
  }

  if (should_stop_.load()) {
    stats.termination_reason = "Stop requested (time/node/gap limit)";
  } else {
    stats.termination_reason = "Queue exhausted (optimal or proven infeasible)";
  }

  // Clean up owned resources.
  steal_pool_.reset();
  node_queue_.reset();
  cut_req_queue_.reset();
  shared_impl_graph_.reset();
  det_token_.reset();
  fallback_logger_.reset();

  return stats;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void ParallelSearchDriver::launch_workers(
    const detail::Node& root_node,
    const ParallelWorkerConfig& config) {

  const BCOptions& opt = ctx_.options();
  const LPModel& base_lp = ctx_.base_lp();

  // Decide whether to spawn a cut worker.
  has_cut_worker_ = config.use_cut_worker &&
                    (opt.cuts != CutType::None) &&
                    (opt.max_cut_depth > 0) &&
                    (config.n_explorers >= 2);

  // Configure simplex options for explorer threads.
  SimplexOptions par_simplex_opt;
  par_simplex_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
  par_simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
  par_simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
  par_simplex_opt.allow_cold_start = false;
  par_simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
  par_simplex_opt.factor_backend =
      simplex_factor_backend_from_id(opt.simplex_factor_backend);

  // Seed shared node queue with root node.
  const bool has_inc = ctx_.has_incumbent();
  node_queue_->push(root_node, has_inc);

  // Determine roles per explorer.
  const std::vector<detail::WorkerRole> roles = assign_roles(config.n_explorers);

  // Root basis hint (shared read-only across threads for warm-start).
  std::shared_ptr<const SimplexBasis> root_basis = root_node.basis_hint;

  // Build the shared standard-form template once. Explorer threads take it by
  // const reference and apply node-specific bound updates locally.
  StandardFormLP base_sf = build_standard_form_lp(base_lp);
  std::vector<char> branchable_cols(
      static_cast<std::size_t>(base_lp.vars.size()), 0);
  for (int j = 0; j < static_cast<int>(base_lp.vars.size()); ++j) {
    if (detail::is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
      branchable_cols[static_cast<std::size_t>(j)] = 1;
    }
  }

  // FallbackLogger (timestamped IPM fallback events).
  // Stored as a class member so it outlives launch_workers() scope.
  const auto t_launch = std::chrono::steady_clock::now();
  fallback_logger_ = std::make_unique<detail::FallbackLogger>(t_launch);

  // Launch explorers.
  const int n_exp = static_cast<int>(roles.size());
  active_bounds_.resize(n_exp);
  explorer_threads_.clear();
  explorer_threads_.reserve(static_cast<std::size_t>(n_exp));

  for (int tid = 0; tid < n_exp; ++tid) {
    const detail::WorkerRole role = roles[static_cast<std::size_t>(tid)];

    explorer_threads_.emplace_back(
        detail::explorer_thread,
        tid,
        role,
        std::ref(*node_queue_),
        std::ref(*cut_req_queue_),
        std::ref(ctx_.cut_pool()),
        std::ref(ctx_.conflict_pool()),
        std::ref(ctx_.solution_pool()),
        // shared_inc_ bridges the thread-native SharedIncumbent API with
        // BCSolveContext.  After join, execute() propagates the best solution
        // back to ctx_.update_incumbent().
        std::ref(shared_inc_),
        std::cref(base_lp),
        std::cref(base_lp),  // pre_cut_lp same as base for now
        std::cref(base_sf),
        std::cref(branchable_cols),
        std::cref(ctx_.clique_table()),
        std::cref(ctx_.implication_graph()),
        std::cref(par_simplex_opt),
        std::cref(opt),
        std::ref(ctx_.pseudocosts()),
        std::ref(ctx_.pseudocost_mutex()),
        std::ref(atomic_stats_),
        std::ref(should_stop_),
        std::ref(active_explorers_),
        std::ref(active_bounds_),
        std::ref(*fallback_logger_),
        root_basis,
        steal_pool_.get(),
        shared_impl_graph_.get(),
        det_token_.get(),
        nullptr
    );
  }

  // Launch cut worker if enabled.
  if (has_cut_worker_) {
    cut_worker_thread_ = std::thread(
        detail::cut_worker_thread,
        std::ref(*cut_req_queue_),
        std::ref(ctx_.cut_pool()),
        std::ref(ctx_.solution_pool()),
        std::ref(shared_inc_),
        std::cref(base_lp),
        std::cref(opt),
        std::ref(atomic_stats_),
        std::ref(should_stop_)
    );
  }
}

ParallelSearchStats ParallelSearchDriver::join_workers() {
  ParallelSearchStats stats;

  // Join all explorer threads.
  for (auto& t : explorer_threads_) {
    if (t.joinable()) t.join();
  }
  explorer_threads_.clear();

  // Join cut worker.
  if (has_cut_worker_ && cut_worker_thread_.joinable()) {
    cut_worker_thread_.join();
  }
  has_cut_worker_ = false;

  return stats;
}

std::vector<detail::WorkerRole> ParallelSearchDriver::assign_roles(
    int n_explorers) const {

  std::vector<detail::WorkerRole> roles;
  roles.reserve(static_cast<std::size_t>(n_explorers));

  const BCOptions& opt = ctx_.options();
  const int presolved_m = static_cast<int>(ctx_.base_lp().A.rows()) +
                          static_cast<int>(ctx_.base_lp().Aeq.rows());
  const bool use_hybrid_ipm =
      (n_explorers >= 2) && (presolved_m > opt.hybrid_ipm_threshold);

  for (int i = 0; i < n_explorers; ++i) {
    if (i == 0) {
      // Thread 0: Diver (DFS-focused, fast incumbent finding).
      roles.push_back(detail::WorkerRole::Diver);
    } else if (use_hybrid_ipm && i == n_explorers - 1) {
      // Last thread on large problems: IPM-based DFS (fast ~30ms node LPs).
      roles.push_back(detail::WorkerRole::IPMDiver);
    } else if (i % 3 == 0) {
      // Every 3rd thread: Diver.
      roles.push_back(detail::WorkerRole::Diver);
    } else if (i % 3 == 1) {
      // Every 3rd+1 thread: Prover (best-bound, proving optimality).
      roles.push_back(detail::WorkerRole::Prover);
    } else {
      // Every 3rd+2 thread: IPMDiver on large problems, else Diver.
      roles.push_back(use_hybrid_ipm ? detail::WorkerRole::IPMDiver
                                     : detail::WorkerRole::Diver);
    }
  }

  return roles;
}

}  // namespace mipsolvers::engine::solver::native::milp::bc

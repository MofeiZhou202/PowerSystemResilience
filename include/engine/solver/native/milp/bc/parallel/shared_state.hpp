/// @file shared_state.hpp
/// @brief Unified B&C solver context: centralizes all mutable runtime state
///        across root, sequential, and parallel execution paths.
///
/// BCSolveContext is the single aggregate that owns:
/// - Problem data (standard form, graph structures)
/// - Incumbent solution and bounds
/// - All pools (cuts, solutions, conflicts)
/// - Branching statistics (pseudocosts)
/// - Solver instances and dispatchers
/// - Runtime counters and state flags
///
/// This design eliminates lambda capture sprawl, provides a clear lifetime
/// root for all components, and mirrors HiGHS's HighsMipSolverData pattern.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/engine/bc/options.hpp"
#include "hacdcpf/engine/solver/solver_adapter.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/detail/bc_types.hpp"
#include "hacdcpf/engine/detail/bc_pools.hpp"
#include "hacdcpf/engine/detail/bc_threading.hpp"
#include "hacdcpf/engine/detail/bc_clique_table.hpp"
#include "hacdcpf/engine/detail/bc_utils.hpp"
#include "hacdcpf/engine/detail/bc_solver_dispatch.hpp"

namespace hacdcpf::engine::solver::native::milp::bc {

/// @class BCSolveContext
/// @brief Single mutable state root for branch-and-cut solver execution.
///
/// Ownership model:
/// - Lifetime: Created at solve start, destroyed at termination
/// - Mutation: Only from root solve, sequential loop, or parallel workers
/// - Sharing: Passed by reference/pointer to all executor components
/// - Synchronization: Internal locks (cut pool, incumbent, pseudocosts)
///
/// Usage pattern:
/// @code
///   BCSolveContext ctx(prob, opt);
///   run_root_pipeline(ctx);         // Root initialization & cuts
///   run_sequential_search(ctx);     // Main B&C loop (or parallel)
///   BCResult result = ctx.build_result();
/// @endcode
class BCSolveContext {
 public:
  // ─────────────────────────────────────────────────────────────────────
  // Initialization
  // ─────────────────────────────────────────────────────────────────────

  /// Construct context from problem and options.
  /// @param prob_in           Input MILP or MINLP model (presolved)
  /// @param opt_in            Algorithm options (algorithm tuning, limits)
  /// @param presolve_stats_in  Optional presolve transformation data
  BCSolveContext(const MIPModel& prob_in,
                 const BCOptions& opt_in,
                 const SolveStats* presolve_stats_in = nullptr);

  /// Deleted copy to enforce single ownership.
  BCSolveContext(const BCSolveContext&) = delete;
  BCSolveContext& operator=(const BCSolveContext&) = delete;

  /// Move not allowed (references and atomics cannot move).
  BCSolveContext(BCSolveContext&&) noexcept = delete;
  BCSolveContext& operator=(BCSolveContext&&) noexcept = delete;

  ~BCSolveContext() = default;

  // ─────────────────────────────────────────────────────────────────────
  // Problem Data (read-only after initialization)
  // ─────────────────────────────────────────────────────────────────────

  /// Original problem (post-presolve if preprocessing was applied).
  const MIPModel& problem() const { return prob; }

  /// Base LP from the problem.
  const LPModel& base_lp() const { return prob.linear_part; }

  /// Standard form representation (lazy-built on first access).
  StandardFormLP* standard_form() { return sf.get(); }
  const StandardFormLP* standard_form() const { return sf.get(); }

  /// Graph structures for bound propagation and cut generation.
  const detail::CliqueTable& clique_table() const { return cliques; }
  detail::CliqueTable& clique_table_mut() { return cliques; }

  const detail::BinaryImplicationGraph& implication_graph() const {
    return implications;
  }
  detail::BinaryImplicationGraph& implication_graph_mut() {
    return implications;
  }

  // ── Cached row-major matrices for domain propagation ──────────────────
  // Lazily built on first call; thread-safe (guarded by row_cache_mtx_).
  const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row_cached() const;
  const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row_cached() const;
  const detail::RowPropagationIndex& row_prop_index() const;

  // ─────────────────────────────────────────────────────────────────────
  // Incumbent Solution State
  // ─────────────────────────────────────────────────────────────────────

  /// Check if any feasible integer solution has been found.
  bool has_incumbent() const { return incumbent_found.load(); }

  /// Current best objective value (or infinity if no incumbent).
  double incumbent_objective() const { return incumbent_obj.load(); }

  /// Get read-only copy of incumbent solution.
  Eigen::VectorXd get_incumbent_solution() const {
    std::lock_guard<std::mutex> lock(incumbent_mtx);
    return incumbent_x;
  }

  /// Update incumbent from a new feasible solution.
  /// @return true if this is a better solution than current incumbent
  bool update_incumbent(const Eigen::VectorXd& new_x, double new_obj);

  // ─────────────────────────────────────────────────────────────────────
  // Bounds Tracking
  // ─────────────────────────────────────────────────────────────────────

  /// Current dual lower bound (best LP relaxation bound).
  double dual_bound() const { return dual_bound_.load(); }

  /// Update dual bound (lower bound in minimization).
  void update_dual_bound(double new_bound);

  /// Current primal upper bound (incumbent or infinity).
  double primal_bound() const {
    if (has_incumbent()) return incumbent_objective();
    return std::numeric_limits<double>::infinity();
  }

  /// Optimality gap in absolute value.
  double gap() const {
    const double dual = dual_bound_.load();
    const double primal = primal_bound();
    if (std::isinf(primal)) return std::numeric_limits<double>::infinity();
    return std::max(0.0, primal - dual);
  }

  // ─────────────────────────────────────────────────────────────────────
  // Pools (Cut, Solution, Conflict)
  // ─────────────────────────────────────────────────────────────────────

  /// Cut pool for root and node cuts.
  detail::SharedCutPool& cut_pool() { return cut_pool_; }
  const detail::SharedCutPool& cut_pool() const { return cut_pool_; }

  /// Solution pool (top-K feasible solutions).
  detail::SharedSolutionPool& solution_pool() { return sol_pool_; }
  const detail::SharedSolutionPool& solution_pool() const {
    return sol_pool_;
  }

  /// Conflict pool (learned branch-domain conflicts).
  detail::SharedConflictPool& conflict_pool() { return conflict_pool_; }
  const detail::SharedConflictPool& conflict_pool() const {
    return conflict_pool_;
  }

  // ─────────────────────────────────────────────────────────────────────
  // Branching Statistics & Guidance
  // ─────────────────────────────────────────────────────────────────────

  /// Per-variable pseudocost table (mutex-protected for concurrent updates).
  std::vector<detail::PseudoCost>& pseudocosts() { return pseudocosts_; }
  const std::vector<detail::PseudoCost>& pseudocosts() const {
    return pseudocosts_;
  }

  /// Lock for pseudocost updates.
  std::mutex& pseudocost_mutex() { return pseudocost_mtx; }

  /// Optional branch variable priorities (hard/soft user hints).
  const std::vector<double>& branch_priorities() const {
    return branch_priority;
  }

  /// Cut family performance tracker (adaptive cut selection).
  detail::CutFamilyTracker& cut_family_tracker() { return cut_tracker; }

  // ─────────────────────────────────────────────────────────────────────
  // Solver Instances & Dispatchers
  // ─────────────────────────────────────────────────────────────────────

  /// Primary simplex dispatcher (warm-started with basis).
  detail::SolverDispatcher& simplex_dispatcher() {
    return *simplex_dispatch;
  }

  /// IPM solver for node LPs (if enabled).
  /// Note: Type elided for Phase 4.1 (void* placeholder, will be wired in Phase 4.2)
  void* ipm_node_solver() { return ipm_node; }
  const void* ipm_node_solver() const {
    return ipm_node;
  }

  /// IPM fallback solver (verify infeasibility claims).
  void* ipm_fallback_solver() { return ipm_fallback; }

  /// IPM solver for cut pool re-solves.
  void* ipm_cut_solver() { return ipm_cut; }

  // ─────────────────────────────────────────────────────────────────────
  // Runtime State & Counters
  // ─────────────────────────────────────────────────────────────────────

  /// Total nodes explored so far.
  int64_t nodes_explored() const { return node_count.load(); }

  /// Increment node count.
  void increment_nodes(int64_t by = 1) { node_count.fetch_add(by); }

  /// Total LP solves performed.
  int64_t lp_solves() const { return lp_solve_count.load(); }

  /// Increment LP solve count.
  void increment_lp_solves(int64_t by = 1) {
    lp_solve_count.fetch_add(by);
  }

  /// Total cuts added (across all rounds).
  int64_t total_cuts() const { return cut_count.load(); }

  /// Increment cut count.
  void increment_cuts(int64_t by = 1) { cut_count.fetch_add(by); }

  /// Check if solve should terminate.
  bool should_terminate() const { return terminate_flag.load(); }

  /// Request graceful termination (time/node/gap limit reached).
  void request_termination() { terminate_flag.store(true); }

  /// Elapsed time in seconds.
  double elapsed_time() const;

  /// Remaining time budget (-1 if no limit).
  double remaining_time() const;

  // ─────────────────────────────────────────────────────────────────────
  // Algorithm Configuration
  // ─────────────────────────────────────────────────────────────────────

  /// Get algorithm options (read-only).
  const BCOptions& options() const { return opt; }

  /// Presolve statistics (if preprocessing applied).
  const SolveStats* presolve_stats() const {
    return presolve_stats_ptr;
  }

  // ─────────────────────────────────────────────────────────────────────
  // Result Assembly
  // ─────────────────────────────────────────────────────────────────────

  /// Build final SolveResult from context state.
  /// Called at termination to package incumbent, bounds, and stats.
  SolveResult build_result(const std::string& status_msg) const;

 private:
  // ─────────────────────────────────────────────────────────────────────
  // Problem Data
  // ─────────────────────────────────────────────────────────────────────

  const MIPModel& prob;  // Non-owning reference
  std::unique_ptr<StandardFormLP> sf;  // Lazy-built standard form
  detail::CliqueTable cliques;                 // Binary implication graph
  detail::BinaryImplicationGraph implications; // Root-probed implications

  // ── Row-cache for domain propagation ──────────────────────────────────
  mutable Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_cached_;
  mutable Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row_cached_;
  mutable detail::RowPropagationIndex row_index_cached_;
  mutable bool row_cache_built_{false};
  mutable std::mutex row_cache_mtx_;

  // ─────────────────────────────────────────────────────────────────────
  // Incumbent Solution
  // ─────────────────────────────────────────────────────────────────────

  std::atomic<bool> incumbent_found{false};
  std::atomic<double> incumbent_obj{std::numeric_limits<double>::infinity()};
  mutable std::mutex incumbent_mtx;
  Eigen::VectorXd incumbent_x;  // Protected by incumbent_mtx

  // ─────────────────────────────────────────────────────────────────────
  // Bounds & Termination
  // ─────────────────────────────────────────────────────────────────────

  std::atomic<double> dual_bound_{std::numeric_limits<double>::lowest()};
  std::atomic<bool> terminate_flag{false};

  // ─────────────────────────────────────────────────────────────────────
  // Pools
  // ─────────────────────────────────────────────────────────────────────

  detail::SharedCutPool cut_pool_;
  detail::SharedSolutionPool sol_pool_;
  detail::SharedConflictPool conflict_pool_;

  // ─────────────────────────────────────────────────────────────────────
  // Branching Statistics
  // ─────────────────────────────────────────────────────────────────────

  std::vector<detail::PseudoCost> pseudocosts_;
  mutable std::mutex pseudocost_mtx;
  std::vector<double> branch_priority;
  detail::CutFamilyTracker cut_tracker;

  // ─────────────────────────────────────────────────────────────────────
  // Solver Instances
  // ─────────────────────────────────────────────────────────────────────

  std::unique_ptr<detail::SolverDispatcher> simplex_dispatch;
  void* ipm_node;     // Non-owning (from registry, Phase 4.2 wiring)
  void* ipm_fallback; // Non-owning (from registry, Phase 4.2 wiring)
  void* ipm_cut;      // Non-owning (from registry, Phase 4.2 wiring)

  // ─────────────────────────────────────────────────────────────────────
  // Runtime Counters & Configuration
  // ─────────────────────────────────────────────────────────────────────

  std::atomic<int64_t> node_count{0};
  std::atomic<int64_t> lp_solve_count{0};
  std::atomic<int64_t> cut_count{0};

  const BCOptions& opt;  // Non-owning reference
  const SolveStats* presolve_stats_ptr;

  std::chrono::steady_clock::time_point start_time =
      std::chrono::steady_clock::now();
};

}  // namespace hacdcpf::engine::solver::native::milp::bc

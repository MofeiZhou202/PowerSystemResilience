/// @file bc_solver_dispatch.hpp
/// @brief Unified solver dispatcher for B&C node LP solves.
///
/// Consolidates all LP solving paths into a single entry point, replacing
/// ad-hoc solver selection scattered across the B&C codebase.
///
/// The dispatcher handles:
///   - Solver selection (warm-start vs cold-start, dense vs sparse)
///   - Tolerance configuration based on solve context
///   - Integration with FallbackManager for numerical failure recovery
///   - PDLP hybrid path for root LP
///
/// All external B&C code should call dispatcher.solve() or dispatcher.solve_root()
/// instead of calling solve_lp_from_sf / solve_lp_with_basis / solve_lp_relaxation
/// directly. Internal routing is handled by the dispatcher.

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>

#include <Eigen/Core>

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"

namespace mipsolvers::engine::detail {

/// @brief Context for a node LP solve — determines solver strategy.
enum class SolveContext {
  RootLP,        ///< Root node LP (may use PDLP hybrid)
  RootCutResolve,///< Re-solving after adding cuts at root
  NodeLP,        ///< Standard B&B node LP
  PoolCuts,      ///< Re-solve after adding pool cuts
  ChildCuts,     ///< Re-solve after generating cuts at child
  FeasPump,      ///< Feasibility pump LP
  LPDive,        ///< LP diving heuristic
  Probing,       ///< Strong branching / probing
};

/// @brief Configuration derived from BCOptions for the solver dispatcher.
struct DispatcherConfig {
  double lp_tol{1e-6};
  int max_lp_iter{500};
  bool use_simplex_lp_nodes{true};
  bool enable_fallback{true};

  // Derived tolerance settings (computed once).
  double feas_tol{1e-7};
  double opt_tol{1e-7};
  int base_max_iter{2000};

  // IPM/Simplex switching thresholds (Task 3.2).
  // These control when PDLP is preferred over simplex for node LPs.
  // When auto_solver_switch is false, the user-specified
  // use_simplex_lp_nodes flag is respected as-is.
  bool auto_solver_switch{false};          ///< Enable heuristic solver selection
  int pdlp_min_rows{500};                  ///< Min constraints for PDLP to be considered
  int pdlp_min_cols{200};                  ///< Min variables for PDLP to be considered
  double pdlp_max_density{0.05};           ///< Max nonzero density for PDLP (sparse only)
  int simplex_max_rows{2000};              ///< Above this, prefer PDLP if sparse enough
  int simplex_factor_backend{0};           ///< 0=A, 1=B (see SimplexFactorBackend)
  bool exact_dse_initialization{false};
  bool enable_degenerate_frontier_remap{false};
  bool suppress_degenerate_frontier_remap{false};
  bool disable_partial_pricing_for_conformance{false};
  bool allow_persistent_lp_state{false};
  LpKernelBackend lp_kernel_backend{LpKernelBackend::HiGHS};
  double time_limit_sec{0.0};
  bool* time_limit_hit{nullptr};

  static DispatcherConfig from_bc_options(const BCOptions& opt) {
    DispatcherConfig c;
    c.lp_tol = opt.lp_tol;
    c.max_lp_iter = opt.max_lp_iter;
    c.use_simplex_lp_nodes = opt.use_simplex_lp_nodes;
    c.simplex_factor_backend = opt.simplex_factor_backend;
    c.lp_kernel_backend = opt.lp_kernel_backend;
    c.enable_fallback = opt.enable_lp_fallback;
    c.feas_tol = std::max(1e-10, opt.lp_tol * 0.1);
    c.opt_tol = std::max(1e-10, opt.lp_tol * 0.1);
    c.base_max_iter = std::max(opt.max_lp_iter * 10, 2000);
    return c;
  }

  /// @brief Heuristic: should PDLP be used given problem dimensions?
  /// Only returns true when auto_solver_switch is enabled and the problem
  /// characteristics favor a first-order method over factorization.
  bool should_use_pdlp(int rows, int cols, int nnz) const {
    if (!auto_solver_switch) return false;
    if (rows < pdlp_min_rows || cols < pdlp_min_cols) return false;
    const double density = (rows > 0 && cols > 0)
        ? static_cast<double>(nnz) / (static_cast<double>(rows) * cols)
        : 1.0;
    if (density > pdlp_max_density) return false;
    // Large sparse LP: prefer PDLP
    return rows > simplex_max_rows;
  }
};

/// @brief Unified LP solver dispatcher for the Branch & Cut framework.
///
/// Single entry point for all LP solves. Handles solver selection,
/// tolerance configuration, and fallback recovery.
class SolverDispatcher {
 public:
  /// @param config Dispatcher configuration derived from BCOptions.
  /// @param fallback Fallback manager for numerical failure recovery. May be null to disable.
  /// @param incumbent_bound Pointer to atomic incumbent for early LP termination (parallel). May be null.
  SolverDispatcher(const DispatcherConfig& config,
                   FallbackManager* fallback,
                   const std::atomic<double>* incumbent_bound = nullptr)
      : config_(config), fallback_(fallback) {
    (void)incumbent_bound;
  }

  void set_root_basis(std::shared_ptr<const SimplexBasis> rb) { root_basis_ = std::move(rb); }

  /// Apply the caller's current wall-clock budget to the next dispatched LP.
  /// The shared hit flag prevents fallback stages from restarting after the
  /// global deadline has already interrupted the primary solve.
  void set_time_limit(double time_limit_sec, bool* time_limit_hit) {
    config_.time_limit_sec = time_limit_sec;
    config_.time_limit_hit = time_limit_hit;
  }

  /// @brief Solve an LP at a B&C node.
  /// @param sf Pre-built standard-form LP (bounds already updated).
  /// @param basis_hint Warm-start basis from parent (nullptr for cold-start).
  /// @param context The solve context (determines tolerance/strategy).
  /// @param node_id Node identifier for fallback logging.
  /// @param node_depth Node depth for fallback logging.
  /// @param expected_n Expected solution dimension.
  /// @param[out] deferred Set to true if node should be deferred (L4 fallback).
  /// @return SimplexResult from the solve.
  SimplexResult solve(StandardFormLP& sf,
                      const SimplexBasis* basis_hint,
                      SolveContext context,
                      int node_id, int node_depth,
                      int expected_n,
                      bool& deferred) {
    deferred = false;
    SimplexOptions opts = make_options(context);
    SimplexBasis effective_basis;
    const SimplexBasis* effective_basis_hint = prepare_basis_hint(
        sf, basis_hint, context, effective_basis);

    const auto solve_start = std::chrono::steady_clock::now();
    auto result = solve_lp_from_sf(sf, opts, effective_basis_hint);
    auto failure_type = classify_lp_result(result, expected_n);

    if (failure_type == LPFailureType::Success ||
        failure_type == LPFailureType::ObjectiveCutoff ||
        failure_type == LPFailureType::Infeasible) {
      retain_persistent_owner(sf, context, result);
      return result;
    }
    if (lp_wall_time_limit_reached(result) || time_limit_hit(opts)) {
      return result;
    }

    // Attempt fallback recovery if enabled.
    if (fallback_ && config_.enable_fallback) {
      if (!deduct_elapsed_time(opts, solve_start)) {
        return mark_time_limit(std::move(result));
      }
      auto recovered = fallback_->handle_failure(
          sf, effective_basis_hint, failure_type, node_id, node_depth,
          expected_n, deferred, &opts);
      retain_persistent_owner(sf, context, recovered);
      return recovered;
    }

    return result;  // No fallback — return failed result
  }

  /// @brief Solve without fallback (used for heuristic LPs where failure is acceptable).
  SimplexResult solve_no_fallback(const StandardFormLP& sf,
                                  const SimplexBasis* basis_hint,
                                  SolveContext context) {
    SimplexOptions opts = make_options(context);
    SimplexBasis effective_basis;
    const SimplexBasis* effective_basis_hint = prepare_basis_hint(
        sf, basis_hint, context, effective_basis);
    auto result = solve_lp_from_sf(sf, opts, effective_basis_hint);
    retain_persistent_owner(sf, context, result);
    return result;
  }

  /// @brief Solve with caller-provided options and fallback recovery.
  /// Use when caller needs custom SimplexOptions (e.g., depth-dependent iter limits)
  /// but still wants fallback recovery on numerical failure.
  SimplexResult solve_with_options(StandardFormLP& sf,
                                   const SimplexBasis* basis_hint,
                                   const SimplexOptions& opts,
                                   int node_id, int node_depth,
                                   int expected_n,
                                   bool& deferred) {
    deferred = false;
    SimplexOptions effective_opts = opts;
    effective_opts.time_limit_sec = config_.time_limit_sec;
    effective_opts.time_limit_hit = config_.time_limit_hit;
    const auto solve_start = std::chrono::steady_clock::now();
    auto result = solve_lp_from_sf(sf, effective_opts, basis_hint);
    auto failure_type = classify_lp_result(result, expected_n);
    if (failure_type == LPFailureType::Success ||
      failure_type == LPFailureType::ObjectiveCutoff ||
      failure_type == LPFailureType::Infeasible) return result;
    if (lp_wall_time_limit_reached(result) || time_limit_hit(effective_opts)) {
      return result;
    }
    if (fallback_ && config_.enable_fallback) {
      if (!deduct_elapsed_time(effective_opts, solve_start)) {
        return mark_time_limit(std::move(result));
      }
      return fallback_->handle_failure(sf, basis_hint, failure_type,
                                       node_id, node_depth, expected_n,
                                       deferred, &effective_opts);
    }
    return result;
  }

  /// @brief Build SimplexOptions appropriate for the given context.
  SimplexOptions make_options(SolveContext context) const {
    SimplexOptions opts;
    opts.feasibility_tol = config_.feas_tol;
    opts.optimality_tol = config_.opt_tol;
    opts.verbose = false;
    opts.factor_backend = simplex_factor_backend_from_id(config_.simplex_factor_backend);
    opts.exact_dse_initialization = config_.exact_dse_initialization;
    opts.enable_degenerate_frontier_remap =
        config_.enable_degenerate_frontier_remap;
    opts.suppress_degenerate_frontier_remap =
        config_.suppress_degenerate_frontier_remap;
    opts.lp_kernel_backend = config_.lp_kernel_backend;
    opts.allow_persistent_lp_state = config_.allow_persistent_lp_state;
    opts.time_limit_sec = config_.time_limit_sec;
    opts.time_limit_hit = config_.time_limit_hit;
    if (config_.disable_partial_pricing_for_conformance) {
      opts.use_partial_pricing = false;
      opts.perturb_degenerate_primal = false;
    }

    switch (context) {
      case SolveContext::RootLP:
      case SolveContext::RootCutResolve:
        opts.max_iter = config_.base_max_iter;
        opts.allow_cold_start = true;
        break;
      case SolveContext::NodeLP:
        opts.max_iter = config_.base_max_iter;
        opts.allow_cold_start = true;
        // Node LPs feed the branch-and-bound proof frontier.  Do not early
        // terminate on the incumbent here: the intermediate cutoff objective
        // is not a certified node lower bound and can falsely prune siblings.
        opts.incumbent_bound = nullptr;
        if (root_basis_) opts.fallback_basis = root_basis_.get();
        break;
      case SolveContext::PoolCuts:
      case SolveContext::ChildCuts:
        opts.max_iter = config_.base_max_iter;
        opts.allow_cold_start = false;
        break;
      case SolveContext::FeasPump:
        opts.max_iter = std::max(config_.max_lp_iter * 5, 1000);
        opts.feasibility_tol = std::max(1e-7, config_.feas_tol);
        opts.optimality_tol = std::max(1e-7, config_.opt_tol);
        opts.allow_cold_start = true;
        break;
      case SolveContext::LPDive:
        opts.max_iter = std::max(config_.max_lp_iter * 5, 1000);
        opts.allow_cold_start = false;
        break;
      case SolveContext::Probing:
        opts.max_iter = std::max(config_.max_lp_iter * 5, 1000);
        opts.allow_cold_start = false;
        break;
    }
    return opts;
  }

 private:
  const SimplexBasis* prepare_basis_hint(
      const StandardFormLP& sf,
      const SimplexBasis* basis_hint,
      SolveContext context,
      SimplexBasis& effective_basis) const {
    if (basis_hint == nullptr || context != SolveContext::NodeLP ||
        !config_.allow_persistent_lp_state) {
      return basis_hint;
    }
    effective_basis = *basis_hint;
    effective_basis.cached_sparse_basis.reset();
    if (persistent_lp_owner_ && persistent_structure_id_ == sf.structure_id) {
      effective_basis.cached_sparse_basis = persistent_lp_owner_;
    }
    return &effective_basis;
  }

  void retain_persistent_owner(const StandardFormLP& sf,
                               SolveContext context,
                               SimplexResult& result) {
    if (context != SolveContext::NodeLP ||
        !config_.allow_persistent_lp_state ||
        !result.basis.cached_sparse_basis) {
      return;
    }
    // Keep one structure-stable owner per dispatcher. Local-cut LPs may have
    // different matrices; they must not evict the main node LP owner.
    if (!persistent_lp_owner_ || persistent_structure_id_ == sf.structure_id) {
      persistent_lp_owner_ = result.basis.cached_sparse_basis;
      persistent_structure_id_ = sf.structure_id;
    }
    // Nodes may be shared by siblings or queues. Never publish the mutable LP
    // owner through their basis snapshots.
    result.basis.cached_sparse_basis.reset();
    result.basis.persist_eta_count = 0;
  }

  static bool time_limit_hit(const SimplexOptions& opts) {
    return opts.time_limit_hit != nullptr && *opts.time_limit_hit;
  }

  static bool deduct_elapsed_time(
      SimplexOptions& opts,
      const std::chrono::steady_clock::time_point& start) {
    if (!(opts.time_limit_sec > 0.0) ||
        !std::isfinite(opts.time_limit_sec)) {
      return true;
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    opts.time_limit_sec -= elapsed;
    if (opts.time_limit_sec > 0.0) return true;
    if (opts.time_limit_hit != nullptr) *opts.time_limit_hit = true;
    return false;
  }

  static SimplexResult mark_time_limit(SimplexResult result) {
    result.result.stats.success = false;
    result.result.stats.status =
        "Time limit: LP dispatcher budget exhausted";
    return result;
  }

  DispatcherConfig config_;
  FallbackManager* fallback_;
  std::shared_ptr<const SimplexBasis> root_basis_;
  std::shared_ptr<BasisOps> persistent_lp_owner_;
  std::uint64_t persistent_structure_id_{0};
};

}  // namespace mipsolvers::engine::detail

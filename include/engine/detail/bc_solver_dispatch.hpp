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
#include <memory>

#include <Eigen/Core>

#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/detail/bc_fallback.hpp"

namespace hacdcpf::engine::detail {

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
  bool allow_vendored_highs_sf_backend{false};

  static DispatcherConfig from_bc_options(const BCOptions& opt) {
    DispatcherConfig c;
    c.lp_tol = opt.lp_tol;
    c.max_lp_iter = opt.max_lp_iter;
    c.use_simplex_lp_nodes = opt.use_simplex_lp_nodes;
    c.simplex_factor_backend = opt.simplex_factor_backend;
    c.allow_vendored_highs_sf_backend = opt.use_vendored_highs_lp_kernel;
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

    auto result = solve_lp_from_sf(sf, opts, basis_hint);
    auto failure_type = classify_lp_result(result, expected_n);

    if (failure_type == LPFailureType::Success ||
        failure_type == LPFailureType::ObjectiveCutoff ||
        failure_type == LPFailureType::Infeasible) {
      return result;
    }

    // Attempt fallback recovery if enabled.
    if (fallback_ && config_.enable_fallback) {
      return fallback_->handle_failure(sf, basis_hint, failure_type,
                                       node_id, node_depth, expected_n,
                                       deferred);
    }

    return result;  // No fallback — return failed result
  }

  /// @brief Solve without fallback (used for heuristic LPs where failure is acceptable).
  SimplexResult solve_no_fallback(const StandardFormLP& sf,
                                  const SimplexBasis* basis_hint,
                                  SolveContext context) {
    SimplexOptions opts = make_options(context);
    return solve_lp_from_sf(sf, opts, basis_hint);
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
    auto result = solve_lp_from_sf(sf, opts, basis_hint);
    auto failure_type = classify_lp_result(result, expected_n);
    if (failure_type == LPFailureType::Success ||
      failure_type == LPFailureType::ObjectiveCutoff ||
      failure_type == LPFailureType::Infeasible) return result;
    if (fallback_ && config_.enable_fallback) {
      return fallback_->handle_failure(sf, basis_hint, failure_type,
                                       node_id, node_depth, expected_n,
                                       deferred);
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
    opts.allow_vendored_highs_sf_backend =
        config_.allow_vendored_highs_sf_backend;
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
        opts.reuse_factorization = false;
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
  DispatcherConfig config_;
  FallbackManager* fallback_;
  std::shared_ptr<const SimplexBasis> root_basis_;
};

}  // namespace hacdcpf::engine::detail

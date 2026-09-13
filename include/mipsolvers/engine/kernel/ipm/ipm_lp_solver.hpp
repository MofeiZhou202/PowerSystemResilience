#pragma once

#include <atomic>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

class LpPresolveActivitySnapshot;
struct LpPresolveMatrixWorkspace;

/// Algebraically equivalent Newton systems available to the LP IPM.
/// Auto compares CHOLMOD symbolic work and fill estimates, then monitors the
/// accepted Newton directions so a numerically weak normal-equations
/// trajectory can be restarted from the original initial point.
enum class IPMNewtonFormulation {
  Auto,
  ForceNormal,
  ForceAugmented,
};

/// Options for the high-performance Interior Point Method LP solver.
///
/// This is a Mehrotra predictor-corrector IPM designed for general LP:
///   min c'x  s.t.  A x <= b,  Aeq x = beq,  lb <= x <= ub
///
/// Key features over LCQP solver:
///   - Full general inequality support (A x <= b via slack variables)
///   - Ruiz + Pock-Chambolle equilibration for better conditioning
///   - Mehrotra predictor-corrector with Gondzio correctors
///   - Multiple centrality corrector steps for faster convergence
///   - Crossover to basic feasible solution for MILP warm-starting
///   - Constraint duals returned for cut generation
///
/// Suitable as the LP relaxation solver in Branch & Cut.
struct IPMLPOptions {
  int max_iter{200};          ///< Maximum IPM iterations
  double time_limit_sec{0.0}; ///< Wall-clock limit for this LP solve (0 disables)
  double tol_primal{1e-8};   ///< Primal feasibility tolerance
  double tol_dual{1e-8};     ///< Dual feasibility tolerance
  double tol_gap{1e-8};      ///< Complementarity gap tolerance
  int ruiz_rounds{10};       ///< Ruiz equilibration rounds (0 = disable)
  // Base Mehrotra uses two solves per factorization. Auto retains Gondzio on
  // cache-resident factors and removes extra factor streams on large factors
  // (§7.4 of docs/archive/native_presolve_lp_2026-08-18.md; Anjos et al. 2020,
  // §6.2). Zero disables; a positive value is an explicit per-iteration cap.
  int max_correctors{-1};    ///< -1=auto, 0=off, >0=max Gondzio correctors
  /// Keep blocking complementarity products away from zero with the adaptive
  /// primal/dual step rule used by modern bounded-variable IPMs.  Disable only
  /// for controlled comparison with the legacy fixed 0.9995 boundary fraction.
  bool centrality_step_control{true};
  bool presolve{true};       ///< Enable presolve (fixed vars, singleton rows, etc.)
  bool crossover{false};     ///< Purify solution to a vertex (for simplex warm-start)
  bool verbose{false};       ///< Print per-iteration log
  /// Opt-in: run adaptive HiGHS presolve before the IPM, solve the reduced LP,
  /// then postsolve the primal back to original space (primal-only, no basis
  /// needed).  Only applied on cold solves (no warm-start).  On any failure the
  /// solver falls back to a direct solve, so a wrong or infeasible answer is
  /// never published.  MIPSOLVERS_PRESOLVE* env vars override the effective
  /// config (see highs_lp_presolve_config_from_env).
  bool use_highs_presolve{false};
  IPMNewtonFormulation newton_formulation{IPMNewtonFormulation::Auto};
  // Cooperative external abort polled once per IPM iteration. When set, the
  // solve stops promptly; used by the LP portfolio to cancel the losing kernel.
  // nullptr (default) leaves the solve unchanged.
  const std::atomic<bool>* cancel_flag{nullptr};
};

/// Original-model KKT audit for a bounded-variable LP solution.
///
/// Dual inputs use the minimization convention, regardless of LPModel::sense:
/// stationarity is c_min - A' y - z_l + z_u = 0, and an upper-only row has
/// y <= 0. This convention is also used internally by the native LP IPM.
struct IPMLPOptimalityAudit {
  bool valid{false};
  double primal_residual_inf{std::numeric_limits<double>::infinity()};
  double dual_residual_inf{std::numeric_limits<double>::infinity()};
  double relative_primal_residual{std::numeric_limits<double>::infinity()};
  double relative_dual_residual{std::numeric_limits<double>::infinity()};
  double primal_objective{std::numeric_limits<double>::quiet_NaN()};
  double dual_objective{std::numeric_limits<double>::quiet_NaN()};
  double relative_gap{std::numeric_limits<double>::infinity()};

  bool acceptable(double primal_tolerance, double dual_tolerance,
                  double gap_tolerance) const;
};

/// Recompute primal feasibility, dual feasibility, and the duality gap in the
/// unscaled LPModel coordinates. Optional bound overrides support cached node
/// LPs without copying the constraint matrices. objective_offset is the
/// original-sense constant from an exact presolve substitution; it shifts both
/// primal and dual objectives before the relative gap is normalized (A&A 1995
/// Section 2.4; native_presolve_lp_2026-08-18.md Section 8.18). Exposed for
/// numerical tests.
IPMLPOptimalityAudit audit_ipm_lp_optimality(
    const LPModel& lp, const Eigen::VectorXd& x,
    const Eigen::VectorXd& row_duals_min,
    const Eigen::VectorXd& box_dual_lb_min,
    const Eigen::VectorXd& box_dual_ub_min,
    const Eigen::VectorXd* lower_bounds_override = nullptr,
    const Eigen::VectorXd* upper_bounds_override = nullptr,
    double objective_offset = 0.0);

/// Forward declaration for Apple Accelerate sparse Cholesky cache.
struct AccelSparseCache;

/// One structure-preserving node-domain change for a cached IPM batch.
/// Bounds are intersected with the shared parent domain.
struct IPMNodeBoundChange {
  int variable{-1};
  double lower_bound{-std::numeric_limits<double>::infinity()};
  double upper_bound{std::numeric_limits<double>::infinity()};
};

struct IPMNodeBatchEntry {
  bool attempted{false};
  SolveResult result;
};

/// Native high-performance IPM for LP.
///
/// Converts general LP (inequality + equality + box) to standard form
/// internally and solves via Mehrotra predictor-corrector with Gondzio
/// centering corrections.  Returns constraint duals and Farkas certificates.
class NativeIPMLPAdapter final : public SolverAdapter {
 public:
  explicit NativeIPMLPAdapter(IPMLPOptions opt = {});
  ~NativeIPMLPAdapter();

  std::string name() const override;
  bool supports(ProblemClass cls) const override;

  SolveResult solve_lp(const LPModel& prob) const override;
  SolveResult solve_lp(const LPModel& prob,
                       const SolveContext& context) const override;

  /// Auto entry point carrying the estimator's original-box row activities
  /// directly into native presolve (design Section 8.30).
  SolveResult solve_lp(
      const LPModel& prob,
      const std::shared_ptr<const LpPresolveActivitySnapshot>&
          activity_snapshot) const;

  /// Solve LP with warm-start from a previous primal solution.
  /// When x0 is provided, the IPM uses it as initial primal point instead
  /// of the expensive least-squares initialization, typically reducing
  /// iteration count by 30-50%.
  SolveResult solve_lp(const LPModel& prob, const Eigen::VectorXd& x0) const;

  /// Solve a node LP: base_lp with overridden variable bounds.
  /// More efficient than copying the LPModel: directly uses node_lb/node_ub
  /// instead of modifying the vars array.
  SolveResult solve_node_lp(const LPModel& base_lp,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub) const;

  /// Solve a node LP with warm-start from parent node's solution.
  SolveResult solve_node_lp(const LPModel& base_lp,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub,
                            const Eigen::VectorXd& x0) const;

  /// Solve against prepared structure while selecting the linear-system path
  /// that matches it. Dense-coupling models use the augmented KKT kernel;
  /// narrow/sparse-normal models use the regular cached node path.
  SolveResult solve_structure_aware_node_lp(
      const Eigen::VectorXd& node_lb,
      const Eigen::VectorXd& node_ub,
      const Eigen::VectorXd& x0) const;

  /// Prepare cached solver state for repeated node solves on the same LP.
  /// Caches CSR, scatter maps, symbolic factorization — only bounds change.
  void prepare_for_node_solves(const LPModel& base_lp);

  /// Fast node LP solve using cached infrastructure from prepare_for_node_solves.
  /// Only updates bounds and warm-start, skipping all structural setup.
  SolveResult solve_cached_node_lp(const Eigen::VectorXd& node_lb,
                                    const Eigen::VectorXd& node_ub,
                                    const Eigen::VectorXd& x0) const;

  /// Solve one-variable bound changes over a shared parent domain. The batch
  /// reuses one bounds buffer and the prepared structural cache. Its wall
  /// budget is divided over entries that have not yet been attempted.
  std::vector<IPMNodeBatchEntry> solve_cached_bound_change_batch(
      const Eigen::VectorXd& parent_lb,
      const Eigen::VectorXd& parent_ub,
      const Eigen::VectorXd& x0,
      const std::vector<IPMNodeBoundChange>& changes,
      double batch_time_limit_sec);

  /// Update the cached cost vector (for feasibility pump objective changes).
  /// new_c is in the original LP convention (sense_sign applied internally).
  /// Length must be n_orig. Slack costs remain 0.
  void update_cached_cost(const Eigen::VectorXd& new_c);

  /// Set the wall-clock limit applied independently to subsequent solves.
  /// Updating this does not invalidate cached structural state.
  void set_solve_time_limit(double time_limit_sec);

 private:
  enum class AugmentedBackendPolicy {
    StructurePreserving,
    PivotingPortfolio,
  };

  /// Core solve with an explicit Ruiz round count.  The public entry points
  /// retry with scaling disabled when a scaled solve fails to converge
  /// (scaling helps most problems but stalls some degenerate ones).
  /// publication_tol_scale tightens the original-model publication audit
  /// below the user's tolerance contract; the native presolve wiring uses it
  /// so a reduced-model solution publishes strictly inside the outer
  /// original-model audit envelope (native_presolve_lp_2026-08-18.md §6 P1).
  SolveResult solve_lp_impl(const LPModel& prob, const Eigen::VectorXd& x0,
                            int ruiz_rounds,
                            double time_limit_sec,
                            AugmentedBackendPolicy backend_policy,
                            IPMNewtonFormulation formulation,
                            double publication_tol_scale = 1.0,
                            double objective_offset = 0.0,
                            const std::shared_ptr<const
                                LpPresolveMatrixWorkspace>& matrix_workspace =
                                {},
                            const char* entry_reason = "primary",
                            int source_iterations = 0) const;

  SolveResult solve_lp_with_presolve_snapshot(
      const LPModel& prob, const Eigen::VectorXd& x0,
      const std::shared_ptr<const LpPresolveActivitySnapshot>&
          activity_snapshot) const;

  IPMLPOptions opt_;
  mutable std::unique_ptr<AccelSparseCache> accel_cache_;

  /// Cached solver state for repeated node solves.
  struct CachedState;
  mutable std::unique_ptr<CachedState> cached_state_;
};

}  // namespace mipsolvers::engine

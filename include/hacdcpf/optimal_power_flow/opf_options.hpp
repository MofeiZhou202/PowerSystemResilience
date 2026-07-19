#pragma once

/// optimal_power_flow/opf_options.hpp
/// =====================================
/// Options for AC OPF, DC OPF, and combined OPF solvers.
/// Consolidates: ac_opf.hpp (ACOPFOptions) + dc_opf.hpp (DCOPFOptions).

#include <string>

namespace hacdcpf::opf {

struct ACOPFResult;

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Solver Backend
// ═══════════════════════════════════════════════════════════════════════
/// Selects which nonlinear-OPF engine `solve_ac_opf` uses.
///
/// - `Auto`             : parity full-space IPM first, then Ipopt as a fallback
///                        if the parity IPM does not converge (and Ipopt is
///                        compiled in).  Pure-AC cases may still use the fast
///                        economic-dispatch path only when explicitly selected.
/// - `ParityIPM`        : the self-developed full-space primal-dual IPM.
/// - `Ipopt`            : the embedded Ipopt (filter line-search) NLP solver,
///                        applied to the same parity formulation.
/// - `EconomicDispatch` : merit-order economic dispatch + a single AC power
///                        flow.  Fast but suboptimal; pure-AC systems only.
enum class ACOPFSolverBackend {
  Auto,
  ParityIPM,
  Ipopt,
  EconomicDispatch
};

/// Continuous objective used by the full-space AC/DC OPF formulation.
/// Economic remains the default for ordinary OPF.  The other modes are used
/// by reactive-power optimisation so that voltage/loss terms are optimised by
/// the inner NLP, rather than merely evaluated after an economic OPF solve.
enum class ACOPFObjective {
  Economic,
  VoltageDeviation,
  ActiveLoss,
  VoltageDeviationAndLoss
};

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Options
// ═══════════════════════════════════════════════════════════════════════
struct ACOPFOptions {
  /// Preferred nonlinear-OPF backend.  When left at `Auto`, the legacy
  /// `use_parity_ipm` / `enable_primal_dual` flags continue to control path
  /// selection (with an Ipopt fallback for hybrid AC/DC and large cases).
  ACOPFSolverBackend ac_solver_backend{ACOPFSolverBackend::Auto};

  ACOPFObjective objective{ACOPFObjective::Economic};
  double voltage_target_pu{1.0};
  double voltage_deviation_weight{1.0};
  double active_loss_weight{1.0};

  int max_inner_iterations{80};
  int max_outer_iterations{8};
  int max_line_search_steps{20};

  double feasibility_tol{1e-6};
  double stationarity_tol{1e-6};

  double barrier_mu0{1e-2};
  double barrier_mu_reduction{0.2};
  double barrier_mu_min{1e-8};

  double merit_penalty{10.0};
  double regularization{1e-6};
  double step_backoff{0.5};
  double interior_fraction{0.995};

  int ac_eval_threads{1};
  bool enable_primal_dual{false};
  bool use_parity_ipm{false};
  bool allow_fallback{true};
  bool verbose{false};

  /// Optional non-owning warm start from a previous compatible OPF solve.
  /// The pointed result must remain alive until solve_ac_opf() returns.  Each
  /// variable family is mapped independently; incompatible/missing families
  /// fall back to the normal physics-informed initial point.
  const ACOPFResult* warm_start{nullptr};

  /// When true (and no explicit warm_start is given), first solve the plain
  /// AC power flow from the case's own operating point and seed the parity
  /// IPM with the AC-feasible (vm, va) — θ ≈ 0 at iteration 0, so the IPM
  /// stays in the feasible basin instead of searching for it.  Measured to
  /// unlock the stressed PEGASE grids (case13659pegase converges at the
  /// reference optimum in 189 iterations / ~40 s).  On well-posed cases the
  /// physics-informed initial point is faster (centrality beats exact
  /// feasibility at init), so this is opt-in, not default.
  bool ac_pf_warm_start{false};

  /// When true, compute the Davidenko objective-homotopy tangent dw/dt at
  /// the returned point (one extra inertia-controlled KKT solve) and expose
  /// it in the result's ipm_tangent_* fields.  homotopy_t supplies the cost
  /// scale t of the problem being solved (the tangent rhs is ∇f/t).
  /// Objective-continuation drivers use it for Davidenko prediction.
  bool compute_homotopy_tangent{false};
  double homotopy_t{1.0};

  /// Theory-guided two-phase solve (docs/numerical_methods.md §12–13): follow
  /// the objective homotopy P(t) = min t·f from t = 0 (feasibility problem)
  /// to t = 1 (full cost), carrying the full primal-dual state between steps
  /// with Davidenko tangent prediction and an adaptive parameter-metric step.
  /// Measured to certify the stressed PEGASE grids at their reference optima
  /// (2869 exact, 9241 −0.013%, 13659 via ac_pf_warm_start).  homotopy_dt0 is
  /// the initial continuation step; negative/zero falls back to 0.10.
  bool objective_homotopy{false};
  double homotopy_dt0{0.10};

  // Constraint-family toggles for the hybrid parity-IPM formulation
  // (multi-converter model §3.1).  Default true keeps the always-enforce
  // behavior; a GUI/caller can disable a family to relax the problem.
  bool enforce_branch_limits{true};
  bool enforce_converter_capacity{true};
  bool enforce_converter_current_limits{true};
  bool enforce_converter_modulation_limits{true};
};

// ═══════════════════════════════════════════════════════════════════════
// DC OPF Solver Backend
// ═══════════════════════════════════════════════════════════════════════
enum class DCOPFSolverBackend {
  Auto,
  Native,
  NativeQP,
  HiGHS,
  Gurobi
};

// ═══════════════════════════════════════════════════════════════════════
// DC OPF Options
// ═══════════════════════════════════════════════════════════════════════
struct DCOPFOptions {
  double feasibility_tol{1e-6};
  double optimality_tol{1e-6};
  int max_iterations{10000};
  bool verbose{false};

  DCOPFSolverBackend solver{DCOPFSolverBackend::Auto};

  int pwl_segments{4};

  bool include_branch_limits{true};
  double branch_limit_margin{1.0};

  bool load_shedding{true};
  double voll{0.0};

  // When true (default), the solver runs a supporting simplex LP after the
  // primal solve to recover constraint dual variables (LMPs and congestion
  // prices).  Set to false when only the primal dispatch is needed and the
  // extra LP solve overhead should be avoided.
  bool compute_lmp{true};
};

}  // namespace hacdcpf::opf

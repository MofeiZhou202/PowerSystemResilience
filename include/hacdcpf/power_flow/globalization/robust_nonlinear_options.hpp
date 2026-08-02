#pragma once

#include <string>

namespace hacdcpf::powerflow {

/// Configuration for the robust nonlinear solver enhancement pipeline.
///
/// Consumed in phases by the Newton solver (and homotopy / LM fallback).
/// All Phase 1 features are enabled by default because the Newton solver
/// already has the infrastructure wired in; Phases 2-5 provide additional
/// fallbacks and can be disabled individually for benchmarking.
struct RobustNonlinearOptions {

  // ── Phase 1: Scaling and diagnostics ──────────────────────────────
  /// Scale residual equations by their nominal magnitudes (max|F_i^spec|, 1).
  /// Convergence check uses the scaled ℓ∞ norm.
  bool enable_residual_scaling{true};

  /// Scale Newton variables (θ, V, Vdc) by nominal magnitudes so that the
  /// scaled step is dimensionless and O(1).
  bool enable_variable_scaling{true};

  /// Apply D_F·J·D_x⁻¹ row/column equilibration to the Jacobian before
  /// factorisation (requires at least one of the scaling flags above).
  bool enable_jacobian_row_col_equilibration{true};

  /// Compute a cheap ℓ₁-row/col proxy for the Jacobian condition number and
  /// store it in SolverProfiling::condition_estimate each iteration.
  bool enable_condition_monitor{true};

  /// Floor for any scale factor (prevents division-by-zero in degenerate cases).
  double min_scale{1e-6};

  /// Ceiling for any scale factor.
  double max_scale{1e6};

  /// Voltage magnitude floor (pu) used in Jacobian ∂P/∂V and ∂Q/∂V entries
  /// to prevent division by zero when Vm collapses near zero.
  double min_vm_pu{1e-8};

  /// Minimum branch reactance |X_pu| below which a branch is treated as a
  /// zero-reactance switch and excluded from the FDPF B' matrix.
  double min_branch_x_pu{1e-20};

  /// Proxy condition number above which the Jacobian is flagged "bad"
  /// (used to trigger LM / PTC fallback when combined with other indicators).
  double bad_condition_threshold{1e10};

  /// Absolute pivot magnitude below which a pivot is considered tiny;
  /// used for informational logging only (regularisation is controlled
  /// separately via PowerFlowOptions::regularization_lambda0).
  double tiny_pivot_threshold{1e-12};

  // ── Phase 2: Nonmonotone line search ──────────────────────────────
  /// Replace standard monotone backtracking with a nonmonotone Armijo search
  /// (Grippo–Lampariello–Lucidi style).  Accepts a step if the merit function
  /// is below max(φ_{k-j}) over the last 'nonmonotone_window' iterates.
  bool enable_nonmonotone_linesearch{true};

  /// Sliding window M for the nonmonotone reference merit φ_k^{max}.
  int nonmonotone_window{5};

  /// Armijo sufficient-decrease constant c₁ ∈ (0, 1).
  double armijo_c{1e-4};

  /// Step-length reduction factor β ∈ (0, 1) per backtrack trial.
  double line_search_beta{0.5};

  /// Maximum number of backtrack trials before the step is rejected.
  int max_line_search_trials{12};

  // ── Phase 2: Smooth Fischer–Burmeister NCP continuation ───────────
  /// Use the μ-perturbed smooth-FB function φ_μ(a,b) = √(a²+b²+2μ)−a−b
  /// for PV/PQ complementarity.  μ is annealed from ncp_mu0 → ncp_mu_min
  /// as the residual decreases, driving toward the exact NCP solution.
  /// Disabled by default to preserve the original semi-smooth Newton behavior;
  /// enable explicitly for difficult cases with near-active Q limits.
  bool enable_smooth_ncp{false};

  /// Initial smoothing parameter μ₀ ≫ 0.
  double ncp_mu0{1e-2};

  /// Minimum smoothing parameter (limit of the continuation).
  double ncp_mu_min{1e-12};

  /// Multiplicative reduction applied to μ when the residual decreases
  /// sufficiently (see implementation for the exact schedule).
  double ncp_mu_factor{0.1};

  /// Slower annealing factor used in the coarse phase (residual still large).
  /// A two-phase schedule is applied:
  ///   - if scaled_resid > ncp_mu_phase_transition * resid_initial → ncp_mu_factor_coarse
  ///   - otherwise (fine phase, near-convergence)                  → ncp_mu_factor
  /// Larger value (e.g. 0.5) keeps μ larger during early iterations when the
  /// active-set structure has not yet stabilised, preventing oscillation near
  /// strongly active Q limits.
  double ncp_mu_factor_coarse{0.5};

  /// Fraction of the initial residual below which μ annealing switches from
  /// the coarse factor to the fine (fast) factor.
  double ncp_mu_phase_transition{0.1};

  /// Prevent chattering by holding the PV/PQ active set for at least
  /// min_active_set_hold_iters iterations before allowing a switch.
  bool enable_activity_hysteresis{true};

  /// Minimum iterations between consecutive PV/PQ status changes.
  int min_active_set_hold_iters{3};

  /// Hysteresis band on Q-limit proximity (pu) before switching PV→PQ.
  double q_limit_hysteresis{1e-4};

  // ── Phase 3: Levenberg–Marquardt trust-region fallback ────────────
  /// When the Newton line search fails (or the Jacobian is flagged bad),
  /// compute an LM step that minimises ½‖F‖₂² via (JᵀJ + λI)Δx = −JᵀF.
  bool enable_lm_trust_region_fallback{true};

  /// Initial LM damping parameter λ₀.
  double lm_lambda0{1e-4};

  /// Minimum LM damping (near-Newton step when the model is highly accurate).
  double lm_lambda_min{1e-12};

  /// Maximum LM damping (prevents the step from collapsing to zero).
  double lm_lambda_max{1e8};

  /// Initial trust-region radius (normalised units) for the LM step.
  double trust_region_radius0{1.0};

  /// Minimum trust-region radius; below this the LM fallback is abandoned.
  double trust_region_radius_min{1e-6};

  /// Maximum trust-region radius.
  double trust_region_radius_max{100.0};

  // ── Phase 3: PTC-SER adaptive pseudo-transient continuation ──────
  /// Upgrade the existing PTC mode to use the Self-Equalising Residual (SER)
  /// exponent γ and clamped δt bounds instead of a fixed growth factor.
  bool enable_ptc_ser{true};

  /// Initial pseudo-timestep δt₀.
  double ptc_dt0{0.1};

  /// Minimum pseudo-timestep (large damping; slow but robust).
  double ptc_dt_min{1e-4};

  /// Maximum pseudo-timestep (approaches pure Newton as δt → ∞).
  double ptc_dt_max{1e6};

  /// SER exponent γ: δt_{k+1} = δt_k · (r_{k-1}/r_k)^γ.
  double ptc_gamma{0.7};

  // ── Phase 4: Homotopy continuation compatibility fields ──────────
  /// Reserved for callers that explicitly invoke HomotopyContinuationSolver.
  /// NewtonSolver does not automatically dispatch to homotopy when it fails.
  bool enable_homotopy{true};

  /// Initial homotopy parameter increment Δλ ∈ (0, 1].
  double homotopy_step0{0.2};

  /// Minimum increment; below this the homotopy is declared failed.
  double homotopy_step_min{1e-3};

  /// Maximum increment (capped to prevent over-shooting λ = 1).
  double homotopy_step_max{1.0};

  /// Maximum number of λ increments (steps) in the homotopy path.
  int homotopy_max_steps{30};

  // ── Phase 5: Advanced (reserved, all disabled by default) ────────
  /// Use ln V and ln Vdc as Newton unknowns (guarantees V > 0 always).
  bool enable_log_voltage{false};

  /// Automatically retry with log-voltage if the first solve fails.
  bool auto_enable_log_voltage_on_failure{false};

  /// Use a matrix-free Krylov (GMRES) inner solver instead of sparse LU
  /// when the Jacobian is very ill-conditioned.
  bool enable_newton_krylov_fallback{false};

  /// Use a Schur-complement AC/DC block preconditioner in the Krylov solver.
  bool enable_schur_preconditioner{false};

  /// When the default line-search Newton step is rejected (line search exhausted
  /// and regularisation failed), automatically attempt a Levenberg--Marquardt
  /// recovery step and, if that also fails, a pseudo-transient continuation step
  /// before declaring the iteration failed.  This closes the gap where Phase 3
  /// fallbacks were only available when opt.globalization was set explicitly to
  /// TrustRegion or PseudoTransient.
  /// Requires enable_lm_trust_region_fallback or enable_ptc_ser to be true.
  bool enable_auto_fallback_scheduling{false};

  // ── Phase 5: Newton-Krylov parameters ────────────────────────────
  /// Proxy condition number above which the NK-GMRES fallback is triggered
  /// (evaluated against the ℓ₁ row/col norm ratio from estimate_condition_proxy).
  /// Only relevant when enable_newton_krylov_fallback is true.
  double nk_condition_trigger{1e8};

  /// Krylov subspace restart size m for GMRES(m).
  int gmres_restart{30};

  /// Maximum number of GMRES outer restart cycles.  The total maximum number
  /// of matrix-vector products is gmres_restart * gmres_max_outer.
  int gmres_max_outer{10};

  /// Relative residual tolerance for GMRES convergence ‖r‖/‖b‖ < tol.
  double gmres_tol{1e-10};
};

/// Solver strategy mode (for logging strategy switches).
enum class NonlinearSolveMode {
  ScaledNewton,         ///< Standard Newton with scaled residual/Jacobian.
  DampedNewton,         ///< Damped Newton with backtracking line search.
  NonmonotoneLineSearch,///< Nonmonotone (GLL) line search.
  LMTrustRegion,        ///< Levenberg–Marquardt trust-region fallback.
  PtcSer,               ///< Pseudo-transient continuation (SER timestep).
  Homotopy,             ///< Homotopy continuation (λ ramp).
  NewtonKrylovFallback  ///< Matrix-free Krylov inner solver (Phase 5).
};

/// Records a single solver strategy transition for diagnostic logging.
struct SolverModeSwitchRecord {
  int iteration{0};
  NonlinearSolveMode from{NonlinearSolveMode::ScaledNewton};
  NonlinearSolveMode to{NonlinearSolveMode::ScaledNewton};
  std::string reason;
};

}  // namespace hacdcpf::powerflow

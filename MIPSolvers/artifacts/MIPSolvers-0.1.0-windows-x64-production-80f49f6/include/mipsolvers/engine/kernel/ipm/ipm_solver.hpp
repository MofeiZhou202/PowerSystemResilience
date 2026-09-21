#pragma once

#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Globalization strategy selector.
/// Merit preserves the historical Mehrotra + normalized-residual merit
/// backtracking line search. Filter (default) enables the Wächter–Biegler
/// primal–dual filter line search with inertia-corrected Newton steps. Filter
/// must be combined with `use_inertia_correction=true` to retain its
/// nonconvex-robustness guarantee; the default configuration does so.
enum class Globalization { Merit, Filter };

/// Algebraically equivalent primal-dual Newton formulations. Auto compares
/// the elimination graphs once and selects the candidate whose symbolic
/// factor work and storage are both strictly smaller.
enum class NewtonFormulation { Auto, Condensed, Augmented };

/// Options for the Interior-Point Method driver. Filter-specific fields are
/// ignored when `globalization == Merit`.
struct IPMOptions {
  int max_iter{400};
  double tol_primal{1e-6};
  /// Stationarity tolerance in the solver's historical metric. With
  /// multiplier_relative_stationarity enabled this is ||grad L||inf divided
  /// by the IPOPT-style average-multiplier scale; otherwise it is unscaled.
  double tol_dual{1e-6};
  /// Unscaled max |s_i mu_i| complementarity guard.
  double tol_complementarity{1e-6};
  /// Optional unscaled ||grad L||inf guard. Zero disables this additional
  /// guard so existing callers retain their historical stationarity contract.
  double raw_dual_guard{0.0};
  /// Optional scaled overall NLP error tolerance. Nonpositive disables this
  /// additional gate so existing component-tolerance callers are unchanged.
  double tol_overall{0.0};
  /// Positive values select the multiplier scale threshold used by the
  /// IPOPT-style average-multiplier scaling. Nonpositive derives it from the
  /// current objective-gradient infinity norm, with one as the unit floor.
  double multiplier_scale_threshold{0.0};
  // Retained for source compatibility. When true, tol_dual applies to the
  // average-multiplier-scaled stationarity metric.
  bool multiplier_relative_stationarity{false};
  /// Scaled overall acceptable level. Acceptance is disabled unless this and
  /// acceptable_iter are positive. The stationarity component uses the same
  /// metric as tol_dual; nonpositive component values inherit tol_accept.
  double tol_accept{0.0};
  double tol_accept_primal{0.0};
  double tol_accept_dual{0.0};
  double tol_accept_complementarity{0.0};
  /// Optional unscaled acceptable-level ||grad L||inf guard. Zero disables it.
  double raw_dual_accept_guard{0.0};
  /// Number of consecutive complete iterates required at the acceptable
  /// level. This prevents one transient or restored best iterate from being
  /// reported as acceptable convergence.
  int acceptable_iter{0};
  // Positive values are explicit caller policies. Nonpositive requests the
  // largest strict-interior fraction distinguishable at machine precision.
  double alpha_max{0.0};
  // Positive values are explicit sparse-memory policies. Nonpositive uses
  // one complete current full-space secant block derived from model dimension.
  int qn_sparse_block_size{0};
  int qn_max_blocks{0};
  bool verbose{false};
  // Keep native solver results native by default. When enabled, a failed
  // native solve may call Ipopt and the solver_name will identify that path.
  bool allow_external_fallback{false};

  // Optional primal-dual central-path warm start. Each vector is used only
  // when its dimension matches the assembled block and all entries are finite;
  // slack and inequality-dual entries must additionally be strictly positive.
  // When both inequality vectors are valid, the automatic barrier initializer
  // recovers mean(s_i * mu_i) from this complete state. A primal-only Phase-I
  // handoff instead retains its row-wise, minimum-product barrier policy.
  Eigen::VectorXd equality_dual_start;
  Eigen::VectorXd inequality_dual_start;
  Eigen::VectorXd slack_start;

  // The caller asserts that x0 was produced by an external Phase I and passes
  // the original-coordinate primal tolerance. Native IPM independently audits
  // that assertion before using the feasible-start initialization policy.
  bool primal_feasible_start{false};

  // Preserve an externally restored x0 without enabling the feasible-start
  // multiplier, Newton, or globalization policies. Native independently
  // audits the original-coordinate primal tolerance before honoring it.
  bool preserve_initial_point{false};

  // A complete external primal-dual state may bypass cold interiorization
  // only after an original-coordinate feasibility and centrality audit.
  // Nonpositive tolerances derive from the corresponding caller KKT
  // tolerance. A nonpositive multiplier limit disables the optional exposed
  // magnitude policy; finiteness and positivity are always required.
  bool central_warm_start{false};
  double central_warm_start_primal_tolerance{0.0};
  double central_warm_start_centrality_tolerance{0.0};
  double central_warm_start_max_inequality_dual{0.0};

  // Cold-start equality multipliers from
  //   min_lambda ||grad f + Jh^T mu + Jg^T lambda||_2.
  // The candidate is used only when it improves stationarity by more than its
  // floating-point comparison error. A valid user warm start takes precedence.
  // Pure quasi-Newton models defer this initialization because their first
  // Lagrangian secants depend on the multiplier trajectory.
  bool least_square_init_duals{false};

  // --- Filter-driver options (PR2+) -----------------------------------
  Globalization globalization{Globalization::Filter};
  bool use_inertia_correction{true};
  NewtonFormulation newton_formulation{NewtonFormulation::Auto};
  // Solve the Newton step in the uncondensed augmented form
  //   [ H + δ_W I   Jgᵀ   Jhᵀ  ] [dx ]   [-r_d                       ]
  //   [ Jg          0     0    ] [dλ ] = [-r_eq                       ]
  //   [ Jh          0   -SM⁻¹  ] [dμ ]   [-r_ineq - M⁻¹(μ̄e - Sμ)      ]
  // instead of the condensed [H + Jhᵀ(M/S)Jh, Jgᵀ; Jg, 0] form.  The
  // augmented form avoids the Jhᵀ(M/S)Jh product (fill + squared
  // conditioning) at the price of a larger sparse factorization.
  /// Backward-compatible force flag. Prefer
  /// `newton_formulation = NewtonFormulation::Augmented` in new code.
  bool use_augmented_newton{false};

  // Additional PR3 feature flags (all default-on; flip false to disable
  // individually).
  bool use_second_order_correction{true};
  bool use_restoration_phase{true};
  bool scale_problem{true};
  // Positive values select a caller scaling target. Nonpositive normalizes
  // derivative infinity norms to the unit scale.
  double scaling_g_max{0.0};
  // A positive value is an explicit proximal-weight policy. A nonpositive
  // value requests the backward-error-level tie breaker derived from the unit
  // l1 restoration penalty.
  double restoration_zeta{0.0};

  // Barrier schedule. `mu_init` is an exposed user policy; a nonpositive value
  // explicitly requests the scale-covariant automatic initializer derived
  // from the current primal-dual state. A nonpositive minimum requests the
  // floating-point representability limit.
  double mu_init{0.0};
  double mu_min{0.0};
  // Positive values expose the legacy monotone barrier policy. With either
  // value nonpositive, the next target is derived from the current product
  // distribution, the caller complementarity gate, and representability.
  double kappa_mu{0.0};
  double theta_mu{0.0};
  // Positive values are an exposed inner-barrier policy. Nonpositive selects
  // the unit central-neighborhood coefficient E_mu <= mu.
  double kappa_epsilon{0.0};

  // Positive values opt into the parameterized Wächter-Biegler switching
  // filter. The all-zero default uses strict Pareto improvement with
  // machine-roundoff margins and a locally resolvable minimum step.
  double filter_gamma_theta{0.0};
  double filter_gamma_phi{0.0};
  double filter_s_theta{0.0};
  double filter_s_phi{0.0};
  double filter_delta{0.0};
  double filter_eta_phi{0.0};
  double filter_theta_min_scale{0.0};
  double filter_alpha_min{0.0};
};

/// Detailed IPM result (extends SolveResult with multiplier information).
struct IPMDetail {
  Eigen::VectorXd lambda_eq;  // equality multipliers
  Eigen::VectorXd mu_ineq;    // inequality multipliers
  Eigen::VectorXd z_slack;    // inequality slacks
  double complementarity{0.0};
  /// Final central-path target. This is distinct from max |s_i mu_i|.
  double barrier_parameter{0.0};
  /// Concrete sparse factorization backend used by the selected Newton path.
  std::string linear_solver_backend{"unselected"};
  std::string newton_formulation{"unselected"};
  int condensed_dimension{0};
  int augmented_dimension{0};
  int condensed_nonzeros{0};
  int augmented_nonzeros{0};
  double condensed_symbolic_flops{0.0};
  double augmented_symbolic_flops{0.0};
  double condensed_symbolic_nonzeros{0.0};
  double augmented_symbolic_nonzeros{0.0};
  int symbolic_analyses{0};
  int numeric_factorizations{0};
  int linear_solves{0};
  int primary_factorizations{0};
  int inertia_retry_factorizations{0};
  int inertia_certificate_factorizations{0};
  int restoration_factorizations{0};
  int retry_factorizations{0};
  int active_set_polish_factorizations{0};
  int accepted_steps{0};
  int rejected_steps{0};
  int trial_value_evaluations{0};
  int trial_full_derivative_evaluations{0};
  int trial_rejections_before_derivatives{0};
  bool primal_feasible_start_requested{false};
  bool primal_feasible_start_accepted{false};
  bool central_warm_start_requested{false};
  bool central_warm_start_accepted{false};
  double central_warm_start_original_primal_violation{0.0};
  double central_warm_start_primal_residual{0.0};
  double central_warm_start_dual_residual{0.0};
  double central_warm_start_centrality{0.0};
  double central_warm_start_mu{0.0};
  double central_warm_start_max_inequality_dual{0.0};
  std::string central_warm_start_rejection_reason;
  bool first_step_accepted{false};
  double first_step_primal_alpha{0.0};
  double first_step_dual_alpha{0.0};
  double first_step_barrier_objective_before{0.0};
  double first_step_barrier_objective_after{0.0};
  bool restoration_warm_start_used{false};
  double restoration_normal_residual_before{0.0};
  double restoration_normal_residual_after{0.0};
  double restoration_state_stationarity{0.0};
  double restoration_control_stationarity{0.0};
  int original_dimension{0};
  int reduced_dimension{0};
  int fixed_variables_eliminated{0};
};

struct IPMTerminationMetrics {
  double dual_scale{1.0};
  double complementarity_scale{1.0};
  double overall_error{0.0};
  bool strict{false};
  bool acceptable{false};
};

/// Evaluate the two-layer IPOPT-style termination contract from
/// original-coordinate residuals without running an IPM trajectory.
[[nodiscard]] IPMTerminationMetrics evaluate_ipm_termination(
    double primal, double raw_dual, double raw_complementarity,
    const Eigen::VectorXd& equality_multipliers,
    const Eigen::VectorXd& complementarity_multipliers,
    double objective_gradient_inf, const IPMOptions& options);

/// Native primal-dual IPM for NLP problems, with a Wächter–Biegler filter
/// driver and an optional Mehrotra-style merit driver.
/// Operates on NLPModel via callbacks: f, grad, hess, g/jac_g, h/jac_h.
/// Variable bounds are read from NLPModel::vars.
class NativeIPMAdapter final : public SolverAdapter {
 public:
  explicit NativeIPMAdapter(IPMOptions opt = {});
  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_nlp(const NLPModel& prob) const override;

  /// Solve and return extended result including multipliers.
  std::pair<SolveResult, IPMDetail> solve_nlp_detail(const NLPModel& prob) const;

 private:
  IPMOptions opt_;
};

}  // namespace mipsolvers::engine

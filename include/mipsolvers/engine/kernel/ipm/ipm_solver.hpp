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
  double tol_dual{1e-6};
  double tol_complementarity{1e-6};
  /// "Acceptable" convergence level (Ipopt-style): if the best iterate
  /// satisfies primal feasibility, dual feasibility, and complementarity at
  /// tol_accept, it is accepted as a near-optimal solution even when the
  /// strict tolerances are not met. Disabled when set to 0.
  double tol_accept{1e-2};
  double alpha_max{0.95};  // fraction-to-boundary τ
  int qn_sparse_block_size{8};
  int qn_max_blocks{6};
  bool verbose{false};
  // Keep native solver results native by default. When enabled, a failed
  // native solve may call Ipopt and the solver_name will identify that path.
  bool allow_external_fallback{false};

  // Opt-in contracts for externally restored starts. The adapter preserves x0
  // only after an original-coordinate audit proves feasibility at tol_primal;
  // otherwise the ordinary interiorization path remains active. See
  // docs/native_ipm_windows_integration_2026-08-13.md, "Restored starts".
  bool primal_feasible_start{false};
  bool preserve_initial_point{false};

  // Audited Phase-I -> Phase-II handoff. Unlike primal_feasible_start, this
  // contract permits a near-feasible point inside a central-path corridor.
  // It is accepted only with complete primal/dual/slack coverage and
  //   max(||g||_inf, ||h+s||_inf) <= central_warm_start_primal_tolerance,
  //   max_i |s_i*z_i/mu_init - 1| <= central_warm_start_centrality_tolerance.
  // Rejection discards every warm vector and follows ordinary cold-start
  // interiorization. See Waechter--Biegler (2006), Sections 2--3.
  bool central_warm_start{false};
  double central_warm_start_primal_tolerance{1e-2};
  double central_warm_start_centrality_tolerance{0.5};
  double central_warm_start_max_inequality_dual{1e4};

  // Optional primal-dual central-path warm start. Each vector is used only
  // when its dimension matches the assembled block and all entries are finite;
  // slack and inequality-dual entries must additionally be strictly positive.
  Eigen::VectorXd equality_dual_start;
  Eigen::VectorXd inequality_dual_start;
  Eigen::VectorXd slack_start;

  // Cold-start equality multipliers from
  //   min_lambda ||grad f + Jh^T mu + Jg^T lambda||_2.
  // The candidate is used only when it improves stationarity and its infinity
  // norm does not exceed constr_mult_init_max. A valid user warm start takes
  // precedence. Pure quasi-Newton models defer this initialization because
  // their first Lagrangian secants depend on the multiplier trajectory.
  bool least_square_init_duals{false};
  double constr_mult_init_max{1000.0};

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
  double scaling_g_max{100.0};
  double restoration_zeta{1e-4};

  // Barrier schedule (IPOPT Implementation Paper defaults).
  double mu_init{0.1};
  double mu_min{1e-11};
  double kappa_mu{0.2};
  double theta_mu{1.5};
  double kappa_epsilon{10.0};

  // Filter line search constants (Wächter–Biegler 2006, Table 1 defaults).
  double filter_gamma_theta{1e-5};
  double filter_gamma_phi{1e-5};
  double filter_s_theta{1.1};
  double filter_s_phi{2.3};
  double filter_delta{1.0};
  double filter_eta_phi{1e-4};
  double filter_theta_min_scale{1e-4};  // θ_min = scale · max(1, θ₀)
  double filter_alpha_min{1e-10};
};

/// Detailed IPM result (extends SolveResult with multiplier information).
struct IPMDetail {
  Eigen::VectorXd lambda_eq;  // equality multipliers
  Eigen::VectorXd mu_ineq;    // inequality multipliers
  Eigen::VectorXd z_slack;    // inequality slacks
  double complementarity{0.0};
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

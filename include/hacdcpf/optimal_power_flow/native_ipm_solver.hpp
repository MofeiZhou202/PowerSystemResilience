#pragma once

/// optimal_power_flow/native_ipm_solver.hpp
/// ==========================================
/// Primal-dual interior-point method (IPM) for nonlinear OPF.
/// Shares result types with parity_ipm_solver.hpp.
/// Replaces part of: optimal_power_flow/parity_ipm.hpp.

#include <limits>
#include <memory>
#include <string>

#include <Eigen/Core>

#include "hacdcpf/optimal_power_flow/formulation.hpp"

namespace hacdcpf::opf::parity {

/// Opaque owning cache for sparse KKT symbolic analysis across public solves.
/// Numeric factors are never exposed or accepted as a solution certificate.
class IPMPreparedState {
 public:
  IPMPreparedState();
  ~IPMPreparedState();
  IPMPreparedState(const IPMPreparedState&) = delete;
  IPMPreparedState& operator=(const IPMPreparedState&) = delete;
  IPMPreparedState(IPMPreparedState&&) noexcept;
  IPMPreparedState& operator=(IPMPreparedState&&) noexcept;
  void reset();

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct IPMOptions {
  int max_iter{400};
  double tol_primal{1e-6};
  double tol_dual{1e-6};
  double tol_complementarity{1e-6};
  double regularization{1e-8};
  double alpha_max{0.95};
  bool verbose{false};
  bool enable_phase_one{true};
  double phase_one_time_limit_ms{5000.0};
  int phase_one_max_iterations{12};
  int phase_one_max_factorizations{14};
  int phase_one_max_backtracks{12};
  /// Phase-I handoff corridor: ||r_p||_inf <= eta_p * mu_0 and
  /// max_i |z_i*mu_i/mu_0 - 1| <= eta_c. Phase II retains the strict final
  /// tolerances above. Waechter--Biegler (2006), Sections 2--3.
  double phase_one_barrier_mu{0.1};
  double phase_one_admission_mu_factor{1.0};
  double phase_one_primal_mu_factor{0.1};
  double phase_one_centrality_tolerance{0.5};
  bool phase_one_dispatch_dual_predictor{false};
  double phase_one_dispatch_dual_min_improvement{0.05};
  /// Optional primal start already expressed in the current Problem layout.
  const Eigen::VectorXd* primal_start{nullptr};
  /// Optional native-IPM state for a structurally compatible problem.  These
  /// blocks are used only when every dimension and positivity check succeeds.
  const Eigen::VectorXd* equality_dual_start{nullptr};
  const Eigen::VectorXd* inequality_dual_start{nullptr};
  const Eigen::VectorXd* slack_start{nullptr};
  /// Optional owner whose symbolic ordering survives this solve. The caller
  /// must reset it whenever Problem::layout_signature changes.
  IPMPreparedState* prepared_state{nullptr};
  /// Experimental numeric-refactor request. It is fail-closed unless the
  /// backend supports a distinct audited refactor operation; current backends
  /// report unsupported and always compute fresh numeric factors.
  bool prepared_numeric_refactor{false};
  double prepared_numeric_max_relative_drift{1e-3};
  double prepared_numeric_backward_error_tolerance{1e-8};
};

struct IPMResult {
  bool converged{false};
  int iterations{0};
  double primal_inf{0.0};
  double dual_inf{0.0};
  double complementarity{0.0};
  double initial_primal_inf{0.0};
  double initial_dual_inf{0.0};
  double initial_complementarity{0.0};
  bool warm_start_used{false};
  double phase_one_initial_violation{
      std::numeric_limits<double>::infinity()};
  double phase_one_constraint_violation{
      std::numeric_limits<double>::infinity()};
  double phase_one_dual_fit_residual{
      std::numeric_limits<double>::infinity()};
  bool phase_one_primal_feasible{false};
  bool phase_one_in_handoff_corridor{false};
  bool phase_one_dual_initialized{false};
  double phase_one_handoff_primal_tolerance{0.0};
  double phase_one_perturbed_primal_residual{
      std::numeric_limits<double>::infinity()};
  double phase_one_centrality{
      std::numeric_limits<double>::infinity()};
  double phase_one_barrier_mu{0.0};
  bool phase_one_budget_exhausted{false};
  int phase_one_iterations{0};
  int phase_one_factorizations{0};
  int phase_one_backtracks{0};
  bool phase_one_structural_step_attempted{false};
  bool phase_one_structural_step_accepted{false};
  int phase_one_structural_factorizations{0};
  double phase_one_structural_violation{
      std::numeric_limits<double>::infinity()};
  std::string phase_one_structure{"ac-state-basic"};
  double phase_one_runtime_ms{0.0};
  std::string phase_one_termination{"not-run"};
  std::string phase_one_linear_solver{"unselected"};
  bool dispatch_dual_predictor_attempted{false};
  bool dispatch_dual_predictor_accepted{false};
  double dispatch_dual_predictor_runtime_ms{0.0};
  double dispatch_dual_predictor_baseline_raw{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_candidate_raw{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_baseline_normalized{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_candidate_normalized{
      std::numeric_limits<double>::infinity()};
  std::string dispatch_dual_predictor_status{"not-attempted"};
  bool phase_two_start_accepted{false};
  std::string phase_two_start_rejection_reason;
  int factorization_calls{0};
  int linear_solve_calls{0};
  int accepted_steps{0};
  int rejected_steps{0};
  int symbolic_analyze_calls{0};
  bool symbolic_reused{false};
  bool numeric_refactor_attempted{false};
  bool numeric_refactor_accepted{false};
  double numeric_refactor_relative_drift{
      std::numeric_limits<double>::infinity()};
  double numeric_refactor_backward_error{
      std::numeric_limits<double>::infinity()};
  std::string numeric_refactor_status{"not-requested"};
  int backend_escalations{0};
  int scaling_rebuilds{0};
  std::string status;
  /// Concrete KKT linear-algebra path used, e.g. "dense_lu",
  /// "sparse_umfpack", "sparse_klu", or "sparse_eigen_lu".
  std::string linear_solver;
  Eigen::VectorXd x;
  Eigen::VectorXd lambda_eq;
  Eigen::VectorXd mu;
  Eigen::VectorXd z;
};

struct IPMInitialPointDiagnostics {
  bool valid{false};
  double primal_inf{std::numeric_limits<double>::infinity()};
  double dual_inf{std::numeric_limits<double>::infinity()};
  double complementarity{std::numeric_limits<double>::infinity()};
};

/// Evaluate the exact cold-dual/slack initialization metrics without a KKT
/// factorization. Used to admit one structure-aware primal over another.
IPMInitialPointDiagnostics evaluate_ipm_initial_point(
    const Problem& prob, const Eigen::VectorXd& primal_start);

/// Solve a parity OPF nonlinear program with a primal-dual interior-point method.
///
/// The problem has the form
///
/// @htmlonly
/// <div>\[
///   \min_x f(x) \quad
///   \text{s.t.}\quad g(x)=0,\quad h(x)\le 0,\quad x_{min}\le x\le x_{max}.
/// \]</div>
/// @endhtmlonly
///
/// Inequalities and bounds are converted to slacks, then Newton steps are
/// computed from the perturbed KKT conditions
///
/// @htmlonly
/// <div>\[
///   \nabla f(x) + J_g(x)^T\lambda + J_h(x)^T\mu + z^+ - z^- = 0,
/// \]</div>
/// <div>\[
///   g(x)=0,\qquad h(x)+s=0,\qquad S\mu=\tau e,
/// \]</div>
/// @endhtmlonly
///
/// with fraction-to-boundary step control.  The returned dual vectors use the
/// row order defined by `Problem::cidx`.
///
/// @param prob Fully assembled parity formulation.
/// @param opt Iteration limits, convergence tolerances, and regularization.
/// @return Primal/dual solution, KKT residual metrics, and convergence status.
IPMResult solve_primal_dual_ipm(const Problem& prob, const IPMOptions& opt = {});

/// Davidenko tangent of the objective homotopy P(t): min t·f(x) at a path
/// point (x, z, lambda, mu) of the t-scaled problem.  Differentiating the
/// perturbed KKT conditions with respect to t gives
///
///   K_aug · (dx/dt, dλ/dt, dμ/dt) = (−∇f/t, 0, 0),  dz/dt = −Jh·dx/dt,
///
/// where ∇f is the gradient of the t-scaled objective (so ∇f/t is the full
/// cost gradient entering the stationarity at rate 1) and K_aug is the
/// IPM's own augmented system — one factorization + back-solve with the
/// existing LDLᵀ machinery (inertia-controlled) yields the full path
/// tangent.  Returns false when t ≈ 0, the system cannot be factored, or
/// the solve is non-finite.  Theory: docs/numerical_methods.md §13.
bool homotopy_tangent(const Problem& prob,
                      const Eigen::VectorXd& x,
                      const Eigen::VectorXd& z,
                      const Eigen::VectorXd& lambda,
                      const Eigen::VectorXd& mu,
                      double t,
                      Eigen::VectorXd& dx_dt,
                      Eigen::VectorXd& dz_dt,
                      Eigen::VectorXd& dlambda_dt,
                      Eigen::VectorXd& dmu_dt);

}  // namespace hacdcpf::opf::parity

#pragma once

/// optimal_power_flow/native_ipm_solver.hpp
/// ==========================================
/// Primal-dual interior-point method (IPM) for nonlinear OPF.
/// Shares result types with parity_ipm_solver.hpp.
/// Replaces part of: optimal_power_flow/parity_ipm.hpp.

#include <string>

#include <Eigen/Core>

#include "hacdcpf/optimal_power_flow/formulation.hpp"

namespace hacdcpf::opf::parity {

struct IPMOptions {
  int max_iter{400};
  double tol_primal{1e-6};
  double tol_dual{1e-6};
  double tol_complementarity{1e-6};
  double regularization{1e-8};
  double alpha_max{0.95};
  bool verbose{false};
  /// Optional primal start already expressed in the current Problem layout.
  const Eigen::VectorXd* primal_start{nullptr};
  /// Optional native-IPM state for a structurally compatible problem.  These
  /// blocks are used only when every dimension and positivity check succeeds.
  const Eigen::VectorXd* equality_dual_start{nullptr};
  const Eigen::VectorXd* inequality_dual_start{nullptr};
  const Eigen::VectorXd* slack_start{nullptr};
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
  int factorization_calls{0};
  int linear_solve_calls{0};
  int accepted_steps{0};
  int rejected_steps{0};
  int symbolic_analyze_calls{0};
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

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
};

struct IPMResult {
  bool converged{false};
  int iterations{0};
  double primal_inf{0.0};
  double dual_inf{0.0};
  double complementarity{0.0};
  std::string status;
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

}  // namespace hacdcpf::opf::parity

#include "hacdcpf/dynamics/integration/Trapezoidal.hpp"

#include <algorithm>

#include <Eigen/Dense>

#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "TimeIntegratorUtils.hpp"

namespace hacdcpf::dynamics {

IntegrationStepResult Trapezoidal::step(DynamicSystem& system,
                                        double t,
                                        double dt) const {
  std::string error;
  const Eigen::VectorXd x0 = system.x.x;
  Eigen::VectorXd f0;
  if (!detail::evaluate_derivatives(system, t, x0, f0, error)) {
    return detail::make_failure(error);
  }
  Eigen::VectorXd x = x0 + dt * f0;
  Eigen::VectorXd f = f0;
  for (int iter = 0; iter < system.options.max_newton_iters; ++iter) {
    if (!detail::evaluate_derivatives(system, t + dt, x, f, error)) {
      return detail::make_failure(error);
    }
    const Eigen::VectorXd residual = x - x0 - 0.5 * dt * (f0 + f);
    const double res_norm = residual.lpNorm<Eigen::Infinity>();
    if (res_norm <= system.options.newton_tol) {
      system.x.x = x;
      system.x.dxdt = f;
      if (!system.solveNetwork(t + dt, error)) return detail::make_failure(error);
      return {true, iter + 2, iter + 1, {}};
    }
    Eigen::MatrixXd jf;
    if (!detail::numerical_state_jacobian(system, t + dt, x, f, jf, error)) {
      return detail::make_failure(error);
    }
    const Eigen::MatrixXd jr =
        Eigen::MatrixXd::Identity(x.size(), x.size()) - 0.5 * dt * jf;
    const Eigen::VectorXd delta = jr.colPivHouseholderQr().solve(-residual);
    if (!delta.allFinite()) return detail::make_failure("Trapezoidal Newton correction is non-finite");

    double alpha = 1.0;
    bool accepted = false;
    const double min_alpha = std::max(1e-9, system.options.newton_damping_min);
    while (alpha >= min_alpha) {
      const Eigen::VectorXd trial = x + alpha * delta;
      Eigen::VectorXd f_trial;
      if (!detail::evaluate_derivatives(system, t + dt, trial, f_trial, error)) {
        return detail::make_failure(error);
      }
      const Eigen::VectorXd r_trial = trial - x0 - 0.5 * dt * (f0 + f_trial);
      if (r_trial.lpNorm<Eigen::Infinity>() <=
          (1.0 - 1e-4 * alpha) * std::max(res_norm, system.options.newton_tol)) {
        x = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) return detail::make_failure("Trapezoidal Newton line search failed");
  }
  return detail::make_failure("Trapezoidal Newton did not converge");
}

}  // namespace hacdcpf::dynamics

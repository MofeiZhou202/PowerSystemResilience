#include "hacdcpf/dynamics/integration/Rosenbrock.hpp"

#include <Eigen/Dense>

#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "TimeIntegratorUtils.hpp"

namespace hacdcpf::dynamics {

IntegrationStepResult Rosenbrock::step(DynamicSystem& system,
                                       double t,
                                       double dt) const {
  std::string error;
  const Eigen::VectorXd x0 = system.x.x;
  Eigen::VectorXd f0;
  if (!detail::evaluate_derivatives(system, t, x0, f0, error)) {
    return detail::make_failure(error);
  }
  Eigen::MatrixXd jf;
  if (!detail::numerical_state_jacobian(system, t, x0, f0, jf, error)) {
    return detail::make_failure(error);
  }
  const Eigen::MatrixXd a =
      Eigen::MatrixXd::Identity(x0.size(), x0.size()) - dt * jf;
  Eigen::VectorXd delta;
  if (!detail::sparse_solve(a, dt * f0, delta, error)) {
    return detail::make_failure(error);
  }
  system.x.x = x0 + delta;
  Eigen::VectorXd f1;
  if (!detail::evaluate_derivatives(system, t + dt, system.x.x, f1, error)) {
    return detail::make_failure(error);
  }
  system.x.dxdt = f1;
  if (!system.solveNetwork(t + dt, error)) return detail::make_failure(error);
  return {true, 2, 1, {}};
}

}  // namespace hacdcpf::dynamics

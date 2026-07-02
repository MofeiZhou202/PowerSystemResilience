#include "hacdcpf/dynamics/integration/RK4.hpp"

#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "TimeIntegratorUtils.hpp"

namespace hacdcpf::dynamics {

IntegrationStepResult RK4::step(DynamicSystem& system,
                                double t,
                                double dt) const {
  std::string error;
  const Eigen::VectorXd x0 = system.x.x;
  Eigen::VectorXd k1, k2, k3, k4;
  if (!detail::evaluate_derivatives(system, t, x0, k1, error)) {
    return detail::make_failure(error);
  }
  if (!detail::evaluate_derivatives(system, t + 0.5 * dt, x0 + 0.5 * dt * k1, k2, error)) {
    return detail::make_failure(error);
  }
  if (!detail::evaluate_derivatives(system, t + 0.5 * dt, x0 + 0.5 * dt * k2, k3, error)) {
    return detail::make_failure(error);
  }
  if (!detail::evaluate_derivatives(system, t + dt, x0 + dt * k3, k4, error)) {
    return detail::make_failure(error);
  }
  system.x.x = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  system.x.dxdt = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
  if (!system.solveNetwork(t + dt, error)) return detail::make_failure(error);
  return {true, 4, 0, {}};
}

}  // namespace hacdcpf::dynamics


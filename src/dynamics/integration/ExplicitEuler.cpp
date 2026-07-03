#include "hacdcpf/dynamics/integration/ExplicitEuler.hpp"

#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "TimeIntegratorUtils.hpp"

namespace hacdcpf::dynamics {

IntegrationStepResult ExplicitEuler::step(DynamicSystem& system,
                                          double t,
                                          double dt) const {
  std::string error;
  Eigen::VectorXd k1;
  if (!detail::evaluate_derivatives(system, t, system.x.x, k1, error)) {
    return detail::make_failure(error);
  }
  system.x.x += dt * k1;
  system.x.dxdt = k1;
  if (!system.solveNetwork(t + dt, error)) return detail::make_failure(error);
  return {true, 1, 0, {}};
}

}  // namespace hacdcpf::dynamics


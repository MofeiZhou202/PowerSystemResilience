#include "hacdcpf/dynamics/solvers/DaeSolver.hpp"

#include "hacdcpf/dynamics/DynamicSystem.hpp"

namespace hacdcpf::dynamics {

DaeStepResult DaeSolver::step(DynamicSystem& system,
                              double t,
                              double dt) const {
  const auto algebraic = algebraic_solver_.solve(system, t);
  if (!algebraic.success) {
    return {false, 0, 0, algebraic.message};
  }
  const auto integration = integrator_.step(system, t, dt);
  if (!integration.success) {
    return {false,
            integration.derivative_evaluations,
            integration.nonlinear_iterations,
            integration.message};
  }
  const auto final_algebraic = algebraic_solver_.solve(system, t + dt);
  if (!final_algebraic.success) {
    return {false,
            integration.derivative_evaluations,
            integration.nonlinear_iterations,
            final_algebraic.message};
  }
  system.x.time_s = t + dt;
  return {true,
          integration.derivative_evaluations,
          integration.nonlinear_iterations,
          {}};
}

}  // namespace hacdcpf::dynamics


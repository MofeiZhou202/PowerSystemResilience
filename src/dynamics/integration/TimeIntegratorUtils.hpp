#pragma once

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Core>

#include "hacdcpf/dynamics/DynamicSystem.hpp"

namespace hacdcpf::dynamics::detail {

inline bool evaluate_derivatives(DynamicSystem& system,
                                 double t,
                                 const Eigen::VectorXd& state,
                                 Eigen::VectorXd& dxdt,
                                 std::string& error) {
  return system.evaluateDerivatives(t, state, dxdt, error);
}

inline bool numerical_state_jacobian(DynamicSystem& system,
                                     double t,
                                     const Eigen::VectorXd& state,
                                     const Eigen::VectorXd& f0,
                                     Eigen::MatrixXd& jac,
                                     std::string& error) {
  const Eigen::Index n = state.size();
  jac = Eigen::MatrixXd::Zero(n, n);
  if (n == 0) return true;
  const double eps0 = std::sqrt(std::numeric_limits<double>::epsilon());
  for (Eigen::Index col = 0; col < n; ++col) {
    Eigen::VectorXd perturbed = state;
    const double h = eps0 * std::max(1.0, std::abs(state[col]));
    perturbed[col] += h;
    Eigen::VectorXd fp;
    if (!evaluate_derivatives(system, t, perturbed, fp, error)) return false;
    jac.col(col) = (fp - f0) / h;
  }
  return true;
}

inline IntegrationStepResult make_failure(std::string message) {
  IntegrationStepResult result;
  result.success = false;
  result.message = std::move(message);
  return result;
}

}  // namespace hacdcpf::dynamics::detail


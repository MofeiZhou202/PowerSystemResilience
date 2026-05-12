#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/optimal_power_flow/parity_formulation.hpp"

namespace hacdcpf::opf::parity {

struct IPMOptions {
  int max_iter{400};
  double tol_primal{1e-6};
  double tol_dual{1e-6};
  double tol_complementarity{1e-6};
  double regularization{1e-8};
  double alpha_max{0.95};  // fraction-to-boundary τ (matches Julia ξ=0.95)
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

IPMResult solve_primal_dual_ipm(const Problem& prob, const IPMOptions& opt = {});

}  // namespace hacdcpf::opf::parity

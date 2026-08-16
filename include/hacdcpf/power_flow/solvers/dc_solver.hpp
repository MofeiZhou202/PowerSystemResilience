#pragma once

#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::powerflow {

/// Assemble J = d(V .* (Gdc*V) - Pspec(V))/dV on non-slack DC buses.
Eigen::SparseMatrix<double> build_dc_newton_jacobian(
    const SolverData& data,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    const Eigen::VectorXd& vdc,
    const std::vector<int>& dc_non_slack);

class DCSolver {
 public:
  DCPowerFlowResult solve(const SolverData& data,
                          const PowerFlowOptions& opt,
                          const InitialState* init = nullptr) const;
};

}  // namespace hacdcpf::powerflow

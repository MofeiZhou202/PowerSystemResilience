#pragma once

#include <Eigen/Core>

#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

struct ResidualBlocks {
  Eigen::VectorXd ac;
  Eigen::VectorXd dc;
  Eigen::VectorXd full;
};

ResidualBlocks evaluate_power_flow_residual(const SolverData& data,
                                            const Eigen::VectorXd& vm,
                                            const Eigen::VectorXd& va,
                                            const Eigen::VectorXd& vdc);

}  // namespace hacdcpf::powerflow

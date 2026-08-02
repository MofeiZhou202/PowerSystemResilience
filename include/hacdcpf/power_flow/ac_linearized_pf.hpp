#pragma once

#include <string>
#include <limits>
#include <vector>

#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

struct ACLinearizedDCResult {
  std::vector<double> va;      // voltage angles (radians)
  std::vector<double> pf_mw;   // branch active power flows (MW)
  double residual_pu{std::numeric_limits<double>::infinity()};
  bool success{false};
  std::string model_scope{"ac-only-linearized-dc"};
  std::string model_limitations;
};

ACLinearizedDCResult solve_ac_linearized_dc(const SolverData& data);

}  // namespace hacdcpf::powerflow

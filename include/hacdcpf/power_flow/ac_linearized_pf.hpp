#pragma once

#include <vector>

#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

struct ACLinearizedDCResult {
  std::vector<double> va;      // voltage angles (radians)
  std::vector<double> pf_mw;   // branch active power flows (MW)
  bool success{false};
};

ACLinearizedDCResult solve_ac_linearized_dc(const SolverData& data);

}  // namespace hacdcpf::powerflow

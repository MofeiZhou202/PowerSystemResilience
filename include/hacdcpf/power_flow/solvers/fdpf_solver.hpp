#pragma once

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

struct FDPFSolver {
  PowerFlowResult solve(const SolverData& data, const PowerFlowOptions& opt) const;
};

}  // namespace hacdcpf::powerflow

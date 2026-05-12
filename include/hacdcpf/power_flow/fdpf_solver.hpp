#pragma once

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

struct FDPFSolver {
  PowerFlowResult solve(const SolverData& data, const PowerFlowOptions& opt) const;
};

}  // namespace hacdcpf::powerflow

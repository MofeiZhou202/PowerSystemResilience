#pragma once

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::powerflow {

class DCSolver {
 public:
  DCPowerFlowResult solve(const SolverData& data,
                          const PowerFlowOptions& opt,
                          const InitialState* init = nullptr) const;
};

}  // namespace hacdcpf::powerflow

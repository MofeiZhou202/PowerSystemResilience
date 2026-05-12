#pragma once

#include "hacdcpf/power_flow/solver_data.hpp"
#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"

namespace hacdcpf::powerflow {

class DCSolver {
 public:
  DCPowerFlowResult solve(const SolverData& data,
                          const PowerFlowOptions& opt,
                          const InitialState* init = nullptr) const;
};

}  // namespace hacdcpf::powerflow

#pragma once

#include <unordered_map>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

class AdaptiveSolver {
 public:
  AdaptiveSolveResult solve(
      const HybridPowerSystem& sys,
      const PowerFlowOptions& opt,
      const std::unordered_map<int, ReactiveLimit>& q_limits = {}) const;
};

}  // namespace hacdcpf::powerflow

#pragma once

#include <unordered_map>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

class AdaptiveSolver {
 public:
  AdaptiveSolveResult solve(
      const HybridPowerSystem& sys,
      const PowerFlowOptions& opt,
      const std::unordered_map<int, ReactiveLimit>& q_limits = {}) const;
};

}  // namespace hacdcpf::powerflow

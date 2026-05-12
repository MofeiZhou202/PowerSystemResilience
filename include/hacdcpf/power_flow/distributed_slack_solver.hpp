#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

DistributedSlack create_participation_factors(
    const HybridPowerSystem& sys,
    const std::string& method = "capacity",
    const std::vector<int>& participating_buses = {},
    const std::unordered_map<int, double>& droop_coeffs = {});

class DistributedSlackSolver {
 public:
  DistributedSlackResult solve_simplified(const HybridPowerSystem& sys,
                                          const DistributedSlack& slack_cfg,
                                          const PowerFlowOptions& opt) const;

  DistributedSlackResult solve_full_jacobian(const HybridPowerSystem& sys,
                                             const DistributedSlack& slack_cfg,
                                             const PowerFlowOptions& opt) const;
};

}  // namespace hacdcpf::powerflow

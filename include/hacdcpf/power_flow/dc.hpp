#pragma once

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

// Solve DC power flow (linear approximation).
// Delegates to hacdcpf::solve_dc_power_flow().
DCPowerFlowResult solve_dc(const HybridPowerSystem& sys,
                            const PowerFlowOptions& opt = {});

}  // namespace hacdcpf::powerflow

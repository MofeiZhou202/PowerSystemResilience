#pragma once

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

// Solve DC power flow (linear approximation).
// Delegates to hacdcpf::solve_dc_power_flow().
DCPowerFlowResult solve_dc(const HybridPowerSystem& sys,
                            const PowerFlowOptions& opt = {});

}  // namespace hacdcpf::powerflow

#pragma once

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

// Solve AC power flow using Newton-Raphson method.
// Delegates to hacdcpf::solve_power_flow().
PowerFlowResult solve_ac(const HybridPowerSystem& sys,
                         const PowerFlowOptions& opt = {});

// Overload accepting ACSystem directly (wraps into HybridPowerSystem).
PowerFlowResult solve_ac(const ACSystem& ac_sys,
                         const PowerFlowOptions& opt = {});

}  // namespace hacdcpf::powerflow

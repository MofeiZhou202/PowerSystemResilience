#pragma once

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

// Solve AC power flow using Newton-Raphson method.
// Delegates to hacdcpf::solve_power_flow().
PowerFlowResult solve_ac(const HybridPowerSystem& sys,
                         const PowerFlowOptions& opt = {});

// Overload accepting ACSystem directly (wraps into HybridPowerSystem).
PowerFlowResult solve_ac(const ACSystem& ac_sys,
                         const PowerFlowOptions& opt = {});

}  // namespace hacdcpf::powerflow

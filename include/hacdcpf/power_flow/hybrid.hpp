#pragma once

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::powerflow {

// Solve hybrid AC/DC power flow (Newton-Raphson with converter coupling).
// Delegates to hacdcpf::solve_power_flow().
PowerFlowResult solve_hybrid(const HybridPowerSystem& sys,
                              const PowerFlowOptions& opt = {});

// Adaptive variant with automatic island detection and solving.
// Delegates to hacdcpf::solve_power_flow_adaptive().
AdaptiveSolveResult solve_hybrid_adaptive(const HybridPowerSystem& sys,
                                           const PowerFlowOptions& opt = {});

}  // namespace hacdcpf::powerflow

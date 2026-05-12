#pragma once

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

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

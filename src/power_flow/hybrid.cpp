#include "hacdcpf/power_flow/hybrid.hpp"

#include "hacdcpf/api/hacdcpf.hpp"

namespace hacdcpf::powerflow {

PowerFlowResult solve_hybrid(const HybridPowerSystem& sys,
                              const PowerFlowOptions& opt) {
  return hacdcpf::solve_power_flow(sys, opt);
}

AdaptiveSolveResult solve_hybrid_adaptive(const HybridPowerSystem& sys,
                                           const PowerFlowOptions& opt) {
  return hacdcpf::solve_power_flow_adaptive(sys, opt);
}

}  // namespace hacdcpf::powerflow

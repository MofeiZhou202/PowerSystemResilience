#include "hacdcpf/power_flow/dc.hpp"

#include "hacdcpf/api/hacdcpf.hpp"

namespace hacdcpf::powerflow {

DCPowerFlowResult solve_dc(const HybridPowerSystem& sys,
                            const PowerFlowOptions& opt) {
  return hacdcpf::solve_dc_power_flow(sys, opt);
}

}  // namespace hacdcpf::powerflow

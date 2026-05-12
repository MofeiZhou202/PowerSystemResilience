#include "hacdcpf/power_flow/ac.hpp"

#include "hacdcpf/api/hacdcpf.hpp"

namespace hacdcpf::powerflow {

PowerFlowResult solve_ac(const HybridPowerSystem& sys,
                         const PowerFlowOptions& opt) {
  return hacdcpf::solve_power_flow(sys, opt);
}

PowerFlowResult solve_ac(const ACSystem& ac_sys,
                         const PowerFlowOptions& opt) {
  HybridPowerSystem sys;
  sys.base_mva = ac_sys.base_mva;
  sys.ac = ac_sys;
  return hacdcpf::solve_power_flow(sys, opt);
}

}  // namespace hacdcpf::powerflow

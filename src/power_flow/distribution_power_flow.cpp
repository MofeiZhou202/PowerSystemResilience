#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace hacdcpf::analysis {

DistributionPFResult solve_distribution_power_flow(
    const HybridPowerSystem& /*sys*/,
    const DistributionPFOptions& /*opt*/) {
  // Stub: distribution backward/forward sweep not yet implemented.
  DistributionPFResult r;
  r.converged   = false;
  r.iterations  = 0;
  r.residual    = 0.0;
  r.status      = "not_implemented";
  return r;
}

}  // namespace hacdcpf::analysis

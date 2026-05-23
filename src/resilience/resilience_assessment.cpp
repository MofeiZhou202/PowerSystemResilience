#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace hacdcpf::analysis {

DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& /*sys*/,
    const DistributionResilienceOptions& /*opt*/) {
  // Stub: resilience assessment not yet implemented.
  DistributionResilienceResult r;
  r.completed           = false;
  r.total_ens_mwh       = 0.0;
  r.max_curtailment_mw  = 0.0;
  r.restoration_time_hr = 0.0;
  r.status              = "not_implemented";
  return r;
}

}  // namespace hacdcpf::analysis

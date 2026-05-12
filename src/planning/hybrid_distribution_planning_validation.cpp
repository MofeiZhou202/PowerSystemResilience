#include "hacdcpf/planning/hybrid_distribution_planning_validation.hpp"

namespace hacdcpf::planning {

HybridDistributionValidationReport validate_hybrid_distribution_planning_result(
    const HybridPowerSystem& /*base_system*/,
    const HybridDistributionPlanningResult& /*result*/,
    const HybridDistributionValidationOptions& /*options*/) {
  // Stub: validation not yet implemented.
  HybridDistributionValidationReport report;
  report.valid  = false;
  report.status = "not_implemented";
  return report;
}

HybridDistributionCandidateReplayResult replay_hybrid_distribution_candidate(
    const HybridPowerSystem& base_system,
    const HybridDistributionPlanningResult& /*result*/,
    const HybridDistributionCandidateReplayOptions& /*options*/) {
  // Stub: replay not yet implemented.
  HybridDistributionCandidateReplayResult r;
  r.completed    = false;
  r.built_system = base_system;
  r.status       = "not_implemented";
  return r;
}

}  // namespace hacdcpf::planning

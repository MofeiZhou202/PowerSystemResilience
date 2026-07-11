#include "hacdcpf/sppt/intelligent_simulation.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::sppt {

IntelligentSimulationDecision evaluate_intelligent_simulation_contract(
    const IntelligentSimulationContract& contract,
    const IntelligentSimulationEvidence& evidence) {
  IntelligentSimulationDecision decision;

  if (contract.id.empty()) decision.reasons.push_back("analysis contract id is empty");
  if (!evidence.guard.accepted)
    decision.reasons.push_back("SPPT model/observable guard rejected the request");
  if (!contract.allow_approximate_projection && !evidence.projection.exact())
    decision.reasons.push_back("approximate projection is outside the contract");
  if (!evidence.solver_converged)
    decision.reasons.push_back("numerical solver did not converge");
  if (!std::isfinite(evidence.solver_residual) ||
      evidence.solver_residual > contract.max_solver_residual)
    decision.reasons.push_back("solver residual exceeds the contract tolerance");

  if (contract.require_independent_residual) {
    if (!evidence.independent.supported)
      decision.reasons.push_back("independent authored-equation certificate is unavailable");
    else if (!evidence.independent.converged ||
             !std::isfinite(evidence.independent.total_residual) ||
             evidence.independent.total_residual > contract.max_independent_residual)
      decision.reasons.push_back("independent authored-equation residual exceeds tolerance");
  }

  if (contract.require_uncertainty_declaration && evidence.uncertainty.empty())
    decision.reasons.push_back("input/model uncertainty is undeclared");
  for (const auto& uncertainty : evidence.uncertainty) {
    if (!uncertainty.declared()) {
      decision.reasons.push_back("uncertainty record is incomplete: " + uncertainty.quantity);
      continue;
    }
    if (uncertainty.relative_uncertainty > contract.max_relative_uncertainty)
      decision.reasons.push_back("relative uncertainty exceeds trust envelope: " +
                                 uncertainty.quantity);
    if (uncertainty.confidence < contract.min_confidence)
      decision.reasons.push_back("confidence is below trust envelope: " +
                                 uncertainty.quantity);
  }

  if (!decision.reasons.empty()) {
    decision.disposition = IntelligentSimulationDisposition::Reject;
    return decision;
  }

  if (contract.authority == ActionAuthority::ReadOnlyAnalysis) {
    decision.disposition = IntelligentSimulationDisposition::AdmitReadOnly;
  } else {
    decision.disposition = IntelligentSimulationDisposition::RequireHumanReview;
    decision.reasons.push_back("state-changing or decision-bearing actions require human authorization");
  }
  return decision;
}

}  // namespace hacdcpf::sppt

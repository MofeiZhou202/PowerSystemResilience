#pragma once

#include <string>
#include <vector>

#include "hacdcpf/sppt/certificate.hpp"
#include "hacdcpf/sppt/guard.hpp"

namespace hacdcpf::sppt {

enum class AnalysisDomain {
  SteadyStateHybridPowerFlow,
  OptimalPowerFlow,
  ThreePhasePowerFlow,
  DynamicDae,
  Harmonics,
  ShortCircuit,
  Reliability,
  TemporalOperation,
};

enum class ActionAuthority {
  ReadOnlyAnalysis,
  ModelEdit,
  AdvisoryDecision,
  PhysicalControl,
};

enum class UncertaintyKind {
  Deterministic,
  StandardDeviation,
  Interval,
  ScenarioEnsemble,
};

struct UncertainQuantity {
  std::string quantity;
  UncertaintyKind kind{UncertaintyKind::Deterministic};
  double relative_uncertainty{0.0};
  double confidence{1.0};
  std::string provenance;

  [[nodiscard]] bool declared() const noexcept {
    return !quantity.empty() && !provenance.empty() && confidence >= 0.0 &&
           confidence <= 1.0 && relative_uncertainty >= 0.0;
  }
};

struct IntelligentSimulationContract {
  std::string id;
  AnalysisDomain domain{AnalysisDomain::SteadyStateHybridPowerFlow};
  ActionAuthority authority{ActionAuthority::ReadOnlyAnalysis};
  std::vector<ObservableKind> requested_observables;
  bool allow_approximate_projection{false};
  bool require_independent_residual{true};
  bool require_uncertainty_declaration{true};
  double max_solver_residual{1e-6};
  double max_independent_residual{1e-6};
  double max_relative_uncertainty{0.10};
  double min_confidence{0.95};
};

struct IntelligentSimulationEvidence {
  GuardVerdict guard;
  ProjectionCertificate projection;
  IndependentResidualCertificate independent;
  bool solver_converged{false};
  double solver_residual{0.0};
  std::vector<UncertainQuantity> uncertainty;
};

enum class IntelligentSimulationDisposition {
  Reject,
  AdmitReadOnly,
  RequireHumanReview,
};

struct IntelligentSimulationDecision {
  IntelligentSimulationDisposition disposition{
      IntelligentSimulationDisposition::Reject};
  std::vector<std::string> reasons;

  [[nodiscard]] bool admitted() const noexcept {
    return disposition != IntelligentSimulationDisposition::Reject;
  }
};

/// Evaluate structural, semantic, numerical, epistemic, and authority gates.
/// This function never executes a physical control action; such actions can only
/// reach RequireHumanReview after all evidence gates pass.
IntelligentSimulationDecision evaluate_intelligent_simulation_contract(
    const IntelligentSimulationContract& contract,
    const IntelligentSimulationEvidence& evidence);

}  // namespace hacdcpf::sppt

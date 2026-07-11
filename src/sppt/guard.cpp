/// sppt/guard.cpp
/// ==============
/// The SPPT admissibility guard (Def. 8.7, Alg. 2) and scripted agent-edit
/// evaluation metrics for the LLM-ready interface (Pillar 5).

#include "hacdcpf/sppt/guard.hpp"

#include <exception>
#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::sppt {
namespace {

bool mentions_reference(const std::string& msg) {
  auto has = [&](const char* n) { return msg.find(n) != std::string::npos; };
  return has("slack") || has("Slack") || has("reference") || has("Reference") ||
         has("swing") || has("Swing");
}

}  // namespace

GuardVerdict guard_system(const HybridPowerSystem& sys) {
  std::vector<ObservableKind> requested{ObservableKind::ACBusVoltage};
  if (!sys.dc.buses.empty()) requested.push_back(ObservableKind::DCBusVoltage);
  if (!sys.vsc_converters.empty()) requested.push_back(ObservableKind::ConverterTransfer);
  if (!sys.ac.switches.empty() || !sys.ac.circuit_breakers.empty())
    requested.push_back(ObservableKind::SwitchTerminalFlow);
  return guard_system(sys, requested);
}

GuardVerdict guard_system(
    const HybridPowerSystem& sys,
    const std::vector<ObservableKind>& requested_observables) {
  GuardVerdict v;

  // ── Gate (i): boundary validation ──────────────────────────────────────────
  validation::ValidationReport rep;
  try {
    rep = validate_full(sys);
  } catch (const std::exception& e) {
    v.validation_ok = false;
    v.details.push_back(std::string("validation threw: ") + e.what());
    v.reason = "validation";
    return v;
  }
  v.validation_ok = rep.is_valid();
  if (!v.validation_ok)
    v.details.push_back("validation: " + rep.summary());

  // ── Gate (ii): well-posedness (unique/sufficient island references) ─────────
  bool reference_error = false;
  for (const auto& issue : rep.issues) {
    if (issue.severity == validation::Severity::Error &&
        mentions_reference(issue.message)) {
      reference_error = true;
      break;
    }
  }
  const powerflow::ConverterCoordinationReport coordination =
      powerflow::evaluate_converter_coordination(sys, true);
  const bool coordination_ok = !coordination.has_blocking_issue();
  v.well_posed = !reference_error && coordination_ok;
  if (!v.well_posed) {
    if (reference_error)
      v.details.push_back("well-posedness: validation reports a missing or conflicting reference");
    for (const auto& issue : coordination.issues) {
      if (issue.severity == powerflow::CoordinationSeverity::Error ||
          issue.severity == powerflow::CoordinationSeverity::Fatal) {
        v.details.push_back("well-posedness[" + issue.rule_id + "]: " + issue.message);
      }
    }
  }

  // ── Gate (iii): attribution totality ────────────────────────────────────────
  v.attribution_total = true;
  try {
    const HybridPowerSystem projected = project_to_canonical_models(sys);
    for (const ObservableKind observable : requested_observables) {
      ObservableAttribution coverage =
          evaluate_attribution(sys, projected, observable);
      if (!coverage.total()) {
        v.attribution_total = false;
        v.details.push_back("attribution: " + coverage.reason + " (" +
                            std::to_string(coverage.attributed_entities) + "/" +
                            std::to_string(coverage.canonical_entities) + ")");
      }
      v.attribution.push_back(std::move(coverage));
    }
  } catch (const std::exception& e) {
    v.attribution_total = false;
    v.details.push_back(std::string("attribution: projection threw: ") + e.what());
  }

  // ── Verdict ─────────────────────────────────────────────────────────────────
  v.accepted = v.validation_ok && v.well_posed && v.attribution_total;
  if (!v.accepted) {
    if (!v.validation_ok) v.reason = "validation";
    else if (!v.well_posed) v.reason = "ill-posed reference";
    else v.reason = "unattributable";
  }
  return v;
}

// ── Metrics ──────────────────────────────────────────────────────────────────
double GuardMetrics::precision() const {
  const int d = tp + fp;
  return d > 0 ? static_cast<double>(tp) / d : 0.0;
}
double GuardMetrics::recall() const {
  const int d = tp + fn;
  return d > 0 ? static_cast<double>(tp) / d : 0.0;
}
double GuardMetrics::catch_rate() const {
  const int d = tn + fp;
  return d > 0 ? static_cast<double>(tn) / d : 0.0;
}
double GuardMetrics::accuracy() const {
  const int d = total();
  return d > 0 ? static_cast<double>(tp + tn) / d : 0.0;
}

GuardMetrics evaluate_guard(const std::vector<LabeledEdit>& edits) {
  GuardMetrics m;
  for (const auto& e : edits) {
    const bool accepted = guard_system(e.system).accepted;
    if (e.expected_admissible && accepted) ++m.tp;
    else if (!e.expected_admissible && accepted) ++m.fp;
    else if (!e.expected_admissible && !accepted) ++m.tn;
    else ++m.fn;
  }
  return m;
}

}  // namespace hacdcpf::sppt

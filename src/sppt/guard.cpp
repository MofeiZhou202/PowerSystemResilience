/// sppt/guard.cpp
/// ==============
/// The SPPT admissibility guard (Def. 8.7, Alg. 2) and its LLM-in-the-loop
/// evaluation metrics (Pillar 5).

#include "hacdcpf/sppt/guard.hpp"

#include <exception>
#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::sppt {
namespace {

bool mentions_reference(const std::string& msg) {
  auto has = [&](const char* n) { return msg.find(n) != std::string::npos; };
  return has("slack") || has("Slack") || has("reference") || has("Reference") ||
         has("swing") || has("Swing");
}

// A structural sufficient condition that each DC island can pin its voltage:
// there is a declared DC_V reference bus, or a converter/DC-DC device that can
// form (or be auto-promoted to form) Vdc.  Vacuously true with no DC network.
bool dc_reference_available(const HybridPowerSystem& sys) {
  if (sys.dc.buses.empty()) return true;
  for (const auto& b : sys.dc.buses)
    if (b.in_service && b.bus_type == DCBusType::DC_V) return true;
  for (const auto& c : sys.vsc_converters)
    if (c.in_service) return true;  // PQ converters auto-promote if in service
  for (const auto& c : sys.dc.dcdc_converters)
    if (c.in_service) return true;  // DC/DC can regulate a port if in service
  return false;
}

}  // namespace

GuardVerdict guard_system(const HybridPowerSystem& sys) {
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
  const bool dc_ok = dc_reference_available(sys);
  v.well_posed = !reference_error && dc_ok;
  if (!v.well_posed) {
    v.details.push_back(reference_error
                            ? "well-posedness: AC island missing angle reference"
                            : "well-posedness: DC island has no voltage reference");
  }

  // ── Gate (iii): attribution totality ────────────────────────────────────────
  v.attribution_total = true;  // identity attribution is total by default
  try {
    powerflow::SolverData sd = powerflow::make_solver_data(sys);
    if (sd.bus_merge_map) {
      const int covered = sd.bus_merge_map->n_original;
      if (covered != n_ac_buses(sys)) {
        v.attribution_total = false;
        v.details.push_back("attribution: merge map covers " +
                            std::to_string(covered) + " of " +
                            std::to_string(n_ac_buses(sys)) + " buses");
      }
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

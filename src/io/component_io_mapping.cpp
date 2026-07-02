#include "hacdcpf/io/component_io_mapping.hpp"

#include <algorithm>
#include <array>
#include <sstream>
#include <utility>

namespace hacdcpf::io {

namespace {

using D = ComponentIODomain;
using P = ComponentIOPolicy;
using V = NumericalVerificationScope;
using S = ComponentStandardFamily;

ComponentIOMapping row(std::string component_type,
                       std::string collection_path,
                       D domain,
                       P json_policy,
                       P canonical_policy,
                       P gridlabd_policy,
                       P opendss_policy,
                       V verification_scope,
                       std::string canonical_target,
                       std::string gridlabd_target,
                       std::string opendss_target,
                       std::string notes) {
  ComponentIOMapping mapping;
  mapping.component_type = std::move(component_type);
  mapping.collection_path = std::move(collection_path);
  mapping.domain = domain;
  mapping.json_policy = json_policy;
  mapping.canonical_policy = canonical_policy;
  mapping.gridlabd_policy = gridlabd_policy;
  mapping.opendss_policy = opendss_policy;
  mapping.verification_scope = verification_scope;
  mapping.canonical_target = std::move(canonical_target);
  mapping.gridlabd_target = std::move(gridlabd_target);
  mapping.opendss_target = std::move(opendss_target);
  mapping.notes = std::move(notes);
  return mapping;
}

ComponentStandardProfile profile(S family,
                                 std::string profile_name,
                                 std::string model_name,
                                 P policy,
                                 V verification_scope,
                                 std::string notes) {
  ComponentStandardProfile item;
  item.family = family;
  item.profile = std::move(profile_name);
  item.model_name = std::move(model_name);
  item.policy = policy;
  item.verification_scope = verification_scope;
  item.notes = std::move(notes);
  return item;
}

void add_profile(std::vector<ComponentIOMapping>& mappings,
                 const std::string& component_type,
                 ComponentStandardProfile item) {
  const auto it = std::find_if(
      mappings.begin(), mappings.end(), [&](const ComponentIOMapping& mapping) {
        return mapping.component_type == component_type;
      });
  if (it != mappings.end()) {
    it->standard_profiles.push_back(std::move(item));
  }
}

void add_standard_profiles(std::vector<ComponentIOMapping>& mappings) {
  add_profile(mappings,
              "ACBus",
              profile(S::IEC61970CIM,
                      "CIM EquipmentCore/Topology",
                      "ConnectivityNode/TopologicalNode",
                      P::Equivalent,
                      V::ExternalPowerFlow,
                      "Use CIM naming, terminals, base-voltage, and "
                      "topological-node identity as the long-term exchange "
                      "contract."));
  add_profile(mappings,
              "ACBranch",
              profile(S::IEC61970CIM,
                      "CIM Wires",
                      "ACLineSegment",
                      P::Equivalent,
                      V::ExternalPowerFlow,
                      "Per-length and sequence impedance metadata should be "
                      "preserved before exact unbalanced exchange is claimed."));
  add_profile(mappings,
              "Transformer2W",
              profile(S::IEC61970CIM,
                      "CIM Wires",
                      "PowerTransformer/TransformerTank",
                      P::Equivalent,
                      V::ExternalPowerFlow,
                      "Nameplate winding bases, tap changer, and regulator "
                      "metadata are the next exact-roundtrip fields."));
  add_profile(mappings,
              "RegulatorControl",
              profile(S::IEC61850,
                      "IEC 61850 logical nodes",
                      "ATCC/TapChangerControl",
                      P::Projected,
                      V::NativeSolver,
                      "Control intent is native today; external text export "
                      "still needs tap sequence replay."));
  add_profile(mappings,
              "Generator",
              profile(S::IEEE,
                      "IEEE synchronous-machine transient family",
                      "GENROU/GENSAL classical-compatible subset",
                      P::Projected,
                      V::NativeSolver,
                      "Static generator records initialize machine dynamic "
                      "states; detailed d/q saturation parameters are future "
                      "profile fields."));
  add_profile(mappings,
              "Generator",
              profile(S::IEEE4215,
                      "IEEE 421.5 excitation systems",
                      "EXAC*/ESAC*/ST* profile placeholder",
                      P::Projected,
                      V::NativeSolver,
                      "Exciter blocks are exposed in dynamics; rich JSON still "
                      "needs named parameter-set storage."));
  add_profile(mappings,
              "Generator",
              profile(S::IEEE,
                      "IEEE turbine-governor families",
                      "TGOV1/HYGOV/GAST/IEESGO profile placeholder",
                      P::Projected,
                      V::NativeSolver,
                      "Governor blocks are exposed in dynamics; exact thermal, "
                      "hydro, and nuclear profiles are planned."));
  add_profile(mappings,
              "AsynchronousMotor",
              profile(S::IEC60909,
                      "IEC 60909 short-circuit contribution",
                      "asynchronous motor equivalent",
                      P::Projected,
                      V::NativeSolver,
                      "Short-circuit contribution fields are already native; "
                      "dynamic motor startup models remain future work."));
  add_profile(mappings,
              "PVSystem",
              profile(S::IEEE1547,
                      "IEEE 1547 grid-support functions",
                      "PV current-source DER subset",
                      P::Projected,
                      V::BoundaryInjectionSnapshot,
                      "Volt-var/frequency-watt semantics should map into the "
                      "GFL inverter control profile for transient exchange."));
  add_profile(mappings,
              "RenewableGen",
              profile(S::NERC,
                      "NERC generic renewable models",
                      "REGC_A/REEC_A/REPC_A boundary subset",
                      P::Projected,
                      V::BoundaryInjectionSnapshot,
                      "Use NERC-style converter/electrical/plant-controller "
                      "taxonomy for wind and PV dynamic IO."));
  add_profile(mappings,
              "Storage",
              profile(S::IEEE1547,
                      "IEEE 1547 DER functions",
                      "BESS GFL/GFM dynamic subset",
                      P::Projected,
                      V::NativeSolver,
                      "SOC and power response are native; exact ride-through "
                      "and grid-support profile fields are future work."));
  add_profile(mappings,
              "VSCConverter",
              profile(S::NERC,
                      "NERC inverter-based resource generic models",
                      "REGC_A/REEC_A/REPC_A or GFM profile",
                      P::Projected,
                      V::NativeSolver,
                      "The dynamic module now exposes GFL/GFM model families; "
                      "external transient equivalence needs event replay tests."));
  add_profile(mappings,
              "DCDCConverter",
              profile(S::HACDCPF,
                      "Hybrid AC/DC native converter profile",
                      "DCDCConverterDynamic",
                      P::Exact,
                      V::NativeSolver,
                      "No common GridLAB-D/OpenDSS exchange target is assumed "
                      "for DC/DC dynamics yet."));
  add_profile(mappings,
              "ThreePhaseACLine",
              profile(S::IEC61970CIM,
                      "CIM phase-domain Wires",
                      "ACLineSegment with phase impedance matrix",
                      P::Projected,
                      V::ExternalPowerFlow,
                      "Exact phase-domain exchange requires matrix impedance "
                      "and phase connectivity export/import."));
}

bool is_unsupported_external_policy(ComponentIOPolicy policy) {
  return policy == P::Unsupported || policy == P::InternalOnly ||
         policy == P::DiagnosticOnly;
}

const std::vector<ComponentIOMapping>& registry() {
  static const std::vector<ComponentIOMapping> mappings = [] {
    std::vector<ComponentIOMapping> mappings = {
      row("ACBus", "ac.buses", D::AC, P::Exact, P::Exact, P::Exact, P::Exact,
          V::ExternalPowerFlow, "ACBus", "node/meter/load bus", "Bus",
          "Balanced AC bus maps directly; GridLAB-D uses meter for swing and "
          "load/node for PQ buses."),
      row("ACBranch", "ac.branches", D::AC, P::Exact, P::Exact, P::Exact,
          P::Exact, V::ExternalPowerFlow, "ACBranch", "overhead_line/line",
          "Line",
          "Balanced positive-sequence branch maps to a line; transformer-like "
          "branches may export as transformers."),
      row("Generator", "ac.generators", D::AC, P::Exact, P::Exact,
          P::BoundaryInjection, P::Equivalent, V::ExternalPowerFlow,
          "Generator", "negative load or swing meter", "Vsource/Generator",
          "Slack generators map to a source; non-slack PV bus voltage "
          "regulation is reduced to PQ injection in text snapshots."),
      row("StaticGenerator", "ac.static_generators", D::AC, P::Exact,
          P::Exact, P::BoundaryInjection, P::Equivalent, V::ExternalPowerFlow,
          "StaticGenerator", "negative constant_power load", "Generator",
          "Static generation is represented as constant P/Q injection."),
      row("Load", "ac.loads", D::AC, P::Exact, P::Exact, P::Exact, P::Exact,
          V::ExternalPowerFlow, "Load", "load.constant_power_*", "Load",
          "Constant-power load round-trips through the balanced subset."),
      row("FlexibleLoad", "ac.flexible_loads", D::AC, P::Exact, P::Projected,
          P::Aggregated, P::Aggregated, V::EquivalentRoundTrip, "Load",
          "load.constant_power_*", "Load",
          "Demand-response capability is kept in JSON and projected to a "
          "constant-power load for static external snapshots."),
      row("AsymmetricLoad", "ac.asymmetric_loads", D::AC, P::Exact,
          P::Projected, P::Aggregated, P::Aggregated, V::EquivalentRoundTrip,
          "Load", "balanced load", "balanced Load",
          "Phase detail is preserved in JSON but aggregated for balanced AC "
          "external formats."),
      row("Shunt", "ac.shunts", D::AC, P::Exact, P::Exact, P::Aggregated,
          P::Equivalent, V::EquivalentRoundTrip, "Shunt",
          "constant-power load equivalent", "Capacitor/Reactor or equivalent",
          "The current text harness folds shunts into bus demand/admittance "
          "equivalents rather than preserving switchable controls."),
      row("Storage", "ac.storage", D::AC, P::Exact, P::Exact,
          P::BoundaryInjection, P::BoundaryInjection, V::ExternalPowerFlow,
          "Storage", "negative load or load", "Storage/Generator equivalent",
          "Snapshot export preserves P/Q injection; SOC dynamics stay native."),
      row("RenewableGen", "ac.renewable_gens", D::AC, P::Exact, P::Exact,
          P::BoundaryInjection, P::BoundaryInjection, V::ExternalPowerFlow,
          "RenewableGen", "negative load", "Generator/PVSystem equivalent",
          "Technology metadata is native JSON; external text sees a PQ "
          "renewable injection."),
      row("PVSystem", "ac.pv_systems", D::AC, P::Exact, P::Exact,
          P::BoundaryInjection, P::Equivalent, V::ExternalPowerFlow,
          "PVSystem", "negative load", "PVSystem/Generator",
          "PV electrical model and controls are native; external static "
          "snapshots use P/Q injection."),
      row("ExternalGrid", "ac.external_grids", D::AC, P::Exact, P::Exact,
          P::Equivalent, P::Equivalent, V::ExternalPowerFlow, "ExternalGrid",
          "swing meter", "Vsource/Circuit source",
          "Short-circuit metadata is native; external text uses source voltage."),
      row("Transformer2W", "ac.transformers_2w", D::AC, P::Exact,
          P::Projected, P::Equivalent, P::Equivalent, V::ExternalPowerFlow,
          "ACBranch", "transformer", "Transformer",
          "Projected to an equivalent branch for native solvers and exported "
          "as a two-winding transformer when possible."),
      row("Transformer3W", "ac.transformers_3w", D::AC, P::Exact,
          P::Projected, P::Equivalent, P::Equivalent, V::EquivalentRoundTrip,
          "three ACBranch equivalents", "multiple transformer/line equivalents",
          "multiple Transformer equivalents",
          "Three-winding transformers are reduced to pairwise branches in the "
          "canonical model."),
      row("RegulatorControl", "ac.regulator_controls", D::AC, P::Exact,
          P::Exact, P::Unsupported, P::DiagnosticOnly, V::NativeSolver,
          "RegulatorControl", "", "RegControl",
          "Native distribution PF uses regulator controls; the current text "
          "harness does not export tap-control sequences yet."),
      row("Switch", "ac.switches", D::AC, P::Exact, P::Projected,
          P::Equivalent, P::Equivalent, V::EquivalentRoundTrip, "ACBranch",
          "switch/line equivalent", "Switch/Line equivalent",
          "Closed/open state is projected to an equivalent branch for static "
          "solvers."),
      row("CircuitBreaker", "ac.circuit_breakers", D::AC, P::Exact,
          P::Projected, P::Equivalent, P::Equivalent, V::EquivalentRoundTrip,
          "ACBranch", "switch/line equivalent", "Switch/Line equivalent",
          "Breaker state is projected to an equivalent branch for static "
          "external snapshots."),
      row("ChargingStation", "ac.charging_stations", D::AC, P::Exact,
          P::Aggregated, P::Aggregated, P::Aggregated, V::ExternalPowerFlow,
          "Load", "load.constant_power_*", "Load",
          "Site demand is represented as aggregate P/Q demand externally."),
      row("Charger", "ac.chargers", D::AC, P::Exact, P::Aggregated,
          P::Aggregated, P::Aggregated, V::EquivalentRoundTrip,
          "ChargingStation", "load.constant_power_*", "Load",
          "Individual chargers roll up into charging stations before static "
          "network solution."),
      row("AsynchronousMotor", "ac.motors", D::AC, P::Exact, P::Projected,
          P::Aggregated, P::Aggregated, V::EquivalentRoundTrip, "Load",
          "load.constant_power_*", "Load",
          "Motor dynamic and short-circuit metadata is native; static external "
          "snapshots use equivalent demand."),
      row("DCBus", "dc.buses", D::DC, P::Exact, P::Exact, P::InternalOnly,
          P::Unsupported, V::NativeSolver, "DCBus", "", "",
          "Native hybrid solvers represent DC buses; GridLAB-D/OpenDSS text "
          "exports are currently AC-scope snapshots."),
      row("DCBranch", "dc.branches", D::DC, P::Exact, P::Exact,
          P::InternalOnly, P::Unsupported, V::NativeSolver, "DCBranch", "", "",
          "DC network equations are verified by native solvers."),
      row("DCLoad", "dc.loads", D::DC, P::Exact, P::Exact, P::InternalOnly,
          P::Unsupported, V::NativeSolver, "DCLoad", "", "",
          "DC loads stay in the native hybrid/DC model."),
      row("Storage", "dc.storage", D::DC, P::Exact, P::Exact,
          P::InternalOnly, P::Unsupported, V::NativeSolver, "Storage", "", "",
          "DC-side materialized storage reuses the Storage schema with active "
          "power only in DC analysis."),
      row("DCStorage", "dc.dc_storage", D::DC, P::Exact, P::Projected,
          P::InternalOnly, P::Unsupported, V::NativeSolver, "Storage", "", "",
          "DCStorage materializes to the native DC storage table for solvers."),
      row("StaticGenerator", "dc.static_generators", D::DC, P::Exact,
          P::Exact, P::InternalOnly, P::Unsupported, V::NativeSolver,
          "StaticGenerator", "", "",
          "Legacy DC static generation vector reuses the AC StaticGenerator "
          "schema and remains native-only externally."),
      row("StaticGeneratorDC", "dc.dc_static_generators", D::DC, P::Exact,
          P::Exact, P::InternalOnly, P::Unsupported, V::NativeSolver,
          "StaticGeneratorDC", "", "",
          "DC static generation is native-only in the current external harness."),
      row("PVArrayDC", "dc.pv_arrays", D::DC, P::Exact, P::Exact,
          P::InternalOnly, P::Unsupported, V::NativeSolver, "PVArrayDC", "",
          "",
          "PV array electrical parameters are preserved in JSON and native DC "
          "analysis."),
      row("DCDCConverter", "dc.dcdc_converters", D::DC, P::Exact, P::Exact,
          P::InternalOnly, P::Unsupported, V::NativeSolver, "DCDCConverter",
          "", "",
          "DC/DC converters are native hybrid solver components."),
      row("DCCircuitBreaker", "dc.dc_circuit_breakers", D::DC, P::Exact,
          P::Projected, P::InternalOnly, P::Unsupported, V::NativeSolver,
          "DCBranch/topology device", "", "",
          "DC breaker state is kept native until a DC external text target is "
          "added."),
      row("VSCConverter", "vsc_converters", D::Hybrid, P::Exact, P::Exact,
          P::BoundaryInjection, P::Unsupported, V::BoundaryInjectionSnapshot,
          "VSCConverter", "scheduled AC boundary injection", "",
          "Native solver keeps AC/DC coupling; GridLAB-D can optionally see the "
          "AC boundary injection only."),
      row("EnergyRouter", "energy_routers", D::Hybrid, P::Exact, P::Projected,
          P::BoundaryInjection, P::Unsupported, V::NativeSolver,
          "VSCConverter/DCDCConverter expansion", "boundary injection", "",
          "Energy routers expand to internal VSC/DC/DC primitives natively."),
      row("MobileStorage", "mobile_storage", D::Hybrid, P::Exact,
          P::Projected, P::BoundaryInjection, P::BoundaryInjection,
          V::EquivalentRoundTrip, "Storage", "load/generator equivalent",
          "Storage/Generator equivalent",
          "Connected mobile storage projects to static storage; transit state "
          "is native metadata."),
      row("VirtualPowerPlant", "vpps", D::Hybrid, P::Exact, P::Projected,
          P::BoundaryInjection, P::BoundaryInjection, V::ExternalPowerFlow,
          "StaticGenerator", "negative load", "Generator equivalent",
          "VPP fleet detail is native; grid exchange exports as a PCC "
          "injection."),
      row("Microgrid", "microgrids", D::Hybrid, P::Exact, P::Projected,
          P::BoundaryInjection, P::BoundaryInjection, V::ExternalPowerFlow,
          "StaticGenerator", "negative load", "Generator equivalent",
          "Grid-connected exchange exports as a PCC injection; islanded "
          "microgrid internals remain native."),
      row("ThreePhaseACBus", "three_phase_ac.buses", D::ThreePhaseAC, P::Exact,
          P::Projected, P::Equivalent, P::Equivalent, V::ExternalPowerFlow,
          "ACBus", "node/meter/load bus", "Bus",
          "Phase-domain bus is preserved in JSON and can be reduced to "
          "balanced AC when needed."),
      row("ThreePhaseACLine", "three_phase_ac.lines", D::ThreePhaseAC,
          P::Exact, P::Projected, P::Equivalent, P::Equivalent,
          V::ExternalPowerFlow, "ACBranch", "overhead_line", "Line/LineCode",
          "Phase matrix data is exact in JSON; balanced export uses an "
          "equivalent sequence line."),
      row("ThreePhaseTransformer", "three_phase_ac.transformers",
          D::ThreePhaseAC, P::Exact, P::Projected, P::Equivalent,
          P::Equivalent, V::EquivalentRoundTrip, "Transformer2W/ACBranch",
          "transformer", "Transformer",
          "Three-phase transformer is reduced to a balanced transformer or "
          "branch for external static snapshots."),
      row("ThreePhaseLoad", "three_phase_ac.loads", D::ThreePhaseAC, P::Exact,
          P::Projected, P::Aggregated, P::Equivalent, V::ExternalPowerFlow,
          "Load", "balanced load", "Load",
          "Per-phase loads are preserved in JSON and aggregated for balanced "
          "snapshots."),
      row("ThreePhaseGenerator", "three_phase_ac.generators", D::ThreePhaseAC,
          P::Exact, P::Projected, P::BoundaryInjection, P::Equivalent,
          V::ExternalPowerFlow, "Generator", "negative load/swing meter",
          "Generator",
          "Phase generator data is reduced to balanced P/Q or source control."),
      row("ThreePhaseExternalGrid", "three_phase_ac.external_grids",
          D::ThreePhaseAC, P::Exact, P::Projected, P::Equivalent,
          P::Equivalent, V::ExternalPowerFlow, "ExternalGrid", "swing meter",
          "Vsource",
          "Phase-domain source impedance is native; balanced export keeps the "
          "positive/zero sequence equivalent where possible."),
      row("ThreePhaseRegulatorControl",
          "three_phase_ac.regulator_controls", D::ThreePhaseAC, P::Exact,
          P::Exact, P::Unsupported, P::DiagnosticOnly, V::NativeSolver,
          "ThreePhaseRegulatorControl", "", "RegControl",
          "Native three-phase PF uses regulator controls; external text export "
          "support is planned."),
    };
    add_standard_profiles(mappings);
    return mappings;
  }();
  return mappings;
}

std::size_t count_for_path(const HybridPowerSystem& sys,
                           const std::string& path) {
  if (path == "ac.buses") return sys.ac.buses.size();
  if (path == "ac.branches") return sys.ac.branches.size();
  if (path == "ac.generators") return sys.ac.generators.size();
  if (path == "ac.static_generators") return sys.ac.static_generators.size();
  if (path == "ac.loads") return sys.ac.loads.size();
  if (path == "ac.flexible_loads") return sys.ac.flexible_loads.size();
  if (path == "ac.asymmetric_loads") return sys.ac.asymmetric_loads.size();
  if (path == "ac.shunts") return sys.ac.shunts.size();
  if (path == "ac.storage") return sys.ac.storage.size();
  if (path == "ac.renewable_gens") return sys.ac.renewable_gens.size();
  if (path == "ac.pv_systems") return sys.ac.pv_systems.size();
  if (path == "ac.external_grids") return sys.ac.external_grids.size();
  if (path == "ac.transformers_2w") return sys.ac.transformers_2w.size();
  if (path == "ac.transformers_3w") return sys.ac.transformers_3w.size();
  if (path == "ac.regulator_controls") return sys.ac.regulator_controls.size();
  if (path == "ac.switches") return sys.ac.switches.size();
  if (path == "ac.circuit_breakers") return sys.ac.circuit_breakers.size();
  if (path == "ac.charging_stations") return sys.ac.charging_stations.size();
  if (path == "ac.chargers") return sys.ac.chargers.size();
  if (path == "ac.motors") return sys.ac.motors.size();
  if (path == "dc.buses") return sys.dc.buses.size();
  if (path == "dc.branches") return sys.dc.branches.size();
  if (path == "dc.loads") return sys.dc.loads.size();
  if (path == "dc.storage") return sys.dc.storage.size();
  if (path == "dc.dc_storage") return sys.dc.dc_storage.size();
  if (path == "dc.static_generators") return sys.dc.static_generators.size();
  if (path == "dc.dc_static_generators") {
    return sys.dc.dc_static_generators.size();
  }
  if (path == "dc.pv_arrays") return sys.dc.pv_arrays.size();
  if (path == "dc.dcdc_converters") return sys.dc.dcdc_converters.size();
  if (path == "dc.dc_circuit_breakers") {
    return sys.dc.dc_circuit_breakers.size();
  }
  if (path == "vsc_converters") return sys.vsc_converters.size();
  if (path == "energy_routers") return sys.energy_routers.size();
  if (path == "mobile_storage") return sys.mobile_storage.size();
  if (path == "vpps") return sys.vpps.size();
  if (path == "microgrids") return sys.microgrids.size();
  if (!sys.three_phase_ac.has_value()) return 0;
  const auto& tp = *sys.three_phase_ac;
  if (path == "three_phase_ac.buses") return tp.buses.size();
  if (path == "three_phase_ac.lines") return tp.lines.size();
  if (path == "three_phase_ac.transformers") return tp.transformers.size();
  if (path == "three_phase_ac.loads") return tp.loads.size();
  if (path == "three_phase_ac.generators") return tp.generators.size();
  if (path == "three_phase_ac.external_grids") {
    return tp.external_grids.size();
  }
  if (path == "three_phase_ac.regulator_controls") {
    return tp.regulator_controls.size();
  }
  return 0;
}

}  // namespace

const std::vector<ComponentIOMapping>& component_io_mappings() {
  return registry();
}

std::optional<ComponentIOMapping> find_component_io_mapping(
    std::string_view component_type) {
  const auto& mappings = registry();
  const auto it = std::find_if(
      mappings.begin(), mappings.end(), [&](const ComponentIOMapping& item) {
        return item.component_type == component_type;
      });
  if (it == mappings.end()) return std::nullopt;
  return *it;
}

ComponentIOCoverageReport analyze_component_io_coverage(
    const HybridPowerSystem& sys) {
  ComponentIOCoverageReport report;
  for (const auto& mapping : registry()) {
    report.items.push_back({mapping, count_for_path(sys, mapping.collection_path)});
  }
  if (sys.dc.storage.size() != 0U) {
    report.diagnostics.push_back(
        "dc.storage reuses the AC Storage schema; treat it as native DC storage "
        "state, not a separate component type.");
  }
  return report;
}

bool is_represented_policy(ComponentIOPolicy policy) {
  return !is_unsupported_external_policy(policy);
}

ComponentIOPolicy policy_for_format(const ComponentIOMapping& mapping,
                                    ComponentIOFormat format) {
  switch (format) {
    case ComponentIOFormat::InternalJSON:
      return mapping.json_policy;
    case ComponentIOFormat::CanonicalModel:
      return mapping.canonical_policy;
    case ComponentIOFormat::GridLABD:
      return mapping.gridlabd_policy;
    case ComponentIOFormat::OpenDSS:
      return mapping.opendss_policy;
  }
  return ComponentIOPolicy::Unsupported;
}

std::size_t ComponentIOCoverageReport::total_instances() const {
  std::size_t total = 0;
  for (const auto& item : items) total += item.count;
  return total;
}

std::size_t ComponentIOCoverageReport::represented_instances(
    ComponentIOFormat format) const {
  std::size_t total = 0;
  for (const auto& item : items) {
    if (is_represented_policy(policy_for_format(item.mapping, format))) {
      total += item.count;
    }
  }
  return total;
}

std::size_t ComponentIOCoverageReport::unrepresented_instances(
    ComponentIOFormat format) const {
  std::size_t total = 0;
  for (const auto& item : items) {
    if (!is_represented_policy(policy_for_format(item.mapping, format))) {
      total += item.count;
    }
  }
  return total;
}

std::string to_string(ComponentIODomain domain) {
  switch (domain) {
    case ComponentIODomain::AC:
      return "AC";
    case ComponentIODomain::DC:
      return "DC";
    case ComponentIODomain::Hybrid:
      return "Hybrid";
    case ComponentIODomain::ThreePhaseAC:
      return "ThreePhaseAC";
  }
  return "Unknown";
}

std::string to_string(ComponentIOFormat format) {
  switch (format) {
    case ComponentIOFormat::InternalJSON:
      return "InternalJSON";
    case ComponentIOFormat::CanonicalModel:
      return "CanonicalModel";
    case ComponentIOFormat::GridLABD:
      return "GridLABD";
    case ComponentIOFormat::OpenDSS:
      return "OpenDSS";
  }
  return "Unknown";
}

std::string to_string(ComponentIOPolicy policy) {
  switch (policy) {
    case ComponentIOPolicy::Exact:
      return "Exact";
    case ComponentIOPolicy::Equivalent:
      return "Equivalent";
    case ComponentIOPolicy::Aggregated:
      return "Aggregated";
    case ComponentIOPolicy::BoundaryInjection:
      return "BoundaryInjection";
    case ComponentIOPolicy::Projected:
      return "Projected";
    case ComponentIOPolicy::InternalOnly:
      return "InternalOnly";
    case ComponentIOPolicy::Unsupported:
      return "Unsupported";
    case ComponentIOPolicy::DiagnosticOnly:
      return "DiagnosticOnly";
  }
  return "Unknown";
}

std::string to_string(NumericalVerificationScope scope) {
  switch (scope) {
    case NumericalVerificationScope::ExactRoundTrip:
      return "ExactRoundTrip";
    case NumericalVerificationScope::EquivalentRoundTrip:
      return "EquivalentRoundTrip";
    case NumericalVerificationScope::NativeSolver:
      return "NativeSolver";
    case NumericalVerificationScope::ExternalPowerFlow:
      return "ExternalPowerFlow";
    case NumericalVerificationScope::BoundaryInjectionSnapshot:
      return "BoundaryInjectionSnapshot";
    case NumericalVerificationScope::StructuralOnly:
      return "StructuralOnly";
    case NumericalVerificationScope::NotApplicable:
      return "NotApplicable";
  }
  return "Unknown";
}

std::string to_string(ComponentStandardFamily family) {
  switch (family) {
    case ComponentStandardFamily::HACDCPF:
      return "HACDCPF";
    case ComponentStandardFamily::IEC61970CIM:
      return "IEC61970CIM";
    case ComponentStandardFamily::IEC61850:
      return "IEC61850";
    case ComponentStandardFamily::IEC60909:
      return "IEC60909";
    case ComponentStandardFamily::IEEE:
      return "IEEE";
    case ComponentStandardFamily::IEEE1547:
      return "IEEE1547";
    case ComponentStandardFamily::IEEE4215:
      return "IEEE4215";
    case ComponentStandardFamily::NERC:
      return "NERC";
    case ComponentStandardFamily::GridLABD:
      return "GridLABD";
    case ComponentStandardFamily::OpenDSS:
      return "OpenDSS";
  }
  return "Unknown";
}

std::vector<std::string> external_io_diagnostics(
    const ComponentIOCoverageReport& report,
    ComponentIOFormat format) {
  std::vector<std::string> diagnostics;
  for (const auto& item : report.items) {
    if (item.count == 0U) continue;
    const auto policy = policy_for_format(item.mapping, format);
    if (is_represented_policy(policy)) continue;
    std::ostringstream os;
    os << item.mapping.collection_path << " contains " << item.count << " "
       << item.mapping.component_type << " object(s), but " << to_string(format)
       << " policy is " << to_string(policy) << ".";
    if (!item.mapping.notes.empty()) os << " " << item.mapping.notes;
    diagnostics.push_back(os.str());
  }
  return diagnostics;
}

}  // namespace hacdcpf::io

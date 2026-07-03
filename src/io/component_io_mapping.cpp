#include "hacdcpf/io/component_io_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <map>
#include <sstream>
#include <utility>

namespace hacdcpf::io {

namespace {

using D = ComponentIODomain;
using P = ComponentIOPolicy;
using V = NumericalVerificationScope;
using S = ComponentStandardFamily;
using C = ComponentParameterCategory;
using Sev = ComponentParameterSeverity;
using DT = DigitalTwinDimension;

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

ComponentParameterRule parameter_rule(std::string component_type,
                                      std::string collection_path,
                                      std::string parameter_path,
                                      C category,
                                      std::optional<double> min_value,
                                      std::optional<double> max_value,
                                      S standard_family,
                                      std::string standard_profile,
                                      std::string units,
                                      bool required = false,
                                      Sev range_severity = Sev::Warning,
                                      Sev missing_severity = Sev::Info,
                                      std::string notes = {},
                                      bool min_inclusive = true,
                                      bool max_inclusive = true) {
  ComponentParameterRule rule;
  rule.component_type = std::move(component_type);
  rule.collection_path = std::move(collection_path);
  rule.parameter_path = std::move(parameter_path);
  rule.category = category;
  rule.min_value = min_value;
  rule.max_value = max_value;
  rule.min_inclusive = min_inclusive;
  rule.max_inclusive = max_inclusive;
  rule.required = required;
  rule.standard_family = standard_family;
  rule.standard_profile = std::move(standard_profile);
  rule.units = std::move(units);
  rule.missing_severity = missing_severity;
  rule.range_severity = range_severity;
  rule.notes = std::move(notes);
  return rule;
}

ComponentParameterRule positive_required(std::string component_type,
                                         std::string collection_path,
                                         std::string parameter_path,
                                         C category,
                                         S standard_family,
                                         std::string standard_profile,
                                         std::string units = {},
                                         Sev severity = Sev::Error) {
  return parameter_rule(std::move(component_type),
                        std::move(collection_path),
                        std::move(parameter_path),
                        category,
                        0.0,
                        std::nullopt,
                        standard_family,
                        std::move(standard_profile),
                        std::move(units),
                        true,
                        severity,
                        severity,
                        {},
                        false);
}

ComponentParameterRule nonnegative_rule(std::string component_type,
                                        std::string collection_path,
                                        std::string parameter_path,
                                        C category,
                                        S standard_family,
                                        std::string standard_profile,
                                        std::string units = {},
                                        bool required = false,
                                        Sev severity = Sev::Warning) {
  return parameter_rule(std::move(component_type),
                        std::move(collection_path),
                        std::move(parameter_path),
                        category,
                        0.0,
                        std::nullopt,
                        standard_family,
                        std::move(standard_profile),
                        std::move(units),
                        required,
                        severity);
}

ComponentParameterRule bounded_rule(std::string component_type,
                                    std::string collection_path,
                                    std::string parameter_path,
                                    C category,
                                    double min_value,
                                    double max_value,
                                    S standard_family,
                                    std::string standard_profile,
                                    std::string units = {},
                                    bool required = false,
                                    Sev severity = Sev::Warning,
                                    bool min_inclusive = true,
                                    bool max_inclusive = true) {
  return parameter_rule(std::move(component_type),
                        std::move(collection_path),
                        std::move(parameter_path),
                        category,
                        min_value,
                        max_value,
                        standard_family,
                        std::move(standard_profile),
                        std::move(units),
                        required,
                        severity,
                        Sev::Info,
                        {},
                        min_inclusive,
                        max_inclusive);
}

ComponentParameterRule presence_rule(std::string component_type,
                                     std::string collection_path,
                                     std::string parameter_path,
                                     C category,
                                     S standard_family,
                                     std::string standard_profile,
                                     Sev severity = Sev::Info,
                                     std::string notes = {}) {
  return parameter_rule(std::move(component_type),
                        std::move(collection_path),
                        std::move(parameter_path),
                        category,
                        std::nullopt,
                        std::nullopt,
                        standard_family,
                        std::move(standard_profile),
                        {},
                        true,
                        severity,
                        severity,
                        std::move(notes));
}

void add_common_dynamic_rules(std::vector<ComponentParameterRule>& rules,
                              const std::string& component_type,
                              const std::string& collection_path,
                              S family,
                              const std::string& profile,
                              Sev missing_severity = Sev::Info) {
  rules.push_back(presence_rule(component_type,
                                collection_path,
                                "dynamic_model",
                                C::Dynamic,
                                family,
                                profile,
                                missing_severity,
                                "Transient validation needs named dynamic "
                                "profiles before external equivalence is "
                                "claimed."));
  rules.push_back(presence_rule(component_type,
                                collection_path,
                                "dynamic_model.standard",
                                C::Dynamic,
                                family,
                                profile,
                                Sev::Info));
  rules.push_back(presence_rule(component_type,
                                collection_path,
                                "dynamic_model.model_name",
                                C::Dynamic,
                                family,
                                profile,
                                Sev::Info));
  rules.push_back(bounded_rule(component_type,
                               collection_path,
                               "dynamic_model.parameters.H",
                               C::Dynamic,
                               0.1,
                               15.0,
                               family,
                               profile,
                               "s"));
  rules.push_back(bounded_rule(component_type,
                               collection_path,
                               "dynamic_model.parameters.D",
                               C::Dynamic,
                               0.0,
                               20.0,
                               family,
                               profile,
                               "pu"));
  rules.push_back(bounded_rule(component_type,
                               collection_path,
                               "dynamic_model.parameters.pll_kp",
                               C::Transient,
                               0.0,
                               100.0,
                               family,
                               profile,
                               "pu"));
  rules.push_back(bounded_rule(component_type,
                               collection_path,
                               "dynamic_model.parameters.pll_ki",
                               C::Transient,
                               0.0,
                               500.0,
                               family,
                               profile,
                               "pu/s"));
}

void add_common_reliability_rules(std::vector<ComponentParameterRule>& rules,
                                  const std::string& component_type,
                                  const std::string& collection_path) {
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "failure_rate",
                                   C::Failure,
                                   S::NERC,
                                   "availability/reliability data",
                                   "1/year"));
  rules.push_back(bounded_rule(component_type,
                               collection_path,
                               "forced_outage_rate",
                               C::Failure,
                               0.0,
                               1.0,
                               S::NERC,
                               "availability/reliability data",
                               "pu"));
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "mttr_hr",
                                   C::Reliability,
                                   S::NERC,
                                   "availability/reliability data",
                                   "h"));
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "mttr_hours",
                                   C::Reliability,
                                   S::NERC,
                                   "availability/reliability data",
                                   "h"));
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "mtbf_hr",
                                   C::Reliability,
                                   S::NERC,
                                   "availability/reliability data",
                                   "h"));
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "mtbf_hours",
                                   C::Reliability,
                                   S::NERC,
                                   "availability/reliability data",
                                   "h"));
  rules.push_back(nonnegative_rule(component_type,
                                   collection_path,
                                   "t_scheduled_hr",
                                   C::Reliability,
                                   S::NERC,
                                   "availability/reliability data",
                                   "h"));
}

const std::vector<ComponentParameterRule>& parameter_registry() {
  static const std::vector<ComponentParameterRule> rules = [] {
    std::vector<ComponentParameterRule> rules;

    rules.push_back(positive_required("ACBus", "ac.buses", "base_kv",
                                      C::Static, S::IEC61970CIM,
                                      "CIM BaseVoltage", "kV"));
    rules.push_back(bounded_rule("ACBus", "ac.buses", "vm_pu", C::Static,
                                 0.5, 1.5, S::IEC61970CIM,
                                 "CIM TopologicalNode voltage", "pu"));
    rules.push_back(bounded_rule("ACBus", "ac.buses", "vmin_pu", C::Static,
                                 0.5, 1.5, S::IEC61970CIM,
                                 "CIM operational limit", "pu"));
    rules.push_back(bounded_rule("ACBus", "ac.buses", "vmax_pu", C::Static,
                                 0.5, 1.5, S::IEC61970CIM,
                                 "CIM operational limit", "pu"));

    rules.push_back(nonnegative_rule("ACBranch", "ac.branches", "r_pu",
                                     C::Static, S::IEC61970CIM,
                                     "CIM ACLineSegment", "pu", true,
                                     Sev::Error));
    rules.push_back(nonnegative_rule("ACBranch", "ac.branches", "x_pu",
                                     C::Static, S::IEC61970CIM,
                                     "CIM ACLineSegment", "pu", true,
                                     Sev::Error));
    rules.push_back(nonnegative_rule("ACBranch", "ac.branches", "rate_a_mva",
                                     C::Static, S::IEC61970CIM,
                                     "thermal rating", "MVA"));
    rules.push_back(nonnegative_rule("ACBranch", "ac.branches", "length_km",
                                     C::Static, S::IEC61970CIM,
                                     "line length", "km"));
    add_common_reliability_rules(rules, "ACBranch", "ac.branches");

    rules.push_back(positive_required("Transformer2W",
                                      "ac.transformers_2w",
                                      "sn_mva",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM PowerTransformer",
                                      "MVA"));
    rules.push_back(positive_required("Transformer2W",
                                      "ac.transformers_2w",
                                      "vn_hv_kv",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM TransformerEnd",
                                      "kV"));
    rules.push_back(positive_required("Transformer2W",
                                      "ac.transformers_2w",
                                      "vn_lv_kv",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM TransformerEnd",
                                      "kV"));
    rules.push_back(bounded_rule("Transformer2W", "ac.transformers_2w",
                                 "vk_percent", C::Static, 0.01, 50.0,
                                 S::IEC61970CIM,
                                 "short-circuit impedance", "%"));
    rules.push_back(bounded_rule("Transformer2W", "ac.transformers_2w",
                                 "vkr_percent", C::Static, 0.0, 50.0,
                                 S::IEC61970CIM,
                                 "short-circuit resistance", "%"));
    add_common_reliability_rules(rules, "Transformer2W",
                                 "ac.transformers_2w");

    rules.push_back(positive_required("Transformer3W",
                                      "ac.transformers_3w",
                                      "sn_hv_mva",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM PowerTransformer",
                                      "MVA"));
    rules.push_back(positive_required("Transformer3W",
                                      "ac.transformers_3w",
                                      "sn_mv_mva",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM PowerTransformer",
                                      "MVA"));
    rules.push_back(positive_required("Transformer3W",
                                      "ac.transformers_3w",
                                      "sn_lv_mva",
                                      C::Static,
                                      S::IEC61970CIM,
                                      "CIM PowerTransformer",
                                      "MVA"));
    add_common_reliability_rules(rules, "Transformer3W",
                                 "ac.transformers_3w");

    rules.push_back(bounded_rule("ExternalGrid", "ac.external_grids", "vm_pu",
                                 C::Static, 0.8, 1.2, S::IEC60909,
                                 "voltage source equivalent", "pu"));
    rules.push_back(nonnegative_rule("ExternalGrid", "ac.external_grids",
                                     "s_sc_max_mva", C::Static, S::IEC60909,
                                     "short-circuit source", "MVA"));
    add_common_dynamic_rules(rules, "ExternalGrid", "ac.external_grids",
                             S::HACDCPF, "grid equivalent dynamic profile");

    rules.push_back(positive_required("AsynchronousMotor", "ac.motors",
                                      "sn_mva", C::Static, S::IEC60909,
                                      "asynchronous motor equivalent", "MVA"));
    rules.push_back(bounded_rule("AsynchronousMotor", "ac.motors",
                                 "efficiency", C::Static, 0.0, 1.0,
                                 S::IEC60909, "motor nameplate", "pu", false,
                                 Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "AsynchronousMotor", "ac.motors",
                             S::IEC60909, "motor transient profile");

    rules.push_back(bounded_rule("Generator", "ac.generators", "vg_pu",
                                 C::Static, 0.8, 1.2, S::IEEE,
                                 "synchronous generator terminal voltage",
                                 "pu"));
    rules.push_back(nonnegative_rule("Generator", "ac.generators", "mbase_mva",
                                     C::Static, S::IEEE,
                                     "machine base", "MVA"));
    rules.push_back(bounded_rule("Generator", "ac.generators", "inertia_h",
                                 C::Dynamic, 0.1, 15.0, S::IEEE,
                                 "synchronous-machine inertia", "s"));
    rules.push_back(bounded_rule("Generator", "ac.generators", "droop_r",
                                 C::Dynamic, 0.001, 0.2, S::IEEE,
                                 "governor droop", "pu"));
    rules.push_back(bounded_rule("Generator", "ac.generators", "xd_pu",
                                 C::Transient, 0.0, 5.0, S::IEEE,
                                 "d-axis reactance", "pu", false,
                                 Sev::Warning, false, true));
    rules.push_back(bounded_rule("Generator", "ac.generators", "xq_pu",
                                 C::Transient, 0.0, 5.0, S::IEEE,
                                 "q-axis reactance", "pu", false,
                                 Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "Generator", "ac.generators", S::IEEE,
                             "GENROU/GENSAL dynamic profile", Sev::Warning);
    add_common_reliability_rules(rules, "Generator", "ac.generators");

    rules.push_back(nonnegative_rule("StaticGenerator",
                                     "ac.static_generators",
                                     "p_rated_mw",
                                     C::Static,
                                     S::IEEE1547,
                                     "DER nameplate",
                                     "MW"));
    rules.push_back(bounded_rule("StaticGenerator", "ac.static_generators",
                                 "scaling", C::Static, 0.0, 2.0,
                                 S::HACDCPF, "static snapshot scaling", "pu"));
    add_common_dynamic_rules(rules, "StaticGenerator",
                             "ac.static_generators", S::NERC,
                             "REGC/REEC/REPC profile");
    add_common_reliability_rules(rules, "StaticGenerator",
                                 "ac.static_generators");

    rules.push_back(bounded_rule("RenewableGen", "ac.renewable_gens",
                                 "capacity_factor", C::Static, 0.0, 1.0,
                                 S::NERC, "renewable plant operating point",
                                 "pu"));
    rules.push_back(nonnegative_rule("RenewableGen", "ac.renewable_gens",
                                     "p_rated_mw", C::Static, S::NERC,
                                     "renewable plant nameplate", "MW"));
    add_common_dynamic_rules(rules, "RenewableGen", "ac.renewable_gens",
                             S::NERC, "REGC/REEC/REPC profile");
    add_common_reliability_rules(rules, "RenewableGen",
                                 "ac.renewable_gens");

    rules.push_back(bounded_rule("PVSystem", "ac.pv_systems",
                                 "inverter_eff", C::Static, 0.0, 1.0,
                                 S::IEEE1547, "PV inverter nameplate", "pu",
                                 false, Sev::Warning, false, true));
    rules.push_back(bounded_rule("PVSystem", "ac.pv_systems", "irradiance",
                                 C::Static, 0.0, 1500.0, S::IEEE1547,
                                 "PV environmental input", "W/m2"));
    rules.push_back(bounded_rule("PVSystem", "ac.pv_systems", "temperature",
                                 C::Static, -50.0, 100.0, S::IEEE1547,
                                 "PV environmental input", "degC"));
    add_common_dynamic_rules(rules, "PVSystem", "ac.pv_systems",
                             S::IEEE1547, "PV DER dynamic profile",
                             Sev::Warning);
    add_common_reliability_rules(rules, "PVSystem", "ac.pv_systems");

    rules.push_back(bounded_rule("Load", "ac.loads", "scaling", C::Static,
                                 0.0, 2.0, S::GridLABD,
                                 "constant-power/ZIP load", "pu"));
    rules.push_back(bounded_rule("Load", "ac.loads", "motor_percent",
                                 C::Static, 0.0, 100.0, S::IEC60909,
                                 "motor contribution fraction", "%"));
    add_common_dynamic_rules(rules, "Load", "ac.loads", S::HACDCPF,
                             "ZIP/dynamic load profile");

    rules.push_back(nonnegative_rule("FlexibleLoad", "ac.flexible_loads",
                                     "flex_up_mw", C::Static, S::HACDCPF,
                                     "demand response", "MW"));
    rules.push_back(nonnegative_rule("FlexibleLoad", "ac.flexible_loads",
                                     "flex_down_mw", C::Static, S::HACDCPF,
                                     "demand response", "MW"));
    rules.push_back(bounded_rule("FlexibleLoad", "ac.flexible_loads",
                                 "availability_pct", C::Reliability, 0.0,
                                 100.0, S::HACDCPF, "demand response", "%"));

    rules.push_back(bounded_rule("AsymmetricLoad", "ac.asymmetric_loads",
                                 "scaling", C::Static, 0.0, 2.0,
                                 S::OpenDSS, "phase load model", "pu"));
    add_common_dynamic_rules(rules, "AsymmetricLoad", "ac.asymmetric_loads",
                             S::OpenDSS, "phase load dynamic profile");

    rules.push_back(nonnegative_rule("Storage", "ac.storage", "e_rated_mwh",
                                     C::Static, S::IEEE1547,
                                     "BESS nameplate", "MWh"));
    rules.push_back(bounded_rule("Storage", "ac.storage", "soc_init",
                                 C::Static, 0.0, 1.0, S::IEEE1547,
                                 "BESS SOC", "pu"));
    rules.push_back(bounded_rule("Storage", "ac.storage", "soc_min",
                                 C::Static, 0.0, 1.0, S::IEEE1547,
                                 "BESS SOC", "pu"));
    rules.push_back(bounded_rule("Storage", "ac.storage", "soc_max",
                                 C::Static, 0.0, 1.0, S::IEEE1547,
                                 "BESS SOC", "pu"));
    rules.push_back(bounded_rule("Storage", "ac.storage", "eta_charge",
                                 C::Static, 0.0, 1.0, S::IEEE1547,
                                 "BESS efficiency", "pu", false,
                                 Sev::Warning, false, true));
    rules.push_back(bounded_rule("Storage", "ac.storage", "eta_discharge",
                                 C::Static, 0.0, 1.0, S::IEEE1547,
                                 "BESS efficiency", "pu", false,
                                 Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "Storage", "ac.storage", S::IEEE1547,
                             "BESS DER dynamic profile", Sev::Warning);
    add_common_reliability_rules(rules, "Storage", "ac.storage");

    rules.push_back(bounded_rule("MobileStorage", "mobile_storage",
                                 "soc_init", C::Static, 0.0, 1.0,
                                 S::IEEE1547, "mobile BESS SOC", "pu"));
    rules.push_back(bounded_rule("MobileStorage", "mobile_storage",
                                 "eta_charge", C::Static, 0.0, 1.0,
                                 S::IEEE1547, "mobile BESS efficiency", "pu",
                                 false, Sev::Warning, false, true));
    rules.push_back(bounded_rule("MobileStorage", "mobile_storage",
                                 "eta_discharge", C::Static, 0.0, 1.0,
                                 S::IEEE1547, "mobile BESS efficiency", "pu",
                                 false, Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "MobileStorage", "mobile_storage",
                             S::IEEE1547, "mobile BESS dynamic profile");
    add_common_reliability_rules(rules, "MobileStorage", "mobile_storage");

    rules.push_back(bounded_rule("Charger", "ac.chargers", "eta", C::Static,
                                 0.0, 1.0, S::IEC61850,
                                 "EVSE conversion efficiency", "pu", false,
                                 Sev::Warning, false, true));
    add_common_reliability_rules(rules, "Charger", "ac.chargers");
    rules.push_back(bounded_rule("ChargingStation", "ac.charging_stations",
                                 "simultaneity_factor", C::Static, 0.0, 1.5,
                                 S::IEC61850, "EVSE site aggregation", "pu"));
    rules.push_back(bounded_rule("ChargingStation", "ac.charging_stations",
                                 "power_factor", C::Static, 0.0, 1.0,
                                 S::IEC61850, "EVSE site aggregation", "pu"));
    add_common_reliability_rules(rules, "ChargingStation",
                                 "ac.charging_stations");

    rules.push_back(nonnegative_rule("Switch", "ac.switches",
                                     "r_contact_ohm", C::Static, S::IEC61850,
                                     "switchgear", "ohm"));
    rules.push_back(nonnegative_rule("Switch", "ac.switches", "p_sw_fail",
                                     C::Failure, S::NERC,
                                     "switch reliability", "pu"));
    add_common_reliability_rules(rules, "Switch", "ac.switches");
    rules.push_back(nonnegative_rule("CircuitBreaker", "ac.circuit_breakers",
                                     "rated_voltage_kv", C::Static,
                                     S::IEC61850, "breaker nameplate", "kV"));
    rules.push_back(nonnegative_rule("CircuitBreaker", "ac.circuit_breakers",
                                     "i_breaking_ka", C::Static, S::IEC61850,
                                     "breaker nameplate", "kA"));

    rules.push_back(positive_required("DCBus", "dc.buses", "base_kv",
                                      C::Static, S::HACDCPF,
                                      "DC bus nameplate", "kV",
                                      Sev::Warning));
    rules.push_back(bounded_rule("DCBus", "dc.buses", "vm_pu", C::Static,
                                 0.5, 1.5, S::HACDCPF, "DC bus voltage",
                                 "pu"));
    rules.push_back(nonnegative_rule("DCBranch", "dc.branches", "r_pu",
                                     C::Static, S::HACDCPF, "DC branch",
                                     "pu", true, Sev::Error));
    rules.push_back(nonnegative_rule("DCBranch", "dc.branches", "rate_a_mva",
                                     C::Static, S::HACDCPF, "DC branch rating",
                                     "MVA"));
    rules.push_back(nonnegative_rule("DCBranch", "dc.branches", "length_km",
                                     C::Static, S::HACDCPF, "DC branch length",
                                     "km"));
    add_common_reliability_rules(rules, "DCBranch", "dc.branches");

    rules.push_back(bounded_rule("DCLoad", "dc.loads", "scaling", C::Static,
                                 0.0, 2.0, S::HACDCPF, "DC load", "pu"));
    add_common_dynamic_rules(rules, "DCLoad", "dc.loads", S::HACDCPF,
                             "DC load dynamic profile");

    rules.push_back(bounded_rule("Storage", "dc.storage", "soc_init",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "legacy DC storage SOC", "pu"));
    rules.push_back(bounded_rule("Storage", "dc.storage", "eta_charge",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "legacy DC storage efficiency", "pu", false,
                                 Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "Storage", "dc.storage", S::HACDCPF,
                             "legacy DC storage dynamic profile");

    rules.push_back(bounded_rule("DCStorage", "dc.dc_storage", "soc_init",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "DC BESS SOC", "pu"));
    rules.push_back(bounded_rule("DCStorage", "dc.dc_storage", "soc_min",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "DC BESS SOC", "pu"));
    rules.push_back(bounded_rule("DCStorage", "dc.dc_storage", "soc_max",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "DC BESS SOC", "pu"));
    rules.push_back(bounded_rule("DCStorage", "dc.dc_storage", "eta_charge",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "DC BESS efficiency", "pu", false,
                                 Sev::Warning, false, true));
    rules.push_back(bounded_rule("DCStorage", "dc.dc_storage",
                                 "eta_discharge", C::Static, 0.0, 1.0,
                                 S::HACDCPF, "DC BESS efficiency", "pu",
                                 false, Sev::Warning, false, true));
    add_common_dynamic_rules(rules, "DCStorage", "dc.dc_storage",
                             S::HACDCPF, "DC BESS dynamic profile");
    add_common_reliability_rules(rules, "DCStorage", "dc.dc_storage");

    rules.push_back(nonnegative_rule("StaticGeneratorDC",
                                     "dc.dc_static_generators", "pmax_mw",
                                     C::Static, S::HACDCPF,
                                     "DC static generator", "MW"));
    add_common_dynamic_rules(rules, "StaticGeneratorDC",
                             "dc.dc_static_generators", S::HACDCPF,
                             "DC source dynamic profile");
    add_common_reliability_rules(rules, "StaticGeneratorDC",
                                 "dc.dc_static_generators");

    rules.push_back(bounded_rule("PVArrayDC", "dc.pv_arrays", "irradiance",
                                 C::Static, 0.0, 1500.0, S::IEEE1547,
                                 "DC PV array environmental input", "W/m2"));
    rules.push_back(bounded_rule("PVArrayDC", "dc.pv_arrays", "temperature",
                                 C::Static, -50.0, 100.0, S::IEEE1547,
                                 "DC PV array environmental input", "degC"));
    add_common_dynamic_rules(rules, "PVArrayDC", "dc.pv_arrays",
                             S::IEEE1547, "DC PV array dynamic profile");
    add_common_reliability_rules(rules, "PVArrayDC", "dc.pv_arrays");

    rules.push_back(bounded_rule("VSCConverter", "vsc_converters", "eta",
                                 C::Static, 0.0, 1.0, S::NERC,
                                 "IBR converter efficiency", "pu", false,
                                 Sev::Warning, false, true));
    rules.push_back(bounded_rule("VSCConverter", "vsc_converters",
                                 "i_max_pu", C::Transient, 0.1, 3.0,
                                 S::IEC60909, "converter current limit", "pu"));
    rules.push_back(bounded_rule("VSCConverter", "vsc_converters",
                                 "v_ac_set_pu", C::Static, 0.5, 1.5,
                                 S::NERC, "converter AC voltage setpoint",
                                 "pu"));
    rules.push_back(bounded_rule("VSCConverter", "vsc_converters",
                                 "v_dc_set_pu", C::Static, 0.5, 1.5,
                                 S::NERC, "converter DC voltage setpoint",
                                 "pu"));
    add_common_dynamic_rules(rules, "VSCConverter", "vsc_converters",
                             S::NERC, "REGC/REEC/REPC or GFM profile",
                             Sev::Warning);
    add_common_reliability_rules(rules, "VSCConverter", "vsc_converters");

    rules.push_back(bounded_rule("DCDCConverter", "dc.dcdc_converters", "eta",
                                 C::Static, 0.0, 1.0, S::HACDCPF,
                                 "DC/DC converter efficiency", "pu", false,
                                 Sev::Warning, false, true));
    rules.push_back(bounded_rule("DCDCConverter", "dc.dcdc_converters",
                                 "d_min", C::Transient, 0.0, 1.0,
                                 S::HACDCPF, "duty-ratio feasibility", "pu"));
    rules.push_back(bounded_rule("DCDCConverter", "dc.dcdc_converters",
                                 "d_max", C::Transient, 0.0, 1.0,
                                 S::HACDCPF, "duty-ratio feasibility", "pu"));
    add_common_dynamic_rules(rules, "DCDCConverter",
                             "dc.dcdc_converters", S::HACDCPF,
                             "DCDCConverterDynamic profile");
    add_common_reliability_rules(rules, "DCDCConverter",
                                 "dc.dcdc_converters");

    rules.push_back(bounded_rule("EnergyRouter", "energy_routers",
                                 "loss_percent", C::Static, 0.0, 100.0,
                                 S::HACDCPF, "energy router losses", "%"));
    add_common_dynamic_rules(rules, "EnergyRouter", "energy_routers",
                             S::HACDCPF,
                             "EnergyRouterAverageModel profile");
    add_common_reliability_rules(rules, "EnergyRouter", "energy_routers");

    rules.push_back(nonnegative_rule("VirtualPowerPlant", "vpps", "pmax_mw",
                                     C::Static, S::NERC,
                                     "DER aggregation", "MW"));
    add_common_dynamic_rules(rules, "VirtualPowerPlant", "vpps", S::NERC,
                             "DER_Aggregate profile");
    add_common_reliability_rules(rules, "VirtualPowerPlant", "vpps");

    rules.push_back(bounded_rule("Microgrid", "microgrids", "v_set_pu",
                                 C::Static, 0.5, 1.5, S::IEEE1547,
                                 "microgrid voltage setpoint", "pu"));
    rules.push_back(bounded_rule("Microgrid", "microgrids", "f_set_hz",
                                 C::Static, 45.0, 65.0, S::IEEE1547,
                                 "microgrid frequency setpoint", "Hz"));
    add_common_dynamic_rules(rules, "Microgrid", "microgrids", S::IEEE1547,
                             "microgrid controller profile");
    add_common_reliability_rules(rules, "Microgrid", "microgrids");

    rules.push_back(positive_required("ThreePhaseACBus",
                                      "three_phase_ac.buses",
                                      "base_kv",
                                      C::Static,
                                      S::OpenDSS,
                                      "phase-domain bus",
                                      "kV",
                                      Sev::Warning));
    rules.push_back(bounded_rule("ThreePhaseACBus", "three_phase_ac.buses",
                                 "vm_a_pu", C::Static, 0.5, 1.5, S::OpenDSS,
                                 "phase voltage", "pu"));
    rules.push_back(bounded_rule("ThreePhaseACBus", "three_phase_ac.buses",
                                 "vm_b_pu", C::Static, 0.5, 1.5, S::OpenDSS,
                                 "phase voltage", "pu"));
    rules.push_back(bounded_rule("ThreePhaseACBus", "three_phase_ac.buses",
                                 "vm_c_pu", C::Static, 0.5, 1.5, S::OpenDSS,
                                 "phase voltage", "pu"));
    rules.push_back(nonnegative_rule("ThreePhaseACLine",
                                     "three_phase_ac.lines", "length_km",
                                     C::Static, S::OpenDSS,
                                     "phase line geometry", "km"));
    rules.push_back(nonnegative_rule("ThreePhaseACLine",
                                     "three_phase_ac.lines", "rate_a_mva",
                                     C::Static, S::OpenDSS,
                                     "phase line rating", "MVA"));
    add_common_reliability_rules(rules, "ThreePhaseACLine",
                                 "three_phase_ac.lines");
    rules.push_back(positive_required("ThreePhaseTransformer",
                                      "three_phase_ac.transformers", "sn_mva",
                                      C::Static, S::OpenDSS,
                                      "phase transformer", "MVA",
                                      Sev::Warning));
    add_common_reliability_rules(rules, "ThreePhaseTransformer",
                                 "three_phase_ac.transformers");
    rules.push_back(bounded_rule("ThreePhaseLoad", "three_phase_ac.loads",
                                 "vmin_pu", C::Static, 0.0, 1.5,
                                 S::OpenDSS, "phase load voltage model", "pu"));
    rules.push_back(bounded_rule("ThreePhaseLoad", "three_phase_ac.loads",
                                 "vmax_pu", C::Static, 0.0, 1.5,
                                 S::OpenDSS, "phase load voltage model", "pu"));
    add_common_dynamic_rules(rules, "ThreePhaseLoad", "three_phase_ac.loads",
                             S::OpenDSS, "phase ZIP/dynamic load profile");
    rules.push_back(bounded_rule("ThreePhaseGenerator",
                                 "three_phase_ac.generators", "vm_pu",
                                 C::Static, 0.8, 1.2, S::OpenDSS,
                                 "phase generator voltage control", "pu"));
    add_common_dynamic_rules(rules, "ThreePhaseGenerator",
                             "three_phase_ac.generators", S::IEEE,
                             "phase generator dynamic profile");
    rules.push_back(bounded_rule("ThreePhaseExternalGrid",
                                 "three_phase_ac.external_grids", "vm_pu",
                                 C::Static, 0.8, 1.2, S::OpenDSS,
                                 "phase source voltage", "pu"));
    add_common_dynamic_rules(rules, "ThreePhaseExternalGrid",
                             "three_phase_ac.external_grids", S::OpenDSS,
                             "phase source dynamic profile");

    return rules;
  }();
  return rules;
}

DigitalTwinReadinessCriterion twin_criterion(std::string criterion_id,
                                             DT dimension,
                                             std::string title,
                                             std::string description,
                                             double weight,
                                             Sev severity,
                                             S standard_family,
                                             std::string standard_profile) {
  DigitalTwinReadinessCriterion item;
  item.criterion_id = std::move(criterion_id);
  item.dimension = dimension;
  item.title = std::move(title);
  item.description = std::move(description);
  item.weight = weight;
  item.severity_if_failed = severity;
  item.standard_family = standard_family;
  item.standard_profile = std::move(standard_profile);
  return item;
}

const std::vector<DigitalTwinReadinessCriterion>& twin_criteria_registry() {
  static const std::vector<DigitalTwinReadinessCriterion> criteria = {
      twin_criterion("DT-IDENTITY-01",
                     DT::AssetIdentity,
                     "Asset identity completeness",
                     "Each physical or logical asset should carry stable "
                     "indices and human-readable names so telemetry, events, "
                     "maintenance records, and external twins can resolve the "
                     "same object.",
                     1.0,
                     Sev::Warning,
                     S::IEC61970CIM,
                     "CIM mRID/name asset identity"),
      twin_criterion("DT-TOPOLOGY-01",
                     DT::TopologyConnectivity,
                     "Topology connectivity executability",
                     "A twin must contain enough AC/DC/three-phase topology to "
                     "form connected islands, boundary injections, and solved "
                     "network equations.",
                     1.2,
                     Sev::Error,
                     S::IEC61970CIM,
                     "CIM ConnectivityNode/Terminal topology"),
      twin_criterion("DT-PARAM-01",
                     DT::ElectricalParameters,
                     "Electrical parameter health",
                     "Nameplate, impedance, limit, SOC, and controller "
                     "parameters should pass the standard-aware range gates.",
                     1.4,
                     Sev::Error,
                     S::HACDCPF,
                     "parameter governance catalog"),
      twin_criterion("DT-DYNAMIC-01",
                     DT::DynamicBehavior,
                     "Dynamic model availability",
                     "Transient-capable devices should carry named dynamic "
                     "model profiles and parameter sets before the model is "
                     "treated as a living twin.",
                     1.1,
                     Sev::Warning,
                     S::NERC,
                     "GENROU/REGC/REEC/IEEE1547 dynamic profile readiness"),
      twin_criterion("DT-TELEM-01",
                     DT::TelemetryObservability,
                     "Telemetry anchor readiness",
                     "A practical twin needs enough observable buses/devices "
                     "with stable names or external identifiers to bind SCADA, "
                     "PMU, AMI, DERMS, or historian streams.",
                     1.0,
                     Sev::Warning,
                     S::IEC61850,
                     "IEC 61850 logical-node observability"),
      twin_criterion("DT-STATE-01",
                     DT::StateSynchronization,
                     "State synchronization seeds",
                     "The initial state should include voltage, SOC, dispatch, "
                     "and status values that can be reconciled against live or "
                     "historical measurements.",
                     1.0,
                     Sev::Warning,
                     S::HACDCPF,
                     "state-estimation initialization contract"),
      twin_criterion("DT-EVENT-01",
                     DT::ScenarioEvents,
                     "Scenario and event readiness",
                     "A twin should expose controllable event surfaces for "
                     "load changes, trips, faults, DER references, and "
                     "restoration actions.",
                     0.8,
                     Sev::Info,
                     S::HACDCPF,
                     "contingency/event catalog"),
      twin_criterion("DT-RELIABILITY-01",
                     DT::ReliabilityLifecycle,
                     "Reliability and lifecycle metadata",
                     "Failure rates, forced-outage rates, MTTR/MTBF, asset "
                     "age, and lifecycle fields make the twin useful for "
                     "risk, maintenance, and resilience workflows.",
                     1.0,
                     Sev::Warning,
                     S::NERC,
                     "availability/reliability data contract"),
      twin_criterion("DT-IO-01",
                     DT::StandardsInteroperability,
                     "Standards and external IO interoperability",
                     "The twin should preserve rich JSON semantics while "
                     "projecting to canonical, GridLAB-D, OpenDSS, and future "
                     "CIM/IEC exchange contracts with declared loss of detail.",
                     1.1,
                     Sev::Warning,
                     S::IEC61970CIM,
                     "CIM/OpenDSS/GridLAB-D interoperability"),
      twin_criterion("DT-VALID-01",
                     DT::NumericalValidation,
                     "Numerical validation readiness",
                     "Network and dynamic results should be cross-checkable "
                     "against native solvers and external references within "
                     "declared scopes and tolerances.",
                     1.2,
                     Sev::Warning,
                     S::OpenDSS,
                     "OpenDSS/GridLAB-D numerical cross-check"),
      twin_criterion("DT-GOV-01",
                     DT::ProvenanceGovernance,
                     "Provenance and governance traceability",
                     "Model data should retain source identifiers, standards, "
                     "parameter-set names, and diagnostics so changes are "
                     "auditable over the twin lifecycle.",
                     1.0,
                     Sev::Warning,
                     S::HACDCPF,
                     "model governance and audit trail"),
  };
  return criteria;
}

const DigitalTwinReadinessCriterion* find_twin_criterion(
    std::string_view criterion_id) {
  const auto& criteria = twin_criteria_registry();
  const auto it = std::find_if(
      criteria.begin(), criteria.end(),
      [&](const DigitalTwinReadinessCriterion& item) {
        return item.criterion_id == criterion_id;
      });
  if (it == criteria.end()) return nullptr;
  return &(*it);
}

Sev readiness_severity(const DigitalTwinReadinessCriterion& criterion,
                       double score_ratio) {
  if (score_ratio >= 0.90) return Sev::Info;
  if (score_ratio >= 0.65) return Sev::Warning;
  return criterion.severity_if_failed;
}

int severity_rank(Sev severity) {
  switch (severity) {
    case Sev::Error:
      return 0;
    case Sev::Warning:
      return 1;
    case Sev::Info:
      return 2;
  }
  return 3;
}

std::string parameter_rule_key(std::string_view component_type,
                               std::string_view collection_path,
                               std::string_view parameter_path) {
  std::string key;
  key.reserve(component_type.size() + collection_path.size() +
              parameter_path.size() + 2U);
  key.append(component_type);
  key.push_back('|');
  key.append(collection_path);
  key.push_back('|');
  key.append(parameter_path);
  return key;
}

const ComponentParameterRule* find_parameter_rule(
    std::string_view component_type,
    std::string_view collection_path,
    std::string_view parameter_path) {
  static const std::map<std::string, const ComponentParameterRule*> lookup = [] {
    std::map<std::string, const ComponentParameterRule*> map;
    for (const auto& rule : parameter_registry()) {
      map.emplace(parameter_rule_key(rule.component_type,
                                     rule.collection_path,
                                     rule.parameter_path),
                  &rule);
    }
    return map;
  }();
  const auto it =
      lookup.find(parameter_rule_key(component_type, collection_path,
                                     parameter_path));
  if (it == lookup.end()) return nullptr;
  return it->second;
}

std::string format_double(double value) {
  std::ostringstream os;
  os << std::setprecision(6) << value;
  return os.str();
}

std::string range_text(const ComponentParameterRule& rule) {
  std::ostringstream os;
  if (rule.min_value && rule.max_value) {
    os << (rule.min_inclusive ? "[" : "(") << format_double(*rule.min_value)
       << ", " << format_double(*rule.max_value)
       << (rule.max_inclusive ? "]" : ")");
  } else if (rule.min_value) {
    os << (rule.min_inclusive ? ">= " : "> ")
       << format_double(*rule.min_value);
  } else if (rule.max_value) {
    os << (rule.max_inclusive ? "<= " : "< ")
       << format_double(*rule.max_value);
  } else {
    os << "provided";
  }
  if (!rule.units.empty()) os << " " << rule.units;
  return os.str();
}

struct ComponentIdentity {
  std::string component_type;
  std::string collection_path;
  std::size_t component_position{0};
  int component_index{0};
  std::string component_name;
};

void add_finding(ComponentParameterAuditReport& report,
                 const ComponentIdentity& id,
                 const ComponentParameterRule& rule,
                 Sev severity,
                 std::optional<double> value,
                 std::string message) {
  ComponentParameterFinding finding;
  finding.component_type = id.component_type;
  finding.collection_path = id.collection_path;
  finding.component_position = id.component_position;
  finding.component_index = id.component_index;
  finding.component_name = id.component_name;
  finding.parameter_path = rule.parameter_path;
  finding.category = rule.category;
  finding.severity = severity;
  finding.value = value;
  finding.expected_min = rule.min_value;
  finding.expected_max = rule.max_value;
  finding.min_inclusive = rule.min_inclusive;
  finding.max_inclusive = rule.max_inclusive;
  finding.required = rule.required;
  finding.standard_family = rule.standard_family;
  finding.standard_profile = rule.standard_profile;
  finding.units = rule.units;
  finding.message = std::move(message);
  report.findings.push_back(std::move(finding));
}

bool below_min(double value, const ComponentParameterRule& rule) {
  if (!rule.min_value) return false;
  return rule.min_inclusive ? value < *rule.min_value : value <= *rule.min_value;
}

bool above_max(double value, const ComponentParameterRule& rule) {
  if (!rule.max_value) return false;
  return rule.max_inclusive ? value > *rule.max_value : value >= *rule.max_value;
}

void check_rule_value(ComponentParameterAuditReport& report,
                      const ComponentIdentity& id,
                      const ComponentParameterRule& rule,
                      double value) {
  constexpr double kUnsetEpsilon = 1e-12;
  if (!rule.required && rule.min_value && *rule.min_value > 0.0 &&
      std::abs(value) <= kUnsetEpsilon) {
    return;
  }
  ++report.checked_parameters;
  if (!std::isfinite(value)) {
    add_finding(report,
                id,
                rule,
                Sev::Error,
                value,
                rule.parameter_path + " is not finite; expected " +
                    range_text(rule) + ".");
    return;
  }
  if (below_min(value, rule) || above_max(value, rule)) {
    add_finding(report,
                id,
                rule,
                rule.range_severity,
                value,
                rule.parameter_path + " = " + format_double(value) +
                    " outside expected " + range_text(rule) + ".");
  }
}

void check_parameter(ComponentParameterAuditReport& report,
                     const ComponentIdentity& id,
                     std::string_view parameter_path,
                     double value,
                     C category,
                     std::optional<double> min_value,
                     std::optional<double> max_value,
                     S family,
                     std::string standard_profile,
                     std::string units = {},
                     bool required = false,
                     Sev severity = Sev::Warning,
                     bool min_inclusive = true,
                     bool max_inclusive = true) {
  const ComponentParameterRule* existing =
      find_parameter_rule(id.component_type, id.collection_path, parameter_path);
  ComponentParameterRule fallback;
  if (!existing) {
    fallback = parameter_rule(id.component_type,
                              id.collection_path,
                              std::string(parameter_path),
                              category,
                              min_value,
                              max_value,
                              family,
                              std::move(standard_profile),
                              std::move(units),
                              required,
                              severity,
                              Sev::Info,
                              {},
                              min_inclusive,
                              max_inclusive);
    existing = &fallback;
  }
  check_rule_value(report, id, *existing, value);
}

void check_registered(ComponentParameterAuditReport& report,
                      const ComponentIdentity& id,
                      std::string_view parameter_path,
                      double value) {
  const auto* rule =
      find_parameter_rule(id.component_type, id.collection_path, parameter_path);
  if (rule) check_rule_value(report, id, *rule, value);
}

void check_presence(ComponentParameterAuditReport& report,
                    const ComponentIdentity& id,
                    std::string_view parameter_path,
                    bool present) {
  const auto* rule =
      find_parameter_rule(id.component_type, id.collection_path, parameter_path);
  if (!rule) return;
  if (present) {
    ++report.checked_parameters;
    return;
  }
  add_finding(report,
              id,
              *rule,
              rule->missing_severity,
              std::nullopt,
              "Missing " + rule->parameter_path + " for " +
                  rule->standard_profile + ".");
}

void check_order(ComponentParameterAuditReport& report,
                 const ComponentIdentity& id,
                 std::string parameter_path,
                 double lower,
                 double upper,
                 C category,
                 S family,
                 std::string profile,
                 std::string units = {},
                 Sev severity = Sev::Error) {
  ++report.checked_parameters;
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
    auto rule = parameter_rule(id.component_type,
                               id.collection_path,
                               std::move(parameter_path),
                               category,
                               std::nullopt,
                               std::nullopt,
                               family,
                               std::move(profile),
                               std::move(units),
                               true,
                               severity,
                               severity);
    add_finding(report,
                id,
                rule,
                severity,
                lower,
                rule.parameter_path + " is inconsistent: lower = " +
                    format_double(lower) + ", upper = " +
                    format_double(upper) + ".");
  }
}

void check_percent_sum(ComponentParameterAuditReport& report,
                       const ComponentIdentity& id,
                       std::string parameter_path,
                       double a,
                       double b,
                       double c,
                       S family,
                       std::string profile,
                       double tolerance = 1.0) {
  const double sum = a + b + c;
  const bool all_zero = std::abs(a) < 1e-12 && std::abs(b) < 1e-12 &&
                        std::abs(c) < 1e-12;
  if (all_zero) return;
  ++report.checked_parameters;
  if (!std::isfinite(sum) || std::abs(sum - 100.0) > tolerance) {
    auto rule = parameter_rule(id.component_type,
                               id.collection_path,
                               std::move(parameter_path),
                               C::Static,
                               100.0 - tolerance,
                               100.0 + tolerance,
                               family,
                               std::move(profile),
                               "%",
                               false,
                               Sev::Warning);
    add_finding(report,
                id,
                rule,
                Sev::Warning,
                sum,
                rule.parameter_path + " sums to " + format_double(sum) +
                    "%; expected approximately 100%.");
  }
}

bool is_dynamic_parameter_key(std::string_view key,
                              std::string_view expected) {
  if (key == expected) return true;
  if (key.size() != expected.size()) return false;
  for (std::size_t i = 0; i < key.size(); ++i) {
    const auto a = static_cast<char>(std::tolower(
        static_cast<unsigned char>(key[i])));
    const auto b = static_cast<char>(std::tolower(
        static_cast<unsigned char>(expected[i])));
    if (a != b) return false;
  }
  return true;
}

void check_dynamic_parameter_key(ComponentParameterAuditReport& report,
                                 const ComponentIdentity& id,
                                 const std::string& key,
                                 double value,
                                 bool nested_component) {
  const std::string prefix =
      nested_component ? "dynamic_model.components.parameters."
                       : "dynamic_model.parameters.";
  const auto path = prefix + key;
  if (is_dynamic_parameter_key(key, "H") ||
      is_dynamic_parameter_key(key, "inertia_h")) {
    check_parameter(report, id, path, value, C::Dynamic, 0.1, 15.0, S::IEEE,
                    "machine/IBR inertia profile", "s");
  } else if (is_dynamic_parameter_key(key, "D")) {
    check_parameter(report, id, path, value, C::Dynamic, 0.0, 20.0, S::IEEE,
                    "damping/droop profile", "pu");
  } else if (is_dynamic_parameter_key(key, "Xd") ||
             is_dynamic_parameter_key(key, "Xq") ||
             is_dynamic_parameter_key(key, "Xdp") ||
             is_dynamic_parameter_key(key, "Xqp") ||
             is_dynamic_parameter_key(key, "Xdpp") ||
             is_dynamic_parameter_key(key, "Xqpp") ||
             is_dynamic_parameter_key(key, "xd_pu") ||
             is_dynamic_parameter_key(key, "xq_pu") ||
             is_dynamic_parameter_key(key, "xdp_pu") ||
             is_dynamic_parameter_key(key, "xdpp_pu")) {
    check_parameter(report, id, path, value, C::Transient, 0.0, 5.0, S::IEEE,
                    "synchronous-machine reactance profile", "pu", false,
                    Sev::Warning, false, true);
  } else if (is_dynamic_parameter_key(key, "Td0p") ||
             is_dynamic_parameter_key(key, "Tq0p") ||
             is_dynamic_parameter_key(key, "Td0pp") ||
             is_dynamic_parameter_key(key, "Tq0pp") ||
             is_dynamic_parameter_key(key, "tau_s") ||
             is_dynamic_parameter_key(key, "Tg") ||
             is_dynamic_parameter_key(key, "Ta")) {
    check_parameter(report, id, path, value, C::Transient, 0.0, 100.0,
                    S::IEEE, "dynamic time-constant profile", "s", false,
                    Sev::Warning, false, true);
  } else if (is_dynamic_parameter_key(key, "R") ||
             is_dynamic_parameter_key(key, "droop") ||
             is_dynamic_parameter_key(key, "droop_r")) {
    check_parameter(report, id, path, value, C::Dynamic, 0.001, 0.2, S::IEEE,
                    "governor/grid-forming droop profile", "pu");
  } else if (is_dynamic_parameter_key(key, "pll_kp") ||
             is_dynamic_parameter_key(key, "kp_pll")) {
    check_parameter(report, id, path, value, C::Transient, 0.0, 100.0, S::NERC,
                    "PLL controller profile", "pu");
  } else if (is_dynamic_parameter_key(key, "pll_ki") ||
             is_dynamic_parameter_key(key, "ki_pll")) {
    check_parameter(report, id, path, value, C::Transient, 0.0, 500.0, S::NERC,
                    "PLL controller profile", "pu/s");
  } else if (is_dynamic_parameter_key(key, "kp") ||
             is_dynamic_parameter_key(key, "ki") ||
             is_dynamic_parameter_key(key, "k_p") ||
             is_dynamic_parameter_key(key, "k_q")) {
    check_parameter(report, id, path, value, C::Dynamic, 0.0, 100.0, S::NERC,
                    "converter/controller gain profile", "pu");
  }
}

void audit_dynamic_profile(ComponentParameterAuditReport& report,
                           const ComponentIdentity& id,
                           const DynamicModelProfile& profile) {
  const bool present = !profile.empty();
  check_presence(report, id, "dynamic_model", present);
  if (!present) return;
  check_presence(report, id, "dynamic_model.standard",
                 !profile.standard.empty());
  check_presence(report, id, "dynamic_model.model_name",
                 !profile.model_name.empty());
  for (const auto& [key, value] : profile.parameters) {
    check_dynamic_parameter_key(report, id, key, value, false);
  }
  for (const auto& component : profile.components) {
    for (const auto& [key, value] : component.parameters) {
      check_dynamic_parameter_key(report, id, key, value, true);
    }
    if (!component.standard.empty() || !component.model.empty()) {
      continue;
    }
    auto rule = presence_rule(id.component_type,
                              id.collection_path,
                              "dynamic_model.components.model",
                              C::Dynamic,
                              S::HACDCPF,
                              "nested controller block",
                              Sev::Info);
    add_finding(report,
                id,
                rule,
                Sev::Info,
                std::nullopt,
                "Nested dynamic component is missing standard/model labels.");
  }
}

template <typename T, typename Fn>
void audit_collection(ComponentParameterAuditReport& report,
                      const std::vector<T>& rows,
                      std::string component_type,
                      std::string collection_path,
                      Fn&& fn) {
  for (std::size_t i = 0; i < rows.size(); ++i) {
    ComponentIdentity id;
    id.component_type = component_type;
    id.collection_path = collection_path;
    id.component_position = i;
    id.component_index = rows[i].index;
    id.component_name = rows[i].name;
    ++report.component_instances_checked;
    fn(id, rows[i]);
  }
}

struct TwinCounts {
  std::size_t total_components{0};
  std::size_t named_components{0};
  std::size_t dynamic_capable_components{0};
  std::size_t dynamic_profile_components{0};
  std::size_t reliability_capable_components{0};
  std::size_t reliability_metadata_components{0};
  std::size_t telemetry_anchor_components{0};
  std::size_t state_seed_components{0};
  std::size_t controllable_event_components{0};
  std::size_t provenance_components{0};
};

bool has_text(const std::string& value) {
  return !value.empty();
}

bool has_dynamic_profile(const DynamicModelProfile& profile) {
  return !profile.empty() &&
         (!profile.standard.empty() || !profile.model_name.empty() ||
          !profile.parameter_set.empty() || !profile.components.empty() ||
          !profile.parameters.empty());
}

bool has_dynamic_provenance(const DynamicModelProfile& profile) {
  return !profile.standard.empty() || !profile.parameter_set.empty() ||
         !profile.source_id.empty() || !profile.notes.empty();
}

bool reliability_present(double failure_rate,
                         double forced_outage_rate,
                         double mttr_hr,
                         double mttr_hours,
                         double mtbf_hr,
                         double mtbf_hours) {
  return failure_rate > 0.0 || forced_outage_rate > 0.0 || mttr_hr > 0.0 ||
         mttr_hours > 0.0 || mtbf_hr > 0.0 || mtbf_hours > 0.0;
}

bool reliability_present(const Storage& st) {
  return reliability_present(0.0,
                             st.forced_outage_rate,
                             st.mttr_hr,
                             0.0,
                             0.0,
                             0.0) ||
         st.mtbf_battery_hr > 0.0 || st.mttr_battery_hr > 0.0 ||
         st.mtbf_pcs_hr > 0.0 || st.mttr_pcs_hr > 0.0 ||
         st.mtbf_bms_hr > 0.0 || st.mttr_bms_hr > 0.0;
}

bool reliability_present(const MobileStorage& st) {
  return reliability_present(0.0,
                             0.0,
                             0.0,
                             st.mttr_hours,
                             0.0,
                             st.mtbf_hours) ||
         st.mtbf_battery_hr > 0.0 || st.mttr_battery_hr > 0.0 ||
         st.mtbf_pcs_hr > 0.0 || st.mttr_pcs_hr > 0.0 ||
         st.mtbf_bms_hr > 0.0 || st.mttr_bms_hr > 0.0 ||
         st.mtbf_vehicle_hr > 0.0 || st.mttr_vehicle_hr > 0.0;
}

bool reliability_present(const DCStorage& st) {
  return reliability_present(
      0.0, st.forced_outage_rate, st.mttr_hr, 0.0, 0.0, 0.0);
}

double ratio(std::size_t numerator, std::size_t denominator) {
  if (denominator == 0U) return 1.0;
  return std::clamp(static_cast<double>(numerator) /
                        static_cast<double>(denominator),
                    0.0,
                    1.0);
}

template <typename Rows, typename Fn>
void collect_twin_counts(const Rows& rows, TwinCounts& counts, Fn&& fn) {
  for (const auto& row : rows) {
    ++counts.total_components;
    fn(row);
  }
}

void collect_twin_counts(const HybridPowerSystem& sys, TwinCounts& counts) {
  collect_twin_counts(sys.ac.buses, counts, [&](const ACBus& b) {
    if (has_text(b.name)) ++counts.named_components;
    if (b.base_kv > 0.0 && b.vm_pu > 0.0) ++counts.state_seed_components;
    if (has_text(b.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.branches, counts, [&](const ACBranch& br) {
    if (has_text(br.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(br.failure_rate, 0.0, br.mttr_hr, 0.0, 0.0, 0.0)) {
      ++counts.reliability_metadata_components;
    }
    if (has_text(br.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.generators, counts, [&](const Generator& gen) {
    if (has_text(gen.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(gen.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, gen.forced_outage_rate, gen.mttr_hr, 0.0,
                            0.0, 0.0)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    if (gen.pg_mw != 0.0 || gen.qg_mvar != 0.0 || gen.vg_pu != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(gen.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(gen.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.static_generators, counts, [&](const StaticGenerator& sg) {
    if (has_text(sg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(sg.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, sg.mttr_hours, 0.0,
                            sg.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (sg.controllable) ++counts.controllable_event_components;
    if (sg.p_mw != 0.0 || sg.q_mvar != 0.0) ++counts.state_seed_components;
    if (has_text(sg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(sg.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.loads, counts, [&](const Load& load) {
    if (has_text(load.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(load.dynamic_model)) ++counts.dynamic_profile_components;
    if (load.controllable) ++counts.controllable_event_components;
    if (load.p_mw != 0.0 || load.q_mvar != 0.0 || load.scaling != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(load.name) || load.n_customers > 0) {
      ++counts.telemetry_anchor_components;
    }
    if (has_dynamic_provenance(load.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.flexible_loads, counts, [&](const FlexibleLoad& load) {
    if (has_text(load.name)) ++counts.named_components;
    ++counts.controllable_event_components;
    if (load.p_mw != 0.0 || load.q_mvar != 0.0) ++counts.state_seed_components;
    if (has_text(load.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.shunts, counts, [&](const Shunt& shunt) {
    if (has_text(shunt.name)) ++counts.named_components;
    if (shunt.switchable) ++counts.controllable_event_components;
    if (shunt.gs_mw != 0.0 || shunt.bs_mvar != 0.0 ||
        shunt.current_step != 0) {
      ++counts.state_seed_components;
    }
    if (has_text(shunt.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.asymmetric_loads, counts, [&](const AsymmetricLoad& load) {
    if (has_text(load.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(load.dynamic_model)) ++counts.dynamic_profile_components;
    if (load.pa_mw != 0.0 || load.pb_mw != 0.0 || load.pc_mw != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(load.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(load.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.storage, counts, [&](const Storage& st) {
    if (has_text(st.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(st.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(st)) {
      ++counts.reliability_metadata_components;
    }
    if (st.controllable) ++counts.controllable_event_components;
    if (st.soc_init >= 0.0 || st.p_mw != 0.0) ++counts.state_seed_components;
    if (has_text(st.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(st.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.renewable_gens, counts, [&](const RenewableGen& rg) {
    if (has_text(rg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(rg.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, rg.mttr_hours, 0.0,
                            rg.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (rg.curtailable) ++counts.controllable_event_components;
    if (rg.p_mw != 0.0 || rg.capacity_factor > 0.0) ++counts.state_seed_components;
    if (has_text(rg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(rg.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.pv_systems, counts, [&](const PVSystem& pv) {
    if (has_text(pv.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(pv.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, pv.mttr_hours, 0.0,
                            pv.mtbf_hours) ||
        pv.mtbf_panel_hours > 0.0 || pv.mtbf_inverter_hours > 0.0) {
      ++counts.reliability_metadata_components;
    }
    if (pv.controllable) ++counts.controllable_event_components;
    if (pv.p_mw != 0.0 || pv.irradiance > 0.0) ++counts.state_seed_components;
    if (has_text(pv.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(pv.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.external_grids, counts, [&](const ExternalGrid& eg) {
    if (has_text(eg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(eg.dynamic_model)) ++counts.dynamic_profile_components;
    if (eg.vm_pu != 0.0) ++counts.state_seed_components;
    if (has_text(eg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(eg.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.ac.transformers_2w, counts, [&](const Transformer2W& tr) {
    if (has_text(tr.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, tr.mttr_hours, 0.0,
                            tr.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (has_text(tr.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.transformers_3w, counts, [&](const Transformer3W& tr) {
    if (has_text(tr.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, tr.mttr_hours, 0.0,
                            tr.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (has_text(tr.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.switches, counts, [&](const Switch& sw) {
    if (has_text(sw.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(sw.p_sw_fail, 0.0, 0.0, sw.mttr_hours, 0.0,
                            sw.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    ++counts.state_seed_components;
    if (has_text(sw.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.circuit_breakers, counts,
                      [&](const CircuitBreaker& cb) {
                        if (has_text(cb.name)) ++counts.named_components;
                        ++counts.controllable_event_components;
                        ++counts.state_seed_components;
                        if (has_text(cb.name)) {
                          ++counts.telemetry_anchor_components;
                        }
                      });
  collect_twin_counts(sys.ac.regulator_controls, counts,
                      [&](const RegulatorControl& reg) {
                        if (has_text(reg.name)) ++counts.named_components;
                        ++counts.controllable_event_components;
                        ++counts.state_seed_components;
                        if (has_text(reg.name)) {
                          ++counts.telemetry_anchor_components;
                        }
                      });
  collect_twin_counts(sys.ac.chargers, counts, [&](const Charger& charger) {
    if (has_text(charger.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, charger.mttr_hours, 0.0,
                            charger.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (charger.v2g_capable) ++counts.controllable_event_components;
    if (charger.p_ev_kw != 0.0 || charger.p_ch_max_kw != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(charger.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.ac.charging_stations, counts, [&](const ChargingStation& st) {
    if (has_text(st.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, st.mttr_hours, 0.0,
                            st.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (st.p_total_kw != 0.0 || st.utilization_rate != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(st.name)) ++counts.telemetry_anchor_components;
  });

  collect_twin_counts(sys.dc.buses, counts, [&](const DCBus& b) {
    if (has_text(b.name)) ++counts.named_components;
    if (b.base_kv > 0.0 && b.vm_pu > 0.0) ++counts.state_seed_components;
    if (has_text(b.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.dc.branches, counts, [&](const DCBranch& br) {
    if (has_text(br.name)) ++counts.named_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, br.mttr_hours, 0.0,
                            br.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (has_text(br.name)) ++counts.telemetry_anchor_components;
  });
  collect_twin_counts(sys.dc.loads, counts, [&](const DCLoad& load) {
    if (has_text(load.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(load.dynamic_model)) ++counts.dynamic_profile_components;
    if (load.controllable) ++counts.controllable_event_components;
    if (load.p_mw != 0.0 || load.scaling != 0.0) ++counts.state_seed_components;
    if (has_text(load.name) || load.n_customers > 0) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(load.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.storage, counts, [&](const Storage& st) {
    if (has_text(st.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(st.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(st)) {
      ++counts.reliability_metadata_components;
    }
    if (st.controllable) ++counts.controllable_event_components;
    if (st.soc_init >= 0.0 || st.p_mw != 0.0) ++counts.state_seed_components;
    if (has_text(st.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(st.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.dc_storage, counts, [&](const DCStorage& st) {
    if (has_text(st.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(st.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(st)) {
      ++counts.reliability_metadata_components;
    }
    if (st.controllable) ++counts.controllable_event_components;
    if (st.soc_init >= 0.0 || st.p_mw != 0.0) ++counts.state_seed_components;
    if (has_text(st.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(st.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.dc_static_generators, counts, [&](const StaticGeneratorDC& sg) {
    if (has_text(sg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(sg.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, sg.mttr_hours, 0.0,
                            sg.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (sg.controllable) ++counts.controllable_event_components;
    if (sg.p_set_mw != 0.0 || sg.scaling != 0.0) ++counts.state_seed_components;
    if (has_text(sg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(sg.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.static_generators, counts, [&](const StaticGenerator& sg) {
    if (has_text(sg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(sg.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, sg.mttr_hours, 0.0,
                            sg.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (sg.controllable) ++counts.controllable_event_components;
    if (sg.p_mw != 0.0 || sg.q_mvar != 0.0) ++counts.state_seed_components;
    if (has_text(sg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(sg.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.pv_arrays, counts, [&](const PVArrayDC& pv) {
    if (has_text(pv.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(pv.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, pv.mttr_hours, 0.0,
                            pv.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    if (pv.p_set_mw != 0.0 || pv.irradiance > 0.0) ++counts.state_seed_components;
    if (has_text(pv.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(pv.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.dc.dcdc_converters, counts, [&](const DCDCConverter& c) {
    if (has_text(c.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(c.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, c.mttr_hours, 0.0,
                            c.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    if (c.p_ref_mw != 0.0 || c.v_ref_pu != 0.0) ++counts.state_seed_components;
    if (has_text(c.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(c.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.vsc_converters, counts, [&](const VSCConverter& c) {
    if (has_text(c.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(c.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, c.forced_outage_rate, c.mttr_hr, 0.0,
                            c.mtbf_hr, 0.0)) {
      ++counts.reliability_metadata_components;
    }
    if (c.controllable) ++counts.controllable_event_components;
    if (c.p_set_mw != 0.0 || c.v_ac_set_pu != 0.0 || c.v_dc_set_pu != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(c.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(c.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.energy_routers, counts, [&](const EnergyRouter& er) {
    if (has_text(er.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(er.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, er.mttr_hours, 0.0,
                            er.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    if (er.p_rated_mw != 0.0 || er.num_ports > 0) ++counts.state_seed_components;
    if (has_text(er.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(er.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.mobile_storage, counts, [&](const MobileStorage& st) {
    if (has_text(st.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(st.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(st)) {
      ++counts.reliability_metadata_components;
    }
    if (st.controllable) ++counts.controllable_event_components;
    if (st.soc_init >= 0.0 || st.p_mw != 0.0) ++counts.state_seed_components;
    if (has_text(st.name) || has_text(st.current_location)) {
      ++counts.telemetry_anchor_components;
    }
    if (has_dynamic_provenance(st.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.vpps, counts, [&](const VirtualPowerPlant& vpp) {
    if (has_text(vpp.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(vpp.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, vpp.mttr_hours, 0.0,
                            vpp.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    if (vpp.p_output_mw != 0.0 || vpp.p_generation_sum_mw != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(vpp.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(vpp.dynamic_model)) ++counts.provenance_components;
  });
  collect_twin_counts(sys.microgrids, counts, [&](const Microgrid& mg) {
    if (has_text(mg.name)) ++counts.named_components;
    ++counts.dynamic_capable_components;
    if (has_dynamic_profile(mg.dynamic_model)) ++counts.dynamic_profile_components;
    ++counts.reliability_capable_components;
    if (reliability_present(0.0, 0.0, 0.0, mg.mttr_hours, 0.0,
                            mg.mtbf_hours)) {
      ++counts.reliability_metadata_components;
    }
    ++counts.controllable_event_components;
    if (mg.v_set_pu != 0.0 || mg.f_set_hz != 0.0 || mg.p_exchange_mw != 0.0) {
      ++counts.state_seed_components;
    }
    if (has_text(mg.name)) ++counts.telemetry_anchor_components;
    if (has_dynamic_provenance(mg.dynamic_model)) ++counts.provenance_components;
  });

  if (sys.three_phase_ac.has_value()) {
    const auto& tp = *sys.three_phase_ac;
    collect_twin_counts(tp.buses, counts, [&](const ThreePhaseACBus& b) {
      if (has_text(b.name)) ++counts.named_components;
      if (b.base_kv > 0.0) ++counts.state_seed_components;
      if (has_text(b.name)) ++counts.telemetry_anchor_components;
    });
    collect_twin_counts(tp.lines, counts, [&](const ThreePhaseACLine& line) {
      if (has_text(line.name)) ++counts.named_components;
      ++counts.reliability_capable_components;
      if (reliability_present(line.failure_rate, 0.0, line.mttr_hr, 0.0,
                              0.0, 0.0)) {
        ++counts.reliability_metadata_components;
      }
      if (has_text(line.name)) ++counts.telemetry_anchor_components;
    });
    collect_twin_counts(tp.transformers,
                        counts,
                        [&](const ThreePhaseTransformer& tr) {
                          if (has_text(tr.name)) ++counts.named_components;
                          ++counts.reliability_capable_components;
                          if (reliability_present(0.0,
                                                  0.0,
                                                  tr.mttr_hr,
                                                  0.0,
                                                  tr.mtbf_hr,
                                                  0.0)) {
                            ++counts.reliability_metadata_components;
                          }
                          if (has_text(tr.name)) {
                            ++counts.telemetry_anchor_components;
                          }
                        });
    collect_twin_counts(tp.loads, counts, [&](const ThreePhaseLoad& load) {
      if (has_text(load.name)) ++counts.named_components;
      ++counts.dynamic_capable_components;
      if (has_dynamic_profile(load.dynamic_model)) ++counts.dynamic_profile_components;
      if (load.p_a_mw != 0.0 || load.p_b_mw != 0.0 || load.p_c_mw != 0.0) {
        ++counts.state_seed_components;
      }
      if (has_text(load.name)) ++counts.telemetry_anchor_components;
      if (has_dynamic_provenance(load.dynamic_model)) ++counts.provenance_components;
    });
    collect_twin_counts(tp.generators, counts, [&](const ThreePhaseGenerator& gen) {
      if (has_text(gen.name)) ++counts.named_components;
      ++counts.dynamic_capable_components;
      if (has_dynamic_profile(gen.dynamic_model)) ++counts.dynamic_profile_components;
      ++counts.controllable_event_components;
      if (gen.p_mw != 0.0 || gen.vm_pu != 0.0) ++counts.state_seed_components;
      if (has_text(gen.name)) ++counts.telemetry_anchor_components;
      if (has_dynamic_provenance(gen.dynamic_model)) ++counts.provenance_components;
    });
    collect_twin_counts(tp.external_grids, counts, [&](const ThreePhaseExternalGrid& eg) {
      if (has_text(eg.name)) ++counts.named_components;
      ++counts.dynamic_capable_components;
      if (has_dynamic_profile(eg.dynamic_model)) ++counts.dynamic_profile_components;
      if (eg.vm_pu != 0.0) ++counts.state_seed_components;
      if (has_text(eg.name)) ++counts.telemetry_anchor_components;
      if (has_dynamic_provenance(eg.dynamic_model)) ++counts.provenance_components;
    });
    collect_twin_counts(tp.regulator_controls,
                        counts,
                        [&](const ThreePhaseRegulatorControl& reg) {
                          if (has_text(reg.name)) ++counts.named_components;
                          ++counts.controllable_event_components;
                          ++counts.state_seed_components;
                          if (has_text(reg.name)) {
                            ++counts.telemetry_anchor_components;
                          }
                        });
  }
}

std::size_t bus_count(const HybridPowerSystem& sys) {
  std::size_t total = sys.ac.buses.size() + sys.dc.buses.size();
  if (sys.three_phase_ac.has_value()) total += sys.three_phase_ac->buses.size();
  return total;
}

std::size_t topology_edge_count(const HybridPowerSystem& sys) {
  std::size_t total = sys.ac.branches.size() + sys.ac.transformers_2w.size() +
                      sys.ac.transformers_3w.size() + sys.ac.switches.size() +
                      sys.ac.circuit_breakers.size() + sys.dc.branches.size() +
                      sys.dc.dcdc_converters.size() + sys.vsc_converters.size();
  if (sys.three_phase_ac.has_value()) {
    total += sys.three_phase_ac->lines.size() +
             sys.three_phase_ac->transformers.size();
  }
  return total;
}

std::string maturity_label(int level) {
  switch (level) {
    case 0:
      return "L0 file exchange";
    case 1:
      return "L1 static network model";
    case 2:
      return "L2 executable simulation model";
    case 3:
      return "L3 validated model";
    case 4:
      return "L4 synchronized operational twin";
    case 5:
      return "L5 predictive closed-loop twin";
  }
  return "Unknown";
}

int maturity_level(double readiness_ratio) {
  if (readiness_ratio >= 0.88) return 5;
  if (readiness_ratio >= 0.72) return 4;
  if (readiness_ratio >= 0.55) return 3;
  if (readiness_ratio >= 0.35) return 2;
  if (readiness_ratio >= 0.15) return 1;
  return 0;
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

const std::vector<ComponentParameterRule>& component_parameter_rules() {
  return parameter_registry();
}

const std::vector<DigitalTwinReadinessCriterion>&
digital_twin_readiness_criteria() {
  return twin_criteria_registry();
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

ComponentParameterAuditReport analyze_component_parameter_quality(
    const HybridPowerSystem& sys) {
  ComponentParameterAuditReport report;

  audit_collection(report, sys.ac.buses, "ACBus", "ac.buses",
                   [&](const ComponentIdentity& id, const ACBus& b) {
                     check_registered(report, id, "base_kv", b.base_kv);
                     check_registered(report, id, "vm_pu", b.vm_pu);
                     check_registered(report, id, "vmin_pu", b.vmin_pu);
                     check_registered(report, id, "vmax_pu", b.vmax_pu);
                     check_order(report, id, "vmin_pu <= vmax_pu", b.vmin_pu,
                                 b.vmax_pu, C::Static, S::IEC61970CIM,
                                 "CIM operational limit", "pu");
                   });

  audit_collection(report, sys.ac.branches, "ACBranch", "ac.branches",
                   [&](const ComponentIdentity& id, const ACBranch& br) {
                     check_registered(report, id, "r_pu", br.r_pu);
                     check_registered(report, id, "x_pu", br.x_pu);
                     check_registered(report, id, "rate_a_mva",
                                      br.rate_a_mva);
                     check_registered(report, id, "length_km", br.length_km);
                     check_registered(report, id, "failure_rate",
                                      br.failure_rate);
                     check_registered(report, id, "mttr_hr", br.mttr_hr);
                     check_registered(report, id, "t_scheduled_hr",
                                      br.t_scheduled_hr);
                   });

  audit_collection(report,
                   sys.ac.transformers_2w,
                   "Transformer2W",
                   "ac.transformers_2w",
                   [&](const ComponentIdentity& id, const Transformer2W& tr) {
                     check_registered(report, id, "sn_mva", tr.sn_mva);
                     check_registered(report, id, "vn_hv_kv", tr.vn_hv_kv);
                     check_registered(report, id, "vn_lv_kv", tr.vn_lv_kv);
                     check_registered(report, id, "vk_percent", tr.vk_percent);
                     check_registered(report, id, "vkr_percent",
                                      tr.vkr_percent);
                     if (tr.vk_percent > 0.0 || tr.vkr_percent > 0.0) {
                       check_order(report,
                                   id,
                                   "vkr_percent <= vk_percent",
                                   tr.vkr_percent,
                                   tr.vk_percent,
                                   C::Static,
                                   S::IEC61970CIM,
                                   "short-circuit impedance",
                                   "%");
                     }
                     if (tr.tap_min != 0 || tr.tap_max != 0 ||
                         tr.tap_pos != 0) {
                       check_order(report,
                                   id,
                                   "tap_min <= tap_pos",
                                   tr.tap_min,
                                   tr.tap_pos,
                                   C::Static,
                                   S::IEC61850,
                                   "tap changer",
                                   "step");
                       check_order(report,
                                   id,
                                   "tap_pos <= tap_max",
                                   tr.tap_pos,
                                   tr.tap_max,
                                   C::Static,
                                   S::IEC61850,
                                   "tap changer",
                                   "step");
                     }
                     check_registered(report, id, "mtbf_hours", tr.mtbf_hours);
                     check_registered(report, id, "mttr_hours", tr.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      tr.t_scheduled_hr);
                   });

  audit_collection(report,
                   sys.ac.transformers_3w,
                   "Transformer3W",
                   "ac.transformers_3w",
                   [&](const ComponentIdentity& id, const Transformer3W& tr) {
                     check_registered(report, id, "sn_hv_mva", tr.sn_hv_mva);
                     check_registered(report, id, "sn_mv_mva", tr.sn_mv_mva);
                     check_registered(report, id, "sn_lv_mva", tr.sn_lv_mva);
                     check_registered(report, id, "mtbf_hours", tr.mtbf_hours);
                     check_registered(report, id, "mttr_hours", tr.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      tr.t_scheduled_hr);
                     if (tr.vk_hv_mv_percent > 0.0 ||
                         tr.vkr_hv_mv_percent > 0.0) {
                       check_order(report,
                                   id,
                                   "vkr_hv_mv_percent <= vk_hv_mv_percent",
                                   tr.vkr_hv_mv_percent,
                                   tr.vk_hv_mv_percent,
                                   C::Static,
                                   S::IEC61970CIM,
                                   "three-winding short-circuit impedance",
                                   "%");
                     }
                   });

  audit_collection(report,
                   sys.ac.external_grids,
                   "ExternalGrid",
                   "ac.external_grids",
                   [&](const ComponentIdentity& id, const ExternalGrid& eg) {
                     check_registered(report, id, "vm_pu", eg.vm_pu);
                     check_registered(report, id, "s_sc_max_mva",
                                      eg.s_sc_max_mva);
                     if (eg.s_sc_max_mva > 0.0 || eg.s_sc_min_mva > 0.0) {
                       check_order(report,
                                   id,
                                   "s_sc_min_mva <= s_sc_max_mva",
                                   eg.s_sc_min_mva,
                                   eg.s_sc_max_mva,
                                   C::Static,
                                   S::IEC60909,
                                   "short-circuit source",
                                   "MVA");
                     }
                     audit_dynamic_profile(report, id, eg.dynamic_model);
                   });

  audit_collection(report, sys.ac.motors, "AsynchronousMotor", "ac.motors",
                   [&](const ComponentIdentity& id,
                       const AsynchronousMotor& motor) {
                     check_registered(report, id, "sn_mva", motor.sn_mva);
                     check_registered(report, id, "efficiency",
                                      motor.efficiency);
                     audit_dynamic_profile(report, id, motor.dynamic_model);
                   });

  audit_collection(report, sys.ac.generators, "Generator", "ac.generators",
                   [&](const ComponentIdentity& id, const Generator& gen) {
                     check_registered(report, id, "vg_pu", gen.vg_pu);
                     check_registered(report, id, "mbase_mva", gen.mbase_mva);
                     check_registered(report, id, "inertia_h", gen.inertia_h);
                     check_registered(report, id, "droop_r", gen.droop_r);
                     check_registered(report, id, "xd_pu", gen.xd_pu);
                     check_registered(report, id, "xq_pu", gen.xq_pu);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 gen.pmin_mw, gen.pmax_mw, C::Static, S::IEEE,
                                 "generator active limits", "MW");
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 gen.qmin_mvar, gen.qmax_mvar, C::Static,
                                 S::IEEE, "generator reactive limits", "MVAr");
                     check_registered(report, id, "forced_outage_rate",
                                      gen.forced_outage_rate);
                     check_registered(report, id, "mttr_hr", gen.mttr_hr);
                     check_registered(report, id, "t_scheduled_hr",
                                      gen.t_scheduled_hr);
                     audit_dynamic_profile(report, id, gen.dynamic_model);
                   });

  audit_collection(report,
                   sys.ac.static_generators,
                   "StaticGenerator",
                   "ac.static_generators",
                   [&](const ComponentIdentity& id, const StaticGenerator& sg) {
                     check_registered(report, id, "p_rated_mw", sg.p_rated_mw);
                     check_registered(report, id, "scaling", sg.scaling);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 sg.pmin_mw, sg.pmax_mw, C::Static,
                                 S::IEEE1547, "DER active limits", "MW",
                                 Sev::Warning);
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 sg.qmin_mvar, sg.qmax_mvar, C::Static,
                                 S::IEEE1547, "DER reactive limits", "MVAr",
                                 Sev::Warning);
                     check_registered(report, id, "mtbf_hours", sg.mtbf_hours);
                     check_registered(report, id, "mttr_hours", sg.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      sg.t_scheduled_hr);
                     audit_dynamic_profile(report, id, sg.dynamic_model);
                   });

  audit_collection(report,
                   sys.ac.renewable_gens,
                   "RenewableGen",
                   "ac.renewable_gens",
                   [&](const ComponentIdentity& id, const RenewableGen& rg) {
                     check_registered(report, id, "capacity_factor",
                                      rg.capacity_factor);
                     check_registered(report, id, "p_rated_mw",
                                      rg.p_rated_mw);
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 rg.qmin_mvar, rg.qmax_mvar, C::Static,
                                 S::NERC, "renewable reactive limits", "MVAr",
                                 Sev::Warning);
                     check_registered(report, id, "mtbf_hours", rg.mtbf_hours);
                     check_registered(report, id, "mttr_hours", rg.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      rg.t_scheduled_hr);
                     audit_dynamic_profile(report, id, rg.dynamic_model);
                   });

  audit_collection(report, sys.ac.pv_systems, "PVSystem", "ac.pv_systems",
                   [&](const ComponentIdentity& id, const PVSystem& pv) {
                     check_registered(report, id, "inverter_eff",
                                      pv.inverter_eff);
                     check_registered(report, id, "irradiance",
                                      pv.irradiance);
                     check_registered(report, id, "temperature",
                                      pv.temperature);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 pv.pmin_mw, pv.pmax_mw, C::Static,
                                 S::IEEE1547, "PV active limits", "MW",
                                 Sev::Warning);
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 pv.qmin_mvar, pv.qmax_mvar, C::Static,
                                 S::IEEE1547, "PV reactive limits", "MVAr",
                                 Sev::Warning);
                     check_registered(report, id, "mtbf_hours", pv.mtbf_hours);
                     check_registered(report, id, "mttr_hours", pv.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      pv.t_scheduled_hr);
                     check_registered(report, id, "mtbf_panel_hours",
                                      pv.mtbf_panel_hours);
                     check_registered(report, id, "mttr_panel_hours",
                                      pv.mttr_panel_hours);
                     check_registered(report, id, "mtbf_inverter_hours",
                                      pv.mtbf_inverter_hours);
                     check_registered(report, id, "mttr_inverter_hours",
                                      pv.mttr_inverter_hours);
                     audit_dynamic_profile(report, id, pv.dynamic_model);
                   });

  audit_collection(report, sys.ac.loads, "Load", "ac.loads",
                   [&](const ComponentIdentity& id, const Load& load) {
                     check_registered(report, id, "scaling", load.scaling);
                     check_registered(report, id, "motor_percent",
                                      load.motor_percent);
                     check_percent_sum(report,
                                       id,
                                       "ZIP active fractions",
                                       load.z_percent_p,
                                       load.i_percent_p,
                                       load.p_percent_p,
                                       S::GridLABD,
                                       "constant-power/ZIP load");
                     check_percent_sum(report,
                                       id,
                                       "ZIP reactive fractions",
                                       load.z_percent_q,
                                       load.i_percent_q,
                                       load.p_percent_q,
                                       S::GridLABD,
                                       "constant-power/ZIP load");
                     audit_dynamic_profile(report, id, load.dynamic_model);
                   });

  audit_collection(report,
                   sys.ac.flexible_loads,
                   "FlexibleLoad",
                   "ac.flexible_loads",
                   [&](const ComponentIdentity& id, const FlexibleLoad& load) {
                     check_registered(report, id, "flex_up_mw",
                                      load.flex_up_mw);
                     check_registered(report, id, "flex_down_mw",
                                      load.flex_down_mw);
                     check_registered(report, id, "availability_pct",
                                      load.availability_pct);
                   });

  audit_collection(report,
                   sys.ac.asymmetric_loads,
                   "AsymmetricLoad",
                   "ac.asymmetric_loads",
                   [&](const ComponentIdentity& id,
                       const AsymmetricLoad& load) {
                     check_registered(report, id, "scaling", load.scaling);
                     check_percent_sum(report,
                                       id,
                                       "phase ZIP fractions",
                                       load.const_z_percent,
                                       load.const_i_percent,
                                       load.const_p_percent,
                                       S::OpenDSS,
                                       "phase load model");
                     audit_dynamic_profile(report, id, load.dynamic_model);
                   });

  const auto audit_storage_common =
      [&](const ComponentIdentity& id,
          const auto& st,
          S family,
          const std::string& profile) {
        check_registered(report, id, "e_rated_mwh", st.e_rated_mwh);
        check_registered(report, id, "soc_init", st.soc_init);
        check_registered(report, id, "soc_min", st.soc_min);
        check_registered(report, id, "soc_max", st.soc_max);
        check_registered(report, id, "eta_charge", st.eta_charge);
        check_registered(report, id, "eta_discharge", st.eta_discharge);
        check_order(report, id, "soc_min <= soc_max", st.soc_min, st.soc_max,
                    C::Static, family, profile, "pu");
        check_order(report, id, "soc_min <= soc_init", st.soc_min,
                    st.soc_init, C::Static, family, profile, "pu");
        check_order(report, id, "soc_init <= soc_max", st.soc_init,
                    st.soc_max, C::Static, family, profile, "pu");
        check_order(report, id, "pmin_mw <= pmax_mw", st.pmin_mw, st.pmax_mw,
                    C::Static, family, profile, "MW", Sev::Warning);
        check_registered(report, id, "forced_outage_rate",
                         st.forced_outage_rate);
        check_registered(report, id, "mttr_hr", st.mttr_hr);
        check_registered(report, id, "t_scheduled_hr", st.t_scheduled_hr);
        audit_dynamic_profile(report, id, st.dynamic_model);
      };

  audit_collection(report, sys.ac.storage, "Storage", "ac.storage",
                   [&](const ComponentIdentity& id, const Storage& st) {
                     audit_storage_common(id, st, S::IEEE1547,
                                          "BESS DER profile");
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 st.qmin_mvar, st.qmax_mvar, C::Static,
                                 S::IEEE1547, "BESS reactive limits", "MVAr",
                                 Sev::Warning);
                   });

  audit_collection(report,
                   sys.mobile_storage,
                   "MobileStorage",
                   "mobile_storage",
                   [&](const ComponentIdentity& id, const MobileStorage& st) {
                     check_registered(report, id, "soc_init", st.soc_init);
                     check_registered(report, id, "eta_charge", st.eta_charge);
                     check_registered(report, id, "eta_discharge",
                                      st.eta_discharge);
                     check_order(report, id, "soc_min <= soc_max", st.soc_min,
                                 st.soc_max, C::Static, S::IEEE1547,
                                 "mobile BESS profile", "pu");
                     check_order(report,
                                 id,
                                 "soc_min <= soc_init",
                                 st.soc_min,
                                 st.soc_init,
                                 C::Static,
                                 S::IEEE1547,
                                 "mobile BESS profile",
                                 "pu");
                     check_order(report,
                                 id,
                                 "soc_init <= soc_max",
                                 st.soc_init,
                                 st.soc_max,
                                 C::Static,
                                 S::IEEE1547,
                                 "mobile BESS profile",
                                 "pu");
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 st.pmin_mw, st.pmax_mw, C::Static,
                                 S::IEEE1547, "mobile BESS active limits",
                                 "MW", Sev::Warning);
                     check_registered(report, id, "mtbf_hours", st.mtbf_hours);
                     check_registered(report, id, "mttr_hours", st.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      st.t_scheduled_hr);
                     audit_dynamic_profile(report, id, st.dynamic_model);
                   });

  audit_collection(report, sys.ac.chargers, "Charger", "ac.chargers",
                   [&](const ComponentIdentity& id, const Charger& charger) {
                     check_registered(report, id, "eta", charger.eta);
                     check_order(report, id, "p_ch_min_kw <= p_ch_max_kw",
                                 charger.p_ch_min_kw, charger.p_ch_max_kw,
                                 C::Static, S::IEC61850, "EVSE active limits",
                                 "kW", Sev::Warning);
                     check_registered(report, id, "mtbf_hours",
                                      charger.mtbf_hours);
                     check_registered(report, id, "mttr_hours",
                                      charger.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      charger.t_scheduled_hr);
                   });

  audit_collection(report,
                   sys.ac.charging_stations,
                   "ChargingStation",
                   "ac.charging_stations",
                   [&](const ComponentIdentity& id,
                       const ChargingStation& station) {
                     check_registered(report, id, "simultaneity_factor",
                                      station.simultaneity_factor);
                     check_registered(report, id, "power_factor",
                                      station.power_factor);
                     check_registered(report, id, "mtbf_hours",
                                      station.mtbf_hours);
                     check_registered(report, id, "mttr_hours",
                                      station.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      station.t_scheduled_hr);
                   });

  audit_collection(report, sys.ac.switches, "Switch", "ac.switches",
                   [&](const ComponentIdentity& id, const Switch& sw) {
                     check_registered(report, id, "r_contact_ohm",
                                      sw.r_contact_ohm);
                     check_registered(report, id, "p_sw_fail", sw.p_sw_fail);
                     check_registered(report, id, "mtbf_hours", sw.mtbf_hours);
                     check_registered(report, id, "mttr_hours", sw.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      sw.t_scheduled_hr);
                   });

  audit_collection(report,
                   sys.ac.circuit_breakers,
                   "CircuitBreaker",
                   "ac.circuit_breakers",
                   [&](const ComponentIdentity& id,
                       const CircuitBreaker& cb) {
                     check_registered(report, id, "rated_voltage_kv",
                                      cb.rated_voltage_kv);
                     check_registered(report, id, "i_breaking_ka",
                                      cb.i_breaking_ka);
                   });

  audit_collection(report, sys.dc.buses, "DCBus", "dc.buses",
                   [&](const ComponentIdentity& id, const DCBus& b) {
                     check_registered(report, id, "base_kv", b.base_kv);
                     check_registered(report, id, "vm_pu", b.vm_pu);
                     check_order(report, id, "vmin_pu <= vmax_pu", b.vmin_pu,
                                 b.vmax_pu, C::Static, S::HACDCPF,
                                 "DC bus voltage limits", "pu");
                   });

  audit_collection(report, sys.dc.branches, "DCBranch", "dc.branches",
                   [&](const ComponentIdentity& id, const DCBranch& br) {
                     check_registered(report, id, "r_pu", br.r_pu);
                     check_registered(report, id, "rate_a_mva",
                                      br.rate_a_mva);
                     check_registered(report, id, "length_km", br.length_km);
                     check_registered(report, id, "mtbf_hours", br.mtbf_hours);
                     check_registered(report, id, "mttr_hours", br.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      br.t_scheduled_hr);
                   });

  audit_collection(report, sys.dc.loads, "DCLoad", "dc.loads",
                   [&](const ComponentIdentity& id, const DCLoad& load) {
                     check_registered(report, id, "scaling", load.scaling);
                     check_percent_sum(report,
                                       id,
                                       "DC ZIP fractions",
                                       load.z_percent,
                                       load.i_percent,
                                       load.p_percent,
                                       S::HACDCPF,
                                       "DC load");
                     audit_dynamic_profile(report, id, load.dynamic_model);
                   });

  audit_collection(report, sys.dc.storage, "Storage", "dc.storage",
                   [&](const ComponentIdentity& id, const Storage& st) {
                     audit_storage_common(id, st, S::HACDCPF,
                                          "legacy DC storage profile");
                   });

  audit_collection(report, sys.dc.dc_storage, "DCStorage", "dc.dc_storage",
                   [&](const ComponentIdentity& id, const DCStorage& st) {
                     audit_storage_common(id, st, S::HACDCPF,
                                          "DC BESS profile");
                   });

  audit_collection(report,
                   sys.dc.dc_static_generators,
                   "StaticGeneratorDC",
                   "dc.dc_static_generators",
                   [&](const ComponentIdentity& id,
                       const StaticGeneratorDC& sg) {
                     check_registered(report, id, "pmax_mw", sg.pmax_mw);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 sg.pmin_mw, sg.pmax_mw, C::Static,
                                 S::HACDCPF, "DC source active limits", "MW",
                                 Sev::Warning);
                     check_registered(report, id, "mtbf_hours", sg.mtbf_hours);
                     check_registered(report, id, "mttr_hours", sg.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      sg.t_scheduled_hr);
                     audit_dynamic_profile(report, id, sg.dynamic_model);
                   });

  audit_collection(report, sys.dc.pv_arrays, "PVArrayDC", "dc.pv_arrays",
                   [&](const ComponentIdentity& id, const PVArrayDC& pv) {
                     check_registered(report, id, "irradiance",
                                      pv.irradiance);
                     check_registered(report, id, "temperature",
                                      pv.temperature);
                     check_registered(report, id, "mtbf_hours", pv.mtbf_hours);
                     check_registered(report, id, "mttr_hours", pv.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      pv.t_scheduled_hr);
                     audit_dynamic_profile(report, id, pv.dynamic_model);
                   });

  audit_collection(report,
                   sys.dc.dcdc_converters,
                   "DCDCConverter",
                   "dc.dcdc_converters",
                   [&](const ComponentIdentity& id, const DCDCConverter& c) {
                     check_registered(report, id, "eta", c.eta);
                     check_registered(report, id, "d_min", c.d_min);
                     check_registered(report, id, "d_max", c.d_max);
                     check_order(report, id, "d_min <= d_max", c.d_min,
                                 c.d_max, C::Transient, S::HACDCPF,
                                 "duty-ratio feasibility", "pu");
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 c.pmin_mw, c.pmax_mw, C::Static, S::HACDCPF,
                                 "DC/DC active limits", "MW", Sev::Warning);
                     check_registered(report, id, "mtbf_hours", c.mtbf_hours);
                     check_registered(report, id, "mttr_hours", c.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      c.t_scheduled_hr);
                     audit_dynamic_profile(report, id, c.dynamic_model);
                   });

  audit_collection(report, sys.vsc_converters, "VSCConverter",
                   "vsc_converters",
                   [&](const ComponentIdentity& id, const VSCConverter& c) {
                     check_registered(report, id, "eta", c.eta);
                     check_registered(report, id, "i_max_pu", c.i_max_pu);
                     check_registered(report, id, "v_ac_set_pu",
                                      c.v_ac_set_pu);
                     check_registered(report, id, "v_dc_set_pu",
                                      c.v_dc_set_pu);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 c.pmin_mw, c.pmax_mw, C::Static, S::NERC,
                                 "IBR active limits", "MW", Sev::Warning);
                     check_order(report, id, "qmin_mvar <= qmax_mvar",
                                 c.qmin_mvar, c.qmax_mvar, C::Static, S::NERC,
                                 "IBR reactive limits", "MVAr", Sev::Warning);
                     if (c.m_min != 0.0 || c.m_max != 0.0) {
                       check_parameter(report,
                                       id,
                                       "m_min",
                                       c.m_min,
                                       C::Transient,
                                       0.0,
                                       2.0,
                                       S::NERC,
                                       "modulation feasibility",
                                       "pu");
                       check_parameter(report,
                                       id,
                                       "m_max",
                                       c.m_max,
                                       C::Transient,
                                       0.0,
                                       2.0,
                                       S::NERC,
                                       "modulation feasibility",
                                       "pu");
                       check_order(report, id, "m_min <= m_max", c.m_min,
                                   c.m_max, C::Transient, S::NERC,
                                   "modulation feasibility", "pu");
                     }
                     check_registered(report, id, "forced_outage_rate",
                                      c.forced_outage_rate);
                     check_registered(report, id, "mttr_hr", c.mttr_hr);
                     check_registered(report, id, "mtbf_hr", c.mtbf_hr);
                     check_registered(report, id, "t_scheduled_hr",
                                      c.t_scheduled_hr);
                     audit_dynamic_profile(report, id, c.dynamic_model);
                   });

  audit_collection(report, sys.energy_routers, "EnergyRouter",
                   "energy_routers",
                   [&](const ComponentIdentity& id, const EnergyRouter& er) {
                     check_registered(report, id, "loss_percent",
                                      er.loss_percent);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 er.pmin_mw, er.pmax_mw, C::Static,
                                 S::HACDCPF, "energy router active limits",
                                 "MW", Sev::Warning);
                     check_registered(report, id, "mtbf_hours", er.mtbf_hours);
                     check_registered(report, id, "mttr_hours", er.mttr_hours);
                     audit_dynamic_profile(report, id, er.dynamic_model);
                     for (std::size_t i = 0; i < er.ports.size(); ++i) {
                       const auto& port = er.ports[i];
                       ComponentIdentity pid = id;
                       pid.component_type = "EnergyRouterPort";
                       pid.collection_path = "energy_routers.ports";
                       pid.component_position = i;
                       pid.component_index = port.index;
                       pid.component_name = er.name + ".port" +
                                            std::to_string(port.index);
                       ++report.component_instances_checked;
                       check_parameter(report, pid, "eta", port.eta,
                                       C::Static, 0.0, 1.0, S::HACDCPF,
                                       "energy router port efficiency", "pu",
                                       false, Sev::Warning, false, true);
                       check_order(report, pid, "pmin_mw <= pmax_mw",
                                   port.pmin_mw, port.pmax_mw, C::Static,
                                   S::HACDCPF,
                                   "energy router port active limits", "MW",
                                   Sev::Warning);
                       audit_dynamic_profile(report, pid, port.dynamic_model);
                     }
                   });

  audit_collection(report, sys.vpps, "VirtualPowerPlant", "vpps",
                   [&](const ComponentIdentity& id,
                       const VirtualPowerPlant& vpp) {
                     check_registered(report, id, "pmax_mw", vpp.pmax_mw);
                     check_order(report, id, "pmin_mw <= pmax_mw",
                                 vpp.pmin_mw, vpp.pmax_mw, C::Static, S::NERC,
                                 "DER aggregation active limits", "MW",
                                 Sev::Warning);
                     check_registered(report, id, "mtbf_hours",
                                      vpp.mtbf_hours);
                     check_registered(report, id, "mttr_hours",
                                      vpp.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      vpp.t_scheduled_hr);
                     audit_dynamic_profile(report, id, vpp.dynamic_model);
                   });

  audit_collection(report, sys.microgrids, "Microgrid", "microgrids",
                   [&](const ComponentIdentity& id, const Microgrid& mg) {
                     check_registered(report, id, "v_set_pu", mg.v_set_pu);
                     check_registered(report, id, "f_set_hz", mg.f_set_hz);
                     check_order(report, id, "p_exchange_min_mw <= "
                                            "p_exchange_max_mw",
                                 mg.p_exchange_min_mw, mg.p_exchange_max_mw,
                                 C::Static, S::IEEE1547,
                                 "microgrid PCC limits", "MW", Sev::Warning);
                     check_registered(report, id, "mtbf_hours",
                                      mg.mtbf_hours);
                     check_registered(report, id, "mttr_hours",
                                      mg.mttr_hours);
                     check_registered(report, id, "t_scheduled_hr",
                                      mg.t_scheduled_hr);
                     audit_dynamic_profile(report, id, mg.dynamic_model);
                   });

  if (sys.three_phase_ac.has_value()) {
    const auto& tp = *sys.three_phase_ac;
    audit_collection(report,
                     tp.buses,
                     "ThreePhaseACBus",
                     "three_phase_ac.buses",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseACBus& b) {
                       check_registered(report, id, "base_kv", b.base_kv);
                       check_registered(report, id, "vm_a_pu", b.vm_a_pu);
                       check_registered(report, id, "vm_b_pu", b.vm_b_pu);
                       check_registered(report, id, "vm_c_pu", b.vm_c_pu);
                       check_order(report, id, "vmin_pu <= vmax_pu",
                                   b.vmin_pu, b.vmax_pu, C::Static,
                                   S::OpenDSS, "phase-domain bus voltage",
                                   "pu");
                     });
    audit_collection(report,
                     tp.lines,
                     "ThreePhaseACLine",
                     "three_phase_ac.lines",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseACLine& line) {
                       check_registered(report, id, "length_km",
                                        line.length_km);
                       check_registered(report, id, "rate_a_mva",
                                        line.rate_a_mva);
                       check_registered(report, id, "failure_rate",
                                        line.failure_rate);
                       check_registered(report, id, "mttr_hr", line.mttr_hr);
                     });
    audit_collection(report,
                     tp.transformers,
                     "ThreePhaseTransformer",
                     "three_phase_ac.transformers",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseTransformer& tr) {
                       check_registered(report, id, "sn_mva", tr.sn_mva);
                       if (tr.vk_percent > 0.0 || tr.vkr_percent > 0.0) {
                         check_order(report,
                                     id,
                                     "vkr_percent <= vk_percent",
                                     tr.vkr_percent,
                                     tr.vk_percent,
                                     C::Static,
                                     S::OpenDSS,
                                     "phase transformer impedance",
                                     "%");
                       }
                       check_registered(report, id, "mtbf_hr", tr.mtbf_hr);
                       check_registered(report, id, "mttr_hr", tr.mttr_hr);
                     });
    audit_collection(report,
                     tp.loads,
                     "ThreePhaseLoad",
                     "three_phase_ac.loads",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseLoad& load) {
                       check_registered(report, id, "vmin_pu", load.vmin_pu);
                       check_registered(report, id, "vmax_pu", load.vmax_pu);
                       check_order(report, id, "vmin_pu <= vmax_pu",
                                   load.vmin_pu, load.vmax_pu, C::Static,
                                   S::OpenDSS, "phase load voltage model",
                                   "pu");
                       const double pz = load.p_const_z_percent >= 0.0
                                             ? load.p_const_z_percent
                                             : load.const_z_percent;
                       const double pi = load.p_const_i_percent >= 0.0
                                             ? load.p_const_i_percent
                                             : load.const_i_percent;
                       const double pp = load.p_const_p_percent >= 0.0
                                             ? load.p_const_p_percent
                                             : load.const_p_percent;
                       check_percent_sum(report, id, "phase ZIP active "
                                                   "fractions",
                                         pz, pi, pp, S::OpenDSS,
                                         "phase ZIP load");
                       audit_dynamic_profile(report, id, load.dynamic_model);
                     });
    audit_collection(report,
                     tp.generators,
                     "ThreePhaseGenerator",
                     "three_phase_ac.generators",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseGenerator& gen) {
                       check_registered(report, id, "vm_pu", gen.vm_pu);
                       check_order(report, id, "pmin_mw <= pmax_mw",
                                   gen.pmin_mw, gen.pmax_mw, C::Static,
                                   S::IEEE, "phase generator active limits",
                                   "MW", Sev::Warning);
                       check_order(report, id, "qmin_mvar <= qmax_mvar",
                                   gen.qmin_mvar, gen.qmax_mvar, C::Static,
                                   S::IEEE, "phase generator reactive limits",
                                   "MVAr", Sev::Warning);
                       audit_dynamic_profile(report, id, gen.dynamic_model);
                     });
    audit_collection(report,
                     tp.external_grids,
                     "ThreePhaseExternalGrid",
                     "three_phase_ac.external_grids",
                     [&](const ComponentIdentity& id,
                         const ThreePhaseExternalGrid& eg) {
                       check_registered(report, id, "vm_pu", eg.vm_pu);
                       audit_dynamic_profile(report, id, eg.dynamic_model);
                     });
  }

  std::stable_sort(report.findings.begin(),
                   report.findings.end(),
                   [](const ComponentParameterFinding& a,
                      const ComponentParameterFinding& b) {
                     const auto severity_rank = [](Sev severity) {
                       switch (severity) {
                         case Sev::Error:
                           return 0;
                         case Sev::Warning:
                           return 1;
                         case Sev::Info:
                           return 2;
                       }
                       return 3;
                     };
                     const int ar = severity_rank(a.severity);
                     const int br = severity_rank(b.severity);
                     if (ar != br) return ar < br;
                     if (a.collection_path != b.collection_path) {
                       return a.collection_path < b.collection_path;
                     }
                     if (a.component_position != b.component_position) {
                       return a.component_position < b.component_position;
                     }
                     return a.parameter_path < b.parameter_path;
                   });

  return report;
}

DigitalTwinReadinessReport analyze_digital_twin_readiness(
    const HybridPowerSystem& sys) {
  const auto coverage = analyze_component_io_coverage(sys);
  const auto parameter_audit = analyze_component_parameter_quality(sys);
  TwinCounts counts;
  collect_twin_counts(sys, counts);

  DigitalTwinReadinessReport report;
  std::map<DigitalTwinDimension, DigitalTwinDimensionScore> dimension_scores;

  const auto bounded = [](double value) {
    return std::clamp(value, 0.0, 1.0);
  };
  const auto safe_ratio = [](std::size_t numerator,
                             std::size_t denominator,
                             double empty_score = 0.0) {
    if (denominator == 0U) return empty_score;
    return ratio(numerator, denominator);
  };
  const auto add_finding = [&](std::string_view criterion_id,
                               double score_ratio,
                               std::string message,
                               std::string evidence) {
    const auto* criterion = find_twin_criterion(criterion_id);
    if (!criterion) return;
    const double max_score = std::max(criterion->weight, 0.0);
    const double score = bounded(score_ratio) * max_score;
    DigitalTwinReadinessFinding finding;
    finding.criterion_id = criterion->criterion_id;
    finding.dimension = criterion->dimension;
    finding.severity = readiness_severity(*criterion, bounded(score_ratio));
    finding.score = score;
    finding.max_score = max_score;
    finding.title = criterion->title;
    finding.message = std::move(message);
    finding.evidence = std::move(evidence);
    finding.standard_family = criterion->standard_family;
    finding.standard_profile = criterion->standard_profile;
    report.findings.push_back(std::move(finding));
    report.score += score;
    report.max_score += max_score;
    auto& dim = dimension_scores[criterion->dimension];
    dim.dimension = criterion->dimension;
    dim.score += score;
    dim.max_score += max_score;
    ++dim.findings;
  };

  const std::size_t total_instances = coverage.total_instances();
  const std::size_t buses = bus_count(sys);
  const std::size_t edges = topology_edge_count(sys);

  {
    const double score = safe_ratio(
        counts.named_components, counts.total_components, 0.0);
    std::ostringstream evidence;
    evidence << counts.named_components << "/" << counts.total_components
             << " components have stable names.";
    add_finding("DT-IDENTITY-01",
                score,
                "资产命名覆盖率决定遥测、事件、维护记录和外部标准模型能否引用同一对象。",
                evidence.str());
  }

  {
    double score = 0.0;
    if (buses > 0U) {
      score = edges > 0U ? 1.0 : 0.55;
      if (sys.ac.buses.empty() && sys.dc.buses.empty() &&
          !sys.three_phase_ac.has_value()) {
        score = 0.25;
      }
    }
    std::ostringstream evidence;
    evidence << buses << " bus object(s), " << edges
             << " branch/transformer/converter edge object(s).";
    add_finding("DT-TOPOLOGY-01",
                score,
                "数字孪生需要可求解拓扑；孤立元件可以保存，但不能支撑网络级校核。",
                evidence.str());
  }

  {
    const std::size_t errors = parameter_audit.count(Sev::Error);
    const std::size_t warnings = parameter_audit.count(Sev::Warning);
    const double checked =
        static_cast<double>(std::max<std::size_t>(
            parameter_audit.checked_parameters, 1U));
    double score =
        total_instances == 0U
            ? 0.0
            : bounded(1.0 -
                      (6.0 * static_cast<double>(errors) +
                       static_cast<double>(warnings)) /
                          checked);
    if (errors > 0U) {
      score = std::min(score, 0.60);
    } else if (warnings > 0U) {
      score = std::min(score, 0.88);
    }
    std::ostringstream evidence;
    evidence << parameter_audit.checked_parameters << " checked parameters, "
             << errors << " errors, " << warnings << " warnings.";
    add_finding("DT-PARAM-01",
                score,
                "静态、动态、暂态、故障和可靠性参数必须落在声明标准/profile的合理范围内。",
                evidence.str());
  }

  {
    const double empty_score =
        counts.total_components == 0U
            ? 0.0
            : (counts.dynamic_capable_components == 0U ? 0.60 : 0.0);
    const double score = safe_ratio(counts.dynamic_profile_components,
                                    counts.dynamic_capable_components,
                                    empty_score);
    std::ostringstream evidence;
    evidence << counts.dynamic_profile_components << "/"
             << counts.dynamic_capable_components
             << " transient-capable components carry DynamicModelProfile.";
    add_finding("DT-DYNAMIC-01",
                score,
                "暂态仿真和跨工具验证要求发电机、负荷、储能和变流器等元件带有可追溯动态模型。",
                evidence.str());
  }

  {
    const double score = safe_ratio(counts.telemetry_anchor_components,
                                    counts.total_components,
                                    0.0);
    std::ostringstream evidence;
    evidence << counts.telemetry_anchor_components << "/"
             << counts.total_components
             << " components expose a telemetry anchor such as name/customer/location.";
    add_finding("DT-TELEM-01",
                score,
                "遥测绑定需要稳定锚点；未来可把这些锚点扩展为IEC 61850逻辑节点、SCADA tag和PMU通道。",
                evidence.str());
  }

  {
    const double score = safe_ratio(
        counts.state_seed_components, counts.total_components, 0.0);
    std::ostringstream evidence;
    evidence << counts.state_seed_components << "/" << counts.total_components
             << " components have voltage/dispatch/SOC/status initialization.";
    add_finding("DT-STATE-01",
                score,
                "潮流解、SOC、开关状态和控制设定值是从离线模型进入在线孪生同步的初始状态。",
                evidence.str());
  }

  {
    const double score = safe_ratio(counts.controllable_event_components,
                                    counts.total_components,
                                    counts.total_components == 0U ? 0.0
                                                                  : 0.35);
    std::ostringstream evidence;
    evidence << counts.controllable_event_components << "/"
             << counts.total_components
             << " components expose controllable/event surfaces.";
    add_finding("DT-EVENT-01",
                score,
                "故障、投切、负荷扰动、DER参考值和恢复动作需要从IO层明确暴露，不能只靠GUI预设。",
                evidence.str());
  }

  {
    const double score = safe_ratio(counts.reliability_metadata_components,
                                    counts.reliability_capable_components,
                                    counts.total_components == 0U ? 0.0
                                                                  : 0.55);
    std::ostringstream evidence;
    evidence << counts.reliability_metadata_components << "/"
             << counts.reliability_capable_components
             << " reliability-capable components include failure/repair data.";
    add_finding("DT-RELIABILITY-01",
                score,
                "可靠性、风险和维护类数字孪生需要failure rate、FOR、MTTR/MTBF等生命周期字段。",
                evidence.str());
  }

  {
    const double total = static_cast<double>(std::max<std::size_t>(
        total_instances, 1U));
    const double json =
        static_cast<double>(coverage.represented_instances(
            ComponentIOFormat::InternalJSON)) /
        total;
    const double canonical =
        static_cast<double>(coverage.represented_instances(
            ComponentIOFormat::CanonicalModel)) /
        total;
    const double gridlabd =
        static_cast<double>(coverage.represented_instances(
            ComponentIOFormat::GridLABD)) /
        total;
    const double opendss =
        static_cast<double>(coverage.represented_instances(
            ComponentIOFormat::OpenDSS)) /
        total;
    const double score =
        total_instances == 0U
            ? 0.0
            : bounded(0.30 * json + 0.25 * canonical + 0.225 * gridlabd +
                      0.225 * opendss);
    std::ostringstream evidence;
    evidence << "represented instances: JSON "
             << coverage.represented_instances(ComponentIOFormat::InternalJSON)
             << ", Canonical "
             << coverage.represented_instances(
                    ComponentIOFormat::CanonicalModel)
             << ", GridLAB-D "
             << coverage.represented_instances(ComponentIOFormat::GridLABD)
             << ", OpenDSS "
             << coverage.represented_instances(ComponentIOFormat::OpenDSS)
             << " of " << total_instances << ".";
    add_finding("DT-IO-01",
                score,
                "IO层应保留本模块rich语义，同时声明向Canonical、GridLAB-D、OpenDSS等外部格式投影时的信息损失。",
                evidence.str());
  }

  {
    std::size_t populated = 0;
    std::size_t verifiable = 0;
    std::size_t external_scope = 0;
    for (const auto& item : coverage.items) {
      if (item.count == 0U) continue;
      ++populated;
      const auto scope = item.mapping.verification_scope;
      if (scope != NumericalVerificationScope::StructuralOnly &&
          scope != NumericalVerificationScope::NotApplicable) {
        ++verifiable;
      }
      if (scope == NumericalVerificationScope::ExternalPowerFlow ||
          scope == NumericalVerificationScope::BoundaryInjectionSnapshot) {
        ++external_scope;
      }
    }
    const double score =
        total_instances == 0U
            ? 0.0
            : bounded(0.65 * safe_ratio(verifiable, populated, 0.0) +
                      0.35 * safe_ratio(external_scope, populated, 0.0));
    std::ostringstream evidence;
    evidence << verifiable << "/" << populated
             << " populated component classes have numerical verification "
                "scope; "
             << external_scope << " are cross-checkable with external solvers.";
    add_finding("DT-VALID-01",
                score,
                "组件级、小系统、规模化系统和外部工具对照应形成验证金字塔，否则只能称为导入成功。",
                evidence.str());
  }

  {
    const double provenance =
        safe_ratio(counts.provenance_components,
                   counts.dynamic_capable_components,
                   counts.total_components == 0U ? 0.0 : 0.35);
    const double registry_health =
        total_instances == 0U || component_parameter_rules().empty() ||
                digital_twin_readiness_criteria().empty()
            ? 0.0
            : 1.0;
    const double score = bounded(0.70 * provenance + 0.30 * registry_health);
    std::ostringstream evidence;
    evidence << counts.provenance_components << "/"
             << counts.dynamic_capable_components
             << " dynamic-capable components include standard/source/profile "
                "provenance; "
             << component_parameter_rules().size()
             << " parameter rules and "
             << digital_twin_readiness_criteria().size()
             << " twin criteria are registered.";
    add_finding("DT-GOV-01",
                score,
                "模型来源、参数集、标准族和诊断结果需要可追溯，才能支持持续校准和版本治理。",
                evidence.str());
  }

  for (const auto& [_, score] : dimension_scores) {
    report.dimension_scores.push_back(score);
  }
  std::stable_sort(report.dimension_scores.begin(),
                   report.dimension_scores.end(),
                   [](const DigitalTwinDimensionScore& a,
                      const DigitalTwinDimensionScore& b) {
                     return to_string(a.dimension) < to_string(b.dimension);
                   });
  std::stable_sort(report.findings.begin(),
                   report.findings.end(),
                   [](const DigitalTwinReadinessFinding& a,
                      const DigitalTwinReadinessFinding& b) {
                     const int ar = severity_rank(a.severity);
                     const int br = severity_rank(b.severity);
                     if (ar != br) return ar < br;
                     return a.criterion_id < b.criterion_id;
                   });

  report.readiness_ratio =
      report.max_score > 0.0 ? bounded(report.score / report.max_score) : 0.0;
  report.maturity_level = maturity_level(report.readiness_ratio);
  report.maturity_label = maturity_label(report.maturity_level);
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

std::size_t ComponentParameterAuditReport::count(
    ComponentParameterSeverity severity) const {
  return static_cast<std::size_t>(
      std::count_if(findings.begin(), findings.end(),
                    [&](const ComponentParameterFinding& finding) {
                      return finding.severity == severity;
                    }));
}

std::size_t ComponentParameterAuditReport::count(
    ComponentParameterCategory category) const {
  return static_cast<std::size_t>(
      std::count_if(findings.begin(), findings.end(),
                    [&](const ComponentParameterFinding& finding) {
                      return finding.category == category;
                    }));
}

std::size_t DigitalTwinReadinessReport::count(
    ComponentParameterSeverity severity) const {
  return static_cast<std::size_t>(
      std::count_if(findings.begin(), findings.end(),
                    [&](const DigitalTwinReadinessFinding& finding) {
                      return finding.severity == severity;
                    }));
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

std::string to_string(ComponentParameterCategory category) {
  switch (category) {
    case ComponentParameterCategory::Static:
      return "Static";
    case ComponentParameterCategory::Dynamic:
      return "Dynamic";
    case ComponentParameterCategory::Transient:
      return "Transient";
    case ComponentParameterCategory::Failure:
      return "Failure";
    case ComponentParameterCategory::Reliability:
      return "Reliability";
  }
  return "Unknown";
}

std::string to_string(ComponentParameterSeverity severity) {
  switch (severity) {
    case ComponentParameterSeverity::Info:
      return "Info";
    case ComponentParameterSeverity::Warning:
      return "Warning";
    case ComponentParameterSeverity::Error:
      return "Error";
  }
  return "Unknown";
}

std::string to_string(DigitalTwinDimension dimension) {
  switch (dimension) {
    case DigitalTwinDimension::AssetIdentity:
      return "AssetIdentity";
    case DigitalTwinDimension::TopologyConnectivity:
      return "TopologyConnectivity";
    case DigitalTwinDimension::ElectricalParameters:
      return "ElectricalParameters";
    case DigitalTwinDimension::DynamicBehavior:
      return "DynamicBehavior";
    case DigitalTwinDimension::TelemetryObservability:
      return "TelemetryObservability";
    case DigitalTwinDimension::StateSynchronization:
      return "StateSynchronization";
    case DigitalTwinDimension::ScenarioEvents:
      return "ScenarioEvents";
    case DigitalTwinDimension::ReliabilityLifecycle:
      return "ReliabilityLifecycle";
    case DigitalTwinDimension::StandardsInteroperability:
      return "StandardsInteroperability";
    case DigitalTwinDimension::NumericalValidation:
      return "NumericalValidation";
    case DigitalTwinDimension::ProvenanceGovernance:
      return "ProvenanceGovernance";
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

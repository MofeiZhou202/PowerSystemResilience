// reliability/failure_mode.cpp
//
// Failure-mode catalog: expands every rich component in a HybridPowerSystem
// into its default failure modes (passive physical, active-on-demand, and
// cyber/control), resolving each mode's reliability parameters through the one
// shared resolver.  This is Layer 2 of the failure-mode-centric architecture
// in docs/reliability_assessment_code_review.md.

#include "hacdcpf/reliability/failure_mode.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// String helpers
// ═══════════════════════════════════════════════════════════════════════
std::string to_string(ReliabilityComponentKind k) {
  switch (k) {
    case ReliabilityComponentKind::ACGenerator:          return "ac_generator";
    case ReliabilityComponentKind::ACBranch:             return "ac_branch";
    case ReliabilityComponentKind::ACLoad:               return "ac_load";
    case ReliabilityComponentKind::ACBusLoad:            return "ac_bus_load";
    case ReliabilityComponentKind::ACStaticGenerator:    return "ac_static_generator";
    case ReliabilityComponentKind::ACRenewableGenerator: return "ac_renewable_generator";
    case ReliabilityComponentKind::ACStorage:            return "ac_storage";
    case ReliabilityComponentKind::ACPVSystem:           return "ac_pv_system";
    case ReliabilityComponentKind::ACTransformer2W:      return "ac_transformer_2w";
    case ReliabilityComponentKind::ACTransformer3W:      return "ac_transformer_3w";
    case ReliabilityComponentKind::ACSwitch:             return "ac_switch";
    case ReliabilityComponentKind::ACCircuitBreaker:     return "ac_circuit_breaker";
    case ReliabilityComponentKind::ExternalGrid:         return "external_grid";
    case ReliabilityComponentKind::DCBusLoad:            return "dc_bus_load";
    case ReliabilityComponentKind::DCBranch:             return "dc_branch";
    case ReliabilityComponentKind::DCLoad:               return "dc_load";
    case ReliabilityComponentKind::DCStaticGenerator:    return "dc_static_generator";
    case ReliabilityComponentKind::DCStaticGeneratorAC:  return "dc_static_generator_ac";
    case ReliabilityComponentKind::DCDCConverter:        return "dcdc_converter";
    case ReliabilityComponentKind::DCCircuitBreaker:     return "dc_circuit_breaker";
    case ReliabilityComponentKind::DCStorage:            return "dc_storage";
    case ReliabilityComponentKind::DCPVArray:            return "dc_pv_array";
    case ReliabilityComponentKind::VSCConverter:         return "vsc_converter";
    case ReliabilityComponentKind::EnergyRouter:         return "energy_router";
    case ReliabilityComponentKind::EnergyRouterPort:     return "energy_router_port";
    case ReliabilityComponentKind::Microgrid:            return "microgrid";
    case ReliabilityComponentKind::MobileStorage:        return "mobile_storage";
    case ReliabilityComponentKind::VirtualPowerPlant:    return "virtual_power_plant";
    case ReliabilityComponentKind::FlexibleLoad:         return "flexible_load";
    case ReliabilityComponentKind::AsymmetricLoad:       return "asymmetric_load";
    case ReliabilityComponentKind::Shunt:                return "shunt";
    case ReliabilityComponentKind::Charger:              return "charger";
    case ReliabilityComponentKind::ChargingStation:      return "charging_station";
    case ReliabilityComponentKind::AsynchronousMotor:    return "asynchronous_motor";
    case ReliabilityComponentKind::Unknown:              return "unknown";
  }
  return "unknown";
}

std::string to_string(FailureActivation a) {
  switch (a) {
    case FailureActivation::Passive:        return "passive";
    case FailureActivation::ActiveOnDemand: return "active_on_demand";
  }
  return "passive";
}

std::string to_string(FailureCause c) {
  switch (c) {
    case FailureCause::Physical:        return "physical";
    case FailureCause::CyberControl:    return "cyber_control";
    case FailureCause::Communication:   return "communication";
    case FailureCause::Measurement:     return "measurement";
    case FailureCause::ProtectionLogic: return "protection_logic";
    case FailureCause::HumanOperation:  return "human_operation";
    case FailureCause::Scheduled:       return "scheduled";
  }
  return "physical";
}

std::string to_string(FailureConsequenceKind c) {
  switch (c) {
    case FailureConsequenceKind::ForcedOutage:           return "forced_outage";
    case FailureConsequenceKind::Derating:               return "derating";
    case FailureConsequenceKind::StuckOpen:              return "stuck_open";
    case FailureConsequenceKind::StuckClosed:            return "stuck_closed";
    case FailureConsequenceKind::FailToOpen:             return "fail_to_open";
    case FailureConsequenceKind::FailToClose:            return "fail_to_close";
    case FailureConsequenceKind::FailToTrip:             return "fail_to_trip";
    case FailureConsequenceKind::NuisanceTrip:           return "nuisance_trip";
    case FailureConsequenceKind::ControlUnavailable:     return "control_unavailable";
    case FailureConsequenceKind::SetpointFrozen:         return "setpoint_frozen";
    case FailureConsequenceKind::MeasurementBias:        return "measurement_bias";
    case FailureConsequenceKind::CommunicationLoss:      return "communication_loss";
    case FailureConsequenceKind::GridFormingUnavailable: return "grid_forming_unavailable";
    case FailureConsequenceKind::ProtectionZoneTrip:     return "protection_zone_trip";
  }
  return "forced_outage";
}

namespace {

// ── Activation/cause filter check ────────────────────────────────────────
bool activation_allowed(FailureActivation a, const FailureModeCatalogOptions& o) {
  return a == FailureActivation::Passive ? o.include_passive : o.include_active_on_demand;
}
bool cause_allowed(FailureCause c, const FailureModeCatalogOptions& o) {
  switch (c) {
    case FailureCause::Physical:        return o.include_physical;
    case FailureCause::CyberControl:    return o.include_cyber_control;
    case FailureCause::Communication:   return o.include_communication;
    case FailureCause::Measurement:     return o.include_measurement;
    case FailureCause::ProtectionLogic: return o.include_protection_logic;
    case FailureCause::HumanOperation:  return o.include_human_operation;
    case FailureCause::Scheduled:       return o.include_scheduled;
  }
  return true;
}
bool kind_included(ReliabilityComponentKind k, const FailureModeCatalogOptions& o) {
  if (o.include_kinds.empty()) return true;
  for (auto x : o.include_kinds) if (x == k) return true;
  return false;
}

std::string nm(const std::string& name, const char* prefix, int idx) {
  return name.empty() ? (std::string(prefix) + std::to_string(idx)) : name;
}

// ── Per-component-kind domain string ─────────────────────────────────────
const char* domain_of(ReliabilityComponentKind k) {
  switch (k) {
    case ReliabilityComponentKind::DCBranch:
    case ReliabilityComponentKind::DCLoad:
    case ReliabilityComponentKind::DCBusLoad:
    case ReliabilityComponentKind::DCStaticGenerator:
    case ReliabilityComponentKind::DCStaticGeneratorAC:
    case ReliabilityComponentKind::DCCircuitBreaker:
    case ReliabilityComponentKind::DCStorage:
    case ReliabilityComponentKind::DCPVArray:
      return "DC";
    case ReliabilityComponentKind::DCDCConverter:
    case ReliabilityComponentKind::VSCConverter:
    case ReliabilityComponentKind::EnergyRouter:
    case ReliabilityComponentKind::EnergyRouterPort:
      return "Hybrid";
    default:
      return "AC";
  }
}

// ── Catalog builder context ──────────────────────────────────────────────
struct CatalogCtx {
  const FailureModeCatalogOptions& opt;
  const ReliabilityDataPolicy& policy;
  std::vector<FailureModeCatalogEntry>& out;

  ComponentRef make_ref(ReliabilityComponentKind k, int idx, const std::string& name) const {
    ComponentRef ref;
    ref.kind = k;
    ref.element_index = idx;
    ref.element_name = name;
    ref.domain = domain_of(k);
    ref.stable_id = to_string(k) + ":" +
                    (name.empty() ? std::to_string(idx) : name);
    return ref;
  }

  // Append one failure mode if its activation/cause pass the filters.
  // `raw` carries the mode's reliability inputs; def_lambda/def_repair are the
  // per-mode template fallbacks used when case data is incomplete.
  void add(const ComponentRef& ref, const std::string& suffix,
           const std::string& disp, FailureActivation act, FailureCause cause,
           FailureConsequenceKind cons, const ReliabilityRawFields& raw,
           double def_lambda, double def_repair,
           double iso_hr, double sw_hr, double residual_capacity = 0.5) {
    if (!kind_included(ref.kind, opt)) return;
    if (!activation_allowed(act, opt)) return;
    if (!cause_allowed(cause, opt)) return;

    FailureModeCatalogEntry e;
    e.mode.ref.component = ref;
    e.mode.ref.mode_id = ref.stable_id + "/" + suffix;
    e.mode.ref.display_name =
        (ref.element_name.empty() ? ref.stable_id : ref.element_name) + " — " + disp;
    e.mode.ref.activation = act;
    e.mode.ref.cause = cause;
    e.mode.ref.consequence = cons;

    e.mode.params = resolve_reliability_params(raw, policy, def_lambda, def_repair);
    e.mode.probability_per_demand = raw.probability_per_demand;
    e.mode.demand_frequency_per_year = raw.demand_frequency_per_year;
    e.mode.cyber_recovery_hr = raw.cyber_recovery_hr;
    e.mode.isolation_hr = iso_hr;
    e.mode.switching_hr = sw_hr;
    e.mode.repair_hr = e.mode.params.repair_hr;
    e.mode.residual_capacity_factor = residual_capacity;

    // Disable modes that resolved to missing data under strict policy.
    if (e.mode.params.data_source == "missing") {
      e.enabled = false;
      e.disabled_reason = "no case reliability data (strict policy)";
    }
    out.push_back(std::move(e));
  }
};

// Convenience raw-field builders.
ReliabilityRawFields rf_lambda(double lam, double mttr) {
  ReliabilityRawFields r; r.failure_rate_per_year = lam; r.mttr_hr = mttr; return r;
}
ReliabilityRawFields rf_for(double f, double mttr) {
  ReliabilityRawFields r; r.forced_outage_rate = f; r.mttr_hr = mttr; return r;
}
ReliabilityRawFields rf_mtbf(double mtbf, double mttr) {
  ReliabilityRawFields r; r.mtbf_hours = mtbf; r.mttr_hours = mttr; return r;
}
ReliabilityRawFields rf_active(double p_d, double nu_d, double recovery_hr,
                               bool is_template = true) {
  ReliabilityRawFields r;
  r.is_active = true;
  r.probability_per_demand = p_d;
  r.demand_frequency_per_year = nu_d;
  r.cyber_recovery_hr = recovery_hr;
  r.active_params_are_template = is_template;
  return r;
}
ReliabilityRawFields rf_cyber(double lam, double recovery_hr) {
  ReliabilityRawFields r;
  r.failure_rate_per_year = lam;
  r.cyber_recovery_hr = recovery_hr;
  return r;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Catalog builder
// ═══════════════════════════════════════════════════════════════════════
std::vector<FailureModeCatalogEntry> build_failure_mode_catalog(
    const HybridPowerSystem& sys,
    const FailureModeCatalogOptions& options,
    const ReliabilityDataPolicy& data_policy) {

  std::vector<FailureModeCatalogEntry> out;
  CatalogCtx ctx{options, data_policy, out};
  const double iso = options.default_isolation_hr;
  const double sw  = options.default_switching_hr;
  using K = ReliabilityComponentKind;
  using A = FailureActivation;
  using C = FailureCause;
  using Q = FailureConsequenceKind;

  auto inservice = [&](bool s) { return !options.only_in_service || s; };

  // ── AC generators ──
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    const auto& g = sys.ac.generators[i];
    if (!inservice(g.in_service)) continue;
    auto ref = ctx.make_ref(K::ACGenerator, (int)i, nm(g.name, "Gen_", g.index));
    ctx.add(ref, "forced_outage", "forced outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_for(g.forced_outage_rate, g.mttr_hr),
            8760.0 / 2000.0, 50.0, iso, sw);
    ctx.add(ref, "derating", "capacity derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.5, 24.0, iso, sw, 0.6);
    ctx.add(ref, "control_unavailable", "AGC/control unavailable", A::Passive,
            C::CyberControl, Q::ControlUnavailable, rf_cyber(0.0, 2.0),
            0.3, 2.0, iso, sw);
  }

  // ── AC branches ──
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& b = sys.ac.branches[i];
    if (!inservice(b.in_service)) continue;
    auto ref = ctx.make_ref(K::ACBranch, (int)i,
        nm(b.name, "ACBr_", b.index));
    ctx.add(ref, "permanent_fault", "permanent line/cable fault", A::Passive,
            C::Physical, Q::ForcedOutage, rf_lambda(b.failure_rate, b.mttr_hr),
            0.35, 10.0, iso, sw);
    ctx.add(ref, "thermal_derating", "thermal derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.1, 8.0, iso, sw, 0.75);
  }

  // ── Transformers 2W / 3W ──
  for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    if (!inservice(t.in_service)) continue;
    auto ref = ctx.make_ref(K::ACTransformer2W, (int)i, nm(t.name, "Trafo2W_", t.index));
    ctx.add(ref, "internal_fault", "internal fault", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(t.mtbf_hours, t.mttr_hours), 0.03, 200.0, iso, sw);
    ctx.add(ref, "tap_changer_failure", "tap-changer failure", A::ActiveOnDemand,
            C::Physical, Q::ControlUnavailable, rf_active(0.005, 50.0, 0.0),
            0.0, 24.0, iso, sw);
    ctx.add(ref, "cooling_derating", "cooling derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.05, 24.0, iso, sw, 0.7);
  }
  for (size_t i = 0; i < sys.ac.transformers_3w.size(); ++i) {
    const auto& t = sys.ac.transformers_3w[i];
    if (!inservice(t.in_service)) continue;
    auto ref = ctx.make_ref(K::ACTransformer3W, (int)i, nm(t.name, "Trafo3W_", t.index));
    ctx.add(ref, "internal_fault", "internal fault", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(t.mtbf_hours, t.mttr_hours), 0.04, 200.0, iso, sw);
    ctx.add(ref, "cooling_derating", "cooling derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.05, 24.0, iso, sw, 0.7);
  }

  // ── Static / renewable generators ──
  for (size_t i = 0; i < sys.ac.static_generators.size(); ++i) {
    const auto& sg = sys.ac.static_generators[i];
    if (!inservice(sg.in_service)) continue;
    auto ref = ctx.make_ref(K::ACStaticGenerator, (int)i, nm(sg.name, "SGen_", sg.index));
    ctx.add(ref, "unit_outage", "unit outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(sg.mtbf_hours, sg.mttr_hours), 1.5, 24.0, iso, sw);
    ctx.add(ref, "dispatch_control_unavailable", "dispatch/control unavailable",
            A::Passive, C::CyberControl, Q::ControlUnavailable, rf_cyber(0.0, 2.0),
            0.3, 2.0, iso, sw);
  }
  for (size_t i = 0; i < sys.ac.renewable_gens.size(); ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    if (!inservice(rg.in_service)) continue;
    auto ref = ctx.make_ref(K::ACRenewableGenerator, (int)i, nm(rg.name, "RGen_", rg.index));
    ctx.add(ref, "unit_outage", "unit outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(rg.mtbf_hours, rg.mttr_hours), 2.0, 48.0, iso, sw);
    ctx.add(ref, "resource_derating", "resource/availability derating", A::Passive,
            C::Physical, Q::Derating, rf_lambda(0.0, 0.0), 2.0, 4.0, iso, sw, 0.5);
  }

  // ── AC PV systems ──
  for (size_t i = 0; i < sys.ac.pv_systems.size(); ++i) {
    const auto& pv = sys.ac.pv_systems[i];
    if (!inservice(pv.in_service)) continue;
    auto ref = ctx.make_ref(K::ACPVSystem, (int)i, nm(pv.name, "PV_", pv.index));
    ctx.add(ref, "plant_outage", "whole plant outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(pv.mtbf_hours, pv.mttr_hours), 1.5, 24.0, iso, sw);
    ctx.add(ref, "inverter_failure", "inverter failure", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 1.0, 12.0, iso, sw, 0.5);
  }

  // ── AC storage ──
  for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!inservice(st.in_service)) continue;
    auto ref = ctx.make_ref(K::ACStorage, (int)i, nm(st.name, "BESS_", st.index));
    ctx.add(ref, "unit_outage", "whole unit outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_for(st.forced_outage_rate, st.mttr_hr), 1.0, 24.0, iso, sw);
    ctx.add(ref, "pcs_failure", "PCS power-stage failure", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.5, 12.0, iso, sw, 0.5);
    ctx.add(ref, "bms_control_unavailable", "BMS/control unavailable", A::Passive,
            C::CyberControl, Q::ControlUnavailable, rf_cyber(0.0, 2.0), 0.3, 2.0, iso, sw);
  }

  // ── AC switches ──
  for (size_t i = 0; i < sys.ac.switches.size(); ++i) {
    const auto& s = sys.ac.switches[i];
    if (!inservice(s.in_service)) continue;
    auto ref = ctx.make_ref(K::ACSwitch, (int)i, nm(s.name, "SW_", s.index));
    // Passive hardware outage.
    ReliabilityRawFields hw = rf_mtbf(s.mtbf_hours, s.mttr_hours);
    ctx.add(ref, "hardware_outage", "mechanism unavailable", A::Passive, C::Physical,
            Q::ForcedOutage, hw, 0.05, 4.0, iso, sw);
    const auto demand_probability = [&](double specific) {
      if (specific > 0.0 && specific < 1.0) return specific;
      return (s.p_sw_fail > 0.0 && s.p_sw_fail < 1.0) ? s.p_sw_fail : 0.0;
    };
    double p_open = demand_probability(s.p_fail_to_open);
    double p_close = demand_probability(s.p_fail_to_close);
    if (s.switch_type == SwitchType::Fuse) {
      ctx.add(ref, "fail_to_clear", "fuse fails to clear fault", A::ActiveOnDemand,
              C::ProtectionLogic, Q::FailToTrip,
              rf_active(p_open, 1.0, 0.0, p_open <= 0.0),
              0.005, 1.0, iso, sw);
    } else {
      const char* open_mode = s.switch_type == SwitchType::Sectionalizer
          ? "fail_to_sectionalize" : "fail_to_open";
      const char* open_description = s.switch_type == SwitchType::Sectionalizer
          ? "fails to open during upstream dead time" : "fail to open on command";
      ctx.add(ref, open_mode, open_description, A::ActiveOnDemand,
              C::Physical, Q::FailToOpen,
              rf_active(p_open, 2.0, 0.0, p_open <= 0.0),
              0.01, 1.0, iso, sw);
      if (s.switch_type == SwitchType::Recloser &&
          s.recloser_protection.successful_reclose_probability > 0.0) {
        p_close = 1.0 - std::clamp(
            s.recloser_protection.successful_reclose_probability, 0.0, 1.0);
      }
      ctx.add(ref, "fail_to_close", "fail to close during restoration",
              A::ActiveOnDemand, C::Physical, Q::FailToClose,
              rf_active(p_close, 2.0, 0.0, p_close <= 0.0),
              0.01, 1.0, iso, sw);
    }
    ctx.add(ref, "stuck_closed", "stuck closed", A::Passive, C::Physical,
            Q::StuckClosed, rf_lambda(0.0, 0.0), 0.005, 4.0, iso, sw);
    ctx.add(ref, "comm_loss", "remote command unavailable", A::Passive,
            C::Communication, Q::CommunicationLoss, rf_cyber(0.5, 2.0),
            0.5, 2.0, iso, sw);
  }

  // ── AC circuit breakers ──
  for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
    const auto& cb = sys.ac.circuit_breakers[i];
    if (!inservice(cb.in_service)) continue;
    auto ref = ctx.make_ref(K::ACCircuitBreaker, (int)i, nm(cb.name, "CB_", cb.index));
    ctx.add(ref, "hardware_outage", "breaker hardware outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.05, 8.0, iso, sw);
    ctx.add(ref, "fail_to_trip", "fail to trip for a fault", A::ActiveOnDemand,
            C::ProtectionLogic, Q::FailToTrip, rf_active(0.005, 1.0, 0.0), 0.005, 1.0, iso, sw);
    ctx.add(ref, "fail_to_close", "fail to close during restoration",
            A::ActiveOnDemand, C::Physical, Q::FailToClose, rf_active(0.01, 2.0, 0.0),
            0.01, 1.0, iso, sw);
    ctx.add(ref, "nuisance_trip", "nuisance trip", A::Passive, C::ProtectionLogic,
            Q::NuisanceTrip, rf_lambda(0.0, 0.0), 0.01, 1.0, iso, sw);
    ctx.add(ref, "comm_status_failure", "trip/close command or status failure",
            A::Passive, C::Communication, Q::CommunicationLoss, rf_cyber(0.3, 2.0),
            0.3, 2.0, iso, sw);
  }

  // ── VSC converters ──
  for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
    const auto& v = sys.vsc_converters[i];
    if (!inservice(v.in_service)) continue;
    auto ref = ctx.make_ref(K::VSCConverter, (int)i, nm(v.name, "VSC_", v.index));
    ctx.add(ref, "power_stage_outage", "power-stage outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_for(v.forced_outage_rate, v.mttr_hr), 0.10, 48.0, iso, sw);
    ctx.add(ref, "derating", "power-stage derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.2, 24.0, iso, sw, 0.7);
    ctx.add(ref, "grid_forming_lost", "grid-forming capability lost",
            A::ActiveOnDemand, C::CyberControl, Q::GridFormingUnavailable,
            rf_active(0.02, 1.0, 1.0), 0.0, 1.0, iso, sw);
    ctx.add(ref, "setpoint_frozen", "control setpoint frozen", A::Passive,
            C::CyberControl, Q::SetpointFrozen, rf_cyber(0.3, 1.0), 0.3, 1.0, iso, sw);
    ctx.add(ref, "comm_loss", "communication loss", A::Passive, C::Communication,
            Q::CommunicationLoss, rf_cyber(0.5, 2.0), 0.5, 2.0, iso, sw);
    ctx.add(ref, "measurement_bias", "measurement bias", A::Passive, C::Measurement,
            Q::MeasurementBias, rf_cyber(0.1, 4.0), 0.1, 4.0, iso, sw);
  }

  // ── DC branches ──
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& b = sys.dc.branches[i];
    if (!inservice(b.in_service)) continue;
    auto ref = ctx.make_ref(K::DCBranch, (int)i, nm(b.name, "DCBr_", b.index));
    ctx.add(ref, "pole_fault", "pole/cable permanent fault", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(b.mtbf_hours, b.mttr_hours), 0.20, 24.0, iso, sw);
    ctx.add(ref, "derating", "derating", A::Passive, C::Physical, Q::Derating,
            rf_lambda(0.0, 0.0), 0.1, 8.0, iso, sw, 0.75);
  }

  // ── DCDC converters ──
  for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
    const auto& d = sys.dc.dcdc_converters[i];
    if (!inservice(d.in_service)) continue;
    auto ref = ctx.make_ref(K::DCDCConverter, (int)i, nm(d.name, "DCDC_", d.index));
    ctx.add(ref, "power_stage_outage", "power-stage outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(d.mtbf_hours, d.mttr_hours), 0.20, 48.0, iso, sw);
    ctx.add(ref, "derating", "power-stage derating", A::Passive, C::Physical,
            Q::Derating, rf_lambda(0.0, 0.0), 0.2, 24.0, iso, sw, 0.7);
    ctx.add(ref, "setpoint_frozen", "duty/setpoint frozen", A::Passive,
            C::CyberControl, Q::SetpointFrozen, rf_cyber(0.3, 1.0), 0.3, 1.0, iso, sw);
    ctx.add(ref, "comm_loss", "communication loss", A::Passive, C::Communication,
            Q::CommunicationLoss, rf_cyber(0.5, 2.0), 0.5, 2.0, iso, sw);
  }

  // ── DC circuit breakers ──
  for (size_t i = 0; i < sys.dc.dc_circuit_breakers.size(); ++i) {
    const auto& cb = sys.dc.dc_circuit_breakers[i];
    if (!inservice(cb.in_service)) continue;
    auto ref = ctx.make_ref(K::DCCircuitBreaker, (int)i, nm(cb.name, "DCCB_", cb.index));
    ctx.add(ref, "hardware_outage", "breaker hardware outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.10, 8.0, iso, sw);
    ctx.add(ref, "fail_to_trip", "fail to trip for a DC fault", A::ActiveOnDemand,
            C::ProtectionLogic, Q::FailToTrip, rf_active(0.01, 1.0, 0.0), 0.01, 1.0, iso, sw);
    ctx.add(ref, "fail_to_close", "fail to close during restoration",
            A::ActiveOnDemand, C::Physical, Q::FailToClose, rf_active(0.01, 2.0, 0.0),
            0.01, 1.0, iso, sw);
  }

  // ── DC storage ──
  for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
    const auto& st = sys.dc.storage[i];
    if (!inservice(st.in_service)) continue;
    auto ref = ctx.make_ref(K::DCStorage, (int)i, nm(st.name, "DCStorage_", st.index));
    ctx.add(ref, "unit_outage", "whole unit outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_for(st.forced_outage_rate, st.mttr_hr), 1.0, 24.0, iso, sw);
    ctx.add(ref, "bms_control_unavailable", "BMS/control unavailable", A::Passive,
            C::CyberControl, Q::ControlUnavailable, rf_cyber(0.0, 2.0), 0.3, 2.0, iso, sw);
  }

  // ── DC PV arrays ──
  for (size_t i = 0; i < sys.dc.pv_arrays.size(); ++i) {
    const auto& pv = sys.dc.pv_arrays[i];
    if (!inservice(pv.in_service)) continue;
    auto ref = ctx.make_ref(K::DCPVArray, (int)i, nm(pv.name, "DCPV_", pv.index));
    ctx.add(ref, "array_outage", "array/string outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(pv.mtbf_hours, pv.mttr_hours), 1.5, 24.0, iso, sw);
  }

  // ── DC static generators (both containers) ──
  for (size_t i = 0; i < sys.dc.dc_static_generators.size(); ++i) {
    const auto& sg = sys.dc.dc_static_generators[i];
    if (!inservice(sg.in_service)) continue;
    auto ref = ctx.make_ref(K::DCStaticGenerator, (int)i, nm(sg.name, "DCSGen_", sg.index));
    ctx.add(ref, "source_outage", "source outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(sg.mtbf_hours, sg.mttr_hours), 1.5, 24.0, iso, sw);
  }
  for (size_t i = 0; i < sys.dc.static_generators.size(); ++i) {
    const auto& sg = sys.dc.static_generators[i];
    if (!inservice(sg.in_service)) continue;
    auto ref = ctx.make_ref(K::DCStaticGeneratorAC, (int)i, nm(sg.name, "DCSGen2_", sg.index));
    ctx.add(ref, "source_outage", "source outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(sg.mtbf_hours, sg.mttr_hours), 1.5, 24.0, iso, sw);
  }

  // ── External grids (upstream supply point) ──
  for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
    const auto& eg = sys.ac.external_grids[i];
    if (!inservice(eg.in_service)) continue;
    auto ref = ctx.make_ref(K::ExternalGrid, (int)i, nm(eg.name, "ExtGrid_", eg.index));
    // ExternalGrid carries no asset reliability fields; use a utility supply-point
    // template (rare but high-impact) when policy permits defaulting.
    ctx.add(ref, "supply_unavailable", "upstream supply unavailable", A::Passive,
            C::Physical, Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.1, 2.0, iso, sw);
    ctx.add(ref, "voltage_support_unavailable", "voltage support unavailable",
            A::Passive, C::CyberControl, Q::ControlUnavailable, rf_cyber(0.0, 2.0),
            0.2, 2.0, iso, sw);
  }

  // ── AC load points (lateral / service-transformer interruption) ──
  for (size_t i = 0; i < sys.ac.loads.size(); ++i) {
    const auto& ld = sys.ac.loads[i];
    if (!inservice(ld.in_service)) continue;
    auto ref = ctx.make_ref(K::ACLoad, (int)i, nm(ld.name, "Load_", ld.index));
    ctx.add(ref, "load_point_interruption", "load-point interruption", A::Passive,
            C::Physical, Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.2, 3.0, iso, sw);
    if (ld.controllable)
      ctx.add(ref, "control_failure", "controllable-load command failure",
              A::ActiveOnDemand, C::CyberControl, Q::ControlUnavailable,
              rf_active(0.01, 4.0, 0.0), 0.0, 1.0, iso, sw);
  }

  // ── AC flexible loads (demand response) ──
  for (size_t i = 0; i < sys.ac.flexible_loads.size(); ++i) {
    const auto& fl = sys.ac.flexible_loads[i];
    if (!inservice(fl.in_service)) continue;
    auto ref = ctx.make_ref(K::FlexibleLoad, (int)i, nm(fl.name, "FlexLoad_", fl.index));
    ctx.add(ref, "dr_unavailable", "demand response unavailable", A::ActiveOnDemand,
            C::CyberControl, Q::ControlUnavailable, rf_active(0.05, 4.0, 0.0),
            0.0, 1.0, iso, sw);
  }

  // ── AC asymmetric loads (phase-specific interruption) ──
  for (size_t i = 0; i < sys.ac.asymmetric_loads.size(); ++i) {
    const auto& al = sys.ac.asymmetric_loads[i];
    if (!inservice(al.in_service)) continue;
    auto ref = ctx.make_ref(K::AsymmetricLoad, (int)i, nm(al.name, "AsymLoad_", al.index));
    ctx.add(ref, "phase_interruption", "phase-specific load interruption", A::Passive,
            C::Physical, Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.2, 3.0, iso, sw);
  }

  // ── AC shunts (reactive support; no steady-state shed effect in DC-OPF) ──
  for (size_t i = 0; i < sys.ac.shunts.size(); ++i) {
    const auto& sh = sys.ac.shunts[i];
    if (!inservice(sh.in_service)) continue;
    auto ref = ctx.make_ref(K::Shunt, (int)i, nm(sh.name, "Shunt_", sh.index));
    ctx.add(ref, "shunt_unavailable", "shunt unavailable", A::Passive, C::Physical,
            Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.1, 8.0, iso, sw);
  }

  // ── EV chargers ──
  for (size_t i = 0; i < sys.ac.chargers.size(); ++i) {
    const auto& ch = sys.ac.chargers[i];
    if (!inservice(ch.in_service)) continue;
    auto ref = ctx.make_ref(K::Charger, (int)i, nm(ch.name, "Charger_", ch.index));
    ctx.add(ref, "charger_outage", "charger outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(ch.mtbf_hours, ch.mttr_hours), 1.0, 8.0, iso, sw);
    ctx.add(ref, "comm_failure", "communication/control failure", A::Passive,
            C::Communication, Q::CommunicationLoss, rf_cyber(0.5, 2.0), 0.5, 2.0, iso, sw);
  }

  // ── EV charging stations ──
  for (size_t i = 0; i < sys.ac.charging_stations.size(); ++i) {
    const auto& cs = sys.ac.charging_stations[i];
    if (!inservice(cs.in_service)) continue;
    auto ref = ctx.make_ref(K::ChargingStation, (int)i, nm(cs.name, "ChgStation_", cs.index));
    ctx.add(ref, "station_outage", "station outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(cs.mtbf_hours, cs.mttr_hours), 0.5, 8.0, iso, sw);
  }

  // ── Asynchronous motors (motor load) ──
  for (size_t i = 0; i < sys.ac.motors.size(); ++i) {
    const auto& mo = sys.ac.motors[i];
    if (!inservice(mo.in_service)) continue;
    auto ref = ctx.make_ref(K::AsynchronousMotor, (int)i, nm(mo.name, "Motor_", mo.index));
    ctx.add(ref, "motor_outage", "motor outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.5, 24.0, iso, sw);
    ctx.add(ref, "start_failure", "start failure on demand", A::ActiveOnDemand,
            C::Physical, Q::FailToClose, rf_active(0.01, 4.0, 0.0), 0.0, 2.0, iso, sw);
  }

  // ── DC load points ──
  for (size_t i = 0; i < sys.dc.loads.size(); ++i) {
    const auto& ld = sys.dc.loads[i];
    if (!inservice(ld.in_service)) continue;
    auto ref = ctx.make_ref(K::DCLoad, (int)i, nm(ld.name, "DCLoad_", ld.index));
    ctx.add(ref, "load_point_interruption", "DC load-point interruption", A::Passive,
            C::Physical, Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.2, 3.0, iso, sw);
    if (ld.controllable)
      ctx.add(ref, "control_failure", "controllable-load command failure",
              A::ActiveOnDemand, C::CyberControl, Q::ControlUnavailable,
              rf_active(0.01, 4.0, 0.0), 0.0, 1.0, iso, sw);
  }

  // ── Energy routers (+ ports) ──
  for (size_t i = 0; i < sys.energy_routers.size(); ++i) {
    const auto& er = sys.energy_routers[i];
    if (!inservice(er.in_service)) continue;
    auto ref = ctx.make_ref(K::EnergyRouter, (int)i, nm(er.name, "ERouter_", er.index));
    ctx.add(ref, "router_outage", "whole router outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(er.mtbf_hours, er.mttr_hours), 0.2, 48.0, iso, sw);
    ctx.add(ref, "routing_control_failure", "routing/control failure", A::Passive,
            C::CyberControl, Q::ControlUnavailable, rf_cyber(0.3, 2.0), 0.3, 2.0, iso, sw);
    ctx.add(ref, "comm_loss", "communication loss", A::Passive, C::Communication,
            Q::CommunicationLoss, rf_cyber(0.5, 2.0), 0.5, 2.0, iso, sw);
    for (size_t p = 0; p < er.ports.size(); ++p) {
      const auto& port = er.ports[p];
      if (!inservice(port.in_service)) continue;
      auto pref = ctx.make_ref(K::EnergyRouterPort, (int)p,
          nm(port.name, "ERPort_", port.index));
      ctx.add(pref, "port_outage", "port outage", A::Passive, C::Physical,
              Q::ForcedOutage, rf_lambda(0.0, 0.0), 0.3, 24.0, iso, sw);
    }
  }

  // ── Mobile storage (transportable ESS) ──
  for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
    const auto& ms = sys.mobile_storage[i];
    if (!inservice(ms.in_service)) continue;
    auto ref = ctx.make_ref(K::MobileStorage, (int)i, nm(ms.name, "MobStorage_", ms.index));
    ctx.add(ref, "unit_outage", "whole unit outage", A::Passive, C::Physical,
            Q::ForcedOutage, rf_mtbf(ms.mtbf_hours, ms.mttr_hours), 1.0, 24.0, iso, sw);
    ctx.add(ref, "vehicle_unavailable", "vehicle unavailable", A::ActiveOnDemand,
            C::HumanOperation, Q::ControlUnavailable, rf_active(0.02, 12.0, 0.0),
            0.0, 4.0, iso, sw);
    ctx.add(ref, "comm_control_failure", "communication/control failure", A::Passive,
            C::Communication, Q::CommunicationLoss, rf_cyber(0.5, 2.0), 0.5, 2.0, iso, sw);
  }

  // ── Virtual power plants (DER aggregation) ──
  for (size_t i = 0; i < sys.vpps.size(); ++i) {
    const auto& vpp = sys.vpps[i];
    if (!inservice(vpp.in_service)) continue;
    auto ref = ctx.make_ref(K::VirtualPowerPlant, (int)i, nm(vpp.name, "VPP_", vpp.index));
    ctx.add(ref, "aggregation_unavailable", "aggregation unavailable", A::Passive,
            C::Physical, Q::ForcedOutage, rf_mtbf(vpp.mtbf_hours, vpp.mttr_hours),
            0.5, 4.0, iso, sw);
    ctx.add(ref, "dispatch_command_failure", "dispatch command failure",
            A::ActiveOnDemand, C::CyberControl, Q::ControlUnavailable,
            rf_active(0.02, 12.0, 0.0), 0.0, 2.0, iso, sw);
  }

  // ── Microgrids (islanding / EMS) ──
  for (size_t i = 0; i < sys.microgrids.size(); ++i) {
    const auto& mg = sys.microgrids[i];
    if (!inservice(mg.in_service)) continue;
    auto ref = ctx.make_ref(K::Microgrid, (int)i, nm(mg.name, "MGrid_", mg.index));
    ctx.add(ref, "supply_outage", "grid-connected supply/export outage", A::Passive,
            C::Physical, Q::ForcedOutage, rf_mtbf(mg.mtbf_hours, mg.mttr_hours),
            0.3, 4.0, iso, sw);
    ctx.add(ref, "islanding_unavailable", "islanding unavailable", A::ActiveOnDemand,
            C::CyberControl, Q::GridFormingUnavailable, rf_active(0.05, 4.0, 1.0),
            0.0, 1.0, iso, sw);
    ctx.add(ref, "controller_comm_failure", "EMS/controller communication failure",
            A::Passive, C::Communication, Q::CommunicationLoss,
            rf_mtbf(mg.mtbf_hours, mg.mttr_hours), 0.3, 4.0, iso, sw);
  }

  return out;
}

// ═══════════════════════════════════════════════════════════════════════
// Coverage summary
// ═══════════════════════════════════════════════════════════════════════
FailureModeCoverage summarize_failure_mode_coverage(
    const std::vector<FailureModeCatalogEntry>& catalog) {
  FailureModeCoverage cov;
  std::vector<std::string> seen_components;
  for (const auto& e : catalog) {
    cov.modes_total++;
    if (e.enabled) cov.modes_enabled++; else cov.modes_disabled++;
    if (!e.supported_by_selected_consequence_model) cov.modes_unsupported++;

    const auto& src = e.mode.params.data_source;
    if (src == "case") cov.modes_case_data++;
    else if (src == "default" || e.mode.params.used_default) cov.modes_template_or_default++;
    else if (src == "missing") cov.modes_missing_data++;

    if (e.mode.ref.activation == FailureActivation::ActiveOnDemand) cov.modes_active++;
    else cov.modes_passive++;

    switch (e.mode.ref.cause) {
      case FailureCause::Physical:        cov.modes_physical++; break;
      case FailureCause::CyberControl:    cov.modes_cyber_control++; break;
      case FailureCause::ProtectionLogic: cov.modes_protection_logic++; break;
      default: break;
    }

    const std::string& cid = e.mode.ref.component.stable_id;
    bool found = false;
    for (const auto& s : seen_components) if (s == cid) { found = true; break; }
    if (!found) seen_components.push_back(cid);
  }
  cov.components_total = (int)seen_components.size();
  return cov;
}

// ═══════════════════════════════════════════════════════════════════════
// Consequence operator  Phi_m
// ═══════════════════════════════════════════════════════════════════════
std::string to_string(MutationKind k) {
  switch (k) {
    case MutationKind::ForceOutOfService:         return "force_out_of_service";
    case MutationKind::ForceLoadShed:             return "force_load_shed";
    case MutationKind::ProtectionZoneExpansion:   return "protection_zone_expansion";
    case MutationKind::ForceClosedNoIsolation:    return "force_closed_no_isolation";
    case MutationKind::CapacityDerate:            return "capacity_derate";
    case MutationKind::RemoveControllability:     return "remove_controllability";
    case MutationKind::RemoveGridForming:         return "remove_grid_forming";
    case MutationKind::RestorationActionForbidden: return "restoration_action_forbidden";
    case MutationKind::ObservationDegraded:       return "observation_degraded";
  }
  return "force_out_of_service";
}
std::string to_string(MutationCategory c) {
  switch (c) {
    case MutationCategory::Topology:    return "topology";
    case MutationCategory::Capacity:    return "capacity";
    case MutationCategory::Control:     return "control";
    case MutationCategory::Protection:  return "protection";
    case MutationCategory::Observation: return "observation";
    case MutationCategory::Restoration: return "restoration";
  }
  return "topology";
}

namespace {
// Component kinds whose in-service / capacity / control state the steady-state
// AC/DC shed engine (DC-OPF and the hybrid FMEA network LP) actually
// represents.  A forced-outage / derating / control mutation on any *other*
// kind has no steady-state shed effect, so build_consequence_patch reports it
// as unsupported rather than applying a silent no-op (code-review Finding 5:
// rich-component vs. canonical-solver divergence).
//
// The kinds returning false here (external grid, energy router/port, mobile
// storage, VPP, microgrid, load points, shunts, chargers, motors, …) are fully
// catalogued with resolved parameters, but wiring their consequences into the
// shed engine is the next milestone (effect-analysis stage).
bool steady_state_engine_models(ReliabilityComponentKind k) {
  using K = ReliabilityComponentKind;
  switch (k) {
    case K::ACGenerator:
    case K::ACBranch:
    case K::ACStaticGenerator:
    case K::ACRenewableGenerator:
    case K::ACStorage:
    case K::ACPVSystem:
    case K::ACTransformer2W:
    case K::ACTransformer3W:
    case K::ACSwitch:
    case K::ACCircuitBreaker:
    case K::DCBranch:
    case K::DCDCConverter:
    case K::DCCircuitBreaker:
    case K::DCStorage:
    case K::DCPVArray:
    case K::DCStaticGenerator:
    case K::DCStaticGeneratorAC:
    case K::VSCConverter:
      return true;
    default:
      return false;
  }
}
}  // namespace

ConsequencePatch build_consequence_patch(
    const HybridPowerSystem& /*sys*/,
    const FailureModeReliability& mode,
    const ConsequenceModelCapabilities& caps) {
  ConsequencePatch patch;
  patch.mode = mode.ref;
  const auto kind = mode.ref.component.kind;
  const int idx = mode.ref.component.element_index;
  patch.affected_canonical_ids.push_back(mode.ref.component.stable_id);

  auto add_mut = [&](MutationCategory cat, MutationKind mk, double factor,
                     const std::string& note) {
    ConsequenceMutation m;
    m.category = cat; m.kind = mk; m.target_kind = kind; m.target_index = idx;
    m.factor = factor; m.note = note;
    patch.mutations.push_back(m);
  };
  auto unsupported = [&](const std::string& reason) {
    patch.representable_by_selected_model = false;
    patch.unsupported_reason = reason;
    patch.warnings.push_back(reason);
  };

  // Load-point interruption: the failed load's own demand is curtailed and
  // counts as shed.  Loads are not dispatchable in the OPF/LP, so the FMEA
  // engine removes the load from demand and adds its MW to the shed result
  // (handled via the ForceLoadShed mutation).  This is supported.
  const bool is_load_point = kind == ReliabilityComponentKind::ACLoad ||
                             kind == ReliabilityComponentKind::DCLoad ||
                             kind == ReliabilityComponentKind::AsymmetricLoad;
  if (is_load_point) {
    if (mode.ref.consequence == FailureConsequenceKind::ForcedOutage) {
      add_mut(MutationCategory::Topology, MutationKind::ForceLoadShed, 0.0,
              "load point interrupted (demand curtailed = shed)");
      return patch;
    }
    // Other load consequences (e.g. controllable-load command failure) are a
    // demand-flexibility loss, not a shed event.
    unsupported("load-point '" + to_string(mode.ref.consequence) +
                "' is a flexibility/control effect, not a shed event");
    return patch;
  }

  // Aggregated supply / transfer sources (external grid, VPP, mobile storage,
  // microgrid, energy router) are materialized as dispatchable injections by the
  // FMEA engine; a forced outage removes that contribution (-> shed).  Only
  // forced-outage is modelled; their control/derating effects are not.
  const bool is_aggregated_source =
      kind == ReliabilityComponentKind::ExternalGrid ||
      kind == ReliabilityComponentKind::VirtualPowerPlant ||
      kind == ReliabilityComponentKind::MobileStorage ||
      kind == ReliabilityComponentKind::Microgrid ||
      kind == ReliabilityComponentKind::EnergyRouter;
  if (is_aggregated_source) {
    if (mode.ref.consequence == FailureConsequenceKind::ForcedOutage ||
        mode.ref.consequence == FailureConsequenceKind::NuisanceTrip) {
      add_mut(MutationCategory::Topology, MutationKind::ForceOutOfService, 0.0,
              "aggregated source removed from service (supplied load is shed)");
      return patch;
    }
    unsupported("only forced-outage of this aggregated source is modelled by "
                "the shed engine");
    return patch;
  }

  // Honest support gate (code-review Finding 5): a topology / capacity / control
  // consequence only has a steady-state shed effect when the engine actually
  // represents this component kind.  Otherwise report unsupported instead of a
  // silent no-op.  Measurement/communication/protection/restoration-only modes
  // are handled (also as unsupported here) by the switch below.
  switch (mode.ref.consequence) {
    case FailureConsequenceKind::ForcedOutage:
    case FailureConsequenceKind::NuisanceTrip:
    case FailureConsequenceKind::StuckOpen:
    case FailureConsequenceKind::Derating:
    case FailureConsequenceKind::ControlUnavailable:
    case FailureConsequenceKind::SetpointFrozen:
    case FailureConsequenceKind::GridFormingUnavailable:
      if (!steady_state_engine_models(kind)) {
        unsupported("component kind '" + to_string(kind) +
                    "' is not represented by the steady-state AC/DC shed engine "
                    "(consequence modelling pending)");
        return patch;
      }
      break;
    default:
      break;
  }

  switch (mode.ref.consequence) {
    case FailureConsequenceKind::ForcedOutage:
    case FailureConsequenceKind::StuckOpen:
      if (caps.supports_forced_outage)
        add_mut(MutationCategory::Topology, MutationKind::ForceOutOfService, 0.0,
                "element removed from service");
      else unsupported("selected model does not support forced-outage topology");
      break;
    case FailureConsequenceKind::NuisanceTrip:
      // A spurious trip removes the element from service (no downstream fault).
      if (caps.supports_forced_outage)
        add_mut(MutationCategory::Topology, MutationKind::ForceOutOfService, 0.0,
                "spurious trip removes element");
      else unsupported("selected model does not support forced-outage topology");
      break;
    case FailureConsequenceKind::Derating: {
      if (caps.supports_derating) {
        // Data-driven severity: the surviving-capacity fraction comes from the
        // failure mode (per-mode template value), not a hard-coded constant.
        const double residual =
            (mode.residual_capacity_factor > 0.0 &&
             mode.residual_capacity_factor <= 1.0)
                ? mode.residual_capacity_factor
                : 0.5;
        add_mut(MutationCategory::Capacity, MutationKind::CapacityDerate, residual,
                "capacity derated (surviving fraction " +
                    std::to_string(residual) + ")");
      } else unsupported("selected model does not support capacity derating");
      break;
    }
    case FailureConsequenceKind::ControlUnavailable:
    case FailureConsequenceKind::SetpointFrozen:
      if (caps.supports_control_unavailable)
        add_mut(MutationCategory::Control, MutationKind::RemoveControllability, 0.0,
                "dispatch/controllability removed (fixed setpoint)");
      else unsupported("selected model does not model control availability");
      break;
    case FailureConsequenceKind::GridFormingUnavailable:
      if (caps.supports_control_unavailable)
        add_mut(MutationCategory::Control, MutationKind::RemoveGridForming, 0.0,
                "island grid-forming capability removed");
      else unsupported("selected model does not model grid-forming capability");
      break;
    case FailureConsequenceKind::StuckClosed:
      // A stuck-closed sectionalizing device cannot open to isolate a fault, so
      // backup protection upstream clears a larger zone (Stage-1 isolation
      // failure of the three-stage restoration model).
      if (caps.supports_protection_modeling)
        add_mut(MutationCategory::Protection, MutationKind::ProtectionZoneExpansion,
                0.0, "stuck-closed device cannot isolate; outage zone expands");
      else unsupported("stuck-closed isolation failure needs the protection/restoration engine");
      break;
    case FailureConsequenceKind::FailToOpen:
      // Fail-to-open is the same Stage-1 isolation failure: the device cannot
      // sectionalize, so the upstream backup de-energizes a larger zone.
      if (caps.supports_protection_modeling)
        add_mut(MutationCategory::Protection, MutationKind::ProtectionZoneExpansion,
                0.0, "fail-to-open cannot isolate; outage zone expands");
      else unsupported("fail-to-open isolation failure needs the protection/restoration engine");
      break;
    case FailureConsequenceKind::FailToClose:
      // Fail-to-close is a Stage-2 restoration-path loss: a normally-open tie
      // that cannot close to back-feed.  This is only meaningful with a co-fault
      // restoration model, so it is deferred to the three-stage restoration
      // engine rather than faked as a steady-state state.
      unsupported("fail-to-close is a restoration-path loss; evaluate with the "
                  "three-stage restoration engine (no steady-state shed effect)");
      break;
    case FailureConsequenceKind::FailToTrip:
    case FailureConsequenceKind::ProtectionZoneTrip:
      if (caps.supports_protection_modeling)
        add_mut(MutationCategory::Protection, MutationKind::ProtectionZoneExpansion,
                0.0, "breaker fail-to-trip: backup protection expands the outage zone");
      else unsupported("protection misoperation needs a protection-zone model");
      break;
    case FailureConsequenceKind::CommunicationLoss: {
      // Loss of the remote dispatch / command channel.  For a *dispatchable*
      // converter or DER this freezes the operating setpoint: the device holds
      // its last command and can no longer be re-dispatched to support a
      // contingency, which the steady-state engine represents as loss of
      // controllability (pinned at its setpoint) -> possible shed.  For
      // non-dispatchable targets (switches, breakers, measurement-only assets)
      // a lost command channel has no steady-state shed effect.
      const bool dispatchable =
          kind == ReliabilityComponentKind::VSCConverter ||
          kind == ReliabilityComponentKind::DCDCConverter ||
          kind == ReliabilityComponentKind::ACStaticGenerator ||
          kind == ReliabilityComponentKind::DCStaticGenerator ||
          kind == ReliabilityComponentKind::ACStorage ||
          kind == ReliabilityComponentKind::DCStorage;
      if (dispatchable && caps.supports_control_unavailable)
        add_mut(MutationCategory::Control, MutationKind::RemoveControllability, 0.0,
                "communication loss freezes the dispatch setpoint (no re-dispatch)");
      else if (caps.supports_observation_cyber)
        add_mut(MutationCategory::Observation, MutationKind::ObservationDegraded, 0.0,
                "communication degraded (no steady-state shed effect)");
      else
        unsupported("communication loss on a non-dispatchable target has no "
                    "steady-state shed effect");
      break;
    }
    case FailureConsequenceKind::MeasurementBias:
      if (caps.supports_observation_cyber)
        add_mut(MutationCategory::Observation, MutationKind::ObservationDegraded, 0.0,
                "measurement bias (no steady-state shed effect)");
      else unsupported("measurement-only mode has no steady-state shed effect");
      break;
  }
  return patch;
}

namespace {

// Set the in-service flag of a target component by kind + index.
void set_in_service(HybridPowerSystem& s, ReliabilityComponentKind k, int i, bool v) {
  using K = ReliabilityComponentKind;
  auto ok = [&](size_t n) { return i >= 0 && (size_t)i < n; };
  switch (k) {
    case K::ACGenerator:         if (ok(s.ac.generators.size()))        s.ac.generators[i].in_service = v; break;
    case K::ACBranch:            if (ok(s.ac.branches.size()))          s.ac.branches[i].in_service = v; break;
    case K::ACLoad:              if (ok(s.ac.loads.size()))             s.ac.loads[i].in_service = v; break;
    case K::AsymmetricLoad:      if (ok(s.ac.asymmetric_loads.size()))  s.ac.asymmetric_loads[i].in_service = v; break;
    case K::ACStaticGenerator:   if (ok(s.ac.static_generators.size())) s.ac.static_generators[i].in_service = v; break;
    case K::ACRenewableGenerator:if (ok(s.ac.renewable_gens.size()))    s.ac.renewable_gens[i].in_service = v; break;
    case K::ACStorage:           if (ok(s.ac.storage.size()))           s.ac.storage[i].in_service = v; break;
    case K::ACPVSystem:          if (ok(s.ac.pv_systems.size()))        s.ac.pv_systems[i].in_service = v; break;
    case K::ACTransformer2W:     if (ok(s.ac.transformers_2w.size()))   s.ac.transformers_2w[i].in_service = v; break;
    case K::ACTransformer3W:     if (ok(s.ac.transformers_3w.size()))   s.ac.transformers_3w[i].in_service = v; break;
    case K::ACSwitch:            if (ok(s.ac.switches.size()))          s.ac.switches[i].in_service = v; break;
    case K::ACCircuitBreaker:    if (ok(s.ac.circuit_breakers.size()))  s.ac.circuit_breakers[i].in_service = v; break;
    case K::DCBranch:            if (ok(s.dc.branches.size()))          s.dc.branches[i].in_service = v; break;
    case K::DCLoad:              if (ok(s.dc.loads.size()))             s.dc.loads[i].in_service = v; break;
    case K::DCDCConverter:       if (ok(s.dc.dcdc_converters.size()))   s.dc.dcdc_converters[i].in_service = v; break;
    case K::DCCircuitBreaker:    if (ok(s.dc.dc_circuit_breakers.size())) s.dc.dc_circuit_breakers[i].in_service = v; break;
    case K::DCStorage:           if (ok(s.dc.storage.size()))           s.dc.storage[i].in_service = v; break;
    case K::DCPVArray:           if (ok(s.dc.pv_arrays.size()))         s.dc.pv_arrays[i].in_service = v; break;
    case K::DCStaticGenerator:   if (ok(s.dc.dc_static_generators.size())) s.dc.dc_static_generators[i].in_service = v; break;
    case K::DCStaticGeneratorAC: if (ok(s.dc.static_generators.size())) s.dc.static_generators[i].in_service = v; break;
    case K::VSCConverter:        if (ok(s.vsc_converters.size()))       s.vsc_converters[i].in_service = v; break;
    case K::ExternalGrid:        if (ok(s.ac.external_grids.size()))    s.ac.external_grids[i].in_service = v; break;
    case K::VirtualPowerPlant:   if (ok(s.vpps.size()))                 s.vpps[i].in_service = v; break;
    case K::MobileStorage:       if (ok(s.mobile_storage.size()))       s.mobile_storage[i].in_service = v; break;
    case K::Microgrid:           if (ok(s.microgrids.size()))           s.microgrids[i].in_service = v; break;
    case K::EnergyRouter:        if (ok(s.energy_routers.size()))       s.energy_routers[i].in_service = v; break;
    default: break;
  }
}

// Multiply the primary flow/power rating of a target by `factor`.
void derate(HybridPowerSystem& s, ReliabilityComponentKind k, int i, double factor) {
  using K = ReliabilityComponentKind;
  auto ok = [&](size_t n) { return i >= 0 && (size_t)i < n; };
  switch (k) {
    case K::ACBranch: if (ok(s.ac.branches.size())) {
        s.ac.branches[i].rate_a_mva *= factor;
        s.ac.branches[i].rate_b_mva *= factor;
        s.ac.branches[i].rate_c_mva *= factor;
      } break;
    case K::ACGenerator: if (ok(s.ac.generators.size())) s.ac.generators[i].pmax_mw *= factor; break;
    case K::VSCConverter: if (ok(s.vsc_converters.size())) {
        // Bidirectional: derate BOTH transfer directions (pmax = DC->AC,
        // pmin = AC->DC) so a converter feeding a DC island is actually limited.
        s.vsc_converters[i].pmax_mw *= factor;
        s.vsc_converters[i].pmin_mw *= factor;
        s.vsc_converters[i].p_rated_mw *= factor;
      } break;
    case K::DCDCConverter: if (ok(s.dc.dcdc_converters.size())) {
        s.dc.dcdc_converters[i].pmax_mw *= factor;
        s.dc.dcdc_converters[i].pmin_mw *= factor;
      } break;
    case K::DCBranch: if (ok(s.dc.branches.size())) {
        s.dc.branches[i].rate_a_mva *= factor;
        s.dc.branches[i].s_max_mva *= factor;
      } break;
    case K::ACStorage: if (ok(s.ac.storage.size())) s.ac.storage[i].pmax_mw *= factor; break;
    case K::DCStorage: if (ok(s.dc.storage.size())) s.dc.storage[i].pmax_mw *= factor; break;
    default: break;  // derating not represented for this kind
  }
}

// Remove controllability / grid-forming of a converter or source (fix setpoint).
void remove_control(HybridPowerSystem& s, ReliabilityComponentKind k, int i) {
  using K = ReliabilityComponentKind;
  auto ok = [&](size_t n) { return i >= 0 && (size_t)i < n; };
  switch (k) {
    case K::VSCConverter:      if (ok(s.vsc_converters.size()))       s.vsc_converters[i].controllable = false; break;
    case K::DCDCConverter:     if (ok(s.dc.dcdc_converters.size()))   s.dc.dcdc_converters[i].controllable = false; break;
    case K::ACStaticGenerator: if (ok(s.ac.static_generators.size())) s.ac.static_generators[i].controllable = false; break;
    case K::DCStaticGenerator: if (ok(s.dc.dc_static_generators.size())) s.dc.dc_static_generators[i].controllable = false; break;
    case K::ACStorage:         if (ok(s.ac.storage.size()))           s.ac.storage[i].controllable = false; break;
    case K::DCStorage:         if (ok(s.dc.storage.size()))           s.dc.storage[i].controllable = false; break;
    default: break;
  }
}

// Breaker / switch fail-to-trip or fail-to-open: the backup protection at the
// upstream bus clears a larger zone.  Conservative radial approximation: remove
// the failed device and de-energize its upstream bus (bus_from) by opening every
// in-service element incident to it.  In a radial feeder this isolates the bus
// and its downstream dependents (the backup zone); meshed buses with an
// alternate feed survive because only bus_from's own incident edges are opened.
void expand_protection_zone(HybridPowerSystem& s, ReliabilityComponentKind k, int i) {
  using K = ReliabilityComponentKind;
  int bus_from = -1;
  bool dc = false;
  if (k == K::ACCircuitBreaker && i >= 0 && (size_t)i < s.ac.circuit_breakers.size()) {
    s.ac.circuit_breakers[i].in_service = false;
    bus_from = s.ac.circuit_breakers[i].bus_from;
  } else if (k == K::DCCircuitBreaker && i >= 0 &&
             (size_t)i < s.dc.dc_circuit_breakers.size()) {
    s.dc.dc_circuit_breakers[i].in_service = false;
    bus_from = s.dc.dc_circuit_breakers[i].bus_from;
    dc = true;
  } else if (k == K::ACSwitch && i >= 0 && (size_t)i < s.ac.switches.size()) {
    // Fail-to-open / stuck-closed sectionalizing switch: cannot isolate, so the
    // upstream backup de-energizes its bus.
    bus_from = s.ac.switches[i].bus_from;
  } else {
    return;
  }
  if (bus_from < 0) return;
  if (!dc) {
    for (auto& br : s.ac.branches)
      if (br.from_bus == bus_from || br.to_bus == bus_from) br.in_service = false;
    for (auto& t : s.ac.transformers_2w)
      if (t.hv_bus == bus_from || t.lv_bus == bus_from) t.in_service = false;
    for (auto& cb : s.ac.circuit_breakers)
      if (cb.bus_from == bus_from || cb.bus_to == bus_from) cb.in_service = false;
    for (auto& sw : s.ac.switches)
      if (sw.bus_from == bus_from || sw.bus_to == bus_from) sw.in_service = false;
  } else {
    for (auto& br : s.dc.branches)
      if (br.from_bus == bus_from || br.to_bus == bus_from) br.in_service = false;
    for (auto& cb : s.dc.dc_circuit_breakers)
      if (cb.bus_from == bus_from || cb.bus_to == bus_from) cb.in_service = false;
  }
}

}  // namespace

HybridPowerSystem apply_consequence_patch(const HybridPowerSystem& sys,
                                          const ConsequencePatch& patch) {
  HybridPowerSystem s = sys;  // copy
  for (const auto& m : patch.mutations) {
    switch (m.kind) {
      case MutationKind::ForceOutOfService:
        set_in_service(s, m.target_kind, m.target_index, false);
        break;
      case MutationKind::ForceLoadShed:
        // Interrupt the load point: remove it from served demand so the OPF/LP
        // does not supply it; the FMEA engine adds its MW to the shed result.
        set_in_service(s, m.target_kind, m.target_index, false);
        break;
      case MutationKind::ProtectionZoneExpansion:
        expand_protection_zone(s, m.target_kind, m.target_index);
        break;
      case MutationKind::CapacityDerate:
        derate(s, m.target_kind, m.target_index, m.factor);
        break;
      case MutationKind::RemoveControllability:
      case MutationKind::RemoveGridForming:
        remove_control(s, m.target_kind, m.target_index);
        break;
      case MutationKind::ForceClosedNoIsolation:
      case MutationKind::RestorationActionForbidden:
      case MutationKind::ObservationDegraded:
        // Restoration/observation-only: no steady-state mutation applied.
        break;
    }
  }
  return s;
}

ConsequencePatch compose_consequence_patches(
    const std::vector<ConsequencePatch>& patches) {
  ConsequencePatch composed;
  // Track a hard outage per target so it dominates derating/control on the
  // same element; detect forced-open vs forced-closed conflicts.
  for (const auto& p : patches) {
    for (const auto& w : p.warnings) composed.warnings.push_back(w);
    for (const auto& id : p.affected_canonical_ids)
      composed.affected_canonical_ids.push_back(id);
    for (const auto& m : p.mutations) {
      bool conflict = false;
      bool dominated = false;
      for (auto& existing : composed.mutations) {
        const bool same_target = existing.target_kind == m.target_kind &&
                                 existing.target_index == m.target_index;
        if (!same_target) continue;
        // Hard outage dominates derating / control on the same element.
        if (existing.kind == MutationKind::ForceOutOfService &&
            (m.kind == MutationKind::CapacityDerate ||
             m.kind == MutationKind::RemoveControllability ||
             m.kind == MutationKind::RemoveGridForming)) {
          dominated = true; break;
        }
        if (m.kind == MutationKind::ForceOutOfService &&
            (existing.kind == MutationKind::CapacityDerate ||
             existing.kind == MutationKind::RemoveControllability ||
             existing.kind == MutationKind::RemoveGridForming)) {
          existing = m; dominated = true; break;  // outage replaces softer mutation
        }
        // Forced-open + forced-closed on the same element is a hard conflict.
        if (existing.kind == MutationKind::ForceOutOfService &&
            m.kind == MutationKind::ForceClosedNoIsolation) {
          conflict = true; break;
        }
        // Multiple deratings compose to the most restrictive capacity.
        if (existing.kind == MutationKind::CapacityDerate &&
            m.kind == MutationKind::CapacityDerate) {
          existing.factor = std::min(existing.factor, m.factor);
          dominated = true; break;
        }
      }
      if (conflict) {
        composed.warnings.push_back(
            "hard conflict: forced-open and forced-closed on " +
            to_string(m.target_kind) + ":" + std::to_string(m.target_index));
        continue;
      }
      if (!dominated) composed.mutations.push_back(m);
    }
  }
  return composed;
}

// ═══════════════════════════════════════════════════════════════════════
// Deterministic failure-mode FMEA engine
// ═══════════════════════════════════════════════════════════════════════
namespace {
// Resolve a ForceLoadShed mutation to the interrupted load's MW, bus id, and
// domain.  The load was removed from the OPF/LP demand by apply_consequence_patch,
// so its own MW must be added back as shed (the engine cannot express "force
// this specific load shed").
void forced_load_point(const HybridPowerSystem& sys, const ConsequenceMutation& mut,
                       double& mw, int& bus_id, bool& is_dc) {
  using K = ReliabilityComponentKind;
  const int i = mut.target_index;
  mw = 0.0; bus_id = -1; is_dc = false;
  if (mut.target_kind == K::ACLoad && i >= 0 && (size_t)i < sys.ac.loads.size()) {
    const auto& ld = sys.ac.loads[i];
    mw = std::max(0.0, ld.p_mw * ld.scaling); bus_id = ld.bus; is_dc = false;
  } else if (mut.target_kind == K::DCLoad && i >= 0 && (size_t)i < sys.dc.loads.size()) {
    const auto& ld = sys.dc.loads[i];
    mw = std::max(0.0, ld.p_mw * ld.scaling); bus_id = ld.bus; is_dc = true;
  } else if (mut.target_kind == K::AsymmetricLoad && i >= 0 &&
             (size_t)i < sys.ac.asymmetric_loads.size()) {
    const auto& al = sys.ac.asymmetric_loads[i];
    mw = std::max(0.0, (al.pa_mw + al.pb_mw + al.pc_mw) * al.scaling);
    bus_id = al.bus; is_dc = false;
  }
}

int resolve_failure_mode_worker_count(int requested_threads, int work_items) {
  return util::resolve_worker_count(requested_threads, work_items);
}
}  // namespace

FailureModeFMEAResult run_failure_mode_fmea(
    const HybridPowerSystem& sys,
    const FailureModeFMEAOptions& options) {
  FailureModeFMEAResult result;

  const bool hybrid =
      !sys.dc.buses.empty() || !sys.dc.branches.empty() || !sys.dc.loads.empty() ||
      !sys.vsc_converters.empty() || !sys.dc.dcdc_converters.empty() ||
      !sys.dc.storage.empty() || !sys.dc.pv_arrays.empty() ||
      !sys.dc.static_generators.empty() || !sys.dc.dc_static_generators.empty();
  result.model_scope = std::string("failure-mode-fmea/") +
                       (hybrid ? "hybrid-acdc-network-lp" : "ac-only-dcopf");
  result.model_limitations =
      "Deterministic failure-mode enumeration. Each enabled mode is mapped to a "
      "consequence patch and evaluated with the steady-state AC/DC shed engine. "
      "Active switching/protection and measurement/communication-only modes are "
      "reported as unsupported by this steady-state engine (no shed effect) and "
      "require the restoration/reconfiguration engine.";

  const size_t nb = sys.ac.buses.size() + (hybrid ? sys.dc.buses.size() : 0U);
  result.nodal_eens_mwh_yr.assign(nb, 0.0);
  std::vector<double> nodal_cif(nb, 0.0);
  std::vector<double> nodal_cid(nb, 0.0);

  FMEAOptions fmea_opts;
  fmea_opts.load_scale_factor = options.load_scale_factor;
  fmea_opts.verbose = false;

  // Build the failure-mode catalog and update support flags via patches.
  auto catalog = build_failure_mode_catalog(sys, options.catalog, options.data_policy);

  const double eps = options.curtail_threshold_mw;

  // Bus id -> nodal index maps for ForceLoadShed accumulation ([AC | DC] layout).
  std::unordered_map<int, int> ac_bus_pos, dc_bus_pos;
  ac_bus_pos.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i)
    ac_bus_pos[sys.ac.buses[i].index] = (int)i;
  if (hybrid)
    for (size_t i = 0; i < sys.dc.buses.size(); ++i)
      dc_bus_pos[sys.dc.buses[i].index] = (int)i;

  // The deterministic FMEA engine expands protection zones for breaker
  // fail-to-trip, so protection modelling is available to the consequence map.
  ConsequenceModelCapabilities caps = options.capabilities;
  caps.supports_protection_modeling = true;

  // Evaluate a consequence patch: apply it to a copy, run the shed engine, and
  // fold in any ForceLoadShed load-point interruptions (the patch removes those
  // loads from demand, so their MW is added back here as shed).  Shared by the
  // single-mode pass and the optional co-failure pass.
  auto evaluate_patch = [&](const ConsequencePatch& patch,
                            std::vector<double>& nodal_out) -> double {
    HybridPowerSystem sys_m = apply_consequence_patch(sys, patch);
    NetworkShedResult shed = evaluate_failed_network_state(sys_m, fmea_opts);
    if (shed.nodal_shed_mw.size() < nb) shed.nodal_shed_mw.resize(nb, 0.0);
    for (const auto& mut : patch.mutations) {
      if (mut.kind != MutationKind::ForceLoadShed) continue;
      double mw = 0.0; int bus_id = -1; bool is_dc = false;
      forced_load_point(sys, mut, mw, bus_id, is_dc);
      if (mw <= 0.0) continue;
      mw *= options.load_scale_factor;
      shed.total_shed_mw += mw;
      int nidx = -1;
      if (is_dc) {
        auto it = dc_bus_pos.find(bus_id);
        if (it != dc_bus_pos.end()) nidx = (int)sys.ac.buses.size() + it->second;
      } else {
        auto it = ac_bus_pos.find(bus_id);
        if (it != ac_bus_pos.end()) nidx = it->second;
      }
      if (nidx >= 0 && (size_t)nidx < shed.nodal_shed_mw.size())
        shed.nodal_shed_mw[nidx] += mw;
    }
    nodal_out = shed.nodal_shed_mw;
    return shed.total_shed_mw;
  };

  // Cache of supported single modes (patch + overlap weights) for the optional
  // second-order co-failure pass.
  struct SupportedMode {
    ConsequencePatch patch;
    std::string component_id;
    double frequency{0.0};
    double duration{0.0};
    double unavailability{0.0};
  };
  std::vector<SupportedMode> supported_modes;

  struct SingleModeWorkResult {
    FailureModeContingency contingency;
    std::vector<double> nodal_eens;
    std::vector<double> nodal_cif;
    std::vector<double> nodal_cid;
    SupportedMode supported_mode;
    bool has_supported_mode{false};
    bool supported_by_selected_model{true};
  };

  auto evaluate_single_mode = [&](const FailureModeCatalogEntry& entry) {
    SingleModeWorkResult work;
    FailureModeContingency c;
    c.ref = entry.mode.ref;
    c.data_source = entry.mode.params.data_source;

    // Event duration = isolation + switching + repair/recovery.
    const double dur = entry.mode.isolation_hr + entry.mode.switching_hr +
                       std::max(entry.mode.params.repair_hr, entry.mode.repair_hr);
    c.duration_hr = dur;
    c.frequency_per_year = entry.mode.params.lambda_per_year;  // active already = nu*p

    if (!entry.enabled) {
      c.supported = false;
      c.unsupported_reason = entry.disabled_reason.empty()
                                 ? "mode disabled" : entry.disabled_reason;
      work.contingency = std::move(c);
      return work;
    }

    // Map the mode to a consequence patch under the engine capabilities.
    ConsequencePatch patch =
        build_consequence_patch(sys, entry.mode, caps);
    work.supported_by_selected_model = patch.representable_by_selected_model;
    c.supported = patch.representable_by_selected_model;
    c.unsupported_reason = patch.unsupported_reason;

    if (!patch.representable_by_selected_model) {
      // Reported but contributes no steady-state shed.
      work.contingency = std::move(c);
      return work;
    }

    // Apply the patch to a copy, evaluate the failed network state, and fold in
    // any load-point interruptions (shared helper).
    std::vector<double> nodal_shed;
    const double total_shed = evaluate_patch(patch, nodal_shed);

    c.total_shed_mw = total_shed;
    c.causes_loss = total_shed > eps;
    c.nodal_shed_mw = nodal_shed;
    c.eens_contribution = c.frequency_per_year * dur * total_shed;
    c.lole_contribution = c.causes_loss ? c.frequency_per_year * dur : 0.0;

    work.nodal_eens.assign(nb, 0.0);
    work.nodal_cif.assign(nb, 0.0);
    work.nodal_cid.assign(nb, 0.0);
    for (size_t b = 0; b < nb && b < nodal_shed.size(); ++b) {
      const double sbed = nodal_shed[b];
      work.nodal_eens[b] += c.frequency_per_year * dur * sbed;
      if (sbed > eps) {
        work.nodal_cif[b] += c.frequency_per_year;
        work.nodal_cid[b] += c.frequency_per_year * dur;
      }
    }

    // Cache this supported mode for the optional co-failure pass.
    if (options.max_order >= 2 && c.frequency_per_year > 0.0 && dur > 0.0) {
      work.supported_mode.patch = patch;
      work.supported_mode.component_id = entry.mode.ref.component.stable_id;
      work.supported_mode.frequency = c.frequency_per_year;
      work.supported_mode.duration = dur;
      work.supported_mode.unavailability =
          std::min(1.0, c.frequency_per_year * dur / 8760.0);
      work.has_supported_mode = true;
    }

    work.contingency = std::move(c);
    return work;
  };

  std::vector<SingleModeWorkResult> single_results(catalog.size());
  const int single_workers = options.enable_parallel
      ? resolve_failure_mode_worker_count(options.parallel_threads,
                                          static_cast<int>(catalog.size()))
      : 1;
  result.parallel_workers = single_workers;
  result.parallel_effective = options.enable_parallel && single_workers > 1 &&
                              catalog.size() > 1U;
  result.parallel_mode = result.parallel_effective
      ? "parallel-failure-mode-fmea"
      : (options.enable_parallel ? "serial/insufficient-work" : "serial/disabled");
  result.parallel_execution = util::make_parallel_execution_info(
      options.enable_parallel, options.parallel_threads,
      static_cast<int>(catalog.size()), "parallel-failure-mode-fmea",
      "failure-modes");
  result.parallel_execution.effective = result.parallel_effective;
  result.parallel_execution.resolved_workers = single_workers;
  result.parallel_execution.mode = result.parallel_mode;
  if (!result.parallel_effective && options.enable_parallel) {
    result.parallel_execution.guard_reason =
        util::insufficient_work_reason(result.parallel_execution);
  }

  if (result.parallel_effective) {
    util::ThreadPool pool(single_workers);
    pool.parallel_for_dynamic(
        catalog.size(),
        [&](size_t i) {
          single_results[i] = evaluate_single_mode(catalog[i]);
        },
        single_workers);
    result.parallel_execution.actual_parallel_evaluations =
        static_cast<long long>(catalog.size());
  } else {
    for (size_t i = 0; i < catalog.size(); ++i) {
      single_results[i] = evaluate_single_mode(catalog[i]);
    }
    result.parallel_execution.serial_evaluations =
        static_cast<long long>(catalog.size());
  }

  for (size_t i = 0; i < single_results.size(); ++i) {
    auto& work = single_results[i];
    catalog[i].supported_by_selected_consequence_model =
        work.supported_by_selected_model;
    catalog[i].unsupported_reason = work.contingency.unsupported_reason;

    result.eens_mwh_yr += work.contingency.eens_contribution;
    result.lole_hr_yr += work.contingency.lole_contribution;
    if (work.contingency.causes_loss) {
      result.lolf_occ_yr += work.contingency.frequency_per_year;
    }
    for (size_t b = 0; b < nb; ++b) {
      if (b < work.nodal_eens.size()) result.nodal_eens_mwh_yr[b] += work.nodal_eens[b];
      if (b < work.nodal_cif.size()) nodal_cif[b] += work.nodal_cif[b];
      if (b < work.nodal_cid.size()) nodal_cid[b] += work.nodal_cid[b];
    }
    if (work.has_supported_mode) supported_modes.push_back(std::move(work.supported_mode));
    result.contingencies.push_back(std::move(work.contingency));
  }

  // ── Optional second-order (co-failure) enumeration ──────────────────────
  // Compose pairs of supported single modes on DISTINCT components, evaluate the
  // joint network state, and accumulate the overlap contribution using the
  // standard independent second-order approximation: the probability that both
  // modes are simultaneously down is U_i*U_j, so the joint state contributes
  // EENS += U_i*U_j*8760*S_ij (MWh/yr) and LOLE += U_i*U_j*8760 (hr/yr).  These
  // are distinct states from the single-mode terms (which approximate "i down,
  // others up"), so the sum is the standard first+second-order state expansion.
  if (options.max_order >= 2 && supported_modes.size() > 1) {
    struct PairWork {
      size_t a{0};
      size_t b{0};
      double joint_unavailability{0.0};
      ConsequencePatch composed;
    };
    struct PairWorkResult {
      bool evaluated{false};
      bool causes_loss{false};
      FailureModeCoContingency co;
      std::vector<double> nodal_eens;
      std::vector<double> nodal_cif;
      std::vector<double> nodal_cid;
    };

    const int max_pairs = std::max(0, options.max_pairs_evaluated);
    std::vector<PairWork> pair_work;
    pair_work.reserve(static_cast<size_t>(std::min<int>(max_pairs, 4096)));
    for (size_t i = 0; i < supported_modes.size() &&
                       static_cast<int>(pair_work.size()) < max_pairs; ++i) {
      for (size_t k = i + 1; k < supported_modes.size(); ++k) {
        if (static_cast<int>(pair_work.size()) >= max_pairs) break;
        const auto& a = supported_modes[i];
        const auto& b = supported_modes[k];
        // Modes of the same component are mutually-exclusive alternatives, not
        // independent co-failures.
        if (a.component_id == b.component_id) continue;
        const double u_ij = a.unavailability * b.unavailability;
        if (u_ij < options.min_pair_unavailability) continue;

        ConsequencePatch composed =
            compose_consequence_patches({a.patch, b.patch});
        bool conflict = false;
        for (const auto& w : composed.warnings)
          if (w.find("hard conflict") != std::string::npos) { conflict = true; break; }
        if (conflict) continue;

        pair_work.push_back({i, k, u_ij, std::move(composed)});
      }
    }

    result.n_pairs_evaluated = static_cast<int>(pair_work.size());

    auto evaluate_pair = [&](const PairWork& pw) {
      PairWorkResult work;
      work.evaluated = true;
      const auto& a = supported_modes[pw.a];
      const auto& b = supported_modes[pw.b];

      std::vector<double> nodal_shed;
      const double shed_ij = evaluate_patch(pw.composed, nodal_shed);
      if (shed_ij <= eps) return work;

      const double eens_ij = pw.joint_unavailability * 8760.0 * shed_ij;
      const double lole_ij = pw.joint_unavailability * 8760.0;
      const double freq_ij =
          a.frequency * b.frequency * (a.duration + b.duration) / 8760.0;
      const double dur_ij = (a.duration + b.duration) > 0.0
                                ? a.duration * b.duration / (a.duration + b.duration)
                                : 0.0;

      work.nodal_eens.assign(nb, 0.0);
      work.nodal_cif.assign(nb, 0.0);
      work.nodal_cid.assign(nb, 0.0);
      for (size_t bb = 0; bb < nb && bb < nodal_shed.size(); ++bb) {
        const double sbed = nodal_shed[bb];
        work.nodal_eens[bb] += pw.joint_unavailability * 8760.0 * sbed;
        if (sbed > eps) {
          work.nodal_cif[bb] += freq_ij;
          work.nodal_cid[bb] += freq_ij * dur_ij;
        }
      }

      work.co.mode_a = a.patch.mode;
      work.co.mode_b = b.patch.mode;
      work.co.joint_frequency_per_year = freq_ij;
      work.co.joint_unavailability = pw.joint_unavailability;
      work.co.duration_hr = dur_ij;
      work.co.total_shed_mw = shed_ij;
      work.co.eens_contribution = eens_ij;
      work.co.lole_contribution = lole_ij;
      work.co.causes_loss = true;
      work.causes_loss = true;
      return work;
    };

    std::vector<PairWorkResult> pair_results(pair_work.size());
    const int pair_workers = options.enable_parallel
        ? resolve_failure_mode_worker_count(options.parallel_threads,
                                            static_cast<int>(pair_work.size()))
        : 1;
    if (options.enable_parallel && pair_workers > 1 && pair_work.size() > 1U) {
      result.parallel_effective = true;
      result.parallel_workers = std::max(result.parallel_workers, pair_workers);
      result.parallel_mode = "parallel-failure-mode-fmea+n2";
      result.parallel_execution.effective = true;
      result.parallel_execution.resolved_workers =
          std::max(result.parallel_execution.resolved_workers, pair_workers);
      result.parallel_execution.mode = result.parallel_mode;
      result.parallel_execution.guard_reason.clear();
      result.parallel_execution.work_items += static_cast<int>(pair_work.size());
      util::ThreadPool pool(pair_workers);
      pool.parallel_for_dynamic(
          pair_work.size(),
          [&](size_t idx) {
            pair_results[idx] = evaluate_pair(pair_work[idx]);
          },
          pair_workers);
      result.parallel_execution.actual_parallel_evaluations +=
          static_cast<long long>(pair_work.size());
    } else {
      for (size_t idx = 0; idx < pair_work.size(); ++idx) {
        pair_results[idx] = evaluate_pair(pair_work[idx]);
      }
      result.parallel_execution.work_items += static_cast<int>(pair_work.size());
      result.parallel_execution.serial_evaluations +=
          static_cast<long long>(pair_work.size());
    }

    for (auto& work : pair_results) {
      if (!work.evaluated || !work.causes_loss) continue;
      result.eens_mwh_yr += work.co.eens_contribution;
      result.lole_hr_yr += work.co.lole_contribution;
      result.lolf_occ_yr += work.co.joint_frequency_per_year;
      for (size_t bb = 0; bb < nb; ++bb) {
        if (bb < work.nodal_eens.size()) result.nodal_eens_mwh_yr[bb] += work.nodal_eens[bb];
        if (bb < work.nodal_cif.size()) nodal_cif[bb] += work.nodal_cif[bb];
        if (bb < work.nodal_cid.size()) nodal_cid[bb] += work.nodal_cid[bb];
      }
      result.co_contingencies.push_back(std::move(work.co));
    }
    std::sort(result.co_contingencies.begin(), result.co_contingencies.end(),
              [](const FailureModeCoContingency& x, const FailureModeCoContingency& y) {
                return x.eens_contribution > y.eens_contribution;
              });
  }

  result.edns_mw = result.eens_mwh_yr / 8760.0;

  std::sort(result.contingencies.begin(), result.contingencies.end(),
            [](const FailureModeContingency& a, const FailureModeContingency& b) {
              return a.eens_contribution > b.eens_contribution;
            });

  result.distribution_idx = compute_distribution_indices(sys, nodal_cif, nodal_cid);
  result.coverage = summarize_failure_mode_coverage(catalog);
  result.data_quality = summarize_reliability_data_quality(sys, options.data_policy);
  return result;
}

}  // namespace hacdcpf::analysis

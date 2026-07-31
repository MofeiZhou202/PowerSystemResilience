#include "hacdcpf/resilience/resilience_dynamic_certification.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::analysis {
namespace {

using ComponentKey = std::string;
using BusServiceKey = std::string;

constexpr double kServiceTolerance = 1.0e-9;

BusServiceKey bus_service_key(const std::string& domain, int bus) {
  return domain + ":" + std::to_string(bus);
}

struct BusServiceState {
  std::string domain;
  int bus{0};
  double ratio{1.0};
};

struct CanonicalBusIds {
  std::unordered_map<int, int> ac;
  std::unordered_map<int, int> dc;

  int resolve(const std::string& domain, int authored_bus) const {
    const auto& mapping = domain == "DC" ? dc : ac;
    const auto it = mapping.find(authored_bus);
    return it == mapping.end() ? authored_bus : it->second;
  }
};

CanonicalBusIds canonical_bus_ids(const HybridPowerSystem& authored,
                                  const HybridPowerSystem& canonical) {
  CanonicalBusIds out;
  if (canonical.bus_merge_map) {
    out.ac.reserve(canonical.bus_merge_map->ext_to_int.size());
    for (const auto& [external, internal] :
         canonical.bus_merge_map->ext_to_int) {
      out.ac[external] = internal + 1;
    }
  } else {
    const std::size_t count =
        std::min(authored.ac.buses.size(), canonical.ac.buses.size());
    out.ac.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      out.ac[authored.ac.buses[i].index] = canonical.ac.buses[i].index;
    }
  }
  const std::size_t dc_count =
      std::min(authored.dc.buses.size(), canonical.dc.buses.size());
  out.dc.reserve(dc_count);
  for (std::size_t i = 0; i < dc_count; ++i) {
    out.dc[authored.dc.buses[i].index] = canonical.dc.buses[i].index;
  }
  return out;
}

std::unordered_map<BusServiceKey, BusServiceState> service_map(
    const DistributionResilienceStepResult& step) {
  std::unordered_map<BusServiceKey, BusServiceState> out;
  const std::size_t count = std::min(
      {step.bus_supply_kind.size(), step.bus_supply_index.size(),
       step.bus_supply_demand_mw.size(), step.bus_supply_served_mw.size()});
  out.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double demand = std::max(0.0, step.bus_supply_demand_mw[i]);
    if (demand <= kServiceTolerance) continue;
    const double served = std::clamp(step.bus_supply_served_mw[i], 0.0, demand);
    BusServiceState state;
    state.domain = step.bus_supply_kind[i];
    state.bus = step.bus_supply_index[i];
    state.ratio = served / demand;
    out[bus_service_key(state.domain, state.bus)] = state;
  }
  return out;
}

void apply_bus_service_ratio(HybridPowerSystem& system,
                             const BusServiceState& service,
                             const CanonicalBusIds& bus_ids) {
  const int canonical_bus = bus_ids.resolve(service.domain, service.bus);
  if (service.domain == "AC") {
    for (auto& bus : system.ac.buses) {
      if (bus.index != canonical_bus) continue;
      bus.pd_mw *= service.ratio;
      bus.qd_mvar *= service.ratio;
    }
    for (auto& load : system.ac.loads) {
      if (load.bus == canonical_bus) load.scaling *= service.ratio;
    }
    for (auto& load : system.ac.asymmetric_loads) {
      if (load.bus == canonical_bus) load.scaling *= service.ratio;
    }
  } else if (service.domain == "DC") {
    for (auto& bus : system.dc.buses) {
      if (bus.index == canonical_bus) {
        bus.pd_mw *= service.ratio;
      }
    }
    for (auto& load : system.dc.loads) {
      if (load.bus == canonical_bus) load.scaling *= service.ratio;
    }
  }
}

void apply_step_service(HybridPowerSystem& system,
                        const DistributionResilienceStepResult& step,
                        const CanonicalBusIds& bus_ids) {
  for (const auto& [key, service] : service_map(step)) {
    (void)key;
    apply_bus_service_ratio(system, service, bus_ids);
  }
}

void append_load_service_events(
    const DistributionResilienceStepResult* previous,
    const DistributionResilienceStepResult& target,
    const DistributionResilienceDAECertificationOptions& options,
    const CanonicalBusIds& bus_ids,
    CertifiedRestorationAction& action,
    std::vector<std::string>& unsupported) {
  if (!options.replay_mip_load_service) return;
  const auto target_service = service_map(target);
  const auto previous_service =
      previous == nullptr
          ? std::unordered_map<BusServiceKey, BusServiceState>{}
          : service_map(*previous);
  for (const auto& [key, target_state] : target_service) {
    double previous_ratio = 1.0;
    if (previous != nullptr) {
      const auto it = previous_service.find(key);
      if (it == previous_service.end()) {
        unsupported.push_back("bus service state missing from previous MIP step: " +
                              key);
        continue;
      }
      previous_ratio = it->second.ratio;
    }
    if (std::abs(previous_ratio - target_state.ratio) <= kServiceTolerance) {
      continue;
    }

    double transition_scale = target_state.ratio;
    if (previous_ratio > kServiceTolerance) {
      transition_scale = target_state.ratio / previous_ratio;
    } else if (target_state.ratio > kServiceTolerance) {
      unsupported.push_back(
          "bus service pickup from zero has no exact DAE initial-load replay: " +
          key);
    }

    dynamics::DynamicEvent event;
    event.time_s = options.switching_time_s;
    event.type = target_state.domain == "DC"
                     ? dynamics::DynamicEventType::DCLoadScale
                     : dynamics::DynamicEventType::ACLoadScale;
    event.bus = target_state.bus;
    event.value = std::max(0.0, transition_scale);
    event.params["scale"] = event.value;
    event.params["canonical_bus"] =
        bus_ids.resolve(target_state.domain, target_state.bus);
    event.component_type = target_state.domain;
    event.label = "mip_bus_service_" + target_state.domain + "_" +
                  std::to_string(target_state.bus) + "_to_" +
                  std::to_string(target_state.ratio);
    action.dynamic_events.push_back(std::move(event));
  }
}

ComponentKey component_key(const ResilienceComponentStateStep& state) {
  return state.component_type + ":" + state.domain + ":" +
         std::to_string(state.canonical_component_index) + ":" +
         std::to_string(state.pair_number);
}

bool effective_closed(const ResilienceComponentStateStep& state) {
  return state.available && state.closed;
}

HybridPowerSystem canonical_restoration_system(const HybridPowerSystem& system) {
  if (system.projection_certificate.has_value()) return system;
  auto input = system;
  const auto mobile_storage = input.mobile_storage;
  input.mobile_storage.clear();
  ProjectionOptions options;
  options.mode = ProjectionMode::ExactIdeal;
  options.strip_dead_islands = false;
  options.preserve_switch_branches = true;
  auto canonical = project_to_canonical_models(std::move(input), options);
  canonical.mobile_storage = mobile_storage;
  return canonical;
}

void set_initial_component_state(HybridPowerSystem& system,
                                 const ResilienceComponentStateStep& state) {
  const bool closed = effective_closed(state);
  if (state.component_type == "ac_branch" ||
      state.component_type == "transformer_2w" ||
      state.component_type == "transformer_3w" ||
      state.component_type == "switch" ||
      state.component_type == "circuit_breaker") {
    for (auto& branch : system.ac.branches) {
      if (branch.index == state.canonical_component_index) {
        branch.in_service = closed;
      }
    }
  } else if (state.component_type == "dc_branch") {
    for (auto& branch : system.dc.branches) {
      if (branch.index == state.canonical_component_index) {
        branch.in_service = closed;
      }
    }
  } else if (state.component_type == "vsc_converter") {
    for (auto& converter : system.vsc_converters) {
      if (converter.index == state.canonical_component_index) {
        converter.in_service = closed;
      }
    }
  } else if (state.component_type == "lcc_converter") {
    for (auto& converter : system.lcc_converters) {
      if (converter.index == state.canonical_component_index) {
        converter.in_service = closed;
      }
    }
  } else if (state.component_type == "dcdc_converter") {
    for (auto& converter : system.dc.dcdc_converters) {
      if (converter.index == state.canonical_component_index) {
        converter.in_service = closed;
      }
    }
  } else if (state.component_type == "dc_circuit_breaker") {
    for (auto& breaker : system.dc.dc_circuit_breakers) {
      if (breaker.index == state.canonical_component_index) {
        breaker.closed = closed;
      }
    }
  }
}

bool authored_component_closed(const HybridPowerSystem& system,
                               const ResilienceComponentStateStep& state) {
  if (state.component_type == "ac_branch" ||
      state.component_type == "transformer_2w" ||
      state.component_type == "transformer_3w" ||
      state.component_type == "switch" ||
      state.component_type == "circuit_breaker") {
    const auto it = std::find_if(
        system.ac.branches.begin(), system.ac.branches.end(),
        [&](const ACBranch& branch) {
          return branch.index == state.canonical_component_index;
        });
    return it != system.ac.branches.end() && it->in_service;
  }
  if (state.component_type == "dc_branch") {
    const auto it = std::find_if(
        system.dc.branches.begin(), system.dc.branches.end(),
        [&](const DCBranch& branch) {
          return branch.index == state.canonical_component_index;
        });
    return it != system.dc.branches.end() && it->in_service;
  }
  if (state.component_type == "vsc_converter") {
    const auto it = std::find_if(
        system.vsc_converters.begin(), system.vsc_converters.end(),
        [&](const VSCConverter& converter) {
          return converter.index == state.canonical_component_index;
        });
    return it != system.vsc_converters.end() && it->in_service;
  }
  if (state.component_type == "lcc_converter") {
    const auto it = std::find_if(
        system.lcc_converters.begin(), system.lcc_converters.end(),
        [&](const LCCConverter& converter) {
          return converter.index == state.canonical_component_index;
        });
    return it != system.lcc_converters.end() && it->in_service;
  }
  if (state.component_type == "dcdc_converter") {
    const auto it = std::find_if(
        system.dc.dcdc_converters.begin(), system.dc.dcdc_converters.end(),
        [&](const DCDCConverter& converter) {
          return converter.index == state.canonical_component_index;
        });
    return it != system.dc.dcdc_converters.end() && it->in_service;
  }
  if (state.component_type == "dc_circuit_breaker") {
    const auto it = std::find_if(
        system.dc.dc_circuit_breakers.begin(),
        system.dc.dc_circuit_breakers.end(),
        [&](const DCCircuitBreaker& breaker) {
          return breaker.index == state.canonical_component_index;
        });
    return it != system.dc.dc_circuit_breakers.end() &&
           it->in_service && it->closed;
  }
  return false;
}

bool make_dynamic_event(const ResilienceComponentStateStep& previous,
                        const ResilienceComponentStateStep& target,
                        double event_time_s,
                        dynamics::DynamicEvent& event,
                        std::string& unsupported) {
  const bool was_closed = effective_closed(previous);
  const bool will_close = effective_closed(target);
  if (was_closed == will_close) return false;

  event.time_s = event_time_s;
  event.component_index = target.canonical_component_index;
  event.component_type = target.component_type;
  event.label = target.component_type + "_" +
                std::to_string(target.component_index) +
                (will_close ? "_close" : "_trip");

  if (target.component_type == "ac_branch" ||
      target.component_type == "transformer_2w" ||
      target.component_type == "transformer_3w" ||
      target.component_type == "switch" ||
      target.component_type == "circuit_breaker") {
    event.type = will_close ? dynamics::DynamicEventType::ACBranchClose
                            : dynamics::DynamicEventType::ACBranchTrip;
    return true;
  }
  if (target.component_type == "dc_branch") {
    event.type = will_close ? dynamics::DynamicEventType::DCBranchClose
                            : dynamics::DynamicEventType::DCBranchTrip;
    return true;
  }
  if (target.component_type == "vsc_converter" && !will_close) {
    event.type = dynamics::DynamicEventType::VSCTrip;
    return true;
  }
  if (target.component_type == "dcdc_converter" && !will_close) {
    event.type = dynamics::DynamicEventType::DCDCTrip;
    return true;
  }

  unsupported = target.component_type + " " +
                std::to_string(target.component_index) +
                (will_close ? " close" : " trip") +
                " has no exact dynamic event implementation";
  return false;
}

std::unordered_map<ComponentKey, const ResilienceComponentStateStep*>
state_map(const DistributionResilienceStepResult& step) {
  std::unordered_map<ComponentKey, const ResilienceComponentStateStep*> out;
  out.reserve(step.component_states.size());
  for (const auto& state : step.component_states) {
    out[component_key(state)] = &state;
  }
  return out;
}

CertifiedRestorationAction make_action(
    const HybridPowerSystem& canonical,
    const DistributionResilienceStepResult* previous,
    const DistributionResilienceStepResult& target,
    const DistributionResilienceDAECertificationOptions& options,
    const CanonicalBusIds& bus_ids,
    std::vector<std::string>& unsupported) {
  CertifiedRestorationAction action;
  action.id = "mip_transition_" +
              std::to_string(previous == nullptr ? -1 : previous->step_index) +
              "_to_" + std::to_string(target.step_index);
  action.description = "Strict restoration MIP topology transition replay";
  action.objective = target.served_mw;
  action.command_path_available = options.command_path_available;
  action.authorization_valid = options.authorization_valid;
  action.acknowledgement_available = options.acknowledgement_available;

  std::unordered_map<ComponentKey, const ResilienceComponentStateStep*> prior;
  if (previous != nullptr) prior = state_map(*previous);
  for (const auto& target_state : target.component_states) {
    ResilienceComponentStateStep authored = target_state;
    authored.available = true;
    authored.closed = authored_component_closed(canonical, target_state);
    const ResilienceComponentStateStep* previous_state = &authored;
    if (const auto it = prior.find(component_key(target_state)); it != prior.end()) {
      previous_state = it->second;
    } else if (previous != nullptr) {
      unsupported.push_back("component state missing from previous MIP step: " +
                            component_key(target_state));
      continue;
    }

    dynamics::DynamicEvent event;
    std::string reason;
    if (make_dynamic_event(*previous_state, target_state,
                           options.switching_time_s, event, reason)) {
      action.dynamic_events.push_back(std::move(event));
    } else if (!reason.empty()) {
      unsupported.push_back(std::move(reason));
    }
  }

  append_load_service_events(previous, target, options, bus_ids, action,
                             unsupported);

  for (const auto& mess : target.mess_states) {
    if (mess.dispatch_mw <= 1e-9) continue;
    action.requires_mess = true;
    action.mess_storage_index = mess.storage_index;
    action.mess_target_ac_bus = mess.bus;
    action.mess_dispatch_mw = mess.dispatch_mw;
    action.mess_available_energy_mwh = mess.energy_mwh;
    action.mess_connection_deadline_s = options.switching_time_s;
    action.mess_travel_time_s = std::max(0.0, mess.remaining_travel_hr * 3600.0);
    action.mess_required_energy_mwh =
        mess.dispatch_mw * std::max(0.0, options.switching_time_s) / 3600.0;
    const auto asset = std::find_if(
        canonical.mobile_storage.begin(), canonical.mobile_storage.end(),
        [&](const MobileStorage& storage) {
          return storage.index == mess.storage_index;
        });
    if (asset != canonical.mobile_storage.end()) {
      action.mess_grid_forming_capable = asset->grid_forming;
      action.requires_grid_forming_mess = asset->grid_forming;
    }
    break;
  }
  return action;
}

bool has_material_change(const DistributionResilienceDAETransition& transition) {
  return !transition.action.dynamic_events.empty() ||
         transition.action.requires_mess ||
         !transition.unsupported_transitions.empty();
}

}  // namespace

std::vector<DistributionResilienceDAETransition>
build_distribution_resilience_dae_transitions(
    const HybridPowerSystem& system,
    const DistributionResilienceResult& restoration,
    const DistributionResilienceDAECertificationOptions& options) {
  std::vector<DistributionResilienceDAETransition> transitions;
  if (!restoration.feasible || restoration.steps.empty() ||
      options.max_transitions <= 0) {
    return transitions;
  }

  const HybridPowerSystem canonical = canonical_restoration_system(system);
  const CanonicalBusIds bus_ids = canonical_bus_ids(system, canonical);
  const auto append_transition = [&](const DistributionResilienceStepResult* previous,
                                     const DistributionResilienceStepResult& target,
                                     bool force,
                                     auto& output) {
    DistributionResilienceDAETransition transition;
    transition.from_step = previous == nullptr ? -1 : previous->step_index;
    transition.to_step = target.step_index;
    transition.initial_system = canonical;
    if (previous != nullptr) {
      for (const auto& state : previous->component_states) {
        set_initial_component_state(transition.initial_system, state);
      }
      if (options.replay_mip_load_service) {
        apply_step_service(transition.initial_system, *previous, bus_ids);
      }
    }
    transition.action = make_action(canonical, previous, target, options,
                                    bus_ids,
                                    transition.unsupported_transitions);
    if (force || has_material_change(transition)) {
      output.push_back(std::move(transition));
    }
  };

  if (options.certify_initial_transition) {
    append_transition(nullptr, restoration.steps.front(), false, transitions);
  }
  for (std::size_t i = 1;
       i < restoration.steps.size() &&
       static_cast<int>(transitions.size()) < options.max_transitions;
       ++i) {
    append_transition(&restoration.steps[i - 1], restoration.steps[i], false,
                      transitions);
  }
  if (options.certify_final_state &&
      static_cast<int>(transitions.size()) < options.max_transitions) {
    const int final_step = restoration.steps.back().step_index;
    const bool final_already_present =
        !transitions.empty() && transitions.back().to_step == final_step;
    if (!final_already_present) {
      append_transition(&restoration.steps.back(), restoration.steps.back(),
                        true, transitions);
    }
  }
  if (static_cast<int>(transitions.size()) > options.max_transitions) {
    transitions.resize(static_cast<std::size_t>(options.max_transitions));
  }
  return transitions;
}

DistributionResilienceDAECertificateResult
certify_distribution_resilience_dynamics(
    const HybridPowerSystem& system,
    const DistributionResilienceResult& restoration,
    const dynamics::DynamicSolverOptions& dynamic_options,
    const MultiFidelityCertificateOptions& certificate_options,
    const DistributionResilienceDAECertificationOptions& options) {
  DistributionResilienceDAECertificateResult result;
  result.limitations = {
      "Certificate scope is the canonical topology, authored dynamic models, thresholds, and simulated horizon.",
      "Aggregated MIP generator, renewable, and fixed-storage dispatch is not replayed per device; the DAE uses authored operating setpoints.",
      "MIP service is replayed as aggregate domain-qualified bus load scaling; allocation among individual loads at the same bus is not recovered.",
      "A pickup from exactly zero prior bus service remains unresolved because the current DAE event model scales the initialized load base.",
      "Branch outage events are represented as trips/closures, not as an explicit short-circuit waveform unless separately authored."};
  if (!restoration.feasible) {
    result.status = "restoration MIP is infeasible; dynamic certification not run";
    return result;
  }

  auto transitions = build_distribution_resilience_dae_transitions(
      system, restoration, options);
  if (transitions.empty()) {
    result.status = "restoration result contains no certifiable states";
    return result;
  }
  result.attempted = true;
  bool all_safe = true;
  bool all_proven = true;
  for (auto& transition : transitions) {
    DistributionResilienceDAETransitionResult evaluated;
    evaluated.from_step = transition.from_step;
    evaluated.to_step = transition.to_step;
    evaluated.action = transition.action;
    evaluated.unsupported_transitions = transition.unsupported_transitions;

    auto oracle_options = dynamic_options;
    oracle_options.project_to_canonical = false;
    MultiFidelityCertificateEngine engine(
        std::move(transition.initial_system), oracle_options,
        certificate_options);
    evaluated.executability = engine.checkExecutability(evaluated.action);
    if (!evaluated.executability.executable) {
      evaluated.certificate.action_id = evaluated.action.id;
      evaluated.certificate.fidelity_level =
          std::clamp(options.fidelity_level, 1, 3);
      evaluated.certificate.label = CertificateLabel::Failed;
      evaluated.certificate.limitations =
          evaluated.executability.failed_constraints;
    } else if (options.require_supported_transitions &&
               !evaluated.unsupported_transitions.empty()) {
      evaluated.certificate.action_id = evaluated.action.id;
      evaluated.certificate.fidelity_level =
          std::clamp(options.fidelity_level, 1, 3);
      evaluated.certificate.label = CertificateLabel::Unresolved;
      evaluated.certificate.limitations =
          evaluated.unsupported_transitions;
    } else {
      evaluated.certificate = engine.evaluate(
          evaluated.action, std::clamp(options.fidelity_level, 1, 3));
      if (!evaluated.unsupported_transitions.empty()) {
        evaluated.certificate.limitations.insert(
            evaluated.certificate.limitations.end(),
            evaluated.unsupported_transitions.begin(),
            evaluated.unsupported_transitions.end());
        evaluated.certificate.proof_valid = false;
        if (evaluated.certificate.label == CertificateLabel::Safe) {
          evaluated.certificate.label = CertificateLabel::Unresolved;
        }
      }
    }

    const bool safe = evaluated.certificate.proof_valid &&
                      evaluated.certificate.label == CertificateLabel::Safe;
    all_safe = all_safe && safe;
    all_proven = all_proven && evaluated.certificate.proof_valid;
    if (!safe) {
      DistributionResilienceDAEFeedback feedback;
      feedback.to_step = evaluated.to_step;
      feedback.action_id = evaluated.action.id;
      for (const auto& event : evaluated.action.dynamic_events) {
        feedback.event_labels.push_back(event.label);
      }
      if (!evaluated.executability.executable) {
        feedback.feedback_type = "executability_cut_candidate";
        feedback.reason = "cyber, authorization, acknowledgement, or MESS executability failed";
        feedback.recommended_master_action =
            "exclude the action until the failed executability constraints are restored";
      } else if (!evaluated.unsupported_transitions.empty()) {
        feedback.feedback_type = "model_resolution_cut_candidate";
        feedback.reason = evaluated.unsupported_transitions.front();
        feedback.recommended_master_action =
            "hold the transition unresolved or replace it with an explicitly modeled black-start/load-pickup sequence";
      } else if (evaluated.certificate.simulation_success &&
                 evaluated.certificate.label == CertificateLabel::Unsafe) {
        feedback.feedback_type = "dynamic_no_good_cut_candidate";
        feedback.reason = "the represented transition violated a declared dynamic threshold";
        feedback.recommended_master_action =
            "exclude this exact transition combination or impose the returned dynamic margin before re-optimization";
      } else {
        feedback.feedback_type = "dynamic_model_resolution_cut_candidate";
        feedback.reason = evaluated.certificate.trajectory.message.empty()
                              ? "the DAE oracle did not produce a valid safety proof"
                              : evaluated.certificate.trajectory.message;
        feedback.recommended_master_action =
            "do not credit the transition; correct initialization/model support before deriving an operational no-good cut";
      }
      result.feedback.push_back(std::move(feedback));
    }
    result.transitions.push_back(std::move(evaluated));
  }
  result.all_transitions_safe = all_safe;
  result.proof_valid = all_safe && all_proven;
  if (result.proof_valid) {
    result.status = "all represented restoration transitions passed scenario-specific DAE threshold certification";
  } else {
    const bool unsafe = std::any_of(
        result.transitions.begin(), result.transitions.end(),
        [](const DistributionResilienceDAETransitionResult& transition) {
          return transition.certificate.proof_valid &&
                 transition.certificate.label == CertificateLabel::Unsafe;
        });
    result.status = unsafe
                        ? "one or more restoration transitions are dynamically unsafe"
                        : "dynamic certification is unresolved or failed for one or more restoration transitions";
  }
  return result;
}

CertifiedDistributionResilienceResult
run_certified_distribution_resilience_mip(
    const HybridPowerSystem& system,
    const DistributionResilienceOptions& restoration_options,
    const dynamics::DynamicSolverOptions& dynamic_options,
    const MultiFidelityCertificateOptions& certificate_options,
    const DistributionResilienceDAECertificationOptions& options) {
  CertifiedDistributionResilienceResult result;
  result.restoration =
      run_distribution_resilience_mip_assessment(system, restoration_options);
  result.dynamic_certification = certify_distribution_resilience_dynamics(
      system, result.restoration, dynamic_options, certificate_options,
      options);
  return result;
}

}  // namespace hacdcpf::analysis

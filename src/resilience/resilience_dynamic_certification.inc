#include "hacdcpf/resilience/resilience_dynamic_certification.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::analysis {
namespace {

using ComponentKey = std::string;
using BusServiceKey = std::string;

constexpr double kServiceTolerance = 1.0e-9;
constexpr const char* kResilienceServiceBaseScale =
    "resilience_service_base_scale";

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
    for (auto& load : system.ac.loads) {
      if (load.bus != canonical_bus) continue;
      const auto base = load.dynamic_model.parameters.find(
          kResilienceServiceBaseScale);
      load.scaling = (base == load.dynamic_model.parameters.end()
                          ? load.scaling
                          : base->second) *
                     service.ratio;
    }
    for (auto& load : system.ac.asymmetric_loads) {
      if (load.bus != canonical_bus) continue;
      const auto base = load.dynamic_model.parameters.find(
          kResilienceServiceBaseScale);
      load.scaling = (base == load.dynamic_model.parameters.end()
                          ? load.scaling
                          : base->second) *
                     service.ratio;
    }
  } else if (service.domain == "DC") {
    for (auto& load : system.dc.loads) {
      if (load.bus != canonical_bus) continue;
      const auto base = load.dynamic_model.parameters.find(
          kResilienceServiceBaseScale);
      load.scaling = (base == load.dynamic_model.parameters.end()
                          ? load.scaling
                          : base->second) *
                     service.ratio;
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

template <typename Range, typename BusFn, typename CapacityFn, typename SetFn>
void allocate_bus_dispatch(Range& range,
                           int bus,
                           double target_mw,
                           BusFn bus_of,
                           CapacityFn capacity_of,
                           SetFn set_dispatch) {
  double total_capacity_mw = 0.0;
  for (const auto& device : range) {
    if (device.in_service && bus_of(device) == bus) {
      total_capacity_mw += std::max(0.0, capacity_of(device));
    }
  }
  for (auto& device : range) {
    if (!device.in_service || bus_of(device) != bus) continue;
    const double share = total_capacity_mw > kServiceTolerance
                             ? std::max(0.0, capacity_of(device)) /
                                   total_capacity_mw
                             : 0.0;
    set_dispatch(device, std::max(0.0, target_mw) * share);
  }
}

void apply_step_dispatch(HybridPowerSystem& system,
                         const DistributionResilienceStepResult& step,
                         const CanonicalBusIds& bus_ids) {
  const std::size_t count = std::min(
      {step.bus_supply_kind.size(), step.bus_supply_index.size(),
       step.bus_supply_dispatchable_generation_mw.size(),
       step.bus_supply_renewable_mw.size(),
       step.bus_supply_fixed_storage_mw.size()});
  for (std::size_t i = 0; i < count; ++i) {
    const std::string& domain = step.bus_supply_kind[i];
    const int bus = bus_ids.resolve(domain, step.bus_supply_index[i]);
    const double dispatchable =
        step.bus_supply_dispatchable_generation_mw[i];
    const double renewable = step.bus_supply_renewable_mw[i];
    const double storage = step.bus_supply_fixed_storage_mw[i];
    if (domain == "AC") {
      double generator_capacity = 0.0;
      for (const auto& generator : system.ac.generators) {
        if (generator.in_service && generator.bus == bus) {
          generator_capacity +=
              std::max({0.0, generator.pmax_mw, generator.pg_mw});
        }
      }
      for (const auto& generator : system.ac.static_generators) {
        if (generator.in_service && generator.bus == bus) {
          generator_capacity +=
              std::max({0.0, generator.pmax_mw, generator.p_mw});
        }
      }
      for (auto& generator : system.ac.generators) {
        if (!generator.in_service || generator.bus != bus) continue;
        const double capacity =
            std::max({0.0, generator.pmax_mw, generator.pg_mw});
        generator.pg_mw = generator_capacity > kServiceTolerance
                              ? dispatchable * capacity / generator_capacity
                              : 0.0;
      }
      for (auto& generator : system.ac.static_generators) {
        if (!generator.in_service || generator.bus != bus) continue;
        const double capacity =
            std::max({0.0, generator.pmax_mw, generator.p_mw});
        generator.p_mw = generator_capacity > kServiceTolerance
                             ? dispatchable * capacity / generator_capacity
                             : 0.0;
      }
      double renewable_capacity = 0.0;
      for (const auto& device : system.ac.renewable_gens) {
        if (device.in_service && device.bus == bus) {
          renewable_capacity += std::max(device.p_rated_mw, device.p_mw);
        }
      }
      for (const auto& device : system.ac.pv_systems) {
        if (device.in_service && device.bus == bus) {
          renewable_capacity += std::max(device.pmax_mw, device.p_mw);
        }
      }
      for (auto& device : system.ac.renewable_gens) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity = std::max(device.p_rated_mw, device.p_mw);
        device.p_mw = renewable_capacity > kServiceTolerance
                          ? renewable * capacity / renewable_capacity
                          : 0.0;
      }
      for (auto& device : system.ac.pv_systems) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity = std::max(device.pmax_mw, device.p_mw);
        device.p_mw = renewable_capacity > kServiceTolerance
                          ? renewable * capacity / renewable_capacity
                          : 0.0;
      }
      allocate_bus_dispatch(
          system.ac.storage, bus, storage,
          [](const auto& device) { return device.bus; },
          [](const auto& device) {
            return std::max(device.p_rated_mw, device.pmax_mw);
          },
          [](auto& device, double value) { device.p_mw = value; });
    } else if (domain == "DC") {
      double generator_capacity = 0.0;
      for (const auto& device : system.dc.static_generators) {
        if (device.in_service && device.bus == bus) {
          generator_capacity +=
              std::max({0.0, device.pmax_mw, device.p_mw});
        }
      }
      for (const auto& device : system.dc.dc_static_generators) {
        if (device.in_service && device.bus == bus) {
          generator_capacity +=
              std::max({0.0, device.pmax_mw, device.p_set_mw});
        }
      }
      for (auto& device : system.dc.static_generators) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity =
            std::max({0.0, device.pmax_mw, device.p_mw});
        device.p_mw = generator_capacity > kServiceTolerance
                          ? dispatchable * capacity / generator_capacity
                          : 0.0;
      }
      for (auto& device : system.dc.dc_static_generators) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity =
            std::max({0.0, device.pmax_mw, device.p_set_mw});
        device.p_set_mw = generator_capacity > kServiceTolerance
                              ? dispatchable * capacity / generator_capacity
                              : 0.0;
      }
      allocate_bus_dispatch(
          system.dc.pv_arrays, bus, renewable,
          [](const auto& device) { return device.bus; },
          [](const auto& device) { return device.p_set_mw; },
          [](auto& device, double value) { device.p_set_mw = value; });
      double storage_capacity = 0.0;
      for (const auto& device : system.dc.storage) {
        if (device.in_service && device.bus == bus) {
          storage_capacity += std::max(device.p_rated_mw, device.pmax_mw);
        }
      }
      for (const auto& device : system.dc.dc_storage) {
        if (device.in_service && device.bus == bus) {
          storage_capacity += std::max(device.p_rated_mw, device.pmax_mw);
        }
      }
      for (auto& device : system.dc.storage) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity = std::max(device.p_rated_mw, device.pmax_mw);
        device.p_mw = storage_capacity > kServiceTolerance
                          ? storage * capacity / storage_capacity
                          : 0.0;
      }
      for (auto& device : system.dc.dc_storage) {
        if (!device.in_service || device.bus != bus) continue;
        const double capacity = std::max(device.p_rated_mw, device.pmax_mw);
        device.p_mw = storage_capacity > kServiceTolerance
                          ? storage * capacity / storage_capacity
                          : 0.0;
      }
    }
  }
}

void assign_dynamic_initialization_references(HybridPowerSystem& system) {
  const int bus_count = static_cast<int>(system.ac.buses.size());
  if (bus_count == 0) return;
  std::unordered_map<int, int> bus_position;
  bus_position.reserve(system.ac.buses.size());
  for (int pos = 0; pos < bus_count; ++pos) {
    if (system.ac.buses[static_cast<std::size_t>(pos)].in_service) {
      bus_position.emplace(
          system.ac.buses[static_cast<std::size_t>(pos)].index, pos);
    }
  }

  std::vector<int> parent(static_cast<std::size_t>(bus_count));
  std::iota(parent.begin(), parent.end(), 0);
  const auto find_root = [&](int value, const auto& self) -> int {
    if (parent[static_cast<std::size_t>(value)] == value) return value;
    parent[static_cast<std::size_t>(value)] =
        self(parent[static_cast<std::size_t>(value)], self);
    return parent[static_cast<std::size_t>(value)];
  };
  const auto join = [&](int lhs, int rhs) {
    const int lhs_root = find_root(lhs, find_root);
    const int rhs_root = find_root(rhs, find_root);
    if (lhs_root != rhs_root) {
      parent[static_cast<std::size_t>(rhs_root)] = lhs_root;
    }
  };
  for (const auto& branch : system.ac.branches) {
    if (!branch.in_service) continue;
    const auto from = bus_position.find(branch.from_bus);
    const auto to = bus_position.find(branch.to_bus);
    if (from != bus_position.end() && to != bus_position.end()) {
      join(from->second, to->second);
    }
  }

  std::unordered_map<int, bool> has_terminal_slack;
  std::unordered_map<int, bool> has_gfm_vsc_reference;
  std::unordered_map<int, std::vector<std::pair<int, int>>> candidates;
  std::unordered_map<int, double> active_load_mw;
  for (int pos = 0; pos < bus_count; ++pos) {
    const auto& bus = system.ac.buses[static_cast<std::size_t>(pos)];
    if (!bus.in_service) continue;
    const int root = find_root(pos, find_root);
    if (bus.bus_type == BusType::SLACK) has_terminal_slack[root] = true;
  }
  for (const auto& converter : system.vsc_converters) {
    if (!converter.in_service ||
        !resolve_device_control_role(converter).is_ac_grid_forming) {
      continue;
    }
    const auto bus = bus_position.find(converter.bus_ac);
    if (bus != bus_position.end()) {
      has_gfm_vsc_reference[find_root(bus->second, find_root)] = true;
    }
  }
  for (const auto& storage : system.ac.storage) {
    if (!storage.in_service || !storage.grid_forming) continue;
    const auto bus = bus_position.find(storage.bus);
    if (bus != bus_position.end()) {
      candidates[find_root(bus->second, find_root)].push_back(
          {storage.index, bus->second});
    }
  }
  for (const auto& generator : system.ac.generators) {
    if (!generator.in_service ||
        !(generator.is_slack || generator.xdpp_pu > 0.0 ||
          generator.xd_pu > 0.0)) {
      continue;
    }
    const auto bus = bus_position.find(generator.bus);
    if (bus != bus_position.end()) {
      // Synchronous machines precede storage when stable component IDs tie.
      candidates[find_root(bus->second, find_root)].push_back(
          {-generator.index, bus->second});
    }
  }
  for (const auto& generator : system.ac.static_generators) {
    if (!generator.in_service || !generator.grid_forming) continue;
    const auto bus = bus_position.find(generator.bus);
    if (bus != bus_position.end()) {
      candidates[find_root(bus->second, find_root)].push_back(
          {generator.index, bus->second});
    }
  }

  for (const auto& load : system.ac.loads) {
    if (!load.in_service) continue;
    const auto bus = bus_position.find(load.bus);
    if (bus != bus_position.end()) {
      active_load_mw[find_root(bus->second, find_root)] +=
          std::abs(load.p_mw * std::max(0.0, load.scaling));
    }
  }
  for (const auto& load : system.ac.asymmetric_loads) {
    if (!load.in_service) continue;
    const auto bus = bus_position.find(load.bus);
    if (bus != bus_position.end()) {
      active_load_mw[find_root(bus->second, find_root)] +=
          std::abs((load.pa_mw + load.pb_mw + load.pc_mw) *
                   std::max(0.0, load.scaling));
    }
  }

  for (auto& [root, island_candidates] : candidates) {
    if (has_terminal_slack[root] || has_gfm_vsc_reference[root] ||
        island_candidates.empty()) {
      continue;
    }
    std::sort(island_candidates.begin(), island_candidates.end());
    const int bus_pos = island_candidates.front().second;
    system.ac.buses[static_cast<std::size_t>(bus_pos)].bus_type = BusType::SLACK;
  }

  for (int pos = 0; pos < bus_count; ++pos) {
    auto& bus = system.ac.buses[static_cast<std::size_t>(pos)];
    if (!bus.in_service) continue;
    const int root = find_root(pos, find_root);
    if (has_terminal_slack[root] || has_gfm_vsc_reference[root] ||
        !candidates[root].empty() || active_load_mw[root] > kServiceTolerance) {
      continue;
    }
    bus.bus_type = BusType::ISOLATED;
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

    dynamics::DynamicEvent event;
    event.time_s = options.switching_time_s;
    event.type = target_state.domain == "DC"
                     ? dynamics::DynamicEventType::DCLoadScale
                     : dynamics::DynamicEventType::ACLoadScale;
    event.bus = target_state.bus;
    event.value = std::max(0.0, target_state.ratio);
    event.params["scale"] = event.value;
    event.params["service_ratio"] = event.value;
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
  HybridPowerSystem canonical;
  if (system.projection_certificate.has_value()) {
    canonical = system;
  } else {
    auto input = system;
    const auto mobile_storage = input.mobile_storage;
    input.mobile_storage.clear();
    ProjectionOptions options;
    options.mode = ProjectionMode::ExactIdeal;
    options.strip_dead_islands = false;
    options.preserve_switch_branches = true;
    canonical = project_to_canonical_models(std::move(input), options);
    canonical.mobile_storage = mobile_storage;
  }

  // Preserve authored load ratings independently of the restoration service
  // ratio. Dynamic load events are absolute assignments, so baking a zero
  // service ratio into p/q would make later pickup mathematically impossible.
  // See docs/modules/resilience/chapters/dynamic_certificate_derivation.tex.
  int next_ac_load_index = 1;
  for (const auto& load : canonical.ac.loads) {
    next_ac_load_index = std::max(next_ac_load_index, load.index + 1);
  }
  for (auto& load : canonical.ac.loads) {
    load.dynamic_model.parameters[kResilienceServiceBaseScale] =
        std::max(0.0, load.scaling);
  }
  for (auto& load : canonical.ac.asymmetric_loads) {
    load.dynamic_model.parameters[kResilienceServiceBaseScale] =
        std::max(0.0, load.scaling);
  }
  for (auto& bus : canonical.ac.buses) {
    if (std::abs(bus.pd_mw) <= kServiceTolerance &&
        std::abs(bus.qd_mvar) <= kServiceTolerance) {
      continue;
    }
    Load load;
    load.index = next_ac_load_index++;
    load.bus = bus.index;
    load.name = "DAE service base for AC bus " + std::to_string(bus.index);
    load.p_mw = bus.pd_mw;
    load.q_mvar = bus.qd_mvar;
    load.scaling = 1.0;
    load.dynamic_model.parameters[kResilienceServiceBaseScale] = 1.0;
    canonical.ac.loads.push_back(std::move(load));
    bus.pd_mw = 0.0;
    bus.qd_mvar = 0.0;
  }

  int next_dc_load_index = 1;
  for (const auto& load : canonical.dc.loads) {
    next_dc_load_index = std::max(next_dc_load_index, load.index + 1);
  }
  for (auto& load : canonical.dc.loads) {
    load.dynamic_model.parameters[kResilienceServiceBaseScale] =
        std::max(0.0, load.scaling);
  }
  for (auto& bus : canonical.dc.buses) {
    if (std::abs(bus.pd_mw) <= kServiceTolerance) continue;
    DCLoad load;
    load.index = next_dc_load_index++;
    load.bus = bus.index;
    load.name = "DAE service base for DC bus " + std::to_string(bus.index);
    load.p_mw = bus.pd_mw;
    load.p_rated_mw = std::abs(bus.pd_mw);
    load.scaling = 1.0;
    load.dynamic_model.parameters[kResilienceServiceBaseScale] = 1.0;
    canonical.dc.loads.push_back(std::move(load));
    bus.pd_mw = 0.0;
  }
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

const DistributionResilienceStepResult* find_restoration_step(
    const DistributionResilienceResult& restoration, int step) {
  const auto it = std::find_if(
      restoration.steps.begin(), restoration.steps.end(),
      [&](const DistributionResilienceStepResult& row) {
        return row.step_index == step;
      });
  return it == restoration.steps.end() ? nullptr : &*it;
}

DistributionResilienceMIPFeedbackCut make_dae_feedback_cut(
    const DistributionResilienceResult& restoration,
    const DistributionResilienceDAETransitionResult& transition) {
  DistributionResilienceMIPFeedbackCut cut;
  cut.source_action_id = transition.action.id;
  cut.reason = "proof-valid Unsafe DAE certificate for the exact restoration transition";

  const auto append_topology = [&](const DistributionResilienceStepResult* step) {
    if (step == nullptr) return;
    for (const auto& state : step->component_states) {
      cut.topology_literals.push_back(
          {step->step_index, state.component_type, state.domain,
           state.component_index, state.pair_number,
           state.available && state.closed});
    }
  };
  const auto* previous = find_restoration_step(restoration, transition.from_step);
  const auto* target = find_restoration_step(restoration, transition.to_step);
  if (previous != nullptr && transition.from_step != transition.to_step) {
    append_topology(previous);
  }
  append_topology(target);

  if (target != nullptr) {
    const auto previous_service =
        previous == nullptr
            ? std::unordered_map<BusServiceKey, BusServiceState>{}
            : service_map(*previous);
    for (const auto& [key, target_service] : service_map(*target)) {
      double previous_served_mw = 0.0;
      if (previous == nullptr) {
        const auto target_pos = std::find_if(
            target->bus_supply_index.begin(), target->bus_supply_index.end(),
            [&](int bus) { return bus == target_service.bus; });
        if (target_pos != target->bus_supply_index.end()) {
          const auto pos = static_cast<std::size_t>(
              target_pos - target->bus_supply_index.begin());
          if (pos < target->bus_supply_demand_mw.size()) {
            previous_served_mw = target->bus_supply_demand_mw[pos];
          }
        }
      } else if (const auto it = previous_service.find(key);
                 it != previous_service.end()) {
        const auto previous_pos = std::find_if(
            previous->bus_supply_index.begin(), previous->bus_supply_index.end(),
            [&](int bus) { return bus == target_service.bus; });
        if (previous_pos != previous->bus_supply_index.end()) {
          const auto pos = static_cast<std::size_t>(
              previous_pos - previous->bus_supply_index.begin());
          if (pos < previous->bus_supply_served_mw.size()) {
            previous_served_mw = previous->bus_supply_served_mw[pos];
          }
        }
      }

      const auto target_pos = std::find_if(
          target->bus_supply_index.begin(), target->bus_supply_index.end(),
          [&](int bus) { return bus == target_service.bus; });
      if (target_pos == target->bus_supply_index.end()) continue;
      const auto pos = static_cast<std::size_t>(
          target_pos - target->bus_supply_index.begin());
      if (pos >= target->bus_supply_served_mw.size()) continue;
      if (target->bus_supply_served_mw[pos] >
          previous_served_mw + kServiceTolerance) {
        cut.service_upper_bounds.push_back(
            {transition.to_step, target_service.domain, target_service.bus,
             previous_served_mw});
      }
    }
  }
  return cut;
}

std::string feedback_cut_key(const DistributionResilienceMIPFeedbackCut& cut) {
  std::string key;
  for (const auto& literal : cut.topology_literals) {
    key += "T:" + std::to_string(literal.step) + ":" +
           literal.component_type + ":" + literal.domain + ":" +
           std::to_string(literal.component_index) + ":" +
           std::to_string(literal.pair_number) + ":" +
           (literal.closed ? "1" : "0") + ";";
  }
  for (const auto& bound : cut.service_upper_bounds) {
    key += "S:" + std::to_string(bound.step) + ":" + bound.domain + ":" +
           std::to_string(bound.bus_index) + ":" +
           std::to_string(bound.served_mw) + ";";
  }
  return key;
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
      apply_step_dispatch(transition.initial_system, *previous, bus_ids);
    }
    assign_dynamic_initialization_references(transition.initial_system);
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
      "Aggregated MIP generator, renewable, and fixed-storage dispatch is replayed per bus and allocated across represented devices in proportion to available power capacity.",
      "MIP service is replayed as aggregate domain-qualified bus load scaling; allocation among individual loads at the same bus is not recovered.",
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
  auto master_options = restoration_options;
  std::vector<DistributionResilienceDAEFeedback> feedback_history;
  std::vector<std::string> applied_cut_keys;
  const int max_feedback_iterations =
      options.enable_feedback_loop ? std::max(0, options.max_feedback_iterations)
                                   : 0;

  for (int iteration = 0; iteration <= max_feedback_iterations; ++iteration) {
    result.restoration =
        run_distribution_resilience_mip_assessment(system, master_options);
    ++result.dynamic_certification.restoration_mip_solves;
    if (!result.restoration.feasible) {
      result.dynamic_certification.feedback_closed_loop_complete =
          !applied_cut_keys.empty();
      result.dynamic_certification.safe_plan_found = false;
      result.dynamic_certification.feedback_iterations = iteration;
      result.dynamic_certification.feedback_cuts_applied =
          static_cast<int>(applied_cut_keys.size());
      result.dynamic_certification.feedback = std::move(feedback_history);
      result.dynamic_certification.status =
          applied_cut_keys.empty()
              ? "restoration MIP is infeasible before dynamic feedback"
              : "DAE feedback closed the finite master with no dynamically safe feasible plan";
      return result;
    }

    auto certificate = certify_distribution_resilience_dynamics(
        system, result.restoration, dynamic_options, certificate_options,
        options);
    certificate.restoration_mip_solves = iteration + 1;
    certificate.feedback_iterations = iteration;
    certificate.feedback_cuts_applied =
        static_cast<int>(applied_cut_keys.size());
    if (certificate.proof_valid && certificate.all_transitions_safe) {
      certificate.feedback_closed_loop_complete = true;
      certificate.safe_plan_found = true;
      feedback_history.insert(feedback_history.end(), certificate.feedback.begin(),
                              certificate.feedback.end());
      certificate.feedback = std::move(feedback_history);
      result.dynamic_certification = std::move(certificate);
      return result;
    }

    std::vector<DistributionResilienceMIPFeedbackCut> new_cuts;
    for (const auto& transition : certificate.transitions) {
      if (!(transition.certificate.proof_valid &&
            transition.certificate.label == CertificateLabel::Unsafe)) {
        continue;
      }
      auto cut = make_dae_feedback_cut(result.restoration, transition);
      if (cut.topology_literals.empty() && cut.service_upper_bounds.empty()) {
        continue;
      }
      const std::string key = feedback_cut_key(cut);
      if (std::find(applied_cut_keys.begin(), applied_cut_keys.end(), key) !=
          applied_cut_keys.end()) {
        continue;
      }
      applied_cut_keys.push_back(key);
      new_cuts.push_back(std::move(cut));
      for (auto& feedback : certificate.feedback) {
        if (feedback.action_id == transition.action.id) {
          feedback.applied_to_restoration_mip = true;
          feedback.applied_iteration = iteration + 1;
        }
      }
    }
    feedback_history.insert(feedback_history.end(), certificate.feedback.begin(),
                            certificate.feedback.end());

    if (!options.enable_feedback_loop || new_cuts.empty() ||
        iteration == max_feedback_iterations) {
      certificate.feedback_closed_loop_complete = false;
      certificate.safe_plan_found = false;
      certificate.feedback_cuts_applied =
          static_cast<int>(applied_cut_keys.size());
      certificate.feedback = std::move(feedback_history);
      if (new_cuts.empty()) {
        certificate.status +=
            "; no proof-valid Unsafe certificate was available for an operational cut";
      } else {
        certificate.status += "; DAE feedback iteration limit reached";
      }
      result.dynamic_certification = std::move(certificate);
      return result;
    }
    master_options.dynamic_feedback_cuts.insert(
        master_options.dynamic_feedback_cuts.end(), new_cuts.begin(),
        new_cuts.end());
  }
  return result;
}

}  // namespace hacdcpf::analysis

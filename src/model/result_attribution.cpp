#include "hacdcpf/projection/result_attribution.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <unordered_map>

#include "hacdcpf/power_flow/pv_power_curve.hpp"

namespace hacdcpf::projection {
namespace {

template <typename T>
std::string display_name(const T& component, const char* fallback) {
  return component.name.empty()
             ? std::string(fallback) + " " + std::to_string(component.index)
             : component.name;
}

template <typename T>
bool component_in_service(const T& component) {
  if constexpr (requires { component.in_service; }) return component.in_service;
  return true;
}

void set_value(RichComponentResult& row, const std::string& name, double value,
               const std::string& unit = {}) {
  auto it = std::find_if(row.values.begin(), row.values.end(),
                         [&](const AttributedValue& item) {
                           return item.name == name;
                         });
  if (it == row.values.end()) {
    row.values.push_back({name, value, unit});
  } else {
    it->value = value;
    it->unit = unit;
  }
}

void set_terminal(RichComponentResult& row, const std::string& name, int bus,
                  bool is_dc, double p, double q, double v, bool has_p,
                  bool has_q, bool has_v) {
  auto it = std::find_if(row.terminals.begin(), row.terminals.end(),
                         [&](const AttributedTerminal& terminal) {
                           return terminal.name == name;
                         });
  AttributedTerminal value{name, bus, is_dc, p, q, v, has_p, has_q, has_v};
  if (it == row.terminals.end()) row.terminals.push_back(std::move(value));
  else *it = std::move(value);
}

void accumulate_terminal(RichComponentResult& row, const std::string& name,
                         int bus, double p, double q, double v) {
  auto it = std::find_if(row.terminals.begin(), row.terminals.end(),
                         [&](const AttributedTerminal& terminal) {
                           return terminal.name == name;
                         });
  if (it == row.terminals.end()) {
    set_terminal(row, name, bus, false, p, q, v, true, true, true);
    return;
  }
  it->p_mw += p;
  it->q_mvar += q;
}

RichComponentResult* find_row(RichResultAttribution& out,
                              const std::string& type,
                              const std::string& domain,
                              int index) {
  auto it = std::find_if(out.components.begin(), out.components.end(),
                         [&](const RichComponentResult& row) {
                           return row.component_type == type &&
                                  row.domain == domain &&
                                  row.component_index == index;
                         });
  return it == out.components.end() ? nullptr : &*it;
}

template <typename Collection, typename Initializer>
void add_collection(RichResultAttribution& out, const Collection& collection,
                    const char* type, const char* domain, const char* fallback,
                    Initializer initialize) {
  for (size_t i = 0; i < collection.size(); ++i) {
    const auto& component = collection[i];
    RichComponentResult row;
    row.component_type = type;
    row.domain = domain;
    row.component_index = component.index;
    row.position = static_cast<int>(i);
    row.name = display_name(component, fallback);
    row.in_service = component_in_service(component);
    row.recovery = RecoveryClass::AuditOnly;
    row.recovery_reason =
        "rich identity is total; solved observable is not yet attached";
    initialize(row, component);
    out.components.push_back(std::move(row));
  }
}

int parse_component_index(const std::string& value) {
  char* end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  return end != value.c_str() && *end == '\0' ? static_cast<int>(parsed) : 0;
}

double bus_voltage(const HybridPowerSystem& rich, const PowerFlowResult* pf,
                   int bus, bool is_dc) {
  if (is_dc) {
    for (size_t i = 0; i < rich.dc.buses.size(); ++i) {
      if (rich.dc.buses[i].index != bus) continue;
      return pf && i < pf->vdc.size() ? pf->vdc[i] : rich.dc.buses[i].vm_pu;
    }
  } else {
    for (size_t i = 0; i < rich.ac.buses.size(); ++i) {
      if (rich.ac.buses[i].index != bus) continue;
      return pf && i < pf->vm.size() ? pf->vm[i] : rich.ac.buses[i].vm_pu;
    }
  }
  return 0.0;
}

RecoveryClass electrical_recovery(const ProjectionBundle& projection) {
  return projection.canonical.projection_certificate &&
                 !projection.canonical.projection_certificate->exact()
             ? RecoveryClass::Approximate
             : RecoveryClass::Strong;
}

template <typename Collection>
std::optional<size_t> position_for_index(const Collection& collection,
                                         int component_index) {
  const auto it = std::find_if(collection.begin(), collection.end(),
                               [&](const auto& component) {
                                 return component.index == component_index;
                               });
  if (it == collection.end()) return std::nullopt;
  return static_cast<size_t>(std::distance(collection.begin(), it));
}

void attach_projection_sources(const ProjectionBundle& projection,
                               RichResultAttribution& out) {
  if (!projection.canonical.projection_report) return;
  for (const auto& mapping : projection.canonical.projection_report->mappings) {
    const int source_index = parse_component_index(mapping.source_id);
    const int canonical_index = parse_component_index(mapping.canonical_id);
    for (auto& row : out.components) {
      if (row.component_index != source_index) continue;
      const bool type_matches =
          (mapping.source_type == "Transformer2W" &&
           row.component_type == "transformer_2w") ||
          (mapping.source_type == "Transformer3W" &&
           row.component_type == "transformer_3w") ||
          (mapping.source_type == "Switch" && row.component_type == "switch") ||
          (mapping.source_type == "CircuitBreaker" &&
           row.component_type == "circuit_breaker") ||
          (mapping.source_type == "FlexibleLoad" &&
           row.component_type == "flexible_load") ||
          (mapping.source_type == "AsymmetricLoad" &&
           row.component_type == "asymmetric_load") ||
          (mapping.source_type == "AsynchronousMotor" &&
           row.component_type == "motor") ||
          (mapping.source_type == "EnergyRouter" &&
           row.component_type == "energy_router") ||
          (mapping.source_type == "VirtualPowerPlant" &&
           row.component_type == "vpp") ||
          (mapping.source_type == "Microgrid" &&
           row.component_type == "microgrid") ||
          (mapping.source_type == "MobileStorage" &&
           row.component_type == "mobile_storage");
      if (!type_matches) continue;
      row.canonical_sources.push_back(
          {mapping.canonical_type, canonical_index,
           mapping.participation_factor});
    }
  }
}

}  // namespace

ProjectionBundle RichToCanonicalOperator::apply(
    const HybridPowerSystem& rich, const ProjectionOptions& options) {
  ProjectionBundle out;
  out.options = options;
  out.canonical = project_to_canonical_models(rich, options);
  return out;
}

ProjectionBundle RichToCanonicalOperator::apply(
    HybridPowerSystem&& rich, const ProjectionOptions& options) {
  ProjectionBundle out;
  out.options = options;
  out.canonical = project_to_canonical_models(std::move(rich), options);
  return out;
}

std::vector<int> CanonicalToRichOperator::ac_bus_reprojection_positions(
    const HybridPowerSystem& rich, const ProjectionBundle& projection) {
  std::vector<int> positions(rich.ac.buses.size(), -1);
  if (projection.canonical.bus_merge_map) {
    const auto& bus_map = *projection.canonical.bus_merge_map;
    for (size_t i = 0; i < rich.ac.buses.size(); ++i) {
      const auto it = bus_map.ext_to_int.find(rich.ac.buses[i].index);
      if (it != bus_map.ext_to_int.end()) positions[i] = it->second;
    }
    return positions;
  }
  for (size_t i = 0; i < rich.ac.buses.size(); ++i) {
    const auto it = std::find_if(
        projection.canonical.ac.buses.begin(),
        projection.canonical.ac.buses.end(), [&](const ACBus& bus) {
          return bus.index == rich.ac.buses[i].index;
        });
    if (it != projection.canonical.ac.buses.end()) {
      positions[i] = static_cast<int>(std::distance(
          projection.canonical.ac.buses.begin(), it));
    }
  }
  return positions;
}

RichResultAttribution CanonicalToRichOperator::apply(
    const HybridPowerSystem& rich, const ProjectionBundle& projection,
    const opf::ACOPFResult* opf_result,
    const PowerFlowResult* canonical_pf,
    const PowerFlowResult* rich_pf,
    const AttributionOptions& options) {
  RichResultAttribution out;
  const RecoveryClass electrical = electrical_recovery(projection);

  add_collection(out, rich.ac.buses, "ac_bus", "AC", "AC Bus",
                 [&](auto& row, const auto& bus) {
                   const double vm = bus_voltage(rich, rich_pf, bus.index, false);
                   set_value(row, "vm_pu", vm, "pu");
                   if (rich_pf && row.position < static_cast<int>(rich_pf->va.size()))
                     set_value(row, "va_rad", rich_pf->va[row.position], "rad");
                   row.recovery = electrical;
                   row.recovery_reason = "BusMergeMap voltage broadcast";
                 });
  add_collection(out, rich.dc.buses, "dc_bus", "DC", "DC Bus",
                 [&](auto& row, const auto& bus) {
                   set_value(row, "vdc_pu",
                             bus_voltage(rich, rich_pf, bus.index, true), "pu");
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable DC bus identity";
                 });

  add_collection(out, rich.ac.branches, "ac_branch", "AC", "AC Line",
                 [&](auto& row, const auto& branch) {
                   if (rich_pf && row.position <
                                      static_cast<int>(rich_pf->branch_flows.size())) {
                     const auto& flow = rich_pf->branch_flows[row.position];
                     set_terminal(row, "from", branch.from_bus, false, flow.pf_mw,
                                  flow.qf_mvar,
                                  bus_voltage(rich, rich_pf, branch.from_bus, false),
                                  true, true, true);
                     set_terminal(row, "to", branch.to_bus, false, flow.pt_mw,
                                  flow.qt_mvar,
                                  bus_voltage(rich, rich_pf, branch.to_bus, false),
                                  true, true, true);
                     set_value(row, "loss_mw", flow.pf_mw + flow.pt_mw, "MW");
                     row.recovery = electrical;
                     row.recovery_reason = "original branch position map";
                   }
                 });
  add_collection(out, rich.dc.branches, "dc_branch", "DC", "DC Line",
                 [&](auto& row, const auto& branch) {
                   const double vf = bus_voltage(rich, rich_pf, branch.from_bus, true);
                   const double vt = bus_voltage(rich, rich_pf, branch.to_bus, true);
                   if (branch.in_service && branch.r_pu > 1e-12) {
                     const double current = (vf - vt) / branch.r_pu;
                     const double pf = vf * current * rich.base_mva;
                     const double pt = -vt * current * rich.base_mva;
                     set_terminal(row, "from", branch.from_bus, true, pf, 0.0, vf,
                                  true, false, true);
                     set_terminal(row, "to", branch.to_bus, true, pt, 0.0, vt,
                                  true, false, true);
                     set_value(row, "loss_mw", pf + pt, "MW");
                     row.recovery = electrical;
                     row.recovery_reason = "DC conductance terminal equation";
                   }
                 });

  add_collection(out, rich.ac.generators, "generator", "AC", "Generator",
                 [&](auto& row, const auto& generator) {
                   const auto canonical_position = position_for_index(
                       projection.canonical.ac.generators, generator.index);
                   const double p = opf_result && canonical_position &&
                                            *canonical_position < opf_result->pg_mw.size()
                                        ? opf_result->pg_mw[*canonical_position]
                                        : generator.pg_mw;
                   const double q = opf_result && canonical_position &&
                                            *canonical_position < opf_result->qg_mvar.size()
                                        ? opf_result->qg_mvar[*canonical_position]
                                        : generator.qg_mvar;
                   set_value(row, "p_mw", p, "MW");
                   set_value(row, "q_mvar", q, "MVar");
                   set_terminal(row, "ac", generator.bus, false, p, q,
                                bus_voltage(rich, rich_pf, generator.bus, false),
                                true, true, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable generator identity";
                 });
  add_collection(out, rich.ac.external_grids, "external_grid", "AC",
                 "External Grid", [&](auto& row, const auto& grid) {
                   const auto canonical_position = position_for_index(
                       projection.canonical.ac.external_grids, grid.index);
                   if (!opf_result || !canonical_position ||
                       *canonical_position >= opf_result->external_grid_p_mw.size() ||
                       *canonical_position >= opf_result->external_grid_q_mvar.size())
                     return;
                   const double p =
                       opf_result->external_grid_p_mw[*canonical_position];
                   const double q =
                       opf_result->external_grid_q_mvar[*canonical_position];
                   set_value(row, "p_mw", p, "MW");
                   set_value(row, "q_mvar", q, "MVar");
                   set_terminal(row, "ac", grid.bus, false, p, q,
                                bus_voltage(rich, rich_pf, grid.bus, false),
                                true, true, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "explicit external-grid OPF variable";
                 });

  add_collection(out, rich.ac.loads, "load", "AC", "Load",
                 [&](auto& row, const auto& load) {
                   set_value(row, "p_mw", load.in_service ? load.p_mw * load.scaling : 0.0, "MW");
                   set_value(row, "q_mvar", load.in_service ? load.q_mvar * load.scaling : 0.0, "MVar");
                 });
  add_collection(out, rich.dc.loads, "dc_load", "DC", "DC Load",
                 [&](auto& row, const auto& load) {
                   set_value(row, "p_mw", load.in_service ? load.p_mw * load.scaling : 0.0, "MW");
                 });
  add_collection(out, rich.ac.static_generators, "static_generator", "AC",
                 "Static Generator", [&](auto& row, const auto& source) {
                   set_value(row, "p_mw", source.in_service ? source.p_mw * source.scaling : 0.0, "MW");
                   set_value(row, "q_mvar", source.in_service ? source.q_mvar * source.scaling : 0.0, "MVar");
                 });
  add_collection(out, rich.dc.static_generators, "static_generator", "DC",
                 "DC Static Generator", [&](auto& row, const auto& source) {
                   set_value(row, "p_mw", source.in_service ? source.p_mw * source.scaling : 0.0, "MW");
                 });
  add_collection(out, rich.dc.dc_static_generators, "dc_static_generator", "DC",
                 "DC Static Generator", [&](auto& row, const auto& source) {
                   set_value(row, "p_mw", source.in_service ? source.p_set_mw * source.scaling : 0.0, "MW");
                 });
  add_collection(out, rich.dc.pv_arrays, "dc_pv_array", "DC", "DC PV",
                 [&](auto& row, const auto& pv) {
                   set_value(row, "p_mw", pv.in_service ? pv.p_set_mw : 0.0, "MW");
                 });

  add_collection(out, rich.ac.storage, "storage", "AC", "ESS",
                 [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "q_mvar", storage.q_mvar, "MVar");
                   set_value(row, "soc", storage.soc_init);
                   set_terminal(row, "ac", storage.bus, false, storage.p_mw,
                                storage.q_mvar,
                                bus_voltage(rich, rich_pf, storage.bus, false),
                                true, true, true);
                   row.canonical_sources.push_back(
                       {"Storage", storage.index, 1.0});
                 });
  add_collection(out, rich.dc.storage, "storage", "DC", "DC ESS",
                 [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "soc", storage.soc_init);
                   set_terminal(row, "dc", storage.bus, true, storage.p_mw,
                                0.0,
                                bus_voltage(rich, rich_pf, storage.bus, true),
                                true, false, true);
                   row.canonical_sources.push_back(
                       {"DCStorage", storage.index, 1.0});
                 });
  add_collection(out, rich.dc.dc_storage, "dc_storage", "DC", "DC ESS",
                 [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "soc", storage.soc_init);
                   set_terminal(row, "dc", storage.bus, true, storage.p_mw,
                                0.0,
                                bus_voltage(rich, rich_pf, storage.bus, true),
                                true, false, true);
                   row.canonical_sources.push_back(
                       {"DCStorage", storage.index, 1.0});
                 });

  add_collection(out, rich.ac.renewable_gens, "renewable_generator", "AC",
                 "Renewable", [&](auto& row, const auto& source) {
                   set_value(row, "p_mw", source.p_mw, "MW");
                   set_value(row, "q_mvar", source.q_mvar, "MVar");
                 });
  add_collection(out, rich.ac.pv_systems, "pv_system", "AC", "PV",
                 [&](auto& row, const auto& source) {
                   const double p_mw =
                       source.voc > 0.0 && source.isc > 0.0 &&
                               source.vmpp > 0.0
                           ? powerflow::compute_pv_power_mw(source)
                           : source.p_mw;
                   set_value(row, "p_mw", p_mw, "MW");
                   set_value(row, "q_mvar", source.q_mvar, "MVar");
                 });
  add_collection(out, rich.ac.flexible_loads, "flexible_load", "AC",
                 "Flexible Load", [&](auto& row, const auto& load) {
                   set_value(row, "p_mw", load.p_mw, "MW");
                   set_value(row, "q_mvar", load.q_mvar, "MVar");
                 });
  add_collection(out, rich.ac.asymmetric_loads, "asymmetric_load", "AC",
                 "Asymmetric Load", [&](auto& row, const auto& load) {
                   set_value(row, "p_mw", load.pa_mw + load.pb_mw + load.pc_mw, "MW");
                   set_value(row, "q_mvar", load.qa_mvar + load.qb_mvar + load.qc_mvar, "MVar");
                 });
  add_collection(out, rich.ac.motors, "motor", "AC", "Motor",
                 [&](auto& row, const auto& motor) {
                   set_value(row, "p_mw", motor.sn_mva * motor.cos_phi, "MW");
                 });
  add_collection(out, rich.ac.shunts, "shunt", "AC", "Shunt",
                 [&](auto& row, const auto& shunt) {
                   set_value(row, "p_mw", -shunt.gs_mw, "MW");
                   set_value(row, "q_mvar", shunt.bs_mvar, "MVar");
                 });

  add_collection(out, rich.ac.transformers_2w, "transformer_2w", "AC",
                 "Transformer 2W", [](auto&, const auto&) {});
  add_collection(out, rich.ac.transformers_3w, "transformer_3w", "AC",
                 "Transformer 3W", [](auto&, const auto&) {});
  add_collection(out, rich.ac.switches, "switch", "AC", "Switch",
                 [](auto&, const auto&) {});
  add_collection(out, rich.ac.circuit_breakers, "circuit_breaker", "AC",
                 "Circuit Breaker", [](auto&, const auto&) {});
  add_collection(out, rich.dc.dc_circuit_breakers, "circuit_breaker", "DC",
                 "DC Circuit Breaker", [](auto&, const auto&) {});

  add_collection(out, rich.vsc_converters, "vsc_converter", "ACDC", "VSC",
                 [&](auto& row, const auto& converter) {
                   set_value(row, "p_schedule_mw", converter.p_schedule_mw, "MW");
                   set_value(row, "p_initial_mw", converter.p_initial_mw, "MW");
                   if (rich_pf) {
                     const auto it = std::find_if(
                         rich_pf->vsc_transfers.begin(), rich_pf->vsc_transfers.end(),
                         [&](const VSCTransfer& transfer) {
                           return transfer.index == converter.index;
                         });
                     if (it != rich_pf->vsc_transfers.end()) {
                       set_terminal(row, "ac", converter.bus_ac, false,
                                    it->p_ac_mw, it->q_ac_mvar,
                                    bus_voltage(rich, rich_pf, converter.bus_ac, false),
                                    true, true, true);
                       set_terminal(row, "dc", converter.bus_dc, true,
                                    it->p_dc_mw, 0.0,
                                    bus_voltage(rich, rich_pf, converter.bus_dc, true),
                                    true, false, true);
                       set_value(row, "loss_mw", it->loss_mw, "MW");
                       row.recovery = RecoveryClass::Strong;
                       row.recovery_reason = "VSC identity and ordered AC/DC terminals";
                     }
                   }
                 });
  add_collection(out, rich.dc.dcdc_converters, "dcdc_converter", "DC",
                 "DCDC", [&](auto& row, const auto& converter) {
                   if (rich_pf) {
                     const auto it = std::find_if(
                         rich_pf->dcdc_transfers.begin(), rich_pf->dcdc_transfers.end(),
                         [&](const DCDCTransfer& transfer) {
                           return transfer.index == converter.index;
                         });
                     if (it != rich_pf->dcdc_transfers.end()) {
                       set_terminal(row, "in", converter.bus_in, true,
                                    -it->p_in_mw, 0.0,
                                    bus_voltage(rich, rich_pf, converter.bus_in, true),
                                    true, false, true);
                       set_terminal(row, "out", converter.bus_out, true,
                                    it->p_out_mw, 0.0,
                                    bus_voltage(rich, rich_pf, converter.bus_out, true),
                                    true, false, true);
                       set_value(row, "loss_mw", it->loss_mw, "MW");
                       row.recovery = RecoveryClass::Strong;
                       row.recovery_reason = "DCDC identity and ordered terminals";
                     }
                   }
                 });

  add_collection(out, rich.ac.regulator_controls, "regulator_control", "AC",
                 "Regulator", [](auto&, const auto&) {});
  add_collection(out, rich.ac.charging_stations, "charging_station", "AC",
                 "Charging Station", [&](auto& row, const auto& station) {
                   set_value(row, "p_mw", station.p_total_kw / 1000.0, "MW");
                   set_value(row, "q_mvar", station.q_total_kvar / 1000.0, "MVar");
                 });
  add_collection(out, rich.ac.chargers, "charger", "AC", "Charger",
                 [&](auto& row, const auto& charger) {
                   set_value(row, "p_rated_kw", charger.p_rated_kw, "kW");
                 });
  add_collection(out, rich.energy_routers, "energy_router", "ACDC",
                 "Energy Router", [](auto&, const auto&) {});
  add_collection(out, rich.mobile_storage, "mobile_storage", "AC",
                 "Mobile ESS", [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "q_mvar", storage.q_mvar, "MVar");
                   set_value(row, "soc", storage.soc_init);
                 });
  add_collection(out, rich.vpps, "vpp", "AC", "VPP",
                 [&](auto& row, const auto& vpp) {
                   set_value(row, "p_mw", vpp.p_output_mw, "MW");
                   set_value(row, "q_mvar", vpp.q_output_mvar, "MVar");
                 });
  add_collection(out, rich.microgrids, "microgrid", "AC", "Microgrid",
                 [&](auto& row, const auto& microgrid) {
                   set_value(row, "p_mw", microgrid.p_exchange_mw, "MW");
                 });

  attach_projection_sources(projection, out);

  // Attribute expanded AC branch terminal powers to rich transformers and
  // non-collapsed switchgear using BranchExpandMap, never vector position.
  if (canonical_pf && projection.canonical.branch_expand_map) {
    std::unordered_map<int, const BranchFlow*> flow_by_branch_index;
    for (size_t i = 0; i < projection.canonical.ac.branches.size() &&
                       i < canonical_pf->branch_flows.size(); ++i) {
      flow_by_branch_index[projection.canonical.ac.branches[i].index] =
          &canonical_pf->branch_flows[i];
    }
    for (const auto& entry : projection.canonical.branch_expand_map->entries) {
      const auto fit = flow_by_branch_index.find(entry.branch_index);
      if (fit == flow_by_branch_index.end()) continue;
      const BranchFlow& flow = *fit->second;
      const char* type = entry.origin_type == BranchOriginType::Transformer2W
                             ? "transformer_2w"
                         : entry.origin_type == BranchOriginType::Transformer3W
                             ? "transformer_3w"
                         : entry.origin_type == BranchOriginType::Switch
                             ? "switch"
                             : "circuit_breaker";
      RichComponentResult* row = find_row(out, type, "AC", entry.origin_index);
      if (!row) continue;
      if (entry.origin_type == BranchOriginType::Transformer3W) {
        const auto transformer = std::find_if(
            rich.ac.transformers_3w.begin(), rich.ac.transformers_3w.end(),
            [&](const auto& item) { return item.index == entry.origin_index; });
        if (transformer != rich.ac.transformers_3w.end()) {
          const int from_bus = entry.pair_number < 2 ? transformer->hv_bus
                                                     : transformer->mv_bus;
          const int to_bus = entry.pair_number == 0 ? transformer->mv_bus
                            : entry.pair_number == 1 ? transformer->lv_bus
                                                     : transformer->lv_bus;
          const std::string from_name = entry.pair_number < 2 ? "hv" : "mv";
          const std::string to_name = entry.pair_number == 0 ? "mv" : "lv";
          accumulate_terminal(*row, from_name, from_bus, flow.pf_mw,
                              flow.qf_mvar,
                              bus_voltage(rich, rich_pf, from_bus, false));
          accumulate_terminal(*row, to_name, to_bus, flow.pt_mw,
                              flow.qt_mvar,
                              bus_voltage(rich, rich_pf, to_bus, false));
        }
      } else {
        int from_bus = entry.bus_from;
        int to_bus = entry.bus_to;
        if (entry.origin_type == BranchOriginType::Transformer2W) {
          const auto transformer = std::find_if(
              rich.ac.transformers_2w.begin(), rich.ac.transformers_2w.end(),
              [&](const auto& item) { return item.index == entry.origin_index; });
          if (transformer != rich.ac.transformers_2w.end()) {
            from_bus = transformer->hv_bus;
            to_bus = transformer->lv_bus;
          }
        }
        set_terminal(*row, "from", from_bus, false, flow.pf_mw,
                     flow.qf_mvar, bus_voltage(rich, rich_pf, from_bus, false),
                     true, true, from_bus >= 0);
        set_terminal(*row, "to", to_bus, false, flow.pt_mw,
                     flow.qt_mvar, bus_voltage(rich, rich_pf, to_bus, false),
                     true, true, to_bus >= 0);
      }
      row->recovery = electrical;
      row->recovery_reason = "BranchExpandMap terminal attribution";
    }
  }

  // Collapsed ideal switchgear has no canonical branch row. Recover its flow
  // from the recorded terminal cut in original topology.
  if (rich_pf) {
    DeviceTerminalFlows device_flows =
        compute_device_terminal_flows(rich, rich_pf->branch_flows);
    for (const auto& flow : device_flows.ac_switches) {
      if (auto* row = find_row(out, "switch", "AC", flow.index)) {
        set_terminal(*row, "from", flow.bus_from, false, flow.pf_mw,
                     flow.qf_mvar, bus_voltage(rich, rich_pf, flow.bus_from, false),
                     true, true, true);
        set_terminal(*row, "to", flow.bus_to, false, flow.pt_mw,
                     flow.qt_mvar, bus_voltage(rich, rich_pf, flow.bus_to, false),
                     true, true, true);
        row->recovery = electrical;
        row->recovery_reason = "collapsed-device terminal-cut attribution";
      }
    }
    for (const auto& flow : device_flows.ac_circuit_breakers) {
      if (auto* row = find_row(out, "circuit_breaker", "AC", flow.index)) {
        set_terminal(*row, "from", flow.bus_from, false, flow.pf_mw,
                     flow.qf_mvar, bus_voltage(rich, rich_pf, flow.bus_from, false),
                     true, true, true);
        set_terminal(*row, "to", flow.bus_to, false, flow.pt_mw,
                     flow.qt_mvar, bus_voltage(rich, rich_pf, flow.bus_to, false),
                     true, true, true);
        row->recovery = electrical;
        row->recovery_reason = "collapsed-device terminal-cut attribution";
      }
    }
  }

  // A source-side breaker may be authored as an inline Canvas device while
  // both electrical endpoints carry the same bus id (source terminal -> bus).
  // Its physical cut flow is the external-grid injection, not zero merely
  // because bus_from == bus_to after canonical contraction.
  if (opf_result) {
    std::unordered_map<int, int> breaker_count_by_bus;
    for (const auto& breaker : rich.ac.circuit_breakers) {
      if (!breaker.in_service || !breaker.closed ||
          breaker.bus_from != breaker.bus_to) continue;
      const bool has_grid = std::any_of(
          rich.ac.external_grids.begin(), rich.ac.external_grids.end(),
          [&](const auto& grid) {
            return grid.in_service && grid.bus == breaker.bus_from;
          });
      if (has_grid) ++breaker_count_by_bus[breaker.bus_from];
    }
    for (const auto& breaker : rich.ac.circuit_breakers) {
      const int count = breaker_count_by_bus[breaker.bus_from];
      if (!breaker.in_service || !breaker.closed ||
          breaker.bus_from != breaker.bus_to || count <= 0) continue;
      double p = 0.0;
      double q = 0.0;
      for (const auto& grid : rich.ac.external_grids) {
        if (!grid.in_service || grid.bus != breaker.bus_from) continue;
        const auto pos = position_for_index(
            projection.canonical.ac.external_grids, grid.index);
        if (!pos) continue;
        if (*pos < opf_result->external_grid_p_mw.size())
          p += opf_result->external_grid_p_mw[*pos];
        if (*pos < opf_result->external_grid_q_mvar.size())
          q += opf_result->external_grid_q_mvar[*pos];
      }
      p /= static_cast<double>(count);
      q /= static_cast<double>(count);
      if (auto* row = find_row(out, "circuit_breaker", "AC", breaker.index)) {
        const double vm = bus_voltage(rich, rich_pf, breaker.bus_from, false);
        set_terminal(*row, "from", breaker.bus_from, false, p, q, vm,
                     true, true, true);
        set_terminal(*row, "to", breaker.bus_to, false, -p, -q, vm,
                     true, true, true);
        row->recovery = RecoveryClass::Strong;
        row->recovery_reason =
            "same-bus source-breaker cut attributed from external-grid dispatch";
      }
    }
  }

  // OPF component maps contain canonical container positions. Translate those
  // positions to canonical component indices before matching rich identities.
  if (opf_result) {
    HybridPowerSystem canonical_components = projection.canonical;
    materialize_dc_storage(canonical_components);
    for (size_t k = 0;
         !options.prefer_rich_storage_dispatch &&
         k < opf_result->pstor_mw.size() && k < opf_result->stor_map.size();
         ++k) {
      const auto ref = opf_result->stor_map[k];
      const bool dc = ref.source_type == 1;
      const auto& storage = dc ? canonical_components.dc.storage
                               : canonical_components.ac.storage;
      if (ref.original_index < 0 ||
          ref.original_index >= static_cast<int>(storage.size())) continue;
      const int rich_index = storage[static_cast<size_t>(ref.original_index)].index;
      RichComponentResult* row = find_row(out, dc ? "dc_storage" : "storage",
                                          dc ? "DC" : "AC", rich_index);
      if (!row && dc) row = find_row(out, "storage", "DC", rich_index);
      if (!row) continue;
      set_value(*row, "p_mw", opf_result->pstor_mw[k], "MW");
      if (!dc && k < opf_result->qstor_mvar.size())
        set_value(*row, "q_mvar", opf_result->qstor_mvar[k], "MVar");
      if (!row->terminals.empty()) {
        row->terminals.front().p_mw = opf_result->pstor_mw[k];
        if (!dc && k < opf_result->qstor_mvar.size())
          row->terminals.front().q_mvar = opf_result->qstor_mvar[k];
      }
      row->recovery = RecoveryClass::Strong;
      row->recovery_reason = "OPF map translated canonical position to rich identity";
    }

    for (size_t k = 0; k < opf_result->pren_mw.size() &&
                       k < opf_result->ren_map.size(); ++k) {
      const auto ref = opf_result->ren_map[k];
      const bool pv = ref.source_type == 1;
      if (ref.original_index < 0) continue;
      int rich_index = -1;
      if (!pv && ref.original_index < static_cast<int>(
                     canonical_components.ac.renewable_gens.size()))
        rich_index = canonical_components.ac.renewable_gens[ref.original_index].index;
      if (pv && ref.original_index < static_cast<int>(
                    canonical_components.ac.pv_systems.size()))
        rich_index = canonical_components.ac.pv_systems[ref.original_index].index;
      if (rich_index < 0) continue;
      if (auto* row = find_row(out, pv ? "pv_system" : "renewable_generator",
                               "AC", rich_index)) {
        set_value(*row, "p_mw", opf_result->pren_mw[k], "MW");
        if (k < opf_result->qren_mvar.size())
          set_value(*row, "q_mvar", opf_result->qren_mvar[k], "MVar");
        row->recovery = RecoveryClass::Strong;
        row->recovery_reason = "OPF renewable map translated to rich identity";
      }
    }
  }

  out.coverage.rich_components = static_cast<int>(out.components.size());
  for (const auto& row : out.components) {
    switch (row.recovery) {
      case RecoveryClass::Strong:
        ++out.coverage.strong_components;
        ++out.coverage.attributed_components;
        break;
      case RecoveryClass::Approximate:
        ++out.coverage.approximate_components;
        ++out.coverage.attributed_components;
        break;
      case RecoveryClass::AuditOnly:
        ++out.coverage.audit_only_components;
        ++out.coverage.attributed_components;
        break;
      case RecoveryClass::Unsupported: ++out.coverage.unsupported_components; break;
    }
  }
  return out;
}

const char* recovery_class_name(RecoveryClass recovery) noexcept {
  switch (recovery) {
    case RecoveryClass::Strong: return "strong";
    case RecoveryClass::Approximate: return "approximate";
    case RecoveryClass::AuditOnly: return "audit_only";
    case RecoveryClass::Unsupported: return "unsupported";
  }
  return "unsupported";
}

}  // namespace hacdcpf::projection

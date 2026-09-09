#include "hacdcpf/projection/result_attribution.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <span>
#include <string_view>
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

// AUD-097: call-local indexes replace repeated linear identity scans. Build
// only after all result rows exist, and only for queried component families.
// Type names are literals owned by this translation unit; row order stays fixed.
class ResultRowIndex {
 public:
  explicit ResultRowIndex(RichResultAttribution& out) : out_(out) {}

  std::span<const size_t> matching(std::string_view type, int index) {
    const auto [family, inserted] = rows_.try_emplace(type);
    if (inserted) {
      for (size_t i = 0; i < out_.components.size(); ++i) {
        const auto& row = out_.components[i];
        if (row.component_type == type) family->second[row.component_index].push_back(i);
      }
    }
    const auto rows = family->second.find(index);
    return rows == family->second.end() ? std::span<const size_t>{} : rows->second;
  }

  RichComponentResult* find(std::string_view type, std::string_view domain,
                            int index) {
    for (const size_t position : matching(type, index)) {
      auto& row = out_.components[position];
      if (row.domain == domain) return &row;
    }
    return nullptr;
  }

 private:
  RichResultAttribution& out_;
  std::unordered_map<std::string_view,
                     std::unordered_map<int, std::vector<size_t>>> rows_;
};

class PositionIndex {
 public:
  template <typename Collection>
  explicit PositionIndex(const Collection& collection, bool needed = true) {
    if (!needed) return;
    positions_.reserve(collection.size());
    for (size_t i = 0; i < collection.size(); ++i)
      positions_.try_emplace(collection[i].index, i);
  }

  std::optional<size_t> find(int index) const {
    const auto it = positions_.find(index);
    return it == positions_.end() ? std::nullopt : std::optional<size_t>(it->second);
  }

 private:
  std::unordered_map<int, size_t> positions_;
};

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
    row.recovery = RecoveryClass::Unsupported;
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

class BusVoltages {
 public:
  BusVoltages(const HybridPowerSystem& rich, const PowerFlowResult* pf) {
    ac_.reserve(rich.ac.buses.size());
    dc_.reserve(rich.dc.buses.size());
    for (size_t i = 0; i < rich.ac.buses.size(); ++i)
      ac_.try_emplace(rich.ac.buses[i].index,
                      pf && i < pf->vm.size() ? pf->vm[i] : rich.ac.buses[i].vm_pu);
    for (size_t i = 0; i < rich.dc.buses.size(); ++i)
      dc_.try_emplace(rich.dc.buses[i].index,
                      pf && i < pf->vdc.size() ? pf->vdc[i] : rich.dc.buses[i].vm_pu);
  }

  double get(int bus, bool is_dc) const {
    const auto& domain = is_dc ? dc_ : ac_;
    const auto it = domain.find(bus);
    return it == domain.end() ? 0.0 : it->second;
  }

 private:
  std::unordered_map<int, double> ac_, dc_;
};

RecoveryClass electrical_recovery(const ProjectionBundle& projection) {
  return projection.canonical.projection_certificate &&
                 !projection.canonical.projection_certificate->exact()
             ? RecoveryClass::Approximate
             : RecoveryClass::Strong;
}

void attach_projection_sources(const ProjectionBundle& projection,
                               RichResultAttribution& out,
                               ResultRowIndex& rows) {
  if (!projection.canonical.projection_report) return;
  static const std::unordered_map<std::string_view, std::string_view> source_types{
      {"Transformer2W", "transformer_2w"}, {"Transformer3W", "transformer_3w"},
      {"Switch", "switch"}, {"CircuitBreaker", "circuit_breaker"},
      {"FlexibleLoad", "flexible_load"}, {"AsymmetricLoad", "asymmetric_load"},
      {"AsynchronousMotor", "motor"}, {"EnergyRouter", "energy_router"},
      {"VirtualPowerPlant", "vpp"}, {"Microgrid", "microgrid"},
      {"MobileStorage", "mobile_storage"}};
  for (const auto& mapping : projection.canonical.projection_report->mappings) {
    const auto type = source_types.find(mapping.source_type);
    if (type == source_types.end()) continue;
    const int source_index = parse_component_index(mapping.source_id);
    const int canonical_index = parse_component_index(mapping.canonical_id);
    // ProjectionReport has no source domain. Preserve its existing all-match
    // provenance semantics; electrical row lookup still requires a domain.
    for (const size_t position : rows.matching(type->second, source_index)) {
      auto& row = out.components[position];
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
  const PositionIndex canonical_buses(projection.canonical.ac.buses);
  for (size_t i = 0; i < rich.ac.buses.size(); ++i)
    if (const auto position = canonical_buses.find(rich.ac.buses[i].index))
      positions[i] = static_cast<int>(*position);
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
  const BusVoltages voltages(rich, rich_pf);
  const PositionIndex generators(projection.canonical.ac.generators, opf_result != nullptr);
  const PositionIndex grids(projection.canonical.ac.external_grids, opf_result != nullptr);

  add_collection(out, rich.ac.buses, "ac_bus", "AC", "AC Bus",
                 [&](auto& row, const auto& bus) {
                   const double vm = voltages.get(bus.index, false);
                   set_value(row, "vm_pu", vm, "pu");
                   if (rich_pf && row.position < static_cast<int>(rich_pf->va.size()))
                     set_value(row, "va_rad", rich_pf->va[row.position], "rad");
                   row.recovery = electrical;
                   row.recovery_reason = "BusMergeMap voltage broadcast";
                 });
  add_collection(out, rich.dc.buses, "dc_bus", "DC", "DC Bus",
                 [&](auto& row, const auto& bus) {
                   set_value(row, "vdc_pu",
                             voltages.get(bus.index, true), "pu");
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
                                  voltages.get(branch.from_bus, false),
                                  true, true, true);
                     set_terminal(row, "to", branch.to_bus, false, flow.pt_mw,
                                  flow.qt_mvar,
                                  voltages.get(branch.to_bus, false),
                                  true, true, true);
                     set_value(row, "loss_mw", flow.pf_mw + flow.pt_mw, "MW");
                     row.recovery = electrical;
                     row.recovery_reason = "original branch position map";
                   }
                 });
  add_collection(out, rich.dc.branches, "dc_branch", "DC", "DC Line",
                 [&](auto& row, const auto& branch) {
                   const double vf = voltages.get(branch.from_bus, true);
                   const double vt = voltages.get(branch.to_bus, true);
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
  add_collection(out, rich.dc.capacitors, "dc_capacitor", "DC",
                 "DC Capacitor", [](auto& row, const auto&) {
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable DC capacitor identity";
                 });
  add_collection(out, rich.dc.reactors, "dc_reactor", "DC",
                 "DC Reactor", [](auto& row, const auto&) {
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable DC reactor identity";
                 });
  add_collection(out, rich.ac.harmonic_filters, "harmonic_filter", "AC",
                 "AC Harmonic Filter", [](auto& row, const auto&) {
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable AC harmonic-filter identity";
                 });
  add_collection(out, rich.dc.harmonic_filters, "harmonic_filter", "DC",
                 "DC Harmonic Filter", [](auto& row, const auto&) {
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable DC harmonic-filter identity";
                 });

  add_collection(out, rich.ac.generators, "generator", "AC", "Generator",
                 [&](auto& row, const auto& generator) {
                   const auto canonical_position = generators.find(generator.index);
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
                                voltages.get(generator.bus, false),
                                true, true, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "stable generator identity";
                 });
  add_collection(out, rich.ac.external_grids, "external_grid", "AC",
                 "External Grid", [&](auto& row, const auto& grid) {
                   const auto canonical_position = grids.find(grid.index);
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
                                voltages.get(grid.bus, false),
                                true, true, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "explicit external-grid OPF variable";
                 });

  add_collection(out, rich.ac.loads, "load", "AC", "Load",
                 [&](auto& row, const auto& load) {
                   const double p =
                       load.in_service ? load.p_mw * load.scaling : 0.0;
                   const double q =
                       load.in_service ? load.q_mvar * load.scaling : 0.0;
                   set_value(row, "p_mw", p, "MW");
                   set_value(row, "q_mvar", q, "MVar");
                   set_terminal(row, "ac", load.bus, false, -p, -q,
                                voltages.get(load.bus, false),
                                true, true, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "authored load identity and time-step demand";
                 });
  add_collection(out, rich.dc.loads, "dc_load", "DC", "DC Load",
                 [&](auto& row, const auto& load) {
                   const double p =
                       load.in_service ? load.p_mw * load.scaling : 0.0;
                   set_value(row, "p_mw", p, "MW");
                   set_terminal(row, "dc", load.bus, true, -p, 0.0,
                                voltages.get(load.bus, true),
                                true, false, true);
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason = "authored DC load identity and time-step demand";
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
                                voltages.get(storage.bus, false),
                                true, true, true);
                   row.canonical_sources.push_back(
                       {"Storage", storage.index, 1.0});
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason =
                       "stable AC storage identity and time-step dispatch";
                 });
  add_collection(out, rich.dc.storage, "storage", "DC", "DC ESS",
                 [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "soc", storage.soc_init);
                   set_terminal(row, "dc", storage.bus, true, storage.p_mw,
                                0.0,
                                voltages.get(storage.bus, true),
                                true, false, true);
                   row.canonical_sources.push_back(
                       {"DCStorage", storage.index, 1.0});
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason =
                       "stable DC storage identity and time-step dispatch";
                 });
  add_collection(out, rich.dc.dc_storage, "dc_storage", "DC", "DC ESS",
                 [&](auto& row, const auto& storage) {
                   set_value(row, "p_mw", storage.p_mw, "MW");
                   set_value(row, "soc", storage.soc_init);
                   set_terminal(row, "dc", storage.bus, true, storage.p_mw,
                                0.0,
                                voltages.get(storage.bus, true),
                                true, false, true);
                   row.canonical_sources.push_back(
                       {"DCStorage", storage.index, 1.0});
                   row.recovery = RecoveryClass::Strong;
                   row.recovery_reason =
                       "stable native DC storage identity and time-step dispatch";
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
                   const double scaling =
                       std::isfinite(load.scaling)
                           ? std::max(0.0, load.scaling)
                           : 0.0;
                   set_value(row, "p_mw",
                             scaling * (load.pa_mw + load.pb_mw + load.pc_mw),
                             "MW");
                   set_value(row, "q_mvar",
                             scaling * (load.qa_mvar + load.qb_mvar +
                                        load.qc_mvar),
                             "MVar");
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
                                    voltages.get(converter.bus_ac, false),
                                    true, true, true);
                       set_terminal(row, "dc", converter.bus_dc, true,
                                    it->p_dc_mw, 0.0,
                                    voltages.get(converter.bus_dc, true),
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
                                    voltages.get(converter.bus_in, true),
                                    true, false, true);
                       set_terminal(row, "out", converter.bus_out, true,
                                    it->p_out_mw, 0.0,
                                    voltages.get(converter.bus_out, true),
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

  ResultRowIndex rows(out);
  attach_projection_sources(projection, out, rows);

  // Attribute expanded AC branch terminal powers to rich transformers and
  // non-collapsed switchgear using BranchExpandMap, never vector position.
  if (canonical_pf && projection.canonical.branch_expand_map) {
    const PositionIndex transformers_2w(rich.ac.transformers_2w);
    const PositionIndex transformers_3w(rich.ac.transformers_3w);
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
      RichComponentResult* row = rows.find(type, "AC", entry.origin_index);
      if (!row) continue;
      if (entry.origin_type == BranchOriginType::Transformer3W) {
        const auto position = transformers_3w.find(entry.origin_index);
        if (position) {
          const auto* transformer = &rich.ac.transformers_3w[*position];
          const int from_bus = entry.pair_number < 2 ? transformer->hv_bus
                                                     : transformer->mv_bus;
          const int to_bus = entry.pair_number == 0 ? transformer->mv_bus
                            : entry.pair_number == 1 ? transformer->lv_bus
                                                     : transformer->lv_bus;
          const std::string from_name = entry.pair_number < 2 ? "hv" : "mv";
          const std::string to_name = entry.pair_number == 0 ? "mv" : "lv";
          accumulate_terminal(*row, from_name, from_bus, flow.pf_mw,
                              flow.qf_mvar,
                              voltages.get(from_bus, false));
          accumulate_terminal(*row, to_name, to_bus, flow.pt_mw,
                              flow.qt_mvar,
                              voltages.get(to_bus, false));
        }
      } else {
        int from_bus = entry.bus_from;
        int to_bus = entry.bus_to;
        if (entry.origin_type == BranchOriginType::Transformer2W) {
          const auto position = transformers_2w.find(entry.origin_index);
          if (position) {
            const auto* transformer = &rich.ac.transformers_2w[*position];
            from_bus = transformer->hv_bus;
            to_bus = transformer->lv_bus;
          }
        }
        set_terminal(*row, "from", from_bus, false, flow.pf_mw,
                     flow.qf_mvar, voltages.get(from_bus, false),
                     true, true, from_bus >= 0);
        set_terminal(*row, "to", to_bus, false, flow.pt_mw,
                     flow.qt_mvar, voltages.get(to_bus, false),
                     true, true, to_bus >= 0);
      }
      row->recovery = electrical;
      row->recovery_reason = "BranchExpandMap terminal attribution";
    }
  }

  // An authored line can become a self-loop when a parallel ideal device
  // merges its endpoint buses. Projection transfers the line's total charging
  // susceptance to the merged bus, so recover that observable at the original
  // terminals even though the series circulating flow is not identifiable.
  if (rich_pf && projection.canonical.bus_merge_map) {
    const auto& map = *projection.canonical.bus_merge_map;
    const double base_mva = rich.base_mva > 0.0
                                ? rich.base_mva
                                : (rich.ac.base_mva > 0.0 ? rich.ac.base_mva
                                                         : 100.0);
    for (size_t pos = 0; pos < rich.ac.branches.size(); ++pos) {
      const auto projected = map.branch_orig_to_proj.find(static_cast<int>(pos));
      if (projected == map.branch_orig_to_proj.end() || projected->second >= 0) {
        continue;
      }
      const auto& branch = rich.ac.branches[pos];
      const auto from = map.ext_to_int.find(branch.from_bus);
      const auto to = map.ext_to_int.find(branch.to_bus);
      if (!branch.in_service || from == map.ext_to_int.end() ||
          to == map.ext_to_int.end() || from->second != to->second) {
        continue;
      }
      RichComponentResult* row =
          rows.find("ac_branch", "AC", branch.index);
      if (!row) continue;
      const double vf = voltages.get(branch.from_bus, false);
      const double vt = voltages.get(branch.to_bus, false);
      const double qf = -0.5 * branch.b_pu * base_mva * vf * vf;
      const double qt = -0.5 * branch.b_pu * base_mva * vt * vt;
      set_terminal(*row, "from", branch.from_bus, false, 0.0, qf, vf,
                   true, true, true);
      set_terminal(*row, "to", branch.to_bus, false, 0.0, qt, vt,
                   true, true, true);
      set_value(*row, "loss_mw", 0.0, "MW");
      row->recovery = RecoveryClass::AuditOnly;
      row->recovery_reason =
          "merged self-loop charging attribution; series flow is unidentifiable";
    }
  }

  // Collapsed ideal switchgear has no canonical branch row. Recover its flow
  // from the recorded terminal cut in original topology.
  if (rich_pf) {
    DeviceTerminalFlows device_flows =
        compute_device_terminal_flows(rich, rich_pf->branch_flows);
    for (const auto& flow : device_flows.ac_switches) {
      if (auto* row = rows.find("switch", "AC", flow.index)) {
        set_terminal(*row, "from", flow.bus_from, false, flow.pf_mw,
                     flow.qf_mvar, voltages.get(flow.bus_from, false),
                     true, true, true);
        set_terminal(*row, "to", flow.bus_to, false, flow.pt_mw,
                     flow.qt_mvar, voltages.get(flow.bus_to, false),
                     true, true, true);
        row->recovery = flow.bus_from == flow.bus_to
                            ? RecoveryClass::AuditOnly
                            : electrical;
        row->recovery_reason = flow.bus_from == flow.bus_to
                                   ? "same-bus PF does not identify a physical cut flow"
                                   : "collapsed-device terminal-cut attribution";
      }
    }
    for (const auto& flow : device_flows.ac_circuit_breakers) {
      if (auto* row = rows.find("circuit_breaker", "AC", flow.index)) {
        set_terminal(*row, "from", flow.bus_from, false, flow.pf_mw,
                     flow.qf_mvar, voltages.get(flow.bus_from, false),
                     true, true, true);
        set_terminal(*row, "to", flow.bus_to, false, flow.pt_mw,
                     flow.qt_mvar, voltages.get(flow.bus_to, false),
                     true, true, true);
        row->recovery = flow.bus_from == flow.bus_to
                            ? RecoveryClass::AuditOnly
                            : electrical;
        row->recovery_reason = flow.bus_from == flow.bus_to
                                   ? "same-bus PF does not identify a physical cut flow"
                                   : "collapsed-device terminal-cut attribution";
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
        const auto pos = grids.find(grid.index);
        if (!pos) continue;
        if (*pos < opf_result->external_grid_p_mw.size())
          p += opf_result->external_grid_p_mw[*pos];
        if (*pos < opf_result->external_grid_q_mvar.size())
          q += opf_result->external_grid_q_mvar[*pos];
      }
      p /= static_cast<double>(count);
      q /= static_cast<double>(count);
      if (auto* row = rows.find("circuit_breaker", "AC", breaker.index)) {
        const double vm = voltages.get(breaker.bus_from, false);
        set_terminal(*row, "from", breaker.bus_from, false, p, q, vm,
                     true, true, true);
        set_terminal(*row, "to", breaker.bus_to, false, -p, -q, vm,
                     true, true, true);
        row->recovery = count == 1 ? RecoveryClass::Strong
                                   : RecoveryClass::Approximate;
        row->recovery_reason =
            count == 1
                ? "same-bus source-breaker cut attributed from external-grid dispatch"
                : "multiple same-bus source breakers use an equal-share convention";
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
      RichComponentResult* row = rows.find(dc ? "dc_storage" : "storage",
                                          dc ? "DC" : "AC", rich_index);
      if (!row && dc) row = rows.find("storage", "DC", rich_index);
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
      if (auto* row = rows.find(pv ? "pv_system" : "renewable_generator",
                               "AC", rich_index)) {
        set_value(*row, "p_mw", opf_result->pren_mw[k], "MW");
        if (k < opf_result->qren_mvar.size())
          set_value(*row, "q_mvar", opf_result->qren_mvar[k], "MVar");
        row->recovery = RecoveryClass::Strong;
        row->recovery_reason = "OPF renewable map translated to rich identity";
      }
    }
  }

  // An out-of-service component cannot carry an operating-point observable.
  // Preserve its row/identity while forcing every numeric result and terminal
  // flow to zero across all component types.
  for (auto& row : out.components) {
    if (row.in_service) continue;
    for (auto& value : row.values) value.value = 0.0;
    for (auto& terminal : row.terminals) {
      terminal.p_mw = 0.0;
      terminal.q_mvar = 0.0;
      terminal.v_pu = 0.0;
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

// Carbon Flow Analysis — Proportional Tracing (BFS) + Matrix-based solve
// Extended to the canonical hybrid AC/DC network.
// Ported from luosipeng/HybridACDCPowerSystemsPlanning (luosipeng branch).

#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/detail/logging.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/model/enums/grid_enums.hpp"
#include "hacdcpf/model/enums/storage_enums.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"
#include "hacdcpf/projection/result_attribution.hpp"

namespace hacdcpf::analysis {

namespace {

constexpr double kTol = 1e-9;

double carbon_intensity_at(const std::vector<double>& w, int node_loc) {
  if (node_loc < 0 || node_loc >= static_cast<int>(w.size())) return 0.0;
  return w[static_cast<size_t>(node_loc)];
}

double clamped_loss_alpha(double alpha) {
  return std::clamp(alpha, 0.0, 1.0);
}

enum class EdgeKind {
  ACBranch,
  ACSwitch,
  ACCircuitBreaker,
  DCBranch,
  VSC,
  DCDC,
  EnergyRouter,
};

struct Source {
  int id{0};
  int bus{0};
  bool is_dc{false};
  int node_loc{-1};
  double p_mw{0.0};
  double ef{0.0};
  bool is_balancing{false};
  bool can_absorb_export{false};
  std::string source_type;
  int component_index{0};
  std::string label;
  double scheduled_p_mw{0.0};
};

struct HybridIDMap {
  int n_ac{0};
  int n_dc{0};
  std::unordered_map<int, int> ac_loc;
  std::unordered_map<int, int> dc_loc;
  std::unordered_map<int, int> ac_global;
  std::unordered_map<int, int> dc_global;
};

struct UnifiedLoad {
  int index{0};
  int bus{0};
  bool is_dc{false};
  int node_loc{-1};
  double p_mw{0.0};
  double q_mvar{0.0};
  bool report_as_load{true};
  bool is_network_loss{false};
  bool is_external_export{false};
};

struct StorageState {
  int index{0};
  int bus{0};
  bool is_dc{false};
  int node_loc{-1};
  bool in_service{false};
  double p_mw{0.0};
  double soc{0.0};
  double e_rated_mwh{0.0};
  double soc_carbon_intensity_tco2_mwh{0.0};
};

struct StorageLoadBuildResult {
  std::vector<UnifiedLoad> loads;
  std::vector<int> storage_pos_for_load;
  std::vector<int> load_pos_for_storage;
};

struct RouterPortState {
  int router_index{0};
  int port_index{0};
  int bus{0};
  bool is_dc{false};
  int node_loc{-1};
  double p_mw{0.0};
};

struct FlowEdge {
  EdgeKind kind{EdgeKind::ACBranch};
  int component_index{0};
  int from_node_loc{-1};
  int to_node_loc{-1};
  int from_bus{0};
  int to_bus{0};
  bool from_is_dc{false};
  bool to_is_dc{false};
  double send_mw{0.0};
  double recv_mw{0.0};
  double loss_mw{0.0};
};

double edge_loss_carbon_intensity(const FlowEdge& e,
                                  const std::vector<double>& w,
                                  double loss_allocation_alpha) {
  if (e.loss_mw <= kTol) return 0.0;
  const double alpha = clamped_loss_alpha(loss_allocation_alpha);
  const double sender = carbon_intensity_at(w, e.from_node_loc);
  const double receiver = carbon_intensity_at(w, e.to_node_loc);
  return alpha * sender + (1.0 - alpha) * receiver;
}

const FlowEdge* find_edge_by_component(const std::vector<FlowEdge>& edges,
                                       EdgeKind kind,
                                       int component_index) {
  for (const auto& e : edges) {
    if (e.kind == kind && e.component_index == component_index) return &e;
  }
  return nullptr;
}

struct SourceBuildResult {
  std::vector<Source> sources;
  int slack_source_pos{-1};
};

struct Upstream {
  int from_node_loc{-1};
  int edge_loc{-1};
  double recv_mw{0.0};
};

struct TracingResult {
  std::vector<std::unordered_map<int, double>> load_supply;
  std::vector<std::unordered_map<int, double>> edge_loss_alloc;
  std::unordered_map<int, double> gen_loss_total;
};

struct MatrixResult {
  std::vector<double> w;
  double residual{0.0};
  double relative_residual{0.0};
  double condition_estimate{0.0};
  int rank{0};
  bool solved{false};
};

struct PowerBalanceDiagnostics {
  double max_abs_error_mw{0.0};
  double total_abs_error_mw{0.0};
  double tolerance_mw{0.0};
  bool verified{false};
  std::vector<NodePowerBalanceError> errors;
};

struct CanonicalCarbonInput {
  HybridPowerSystem system;
  PowerFlowResult power_flow;
  std::optional<BusMergeMap> ac_bus_map;
  std::vector<int> dc_canonical_to_original;
  std::vector<int> original_branch_to_canonical;
};

long long router_port_key(int router_index, int port_index) {
  return (static_cast<long long>(router_index) << 32) |
         static_cast<unsigned int>(port_index);
}

HybridIDMap build_id_map(const HybridPowerSystem& sys) {
  HybridIDMap id;
  id.n_ac = static_cast<int>(sys.ac.buses.size());
  id.n_dc = static_cast<int>(sys.dc.buses.size());

  for (int i = 0; i < id.n_ac; ++i) {
    const int bus = sys.ac.buses[static_cast<size_t>(i)].index;
    id.ac_loc[bus] = i;
    id.ac_global[bus] = i;
  }
  for (int i = 0; i < id.n_dc; ++i) {
    const int bus = sys.dc.buses[static_cast<size_t>(i)].index;
    id.dc_loc[bus] = i;
    id.dc_global[bus] = id.n_ac + i;
  }
  return id;
}

int global_bus_loc(const HybridIDMap& id, bool is_dc, int bus) {
  const auto& map = is_dc ? id.dc_global : id.ac_global;
  auto it = map.find(bus);
  return (it == map.end()) ? -1 : it->second;
}

std::vector<double> project_ac_bus_values(
    const HybridPowerSystem& original,
    const HybridPowerSystem& canonical,
    const std::vector<double>& original_values,
    const std::optional<BusMergeMap>& map,
    double default_value) {
  std::vector<double> values(canonical.ac.buses.size(), default_value);
  std::vector<bool> assigned(canonical.ac.buses.size(), false);

  if (map.has_value()) {
    for (const auto& [external_bus, internal_pos] : map->ext_to_int) {
      if (internal_pos < 0 || internal_pos >= static_cast<int>(values.size())) continue;
      const auto original_pos_it = map->ext_to_orig_pos.find(external_bus);
      if (original_pos_it == map->ext_to_orig_pos.end()) continue;
      const int original_pos = original_pos_it->second;
      if (original_pos < 0 || original_pos >= static_cast<int>(original_values.size())) continue;
      values[static_cast<size_t>(internal_pos)] =
          original_values[static_cast<size_t>(original_pos)];
      assigned[static_cast<size_t>(internal_pos)] = true;
    }
  }

  std::unordered_map<int, size_t> original_pos_by_bus;
  original_pos_by_bus.reserve(original.ac.buses.size());
  for (size_t i = 0; i < original.ac.buses.size(); ++i) {
    original_pos_by_bus.emplace(original.ac.buses[i].index, i);
  }
  for (size_t i = 0; i < canonical.ac.buses.size(); ++i) {
    if (assigned[i]) continue;
    const auto it = original_pos_by_bus.find(canonical.ac.buses[i].index);
    if (it != original_pos_by_bus.end() && it->second < original_values.size()) {
      values[i] = original_values[it->second];
      assigned[i] = true;
    } else if (i < original_values.size()) {
      values[i] = original_values[i];
    }
  }
  return values;
}

CanonicalCarbonInput build_canonical_carbon_input(
    const HybridPowerSystem& original,
    const PowerFlowResult& original_pf) {
  CanonicalCarbonInput input;
  input.system =
      projection::RichToCanonicalOperator::apply(original).canonical;
  input.ac_bus_map = input.system.bus_merge_map;
  input.dc_canonical_to_original.reserve(original.dc.buses.size());
  for (const auto& bus : original.dc.buses) {
    input.dc_canonical_to_original.push_back(bus.index);
  }
  input.power_flow = original_pf;
  input.power_flow.vm = project_ac_bus_values(
      original, input.system, original_pf.vm, input.ac_bus_map, 1.0);
  input.power_flow.va = project_ac_bus_values(
      original, input.system, original_pf.va, input.ac_bus_map, 0.0);

  input.power_flow.vdc.resize(input.system.dc.buses.size(), 1.0);

  input.power_flow.branch_flows.assign(input.system.ac.branches.size(), {});
  if (original_pf.canonical_branch_flows.size() ==
      input.system.ac.branches.size()) {
    input.power_flow.branch_flows = original_pf.canonical_branch_flows;
  }

  input.original_branch_to_canonical.assign(original.ac.branches.size(), -1);
  for (size_t original_pos = 0; original_pos < original.ac.branches.size(); ++original_pos) {
    int canonical_pos = static_cast<int>(original_pos);
    if (input.ac_bus_map.has_value()) {
      const auto map_it = input.ac_bus_map->branch_orig_to_proj.find(
          static_cast<int>(original_pos));
      canonical_pos = (map_it == input.ac_bus_map->branch_orig_to_proj.end())
                          ? -1
                          : map_it->second;
    }
    input.original_branch_to_canonical[original_pos] = canonical_pos;
    if (canonical_pos < 0 ||
        canonical_pos >= static_cast<int>(input.power_flow.branch_flows.size()) ||
        original_pos >= original_pf.branch_flows.size()) {
      continue;
    }
    input.power_flow.branch_flows[static_cast<size_t>(canonical_pos)] =
        original_pf.branch_flows[original_pos];
  }

  std::unordered_map<int, std::pair<int, int>> vsc_buses;
  vsc_buses.reserve(input.system.vsc_converters.size());
  for (const auto& converter : input.system.vsc_converters) {
    vsc_buses[converter.index] = {converter.bus_ac, converter.bus_dc};
  }
  for (auto& transfer : input.power_flow.vsc_transfers) {
    const auto it = vsc_buses.find(transfer.index);
    if (it == vsc_buses.end()) continue;
    transfer.bus_ac = it->second.first;
    transfer.bus_dc = it->second.second;
  }

  std::unordered_map<int, std::pair<int, int>> dcdc_buses;
  dcdc_buses.reserve(input.system.dc.dcdc_converters.size());
  for (const auto& converter : input.system.dc.dcdc_converters) {
    dcdc_buses[converter.index] = {converter.bus_in, converter.bus_out};
  }
  for (auto& transfer : input.power_flow.dcdc_transfers) {
    const auto it = dcdc_buses.find(transfer.index);
    if (it == dcdc_buses.end()) continue;
    transfer.bus_in = it->second.first;
    transfer.bus_out = it->second.second;
  }

  if (!original.ac.switches.empty() || !original.ac.circuit_breakers.empty()) {
    input.power_flow.ac_switch_flows.clear();
    input.power_flow.ac_circuit_breaker_flows.clear();
  }
  return input;
}

double ac_voltage_magnitude(const HybridIDMap& id,
                            const PowerFlowResult& pf,
                            int bus) {
  const auto it = id.ac_loc.find(bus);
  if (it == id.ac_loc.end() || it->second < 0 ||
      it->second >= static_cast<int>(pf.vm.size())) {
    return 1.0;
  }
  return std::max(pf.vm[static_cast<size_t>(it->second)], 0.0);
}

double zip_active_power_mw(const Load& load, double vm_pu) {
  const double nominal = load.p_mw * std::max(load.scaling, 0.0);
  const double constant_power = load.p_percent_p / 100.0;
  const double constant_current = load.i_percent_p / 100.0;
  const double constant_impedance = load.z_percent_p / 100.0;
  const double multiplier = constant_power + constant_current * vm_pu +
                            constant_impedance * vm_pu * vm_pu;
  return nominal * multiplier;
}

std::vector<UnifiedLoad> build_unified_ac_loads(const ACSystem& ac,
                                                const HybridIDMap& id,
                                                const PowerFlowResult& pf) {
  std::vector<UnifiedLoad> out;

  for (const auto& ld : ac.loads) {
    const double vm_pu = ac_voltage_magnitude(id, pf, ld.bus);
    const double p_mw = zip_active_power_mw(ld, vm_pu);
    const double q_nominal = ld.q_mvar * std::max(ld.scaling, 0.0);
    const double q_multiplier = ld.p_percent_q / 100.0 +
                                (ld.i_percent_q / 100.0) * vm_pu +
                                (ld.z_percent_q / 100.0) * vm_pu * vm_pu;
    const double q_mvar = q_nominal * q_multiplier;
    if (!ld.in_service || p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, ld.bus);
    if (node_loc < 0) continue;
    out.push_back({ld.index, ld.bus, false, node_loc, p_mw, q_mvar});
  }

  int synthetic_load_id = -100000;
  for (const auto& cs : ac.charging_stations) {
    if (!cs.in_service || cs.p_total_kw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, cs.bus);
    if (node_loc < 0) continue;
    out.push_back({synthetic_load_id--,
                   cs.bus,
                   false,
                   node_loc,
                   cs.p_total_kw / 1000.0,
                   cs.q_total_kvar / 1000.0});
  }

  int syn_id = -1;
  for (const auto& b : ac.buses) {
    if (b.pd_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, b.index);
    if (node_loc < 0) continue;
    out.push_back({syn_id--, b.index, false, node_loc, b.pd_mw, b.qd_mvar});
  }

  int shunt_sink_id = -200000;
  for (const auto& bus : ac.buses) {
    const double vm_pu = ac_voltage_magnitude(id, pf, bus.index);
    const double p_mw = std::max(bus.gs_mw, 0.0) * vm_pu * vm_pu;
    if (!bus.in_service || p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, bus.index);
    if (node_loc < 0) continue;
    out.push_back({shunt_sink_id--,
                   bus.index,
                   false,
                   node_loc,
                   p_mw,
                   0.0,
                   false,
                   true});
  }
  for (const auto& shunt : ac.shunts) {
    const double vm_pu = ac_voltage_magnitude(id, pf, shunt.bus);
    const double p_mw = std::max(shunt.gs_mw, 0.0) * vm_pu * vm_pu;
    if (!shunt.in_service || p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, shunt.bus);
    if (node_loc < 0) continue;
    out.push_back({shunt_sink_id--,
                   shunt.bus,
                   false,
                   node_loc,
                   p_mw,
                   0.0,
                   false,
                   true});
  }

  int signed_device_sink_id = -400000;
  std::unordered_set<int> slack_buses;
  for (const auto& bus : ac.buses) {
    if (bus.in_service && bus.bus_type == BusType::SLACK) {
      slack_buses.insert(bus.index);
    }
  }
  const auto add_negative_injection = [&](int bus, double p_mw) {
    if (p_mw >= -kTol) return;
    const int node_loc = global_bus_loc(id, false, bus);
    if (node_loc < 0) return;
    out.push_back(
        {signed_device_sink_id--, bus, false, node_loc, -p_mw, 0.0});
  };
  for (const auto& gen : ac.generators) {
    const bool is_balancing =
        gen.is_slack || slack_buses.count(gen.bus) != 0;
    if (gen.in_service && !is_balancing) {
      add_negative_injection(gen.bus, gen.pg_mw);
    }
  }
  for (const auto& gen : ac.static_generators) {
    if (gen.in_service) {
      add_negative_injection(gen.bus, gen.p_mw * gen.scaling);
    }
  }
  for (const auto& gen : ac.renewable_gens) {
    if (gen.in_service) add_negative_injection(gen.bus, gen.p_mw);
  }
  for (const auto& gen : ac.pv_systems) {
    if (gen.in_service) {
      add_negative_injection(gen.bus, powerflow::compute_pv_power_mw(gen));
    }
  }

  return out;
}

std::vector<UnifiedLoad> build_unified_dc_loads(const DCSystem& dc,
                                                const HybridIDMap& id) {
  std::vector<UnifiedLoad> out;
  for (const auto& ld : dc.loads) {
    const double p_mw = model::effective_load_p_mw(ld);
    if (!ld.in_service || p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, true, ld.bus);
    if (node_loc < 0) continue;
    out.push_back({ld.index, ld.bus, true, node_loc, p_mw, 0.0});
  }
  int syn_id = -1;
  for (const auto& b : dc.buses) {
    if (!b.in_service || b.pd_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, true, b.index);
    if (node_loc < 0) continue;
    out.push_back({syn_id--, b.index, true, node_loc, b.pd_mw, 0.0});
  }

  int signed_device_sink_id = -500000;
  const auto add_negative_injection = [&](int bus, double p_mw) {
    if (p_mw >= -kTol) return;
    const int node_loc = global_bus_loc(id, true, bus);
    if (node_loc < 0) return;
    out.push_back(
        {signed_device_sink_id--, bus, true, node_loc, -p_mw, 0.0});
  };
  for (const auto& gen : dc.static_generators) {
    if (gen.in_service) {
      add_negative_injection(gen.bus, gen.p_mw * gen.scaling);
    }
  }
  for (const auto& gen : dc.dc_static_generators) {
    if (gen.in_service) {
      add_negative_injection(gen.bus, gen.p_set_mw * gen.scaling);
    }
  }
  for (const auto& pv : dc.pv_arrays) {
    if (pv.in_service) add_negative_injection(pv.bus, pv.p_set_mw);
  }

  return out;
}

std::vector<StorageState> build_storage_states(const HybridPowerSystem& sys,
                                               const HybridIDMap& id) {
  std::vector<StorageState> out;
  out.reserve(sys.ac.storage.size() + sys.dc.storage.size());

  for (const auto& st : sys.ac.storage) {
    out.push_back({st.index,
                   st.bus,
                   false,
                   global_bus_loc(id, false, st.bus),
                   st.in_service,
                   st.p_mw,
                   st.soc_init,
                   st.e_rated_mwh,
                   st.soc_carbon_intensity_tco2_mwh});
  }
  for (const auto& st : sys.dc.storage) {
    out.push_back({st.index,
                   st.bus,
                   true,
                   global_bus_loc(id, true, st.bus),
                   st.in_service,
                   st.p_mw,
                   st.soc_init,
                   st.e_rated_mwh,
                   st.soc_carbon_intensity_tco2_mwh});
  }

  return out;
}

StorageLoadBuildResult build_storage_charging_loads(
    const std::vector<StorageState>& storage_states) {
  StorageLoadBuildResult out;
  out.load_pos_for_storage.assign(storage_states.size(), -1);

  for (size_t si = 0; si < storage_states.size(); ++si) {
    const auto& st = storage_states[si];
    if (!st.in_service || st.node_loc < 0 || st.p_mw >= -kTol) continue;
    out.load_pos_for_storage[si] = static_cast<int>(out.loads.size());
    out.storage_pos_for_load.push_back(static_cast<int>(si));
    out.loads.push_back({st.index, st.bus, st.is_dc, st.node_loc, -st.p_mw, 0.0});
  }

  return out;
}

std::unordered_map<int, std::vector<RouterPortState>> build_energy_router_port_states(
    const HybridPowerSystem& sys,
    const PowerFlowResult& pf,
    const HybridIDMap& id) {
  std::unordered_map<int, std::vector<RouterPortState>> by_router;
  by_router.reserve(sys.energy_routers.size());

  std::unordered_map<long long, ERPortTransfer> pf_port_by_key;
  pf_port_by_key.reserve(pf.er_port_transfers.size());
  for (const auto& tr : pf.er_port_transfers) {
    pf_port_by_key.emplace(router_port_key(tr.router_index, tr.port_index), tr);
  }

  for (const auto& er : sys.energy_routers) {
    auto& ports = by_router[er.index];
    ports.reserve(er.ports.size());
    for (const auto& p : er.ports) {
      if (!er.in_service || !p.in_service) continue;
      const auto pf_it = pf_port_by_key.find(router_port_key(er.index, p.index));
      const bool is_dc = (pf_it != pf_port_by_key.end()) ? !pf_it->second.is_ac
                                                         : (p.port_type == ERPortType::DC);
      const int bus = (pf_it != pf_port_by_key.end()) ? pf_it->second.bus : p.bus;
      const double p_mw = (pf_it != pf_port_by_key.end()) ? pf_it->second.p_mw : p.p_mw;
      ports.push_back(
          {er.index, p.index, bus, is_dc, global_bus_loc(id, is_dc, bus), p_mw});
    }
  }

  return by_router;
}

std::vector<FlowEdge> build_directed_flows(const HybridPowerSystem& sys,
                                           const PowerFlowResult& pf,
                                           const HybridIDMap& id) {
  std::vector<FlowEdge> edges;
  edges.reserve(sys.ac.branches.size() + sys.dc.branches.size() +
                pf.vsc_transfers.size() + pf.dcdc_transfers.size() +
                pf.er_port_transfers.size());

  for (size_t l = 0; l < sys.ac.branches.size() && l < pf.branch_flows.size(); ++l) {
    const auto& br = sys.ac.branches[l];
    const auto& bf = pf.branch_flows[l];
    const double loss = std::max(bf.pf_mw + bf.pt_mw, 0.0);
    const int from_loc = global_bus_loc(id, false, br.from_bus);
    const int to_loc = global_bus_loc(id, false, br.to_bus);
    if (from_loc < 0 || to_loc < 0) continue;

    if (bf.pf_mw > kTol) {
      edges.push_back({EdgeKind::ACBranch,
                       br.index,
                       from_loc,
                       to_loc,
                       br.from_bus,
                       br.to_bus,
                       false,
                       false,
                       bf.pf_mw,
                       std::max(-bf.pt_mw, 0.0),
                       loss});
    } else if (bf.pt_mw > kTol) {
      edges.push_back({EdgeKind::ACBranch,
                       br.index,
                       to_loc,
                       from_loc,
                       br.to_bus,
                       br.from_bus,
                       false,
                       false,
                       bf.pt_mw,
                       std::max(-bf.pf_mw, 0.0),
                       loss});
    }
  }

  auto add_ac_terminal_flow = [&](EdgeKind kind, const DeviceTerminalFlow& flow) {
    if (!flow.closed) return;
    const int from_loc = global_bus_loc(id, false, flow.bus_from);
    const int to_loc = global_bus_loc(id, false, flow.bus_to);
    if (from_loc < 0 || to_loc < 0) return;
    const double loss = std::max(flow.pf_mw + flow.pt_mw, 0.0);

    if (flow.pf_mw > kTol) {
      edges.push_back({kind,
                       flow.index,
                       from_loc,
                       to_loc,
                       flow.bus_from,
                       flow.bus_to,
                       false,
                       false,
                       flow.pf_mw,
                       std::max(-flow.pt_mw, 0.0),
                       loss});
    } else if (flow.pt_mw > kTol) {
      edges.push_back({kind,
                       flow.index,
                       to_loc,
                       from_loc,
                       flow.bus_to,
                       flow.bus_from,
                       false,
                       false,
                       flow.pt_mw,
                       std::max(-flow.pf_mw, 0.0),
                       loss});
    }
  };

  for (const auto& flow : pf.ac_switch_flows) {
    add_ac_terminal_flow(EdgeKind::ACSwitch, flow);
  }
  for (const auto& flow : pf.ac_circuit_breaker_flows) {
    add_ac_terminal_flow(EdgeKind::ACCircuitBreaker, flow);
  }

  struct DCTerminalFlow {
    double pf_mw{0.0};
    double pt_mw{0.0};
  };
  const double base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;
  std::vector<DCTerminalFlow> dc_terminal(sys.dc.branches.size());
  for (size_t bi = 0; bi < sys.dc.branches.size(); ++bi) {
    const auto& br = sys.dc.branches[bi];
    if (!br.in_service || br.r_pu <= kTol) continue;
    const auto itf = id.dc_loc.find(br.from_bus);
    const auto itt = id.dc_loc.find(br.to_bus);
    if (itf == id.dc_loc.end() || itt == id.dc_loc.end()) continue;
    const int from_bus_loc = itf->second;
    const int to_bus_loc = itt->second;
    if (from_bus_loc < 0 || from_bus_loc >= static_cast<int>(pf.vdc.size()) ||
        to_bus_loc < 0 || to_bus_loc >= static_cast<int>(pf.vdc.size())) {
      continue;
    }

    const double v_from = pf.vdc[static_cast<size_t>(from_bus_loc)];
    const double v_to = pf.vdc[static_cast<size_t>(to_bus_loc)];
    const double i_pu = (v_from - v_to) / br.r_pu;
    const double p_from_mw = base_mva * v_from * i_pu;
    const double p_to_mw = -base_mva * v_to * i_pu;
    dc_terminal[bi] = {p_from_mw, p_to_mw};
  }

  // Some hybrid solves hold a DC voltage-boundary pair at the same reported
  // voltage while converter coupling still establishes a non-zero feeder
  // transfer. Recover radial DC terminal flows from solved nodal injections so
  // carbon tracing uses the same KCL-consistent operating point as PF output.
  std::unordered_map<int, double> dc_net_export;
  for (const auto& bus : sys.dc.buses)
    if (bus.in_service) dc_net_export[bus.index] -= bus.pd_mw;
  for (const auto& ld : sys.dc.loads)
    if (ld.in_service) dc_net_export[ld.bus] -= model::effective_load_p_mw(ld);
  for (const auto& pv : sys.dc.pv_arrays)
    if (pv.in_service) dc_net_export[pv.bus] += pv.p_set_mw;
  for (const auto& sg : sys.dc.static_generators)
    if (sg.in_service) dc_net_export[sg.bus] += sg.p_mw * sg.scaling;
  for (const auto& sg : sys.dc.dc_static_generators)
    if (sg.in_service) dc_net_export[sg.bus] += sg.p_set_mw * sg.scaling;
  for (const auto& st : sys.dc.storage)
    if (st.in_service) dc_net_export[st.bus] += st.p_mw;
  for (const auto& tr : pf.vsc_transfers)
    dc_net_export[tr.bus_dc] += tr.p_dc_mw;
  for (const auto& tr : pf.dcdc_transfers) {
    dc_net_export[tr.bus_in] -= tr.p_in_mw;
    dc_net_export[tr.bus_out] += tr.p_out_mw;
  }

  std::unordered_map<int, std::vector<std::pair<int, size_t>>> dc_adj;
  std::unordered_set<int> dc_bus_ids;
  for (const auto& bus : sys.dc.buses) {
    if (bus.in_service && bus.bus_type != DCBusType::DC_ISOLATED)
      dc_bus_ids.insert(bus.index);
  }
  for (size_t bi = 0; bi < sys.dc.branches.size(); ++bi) {
    const auto& br = sys.dc.branches[bi];
    if (!br.in_service || !dc_bus_ids.count(br.from_bus) ||
        !dc_bus_ids.count(br.to_bus)) continue;
    dc_adj[br.from_bus].push_back({br.to_bus, bi});
    dc_adj[br.to_bus].push_back({br.from_bus, bi});
  }
  const auto is_dc_reference = [&](int bus_index) {
    const auto it = id.dc_loc.find(bus_index);
    return it != id.dc_loc.end() && it->second >= 0 &&
           it->second < static_cast<int>(sys.dc.buses.size()) &&
           sys.dc.buses[static_cast<size_t>(it->second)].bus_type ==
               DCBusType::DC_V;
  };
  const auto set_dc_terminal_from_bus = [&](size_t branch_pos, int bus,
                                            double outflow_mw) {
    const auto& br = sys.dc.branches[branch_pos];
    double vm = 1.0;
    const auto vit = id.dc_loc.find(bus);
    if (vit != id.dc_loc.end() && vit->second >= 0 &&
        vit->second < static_cast<int>(pf.vdc.size())) {
      vm = std::max(std::abs(pf.vdc[static_cast<size_t>(vit->second)]), 1e-3);
    }
    const double i_pu = std::abs(outflow_mw) / (base_mva * vm);
    const double loss = std::max(br.r_pu, 0.0) * i_pu * i_pu * base_mva;
    if (bus == br.from_bus) {
      dc_terminal[branch_pos] = {outflow_mw, loss - outflow_mw};
    } else {
      dc_terminal[branch_pos] = {loss - outflow_mw, outflow_mw};
    }
  };

  std::unordered_set<int> visited_dc;
  for (int start : dc_bus_ids) {
    if (visited_dc.count(start)) continue;
    std::vector<int> component;
    std::vector<int> roots;
    std::vector<int> stack{start};
    visited_dc.insert(start);
    size_t degree_sum = 0;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      component.push_back(u);
      if (is_dc_reference(u)) roots.push_back(u);
      degree_sum += dc_adj[u].size();
      for (const auto& [v, edge_pos] : dc_adj[u]) {
        (void)edge_pos;
        if (visited_dc.insert(v).second) stack.push_back(v);
      }
    }
    if (roots.size() != 1 || degree_sum / 2 + 1 != component.size()) continue;
    const std::unordered_set<int> component_set(component.begin(), component.end());
    std::function<double(int, int)> repair_tree = [&](int u, int parent) {
      double subtree_export = dc_net_export[u];
      for (const auto& [v, edge_pos] : dc_adj[u]) {
        if (v == parent || !component_set.count(v)) continue;
        const double child_export = repair_tree(v, u);
        set_dc_terminal_from_bus(edge_pos, v, child_export);
        subtree_export += child_export;
      }
      return subtree_export;
    };
    (void)repair_tree(roots.front(), -1);
  }

  for (size_t bi = 0; bi < sys.dc.branches.size(); ++bi) {
    const auto& br = sys.dc.branches[bi];
    if (!br.in_service) continue;
    const double p_from_mw = dc_terminal[bi].pf_mw;
    const double p_to_mw = dc_terminal[bi].pt_mw;
    const double loss = std::max(p_from_mw + p_to_mw, 0.0);

    const int from_loc = global_bus_loc(id, true, br.from_bus);
    const int to_loc = global_bus_loc(id, true, br.to_bus);
    if (from_loc < 0 || to_loc < 0) continue;

    if (p_from_mw > kTol) {
      edges.push_back({EdgeKind::DCBranch,
                       br.index,
                       from_loc,
                       to_loc,
                       br.from_bus,
                       br.to_bus,
                       true,
                       true,
                       p_from_mw,
                       std::max(-p_to_mw, 0.0),
                       loss});
    } else if (p_to_mw > kTol) {
      edges.push_back({EdgeKind::DCBranch,
                       br.index,
                       to_loc,
                       from_loc,
                       br.to_bus,
                       br.from_bus,
                       true,
                       true,
                       p_to_mw,
                       std::max(-p_from_mw, 0.0),
                       loss});
    }
  }

  for (const auto& tr : pf.vsc_transfers) {
    const double loss = std::max(tr.loss_mw, 0.0);
    if (tr.p_ac_mw < -kTol) {
      const int from_loc = global_bus_loc(id, false, tr.bus_ac);
      const int to_loc = global_bus_loc(id, true, tr.bus_dc);
      if (from_loc < 0 || to_loc < 0) continue;
      const double send = -tr.p_ac_mw;
      const double recv = std::max(tr.p_dc_mw, 0.0);
      edges.push_back({EdgeKind::VSC,
                       tr.index,
                       from_loc,
                       to_loc,
                       tr.bus_ac,
                       tr.bus_dc,
                       false,
                       true,
                       send,
                       recv,
                       std::max(loss, send - recv)});
    } else if (tr.p_dc_mw < -kTol) {
      const int from_loc = global_bus_loc(id, true, tr.bus_dc);
      const int to_loc = global_bus_loc(id, false, tr.bus_ac);
      if (from_loc < 0 || to_loc < 0) continue;
      const double send = -tr.p_dc_mw;
      const double recv = std::max(tr.p_ac_mw, 0.0);
      edges.push_back({EdgeKind::VSC,
                       tr.index,
                       from_loc,
                       to_loc,
                       tr.bus_dc,
                       tr.bus_ac,
                       true,
                       false,
                       send,
                       recv,
                       std::max(loss, send - recv)});
    }
  }

  for (const auto& tr : pf.dcdc_transfers) {
    const double inj_in = -tr.p_in_mw;
    const double inj_out = tr.p_out_mw;
    const double loss = std::max(tr.loss_mw, 0.0);

    if (inj_in < -kTol) {
      const int from_loc = global_bus_loc(id, true, tr.bus_in);
      const int to_loc = global_bus_loc(id, true, tr.bus_out);
      if (from_loc < 0 || to_loc < 0) continue;
      const double send = -inj_in;
      const double recv = std::max(inj_out, 0.0);
      edges.push_back({EdgeKind::DCDC,
                       tr.index,
                       from_loc,
                       to_loc,
                       tr.bus_in,
                       tr.bus_out,
                       true,
                       true,
                       send,
                       recv,
                       std::max(loss, send - recv)});
    } else if (inj_out < -kTol) {
      const int from_loc = global_bus_loc(id, true, tr.bus_out);
      const int to_loc = global_bus_loc(id, true, tr.bus_in);
      if (from_loc < 0 || to_loc < 0) continue;
      const double send = -inj_out;
      const double recv = std::max(inj_in, 0.0);
      edges.push_back({EdgeKind::DCDC,
                       tr.index,
                       from_loc,
                       to_loc,
                       tr.bus_out,
                       tr.bus_in,
                       true,
                       true,
                       send,
                       recv,
                       std::max(loss, send - recv)});
    }
  }

  const auto router_ports = build_energy_router_port_states(sys, pf, id);
  for (const auto& er : sys.energy_routers) {
    auto it = router_ports.find(er.index);
    if (it == router_ports.end()) continue;

    std::vector<const RouterPortState*> input_ports;
    std::vector<const RouterPortState*> output_ports;
    double total_input_mw = 0.0;
    double total_output_mw = 0.0;

    for (const auto& p : it->second) {
      if (p.node_loc < 0) continue;
      if (p.p_mw < -kTol) {
        input_ports.push_back(&p);
        total_input_mw += -p.p_mw;
      } else if (p.p_mw > kTol) {
        output_ports.push_back(&p);
        total_output_mw += p.p_mw;
      }
    }

    if (input_ports.empty() || output_ports.empty() || total_input_mw <= kTol) continue;

    const double recv_ratio = std::clamp(total_output_mw / total_input_mw, 0.0, 1.0);
    for (const RouterPortState* in_port : input_ports) {
      const double input_power_mw = -in_port->p_mw;
      for (const RouterPortState* out_port : output_ports) {
        const double output_share = out_port->p_mw / std::max(total_output_mw, kTol);
        const double send_mw = input_power_mw * output_share;
        const double recv_mw = send_mw * recv_ratio;
        if (send_mw <= kTol && recv_mw <= kTol) continue;
        edges.push_back({EdgeKind::EnergyRouter,
                         er.index,
                         in_port->node_loc,
                         out_port->node_loc,
                         in_port->bus,
                         out_port->bus,
                         in_port->is_dc,
                         out_port->is_dc,
                         send_mw,
                         recv_mw,
                         std::max(send_mw - recv_mw, 0.0)});
      }
    }
  }

  return edges;
}

SourceBuildResult build_sources(const HybridPowerSystem& sys,
                                 const HybridIDMap& id,
                                 const PowerFlowResult& pf) {
  SourceBuildResult out;
  out.sources.reserve(sys.ac.generators.size() + sys.ac.static_generators.size() +
                      sys.ac.external_grids.size() +
                      sys.ac.renewable_gens.size() + sys.ac.pv_systems.size() +
                      sys.ac.loads.size() + sys.ac.buses.size() +
                      sys.dc.static_generators.size() +
                      sys.dc.dc_static_generators.size() + sys.dc.buses.size() +
                      sys.dc.loads.size() +
                      sys.ac.storage.size() + sys.dc.storage.size());

  int src_id = 0;
  const auto push_source = [&](int bus,
                               bool is_dc,
                               int node_loc,
                               double p_mw,
                               double ef,
                               bool is_balancing,
                               bool can_absorb_export,
                               const std::string& source_type,
                               int component_index,
                               const std::string& label) {
    Source source;
    source.id = src_id++;
    source.bus = bus;
    source.is_dc = is_dc;
    source.node_loc = node_loc;
    source.p_mw = p_mw;
    source.ef = ef;
    source.is_balancing = is_balancing;
    source.can_absorb_export = can_absorb_export;
    source.source_type = source_type;
    source.component_index = component_index;
    source.label = label;
    source.scheduled_p_mw = p_mw;
    out.sources.push_back(std::move(source));
    return static_cast<int>(out.sources.size()) - 1;
  };
  std::unordered_set<int> external_grid_buses;
  for (const auto& grid : sys.ac.external_grids) {
    if (grid.in_service) external_grid_buses.insert(grid.bus);
  }
  std::unordered_set<int> slack_buses;
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service && bus.bus_type == BusType::SLACK) {
      slack_buses.insert(bus.index);
    }
  }

  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const int node_loc = global_bus_loc(id, false, g.bus);
    if (node_loc < 0) continue;

    const bool is_balancing = g.is_slack || slack_buses.count(g.bus) != 0;
    // A slack generator collocated with an ExternalGrid is the numerical
    // reference for that grid, not a second physical source.  The external
    // grid owns the residual injection and its carbon profile in this case.
    if (is_balancing && external_grid_buses.count(g.bus) != 0) continue;

    const double p_mw = is_balancing ? 0.0 : std::max(g.pg_mw, 0.0);
    const int source_pos = push_source(
        g.bus, false, node_loc, p_mw, g.emission_factor_tco2_mwh,
        is_balancing, is_balancing, "generator", g.index,
        g.name.empty() ? "Gen" + std::to_string(g.index) : g.name);
    if (is_balancing && out.slack_source_pos < 0) {
      out.slack_source_pos = source_pos;
    }
  }

  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const int node_loc = global_bus_loc(id, false, eg.bus);
    if (node_loc < 0) continue;
    const int source_pos = push_source(
        eg.bus, false, node_loc, 0.0, eg.emission_factor_tco2_mwh, true,
        true, "external_grid", eg.index,
        eg.name.empty() ? "Grid" + std::to_string(eg.index) : eg.name);
    if (out.slack_source_pos < 0) {
      out.slack_source_pos = source_pos;
    }
  }

  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, false, sg.bus);
    const double p_mw = sg.p_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    push_source(sg.bus, false, node_loc, p_mw, sg.co2_emission_rate, false,
                false, "static_generator", sg.index,
                sg.name.empty() ? "Sgen" + std::to_string(sg.index) : sg.name);
  }

  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service || rg.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, rg.bus);
    if (node_loc < 0) continue;
    push_source(rg.bus, false, node_loc, rg.p_mw, 0.0, false, false,
                "renewable_generator", rg.index,
                rg.name.empty() ? "Ren" + std::to_string(rg.index) : rg.name);
  }

  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    const int node_loc = global_bus_loc(id, false, pv.bus);
    const double p_mw = powerflow::compute_pv_power_mw(pv);
    if (node_loc < 0 || p_mw <= kTol) continue;
    push_source(pv.bus, false, node_loc, p_mw, 0.0, false, false,
                "pv_system", pv.index,
                pv.name.empty() ? "PV" + std::to_string(pv.index) : pv.name);
  }

  // Some import formats encode fixed generation as a negative load. Preserve
  // that signed injection in the carbon snapshot instead of reporting it as a
  // negative demand. Load records have no carbon-factor field, so their direct
  // emission factor is zero by definition.
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    const double p_mw = zip_active_power_mw(
        load, ac_voltage_magnitude(id, pf, load.bus));
    if (p_mw >= -kTol) continue;
    const int node_loc = global_bus_loc(id, false, load.bus);
    if (node_loc < 0) continue;
    push_source(load.bus, false, node_loc, -p_mw, 0.0, false, false,
                "negative_load_injection", load.index,
                load.name.empty() ? "LoadInjection" + std::to_string(load.index)
                                  : load.name);
  }
  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service || bus.pd_mw >= -kTol) continue;
    const int node_loc = global_bus_loc(id, false, bus.index);
    if (node_loc < 0) continue;
    push_source(bus.index, false, node_loc, -bus.pd_mw, 0.0, false, false,
                "negative_bus_demand", bus.index,
                bus.name.empty() ? "BusInjection" + std::to_string(bus.index)
                                 : bus.name);
  }

  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, true, sg.bus);
    const double p_mw = sg.p_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    push_source(sg.bus, true, node_loc, p_mw, sg.co2_emission_rate, false,
                false, "dc_static_generator", sg.index,
                sg.name.empty() ? "DCSgen" + std::to_string(sg.index) : sg.name);
  }

  // Native DC fixed generators carry their own factor; PV arrays retain the
  // zero direct-emission convention below.
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, true, sg.bus);
    const double p_mw = sg.p_set_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    push_source(sg.bus, true, node_loc, p_mw,
                sg.emission_factor_tco2_mwh, false, false, "dc_generator",
                sg.index,
                sg.name.empty() ? "DCGen" + std::to_string(sg.index) : sg.name);
  }

  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    const int node_loc = global_bus_loc(id, true, pv.bus);
    const double p_mw = pv.p_set_mw;
    if (node_loc < 0 || p_mw <= kTol) continue;
    push_source(pv.bus, true, node_loc, p_mw, 0.0, false, false,
                "dc_pv_array", pv.index,
                pv.name.empty() ? "DCPV" + std::to_string(pv.index) : pv.name);
  }

  for (const auto& load : sys.dc.loads) {
    const double p_mw = model::effective_load_p_mw(load);
    if (!load.in_service || p_mw >= -kTol) continue;
    const int node_loc = global_bus_loc(id, true, load.bus);
    if (node_loc < 0) continue;
    push_source(load.bus, true, node_loc, -p_mw, 0.0, false, false,
                "negative_dc_load_injection", load.index,
                load.name.empty()
                    ? "DCLoadInjection" + std::to_string(load.index)
                    : load.name);
  }
  for (const auto& bus : sys.dc.buses) {
    if (!bus.in_service || bus.pd_mw >= -kTol) continue;
    const int node_loc = global_bus_loc(id, true, bus.index);
    if (node_loc < 0) continue;
    push_source(bus.index, true, node_loc, -bus.pd_mw, 0.0, false,
                false, "negative_dc_bus_demand", bus.index,
                bus.name.empty()
                    ? "DCBusInjection" + std::to_string(bus.index)
                    : bus.name);
  }

  // A DC_V bus is an ideal voltage boundary in the power-flow equations and
  // therefore supplies or absorbs the residual power of its DC island.  Model
  // the supplying direction explicitly so the carbon snapshot uses the same
  // physical boundary as the solved power flow.
  for (const auto& bus : sys.dc.buses) {
    if (!bus.in_service || bus.bus_type != DCBusType::DC_V) continue;
    const int node_loc = global_bus_loc(id, true, bus.index);
    if (node_loc < 0) continue;
    push_source(bus.index, true, node_loc, 0.0,
                bus.emission_factor_tco2_mwh, true, true,
                "dc_voltage_boundary", bus.index,
                bus.name.empty() ? "DCGrid" + std::to_string(bus.index)
                                 : bus.name);
  }

  for (const auto& st : sys.ac.storage) {
    if (!st.in_service || st.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, st.bus);
    if (node_loc < 0) continue;
    push_source(st.bus, false, node_loc, st.p_mw,
                st.soc_carbon_intensity_tco2_mwh, false, false,
                "storage_discharge", st.index,
                st.name.empty() ? "BESS" + std::to_string(st.index) : st.name);
  }

  for (const auto& st : sys.dc.storage) {
    if (!st.in_service || st.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, true, st.bus);
    if (node_loc < 0) continue;
    push_source(st.bus, true, node_loc, st.p_mw,
                st.soc_carbon_intensity_tco2_mwh, false, false,
                "dc_storage_discharge", st.index,
                st.name.empty() ? "DCBESS" + std::to_string(st.index) : st.name);
  }

  return out;
}

double sum_ac_shunt_load(const ACSystem& ac) {
  double total = 0.0;
  for (const auto& b : ac.buses) total += b.gs_mw;
  return total;
}

double sum_mobile_storage_net_injection(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service || ms.status == MobileStorageStatus::InTransit) continue;
    total += ms.p_mw;
  }
  return total;
}

double sum_aggregate_net_injection(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& vpp : sys.vpps) {
    if (vpp.in_service) total += vpp.p_output_mw;
  }
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service) continue;
    if (mg.operating_mode != MicrogridMode::GridConnected) continue;
    total += mg.p_exchange_mw;
  }
  return total;
}

void reconstruct_slack_output(SourceBuildResult& source_build,
                              const HybridPowerSystem& sys,
                              const std::vector<UnifiedLoad>& all_loads,
                              const std::vector<FlowEdge>& edges) {
  if (source_build.slack_source_pos < 0 ||
      source_build.slack_source_pos >= static_cast<int>(source_build.sources.size())) {
    return;
  }

  double total_load = 0.0;
  for (const auto& ld : all_loads) total_load += std::max(ld.p_mw, 0.0);
  total_load += sum_ac_shunt_load(sys.ac);

  double total_loss = 0.0;
  for (const auto& e : edges) total_loss += e.loss_mw;

  double total_non_slack_generation = 0.0;
  for (int i = 0; i < static_cast<int>(source_build.sources.size()); ++i) {
    if (i == source_build.slack_source_pos) continue;
    total_non_slack_generation += source_build.sources[static_cast<size_t>(i)].p_mw;
  }

  const double slack_p = total_load + total_loss - total_non_slack_generation -
                         sum_mobile_storage_net_injection(sys) -
                         sum_aggregate_net_injection(sys);
  source_build.sources[static_cast<size_t>(source_build.slack_source_pos)].p_mw =
      std::max(slack_p, 0.0);
}

std::vector<double> compute_node_net_source_requirement(
    const HybridPowerSystem& sys,
    int node_count,
    const std::vector<UnifiedLoad>& loads,
    const std::vector<FlowEdge>& edges,
    double loss_allocation_alpha) {
  std::vector<double> requirement(static_cast<size_t>(node_count), 0.0);

  for (const auto& ld : loads) {
    if (ld.p_mw <= kTol || ld.node_loc < 0 || ld.node_loc >= node_count) continue;
    requirement[static_cast<size_t>(ld.node_loc)] += ld.p_mw;
  }
  (void)loss_allocation_alpha;
  for (const auto& e : edges) {
    if (e.from_node_loc < 0 || e.to_node_loc < 0 ||
        e.from_node_loc >= node_count || e.to_node_loc >= node_count) {
      continue;
    }
    requirement[static_cast<size_t>(e.from_node_loc)] +=
        std::max(e.send_mw, 0.0);
    requirement[static_cast<size_t>(e.to_node_loc)] -=
        std::max(e.recv_mw, 0.0);
  }

  for (double& p : requirement) {
    if (std::abs(p) < 1e-7) p = 0.0;
  }
  return requirement;
}

void add_external_grid_export_sinks(SourceBuildResult& source_build,
                                    const HybridPowerSystem& sys,
                                    std::vector<UnifiedLoad>& loads,
                                    const std::vector<FlowEdge>& edges,
                                    int node_count,
                                    const CarbonAnalysisOptions& opt) {
  const auto requirement = compute_node_net_source_requirement(
      sys, node_count, loads, edges, opt.loss_allocation_alpha);
  std::vector<double> fixed_source_mw(static_cast<size_t>(node_count), 0.0);
  std::vector<bool> has_export_boundary(static_cast<size_t>(node_count), false);
  for (const auto& source : source_build.sources) {
    if (source.node_loc < 0 || source.node_loc >= node_count) continue;
    if (source.is_balancing) {
      has_export_boundary[static_cast<size_t>(source.node_loc)] =
          has_export_boundary[static_cast<size_t>(source.node_loc)] ||
          source.can_absorb_export;
    } else {
      fixed_source_mw[static_cast<size_t>(source.node_loc)] +=
          std::max(source.p_mw, 0.0);
    }
  }
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (sys.ac.buses[i].in_service &&
        sys.ac.buses[i].bus_type == BusType::SLACK) {
      has_export_boundary[i] = true;
    }
  }
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    if (sys.dc.buses[i].in_service &&
        sys.dc.buses[i].bus_type == DCBusType::DC_V) {
      has_export_boundary[sys.ac.buses.size() + i] = true;
    }
  }

  int export_index = -300000;
  for (int loc = 0; loc < node_count; ++loc) {
    if (!has_export_boundary[static_cast<size_t>(loc)]) continue;
    const double balancing_requirement =
        requirement[static_cast<size_t>(loc)] -
        fixed_source_mw[static_cast<size_t>(loc)];
    if (balancing_requirement >= -kTol) continue;

    const bool is_dc = loc >= static_cast<int>(sys.ac.buses.size());
    const int local_pos = is_dc ? loc - static_cast<int>(sys.ac.buses.size()) : loc;
    const int bus = is_dc ? sys.dc.buses[static_cast<size_t>(local_pos)].index
                          : sys.ac.buses[static_cast<size_t>(local_pos)].index;
    loads.push_back({export_index--,
                     bus,
                     is_dc,
                     loc,
                     -balancing_requirement,
                     0.0,
                     false,
                     false,
                     true});
  }
}

void calibrate_balancing_sources(SourceBuildResult& source_build,
                                 const HybridPowerSystem& sys,
                                 const std::vector<UnifiedLoad>& loads,
                                 const std::vector<FlowEdge>& edges,
                                 int node_count,
                                 const CarbonAnalysisOptions& opt) {
  const std::vector<double> requirement =
      compute_node_net_source_requirement(sys, node_count, loads, edges,
                                           opt.loss_allocation_alpha);

  std::vector<std::vector<size_t>> source_pos_by_node(static_cast<size_t>(node_count));
  for (size_t i = 0; i < source_build.sources.size(); ++i) {
    const int loc = source_build.sources[i].node_loc;
    if (loc < 0 || loc >= node_count) continue;
    source_pos_by_node[static_cast<size_t>(loc)].push_back(i);
  }

  for (int loc = 0; loc < node_count; ++loc) {
    const auto& positions = source_pos_by_node[static_cast<size_t>(loc)];
    if (positions.empty()) continue;

    double fixed_generation = 0.0;
    std::vector<size_t> balancing_positions;
    for (const size_t pos : positions) {
      if (source_build.sources[pos].is_balancing) {
        balancing_positions.push_back(pos);
      } else {
        fixed_generation += std::max(source_build.sources[pos].p_mw, 0.0);
      }
    }
    if (balancing_positions.empty()) continue;

    const double required = std::max(
        requirement[static_cast<size_t>(loc)] - fixed_generation, 0.0);

    double scheduled_total = 0.0;
    for (const size_t pos : balancing_positions) {
      scheduled_total += std::max(source_build.sources[pos].p_mw, 0.0);
    }

    if (scheduled_total > kTol) {
      const double scale = required / scheduled_total;
      for (const size_t pos : balancing_positions) {
        source_build.sources[pos].p_mw =
            std::max(source_build.sources[pos].p_mw, 0.0) * scale;
      }
    } else {
      source_build.sources[balancing_positions.front()].p_mw = required;
      for (size_t i = 1; i < balancing_positions.size(); ++i) {
        source_build.sources[balancing_positions[i]].p_mw = 0.0;
      }
    }
  }
}

PowerBalanceDiagnostics verify_node_power_balance(
    const HybridPowerSystem& sys,
    const std::vector<Source>& sources,
    const std::vector<UnifiedLoad>& loads,
    const std::vector<FlowEdge>& edges,
    int node_count,
    const CarbonAnalysisOptions& opt) {
  const auto requirement = compute_node_net_source_requirement(
      sys, node_count, loads, edges, opt.loss_allocation_alpha);
  std::vector<double> source_mw(static_cast<size_t>(node_count), 0.0);
  double scale_mw = 0.0;
  for (const auto& source : sources) {
    if (source.node_loc < 0 || source.node_loc >= node_count) continue;
    source_mw[static_cast<size_t>(source.node_loc)] += std::max(source.p_mw, 0.0);
    scale_mw += std::max(source.p_mw, 0.0);
  }
  for (const auto& load : loads) scale_mw += std::max(load.p_mw, 0.0);

  PowerBalanceDiagnostics diagnostics;
  diagnostics.tolerance_mw =
      std::max(1e-7, opt.verification_tol * std::max(scale_mw, 1.0));
  for (int loc = 0; loc < node_count; ++loc) {
    const double error = source_mw[static_cast<size_t>(loc)] -
                         requirement[static_cast<size_t>(loc)];
    const double abs_error = std::abs(error);
    diagnostics.max_abs_error_mw =
        std::max(diagnostics.max_abs_error_mw, abs_error);
    diagnostics.total_abs_error_mw += abs_error;
    if (abs_error > diagnostics.tolerance_mw) {
      const bool is_dc = loc >= static_cast<int>(sys.ac.buses.size());
      const int local_pos =
          is_dc ? loc - static_cast<int>(sys.ac.buses.size()) : loc;
      const int bus = is_dc
                          ? sys.dc.buses[static_cast<size_t>(local_pos)].index
                          : sys.ac.buses[static_cast<size_t>(local_pos)].index;
      diagnostics.errors.push_back(
          {bus, is_dc, error, source_mw[static_cast<size_t>(loc)],
           requirement[static_cast<size_t>(loc)]});
    }
  }
  diagnostics.verified =
      diagnostics.max_abs_error_mw <= diagnostics.tolerance_mw;
  return diagnostics;
}

TracingResult proportional_tracing(const std::vector<Source>& sources,
                                   const std::vector<FlowEdge>& edges,
                                   const std::vector<UnifiedLoad>& loads,
                                   int node_count,
                                   const CarbonAnalysisOptions& opt) {
  std::vector<double> total_inflow(static_cast<size_t>(node_count), 0.0);
  for (const auto& source : sources) {
    if (source.p_mw <= kTol || source.node_loc < 0 || source.node_loc >= node_count) continue;
    total_inflow[static_cast<size_t>(source.node_loc)] += source.p_mw;
  }
  for (const auto& edge : edges) {
    if (edge.to_node_loc < 0 || edge.to_node_loc >= node_count) continue;
    total_inflow[static_cast<size_t>(edge.to_node_loc)] += std::max(edge.recv_mw, 0.0);
  }

  TracingResult result;
  result.load_supply.resize(loads.size());
  result.edge_loss_alloc.resize(edges.size());
  if (node_count == 0 || sources.empty()) return result;

  using Sparse = Eigen::SparseMatrix<double>;
  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> allocation_triplets;
  allocation_triplets.reserve(static_cast<size_t>(node_count) + edges.size());
  for (int node = 0; node < node_count; ++node) {
    allocation_triplets.emplace_back(
        node, node,
        (total_inflow[static_cast<size_t>(node)] > kTol)
            ? total_inflow[static_cast<size_t>(node)]
            : 1.0);
  }
  for (const auto& edge : edges) {
    if (edge.from_node_loc < 0 || edge.from_node_loc >= node_count ||
        edge.to_node_loc < 0 || edge.to_node_loc >= node_count) {
      continue;
    }
    allocation_triplets.emplace_back(
        edge.to_node_loc, edge.from_node_loc,
        -std::max(edge.recv_mw, 0.0));
  }
  Sparse allocation_matrix(node_count, node_count);
  allocation_matrix.setFromTriplets(allocation_triplets.begin(),
                                    allocation_triplets.end());
  allocation_matrix.makeCompressed();

  Eigen::MatrixXd source_injection =
      Eigen::MatrixXd::Zero(node_count, static_cast<int>(sources.size()));
  for (size_t source_pos = 0; source_pos < sources.size(); ++source_pos) {
    const auto& source = sources[source_pos];
    if (source.p_mw <= kTol || source.node_loc < 0 || source.node_loc >= node_count) continue;
    source_injection(source.node_loc, static_cast<int>(source_pos)) += source.p_mw;
  }

  // Davis, Direct Methods for Sparse Linear Systems, Ch. 5-6: the tracing
  // stage needs a nonsingular solve but exposes no rank diagnostic, so sparse
  // LU avoids the extra rank-revealing QR work.  The audited carbon-potential
  // solve below retains SparseQR rank and pivot-condition checks.
  Eigen::SparseLU<Sparse, Eigen::COLAMDOrdering<int>> lu;
  lu.analyzePattern(allocation_matrix);
  lu.factorize(allocation_matrix);
  if (lu.info() != Eigen::Success) return result;
  Eigen::MatrixXd source_fraction = lu.solve(source_injection);
  if (lu.info() != Eigen::Success || !source_fraction.allFinite()) return result;

  for (int node = 0; node < node_count; ++node) {
    for (int source_pos = 0; source_pos < source_fraction.cols(); ++source_pos) {
      double& fraction = source_fraction(node, source_pos);
      if (fraction < 0.0 && fraction > -1e-10) fraction = 0.0;
    }
  }

  for (size_t load_pos = 0; load_pos < loads.size(); ++load_pos) {
    const auto& load = loads[load_pos];
    if (load.p_mw <= kTol || load.node_loc < 0 || load.node_loc >= node_count) continue;
    for (size_t source_pos = 0; source_pos < sources.size(); ++source_pos) {
      const double contribution =
          load.p_mw * source_fraction(load.node_loc, static_cast<int>(source_pos));
      if (contribution < opt.min_contribution_mw) continue;
      result.load_supply[load_pos][sources[source_pos].id] += contribution;
    }
  }

  for (size_t edge_pos = 0; edge_pos < edges.size(); ++edge_pos) {
    const auto& edge = edges[edge_pos];
    if (edge.loss_mw <= kTol || edge.from_node_loc < 0 ||
        edge.from_node_loc >= node_count) {
      continue;
    }
    for (size_t source_pos = 0; source_pos < sources.size(); ++source_pos) {
      const double allocation =
          edge.loss_mw *
          source_fraction(edge.from_node_loc, static_cast<int>(source_pos));
      if (allocation < opt.min_contribution_mw) continue;
      result.edge_loss_alloc[edge_pos][sources[source_pos].id] += allocation;
      result.gen_loss_total[sources[source_pos].id] += allocation;
    }
  }

  return result;
}

std::unordered_map<int, double> build_ef_map(const std::vector<Source>& sources) {
  std::unordered_map<int, double> ef_map;
  ef_map.reserve(sources.size());
  for (const auto& s : sources) ef_map[s.id] = s.ef;
  return ef_map;
}

std::pair<double, double> summarize_weighted_emissions(
    const std::unordered_map<int, double>& contribution_mw,
    const std::unordered_map<int, double>& ef_map) {
  double total_emission = 0.0;
  double total_power = 0.0;
  for (const auto& [sid, mw] : contribution_mw) {
    const auto it = ef_map.find(sid);
    const double ef = (it == ef_map.end()) ? 0.0 : it->second;
    total_emission += ef * mw;
    total_power += mw;
  }
  return {total_emission, total_power};
}

std::vector<LoadCarbonResult> build_load_carbon(
    const std::vector<UnifiedLoad>& loads,
    const TracingResult& tracing,
    const std::vector<Source>& sources) {
  const auto ef_map = build_ef_map(sources);
  std::vector<LoadCarbonResult> results(loads.size());

  for (size_t li = 0; li < loads.size(); ++li) {
    auto& r = results[li];
    r.load_index = loads[li].index;
    r.bus = loads[li].bus;
    r.demand_mw = loads[li].p_mw;
    r.generator_supply_mw = tracing.load_supply[li];

    const auto [total_emission, total_supply] =
        summarize_weighted_emissions(r.generator_supply_mw, ef_map);
    r.carbon_intensity_tco2_mwh = (total_supply > kTol) ? (total_emission / total_supply) : 0.0;
    r.total_emissions_tco2 = total_emission;
  }

  return results;
}

BranchCarbonResult build_branch_carbon_result(
    int branch_index,
    int from_bus,
    int to_bus,
    double loss_mw,
    const std::unordered_map<int, double>& generator_loss_mw,
    const std::unordered_map<int, double>& ef_map) {
  BranchCarbonResult r;
  r.branch_index = branch_index;
  r.from_bus = from_bus;
  r.to_bus = to_bus;
  r.loss_mw = loss_mw;
  r.generator_loss_mw = generator_loss_mw;
  const auto [total_emission, total_loss_tracked] =
      summarize_weighted_emissions(r.generator_loss_mw, ef_map);
  r.carbon_intensity_tco2_mwh =
      (total_loss_tracked > kTol) ? (total_emission / total_loss_tracked) : 0.0;
  r.total_emissions_tco2 = total_emission;
  return r;
}

VSCCarbonResult build_vsc_carbon_result(const VSCTransfer& tr,
                                        const FlowEdge* edge,
                                        const std::unordered_map<int, double>& ef_map,
                                        const std::unordered_map<int, double>& generator_loss_mw) {
  VSCCarbonResult r;
  r.converter_index = tr.index;
  r.bus_ac = tr.bus_ac;
  r.bus_dc = tr.bus_dc;
  r.ac_to_dc = edge ? !edge->from_is_dc : (tr.p_ac_mw < 0.0);
  r.input_power_mw = edge ? edge->send_mw : 0.0;
  r.output_power_mw = edge ? edge->recv_mw : 0.0;
  r.loss_mw = edge ? edge->loss_mw : std::max(tr.loss_mw, 0.0);
  r.generator_loss_mw = generator_loss_mw;
  const auto [total_emission, total_loss_tracked] =
      summarize_weighted_emissions(r.generator_loss_mw, ef_map);
  r.carbon_intensity_tco2_mwh =
      (total_loss_tracked > kTol) ? (total_emission / total_loss_tracked) : 0.0;
  r.total_emissions_tco2 = total_emission;
  return r;
}

DCDCCarbonResult build_dcdc_carbon_result(const DCDCTransfer& tr,
                                          const FlowEdge* edge,
                                          const std::unordered_map<int, double>& ef_map,
                                          const std::unordered_map<int, double>& generator_loss_mw) {
  DCDCCarbonResult r;
  r.converter_index = tr.index;
  r.bus_in = tr.bus_in;
  r.bus_out = tr.bus_out;
  r.input_to_output = edge ? (edge->from_bus == tr.bus_in) : (tr.p_in_mw >= 0.0);
  r.input_power_mw = edge ? edge->send_mw : std::max(std::abs(tr.p_in_mw), 0.0);
  r.output_power_mw = edge ? edge->recv_mw : std::max(std::abs(tr.p_out_mw), 0.0);
  r.loss_mw = edge ? edge->loss_mw : std::max(tr.loss_mw, 0.0);
  r.generator_loss_mw = generator_loss_mw;
  const auto [total_emission, total_loss_tracked] =
      summarize_weighted_emissions(r.generator_loss_mw, ef_map);
  r.carbon_intensity_tco2_mwh =
      (total_loss_tracked > kTol) ? (total_emission / total_loss_tracked) : 0.0;
  r.total_emissions_tco2 = total_emission;
  return r;
}

void accumulate_contribution_map(std::unordered_map<int, double>& dst,
                                 const std::unordered_map<int, double>& src) {
  for (const auto& [sid, mw] : src) dst[sid] += mw;
}

StorageCarbonResult build_storage_carbon_result(
    const StorageState& st,
    const LoadCarbonResult* charging_load) {
  StorageCarbonResult r;
  r.storage_index = st.index;
  r.bus = st.bus;
  r.is_dc = st.is_dc;
  r.p_mw = st.p_mw;
  r.soc = st.soc;
  r.stored_energy_mwh = std::max(st.soc, 0.0) * std::max(st.e_rated_mwh, 0.0);
  r.soc_carbon_intensity_tco2_mwh = st.soc_carbon_intensity_tco2_mwh;

  if (charging_load != nullptr && st.p_mw < -kTol) {
    r.carbon_intensity_tco2_mwh = charging_load->carbon_intensity_tco2_mwh;
    r.total_emissions_tco2 = charging_load->total_emissions_tco2;
    r.source_supply_mw = charging_load->generator_supply_mw;
  } else {
    r.carbon_intensity_tco2_mwh = st.soc_carbon_intensity_tco2_mwh;
    r.total_emissions_tco2 = std::max(st.p_mw, 0.0) * r.carbon_intensity_tco2_mwh;
  }

  return r;
}

EnergyRouterCarbonResult build_energy_router_carbon_result(
    int router_index,
    const std::vector<RouterPortState>& ports,
    const std::unordered_map<int, double>& ef_map,
    const std::unordered_map<int, double>& source_loss_mw) {
  EnergyRouterCarbonResult r;
  r.router_index = router_index;
  r.source_loss_mw = source_loss_mw;

  for (const auto& p : ports) {
    if (p.node_loc < 0) continue;
    if (p.p_mw < -kTol) {
      r.input_power_mw += -p.p_mw;
      ++r.active_input_ports;
    } else if (p.p_mw > kTol) {
      r.output_power_mw += p.p_mw;
      ++r.active_output_ports;
    }
  }
  r.loss_mw = std::max(r.input_power_mw - r.output_power_mw, 0.0);

  const auto [total_emission, total_loss_tracked] =
      summarize_weighted_emissions(r.source_loss_mw, ef_map);
  r.carbon_intensity_tco2_mwh =
      (total_loss_tracked > kTol) ? (total_emission / total_loss_tracked) : 0.0;
  r.total_emissions_tco2 = total_emission;
  return r;
}

MatrixResult solve_carbon_matrix(const HybridPowerSystem& sys,
                                 const std::vector<Source>& sources,
                                 const std::vector<FlowEdge>& edges,
                                 const std::vector<UnifiedLoad>& loads,
                                 const CarbonAnalysisOptions& opt) {
  const int node_count = static_cast<int>(sys.ac.buses.size() + sys.dc.buses.size());
  MatrixResult res;
  res.w.assign(static_cast<size_t>(node_count), 0.0);

  if (node_count == 0) {
    res.solved = true;
    return res;
  }

  Eigen::VectorXd Pout = Eigen::VectorXd::Zero(node_count);

  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> in_triplets;
  in_triplets.reserve(edges.size());

  for (const auto& e : edges) {
    if (e.from_node_loc < 0 || e.to_node_loc < 0 ||
        e.from_node_loc >= node_count || e.to_node_loc >= node_count) {
      continue;
    }
    const double send = std::max(e.send_mw, 0.0);
    const double recv = std::max(e.recv_mw, 0.0);
    const double loss = std::clamp(std::max(e.loss_mw, 0.0), 0.0, send);
    const double alpha = clamped_loss_alpha(opt.loss_allocation_alpha);
    const double receiver_loss_share = (1.0 - alpha) * loss;
    const double carbon_transfer = recv + alpha * loss;
    const double sender_diagonal = send + (2.0 * alpha - 1.0) * loss;

    Pout(e.from_node_loc) += std::max(sender_diagonal, 0.0);
    Pout(e.to_node_loc) += receiver_loss_share;
    if (carbon_transfer > kTol) {
      in_triplets.emplace_back(e.to_node_loc, e.from_node_loc, carbon_transfer);
    }
  }

  for (const auto& ld : loads) {
    if (ld.p_mw <= kTol || ld.node_loc < 0 || ld.node_loc >= node_count) continue;
    Pout(ld.node_loc) += ld.p_mw;
  }

  using Sparse = Eigen::SparseMatrix<double>;
  Sparse A(node_count, node_count);
  std::vector<Triplet> a_triplets;
  a_triplets.reserve(static_cast<size_t>(node_count + static_cast<int>(in_triplets.size())));
  Eigen::VectorXd b = Eigen::VectorXd::Zero(node_count);
  Eigen::VectorXd gen_at_bus = Eigen::VectorXd::Zero(node_count);
  for (const auto& s : sources) {
    if (s.node_loc < 0 || s.node_loc >= node_count) continue;
    b(s.node_loc) += s.ef * s.p_mw;
    gen_at_bus(s.node_loc) += s.p_mw;
  }

  std::vector<double> load_at_bus(static_cast<size_t>(node_count), 0.0);
  for (const auto& ld : loads) {
    if (ld.node_loc >= 0 && ld.node_loc < node_count) {
      load_at_bus[static_cast<size_t>(ld.node_loc)] += ld.p_mw;
    }
  }
  std::vector<double> row_max(static_cast<size_t>(node_count), 0.0);
  for (const auto& t : in_triplets) {
    row_max[static_cast<size_t>(t.row())] =
        std::max(row_max[static_cast<size_t>(t.row())], std::abs(t.value()));
  }
  std::vector<bool> inactive_row(static_cast<size_t>(node_count), false);
  for (int i = 0; i < node_count; ++i) {
    const double diag = (std::abs(Pout(i)) > opt.regularization_eps)
                            ? Pout(i)
                            : opt.regularization_eps;
    row_max[static_cast<size_t>(i)] =
        std::max(row_max[static_cast<size_t>(i)], std::abs(diag));
    inactive_row[static_cast<size_t>(i)] =
        load_at_bus[static_cast<size_t>(i)] < 1e-8 && gen_at_bus(i) < 1e-8 &&
        row_max[static_cast<size_t>(i)] < 1e-6;
    if (inactive_row[static_cast<size_t>(i)]) {
      a_triplets.emplace_back(i, i, 1.0);
      b(i) = 0.0;
      row_max[static_cast<size_t>(i)] = 1.0;
    } else {
      a_triplets.emplace_back(i, i, diag);
    }
  }
  for (const auto& t : in_triplets) {
    if (!inactive_row[static_cast<size_t>(t.row())]) {
      a_triplets.emplace_back(t.row(), t.col(), -t.value());
    }
  }
  A.setFromTriplets(a_triplets.begin(), a_triplets.end());
  A.makeCompressed();

  Sparse A_scaled = A;
  Eigen::VectorXd b_scaled = b;
  const double min_scale = std::max(opt.regularization_eps, 1e-15);
  for (int col = 0; col < A_scaled.outerSize(); ++col) {
    for (Sparse::InnerIterator it(A_scaled, col); it; ++it) {
      const double scale = row_max[static_cast<size_t>(it.row())];
      if (scale > min_scale) it.valueRef() /= scale;
    }
  }
  for (int i = 0; i < node_count; ++i) {
    const double scale = row_max[static_cast<size_t>(i)];
    if (scale > min_scale) b_scaled(i) /= scale;
  }

  // Davis, Direct Methods for Sparse Linear Systems, Ch. 7. SparseQR keeps
  // the prior rank-revealing QR contract while avoiding dense n-by-n storage.
  Eigen::SparseQR<Sparse, Eigen::COLAMDOrdering<int>> qr;
  qr.setPivotThreshold(std::max(opt.regularization_eps, 1e-12));
  qr.compute(A_scaled);
  if (qr.info() != Eigen::Success) {
    res.condition_estimate = std::numeric_limits<double>::max();
    return res;
  }
  res.rank = static_cast<int>(qr.rank());
  if (res.rank < node_count) {
    res.condition_estimate = std::numeric_limits<double>::max();
    return res;
  }

  double max_pivot = 0.0;
  double min_pivot = std::numeric_limits<double>::max();
  for (int i = 0; i < node_count; ++i) {
    const double pivot = std::abs(qr.matrixR().coeff(i, i));
    max_pivot = std::max(max_pivot, pivot);
    min_pivot = std::min(min_pivot, pivot);
  }
  res.condition_estimate =
      (min_pivot > 0.0 && std::isfinite(max_pivot) && std::isfinite(min_pivot))
          ? max_pivot / min_pivot
          : std::numeric_limits<double>::max();
  if (!std::isfinite(res.condition_estimate) ||
      (opt.max_matrix_condition_estimate > 0.0 &&
       res.condition_estimate > opt.max_matrix_condition_estimate)) {
    return res;
  }

  Eigen::VectorXd w = qr.solve(b_scaled);
  if (qr.info() != Eigen::Success || !w.allFinite()) {
    return res;
  }
  for (int i = 0; i < node_count; ++i) {
    if (w(i) < -std::max(opt.regularization_eps, 1e-10)) {
      return res;
    }
    if (w(i) < 0.0) w(i) = 0.0;
  }

  const Eigen::VectorXd residual_vector = A * w - b;
  res.residual = residual_vector.norm();
  const double residual_scale =
      std::max(A.norm() * w.norm() + b.norm(), 1e-15);
  res.relative_residual = res.residual / residual_scale;
  if (!std::isfinite(res.residual) || !std::isfinite(res.relative_residual) ||
      (res.residual > opt.regularization_eps &&
       res.relative_residual > opt.verification_tol)) {
    return res;
  }
  res.solved = true;
  for (int i = 0; i < node_count; ++i) res.w[static_cast<size_t>(i)] = w(i);

  if (opt.verbose) {
    HACDCPF_LOG_DEBUG(
        "[CarbonAnalysis] Matrix solve residual: {}, relative: {}, rank: {}, condition estimate: {}",
        res.residual, res.relative_residual, res.rank, res.condition_estimate);
  }

  return res;
}

// Post-process carbon results to use matrix-solved intensities for emissions.
// The BFS tracing preserves provenance (which generator supplied which load),
// but the total_emissions_tco2 fields are updated to the balanced matrix intensities.
void update_emissions_from_matrix(
    std::vector<LoadCarbonResult>& load_carbon,
    std::vector<LoadCarbonResult>& dc_load_carbon,
    std::vector<BranchCarbonResult>& branch_carbon,
    std::vector<BranchCarbonResult>& dc_branch_carbon,
    std::vector<VSCCarbonResult>& vsc_carbon,
    std::vector<DCDCCarbonResult>& dcdc_carbon,
    std::vector<EnergyRouterCarbonResult>& energy_router_carbon,
    std::vector<StorageCarbonResult>& storage_carbon,
    const std::vector<double>& w,
    const HybridIDMap& id,
    const std::vector<FlowEdge>& edges,
    double loss_allocation_alpha) {
  for (auto& lc : load_carbon) {
    auto it = id.ac_loc.find(lc.bus);
    if (it != id.ac_loc.end() && it->second >= 0 &&
        it->second < static_cast<int>(w.size())) {
      lc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(it->second)];
      lc.total_emissions_tco2 =
          lc.carbon_intensity_tco2_mwh * std::max(lc.demand_mw, 0.0);
    }
  }

  for (auto& lc : dc_load_carbon) {
    auto it = id.dc_loc.find(lc.bus);
    if (it != id.dc_loc.end()) {
      int node_loc = id.n_ac + it->second;
      if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
        lc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
        lc.total_emissions_tco2 =
            lc.carbon_intensity_tco2_mwh * std::max(lc.demand_mw, 0.0);
      }
    }
  }

  for (auto& bc : branch_carbon) {
    if (const FlowEdge* edge =
            find_edge_by_component(edges, EdgeKind::ACBranch, bc.branch_index)) {
      bc.carbon_intensity_tco2_mwh =
          edge_loss_carbon_intensity(*edge, w, loss_allocation_alpha);
      bc.total_emissions_tco2 = bc.carbon_intensity_tco2_mwh * bc.loss_mw;
    }
  }

  for (auto& bc : dc_branch_carbon) {
    if (const FlowEdge* edge =
            find_edge_by_component(edges, EdgeKind::DCBranch, bc.branch_index)) {
      bc.carbon_intensity_tco2_mwh =
          edge_loss_carbon_intensity(*edge, w, loss_allocation_alpha);
      bc.total_emissions_tco2 = bc.carbon_intensity_tco2_mwh * bc.loss_mw;
    }
  }

  for (auto& vc : vsc_carbon) {
    if (const FlowEdge* edge =
            find_edge_by_component(edges, EdgeKind::VSC, vc.converter_index)) {
      vc.carbon_intensity_tco2_mwh =
          edge_loss_carbon_intensity(*edge, w, loss_allocation_alpha);
      vc.total_emissions_tco2 = vc.carbon_intensity_tco2_mwh * vc.loss_mw;
    }
  }

  for (auto& dc : dcdc_carbon) {
    if (const FlowEdge* edge =
            find_edge_by_component(edges, EdgeKind::DCDC, dc.converter_index)) {
      dc.carbon_intensity_tco2_mwh =
          edge_loss_carbon_intensity(*edge, w, loss_allocation_alpha);
      dc.total_emissions_tco2 = dc.carbon_intensity_tco2_mwh * dc.loss_mw;
    }
  }

  for (auto& er : energy_router_carbon) {
    if (er.loss_mw <= kTol) {
      er.total_emissions_tco2 = 0.0;
      er.carbon_intensity_tco2_mwh = 0.0;
      continue;
    }
    double weighted_emissions = 0.0;
    for (const auto& e : edges) {
      if (e.kind != EdgeKind::EnergyRouter ||
          e.component_index != er.router_index || e.loss_mw <= kTol) {
        continue;
      }
      weighted_emissions +=
          edge_loss_carbon_intensity(e, w, loss_allocation_alpha) * e.loss_mw;
    }
    er.total_emissions_tco2 = weighted_emissions;
    er.carbon_intensity_tco2_mwh =
        (er.loss_mw > kTol) ? (er.total_emissions_tco2 / er.loss_mw) : 0.0;
  }

  for (auto& sc : storage_carbon) {
    if (sc.p_mw < -kTol) {
      int node_loc = -1;
      if (sc.is_dc) {
        auto it = id.dc_loc.find(sc.bus);
        if (it != id.dc_loc.end()) node_loc = id.n_ac + it->second;
      } else {
        auto it = id.ac_loc.find(sc.bus);
        if (it != id.ac_loc.end()) node_loc = it->second;
      }
      if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
        sc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
        sc.total_emissions_tco2 = sc.carbon_intensity_tco2_mwh * std::abs(sc.p_mw);
      }
    }
  }
}

EmissionsSummary compute_tracing_summary(
    const std::vector<Source>& sources,
    const std::vector<UnifiedLoad>& loads,
    const std::vector<LoadCarbonResult>& all_load_carbon,
    const std::vector<BranchCarbonResult>& ac_branch_carbon,
    const std::vector<BranchCarbonResult>& dc_branch_carbon,
    const std::vector<VSCCarbonResult>& vsc_carbon,
    const std::vector<DCDCCarbonResult>& dcdc_carbon,
    const std::vector<EnergyRouterCarbonResult>& energy_router_carbon) {
  EmissionsSummary s;
  for (const auto& src : sources) {
    s.total_generation_emissions_tco2 += src.ef * src.p_mw;
  }
  for (size_t i = 0; i < all_load_carbon.size() && i < loads.size(); ++i) {
    if (loads[i].is_network_loss) {
      s.total_loss_emissions_tco2 += all_load_carbon[i].total_emissions_tco2;
    } else {
      s.total_load_emissions_tco2 += all_load_carbon[i].total_emissions_tco2;
    }
  }
  for (const auto& r : ac_branch_carbon) s.total_loss_emissions_tco2 += r.total_emissions_tco2;
  for (const auto& r : dc_branch_carbon) s.total_loss_emissions_tco2 += r.total_emissions_tco2;
  for (const auto& r : vsc_carbon) s.total_loss_emissions_tco2 += r.total_emissions_tco2;
  for (const auto& r : dcdc_carbon) s.total_loss_emissions_tco2 += r.total_emissions_tco2;
  for (const auto& r : energy_router_carbon) s.total_loss_emissions_tco2 += r.total_emissions_tco2;

  s.balance_error_tco2 = s.total_generation_emissions_tco2 -
                         s.total_load_emissions_tco2 -
                         s.total_loss_emissions_tco2;
  const double denom = std::max(s.total_generation_emissions_tco2, 1e-12);
  s.balance_error_pct = std::abs(s.balance_error_tco2) / denom * 100.0;
  return s;
}

EmissionsSummary compute_matrix_summary(const HybridPowerSystem& sys,
                                        const std::vector<Source>& sources,
                                        const std::vector<UnifiedLoad>& loads,
                                        const std::vector<FlowEdge>& edges,
                                        const std::vector<double>& w,
                                        double loss_allocation_alpha) {
  EmissionsSummary s;
  for (const auto& src : sources) {
    s.total_generation_emissions_tco2 += src.ef * src.p_mw;
  }

  for (const auto& ld : loads) {
    if (ld.node_loc < 0 || ld.node_loc >= static_cast<int>(w.size())) continue;
    const double emissions =
        w[static_cast<size_t>(ld.node_loc)] * std::max(ld.p_mw, 0.0);
    if (ld.is_network_loss) {
      s.total_loss_emissions_tco2 += emissions;
    } else {
      s.total_load_emissions_tco2 += emissions;
    }
  }

  for (const auto& e : edges) {
    if (e.loss_mw <= kTol) continue;
    s.total_loss_emissions_tco2 +=
        edge_loss_carbon_intensity(e, w, loss_allocation_alpha) * e.loss_mw;
  }

  s.balance_error_tco2 = s.total_generation_emissions_tco2 -
                         s.total_load_emissions_tco2 -
                         s.total_loss_emissions_tco2;
  const double denom = std::max(s.total_generation_emissions_tco2, 1e-12);
  s.balance_error_pct = std::abs(s.balance_error_tco2) / denom * 100.0;
  return s;
}

std::vector<double> compute_reported_bus_intensity(
    const HybridPowerSystem& sys,
    int node_count,
    const std::vector<Source>& sources,
    const std::vector<FlowEdge>& edges,
    const std::vector<UnifiedLoad>& loads,
    const std::vector<double>& matrix_w) {
  std::vector<double> reported = matrix_w;
  if (reported.size() < static_cast<size_t>(node_count)) {
    reported.resize(static_cast<size_t>(node_count), 0.0);
  }

  std::vector<double> source_power(static_cast<size_t>(node_count), 0.0);
  std::vector<double> source_emissions(static_cast<size_t>(node_count), 0.0);
  std::vector<double> incoming_power(static_cast<size_t>(node_count), 0.0);
  std::vector<double> outgoing_power(static_cast<size_t>(node_count), 0.0);
  std::vector<double> sink_power(static_cast<size_t>(node_count), 0.0);

  for (const auto& s : sources) {
    if (s.p_mw <= kTol || s.node_loc < 0 || s.node_loc >= node_count) continue;
    source_power[static_cast<size_t>(s.node_loc)] += s.p_mw;
    source_emissions[static_cast<size_t>(s.node_loc)] += s.p_mw * s.ef;
  }

  for (const auto& e : edges) {
    if (e.from_node_loc >= 0 && e.from_node_loc < node_count) {
      outgoing_power[static_cast<size_t>(e.from_node_loc)] +=
          std::max(e.send_mw, 0.0);
    }
    if (e.to_node_loc >= 0 && e.to_node_loc < node_count) {
      incoming_power[static_cast<size_t>(e.to_node_loc)] +=
          std::max(e.recv_mw, 0.0);
    }
  }

  for (const auto& ld : loads) {
    if (ld.p_mw <= kTol || ld.node_loc < 0 || ld.node_loc >= node_count) continue;
    sink_power[static_cast<size_t>(ld.node_loc)] += ld.p_mw;
  }
  for (int loc = 0; loc < node_count; ++loc) {
    const size_t i = static_cast<size_t>(loc);
    if (source_power[i] > kTol &&
        incoming_power[i] <= kTol &&
        sink_power[i] <= kTol) {
      reported[i] = source_emissions[i] / source_power[i];
      continue;
    }

    if (source_power[i] <= kTol &&
        incoming_power[i] <= kTol &&
        sink_power[i] <= kTol &&
        outgoing_power[i] <= kTol) {
      reported[i] = 0.0;
    }
  }

  return reported;
}

}  // namespace

CarbonAnalysisResult compute_carbon_analysis_canonical(
    const HybridPowerSystem& sys,
    const PowerFlowResult& pf_result,
    const CarbonAnalysisOptions& opt) {
  CarbonAnalysisResult result;
  if (!pf_result.converged) return result;

  HybridPowerSystem projected = sys;
  materialize_dc_storage(projected);
  if (projected.ac.buses.empty() && projected.dc.buses.empty()) return result;

  const HybridIDMap id = build_id_map(projected);
  const int node_count = id.n_ac + id.n_dc;

  auto ac_loads = build_unified_ac_loads(projected.ac, id, pf_result);
  auto dc_loads = build_unified_dc_loads(projected.dc, id);
  const auto storage_states = build_storage_states(projected, id);
  const auto storage_load_build = build_storage_charging_loads(storage_states);
  std::vector<UnifiedLoad> all_loads = ac_loads;
  all_loads.insert(all_loads.end(), dc_loads.begin(), dc_loads.end());
  const size_t storage_load_offset = all_loads.size();
  all_loads.insert(all_loads.end(),
                   storage_load_build.loads.begin(),
                   storage_load_build.loads.end());

  const auto edges = build_directed_flows(projected, pf_result, id);
  auto source_build = build_sources(projected, id, pf_result);
  add_external_grid_export_sinks(source_build, projected, all_loads, edges,
                                 node_count, opt);
  calibrate_balancing_sources(source_build, projected, all_loads, edges,
                              node_count, opt);
  const auto& sources = source_build.sources;
  result.carbon_sources.reserve(sources.size());
  for (const auto& source : sources) {
    result.carbon_sources.push_back(
        {source.id,
         source.source_type,
         source.component_index,
         source.label,
         source.bus,
         source.is_dc,
         source.scheduled_p_mw,
         source.p_mw,
         source.ef,
         source.is_balancing});
  }
  const auto power_balance = verify_node_power_balance(
      projected, sources, all_loads, edges, node_count, opt);
  result.power_balance_verified = power_balance.verified;
  result.max_node_power_balance_error_mw = power_balance.max_abs_error_mw;
  result.total_power_balance_error_mw = power_balance.total_abs_error_mw;
  result.node_power_balance_errors = power_balance.errors;

  if (opt.verbose) {
    HACDCPF_LOG_DEBUG("[CarbonAnalysis] {} sources, {} AC buses, {} DC buses, {} directed carriers, {} AC loads, {} DC loads, {} charging storage sinks",
              sources.size(), projected.ac.buses.size(), projected.dc.buses.size(),
              edges.size(), ac_loads.size(), dc_loads.size(), storage_load_build.loads.size());
  }

  const auto tracing = proportional_tracing(sources, edges, all_loads, node_count, opt);
  const auto all_load_carbon = build_load_carbon(all_loads, tracing, sources);
  for (size_t i = 0; i < all_loads.size(); ++i) {
    if (!all_loads[i].is_external_export) continue;
    result.total_external_export_mw += std::max(all_loads[i].p_mw, 0.0);
    result.total_external_export_emissions_tco2 +=
        all_load_carbon[i].total_emissions_tco2;
  }
  for (size_t i = 0; i < storage_load_offset; ++i) {
    if (!all_loads[i].report_as_load) continue;
    if (all_loads[i].is_dc) {
      result.dc_load_carbon.push_back(all_load_carbon[i]);
    } else {
      result.load_carbon.push_back(all_load_carbon[i]);
    }
  }

  result.storage_carbon.reserve(storage_states.size());
  for (size_t si = 0; si < storage_states.size(); ++si) {
    const LoadCarbonResult* charging_load = nullptr;
    if (si < storage_load_build.load_pos_for_storage.size()) {
      const int load_pos = storage_load_build.load_pos_for_storage[si];
      if (load_pos >= 0) {
        charging_load = &all_load_carbon[storage_load_offset +
                                         static_cast<size_t>(load_pos)];
      }
    }
    result.storage_carbon.push_back(
        build_storage_carbon_result(storage_states[si], charging_load));
  }

  const auto ef_map = build_ef_map(sources);
  std::unordered_map<int, size_t> ac_branch_edge_loc;
  std::unordered_map<int, size_t> dc_branch_edge_loc;
  std::unordered_map<int, size_t> vsc_edge_loc;
  std::unordered_map<int, size_t> dcdc_edge_loc;
  std::unordered_map<int, std::vector<size_t>> energy_router_edge_locs;
  for (size_t eidx = 0; eidx < edges.size(); ++eidx) {
    switch (edges[eidx].kind) {
      case EdgeKind::ACBranch:
        ac_branch_edge_loc.emplace(edges[eidx].component_index, eidx);
        break;
      case EdgeKind::ACSwitch:
      case EdgeKind::ACCircuitBreaker:
        break;
      case EdgeKind::DCBranch:
        dc_branch_edge_loc.emplace(edges[eidx].component_index, eidx);
        break;
      case EdgeKind::VSC:
        vsc_edge_loc.emplace(edges[eidx].component_index, eidx);
        break;
      case EdgeKind::DCDC:
        dcdc_edge_loc.emplace(edges[eidx].component_index, eidx);
        break;
      case EdgeKind::EnergyRouter:
        energy_router_edge_locs[edges[eidx].component_index].push_back(eidx);
        break;
    }
  }

  result.branch_carbon.reserve(projected.ac.branches.size());
  for (size_t l = 0; l < projected.ac.branches.size(); ++l) {
    const auto& br = projected.ac.branches[l];
    double loss_mw = 0.0;
    if (l < pf_result.branch_flows.size()) {
      loss_mw = std::max(pf_result.branch_flows[l].pf_mw + pf_result.branch_flows[l].pt_mw, 0.0);
    }
    std::unordered_map<int, double> alloc;
    auto it = ac_branch_edge_loc.find(br.index);
    if (it != ac_branch_edge_loc.end()) alloc = tracing.edge_loss_alloc[it->second];
    result.branch_carbon.push_back(
        build_branch_carbon_result(br.index, br.from_bus, br.to_bus, loss_mw, alloc, ef_map));
  }

  result.dc_branch_carbon.reserve(projected.dc.branches.size());
  const double base_mva = (projected.base_mva > 0.0) ? projected.base_mva : 100.0;
  for (const auto& br : projected.dc.branches) {
    double loss_mw = 0.0;
    auto itf = id.dc_loc.find(br.from_bus);
    auto itt = id.dc_loc.find(br.to_bus);
    if (br.in_service && br.r_pu > kTol && itf != id.dc_loc.end() && itt != id.dc_loc.end() &&
        itf->second < static_cast<int>(pf_result.vdc.size()) &&
        itt->second < static_cast<int>(pf_result.vdc.size())) {
      const double v_from = pf_result.vdc[static_cast<size_t>(itf->second)];
      const double v_to = pf_result.vdc[static_cast<size_t>(itt->second)];
      const double i_pu = (v_from - v_to) / br.r_pu;
      const double p_from_mw = base_mva * v_from * i_pu;
      const double p_to_mw = -base_mva * v_to * i_pu;
      loss_mw = std::max(p_from_mw + p_to_mw, 0.0);
    }
    std::unordered_map<int, double> alloc;
    auto it = dc_branch_edge_loc.find(br.index);
    if (it != dc_branch_edge_loc.end()) alloc = tracing.edge_loss_alloc[it->second];
    result.dc_branch_carbon.push_back(
        build_branch_carbon_result(br.index, br.from_bus, br.to_bus, loss_mw, alloc, ef_map));
  }

  std::unordered_map<int, VSCTransfer> vsc_transfer_by_index;
  for (const auto& tr : pf_result.vsc_transfers) vsc_transfer_by_index.emplace(tr.index, tr);
  result.vsc_carbon.reserve(projected.vsc_converters.size());
  for (const auto& c : projected.vsc_converters) {
    VSCTransfer tr;
    tr.index = c.index;
    tr.bus_ac = c.bus_ac;
    tr.bus_dc = c.bus_dc;
    auto tr_it = vsc_transfer_by_index.find(c.index);
    if (tr_it != vsc_transfer_by_index.end()) tr = tr_it->second;

    const FlowEdge* edge = nullptr;
    auto it = vsc_edge_loc.find(c.index);
    if (it != vsc_edge_loc.end()) edge = &edges[it->second];
    const std::unordered_map<int, double> alloc =
        (it != vsc_edge_loc.end()) ? tracing.edge_loss_alloc[it->second]
                                   : std::unordered_map<int, double>{};
    result.vsc_carbon.push_back(build_vsc_carbon_result(tr, edge, ef_map, alloc));
  }

  std::unordered_map<int, DCDCTransfer> dcdc_transfer_by_index;
  for (const auto& tr : pf_result.dcdc_transfers) dcdc_transfer_by_index.emplace(tr.index, tr);
  result.dcdc_carbon.reserve(projected.dc.dcdc_converters.size());
  for (const auto& c : projected.dc.dcdc_converters) {
    DCDCTransfer tr;
    tr.index = c.index;
    tr.bus_in = c.bus_in;
    tr.bus_out = c.bus_out;
    auto tr_it = dcdc_transfer_by_index.find(c.index);
    if (tr_it != dcdc_transfer_by_index.end()) tr = tr_it->second;

    const FlowEdge* edge = nullptr;
    auto it = dcdc_edge_loc.find(c.index);
    if (it != dcdc_edge_loc.end()) edge = &edges[it->second];
    const std::unordered_map<int, double> alloc =
        (it != dcdc_edge_loc.end()) ? tracing.edge_loss_alloc[it->second]
                                    : std::unordered_map<int, double>{};
    result.dcdc_carbon.push_back(build_dcdc_carbon_result(tr, edge, ef_map, alloc));
  }

  const auto router_ports_by_index =
      build_energy_router_port_states(projected, pf_result, id);
  result.energy_router_carbon.reserve(projected.energy_routers.size());
  for (const auto& er : projected.energy_routers) {
    std::unordered_map<int, double> alloc;
    auto eit = energy_router_edge_locs.find(er.index);
    if (eit != energy_router_edge_locs.end()) {
      for (size_t edge_loc : eit->second) {
        accumulate_contribution_map(alloc, tracing.edge_loss_alloc[edge_loc]);
      }
    }

    const auto pit = router_ports_by_index.find(er.index);
    const std::vector<RouterPortState> empty_ports;
    const auto& ports = (pit != router_ports_by_index.end()) ? pit->second : empty_ports;
    result.energy_router_carbon.push_back(
        build_energy_router_carbon_result(er.index, ports, ef_map, alloc));
  }

  result.generator_loss_allocation = tracing.gen_loss_total;

  result.tracing_summary = compute_tracing_summary(
      sources, all_loads, all_load_carbon, result.branch_carbon,
      result.dc_branch_carbon, result.vsc_carbon, result.dcdc_carbon,
      result.energy_router_carbon);

  // Solve the matrix-based carbon flow equations
  MatrixResult mat;
  if (result.power_balance_verified) {
    mat = solve_carbon_matrix(projected, sources, edges, all_loads, opt);
  }
  result.matrix_solved = mat.solved;
  result.matrix_residual = mat.residual;
  result.matrix_relative_residual = mat.relative_residual;
  result.matrix_condition_estimate = mat.condition_estimate;
  result.matrix_rank = mat.rank;
  if (mat.solved) {
    const auto reported_w =
        compute_reported_bus_intensity(projected, node_count, sources, edges, all_loads, mat.w);
    result.bus_carbon.reserve(projected.ac.buses.size());
    for (size_t i = 0; i < projected.ac.buses.size(); ++i) {
      result.bus_carbon.push_back(
          {projected.ac.buses[i].index, reported_w[i]});
    }
    result.dc_bus_carbon.reserve(projected.dc.buses.size());
    for (size_t i = 0; i < projected.dc.buses.size(); ++i) {
      result.dc_bus_carbon.push_back(
          {projected.dc.buses[i].index,
           reported_w[projected.ac.buses.size() + i]});
    }
    result.matrix_summary = compute_matrix_summary(projected, sources, all_loads,
                                                   edges, mat.w,
                                                   opt.loss_allocation_alpha);
    result.total_external_export_emissions_tco2 = 0.0;
    for (const auto& load : all_loads) {
      if (!load.is_external_export || load.node_loc < 0 ||
          load.node_loc >= static_cast<int>(mat.w.size())) {
        continue;
      }
      result.total_external_export_emissions_tco2 +=
          mat.w[static_cast<size_t>(load.node_loc)] * std::max(load.p_mw, 0.0);
    }

    // Update all carbon results to use matrix-solved intensities
    update_emissions_from_matrix(
        result.load_carbon, result.dc_load_carbon,
        result.branch_carbon, result.dc_branch_carbon,
        result.vsc_carbon, result.dcdc_carbon,
        result.energy_router_carbon, result.storage_carbon,
        mat.w, id, edges, opt.loss_allocation_alpha);
  }

  for (const auto& sc : result.storage_carbon) {
    if (sc.p_mw < -kTol) {
      result.total_storage_charge_emissions_tco2 += sc.total_emissions_tco2;
    } else if (sc.p_mw > kTol) {
      result.total_storage_discharge_emissions_tco2 += sc.total_emissions_tco2;
    }
  }

  result.tracing_verified =
      result.power_balance_verified && result.matrix_solved &&
      result.tracing_summary.balance_error_pct < opt.verification_tol * 100.0 &&
      result.matrix_summary.balance_error_pct < opt.verification_tol * 100.0;

  if (opt.verbose) {
    [[maybe_unused]] const auto& ts = result.tracing_summary;
    HACDCPF_LOG_DEBUG("[CarbonAnalysis] Tracing Summary:");
    HACDCPF_LOG_DEBUG("  Generation emissions: {} tCO2", ts.total_generation_emissions_tco2);
    HACDCPF_LOG_DEBUG("  Load emissions:       {} tCO2", ts.total_load_emissions_tco2);
    HACDCPF_LOG_DEBUG("  Loss emissions:       {} tCO2", ts.total_loss_emissions_tco2);
    HACDCPF_LOG_DEBUG("  Balance error:        {} %", ts.balance_error_pct);
    if (result.matrix_solved) {
      [[maybe_unused]] const auto& ms = result.matrix_summary;
      HACDCPF_LOG_DEBUG("[CarbonAnalysis] Matrix Summary:");
      HACDCPF_LOG_DEBUG("  Load emissions:  {} tCO2", ms.total_load_emissions_tco2);
      HACDCPF_LOG_DEBUG("  Loss emissions:  {} tCO2", ms.total_loss_emissions_tco2);
      HACDCPF_LOG_DEBUG("  Balance error:   {} %", ms.balance_error_pct);
      HACDCPF_LOG_DEBUG("  Residual:        {}", result.matrix_residual);
    }
  }

  return result;
}

int canonical_ac_bus_to_external(const std::optional<BusMergeMap>& map,
                                 int canonical_bus) {
  if (!map.has_value() || canonical_bus <= 0) return canonical_bus;
  const int internal_pos = canonical_bus - 1;
  if (internal_pos < 0 || internal_pos >= static_cast<int>(map->int_to_ext.size())) {
    return canonical_bus;
  }
  const int external_bus = map->int_to_ext[static_cast<size_t>(internal_pos)];
  return external_bus > 0 ? external_bus : canonical_bus;
}

int canonical_dc_bus_to_external(const std::vector<int>& map,
                                 int canonical_bus) {
  if (canonical_bus <= 0 ||
      canonical_bus > static_cast<int>(map.size())) {
    return canonical_bus;
  }
  return map[static_cast<size_t>(canonical_bus - 1)];
}

void remap_carbon_result_to_original(
    CarbonAnalysisResult& result,
    const HybridPowerSystem& original,
    const CanonicalCarbonInput& input) {
  if (input.ac_bus_map.has_value()) {
    std::vector<BusCarbonResult> original_bus_carbon;
    original_bus_carbon.reserve(original.ac.buses.size());
    for (const auto& bus : original.ac.buses) {
      double intensity = 0.0;
      const auto it = input.ac_bus_map->ext_to_int.find(bus.index);
      if (it != input.ac_bus_map->ext_to_int.end() && it->second >= 0 &&
          it->second < static_cast<int>(result.bus_carbon.size())) {
        intensity = result.bus_carbon[static_cast<size_t>(it->second)]
                        .carbon_intensity_tco2_mwh;
      }
      original_bus_carbon.push_back({bus.index, intensity});
    }
    result.bus_carbon = std::move(original_bus_carbon);
  }

  std::unordered_map<int, double> dc_intensity_by_canonical_bus;
  dc_intensity_by_canonical_bus.reserve(result.dc_bus_carbon.size());
  for (const auto& bus : result.dc_bus_carbon) {
    dc_intensity_by_canonical_bus.emplace(
        bus.bus_index, bus.carbon_intensity_tco2_mwh);
  }
  std::vector<BusCarbonResult> original_dc_bus_carbon;
  original_dc_bus_carbon.reserve(original.dc.buses.size());
  for (size_t i = 0; i < original.dc.buses.size(); ++i) {
    const int canonical_bus = static_cast<int>(i) + 1;
    const auto intensity = dc_intensity_by_canonical_bus.find(canonical_bus);
    original_dc_bus_carbon.push_back(
        {original.dc.buses[i].index,
         intensity != dc_intensity_by_canonical_bus.end()
             ? intensity->second
             : 0.0});
  }
  result.dc_bus_carbon = std::move(original_dc_bus_carbon);

  std::unordered_map<int, int> original_load_bus;
  original_load_bus.reserve(original.ac.loads.size());
  for (const auto& load : original.ac.loads) {
    original_load_bus.emplace(load.index, load.bus);
  }
  for (auto& load : result.load_carbon) {
    const auto it = original_load_bus.find(load.load_index);
    load.bus = (it != original_load_bus.end())
                   ? it->second
                   : canonical_ac_bus_to_external(input.ac_bus_map, load.bus);
  }
  std::unordered_map<int, int> original_dc_load_bus;
  original_dc_load_bus.reserve(original.dc.loads.size());
  for (const auto& load : original.dc.loads) {
    original_dc_load_bus.emplace(load.index, load.bus);
  }
  for (auto& load : result.dc_load_carbon) {
    const auto it = original_dc_load_bus.find(load.load_index);
    load.bus = (it != original_dc_load_bus.end())
                   ? it->second
                   : canonical_dc_bus_to_external(
                         input.dc_canonical_to_original, load.bus);
  }
  for (auto& storage : result.storage_carbon) {
    if (storage.is_dc) {
      storage.bus = canonical_dc_bus_to_external(
          input.dc_canonical_to_original, storage.bus);
    } else {
      storage.bus = canonical_ac_bus_to_external(input.ac_bus_map, storage.bus);
    }
  }
  for (auto& source : result.carbon_sources) {
    if (source.is_dc) {
      source.bus = canonical_dc_bus_to_external(
          input.dc_canonical_to_original, source.bus);
    } else {
      source.bus = canonical_ac_bus_to_external(input.ac_bus_map, source.bus);
    }
  }
  for (auto& converter : result.vsc_carbon) {
    converter.bus_ac =
        canonical_ac_bus_to_external(input.ac_bus_map, converter.bus_ac);
    converter.bus_dc = canonical_dc_bus_to_external(
        input.dc_canonical_to_original, converter.bus_dc);
  }
  for (auto& converter : result.dcdc_carbon) {
    converter.bus_in = canonical_dc_bus_to_external(
        input.dc_canonical_to_original, converter.bus_in);
    converter.bus_out = canonical_dc_bus_to_external(
        input.dc_canonical_to_original, converter.bus_out);
  }
  for (auto& error : result.node_power_balance_errors) {
    if (error.is_dc) {
      error.bus_index = canonical_dc_bus_to_external(
          input.dc_canonical_to_original, error.bus_index);
    } else {
      error.bus_index =
          canonical_ac_bus_to_external(input.ac_bus_map, error.bus_index);
    }
  }

  std::unordered_map<int, std::pair<int, int>> original_dc_branch_buses;
  original_dc_branch_buses.reserve(original.dc.branches.size());
  for (const auto& branch : original.dc.branches) {
    original_dc_branch_buses.emplace(
        branch.index, std::pair{branch.from_bus, branch.to_bus});
  }
  for (auto& branch : result.dc_branch_carbon) {
    const auto it = original_dc_branch_buses.find(branch.branch_index);
    if (it != original_dc_branch_buses.end()) {
      branch.from_bus = it->second.first;
      branch.to_bus = it->second.second;
    } else {
      branch.from_bus = canonical_dc_bus_to_external(
          input.dc_canonical_to_original, branch.from_bus);
      branch.to_bus = canonical_dc_bus_to_external(
          input.dc_canonical_to_original, branch.to_bus);
    }
  }

  std::vector<BranchCarbonResult> original_branch_carbon;
  original_branch_carbon.reserve(original.ac.branches.size());
  for (size_t original_pos = 0; original_pos < original.ac.branches.size(); ++original_pos) {
    const auto& branch = original.ac.branches[original_pos];
    BranchCarbonResult carbon;
    carbon.branch_index = branch.index;
    carbon.from_bus = branch.from_bus;
    carbon.to_bus = branch.to_bus;
    const int canonical_pos =
        (original_pos < input.original_branch_to_canonical.size())
            ? input.original_branch_to_canonical[original_pos]
            : -1;
    if (canonical_pos >= 0 &&
        canonical_pos < static_cast<int>(result.branch_carbon.size())) {
      carbon = result.branch_carbon[static_cast<size_t>(canonical_pos)];
      carbon.branch_index = branch.index;
      carbon.from_bus = branch.from_bus;
      carbon.to_bus = branch.to_bus;
    }
    original_branch_carbon.push_back(std::move(carbon));
  }
  result.branch_carbon = std::move(original_branch_carbon);
}

CarbonAnalysisResult compute_carbon_analysis(const HybridPowerSystem& sys,
                                             const PowerFlowResult& pf_result,
                                             const CarbonAnalysisOptions& opt) {
  if (!pf_result.converged) return {};
  CanonicalCarbonInput input = build_canonical_carbon_input(sys, pf_result);
  CarbonAnalysisResult result =
      compute_carbon_analysis_canonical(input.system, input.power_flow, opt);
  remap_carbon_result_to_original(result, sys, input);
  return result;
}

CarbonAnalysisResult compute_carbon_analysis(const HybridPowerSystem& sys,
                                             const PowerFlowOptions& pf_opt,
                                             const CarbonAnalysisOptions& ca_opt) {
  const auto pf = ::hacdcpf::solve_power_flow(sys, pf_opt);
  return compute_carbon_analysis(sys, pf, ca_opt);
}

}  // namespace hacdcpf::analysis

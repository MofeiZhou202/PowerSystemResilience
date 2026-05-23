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

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/detail/logging.hpp"
#include "hacdcpf/model/enums/grid_enums.hpp"
#include "hacdcpf/model/enums/storage_enums.hpp"

namespace hacdcpf::analysis {

namespace {

constexpr double kTol = 1e-9;

enum class EdgeKind {
  ACBranch,
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
  bool solved{false};
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

std::vector<UnifiedLoad> build_unified_ac_loads(const ACSystem& ac,
                                                const HybridIDMap& id) {
  std::vector<UnifiedLoad> out;
  std::unordered_set<int> buses_with_explicit_load;

  for (const auto& ld : ac.loads) {
    if (!ld.in_service || ld.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, ld.bus);
    if (node_loc < 0) continue;
    out.push_back({ld.index, ld.bus, false, node_loc, ld.p_mw, ld.q_mvar});
    buses_with_explicit_load.insert(ld.bus);
  }

  int syn_id = -1;
  for (const auto& b : ac.buses) {
    if (b.pd_mw <= kTol) continue;
    if (buses_with_explicit_load.count(b.index) != 0) continue;
    const int node_loc = global_bus_loc(id, false, b.index);
    if (node_loc < 0) continue;
    out.push_back({syn_id--, b.index, false, node_loc, b.pd_mw, b.qd_mvar});
  }

  return out;
}

std::vector<UnifiedLoad> build_unified_dc_loads(const DCSystem& dc,
                                                const HybridIDMap& id) {
  std::vector<UnifiedLoad> out;
  std::unordered_set<int> buses_with_explicit_load;

  for (const auto& ld : dc.loads) {
    if (!ld.in_service || ld.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, true, ld.bus);
    if (node_loc < 0) continue;
    out.push_back({ld.index, ld.bus, true, node_loc, ld.p_mw, 0.0});
    buses_with_explicit_load.insert(ld.bus);
  }

  int syn_id = -1;
  for (const auto& b : dc.buses) {
    if (b.pd_mw <= kTol) continue;
    if (buses_with_explicit_load.count(b.index) != 0) continue;
    const int node_loc = global_bus_loc(id, true, b.index);
    if (node_loc < 0) continue;
    out.push_back({syn_id--, b.index, true, node_loc, b.pd_mw, 0.0});
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

  const double base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;
  for (const auto& br : sys.dc.branches) {
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
                                const HybridIDMap& id) {
  SourceBuildResult out;
  out.sources.reserve(sys.ac.generators.size() + sys.ac.static_generators.size() +
                      sys.ac.renewable_gens.size() + sys.ac.pv_systems.size() +
                      sys.dc.static_generators.size() +
                      sys.ac.storage.size() + sys.dc.storage.size());

  int src_id = 0;
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const int node_loc = global_bus_loc(id, false, g.bus);
    if (node_loc < 0) continue;

    double p_mw = std::max(g.pg_mw, 0.0);
    if (g.is_slack && out.slack_source_pos < 0) {
      p_mw = 0.0;
      out.slack_source_pos = static_cast<int>(out.sources.size());
    }
    out.sources.push_back({src_id++, g.bus, false, node_loc, p_mw,
                           g.emission_factor_tco2_mwh});
  }

  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, false, sg.bus);
    const double p_mw = sg.p_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    out.sources.push_back({src_id++, sg.bus, false, node_loc, p_mw,
                           sg.co2_emission_rate});
  }

  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service || rg.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, rg.bus);
    if (node_loc < 0) continue;
    out.sources.push_back({src_id++, rg.bus, false, node_loc, rg.p_mw, 0.0});
  }

  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service || pv.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, pv.bus);
    if (node_loc < 0) continue;
    out.sources.push_back({src_id++, pv.bus, false, node_loc, pv.p_mw, 0.0});
  }

  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, true, sg.bus);
    const double p_mw = sg.p_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    out.sources.push_back({src_id++, sg.bus, true, node_loc, p_mw,
                           sg.co2_emission_rate});
  }

  // DC fixed generators and PV arrays: zero-emission sources until
  // a dedicated emission model is added for these types.
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const int node_loc = global_bus_loc(id, true, sg.bus);
    const double p_mw = sg.p_set_mw * sg.scaling;
    if (node_loc < 0 || p_mw <= kTol) continue;
    out.sources.push_back({src_id++, sg.bus, true, node_loc, p_mw, 0.0});
  }

  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    const int node_loc = global_bus_loc(id, true, pv.bus);
    const double p_mw = pv.p_set_mw;
    if (node_loc < 0 || p_mw <= kTol) continue;
    out.sources.push_back({src_id++, pv.bus, true, node_loc, p_mw, 0.0});
  }

  for (const auto& st : sys.ac.storage) {
    if (!st.in_service || st.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, false, st.bus);
    if (node_loc < 0) continue;
    out.sources.push_back({src_id++, st.bus, false, node_loc, st.p_mw,
                           st.soc_carbon_intensity_tco2_mwh});
  }

  for (const auto& st : sys.dc.storage) {
    if (!st.in_service || st.p_mw <= kTol) continue;
    const int node_loc = global_bus_loc(id, true, st.bus);
    if (node_loc < 0) continue;
    out.sources.push_back({src_id++, st.bus, true, node_loc, st.p_mw,
                           st.soc_carbon_intensity_tco2_mwh});
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
  for (const auto& ld : all_loads) total_load += ld.p_mw;
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

TracingResult proportional_tracing(const std::vector<Source>& sources,
                                   const std::vector<FlowEdge>& edges,
                                   const std::vector<UnifiedLoad>& loads,
                                   int node_count,
                                   const CarbonAnalysisOptions& opt) {
  std::vector<std::vector<Upstream>> upstream(static_cast<size_t>(node_count));
  for (int eidx = 0; eidx < static_cast<int>(edges.size()); ++eidx) {
    const auto& e = edges[static_cast<size_t>(eidx)];
    if (e.to_node_loc < 0 || e.to_node_loc >= node_count) continue;
    upstream[static_cast<size_t>(e.to_node_loc)].push_back(
        {e.from_node_loc, eidx, e.recv_mw});
  }

  std::vector<std::unordered_map<int, double>> bus_gen_inj(static_cast<size_t>(node_count));
  for (const auto& s : sources) {
    if (s.p_mw <= kTol || s.node_loc < 0 || s.node_loc >= node_count) continue;
    bus_gen_inj[static_cast<size_t>(s.node_loc)][s.id] += s.p_mw;
  }

  std::vector<double> total_inflow(static_cast<size_t>(node_count), 0.0);
  for (int n = 0; n < node_count; ++n) {
    for (const auto& [sid, mw] : bus_gen_inj[static_cast<size_t>(n)]) {
      total_inflow[static_cast<size_t>(n)] += mw;
    }
    for (const auto& up : upstream[static_cast<size_t>(n)]) {
      total_inflow[static_cast<size_t>(n)] += up.recv_mw;
    }
  }

  TracingResult result;
  result.load_supply.resize(loads.size());
  result.edge_loss_alloc.resize(edges.size());

  for (size_t li = 0; li < loads.size(); ++li) {
    const auto& ld = loads[li];
    if (ld.p_mw <= kTol || ld.node_loc < 0 || ld.node_loc >= node_count) continue;

    const double bus_total = total_inflow[static_cast<size_t>(ld.node_loc)];
    if (bus_total <= kTol) continue;

    using BFSItem = std::pair<int, double>;
    std::queue<BFSItem> q;
    q.push({ld.node_loc, ld.p_mw / bus_total});

    std::unordered_set<int> visited_edges;
    int depth = 0;
    const int max_depth =
        (opt.max_tracing_depth > 0) ? opt.max_tracing_depth : node_count + 1;

    while (!q.empty() && depth < max_depth) {
      const int level_size = static_cast<int>(q.size());
      for (int k = 0; k < level_size; ++k) {
        const auto [node_loc, frac] = q.front();
        q.pop();

        for (const auto& [sid, mw] : bus_gen_inj[static_cast<size_t>(node_loc)]) {
          const double contrib = frac * mw;
          if (contrib < opt.min_contribution_mw) continue;
          result.load_supply[li][sid] += contrib;
        }

        for (const auto& up : upstream[static_cast<size_t>(node_loc)]) {
          if (visited_edges.count(up.edge_loc) != 0) continue;
          visited_edges.insert(up.edge_loc);

          if (total_inflow[static_cast<size_t>(node_loc)] <= kTol) continue;
          const double edge_frac = up.recv_mw / total_inflow[static_cast<size_t>(node_loc)];
          const double upstream_frac = frac * edge_frac;
          if (upstream_frac < kTol) continue;

          const double upstream_total = total_inflow[static_cast<size_t>(up.from_node_loc)];
          if (upstream_total <= kTol) continue;
          q.push({up.from_node_loc,
                  upstream_frac * total_inflow[static_cast<size_t>(node_loc)] / upstream_total});
        }
      }
      ++depth;
    }
  }

  for (size_t eidx = 0; eidx < edges.size(); ++eidx) {
    const auto& e = edges[eidx];
    if (e.loss_mw <= kTol || e.from_node_loc < 0 || e.from_node_loc >= node_count) continue;

    using BFSItem = std::pair<int, double>;
    std::queue<BFSItem> q;
    q.push({e.from_node_loc, 1.0});

    std::unordered_set<int> visited;
    int depth = 0;
    const int max_depth =
        (opt.max_tracing_depth > 0) ? opt.max_tracing_depth : node_count + 1;

    while (!q.empty() && depth < max_depth) {
      const int level_size = static_cast<int>(q.size());
      for (int k = 0; k < level_size; ++k) {
        const auto [node_loc, frac] = q.front();
        q.pop();
        const double node_total = total_inflow[static_cast<size_t>(node_loc)];
        if (node_total <= kTol) continue;

        for (const auto& [sid, mw] : bus_gen_inj[static_cast<size_t>(node_loc)]) {
          const double share = frac * (mw / node_total) * e.loss_mw;
          if (share < opt.min_contribution_mw) continue;
          result.edge_loss_alloc[eidx][sid] += share;
          result.gen_loss_total[sid] += share;
        }

        for (const auto& up : upstream[static_cast<size_t>(node_loc)]) {
          const int key = up.edge_loc * std::max(node_count, 1) + up.from_node_loc;
          if (visited.count(key) != 0) continue;
          visited.insert(key);
          if (node_total <= kTol) continue;
          const double edge_frac = up.recv_mw / node_total;
          const double upstream_frac = frac * edge_frac;
          if (upstream_frac < kTol) continue;
          q.push({up.from_node_loc, upstream_frac});
        }
      }
      ++depth;
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

  const double alpha = opt.loss_allocation_alpha;
  Eigen::VectorXd Pin = Eigen::VectorXd::Zero(node_count);
  Eigen::VectorXd Pout = Eigen::VectorXd::Zero(node_count);

  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> in_triplets;
  in_triplets.reserve(edges.size());

  for (const auto& e : edges) {
    if (e.from_node_loc < 0 || e.to_node_loc < 0 ||
        e.from_node_loc >= node_count || e.to_node_loc >= node_count) {
      continue;
    }
    const double transported = std::max(e.send_mw - alpha * e.loss_mw, 0.0);
    Pin(e.to_node_loc) += transported;
    Pout(e.from_node_loc) += e.send_mw;
    Pout(e.to_node_loc) += (1.0 - alpha) * e.loss_mw;
    if (transported > kTol) {
      in_triplets.emplace_back(e.to_node_loc, e.from_node_loc, transported);
    }
  }

  for (const auto& s : sources) {
    if (s.p_mw <= kTol || s.node_loc < 0 || s.node_loc >= node_count) continue;
    Pin(s.node_loc) += s.p_mw;
  }

  for (const auto& ld : loads) {
    if (ld.p_mw <= kTol || ld.node_loc < 0 || ld.node_loc >= node_count) continue;
    Pout(ld.node_loc) += ld.p_mw;
  }

  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const int loc = static_cast<int>(i);
    if (loc >= 0 && loc < node_count) {
      Pout(loc) += sys.ac.buses[i].gs_mw;
    }
  }

  // Fix for transit nodes with injection but negligible outflow edges:
  // If Pout(i) << Pin(i), power balance is broken (uniform DC voltages).
  // Use Pin as the effective Pout so carbon intensity propagates correctly.
  constexpr double kBalanceTol = 0.1;
  constexpr double kMinPower = 0.01;
  for (int i = 0; i < node_count; ++i) {
    if (Pin(i) > kMinPower && Pout(i) < kBalanceTol * Pin(i)) {
      Pout(i) = Pin(i);
    }
  }

  Eigen::SparseMatrix<double> A_in(node_count, node_count);
  A_in.setFromTriplets(in_triplets.begin(), in_triplets.end());

  Eigen::SparseMatrix<double> A(node_count, node_count);
  std::vector<Triplet> a_triplets;
  a_triplets.reserve(static_cast<size_t>(node_count + static_cast<int>(in_triplets.size())));
  for (int i = 0; i < node_count; ++i) {
    const double diag = (std::abs(Pout(i)) > opt.regularization_eps) ? Pout(i)
                                                                     : opt.regularization_eps;
    a_triplets.emplace_back(i, i, diag);
  }
  for (const auto& t : in_triplets) {
    a_triplets.emplace_back(t.row(), t.col(), -t.value());
  }
  A.setFromTriplets(a_triplets.begin(), a_triplets.end());

  Eigen::VectorXd b = Eigen::VectorXd::Zero(node_count);
  Eigen::VectorXd gen_at_bus = Eigen::VectorXd::Zero(node_count);
  for (const auto& s : sources) {
    if (s.node_loc < 0 || s.node_loc >= node_count) continue;
    b(s.node_loc) += s.ef * s.p_mw;
    gen_at_bus(s.node_loc) += s.p_mw;
  }

  Eigen::MatrixXd A_dense = Eigen::MatrixXd(A);
  std::vector<std::vector<int>> neighbors(static_cast<size_t>(node_count));
  for (const auto& e : edges) {
    if (e.from_node_loc < 0 || e.to_node_loc < 0) continue;
    neighbors[static_cast<size_t>(e.from_node_loc)].push_back(e.to_node_loc);
    neighbors[static_cast<size_t>(e.to_node_loc)].push_back(e.from_node_loc);
  }

  std::vector<double> load_at_bus(static_cast<size_t>(node_count), 0.0);
  for (const auto& ld : loads) {
    if (ld.node_loc >= 0 && ld.node_loc < node_count) {
      load_at_bus[static_cast<size_t>(ld.node_loc)] += ld.p_mw;
    }
  }
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    load_at_bus[i] += sys.ac.buses[i].gs_mw;
  }

  for (int i = 0; i < node_count; ++i) {
    if (load_at_bus[static_cast<size_t>(i)] < 1e-8 && gen_at_bus(i) < 1e-8) {
      const double row_norm = A_dense.row(i).norm();
      if (row_norm < 1e-6) {
        A_dense.row(i).setZero();
        b(i) = 0.0;
        const auto& nbrs = neighbors[static_cast<size_t>(i)];
        if (nbrs.size() == 1) {
          A_dense(i, i) = 1.0;
          A_dense(i, nbrs[0]) = -1.0;
        } else if (!nbrs.empty()) {
          A_dense(i, i) = static_cast<double>(nbrs.size());
          for (int n : nbrs) A_dense(i, n) = -1.0;
        } else {
          A_dense(i, i) = 1.0;
        }
      }
    }
  }

  const Eigen::VectorXd w = A_dense.colPivHouseholderQr().solve(b);
  if (!w.allFinite()) {
    return res;
  }

  const Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();
  res.residual = (A_sparse * w - b).norm();
  res.solved = true;
  for (int i = 0; i < node_count; ++i) res.w[static_cast<size_t>(i)] = w(i);

  if (opt.verbose) {
    HACDCPF_LOG_DEBUG("[CarbonAnalysis] Matrix solve residual: {}", res.residual);
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
    const HybridIDMap& id) {
  for (auto& lc : load_carbon) {
    auto it = id.ac_loc.find(lc.bus);
    if (it != id.ac_loc.end() && it->second >= 0 &&
        it->second < static_cast<int>(w.size())) {
      lc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(it->second)];
      lc.total_emissions_tco2 = lc.carbon_intensity_tco2_mwh * lc.demand_mw;
    }
  }

  for (auto& lc : dc_load_carbon) {
    auto it = id.dc_loc.find(lc.bus);
    if (it != id.dc_loc.end()) {
      int node_loc = id.n_ac + it->second;
      if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
        lc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
        lc.total_emissions_tco2 = lc.carbon_intensity_tco2_mwh * lc.demand_mw;
      }
    }
  }

  for (auto& bc : branch_carbon) {
    auto it = id.ac_loc.find(bc.from_bus);
    if (it != id.ac_loc.end() && it->second >= 0 &&
        it->second < static_cast<int>(w.size())) {
      bc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(it->second)];
      bc.total_emissions_tco2 = bc.carbon_intensity_tco2_mwh * bc.loss_mw;
    }
  }

  for (auto& bc : dc_branch_carbon) {
    auto it = id.dc_loc.find(bc.from_bus);
    if (it != id.dc_loc.end()) {
      int node_loc = id.n_ac + it->second;
      if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
        bc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
        bc.total_emissions_tco2 = bc.carbon_intensity_tco2_mwh * bc.loss_mw;
      }
    }
  }

  for (auto& vc : vsc_carbon) {
    int node_loc = -1;
    if (vc.ac_to_dc) {
      auto it = id.ac_loc.find(vc.bus_ac);
      if (it != id.ac_loc.end()) node_loc = it->second;
    } else {
      auto it = id.dc_loc.find(vc.bus_dc);
      if (it != id.dc_loc.end()) node_loc = id.n_ac + it->second;
    }
    if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
      vc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
      vc.total_emissions_tco2 = vc.carbon_intensity_tco2_mwh * vc.loss_mw;
    }
  }

  for (auto& dc : dcdc_carbon) {
    int node_loc = -1;
    int source_bus = dc.input_to_output ? dc.bus_in : dc.bus_out;
    auto it = id.dc_loc.find(source_bus);
    if (it != id.dc_loc.end()) node_loc = id.n_ac + it->second;
    if (node_loc >= 0 && node_loc < static_cast<int>(w.size())) {
      dc.carbon_intensity_tco2_mwh = w[static_cast<size_t>(node_loc)];
      dc.total_emissions_tco2 = dc.carbon_intensity_tco2_mwh * dc.loss_mw;
    }
  }

  for (auto& er : energy_router_carbon) {
    if (er.loss_mw <= kTol) {
      er.total_emissions_tco2 = 0.0;
    }
    // Keep existing carbon_intensity from tracing; total_emissions updated above
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
  for (const auto& r : all_load_carbon) s.total_load_emissions_tco2 += r.total_emissions_tco2;
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

EmissionsSummary compute_matrix_summary(const std::vector<Source>& sources,
                                        const std::vector<UnifiedLoad>& loads,
                                        const std::vector<FlowEdge>& edges,
                                        const std::vector<double>& w) {
  EmissionsSummary s;
  for (const auto& src : sources) {
    s.total_generation_emissions_tco2 += src.ef * src.p_mw;
  }

  for (const auto& ld : loads) {
    if (ld.node_loc < 0 || ld.node_loc >= static_cast<int>(w.size())) continue;
    s.total_load_emissions_tco2 += w[static_cast<size_t>(ld.node_loc)] * ld.p_mw;
  }

  for (const auto& e : edges) {
    if (e.loss_mw <= kTol || e.from_node_loc < 0 ||
        e.from_node_loc >= static_cast<int>(w.size())) {
      continue;
    }
    s.total_loss_emissions_tco2 += w[static_cast<size_t>(e.from_node_loc)] * e.loss_mw;
  }

  s.balance_error_tco2 = s.total_generation_emissions_tco2 -
                         s.total_load_emissions_tco2 -
                         s.total_loss_emissions_tco2;
  const double denom = std::max(s.total_generation_emissions_tco2, 1e-12);
  s.balance_error_pct = std::abs(s.balance_error_tco2) / denom * 100.0;
  return s;
}

}  // namespace

CarbonAnalysisResult compute_carbon_analysis(const HybridPowerSystem& sys,
                                             const PowerFlowResult& pf_result,
                                             const CarbonAnalysisOptions& opt) {
  CarbonAnalysisResult result;
  if (!pf_result.converged) return result;

  const HybridPowerSystem projected = project_to_canonical_models(sys);
  if (projected.ac.buses.empty() && projected.dc.buses.empty()) return result;

  const HybridIDMap id = build_id_map(projected);
  const int node_count = id.n_ac + id.n_dc;

  auto ac_loads = build_unified_ac_loads(projected.ac, id);
  auto dc_loads = build_unified_dc_loads(projected.dc, id);
  const auto storage_states = build_storage_states(projected, id);
  const auto storage_load_build = build_storage_charging_loads(storage_states);
  std::vector<UnifiedLoad> all_loads = ac_loads;
  all_loads.insert(all_loads.end(), dc_loads.begin(), dc_loads.end());
  const size_t standard_load_count = all_loads.size();
  all_loads.insert(all_loads.end(),
                   storage_load_build.loads.begin(),
                   storage_load_build.loads.end());

  const auto edges = build_directed_flows(projected, pf_result, id);
  auto source_build = build_sources(projected, id);
  reconstruct_slack_output(source_build, projected, all_loads, edges);
  const auto& sources = source_build.sources;

  if (opt.verbose) {
    HACDCPF_LOG_DEBUG("[CarbonAnalysis] {} sources, {} AC buses, {} DC buses, {} directed carriers, {} AC loads, {} DC loads, {} charging storage sinks",
              sources.size(), projected.ac.buses.size(), projected.dc.buses.size(),
              edges.size(), ac_loads.size(), dc_loads.size(), storage_load_build.loads.size());
  }

  const auto tracing = proportional_tracing(sources, edges, all_loads, node_count, opt);
  const auto all_load_carbon = build_load_carbon(all_loads, tracing, sources);
  for (size_t i = 0; i < standard_load_count; ++i) {
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
        charging_load = &all_load_carbon[standard_load_count +
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

  // Solve the matrix-based carbon flow equations
  const auto mat = solve_carbon_matrix(projected, sources, edges, all_loads, opt);
  result.matrix_solved = mat.solved;
  result.matrix_residual = mat.residual;
  if (mat.solved) {
    result.bus_carbon.reserve(projected.ac.buses.size());
    for (size_t i = 0; i < projected.ac.buses.size(); ++i) {
      result.bus_carbon.push_back(
          {projected.ac.buses[i].index, mat.w[i]});
    }
    result.dc_bus_carbon.reserve(projected.dc.buses.size());
    for (size_t i = 0; i < projected.dc.buses.size(); ++i) {
      result.dc_bus_carbon.push_back(
          {projected.dc.buses[i].index,
           mat.w[projected.ac.buses.size() + i]});
    }
    result.matrix_summary = compute_matrix_summary(sources, all_loads, edges, mat.w);

    // Update all carbon results to use matrix-solved intensities
    update_emissions_from_matrix(
        result.load_carbon, result.dc_load_carbon,
        result.branch_carbon, result.dc_branch_carbon,
        result.vsc_carbon, result.dcdc_carbon,
        result.energy_router_carbon, result.storage_carbon,
        mat.w, id);
  }

  // Combine AC and DC load carbon for tracing summary
  std::vector<LoadCarbonResult> combined_load_carbon;
  combined_load_carbon.reserve(result.load_carbon.size() + result.dc_load_carbon.size());
  combined_load_carbon.insert(combined_load_carbon.end(),
                              result.load_carbon.begin(), result.load_carbon.end());
  combined_load_carbon.insert(combined_load_carbon.end(),
                              result.dc_load_carbon.begin(), result.dc_load_carbon.end());

  result.tracing_summary = compute_tracing_summary(sources,
                                                   combined_load_carbon,
                                                   result.branch_carbon,
                                                   result.dc_branch_carbon,
                                                   result.vsc_carbon,
                                                   result.dcdc_carbon,
                                                   result.energy_router_carbon);

  // Include storage charging emissions in load total
  for (const auto& sc : result.storage_carbon) {
    if (sc.p_mw < -kTol) {
      result.tracing_summary.total_load_emissions_tco2 += sc.total_emissions_tco2;
    }
  }

  // Recompute balance error after including storage
  result.tracing_summary.balance_error_tco2 =
      result.tracing_summary.total_generation_emissions_tco2 -
      result.tracing_summary.total_load_emissions_tco2 -
      result.tracing_summary.total_loss_emissions_tco2;
  const double denom = std::max(result.tracing_summary.total_generation_emissions_tco2, 1e-12);
  result.tracing_summary.balance_error_pct =
      std::abs(result.tracing_summary.balance_error_tco2) / denom * 100.0;

  result.tracing_verified =
      result.tracing_summary.balance_error_pct < opt.verification_tol * 100.0;

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

CarbonAnalysisResult compute_carbon_analysis(const HybridPowerSystem& sys,
                                             const PowerFlowOptions& pf_opt,
                                             const CarbonAnalysisOptions& ca_opt) {
  const auto pf = ::hacdcpf::solve_power_flow(sys, pf_opt);
  return compute_carbon_analysis(sys, pf, ca_opt);
}

}  // namespace hacdcpf::analysis


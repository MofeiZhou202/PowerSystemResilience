/// src/graph/power_system_graph.cpp
/// =================================
/// Build a PowerSystemGraph from a HybridPowerSystem.

#include "hacdcpf/graph/power_system_graph.hpp"

#include <cmath>

namespace hacdcpf::graph {

namespace {

// ── helper: add a node for an AC bus ─────────────────────────────────

static int add_ac_node(PowerSystemGraph& g, const ACBus& bus) {
  const int idx = static_cast<int>(g.nodes.size());
  GraphNode nd;
  nd.bus_id   = bus.index;
  nd.domain   = NodeDomain::AC;
  nd.ac_bus_type  = bus.bus_type;
  nd.dc_bus_type  = DCBusType::DC_P; // unused for AC
  nd.base_kv  = bus.base_kv;
  // An ISOLATED bus is disconnected/removed from solve, so it is treated as
  // out-of-service for all graph consumers (island detection, connectivity,
  // switch contraction) — mirrors the DC_ISOLATED handling below.
  nd.in_service = bus.in_service && (bus.bus_type != BusType::ISOLATED);
  nd.is_slack = (bus.bus_type == BusType::SLACK);
  nd.is_voltage_controlled = (bus.bus_type == BusType::PV ||
                               bus.bus_type == BusType::SLACK);
  // Loads on the bus object itself
  nd.has_load = (bus.pd_mw != 0.0 || bus.qd_mvar != 0.0);
  nd.has_shunt = (bus.gs_mw != 0.0 || bus.bs_mvar != 0.0);
  g.nodes.push_back(nd);
  // Populate both the legacy shared map and the AC-specific map.
  // The shared map is for AC-only graph consumers; the AC-specific map is
  // authoritative for hybrid lookups.
  g.bus_id_to_node_idx[bus.index] = idx;
  g.ac_bus_id_to_node_idx[bus.index] = idx;
  g.adj.emplace_back();
  return idx;
}

// ── helper: add a node for a DC bus ──────────────────────────────────

static int add_dc_node(PowerSystemGraph& g, const DCBus& bus) {
  const int idx = static_cast<int>(g.nodes.size());
  GraphNode nd;
  nd.bus_id   = bus.index;
  nd.domain   = NodeDomain::DC;
  nd.ac_bus_type  = BusType::PQ; // unused for DC
  nd.dc_bus_type  = bus.bus_type;
  nd.base_kv  = bus.base_kv;
  // An isolated DC bus is de-energized and removed from solve/topology, so it
  // is treated as out-of-service for all graph consumers (island detection,
  // connectivity, switch contraction).
  nd.in_service = bus.in_service && (bus.bus_type != DCBusType::DC_ISOLATED);
  nd.is_slack = (bus.bus_type == DCBusType::DC_V);
  nd.is_voltage_controlled = nd.is_slack;
  nd.has_load = (bus.pd_mw != 0.0) || bus.is_load;
  g.nodes.push_back(nd);
  // Write ONLY to the DC-specific map.  DC and AC buses may share the same
  // index values (e.g. both have bus 1), so we must NOT write DC nodes to
  // the shared bus_id_to_node_idx map, which is reserved for AC consumers.
  g.dc_bus_id_to_node_idx[bus.index] = idx;
  g.adj.emplace_back();
  return idx;
}

// ── helper: add a directed edge (undirected: add both directions) ─────

static void add_edge(PowerSystemGraph& g, GraphEdge e) {
  const int eid = static_cast<int>(g.edges.size());
  e.edge_id = eid;
  g.edges.push_back(e);
  if (e.from_node >= 0 && e.from_node < static_cast<int>(g.adj.size()))
    g.adj[e.from_node].emplace_back(eid, e.to_node);
  if (e.to_node >= 0 && e.to_node < static_cast<int>(g.adj.size()))
    g.adj[e.to_node].emplace_back(eid, e.from_node);
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────
// build_power_system_graph
// ─────────────────────────────────────────────────────────────────────

PowerSystemGraph build_power_system_graph(
    const HybridPowerSystem& system,
    double zero_impedance_threshold)
{
  PowerSystemGraph g;

  // ── 1. AC buses ──────────────────────────────────────────────────
  for (const auto& bus : system.ac.buses)
    add_ac_node(g, bus);

  // ── 2. DC buses ──────────────────────────────────────────────────
  for (const auto& bus : system.dc.buses)
    add_dc_node(g, bus);

  // ── 3. Mark generators on their AC buses ─────────────────────────
  for (const auto& gen : system.ac.generators) {
    if (!gen.in_service) continue;
    int ni = g.ac_node_idx(gen.bus);
    if (ni >= 0) {
      g.nodes[ni].has_generator = true;
      if (gen.is_slack) g.nodes[ni].is_slack = true;
      // PV control: if the bus type is PV, it is voltage-controlled
      // (bus_type was set when nodes were added; generators only add flags)
    }
  }
  for (const auto& eg : system.ac.external_grids) {
    if (!eg.in_service) continue;
    int ni = g.ac_node_idx(eg.bus);
    if (ni >= 0) {
      g.nodes[ni].has_generator = true;
      g.nodes[ni].is_slack      = true;
      g.nodes[ni].is_voltage_controlled = true;
    }
  }
  for (const auto& sg : system.ac.static_generators) {
    if (!sg.in_service) continue;
    int ni = g.ac_node_idx(sg.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& rg : system.ac.renewable_gens) {
    if (!rg.in_service) continue;
    int ni = g.ac_node_idx(rg.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& pv : system.ac.pv_systems) {
    if (!pv.in_service) continue;
    int ni = g.ac_node_idx(pv.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }

  // ── 4. Mark loads on their AC buses ──────────────────────────────
  for (const auto& ld : system.ac.loads) {
    if (!ld.in_service) continue;
    int ni = g.ac_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }
  for (const auto& ld : system.ac.flexible_loads) {
    if (!ld.in_service) continue;
    int ni = g.ac_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }
  for (const auto& ld : system.ac.asymmetric_loads) {
    if (!ld.in_service) continue;
    int ni = g.ac_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }
  for (const auto& motor : system.ac.motors) {
    if (!motor.in_service) continue;
    int ni = g.ac_node_idx(motor.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }
  for (const auto& station : system.ac.charging_stations) {
    if (!station.in_service) continue;
    int ni = g.ac_node_idx(station.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }

  // ── 5. Mark storage ──────────────────────────────────────────────
  for (const auto& st : system.ac.storage) {
    if (!st.in_service) continue;
    int ni = g.ac_node_idx(st.bus);
    if (ni >= 0) g.nodes[ni].has_storage = true;
  }
  for (const auto& st : system.mobile_storage) {
    if (!st.in_service) continue;
    int ni = g.ac_node_idx(st.bus);
    if (ni >= 0) g.nodes[ni].has_storage = true;
  }

  // ── 6. Mark shunts ───────────────────────────────────────────────
  for (const auto& sh : system.ac.shunts) {
    if (!sh.in_service) continue;
    int ni = g.ac_node_idx(sh.bus);
    if (ni >= 0) g.nodes[ni].has_shunt = true;
  }

  for (const auto& vpp : system.vpps) {
    if (!vpp.in_service) continue;
    int ni = g.ac_node_idx(vpp.pcc_bus);
    if (ni >= 0) {
      g.nodes[ni].has_generator = true;
      g.nodes[ni].has_load = true;
      g.nodes[ni].has_controllable = true;
    }
  }
  for (const auto& microgrid : system.microgrids) {
    if (!microgrid.in_service) continue;
    int ni = g.ac_node_idx(microgrid.pcc_bus);
    if (ni >= 0) {
      g.nodes[ni].has_generator = true;
      g.nodes[ni].has_load = true;
      g.nodes[ni].has_controllable = true;
    }
    for (int bus : microgrid.internal_buses) {
      ni = g.ac_node_idx(bus);
      if (ni >= 0) g.nodes[ni].has_controllable = true;
    }
  }

  // ── 7. Mark VSC connections ───────────────────────────────────────
  for (const auto& vsc : system.vsc_converters) {
    if (!vsc.in_service) continue;
    int ni_ac = g.ac_node_idx(vsc.bus_ac);
    int ni_dc = g.dc_node_idx(vsc.bus_dc);
    if (ni_ac >= 0) g.nodes[ni_ac].has_vsc_ac = true;
    if (ni_dc >= 0) g.nodes[ni_dc].has_vsc_dc = true;
  }

  // ── 8. Mark DCDC connections ─────────────────────────────────────
  for (const auto& dc : system.dc.dcdc_converters) {
    if (!dc.in_service) continue;
    int ni_in  = g.dc_node_idx(dc.bus_in);
    int ni_out = g.dc_node_idx(dc.bus_out);
    if (ni_in  >= 0) g.nodes[ni_in ].has_dcdc = true;
    if (ni_out >= 0) g.nodes[ni_out].has_dcdc = true;
  }

  for (const auto& lcc : system.lcc_converters) {
    if (!lcc.in_service) continue;
    int ni_ac = g.ac_node_idx(lcc.ac_bus);
    int ni_dc = g.dc_node_idx(lcc.dc_bus);
    if (ni_ac >= 0) g.nodes[ni_ac].has_lcc = true;
    if (ni_dc >= 0) g.nodes[ni_dc].has_lcc = true;
  }
  for (const auto& router : system.energy_routers) {
    if (!router.in_service) continue;
    for (const auto& port : router.ports) {
      if (!port.in_service) continue;
      int ni = port.port_type == ERPortType::DC
                   ? g.dc_node_idx(port.bus)
                   : g.ac_node_idx(port.bus);
      if (ni >= 0) {
        g.nodes[ni].has_energy_router = true;
        g.nodes[ni].has_controllable = true;
      }
    }
  }

  // ── 9. Mark DC generators / loads ────────────────────────────────
  for (const auto& sg : system.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    int ni = g.dc_node_idx(sg.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& sg : system.dc.static_generators) {
    if (!sg.in_service) continue;
    int ni = g.dc_node_idx(sg.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& pv : system.dc.pv_arrays) {
    if (!pv.in_service) continue;
    int ni = g.dc_node_idx(pv.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& ld : system.dc.loads) {
    if (!ld.in_service) continue;
    int ni = g.dc_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }
  for (const auto& st : system.dc.storage) {
    if (!st.in_service) continue;
    int ni = g.dc_node_idx(st.bus);
    if (ni >= 0) g.nodes[ni].has_storage = true;
  }
  for (const auto& st : system.dc.dc_storage) {
    if (!st.in_service) continue;
    int ni = g.dc_node_idx(st.bus);
    if (ni >= 0) g.nodes[ni].has_storage = true;
  }
  for (const auto& capacitor : system.dc.capacitors) {
    if (!capacitor.in_service) continue;
    int ni = g.dc_node_idx(capacitor.bus);
    if (ni >= 0) g.nodes[ni].has_shunt = true;
  }

  if (system.three_phase_ac.has_value()) {
    for (const auto& bus : system.three_phase_ac->buses) {
      int ni = g.ac_node_idx(bus.index);
      if (ni >= 0) g.nodes[ni].has_three_phase_model = true;
    }
  }

  // ── 10. AC branches ───────────────────────────────────────────────
  int edge_seq = 0;
  for (const auto& br : system.ac.branches) {
    int fn = g.ac_node_idx(br.from_bus);
    int tn = g.ac_node_idx(br.to_bus);
    if (fn < 0 || tn < 0) continue;
    const double z_mag = std::hypot(br.r_pu, br.x_pu);
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = br.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = br.from_bus;
    e.to_bus_id   = br.to_bus;
    e.category   = EdgeCategory::AC_Line;
    e.in_service = br.in_service;
    e.r_pu       = br.r_pu;
    e.x_pu       = br.x_pu;
    e.b_pu       = br.b_pu;
    e.tap        = br.tap;
    e.shift_deg  = br.shift_deg;
    e.rate_a_mva = br.rate_a_mva;
    e.is_zero_impedance = (z_mag < zero_impedance_threshold);
    add_edge(g, e);
  }

  // ── 11. Transformer 2W as AC branches ────────────────────────────
  for (const auto& tr : system.ac.transformers_2w) {
    // MATPOWER imports retain the exact transformer pi-model as an ACBranch;
    // Transformer2W is linked metadata in that case, not an extra electrical
    // edge. Adding both lets topology screening see a path that DC-OPF does
    // not have after the branch fails.
    if (tr.source_branch_idx > 0) continue;
    int fn = g.ac_node_idx(tr.hv_bus);
    int tn = g.ac_node_idx(tr.lv_bus);
    if (fn < 0 || tn < 0) continue;
    // Mark controllable flag
    if (tr.tap_min != tr.tap_max) {
      g.nodes[fn].has_controllable = true;
      g.nodes[tn].has_controllable = true;
    }
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = tr.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = tr.hv_bus;
    e.to_bus_id   = tr.lv_bus;
    e.category   = EdgeCategory::AC_Transformer;
    e.in_service = tr.in_service;
    e.is_zero_impedance = false;
    add_edge(g, e);
  }

  // A three-winding transformer is represented by a two-edge connectivity
  // tree rooted at the HV terminal. This preserves component connectivity
  // without inventing a pairwise electrical impedance or a false cycle.
  for (const auto& tr : system.ac.transformers_3w) {
    if (!tr.in_service) continue;
    const int hv = g.ac_node_idx(tr.hv_bus);
    const int mv = g.ac_node_idx(tr.mv_bus);
    const int lv = g.ac_node_idx(tr.lv_bus);
    if (hv < 0 || mv < 0 || lv < 0) continue;
    g.nodes[hv].has_controllable = true;
    g.nodes[mv].has_controllable = true;
    g.nodes[lv].has_controllable = true;
    for (const auto [from_node, to_node] :
         {std::pair{hv, mv}, std::pair{hv, lv}}) {
      GraphEdge edge;
      edge.edge_id = edge_seq++;
      edge.comp_index = tr.index;
      edge.from_node = from_node;
      edge.to_node = to_node;
      edge.from_bus_id = g.nodes[from_node].bus_id;
      edge.to_bus_id = g.nodes[to_node].bus_id;
      edge.category = EdgeCategory::AC_Transformer;
      edge.in_service = true;
      add_edge(g, edge);
    }
  }

  // ── 12. Switches ─────────────────────────────────────────────────
  for (const auto& sw : system.ac.switches) {
    int fn = g.ac_node_idx(sw.bus_from);
    int tn = g.ac_node_idx(sw.bus_to);
    if (fn < 0 || tn < 0) continue;
    const bool closed = sw.closed;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = sw.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = sw.bus_from;
    e.to_bus_id   = sw.bus_to;
    e.category   = EdgeCategory::Switch;
    e.in_service = sw.in_service && closed;
    e.r_pu       = 0.0;
    e.x_pu       = 0.0;
    e.is_zero_impedance  = closed; // closed switch is topologically zero-Z
    e.is_closed_switch   = closed;
    add_edge(g, e);
  }

  // ── 13. Circuit breakers ──────────────────────────────────────────
  for (const auto& cb : system.ac.circuit_breakers) {
    int fn = g.ac_node_idx(cb.bus_from);
    int tn = g.ac_node_idx(cb.bus_to);
    if (fn < 0 || tn < 0) continue;
    const bool closed = cb.closed;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = cb.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = cb.bus_from;
    e.to_bus_id   = cb.bus_to;
    e.category   = EdgeCategory::Breaker;
    e.in_service = cb.in_service && closed;
    e.r_pu       = 0.0;
    e.x_pu       = 0.0;
    e.is_zero_impedance  = closed;
    e.is_closed_switch   = closed;
    add_edge(g, e);
  }

  // ── 14. DC branches ───────────────────────────────────────────────
  for (const auto& br : system.dc.branches) {
    int fn = g.dc_node_idx(br.from_bus);
    int tn = g.dc_node_idx(br.to_bus);
    if (fn < 0 || tn < 0) continue;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = br.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = br.from_bus;
    e.to_bus_id   = br.to_bus;
    e.category   = EdgeCategory::DC_Line;
    e.in_service = br.in_service;
    e.r_pu       = br.r_pu;
    e.is_zero_impedance = (std::abs(br.r_pu) < zero_impedance_threshold);
    add_edge(g, e);
  }

  // ── 14.5. DC circuit breakers ─────────────────────────────────────
  // island_detector.cpp treats closed DC CBs as connectivity edges; the
  // graph model must match that semantic.  An open CB is inserted as an
  // out-of-service edge so the topology-analysis DFS can still see the
  // latent connection.
  for (const auto& cb : system.dc.dc_circuit_breakers) {
    int fn = g.dc_node_idx(cb.bus_from);
    int tn = g.dc_node_idx(cb.bus_to);
    if (fn < 0 || tn < 0) continue;
    const bool closed = cb.in_service && cb.closed;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = cb.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = cb.bus_from;
    e.to_bus_id   = cb.bus_to;
    e.category   = EdgeCategory::DC_Switch;
    e.in_service = closed;
    e.r_pu       = 0.0;
    e.is_zero_impedance = closed;
    e.is_closed_switch  = closed;
    add_edge(g, e);
  }

  // ── 15. VSC coupling edges (virtual, for connectivity) ────────────
  for (const auto& vsc : system.vsc_converters) {
    if (!vsc.in_service) continue;
    int fn = g.ac_node_idx(vsc.bus_ac);
    int tn = g.dc_node_idx(vsc.bus_dc);
    if (fn < 0 || tn < 0) continue;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = vsc.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = vsc.bus_ac;
    e.to_bus_id   = vsc.bus_dc;
    e.category   = EdgeCategory::VSC_Coupling;
    e.in_service = vsc.in_service;
    e.is_zero_impedance  = false;
    e.is_closed_switch   = false;
    add_edge(g, e);
  }

  for (const auto& lcc : system.lcc_converters) {
    if (!lcc.in_service) continue;
    const int fn = g.ac_node_idx(lcc.ac_bus);
    const int tn = g.dc_node_idx(lcc.dc_bus);
    if (fn < 0 || tn < 0) continue;
    GraphEdge edge;
    edge.edge_id = edge_seq++;
    edge.comp_index = lcc.index;
    edge.from_node = fn;
    edge.to_node = tn;
    edge.from_bus_id = lcc.ac_bus;
    edge.to_bus_id = lcc.dc_bus;
    edge.category = EdgeCategory::LCC_Coupling;
    edge.in_service = true;
    add_edge(g, edge);
  }

  // ── 16. DCDC coupling edges ───────────────────────────────────────
  for (const auto& dc : system.dc.dcdc_converters) {
    if (!dc.in_service) continue;
    int fn = g.dc_node_idx(dc.bus_in);
    int tn = g.dc_node_idx(dc.bus_out);
    if (fn < 0 || tn < 0) continue;
    GraphEdge e;
    e.edge_id    = edge_seq++;
    e.comp_index = dc.index;
    e.from_node  = fn;
    e.to_node    = tn;
    e.from_bus_id = dc.bus_in;
    e.to_bus_id   = dc.bus_out;
    e.category   = EdgeCategory::DCDC_Coupling;
    e.in_service = dc.in_service;
    e.is_zero_impedance = false;
    add_edge(g, e);
  }

  // A multi-port router contributes a connectivity tree from its first valid
  // port to every remaining valid port. The edges are virtual and carry no
  // impedance or direction assumptions.
  for (const auto& router : system.energy_routers) {
    if (!router.in_service) continue;
    std::vector<std::pair<int, const EnergyRouterPort*>> terminals;
    for (const auto& port : router.ports) {
      if (!port.in_service) continue;
      const int node = port.port_type == ERPortType::DC
                           ? g.dc_node_idx(port.bus)
                           : g.ac_node_idx(port.bus);
      if (node >= 0) terminals.emplace_back(node, &port);
    }
    for (std::size_t i = 1; i < terminals.size(); ++i) {
      GraphEdge edge;
      edge.edge_id = edge_seq++;
      edge.comp_index = router.index;
      edge.from_node = terminals.front().first;
      edge.to_node = terminals[i].first;
      edge.from_bus_id = terminals.front().second->bus;
      edge.to_bus_id = terminals[i].second->bus;
      edge.category = EdgeCategory::EnergyRouter_Coupling;
      edge.in_service = true;
      add_edge(g, edge);
    }
  }

  return g;
}

PowerSystemGraph build_three_phase_power_system_graph(
    const ThreePhaseACSystem& system,
    double zero_impedance_threshold) {
  PowerSystemGraph graph;
  for (const auto& bus : system.buses) {
    ACBus balanced;
    balanced.index = bus.index;
    balanced.bus_type = bus.bus_type;
    balanced.base_kv = bus.base_kv;
    balanced.in_service = bus.in_service;
    balanced.pd_mw = bus.pd_a_mw + bus.pd_b_mw + bus.pd_c_mw;
    balanced.qd_mvar = bus.qd_a_mvar + bus.qd_b_mvar + bus.qd_c_mvar;
    balanced.gs_mw = bus.gs_a_mw + bus.gs_b_mw + bus.gs_c_mw;
    balanced.bs_mvar = bus.bs_a_mvar + bus.bs_b_mvar + bus.bs_c_mvar;
    const int node = add_ac_node(graph, balanced);
    graph.nodes[node].has_three_phase_model = true;
  }
  for (const auto& load : system.loads) {
    if (!load.in_service) continue;
    const int node = graph.ac_node_idx(load.bus);
    if (node >= 0) graph.nodes[node].has_load = true;
  }
  for (const auto& generator : system.generators) {
    if (!generator.in_service) continue;
    const int node = graph.ac_node_idx(generator.bus);
    if (node >= 0) {
      graph.nodes[node].has_generator = true;
      graph.nodes[node].is_slack |= generator.is_slack;
    }
  }
  for (const auto& source : system.external_grids) {
    if (!source.in_service) continue;
    const int node = graph.ac_node_idx(source.bus);
    if (node >= 0) {
      graph.nodes[node].has_generator = true;
      graph.nodes[node].is_slack = true;
      graph.nodes[node].is_voltage_controlled = true;
    }
  }
  for (const auto& line : system.lines) {
    const int from = graph.ac_node_idx(line.from_bus);
    const int to = graph.ac_node_idx(line.to_bus);
    if (from < 0 || to < 0) continue;
    GraphEdge edge;
    edge.comp_index = line.index;
    edge.from_node = from;
    edge.to_node = to;
    edge.from_bus_id = line.from_bus;
    edge.to_bus_id = line.to_bus;
    edge.category = EdgeCategory::ThreePhase_Line;
    edge.in_service = line.in_service;
    edge.r_pu = line.r1_pu;
    edge.x_pu = line.x1_pu;
    edge.b_pu = line.b1_pu;
    edge.rate_a_mva = line.rate_a_mva;
    edge.is_zero_impedance =
        std::hypot(line.r1_pu, line.x1_pu) < zero_impedance_threshold;
    add_edge(graph, edge);
  }
  for (const auto& transformer : system.transformers) {
    const int from = graph.ac_node_idx(transformer.hv_bus);
    const int to = graph.ac_node_idx(transformer.lv_bus);
    if (from < 0 || to < 0) continue;
    GraphEdge edge;
    edge.comp_index = transformer.index;
    edge.from_node = from;
    edge.to_node = to;
    edge.from_bus_id = transformer.hv_bus;
    edge.to_bus_id = transformer.lv_bus;
    edge.category = EdgeCategory::ThreePhase_Transformer;
    edge.in_service = transformer.in_service;
    add_edge(graph, edge);
    graph.nodes[from].has_controllable = true;
    graph.nodes[to].has_controllable = true;
  }
  return graph;
}

}  // namespace hacdcpf::graph

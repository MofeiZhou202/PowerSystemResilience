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
  // Clamp node indices
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
    int ni = g.ac_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
  }

  // ── 5. Mark storage ──────────────────────────────────────────────
  for (const auto& st : system.ac.storage) {
    int ni = g.ac_node_idx(st.bus);
    if (ni >= 0) g.nodes[ni].has_storage = true;
  }

  // ── 6. Mark shunts ───────────────────────────────────────────────
  for (const auto& sh : system.ac.shunts) {
    int ni = g.ac_node_idx(sh.bus);
    if (ni >= 0) g.nodes[ni].has_shunt = true;
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

  // ── 9. Mark DC generators / loads ────────────────────────────────
  for (const auto& sg : system.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    int ni = g.dc_node_idx(sg.bus);
    if (ni >= 0) g.nodes[ni].has_generator = true;
  }
  for (const auto& ld : system.dc.loads) {
    if (!ld.in_service) continue;
    int ni = g.dc_node_idx(ld.bus);
    if (ni >= 0) g.nodes[ni].has_load = true;
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
    e.is_zero_impedance = (br.r_pu < zero_impedance_threshold);
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

  return g;
}

}  // namespace hacdcpf::graph

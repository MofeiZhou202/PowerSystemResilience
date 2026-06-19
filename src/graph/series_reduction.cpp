/// src/graph/series_reduction.cpp
/// =================================
/// Series reduction (passive degree-2 node elimination) and
/// pendant branch folding.

#include "hacdcpf/graph/series_reduction.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::graph {

// ─────────────────────────────────────────────────────────────────────
// apply_series_reduction
// ─────────────────────────────────────────────────────────────────────

SeriesReductionResult apply_series_reduction(
    const PowerSystemGraph&      graph,
    const HybridPowerSystem&     system,
    const ReductionPlan&         plan,
    const GraphReductionOptions& /*options*/)
{
  SeriesReductionResult result;
  result.reduced_system = system;

  // Copy the graph as a starting point
  result.reduced_graph = graph;

  // Build initial identity mapping (both legacy and domain-qualified maps)
  for (const auto& nd : graph.nodes) {
    result.mapping.original_to_reduced_bus[nd.bus_id] = nd.bus_id;
    result.mapping.reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    if (nd.domain == NodeDomain::DC) {
      result.mapping.dc_original_to_reduced_bus[nd.bus_id] = nd.bus_id;
      result.mapping.dc_reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    } else {
      result.mapping.ac_original_to_reduced_bus[nd.bus_id] = nd.bus_id;
      result.mapping.ac_reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    }
  }
  for (const auto& e : graph.edges) {
    result.mapping.original_to_reduced_branch[e.edge_id] = e.edge_id;
    result.mapping.reduced_to_original_branches[e.edge_id] = {e.edge_id};
  }

  // Track next IDs for new branches.  The new equivalent branch must not
  // collide with any existing component index (the graph edge_id space and the
  // model .index space are independent), so start beyond both.
  int next_branch_id = static_cast<int>(graph.edges.size());
  for (const auto& br : system.ac.branches)
    next_branch_id = std::max(next_branch_id, br.index + 1);
  for (const auto& br : system.dc.branches)
    next_branch_id = std::max(next_branch_id, br.index + 1);

  // Process series reduction actions
  for (const auto& action : plan.actions) {
    if (action.type != ReductionActionType::SeriesReduction) continue;
    if (action.eliminated_buses.size() != 1) continue;
    if (action.eliminated_branches.size() != 2) continue;

    int elim_bus_id = action.eliminated_buses[0];
    int eid_ij = action.eliminated_branches[0];
    int eid_jk = action.eliminated_branches[1];

    // Find edges in the current reduced graph
    int rni_j = (action.bus_domain == NodeDomain::AC)
                 ? result.reduced_graph.ac_node_idx(elim_bus_id)
                 : result.reduced_graph.dc_node_idx(elim_bus_id);
    if (rni_j < 0) continue;

    // Identify the two edges from the eliminated node
    const GraphEdge* e_ij_ptr = nullptr;
    const GraphEdge* e_jk_ptr = nullptr;
    for (const auto& e : result.reduced_graph.edges) {
      if (!e.in_service) continue;
      if (e.edge_id == eid_ij) e_ij_ptr = &e;
      if (e.edge_id == eid_jk) e_jk_ptr = &e;
    }
    if (!e_ij_ptr || !e_jk_ptr) continue;

    const GraphEdge& e_ij = *e_ij_ptr;
    const GraphEdge& e_jk = *e_jk_ptr;

    // Capture the source component indices by value NOW.  The push_back into
    // reduced_graph.edges below may reallocate the vector and invalidate the
    // e_ij / e_jk references, so they must not be dereferenced afterwards.
    const int comp_ij = e_ij.comp_index;
    const int comp_jk = e_jk.comp_index;

    // Determine node i and node k
    int rni_i = (e_ij.from_node == rni_j) ? e_ij.to_node : e_ij.from_node;
    int rni_k = (e_jk.from_node == rni_j) ? e_jk.to_node : e_jk.from_node;

    if (rni_i < 0 || rni_k < 0) continue;

    int bus_i = result.reduced_graph.nodes[rni_i].bus_id;
    int bus_k = result.reduced_graph.nodes[rni_k].bus_id;

    // Equivalent impedance: Z_ik = Z_ij + Z_jk
    double r_eq = e_ij.r_pu + e_jk.r_pu;
    double x_eq = e_ij.x_pu + e_jk.x_pu;
    double b_eq = e_ij.b_pu + e_jk.b_pu; // approximate: sum charging

    // Create new merged edge
    GraphEdge new_edge;
    new_edge.edge_id      = next_branch_id++;
    new_edge.comp_index   = new_edge.edge_id;
    new_edge.from_node    = rni_i;
    new_edge.to_node      = rni_k;
    new_edge.from_bus_id  = bus_i;
    new_edge.to_bus_id    = bus_k;
    new_edge.category     = e_ij.category; // inherit from first segment
    new_edge.in_service   = true;
    new_edge.r_pu         = r_eq;
    new_edge.x_pu         = x_eq;
    new_edge.b_pu         = b_eq;
    new_edge.tap          = 1.0;
    new_edge.shift_deg    = 0.0;
    new_edge.rate_a_mva   = 0.0;
    new_edge.is_zero_impedance  = false;
    new_edge.is_closed_switch   = false;

    // Record reduction
    SeriesReductionRecord rec;
    rec.eliminated_bus_id  = elim_bus_id;
    rec.from_bus_id        = bus_i;
    rec.to_bus_id          = bus_k;
    rec.new_branch_id      = new_edge.edge_id;
    // Store the source-model component indices (NOT positional edge ids) so
    // result recovery can look the i–j segment impedance up in the original
    // system's branch table by .index.  original_branch_ids[0] is the i–j
    // segment (matching rec.from_bus_id = bus_i).
    rec.original_branch_ids = {comp_ij, comp_jk};
    rec.r_eq = r_eq;
    rec.x_eq = x_eq;
    rec.b_eq = b_eq;
    rec.domain = action.bus_domain;
    result.mapping.series_records.push_back(rec);

    // Update mapping for original branches
    for (int orig_eid : {eid_ij, eid_jk}) {
      result.mapping.original_to_reduced_branch[orig_eid] = new_edge.edge_id;
    }
    result.mapping.original_to_reduced_bus[elim_bus_id] = bus_i; // map to i
    // Also update the domain-qualified map
    if (action.bus_domain == NodeDomain::DC) {
      result.mapping.dc_original_to_reduced_bus[elim_bus_id] = bus_i;
    } else {
      result.mapping.ac_original_to_reduced_bus[elim_bus_id] = bus_i;
    }

    // Update reduced_to_original for new branch
    std::vector<int> orig_branches;
    if (result.mapping.reduced_to_original_branches.count(eid_ij))
      for (int b : result.mapping.reduced_to_original_branches.at(eid_ij))
        orig_branches.push_back(b);
    if (result.mapping.reduced_to_original_branches.count(eid_jk))
      for (int b : result.mapping.reduced_to_original_branches.at(eid_jk))
        orig_branches.push_back(b);
    result.mapping.reduced_to_original_branches[new_edge.edge_id] = orig_branches;

    // Disable old edges and eliminated node in the graph
    for (auto& e : result.reduced_graph.edges) {
      if (e.edge_id == eid_ij || e.edge_id == eid_jk)
        e.in_service = false;
    }
    result.reduced_graph.nodes[rni_j].in_service = false;

    // Add new edge to graph
    int new_eid_idx = static_cast<int>(result.reduced_graph.edges.size());
    result.reduced_graph.edges.push_back(new_edge);
    result.reduced_graph.adj[rni_i].emplace_back(new_eid_idx, rni_k);
    result.reduced_graph.adj[rni_k].emplace_back(new_eid_idx, rni_i);

    // Update branch in the system — domain-correct replacement
    if (new_edge.category == EdgeCategory::DC_Line) {
      // DC series segment: write merged equivalent to dc.branches
      DCBranch new_br;
      new_br.index      = new_edge.edge_id;
      new_br.from_bus   = bus_i;
      new_br.to_bus     = bus_k;
      new_br.r_pu       = r_eq;
      new_br.in_service = true;
      result.reduced_system.dc.branches.push_back(new_br);
      for (auto& br : result.reduced_system.dc.branches) {
        if (br.index == comp_ij || br.index == comp_jk)
          br.in_service = false;
      }
      for (auto& bus : result.reduced_system.dc.buses) {
        if (bus.index == elim_bus_id) { bus.in_service = false; break; }
      }
    } else {
      // AC segment (ACBranch, transformer, AC switch …)
      ACBranch new_br;
      new_br.index      = new_edge.edge_id;
      new_br.from_bus   = bus_i;
      new_br.to_bus     = bus_k;
      new_br.r_pu       = r_eq;
      new_br.x_pu       = x_eq;
      new_br.b_pu       = b_eq;
      new_br.tap        = 1.0;
      new_br.in_service = true;
      result.reduced_system.ac.branches.push_back(new_br);
      for (auto& br : result.reduced_system.ac.branches) {
        if (br.index == comp_ij || br.index == comp_jk)
          br.in_service = false;
      }
      for (auto& bus : result.reduced_system.ac.buses) {
        if (bus.index == elim_bus_id) { bus.in_service = false; break; }
      }
    }
  }

  return result;
}

// ─────────────────────────────────────────────────────────────────────
// apply_pendant_reduction
// ─────────────────────────────────────────────────────────────────────

PendantReductionResult apply_pendant_reduction(
    const PowerSystemGraph&      graph,
    const HybridPowerSystem&     system,
    const ReductionPlan&         plan,
    const GraphReductionOptions& /*options*/)
{
  PendantReductionResult result;
  result.reduced_system = system;
  result.reduced_graph  = graph;

  // Identity mapping init (both legacy and domain-qualified maps)
  for (const auto& nd : graph.nodes) {
    result.mapping.original_to_reduced_bus[nd.bus_id] = nd.bus_id;
    result.mapping.reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    if (nd.domain == NodeDomain::DC) {
      result.mapping.dc_original_to_reduced_bus[nd.bus_id] = nd.bus_id;
      result.mapping.dc_reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    } else {
      result.mapping.ac_original_to_reduced_bus[nd.bus_id] = nd.bus_id;
      result.mapping.ac_reduced_to_original_buses[nd.bus_id] = {nd.bus_id};
    }
  }

  const double V_parent_sq = 1.0; // |V_i|² ≈ 1.0 pu² (flat-start approximation)

  for (const auto& action : plan.actions) {
    if (action.type != ReductionActionType::PendantFold) continue;
    if (action.eliminated_buses.size() != 1) continue;

    int elim_bus_id    = action.eliminated_buses[0];
    int parent_bus_id  = action.retained_buses.empty() ? -1 : action.retained_buses[0];
    int branch_eid     = action.eliminated_branches.empty() ? -1 : action.eliminated_branches[0];
    if (parent_bus_id < 0 || branch_eid < 0) continue;

    // Get the connecting branch
    const GraphEdge* edge_ptr = nullptr;
    for (const auto& e : result.reduced_graph.edges) {
      if (e.edge_id == branch_eid) { edge_ptr = &e; break; }
    }
    if (!edge_ptr) continue;

    double r_ij = edge_ptr->r_pu;
    double x_ij = edge_ptr->x_pu;
    const int branch_comp = edge_ptr->comp_index;

    // Collect load from the eliminated bus (domain-aware)
    double p_j = 0.0, q_j = 0.0;
    if (action.bus_domain == NodeDomain::DC) {
      for (const auto& bus : system.dc.buses) {
        if (bus.index == elim_bus_id) { p_j += bus.pd_mw; break; }
      }
      for (const auto& ld : system.dc.loads) {
        if (ld.bus == elim_bus_id && ld.in_service) p_j += ld.p_mw;
      }
    } else {
      for (const auto& bus : system.ac.buses) {
        if (bus.index == elim_bus_id) { p_j += bus.pd_mw;  q_j += bus.qd_mvar; break; }
      }
      for (const auto& ld : system.ac.loads) {
        if (ld.bus == elim_bus_id && ld.in_service) {
          p_j += ld.p_mw; q_j += ld.q_mvar;
        }
      }
    }

    // Convert MW/Mvar to pu
    double base_mva = system.base_mva > 0.0 ? system.base_mva : 100.0;
    double p_pu = p_j / base_mva;
    double q_pu = q_j / base_mva;

    // Approximate branch losses (DC networks: x_ij = 0, q_loss = 0)
    double s2 = p_pu * p_pu + q_pu * q_pu;
    double p_loss = r_ij * s2 / V_parent_sq;
    double q_loss = (action.bus_domain == NodeDomain::DC) ? 0.0 : x_ij * s2 / V_parent_sq;

    double p_absorbed = p_pu + p_loss; // in pu
    double q_absorbed = q_pu + q_loss;

    // Record
    PendantReductionRecord rec;
    rec.eliminated_bus_id  = elim_bus_id;
    rec.parent_bus_id      = parent_bus_id;
    rec.branch_id          = branch_eid;
    rec.p_load_absorbed    = p_absorbed * base_mva; // store as MW
    rec.q_load_absorbed    = q_absorbed * base_mva;
    rec.domain             = action.bus_domain;
    result.mapping.pendant_records.push_back(rec);
    result.mapping.original_to_reduced_bus[elim_bus_id] = parent_bus_id;
    // Also update domain-qualified map
    if (action.bus_domain == NodeDomain::DC) {
      result.mapping.dc_original_to_reduced_bus[elim_bus_id] = parent_bus_id;
    } else {
      result.mapping.ac_original_to_reduced_bus[elim_bus_id] = parent_bus_id;
    }

    // Add load to parent bus in the reduced system (domain-aware)
    if (action.bus_domain == NodeDomain::DC) {
      for (auto& bus : result.reduced_system.dc.buses) {
        if (bus.index == parent_bus_id) { bus.pd_mw += rec.p_load_absorbed; break; }
      }
      // Mark eliminated DC bus and pendant DC branch out-of-service
      for (auto& bus : result.reduced_system.dc.buses)
        if (bus.index == elim_bus_id) { bus.in_service = false; break; }
      for (auto& br : result.reduced_system.dc.branches)
        if (br.index == branch_comp) { br.in_service = false; break; }
    } else {
      for (auto& bus : result.reduced_system.ac.buses) {
        if (bus.index == parent_bus_id) {
          bus.pd_mw   += rec.p_load_absorbed;
          bus.qd_mvar += rec.q_load_absorbed;
          break;
        }
      }
      // Mark eliminated AC bus and pendant AC branch out-of-service
      for (auto& bus : result.reduced_system.ac.buses)
        if (bus.index == elim_bus_id) { bus.in_service = false; break; }
      for (auto& e : result.reduced_system.ac.branches)
        if (e.index == branch_comp) { e.in_service = false; break; }
    }

    int rni_j = (action.bus_domain == NodeDomain::AC)
                 ? result.reduced_graph.ac_node_idx(elim_bus_id)
                 : result.reduced_graph.dc_node_idx(elim_bus_id);
    if (rni_j >= 0) result.reduced_graph.nodes[rni_j].in_service = false;
    for (auto& e : result.reduced_graph.edges)
      if (e.edge_id == branch_eid) { e.in_service = false; break; }
  }

  return result;
}

}  // namespace hacdcpf::graph

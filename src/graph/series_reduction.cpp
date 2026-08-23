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

  result.mapping = make_identity_reduction_mapping(graph);

  // Track next IDs for new branches.  The new equivalent branch must not
  // collide with any existing component index (the graph edge_id space and the
  // model .index space are independent), so start beyond both.
  int next_branch_component_index = 0;
  for (const auto& br : system.ac.branches)
    next_branch_component_index =
        std::max(next_branch_component_index, br.index + 1);
  for (const auto& br : system.dc.branches)
    next_branch_component_index =
        std::max(next_branch_component_index, br.index + 1);

  // Process series reduction actions
  for (const auto& action : plan.actions) {
    if (action.type != ReductionActionType::SeriesReduction) continue;
    if (action.eliminated_buses.size() != 1) continue;
    if (action.eliminated_edge_positions.size() != 2) continue;

    const BusRef eliminated_bus = action.eliminated_buses[0];
    const int elim_bus_id = eliminated_bus.bus_id;
    const int edge_pos_ij = action.eliminated_edge_positions[0];
    const int edge_pos_jk = action.eliminated_edge_positions[1];

    // Find edges in the current reduced graph
    int rni_j = (eliminated_bus.domain == NodeDomain::AC)
                 ? result.reduced_graph.ac_node_idx(elim_bus_id)
                 : result.reduced_graph.dc_node_idx(elim_bus_id);
    if (rni_j < 0) continue;

    if (edge_pos_ij < 0 || edge_pos_jk < 0 ||
        edge_pos_ij >= result.reduced_graph.edge_count() ||
        edge_pos_jk >= result.reduced_graph.edge_count()) {
      continue;
    }
    const GraphEdge& e_ij = result.reduced_graph.edges[edge_pos_ij];
    const GraphEdge& e_jk = result.reduced_graph.edges[edge_pos_jk];
    if (!e_ij.in_service || !e_jk.in_service) continue;

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
    new_edge.edge_id      = result.reduced_graph.edge_count();
    new_edge.comp_index   = next_branch_component_index++;
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
    rec.new_branch_id      = new_edge.comp_index;
    // Store the source-model component indices (NOT positional edge ids) so
    // result recovery can look the i–j segment impedance up in the original
    // system's branch table by .index.  original_branch_ids[0] is the i–j
    // segment (matching rec.from_bus_id = bus_i).
    rec.original_branch_ids = {comp_ij, comp_jk};
    rec.r_eq = r_eq;
    rec.x_eq = x_eq;
    rec.b_eq = b_eq;
    rec.domain = eliminated_bus.domain;
    result.mapping.series_records.push_back(rec);

    const BranchRef reduced_branch{eliminated_bus.domain,
                                   new_edge.comp_index};
    set_branch_reduction(result.mapping,
                         {eliminated_bus.domain, comp_ij}, reduced_branch);
    set_branch_reduction(result.mapping,
                         {eliminated_bus.domain, comp_jk}, reduced_branch);
    set_bus_reduction(result.mapping, eliminated_bus,
                      {eliminated_bus.domain, bus_i});

    // Disable old edges and eliminated node in the graph.
    result.reduced_graph.edges[edge_pos_ij].in_service = false;
    result.reduced_graph.edges[edge_pos_jk].in_service = false;
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
      new_br.index      = new_edge.comp_index;
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
      new_br.index      = new_edge.comp_index;
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

  result.mapping = make_identity_reduction_mapping(graph);

  const double V_parent_sq = 1.0; // |V_i|² ≈ 1.0 pu² (flat-start approximation)

  for (const auto& action : plan.actions) {
    if (action.type != ReductionActionType::PendantFold) continue;
    if (action.eliminated_buses.size() != 1) continue;

    const BusRef eliminated_bus = action.eliminated_buses[0];
    if (action.retained_buses.empty() ||
        action.eliminated_edge_positions.empty()) {
      continue;
    }
    const BusRef parent_bus = action.retained_buses[0];
    const int elim_bus_id = eliminated_bus.bus_id;
    const int parent_bus_id = parent_bus.bus_id;
    const int branch_edge_pos = action.eliminated_edge_positions[0];
    if (parent_bus.domain != eliminated_bus.domain || parent_bus_id < 0 ||
        branch_edge_pos < 0 ||
        branch_edge_pos >= result.reduced_graph.edge_count()) {
      continue;
    }

    // Get the connecting branch
    const GraphEdge* edge_ptr =
        &result.reduced_graph.edges[branch_edge_pos];
    if (!edge_ptr->in_service ||
        (edge_ptr->category != EdgeCategory::AC_Line &&
         edge_ptr->category != EdgeCategory::DC_Line)) {
      continue;
    }

    double r_ij = edge_ptr->r_pu;
    double x_ij = edge_ptr->x_pu;
    const int branch_comp = edge_ptr->comp_index;

    // Collect load from the eliminated bus (domain-aware)
    double p_j = 0.0, q_j = 0.0;
    if (eliminated_bus.domain == NodeDomain::DC) {
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
    double q_loss = (eliminated_bus.domain == NodeDomain::DC)
                        ? 0.0
                        : x_ij * s2 / V_parent_sq;

    double p_absorbed = p_pu + p_loss; // in pu
    double q_absorbed = q_pu + q_loss;

    // Record
    PendantReductionRecord rec;
    rec.eliminated_bus_id  = elim_bus_id;
    rec.parent_bus_id      = parent_bus_id;
    rec.source_branch_index = branch_comp;
    rec.p_load_absorbed    = p_absorbed * base_mva; // store as MW
    rec.q_load_absorbed    = q_absorbed * base_mva;
    rec.domain             = eliminated_bus.domain;
    result.mapping.pendant_records.push_back(rec);
    set_bus_reduction(result.mapping, eliminated_bus, parent_bus);
    set_branch_reduction(result.mapping,
                         {eliminated_bus.domain, branch_comp},
                         {eliminated_bus.domain, -1});

    // Add load to parent bus in the reduced system (domain-aware)
    if (eliminated_bus.domain == NodeDomain::DC) {
      for (auto& bus : result.reduced_system.dc.buses) {
        if (bus.index == parent_bus_id) { bus.pd_mw += rec.p_load_absorbed; break; }
      }
      // Mark eliminated DC bus and pendant DC branch out-of-service
      for (auto& bus : result.reduced_system.dc.buses)
        if (bus.index == elim_bus_id) { bus.in_service = false; break; }
      for (auto& br : result.reduced_system.dc.branches)
        if (br.index == branch_comp) { br.in_service = false; break; }
      for (auto& load : result.reduced_system.dc.loads) {
        if (load.bus == elim_bus_id) load.in_service = false;
      }
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
      for (auto& load : result.reduced_system.ac.loads) {
        if (load.bus == elim_bus_id) load.in_service = false;
      }
    }

    int rni_j = (eliminated_bus.domain == NodeDomain::AC)
                 ? result.reduced_graph.ac_node_idx(elim_bus_id)
                 : result.reduced_graph.dc_node_idx(elim_bus_id);
    if (rni_j >= 0) result.reduced_graph.nodes[rni_j].in_service = false;
    result.reduced_graph.edges[branch_edge_pos].in_service = false;
  }

  return result;
}

}  // namespace hacdcpf::graph

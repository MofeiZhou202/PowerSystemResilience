/// src/graph/reduction_plan.cpp
/// =============================
/// Candidate classification and reduction plan generation.

#include "hacdcpf/graph/reduction_plan.hpp"

#include <unordered_set>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::graph {

// ─────────────────────────────────────────────────────────────────────
// classify_reduction_candidates
// ─────────────────────────────────────────────────────────────────────

ReductionCandidates classify_reduction_candidates(
    const PowerSystemGraph&          graph,
    const hacdcpf::HybridPowerSystem& /*system*/,
    const GraphReductionOptions&     options)
{
  ReductionCandidates result;

  for (int ni = 0; ni < graph.node_count(); ++ni) {
    const GraphNode& nd = graph.nodes[ni];
    if (!nd.in_service) continue;

    BusCandidate cand;
    cand.bus_id = nd.bus_id;
    cand.domain = nd.domain;

    // ── Must-retain rules ───────────────────────────────────────────
    bool must_retain = false;
    std::string retain_reason;

    if (nd.is_slack) {
      must_retain   = true;
      retain_reason = "slack / external grid bus";
    } else if (nd.is_voltage_controlled && options.preserve_all_generator_buses) {
      must_retain   = true;
      retain_reason = "PV / voltage-controlled bus";
    } else if (nd.has_generator && options.preserve_all_generator_buses) {
      must_retain   = true;
      retain_reason = "bus has generator";
    } else if (nd.has_storage) {
      must_retain   = true;
      retain_reason = "bus has storage device";
    } else if (nd.has_vsc_ac || nd.has_vsc_dc) {
      must_retain   = true;
      retain_reason = "VSC converter terminal";
    } else if (nd.has_dcdc) {
      must_retain   = true;
      retain_reason = "DCDC converter terminal";
    } else if (nd.has_controllable) {
      must_retain   = true;
      retain_reason = "bus has controllable device (tap/shunt)";
    } else if (nd.is_monitored && options.preserve_all_monitored_buses) {
      must_retain   = true;
      retain_reason = "monitored output bus";
    } else if (nd.has_load && options.preserve_all_load_buses) {
      must_retain   = true;
      retain_reason = "bus has load (preserve_all_load_buses=true)";
    }

    if (must_retain) {
      cand.type   = CandidateType::MustRetain;
      cand.reason = retain_reason;
      result.candidates.push_back(cand);
      continue;
    }

    // ── Count in-service degree ──────────────────────────────────────
    int degree = 0;
    for (auto [eid, v] : graph.adj[ni]) {
      if (graph.edges[eid].in_service && graph.nodes[v].in_service) ++degree;
    }

    // ── Classify passive nodes ───────────────────────────────────────
    // Node is passive if: no load, no generator, no shunt, no storage,
    //                     no controllable device, no VSC/DCDC terminal
    const bool passive =
        !nd.has_load && !nd.has_generator && !nd.has_shunt &&
        !nd.has_storage && !nd.has_vsc_ac && !nd.has_vsc_dc &&
        !nd.has_dcdc && !nd.has_controllable;

    if (passive && degree == 0) {
      // Isolated passive — not useful to eliminate, but harmless
      cand.type   = CandidateType::MustRetain;
      cand.reason = "isolated passive node";
    } else if (passive && degree == 1) {
      // Leaf node, no load, no gen — pure dangling connector (rare)
      cand.type   = CandidateType::ZeroInjectionPassiveInterior;
      cand.reason = "passive leaf node (no injection)";
    } else if (passive && degree == 2) {
      cand.type   = CandidateType::ZeroInjectionDegree2;
      cand.reason = "passive degree-2 node — series reduction candidate";
    } else if (passive && degree > 2) {
      cand.type   = CandidateType::KronPassiveNode;
      cand.reason = "passive interior node — Kron reduction candidate";
    } else if (!passive && degree == 1) {
      // Pendant with load (but load buses are retained by default)
      // This branch only reached if preserve_all_load_buses = false
      cand.type   = CandidateType::PendantLoad;
      cand.reason = "pendant load node — approximate folding candidate";
    } else {
      // Has injection but not a must-retain: edge case
      cand.type   = CandidateType::MustRetain;
      cand.reason = "has injection, retain by default";
    }

    result.candidates.push_back(cand);
  }

  return result;
}

// ─────────────────────────────────────────────────────────────────────
// make_reduction_plan
// ─────────────────────────────────────────────────────────────────────

ReductionPlan make_reduction_plan(
    const PowerSystemGraph&      graph,
    const ReductionCandidates&   candidates,
    const GraphReductionOptions& options)
{
  ReductionPlan plan;

  // Build a fast lookup: bus_id → candidate
  std::unordered_map<int, const BusCandidate*> cmap;
  for (const auto& c : candidates.candidates)
    cmap[c.bus_id] = &c;

  // For series reduction we need to know which branches connect two nodes
  // Build: node_idx → list of (eid, neighbour_node_idx)
  // (already in graph.adj)

  std::unordered_set<int> eliminated_buses;
  std::unordered_set<int> eliminated_edges;

  // ── Series reductions (degree-2 passive nodes) ───────────────────
  if (options.enable_series_reduction) {
    for (const auto& cand : candidates.candidates) {
      if (cand.type != CandidateType::ZeroInjectionDegree2) continue;
      int ni = (cand.domain == NodeDomain::AC)
               ? graph.ac_node_idx(cand.bus_id)
               : graph.dc_node_idx(cand.bus_id);
      if (ni < 0) continue;

      // Collect the two in-service neighbours
      std::vector<std::pair<int,int>> nbrs; // (eid, node_idx)
      for (auto [eid, v] : graph.adj[ni]) {
        if (!graph.edges[eid].in_service) continue;
        if (!graph.nodes[v].in_service) continue;
        nbrs.emplace_back(eid, v);
      }
      if (nbrs.size() != 2) continue; // guard: must have exactly 2

      // Check branch flow limit preservation
      if (options.preserve_branch_flow_limited_edges) {
        bool any_limited = false;
        for (auto [eid, v] : nbrs) {
          if (graph.edges[eid].rate_a_mva > 0) { any_limited = true; break; }
        }
        if (any_limited) {
          // Cannot eliminate — would lose thermal constraints
          continue;
        }
      }

      // Only merge plain line segments.  A transformer carries tap / phase
      // shift (and lives in a separate component list), and a switch / breaker
      // carries open-close status; a pure series Z-equivalent cannot represent
      // any of these.  A degree-2 node incident to a non-line edge is therefore
      // retained rather than series-reduced.
      {
        bool both_lines = true;
        for (auto [eid, v] : nbrs) {
          const EdgeCategory cat = graph.edges[eid].category;
          if (cat != EdgeCategory::AC_Line && cat != EdgeCategory::DC_Line) {
            both_lines = false;
            break;
          }
        }
        if (!both_lines) continue;
      }

      // Check that neighbours are not in eliminated set (avoid chain issues)
      if (eliminated_buses.count(graph.nodes[nbrs[0].second].bus_id) ||
          eliminated_buses.count(graph.nodes[nbrs[1].second].bus_id))
        continue;

      // Build action
      ReductionAction act;
      act.type = ReductionActionType::SeriesReduction;
      act.eliminated_buses   = {cand.bus_id};
      act.eliminated_branches = {graph.edges[nbrs[0].first].edge_id,
                                  graph.edges[nbrs[1].first].edge_id};
      act.retained_buses     = {graph.nodes[nbrs[0].second].bus_id,
                                  graph.nodes[nbrs[1].second].bus_id};
      act.bus_domain = cand.domain;
      act.reason = cand.reason;
      plan.actions.push_back(act);
      eliminated_buses.insert(cand.bus_id);
      eliminated_edges.insert(nbrs[0].first);
      eliminated_edges.insert(nbrs[1].first);
      ++plan.n_buses_eliminated;
      plan.n_branches_eliminated += 2; // two original branches → one new
    }
  }

  // ── Pendant reductions ────────────────────────────────────────────
  if (options.enable_pendant_reduction) {
    for (const auto& cand : candidates.candidates) {
      if (cand.type != CandidateType::PendantLoad) continue;
      int ni = (cand.domain == NodeDomain::AC)
               ? graph.ac_node_idx(cand.bus_id)
               : graph.dc_node_idx(cand.bus_id);
      if (ni < 0) continue;
      if (eliminated_buses.count(cand.bus_id)) continue;

      std::pair<int,int> nbr{-1,-1};
      for (auto [eid, v] : graph.adj[ni]) {
        if (!graph.edges[eid].in_service) continue;
        if (!graph.nodes[v].in_service) continue;
        nbr = {eid, v}; break;
      }
      if (nbr.first < 0) continue;
      if (eliminated_buses.count(graph.nodes[nbr.second].bus_id)) continue;

      ReductionAction act;
      act.type = ReductionActionType::PendantFold;
      act.eliminated_buses    = {cand.bus_id};
      act.eliminated_branches = {graph.edges[nbr.first].edge_id};
      act.retained_buses      = {graph.nodes[nbr.second].bus_id};
      act.bus_domain = cand.domain;
      act.reason = cand.reason;
      plan.actions.push_back(act);
      eliminated_buses.insert(cand.bus_id);
      eliminated_edges.insert(nbr.first);
      ++plan.n_buses_eliminated;
      ++plan.n_branches_eliminated;
    }
  }

  // ── Kron reductions ───────────────────────────────────────────────
  if (options.enable_kron_reduction) {
    std::vector<int> kron_candidates;
    for (const auto& cand : candidates.candidates) {
      if (cand.type != CandidateType::KronPassiveNode) continue;
      if (eliminated_buses.count(cand.bus_id)) continue;
      kron_candidates.push_back(cand.bus_id);
    }
    if (!kron_candidates.empty()) {
      ReductionAction act;
      act.type = ReductionActionType::KronEliminate;
      act.eliminated_buses = kron_candidates;
      // Retained buses: all non-eliminated
      for (const auto& cand : candidates.candidates) {
        if (cand.type == CandidateType::MustRetain)
          act.retained_buses.push_back(cand.bus_id);
      }
      act.reason = "Kron (Schur complement) passive node elimination";
      plan.actions.push_back(act);
      plan.n_buses_eliminated += static_cast<int>(kron_candidates.size());
    }
  }

  return plan;
}

}  // namespace hacdcpf::graph

/// src/graph/reduction_plan.cpp
/// =============================
/// Candidate classification and reduction plan generation.

#include "hacdcpf/graph/reduction_plan.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

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
    cand.bus = BusRef{nd.domain, nd.bus_id};

    // ── Must-retain rules ───────────────────────────────────────────
    bool must_retain = false;
    std::string retain_reason;

    if (nd.is_slack) {
      must_retain   = true;
      retain_reason = "slack / external grid bus";
    } else if (nd.is_voltage_controlled &&
               options.preserve_all_voltage_constrained_buses) {
      must_retain   = true;
      retain_reason = "PV / voltage-controlled bus";
    } else if (nd.has_generator && options.preserve_all_generator_buses) {
      must_retain   = true;
      retain_reason = "bus has generator";
    } else if (nd.has_storage) {
      must_retain   = true;
      retain_reason = "bus has storage device";
    } else if (nd.has_vsc_ac || nd.has_vsc_dc || nd.has_lcc) {
      must_retain   = true;
      retain_reason = "AC/DC converter terminal";
    } else if (nd.has_dcdc || nd.has_energy_router) {
      must_retain   = true;
      retain_reason = "multi-terminal converter terminal";
    } else if (nd.has_three_phase_model) {
      must_retain   = true;
      retain_reason = "bus participates in three-phase model";
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
        !nd.has_dcdc && !nd.has_lcc && !nd.has_energy_router &&
        !nd.has_three_phase_model && !nd.has_controllable;

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

  const bool enable_kron = options.enable_kron_reduction ||
      options.mode == ReductionMode::Moderate ||
      options.mode == ReductionMode::Aggressive;
  const bool enable_pendant = options.enable_pendant_reduction ||
      options.mode == ReductionMode::Aggressive;
  std::unordered_set<BusRef, BusRefHash> eliminated_buses;

  // Switch actions are audit records. The contraction executor chooses the
  // representative after voltage-base and multi-reference guards are applied.
  if (options.enable_switch_contraction) {
    for (int edge_pos = 0; edge_pos < graph.edge_count(); ++edge_pos) {
      const auto& edge = graph.edges[edge_pos];
      if (!edge.in_service) continue;
      const bool closed_switch =
          (edge.category == EdgeCategory::Switch ||
           edge.category == EdgeCategory::Breaker ||
           edge.category == EdgeCategory::DC_Switch) &&
          edge.is_closed_switch;
      const bool zero_line =
          (edge.category == EdgeCategory::AC_Line ||
           edge.category == EdgeCategory::DC_Line) &&
          std::hypot(edge.r_pu, edge.x_pu) <
              options.zero_impedance_threshold;
      if (!closed_switch && !zero_line) continue;

      ReductionAction action;
      action.type = ReductionActionType::SwitchContraction;
      action.method = ReductionMethod::SwitchContraction;
      action.affected_buses = {
          {graph.nodes[edge.from_node].domain, edge.from_bus_id},
          {graph.nodes[edge.to_node].domain, edge.to_bus_id}};
      action.eliminated_edge_positions = {edge_pos};
      action.reason = closed_switch ? "closed switch/breaker contraction"
                                    : "zero-impedance line contraction";
      plan.actions.push_back(std::move(action));
    }
  }

  for (const auto& candidate : candidates.candidates) {
    if (candidate.type != CandidateType::MustRetain) continue;
    ReductionAction action;
    action.type = ReductionActionType::Retain;
    action.method = ReductionMethod::None;
    action.retained_buses = {candidate.bus};
    action.reason = candidate.reason;
    plan.actions.push_back(std::move(action));
  }

  // ── Series reductions (degree-2 passive nodes) ───────────────────
  if (options.enable_series_reduction) {
    for (const auto& cand : candidates.candidates) {
      if (cand.type != CandidateType::ZeroInjectionDegree2) continue;
      int ni = (cand.bus.domain == NodeDomain::AC)
               ? graph.ac_node_idx(cand.bus.bus_id)
               : graph.dc_node_idx(cand.bus.bus_id);
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
      const BusRef neighbor_a{graph.nodes[nbrs[0].second].domain,
                              graph.nodes[nbrs[0].second].bus_id};
      const BusRef neighbor_b{graph.nodes[nbrs[1].second].domain,
                              graph.nodes[nbrs[1].second].bus_id};
      if (eliminated_buses.contains(neighbor_a) ||
          eliminated_buses.contains(neighbor_b))
        continue;

      // Build action
      ReductionAction act;
      act.type = ReductionActionType::SeriesReduction;
      act.method = ReductionMethod::SeriesReduction;
      act.eliminated_buses = {cand.bus};
      act.eliminated_edge_positions = {nbrs[0].first, nbrs[1].first};
      act.retained_buses = {neighbor_a, neighbor_b};
      act.reason = cand.reason;
      plan.actions.push_back(act);
      eliminated_buses.insert(cand.bus);
      ++plan.n_buses_eliminated;
      plan.n_branches_eliminated += 2; // two original branches → one new
    }
  }

  // ── Pendant reductions ────────────────────────────────────────────
  if (enable_pendant) {
    for (const auto& cand : candidates.candidates) {
      if (cand.type != CandidateType::PendantLoad) continue;
      int ni = (cand.bus.domain == NodeDomain::AC)
               ? graph.ac_node_idx(cand.bus.bus_id)
               : graph.dc_node_idx(cand.bus.bus_id);
      if (ni < 0) continue;
      if (eliminated_buses.contains(cand.bus)) continue;

      std::pair<int,int> nbr{-1,-1};
      for (auto [eid, v] : graph.adj[ni]) {
        if (!graph.edges[eid].in_service) continue;
        if (!graph.nodes[v].in_service) continue;
        nbr = {eid, v}; break;
      }
      if (nbr.first < 0) continue;
      const auto category = graph.edges[nbr.first].category;
      if (category != EdgeCategory::AC_Line &&
          category != EdgeCategory::DC_Line) {
        continue;
      }
      if (options.preserve_branch_flow_limited_edges &&
          graph.edges[nbr.first].rate_a_mva > 0.0) {
        continue;
      }
      const BusRef parent{graph.nodes[nbr.second].domain,
                          graph.nodes[nbr.second].bus_id};
      if (eliminated_buses.contains(parent)) continue;

      ReductionAction act;
      act.type = ReductionActionType::PendantFold;
      act.method = ReductionMethod::PendantReduction;
      act.eliminated_buses = {cand.bus};
      act.eliminated_edge_positions = {nbr.first};
      act.retained_buses = {parent};
      act.reason = cand.reason;
      plan.actions.push_back(act);
      eliminated_buses.insert(cand.bus);
      ++plan.n_buses_eliminated;
      ++plan.n_branches_eliminated;
    }
  }

  // ── Kron reductions ───────────────────────────────────────────────
  if (enable_kron) {
    for (NodeDomain domain : {NodeDomain::AC, NodeDomain::DC}) {
      std::vector<BusRef> kron_candidates;
      for (const auto& cand : candidates.candidates) {
        if (cand.bus.domain != domain) continue;
        if (cand.type != CandidateType::KronPassiveNode &&
            cand.type != CandidateType::ZeroInjectionPassiveInterior) {
          continue;
        }
        if (eliminated_buses.contains(cand.bus)) continue;
        kron_candidates.push_back(cand.bus);
      }
      if (kron_candidates.empty()) continue;

      ReductionAction act;
      act.type = ReductionActionType::KronEliminate;
      act.method = ReductionMethod::KronPassiveOnly;
      act.eliminated_buses = kron_candidates;
      act.max_fill_ratio = options.max_fill_ratio;
      for (const auto& cand : candidates.candidates) {
        if (cand.bus.domain != domain ||
            eliminated_buses.contains(cand.bus) ||
            std::find(kron_candidates.begin(), kron_candidates.end(),
                      cand.bus) != kron_candidates.end()) {
          continue;
        }
        act.retained_buses.push_back(cand.bus);
      }
      act.reason = "Kron (Schur complement) passive node elimination";
      plan.actions.push_back(act);
      plan.n_buses_eliminated += static_cast<int>(kron_candidates.size());
    }
  }

  return plan;
}

}  // namespace hacdcpf::graph

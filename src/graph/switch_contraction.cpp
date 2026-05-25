/// src/graph/switch_contraction.cpp
/// ==================================
/// Zero-impedance supernode merging using Union-Find (DSU).
///
/// Algorithm:
///   1. Union-Find: merge all pairs connected by zero-impedance / closed-
///      switch edges in the contracted graph.
///   2. For each component (super-node): aggregate loads, generators,
///      shunts; pick representative bus; redirect external branches.
///   3. Validate: voltage-base consistency, multi-slack detection.

#include "hacdcpf/graph/switch_contraction.hpp"

#include <cmath>
#include <numeric>
#include <unordered_set>

namespace hacdcpf::graph {

namespace {

// ── Union-Find (path compression + union by rank) ────────────────────

struct DSU {
  std::vector<int> parent, rank_;
  explicit DSU(int n) : parent(n), rank_(n, 0) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  int find(int x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  }
  void unite(int a, int b) {
    a = find(a); b = find(b);
    if (a == b) return;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent[b] = a;
    if (rank_[a] == rank_[b]) ++rank_[a];
  }
  bool same(int a, int b) { return find(a) == find(b); }
};

// ── Bus type priority ────────────────────────────────────────────────

static int bus_type_priority(BusType t) {
  switch (t) {
    case BusType::SLACK:    return 4;
    case BusType::PV:       return 3;
    case BusType::PQ:       return 2;
    case BusType::ISOLATED: return 1;
    default:                return 0;
  }
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────
// contract_zero_impedance_edges
// ─────────────────────────────────────────────────────────────────────

ContractionResult contract_zero_impedance_edges(
    const PowerSystemGraph&   graph,
    const HybridPowerSystem&  system,
    const ContractionOptions& options)
{
  ContractionResult result;
  result.contracted_system = system; // start from a copy

  const int n = graph.node_count();
  DSU dsu(n);

  // ── Step 1: Union nodes connected by zero-impedance edges ──────────
  for (const auto& e : graph.edges) {
    if (!e.in_service) continue;
    if (!e.is_zero_impedance && !e.is_closed_switch) continue;

    // Check which category we're handling
    bool do_merge = false;
    if (e.is_closed_switch && options.contract_closed_switches &&
        (e.category == EdgeCategory::Switch || e.category == EdgeCategory::Breaker))
      do_merge = true;
    if (e.is_zero_impedance && options.contract_zero_impedance_lines &&
        e.category == EdgeCategory::AC_Line)
      do_merge = true;

    if (!do_merge) continue;

    int fn = e.from_node, tn = e.to_node;
    if (fn < 0 || tn < 0) continue;

    // Voltage base check
    double bkv_f = graph.nodes[fn].base_kv;
    double bkv_t = graph.nodes[tn].base_kv;
    if (bkv_f > 0 && bkv_t > 0 &&
        std::abs(bkv_f - bkv_t) > options.voltage_base_tolerance * std::max(bkv_f, bkv_t)) {
      Diagnostic d;
      d.code = DiagCode::GraphVoltageBaseMismatch;
      d.message = "Cannot merge buses " +
                  std::to_string(e.from_bus_id) + " and " +
                  std::to_string(e.to_bus_id) +
                  ": base_kv mismatch (" + std::to_string(bkv_f) +
                  " vs " + std::to_string(bkv_t) + " kV).";
      d.related_buses = {e.from_bus_id, e.to_bus_id};
      result.diagnostics.push_back(d);
      continue;
    }

    dsu.unite(fn, tn);
  }

  // ── Step 2: Build super-node groups ─────────────────────────────────
  // Map: root node_idx → list of node_idxes in the group
  std::unordered_map<int, std::vector<int>> groups;
  for (int i = 0; i < n; ++i) {
    if (!graph.nodes[i].in_service) continue;
    groups[dsu.find(i)].push_back(i);
  }

  // ── Step 3: Validate multi-slack within a group ──────────────────────
  for (auto& [root, members] : groups) {
    int n_slack = 0;
    for (int ni : members) {
      if (graph.nodes[ni].is_slack) ++n_slack;
    }
    if (n_slack > 1) {
      // Collect bus IDs
      std::vector<int> bus_ids;
      for (int ni : members) bus_ids.push_back(graph.nodes[ni].bus_id);
      if (!options.allow_multi_slack_merge) {
        Diagnostic d;
        d.code    = DiagCode::GraphMultipleSlack;
        d.message = "Super-node would contain multiple slack buses.";
        d.related_buses = bus_ids;
        result.diagnostics.push_back(d);
      } else {
        Diagnostic d;
        d.code    = DiagCode::GraphMultipleSlack;
        d.message = "Warning: super-node contains multiple slack buses; "
                    "first slack is kept.";
        d.related_buses = bus_ids;
        result.diagnostics.push_back(d);
      }
    }
  }

  // ── Step 4: Choose representative bus for each super-node ─────────────
  // Representative = highest-priority bus type, then lowest bus_id
  std::unordered_map<int, int> root_to_rep; // root_node_idx → rep_bus_id

  for (auto& [root, members] : groups) {
    int best_ni  = members[0];
    int best_pri = bus_type_priority(graph.nodes[best_ni].ac_bus_type);
    for (int ni : members) {
      int pri = bus_type_priority(graph.nodes[ni].ac_bus_type);
      if (pri > best_pri || (pri == best_pri &&
          graph.nodes[ni].bus_id < graph.nodes[best_ni].bus_id)) {
        best_ni  = ni;
        best_pri = pri;
      }
    }
    root_to_rep[root] = graph.nodes[best_ni].bus_id;
  }

  // ── Step 5: Populate bus_to_super / super_to_buses maps ──────────────
  for (auto& [root, members] : groups) {
    int rep = root_to_rep[root];
    result.super_to_buses[rep] = {};
    for (int ni : members) {
      int bid = graph.nodes[ni].bus_id;
      result.bus_to_super[bid] = rep;
      result.super_to_buses[rep].push_back(bid);
    }
  }

  // ── Step 6: Build contraction records ────────────────────────────────
  for (auto& [rep, orig_buses] : result.super_to_buses) {
    if (orig_buses.size() == 1 && orig_buses[0] == rep) continue; // trivial
    SwitchContractionRecord rec;
    rec.original_bus_ids = orig_buses;
    rec.super_bus_id     = rep;
    rec.reason           = "zero-impedance / closed-switch merge";
    result.switch_records.push_back(rec);
  }

  // ── Step 7: Build contracted PowerSystemGraph ─────────────────────────
  PowerSystemGraph& cg = result.contracted_graph;

  // Add one node per super-node (using the representative bus data)
  for (auto& [root, members] : groups) {
    int rep_bid = root_to_rep[root];
    // Find the original graph node for the representative bus
    int rep_ni = graph.node_idx(rep_bid);
    if (rep_ni < 0) continue;

    const GraphNode& rep_node = graph.nodes[rep_ni];
    GraphNode new_node = rep_node;
    new_node.bus_id = rep_bid;

    // Aggregate flags from all members
    for (int ni : members) {
      const auto& nd = graph.nodes[ni];
      if (nd.has_generator)    new_node.has_generator    = true;
      if (nd.has_load)         new_node.has_load         = true;
      if (nd.has_shunt)        new_node.has_shunt        = true;
      if (nd.has_storage)      new_node.has_storage      = true;
      if (nd.has_vsc_ac)       new_node.has_vsc_ac       = true;
      if (nd.has_vsc_dc)       new_node.has_vsc_dc       = true;
      if (nd.has_dcdc)         new_node.has_dcdc         = true;
      if (nd.has_controllable) new_node.has_controllable = true;
      if (nd.is_slack)         new_node.is_slack         = true;
      if (nd.is_monitored)     new_node.is_monitored     = true;
      if (nd.is_voltage_controlled) new_node.is_voltage_controlled = true;
    }

    int new_ni = static_cast<int>(cg.nodes.size());
    cg.nodes.push_back(new_node);
    cg.bus_id_to_node_idx[rep_bid] = new_ni;
    cg.adj.emplace_back();
  }

  // Add edges, re-mapping endpoints to super-nodes;
  // skip internal edges (both endpoints in the same super-node)
  int new_eid = 0;
  for (const auto& e : graph.edges) {
    if (!e.in_service) continue;
    int rep_f = result.bus_to_super.count(e.from_bus_id)
                    ? result.bus_to_super.at(e.from_bus_id) : e.from_bus_id;
    int rep_t = result.bus_to_super.count(e.to_bus_id)
                    ? result.bus_to_super.at(e.to_bus_id) : e.to_bus_id;
    if (rep_f == rep_t) continue; // internal to super-node, skip

    int fn = cg.node_idx(rep_f);
    int tn = cg.node_idx(rep_t);
    if (fn < 0 || tn < 0) continue;

    GraphEdge ne = e;
    ne.edge_id     = new_eid++;
    ne.from_node   = fn;
    ne.to_node     = tn;
    ne.from_bus_id = rep_f;
    ne.to_bus_id   = rep_t;
    // Zero-impedance switches inside super-nodes were skipped above;
    // remaining zero-impedance branches between different super-nodes
    // are real branches (e.g. zero-resistance DC lines or distinct voltage
    // levels) — preserve them.
    ne.is_closed_switch   = false;
    ne.is_zero_impedance  = false; // they now connect distinct super-nodes
    int eid_idx = static_cast<int>(cg.edges.size());
    cg.edges.push_back(ne);
    cg.adj[fn].emplace_back(eid_idx, tn);
    cg.adj[tn].emplace_back(eid_idx, fn);
  }

  // ── Step 8: Aggregate AC bus quantities in contracted_system ─────────
  // For each super-node with multiple members, sum up pd/qd/gs/bs
  for (auto& [rep_bid, orig_buses] : result.super_to_buses) {
    if (orig_buses.size() <= 1) continue;
    // Find rep bus in contracted system
    ACBus* rep_bus_ptr = nullptr;
    for (auto& bus : result.contracted_system.ac.buses) {
      if (bus.index == rep_bid) { rep_bus_ptr = &bus; break; }
    }
    if (!rep_bus_ptr) continue;

    double pd_sum = 0, qd_sum = 0, gs_sum = 0, bs_sum = 0;
    for (int bid : orig_buses) {
      for (const auto& b : system.ac.buses) {
        if (b.index != bid) continue;
        pd_sum += b.pd_mw;
        qd_sum += b.qd_mvar;
        gs_sum += b.gs_mw;
        bs_sum += b.bs_mvar;
      }
    }
    rep_bus_ptr->pd_mw   = pd_sum;
    rep_bus_ptr->qd_mvar = qd_sum;
    rep_bus_ptr->gs_mw   = gs_sum;
    rep_bus_ptr->bs_mvar = bs_sum;

    // Set correct bus type using priority rules
    BusType best_type = BusType::ISOLATED;
    for (int bid : orig_buses) {
      for (const auto& b : system.ac.buses) {
        if (b.index != bid) continue;
        if (bus_type_priority(b.bus_type) > bus_type_priority(best_type))
          best_type = b.bus_type;
      }
    }
    rep_bus_ptr->bus_type = best_type;
  }

  // Remove non-representative buses from the contracted system
  {
    std::unordered_set<int> reps_set;
    for (auto& [bid, rep] : result.bus_to_super) reps_set.insert(rep);
    auto& buses = result.contracted_system.ac.buses;
    buses.erase(
      std::remove_if(buses.begin(), buses.end(),
        [&](const ACBus& b){ return reps_set.count(b.index) == 0 &&
                                    result.bus_to_super.count(b.index) &&
                                    result.bus_to_super.at(b.index) != b.index; }),
      buses.end());
  }

  // Re-map all component bus references to representative buses
  auto remap = [&](int bid) -> int {
    auto it = result.bus_to_super.find(bid);
    return (it != result.bus_to_super.end()) ? it->second : bid;
  };

  for (auto& gen : result.contracted_system.ac.generators)
    gen.bus = remap(gen.bus);
  for (auto& ld  : result.contracted_system.ac.loads)
    ld.bus = remap(ld.bus);
  for (auto& sh  : result.contracted_system.ac.shunts)
    sh.bus = remap(sh.bus);
  for (auto& st  : result.contracted_system.ac.storage)
    st.bus = remap(st.bus);
  for (auto& eg  : result.contracted_system.ac.external_grids)
    eg.bus = remap(eg.bus);
  for (auto& sg  : result.contracted_system.ac.static_generators)
    sg.bus = remap(sg.bus);
  for (auto& rg  : result.contracted_system.ac.renewable_gens)
    rg.bus = remap(rg.bus);
  for (auto& pv  : result.contracted_system.ac.pv_systems)
    pv.bus = remap(pv.bus);

  for (auto& br  : result.contracted_system.ac.branches) {
    br.from_bus = remap(br.from_bus);
    br.to_bus   = remap(br.to_bus);
  }
  for (auto& tr  : result.contracted_system.ac.transformers_2w) {
    tr.hv_bus = remap(tr.hv_bus);
    tr.lv_bus = remap(tr.lv_bus);
  }
  for (auto& vsc : result.contracted_system.vsc_converters)
    vsc.bus_ac = remap(vsc.bus_ac);

  // Remove self-loop branches (from_bus == to_bus after remap)
  {
    auto& brs = result.contracted_system.ac.branches;
    brs.erase(std::remove_if(brs.begin(), brs.end(),
        [](const ACBranch& b){ return b.from_bus == b.to_bus; }),
      brs.end());
  }

  return result;
}

}  // namespace hacdcpf::graph

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

// DC_V (voltage reference) must survive contraction; DC_P (load/gen) is
// subordinate — mirrors the AC SLACK > PV > PQ hierarchy.
static int dc_bus_type_priority(DCBusType t) {
  switch (t) {
    case DCBusType::DC_V: return 2;
    case DCBusType::DC_P: return 1;
    default:              return 0;
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
    // Quick reject: switch-type edges that are open can never contract;
    // line-type edges are only candidates when contract_zero_impedance_lines
    // is set; all other edge types (e.g. VSC) are never contracted.
    const bool is_switch_type = (e.category == EdgeCategory::Switch ||
                                 e.category == EdgeCategory::Breaker ||
                                 e.category == EdgeCategory::DC_Switch);
    const bool is_line_type   = (e.category == EdgeCategory::AC_Line ||
                                 e.category == EdgeCategory::DC_Line);
    if (is_switch_type && !e.is_closed_switch) continue;
    if (is_line_type  && !options.contract_zero_impedance_lines) continue;
    if (!is_switch_type && !is_line_type) continue;

    // Decide whether to merge this edge.
    // contract_closed_switches governs Switch; contract_closed_breakers governs
    // Breaker and DC_Switch.  For line edges, always re-evaluate from the raw
    // impedance magnitude — the pre-set is_zero_impedance flag is intentionally
    // ignored so that the threshold can be raised or lowered at contraction
    // time independently of the value used when the graph was originally built.
    bool do_merge = false;
    if (is_switch_type && e.is_closed_switch) {
      if (e.category == EdgeCategory::Switch && options.contract_closed_switches)
        do_merge = true;
      if ((e.category == EdgeCategory::Breaker ||
           e.category == EdgeCategory::DC_Switch) &&
          options.contract_closed_breakers)
        do_merge = true;
    }
    if (!do_merge && is_line_type && options.contract_zero_impedance_lines) {
      // DC_Line and AC_Line both handled here.
      if (std::hypot(e.r_pu, e.x_pu) < options.zero_impedance_threshold)
        do_merge = true;
    }

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

    // ── Multi-slack guard ──────────────────────────────────────────────
    // If merging fn and tn would create a super-node containing more than
    // one slack bus, and allow_multi_slack_merge is false, skip the union
    // and record a diagnostic.  Checking here (before unite) avoids the need
    // to undo DSU merges later.
    if (!options.allow_multi_slack_merge &&
        dsu.find(fn) != dsu.find(tn)) {
      // Scan all nodes to check if each group already has a slack.
      bool fn_root_has_slack = false, tn_root_has_slack = false;
      int fn_root = dsu.find(fn), tn_root = dsu.find(tn);
      for (int k = 0; k < n; ++k) {
        if (!graph.nodes[k].in_service || !graph.nodes[k].is_slack) continue;
        int kr = dsu.find(k);
        if (kr == fn_root) fn_root_has_slack = true;
        if (kr == tn_root) tn_root_has_slack = true;
      }
      if (fn_root_has_slack && tn_root_has_slack) {
        Diagnostic d;
        d.code    = DiagCode::GraphMultipleSlack;
        d.message = "Skipped merge of buses " +
                    std::to_string(e.from_bus_id) + " and " +
                    std::to_string(e.to_bus_id) +
                    ": would combine two slack buses into one super-node.";
        d.related_buses = {e.from_bus_id, e.to_bus_id};
        result.diagnostics.push_back(d);
        continue;  // do NOT unite
      }
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
  // Representative = highest-priority bus type, then lowest bus_id.
  // Also track the domain (AC/DC) of each representative so later steps can
  // use the correct node-lookup map instead of AC-first fallback.
  std::unordered_map<int, int>        root_to_rep;        // root_node_idx → rep_bus_id
  std::unordered_map<int, NodeDomain> root_to_rep_domain; // root_node_idx → domain

  for (auto& [root, members] : groups) {
    int best_ni  = members[0];
    // Use the domain-appropriate priority function so that a DC_V bus is
    // always preferred over DC_P when contracting DC super-nodes.
    const NodeDomain grp_dom = graph.nodes[best_ni].domain;
    auto node_pri = [&](int ni) -> int {
      return (graph.nodes[ni].domain == NodeDomain::DC)
          ? dc_bus_type_priority(graph.nodes[ni].dc_bus_type)
          : bus_type_priority(graph.nodes[ni].ac_bus_type);
    };
    int best_pri = node_pri(best_ni);
    for (int ni : members) {
      int pri = node_pri(ni);
      if (pri > best_pri || (pri == best_pri &&
          graph.nodes[ni].bus_id < graph.nodes[best_ni].bus_id)) {
        best_ni  = ni;
        best_pri = pri;
      }
    }
    (void)grp_dom;
    root_to_rep[root]        = graph.nodes[best_ni].bus_id;
    root_to_rep_domain[root] = graph.nodes[best_ni].domain;
  }

  // ── Step 5: Populate domain-qualified and legacy bus maps ────────────
  // Domain maps are unambiguous even when AC bus N and DC bus N coexist.
  // Legacy maps are kept for backward compatibility but may lose entries
  // when AC and DC share a bus ID (DC overwrites AC in that case).
  for (auto& [root, members] : groups) {
    int rep = root_to_rep[root];
    NodeDomain dom = root_to_rep_domain[root];
    auto& b2s_dom = (dom == NodeDomain::AC) ? result.ac_bus_to_super
                                            : result.dc_bus_to_super;
    auto& s2b_dom = (dom == NodeDomain::AC) ? result.ac_super_to_buses
                                            : result.dc_super_to_buses;
    s2b_dom[rep] = {};
    result.super_to_buses[rep] = {};  // legacy
    for (int ni : members) {
      int bid = graph.nodes[ni].bus_id;
      b2s_dom[bid]           = rep;
      result.bus_to_super[bid] = rep;  // legacy (may overwrite on AC/DC ID collision)
      s2b_dom[rep].push_back(bid);
      result.super_to_buses[rep].push_back(bid);  // legacy
    }
  }

  // ── Step 6: Build contraction records (from domain maps) ─────────────
  auto make_records = [&](const std::unordered_map<int, std::vector<int>>& s2b) {
    for (auto& [rep, orig_buses] : s2b) {
      if (orig_buses.size() == 1 && orig_buses[0] == rep) continue; // trivial
      SwitchContractionRecord rec;
      rec.original_bus_ids = orig_buses;
      rec.super_bus_id     = rep;
      rec.reason           = "zero-impedance / closed-switch merge";
      result.switch_records.push_back(rec);
    }
  };
  make_records(result.ac_super_to_buses);
  make_records(result.dc_super_to_buses);

  // ── Step 7: Build contracted PowerSystemGraph ─────────────────────────
  PowerSystemGraph& cg = result.contracted_graph;

  // Add one node per super-node (using the representative bus data)
  for (auto& [root, members] : groups) {
    int rep_bid = root_to_rep[root];
    // Find the original graph node for the representative bus.
    // Use the domain-qualified lookup to avoid AC/DC bus ID collision.
    const NodeDomain rep_dom = root_to_rep_domain.at(root);
    int rep_ni = (rep_dom == NodeDomain::AC) ? graph.ac_node_idx(rep_bid)
                                             : graph.dc_node_idx(rep_bid);
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
    // Legacy map: only AC representatives, matching the convention in
    // power_system_graph.cpp (DC bus IDs are NOT written to the legacy map
    // to prevent AC/DC same-ID collisions).
    if (rep_node.domain == NodeDomain::AC)
      cg.bus_id_to_node_idx[rep_bid] = new_ni;
    if (rep_node.domain == NodeDomain::AC)
      cg.ac_bus_id_to_node_idx[rep_bid] = new_ni;
    else
      cg.dc_bus_id_to_node_idx[rep_bid] = new_ni;
    cg.adj.emplace_back();
  }

  // Add edges, re-mapping endpoints to super-nodes;
  // skip internal edges (both endpoints in the same super-node)
  int new_eid = 0;
  for (const auto& e : graph.edges) {
    if (!e.in_service) continue;
    // Use domain-qualified maps so AC bus N and DC bus N are not confused.
    const NodeDomain dom_f = graph.nodes[e.from_node].domain;
    const NodeDomain dom_t = graph.nodes[e.to_node].domain;
    const auto& b2s_f = (dom_f == NodeDomain::AC) ? result.ac_bus_to_super
                                                   : result.dc_bus_to_super;
    const auto& b2s_t = (dom_t == NodeDomain::AC) ? result.ac_bus_to_super
                                                   : result.dc_bus_to_super;
    int rep_f = b2s_f.count(e.from_bus_id) ? b2s_f.at(e.from_bus_id) : e.from_bus_id;
    int rep_t = b2s_t.count(e.to_bus_id)   ? b2s_t.at(e.to_bus_id)   : e.to_bus_id;
    // Skip only when both endpoints collapse into the same super-node AND are
    // in the same domain.  A cross-domain edge (e.g. a VSC connecting AC bus N
    // to DC bus N) must NOT be discarded just because the two representative
    // bus IDs happen to be numerically equal.
    if (dom_f == dom_t && rep_f == rep_t) continue; // internal same-domain super-node

    int fn = (dom_f == NodeDomain::AC) ? cg.ac_node_idx(rep_f) : cg.dc_node_idx(rep_f);
    int tn = (dom_t == NodeDomain::AC) ? cg.ac_node_idx(rep_t) : cg.dc_node_idx(rep_t);
    if (fn < 0 || tn < 0) continue;

    GraphEdge ne = e;
    ne.edge_id     = new_eid++;
    ne.from_node   = fn;
    ne.to_node     = tn;
    ne.from_bus_id = rep_f;
    ne.to_bus_id   = rep_t;
    // Retain is_closed_switch and is_zero_impedance from the source edge.
    // If contraction was disabled or the edge spans distinct super-nodes due
    // to a base-kv mismatch, the original semantics remain valid — clearing
    // these flags would silently hide closed switches from downstream
    // contraction passes, diagnostics, and export.
    int eid_idx = static_cast<int>(cg.edges.size());
    cg.edges.push_back(ne);
    cg.adj[fn].emplace_back(eid_idx, tn);
    cg.adj[tn].emplace_back(eid_idx, fn);
  }

  // ── Step 8: Aggregate AC bus quantities in contracted_system ─────────
  // For each super-node with multiple members, sum up pd/qd/gs/bs.
  // Use ac_super_to_buses (not the legacy super_to_buses) so that DC buses
  // with the same numeric ID never corrupt AC bus aggregation.
  for (auto& [rep_bid, orig_buses] : result.ac_super_to_buses) {
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

  // ── Step 8b: Aggregate DC bus quantities in contracted_system ────────
  // Same logic as Step 8 above, but for dc_super_to_buses.
  // DCBus::pd_mw holds bus-level DC load; n_customers / is_load are OR-ed;
  // bus_type is chosen using dc_bus_type_priority (DC_V beats DC_P).
  for (auto& [rep_bid, orig_buses] : result.dc_super_to_buses) {
    if (orig_buses.size() <= 1) continue;
    DCBus* rep_ptr = nullptr;
    for (auto& bus : result.contracted_system.dc.buses) {
      if (bus.index == rep_bid) { rep_ptr = &bus; break; }
    }
    if (!rep_ptr) continue;

    double pd_sum   = 0.0;
    int    n_cust   = 0;
    bool   is_load  = false;
    DCBusType best_type = DCBusType::DC_P;
    for (int bid : orig_buses) {
      for (const auto& b : system.dc.buses) {
        if (b.index != bid) continue;
        pd_sum  += b.pd_mw;
        n_cust  += b.n_customers;
        is_load |= b.is_load;
        if (dc_bus_type_priority(b.bus_type) > dc_bus_type_priority(best_type))
          best_type = b.bus_type;
      }
    }
    rep_ptr->pd_mw        = pd_sum;
    rep_ptr->n_customers  = n_cust;
    rep_ptr->is_load      = is_load;
    rep_ptr->bus_type     = best_type;
  }

  // Remove non-representative buses from the contracted system
  {
    std::unordered_set<int> reps_set;
    for (auto& [bid, rep] : result.ac_bus_to_super) reps_set.insert(rep);
    auto& buses = result.contracted_system.ac.buses;
    buses.erase(
      std::remove_if(buses.begin(), buses.end(),
        [&](const ACBus& b){ return reps_set.count(b.index) == 0 &&
                                    result.ac_bus_to_super.count(b.index) &&
                                    result.ac_bus_to_super.at(b.index) != b.index; }),
      buses.end());
  }

  // Re-map all component bus references to representative buses.
  // AC components use ac_bus_to_super; DC bus references (dc.branches etc.)
  // use dc_bus_to_super.  The lambdas are named accordingly.
  auto remap_ac = [&](int bid) -> int {
    auto it = result.ac_bus_to_super.find(bid);
    return (it != result.ac_bus_to_super.end()) ? it->second : bid;
  };
  auto remap_dc = [&](int bid) -> int {
    auto it = result.dc_bus_to_super.find(bid);
    return (it != result.dc_bus_to_super.end()) ? it->second : bid;
  };
  // Backward-compat alias used for AC-only component lists below.
  auto& remap = remap_ac;  // NOLINT(misc-const-correctness)

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
  // Remaining AC bus-referencing components
  for (auto& fl  : result.contracted_system.ac.flexible_loads)
    fl.bus = remap(fl.bus);
  for (auto& al  : result.contracted_system.ac.asymmetric_loads)
    al.bus = remap(al.bus);
  for (auto& cs  : result.contracted_system.ac.charging_stations)
    cs.bus = remap(cs.bus);
  for (auto& mo  : result.contracted_system.ac.motors)
    mo.bus = remap(mo.bus);
  for (auto& sw  : result.contracted_system.ac.switches) {
    sw.bus_from = remap(sw.bus_from);
    sw.bus_to   = remap(sw.bus_to);
  }
  for (auto& cb  : result.contracted_system.ac.circuit_breakers) {
    cb.bus_from = remap(cb.bus_from);
    cb.bus_to   = remap(cb.bus_to);
  }

  for (auto& br  : result.contracted_system.ac.branches) {
    br.from_bus = remap(br.from_bus);
    br.to_bus   = remap(br.to_bus);
  }
  for (auto& tr  : result.contracted_system.ac.transformers_2w) {
    tr.hv_bus = remap(tr.hv_bus);
    tr.lv_bus = remap(tr.lv_bus);
  }
  for (auto& tr3 : result.contracted_system.ac.transformers_3w) {
    tr3.hv_bus = remap(tr3.hv_bus);
    tr3.mv_bus = remap(tr3.mv_bus);
    tr3.lv_bus = remap(tr3.lv_bus);
  }
  for (auto& vsc : result.contracted_system.vsc_converters) {
    vsc.bus_ac = remap_ac(vsc.bus_ac);
    vsc.bus_dc = remap_dc(vsc.bus_dc);
  }

  // Remap DC component bus references using dc_bus_to_super
  for (auto& br : result.contracted_system.dc.branches) {
    br.from_bus = remap_dc(br.from_bus);
    br.to_bus   = remap_dc(br.to_bus);
  }

  // Single-bus DC components
  for (auto& ld  : result.contracted_system.dc.loads)
    ld.bus = remap_dc(ld.bus);
  for (auto& st  : result.contracted_system.dc.storage)
    st.bus = remap_dc(st.bus);
  for (auto& sg  : result.contracted_system.dc.static_generators)
    sg.bus = remap_dc(sg.bus);
  for (auto& dsg : result.contracted_system.dc.dc_static_generators)
    dsg.bus = remap_dc(dsg.bus);
  for (auto& pv  : result.contracted_system.dc.pv_arrays)
    pv.bus = remap_dc(pv.bus);

  // Two-terminal DC components
  for (auto& dd  : result.contracted_system.dc.dcdc_converters) {
    dd.bus_in  = remap_dc(dd.bus_in);
    dd.bus_out = remap_dc(dd.bus_out);
  }
  for (auto& cb  : result.contracted_system.dc.dc_circuit_breakers) {
    cb.bus_from = remap_dc(cb.bus_from);
    cb.bus_to   = remap_dc(cb.bus_to);
  }

  // Remove non-representative DC buses (analogous to the AC bus removal above)
  {
    std::unordered_set<int> dc_reps_set;
    for (auto& [bid, rep] : result.dc_bus_to_super) dc_reps_set.insert(rep);
    auto& dc_buses = result.contracted_system.dc.buses;
    dc_buses.erase(
      std::remove_if(dc_buses.begin(), dc_buses.end(),
        [&](const DCBus& b) {
          return result.dc_bus_to_super.count(b.index) &&
                 result.dc_bus_to_super.at(b.index) != b.index;
        }),
      dc_buses.end());
  }

  // Remove self-loop branches (from_bus == to_bus after remap)
  {
    auto& brs = result.contracted_system.ac.branches;
    brs.erase(std::remove_if(brs.begin(), brs.end(),
        [](const ACBranch& b){ return b.from_bus == b.to_bus; }),
      brs.end());
  }
  {
    auto& brs = result.contracted_system.dc.branches;
    brs.erase(std::remove_if(brs.begin(), brs.end(),
        [](const DCBranch& b){ return b.from_bus == b.to_bus; }),
      brs.end());
  }
  // Remove two-terminal DC components that became self-loops after remap
  {
    auto& dcs = result.contracted_system.dc.dcdc_converters;
    dcs.erase(std::remove_if(dcs.begin(), dcs.end(),
        [](const DCDCConverter& d){ return d.bus_in == d.bus_out; }),
      dcs.end());
  }
  {
    auto& cbs = result.contracted_system.dc.dc_circuit_breakers;
    cbs.erase(std::remove_if(cbs.begin(), cbs.end(),
        [](const DCCircuitBreaker& c){ return c.bus_from == c.bus_to; }),
      cbs.end());
  }
  // Remove AC switch / circuit-breaker self-loops that formed after remap.
  // Without this cleanup, subsequent graph builds would see self-loop edges
  // that corrupt radiality, cycle, and topology analyses.
  {
    auto& sws = result.contracted_system.ac.switches;
    sws.erase(std::remove_if(sws.begin(), sws.end(),
        [](const Switch& sw){ return sw.bus_from == sw.bus_to; }),
      sws.end());
  }
  {
    auto& cbs = result.contracted_system.ac.circuit_breakers;
    cbs.erase(std::remove_if(cbs.begin(), cbs.end(),
        [](const CircuitBreaker& cb){ return cb.bus_from == cb.bus_to; }),
      cbs.end());
  }

  return result;
}

}  // namespace hacdcpf::graph

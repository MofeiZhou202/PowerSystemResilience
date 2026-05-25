/// src/graph/topology_analysis.cpp
/// =================================
/// Full topological analysis: islands, radiality, bridges, articulation
/// points, and fundamental cycle basis.
///
/// Algorithms:
///   - Connected components: iterative DFS with Union-Find
///   - Bridges / articulation points: Tarjan's DFS (O(V+E))
///   - Cycle basis: spanning-tree + back-edge detection

#include "hacdcpf/graph/topology_analysis.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stack>
#include <unordered_set>

namespace hacdcpf::graph {

// ─────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────

namespace {

// Iterative DFS returning component assignment vector.
// Returns: comp[node_idx] = component id (0..n_components-1)
// Considers only in-service nodes and in-service edges.
std::vector<int> dfs_components(const PowerSystemGraph& g) {
  const int n = g.node_count();
  std::vector<int> comp(n, -1);
  int cid = 0;
  std::vector<int> stk;
  for (int start = 0; start < n; ++start) {
    if (!g.nodes[start].in_service) continue;
    if (comp[start] >= 0) continue;
    comp[start] = cid;
    stk.clear();
    stk.push_back(start);
    while (!stk.empty()) {
      int u = stk.back(); stk.pop_back();
      for (auto [eid, v] : g.adj[u]) {
        if (!g.edges[eid].in_service) continue;
        if (!g.nodes[v].in_service) continue;
        if (comp[v] >= 0) continue;
        comp[v] = cid;
        stk.push_back(v);
      }
    }
    ++cid;
  }
  return comp;
}

// Tarjan bridge/articulation-point algorithm (iterative).
// out_bridges:   indices in g.edges[] that are cut-edges
// out_apoints:   indices in g.nodes[] that are articulation points
void tarjan_bridge_ap(const PowerSystemGraph& g,
                      std::vector<int>& out_bridges,
                      std::vector<int>& out_apoints) {
  const int n = g.node_count();
  std::vector<int>  disc(n, -1), low(n, 0), parent(n, -1);
  std::vector<bool> is_ap(n, false);
  int timer = 0;

  // Iterative Tarjan using explicit stack
  // Stack frame: (node, adj_iterator_index, parent_edge_id)
  struct Frame { int u; int ai; int parent_eid; };
  std::vector<Frame> stk;

  for (int root = 0; root < n; ++root) {
    if (!g.nodes[root].in_service) continue;
    if (disc[root] >= 0) continue;

    stk.push_back({root, 0, -1});
    disc[root] = low[root] = timer++;
    int root_child_count = 0;

    while (!stk.empty()) {
      auto& [u, ai, pe] = stk.back();
      if (ai < static_cast<int>(g.adj[u].size())) {
        auto [eid, v] = g.adj[u][ai];
        ++ai;
        if (!g.edges[eid].in_service) continue;
        if (!g.nodes[v].in_service) continue;
        // Skip the edge we came from (for simple graphs); but handle
        // multi-edges by checking edge id, not just neighbor
        if (eid == pe) continue;

        if (disc[v] < 0) {
          // Tree edge
          parent[v] = u;
          disc[v] = low[v] = timer++;
          if (u == root) ++root_child_count;
          stk.push_back({v, 0, eid});
        } else {
          // Back edge
          if (low[u] > disc[v]) low[u] = disc[v];
        }
      } else {
        // Done processing u — propagate low upward
        stk.pop_back();
        if (!stk.empty()) {
          auto& [p, pai, ppe] = stk.back();
          (void)pai; (void)ppe;
          if (low[p] > low[u]) low[p] = low[u];
          // Articulation point check (non-root)
          if (parent[p] >= 0 && low[u] >= disc[p])
            is_ap[p] = true;
          // Bridge check
          if (low[u] > disc[p]) {
            // Find the edge id connecting p -> u
            // The parent edge id is in the child frame we just popped
            // We stored pe in the child's frame; retrieve it
            // (pe was set when we pushed v's frame)
            // Search adj[p] for edge to u:
            for (auto [eid2, nb] : g.adj[p]) {
              if (nb == u && g.edges[eid2].in_service) {
                out_bridges.push_back(eid2);
                break;
              }
            }
          }
        }
      }
    }
    // Root articulation point
    if (root_child_count > 1) is_ap[root] = true;
  }

  for (int i = 0; i < n; ++i)
    if (is_ap[i]) out_apoints.push_back(i);
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────
// count_islands
// ─────────────────────────────────────────────────────────────────────

int count_islands(const PowerSystemGraph& g) {
  const auto comp = dfs_components(g);
  if (comp.empty()) return 0;
  int mx = -1;
  for (int c : comp) if (c > mx) mx = c;
  return mx + 1;
}

// ─────────────────────────────────────────────────────────────────────
// is_connected
// ─────────────────────────────────────────────────────────────────────

bool is_connected(const PowerSystemGraph& g) {
  return count_islands(g) == 1;
}

// ─────────────────────────────────────────────────────────────────────
// is_radial
// ─────────────────────────────────────────────────────────────────────

bool is_radial(const PowerSystemGraph& g) {
  // Count in-service nodes and in-service edges
  int n_nodes = 0, n_edges = 0;
  for (const auto& nd : g.nodes) if (nd.in_service) ++n_nodes;
  for (const auto& e  : g.edges) if (e.in_service)  ++n_edges;
  if (n_nodes == 0) return true;
  // Radial iff connected and #edges == #nodes - 1
  return is_connected(g) && (n_edges == n_nodes - 1);
}

// ─────────────────────────────────────────────────────────────────────
// find_bridges
// ─────────────────────────────────────────────────────────────────────

std::vector<int> find_bridges(const PowerSystemGraph& g) {
  std::vector<int> bridges, apoints;
  tarjan_bridge_ap(g, bridges, apoints);
  return bridges;
}

// ─────────────────────────────────────────────────────────────────────
// find_articulation_points
// ─────────────────────────────────────────────────────────────────────

std::vector<int> find_articulation_points(const PowerSystemGraph& g) {
  std::vector<int> bridges, apoints;
  tarjan_bridge_ap(g, bridges, apoints);
  return apoints;
}

// ─────────────────────────────────────────────────────────────────────
// find_fundamental_cycles
// ─────────────────────────────────────────────────────────────────────

std::vector<std::vector<int>> find_fundamental_cycles(
    const PowerSystemGraph& g)
{
  const int n = g.node_count();
  std::vector<std::vector<int>> cycles;

  // Approach: build a spanning forest edge-by-edge using Union-Find.
  // Every non-tree edge (u,v) defines one fundamental cycle:
  // the cycle consists of the non-tree edge plus the unique tree path u→v.
  //
  // After building the spanning tree we recover the path u→v by
  // tracing parent pointers from u and v up to their LCA.

  // ── Union-Find (path-compression + union-by-rank) ──────────────────
  std::vector<int> parent(n), rank(n, 0);
  std::iota(parent.begin(), parent.end(), 0);
  std::function<int(int)> find = [&](int x) -> int {
    if (parent[x] != x) parent[x] = find(parent[x]);
    return parent[x];
  };
  auto unite = [&](int x, int y) {
    int rx = find(x), ry = find(y);
    if (rx == ry) return false;
    if (rank[rx] < rank[ry]) std::swap(rx, ry);
    parent[ry] = rx;
    if (rank[rx] == rank[ry]) ++rank[rx];
    return true;
  };

  // ── Build spanning tree ────────────────────────────────────────────
  // tree_parent[v] = (parent_node, edge_id) or (-1,-1) for roots
  std::vector<std::pair<int,int>> tree_parent(n, {-1,-1});
  std::vector<int>                tree_depth(n, 0);
  std::vector<int>                non_tree_edges;

  // First pass: BFS to build the spanning tree and collect non-tree edges
  std::vector<bool> seen_edge(static_cast<int>(g.edges.size()), false);
  std::vector<bool> bfs_visited(n, false);
  std::queue<int> bfs_q;

  for (int root = 0; root < n; ++root) {
    if (!g.nodes[root].in_service) continue;
    if (bfs_visited[root]) continue;
    bfs_q.push(root);
    bfs_visited[root] = true;
    while (!bfs_q.empty()) {
      int u = bfs_q.front(); bfs_q.pop();
      for (auto [eid, v] : g.adj[u]) {
        if (!g.edges[eid].in_service) continue;
        if (!g.nodes[v].in_service) continue;
        if (seen_edge[eid]) continue;
        seen_edge[eid] = true;
        if (!bfs_visited[v]) {
          bfs_visited[v]  = true;
          tree_parent[v]  = {u, eid};
          tree_depth[v]   = tree_depth[u] + 1;
          unite(u, v);
          bfs_q.push(v);
        } else if (find(u) == find(v)) {
          // Both already connected → non-tree (cycle-defining) edge
          non_tree_edges.push_back(eid);
        }
      }
    }
  }

  // ── Trace cycle for each non-tree edge ────────────────────────────
  for (int eid : non_tree_edges) {
    const auto& e = g.edges[eid];
    int u = e.from_node, v = e.to_node;
    // Bring u and v to the same depth, then trace to LCA
    std::vector<int> path_u, path_v;
    int du = tree_depth[u], dv = tree_depth[v];
    int cu = u, cv = v;
    while (du > dv) {
      auto [pu, eu] = tree_parent[cu];
      path_u.push_back(eu);
      cu = pu; --du;
    }
    while (dv > du) {
      auto [pv, ev] = tree_parent[cv];
      path_v.push_back(ev);
      cv = pv; --dv;
    }
    while (cu != cv) {
      auto [pu, eu] = tree_parent[cu];
      path_u.push_back(eu);
      cu = pu;
      auto [pv, ev] = tree_parent[cv];
      path_v.push_back(ev);
      cv = pv;
    }
    std::vector<int> cycle;
    cycle.push_back(eid);
    cycle.insert(cycle.end(), path_u.begin(), path_u.end());
    cycle.insert(cycle.end(), path_v.begin(), path_v.end());
    cycles.push_back(std::move(cycle));
  }

  return cycles;
}

// ─────────────────────────────────────────────────────────────────────
// analyze_topology  (full report)
// ─────────────────────────────────────────────────────────────────────

TopologyReport analyze_topology(const PowerSystemGraph& g) {
  TopologyReport rep;

  // ── Component assignment ──────────────────────────────────────────
  const auto comp = dfs_components(g);
  int n_components = 0;
  for (int c : comp) if (c >= n_components) n_components = c + 1;

  // Collect island info
  rep.islands.resize(n_components);
  for (int k = 0; k < n_components; ++k) {
    rep.islands[k].island_id = k;
    rep.islands[k].status = IslandStatus::Valid;
  }

  for (int ni = 0; ni < g.node_count(); ++ni) {
    if (!g.nodes[ni].in_service) continue;
    int cid = comp[ni];
    if (cid < 0) continue;
    auto& isl = rep.islands[cid];
    isl.bus_ids.push_back(g.nodes[ni].bus_id);

    if (g.nodes[ni].domain == NodeDomain::AC) {
      isl.domain = NodeDomain::AC;
      ++rep.n_ac_islands;  // overcounts, fixed below
      if (g.nodes[ni].is_slack) isl.has_ac_slack = true;
    } else {
      isl.domain = NodeDomain::DC;
      if (g.nodes[ni].is_slack) isl.has_dc_voltage_ref = true;
    }
  }

  // Fix island count (overcounted above)
  rep.n_ac_islands = 0; rep.n_dc_islands = 0;
  for (const auto& isl : rep.islands) {
    if (isl.domain == NodeDomain::AC) ++rep.n_ac_islands;
    else                              ++rep.n_dc_islands;
  }

  // ── Island validity ───────────────────────────────────────────────
  rep.all_islands_valid = true;
  for (auto& isl : rep.islands) {
    if (isl.bus_ids.empty()) {
      isl.status = IslandStatus::Empty;
      continue;
    }
    if (isl.domain == NodeDomain::AC && !isl.has_ac_slack) {
      isl.status = IslandStatus::NoSlack;
      rep.all_islands_valid = false;
      Diagnostic d;
      d.code    = DiagCode::GraphNoSlackInIsland;
      d.message = "AC island " + std::to_string(isl.island_id) +
                  " has no slack bus or external grid.";
      d.related_buses = isl.bus_ids;
      rep.diagnostics.push_back(d);
    } else if (isl.domain == NodeDomain::DC && !isl.has_dc_voltage_ref) {
      isl.status = IslandStatus::NoDCVoltageRef;
      rep.all_islands_valid = false;
      Diagnostic d;
      d.code    = DiagCode::GraphNoDCVoltageRef;
      d.message = "DC island " + std::to_string(isl.island_id) +
                  " has no voltage-reference (DC_V) bus.";
      d.related_buses = isl.bus_ids;
      rep.diagnostics.push_back(d);
    }
  }

  // ── Isolated load nodes ───────────────────────────────────────────
  // A bus with load but no neighbour through in-service edges
  for (int ni = 0; ni < g.node_count(); ++ni) {
    if (!g.nodes[ni].in_service) continue;
    if (!g.nodes[ni].has_load) continue;
    bool has_neighbour = false;
    for (auto [eid, v] : g.adj[ni]) {
      if (g.edges[eid].in_service && g.nodes[v].in_service) {
        has_neighbour = true; break;
      }
    }
    if (!has_neighbour) {
      rep.all_islands_valid = false;
      Diagnostic d;
      d.code    = DiagCode::GraphIsolatedLoad;
      d.message = "Bus " + std::to_string(g.nodes[ni].bus_id) +
                  " has load but no in-service connection.";
      d.related_buses = {g.nodes[ni].bus_id};
      rep.diagnostics.push_back(d);
    }
  }

  // ── Radiality ─────────────────────────────────────────────────────
  int n_nodes_svc = 0, n_edges_svc = 0;
  for (const auto& nd : g.nodes) if (nd.in_service) ++n_nodes_svc;
  for (const auto& e  : g.edges) if (e.in_service)  ++n_edges_svc;

  rep.is_connected = (n_components == 1);
  // cycle count = edges - nodes + components (for a forest)
  rep.cycle_count  = n_edges_svc - n_nodes_svc + n_components;
  if (rep.cycle_count < 0) rep.cycle_count = 0;
  rep.is_radial = rep.is_connected && (rep.cycle_count == 0);

  // ── Bridges and articulation points ──────────────────────────────
  std::vector<int> ap_node_idxs;
  tarjan_bridge_ap(g, rep.bridge_edge_ids, ap_node_idxs);
  // Deduplicate bridges
  {
    std::sort(rep.bridge_edge_ids.begin(), rep.bridge_edge_ids.end());
    rep.bridge_edge_ids.erase(
        std::unique(rep.bridge_edge_ids.begin(), rep.bridge_edge_ids.end()),
        rep.bridge_edge_ids.end());
  }
  // Convert node indices to bus IDs for cut vertices
  for (int ni : ap_node_idxs)
    rep.cut_vertex_bus_ids.push_back(g.nodes[ni].bus_id);

  // ── Fundamental cycles ───────────────────────────────────────────
  if (rep.cycle_count > 0)
    rep.fundamental_cycles = find_fundamental_cycles(g);

  return rep;
}

}  // namespace hacdcpf::graph

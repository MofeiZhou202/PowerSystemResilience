// Topology Analysis — connectivity checks + ONR via LinDistFlow MILP
// DistributionPowerFlow.jl TopologyAnalysis module equivalent (C++20)
//
// The ONR solver ("own B&C") builds a LinDistFlow-based MILP with
// spanning-tree connectivity constraints and calls solver::solve_milp_bc()
// directly with MIR cutting planes.
//
// MILP variable layout (m = #branches, n = #buses):
//   [0,        m)        alpha[b]  ∈ {0,1}  — branch status (1=closed)
//   [m,       2m)        P[b]      ∈ ℝ      — active power flow [pu]
//   [2m,      3m)        Q[b]      ∈ ℝ      — reactive power flow [pu]
//   [3m,      3m+n)      v[i]      ∈ ℝ      — squared voltage [pu²]
//   [3m+n,    4m+n)      f[b]      ∈ ℝ      — signed commodity flow
//   [4m+n,    5m+n)      t[b]      ≥ 0      — |P[b]| auxiliary (obj)

#include "hacdcpf/network_reconfiguration/topology_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"
#include "hacdcpf/solver/problem_types.hpp"

namespace hacdcpf::analysis {

namespace {

// -------------------------------------------------------------------------
// Bus-ID → local index
// -------------------------------------------------------------------------
std::unordered_map<int, int>
build_id_map(const std::vector<ACBus>& buses) {
  std::unordered_map<int, int> m;
  m.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i)
    m[buses[i].index] = i;
  return m;
}

// -------------------------------------------------------------------------
// Generic DFS connectivity (used by is_connected / is_radial)
// -------------------------------------------------------------------------
int dfs_component_count(const std::vector<ACBus>&   buses,
                        const std::vector<ACBranch>& branches,
                        const std::unordered_map<int, int>& id_map) {
  const int n = static_cast<int>(buses.size());
  if (n == 0) return 0;

  std::vector<std::vector<int>> adj(n);
  for (const auto& br : branches) {
    if (!br.in_service) continue;
    auto it_f = id_map.find(br.from_bus);
    auto it_t = id_map.find(br.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    adj[it_f->second].push_back(it_t->second);
    adj[it_t->second].push_back(it_f->second);
  }

  int components = 0;
  std::vector<char> visited(n, 0);
  std::vector<int> stk;
  for (int start = 0; start < n; ++start) {
    if (visited[start]) continue;
    ++components;
    stk.clear();
    stk.push_back(start);
    while (!stk.empty()) {
      int u = stk.back(); stk.pop_back();
      if (visited[u]) continue;
      visited[u] = 1;
      for (int v : adj[u])
        if (!visited[v]) stk.push_back(v);
    }
  }
  return components;
}

// -------------------------------------------------------------------------
// Count in-service branches
// -------------------------------------------------------------------------
int count_in_service_branches(const std::vector<ACBranch>& branches) {
  return static_cast<int>(
      std::count_if(branches.begin(), branches.end(),
                    [](const ACBranch& b) { return b.in_service; }));
}

}  // anonymous namespace

// =========================================================================
// Connectivity / radiality queries
// =========================================================================
bool is_connected(const ACSystem& ac_sys) {
  if (ac_sys.buses.empty()) return true;
  const auto id_map = build_id_map(ac_sys.buses);
  return dfs_component_count(ac_sys.buses, ac_sys.branches, id_map) == 1;
}

bool is_radial(const ACSystem& ac_sys) {
  const int n = static_cast<int>(ac_sys.buses.size());
  if (n == 0) return true;
  const int e = count_in_service_branches(ac_sys.branches);
  if (e != n - 1) return false;         // must have exactly n-1 edges
  return is_connected(ac_sys);
}

int count_islands(const ACSystem& ac_sys) {
  if (ac_sys.buses.empty()) return 0;
  const auto id_map = build_id_map(ac_sys.buses);
  return dfs_component_count(ac_sys.buses, ac_sys.branches, id_map);
}

// =========================================================================
// ONRResult::summary
// =========================================================================
std::string ONRResult::summary() const {
  std::ostringstream ss;
  ss << "ONR Result: " << (optimal ? "OPTIMAL" : (feasible ? "FEASIBLE" : "INFEASIBLE"))
     << "\n  MILP objective (lin. loss proxy) = " << milp_objective
     << "\n  Estimated loss = " << estimated_loss_mw << " MW"
     << "\n  Open branches:  ";
  for (int b : open_branch_ids) ss << b << " ";
  ss << "\n  Closed branches: ";
  for (int b : closed_branch_ids) ss << b << " ";
  ss << "\n  B&C nodes explored = " << bc_stats.nodes_explored
     << "  cuts added = " << bc_stats.cuts_added
     << "  gap = " << bc_stats.gap
     << "  runtime = " << bc_stats.runtime_sec << " s\n";
  return ss.str();
}

// =========================================================================
// solve_optimal_reconfiguration — own B&C entry point
// =========================================================================
ONRResult solve_optimal_reconfiguration(const ACSystem& ac_sys,
                                        const ONROptions& opt) {
  using namespace solver;

  const auto& buses    = ac_sys.buses;
  const auto& branches = ac_sys.branches;
  const double base_mva = (ac_sys.base_mva > 0.0) ? ac_sys.base_mva : 100.0;

  const int n = static_cast<int>(buses.size());
  const int m = static_cast<int>(branches.size());

  ONRResult result;
  if (n == 0 || m == 0) return result;

  // ── Graph-based topology pre-analysis ─────────────────────────────────
  // Identify bridge edges so their alpha lower bound is fixed to 1 in the
  // MILP (opening a bridge disconnects the network → always infeasible).
  // Also pre-validates topology (missing slack, isolated loads) and logs
  // the loop/bridge counts when verbose mode is on.
  //
  // IMPORTANT: build the "potential" graph with ALL branches treated as
  // in-service (including normally-open tie switches), so that fundamental
  // cycles formed by switchable branches are visible. A branch is only a
  // true structural bridge (and thus must stay closed) if it remains a
  // bridge even when all switchable branches are available.
  std::vector<bool> is_bridge(m, false);
  {
    HybridPowerSystem wrap;
    wrap.ac = ac_sys;
    // Temporarily mark all branches as in-service so the graph includes the
    // full potential topology (tie switches included).
    for (auto& br : wrap.ac.branches) br.in_service = true;
    auto g    = hacdcpf::graph::build_power_system_graph(wrap);
    auto topo = hacdcpf::graph::analyze_topology(g);

    if (!topo.all_islands_valid) {
      for (const auto& d : topo.diagnostics)
        spdlog::warn("ONR: topology pre-check: {}", d.message);
    }

    // build_power_system_graph inserts AC branches first, so
    // g.edges[eid] corresponds to ac_sys.branches[eid] for eid < m.
    for (int eid : topo.bridge_edge_ids) {
      if (eid >= 0 && eid < m) is_bridge[eid] = true;
    }

    if (opt.verbose) {
      int nb = static_cast<int>(std::count(is_bridge.begin(), is_bridge.end(), true));
      spdlog::info("ONR: {} buses, {} branches, {} bridge(s), {} fundamental cycle(s)",
                   n, m, nb, topo.fundamental_cycles.size());
    }
  }

  // --- Determine candidate (switchable) branch set -----------------------
  // If ONROptions::switchable_branch_ids is empty ⇒ all branches are candidates.
  std::vector<int> cand;   // local indices into branches[]
  if (opt.switchable_branch_ids.empty()) {
    cand.resize(m);
    std::iota(cand.begin(), cand.end(), 0);
  } else {
    cand = opt.switchable_branch_ids;
    for (int b : cand)
      if (b < 0 || b >= m)
        throw std::out_of_range("solve_optimal_reconfiguration: branch index out of range");
  }
  // For ONR the MILP uses exactly the candidate branches.
  // Non-candidate branches keep their current in_service status.
  // For simplicity in this implementation all branches are candidates.
  // (Non-switchable branches have alpha fixed to 1 if in_service else 0.)

  // --- Build bus-ID map ---------------------------------------------------
  const auto id_map = build_id_map(buses);

  // --- Find root (slack) bus ---------------------------------------------
  int root = -1;
  for (int i = 0; i < n; ++i)
    if (buses[i].bus_type == BusType::SLACK) { root = i; break; }
  if (root < 0) {
    // Fall back to first bus
    root = 0;
  }

  // --- Variable index helpers --------------------------------------------
  //   idx_alpha = b              (0..m-1)
  //   idx_P     = m + b          (m..2m-1)
  //   idx_Q     = 2m + b         (2m..3m-1)
  //   idx_v     = 3m + i         (3m..3m+n-1)
  //   idx_f     = 3m+n + b       (3m+n..4m+n-1)
  //   idx_t     = 4m+n + b       (4m+n..5m+n-1)
  const int n_vars = 5 * m + n;

  auto idx_alpha = [&](int b) { return b; };
  auto idx_P     = [&](int b) { return m + b; };
  auto idx_Q     = [&](int b) { return 2 * m + b; };
  auto idx_v     = [&](int i) { return 3 * m + i; };
  auto idx_f     = [&](int b) { return 3 * m + n + b; };
  auto idx_t     = [&](int b) { return 4 * m + n + b; };

  // --- Net injection at each bus (pu) ------------------------------------
  // Net injection = demand - generation. Positive P_net means load dominates.
  std::vector<double> P_net(n, 0.0), Q_net(n, 0.0);

  // Load demand: use Load table if available, else bus-level pd/qd.
  if (!ac_sys.loads.empty()) {
    for (const auto& ld : ac_sys.loads) {
      if (!ld.in_service) continue;
      auto it = id_map.find(ld.bus);
      if (it == id_map.end()) continue;
      int idx = it->second;
      if (idx == root) continue;
      P_net[idx] += ld.p_mw / base_mva;
      Q_net[idx] += ld.q_mvar / base_mva;
    }
  } else {
    for (int i = 0; i < n; ++i) {
      if (i == root) continue;
      P_net[i] = buses[i].pd_mw / base_mva;
      Q_net[i] = buses[i].qd_mvar / base_mva;
    }
  }
  for (const auto& g : ac_sys.generators) {
    if (!g.in_service) continue;
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] -= g.pg_mw   / base_mva;
    Q_net[gi] -= g.qg_mvar / base_mva;
  }
  // Static generators (distributed generation)
  for (const auto& sg : ac_sys.static_generators) {
    if (!sg.in_service) continue;
    auto it = id_map.find(sg.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] -= sg.p_mw * sg.scaling / base_mva;
    Q_net[gi] -= sg.q_mvar * sg.scaling / base_mva;
  }
  // Renewable generators
  for (const auto& rg : ac_sys.renewable_gens) {
    if (!rg.in_service) continue;
    auto it = id_map.find(rg.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] -= rg.p_mw / base_mva;
    Q_net[gi] -= rg.q_mvar / base_mva;
  }
  // PV systems
  for (const auto& pv : ac_sys.pv_systems) {
    if (!pv.in_service) continue;
    auto it = id_map.find(pv.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] -= pv.p_mw / base_mva;
    Q_net[gi] -= pv.q_mvar / base_mva;
  }
  // Storage (positive p_mw = discharge = generation)
  for (const auto& st : ac_sys.storage) {
    if (!st.in_service) continue;
    auto it = id_map.find(st.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] -= st.p_mw / base_mva;
    Q_net[gi] -= st.q_mvar / base_mva;
  }
  // ChargingStations (act as loads, kW→MW)
  for (const auto& cs : ac_sys.charging_stations) {
    if (!cs.in_service) continue;
    auto it = id_map.find(cs.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    if (gi == root) continue;
    P_net[gi] += (cs.p_total_kw / 1000.0) / base_mva;
    Q_net[gi] += (cs.q_total_kvar / 1000.0) / base_mva;
  }
  // FlexibleLoads (their base demand is in bus pd_mw; net reduction not modeled in ONR)
  // — no separate ONR variable, just the base demand already included above.
  // Note: VPPs, Microgrids, and MobileStorage live on HybridPowerSystem (not ACSystem)
  // and are not accessible from this function signature.

  if (opt.verbose) {
    double total_p_net = 0.0;
    for (int i = 0; i < n; ++i) total_p_net += P_net[i];
    spdlog::info("ONR: n={} m={} root={} base_mva={:.1f} total_P_net={:.6f} pu ({:.4f} MW)",
                 n, m, root, base_mva, total_p_net, total_p_net * base_mva);
  }

  // --- MILP model construction -------------------------------------------
  MIPModel milp;
  auto& lp = milp.linear_part;
  lp.sense = Sense::Minimize;

  // Objective: min Σ r[b] * t[b]
  lp.c = Eigen::VectorXd::Zero(n_vars);
  for (int b = 0; b < m; ++b)
    lp.c[idx_t(b)] = branches[b].r_pu;

  // --- Variable metadata (bounds and types) ------------------------------
  lp.vars.resize(n_vars);

  const double Pmax_default = opt.default_rate_mva / base_mva;
  const double f_max = static_cast<double>(n - 1);
  const double vmin2 = opt.v_min_pu * opt.v_min_pu;
  const double vmax2 = opt.v_max_pu * opt.v_max_pu;

  for (int b = 0; b < m; ++b) {
    // alpha  (bridge edges must stay closed: lb = 1)
    lp.vars[idx_alpha(b)] = {VarType::Binary, is_bridge[b] ? 1.0 : 0.0, 1.0, "a" + std::to_string(b)};
    // P (active power flow)
    double pmax = (branches[b].rate_a_mva > 1e-9)
                  ? branches[b].rate_a_mva / base_mva : Pmax_default;
    lp.vars[idx_P(b)] = {VarType::Continuous, -pmax, pmax, "P" + std::to_string(b)};
    // Q (reactive power flow)
    lp.vars[idx_Q(b)] = {VarType::Continuous, -pmax, pmax, "Q" + std::to_string(b)};
    // f (signed commodity flow)
    lp.vars[idx_f(b)] = {VarType::Continuous, -f_max, f_max, "f" + std::to_string(b)};
    // t (|P| auxiliary)
    lp.vars[idx_t(b)] = {VarType::Continuous, 0.0, pmax, "t" + std::to_string(b)};
  }
  for (int i = 0; i < n; ++i) {
    double lb = (i == root) ? 1.0 : vmin2;  // root: fixed at 1.0 pu²
    double ub = (i == root) ? 1.0 : vmax2;
    lp.vars[idx_v(i)] = {VarType::Continuous, lb, ub, "v" + std::to_string(i)};
  }
  milp.binary_idx.resize(m);
  std::iota(milp.binary_idx.begin(), milp.binary_idx.end(), 0);

  // -----------------------------------------------------------------------
  // Equality constraints
  //
  // Row layout:
  //   0               : spanning-tree count  Σ alpha = n-1
  //   1..n-1          : active power balance  (non-root buses)
  //   n..2n-2         : reactive power balance (non-root buses)
  //   2n-1            : root voltage  v[root] = 1
  //   2n..3n-1        : commodity flow balance (all buses)
  //
  //   Total = 1 + (n-1) + (n-1) + 1 + n = 3n
  // -----------------------------------------------------------------------
  const int n_eq = 3 * n;
  lp.Aeq.resize(n_eq, n_vars);
  lp.beq = Eigen::VectorXd::Zero(n_eq);

  // We use a triplet accumulator to build Aeq
  std::vector<Eigen::Triplet<double>> eq_trips;
  eq_trips.reserve(6 * m + 4 * n);

  // Row 0: spanning-tree edge count  Σ alpha[b] = n - 1
  for (int b = 0; b < m; ++b)
    eq_trips.emplace_back(0, idx_alpha(b), 1.0);
  lp.beq[0] = static_cast<double>(n - 1);

  // Power balance rows (skip root = local index `root`)
  // Map non-root buses to row indices 1..(n-1) and n..(2n-2)
  // We iterate over all buses and skip root
  {
    // Build non-root bus → balance row index
    std::vector<int> bus_to_row_p(n, -1), bus_to_row_q(n, -1);
    int rp = 1, rq = n;
    for (int i = 0; i < n; ++i) {
      if (i == root) continue;
      bus_to_row_p[i] = rp++;
      bus_to_row_q[i] = rq++;
    }

    for (int b = 0; b < m; ++b) {
      auto it_f = id_map.find(branches[b].from_bus);
      auto it_t = id_map.find(branches[b].to_bus);
      if (it_f == id_map.end() || it_t == id_map.end()) continue;
      int fi = it_f->second, ti = it_t->second;

      // Active power:
      if (ti != root) {
        int rp_ti = bus_to_row_p[ti];
        eq_trips.emplace_back(rp_ti, idx_P(b),  1.0);  // P[b] in at ti
      }
      if (fi != root) {
        int rp_fi = bus_to_row_p[fi];
        eq_trips.emplace_back(rp_fi, idx_P(b), -1.0);  // P[b] out at fi
      }

      // Reactive power:
      if (ti != root) {
        int rq_ti = bus_to_row_q[ti];
        eq_trips.emplace_back(rq_ti, idx_Q(b),  1.0);
      }
      if (fi != root) {
        int rq_fi = bus_to_row_q[fi];
        eq_trips.emplace_back(rq_fi, idx_Q(b), -1.0);
      }
    }

    // RHS for power balance
    for (int i = 0; i < n; ++i) {
      if (i == root) continue;
      lp.beq[bus_to_row_p[i]] = P_net[i];
      lp.beq[bus_to_row_q[i]] = Q_net[i];
    }
  }

  // Row 2n-1: root voltage fixed at 1 p.u.²
  {
    int row_vr = 2 * n - 1;
    eq_trips.emplace_back(row_vr, idx_v(root), 1.0);
    lp.beq[row_vr] = 1.0;
  }

  // Rows 2n..(3n-1): commodity flow balance
  // Root: Σ_{b: from=root} f[b] - Σ_{b: to=root} f[b] = n-1  (outflow)
  // Other bus i: Σ_{b: to=i} f[b] - Σ_{b: from=i} f[b] = 1   (unit inflow)
  {
    // Build a row for every bus (root at offset 2n, bus i at offset 2n+i)
    // Note: we store root at 2n + root
    for (int b = 0; b < m; ++b) {
      auto it_f = id_map.find(branches[b].from_bus);
      auto it_t = id_map.find(branches[b].to_bus);
      if (it_f == id_map.end() || it_t == id_map.end()) continue;
      int fi = it_f->second, ti = it_t->second;

      // Commodity flow sign convention: positive f[b] = from fi to ti
      // For commodity balance at bus ti: +f[b]  (inflow)
      // For commodity balance at bus fi: -f[b]  (outflow)
      eq_trips.emplace_back(2 * n + ti, idx_f(b),  1.0);
      eq_trips.emplace_back(2 * n + fi, idx_f(b), -1.0);
    }

    // Root is the source: net outflow = n-1  → inflow - outflow = -(n-1)
    // Non-root buses are sinks: net inflow = 1 → inflow - outflow = 1
    // (Kirchhoff: Σ_i beq[i] = -(n-1) + (n-1)*1 = 0  ✓)
    for (int i = 0; i < n; ++i) {
      int row_cf = 2 * n + i;
      lp.beq[row_cf] = (i == root) ? -static_cast<double>(n - 1) : 1.0;
    }
  }

  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();

  // -----------------------------------------------------------------------
  // Inequality constraints:  A * x ≤ b
  //
  // Row layout (n_ineq = 10*m):
  //   [0,  m) : LinDistFlow upper:  v[to] - v[from] + 2r*P + 2x*Q - M*alpha ≤ M
  //   [m, 2m) : LinDistFlow lower: -v[to] + v[from] - 2r*P - 2x*Q - M*alpha ≤ M
  //   [2m,3m) : P thermal upper:   P[b] - Pmax*alpha[b] ≤ 0
  //   [3m,4m) : P thermal lower:  -P[b] - Pmax*alpha[b] ≤ 0
  //   [4m,5m) : Q thermal upper:   Q[b] - Qmax*alpha[b] ≤ 0
  //   [5m,6m) : Q thermal lower:  -Q[b] - Qmax*alpha[b] ≤ 0
  //   [6m,7m) : f capacity upper:  f[b] - (n-1)*alpha[b] ≤ 0
  //   [7m,8m) : f capacity lower: -f[b] - (n-1)*alpha[b] ≤ 0
  //   [8m,9m) : |P| aux upper:     P[b] - t[b] ≤ 0
  //   [9m,10m): |P| aux lower:    -P[b] - t[b] ≤ 0
  // -----------------------------------------------------------------------
  const int n_ineq = 10 * m;
  lp.A.resize(n_ineq, n_vars);
  lp.b = Eigen::VectorXd::Zero(n_ineq);

  std::vector<Eigen::Triplet<double>> ineq_trips;
  ineq_trips.reserve(8 * m);

  const double M = opt.big_m;

  for (int b = 0; b < m; ++b) {
    auto it_f = id_map.find(branches[b].from_bus);
    auto it_t = id_map.find(branches[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;

    double r = branches[b].r_pu, x = branches[b].x_pu;
    double pmax = (branches[b].rate_a_mva > 1e-9)
                  ? branches[b].rate_a_mva / base_mva : Pmax_default;

    // LinDistFlow big-M: when alpha=1 (closed), enforce voltage drop;
    // when alpha=0 (open), constraint is trivially loose.
    // Upper:  v[ti] - v[fi] + 2rP + 2xQ  ≤  M*(1-alpha)
    //     →   v[ti] - v[fi] + 2rP + 2xQ  + M*alpha  ≤  M
    {
      int row = b;
      ineq_trips.emplace_back(row, idx_v(ti),    1.0);
      ineq_trips.emplace_back(row, idx_v(fi),   -1.0);
      ineq_trips.emplace_back(row, idx_P(b),     2.0 * r);
      ineq_trips.emplace_back(row, idx_Q(b),     2.0 * x);
      ineq_trips.emplace_back(row, idx_alpha(b),  M);   // +M, not -M
      lp.b[row] = M;
    }
    // Lower: -v[ti] + v[fi] - 2rP - 2xQ  ≤  M*(1-alpha)
    //    →   -v[ti] + v[fi] - 2rP - 2xQ  + M*alpha  ≤  M
    {
      int row = m + b;
      ineq_trips.emplace_back(row, idx_v(ti),   -1.0);
      ineq_trips.emplace_back(row, idx_v(fi),    1.0);
      ineq_trips.emplace_back(row, idx_P(b),    -2.0 * r);
      ineq_trips.emplace_back(row, idx_Q(b),    -2.0 * x);
      ineq_trips.emplace_back(row, idx_alpha(b),  M);   // +M, not -M
      lp.b[row] = M;
    }
    // P thermal upper: P[b] - pmax*alpha ≤ 0
    {
      int row = 2 * m + b;
      ineq_trips.emplace_back(row, idx_P(b),      1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -pmax);
    }
    // P thermal lower: -P[b] - pmax*alpha ≤ 0
    {
      int row = 3 * m + b;
      ineq_trips.emplace_back(row, idx_P(b),     -1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -pmax);
    }
    // Q thermal upper
    {
      int row = 4 * m + b;
      ineq_trips.emplace_back(row, idx_Q(b),      1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -pmax);
    }
    // Q thermal lower
    {
      int row = 5 * m + b;
      ineq_trips.emplace_back(row, idx_Q(b),     -1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -pmax);
    }
    // f capacity upper:  f[b] - (n-1)*alpha ≤ 0
    {
      int row = 6 * m + b;
      ineq_trips.emplace_back(row, idx_f(b),      1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -f_max);
    }
    // f capacity lower: -f[b] - (n-1)*alpha ≤ 0
    {
      int row = 7 * m + b;
      ineq_trips.emplace_back(row, idx_f(b),     -1.0);
      ineq_trips.emplace_back(row, idx_alpha(b), -f_max);
    }
    // |P| aux upper: P[b] - t[b] ≤ 0
    {
      int row = 8 * m + b;
      ineq_trips.emplace_back(row, idx_P(b),  1.0);
      ineq_trips.emplace_back(row, idx_t(b), -1.0);
    }
    // |P| aux lower: -P[b] - t[b] ≤ 0
    {
      int row = 9 * m + b;
      ineq_trips.emplace_back(row, idx_P(b), -1.0);
      ineq_trips.emplace_back(row, idx_t(b), -1.0);
    }
  }

  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();

  // -----------------------------------------------------------------------
  // B&C solver options (domain-specific tuning for network topology)
  // -----------------------------------------------------------------------
  BCOptions bc_opt;
  bc_opt.time_limit_sec       = static_cast<double>(opt.max_time_s);
  bc_opt.gap_tol              = opt.mip_gap;
  bc_opt.cuts                 = CutType::MIR;   // MIR cuts suitable for mixed topology
  bc_opt.root_cut_rounds      = 5;
  bc_opt.cuts_per_round       = 20;
  bc_opt.branching            = BranchingStrategy::Pseudocost;
  bc_opt.node_sel             = NodeSelection::Hybrid;
  bc_opt.use_feasibility_pump = true;
  bc_opt.use_simplex_lp_nodes = true;
  bc_opt.verbose              = opt.verbose;

  // -----------------------------------------------------------------------
  // Call the native B&C solver
  // -----------------------------------------------------------------------
  BCResult bc_result = solve_milp_bc(milp, bc_opt);

  if (opt.verbose) {
    spdlog::info("ONR B&C: success={} nodes={} gap={:.4f} time={:.2f}s",
                 bc_result.stats.success, bc_result.bc_stats.nodes_explored,
                 bc_result.bc_stats.gap, bc_result.bc_stats.runtime_sec);
  }

  // -----------------------------------------------------------------------
  // Extract results
  // -----------------------------------------------------------------------
  result.bc_stats   = bc_result.bc_stats;
  result.feasible   = bc_result.stats.success;
  result.optimal    = bc_result.stats.success &&
                      (bc_result.bc_stats.gap < opt.mip_gap * 2.0);
  result.milp_objective = bc_result.stats.objective;

  if (!result.feasible || bc_result.x.size() < n_vars) {
    return result;
  }

  const Eigen::VectorXd& x_sol = bc_result.x;

  // Post-solve constraint verification — mirrors topology_reconfiguration.cpp.
  // Catches spurious "success" returns from the B&C solver.
  if (lp.Aeq.rows() > 0) {
    Eigen::VectorXd res_eq = lp.Aeq * x_sol - lp.beq;
    double eq_viol = res_eq.cwiseAbs().maxCoeff();
    if (eq_viol > 1e-6) {
      spdlog::warn("[ONR 旧路径] 后验等式约束违约: eq_viol={:.2e}", eq_viol);
      result.feasible = false;
      return result;
    }
  }
  if (lp.A.rows() > 0) {
    Eigen::VectorXd res_ineq = lp.A * x_sol - lp.b;
    double ineq_viol = res_ineq.cwiseMax(0.0).maxCoeff();
    if (ineq_viol > 1e-6) {
      spdlog::warn("[ONR 旧路径] 后验不等式约束违约: ineq_viol={:.2e}", ineq_viol);
      result.feasible = false;
      return result;
    }
  }
  // Binary integrality check on alpha variables.
  // Use 1e-4 (not 0.1) so that marginally fractional B&C solutions are
  // rejected before the Kruskal rounding step produces an unverified topology.
  for (int b = 0; b < m; ++b) {
    double v    = x_sol[idx_alpha(b)];
    double frac = std::min(v - std::floor(v), std::ceil(v) - v);
    if (frac > 1e-4) {
      spdlog::warn("[ONR 旧路径] 后验整数性违约: alpha[{}]={:.6f} (frac={:.2e})", b, v, frac);
      result.feasible = false;
      return result;
    }
  }

  // -------------------------------------------------------------------
  // Round alpha to binary AND guarantee a valid spanning tree
  // (connected, exactly n-1 closed branches, no islands).
  //
  // Strategy: Kruskal-style greedy with Union-Find.
  //   1. Sort branches by MILP alpha value descending (prefer alpha≈1).
  //   2. Greedily add branches that merge two distinct components.
  //   3. Stop after exactly n-1 edges ⇒ guaranteed spanning tree.
  // -------------------------------------------------------------------
  std::vector<double> alpha_raw(m);
  for (int b = 0; b < m; ++b)
    alpha_raw[b] = x_sol[idx_alpha(b)];

  // Union-Find data structure
  std::vector<int> uf_parent(n), uf_rank(n, 0);
  std::iota(uf_parent.begin(), uf_parent.end(), 0);
  std::function<int(int)> uf_find = [&](int x) -> int {
    return uf_parent[x] == x ? x : (uf_parent[x] = uf_find(uf_parent[x]));
  };
  auto uf_union = [&](int a, int b) -> bool {
    int ra = uf_find(a), rb = uf_find(b);
    if (ra == rb) return false;  // same component — would create cycle
    if (uf_rank[ra] < uf_rank[rb]) std::swap(ra, rb);
    uf_parent[rb] = ra;
    if (uf_rank[ra] == uf_rank[rb]) ++uf_rank[ra];
    return true;
  };

  // Sort branches by alpha value (descending) — prefer MILP-selected edges
  std::vector<int> order(m);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b) { return alpha_raw[a] > alpha_raw[b]; });

  std::vector<double> alpha(m, 0.0);
  const int target = n - 1;
  int n_tree = 0;

  for (int b_idx : order) {
    if (n_tree >= target) break;
    auto it_f = id_map.find(branches[b_idx].from_bus);
    auto it_t = id_map.find(branches[b_idx].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;
    if (uf_union(fi, ti)) {
      alpha[b_idx] = 1.0;
      ++n_tree;
    }
  }

  if (n_tree < target) {
    spdlog::warn("ONR: spanning tree incomplete ({}/{} edges). "
                 "Graph may be disconnected.", n_tree, target);
  }

  if (opt.verbose) {
    spdlog::info("ONR rounding: raw closed={}, tree edges={}/{}",
                 static_cast<int>(std::count_if(alpha_raw.begin(), alpha_raw.end(),
                                                [](double v){ return v > 0.5; })),
                 n_tree, target);
  }

  // Identify open / closed branches — emit stable ACBranch::index values
  for (int b = 0; b < m; ++b) {
    if (alpha[b] > 0.5)
      result.closed_branch_ids.push_back(branches[b].index);
    else
      result.open_branch_ids.push_back(branches[b].index);
  }

  // Estimated loss (MW) — only count closed branches
  double obj_pu = 0.0;
  for (int b = 0; b < m; ++b)
    if (alpha[b] > 0.5)
      obj_pu += branches[b].r_pu * std::abs(x_sol[idx_P(b)]);
  result.estimated_loss_mw = obj_pu * base_mva;

  // -----------------------------------------------------------------------
  // Verification: apply the optimal switching and run Newton power flow.
  // -----------------------------------------------------------------------
  ACSystem ac_opt = ac_sys;
  for (int b = 0; b < m; ++b)
    ac_opt.branches[b].in_service = (alpha[b] > 0.5);

  HybridPowerSystem sys_opt;
  sys_opt.base_mva = base_mva;
  sys_opt.ac = ac_opt;

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 200;
  pf_opt.tol      = 1e-6;
  result.verification_pf = solve_power_flow(sys_opt, pf_opt);

  return result;
}

ONRResult solve_optimal_reconfiguration(const HybridPowerSystem& sys,
                                        const ONROptions& opt) {
  // Project through canonical layer so Transformer2W/3W, Switches,
  // FlexibleLoads, AsymmetricLoads, and Chargers are expanded.
  const HybridPowerSystem projected = project_to_canonical_models(sys);
  return solve_optimal_reconfiguration(projected.ac, opt);
}

}  // namespace hacdcpf::analysis

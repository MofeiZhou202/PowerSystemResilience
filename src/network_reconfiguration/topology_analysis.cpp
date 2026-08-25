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
#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"
#include "hacdcpf/projection/result_attribution.hpp"
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

  // Compatibility wrapper: route the historical AC-only ONR API through the
  // maintained hybrid-aware reconfiguration solver, then project the result
  // back into the legacy ONRResult shape.  The old inline MILP below is kept
  // as reference documentation, but the newer solver has the richer source,
  // switch, load-shed, and solver-backend handling used by the rest of the
  // platform.
  {
    HybridPowerSystem wrap;
    wrap.base_mva = base_mva;
    wrap.ac = ac_sys;

    TopoReconfOptions reconf_opt;
    reconf_opt.v_min_pu = opt.v_min_pu;
    reconf_opt.v_max_pu = opt.v_max_pu;
    reconf_opt.default_rate_mva = opt.default_rate_mva;
    reconf_opt.big_m_v = opt.big_m;
    reconf_opt.max_time_s = opt.max_time_s;
    reconf_opt.mip_gap = opt.mip_gap;
    reconf_opt.verbose = opt.verbose;
    reconf_opt.enable_pf = true;
    reconf_opt.enable_voltage = true;
    reconf_opt.enable_thermal = true;
    reconf_opt.loss_aware = true;
    reconf_opt.lambda_switch = 0.0;
    reconf_opt.lambda_loss = 1.0;
    reconf_opt.lambda_shed = 1e4;
    reconf_opt.lambda_island = 1e5;
    reconf_opt.solver = "highs";

    if (opt.switchable_branch_ids.empty()) {
      reconf_opt.switchable_branch_ids.reserve(branches.size());
      for (const auto& br : branches) reconf_opt.switchable_branch_ids.push_back(br.index);
    } else {
      reconf_opt.switchable_branch_ids = opt.switchable_branch_ids;
    }

    const TopoReconfResult topo = run_topology_reconfiguration(wrap, reconf_opt);

    result.feasible = topo.feasible;
    result.optimal = topo.optimal || topo.proven_optimal;
    result.milp_objective = topo.milp_objective;
    result.estimated_loss_mw = topo.reconf_loss_mw;
    result.bc_stats = topo.bc_stats;

    for (const auto& ref : topo.open_branches) {
      if (ref.category == hacdcpf::graph::EdgeCategory::AC_Line)
        result.open_branch_ids.push_back(ref.index);
    }
    for (const auto& ref : topo.closed_branches) {
      if (ref.category == hacdcpf::graph::EdgeCategory::AC_Line)
        result.closed_branch_ids.push_back(ref.index);
    }

    if (result.feasible) {
      std::unordered_map<int, bool> closed_by_id;
      closed_by_id.reserve(result.closed_branch_ids.size());
      for (int id : result.closed_branch_ids) closed_by_id[id] = true;

      ACSystem ac_opt = ac_sys;
      for (auto& br : ac_opt.branches)
        br.in_service = closed_by_id.find(br.index) != closed_by_id.end();

      HybridPowerSystem sys_opt;
      sys_opt.base_mva = base_mva;
      sys_opt.ac = ac_opt;

      PowerFlowOptions pf_opt;
      pf_opt.max_iter = 200;
      pf_opt.tol = 1e-6;
      result.verification_pf = solve_power_flow(sys_opt, pf_opt);
    }

    if (!result.feasible && is_connected(ac_sys)) {
      HybridPowerSystem sys_base;
      sys_base.base_mva = base_mva;
      sys_base.ac = ac_sys;

      PowerFlowOptions pf_opt;
      pf_opt.max_iter = 200;
      pf_opt.tol = 1e-6;
      PowerFlowResult pf = solve_power_flow(sys_base, pf_opt);
      if (pf.converged) {
        result.feasible = true;
        result.optimal = false;
        // The core MILP failed; this reports the unchanged base topology as a
        // connectivity fallback, not an ONR incumbent. Mark it so callers can
        // tell it apart from a solved reconfiguration.
        result.fallback_used = true;
        result.verification_pf = pf;
        result.open_branch_ids.clear();
        result.closed_branch_ids.clear();
        result.estimated_loss_mw = 0.0;
        for (const auto& br : branches) {
          if (br.in_service) {
            result.closed_branch_ids.push_back(br.index);
            result.estimated_loss_mw += br.r_pu * base_mva;
          } else {
            result.open_branch_ids.push_back(br.index);
          }
        }
        result.milp_objective = 0.0;
      }
    }

    return result;
  }
}

ONRResult solve_optimal_reconfiguration(const HybridPowerSystem& sys,
                                        const ONROptions& opt) {
  // Project through canonical layer so Transformer2W/3W, Switches,
  // FlexibleLoads, AsymmetricLoads, and Chargers are expanded.
  const HybridPowerSystem projected =
      projection::RichToCanonicalOperator::apply(sys).canonical;
  return solve_optimal_reconfiguration(projected.ac, opt);
}

}  // namespace hacdcpf::analysis

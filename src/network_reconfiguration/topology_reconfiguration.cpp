// =====================================================================
// topology_reconfiguration.cpp — 配电网故障后拓扑重构
//
// 当发生线路故障 (ξ=1) 时，断开故障线路并在满足辐射状约束的前提下
// 闭合联络开关，形成新的供电路径。
//
// 对应 Julia DistributionPowerFlow-runze/TopologyAnalysis 实现。
//
// MILP 变量布局 (0-based C++ indexing):
//   Fij      [0,            nl)          虚拟潮流 (AC+DC 支路)
//   Fij_vsc  [nl,           nl+nl_vsc)   虚拟潮流 (VSC)
//   Fg       [nl+nl_vsc,    nl+nl_vsc+ng)发电机虚拟注入
//   β        [off_beta, ...]             开关状态 {0,1}
//   γ        [off_gamma, ...]            根节点指示 {0,1}
//  --- 以下仅 enable_pf=true 时存在 ---
//   Pij, Pij_vsc, Qij, Qij_vsc, Pg, Qg, v, sP, sQ
// =====================================================================

#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"
#include "hacdcpf/solver/problem_types.hpp"

namespace hacdcpf::analysis {

// =====================================================================
// summary()
// =====================================================================
std::string TopoReconfResult::summary() const {
  std::ostringstream ss;
  ss << "TopoReconfResult { "
     << (optimal ? "OPTIMAL" : (feasible ? "FEASIBLE" : "INFEASIBLE"))
     << ", obj=" << milp_objective
     << ", switched_on=" << n_switch_on
     << ", switched_off=" << n_switch_off
     << ", reconf_loss=" << reconf_loss_mw << " MW"
     << ", solve_time=" << solve_time_s << "s"
     << ", open=[";
  for (size_t i = 0; i < open_branch_ids.size(); ++i) {
    if (i) ss << ",";
    ss << open_branch_ids[i];
  }
  ss << "], closed=[";
  for (size_t i = 0; i < closed_branch_ids.size(); ++i) {
    if (i) ss << ",";
    ss << closed_branch_ids[i];
  }
  ss << "] }";
  return ss.str();
}

// =====================================================================
// Variable index helper
// =====================================================================
namespace {

struct VarLayout {
  int nl, nl_vsc, ng, nb, nb_ac, nl_ac;

  int off_Fij, off_Fij_vsc, off_Fg, off_beta, off_gamma;
  int off_Pij, off_Pij_vsc, off_Qij, off_Qij_vsc;
  int off_Pg, off_Qg, off_v, off_sP, off_sQ;

  int n_vars;
  bool has_pf;

  VarLayout(int nl_, int nl_vsc_, int ng_, int nb_, int nb_ac_, int nl_ac_,
            bool enable_pf)
      : nl(nl_), nl_vsc(nl_vsc_), ng(ng_), nb(nb_), nb_ac(nb_ac_),
        nl_ac(nl_ac_), has_pf(enable_pf) {
    off_Fij     = 0;
    off_Fij_vsc = off_Fij + nl;
    off_Fg      = off_Fij_vsc + nl_vsc;
    off_beta    = off_Fg + ng;
    off_gamma   = off_beta + nl + nl_vsc;

    if (enable_pf) {
      off_Pij     = off_gamma + ng;
      off_Pij_vsc = off_Pij + nl;
      off_Qij     = off_Pij_vsc + nl_vsc;
      off_Qij_vsc = off_Qij + nl_ac;
      off_Pg      = off_Qij_vsc + nl_vsc;
      off_Qg      = off_Pg + ng;
      off_v       = off_Qg + ng;
      off_sP      = off_v + nb;
      off_sQ      = off_sP + nb;
      n_vars      = off_sQ + nb_ac;
    } else {
      off_Pij = off_Pij_vsc = off_Qij = off_Qij_vsc = 0;
      off_Pg = off_Qg = off_v = off_sP = off_sQ = 0;
      n_vars = off_gamma + ng;
    }
  }

  int Fij(int b)     const { return off_Fij + b; }
  int Fij_vsc(int b) const { return off_Fij_vsc + b; }
  int Fg(int g)      const { return off_Fg + g; }
  int beta(int b)    const { return off_beta + b; }
  int gamma(int g)   const { return off_gamma + g; }
  int Pij(int b)     const { return off_Pij + b; }
  int Pij_vsc(int b) const { return off_Pij_vsc + b; }
  int Qij(int b)     const { return off_Qij + b; }
  int Qij_vsc(int b) const { return off_Qij_vsc + b; }
  int Pg(int g)      const { return off_Pg + g; }
  int Qg(int g)      const { return off_Qg + g; }
  int v(int i)       const { return off_v + i; }
  int sP(int i)      const { return off_sP + i; }
  int sQ(int i)      const { return off_sQ + i; }
};

}  // anonymous namespace

// =====================================================================
// run_topology_reconfiguration — main implementation
// =====================================================================
TopoReconfResult run_topology_reconfiguration(
    const HybridPowerSystem& sys,
    const TopoReconfOptions& opt) {

  using namespace solver;
  namespace chr = std::chrono;
  const auto t_start = chr::steady_clock::now();

  // Project system to canonical form
  const HybridPowerSystem proj = project_to_canonical_models(sys);
  const auto& ac = proj.ac;
  const auto& dc = proj.dc;
  const auto& vscs = proj.vsc_converters;
  const double base_mva = (proj.base_mva > 0) ? proj.base_mva : 100.0;

  const int nb_ac  = static_cast<int>(ac.buses.size());
  const int nb_dc  = static_cast<int>(dc.buses.size());
  const int nb     = nb_ac + nb_dc;
  const int nl_ac  = static_cast<int>(ac.branches.size());
  const int nl_dc  = static_cast<int>(dc.branches.size());
  const int nl     = nl_ac + nl_dc;
  const int nl_vsc = static_cast<int>(vscs.size());
  const int ng     = static_cast<int>(ac.generators.size());

  TopoReconfResult result;
  if (nb == 0 || nl == 0) {
    spdlog::warn("[拓扑重构] 系统为空 (nb={}, nl={})", nb, nl);
    return result;
  }

  spdlog::info("[拓扑重构] 系统: {} 母线(AC={},DC={}), {} 支路(AC={},DC={}), "
               "{} VSC, {} 发电机",
               nb, nb_ac, nb_dc, nl, nl_ac, nl_dc, nl_vsc, ng);

  // -------------------------------------------------------------------
  // Bus ID → local index (AC 0..nb_ac-1, DC nb_ac..nb-1)
  // AC and DC buses may share the same index values (e.g. both have bus 1),
  // so we MUST use separate maps to avoid collisions.
  // -------------------------------------------------------------------
  std::unordered_map<int, int> ac_id_map, dc_id_map;
  ac_id_map.reserve(nb_ac);
  dc_id_map.reserve(nb_dc);
  for (int i = 0; i < nb_ac; ++i)
    ac_id_map[ac.buses[i].index] = i;
  for (int i = 0; i < nb_dc; ++i)
    dc_id_map[dc.buses[i].index] = nb_ac + i;

  // -------------------------------------------------------------------
  // Unified edge arrays: AC branches [0,nl_ac) + DC [nl_ac,nl) + VSC [nl,nl+nl_vsc)
  // -------------------------------------------------------------------
  std::vector<int> edge_from(nl + nl_vsc, -1), edge_to(nl + nl_vsc, -1);
  std::vector<double> edge_r(nl + nl_vsc, 0.0), edge_x(nl + nl_vsc, 0.0);
  std::vector<double> edge_rate(nl + nl_vsc, 0.0);
  std::vector<bool> edge_status(nl + nl_vsc, true);  // α (initial in_service)
  std::vector<int> edge_orig_idx(nl + nl_vsc, -1);

  const double default_rate_pu = opt.default_rate_mva / base_mva;

  for (int i = 0; i < nl_ac; ++i) {
    const auto& br = ac.branches[i];
    auto it_f = ac_id_map.find(br.from_bus);
    auto it_t = ac_id_map.find(br.to_bus);
    if (it_f == ac_id_map.end() || it_t == ac_id_map.end()) continue;
    edge_from[i]     = it_f->second;
    edge_to[i]       = it_t->second;
    edge_r[i]        = br.r_pu;
    edge_x[i]        = br.x_pu;
    edge_rate[i]     = (br.rate_a_mva > 1e-9) ? br.rate_a_mva / base_mva : default_rate_pu;
    edge_status[i]   = br.in_service;
    edge_orig_idx[i] = br.index;
  }
  for (int i = 0; i < nl_dc; ++i) {
    const auto& br = dc.branches[i];
    auto it_f = dc_id_map.find(br.from_bus);
    auto it_t = dc_id_map.find(br.to_bus);
    if (it_f == dc_id_map.end() || it_t == dc_id_map.end()) continue;
    int e = nl_ac + i;
    edge_from[e]     = it_f->second;
    edge_to[e]       = it_t->second;
    edge_r[e]        = br.r_pu;
    edge_x[e]        = 0.0;
    edge_rate[e]     = (br.rate_a_mva > 1e-9) ? br.rate_a_mva / base_mva : default_rate_pu;
    edge_status[e]   = br.in_service;
    edge_orig_idx[e] = br.index;
  }
  for (int i = 0; i < nl_vsc; ++i) {
    const auto& vsc = vscs[i];
    auto it_ac = ac_id_map.find(vsc.bus_ac);
    auto it_dc = dc_id_map.find(vsc.bus_dc);
    if (it_ac == ac_id_map.end() || it_dc == dc_id_map.end()) continue;
    int e = nl + i;
    edge_from[e]     = it_ac->second;
    edge_to[e]       = it_dc->second;
    edge_r[e]        = 0.0;
    edge_x[e]        = 0.0;
    edge_rate[e]     = (vsc.pmax_mw > 1e-9) ? vsc.pmax_mw / base_mva : default_rate_pu;
    edge_status[e]   = vsc.in_service;
    edge_orig_idx[e] = vsc.index;
  }

  // -------------------------------------------------------------------
  // Fault status ζ: 1=available, 0=faulted
  // -------------------------------------------------------------------
  std::vector<double> zeta(nl + nl_vsc, 1.0);
  for (int f : opt.line_failures) {
    // line_failures contains ACBranch::index values (the .index field stored on each
    // branch), NOT 0-based array positions.  edge_orig_idx[i] == br.index, so the
    // match is correct.  Search only AC branch edges [0, nl_ac) to avoid spurious
    // matches against DC branches or VSC converters that may share the same index value.
    for (int i = 0; i < nl_ac; ++i) {
      if (edge_orig_idx[i] == f) { zeta[i] = 0.0; break; }
    }
  }

  // -------------------------------------------------------------------
  // Root bus (first active generator / slack bus)
  // -------------------------------------------------------------------
  int root_bus = -1;
  for (const auto& g : ac.generators) {
    if (!g.in_service) continue;
    auto it = ac_id_map.find(g.bus);
    if (it != ac_id_map.end()) { root_bus = it->second; break; }
  }
  if (root_bus < 0) {
    for (int i = 0; i < nb_ac; ++i)
      if (ac.buses[i].bus_type == BusType::SLACK) { root_bus = i; break; }
  }
  if (root_bus < 0) root_bus = 0;

  std::vector<int> gen_bus(ng);
  for (int g = 0; g < ng; ++g) {
    auto it = ac_id_map.find(ac.generators[g].bus);
    gen_bus[g] = (it != ac_id_map.end()) ? it->second : -1;
  }

  // -------------------------------------------------------------------
  // Variable layout
  // -------------------------------------------------------------------
  VarLayout idx(nl, nl_vsc, ng, nb, nb_ac, nl_ac, opt.enable_pf);
  spdlog::info("[拓扑重构] 变量数: {} (enable_pf={})", idx.n_vars, opt.enable_pf);

  // -------------------------------------------------------------------
  // Safety bound propagation (Julia T5):
  //   α=1,ζ=1 → β=1 | ζ=0 → β=0,Fij=0 | α=0,ζ=1 → β∈[0,1]
  // -------------------------------------------------------------------
  std::vector<double> lb(idx.n_vars, -1e20), ub(idx.n_vars, 1e20);

  for (int i = 0; i < nl; ++i) {
    lb[idx.Fij(i)] = -nb; ub[idx.Fij(i)] = nb;
  }
  for (int i = 0; i < nl_vsc; ++i) {
    lb[idx.Fij_vsc(i)] = -nb; ub[idx.Fij_vsc(i)] = nb;
  }
  for (int g = 0; g < ng; ++g) {
    lb[idx.Fg(g)] = 0; ub[idx.Fg(g)] = nb;
  }

  std::vector<bool> beta_free(nl + nl_vsc, false);
  int n_beta_fixed = 0;
  for (int i = 0; i < nl + nl_vsc; ++i) {
    lb[idx.beta(i)] = 0.0;
    ub[idx.beta(i)] = 1.0;

    if (zeta[i] < 0.5) {
      lb[idx.beta(i)] = 0.0; ub[idx.beta(i)] = 0.0;
      if (i < nl) { lb[idx.Fij(i)] = 0.0; ub[idx.Fij(i)] = 0.0; }
      else        { lb[idx.Fij_vsc(i-nl)] = 0.0; ub[idx.Fij_vsc(i-nl)] = 0.0; }
      ++n_beta_fixed;
    } else {
      beta_free[i] = true;
    }
  }
  int n_beta_free = static_cast<int>(
      std::count(beta_free.begin(), beta_free.end(), true));

  for (int g = 0; g < ng; ++g) {
    lb[idx.gamma(g)] = 0.0; ub[idx.gamma(g)] = 1.0;
  }
  if (ng > 0) { lb[idx.gamma(0)] = 1.0; ub[idx.gamma(0)] = 1.0; }

  spdlog::info("[拓扑重构] β固定={}, β自由(联络线)={}", n_beta_fixed, n_beta_free);

  // -------------------------------------------------------------------
  // Connected component awareness: fix same-component tie switches
  // -------------------------------------------------------------------
  {
    std::vector<std::vector<int>> tree_adj(nb);
    for (int i = 0; i < nl + nl_vsc; ++i) {
      if (edge_status[i] && zeta[i] > 0.5 &&
          edge_from[i] >= 0 && edge_to[i] >= 0) {
        tree_adj[edge_from[i]].push_back(edge_to[i]);
        tree_adj[edge_to[i]].push_back(edge_from[i]);
      }
    }
    std::vector<int> comp_id(nb, -1);
    int n_comp = 0;
    for (int s = 0; s < nb; ++s) {
      if (comp_id[s] >= 0) continue;
      std::queue<int> bfs;
      bfs.push(s); comp_id[s] = n_comp;
      while (!bfs.empty()) {
        int u = bfs.front(); bfs.pop();
        for (int v : tree_adj[u]) {
          if (comp_id[v] < 0) { comp_id[v] = n_comp; bfs.push(v); }
        }
      }
      ++n_comp;
    }
    if (n_comp > 1) {
      int n_comp_fixed = 0;
      for (int i = 0; i < nl + nl_vsc; ++i) {
        if (!beta_free[i]) continue;
        // Only prune tie switches (not-in-service), NOT in-service edges.
        // In-service same-component edges form the tree within the component
        // and must stay free (preferentially β=1 via objective).
        if (edge_status[i]) continue;
        int u = edge_from[i], v = edge_to[i];
        if (u >= 0 && v >= 0 && comp_id[u] == comp_id[v]) {
          lb[idx.beta(i)] = 0.0; ub[idx.beta(i)] = 0.0;
          if (i < nl) { lb[idx.Fij(i)] = 0.0; ub[idx.Fij(i)] = 0.0; }
          else { lb[idx.Fij_vsc(i-nl)] = 0.0; ub[idx.Fij_vsc(i-nl)] = 0.0; }
          beta_free[i] = false; ++n_beta_fixed; ++n_comp_fixed;
        }
      }
      n_beta_free = static_cast<int>(
          std::count(beta_free.begin(), beta_free.end(), true));
      spdlog::info("[拓扑重构] 连通分量: {} 个, 同分量固定={}, 跨分量={}",
                   n_comp, n_comp_fixed, n_beta_free);
    }
  }

  // -------------------------------------------------------------------
  // Load demand (p.u.)
  // -------------------------------------------------------------------
  std::vector<double> Pd_pu(nb, 0.0), Qd_pu(nb, 0.0);
  for (const auto& ld : ac.loads) {
    if (!ld.in_service) continue;
    auto it = ac_id_map.find(ld.bus);
    if (it == ac_id_map.end()) continue;
    Pd_pu[it->second] += ld.p_mw / base_mva;
    Qd_pu[it->second] += ld.q_mvar / base_mva;
  }
  for (const auto& ld : dc.loads) {
    if (!ld.in_service) continue;
    auto it = dc_id_map.find(ld.bus);
    if (it == dc_id_map.end()) continue;
    Pd_pu[it->second] += ld.p_mw / base_mva;
  }
  if (ac.loads.empty()) {
    for (int i = 0; i < nb_ac; ++i) {
      Pd_pu[i] += ac.buses[i].pd_mw / base_mva;
      Qd_pu[i] += ac.buses[i].qd_mvar / base_mva;
    }
  }

  // Generator parameters
  std::vector<double> gen_Pmax(ng), gen_Pmin(ng), gen_Qmax(ng), gen_Qmin(ng);
  for (int g = 0; g < ng; ++g) {
    gen_Pmax[g] = ac.generators[g].pmax_mw / base_mva;
    gen_Pmin[g] = ac.generators[g].pmin_mw / base_mva;
    gen_Qmax[g] = ac.generators[g].qmax_mvar / base_mva;
    gen_Qmin[g] = ac.generators[g].qmin_mvar / base_mva;
  }

  // Voltage squared bounds
  std::vector<double> Vmin2(nb), Vmax2(nb);
  for (int i = 0; i < nb_ac; ++i) {
    double vmin = (ac.buses[i].vmin_pu > 0) ? ac.buses[i].vmin_pu : opt.v_min_pu;
    double vmax = (ac.buses[i].vmax_pu > 0) ? ac.buses[i].vmax_pu : opt.v_max_pu;
    Vmin2[i] = vmin * vmin;
    Vmax2[i] = vmax * vmax;
  }
  for (int i = 0; i < nb_dc; ++i) {
    double vmin = (dc.buses[i].vmin_pu > 0) ? dc.buses[i].vmin_pu : opt.v_min_pu;
    double vmax = (dc.buses[i].vmax_pu > 0) ? dc.buses[i].vmax_pu : opt.v_max_pu;
    Vmin2[nb_ac + i] = vmin * vmin;
    Vmax2[nb_ac + i] = vmax * vmax;
  }

  // VSC capacity
  double max_rate_pu = default_rate_pu;
  for (int i = 0; i < nl; ++i)
    max_rate_pu = std::max(max_rate_pu, edge_rate[i]);
  std::vector<double> Smax_vsc(nl_vsc, max_rate_pu * 0.5);
  for (int i = 0; i < nl_vsc; ++i)
    if (vscs[i].pmax_mw > 1e-9)
      Smax_vsc[i] = vscs[i].pmax_mw / base_mva;

  // -------------------------------------------------------------------
  // LinDistFlow variable bounds
  // -------------------------------------------------------------------
  if (opt.enable_pf) {
    for (int i = 0; i < nl; ++i) {
      lb[idx.Pij(i)] = -edge_rate[i]; ub[idx.Pij(i)] = edge_rate[i];
    }
    for (int i = 0; i < nl_vsc; ++i) {
      lb[idx.Pij_vsc(i)] = -Smax_vsc[i]; ub[idx.Pij_vsc(i)] = Smax_vsc[i];
    }
    for (int i = 0; i < nl_ac; ++i) {
      lb[idx.Qij(i)] = -edge_rate[i]; ub[idx.Qij(i)] = edge_rate[i];
    }
    for (int i = 0; i < nl_vsc; ++i) {
      lb[idx.Qij_vsc(i)] = -Smax_vsc[i]; ub[idx.Qij_vsc(i)] = Smax_vsc[i];
    }
    for (int g = 0; g < ng; ++g) {
      lb[idx.Pg(g)] = gen_Pmin[g]; ub[idx.Pg(g)] = gen_Pmax[g];
      lb[idx.Qg(g)] = gen_Qmin[g]; ub[idx.Qg(g)] = gen_Qmax[g];
    }
    for (int i = 0; i < nb; ++i) {
      lb[idx.v(i)] = Vmin2[i]; ub[idx.v(i)] = Vmax2[i];
    }
    for (int i = 0; i < nb; ++i) {
      lb[idx.sP(i)] = 0.0; ub[idx.sP(i)] = 1e20;
    }
    for (int i = 0; i < nb_ac; ++i) {
      lb[idx.sQ(i)] = 0.0; ub[idx.sQ(i)] = 1e20;
    }
  }

  // -------------------------------------------------------------------
  // Objective function
  // -------------------------------------------------------------------
  Eigen::VectorXd c = Eigen::VectorXd::Zero(idx.n_vars);

  const double lambda_sw     = 1.0;
  const double lambda_loss   = 10.0;
  const double lambda_shed   = 1e4;
  const double lambda_island = 1e5;

  for (int i = 0; i < nl + nl_vsc; ++i) {
    if (!edge_status[i])
      c[idx.beta(i)] += lambda_sw;    // penalize closing tie switches
    else
      c[idx.beta(i)] -= lambda_sw;    // reward keeping in-service edges closed
  }
  for (int i = 0; i < nl; ++i)
    c[idx.beta(i)] += lambda_loss * std::abs(edge_r[i]);
  if (opt.enable_pf) {
    for (int i = 0; i < nb; ++i)   c[idx.sP(i)] = lambda_shed;
    for (int i = 0; i < nb_ac; ++i) c[idx.sQ(i)] = lambda_shed;
  }
  for (int g = 1; g < ng; ++g)
    c[idx.gamma(g)] = lambda_island;

  // -------------------------------------------------------------------
  // Constraint dimensions
  // -------------------------------------------------------------------
  const int n_eq_topo = nb + 1;
  const int n_eq_pf   = opt.enable_pf ? (nb + nb_ac) : 0;
  const int n_eq      = n_eq_topo + n_eq_pf;

  const int n_ineq_topo = 2 * n_beta_free + ng + 1;
  const int n_ineq_pf = opt.enable_pf
      ? (2*(nl+nl_vsc) + 2*(nl+nl_vsc) + 2*(nl_ac+nl_vsc))
      : 0;
  const int n_ineq = n_ineq_topo + n_ineq_pf;

  // -------------------------------------------------------------------
  // Equality constraints (triplets)
  // -------------------------------------------------------------------
  std::vector<Eigen::Triplet<double>> eq_trips;
  eq_trips.reserve(4*(nl+nl_vsc+ng) + (opt.enable_pf ? 8*nl : 0));
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq);
  int eq_row = 0;

  // (T1) Fictitious flow conservation: Cft'·Fij + Cvsc'·Fvsc - Cg'·Fg = -1
  for (int i = 0; i < nl; ++i) {
    if (edge_from[i] < 0) continue;
    eq_trips.emplace_back(eq_row + edge_from[i], idx.Fij(i),  1.0);
    eq_trips.emplace_back(eq_row + edge_to[i],   idx.Fij(i), -1.0);
  }
  for (int i = 0; i < nl_vsc; ++i) {
    int e = nl + i;
    if (edge_from[e] < 0) continue;
    eq_trips.emplace_back(eq_row + edge_from[e], idx.Fij_vsc(i),  1.0);
    eq_trips.emplace_back(eq_row + edge_to[e],   idx.Fij_vsc(i), -1.0);
  }
  for (int g = 0; g < ng; ++g)
    if (gen_bus[g] >= 0)
      eq_trips.emplace_back(eq_row + gen_bus[g], idx.Fg(g), -1.0);
  for (int i = 0; i < nb; ++i)
    beq[eq_row + i] = -1.0;
  eq_row += nb;

  // (T4) Forest property: Σβ + Σγ = nb
  for (int i = 0; i < nl + nl_vsc; ++i)
    eq_trips.emplace_back(eq_row, idx.beta(i), 1.0);
  for (int g = 0; g < ng; ++g)
    eq_trips.emplace_back(eq_row, idx.gamma(g), 1.0);
  beq[eq_row] = static_cast<double>(nb);
  eq_row += 1;

  // (P1) Active power balance
  if (opt.enable_pf) {
    for (int i = 0; i < nl; ++i) {
      if (edge_from[i] < 0) continue;
      eq_trips.emplace_back(eq_row + edge_from[i], idx.Pij(i),  1.0);
      eq_trips.emplace_back(eq_row + edge_to[i],   idx.Pij(i), -1.0);
    }
    for (int i = 0; i < nl_vsc; ++i) {
      int e = nl + i;
      if (edge_from[e] < 0) continue;
      eq_trips.emplace_back(eq_row + edge_from[e], idx.Pij_vsc(i),  1.0);
      eq_trips.emplace_back(eq_row + edge_to[e],   idx.Pij_vsc(i), -1.0);
    }
    for (int g = 0; g < ng; ++g)
      if (gen_bus[g] >= 0)
        eq_trips.emplace_back(eq_row + gen_bus[g], idx.Pg(g), -1.0);
    for (int i = 0; i < nb; ++i) {
      eq_trips.emplace_back(eq_row + i, idx.sP(i), -1.0);
      beq[eq_row + i] = -Pd_pu[i];
    }
    eq_row += nb;

    // (P2) Reactive power balance (AC buses only)
    for (int i = 0; i < nl_ac; ++i) {
      int fi = edge_from[i], ti = edge_to[i];
      if (fi < 0) continue;
      if (fi < nb_ac) eq_trips.emplace_back(eq_row + fi, idx.Qij(i),  1.0);
      if (ti < nb_ac) eq_trips.emplace_back(eq_row + ti, idx.Qij(i), -1.0);
    }
    for (int i = 0; i < nl_vsc; ++i) {
      int e = nl + i;
      if (edge_from[e] >= 0 && edge_from[e] < nb_ac)
        eq_trips.emplace_back(eq_row + edge_from[e], idx.Qij_vsc(i),  1.0);
      if (edge_to[e] >= 0 && edge_to[e] < nb_ac)
        eq_trips.emplace_back(eq_row + edge_to[e],   idx.Qij_vsc(i), -1.0);
    }
    for (int g = 0; g < ng; ++g) {
      int gb = gen_bus[g];
      if (gb >= 0 && gb < nb_ac)
        eq_trips.emplace_back(eq_row + gb, idx.Qg(g), -1.0);
    }
    for (int i = 0; i < nb_ac; ++i) {
      eq_trips.emplace_back(eq_row + i, idx.sQ(i), -1.0);
      beq[eq_row + i] = -Qd_pu[i];
    }
    eq_row += nb_ac;
  }

  // -------------------------------------------------------------------
  // Inequality constraints (triplets)
  // -------------------------------------------------------------------
  std::vector<Eigen::Triplet<double>> ineq_trips;
  ineq_trips.reserve(n_ineq * 3);
  Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(n_ineq);
  int ineq_row = 0;

  // (T2) -nb·β ≤ Fij ≤ nb·β — free β only
  for (int i = 0; i < nl; ++i) {
    if (!beta_free[i]) continue;
    ineq_trips.emplace_back(ineq_row, idx.Fij(i), 1.0);
    ineq_trips.emplace_back(ineq_row, idx.beta(i), static_cast<double>(-nb));
    ++ineq_row;
  }
  for (int i = 0; i < nl_vsc; ++i) {
    if (!beta_free[nl+i]) continue;
    ineq_trips.emplace_back(ineq_row, idx.Fij_vsc(i), 1.0);
    ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), static_cast<double>(-nb));
    ++ineq_row;
  }
  for (int i = 0; i < nl; ++i) {
    if (!beta_free[i]) continue;
    ineq_trips.emplace_back(ineq_row, idx.Fij(i), -1.0);
    ineq_trips.emplace_back(ineq_row, idx.beta(i), static_cast<double>(-nb));
    ++ineq_row;
  }
  for (int i = 0; i < nl_vsc; ++i) {
    if (!beta_free[nl+i]) continue;
    ineq_trips.emplace_back(ineq_row, idx.Fij_vsc(i), -1.0);
    ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), static_cast<double>(-nb));
    ++ineq_row;
  }

  // (T6) Fg ≤ nb·γ
  for (int g = 0; g < ng; ++g) {
    ineq_trips.emplace_back(ineq_row, idx.Fg(g), 1.0);
    ineq_trips.emplace_back(ineq_row, idx.gamma(g), static_cast<double>(-nb));
    ++ineq_row;
  }

  // (T7) Σγ ≥ 1 → -Σγ ≤ -1
  for (int g = 0; g < ng; ++g)
    ineq_trips.emplace_back(ineq_row, idx.gamma(g), -1.0);
  b_ineq[ineq_row] = -1.0;
  ++ineq_row;

  // LinDistFlow inequalities
  const double bigM_v = opt.big_m_v;
  if (opt.enable_pf) {
    // (P3) Voltage drop big-M
    for (int i = 0; i < nl; ++i) {
      int fi = edge_from[i], ti = edge_to[i];
      if (fi < 0) { ineq_row += 2; continue; }
      double r = edge_r[i], x = edge_x[i];
      // Upper
      ineq_trips.emplace_back(ineq_row, idx.v(ti),   1.0);
      ineq_trips.emplace_back(ineq_row, idx.v(fi),  -1.0);
      ineq_trips.emplace_back(ineq_row, idx.Pij(i),  2.0*r);
      if (i < nl_ac)
        ineq_trips.emplace_back(ineq_row, idx.Qij(i), 2.0*x);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), bigM_v);
      b_ineq[ineq_row] = bigM_v;
      ++ineq_row;
      // Lower
      ineq_trips.emplace_back(ineq_row, idx.v(fi),   1.0);
      ineq_trips.emplace_back(ineq_row, idx.v(ti),  -1.0);
      ineq_trips.emplace_back(ineq_row, idx.Pij(i), -2.0*r);
      if (i < nl_ac)
        ineq_trips.emplace_back(ineq_row, idx.Qij(i), -2.0*x);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), bigM_v);
      b_ineq[ineq_row] = bigM_v;
      ++ineq_row;
    }
    for (int i = 0; i < nl_vsc; ++i) {
      int e = nl + i;
      int fi = edge_from[e], ti = edge_to[e];
      if (fi < 0) { ineq_row += 2; continue; }
      ineq_trips.emplace_back(ineq_row, idx.v(ti),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.v(fi), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), bigM_v);
      b_ineq[ineq_row] = bigM_v;
      ++ineq_row;
      ineq_trips.emplace_back(ineq_row, idx.v(fi),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.v(ti), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), bigM_v);
      b_ineq[ineq_row] = bigM_v;
      ++ineq_row;
    }

    // (P4) P thermal: P - Smax·β ≤ 0
    for (int i = 0; i < nl; ++i) {
      ineq_trips.emplace_back(ineq_row, idx.Pij(i),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), -edge_rate[i]);
      ++ineq_row;
      ineq_trips.emplace_back(ineq_row, idx.Pij(i), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), -edge_rate[i]);
      ++ineq_row;
    }
    for (int i = 0; i < nl_vsc; ++i) {
      ineq_trips.emplace_back(ineq_row, idx.Pij_vsc(i),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), -Smax_vsc[i]);
      ++ineq_row;
      ineq_trips.emplace_back(ineq_row, idx.Pij_vsc(i), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), -Smax_vsc[i]);
      ++ineq_row;
    }

    // (P5) Q thermal (AC + VSC)
    for (int i = 0; i < nl_ac; ++i) {
      ineq_trips.emplace_back(ineq_row, idx.Qij(i),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), -edge_rate[i]);
      ++ineq_row;
      ineq_trips.emplace_back(ineq_row, idx.Qij(i), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(i), -edge_rate[i]);
      ++ineq_row;
    }
    for (int i = 0; i < nl_vsc; ++i) {
      ineq_trips.emplace_back(ineq_row, idx.Qij_vsc(i),  1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), -Smax_vsc[i]);
      ++ineq_row;
      ineq_trips.emplace_back(ineq_row, idx.Qij_vsc(i), -1.0);
      ineq_trips.emplace_back(ineq_row, idx.beta(nl+i), -Smax_vsc[i]);
      ++ineq_row;
    }
  }

  // -------------------------------------------------------------------
  // Assemble sparse matrices
  // -------------------------------------------------------------------
  Eigen::SparseMatrix<double> Aeq_mat(n_eq, idx.n_vars);
  Aeq_mat.setFromTriplets(eq_trips.begin(), eq_trips.end());
  Aeq_mat.makeCompressed();

  Eigen::SparseMatrix<double> A_ineq_mat(n_ineq, idx.n_vars);
  A_ineq_mat.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  A_ineq_mat.makeCompressed();

  // -------------------------------------------------------------------
  // Build MIPModel
  // -------------------------------------------------------------------
  MIPModel milp;
  auto& lp = milp.linear_part;
  lp.sense = Sense::Minimize;
  lp.c     = c;
  lp.A     = A_ineq_mat;
  lp.b     = b_ineq;
  lp.Aeq   = Aeq_mat;
  lp.beq   = beq;

  lp.vars.resize(idx.n_vars);
  for (int i = 0; i < idx.n_vars; ++i)
    lp.vars[i] = {VarType::Continuous, lb[i], ub[i], {}};

  for (int i = 0; i < nl + nl_vsc; ++i) {
    lp.vars[idx.beta(i)].type = VarType::Binary;
    milp.binary_idx.push_back(idx.beta(i));
  }
  for (int g = 0; g < ng; ++g) {
    lp.vars[idx.gamma(g)].type = VarType::Binary;
    milp.binary_idx.push_back(idx.gamma(g));
  }

  spdlog::info("[拓扑重构] MILP: {} 变量, {} 等式, {} 不等式, {} 二元",
               idx.n_vars, n_eq, n_ineq,
               static_cast<int>(milp.binary_idx.size()));

  // -------------------------------------------------------------------
  // Graph heuristic: O(V+E) BFS for cheapest reconnecting tie switch
  // -------------------------------------------------------------------
  bool heuristic_solved = false;
  Eigen::VectorXd x_heur;

  if (!opt.skip_heuristic) {
    const auto t_heur = chr::steady_clock::now();

    // Active tree adjacency
    std::vector<std::vector<std::pair<int,int>>> adj(nb);
    for (int i = 0; i < nl + nl_vsc; ++i) {
      if (edge_status[i] && zeta[i] > 0.5 &&
          edge_from[i] >= 0 && edge_to[i] >= 0) {
        adj[edge_from[i]].emplace_back(edge_to[i], i);
        adj[edge_to[i]].emplace_back(edge_from[i], i);
      }
    }

    std::vector<bool> visited(nb, false);
    std::vector<int> parent_edge(nb, -1), parent_nd(nb, -1);
    std::vector<int> bfs_order;
    bfs_order.reserve(nb);

    {
      std::queue<int> bfs;
      bfs.push(root_bus); visited[root_bus] = true;
      bfs_order.push_back(root_bus);
      while (!bfs.empty()) {
        int u = bfs.front(); bfs.pop();
        for (auto [v, e] : adj[u]) {
          if (!visited[v]) {
            visited[v] = true;
            parent_edge[v] = e; parent_nd[v] = u;
            bfs.push(v);
            bfs_order.push_back(v);
          }
        }
      }
    }
    int n_reachable = static_cast<int>(
        std::count(visited.begin(), visited.end(), true));

    // Find cheapest cross-component tie switch
    int best_edge = -1;
    double best_cost = 1e20;
    for (int i = 0; i < nl + nl_vsc; ++i) {
      if (edge_status[i]) continue;
      if (zeta[i] < 0.5 || edge_from[i] < 0) continue;
      bool f_vis = visited[edge_from[i]], t_vis = visited[edge_to[i]];
      if (f_vis != t_vis) {
        double cost_i = c[idx.beta(i)];
        if (cost_i < best_cost) { best_cost = cost_i; best_edge = i; }
      }
    }

    // Build heuristic solution
    auto try_heuristic = [&](bool extend) -> bool {
      if (extend && best_edge >= 0) {
        int start2 = visited[edge_from[best_edge]]
                     ? edge_to[best_edge] : edge_from[best_edge];
        int parent2 = visited[edge_from[best_edge]]
                      ? edge_from[best_edge] : edge_to[best_edge];
        visited[start2] = true;
        parent_edge[start2] = best_edge; parent_nd[start2] = parent2;
        bfs_order.push_back(start2);
        std::queue<int> bfs;
        bfs.push(start2);
        while (!bfs.empty()) {
          int u = bfs.front(); bfs.pop();
          for (auto [v, e] : adj[u]) {
            if (!visited[v]) {
              visited[v] = true;
              parent_edge[v] = e; parent_nd[v] = u;
              bfs.push(v); bfs_order.push_back(v);
            }
          }
        }
      }

      std::vector<int> subtree_sz(nb, 1);
      for (int k = static_cast<int>(bfs_order.size()) - 1; k >= 0; --k) {
        int v = bfs_order[k];
        if (parent_nd[v] >= 0) subtree_sz[parent_nd[v]] += subtree_sz[v];
      }

      x_heur = Eigen::VectorXd::Zero(idx.n_vars);
      // Set β based on BFS spanning tree (exactly nb-1 edges for nb visited)
      // rather than all in-service edges, to satisfy T4: Σβ + Σγ = nb
      for (int i = 0; i < nl + nl_vsc; ++i)
        x_heur[idx.beta(i)] = 0.0;
      for (int k = 1; k < static_cast<int>(bfs_order.size()); ++k) {
        int v = bfs_order[k];
        if (parent_edge[v] >= 0) x_heur[idx.beta(parent_edge[v])] = 1.0;
      }
      x_heur[idx.gamma(0)] = 1.0;

      for (int k = 1; k < static_cast<int>(bfs_order.size()); ++k) {
        int v = bfs_order[k];
        int e = parent_edge[v];
        if (e < 0) continue;
        double flow = static_cast<double>(subtree_sz[v]);
        if (e < nl)
          x_heur[idx.Fij(e)] = (edge_from[e] == parent_nd[v]) ? flow : -flow;
        else
          x_heur[idx.Fij_vsc(e-nl)] = (edge_from[e] == parent_nd[v]) ? flow : -flow;
      }
      x_heur[idx.Fg(0)] = static_cast<double>(subtree_sz[root_bus]);

      Eigen::VectorXd eq_res = Aeq_mat * x_heur - beq;
      Eigen::VectorXd iq_res = A_ineq_mat * x_heur - b_ineq;
      double eq_viol  = eq_res.cwiseAbs().maxCoeff();
      double iq_viol  = iq_res.cwiseMax(0.0).maxCoeff();
      double lb_viol  = 0.0, ub_viol = 0.0;
      for (int i = 0; i < idx.n_vars; ++i) {
        lb_viol = std::max(lb_viol, lb[i] - x_heur[i]);
        ub_viol = std::max(ub_viol, x_heur[i] - ub[i]);
      }
      double total = std::max({eq_viol, iq_viol, lb_viol, ub_viol});
      spdlog::info("[拓扑重构] 启发式: obj={:.6f}, viol={:.8f}",
                   c.dot(x_heur), total);
      return total < 1e-6;
    };

    if (best_edge >= 0 && n_reachable < nb) {
      spdlog::info("[拓扑重构] 启发式: 替换边 #{} ({}→{})",
                   best_edge, edge_from[best_edge], edge_to[best_edge]);
      heuristic_solved = try_heuristic(true);
    } else if (n_reachable == nb) {
      spdlog::info("[拓扑重构] 启发式: 网络仍连通（故障非桥边）");
      heuristic_solved = try_heuristic(false);
    }
    if (heuristic_solved) {
      double ms = chr::duration<double,std::milli>(
          chr::steady_clock::now() - t_heur).count();
      spdlog::info("[拓扑重构] 启发式成功，耗时 {:.1f}ms", ms);
    }
  }

  // -------------------------------------------------------------------
  // B&C MILP solve (fallback)
  // -------------------------------------------------------------------
  Eigen::VectorXd x_sol;
  bool solved_ok = false;

  if (heuristic_solved) {
    x_sol = x_heur;
    solved_ok = true;
    result.milp_objective = c.dot(x_sol);
  } else {
    BCOptions bc_opt;
    bc_opt.time_limit_sec = static_cast<double>(opt.max_time_s);
    bc_opt.gap_tol = opt.mip_gap;
    bc_opt.cuts = CutType::MIR;
    bc_opt.root_cut_rounds = 5;
    bc_opt.cuts_per_round = 20;
    bc_opt.branching = BranchingStrategy::Pseudocost;
    bc_opt.node_sel = NodeSelection::Hybrid;
    bc_opt.use_feasibility_pump = true;
    bc_opt.use_simplex_lp_nodes = true;
    bc_opt.accept_verified_warm_start_incumbent = true;
    bc_opt.verbose = opt.verbose;

    if (x_heur.size() == idx.n_vars) {
      milp.initial_solution = x_heur;
    }

    spdlog::info("[拓扑重构] 启发式未找到可行方案，调用本地 B&C MILP fallback");
    BCResult bc_result = solve_milp_bc(milp, bc_opt);
    result.bc_stats = bc_result.bc_stats;
    if (bc_result.stats.success && bc_result.x.size() >= idx.n_vars) {
      x_sol = bc_result.x;
      solved_ok = true;
      result.milp_objective = bc_result.stats.objective;
      spdlog::info("[拓扑重构] B&C fallback 成功: nodes={} gap={:.6f} obj={:.6f}",
                   bc_result.bc_stats.nodes_explored,
                   bc_result.bc_stats.gap,
                   result.milp_objective);
    } else {
      solved_ok = false;
      spdlog::warn("[拓扑重构] B&C fallback 未找到可行解: {}",
                   bc_result.stats.status);
    }
  }

  // -------------------------------------------------------------------
  // Extract results
  // -------------------------------------------------------------------
  if (!solved_ok || x_sol.size() < idx.n_vars) {
    result.feasible = false;
    result.solve_time_s = chr::duration<double>(
        chr::steady_clock::now() - t_start).count();
    spdlog::warn("[拓扑重构] 求解失败");
    return result;
  }

  result.feasible = true;
  // optimal is only asserted when the B&C solver ran and proved the gap is
  // within tolerance.  A heuristic incumbent satisfies feasibility but carries
  // no optimality certificate, so optimal stays false in that case.
  result.optimal = !heuristic_solved && (result.bc_stats.gap <= opt.mip_gap + 1e-9);

  for (int i = 0; i < nl + nl_vsc; ++i) {
    bool was_on = edge_status[i];
    bool now_on = (x_sol[idx.beta(i)] > 0.5);

    if (now_on) result.closed_branch_ids.push_back(edge_orig_idx[i]);
    else        result.open_branch_ids.push_back(edge_orig_idx[i]);

    if (was_on && !now_on) {
      result.switched_off_ids.push_back(edge_orig_idx[i]);
      ++result.n_switch_off;
    } else if (!was_on && now_on) {
      result.switched_on_ids.push_back(edge_orig_idx[i]);
      ++result.n_switch_on;
    }
  }

  // Loss estimate
  double loss_proxy = 0.0;
  for (int i = 0; i < nl; ++i)
    if (x_sol[idx.beta(i)] > 0.5)
      loss_proxy += edge_r[i];
  result.reconf_loss_mw = loss_proxy * base_mva * lambda_loss;
  result.milp_objective = c.dot(x_sol);

  result.solve_time_s = chr::duration<double>(
      chr::steady_clock::now() - t_start).count();

  spdlog::info("[拓扑重构] 完成: 合闸={} 分闸={} obj={:.4f} time={:.2f}s",
               result.n_switch_on, result.n_switch_off,
               result.milp_objective, result.solve_time_s);

  return result;
}

}  // namespace hacdcpf::analysis

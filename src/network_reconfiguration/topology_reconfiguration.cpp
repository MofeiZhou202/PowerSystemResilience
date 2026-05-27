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
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"
#include "hacdcpf/solver/problem_types.hpp"

namespace hacdcpf::analysis {

namespace {

std::string normalize_status(std::string status) {
  std::transform(status.begin(), status.end(), status.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  const auto first = status.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) return {};
  const auto last = status.find_last_not_of(" \t\n\r");
  std::string trimmed = status.substr(first, last - first + 1);
  std::string collapsed;
  collapsed.reserve(trimmed.size());
  bool previous_space = false;
  for (char ch : trimmed) {
    const bool is_space = std::isspace(static_cast<unsigned char>(ch)) != 0;
    if (is_space) {
      if (!previous_space) collapsed.push_back(' ');
    } else {
      collapsed.push_back(ch);
    }
    previous_space = is_space;
  }
  return collapsed;
}

bool status_indicates_optimal(const std::string& status) {
  const std::string normalized = normalize_status(status);
  return normalized == "optimal" ||
         normalized == "highs optimal" ||
         normalized == "optimal solution found" ||
         normalized == "optimal (root gap closed)" ||
         normalized == "optimal (tree exhausted)" ||
         normalized == "optimality gap reached" ||
         normalized == "solved to optimality";
}

}  // namespace

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
  const int n_ac_gen = static_cast<int>(ac.generators.size());

  TopoReconfResult result;
  result.model_scope = opt.enable_pf ? "hybrid-acdc-topology-lindistflow"
                                     : "hybrid-acdc-topology-connectivity";
  result.validity.radial_topology_enforced = true;
  result.validity.ac_lindistflow_enforced = opt.enable_pf;
  result.validity.dc_network_modelled = nb_dc > 0;
  result.validity.vsc_active_transfer_modelled = nl_vsc > 0;
  result.validity.vsc_reactive_power_approximated = opt.enable_pf && nl_vsc > 0;
  if (nb == 0 || nl == 0) {
    if (opt.verbose) {
      spdlog::warn("[拓扑重构] 系统为空 (nb={}, nl={})", nb, nl);
    }
    return result;
  }

  if (opt.verbose) {
    spdlog::info("[拓扑重构] 系统: {} 母线(AC={},DC={}), {} 支路(AC={},DC={}), "
                 "{} VSC, {} 发电机",
                 nb, nb_ac, nb_dc, nl, nl_ac, nl_dc, nl_vsc, n_ac_gen);
  }

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
  std::vector<bool> edge_switchable(nl + nl_vsc, false);
  std::vector<int> edge_orig_idx(nl + nl_vsc, -1);

  const bool has_explicit_switchables = !opt.switchable_branches.empty() ||
                                        !opt.switchable_branch_ids.empty();
  std::unordered_set<int> explicit_ac_switchables(
      opt.switchable_branch_ids.begin(), opt.switchable_branch_ids.end());
  std::unordered_set<int> explicit_dc_switchables;
  std::unordered_set<int> explicit_vsc_switchables;
  for (const auto& ref : opt.switchable_branches) {
    switch (ref.category) {
      case graph::EdgeCategory::AC_Line:
        explicit_ac_switchables.insert(ref.index);
        break;
      case graph::EdgeCategory::DC_Line:
        explicit_dc_switchables.insert(ref.index);
        break;
      case graph::EdgeCategory::VSC_Coupling:
        explicit_vsc_switchables.insert(ref.index);
        break;
      default:
        break;
    }
  }
  auto is_explicit_ac_switchable = [&](int id) {
    return explicit_ac_switchables.find(id) != explicit_ac_switchables.end();
  };
  auto is_explicit_dc_switchable = [&](int id) {
    return explicit_dc_switchables.find(id) != explicit_dc_switchables.end();
  };
  auto is_explicit_vsc_switchable = [&](int id) {
    return explicit_vsc_switchables.find(id) != explicit_vsc_switchables.end();
  };

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
    edge_switchable[i] = has_explicit_switchables ? is_explicit_ac_switchable(br.index) : !br.in_service;
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
    edge_switchable[e] = has_explicit_switchables ? is_explicit_dc_switchable(br.index) : !br.in_service;
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
    edge_switchable[e] = has_explicit_switchables && is_explicit_vsc_switchable(vsc.index);
  }

  // -------------------------------------------------------------------
  // Fault status ζ: 1=available, 0=faulted
  // -------------------------------------------------------------------
  std::vector<double> zeta(nl + nl_vsc, 1.0);
  auto mark_faulted_edge = [&](graph::EdgeCategory category, int component_index) {
    int begin = 0;
    int end = 0;
    switch (category) {
      case graph::EdgeCategory::AC_Line:
        begin = 0;
        end = nl_ac;
        break;
      case graph::EdgeCategory::DC_Line:
        begin = nl_ac;
        end = nl;
        break;
      case graph::EdgeCategory::VSC_Coupling:
        begin = nl;
        end = nl + nl_vsc;
        break;
      default:
        return;
    }
    for (int edge = begin; edge < end; ++edge) {
      if (edge_orig_idx[edge] == component_index) {
        zeta[edge] = 0.0;
        return;
      }
    }
  };
  for (int f : opt.line_failures) {
    // line_failures contains ACBranch::index values (the .index field stored on each
    // branch), NOT 0-based array positions.  edge_orig_idx[i] == br.index, so the
    // match is correct.  Search only AC branch edges [0, nl_ac) to avoid spurious
    // matches against DC branches or VSC converters that may share the same index value.
    mark_faulted_edge(graph::EdgeCategory::AC_Line, f);
  }
  for (const auto& ref : opt.faulted_branches) {
    mark_faulted_edge(ref.category, ref.index);
  }

  // -------------------------------------------------------------------
  // Source buses (AC and DC controllable/fixed injections + voltage sources).
  // -------------------------------------------------------------------
  int root_bus = -1;
  std::vector<bool> source_at(static_cast<size_t>(nb), false);
  std::vector<double> source_pmin(static_cast<size_t>(nb), 0.0);
  std::vector<double> source_pmax(static_cast<size_t>(nb), 0.0);
  std::vector<double> source_qmin(static_cast<size_t>(nb), 0.0);
  std::vector<double> source_qmax(static_cast<size_t>(nb), 0.0);
  bool dc_source_present = false;

  auto add_source_pos = [&](int pos, double pmin, double pmax, double qmin, double qmax) {
    if (pos < 0 || pos >= nb) return;
    const double p_cap = std::max({0.0, pmin, pmax});
    const double q_cap = std::max(std::abs(qmin), std::abs(qmax));
    if (p_cap <= 1e-9 && q_cap <= 1e-9) return;
    source_at[pos] = true;
    source_pmin[pos] += std::max(0.0, pmin) / base_mva;
    source_pmax[pos] += std::max(0.0, pmax) / base_mva;
    source_qmin[pos] += qmin / base_mva;
    source_qmax[pos] += std::max(0.0, qmax) / base_mva;
    if (root_bus < 0) root_bus = pos;
  };
  auto add_ac_source = [&](int bus, double pmin, double pmax, double qmin, double qmax) {
    auto it = ac_id_map.find(bus);
    if (it == ac_id_map.end()) return;
    add_source_pos(it->second, pmin, pmax, qmin, qmax);
  };
  auto add_dc_source = [&](int bus, double pmin, double pmax) {
    auto it = dc_id_map.find(bus);
    if (it == dc_id_map.end()) return;
    const bool was_source = source_at[static_cast<size_t>(it->second)];
    add_source_pos(it->second, pmin, pmax, 0.0, 0.0);
    if (!was_source && source_at[static_cast<size_t>(it->second)]) {
      dc_source_present = true;
    }
  };

  for (const auto& g : ac.generators) {
    if (!g.in_service) continue;
    add_ac_source(g.bus, g.pmin_mw, g.pmax_mw > 0.0 ? g.pmax_mw : g.pg_mw,
                  g.qmin_mvar, g.qmax_mvar > 0.0 ? g.qmax_mvar : std::abs(g.qg_mvar));
  }
  for (const auto& sg : ac.static_generators) {
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    if (cap > 0.0) add_ac_source(sg.bus, 0.0, cap, sg.qmin_mvar, sg.qmax_mvar);
  }
  for (const auto& rg : ac.renewable_gens) {
    if (!rg.in_service) continue;
    const double cap = rg.p_rated_mw > 0.0 ? rg.p_rated_mw * rg.capacity_factor : rg.p_mw;
    if (cap > 0.0) add_ac_source(rg.bus, 0.0, cap, rg.qmin_mvar, rg.qmax_mvar);
  }
  for (const auto& pv : ac.pv_systems) {
    if (!pv.in_service) continue;
    const double cap = pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw;
    if (cap > 0.0) add_ac_source(pv.bus, 0.0, cap, pv.qmin_mvar, pv.qmax_mvar);
  }
  for (const auto& st : ac.storage) {
    const double cap = hacdcpf::model::effective_capacity_mw(st);
    if (cap > 0.0) add_ac_source(st.bus, 0.0, cap, st.qmin_mvar, st.qmax_mvar);
  }
  for (const auto& eg : ac.external_grids) {
    if (!eg.in_service) continue;
    const double cap = eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : opt.default_rate_mva;
    add_ac_source(eg.bus, 0.0, cap, -cap, cap);
  }
  for (const auto& sg : dc.dc_static_generators) {
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    if (cap > 0.0) add_dc_source(sg.bus, 0.0, cap);
  }
  for (const auto& sg : dc.static_generators) {
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    if (cap > 0.0) add_dc_source(sg.bus, 0.0, cap);
  }
  for (const auto& st : dc.storage) {
    const double cap = hacdcpf::model::effective_capacity_mw(st);
    if (cap > 0.0) add_dc_source(st.bus, 0.0, cap);
  }
  for (const auto& pv : dc.pv_arrays) {
    if (!pv.in_service) continue;
    if (pv.p_set_mw > 0.0) add_dc_source(pv.bus, 0.0, pv.p_set_mw);
  }
  for (const auto& b : dc.buses) {
    if (!b.in_service || b.bus_type != DCBusType::DC_V) continue;
    const double cap = std::max(opt.default_rate_mva, base_mva);
    add_dc_source(b.index, 0.0, cap);
  }
  if (root_bus < 0) {
    for (int i = 0; i < nb_ac; ++i) {
      if (ac.buses[i].bus_type == BusType::SLACK) {
        const double cap = std::max(opt.default_rate_mva, base_mva);
        add_ac_source(ac.buses[i].index, 0.0, cap, -cap, cap);
        break;
      }
    }
  }
  if (root_bus < 0) {
    result.solver_status = "infeasible: no AC/DC source bus available";
    if (opt.verbose) {
      spdlog::warn("[拓扑重构] 无可用源节点，无法构造拓扑根节点");
    }
    return result;
  }
  result.validity.dc_source_dispatch_modelled = dc_source_present;

  std::vector<int> gen_bus;
  std::vector<double> gen_Pmax, gen_Pmin, gen_Qmax, gen_Qmin;
  for (int i = 0; i < nb; ++i) {
    if (!source_at[i]) continue;
    gen_bus.push_back(i);
    gen_Pmax.push_back(source_pmax[i]);
    gen_Pmin.push_back(source_pmin[i]);
    gen_Qmax.push_back(source_qmax[i]);
    gen_Qmin.push_back(source_qmin[i]);
  }
  const int ng = static_cast<int>(gen_bus.size());

  // -------------------------------------------------------------------
  // Variable layout
  // -------------------------------------------------------------------
  VarLayout idx(nl, nl_vsc, ng, nb, nb_ac, nl_ac, opt.enable_pf);
  if (opt.verbose) {
    spdlog::info("[拓扑重构] 变量数: {} (enable_pf={})", idx.n_vars, opt.enable_pf);
  }

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
    } else if (edge_switchable[i]) {
      beta_free[i] = true;
    } else {
      const double fixed = edge_status[i] ? 1.0 : 0.0;
      lb[idx.beta(i)] = fixed;
      ub[idx.beta(i)] = fixed;
      if (fixed < 0.5) {
        if (i < nl) { lb[idx.Fij(i)] = 0.0; ub[idx.Fij(i)] = 0.0; }
        else        { lb[idx.Fij_vsc(i-nl)] = 0.0; ub[idx.Fij_vsc(i-nl)] = 0.0; }
      }
      ++n_beta_fixed;
    }
  }
  int n_beta_free = static_cast<int>(
      std::count(beta_free.begin(), beta_free.end(), true));

  for (int g = 0; g < ng; ++g) {
    lb[idx.gamma(g)] = 0.0; ub[idx.gamma(g)] = 1.0;
  }
  if (ng > 0) { lb[idx.gamma(0)] = 1.0; ub[idx.gamma(0)] = 1.0; }

  if (opt.verbose) {
    spdlog::info("[拓扑重构] β固定={}, β自由(联络线)={}", n_beta_fixed, n_beta_free);
  }

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
      if (opt.verbose) {
        spdlog::info("[拓扑重构] 连通分量: {} 个, 同分量固定={}, 跨分量={}",
                     n_comp, n_comp_fixed, n_beta_free);
      }
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
  for (int i = 0; i < nb_ac; ++i) {
    Pd_pu[i] += ac.buses[i].pd_mw / base_mva;
    Qd_pu[i] += ac.buses[i].qd_mvar / base_mva;
  }
  for (int i = 0; i < nb_dc; ++i) {
    Pd_pu[nb_ac + i] += dc.buses[i].pd_mw / base_mva;
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

  if (opt.verbose) {
    spdlog::info("[拓扑重构] MILP: {} 变量, {} 等式, {} 不等式, {} 二元",
                 idx.n_vars, n_eq, n_ineq,
                 static_cast<int>(milp.binary_idx.size()));
  }

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
      if (!beta_free[i]) continue;
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
      if (opt.verbose) {
        spdlog::info("[拓扑重构] 启发式: obj={:.6f}, viol={:.8f}",
                     c.dot(x_heur), total);
      }
      return total < 1e-6;
    };

    if (best_edge >= 0 && n_reachable < nb) {
      if (opt.verbose) {
        spdlog::info("[拓扑重构] 启发式: 替换边 #{} ({}→{})",
                     best_edge, edge_from[best_edge], edge_to[best_edge]);
      }
      heuristic_solved = try_heuristic(true);
    } else if (n_reachable == nb) {
      if (opt.verbose) {
        spdlog::info("[拓扑重构] 启发式: 网络仍连通（故障非桥边）");
      }
      heuristic_solved = try_heuristic(false);
    }
    if (heuristic_solved) {
      double ms = chr::duration<double,std::milli>(
          chr::steady_clock::now() - t_heur).count();
      if (opt.verbose) {
        spdlog::info("[拓扑重构] 启发式成功，耗时 {:.1f}ms", ms);
      }
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
    result.solver_backend = "graph-heuristic";
    result.solver_status = "feasible incumbent";
    result.solver_mip_gap = std::numeric_limits<double>::infinity();
    result.proven_optimal = false;
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

    if (opt.verbose) {
      spdlog::info("[拓扑重构] 启发式未找到可行方案，调用 MILP fallback");
    }

    auto try_native_bc = [&](const std::string& previous_failure) -> bool {
      BCResult bc_result = solve_milp_bc(milp, bc_opt);
      result.bc_stats = bc_result.bc_stats;
      if (bc_result.stats.success && bc_result.x.size() >= idx.n_vars) {
        x_sol = bc_result.x;
        result.milp_objective = bc_result.stats.objective;
        result.solver_backend = previous_failure.empty()
            ? "NativeB&C"
            : "NativeB&C(after HiGHS failure)";
        result.solver_status = previous_failure.empty()
            ? bc_result.stats.status
            : "HiGHS failed: " + previous_failure + "; NativeB&C: " + bc_result.stats.status;
        result.solver_mip_gap = bc_result.bc_stats.gap;
        result.proven_optimal = status_indicates_optimal(bc_result.stats.status) &&
          std::isfinite(result.solver_mip_gap) &&
          result.solver_mip_gap <= opt.mip_gap + 1e-9;
        if (opt.verbose) {
          spdlog::info("[拓扑重构] B&C fallback 成功: nodes={} gap={:.6f} obj={:.6f}",
                       bc_result.bc_stats.nodes_explored,
                       bc_result.bc_stats.gap,
                       result.milp_objective);
        }
        return true;
      }
      result.solver_backend = previous_failure.empty()
          ? "NativeB&C"
          : "HiGHS+NativeB&C";
      result.solver_status = previous_failure.empty()
          ? bc_result.stats.status
          : "HiGHS failed: " + previous_failure + "; NativeB&C failed: " + bc_result.stats.status;
      result.solver_mip_gap = bc_result.bc_stats.gap;
      result.proven_optimal = false;
      if (opt.verbose) {
        spdlog::warn("[拓扑重构] B&C fallback 未找到可行解: {}",
                     bc_result.stats.status);
      }
      return false;
    };

    engine::HighsAdapter highs;
    if (highs.available()) {
      auto highs_result = highs.solve_milp(milp);
      if (highs_result.stats.success && highs_result.x.size() >= idx.n_vars) {
        x_sol = highs_result.x;
        solved_ok = true;
        result.milp_objective = highs_result.stats.objective;
        result.solver_backend = "HiGHS";
        result.solver_status = highs_result.stats.status;
        result.solver_mip_gap = highs_result.stats.mip_gap;
        result.proven_optimal = status_indicates_optimal(result.solver_status) &&
          std::isfinite(result.solver_mip_gap) &&
          result.solver_mip_gap <= opt.mip_gap + 1e-9;
        if (opt.verbose) {
          spdlog::info("[拓扑重构] HiGHS fallback 成功: obj={:.6f}",
                       result.milp_objective);
        }
      } else {
        std::string reason = highs_result.stats.status.empty()
            ? std::string("unsuccessful solve")
            : highs_result.stats.status;
        if (highs_result.stats.success && highs_result.x.size() < idx.n_vars) {
          reason += " (solution vector too small)";
        }
        if (opt.verbose) {
          spdlog::warn("[拓扑重构] HiGHS fallback 未找到可行解: {}; retry NativeB&C",
                       reason);
        }
        solved_ok = try_native_bc(reason);
      }
    } else {
      solved_ok = try_native_bc("");
    }
  }

  // -------------------------------------------------------------------
  // Extract results
  // -------------------------------------------------------------------
  if (!solved_ok || x_sol.size() < idx.n_vars) {
    result.feasible = false;
    result.solve_time_s = chr::duration<double>(
        chr::steady_clock::now() - t_start).count();
    if (opt.verbose) {
      spdlog::warn("[拓扑重构] 求解失败");
    }
    return result;
  }

  result.feasible = true;
  // Post-solve constraint verification (mirrors resilience analyze_solution).
  // Catches cases where the solver declares success but the solution violates
  // a constraint beyond floating-point round-off.
  {
    double eq_viol   = 0.0;
    double ineq_viol = 0.0;
    if (Aeq_mat.rows() > 0) {
      Eigen::VectorXd res_eq = Aeq_mat * x_sol - beq;
      eq_viol = res_eq.cwiseAbs().maxCoeff();
    }
    if (A_ineq_mat.rows() > 0) {
      Eigen::VectorXd res_ineq = A_ineq_mat * x_sol - b_ineq;
      ineq_viol = res_ineq.cwiseMax(0.0).maxCoeff();
    }
    if (eq_viol > 1e-6 || ineq_viol > 1e-6) {
      if (opt.verbose) {
        spdlog::warn("[拓扑重构] 后验约束违约: eq_viol={:.2e} ineq_viol={:.2e}",
                     eq_viol, ineq_viol);
      }
      result.feasible = false;
      result.solve_time_s = chr::duration<double>(
          chr::steady_clock::now() - t_start).count();
      return result;
    }
    // Binary integrality check: each declared-binary variable must be within
    // 1e-4 of 0 or 1.  A tolerance of 0.1 would accept 0.09 / 0.91, which
    // the >0.5 rounding step turns into 0/1 without re-checking feasibility.
    for (int bi : milp.binary_idx) {
      if (bi >= static_cast<int>(x_sol.size())) continue;
      double v = x_sol[bi];
      double frac = std::min(v - std::floor(v), std::ceil(v) - v);
      if (frac > 1e-4) {
        if (opt.verbose) {
          spdlog::warn("[拓扑重构] 后验整数性违约: var[{}]={:.6f} (frac={:.2e})", bi, v, frac);
        }
        result.feasible = false;
        result.solve_time_s = chr::duration<double>(
            chr::steady_clock::now() - t_start).count();
        return result;
      }
    }
    // Variable bounds check
    for (int i = 0; i < idx.n_vars; ++i) {
      if (i >= static_cast<int>(x_sol.size())) continue;
      if (x_sol[i] < lb[i] - 1e-6 || x_sol[i] > ub[i] + 1e-6) {
        if (opt.verbose) {
          spdlog::warn("[拓扑重构] 后验变量界违约: var[{}]={:.4f} bounds=[{:.4f},{:.4f}]",
                       i, x_sol[i], lb[i], ub[i]);
        }
        result.feasible = false;
        result.solve_time_s = chr::duration<double>(
            chr::steady_clock::now() - t_start).count();
        return result;
      }
    }
  }

  // optimal is only asserted when the B&C solver ran and proved the gap is
  // within tolerance.  A heuristic incumbent satisfies feasibility but carries
  // no optimality certificate, so optimal stays false in that case.
  result.optimal = result.proven_optimal;

  for (int i = 0; i < nl + nl_vsc; ++i) {
    bool was_on = edge_status[i];
    bool now_on = (x_sol[idx.beta(i)] > 0.5);

    // Build a structured reference so callers can distinguish AC/DC/VSC.
    BranchRef ref;
    if      (i < idx.nl_ac) ref.category = graph::EdgeCategory::AC_Line;
    else if (i < nl)        ref.category = graph::EdgeCategory::DC_Line;
    else                    ref.category = graph::EdgeCategory::VSC_Coupling;
    ref.index = edge_orig_idx[i];

    if (now_on) {
      result.closed_branch_ids.push_back(edge_orig_idx[i]);  // legacy
      result.closed_branches.push_back(ref);
    } else {
      result.open_branch_ids.push_back(edge_orig_idx[i]);    // legacy
      result.open_branches.push_back(ref);
    }

    if (was_on && !now_on) {
      result.switched_off_ids.push_back(edge_orig_idx[i]);   // legacy
      result.switched_off.push_back(ref);
      ++result.n_switch_off;
    } else if (!was_on && now_on) {
      result.switched_on_ids.push_back(edge_orig_idx[i]);    // legacy
      result.switched_on.push_back(ref);
      ++result.n_switch_on;
    }
  }

  // Loss estimate
  // NOTE: reconf_loss_mw is a nominal-current approximation, NOT a measured
  // MW value.  loss_proxy = Σ r_pu for all closed branches.  Multiplying by
  // base_mva gives r_pu × MVA, which equals I²R [MW] only when every branch
  // carries its full rated current at 1 pu voltage — a rough proxy for
  // topology comparison.  Do NOT multiply by lambda_loss (the MILP objective
  // weight); that would mix a physical unit with a tuning parameter.
  // For accurate loss accounting, run a full power flow after reconfiguration.
  double loss_proxy = 0.0;
  for (int i = 0; i < nl; ++i)
    if (x_sol[idx.beta(i)] > 0.5)
      loss_proxy += edge_r[i];
  result.reconf_loss_mw = loss_proxy * base_mva;  // nominal-current proxy [MW]
  result.milp_objective = c.dot(x_sol);

  result.solve_time_s = chr::duration<double>(
      chr::steady_clock::now() - t_start).count();

  if (opt.verbose) {
    spdlog::info("[拓扑重构] 完成: 合闸={} 分闸={} obj={:.4f} time={:.2f}s",
                 result.n_switch_on, result.n_switch_off,
                 result.milp_objective, result.solve_time_s);
  }

  return result;
}

}  // namespace hacdcpf::analysis

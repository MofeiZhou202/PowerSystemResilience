#include "hacdcpf/resilience/resilience_assessment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/native_adapters.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"

namespace hacdcpf::analysis {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;

std::string resilience_mip_solver_name(DistributionResilienceMIPSolver solver,
                                       int num_threads) {
  switch (solver) {
    case DistributionResilienceMIPSolver::Native:
      return num_threads > 1 ? "native-bc-par" : "native-bc";
    case DistributionResilienceMIPSolver::HiGHS:
      return "HiGHS";
    case DistributionResilienceMIPSolver::Gurobi:
      return "Gurobi";
  }
  return "unknown";
}

solver::NodeSelection to_bc_node_selection(
    DistributionResilienceNativeNodeSelection selection) {
  switch (selection) {
    case DistributionResilienceNativeNodeSelection::Hybrid:
      return solver::NodeSelection::Hybrid;
    case DistributionResilienceNativeNodeSelection::BestFirst:
      return solver::NodeSelection::BestFirst;
  }
  return solver::NodeSelection::Hybrid;
}

double sample_profile(const std::vector<double>& profile, double hour) {
  if (profile.empty()) return 1.0;
  const double period = static_cast<double>(profile.size());
  double h = std::fmod(hour, period);
  if (h < 0.0) h += period;
  const size_t lo = static_cast<size_t>(h) % profile.size();
  const size_t hi = (lo + 1) % profile.size();
  const double frac = h - std::floor(h);
  const double value = profile[lo] * (1.0 - frac) + profile[hi] * frac;
  return std::max(0.0, value);
}

std::unordered_map<int, int> make_bus_pos_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  out.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) out[sys.ac.buses[i].index] = static_cast<int>(i);
  return out;
}

std::unordered_map<int, int> make_branch_pos_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  out.reserve(sys.ac.branches.size());
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const int idx = sys.ac.branches[i].index != 0 ? sys.ac.branches[i].index : static_cast<int>(i + 1);
    out[idx] = static_cast<int>(i);
  }
  return out;
}

enum class PriorityTier { Critical, High, Medium, Low };

double penalty_from_priority(PriorityTier tier, const DistributionResilienceMIPOptions& opts) {
  switch (tier) {
    case PriorityTier::Critical: return opts.shed_penalty_critical;
    case PriorityTier::High: return opts.shed_penalty_high;
    case PriorityTier::Medium: return opts.shed_penalty_medium;
    case PriorityTier::Low: return opts.shed_penalty_low;
  }
  return opts.shed_penalty_low;
}

PriorityTier priority_from_importance(double importance) {
  if (importance >= 4.0 - kEps) return PriorityTier::Critical;
  if (importance >= 2.5 - kEps) return PriorityTier::High;
  if (importance >= 1.5 - kEps) return PriorityTier::Medium;
  return PriorityTier::Low;
}

struct BusData {
  int index{0};
  bool base_source_available{false};
  double base_source_cap_mw{0.0};
  double dispatchable_gen_cap_mw{0.0};
  double renewable_cap_mw{0.0};
  double importance{1.0};
  PriorityTier priority{PriorityTier::Low};
};

struct BranchData {
  int index{0};
  int from_pos{0};
  int to_pos{0};
  double r_pu{0.01};
  double rate_mw{10.0};
  bool initial_closed{true};
};

struct FixedStorageData {
  int index{0};
  int bus_pos{0};
  double pmax_mw{0.0};
  double e_init_mwh{0.0};
  double e_min_mwh{0.0};
  double e_max_mwh{0.0};
  double eta_discharge{1.0};
  bool grid_forming{false};
};

struct MESSData {
  int index{0};
  int init_bus_pos{0};
  double pmax_mw{0.0};
  double e_init_mwh{0.0};
  double e_min_mwh{0.0};
  double e_max_mwh{0.0};
  double eta_discharge{1.0};
  double e_travel_mwh_km{0.0};
  double max_travel_distance_km{kInf};
};

struct FaultData {
  int branch_pos{0};
  int branch_index{0};
  double start_hr{0.0};
  double repair_hr{0.0};
  std::string name;
};

using Adj = std::vector<std::vector<std::pair<int, double>>>;

Adj build_transport_graph(const HybridPowerSystem& sys,
                          const DistributionResilienceOptions& opts,
                          const std::unordered_map<int, int>& bus_pos) {
  Adj graph(sys.ac.buses.size());
  auto add_edge = [&](int fb, int tb, double dist) {
    const auto it_f = bus_pos.find(fb);
    const auto it_t = bus_pos.find(tb);
    if (it_f == bus_pos.end() || it_t == bus_pos.end()) return;
    const double w = dist > 0.0 ? dist : 1.0;
    graph[static_cast<size_t>(it_f->second)].push_back({it_t->second, w});
    graph[static_cast<size_t>(it_t->second)].push_back({it_f->second, w});
  };
  if (!opts.transport_edges.empty()) {
    for (const auto& e : opts.transport_edges) {
      if (e.available) add_edge(e.from_bus, e.to_bus, e.distance_km);
    }
    return graph;
  }
  if (opts.use_electrical_graph_as_transport_proxy) {
    for (const auto& br : sys.ac.branches) add_edge(br.from_bus, br.to_bus, br.length_km);
  }
  return graph;
}

std::vector<double> shortest_path_distances(const Adj& graph, int start) {
  std::vector<double> dist(graph.size(), kInf);
  using Node = std::pair<double, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
  dist[static_cast<size_t>(start)] = 0.0;
  pq.push({0.0, start});
  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[static_cast<size_t>(u)] + kEps) continue;
    for (const auto& [v, w] : graph[static_cast<size_t>(u)]) {
      const double nd = d + w;
      if (nd + kEps < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = nd;
        pq.push({nd, v});
      }
    }
  }
  return dist;
}

std::vector<FaultData> build_faults(const HybridPowerSystem& sys,
                                    const DistributionResilienceOptions& opts,
                                    const std::unordered_map<int, int>& branch_pos) {
  std::vector<FaultData> out;
  if (!opts.faults.empty()) {
    out.reserve(opts.faults.size());
    for (const auto& f : opts.faults) {
      const auto it = branch_pos.find(f.ac_branch_index);
      if (it == branch_pos.end()) continue;
      const auto& br = sys.ac.branches[static_cast<size_t>(it->second)];
      out.push_back({it->second,
                     br.index != 0 ? br.index : f.ac_branch_index,
                     f.outage_start_hr,
                     std::max(opts.time_step_hr, f.repair_duration_hr),
                     f.name.empty() ? br.name : f.name});
    }
    return out;
  }
  std::vector<int> candidates;
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    if (sys.ac.branches[i].in_service) candidates.push_back(static_cast<int>(i));
  }
  const int count = std::min(opts.default_fault_count, static_cast<int>(candidates.size()));
  for (int k = 0; k < count; ++k) {
    const int pos = candidates[static_cast<size_t>(k)];
    const auto& br = sys.ac.branches[static_cast<size_t>(pos)];
    out.push_back({pos,
                   br.index != 0 ? br.index : static_cast<int>(pos + 1),
                   opts.auto_fault_start_hr + k * std::max(0.0, opts.auto_fault_stagger_hr),
                   std::max(opts.time_step_hr, opts.default_repair_time_hr),
                   br.name});
  }
  return out;
}

struct ArcDef {
  int from_pos{0};
  int to_pos{0};
  int depart_t{0};
  int arrive_t{0};
  double distance_km{0.0};
  double travel_mwh{0.0};
  bool is_stay{false};
};

struct IndexMaps {
  int T{0};
  int n_bus{0};
  int n_branch{0};
  int n_fixed_storage{0};
  int n_mess{0};

  std::vector<int> z;
  std::vector<int> y_on;
  std::vector<int> y_off;
  std::vector<int> p_branch;
  std::vector<int> f_branch;
  std::vector<int> v_bus;
  std::vector<int> u_bus;
  std::vector<int> root_bus;
  std::vector<int> root_base;
  std::vector<int> root_flow;
  std::vector<int> p_source;
  std::vector<int> shed;
  std::vector<int> renewable_used;
  std::vector<int> gen_dispatch;
  std::vector<int> fs_dis;
  std::vector<int> fs_e;
  std::vector<int> fs_root;
  std::vector<int> mess_e;
  std::vector<int> mess_x;
  std::vector<int> mess_root;
  std::vector<int> mess_dis;
  std::vector<std::vector<int>> mess_arc_var;

  int bt(int b, int t) const { return z[static_cast<size_t>(t * n_branch + b)]; }
  int yon(int b, int t) const { return y_on[static_cast<size_t>(t * n_branch + b)]; }
  int yoff(int b, int t) const { return y_off[static_cast<size_t>(t * n_branch + b)]; }
  int pb(int b, int t) const { return p_branch[static_cast<size_t>(t * n_branch + b)]; }
  int fb(int b, int t) const { return f_branch[static_cast<size_t>(t * n_branch + b)]; }
  int vb(int i, int t) const { return v_bus[static_cast<size_t>(t * n_bus + i)]; }
  int ub(int i, int t) const { return u_bus[static_cast<size_t>(t * n_bus + i)]; }
  int rb(int i, int t) const { return root_bus[static_cast<size_t>(t * n_bus + i)]; }
  int rbase(int i, int t) const { return root_base[static_cast<size_t>(t * n_bus + i)]; }
  int rflow(int i, int t) const { return root_flow[static_cast<size_t>(t * n_bus + i)]; }
  int psrc(int i, int t) const { return p_source[static_cast<size_t>(t * n_bus + i)]; }
  int shedv(int i, int t) const { return shed[static_cast<size_t>(t * n_bus + i)]; }
  int ren(int i, int t) const { return renewable_used[static_cast<size_t>(t * n_bus + i)]; }
  int gen(int i, int t) const { return gen_dispatch[static_cast<size_t>(t * n_bus + i)]; }
  int fsd(int s, int t) const { return fs_dis[static_cast<size_t>(t * n_fixed_storage + s)]; }
  int fse(int s, int t) const { return fs_e[static_cast<size_t>(t * n_fixed_storage + s)]; }
  int fsr(int s, int t) const { return fs_root[static_cast<size_t>(t * n_fixed_storage + s)]; }
  int me(int m, int t) const { return mess_e[static_cast<size_t>(t * n_mess + m)]; }
  int mx(int m, int i, int t) const { return mess_x[static_cast<size_t>((t * n_mess + m) * n_bus + i)]; }
  int mr(int m, int i, int t) const { return mess_root[static_cast<size_t>((t * n_mess + m) * n_bus + i)]; }
  int md(int m, int i, int t) const { return mess_dis[static_cast<size_t>((t * n_mess + m) * n_bus + i)]; }
};

struct BuildArtifacts {
  solver::MIPModel model;
  IndexMaps idx;
  std::vector<BusData> buses;
  std::vector<BranchData> branches;
  std::vector<FixedStorageData> fixed_storage;
  std::vector<MESSData> mess;
  std::vector<std::vector<ArcDef>> mess_arcs;
  std::vector<std::vector<double>> demand_mw;
  std::vector<std::vector<double>> renewable_avail_mw;
  std::vector<std::vector<bool>> branch_available;
  std::vector<FaultData> faults;
  DistributionResilienceModelStats stats;
  int steps{0};
};

struct SolutionDiagnostics {
  double max_bound_violation{0.0};
  int max_bound_var{-1};
  std::string max_bound_var_name;
  double recomputed_objective{0.0};
  double min_objective_coeff{0.0};
  // Constraint residuals (independent of the solver's internal tolerances).
  double max_ineq_violation{0.0};   // max(A*x - b)  (>0 means infeasible)
  double max_eq_violation{0.0};     // max|Aeq*x - beq|
  // Integrality: max |x[i] - round(x[i])| over all binary/integer variables.
  double max_integrality_violation{0.0};
};

SolutionDiagnostics analyze_solution(const solver::MIPModel& mip,
                                     const Eigen::VectorXd& x) {
  const solver::LPModel& lp = mip.linear_part;
  SolutionDiagnostics diag;
  const int n = static_cast<int>(lp.vars.size());
  diag.min_objective_coeff = n > 0 ? lp.c.minCoeff() : 0.0;
  if (x.size() != n) {
    diag.max_bound_violation = kInf;
    return diag;
  }
  for (int j = 0; j < n; ++j) {
    const double lb_violation = std::max(0.0, lp.vars[j].lb - x[j]);
    const double ub_violation = std::max(0.0, x[j] - lp.vars[j].ub);
    const double violation = std::max(lb_violation, ub_violation);
    if (violation > diag.max_bound_violation) {
      diag.max_bound_violation = violation;
      diag.max_bound_var = j;
      diag.max_bound_var_name = lp.vars[static_cast<size_t>(j)].name;
    }
  }
  diag.recomputed_objective = lp.c.dot(x);

  // ── Inequality residuals: A*x <= b (and optional row_lhs <= A*x) ──────────
  if (lp.A.rows() > 0 && lp.b.size() == lp.A.rows()) {
    const Eigen::VectorXd Ax = lp.A * x;
    // lp_has_row_lhs requires row_lhs.size() == A.rows() — check once outside loop.
    const bool has_lhs = (lp.row_lhs.size() == static_cast<Eigen::Index>(lp.A.rows()));
    for (int i = 0; i < lp.A.rows(); ++i) {
      // Upper-side violation: Ax[i] > b[i]
      diag.max_ineq_violation = std::max(diag.max_ineq_violation, Ax[i] - lp.b[i]);
      // Lower-side violation (range row): row_lhs[i] > Ax[i]
      if (has_lhs) {
        diag.max_ineq_violation = std::max(diag.max_ineq_violation, lp.row_lhs[i] - Ax[i]);
      }
    }
  }

  // ── Equality residuals: Aeq*x = beq ─────────────────────────────────────
  if (lp.Aeq.rows() > 0 && lp.beq.size() == lp.Aeq.rows()) {
    const Eigen::VectorXd res = lp.Aeq * x - lp.beq;
    diag.max_eq_violation = res.cwiseAbs().maxCoeff();
  }

  // ── Integrality: max |x[i] - round(x[i])| over binary + integer vars ─────
  auto check_integrality = [&](const std::vector<int>& idx_vec) {
    for (int i : idx_vec) {
      if (i >= 0 && i < n)
        diag.max_integrality_violation = std::max(
            diag.max_integrality_violation,
            std::abs(x[i] - std::round(x[i])));
    }
  };
  check_integrality(mip.binary_idx);
  check_integrality(mip.integer_idx);

  return diag;
}

int count_islands(const std::vector<BranchData>& branches,
                  const std::vector<int>& closed_branch_ids,
                  const std::vector<int>& energized) {
  if (energized.empty()) return 0;
  const int n = static_cast<int>(energized.size());
  std::vector<std::vector<int>> adj(n);
  for (size_t bi = 0; bi < branches.size(); ++bi) {
    if (!closed_branch_ids[bi]) continue;
    const auto& br = branches[bi];
    if (!energized[static_cast<size_t>(br.from_pos)] || !energized[static_cast<size_t>(br.to_pos)]) continue;
    adj[static_cast<size_t>(br.from_pos)].push_back(br.to_pos);
    adj[static_cast<size_t>(br.to_pos)].push_back(br.from_pos);
  }
  std::vector<int> seen(n, 0);
  std::vector<int> stack;
  int comp = 0;
  for (int i = 0; i < n; ++i) {
    if (!energized[static_cast<size_t>(i)] || seen[static_cast<size_t>(i)]) continue;
    ++comp;
    stack.clear();
    stack.push_back(i);
    seen[static_cast<size_t>(i)] = 1;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      for (int v : adj[static_cast<size_t>(u)]) {
        if (seen[static_cast<size_t>(v)]) continue;
        seen[static_cast<size_t>(v)] = 1;
        stack.push_back(v);
      }
    }
  }
  return comp;
}

// ── AC-ONLY MODEL SCOPE ───────────────────────────────────────────────────────
// build_mip_skeleton() and run_distribution_resilience_mip_assessment() model
// the AC distribution network only (sys.ac.buses, sys.ac.branches).
// DC buses, DC branches, VSC converters, and DC loads are NOT represented in
// the MILP; their energy cannot be dispatched and their faults cannot be
// scheduled.  The restoration result therefore applies only to the AC side.
// Do not advertise this as a full hybrid AC/DC restoration MIP.
// ─────────────────────────────────────────────────────────────────────────────
BuildArtifacts build_mip_skeleton(const HybridPowerSystem& sys,
                                  const DistributionResilienceOptions& opts) {
  BuildArtifacts out;
  out.stats.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  out.stats.model_built = false;
  out.stats.formulation_notes =
      "Strict unexpected-fault restoration skeleton (AC network only): "
      "multi-period MILP with forest radiality, active-power LinDistFlow "
      "voltage envelopes, fixed storage energy dynamics, and time-space MESS "
      "routing arcs.  DC buses / branches / VSC not modelled.";

  const auto bus_pos = make_bus_pos_map(sys);
  const auto branch_pos = make_branch_pos_map(sys);
  const auto faults = build_faults(sys, opts, branch_pos);
  out.faults = faults;

  const int T = std::max(1, static_cast<int>(std::ceil(opts.horizon_hours / opts.time_step_hr)));
  const int n_bus = static_cast<int>(sys.ac.buses.size());
  const int n_branch = static_cast<int>(sys.ac.branches.size());
  const double dt = opts.time_step_hr;

  out.steps = T;
  out.buses.resize(static_cast<size_t>(n_bus));
  out.branches.resize(static_cast<size_t>(n_branch));
  out.demand_mw.assign(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(n_bus), 0.0));
  out.renewable_avail_mw.assign(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(n_bus), 0.0));
  out.branch_available.assign(static_cast<size_t>(T), std::vector<bool>(static_cast<size_t>(n_branch), true));

  double system_peak_demand_mw = 0.0;
  for (int i = 0; i < n_bus; ++i) {
    out.buses[static_cast<size_t>(i)].index = sys.ac.buses[static_cast<size_t>(i)].index;
    out.buses[static_cast<size_t>(i)].importance = std::max(1.0, sys.ac.buses[static_cast<size_t>(i)].importance);
    out.buses[static_cast<size_t>(i)].priority = priority_from_importance(out.buses[static_cast<size_t>(i)].importance);
  }

  const auto& load_prof = opts.load_profile;
  const auto& ren_prof = opts.renewable_profile;
  for (int t = 0; t < T; ++t) {
    const double hour = static_cast<double>(t) * dt;
    const double load_mult = sample_profile(load_prof, hour);
    const double ren_mult = sample_profile(ren_prof, hour);
    // P1b: DC-OPF formulation adds bus.pd_mw and ac.loads additively as demand;
    // the demand matrix must mirror the same convention.  Both sources are
    // always accumulated unconditionally so neither is silently omitted.
    for (int i = 0; i < n_bus; ++i) {
      const double demand = std::max(0.0, sys.ac.buses[static_cast<size_t>(i)].pd_mw * opts.load_scale_factor * load_mult);
      out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)] += demand;
    }
    for (const auto& ld : sys.ac.loads) {
        if (!ld.in_service) continue;
        const auto it = bus_pos.find(ld.bus);
        if (it == bus_pos.end()) continue;
        const int i = it->second;
        const double demand = std::max(0.0, ld.p_mw * std::max(0.0, ld.scaling) * opts.load_scale_factor * load_mult);
        out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)] += demand;
        switch (ld.priority) {
          case LoadPriority::Critical: out.buses[static_cast<size_t>(i)].priority = PriorityTier::Critical; break;
          case LoadPriority::High:
            if (out.buses[static_cast<size_t>(i)].priority != PriorityTier::Critical) out.buses[static_cast<size_t>(i)].priority = PriorityTier::High;
            break;
          case LoadPriority::Medium:
            if (out.buses[static_cast<size_t>(i)].priority == PriorityTier::Low) out.buses[static_cast<size_t>(i)].priority = PriorityTier::Medium;
            break;
          case LoadPriority::Low: break;
        }
    }
    // Track true system-wide peak demand: total demand across all buses at
    // time t, max'd over t.  Earlier this variable was named "peak" but only
    // tracked the largest *single-bus* load, which under-sizes default slack
    // capacities (set to 1.5 * peak below) on systems with many small loads.
    double system_demand_at_t = 0.0;
    for (int i = 0; i < n_bus; ++i) system_demand_at_t += out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
    system_peak_demand_mw = std::max(system_peak_demand_mw, system_demand_at_t);
    for (const auto& rg : sys.ac.renewable_gens) {
      if (!rg.in_service) continue;
      const auto it = bus_pos.find(rg.bus);
      if (it == bus_pos.end()) continue;
      out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, (rg.p_rated_mw > 0.0 ? rg.p_rated_mw : rg.p_mw) * ren_mult);
    }
    for (const auto& pv : sys.ac.pv_systems) {
      if (!pv.in_service) continue;
      const auto it = bus_pos.find(pv.bus);
      if (it == bus_pos.end()) continue;
      out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, (pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw) * ren_mult);
    }
  }

  for (int i = 0; i < n_bus; ++i) {
    auto& bus = out.buses[static_cast<size_t>(i)];
    if (sys.ac.buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) bus.base_source_available = true;
    bus.base_source_cap_mw = system_peak_demand_mw > 0.0 ? 1.5 * system_peak_demand_mw : 10.0;
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = bus_pos.find(eg.bus);
    if (it == bus_pos.end()) continue;
    out.buses[static_cast<size_t>(it->second)].base_source_available = true;
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const auto it = bus_pos.find(g.bus);
    if (it == bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max({0.0, g.pmax_mw, g.pg_mw});
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || g.is_slack || cap > kEps;
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const auto it = bus_pos.find(sg.bus);
    if (it == bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max(0.0, (sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_mw) * std::max(0.0, sg.scaling));
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || cap > kEps;
  }

  for (int b = 0; b < n_branch; ++b) {
    const auto& br = sys.ac.branches[static_cast<size_t>(b)];
    const auto it_f = bus_pos.find(br.from_bus);
    const auto it_t = bus_pos.find(br.to_bus);
    out.branches[static_cast<size_t>(b)] = {
        br.index != 0 ? br.index : b + 1,
        it_f != bus_pos.end() ? it_f->second : 0,
        it_t != bus_pos.end() ? it_t->second : 0,
        std::max(1e-4, std::abs(br.r_pu)),
        br.rate_a_mva > 1e-9 ? br.rate_a_mva : opts.mip.default_branch_rate_mva,
        br.in_service};
  }

  for (int t = 0; t < T; ++t) {
    const double hour = static_cast<double>(t) * dt;
    for (const auto& f : faults) {
      if (hour >= f.start_hr && hour < f.start_hr + f.repair_hr - kEps) {
        out.branch_available[static_cast<size_t>(t)][static_cast<size_t>(f.branch_pos)] = false;
      }
    }
  }

  for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!st.in_service) continue;
    const auto it = bus_pos.find(st.bus);
    if (it == bus_pos.end()) continue;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    out.fixed_storage.push_back({st.index != 0 ? st.index : static_cast<int>(i + 1),
                                 it->second,
                                 std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                                 std::max(0.0, e0),
                                 st.soc_min * st.e_rated_mwh,
                                 std::max(0.0, st.e_rated_mwh),
                                 st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                                 st.grid_forming});
  }
  for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
    const auto& st = sys.mobile_storage[i];
    if (!st.in_service) continue;
    const auto it = bus_pos.find(st.bus);
    if (it == bus_pos.end()) continue;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    out.mess.push_back({st.index != 0 ? st.index : static_cast<int>(i + 1),
                        it->second,
                        std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                        std::max(0.0, e0),
                        st.soc_min * st.e_rated_mwh,
                        std::max(0.0, st.e_rated_mwh),
                        st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                        std::max(0.0, st.e_consumption_mwh_km),
                        st.max_travel_distance_km > 0.0 ? st.max_travel_distance_km : kInf});
  }

  const Adj transport_graph = build_transport_graph(sys, opts, bus_pos);
  std::vector<std::vector<double>> distance_matrix(static_cast<size_t>(n_bus), std::vector<double>(static_cast<size_t>(n_bus), kInf));
  for (int i = 0; i < n_bus; ++i) distance_matrix[static_cast<size_t>(i)] = shortest_path_distances(transport_graph, i);

  out.idx.T = T;
  out.idx.n_bus = n_bus;
  out.idx.n_branch = n_branch;
  out.idx.n_fixed_storage = static_cast<int>(out.fixed_storage.size());
  out.idx.n_mess = static_cast<int>(out.mess.size());

  auto& milp = out.model;
  auto& lp = milp.linear_part;
  lp.sense = solver::Sense::Minimize;

  std::vector<double> c;
  std::vector<solver::VariableMeta> vars;

  auto add_var = [&](solver::VarType type, double lb, double ub, const std::string& name, double obj) {
    const int idx = static_cast<int>(vars.size());
    vars.push_back({type, lb, ub, name});
    c.push_back(obj);
    if (type == solver::VarType::Binary) milp.binary_idx.push_back(idx);
    else if (type == solver::VarType::Integer) milp.integer_idx.push_back(idx);
    return idx;
  };

  const double vmin2 = opts.mip.v_min_pu * opts.mip.v_min_pu;
  const double vmax2 = opts.mip.v_max_pu * opts.mip.v_max_pu;
  const double commodity_cap = static_cast<double>(n_bus);

  out.idx.z.reserve(static_cast<size_t>(T * n_branch));
  out.idx.y_on.reserve(static_cast<size_t>(T * n_branch));
  out.idx.y_off.reserve(static_cast<size_t>(T * n_branch));
  out.idx.p_branch.reserve(static_cast<size_t>(T * n_branch));
  out.idx.f_branch.reserve(static_cast<size_t>(T * n_branch));
  out.idx.v_bus.reserve(static_cast<size_t>(T * n_bus));
  out.idx.u_bus.reserve(static_cast<size_t>(T * n_bus));
  out.idx.root_bus.reserve(static_cast<size_t>(T * n_bus));
  out.idx.root_base.reserve(static_cast<size_t>(T * n_bus));
  out.idx.root_flow.reserve(static_cast<size_t>(T * n_bus));
  out.idx.p_source.reserve(static_cast<size_t>(T * n_bus));
  out.idx.shed.reserve(static_cast<size_t>(T * n_bus));
  out.idx.renewable_used.reserve(static_cast<size_t>(T * n_bus));
  out.idx.gen_dispatch.reserve(static_cast<size_t>(T * n_bus));
  out.idx.fs_dis.reserve(static_cast<size_t>(T * out.fixed_storage.size()));
  out.idx.fs_e.reserve(static_cast<size_t>(T * out.fixed_storage.size()));
  out.idx.fs_root.reserve(static_cast<size_t>(T * out.fixed_storage.size()));
  out.idx.mess_e.reserve(static_cast<size_t>(T * out.mess.size()));
  out.idx.mess_x.reserve(static_cast<size_t>(T * out.mess.size() * n_bus));
  out.idx.mess_root.reserve(static_cast<size_t>(T * out.mess.size() * n_bus));
  out.idx.mess_dis.reserve(static_cast<size_t>(T * out.mess.size() * n_bus));
  out.idx.mess_arc_var.resize(out.mess.size());

  for (int t = 0; t < T; ++t) {
    for (int b = 0; b < n_branch; ++b) {
      const bool available = out.branch_available[static_cast<size_t>(t)][static_cast<size_t>(b)];
      double z_lb = 0.0;
      double z_ub = available ? 1.0 : 0.0;
      if (!opts.allow_reconfiguration) {
        const double fixed = out.branches[static_cast<size_t>(b)].initial_closed && available ? 1.0 : 0.0;
        z_lb = fixed;
        z_ub = fixed;
      }
      out.idx.z.push_back(add_var(solver::VarType::Binary, z_lb, z_ub,
                                  "z_" + std::to_string(b) + "_" + std::to_string(t), 0.0));
      out.idx.y_on.push_back(add_var(solver::VarType::Binary, 0.0, 1.0,
                                     "yon_" + std::to_string(b) + "_" + std::to_string(t), opts.mip.switching_cost));
      out.idx.y_off.push_back(add_var(solver::VarType::Binary, 0.0, 1.0,
                                      "yoff_" + std::to_string(b) + "_" + std::to_string(t), opts.mip.switching_cost));
      const double rate = out.branches[static_cast<size_t>(b)].rate_mw;
      out.idx.p_branch.push_back(add_var(solver::VarType::Continuous, -rate, rate,
                                         "pb_" + std::to_string(b) + "_" + std::to_string(t), 0.0));
      out.idx.f_branch.push_back(add_var(solver::VarType::Continuous, -commodity_cap, commodity_cap,
                                         "fb_" + std::to_string(b) + "_" + std::to_string(t), 0.0));
    }
    for (int i = 0; i < n_bus; ++i) {
      out.idx.v_bus.push_back(add_var(solver::VarType::Continuous, 0.0, vmax2,
                                      "v_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.u_bus.push_back(add_var(solver::VarType::Binary, 0.0, 1.0,
                                      "u_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.root_bus.push_back(add_var(solver::VarType::Binary, 0.0, 1.0,
                                         "rbus_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.root_base.push_back(add_var(solver::VarType::Binary,
                                          0.0,
                                          out.buses[static_cast<size_t>(i)].base_source_available ? 1.0 : 0.0,
                                          "rbase_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.root_flow.push_back(add_var(solver::VarType::Continuous, 0.0, commodity_cap,
                                          "rflow_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.p_source.push_back(add_var(solver::VarType::Continuous, 0.0, out.buses[static_cast<size_t>(i)].base_source_cap_mw,
                                         "psrc_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.shed.push_back(add_var(solver::VarType::Continuous,
                                     0.0,
                                     out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)],
                                     "shed_" + std::to_string(i) + "_" + std::to_string(t),
                                     penalty_from_priority(out.buses[static_cast<size_t>(i)].priority, opts.mip) * dt));
      out.idx.renewable_used.push_back(add_var(solver::VarType::Continuous,
                                               0.0,
                                               out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(i)],
                                               "ren_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      out.idx.gen_dispatch.push_back(add_var(solver::VarType::Continuous,
                                             0.0,
                                             out.buses[static_cast<size_t>(i)].dispatchable_gen_cap_mw,
                                             "gen_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
    }
  }

  for (int t = 0; t < T; ++t) {
    for (size_t s = 0; s < out.fixed_storage.size(); ++s) {
      const auto& st = out.fixed_storage[s];
      out.idx.fs_dis.push_back(add_var(solver::VarType::Continuous, 0.0, st.pmax_mw,
                                       "fsdis_" + std::to_string(s) + "_" + std::to_string(t), 0.0));
      out.idx.fs_e.push_back(add_var(solver::VarType::Continuous, st.e_min_mwh, st.e_max_mwh,
                                     "fse_" + std::to_string(s) + "_" + std::to_string(t), 0.0));
      out.idx.fs_root.push_back(add_var(solver::VarType::Binary,
                                        0.0,
                                        st.grid_forming && opts.mip.allow_grid_forming_storage_roots ? 1.0 : 0.0,
                                        "fsroot_" + std::to_string(s) + "_" + std::to_string(t), 0.0));
    }
  }

  std::vector<std::vector<ArcDef>> mess_arcs(out.mess.size());
  for (size_t m = 0; m < out.mess.size(); ++m) {
    for (int t = 0; t < T; ++t) {
      out.idx.mess_e.push_back(add_var(solver::VarType::Continuous,
                                       out.mess[m].e_min_mwh,
                                       out.mess[m].e_max_mwh,
                                       "me_" + std::to_string(m) + "_" + std::to_string(t), 0.0));
      for (int i = 0; i < n_bus; ++i) {
        out.idx.mess_x.push_back(add_var(solver::VarType::Binary, 0.0, 1.0,
                                         "mx_" + std::to_string(m) + "_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
        out.idx.mess_root.push_back(add_var(solver::VarType::Binary,
                                            0.0,
                                            opts.mip.allow_mess_black_start ? 1.0 : 0.0,
                                            "mr_" + std::to_string(m) + "_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
        out.idx.mess_dis.push_back(add_var(solver::VarType::Continuous,
                                           0.0,
                                           opts.allow_mess_dispatch ? out.mess[m].pmax_mw : 0.0,
                                           "md_" + std::to_string(m) + "_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
      }
    }
    if (T <= 1) continue;
    for (int t = 0; t < T - 1; ++t) {
      for (int i = 0; i < n_bus; ++i) {
        mess_arcs[m].push_back({i, i, t, t + 1, 0.0, 0.0, true});
        for (int j = 0; j < n_bus; ++j) {
          if (i == j || !opts.allow_mess_dispatch) continue;
          const double dist = distance_matrix[static_cast<size_t>(i)][static_cast<size_t>(j)];
          if (!std::isfinite(dist) || dist <= kEps || dist > out.mess[m].max_travel_distance_km + kEps) continue;
          const int steps_needed = std::max(1, static_cast<int>(std::ceil((dist / std::max(opts.mess_travel_speed_kmph, 1.0)) / dt)));
          const int arrive = t + steps_needed;
          if (arrive >= T) continue;
          mess_arcs[m].push_back({i, j, t, arrive, dist, dist * out.mess[m].e_travel_mwh_km, false});
        }
      }
    }
    out.idx.mess_arc_var[m].reserve(mess_arcs[m].size());
    for (size_t a = 0; a < mess_arcs[m].size(); ++a) {
      out.idx.mess_arc_var[m].push_back(add_var(solver::VarType::Binary,
                                                0.0,
                                                1.0,
                                                "ma_" + std::to_string(m) + "_" + std::to_string(a),
                                                mess_arcs[m][a].is_stay ? 0.0 : opts.mip.mess_travel_cost_per_km * mess_arcs[m][a].distance_km));
    }
  }
  out.mess_arcs = mess_arcs;

  lp.vars = vars;
  lp.c = Eigen::VectorXd::Zero(static_cast<int>(c.size()));
  for (size_t i = 0; i < c.size(); ++i) lp.c[static_cast<Eigen::Index>(i)] = c[i];

  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  std::vector<double> beq_vals;
  std::vector<double> b_vals;

  auto add_eq = [&](const std::vector<std::pair<int, double>>& terms, double rhs) {
    const int row = static_cast<int>(beq_vals.size());
    for (const auto& [col, val] : terms) {
      if (std::abs(val) > kEps) eq_trips.emplace_back(row, col, val);
    }
    beq_vals.push_back(rhs);
  };
  auto add_le = [&](const std::vector<std::pair<int, double>>& terms, double rhs) {
    const int row = static_cast<int>(b_vals.size());
    for (const auto& [col, val] : terms) {
      if (std::abs(val) > kEps) ineq_trips.emplace_back(row, col, val);
    }
    b_vals.push_back(rhs);
  };

  for (int t = 0; t < T; ++t) {
    for (int b = 0; b < n_branch; ++b) {
      const double z_prev = (t == 0) ? (out.branches[static_cast<size_t>(b)].initial_closed ? 1.0 : 0.0)
                                     : 0.0;
      if (t == 0) {
        add_eq({{out.idx.bt(b, t), 1.0}, {out.idx.yon(b, t), -1.0}, {out.idx.yoff(b, t), 1.0}}, z_prev);
      } else {
        add_eq({{out.idx.bt(b, t), 1.0}, {out.idx.bt(b, t - 1), -1.0}, {out.idx.yon(b, t), -1.0}, {out.idx.yoff(b, t), 1.0}}, 0.0);
      }
      add_le({{out.idx.yon(b, t), 1.0}, {out.idx.yoff(b, t), 1.0}}, 1.0);
      const double rate = out.branches[static_cast<size_t>(b)].rate_mw;
      add_le({{out.idx.pb(b, t), 1.0}, {out.idx.bt(b, t), -rate}}, 0.0);
      add_le({{out.idx.pb(b, t), -1.0}, {out.idx.bt(b, t), -rate}}, 0.0);
      add_le({{out.idx.fb(b, t), 1.0}, {out.idx.bt(b, t), -commodity_cap}}, 0.0);
      add_le({{out.idx.fb(b, t), -1.0}, {out.idx.bt(b, t), -commodity_cap}}, 0.0);

      const auto& br = out.branches[static_cast<size_t>(b)];
      add_le({{out.idx.bt(b, t), 1.0}, {out.idx.ub(br.from_pos, t), -1.0}}, 0.0);
      add_le({{out.idx.bt(b, t), 1.0}, {out.idx.ub(br.to_pos, t), -1.0}}, 0.0);

      add_le({{out.idx.vb(br.to_pos, t), 1.0},
              {out.idx.vb(br.from_pos, t), -1.0},
              {out.idx.pb(b, t), 2.0 * br.r_pu},
              {out.idx.bt(b, t), opts.mip.big_m_voltage}},
             opts.mip.big_m_voltage);
      add_le({{out.idx.vb(br.to_pos, t), -1.0},
              {out.idx.vb(br.from_pos, t), 1.0},
              {out.idx.pb(b, t), -2.0 * br.r_pu},
              {out.idx.bt(b, t), opts.mip.big_m_voltage}},
             opts.mip.big_m_voltage);
    }

    for (int i = 0; i < n_bus; ++i) {
      std::vector<std::pair<int, double>> tree_terms;
      for (int b = 0; b < n_branch; ++b) tree_terms.push_back({out.idx.bt(b, t), 1.0});
      for (int j = 0; j < n_bus; ++j) tree_terms.push_back({out.idx.rb(j, t), 1.0});
      for (int j = 0; j < n_bus; ++j) tree_terms.push_back({out.idx.ub(j, t), -1.0});
      if (i == 0) add_eq(tree_terms, 0.0);

      std::vector<std::pair<int, double>> power_terms{{out.idx.psrc(i, t), 1.0},
                                                      {out.idx.gen(i, t), 1.0},
                                                      {out.idx.ren(i, t), 1.0},
                                                      {out.idx.shedv(i, t), 1.0}};
      double rhs = out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
      for (int b = 0; b < n_branch; ++b) {
        const auto& br = out.branches[static_cast<size_t>(b)];
        if (br.to_pos == i) power_terms.push_back({out.idx.pb(b, t), 1.0});
        if (br.from_pos == i) power_terms.push_back({out.idx.pb(b, t), -1.0});
      }
      for (size_t s = 0; s < out.fixed_storage.size(); ++s) {
        if (out.fixed_storage[s].bus_pos == i) power_terms.push_back({out.idx.fsd(static_cast<int>(s), t), 1.0});
      }
      for (size_t m = 0; m < out.mess.size(); ++m) {
        power_terms.push_back({out.idx.md(static_cast<int>(m), i, t), 1.0});
      }
      add_eq(power_terms, rhs);

      std::vector<std::pair<int, double>> commodity_terms{{out.idx.rflow(i, t), 1.0}, {out.idx.ub(i, t), -1.0}};
      for (int b = 0; b < n_branch; ++b) {
        const auto& br = out.branches[static_cast<size_t>(b)];
        if (br.to_pos == i) commodity_terms.push_back({out.idx.fb(b, t), 1.0});
        if (br.from_pos == i) commodity_terms.push_back({out.idx.fb(b, t), -1.0});
      }
      add_eq(commodity_terms, 0.0);

      std::vector<std::pair<int, double>> energize_terms{{out.idx.ub(i, t), 1.0}, {out.idx.rb(i, t), -1.0}};
      for (int b = 0; b < n_branch; ++b) {
        const auto& br = out.branches[static_cast<size_t>(b)];
        if (br.from_pos == i || br.to_pos == i) energize_terms.push_back({out.idx.bt(b, t), -1.0});
      }
      add_le(energize_terms, 0.0);

      add_le({{out.idx.rb(i, t), 1.0}, {out.idx.ub(i, t), -1.0}}, 0.0);
      add_le({{out.idx.rbase(i, t), 1.0}, {out.idx.rb(i, t), -1.0}}, 0.0);
      add_le({{out.idx.psrc(i, t), 1.0}, {out.idx.rbase(i, t), -out.buses[static_cast<size_t>(i)].base_source_cap_mw}}, 0.0);
      add_le({{out.idx.gen(i, t), 1.0}, {out.idx.ub(i, t), -out.buses[static_cast<size_t>(i)].dispatchable_gen_cap_mw}}, 0.0);
      add_le({{out.idx.ren(i, t), 1.0}, {out.idx.ub(i, t), -out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(i)]}}, 0.0);
            add_le({{out.idx.shedv(i, t), -1.0}, {out.idx.ub(i, t), -out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)]}},
             -out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)]);
      add_le({{out.idx.rflow(i, t), 1.0}, {out.idx.rb(i, t), -commodity_cap}}, 0.0);
      add_le({{out.idx.vb(i, t), 1.0}, {out.idx.rb(i, t), opts.mip.big_m_voltage}}, 1.0 + opts.mip.big_m_voltage);
      add_le({{out.idx.vb(i, t), -1.0}, {out.idx.rb(i, t), opts.mip.big_m_voltage}}, -1.0 + opts.mip.big_m_voltage);
      add_le({{out.idx.vb(i, t), -1.0}, {out.idx.ub(i, t), vmin2}}, 0.0);

      std::vector<std::pair<int, double>> root_supply{{out.idx.rb(i, t), 1.0}, {out.idx.rbase(i, t), -1.0}};
      for (size_t s = 0; s < out.fixed_storage.size(); ++s) {
        if (out.fixed_storage[s].bus_pos == i) root_supply.push_back({out.idx.fsr(static_cast<int>(s), t), -1.0});
      }
      for (size_t m = 0; m < out.mess.size(); ++m) root_supply.push_back({out.idx.mr(static_cast<int>(m), i, t), -1.0});
      add_le(root_supply, 0.0);
    }
  }

  for (size_t s = 0; s < out.fixed_storage.size(); ++s) {
    const auto& st = out.fixed_storage[s];
    add_eq({{out.idx.fse(static_cast<int>(s), 0), 1.0}}, st.e_init_mwh);
    for (int t = 0; t < T; ++t) {
      add_le({{out.idx.fsd(static_cast<int>(s), t), 1.0}, {out.idx.ub(st.bus_pos, t), -st.pmax_mw}}, 0.0);
      add_le({{out.idx.fsr(static_cast<int>(s), t), 1.0}, {out.idx.rb(st.bus_pos, t), -1.0}}, 0.0);
      add_le({{out.idx.fse(static_cast<int>(s), t), -1.0}, {out.idx.fsr(static_cast<int>(s), t), 1e-3}}, -st.e_min_mwh);
      if (t > 0) {
        add_eq({{out.idx.fse(static_cast<int>(s), t), 1.0},
                {out.idx.fse(static_cast<int>(s), t - 1), -1.0},
                {out.idx.fsd(static_cast<int>(s), t - 1), dt / std::max(st.eta_discharge, 1e-6)}},
               0.0);
      }
    }
  }

  for (size_t m = 0; m < out.mess.size(); ++m) {
    const auto& ms = out.mess[m];
    add_eq({{out.idx.me(static_cast<int>(m), 0), 1.0}}, ms.e_init_mwh);
    for (int i = 0; i < n_bus; ++i) {
      add_eq({{out.idx.mx(static_cast<int>(m), i, 0), 1.0}}, i == ms.init_bus_pos ? 1.0 : 0.0);
    }
    for (int t = 0; t < T; ++t) {
      std::vector<std::pair<int, double>> unique_loc;
      for (int i = 0; i < n_bus; ++i) unique_loc.push_back({out.idx.mx(static_cast<int>(m), i, t), 1.0});
      add_eq(unique_loc, 1.0);

      std::vector<std::pair<int, double>> root_one;
      for (int i = 0; i < n_bus; ++i) {
        add_le({{out.idx.md(static_cast<int>(m), i, t), 1.0}, {out.idx.mx(static_cast<int>(m), i, t), -ms.pmax_mw}}, 0.0);
        add_le({{out.idx.mr(static_cast<int>(m), i, t), 1.0}, {out.idx.mx(static_cast<int>(m), i, t), -1.0}}, 0.0);
        add_le({{out.idx.mr(static_cast<int>(m), i, t), 1.0}, {out.idx.rb(i, t), -1.0}}, 0.0);
        root_one.push_back({out.idx.mr(static_cast<int>(m), i, t), 1.0});
      }
      add_le(root_one, 1.0);
      std::vector<std::pair<int, double>> energy_root{{out.idx.me(static_cast<int>(m), t), -1.0}};
      for (int i = 0; i < n_bus; ++i) energy_root.push_back({out.idx.mr(static_cast<int>(m), i, t), 1e-3});
      add_le(energy_root, -ms.e_min_mwh);

      if (t > 0) {
        std::vector<std::pair<int, double>> eterms{{out.idx.me(static_cast<int>(m), t), 1.0}, {out.idx.me(static_cast<int>(m), t - 1), -1.0}};
        for (int i = 0; i < n_bus; ++i) eterms.push_back({out.idx.md(static_cast<int>(m), i, t - 1), dt / std::max(ms.eta_discharge, 1e-6)});
        for (size_t a = 0; a < mess_arcs[m].size(); ++a) {
          if (mess_arcs[m][a].depart_t == t - 1) eterms.push_back({out.idx.mess_arc_var[m][a], mess_arcs[m][a].travel_mwh});
        }
        add_eq(eterms, 0.0);
      }
    }

    if (T <= 1) continue;
    for (int t = 0; t < T - 1; ++t) {
      for (int i = 0; i < n_bus; ++i) {
        std::vector<std::pair<int, double>> depart{{out.idx.mx(static_cast<int>(m), i, t), -1.0}};
        for (size_t a = 0; a < mess_arcs[m].size(); ++a) {
          if (mess_arcs[m][a].depart_t == t && mess_arcs[m][a].from_pos == i) {
            depart.push_back({out.idx.mess_arc_var[m][a], 1.0});
          }
        }
        add_eq(depart, 0.0);
      }
    }
    for (int t = 1; t < T; ++t) {
      for (int i = 0; i < n_bus; ++i) {
        std::vector<std::pair<int, double>> arrive{{out.idx.mx(static_cast<int>(m), i, t), -1.0}};
        for (size_t a = 0; a < mess_arcs[m].size(); ++a) {
          if (mess_arcs[m][a].arrive_t == t && mess_arcs[m][a].to_pos == i) {
            arrive.push_back({out.idx.mess_arc_var[m][a], 1.0});
          }
        }
        add_eq(arrive, 0.0);
      }
    }
  }

  lp.Aeq.resize(static_cast<int>(beq_vals.size()), static_cast<int>(vars.size()));
  lp.beq = Eigen::VectorXd::Zero(static_cast<int>(beq_vals.size()));
  for (size_t i = 0; i < beq_vals.size(); ++i) lp.beq[static_cast<Eigen::Index>(i)] = beq_vals[i];
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();

  lp.A.resize(static_cast<int>(b_vals.size()), static_cast<int>(vars.size()));
  lp.b = Eigen::VectorXd::Zero(static_cast<int>(b_vals.size()));
  for (size_t i = 0; i < b_vals.size(); ++i) lp.b[static_cast<Eigen::Index>(i)] = b_vals[i];
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();

  out.stats.num_variables = static_cast<int>(vars.size());
  out.stats.num_binary_variables = static_cast<int>(milp.binary_idx.size());
  out.stats.num_integer_variables = static_cast<int>(milp.integer_idx.size());
  out.stats.num_eq_constraints = static_cast<int>(beq_vals.size());
  out.stats.num_ineq_constraints = static_cast<int>(b_vals.size());
  out.stats.model_built = true;
  return out;
}

}  // namespace

DistributionResilienceResult run_distribution_resilience_mip_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts) {
  DistributionResilienceResult result;
  result.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  result.model_stats.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  if (opts.horizon_hours <= 0 || opts.time_step_hr <= 0.0) {
    result.feasible = false;
    result.status = "Invalid horizon or timestep for strict resilience MIP";
    return result;
  }
  if (sys.ac.buses.empty() || sys.ac.branches.empty()) {
    result.feasible = false;
    result.status = "System has no AC distribution network for strict resilience MIP";
    return result;
  }

  auto built = build_mip_skeleton(sys, opts);
  result.model_stats = built.stats;
  for (const auto& f : built.faults) result.fault_sequence.push_back({ResilienceBranchKind::AC, f.branch_index, f.start_hr, f.repair_hr, f.name});

  solver::BCOptions bc_opts;
  bc_opts.time_limit_sec = static_cast<double>(opts.mip.max_time_s);
  bc_opts.max_nodes = opts.mip.max_nodes;
  bc_opts.gap_tol = opts.mip.mip_gap;
  if (opts.mip.num_threads > 0) {
    bc_opts.num_threads = opts.mip.num_threads;
  } else {
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (hw >= 4) {
      bc_opts.num_threads = std::min(hw, 8);
    }
  }
  // Strict resilience MIPs are large, binary-heavy, and structurally close to
  // network restoration / routing hybrids. Use a stronger-than-default native
  // profile, but keep it deliberately moderate so run-to-run tree quality is
  // more stable than the previous aggressively tuned experiment.
  bc_opts.cuts = solver::CutType::All;
  // Cut pipeline baseline. On this LinDistFlow + big-M restoration
  // structure, gap_closed_by_cuts is near zero — adding more rounds or
  // deeper tree cuts just steals time from the primal polishing phase
  // (root LNS + feasibility pump) without tightening the dual bound.
  bc_opts.root_cut_rounds = 8;
  bc_opts.root_cut_max_stalls = 3;
  bc_opts.root_cut_stall_tol = 1e-5;
  bc_opts.cuts_per_round = 32;
  bc_opts.max_cut_depth = 2;
  bc_opts.branching = solver::BranchingStrategy::Pseudocost;
  bc_opts.node_sel = to_bc_node_selection(opts.mip.native_node_selection);
  bc_opts.use_feasibility_pump = true;
  // Enable LNS (RINS-style root sub-MIP) — without it, the tree search
  // has no primal-polishing path after the feasibility pump, and the
  // incumbent stagnates well above Gurobi's optimum.
  bc_opts.enable_lns = true;
  bc_opts.probe_reliability = 3;
  bc_opts.probe_max_candidates = 4;
  bc_opts.max_probe_vars = 64;
  bc_opts.pool_cut_depth_limit = 3;
  bc_opts.pool_cut_row_threshold = 1500;
  bc_opts.cut_pool_max_size = 3000;
  bc_opts.solution_pool_size = 12;
  bc_opts.bound_propagation_rounds = 4;
  bc_opts.stall_node_window = 1500;
  bc_opts.stall_gap_min_improvement = 0.02;
  bc_opts.verbose = opts.mip.verbose;
  // A/B gate: enable CGLP disjunctive cuts when the caller requests it.
  // Has no effect unless solver == Native (other adapters ignore BCOptions).
  bc_opts.enable_cglp_cuts = opts.mip.enable_cglp_cuts;

  engine::SolveResult solve_result;
  switch (opts.mip.solver) {
    case DistributionResilienceMIPSolver::Native: {
      engine::NativeBranchAndCutAdapter adapter(bc_opts);
      solve_result = adapter.solve_milp(built.model);
      break;
    }
    case DistributionResilienceMIPSolver::HiGHS: {
      engine::HighsAdapter adapter;
      solve_result = adapter.solve_milp(built.model);
      break;
    }
    case DistributionResilienceMIPSolver::Gurobi: {
      engine::GurobiAdapter adapter;
      solve_result = adapter.solve_milp(built.model);
      break;
    }
  }

  const auto solution_diag = analyze_solution(built.model, solve_result.x);
  result.model_stats.model_solved = true;
  result.model_stats.objective_value = solution_diag.recomputed_objective;
  result.model_stats.mip_gap = solve_result.stats.mip_gap;
  result.model_stats.runtime_sec = solve_result.stats.runtime_sec;
  result.model_stats.solver_name = resilience_mip_solver_name(opts.mip.solver, bc_opts.num_threads);
  result.model_stats.solver_status = solve_result.stats.status;
  result.model_stats.cglp_cuts_added = solve_result.stats.cglp_cuts_added;

  // Post-solve capability flags.  AC-only model_scope is hard-coded because
  // the MIP skeleton does not represent DC components; flipping this string
  // requires actually adding DC/VSC variables and constraints.  The
  // `mip_solved_to_proven_optimum` flag is set only when the solver reports
  // a closed optimality gap; non-zero gap means the incumbent is feasible
  // but its global optimality is unproven.
  result.model_stats.model_scope = "ac-only-lindistflow";
  result.model_stats.validity = DistributionResilienceModelStats::ValidityFlags{};
  result.model_stats.validity.mip_gap_within_tolerance =
      solve_result.stats.success && solve_result.stats.mip_gap <= opts.mip.mip_gap + 1.0e-9;

  constexpr double kFeasTol = 1.0e-6;
  const bool size_ok = solve_result.x.size() == built.model.linear_part.c.size();
  const bool bounds_ok       = solution_diag.max_bound_violation      <= kFeasTol;
  const bool ineq_ok         = solution_diag.max_ineq_violation        <= kFeasTol;
  const bool eq_ok           = solution_diag.max_eq_violation          <= kFeasTol;
  const bool integrality_ok  = solution_diag.max_integrality_violation <= kFeasTol;
  const bool nonnegative_objective_ok =
      solution_diag.min_objective_coeff < -1.0e-9 || solution_diag.recomputed_objective >= -1.0e-6;
  result.feasible = solve_result.stats.success && size_ok && bounds_ok &&
                    ineq_ok && eq_ok && integrality_ok && nonnegative_objective_ok;
  if (!result.feasible) {
    std::ostringstream status;
    status << "Strict resilience MIP skeleton built but solver returned an invalid incumbent";
    if (!solve_result.stats.success) {
      status << " (solver reported no feasible incumbent)";
    } else if (!size_ok) {
      status << " (solution size mismatch)";
    } else if (!bounds_ok) {
      status << " (max bound violation=" << solution_diag.max_bound_violation
             << " at var " << solution_diag.max_bound_var;
      if (!solution_diag.max_bound_var_name.empty()) {
        status << " [" << solution_diag.max_bound_var_name << "]";
      }
      status << ")";
    } else if (!ineq_ok) {
      status << " (max inequality residual=" << solution_diag.max_ineq_violation << ")";
    } else if (!eq_ok) {
      status << " (max equality residual=" << solution_diag.max_eq_violation << ")";
    } else if (!integrality_ok) {
      status << " (max integrality violation=" << solution_diag.max_integrality_violation
             << " — binary/integer variable not rounded)";
    } else if (!nonnegative_objective_ok) {
      status << " (negative objective " << solution_diag.recomputed_objective
             << " despite nonnegative objective coefficients)";
    }
    result.status = status.str() + ": " + solve_result.stats.status;
    return result;
  }

  const auto& x = solve_result.x;
  const auto& idx = built.idx;
  const int T = built.steps;
  result.steps.reserve(static_cast<size_t>(T));
  result.total_demand_mwh = 0.0;
  result.total_served_mwh = 0.0;
  result.total_shed_mwh = 0.0;
  result.weighted_unserved_mwh = 0.0;
  result.mess_energy_delivered_mwh = 0.0;
  result.mess_travel_distance_km = 0.0;
  result.total_switch_actions = 0;

  for (int t = 0; t < T; ++t) {
    DistributionResilienceStepResult sr;
    sr.step_index = t;
    sr.hour = static_cast<double>(t) * opts.time_step_hr;
    sr.shed_by_priority.assign(4, 0.0);

    std::vector<int> energized(static_cast<size_t>(built.buses.size()), 0);
    std::vector<int> closed(static_cast<size_t>(built.branches.size()), 0);
    for (size_t i = 0; i < built.buses.size(); ++i) {
      const double demand = built.demand_mw[static_cast<size_t>(t)][i];
      const double shed = std::max(0.0, x[idx.shedv(static_cast<int>(i), t)]);
      const double served = std::max(0.0, demand - shed);
      sr.total_demand_mw += demand;
      sr.shed_mw += shed;
      sr.served_mw += served;
      energized[i] = x[idx.ub(static_cast<int>(i), t)] > 0.5 ? 1 : 0;
      const int pidx = static_cast<int>(built.buses[i].priority);
      sr.shed_by_priority[static_cast<size_t>(pidx)] += shed;
      result.weighted_unserved_mwh += penalty_from_priority(built.buses[i].priority, opts.mip) * shed * opts.time_step_hr;
      sr.total_res_mw += std::max(0.0, x[idx.ren(static_cast<int>(i), t)]);
    }
    for (size_t b = 0; b < built.branches.size(); ++b) {
      const bool is_closed = x[idx.bt(static_cast<int>(b), t)] > 0.5;
      closed[b] = is_closed ? 1 : 0;
      if (is_closed) {
        if (!built.branches[b].initial_closed) sr.closed_tie_branch_ids.push_back(built.branches[b].index);
      } else {
        sr.open_ac_branch_ids.push_back(built.branches[b].index);
      }
      sr.switch_actions += static_cast<int>(std::lround(std::max(0.0, x[idx.yon(static_cast<int>(b), t)]) + std::max(0.0, x[idx.yoff(static_cast<int>(b), t)])));
    }
    sr.active_faults = 0;
    sr.repaired_faults = 0;
    for (const auto& f : built.faults) {
      const double hour = sr.hour;
      if (hour >= f.start_hr && hour < f.start_hr + f.repair_hr - kEps) ++sr.active_faults;
      if (hour >= f.start_hr + f.repair_hr - kEps) ++sr.repaired_faults;
    }
    sr.island_count = count_islands(built.branches, closed, energized);
    sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;

    for (size_t m = 0; m < built.mess.size(); ++m) {
      MESSStateStep ms;
      ms.storage_index = built.mess[m].index;
      ms.energy_mwh = std::max(0.0, x[idx.me(static_cast<int>(m), t)]);
      ms.soc = built.mess[m].e_max_mwh > kEps ? ms.energy_mwh / built.mess[m].e_max_mwh : 0.0;
      int bus_pos = built.mess[m].init_bus_pos;
      double best_x = -1.0;
      for (size_t i = 0; i < built.buses.size(); ++i) {
        const double xv = x[idx.mx(static_cast<int>(m), static_cast<int>(i), t)];
        if (xv > best_x) {
          best_x = xv;
          bus_pos = static_cast<int>(i);
        }
      }
      ms.bus = built.buses[static_cast<size_t>(bus_pos)].index;
      ms.target_bus = ms.bus;
      double dispatch = 0.0;
      for (size_t i = 0; i < built.buses.size(); ++i) dispatch += std::max(0.0, x[idx.md(static_cast<int>(m), static_cast<int>(i), t)]);
      ms.dispatch_mw = dispatch;
      ms.status = dispatch > kEps ? "Deployed" : "Stationary";
      if (t < T - 1) {
        for (size_t a = 0; a < built.idx.mess_arc_var[m].size() && a < built.mess_arcs[m].size(); ++a) {
          if (x[built.idx.mess_arc_var[m][a]] <= 0.5) continue;
          ms.target_bus = built.buses[static_cast<size_t>(built.mess_arcs[m][a].to_pos)].index;
          if (!built.mess_arcs[m][a].is_stay) {
            ms.status = "InTransit";
            ms.remaining_travel_hr = std::max(0.0, static_cast<double>(built.mess_arcs[m][a].arrive_t - built.mess_arcs[m][a].depart_t) * opts.time_step_hr);
            result.mess_travel_distance_km += built.mess_arcs[m][a].distance_km;
          }
          break;
        }
      }
      sr.mess_states.push_back(ms);
      result.mess_energy_delivered_mwh += dispatch * opts.time_step_hr;
    }

    result.total_switch_actions += sr.switch_actions;
    result.total_demand_mwh += sr.total_demand_mw * opts.time_step_hr;
    result.total_served_mwh += sr.served_mw * opts.time_step_hr;
    result.total_shed_mwh += sr.shed_mw * opts.time_step_hr;
    result.peak_shed_mw = std::max(result.peak_shed_mw, sr.shed_mw);
    result.steps.push_back(std::move(sr));
  }

  result.resilience_index = result.total_demand_mwh > kEps ? result.total_served_mwh / result.total_demand_mwh : 1.0;
  double avg_ratio = 0.0;
  for (const auto& step : result.steps) avg_ratio += step.restoration_ratio;
  result.avg_restoration_ratio = result.steps.empty() ? 1.0 : avg_ratio / static_cast<double>(result.steps.size());
  result.final_restoration_ratio = result.steps.empty() ? 1.0 : result.steps.back().restoration_ratio;
  result.total_repaired_faults = result.steps.empty() ? 0 : result.steps.back().repaired_faults;
  result.status = "Strict multi-period MIP/LinDistFlow restoration skeleton solved";
  return result;
}

}  // namespace hacdcpf::analysis

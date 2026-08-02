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
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/solver/branch_and_cut.hpp"

namespace hacdcpf::analysis {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;
// Tiny secondary objective terms for MESS use.  They only break ties between
// otherwise equivalent routes/dispatches; load-shed penalties are orders larger.
constexpr double kMessMoveTieBreakCost = 1e-4;
constexpr double kMessDispatchTieBreakCostPerMWh = 1e-4;

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

std::unordered_map<int, int> make_dc_bus_pos_map(const HybridPowerSystem& sys,
                                                  int offset) {
  std::unordered_map<int, int> out;
  out.reserve(sys.dc.buses.size());
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    out[sys.dc.buses[i].index] = offset + static_cast<int>(i);
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
  bool dc_side{false};
  bool base_source_available{false};
  double base_source_cap_mw{0.0};
  double dispatchable_gen_cap_mw{0.0};
  double renewable_cap_mw{0.0};
  double importance{1.0};
  PriorityTier priority{PriorityTier::Low};
};

struct BranchData {
  int index{0};
  int canonical_index{0};
  int pair_number{0};
  ResilienceBranchKind domain{ResilienceBranchKind::AC};
  std::string component_type{"ac_branch"};
  int from_pos{0};
  int to_pos{0};
  double r_pu{0.01};
  double rate_mw{10.0};
  bool initial_closed{true};
  bool enforce_voltage_drop{true};
};

struct FixedStorageData {
  int index{0};
  bool dc_side{false};
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
  int available_step{0};
  double pmax_mw{0.0};
  double e_init_mwh{0.0};
  double e_min_mwh{0.0};
  double e_max_mwh{0.0};
  double eta_discharge{1.0};
  double e_travel_mwh_km{0.0};
  double max_travel_distance_km{kInf};
  bool grid_forming{false};
};

struct FaultData {
  int branch_pos{0};
  int branch_index{0};
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  double start_hr{0.0};
  double repair_hr{0.0};
  std::string name;
};

int requested_branch_index(const DistributionResilienceFault& f) {
  return (f.branch_index == 0 && f.ac_branch_index != 0) ? f.ac_branch_index : f.branch_index;
}

using Adj = std::vector<std::vector<std::pair<int, double>>>;

Adj build_transport_graph(const HybridPowerSystem& sys,
                          const DistributionResilienceOptions& opts,
                          const std::unordered_map<int, int>& bus_pos,
                          int node_count) {
  Adj graph(static_cast<size_t>(node_count));
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

std::vector<FaultData> build_faults(const std::vector<BranchData>& branches,
                                    const DistributionResilienceOptions& opts) {
  std::vector<FaultData> out;
  if (!opts.faults.empty()) {
    out.reserve(opts.faults.size());
    for (const auto& f : opts.faults) {
      const int requested = requested_branch_index(f);
      const auto it = std::find_if(
          branches.begin(), branches.end(), [&](const BranchData& branch) {
            return branch.index == requested && branch.domain == f.branch_kind &&
                   (branch.component_type == "ac_branch" ||
                    branch.component_type == "dc_branch");
          });
      if (it == branches.end()) continue;
      const int branch_pos = static_cast<int>(std::distance(branches.begin(), it));
      out.push_back({branch_pos,
                     it->index,
                     it->domain,
                     f.outage_start_hr,
                     std::max(opts.time_step_hr, f.repair_duration_hr),
                     f.name.empty() ? it->component_type + "_" +
                                          std::to_string(it->index)
                                    : f.name});
    }
    return out;
  }
  std::vector<int> candidates;
  for (size_t i = 0; i < branches.size(); ++i) {
    const auto& branch = branches[i];
    if (branch.initial_closed &&
        (branch.component_type == "ac_branch" ||
         branch.component_type == "dc_branch")) {
      candidates.push_back(static_cast<int>(i));
    }
  }
  const int count = std::min(opts.default_fault_count, static_cast<int>(candidates.size()));
  for (int k = 0; k < count; ++k) {
    const int pos = candidates[static_cast<size_t>(k)];
    const auto& br = branches[static_cast<size_t>(pos)];
    out.push_back({pos,
                     br.index,
                     br.domain,
                   opts.auto_fault_start_hr + k * std::max(0.0, opts.auto_fault_stagger_hr),
                   std::max(opts.time_step_hr, opts.default_repair_time_hr),
                   br.component_type + "_" + std::to_string(br.index)});
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

DistributionResilienceResult make_no_fault_baseline_result(
    const BuildArtifacts& built,
    const DistributionResilienceOptions& opts) {
  DistributionResilienceResult result;
  result.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  result.model_stats = built.stats;
  result.model_stats.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  result.model_stats.model_built = true;
  result.model_stats.model_solved = true;
  result.model_stats.model_scope = "hybrid-acdc-restoration-milp";
  result.model_stats.solver_name = "no-fault-baseline";
  result.model_stats.solver_status =
      "No explicit faults; evaluated intact hybrid topology without MILP search";
  result.model_stats.validity.mip_gap_within_tolerance = true;
  result.model_stats.mip_gap = 0.0;
  result.feasible = true;
  result.status = "Strict multi-period MIP/LinDistFlow no-fault baseline evaluated without MILP search";

  const int T = built.steps;
  const int n_bus = static_cast<int>(built.buses.size());
  std::vector<int> closed(static_cast<size_t>(built.branches.size()), 0);
  std::vector<std::vector<int>> adj(static_cast<size_t>(n_bus));
  for (size_t b = 0; b < built.branches.size(); ++b) {
    const auto& br = built.branches[b];
    if (!br.initial_closed) continue;
    closed[b] = 1;
    adj[static_cast<size_t>(br.from_pos)].push_back(br.to_pos);
    adj[static_cast<size_t>(br.to_pos)].push_back(br.from_pos);
  }

  std::vector<int> comp(static_cast<size_t>(n_bus), -1);
  int comp_count = 0;
  for (int i = 0; i < n_bus; ++i) {
    if (comp[static_cast<size_t>(i)] >= 0) continue;
    std::vector<int> stack{i};
    comp[static_cast<size_t>(i)] = comp_count;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      for (int v : adj[static_cast<size_t>(u)]) {
        if (comp[static_cast<size_t>(v)] >= 0) continue;
        comp[static_cast<size_t>(v)] = comp_count;
        stack.push_back(v);
      }
    }
    ++comp_count;
  }

  result.steps.reserve(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    std::vector<double> comp_demand(static_cast<size_t>(comp_count), 0.0);
    std::vector<double> comp_supply(static_cast<size_t>(comp_count), 0.0);
    for (int i = 0; i < n_bus; ++i) {
      const int c = comp[static_cast<size_t>(i)];
      if (c < 0) continue;
      comp_demand[static_cast<size_t>(c)] += built.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
      comp_supply[static_cast<size_t>(c)] += built.buses[static_cast<size_t>(i)].base_source_cap_mw;
      comp_supply[static_cast<size_t>(c)] += built.buses[static_cast<size_t>(i)].dispatchable_gen_cap_mw;
      comp_supply[static_cast<size_t>(c)] += built.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
    }

    DistributionResilienceStepResult sr;
    sr.step_index = t;
    sr.hour = static_cast<double>(t) * opts.time_step_hr;
    sr.shed_by_priority.assign(4, 0.0);
    std::vector<int> energized(static_cast<size_t>(n_bus), 0);

    for (int i = 0; i < n_bus; ++i) {
      const int c = comp[static_cast<size_t>(i)];
      const double demand = built.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
      const double ratio = (c >= 0 && comp_demand[static_cast<size_t>(c)] > kEps)
          ? std::clamp(comp_supply[static_cast<size_t>(c)] / comp_demand[static_cast<size_t>(c)], 0.0, 1.0)
          : 1.0;
      const double served = demand * ratio;
      const double shed = std::max(0.0, demand - served);
      energized[static_cast<size_t>(i)] = ratio > kEps ? 1 : 0;

      sr.total_demand_mw += demand;
      sr.served_mw += served;
      sr.shed_mw += shed;
      sr.total_res_mw += built.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(i)];
      result.weighted_unserved_mwh +=
          penalty_from_priority(built.buses[static_cast<size_t>(i)].priority, opts.mip) * shed * opts.time_step_hr;
      const int pidx = static_cast<int>(built.buses[static_cast<size_t>(i)].priority);
      if (static_cast<size_t>(pidx) < sr.shed_by_priority.size()) {
        sr.shed_by_priority[static_cast<size_t>(pidx)] += shed;
      }

      sr.bus_supply_kind.push_back(
          built.buses[static_cast<size_t>(i)].dc_side ? "DC" : "AC");
      sr.bus_supply_index.push_back(built.buses[static_cast<size_t>(i)].index);
      sr.bus_supply_demand_mw.push_back(demand);
      sr.bus_supply_served_mw.push_back(served);
      sr.bus_supply_shed_mw.push_back(shed);
      sr.bus_supply_priority_tier.push_back(pidx);
      sr.bus_supply_importance.push_back(built.buses[static_cast<size_t>(i)].importance);
    }

    for (size_t b = 0; b < built.branches.size(); ++b) {
      const auto& branch = built.branches[b];
      if (!closed[b] && branch.component_type == "dc_branch") {
        sr.open_dc_branch_ids.push_back(branch.index);
      } else if (!closed[b] && branch.component_type == "ac_branch") {
        sr.open_ac_branch_ids.push_back(branch.index);
      }
      const std::string domain =
          branch.component_type == "vsc_converter" ||
                  branch.component_type == "lcc_converter"
              ? "AC-DC"
              : (branch.domain == ResilienceBranchKind::DC ? "DC" : "AC");
      sr.component_states.push_back(
          {branch.component_type,
           domain,
           branch.index,
           branch.canonical_index,
           branch.pair_number,
           built.buses[static_cast<size_t>(branch.from_pos)].index,
           built.buses[static_cast<size_t>(branch.to_pos)].index,
           true,
           closed[b] != 0,
           false,
           0.0});
    }
    sr.island_count = count_islands(built.branches, closed, energized);
    sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;

    for (const auto& m : built.mess) {
      MESSStateStep ms;
      ms.storage_index = m.index;
      ms.bus = built.buses[static_cast<size_t>(m.init_bus_pos)].index;
      ms.target_bus = ms.bus;
      ms.status = "Stationary";
      ms.energy_mwh = m.e_init_mwh;
      ms.soc = m.e_max_mwh > kEps ? std::clamp(m.e_init_mwh / m.e_max_mwh, 0.0, 1.0) : 0.0;
      ms.arrival_time_hr = sr.hour;
      sr.mess_states.push_back(ms);
    }

    result.total_demand_mwh += sr.total_demand_mw * opts.time_step_hr;
    result.total_served_mwh += sr.served_mw * opts.time_step_hr;
    result.total_shed_mwh += sr.shed_mw * opts.time_step_hr;
    result.peak_shed_mw = std::max(result.peak_shed_mw, sr.shed_mw);
    result.steps.push_back(std::move(sr));
  }

  result.resilience_index = result.total_demand_mwh > kEps
      ? result.total_served_mwh / result.total_demand_mwh
      : 1.0;
  double avg_ratio = 0.0;
  for (const auto& step : result.steps) avg_ratio += step.restoration_ratio;
  result.avg_restoration_ratio = result.steps.empty() ? 1.0 : avg_ratio / static_cast<double>(result.steps.size());
  result.final_restoration_ratio = result.steps.empty() ? 1.0 : result.steps.back().restoration_ratio;
  result.total_ens_mwh = result.total_shed_mwh;
  result.max_curtailment_mw = result.peak_shed_mw;
  result.completed = result.feasible;
  if (!result.steps.empty()) result.restoration_time_hr = result.steps.back().hour;
  return result;
}

// The strict model consumes a canonical rich-system projection. AC and DC
// nodes share only internal positions; all external lookups remain domain
// qualified. Converter edges carry bounded active power between domains.
BuildArtifacts build_mip_skeleton(const HybridPowerSystem& sys,
                                  const DistributionResilienceOptions& opts) {
  BuildArtifacts out;
  out.stats.model = DistributionResilienceModel::MultiPeriodMIPLinDistFlow;
  out.stats.model_built = false;
  out.stats.formulation_notes =
      "Strict hybrid AC/DC unexpected-fault restoration MILP: canonical rich-"
      "component projection, multi-period forest topology, active-power "
      "LinDistFlow-style AC/DC voltage envelopes, bounded VSC/LCC/DC-DC "
      "transfers, fixed AC/DC storage energy dynamics, and time-space MESS "
      "routing. Reactive power, converter losses, protection logic, and "
      "transient limits require downstream verification.";

  const auto bus_pos = make_bus_pos_map(sys);
  const int n_ac_bus = static_cast<int>(sys.ac.buses.size());
  const auto dc_bus_pos = make_dc_bus_pos_map(sys, n_ac_bus);

  const int T = std::max(1, static_cast<int>(std::ceil(opts.horizon_hours / opts.time_step_hr)));
  const int n_bus = n_ac_bus + static_cast<int>(sys.dc.buses.size());
  const double dt = opts.time_step_hr;

  out.steps = T;
  out.buses.resize(static_cast<size_t>(n_bus));
  out.demand_mw.assign(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(n_bus), 0.0));
  out.renewable_avail_mw.assign(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(n_bus), 0.0));

  double system_peak_demand_mw = 0.0;
  for (int i = 0; i < n_ac_bus; ++i) {
    out.buses[static_cast<size_t>(i)].index = sys.ac.buses[static_cast<size_t>(i)].index;
    out.buses[static_cast<size_t>(i)].importance = std::max(1.0, sys.ac.buses[static_cast<size_t>(i)].importance);
    out.buses[static_cast<size_t>(i)].priority = priority_from_importance(out.buses[static_cast<size_t>(i)].importance);
  }
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    auto& bus = out.buses[static_cast<size_t>(n_ac_bus) + i];
    bus.index = sys.dc.buses[i].index;
    bus.dc_side = true;
    bus.importance = std::max(1.0, sys.dc.buses[i].importance);
    bus.priority = priority_from_importance(bus.importance);
  }

  const auto& load_prof = opts.load_profile;
  const auto& ren_prof = opts.renewable_profile;
  const auto& pv_prof = opts.pv_profile.empty() ? ren_prof : opts.pv_profile;
  const auto& wind_prof = opts.wind_profile.empty() ? ren_prof : opts.wind_profile;
  for (int t = 0; t < T; ++t) {
    const double hour = static_cast<double>(t) * dt;
    const double load_mult = sample_profile(load_prof, hour);
    const double ren_mult = sample_profile(ren_prof, hour);
    const double pv_mult = sample_profile(pv_prof, hour);
    const double wind_mult = sample_profile(wind_prof, hour);
    // P1b: DC-OPF formulation adds bus.pd_mw and ac.loads additively as demand;
    // the demand matrix must mirror the same convention.  Both sources are
    // always accumulated unconditionally so neither is silently omitted.
    for (int i = 0; i < n_ac_bus; ++i) {
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
    for (const auto& station : sys.ac.charging_stations) {
      if (!station.in_service) continue;
      const auto it = bus_pos.find(station.bus);
      if (it == bus_pos.end()) continue;
      double demand = station.p_total_kw / 1000.0;
      if (demand <= kEps) {
        demand = station.max_power_kw * std::max(0.0, station.utilization_rate) /
                 1000.0;
      }
      out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, demand * opts.load_scale_factor * load_mult);
    }
    for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const int pos = n_ac_bus + static_cast<int>(i);
      out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(pos)] +=
          std::max(0.0, sys.dc.buses[i].pd_mw * opts.load_scale_factor * load_mult);
    }
    for (const auto& ld : sys.dc.loads) {
      if (!ld.in_service) continue;
      const auto it = dc_bus_pos.find(ld.bus);
      if (it == dc_bus_pos.end()) continue;
      out.demand_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, model::effective_load_p_mw(ld) *
                            opts.load_scale_factor * load_mult);
      switch (ld.priority) {
        case LoadPriority::Critical:
          out.buses[static_cast<size_t>(it->second)].priority = PriorityTier::Critical;
          break;
        case LoadPriority::High:
          if (out.buses[static_cast<size_t>(it->second)].priority != PriorityTier::Critical)
            out.buses[static_cast<size_t>(it->second)].priority = PriorityTier::High;
          break;
        case LoadPriority::Medium:
          if (out.buses[static_cast<size_t>(it->second)].priority == PriorityTier::Low)
            out.buses[static_cast<size_t>(it->second)].priority = PriorityTier::Medium;
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
      double mult = ren_mult;
      if (rg.type == RenewableType::Wind) mult = wind_mult;
      else if (rg.type == RenewableType::SolarPV || rg.type == RenewableType::SolarCSP) mult = pv_mult;
      out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, (rg.p_rated_mw > 0.0 ? rg.p_rated_mw : rg.p_mw) * mult);
    }
    for (const auto& pv : sys.ac.pv_systems) {
      if (!pv.in_service) continue;
      const auto it = bus_pos.find(pv.bus);
      if (it == bus_pos.end()) continue;
      out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, (pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw) * pv_mult);
    }
    for (const auto& pv : sys.dc.pv_arrays) {
      if (!pv.in_service) continue;
      const auto it = dc_bus_pos.find(pv.bus);
      if (it == dc_bus_pos.end()) continue;
      out.renewable_avail_mw[static_cast<size_t>(t)][static_cast<size_t>(it->second)] +=
          std::max(0.0, pv.p_set_mw * pv_mult);
    }
  }

  for (int i = 0; i < n_ac_bus; ++i) {
    auto& bus = out.buses[static_cast<size_t>(i)];
    if (sys.ac.buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      bus.base_source_available = true;
      bus.base_source_cap_mw =
          system_peak_demand_mw > 0.0 ? 1.5 * system_peak_demand_mw : 10.0;
    }
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = bus_pos.find(eg.bus);
    if (it == bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    bus.base_source_available = true;
    bus.base_source_cap_mw = std::max(
        bus.base_source_cap_mw,
        system_peak_demand_mw > 0.0 ? 1.5 * system_peak_demand_mw : 10.0);
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const auto it = bus_pos.find(g.bus);
    if (it == bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max({0.0, g.pmax_mw, g.pg_mw});
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || g.is_slack || cap > kEps;
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const auto it = bus_pos.find(sg.bus);
    if (it == bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max(0.0, (sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_mw) * std::max(0.0, sg.scaling));
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || cap > kEps;
  }

  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    auto& bus = out.buses[static_cast<size_t>(n_ac_bus) + i];
    if (sys.dc.buses[i].bus_type == DCBusType::DC_V) {
      bus.base_source_available = true;
      bus.base_source_cap_mw = system_peak_demand_mw > 0.0
                                   ? 1.5 * system_peak_demand_mw
                                   : 10.0;
    }
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const auto it = dc_bus_pos.find(sg.bus);
    if (it == dc_bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max(0.0, (sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_mw) *
                                         std::max(0.0, sg.scaling));
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || cap > kEps;
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const auto it = dc_bus_pos.find(sg.bus);
    if (it == dc_bus_pos.end()) continue;
    auto& bus = out.buses[static_cast<size_t>(it->second)];
    const double cap = std::max({0.0, sg.pmax_mw, sg.p_set_mw}) *
                       model::sanitize_scaling(sg.scaling);
    bus.dispatchable_gen_cap_mw += cap;
    bus.base_source_available = bus.base_source_available || cap > kEps;
  }

  std::unordered_map<int, BranchExpandEntry> ac_branch_origins;
  if (sys.branch_expand_map.has_value()) {
    for (const auto& entry : sys.branch_expand_map->entries) {
      ac_branch_origins[entry.branch_index] = entry;
    }
  }
  auto origin_component_type = [](BranchOriginType origin) {
    switch (origin) {
      case BranchOriginType::Transformer2W: return std::string{"transformer_2w"};
      case BranchOriginType::Transformer3W: return std::string{"transformer_3w"};
      case BranchOriginType::Switch: return std::string{"switch"};
      case BranchOriginType::CircuitBreaker: return std::string{"circuit_breaker"};
    }
    return std::string{"ac_branch"};
  };

  auto add_branch = [&](int index, int canonical_index, int pair_number,
                        ResilienceBranchKind domain,
                        std::string component_type, int from_pos, int to_pos,
                        double r_pu, double rate_mw, bool initial_closed,
                        bool enforce_voltage_drop) {
    if (from_pos < 0 || to_pos < 0 || from_pos == to_pos) return;
    out.branches.push_back({index, canonical_index, pair_number, domain,
                            std::move(component_type), from_pos, to_pos,
                            std::max(1e-4, std::abs(r_pu)),
                            rate_mw > kEps ? rate_mw
                                           : opts.mip.default_branch_rate_mva,
                            initial_closed, enforce_voltage_drop});
  };

  for (const auto& br : sys.ac.branches) {
    const auto it_f = bus_pos.find(br.from_bus);
    const auto it_t = bus_pos.find(br.to_bus);
    if (it_f == bus_pos.end() || it_t == bus_pos.end()) continue;
    int stable_index = br.index;
    int pair_number = 0;
    std::string component_type = "ac_branch";
    if (const auto origin = ac_branch_origins.find(br.index);
        origin != ac_branch_origins.end()) {
      stable_index = origin->second.origin_index;
      pair_number = origin->second.pair_number;
      component_type = origin_component_type(origin->second.origin_type);
    }
    add_branch(stable_index, br.index, pair_number, ResilienceBranchKind::AC,
               std::move(component_type), it_f->second, it_t->second, br.r_pu,
               br.rate_a_mva > kEps ? br.rate_a_mva : br.sn_mva,
               br.in_service, true);
  }
  for (const auto& br : sys.dc.branches) {
    const auto it_f = dc_bus_pos.find(br.from_bus);
    const auto it_t = dc_bus_pos.find(br.to_bus);
    if (it_f == dc_bus_pos.end() || it_t == dc_bus_pos.end()) continue;
    add_branch(br.index, br.index, 0, ResilienceBranchKind::DC, "dc_branch",
               it_f->second, it_t->second, br.r_pu,
               br.rate_a_mva > kEps ? br.rate_a_mva : br.s_max_mva,
               br.in_service, true);
  }
  for (const auto& converter : sys.vsc_converters) {
    if (!converter.in_service) continue;
    const auto ac = bus_pos.find(converter.bus_ac);
    const auto dc = dc_bus_pos.find(converter.bus_dc);
    if (ac == bus_pos.end() || dc == dc_bus_pos.end()) continue;
    const double cap = std::max({std::abs(converter.pmax_mw),
                                 std::abs(converter.pmin_mw),
                                 std::abs(converter.p_set_mw),
                                 converter.p_rated_mw});
    add_branch(converter.index, converter.index, 0, ResilienceBranchKind::AC,
               "vsc_converter", ac->second, dc->second, 1e-4, cap, true,
               false);
  }
  for (const auto& converter : sys.lcc_converters) {
    if (!converter.in_service) continue;
    const auto ac = bus_pos.find(converter.ac_bus);
    const auto dc = dc_bus_pos.find(converter.dc_bus);
    if (ac == bus_pos.end() || dc == dc_bus_pos.end()) continue;
    double cap = std::abs(converter.p_set_mw);
    if (cap <= kEps && converter.rated_current_a > 0.0 &&
        converter.rated_dc_kv > 0.0) {
      cap = converter.rated_current_a * converter.rated_dc_kv / 1000.0;
    }
    add_branch(converter.index, converter.index, 0, ResilienceBranchKind::AC,
               "lcc_converter", ac->second, dc->second, 1e-4, cap, true,
               false);
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    if (!converter.in_service) continue;
    const auto from = dc_bus_pos.find(converter.bus_in);
    const auto to = dc_bus_pos.find(converter.bus_out);
    if (from == dc_bus_pos.end() || to == dc_bus_pos.end()) continue;
    const double cap = std::max({std::abs(converter.pmax_mw),
                                 std::abs(converter.pmin_mw),
                                 std::abs(converter.p_ref_mw),
                                 converter.sn_mva});
    add_branch(converter.index, converter.index, 0, ResilienceBranchKind::DC,
               "dcdc_converter", from->second, to->second, 1e-4, cap, true,
               false);
  }
  for (const auto& breaker : sys.dc.dc_circuit_breakers) {
    if (!breaker.in_service) continue;
    const bool controls_existing = breaker.element_id > 0 &&
        std::any_of(sys.dc.branches.begin(), sys.dc.branches.end(),
                    [&](const DCBranch& branch) {
                      return branch.index == breaker.element_id;
                    });
    if (controls_existing) {
      for (auto& branch : out.branches) {
        if (branch.component_type == "dc_branch" &&
            branch.index == breaker.element_id) {
          branch.initial_closed = branch.initial_closed && breaker.closed;
        }
      }
      continue;
    }
    const auto from = dc_bus_pos.find(breaker.bus_from);
    const auto to = dc_bus_pos.find(breaker.bus_to);
    if (from == dc_bus_pos.end() || to == dc_bus_pos.end()) continue;
    add_branch(breaker.index, breaker.index, 0, ResilienceBranchKind::DC,
               "dc_circuit_breaker", from->second, to->second, 1e-4,
               breaker.i_rated_ka > 0.0 && breaker.rated_voltage_kv > 0.0
                   ? breaker.i_rated_ka * breaker.rated_voltage_kv
                   : opts.mip.default_branch_rate_mva,
               breaker.closed, false);
  }

  const int n_branch = static_cast<int>(out.branches.size());
  const auto faults = build_faults(out.branches, opts);
  out.faults = faults;
  out.branch_available.assign(static_cast<size_t>(T),
                              std::vector<bool>(static_cast<size_t>(n_branch), true));
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
                                 false,
                                 it->second,
                                 std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                                 std::max(0.0, e0),
                                 st.soc_min * st.e_rated_mwh,
                                 std::max(0.0, st.e_rated_mwh),
                                 st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                                 st.grid_forming});
  }
  for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
    const auto& st = sys.dc.storage[i];
    if (!st.in_service) continue;
    const auto it = dc_bus_pos.find(st.bus);
    if (it == dc_bus_pos.end()) continue;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    out.fixed_storage.push_back({st.index != 0 ? st.index : static_cast<int>(i + 1),
                                 true,
                                 it->second,
                                 std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                                 std::max(0.0, e0),
                                 st.soc_min * st.e_rated_mwh,
                                 std::max(0.0, st.e_rated_mwh),
                                 st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                                 st.grid_forming});
  }
  for (size_t i = 0; i < sys.dc.dc_storage.size(); ++i) {
    const auto& st = sys.dc.dc_storage[i];
    if (!st.in_service) continue;
    const auto it = dc_bus_pos.find(st.bus);
    if (it == dc_bus_pos.end()) continue;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    out.fixed_storage.push_back({st.index != 0 ? st.index : static_cast<int>(i + 1),
                                 true,
                                 it->second,
                                 std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                                 std::max(0.0, e0),
                                 st.soc_min * st.e_rated_mwh,
                                 std::max(0.0, st.e_rated_mwh),
                                 st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                                 false});
  }
  for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
    const auto& st = sys.mobile_storage[i];
    if (!st.in_service) continue;
    const auto it = bus_pos.find(st.bus);
    if (it == bus_pos.end()) continue;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    const int stable_index =
        st.index != 0 ? st.index : static_cast<int>(i + 1);
    const auto available_it =
        opts.mobile_storage_available_from_hr.find(stable_index);
    const double available_hr = available_it ==
            opts.mobile_storage_available_from_hr.end()
        ? 0.0
        : std::max(0.0, available_it->second);
    const int available_step = std::clamp(
        static_cast<int>(std::ceil(available_hr / dt - kEps)), 0, T);
    out.mess.push_back({stable_index,
                        it->second,
                        available_step,
                        std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw}),
                        std::max(0.0, e0),
                        st.soc_min * st.e_rated_mwh,
                        std::max(0.0, st.e_rated_mwh),
                        st.eta_discharge > 0.0 ? st.eta_discharge : 1.0,
                        std::max(0.0, st.e_consumption_mwh_km),
                        st.max_travel_distance_km > 0.0 ? st.max_travel_distance_km : kInf,
                        st.grid_forming});
  }

  const Adj transport_graph = build_transport_graph(sys, opts, bus_pos, n_bus);
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
                                            t >= out.mess[m].available_step &&
                                                    opts.mip.allow_mess_black_start &&
                                                    out.mess[m].grid_forming
                                                ? 1.0
                                                : 0.0,
                                            "mr_" + std::to_string(m) + "_" + std::to_string(i) + "_" + std::to_string(t), 0.0));
        out.idx.mess_dis.push_back(add_var(solver::VarType::Continuous,
                                           0.0,
                                           opts.allow_mess_dispatch &&
                                                   t >= out.mess[m].available_step
                                               ? out.mess[m].pmax_mw
                                               : 0.0,
                                           "md_" + std::to_string(m) + "_" + std::to_string(i) + "_" + std::to_string(t),
                                           kMessDispatchTieBreakCostPerMWh * dt));
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
      const double arc_cost = mess_arcs[m][a].is_stay
          ? 0.0
          : opts.mip.mess_travel_cost_per_km * mess_arcs[m][a].distance_km +
                kMessMoveTieBreakCost;
      out.idx.mess_arc_var[m].push_back(add_var(solver::VarType::Binary,
                                                0.0,
                                                mess_arcs[m][a].is_stay ||
                                                        mess_arcs[m][a].depart_t >=
                                                            out.mess[m].available_step
                                                    ? 1.0
                                                    : 0.0,
                                                "ma_" + std::to_string(m) + "_" + std::to_string(a),
                                                arc_cost));
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

      if (br.enforce_voltage_drop) {
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
      add_le({{out.idx.fsd(static_cast<int>(s), t),
               dt / std::max(st.eta_discharge, 1e-6)},
              {out.idx.fse(static_cast<int>(s), t), -1.0}},
             -st.e_min_mwh);
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

      std::vector<std::pair<int, double>> dispatch_energy{
          {out.idx.me(static_cast<int>(m), t), -1.0}};
      for (int i = 0; i < n_bus; ++i) {
        dispatch_energy.push_back(
            {out.idx.md(static_cast<int>(m), i, t),
             dt / std::max(ms.eta_discharge, 1e-6)});
      }
      add_le(dispatch_energy, -ms.e_min_mwh);

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
  out.stats.model_scope = "hybrid-acdc-restoration-milp";
  auto& validity = out.stats.validity;
  validity.canonical_projection_used = sys.projection_certificate.has_value();
  validity.dc_network_modelled = !sys.dc.buses.empty();
  validity.vsc_dispatch_modelled = !sys.vsc_converters.empty();
  validity.lcc_dispatch_modelled = !sys.lcc_converters.empty();
  validity.dcdc_dispatch_modelled = !sys.dc.dcdc_converters.empty();
  validity.dc_branch_flow_limits_enforced = !sys.dc.branches.empty();
  validity.converter_transfer_limits_enforced =
      validity.vsc_dispatch_modelled || validity.lcc_dispatch_modelled ||
      validity.dcdc_dispatch_modelled;
  validity.ac_dc_storage_modelled =
      !sys.ac.storage.empty() || !sys.dc.storage.empty() ||
      !sys.dc.dc_storage.empty() || !sys.mobile_storage.empty();
  if (sys.projection_report.has_value()) {
    const auto& report = *sys.projection_report;
    validity.energy_router_modelled = report.count_source("EnergyRouter") > 0;
    validity.transformers_modelled =
        report.count_source("Transformer2W") > 0 ||
        report.count_source("Transformer3W") > 0;
    validity.switches_and_breakers_modelled =
        report.count_source("Switch") > 0 ||
        report.count_source("CircuitBreaker") > 0 ||
        !sys.dc.dc_circuit_breakers.empty();
    validity.rich_loads_modelled =
        report.count_source("FlexibleLoad") > 0 ||
        report.count_source("AsymmetricLoad") > 0 ||
        report.count_source("AsynchronousMotor") > 0 ||
        !sys.ac.charging_stations.empty();
    validity.aggregated_resources_modelled =
        report.count_source("VirtualPowerPlant") > 0 ||
        report.count_source("Microgrid") > 0;
  }
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
  if (sys.ac.buses.empty() && sys.dc.buses.empty()) {
    result.feasible = false;
    result.status = "System has no AC or DC distribution network for strict resilience MIP";
    return result;
  }

  HybridPowerSystem model_system;
  if (sys.projection_certificate.has_value()) {
    model_system = sys;
  } else {
    auto projection_input = sys;
    const auto mobile_storage = projection_input.mobile_storage;
    projection_input.mobile_storage.clear();
    ProjectionOptions projection_options;
    projection_options.mode = ProjectionMode::ExactIdeal;
    projection_options.strip_dead_islands = false;
    projection_options.preserve_switch_branches = true;
    model_system = project_to_canonical_models(std::move(projection_input),
                                                projection_options);
    model_system.mobile_storage = mobile_storage;
  }

  auto built = build_mip_skeleton(model_system, opts);
  result.model_stats = built.stats;
  for (const auto& f : built.faults) {
    result.fault_sequence.push_back(
        {f.branch_kind, f.branch_index, f.start_hr, f.repair_hr, f.name});
  }
  if (!opts.faults.empty() && built.faults.size() != opts.faults.size()) {
    result.feasible = false;
    result.status =
        "One or more explicit faults do not match an authored AC/DC branch "
        "in the canonical restoration model";
    return result;
  }
  if (built.faults.empty() && opts.faults.empty() && opts.default_fault_count <= 0) {
    return make_no_fault_baseline_result(built, opts);
  }

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
      engine::StrictHighsBranchAndCutAdapter adapter(bc_opts);
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

  // Preserve the component-level capability flags populated while building
  // the canonical hybrid model. A non-zero gap means the incumbent is
  // feasible but its global optimality is unproven.
  result.model_stats.model_scope = "hybrid-acdc-restoration-milp";
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
      sr.bus_supply_kind.push_back(built.buses[i].dc_side ? "DC" : "AC");
      sr.bus_supply_index.push_back(built.buses[i].index);
      sr.bus_supply_demand_mw.push_back(demand);
      sr.bus_supply_served_mw.push_back(served);
      sr.bus_supply_shed_mw.push_back(shed);
      sr.bus_supply_priority_tier.push_back(pidx);
      sr.bus_supply_importance.push_back(built.buses[i].importance);

      BusVoltageStep voltage;
      voltage.bus_index = built.buses[i].index;
      voltage.domain = built.buses[i].dc_side ? "DC" : "AC";
      voltage.vm_pu = std::sqrt(std::max(0.0, x[idx.vb(static_cast<int>(i), t)]));
      sr.bus_voltages.push_back(std::move(voltage));
    }
    for (size_t b = 0; b < built.branches.size(); ++b) {
      const auto& branch = built.branches[b];
      const bool is_closed = x[idx.bt(static_cast<int>(b), t)] > 0.5;
      closed[b] = is_closed ? 1 : 0;
      if (is_closed) {
        if (!branch.initial_closed && branch.component_type == "dc_branch") {
          sr.closed_dc_tie_branch_ids.push_back(branch.index);
        } else if (!branch.initial_closed && branch.component_type == "ac_branch") {
          sr.closed_tie_branch_ids.push_back(branch.index);
        }
      } else if (branch.component_type == "dc_branch") {
        sr.open_dc_branch_ids.push_back(branch.index);
      } else if (branch.component_type == "ac_branch") {
        sr.open_ac_branch_ids.push_back(branch.index);
      }
      sr.switch_actions += static_cast<int>(std::lround(std::max(0.0, x[idx.yon(static_cast<int>(b), t)]) + std::max(0.0, x[idx.yoff(static_cast<int>(b), t)])));
      const double flow = x[idx.pb(static_cast<int>(b), t)];
      const std::string domain =
          branch.component_type == "vsc_converter" ||
                  branch.component_type == "lcc_converter"
              ? "AC-DC"
              : (branch.domain == ResilienceBranchKind::DC ? "DC" : "AC");
      sr.component_states.push_back(
          {branch.component_type,
           domain,
           branch.index,
           branch.canonical_index,
           branch.pair_number,
           built.buses[static_cast<size_t>(branch.from_pos)].index,
           built.buses[static_cast<size_t>(branch.to_pos)].index,
           built.branch_available[static_cast<size_t>(t)][b],
           is_closed,
           true,
           flow});
      BranchFlowStep flow_result;
      flow_result.branch_index = branch.index;
      flow_result.canonical_branch_index = branch.canonical_index;
      flow_result.pair_number = branch.pair_number;
      flow_result.component_type = branch.component_type;
      flow_result.domain = domain;
      flow_result.from_bus = built.buses[static_cast<size_t>(branch.from_pos)].index;
      flow_result.to_bus = built.buses[static_cast<size_t>(branch.to_pos)].index;
      flow_result.pf_mw = flow;
      flow_result.pt_mw = -flow;
      flow_result.loading_percent = branch.rate_mw > kEps
                                        ? 100.0 * std::abs(flow) / branch.rate_mw
                                        : 0.0;
      sr.branch_flows.push_back(std::move(flow_result));
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
      ms.arrival_time_hr = sr.hour;
      ms.remaining_travel_hr = 0.0;
      double dispatch = 0.0;
      for (size_t i = 0; i < built.buses.size(); ++i) dispatch += std::max(0.0, x[idx.md(static_cast<int>(m), static_cast<int>(i), t)]);
      ms.dispatch_mw = dispatch;
      ms.status = dispatch > kEps ? "Deployed" : "Stationary";

      const ArcDef* active_travel_arc = nullptr;
      for (size_t a = 0; a < built.idx.mess_arc_var[m].size() && a < built.mess_arcs[m].size(); ++a) {
        if (x[built.idx.mess_arc_var[m][a]] <= 0.5) continue;
        const auto& arc = built.mess_arcs[m][a];
        if (!arc.is_stay && arc.depart_t == t) {
          result.mess_travel_distance_km += arc.distance_km;
        }
        if (!arc.is_stay && arc.depart_t <= t && t < arc.arrive_t) {
          active_travel_arc = &arc;
        }
      }
      if (active_travel_arc != nullptr) {
        ms.bus = built.buses[static_cast<size_t>(active_travel_arc->from_pos)].index;
        ms.target_bus = built.buses[static_cast<size_t>(active_travel_arc->to_pos)].index;
        ms.status = "InTransit";
        ms.remaining_travel_hr = std::max(
            0.0,
            static_cast<double>(active_travel_arc->arrive_t - t) * opts.time_step_hr);
        ms.arrival_time_hr = static_cast<double>(active_travel_arc->arrive_t) * opts.time_step_hr;
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

  if (result.total_shed_mwh <= kEps) {
    std::unordered_map<int, MESSStateStep> initial_mess_state;
    for (const auto& step : result.steps) {
      for (const auto& ms : step.mess_states) {
        initial_mess_state.emplace(ms.storage_index, ms);
      }
    }
    for (auto& step : result.steps) {
      for (auto& ms : step.mess_states) {
        const auto it = initial_mess_state.find(ms.storage_index);
        if (it == initial_mess_state.end()) continue;
        ms.bus = it->second.bus;
        ms.target_bus = it->second.bus;
        ms.status = "Stationary";
        ms.dispatch_mw = 0.0;
        ms.energy_mwh = it->second.energy_mwh;
        ms.soc = it->second.soc;
        ms.arrival_time_hr = step.hour;
        ms.remaining_travel_hr = 0.0;
      }
    }
    result.mess_travel_distance_km = 0.0;
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

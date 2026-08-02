#include "hacdcpf/resilience/resilience_assessment.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
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

const std::vector<double> kDefaultLoadProfile = {
    0.64, 0.60, 0.58, 0.56, 0.56, 0.58, 0.64, 0.76,
    0.87, 0.95, 0.99, 1.00, 0.99, 1.00, 1.00, 0.97,
    0.96, 0.96, 0.93, 0.92, 0.92, 0.93, 0.87, 0.72};

const std::vector<double> kDefaultRenewableProfile = {
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.05, 0.20,
    0.45, 0.70, 0.85, 0.95, 1.00, 0.95, 0.85, 0.70,
    0.45, 0.20, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00};

struct StagePriorityWeights { double importance; int tier; };

StagePriorityWeights stage_priority_weights(LoadPriority p) {
  switch (p) {
    case LoadPriority::Critical: return {4.0, 0};
    case LoadPriority::High:     return {2.5, 1};
    case LoadPriority::Medium:   return {1.5, 2};
    case LoadPriority::Low:      return {1.0, 3};
  }
  return {1.0, 3};
}

int priority_tier_from_importance(double importance) {
  if (importance >= 4.0 - kEps) return 0;
  if (importance >= 2.5 - kEps) return 1;
  if (importance >= 1.5 - kEps) return 2;
  return 3;
}

struct StageBus {
  ResilienceBranchKind kind{ResilienceBranchKind::AC};
  int index{0};
  bool source{false};
  double demand_mw{0.0};
  double gen_cap_mw{0.0};
  double importance{1.0};
  int priority_tier{3};
};

void apply_adaptive_priority_tiers(std::vector<StageBus>& buses) {
  std::vector<int> demand_pos;
  demand_pos.reserve(buses.size());
  for (size_t i = 0; i < buses.size(); ++i) {
    if (buses[i].demand_mw > kEps) demand_pos.push_back(static_cast<int>(i));
  }
  if (demand_pos.size() < 4) return;
  const int first_tier = buses[static_cast<size_t>(demand_pos.front())].priority_tier;
  const bool all_same = std::all_of(demand_pos.begin(), demand_pos.end(), [&](int pos) {
    return buses[static_cast<size_t>(pos)].priority_tier == first_tier;
  });
  if (!all_same) return;

  std::sort(demand_pos.begin(), demand_pos.end(), [&](int a, int b) {
    return buses[static_cast<size_t>(a)].demand_mw > buses[static_cast<size_t>(b)].demand_mw;
  });
  const size_t n = demand_pos.size();
  for (size_t rank = 0; rank < n; ++rank) {
    auto& bus = buses[static_cast<size_t>(demand_pos[rank])];
    const double q = (static_cast<double>(rank) + 0.5) / static_cast<double>(n);
    if (q <= 0.15) { bus.priority_tier = 0; bus.importance = 4.0; }
    else if (q <= 0.40) { bus.priority_tier = 1; bus.importance = 2.5; }
    else if (q <= 0.75) { bus.priority_tier = 2; bus.importance = 1.5; }
    else { bus.priority_tier = 3; bus.importance = 1.0; }
  }
}

struct StageEdge {
  ResilienceBranchKind kind{ResilienceBranchKind::AC};
  int index{0};
  int from{0};
  int to{0};
  bool initial_closed{true};
  bool switchable{false};
  bool tie{false};
  bool vsc{false};
  double rate_mw{10.0};
  std::vector<int> switch_ids;
};

struct StageFault {
  ResilienceBranchKind kind{ResilienceBranchKind::AC};
  int edge_pos{-1};
  int branch_index{0};
  double start_hr{0.0};
  double repair_hr{0.0};
  std::string name;
};

int requested_branch_index(const DistributionResilienceFault& f) {
  return (f.branch_index == 0 && f.ac_branch_index != 0) ? f.ac_branch_index : f.branch_index;
}

struct StageMessState {
  int storage_index{0};
  int ac_bus_id{0};
  int target_ac_bus_id{0};
  MobileStorageStatus status{MobileStorageStatus::Stationary};
  double pmax_mw{0.0};
  double e_rated_mwh{0.0};
  double e_min_mwh{0.0};
  double e_max_mwh{0.0};
  double energy_mwh{0.0};
  double eta_discharge{1.0};
  double e_consumption_mwh_km{0.0};
  double max_travel_distance_km{kInf};
  double dispatch_mw{0.0};
  double arrival_time_hr{0.0};
  double remaining_travel_hr{0.0};
};

struct StageSwitch {
  int id{0};
  int edge_pos{-1};
  bool initial_closed{true};
  bool remote{false};
  bool tie{false};
  bool breaker{false};
  std::string name;
};

struct StageSolveStats {
  int num_variables{0};
  int num_binary_variables{0};
  int num_integer_variables{0};
  int num_eq_constraints{0};
  int num_ineq_constraints{0};
  double objective_value{0.0};
  double mip_gap{0.0};
  double runtime_sec{0.0};
  std::string solver_status;
};

struct StageData {
  std::vector<StageBus> buses;
  std::vector<StageEdge> edges;
  std::unordered_map<int, int> ac_bus_pos;
  std::unordered_map<int, int> dc_bus_pos;
  std::unordered_map<int, int> ac_branch_edge_pos;
  std::unordered_map<int, int> dc_branch_edge_pos;
  std::unordered_map<int, int> switch_pos_by_id;
  std::vector<int> all_switch_ids;
  std::vector<StageSwitch> switches;
  bool uses_virtual_branch_switches{false};
  double total_demand_mw{0.0};
  double big_m{1000.0};
};

struct StageSolution {
  bool feasible{false};
  std::string status;
  double objective{0.0};
  std::vector<int> beta;
  std::vector<int> z;
  std::vector<double> shed;
  std::vector<int> fault_zone_ac_bus_ids;
  std::vector<int> fault_zone_dc_bus_ids;
  std::vector<int> open_switch_ids;
  std::vector<int> closed_switch_ids;
  StageSolveStats stats;
};

struct StageVarIndex {
  int n_bus{0};
  int n_edge{0};
  std::vector<int> beta;
  std::vector<int> z;
  std::vector<int> energized;
  std::vector<int> shed;
  std::vector<int> gen;
  std::vector<int> flow;
  std::vector<int> virt;
  int beta_at(int e) const { return beta[static_cast<size_t>(e)]; }
  int z_at(int b) const { return z[static_cast<size_t>(b)]; }
  int energized_at(int b) const { return energized[static_cast<size_t>(b)]; }
  int shed_at(int b) const { return shed[static_cast<size_t>(b)]; }
  int gen_at(int b) const { return gen[static_cast<size_t>(b)]; }
  int flow_at(int e) const { return flow[static_cast<size_t>(e)]; }
  int virt_at(int b) const { return virt[static_cast<size_t>(b)]; }
};

std::string stage_solver_name(DistributionResilienceMIPSolver solver, int num_threads) {
  switch (solver) {
    case DistributionResilienceMIPSolver::Native:
      return num_threads > 1 ? "native-bc-par" : "native-bc";
    case DistributionResilienceMIPSolver::HiGHS: return "HiGHS";
    case DistributionResilienceMIPSolver::Gurobi: return "Gurobi";
  }
  return "unknown";
}

solver::NodeSelection to_bc_node_selection(DistributionResilienceNativeNodeSelection selection) {
  switch (selection) {
    case DistributionResilienceNativeNodeSelection::Hybrid: return solver::NodeSelection::Hybrid;
    case DistributionResilienceNativeNodeSelection::BestFirst: return solver::NodeSelection::BestFirst;
  }
  return solver::NodeSelection::Hybrid;
}

std::pair<int, int> edge_key(int a, int b) {
  return {std::min(a, b), std::max(a, b)};
}

std::string lower_copy(std::string value) {
  for (char& c : value) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return value;
}

bool looks_like_tie_switch(const std::string& name) {
  const auto lower = lower_copy(name);
  return lower.find("tie") != std::string::npos || lower.find("联络") != std::string::npos;
}

void sort_unique(std::vector<int>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

std::vector<int> initial_beta(const StageData& data) {
  std::vector<int> beta(data.edges.size(), 0);
  for (size_t e = 0; e < data.edges.size(); ++e) beta[e] = data.edges[e].initial_closed ? 1 : 0;
  return beta;
}

void populate_switch_state_from_beta(const StageData& data,
                                     const std::vector<int>& beta,
                                     StageSolution& sol) {
  sol.fault_zone_ac_bus_ids.clear();
  sol.fault_zone_dc_bus_ids.clear();
  sol.open_switch_ids.clear();
  sol.closed_switch_ids.clear();
  for (const auto& sw : data.switches) {
    bool closed = sw.initial_closed;
    if (sw.edge_pos >= 0 && sw.edge_pos < static_cast<int>(beta.size())) {
      closed = beta[static_cast<size_t>(sw.edge_pos)] != 0;
    }
    if (closed) sol.closed_switch_ids.push_back(sw.id);
    else sol.open_switch_ids.push_back(sw.id);
  }
  sort_unique(sol.open_switch_ids);
  sort_unique(sol.closed_switch_ids);
}

StageSolution make_initial_topology_solution(const StageData& data,
                                             std::string status) {
  StageSolution sol;
  sol.feasible = true;
  sol.status = std::move(status);
  sol.beta = initial_beta(data);
  sol.z.assign(data.buses.size(), 0);
  sol.shed.assign(data.buses.size(), 0.0);
  populate_switch_state_from_beta(data, sol.beta, sol);
  return sol;
}

std::unordered_map<int, int> make_ac_branch_pos(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    out[sys.ac.branches[i].index] = static_cast<int>(i);
  }
  return out;
}

std::unordered_map<int, int> make_dc_branch_pos(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    out[sys.dc.branches[i].index] = static_cast<int>(i);
  }
  return out;
}

double sample_profile(const std::vector<double>& profile, double hour) {
  if (profile.empty()) return 1.0;
  const double period = static_cast<double>(profile.size());
  double h = std::fmod(hour, period);
  if (h < 0.0) h += period;
  const size_t lo = static_cast<size_t>(h) % profile.size();
  const size_t hi = (lo + 1) % profile.size();
  const double frac = h - std::floor(h);
  return std::max(0.0, profile[lo] * (1.0 - frac) + profile[hi] * frac);
}

double mapped_profile_value(const std::unordered_map<int, std::vector<double>>& profiles,
                            int key,
                            double hour,
                            double fallback) {
  const auto it = profiles.find(key);
  return it == profiles.end() || it->second.empty() ? fallback : sample_profile(it->second, hour);
}

StageData build_stage_data(const HybridPowerSystem& sys,
                           const DistributionResilienceOptions& opts,
                           double hour) {
  StageData data;
  const auto& load_profile = opts.load_profile.empty() ? kDefaultLoadProfile : opts.load_profile;
  const auto& renewable_profile = opts.renewable_profile.empty() ? kDefaultRenewableProfile : opts.renewable_profile;
  const auto& pv_profile = opts.pv_profile.empty() ? renewable_profile : opts.pv_profile;
  const auto& wind_profile = opts.wind_profile.empty() ? renewable_profile : opts.wind_profile;
  const double load_mult = sample_profile(load_profile, hour);
  const double ren_mult = sample_profile(renewable_profile, hour);
  const double pv_mult = sample_profile(pv_profile, hour);
  const double wind_mult = sample_profile(wind_profile, hour);

  data.buses.reserve(sys.ac.buses.size() + sys.dc.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& b = sys.ac.buses[i];
    if (!b.in_service) continue;
    const int pos = static_cast<int>(data.buses.size());
    data.ac_bus_pos[b.index] = pos;
    StageBus sb;
    sb.kind = ResilienceBranchKind::AC;
    sb.index = b.index;
    sb.source = b.bus_type == BusType::SLACK;
    const double bus_mult = mapped_profile_value(opts.ac_bus_load_profiles_by_bus, b.index, hour, load_mult);
    sb.demand_mw = std::max(0.0, b.pd_mw * opts.load_scale_factor * bus_mult);
    sb.importance = std::max(1.0, b.importance);
    sb.priority_tier = priority_tier_from_importance(sb.importance);
    data.buses.push_back(sb);
  }
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    const auto& b = sys.dc.buses[i];
    if (!b.in_service) continue;
    const int pos = static_cast<int>(data.buses.size());
    data.dc_bus_pos[b.index] = pos;
    StageBus sb;
    sb.kind = ResilienceBranchKind::DC;
    sb.index = b.index;
    sb.source = b.bus_type == DCBusType::DC_V;
    const double bus_mult = mapped_profile_value(opts.dc_bus_load_profiles_by_bus, b.index, hour, load_mult);
    sb.demand_mw = std::max(0.0, b.pd_mw * opts.load_scale_factor * bus_mult);
    sb.importance = std::max(1.0, b.importance);
    sb.priority_tier = priority_tier_from_importance(sb.importance);
    data.buses.push_back(sb);
  }
  if (!sys.ac.loads.empty()) {
    for (size_t li = 0; li < sys.ac.loads.size(); ++li) {
      const auto& ld = sys.ac.loads[li];
      if (!ld.in_service) continue;
      const auto it = data.ac_bus_pos.find(ld.bus);
      if (it == data.ac_bus_pos.end()) continue;
      auto& bus = data.buses[static_cast<size_t>(it->second)];
      const auto weights = stage_priority_weights(ld.priority);
      double profile_mult = mapped_profile_value(opts.ac_load_profiles_by_position, static_cast<int>(li), hour, load_mult);
      profile_mult = mapped_profile_value(opts.ac_load_profiles_by_index, ld.index, hour, profile_mult);
      profile_mult = mapped_profile_value(opts.ac_load_profiles_by_bus, ld.bus, hour, profile_mult);
      bus.demand_mw += std::max(0.0, ld.p_mw * std::max(ld.scaling, 1.0) * opts.load_scale_factor * profile_mult);
      if (weights.importance > bus.importance + kEps) {
        bus.importance = weights.importance;
        bus.priority_tier = weights.tier;
      }
    }
  }
  for (size_t li = 0; li < sys.dc.loads.size(); ++li) {
    const auto& ld = sys.dc.loads[li];
    if (!ld.in_service) continue;
    const auto it = data.dc_bus_pos.find(ld.bus);
    if (it == data.dc_bus_pos.end()) continue;
    const double base = ld.p_mw > 0.0 ? ld.p_mw : ld.p_rated_mw;
    auto& bus = data.buses[static_cast<size_t>(it->second)];
    const auto weights = stage_priority_weights(ld.priority);
    double profile_mult = mapped_profile_value(opts.dc_load_profiles_by_position, static_cast<int>(li), hour, load_mult);
    profile_mult = mapped_profile_value(opts.dc_load_profiles_by_index, ld.index, hour, profile_mult);
    profile_mult = mapped_profile_value(opts.dc_load_profiles_by_bus, ld.bus, hour, profile_mult);
    bus.demand_mw += std::max(0.0, base * std::max(ld.scaling, 1.0) * opts.load_scale_factor * profile_mult);
    if (weights.importance > bus.importance + kEps) {
      bus.importance = weights.importance;
      bus.priority_tier = weights.tier;
    }
  }

  // Keep the user/model-defined load priorities stable across time.  Re-ranking
  // buses by hourly demand makes the same curtailed load jump between Medium and
  // Low in the GUI priority-shed chart as the scenario profile changes.

  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = data.ac_bus_pos.find(eg.bus);
    if (it != data.ac_bus_pos.end()) data.buses[static_cast<size_t>(it->second)].source = true;
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const auto it = data.ac_bus_pos.find(g.bus);
    if (it == data.ac_bus_pos.end()) continue;
    auto& b = data.buses[static_cast<size_t>(it->second)];
    const double cap = std::max({0.0, g.pmax_mw, g.pg_mw});
    b.gen_cap_mw += cap;
    b.source = b.source || g.is_slack || cap > kEps;
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const auto it = data.ac_bus_pos.find(sg.bus);
    if (it == data.ac_bus_pos.end()) continue;
    const double cap = std::max(0.0, (sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_mw) * std::max(sg.scaling, 1.0));
    data.buses[static_cast<size_t>(it->second)].gen_cap_mw += cap;
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    const auto it = data.ac_bus_pos.find(rg.bus);
    if (it == data.ac_bus_pos.end()) continue;
    double mult = ren_mult;
    if (rg.type == RenewableType::Wind) mult = wind_mult;
    else if (rg.type == RenewableType::SolarPV || rg.type == RenewableType::SolarCSP) mult = pv_mult;
    const double cap = std::max(0.0, (rg.p_rated_mw > 0.0 ? rg.p_rated_mw : rg.p_mw) * mult);
    data.buses[static_cast<size_t>(it->second)].gen_cap_mw += cap;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    const auto it = data.ac_bus_pos.find(pv.bus);
    if (it == data.ac_bus_pos.end()) continue;
    const double cap = std::max(0.0, (pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw) * pv_mult);
    data.buses[static_cast<size_t>(it->second)].gen_cap_mw += cap;
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const auto it = data.dc_bus_pos.find(sg.bus);
    if (it == data.dc_bus_pos.end()) continue;
    data.buses[static_cast<size_t>(it->second)].gen_cap_mw +=
        std::max(0.0, (sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_set_mw) * std::max(sg.scaling, 1.0));
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    const auto it = data.dc_bus_pos.find(pv.bus);
    if (it == data.dc_bus_pos.end()) continue;
    data.buses[static_cast<size_t>(it->second)].gen_cap_mw +=
        std::max(0.0, pv.p_set_mw * pv_mult);
  }

  data.edges.reserve(sys.ac.branches.size() + sys.dc.branches.size() + sys.vsc_converters.size());
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    const auto it_f = data.ac_bus_pos.find(br.from_bus);
    const auto it_t = data.ac_bus_pos.find(br.to_bus);
    if (it_f == data.ac_bus_pos.end() || it_t == data.ac_bus_pos.end()) continue;
    StageEdge e;
    e.kind = ResilienceBranchKind::AC;
    e.index = br.index;
    e.from = it_f->second;
    e.to = it_t->second;
    e.initial_closed = br.in_service;
    e.tie = !br.in_service;
    e.rate_mw = br.rate_a_mva > 1e-9 ? br.rate_a_mva : opts.mip.default_branch_rate_mva;
    data.ac_branch_edge_pos[e.index] = static_cast<int>(data.edges.size());
    data.edges.push_back(e);
  }
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    const auto it_f = data.dc_bus_pos.find(br.from_bus);
    const auto it_t = data.dc_bus_pos.find(br.to_bus);
    if (it_f == data.dc_bus_pos.end() || it_t == data.dc_bus_pos.end()) continue;
    StageEdge e;
    e.kind = ResilienceBranchKind::DC;
    e.index = br.index;
    e.from = it_f->second;
    e.to = it_t->second;
    e.initial_closed = br.in_service;
    e.tie = !br.in_service;
    e.rate_mw = br.rate_a_mva > 1e-9 ? br.rate_a_mva : opts.mip.default_branch_rate_mva;
    data.dc_branch_edge_pos[e.index] = static_cast<int>(data.edges.size());
    data.edges.push_back(e);
  }
  for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
    const auto& c = sys.vsc_converters[i];
    if (!c.in_service) continue;
    const auto it_ac = data.ac_bus_pos.find(c.bus_ac);
    const auto it_dc = data.dc_bus_pos.find(c.bus_dc);
    if (it_ac == data.ac_bus_pos.end() || it_dc == data.dc_bus_pos.end()) continue;
    StageEdge e;
    e.kind = ResilienceBranchKind::AC;
    e.index = static_cast<int>(100000 + i + 1);
    e.from = it_ac->second;
    e.to = it_dc->second;
    e.initial_closed = true;
    e.switchable = false;
    e.vsc = true;
    e.rate_mw = c.pmax_mw > 1e-9 ? c.pmax_mw : opts.mip.default_branch_rate_mva;
    data.edges.push_back(e);
  }

  std::map<std::pair<int, int>, std::vector<int>> edge_by_pair;
  for (size_t ei = 0; ei < data.edges.size(); ++ei) {
    const auto& e = data.edges[ei];
    if (e.kind != ResilienceBranchKind::AC || e.vsc) continue;
    edge_by_pair[edge_key(data.buses[static_cast<size_t>(e.from)].index,
                          data.buses[static_cast<size_t>(e.to)].index)].push_back(static_cast<int>(ei));
  }
  bool mapped_explicit_switching_device = false;
  for (const auto& sw : sys.ac.switches) {
    if (!sw.in_service) continue;
    if (opts.use_remote_switch_only && !sw.is_remote) continue;
    const int sw_id = sw.index != 0 ? sw.index : static_cast<int>(data.all_switch_ids.size() + 1);
    const bool tie_switch = !sw.closed || looks_like_tie_switch(sw.name);
    data.all_switch_ids.push_back(sw_id);
    data.switch_pos_by_id[sw_id] = static_cast<int>(data.all_switch_ids.size() - 1);
    StageSwitch meta;
    meta.id = sw_id;
    meta.initial_closed = sw.closed;
    meta.remote = sw.is_remote;
    meta.tie = tie_switch;
    meta.name = sw.name;
    const auto it = edge_by_pair.find(edge_key(sw.bus_from, sw.bus_to));
    if (it != edge_by_pair.end() && !it->second.empty()) {
      int chosen = it->second.front();
      for (int cand : it->second) {
        if (data.edges[static_cast<size_t>(cand)].initial_closed == sw.closed) {
          chosen = cand;
          break;
        }
      }
      auto& e = data.edges[static_cast<size_t>(chosen)];
      e.switchable = true;
      e.tie = e.tie || tie_switch;
      e.initial_closed = e.initial_closed && sw.closed;
      e.switch_ids.push_back(sw_id);
      meta.edge_pos = chosen;
      mapped_explicit_switching_device = true;
    }
    data.switches.push_back(std::move(meta));
  }
  for (const auto& cb : sys.ac.circuit_breakers) {
    if (!cb.in_service) continue;
    const auto it = edge_by_pair.find(edge_key(cb.bus_from, cb.bus_to));
    if (it == edge_by_pair.end() || it->second.empty()) continue;
    auto& e = data.edges[static_cast<size_t>(it->second.front())];
    e.switchable = true;
    e.tie = e.tie || !cb.closed || cb.element_type == "tie";
    mapped_explicit_switching_device = true;
  }
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    if (!cb.in_service) continue;
    for (auto& e : data.edges) {
      if (e.kind != ResilienceBranchKind::DC) continue;
      const int fb = data.buses[static_cast<size_t>(e.from)].index;
      const int tb = data.buses[static_cast<size_t>(e.to)].index;
      if (edge_key(fb, tb) == edge_key(cb.bus_from, cb.bus_to)) {
        e.switchable = true;
        e.tie = e.tie || !cb.closed;
        mapped_explicit_switching_device = true;
      }
    }
  }

  if (!mapped_explicit_switching_device) {
    bool injected_virtual_switch = false;
    for (size_t ei = 0; ei < data.edges.size(); ++ei) {
      auto& e = data.edges[ei];
      if (e.vsc) continue;
      if (e.kind != ResilienceBranchKind::AC && e.kind != ResilienceBranchKind::DC) continue;
      const int virtual_id = e.kind == ResilienceBranchKind::DC
                                 ? -200000 - std::max(0, e.index)
                                 : -100000 - std::max(0, e.index);
      e.switchable = true;
      e.switch_ids.push_back(virtual_id);
      data.all_switch_ids.push_back(virtual_id);
      data.switch_pos_by_id[virtual_id] = static_cast<int>(data.all_switch_ids.size() - 1);
      StageSwitch meta;
      meta.id = virtual_id;
      meta.edge_pos = static_cast<int>(ei);
      meta.initial_closed = e.initial_closed;
      meta.remote = true;
      meta.tie = e.tie || !e.initial_closed;
      meta.breaker = true;
      meta.name = (e.kind == ResilienceBranchKind::DC ? "virtual_dc_branch_breaker_"
                                                       : "virtual_ac_branch_breaker_") + std::to_string(e.index);
      data.switches.push_back(std::move(meta));
      injected_virtual_switch = true;
    }
    data.uses_virtual_branch_switches = injected_virtual_switch;
  }

  data.total_demand_mw = 0.0;
  double total_gen = 0.0;
  for (const auto& b : data.buses) {
    data.total_demand_mw += b.demand_mw;
    total_gen += b.gen_cap_mw + (b.source ? data.total_demand_mw + 10.0 : 0.0);
  }
  data.big_m = std::max({100.0, 4.0 * data.total_demand_mw + 10.0, total_gen + 10.0});
  return data;
}

std::vector<StageFault> build_stage_faults(const HybridPowerSystem& sys,
                                           const DistributionResilienceOptions& opts,
                                           const StageData& data) {
  const auto ac_pos = make_ac_branch_pos(sys);
  const auto dc_pos = make_dc_branch_pos(sys);
  std::vector<StageFault> out;
  if (!opts.faults.empty()) {
    for (const auto& f : opts.faults) {
      const ResilienceBranchKind kind = f.branch_kind;
      const int requested = requested_branch_index(f);
      if (kind == ResilienceBranchKind::DC) {
        const auto edge_it = data.dc_branch_edge_pos.find(requested);
        if (edge_it == data.dc_branch_edge_pos.end()) continue;
        out.push_back({kind, edge_it->second, requested, f.outage_start_hr,
                       std::max(opts.time_step_hr, f.repair_duration_hr), f.name});
      } else {
        const auto edge_it = data.ac_branch_edge_pos.find(requested);
        if (edge_it == data.ac_branch_edge_pos.end()) continue;
        out.push_back({kind, edge_it->second, requested, f.outage_start_hr,
                       std::max(opts.time_step_hr, f.repair_duration_hr), f.name});
      }
    }
    return out;
  }
  int added = 0;
  for (const auto& [idx, pos] : ac_pos) {
    if (added >= opts.default_fault_count) break;
    const auto edge_it = data.ac_branch_edge_pos.find(idx);
    if (edge_it == data.ac_branch_edge_pos.end()) continue;
    const auto& br = sys.ac.branches[static_cast<size_t>(pos)];
    if (!br.in_service) continue;
    out.push_back({ResilienceBranchKind::AC, edge_it->second, idx,
                   opts.auto_fault_start_hr + added * std::max(0.0, opts.auto_fault_stagger_hr),
                   std::max(opts.time_step_hr, opts.default_repair_time_hr), br.name});
    ++added;
  }
  return out;
}

DistributionDisasterStage stage_for_hour(const std::vector<StageFault>& faults,
                                         double hour,
                                         double window_hr) {
  if (faults.empty()) return DistributionDisasterStage::Normal;
  double first = kInf;
  double last = -kInf;
  for (const auto& f : faults) {
    first = std::min(first, f.start_hr);
    last = std::max(last, f.start_hr);
  }
  if (hour < first - kEps) return DistributionDisasterStage::Normal;
  if (hour <= last + kEps) return DistributionDisasterStage::DisasterIsolation;
  if (hour <= last + window_hr + kEps) return DistributionDisasterStage::DisasterPostFaultReconfig;
  return DistributionDisasterStage::PostDisasterRepair;
}

std::vector<int> occurred_fault_edges(const std::vector<StageFault>& faults,
                                      double hour,
                                      bool disaster_stage) {
  std::vector<int> out;
  for (const auto& f : faults) {
    const bool occurred = hour >= f.start_hr - kEps;
    const bool active = hour >= f.start_hr - kEps && hour < f.start_hr + f.repair_hr - kEps;
    if ((disaster_stage && occurred) || (!disaster_stage && active)) out.push_back(f.edge_pos);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<int> occurred_fault_edges_no_repair(const std::vector<StageFault>& faults,
                                                double hour) {
  std::vector<int> out;
  for (const auto& f : faults) {
    if (hour >= f.start_hr - kEps) out.push_back(f.edge_pos);
  }
  sort_unique(out);
  return out;
}

struct ModelBuilder {
  engine::MIPModel model;
  StageVarIndex idx;
  std::vector<double> c;
  std::vector<engine::VariableMeta> vars;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  std::vector<double> beq;
  std::vector<double> b;

  int add_var(engine::VarType type, double lb, double ub, std::string name, double obj) {
    const int col = static_cast<int>(vars.size());
    vars.push_back({type, lb, ub, std::move(name)});
    c.push_back(obj);
    if (type == engine::VarType::Binary) model.binary_idx.push_back(col);
    if (type == engine::VarType::Integer) model.integer_idx.push_back(col);
    return col;
  }
  void add_eq(const std::vector<std::pair<int, double>>& terms, double rhs) {
    const int row = static_cast<int>(beq.size());
    for (const auto& [col, val] : terms) if (std::abs(val) > kEps) eq_trips.emplace_back(row, col, val);
    beq.push_back(rhs);
  }
  void add_le(const std::vector<std::pair<int, double>>& terms, double rhs) {
    const int row = static_cast<int>(b.size());
    for (const auto& [col, val] : terms) if (std::abs(val) > kEps) ineq_trips.emplace_back(row, col, val);
    b.push_back(rhs);
  }
  void finalize(const DistributionResilienceOptions& opts) {
    (void)opts;
    auto& lp = model.linear_part;
    lp.sense = engine::Sense::Minimize;
    lp.vars = vars;
    lp.c = Eigen::VectorXd::Zero(static_cast<int>(c.size()));
    for (size_t i = 0; i < c.size(); ++i) lp.c[static_cast<Eigen::Index>(i)] = c[i];
    lp.Aeq.resize(static_cast<int>(beq.size()), static_cast<int>(vars.size()));
    lp.beq = Eigen::VectorXd::Zero(static_cast<int>(beq.size()));
    for (size_t i = 0; i < beq.size(); ++i) lp.beq[static_cast<Eigen::Index>(i)] = beq[i];
    lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
    lp.Aeq.makeCompressed();
    lp.A.resize(static_cast<int>(b.size()), static_cast<int>(vars.size()));
    lp.b = Eigen::VectorXd::Zero(static_cast<int>(b.size()));
    for (size_t i = 0; i < b.size(); ++i) lp.b[static_cast<Eigen::Index>(i)] = b[i];
    lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
    lp.A.makeCompressed();
  }
};

void add_common_stage_constraints(ModelBuilder& mb,
                                  const StageData& data,
                                  const DistributionResilienceOptions& opts,
                                  const std::vector<int>& forced_open_edges,
                                  const std::vector<int>* stage1_z,
                                  const std::vector<int>* previous_beta,
                                  bool stage2) {
  const int nb = static_cast<int>(data.buses.size());
  const int ne = static_cast<int>(data.edges.size());
  const double M = data.big_m;
  std::set<int> forced(forced_open_edges.begin(), forced_open_edges.end());
  for (int e = 0; e < ne; ++e) {
    const auto& edge = data.edges[static_cast<size_t>(e)];
    mb.add_le({{mb.idx.beta_at(e), 1.0}}, forced.count(e) ? 0.0 : 1.0);
    if (!forced.count(e)) {
      const double alpha = edge.initial_closed ? 1.0 : 0.0;
      const bool enforce_switch_constraints = opts.require_switch_for_nonfault_branch_operation &&
                                              !opts.allow_branch_operation_without_switch;
      if (stage2 && previous_beta != nullptr) {
        const double prev = (*previous_beta)[static_cast<size_t>(e)];
        if (stage1_z != nullptr) {
          const double zu = (*stage1_z)[static_cast<size_t>(edge.from)];
          const double zv = (*stage1_z)[static_cast<size_t>(edge.to)];
          if (zu > 0.5 || zv > 0.5) {
            mb.add_le({{mb.idx.beta_at(e), 1.0}}, 0.0);
            continue;
          }
        }
        if (enforce_switch_constraints && !edge.switchable) {
          mb.add_le({{mb.idx.beta_at(e), 1.0}}, alpha);
          mb.add_le({{mb.idx.beta_at(e), -1.0}}, -alpha);
        } else if (edge.tie || !edge.initial_closed) {
          if (opts.allow_stage2_close_ties && edge.switchable) {
            mb.add_le({{mb.idx.beta_at(e), 1.0}}, 1.0);
          } else {
            mb.add_le({{mb.idx.beta_at(e), 1.0}}, alpha);
          }
        } else {
          const double r = edge.switchable ? 1.0 : 0.0;
          mb.add_le({{mb.idx.beta_at(e), 1.0}}, prev + r);
          mb.add_le({{mb.idx.beta_at(e), -1.0}}, -(prev - r));
        }
      } else {
        if (enforce_switch_constraints && !edge.switchable) {
          mb.add_le({{mb.idx.beta_at(e), 1.0}}, alpha);
          mb.add_le({{mb.idx.beta_at(e), -1.0}}, -alpha);
        } else if (edge.initial_closed && edge.switchable && opts.allow_stage1_open_switches) {
          mb.add_le({{mb.idx.beta_at(e), 1.0}}, 1.0);
        } else {
          mb.add_le({{mb.idx.beta_at(e), 1.0}}, alpha);
          mb.add_le({{mb.idx.beta_at(e), -1.0}}, -alpha);
        }
      }
    }
    mb.add_le({{mb.idx.flow_at(e), 1.0}, {mb.idx.beta_at(e), -M}}, 0.0);
    mb.add_le({{mb.idx.flow_at(e), -1.0}, {mb.idx.beta_at(e), -M}}, 0.0);
  }

  for (int i = 0; i < nb; ++i) {
    const auto& bus = data.buses[static_cast<size_t>(i)];
    mb.add_le({{mb.idx.shed_at(i), 1.0}}, bus.demand_mw);
    mb.add_le({{mb.idx.shed_at(i), -1.0}, {mb.idx.energized_at(i), -bus.demand_mw}}, -bus.demand_mw);
    mb.add_le({{mb.idx.gen_at(i), 1.0}}, bus.gen_cap_mw + (bus.source ? M : 0.0));
    mb.add_le({{mb.idx.gen_at(i), 1.0}, {mb.idx.energized_at(i), -M}}, 0.0);
    if (stage1_z) {
      const double z = (*stage1_z)[static_cast<size_t>(i)];
      if (z > 0.5) {
        mb.add_le({{mb.idx.energized_at(i), 1.0}}, 0.0);
      }
    }
    if (!bus.source && bus.gen_cap_mw <= kEps) {
      // Non-source buses may still be energized through network flow.
    }
  }

  for (int i = 0; i < nb; ++i) {
    std::vector<std::pair<int, double>> balance{{mb.idx.gen_at(i), 1.0}, {mb.idx.shed_at(i), 1.0}};
    for (int e = 0; e < ne; ++e) {
      const auto& edge = data.edges[static_cast<size_t>(e)];
      if (edge.to == i) balance.push_back({mb.idx.flow_at(e), 1.0});
      if (edge.from == i) balance.push_back({mb.idx.flow_at(e), -1.0});
    }
    mb.add_eq(balance, data.buses[static_cast<size_t>(i)].demand_mw);
  }
}

StageSolution solve_stage_milp(const StageData& data,
                               const DistributionResilienceOptions& opts,
                               const std::vector<int>& forced_open_edges,
                               const std::vector<int>* prev_beta,
                               const std::vector<int>* stage1_z,
                               bool stage2) {
  ModelBuilder mb;
  const int nb = static_cast<int>(data.buses.size());
  const int ne = static_cast<int>(data.edges.size());
  mb.idx.n_bus = nb;
  mb.idx.n_edge = ne;
  mb.idx.beta.reserve(ne);
  mb.idx.flow.reserve(ne);
  mb.idx.z.reserve(nb);
  mb.idx.energized.reserve(nb);
  mb.idx.shed.reserve(nb);
  mb.idx.gen.reserve(nb);
  mb.idx.virt.reserve(nb);

  for (int e = 0; e < ne; ++e) {
    mb.idx.beta.push_back(mb.add_var(engine::VarType::Binary, 0.0, 1.0, "beta_" + std::to_string(e),
                                     stage2 ? 0.0 : -1.0));
    mb.idx.flow.push_back(mb.add_var(engine::VarType::Continuous, -data.big_m, data.big_m,
                                     "p_" + std::to_string(e), 0.0));
  }
  for (int i = 0; i < nb; ++i) {
    mb.idx.z.push_back(mb.add_var(engine::VarType::Binary, 0.0, 1.0, "z_" + std::to_string(i),
                                  stage2 ? 0.0 : 1.0));
    mb.idx.energized.push_back(mb.add_var(engine::VarType::Binary, 0.0, 1.0,
                                          "en_" + std::to_string(i), 0.0));
    const auto& bus = data.buses[static_cast<size_t>(i)];
    const double shed_cost = (stage2 ? 10.0 : 100.0) * std::max(1.0, bus.importance);
    mb.idx.shed.push_back(mb.add_var(engine::VarType::Continuous, 0.0, bus.demand_mw,
                                     "shed_" + std::to_string(i), shed_cost));
    mb.idx.gen.push_back(mb.add_var(engine::VarType::Continuous, 0.0, data.big_m,
                                    "gen_" + std::to_string(i), 0.0));
    mb.idx.virt.push_back(mb.add_var(engine::VarType::Binary, 0.0, 1.0,
                                     "virt_" + std::to_string(i), stage2 ? 1.0 : 0.0));
  }

  add_common_stage_constraints(mb, data, opts, forced_open_edges, stage1_z, prev_beta, stage2);

  if (!stage2) {
    if (!data.uses_virtual_branch_switches) {
      for (int edge_pos : forced_open_edges) {
        if (edge_pos < 0 || edge_pos >= ne) continue;
        const auto& e = data.edges[static_cast<size_t>(edge_pos)];
        mb.add_le({{mb.idx.z_at(e.from), -1.0}}, -1.0);
        mb.add_le({{mb.idx.z_at(e.to), -1.0}}, -1.0);
      }
      for (int e = 0; e < ne; ++e) {
        const auto& edge = data.edges[static_cast<size_t>(e)];
        // If a closed line remains closed, both end nodes must share fault-zone status.
        mb.add_le({{mb.idx.z_at(edge.from), 1.0}, {mb.idx.z_at(edge.to), -1.0}, {mb.idx.beta_at(e), 1.0}}, 1.0);
        mb.add_le({{mb.idx.z_at(edge.from), -1.0}, {mb.idx.z_at(edge.to), 1.0}, {mb.idx.beta_at(e), 1.0}}, 1.0);
      }
    }
  } else if (stage1_z != nullptr) {
    for (int e = 0; e < ne; ++e) {
      const auto& edge = data.edges[static_cast<size_t>(e)];
      const double zu = (*stage1_z)[static_cast<size_t>(edge.from)];
      const double zv = (*stage1_z)[static_cast<size_t>(edge.to)];
      if (std::abs(zu - zv) > 0.5) mb.add_le({{mb.idx.beta_at(e), 1.0}}, 0.0);
    }
  }

  mb.finalize(opts);

  solver::BCOptions bc_opts;
  bc_opts.time_limit_sec = static_cast<double>(opts.mip.max_time_s);
  bc_opts.max_nodes = opts.mip.max_nodes;
  bc_opts.gap_tol = opts.mip.mip_gap;
  if (opts.mip.num_threads > 0) {
    bc_opts.num_threads = opts.mip.num_threads;
  } else {
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (hw >= 4) bc_opts.num_threads = std::min(hw, 8);
  }
  bc_opts.branching = solver::BranchingStrategy::Pseudocost;
  bc_opts.node_sel = to_bc_node_selection(opts.mip.native_node_selection);
  bc_opts.use_feasibility_pump = true;
  bc_opts.verbose = opts.mip.verbose;

  engine::SolveResult solve_result;
  switch (opts.mip.solver) {
    case DistributionResilienceMIPSolver::Native: {
      engine::NativeBranchAndCutAdapter adapter(bc_opts);
      solve_result = adapter.solve_milp(mb.model);
      break;
    }
    case DistributionResilienceMIPSolver::HiGHS: {
      engine::StrictHighsBranchAndCutAdapter adapter(bc_opts);
      solve_result = adapter.solve_milp(mb.model);
      break;
    }
    case DistributionResilienceMIPSolver::Gurobi: {
      engine::GurobiAdapter adapter;
      solve_result = adapter.solve_milp(mb.model);
      break;
    }
  }

  StageSolution sol;
  sol.stats.num_variables = static_cast<int>(mb.model.linear_part.vars.size());
  sol.stats.num_binary_variables = static_cast<int>(mb.model.binary_idx.size());
  sol.stats.num_integer_variables = static_cast<int>(mb.model.integer_idx.size());
  sol.stats.num_eq_constraints = static_cast<int>(mb.model.linear_part.Aeq.rows());
  sol.stats.num_ineq_constraints = static_cast<int>(mb.model.linear_part.A.rows());
  sol.stats.objective_value = solve_result.stats.objective;
  sol.stats.mip_gap = solve_result.stats.mip_gap;
  sol.stats.runtime_sec = solve_result.stats.runtime_sec;
  sol.stats.solver_status = solve_result.stats.status;
  sol.feasible = solve_result.stats.success && solve_result.x.size() == mb.model.linear_part.c.size();
  sol.status = solve_result.stats.status;
  sol.objective = solve_result.stats.objective;
  sol.beta.assign(static_cast<size_t>(ne), 0);
  sol.z.assign(static_cast<size_t>(nb), 0);
  sol.shed.assign(static_cast<size_t>(nb), 0.0);
  if (!sol.feasible) return sol;
  for (int e = 0; e < ne; ++e) sol.beta[static_cast<size_t>(e)] = solve_result.x[mb.idx.beta_at(e)] > 0.5 ? 1 : 0;
  for (int i = 0; i < nb; ++i) {
    sol.z[static_cast<size_t>(i)] = solve_result.x[mb.idx.z_at(i)] > 0.5 ? 1 : 0;
    sol.shed[static_cast<size_t>(i)] = std::max(0.0, solve_result.x[mb.idx.shed_at(i)]);
  }
  for (int i = 0; i < nb; ++i) {
    if (sol.z[static_cast<size_t>(i)] <= 0) continue;
    if (data.buses[static_cast<size_t>(i)].kind == ResilienceBranchKind::AC) {
      sol.fault_zone_ac_bus_ids.push_back(data.buses[static_cast<size_t>(i)].index);
    } else {
      sol.fault_zone_dc_bus_ids.push_back(data.buses[static_cast<size_t>(i)].index);
    }
  }
  sort_unique(sol.fault_zone_ac_bus_ids);
  sort_unique(sol.fault_zone_dc_bus_ids);
  populate_switch_state_from_beta(data, sol.beta, sol);
  return sol;
}

std::vector<int> component_labels(const StageData& data, const std::vector<int>& beta) {
  std::vector<int> comp(data.buses.size(), -1);
  std::vector<std::vector<int>> adj(data.buses.size());
  for (size_t e = 0; e < data.edges.size(); ++e) {
    if (e >= beta.size() || beta[e] == 0) continue;
    const auto& edge = data.edges[e];
    adj[static_cast<size_t>(edge.from)].push_back(edge.to);
    adj[static_cast<size_t>(edge.to)].push_back(edge.from);
  }
  int cid = 0;
  std::vector<int> stack;
  for (size_t i = 0; i < data.buses.size(); ++i) {
    if (comp[i] >= 0) continue;
    comp[i] = cid;
    stack = {static_cast<int>(i)};
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      for (int v : adj[static_cast<size_t>(u)]) {
        if (comp[static_cast<size_t>(v)] >= 0) continue;
        comp[static_cast<size_t>(v)] = cid;
        stack.push_back(v);
      }
    }
    ++cid;
  }
  return comp;
}

int count_islands(const StageData& data, const std::vector<int>& beta) {
  const auto comp = component_labels(data, beta);
  int n = 0;
  for (int c : comp) n = std::max(n, c + 1);
  return n;
}

double solution_shed_mw(const StageSolution& sol) {
  double total = 0.0;
  for (double shed : sol.shed) total += std::max(0.0, shed);
  return total;
}

std::string stage_mess_status_str(MobileStorageStatus st) {
  switch (st) {
    case MobileStorageStatus::InTransit: return "InTransit";
    case MobileStorageStatus::Deployed: return "Deployed";
    case MobileStorageStatus::Stationary: return "Stationary";
  }
  return "Stationary";
}

std::vector<StageMessState> collect_stage_mess(const HybridPowerSystem& sys,
                                               const StageData& data) {
  std::vector<StageMessState> out;
  out.reserve(sys.mobile_storage.size());
  for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
    const auto& st = sys.mobile_storage[i];
    if (!st.in_service) continue;
    if (data.ac_bus_pos.find(st.bus) == data.ac_bus_pos.end()) continue;
    StageMessState s;
    s.storage_index = st.index != 0 ? st.index : static_cast<int>(i + 1);
    s.ac_bus_id = st.bus;
    s.target_ac_bus_id = st.target_bus != 0 ? st.target_bus : st.bus;
    if (data.ac_bus_pos.find(s.target_ac_bus_id) == data.ac_bus_pos.end()) s.target_ac_bus_id = st.bus;
    s.status = st.status;
    s.pmax_mw = std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw});
    s.e_rated_mwh = std::max(0.0, st.e_rated_mwh);
    const double soc_lo = std::min(st.soc_min, st.soc_max);
    s.e_min_mwh = soc_lo * s.e_rated_mwh;
    s.e_max_mwh = std::max(0.0, s.e_rated_mwh);
    const double soc0 = std::max(soc_lo, st.soc_init);
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : soc0 * s.e_rated_mwh;
    s.energy_mwh = std::clamp(std::max(s.e_min_mwh, e0), s.e_min_mwh, s.e_max_mwh > kEps ? s.e_max_mwh : std::max(s.e_min_mwh, e0));
    s.eta_discharge = st.eta_discharge > 0.0 ? st.eta_discharge : 1.0;
    s.e_consumption_mwh_km = std::max(0.0, st.e_consumption_mwh_km);
    s.max_travel_distance_km = st.max_travel_distance_km > 0.0 ? st.max_travel_distance_km : kInf;
    s.arrival_time_hr = st.arrival_time;
    out.push_back(s);
  }
  return out;
}

void apply_stage_mess_dispatch(const StageData& data,
                               const StageSolution& sol,
                               double dt_hr,
                               bool allow_dispatch,
                               std::vector<StageMessState>& mess,
                               DistributionResilienceStepResult& sr,
                               double& delivered_mwh) {
  delivered_mwh = 0.0;
  for (auto& st : mess) st.dispatch_mw = 0.0;
  if (mess.empty()) return;

  if (allow_dispatch && dt_hr > kEps && sr.shed_mw > kEps) {
    const auto comp = component_labels(data, sol.beta);
    std::vector<size_t> bus_order(data.buses.size());
    std::iota(bus_order.begin(), bus_order.end(), size_t{0});
    std::sort(bus_order.begin(), bus_order.end(), [&](size_t a, size_t b) {
      const auto& ba = data.buses[a];
      const auto& bb = data.buses[b];
      if (std::abs(ba.importance - bb.importance) > kEps) return ba.importance > bb.importance;
      return ba.demand_mw > bb.demand_mw;
    });

    for (auto& st : mess) {
      if (st.status == MobileStorageStatus::InTransit) continue;
      const auto pos_it = data.ac_bus_pos.find(st.ac_bus_id);
      if (pos_it == data.ac_bus_pos.end()) continue;
      const int mess_pos = pos_it->second;
      if (mess_pos < 0 || mess_pos >= static_cast<int>(comp.size())) continue;
      const int mess_comp = comp[static_cast<size_t>(mess_pos)];
      double deliverable_mw = std::min(st.pmax_mw,
          std::max(0.0, (st.energy_mwh - st.e_min_mwh) * st.eta_discharge / dt_hr));
      double dispatched = 0.0;
      for (size_t bus_pos : bus_order) {
        if (deliverable_mw <= kEps) break;
        if (bus_pos >= comp.size() || comp[bus_pos] != mess_comp) continue;
        if (bus_pos >= sr.bus_supply_shed_mw.size() || bus_pos >= sr.bus_supply_served_mw.size()) continue;
        const double serve = std::min(deliverable_mw, std::max(0.0, sr.bus_supply_shed_mw[bus_pos]));
        if (serve <= kEps) continue;
        sr.bus_supply_shed_mw[bus_pos] -= serve;
        sr.bus_supply_served_mw[bus_pos] += serve;
        sr.shed_mw = std::max(0.0, sr.shed_mw - serve);
        sr.served_mw += serve;
        const double importance = bus_pos < data.buses.size() ? std::max(1.0, data.buses[bus_pos].importance) : 1.0;
        sr.weighted_shed_mw = std::max(0.0, sr.weighted_shed_mw - serve * importance);
        const int tier = bus_pos < data.buses.size() ? std::clamp(data.buses[bus_pos].priority_tier, 0, 3) : 3;
        if (static_cast<size_t>(tier) < sr.shed_by_priority.size()) {
          sr.shed_by_priority[static_cast<size_t>(tier)] =
              std::max(0.0, sr.shed_by_priority[static_cast<size_t>(tier)] - serve);
        }
        deliverable_mw -= serve;
        dispatched += serve;
      }
      if (dispatched > kEps) {
        st.dispatch_mw = dispatched;
        st.energy_mwh = std::max(st.e_min_mwh, st.energy_mwh - dispatched * dt_hr / st.eta_discharge);
        delivered_mwh += dispatched * dt_hr;
      }
    }
    sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;
  }

  for (const auto& st : mess) {
    MESSStateStep ms;
    ms.storage_index = st.storage_index;
    ms.bus = st.ac_bus_id;
    ms.target_bus = st.target_ac_bus_id;
    ms.status = stage_mess_status_str(st.status);
    ms.dispatch_mw = st.dispatch_mw;
    ms.energy_mwh = st.energy_mwh;
    ms.soc = st.e_rated_mwh > kEps ? st.energy_mwh / st.e_rated_mwh : 0.0;
    ms.arrival_time_hr = st.arrival_time_hr;
    ms.remaining_travel_hr = st.remaining_travel_hr;
    sr.mess_states.push_back(ms);
  }
}

struct StageMessArc {
  int from_pos{0};
  int to_pos{0};
  int depart_t{0};
  int arrive_t{0};
  double distance_km{0.0};
  double travel_mwh{0.0};
  bool is_stay{false};
};

struct StageMessOnlyIndex {
  int T{0};
  int n_bus{0};
  int n_mess{0};
  std::vector<int> energy;
  std::vector<int> loc;
  std::vector<int> dispatch;
  std::vector<int> residual;
  std::vector<std::vector<int>> arc;
  int e_at(int m, int t) const { return energy[static_cast<size_t>(m * (T + 1) + t)]; }
  int x_at(int m, int b, int t) const { return loc[static_cast<size_t>((t * n_mess + m) * n_bus + b)]; }
  int d_at(int m, int b, int t) const { return dispatch[static_cast<size_t>((t * n_mess + m) * n_bus + b)]; }
  int r_at(int b, int t) const { return residual[static_cast<size_t>(t * n_bus + b)]; }
};

struct StageMessOnlySolution {
  bool feasible{false};
  std::string status;
  StageSolveStats stats;
  std::vector<std::vector<MESSStateStep>> mess_states_by_step;
  std::vector<std::unordered_map<int, double>> served_mw_by_ac_bus;
  double delivered_mwh{0.0};
  double travel_distance_km{0.0};
};

using StageMessAdj = std::vector<std::vector<std::pair<int, double>>>;

StageMessAdj build_stage_mess_transport_graph(const HybridPowerSystem& sys,
                                              const DistributionResilienceOptions& opts,
                                              const std::unordered_map<int, int>& ac_bus_pos) {
  StageMessAdj graph(ac_bus_pos.size());
  auto add_edge = [&](int from_bus, int to_bus, double dist_km) {
    const auto it_f = ac_bus_pos.find(from_bus);
    const auto it_t = ac_bus_pos.find(to_bus);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) return;
    const double w = dist_km > kEps ? dist_km : 1.0;
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
    for (const auto& br : sys.ac.branches) {
      if (!br.in_service) continue;
      add_edge(br.from_bus, br.to_bus, br.length_km);
    }
  }
  return graph;
}

std::vector<double> stage_mess_shortest_distances(const StageMessAdj& graph, int start) {
  std::vector<double> dist(graph.size(), kInf);
  if (start < 0 || start >= static_cast<int>(graph.size())) return dist;
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

std::string stage_mess_solver_name(DistributionResilienceMIPSolver solver, int num_threads) {
  return stage_solver_name(solver, num_threads);
}

engine::SolveResult solve_stage_mess_model(const engine::MIPModel& model,
                                           const DistributionResilienceOptions& opts) {
  solver::BCOptions bc_opts;
  bc_opts.time_limit_sec = static_cast<double>(opts.mip.max_time_s);
  bc_opts.max_nodes = opts.mip.max_nodes;
  bc_opts.gap_tol = opts.mip.mip_gap;
  if (opts.mip.num_threads > 0) {
    bc_opts.num_threads = opts.mip.num_threads;
  } else {
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (hw >= 4) bc_opts.num_threads = std::min(hw, 8);
  }
  bc_opts.branching = solver::BranchingStrategy::Pseudocost;
  bc_opts.node_sel = to_bc_node_selection(opts.mip.native_node_selection);
  bc_opts.use_feasibility_pump = true;
  bc_opts.verbose = opts.mip.verbose;

  switch (opts.mip.solver) {
    case DistributionResilienceMIPSolver::Native: {
      engine::NativeBranchAndCutAdapter adapter(bc_opts);
      return adapter.solve_milp(model);
    }
    case DistributionResilienceMIPSolver::HiGHS: {
      engine::StrictHighsBranchAndCutAdapter adapter(bc_opts);
      return adapter.solve_milp(model);
    }
    case DistributionResilienceMIPSolver::Gurobi: {
      engine::GurobiAdapter adapter;
      return adapter.solve_milp(model);
    }
  }
  return {};
}

StageMessOnlySolution make_stationary_mess_solution(const std::vector<StageMessState>& mess,
                                                    int steps,
                                                    double dt_hr,
                                                    const std::string& status) {
  StageMessOnlySolution sol;
  sol.feasible = true;
  sol.status = status;
  sol.mess_states_by_step.assign(static_cast<size_t>(steps), {});
  sol.served_mw_by_ac_bus.assign(static_cast<size_t>(steps), {});
  for (int t = 0; t < steps; ++t) {
    auto& states = sol.mess_states_by_step[static_cast<size_t>(t)];
    states.reserve(mess.size());
    for (const auto& st : mess) {
      MESSStateStep ms;
      ms.storage_index = st.storage_index;
      ms.bus = st.ac_bus_id;
      ms.target_bus = st.ac_bus_id;
      ms.status = "Stationary";
      ms.dispatch_mw = 0.0;
      ms.energy_mwh = st.energy_mwh;
      ms.soc = st.e_rated_mwh > kEps ? st.energy_mwh / st.e_rated_mwh : 0.0;
      ms.arrival_time_hr = static_cast<double>(t) * dt_hr;
      ms.remaining_travel_hr = 0.0;
      states.push_back(ms);
    }
  }
  return sol;
}

StageMessOnlySolution make_greedy_ra_residual_mess_solution(
    const std::vector<int>& ac_bus_ids,
    const std::unordered_map<int, int>& ac_bus_pos,
    const std::vector<std::vector<double>>& residual,
    const std::vector<std::vector<double>>& weight,
    const std::vector<std::vector<double>>& dist,
    const std::vector<StageMessState>& mess,
    const DistributionResilienceOptions& opts,
    const std::string& status) {
  const int T = static_cast<int>(residual.size());
  const int nb = static_cast<int>(ac_bus_ids.size());
  const double dt = opts.time_step_hr;
  StageMessOnlySolution sol;
  sol.feasible = true;
  sol.status = status;
  sol.mess_states_by_step.assign(static_cast<size_t>(T), {});
  sol.served_mw_by_ac_bus.assign(static_cast<size_t>(T), {});
  if (T <= 0 || nb <= 0 || dt <= kEps) return sol;

  std::vector<std::vector<double>> remaining = residual;
  const double max_step_dist = std::max(opts.mess_travel_speed_kmph, 1.0) * dt;

  for (const auto& st : mess) {
    int current_pos = 0;
    if (const auto it = ac_bus_pos.find(st.ac_bus_id); it != ac_bus_pos.end()) {
      current_pos = it->second;
    }
    double energy_mwh = st.energy_mwh;
    double traveled_km = 0.0;

    for (int t = 0; t < T; ++t) {
      MESSStateStep ms;
      ms.storage_index = st.storage_index;
      ms.bus = ac_bus_ids[static_cast<size_t>(current_pos)];
      ms.target_bus = ms.bus;
      ms.status = "Stationary";
      ms.arrival_time_hr = static_cast<double>(t) * dt;
      ms.remaining_travel_hr = 0.0;

      const double usable_mwh = std::max(0.0, energy_mwh - st.e_min_mwh);
      const double dispatch_cap_mw = dt > kEps
          ? std::min(st.pmax_mw, usable_mwh * std::max(st.eta_discharge, 1.0e-6) / dt)
          : 0.0;
      const double local_need = remaining[static_cast<size_t>(t)][static_cast<size_t>(current_pos)];
      const double dispatch_mw = std::min(dispatch_cap_mw, std::max(0.0, local_need));
      if (dispatch_mw > 1.0e-7) {
        remaining[static_cast<size_t>(t)][static_cast<size_t>(current_pos)] =
            std::max(0.0, local_need - dispatch_mw);
        sol.served_mw_by_ac_bus[static_cast<size_t>(t)][ms.bus] += dispatch_mw;
        energy_mwh = std::max(st.e_min_mwh,
                              energy_mwh - dispatch_mw * dt / std::max(st.eta_discharge, 1.0e-6));
        sol.delivered_mwh += dispatch_mw * dt;
        ms.dispatch_mw = dispatch_mw;
        ms.status = "Deployed";
      } else if (t + 1 < T && usable_mwh > kEps) {
        auto future_score = [&](int bus_pos) {
          double score = 0.0;
          for (int tau = t + 1; tau < T; ++tau) {
            score += remaining[static_cast<size_t>(tau)][static_cast<size_t>(bus_pos)] *
                     weight[static_cast<size_t>(tau)][static_cast<size_t>(bus_pos)] * dt;
          }
          return score;
        };

        int best_pos = current_pos;
        double best_score = future_score(current_pos);
        for (int b = 0; b < nb; ++b) {
          if (b == current_pos) continue;
          const double d = dist[static_cast<size_t>(current_pos)][static_cast<size_t>(b)];
          if (!std::isfinite(d) || d <= kEps || d > max_step_dist + kEps) continue;
          if (traveled_km + d > st.max_travel_distance_km + kEps) continue;
          const double travel_mwh = d * st.e_consumption_mwh_km;
          if (energy_mwh - travel_mwh < st.e_min_mwh - kEps) continue;
          const double score = future_score(b);
          if (score > best_score + 1.0e-9) {
            best_score = score;
            best_pos = b;
          }
        }

        if (best_pos != current_pos) {
          const double d = dist[static_cast<size_t>(current_pos)][static_cast<size_t>(best_pos)];
          energy_mwh = std::max(st.e_min_mwh, energy_mwh - d * st.e_consumption_mwh_km);
          traveled_km += d;
          sol.travel_distance_km += d;
          ms.target_bus = ac_bus_ids[static_cast<size_t>(best_pos)];
          ms.status = "InTransit";
          ms.remaining_travel_hr = dt;
          ms.arrival_time_hr = static_cast<double>(t + 1) * dt;
          current_pos = best_pos;
        }
      }

      ms.energy_mwh = energy_mwh;
      ms.soc = st.e_rated_mwh > kEps ? std::clamp(energy_mwh / st.e_rated_mwh, 0.0, 1.0) : 0.0;
      sol.mess_states_by_step[static_cast<size_t>(t)].push_back(ms);
    }
  }

  return sol;
}

StageMessOnlySolution solve_ra_residual_mess_milp(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts,
    const std::vector<DistributionResilienceStepResult>& base_steps,
    const std::vector<StageMessState>& mess) {
  const int T = static_cast<int>(base_steps.size());
  const double dt = opts.time_step_hr;
  if (T <= 0 || mess.empty() || dt <= kEps) {
    return make_stationary_mess_solution(mess, std::max(0, T), dt, "No MESS-only MILP input");
  }

  std::vector<int> ac_bus_ids;
  ac_bus_ids.reserve(sys.ac.buses.size());
  std::unordered_map<int, int> ac_bus_pos;
  for (const auto& b : sys.ac.buses) {
    if (!b.in_service) continue;
    if (ac_bus_pos.count(b.index)) continue;
    ac_bus_pos[b.index] = static_cast<int>(ac_bus_ids.size());
    ac_bus_ids.push_back(b.index);
  }
  const int nb = static_cast<int>(ac_bus_ids.size());
  const int nm = static_cast<int>(mess.size());
  if (nb == 0) return make_stationary_mess_solution(mess, T, dt, "No AC buses for MESS-only MILP");

  std::vector<std::vector<double>> residual(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(nb), 0.0));
  std::vector<std::vector<double>> weight(static_cast<size_t>(T), std::vector<double>(static_cast<size_t>(nb), 1.0));
  double total_residual_mwh = 0.0;
  for (int t = 0; t < T; ++t) {
    const auto& step = base_steps[static_cast<size_t>(t)];
    for (size_t i = 0; i < step.bus_supply_kind.size() && i < step.bus_supply_index.size(); ++i) {
      if (step.bus_supply_kind[i] != "AC") continue;
      const auto it = ac_bus_pos.find(step.bus_supply_index[i]);
      if (it == ac_bus_pos.end()) continue;
      const int b = it->second;
      const double shed = i < step.bus_supply_shed_mw.size() ? std::max(0.0, step.bus_supply_shed_mw[i]) : 0.0;
      const double importance = i < step.bus_supply_importance.size() ? std::max(1.0, step.bus_supply_importance[i]) : 1.0;
      residual[static_cast<size_t>(t)][static_cast<size_t>(b)] += shed;
      weight[static_cast<size_t>(t)][static_cast<size_t>(b)] = std::max(weight[static_cast<size_t>(t)][static_cast<size_t>(b)], importance);
      total_residual_mwh += shed * dt;
    }
  }
  if (total_residual_mwh <= kEps) {
    return make_stationary_mess_solution(mess, T, dt, "No RA residual shed for MESS-only MILP");
  }

  const auto graph = build_stage_mess_transport_graph(sys, opts, ac_bus_pos);
  std::vector<std::vector<double>> dist(static_cast<size_t>(nb), std::vector<double>(static_cast<size_t>(nb), kInf));
  for (int b = 0; b < nb; ++b) dist[static_cast<size_t>(b)] = stage_mess_shortest_distances(graph, b);

  std::vector<std::vector<StageMessArc>> arcs(static_cast<size_t>(nm));
  const double max_step_dist = std::max(opts.mess_travel_speed_kmph, 1.0) * dt;
  for (int m = 0; m < nm; ++m) {
    auto& ma = arcs[static_cast<size_t>(m)];
    for (int t = 0; t < T - 1; ++t) {
      for (int i = 0; i < nb; ++i) {
        ma.push_back({i, i, t, t + 1, 0.0, 0.0, true});
        for (int j = 0; j < nb; ++j) {
          if (i == j) continue;
          const double d = dist[static_cast<size_t>(i)][static_cast<size_t>(j)];
          if (!std::isfinite(d) || d <= kEps || d > max_step_dist + kEps) continue;
          if (d > mess[static_cast<size_t>(m)].max_travel_distance_km + kEps) continue;
          ma.push_back({i, j, t, t + 1, d, d * mess[static_cast<size_t>(m)].e_consumption_mwh_km, false});
        }
      }
    }
  }

  ModelBuilder mb;
  StageMessOnlyIndex idx;
  idx.T = T;
  idx.n_bus = nb;
  idx.n_mess = nm;
  idx.arc.resize(static_cast<size_t>(nm));

  auto add_var = [&](engine::VarType type, double lb, double ub, const std::string& name, double obj) {
    return mb.add_var(type, lb, ub, name, obj);
  };

  for (int m = 0; m < nm; ++m) {
    const auto& st = mess[static_cast<size_t>(m)];
    for (int t = 0; t <= T; ++t) {
      idx.energy.push_back(add_var(engine::VarType::Continuous, st.e_min_mwh, std::max(st.e_min_mwh, st.e_max_mwh),
                                   "mre_" + std::to_string(m) + "_" + std::to_string(t), 0.0));
    }
  }
  constexpr double kResidualPenaltyScale = 1000.0;
  constexpr double kDispatchTieCost = 1.0e-4;
  constexpr double kMoveTieCost = 1.0e-4;
  for (int t = 0; t < T; ++t) {
    for (int m = 0; m < nm; ++m) {
      for (int b = 0; b < nb; ++b) {
        idx.loc.push_back(add_var(engine::VarType::Binary, 0.0, 1.0,
                                  "mrx_" + std::to_string(m) + "_" + std::to_string(b) + "_" + std::to_string(t), 0.0));
        idx.dispatch.push_back(add_var(engine::VarType::Continuous, 0.0, mess[static_cast<size_t>(m)].pmax_mw,
                                       "mrd_" + std::to_string(m) + "_" + std::to_string(b) + "_" + std::to_string(t),
                                       kDispatchTieCost * dt));
      }
    }
    for (int b = 0; b < nb; ++b) {
      idx.residual.push_back(add_var(engine::VarType::Continuous, 0.0, residual[static_cast<size_t>(t)][static_cast<size_t>(b)],
                                     "mrr_" + std::to_string(b) + "_" + std::to_string(t),
                                     kResidualPenaltyScale * weight[static_cast<size_t>(t)][static_cast<size_t>(b)] * dt));
    }
  }
  for (int m = 0; m < nm; ++m) {
    auto& ids = idx.arc[static_cast<size_t>(m)];
    ids.reserve(arcs[static_cast<size_t>(m)].size());
    for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
      const auto& arc = arcs[static_cast<size_t>(m)][a];
      const double obj = arc.is_stay ? 0.0 : opts.mip.mess_travel_cost_per_km * arc.distance_km + kMoveTieCost;
      ids.push_back(add_var(engine::VarType::Binary, 0.0, 1.0,
                            "mra_" + std::to_string(m) + "_" + std::to_string(a), obj));
    }
  }

  for (int m = 0; m < nm; ++m) {
    const auto& st = mess[static_cast<size_t>(m)];
    mb.add_eq({{idx.e_at(m, 0), 1.0}}, st.energy_mwh);
    const auto init_it = ac_bus_pos.find(st.ac_bus_id);
    for (int b = 0; b < nb; ++b) {
      mb.add_eq({{idx.x_at(m, b, 0), 1.0}}, init_it != ac_bus_pos.end() && b == init_it->second ? 1.0 : 0.0);
    }
    for (int t = 0; t < T; ++t) {
      std::vector<std::pair<int, double>> one_loc;
      for (int b = 0; b < nb; ++b) one_loc.push_back({idx.x_at(m, b, t), 1.0});
      mb.add_eq(one_loc, 1.0);
      for (int b = 0; b < nb; ++b) {
        if (t < T - 1) {
          std::vector<std::pair<int, double>> stay_terms{{idx.d_at(m, b, t), 1.0}};
          for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
            const auto& arc = arcs[static_cast<size_t>(m)][a];
            if (arc.depart_t == t && arc.from_pos == b && arc.is_stay) {
              stay_terms.push_back({idx.arc[static_cast<size_t>(m)][a], -st.pmax_mw});
            }
          }
          mb.add_le(stay_terms, 0.0);
        } else {
          mb.add_le({{idx.d_at(m, b, t), 1.0}, {idx.x_at(m, b, t), -st.pmax_mw}}, 0.0);
        }
      }
      std::vector<std::pair<int, double>> energy_terms{{idx.e_at(m, t + 1), 1.0}, {idx.e_at(m, t), -1.0}};
      for (int b = 0; b < nb; ++b) energy_terms.push_back({idx.d_at(m, b, t), dt / std::max(st.eta_discharge, 1.0e-6)});
      if (t < T - 1) {
        for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
          const auto& arc = arcs[static_cast<size_t>(m)][a];
          if (arc.depart_t == t) energy_terms.push_back({idx.arc[static_cast<size_t>(m)][a], arc.travel_mwh});
        }
      }
      mb.add_eq(energy_terms, 0.0);
    }
    std::vector<std::pair<int, double>> travel_terms;
    for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
      travel_terms.push_back({idx.arc[static_cast<size_t>(m)][a], arcs[static_cast<size_t>(m)][a].distance_km});
    }
    if (!travel_terms.empty() && std::isfinite(st.max_travel_distance_km)) mb.add_le(travel_terms, st.max_travel_distance_km);

    for (int t = 0; t < T - 1; ++t) {
      for (int b = 0; b < nb; ++b) {
        std::vector<std::pair<int, double>> depart{{idx.x_at(m, b, t), -1.0}};
        std::vector<std::pair<int, double>> arrive{{idx.x_at(m, b, t + 1), -1.0}};
        for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
          const auto& arc = arcs[static_cast<size_t>(m)][a];
          if (arc.depart_t == t && arc.from_pos == b) depart.push_back({idx.arc[static_cast<size_t>(m)][a], 1.0});
          if (arc.arrive_t == t + 1 && arc.to_pos == b) arrive.push_back({idx.arc[static_cast<size_t>(m)][a], 1.0});
        }
        mb.add_eq(depart, 0.0);
        mb.add_eq(arrive, 0.0);
      }
    }
  }
  for (int t = 0; t < T; ++t) {
    for (int b = 0; b < nb; ++b) {
      std::vector<std::pair<int, double>> terms{{idx.r_at(b, t), 1.0}};
      for (int m = 0; m < nm; ++m) terms.push_back({idx.d_at(m, b, t), 1.0});
      mb.add_eq(terms, residual[static_cast<size_t>(t)][static_cast<size_t>(b)]);
    }
  }
  mb.finalize(opts);

  auto solve_result = solve_stage_mess_model(mb.model, opts);
  StageMessOnlySolution sol;
  sol.stats.num_variables = static_cast<int>(mb.model.linear_part.vars.size());
  sol.stats.num_binary_variables = static_cast<int>(mb.model.binary_idx.size());
  sol.stats.num_integer_variables = static_cast<int>(mb.model.integer_idx.size());
  sol.stats.num_eq_constraints = static_cast<int>(mb.model.linear_part.Aeq.rows());
  sol.stats.num_ineq_constraints = static_cast<int>(mb.model.linear_part.A.rows());
  sol.stats.objective_value = solve_result.stats.objective;
  sol.stats.mip_gap = solve_result.stats.mip_gap;
  sol.stats.runtime_sec = solve_result.stats.runtime_sec;
  sol.stats.solver_status = solve_result.stats.status;
  sol.feasible = solve_result.stats.success && solve_result.x.size() == mb.model.linear_part.c.size();
  sol.status = solve_result.stats.status;
  auto greedy_repair = [&]() {
    StageMessOnlySolution greedy = make_greedy_ra_residual_mess_solution(
        ac_bus_ids, ac_bus_pos, residual, weight, dist, mess, opts,
        sol.status.empty()
            ? "Constructive RA residual MESS routing fallback"
            : sol.status + "; constructive RA residual MESS routing fallback");
    greedy.stats = sol.stats;
    return greedy;
  };
  if (!sol.feasible) {
    StageMessOnlySolution greedy = greedy_repair();
    if (greedy.delivered_mwh > kEps || greedy.travel_distance_km > kEps) return greedy;
    return sol;
  }

  const auto& x = solve_result.x;
  sol.mess_states_by_step.assign(static_cast<size_t>(T), {});
  sol.served_mw_by_ac_bus.assign(static_cast<size_t>(T), {});
  for (int t = 0; t < T; ++t) {
    auto& states = sol.mess_states_by_step[static_cast<size_t>(t)];
    states.reserve(static_cast<size_t>(nm));
    for (int m = 0; m < nm; ++m) {
      const auto& st = mess[static_cast<size_t>(m)];
      int bus_pos = ac_bus_pos.count(st.ac_bus_id) ? ac_bus_pos.at(st.ac_bus_id) : 0;
      double best = -1.0;
      for (int b = 0; b < nb; ++b) {
        const double xv = x[idx.x_at(m, b, t)];
        if (xv > best) {
          best = xv;
          bus_pos = b;
        }
      }
      double dispatch = 0.0;
      for (int b = 0; b < nb; ++b) {
        const double dval = std::max(0.0, x[idx.d_at(m, b, t)]);
        if (dval <= 1.0e-7) continue;
        dispatch += dval;
        sol.served_mw_by_ac_bus[static_cast<size_t>(t)][ac_bus_ids[static_cast<size_t>(b)]] += dval;
      }
      const StageMessArc* moving = nullptr;
      if (t < T - 1) {
        for (size_t a = 0; a < arcs[static_cast<size_t>(m)].size(); ++a) {
          if (x[idx.arc[static_cast<size_t>(m)][a]] <= 0.5) continue;
          const auto& arc = arcs[static_cast<size_t>(m)][a];
          if (arc.depart_t != t || arc.is_stay) continue;
          moving = &arc;
          sol.travel_distance_km += arc.distance_km;
          break;
        }
      }
      MESSStateStep ms;
      ms.storage_index = st.storage_index;
      ms.bus = ac_bus_ids[static_cast<size_t>(bus_pos)];
      ms.target_bus = ms.bus;
      ms.dispatch_mw = dispatch;
      ms.energy_mwh = std::max(0.0, x[idx.e_at(m, t + 1)]);
      ms.soc = st.e_rated_mwh > kEps ? std::clamp(ms.energy_mwh / st.e_rated_mwh, 0.0, 1.0) : 0.0;
      ms.arrival_time_hr = static_cast<double>(t) * dt;
      ms.remaining_travel_hr = 0.0;
      ms.status = dispatch > kEps ? "Deployed" : "Stationary";
      if (moving != nullptr) {
        ms.bus = ac_bus_ids[static_cast<size_t>(moving->from_pos)];
        ms.target_bus = ac_bus_ids[static_cast<size_t>(moving->to_pos)];
        ms.status = "InTransit";
        ms.remaining_travel_hr = dt;
        ms.arrival_time_hr = static_cast<double>(t + 1) * dt;
      }
      states.push_back(ms);
      sol.delivered_mwh += dispatch * dt;
    }
  }
  if (total_residual_mwh > kEps && sol.delivered_mwh <= kEps) {
    StageMessOnlySolution greedy = greedy_repair();
    if (greedy.delivered_mwh > kEps || greedy.travel_distance_km > kEps) return greedy;
  }
  return sol;
}

void recompute_step_supply_totals(DistributionResilienceStepResult& sr) {
  sr.total_demand_mw = 0.0;
  sr.served_mw = 0.0;
  sr.shed_mw = 0.0;
  sr.weighted_shed_mw = 0.0;
  sr.shed_by_priority.assign(4, 0.0);
  for (size_t i = 0; i < sr.bus_supply_demand_mw.size(); ++i) {
    const double demand = std::max(0.0, sr.bus_supply_demand_mw[i]);
    const double served = i < sr.bus_supply_served_mw.size() ? std::max(0.0, sr.bus_supply_served_mw[i]) : 0.0;
    const double shed = i < sr.bus_supply_shed_mw.size() ? std::max(0.0, sr.bus_supply_shed_mw[i]) : std::max(0.0, demand - served);
    const double importance = i < sr.bus_supply_importance.size() ? std::max(1.0, sr.bus_supply_importance[i]) : 1.0;
    const int tier = i < sr.bus_supply_priority_tier.size() ? std::clamp(sr.bus_supply_priority_tier[i], 0, 3) : 3;
    sr.total_demand_mw += demand;
    sr.served_mw += served;
    sr.shed_mw += shed;
    sr.weighted_shed_mw += shed * importance;
    sr.shed_by_priority[static_cast<size_t>(tier)] += shed;
  }
  sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;
}

void apply_mess_only_solution_to_steps(const StageMessOnlySolution& mess_sol,
                                       std::vector<DistributionResilienceStepResult>& steps) {
  for (size_t t = 0; t < steps.size(); ++t) {
    auto& sr = steps[t];
    if (t < mess_sol.served_mw_by_ac_bus.size()) {
      for (const auto& [bus_id, served_total] : mess_sol.served_mw_by_ac_bus[t]) {
        double remaining = served_total;
        for (size_t i = 0; i < sr.bus_supply_kind.size() && i < sr.bus_supply_index.size(); ++i) {
          if (remaining <= kEps) break;
          if (sr.bus_supply_kind[i] != "AC" || sr.bus_supply_index[i] != bus_id) continue;
          if (i >= sr.bus_supply_shed_mw.size() || i >= sr.bus_supply_served_mw.size()) continue;
          const double serve = std::min(remaining, std::max(0.0, sr.bus_supply_shed_mw[i]));
          sr.bus_supply_shed_mw[i] -= serve;
          sr.bus_supply_served_mw[i] += serve;
          remaining -= serve;
        }
      }
    }
    if (t < mess_sol.mess_states_by_step.size()) sr.mess_states = mess_sol.mess_states_by_step[t];
    recompute_step_supply_totals(sr);
  }
}

void apply_stage_mess_dispatch_to_steps(const std::vector<StageData>& step_data,
                                        const std::vector<StageSolution>& step_solutions,
                                        const DistributionResilienceOptions& opts,
                                        std::vector<StageMessState> mess,
                                        std::vector<DistributionResilienceStepResult>& steps) {
  for (size_t t = 0; t < steps.size() && t < step_data.size() && t < step_solutions.size(); ++t) {
    double delivered = 0.0;
    apply_stage_mess_dispatch(step_data[t], step_solutions[t], opts.time_step_hr,
                              opts.allow_mess_dispatch, mess, steps[t], delivered);
  }
}

void accumulate_resilience_totals(DistributionResilienceResult& result,
                                  const DistributionResilienceOptions& opts) {
  result.total_switch_actions = 0;
  result.total_demand_mwh = 0.0;
  result.total_served_mwh = 0.0;
  result.total_shed_mwh = 0.0;
  result.weighted_unserved_mwh = 0.0;
  result.total_repaired_faults = 0;
  result.peak_shed_mw = 0.0;
  result.mess_energy_delivered_mwh = 0.0;
  for (const auto& sr : result.steps) {
    result.total_switch_actions += sr.switch_actions;
    result.total_demand_mwh += sr.total_demand_mw * opts.time_step_hr;
    result.total_served_mwh += sr.served_mw * opts.time_step_hr;
    result.total_shed_mwh += sr.shed_mw * opts.time_step_hr;
    result.weighted_unserved_mwh += sr.weighted_shed_mw * opts.time_step_hr;
    result.total_repaired_faults = std::max(result.total_repaired_faults, sr.repaired_faults);
    result.peak_shed_mw = std::max(result.peak_shed_mw, sr.shed_mw);
    for (const auto& ms : sr.mess_states) result.mess_energy_delivered_mwh += std::max(0.0, ms.dispatch_mw) * opts.time_step_hr;
  }
}

void apply_strict_mip_mess_dispatch(const DistributionResilienceStepResult& strict_step,
                                    double dt_hr,
                                    std::unordered_map<int, double>& projected_energy_mwh,
                                    std::unordered_map<int, double>& projected_energy_capacity_mwh,
                                    std::unordered_map<int, double>& last_strict_energy_mwh,
                                    std::unordered_map<int, double>& last_strict_dispatch_mw,
                                    std::unordered_map<int, double>& last_applied_dispatch_mw,
                                    std::unordered_map<int, bool>& last_strict_travel_evidence,
                                    std::unordered_map<int, int>& projected_bus,
                                    DistributionResilienceStepResult& sr,
                                    double& delivered_mwh) {
  delivered_mwh = 0.0;
  sr.mess_states.clear();
  if (strict_step.mess_states.empty()) return;

  for (const auto& strict_ms : strict_step.mess_states) {
    MESSStateStep ms = strict_ms;
    const int storage_index = ms.storage_index;
    const bool strict_travel_evidence = strict_ms.status == "InTransit" ||
                                        strict_ms.target_bus != strict_ms.bus ||
                                        strict_ms.remaining_travel_hr > kEps;
    if (projected_energy_mwh.find(storage_index) == projected_energy_mwh.end()) {
      projected_energy_mwh[storage_index] = strict_ms.energy_mwh;
      projected_bus[storage_index] = strict_ms.bus;
      if (strict_ms.soc > kEps) {
        projected_energy_capacity_mwh[storage_index] = strict_ms.energy_mwh / strict_ms.soc;
      }
    } else {
      const double strict_delta = last_strict_energy_mwh[storage_index] - strict_ms.energy_mwh;
      const double strict_dispatch_delta = std::max(0.0, last_strict_dispatch_mw[storage_index]) * dt_hr;
      const double applied_dispatch_delta = std::max(0.0, last_applied_dispatch_mw[storage_index]) * dt_hr;
      const double travel_delta = last_strict_travel_evidence[storage_index]
          ? std::max(0.0, strict_delta - strict_dispatch_delta)
          : 0.0;
      projected_energy_mwh[storage_index] = std::max(
          0.0,
          projected_energy_mwh[storage_index] - travel_delta - applied_dispatch_delta);
    }
    double usable_dispatch_mw = std::max(0.0, strict_ms.dispatch_mw);
    double applied_dispatch_mw = 0.0;
    if (dt_hr > kEps && usable_dispatch_mw > kEps && sr.shed_mw > kEps) {
      for (size_t i = 0; i < sr.bus_supply_kind.size() && i < sr.bus_supply_index.size(); ++i) {
        if (sr.bus_supply_kind[i] != "AC") continue;
        if (sr.bus_supply_index[i] != strict_ms.bus) continue;
        if (i >= sr.bus_supply_shed_mw.size() || i >= sr.bus_supply_served_mw.size()) continue;
        const double serve = std::min(usable_dispatch_mw, std::max(0.0, sr.bus_supply_shed_mw[i]));
        if (serve <= kEps) break;
        sr.bus_supply_shed_mw[i] -= serve;
        sr.bus_supply_served_mw[i] += serve;
        sr.shed_mw = std::max(0.0, sr.shed_mw - serve);
        sr.served_mw += serve;
        const double importance = i < sr.bus_supply_importance.size()
            ? std::max(1.0, sr.bus_supply_importance[i])
            : 1.0;
        sr.weighted_shed_mw = std::max(0.0, sr.weighted_shed_mw - serve * importance);
        const int tier = i < sr.bus_supply_priority_tier.size()
            ? std::clamp(sr.bus_supply_priority_tier[i], 0, 3)
            : 3;
        if (static_cast<size_t>(tier) < sr.shed_by_priority.size()) {
          sr.shed_by_priority[static_cast<size_t>(tier)] =
              std::max(0.0, sr.shed_by_priority[static_cast<size_t>(tier)] - serve);
        }
        usable_dispatch_mw -= serve;
        applied_dispatch_mw += serve;
        if (usable_dispatch_mw <= kEps) break;
      }
    }
    ms.dispatch_mw = applied_dispatch_mw;
    if (strict_travel_evidence) {
      ms.bus = strict_ms.bus;
      ms.target_bus = strict_ms.target_bus;
      ms.status = strict_ms.status;
      ms.remaining_travel_hr = strict_ms.remaining_travel_hr;
      ms.arrival_time_hr = strict_ms.arrival_time_hr;
    } else {
      if (last_strict_travel_evidence[storage_index]) {
        projected_bus[storage_index] = strict_ms.bus;
      }
      ms.bus = projected_bus[storage_index];
      ms.target_bus = ms.bus;
      ms.remaining_travel_hr = 0.0;
      ms.arrival_time_hr = strict_ms.arrival_time_hr;
      ms.status = applied_dispatch_mw > kEps ? "Deployed" : "Stationary";
    }
    if (strict_travel_evidence && strict_ms.status != "InTransit") {
      projected_bus[storage_index] = strict_ms.bus;
    }
    ms.energy_mwh = projected_energy_mwh[storage_index];
    const double cap = projected_energy_capacity_mwh[storage_index];
    ms.soc = cap > kEps ? std::clamp(ms.energy_mwh / cap, 0.0, 1.0) : 0.0;
    delivered_mwh += applied_dispatch_mw * dt_hr;
    last_strict_energy_mwh[storage_index] = strict_ms.energy_mwh;
    last_strict_dispatch_mw[storage_index] = strict_ms.dispatch_mw;
    last_applied_dispatch_mw[storage_index] = applied_dispatch_mw;
    last_strict_travel_evidence[storage_index] = strict_travel_evidence;
    sr.mess_states.push_back(ms);
  }
  sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;
}

void fill_step_from_solution(DistributionResilienceStepResult& sr,
                             const StageData& data,
                             const StageSolution& sol,
                             const StageSolution* prev,
                             const std::vector<StageFault>& faults,
                             DistributionDisasterStage stage,
                             double hour,
                             double dt) {
  sr.shed_by_priority.assign(4, 0.0);
  for (size_t i = 0; i < data.buses.size(); ++i) {
    const auto& b = data.buses[i];
    const double shed = i < sol.shed.size() ? sol.shed[i] : b.demand_mw;
    sr.total_demand_mw += b.demand_mw;
    sr.shed_mw += shed;
    sr.served_mw += std::max(0.0, b.demand_mw - shed);
    sr.weighted_shed_mw += shed * std::max(1.0, b.importance);
    const int tier = std::clamp(b.priority_tier, 0, 3);
    sr.shed_by_priority[static_cast<size_t>(tier)] += shed;
    sr.bus_supply_kind.push_back(b.kind == ResilienceBranchKind::DC ? "DC" : "AC");
    sr.bus_supply_index.push_back(b.index);
    sr.bus_supply_demand_mw.push_back(b.demand_mw);
    sr.bus_supply_served_mw.push_back(std::max(0.0, b.demand_mw - shed));
    sr.bus_supply_shed_mw.push_back(shed);
    sr.bus_supply_priority_tier.push_back(tier);
    sr.bus_supply_importance.push_back(b.importance);
  }
  for (size_t e = 0; e < data.edges.size(); ++e) {
    const auto& edge = data.edges[e];
    const bool closed = e < sol.beta.size() && sol.beta[e] != 0;
    if (!closed) {
      if (edge.kind == ResilienceBranchKind::DC) sr.open_dc_branch_ids.push_back(edge.index);
      else if (!edge.vsc) sr.open_ac_branch_ids.push_back(edge.index);
    } else if (edge.tie && !edge.vsc) {
      if (edge.kind == ResilienceBranchKind::DC) sr.closed_dc_tie_branch_ids.push_back(edge.index);
      else sr.closed_tie_branch_ids.push_back(edge.index);
    }
    if (prev && e < prev->beta.size() && edge.switchable) {
      const bool was = prev->beta[e] != 0;
      if (was != closed) {
        if (closed) {
          for (int id : edge.switch_ids) sr.switched_closed_ids.push_back(id);
        } else {
          for (int id : edge.switch_ids) sr.switched_open_ids.push_back(id);
        }
      }
    }
  }
  sr.open_switch_ids = sol.open_switch_ids;
  sr.closed_switch_ids = sol.closed_switch_ids;
  sort_unique(sr.switched_open_ids);
  sort_unique(sr.switched_closed_ids);
  sr.switch_open_actions = static_cast<int>(sr.switched_open_ids.size());
  sr.switch_close_actions = static_cast<int>(sr.switched_closed_ids.size());
  sr.switch_actions = sr.switch_open_actions + sr.switch_close_actions;
  sr.fault_zone_ac_bus_ids = sol.fault_zone_ac_bus_ids;
  sr.fault_zone_dc_bus_ids = sol.fault_zone_dc_bus_ids;
  if (stage == DistributionDisasterStage::DisasterIsolation) {
    sr.isolation_open_ac_branch_ids = sr.open_ac_branch_ids;
    sr.isolation_open_dc_branch_ids = sr.open_dc_branch_ids;
    sr.isolation_switch_actions = sr.switch_open_actions;
  } else if (stage == DistributionDisasterStage::DisasterPostFaultReconfig ||
             stage == DistributionDisasterStage::PostDisasterRepair) {
    for (size_t e = 0; e < data.edges.size(); ++e) {
      const auto& edge = data.edges[e];
      const bool closed = e < sol.beta.size() && sol.beta[e] != 0;
      const bool was = prev && e < prev->beta.size() && prev->beta[e] != 0;
      if (!closed || was || !edge.tie) continue;
      for (int id : edge.switch_ids) sr.closed_tie_switch_ids.push_back(id);
    }
    sort_unique(sr.closed_tie_switch_ids);
    sr.reconfiguration_switch_actions = static_cast<int>(sr.closed_tie_switch_ids.size());
  }
  sr.island_count = count_islands(data, sol.beta);
  for (const auto& f : faults) {
    if (hour >= f.start_hr - kEps && hour < f.start_hr + f.repair_hr - kEps) ++sr.active_faults;
    if (hour >= f.start_hr + f.repair_hr - kEps) ++sr.repaired_faults;
  }
  sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;
  (void)dt;
}

}  // namespace

DistributionResilienceResult run_distribution_resilience_stage_milp_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts) {
  DistributionResilienceResult result;
  result.model = DistributionResilienceModel::RAStyleStageMILP;
  result.model_stats.model = DistributionResilienceModel::RAStyleStageMILP;
  if (opts.horizon_hours <= 0 || opts.time_step_hr <= 0.0) {
    result.feasible = false;
    result.status = "Invalid horizon or timestep for RA-style stage MILP";
    return result;
  }
  if (sys.ac.buses.empty()) {
    result.feasible = false;
    result.status = "System has no AC network for RA-style stage MILP";
    return result;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(opts.horizon_hours / opts.time_step_hr)));
  const double legacy_window = opts.disaster_post_fault_reconfig_window_hr;
  const double effective_window_hr = std::abs(opts.post_fault_reconfig_window_hr - 2.0) > kEps
                                         ? opts.post_fault_reconfig_window_hr
                                         : legacy_window;
  result.feasible = true;
  result.model_stats.model_built = true;
  result.model_stats.model_solved = true;
  result.model_stats.solver_name = stage_solver_name(opts.mip.solver, opts.mip.num_threads);
  result.model_stats.model_scope = "ra-stage-topology-ac-dc-vsc";
  result.model_stats.validity.dc_network_modelled = true;
  result.model_stats.validity.vsc_dispatch_modelled = false;
  result.model_stats.validity.ac_branch_flow_limits_enforced = false;
  result.model_stats.validity.lindistflow_voltage_envelope_enforced = false;
  result.model_stats.validity.radial_topology_enforced = true;
  result.model_stats.validity.mip_gap_within_tolerance = true;
  result.model_stats.formulation_notes =
      "RA-Validation-style two-stage topology MILP: stage 1 fault isolation and stage 2 post-fault reconfiguration; AC/DC branches and VSC edges are included, DC branches are fixed unless faulted or protected by DC breakers.";

  StageSolution prev_sol;
  bool have_prev = false;
  StageSolution last_stage1;
  bool have_stage1 = false;
  StageSolution last_stage2;
  bool have_stage2 = false;
  bool used_virtual_branch_switches = false;
  std::set<int> repaired_fault_edges;
  std::vector<StageData> step_data;
  std::vector<StageSolution> step_solutions;
  step_data.reserve(static_cast<size_t>(steps));
  step_solutions.reserve(static_cast<size_t>(steps));
  std::vector<StageMessState> mess = collect_stage_mess(sys, build_stage_data(sys, opts, 0.0));

  for (int step = 0; step < steps; ++step) {
    const double hour = static_cast<double>(step) * opts.time_step_hr;
    StageData data = build_stage_data(sys, opts, hour);
    used_virtual_branch_switches = used_virtual_branch_switches || data.uses_virtual_branch_switches;
    const auto faults = build_stage_faults(sys, opts, data);
    if (step == 0) {
      for (const auto& f : faults) result.fault_sequence.push_back({f.kind, f.branch_index, f.start_hr, f.repair_hr, f.name});
    }
    const auto stage = stage_for_hour(faults, hour, effective_window_hr);
    auto effective_stage = stage;
    const bool disaster_stage = stage == DistributionDisasterStage::DisasterIsolation ||
                                stage == DistributionDisasterStage::DisasterPostFaultReconfig;
    std::vector<int> forced = disaster_stage ? occurred_fault_edges_no_repair(faults, hour)
                                             : occurred_fault_edges_no_repair(faults, hour);
    if (stage == DistributionDisasterStage::PostDisasterRepair) {
      forced.erase(std::remove_if(forced.begin(), forced.end(), [&](int edge_pos) {
                     return repaired_fault_edges.count(edge_pos) != 0;
                   }),
                   forced.end());
    }
    sort_unique(forced);

    StageSolution sol;
    if (stage == DistributionDisasterStage::Normal) {
      sol.feasible = true;
      sol.status = "Baseline";
      sol.beta.assign(data.edges.size(), 0);
      sol.z.assign(data.buses.size(), 0);
      sol.shed.assign(data.buses.size(), 0.0);
      sol.beta = initial_beta(data);
      populate_switch_state_from_beta(data, sol.beta, sol);
    } else if (stage == DistributionDisasterStage::DisasterIsolation) {
      sol = solve_stage_milp(data, opts, forced, nullptr, nullptr, false);
      last_stage1 = sol;
      have_stage1 = sol.feasible;
      have_stage2 = false;
    } else if (stage == DistributionDisasterStage::DisasterPostFaultReconfig) {
      if (!have_stage1) {
        last_stage1 = solve_stage_milp(data, opts, forced, nullptr, nullptr, false);
        have_stage1 = last_stage1.feasible;
      }
      sol = solve_stage_milp(data, opts, forced, &last_stage1.beta, nullptr, true);
      if (!sol.feasible && have_stage1) sol = last_stage1;
      last_stage2 = sol;
      have_stage2 = sol.feasible;
    } else {
      std::vector<int> repair_base_beta;
      const std::vector<int>* repair_prev_beta = nullptr;
      if (have_prev) {
        repair_prev_beta = &prev_sol.beta;
      } else if (have_stage2) {
        repair_prev_beta = &last_stage2.beta;
      } else if (have_stage1) {
        repair_prev_beta = &last_stage1.beta;
      } else {
        repair_base_beta = initial_beta(data);
        repair_prev_beta = &repair_base_beta;
      }

      if (forced.empty()) {
        sol = make_initial_topology_solution(data, "All damaged lines repaired; restored initial topology");
        effective_stage = DistributionDisasterStage::Normal;
      } else {
        bool have_candidate = false;
        int repaired_this_hour = -1;
        double best_shed = kInf;
        StageSolution best_sol;
        std::vector<int> best_forced = forced;
        for (int candidate_edge : forced) {
          std::vector<int> candidate_forced = forced;
          candidate_forced.erase(std::remove(candidate_forced.begin(), candidate_forced.end(), candidate_edge),
                                 candidate_forced.end());
          StageSolution candidate_sol = solve_stage_milp(data, opts, candidate_forced, repair_prev_beta, nullptr, true);
          if (!candidate_sol.feasible) continue;
          const double candidate_shed = solution_shed_mw(candidate_sol);
          if (!have_candidate || candidate_shed + kEps < best_shed) {
            have_candidate = true;
            repaired_this_hour = candidate_edge;
            best_shed = candidate_shed;
            best_sol = candidate_sol;
            best_forced = candidate_forced;
          }
        }
        if (have_candidate) {
          repaired_fault_edges.insert(repaired_this_hour);
          forced = std::move(best_forced);
          sol = std::move(best_sol);
          if (forced.empty()) {
            sol = make_initial_topology_solution(data, "All damaged lines repaired; restored initial topology");
            effective_stage = DistributionDisasterStage::Normal;
          }
          for (size_t i = 0; i < faults.size() && i < result.fault_sequence.size(); ++i) {
            if (faults[i].edge_pos != repaired_this_hour) continue;
            result.fault_sequence[i].repair_hr = std::max(0.0, hour - faults[i].start_hr);
            break;
          }
        } else if (have_prev) {
          sol = prev_sol;
        } else if (have_stage2) {
          sol = last_stage2;
        } else if (have_stage1) {
          sol = last_stage1;
        } else {
          sol = solve_stage_milp(data, opts, forced, repair_prev_beta, nullptr, true);
        }
        populate_switch_state_from_beta(data, sol.beta, sol);
      }
    }

    DistributionResilienceStepResult sr;
    sr.step_index = step;
    sr.hour = hour;
    const auto& load_profile = opts.load_profile.empty() ? kDefaultLoadProfile : opts.load_profile;
    const auto& renewable_profile = opts.renewable_profile.empty() ? kDefaultRenewableProfile : opts.renewable_profile;
    const auto& pv_profile = opts.pv_profile.empty() ? renewable_profile : opts.pv_profile;
    const auto& wind_profile = opts.wind_profile.empty() ? renewable_profile : opts.wind_profile;
    sr.load_multiplier = sample_profile(load_profile, hour);
    sr.res_multiplier = sample_profile(renewable_profile, hour);
    sr.pv_multiplier = sample_profile(pv_profile, hour);
    sr.wind_multiplier = sample_profile(wind_profile, hour);
    sr.disaster_stage = to_string(effective_stage);
    if (!sol.feasible) {
      result.feasible = false;
      result.status = "RA-style stage MILP failed at hour " + std::to_string(hour) + ": " + sol.status;
      result.model_stats.model_solved = false;
      result.model_stats.solver_status = sol.status;
      break;
    }
    if (sol.stats.num_variables > 0) {
      result.model_stats.num_variables = std::max(result.model_stats.num_variables, sol.stats.num_variables);
      result.model_stats.num_binary_variables = std::max(result.model_stats.num_binary_variables, sol.stats.num_binary_variables);
      result.model_stats.num_integer_variables = std::max(result.model_stats.num_integer_variables, sol.stats.num_integer_variables);
      result.model_stats.num_eq_constraints = std::max(result.model_stats.num_eq_constraints, sol.stats.num_eq_constraints);
      result.model_stats.num_ineq_constraints = std::max(result.model_stats.num_ineq_constraints, sol.stats.num_ineq_constraints);
      result.model_stats.objective_value += sol.stats.objective_value;
      result.model_stats.mip_gap = std::max(result.model_stats.mip_gap, sol.stats.mip_gap);
      result.model_stats.runtime_sec += sol.stats.runtime_sec;
      result.model_stats.solver_status = sol.stats.solver_status;
    }
    fill_step_from_solution(sr, data, sol, have_prev ? &prev_sol : nullptr, faults, effective_stage, hour, opts.time_step_hr);
    if (stage == DistributionDisasterStage::PostDisasterRepair) {
      sr.active_faults = static_cast<int>(forced.size());
      sr.repaired_faults = static_cast<int>(repaired_fault_edges.size());
    } else if (stage == DistributionDisasterStage::DisasterIsolation ||
               stage == DistributionDisasterStage::DisasterPostFaultReconfig) {
      sr.active_faults = static_cast<int>(forced.size());
      sr.repaired_faults = static_cast<int>(repaired_fault_edges.size());
    }
    prev_sol = sol;
    have_prev = true;

    step_data.push_back(std::move(data));
    step_solutions.push_back(std::move(sol));
    result.steps.push_back(std::move(sr));
  }

  if (result.feasible) {
    if (!opts.allow_mess_dispatch) {
      result.mess_dispatch_model = "disabled";
    } else if (mess.empty()) {
      result.mess_dispatch_model = "none";
    } else if (opts.use_strict_mip_for_mess) {
      const auto mess_sol = solve_ra_residual_mess_milp(sys, opts, result.steps, mess);
      if (mess_sol.stats.num_variables > 0) {
        result.model_stats.num_variables = std::max(result.model_stats.num_variables, mess_sol.stats.num_variables);
        result.model_stats.num_binary_variables = std::max(result.model_stats.num_binary_variables, mess_sol.stats.num_binary_variables);
        result.model_stats.num_integer_variables = std::max(result.model_stats.num_integer_variables, mess_sol.stats.num_integer_variables);
        result.model_stats.num_eq_constraints = std::max(result.model_stats.num_eq_constraints, mess_sol.stats.num_eq_constraints);
        result.model_stats.num_ineq_constraints = std::max(result.model_stats.num_ineq_constraints, mess_sol.stats.num_ineq_constraints);
        result.model_stats.objective_value += mess_sol.stats.objective_value;
        result.model_stats.mip_gap = std::max(result.model_stats.mip_gap, mess_sol.stats.mip_gap);
        result.model_stats.runtime_sec += mess_sol.stats.runtime_sec;
        result.model_stats.solver_status = mess_sol.stats.solver_status;
        result.model_stats.validity.mip_gap_within_tolerance =
            result.model_stats.validity.mip_gap_within_tolerance &&
            mess_sol.stats.mip_gap <= opts.mip.mip_gap + 1.0e-9;
      }
      if (mess_sol.feasible) {
        result.mess_dispatch_model = "ra_residual_mess_milp";
        result.mess_travel_distance_km = mess_sol.travel_distance_km;
        apply_mess_only_solution_to_steps(mess_sol, result.steps);
        result.model_stats.formulation_notes +=
            " MESS dispatch uses a dedicated RA-residual-shed time-space routing/SOC MILP solved with the user-selected solver; it no longer projects the AC-only strict restoration MIP trace.";
      } else if (opts.fallback_to_stage_mess_dispatch) {
        result.mess_dispatch_model = "stage_dispatch_fallback";
        apply_stage_mess_dispatch_to_steps(step_data, step_solutions, opts, mess, result.steps);
        result.model_stats.formulation_notes +=
            " RA-residual MESS MILP unavailable within selected solver/time/gap limits; falling back to stage-local MESS dispatch (" +
            mess_sol.status + ").";
      } else {
        result.mess_dispatch_model = "none";
        result.model_stats.formulation_notes +=
            " RA-residual MESS MILP unavailable and stage-local fallback disabled (" + mess_sol.status + ").";
      }
    } else if (opts.fallback_to_stage_mess_dispatch) {
      result.mess_dispatch_model = "stage_dispatch_fallback";
      apply_stage_mess_dispatch_to_steps(step_data, step_solutions, opts, mess, result.steps);
    } else {
      result.mess_dispatch_model = "none";
    }
  }

  if (result.feasible) accumulate_resilience_totals(result, opts);

  if (!result.steps.empty()) {
    result.resilience_index = result.total_demand_mwh > kEps ? result.total_served_mwh / result.total_demand_mwh : 1.0;
    double ratio_sum = 0.0;
    for (const auto& s : result.steps) ratio_sum += s.restoration_ratio;
    result.avg_restoration_ratio = ratio_sum / static_cast<double>(result.steps.size());
    result.final_restoration_ratio = result.steps.back().restoration_ratio;
  }
  if (used_virtual_branch_switches) {
    result.model_stats.formulation_notes +=
        " No explicit switching devices were mapped; virtual operable AC/DC branch breakers were used for all physical branches.";
  }
  result.total_ens_mwh = result.total_shed_mwh;
  result.max_curtailment_mw = result.peak_shed_mw;
  result.completed = result.feasible;
  if (!result.steps.empty()) result.restoration_time_hr = result.steps.back().hour;
  if (result.status.empty()) result.status = result.feasible ? "RA-style stage MILP completed" : "Failed";
  return result;
}

}  // namespace hacdcpf::analysis

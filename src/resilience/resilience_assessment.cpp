#include "hacdcpf/resilience/resilience_assessment.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/api/hacdcpf.hpp"

namespace hacdcpf::analysis {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;
constexpr double kPi  = 3.14159265358979323846;

// Demo-data default values.
constexpr double kDemoCustomersPerMw       = 120.0;
constexpr double kDemoLoadHighThreshold     = 0.8;
constexpr double kDemoLoadMedThreshold      = 0.3;
constexpr double kDemoBranchBaseLength      = 0.8;
constexpr double kDemoBranchLengthIncrement = 0.15;
constexpr double kDemoRepairHr              = 6.0;
constexpr double kDemoMessPmaxMw            = 0.75;
constexpr double kDemoMessEnergyMwh         = 2.5;
constexpr double kDemoMessSocInit           = 0.85;
constexpr double kDemoMessSocMin            = 0.15;
constexpr double kDemoMessSocMax            = 0.95;
constexpr double kDemoMessEta               = 0.95;
constexpr double kDemoMessConsumptionMwhKm  = 0.01;
constexpr double kDemoMessMaxTravelKm       = 120.0;

// Priority-based weighting for importance and estimated customer counts.
struct PriorityWeights { double importance; int customers_per_mw; };
PriorityWeights priority_weights(LoadPriority p) {
  switch (p) {
    case LoadPriority::Critical: return {4.0, 500};
    case LoadPriority::High:     return {2.5, 250};
    case LoadPriority::Medium:   return {1.5, 100};
    case LoadPriority::Low:      return {1.0,  40};
  }
  return {1.0, 100};
}

// MATLAB-derived 24-h residential load profile (normalised to peak = 1.0).
const std::vector<double> kDefaultLoadProfile = {
    0.64, 0.60, 0.58, 0.56, 0.56, 0.58, 0.64, 0.76,
    0.87, 0.95, 0.99, 1.00, 0.99, 1.00, 1.00, 0.97,
    0.96, 0.96, 0.93, 0.92, 0.92, 0.93, 0.87, 0.72};

// Simple daytime PV availability curve (normalised, sunrise ~ 6 h, sunset ~ 19 h).
const std::vector<double> kDefaultRenewableProfile = {
    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.05, 0.20,
    0.45, 0.70, 0.85, 0.95, 1.00, 0.95, 0.85, 0.70,
    0.45, 0.20, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00};

/// Sample from a cyclic profile with linear interpolation.
/// Negative profile values are clamped to 0.
double sample_profile(const std::vector<double>& profile, double hour) {
  if (profile.empty()) return 1.0;
  const double period = static_cast<double>(profile.size());
  double h = std::fmod(hour, period);
  if (h < 0.0) h += period;
  const size_t lo = static_cast<size_t>(h) % profile.size();
  const size_t hi = (lo + 1) % profile.size();
  const double frac = h - std::floor(h);
  const double val = profile[lo] * (1.0 - frac) + profile[hi] * frac;
  return std::max(0.0, val);
}

struct BusLoadEntry {
  int bus_pos{0};
  double demand_mw{0.0};
  double importance{1.0};
  int customers{0};
};

struct FixedStorageState {
  int bus_pos{0};
  int storage_index{0};
  double pmax_mw{0.0};
  double e_min_mwh{0.0};
  double energy_mwh{0.0};
  double eta_discharge{1.0};
};

struct MobileStorageState {
  int storage_index{0};
  int bus_pos{0};
  int target_bus_pos{0};
  MobileStorageStatus status{MobileStorageStatus::Stationary};
  double pmax_mw{0.0};
  double e_min_mwh{0.0};
  double e_rated_mwh{0.0};
  double energy_mwh{0.0};
  double e_consumption_mwh_km{0.0};
  double max_travel_distance_km{kInf};
  double eta_discharge{1.0};
  double arrival_time_hr{0.0};
  double dispatch_mw{0.0};
  double cumulative_travel_km{0.0};
  std::vector<int> route_bus_positions;
  std::vector<double> route_leg_distance_km;
  size_t route_leg_index{0};
  double leg_remaining_hr{0.0};
  double remaining_travel_hr{0.0};
};

struct ComponentEvaluation {
  double total_demand_mw{0.0};
  double served_mw{0.0};
  double shed_mw{0.0};
  double weighted_shed_mw{0.0};
  double res_mw{0.0};
  /// Shed MW by priority tier: [Critical, High, Medium, Low].
  std::vector<double> shed_by_priority{0.0, 0.0, 0.0, 0.0};
  int representative_bus_pos{-1};
  std::vector<double> fixed_storage_dispatch;
  std::vector<double> mess_dispatch;
};

struct FaultRuntime {
  int branch_pos{-1};
  int branch_index{0};
  double start_hr{0.0};
  double repair_hr{0.0};
  std::string name;
};

struct IslandInfo {
  std::vector<int> bus_positions;
  bool has_grid_source{false};
  double total_demand_mw{0.0};
};

using Adj = std::vector<std::vector<std::pair<int, double>>>;

template <class T>
T nonnegative_scale(const T& value) {
  return std::max(T(0), value);
}

std::unordered_map<int, int> make_bus_pos_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  out.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    assert(out.find(sys.ac.buses[i].index) == out.end() &&
           "Duplicate bus index in AC network");
    out[sys.ac.buses[i].index] = static_cast<int>(i);
  }
  return out;
}

std::unordered_map<int, int> make_branch_pos_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  out.reserve(sys.ac.branches.size());
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const int idx = sys.ac.branches[i].index != 0 ? sys.ac.branches[i].index
                                                  : static_cast<int>(i + 1);
    out[idx] = static_cast<int>(i);
  }
  return out;
}

std::vector<BusLoadEntry> collect_bus_loads(const HybridPowerSystem& sys,
                                            double load_scale_factor,
                                            const std::unordered_map<int, int>& bus_pos) {
  std::vector<BusLoadEntry> loads;

  // P1b: DC-OPF formulation adds bus.pd_mw and ac.loads additively as demand;
  // resilience demand collection must mirror the same convention.  Both sources
  // are always iterated unconditionally so neither is silently omitted when the
  // other is present.  Callers aggregate demand_by_bus by bus_pos, so separate
  // entries for the same bus are correctly summed downstream.
  loads.reserve(sys.ac.buses.size() + sys.ac.loads.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& bus = sys.ac.buses[i];
    const double demand_mw = std::max(0.0, bus.pd_mw * load_scale_factor);
    if (!bus.in_service || demand_mw <= 0.0) continue;
    BusLoadEntry e;
    e.bus_pos = static_cast<int>(i);
    e.demand_mw = demand_mw;
    e.importance = std::max(1.0, bus.importance);
    e.customers = bus.n_customers > 0 ? bus.n_customers
                                      : std::max(1, static_cast<int>(std::lround(demand_mw * 100.0)));
    loads.push_back(e);
  }
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    const auto it = bus_pos.find(ld.bus);
    if (it == bus_pos.end()) continue;
    const auto& bus = sys.ac.buses[static_cast<size_t>(it->second)];
    BusLoadEntry e;
    e.bus_pos = it->second;
    e.demand_mw = std::max(0.0, ld.p_mw * nonnegative_scale(ld.scaling) * load_scale_factor);
    e.importance = std::max(bus.importance, priority_weights(ld.priority).importance);
    e.customers = ld.n_customers > 0 ? ld.n_customers
                                     : std::max(1, static_cast<int>(std::lround(e.demand_mw * priority_weights(ld.priority).customers_per_mw)));
    if (e.demand_mw > 0.0) loads.push_back(e);
  }
  return loads;
}

std::vector<bool> identify_grid_source_buses(const HybridPowerSystem& sys,
                                              const std::unordered_map<int, int>& bus_pos) {
  std::vector<bool> source(sys.ac.buses.size(), false);

  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (sys.ac.buses[i].in_service && sys.ac.buses[i].bus_type == BusType::SLACK)
      source[i] = true;
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = bus_pos.find(eg.bus);
    if (it != bus_pos.end()) source[static_cast<size_t>(it->second)] = true;
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service || !g.is_slack) continue;
    const auto it = bus_pos.find(g.bus);
    if (it != bus_pos.end()) source[static_cast<size_t>(it->second)] = true;
  }
  return source;
}

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
      if (!e.available) continue;
      add_edge(e.from_bus, e.to_bus, e.distance_km);
    }
    return graph;
  }

  if (opts.use_electrical_graph_as_transport_proxy) {
    for (const auto& br : sys.ac.branches) {
      add_edge(br.from_bus, br.to_bus, br.length_km);
    }
    // Transformers are co-located equipment (HV and LV terminals at the
    // same substation), so add them as zero-length transport edges to
    // keep the routing graph consistent with the electrical topology.
    for (const auto& t : sys.ac.transformers_2w) {
      if (!t.in_service) continue;
      add_edge(t.hv_bus, t.lv_bus, 0.0);
    }
  }
  return graph;
}

std::pair<double, std::vector<int>> shortest_path_route_km(const Adj& graph, int start, int goal) {
  if (start < 0 || goal < 0 || start >= static_cast<int>(graph.size()) ||
      goal >= static_cast<int>(graph.size())) {
    return {kInf, {}};
  }
  if (start == goal) return {0.0, {start}};

  std::vector<double> dist(graph.size(), kInf);
  std::vector<int> parent(graph.size(), -1);
  using Node = std::pair<double, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
  dist[static_cast<size_t>(start)] = 0.0;
  pq.push({0.0, start});

  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[static_cast<size_t>(u)] + kEps) continue;
    if (u == goal) break;
    for (const auto& [v, w] : graph[static_cast<size_t>(u)]) {
      const double nd = d + w;
      if (nd + kEps < dist[static_cast<size_t>(v)]) {
        dist[static_cast<size_t>(v)] = nd;
        parent[static_cast<size_t>(v)] = u;
        pq.push({nd, v});
      }
    }
  }
  if (!std::isfinite(dist[static_cast<size_t>(goal)])) return {kInf, {}};

  std::vector<int> rev_path;
  for (int v = goal; v >= 0; v = parent[static_cast<size_t>(v)]) {
    rev_path.push_back(v);
    if (v == start) break;
  }
  if (rev_path.empty() || rev_path.back() != start) return {kInf, {}};
  std::reverse(rev_path.begin(), rev_path.end());
  return {dist[static_cast<size_t>(goal)], rev_path};
}

double edge_distance_km(const Adj& graph, int from, int to) {
  if (from < 0 || to < 0 || from >= static_cast<int>(graph.size())) return kInf;
  for (const auto& [v, w] : graph[static_cast<size_t>(from)]) {
    if (v == to) return w;
  }
  return kInf;
}

std::vector<int> compute_components(const HybridPowerSystem& sys,
                                    const std::vector<bool>& closed,
                                    const std::unordered_map<int, int>& bus_pos) {
  std::vector<int> comp(sys.ac.buses.size(), -1);
  std::vector<std::vector<int>> adj(sys.ac.buses.size());
  for (size_t bi = 0; bi < sys.ac.branches.size(); ++bi) {
    if (!closed[bi]) continue;
    const auto& br = sys.ac.branches[bi];
    const auto it_f = bus_pos.find(br.from_bus);
    const auto it_t = bus_pos.find(br.to_bus);
    if (it_f == bus_pos.end() || it_t == bus_pos.end()) continue;
    adj[static_cast<size_t>(it_f->second)].push_back(it_t->second);
    adj[static_cast<size_t>(it_t->second)].push_back(it_f->second);
  }
  // Two-winding transformers also tie HV and LV buses together.
  for (const auto& t : sys.ac.transformers_2w) {
    if (!t.in_service) continue;
    const auto it_f = bus_pos.find(t.hv_bus);
    const auto it_t = bus_pos.find(t.lv_bus);
    if (it_f == bus_pos.end() || it_t == bus_pos.end()) continue;
    adj[static_cast<size_t>(it_f->second)].push_back(it_t->second);
    adj[static_cast<size_t>(it_t->second)].push_back(it_f->second);
  }

  int cid = 0;
  std::vector<int> stack;
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (comp[i] >= 0) continue;
    stack.clear();
    stack.push_back(static_cast<int>(i));
    comp[i] = cid;
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

std::vector<IslandInfo> build_islands(const std::vector<int>& comp,
                                      const std::vector<bool>& source_bus,
                                      const std::vector<double>& demand_by_bus) {
  int ncomp = 0;
  for (int c : comp) ncomp = std::max(ncomp, c + 1);
  std::vector<IslandInfo> islands(static_cast<size_t>(ncomp));
  for (size_t b = 0; b < comp.size(); ++b) {
    const int cid = comp[b];
    if (cid < 0) continue;
    auto& island = islands[static_cast<size_t>(cid)];
    island.bus_positions.push_back(static_cast<int>(b));
    island.has_grid_source = island.has_grid_source || source_bus[b];
    island.total_demand_mw += demand_by_bus[b];
  }
  return islands;
}

double generator_capacity_mw(const Generator& g) {
  return std::max({0.0, g.pmax_mw, g.pg_mw});
}

double static_generator_capacity_mw(const StaticGenerator& g) {
  const double p = g.pmax_mw > 0.0 ? g.pmax_mw : g.p_mw;
  return std::max(0.0, p * nonnegative_scale(g.scaling));
}

double renewable_capacity_mw(const RenewableGen& g) {
  const double p = g.p_rated_mw > 0.0 ? std::min(g.p_rated_mw, std::max(0.0, g.p_mw))
                                      : std::max(0.0, g.p_mw);
  return std::max(0.0, p);
}

double pv_capacity_mw(const PVSystem& pv) {
  if (pv.pmax_mw > 0.0) return pv.pmax_mw;
  return std::max(0.0, pv.p_mw);
}

std::vector<FixedStorageState> collect_fixed_storage(const HybridPowerSystem& sys,
                                                     const std::unordered_map<int, int>& bus_pos) {
  std::vector<FixedStorageState> out;
  out.reserve(sys.ac.storage.size());
  for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!st.in_service) continue;
    const auto it = bus_pos.find(st.bus);
    if (it == bus_pos.end()) continue;
    FixedStorageState s;
    s.bus_pos = it->second;
    s.storage_index = st.index != 0 ? st.index : static_cast<int>(i + 1);
    s.pmax_mw = std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw});
    s.e_min_mwh = st.soc_min * st.e_rated_mwh;
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : st.soc_init * st.e_rated_mwh;
    s.energy_mwh = std::max(s.e_min_mwh, e0);
    s.eta_discharge = st.eta_discharge > 0.0 ? st.eta_discharge : 1.0;
    out.push_back(s);
  }
  return out;
}

std::vector<MobileStorageState> collect_mobile_storage(const HybridPowerSystem& sys,
                                                       const std::unordered_map<int, int>& bus_pos) {
  std::vector<MobileStorageState> out;
  out.reserve(sys.mobile_storage.size());
  for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
    const auto& st = sys.mobile_storage[i];
    if (!st.in_service) continue;
    const auto it = bus_pos.find(st.bus);
    if (it == bus_pos.end()) continue;
    MobileStorageState s;
    s.storage_index = st.index != 0 ? st.index : static_cast<int>(i + 1);
    s.bus_pos = it->second;
    s.target_bus_pos = s.bus_pos;
    s.status = st.status;
    s.pmax_mw = std::max({0.0, st.pmax_mw, st.p_rated_mw, st.p_mw});
    s.e_rated_mwh = std::max(0.0, st.e_rated_mwh);
    const double soc_lo = std::min(st.soc_min, st.soc_max);
    s.e_min_mwh = soc_lo * s.e_rated_mwh;
    const double soc0 = std::max(soc_lo, st.soc_init);
    const double e0 = st.e_mwh > 0.0 ? st.e_mwh : soc0 * s.e_rated_mwh;
    s.energy_mwh = std::max(s.e_min_mwh, e0);
    s.e_consumption_mwh_km = std::max(0.0, st.e_consumption_mwh_km);
    s.max_travel_distance_km = st.max_travel_distance_km > 0.0 ? st.max_travel_distance_km : kInf;
    s.eta_discharge = st.eta_discharge > 0.0 ? st.eta_discharge : 1.0;
    s.arrival_time_hr = st.arrival_time;
    out.push_back(s);
  }
  return out;
}

std::vector<FaultRuntime> build_faults(const HybridPowerSystem& sys,
                                       const DistributionResilienceOptions& opts,
                                       const std::unordered_map<int, int>& branch_pos) {
  std::vector<FaultRuntime> out;
  if (!opts.faults.empty()) {
    out.reserve(opts.faults.size());
    for (const auto& f : opts.faults) {
      auto it = branch_pos.find(f.ac_branch_index);
      if (it == branch_pos.end()) continue;
      const auto& br = sys.ac.branches[static_cast<size_t>(it->second)];
      out.push_back({it->second,
                     br.index != 0 ? br.index : f.ac_branch_index,
                     f.outage_start_hr,
                     std::max(opts.time_step_hr, f.repair_duration_hr),
                     f.name.empty() ? br.name : f.name});
    }
    // Merge overlapping faults on the same branch.
    std::sort(out.begin(), out.end(), [](const FaultRuntime& a, const FaultRuntime& b) {
      return a.branch_pos != b.branch_pos ? a.branch_pos < b.branch_pos
                                          : a.start_hr < b.start_hr;
    });
    std::vector<FaultRuntime> merged;
    merged.reserve(out.size());
    for (const auto& f : out) {
      if (!merged.empty() && merged.back().branch_pos == f.branch_pos &&
          f.start_hr < merged.back().start_hr + merged.back().repair_hr - kEps) {
        auto& m = merged.back();
        double end = std::max(m.start_hr + m.repair_hr, f.start_hr + f.repair_hr);
        m.repair_hr = end - m.start_hr;
      } else {
        merged.push_back(f);
      }
    }
    return merged;
  }

  std::vector<int> candidates;
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    if (sys.ac.branches[i].in_service) candidates.push_back(static_cast<int>(i));
  }
  std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
    const auto& ba = sys.ac.branches[static_cast<size_t>(a)];
    const auto& bb = sys.ac.branches[static_cast<size_t>(b)];
    const double wa = ba.length_km > 0.0 ? ba.length_km : static_cast<double>(a + 1);
    const double wb = bb.length_km > 0.0 ? bb.length_km : static_cast<double>(b + 1);
    return wa > wb;
  });
  const int count = std::min(opts.default_fault_count, static_cast<int>(candidates.size()));
  for (int i = 0; i < count; ++i) {
    const int pos = candidates[static_cast<size_t>(i)];
    const auto& br = sys.ac.branches[static_cast<size_t>(pos)];
    const double start = opts.auto_fault_start_hr + i * std::max(0.0, opts.auto_fault_stagger_hr);
    out.push_back({pos,
                   br.index != 0 ? br.index : static_cast<int>(pos + 1),
                   start,
                   std::max(opts.time_step_hr, opts.default_repair_time_hr),
                   br.name});
  }
  return out;
}

std::vector<bool> active_branch_status(const HybridPowerSystem& sys,
                                       const std::vector<bool>& previous_closed,
                                       const std::vector<FaultRuntime>& faults,
                                       double hour,
                                       int& active_faults,
                                       int& repaired_faults,
                                       std::vector<int>& open_branch_ids,
                                       std::vector<bool>& fault_active) {
  std::vector<bool> closed = previous_closed;
  fault_active.assign(sys.ac.branches.size(), false);
  active_faults = 0;
  repaired_faults = 0;
  open_branch_ids.clear();

  for (const auto& f : faults) {
    if (f.branch_pos < 0 || f.branch_pos >= static_cast<int>(fault_active.size())) continue;
    if (hour >= f.start_hr && hour < f.start_hr + f.repair_hr - kEps) {
      fault_active[static_cast<size_t>(f.branch_pos)] = true;
      ++active_faults;
    } else if (hour >= f.start_hr + f.repair_hr - kEps) {
      ++repaired_faults;
    }
  }

  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    if (fault_active[i]) closed[i] = false;
    if (!closed[i]) {
      const int idx = sys.ac.branches[i].index != 0 ? sys.ac.branches[i].index
                                                    : static_cast<int>(i + 1);
      open_branch_ids.push_back(idx);
    }
  }
  return closed;
}

void greedily_reconfigure(const HybridPowerSystem& sys,
                          const std::vector<double>& demand_by_bus,
                          const std::vector<bool>& source_bus,
                          const std::vector<bool>& fault_active,
                          const std::unordered_map<int, int>& bus_pos,
                          std::vector<bool>& closed,
                          std::vector<int>& closed_tie_branch_ids,
                          int max_iterations = 50) {
  closed_tie_branch_ids.clear();
  for (int iter = 0; iter < max_iterations; ++iter) {
    const auto comp = compute_components(sys, closed, bus_pos);
    const auto islands = build_islands(comp, source_bus, demand_by_bus);

    int best_branch = -1;
    double best_score = 0.0;
    for (size_t bi = 0; bi < sys.ac.branches.size(); ++bi) {
      if (closed[bi]) continue;
      if (bi < fault_active.size() && fault_active[bi]) continue;
      const auto& br = sys.ac.branches[bi];
      const auto it_f = bus_pos.find(br.from_bus);
      const auto it_t = bus_pos.find(br.to_bus);
      if (it_f == bus_pos.end() || it_t == bus_pos.end()) continue;
      const int u = it_f->second;
      const int v = it_t->second;
      const int cu = comp[static_cast<size_t>(u)];
      const int cv = comp[static_cast<size_t>(v)];
      if (cu == cv || cu < 0 || cv < 0) continue;
      const auto& iu = islands[static_cast<size_t>(cu)];
      const auto& iv = islands[static_cast<size_t>(cv)];
      double score = 0.0;
      if (iu.has_grid_source != iv.has_grid_source) {
        score = iu.has_grid_source ? iv.total_demand_mw : iu.total_demand_mw;
      } else if (!iu.has_grid_source && !iv.has_grid_source) {
        score = 0.1 * std::min(iu.total_demand_mw, iv.total_demand_mw);
      }
      if (score > best_score + kEps) {
        best_score = score;
        best_branch = static_cast<int>(bi);
      }
    }

    if (best_branch < 0 || best_score <= kEps) break;
    closed[static_cast<size_t>(best_branch)] = true;
    const int idx = sys.ac.branches[static_cast<size_t>(best_branch)].index != 0
                        ? sys.ac.branches[static_cast<size_t>(best_branch)].index
                        : best_branch + 1;
    closed_tie_branch_ids.push_back(idx);
  }
}

/// Bundles per-step state passed to island evaluation.
struct StepContext {
  const HybridPowerSystem& sys;
  const std::vector<BusLoadEntry>& loads;
  const std::vector<int>& comp;
  const std::unordered_map<int, int>& bus_pos;
  const std::vector<FixedStorageState>& fixed_storage;
  const std::vector<MobileStorageState>& mess;
  bool enable_mess_dispatch;
  double dt_hr;
  double res_multiplier;
};

ComponentEvaluation evaluate_island(const StepContext& ctx,
                                    const IslandInfo& island) {
  ComponentEvaluation out;
  out.fixed_storage_dispatch.assign(ctx.fixed_storage.size(), 0.0);
  out.mess_dispatch.assign(ctx.mess.size(), 0.0);
  if (island.bus_positions.empty()) return out;

  const auto& comp = ctx.comp;
  const auto& bus_pos = ctx.bus_pos;
  const int island_comp = comp[static_cast<size_t>(island.bus_positions.front())];
  out.representative_bus_pos = island.bus_positions.front();

  std::vector<BusLoadEntry> island_loads;
  for (const auto& ld : ctx.loads) {
    if (static_cast<size_t>(ld.bus_pos) >= comp.size()) continue;
    if (comp[static_cast<size_t>(ld.bus_pos)] != island_comp) continue;
    island_loads.push_back(ld);
    out.total_demand_mw += ld.demand_mw;
    if (ld.importance > ctx.loads.front().importance + kEps ||
        out.representative_bus_pos == island.bus_positions.front()) {
      out.representative_bus_pos = ld.bus_pos;
    }
  }

  if (island.has_grid_source) {
    out.served_mw = out.total_demand_mw;
    out.shed_by_priority.assign(4, 0.0);
    return out;
  }

  // Helper: check if a bus belongs to this island's component.
  auto in_island = [&](int bus_id) -> bool {
    auto it = bus_pos.find(bus_id);
    if (it == bus_pos.end()) return false;
    if (static_cast<size_t>(it->second) >= comp.size()) return false;
    return comp[static_cast<size_t>(it->second)] == island_comp;
  };

  double available_supply = 0.0;
  for (const auto& g : ctx.sys.ac.generators) {
    if (!g.in_service || !in_island(g.bus)) continue;
    available_supply += generator_capacity_mw(g);
  }
  for (const auto& sg : ctx.sys.ac.static_generators) {
    if (!sg.in_service || !in_island(sg.bus)) continue;
    available_supply += static_generator_capacity_mw(sg);
  }
  for (const auto& rg : ctx.sys.ac.renewable_gens) {
    if (!rg.in_service || !in_island(rg.bus)) continue;
    const double cap = renewable_capacity_mw(rg) * ctx.res_multiplier;
    available_supply += cap;
    out.res_mw += cap;
  }
  for (const auto& pv : ctx.sys.ac.pv_systems) {
    if (!pv.in_service || !in_island(pv.bus)) continue;
    const double cap = pv_capacity_mw(pv) * ctx.res_multiplier;
    available_supply += cap;
    out.res_mw += cap;
  }

  double remaining = std::max(0.0, out.total_demand_mw - available_supply);
  for (size_t i = 0; i < ctx.fixed_storage.size() && remaining > kEps; ++i) {
    const auto& st = ctx.fixed_storage[i];
    if (static_cast<size_t>(st.bus_pos) >= comp.size() || comp[static_cast<size_t>(st.bus_pos)] != island_comp) continue;
    const double deliverable_mw = std::min(st.pmax_mw,
                                           std::max(0.0, (st.energy_mwh - st.e_min_mwh) * st.eta_discharge / ctx.dt_hr));
    const double dispatch = std::min(remaining, deliverable_mw);
    out.fixed_storage_dispatch[i] = dispatch;
    available_supply += dispatch;
    remaining -= dispatch;
  }
  if (ctx.enable_mess_dispatch) {
    for (size_t i = 0; i < ctx.mess.size() && remaining > kEps; ++i) {
      const auto& st = ctx.mess[i];
      if (st.status == MobileStorageStatus::InTransit) continue;
      if (static_cast<size_t>(st.bus_pos) >= comp.size() || comp[static_cast<size_t>(st.bus_pos)] != island_comp) continue;
      const double deliverable_mw = std::min(st.pmax_mw,
                                             std::max(0.0, (st.energy_mwh - st.e_min_mwh) * st.eta_discharge / ctx.dt_hr));
      const double dispatch = std::min(remaining, deliverable_mw);
      out.mess_dispatch[i] = dispatch;
      available_supply += dispatch;
      remaining -= dispatch;
    }
  }

  out.served_mw = std::min(out.total_demand_mw, available_supply);
  out.shed_mw = std::max(0.0, out.total_demand_mw - out.served_mw);

  std::sort(island_loads.begin(), island_loads.end(), [](const BusLoadEntry& a, const BusLoadEntry& b) {
    if (std::abs(a.importance - b.importance) > kEps) return a.importance > b.importance;
    return a.demand_mw > b.demand_mw;
  });
  double served_remaining = out.served_mw;
  out.shed_by_priority.assign(4, 0.0);
  for (const auto& ld : island_loads) {
    const double served = std::min(ld.demand_mw, served_remaining);
    const double shed = ld.demand_mw - served;
    served_remaining -= served;
    out.weighted_shed_mw += shed * ld.importance;
    // Bin shed into priority tiers by importance value.
    int tier = 3;  // Low by default
    if (ld.importance >= 4.0 - kEps)      tier = 0;  // Critical
    else if (ld.importance >= 2.5 - kEps)  tier = 1;  // High
    else if (ld.importance >= 1.5 - kEps)  tier = 2;  // Medium
    out.shed_by_priority[static_cast<size_t>(tier)] += shed;
  }
  return out;
}

void apply_dispatch_to_storage(std::vector<FixedStorageState>& fixed_storage,
                               std::vector<MobileStorageState>& mess,
                               const ComponentEvaluation& eval,
                               double dt_hr,
                               double& mess_energy_delivered_mwh) {
  for (size_t i = 0; i < fixed_storage.size(); ++i) {
    if (eval.fixed_storage_dispatch[i] <= 0.0) continue;
    auto& st = fixed_storage[i];
    st.energy_mwh = std::max(st.e_min_mwh,
                             st.energy_mwh - eval.fixed_storage_dispatch[i] * dt_hr / st.eta_discharge);
  }
  for (size_t i = 0; i < mess.size(); ++i) {
    mess[i].dispatch_mw = eval.mess_dispatch[i];
    if (eval.mess_dispatch[i] <= 0.0) continue;
    auto& st = mess[i];
    st.energy_mwh = std::max(st.e_min_mwh,
                             st.energy_mwh - eval.mess_dispatch[i] * dt_hr / st.eta_discharge);
    mess_energy_delivered_mwh += eval.mess_dispatch[i] * dt_hr;
  }
}

/// Reset MESS routing state and mark as deployed at its target.
void mark_mess_deployed(MobileStorageState& st, double arrival_hr) {
  st.status = MobileStorageStatus::Deployed;
  st.bus_pos = st.target_bus_pos;
  st.remaining_travel_hr = 0.0;
  st.arrival_time_hr = arrival_hr;
  st.route_bus_positions.clear();
  st.route_leg_distance_km.clear();
  st.route_leg_index = 0;
  st.leg_remaining_hr = 0.0;
}

/// Complete one leg of the discrete route and account for travel energy.
void complete_mess_leg(MobileStorageState& st, double& total_travel_distance_km) {
  const double leg_dist_km = st.route_leg_distance_km[st.route_leg_index];
  st.energy_mwh = std::max(st.e_min_mwh,
                           st.energy_mwh - leg_dist_km * st.e_consumption_mwh_km);
  total_travel_distance_km += leg_dist_km;
  st.cumulative_travel_km += leg_dist_km;
  ++st.route_leg_index;
  if (st.route_leg_index < st.route_bus_positions.size() &&
      st.route_leg_index < static_cast<size_t>(std::numeric_limits<int>::max())) {
    st.bus_pos = st.route_bus_positions[st.route_leg_index];
  }
}

void advance_mess_transit(double hour,
                          double dt_hr,
                          const DistributionResilienceOptions& opts,
                          std::vector<MobileStorageState>& mess,
                          double& total_travel_distance_km) {
  const double speed = std::max(opts.mess_travel_speed_kmph, 1.0);
  for (auto& st : mess) {
    st.dispatch_mw = 0.0;
    if (st.status != MobileStorageStatus::InTransit) {
      st.leg_remaining_hr = 0.0;
      st.remaining_travel_hr = 0.0;
      continue;
    }

    if (st.route_leg_distance_km.empty()) {
      const double rem = std::max(0.0, st.arrival_time_hr - hour);
      if (rem <= dt_hr + kEps) {
        mark_mess_deployed(st, hour);
      } else {
        st.remaining_travel_hr = rem - dt_hr;
        st.arrival_time_hr = hour + st.remaining_travel_hr;
      }
      continue;
    }

    double budget_hr = dt_hr;
    while (budget_hr > kEps && st.status == MobileStorageStatus::InTransit) {
      if (st.route_leg_index >= st.route_leg_distance_km.size()) {
        mark_mess_deployed(st, hour);
        break;
      }

      if (st.leg_remaining_hr <= kEps) {
        if (st.route_leg_index >= st.route_leg_distance_km.size()) break;
        st.leg_remaining_hr = st.route_leg_distance_km[st.route_leg_index] / speed;
      }
      const double step_hr = std::min(budget_hr, st.leg_remaining_hr);
      st.leg_remaining_hr -= step_hr;
      st.remaining_travel_hr = std::max(0.0, st.remaining_travel_hr - step_hr);
      budget_hr -= step_hr;

      if (st.leg_remaining_hr <= kEps) {
        complete_mess_leg(st, total_travel_distance_km);
        if (st.route_leg_index >= st.route_leg_distance_km.size()) {
          st.target_bus_pos = st.bus_pos;
          mark_mess_deployed(st, hour + (dt_hr - budget_hr));
        }
      }
    }

    if (st.status == MobileStorageStatus::InTransit) {
      st.arrival_time_hr = hour + st.remaining_travel_hr;
    }
  }
}

void assign_mess_movements(double hour,
                           double dt_hr,
                           const Adj& transport_graph,
                           const std::vector<int>& comp,
                           const std::vector<IslandInfo>& islands,
                           const std::vector<ComponentEvaluation>& evals,
                           const DistributionResilienceOptions& opts,
                           std::vector<MobileStorageState>& mess) {
  struct Target {
    int bus_pos{-1};
    double deficit{0.0};
  };

  std::vector<Target> targets;
  for (size_t i = 0; i < islands.size(); ++i) {
    if (islands[i].has_grid_source) continue;
    if (evals[i].shed_mw <= kEps) continue;
    targets.push_back({evals[i].representative_bus_pos, evals[i].weighted_shed_mw});
  }
  std::sort(targets.begin(), targets.end(), [](const Target& a, const Target& b) {
    return a.deficit > b.deficit;
  });
  if (targets.empty()) return;

  const double speed = std::max(opts.mess_travel_speed_kmph, 1.0);

  // Dijkstra route cache: avoid recomputing the same (start, goal) pair.
  std::unordered_map<int64_t, std::pair<double, std::vector<int>>> route_cache;
  auto cached_shortest_path = [&](int start, int goal) -> const std::pair<double, std::vector<int>>& {
    const int64_t key = (static_cast<int64_t>(static_cast<uint32_t>(start)) << 32) |
                        static_cast<int64_t>(static_cast<uint32_t>(goal));
    auto it = route_cache.find(key);
    if (it != route_cache.end()) return it->second;
    return route_cache.emplace(key, shortest_path_route_km(transport_graph, start, goal)).first->second;
  };

  for (auto& st : mess) {
    if (st.status == MobileStorageStatus::InTransit) continue;
    if (st.dispatch_mw > kEps) continue;  // already serving load
    const int current_comp = comp[static_cast<size_t>(st.bus_pos)];
    const bool local_deficit = current_comp >= 0 && current_comp < static_cast<int>(islands.size())
                               && !islands[static_cast<size_t>(current_comp)].has_grid_source
                               && evals[static_cast<size_t>(current_comp)].shed_mw > kEps;
    if (local_deficit) continue;
    if (st.energy_mwh <= st.e_min_mwh + kEps) continue;

    int best_target_bus = -1;
    double best_distance_km = kInf;
    std::vector<int> best_route;
    double best_score = -kInf;
    for (const auto& target : targets) {
      if (target.bus_pos == st.bus_pos) continue;
      const auto& [distance_km, route] = cached_shortest_path(st.bus_pos, target.bus_pos);
      if (!std::isfinite(distance_km) || route.size() < 2) continue;
      if (st.cumulative_travel_km + distance_km > st.max_travel_distance_km + kEps) continue;
      const double travel_energy = distance_km * st.e_consumption_mwh_km;
      if (st.energy_mwh - travel_energy <= st.e_min_mwh + kEps) continue;
      const double travel_hr = distance_km / speed;
      const double score = target.deficit / (1.0 + travel_hr / std::max(dt_hr, kEps));
      if (score > best_score + kEps) {
        best_score = score;
        best_target_bus = target.bus_pos;
        best_distance_km = distance_km;
        best_route = route;
      }
    }
    if (best_target_bus < 0 || best_route.size() < 2) continue;

    st.status = MobileStorageStatus::InTransit;
    st.target_bus_pos = best_target_bus;
    st.route_bus_positions = best_route;
    st.route_leg_distance_km.clear();
    st.route_leg_distance_km.reserve(best_route.size() - 1);
    for (size_t i = 0; i + 1 < best_route.size(); ++i) {
      const double leg_dist = edge_distance_km(transport_graph, best_route[i], best_route[i + 1]);
      if (!std::isfinite(leg_dist) || leg_dist <= kEps) {
        st.route_leg_distance_km.clear();
        break;
      }
      st.route_leg_distance_km.push_back(leg_dist);
    }
    if (st.route_leg_distance_km.empty()) {
      st.status = MobileStorageStatus::Stationary;
      st.route_bus_positions.clear();
      st.target_bus_pos = st.bus_pos;
      continue;
    }
    st.route_leg_index = 0;
    st.leg_remaining_hr = st.route_leg_distance_km[0] / speed;
    st.remaining_travel_hr = best_distance_km / speed;
    st.arrival_time_hr = hour + st.remaining_travel_hr;
  }
}

}  // namespace

/// Build a PF snapshot for a single resilience step and solve it.
void run_step_power_flow(const HybridPowerSystem& pf_template,
                         const std::vector<bool>& closed,
                         double load_multiplier,
                         const std::vector<MobileStorageState>& mess,
                         const std::vector<bool>& source_bus,
                         DistributionResilienceStepResult& sr) {
  HybridPowerSystem pf_sys = pf_template;

  // Apply branch topology.
  for (size_t bi = 0; bi < pf_sys.ac.branches.size(); ++bi) {
    pf_sys.ac.branches[bi].in_service = closed[bi];
  }

  // Scale loads by the hour's multiplier.
  for (auto& bus : pf_sys.ac.buses) {
    bus.pd_mw *= load_multiplier;
    bus.qd_mvar *= load_multiplier;
  }
  for (auto& ld : pf_sys.ac.loads) {
    ld.p_mw *= load_multiplier;
    ld.q_mvar *= load_multiplier;
  }

  // Model dispatching MESS units as generators.
  const auto bus_pos_map = make_bus_pos_map(pf_sys);
  for (const auto& st : mess) {
    if (st.status == MobileStorageStatus::InTransit) continue;
    if (st.dispatch_mw <= kEps) continue;

    Generator g;
    g.index = 9000 + st.storage_index;
    g.bus = pf_sys.ac.buses[static_cast<size_t>(st.bus_pos)].index;
    g.pg_mw = st.dispatch_mw;
    g.qg_mvar = 0.0;
    g.pmax_mw = st.pmax_mw * 2.0;
    g.pmin_mw = 0.0;
    g.qmax_mvar = st.pmax_mw * 2.0;
    g.qmin_mvar = -st.pmax_mw * 2.0;
    g.vg_pu = 1.0;
    g.in_service = true;
    g.name = "MESS-" + std::to_string(st.storage_index);

    const auto it = bus_pos_map.find(g.bus);
    if (it != bus_pos_map.end() && !source_bus[static_cast<size_t>(it->second)]) {
      g.is_slack = true;
      pf_sys.ac.buses[static_cast<size_t>(it->second)].bus_type = BusType::SLACK;
      pf_sys.ac.buses[static_cast<size_t>(it->second)].vm_pu = 1.0;
    }
    pf_sys.ac.generators.push_back(g);
  }

  PowerFlowOptions pf_opt;
  pf_opt.enable_auto_swing_selection = true;
  pf_opt.max_iter = 100;
  pf_opt.tol = 1e-6;
  const auto pf_result = hacdcpf::solve_power_flow_adaptive(pf_sys, pf_opt);
  sr.pf_converged = pf_result.converged;
  sr.pf_residual = pf_result.residual;

  sr.bus_voltages.reserve(pf_sys.ac.buses.size());
  for (size_t i = 0; i < pf_sys.ac.buses.size(); ++i) {
    BusVoltageStep bv;
    bv.bus_index = pf_sys.ac.buses[i].index;
    bv.vm_pu = i < pf_result.vm.size() ? pf_result.vm[i] : 1.0;
    bv.va_deg = i < pf_result.va.size() ? pf_result.va[i] * (180.0 / kPi) : 0.0;
    sr.bus_voltages.push_back(bv);
  }

  sr.branch_flows.reserve(pf_sys.ac.branches.size());
  for (size_t bi = 0; bi < pf_sys.ac.branches.size(); ++bi) {
    const auto& br = pf_sys.ac.branches[bi];
    BranchFlowStep bf;
    bf.branch_index = br.index != 0 ? br.index : static_cast<int>(bi + 1);
    bf.from_bus = br.from_bus;
    bf.to_bus = br.to_bus;
    if (bi < pf_result.branch_flows.size()) {
      bf.pf_mw = pf_result.branch_flows[bi].pf_mw;
      bf.pt_mw = pf_result.branch_flows[bi].pt_mw;
      bf.qf_mvar = pf_result.branch_flows[bi].qf_mvar;
      bf.qt_mvar = pf_result.branch_flows[bi].qt_mvar;
      if (br.rate_a_mva > 0.0) {
        const double s_from = std::sqrt(bf.pf_mw * bf.pf_mw + bf.qf_mvar * bf.qf_mvar);
        bf.loading_percent = 100.0 * s_from / br.rate_a_mva;
      }
    }
    sr.branch_flows.push_back(bf);
  }
}

std::string DistributionResilienceResult::summary() const {
  std::ostringstream ss;
  ss << "Distribution Resilience Result"
     << "\n  steps = " << steps.size()
     << "\n  total demand = " << total_demand_mwh << " MWh"
     << "\n  total served = " << total_served_mwh << " MWh"
     << "\n  total shed = " << total_shed_mwh << " MWh"
     << "\n  resilience index = " << resilience_index
     << "\n  avg restoration ratio = " << avg_restoration_ratio
     << "\n  final restoration ratio = " << final_restoration_ratio
     << "\n  peak shed = " << peak_shed_mw << " MW"
     << "\n  MESS delivered = " << mess_energy_delivered_mwh << " MWh"
     << "\n  MESS travel = " << mess_travel_distance_km << " km"
     << "\n  switch actions = " << total_switch_actions
     << "\n  repaired faults = " << total_repaired_faults
     << "\n  status = " << status;
  return ss.str();
}

void apply_distribution_resilience_demo_data(HybridPowerSystem& sys) {
  const auto bus_pos = make_bus_pos_map(sys);
  std::vector<double> bus_load(sys.ac.buses.size(), 0.0);

  if (!sys.ac.loads.empty()) {
    for (const auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;
      const auto it = bus_pos.find(ld.bus);
      if (it == bus_pos.end()) continue;
      bus_load[static_cast<size_t>(it->second)] += ld.p_mw * nonnegative_scale(ld.scaling);
    }
  } else {
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) bus_load[i] = std::max(0.0, sys.ac.buses[i].pd_mw);
  }

  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    auto& bus = sys.ac.buses[i];
    const double load = bus_load[i];
    if (bus.n_customers <= 0) bus.n_customers = std::max(1, static_cast<int>(std::lround(load * kDemoCustomersPerMw)));
    if (bus.importance <= 1.0 + kEps) {
      bus.importance = load > kDemoLoadHighThreshold ? 3.0 : (load > kDemoLoadMedThreshold ? 2.0 : 1.0);
    }
  }
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    auto& br = sys.ac.branches[i];
    if (br.length_km <= 0.0) br.length_km = kDemoBranchBaseLength + kDemoBranchLengthIncrement * static_cast<double>(i);
    if (br.mttr_hr <= 0.0) br.mttr_hr = kDemoRepairHr;
  }

  if (!sys.mobile_storage.empty()) {
    for (auto& st : sys.mobile_storage) {
      if (st.e_rated_mwh <= 0.0) st.e_rated_mwh = kDemoMessEnergyMwh;
      if (st.pmax_mw <= 0.0) st.pmax_mw = kDemoMessPmaxMw;
      if (st.p_rated_mw <= 0.0) st.p_rated_mw = st.pmax_mw;
      if (st.e_mwh <= 0.0) st.e_mwh = st.soc_init * st.e_rated_mwh;
      if (st.e_consumption_mwh_km <= 0.0) st.e_consumption_mwh_km = kDemoMessConsumptionMwhKm;
      if (st.max_travel_distance_km <= 0.0) st.max_travel_distance_km = kDemoMessMaxTravelKm;
    }
    return;
  }

  int source_bus_idx = !sys.ac.buses.empty() ? sys.ac.buses.front().index : 1;
  for (const auto& bus : sys.ac.buses) {
    if (bus.bus_type == BusType::SLACK) {
      source_bus_idx = bus.index;
      break;
    }
  }

  std::vector<int> demand_order(sys.ac.buses.size());
  std::iota(demand_order.begin(), demand_order.end(), 0);
  std::sort(demand_order.begin(), demand_order.end(), [&](int a, int b) {
    return bus_load[static_cast<size_t>(a)] > bus_load[static_cast<size_t>(b)];
  });
  const int second_bus = !demand_order.empty() ? sys.ac.buses[static_cast<size_t>(demand_order.front())].index : source_bus_idx;

  for (int k = 0; k < 2; ++k) {
    MobileStorage st;
    st.index = k + 1;
    st.name = k == 0 ? "MESS-1" : "MESS-2";
    st.bus = k == 0 ? source_bus_idx : second_bus;
    st.in_service = true;
    st.status = MobileStorageStatus::Stationary;
    st.p_rated_mw = kDemoMessPmaxMw;
    st.pmax_mw = kDemoMessPmaxMw;
    st.pmin_mw = -kDemoMessPmaxMw;
    st.e_rated_mwh = kDemoMessEnergyMwh;
    st.soc_init = kDemoMessSocInit;
    st.soc_min = kDemoMessSocMin;
    st.soc_max = kDemoMessSocMax;
    st.e_mwh = st.soc_init * st.e_rated_mwh;
    st.eta_charge = kDemoMessEta;
    st.eta_discharge = kDemoMessEta;
    st.e_consumption_mwh_km = kDemoMessConsumptionMwhKm;
    st.max_travel_distance_km = kDemoMessMaxTravelKm;
    sys.mobile_storage.push_back(st);
  }
}

/// Run the multi-hour distribution resilience assessment.
///
/// Algorithm overview:
///   1. Validate inputs (horizon, timestep, AC network).
///   2. Collect base loads, grid sources, fixed/mobile storage, faults,
///      transport graph.
///   3. For each time step t in [0, horizon):
///      a. Sample load/RES profiles -> load_mult, res_mult.
///      b. Advance MESS units in transit (discrete time-space progression).
///      c. Compute active branch status (faults, repairs).
///      d. If reconfiguration enabled, greedily close tie switches.
///      e. Identify connected components (islands) via DFS.
///      f. Evaluate each island: supply/demand balance, storage dispatch,
///         priority-weighted load shedding.
///      g. If MESS dispatch enabled and any island sheds load, route idle
///         MESS to deficit islands (Dijkstra with caching).
///      h. Optionally run AC power flow for physical validation.
///      i. Accumulate step results.
///   4. Compute summary statistics (RI, avg restoration, peak shed).
DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& input_sys,
    const DistributionResilienceOptions& opts) {
  // Dispatch to the strict multi-period MIP when requested.
  // run_distribution_resilience_mip_assessment() is the canonical entry
  // point for MultiPeriodMIPLinDistFlow; callers that set opts.model to that
  // value will now reach it through this unified gateway.
  if (opts.model == DistributionResilienceModel::MultiPeriodMIPLinDistFlow) {
    return run_distribution_resilience_mip_assessment(input_sys, opts);
  }

  DistributionResilienceResult result;
  if (opts.horizon_hours <= 0 || opts.time_step_hr <= 0.0) {
    result.feasible = false;
    result.status = "Invalid horizon or timestep";
    return result;
  }

  const auto& sys = input_sys;
  if (sys.ac.buses.empty() || sys.ac.branches.empty()) {
    result.feasible = false;
    result.status = "System has no AC distribution network";
    return result;
  }

  const auto bus_pos = make_bus_pos_map(sys);
  const auto branch_pos = make_branch_pos_map(sys);
  const auto base_loads = collect_bus_loads(sys, opts.load_scale_factor, bus_pos);
  const auto source_bus = identify_grid_source_buses(sys, bus_pos);
  auto fixed_storage = collect_fixed_storage(sys, bus_pos);
  auto mess = collect_mobile_storage(sys, bus_pos);
  const auto faults = build_faults(sys, opts, branch_pos);
  const auto transport_graph = build_transport_graph(sys, opts, bus_pos);

  for (const auto& f : faults) {
    result.fault_sequence.push_back({f.branch_index, f.start_hr, f.repair_hr, f.name});
  }

  const auto& load_prof = opts.load_profile.empty() ? kDefaultLoadProfile : opts.load_profile;
  const auto& res_prof = opts.renewable_profile.empty() ? kDefaultRenewableProfile : opts.renewable_profile;

  const int steps = std::max(1, static_cast<int>(std::ceil(opts.horizon_hours / opts.time_step_hr)));
  result.steps.reserve(static_cast<size_t>(steps));

  std::vector<bool> previous_closed(sys.ac.branches.size(), false);
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) previous_closed[i] = sys.ac.branches[i].in_service;

  HybridPowerSystem pf_template;
  if (opts.run_power_flow) {
    pf_template = sys;
    pf_template.dc = {};
    pf_template.vsc_converters.clear();
    pf_template.dc.dcdc_converters.clear();
  }

  std::vector<BusLoadEntry> loads;
  std::vector<double> demand_by_bus;
  std::vector<int> open_branch_ids;
  std::vector<bool> fault_active;

  for (int step = 0; step < steps; ++step) {
    const double hour = static_cast<double>(step) * opts.time_step_hr;

    const double load_mult = sample_profile(load_prof, hour);
    const double res_mult = sample_profile(res_prof, hour);

    loads = base_loads;
    for (auto& ld : loads) ld.demand_mw *= load_mult;
    demand_by_bus.assign(sys.ac.buses.size(), 0.0);
    for (const auto& ld : loads) demand_by_bus[static_cast<size_t>(ld.bus_pos)] += ld.demand_mw;

    advance_mess_transit(hour, opts.time_step_hr, opts, mess, result.mess_travel_distance_km);

    DistributionResilienceStepResult sr;
    sr.step_index = step;
    sr.hour = hour;
    sr.load_multiplier = load_mult;
    sr.res_multiplier = res_mult;
    open_branch_ids.clear();
    fault_active.clear();
    sr.repaired_faults = 0;
    sr.active_faults = 0;
    auto closed = active_branch_status(sys, previous_closed, faults, hour,
                                       sr.active_faults, sr.repaired_faults,
                                       open_branch_ids, fault_active);
    if (opts.allow_reconfiguration) {
      greedily_reconfigure(sys, demand_by_bus, source_bus, fault_active, bus_pos, closed,
                           sr.closed_tie_branch_ids, opts.max_reconfig_iterations);
    }
    sr.open_ac_branch_ids = open_branch_ids;
    sr.switch_actions = 0;
    for (size_t i = 0; i < closed.size(); ++i) {
      if (closed[i] != previous_closed[i]) ++sr.switch_actions;
    }
    previous_closed = closed;

    const auto comp = compute_components(sys, closed, bus_pos);
    const auto islands = build_islands(comp, source_bus, demand_by_bus);
    sr.island_count = static_cast<int>(islands.size());

    // ── Graph topology analysis: cut-vertices and bridges for restoration priority
    {
      namespace gr = hacdcpf::graph;
      // Build a graph snapshot reflecting the current closed[]/open[] state.
      HybridPowerSystem sys_snap = sys;
      for (size_t i = 0; i < sys_snap.ac.branches.size(); ++i)
        sys_snap.ac.branches[i].in_service = closed[i];
      const auto g    = gr::build_power_system_graph(sys_snap);
      const auto topo = gr::analyze_topology(g);
      sr.cut_vertex_bus_ids = topo.cut_vertex_bus_ids;
      // Map graph bridge edge IDs back to AC branch indices.
      sr.bridge_branch_ids.clear();
      sr.bridge_branch_ids.reserve(topo.bridge_edge_ids.size());
      const int n_branches = static_cast<int>(sys.ac.branches.size());
      for (int eid : topo.bridge_edge_ids) {
        if (eid >= 0 && eid < n_branches)
          sr.bridge_branch_ids.push_back(sys.ac.branches[static_cast<size_t>(eid)].index);
      }
    }

    std::vector<ComponentEvaluation> evals(islands.size());
    const StepContext ctx{sys, loads, comp, bus_pos, fixed_storage, mess,
                          opts.allow_mess_dispatch, opts.time_step_hr, res_mult};
    for (size_t i = 0; i < islands.size(); ++i) {
      evals[i] = evaluate_island(ctx, islands[i]);
    }

    if (opts.allow_mess_dispatch) {
      for (const auto& ev : evals) {
        for (size_t mi = 0; mi < mess.size(); ++mi) {
          if (ev.mess_dispatch[mi] > kEps) mess[mi].dispatch_mw = ev.mess_dispatch[mi];
        }
      }
      bool any_shed = false;
      for (const auto& ev : evals) {
        if (ev.shed_mw > kEps) { any_shed = true; break; }
      }
      if (any_shed) {
        assign_mess_movements(hour, opts.time_step_hr, transport_graph, comp, islands, evals,
                              opts, mess);
      }
    }

    double step_mess_energy = 0.0;
    sr.shed_by_priority.assign(4, 0.0);
    for (size_t i = 0; i < evals.size(); ++i) {
      apply_dispatch_to_storage(fixed_storage, mess, evals[i], opts.time_step_hr, step_mess_energy);
      sr.total_demand_mw += evals[i].total_demand_mw;
      sr.served_mw += evals[i].served_mw;
      sr.shed_mw += evals[i].shed_mw;
      sr.weighted_shed_mw += evals[i].weighted_shed_mw;
      sr.total_res_mw += evals[i].res_mw;
      for (size_t p = 0; p < 4 && p < evals[i].shed_by_priority.size(); ++p) {
        sr.shed_by_priority[p] += evals[i].shed_by_priority[p];
      }
    }
    result.mess_energy_delivered_mwh += step_mess_energy;

    sr.restoration_ratio = sr.total_demand_mw > kEps ? sr.served_mw / sr.total_demand_mw : 1.0;
    for (const auto& st : mess) {
      MESSStateStep ms;
      ms.storage_index = st.storage_index;
      ms.bus = sys.ac.buses[static_cast<size_t>(st.bus_pos)].index;
      ms.target_bus = sys.ac.buses[static_cast<size_t>(st.target_bus_pos)].index;
      ms.status = mobile_storage_status_str(st.status);
      ms.dispatch_mw = st.dispatch_mw;
      ms.energy_mwh = st.energy_mwh;
      ms.soc = st.e_rated_mwh > kEps ? st.energy_mwh / st.e_rated_mwh : 0.0;
      ms.arrival_time_hr = st.arrival_time_hr;
      ms.remaining_travel_hr = st.remaining_travel_hr;
      sr.mess_states.push_back(ms);
    }

    if (opts.run_power_flow) {
      run_step_power_flow(pf_template, closed, load_mult, mess, source_bus, sr);
    }

    result.total_demand_mwh += sr.total_demand_mw * opts.time_step_hr;
    result.total_served_mwh += sr.served_mw * opts.time_step_hr;
    result.total_shed_mwh += sr.shed_mw * opts.time_step_hr;
    result.weighted_unserved_mwh += sr.weighted_shed_mw * opts.time_step_hr;
    result.total_switch_actions += sr.switch_actions;
    result.total_repaired_faults = std::max(result.total_repaired_faults, sr.repaired_faults);
    result.peak_shed_mw = std::max(result.peak_shed_mw, sr.shed_mw);
    result.steps.push_back(sr);

    if (opts.progress_callback && !opts.progress_callback(step, sr.restoration_ratio, sr.shed_mw)) {
      result.status = "Cancelled";
      result.feasible = false;
      break;
    }
  }

  if (!result.steps.empty()) {
    result.resilience_index = result.total_demand_mwh > kEps
                                  ? result.total_served_mwh / result.total_demand_mwh
                                  : 1.0;
    double ratio_sum = 0.0;
    for (const auto& s : result.steps) ratio_sum += s.restoration_ratio;
    result.avg_restoration_ratio = ratio_sum / static_cast<double>(result.steps.size());
    result.final_restoration_ratio = result.steps.back().restoration_ratio;
  }
  // Sync legacy fields.
  result.total_ens_mwh = result.total_shed_mwh;
  result.max_curtailment_mw = result.peak_shed_mw;
  result.completed = result.feasible;
  if (!result.steps.empty()) {
    result.restoration_time_hr = result.steps.back().hour;
  }
  if (result.status.empty()) result.status = result.feasible ? "Completed" : "Failed";
  return result;
}

}  // namespace hacdcpf::analysis

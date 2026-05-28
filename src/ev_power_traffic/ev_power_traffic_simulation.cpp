#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {
namespace {

constexpr double kTol = 1e-9;

// Forward declarations (definitions appear later in this file)
int route_arrival_step(const RouteAlternative& route,
                       const std::unordered_map<int, std::size_t>& link_pos,
                       const EVPowerTrafficProblem& problem,
                       int departure_step,
                       double dt_hr);
double n_plugs_for_station(const HybridPowerSystem& sys, int station_id);

bool is_capacity_aware_greedy(AssignmentModel model) {
  return model == AssignmentModel::CapacityAwareGreedy;
}

bool is_system_optimal_solver(AssignmentModel model) {
  return model == AssignmentModel::SystemOptimalLP ||
         model == AssignmentModel::SystemOptimalMILP;
}

bool is_system_optimal_mip(AssignmentModel model) {
  return model == AssignmentModel::SystemOptimalMILP;
}

template <typename T>
const T* find_by_index(const std::vector<T>& values, int index) {
  for (const auto& v : values) {
    if (v.index == index) return &v;
  }
  return nullptr;
}

template <typename T>
T* find_by_index(std::vector<T>& values, int index) {
  for (auto& v : values) {
    if (v.index == index) return &v;
  }
  return nullptr;
}

double profile_value(const std::vector<double>& values,
                     int step,
                     double fallback) {
  if (step >= 0 && step < static_cast<int>(values.size())) {
    return values[static_cast<std::size_t>(step)];
  }
  return fallback;
}

bool availability_value(const std::vector<bool>& values,
                        int step,
                        bool fallback) {
  if (step >= 0 && step < static_cast<int>(values.size())) {
    return values[static_cast<std::size_t>(step)];
  }
  return fallback;
}

double link_capacity_vehicles(const TrafficLink& link,
                              int step,
                              double dt_hr) {
  const bool available =
      availability_value(link.availability_profile, step, link.available);
  if (!available || dt_hr <= 0.0) return 0.0;
  const double cap_per_hr =
      profile_value(link.capacity_profile_veh_per_hr,
                    step,
                    link.capacity_veh_per_hr);
  return std::max(0.0, cap_per_hr * dt_hr);
}

double link_travel_time_hr(const TrafficLink& link,
                           double inflow_vehicles,
                           double capacity_vehicles) {
  if (capacity_vehicles <= kTol) {
    return std::numeric_limits<double>::infinity();
  }
  const double t0 = std::max(link.free_flow_time_hr, 0.0);
  if (t0 <= kTol) return 0.0;
  const double ratio = std::max(0.0, inflow_vehicles / capacity_vehicles);
  return t0 * (1.0 + link.alpha * std::pow(ratio, link.beta));
}

double station_price(const EVPowerTrafficProblem& problem,
                     const EVPowerTrafficOptions& options,
                     int station_id,
                     int step) {
  for (const auto& p : problem.station_prices) {
    if (p.station_id == station_id) {
      return profile_value(p.price_per_kwh,
                           step,
                           options.default_station_price_per_kwh);
    }
  }
  return options.default_station_price_per_kwh;
}

double route_generalized_cost(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options,
    const RouteAlternative& route,
    int step,
    const std::unordered_map<int, std::size_t>& link_pos,
    const std::vector<double>& link_used,
    const std::vector<double>& link_capacity,
    // Optional: current station-reservation state for omega_Q estimation.
    // Pass nullptr (default) to disable queueing cost (single-pass mode or LP).
    const std::unordered_map<int, std::vector<double>>* station_reserved_kw_ptr = nullptr,
    const std::unordered_map<int, double>* charge_cap_kw_ptr = nullptr) {
  double travel_hr = 0.0;
  for (int link_index : route.link_indices) {
    auto pos_it = link_pos.find(link_index);
    if (pos_it == link_pos.end()) return std::numeric_limits<double>::infinity();
    const std::size_t pos = pos_it->second;
    const auto& link = problem.traffic.links[pos];
    if (link_capacity[pos] <= kTol) return std::numeric_limits<double>::infinity();
    travel_hr += link_travel_time_hr(link, link_used[pos], link_capacity[pos]);
  }

  double charging_cost = 0.0;
  for (const auto& stop : route.charging_stops) {
    charging_cost += station_price(problem, options, stop.station_id, step) *
                     stop.requested_energy_kwh_per_vehicle;
  }

  // ── omega_Q * Σ_s W_{s, kappa(s,r,k)} (eq. evpt-generalized-cost) ────────
  // W is estimated from current station reservations as an M/D/1 approximation.
  // Only applied when queueing_weight > 0 and reservation data is provided.
  double queueing_cost = 0.0;
  if (options.queueing_weight > kTol && station_reserved_kw_ptr != nullptr &&
      !route.charging_stops.empty()) {
    const int arrival_step =
        route_arrival_step(route, link_pos, problem, step, options.time_step_hr);
    for (const auto& stop : route.charging_stops) {
      const double p_per_veh =
          stop.max_charge_kw_per_vehicle > 0.0
              ? stop.max_charge_kw_per_vehicle
              : options.default_route_stop_power_kw_per_vehicle;
      if (p_per_veh <= kTol) continue;
      auto res_it = station_reserved_kw_ptr->find(stop.station_id);
      if (res_it == station_reserved_kw_ptr->end()) continue;
      const int k = arrival_step;
      const double reserved_kw =
          (k >= 0 && k < static_cast<int>(res_it->second.size()))
              ? res_it->second[static_cast<std::size_t>(k)]
              : 0.0;
      const double reserved_veh = reserved_kw / p_per_veh;
      const double n_plug = n_plugs_for_station(problem.system, stop.station_id);
      // queue estimate: vehicles waiting because plugs are occupied
      const double queue_est =
          std::isinf(n_plug) ? 0.0 : std::max(0.0, reserved_veh - n_plug);
      // service rate mu [veh/hr] from station charge capacity
      const double cap_kw =
          (charge_cap_kw_ptr && charge_cap_kw_ptr->count(stop.station_id))
              ? charge_cap_kw_ptr->at(stop.station_id)
              : options.default_station_power_kw;
      const double mu = cap_kw / std::max(p_per_veh, kTol);
      queueing_cost += options.queueing_weight *
                       (queue_est / std::max(mu, 1e-9));
    }
  }

  return options.value_of_time_per_hr * travel_hr +
         options.station_energy_cost_weight * charging_cost +
         queueing_cost +
         route.toll_cost;
}

double route_free_flow_generalized_cost(const EVPowerTrafficProblem& problem,
                                        const EVPowerTrafficOptions& options,
                                        const RouteAlternative& route,
                                        int step,
                                        const std::unordered_map<int, std::size_t>& link_pos) {
  double travel_hr = 0.0;
  for (int link_index : route.link_indices) {
    auto pos_it = link_pos.find(link_index);
    if (pos_it == link_pos.end()) return std::numeric_limits<double>::infinity();
    travel_hr += std::max(0.0, problem.traffic.links[pos_it->second].free_flow_time_hr);
  }

  double charging_cost = 0.0;
  for (const auto& stop : route.charging_stops) {
    charging_cost += station_price(problem, options, stop.station_id, step) *
                     stop.requested_energy_kwh_per_vehicle;
  }

  return options.value_of_time_per_hr * travel_hr +
         options.station_energy_cost_weight * charging_cost +
         route.toll_cost;
}

double route_travel_time_hr(const EVPowerTrafficProblem& problem,
                            const RouteAlternative& route,
                            const std::unordered_map<int, std::size_t>& link_pos,
                            const std::vector<double>& link_used,
                            const std::vector<double>& link_capacity) {
  double travel_hr = 0.0;
  for (int link_index : route.link_indices) {
    auto pos_it = link_pos.find(link_index);
    if (pos_it == link_pos.end()) return std::numeric_limits<double>::infinity();
    const std::size_t pos = pos_it->second;
    if (link_capacity[pos] <= kTol) return std::numeric_limits<double>::infinity();
    travel_hr += link_travel_time_hr(problem.traffic.links[pos],
                                     link_used[pos],
                                     link_capacity[pos]);
  }
  return travel_hr;
}

double route_residual_capacity(const RouteAlternative& route,
                               const std::unordered_map<int, std::size_t>& link_pos,
                               const std::vector<double>& link_used,
                               const std::vector<double>& link_capacity) {
  double residual = std::numeric_limits<double>::infinity();
  for (int link_index : route.link_indices) {
    auto pos_it = link_pos.find(link_index);
    if (pos_it == link_pos.end()) return 0.0;
    const std::size_t pos = pos_it->second;
    residual = std::min(residual, link_capacity[pos] - link_used[pos]);
  }
  if (!std::isfinite(residual)) return 0.0;
  return std::max(0.0, residual);
}

double route_station_residual_vehicles(
    const RouteAlternative& route,
    int arrival_step,
    const EVPowerTrafficOptions& options,
    const std::unordered_map<int, double>& charge_cap_kw,
    const std::unordered_map<int, std::vector<double>>& station_reserved_kw) {
  double residual = std::numeric_limits<double>::infinity();
  for (const auto& stop : route.charging_stops) {
    const double p_per_vehicle =
        stop.max_charge_kw_per_vehicle > 0.0
            ? stop.max_charge_kw_per_vehicle
            : options.default_route_stop_power_kw_per_vehicle;
    if (p_per_vehicle <= kTol) continue;
    const double cap =
        charge_cap_kw.count(stop.station_id)
            ? charge_cap_kw.at(stop.station_id)
            : options.default_station_power_kw;
    auto reserved_it = station_reserved_kw.find(stop.station_id);
    for (int offset = 0; offset < std::max(1, stop.dwell_steps); ++offset) {
      const int k = arrival_step + offset;
      double used = 0.0;
      if (reserved_it != station_reserved_kw.end() &&
          k >= 0 && k < static_cast<int>(reserved_it->second.size())) {
        used = reserved_it->second[static_cast<std::size_t>(k)];
      }
      residual = std::min(residual, (cap - used) / p_per_vehicle);
    }
  }
  if (!std::isfinite(residual)) return std::numeric_limits<double>::infinity();
  return std::max(0.0, residual);
}

void reserve_station_power(
    const RouteAlternative& route,
    int arrival_step,
    double vehicles,
    const EVPowerTrafficOptions& options,
    std::unordered_map<int, std::vector<double>>& station_reserved_kw) {
  if (vehicles <= kTol) return;
  for (const auto& stop : route.charging_stops) {
    const double p_per_vehicle =
        stop.max_charge_kw_per_vehicle > 0.0
            ? stop.max_charge_kw_per_vehicle
            : options.default_route_stop_power_kw_per_vehicle;
    if (p_per_vehicle <= kTol) continue;
    auto& reserved = station_reserved_kw[stop.station_id];
    for (int offset = 0; offset < std::max(1, stop.dwell_steps); ++offset) {
      const int k = arrival_step + offset;
      if (k >= 0 && k < static_cast<int>(reserved.size())) {
        reserved[static_cast<std::size_t>(k)] += p_per_vehicle * vehicles;
      }
    }
  }
}

std::vector<const RouteAlternative*> candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVDemand& demand) {
  std::vector<const RouteAlternative*> out;
  if (!demand.candidate_route_indices.empty()) {
    for (int route_index : demand.candidate_route_indices) {
      if (const auto* route = find_by_index(problem.routes, route_index)) {
        if (route->origin_node == demand.origin_node &&
            route->destination_node == demand.destination_node) {
          out.push_back(route);
        }
      }
    }
  } else {
    for (const auto& route : problem.routes) {
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node) {
        out.push_back(&route);
      }
    }
  }
  return out;
}

int route_arrival_step(const RouteAlternative& route,
                       const std::unordered_map<int, std::size_t>& link_pos,
                       const EVPowerTrafficProblem& problem,
                       int departure_step,
                       double dt_hr) {
  double travel_hr = 0.0;
  for (int link_index : route.link_indices) {
    auto pos_it = link_pos.find(link_index);
    if (pos_it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[pos_it->second];
    travel_hr += std::max(0.0, link.free_flow_time_hr);
  }
  const int offset = std::max(0, static_cast<int>(std::ceil(travel_hr / dt_hr)));
  return departure_step + offset;
}

double charging_power_limit(const RouteChargingStop& stop,
                            const EVPowerTrafficOptions& options,
                            double vehicles) {
  const double per_vehicle =
      stop.max_charge_kw_per_vehicle > 0.0
          ? stop.max_charge_kw_per_vehicle
          : options.default_route_stop_power_kw_per_vehicle;
  return std::max(0.0, per_vehicle * vehicles);
}

double discharge_power_limit(const RouteChargingStop& stop,
                             double vehicles) {
  return std::max(0.0, stop.max_discharge_kw_per_vehicle * vehicles);
}

double full_energy_power_kw_per_vehicle(const RouteChargingStop& stop,
                                        const EVPowerTrafficOptions& options,
                                        double dt_hr) {
  const double e_req = std::max(0.0, stop.requested_energy_kwh_per_vehicle);
  if (e_req <= kTol) return 0.0;
  const double p_max =
      stop.max_charge_kw_per_vehicle > 0.0
          ? stop.max_charge_kw_per_vehicle
          : options.default_route_stop_power_kw_per_vehicle;
  const int dwell = std::max(1, stop.dwell_steps);
  const double p_req =
      e_req / (std::max(options.default_charging_efficiency, kTol) *
               std::max(dt_hr, kTol) * dwell);
  if (p_req > p_max + 1e-7) {
    return std::numeric_limits<double>::infinity();
  }
  return p_req;
}

bool route_can_deliver_requested_energy(const RouteAlternative& route,
                                        const EVPowerTrafficOptions& options,
                                        double dt_hr) {
  for (const auto& stop : route.charging_stops) {
    if (!std::isfinite(full_energy_power_kw_per_vehicle(stop, options, dt_hr))) {
      return false;
    }
  }
  return true;
}

// Battery SOC feasibility check (eqs. evpt-route-soc, evpt-route-soc-bounds).
// Returns true when drive_energy_kwh_per_veh_km == 0 on all links (no data),
// or when demand.initial_energy_kwh < 0 (not set).  Otherwise simulates E along
// the route: traction energy is consumed per link; all charging stops are
// assumed to occur after the last road segment (conservative single-stop model).
bool route_soc_feasible(const RouteAlternative& route,
                        const EVDemand& demand,
                        const EVPowerTrafficProblem& problem,
                        const std::unordered_map<int, std::size_t>& link_pos) {
  // Skip check when initial energy is not specified
  if (demand.initial_energy_kwh < 0.0) return true;

  // Skip check when no link has drive-energy data
  bool has_drive_data = false;
  for (int link_idx : route.link_indices) {
    auto it = link_pos.find(link_idx);
    if (it != link_pos.end() &&
        problem.traffic.links[it->second].drive_energy_kwh_per_veh_km > kTol) {
      has_drive_data = true;
      break;
    }
  }
  if (!has_drive_data) return true;

  const double E_min = demand.energy_min_kwh;
  const double E_max = demand.energy_max_kwh >= 0.0
                           ? demand.energy_max_kwh
                           : std::numeric_limits<double>::infinity();
  double E = demand.initial_energy_kwh;
  if (E < E_min - kTol || E > E_max + kTol) return false;

  // Traverse road links and deplete traction energy (eq. evpt-route-soc).
  // Charging stops are modelled as occurring after all links.
  for (int link_idx : route.link_indices) {
    auto it = link_pos.find(link_idx);
    if (it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[it->second];
    E -= link.drive_energy_kwh_per_veh_km * link.length_km;
    if (E < E_min - kTol) return false;  // stranded before reaching station
  }

  // Apply charging stop energy (assuming stop is at/after destination)
  for (const auto& stop : route.charging_stops) {
    E += stop.requested_energy_kwh_per_vehicle;
    E = std::min(E, E_max);  // battery cannot exceed capacity
  }

  return E >= E_min - kTol;
}

void add_warning(EVPowerTrafficResult& result, const std::string& warning) {
  result.warnings.push_back(warning);
}

std::unordered_map<int, double> station_capacity_kw(const HybridPowerSystem& sys,
                                                    const EVPowerTrafficOptions& options) {
  std::unordered_map<int, double> cap;
  for (const auto& st : sys.ac.charging_stations) {
    double p = st.max_power_kw;
    if (p <= kTol) {
      p = st.p_fast_max_kw * st.n_fast + st.p_slow_max_kw * st.n_slow;
    }
    if (p <= kTol) p = options.default_station_power_kw;
    cap[st.index] = p * std::clamp(st.simultaneity_factor, 0.0, 1.0);
  }
  return cap;
}

std::unordered_map<int, double> station_discharge_capacity_kw(
    const HybridPowerSystem& sys) {
  std::unordered_map<int, double> cap;
  for (const auto& charger : sys.ac.chargers) {
    if (!charger.in_service || !charger.v2g_capable) continue;
    cap[charger.station_id] += std::max(0.0, charger.p_dis_max_kw);
  }
  return cap;
}

double total_base_load_mw(const HybridPowerSystem& sys) {
  double p = 0.0;
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service) p += std::max(0.0, bus.pd_mw);
  }
  for (const auto& load : sys.ac.loads) {
    if (load.in_service) p += std::max(0.0, load.p_mw * load.scaling);
  }
  return p;
}

double total_generation_capacity_mw(const HybridPowerSystem& sys,
                                    const EVPowerTrafficOptions& options) {
  double p = std::max(0.0, options.external_grid_capacity_mw);
  for (const auto& gen : sys.ac.generators) {
    if (gen.in_service) p += std::max(0.0, gen.pmax_mw);
  }
  for (const auto& gen : sys.ac.static_generators) {
    if (gen.in_service) {
      p += std::max({0.0, gen.pmax_mw, gen.p_rated_mw, gen.p_mw});
    }
  }
  for (const auto& gen : sys.ac.renewable_gens) {
    if (gen.in_service) p += std::max({0.0, gen.p_rated_mw, gen.p_mw});
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (pv.in_service) p += std::max({0.0, pv.pmax_mw, pv.p_mw});
  }
  return p;
}

double station_power_factor(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    return std::clamp(st->power_factor, 0.01, 1.0);
  }
  return 0.95;
}

int station_bus(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    return st->bus;
  }
  return 0;
}

// Returns N^plug_s: total usable plugs for station s (eq. evpt-queue-dynamics).
// Infers from n_fast + n_slow + num_chargers; returns infinity when none set.
double n_plugs_for_station(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    const int n = st->n_fast + st->n_slow + st->num_chargers;
    if (n > 0) return static_cast<double>(n);
  }
  return std::numeric_limits<double>::infinity();
}

std::vector<int> active_station_ids(const EVPowerTrafficProblem& problem) {
  std::vector<int> ids;
  std::unordered_set<int> seen;
  auto add = [&](int id) {
    if (id == 0 || seen.count(id)) return;
    seen.insert(id);
    ids.push_back(id);
  };
  for (const auto& st : problem.system.ac.charging_stations) add(st.index);
  for (const auto& route : problem.routes) {
    for (const auto& stop : route.charging_stops) add(stop.station_id);
  }
  for (const auto& session : problem.initial_sessions) add(session.station_id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

struct SystemOptimalColumn {
  enum class Kind { RouteFlow, UnservedDemand };

  Kind kind{Kind::RouteFlow};
  int demand_pos{-1};
  const RouteAlternative* route{nullptr};
  int departure_step{0};
  int arrival_step{0};
};

struct SystemOptimalSolve {
  bool solved{false};
  bool is_mip{false};
  bool proven_optimal{false};
  std::string backend;
  std::string status;
  double objective{0.0};
  double best_bound{0.0};
  double mip_gap{0.0};
  Eigen::VectorXd x;
  std::vector<SystemOptimalColumn> cols;
};

double inferred_minimization_bound(double objective, double mip_gap) {
  if (!std::isfinite(objective) || !std::isfinite(mip_gap)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return objective - std::max(0.0, mip_gap) * std::max(1.0, std::abs(objective));
}

void solve_system_optimal_assignment(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options,
    int T,
    double dt,
    const std::unordered_map<int, std::size_t>& link_pos,
    const std::vector<std::vector<double>>& link_capacity,
    const std::unordered_map<int, double>& charge_cap_kw,
    EVPowerTrafficResult& result,
    std::vector<std::vector<double>>& link_used,
    std::vector<EVChargingSession>& sessions,
    int& next_session_index) {
  SystemOptimalSolve cert;
  cert.is_mip = is_system_optimal_mip(options.assignment_model);

  std::vector<int> active_demand_positions;
  active_demand_positions.reserve(problem.demands.size());
  std::unordered_map<int, int> demand_eq_row;

  for (int di = 0; di < static_cast<int>(problem.demands.size()); ++di) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    if (demand.departure_step < 0 || demand.departure_step >= T ||
        demand.vehicles <= kTol) {
      continue;
    }
    demand_eq_row[di] = static_cast<int>(active_demand_positions.size());
    active_demand_positions.push_back(di);
    result.total_demand_vehicles += std::max(0.0, demand.vehicles);
  }

  const int n_eq = static_cast<int>(active_demand_positions.size());
  if (n_eq == 0) {
    result.optimization_solved = true;
    result.optimization_is_mip = cert.is_mip;
    result.optimization_proven_optimal = true;
    result.optimization_backend = cert.is_mip ? "MILP-empty" : "LP-empty";
    result.optimization_status = "empty";
    return;
  }

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<SystemOptimalColumn> cols;
  std::vector<double> obj;
  std::vector<engine::VariableMeta> vars;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<double> beq(static_cast<std::size_t>(n_eq), 0.0);

  auto add_col = [&](SystemOptimalColumn col,
                     double lb,
                     double ub,
                     double c,
                     const std::string& name,
                     bool integer) {
    const int idx = static_cast<int>(cols.size());
    cols.push_back(col);
    obj.push_back(c);
    vars.push_back({integer ? engine::VarType::Integer : engine::VarType::Continuous,
                    lb,
                    ub,
                    name});
    if (integer) mip.integer_idx.push_back(idx);
    return idx;
  };

  const bool requested_mip = is_system_optimal_mip(options.assignment_model);
  bool all_integer_demands = true;
  for (int di : active_demand_positions) {
    const double vehicles = problem.demands[static_cast<std::size_t>(di)].vehicles;
    if (std::abs(vehicles - std::round(vehicles)) > 1e-7) {
      all_integer_demands = false;
      break;
    }
  }
  const bool use_integer_vars = requested_mip && all_integer_demands;
  if (requested_mip && !all_integer_demands) {
    add_warning(result,
                "SystemOptimalMILP requested with noninteger demand; solving LP relaxation");
    cert.is_mip = false;
  }

  for (int di : active_demand_positions) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    const int eq = demand_eq_row.at(di);
    beq[static_cast<std::size_t>(eq)] = demand.vehicles;
    const auto routes = candidate_routes(problem, demand);
    for (const auto* route : routes) {
      if (!route_can_deliver_requested_energy(*route, options, dt)) continue;
      if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;
      const double route_cost =
          route_free_flow_generalized_cost(problem,
                                           options,
                                           *route,
                                           demand.departure_step,
                                           link_pos);
      if (!std::isfinite(route_cost)) continue;
      const int arrival_step =
          route_arrival_step(*route, link_pos, problem, demand.departure_step, dt);
      bool charging_window_inside_horizon = true;
      for (const auto& stop : route->charging_stops) {
        const int departure_step =
            arrival_step + std::max(1, stop.dwell_steps);
        if (arrival_step < 0 || arrival_step >= T || departure_step > T) {
          charging_window_inside_horizon = false;
          break;
        }
      }
      if (!charging_window_inside_horizon) continue;
      SystemOptimalColumn col;
      col.kind = SystemOptimalColumn::Kind::RouteFlow;
      col.demand_pos = di;
      col.route = route;
      col.departure_step = demand.departure_step;
      col.arrival_step = arrival_step;
      const int x_idx =
          add_col(col,
                  0.0,
                  demand.vehicles,
                  route_cost,
                  "x_d" + std::to_string(demand.index) + "_r" +
                      std::to_string(route->index) + "_t" +
                      std::to_string(demand.departure_step),
                  use_integer_vars);
      eq_trips.emplace_back(eq, x_idx, 1.0);
    }

    SystemOptimalColumn u_col;
    u_col.kind = SystemOptimalColumn::Kind::UnservedDemand;
    u_col.demand_pos = di;
    u_col.departure_step = demand.departure_step;
    const int u_idx =
        add_col(u_col,
                0.0,
                demand.vehicles,
                std::max(0.0, options.system_optimal_unserved_trip_penalty),
                "u_d" + std::to_string(demand.index) + "_t" +
                    std::to_string(demand.departure_step),
                use_integer_vars);
    eq_trips.emplace_back(eq, u_idx, 1.0);
  }

  const int n = static_cast<int>(cols.size());
  lp.c = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n; ++j) lp.c[j] = obj[static_cast<std::size_t>(j)];
  lp.vars = std::move(vars);

  lp.Aeq.resize(n_eq, n);
  lp.beq = Eigen::VectorXd::Zero(n_eq);
  for (int r = 0; r < n_eq; ++r) lp.beq[r] = beq[static_cast<std::size_t>(r)];
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();

  std::map<std::pair<int, int>, int> road_rows;
  std::map<std::pair<int, int>, int> station_rows;
  std::map<int, int> gen_rows;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  std::vector<double> b_vals;

  auto add_ineq_row = [&](double rhs) {
    const int row = static_cast<int>(b_vals.size());
    b_vals.push_back(rhs);
    return row;
  };

  auto road_row = [&](int step, int link_index, double rhs) {
    const auto key = std::make_pair(step, link_index);
    auto [it, inserted] = road_rows.emplace(key, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto station_row = [&](int step, int station_id, double rhs) {
    const auto key = std::make_pair(step, station_id);
    auto [it, inserted] = station_rows.emplace(key, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto gen_row = [&](int step, double rhs) {
    auto [it, inserted] = gen_rows.emplace(step, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  const double base_load_mw = total_base_load_mw(problem.system);
  const double gen_cap_mw = total_generation_capacity_mw(problem.system, options);
  const double gen_ev_cap_kw =
      std::max(0.0, (gen_cap_mw - base_load_mw) * 1000.0);

  for (int j = 0; j < n; ++j) {
    const auto& col = cols[static_cast<std::size_t>(j)];
    if (col.kind != SystemOptimalColumn::Kind::RouteFlow || col.route == nullptr) {
      continue;
    }
    if (options.enforce_road_capacity) {
      for (int link_index : col.route->link_indices) {
        auto pos_it = link_pos.find(link_index);
        if (pos_it == link_pos.end()) continue;
        const double cap =
            link_capacity[static_cast<std::size_t>(col.departure_step)][pos_it->second];
        const int row = road_row(col.departure_step, link_index, cap);
        ineq_trips.emplace_back(row, j, 1.0);
      }
    }
    for (const auto& stop : col.route->charging_stops) {
      const double p_per_vehicle =
          full_energy_power_kw_per_vehicle(stop, options, dt);
      if (p_per_vehicle <= kTol) continue;
      const double station_cap =
          charge_cap_kw.count(stop.station_id)
              ? charge_cap_kw.at(stop.station_id)
              : options.default_station_power_kw;
      for (int offset = 0; offset < std::max(1, stop.dwell_steps); ++offset) {
        const int k = col.arrival_step + offset;
        if (k < 0 || k >= T) continue;
        const int sr = station_row(k, stop.station_id, station_cap);
        ineq_trips.emplace_back(sr, j, p_per_vehicle);
        if (options.enforce_generation_capacity) {
          const int gr = gen_row(k, gen_ev_cap_kw);
          ineq_trips.emplace_back(gr, j, p_per_vehicle);
        }
      }
    }
  }

  lp.A.resize(static_cast<int>(b_vals.size()), n);
  lp.b = Eigen::VectorXd::Zero(static_cast<int>(b_vals.size()));
  for (int r = 0; r < static_cast<int>(b_vals.size()); ++r) {
    lp.b[r] = b_vals[static_cast<std::size_t>(r)];
  }
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();

  cert.cols = cols;
  if (!cert.is_mip || (mip.integer_idx.empty() && mip.binary_idx.empty())) {
    engine::SimplexOptions simp_opt;
    simp_opt.max_iter = 10000;
    simp_opt.feasibility_tol = 1e-8;
    simp_opt.optimality_tol = 1e-8;
    simp_opt.verbose = false;
    auto lp_res = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
    cert.x = lp_res.result.x;
    cert.solved = lp_res.result.stats.success;
    cert.backend = "NativeDualSimplex";
    cert.status = lp_res.result.stats.status;
    cert.objective = lp_res.result.stats.objective;
    cert.best_bound = cert.objective;
    cert.mip_gap = 0.0;
    cert.proven_optimal = cert.solved;
    cert.is_mip = false;
  } else {
    auto solve_with_native_bc = [&](const std::string& previous_failure) {
      engine::BCOptions bc_opt;
      bc_opt.max_nodes = std::max(1, options.system_optimal_max_nodes);
      bc_opt.time_limit_sec = std::max(0.0, options.system_optimal_time_limit_sec);
      bc_opt.gap_tol = std::max(0.0, options.system_optimal_mip_gap);
      bc_opt.verbose = false;
      bc_opt.use_simplex_lp_nodes = true;
      auto bc_res = engine::solve_milp_bc(mip, bc_opt);
      cert.x = bc_res.x;
      cert.solved = bc_res.stats.success;
      cert.backend = previous_failure.empty() ? "NativeB&C" : "HiGHS+NativeB&C";
      cert.status = previous_failure.empty()
          ? bc_res.stats.status
          : "HiGHS failed: " + previous_failure + "; NativeB&C: " +
                bc_res.stats.status;
      cert.objective = bc_res.stats.objective;
      cert.best_bound = bc_res.bc_stats.best_bound;
      cert.mip_gap = std::isfinite(bc_res.bc_stats.gap)
                         ? bc_res.bc_stats.gap
                         : bc_res.stats.mip_gap;
      cert.proven_optimal =
          cert.solved &&
          std::isfinite(cert.mip_gap) &&
          cert.mip_gap <= options.system_optimal_mip_gap + 1e-9;
    };

    engine::HighsAdapter highs;
    if (highs.available()) {
      auto highs_res = highs.solve_milp(mip);
      cert.x = highs_res.x;
      cert.solved = highs_res.stats.success;
      cert.backend = "HiGHS";
      cert.status = highs_res.stats.status;
      cert.objective = highs_res.stats.objective;
      cert.mip_gap = highs_res.stats.mip_gap;
      cert.best_bound = inferred_minimization_bound(cert.objective, cert.mip_gap);
      cert.proven_optimal =
          cert.solved &&
          std::isfinite(cert.mip_gap) &&
          cert.mip_gap <= options.system_optimal_mip_gap + 1e-9;
      if (!cert.solved || cert.x.size() != n) {
        std::string reason =
            cert.status.empty() ? std::string("unsuccessful solve") : cert.status;
        if (cert.solved && cert.x.size() != n) {
          reason += " (solution vector has wrong size)";
        }
        solve_with_native_bc(reason);
      }
    } else {
      solve_with_native_bc("");
    }
  }

  result.optimization_solved = cert.solved;
  result.optimization_is_mip = cert.is_mip;
  result.optimization_proven_optimal = cert.solved && cert.proven_optimal;
  result.optimization_backend = cert.backend;
  result.optimization_status = cert.status;
  result.optimization_objective = cert.objective;
  result.optimization_best_bound = cert.best_bound;
  result.optimization_mip_gap = cert.mip_gap;

  if (!cert.solved || cert.x.size() != n) {
    result.status = cert.status.empty() ? "system-optimal solve failed" : cert.status;
    add_warning(result, "system-optimal LP/MILP solve failed");
    for (int di : active_demand_positions) {
      const auto& demand = problem.demands[static_cast<std::size_t>(di)];
      result.unmet_demand_vehicles += demand.vehicles;
      result.assignments.push_back({demand.index,
                                    0,
                                    demand.departure_step,
                                    demand.vehicles,
                                    0.0,
                                    0.0,
                                    false,
                                    "system-optimal solve failed"});
    }
    return;
  }

  std::vector<std::pair<int, double>> positive_route_cols;
  std::vector<std::pair<int, double>> positive_unserved_cols;
  for (int j = 0; j < n; ++j) {
    double value = cert.x[j];
    if (!std::isfinite(value) || value <= 1e-7) continue;
    if (cert.is_mip) value = std::round(value);
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    if (col.kind == SystemOptimalColumn::Kind::RouteFlow) {
      positive_route_cols.emplace_back(j, value);
      if (col.route != nullptr) {
        for (int link_index : col.route->link_indices) {
          if (auto pos_it = link_pos.find(link_index); pos_it != link_pos.end()) {
            link_used[static_cast<std::size_t>(col.departure_step)][pos_it->second] +=
                value;
          }
        }
      }
    } else {
      positive_unserved_cols.emplace_back(j, value);
    }
  }

  for (const auto& [j, vehicles] : positive_route_cols) {
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    if (col.route == nullptr) continue;
    const auto& demand = problem.demands[static_cast<std::size_t>(col.demand_pos)];
    const double route_cost =
        route_generalized_cost(problem,
                               options,
                               *col.route,
                               col.departure_step,
                               link_pos,
                               link_used[static_cast<std::size_t>(col.departure_step)],
                               link_capacity[static_cast<std::size_t>(col.departure_step)]);
    const double route_travel_time =
        route_travel_time_hr(problem,
                             *col.route,
                             link_pos,
                             link_used[static_cast<std::size_t>(col.departure_step)],
                             link_capacity[static_cast<std::size_t>(col.departure_step)]);
    result.assignments.push_back({demand.index,
                                  col.route->index,
                                  col.departure_step,
                                  vehicles,
                                  route_cost,
                                  route_travel_time,
                                  true,
                                  cert.is_mip ? "served-system-optimal-milp"
                                              : "served-system-optimal-lp"});
    result.served_demand_vehicles += vehicles;
    if (std::isfinite(route_travel_time)) {
      result.total_travel_time_hr += vehicles * route_travel_time;
    }
    if (std::isfinite(route_cost)) {
      result.total_assignment_cost += vehicles * route_cost;
    }

    for (const auto& stop : col.route->charging_stops) {
      if (col.arrival_step >= T) continue;
      EVChargingSession session;
      session.index = next_session_index++;
      session.station_id = stop.station_id;
      session.arrival_step = col.arrival_step;
      session.departure_step =
          std::min(T, col.arrival_step + std::max(1, stop.dwell_steps));
      session.vehicle_count = vehicles;
      const double e_req =
          std::max(0.0, stop.requested_energy_kwh_per_vehicle * vehicles);
      session.energy_initial_kwh = 0.0;
      session.energy_target_kwh = e_req;
      session.energy_min_kwh = 0.0;
      const double p_full = full_energy_power_kw_per_vehicle(stop, options, dt);
      session.max_charge_kw =
          std::isfinite(p_full)
              ? std::max(0.0, p_full * vehicles)
              : charging_power_limit(stop, options, vehicles);
      session.max_discharge_kw = discharge_power_limit(stop, vehicles);
      session.energy_max_kwh =
          std::max(e_req, e_req + session.max_discharge_kw * dt);
      session.eta_charge = options.default_charging_efficiency;
      session.eta_discharge = options.default_charging_efficiency;
      session.v2g_capable = stop.v2g_capable;
      session.source_demand_index = demand.index;
      session.source_route_index = col.route->index;
      sessions.push_back(session);
      result.total_requested_energy_kwh += e_req;
    }
  }

  for (const auto& [j, vehicles] : positive_unserved_cols) {
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    const auto& demand = problem.demands[static_cast<std::size_t>(col.demand_pos)];
    result.unmet_demand_vehicles += vehicles;
    result.assignments.push_back({demand.index,
                                  0,
                                  demand.departure_step,
                                  vehicles,
                                  0.0,
                                  0.0,
                                  false,
                                  "system-optimal unmet demand"});
  }
}

void finalize_status(EVPowerTrafficResult& result,
                     const EVPowerTrafficOptions& options) {
  result.feasible = true;
  if (is_system_optimal_solver(options.assignment_model) &&
      !result.optimization_solved) {
    result.feasible = false;
  }
  if (!options.allow_unserved_travel_demand &&
      result.unmet_demand_vehicles > 1e-7) {
    result.feasible = false;
  }
  if (!options.allow_unserved_charging_energy &&
      result.total_unserved_energy_kwh > 1e-6) {
    result.feasible = false;
  }
  for (const auto& step : result.steps) {
    if (!step.power.generation_capacity_ok) {
      result.feasible = false;
      break;
    }
    if (step.power.power_flow_ran && !step.power.power_flow_converged) {
      result.feasible = false;
      break;
    }
  }

  if (result.feasible) {
    result.status = "ok";
  } else if (result.unmet_demand_vehicles > 1e-7) {
    result.status = "infeasible: unmet travel demand";
  } else if (result.total_unserved_energy_kwh > 1e-6) {
    result.status = "infeasible: unserved charging energy";
  } else {
    result.status = "infeasible: power validation failed";
  }
}

}  // namespace

void apply_ev_station_loads(HybridPowerSystem& system,
                            const EVPowerTrafficStepResult& step) {
  for (const auto& station_step : step.stations) {
    auto* station =
        find_by_index(system.ac.charging_stations, station_step.station_id);
    if (station == nullptr) continue;
    station->p_total_kw = station_step.p_ev_kw;
    station->q_total_kvar = station_step.q_ev_kvar;
    station->utilization_rate =
        station->max_power_kw > kTol
            ? std::abs(station_step.p_ev_kw) / station->max_power_kw
            : 0.0;
  }
}

EVPowerTrafficResult simulate_ev_power_traffic(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options) {
  EVPowerTrafficResult result;
  result.final_system = problem.system;

  if (options.num_steps <= 0 || options.time_step_hr <= 0.0) {
    result.status = "invalid options: num_steps and time_step_hr must be positive";
    return result;
  }
  if (problem.traffic.links.empty() && !problem.demands.empty()) {
    result.status = "invalid problem: traffic links are required when demands exist";
    return result;
  }

  const int T = options.num_steps;
  const double dt = options.time_step_hr;
  const auto station_ids = active_station_ids(problem);
  const auto charge_cap_kw = station_capacity_kw(problem.system, options);
  const auto dis_cap_kw = station_discharge_capacity_kw(problem.system);

  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i) {
    link_pos[problem.traffic.links[i].index] = i;
  }

  std::vector<std::vector<double>> link_used(
      static_cast<std::size_t>(T),
      std::vector<double>(problem.traffic.links.size(), 0.0));
  std::vector<std::vector<double>> link_capacity(
      static_cast<std::size_t>(T),
      std::vector<double>(problem.traffic.links.size(), 0.0));
  for (int k = 0; k < T; ++k) {
    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      link_capacity[static_cast<std::size_t>(k)][li] =
          link_capacity_vehicles(problem.traffic.links[li], k, dt);
    }
  }

  std::unordered_map<int, std::vector<double>> station_reserved_kw;
  for (int station_id : station_ids) {
    station_reserved_kw[station_id] =
        std::vector<double>(static_cast<std::size_t>(T), 0.0);
  }

  std::vector<EVChargingSession> sessions = problem.initial_sessions;
  int next_session_index = 1;
  for (const auto& session : sessions) {
    next_session_index = std::max(next_session_index, session.index + 1);
  }

  if (is_system_optimal_solver(options.assignment_model)) {
    solve_system_optimal_assignment(problem,
                                    options,
                                    T,
                                    dt,
                                    link_pos,
                                    link_capacity,
                                    charge_cap_kw,
                                    result,
                                    link_used,
                                    sessions,
                                    next_session_index);
  } else {
  for (const auto& demand : problem.demands) {
    if (demand.departure_step < 0 || demand.departure_step >= T ||
        demand.vehicles <= kTol) {
      continue;
    }
    result.total_demand_vehicles += std::max(0.0, demand.vehicles);

    const auto routes = candidate_routes(problem, demand);
    if (routes.empty()) {
      result.unmet_demand_vehicles += demand.vehicles;
      result.assignments.push_back(
          {demand.index, 0, demand.departure_step, demand.vehicles, 0.0, 0.0, false,
           "no candidate route"});
      continue;
    }

    std::vector<const RouteAlternative*> ranked = routes;
    // Filter out routes that are SOC-infeasible (eqs. evpt-route-soc-bounds)
    ranked.erase(
        std::remove_if(ranked.begin(),
                       ranked.end(),
                       [&](const RouteAlternative* r) {
                         return !route_soc_feasible(*r, demand, problem, link_pos);
                       }),
        ranked.end());
    auto cost_of = [&](const RouteAlternative* route) {
      return route_generalized_cost(
          problem,
          options,
          *route,
          demand.departure_step,
          link_pos,
          link_used[static_cast<std::size_t>(demand.departure_step)],
          link_capacity[static_cast<std::size_t>(demand.departure_step)],
          &station_reserved_kw,
          &charge_cap_kw);
    };
    auto travel_time_of = [&](const RouteAlternative* route) {
      return route_travel_time_hr(
          problem,
          *route,
          link_pos,
          link_used[static_cast<std::size_t>(demand.departure_step)],
          link_capacity[static_cast<std::size_t>(demand.departure_step)]);
    };
    std::sort(ranked.begin(),
              ranked.end(),
              [&](const RouteAlternative* a, const RouteAlternative* b) {
                return cost_of(a) < cost_of(b);
              });

    double remaining = demand.vehicles;
    if (options.assignment_model == AssignmentModel::Logit &&
        !options.enforce_road_capacity) {
      std::vector<double> weights;
      weights.reserve(ranked.size());
      double weight_sum = 0.0;
      for (const auto* route : ranked) {
        const double c = cost_of(route);
        const double w = std::isfinite(c)
                             ? std::exp(-options.logit_theta * c)
                             : 0.0;
        weights.push_back(w);
        weight_sum += w;
      }
      for (std::size_t ri = 0; ri < ranked.size(); ++ri) {
        if (weights[ri] <= kTol || weight_sum <= kTol) continue;
        const auto* route = ranked[ri];
        const double vehicles = demand.vehicles * weights[ri] / weight_sum;
        const int step = demand.departure_step;
        for (int link_index : route->link_indices) {
          if (auto pos_it = link_pos.find(link_index); pos_it != link_pos.end()) {
            link_used[static_cast<std::size_t>(step)][pos_it->second] += vehicles;
          }
        }
        const double route_cost = cost_of(route);
        const double route_travel_time = travel_time_of(route);
        result.assignments.push_back(
            {demand.index,
             route->index,
             step,
             vehicles,
             route_cost,
             route_travel_time,
             true,
             "served"});
        result.served_demand_vehicles += vehicles;
        if (std::isfinite(route_travel_time)) {
          result.total_travel_time_hr += vehicles * route_travel_time;
        }
        if (std::isfinite(route_cost)) {
          result.total_assignment_cost += vehicles * route_cost;
        }
        remaining -= vehicles;

        const int arrival_step = route_arrival_step(*route, link_pos, problem, step, dt);
        for (const auto& stop : route->charging_stops) {
          if (arrival_step >= T) continue;
          EVChargingSession session;
          session.index = next_session_index++;
          session.station_id = stop.station_id;
          session.arrival_step = arrival_step;
          session.departure_step = std::min(
              T, arrival_step + std::max(1, stop.dwell_steps));
          session.vehicle_count = vehicles;
          const double e_req =
              std::max(0.0, stop.requested_energy_kwh_per_vehicle * vehicles);
          session.energy_initial_kwh = 0.0;
          session.energy_target_kwh = e_req;
          session.energy_min_kwh = 0.0;
          session.energy_max_kwh = e_req + stop.max_discharge_kw_per_vehicle *
                                            vehicles * dt *
                                            std::max(0, session.departure_step - arrival_step);
          session.max_charge_kw = charging_power_limit(stop, options, vehicles);
          session.max_discharge_kw = discharge_power_limit(stop, vehicles);
          session.eta_charge = options.default_charging_efficiency;
          session.eta_discharge = options.default_charging_efficiency;
          session.v2g_capable = stop.v2g_capable;
          session.source_demand_index = demand.index;
          session.source_route_index = route->index;
          sessions.push_back(session);
          result.total_requested_energy_kwh += e_req;
        }
      }
    } else {
      for (const auto* route : ranked) {
        if (remaining <= kTol) break;
        const int step = demand.departure_step;
        const int arrival_step = route_arrival_step(*route, link_pos, problem, step, dt);
        const double residual =
            options.enforce_road_capacity
                ? route_residual_capacity(
                      *route,
                      link_pos,
                      link_used[static_cast<std::size_t>(step)],
                      link_capacity[static_cast<std::size_t>(step)])
                : remaining;
        double station_residual = std::numeric_limits<double>::infinity();
        if (is_capacity_aware_greedy(options.assignment_model)) {
          station_residual =
              route_station_residual_vehicles(*route,
                                              arrival_step,
                                              options,
                                              charge_cap_kw,
                                              station_reserved_kw);
        }
        const double joint_residual = std::min(residual, station_residual);
        if (joint_residual <= kTol) continue;
        const double vehicles = std::min(remaining, joint_residual);
        for (int link_index : route->link_indices) {
          if (auto pos_it = link_pos.find(link_index); pos_it != link_pos.end()) {
            link_used[static_cast<std::size_t>(step)][pos_it->second] += vehicles;
          }
        }
        reserve_station_power(*route,
                              arrival_step,
                              vehicles,
                              options,
                              station_reserved_kw);
        const double route_cost = cost_of(route);
        const double route_travel_time = travel_time_of(route);
        result.assignments.push_back(
            {demand.index,
             route->index,
             step,
             vehicles,
             route_cost,
             route_travel_time,
             true,
             is_capacity_aware_greedy(options.assignment_model)
                 ? "served-capacity-aware-greedy"
                 : "served"});
        result.served_demand_vehicles += vehicles;
        if (std::isfinite(route_travel_time)) {
          result.total_travel_time_hr += vehicles * route_travel_time;
        }
        if (std::isfinite(route_cost)) {
          result.total_assignment_cost += vehicles * route_cost;
        }
        remaining -= vehicles;

        for (const auto& stop : route->charging_stops) {
          if (arrival_step >= T) continue;
          EVChargingSession session;
          session.index = next_session_index++;
          session.station_id = stop.station_id;
          session.arrival_step = arrival_step;
          session.departure_step = std::min(
              T, arrival_step + std::max(1, stop.dwell_steps));
          session.vehicle_count = vehicles;
          const double e_req =
              std::max(0.0, stop.requested_energy_kwh_per_vehicle * vehicles);
          session.energy_initial_kwh = 0.0;
          session.energy_target_kwh = e_req;
          session.energy_min_kwh = 0.0;
          session.max_charge_kw = charging_power_limit(stop, options, vehicles);
          session.max_discharge_kw = discharge_power_limit(stop, vehicles);
          session.energy_max_kwh = std::max(e_req,
                                            e_req + session.max_discharge_kw * dt);
          session.eta_charge = options.default_charging_efficiency;
          session.eta_discharge = options.default_charging_efficiency;
          session.v2g_capable = stop.v2g_capable;
          session.source_demand_index = demand.index;
          session.source_route_index = route->index;
          sessions.push_back(session);
          result.total_requested_energy_kwh += e_req;
        }
      }
    }

    if (remaining > 1e-7) {
      result.unmet_demand_vehicles += remaining;
      result.assignments.push_back(
          {demand.index, 0, demand.departure_step, remaining, 0.0, 0.0, false,
          "insufficient route capacity"});
    }
  }
  }

  for (const auto& session : problem.initial_sessions) {
    result.total_requested_energy_kwh +=
        std::max(0.0, session.energy_target_kwh - session.energy_initial_kwh);
  }

  result.sessions = sessions;
  result.session_results.reserve(sessions.size());

  std::unordered_map<int, std::vector<double>> station_p_kw;
  std::unordered_map<int, std::vector<double>> station_q_kvar;
  std::unordered_map<int, std::vector<double>> station_reverse_kw;
  std::unordered_map<int, std::vector<double>> station_arrivals;
  std::unordered_map<int, std::vector<double>> station_requested_energy;
  for (int station_id : station_ids) {
    station_p_kw[station_id] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
    station_q_kvar[station_id] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
    station_reverse_kw[station_id] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
    station_arrivals[station_id] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
    station_requested_energy[station_id] =
        std::vector<double>(static_cast<std::size_t>(T), 0.0);
  }

  std::unordered_map<int, std::vector<double>> station_charge_used =
      station_p_kw;
  std::unordered_map<int, std::vector<double>> station_discharge_used =
      station_p_kw;

  for (const auto& session : sessions) {
    ChargingSessionResult sr;
    sr.session_index = session.index;
    sr.station_id = session.station_id;
    sr.p_charge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.p_discharge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.energy_kwh.assign(static_cast<std::size_t>(T + 1), session.energy_initial_kwh);

    if (session.arrival_step >= 0 && session.arrival_step < T) {
      station_arrivals[session.station_id][static_cast<std::size_t>(session.arrival_step)] +=
          std::max(0.0, session.vehicle_count);
      station_requested_energy[session.station_id]
                              [static_cast<std::size_t>(session.arrival_step)] +=
          std::max(0.0, session.energy_target_kwh - session.energy_initial_kwh);
    }

    double energy = session.energy_initial_kwh;
    for (int k = 0; k < T; ++k) {
      sr.energy_kwh[static_cast<std::size_t>(k)] = energy;
      if (k < session.arrival_step || k >= session.departure_step) {
        continue;
      }

      const double price = station_price(problem, options, session.station_id, k);
      const double cap_ch =
          charge_cap_kw.count(session.station_id)
              ? charge_cap_kw.at(session.station_id)
              : options.default_station_power_kw;
      const double station_ch_res =
          std::max(0.0, cap_ch - station_charge_used[session.station_id]
                                      [static_cast<std::size_t>(k)]);
      const double needed =
          std::max(0.0, session.energy_target_kwh - energy);
      double p_ch = 0.0;
      if (needed > kTol) {
        p_ch = std::min({session.max_charge_kw,
                         station_ch_res,
                         needed / (std::max(session.eta_charge, kTol) * dt)});
      }

      double p_dis = 0.0;
      const bool v2g_allowed =
          options.allow_v2g && session.v2g_capable && session.max_discharge_kw > kTol;
      if (v2g_allowed && price >= options.high_price_threshold_per_kwh) {
        const double dis_cap =
            dis_cap_kw.count(session.station_id)
                ? dis_cap_kw.at(session.station_id)
                : std::numeric_limits<double>::infinity();
        const double station_dis_res =
            std::max(0.0, dis_cap - station_discharge_used[session.station_id]
                                        [static_cast<std::size_t>(k)]);
        const double surplus =
            std::max(0.0, energy - std::max(session.energy_min_kwh,
                                            session.energy_target_kwh));
        p_dis = std::min({session.max_discharge_kw,
                          station_dis_res,
                          surplus * std::max(session.eta_discharge, kTol) / dt});
      }

      if (price <= options.low_price_threshold_per_kwh && needed > kTol) {
        p_dis = 0.0;
      }

      energy += dt * (session.eta_charge * p_ch -
                      p_dis / std::max(session.eta_discharge, kTol));
      energy = std::clamp(energy, session.energy_min_kwh, session.energy_max_kwh);

      sr.p_charge_kw[static_cast<std::size_t>(k)] = p_ch;
      sr.p_discharge_kw[static_cast<std::size_t>(k)] = p_dis;
      station_charge_used[session.station_id][static_cast<std::size_t>(k)] += p_ch;
      station_discharge_used[session.station_id][static_cast<std::size_t>(k)] += p_dis;
      station_p_kw[session.station_id][static_cast<std::size_t>(k)] += p_ch - p_dis;
      if (p_dis > p_ch) {
        station_reverse_kw[session.station_id][static_cast<std::size_t>(k)] +=
            p_dis - p_ch;
      }
      result.total_delivered_energy_kwh += dt * session.eta_charge * p_ch;
      result.total_v2g_energy_kwh += dt * p_dis / std::max(session.eta_discharge, kTol);
    }
    sr.energy_kwh[static_cast<std::size_t>(T)] = energy;
    sr.unserved_energy_kwh = std::max(0.0, session.energy_target_kwh - energy);
    result.total_unserved_energy_kwh += sr.unserved_energy_kwh;
    result.session_results.push_back(std::move(sr));
  }

  for (int station_id : station_ids) {
    const double pf = station_power_factor(problem.system, station_id);
    const double tan_phi = std::tan(std::acos(std::clamp(pf, 0.01, 1.0)));
    for (int k = 0; k < T; ++k) {
      const double p = station_p_kw[station_id][static_cast<std::size_t>(k)];
      station_q_kvar[station_id][static_cast<std::size_t>(k)] =
          p >= 0.0 ? p * tan_phi : 0.0;
    }
  }

  // ── Link conservation (eq. evpt-link-conservation) ──────────────────────
  // n_{a,k+1} = n_{a,k} + Δt * (u_{a,k} − v_{a,k})
  // Sending function: v_{a,k} = min(n_{a,k}/Δt, C̄_{a,k})
  // link_used[k][li] supplies u_{a,k} (inflow approximation at departure step).
  const std::size_t n_links = problem.traffic.links.size();
  std::vector<std::vector<double>> link_n(
      static_cast<std::size_t>(T + 1),
      std::vector<double>(n_links, 0.0));
  for (int k = 0; k < T; ++k) {
    for (std::size_t li = 0; li < n_links; ++li) {
      const double n_ak = link_n[static_cast<std::size_t>(k)][li];
      const double u_ak = link_used[static_cast<std::size_t>(k)][li];
      const double c_ak = link_capacity[static_cast<std::size_t>(k)][li];
      // Sending function S_{a,k}(n) = min(n/Δt, C̄_{a,k}) (eq. evpt-sending)
      const double v_ak = dt > kTol ? std::min(n_ak / dt, c_ak) : 0.0;
      link_n[static_cast<std::size_t>(k + 1)][li] =
          std::max(0.0, n_ak + dt * (u_ak - v_ak));
    }
  }

  // ── Station queue state (eqs. evpt-queue-dynamics, evpt-occupancy-dynamics)
  // q_{s,k+1} = q_{s,k} + λ_{s,k} − u_{s,k}
  // m_{s,k+1} = m_{s,k} + u_{s,k} − r_{s,k}
  // W_{s,k} ≈ q_{s,k} / max(μ_{s,k}, ε)  (eq. evpt-wait-approx)
  std::unordered_map<int, std::vector<double>> station_q_veh;
  std::unordered_map<int, std::vector<double>> station_m_veh;
  std::unordered_map<int, std::vector<double>> station_wait_hr;
  for (int sid : station_ids) {
    station_q_veh[sid]   = std::vector<double>(static_cast<std::size_t>(T + 1), 0.0);
    station_m_veh[sid]   = std::vector<double>(static_cast<std::size_t>(T), 0.0);
    station_wait_hr[sid] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
  }
  // m_{s,k}: aggregate plugged-in vehicles from all active sessions
  for (const auto& session : sessions) {
    auto m_it = station_m_veh.find(session.station_id);
    if (m_it == station_m_veh.end()) continue;
    const int a = session.arrival_step;
    const int d = std::min(session.departure_step, T);
    for (int k = std::max(a, 0); k < d; ++k) {
      m_it->second[static_cast<std::size_t>(k)] +=
          std::max(0.0, session.vehicle_count);
    }
  }
  // r_{s,k}: vehicles completing service at end of step k (departure_step == k+1)
  std::unordered_map<int, std::vector<double>> station_r_veh;
  for (int sid : station_ids) {
    station_r_veh[sid] = std::vector<double>(static_cast<std::size_t>(T), 0.0);
  }
  for (const auto& session : sessions) {
    auto r_it = station_r_veh.find(session.station_id);
    if (r_it == station_r_veh.end()) continue;
    const int k = session.departure_step - 1;  // completes at end of step k
    if (k >= 0 && k < T) {
      r_it->second[static_cast<std::size_t>(k)] +=
          std::max(0.0, session.vehicle_count);
    }
  }
  // Evolve queue dynamics and compute waiting times
  for (int k = 0; k < T; ++k) {
    for (int sid : station_ids) {
      const double lam  = station_arrivals[sid][static_cast<std::size_t>(k)];
      const double q_sk = station_q_veh[sid][static_cast<std::size_t>(k)];
      const double m_sk = station_m_veh[sid][static_cast<std::size_t>(k)];
      const double n_plug = n_plugs_for_station(problem.system, sid);
      // u_{s,k} = min(q_{s,k} + λ_{s,k}, max(0, N^plug_s − m_{s,k}))
      const double avail = std::isinf(n_plug)
                               ? q_sk + lam
                               : std::max(0.0, n_plug - m_sk);
      const double u_sk = std::min(q_sk + lam, avail);
      // q_{s,k+1} = max(0, q_{s,k} + λ_{s,k} − u_{s,k})
      station_q_veh[sid][static_cast<std::size_t>(k + 1)] =
          std::max(0.0, q_sk + lam - u_sk);
      // Service rate μ_{s,k} [vehicles/hr] = charge capacity / power per vehicle
      const double cap_kw = charge_cap_kw.count(sid)
                                ? charge_cap_kw.at(sid)
                                : options.default_station_power_kw;
      const double pv = std::max(kTol, options.default_route_stop_power_kw_per_vehicle);
      const double mu_sk = cap_kw / pv;
      // W_{s,k} ≈ q_{s,k} / max(μ_{s,k}, ε)
      station_wait_hr[sid][static_cast<std::size_t>(k)] =
          q_sk / std::max(mu_sk, 1e-9);
    }
  }

  const double base_load_mw = total_base_load_mw(problem.system);
  const double gen_cap_mw = total_generation_capacity_mw(problem.system, options);
  result.steps.reserve(static_cast<std::size_t>(T));
  result.system_by_step.reserve(static_cast<std::size_t>(T));
  for (int k = 0; k < T; ++k) {
    EVPowerTrafficStepResult step;
    step.step_index = k;
    step.hour = k * dt;

    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      const auto& link = problem.traffic.links[li];
      TrafficLinkStepResult lr;
      lr.link_index = link.index;
      lr.inflow_vehicles = link_used[static_cast<std::size_t>(k)][li];
      // n_{a,k} from link conservation (eq. evpt-link-conservation)
      lr.occupancy_vehicles = link_n[static_cast<std::size_t>(k)][li];
      lr.capacity_vehicles = link_capacity[static_cast<std::size_t>(k)][li];
      lr.blocked = lr.capacity_vehicles <= kTol;
      lr.travel_time_hr =
          link_travel_time_hr(link, lr.inflow_vehicles, lr.capacity_vehicles);
      step.traffic_links.push_back(lr);
    }

    double total_ev_mw = 0.0;
    for (int station_id : station_ids) {
      StationStepResult ss;
      ss.station_id = station_id;
      ss.bus = station_bus(problem.system, station_id);
      ss.arrivals_vehicles = station_arrivals[station_id][static_cast<std::size_t>(k)];
      ss.requested_energy_kwh =
          station_requested_energy[station_id][static_cast<std::size_t>(k)];
      ss.p_ev_kw = station_p_kw[station_id][static_cast<std::size_t>(k)];
      ss.q_ev_kvar = station_q_kvar[station_id][static_cast<std::size_t>(k)];
      ss.reverse_power_kw =
          station_reverse_kw[station_id][static_cast<std::size_t>(k)];
      // Queue state (eqs. evpt-queue-dynamics, evpt-occupancy-dynamics, evpt-wait-approx)
      ss.queue_vehicles  = station_q_veh[station_id][static_cast<std::size_t>(k)];
      ss.plug_occupancy  = station_m_veh[station_id][static_cast<std::size_t>(k)];
      ss.waiting_time_hr = station_wait_hr[station_id][static_cast<std::size_t>(k)];
      total_ev_mw += ss.p_ev_kw * 1e-3;
      step.stations.push_back(ss);
    }

    step.power.step_index = k;
    step.power.total_base_load_mw = base_load_mw;
    step.power.total_ev_load_mw = total_ev_mw;
    step.power.total_generation_capacity_mw = gen_cap_mw;
    step.power.capacity_margin_mw = gen_cap_mw - base_load_mw - total_ev_mw;
    step.power.generation_capacity_ok =
        !options.enforce_generation_capacity ||
        step.power.capacity_margin_mw >= -1e-7;

    HybridPowerSystem step_system = problem.system;
    if (options.update_system_charging_stations) {
      apply_ev_station_loads(step_system, step);
    }
    if (options.run_power_flow_validation) {
      step.power.power_flow_ran = true;
      const auto pf = solve_power_flow(step_system, options.pf_options);
      step.power.power_flow_converged = pf.converged;
      if (!pf.vm.empty()) {
        step.power.min_vm_pu =
            *std::min_element(pf.vm.begin(), pf.vm.end());
      }
    }

    result.system_by_step.push_back(step_system);
    result.steps.push_back(step);
  }

  if (!result.system_by_step.empty()) {
    result.final_system = result.system_by_step.back();
  }

  if (result.unmet_demand_vehicles > 1e-7) {
    std::ostringstream oss;
    oss << "unmet travel demand vehicles=" << result.unmet_demand_vehicles;
    add_warning(result, oss.str());
  }
  if (result.total_unserved_energy_kwh > 1e-6) {
    std::ostringstream oss;
    oss << "unserved charging energy kWh=" << result.total_unserved_energy_kwh;
    add_warning(result, oss.str());
  }

  finalize_status(result, options);
  return result;
}

}  // namespace hacdcpf::evpt

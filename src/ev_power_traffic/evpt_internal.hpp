// evpt_internal.hpp
// ──────────────────────────────────────────────────────────────────────────
// Private implementation header for the ev_power_traffic source files.
// NOT exported — include only from src/ev_power_traffic/*.cpp.
//
// Provides:
//   • shared inline helper functions (route cost, feasibility, capacity)
//   • power-system utility helpers (station capacity, load totals, …)
//   • SystemOptimalColumn / SystemOptimalSolve (LP/MILP data structures)
//   • forward declaration of solve_system_optimal_assignment()
//     (defined in assignment_lp.cpp)
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "hacdcpf/ev_power_traffic/joint_social_welfare.hpp"

namespace hacdcpf::evpt {

// ── Tolerance constant ─────────────────────────────────────────────────────

inline constexpr double kTol = 1e-9;

// ── Assignment-model predicates ────────────────────────────────────────────

inline bool is_capacity_aware_greedy(AssignmentModel model) {
  return model == AssignmentModel::CapacityAwareGreedy;
}

inline bool is_system_optimal_solver(AssignmentModel model) {
  return model == AssignmentModel::SystemOptimalLP ||
         model == AssignmentModel::SystemOptimalMILP;
}

inline bool is_system_optimal_mip(AssignmentModel model) {
  return model == AssignmentModel::SystemOptimalMILP;
}

// ── Generic helpers ────────────────────────────────────────────────────────

template <typename T>
inline const T* find_by_index(const std::vector<T>& values, int index) {
  for (const auto& v : values) {
    if (v.index == index) return &v;
  }
  return nullptr;
}

template <typename T>
inline T* find_by_index(std::vector<T>& values, int index) {
  for (auto& v : values) {
    if (v.index == index) return &v;
  }
  return nullptr;
}

inline double profile_value(const std::vector<double>& values,
                            int step,
                            double fallback) {
  if (step >= 0 && step < static_cast<int>(values.size())) {
    return values[static_cast<std::size_t>(step)];
  }
  return fallback;
}

inline bool availability_value(const std::vector<bool>& values,
                               int step,
                               bool fallback) {
  if (step >= 0 && step < static_cast<int>(values.size())) {
    return values[static_cast<std::size_t>(step)];
  }
  return fallback;
}

// ── Traffic link helpers ───────────────────────────────────────────────────

inline double link_capacity_vehicles(const TrafficLink& link,
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

inline double link_travel_time_hr(const TrafficLink& link,
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

// ── Station price helper ───────────────────────────────────────────────────

inline double station_price(const EVPowerTrafficProblem& problem,
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

// ── Route cost helpers ─────────────────────────────────────────────────────

// Forward-declare route_arrival_step so route_generalized_cost can use it.
inline int route_arrival_step(const RouteAlternative& route,
                              const std::unordered_map<int, std::size_t>& link_pos,
                              const EVPowerTrafficProblem& problem,
                              int departure_step,
                              double dt_hr);

inline double route_generalized_cost(
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
  // W_{s,k} ≈ (q_{s,k} / μ_s) · Δt  [hours]  (eq. evpt-wait)
  // μ_s = N^plug_s / Δ_s ≈ P^max_s / (p_veh · Δ_s)
  // Here we use a reservation-based proxy for q and Δ_s = dwell_steps (or 1).
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
      const double cap_kw =
          (charge_cap_kw_ptr && charge_cap_kw_ptr->count(stop.station_id))
              ? charge_cap_kw_ptr->at(stop.station_id)
              : options.default_station_power_kw;
      // Δ_s: dwell duration in steps for this stop (≥ 1)
      const double delta_s = static_cast<double>(std::max(1, stop.dwell_steps));
      // μ_s [veh/step] = N^plug / Δ_s ≈ P^max / (p_veh · Δ_s)
      const double mu = cap_kw / std::max(p_per_veh * delta_s, kTol);
      // Queue estimate: vehicles waiting above current plug capacity
      const double queue_est = std::max(
          0.0,
          reserved_veh - (std::isinf(mu) ? reserved_veh : mu));
      // W_{s,k} ≈ (queue_est / μ) · Δt  [hours]
      queueing_cost += options.queueing_weight *
                       (queue_est / std::max(mu, 1e-9)) *
                       options.time_step_hr;
    }
  }

  return options.value_of_time_per_hr * travel_hr +
         options.station_energy_cost_weight * charging_cost +
         queueing_cost +
         route.toll_cost;
}

inline double route_free_flow_generalized_cost(
    const EVPowerTrafficProblem& problem,
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

inline double route_travel_time_hr(
    const EVPowerTrafficProblem& problem,
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

inline double route_residual_capacity(
    const RouteAlternative& route,
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

inline double route_station_residual_vehicles(
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

inline void reserve_station_power(
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

inline std::vector<const RouteAlternative*> candidate_routes(
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

inline int route_arrival_step(const RouteAlternative& route,
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

inline double charging_power_limit(const RouteChargingStop& stop,
                                   const EVPowerTrafficOptions& options,
                                   double vehicles) {
  const double per_vehicle =
      stop.max_charge_kw_per_vehicle > 0.0
          ? stop.max_charge_kw_per_vehicle
          : options.default_route_stop_power_kw_per_vehicle;
  return std::max(0.0, per_vehicle * vehicles);
}

inline double discharge_power_limit(const RouteChargingStop& stop,
                                    double vehicles) {
  return std::max(0.0, stop.max_discharge_kw_per_vehicle * vehicles);
}

inline double full_energy_power_kw_per_vehicle(const RouteChargingStop& stop,
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

inline bool route_can_deliver_requested_energy(const RouteAlternative& route,
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
inline bool route_soc_feasible(const RouteAlternative& route,
                               const EVDemand& demand,
                               const EVPowerTrafficProblem& problem,
                               const std::unordered_map<int, std::size_t>& link_pos) {
  if (demand.initial_energy_kwh < 0.0) return true;

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

  for (int link_idx : route.link_indices) {
    auto it = link_pos.find(link_idx);
    if (it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[it->second];
    E -= link.drive_energy_kwh_per_veh_km * link.length_km;
    if (E < E_min - kTol) return false;
  }

  for (const auto& stop : route.charging_stops) {
    E += stop.requested_energy_kwh_per_vehicle;
    E = std::min(E, E_max);
  }

  return E >= E_min - kTol;
}

// ── Warning helper ─────────────────────────────────────────────────────────

inline void add_warning(EVPowerTrafficResult& result, const std::string& warning) {
  result.warnings.push_back(warning);
}

// ── Power system helpers ───────────────────────────────────────────────────

inline std::unordered_map<int, double> station_capacity_kw(
    const HybridPowerSystem& sys,
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

inline std::unordered_map<int, double> station_discharge_capacity_kw(
    const HybridPowerSystem& sys) {
  std::unordered_map<int, double> cap;
  for (const auto& charger : sys.ac.chargers) {
    if (!charger.in_service || !charger.v2g_capable) continue;
    cap[charger.station_id] += std::max(0.0, charger.p_dis_max_kw);
  }
  return cap;
}

inline double total_base_load_mw(const HybridPowerSystem& sys) {
  double p = 0.0;
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service) p += std::max(0.0, bus.pd_mw);
  }
  for (const auto& load : sys.ac.loads) {
    if (load.in_service) p += std::max(0.0, load.p_mw * load.scaling);
  }
  return p;
}

inline double total_generation_capacity_mw(const HybridPowerSystem& sys,
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

inline double station_power_factor(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    return std::clamp(st->power_factor, 0.01, 1.0);
  }
  return 0.95;
}

inline int station_bus(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    return st->bus;
  }
  return 0;
}

// Returns N^plug_s: total usable plugs for station s (eq. evpt-queue-dynamics).
inline double n_plugs_for_station(const HybridPowerSystem& sys, int station_id) {
  if (const auto* st = find_by_index(sys.ac.charging_stations, station_id)) {
    const int n = st->n_fast + st->n_slow + st->num_chargers;
    if (n > 0) return static_cast<double>(n);
  }
  return std::numeric_limits<double>::infinity();
}

inline std::vector<int> active_station_ids(const EVPowerTrafficProblem& problem) {
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

// ── LP/MILP assignment data structures ────────────────────────────────────

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

inline double inferred_minimization_bound(double objective, double mip_gap) {
  if (!std::isfinite(objective) || !std::isfinite(mip_gap)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return objective - std::max(0.0, mip_gap) * std::max(1.0, std::abs(objective));
}

// ── Forward declaration ────────────────────────────────────────────────────
// Defined in assignment_lp.cpp.

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
    int& next_session_index);

}  // namespace hacdcpf::evpt

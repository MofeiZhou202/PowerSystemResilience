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
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/ev_power_traffic/joint_social_welfare.hpp"
#include "hacdcpf/ev_power_traffic/ltm_network.hpp"

namespace hacdcpf::evpt {

// ── Tolerance constant ─────────────────────────────────────────────────────

inline constexpr double kTol = 1e-9;

// ── CTM link helpers (shared with ctm_so_lp, ctm_due_vi) ──────────────────

/// Derive free-flow speed [km/hr] from free_flow_time_hr and length_km.
inline double link_free_flow_speed(const TrafficLink& link) noexcept {
  if (link.length_km > kTol && link.free_flow_time_hr > kTol)
    return link.length_km / link.free_flow_time_hr;
  return 1.0e6;
}

/// Choose CTM sub-step Δt_ctm satisfying the CFL stability condition.
inline double choose_ctm_dt(const EVPowerTrafficProblem& problem,
                             const CTMOptions& ctm_opts,
                             double dt_sim) {
  if (ctm_opts.dt_ctm_hr > kTol) return ctm_opts.dt_ctm_hr;
  double min_cfl = dt_sim;
  for (const auto& lnk : problem.traffic.links) {
    const double vf = link_free_flow_speed(lnk);
    if (vf < kTol) continue;
    const double delta = lnk.length_km > kTol
        ? lnk.length_km / static_cast<double>(ctm_opts.n_cells_per_link > 0
              ? ctm_opts.n_cells_per_link
              : std::max(1, static_cast<int>(std::floor(lnk.length_km / (vf * dt_sim)))))
        : vf * dt_sim;
    min_cfl = std::min(min_cfl, delta / vf);
  }
  return std::max(kTol, 0.9 * min_cfl);
}

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

// ── Schedule delay cost (eq:icv-schedule-delay) ───────────────────────────
//
// C^{schedule,F}_{w,r,k}(h) = γ_e · max(0, t*_w − t^arr) + γ_l · max(0, t^arr − t*_w)
//
// Parameters:
//   t_arr_hr      — experienced arrival time [hr]:  t^arr = k·Δt + T^exp_{r,k}
//   t_star_hr     — desired arrival time t*_w [hr]; negative = not set (returns 0)
//   gamma_early   — γ_e [$/hr] — penalty per unit of early arrival
//   gamma_late    — γ_l [$/hr] — penalty per unit of late arrival
// Returns: schedule delay cost [$/vehicle].
inline double schedule_delay_cost(double t_arr_hr,
                                  double t_star_hr,
                                  double gamma_early,
                                  double gamma_late) noexcept {
  if (t_star_hr < 0.0) return 0.0;
  return gamma_early * std::max(0.0, t_star_hr - t_arr_hr) +
         gamma_late  * std::max(0.0, t_arr_hr  - t_star_hr);
}

// ── Route cost helpers ─────────────────────────────────────────────────────

// Forward-declare route_arrival_step so route_generalized_cost can use it.
inline int route_arrival_step(const RouteAlternative& route,
                              const std::unordered_map<int, std::size_t>& link_pos,
                              const EVPowerTrafficProblem& problem,
                              int departure_step,
                              double dt_hr);

inline double route_free_flow_time(
    const RouteAlternative& route,
    const std::unordered_map<int, std::size_t>& link_pos,
    const EVPowerTrafficProblem& problem) noexcept;

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
    // V2G revenue: subtract π^dis · E^dis (eq:ev-energy-cost).
    // C^energy = Σ(π^ch · p^ch − π^dis · p^dis) · Δt
    // Uses station charging price as π^dis; callers may set separate
    // discharge prices via station_discharge_prices in the future.
    if (stop.v2g_capable &&
        stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
      charging_cost -= station_price(problem, options, stop.station_id, step) *
                       stop.requested_discharge_energy_kwh_per_vehicle;
    }
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
    // V2G revenue: subtract π^dis · E^dis (eq:ev-energy-cost).
    if (stop.v2g_capable &&
        stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
      charging_cost -= station_price(problem, options, stop.station_id, step) *
                       stop.requested_discharge_energy_kwh_per_vehicle;
    }
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

// Battery SOC feasibility check (eqs. evpt-route-soc, evpt-route-soc-bounds,
// eq:soc-feasibility).
// Two constraints are checked at every link traversal:
//   1. Battery floor:    e_{v,k} ≥ E_min (energy_min_kwh)
//   2. Mobility reserve: e_{v,k} − E^rem_{v,k} ≥ e^res_v (reserve_energy_kwh)
// Constraint 2 uses the cumulative remaining drive energy from the current
// position to the destination.  It is active only when demand.reserve_energy_kwh > 0.
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

  const double E_min    = demand.energy_min_kwh;
  const double E_max    = demand.energy_max_kwh >= 0.0
                              ? demand.energy_max_kwh
                              : std::numeric_limits<double>::infinity();
  const double e_res    = demand.reserve_energy_kwh;  // e^res_v
  double E = demand.initial_energy_kwh;
  if (E < E_min - kTol || E > E_max + kTol) return false;

  // Pre-compute remaining drive energy from each position (right-to-left sum).
  // e_rem[i] = total drive energy from link i through the end of the route.
  const int n_links = static_cast<int>(route.link_indices.size());
  std::vector<double> e_rem(static_cast<std::size_t>(n_links + 1), 0.0);
  for (int i = n_links - 1; i >= 0; --i) {
    auto it = link_pos.find(route.link_indices[static_cast<std::size_t>(i)]);
    double seg = 0.0;
    if (it != link_pos.end()) {
      const auto& lnk = problem.traffic.links[it->second];
      seg = lnk.drive_energy_kwh_per_veh_km * lnk.length_km;
    }
    e_rem[static_cast<std::size_t>(i)] =
        e_rem[static_cast<std::size_t>(i + 1)] + seg;
  }

  for (int i = 0; i < n_links; ++i) {
    auto it = link_pos.find(route.link_indices[static_cast<std::size_t>(i)]);
    if (it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[it->second];
    E -= link.drive_energy_kwh_per_veh_km * link.length_km;
    // Constraint 1: battery floor
    if (E < E_min - kTol) return false;
    // Constraint 2: mobility reserve — e_{v,k} − E^rem_{v,k} ≥ e^res_v
    // E^rem[i+1] = drive energy still needed after traversing link i
    if (e_res > kTol &&
        E - e_rem[static_cast<std::size_t>(i + 1)] < e_res - kTol) {
      return false;
    }
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

struct SolverCertificateResiduals {
  double primal_max_violation{0.0};
  double integrality_max_violation{0.0};
  int primal_violation_block{0};  // 1 bounds, 2 equality, 3 inequality
  int primal_violation_index{-1};
};

struct DUEVICertificate {
  bool available{false};
  double vi_gap{0.0};
  double normalized_vi_gap{0.0};
  double complementarity_max_violation{0.0};
  double demand_conservation_max_violation{0.0};
};

inline double finite_bound_violation(double value, double lower, double upper) {
  double v = 0.0;
  if (std::isfinite(lower)) v = std::max(v, lower - value);
  if (std::isfinite(upper)) v = std::max(v, value - upper);
  return std::max(0.0, v);
}

inline SolverCertificateResiduals compute_solver_certificate_residuals(
    const engine::MIPModel& mip,
    const Eigen::VectorXd& x) {
  SolverCertificateResiduals cert;
  const engine::LPModel& lp = mip.linear_part;
  if (x.size() != static_cast<Eigen::Index>(lp.vars.size())) {
    cert.primal_max_violation = std::numeric_limits<double>::infinity();
    cert.integrality_max_violation = std::numeric_limits<double>::infinity();
    return cert;
  }

  for (int j = 0; j < static_cast<int>(lp.vars.size()); ++j) {
    const double xj = x[j];
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    const double v = finite_bound_violation(xj, var.lb, var.ub);
    if (v > cert.primal_max_violation) {
      cert.primal_max_violation = v;
      cert.primal_violation_block = 1;
      cert.primal_violation_index = j;
    }
    if (var.type == engine::VarType::Integer ||
        var.type == engine::VarType::Binary) {
      cert.integrality_max_violation = std::max(
          cert.integrality_max_violation,
          std::abs(xj - std::round(xj)));
    }
  }

  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd r = lp.Aeq * x - lp.beq;
    for (int i = 0; i < r.size(); ++i) {
      const double v = std::abs(r[i]);
      if (v > cert.primal_max_violation) {
        cert.primal_max_violation = v;
        cert.primal_violation_block = 2;
        cert.primal_violation_index = i;
      }
    }
  }
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd ax = lp.A * x;
    for (int i = 0; i < ax.size(); ++i) {
      const double lhs = engine::lp_row_lhs_or_neg_inf(lp, i);
      const double rhs = (i < lp.b.size()) ? lp.b[i]
                                           : std::numeric_limits<double>::infinity();
      const double v = finite_bound_violation(ax[i], lhs, rhs);
      if (v > cert.primal_max_violation) {
        cert.primal_max_violation = v;
        cert.primal_violation_block = 3;
        cert.primal_violation_index = i;
      }
    }
  }
  return cert;
}

inline std::unordered_map<int, std::vector<double>>
forecast_station_arrivals_from_demands(const EVPowerTrafficProblem& problem,
                                       const EVPowerTrafficOptions& opts) {
  std::unordered_map<int, std::vector<double>> forecast;
  for (int sid : active_station_ids(problem)) {
    forecast[sid].assign(static_cast<std::size_t>(std::max(0, opts.num_steps)), 0.0);
  }

  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i) {
    link_pos[problem.traffic.links[i].index] = i;
  }

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    const auto routes = candidate_routes(problem, demand);
    if (routes.empty()) continue;

    double inv_cost_sum = 0.0;
    std::vector<double> weights;
    weights.reserve(routes.size());
    for (const auto* route : routes) {
      if (!route || !route_soc_feasible(*route, demand, problem, link_pos)) {
        weights.push_back(0.0);
        continue;
      }
      const double tt = route_free_flow_time(*route, link_pos, problem);
      const double w = 1.0 / std::max(kTol, tt);
      weights.push_back(w);
      inv_cost_sum += w;
    }
    if (inv_cost_sum <= kTol) continue;

    for (std::size_t ri = 0; ri < routes.size(); ++ri) {
      const auto* route = routes[ri];
      if (!route || weights[ri] <= kTol) continue;
      const double flow = demand.vehicles * weights[ri] / inv_cost_sum;
      const int arrival_step =
          route_arrival_step(*route, link_pos, problem,
                             demand.departure_step, opts.time_step_hr);
      for (const auto& stop : route->charging_stops) {
        auto it = forecast.find(stop.station_id);
        if (it == forecast.end()) {
          it = forecast.emplace(stop.station_id,
                                std::vector<double>(
                                    static_cast<std::size_t>(
                                        std::max(0, opts.num_steps)), 0.0)).first;
        }
        if (arrival_step >= 0 &&
            arrival_step < static_cast<int>(it->second.size())) {
          it->second[static_cast<std::size_t>(arrival_step)] += flow;
        }
      }
    }
  }
  return forecast;
}

inline DUEVICertificate compute_due_vi_certificate(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& opts,
    const CTMSimulationResult& ctm,
    const std::unordered_map<int, int>& route_pos_map,
    const std::vector<std::vector<double>>& flows,
    int R) {
  DUEVICertificate cert;
  if (R <= 0 || flows.empty()) return cert;
  cert.available = true;

  auto dep_steps_for = [](const EVDemand& demand) {
    if (!demand.departure_window_steps.empty()) {
      return demand.departure_window_steps;
    }
    return std::vector<int>{demand.departure_step};
  };

  const int T_ctm = static_cast<int>(flows.size());
  const double INF = std::numeric_limits<double>::infinity();

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    double demand_flow = 0.0;
    double min_cost = INF;
    double used_cost_weighted = 0.0;
    double used_flow = 0.0;
    std::vector<std::pair<double, double>> used_pairs;  // flow, cost

    for (int dep : dep_steps_for(demand)) {
      const int k_ctm = dep * R;
      if (k_ctm < 0 || k_ctm >= T_ctm) continue;
      for (const auto* route : candidate_routes(problem, demand)) {
        if (!route || !route_pos_map.count(route->index)) continue;
        const int rp = route_pos_map.at(route->index);
        if (rp < 0 || rp >= static_cast<int>(flows[static_cast<std::size_t>(k_ctm)].size())) {
          continue;
        }
        double tt = INF;
        auto tt_it = ctm.route_travel_time.find(route->index);
        if (tt_it != ctm.route_travel_time.end() &&
            k_ctm < static_cast<int>(tt_it->second.size())) {
          tt = tt_it->second[static_cast<std::size_t>(k_ctm)];
        }
        if (!std::isfinite(tt)) continue;
        const double arr_hr = dep * opts.time_step_hr + tt;
        double cost = opts.value_of_time_per_hr * tt + route->toll_cost +
            schedule_delay_cost(arr_hr, demand.desired_arrival_time_hr,
                                demand.early_penalty_per_hr,
                                demand.late_penalty_per_hr);
        for (const auto& stop : route->charging_stops) {
          const double price = station_price(problem, opts, stop.station_id, dep);
          cost += opts.station_energy_cost_weight * price *
                  stop.requested_energy_kwh_per_vehicle;
          if (stop.v2g_capable &&
              stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
            cost -= opts.station_energy_cost_weight * price *
                    stop.requested_discharge_energy_kwh_per_vehicle;
          }
        }
        min_cost = std::min(min_cost, cost);
        const double f = flows[static_cast<std::size_t>(k_ctm)]
                              [static_cast<std::size_t>(rp)];
        if (f > kTol) {
          demand_flow += f;
          used_flow += f;
          used_cost_weighted += f * cost;
          used_pairs.push_back({f, cost});
        }
      }
    }

    cert.demand_conservation_max_violation = std::max(
        cert.demand_conservation_max_violation,
        std::abs(demand_flow - demand.vehicles));
    if (!std::isfinite(min_cost) || used_flow <= kTol) continue;

    cert.vi_gap += std::max(0.0, used_cost_weighted -
                                     demand.vehicles * min_cost);
    for (const auto& [f, cost] : used_pairs) {
      (void)f;
      cert.complementarity_max_violation = std::max(
          cert.complementarity_max_violation,
          std::max(0.0, cost - min_cost));
    }
  }

  double current_cost = 0.0;
  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    for (int dep : dep_steps_for(demand)) {
      const int k_ctm = dep * R;
      if (k_ctm < 0 || k_ctm >= T_ctm) continue;
      for (const auto* route : candidate_routes(problem, demand)) {
        if (!route || !route_pos_map.count(route->index)) continue;
        const int rp = route_pos_map.at(route->index);
        if (rp < 0 || rp >= static_cast<int>(flows[static_cast<std::size_t>(k_ctm)].size())) {
          continue;
        }
        const double f = flows[static_cast<std::size_t>(k_ctm)]
                              [static_cast<std::size_t>(rp)];
        if (f <= kTol) continue;
        auto tt_it = ctm.route_travel_time.find(route->index);
        if (tt_it == ctm.route_travel_time.end() ||
            k_ctm >= static_cast<int>(tt_it->second.size())) {
          continue;
        }
        const double tt = tt_it->second[static_cast<std::size_t>(k_ctm)];
        current_cost += f * opts.value_of_time_per_hr * tt;
      }
    }
  }
  cert.normalized_vi_gap =
      cert.vi_gap / std::max(1.0, std::abs(current_cost));
  return cert;
}

inline EVPowerTrafficProblem maybe_generate_candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& opts);

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

// ── K-shortest loopless paths (Yen 1971) ──────────────────────────────────
//
// Returns at most K loopless paths (by free_flow_time_hr, shortest-first)
// from origin to destination in the traffic graph.
// Each path stores the ordered list of link indices.
// Links with available==false are impassable.
// May return fewer than K paths if the graph has insufficient loopless paths.

struct YenPath {
  double cost{0.0};
  std::vector<int> nodes;  ///< ordered node indices (size = links + 1)
  std::vector<int> links;  ///< ordered link indices (size = nodes - 1)
  bool operator>(const YenPath& o) const noexcept { return cost > o.cost; }
  bool operator<(const YenPath& o) const noexcept { return cost < o.cost; }
};

inline std::vector<YenPath> yen_k_shortest_paths(
    const TrafficGraph& graph,
    int origin,
    int destination,
    int K)
{
  if (K <= 0) return {};

  const int N = static_cast<int>(graph.nodes.size());
  if (N == 0) return {};

  // Map node *index* → position in graph.nodes
  std::unordered_map<int, int> node_idx_to_pos;
  node_idx_to_pos.reserve(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i)
    node_idx_to_pos[graph.nodes[static_cast<std::size_t>(i)].index] = i;

  auto pos_of = [&](int node_idx) -> int {
    auto it = node_idx_to_pos.find(node_idx);
    return (it != node_idx_to_pos.end()) ? it->second : -1;
  };

  const int src = pos_of(origin);
  const int snk = pos_of(destination);
  if (src < 0 || snk < 0) return {};
  if (src == snk) {
    YenPath p; p.nodes.push_back(origin); return {p};
  }

  // Adjacency list: pos -> {traffic link id, to_pos, cost}
  struct Edge { int link_idx; int to_pos; double cost; };
  std::vector<std::vector<Edge>> adj(static_cast<std::size_t>(N));
  for (int li = 0; li < static_cast<int>(graph.links.size()); ++li) {
    const auto& lnk = graph.links[static_cast<std::size_t>(li)];
    if (!lnk.available) continue;
    const int fp = pos_of(lnk.from_node);
    const int tp = pos_of(lnk.to_node);
    if (fp < 0 || tp < 0) continue;
    adj[static_cast<std::size_t>(fp)].push_back(
        {lnk.index, tp, std::max(0.0, lnk.free_flow_time_hr)});
  }

  // Dijkstra from start_pos to snk, excluding excl_links and excl_node_pos.
  // Returns a YenPath with positions translated back to node indices.
  auto dijkstra = [&](int start_pos,
                      const std::unordered_set<int>& excl_links,
                      const std::unordered_set<int>& excl_nodes_pos) -> YenPath {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(static_cast<std::size_t>(N), INF);
    std::vector<int>    prev_pos(static_cast<std::size_t>(N), -1);
    std::vector<int>    prev_link(static_cast<std::size_t>(N), -1);
    dist[static_cast<std::size_t>(start_pos)] = 0.0;

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, start_pos});

    while (!pq.empty()) {
      auto [d, u] = pq.top(); pq.pop();
      if (d > dist[static_cast<std::size_t>(u)] + kTol) continue;
      if (u == snk) break;
      for (const auto& e : adj[static_cast<std::size_t>(u)]) {
        if (excl_links.count(e.link_idx)) continue;
        if (excl_nodes_pos.count(e.to_pos)) continue;
        const double nd = d + e.cost;
        if (nd < dist[static_cast<std::size_t>(e.to_pos)] - kTol) {
          dist[static_cast<std::size_t>(e.to_pos)] = nd;
          prev_pos[static_cast<std::size_t>(e.to_pos)] = u;
          prev_link[static_cast<std::size_t>(e.to_pos)] = e.link_idx;
          pq.push({nd, e.to_pos});
        }
      }
    }

    YenPath p;
    if (!std::isfinite(dist[static_cast<std::size_t>(snk)])) return p;
    p.cost = dist[static_cast<std::size_t>(snk)];
    int cur = snk;
    while (cur != start_pos) {
      p.nodes.push_back(graph.nodes[static_cast<std::size_t>(cur)].index);
      p.links.push_back(prev_link[static_cast<std::size_t>(cur)]);
      cur = prev_pos[static_cast<std::size_t>(cur)];
    }
    p.nodes.push_back(graph.nodes[static_cast<std::size_t>(start_pos)].index);
    std::reverse(p.nodes.begin(), p.nodes.end());
    std::reverse(p.links.begin(), p.links.end());
    return p;
  };

  std::vector<YenPath> A;  // confirmed shortest paths
  // B: candidate min-heap
  std::priority_queue<YenPath, std::vector<YenPath>, std::greater<YenPath>> B;

  // First shortest path
  {
    YenPath p1 = dijkstra(src, {}, {});
    if (p1.nodes.empty()) return {};
    A.push_back(std::move(p1));
  }

  for (int k = 1; k < K; ++k) {
    const YenPath& prev = A.back();
    const int spur_count = static_cast<int>(prev.nodes.size()) - 1;

    for (int i = 0; i < spur_count; ++i) {
      const int spur_node_idx = prev.nodes[static_cast<std::size_t>(i)];
      const int spur_pos      = pos_of(spur_node_idx);
      if (spur_pos < 0) continue;

      // Remove root-prefix links that lead to duplicate paths already in A
      std::unordered_set<int> excl_links;
      for (const auto& ap : A) {
        if (static_cast<int>(ap.nodes.size()) <= i) continue;
        bool same = true;
        for (int j = 0; j <= i; ++j) {
          if (ap.nodes[static_cast<std::size_t>(j)] !=
              prev.nodes[static_cast<std::size_t>(j)]) { same = false; break; }
        }
        if (same && i < static_cast<int>(ap.links.size()))
          excl_links.insert(ap.links[static_cast<std::size_t>(i)]);
      }

      // Remove all root nodes except spur (prevents loops)
      std::unordered_set<int> excl_nodes_pos;
      for (int j = 0; j < i; ++j) {
        const int np = pos_of(prev.nodes[static_cast<std::size_t>(j)]);
        if (np >= 0) excl_nodes_pos.insert(np);
      }

      YenPath spur = dijkstra(spur_pos, excl_links, excl_nodes_pos);
      if (spur.nodes.empty()) continue;

      // Combine root (nodes/links 0..i-1) + spur
      YenPath cand;
      cand.cost = 0.0;
      for (int j = 0; j < i; ++j) {
        cand.nodes.push_back(prev.nodes[static_cast<std::size_t>(j)]);
        int li = prev.links[static_cast<std::size_t>(j)];
        cand.links.push_back(li);
        for (const auto& lnk : graph.links) {
          if (lnk.index == li) {
            cand.cost += std::max(0.0, lnk.free_flow_time_hr);
            break;
          }
        }
      }
      for (int nd : spur.nodes) cand.nodes.push_back(nd);
      for (int li : spur.links) cand.links.push_back(li);
      cand.cost += spur.cost;

      // Avoid duplicates already in A
      bool dup = false;
      for (const auto& ap : A) if (ap.links == cand.links) { dup = true; break; }
      if (!dup) B.push(cand);
    }

    if (B.empty()) break;

    // Pop the best candidate, skipping any that duplicate A
    while (!B.empty()) {
      YenPath best = B.top(); B.pop();
      bool dup = false;
      for (const auto& ap : A) if (ap.links == best.links) { dup = true; break; }
      if (!dup) { A.push_back(std::move(best)); break; }
    }
    if (static_cast<int>(A.size()) == k) {
      // Nothing new was added; keep trying remaining candidates
    }
  }
  return A;
}

// ── Automatic candidate route generation ──────────────────────────────────
//
// Generates the set Ω_{w,k} (eq:plan-set) for all EV demands without
// pre-assigned candidate_route_indices by:
//   1. Running Yen's K-shortest-paths for each OD pair.
//   2. For each path: (a) no-stop, (b) single-station, (c) two-station routes.
//   3. Pruning infeasible routes by SOC feasibility.
//
// Returns a copy of `problem` with new routes appended and each demand's
// candidate_route_indices populated.
struct GenerateRoutesOptions {
  int K{3};
  bool add_no_stop_route{true};
  bool add_single_station_routes{true};
  bool add_two_station_routes{false};   ///< combinatorial — enable only for small nets
  bool prune_by_soc{true};
  double default_charge_energy_kwh{20.0};
  int default_dwell_steps{2};
  double default_max_charge_kw{50.0};
};

inline EVPowerTrafficProblem generate_candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& opts,
    const GenerateRoutesOptions& gen = {})
{
  EVPowerTrafficProblem out = problem;
  const int K = std::max(1, gen.K);

  // link_pos map for SOC feasibility
  std::unordered_map<int, std::size_t> lp;
  for (std::size_t i = 0; i < out.traffic.links.size(); ++i)
    lp[out.traffic.links[i].index] = i;

  // station → bus and node → stations
  std::unordered_map<int, std::vector<int>> node_to_stn;
  for (const auto& st : out.system.ac.charging_stations)
    node_to_stn[st.bus].push_back(st.index);

  // next free route index
  int nri = 0;
  for (const auto& r : out.routes) nri = std::max(nri, r.index + 1);

  auto make_stop = [&](int sid) {
    RouteChargingStop s;
    s.station_id = sid;
    s.requested_energy_kwh_per_vehicle = gen.default_charge_energy_kwh;
    s.dwell_steps = gen.default_dwell_steps;
    s.max_charge_kw_per_vehicle = gen.default_max_charge_kw;
    return s;
  };

  auto try_add = [&](RouteAlternative& r,
                     const EVDemand& demand,
                     std::vector<int>& indices) {
    if (!gen.prune_by_soc || route_soc_feasible(r, demand, out, lp)) {
      out.routes.push_back(r);
      indices.push_back(r.index);
    }
  };

  for (auto& demand : out.demands) {
    if (!demand.candidate_route_indices.empty()) continue;

    std::vector<YenPath> paths =
        yen_k_shortest_paths(out.traffic, demand.origin_node, demand.destination_node, K);
    if (paths.empty()) continue;

    std::vector<int> new_idx;
    for (const auto& path : paths) {
      if (path.links.empty() && demand.origin_node != demand.destination_node) continue;

      if (gen.add_no_stop_route) {
        RouteAlternative r;
        r.index = nri++; r.origin_node = demand.origin_node;
        r.destination_node = demand.destination_node; r.link_indices = path.links;
        try_add(r, demand, new_idx);
      }

      // Gather stations reachable from nodes on this path
      std::vector<int> path_stns;
      {
        std::unordered_set<int> seen;
        for (int nd : path.nodes) {
          auto it = node_to_stn.find(nd);
          if (it == node_to_stn.end()) continue;
          for (int sid : it->second)
            if (seen.insert(sid).second) path_stns.push_back(sid);
        }
      }

      if (gen.add_single_station_routes) {
        for (int sid : path_stns) {
          RouteAlternative r;
          r.index = nri++; r.origin_node = demand.origin_node;
          r.destination_node = demand.destination_node; r.link_indices = path.links;
          r.charging_stops = {make_stop(sid)};
          try_add(r, demand, new_idx);
        }
      }

      if (gen.add_two_station_routes && path_stns.size() >= 2) {
        for (std::size_t a = 0; a < path_stns.size(); ++a) {
          for (std::size_t b = a + 1; b < path_stns.size(); ++b) {
            RouteAlternative r;
            r.index = nri++; r.origin_node = demand.origin_node;
            r.destination_node = demand.destination_node; r.link_indices = path.links;
            r.charging_stops = {make_stop(path_stns[a]), make_stop(path_stns[b])};
            try_add(r, demand, new_idx);
          }
        }
      }
    }
    demand.candidate_route_indices = std::move(new_idx);
  }
  return out;
}

inline EVPowerTrafficProblem maybe_generate_candidate_routes(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& opts) {
  if (!opts.auto_generate_routes) return problem;
  GenerateRoutesOptions gen;
  gen.K = opts.k_shortest_paths;
  return generate_candidate_routes(problem, opts, gen);
}

// ── Station spillback helper (eq:spillback-receiving) ─────────────────────
//
// Returns the reduced receiving capacity [veh per step] for the access link
// of charging station `sp` at simulation step k, given the current queue
// state Q_{s,k}.
//
// Reduction model (smooth sigmoid):
//   R^access_{s,k} = R^0 × max(0, 1 − Q_{s,k} / Q̄_s)
//
// This returns a multiplicative factor ∈ [0, 1].  The caller multiplies
// this by the nominal receiving capacity computed from the CTM or LTM formula.
// Returns 1.0 if queue_capacity == inf (no spillback) or access_link_id < 0.
inline double spillback_receiving_factor(const ChargingStationParams& sp,
                                         double queue_veh) noexcept {
  if (sp.access_link_id < 0) return 1.0;
  if (!std::isfinite(sp.queue_capacity) || sp.queue_capacity <= kTol) return 1.0;
  return std::max(0.0, 1.0 - queue_veh / sp.queue_capacity);
}

/// Free-flow travel time for a route [hr] (sum of link free-flow times).
inline double route_free_flow_time(
    const RouteAlternative& route,
    const std::unordered_map<int, std::size_t>& link_pos,
    const EVPowerTrafficProblem& problem) noexcept
{
  double tt = 0.0;
  for (int li : route.link_indices) {
    auto it = link_pos.find(li);
    if (it == link_pos.end()) continue;
    const auto& lnk = problem.traffic.links[it->second];
    const double vf  = link_free_flow_speed(lnk);
    tt += std::max(kTol, lnk.length_km) / vf;
  }
  return tt;
}

}  // namespace hacdcpf::evpt

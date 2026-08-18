#include "evpt_internal.hpp"
#include "hacdcpf/api/hacdcpf.hpp"

namespace hacdcpf::evpt {
namespace {

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
  if (options.auto_generate_routes) {
    EVPowerTrafficOptions local_options = options;
    local_options.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, options);
    return simulate_ev_power_traffic(generated, local_options);
  }

  EVPowerTrafficResult result;
  result.final_system = problem.system;

  if (options.require_exact_mathematical_model) {
    result.feasible = false;
    result.status =
        "unsupported: exact full EV power-traffic mathematical-model "
        "verification is not available for the decomposed simulation path";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: route assignment, charging dispatch, and power validation "
        "are not one globally certified mathematical programme";
    result.warnings.push_back(result.mathematical_model_verification_status);
    return result;
  }

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
          session.energy_max_kwh =
              e_req + session_arbitrage_headroom_kwh(
                          e_req, session.max_discharge_kw,
                          (session.departure_step - session.arrival_step) * dt,
                          vehicles, demand.energy_max_kwh);
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
      const bool v2g_allowed =
          options.allow_v2g && session.v2g_capable && session.max_discharge_kw > kTol;
      const double needed =
          std::max(0.0, session.energy_target_kwh - energy);
      // Arbitrage pre-charge: at low-price steps a V2G session may fill its
      // headroom above the trip target so that high-price discharge later in
      // the dwell window is reachable (without this, energy never exceeds the
      // target and the discharge branch below is dead).
      double charge_room = needed;
      if (v2g_allowed && price <= options.low_price_threshold_per_kwh) {
        charge_room =
            std::max(needed, session.energy_max_kwh - energy);
      }
      double p_ch = 0.0;
      if (charge_room > kTol) {
        p_ch = std::min({session.max_charge_kw,
                         station_ch_res,
                         charge_room / (std::max(session.eta_charge, kTol) * dt)});
      }

      double p_dis = 0.0;
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
  // Per-station average dwell Δ_s [steps] (eq. evpt-wait: μ_s = N^plug_s / Δ_s)
  std::unordered_map<int, double> station_avg_dwell;
  std::unordered_map<int, int> station_dwell_count;
  for (int sid : station_ids) {
    station_avg_dwell[sid]   = 1.0;  // default Δ_s = 1
    station_dwell_count[sid] = 0;
  }
  for (const auto& session : sessions) {
    const int dwell = session.departure_step - session.arrival_step;
    if (dwell > 0 && station_avg_dwell.count(session.station_id)) {
      station_avg_dwell[session.station_id] += static_cast<double>(dwell);
      station_dwell_count[session.station_id]++;
    }
  }
  for (int sid : station_ids) {
    if (station_dwell_count[sid] > 0) {
      station_avg_dwell[sid] /= static_cast<double>(station_dwell_count[sid]);
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
      // Service throughput μ_s [veh/step] = N^plug_s / Δ_s ≈ P^max_s / (p_veh · Δ_s)
      // (eq. evpt-wait)
      const double cap_kw = charge_cap_kw.count(sid)
                                ? charge_cap_kw.at(sid)
                                : options.default_station_power_kw;
      const double pv      = std::max(kTol, options.default_route_stop_power_kw_per_vehicle);
      const double delta_s = std::max(1.0, station_avg_dwell[sid]);
      const double mu_sk   = cap_kw / (pv * delta_s);
      // W_{s,k} ≈ (q_{s,k} / μ_s) · Δt  [hours]  (eq. evpt-wait)
      station_wait_hr[sid][static_cast<std::size_t>(k)] =
          q_sk / std::max(mu_sk, 1e-9) * dt;
    }
  }

  const double base_load_mw = total_base_load_mw(problem.system);
  const double gen_cap_mw = total_generation_capacity_mw(problem.system, options);
  result.steps.reserve(static_cast<std::size_t>(T));
  result.system_by_step.reserve(static_cast<std::size_t>(T));
  // The network topology is fixed across the horizon (only station loads change
  // per step), so a prepared session reuses the canonical projection, solver
  // assembly, and symbolic factorization across steps. solve() rebuilds
  // automatically if a step ever changes topology, so results are unchanged.
  PreparedPowerFlowSession pf_session(options.pf_options);
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
      const auto pf = pf_session.solve(step_system);
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

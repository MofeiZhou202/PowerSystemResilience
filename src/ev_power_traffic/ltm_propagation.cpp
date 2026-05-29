// ltm_propagation.cpp
// ──────────────────────────────────────────────────────────────────────────
// Link Transmission Model (LTM) mesoscopic–microscopic traffic framework.
//
// Implements:
//   • Single-link LTM simulation (cumulative N_in / N_out curves)
//   • Multi-link network simulation with proportional node model
//   • Charging station service-node dynamics (Q / B / H populations)
//   • Individual EV agent simulation (SOC evolution, charging decision,
//     queueing, dwell)
//   • FIFO-consistent vehicle trajectory reconstruction
//   • Aggregate V2G potential computation
//
// Reference:
//   Yperman, I. (2007). "The Link Transmission Model for Dynamic Network
//   Loading." PhD Thesis, KU Leuven.
// ──────────────────────────────────────────────────────────────────────────

#include "hacdcpf/ev_power_traffic/ltm_network.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace hacdcpf::evpt {

namespace {

inline constexpr double kLTMTol = 1e-9;

// ── Demand helper ──────────────────────────────────────────────────────────

inline double demand_at(const std::vector<double>& dem, int k) {
  if (k >= 0 && k < static_cast<int>(dem.size()))
    return dem[static_cast<std::size_t>(k)];
  return 0.0;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────
// 1. Single-link LTM simulation
// ─────────────────────────────────────────────────────────────────────────
// Propagates cumulative N_in and N_out curves for one isolated link.
// demand_veh_hr[k]  – entering demand in veh/h at step k.
// downstream_receiving_override[k] – override for R^{down}_k [veh per step].
//   Empty vector or value <= 0 means free downstream (no constraint).
//
// Equations (technical notebook §20.5):
//   S_{a,k} = min(N_in[k-τ_ff] - N_out[k],  q_max * dt)
//   R_{a,k} = min(N_out[k-τ_bw] + N_jam - N_in[k],  q_max * dt)
//   u_{a,k} = min(d_k * dt,  R_{a,k})
//   v_{a,k} = min(S_{a,k},  R^{down}_{a,k})

LTMLinkState simulate_ltm_single_link(
    const LTMLinkParams&        params,
    const std::vector<double>&  demand_veh_hr,
    const std::vector<double>&  downstream_receiving_override)
{
  const int T = static_cast<int>(demand_veh_hr.size());
  LTMLinkState ls;
  ls.id     = params.id;
  ls.params = params;
  ls.init(T);

  for (int k = 0; k < T; ++k) {
    const double R_k = ls.receiving(k);
    const double S_k = ls.sending(k);

    // Entry flow: limited by demand and receiving capacity
    const double d_k  = demand_at(demand_veh_hr, k);
    const double u_k  = std::min(d_k * params.dt_hr, R_k);

    // Exit flow: limited by sending and downstream receiving.
    // NOTE: a value of 0 in downstream_receiving_override means FULLY BLOCKED
    // (zero receiving capacity).  Only a *negative* value (or an empty vector)
    // means "no override / free downstream."
    double ds_recv = std::numeric_limits<double>::infinity();
    if (k < static_cast<int>(downstream_receiving_override.size())) {
      const double ov = downstream_receiving_override[static_cast<std::size_t>(k)];
      if (ov >= 0.0) ds_recv = ov;  // 0 = fully blocked, negative = no constraint
    }
    const double v_k = std::min(S_k, ds_recv);

    ls.N_in [static_cast<std::size_t>(k + 1)] =
        ls.N_in [static_cast<std::size_t>(k)] + u_k;
    ls.N_out[static_cast<std::size_t>(k + 1)] =
        ls.N_out[static_cast<std::size_t>(k)] + v_k;
  }
  return ls;
}

// ─────────────────────────────────────────────────────────────────────────
// 2. Multi-link network LTM simulation
// ─────────────────────────────────────────────────────────────────────────
// Proportional node model: for each link a with downstream turns (a -> b1,
// a -> b2, ...), the sending flow is allocated proportionally to β values and
// constrained by the minimum receiving capacity across downstream links.
//
// Link update order: topological (origin links first).  For simplicity in
// the current implementation we update all links in order of their position
// in the links vector, which works correctly for DAG-structured corridors and
// diverges as long as origins are listed before sinks.

LTMSimulationResult simulate_ltm_network(const LTMNetworkProblem& problem)
{
  const int    T  = problem.T;
  const double dt = problem.dt_hr;

  LTMSimulationResult result;

  // ── Initialise link states ───────────────────────────────────────────
  for (const auto& lp : problem.links) {
    LTMLinkState ls;
    ls.id     = lp.id;
    ls.params = lp;
    ls.init(T);
    result.link_states[lp.id] = std::move(ls);

    result.link_results[lp.id].resize(static_cast<std::size_t>(T));
  }

  // ── Initialise charging stations ─────────────────────────────────────
  for (const auto& sp : problem.stations) {
    ChargingStationState ss;
    ss.id     = sp.id;
    ss.params = sp;
    ss.init(T);
    result.station_states[sp.id] = std::move(ss);
  }

  // ── Per-step network update ──────────────────────────────────────────
  for (int k = 0; k < T; ++k) {

    // Compute sending and receiving for all links
    std::unordered_map<int, double> S, R;
    for (auto& [lid, ls] : result.link_states) {
      S[lid] = ls.sending(k);
      R[lid] = ls.receiving(k);
    }

    // Compute inflow u_k and outflow v_k for each link
    std::unordered_map<int, double> u_total, v_total;
    for (auto& [lid, _] : result.link_states) {
      u_total[lid] = 0.0;
      v_total[lid] = 0.0;
    }

    // Add origin demand to links that have it
    for (auto& [lid, dem] : problem.origin_demand) {
      if (result.link_states.count(lid) == 0) continue;
      u_total[lid] += demand_at(dem, k) * dt;
    }

    // Turning flows: for each link a with downstream turns
    for (auto& [a_id, turns] : problem.downstream_turns) {
      if (result.link_states.count(a_id) == 0) continue;
      if (turns.empty()) {
        // Sink link: flow exits freely up to sending capacity
        v_total[a_id] += S.at(a_id);
        continue;
      }

      // Proportional node model:
      // maximum exit from a constrained by min(R_b / β_b) for all b
      double max_from_recv = std::numeric_limits<double>::infinity();
      double total_beta    = 0.0;
      for (const auto& t : turns) {
        total_beta += t.beta;
        if (t.beta > kLTMTol && result.link_states.count(t.downstream_link_id)) {
          max_from_recv = std::min(max_from_recv,
              R.at(t.downstream_link_id) / t.beta);
        }
      }
      const double actual_v = std::min(S.at(a_id), max_from_recv);
      v_total[a_id] += actual_v;

      for (const auto& t : turns) {
        if (!result.link_states.count(t.downstream_link_id)) continue;
        const double y_ab = (total_beta > kLTMTol)
                                ? actual_v * (t.beta / total_beta)
                                : 0.0;
        u_total[t.downstream_link_id] += y_ab;
      }
    }

    // Enforce receiving on total inflow; update cumulative curves
    for (auto& [lid, ls] : result.link_states) {
      const double u_k = std::min(u_total.at(lid), R.at(lid));
      const double v_k = v_total.at(lid);

      ls.N_in [static_cast<std::size_t>(k + 1)] =
          ls.N_in [static_cast<std::size_t>(k)] + u_k;
      ls.N_out[static_cast<std::size_t>(k + 1)] =
          ls.N_out[static_cast<std::size_t>(k)] + v_k;

      // Store step summary
      auto& sr         = result.link_results.at(lid)[static_cast<std::size_t>(k)];
      sr.link_id       = lid;
      sr.step          = k;
      sr.inflow_veh_hr = (dt > kLTMTol) ? u_k / dt : 0.0;
      sr.outflow_veh_hr= (dt > kLTMTol) ? v_k / dt : 0.0;
      sr.occupancy_veh = ls.occupancy(k);
      sr.sending_veh   = S.at(lid);
      sr.receiving_veh = R.at(lid);
    }
  }

  // ── Simulate charging stations ───────────────────────────────────────
  // Station arrivals are driven by EV fraction of total link outflow on the
  // access link.  For the network-only simulation without EV agents, use a
  // constant fraction of the first sink link's outflow as a proxy.
  // (Stations are fully exercised with individual EV agents in the micro layer.)
  for (auto& [sid, ss] : result.station_states) {
    const double mu       = ss.params.mu_per_step();
    const double dw_rate  = ss.params.dwell_rate();
    const int    n_plugs  = ss.params.n_plugs;

    for (int k = 0; k < T; ++k) {
      const double arr_k = ss.u_arr[static_cast<std::size_t>(k)];  // set by caller / EV agents

      // Service admission: limited by available plugs
      const double avail_plugs = std::max(0.0,
          static_cast<double>(n_plugs) - ss.B[static_cast<std::size_t>(k)]);
      const double b_k = std::min(
          ss.Q[static_cast<std::size_t>(k)] + arr_k, avail_plugs);
      ss.b_adm[static_cast<std::size_t>(k)] = b_k;

      // Service completion: deterministic rate
      const double c_k = std::min(
          ss.B[static_cast<std::size_t>(k)],
          ss.B[static_cast<std::size_t>(k)] * mu);
      ss.c_cmp[static_cast<std::size_t>(k)] = c_k;

      // Departures: limited by dwell completion rate
      const double dwell_pop = ss.H[static_cast<std::size_t>(k)] + c_k;
      const double o_k = std::min(dwell_pop, dwell_pop * dw_rate);
      ss.o_dep[static_cast<std::size_t>(k)] = o_k;

      // State update (eq. st-Q, st-B, st-H)
      ss.Q[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss.Q[static_cast<std::size_t>(k)] + arr_k - b_k);
      ss.B[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss.B[static_cast<std::size_t>(k)] + b_k - c_k);
      ss.H[static_cast<std::size_t>(k + 1)] =
          std::max(0.0, ss.H[static_cast<std::size_t>(k)] + c_k - o_k);
    }
  }

  // ── Individual EV agents ─────────────────────────────────────────────
  for (const auto& ev_p : problem.ev_agents) {
    EVAgentRecord rec;
    rec.id = ev_p.id;

    double energy  = ev_p.initial_energy_kwh();
    auto   state   = EVBehaviourState::Driving;
    double t_hr    = 0.0;
    int    k_step  = 0;
    int    path_i  = 0;

    double charge_remaining_hr  = 0.0;
    double dwell_remaining_hr   = 0.0;
    bool   charging_done        = false;  // has EV already charged?

    EVChargingEvent cur_event;
    bool            event_open = false;
    int             station_id = -1;

    // Which station is associated with this EV?
    // For now, attach to the first available station in problem.stations.
    if (!problem.stations.empty()) {
      station_id = problem.stations.front().id;
    }

    const int max_steps = T * 4;  // guard against infinite loop

    while (path_i < static_cast<int>(ev_p.path.size()) && k_step < max_steps) {
      const int lid = ev_p.path[static_cast<std::size_t>(path_i)];

      // --- Estimate remaining trip energy ---
      double e_rem = 0.0;
      for (int pi = path_i; pi < static_cast<int>(ev_p.path.size()); ++pi) {
        const int lpi = ev_p.path[static_cast<std::size_t>(pi)];
        for (const auto& lp : problem.links) {
          if (lp.id == lpi) { e_rem += lp.length_km * ev_p.consumption_kwh_per_km; break; }
        }
      }

      // --- Charging trigger (eq:charging-trigger-economic / eq:soc-feasibility) ---
      // Charge when remaining energy after the trip would fall below reserve:
      //   e_{v,k} − E^rem_{v,k} < e^res_v  (spec eq:soc-feasibility)
      // The additional SOC-percentage condition has been removed to match the
      // theoretical model; the mobility-reserve check is sufficient.
      if (state == EVBehaviourState::Driving &&
          !charging_done &&
          energy - e_rem < ev_p.reserve_kwh &&
          station_id >= 0) {
        state = EVBehaviourState::Queueing;
        event_open         = true;
        cur_event          = EVChargingEvent{};
        cur_event.station_id     = station_id;
        cur_event.arrival_time_hr = t_hr;
      }

      // --- State machine ---
      if (state == EVBehaviourState::Queueing) {
        // Check station queue
        double q_now = 0.0;
        if (result.station_states.count(station_id)) {
          const auto& ss  = result.station_states.at(station_id);
          const int   idx = std::min(k_step, T);
          q_now = ss.Q[static_cast<std::size_t>(idx)];
        }
        const double n_plugs = problem.stations.empty() ? 1.0
            : static_cast<double>(problem.stations.front().n_plugs);

        if (q_now < n_plugs) {
          // Plug available: start charging
          state = EVBehaviourState::Charging;
          const double target_e    = ev_p.battery_capacity_kwh * 0.90;
          const double needed      = std::max(0.0, target_e - energy);
          charge_remaining_hr      = needed / (ev_p.charge_power_kw * ev_p.eta_ch);
          if (event_open) cur_event.charge_start_hr = t_hr;

          // Record arrival in station state
          if (result.station_states.count(station_id)) {
            const auto ki = static_cast<std::size_t>(std::min(k_step, T - 1));
            result.station_states.at(station_id).u_arr[ki] += 1.0;
          }
        } else {
          if (event_open) cur_event.queue_wait_hr += problem.dt_hr;
        }

      } else if (state == EVBehaviourState::Charging) {
        const double power_in = ev_p.charge_power_kw * ev_p.eta_ch * dt;
        energy = std::min(energy + power_in, ev_p.battery_capacity_kwh);
        charge_remaining_hr -= dt;
        if (event_open) cur_event.energy_added_kwh += power_in;

        if (charge_remaining_hr <= 0.0) {
          state = EVBehaviourState::Dwelling;
          dwell_remaining_hr = problem.stations.empty()
              ? 0.25 : problem.stations.front().mean_dwell_time_hr;
          if (event_open) cur_event.charge_end_hr = t_hr;
        }

      } else if (state == EVBehaviourState::Dwelling) {
        dwell_remaining_hr -= dt;
        if (dwell_remaining_hr <= 0.0) {
          // After dwell: enter Discharging if V2G capable and surplus above reserve
          // (eq:soc-evolution: e_{v,k+1} = e_{v,k} - p^dis Δt / η^dis)
          if (ev_p.max_discharge_kw > kLTMTol &&
              energy > ev_p.reserve_kwh + kLTMTol) {
            state = EVBehaviourState::Discharging;
          } else {
            state        = EVBehaviourState::Driving;
            charging_done = true;
            if (event_open) {
              cur_event.departure_time_hr = t_hr;
              rec.charging_events.push_back(cur_event);
              event_open = false;
            }
          }
        }

      } else if (state == EVBehaviourState::Discharging) {
        // V2G discharge phase (eq:soc-evolution):
        //   e_{v,k+1} = e_{v,k} - p^dis_{v,k} · Δt / η^dis
        // Discharge at max rate until SOC falls to reserve.
        const double p_dis    = ev_p.max_discharge_kw;
        const double e_loss   = p_dis / ev_p.eta_dis * dt;
        energy = std::max(ev_p.reserve_kwh, energy - e_loss);

        // Exit Discharging when SOC is at reserve (no more V2G surplus)
        if (energy <= ev_p.reserve_kwh + kLTMTol) {
          state        = EVBehaviourState::Driving;
          charging_done = true;
          if (event_open) {
            cur_event.departure_time_hr = t_hr;
            rec.charging_events.push_back(cur_event);
            event_open = false;
          }
        }
      }

      if (state == EVBehaviourState::Driving) {
        // Look up link parameters
        double link_len  = 1.0;
        int    tf_steps  = 1;
        for (const auto& lp : problem.links) {
          if (lp.id == lid) {
            link_len = lp.length_km;
            tf_steps = lp.tau_ff();
            break;
          }
        }
        const double e_consumed = link_len * ev_p.consumption_kwh_per_km;
        energy    = std::max(0.0, energy - e_consumed);
        t_hr     += static_cast<double>(tf_steps) * dt;
        k_step   += tf_steps;
        path_i   += 1;
      } else {
        t_hr   += dt;
        k_step += 1;
      }

      rec.times_hr.push_back(t_hr);
      rec.soc.push_back(energy / ev_p.battery_capacity_kwh);
      rec.energy_kwh.push_back(energy);
      rec.states.push_back(state);
    }

    if (path_i >= static_cast<int>(ev_p.path.size())) {
      rec.states.push_back(EVBehaviourState::Completed);
      rec.times_hr.push_back(t_hr);
      rec.soc.push_back(energy / ev_p.battery_capacity_kwh);
      rec.energy_kwh.push_back(energy);
    }

    // Re-run station simulation with updated arrivals
    for (auto& [sid, ss] : result.station_states) {
      const double mu      = ss.params.mu_per_step();
      const double dw_rate = ss.params.dwell_rate();
      const int    n_plugs = ss.params.n_plugs;
      for (int k = 0; k < T; ++k) {
        const double arr_k = ss.u_arr[static_cast<std::size_t>(k)];
        const double avail_plugs = std::max(0.0,
            static_cast<double>(n_plugs) - ss.B[static_cast<std::size_t>(k)]);
        const double b_k = std::min(ss.Q[static_cast<std::size_t>(k)] + arr_k, avail_plugs);
        ss.b_adm[static_cast<std::size_t>(k)] = b_k;
        const double c_k = std::min(ss.B[static_cast<std::size_t>(k)],
                                    ss.B[static_cast<std::size_t>(k)] * mu);
        ss.c_cmp[static_cast<std::size_t>(k)] = c_k;
        const double dwell_pop = ss.H[static_cast<std::size_t>(k)] + c_k;
        const double o_k = std::min(dwell_pop, dwell_pop * dw_rate);
        ss.o_dep[static_cast<std::size_t>(k)] = o_k;
        ss.Q[static_cast<std::size_t>(k + 1)] =
            std::max(0.0, ss.Q[static_cast<std::size_t>(k)] + arr_k - b_k);
        ss.B[static_cast<std::size_t>(k + 1)] =
            std::max(0.0, ss.B[static_cast<std::size_t>(k)] + b_k - c_k);
        ss.H[static_cast<std::size_t>(k + 1)] =
            std::max(0.0, ss.H[static_cast<std::size_t>(k)] + c_k - o_k);
      }
    }

    result.ev_records.push_back(std::move(rec));
  }

  // ── V2G potential ─────────────────────────────────────────────────────
  for (auto& [sid, ss] : result.station_states) {
    // Use per-step average SOC of dwell population from EV records
    // (simplified: use a flat average across all agents that visited station)
    std::vector<double> avg_soc(static_cast<std::size_t>(T), 0.70);

    // Find EV agents that visited this station and compute average end-of-charge SOC
    double sum_soc = 0.0;
    int    cnt_ev  = 0;
    for (const auto& rec : result.ev_records) {
      for (const auto& ev : rec.charging_events) {
        if (ev.station_id == sid) {
          sum_soc += (rec.soc.empty() ? 0.7 : rec.soc.back());
          ++cnt_ev;
        }
      }
    }
    if (cnt_ev > 0) {
      const double mean_soc = sum_soc / cnt_ev;
      std::fill(avg_soc.begin(), avg_soc.end(), mean_soc);
    }

    // Default EV parameters for V2G (use first agent if available)
    double bat_cap    = 80.0;
    double max_dis_kw = 30.0;
    double eta_dis    = 0.95;
    double res_soc    = 0.10;
    if (!problem.ev_agents.empty()) {
      const auto& ea = problem.ev_agents.front();
      bat_cap    = ea.battery_capacity_kwh;
      max_dis_kw = ea.max_discharge_kw;
      eta_dis    = ea.eta_dis;
      res_soc    = ea.reserve_kwh / ea.battery_capacity_kwh;
    }

    result.v2g_potential_kw[sid] = compute_v2g_potential(
        ss, avg_soc, res_soc, bat_cap, max_dis_kw, eta_dis);
  }

  return result;
}

// ─────────────────────────────────────────────────────────────────────────
// 3. FIFO trajectory reconstruction
// ─────────────────────────────────────────────────────────────────────────
// For each vehicle with cumulative rank ξ_{v,a} = entry_ranks[i],
// find k^{out}_{v,a} = min{k : N_out[k] >= ξ_{v,a}}  (eq. fifo-exit).

std::vector<ReconstructedTrajectory> reconstruct_trajectories(
    const LTMLinkState&        link,
    const std::vector<double>& entry_ranks,
    const std::vector<int>&    entry_steps)
{
  const int T  = static_cast<int>(link.N_out.size()) - 1;
  const double dt = link.params.dt_hr;

  std::vector<ReconstructedTrajectory> results;
  results.reserve(entry_ranks.size());

  for (std::size_t i = 0; i < entry_ranks.size(); ++i) {
    const double xi  = entry_ranks[i];
    const int    k_e = (i < entry_steps.size()) ? entry_steps[i] : 0;

    // Find first k where N_out[k] >= xi
    int k_out = T;
    for (int k = k_e; k <= T; ++k) {
      if (link.N_out[static_cast<std::size_t>(k)] >= xi - kLTMTol) {
        k_out = k;
        break;
      }
    }

    ReconstructedTrajectory rt;
    rt.vehicle_rank     = static_cast<int>(i);
    rt.entry_time_hr    = static_cast<double>(k_e)   * dt;
    rt.exit_time_hr     = static_cast<double>(k_out) * dt;
    rt.experienced_travel_time_hr =
        static_cast<double>(k_out - k_e) * dt;
    results.push_back(rt);
  }
  return results;
}

// ─────────────────────────────────────────────────────────────────────────
// 4. V2G potential
// ─────────────────────────────────────────────────────────────────────────
// P^{v2g,pot}_{s,k} = Σ_{v∈dwell} min(p̄^{dis}_v, (e_{v,k}-e^{res}_v)/dt * η^{dis})
// (eq. v2g-potential)
//
// Here we use aggregate approximation:
//   P = H_{s,k} * min(max_discharge_kw,
//         max(0, avg_soc - reserve_soc) * bat_cap / dt * eta_dis)

std::vector<double> compute_v2g_potential(
    const ChargingStationState& station,
    const std::vector<double>&  avg_soc_dwell,
    double reserve_soc,
    double battery_capacity_kwh,
    double max_discharge_kw,
    double eta_dis)
{
  const int T = static_cast<int>(station.H.size()) - 1;
  std::vector<double> v2g(static_cast<std::size_t>(T), 0.0);

  const double dt = station.params.dt_hr;

  for (int k = 0; k < T; ++k) {
    const double h_k   = station.H[static_cast<std::size_t>(k)];
    if (h_k < kLTMTol) continue;

    const double soc_k  = (k < static_cast<int>(avg_soc_dwell.size()))
                              ? avg_soc_dwell[static_cast<std::size_t>(k)]
                              : reserve_soc;
    const double avail_e = std::max(0.0, (soc_k - reserve_soc) * battery_capacity_kwh);
    const double p_per_veh = std::min(max_discharge_kw,
                                      avail_e / (dt + kLTMTol) * eta_dis);
    v2g[static_cast<std::size_t>(k)] = h_k * p_per_veh;
  }
  return v2g;
}

}  // namespace hacdcpf::evpt

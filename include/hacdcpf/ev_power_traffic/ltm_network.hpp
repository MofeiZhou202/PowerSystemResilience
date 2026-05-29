#pragma once

// ev_power_traffic/ltm_network.hpp
// ──────────────────────────────────────────────────────────────────────────
// Data structures and public API for the Link Transmission Model (LTM) based
// mesoscopic–microscopic traffic framework.
//
// Reference:
//   Yperman, I. (2007). "The Link Transmission Model for Dynamic Network
//   Loading." PhD Thesis, KU Leuven.
//
// Architecture (Section 20 of the technical notebook):
//   Mesoscopic layer  – LTM cumulative curves N_in / N_out; sending and
//                       receiving functions; intersection node model.
//   Charging stations – endogenous service nodes (Q / B / H populations).
//   Microscopic layer – individual EV agents with SOC evolution, charging
//                       decision, queueing, dwell, and FIFO trajectory
//                       reconstruction.
// ──────────────────────────────────────────────────────────────────────────

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace hacdcpf::evpt {

// ── Vehicle class ──────────────────────────────────────────────────────────

enum class LTMVehicleClass : int {
  Fuel = 0,  ///< Internal combustion vehicle
  EV   = 1,  ///< Battery electric vehicle
};

// ── Link parameters (fundamental diagram + geometry) ──────────────────────

/// Parameters that fully define the triangular fundamental diagram for one
/// directed road link (eq. triangular-fd in the technical notebook).
struct LTMLinkParams {
  int    id{0};
  double length_km{1.0};         ///< L_a
  double v_ff_km_hr{60.0};       ///< v^{ff}_a  free-flow speed
  double w_km_hr{20.0};          ///< w_a       backward wave speed
  double capacity_veh_hr{1800.0};///< \bar{q}_a link capacity [veh/h]
  double dt_hr{0.25};            ///< \Delta t  time step [h]

  /// Derived: critical density k^c = q_max / v_ff  [veh/km]
  double k_crit() const { return capacity_veh_hr / v_ff_km_hr; }

  /// Derived: jam density [veh/km]
  double k_jam() const {
    return capacity_veh_hr * (1.0 / v_ff_km_hr + 1.0 / w_km_hr);
  }

  /// Derived: jam storage N^jam_a = k_jam * L_a  [veh]
  double n_jam() const { return k_jam() * length_km; }

  /// Derived: free-flow travel time in discrete steps τ^{ff}_a (eq. tau-ff-bw)
  int tau_ff() const {
    return std::max(1, static_cast<int>(
        std::ceil(length_km / (v_ff_km_hr * dt_hr))));
  }

  /// Derived: backward wave travel time in discrete steps τ^{bw}_a
  int tau_bw() const {
    return std::max(1, static_cast<int>(
        std::ceil(length_km / (w_km_hr * dt_hr))));
  }
};

// ── LTM link runtime state (cumulative curves) ─────────────────────────────

/// Runtime state for one LTM link over T time steps.
/// N_in[k] and N_out[k] are indexed from 0 to T inclusive (size T+1).
struct LTMLinkState {
  int id{0};
  LTMLinkParams params;

  /// N^{in}_{a,k}  – cumulative vehicles having entered link a by step k.
  std::vector<double> N_in;
  /// N^{out}_{a,k} – cumulative vehicles having exited link a by step k.
  std::vector<double> N_out;

  /// Class-specific cumulative entering curves (indexed by LTMVehicleClass).
  std::unordered_map<int, std::vector<double>> N_in_class;
  std::unordered_map<int, std::vector<double>> N_out_class;

  void init(int T) {
    N_in.assign(static_cast<std::size_t>(T + 1), 0.0);
    N_out.assign(static_cast<std::size_t>(T + 1), 0.0);
  }

  /// n_{a,k} = N^{in}_{a,k} - N^{out}_{a,k}
  double occupancy(int k) const {
    return N_in[static_cast<std::size_t>(k)] -
           N_out[static_cast<std::size_t>(k)];
  }

  /// LTM sending function S_{a,k} (eq. ltm-sending) [vehicles per step]
  double sending(int k) const {
    const int tf  = params.tau_ff();
    const int k_d = std::max(0, k - tf);
    const double s = N_in[static_cast<std::size_t>(k_d)] -
                     N_out[static_cast<std::size_t>(k)];
    return std::max(0.0,
        std::min(s, params.capacity_veh_hr * params.dt_hr));
  }

  /// LTM receiving function R_{a,k} (eq. ltm-receiving) [vehicles per step]
  double receiving(int k) const {
    const int tb  = params.tau_bw();
    const int k_d = std::max(0, k - tb);
    const double r = N_out[static_cast<std::size_t>(k_d)] +
                     params.n_jam() -
                     N_in[static_cast<std::size_t>(k)];
    return std::max(0.0,
        std::min(r, params.capacity_veh_hr * params.dt_hr));
  }
};

// ── Downstream connectivity ────────────────────────────────────────────────

/// One outgoing turn from a link, with a fixed turning ratio β.
struct LTMTurn {
  int downstream_link_id{0};
  double beta{1.0};  ///< turning ratio (must sum to 1 across outgoing links)
};

// ── Charging station parameters ────────────────────────────────────────────

/// Parameters for a charging station service node (Section 20.7).
struct ChargingStationParams {
  int    id{0};
  int    n_plugs{10};              ///< \bar{B}_s  maximum simultaneous chargers
  double power_kw{50.0};           ///< charging power per plug [kW]
  double mean_service_time_hr{0.5};///< 1/μ mean charging duration
  double mean_dwell_time_hr{0.25}; ///< mean post-charge dwell time
  double dt_hr{0.25};

  double mu_per_step() const { return dt_hr / mean_service_time_hr; }
  double dwell_rate() const  { return dt_hr / mean_dwell_time_hr; }

  /// Spec μ^step = B̄_s · Δt / T̄^service (eq:completion-constraints).
  /// This is the maximum total service completions per step across all plugs.
  /// Used as an upper-bound check: c_{s,k} ≤ μ^step.
  /// In the fluid departure model the actual rate is B_k · mu_per_step(),
  /// which always satisfies c_k ≤ B_k ≤ μ^step since B_k ≤ B̄_s.
  double mu_step() const { return static_cast<double>(n_plugs) * mu_per_step(); }
};

// ── Charging station runtime state ─────────────────────────────────────────

/// Runtime state for one charging station over T steps.
/// Q, B, H are indexed 0..T (size T+1); flow arrays are size T.
struct ChargingStationState {
  int id{0};
  ChargingStationParams params;

  std::vector<double> Q;    ///< Q_{s,k}  waiting queue
  std::vector<double> B;    ///< B_{s,k}  in service (charging)
  std::vector<double> H;    ///< H_{s,k}  post-service dwell

  std::vector<double> u_arr;///< u^S_{s,k}  arrivals per step
  std::vector<double> b_adm;///< b_{s,k}   admitted to charging
  std::vector<double> c_cmp;///< c_{s,k}   completed charging
  std::vector<double> o_dep;///< o_{s,k}   station departures

  void init(int T) {
    const auto sz  = static_cast<std::size_t>(T + 1);
    const auto szT = static_cast<std::size_t>(T);
    Q.assign(sz,  0.0);
    B.assign(sz,  0.0);
    H.assign(sz,  0.0);
    u_arr.assign(szT, 0.0);
    b_adm.assign(szT, 0.0);
    c_cmp.assign(szT, 0.0);
    o_dep.assign(szT, 0.0);
  }

  /// Cumulative arrivals U_{s,k} (size T+1)
  std::vector<double> cumulative_arrivals() const {
    std::vector<double> cum(u_arr.size() + 1, 0.0);
    for (std::size_t i = 0; i < u_arr.size(); ++i)
      cum[i + 1] = cum[i] + u_arr[i];
    return cum;
  }

  std::vector<double> cumulative_service_start() const {
    std::vector<double> cum(b_adm.size() + 1, 0.0);
    for (std::size_t i = 0; i < b_adm.size(); ++i)
      cum[i + 1] = cum[i] + b_adm[i];
    return cum;
  }

  std::vector<double> cumulative_service_done() const {
    std::vector<double> cum(c_cmp.size() + 1, 0.0);
    for (std::size_t i = 0; i < c_cmp.size(); ++i)
      cum[i + 1] = cum[i] + c_cmp[i];
    return cum;
  }

  std::vector<double> cumulative_departures() const {
    std::vector<double> cum(o_dep.size() + 1, 0.0);
    for (std::size_t i = 0; i < o_dep.size(); ++i)
      cum[i + 1] = cum[i] + o_dep[i];
    return cum;
  }
};

// ── EV agent ───────────────────────────────────────────────────────────────

/// Behavioural states for a microscopic EV agent (Section 20.8).
/// Matches the state space σ_{v,k} from eq:ev-states:
///   {driving, accessing_station, queueing, charging,
///    discharging, dwelling, egressing, completed}
/// Values 0–4 are kept stable for backward compatibility; the three
/// extended states (5–7) are appended.
enum class EVBehaviourState : int {
  Driving          = 0,
  Queueing         = 1,
  Charging         = 2,
  Dwelling         = 3,
  Completed        = 4,
  AccessingStation = 5,  ///< EV moving from road link to station access queue
  Discharging      = 6,  ///< EV actively discharging (V2G); SOC decreases by p^dis Δt/η^dis
  Egressing        = 7,  ///< EV leaving station back to road network
};

/// A charging session record for one EV at one station.
struct EVChargingEvent {
  int    station_id{0};
  double arrival_time_hr{0.0};
  double queue_wait_hr{0.0};
  double charge_start_hr{-1.0};
  double charge_end_hr{-1.0};
  double departure_time_hr{-1.0};
  double energy_added_kwh{0.0};
};

/// Parameters for a single EV agent.
struct EVAgentParams {
  int    id{0};
  double battery_capacity_kwh{80.0};
  double initial_soc{0.7};          ///< fraction [0,1]
  double consumption_kwh_per_km{0.20};
  double reserve_kwh{8.0};          ///< e^{res}_v minimum reserve
  double charge_power_kw{50.0};
  double eta_ch{0.95};              ///< charging efficiency
  double eta_dis{0.95};             ///< discharging efficiency
  double max_discharge_kw{30.0};    ///< \bar{p}^{dis}_v for V2G

  /// Ordered sequence of link IDs forming the EV's activity path.
  std::vector<int> path;

  double initial_energy_kwh() const {
    return initial_soc * battery_capacity_kwh;
  }
};

/// Full simulation record for one EV agent.
struct EVAgentRecord {
  int    id{0};
  std::vector<double>          times_hr;
  std::vector<double>          soc;
  std::vector<double>          energy_kwh;
  std::vector<EVBehaviourState> states;
  std::vector<EVChargingEvent> charging_events;

  double final_soc() const {
    return soc.empty() ? 0.0 : soc.back();
  }
  bool completed() const {
    return !states.empty() && states.back() == EVBehaviourState::Completed;
  }
};

// ── FIFO trajectory reconstruction ────────────────────────────────────────

/// The reconstructed trajectory of one vehicle on one link.
struct TrajectorySegment {
  int    link_id{0};
  double entry_time_hr{0.0};
  double exit_time_hr{0.0};
  double travel_time_hr{0.0};
};

/// Full FIFO-reconstructed trajectory for a cohort vehicle.
struct ReconstructedTrajectory {
  int    vehicle_rank{0};      ///< ξ_{v,a} – cumulative rank at link entry
  double entry_time_hr{0.0};
  double exit_time_hr{0.0};
  double experienced_travel_time_hr{0.0};
};

// ── Network problem description ────────────────────────────────────────────

/// Complete description of an LTM network loading problem.
struct LTMNetworkProblem {
  int    T{96};                ///< number of time steps
  double dt_hr{0.25};          ///< time step size [h]

  std::vector<LTMLinkParams> links;

  /// downstream_turns[link_id] = list of outgoing turns (can be empty = sink).
  std::unordered_map<int, std::vector<LTMTurn>> downstream_turns;

  /// origin_demand[link_id] = demand array of length T [veh/h].
  /// This is d^{orig}_{b,k} injected at the link entry.
  std::unordered_map<int, std::vector<double>> origin_demand;

  /// EV penetration fraction used for class-split semi-dynamic analysis.
  double ev_penetration{0.30};

  /// Charging stations attached to the network.
  std::vector<ChargingStationParams> stations;

  /// EV agents to simulate in the microscopic layer.
  std::vector<EVAgentParams> ev_agents;
};

// ── Simulation result ──────────────────────────────────────────────────────

/// Per-link per-step summary.
struct LTMLinkStepResult {
  int    link_id{0};
  int    step{0};
  double inflow_veh_hr{0.0};
  double outflow_veh_hr{0.0};
  double occupancy_veh{0.0};
  double sending_veh{0.0};
  double receiving_veh{0.0};
};

/// Full result of one LTM network simulation.
struct LTMSimulationResult {
  /// Runtime link states (N_in / N_out cumulative curves).
  std::unordered_map<int, LTMLinkState> link_states;

  /// Per-link per-step summaries (link_id -> vector over T steps).
  std::unordered_map<int, std::vector<LTMLinkStepResult>> link_results;

  /// Charging station states.
  std::unordered_map<int, ChargingStationState> station_states;

  /// Individual EV agent records.
  std::vector<EVAgentRecord> ev_records;

  /// FIFO trajectory reconstructions per link (link_id -> cohort results).
  std::unordered_map<int, std::vector<ReconstructedTrajectory>> trajectories;

  /// Aggregate V2G potential per station per step [kW].
  std::unordered_map<int, std::vector<double>> v2g_potential_kw;
};

// ── Public API ─────────────────────────────────────────────────────────────

/// Simulate an LTM network given a problem description.
/// Returns the full LTMSimulationResult including link states, station states,
/// EV agent records, and V2G potential.
LTMSimulationResult simulate_ltm_network(const LTMNetworkProblem& problem);

/// Simulate a single LTM link in isolation (useful for unit testing).
/// Returns the LTMLinkState after T steps.
LTMLinkState simulate_ltm_single_link(
    const LTMLinkParams&        params,
    const std::vector<double>&  demand_veh_hr,
    const std::vector<double>&  downstream_receiving_override = {});

/// FIFO-consistent exit-time reconstruction for a cohort on one link.
/// entry_ranks[i] = N_in value at the moment vehicle i entered the link.
/// Returns one ReconstructedTrajectory per vehicle.
std::vector<ReconstructedTrajectory> reconstruct_trajectories(
    const LTMLinkState&         link,
    const std::vector<double>&  entry_ranks,
    const std::vector<int>&     entry_steps);

/// Compute aggregate V2G potential at a charging station.
std::vector<double> compute_v2g_potential(
    const ChargingStationState& station,
    const std::vector<double>&  avg_soc_dwell,
    double reserve_soc,
    double battery_capacity_kwh,
    double max_discharge_kw,
    double eta_dis);

}  // namespace hacdcpf::evpt

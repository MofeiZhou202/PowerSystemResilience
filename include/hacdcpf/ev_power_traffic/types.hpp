#pragma once

// ev_power_traffic/types.hpp
// ──────────────────────────────────────────────────────────────────────────
// Pure data-structure definitions for the EV power-traffic coupling layer.
// No dependency on the power-system model or solver headers; safe to include
// from any translation unit.

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace hacdcpf::evpt {

// ── Assignment model selector ─────────────────────────────────────────────

enum class AssignmentModel {
  DeterministicShortestPath,
  Logit,
  CapacityAwareGreedy,
  SystemOptimalLP,
  SystemOptimalMILP,
  // Backward-compatible alias for earlier examples.  This mode is a
  // capacity-aware greedy approximation, not an exact system-optimal solver.
  SystemOptimal = CapacityAwareGreedy,
};

// ── Traffic network ───────────────────────────────────────────────────────

struct TrafficNode {
  int index{0};
  std::string name;
};

struct TrafficLink {
  int index{0};
  int from_node{0};
  int to_node{0};
  double length_km{0.0};
  double free_flow_time_hr{0.0};
  double capacity_veh_per_hr{0.0};
  double jam_vehicles{0.0};
  bool available{true};

  // Optional time-indexed overrides.  Empty vectors use the scalar fields.
  std::vector<double> capacity_profile_veh_per_hr;
  std::vector<bool> availability_profile;

  // BPR-style delay parameters used for generalized-cost calculation.
  double alpha{0.15};
  double beta{4.0};

  // Per-vehicle traction energy consumption (eq. evpt-drive-energy).
  // Units: kWh / (vehicle · km).  Zero disables SOC feasibility checking.
  double drive_energy_kwh_per_veh_km{0.0};
};

struct TrafficGraph {
  std::vector<TrafficNode> nodes;
  std::vector<TrafficLink> links;
};

// ── CTM network state ─────────────────────────────────────────────────────
//
// Each directed link is discretised into M_a cells.  For commodity-aware
// routing (route-specific CTM) each cell carries per-commodity occupancy so
// that station-choice identity is preserved through diverges.
//
// Reference: Daganzo (1994) "The cell transmission model", Transportation
// Research Part B.

struct CTMLinkCell {
  // Aggregate and commodity-specific occupancy [vehicles].
  double n{0.0};                      // n_{a,m,k}  – total occupancy
  std::vector<double> n_route;        // n^r_{a,m,k} – per-route share (size = n_routes)
};

struct CTMLinkState {
  int link_index{0};
  int n_cells{1};
  double cell_length_km{0.0};         // δ_a = L_a / M_a
  double free_flow_speed_km_hr{0.0};  // v^f_a
  double backward_wave_speed_km_hr{0.0};  // w_a
  double max_flow_veh_per_hr{0.0};    // q^max_a = v^f_a * w_a / (v^f_a + w_a) * k^jam_a
  double jam_occupancy_per_cell{0.0}; // N^jam_{a,m} = k^jam_a * δ_a
  std::vector<CTMLinkCell> cells;     // cells[0..n_cells-1]

  // Sending function S_{a,m,k}(n) = min(n * v^f/δ, q^max) * dt
  // Receiving function R_{a,m,k}(n) = min(q^max, w*(N^jam - n)/δ) * dt
};

struct CTMNodeTurningFractions {
  int node_index{0};
  // turning_fraction[in_link_idx][out_link_idx][route_idx] ∈ [0,1]
  // Rows: incoming link indices; Cols: outgoing link indices; Depth: route.
  // Empty means: infer from route topology (precomputed from route link sequences).
  std::map<int, std::map<int, std::vector<double>>> fractions;
};

// Per-step summary of a single link for output reporting.
struct CTMLinkStepResult {
  int link_index{0};
  double total_inflow_veh{0.0};    // y_{a,0,k}   – first-cell entry flow
  double total_outflow_veh{0.0};   // y_{a,M,k}   – last-cell exit flow
  double total_occupancy_veh{0.0}; // Σ_m n_{a,m,k}
  double mean_travel_time_hr{0.0}; // experienced travel time this step
};

// Full CTM simulation result for one forward pass.
struct CTMSimulationResult {
  // n_steps × n_links × n_cells occupancy history (row-major: [k][a][m]).
  // Stored only when record_cell_history = true.
  std::vector<std::vector<CTMLinkState>> cell_history;

  // Per-link per-step summary (always stored).
  // Indexed as step_link_results[step][link_pos].
  std::vector<std::vector<CTMLinkStepResult>> step_link_results;

  // Station arrival flows λ_{s,k} (vehicles arriving per step per station).
  std::unordered_map<int, std::vector<double>> station_arrivals;

  // Experienced route travel times T^{CTM}_{r,k} [hr] for each route
  // at each departure step (used as cost in DUE iterations).
  // Indexed as route_travel_time[route_index][departure_step].
  std::unordered_map<int, std::vector<double>> route_travel_time;
};

// Forward declarations needed by CTMDUEResult (defined later in this file).
struct EVChargingSession;
struct ChargingSessionResult;

// Dynamic User Equilibrium (DUE) result.
struct CTMDUEResult {
  bool converged{false};
  int iterations{0};
  double gap{0.0};          // max |x^{(i)} - x^{(i-1)}| / demand
  double relative_gap{0.0}; // Wardrop relative gap: (SC - UC) / UC

  // Final route flows x_{d,r,k} after convergence.
  // Indexed: flow_by_demand_route[demand_index][route_index] = vehicles.
  std::unordered_map<int, std::unordered_map<int, double>> flow_by_demand_route;

  // Iteration history: [gap, relative_gap, max_shift].
  struct IterRecord {
    int iteration{0};
    double gap{0.0};
    double relative_gap{0.0};
    double max_shift_veh{0.0};
  };
  std::vector<IterRecord> history;

  CTMSimulationResult final_ctm;

  // ── Session synthesis and charging dispatch (post-DUE) ─────────────────
  // One EVChargingSession per (demand, route, charging_stop) triple with
  // positive DUE flow.  Arrival steps are computed from CTM-derived travel
  // times (not free-flow), making them more accurate than Formulation A's
  // precomputed steps.  Dispatch follows the same greedy smart-charging LP
  // heuristic (Eqs. evpt-smart-obj–evpt-smart-slack) as Formulation A.
  std::vector<EVChargingSession>   sessions;
  std::vector<ChargingSessionResult> session_results;
  double total_requested_energy_kwh{0.0};
  double total_delivered_energy_kwh{0.0};
  double total_unserved_energy_kwh{0.0};

  // Per-station per-simulation-step net EV load [kW] (charge minus V2G).
  // Indexed: station_ev_load_kw[station_id][sim_step].
  std::unordered_map<int, std::vector<double>> station_ev_load_kw;

  // Station queue state at each simulation step
  // (Eqs. evpt-queue-serve, evpt-queue-update, evpt-plug-occ, evpt-wait).
  // station_queue_veh has size T_sim+1; plug_occupancy and waiting_time
  // have size T_sim.  Indexed: [station_id][sim_step].
  std::unordered_map<int, std::vector<double>> station_queue_veh;
  std::unordered_map<int, std::vector<double>> station_plug_occupancy_veh;
  std::unordered_map<int, std::vector<double>> station_waiting_time_hr;

  // ── Price-of-anarchy benchmark ──────────────────────────────────────────
  // Populated when DUEOptions::compute_system_optimal_benchmark = true.
  // DUE TSTT is computed from converged CTM route flows × CTM travel times.
  // SO TSTT comes from a Formulation-A system-optimal LP run on the same
  // network.  The two use different traffic models (CTM vs. BPR) so the
  // ratio is approximate; it serves as an upper-bound indicator of DUE
  // inefficiency.
  bool   has_so_benchmark{false};
  double poa_tstt_ratio{1.0};  ///< TSTT_DUE(CTM) / TSTT_SO(LP)
  double due_tstt_hr{0.0};     ///< total system travel time under DUE
  double so_tstt_hr{0.0};      ///< total system travel time under SO LP
};

// ── Route and charging stop definitions ───────────────────────────────────

struct RouteChargingStop {
  int station_id{0};
  double requested_energy_kwh_per_vehicle{0.0};
  int dwell_steps{1};
  double max_charge_kw_per_vehicle{0.0};
  bool v2g_capable{false};
  double max_discharge_kw_per_vehicle{0.0};
};

struct RouteAlternative {
  int index{0};
  int origin_node{0};
  int destination_node{0};
  std::vector<int> link_indices;
  std::vector<RouteChargingStop> charging_stops;
  double toll_cost{0.0};
};

// ── Demand and session definitions ────────────────────────────────────────

struct EVDemand {
  int index{0};
  int origin_node{0};
  int destination_node{0};
  int ev_class{0};
  int departure_step{0};
  double vehicles{0.0};
  std::vector<int> candidate_route_indices;

  // Battery state at departure (eq. evpt-route-soc / evpt-route-soc-bounds).
  // Negative values mean "not set"; SOC feasibility checking is skipped.
  double initial_energy_kwh{-1.0};  // E_1 at departure (< 0 = not set)
  double energy_min_kwh{0.0};       // \underline{E}_c
  double energy_max_kwh{-1.0};      // \overline{E}_c (< 0 = unbounded)

  // Gross willingness-to-pay per served vehicle ($/vehicle).
  // Used by solve_joint_social_welfare() to compute consumer benefit.
  // Zero means "not set"; only affects social welfare accounting.
  double willingness_to_pay_per_vehicle{0.0};
};

struct EVChargingSession {
  int index{0};
  int station_id{0};
  int arrival_step{0};
  int departure_step{0};

  // The following energy and power fields are aggregate values for the session.
  double vehicle_count{1.0};
  double energy_initial_kwh{0.0};
  double energy_target_kwh{0.0};
  double energy_min_kwh{0.0};
  double energy_max_kwh{0.0};
  double max_charge_kw{0.0};
  double max_discharge_kw{0.0};
  double eta_charge{0.95};
  double eta_discharge{0.95};
  bool v2g_capable{false};

  int source_demand_index{0};
  int source_route_index{0};
};

struct StationPriceProfile {
  int station_id{0};
  std::vector<double> price_per_kwh;
};

// ── Per-step and aggregate result structures ──────────────────────────────

struct RouteAssignmentResult {
  int demand_index{0};
  int route_index{0};
  int departure_step{0};
  double vehicles{0.0};
  double generalized_cost{0.0};
  double travel_time_hr{0.0};
  bool served{true};
  std::string status;
};

struct TrafficLinkStepResult {
  int link_index{0};
  double inflow_vehicles{0.0};
  double occupancy_vehicles{0.0};
  double capacity_vehicles{0.0};
  double travel_time_hr{0.0};
  bool blocked{false};
};

struct ChargingSessionResult {
  int session_index{0};
  int station_id{0};
  std::vector<double> p_charge_kw;
  std::vector<double> p_discharge_kw;
  std::vector<double> energy_kwh;
  double unserved_energy_kwh{0.0};
};

struct StationStepResult {
  int station_id{0};
  int bus{0};
  double arrivals_vehicles{0.0};
  double requested_energy_kwh{0.0};
  double p_ev_kw{0.0};
  double q_ev_kvar{0.0};
  double reverse_power_kw{0.0};
  // Queue state (eqs. evpt-queue-dynamics, evpt-occupancy-dynamics, evpt-wait-approx)
  double queue_vehicles{0.0};   // q_{s,k}: vehicles waiting in queue
  double plug_occupancy{0.0};   // m_{s,k}: vehicles currently plugged in
  double waiting_time_hr{0.0};  // W_{s,k}: estimated queue waiting time
};

struct PowerValidationStep {
  int step_index{0};
  double total_base_load_mw{0.0};
  double total_ev_load_mw{0.0};
  double total_generation_capacity_mw{0.0};
  double capacity_margin_mw{0.0};
  bool generation_capacity_ok{true};
  bool power_flow_ran{false};
  bool power_flow_converged{false};
  double min_vm_pu{0.0};
};

struct EVPowerTrafficStepResult {
  int step_index{0};
  double hour{0.0};
  std::vector<TrafficLinkStepResult> traffic_links;
  std::vector<StationStepResult> stations;
  PowerValidationStep power;
};

}  // namespace hacdcpf::evpt

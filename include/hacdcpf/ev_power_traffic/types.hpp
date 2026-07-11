#pragma once

// ev_power_traffic/types.hpp
// ──────────────────────────────────────────────────────────────────────────
// Pure data-structure definitions for the EV power-traffic coupling layer.
// No dependency on the power-system model or solver headers; safe to include
// from any translation unit.

#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace hacdcpf::evpt {

// ── Vehicle class (Phase I: heterogeneous traffic) ──────────────────────

/// Vehicle class identifier for heterogeneous (multi-class) CTM.
/// Phase I extends the CTM to track ICVs and EVs separately while sharing
/// road capacity through passenger-car-equivalent (PCE) weighting.
enum class VehicleClass : int {
  ICV  = 0,  ///< Internal Combustion Vehicle
  EV   = 1,  ///< Battery Electric Vehicle
  PHEV = 2,  ///< Plug-in Hybrid Electric Vehicle
};

/// Per-class fundamental-diagram and cost parameters.
/// Used in CTMOptions to configure the multi-class CTM (Phase I).
struct VehicleClassParams {
  double pce{1.0};                            ///< γ^c — passenger-car-equivalent
  double free_flow_speed_override_km_hr{0.0}; ///< v^{f,c} override; 0 = use link default
  double fuel_cost_per_km{0.0};              ///< $/km fuel cost (ICV / PHEV)
  double value_of_time_per_hr{0.0};          ///< $/veh·hr VOT override; 0 = use global
};

/// Non-electric ICV demand: shares road capacity with EV demands.
/// ICVs route by minimum generalized cost (travel time + fuel cost);
/// they do not charge at stations and do not have SOC constraints.
/// Populated in EVPowerTrafficProblem::icv_demands (Phase I).
struct ICVDemand {
  int index{0};
  int origin_node{0};
  int destination_node{0};
  int departure_step{0};
  double vehicles{0.0};
  VehicleClass vehicle_class{VehicleClass::ICV};

  /// Candidate routes for this demand.  Empty = all OD-compatible routes.
  std::vector<int> candidate_route_indices;

  double value_of_time_per_hr{1.0};  ///< $/veh·hr
  double fuel_cost_per_km{0.0};      ///< $/km — added to generalized cost

  // ── Schedule delay (eq:icv-schedule-delay, eq:psi-icv) ───────────────
  /// t*_w — desired arrival time [hr].  Negative = not set (disables schedule delay).
  double desired_arrival_time_hr{-1.0};
  /// γ_e [$/hr] — penalty per hour of early arrival (t^arr < t*_w).
  double early_penalty_per_hr{0.0};
  /// γ_l [$/hr] — penalty per hour of late arrival (t^arr > t*_w).
  double late_penalty_per_hr{0.0};

  // ── Full DUE / endogenous departure time (DUEMode::FullEndogenous) ─────
  /// Candidate departure steps for this demand (K^{dep}_w in eq:full-due-set).
  /// When non-empty and DUEMode::FullEndogenous is active, the DUE solver
  /// distributes `vehicles` across these steps endogenously, minimising total
  /// generalised cost (schedule delay + travel time + charging cost).
  /// When empty, `departure_step` is used as a fixed departure (mode ii).
  std::vector<int> departure_window_steps;
};

// ── Assignment model selector ─────────────────────────────────────────────

enum class AssignmentModel {
  DeterministicShortestPath,
  Logit,
  CapacityAwareGreedy,
  SystemOptimalLP,
  SystemOptimalMILP,
  // Backward-compatible legacy alias for earlier examples.  This mode is a
  // capacity-aware greedy approximation, not an exact system-optimal solver.
  // New exact studies should use SystemOptimalLP or SystemOptimalMILP.
  SystemOptimal = CapacityAwareGreedy,
};

// ── Traffic network ───────────────────────────────────────────────────────

struct TrafficNode {
  int index{0};
  std::string name;
  /// Optional layout coordinates [km or arbitrary units] for visualization.
  /// NaN (default) = not set; renderers fall back to automatic layout.
  double x{std::numeric_limits<double>::quiet_NaN()};
  double y{std::numeric_limits<double>::quiet_NaN()};
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

  /// Per-class occupancy estimate [vehicles], keyed by static_cast<int>(VehicleClass).
  /// Populated when CTMOptions::enable_multiclass = true.
  std::unordered_map<int, double> class_occupancy_veh;
};

// Full CTM simulation result for one forward pass.
struct CTMSimulationResult {
  // Time discretization actually used for this pass.  dt_ctm_hr is the CTM
  // sub-step Δt_ctm; steps_per_sim_step is R = dt_sim / Δt_ctm (an exact
  // integer), so step_link_results.size() = T_sim · R and
  // T_ctm · Δt_ctm = T_sim · dt_sim (full horizon coverage).
  double dt_ctm_hr{0.0};
  int steps_per_sim_step{1};

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
  bool vi_certificate_available{false};
  bool mathematical_model_verified{false};
  std::string mathematical_model_verification_status{
      "not verified: DUE is certified by fixed-point/VI residuals, not a full solver proof"};
  double vi_gap{0.0};                 ///< c(h)^T (h - h_aon) at final CTM costs
  double normalized_vi_gap{0.0};      ///< vi_gap / max(1, c(h)^T h)
  double complementarity_max_violation{0.0}; ///< max used-path excess cost
  double demand_conservation_max_violation{0.0};
  /// ε^F — final ICV Wardrop relative gap (eq:icv-gap).
  double icv_relative_gap{0.0};
  /// ε^E — final EV Wardrop relative gap (eq:ev-gap).
  double ev_relative_gap{0.0};
  /// True when the VI certificate (vi_gap, complementarity_max_violation, etc.)
  /// was computed over EV flows only (problem.demands), not the full
  /// mixed-fleet flow including ICV demands.  Set to false when
  /// CTMDUEVIOptions::include_icv_in_due is false or icv_demands is empty.
  bool vi_certificate_covers_ev_only{false};
  /// True when the charging dispatch used a greedy heuristic rather than a
  /// monolithic optimal smart-charging/V2G LP solve.
  bool charging_dispatch_is_greedy{false};

  // Final route flows x_{d,r,k} after convergence.
  // Indexed: flow_by_demand_route[demand_index][route_index] = vehicles.
  std::unordered_map<int, std::unordered_map<int, double>> flow_by_demand_route;
  // Full-DUE disaggregate flows.
  // Indexed: flow_by_demand_departure_route[demand_index][departure_step][route_index].
  std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, double>>>
      flow_by_demand_departure_route;
  bool full_due_enabled{false};
  double departure_time_max_shift_veh{0.0};

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
  /// Battery energy exported via V2G [kWh] across all dispatched sessions.
  double total_v2g_energy_kwh{0.0};

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

  // ── Phase I: multi-class per-class stats ──────────────────────────────
  // Populated when CTMOptions::enable_multiclass = true and icv_demands non-empty.
  struct PerClassStats {
    VehicleClass vehicle_class{VehicleClass::EV};
    double total_demand{0.0};    ///< Σ_d vehicles [veh]
    double tstt_hr{0.0};         ///< total system travel time [hr·veh]
    double relative_gap{0.0};    ///< Wardrop relative gap at convergence
    bool   converged{false};
  };
  std::vector<PerClassStats> class_stats;

  /// ICV route flows (analogous to flow_by_demand_route for EV demands).
  /// flow_by_icv_route[icv_demand_index][route_index] = vehicles.
  std::unordered_map<int, std::unordered_map<int, double>> flow_by_icv_route;
};

// ── System-Optimal CTM LP result (Formulation E) ─────────────────────────

struct CTMSOLPResult {
  bool solved{false};
  bool infeasible{false};
  bool proven_optimal{false};
  bool mathematical_model_verified{false};
  std::string mathematical_model_verification_status{
      "not verified: wrapper certificate covers the assembled SO-CTM LP only"};
  bool dynamic_constraints_enforced{false};
  bool node_flow_variables_enforced{false};
  std::string status;
  double objective{0.0};   ///< LP objective value [veh·hr]
  double so_tstt_hr{0.0};  ///< total system travel time under SO-CTM [veh·hr]
  double best_bound{0.0};  ///< LP dual bound; equals objective when certified
  /// True when best_bound was set to the primal objective without an
  /// independent dual-feasibility check.  The solver (HiGHS / NativeSimplex)
  /// certifies optimality internally, but this wrapper does not re-verify dual
  /// feasibility from the certificate residuals alone.
  bool best_bound_is_primal_only{false};
  double mip_gap{0.0};     ///< zero for this pure LP
  double primal_max_violation{0.0};
  double integrality_max_violation{0.0};
  double node_flow_conservation_max_violation{0.0};
  /// True when use_aggregate_link_variables=false (default): per-route
  /// decomposition is used, which is a multi-commodity tighter relaxation
  /// of the single-commodity aggregate SO programme.
  bool per_route_decomposition_used{false};
  /// In CTM aggregate mode (use_aggregate_link_variables=true), internal
  /// boundary flows are shared (y_agg_int) and per-route OD entry flows are
  /// kept.  Closed-link availability now zeros both per-route entry/exit and
  /// shared internal flows, so this flag is true.  In per-route mode it is
  /// also true.  This documents that closed-link handling is rigorous.
  bool closed_link_handling_strict{true};
  int n_node_flow_variables{0};

  /// SO route flows: flow_by_demand_route[demand_index][route_index] = vehicles.
  std::unordered_map<int, std::unordered_map<int, double>> flow_by_demand_route;
};

// ── System-Optimal LTM LP result (Formulation E-LTM) ─────────────────────

struct LTMSOLPResult {
  bool solved{false};
  bool infeasible{false};
  bool proven_optimal{false};
  bool mathematical_model_verified{false};
  std::string mathematical_model_verification_status{
      "not verified: wrapper certificate covers the assembled SO-LTM LP only"};
  bool dynamic_constraints_enforced{false};
  bool node_flow_variables_enforced{false};
  std::string status;
  double objective{0.0};   ///< LP objective value [veh·hr]
  double so_tstt_hr{0.0};  ///< total system travel time under SO-LTM [veh·hr]
  double best_bound{0.0};  ///< LP dual bound; equals objective when certified
  /// True when best_bound was set to the primal objective without an
  /// independent dual-feasibility check.
  bool best_bound_is_primal_only{false};
  /// True when use_aggregate_link_variables=false (default): per-route
  /// decomposition is used.
  bool per_route_decomposition_used{false};
  /// In LTM aggregate mode, Nin_agg is now lower-bounded by the cumulative
  /// sum of upstream per-route Ynode movements arriving at each intermediate
  /// link (Bug 5b fix).  This pins intermediate-link Nin_agg correctly.
  bool aggregate_intermediate_conservation_strict{true};
  /// In LTM aggregate mode, the timing of per-route origin injection at
  /// entry links is NOT strictly enforced: only the cumulative total at
  /// horizon end (Nin_agg[entry][T] = Σ h_route) is constrained.  Per-step
  /// entry-link injection timing must be respected by the per-route mode.
  /// This flag is true only in per-route mode (use_aggregate_link_variables=false).
  bool aggregate_origin_timing_strict{false};
  double mip_gap{0.0};     ///< zero for this pure LP
  double primal_max_violation{0.0};
  double integrality_max_violation{0.0};
  double node_flow_conservation_max_violation{0.0};
  int n_node_flow_variables{0};
  std::unordered_map<int, std::unordered_map<int, double>> flow_by_demand_route;
};

// ── Route and charging stop definitions ───────────────────────────────────

struct RouteChargingStop {
  int station_id{0};
  double requested_energy_kwh_per_vehicle{0.0};
  /// E^dis_ω per vehicle [kWh] — planned V2G discharge energy at this stop.
  /// Used in route cost (eq:ev-energy-cost): C^energy subtracts π^dis · E^dis.
  /// Zero (default) disables the V2G revenue credit for this stop.
  double requested_discharge_energy_kwh_per_vehicle{0.0};
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

  /// e^res_v — mobility reserve energy [kWh] (eq:soc-feasibility).
  /// route_soc_feasible() enforces e_{v,k} - E^rem_{v,k} >= reserve_energy_kwh
  /// at every link traversal.  Zero (default) disables this check.
  double reserve_energy_kwh{0.0};

  // Gross willingness-to-pay per served vehicle ($/vehicle).
  // Used by solve_joint_social_welfare() to compute consumer benefit.
  // Zero means "not set"; only affects social welfare accounting.
  double willingness_to_pay_per_vehicle{0.0};

  // ── Phase I multi-class fields ──────────────────────────────────────
  VehicleClass vehicle_class{VehicleClass::EV};  ///< class tag (default EV)
  double value_of_time_per_hr{0.0};              ///< 0 = inherit global options

  // ── Schedule delay (eq:icv-schedule-delay, eq:psi-icv) ───────────────
  /// t*_w — desired arrival time [hr].  Negative = not set (disables schedule delay).
  double desired_arrival_time_hr{-1.0};
  /// γ_e [$/hr] — penalty per hour of early arrival (t^arr < t*_w).
  double early_penalty_per_hr{0.0};
  /// γ_l [$/hr] — penalty per hour of late arrival (t^arr > t*_w).
  double late_penalty_per_hr{0.0};

  // ── Full DUE / endogenous departure time (DUEMode::FullEndogenous) ─────
  /// Candidate departure steps for this demand (K^{dep}_w in eq:full-due-set).
  /// When non-empty and DUEMode::FullEndogenous is active, the DUE solver
  /// distributes `vehicles` across these steps endogenously.
  /// When empty, `departure_step` is used as a fixed departure (mode ii).
  std::vector<int> departure_window_steps;
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

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"

namespace hacdcpf::evpt {

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

struct EVPowerTrafficOptions {
  int num_steps{24};
  double time_step_hr{1.0};

  AssignmentModel assignment_model{AssignmentModel::DeterministicShortestPath};
  double logit_theta{1.0};
  bool enforce_road_capacity{true};
  bool allow_unserved_travel_demand{false};
  bool allow_unserved_charging_energy{false};
  bool allow_v2g{true};

  double value_of_time_per_hr{1.0};
  double queueing_weight{0.0};  // omega_Q: weight on W_{s,k} in generalized cost
                               // (eq. evpt-generalized-cost); 0 = disabled
  double station_energy_cost_weight{1.0};
  double default_station_price_per_kwh{0.30};
  double low_price_threshold_per_kwh{0.12};
  double high_price_threshold_per_kwh{0.45};
  double default_charging_efficiency{0.95};

  double default_route_stop_power_kw_per_vehicle{7.0};
  double default_station_power_kw{1.0e9};

  // Certified system-optimal LP/MILP solve controls.
  double system_optimal_unserved_trip_penalty{1.0e6};
  double system_optimal_mip_gap{1.0e-6};
  double system_optimal_time_limit_sec{30.0};
  int system_optimal_max_nodes{4096};

  bool update_system_charging_stations{true};
  bool enforce_generation_capacity{true};
  double external_grid_capacity_mw{0.0};
  bool run_power_flow_validation{false};
  PowerFlowOptions pf_options;
};

struct EVPowerTrafficProblem {
  HybridPowerSystem system;
  TrafficGraph traffic;
  std::vector<RouteAlternative> routes;
  std::vector<EVDemand> demands;
  std::vector<EVChargingSession> initial_sessions;
  std::vector<StationPriceProfile> station_prices;
};

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

struct EVPowerTrafficResult {
  bool feasible{false};
  std::string status;
  std::vector<std::string> warnings;

  double total_demand_vehicles{0.0};
  double served_demand_vehicles{0.0};
  double unmet_demand_vehicles{0.0};
  double total_requested_energy_kwh{0.0};
  double total_delivered_energy_kwh{0.0};
  double total_v2g_energy_kwh{0.0};
  double total_unserved_energy_kwh{0.0};
  double total_travel_time_hr{0.0};
  double total_assignment_cost{0.0};

  bool optimization_solved{false};
  bool optimization_is_mip{false};
  bool optimization_proven_optimal{false};
  std::string optimization_backend;
  std::string optimization_status;
  double optimization_objective{0.0};
  double optimization_best_bound{0.0};
  double optimization_mip_gap{0.0};

  std::vector<RouteAssignmentResult> assignments;
  std::vector<EVChargingSession> sessions;
  std::vector<ChargingSessionResult> session_results;
  std::vector<EVPowerTrafficStepResult> steps;

  HybridPowerSystem final_system;
  std::vector<HybridPowerSystem> system_by_step;
};

EVPowerTrafficResult simulate_ev_power_traffic(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options = {});

void apply_ev_station_loads(HybridPowerSystem& system,
                            const EVPowerTrafficStepResult& step);

// ──────────────────────────────────────────────────────────────────────────
// Joint Power-Traffic Social Welfare Maximisation
// ──────────────────────────────────────────────────────────────────────────
// Iteratively coordinates a DC-OPF (power side) with a system-optimal LP
// traffic assignment (traffic side) via locational marginal price (LMP)
// feedback.  Converges when nodal electricity prices at charging stations
// stabilise between iterations.
//
// Social welfare:  W = B_EV − C_gen − C_delay
//   B_EV   = Σ_d  WTP_d · served_d          (gross consumer benefit)
//   C_gen  = Σ_k  opf_objective_k           (generation dispatch cost)
//   C_delay= Σ_(d,r) VOT · travel_time · x  (traffic delay externality)

struct JointSocialWelfareOptions {
  // Iterative coordination parameters
  int max_iterations{30};
  double price_convergence_tol{1e-4};  ///< $/kWh; stop when max LMP change < tol
  double price_update_step{0.6};       ///< α in π^{new} = α·λ_{LMP} + (1-α)·π^{old}

  // When false the DC-OPF step is skipped and fixed default prices are used
  // (degenerates to a plain system-optimal LP with no power coupling).
  bool use_dcopf_prices{true};

  bool verbose{false};  ///< print per-iteration summary to stdout

  // Sub-problem options
  EVPowerTrafficOptions evpt_opts;
  opf::DCOPFOptions     dcopf_opts;
};

/// One row in the convergence history.
struct JointIterationRecord {
  int    iteration{0};
  double social_welfare{0.0};
  double ev_gross_benefit{0.0};   ///< Σ WTP_d · served_d
  double generation_cost{0.0};    ///< Σ_k opf_cost_k
  double traffic_delay_cost{0.0}; ///< VOT · Σ travel_time
  double price_change{0.0};       ///< max |π^{new} − π^{old}| over all stations/steps
  bool   opf_converged{false};
};

struct JointSocialWelfareResult {
  bool        converged{false};
  int         iterations{0};
  std::string status;

  // Social welfare at the final (converged / last) iteration
  double social_welfare{0.0};        ///< B_EV - C_gen - C_delay
  double ev_gross_benefit{0.0};      ///< Σ WTP_d · served_d
  double ev_consumer_surplus{0.0};   ///< Σ (WTP_d − price_paid_d) · served_d
  double generation_cost{0.0};       ///< total generation dispatch cost ($/step)
  double traffic_delay_cost{0.0};    ///< VOT · total travel time

  // Converged station prices ($/kWh) per step: [station_id][step]
  std::unordered_map<int, std::vector<double>> station_prices_per_step;

  // LMPs ($/MWh) at each charging-station bus, per step: [step][bus]
  std::vector<std::unordered_map<int, double>> lmp_by_step;

  // Final traffic assignment result
  EVPowerTrafficResult final_evpt;

  // DC-OPF result for each time step (empty when use_dcopf_prices=false)
  std::vector<opf::DCOPFResult> opf_by_step;

  // Per-iteration convergence history
  std::vector<JointIterationRecord> history;
};

JointSocialWelfareResult solve_joint_social_welfare(
    EVPowerTrafficProblem problem,   // taken by value; prices mutated internally
    const JointSocialWelfareOptions& options = {});

}  // namespace hacdcpf::evpt

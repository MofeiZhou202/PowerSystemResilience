#pragma once

// ev_power_traffic/options.hpp
// ──────────────────────────────────────────────────────────────────────────
// Simulation options for the EV power-traffic coupling layer.

#include "hacdcpf/ev_power_traffic/types.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"

namespace hacdcpf::evpt {

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

// ── CTM discretisation options ────────────────────────────────────────────
//
// Parameters that control the Daganzo Cell Transmission Model.
// Used only when simulate_ev_power_traffic_ctm_due() is called.

struct CTMOptions {
  // Time step for CTM propagation [hr].
  // Must satisfy CFL: dt_ctm <= min_a(delta_a / v^f_a).
  // Set to 0 to auto-select from link geometry and free-flow speeds.
  double dt_ctm_hr{0.0};

  // Number of CTM cells per link.  0 = auto-compute from
  //   M_a = max(1, floor(L_a / (v^f_a * dt_ctm_hr)))
  // so that each cell is exactly one free-flow time step.
  // Positive value forces the same cell count for every link.
  int n_cells_per_link{0};

  // Backward wave speed fallback [km/h] used when TrafficLink does not
  // specify jam_vehicles (triangular FD).
  // w = q_max * v_f / (k_jam * v_f - q_max)  -- derived from triangular FD.
  // If jam_vehicles == 0, the model infers k_jam from
  //   k_jam = q_max / v_f * (1 + v_f/w_fallback)
  double backward_wave_speed_fallback_km_hr{20.0};

  // Whether to store full per-cell per-step occupancy history.
  // Memory: O(T_ctm * n_links * n_cells * n_routes).  Disable for large networks.
  bool record_cell_history{false};

  // Use route-specific (commodity) cell occupancy to preserve station-choice
  // identity through diverge nodes.  If false, aggregate CTM is used with
  // precomputed turning fractions (faster but loses per-route station tracking).
  bool route_specific_cells{true};
};

// ── Dynamic User Equilibrium (DUE) options ────────────────────────────────

struct DUEOptions {
  // Maximum number of MSA/method-of-successive-averages iterations.
  int max_iterations{50};

  // Convergence: stop when relative Wardrop gap < tol.
  // gap = (system_cost_at_current_flows - UE_lower_bound) / UE_lower_bound
  double convergence_tol{1e-3};

  // MSA step size at iteration i: step = 1/i (standard MSA).
  // If msa_fixed_step > 0, use a fixed step size instead.
  double msa_fixed_step{0.0};

  // If true, after DUE convergence run one pass of the system-optimal LP
  // (Formulation A) to compute the price-of-anarchy gap.
  bool compute_system_optimal_benchmark{false};

  // Generalized cost weight on charging-price term (passed through to route cost).
  double logit_theta{0.0};  // 0 = deterministic shortest path; > 0 = stochastic
};

// ── Formulation C: CTM-DUE + DC-OPF iterative coordination ──────────────
//
// Outer price-feedback loop: runs CTM-DUE with current station prices,
// extracts EV charging loads, solves DC-OPF to obtain LMPs, and updates
// prices via exponential smoothing until convergence.

struct CTMJointWelfareOptions {
  // ── Outer iteration control ───────────────────────────────────────────

  /// Maximum number of outer (price-coordination) iterations.
  int max_iterations{30};

  /// Price convergence tolerance [$/kWh]:
  /// stop when max_s,k |π^{new}_{s,k} − π^{old}_{s,k}| < tol.
  double price_convergence_tol{1e-4};

  /// Step size α ∈ (0,1] in the price update rule:
  ///   π^{new} = α · λ_{LMP}/1000 + (1−α) · π^{old}
  /// (LMP in $/MWh; dividing by 1000 converts to $/kWh).
  double price_update_step{0.6};

  /// When false, the DC-OPF step is skipped and station prices remain fixed
  /// at the initial default.  The loop runs only one traffic iteration,
  /// equivalent to a plain CTM-DUE without power-system feedback.
  bool use_dcopf_prices{true};

  /// Print per-iteration summary to stdout.
  bool verbose{false};

  // ── Sub-problem options ───────────────────────────────────────────────
  EVPowerTrafficOptions evpt_opts;
  CTMOptions            ctm_opts;
  DUEOptions            due_opts;
  opf::DCOPFOptions     dcopf_opts;
};

// ── Formulation D: Single LP/QP/MILP/MCP joint optimizer ─────────────────
//
// Assembles route-assignment variables, EV charging loads, station power
// limits, and optional DC-OPF power balance into a single LP/MILP and
// solves it with a certified optimal solver.  Unlike the iterative
// Formulation C (CTM-DUE + DC-OPF price coordination), Formulation D
// produces a provably optimal social-welfare or user-benefit maximiser in
// a single solver call.
//
// Key differences from Formulations A/B/C:
//   A: routes only LP, no DC-OPF coupling in the LP
//   C: iterative CTM-DUE ↔ DC-OPF coordination (converged, not certified)
//   D: simultaneous routes + DC-OPF in one LP/MILP (solver-certified)
//
// Mathematical model — Eq. evpt-d-*  (technical notebook §Formulation D)

enum class JointOptimizerMode {
  SocialWelfareMax,    ///< max W = B_EV − C_gen − C_delay  (Eq. evpt-d-social-welfare)
  UserBenefitMax,      ///< max U = B_EV − elec_cost − C_delay (Eq. evpt-d-user-benefit)
  UserEquilibriumMILP, ///< Wardrop DUE via big-M complementarity (Eq. evpt-d-ue-milp)
};

struct JointOptimizerOptions {
  int    num_steps{6};
  double time_step_hr{1.0};

  JointOptimizerMode mode{JointOptimizerMode::SocialWelfareMax};

  /// Include DC-OPF power balance, branch flow, and generator dispatch
  /// constraints simultaneously with route assignment.  When false the LP
  /// reduces to a route-assignment-only formulation (same as Formulation A).
  bool include_dcopf{false};

  /// Include V2G discharge.  Currently reserved; route-level discharge
  /// models are post-processed after the LP solve.
  bool allow_v2g{false};

  /// Add binary δ^{ch}, δ^{dis} to prevent simultaneous charge/discharge.
  /// Omitting yields the LP relaxation (exact when η < 1 and c_deg ≥ 0).
  bool charge_discharge_binaries{false};

  // ── Road and station defaults (passed to the routing sub-problem) ────
  bool enforce_road_capacity{true};
  bool enforce_generation_capacity{false};

  // ── Penalty / big-M parameters ────────────────────────────────────────
  double unserved_trip_penalty{1.0e6};   ///< M_out [$/unserved vehicle]
  double power_slack_penalty{1.0e4};     ///< M_p   [$/MW power imbalance]
  double outside_option_cost{0.0};       ///< Ω     [$/cancelled trip — UB mode]

  // ── Objective weights ──────────────────────────────────────────────────
  double value_of_time_per_hr{1.0};        ///< VOT [$/veh·hr]
  double station_energy_cost_weight{1.0};  ///< ω_E

  /// Default charging efficiency (used when route stop does not specify one).
  double default_charging_efficiency{0.95};

  /// Default per-vehicle charging power [kW] (used when route stop omits it).
  double default_route_stop_power_kw_per_vehicle{7.0};

  /// Default station capacity [kW] (used when the power system model omits it).
  double default_station_power_kw{1.0e9};

  /// Default per-step electricity price used in UserBenefitMax when no
  /// explicit StationPriceProfile is attached to the problem.
  double default_station_price_per_kwh{0.03};

  // ── Solver controls ────────────────────────────────────────────────────
  double mip_gap{1e-6};
  double time_limit_sec{30.0};
  int    max_nodes{4096};
  bool   verbose{false};

  // ── Sub-problem options ────────────────────────────────────────────────
  opf::DCOPFOptions dcopf_opts;
};

}  // namespace hacdcpf::evpt

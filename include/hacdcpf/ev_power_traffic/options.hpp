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

  // ── Automatic K-shortest-path route generation ────────────────────────
  /// When true, generate_candidate_routes() is called automatically before
  /// the assignment solve to populate EVDemand::candidate_route_indices.
  /// When false (default), caller must supply routes explicitly.
  bool auto_generate_routes{false};

  /// K — number of shortest paths per OD pair to enumerate when
  /// auto_generate_routes = true.
  int k_shortest_paths{3};

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

  /// Require an equation-by-equation implementation and certificate for the
  /// full mathematical EV power-traffic model.  The current simulation entry
  /// point contains heuristic/iterative layers, so it rejects this request
  /// instead of returning a result that could be mistaken for a full proof.
  bool require_exact_mathematical_model{false};
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

  // ── Phase I: multi-class (heterogeneous vehicle) options ──────────────
  //
  // When enable_multiclass = true and EVPowerTrafficProblem::icv_demands is
  // non-empty, the CTM-DUE loop maintains separate EV and ICV route-flow
  // vectors and combines them with PCE weighting before the CTM forward pass.
  //
  // PCE-weighted aggregate occupancy:
  //   n_eff_{a,m,k} = gamma_ev * n^EV_{a,m,k} + gamma_icv * n^ICV_{a,m,k}
  //
  // Sending/receiving are evaluated on n_eff so that all vehicle classes
  // compete for the same physical road capacity.  Per-class route costs differ:
  //   ICV: VOT_icv * T^CTM_{r,k} + fuel_cost/km * d_r
  //   EV:  VOT_ev  * T^CTM_{r,k} + electricity_cost (unchanged)
  //
  // Single-class behavior is preserved when enable_multiclass = false.
  bool   enable_multiclass{false}; ///< activate PCE-weighted multi-class CTM
  double ev_pce{1.0};              ///< gamma^EV  — PCE factor for EVs
  double icv_pce{1.0};             ///< gamma^ICV — PCE factor for ICVs
  double phev_pce{1.0};            ///< gamma^PHEV — PCE factor for PHEVs

  // ── Aggregate (single-commodity) SO-CTM LP mode ───────────────────────
  //
  // When true, the SO-CTM LP (solve_ctm_so_lp) uses one shared cell-occupancy
  // variable n_{a,m,k} per link-cell-step for the internal CTM dynamics,
  // while retaining per-route entry/exit variables for OD accounting and node
  // movements.  This is an aggregate relaxation/approximation of the default
  // per-route formulation and reduces the LP size from O(|R|*|A|*M*T) to
  // O(|A|*M*T) state variables.
  bool use_aggregate_link_variables{false};
};

// ── Dynamic User Equilibrium (DUE) options ────────────────────────────────

/// Assignment mode selector for the DUE solver (eq:full-due / eq:fixed-due).
enum class DUEMode {
  /// Mode (ii): departure_step is exogenous — equilibrate route choice only.
  /// This is the default and has been fully implemented and validated.
  FixedDeparture,
  /// Mode (i): departure time is endogenous — outer MSA loop reallocates
  /// vehicles across all steps in EVDemand::departure_window_steps while
  /// the inner loop equilibrates route choice at each departure step.
  /// Vehicles with an empty departure_window_steps fall back to FixedDeparture.
  FullEndogenous,
};

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

  // ── Full DUE (endogenous departure time) ──────────────────────────────
  /// Selects between fixed-departure (mode ii) and full endogenous departure
  /// (mode i) DUE.  Full mode requires non-empty departure_window_steps in
  /// the demand objects.  Default: FixedDeparture (backward-compatible).
  DUEMode due_mode{DUEMode::FixedDeparture};

  /// Maximum number of outer departure-time reallocation iterations.
  /// Only used when due_mode == DUEMode::FullEndogenous.
  int max_departure_iterations{30};

  /// Convergence tolerance for the departure-time Wardrop gap.
  /// Stop when max_{w,k} |x_{w,k}^{dep,(i)} - x_{w,k}^{dep,(i-1)}| / D_w < tol.
  double departure_convergence_tol{1e-3};

  /// Require an equation-by-equation implementation and certificate for the
  /// full DUE mathematical model.  Current CTM-DUE paths are fixed-point/
  /// residual solvers, not globally certified VI/MPEC solves, so they reject
  /// this request.
  bool require_exact_mathematical_model{false};
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

// ── Formulation D: Single LP/MILP joint optimizer ────────────────────────
//
// Assembles route-assignment variables, aggregate EV charging/discharging
// schedules, battery-energy constraints, station power limits, time-varying
// road-capacity profiles, and optional DC-OPF power balance into one solver
// call.  The default SocialWelfare/UserBenefit modes are LP/MILP models with
// exogenous route costs.  CertifiedDynamic*MPECMILP additionally assembles a
// finite linear dynamic MPEC with endogenous affine congestion and Wardrop
// big-M complementarity.  FullJoint*NLP assembles a nonlinear reduced MPEC but
// has only local stationarity certificates.
//
// Key differences from Formulations A/B/C:
//   A: routes only LP, no DC-OPF coupling in the LP
//   C: iterative CTM-DUE ↔ DC-OPF coordination (converged, not certified)
//   D: simultaneous routes + charging/V2G + DC-OPF in one LP/MILP
//      (solver-certified; no MSA/greedy repair)
//   D-certified: finite dynamic MILP/MPEC with affine congestion + Wardrop
//      big-M rows and solver-certified global MIP gap for that finite model
//
// Mathematical model — Eq. evpt-d-*  (technical notebook §Formulation D)

enum class JointOptimizerMode {
  SocialWelfareMax,    ///< max W = B_EV − C_gen − C_delay  (Eq. evpt-d-social-welfare)
  UserBenefitMax,      ///< max U = B_EV − elec_cost + V2G revenue − C_delay − Ω·u
  IntegerRouteMILP,    ///< Integer route/unserved flows solved by MILP;
                       ///< \b not Wardrop big-M complementarity.
  CertifiedDynamicMPECMILP, ///< Time-expanded linear dynamic traffic +
                            ///< charging/V2G + DC-OPF + Wardrop big-M MPEC
                            ///< with social-welfare objective, solved as one
                            ///< globally certified MILP.
  CertifiedDynamicUserBenefitMPECMILP, ///< Same certified finite MILP/MPEC,
                                       ///< using private user-benefit costs.
  CertifiedFullJointLtmPwlMILP, ///< Time-expanded LTM cumulative-count
                                ///< dynamics + charging/V2G + DC-OPF +
                                ///< Wardrop big-M equilibrium in one global
                                ///< MILP. Nonlinear travel-cost terms are
                                ///< replaced by documented PWL envelopes with
                                ///< reported approximation-error bounds.
  CertifiedFullJointLtmUserBenefitPwlMILP, ///< Same LTM-PWL MILP using the
                                           ///< private user-benefit objective.
  FullJointSocialWelfareNLP, ///< Monolithic nonlinear MPEC/NLP with Wardrop
                             ///< NCP rows; local stationarity certificate only.
  FullJointUserBenefitNLP,   ///< Same monolithic nonlinear MPEC/NLP, using the
                             ///< private user-benefit objective.
  UserEquilibriumMILP = IntegerRouteMILP, ///< Backward-compatible legacy alias.
};

struct JointOptimizerOptions {
  int    num_steps{6};
  double time_step_hr{1.0};

  JointOptimizerMode mode{JointOptimizerMode::SocialWelfareMax};

  /// Include DC-OPF power balance, branch flow, and generator dispatch
  /// constraints simultaneously with route assignment.  When false the LP
  /// reduces to a route-assignment-only formulation (same as Formulation A).
  bool include_dcopf{false};

  /// Include V2G discharge variables \c p_dis in the Formulation-D LP.
  /// When false, all discharge columns have zero upper bound.  When true,
  /// route stops with \c v2g_capable=true and positive
  /// \c max_discharge_kw_per_vehicle may discharge subject to the aggregate
  /// battery-energy equations and station discharge limit.
  /// Simultaneous charge/discharge at the same step is never cost-optimal
  /// (Proposition 1 in the technical notebook), so no binary mode variables
  /// are needed; the model remains a pure LP.
  bool allow_v2g{false};

  // ── Road and station defaults (passed to the routing sub-problem) ────
  bool enforce_road_capacity{true};
  bool enforce_generation_capacity{false};

  // ── Penalty / big-M parameters ────────────────────────────────────────
  double unserved_trip_penalty{1.0e6};   ///< M_out [$/unserved vehicle]
  double power_slack_penalty{1.0e4};     ///< M_p   [$/MW power imbalance]
  double outside_option_cost{0.0};       ///< Ω     [$/cancelled trip, added in UB mode]

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

  // ── Dynamic travel times (FormD + CTM coupling) ───────────────────────
  /// When true, run one CTM forward pass (with initial uniform route flows)
  /// before the LP/MILP is constructed and use the CTM-derived route travel
  /// times instead of free-flow times.  This is the "dynamic joint optimizer"
  /// mode described in the technical notebook §Formulation D (dynamic).
  /// When false (default), free-flow travel times are used (static mode).
  bool use_ctm_travel_times{false};

  /// CTM options used when use_ctm_travel_times = true.
  CTMOptions ctm_opts;

  // ── Monolithic nonlinear full-joint mode ───────────────────────────────
  /// Request Ipopt for FullJoint* NLP modes.  The current implementation keeps
  /// this reduced MPEC on the in-process sparse NativeNLP path so derivative
  /// verification is deterministic; both Ipopt and NativeNLP would be local
  /// NLP solvers, not global MPEC/MINLP proof engines.
  bool full_joint_prefer_ipopt{false};

  /// Finite-difference step used only by the optional sparse derivative
  /// checker for the assembled nonlinear MPEC/NLP.  Solver callbacks use
  /// analytic sparse gradient/Jacobian routines.
  double full_joint_fd_step{1e-6};

  /// Additional quadratic penalty on Wardrop NCP residuals in the NLP
  /// objective.  The residuals are also explicit equality rows; this penalty
  /// improves local convergence without replacing the equations.
  double full_joint_equilibrium_penalty{10.0};

  /// Number of breakpoints used by CertifiedFullJointLtm*PwlMILP for convex
  /// piecewise-linear BPR travel-cost envelopes.  The implemented envelope
  /// is globally certifiable for the assembled PWL-MILP approximation; the
  /// original nonlinear travel-cost error is reported in the result.
  int full_joint_ltm_pwl_segments{6};

  /// Fallback backward-wave speed used by the LTM-PWL MILP when a link does
  /// not provide enough information to infer it from the triangular FD.
  double full_joint_ltm_backward_wave_speed_fallback_km_hr{20.0};

  /// Verify the sparse analytic gradient/Jacobian used by FullJoint* NLP
  /// against finite differences at the assembled initial point.
  bool full_joint_verify_sparse_derivatives{false};

  /// Tolerance used by the sparse derivative checker.
  double full_joint_derivative_check_tol{1e-4};

  /// If true, return infeasible/unsupported instead of solving a local NLP
  /// whenever the caller asks for a globally exact nonlinear MPEC certificate.
  /// Current in-tree solvers do not provide such a certificate for this model.
  bool require_global_nonlinear_certificate{false};

  /// Stronger paper-audit switch: require Formulation D to be the complete
  /// mathematical joint power-traffic model with endogenous congestion,
  /// charging/V2G, DC-OPF, equilibrium conditions, and a valid global
  /// optimality certificate.  The certified dynamic MILP mode satisfies this
  /// only for its documented finite, linear time-expanded model; the nonlinear
  /// FullJoint* NLP modes still return unsupported under this switch.
  bool require_exact_mathematical_model{false};
};

// ── Formulation E-LTM: System-Optimal LTM LP options ─────────────────────

struct LTMSOLPOptions {
  /// Fallback backward wave speed [km/hr] when link value is zero.
  double backward_wave_speed_fallback_km_hr{20.0};

  /// When true, the SO-LTM LP uses aggregate per-link Nin/Nout variables
  /// (one per link-step) for the link sending/receiving envelope instead of
  /// per-route cumulative count variables.  Per-route route-choice variables
  /// h_{d,r} and node movements are kept for OD accounting.  This is a
  /// reduced aggregate mode, not a full per-route FIFO certificate.  Reduces LP
  /// size from O(|R|*|A|*T) to O(|A|*T) state variables.
  bool use_aggregate_link_variables{false};

  /// Require the exact route-commodity SO-LTM mathematical model.  Aggregate
  /// mode is rejected under this switch because it is a reduced relaxation.
  bool require_exact_mathematical_model{false};
};

// ── Formulation F: DUE-VI / Frank-Wolfe options ───────────────────────────

struct CTMDUEVIOptions {
  int    max_iterations{50};
  double convergence_tol{1e-3};
  bool   compute_so_benchmark{false};  ///< also run SO-CTM LP for PoA
  /// Number of golden-section bisection steps for the one-dimensional
  /// Frank-Wolfe line search.  Each step adds one extra CTM forward pass.  Set to 0 to fall
  /// back to the classical MSA step 1/(iter+1).
  int    line_search_steps{8};

  /// When true and EVPowerTrafficProblem::icv_demands is non-empty, the
  /// Frank-Wolfe DUE-VI loop runs a mixed-fleet equilibrium: ICV vehicles are
  /// assigned by minimum generalised cost (VOT·TT + fuel_cost·dist) using the
  /// same Frank-Wolfe descent step as EVs.  The shared CTM forward pass uses
  /// PCE-weighted aggregate occupancy (CTMOptions::icv_pce) so that both
  /// fleets compete for the same physical capacity (eq:mixed-fleet-due-vi).
  /// Ignored when enable_multiclass = false in the accompanying CTMOptions.
  bool include_icv_in_due{false};

  /// Require a full mixed-fleet VI certificate, not just EV VI residuals plus
  /// separate ICV Wardrop gaps.  Current code returns unsupported when this is
  /// requested together with ICV participation.
  bool require_exact_mathematical_model{false};
};

// ── Infrastructure design MILP options ───────────────────────────────────

struct InfraDesignMILPOptions {
  double capital_cost_per_plug{5000.0};   ///< $/plug
  double operating_cost_scale{1.0};       ///< multiplier on charging energy cost
  int    max_plugs_per_station{20};       ///< B̄^max_s
  bool   include_road_capacity{true};
  /// Enforce a time-expanded, LTM-style dynamic road-capacity envelope in the
  /// design MILP: route flow can enter a downstream link only after upstream
  /// free-flow lag, and each link-step has capacity and availability bounds.
  bool   include_ltm_dynamic_constraints{false};
  double mip_gap{1e-3};
  double time_limit_sec{120.0};

  /// Require the exact infrastructure-design objective and dynamic model from
  /// the notebook.  The current MILP contains documented piecewise-linear and
  /// optional dynamic-envelope approximations, so it rejects this request.
  bool require_exact_mathematical_model{false};
};

// ── LTM-MPC options ───────────────────────────────────────────────────────

struct LTMMPCOptions {
  int    horizon_steps{6};        ///< prediction horizon H
  double weight_queue{1.0};       ///< ω_Q — station queue penalty
  double weight_travel_time{1.0}; ///< ω_T — travel time penalty
  double weight_v2g{0.5};         ///< ω_V2G — V2G revenue weight
  /// When true, station access-link inflow is bounded by an exogenous forecast
  /// derived from OD demand and route charging stops. This prevents the MPC
  /// from creating artificial arrivals merely to improve the objective.
  bool   enforce_forecast_arrivals{true};

  /// When true, the LTM-MPC builds Nin/Nout LTM variables for every link on
  /// the route leading to each station (not just the final access link), and
  /// the VHT objective ω_T·dt·(Nin[a,j]−Nout[a,j]) is summed over ALL tracked
  /// route links.  The admission constraint b[s,j] ≤ ΔNout[access_link,j]
  /// still uses only the final access link for physical correctness.
  /// Implements the documented tracked-link MPC proxy for
  /// eq:mpc-ltm-obj / eq:mpc-ltm-cap-send / eq:mpc-ltm-cap-recv; it remains
  /// driven by the exogenous OD/station arrival forecast.
  bool   track_all_route_links{false};

  /// Require full-network endogenous LTM-MPC equivalence.  The implemented MPC
  /// remains forecast-driven and receding-horizon, so it rejects this request.
  bool   require_exact_mathematical_model{false};
};

}  // namespace hacdcpf::evpt

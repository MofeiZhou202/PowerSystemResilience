#pragma once

// ev_power_traffic/simulation.hpp
// ──────────────────────────────────────────────────────────────────────────
// EVPowerTrafficProblem, EVPowerTrafficResult, and the public entry
// points for the EV power-traffic simulation.

#include <unordered_map>
#include <vector>
#include <string>

#include "hacdcpf/ev_power_traffic/options.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::evpt {

// ── Problem definition ────────────────────────────────────────────────────

struct EVPowerTrafficProblem {
  HybridPowerSystem system;
  TrafficGraph traffic;
  std::vector<RouteAlternative> routes;
  std::vector<EVDemand> demands;
  std::vector<EVChargingSession> initial_sessions;
  std::vector<StationPriceProfile> station_prices;

  /// Phase I: heterogeneous ICV demand sharing road capacity with EVs.
  /// Populated when CTMOptions::enable_multiclass = true; ignored otherwise.
  std::vector<ICVDemand> icv_demands;
};

// ── Aggregate result ──────────────────────────────────────────────────────

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

// ── Public entry points ───────────────────────────────────────────────────

EVPowerTrafficResult simulate_ev_power_traffic(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options = {});

// CTM-based propagation with Dynamic User Equilibrium (DUE).
//
// Uses Daganzo's Cell Transmission Model for traffic dynamics and
// Method of Successive Averages (MSA) to find the dynamic user equilibrium.
// Route-specific (commodity) CTM preserves station-choice identity through
// diverge nodes.  Optional DC-OPF coupling is inherited from `ev_options`.
//
// Returns both the DUE route-flow solution and the final CTM simulation result
// (station arrivals, experienced travel times, cell occupancy history).
CTMDUEResult simulate_ev_power_traffic_ctm_due(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_options = {},
    const CTMOptions& ctm_options = {},
    const DUEOptions& due_options = {});

void apply_ev_station_loads(HybridPowerSystem& system,
                            const EVPowerTrafficStepResult& step);

// ── Formulation C: CTM-DUE jointly coupled with DC-OPF ───────────────────
//
// Iteratively coordinates the CTM-DUE (Formulation B+) inner loop with
// DC-OPF locational marginal prices (LMPs):
//
//   For i = 1..max_iterations:
//     (a) Run CTM-DUE with current station prices → route flows,
//         session synthesis, smart-charging dispatch, queue model.
//     (b) For each step k: inject EV loads into the power system;
//         solve DC-OPF; extract LMPs \lambda_{b,k}.
//     (c) Update prices: \pi^{new}_{s,k} = \alpha * \lambda_{bus(s),k}/1000
//                                         + (1-\alpha) * \pi^{old}_{s,k}
//     (d) Convergence: max |\pi^{new} - \pi^{old}| < price_convergence_tol.
//
// Social welfare:  W = B_EV - C_gen - C_delay
//   B_EV   = \sum_d WTP_d * served_d                (consumer benefit)
//   C_gen  = \sum_k OPF_objective_k                 (generation dispatch cost)
//   C_delay= VOT * TSTT^{CTM}                        (congestion externality)

/// Per-iteration record for Formulation C convergence history.
struct CTMJointIterationRecord {
  int    iteration{0};
  double social_welfare{0.0};
  double ev_gross_benefit{0.0};    ///< \sum_d WTP_d * served_d
  double generation_cost{0.0};     ///< \sum_k OPF cost
  double traffic_delay_cost{0.0};  ///< VOT * TSTT^{CTM}
  double price_change{0.0};        ///< max |\pi^{new} - \pi^{old}|
  bool   opf_converged{false};
  bool   due_converged{false};
  double due_gap{0.0};             ///< Wardrop relative gap at this iteration
};

/// Full result of the Formulation C joint CTM-DUE + DC-OPF simulation.
struct CTMJointWelfareResult {
  bool        converged{false};    ///< true when price_change < tol
  int         iterations{0};
  std::string status;

  // Social welfare at the final (converged / last) iteration
  double social_welfare{0.0};      ///< B_EV - C_gen - C_delay
  double ev_gross_benefit{0.0};    ///< \sum_d WTP_d * served_d
  double generation_cost{0.0};     ///< total OPF dispatch cost ($)
  double traffic_delay_cost{0.0};  ///< VOT * TSTT^{CTM}

  /// Converged station prices [$/kWh] per sim step: [station_id][step].
  std::unordered_map<int, std::vector<double>> station_prices_per_step;

  /// LMPs [$/MWh] at each bus, per sim step: [step][bus_index].
  std::vector<std::unordered_map<int, double>> lmp_by_step;

  /// Final CTM-DUE result (converged or last iteration).
  CTMDUEResult final_ctm_due;

  /// DC-OPF result for each time step (empty when use_dcopf_prices=false).
  std::vector<opf::DCOPFResult> opf_by_step;

  /// Per-iteration convergence history.
  std::vector<CTMJointIterationRecord> history;
};

/// Formulation C entry point.
CTMJointWelfareResult simulate_ev_power_traffic_ctm_joint(
    EVPowerTrafficProblem problem,   // taken by value; prices mutated internally
    const CTMJointWelfareOptions& options = {});

// ── Formulation D: Single LP/MILP joint optimizer ────────────────────────
//
// Simultaneously optimises route-assignment, EV charge/discharge schedules,
// aggregate battery energy, time-varying road capacity, station limits, and
// (optionally) generator dispatch + power balance in a single LP/MILP.
// Returns a solver-certified optimal solution — no iterative coordination.
//
// LP structure  (see §Formulation D in the technical notebook)
// ──────────────────────────────────────────────────────────────
//   Variables: h_{d,r}  (route flow), u_d (unserved),
//              p_ch, p_dis, e for aggregate route-stop charging groups,
//              optional δ_ch/δ_dis mode binaries,
//              P^g_{g,k}, θ_{b,k}, f_{ℓ,k}, ℓ^p_{b,k}  [if include_dcopf]
//
//   IntegerRouteMILP mode enforces integral route/unserved variables only;
//   it is not a Wardrop MCP/KKT or big-M complementarity formulation.
//
//   Equality:   demand balance, DC flow, DC power balance
//   Inequality: road capacity, station capacity
//   Variable bounds: P^g ∈ [Pmin,Pmax], f ∈ [-Fmax,Fmax], ℓ^p ≥ 0
//
//   SocialWelfareMax (default):
//     min Σ c_g·P^g·Δt − Σ WTP·h + VOT·TSTT + M_out·u + M_p·ℓ^p
//
//   UserBenefitMax:
//     min Σ π_s·(p_ch−p_dis)·Δt − Σ WTP·h + VOT·TSTT + (M_out+Ω)·u

struct JointOptimizerResult {
  // ── Solver certificate ─────────────────────────────────────────────────
  bool   feasible{false};
  bool   proven_optimal{false};
  std::string solver_backend;   ///< "NativeDualSimplex" / "HiGHS" / "NativeB&C"
  std::string solver_status;
  double objective{0.0};        ///< solver minimization objective value
  double best_bound{0.0};       ///< dual bound (= objective for LP; best_bound ≥ objective for MILP)
  double mip_gap{0.0};
  double primal_max_violation{0.0};      ///< max bound/equality/inequality violation
  double integrality_max_violation{0.0}; ///< max distance to nearest integer/binary value
  double solve_time_sec{0.0};
  int    n_variables{0};
  int    n_constraints{0};

  // ── Welfare decomposition ──────────────────────────────────────────────
  double social_welfare{0.0};        ///< W = B_EV − C_gen − C_delay
  double ev_benefit{0.0};            ///< B_EV = Σ WTP_d · served_d
  double gen_cost{0.0};              ///< C_gen = Σ_{g,k} c_g · P^g · Δt
  double traffic_delay_cost{0.0};    ///< C_delay = VOT · TSTT (free-flow)

  // ── Route assignment ───────────────────────────────────────────────────
  double total_served_vehicles{0.0};
  double total_unserved_vehicles{0.0};
  double total_travel_time_hr{0.0};
  double total_assignment_cost{0.0};

  /// route_flow[demand_index][route_index] = vehicles
  std::unordered_map<int, std::unordered_map<int, double>> route_flow;

  /// Flat assignment list (compatible with EVPowerTrafficResult::assignments).
  std::vector<RouteAssignmentResult> assignments;

  // ── Charging and energy ────────────────────────────────────────────────
  double total_requested_energy_kwh{0.0};
  double total_delivered_energy_kwh{0.0};
  double total_v2g_energy_kwh{0.0};
  double total_unserved_energy_kwh{0.0};

  std::vector<EVChargingSession>     sessions;
  std::vector<ChargingSessionResult> session_results;

  // ── Power system (populated when include_dcopf = true) ────────────────
  /// Generator dispatch [MW] per step: gen_dispatch[step][gen_index].
  std::vector<std::unordered_map<int, double>> gen_dispatch_mw;

  /// LMPs [$/MWh] per step: lmp_by_step[step][bus_index].
  std::vector<std::unordered_map<int, double>> lmp_by_step;

  std::vector<std::string> warnings;
};

/// Formulation D entry point — single LP/MILP assembly.
/// Assembles route flows, aggregate charge/discharge schedules, battery
/// energy, station capacity, and optionally DC-OPF power balance into one
/// mathematical program; solves with HiGHS or native LP/MILP.
JointOptimizerResult solve_joint_optimizer(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& options = {});

}  // namespace hacdcpf::evpt

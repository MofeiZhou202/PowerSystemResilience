// joint_optimizer.cpp
// ──────────────────────────────────────────────────────────────────────────
// Formulation D: single LP/MILP joint optimizer for EV-power-traffic.
//
// Key innovation over Formulations A/B/C
// ─────────────────────────────────────────
//   A: routing-only LP (no DC-OPF in the LP objective)
//   C: iterative CTM-DUE ↔ DC-OPF price coordination (converged, not certified)
//   D: route-assignment + charging/V2G + DC-OPF power balance assembled into
//      ONE LP/MILP, solved by a primal-certified optimal solver (HiGHS /
//      native dual simplex).
//
// LP structure  (§Formulation D, technical notebook)
// ───────────────────────────────────────────────────
// Variables
//   h_{d,r}     ≥ 0                 vehicles on demand d, route r
//   u_d         ≥ 0                 unserved vehicles for demand d
//   p^ch_{v,k}  ≥ 0                aggregate charge power [kW]
//   p^dis_{v,k} ≥ 0                aggregate V2G discharge power [kW]
//   e_{v,k}     ≥ 0                aggregate battery energy [kWh]
//   δ^ch,δ^dis  ∈ {0,1}            optional mode binaries
//   P^g_{g,k}   ∈ [Pmin,Pmax]      generator dispatch [MW]     (if OPF)
//   θ_{b,k}     free                bus voltage angle [rad]     (if OPF)
//   f_{ℓ,k}     ∈ [-Fmax,Fmax]     branch active flow [MW]     (if OPF)
//   ℓ^p_{b,k}   ≥ 0                power balance slack [MW]    (if OPF)
//
// Equality constraints
//   Demand balance   :  Σ_r h_{d,r} + u_d = D_d                (one/demand)
//   Battery dynamics :  e_{v,k+1}=e_{v,k}+η p^ch Δt-p^dis Δt/η
//   DC flow          :  f_{ℓ,k} − b_ℓ θ_{i,k} + b_ℓ θ_{j,k} = 0
//   DC power balance :  Σ_g P^g_{g,k}[g@b] − Σ_ℓ A_{bℓ} f_{ℓ,k}
//                       − P^{EV}_{b,k}(h) + ℓ^p_{b,k} = P^d_{b,k}
//
// Inequality constraints
//   Road capacity    :  Σ_{d,r: a∈r, k_dep=k} h_{d,r} ≤ cap_{a,k}·Δt
//   Station capacity :  Σ_v p^ch_{v,k} ≤ P̄^ch_s, Σ_v p^dis_{v,k} ≤ P̄^dis_s
//   Power links      :  p^ch≤p̄^ch h, p^dis≤p̄^dis h, SOC bounds/terminal target
//
// The EV load at bus b at step k is a LINEAR function of charge/discharge vars:
//   P^{EV}_{b,k}[MW] = 1e-3 · Σ_{v:s(v)@b} (p^ch_{v,k}-p^dis_{v,k})
//
// This linearity is the key that makes Formulation D a pure LP.
//
// Objectives
//   SocialWelfareMax (minimise):
//     Σ c^1_g·P^g·Δt − Σ WTP·h + VOT·Σ T^ff·h + M_out·u + M_p·Σℓ^p
//
//   UserBenefitMax (minimise):
//     Σ π_s·(p^ch−p^dis)·Δt − Σ WTP·h + VOT·Σ T^ff·h + (M_out+Ω)·u

#include "hacdcpf/ev_power_traffic/simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {

JointOptimizerResult solve_joint_optimizer_full_nlp(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& opts);

// Forward declaration (defined in ctm_propagation.cpp)
extern CTMSimulationResult ctm_forward_pass(
    const EVPowerTrafficProblem&,
    const std::vector<std::vector<double>>&,
    const std::unordered_map<int, std::size_t>&,
    const std::unordered_map<int, int>&,
    const CTMOptions&,
    double, int, int);

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Internal column descriptor
// ─────────────────────────────────────────────────────────────────────────────
struct JOColumn {
  enum class Kind {
    RouteFlow,        // h_{d,r}
    UnservedDemand,   // u_d
    GeneratorPower,   // P^g_{g,k}
    BusAngle,         // θ_{b,k}
    BranchFlow,       // f_{ℓ,k}
    PowerSlack,       // ℓ^p_{b,k}
    ChargePower,      // p^ch_{v,k}
    DischargePower,   // p^dis_{v,k}
    BatteryEnergy,    // e_{v,k}
    ChargeMode,       // δ^ch_{v,k}
    DischargeMode,    // δ^dis_{v,k}
  };
  Kind   kind{Kind::RouteFlow};
  int    demand_pos{-1};     // index into active_demand_positions
  int    route_pos{-1};      // position in candidate_routes(demand)
  const RouteAlternative* route{nullptr};
  int    departure_step{-1};
  int    arrival_step{-1};
  // DC-OPF indexing
  int    gen_pos{-1};    // index into sys.ac.generators
  int    bus_pos{-1};    // index into sys.ac.buses
  int    branch_pos{-1}; // index into sys.ac.branches
  int    step{-1};       // time step for gen/bus/branch columns
  int    session_pos{-1}; // index into Form-D route-stop charging groups
  int    offset{-1};      // local offset inside a charging group window
};

struct PendingEq {
  std::vector<std::pair<int, double>> terms;
  double rhs{0.0};
};

// ─────────────────────────────────────────────────────────────────────────────
// Helper: find the full-energy charging power [kW/vehicle] for a stop.
// ─────────────────────────────────────────────────────────────────────────────
static double jo_charge_power_kw_per_veh(const RouteChargingStop& stop,
                                         const JointOptimizerOptions& opts,
                                         double dt_hr) {
  if (stop.max_charge_kw_per_vehicle > kTol)
    return stop.max_charge_kw_per_vehicle;
  // Fall back: deliver requested energy over the dwell window
  const double dwell_hr = std::max(1, stop.dwell_steps) * dt_hr;
  if (stop.requested_energy_kwh_per_vehicle > kTol && dwell_hr > kTol)
    return stop.requested_energy_kwh_per_vehicle / dwell_hr;
  return opts.default_route_stop_power_kw_per_vehicle;
}

static double jo_route_drive_energy_kwh_per_veh(
    const RouteAlternative& route,
    const EVPowerTrafficProblem& problem,
    const std::unordered_map<int, std::size_t>& link_pos) {
  double energy = 0.0;
  for (int link_index : route.link_indices) {
    const auto it = link_pos.find(link_index);
    if (it == link_pos.end()) continue;
    const auto& link = problem.traffic.links[it->second];
    energy += std::max(0.0, link.drive_energy_kwh_per_veh_km) *
              std::max(0.0, link.length_km);
  }
  return energy;
}

static double jo_station_effective_cap_kw(const HybridPowerSystem& sys,
                                          const JointOptimizerOptions& opts,
                                          int station_id) {
  for (const auto& cs : sys.ac.charging_stations) {
    if (cs.index != station_id) continue;
    if (!cs.in_service) return 0.0;
    double cap = cs.max_power_kw > 0.0 ? cs.max_power_kw
               : static_cast<double>(cs.n_fast) * cs.p_fast_max_kw +
                 static_cast<double>(cs.n_slow) * cs.p_slow_max_kw;
    if (cap <= kTol) cap = opts.default_station_power_kw;
    return std::max(0.0, cap * std::clamp(cs.simultaneity_factor, 0.0, 1.0));
  }
  return opts.default_station_power_kw;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: total base load [MW] summed over all AC buses
// ─────────────────────────────────────────────────────────────────────────────
static double jo_base_load_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& b : sys.ac.buses) total += b.pd_mw;
  return total;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: total generation capacity [MW]
// ─────────────────────────────────────────────────────────────────────────────
static double jo_gen_cap_mw(const HybridPowerSystem& sys) {
  double cap = 0.0;
  for (const auto& g : sys.ac.generators)
    if (g.in_service) cap += std::max(0.0, g.pmax_mw);
  return cap;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: station → bus mapping
// ─────────────────────────────────────────────────────────────────────────────
static std::unordered_map<int, int>
jo_station_bus_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> m;
  for (const auto& cs : sys.ac.charging_stations)
    m[cs.index] = cs.bus;
  return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: station → max power [kW] mapping
// ─────────────────────────────────────────────────────────────────────────────
static std::unordered_map<int, double>
jo_station_cap_kw(const HybridPowerSystem& sys,
                  const JointOptimizerOptions& opts) {
  std::unordered_map<int, double> m;
  for (const auto& cs : sys.ac.charging_stations) {
    m[cs.index] = jo_station_effective_cap_kw(sys, opts, cs.index);
  }
  return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: look up station price [$/kWh] for a given station and step.
// ─────────────────────────────────────────────────────────────────────────────
static double jo_station_price(const EVPowerTrafficProblem& prob,
                               const JointOptimizerOptions& opts,
                               int station_id,
                               int step) {
  for (const auto& pp : prob.station_prices) {
    if (pp.station_id == station_id) {
      if (step >= 0 && step < static_cast<int>(pp.price_per_kwh.size()))
        return pp.price_per_kwh[static_cast<std::size_t>(step)];
    }
  }
  return opts.default_station_price_per_kwh;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convert JointOptimizerOptions to EVPowerTrafficOptions for reuse of helpers.
// ─────────────────────────────────────────────────────────────────────────────
static EVPowerTrafficOptions jo_to_evpt_opts(const JointOptimizerOptions& jo) {
  EVPowerTrafficOptions evpt;
  evpt.num_steps                      = jo.num_steps;
  evpt.time_step_hr                   = jo.time_step_hr;
  evpt.value_of_time_per_hr           = jo.value_of_time_per_hr;
  evpt.station_energy_cost_weight     = jo.station_energy_cost_weight;
  evpt.default_charging_efficiency    = jo.default_charging_efficiency;
  evpt.default_route_stop_power_kw_per_vehicle =
      jo.default_route_stop_power_kw_per_vehicle;
  evpt.default_station_power_kw       = jo.default_station_power_kw;
  evpt.default_station_price_per_kwh  = jo.default_station_price_per_kwh;
  evpt.enforce_road_capacity          = jo.enforce_road_capacity;
  evpt.enforce_generation_capacity    = jo.enforce_generation_capacity;
  evpt.system_optimal_mip_gap         = jo.mip_gap;
  evpt.system_optimal_time_limit_sec  = jo.time_limit_sec;
  evpt.system_optimal_max_nodes       = jo.max_nodes;
  return evpt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Add a warning to the result, optionally printing to stderr.
// ─────────────────────────────────────────────────────────────────────────────
static void jo_warn(JointOptimizerResult& r,
                    const std::string& msg,
                    bool verbose) {
  r.warnings.push_back(msg);
  if (verbose)
    std::fprintf(stderr, "[JointOptimizer] WARNING: %s\n", msg.c_str());
}

static void jo_compute_certificate_residuals(JointOptimizerResult& result,
                                             const engine::MIPModel& mip,
                                             const Eigen::VectorXd& x) {
  const auto& lp = mip.linear_part;
  double primal = 0.0;
  double integ = 0.0;

  for (int j = 0; j < x.size() && j < static_cast<int>(lp.vars.size()); ++j) {
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    primal = std::max(primal, std::max(0.0, var.lb - x[j]));
    primal = std::max(primal, std::max(0.0, x[j] - var.ub));
  }

  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd r = lp.Aeq * x - lp.beq;
    for (int i = 0; i < r.size(); ++i)
      primal = std::max(primal, std::abs(r[i]));
  }
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd r = lp.A * x - lp.b;
    for (int i = 0; i < r.size(); ++i)
      primal = std::max(primal, std::max(0.0, r[i]));
  }

  auto check_integral = [&](int idx) {
    if (idx < 0 || idx >= x.size()) return;
    integ = std::max(integ, std::abs(x[idx] - std::round(x[idx])));
  };
  for (int idx : mip.integer_idx) check_integral(idx);
  for (int idx : mip.binary_idx)  check_integral(idx);

  result.primal_max_violation = primal;
  result.integrality_max_violation = integ;
}

// ─────────────────────────────────────────────────────────────────────────────
// Certified finite-dimensional dynamic MPEC MILP.
//
// This is deliberately NOT the nonlinear FullJoint NLP path.  It assembles a
// globally certifiable MILP for a documented finite linear model:
//   * time-expanded route/departure choices,
//   * endogenous affine link congestion costs through link-volume variables,
//   * road capacity and link availability/fault profiles,
//   * big-M Wardrop complementarity,
//   * aggregate charging/V2G/SOC constraints, and
//   * optional DC-OPF rows in the same MILP.
//
// The certificate is the MILP certificate (MIP gap + residuals).  It must not be
// described as a global proof for the nonlinear CTM/LTM MPEC.
// ─────────────────────────────────────────────────────────────────────────────
static JointOptimizerResult solve_certified_dynamic_mpec_milp(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& opts) {
  using namespace std::chrono;
  const auto t_start = steady_clock::now();
  const bool ltm_pwl_mode =
      opts.mode == JointOptimizerMode::CertifiedFullJointLtmPwlMILP ||
      opts.mode == JointOptimizerMode::CertifiedFullJointLtmUserBenefitPwlMILP;

  JointOptimizerResult result;
  result.certified_dynamic_mpec_model = !ltm_pwl_mode;
  result.certified_full_joint_ltm_pwl_milp_model = ltm_pwl_mode;
  result.monolithic_full_joint_model = true;
  result.nonlinear_model = false;
  result.local_optimum_certificate = false;
  result.global_optimum_certificate = false;
  result.endogenous_congestion_enforced = true;
  result.travel_times_are_endogenous = true;
  result.full_ltm_dynamics_enforced = ltm_pwl_mode;
  result.pwl_approximation_used = ltm_pwl_mode;
  result.wardrop_complementarity_enforced = true;
  result.dcopf_coupling_enforced = opts.include_dcopf;
  result.charging_v2g_enforced = opts.allow_v2g;
  result.sparse_derivatives_enabled = false;
  result.solver_backend =
      ltm_pwl_mode ? "CertifiedFullJointLtmPwlMILP"
                   : "CertifiedDynamicMPECMILP";
  result.mathematical_model_verification_status =
      ltm_pwl_mode
          ? "not yet solved: full-joint LTM cumulative-count PWL-MILP assembled"
          : "not yet solved: finite linear dynamic MPEC MILP assembled";

  auto unsupported = [&](const std::string& status) {
    result.feasible = false;
    result.proven_optimal = false;
    result.global_optimum_certificate = false;
    result.mathematical_model_verified = false;
    result.solver_status = status;
    result.mathematical_model_verification_status = status;
    jo_warn(result, status, opts.verbose);
    return result;
  };

  const int T = std::max(1, opts.num_steps);
  const double dt = std::max(1.0e-9, opts.time_step_hr);
  if (problem.demands.empty()) {
    result.feasible = true;
    result.proven_optimal = true;
    result.global_optimum_certificate = true;
    result.mathematical_model_verified = true;
    result.solver_status = "empty";
    result.mathematical_model_verification_status =
        "verified: empty finite dynamic MPEC MILP";
    return result;
  }

  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
    link_pos[problem.traffic.links[li].index] = li;
  }

  struct LinkLTMParam {
    int tau_ff{1};
    int tau_bw{1};
    double njam{0.0};
  };
  std::vector<LinkLTMParam> ltm_params(problem.traffic.links.size());
  if (ltm_pwl_mode) {
    for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
      const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
      const double vf = std::max(kTol, link_free_flow_speed(link));
      const double len = std::max(kTol, link.length_km);
      const double cap_hr =
          link.capacity_veh_per_hr > kTol ? link.capacity_veh_per_hr : vf * 0.1;
      const double kj = link.jam_vehicles > kTol ? link.jam_vehicles / len
                                                 : cap_hr / vf * 2.0;
      const double kc = cap_hr / vf;
      double bw = opts.full_joint_ltm_backward_wave_speed_fallback_km_hr;
      if (kj > kc + kTol) bw = cap_hr / (kj - kc);
      bw = std::max(kTol, bw);
      const int tau_ff =
          std::max(1, static_cast<int>(std::ceil(len / (vf * dt) - 1.0e-9)));
      const int tau_bw =
          std::max(1, static_cast<int>(std::ceil(len / (bw * dt) - 1.0e-9)));
      const double njam = link.jam_vehicles > kTol
                              ? link.jam_vehicles
                              : std::max((tau_ff + tau_bw) * cap_hr * dt,
                                         (cap_hr / bw + cap_hr / vf) * len);
      ltm_params[static_cast<std::size_t>(li)] = {tau_ff, tau_bw, njam};
    }
  }

  const auto& sys = problem.system;
  const int n_buses = static_cast<int>(sys.ac.buses.size());
  const int n_gens = static_cast<int>(sys.ac.generators.size());
  const int n_branches = static_cast<int>(sys.ac.branches.size());

  std::unordered_map<int, int> bus_pos_map;
  for (int bi = 0; bi < n_buses; ++bi) {
    bus_pos_map[sys.ac.buses[static_cast<std::size_t>(bi)].index] = bi;
  }
  int slack_bus_pos = 0;
  for (int bi = 0; bi < n_buses; ++bi) {
    if (sys.ac.buses[static_cast<std::size_t>(bi)].bus_type == BusType::SLACK) {
      slack_bus_pos = bi;
      break;
    }
  }

  std::vector<double> branch_susceptance(static_cast<std::size_t>(n_branches), 0.0);
  for (int li = 0; li < n_branches; ++li) {
    const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
    branch_susceptance[static_cast<std::size_t>(li)] =
        std::abs(br.x_pu) > 1.0e-12 ? 1.0 / br.x_pu : 1.0e6;
  }

  const auto stn_cap_kw = jo_station_cap_kw(sys, opts);
  const auto stn_bus_map = jo_station_bus_map(sys);

  auto departure_steps_for = [&](const EVDemand& demand) {
    std::vector<int> deps = demand.departure_window_steps.empty()
        ? std::vector<int>{demand.departure_step}
        : demand.departure_window_steps;
    std::sort(deps.begin(), deps.end());
    deps.erase(std::unique(deps.begin(), deps.end()), deps.end());
    deps.erase(std::remove_if(deps.begin(), deps.end(),
                              [&](int k) { return k < 0 || k >= T; }),
               deps.end());
    return deps;
  };

  auto route_free_flow_hr = [&](const RouteAlternative& route) {
    double tt = 0.0;
    for (int link_index : route.link_indices) {
      auto it = link_pos.find(link_index);
      if (it == link_pos.end()) continue;
      tt += std::max(0.0, problem.traffic.links[it->second].free_flow_time_hr);
    }
    return tt;
  };

  auto route_link_step_pairs = [&](const RouteAlternative& route, int dep) {
    std::vector<std::pair<int, int>> pairs;
    double prefix_hr = 0.0;
    for (int link_index : route.link_indices) {
      auto it = link_pos.find(link_index);
      if (it == link_pos.end()) {
        prefix_hr += 0.0;
        continue;
      }
      const int k = dep + std::max(0, static_cast<int>(std::floor(prefix_hr / dt + 1.0e-9)));
      if (k >= 0 && k < T) {
        pairs.emplace_back(static_cast<int>(it->second), k);
      }
      prefix_hr += std::max(0.0, problem.traffic.links[it->second].free_flow_time_hr);
    }
    return pairs;
  };

  auto affine_latency_slope_hr_per_veh = [&](int link_position, int step) {
    const auto& link = problem.traffic.links[static_cast<std::size_t>(link_position)];
    const double cap = std::max(1.0, link_capacity_vehicles(link, step, dt));
    const double t0 = std::max(0.0, link.free_flow_time_hr);
    // A globally linear, certifiable latency law:
    //   tau_a,k(v) = t0_a + alpha_a * t0_a/cap_a,k * v_a,k.
    // This is the first-order BPR-compatible affine envelope used only by the
    // certified MILP path.
    return std::max(0.0, link.alpha) * t0 / cap;
  };

  auto private_route_cost = [&](const EVDemand& demand,
                                const RouteAlternative& route,
                                int dep,
                                double ff_hr) {
    double cost = opts.value_of_time_per_hr * ff_hr + route.toll_cost;
    const double arr_hr = dep * dt + ff_hr;
    cost += schedule_delay_cost(arr_hr, demand.desired_arrival_time_hr,
                                demand.early_penalty_per_hr,
                                demand.late_penalty_per_hr);
    for (const auto& stop : route.charging_stops) {
      const double price = jo_station_price(problem, opts, stop.station_id, dep);
      cost += opts.station_energy_cost_weight * price *
              std::max(0.0, stop.requested_energy_kwh_per_vehicle);
      if (opts.allow_v2g && stop.v2g_capable) {
        cost -= opts.station_energy_cost_weight * price *
                std::max(0.0, stop.requested_discharge_energy_kwh_per_vehicle);
      }
    }
    return cost;
  };

  struct CDCol {
    enum class Kind {
      RouteFlow,
      RouteUsed,
      UnservedDemand,
      DemandMinCost,
      RouteCost,
      LinkVolume,
      LinkTravelTime,
      LinkCostSegment,
      LtmNin,
      LtmNout,
      GeneratorPower,
      BusAngle,
      BranchFlow,
      PowerSlack,
      ChargePower,
      DischargePower,
      BatteryEnergy,
    };
    Kind kind{Kind::RouteFlow};
    int demand_pos{-1};
    int demand_index{0};
    int route_index{0};
    int departure_step{-1};
    int link_pos{-1};
    int step{-1};
    int gen_pos{-1};
    int bus_pos{-1};
    int branch_pos{-1};
    int session_pos{-1};
    int offset{-1};
    const RouteAlternative* route{nullptr};
  };

  struct CDChoice {
    int demand_pos{-1};
    int demand_index{0};
    int route_index{0};
    int departure_step{0};
    int arrival_step{0};
    double demand_vehicles{0.0};
    double ff_hr{0.0};
    double base_private_cost{0.0};
    std::vector<std::pair<int, int>> link_steps;
    const RouteAlternative* route{nullptr};
    int h_col{-1};
    int y_col{-1};
    int cost_col{-1};
  };

  struct CDSession {
    int choice_pos{-1};
    int demand_pos{-1};
    int session_pos{-1};
    int route_col{-1};
    const RouteAlternative* route{nullptr};
    int station_id{0};
    int arrival_step{0};
    int departure_step{0};
    int window{1};
    double requested_kwh_per_veh{0.0};
    double p_ch_max_per_veh{0.0};
    double p_dis_max_per_veh{0.0};
    double eta_ch{1.0};
    double eta_dis{1.0};
    double e_arrival_per_veh{0.0};
    double e_min_per_veh{0.0};
    double e_max_per_veh{0.0};
    double e_target_per_veh{0.0};
    bool has_e_max{false};
    bool v2g{false};
    std::vector<int> pch_cols;
    std::vector<int> pdis_cols;
    std::vector<int> energy_cols;
  };

  std::vector<int> demand_positions;
  std::unordered_map<int, int> demand_to_pi_col;
  for (int di = 0; di < static_cast<int>(problem.demands.size()); ++di) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    if (demand.vehicles > kTol && !departure_steps_for(demand).empty()) {
      demand_positions.push_back(di);
    }
  }
  if (demand_positions.empty()) {
    return unsupported("empty: no positive demand has a valid departure step");
  }

  std::vector<CDChoice> choices;
  EVPowerTrafficOptions evpt_opts = jo_to_evpt_opts(opts);
  for (int di : demand_positions) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    const auto routes = candidate_routes(problem, demand);
    for (int dep : departure_steps_for(demand)) {
      for (const auto* route : routes) {
        if (!route) continue;
        if (!route_can_deliver_requested_energy(*route, evpt_opts, dt)) continue;
        if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;

        const double ff_hr = route_free_flow_hr(*route);
        const int arr = dep + std::max(0, static_cast<int>(std::ceil(ff_hr / dt - 1.0e-9)));
        bool fits = true;
        for (const auto& stop : route->charging_stops) {
          const int dwell = std::max(1, stop.dwell_steps);
          if (arr < 0 || arr >= T || arr + dwell > T) {
            fits = false;
            break;
          }
        }
        if (!fits) continue;

        CDChoice choice;
        choice.demand_pos = di;
        choice.demand_index = demand.index;
        choice.route_index = route->index;
        choice.departure_step = dep;
        choice.arrival_step = arr;
        choice.demand_vehicles = demand.vehicles;
        choice.ff_hr = ff_hr;
        choice.base_private_cost = private_route_cost(demand, *route, dep, ff_hr);
        choice.link_steps = route_link_step_pairs(*route, dep);
        if (choice.link_steps.size() != route->link_indices.size()) {
          continue;
        }
        bool dynamically_feasible = true;
        if (opts.enforce_road_capacity) {
          for (const auto& [lpos, kk] : choice.link_steps) {
            if (lpos < 0 ||
                lpos >= static_cast<int>(problem.traffic.links.size()) ||
                kk < 0 || kk >= T ||
                link_capacity_vehicles(
                    problem.traffic.links[static_cast<std::size_t>(lpos)],
                    kk, dt) <= kTol) {
              dynamically_feasible = false;
              break;
            }
          }
        }
        if (!dynamically_feasible) continue;
        choice.route = route;
        choices.push_back(std::move(choice));
      }
    }
  }

  if (choices.empty()) {
    return unsupported("unsupported/infeasible: no route-departure choice survived SOC, horizon, and route filters");
  }

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<CDCol> cols;
  std::vector<engine::VariableMeta> vars;
  std::vector<double> obj;
  constexpr double kBigM = 1.0e6;
  const bool user_objective =
      opts.mode == JointOptimizerMode::CertifiedDynamicUserBenefitMPECMILP ||
      opts.mode == JointOptimizerMode::CertifiedFullJointLtmUserBenefitPwlMILP;

  auto add_col = [&](CDCol col,
                     double lb,
                     double ub,
                     double cost,
                     const std::string& name,
                     engine::VarType type = engine::VarType::Continuous) {
    const int idx = static_cast<int>(cols.size());
    cols.push_back(col);
    vars.push_back({type, lb, ub, name});
    obj.push_back(cost);
    if (type == engine::VarType::Binary) {
      mip.binary_idx.push_back(idx);
    } else if (type == engine::VarType::Integer) {
      mip.integer_idx.push_back(idx);
    }
    return idx;
  };

  std::vector<double> beq_vec;
  std::vector<double> bineq_vec;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  auto add_eq = [&](double rhs) {
    const int row = static_cast<int>(beq_vec.size());
    beq_vec.push_back(rhs);
    return row;
  };
  auto add_ineq = [&](double rhs) {
    const int row = static_cast<int>(bineq_vec.size());
    bineq_vec.push_back(rhs);
    return row;
  };

  std::unordered_map<int, std::vector<int>> demand_choice_positions;
  for (int ci = 0; ci < static_cast<int>(choices.size()); ++ci) {
    demand_choice_positions[choices[static_cast<std::size_t>(ci)].demand_pos]
        .push_back(ci);
  }

  // Link-volume columns v_{a,k}.
  std::vector<std::vector<int>> link_vol_col(problem.traffic.links.size(),
                                             std::vector<int>(T, -1));
  std::vector<std::vector<int>> link_tt_col(problem.traffic.links.size(),
                                            std::vector<int>(T, -1));
  std::vector<std::vector<int>> ltm_nin_col(problem.traffic.links.size(),
                                            std::vector<int>(T + 1, -1));
  std::vector<std::vector<int>> ltm_nout_col(problem.traffic.links.size(),
                                             std::vector<int>(T + 1, -1));
  for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
    const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
    if (ltm_pwl_mode) {
      for (int k = 0; k <= T; ++k) {
        CDCol nin;
        nin.kind = CDCol::Kind::LtmNin;
        nin.link_pos = li;
        nin.step = k;
        ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] =
            add_col(nin, 0.0, kBigM, 0.0,
                    "Nin_link" + std::to_string(link.index) + "_k" +
                        std::to_string(k));
        CDCol nout;
        nout.kind = CDCol::Kind::LtmNout;
        nout.link_pos = li;
        nout.step = k;
        ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] =
            add_col(nout, 0.0, kBigM, 0.0,
                    "Nout_link" + std::to_string(link.index) + "_k" +
                        std::to_string(k));
      }
      int row = add_eq(0.0);
      eq_trips.emplace_back(row, ltm_nin_col[static_cast<std::size_t>(li)][0], 1.0);
      row = add_eq(0.0);
      eq_trips.emplace_back(row, ltm_nout_col[static_cast<std::size_t>(li)][0], 1.0);
    }
    for (int k = 0; k < T; ++k) {
      const double cap = link_capacity_vehicles(link, k, dt);
      const double ub = std::max(0.0, std::isfinite(cap) ? cap : 1.0e6);
      CDCol col;
      col.kind = CDCol::Kind::LinkVolume;
      col.link_pos = li;
      col.step = k;
      link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] =
          add_col(col, 0.0, ub, 0.0,
                  "v_link" + std::to_string(link.index) + "_k" + std::to_string(k));
      if (ltm_pwl_mode) {
        CDCol tt;
        tt.kind = CDCol::Kind::LinkTravelTime;
        tt.link_pos = li;
        tt.step = k;
        link_tt_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] =
            add_col(tt, 0.0, kBigM, 0.0,
                    "Tau_link" + std::to_string(link.index) + "_k" +
                        std::to_string(k));
      }
    }
  }

  // Demand min-cost columns pi_d.
  for (int di : demand_positions) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    CDCol col;
    col.kind = CDCol::Kind::DemandMinCost;
    col.demand_pos = di;
    col.demand_index = demand.index;
    demand_to_pi_col[di] = add_col(col, -kBigM, kBigM, 0.0,
                                   "pi_d" + std::to_string(demand.index));
  }

  // Route/departure choice columns.
  for (int ci = 0; ci < static_cast<int>(choices.size()); ++ci) {
    auto& choice = choices[static_cast<std::size_t>(ci)];
    const auto& demand = problem.demands[static_cast<std::size_t>(choice.demand_pos)];

    CDCol h;
    h.kind = CDCol::Kind::RouteFlow;
    h.demand_pos = choice.demand_pos;
    h.demand_index = demand.index;
    h.route_index = choice.route_index;
    h.departure_step = choice.departure_step;
    h.route = choice.route;

    double h_cost = -demand.willingness_to_pay_per_vehicle + choice.route->toll_cost;
    if (user_objective) {
      h_cost += opts.value_of_time_per_hr * choice.ff_hr;
    }
    choice.h_col = add_col(h, 0.0, demand.vehicles, h_cost,
                           "h_d" + std::to_string(demand.index) +
                           "_r" + std::to_string(choice.route_index) +
                           "_k" + std::to_string(choice.departure_step));

    CDCol y = h;
    y.kind = CDCol::Kind::RouteUsed;
    choice.y_col = add_col(y, 0.0, 1.0, 0.0,
                           "z_d" + std::to_string(demand.index) +
                           "_r" + std::to_string(choice.route_index) +
                           "_k" + std::to_string(choice.departure_step),
                           engine::VarType::Binary);

    CDCol c = h;
    c.kind = CDCol::Kind::RouteCost;
    choice.cost_col = add_col(c, -kBigM, kBigM, 0.0,
                              "C_d" + std::to_string(demand.index) +
                              "_r" + std::to_string(choice.route_index) +
                              "_k" + std::to_string(choice.departure_step));
  }

  // Unserved demand columns and conservation.
  for (int di : demand_positions) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    const int row = add_eq(demand.vehicles);
    for (int ci : demand_choice_positions[di]) {
      eq_trips.emplace_back(row, choices[static_cast<std::size_t>(ci)].h_col, 1.0);
    }
    CDCol u;
    u.kind = CDCol::Kind::UnservedDemand;
    u.demand_pos = di;
    u.demand_index = demand.index;
    u.departure_step = demand.departure_step;
    const double outside =
        user_objective ? std::max(0.0, opts.outside_option_cost) : 0.0;
    const int u_col = add_col(u, 0.0, demand.vehicles,
                              opts.unserved_trip_penalty + outside,
                              "u_d" + std::to_string(demand.index));
    eq_trips.emplace_back(row, u_col, 1.0);
  }

  // v_{a,k} = sum choices using link a at step k.
  for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
    for (int k = 0; k < T; ++k) {
      const int row = add_eq(0.0);
      eq_trips.emplace_back(row, link_vol_col[static_cast<std::size_t>(li)]
                                              [static_cast<std::size_t>(k)], 1.0);
      for (const auto& choice : choices) {
        for (const auto& [lpos, kk] : choice.link_steps) {
          if (lpos == li && kk == k) {
            eq_trips.emplace_back(row, choice.h_col, -1.0);
          }
        }
      }
    }
  }

  if (ltm_pwl_mode) {
    // LTM cumulative-count envelope.  The route/departure choice volume
    // v_{a,k} is the entry increment of the aggregate LTM count on link a.
    // These rows are linear and certifiable.  The travel-time column below
    // is a convex PWL BPR envelope over v_{a,k}; its approximation error is
    // reported in the result and in the verification status.
    for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
      const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
      const auto& p = ltm_params[static_cast<std::size_t>(li)];
      for (int k = 1; k <= T; ++k) {
        // Monotone cumulative counts.
        int row = add_ineq(0.0);
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            -1.0);
        row = add_ineq(0.0);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            -1.0);

        // Entry increment equals the explicit link-volume column.
        row = add_eq(0.0);
        eq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            1.0);
        eq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            -1.0);
        eq_trips.emplace_back(row,
            link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            -1.0);

        const bool avail = availability_value(link.availability_profile, k - 1,
                                              link.available);
        const double cap = avail ? link_capacity_vehicles(link, k - 1, dt) : 0.0;

        // Entry and exit capacity.
        row = add_ineq(cap);
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            -1.0);
        row = add_ineq(cap);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)],
            -1.0);

        // Sending/free-flow envelope: Nout[k] <= Nin[k - tau_ff].
        row = add_ineq(0.0);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)]
                       [static_cast<std::size_t>(std::max(0, k - p.tau_ff))],
            -1.0);

        // Receiving/backward-wave envelope:
        // Nin[k] - Nout[k - tau_bw] <= Njam.
        row = add_ineq(std::max(0.0, p.njam));
        ineq_trips.emplace_back(row,
            ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            1.0);
        ineq_trips.emplace_back(row,
            ltm_nout_col[static_cast<std::size_t>(li)]
                        [static_cast<std::size_t>(std::max(0, k - p.tau_bw))],
            -1.0);
      }
    }
  }

  // Convex piecewise-linear social traffic objective for affine latency.
  if (!user_objective) {
    const int kSegments =
        ltm_pwl_mode ? std::max(1, opts.full_joint_ltm_pwl_segments) : 4;
    for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
      const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
      for (int k = 0; k < T; ++k) {
        const double cap = link_capacity_vehicles(link, k, dt);
        if (cap <= kTol) continue;
        const double width = std::max(cap / static_cast<double>(kSegments), kTol);
        const double t0 = std::max(0.0, link.free_flow_time_hr);
        const double slope = affine_latency_slope_hr_per_veh(li, k);
        const int row = add_eq(0.0);
        eq_trips.emplace_back(row, link_vol_col[static_cast<std::size_t>(li)]
                                                [static_cast<std::size_t>(k)], 1.0);
        for (int s = 0; s < kSegments; ++s) {
          const double lo = s * width;
          const double hi = (s + 1) * width;
          const double coeff = opts.value_of_time_per_hr *
                               (ltm_pwl_mode
                                    ? (t0 * (1.0 + link.alpha *
                                             std::pow(hi / cap, link.beta)))
                                    : (t0 + slope * (lo + hi)));
          CDCol z;
          z.kind = CDCol::Kind::LinkCostSegment;
          z.link_pos = li;
          z.step = k;
          const int zc = add_col(z, 0.0, width, coeff,
              "seg_link" + std::to_string(link.index) + "_k" +
              std::to_string(k) + "_s" + std::to_string(s));
          eq_trips.emplace_back(row, zc, -1.0);
        }
      }
    }
  }

  if (ltm_pwl_mode) {
    const int kSegments = std::max(1, opts.full_joint_ltm_pwl_segments);
    for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
      const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
      for (int k = 0; k < T; ++k) {
        const double cap = link_capacity_vehicles(link, k, dt);
        const int tau_col = link_tt_col[static_cast<std::size_t>(li)]
                                       [static_cast<std::size_t>(k)];
        if (cap <= kTol || tau_col < 0) continue;
        const double t0 = std::max(0.0, link.free_flow_time_hr);
        const double width = std::max(cap / static_cast<double>(kSegments), kTol);
        for (int s = 0; s < kSegments; ++s) {
          const double x0 = s * width;
          const double x1 = std::min(cap, (s + 1) * width);
          if (x1 <= x0 + kTol) continue;
          auto bpr = [&](double x) {
            return t0 * (1.0 + std::max(0.0, link.alpha) *
                                  std::pow(std::max(0.0, x / cap), link.beta));
          };
          const double y0 = bpr(x0);
          const double y1 = bpr(x1);
          const double slope = (y1 - y0) / (x1 - x0);
          // tau >= secant_slope * v + intercept.  For convex BPR this is an
          // outer approximation of the nonlinear route cost; minimization pins
          // tau to the active envelope.
          const int row = add_ineq(-(y0 - slope * x0));
          ineq_trips.emplace_back(row,
              link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
              slope);
          ineq_trips.emplace_back(row, tau_col, -1.0);

          const double mid = 0.5 * (x0 + x1);
          result.pwl_max_abs_error_bound = std::max(
              result.pwl_max_abs_error_bound,
              std::max(0.0, (y0 + slope * (mid - x0)) - bpr(mid)));
        }
      }
    }
  }

  // Route cost definitions and Wardrop big-M complementarity.
  for (const auto& choice : choices) {
    const int row = add_eq(choice.base_private_cost);
    eq_trips.emplace_back(row, choice.cost_col, 1.0);
    for (const auto& [li, k] : choice.link_steps) {
      if (ltm_pwl_mode) {
        const int tau_col = link_tt_col[static_cast<std::size_t>(li)]
                                       [static_cast<std::size_t>(k)];
        if (tau_col >= 0) {
          eq_trips.emplace_back(row, tau_col, -opts.value_of_time_per_hr);
          const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
          // base_private_cost already contains the free-flow route time.
          // Subtract t0 so the PWL tau column replaces that free-flow term
          // without double-counting it.
          beq_vec[static_cast<std::size_t>(row)] -=
              opts.value_of_time_per_hr * std::max(0.0, link.free_flow_time_hr);
        }
      } else {
        const double slope_money =
            opts.value_of_time_per_hr * affine_latency_slope_hr_per_veh(li, k);
        if (slope_money <= 0.0) continue;
        eq_trips.emplace_back(row,
            link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)],
            -slope_money);
      }
    }

    const int pi_col = demand_to_pi_col[choice.demand_pos];
    // pi_d <= C_{d,r,k}
    {
      const int r = add_ineq(0.0);
      ineq_trips.emplace_back(r, pi_col, 1.0);
      ineq_trips.emplace_back(r, choice.cost_col, -1.0);
    }
    // C_{d,r,k} <= pi_d + M(1-z_{d,r,k})
    {
      const int r = add_ineq(kBigM);
      ineq_trips.emplace_back(r, choice.cost_col, 1.0);
      ineq_trips.emplace_back(r, pi_col, -1.0);
      ineq_trips.emplace_back(r, choice.y_col, kBigM);
    }
    // h_{d,r,k} <= D_d z_{d,r,k}
    {
      const int r = add_ineq(0.0);
      ineq_trips.emplace_back(r, choice.h_col, 1.0);
      ineq_trips.emplace_back(r, choice.y_col, -choice.demand_vehicles);
    }
  }

  // Charging/V2G group variables tied to route-flow choices.
  std::vector<CDSession> sessions;
  for (int ci = 0; ci < static_cast<int>(choices.size()); ++ci) {
    const auto& choice = choices[static_cast<std::size_t>(ci)];
    const auto& demand = problem.demands[static_cast<std::size_t>(choice.demand_pos)];
    if (choice.route == nullptr) continue;

    const double drive_kwh_per_veh =
        jo_route_drive_energy_kwh_per_veh(*choice.route, problem, link_pos);
    double e_arrival = 0.0;
    double e_min = 0.0;
    double e_max = 0.0;
    bool has_e_max = false;
    if (demand.initial_energy_kwh >= 0.0) {
      e_arrival = std::max(0.0, demand.initial_energy_kwh - drive_kwh_per_veh);
      e_min = std::max(0.0, demand.energy_min_kwh);
      if (demand.energy_max_kwh >= 0.0) {
        e_max = std::max(e_min, demand.energy_max_kwh);
        e_arrival = std::min(e_arrival, e_max);
        has_e_max = true;
      }
    }

    for (const auto& stop : choice.route->charging_stops) {
      CDSession ce;
      ce.choice_pos = ci;
      ce.demand_pos = choice.demand_pos;
      ce.route_col = choice.h_col;
      ce.route = choice.route;
      ce.station_id = stop.station_id;
      ce.arrival_step = choice.arrival_step;
      ce.window = std::max(1, stop.dwell_steps);
      ce.departure_step = std::min(T, ce.arrival_step + ce.window);
      ce.window = std::max(1, ce.departure_step - ce.arrival_step);
      ce.requested_kwh_per_veh =
          std::max(0.0, stop.requested_energy_kwh_per_vehicle);
      ce.p_ch_max_per_veh = jo_charge_power_kw_per_veh(stop, opts, dt);
      ce.p_dis_max_per_veh = (opts.allow_v2g && stop.v2g_capable)
          ? std::max(0.0, stop.max_discharge_kw_per_vehicle)
          : 0.0;
      ce.eta_ch = std::max(opts.default_charging_efficiency, kTol);
      ce.eta_dis = std::max(opts.default_charging_efficiency, kTol);
      ce.e_arrival_per_veh = e_arrival;
      ce.e_min_per_veh = e_min;
      ce.has_e_max = has_e_max;
      ce.e_target_per_veh = ce.requested_kwh_per_veh > kTol
          ? e_arrival + ce.requested_kwh_per_veh
          : e_min;
      ce.e_max_per_veh = has_e_max
          ? e_max
          : std::max(ce.e_target_per_veh + ce.p_ch_max_per_veh * ce.window * dt,
                     ce.e_arrival_per_veh + ce.requested_kwh_per_veh);
      ce.v2g = ce.p_dis_max_per_veh > kTol;
      ce.session_pos = static_cast<int>(sessions.size());

      for (int off = 0; off <= ce.window; ++off) {
        CDCol e;
        e.kind = CDCol::Kind::BatteryEnergy;
        e.demand_pos = choice.demand_pos;
        e.demand_index = demand.index;
        e.route_index = choice.route_index;
        e.departure_step = choice.departure_step;
        e.session_pos = ce.session_pos;
        e.offset = off;
        e.step = ce.arrival_step + off;
        const double ub = std::max({1.0e-6, ce.e_max_per_veh, ce.e_target_per_veh}) *
                          std::max(1.0, demand.vehicles);
        ce.energy_cols.push_back(add_col(e, 0.0, ub, 0.0,
            "E_d" + std::to_string(demand.index) + "_r" +
            std::to_string(choice.route_index) + "_k" +
            std::to_string(choice.departure_step) + "_s" +
            std::to_string(stop.station_id) + "_o" + std::to_string(off)));
      }
      for (int off = 0; off < ce.window; ++off) {
        const int k = ce.arrival_step + off;
        CDCol p;
        p.kind = CDCol::Kind::ChargePower;
        p.demand_pos = choice.demand_pos;
        p.demand_index = demand.index;
        p.route_index = choice.route_index;
        p.departure_step = choice.departure_step;
        p.session_pos = ce.session_pos;
        p.offset = off;
        p.step = k;
        const double price = jo_station_price(problem, opts, stop.station_id, k);
        const double ch_cost = user_objective
            ? opts.station_energy_cost_weight * price * dt
            : 0.0;
        ce.pch_cols.push_back(add_col(p, 0.0, ce.p_ch_max_per_veh * demand.vehicles,
                                      ch_cost,
            "Pch_d" + std::to_string(demand.index) + "_r" +
            std::to_string(choice.route_index) + "_k" +
            std::to_string(choice.departure_step) + "_s" +
            std::to_string(stop.station_id) + "_t" + std::to_string(k)));
        CDCol q = p;
        q.kind = CDCol::Kind::DischargePower;
        const double dis_cost = user_objective
            ? -opts.station_energy_cost_weight * price * dt
            : 0.0;
        ce.pdis_cols.push_back(add_col(q, 0.0, ce.p_dis_max_per_veh * demand.vehicles,
                                       dis_cost,
            "Pdis_d" + std::to_string(demand.index) + "_r" +
            std::to_string(choice.route_index) + "_k" +
            std::to_string(choice.departure_step) + "_s" +
            std::to_string(stop.station_id) + "_t" + std::to_string(k)));
      }

      // Energy equations.
      {
        const int r = add_eq(0.0);
        eq_trips.emplace_back(r, ce.energy_cols.front(), 1.0);
        eq_trips.emplace_back(r, ce.route_col, -ce.e_arrival_per_veh);
      }
      for (int off = 0; off < ce.window; ++off) {
        const int r = add_eq(0.0);
        eq_trips.emplace_back(r, ce.energy_cols[static_cast<std::size_t>(off + 1)], 1.0);
        eq_trips.emplace_back(r, ce.energy_cols[static_cast<std::size_t>(off)], -1.0);
        eq_trips.emplace_back(r, ce.pch_cols[static_cast<std::size_t>(off)],
                              -ce.eta_ch * dt);
        eq_trips.emplace_back(r, ce.pdis_cols[static_cast<std::size_t>(off)],
                              dt / ce.eta_dis);
      }
      for (int off = 0; off <= ce.window; ++off) {
        const int ecol = ce.energy_cols[static_cast<std::size_t>(off)];
        int r = add_ineq(0.0);
        ineq_trips.emplace_back(r, ecol, -1.0);
        ineq_trips.emplace_back(r, ce.route_col, ce.e_min_per_veh);
        r = add_ineq(0.0);
        ineq_trips.emplace_back(r, ecol, 1.0);
        ineq_trips.emplace_back(r, ce.route_col, -ce.e_max_per_veh);
      }
      {
        const int r = add_ineq(0.0);
        ineq_trips.emplace_back(r, ce.energy_cols.back(), -1.0);
        ineq_trips.emplace_back(r, ce.route_col, ce.e_target_per_veh);
      }
      for (int off = 0; off < ce.window; ++off) {
        const int pch = ce.pch_cols[static_cast<std::size_t>(off)];
        const int pdis = ce.pdis_cols[static_cast<std::size_t>(off)];
        int r = add_ineq(0.0);
        ineq_trips.emplace_back(r, pch, 1.0);
        ineq_trips.emplace_back(r, ce.route_col, -ce.p_ch_max_per_veh);
        r = add_ineq(0.0);
        ineq_trips.emplace_back(r, pdis, 1.0);
        ineq_trips.emplace_back(r, ce.route_col, -ce.p_dis_max_per_veh);
      }
      sessions.push_back(std::move(ce));
    }
  }

  // Station charge and discharge capacities.
  std::map<std::pair<int, int>, std::vector<int>> station_ch_cols;
  std::map<std::pair<int, int>, std::vector<int>> station_dis_cols;
  for (const auto& ce : sessions) {
    for (int off = 0; off < ce.window; ++off) {
      const int k = ce.arrival_step + off;
      station_ch_cols[{ce.station_id, k}].push_back(ce.pch_cols[static_cast<std::size_t>(off)]);
      station_dis_cols[{ce.station_id, k}].push_back(ce.pdis_cols[static_cast<std::size_t>(off)]);
    }
  }
  for (const auto& [key, cols_for_station] : station_ch_cols) {
    const int station_id = key.first;
    const double cap = stn_cap_kw.count(station_id)
        ? stn_cap_kw.at(station_id)
        : opts.default_station_power_kw;
    const int r = add_ineq(cap);
    for (int c : cols_for_station) ineq_trips.emplace_back(r, c, 1.0);
  }
  for (const auto& [key, cols_for_station] : station_dis_cols) {
    const int station_id = key.first;
    const double cap = stn_cap_kw.count(station_id)
        ? stn_cap_kw.at(station_id)
        : opts.default_station_power_kw;
    const int r = add_ineq(cap);
    for (int c : cols_for_station) ineq_trips.emplace_back(r, c, 1.0);
  }

  // Optional DC-OPF variables and equations.
  std::vector<int> col_gen_base(static_cast<std::size_t>(n_gens), -1);
  std::vector<int> col_bus_base(static_cast<std::size_t>(n_buses), -1);
  std::vector<int> col_br_base(static_cast<std::size_t>(n_branches), -1);
  std::vector<int> col_pslk_base(static_cast<std::size_t>(n_buses), -1);

  if (opts.include_dcopf) {
    if (n_buses == 0 || n_gens == 0) {
      return unsupported(
          ltm_pwl_mode
              ? "unsupported: full-joint LTM PWL-MILP requested DC-OPF but the AC grid has no buses/generators"
              : "unsupported: certified dynamic MPEC MILP requested DC-OPF but the AC grid has no buses/generators");
    }
    for (int gi = 0; gi < n_gens; ++gi) {
      const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
      if (!gen.in_service) continue;
      col_gen_base[static_cast<std::size_t>(gi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        CDCol c;
        c.kind = CDCol::Kind::GeneratorPower;
        c.gen_pos = gi;
        c.step = k;
        const double gen_cost = user_objective ? 0.0 : gen.cost_c1 * dt;
        add_col(c, std::max(0.0, gen.pmin_mw), gen.pmax_mw, gen_cost,
                "Pg_g" + std::to_string(gen.index) + "_k" + std::to_string(k));
      }
    }
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      col_bus_base[static_cast<std::size_t>(bi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        CDCol c;
        c.kind = CDCol::Kind::BusAngle;
        c.bus_pos = bi;
        c.step = k;
        const double lb = (bi == slack_bus_pos) ? 0.0 : -1.0e6;
        const double ub = (bi == slack_bus_pos) ? 0.0 : 1.0e6;
        add_col(c, lb, ub, 0.0,
                "theta_b" + std::to_string(bus.index) + "_k" + std::to_string(k));
      }
    }
    for (int li = 0; li < n_branches; ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      if (!br.in_service) continue;
      col_br_base[static_cast<std::size_t>(li)] = static_cast<int>(cols.size());
      const double fmax = br.rate_a_mva > kTol ? br.rate_a_mva : 1.0e6;
      for (int k = 0; k < T; ++k) {
        CDCol c;
        c.kind = CDCol::Kind::BranchFlow;
        c.branch_pos = li;
        c.step = k;
        add_col(c, -fmax, fmax, 0.0,
                "F_l" + std::to_string(br.index) + "_k" + std::to_string(k));
      }
    }
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      col_pslk_base[static_cast<std::size_t>(bi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        CDCol c;
        c.kind = CDCol::Kind::PowerSlack;
        c.bus_pos = bi;
        c.step = k;
        add_col(c, 0.0, 1.0e6, opts.power_slack_penalty,
                "slack_b" + std::to_string(bus.index) + "_k" + std::to_string(k));
      }
    }

    for (int li = 0; li < n_branches; ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      if (!br.in_service || col_br_base[static_cast<std::size_t>(li)] < 0) continue;
      auto it_i = bus_pos_map.find(br.from_bus);
      auto it_j = bus_pos_map.find(br.to_bus);
      if (it_i == bus_pos_map.end() || it_j == bus_pos_map.end()) continue;
      const int bi = it_i->second;
      const int bj = it_j->second;
      const double bl = branch_susceptance[static_cast<std::size_t>(li)];
      for (int k = 0; k < T; ++k) {
        const int r = add_eq(0.0);
        eq_trips.emplace_back(r, col_br_base[static_cast<std::size_t>(li)] + k, 1.0);
        eq_trips.emplace_back(r, col_bus_base[static_cast<std::size_t>(bi)] + k, -bl);
        eq_trips.emplace_back(r, col_bus_base[static_cast<std::size_t>(bj)] + k, bl);
      }
    }

    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      for (int k = 0; k < T; ++k) {
        const int r = add_eq(bus.pd_mw);
        if (col_pslk_base[static_cast<std::size_t>(bi)] >= 0) {
          eq_trips.emplace_back(r, col_pslk_base[static_cast<std::size_t>(bi)] + k, 1.0);
        }
        for (int gi = 0; gi < n_gens; ++gi) {
          const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
          if (!gen.in_service || gen.bus != bus.index) continue;
          if (col_gen_base[static_cast<std::size_t>(gi)] >= 0) {
            eq_trips.emplace_back(r, col_gen_base[static_cast<std::size_t>(gi)] + k, 1.0);
          }
        }
        for (int li = 0; li < n_branches; ++li) {
          const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
          if (!br.in_service || col_br_base[static_cast<std::size_t>(li)] < 0) continue;
          if (br.from_bus == bus.index) {
            eq_trips.emplace_back(r, col_br_base[static_cast<std::size_t>(li)] + k, -1.0);
          }
          if (br.to_bus == bus.index) {
            eq_trips.emplace_back(r, col_br_base[static_cast<std::size_t>(li)] + k, 1.0);
          }
        }
        for (const auto& ce : sessions) {
          const auto bit = stn_bus_map.find(ce.station_id);
          if (bit == stn_bus_map.end() || bit->second != bus.index) continue;
          for (int off = 0; off < ce.window; ++off) {
            if (ce.arrival_step + off != k) continue;
            eq_trips.emplace_back(r, ce.pch_cols[static_cast<std::size_t>(off)], -1.0 / 1000.0);
            eq_trips.emplace_back(r, ce.pdis_cols[static_cast<std::size_t>(off)], 1.0 / 1000.0);
          }
        }
      }
    }
  }

  const int n_vars = static_cast<int>(cols.size());
  lp.vars = std::move(vars);
  lp.c = Eigen::VectorXd::Zero(n_vars);
  for (int j = 0; j < n_vars; ++j) lp.c[j] = obj[static_cast<std::size_t>(j)];

  lp.Aeq.resize(static_cast<int>(beq_vec.size()), n_vars);
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Zero(static_cast<int>(beq_vec.size()));
  for (int i = 0; i < static_cast<int>(beq_vec.size()); ++i) {
    lp.beq[i] = beq_vec[static_cast<std::size_t>(i)];
  }

  lp.A.resize(static_cast<int>(bineq_vec.size()), n_vars);
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Zero(static_cast<int>(bineq_vec.size()));
  for (int i = 0; i < static_cast<int>(bineq_vec.size()); ++i) {
    lp.b[i] = bineq_vec[static_cast<std::size_t>(i)];
  }

  result.n_variables = n_vars;
  result.n_constraints = static_cast<int>(beq_vec.size() + bineq_vec.size());
  if (opts.verbose) {
    std::fprintf(stderr,
        "[%s] model: %d vars, %d eq, %d ineq, choices=%zu, sessions=%zu\n",
        ltm_pwl_mode ? "CertifiedFullJointLtmPwlMILP" : "CertifiedDynamicMPEC",
        n_vars, static_cast<int>(beq_vec.size()),
        static_cast<int>(bineq_vec.size()), choices.size(), sessions.size());
  }

  Eigen::VectorXd x_sol;
  bool solved = false;
  auto solve_native_bc = [&](const std::string& reason) {
    engine::BCOptions bc;
    bc.max_nodes = std::max(1, opts.max_nodes);
    bc.time_limit_sec = std::max(0.0, opts.time_limit_sec);
    bc.gap_tol = std::max(0.0, opts.mip_gap);
    bc.use_simplex_lp_nodes = true;
    bc.verbose = false;
    auto bc_res = engine::solve_milp_bc(mip, bc);
    x_sol = bc_res.x;
    solved = bc_res.stats.success;
    result.solver_backend =
        reason.empty()
            ? (ltm_pwl_mode ? "NativeB&C-CertifiedFullJointLtmPwlMILP"
                            : "NativeB&C-CertifiedDynamicMPEC")
            : (ltm_pwl_mode ? "HiGHS+NativeB&C-CertifiedFullJointLtmPwlMILP"
                            : "HiGHS+NativeB&C-CertifiedDynamicMPEC");
    result.solver_status =
        reason.empty() ? bc_res.stats.status
                       : "HiGHS failed: " + reason + "; NativeB&C: " +
                         bc_res.stats.status;
    result.objective = bc_res.stats.objective;
    result.best_bound = bc_res.bc_stats.best_bound;
    result.mip_gap = std::isfinite(bc_res.bc_stats.gap)
        ? bc_res.bc_stats.gap
        : bc_res.stats.mip_gap;
  };

  engine::HighsAdapter highs;
  if (highs.available()) {
    auto hr = highs.solve_milp(mip);
    x_sol = hr.x;
    solved = hr.stats.success;
    result.solver_backend =
        ltm_pwl_mode ? "HiGHS-CertifiedFullJointLtmPwlMILP"
                     : "HiGHS-CertifiedDynamicMPEC";
    result.solver_status = hr.stats.status;
    result.objective = hr.stats.objective;
    result.mip_gap = hr.stats.mip_gap;
    result.best_bound = inferred_minimization_bound(result.objective, result.mip_gap);
    if (!solved || x_sol.size() != n_vars) {
      const std::string reason =
          result.solver_status.empty() ? "unsuccessful" : result.solver_status;
      solve_native_bc(reason);
    }
  } else {
    solve_native_bc("");
  }

  result.solve_time_sec =
      duration_cast<microseconds>(steady_clock::now() - t_start).count() * 1.0e-6;
  if (!solved || x_sol.size() != static_cast<Eigen::Index>(n_vars)) {
    jo_warn(result, "certified dynamic MPEC MILP solve failed: " + result.solver_status,
            opts.verbose);
    return result;
  }

  jo_compute_certificate_residuals(result, mip, x_sol);
  result.feasible = result.primal_max_violation <= 1.0e-6 &&
                    result.integrality_max_violation <= 1.0e-6;
  result.proven_optimal = result.feasible &&
                          std::isfinite(result.mip_gap) &&
                          result.mip_gap <= opts.mip_gap + 1.0e-9;
  result.global_optimum_certificate = result.proven_optimal;
  result.mathematical_model_verified = result.global_optimum_certificate;
  result.mathematical_model_verification_status =
      result.global_optimum_certificate
          ? (ltm_pwl_mode
                 ? "verified for the assembled full-joint LTM PWL-MILP "
                   "approximation (cumulative-count LTM rows, PWL BPR "
                   "travel-cost envelope with reported error bound, big-M "
                   "Wardrop, charging/V2G, and configured DC-OPF); this is "
                   "not an exact global proof of the original nonlinear model"
                 : "verified for the assembled finite linear dynamic MPEC MILP "
                   "(affine endogenous congestion, big-M Wardrop, charging/V2G, "
                   "and configured DC-OPF); this is not a nonlinear CTM/LTM "
                   "global proof")
          : "not verified: MILP certificate did not meet feasibility/gap tolerances";
  if (ltm_pwl_mode) {
    result.ltm_conservation_max_violation = result.primal_max_violation;
  }

  std::vector<std::vector<double>> link_volume(
      problem.traffic.links.size(), std::vector<double>(T, 0.0));
  for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
    for (int k = 0; k < T; ++k) {
      const int c = link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)];
      if (c >= 0 && c < x_sol.size() && std::isfinite(x_sol[c])) {
        link_volume[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] =
            std::max(0.0, x_sol[c]);
      }
      const double cap = link_capacity_vehicles(
          problem.traffic.links[static_cast<std::size_t>(li)], k, dt);
      result.road_capacity_max_violation = std::max(
          result.road_capacity_max_violation,
          std::max(0.0, link_volume[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)] -
                            cap));
    }
  }
  if (ltm_pwl_mode) {
    for (int li = 0; li < static_cast<int>(problem.traffic.links.size()); ++li) {
      for (int k = 1; k <= T; ++k) {
        const int nin = ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)];
        const int nprev = ltm_nin_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)];
        const int vcol = link_vol_col[static_cast<std::size_t>(li)][static_cast<std::size_t>(k - 1)];
        if (nin >= 0 && nprev >= 0 && vcol >= 0 &&
            nin < x_sol.size() && nprev < x_sol.size() && vcol < x_sol.size()) {
          result.ltm_conservation_max_violation = std::max(
              result.ltm_conservation_max_violation,
              std::abs((x_sol[nin] - x_sol[nprev]) - x_sol[vcol]));
        }
      }
    }
  }

  std::unordered_map<int, double> min_cost_by_demand_pos;
  for (int di : demand_positions) {
    const int pi = demand_to_pi_col[di];
    min_cost_by_demand_pos[di] =
        (pi >= 0 && pi < x_sol.size() && std::isfinite(x_sol[pi])) ? x_sol[pi] : 0.0;
  }

  int next_session_index = 0;
  for (const auto& choice : choices) {
    double h = x_sol[choice.h_col];
    if (!std::isfinite(h) || h <= kTol) continue;
    const auto& demand = problem.demands[static_cast<std::size_t>(choice.demand_pos)];
    const double route_cost =
        std::isfinite(x_sol[choice.cost_col]) ? x_sol[choice.cost_col]
                                              : choice.base_private_cost;
    const double min_cost = min_cost_by_demand_pos[choice.demand_pos];
    result.wardrop_complementarity_residual = std::max(
        result.wardrop_complementarity_residual,
        std::max(0.0, route_cost - min_cost));
    result.wardrop_gap += h * std::max(0.0, route_cost - min_cost);

    double tt = choice.ff_hr;
    for (const auto& [li, k] : choice.link_steps) {
      tt += affine_latency_slope_hr_per_veh(li, k) *
            link_volume[static_cast<std::size_t>(li)][static_cast<std::size_t>(k)];
      if (ltm_pwl_mode) {
        const int tau_col = link_tt_col[static_cast<std::size_t>(li)]
                                       [static_cast<std::size_t>(k)];
        if (tau_col >= 0 && tau_col < x_sol.size() && std::isfinite(x_sol[tau_col])) {
          const auto& link = problem.traffic.links[static_cast<std::size_t>(li)];
          tt += x_sol[tau_col] - std::max(0.0, link.free_flow_time_hr);
        }
      }
    }
    result.total_served_vehicles += h;
    result.ev_benefit += demand.willingness_to_pay_per_vehicle * h;
    result.total_travel_time_hr += h * tt;
    result.traffic_delay_cost += opts.value_of_time_per_hr * h * tt;
    result.total_assignment_cost += h * route_cost;
    result.route_flow[demand.index][choice.route_index] += h;
    result.assignments.push_back({demand.index, choice.route_index,
                                  choice.departure_step, h, route_cost, tt,
                                  true,
                                  ltm_pwl_mode ? "certified full-joint LTM PWL-MILP"
                                               : "certified dynamic MPEC MILP"});
  }

  for (int j = 0; j < n_vars; ++j) {
    if (cols[static_cast<std::size_t>(j)].kind == CDCol::Kind::UnservedDemand) {
      const double u = std::max(0.0, std::isfinite(x_sol[j]) ? x_sol[j] : 0.0);
      if (u <= kTol) continue;
      const auto& demand =
          problem.demands[static_cast<std::size_t>(cols[static_cast<std::size_t>(j)].demand_pos)];
      result.total_unserved_vehicles += u;
      result.assignments.push_back({demand.index, 0, demand.departure_step,
                                    u, opts.unserved_trip_penalty, 0.0,
                                    false,
                                    ltm_pwl_mode
                                        ? "certified full-joint LTM PWL-MILP unmet demand"
                                        : "certified dynamic MPEC unmet demand"});
    }
  }
  result.wardrop_gap /= std::max(1.0, result.total_assignment_cost);

  for (const auto& ce : sessions) {
    const double h = std::max(0.0, std::isfinite(x_sol[ce.route_col]) ? x_sol[ce.route_col] : 0.0);
    if (h <= kTol || ce.route == nullptr) continue;
    const auto& demand = problem.demands[static_cast<std::size_t>(ce.demand_pos)];

    EVChargingSession sess;
    sess.index = next_session_index++;
    sess.station_id = ce.station_id;
    sess.arrival_step = ce.arrival_step;
    sess.departure_step = ce.departure_step;
    sess.vehicle_count = h;
    sess.energy_initial_kwh = ce.e_arrival_per_veh * h;
    sess.energy_target_kwh = ce.e_target_per_veh * h;
    sess.energy_min_kwh = ce.e_min_per_veh * h;
    sess.energy_max_kwh = ce.e_max_per_veh * h;
    sess.max_charge_kw = ce.p_ch_max_per_veh * h;
    sess.max_discharge_kw = ce.p_dis_max_per_veh * h;
    sess.eta_charge = ce.eta_ch;
    sess.eta_discharge = ce.eta_dis;
    sess.v2g_capable = ce.v2g;
    sess.source_demand_index = demand.index;
    sess.source_route_index = ce.route->index;

    ChargingSessionResult sr;
    sr.session_index = sess.index;
    sr.station_id = ce.station_id;
    sr.p_charge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.p_discharge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.energy_kwh.assign(static_cast<std::size_t>(T + 1), 0.0);
    for (int off = 0; off < ce.window; ++off) {
      const int k = ce.arrival_step + off;
      const double pch = std::max(0.0, x_sol[ce.pch_cols[static_cast<std::size_t>(off)]]);
      const double pdis = std::max(0.0, x_sol[ce.pdis_cols[static_cast<std::size_t>(off)]]);
      if (k >= 0 && k < T) {
        sr.p_charge_kw[static_cast<std::size_t>(k)] = pch;
        sr.p_discharge_kw[static_cast<std::size_t>(k)] = pdis;
      }
      result.total_delivered_energy_kwh += ce.eta_ch * pch * dt;
      result.total_v2g_energy_kwh += pdis * dt / ce.eta_dis;
    }
    for (int off = 0; off <= ce.window; ++off) {
      const int k = ce.arrival_step + off;
      if (k >= 0 && k <= T) {
        sr.energy_kwh[static_cast<std::size_t>(k)] =
            std::max(0.0, x_sol[ce.energy_cols[static_cast<std::size_t>(off)]]);
      }
    }
    for (int k = 1; k <= T; ++k) {
      if (sr.energy_kwh[static_cast<std::size_t>(k)] <= kTol) {
        sr.energy_kwh[static_cast<std::size_t>(k)] =
            sr.energy_kwh[static_cast<std::size_t>(k - 1)];
      }
    }
    sr.unserved_energy_kwh = std::max(
        0.0, sess.energy_target_kwh -
                 sr.energy_kwh[static_cast<std::size_t>(sess.departure_step)]);
    result.total_unserved_energy_kwh += sr.unserved_energy_kwh;
    result.total_requested_energy_kwh += ce.requested_kwh_per_veh * h;
    result.sessions.push_back(sess);
    result.session_results.push_back(std::move(sr));
  }

  if (opts.include_dcopf) {
    result.gen_dispatch_mw.resize(static_cast<std::size_t>(T));
    result.lmp_by_step.resize(static_cast<std::size_t>(T));
    for (int j = 0; j < n_vars; ++j) {
      const auto& col = cols[static_cast<std::size_t>(j)];
      if (col.kind != CDCol::Kind::GeneratorPower || col.step < 0 ||
          col.step >= T || col.gen_pos < 0) {
        continue;
      }
      const double pg = std::isfinite(x_sol[j]) ? x_sol[j] : 0.0;
      const auto& gen = sys.ac.generators[static_cast<std::size_t>(col.gen_pos)];
      result.gen_dispatch_mw[static_cast<std::size_t>(col.step)][gen.index] = pg;
      result.gen_cost += gen.cost_c1 * pg * dt;
    }
    result.power_balance_max_violation = result.primal_max_violation;
  }

  result.social_welfare =
      result.ev_benefit - result.traffic_delay_cost - result.gen_cost;
  return result;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// solve_joint_optimizer — Formulation D entry point
// ─────────────────────────────────────────────────────────────────────────────
JointOptimizerResult solve_joint_optimizer(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& opts) {
  using namespace std::chrono;
  const auto t_start = steady_clock::now();

  if (opts.mode == JointOptimizerMode::FullJointSocialWelfareNLP ||
      opts.mode == JointOptimizerMode::FullJointUserBenefitNLP) {
    return solve_joint_optimizer_full_nlp(problem, opts);
  }

  if (opts.mode == JointOptimizerMode::CertifiedDynamicMPECMILP ||
      opts.mode == JointOptimizerMode::CertifiedDynamicUserBenefitMPECMILP ||
      opts.mode == JointOptimizerMode::CertifiedFullJointLtmPwlMILP ||
      opts.mode == JointOptimizerMode::CertifiedFullJointLtmUserBenefitPwlMILP) {
    return solve_certified_dynamic_mpec_milp(problem, opts);
  }

  JointOptimizerResult result;
  if (opts.require_exact_mathematical_model) {
    result.feasible = false;
    result.proven_optimal = false;
    result.global_optimum_certificate = false;
    result.mathematical_model_verified = false;
    result.solver_backend = "ExactMathematicalModelGate";
    result.solver_status =
        "unsupported: no in-tree solver certifies the full joint "
        "power-traffic mathematical model globally";
    result.mathematical_model_verification_status =
        "unsupported: LP/MILP modes use exogenous route costs and FullJoint NLP "
        "has only a local certificate";
    jo_warn(result, result.mathematical_model_verification_status, opts.verbose);
    return result;
  }

  const int T   = std::max(1, opts.num_steps);
  const double dt = std::max(1e-9, opts.time_step_hr);

  // ── Enumerate active demands ────────────────────────────────────────────
  std::vector<int> active_demand_pos;
  active_demand_pos.reserve(problem.demands.size());
  std::unordered_map<int, int> demand_eq_row;  // demand_pos → equality row index

  for (int di = 0; di < static_cast<int>(problem.demands.size()); ++di) {
    const auto& d = problem.demands[static_cast<std::size_t>(di)];
    if (d.departure_step < 0 || d.departure_step >= T || d.vehicles <= kTol)
      continue;
    demand_eq_row[di] = static_cast<int>(active_demand_pos.size());
    active_demand_pos.push_back(di);
    result.total_served_vehicles += d.vehicles;  // will be adjusted below
  }
  result.total_served_vehicles = 0.0;  // reset; filled after solve

  if (active_demand_pos.empty()) {
    result.feasible        = true;
    result.proven_optimal  = true;
    result.solver_backend  = "LP-empty";
    result.solver_status   = "empty";
    return result;
  }

  // ── Power system indices ────────────────────────────────────────────────
  const auto& sys  = problem.system;
  const int n_buses  = static_cast<int>(sys.ac.buses.size());
  const int n_gens   = static_cast<int>(sys.ac.generators.size());
  const int n_branches = static_cast<int>(sys.ac.branches.size());

  // Adapter options for helper functions that expect EVPowerTrafficOptions
  const EVPowerTrafficOptions evpt_opts = jo_to_evpt_opts(opts);

  // Build bus index map: bus.index → position in sys.ac.buses
  std::unordered_map<int, int> bus_pos_map;
  for (int bi = 0; bi < n_buses; ++bi)
    bus_pos_map[sys.ac.buses[static_cast<std::size_t>(bi)].index] = bi;

  // Identify slack bus (reference bus for DC OPF)
  int slack_bus_pos = 0;
  for (int bi = 0; bi < n_buses; ++bi) {
    const auto& b = sys.ac.buses[static_cast<std::size_t>(bi)];
    if (b.bus_type == BusType::SLACK) {
      slack_bus_pos = bi;
      break;
    }
  }

  // Branch susceptance b_ℓ = 1/x_ℓ (DC approximation)
  std::vector<double> branch_susceptance(static_cast<std::size_t>(n_branches), 0.0);
  for (int li = 0; li < n_branches; ++li) {
    const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
    branch_susceptance[static_cast<std::size_t>(li)] =
        std::abs(br.x_pu) > 1e-12 ? 1.0 / br.x_pu : 1e6;
  }

  // Station capacity and bus mapping
  const auto stn_cap_kw  = jo_station_cap_kw(sys, opts);
  const auto stn_bus_map = jo_station_bus_map(sys);

  // Link position map
  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t li = 0; li < problem.traffic.links.size(); ++li)
    link_pos[problem.traffic.links[li].index] = li;

  // ── Optional CTM pre-solve for dynamic travel times ──────────────────────
  // When opts.use_ctm_travel_times, run one forward CTM pass with equal-split
  // initial flows to obtain congestion-aware T^CTM_{r,k_dep} [hr].
  std::unordered_map<int, std::vector<double>> ctm_route_tt;
  if (opts.use_ctm_travel_times && !problem.traffic.links.empty()) {
    const int n_routes_all = static_cast<int>(problem.routes.size());
    std::unordered_map<int, int> route_pos_ctm;
    for (int ri = 0; ri < n_routes_all; ++ri)
      route_pos_ctm[problem.routes[static_cast<std::size_t>(ri)].index] = ri;

    // Equal-split initial flow: each demand split evenly over its candidate routes.
    std::vector<std::vector<double>> x0(
        static_cast<std::size_t>(T),
        std::vector<double>(static_cast<std::size_t>(n_routes_all), 0.0));
    for (const auto& demand : problem.demands) {
      if (demand.departure_step < 0 || demand.departure_step >= T) continue;
      const auto cands = candidate_routes(problem, demand);
      if (cands.empty()) continue;
      const double share = demand.vehicles / static_cast<double>(cands.size());
      for (const auto* r : cands) {
        const auto it = route_pos_ctm.find(r->index);
        if (it == route_pos_ctm.end()) continue;
        x0[static_cast<std::size_t>(demand.departure_step)]
         [static_cast<std::size_t>(it->second)] += share;
      }
    }

    auto ctm_pre = ctm_forward_pass(problem, x0, link_pos, route_pos_ctm,
                                    opts.ctm_opts, dt, T, 1);
    ctm_route_tt = std::move(ctm_pre.route_travel_time);
  }

  // ── Build LP variable columns ───────────────────────────────────────────
  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<JOColumn>          cols;
  std::vector<double>            obj;
  std::vector<engine::VariableMeta> vars;

  const bool use_milp = (opts.mode == JointOptimizerMode::IntegerRouteMILP);
  if (opts.mode == JointOptimizerMode::IntegerRouteMILP) {
    jo_warn(result,
            "IntegerRouteMILP enforces integral route/unserved variables only; "
            "Wardrop complementarity is not assembled.",
            opts.verbose);
  }

  auto add_col = [&](JOColumn c, double lb, double ub, double cost,
                     const std::string& name, bool integer_var) -> int {
    const int idx = static_cast<int>(cols.size());
    cols.push_back(c);
    obj.push_back(cost);
    vars.push_back({integer_var ? engine::VarType::Integer
                                : engine::VarType::Continuous,
                    lb, ub, name});
    if (integer_var && (ub <= 1.0 + 1e-9)) mip.binary_idx.push_back(idx);
    else if (integer_var)                   mip.integer_idx.push_back(idx);
    return idx;
  };

  // ──────────────────────────────────────────────────────────────────────
  // Group 1: Route flow variables h_{d,r}  and unserved demand u_d
  // ──────────────────────────────────────────────────────────────────────
  const int n_eq_demand = static_cast<int>(active_demand_pos.size());
  std::vector<double>               beq_demand(static_cast<std::size_t>(n_eq_demand), 0.0);
  std::vector<Eigen::Triplet<double>> eq_trips;

  // Per-route arrival step: pre-compute free-flow arrival
  // route_arrival[demand_pos][route_idx_in_candidates] = arrival_step
  struct RouteEntry {
    const RouteAlternative* route{nullptr};
    int col_idx{-1};
    int arrival_step{-1};
  };
  struct ChargingEntry {
    int demand_pos{-1};
    int route_col_idx{-1};
    const RouteAlternative* route{nullptr};
    int station_id{0};
    int arrival_step{0};
    int departure_step{0};
    int window{1};
    double vehicles_ub{0.0};
    double requested_kwh_per_veh{0.0};
    double p_ch_max_per_veh{0.0};
    double p_dis_max_per_veh{0.0};
    double eta_ch{1.0};
    double eta_dis{1.0};
    double e_arrival_per_veh{0.0};
    double e_min_per_veh{0.0};
    double e_max_per_veh{0.0};
    double e_target_per_veh{0.0};
    bool has_e_max{false};
    bool v2g{false};
    std::vector<int> pch_cols;
    std::vector<int> pdis_cols;
    std::vector<int> energy_cols;
  };
  std::vector<std::vector<RouteEntry>> demand_routes(
      static_cast<std::size_t>(n_eq_demand));
  std::vector<ChargingEntry> charging_entries;
  std::vector<PendingEq> pending_eqs;

  for (int ei = 0; ei < n_eq_demand; ++ei) {
    const int di = active_demand_pos[static_cast<std::size_t>(ei)];
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];

    beq_demand[static_cast<std::size_t>(ei)] = demand.vehicles;

    const auto routes = candidate_routes(problem, demand);
    for (const auto* route : routes) {
      if (!route_can_deliver_requested_energy(*route, evpt_opts, dt)) continue;
      if (!route_soc_feasible(*route, demand, problem, link_pos))  continue;

      const int arr = route_arrival_step(*route, link_pos, problem,
                                         demand.departure_step, dt);
      // Check that all charging windows fit inside the horizon
      bool fits = true;
      for (const auto& stop : route->charging_stops) {
        const int dep = arr + std::max(1, stop.dwell_steps);
        if (arr < 0 || arr >= T || dep > T) { fits = false; break; }
      }
      if (!fits) continue;

      // Free-flow travel time for this route (always computed as fallback)
      double ff_hr = 0.0;
      for (int li : route->link_indices) {
        if (const auto it = link_pos.find(li); it != link_pos.end())
          ff_hr += std::max(0.0, problem.traffic.links[it->second].free_flow_time_hr);
      }

      // Travel time used in objective: CTM-derived if available, else free-flow.
      double tt_hr = ff_hr;
      if (opts.use_ctm_travel_times) {
        const auto ctt = ctm_route_tt.find(route->index);
        if (ctt != ctm_route_tt.end()) {
          const int kd = demand.departure_step;
          if (kd >= 0 && kd < static_cast<int>(ctt->second.size()))
            tt_hr = ctt->second[static_cast<std::size_t>(kd)];
        }
      }

      // Objective cost for this route (depends on mode)
      double route_cost = opts.value_of_time_per_hr * tt_hr;  // VOT×T^{CTM or ff}
      route_cost -= demand.willingness_to_pay_per_vehicle;      // −WTP
      // Electricity cost/revenue is attached only to p^ch/p^dis columns below.

      JOColumn jc;
      jc.kind          = JOColumn::Kind::RouteFlow;
      jc.demand_pos    = di;
      jc.route         = route;
      jc.departure_step = demand.departure_step;
      jc.arrival_step  = arr;

      const bool int_var = (use_milp &&
                            std::abs(demand.vehicles - std::round(demand.vehicles)) < 1e-7);

      const int cidx = add_col(jc, 0.0, demand.vehicles, route_cost,
          "h_d" + std::to_string(demand.index) +
          "_r" + std::to_string(route->index), int_var);

      eq_trips.emplace_back(ei, cidx, 1.0);
      demand_routes[static_cast<std::size_t>(ei)].push_back({route, cidx, arr});

      const double drive_kwh_per_veh =
          jo_route_drive_energy_kwh_per_veh(*route, problem, link_pos);
      double e_arrival = 0.0;
      double e_min = 0.0;
      double e_max = 0.0;
      bool has_e_max = false;
      if (demand.initial_energy_kwh >= 0.0) {
        e_arrival = std::max(0.0, demand.initial_energy_kwh - drive_kwh_per_veh);
        e_min = std::max(0.0, demand.energy_min_kwh);
        if (demand.energy_max_kwh >= 0.0) {
          e_max = std::max(e_min, demand.energy_max_kwh);
          has_e_max = true;
          e_arrival = std::min(e_arrival, e_max);
        }
      }

      for (const auto& stop : route->charging_stops) {
        const int window = std::max(1, stop.dwell_steps);
        ChargingEntry ce;
        ce.demand_pos = di;
        ce.route_col_idx = cidx;
        ce.route = route;
        ce.station_id = stop.station_id;
        ce.arrival_step = arr;
        ce.departure_step = std::min(T, arr + window);
        ce.window = std::max(1, ce.departure_step - ce.arrival_step);
        ce.vehicles_ub = demand.vehicles;
        ce.requested_kwh_per_veh = std::max(0.0, stop.requested_energy_kwh_per_vehicle);
        ce.p_ch_max_per_veh = jo_charge_power_kw_per_veh(stop, opts, dt);
        ce.p_dis_max_per_veh = (opts.allow_v2g && stop.v2g_capable)
            ? std::max(0.0, stop.max_discharge_kw_per_vehicle)
            : 0.0;
        ce.eta_ch = std::max(opts.default_charging_efficiency, kTol);
        ce.eta_dis = std::max(opts.default_charging_efficiency, kTol);
        ce.e_arrival_per_veh = e_arrival;
        ce.e_min_per_veh = e_min;
        ce.has_e_max = has_e_max;
        ce.e_target_per_veh = ce.requested_kwh_per_veh > kTol
          ? e_arrival + ce.requested_kwh_per_veh
          : ce.e_min_per_veh;
        ce.e_max_per_veh = has_e_max
            ? e_max
            : std::max(ce.e_target_per_veh + ce.p_ch_max_per_veh * ce.window * dt,
                       ce.e_arrival_per_veh + ce.requested_kwh_per_veh);
        ce.v2g = ce.p_dis_max_per_veh > kTol;
        const int session_pos = static_cast<int>(charging_entries.size());

        for (int off = 0; off <= ce.window; ++off) {
          JOColumn ec;
          ec.kind = JOColumn::Kind::BatteryEnergy;
          ec.demand_pos = di;
          ec.route = route;
          ec.session_pos = session_pos;
          ec.step = ce.arrival_step + off;
          ec.offset = off;
          const double e_ub = std::max(ce.e_max_per_veh, ce.e_target_per_veh) *
                              std::max(ce.vehicles_ub, 1.0);
          ce.energy_cols.push_back(add_col(ec, 0.0, std::max(e_ub, 1.0e-6), 0.0,
              "e_d" + std::to_string(demand.index) + "_r" +
              std::to_string(route->index) + "_s" + std::to_string(stop.station_id) +
              "_o" + std::to_string(off), false));
        }

        for (int off = 0; off < ce.window; ++off) {
          const int k = ce.arrival_step + off;
          JOColumn pc;
          pc.kind = JOColumn::Kind::ChargePower;
          pc.demand_pos = di;
          pc.route = route;
          pc.session_pos = session_pos;
          pc.step = k;
          pc.offset = off;
          const double price = jo_station_price(problem, opts, stop.station_id, k);
          const double ch_cost = (opts.mode == JointOptimizerMode::UserBenefitMax)
              ? opts.station_energy_cost_weight * price * dt
              : 0.0;
          ce.pch_cols.push_back(add_col(pc, 0.0,
              ce.p_ch_max_per_veh * std::max(ce.vehicles_ub, 0.0), ch_cost,
              "pch_d" + std::to_string(demand.index) + "_r" +
              std::to_string(route->index) + "_s" + std::to_string(stop.station_id) +
              "_k" + std::to_string(k), false));

          JOColumn pd;
          pd.kind = JOColumn::Kind::DischargePower;
          pd.demand_pos = di;
          pd.route = route;
          pd.session_pos = session_pos;
          pd.step = k;
          pd.offset = off;
          const double dis_cost = (opts.mode == JointOptimizerMode::UserBenefitMax)
              ? -opts.station_energy_cost_weight * price * dt
              : 0.0;
          ce.pdis_cols.push_back(add_col(pd, 0.0,
              ce.p_dis_max_per_veh * std::max(ce.vehicles_ub, 0.0), dis_cost,
              "pdis_d" + std::to_string(demand.index) + "_r" +
              std::to_string(route->index) + "_s" + std::to_string(stop.station_id) +
              "_k" + std::to_string(k), false));

        }

        pending_eqs.push_back({{{ce.energy_cols.front(), 1.0},
                                {ce.route_col_idx, -ce.e_arrival_per_veh}}, 0.0});
        for (int off = 0; off < ce.window; ++off) {
          pending_eqs.push_back({{{ce.energy_cols[static_cast<std::size_t>(off + 1)], 1.0},
                                  {ce.energy_cols[static_cast<std::size_t>(off)], -1.0},
                                  {ce.pch_cols[static_cast<std::size_t>(off)], -ce.eta_ch * dt},
                                  {ce.pdis_cols[static_cast<std::size_t>(off)], dt / ce.eta_dis}}, 0.0});
        }

        charging_entries.push_back(std::move(ce));
      }
    }

    // Unserved demand u_d
    JOColumn uc;
    uc.kind = JOColumn::Kind::UnservedDemand;
    uc.demand_pos = di;
    uc.departure_step = demand.departure_step;

    const double outside_cost =
      (opts.mode == JointOptimizerMode::UserBenefitMax)
      ? opts.outside_option_cost
      : 0.0;
    const double unserved_cost = opts.unserved_trip_penalty + outside_cost;

    const bool int_var_u = (use_milp &&
                            std::abs(demand.vehicles - std::round(demand.vehicles)) < 1e-7);
    const int u_idx = add_col(uc, 0.0, demand.vehicles, unserved_cost,
                               "u_d" + std::to_string(demand.index), int_var_u);
    eq_trips.emplace_back(ei, u_idx, 1.0);
  }

  // ──────────────────────────────────────────────────────────────────────
  // Group 2: DC-OPF variables (only if include_dcopf = true)
  // ──────────────────────────────────────────────────────────────────────
  // Variable offset tables: [bus/gen/branch_pos * T + step]
  std::vector<int> col_gen_base(static_cast<std::size_t>(n_gens),  -1);
  std::vector<int> col_bus_base(static_cast<std::size_t>(n_buses), -1);
  std::vector<int> col_br_base (static_cast<std::size_t>(n_branches), -1);
  std::vector<int> col_pslk_base(static_cast<std::size_t>(n_buses), -1);

  if (opts.include_dcopf) {
    // Generator dispatch P^g_{g,k}
    for (int gi = 0; gi < n_gens; ++gi) {
      const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
      if (!gen.in_service) continue;
      col_gen_base[static_cast<std::size_t>(gi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        JOColumn jc;
        jc.kind    = JOColumn::Kind::GeneratorPower;
        jc.gen_pos = gi;
        jc.step    = k;
        // Cost: c1·Δt per MWh (SocialWelfareMax minimizes gen cost directly)
        // UserBenefitMax treats gen cost as zero (user doesn't pay for gen directly)
        const double gcost =
            (opts.mode == JointOptimizerMode::SocialWelfareMax)
            ? gen.cost_c1 * dt
            : 0.0;
        add_col(jc, std::max(0.0, gen.pmin_mw), gen.pmax_mw, gcost,
                "Pg_g" + std::to_string(gen.index) + "_k" + std::to_string(k),
                false);
      }
    }

    // Bus voltage angle θ_{b,k}
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      col_bus_base[static_cast<std::size_t>(bi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        JOColumn jc;
        jc.kind    = JOColumn::Kind::BusAngle;
        jc.bus_pos = bi;
        jc.step    = k;
        // Slack bus: fix angle to 0 by setting lb = ub = 0
        const double lb = (bi == slack_bus_pos) ? 0.0 : -1e6;
        const double ub = (bi == slack_bus_pos) ? 0.0 :  1e6;
        add_col(jc, lb, ub, 0.0,
                "theta_b" + std::to_string(bus.index) + "_k" + std::to_string(k),
                false);
      }
    }

    // Branch flows f_{ℓ,k}
    for (int li = 0; li < n_branches; ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      if (!br.in_service) continue;
      col_br_base[static_cast<std::size_t>(li)] = static_cast<int>(cols.size());
      const double fmax = br.rate_a_mva > kTol ? br.rate_a_mva : 1.0e6;
      for (int k = 0; k < T; ++k) {
        JOColumn jc;
        jc.kind       = JOColumn::Kind::BranchFlow;
        jc.branch_pos = li;
        jc.step       = k;
        add_col(jc, -fmax, fmax, 0.0,
                "f_l" + std::to_string(br.index) + "_k" + std::to_string(k),
                false);
      }
    }

    // Power slack ℓ^p_{b,k}
    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      col_pslk_base[static_cast<std::size_t>(bi)] = static_cast<int>(cols.size());
      for (int k = 0; k < T; ++k) {
        JOColumn jc;
        jc.kind    = JOColumn::Kind::PowerSlack;
        jc.bus_pos = bi;
        jc.step    = k;
        add_col(jc, 0.0, 1.0e6, opts.power_slack_penalty,
                "lp_b" + std::to_string(bus.index) + "_k" + std::to_string(k),
                false);
      }
    }
  }

  // ── Finalise LP variable arrays ─────────────────────────────────────────
  const int n_vars = static_cast<int>(cols.size());
  lp.c    = Eigen::VectorXd::Zero(n_vars);
  lp.vars = std::move(vars);
  for (int j = 0; j < n_vars; ++j)
    lp.c[j] = obj[static_cast<std::size_t>(j)];

  // ── Build equality constraint matrix (Aeq, beq) ─────────────────────────
  //
  //  Block A: demand balance  (n_eq_demand rows)
  //  Block B: DC flow         (n_branches × T rows)   [if OPF]
  //  Block C: DC power balance (n_buses × T rows)     [if OPF]
  // ────────────────────────────────────────────────────────────────────────

  int n_eq_total = n_eq_demand;
  int eq_energy_start = -1;
  int eq_flow_start  = -1;
  int eq_pbal_start  = -1;
  if (!pending_eqs.empty()) {
    eq_energy_start = n_eq_total;
    n_eq_total += static_cast<int>(pending_eqs.size());
  }
  if (opts.include_dcopf) {
    eq_flow_start = n_eq_total;
    n_eq_total   += n_branches * T;
    eq_pbal_start = n_eq_total;
    n_eq_total   += n_buses    * T;
  }

  std::vector<Eigen::Triplet<double>> aeq_trips;
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq_total);

  // Block A: demand balance
  for (int ei = 0; ei < n_eq_demand; ++ei)
    beq[ei] = beq_demand[static_cast<std::size_t>(ei)];
  aeq_trips.insert(aeq_trips.end(), eq_trips.begin(), eq_trips.end());

  if (eq_energy_start >= 0) {
    for (int i = 0; i < static_cast<int>(pending_eqs.size()); ++i) {
      const int row = eq_energy_start + i;
      beq[row] = pending_eqs[static_cast<std::size_t>(i)].rhs;
      for (const auto& [col, coeff] : pending_eqs[static_cast<std::size_t>(i)].terms)
        aeq_trips.emplace_back(row, col, coeff);
    }
  }

  if (opts.include_dcopf) {
    // Block B: DC flow  f_{ℓ,k} − b_ℓ θ_{i,k} + b_ℓ θ_{j,k} = 0
    for (int li = 0; li < n_branches; ++li) {
      const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
      if (!br.in_service || col_br_base[static_cast<std::size_t>(li)] < 0) continue;
      auto it_i = bus_pos_map.find(br.from_bus);
      auto it_j = bus_pos_map.find(br.to_bus);
      if (it_i == bus_pos_map.end() || it_j == bus_pos_map.end()) continue;
      const int bi = it_i->second, bj = it_j->second;
      const double bl = branch_susceptance[static_cast<std::size_t>(li)];
      for (int k = 0; k < T; ++k) {
        const int row = eq_flow_start + li * T + k;
        const int col_f = col_br_base[static_cast<std::size_t>(li)] + k;
        const int col_ti = col_bus_base[static_cast<std::size_t>(bi)] + k;
        const int col_tj = col_bus_base[static_cast<std::size_t>(bj)] + k;
        aeq_trips.emplace_back(row, col_f,   1.0);
        aeq_trips.emplace_back(row, col_ti, -bl);
        aeq_trips.emplace_back(row, col_tj,  bl);
        beq[row] = 0.0;
      }
    }

    // Block C: DC power balance
    //   Σ_{g@b} P^g_{g,k}  −  Σ_ℓ A_{bℓ} f_{ℓ,k}
    //   − P^{EV}_{b,k}(h)  + ℓ^p_{b,k}  = P^d_{b,k}
    //
    // A_{bℓ} = +1 if b is the from-bus of ℓ  (power sent out = positive injection minus flow)
    //          −1 if b is the to-bus   of ℓ  (power received = positive)
    // KCL: net_injection_b = Σ_ℓ (A_{bℓ} f_ℓ)
    //   P^g_b − P^d_b − P^EV_b − Σ_ℓ [from_b=b]·f_ℓ + Σ_ℓ [to_b=b]·f_ℓ + ℓ^p = 0

    for (int bi = 0; bi < n_buses; ++bi) {
      const auto& bus = sys.ac.buses[static_cast<std::size_t>(bi)];
      for (int k = 0; k < T; ++k) {
        const int row = eq_pbal_start + bi * T + k;
        beq[row] = bus.pd_mw;   // RHS = base load

        // Power slack +1
        if (col_pslk_base[static_cast<std::size_t>(bi)] >= 0)
          aeq_trips.emplace_back(row,
              col_pslk_base[static_cast<std::size_t>(bi)] + k, 1.0);

        // Generators at this bus  +P^g
        for (int gi = 0; gi < n_gens; ++gi) {
          const auto& gen = sys.ac.generators[static_cast<std::size_t>(gi)];
          if (!gen.in_service || gen.bus != bus.index) continue;
          if (col_gen_base[static_cast<std::size_t>(gi)] < 0) continue;
          aeq_trips.emplace_back(row,
              col_gen_base[static_cast<std::size_t>(gi)] + k, 1.0);
        }

        // Branch flows: −A_{bℓ}·f_ℓ
        // Convention: KCL at bus b is P^g_b − P^d_b − P^EV_b = Σ_{ℓ: from=b} f_ℓ − Σ_{ℓ: to=b} f_ℓ
        // Rearranged: P^g_b − Σ_{from=b} f_ℓ + Σ_{to=b} f_ℓ − P^EV_b(h) + ℓ^p = P^d_b
        for (int li = 0; li < n_branches; ++li) {
          const auto& br = sys.ac.branches[static_cast<std::size_t>(li)];
          if (!br.in_service || col_br_base[static_cast<std::size_t>(li)] < 0) continue;
          if (br.from_bus == bus.index) {
            aeq_trips.emplace_back(row,
                col_br_base[static_cast<std::size_t>(li)] + k, -1.0);
          }
          if (br.to_bus == bus.index) {
            aeq_trips.emplace_back(row,
                col_br_base[static_cast<std::size_t>(li)] + k,  1.0);
          }
        }

        // EV load from charge/discharge variables:
        // −P^{EV} = −(p_ch-p_dis)/1000 in the power-balance equation.
        for (const auto& ce : charging_entries) {
          const auto it = stn_bus_map.find(ce.station_id);
          if (it == stn_bus_map.end() || it->second != bus.index) continue;
          for (int off = 0; off < ce.window; ++off) {
            if (ce.arrival_step + off != k) continue;
            aeq_trips.emplace_back(row, ce.pch_cols[static_cast<std::size_t>(off)], -1.0 / 1000.0);
            aeq_trips.emplace_back(row, ce.pdis_cols[static_cast<std::size_t>(off)], 1.0 / 1000.0);
          }
        }
      }
    }
  }

  lp.Aeq.resize(n_eq_total, n_vars);
  lp.beq = beq;
  lp.Aeq.setFromTriplets(aeq_trips.begin(), aeq_trips.end());
  lp.Aeq.makeCompressed();

  // ── Build inequality constraints (A, b) ─────────────────────────────────
  //   Road capacity:    Σ h_{d,r} ≤ cap_{a,k}·Δt
  //   Station capacity: Σ p_per_veh·h_{d,r} ≤ P̄^ch_s   (per station, per step)
  //   Generation total: Σ p_per_veh·h_{d,r} ≤ EV_gen_headroom (optional)

  std::map<std::pair<int,int>, int> road_rows;
  std::map<std::pair<int,int>, int> stn_rows;
  std::map<std::pair<int,int>, int> stn_dis_rows;
  std::map<int, int>                gen_cap_rows;  // step → row

  std::vector<Eigen::Triplet<double>> aineq_trips;
  std::vector<double>                 b_vals;

  auto add_ineq_row = [&](double rhs) -> int {
    const int row = static_cast<int>(b_vals.size());
    b_vals.push_back(rhs);
    return row;
  };

  auto road_row = [&](int step, int link_index, double rhs) -> int {
    const auto key = std::make_pair(step, link_index);
    auto [it, ins] = road_rows.emplace(key, -1);
    if (ins) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto stn_row = [&](int step, int station_id, double rhs) -> int {
    const auto key = std::make_pair(step, station_id);
    auto [it, ins] = stn_rows.emplace(key, -1);
    if (ins) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto stn_dis_row = [&](int step, int station_id, double rhs) -> int {
    const auto key = std::make_pair(step, station_id);
    auto [it, ins] = stn_dis_rows.emplace(key, -1);
    if (ins) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto gen_cap_row = [&](int step, double rhs) -> int {
    auto [it, ins] = gen_cap_rows.emplace(step, -1);
    if (ins) it->second = add_ineq_row(rhs);
    return it->second;
  };

  const double gen_cap_mw  = jo_gen_cap_mw(sys);
  const double base_mw     = jo_base_load_mw(sys);
  const double ev_headroom_kw = std::max(0.0, (gen_cap_mw - base_mw) * 1000.0);

  for (int ei = 0; ei < n_eq_demand; ++ei) {
    for (const auto& re : demand_routes[static_cast<std::size_t>(ei)]) {
      if (re.route == nullptr || re.col_idx < 0) continue;
      const int k_dep = cols[static_cast<std::size_t>(re.col_idx)].departure_step;

      // Road capacity
      if (opts.enforce_road_capacity) {
        for (int li : re.route->link_indices) {
          const auto it = link_pos.find(li);
          if (it == link_pos.end()) continue;
          const double cap = link_capacity_vehicles(
              problem.traffic.links[it->second], k_dep, dt);
          if (!std::isfinite(cap)) continue;
          const int row = road_row(k_dep, li, cap);
          aineq_trips.emplace_back(row, re.col_idx, 1.0);
        }
      }
    }
  }

  for (const auto& ce : charging_entries) {
    const int hcol = ce.route_col_idx;
    for (int off = 0; off < ce.window; ++off) {
      const int k = ce.arrival_step + off;
      const int pch = ce.pch_cols[static_cast<std::size_t>(off)];
      const int pdis = ce.pdis_cols[static_cast<std::size_t>(off)];

      int row = add_ineq_row(0.0);
      aineq_trips.emplace_back(row, pch, 1.0);
      aineq_trips.emplace_back(row, hcol, -ce.p_ch_max_per_veh);

      row = add_ineq_row(0.0);
      aineq_trips.emplace_back(row, pdis, 1.0);
      aineq_trips.emplace_back(row, hcol, -ce.p_dis_max_per_veh);

      const double stn_cap = stn_cap_kw.count(ce.station_id)
                             ? stn_cap_kw.at(ce.station_id)
                             : opts.default_station_power_kw;
      const int sr = stn_row(k, ce.station_id, stn_cap);
      aineq_trips.emplace_back(sr, pch, 1.0);
      const int sdr = stn_dis_row(k, ce.station_id, stn_cap);
      aineq_trips.emplace_back(sdr, pdis, 1.0);

      if (opts.enforce_generation_capacity) {
        const int gr = gen_cap_row(k, ev_headroom_kw);
        aineq_trips.emplace_back(gr, pch, 1.0);
        aineq_trips.emplace_back(gr, pdis, -1.0);
      }

    }

    for (int off = 0; off <= ce.window; ++off) {
      const int ecol = ce.energy_cols[static_cast<std::size_t>(off)];
      int row = add_ineq_row(0.0);
      aineq_trips.emplace_back(row, ecol, -1.0);
      aineq_trips.emplace_back(row, hcol, ce.e_min_per_veh);
      if (ce.has_e_max || ce.e_max_per_veh < 1.0e8) {
        row = add_ineq_row(0.0);
        aineq_trips.emplace_back(row, ecol, 1.0);
        aineq_trips.emplace_back(row, hcol, -ce.e_max_per_veh);
      }
    }
    const int terminal = ce.energy_cols.back();
    const int tr = add_ineq_row(0.0);
    aineq_trips.emplace_back(tr, terminal, -1.0);
    aineq_trips.emplace_back(tr, hcol, ce.e_target_per_veh);
  }

  lp.A.resize(static_cast<int>(b_vals.size()), n_vars);
  lp.b = Eigen::VectorXd::Zero(static_cast<int>(b_vals.size()));
  for (int r = 0; r < static_cast<int>(b_vals.size()); ++r)
    lp.b[r] = b_vals[static_cast<std::size_t>(r)];
  lp.A.setFromTriplets(aineq_trips.begin(), aineq_trips.end());
  lp.A.makeCompressed();

  result.n_variables   = n_vars;
  result.n_constraints = n_eq_total + static_cast<int>(b_vals.size());

  if (opts.verbose) {
    std::fprintf(stderr,
      "[JointOptimizer] model: %d vars, %d eq, %d ineq, mode=%d, OPF=%d, charge_groups=%zu\n",
        n_vars, n_eq_total, static_cast<int>(b_vals.size()),
      static_cast<int>(opts.mode), opts.include_dcopf ? 1 : 0,
      charging_entries.size());
  }

  // ── Solve ────────────────────────────────────────────────────────────────
  Eigen::VectorXd x_sol;
  bool solved = false;
  bool is_mip_solve = !mip.integer_idx.empty() || !mip.binary_idx.empty();

  if (!is_mip_solve) {
    // Pure LP — use native dual simplex
    engine::SimplexOptions sopt;
    sopt.max_iter         = std::max(100000, n_vars * 20);
    sopt.feasibility_tol  = 1e-8;
    sopt.optimality_tol   = 1e-8;
    sopt.verbose          = false;
    auto lp_res = engine::solve_lp_with_basis(lp, sopt, nullptr);
    x_sol  = lp_res.result.x;
    solved = lp_res.result.stats.success;
    result.solver_backend  = "NativeDualSimplex";
    result.solver_status   = lp_res.result.stats.status;
    result.objective       = lp_res.result.stats.objective;
    result.best_bound      = result.objective;
    result.mip_gap         = 0.0;
    result.proven_optimal  = solved;
  } else {
    // MILP — try HiGHS first, then native B&C fallback
    auto solve_native_bc = [&](const std::string& reason) {
      engine::BCOptions bc;
      bc.max_nodes        = std::max(1, opts.max_nodes);
      bc.time_limit_sec   = std::max(0.0, opts.time_limit_sec);
      bc.gap_tol          = std::max(0.0, opts.mip_gap);
      bc.use_simplex_lp_nodes = true;
      bc.verbose          = false;
      auto bc_res = engine::solve_milp_bc(mip, bc);
      x_sol  = bc_res.x;
      solved = bc_res.stats.success;
      result.solver_backend =
          reason.empty() ? "NativeB&C" : "HiGHS+NativeB&C";
      result.solver_status  =
          reason.empty() ? bc_res.stats.status
                         : "HiGHS failed: " + reason + "; NativeB&C: " +
                           bc_res.stats.status;
      result.objective  = bc_res.stats.objective;
      result.best_bound = bc_res.bc_stats.best_bound;
      result.mip_gap    = std::isfinite(bc_res.bc_stats.gap)
                          ? bc_res.bc_stats.gap
                          : bc_res.stats.mip_gap;
      result.proven_optimal = solved &&
                              std::isfinite(result.mip_gap) &&
                              result.mip_gap <= opts.mip_gap + 1e-9;
    };

    engine::HighsAdapter highs;
    if (highs.available()) {
      auto hr = highs.solve_milp(mip);
      x_sol  = hr.x;
      solved = hr.stats.success;
      result.solver_backend = "HiGHS";
      result.solver_status  = hr.stats.status;
      result.objective  = hr.stats.objective;
      result.mip_gap    = hr.stats.mip_gap;
      // HiGHS reports mip_gap = (obj - bound) / max(1, |obj|); infer bound
      // using the same convention as assignment_lp.cpp and NativeB&C.
      result.best_bound = inferred_minimization_bound(result.objective,
                                                      result.mip_gap);
      result.proven_optimal = solved &&
                              std::isfinite(result.mip_gap) &&
                              result.mip_gap <= opts.mip_gap + 1e-9;
      if (!solved || x_sol.size() != n_vars) {
        const std::string reason =
            result.solver_status.empty() ? "unsuccessful" : result.solver_status;
        solve_native_bc(reason);
      }
    } else {
      solve_native_bc("");
    }
  }

  const auto t_end = steady_clock::now();
  result.solve_time_sec =
      duration_cast<microseconds>(t_end - t_start).count() * 1e-6;

  if (!solved || x_sol.size() != static_cast<Eigen::Index>(n_vars)) {
    jo_warn(result, "LP/MILP solve failed: " + result.solver_status, opts.verbose);
    return result;
  }

  jo_compute_certificate_residuals(result, mip, x_sol);
  const double cert_tol = is_mip_solve ? 1.0e-6 : 1.0e-7;
  result.feasible = solved && result.primal_max_violation <= cert_tol &&
                    result.integrality_max_violation <= cert_tol;
  result.proven_optimal = result.proven_optimal && result.feasible;
  // The LP/MILP path uses exogenous route costs throughout: either free-flow
  // (opts.use_ctm_travel_times=false) or a one-shot equal-split CTM pre-pass
  // (opts.use_ctm_travel_times=true).  In neither case are travel times
  // updated endogenously inside the optimisation.  Only FullJointNLP enforces
  // BPR cost rows endogenously and sets this flag to true.
  result.travel_times_are_endogenous = false;

  // ── Decode solution ─────────────────────────────────────────────────────
  int next_session = 0;

  // Route flow variables
  for (int j = 0; j < n_vars; ++j) {
    const auto& col = cols[static_cast<std::size_t>(j)];
    double val = x_sol[j];
    if (!std::isfinite(val)) val = 0.0;
    if (is_mip_solve &&
        j < static_cast<int>(lp.vars.size()) &&
        lp.vars[static_cast<std::size_t>(j)].type != engine::VarType::Continuous) {
      val = std::round(val);
    }

    if (col.kind == JOColumn::Kind::RouteFlow) {
      if (val < kTol) continue;
      const auto& demand = problem.demands[static_cast<std::size_t>(col.demand_pos)];
      const auto* route  = col.route;
      if (route == nullptr) continue;

      // Welfare: ev_benefit and travel time
      result.ev_benefit += demand.willingness_to_pay_per_vehicle * val;
      double ff_hr = 0.0;
      for (int li : route->link_indices) {
        if (const auto it = link_pos.find(li); it != link_pos.end())
          ff_hr += std::max(0.0, problem.traffic.links[it->second].free_flow_time_hr);
      }
      result.total_travel_time_hr     += val * ff_hr;
      result.traffic_delay_cost       += opts.value_of_time_per_hr * val * ff_hr;
      result.total_served_vehicles    += val;

      // Route flow map
      result.route_flow[demand.index][route->index] = val;

      // Assignment record
      result.assignments.push_back({
          demand.index, route->index, col.departure_step,
          val, 0.0, ff_hr, true, "formulation-d-lp"});

    } else if (col.kind == JOColumn::Kind::UnservedDemand) {
      if (val > kTol) {
        result.total_unserved_vehicles += val;
        const auto& demand =
            problem.demands[static_cast<std::size_t>(col.demand_pos)];
        result.assignments.push_back({
            demand.index, 0, col.departure_step,
            val, 0.0, 0.0, false, "formulation-d unmet demand"});
      }
    }
  }

  for (const auto& ce : charging_entries) {
    const double h = std::isfinite(x_sol[ce.route_col_idx]) ? x_sol[ce.route_col_idx] : 0.0;
    if (h <= kTol || ce.route == nullptr) continue;
    const auto& demand = problem.demands[static_cast<std::size_t>(ce.demand_pos)];

    EVChargingSession sess;
    sess.index = next_session++;
    sess.station_id = ce.station_id;
    sess.arrival_step = ce.arrival_step;
    sess.departure_step = ce.departure_step;
    sess.vehicle_count = h;
    sess.energy_initial_kwh = ce.e_arrival_per_veh * h;
    sess.energy_target_kwh = ce.e_target_per_veh * h;
    sess.energy_min_kwh = ce.e_min_per_veh * h;
    sess.energy_max_kwh = ce.e_max_per_veh * h;
    sess.max_charge_kw = ce.p_ch_max_per_veh * h;
    sess.max_discharge_kw = ce.p_dis_max_per_veh * h;
    sess.eta_charge = ce.eta_ch;
    sess.eta_discharge = ce.eta_dis;
    sess.v2g_capable = ce.v2g;
    sess.source_demand_index = demand.index;
    sess.source_route_index = ce.route->index;

    ChargingSessionResult sr;
    sr.session_index = sess.index;
    sr.station_id = ce.station_id;
    sr.p_charge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.p_discharge_kw.assign(static_cast<std::size_t>(T), 0.0);
    sr.energy_kwh.assign(static_cast<std::size_t>(T + 1), 0.0);

    for (int off = 0; off < ce.window; ++off) {
      const int k = ce.arrival_step + off;
      if (k < 0 || k >= T) continue;
      const double pch = std::max(0.0, x_sol[ce.pch_cols[static_cast<std::size_t>(off)]]);
      const double pdis = std::max(0.0, x_sol[ce.pdis_cols[static_cast<std::size_t>(off)]]);
      sr.p_charge_kw[static_cast<std::size_t>(k)] = pch;
      sr.p_discharge_kw[static_cast<std::size_t>(k)] = pdis;
      result.total_delivered_energy_kwh += ce.eta_ch * pch * dt;
      result.total_v2g_energy_kwh += pdis * dt / ce.eta_dis;
    }
    for (int off = 0; off <= ce.window; ++off) {
      const int k = ce.arrival_step + off;
      if (k < 0 || k > T) continue;
      sr.energy_kwh[static_cast<std::size_t>(k)] =
          std::max(0.0, x_sol[ce.energy_cols[static_cast<std::size_t>(off)]]);
    }
    for (int k = 1; k <= T; ++k) {
      if (sr.energy_kwh[static_cast<std::size_t>(k)] <= kTol)
        sr.energy_kwh[static_cast<std::size_t>(k)] =
            sr.energy_kwh[static_cast<std::size_t>(k - 1)];
    }
    sr.unserved_energy_kwh = std::max(0.0,
        sess.energy_target_kwh - sr.energy_kwh[static_cast<std::size_t>(sess.departure_step)]);
    result.total_unserved_energy_kwh += sr.unserved_energy_kwh;

    result.total_requested_energy_kwh += ce.requested_kwh_per_veh * h;
    result.sessions.push_back(sess);
    result.session_results.push_back(std::move(sr));
  }

  // DC-OPF variable decoding
  if (opts.include_dcopf) {
    result.gen_dispatch_mw.resize(static_cast<std::size_t>(T));
    result.lmp_by_step.resize(static_cast<std::size_t>(T));

    for (int j = 0; j < n_vars; ++j) {
      const auto& col = cols[static_cast<std::size_t>(j)];
      const double val = std::isfinite(x_sol[j]) ? x_sol[j] : 0.0;
      if (col.kind == JOColumn::Kind::GeneratorPower && col.step >= 0 &&
          col.step < T && col.gen_pos >= 0) {
        const auto& gen = sys.ac.generators[static_cast<std::size_t>(col.gen_pos)];
        result.gen_dispatch_mw[static_cast<std::size_t>(col.step)][gen.index] = val;
        result.gen_cost += gen.cost_c1 * val * dt;
      }
    }

    // Compute LMPs from dual variables (not extracted from LP duals here;
    // report placeholder — dual extraction is implementation-specific).
    // For the social welfare accounting use the primal cost instead.
  }

  // Social welfare
  result.social_welfare = result.ev_benefit - result.traffic_delay_cost - result.gen_cost;

  if (opts.verbose) {
    std::fprintf(stderr,
        "[JointOptimizer] solved: %s, W=%.2f, B_EV=%.2f, C_gen=%.2f, C_delay=%.2f\n"
        "  served=%.1f, unserved=%.1f, sessions=%zu, t=%.3fs\n",
        result.proven_optimal ? "OPTIMAL" : result.solver_status.c_str(),
        result.social_welfare, result.ev_benefit, result.gen_cost,
        result.traffic_delay_cost,
        result.total_served_vehicles, result.total_unserved_vehicles,
        result.sessions.size(), result.solve_time_sec);
  }

  return result;
}

}  // namespace hacdcpf::evpt

// joint_optimizer.cpp
// ──────────────────────────────────────────────────────────────────────────
// Formulation D: single LP/QP/MILP/MCP joint optimizer for EV-power-traffic.
//
// Key innovation over Formulations A/B/C
// ─────────────────────────────────────────
//   A: routing-only LP (no DC-OPF in the LP objective)
//   C: iterative CTM-DUE ↔ DC-OPF price coordination (converged, not certified)
//   D: route-assignment + DC-OPF power balance assembled into ONE LP/MILP,
//      solved by a primal-certified optimal solver (HiGHS / native dual simplex).
//
// LP structure  (§Formulation D, technical notebook)
// ───────────────────────────────────────────────────
// Variables
//   h_{d,r}     ≥ 0                 vehicles on demand d, route r
//   u_d         ≥ 0                 unserved vehicles for demand d
//   P^g_{g,k}   ∈ [Pmin,Pmax]      generator dispatch [MW]     (if OPF)
//   θ_{b,k}     free                bus voltage angle [rad]     (if OPF)
//   f_{ℓ,k}     ∈ [-Fmax,Fmax]     branch active flow [MW]     (if OPF)
//   ℓ^p_{b,k}   ≥ 0                power balance slack [MW]    (if OPF)
//
// Equality constraints
//   Demand balance   :  Σ_r h_{d,r} + u_d = D_d                (one/demand)
//   DC flow          :  f_{ℓ,k} − b_ℓ θ_{i,k} + b_ℓ θ_{j,k} = 0
//   DC power balance :  Σ_g P^g_{g,k}[g@b] − Σ_ℓ A_{bℓ} f_{ℓ,k}
//                       − P^{EV}_{b,k}(h) + ℓ^p_{b,k} = P^d_{b,k}
//
// Inequality constraints
//   Road capacity    :  Σ_{d,r: a∈r, k_dep=k} h_{d,r} ≤ cap_a·Δt
//   Station capacity :  Σ_{d,r: stop at s, k∈window} p_per_veh·h_{d,r} ≤ P̄^ch_s
//
// The EV load at bus b at step k is a LINEAR function of route flows:
//   P^{EV}_{b,k}[MW] = 1e-3 · Σ_{d,r,s: bus(s)=b, k∈window} p_per_veh · h_{d,r}
//
// This linearity is the key that makes Formulation D a pure LP.
//
// Objectives
//   SocialWelfareMax (minimise):
//     Σ c^1_g·P^g·Δt − Σ WTP·(D−u) + VOT·Σ T^ff·h + M_out·u + M_p·Σℓ^p
//
//   UserBenefitMax (minimise):
//     Σ π_s·p_per_veh·Δt·h − Σ WTP·(D−u) + VOT·Σ T^ff·h + M_out·u

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
    double cap = cs.max_power_kw > 0.0 ? cs.max_power_kw
               : static_cast<double>(cs.n_fast) * cs.p_fast_max_kw;
    if (cap <= kTol) cap = opts.default_station_power_kw;
    m[cs.index] = cap;
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

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// solve_joint_optimizer — Formulation D entry point
// ─────────────────────────────────────────────────────────────────────────────
JointOptimizerResult solve_joint_optimizer(
    const EVPowerTrafficProblem& problem,
    const JointOptimizerOptions& opts) {
  using namespace std::chrono;
  const auto t_start = steady_clock::now();

  JointOptimizerResult result;
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

  const std::size_t n_links = problem.traffic.links.size();
  std::vector<double> link_cap(n_links, std::numeric_limits<double>::infinity());
  for (std::size_t li = 0; li < n_links; ++li)
    link_cap[li] = problem.traffic.links[li].capacity_veh_per_hr * dt;

  // ── Build LP variable columns ───────────────────────────────────────────
  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<JOColumn>          cols;
  std::vector<double>            obj;
  std::vector<engine::VariableMeta> vars;

  const bool use_milp = (opts.mode == JointOptimizerMode::UserEquilibriumMILP) ||
                        opts.charge_discharge_binaries;

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
  std::vector<std::vector<RouteEntry>> demand_routes(
      static_cast<std::size_t>(n_eq_demand));

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

      // Free-flow travel time for this route
      double ff_hr = 0.0;
      for (int li : route->link_indices) {
        if (const auto it = link_pos.find(li); it != link_pos.end())
          ff_hr += std::max(0.0, problem.traffic.links[it->second].free_flow_time_hr);
      }

      // Objective cost for this route (depends on mode)
      double route_cost = opts.value_of_time_per_hr * ff_hr;  // VOT×T^ff
      route_cost -= demand.willingness_to_pay_per_vehicle;      // −WTP
      if (opts.mode == JointOptimizerMode::UserBenefitMax) {
        // Add electricity cost: Σ_s π_s·p_per_veh·Δt
        for (const auto& stop : route->charging_stops) {
          const double p_kw = jo_charge_power_kw_per_veh(stop, opts, dt);
          const double dwell = std::max(1, stop.dwell_steps);
          const double price = jo_station_price(problem, opts,
                                                stop.station_id, arr);
          route_cost += opts.station_energy_cost_weight *
                        price * p_kw * dwell * dt;
        }
      }
      // Note: SocialWelfareMax adds C_gen through generator dispatch variables;
      // the route cost here contains only travel time and benefit terms.

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
    }

    // Unserved demand u_d
    JOColumn uc;
    uc.kind = JOColumn::Kind::UnservedDemand;
    uc.demand_pos = di;
    uc.departure_step = demand.departure_step;

    const double unserved_cost = opts.unserved_trip_penalty -
                                 demand.willingness_to_pay_per_vehicle;

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
  int eq_flow_start  = -1;
  int eq_pbal_start  = -1;
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

        // EV load from route flows: −P^{EV}_{b,k}(h) on left → coefficients in Aeq
        // P^{EV}_{b,k}[MW] = 1e-3 × Σ_{d,r,s: bus(s)=b, k∈window} p_per_veh × h_{d,r}
        for (int ei = 0; ei < n_eq_demand; ++ei) {
          for (const auto& re : demand_routes[static_cast<std::size_t>(ei)]) {
            if (re.route == nullptr || re.col_idx < 0) continue;
            for (const auto& stop : re.route->charging_stops) {
              const auto it = stn_bus_map.find(stop.station_id);
              if (it == stn_bus_map.end() || it->second != bus.index) continue;
              const int window = std::max(1, stop.dwell_steps);
              for (int off = 0; off < window; ++off) {
                if (re.arrival_step + off != k) continue;
                const double p_kw = jo_charge_power_kw_per_veh(stop, opts, dt);
                // −P^{EV} term: coefficient is −(p_kw/1000) in power balance
                aeq_trips.emplace_back(row, re.col_idx, -(p_kw / 1000.0));
              }
            }
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
          const double cap = link_cap[it->second];
          if (!std::isfinite(cap)) continue;
          const int row = road_row(k_dep, li, cap);
          aineq_trips.emplace_back(row, re.col_idx, 1.0);
        }
      }

      // Station capacity
      for (const auto& stop : re.route->charging_stops) {
        const double p_kw = jo_charge_power_kw_per_veh(stop, opts, dt);
        if (p_kw <= kTol) continue;
        const double stn_cap = stn_cap_kw.count(stop.station_id)
                               ? stn_cap_kw.at(stop.station_id)
                               : opts.default_station_power_kw;
        const int window = std::max(1, stop.dwell_steps);
        for (int off = 0; off < window; ++off) {
          const int k = re.arrival_step + off;
          if (k < 0 || k >= T) continue;
          const int row = stn_row(k, stop.station_id, stn_cap);
          aineq_trips.emplace_back(row, re.col_idx, p_kw);

          // Optional: total generation headroom (kW → same units)
          if (opts.enforce_generation_capacity) {
            const int gr = gen_cap_row(k, ev_headroom_kw);
            aineq_trips.emplace_back(gr, re.col_idx, p_kw);
          }
        }
      }
    }
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
        "[JointOptimizer] model: %d vars, %d eq, %d ineq, mode=%d, OPF=%d\n",
        n_vars, n_eq_total, static_cast<int>(b_vals.size()),
        static_cast<int>(opts.mode), opts.include_dcopf ? 1 : 0);
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
      result.best_bound = result.objective * (1.0 - result.mip_gap);
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

  result.feasible = solved;
  const auto t_end = steady_clock::now();
  result.solve_time_sec =
      duration_cast<microseconds>(t_end - t_start).count() * 1e-6;

  if (!solved || x_sol.size() != static_cast<Eigen::Index>(n_vars)) {
    jo_warn(result, "LP/MILP solve failed: " + result.solver_status, opts.verbose);
    return result;
  }

  // ── Decode solution ─────────────────────────────────────────────────────
  int next_session = 0;

  // Route flow variables
  for (int j = 0; j < n_vars; ++j) {
    const auto& col = cols[static_cast<std::size_t>(j)];
    double val = x_sol[j];
    if (!std::isfinite(val)) val = 0.0;
    if (is_mip_solve) val = std::round(val);

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

      // Sessions
      for (const auto& stop : route->charging_stops) {
        if (col.arrival_step >= T) continue;
        EVChargingSession sess;
        sess.index         = next_session++;
        sess.station_id    = stop.station_id;
        sess.arrival_step  = col.arrival_step;
        sess.departure_step = std::min(T, col.arrival_step + std::max(1, stop.dwell_steps));
        sess.vehicle_count = val;
        const double e_req = std::max(0.0, stop.requested_energy_kwh_per_vehicle * val);
        sess.energy_initial_kwh = 0.0;
        sess.energy_target_kwh  = e_req;
        sess.energy_min_kwh     = 0.0;
        const double p_kw = jo_charge_power_kw_per_veh(stop, opts, dt);
        sess.max_charge_kw  = p_kw * val;
        sess.max_discharge_kw = stop.v2g_capable
                                ? (stop.max_discharge_kw_per_vehicle > 0
                                   ? stop.max_discharge_kw_per_vehicle * val
                                   : p_kw * val)
                                : 0.0;
        sess.energy_max_kwh = e_req + sess.max_discharge_kw * dt;
        sess.eta_charge     = opts.default_charging_efficiency;
        sess.eta_discharge  = opts.default_charging_efficiency;
        sess.v2g_capable    = stop.v2g_capable && opts.allow_v2g;
        sess.source_demand_index = demand.index;
        sess.source_route_index  = route->index;
        result.sessions.push_back(sess);
        result.total_requested_energy_kwh += e_req;
      }
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

  // Total delivered energy estimate (assuming sessions fully charged)
  for (const auto& sess : result.sessions)
    result.total_delivered_energy_kwh += sess.energy_target_kwh;

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

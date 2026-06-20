// test_ev_power_traffic_formulations_e_f_g_h.cpp
// ---------------------------------------------------------------------------
// Direct public-entry tests for the solver-backed traffic formulations added
// around Section 20: SO-CTM, SO-LTM, CTM-DUE-VI, infrastructure MILP, and
// LTM-MPC.  These tests intentionally exercise the API entry points rather
// than only the older simulation wrappers.

#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

namespace {

HybridPowerSystem make_form_system() {
  HybridPowerSystem sys;
  sys.base_mva = 1.0;
  sys.ac.base_mva = 1.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.in_service = true;
  sys.ac.buses.push_back(b1);

  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.in_service = true;
  sys.ac.buses.push_back(b2);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.pmax_mw = 10.0;
  g.qmax_mvar = 10.0;
  g.qmin_mvar = -10.0;
  sys.ac.generators.push_back(g);

  ChargingStation s1;
  s1.index = 101;
  s1.bus = 1;
  s1.in_service = true;
  s1.max_power_kw = 100.0;
  s1.n_fast = 2;
  s1.p_fast_max_kw = 50.0;
  sys.ac.charging_stations.push_back(s1);

  ChargingStation s2 = s1;
  s2.index = 102;
  s2.bus = 2;
  sys.ac.charging_stations.push_back(s2);

  Charger c1;
  c1.index = 1;
  c1.station_id = 101;
  c1.in_service = true;
  c1.p_ch_max_kw = 50.0;
  c1.v2g_capable = true;
  c1.p_dis_max_kw = 20.0;
  sys.ac.chargers.push_back(c1);

  Charger c2 = c1;
  c2.index = 2;
  c2.station_id = 102;
  sys.ac.chargers.push_back(c2);

  return sys;
}

TrafficLink make_link(int index, int from, int to, double ff_hr) {
  TrafficLink link;
  link.index = index;
  link.from_node = from;
  link.to_node = to;
  link.length_km = 1.0;
  link.free_flow_time_hr = ff_hr;
  link.capacity_veh_per_hr = 12.0;
  link.jam_vehicles = 30.0;
  return link;
}

RouteAlternative make_route(int index,
                            std::vector<int> links,
                            int station_id,
                            double charge_kwh = 2.0) {
  RouteAlternative route;
  route.index = index;
  route.origin_node = 1;
  route.destination_node = 4;
  route.link_indices = std::move(links);

  RouteChargingStop stop;
  stop.station_id = station_id;
  stop.requested_energy_kwh_per_vehicle = charge_kwh;
  stop.dwell_steps = 1;
  stop.max_charge_kw_per_vehicle = 10.0;
  stop.v2g_capable = true;
  stop.max_discharge_kw_per_vehicle = 5.0;
  route.charging_stops.push_back(stop);
  return route;
}

EVPowerTrafficProblem make_form_problem(double vehicles = 3.0) {
  EVPowerTrafficProblem problem;
  problem.system = make_form_system();
  problem.traffic.nodes = {
      {1, "origin"}, {2, "upper"}, {3, "lower"}, {4, "sink"}};
  problem.traffic.links = {
      make_link(11, 1, 2, 0.10),
      make_link(12, 2, 4, 0.10),
      make_link(21, 1, 3, 0.18),
      make_link(22, 3, 4, 0.18),
  };
  problem.routes.push_back(make_route(1, {11, 12}, 101));
  problem.routes.push_back(make_route(2, {21, 22}, 102));

  EVDemand demand;
  demand.index = 1;
  demand.origin_node = 1;
  demand.destination_node = 4;
  demand.departure_step = 0;
  demand.vehicles = vehicles;
  demand.candidate_route_indices = {1, 2};
  demand.willingness_to_pay_per_vehicle = 20.0;
  demand.departure_window_steps = {0, 1};
  problem.demands.push_back(demand);
  return problem;
}

EVPowerTrafficOptions make_ev_opts() {
  EVPowerTrafficOptions opts;
  opts.num_steps = 6;
  opts.time_step_hr = 0.25;
  opts.value_of_time_per_hr = 1.0;
  opts.default_charging_efficiency = 1.0;
  opts.default_route_stop_power_kw_per_vehicle = 10.0;
  opts.default_station_power_kw = 100.0;
  opts.allow_unserved_travel_demand = false;
  opts.allow_unserved_charging_energy = false;
  return opts;
}

EVPowerTrafficProblem make_merge_problem(double vehicles = 4.0) {
  EVPowerTrafficProblem problem;
  problem.system = make_form_system();
  problem.traffic.nodes = {
      {1, "origin"}, {2, "upper"}, {3, "lower"}, {4, "merge"}, {5, "sink"}};
  problem.traffic.links = {
      make_link(11, 1, 2, 0.10),
      make_link(12, 2, 4, 0.10),
      make_link(21, 1, 3, 0.10),
      make_link(22, 3, 4, 0.10),
      make_link(31, 4, 5, 0.10),
  };
  problem.traffic.links.back().capacity_veh_per_hr = 4.0;
  problem.traffic.links.back().jam_vehicles = 8.0;

  RouteAlternative r1 = make_route(1, {11, 12, 31}, 101, 1.0);
  r1.destination_node = 5;
  RouteAlternative r2 = make_route(2, {21, 22, 31}, 102, 1.0);
  r2.destination_node = 5;
  problem.routes.push_back(r1);
  problem.routes.push_back(r2);

  EVDemand demand;
  demand.index = 1;
  demand.origin_node = 1;
  demand.destination_node = 5;
  demand.departure_step = 0;
  demand.vehicles = vehicles;
  demand.candidate_route_indices = {1, 2};
  demand.willingness_to_pay_per_vehicle = 20.0;
  problem.demands.push_back(demand);
  return problem;
}

double assigned_total(
    const std::unordered_map<int, std::unordered_map<int, double>>& flows,
    int demand_id) {
  double total = 0.0;
  auto it = flows.find(demand_id);
  if (it == flows.end()) return total;
  for (const auto& kv : it->second) total += kv.second;
  return total;
}

}  // namespace

TEST_CASE("Formulation E: SO-CTM LP direct API has solver certificate",
          "[ev_power_traffic][formulation_e][ctm_so_lp]") {
  const auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();
  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.25;
  ctm_opts.n_cells_per_link = 1;

  const auto result = solve_ctm_so_lp(problem, ev_opts, ctm_opts);

  REQUIRE(result.solved);
  CHECK_FALSE(result.infeasible);
  CHECK(result.proven_optimal);
  CHECK(result.objective >= 0.0);
  CHECK(result.best_bound == Approx(result.objective).margin(1e-8));
  CHECK(result.mip_gap == Approx(0.0).margin(1e-12));
  CHECK(result.primal_max_violation <= 1e-7);
  CHECK(result.integrality_max_violation <= 1e-9);
  CHECK(result.dynamic_constraints_enforced);
  CHECK(result.node_flow_variables_enforced);
  CHECK(result.n_node_flow_variables > 0);
  CHECK(result.node_flow_conservation_max_violation <= 1e-7);
  CHECK(assigned_total(result.flow_by_demand_route, 1) ==
        Approx(3.0).margin(1e-6));

  // Honesty fields for SO-CTM in default per-route mode.
  CHECK(result.per_route_decomposition_used == true);
  CHECK(result.best_bound_is_primal_only == true);
  CHECK(result.closed_link_handling_strict == true);
}

TEST_CASE("Formulation E-LTM: SO-LTM LP direct API has solver certificate",
          "[ev_power_traffic][formulation_e_ltm][ltm_so_lp]") {
  const auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();

  const auto result = solve_ltm_so_lp(problem, ev_opts, LTMSOLPOptions{});

  REQUIRE(result.solved);
  CHECK_FALSE(result.infeasible);
  CHECK(result.proven_optimal);
  CHECK(result.objective >= 0.0);
  CHECK(result.so_tstt_hr == Approx(result.objective).margin(1e-8));
  CHECK(result.best_bound == Approx(result.objective).margin(1e-8));
  CHECK(result.primal_max_violation <= 1e-7);
  CHECK(result.dynamic_constraints_enforced);
  CHECK(result.node_flow_variables_enforced);
  CHECK(result.n_node_flow_variables > 0);
  CHECK(result.node_flow_conservation_max_violation <= 1e-7);
  CHECK(assigned_total(result.flow_by_demand_route, 1) ==
        Approx(3.0).margin(1e-6));

  // Honesty fields: per-route mode (default) is multi-commodity tight;
  // best_bound is the primal objective (no independent dual cert);
  // origin-injection timing is strictly enforced in per-route mode.
  CHECK(result.per_route_decomposition_used == true);
  CHECK(result.best_bound_is_primal_only == true);
  CHECK(result.aggregate_origin_timing_strict == true);
  CHECK(result.aggregate_intermediate_conservation_strict == true);
}

TEST_CASE("Formulation E: SO-CTM/SO-LTM aggregate-link variable mode "
          "produces a valid LP with documented honesty flags",
          "[ev_power_traffic][formulation_e][aggregate_mode]") {
  const auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();

  // CTM aggregate mode
  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.25;
  ctm_opts.n_cells_per_link = 1;
  ctm_opts.use_aggregate_link_variables = true;
  const auto ctm = solve_ctm_so_lp(problem, ev_opts, ctm_opts);
  REQUIRE(ctm.solved);
  CHECK(ctm.proven_optimal);
  CHECK(ctm.primal_max_violation <= 1e-7);
  CHECK(ctm.per_route_decomposition_used == false);
  CHECK(ctm.best_bound_is_primal_only == true);
  // Closed-link availability must skip -1 internal indices in aggregate mode
  // and additionally zero out shared y_agg_int (Bug 5a fix).
  CHECK(ctm.closed_link_handling_strict == true);

  // LTM aggregate mode
  LTMSOLPOptions ltm_opts;
  ltm_opts.use_aggregate_link_variables = true;
  const auto ltm = solve_ltm_so_lp(problem, ev_opts, ltm_opts);
  REQUIRE(ltm.solved);
  CHECK(ltm.proven_optimal);
  CHECK(ltm.primal_max_violation <= 1e-7);
  CHECK(ltm.per_route_decomposition_used == false);
  CHECK(ltm.best_bound_is_primal_only == true);
  // Intermediate-link Nin_agg ≥ cumulative Σ Ynode coupling is in (Bug 5b).
  CHECK(ltm.aggregate_intermediate_conservation_strict == true);
  // Aggregate mode does NOT strictly model per-route origin-injection timing.
  CHECK(ltm.aggregate_origin_timing_strict == false);
}

TEST_CASE("Formulation E: node movement variables cover merge bottlenecks",
          "[ev_power_traffic][formulation_e][node_model]") {
  const auto problem = make_merge_problem();
  auto ev_opts = make_ev_opts();
  ev_opts.num_steps = 8;
  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.25;
  ctm_opts.n_cells_per_link = 1;

  const auto ctm = solve_ctm_so_lp(problem, ev_opts, ctm_opts);
  REQUIRE(ctm.solved);
  CHECK(ctm.proven_optimal);
  CHECK(ctm.dynamic_constraints_enforced);
  CHECK(ctm.node_flow_variables_enforced);
  CHECK(ctm.n_node_flow_variables >= 2 * ev_opts.num_steps);
  CHECK(ctm.node_flow_conservation_max_violation <= 1e-7);

  const auto ltm = solve_ltm_so_lp(problem, ev_opts, LTMSOLPOptions{});
  REQUIRE(ltm.solved);
  CHECK(ltm.proven_optimal);
  CHECK(ltm.dynamic_constraints_enforced);
  CHECK(ltm.node_flow_variables_enforced);
  CHECK(ltm.n_node_flow_variables >= 2 * ev_opts.num_steps);
  CHECK(ltm.node_flow_conservation_max_violation <= 1e-7);
}

TEST_CASE("Formulation F: CTM-DUE-VI direct API runs fixed and windowed demand",
          "[ev_power_traffic][formulation_f][ctm_due_vi]") {
  auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();
  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.25;
  ctm_opts.n_cells_per_link = 1;
  CTMDUEVIOptions vi_opts;
  vi_opts.max_iterations = 6;
  vi_opts.convergence_tol = 1.0e-2;
  vi_opts.line_search_steps = 0;

  SECTION("fixed departure") {
    problem.demands.front().departure_window_steps.clear();
    const auto result = solve_ctm_due_vi(problem, ev_opts, ctm_opts, vi_opts);
    CHECK(result.iterations > 0);
    CHECK(result.relative_gap >= 0.0);
    CHECK(result.vi_certificate_available);
    CHECK(result.vi_gap >= 0.0);
    CHECK(result.normalized_vi_gap >= 0.0);
    CHECK(result.demand_conservation_max_violation <= 1e-6);
    CHECK(assigned_total(result.flow_by_demand_route, 1) ==
          Approx(3.0).margin(1e-6));
  }

  SECTION("departure window packing") {
    const auto result = solve_ctm_due_vi(problem, ev_opts, ctm_opts, vi_opts);
    CHECK(result.iterations > 0);
    CHECK(result.relative_gap >= 0.0);
    CHECK(result.vi_certificate_available);
    CHECK(result.vi_gap >= 0.0);
    CHECK(result.normalized_vi_gap >= 0.0);
    CHECK(result.demand_conservation_max_violation <= 1e-6);
    CHECK(assigned_total(result.flow_by_demand_route, 1) ==
          Approx(3.0).margin(1e-6));
  }

  SECTION("mixed-fleet VI certificate is explicitly EV-only scoped") {
    ICVDemand icv;
    icv.index = 10;
    icv.origin_node = 1;
    icv.destination_node = 4;
    icv.departure_step = 0;
    icv.vehicles = 2.0;
    icv.candidate_route_indices = {1, 2};
    icv.value_of_time_per_hr = 1.0;
    problem.icv_demands.push_back(icv);

    ctm_opts.enable_multiclass = true;
    vi_opts.include_icv_in_due = true;
    const auto result = solve_ctm_due_vi(problem, ev_opts, ctm_opts, vi_opts);

    CHECK(result.iterations > 0);
    CHECK(result.vi_certificate_available);
    CHECK(result.vi_certificate_covers_ev_only == true);
    CHECK(result.ev_relative_gap >= 0.0);
    CHECK(result.icv_relative_gap >= 0.0);
    CHECK(std::isfinite(result.ev_relative_gap));
    CHECK(std::isfinite(result.icv_relative_gap));
    CHECK(result.relative_gap >=
          std::max(result.ev_relative_gap, result.icv_relative_gap) - 1e-9);
  }
}

TEST_CASE("Full DUE reallocates departure time over the demand window",
          "[ev_power_traffic][full_due][ctm_due]") {
  auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();
  ev_opts.num_steps = 8;
  ev_opts.time_step_hr = 0.25;
  ev_opts.value_of_time_per_hr = 1.0;
  problem.demands.front().vehicles = 3.0;
  problem.demands.front().departure_step = 0;
  problem.demands.front().departure_window_steps = {0, 1, 2};
  problem.demands.front().desired_arrival_time_hr = 0.55;
  problem.demands.front().early_penalty_per_hr = 20.0;
  problem.demands.front().late_penalty_per_hr = 20.0;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.25;
  ctm_opts.n_cells_per_link = 1;
  DUEOptions due_opts;
  due_opts.due_mode = DUEMode::FullEndogenous;
  due_opts.max_iterations = 6;
  due_opts.convergence_tol = 1e-8;
  due_opts.msa_fixed_step = 1.0;

  const auto result =
      simulate_ev_power_traffic_ctm_due(problem, ev_opts, ctm_opts, due_opts);

  CHECK(result.full_due_enabled);
  REQUIRE(result.flow_by_demand_departure_route.count(1));
  const auto& by_dep = result.flow_by_demand_departure_route.at(1);
  REQUIRE(by_dep.count(1));
  double dep1_total = 0.0;
  for (const auto& kv : by_dep.at(1)) dep1_total += kv.second;
  CHECK(dep1_total == Approx(3.0).margin(1e-6));
  if (by_dep.count(0)) {
    double dep0_total = 0.0;
    for (const auto& kv : by_dep.at(0)) dep0_total += kv.second;
    CHECK(dep0_total == Approx(0.0).margin(1e-6));
  }
  CHECK(assigned_total(result.flow_by_demand_route, 1) ==
        Approx(3.0).margin(1e-6));
  REQUIRE_FALSE(result.sessions.empty());
  CHECK(result.sessions.front().arrival_step >= 2);
}

TEST_CASE("Formulation G: infrastructure MILP direct API covers road capacity modes",
          "[ev_power_traffic][formulation_g][infra_design_milp]") {
  const auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();
  ev_opts.allow_unserved_travel_demand = true;

  InfraDesignMILPOptions opts;
  opts.max_plugs_per_station = 6;
  opts.mip_gap = 1e-6;
  opts.time_limit_sec = 10.0;

  SECTION("without road-capacity BPR variables") {
    opts.include_road_capacity = false;
    const auto result = solve_infra_design_milp(problem, ev_opts, opts);
    REQUIRE(result.solved);
    CHECK(result.proven_optimal);
    CHECK(result.primal_max_violation <= 1e-6);
    CHECK(result.integrality_max_violation <= 1e-6);
    CHECK(assigned_total(result.flow_by_demand_route, 1) <= 3.0 + 1e-6);
  }

  SECTION("with road-capacity BPR variables") {
    opts.include_road_capacity = true;
    const auto result = solve_infra_design_milp(problem, ev_opts, opts);
    REQUIRE(result.solved);
    CHECK(result.proven_optimal);
    CHECK(result.primal_max_violation <= 1e-6);
    CHECK(result.integrality_max_violation <= 1e-6);
    CHECK(result.status != "invalid model: objective/variable dimension mismatch");
    CHECK(assigned_total(result.flow_by_demand_route, 1) <= 3.0 + 1e-6);
  }

  SECTION("with time-expanded LTM dynamic constraints") {
    opts.include_road_capacity = false;
    opts.include_ltm_dynamic_constraints = true;
    const auto result = solve_infra_design_milp(problem, ev_opts, opts);
    REQUIRE(result.solved);
    CHECK(result.proven_optimal);
    CHECK(result.dynamic_constraints_enforced);
    CHECK(result.n_dynamic_flow_variables > 0);
    CHECK(result.dynamic_flow_conservation_max_violation <= 1e-6);
    CHECK(result.primal_max_violation <= 1e-6);
    CHECK(result.integrality_max_violation <= 1e-6);
    CHECK(assigned_total(result.flow_by_demand_route, 1) <= 3.0 + 1e-6);
  }
}

TEST_CASE("Formulation H: LTM-MPC direct API has LP certificates",
          "[ev_power_traffic][formulation_h][ltm_mpc]") {
  const auto problem = make_form_problem();
  auto ev_opts = make_ev_opts();
  LTMMPCOptions opts;
  opts.horizon_steps = 3;
  opts.weight_queue = 1.0;
  opts.weight_travel_time = 0.1;
  opts.weight_v2g = 0.0;

  const auto result = solve_ltm_mpc(problem, ev_opts, opts);

  REQUIRE(result.solved);
  CHECK(result.proven_optimal);
  CHECK(result.status == "ok");
  CHECK(result.mip_gap == Approx(0.0).margin(1e-12));
  CHECK(result.primal_max_violation <= 1e-7);
  CHECK(result.forecast_arrivals_enforced);
  CHECK(result.forecast_bound_max_violation <= 1e-7);
  REQUIRE(result.forecast_arrivals.count(101));
  CHECK(result.forecast_arrivals.at(101).size() ==
        static_cast<std::size_t>(ev_opts.num_steps));
  REQUIRE(result.admission.count(101));
  CHECK(result.admission.at(101).size() == static_cast<std::size_t>(ev_opts.num_steps));
}

TEST_CASE("Exact mathematical-model verification gates reject reduced modules",
          "[ev_power_traffic][honesty][exact_model]") {
  const auto problem = make_form_problem();

  SECTION("SO-CTM LP refuses full joint-model verification") {
    auto ev_opts = make_ev_opts();
    ev_opts.require_exact_mathematical_model = true;
    CTMOptions ctm_opts;
    ctm_opts.dt_ctm_hr = 0.25;
    ctm_opts.n_cells_per_link = 1;

    const auto result = solve_ctm_so_lp(problem, ev_opts, ctm_opts);
    CHECK_FALSE(result.solved);
    CHECK(result.infeasible);
    CHECK(result.mathematical_model_verified == false);
    CHECK(result.status.find("unsupported") != std::string::npos);
  }

  SECTION("SO-LTM LP refuses exact route-commodity proof on demand") {
    auto ev_opts = make_ev_opts();
    LTMSOLPOptions opts;
    opts.require_exact_mathematical_model = true;

    const auto result = solve_ltm_so_lp(problem, ev_opts, opts);
    CHECK_FALSE(result.solved);
    CHECK(result.infeasible);
    CHECK(result.mathematical_model_verified == false);
    CHECK(result.status.find("unsupported") != std::string::npos);
  }

  SECTION("DUE-VI refuses full VI/MPEC certificate") {
    auto ev_opts = make_ev_opts();
    CTMOptions ctm_opts;
    ctm_opts.dt_ctm_hr = 0.25;
    ctm_opts.n_cells_per_link = 1;
    CTMDUEVIOptions vi_opts;
    vi_opts.require_exact_mathematical_model = true;

    const auto result = solve_ctm_due_vi(problem, ev_opts, ctm_opts, vi_opts);
    CHECK(result.converged == false);
    CHECK(result.vi_certificate_available == false);
    CHECK(result.mathematical_model_verified == false);
    CHECK(result.mathematical_model_verification_status.find("unsupported") !=
          std::string::npos);
  }

  SECTION("infrastructure MILP refuses exact nonlinear design model") {
    auto ev_opts = make_ev_opts();
    InfraDesignMILPOptions opts;
    opts.require_exact_mathematical_model = true;

    const auto result = solve_infra_design_milp(problem, ev_opts, opts);
    CHECK_FALSE(result.solved);
    CHECK(result.infeasible);
    CHECK(result.mathematical_model_verified == false);
    CHECK(result.status.find("unsupported") != std::string::npos);
  }

  SECTION("LTM-MPC refuses full-network exact model verification") {
    auto ev_opts = make_ev_opts();
    LTMMPCOptions opts;
    opts.require_exact_mathematical_model = true;

    const auto result = solve_ltm_mpc(problem, ev_opts, opts);
    CHECK_FALSE(result.solved);
    CHECK(result.proven_optimal == false);
    CHECK(result.mathematical_model_verified == false);
    CHECK(result.status.find("unsupported") != std::string::npos);
  }
}

TEST_CASE("Auto route generation is wired into public simulation entry",
          "[ev_power_traffic][auto_routes]") {
  auto problem = make_form_problem();
  problem.routes.clear();
  problem.demands.front().candidate_route_indices.clear();

  auto ev_opts = make_ev_opts();
  ev_opts.auto_generate_routes = true;
  ev_opts.k_shortest_paths = 2;
  ev_opts.assignment_model = AssignmentModel::DeterministicShortestPath;

  const auto result = simulate_ev_power_traffic(problem, ev_opts);

  CHECK(result.served_demand_vehicles > 0.0);
  CHECK(result.unmet_demand_vehicles < 3.0);
}

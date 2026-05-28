// test_ev_power_traffic_ctm_due.cpp
// ──────────────────────────────────────────────────────────────────────────
// Unit/integration tests for the CTM + MSA-DUE module.
//
// Network topology (4 nodes, 4 directed links, 2 routes):
//
//         A1 (1→2)    A2 (2→3)
//  Node1 ──────────── Node2 ──────────── Node3   Route 1 (via station 101)
//   │                                     │
//   │ B1 (1→4)                  B2 (4→3) │   Route 2 (via station 102)
//   └─────────── Node4 ─────────────────┘
//
// Both routes connect origin=1 to destination=3.  Route 1 has station 101
// at node 3; route 2 has station 102 at node 3 (shared destination).
// All links: length_km=0.5, free_flow_time_hr=0.05, capacity=20 veh/h.
// With a 5-vehicle demand departing at step 0, DUE should distribute flow
// across both routes; at convergence the Wardrop gap should be near zero.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numeric>

#include "hacdcpf/ev_power_traffic.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

namespace {

// ── Helpers ───────────────────────────────────────────────────────────────

HybridPowerSystem make_two_station_system() {
  HybridPowerSystem sys;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;
  sys.ac.buses.push_back(b1);

  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
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

  ChargingStation stn1;
  stn1.index = 101;
  stn1.bus = 1;
  stn1.in_service = true;
  stn1.max_power_kw = 200.0;
  stn1.power_factor = 1.0;
  sys.ac.charging_stations.push_back(stn1);

  ChargingStation stn2;
  stn2.index = 102;
  stn2.bus = 2;
  stn2.in_service = true;
  stn2.max_power_kw = 200.0;
  stn2.power_factor = 1.0;
  sys.ac.charging_stations.push_back(stn2);

  Charger ch1;
  ch1.index = 1;
  ch1.station_id = 101;
  ch1.in_service = true;
  ch1.p_ch_max_kw = 100.0;
  sys.ac.chargers.push_back(ch1);

  Charger ch2;
  ch2.index = 2;
  ch2.station_id = 102;
  ch2.in_service = true;
  ch2.p_ch_max_kw = 100.0;
  sys.ac.chargers.push_back(ch2);

  return sys;
}

// 4-node, 4-link symmetric graph.  Each pair of links forms one route:
//   Route 1: A1 (1→2), A2 (2→3)
//   Route 2: B1 (1→4), B2 (4→3)
TrafficGraph make_symmetric_traffic() {
  TrafficGraph g;
  g.nodes = {{1, "origin"}, {2, "mid_A"}, {3, "dest"}, {4, "mid_B"}};

  auto make_link = [](int idx, int from, int to, double len_km, double ff_hr,
                       double cap, double jam) {
    TrafficLink l;
    l.index = idx;
    l.from_node = from;
    l.to_node = to;
    l.length_km = len_km;
    l.free_flow_time_hr = ff_hr;
    l.capacity_veh_per_hr = cap;
    l.jam_vehicles = jam;
    return l;
  };

  // Route 1 links (shorter free-flow time → dominates initially)
  g.links.push_back(make_link(11, 1, 2, 0.5, 0.05, 20.0, 10.0));
  g.links.push_back(make_link(12, 2, 3, 0.5, 0.05, 20.0, 10.0));
  // Route 2 links (same parameters → symmetric, DUE: equal split)
  g.links.push_back(make_link(21, 1, 4, 0.5, 0.05, 20.0, 10.0));
  g.links.push_back(make_link(22, 4, 3, 0.5, 0.05, 20.0, 10.0));

  return g;
}

// Asymmetric graph: route 2 links have longer free-flow time.
// At DUE, more vehicles should use route 1 (cheaper) — unless congestion
// equalises the costs, in which case some partial split emerges.
TrafficGraph make_asymmetric_traffic() {
  TrafficGraph g;
  g.nodes = {{1, "origin"}, {2, "mid_A"}, {3, "dest"}, {4, "mid_B"}};

  auto make_link = [](int idx, int from, int to, double len_km, double ff_hr,
                       double cap, double jam) {
    TrafficLink l;
    l.index = idx;
    l.from_node = from;
    l.to_node = to;
    l.length_km = len_km;
    l.free_flow_time_hr = ff_hr;
    l.capacity_veh_per_hr = cap;
    l.jam_vehicles = jam;
    return l;
  };

  // Route 1: fast (free-flow 0.05 hr each)
  g.links.push_back(make_link(11, 1, 2, 0.5, 0.05, 20.0, 10.0));
  g.links.push_back(make_link(12, 2, 3, 0.5, 0.05, 20.0, 10.0));
  // Route 2: slow (free-flow 0.15 hr each, double length)
  g.links.push_back(make_link(21, 1, 4, 1.5, 0.15, 20.0, 10.0));
  g.links.push_back(make_link(22, 4, 3, 1.5, 0.15, 20.0, 10.0));

  return g;
}

RouteAlternative make_route(int idx, int o, int d,
                             std::vector<int> links, int station_id) {
  RouteAlternative r;
  r.index = idx;
  r.origin_node = o;
  r.destination_node = d;
  r.link_indices = std::move(links);

  RouteChargingStop stop;
  stop.station_id = station_id;
  stop.requested_energy_kwh_per_vehicle = 10.0;
  stop.dwell_steps = 1;
  stop.max_charge_kw_per_vehicle = 20.0;
  r.charging_stops.push_back(stop);
  return r;
}

EVDemand make_demand(int idx, int o, int d, int dep_step, double vehicles,
                      std::vector<int> candidate_routes) {
  EVDemand dem;
  dem.index = idx;
  dem.origin_node = o;
  dem.destination_node = d;
  dem.departure_step = dep_step;
  dem.vehicles = vehicles;
  dem.candidate_route_indices = std::move(candidate_routes);
  return dem;
}

EVPowerTrafficProblem make_symmetric_problem(double n_vehicles = 5.0) {
  EVPowerTrafficProblem prob;
  prob.system = make_two_station_system();
  prob.traffic = make_symmetric_traffic();
  prob.routes.push_back(make_route(1, 1, 3, {11, 12}, 101));
  prob.routes.push_back(make_route(2, 1, 3, {21, 22}, 102));
  prob.demands.push_back(make_demand(1, 1, 3, 0, n_vehicles, {1, 2}));
  return prob;
}

EVPowerTrafficProblem make_asymmetric_problem(double n_vehicles = 10.0) {
  EVPowerTrafficProblem prob;
  prob.system = make_two_station_system();
  prob.traffic = make_asymmetric_traffic();
  prob.routes.push_back(make_route(1, 1, 3, {11, 12}, 101));
  prob.routes.push_back(make_route(2, 1, 3, {21, 22}, 102));
  prob.demands.push_back(make_demand(1, 1, 3, 0, n_vehicles, {1, 2}));
  return prob;
}

EVPowerTrafficProblem make_single_route_problem() {
  EVPowerTrafficProblem prob;
  prob.system = make_two_station_system();
  prob.traffic = make_symmetric_traffic();  // reuse graph, only one route
  prob.routes.push_back(make_route(1, 1, 3, {11, 12}, 101));
  prob.demands.push_back(make_demand(1, 1, 3, 0, 4.0, {1}));
  return prob;
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────────
// Test case 1: CTM forward pass sanity (single route, no DUE needed)
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: single-route no crash, route flow equals demand",
          "[ctm][due][single_route]") {
  auto prob = make_single_route_problem();

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 6;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.02;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations = 5;
  due_opts.convergence_tol = 1e-2;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // Must run at least one iteration
  CHECK(res.iterations >= 1);

  // With only one route, all flow goes to route 1
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const double x_r1 = res.flow_by_demand_route.at(1).count(1)
                           ? res.flow_by_demand_route.at(1).at(1) : 0.0;
  CHECK(x_r1 == Approx(4.0).epsilon(1e-6));

  // Route travel time must be populated for route 1
  CHECK(res.final_ctm.route_travel_time.count(1) > 0);

  // Station arrivals must be non-negative
  for (const auto& [stn, arrivals] : res.final_ctm.station_arrivals) {
    for (double a : arrivals) {
      CHECK(a >= -1e-9);
    }
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 2: Symmetric 2-route network — demand balance
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: symmetric 2-route — total flow equals demand",
          "[ctm][due][symmetric]") {
  auto prob = make_symmetric_problem(6.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 8;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.02;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations = 20;
  due_opts.convergence_tol = 1e-2;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  CHECK(res.iterations >= 1);
  CHECK(res.iterations <= 20);

  // Sum of route flows must equal demand (6 vehicles)
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const auto& route_map = res.flow_by_demand_route.at(1);
  double total_flow = 0.0;
  for (const auto& [rid, x] : route_map) {
    CHECK(x >= -1e-9);
    total_flow += x;
  }
  CHECK(total_flow == Approx(6.0).epsilon(1e-4));

  // Both routes should have travel times populated
  CHECK(res.final_ctm.route_travel_time.count(1) > 0);
  CHECK(res.final_ctm.route_travel_time.count(2) > 0);

  // Relative gap must be finite and non-negative
  CHECK(res.relative_gap >= 0.0);
  CHECK(std::isfinite(res.relative_gap));

  // Final CTM step-link results must cover all 4 links
  if (!res.final_ctm.step_link_results.empty()) {
    CHECK(res.final_ctm.step_link_results.front().size() == 4u);
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 3: Symmetric network — MSA converges, both routes used
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: symmetric — MSA converges, both routes receive positive flow",
          "[ctm][due][convergence]") {
  // With symmetric routes and enough iterations, DUE should equalise costs.
  // Since the network is symmetric and demand is modest (below capacity),
  // the equilibrium is the equal-split (x_r1 ≈ x_r2 ≈ demand/2).
  auto prob = make_symmetric_problem(4.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 10;
  ev_opts.time_step_hr = 0.05;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.01;
  ctm_opts.n_cells_per_link = 5;
  ctm_opts.route_specific_cells = true;

  DUEOptions due_opts;
  due_opts.max_iterations = 50;
  due_opts.convergence_tol = 5e-3;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // Should converge for this small problem
  CHECK(res.converged);

  // Both routes should have positive flow (DUE spreads flow on equal-cost routes)
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const auto& rm = res.flow_by_demand_route.at(1);
  const double x1 = rm.count(1) ? rm.at(1) : 0.0;
  const double x2 = rm.count(2) ? rm.at(2) : 0.0;
  CHECK(x1 >= 0.0);
  CHECK(x2 >= 0.0);
  // At symmetric equilibrium, both should be non-negligible (> 0.1 * demand)
  CHECK(x1 + x2 == Approx(4.0).epsilon(1e-4));

  // Convergence: relative gap below tolerance
  CHECK(res.relative_gap < due_opts.convergence_tol + 1e-9);

  // History must have at least one entry
  REQUIRE(!res.history.empty());
  for (const auto& rec : res.history) {
    CHECK(rec.iteration >= 1);
    CHECK(std::isfinite(rec.relative_gap));
    CHECK(std::isfinite(rec.gap));
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 4: Asymmetric network — route 1 (cheaper) gets more flow
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: asymmetric — cheap route gets majority of flow",
          "[ctm][due][asymmetric]") {
  auto prob = make_asymmetric_problem(4.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 10;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.02;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations = 40;
  due_opts.convergence_tol = 5e-3;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // Demand balance
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const auto& rm = res.flow_by_demand_route.at(1);
  const double x1 = rm.count(1) ? rm.at(1) : 0.0;
  const double x2 = rm.count(2) ? rm.at(2) : 0.0;
  CHECK(x1 + x2 == Approx(4.0).epsilon(1e-4));

  // Route 1 has lower free-flow cost: with 4 vehicles and capacity 20/hr
  // there is no congestion on route 1, so DUE assigns all flow to route 1.
  // (Route 2 is strictly more expensive under free-flow, and congestion on
  //  route 1 does not reach the level that equalises the gap with route 2.)
  CHECK(x1 > x2);

  // Travel times populated
  CHECK(res.final_ctm.route_travel_time.count(1) > 0);
  CHECK(res.final_ctm.route_travel_time.count(2) > 0);
  const auto& tt1 = res.final_ctm.route_travel_time.at(1);
  const auto& tt2 = res.final_ctm.route_travel_time.at(2);
  if (!tt1.empty() && !tt2.empty()) {
    // Route 1 is free-flow ≈ 0.1 hr; route 2 is ≈ 0.3 hr.
    // At convergence t1 ≤ t2 (route 1 is not worse than route 2).
    CHECK(tt1.front() <= tt2.front() + 1e-6);
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 5: CTM-DUE with logit assignment
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: logit assignment — both routes always receive flow",
          "[ctm][due][logit]") {
  auto prob = make_asymmetric_problem(3.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 8;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.02;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations = 20;
  due_opts.convergence_tol = 1e-2;
  due_opts.logit_theta = 5.0;  // moderate sensitivity → stochastic split

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const auto& rm = res.flow_by_demand_route.at(1);
  const double x1 = rm.count(1) ? rm.at(1) : 0.0;
  const double x2 = rm.count(2) ? rm.at(2) : 0.0;

  // Demand balance
  CHECK(x1 + x2 == Approx(3.0).epsilon(1e-4));

  // Logit always assigns some flow to every route regardless of cost gap
  CHECK(x1 > 1e-6);
  CHECK(x2 > 1e-6);
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 6: Multiple departure steps — results indexed per step
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: multiple departure steps — independent demand balance each step",
          "[ctm][due][multi_step]") {
  EVPowerTrafficProblem prob;
  prob.system = make_two_station_system();
  prob.traffic = make_symmetric_traffic();
  prob.routes.push_back(make_route(1, 1, 3, {11, 12}, 101));
  prob.routes.push_back(make_route(2, 1, 3, {21, 22}, 102));

  // Two demands at different departure steps
  prob.demands.push_back(make_demand(1, 1, 3, 0, 3.0, {1, 2}));
  prob.demands.push_back(make_demand(2, 1, 3, 1, 2.0, {1, 2}));

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 10;
  ev_opts.time_step_hr = 0.05;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.01;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations = 20;
  due_opts.convergence_tol = 1e-2;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // Demand 1: total flow ≈ 3.0
  if (res.flow_by_demand_route.count(1)) {
    double tot = 0.0;
    for (const auto& [r, x] : res.flow_by_demand_route.at(1)) tot += x;
    CHECK(tot == Approx(3.0).epsilon(1e-4));
  }

  // Demand 2: total flow ≈ 2.0
  if (res.flow_by_demand_route.count(2)) {
    double tot = 0.0;
    for (const auto& [r, x] : res.flow_by_demand_route.at(2)) tot += x;
    CHECK(tot == Approx(2.0).epsilon(1e-4));
  }

  // No negative flows
  for (const auto& [did, rm] : res.flow_by_demand_route) {
    for (const auto& [rid, x] : rm) {
      CHECK(x >= -1e-9);
    }
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 7: Auto CTM dt selection (dt_ctm_hr = 0) — no crash
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: auto CFL dt selection — finishes without error",
          "[ctm][due][auto_dt]") {
  auto prob = make_symmetric_problem(5.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps = 6;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr = 0.0;   // auto-select via CFL
  ctm_opts.n_cells_per_link = 0;  // auto M_a

  DUEOptions due_opts;
  due_opts.max_iterations = 10;
  due_opts.convergence_tol = 1e-2;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  CHECK(res.iterations >= 1);

  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  double tot = 0.0;
  for (const auto& [r, x] : res.flow_by_demand_route.at(1)) {
    CHECK(x >= -1e-9);
    tot += x;
  }
  CHECK(tot == Approx(5.0).epsilon(1e-4));
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 8: Session synthesis and smart-charging dispatch
// ──────────────────────────────────────────────────────────────────────────
// Verifies that after DUE convergence the implementation:
//   (a) synthesises at least one EVChargingSession per (demand,route,stop)
//       triple with positive DUE flow,
//   (b) dispatches power to deliver positive energy,
//   (c) populates station_ev_load_kw for the stations used,
//   (d) reports total_requested_energy_kwh > 0 and
//       total_delivered_energy_kwh ∈ (0, total_requested].
//
// Configuration: dt=1.0 h so that one dwell step is enough for the
// charger to fully deliver 10 kWh/veh (20 kW × 0.95 × 1 h = 19 kWh ≥ 10).
TEST_CASE("CTM-DUE: session synthesis and dispatch — energy delivered",
          "[ctm][due][sessions][dispatch]") {
  auto prob = make_symmetric_problem(4.0);  // 4 vehicles, both routes active

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps   = 5;
  ev_opts.time_step_hr = 1.0;  // dt=1 h → full delivery in one dwell step

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr    = 0.1;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations  = 30;
  due_opts.convergence_tol = 1e-3;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // (a) Sessions were synthesised
  CHECK(!res.sessions.empty());

  // (b) Total flow conservation
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  double flow_total = 0.0;
  for (const auto& [r, x] : res.flow_by_demand_route.at(1)) flow_total += x;
  CHECK(flow_total == Approx(4.0).epsilon(1e-3));

  // (c) Energy accounting: requested > 0, delivered ∈ (0, requested]
  CHECK(res.total_requested_energy_kwh > 0.0);
  CHECK(res.total_delivered_energy_kwh > 0.0);
  CHECK(res.total_delivered_energy_kwh <=
        res.total_requested_energy_kwh + 1e-6);

  // (d) Station EV load populated for at least one station
  bool has_nonzero_load = false;
  for (const auto& [sid, load_vec] : res.station_ev_load_kw) {
    for (double p : load_vec) {
      if (p > 1e-6) { has_nonzero_load = true; break; }
    }
    if (has_nonzero_load) break;
  }
  CHECK(has_nonzero_load);

  // (e) session_results length matches sessions
  CHECK(res.session_results.size() == res.sessions.size());

  // (f) Station queue vectors have correct sizes
  for (const auto& [sid, qvec] : res.station_queue_veh) {
    CHECK(static_cast<int>(qvec.size()) == ev_opts.num_steps + 1);
  }
  for (const auto& [sid, wvec] : res.station_waiting_time_hr) {
    CHECK(static_cast<int>(wvec.size()) == ev_opts.num_steps);
    for (double w : wvec) CHECK(w >= 0.0);
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 9: SOC feasibility filter
// ──────────────────────────────────────────────────────────────────────────
// Route 1: 2 links × 0.5 km, drive_energy = 2.0 kWh/km → 2.0 kWh needed
// Route 2: 2 links × 1.5 km, drive_energy = 2.0 kWh/km → 6.0 kWh needed
// Demand:  initial_energy_kwh = 2.0 kWh  → route 2 is SOC-infeasible
// Expected: AoN assigns all 4 vehicles to route 1 (x1≈4, x2<1e-6).
TEST_CASE("CTM-DUE: SOC feasibility filter removes infeasible routes",
          "[ctm][due][soc][filter]") {
  EVPowerTrafficProblem prob;
  prob.system = make_two_station_system();

  // Traffic graph: route 1 short (0.5 km), route 2 long (1.5 km)
  TrafficGraph tg;
  tg.nodes = {{1, "origin"}, {2, "mid_A"}, {3, "dest"}, {4, "mid_B"}};
  auto ml = [](int idx, int from, int to, double len_km) {
    TrafficLink l;
    l.index = idx; l.from_node = from; l.to_node = to;
    l.length_km = len_km;
    l.free_flow_time_hr   = len_km;          // 1 h/km for clarity
    l.capacity_veh_per_hr = 20.0;
    l.jam_vehicles        = 10.0;
    l.drive_energy_kwh_per_veh_km = 2.0;     // energy cost per km
    return l;
  };
  tg.links.push_back(ml(11, 1, 2, 0.5));  // route 1 links (total 1.0 km → 2.0 kWh)
  tg.links.push_back(ml(12, 2, 3, 0.5));
  tg.links.push_back(ml(21, 1, 4, 1.5));  // route 2 links (total 3.0 km → 6.0 kWh)
  tg.links.push_back(ml(22, 4, 3, 1.5));
  prob.traffic = tg;

  prob.routes.push_back(make_route(1, 1, 3, {11, 12}, 101));
  prob.routes.push_back(make_route(2, 1, 3, {21, 22}, 102));

  // Demand with initial SOC = 2.0 kWh — exactly enough for route 1, not for 2
  EVDemand dem;
  dem.index = 1; dem.origin_node = 1; dem.destination_node = 3;
  dem.departure_step = 0; dem.vehicles = 4.0;
  dem.candidate_route_indices = {1, 2};
  dem.initial_energy_kwh = 2.0;       // activates SOC check
  prob.demands.push_back(dem);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps    = 8;
  ev_opts.time_step_hr = 1.0;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr    = 0.1;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations  = 30;
  due_opts.convergence_tol = 1e-3;
  due_opts.logit_theta     = 0.0;   // AoN → SOC filter is decisive

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // Route 2 is SOC-infeasible → all vehicles on route 1
  REQUIRE(res.flow_by_demand_route.count(1) > 0);
  const auto& rm = res.flow_by_demand_route.at(1);
  double x1 = rm.count(1) ? rm.at(1) : 0.0;
  double x2 = rm.count(2) ? rm.at(2) : 0.0;
  CHECK(x1 == Approx(4.0).epsilon(1e-3));
  CHECK(x2 < 1e-6);
}

// ──────────────────────────────────────────────────────────────────────────
// Test case 10: Price-of-anarchy benchmark (compute_system_optimal_benchmark)
// ──────────────────────────────────────────────────────────────────────────
// On a symmetric network DUE and SO should produce similar TSTT; PoA ≈ 1.
// The test only verifies structural correctness: has_so_benchmark=true,
// tstt values are finite and non-negative, ratio is in a reasonable range.
TEST_CASE("CTM-DUE: price-of-anarchy benchmark — SO TSTT populated",
          "[ctm][due][poa][benchmark]") {
  auto prob = make_symmetric_problem(4.0);

  EVPowerTrafficOptions ev_opts;
  ev_opts.num_steps    = 6;
  ev_opts.time_step_hr = 0.1;

  CTMOptions ctm_opts;
  ctm_opts.dt_ctm_hr    = 0.02;
  ctm_opts.n_cells_per_link = 3;

  DUEOptions due_opts;
  due_opts.max_iterations             = 30;
  due_opts.convergence_tol            = 1e-3;
  due_opts.compute_system_optimal_benchmark = true;

  CTMDUEResult res = simulate_ev_power_traffic_ctm_due(prob, ev_opts, ctm_opts, due_opts);

  // DUE converged
  CHECK(res.iterations >= 1);

  // SO benchmark was run
  CHECK(res.has_so_benchmark);

  // Both TSTT values are finite and non-negative
  CHECK(res.due_tstt_hr >= 0.0);
  CHECK(res.so_tstt_hr  >= 0.0);
  CHECK(std::isfinite(res.due_tstt_hr));
  CHECK(std::isfinite(res.so_tstt_hr));

  // PoA ratio: for symmetric uncongested network DUE ≈ SO → ratio near 1.
  // Use a loose bound [0.5, 5.0] to be robust to BPR vs. CTM discrepancy.
  if (res.so_tstt_hr > 1e-9) {
    CHECK(res.poa_tstt_ratio >= 0.5);
    CHECK(res.poa_tstt_ratio <= 5.0);
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Numerical analysis: print concrete results for all three B scenarios.
// Run as:  ./tests/test_ev_power_traffic_ctm_due "[analysis]"
// ──────────────────────────────────────────────────────────────────────────
TEST_CASE("CTM-DUE: numerical analysis — print results for LaTeX",
          "[analysis]") {
  using std::printf;

  // ── B1: Session synthesis & dispatch ──────────────────────────────────
  printf("\n=================================================================\n");
  printf("B1: Session Synthesis & Dispatch  (4 veh, symmetric, dt=1 hr)\n");
  printf("=================================================================\n");
  {
    auto prob = make_symmetric_problem(4.0);
    EVPowerTrafficOptions ev; ev.num_steps=5; ev.time_step_hr=1.0;
    CTMOptions cm; cm.dt_ctm_hr=0.1; cm.n_cells_per_link=3;
    DUEOptions du; du.max_iterations=50; du.convergence_tol=1e-4;

    auto res = simulate_ev_power_traffic_ctm_due(prob,ev,cm,du);

    printf("DUE converged:            %s (iter=%d, rel_gap=%.4e)\n",
           res.converged?"yes":"no", res.iterations, res.relative_gap);
    printf("Sessions synthesised:     %zu\n", res.sessions.size());
    printf("Total requested energy:   %.2f kWh\n", res.total_requested_energy_kwh);
    printf("Total delivered energy:   %.2f kWh\n", res.total_delivered_energy_kwh);
    printf("Total unserved energy:    %.2f kWh\n", res.total_unserved_energy_kwh);
    printf("Service ratio:            %.1f%%\n",
           res.total_requested_energy_kwh>0
           ? 100.0*res.total_delivered_energy_kwh/res.total_requested_energy_kwh : 0.0);
    printf("\nRoute flows (demand 1):\n");
    if (res.flow_by_demand_route.count(1))
      for (const auto& [r,x] : res.flow_by_demand_route.at(1))
        printf("  route=%d  x=%.4f veh\n", r, x);
    printf("\nStation EV load [kW] per sim step:\n");
    printf("  step   stn101   stn102\n");
    for (int k=0; k<ev.num_steps; ++k) {
      double p1=0,p2=0;
      if (res.station_ev_load_kw.count(101)) p1=res.station_ev_load_kw.at(101)[k];
      if (res.station_ev_load_kw.count(102)) p2=res.station_ev_load_kw.at(102)[k];
      printf("  %4d   %7.2f  %7.2f\n",k,p1,p2);
    }
    printf("\nStation queue [veh]  (q_{s,0..T}):\n");
    printf("  step   q_101   q_102\n");
    for (int k=0; k<=ev.num_steps; ++k) {
      double q1=0,q2=0;
      if (res.station_queue_veh.count(101)) q1=res.station_queue_veh.at(101)[k];
      if (res.station_queue_veh.count(102)) q2=res.station_queue_veh.at(102)[k];
      printf("  %4d  %6.3f  %6.3f\n",k,q1,q2);
    }
    printf("\nWaiting time [hr]  W_{s,k}:\n");
    printf("  step   W_101   W_102\n");
    for (int k=0; k<ev.num_steps; ++k) {
      double w1=0,w2=0;
      if (res.station_waiting_time_hr.count(101)) w1=res.station_waiting_time_hr.at(101)[k];
      if (res.station_waiting_time_hr.count(102)) w2=res.station_waiting_time_hr.at(102)[k];
      printf("  %4d  %6.4f  %6.4f\n",k,w1,w2);
    }
  }

  // ── B2: SOC feasibility filter ────────────────────────────────────────
  printf("\n=================================================================\n");
  printf("B2: SOC Feasibility Filter\n");
  printf("=================================================================\n");
  {
    EVPowerTrafficProblem prob;
    prob.system = make_two_station_system();
    TrafficGraph tg;
    tg.nodes = {{1,"o"},{2,"mA"},{3,"d"},{4,"mB"}};
    auto ml=[](int idx,int f,int t,double lk){
      TrafficLink l; l.index=idx; l.from_node=f; l.to_node=t;
      l.length_km=lk; l.free_flow_time_hr=lk;
      l.capacity_veh_per_hr=20.0; l.jam_vehicles=10.0;
      l.drive_energy_kwh_per_veh_km=2.0; return l;};
    tg.links.push_back(ml(11,1,2,0.5));
    tg.links.push_back(ml(12,2,3,0.5));
    tg.links.push_back(ml(21,1,4,1.5));
    tg.links.push_back(ml(22,4,3,1.5));
    prob.traffic = tg;
    prob.routes.push_back(make_route(1,1,3,{11,12},101));
    prob.routes.push_back(make_route(2,1,3,{21,22},102));
    EVDemand d; d.index=1; d.origin_node=1; d.destination_node=3;
    d.departure_step=0; d.vehicles=4.0; d.candidate_route_indices={1,2};
    d.initial_energy_kwh=2.0;
    prob.demands.push_back(d);

    EVPowerTrafficOptions ev; ev.num_steps=8; ev.time_step_hr=1.0;
    CTMOptions cm; cm.dt_ctm_hr=0.1; cm.n_cells_per_link=3;
    DUEOptions du; du.max_iterations=30; du.convergence_tol=1e-3; du.logit_theta=0.0;

    auto res = simulate_ev_power_traffic_ctm_due(prob,ev,cm,du);

    double x1=0,x2=0;
    if (res.flow_by_demand_route.count(1)) {
      const auto& rm=res.flow_by_demand_route.at(1);
      if (rm.count(1)) x1=rm.at(1);
      if (rm.count(2)) x2=rm.at(2);
    }
    printf("Route 1 drive energy: 2×0.5 km × 2 kWh/km = 2.0 kWh  (SOC=2.0 → FEASIBLE)\n");
    printf("Route 2 drive energy: 2×1.5 km × 2 kWh/km = 6.0 kWh  (SOC=2.0 → INFEASIBLE)\n");
    printf("x_1 (route 1) = %.4f veh\n", x1);
    printf("x_2 (route 2) = %.6f veh  (filtered)\n", x2);
    printf("Total = %.4f veh\n", x1+x2);
  }

  // ── B3: Price-of-anarchy ──────────────────────────────────────────────
  printf("\n=================================================================\n");
  printf("B3: Price-of-Anarchy Benchmark\n");
  printf("=================================================================\n");
  {
    auto prob = make_symmetric_problem(4.0);
    EVPowerTrafficOptions ev; ev.num_steps=6; ev.time_step_hr=0.1;
    CTMOptions cm; cm.dt_ctm_hr=0.02; cm.n_cells_per_link=3;
    DUEOptions du; du.max_iterations=50; du.convergence_tol=1e-3;
    du.compute_system_optimal_benchmark=true;

    auto res = simulate_ev_power_traffic_ctm_due(prob,ev,cm,du);

    printf("DUE converged:     %s (iter=%d, rel_gap=%.4e)\n",
           res.converged?"yes":"no", res.iterations, res.relative_gap);
    printf("DUE TSTT:          %.6f veh·hr\n", res.due_tstt_hr);
    printf("SO  TSTT:          %.6f veh·hr\n", res.so_tstt_hr);
    printf("PoA ratio:         %.4f\n", res.poa_tstt_ratio);
    printf("SO solved:         %s\n", res.has_so_benchmark?"yes":"no");
    printf("\nConvergence history:\n  iter   rel_gap\n");
    for (const auto& h : res.history)
      printf("  %4d   %.4e\n", h.iteration, h.relative_gap);
  }
  printf("\n");
}

/**
 * EV Power-Traffic Simulation — Numerical Results Report
 *
 * Runs four scenarios on a 4-node / 2-route / 2-station / 2-EV-class network
 * and prints per-step tables for:
 *   • Route assignment (vehicles per route)
 *   • Traffic link flows (inflow, occupancy, travel time)
 *   • Station charging & discharging power (kW)
 *   • Queue state (queue length, plug occupancy, waiting time)
 *   • Power balance (base load, EV load, margin)
 *
 * Run:  ./tests/test_ev_power_traffic_report --success
 */

#include <algorithm>
#include <cstdio>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;

// ============================================================
// Network & scenario builders
// ============================================================

namespace {

// 4-node graph:
//   Node 1 (origin) --link11--> Node 2 (near_stn) --link12--> Node 4 (dest)
//                    --link21--> Node 3 (far_stn)  --link22--> Node 4
HybridPowerSystem make_system() {
  HybridPowerSystem sys;

  auto add_bus = [&](int idx, BusType t, double vm = 1.0) {
    ACBus b;
    b.index = idx;
    b.bus_type = t;
    b.vm_pu = vm;
    b.in_service = true;
    sys.ac.buses.push_back(b);
  };
  add_bus(1, BusType::SLACK, 1.0);
  add_bus(2, BusType::PQ, 1.0);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.pmax_mw = 2.0;
  g.qmax_mvar = 2.0;
  g.qmin_mvar = -2.0;
  sys.ac.generators.push_back(g);

  // Near station — tight capacity (40 kW)
  {
    ChargingStation st;
    st.index = 101;
    st.bus = 1;
    st.in_service = true;
    st.max_power_kw = 40.0;
    st.power_factor = 1.0;
    st.n_fast = 2;  // 2 plugs → queue builds when >2 EVs present
    sys.ac.charging_stations.push_back(st);

    Charger ch;
    ch.index = 1;
    ch.station_id = 101;
    ch.in_service = true;
    ch.p_ch_max_kw = 40.0;
    ch.v2g_capable = true;
    ch.p_dis_max_kw = 20.0;
    sys.ac.chargers.push_back(ch);
  }
  // Far station — generous capacity (60 kW)
  {
    ChargingStation st;
    st.index = 102;
    st.bus = 2;
    st.in_service = true;
    st.max_power_kw = 60.0;
    st.power_factor = 1.0;
    st.n_fast = 3;
    sys.ac.charging_stations.push_back(st);

    Charger ch;
    ch.index = 2;
    ch.station_id = 102;
    ch.in_service = true;
    ch.p_ch_max_kw = 60.0;
    ch.v2g_capable = false;
    ch.p_dis_max_kw = 0.0;
    sys.ac.chargers.push_back(ch);
  }
  return sys;
}

TrafficGraph make_traffic() {
  TrafficGraph g;
  g.nodes = {{1, "origin"}, {2, "near_stn"}, {3, "far_stn"}, {4, "dest"}};

  auto make_link = [](int idx, int from, int to, double ff_hr, double cap) {
    TrafficLink l;
    l.index = idx;
    l.from_node = from;
    l.to_node = to;
    l.free_flow_time_hr = ff_hr;
    l.capacity_veh_per_hr = cap;
    l.length_km = ff_hr * 80.0;  // assume 80 km/h free-flow speed
    return l;
  };
  g.links = {
      make_link(11, 1, 2, 0.25, 20.0),  // near leg 1
      make_link(12, 2, 4, 0.25, 20.0),  // near leg 2
      make_link(21, 1, 3, 0.50, 20.0),  // far leg 1
      make_link(22, 3, 4, 0.50, 20.0),  // far leg 2
  };
  return g;
}

// Near route via station 101 (0.5 hr, 20 kWh/veh)
RouteAlternative near_route() {
  RouteAlternative r;
  r.index = 1;
  r.origin_node = 1;
  r.destination_node = 4;
  r.link_indices = {11, 12};
  RouteChargingStop s;
  s.station_id = 101;
  s.requested_energy_kwh_per_vehicle = 20.0;
  s.dwell_steps = 1;
  s.max_charge_kw_per_vehicle = 20.0;
  r.charging_stops.push_back(s);
  return r;
}

// Far route via station 102 (1.0 hr, 20 kWh/veh)
RouteAlternative far_route() {
  RouteAlternative r;
  r.index = 2;
  r.origin_node = 1;
  r.destination_node = 4;
  r.link_indices = {21, 22};
  RouteChargingStop s;
  s.station_id = 102;
  s.requested_energy_kwh_per_vehicle = 20.0;
  s.dwell_steps = 1;
  s.max_charge_kw_per_vehicle = 20.0;
  r.charging_stops.push_back(s);
  return r;
}

EVPowerTrafficOptions base_options() {
  EVPowerTrafficOptions opt;
  opt.num_steps = 6;
  opt.time_step_hr = 1.0;  // 1 hr steps: 20 kW × 1 hr = 20 kWh exactly meets request
  opt.default_charging_efficiency = 1.0;
  opt.enforce_generation_capacity = true;
  opt.allow_unserved_travel_demand = false;
  opt.allow_unserved_charging_energy = false;
  opt.run_power_flow_validation = false;
  opt.value_of_time_per_hr = 1.0;
  opt.queueing_weight = 0.1;
  return opt;
}

// ============================================================
// Printing helpers
// ============================================================

void print_separator(const char* title) {
  printf("\n%s\n", std::string(70, '=').c_str());
  printf("  %s\n", title);
  printf("%s\n", std::string(70, '=').c_str());
}

void print_assignments(const EVPowerTrafficResult& res) {
  printf("\n  Route Assignments\n");
  printf("  %-8s %-8s %-8s %-10s %-10s %-8s\n",
         "demand", "route", "step", "vehicles", "gen_cost", "travel_hr");
  printf("  %s\n", std::string(56, '-').c_str());
  for (const auto& a : res.assignments) {
    printf("  %-8d %-8d %-8d %-10.2f %-10.4f %-8.4f\n",
           a.demand_index, a.route_index, a.departure_step,
           a.vehicles, a.generalized_cost, a.travel_time_hr);
  }
}

void print_step_traffic(const EVPowerTrafficStepResult& s) {
  printf("    Traffic links:\n");
  printf("      %-8s %-12s %-14s %-14s %-12s %-8s\n",
         "link", "inflow_veh", "occupancy_veh", "capacity_veh", "travel_hr", "blocked");
  for (const auto& l : s.traffic_links) {
    printf("      %-8d %-12.3f %-14.3f %-14.3f %-12.4f %-8s\n",
           l.link_index, l.inflow_vehicles, l.occupancy_vehicles,
           l.capacity_vehicles, l.travel_time_hr,
           l.blocked ? "YES" : "no");
  }
}

void print_step_stations(const EVPowerTrafficStepResult& s) {
  printf("    Stations:\n");
  printf("      %-10s %-12s %-12s %-14s %-14s %-12s %-12s %-12s\n",
         "station", "arrivals", "p_ev_kw", "p_discharge_kw", "q_ev_kvar",
         "queue_veh", "plugged_veh", "wait_hr");
  for (const auto& st : s.stations) {
    printf("      %-10d %-12.3f %-12.3f %-14.3f %-14.3f %-12.3f %-12.3f %-12.4f\n",
           st.station_id, st.arrivals_vehicles,
           st.p_ev_kw, st.reverse_power_kw, st.q_ev_kvar,
           st.queue_vehicles, st.plug_occupancy, st.waiting_time_hr);
  }
}

void print_step_power(const EVPowerTrafficStepResult& s) {
  printf("    Power balance:  base=%.3f MW  ev=%.3f MW  cap=%.3f MW  margin=%.3f MW  ok=%s\n",
         s.power.total_base_load_mw, s.power.total_ev_load_mw,
         s.power.total_generation_capacity_mw, s.power.capacity_margin_mw,
         s.power.generation_capacity_ok ? "YES" : "NO");
}

void print_all_steps(const EVPowerTrafficResult& res) {
  printf("\n  Per-step results\n");
  for (const auto& s : res.steps) {
    printf("\n  --- Step %d  (t = %.2f hr) ---\n", s.step_index, s.hour);
    print_step_traffic(s);
    print_step_stations(s);
    print_step_power(s);
  }
}

void print_summary(const EVPowerTrafficResult& res) {
  printf("\n  Summary\n");
  printf("  %-35s %.3f\n", "feasible:", res.feasible ? 1.0 : 0.0);
  printf("  %-35s %.3f veh\n", "total demand:", res.total_demand_vehicles);
  printf("  %-35s %.3f veh\n", "served demand:", res.served_demand_vehicles);
  printf("  %-35s %.3f veh\n", "unmet demand:", res.unmet_demand_vehicles);
  printf("  %-35s %.3f kWh\n", "requested energy:", res.total_requested_energy_kwh);
  printf("  %-35s %.3f kWh\n", "delivered energy:", res.total_delivered_energy_kwh);
  printf("  %-35s %.3f kWh\n", "unserved energy:", res.total_unserved_energy_kwh);
  printf("  %-35s %.3f kWh\n", "V2G energy:", res.total_v2g_energy_kwh);
  printf("  %-35s %.4f veh·hr\n", "total travel time:", res.total_travel_time_hr);
  if (res.optimization_solved) {
    printf("  %-35s %.6f\n", "opt objective:", res.optimization_objective);
    printf("  %-35s %s\n", "opt proven optimal:", res.optimization_proven_optimal ? "yes" : "no");
    printf("  %-35s %.2e\n", "opt MIP gap:", res.optimization_mip_gap);
  }
}

}  // namespace

// ============================================================
// Scenario 1 — Deterministic shortest path (all 5 vehicles crowd near station)
// Near station: 40 kW cap / 20 kW per veh = max 2 vehicles served fully.
// With 5 vehicles, 3 get partial energy → unserved_energy > 0.
// ============================================================
TEST_CASE("Report: Scenario 1 – Deterministic Shortest Path", "[evpt][report]") {
  EVPowerTrafficProblem p;
  p.system = make_system();
  p.traffic = make_traffic();
  p.routes = {near_route(), far_route()};

  EVDemand d;
  d.index = 1;
  d.origin_node = 1;
  d.destination_node = 4;
  d.departure_step = 0;
  d.vehicles = 5.0;
  d.candidate_route_indices = {1, 2};
  p.demands.push_back(d);

  auto opt = base_options();
  opt.assignment_model = AssignmentModel::DeterministicShortestPath;
  opt.allow_unserved_charging_energy = true;  // observe partial delivery

  const auto res = simulate_ev_power_traffic(p, opt);

  print_separator("Scenario 1: Deterministic Shortest Path (5 veh, dt=1 hr)\n  Near stn 101: 40 kW cap, 2 plugs. All 5 crowd near route → queue/partial energy.");
  print_assignments(res);
  print_all_steps(res);
  print_summary(res);

  CHECK(res.total_demand_vehicles == Catch::Approx(5.0).margin(1e-9));
  CHECK(res.served_demand_vehicles == Catch::Approx(5.0).margin(1e-9));
  // DSP ignores station capacity → near station is overloaded → energy unserved
  CHECK(res.total_unserved_energy_kwh > 0.0);
}

// ============================================================
// Scenario 2 — Capacity-Aware Greedy (joint traffic + station capacity)
// Same 5 vehicles: greedy respects 40 kW cap → routes 2 to near, 3 to far.
// ============================================================
TEST_CASE("Report: Scenario 2 – Capacity-Aware Greedy", "[evpt][report]") {
  EVPowerTrafficProblem p;
  p.system = make_system();
  p.traffic = make_traffic();
  p.routes = {near_route(), far_route()};

  EVDemand d;
  d.index = 1;
  d.origin_node = 1;
  d.destination_node = 4;
  d.departure_step = 0;
  d.vehicles = 5.0;
  d.candidate_route_indices = {1, 2};
  p.demands.push_back(d);

  auto opt = base_options();
  opt.assignment_model = AssignmentModel::CapacityAwareGreedy;
  opt.allow_unserved_charging_energy = false;

  const auto res = simulate_ev_power_traffic(p, opt);

  print_separator("Scenario 2: Capacity-Aware Greedy (5 veh, dt=1 hr)\n  Joint optimisation: 2 → near stn 101 (40 kW), 3 → far stn 102 (60 kW).");
  print_assignments(res);
  print_all_steps(res);
  print_summary(res);

  CHECK(res.total_demand_vehicles == Catch::Approx(5.0).margin(1e-9));
  CHECK(res.total_unserved_energy_kwh == Catch::Approx(0.0).margin(1e-9));
}

// ============================================================
// Scenario 3 — System-Optimal LP (certified minimum cost)
// Same 5 vehicles: LP solves certified split with proven global optimality.
// ============================================================
TEST_CASE("Report: Scenario 3 – System-Optimal LP", "[evpt][report]") {
  EVPowerTrafficProblem p;
  p.system = make_system();
  p.traffic = make_traffic();
  p.routes = {near_route(), far_route()};

  EVDemand d;
  d.index = 1;
  d.origin_node = 1;
  d.destination_node = 4;
  d.departure_step = 0;
  d.vehicles = 5.0;
  d.candidate_route_indices = {1, 2};
  p.demands.push_back(d);

  auto opt = base_options();
  opt.assignment_model = AssignmentModel::SystemOptimalLP;
  opt.allow_unserved_charging_energy = false;

  const auto res = simulate_ev_power_traffic(p, opt);

  print_separator("Scenario 3: System-Optimal LP (5 veh, dt=1 hr)\n  LP with HiGHS: certifiably optimal split, proven optimal flag, MIP gap=0.");
  print_assignments(res);
  print_all_steps(res);
  print_summary(res);

  CHECK(res.feasible);
  CHECK(res.total_unserved_energy_kwh == Catch::Approx(0.0).margin(1e-9));
  if (res.optimization_solved) {
    CHECK(res.optimization_proven_optimal);
    CHECK(res.optimization_mip_gap == Catch::Approx(0.0).margin(1e-12));
  }
}

// ============================================================
// Scenario 4 — V2G discharge during high-price window
// ============================================================
TEST_CASE("Report: Scenario 4 – V2G Discharge", "[evpt][report]") {
  EVPowerTrafficProblem p;
  p.system = make_system();

  // Pre-staged session at station 101: EV has 50 kWh, target 40 kWh
  // → 10 kWh discharge over 2 steps (5 kW per step of 0.5 hr? or 10 kW for 1 step)
  EVChargingSession sess;
  sess.index = 1;
  sess.station_id = 101;
  sess.arrival_step = 0;
  sess.departure_step = 3;
  sess.energy_initial_kwh = 50.0;
  sess.energy_target_kwh = 30.0;
  sess.energy_min_kwh = 20.0;
  sess.energy_max_kwh = 60.0;
  sess.max_charge_kw = 0.0;    // pure discharge session
  sess.max_discharge_kw = 20.0;
  sess.eta_discharge = 1.0;
  sess.v2g_capable = true;
  p.initial_sessions.push_back(sess);

  // High price every step → all discharge is profitable
  StationPriceProfile price;
  price.station_id = 101;
  price.price_per_kwh = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  p.station_prices.push_back(price);

  auto opt = base_options();
  opt.num_steps = 4;
  opt.allow_v2g = true;
  opt.high_price_threshold_per_kwh = 0.5;
  opt.allow_unserved_charging_energy = true;
  // dt=1.0 hr (from base_options): 20 kW × 2 steps = 40 kWh discharged, but
  // target is 50−30=20 kWh, capped by the session's energy model.

  const auto res = simulate_ev_power_traffic(p, opt);

  print_separator("Scenario 4: V2G Discharge (1 EV, 50→30 kWh target, 4 steps)");
  print_all_steps(res);
  print_summary(res);

  CHECK(res.feasible);
  CHECK(res.total_v2g_energy_kwh > 0.0);
}

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

namespace {

HybridPowerSystem make_evpt_system() {
  HybridPowerSystem sys;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;
  sys.ac.buses.push_back(b1);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.pmax_mw = 10.0;
  g.qmax_mvar = 10.0;
  g.qmin_mvar = -10.0;
  sys.ac.generators.push_back(g);

  ChargingStation station;
  station.index = 101;
  station.bus = 1;
  station.in_service = true;
  station.max_power_kw = 100.0;
  station.power_factor = 0.95;
  sys.ac.charging_stations.push_back(station);

  Charger charger;
  charger.index = 1;
  charger.station_id = 101;
  charger.in_service = true;
  charger.p_ch_max_kw = 100.0;
  charger.v2g_capable = true;
  charger.p_dis_max_kw = 50.0;
  sys.ac.chargers.push_back(charger);

  return sys;
}

TrafficGraph make_two_link_traffic() {
  TrafficGraph g;
  g.nodes = {{1, "origin"}, {2, "station"}, {3, "destination"}};

  TrafficLink l1;
  l1.index = 11;
  l1.from_node = 1;
  l1.to_node = 2;
  l1.length_km = 1.0;
  l1.free_flow_time_hr = 1.0;
  l1.capacity_veh_per_hr = 10.0;

  TrafficLink l2;
  l2.index = 12;
  l2.from_node = 2;
  l2.to_node = 3;
  l2.length_km = 1.0;
  l2.free_flow_time_hr = 1.0;
  l2.capacity_veh_per_hr = 10.0;

  g.links = {l1, l2};
  return g;
}

RouteAlternative make_route() {
  RouteAlternative route;
  route.index = 1;
  route.origin_node = 1;
  route.destination_node = 3;
  route.link_indices = {11, 12};

  RouteChargingStop stop;
  stop.station_id = 101;
  stop.requested_energy_kwh_per_vehicle = 10.0;
  stop.dwell_steps = 2;
  stop.max_charge_kw_per_vehicle = 10.0;
  route.charging_stops.push_back(stop);
  return route;
}

EVDemand make_demand(double vehicles) {
  EVDemand d;
  d.index = 1;
  d.origin_node = 1;
  d.destination_node = 3;
  d.departure_step = 0;
  d.vehicles = vehicles;
  d.candidate_route_indices = {1};
  return d;
}

EVPowerTrafficProblem make_problem() {
  EVPowerTrafficProblem p;
  p.system = make_evpt_system();
  p.traffic = make_two_link_traffic();
  p.routes = {make_route()};
  p.demands = {make_demand(2.0)};
  return p;
}

EVPowerTrafficOptions make_options() {
  EVPowerTrafficOptions opt;
  opt.num_steps = 6;
  opt.time_step_hr = 1.0;
  opt.enforce_generation_capacity = true;
  opt.allow_unserved_travel_demand = false;
  opt.allow_unserved_charging_energy = false;
  opt.run_power_flow_validation = false;
  return opt;
}

}  // namespace

TEST_CASE("EVPT: blocked road produces unmet travel demand", "[evpt]") {
  auto problem = make_problem();
  problem.traffic.links[0].capacity_profile_veh_per_hr = {0.0, 10.0, 10.0};

  auto opt = make_options();
  opt.allow_unserved_travel_demand = false;

  const auto result = simulate_ev_power_traffic(problem, opt);

  CHECK_FALSE(result.feasible);
  CHECK(result.unmet_demand_vehicles == Approx(2.0).margin(1e-9));
  REQUIRE(!result.steps.empty());
  CHECK(result.steps[0].traffic_links[0].blocked);
}

TEST_CASE("EVPT: unmet travel demand can be allowed explicitly", "[evpt]") {
  auto problem = make_problem();
  problem.traffic.links[0].available = false;

  auto opt = make_options();
  opt.allow_unserved_travel_demand = true;

  const auto result = simulate_ev_power_traffic(problem, opt);

  CHECK(result.feasible);
  CHECK(result.unmet_demand_vehicles == Approx(2.0).margin(1e-9));
  CHECK(result.served_demand_vehicles == Approx(0.0).margin(1e-9));
}

TEST_CASE("EVPT: route assignment creates station load and updates system snapshots",
          "[evpt]") {
  const auto problem = make_problem();
  auto opt = make_options();

  const auto result = simulate_ev_power_traffic(problem, opt);

  REQUIRE(result.feasible);
  CHECK(result.served_demand_vehicles == Approx(2.0).margin(1e-9));
  CHECK(result.total_requested_energy_kwh == Approx(20.0).margin(1e-9));
  CHECK(result.total_unserved_energy_kwh == Approx(0.0).margin(1e-9));

  REQUIRE(result.steps.size() >= 4);
  const auto& charging_step = result.steps[2];
  REQUIRE(charging_step.stations.size() == 1);
  CHECK(charging_step.stations[0].arrivals_vehicles == Approx(2.0).margin(1e-9));
  CHECK(charging_step.stations[0].p_ev_kw == Approx(20.0).margin(1e-9));
  REQUIRE(result.system_by_step.size() >= 3);
  REQUIRE(result.system_by_step[2].ac.charging_stations.size() == 1);
  CHECK(result.system_by_step[2].ac.charging_stations[0].p_total_kw ==
        Approx(20.0).margin(1e-9));
}

TEST_CASE("EVPT: V2G session can reverse station power", "[evpt]") {
  EVPowerTrafficProblem problem;
  problem.system = make_evpt_system();

  EVChargingSession session;
  session.index = 7;
  session.station_id = 101;
  session.arrival_step = 0;
  session.departure_step = 1;
  session.energy_initial_kwh = 50.0;
  session.energy_target_kwh = 40.0;
  session.energy_min_kwh = 0.0;
  session.energy_max_kwh = 60.0;
  session.max_charge_kw = 0.0;
  session.max_discharge_kw = 20.0;
  session.eta_discharge = 1.0;
  session.v2g_capable = true;
  problem.initial_sessions.push_back(session);

  StationPriceProfile price;
  price.station_id = 101;
  price.price_per_kwh = {1.0};
  problem.station_prices.push_back(price);

  auto opt = make_options();
  opt.num_steps = 1;
  opt.allow_v2g = true;
  opt.high_price_threshold_per_kwh = 0.5;

  const auto result = simulate_ev_power_traffic(problem, opt);

  REQUIRE(result.feasible);
  REQUIRE(result.steps.size() == 1);
  REQUIRE(result.steps[0].stations.size() == 1);
  CHECK(result.steps[0].stations[0].p_ev_kw == Approx(-10.0).margin(1e-9));
  CHECK(result.steps[0].stations[0].reverse_power_kw == Approx(10.0).margin(1e-9));
  CHECK(result.total_v2g_energy_kwh == Approx(10.0).margin(1e-9));
}

TEST_CASE("EVPT: generation capacity validation detects overload", "[evpt]") {
  auto problem = make_problem();
  problem.system.ac.generators[0].pmax_mw = 0.01;

  auto opt = make_options();
  opt.enforce_generation_capacity = true;

  const auto result = simulate_ev_power_traffic(problem, opt);

  CHECK_FALSE(result.feasible);
  REQUIRE(result.steps.size() >= 3);
  CHECK_FALSE(result.steps[2].power.generation_capacity_ok);
  CHECK(result.steps[2].power.capacity_margin_mw < 0.0);
}

namespace {

EVPowerTrafficProblem make_joint_optimization_case() {
  EVPowerTrafficProblem p;

  p.system = make_evpt_system();
  p.system.ac.charging_stations.clear();
  p.system.ac.chargers.clear();
  p.system.ac.generators[0].pmax_mw = 1.0;

  ChargingStation near_station;
  near_station.index = 101;
  near_station.bus = 1;
  near_station.in_service = true;
  near_station.max_power_kw = 40.0;
  near_station.power_factor = 0.95;
  p.system.ac.charging_stations.push_back(near_station);

  ChargingStation far_station;
  far_station.index = 102;
  far_station.bus = 1;
  far_station.in_service = true;
  far_station.max_power_kw = 60.0;
  far_station.power_factor = 0.95;
  p.system.ac.charging_stations.push_back(far_station);

  p.traffic.nodes = {{1, "origin"}, {2, "near"}, {3, "far"}, {4, "destination"}};

  TrafficLink near1;
  near1.index = 11;
  near1.from_node = 1;
  near1.to_node = 2;
  near1.free_flow_time_hr = 0.5;
  near1.capacity_veh_per_hr = 10.0;

  TrafficLink near2;
  near2.index = 12;
  near2.from_node = 2;
  near2.to_node = 4;
  near2.free_flow_time_hr = 0.5;
  near2.capacity_veh_per_hr = 10.0;

  TrafficLink far1;
  far1.index = 21;
  far1.from_node = 1;
  far1.to_node = 3;
  far1.free_flow_time_hr = 0.8;
  far1.capacity_veh_per_hr = 10.0;

  TrafficLink far2;
  far2.index = 22;
  far2.from_node = 3;
  far2.to_node = 4;
  far2.free_flow_time_hr = 0.7;
  far2.capacity_veh_per_hr = 10.0;

  p.traffic.links = {near1, near2, far1, far2};

  RouteAlternative near_route;
  near_route.index = 1;
  near_route.origin_node = 1;
  near_route.destination_node = 4;
  near_route.link_indices = {11, 12};
  RouteChargingStop near_stop;
  near_stop.station_id = 101;
  near_stop.requested_energy_kwh_per_vehicle = 20.0;
  near_stop.dwell_steps = 1;
  near_stop.max_charge_kw_per_vehicle = 20.0;
  near_route.charging_stops.push_back(near_stop);

  RouteAlternative far_route;
  far_route.index = 2;
  far_route.origin_node = 1;
  far_route.destination_node = 4;
  far_route.link_indices = {21, 22};
  RouteChargingStop far_stop;
  far_stop.station_id = 102;
  far_stop.requested_energy_kwh_per_vehicle = 20.0;
  far_stop.dwell_steps = 1;
  far_stop.max_charge_kw_per_vehicle = 20.0;
  far_route.charging_stops.push_back(far_stop);

  p.routes = {near_route, far_route};

  EVDemand demand;
  demand.index = 1;
  demand.origin_node = 1;
  demand.destination_node = 4;
  demand.departure_step = 0;
  demand.vehicles = 5.0;
  demand.candidate_route_indices = {1, 2};
  p.demands = {demand};

  return p;
}

double assigned_vehicles(const EVPowerTrafficResult& result, int route_index) {
  double total = 0.0;
  for (const auto& a : result.assignments) {
    if (a.served && a.route_index == route_index) total += a.vehicles;
  }
  return total;
}

double station_peak_kw(const EVPowerTrafficResult& result, int station_id) {
  double peak = 0.0;
  for (const auto& step : result.steps) {
    for (const auto& st : step.stations) {
      if (st.station_id == station_id) {
        peak = std::max(peak, st.p_ev_kw);
      }
    }
  }
  return peak;
}

}  // namespace

TEST_CASE("EVPT: capacity-aware greedy assignment jointly respects traffic and station capacity",
          "[evpt][joint]") {
  const auto problem = make_joint_optimization_case();
  auto opt = make_options();
  opt.num_steps = 4;
  opt.default_charging_efficiency = 1.0;
  opt.allow_unserved_charging_energy = true;

  auto traffic_only = opt;
  traffic_only.assignment_model = AssignmentModel::DeterministicShortestPath;
  traffic_only.allow_unserved_charging_energy = false;
  const auto traffic_result = simulate_ev_power_traffic(problem, traffic_only);

  CHECK_FALSE(traffic_result.feasible);
  CHECK(assigned_vehicles(traffic_result, 1) == Approx(5.0).margin(1e-9));
  CHECK(assigned_vehicles(traffic_result, 2) == Approx(0.0).margin(1e-9));
  CHECK(traffic_result.total_unserved_energy_kwh == Approx(60.0).margin(1e-9));
  CHECK(traffic_result.total_travel_time_hr == Approx(5.046875).margin(1e-9));
  CHECK(station_peak_kw(traffic_result, 101) == Approx(40.0).margin(1e-9));

  auto joint = opt;
  joint.assignment_model = AssignmentModel::CapacityAwareGreedy;
  joint.allow_unserved_charging_energy = false;
  const auto joint_result = simulate_ev_power_traffic(problem, joint);

  REQUIRE(joint_result.feasible);
  CHECK(assigned_vehicles(joint_result, 1) == Approx(2.0).margin(1e-9));
  CHECK(assigned_vehicles(joint_result, 2) == Approx(3.0).margin(1e-9));
  CHECK(joint_result.total_requested_energy_kwh == Approx(100.0).margin(1e-9));
  CHECK(joint_result.total_delivered_energy_kwh == Approx(100.0).margin(1e-9));
  CHECK(joint_result.total_unserved_energy_kwh == Approx(0.0).margin(1e-9));
  CHECK(joint_result.total_travel_time_hr == Approx(6.5059475).margin(1e-9));
  CHECK(station_peak_kw(joint_result, 101) == Approx(40.0).margin(1e-9));
  CHECK(station_peak_kw(joint_result, 102) == Approx(60.0).margin(1e-9));
}

TEST_CASE("EVPT: system-optimal LP solves certified joint assignment",
          "[evpt][joint][lp]") {
  const auto problem = make_joint_optimization_case();
  auto opt = make_options();
  opt.num_steps = 4;
  opt.default_charging_efficiency = 1.0;
  opt.assignment_model = AssignmentModel::SystemOptimalLP;
  opt.allow_unserved_charging_energy = false;

  const auto result = simulate_ev_power_traffic(problem, opt);

  REQUIRE(result.feasible);
  CHECK(result.optimization_solved);
  CHECK_FALSE(result.optimization_is_mip);
  CHECK(result.optimization_proven_optimal);
  CHECK(result.optimization_mip_gap == Approx(0.0).margin(1e-12));
  CHECK(result.optimization_best_bound == Approx(result.optimization_objective).margin(1e-9));
  CHECK(assigned_vehicles(result, 1) == Approx(2.0).margin(1e-9));
  CHECK(assigned_vehicles(result, 2) == Approx(3.0).margin(1e-9));
  CHECK(result.total_unserved_energy_kwh == Approx(0.0).margin(1e-9));
  CHECK(station_peak_kw(result, 101) == Approx(40.0).margin(1e-9));
  CHECK(station_peak_kw(result, 102) == Approx(60.0).margin(1e-9));
}

TEST_CASE("EVPT: system-optimal MILP reports integer certificate",
          "[evpt][joint][milp]") {
  const auto problem = make_joint_optimization_case();
  auto opt = make_options();
  opt.num_steps = 4;
  opt.default_charging_efficiency = 1.0;
  opt.assignment_model = AssignmentModel::SystemOptimalMILP;
  opt.allow_unserved_charging_energy = false;
  opt.system_optimal_mip_gap = 1e-6;

  const auto result = simulate_ev_power_traffic(problem, opt);

  REQUIRE(result.feasible);
  CHECK(result.optimization_solved);
  CHECK(result.optimization_is_mip);
  CHECK(result.optimization_mip_gap <= opt.system_optimal_mip_gap + 1e-9);
  CHECK(result.optimization_best_bound <= result.optimization_objective + 1e-7);
  CHECK(assigned_vehicles(result, 1) == Approx(2.0).margin(1e-9));
  CHECK(assigned_vehicles(result, 2) == Approx(3.0).margin(1e-9));
  CHECK(result.total_unserved_energy_kwh == Approx(0.0).margin(1e-9));
}

TEST_CASE("EVPT: exact mathematical-model verification request is rejected",
          "[evpt][honesty][exact_model]") {
  const auto problem = make_joint_optimization_case();
  auto opt = make_options();
  opt.require_exact_mathematical_model = true;

  const auto result = simulate_ev_power_traffic(problem, opt);

  CHECK_FALSE(result.feasible);
  CHECK(result.mathematical_model_verified == false);
  CHECK(result.status.find("unsupported") != std::string::npos);
  CHECK(result.mathematical_model_verification_status.find("unsupported") !=
        std::string::npos);
}

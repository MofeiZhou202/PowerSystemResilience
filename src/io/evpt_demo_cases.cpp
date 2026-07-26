#include "hacdcpf/io/evpt_demo_cases.hpp"

#include <stdexcept>

#include "hacdcpf/io/case_builders.hpp"

namespace hacdcpf::io {

using namespace hacdcpf::evpt;

std::vector<std::string> evpt_demo_case_names() {
  return {"evpt_demo_small", "evpt_demo_grid", "evpt_demo_comprehensive"};
}

EVPowerTrafficProblem build_evpt_demo_case(const std::string& name) {
  if (name == "evpt_demo_small") return build_evpt_demo_small();
  if (name == "evpt_demo_grid") return build_evpt_demo_grid();
  if (name == "evpt_demo_comprehensive") return build_evpt_demo_comprehensive();
  throw std::runtime_error("Unsupported EV-traffic demo case: " + name);
}

// ── evpt_demo_small ────────────────────────────────────────────────────────

namespace {

HybridPowerSystem make_two_bus_demo_system() {
  HybridPowerSystem sys;
  sys.name = "EVPT demo: 2-bus cheap/expensive";
  sys.base_mva = 1.0;
  sys.ac.base_mva = 1.0;

  {
    ACBus b; b.index = 1; b.bus_type = BusType::SLACK;
    b.base_kv = 110.0; b.vmax_pu = 1.05; b.vmin_pu = 0.95;
    b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b);
  }
  {
    ACBus b; b.index = 2; b.bus_type = BusType::PQ;
    b.base_kv = 110.0; b.vmax_pu = 1.05; b.vmin_pu = 0.95;
    b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b);
  }

  // G1: cheap baseload on bus 1;  G2: expensive peaker on bus 2.
  {
    Generator g; g.index = 1; g.bus = 1; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 2.0;
    g.cost_c2 = 0.0; g.cost_c1 = 10.0; g.cost_c0 = 0.0;
    g.is_slack = true;
    sys.ac.generators.push_back(g);
  }
  {
    Generator g; g.index = 2; g.bus = 2; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 0.5;
    g.cost_c2 = 0.0; g.cost_c1 = 50.0; g.cost_c0 = 0.0;
    g.is_slack = false;
    sys.ac.generators.push_back(g);
  }

  // Tie 1-2: 0.5 MW — congests once station 102's EV load grows.
  {
    ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.0; br.x_pu = 0.1; br.b_pu = 0.0;
    br.rate_a_mva = 0.5; br.in_service = true;
    sys.ac.branches.push_back(br);
  }

  {
    ChargingStation cs; cs.index = 101; cs.bus = 1; cs.in_service = true;
    cs.name = "CS-101"; cs.n_fast = 4; cs.p_fast_max_kw = 20.0;
    cs.max_power_kw = 80.0;
    sys.ac.charging_stations.push_back(cs);
  }
  {
    ChargingStation cs; cs.index = 102; cs.bus = 2; cs.in_service = true;
    cs.name = "CS-102"; cs.n_fast = 3; cs.p_fast_max_kw = 20.0;
    cs.max_power_kw = 60.0;
    sys.ac.charging_stations.push_back(cs);
  }

  return sys;
}

TrafficLink make_demo_link(int idx, int from, int to, double ff_hr, double cap,
                           double jam) {
  TrafficLink lk;
  lk.index = idx;
  lk.from_node = from;
  lk.to_node = to;
  lk.length_km = ff_hr * 10.0;  // 10 km/h effective free-flow speed
  lk.free_flow_time_hr = ff_hr;
  lk.capacity_veh_per_hr = cap;
  lk.jam_vehicles = jam;
  return lk;
}

}  // namespace

EVPowerTrafficProblem build_evpt_demo_small() {
  EVPowerTrafficProblem prob;
  prob.system = make_two_bus_demo_system();

  // Diamond road network: 1 → {2 upper, 3 lower} → 4.
  const double coords[4][2] = {{0.0, 1.0}, {1.0, 0.0}, {1.0, 2.0}, {2.0, 1.0}};
  for (int i = 1; i <= 4; ++i) {
    TrafficNode nd;
    nd.index = i;
    nd.name = "N" + std::to_string(i);
    nd.x = coords[i - 1][0];
    nd.y = coords[i - 1][1];
    prob.traffic.nodes.push_back(nd);
  }

  // Route 1 (near, via node 2): 0.5 h free-flow → station 101 (cheap bus).
  prob.traffic.links.push_back(make_demo_link(11, 1, 2, 0.25, 20.0, 10.0));
  prob.traffic.links.push_back(make_demo_link(12, 2, 4, 0.25, 20.0, 10.0));
  // Route 2 (far, via node 3): 1.0 h free-flow → station 102 (peaker bus).
  prob.traffic.links.push_back(make_demo_link(21, 1, 3, 0.50, 20.0, 10.0));
  prob.traffic.links.push_back(make_demo_link(22, 3, 4, 0.50, 20.0, 10.0));

  {
    RouteAlternative r; r.index = 1; r.origin_node = 1; r.destination_node = 4;
    r.link_indices = {11, 12};
    RouteChargingStop s; s.station_id = 101;
    s.requested_energy_kwh_per_vehicle = 10.0;
    s.dwell_steps = 2; s.max_charge_kw_per_vehicle = 20.0;
    s.v2g_capable = true; s.max_discharge_kw_per_vehicle = 10.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r);
  }
  {
    RouteAlternative r; r.index = 2; r.origin_node = 1; r.destination_node = 4;
    r.link_indices = {21, 22};
    RouteChargingStop s; s.station_id = 102;
    s.requested_energy_kwh_per_vehicle = 10.0;
    s.dwell_steps = 2; s.max_charge_kw_per_vehicle = 20.0;
    s.v2g_capable = true; s.max_discharge_kw_per_vehicle = 10.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r);
  }

  {
    EVDemand d; d.index = 1; d.origin_node = 1; d.destination_node = 4;
    d.departure_step = 0; d.vehicles = 4.0;
    d.candidate_route_indices = {1, 2};
    d.willingness_to_pay_per_vehicle = 50.0;
    d.energy_max_kwh = 30.0;  // battery cap: bounded V2G arbitrage headroom
    prob.demands.push_back(d);
  }

  // Fixed price spread so mode B exercises smart charging + V2G directly.
  {
    StationPriceProfile p; p.station_id = 101;
    p.price_per_kwh = {0.10, 0.10, 0.50, 0.50, 0.10, 0.10};
    prob.station_prices.push_back(p);
  }
  {
    StationPriceProfile p; p.station_id = 102;
    p.price_per_kwh = {0.05, 0.05, 0.50, 0.50, 0.05, 0.05};
    prob.station_prices.push_back(p);
  }

  return prob;
}

// ── evpt_demo_grid ─────────────────────────────────────────────────────────

namespace {

// 5×5 one-way (east/south) grid helpers, node ids 1..25 row-major.
int grid_node(int r, int c) { return r * 5 + c + 1; }
int grid_hlink(int r, int c) { return r * 4 + c + 1; }        // 1..20
int grid_vlink(int r, int c) { return 100 + r * 5 + c + 1; }  // 101..120

}  // namespace

EVPowerTrafficProblem build_evpt_demo_grid() {
  EVPowerTrafficProblem prob;

  // Power side: IEEE 33-bus feeder + a costlier local DG at the feeder tail.
  // The feeder-head branch rating is tightened so that EV charging load
  // congests it and the DG sets the downstream marginal price (LMP split).
  prob.system = build_case33bw_acdc();
  prob.system.name = "EVPT demo: case33bw + 5x5 road grid";
  {
    Generator dg;
    dg.index = 901;
    dg.name = "Local-DG";
    dg.bus = 18;
    dg.in_service = true;
    dg.pmin_mw = 0.0;
    dg.pmax_mw = 1.5;
    dg.cost_c2 = 0.0;
    dg.cost_c1 = 80.0;
    dg.cost_c0 = 0.0;
    dg.is_slack = false;
    prob.system.ac.generators.push_back(dg);
  }
  if (!prob.system.ac.branches.empty()) {
    // Feeder-head branch (bus 1 → 2) carries the whole feeder load
    // (≈3.7 MW): cap it just above that so EV load pushes it into congestion.
    prob.system.ac.branches.front().rate_a_mva = 3.8;
  }

  // Charging stations on feeder buses 6 / 15 / 30.
  const struct { int id; int bus; const char* name; } stations[] = {
      {201, 6, "CS-201"}, {202, 15, "CS-202"}, {203, 30, "CS-203"}};
  for (const auto& st : stations) {
    ChargingStation cs;
    cs.index = st.id;
    cs.bus = st.bus;
    cs.name = st.name;
    cs.in_service = true;
    cs.n_fast = 20;
    cs.p_fast_max_kw = 30.0;
    cs.max_power_kw = 400.0;
    prob.system.ac.charging_stations.push_back(cs);
  }

  // Traffic side: 5×5 one-way grid, 2 km links at 20 km/h free flow.
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 5; ++c) {
      TrafficNode nd;
      nd.index = grid_node(r, c);
      nd.name = "N" + std::to_string(r) + std::to_string(c);
      nd.x = 2.0 * c;
      nd.y = 2.0 * r;
      prob.traffic.nodes.push_back(nd);
    }
  }
  auto add_link = [&](int idx, int from, int to) {
    TrafficLink lk;
    lk.index = idx;
    lk.from_node = from;
    lk.to_node = to;
    lk.length_km = 2.0;
    lk.free_flow_time_hr = 0.1;  // 20 km/h
    lk.capacity_veh_per_hr = 1800.0;
    lk.jam_vehicles = 40.0;
    lk.drive_energy_kwh_per_veh_km = 0.18;
    prob.traffic.links.push_back(lk);
  };
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 4; ++c) {
      add_link(grid_hlink(r, c), grid_node(r, c), grid_node(r, c + 1));
    }
  }
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 5; ++c) {
      add_link(grid_vlink(r, c), grid_node(r, c), grid_node(r + 1, c));
    }
  }

  auto add_stop = [](RouteAlternative& r, int station_id, bool v2g) {
    RouteChargingStop s;
    s.station_id = station_id;
    s.requested_energy_kwh_per_vehicle = 8.0;
    s.dwell_steps = 2;
    s.max_charge_kw_per_vehicle = 20.0;
    if (v2g) {
      s.v2g_capable = true;
      s.max_discharge_kw_per_vehicle = 10.0;
    }
    r.charging_stops.push_back(s);
  };

  // OD 1 → 25 (NW corner to SE corner), three alternatives.
  {
    RouteAlternative r; r.index = 1; r.origin_node = 1; r.destination_node = 25;
    r.link_indices = {grid_hlink(0, 0), grid_hlink(0, 1), grid_hlink(0, 2),
                      grid_hlink(0, 3), grid_vlink(0, 4), grid_vlink(1, 4),
                      grid_vlink(2, 4), grid_vlink(3, 4)};
    add_stop(r, 201, true);
    prob.routes.push_back(r);
  }
  {
    RouteAlternative r; r.index = 2; r.origin_node = 1; r.destination_node = 25;
    r.link_indices = {grid_vlink(0, 0), grid_vlink(1, 0), grid_vlink(2, 0),
                      grid_vlink(3, 0), grid_hlink(4, 0), grid_hlink(4, 1),
                      grid_hlink(4, 2), grid_hlink(4, 3)};
    add_stop(r, 202, false);
    prob.routes.push_back(r);
  }
  {
    RouteAlternative r; r.index = 3; r.origin_node = 1; r.destination_node = 25;
    r.link_indices = {grid_hlink(0, 0), grid_vlink(0, 1), grid_hlink(1, 1),
                      grid_vlink(1, 2), grid_hlink(2, 2), grid_vlink(2, 3),
                      grid_hlink(3, 3), grid_vlink(3, 4)};
    add_stop(r, 201, true);
    prob.routes.push_back(r);
  }

  // OD 3 → 25 (top mid to SE corner), two alternatives.
  {
    RouteAlternative r; r.index = 4; r.origin_node = 3; r.destination_node = 25;
    r.link_indices = {grid_vlink(0, 2), grid_vlink(1, 2), grid_vlink(2, 2),
                      grid_vlink(3, 2), grid_hlink(4, 2), grid_hlink(4, 3)};
    add_stop(r, 202, false);
    prob.routes.push_back(r);
  }
  {
    RouteAlternative r; r.index = 5; r.origin_node = 3; r.destination_node = 25;
    r.link_indices = {grid_hlink(0, 2), grid_hlink(0, 3), grid_vlink(0, 4),
                      grid_vlink(1, 4), grid_vlink(2, 4), grid_vlink(3, 4)};
    add_stop(r, 203, true);
    prob.routes.push_back(r);
  }

  {
    EVDemand d; d.index = 1; d.origin_node = 1; d.destination_node = 25;
    d.departure_step = 0; d.vehicles = 12.0;
    d.candidate_route_indices = {1, 2, 3};
    d.willingness_to_pay_per_vehicle = 60.0;
    d.energy_max_kwh = 24.0;
    prob.demands.push_back(d);
  }
  {
    EVDemand d; d.index = 2; d.origin_node = 3; d.destination_node = 25;
    d.departure_step = 1; d.vehicles = 8.0;
    d.candidate_route_indices = {4, 5};
    d.willingness_to_pay_per_vehicle = 60.0;
    d.energy_max_kwh = 24.0;
    prob.demands.push_back(d);
  }

  return prob;
}

// ── evpt_demo_comprehensive ───────────────────────────────────────────────

EVPowerTrafficProblem build_evpt_demo_comprehensive() {
  EVPowerTrafficProblem prob = build_evpt_demo_grid();
  prob.system.name = "EVPT comprehensive verification: 33-bus + urban grid";

  // A pair of time-varying central bottlenecks creates departure-dependent
  // congestion and makes the DUE route split observable in the GUI.
  for (auto& link : prob.traffic.links) {
    if (link.index == grid_hlink(2, 2) || link.index == grid_vlink(2, 2)) {
      link.capacity_veh_per_hr = 18.0;
      link.jam_vehicles = 12.0;
      link.capacity_profile_veh_per_hr =
          {18.0, 12.0, 7.0, 7.0, 10.0, 16.0, 18.0, 18.0};
    }
  }

  auto add_stop = [](RouteAlternative& route, int station_id, bool v2g) {
    RouteChargingStop stop;
    stop.station_id = station_id;
    stop.requested_energy_kwh_per_vehicle = 12.0;
    stop.dwell_steps = 2;
    stop.max_charge_kw_per_vehicle = 22.0;
    stop.v2g_capable = v2g;
    stop.max_discharge_kw_per_vehicle = v2g ? 11.0 : 0.0;
    route.charging_stops.push_back(stop);
  };

  // A third OD pair (node 6 → 25) has north/east and south/east alternatives
  // connected to different feeder buses and tariffs.
  {
    RouteAlternative route;
    route.index = 6;
    route.origin_node = 6;
    route.destination_node = 25;
    route.link_indices = {grid_hlink(1, 0), grid_hlink(1, 1),
                          grid_hlink(1, 2), grid_hlink(1, 3),
                          grid_vlink(1, 4), grid_vlink(2, 4),
                          grid_vlink(3, 4)};
    add_stop(route, 203, true);
    prob.routes.push_back(route);
  }
  {
    RouteAlternative route;
    route.index = 7;
    route.origin_node = 6;
    route.destination_node = 25;
    route.link_indices = {grid_vlink(1, 0), grid_vlink(2, 0),
                          grid_vlink(3, 0), grid_hlink(4, 0),
                          grid_hlink(4, 1), grid_hlink(4, 2),
                          grid_hlink(4, 3)};
    add_stop(route, 202, false);
    prob.routes.push_back(route);
  }

  // Increase the original cohorts enough to expose capacity and price
  // feedback, then add later departures to exercise dynamic propagation.
  prob.demands[0].vehicles = 24.0;
  prob.demands[0].energy_max_kwh = 40.0;
  prob.demands[1].vehicles = 16.0;
  prob.demands[1].energy_max_kwh = 40.0;
  {
    EVDemand demand;
    demand.index = 3;
    demand.origin_node = 6;
    demand.destination_node = 25;
    demand.departure_step = 2;
    demand.vehicles = 18.0;
    demand.candidate_route_indices = {6, 7};
    demand.initial_energy_kwh = 18.0;
    demand.energy_min_kwh = 5.0;
    demand.energy_max_kwh = 45.0;
    demand.reserve_energy_kwh = 5.0;
    demand.willingness_to_pay_per_vehicle = 75.0;
    prob.demands.push_back(demand);
  }
  {
    EVDemand demand;
    demand.index = 4;
    demand.origin_node = 1;
    demand.destination_node = 25;
    demand.departure_step = 3;
    demand.departure_window_steps = {2, 3, 4};
    demand.vehicles = 10.0;
    demand.candidate_route_indices = {1, 2, 3};
    demand.initial_energy_kwh = 20.0;
    demand.energy_min_kwh = 5.0;
    demand.energy_max_kwh = 45.0;
    demand.reserve_energy_kwh = 5.0;
    demand.willingness_to_pay_per_vehicle = 80.0;
    prob.demands.push_back(demand);
  }

  prob.station_prices.clear();
  const struct {
    int station_id;
    std::vector<double> prices;
  } price_profiles[] = {
      {201, {0.11, 0.12, 0.32, 0.48, 0.40, 0.20, 0.13, 0.11}},
      {202, {0.18, 0.18, 0.22, 0.26, 0.24, 0.20, 0.18, 0.18}},
      {203, {0.25, 0.18, 0.09, 0.07, 0.08, 0.15, 0.24, 0.27}},
  };
  for (const auto& source : price_profiles) {
    StationPriceProfile profile;
    profile.station_id = source.station_id;
    profile.price_per_kwh = source.prices;
    prob.station_prices.push_back(std::move(profile));
  }

  return prob;
}

}  // namespace hacdcpf::io

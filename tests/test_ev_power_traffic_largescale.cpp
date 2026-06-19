/**
 * Large-Scale EV Fleet Routing — Scalability Benchmark
 *
 * Power grid  : IEEE 118-bus (case118.m) — first 100 buses each host
 *               one fast-charging station (150 kW, 3 fast plugs).
 *
 * Traffic grid: 10×10 Cartesian grid (100 nodes, 180 directed links).
 *               Node (row, col) maps to power-bus (row·10 + col + 1).
 *
 * Corridors   : 5 N–S corridors, each offering two routes:
 *               Route A — straight down the left column, one step right
 *                          at the bottom.  Charging stop at mid-left node.
 *               Route B — one step right at the top, straight down the
 *                          right column.  Charging stop at mid-right node.
 *
 * EV demands  : 5 corridors × 4 departure steps = 20 fleet demands.
 *               Each fleet has 500 vehicles (10 000 total).
 *               Each vehicle requests 30 kWh  @ up to 50 kW.
 *
 * Test cases
 *   1. CapacityAwareGreedy  — fast heuristic, checks balance across stations.
 *   2. SystemOptimalLP      — HiGHS certified optimum, times the LP solve.
 *
 * Metrics printed: solve wall-time, per-corridor assignment, station peak
 *                  load, aggregate energy delivery, unserved energy.
 *
 * Run: ./tests/test_ev_power_traffic_largescale --success
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;
using namespace hacdcpf::evpt;

// ============================================================
// Grid helpers
// ============================================================

/// 10×10 grid node index (1-based), mapping to power-bus of same number.
static int node_id(int row, int col) { return row * 10 + col + 1; }

/// Horizontal link index: (row, col)→(row, col+1) — indices 1..90.
static int hlink_id(int row, int col) { return row * 9 + col + 1; }

/// Vertical link index: (row, col)→(row+1, col) — indices 101..190.
static int vlink_id(int row, int col) { return 100 + row * 10 + col + 1; }

// ============================================================
// Build the EVPowerTrafficProblem
// ============================================================

static EVPowerTrafficProblem make_largescale_problem() {
  EVPowerTrafficProblem prob;

  // ----------------------------------------------------------
  // Power grid: IEEE-118, add 100 fast-charging stations
  // ----------------------------------------------------------
  {
    const std::string path =
        std::string(HACDCPF_TEST_DATA_DIR) + "/case118.m";
    prob.system = io::parse_matpower(path);
  }

  // Station s (s=1..100) on bus s: 150 kW total, 3 fast plugs @ 50 kW each.
  for (int s = 1; s <= 100; ++s) {
    ChargingStation cs;
    cs.index         = s;
    cs.name          = "CS_" + std::to_string(s);
    cs.bus           = s;
    cs.in_service    = true;
    // Fleet depot scale: 300 fast chargers × 50 kW = 15 MW per station.
    // Sized to accommodate a 500-vehicle fleet (aggregated demand) that
    // can be split 250/250 across two corridor stations without queueing.
    cs.n_fast        = 300;
    cs.n_slow        = 0;
    cs.p_fast_max_kw = 50.0;
    cs.max_power_kw  = 15000.0;
    prob.system.ac.charging_stations.push_back(cs);
  }

  // ----------------------------------------------------------
  // Traffic graph: 10×10 grid (100 nodes, 180 links)
  // ----------------------------------------------------------
  // Nodes: one per grid cell, matching power-bus numbering.
  for (int r = 0; r < 10; ++r) {
    for (int c = 0; c < 10; ++c) {
      TrafficNode nd;
      nd.index = node_id(r, c);
      nd.name  = "N" + std::to_string(r) + std::to_string(c);
      prob.traffic.nodes.push_back(nd);
    }
  }

  const double ff_hr         = 0.25;   // free-flow time per link
  const double cap_veh_per_hr = 2000.0; // 2 000 veh/hr per link

  // Horizontal links: (row, col)→(row, col+1), col=0..8
  for (int r = 0; r < 10; ++r) {
    for (int c = 0; c < 9; ++c) {
      TrafficLink lk;
      lk.index               = hlink_id(r, c);
      lk.from_node           = node_id(r, c);
      lk.to_node             = node_id(r, c + 1);
      lk.length_km           = 2.0;
      lk.free_flow_time_hr   = ff_hr;
      lk.capacity_veh_per_hr = cap_veh_per_hr;
      prob.traffic.links.push_back(lk);
    }
  }

  // Vertical links: (row, col)→(row+1, col), row=0..8
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 10; ++c) {
      TrafficLink lk;
      lk.index               = vlink_id(r, c);
      lk.from_node           = node_id(r,     c);
      lk.to_node             = node_id(r + 1, c);
      lk.length_km           = 2.0;
      lk.free_flow_time_hr   = ff_hr;
      lk.capacity_veh_per_hr = cap_veh_per_hr;
      prob.traffic.links.push_back(lk);
    }
  }

  // ----------------------------------------------------------
  // Routes: 5 N-S corridors × 2 routes each = 10 routes
  // Each vehicle requests 30 kWh @ up to 50 kW.
  // ----------------------------------------------------------
  //
  // Corridor k (k=0..4):
  //   left_col  = 2k,   right_col = 2k+1
  //   origin    = node(0, left_col)          (top of left column)
  //   dest      = node(9, right_col)         (bottom of right column)
  //
  //   Route A (idx = 2k+1): 9 vertical links down left_col,
  //                          then 1 horizontal link right at bottom.
  //                          Charging stop at node(4, left_col).
  //
  //   Route B (idx = 2k+2): 1 horizontal link right at top,
  //                          then 9 vertical links down right_col.
  //                          Charging stop at node(4, right_col).

  int route_idx = 1;
  for (int k = 0; k < 5; ++k) {
    const int lc = 2 * k;       // left column
    const int rc = 2 * k + 1;   // right column

    // --- Route A ---
    {
      RouteAlternative ra;
      ra.index        = route_idx++;
      ra.origin_node  = node_id(0, lc);
      ra.destination_node = node_id(9, rc);

      // 9 vertical links down left column
      for (int r = 0; r < 9; ++r) ra.link_indices.push_back(vlink_id(r, lc));
      // 1 horizontal link right at bottom
      ra.link_indices.push_back(hlink_id(9, lc));

      // Charging stop at mid-left node (row 4)
      RouteChargingStop stop;
      stop.station_id                       = node_id(4, lc);  // bus = node
      stop.requested_energy_kwh_per_vehicle = 30.0;
      stop.dwell_steps                      = 1;
      stop.max_charge_kw_per_vehicle        = 50.0;
      ra.charging_stops.push_back(stop);

      prob.routes.push_back(ra);
    }

    // --- Route B ---
    {
      RouteAlternative rb;
      rb.index        = route_idx++;
      rb.origin_node  = node_id(0, lc);
      rb.destination_node = node_id(9, rc);

      // 1 horizontal link right at top
      rb.link_indices.push_back(hlink_id(0, lc));
      // 9 vertical links down right column
      for (int r = 0; r < 9; ++r) rb.link_indices.push_back(vlink_id(r, rc));

      // Charging stop at mid-right node (row 4)
      RouteChargingStop stop;
      stop.station_id                       = node_id(4, rc);
      stop.requested_energy_kwh_per_vehicle = 30.0;
      stop.dwell_steps                      = 1;
      stop.max_charge_kw_per_vehicle        = 50.0;
      rb.charging_stops.push_back(stop);

      prob.routes.push_back(rb);
    }
  }

  // ----------------------------------------------------------
  // EV demands: 5 corridors × 4 departure steps = 20 demands.
  // Fleet size 500 vehicles per demand → 10 000 vehicles total.
  // ----------------------------------------------------------
  int demand_idx = 1;
  for (int k = 0; k < 5; ++k) {
    const int lc = 2 * k;
    const int route_A = 2 * k + 1;  // Route A index for corridor k
    const int route_B = 2 * k + 2;  // Route B index

    for (int dep = 0; dep < 4; ++dep) {
      EVDemand d;
      d.index               = demand_idx++;
      d.origin_node         = node_id(0, lc);
      d.destination_node    = node_id(9, 2 * k + 1);
      d.departure_step      = dep;
      d.vehicles            = 500.0;  // fleet of 500 vehicles
      d.candidate_route_indices = {route_A, route_B};
      prob.demands.push_back(d);
    }
  }

  return prob;
}

// ============================================================
// Options factory
// ============================================================

static EVPowerTrafficOptions make_options(AssignmentModel model) {
  EVPowerTrafficOptions opt;
  opt.num_steps                      = 12;
  opt.time_step_hr                   = 1.0;
  opt.assignment_model               = model;
  opt.allow_unserved_charging_energy = false;
  opt.allow_unserved_travel_demand   = false;
  opt.enforce_road_capacity          = true;
  opt.queueing_weight                = 0.1;
  opt.value_of_time_per_hr           = 1.0;
  opt.station_energy_cost_weight     = 1.0;
  opt.default_station_price_per_kwh  = 0.30;
  opt.default_charging_efficiency    = 1.0;
  opt.system_optimal_time_limit_sec  = 60.0;
  opt.enforce_generation_capacity    = false;  // focus on routing, skip PF
  opt.update_system_charging_stations = false;
  return opt;
}

// ============================================================
// Print helpers
// ============================================================

static void print_banner(const char* title) {
  std::printf("\n");
  std::printf("======================================================================\n");
  std::printf("  %s\n", title);
  std::printf("======================================================================\n");
}

static void print_assignments(const EVPowerTrafficResult& res,
                               const EVPowerTrafficProblem& prob) {
  std::printf("\n  Fleet Assignments  (demand | route | dep_step | vehicles | gen_cost)\n");
  std::printf("  %-8s %-8s %-8s %-10s %-10s\n",
              "demand", "route", "dep", "vehicles", "gen_cost");
  std::printf("  %s\n", std::string(50, '-').c_str());
  for (const auto& a : res.assignments) {
    if (!a.served) {
      std::printf("  %-8d %-8s %-8d %-10.0f  UNSERVED (%s)\n",
                  a.demand_index, "--", a.departure_step,
                  a.vehicles, a.status.c_str());
    } else {
      std::printf("  %-8d %-8d %-8d %-10.0f %-10.4f\n",
                  a.demand_index, a.route_index, a.departure_step,
                  a.vehicles, a.generalized_cost);
    }
  }
}

static void print_station_peaks(const EVPowerTrafficResult& res,
                                const std::vector<int>& station_ids) {
  // Build per-station peak from step results
  std::printf("\n  Station peak charging load (top-10 by load)\n");
  std::printf("  %-10s %-12s %-12s %-12s\n",
              "station", "peak_kW", "peak_q_veh", "peak_wait_hr");
  std::printf("  %s\n", std::string(52, '-').c_str());

  struct StaPeak {
    int id;
    double peak_kw;
    double peak_q;
    double peak_wait;
  };
  std::vector<StaPeak> peaks;

  for (int sid : station_ids) {
    double pk = 0.0, pq = 0.0, pw = 0.0;
    for (const auto& step : res.steps) {
      for (const auto& s : step.stations) {
        if (s.station_id == sid) {
          pk = std::max(pk, std::abs(s.p_ev_kw));
          pq = std::max(pq, s.queue_vehicles);
          pw = std::max(pw, s.waiting_time_hr);
        }
      }
    }
    if (pk > 0.0 || pq > 0.0)
      peaks.push_back({sid, pk, pq, pw});
  }

  std::sort(peaks.begin(), peaks.end(),
            [](const StaPeak& a, const StaPeak& b) {
              return a.peak_kw > b.peak_kw;
            });

  const int show = std::min(static_cast<int>(peaks.size()), 10);
  for (int i = 0; i < show; ++i) {
    std::printf("  %-10d %-12.1f %-12.1f %-12.4f\n",
                peaks[i].id, peaks[i].peak_kw,
                peaks[i].peak_q, peaks[i].peak_wait);
  }
}

static void print_summary(const char* label,
                           double elapsed_ms,
                           const EVPowerTrafficResult& res) {
  std::printf("\n  Summary — %s\n", label);
  std::printf("  %-38s %s\n",
              "wall-clock solve time:", "");
  std::printf("  %-38s %.1f ms\n", "", elapsed_ms);
  std::printf("  %-38s %.0f veh\n",
              "total fleet vehicles:", res.total_demand_vehicles);
  std::printf("  %-38s %.0f veh\n",
              "served demand:", res.served_demand_vehicles);
  std::printf("  %-38s %.0f veh\n",
              "unmet demand:", res.unmet_demand_vehicles);
  std::printf("  %-38s %.1f kWh\n",
              "requested energy:", res.total_requested_energy_kwh);
  std::printf("  %-38s %.1f kWh\n",
              "delivered energy:", res.total_delivered_energy_kwh);
  std::printf("  %-38s %.1f kWh\n",
              "unserved energy:", res.total_unserved_energy_kwh);
  std::printf("  %-38s %.2f veh·hr\n",
              "total travel time:", res.total_travel_time_hr);
  if (res.optimization_solved) {
    std::printf("  %-38s %.6g\n",
                "LP objective:", res.optimization_objective);
    std::printf("  %-38s %s\n",
                "proven optimal:", res.optimization_proven_optimal ? "yes" : "no");
    std::printf("  %-38s %.2e\n",
                "MIP gap:", res.optimization_mip_gap);
  }
}

// ============================================================
// Collect the station IDs used by routes in the problem
// ============================================================

static std::vector<int> route_station_ids(const EVPowerTrafficProblem& prob) {
  std::vector<int> ids;
  for (const auto& r : prob.routes) {
    for (const auto& s : r.charging_stops) {
      ids.push_back(s.station_id);
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

// ============================================================
// TEST CASES
// ============================================================

TEST_CASE("Large-scale EV fleet: CapacityAwareGreedy",
          "[ev_power_traffic][largescale][greedy]") {

  const auto prob = make_largescale_problem();
  auto opt        = make_options(AssignmentModel::CapacityAwareGreedy);

  print_banner("Scenario A: CapacityAwareGreedy  "
               "(100 stations, 10 routes, 20 fleet demands, 10 000 vehicles)");

  const auto t0 = std::chrono::steady_clock::now();
  const auto res = simulate_ev_power_traffic(prob, opt);
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  print_assignments(res, prob);
  print_station_peaks(res, route_station_ids(prob));
  print_summary("CapacityAwareGreedy", elapsed_ms, res);

  // Assertions: all fleets routed, all energy delivered (stations have
  // capacity 150 kW each; with allow_unserved=false the greedy should
  // find feasible routes for each fleet departure).
  REQUIRE( res.feasible );
  CHECK( res.total_demand_vehicles  == Catch::Approx(10000.0).margin(1e-6) );
  CHECK( res.served_demand_vehicles  == Catch::Approx(10000.0).margin(1e-6) );
  CHECK( res.total_unserved_energy_kwh == Catch::Approx(0.0).margin(1e-3) );
  // 10 000 vehicles × 30 kWh = 300 000 kWh must be delivered.
  CHECK( res.total_delivered_energy_kwh == Catch::Approx(300000.0).margin(1.0) );
  CHECK( elapsed_ms < 5000.0 );   // must complete in < 5 s
}

TEST_CASE("Large-scale EV fleet: SystemOptimalLP",
          "[ev_power_traffic][largescale][lp]") {

  const auto prob = make_largescale_problem();
  auto opt        = make_options(AssignmentModel::SystemOptimalLP);

  print_banner("Scenario B: SystemOptimalLP  "
               "(100 stations, 10 routes, 20 fleet demands, 10 000 vehicles)");

  const auto t0 = std::chrono::steady_clock::now();
  const auto res = simulate_ev_power_traffic(prob, opt);
  const auto t1 = std::chrono::steady_clock::now();
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  print_assignments(res, prob);
  print_station_peaks(res, route_station_ids(prob));
  print_summary("SystemOptimalLP", elapsed_ms, res);

  REQUIRE( res.feasible );
  CHECK( res.total_demand_vehicles  == Catch::Approx(10000.0).margin(1e-6) );
  CHECK( res.total_unserved_energy_kwh == Catch::Approx(0.0).margin(1e-3) );
  CHECK( res.total_delivered_energy_kwh == Catch::Approx(300000.0).margin(1.0) );
  CHECK( res.optimization_proven_optimal );
  CHECK( res.optimization_mip_gap == Catch::Approx(0.0).margin(1e-10) );
  CHECK( elapsed_ms < 30000.0 );  // LP must solve in < 30 s
}

TEST_CASE("Large-scale EV fleet: station utilisation statistics",
          "[ev_power_traffic][largescale][stats]") {

  // Run greedy (fast) and report utilisation across all 10 route stations.
  const auto prob = make_largescale_problem();
  auto opt        = make_options(AssignmentModel::CapacityAwareGreedy);
  const auto res  = simulate_ev_power_traffic(prob, opt);

  const auto sta_ids = route_station_ids(prob);

  // For each station, compute: peak load, total delivered, active steps.
  print_banner("Station Utilisation Statistics (CapacityAwareGreedy)");
  std::printf("\n  %-10s %-12s %-15s %-10s\n",
              "station", "peak_kW", "delivered_kWh", "act_steps");
  std::printf("  %s\n", std::string(52, '-').c_str());

  double total_delivered = 0.0;
  for (int sid : sta_ids) {
    double peak = 0.0, delivered = 0.0;
    int act_steps = 0;
    for (const auto& step : res.steps) {
      for (const auto& s : step.stations) {
        if (s.station_id != sid) continue;
        if (s.p_ev_kw > 0.0) {
          peak = std::max(peak, s.p_ev_kw);
          delivered += s.p_ev_kw * opt.time_step_hr;
          ++act_steps;
        }
      }
    }
    total_delivered += delivered;
    std::printf("  %-10d %-12.1f %-15.1f %-10d\n",
                sid, peak, delivered, act_steps);
  }

  std::printf("\n  Total delivered across %zu route stations: %.1f kWh\n",
              sta_ids.size(), total_delivered);
  std::printf("  Expected (10 000 veh × 30 kWh):           %.1f kWh\n",
              10000.0 * 30.0);

  // Delivered across route stations must equal total requested
  // (allow_unserved=false, station capacity large enough for split routing).
  CHECK( total_delivered == Catch::Approx(10000.0 * 30.0).margin(10.0) );
}

// test_ev_power_traffic_ctm_joint.cpp
// ──────────────────────────────────────────────────────────────────────────
// Tests for Formulation C: CTM-DUE jointly coupled with DC-OPF
// via iterative LMP price coordination.
//
// Network (same as test_ev_power_traffic_joint_opt.cpp):
//   Power grid: 2-bus system
//     Bus 1 (SLACK): generator G1 — cheap  (c1 = 10 $/MWh)
//     Bus 2  (PQ)  : generator G2 — costly (c1 = 50 $/MWh)
//     Branch 1-2: x=0.1 pu, limit 0.5 MW
//   Traffic: 4-node, 2-route (origin=1, destination=4)
//     Route 1 (near): station 101 on bus 1 — ff 0.5 h
//     Route 2 (far) : station 102 on bus 2 — ff 1.0 h
//   Demand: 4 vehicles, WTP = $50/vehicle.
//
// Tests
//   C1: No OPF coupling — one outer iteration, prices unchanged, result struct valid
//   C2: OPF coupling    — prices update toward LMPs, converge or exhaust budget
//   C3: Social welfare  — W = B_EV - C_gen - C_delay is finite and consistent
//   C4: Price sensitivity (generalized cost): asymmetric price → route shift
//   Analysis: print results for LaTeX case study
//
// Run: ./tests/test_ev_power_traffic_ctm_joint
//      ./tests/test_ev_power_traffic_ctm_joint "[analysis]"
// ──────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: build 2-bus power system with cheap/expensive generators
// and two charging stations.
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem make_two_bus_system() {
  HybridPowerSystem sys;
  sys.base_mva      = 1.0;
  sys.ac.base_mva   = 1.0;

  // Bus 1 — slack
  { ACBus b; b.index = 1; b.bus_type = BusType::SLACK;
    b.base_kv = 110.0; b.vmax_pu = 1.05; b.vmin_pu = 0.95;
    b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b); }

  // Bus 2 — PQ
  { ACBus b; b.index = 2; b.bus_type = BusType::PQ;
    b.base_kv = 110.0; b.vmax_pu = 1.05; b.vmin_pu = 0.95;
    b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b); }

  // G1: cheap baseload on bus 1
  { Generator g; g.index = 1; g.bus = 1; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 2.0;
    g.cost_c2 = 0.0; g.cost_c1 = 10.0; g.cost_c0 = 0.0;
    g.is_slack = true;
    sys.ac.generators.push_back(g); }

  // G2: expensive peaker on bus 2
  { Generator g; g.index = 2; g.bus = 2; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 0.5;
    g.cost_c2 = 0.0; g.cost_c1 = 50.0; g.cost_c0 = 0.0;
    g.is_slack = false;
    sys.ac.generators.push_back(g); }

  // Branch 1-2: lossless DC, x=0.1 pu, limit=0.5 MW
  { ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
    br.r_pu = 0.0; br.x_pu = 0.1; br.b_pu = 0.0;
    br.rate_a_mva = 0.5; br.in_service = true;
    sys.ac.branches.push_back(br); }

  // Station 101 on bus 1: 40 kW
  { ChargingStation cs; cs.index = 101; cs.bus = 1; cs.in_service = true;
    cs.n_fast = 2; cs.p_fast_max_kw = 20.0; cs.max_power_kw = 40.0;
    sys.ac.charging_stations.push_back(cs); }

  // Station 102 on bus 2: 60 kW
  { ChargingStation cs; cs.index = 102; cs.bus = 2; cs.in_service = true;
    cs.n_fast = 3; cs.p_fast_max_kw = 20.0; cs.max_power_kw = 60.0;
    sys.ac.charging_stations.push_back(cs); }

  return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: build the 4-node 2-route traffic problem coupled to the 2-bus grid.
// Routes:
//   Route 1 (near): 1→2→4, ff=0.25+0.25=0.50 hr  station 101 (bus 1, cheap)
//   Route 2 (far) : 1→3→4, ff=0.50+0.50=1.00 hr  station 102 (bus 2, costly)
// ─────────────────────────────────────────────────────────────────────────────
static EVPowerTrafficProblem make_ctm_joint_problem(
    double n_vehicles = 4.0,
    double wtp_per_veh = 50.0,
    double e_req_per_veh = 10.0) {
  EVPowerTrafficProblem prob;
  prob.system = make_two_bus_system();

  // Traffic nodes
  for (int i = 1; i <= 4; ++i) {
    TrafficNode nd; nd.index = i;
    nd.name = "N" + std::to_string(i);
    prob.traffic.nodes.push_back(nd);
  }

  // Traffic links
  auto mklnk = [&](int idx, int from, int to, double ff_hr, double cap) {
    TrafficLink lk; lk.index = idx; lk.from_node = from; lk.to_node = to;
    lk.length_km = ff_hr * 10.0;  // 10 km/h effective speed (length matches ff)
    lk.free_flow_time_hr = ff_hr;
    lk.capacity_veh_per_hr = cap;
    lk.jam_vehicles = 10.0;
    return lk;
  };
  // Route 1 links (near): 1→2, 2→4
  prob.traffic.links.push_back(mklnk(11, 1, 2, 0.25, 20.0));
  prob.traffic.links.push_back(mklnk(12, 2, 4, 0.25, 20.0));
  // Route 2 links (far):  1→3, 3→4
  prob.traffic.links.push_back(mklnk(21, 1, 3, 0.50, 20.0));
  prob.traffic.links.push_back(mklnk(22, 3, 4, 0.50, 20.0));

  // Route 1: near, station 101
  { RouteAlternative r; r.index = 1; r.origin_node = 1; r.destination_node = 4;
    r.link_indices = {11, 12};
    RouteChargingStop s; s.station_id = 101;
    s.requested_energy_kwh_per_vehicle = e_req_per_veh;
    s.dwell_steps = 1; s.max_charge_kw_per_vehicle = 20.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r); }

  // Route 2: far, station 102
  { RouteAlternative r; r.index = 2; r.origin_node = 1; r.destination_node = 4;
    r.link_indices = {21, 22};
    RouteChargingStop s; s.station_id = 102;
    s.requested_energy_kwh_per_vehicle = e_req_per_veh;
    s.dwell_steps = 1; s.max_charge_kw_per_vehicle = 20.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r); }

  // Demand
  { EVDemand d; d.index = 1; d.origin_node = 1; d.destination_node = 4;
    d.departure_step = 0; d.vehicles = n_vehicles;
    d.candidate_route_indices = {1, 2};
    d.willingness_to_pay_per_vehicle = wtp_per_veh;
    prob.demands.push_back(d); }

  return prob;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: base options for Formulation C
// ─────────────────────────────────────────────────────────────────────────────
static CTMJointWelfareOptions make_ctm_joint_options(bool use_opf) {
  CTMJointWelfareOptions opt;
  opt.max_iterations        = 20;
  opt.price_convergence_tol = 1e-4;
  opt.price_update_step     = 0.6;
  opt.use_dcopf_prices      = use_opf;
  opt.verbose               = false;

  opt.evpt_opts.num_steps                       = 6;
  opt.evpt_opts.time_step_hr                    = 1.0;
  opt.evpt_opts.default_charging_efficiency     = 1.0;
  opt.evpt_opts.value_of_time_per_hr            = 1.0;
  opt.evpt_opts.station_energy_cost_weight      = 1.0;
  opt.evpt_opts.default_station_price_per_kwh   = 0.03;
  opt.evpt_opts.enforce_generation_capacity     = false;
  opt.evpt_opts.update_system_charging_stations = false;

  opt.ctm_opts.dt_ctm_hr            = 0.05;
  opt.ctm_opts.n_cells_per_link     = 3;
  opt.ctm_opts.route_specific_cells = true;

  opt.due_opts.max_iterations    = 30;
  opt.due_opts.convergence_tol   = 5e-3;
  opt.due_opts.logit_theta       = 0.0;  // AoN for determinism

  opt.dcopf_opts.compute_lmp   = true;
  opt.dcopf_opts.load_shedding = true;
  opt.dcopf_opts.verbose       = false;

  return opt;
}

// =============================================================================
// Test C1: No OPF coupling — fixed prices, one outer iteration
// =============================================================================

TEST_CASE("CTM-Joint C: no-OPF pass (fixed prices, single iteration)",
          "[ctm][joint][formulation_c][c1]") {

  const auto prob = make_ctm_joint_problem();
  auto opts       = make_ctm_joint_options(/*use_opf=*/false);

  const CTMJointWelfareResult res =
      simulate_ev_power_traffic_ctm_joint(prob, opts);

  // With no OPF, we converge in exactly 1 iteration
  CHECK(res.converged == true);
  CHECK(res.iterations == 1);
  CHECK(!res.status.empty());

  // Result struct should be populated
  CHECK(!res.history.empty());
  CHECK(res.history.front().iteration == 1);

  // Station prices unchanged from default (0.03 $/kWh)
  for (const auto& [sid, pvec] : res.station_prices_per_step) {
    for (double p : pvec) {
      CHECK(p == Approx(opts.evpt_opts.default_station_price_per_kwh).margin(1e-6));
    }
  }

  // OPF results should be empty (no OPF run)
  CHECK(res.opf_by_step.empty());

  // CTM-DUE should have produced sessions
  CHECK(!res.final_ctm_due.sessions.empty());
  CHECK(res.final_ctm_due.total_requested_energy_kwh > 0.0);
  CHECK(res.final_ctm_due.total_delivered_energy_kwh >= 0.0);

  // Social welfare components are populated
  CHECK(std::isfinite(res.social_welfare));
  CHECK(std::isfinite(res.ev_gross_benefit));
  CHECK(std::isfinite(res.traffic_delay_cost));
  CHECK(res.traffic_delay_cost >= 0.0);

  // History record has correct iteration
  CHECK(res.history[0].due_gap >= 0.0);
}

// =============================================================================
// Test C2: OPF coupling — prices update from LMPs and converge
// =============================================================================

TEST_CASE("CTM-Joint C: OPF-coupled price convergence",
          "[ctm][joint][formulation_c][c2]") {

  const auto prob = make_ctm_joint_problem();
  auto opts       = make_ctm_joint_options(/*use_opf=*/true);
  opts.verbose    = false;

  const CTMJointWelfareResult res =
      simulate_ev_power_traffic_ctm_joint(prob, opts);

  // Ran at least one iteration
  CHECK(res.iterations >= 1);
  CHECK(!res.status.empty());

  // After convergence or max_iterations, prices should differ from the initial
  // default (OPF feedback has occurred)
  bool any_price_changed = false;
  const double init_price = opts.evpt_opts.default_station_price_per_kwh;
  for (const auto& [sid, pvec] : res.station_prices_per_step) {
    for (double p : pvec) {
      if (std::abs(p - init_price) > 1e-8) { any_price_changed = true; break; }
    }
  }
  CHECK(any_price_changed == true);

  // OPF results populated
  CHECK(res.opf_by_step.size() == static_cast<std::size_t>(opts.evpt_opts.num_steps));

  // At least one step should have a converged OPF
  bool any_opf_conv = false;
  for (const auto& opf_r : res.opf_by_step) {
    if (opf_r.converged) { any_opf_conv = true; break; }
  }
  CHECK(any_opf_conv == true);

  // History: price_change should be decreasing (or zero on light network)
  CHECK(!res.history.empty());
  CHECK(res.history.back().price_change >= 0.0);

  // Social welfare finite
  CHECK(std::isfinite(res.social_welfare));
  CHECK(res.generation_cost >= 0.0);
  CHECK(res.traffic_delay_cost >= 0.0);
}

// =============================================================================
// Test C3: Social welfare accounting — W = B_EV - C_gen - C_delay
// =============================================================================

TEST_CASE("CTM-Joint C: social welfare accounting",
          "[ctm][joint][formulation_c][c3]") {

  const auto prob = make_ctm_joint_problem(/*n_vehicles=*/4.0, /*wtp=*/50.0);
  auto opts       = make_ctm_joint_options(/*use_opf=*/true);

  const CTMJointWelfareResult res =
      simulate_ev_power_traffic_ctm_joint(prob, opts);

  // W = B_EV - C_gen - C_delay (from the last iteration)
  const double W_expected =
      res.ev_gross_benefit - res.generation_cost - res.traffic_delay_cost;
  CHECK(res.social_welfare == Approx(W_expected).margin(1e-6));

  // B_EV = WTP * total_served = 50 * 4 = 200 $ (all vehicles served)
  CHECK(res.ev_gross_benefit == Approx(200.0).margin(0.1));

  // C_delay = VOT * TSTT >= 0
  CHECK(res.traffic_delay_cost >= 0.0);

  // Generation cost >= 0
  CHECK(res.generation_cost >= 0.0);

  // All vehicles should have at least received some energy
  CHECK(res.final_ctm_due.total_delivered_energy_kwh > 0.0);
}

// =============================================================================
// Test C4: Price sensitivity — station price affects route choice
//
// Set station 102 (bus 2) price very high → generalized cost of route 2
// increases → DUE shifts flow toward route 1.
// =============================================================================

TEST_CASE("CTM-Joint C: price sensitivity shifts route choice",
          "[ctm][joint][formulation_c][c4]") {

  auto prob = make_ctm_joint_problem(/*n_vehicles=*/4.0);

  // Set station 102 price very high: route 2 becomes much more expensive
  StationPriceProfile p1; p1.station_id = 101;
  p1.price_per_kwh.assign(6, 0.03);    // 0.03 $/kWh (cheap)
  StationPriceProfile p2; p2.station_id = 102;
  p2.price_per_kwh.assign(6, 5.00);    // 5.00 $/kWh (expensive!)
  prob.station_prices = {p1, p2};

  auto opts_low = make_ctm_joint_options(/*use_opf=*/false);
  opts_low.evpt_opts.station_energy_cost_weight = 1.0;
  opts_low.evpt_opts.value_of_time_per_hr       = 1.0;  // VOT=1 → price term dominates

  const CTMJointWelfareResult res =
      simulate_ev_power_traffic_ctm_joint(prob, opts_low);

  // With station 102 at 5 $/kWh and 10 kWh/veh, route 2 extra cost = 50 $/veh
  // normalized by VOT=1 → +50 h equivalent.  Route 1 = 0.5h + 0.03*10 = 0.8h.
  // Route 2 = 1.0h + 5.0*10 = 51.0h. AoN → all flow to route 1.
  double x1 = 0.0, x2 = 0.0;
  if (res.final_ctm_due.flow_by_demand_route.count(1)) {
    const auto& rm = res.final_ctm_due.flow_by_demand_route.at(1);
    if (rm.count(1)) x1 = rm.at(1);
    if (rm.count(2)) x2 = rm.at(2);
  }

  // Route 1 should dominate (generalized cost of route 2 is far higher)
  CHECK(x1 > x2);
  CHECK(x1 == Approx(4.0).margin(0.5));  // most vehicles on route 1
}

// =============================================================================
// Analysis test: print numerical results for LaTeX case study
// Run with: ./tests/test_ev_power_traffic_ctm_joint "[analysis]"
// =============================================================================

TEST_CASE("CTM-Joint C: numerical analysis — print results for LaTeX",
          "[analysis]") {

  std::printf("\n");
  std::printf("================================================================\n");
  std::printf(" Formulation C — Numerical Analysis\n");
  std::printf("================================================================\n");

  const auto prob_base = make_ctm_joint_problem(/*n_veh=*/4.0, /*wtp=*/50.0,
                                                 /*e_req=*/10.0);

  // ── Scenario C1: No OPF (fixed prices, single outer iteration) ────────────
  {
    std::printf("\n--- Scenario C1: Fixed prices (no OPF coupling) ---\n");
    auto opts = make_ctm_joint_options(/*use_opf=*/false);
    opts.verbose = false;
    const auto res = simulate_ev_power_traffic_ctm_joint(prob_base, opts);

    std::printf("  Outer iterations   : %d\n", res.iterations);
    std::printf("  Status             : %s\n", res.status.c_str());
    std::printf("  DUE converged      : %s\n",
                res.final_ctm_due.converged ? "true" : "false");
    std::printf("  DUE relative gap   : %.4e\n", res.final_ctm_due.relative_gap);
    std::printf("  Sessions created   : %zu\n", res.final_ctm_due.sessions.size());
    std::printf("  Requested energy   : %.2f kWh\n",
                res.final_ctm_due.total_requested_energy_kwh);
    std::printf("  Delivered energy   : %.2f kWh\n",
                res.final_ctm_due.total_delivered_energy_kwh);
    std::printf("  Service rate       : %.1f%%\n",
                res.final_ctm_due.total_requested_energy_kwh > 0.0
                    ? 100.0 * res.final_ctm_due.total_delivered_energy_kwh /
                          res.final_ctm_due.total_requested_energy_kwh
                    : 0.0);

    // Print route flows
    if (res.final_ctm_due.flow_by_demand_route.count(1)) {
      const auto& rm = res.final_ctm_due.flow_by_demand_route.at(1);
      std::printf("  Flow x1 (route 1)  : %.4f veh\n",
                  rm.count(1) ? rm.at(1) : 0.0);
      std::printf("  Flow x2 (route 2)  : %.4f veh\n",
                  rm.count(2) ? rm.at(2) : 0.0);
    }

    // Station loads
    for (const auto& [sid, load_vec] : res.final_ctm_due.station_ev_load_kw) {
      std::printf("  Station %d EV load [kW]: ", sid);
      for (int k = 0; k < static_cast<int>(load_vec.size()); ++k)
        std::printf(" %.2f", load_vec[static_cast<std::size_t>(k)]);
      std::printf("\n");
    }

    std::printf("  B_EV               : $%.4f\n", res.ev_gross_benefit);
    std::printf("  C_delay            : $%.4f\n", res.traffic_delay_cost);
    std::printf("  Social welfare     : $%.4f\n", res.social_welfare);
  }

  // ── Scenario C2: OPF coupled — price feedback ──────────────────────────────
  {
    std::printf("\n--- Scenario C2: OPF-coupled price feedback ---\n");
    auto opts = make_ctm_joint_options(/*use_opf=*/true);
    opts.verbose = false;
    opts.max_iterations = 20;
    const auto res = simulate_ev_power_traffic_ctm_joint(prob_base, opts);

    std::printf("  Outer iterations   : %d\n", res.iterations);
    std::printf("  Converged          : %s\n", res.converged ? "true" : "false");
    std::printf("  Final max Δπ       : %.2e $/kWh\n",
                res.history.empty() ? 0.0 : res.history.back().price_change);

    std::printf("  Convergence history:\n");
    std::printf("  %4s  %8s  %8s  %8s  %8s  %10s  %s\n",
                "Iter", "W[$]", "B_EV[$]", "C_gen[$]", "C_dly[$]",
                "Δπ[$/kWh]", "OPF");
    for (const auto& rec : res.history) {
      std::printf("  %4d  %8.4f  %8.4f  %8.4f  %8.4f  %10.2e  %s\n",
                  rec.iteration, rec.social_welfare, rec.ev_gross_benefit,
                  rec.generation_cost, rec.traffic_delay_cost,
                  rec.price_change,
                  rec.opf_converged ? "ok" : "FAIL");
    }

    // Final prices
    std::printf("  Converged station prices [$/kWh]:\n");
    for (const auto& [sid, pvec] : res.station_prices_per_step) {
      std::printf("  Station %d: ", sid);
      for (int k = 0; k < opts.evpt_opts.num_steps; ++k)
        std::printf(" %.5f", pvec[static_cast<std::size_t>(k)]);
      std::printf("\n");
    }

    // Route flows at convergence
    std::printf("  Final route flows:\n");
    if (res.final_ctm_due.flow_by_demand_route.count(1)) {
      const auto& rm = res.final_ctm_due.flow_by_demand_route.at(1);
      std::printf("    x1 (station 101, cheap) = %.4f veh\n",
                  rm.count(1) ? rm.at(1) : 0.0);
      std::printf("    x2 (station 102, costly) = %.4f veh\n",
                  rm.count(2) ? rm.at(2) : 0.0);
    }

    std::printf("  B_EV               : $%.4f\n", res.ev_gross_benefit);
    std::printf("  C_gen              : $%.4f\n", res.generation_cost);
    std::printf("  C_delay            : $%.4f\n", res.traffic_delay_cost);
    std::printf("  Social welfare W   : $%.4f\n", res.social_welfare);
  }

  // ── Scenario C3: Price-driven route shift ─────────────────────────────────
  {
    std::printf("\n--- Scenario C3: High price on route 2 drives flow to route 1 ---\n");
    auto prob = make_ctm_joint_problem();
    StationPriceProfile p1; p1.station_id = 101;
    p1.price_per_kwh.assign(6, 0.03);
    StationPriceProfile p2; p2.station_id = 102;
    p2.price_per_kwh.assign(6, 5.00);
    prob.station_prices = {p1, p2};

    auto opts = make_ctm_joint_options(/*use_opf=*/false);
    opts.evpt_opts.station_energy_cost_weight = 1.0;
    opts.evpt_opts.value_of_time_per_hr       = 1.0;
    const auto res = simulate_ev_power_traffic_ctm_joint(prob, opts);

    double x1 = 0.0, x2 = 0.0;
    if (res.final_ctm_due.flow_by_demand_route.count(1)) {
      const auto& rm = res.final_ctm_due.flow_by_demand_route.at(1);
      if (rm.count(1)) x1 = rm.at(1);
      if (rm.count(2)) x2 = rm.at(2);
    }
    std::printf("  Station 101 price  : 0.03 $/kWh\n");
    std::printf("  Station 102 price  : 5.00 $/kWh\n");
    std::printf("  Route 1 gen cost   : 0.50 + 0.03*10 = 0.80 h-equiv\n");
    std::printf("  Route 2 gen cost   : 1.00 + 5.00*10 = 51.0 h-equiv\n");
    std::printf("  x1 (route 1)       : %.4f veh\n", x1);
    std::printf("  x2 (route 2)       : %.4f veh\n", x2);
    std::printf("  Route 1 dominates  : %s\n", x1 > x2 ? "YES" : "NO");
  }

  std::printf("\n================================================================\n");
  std::printf(" End of Formulation C Analysis\n");
  std::printf("================================================================\n\n");
}

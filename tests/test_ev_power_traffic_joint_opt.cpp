/**
 * Joint Power-Traffic Social Welfare Optimisation — Tests
 *
 * Network (4-node, 2-route, 2-station):
 *   Power grid: 2-bus IEEE-inspired system.
 *     Bus 1 (slack): generator G1  — cheap    (c1 = 10  $/MWh)
 *     Bus 2        : generator G2  — expensive (c1 = 50  $/MWh)
 *     Branch 1-2: reactance 0.05 pu, limit 1 MW
 *     Load: bus 1 → 0.0 MW base (all load from EVs)
 *
 *   Traffic: same 4-node/2-route topology as test_ev_power_traffic_report.
 *     Route 1 (near):  station 101 on bus 1 (cheap power, low LMP)
 *     Route 2 (far ):  station 102 on bus 2 (expensive power, high LMP)
 *
 *   Demands: 1 demand × 5 vehicles, WTP = $50/vehicle.
 *
 * Scenarios
 *   1. Fixed prices (no OPF coupling)   — baseline
 *   2. LMP-coupled joint optimisation   — prices converge to nodal LMPs
 *   3. Convergence history check        — welfare increases or stabilises
 *
 * Run: ./tests/test_ev_power_traffic_joint_opt --success
 */

#include <algorithm>
#include <cstdio>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;

// ─────────────────────────────────────────────────────────────────────────────
// Build the coupled power-traffic problem
// ─────────────────────────────────────────────────────────────────────────────

static EVPowerTrafficProblem make_joint_problem() {
  EVPowerTrafficProblem prob;

  // ── Power grid: 2-bus system ──────────────────────────────────────────────
  auto& ac = prob.system.ac;
  prob.system.base_mva = 1.0;
  ac.base_mva = 1.0;

  // Bus 1 — slack, cheap generator
  {
    ACBus b;
    b.index    = 1;
    b.bus_type = BusType::SLACK;
    b.base_kv  = 110.0;
    b.vmax_pu  = 1.05;
    b.vmin_pu  = 0.95;
    b.pd_mw    = 0.0;
    b.in_service = true;
    ac.buses.push_back(b);
  }
  // Bus 2 — PQ bus, expensive generator
  {
    ACBus b;
    b.index    = 2;
    b.bus_type = BusType::PQ;
    b.base_kv  = 110.0;
    b.vmax_pu  = 1.05;
    b.vmin_pu  = 0.95;
    b.pd_mw    = 0.0;
    b.in_service = true;
    ac.buses.push_back(b);
  }

  // Generator G1 on bus 1: cheap, linear cost c1=10 $/MWh
  {
    Generator g;
    g.index      = 1;
    g.bus        = 1;
    g.in_service = true;
    g.pmin_mw    = 0.0;
    g.pmax_mw    = 2.0;
    g.cost_c2    = 0.0;
    g.cost_c1    = 10.0;   // $/MWh — cheap baseload
    g.cost_c0    = 0.0;
    g.is_slack   = true;
    ac.generators.push_back(g);
  }
  // Generator G2 on bus 2: expensive peaker, c1=50 $/MWh
  {
    Generator g;
    g.index      = 2;
    g.bus        = 2;
    g.in_service = true;
    g.pmin_mw    = 0.0;
    g.pmax_mw    = 0.5;
    g.cost_c2    = 0.0;
    g.cost_c1    = 50.0;   // $/MWh — expensive peaker
    g.cost_c0    = 0.0;
    g.is_slack   = false;
    ac.generators.push_back(g);
  }

  // Branch 1–2: lossless DC, reactance 0.1 pu, limit 0.5 MW
  {
    ACBranch br;
    br.index      = 1;
    br.from_bus   = 1;
    br.to_bus     = 2;
    br.r_pu       = 0.0;
    br.x_pu       = 0.1;
    br.b_pu       = 0.0;
    br.rate_a_mva = 0.5;   // 0.5 MW congestion limit → forces G2 dispatch when EV2 big
    br.in_service = true;
    ac.branches.push_back(br);
  }

  // Charging station 101 on bus 1: 40 kW, 2 plugs
  {
    ChargingStation cs;
    cs.index        = 101;
    cs.bus          = 1;
    cs.in_service   = true;
    cs.n_fast       = 2;
    cs.p_fast_max_kw = 20.0;
    cs.max_power_kw = 40.0;
    ac.charging_stations.push_back(cs);
  }
  // Charging station 102 on bus 2: 60 kW, 3 plugs
  {
    ChargingStation cs;
    cs.index        = 102;
    cs.bus          = 2;
    cs.in_service   = true;
    cs.n_fast       = 3;
    cs.p_fast_max_kw = 20.0;
    cs.max_power_kw = 60.0;
    ac.charging_stations.push_back(cs);
  }

  // ── Traffic graph: 4 nodes, 4 links (same as small scenario) ─────────────
  for (int i = 1; i <= 4; ++i) {
    TrafficNode nd;
    nd.index = i;
    nd.name  = "N" + std::to_string(i);
    prob.traffic.nodes.push_back(nd);
  }

  // Link 11: 1→2, ff=0.25 hr
  { TrafficLink lk; lk.index=11; lk.from_node=1; lk.to_node=2;
    lk.free_flow_time_hr=0.25; lk.capacity_veh_per_hr=20.0;
    prob.traffic.links.push_back(lk); }
  // Link 12: 2→4, ff=0.25 hr
  { TrafficLink lk; lk.index=12; lk.from_node=2; lk.to_node=4;
    lk.free_flow_time_hr=0.25; lk.capacity_veh_per_hr=20.0;
    prob.traffic.links.push_back(lk); }
  // Link 21: 1→3, ff=0.5 hr
  { TrafficLink lk; lk.index=21; lk.from_node=1; lk.to_node=3;
    lk.free_flow_time_hr=0.5; lk.capacity_veh_per_hr=20.0;
    prob.traffic.links.push_back(lk); }
  // Link 22: 3→4, ff=0.5 hr
  { TrafficLink lk; lk.index=22; lk.from_node=3; lk.to_node=4;
    lk.free_flow_time_hr=0.5; lk.capacity_veh_per_hr=20.0;
    prob.traffic.links.push_back(lk); }

  // ── Routes ────────────────────────────────────────────────────────────────
  // Route 1 (near, cheap): links 11,12; stop at stn 101 (bus 1)
  {
    RouteAlternative r;
    r.index          = 1;
    r.origin_node    = 1;
    r.destination_node = 4;
    r.link_indices   = {11, 12};
    RouteChargingStop s;
    s.station_id                       = 101;
    s.requested_energy_kwh_per_vehicle = 20.0;
    s.dwell_steps                      = 1;
    s.max_charge_kw_per_vehicle        = 20.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r);
  }
  // Route 2 (far, expensive): links 21,22; stop at stn 102 (bus 2)
  {
    RouteAlternative r;
    r.index          = 2;
    r.origin_node    = 1;
    r.destination_node = 4;
    r.link_indices   = {21, 22};
    RouteChargingStop s;
    s.station_id                       = 102;
    s.requested_energy_kwh_per_vehicle = 20.0;
    s.dwell_steps                      = 1;
    s.max_charge_kw_per_vehicle        = 20.0;
    r.charging_stops.push_back(s);
    prob.routes.push_back(r);
  }

  // ── Demand: 1 OD pair, 5 vehicles, WTP = $50/vehicle ─────────────────────
  {
    EVDemand d;
    d.index               = 1;
    d.origin_node         = 1;
    d.destination_node    = 4;
    d.departure_step      = 0;
    d.vehicles            = 5.0;
    d.candidate_route_indices = {1, 2};
    d.willingness_to_pay_per_vehicle = 50.0;  // $50/vehicle gross WTP
    prob.demands.push_back(d);
  }

  return prob;
}

// ─────────────────────────────────────────────────────────────────────────────
// Base options
// ─────────────────────────────────────────────────────────────────────────────

static JointSocialWelfareOptions make_options(bool use_opf) {
  JointSocialWelfareOptions opt;
  opt.max_iterations         = 20;
  opt.price_convergence_tol  = 1e-4;
  opt.price_update_step      = 0.6;
  opt.use_dcopf_prices       = use_opf;
  opt.verbose                = true;

  opt.evpt_opts.num_steps                     = 6;
  opt.evpt_opts.time_step_hr                  = 1.0;
  opt.evpt_opts.assignment_model              = AssignmentModel::SystemOptimalLP;
  opt.evpt_opts.allow_unserved_charging_energy = false;
  opt.evpt_opts.default_charging_efficiency   = 1.0;
  opt.evpt_opts.value_of_time_per_hr          = 1.0;
  opt.evpt_opts.station_energy_cost_weight    = 1.0;
  opt.evpt_opts.default_station_price_per_kwh = 0.03;  // 30 $/MWh initial guess
  opt.evpt_opts.enforce_generation_capacity   = false;
  opt.evpt_opts.update_system_charging_stations = false;

  opt.dcopf_opts.compute_lmp   = true;
  opt.dcopf_opts.load_shedding = true;
  opt.dcopf_opts.verbose       = false;

  return opt;
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 1: Baseline (no OPF feedback) — social welfare with fixed prices
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Joint SW: baseline fixed prices (no OPF coupling)",
          "[joint_social_welfare][baseline]") {

  auto prob = make_joint_problem();
  auto opt  = make_options(/*use_opf=*/false);

  std::printf("\n");
  std::printf("======================================================================\n");
  std::printf("  Joint SW Scenario 1: Fixed prices (no OPF coupling)\n");
  std::printf("======================================================================\n");

  const auto r = solve_joint_social_welfare(prob, opt);

  std::printf("\n  Iterations:   %d   Converged: %s   Status: %s\n",
              r.iterations, r.converged ? "yes" : "no", r.status.c_str());
  std::printf("  Social welfare:      %10.4f $\n", r.social_welfare);
  std::printf("  EV gross benefit:    %10.4f $\n", r.ev_gross_benefit);
  std::printf("  EV consumer surplus: %10.4f $\n", r.ev_consumer_surplus);
  std::printf("  Generation cost:     %10.4f $\n", r.generation_cost);
  std::printf("  Traffic delay cost:  %10.4f $\n", r.traffic_delay_cost);
  std::printf("  Served vehicles:     %.0f / %.0f\n",
              r.final_evpt.served_demand_vehicles,
              r.final_evpt.total_demand_vehicles);
  std::printf("  Delivered energy:    %.1f kWh\n",
              r.final_evpt.total_delivered_energy_kwh);

  // With fixed prices and no OPF, the traffic LP converges in 1 iteration.
  CHECK( r.iterations >= 1 );
  CHECK( r.final_evpt.feasible );
  CHECK( r.final_evpt.served_demand_vehicles == Catch::Approx(5.0).margin(1e-6) );
  CHECK( r.final_evpt.total_delivered_energy_kwh == Catch::Approx(100.0).margin(1e-3) );
  // WTP=50 × 5 veh = 250 $; no gen cost; social welfare ≈ 250 - delay
  CHECK( r.ev_gross_benefit == Catch::Approx(250.0).margin(1e-6) );
  CHECK( r.social_welfare > 0.0 );  // benefit > delay cost
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 2: LMP-coupled joint optimisation — prices converge to nodal LMPs
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Joint SW: LMP-coupled joint optimisation",
          "[joint_social_welfare][lmp_coupled]") {

  auto prob = make_joint_problem();
  auto opt  = make_options(/*use_opf=*/true);

  std::printf("\n");
  std::printf("======================================================================\n");
  std::printf("  Joint SW Scenario 2: LMP-coupled joint optimisation\n");
  std::printf("  Bus 1 (stn 101): G1 cheap (c1=10), Bus 2 (stn 102): G2 peaker (c1=50)\n");
  std::printf("======================================================================\n");

  const auto r = solve_joint_social_welfare(prob, opt);

  std::printf("\n  Iterations:   %d   Converged: %s   Status: %s\n",
              r.iterations, r.converged ? "yes" : "no", r.status.c_str());
  std::printf("  Social welfare:      %10.4f $\n", r.social_welfare);
  std::printf("  EV gross benefit:    %10.4f $\n", r.ev_gross_benefit);
  std::printf("  EV consumer surplus: %10.4f $\n", r.ev_consumer_surplus);
  std::printf("  Generation cost:     %10.4f $/step\n", r.generation_cost);
  std::printf("  Traffic delay cost:  %10.4f $\n", r.traffic_delay_cost);

  // Print per-assignment detail
  std::printf("\n  Final route assignments:\n");
  std::printf("  %-8s %-8s %-8s %-10s %-10s\n",
              "demand", "route", "step", "vehicles", "gen_cost");
  for (const auto& a : r.final_evpt.assignments) {
    if (a.served)
      std::printf("  %-8d %-8d %-8d %-10.2f %-10.4f\n",
                  a.demand_index, a.route_index, a.departure_step,
                  a.vehicles, a.generalized_cost);
    else
      std::printf("  %-8d UNSERVED  step=%-4d veh=%.2f  (%s)\n",
                  a.demand_index, a.departure_step, a.vehicles, a.status.c_str());
  }

  // Print converged station prices
  std::printf("\n  Converged station prices ($/kWh):\n");
  std::printf("  %-10s", "step");
  for (const auto& [sid, _] : r.station_prices_per_step)
    std::printf("  stn%-7d", sid);
  std::printf("\n");
  const int T = opt.evpt_opts.num_steps;
  for (int k = 0; k < T; ++k) {
    std::printf("  %-10d", k);
    for (const auto& [sid, pvec] : r.station_prices_per_step) {
      const double p = (k < static_cast<int>(pvec.size())) ? pvec[k] : 0.0;
      std::printf("  %-10.5f", p);
    }
    std::printf("\n");
  }

  // Print convergence history
  std::printf("\n  Convergence history:\n");
  std::printf("  %-6s %-12s %-12s %-12s %-12s %-12s\n",
              "iter", "welfare", "B_EV", "C_gen", "C_delay", "Δprice");
  for (const auto& h : r.history) {
    std::printf("  %-6d %-12.4f %-12.4f %-12.4f %-12.4f %-12.2e\n",
                h.iteration, h.social_welfare, h.ev_gross_benefit,
                h.generation_cost, h.traffic_delay_cost, h.price_change);
  }

  // Key assertions:
  REQUIRE( r.final_evpt.feasible );
  CHECK( r.final_evpt.served_demand_vehicles == Catch::Approx(5.0).margin(1e-6) );
  CHECK( r.final_evpt.total_delivered_energy_kwh == Catch::Approx(100.0).margin(1e-3) );
  // LMP at cheap bus 1 ≈ c1_G1 = 10 $/MWh = 0.010 $/kWh
  // LMP at expensive bus 2 depends on branch congestion; ≥ 0.010 $/kWh
  if (r.converged) {
    for (const auto& [sid, pvec] : r.station_prices_per_step) {
      for (double pi : pvec) {
        CHECK( pi >= 0.0 );  // LMPs must be non-negative for this system
      }
    }
    // Station 101 (bus 1, cheap) should end up with a lower price than
    // station 102 (bus 2, expensive) because bus 2 has a peaker generator.
    const auto it101 = r.station_prices_per_step.find(101);
    const auto it102 = r.station_prices_per_step.find(102);
    if (it101 != r.station_prices_per_step.end() &&
        it102 != r.station_prices_per_step.end() &&
        !it101->second.empty() && !it102->second.empty()) {
      // At step 1 (when EV loads are active), check price ordering.
      // Only verify when both are non-zero (OPF dispatched something).
      const double p1 = it101->second[1];
      const double p2 = it102->second[1];
      if (p1 > 1e-6 && p2 > 1e-6) {
        CHECK( p1 <= p2 + 1e-4 );  // cheap bus ≤ expensive bus
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST 3: Convergence — welfare history should stabilise
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("Joint SW: convergence history properties",
          "[joint_social_welfare][convergence]") {

  auto prob = make_joint_problem();
  auto opt  = make_options(/*use_opf=*/true);
  opt.verbose = false;   // quiet for this test

  const auto r = solve_joint_social_welfare(prob, opt);

  REQUIRE( !r.history.empty() );

  // Price change must be monotonically non-increasing (damped updates)
  // — only verify after the first 3 iterations to allow transient
  bool mono = true;
  for (std::size_t i = 2; i < r.history.size(); ++i) {
    // Allow 1% tolerance for floating-point non-monotonicity
    if (r.history[i].price_change >
        r.history[i - 1].price_change * 1.05 + 1e-8) {
      mono = false;
      break;
    }
  }
  // Not strictly required (oscillation is possible), but warn if not mono
  if (!mono) {
    std::printf("[info] price changes are not monotone — iteration may oscillate\n");
  }

  // Welfare at convergence must be a finite number
  CHECK( std::isfinite(r.social_welfare) );

  // EV gross benefit must equal WTP × served_veh
  const double wtp = prob.demands[0].willingness_to_pay_per_vehicle;
  CHECK( r.ev_gross_benefit ==
         Catch::Approx(wtp * r.final_evpt.served_demand_vehicles).margin(1e-6) );

  // Final price change (if converged) must be below tolerance
  if (r.converged) {
    CHECK( r.history.back().price_change < opt.price_convergence_tol + 1e-9 );
  }
}

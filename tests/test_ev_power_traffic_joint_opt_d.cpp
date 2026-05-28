// test_ev_power_traffic_joint_opt_d.cpp
// ──────────────────────────────────────────────────────────────────────────
// Numerical tests for Formulation D: single LP/QP/MILP/MCP joint optimizer.
//
// Network (same as Formulations A and C for cross-validation):
//   Power grid: 2-bus system
//     Bus 1 (SLACK): generator G1 — cheap  (c1 = 10 $/MWh, pmax = 2 MW)
//     Bus 2  (PQ)  : generator G2 — costly (c1 = 50 $/MWh, pmax = 0.5 MW)
//     Branch 1-2   : x = 0.1 pu, rate = 0.5 MW
//     Station 101 on bus 1: 40 kW capacity
//     Station 102 on bus 2: 60 kW capacity
//
//   Traffic: 4-node, 2-route (origin=1, destination=4)
//     Route 1 (near): 1→2→4, ff=0.50 h, station 101
//     Route 2 (far) : 1→3→4, ff=1.00 h, station 102
//
//   Demand: 4 vehicles, WTP=50 $/veh, e_req=10 kWh/veh, dep_step=0
//
// Test cases (Case Study 7 in technical notebook §Formulation D)
// ──────────────────────────────────────────────────────────────────────────
//   D1: SocialWelfareMax LP — proven_optimal=true, h1=4, h2=0,
//       optimized p_ch=40 kW delivers exactly 40 kWh; W=200−2=$198
//   D2: SocialWelfareMax LP + DC-OPF coupling (include_dcopf=true):
//       W = $197.60 (C_gen = $0.40 from G1 serving 0.04 MW EV load × 10$/MWh)
//   D3: UserBenefitMax LP — same route choice, obj includes electricity cost
//   D4: Welfare ≥ Formulation C iterative (single-shot LP is at least as good)
//   D5: MILP integer-flow mode — proven_optimal via B&C with integer vehicles
//   D6: price asymmetry — validates UserBenefitMax station-price term
//   D7: verbose SocialWelfareMax — analysis printout (no assertions, just runs)
//   D8: time-varying road blockage — availability_profile enforced in LP
//   D9: V2G + charge/discharge binaries — p_dis and δ mode constraints active

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/ev_power_traffic.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::evpt;
using Approx = Catch::Approx;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: build 2-bus power system
// ─────────────────────────────────────────────────────────────────────────────
static HybridPowerSystem make_two_bus_system_d() {
  HybridPowerSystem sys;
  sys.base_mva    = 1.0;
  sys.ac.base_mva = 1.0;

  // Bus 1 — slack (reference angle = 0)
  { ACBus b; b.index = 1; b.bus_type = BusType::SLACK;
    b.base_kv = 110.0; b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b); }

  // Bus 2 — PQ
  { ACBus b; b.index = 2; b.bus_type = BusType::PQ;
    b.base_kv = 110.0; b.pd_mw = 0.0; b.in_service = true;
    sys.ac.buses.push_back(b); }

  // G1: cheap baseload on bus 1  (c1 = 10 $/MWh)
  { Generator g; g.index = 1; g.bus = 1; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 2.0;
    g.cost_c2 = 0.0; g.cost_c1 = 10.0; g.cost_c0 = 0.0;
    g.is_slack = true;
    sys.ac.generators.push_back(g); }

  // G2: expensive peaker on bus 2 (c1 = 50 $/MWh)
  { Generator g; g.index = 2; g.bus = 2; g.in_service = true;
    g.pmin_mw = 0.0; g.pmax_mw = 0.5;
    g.cost_c2 = 0.0; g.cost_c1 = 50.0; g.cost_c0 = 0.0;
    g.is_slack = false;
    sys.ac.generators.push_back(g); }

  // Branch 1-2: lossless DC, x=0.1 pu, rate=0.5 MW
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
// Helper: build the 4-node 2-route EVPowerTrafficProblem
// Route 1 (near): 1→2→4, ff=0.50 h, station 101 (bus 1, cheap)
// Route 2 (far) : 1→3→4, ff=1.00 h, station 102 (bus 2, costly)
// ─────────────────────────────────────────────────────────────────────────────
static EVPowerTrafficProblem make_joint_opt_d_problem(
    double n_vehicles = 4.0,
    double wtp_per_veh = 50.0,
    double e_req_per_veh = 10.0) {
  EVPowerTrafficProblem prob;
  prob.system = make_two_bus_system_d();

  // Traffic nodes
  for (int i = 1; i <= 4; ++i) {
    TrafficNode nd; nd.index = i;
    nd.name = "N" + std::to_string(i);
    prob.traffic.nodes.push_back(nd);
  }

  // Traffic links
  auto mklnk = [&](int idx, int from, int to, double ff_hr, double cap) {
    TrafficLink lk; lk.index = idx; lk.from_node = from; lk.to_node = to;
    lk.free_flow_time_hr   = ff_hr;
    lk.capacity_veh_per_hr = cap;
    lk.jam_vehicles        = 10.0;
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

  // Demand: all vehicles depart at step 0
  { EVDemand d; d.index = 1; d.origin_node = 1; d.destination_node = 4;
    d.departure_step = 0; d.vehicles = n_vehicles;
    d.candidate_route_indices = {1, 2};
    d.willingness_to_pay_per_vehicle = wtp_per_veh;
    prob.demands.push_back(d); }

  return prob;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: base JointOptimizerOptions (no DC-OPF, SocialWelfareMax)
// ─────────────────────────────────────────────────────────────────────────────
static JointOptimizerOptions make_d_options(bool opf = false) {
  JointOptimizerOptions opt;
  opt.num_steps                            = 6;
  opt.time_step_hr                         = 1.0;
  opt.mode                                 = JointOptimizerMode::SocialWelfareMax;
  opt.include_dcopf                        = opf;
  opt.value_of_time_per_hr                 = 1.0;
  opt.station_energy_cost_weight           = 1.0;
  opt.default_charging_efficiency          = 1.0;
  opt.default_station_price_per_kwh        = 0.03;
  opt.enforce_road_capacity                = true;
  opt.enforce_generation_capacity          = false;
  opt.unserved_trip_penalty                = 1.0e6;
  opt.power_slack_penalty                  = 1.0e4;
  opt.mip_gap                              = 1.0e-6;
  opt.verbose                              = false;
  opt.dcopf_opts.compute_lmp               = true;
  opt.dcopf_opts.load_shedding             = false;
  opt.dcopf_opts.verbose                   = false;
  return opt;
}

// =============================================================================
// Test D1: SocialWelfareMax LP — no OPF coupling
// =============================================================================
// Expected:
//   - proven_optimal = true, feasible = true
//   - h_{1,1} = 4 vehicles: p_ch is optimized to 10 kW/veh for 1 h,
//     so station 101's 40 kW cap can deliver exactly 40 kWh
//   - h_{1,2} = 0 vehicles
//   - u_d = 0 (all served)
//   - B_EV = 4 × 50 = $200
//   - C_delay = 1 × (4 × 0.5) = $2.00
//   - C_gen = $0  (no OPF coupling → generator cost not in LP objective)
//   - W = $198
// =============================================================================
TEST_CASE("FormD D1: SocialWelfareMax LP — solver-certified, no OPF",
          "[ev_power_traffic][formulation_d][lp]") {

  const auto prob = make_joint_opt_d_problem();
  const auto opts = make_d_options(/*opf=*/false);

  const auto res = solve_joint_optimizer(prob, opts);

  SECTION("solver certificate") {
    CHECK(res.feasible       == true);
    CHECK(res.proven_optimal == true);
    CHECK(res.solver_status  != "failed");
    // LP: backend should be NativeDualSimplex or HiGHS
    CHECK(res.solver_backend != "");
    CHECK(res.n_variables    >  0);
    CHECK(res.n_constraints  >  0);
    CHECK(res.primal_max_violation      <= 1e-7);
    CHECK(res.integrality_max_violation <= 1e-9);
  }

  SECTION("route assignment: optimized charging keeps all flow on near route") {
    // Demand 1 → 4 vehicles on Route 1.  The LP chooses p_ch=40 kW,
    // not the old fixed 20 kW/vehicle route coefficient.
    REQUIRE(res.route_flow.count(1));
    const auto& rf = res.route_flow.at(1);
    const double h1 = rf.count(1) ? rf.at(1) : 0.0;
    const double h2 = rf.count(2) ? rf.at(2) : 0.0;
    CHECK(h1 == Approx(4.0).margin(0.01));
    CHECK(h2 == Approx(0.0).margin(0.01));
    CHECK(res.total_served_vehicles   == Approx(4.0).margin(0.01));
    CHECK(res.total_unserved_vehicles == Approx(0.0).margin(0.01));
  }

  SECTION("welfare accounting — no OPF") {
    // B_EV = 4 × 50 = 200
    CHECK(res.ev_benefit         == Approx(200.0).margin(0.1));
    // C_delay = VOT × (4×0.5) = 2.0
    CHECK(res.traffic_delay_cost == Approx(2.0).margin(0.1));
    // No generator dispatch in LP → C_gen = 0
    CHECK(res.gen_cost           == Approx(0.0).margin(1e-6));
    // W = 200 − 2 = 198
    CHECK(res.social_welfare     == Approx(198.0).margin(0.1));
  }

  SECTION("charging variables deliver exact requested energy") {
    REQUIRE(res.sessions.size() == 1u);
    REQUIRE(res.session_results.size() == 1u);
    // Requested energy: 4 vehicles × 10 kWh = 40 kWh total
    CHECK(res.total_requested_energy_kwh == Approx(40.0).margin(0.1));
    CHECK(res.total_delivered_energy_kwh == Approx(40.0).margin(0.1));
    const auto& sr = res.session_results.front();
    REQUIRE(sr.p_charge_kw.size() > 1u);
    CHECK(sr.p_charge_kw[1] == Approx(40.0).margin(1e-5));
    CHECK(sr.unserved_energy_kwh == Approx(0.0).margin(1e-6));
  }
}

// =============================================================================
// Test D2: SocialWelfareMax LP + DC-OPF power balance
// =============================================================================
// Expected:
//   - h1=4, h2=0
//   - EV load at bus 1: 40 kW = 0.04 MW (at step 1, arrival)
//   - G1 dispatched at 0.04 MW, G2 = 0
//   - C_gen = 10 × 0.04 × 1h = $0.40
//   - W = 200 − 2 − 0.40 = $197.60
// =============================================================================
TEST_CASE("FormD D2: SocialWelfareMax LP + DC-OPF simultaneous coupling",
          "[ev_power_traffic][formulation_d][lp][dcopf]") {

  const auto prob = make_joint_opt_d_problem();
  auto opts = make_d_options(/*opf=*/true);

  const auto res = solve_joint_optimizer(prob, opts);

  SECTION("solver certificate with OPF") {
    CHECK(res.feasible       == true);
    CHECK(res.proven_optimal == true);
    CHECK(res.n_variables    >  0);
    // With OPF: more variables than without (generators, angles, branches)
    CHECK(res.n_variables    > 4);  // at minimum: 2 route flows + 2 gen×6 + 2 angle×6 + ...
  }

  SECTION("route assignment unchanged by OPF coupling") {
    REQUIRE(res.route_flow.count(1));
    const auto& rf = res.route_flow.at(1);
    const double h1 = rf.count(1) ? rf.at(1) : 0.0;
    const double h2 = rf.count(2) ? rf.at(2) : 0.0;
    CHECK(h1 == Approx(4.0).margin(0.05));
    CHECK(h2 == Approx(0.0).margin(0.05));
    CHECK(res.total_unserved_vehicles == Approx(0.0).margin(0.01));
  }

  SECTION("welfare with generation cost") {
    // B_EV = $200, C_delay = $2
    CHECK(res.ev_benefit         == Approx(200.0).margin(0.1));
    CHECK(res.traffic_delay_cost == Approx(2.0).margin(0.1));
    // C_gen ≈ 0.40 (G1 cheap at 10 $/MWh × 0.04 MW × 1h)
    CHECK(res.gen_cost           == Approx(0.40).margin(0.1));
    // W = 200 − 2 − 0.40 = 197.60
    CHECK(res.social_welfare     == Approx(197.60).margin(0.2));
  }

  SECTION("generator dispatch populated") {
    REQUIRE_FALSE(res.gen_dispatch_mw.empty());
    // At arrival step 1, G1 should dispatch ≈ 0.04 MW to serve total EV load.
    bool g1_dispatched = false;
    for (const auto& step_map : res.gen_dispatch_mw) {
      for (const auto& [gidx, pmw] : step_map) {
        if (gidx == 1 && pmw > 0.01) { g1_dispatched = true; break; }
      }
    }
    CHECK(g1_dispatched == true);
  }
}

// =============================================================================
// Test D3: UserBenefitMax LP
// =============================================================================
// With equal station prices (0.03 $/kWh), the user-benefit objective prefers
// the near route and uses optimized charge power to satisfy the requested
// energy within the station capacity.
// Expected:
//   - proven_optimal = true
//   - h1 = 4, h2 = 0
//   - B_EV = $200, C_delay = $2.00
//   - User electricity cost = 4 × 10 kWh × 0.03 $/kWh = $1.20
// =============================================================================
TEST_CASE("FormD D3: UserBenefitMax LP — electricity cost in objective",
          "[ev_power_traffic][formulation_d][lp][user_benefit]") {

  const auto prob = make_joint_opt_d_problem();
  auto opts = make_d_options(/*opf=*/false);
  opts.mode = JointOptimizerMode::UserBenefitMax;

  const auto res = solve_joint_optimizer(prob, opts);

  SECTION("solver certificate") {
    CHECK(res.feasible       == true);
    CHECK(res.proven_optimal == true);
  }

  SECTION("route assignment — same as SocialWelfareMax with equal prices") {
    REQUIRE(res.route_flow.count(1));
    const auto& rf = res.route_flow.at(1);
    const double h1 = rf.count(1) ? rf.at(1) : 0.0;
    const double h2 = rf.count(2) ? rf.at(2) : 0.0;
    CHECK(h1 == Approx(4.0).margin(0.05));
    CHECK(h2 == Approx(0.0).margin(0.05));
  }

  SECTION("B_EV and delay computed correctly") {
    CHECK(res.ev_benefit         == Approx(200.0).margin(0.1));
    CHECK(res.traffic_delay_cost == Approx(2.0).margin(0.1));
  }
}

// =============================================================================
// Test D4: Welfare ≥ Formulation C (LP is globally optimal)
// =============================================================================
// Formulation D produces a single-shot LP optimal solution; its social welfare
// is at least as large as the converged iterative Formulation C result.
// Compare LP (D1) vs Formulation C on the same 2-bus network.
// =============================================================================
TEST_CASE("FormD D4: FormD social welfare >= FormC iterative (LP certified)",
          "[ev_power_traffic][formulation_d][cross_validation]") {

  const auto prob = make_joint_opt_d_problem();

  // Formulation D: SocialWelfareMax LP (no OPF — same assumptions as D1)
  const auto opts_d = make_d_options(/*opf=*/false);
  const auto res_d  = solve_joint_optimizer(prob, opts_d);
  REQUIRE(res_d.proven_optimal == true);

  // Formulation C: iterative CTM-DUE + DC-OPF
  CTMJointWelfareOptions opts_c;
  opts_c.max_iterations        = 20;
  opts_c.price_convergence_tol = 1e-4;
  opts_c.price_update_step     = 0.6;
  opts_c.use_dcopf_prices      = false;  // no OPF to match D1 (no OPF)
  opts_c.verbose               = false;

  opts_c.evpt_opts.num_steps              = 6;
  opts_c.evpt_opts.time_step_hr           = 1.0;
  opts_c.evpt_opts.default_charging_efficiency = 1.0;
  opts_c.evpt_opts.value_of_time_per_hr   = 1.0;
  opts_c.evpt_opts.station_energy_cost_weight = 1.0;
  opts_c.evpt_opts.default_station_price_per_kwh = 0.03;
  opts_c.evpt_opts.enforce_generation_capacity = false;
  opts_c.evpt_opts.update_system_charging_stations = false;

  opts_c.ctm_opts.dt_ctm_hr          = 0.05;
  opts_c.ctm_opts.n_cells_per_link   = 3;
  opts_c.ctm_opts.route_specific_cells = true;

  opts_c.due_opts.max_iterations  = 30;
  opts_c.due_opts.convergence_tol = 5e-3;
  opts_c.due_opts.logit_theta     = 0.0;

  EVPowerTrafficProblem prob_c = prob;
  const auto res_c = simulate_ev_power_traffic_ctm_joint(prob_c, opts_c);

  // Formulation D social welfare ≥ Formulation C (D is globally optimal LP)
  // Allow 5% tolerance for model differences (CTM vs free-flow travel time)
  CHECK(res_d.social_welfare >= res_c.social_welfare - 5.0);

  // Both should serve all 4 vehicles
  CHECK(res_d.total_served_vehicles >= 3.9);
}

// =============================================================================
// Test D5: MILP integer-flow mode (UserEquilibriumMILP)
// =============================================================================
// With 4 integer vehicles, the MILP keeps all vehicles on the cheaper near
// route because optimized charge power can deliver exactly 10 kWh/veh.
// =============================================================================
TEST_CASE("FormD D5: MILP integer route flows — proven optimal via B&C",
          "[ev_power_traffic][formulation_d][milp]") {

  // Use integer demand (4.0 vehicles → integer variables when mode = MILP)
  const auto prob = make_joint_opt_d_problem(4.0, 50.0, 10.0);
  auto opts = make_d_options(/*opf=*/false);
  opts.mode                  = JointOptimizerMode::UserEquilibriumMILP;
  opts.charge_discharge_binaries = false;  // only mode binaries for UE
  opts.mip_gap               = 1e-4;
  opts.time_limit_sec        = 20.0;
  opts.max_nodes             = 2048;

  const auto res = solve_joint_optimizer(prob, opts);

  SECTION("MILP certificate") {
    CHECK(res.feasible == true);
    // proven_optimal = true when MIP gap ≤ mip_gap tolerance
    CHECK(res.proven_optimal == true);
    CHECK(res.mip_gap        <= 1e-3);
  }

  SECTION("integer route assignment") {
    REQUIRE(res.route_flow.count(1));
    const auto& rf = res.route_flow.at(1);
    const double h1 = rf.count(1) ? rf.at(1) : 0.0;
    const double h2 = rf.count(2) ? rf.at(2) : 0.0;
    CHECK(h1 + h2 == Approx(4.0).margin(0.1));
    CHECK(h1      == Approx(4.0).margin(0.1));
    CHECK(h2      == Approx(0.0).margin(0.1));
    CHECK(res.total_served_vehicles == Approx(4.0).margin(0.01));
    CHECK(res.integrality_max_violation <= 1e-9);
  }
}

// =============================================================================
// Test D6: Price asymmetry shifts flow to cheaper station
// =============================================================================
// Setting station 102 price very high → UserBenefitMax routes all vehicles
// to station 101 (bus 1, cheap), with optimized charge power within capacity.
// =============================================================================
TEST_CASE("FormD D6: UserBenefitMax — high price at station 102 → route shift",
          "[ev_power_traffic][formulation_d][lp][price_sensitivity]") {

  auto prob = make_joint_opt_d_problem(4.0, 50.0, 10.0);

  // Set station 102 price very high: 5 $/kWh
  // Electricity cost per vehicle via route 2: 5 × 10 = $50/veh
  // Generalized cost: route 2 = 1.0h + 50 = 51.0 [$/veh]; route 1 = 0.5h + 0.03×10 = 0.8 [$/veh]
  // → LP wants to maximize h1 (route 1); exact p_ch variables make h1=4 feasible.
  StationPriceProfile pp2; pp2.station_id = 102;
  pp2.price_per_kwh.assign(6, 5.0);
  prob.station_prices.push_back(pp2);

  auto opts = make_d_options(/*opf=*/false);
  opts.mode = JointOptimizerMode::UserBenefitMax;

  const auto res = solve_joint_optimizer(prob, opts);

  REQUIRE(res.proven_optimal == true);
  REQUIRE(res.route_flow.count(1));

  const auto& rf = res.route_flow.at(1);
  const double h1 = rf.count(1) ? rf.at(1) : 0.0;

  // Route 1 should carry all demand.
  CHECK(h1 == Approx(4.0).margin(0.1));
  // All 4 served (u_d = 0 since WTP >> cost)
  CHECK(res.total_served_vehicles >= 3.9);
}

// =============================================================================
// Test D8: Time-varying road blockage profile
// =============================================================================
// Closing the first link on Route 1 at departure step 0 forces the LP to route
// all demand over Route 2.  This validates that Formulation D uses
// TrafficLink::availability_profile / capacity_profile_veh_per_hr rather than
// only the scalar capacity_veh_per_hr field.
// =============================================================================
TEST_CASE("FormD D8: road availability profile blocks closed route",
          "[ev_power_traffic][formulation_d][lp][road_fault]") {

  auto prob = make_joint_opt_d_problem(4.0, 50.0, 10.0);
  for (auto& link : prob.traffic.links) {
    if (link.index == 11) {
      link.availability_profile.assign(6, true);
      link.availability_profile[0] = false;
    }
  }

  const auto opts = make_d_options(/*opf=*/false);
  const auto res = solve_joint_optimizer(prob, opts);

  REQUIRE(res.proven_optimal == true);
  REQUIRE(res.route_flow.count(1));
  const auto& rf = res.route_flow.at(1);
  const double h1 = rf.count(1) ? rf.at(1) : 0.0;
  const double h2 = rf.count(2) ? rf.at(2) : 0.0;

  CHECK(h1 == Approx(0.0).margin(1e-6));
  CHECK(h2 == Approx(4.0).margin(1e-6));
  CHECK(res.total_served_vehicles == Approx(4.0).margin(1e-6));
  CHECK(res.total_requested_energy_kwh == Approx(40.0).margin(1e-6));
  CHECK(res.total_delivered_energy_kwh == Approx(40.0).margin(1e-6));
}

// =============================================================================
// Test D9: V2G discharge variables and charge/discharge mode binaries
// =============================================================================
// A high-price V2G-capable stop with initial battery surplus should discharge
// up to its power limit while respecting aggregate battery energy and
// δ_ch + δ_dis ≤ 1 mode constraints.
// =============================================================================
TEST_CASE("FormD D9: V2G discharge with charge/discharge binaries",
          "[ev_power_traffic][formulation_d][milp][v2g]") {

  auto prob = make_joint_opt_d_problem(1.0, 50.0, 0.0);
  REQUIRE_FALSE(prob.routes.empty());
  REQUIRE_FALSE(prob.routes.front().charging_stops.empty());
  auto& stop = prob.routes.front().charging_stops.front();
  stop.dwell_steps = 2;
  stop.v2g_capable = true;
  stop.max_discharge_kw_per_vehicle = 10.0;

  REQUIRE_FALSE(prob.demands.empty());
  prob.demands.front().initial_energy_kwh = 50.0;
  prob.demands.front().energy_min_kwh = 20.0;
  prob.demands.front().energy_max_kwh = 60.0;
  prob.demands.front().candidate_route_indices = {1};

  StationPriceProfile pp; pp.station_id = 101;
  pp.price_per_kwh.assign(6, 1.0);
  prob.station_prices.push_back(pp);

  auto opts = make_d_options(/*opf=*/false);
  opts.mode = JointOptimizerMode::UserBenefitMax;
  opts.allow_v2g = true;
  opts.charge_discharge_binaries = true;
  opts.default_charging_efficiency = 1.0;

  const auto res = solve_joint_optimizer(prob, opts);

  REQUIRE(res.feasible == true);
  REQUIRE(res.proven_optimal == true);
  REQUIRE(res.session_results.size() == 1u);

  const auto& sr = res.session_results.front();
  REQUIRE(sr.p_charge_kw.size() > 2u);
  REQUIRE(sr.p_discharge_kw.size() > 2u);
  REQUIRE(sr.energy_kwh.size() > 3u);

  CHECK(sr.p_charge_kw[1] == Approx(0.0).margin(1e-7));
  CHECK(sr.p_charge_kw[2] == Approx(0.0).margin(1e-7));
  CHECK(sr.p_discharge_kw[1] == Approx(10.0).margin(1e-6));
  CHECK(sr.p_discharge_kw[2] == Approx(10.0).margin(1e-6));
  CHECK(sr.energy_kwh[3] == Approx(30.0).margin(1e-6));
  CHECK(res.total_v2g_energy_kwh == Approx(20.0).margin(1e-6));
  CHECK(res.integrality_max_violation <= 1e-9);
}

// =============================================================================
// Test D7: Analysis printout (verbose mode)
// =============================================================================
TEST_CASE("FormD D7: verbose analysis printout — no assertion failures",
          "[ev_power_traffic][formulation_d][analysis]") {

  const auto prob = make_joint_opt_d_problem();
  auto opts = make_d_options(/*opf=*/true);
  opts.verbose = true;

  const auto res = solve_joint_optimizer(prob, opts);

  // Just verify the solve completes without throwing
  CHECK(res.feasible == true);
  CHECK(res.n_variables > 0);

  // Print summary for LaTeX Case Study 7
  std::printf("\n=== Formulation D (Case Study 7) Analysis ===\n");
  std::printf("  Solver:          %s\n",   res.solver_backend.c_str());
  std::printf("  Status:          %s\n",   res.solver_status.c_str());
  std::printf("  Proven optimal:  %s\n",   res.proven_optimal ? "YES" : "NO");
  std::printf("  Variables:       %d\n",   res.n_variables);
  std::printf("  Constraints:     %d\n",   res.n_constraints);
  std::printf("  Solve time:      %.3f s\n", res.solve_time_sec);
  std::printf("  Objective:       %.4f\n", res.objective);
  std::printf("  Social welfare:  %.4f\n", res.social_welfare);
  std::printf("  B_EV:            %.4f\n", res.ev_benefit);
  std::printf("  C_gen:           %.4f\n", res.gen_cost);
  std::printf("  C_delay:         %.4f\n", res.traffic_delay_cost);
  std::printf("  Served:          %.1f veh\n", res.total_served_vehicles);
  std::printf("  Unserved:        %.1f veh\n", res.total_unserved_vehicles);
  std::printf("  Energy req:      %.1f kWh\n", res.total_requested_energy_kwh);
  std::printf("  Sessions:        %zu\n",  res.sessions.size());

  if (!res.route_flow.empty()) {
    std::printf("  Route flows:\n");
    for (const auto& [did, rmap] : res.route_flow) {
      for (const auto& [rid, veh] : rmap) {
        std::printf("    demand %d, route %d: %.2f veh\n", did, rid, veh);
      }
    }
  }

  if (!res.gen_dispatch_mw.empty()) {
    std::printf("  Generator dispatch (step 1):\n");
    if (res.gen_dispatch_mw.size() > 1) {
      for (const auto& [gid, pmw] : res.gen_dispatch_mw[1]) {
        if (pmw > 0.001) std::printf("    G%d: %.4f MW\n", gid, pmw);
      }
    }
  }

  if (!res.warnings.empty()) {
    std::printf("  Warnings:\n");
    for (const auto& w : res.warnings)
      std::printf("    - %s\n", w.c_str());
  }
  std::printf("=== End FormD Analysis ===\n\n");
}

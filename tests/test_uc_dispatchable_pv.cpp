// Unit tests for the optional dispatchable-PV curtailment model in the
// time-series unit commitment (docs/sequential_production_simulation_rich_models.md §4.3).
//
// Must-take distributed sources (AC PVSystem, DC PVArrayDC, DC static gens) gain
// a curtailment claw-back variable in [0, available].  When off (default) they
// keep their must-take behaviour; when on, the optimiser may curtail them under
// oversupply / reverse-power conditions so the problem stays feasible.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Single AC bus: a small load, a zero-cost generator, and an oversized AC PV
// system whose must-take output exceeds the load.  Without curtailment the
// copper-plate balance is over-supplied (infeasible for non-negative generation).
HybridPowerSystem make_oversupply_system(double load_mw, double pv_mw) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  Load ld;
  ld.index = 1;
  ld.bus = 1;
  ld.p_mw = load_mw;
  ld.in_service = true;
  sys.ac.loads.push_back(ld);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.pmax_mw = 50.0;
  g.pmin_mw = 0.0;
  g.cost_c1 = 0.0;
  g.is_slack = true;
  sys.ac.generators.push_back(g);

  // Simple (non-curve) PV system: must-take output = p_mw.
  PVSystem pv;
  pv.index = 1;
  pv.bus = 1;
  pv.p_mw = pv_mw;
  pv.in_service = true;
  sys.ac.pv_systems.push_back(pv);

  return sys;
}

TimeSeriesData one_step() {
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("Dispatchable PV: oversupplied must-take PV is infeasible when off",
          "[time_series][dispatchable_pv]") {
  // PV must-take 80 MW > 30 MW load with no sink → over-supply.
  auto sys = make_oversupply_system(/*load*/ 30.0, /*pv*/ 80.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_dispatchable_pv = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  CHECK_FALSE(sched.feasible);
}

TEST_CASE("Dispatchable PV: curtailment restores feasibility under oversupply",
          "[time_series][dispatchable_pv]") {
  auto sys = make_oversupply_system(/*load*/ 30.0, /*pv*/ 80.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_dispatchable_pv = true;
  opts.pv_curtail_penalty = 50.0;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // PV must be curtailed by at least 50 MW (80 available − 30 load), at a
  // penalty of 50 $/MWh → objective ≥ 2500.
  CHECK(sched.total_cost >= 2500.0 - 1e-6);
}

TEST_CASE("Dispatchable PV: no curtailment when supply and load balance",
          "[time_series][dispatchable_pv]") {
  // PV 20 MW ≤ 30 MW load: the generator covers the 10 MW gap, no curtailment.
  auto sys = make_oversupply_system(/*load*/ 30.0, /*pv*/ 20.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_dispatchable_pv = true;
  opts.pv_curtail_penalty = 50.0;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Generator is free (cost 0) and no curtailment is needed → objective 0.
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Dispatchable PV: operating cost changes economic dispatch",
          "[time_series][dispatchable_pv][cost]") {
  auto sys = make_oversupply_system(/*load*/ 30.0, /*pv*/ 20.0);
  sys.ac.generators.front().cost_c1 = 10.0;
  sys.ac.pv_systems.front().cost_c1 = 100.0;
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_dispatchable_pv = true;
  opts.pv_curtail_penalty = 0.0;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  REQUIRE(sched.gen_dispatch.size() == 1);
  REQUIRE(sched.ac_pv_dispatch.size() == 1);
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(30.0, 1e-2));
  CHECK_THAT(sched.ac_pv_dispatch[0][0], Catch::Matchers::WithinAbs(0.0, 1e-2));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(300.0, 1e-2));
}

TEST_CASE("Dispatchable PV: AC static generators supply as dispatchable sources",
          "[time_series][dispatchable_pv]") {
  // Expensive generator (100 $/MWh), 50 MW load, plus a 30 MW AC static gen.
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  ACBus bus; bus.index = 1; bus.bus_type = BusType::SLACK; bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);
  Load ld; ld.index = 1; ld.bus = 1; ld.p_mw = 50.0; ld.in_service = true;
  sys.ac.loads.push_back(ld);
  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 200.0; g.cost_c1 = 100.0;
  g.is_slack = true; sys.ac.generators.push_back(g);
  StaticGenerator sg; sg.index = 1; sg.bus = 1; sg.p_mw = 30.0; sg.scaling = 1.0;
  sg.in_service = true; sys.ac.static_generators.push_back(sg);
  TimeSeriesData ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;

  SECTION("off -> static gen not modelled, generator serves all load") {
    opts.enable_dispatchable_pv = false;
    const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
    REQUIRE(sched.feasible);
    CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(50.0, 1e-2));
    CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 1e-2));
  }
  SECTION("on -> static gen supplies 30 MW, generator serves 20 MW") {
    opts.enable_dispatchable_pv = true;
    opts.pv_curtail_penalty = 50.0;
    const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
    REQUIRE(sched.feasible);
    REQUIRE(sched.ac_sgen_dispatch.size() == 1);
    CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(20.0, 1e-2));
    CHECK_THAT(sched.ac_sgen_dispatch[0][0], Catch::Matchers::WithinAbs(30.0, 1e-2));
    // Reported cost is clean (obj_offset cancels the utilisation reward): 100*20.
    CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(2000.0, 1e-2));
  }
}

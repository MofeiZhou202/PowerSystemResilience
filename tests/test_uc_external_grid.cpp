// Unit tests for the optional external-grid model in the time-series unit
// commitment (docs/sequential_production_simulation_rich_models.md §4.1).
//
// External grids participate by default, matching power-flow and dynamic-source
// semantics. They may still be explicitly disabled for isolated-system studies.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Minimal single-bus system: one load, one (expensive) generator, one (cheap)
// external grid.  Copper-plate balance is sufficient for these economics tests.
HybridPowerSystem make_grid_test_system(double gen_cost_c1,
                                         double ext_cost_c1,
                                         double load_mw,
                                         double gen_pmax_mw,
                                         bool gen_in_service = true) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.base_kv = 110.0;
  bus.in_service = true;
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
  g.pmax_mw = gen_pmax_mw;
  g.pmin_mw = 0.0;
  g.cost_c1 = gen_cost_c1;
  g.is_slack = true;
  g.in_service = gen_in_service;
  sys.ac.generators.push_back(g);

  ExternalGrid xg;
  xg.index = 1;
  xg.bus = 1;
  xg.cost_c1 = ext_cost_c1;
  xg.price_profile_id = -1;
  xg.in_service = true;
  sys.ac.external_grids.push_back(xg);

  return sys;
}

TimeSeriesData make_single_step() {
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("External grid: explicit disable leaves dispatch to generators",
          "[time_series][external_grid]") {
  // Expensive generator (100 $/MWh), cheap grid (10 $/MWh), 50 MW load.
  auto sys = make_grid_test_system(/*gen*/ 100.0, /*ext*/ 10.0,
                                   /*load*/ 50.0, /*pmax*/ 100.0);
  auto ts = make_single_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_external_grid = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  CHECK(sched.external_grid_dispatch.empty());
  // Generator must serve the whole load (no grid available).
  REQUIRE(sched.gen_dispatch.size() == 1);
  CHECK_THAT(sched.gen_dispatch[0][0],
             Catch::Matchers::WithinAbs(50.0, 1e-3));
  // Objective = 100 $/MWh * 50 MW * 1 h.
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 1e-3));
}

TEST_CASE("External grid: enabling a cheaper tie displaces the generator",
          "[time_series][external_grid]") {
  auto sys = make_grid_test_system(/*gen*/ 100.0, /*ext*/ 10.0,
                                   /*load*/ 50.0, /*pmax*/ 100.0);
  auto ts = make_single_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_external_grid = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  REQUIRE(sched.external_grid_dispatch.size() == 1);
  CHECK_THAT(sched.external_grid_dispatch[0][0],
             Catch::Matchers::WithinAbs(50.0, 1e-2));
  // The cheap grid should supply the load; the generator backs off to ~0.
  CHECK_THAT(sched.gen_dispatch[0][0],
             Catch::Matchers::WithinAbs(0.0, 1e-2));
  // Objective = 10 $/MWh * 50 MW * 1 h — an order of magnitude below gen-only.
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(500.0, 1e-3));
}

TEST_CASE("External grid: supplies load that local generation cannot",
          "[time_series][external_grid]") {
  // Generator out of service; only the grid can serve the 50 MW load.
  auto sys = make_grid_test_system(/*gen*/ 100.0, /*ext*/ 10.0,
                                   /*load*/ 50.0, /*pmax*/ 100.0,
                                   /*gen_in_service*/ false);
  auto ts = make_single_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;

  SECTION("grid enabled by default -> feasible, grid covers the load") {
    const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
    REQUIRE(sched.feasible);
    REQUIRE(sched.external_grid_dispatch.size() == 1);
    CHECK_THAT(sched.external_grid_dispatch[0][0],
               Catch::Matchers::WithinAbs(50.0, 1e-2));
  }

  SECTION("grid disabled -> infeasible") {
    opts.enable_external_grid = false;
    const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
    CHECK_FALSE(sched.feasible);
  }
  SECTION("grid enabled -> feasible, grid covers the load") {
    opts.enable_external_grid = true;
    const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
    REQUIRE(sched.feasible);
    CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(500.0, 1e-3));
  }
}

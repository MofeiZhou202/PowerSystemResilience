// Unit tests for the optional microgrid PCC-exchange + islanding model in the
// time-series unit commitment (docs/sequential_production_simulation_rich_models.md §4.6).
//
// Each in-service Microgrid contributes a PCC net-exchange variable
// (+export / −import, bounded by p_export_max / p_import_max) gated by a binary
// connection indicator o (1 = grid-connected).  Disabled by default; when on,
// the optimiser may exchange power with the microgrid, and may island (o = 0,
// zero exchange) at a configurable penalty.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Single AC bus: one expensive generator, a load, and a microgrid at the bus.
HybridPowerSystem make_mg_system(double load_mw, double export_max,
                                 bool islanding_capable) {
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
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.cost_c1 = 100.0;
  g.is_slack = true;
  sys.ac.generators.push_back(g);

  Microgrid mg;
  mg.index = 1;
  mg.pcc_bus = 1;
  mg.p_export_max_mw = export_max;
  mg.p_import_max_mw = export_max;
  mg.islanding_capability = islanding_capable;
  mg.in_service = true;
  sys.microgrids.push_back(mg);

  return sys;
}

TimeSeriesData one_step() {
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("Microgrid: disabled by default does not exchange power",
          "[time_series][microgrid]") {
  auto sys = make_mg_system(/*load*/ 50.0, /*export_max*/ 40.0,
                            /*islanding*/ true);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_microgrid = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Generator serves the full 50 MW; objective = 100 * 50.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(50.0, 1e-3));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 1e-3));
}

TEST_CASE("Microgrid: enabled PCC export displaces expensive generation",
          "[time_series][microgrid]") {
  auto sys = make_mg_system(/*load*/ 50.0, /*export_max*/ 40.0,
                            /*islanding*/ true);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_microgrid = true;
  opts.w_microgrid_island = 100.0;  // keeps the microgrid connected

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Microgrid exports its full 40 MW; the generator covers the remaining 10 MW.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(10.0, 1e-2));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(1000.0, 1e-2));
}

TEST_CASE("Microgrid: a non-islanding microgrid stays connected and exchanges",
          "[time_series][microgrid]") {
  auto sys = make_mg_system(/*load*/ 50.0, /*export_max*/ 40.0,
                            /*islanding*/ false);
  auto ts = one_step();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_microgrid = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Pinned grid-connected (o ≡ 1) → still exports 40 MW, generator serves 10 MW.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(10.0, 1e-2));
}

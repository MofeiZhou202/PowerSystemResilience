// Unit tests for the optional VPP aggregate-dispatch model in the time-series
// unit commitment (docs/sequential_production_simulation_rich_models.md §4.5).
//
// Each in-service VirtualPowerPlant gets a net-output variable at its PCC bus
// bounded by [pmin, pmax] with ramp limits, plus an aggregate energy state
// bounded by e_storage_sum_mwh.  A generation VPP (e_storage_sum = 0) is limited
// only by its power bounds; a storage-backed VPP is additionally energy-limited.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

HybridPowerSystem make_vpp_system(double vpp_pmax, double e_storage_sum) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus; bus.index = 1; bus.bus_type = BusType::SLACK; bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  Load ld; ld.index = 1; ld.bus = 1; ld.p_mw = 50.0; ld.in_service = true;
  sys.ac.loads.push_back(ld);

  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 200.0; g.cost_c1 = 100.0;
  g.is_slack = true; sys.ac.generators.push_back(g);

  VirtualPowerPlant vpp; vpp.index = 1; vpp.pcc_bus = 1;
  vpp.pmin_mw = 0.0; vpp.pmax_mw = vpp_pmax;
  vpp.e_storage_sum_mwh = e_storage_sum;
  vpp.in_service = true;
  sys.vpps.push_back(vpp);

  return sys;
}

double total_gen(const UCSchedule& s) {
  double g = 0.0;
  if (!s.gen_dispatch.empty())
    for (double p : s.gen_dispatch[0]) g += p;
  return g;
}

TimeSeriesData two_steps() {
  TimeSeriesData ts; ts.num_steps = 2; ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("VPP: disabled by default leaves dispatch to generators",
          "[time_series][vpp]") {
  auto sys = make_vpp_system(/*pmax*/ 30.0, /*E*/ 0.0);
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_vpp = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Generator serves both 50 MW steps → 100 MWh; cost 100·100.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(100.0, 1e-2));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(10000.0, 1e-2));
}

TEST_CASE("VPP: a generation VPP exports up to its power bound each step",
          "[time_series][vpp]") {
  auto sys = make_vpp_system(/*pmax*/ 30.0, /*E*/ 0.0);  // non-binding energy
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_vpp = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // VPP exports 30 MW each step (free) → generator serves 20 MW each → 40 MWh.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(40.0, 1e-1));
}

TEST_CASE("VPP: the storage envelope energy-limits the aggregate output",
          "[time_series][vpp]") {
  // e_storage_sum = 10 MWh (starts half full at 5) → total net export ≤ 5 MWh.
  auto sys = make_vpp_system(/*pmax*/ 30.0, /*E*/ 10.0);
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_vpp = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // The VPP can supply at most 5 MWh total, so the generator serves ≥ 95 MWh.
  CHECK(total_gen(sched) >= 95.0 - 1e-1);
}

// Unit tests for the optional demand-response model on FlexibleLoad in the
// time-series unit commitment (docs/sequential_production_simulation_rich_models.md §4.2).
//
// Served demand = baseline + up − down, with up,down ≥ 0 bounded by
// flex_up_mw / flex_down_mw (× availability) and a discomfort penalty
// w_demand_response·(up+down).  Disabled by default; shiftable mode adds a
// per-load energy-neutrality constraint over the horizon.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <numeric>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Single-bus system: one expensive generator and one flexible load.
HybridPowerSystem make_dr_system(double load_mw, double flex_down, double flex_up) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.cost_c1 = 100.0;  // $/MWh
  g.is_slack = true;
  sys.ac.generators.push_back(g);

  FlexibleLoad fl;
  fl.index = 1;
  fl.bus = 1;
  fl.p_mw = load_mw;
  fl.flex_down_mw = flex_down;
  fl.flex_up_mw = flex_up;
  fl.availability_pct = 100.0;
  fl.in_service = true;
  sys.ac.flexible_loads.push_back(fl);

  return sys;
}

TimeSeriesData make_steps(int n) {
  TimeSeriesData ts;
  ts.num_steps = n;
  ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("Demand response: off by default keeps the flexible load at baseline",
          "[time_series][demand_response]") {
  auto sys = make_dr_system(/*load*/ 50.0, /*down*/ 20.0, /*up*/ 10.0);
  auto ts = make_steps(1);

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_demand_response = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Generator serves the full baseline 50 MW; objective = 100 * 50.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(50.0, 1e-3));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 1e-3));
}

TEST_CASE("Demand response: zero penalty lets the optimizer shed flexible demand",
          "[time_series][demand_response]") {
  auto sys = make_dr_system(/*load*/ 50.0, /*down*/ 20.0, /*up*/ 10.0);
  auto ts = make_steps(1);

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_demand_response = true;
  opts.dr_shiftable = false;       // adjustable
  opts.w_demand_response = 0.0;    // no discomfort cost

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Demand drops by the full 20 MW headroom → generator serves 30 MW, cost 3000.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(30.0, 1e-2));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(3000.0, 1e-2));
}

TEST_CASE("Demand response: a high discomfort penalty suppresses shifting",
          "[time_series][demand_response]") {
  auto sys = make_dr_system(/*load*/ 50.0, /*down*/ 20.0, /*up*/ 10.0);
  auto ts = make_steps(1);

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_demand_response = true;
  opts.w_demand_response = 1000.0;  // far above the 100 $/MWh generation saving

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Penalty dominates → no demand reduction → baseline 50 MW, cost 5000.
  CHECK_THAT(sched.gen_dispatch[0][0], Catch::Matchers::WithinAbs(50.0, 1e-2));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 1e-2));
}

TEST_CASE("Demand response: shiftable mode conserves energy across the horizon",
          "[time_series][demand_response]") {
  // Two steps with a cheap/expensive price split via a time-varying generator
  // cost would be ideal, but here we simply assert energy neutrality: with the
  // shiftable constraint, total served demand equals total baseline demand.
  auto sys = make_dr_system(/*load*/ 50.0, /*down*/ 20.0, /*up*/ 20.0);
  auto ts = make_steps(2);

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_demand_response = true;
  opts.dr_shiftable = true;
  opts.w_demand_response = 0.0;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Total generation over the horizon must equal total baseline load (2 × 50),
  // because shiftable DR can move energy between steps but not remove it.
  const double total_gen =
      sched.gen_dispatch[0][0] + sched.gen_dispatch[0][1];
  CHECK_THAT(total_gen, Catch::Matchers::WithinAbs(100.0, 1e-2));
}

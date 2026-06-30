// Unit tests for the optional storage cycle-aging (degradation) cost in the
// time-series unit commitment
// (docs/sequential_production_simulation_rich_models.md §4.7).
//
// Throughput is priced at replacement_cost / (2·max_cycles·E) per MWh moved via
// an absolute-value auxiliary g ≥ |p_ess|.  A daily_cycle_limit caps throughput
// at 2·limit·E over the horizon.  Disabled by default.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Two steps: cheap then expensive generation, plus a battery.  Without a
// degradation cost the battery arbitrages (charges cheap, discharges dear);
// a high degradation cost suppresses that cycling.
HybridPowerSystem make_arbitrage_system(double replacement_cost,
                                        double daily_cycle_limit) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus bus; bus.index = 1; bus.bus_type = BusType::SLACK; bus.base_kv = 110.0;
  sys.ac.buses.push_back(bus);

  Load ld; ld.index = 1; ld.bus = 1; ld.p_mw = 50.0; ld.in_service = true;
  sys.ac.loads.push_back(ld);

  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 200.0; g.cost_c1 = 50.0;
  g.is_slack = true; sys.ac.generators.push_back(g);

  Storage s; s.index = 1; s.bus = 1;
  s.pmax_mw = 20.0; s.pmin_mw = -20.0;     // ±20 MW
  s.e_rated_mwh = 40.0; s.soc_init = 0.5; s.soc_min = 0.1; s.soc_max = 0.9;
  s.eta_charge = 1.0; s.eta_discharge = 1.0;
  s.max_cycles = 5000; s.replacement_cost = replacement_cost;
  s.daily_cycle_limit = daily_cycle_limit;
  s.in_service = true;
  sys.ac.storage.push_back(s);

  return sys;
}

TimeSeriesData two_steps() {
  TimeSeriesData ts; ts.num_steps = 2; ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("Storage degradation: disabled by default adds no throughput cost",
          "[time_series][storage_degradation]") {
  auto sys = make_arbitrage_system(/*replacement*/ 1.0e6, /*cap*/ 0.0);
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_storage_degradation = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // No degradation term in the objective.
  CHECK(sched.gen_dispatch.size() == 1);
}

TEST_CASE("Storage degradation: a high cost suppresses storage cycling",
          "[time_series][storage_degradation]") {
  // Very expensive replacement → cycling the battery costs more than it saves.
  auto sys = make_arbitrage_system(/*replacement*/ 1.0e9, /*cap*/ 0.0);
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_storage_degradation = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // The battery should not move power (|p_ess| ≈ 0) under the huge aging cost.
  double throughput = 0.0;
  for (double p : sched.ess_dispatch[0]) throughput += std::abs(p);
  CHECK(throughput < 1e-3);
}

TEST_CASE("Storage degradation: daily cycle cap bounds throughput",
          "[time_series][storage_degradation]") {
  // Cheap replacement (low aging cost) but a tight cycle cap of 0.05 cycles:
  // throughput ≤ 2·0.05·40 = 4 MWh over the horizon.
  auto sys = make_arbitrage_system(/*replacement*/ 1.0, /*cap*/ 0.05);
  auto ts = two_steps();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_storage_degradation = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  double throughput = 0.0;  // Σ|p|·dt
  for (double p : sched.ess_dispatch[0]) throughput += std::abs(p) * ts.step_duration_hr;
  CHECK(throughput <= 4.0 + 1e-3);
}

// Unit tests for the optional mobile-storage model in the time-series unit
// commitment (docs/sequential_production_simulation_rich_models.md §4.4).
//
// With enable_mobile_storage, each in-service MobileStorage gets a signed power
// variable (+discharge) and an energy state in [soc_min, soc_max].  Its
// connection bus follows an exogenous schedule from departure_time /
// arrival_time / status: it injects at its current bus before departure, is
// disconnected (power pinned to 0) while in transit, and injects at target_bus
// after arrival.  The relocation timing is taken as given input (no per-bus
// assignment binaries).
//
// The tests use a copper-plate two-step horizon with a peaky load (10 MW then
// 50 MW) served by a single generator.  A stationary battery can shift its full
// energy into the expensive peak step; a battery that is in transit during the
// peak can only help the off-peak step, so more generation is required.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// One bus, one generator, one peaky load.  `relocate` makes the mobile battery
// depart after the first step so it is in transit (disconnected) during the
// second (peak) step.
HybridPowerSystem make_mobile_system(bool relocate) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
  sys.ac.buses.push_back(b1);

  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 100.0; g.pmin_mw = 0.0;
  g.cost_c1 = 100.0; g.is_slack = true; sys.ac.generators.push_back(g);

  // Base load 50 MW scaled by profile 5 → 10 MW (step 0), 50 MW (step 1).
  Load ld; ld.index = 1; ld.bus = 1; ld.p_mw = 50.0; ld.profile_id = 5;
  ld.in_service = true; sys.ac.loads.push_back(ld);

  MobileStorage ms; ms.index = 1; ms.bus = 1; ms.in_service = true;
  ms.p_rated_mw = 40.0; ms.pmax_mw = 40.0; ms.pmin_mw = -40.0;
  ms.e_rated_mwh = 40.0; ms.soc_init = 1.0; ms.soc_min = 0.0; ms.soc_max = 1.0;
  ms.eta_charge = 1.0; ms.eta_discharge = 1.0;
  if (relocate) {
    // Depart at hour 1, arrive at hour 2 → in transit during step 1 (the peak).
    ms.status = MobileStorageStatus::InTransit;
    ms.departure_time = 1.0; ms.arrival_time = 2.0; ms.target_bus = 2;
  } else {
    ms.status = MobileStorageStatus::Stationary;
  }
  sys.mobile_storage.push_back(ms);

  return sys;
}

double total_gen(const UCSchedule& s) {
  double g = 0.0;
  if (!s.gen_dispatch.empty())
    for (double p : s.gen_dispatch[0]) g += p;
  return g;
}

// Two steps, dt = 1 h, load profile [0.2, 1.0] (→ 10 MW then 50 MW).
TimeSeriesData two_step_peaky() {
  TimeSeriesData ts; ts.num_steps = 2; ts.step_duration_hr = 1.0;
  TimeSeriesProfile p; p.id = 5; p.name = "peaky"; p.values = {0.2, 1.0};
  ts.profiles.push_back(p);
  return ts;
}

}  // namespace

TEST_CASE("Mobile storage: disabled leaves the generator to serve the peak",
          "[time_series][mobile_storage]") {
  auto sys = make_mobile_system(/*relocate*/ false);
  auto ts = two_step_peaky();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_mobile_storage = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Generator serves the whole 10 + 50 = 60 MWh → cost 100·60.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(60.0, 1e-1));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(6000.0, 1e-2));
}

TEST_CASE("Mobile storage: a stationary battery shaves the peak",
          "[time_series][mobile_storage]") {
  auto sys = make_mobile_system(/*relocate*/ false);
  auto ts = two_step_peaky();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_mobile_storage = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // The full 40 MWh discharges into the peak → generation drops to 60 − 40 = 20.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(20.0, 5e-1));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(2000.0, 2e-2));
}

TEST_CASE("Mobile storage: a battery in transit cannot serve the peak",
          "[time_series][mobile_storage]") {
  auto sys = make_mobile_system(/*relocate*/ true);
  auto ts = two_step_peaky();

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_mobile_storage = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Connected only in the 10 MW off-peak step, the battery offsets at most 10 MW
  // there; the generator still serves the full 50 MW peak → 50 MWh total.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(50.0, 5e-1));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(5000.0, 2e-2));
  // Strictly costlier than the stationary battery but cheaper than no battery.
  CHECK(sched.total_cost > 2000.0 + 1.0);
  CHECK(sched.total_cost < 6000.0 - 1.0);
}

// ── Co-optimised relocation (mobile_storage_corelocate) ─────────────────────
// A nodal two-bus system where bus 2 holds a load behind a thermally limited
// line and has only an expensive local generator.  A pre-charged battery starts
// at bus 1; with co-optimised relocation the optimiser decides to move it to
// bus 2 (origin → transit → target) so it can serve the congested load locally.
HybridPowerSystem make_relocation_system() {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0;
  sys.ac.buses.push_back(b1);
  sys.ac.buses.push_back(b2);

  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
  br.x_pu = 0.1; br.rate_a_mva = 5.0; br.in_service = true;  // tight limit
  sys.ac.branches.push_back(br);

  Generator gc; gc.index = 1; gc.bus = 1; gc.pmax_mw = 100.0; gc.pmin_mw = 0.0;
  gc.cost_c1 = 10.0; gc.is_slack = true; sys.ac.generators.push_back(gc);
  Generator ge; ge.index = 2; ge.bus = 2; ge.pmax_mw = 100.0; ge.pmin_mw = 0.0;
  ge.cost_c1 = 100.0; sys.ac.generators.push_back(ge);

  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30.0; ld.in_service = true;
  sys.ac.loads.push_back(ld);

  MobileStorage ms; ms.index = 1; ms.bus = 1; ms.target_bus = 2; ms.in_service = true;
  ms.p_rated_mw = 30.0; ms.pmax_mw = 30.0; ms.pmin_mw = -30.0;
  ms.e_rated_mwh = 90.0; ms.soc_init = 1.0; ms.soc_min = 0.0; ms.soc_max = 1.0;
  ms.eta_charge = 1.0; ms.eta_discharge = 1.0;
  sys.mobile_storage.push_back(ms);

  return sys;
}

double total_cost_disabled_relocation() {
  auto sys = make_relocation_system();
  TimeSeriesData ts; ts.num_steps = 4; ts.step_duration_hr = 1.0;
  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_network_constraints = true;
  opts.enable_mobile_storage = false;
  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  return sched.total_cost;
}

TEST_CASE("Mobile storage: co-optimised relocation serves a congested bus",
          "[time_series][mobile_storage]") {
  auto sys = make_relocation_system();
  TimeSeriesData ts; ts.num_steps = 4; ts.step_duration_hr = 1.0;

  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::SCIP;
  opts.enable_network_constraints = true;
  opts.enable_mobile_storage = true;
  opts.mobile_storage_corelocate = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);

  // Without the battery the expensive bus-2 generator covers the 25 MW that the
  // line cannot carry every step (4 × $2550 = $10200).  Relocating the battery
  // (origin → transit → target) lets it serve bus 2 locally once it arrives, so
  // the total is markedly lower.
  const double baseline = total_cost_disabled_relocation();
  CHECK_THAT(baseline, Catch::Matchers::WithinRel(10200.0, 5e-2));
  CHECK(sched.total_cost < baseline - 1000.0);
}

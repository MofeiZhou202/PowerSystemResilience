// tests/test_hacdcpf.cpp
//
// Smoke tests for hacdcpf: model construction, power flow, OPF, and analysis.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/power_models/hybrid_opf_model_builder.hpp"

using Catch::Matchers::WithinAbs;

// ═══════════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════════

static hacdcpf::HybridPowerSystem make_simple_ac_system() {
  using namespace hacdcpf;

  HybridPowerSystem sys;
  sys.name     = "test_2bus";
  sys.base_mva = 100.0;

  // Bus 1: slack
  ACBus b1;
  b1.index    = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu    = 1.05;
  b1.va_deg   = 0.0;
  b1.pd_mw    = 0.0;
  b1.qd_mvar  = 0.0;
  b1.in_service = true;

  // Bus 2: PQ load bus
  ACBus b2;
  b2.index    = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu    = 1.0;
  b2.va_deg   = 0.0;
  b2.pd_mw    = 50.0;
  b2.qd_mvar  = 20.0;
  b2.in_service = true;

  sys.ac.buses = {b1, b2};

  // Branch 1→2
  ACBranch br;
  br.from_bus   = 1;
  br.to_bus     = 2;
  br.r_pu       = 0.01;
  br.x_pu       = 0.05;
  br.b_pu       = 0.0;
  br.tap        = 1.0;
  br.shift_deg  = 0.0;
  br.in_service = true;

  sys.ac.branches = {br};

  // Slack generator at bus 1
  Generator g;
  g.bus         = 1;
  g.is_slack    = true;
  g.pg_mw       = 0.0;
  g.qg_mvar     = 0.0;
  g.pmax_mw     = 200.0;
  g.pmin_mw     = 0.0;
  g.qmax_mvar   = 100.0;
  g.qmin_mvar   = -100.0;
  g.vg_pu       = 1.05;
  g.in_service  = true;

  sys.ac.generators = {g};

  return sys;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: model construction
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("HybridPowerSystem: simple 2-bus AC system construction", "[model]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();

  REQUIRE(sys.ac.buses.size() == 2);
  REQUIRE(sys.ac.branches.size() == 1);
  REQUIRE(sys.ac.generators.size() == 1);
  REQUIRE(sys.dc.buses.empty());
  REQUIRE(sys.vsc_converters.empty());
  REQUIRE_THAT(sys.base_mva, WithinAbs(100.0, 1e-9));
}

TEST_CASE("HybridPowerSystem: IEEE14 AC/DC case builder", "[model][case_builder]") {
  auto sys = hacdcpf::io::build_ieee14_acdc();

  CHECK(sys.ac.buses.size() >= 14);
  CHECK_FALSE(sys.ac.branches.empty());
  CHECK_FALSE(sys.ac.generators.empty());
}

TEST_CASE("HybridPowerSystem: IEEE24 3-area AC/DC case builder", "[model][case_builder]") {
  auto sys = hacdcpf::io::build_ieee24_3area_acdc();

  CHECK(sys.ac.buses.size() >= 24);
  CHECK_FALSE(sys.ac.branches.empty());
  CHECK_FALSE(sys.ac.generators.empty());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: AC power flow
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("AC power flow: 2-bus converges", "[power_flow][ac]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  PowerFlowOptions opt;
  opt.max_iter = 50;
  opt.tol      = 1e-8;

  auto result = solve_power_flow(sys, opt);

  REQUIRE(result.converged);
  REQUIRE(result.iterations > 0);
  REQUIRE(result.iterations <= opt.max_iter);
}

TEST_CASE("AC power flow: IEEE14 AC/DC converges", "[power_flow][ac]") {
  using namespace hacdcpf;

  auto sys    = hacdcpf::io::build_ieee14_acdc();
  auto result = solve_power_flow(sys);

  CHECK(result.converged);
  CHECK(result.iterations > 0);
}

TEST_CASE("AC power flow: 2-bus bus voltages are finite", "[power_flow][ac]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_power_flow(sys);

  REQUIRE(result.converged);
  for (const auto& bv : result.vm) {
    REQUIRE(std::isfinite(bv));
    REQUIRE(bv > 0.0);
  }
  for (const auto& ba : result.va) {
    REQUIRE(std::isfinite(ba));
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: DC power flow
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("DC power flow: 2-bus system (AC-only), result object valid", "[power_flow][dc]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_dc_power_flow(sys);

  // For a pure AC system the DC result's vdc vector may be empty (no DC buses).
  // The call itself should not throw.
  (void)result;
  SUCCEED();
}

TEST_CASE("DC power flow: IEEE14 AC/DC", "[power_flow][dc]") {
  using namespace hacdcpf;

  auto sys    = hacdcpf::io::build_ieee14_acdc();
  auto result = solve_dc_power_flow(sys);

  CHECK(result.converged);
}

TEST_CASE("DC power flow: isolated DC bus does not break the solve",
          "[power_flow][dc][isolated]") {
  using namespace hacdcpf;

  // Start from a known-good hybrid case and append an unconnected DC bus that
  // is explicitly typed DC_ISOLATED.  Without isolated-bus handling this bus
  // would contribute a zero row/column to the DC system and make the Jacobian
  // singular; the solve must still converge.
  auto sys = hacdcpf::io::build_ieee14_acdc();
  int max_idx = 0;
  for (const auto& b : sys.dc.buses) max_idx = std::max(max_idx, b.index);

  DCBus iso;
  iso.index      = max_idx + 1;
  iso.bus_type   = DCBusType::DC_ISOLATED;
  iso.vm_pu      = 1.0;
  iso.pd_mw      = 2.0;       // a load that is intentionally stranded
  iso.in_service = true;
  iso.name       = "DC_ISOLATED";
  sys.dc.buses.push_back(iso);

  auto result = solve_dc_power_flow(sys);
  CHECK(result.converged);
  for (double v : result.vdc) CHECK(std::isfinite(v));
}

TEST_CASE("Hybrid OPF data: isolated DC bus is excluded from the model",
          "[opf][dc][isolated]") {
  using namespace hacdcpf;

  // One DC_V reference, one DC_P load bus, and one DC_ISOLATED bus.  The
  // isolated bus (and any branch touching it) must not enter the OPF model.
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;

  ACBus a1; a1.index = 1; a1.bus_type = BusType::SLACK; a1.base_kv = 110.0; a1.in_service = true;
  sys.ac.buses = {a1};

  DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V;        d1.vm_pu = 1.0; d1.in_service = true;
  DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P;        d2.pd_mw = 1.0; d2.in_service = true;
  DCBus d3; d3.index = 3; d3.bus_type = DCBusType::DC_ISOLATED; d3.pd_mw = 1.0; d3.in_service = true;
  sys.dc.buses = {d1, d2, d3};

  DCBranch b12; b12.index = 1; b12.from_bus = 1; b12.to_bus = 2; b12.r_pu = 0.01; b12.in_service = true;
  DCBranch b23; b23.index = 2; b23.from_bus = 2; b23.to_bus = 3; b23.r_pu = 0.01; b23.in_service = true;
  sys.dc.branches = {b12, b23};

  const auto data = power_models::to_acdcopf_data(sys);

  // Only the two energized DC buses survive; the isolated one is dropped.
  CHECK(data.dc_buses.size() == 2);
  // The branch 2->3 that touches the isolated bus is dropped; 1->2 remains.
  CHECK(data.dc_branches.size() == 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: FDPF solver
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("FDPF: 2-bus converges", "[power_flow][fdpf]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_power_flow_fdpf(sys);

  // FDPF should converge for this well-conditioned case
  CHECK(result.converged);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Adaptive solver
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Adaptive solver: 2-bus completes", "[power_flow][adaptive]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_power_flow_adaptive(sys);

  REQUIRE(result.converged);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Island detection
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Islanded solver: 2-bus (connected), completes", "[power_flow][island]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_power_flow_islanded(sys);

  CHECK(result.converged);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Linearized DC power flow
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Linearized DC PF: 2-bus valid", "[power_flow][linear]") {
  using namespace hacdcpf;

  auto sys    = make_simple_ac_system();
  auto result = solve_ac_dc_power_flow(sys);

  // Linearized DC should always produce a solution
  REQUIRE_FALSE(result.va.empty());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Hybrid (AC/DC) power flow
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Hybrid power flow: IEEE14 AC/DC converges", "[power_flow][hybrid]") {
  using namespace hacdcpf;

  auto sys    = hacdcpf::io::build_ieee14_acdc();
  auto result = solve_power_flow(sys);

  CHECK(result.converged);
  CHECK_FALSE(result.vm.empty());
  CHECK_FALSE(result.va.empty());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Solver handle (stateful solver)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Solver handle: create, solve, destroy", "[power_flow][handle]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  auto* h  = create_solver_handle(sys);
  REQUIRE(h != nullptr);

  auto result = solve_handle(h);
  CHECK(result.converged);

  destroy_solver_handle(h);
}

TEST_CASE("Solver handle: reset and re-solve", "[power_flow][handle]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  auto* h  = create_solver_handle(sys);
  REQUIRE(h != nullptr);

  // Solve once
  auto r1 = solve_handle(h);
  CHECK(r1.converged);

  // Reset and re-solve
  reset_solver_handle(h, sys);
  auto r2 = solve_handle(h);
  CHECK(r2.converged);

  destroy_solver_handle(h);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: DC OPF
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("DC OPF: IEEE14 AC/DC runs", "[opf][dc_opf]") {
  using namespace hacdcpf;

  auto sys          = hacdcpf::io::build_ieee14_acdc();
  opf::DCOPFOptions opt;
  opt.verbose = false;

  auto result = hacdcpf::solve_dc_opf(sys, opt);

  // DC OPF should at minimum produce a result struct; convergence depends on
  // solver availability.
  REQUIRE(result.status.size() > 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: AC OPF
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("AC OPF: IEEE14 AC/DC runs", "[opf][ac_opf]") {
  using namespace hacdcpf;

  auto sys         = hacdcpf::io::build_ieee14_acdc();
  opf::ACOPFOptions opt;
  opt.verbose = false;

  auto result = hacdcpf::solve_ac_opf(sys, opt);

  REQUIRE(result.status.size() > 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: JSON I/O round-trip
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("JSON I/O: 2-bus AC system round-trip", "[io][json]") {
  using namespace hacdcpf;

  auto orig = make_simple_ac_system();
  auto json_str = hacdcpf::io::to_json(orig);

  REQUIRE_FALSE(json_str.empty());

  auto loaded = hacdcpf::io::from_json(json_str);
  REQUIRE(loaded.ac.buses.size() == orig.ac.buses.size());
  REQUIRE(loaded.ac.branches.size() == orig.ac.branches.size());
  REQUIRE(loaded.ac.generators.size() == orig.ac.generators.size());
  REQUIRE_THAT(loaded.base_mva, WithinAbs(orig.base_mva, 1e-9));
}

TEST_CASE("JSON I/O preserves reliability and DC operating-cost parameters",
          "[io][json][reliability][time_series]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  ACBus ac_bus; ac_bus.index = 1; ac_bus.bus_type = BusType::SLACK;
  sys.ac.buses.push_back(ac_bus);
  DCBus dc_bus; dc_bus.index = 1;
  sys.dc.buses.push_back(dc_bus);

  Generator gen; gen.index = 1; gen.bus = 1;
  gen.forced_outage_rate = 0.031; gen.mttr_hr = 17.0; gen.t_scheduled_hr = 9.0;
  sys.ac.generators.push_back(gen);
  StaticGeneratorDC dc_gen; dc_gen.index = 2; dc_gen.bus = 1;
  dc_gen.cost_c1 = 23.0; dc_gen.emission_factor_tco2_mwh = 0.41;
  dc_gen.mtbf_hours = 1200.0; dc_gen.mttr_hours = 11.0;
  sys.dc.dc_static_generators.push_back(dc_gen);
  DCStorage storage; storage.index = 3; storage.bus = 1;
  storage.charge_bid_price = 4.0; storage.discharge_bid_price = 8.0;
  storage.daily_cycle_limit = 1.25; storage.forced_outage_rate = 0.02;
  storage.mttr_hr = 13.0; storage.t_scheduled_hr = 7.0;
  sys.dc.dc_storage.push_back(storage);

  const auto roundtrip = hacdcpf::io::from_json(hacdcpf::io::to_json(sys));
  REQUIRE(roundtrip.ac.generators.size() == 1);
  CHECK_THAT(roundtrip.ac.generators[0].forced_outage_rate, WithinAbs(0.031, 1e-12));
  CHECK_THAT(roundtrip.ac.generators[0].mttr_hr, WithinAbs(17.0, 1e-12));
  CHECK_THAT(roundtrip.ac.generators[0].t_scheduled_hr, WithinAbs(9.0, 1e-12));
  REQUIRE(roundtrip.dc.dc_static_generators.size() == 1);
  CHECK_THAT(roundtrip.dc.dc_static_generators[0].cost_c1, WithinAbs(23.0, 1e-12));
  CHECK_THAT(roundtrip.dc.dc_static_generators[0].emission_factor_tco2_mwh, WithinAbs(0.41, 1e-12));
  REQUIRE(roundtrip.dc.dc_storage.size() == 1);
  CHECK_THAT(roundtrip.dc.dc_storage[0].charge_bid_price, WithinAbs(4.0, 1e-12));
  CHECK_THAT(roundtrip.dc.dc_storage[0].discharge_bid_price, WithinAbs(8.0, 1e-12));
  CHECK_THAT(roundtrip.dc.dc_storage[0].daily_cycle_limit, WithinAbs(1.25, 1e-12));
  CHECK_THAT(roundtrip.dc.dc_storage[0].t_scheduled_hr, WithinAbs(7.0, 1e-12));
}

// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Carbon analysis (full implementation)
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Carbon analysis: converged result has populated load_carbon", "[analysis][carbon]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  analysis::CarbonAnalysisOptions ca_opt;
  PowerFlowOptions pf_opt;

  // Use the two-arg overload (runs PF internally).
  auto result = analysis::compute_carbon_analysis(sys, pf_opt, ca_opt);

  // If power flow converged the struct should contain per-load carbon results.
  if (result.tracing_verified || result.matrix_solved || !result.load_carbon.empty()) {
    CHECK(!result.load_carbon.empty());
    for (const auto& lc : result.load_carbon) {
      CHECK(lc.carbon_intensity_tco2_mwh >= 0.0);
    }
  }
}

TEST_CASE("Carbon analysis: backward-compat run_carbon_analysis wrapper", "[analysis][carbon]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  analysis::CarbonAnalysisOptions opt;
  auto result = analysis::run_carbon_analysis(sys, opt);

  // Backward-compat wrapper should not throw and should return a valid struct.
  CHECK(result.matrix_residual >= 0.0);
}

TEST_CASE("Carbon analysis: unconverged PF returns empty result", "[analysis][carbon]") {
  using namespace hacdcpf;

  auto sys = make_simple_ac_system();
  PowerFlowResult unconverged;
  unconverged.converged = false;

  auto result = analysis::compute_carbon_analysis(sys, unconverged);
  CHECK_FALSE(result.tracing_verified);
  CHECK_FALSE(result.matrix_solved);
  CHECK(result.load_carbon.empty());
}


// ═══════════════════════════════════════════════════════════════════════════════
// Tests: Resilience stub
// ═══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Resilience assessment: stub returns valid struct", "[analysis][resilience]") {
  using namespace hacdcpf;

  auto sys                                = make_simple_ac_system();
  analysis::DistributionResilienceOptions opt;
  auto result = hacdcpf::run_distribution_resilience_assessment(sys, opt);

  CHECK(result.completed);
  CHECK(result.status == "Completed");
}

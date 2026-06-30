// Unit tests for the optional explicit DC branch flow + thermal-limit model in
// the time-series unit commitment
// (docs/sequential_production_simulation_rich_models.md §4.8).
//
// With enable_dc_branch_flows, each DC branch gets a transport flow variable
// bounded by its rating that enters the DC nodal balance, so power can move
// between DC buses up to the line capacity.  Without it, a DC bus that has only
// load (no local converter/source) cannot self-balance and the problem is
// infeasible.

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// AC slack bus + generator feeding a VSC into DC bus 10; a DC branch carries
// power to DC bus 11 which holds a DC load.
HybridPowerSystem make_dc_transport_system(double dc_load_mw, double branch_rate_mva) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  // Two AC buses so the nodal (DC power flow) UC path activates (net.n_bus > 1).
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0;
  sys.ac.buses.push_back(b1);
  sys.ac.buses.push_back(b2);

  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
  br.x_pu = 0.1; br.rate_a_mva = 1000.0; br.in_service = true;
  sys.ac.branches.push_back(br);

  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 500.0; g.pmin_mw = 0.0;
  g.cost_c1 = 10.0; g.is_slack = true; sys.ac.generators.push_back(g);

  // Two DC buses: 10 (voltage reference) and 11 (load).
  DCBus d10; d10.index = 10; d10.bus_type = DCBusType::DC_V; d10.in_service = true;
  DCBus d11; d11.index = 11; d11.bus_type = DCBusType::DC_P; d11.in_service = true;
  sys.dc.buses.push_back(d10);
  sys.dc.buses.push_back(d11);

  DCBranch dbr; dbr.index = 1; dbr.from_bus = 10; dbr.to_bus = 11;
  dbr.r_pu = 0.01; dbr.rate_a_mva = branch_rate_mva; dbr.in_service = true;
  sys.dc.branches.push_back(dbr);

  DCLoad dl; dl.index = 1; dl.bus = 11; dl.p_mw = dc_load_mw; dl.in_service = true;
  sys.dc.loads.push_back(dl);

  // VSC between AC bus 1 and DC bus 10 (can move power either way).
  VSCConverter c; c.index = 1; c.bus_ac = 1; c.bus_dc = 10;
  c.control_mode = ConverterMode::PQ_MODE;
  c.pmin_mw = -200.0; c.pmax_mw = 200.0; c.eta = 1.0; c.in_service = true;
  sys.vsc_converters.push_back(c);

  return sys;
}

TimeSeriesPFOptions dc_network_opts() {
  TimeSeriesPFOptions o;
  o.uc_solver = UCSolverChoice::SCIP;
  o.enable_network_constraints = true;
  o.enable_dc_network_constraints = true;
  return o;
}

TimeSeriesData one_step() {
  TimeSeriesData ts; ts.num_steps = 1; ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("DC branch flows: a load-only DC bus is infeasible without flows",
          "[time_series][dc_flows]") {
  auto sys = make_dc_transport_system(/*dc_load*/ 20.0, /*rate*/ 50.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = dc_network_opts();
  opts.enable_dc_branch_flows = false;  // flat-voltage self-balance

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  // DC bus 11 has only load and no local source → cannot self-balance.
  CHECK_FALSE(sched.feasible);
}

TEST_CASE("DC branch flows: transport over the DC line restores feasibility",
          "[time_series][dc_flows]") {
  auto sys = make_dc_transport_system(/*dc_load*/ 20.0, /*rate*/ 50.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = dc_network_opts();
  opts.enable_dc_branch_flows = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
}

TEST_CASE("DC branch flows: a too-small line rating is infeasible",
          "[time_series][dc_flows]") {
  // DC load 40 MW but the line can only carry 10 MW → transport is binding.
  auto sys = make_dc_transport_system(/*dc_load*/ 40.0, /*rate*/ 10.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = dc_network_opts();
  opts.enable_dc_branch_flows = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  CHECK_FALSE(sched.feasible);
}

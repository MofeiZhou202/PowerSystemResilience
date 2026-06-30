// Unit tests for the optional energy-router (multi-port converter) model in the
// time-series unit commitment
// (docs/sequential_production_simulation_rich_models.md §4.9).
//
// With enable_energy_router, each in-service EnergyRouter that still carries
// explicit ports contributes, per port, a signed net injection at the port's
// bus (modelled as an inflow/outflow pair) coupled by a lossy internal
// conservation constraint Σ_j η_j·p_in_j = Σ_j p_out_j.  This realises the
// N-port generalisation of the two-port VSC/DC-DC coupling.
//
// The router is exercised under transmission congestion: two AC buses joined by
// a thermally limited line, with all generation on bus 1 and all load on bus 2.
// When the line cannot carry the load, the router provides a parallel transfer
// path; its port efficiency shows up as extra generation (round-trip loss).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"

using namespace hacdcpf;

namespace {

// Bus 1 (slack) holds a cheap generator; bus 2 holds the load.  The 1–2 line is
// thermally limited to `line_rate` MVA, so at most `line_rate` MW can flow
// directly.  A two-port EnergyRouter bridges bus 1 → bus 2 outside the line.
HybridPowerSystem make_router_system(double port_eta, double line_rate, double load_mw) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ;    b2.base_kv = 110.0;
  sys.ac.buses.push_back(b1);
  sys.ac.buses.push_back(b2);

  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
  br.x_pu = 0.1; br.rate_a_mva = line_rate; br.in_service = true;
  sys.ac.branches.push_back(br);

  Generator g; g.index = 1; g.bus = 1; g.pmax_mw = 500.0; g.pmin_mw = 0.0;
  g.cost_c1 = 10.0; g.is_slack = true; sys.ac.generators.push_back(g);

  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = load_mw; ld.in_service = true;
  sys.ac.loads.push_back(ld);

  // Two-port router: port A (side 0) at bus 1, port B (side 1) at bus 2.
  EnergyRouter er; er.index = 1; er.in_service = true; er.p_rated_mw = 500.0;
  er.num_ports = 2;
  EnergyRouterPort pa; pa.index = 1; pa.bus = 1; pa.side = 0;
  pa.port_type = ERPortType::AC; pa.pmax_mw = 200.0; pa.eta = port_eta;
  pa.in_service = true;
  EnergyRouterPort pb; pb.index = 2; pb.bus = 2; pb.side = 1;
  pb.port_type = ERPortType::AC; pb.pmax_mw = 200.0; pb.eta = port_eta;
  pb.in_service = true;
  er.ports.push_back(pa);
  er.ports.push_back(pb);
  sys.energy_routers.push_back(er);

  return sys;
}

double total_gen(const UCSchedule& s) {
  double g = 0.0;
  if (!s.gen_dispatch.empty())
    for (double p : s.gen_dispatch[0]) g += p;
  return g;
}

TimeSeriesPFOptions nodal_opts() {
  TimeSeriesPFOptions o;
  o.uc_solver = UCSolverChoice::SCIP;
  o.enable_network_constraints = true;  // nodal DC-PF → line limit is active
  return o;
}

TimeSeriesData one_step() {
  TimeSeriesData ts; ts.num_steps = 1; ts.step_duration_hr = 1.0;
  return ts;
}

}  // namespace

TEST_CASE("Energy router: congested load is infeasible without the router",
          "[time_series][energy_router]") {
  // Load 50 MW on bus 2, but the only line can carry 10 MW and bus 2 has no
  // local source → 40 MW cannot be served.
  auto sys = make_router_system(/*eta*/ 1.0, /*line_rate*/ 10.0, /*load*/ 50.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = nodal_opts();
  opts.enable_energy_router = false;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  CHECK_FALSE(sched.feasible);
}

TEST_CASE("Energy router: a lossless router relieves the congestion",
          "[time_series][energy_router]") {
  auto sys = make_router_system(/*eta*/ 1.0, /*line_rate*/ 10.0, /*load*/ 50.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = nodal_opts();
  opts.enable_energy_router = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  // Line carries 10 MW, the router transfers the remaining 40 MW losslessly.
  // The cheap generator therefore serves the full 50 MW load → cost 10·50.
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(50.0, 1e-1));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(500.0, 1e-2));
}

TEST_CASE("Energy router: port efficiency shows up as round-trip loss",
          "[time_series][energy_router]") {
  // Same congestion, but the router ports are 80% efficient: delivering 40 MW
  // through the router consumes 40/0.8 = 50 MW at bus 1, so the generator must
  // produce 10 (line) + 50 (router draw) = 60 MW → cost 10·60 = 600.
  auto sys = make_router_system(/*eta*/ 0.8, /*line_rate*/ 10.0, /*load*/ 50.0);
  auto ts = one_step();

  TimeSeriesPFOptions opts = nodal_opts();
  opts.enable_energy_router = true;

  const UCSchedule sched = solve_unit_commitment(sys, ts, opts);
  REQUIRE(sched.feasible);
  CHECK_THAT(total_gen(sched), Catch::Matchers::WithinAbs(60.0, 5e-1));
  CHECK_THAT(sched.total_cost, Catch::Matchers::WithinRel(600.0, 2e-2));
  // The loss makes it strictly costlier than the lossless transfer.
  CHECK(sched.total_cost > 500.0 + 1.0);
}

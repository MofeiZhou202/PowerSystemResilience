// ONR option tests: split-domain radiality (AC/DC trees, converters as free
// bridges) and individual G4 (voltage) / G5 (thermal) constraint toggles.
#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {
// Two AC feeders, each with its own slack, bridged to a 2-bus DC link by two
// VSCs. A single merged spanning tree cannot keep both converters closed (it
// would create an AC-DC-AC loop); split-domain trees allow both.
HybridPowerSystem make_loop_case() {
  HybridPowerSystem s; s.base_mva = 100.0;
  ACBus a1; a1.index = 1; a1.bus_type = BusType::SLACK; a1.base_kv = 10; a1.vmax_pu = 1.1; a1.vmin_pu = 0.9;
  ACBus a2; a2.index = 2; a2.bus_type = BusType::SLACK; a2.base_kv = 10; a2.vmax_pu = 1.1; a2.vmin_pu = 0.9;
  s.ac.buses = {a1, a2};
  ExternalGrid e1; e1.index = 1; e1.bus = 1; e1.in_service = true; e1.s_sc_max_mva = 100;
  ExternalGrid e2; e2.index = 2; e2.bus = 2; e2.in_service = true; e2.s_sc_max_mva = 100;
  s.ac.external_grids = {e1, e2};
  ACBranch al; al.index = 1; al.from_bus = 1; al.to_bus = 2; al.r_pu = 0.05; al.x_pu = 0.1; al.rate_a_mva = 50; al.in_service = true;
  s.ac.branches = {al};
  DCBus d1; d1.index = 1; d1.base_kv = 5; DCBus d2; d2.index = 2; d2.base_kv = 5;
  s.dc.buses = {d1, d2};
  DCBranch dl; dl.index = 1; dl.from_bus = 1; dl.to_bus = 2; dl.r_pu = 0.01; dl.rate_a_mva = 50; dl.in_service = true;
  s.dc.branches = {dl};
  VSCConverter v1; v1.index = 1; v1.bus_ac = 1; v1.bus_dc = 1; v1.pmax_mw = 30; v1.in_service = true;
  VSCConverter v2; v2.index = 2; v2.bus_ac = 2; v2.bus_dc = 2; v2.pmax_mw = 30; v2.in_service = true;
  s.vsc_converters = {v1, v2};
  return s;
}
}  // namespace

TEST_CASE("Split-domain radiality keeps both converters as bridges", "[reconfig_opts]") {
  auto s = make_loop_case();
  TopoReconfOptions opt; opt.enable_pf = false; opt.split_domain_trees = true; opt.skip_heuristic = true;
  auto r = run_topology_reconfiguration(s, opt);
  int vsc_open = 0;
  for (const auto& b : r.open_branches)
    if (b.category == graph::EdgeCategory::VSC_Coupling) ++vsc_open;
  CHECK(r.feasible);
  CHECK(vsc_open == 0);  // both VSC bridges may stay closed
}

TEST_CASE("G4/G5 toggles do not break feasibility", "[reconfig_opts]") {
  auto s = make_loop_case();
  TopoReconfOptions base; base.split_domain_trees = true; base.skip_heuristic = true;
  base.enable_pf = true; base.enable_voltage = false; base.enable_thermal = false;
  // Connectivity-only PF: voltage and thermal groups disabled — still solvable.
  CHECK(run_topology_reconfiguration(s, base).feasible);
  TopoReconfOptions therm = base; therm.enable_thermal = true;
  CHECK(run_topology_reconfiguration(s, therm).feasible);
}

TEST_CASE("Per-domain root: meshed DC with own voltage source stays feasible", "[reconfig_opts]") {
  auto s = make_loop_case();
  s.dc.buses[0].bus_type = DCBusType::DC_V;  // DC source/reference
  TopoReconfOptions opt; opt.enable_pf = false; opt.split_domain_trees = true; opt.skip_heuristic = true;
  auto r = run_topology_reconfiguration(s, opt);
  CHECK(r.feasible);
}

TEST_CASE("DC mesh: parallel DC links may both stay closed", "[reconfig_opts]") {
  auto s = make_loop_case();
  s.dc.buses[0].bus_type = DCBusType::DC_V;
  DCBranch dl2; dl2.index = 2; dl2.from_bus = 1; dl2.to_bus = 2; dl2.r_pu = 0.02; dl2.rate_a_mva = 50; dl2.in_service = true;
  s.dc.branches.push_back(dl2);  // parallel DC link → DC loop
  TopoReconfOptions opt; opt.enable_pf = false; opt.allow_dc_mesh = true; opt.skip_heuristic = true;
  auto r = run_topology_reconfiguration(s, opt);
  int dc_open = 0;
  for (const auto& b : r.open_branches) if (b.category == graph::EdgeCategory::DC_Line) ++dc_open;
  CHECK(r.feasible);
  CHECK(dc_open == 0);  // DC loop allowed, neither parallel link opened
}

TEST_CASE("Split-domain radiality opens a DC cycle independently of AC", "[reconfig_opts]") {
  auto s = make_loop_case();
  DCBranch dl2; dl2.index = 2; dl2.from_bus = 1; dl2.to_bus = 2;
  dl2.r_pu = 0.02; dl2.rate_a_mva = 50; dl2.in_service = true;
  s.dc.branches.push_back(dl2);

  TopoReconfOptions opt;
  opt.enable_pf = false;
  opt.split_domain_trees = true;
  opt.skip_heuristic = true;
  opt.switchable_branches = {
      {graph::EdgeCategory::DC_Line, 1},
      {graph::EdgeCategory::DC_Line, 2},
  };
  const auto r = run_topology_reconfiguration(s, opt);

  REQUIRE(r.feasible);
  int dc_closed = 0;
  int ac_closed = 0;
  for (const auto& ref : r.closed_branches) {
    if (ref.category == graph::EdgeCategory::DC_Line) ++dc_closed;
    if (ref.category == graph::EdgeCategory::AC_Line) ++ac_closed;
  }
  CHECK(dc_closed == 1);
  CHECK(ac_closed == 1);
}

TEST_CASE("Loss-aware objective keeps a needed tie closed", "[reconfig_opts]") {
  // Single feeder fed at bus 1, load at 3; the only path is via the CB-less
  // line+tie. With loss-aware on, the supplying branch must stay closed.
  HybridPowerSystem s; s.base_mva = 100.0;
  ACBus b1; b1.index=1; b1.bus_type=BusType::SLACK; b1.base_kv=10; b1.vmax_pu=1.1; b1.vmin_pu=0.9;
  ACBus b2; b2.index=2; b2.base_kv=10; b2.vmax_pu=1.1; b2.vmin_pu=0.9;
  ACBus b3; b3.index=3; b3.base_kv=10; b3.vmax_pu=1.1; b3.vmin_pu=0.9;
  s.ac.buses = {b1, b2, b3};
  ACBranch a; a.index = 1; a.from_bus = 1; a.to_bus = 2; a.r_pu = 0.05; a.x_pu = 0.1; a.rate_a_mva = 50; a.in_service = true;
  ACBranch b; b.index = 2; b.from_bus = 2; b.to_bus = 3; b.r_pu = 0.05; b.x_pu = 0.1; b.rate_a_mva = 50; b.in_service = true;
  s.ac.branches = {a, b};
  ExternalGrid e; e.index = 1; e.bus = 1; e.in_service = true; e.s_sc_max_mva = 100; s.ac.external_grids = {e};
  Load ld; ld.index = 1; ld.bus = 3; ld.p_mw = 2; ld.in_service = true; s.ac.loads = {ld};
  TopoReconfOptions opt; opt.enable_pf = true; opt.loss_aware = true; opt.skip_heuristic = true;
  auto r = run_topology_reconfiguration(s, opt);
  CHECK(r.feasible);
  CHECK(r.closed_branches.size() == 2);  // both lines needed to feed the load
}

TEST_CASE("Fuse is never an immediate topology restoration candidate",
          "[reconfig_opts][device_constraints]") {
  HybridPowerSystem s;
  s.base_mva = s.ac.base_mva = 10.0;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 10.0;
  ACBus b2 = b1; b2.index = 2; b2.bus_type = BusType::PQ;
  ACBus b3 = b2; b3.index = 3;
  s.ac.buses = {b1, b2, b3};
  ACBranch feeder; feeder.index = 1; feeder.from_bus = 1; feeder.to_bus = 2;
  feeder.in_service = true; feeder.r_pu = 0.001; feeder.x_pu = 0.001;
  feeder.rate_a_mva = 10.0; s.ac.branches = {feeder};
  ExternalGrid grid; grid.index = 1; grid.bus = 1; grid.in_service = true;
  grid.s_sc_max_mva = 100.0; s.ac.external_grids = {grid};
  Load load; load.index = 1; load.bus = 3; load.p_mw = 0.5;
  load.in_service = true; s.ac.loads = {load};
  Switch fuse; fuse.index = 10; fuse.bus_from = 2; fuse.bus_to = 3;
  fuse.closed = false; fuse.normal_closed = false;
  fuse.switch_type = SwitchType::Fuse; fuse.role = SwitchRole::Protection;
  fuse.capabilities = effective_switch_capabilities(fuse);
  fuse.capabilities_explicit = true; s.ac.switches = {fuse};

  TopoReconfOptions opt; opt.enable_pf = true; opt.skip_heuristic = true;
  // The switch equivalent follows physical branch #1 as canonical branch #2.
  // Explicit caller authorization must still not bypass fuse safety policy.
  opt.switchable_branches = {{graph::EdgeCategory::AC_Line, 2}};
  const auto result = run_topology_reconfiguration(s, opt);

  REQUIRE(result.feasible);
  CHECK(result.total_shed_mw > 0.49);
  CHECK(std::none_of(result.switch_operations.begin(),
                     result.switch_operations.end(), [](const auto& action) {
                       return action.kind == TopoReconfResult::DeviceKind::Switch &&
                              action.index == 10;
                     }));
  CHECK(result.validity.device_capability_constraints_enforced);
}

TEST_CASE("Capable tie switch restores load with a validated direct action",
          "[reconfig_opts][device_constraints]") {
  HybridPowerSystem s;
  s.base_mva = s.ac.base_mva = 10.0;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 10.0;
  ACBus b2 = b1; b2.index = 2; b2.bus_type = BusType::PQ;
  s.ac.buses = {b1, b2};
  ExternalGrid grid; grid.index = 1; grid.bus = 1; grid.in_service = true;
  grid.s_sc_max_mva = 100.0; s.ac.external_grids = {grid};
  Load load; load.index = 1; load.bus = 2; load.p_mw = 0.5;
  load.in_service = true; s.ac.loads = {load};
  Switch tie; tie.index = 11; tie.bus_from = 1; tie.bus_to = 2;
  tie.closed = false; tie.normal_closed = false;
  tie.switch_type = SwitchType::LoadBreakSwitch; tie.role = SwitchRole::Tie;
  tie.capabilities = effective_switch_capabilities(tie);
  tie.capabilities_explicit = true; s.ac.switches = {tie};

  TopoReconfOptions opt; opt.enable_pf = true; opt.skip_heuristic = true;
  const auto result = run_topology_reconfiguration(s, opt);

  REQUIRE(result.feasible);
  CHECK(result.total_shed_mw < 1e-8);
  REQUIRE(result.ordered_switch_actions.size() == 1);
  CHECK(result.ordered_switch_actions.front().index == 11);
  CHECK(result.ordered_switch_actions.front().action == "close");
  CHECK(result.switch_sequence_valid);
}

TEST_CASE("Isolation switch operation is ordered behind upstream protection",
          "[reconfig_opts][device_constraints][interlock]") {
  HybridPowerSystem s;
  s.base_mva = s.ac.base_mva = 10.0;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 10.0;
  ACBus b2 = b1; b2.index = 2; b2.bus_type = BusType::PQ;
  ACBus b3 = b2; b3.index = 3;
  s.ac.buses = {b1, b2, b3};
  ExternalGrid grid; grid.index = 1; grid.bus = 1; grid.in_service = true;
  grid.s_sc_max_mva = 100.0; s.ac.external_grids = {grid};
  Load load; load.index = 1; load.bus = 3; load.p_mw = 0.5;
  load.in_service = true; s.ac.loads = {load};
  Switch breaker; breaker.index = 20; breaker.bus_from = 1; breaker.bus_to = 2;
  breaker.closed = true; breaker.switch_type = SwitchType::CircuitBreaker;
  breaker.role = SwitchRole::Protection;
  breaker.capabilities = effective_switch_capabilities(breaker);
  breaker.capabilities_explicit = true;
  Switch isolator; isolator.index = 21; isolator.bus_from = 2; isolator.bus_to = 3;
  isolator.closed = false; isolator.normal_closed = false;
  isolator.switch_type = SwitchType::Disconnector;
  isolator.role = SwitchRole::Isolation;
  isolator.upstream_protective_switch_index = 20;
  isolator.capabilities = effective_switch_capabilities(isolator);
  isolator.capabilities_explicit = true;
  s.ac.switches = {breaker, isolator};

  TopoReconfOptions opt; opt.enable_pf = true; opt.skip_heuristic = true;
  const auto result = run_topology_reconfiguration(s, opt);

  REQUIRE(result.feasible);
  CHECK(result.total_shed_mw < 1e-8);
  REQUIRE(result.ordered_switch_actions.size() == 3);
  CHECK(result.ordered_switch_actions[0].index == 20);
  CHECK(result.ordered_switch_actions[0].action == "open");
  CHECK(result.ordered_switch_actions[1].index == 21);
  CHECK(result.ordered_switch_actions[1].action == "close");
  CHECK(result.ordered_switch_actions[2].index == 20);
  CHECK(result.ordered_switch_actions[2].action == "reclose");
  CHECK(result.switch_sequence_valid);
  CHECK(result.validity.protection_interlocks_enforced);

  TopoReconfOptions constrained = opt;
  constrained.max_switch_ops = 2;
  const auto budgeted = run_topology_reconfiguration(s, constrained);
  REQUIRE(budgeted.feasible);
  CHECK(budgeted.total_shed_mw > 0.49);
  CHECK(std::none_of(budgeted.switch_operations.begin(),
                     budgeted.switch_operations.end(), [](const auto& action) {
                       return action.kind == TopoReconfResult::DeviceKind::Switch &&
                              action.index == 21;
                     }));
}

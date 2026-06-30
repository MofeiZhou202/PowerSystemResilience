// ONR option tests: split-domain radiality (AC/DC trees, converters as free
// bridges) and individual G4 (voltage) / G5 (thermal) constraint toggles.
#include <catch2/catch_test_macros.hpp>

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

// Device-flow + reconfig back-projection tests: validates the rich↔canonical
// mapping (BranchExpandMap CB/Switch endpoints), compute_device_terminal_flows
// across a merged-out breaker, and reconfig→Switch/CB operation projection.
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"

using namespace hacdcpf;

namespace {
ACBus mk_bus(int idx, BusType t = BusType::PQ, double kv = 12.47) {
  ACBus b; b.index = idx; b.bus_type = t; b.base_kv = kv; b.vmax_pu = 1.1; b.vmin_pu = 0.9; return b;
}
}  // namespace

TEST_CASE("CB collapsed by merge still reports flow via cut", "[device_flows]") {
  HybridPowerSystem s; s.base_mva = 100.0;
  s.ac.buses = {mk_bus(1, BusType::SLACK), mk_bus(2), mk_bus(3)};
  ACBranch ln; ln.index = 1; ln.from_bus = 1; ln.to_bus = 2; ln.r_pu = 0.01; ln.x_pu = 0.02; ln.rate_a_mva = 10; ln.in_service = true;
  s.ac.branches = {ln};
  CircuitBreaker cb; cb.index = 1; cb.bus_from = 2; cb.bus_to = 3; cb.closed = true; cb.in_service = true; cb.i_rated_ka = 1.0;
  s.ac.circuit_breakers = {cb};
  ExternalGrid eg; eg.index = 1; eg.bus = 1; eg.in_service = true; s.ac.external_grids = {eg};
  Load ld; ld.index = 1; ld.bus = 3; ld.p_mw = 4.0; ld.q_mvar = 2.0; ld.in_service = true; s.ac.loads = {ld};

  auto flows = compute_device_terminal_flows(s, {}, {});
  REQUIRE(flows.ac_circuit_breakers.size() == 1);
  const auto& f = flows.ac_circuit_breakers[0];
  CHECK(f.closed);
  // Power must cross the breaker to feed the 4 MW load behind it.
  CHECK(std::abs(std::abs(f.pf_mw) - 4.0) < 1e-6);
  CHECK(std::abs(std::abs(f.qf_mvar) - 2.0) < 1e-6);

  // Provenance: CB recorded as a CircuitBreaker origin with endpoints.
  auto proj = project_to_canonical_models(s);
  REQUIRE(proj.branch_expand_map.has_value());
  bool found = false;
  for (const auto& e : proj.branch_expand_map->entries)
    if (e.origin_type == BranchOriginType::CircuitBreaker && e.origin_index == 1) {
      found = true; CHECK(e.bus_from == 2); CHECK(e.bus_to == 3);
    }
  CHECK(found);
}

TEST_CASE("PF-aware CB flow uses solved line flows", "[device_flows]") {
  HybridPowerSystem s; s.base_mva = 100.0;
  s.ac.buses = {mk_bus(1, BusType::SLACK), mk_bus(2)};
  ACBranch ln; ln.index = 1; ln.from_bus = 1; ln.to_bus = 2; ln.r_pu = 0.01; ln.rate_a_mva = 10; ln.in_service = true;
  s.ac.branches = {ln};
  CircuitBreaker cb; cb.index = 1; cb.bus_from = 1; cb.bus_to = 2; cb.closed = true; cb.in_service = true; cb.i_rated_ka = 1.0;
  s.ac.circuit_breakers = {cb};
  // Solved line carries 5 MW / 2.5 MVAr into bus 2; CB parallels it.
  std::vector<BranchFlow> bf(1); bf[0].pf_mw = 5; bf[0].pt_mw = -5; bf[0].qf_mvar = 2.5; bf[0].qt_mvar = -2.5;
  auto flows = compute_device_terminal_flows(s, bf);
  REQUIRE(flows.ac_circuit_breakers.size() == 1);
  CHECK(std::abs(std::abs(flows.ac_circuit_breakers[0].pf_mw) - 5.0) < 1e-6);
}

TEST_CASE("Open breaker reports zero flow", "[device_flows]") {
  HybridPowerSystem s; s.base_mva = 100.0;
  s.ac.buses = {mk_bus(1, BusType::SLACK), mk_bus(2)};
  CircuitBreaker cb; cb.index = 7; cb.bus_from = 1; cb.bus_to = 2; cb.closed = false; cb.in_service = true;
  s.ac.circuit_breakers = {cb};
  auto flows = compute_device_terminal_flows(s, {}, {});
  REQUIRE(flows.ac_circuit_breakers.size() == 1);
  CHECK_FALSE(flows.ac_circuit_breakers[0].closed);
  CHECK(flows.ac_circuit_breakers[0].pf_mw == 0.0);
}

TEST_CASE("Reconfig keeps dead sections behind open ties", "[device_flows]") {
  // Bus 3 is fed only through an open tie; default projection strips it, the
  // reconfig-friendly overload must keep it as a reconnection candidate.
  HybridPowerSystem s; s.base_mva = 100.0;
  s.ac.buses = {mk_bus(1, BusType::SLACK), mk_bus(2), mk_bus(3)};
  ACBranch l12; l12.index = 1; l12.from_bus = 1; l12.to_bus = 2; l12.r_pu = 0.01; l12.x_pu = 0.02; l12.rate_a_mva = 10; l12.in_service = true;
  ACBranch tie; tie.index = 2; tie.from_bus = 2; tie.to_bus = 3; tie.r_pu = 0.01; tie.x_pu = 0.02; tie.rate_a_mva = 10; tie.in_service = false;
  s.ac.branches = {l12, tie};
  ExternalGrid eg; eg.index = 1; eg.bus = 1; eg.in_service = true; s.ac.external_grids = {eg};
  Load ld; ld.index = 1; ld.bus = 3; ld.p_mw = 2.0; ld.in_service = true; s.ac.loads = {ld};

  CHECK(project_to_canonical_models(s, true).ac.buses.size() == 2);   // bus 3 stripped
  CHECK(project_to_canonical_models(s, false).ac.buses.size() == 3);  // bus 3 preserved
}


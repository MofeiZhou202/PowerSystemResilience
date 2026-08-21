/// @file test_dc_short_circuit.cpp
/// @brief Resistive DC bolted-fault-level estimator (analysis/dc_short_circuit).
///
/// Tags: [analysis], [dc], [shortcircuit]

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>

#include "hacdcpf/analysis/dc_short_circuit.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Matchers::WithinAbs;

namespace {

// source(DC_V) --r12-- mid(DC_P) --r23-- end(DC_P)   (radial), all at base_kv kV.
HybridPowerSystem radial_dc(double r12, double r23, double base_kv = 0.75,
                            double base_mva = 1.0) {
  HybridPowerSystem sys;
  sys.base_mva = base_mva;
  sys.dc.base_mva = base_mva;
  auto bus = [&](int idx, const char* nm, DCBusType t) {
    DCBus b;
    b.index = idx;
    b.name = nm;
    b.bus_type = t;
    b.base_kv = base_kv;
    b.vm_pu = 1.0;
    return b;
  };
  sys.dc.buses.push_back(bus(1, "S", DCBusType::DC_V));
  sys.dc.buses.push_back(bus(2, "MID", DCBusType::DC_P));
  sys.dc.buses.push_back(bus(3, "END", DCBusType::DC_P));
  auto br = [&](int idx, int f, int t, double r) {
    DCBranch b;
    b.index = idx;
    b.name = "DL" + std::to_string(idx);
    b.from_bus = f;
    b.to_bus = t;
    b.r_pu = r;
    return b;
  };
  sys.dc.branches.push_back(br(1, 1, 2, r12));
  sys.dc.branches.push_back(br(2, 2, 3, r23));
  return sys;
}

}  // namespace

TEST_CASE("DC fault level: radial network bolted fault", "[analysis][dc][shortcircuit]") {
  // Fault at the MID bus: only the 0.1 pu source branch is in the path.
  const HybridPowerSystem sys = radial_dc(0.1, 0.1);
  const DCFaultResult r = dc_bus_fault_level(sys, /*dc_bus_id=*/2);
  REQUIRE(r.solved);
  CHECK_THAT(r.v_prefault_pu, WithinAbs(1.0, 1e-9));   // no load -> source voltage
  CHECK_THAT(r.r_thevenin_pu, WithinAbs(0.1, 1e-9));   // single 0.1 pu branch
  CHECK_THAT(r.i_fault_pu, WithinAbs(10.0, 1e-9));     // 1.0 / 0.1
  // I_base = base_mva / base_kv = 1 / 0.75; I_f = 10 * I_base.
  CHECK_THAT(r.i_fault_ka, WithinAbs(10.0 / 0.75, 1e-6));
}

TEST_CASE("DC fault level: series resistance adds up at a downstream bus",
          "[analysis][dc][shortcircuit]") {
  // Fault at END: path is r12 + r23 = 0.05 + 0.05 = 0.1 pu.
  const HybridPowerSystem sys = radial_dc(0.05, 0.05);
  const DCFaultResult r = dc_bus_fault_level(sys, /*dc_bus_id=*/3);
  REQUIRE(r.solved);
  CHECK_THAT(r.r_thevenin_pu, WithinAbs(0.1, 1e-9));
  CHECK_THAT(r.i_fault_pu, WithinAbs(10.0, 1e-9));
}

TEST_CASE("DC fault level: explicit fault resistance lowers the current",
          "[analysis][dc][shortcircuit]") {
  const HybridPowerSystem sys = radial_dc(0.1, 0.1);
  DCFaultOptions opt;
  opt.fault_resistance_pu = 0.1;  // total path 0.1 + 0.1 = 0.2 pu
  const DCFaultResult r = dc_bus_fault_level(sys, 2, opt);
  REQUIRE(r.solved);
  CHECK_THAT(r.i_fault_pu, WithinAbs(5.0, 1e-9));  // 1.0 / 0.2
}

TEST_CASE("DC fault level: faulting a stiff source bus is reported, not solved",
          "[analysis][dc][shortcircuit]") {
  const HybridPowerSystem sys = radial_dc(0.1, 0.1);
  const DCFaultResult r = dc_bus_fault_level(sys, /*dc_bus_id=*/1);  // the DC_V bus
  CHECK_FALSE(r.solved);
  CHECK_FALSE(r.message.empty());
}

TEST_CASE("DC fault level: no voltage source means no fault level",
          "[analysis][dc][shortcircuit]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1);
  sys.dc.buses[0].bus_type = DCBusType::DC_P;  // remove the only DC_V source
  const DCFaultResult r = dc_bus_fault_level(sys, 2);
  CHECK_FALSE(r.solved);
  CHECK_FALSE(r.message.empty());
}

TEST_CASE("DC fault level: closed DCCB adds series resistance to controlled branch",
          "[analysis][dc][shortcircuit][dccb]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1, 1.0, 1.0);
  DCCircuitBreaker cb;
  cb.index = 7;
  cb.name = "DCCB_source_mid";
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.element_type = "dc_branch";
  cb.element_id = 1;
  cb.closed = true;
  cb.in_service = true;
  cb.r_ohm = 0.1;          // Zbase = 1^2/1 = 1 ohm => 0.1 pu
  cb.rated_voltage_kv = 1.0;
  cb.i_breaking_ka = 4.0;
  sys.dc.dc_circuit_breakers = {cb};

  const DCFaultResult r = dc_bus_fault_level(sys, 2);
  REQUIRE(r.solved);
  CHECK_THAT(r.r_thevenin_pu, WithinAbs(0.2, 1e-9));
  CHECK_THAT(r.i_fault_pu, WithinAbs(5.0, 1e-9));
  REQUIRE(r.breaker_duties.size() == 1);
  CHECK(r.breaker_duties.front().controls_branch);
  CHECK(r.breaker_duties.front().model == "closed_series_branch");
  CHECK_FALSE(r.breaker_duties.front().breaking_rating_ok);
}

TEST_CASE("DC fault level: open DCCB blocks controlled branch",
          "[analysis][dc][shortcircuit][dccb]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1);
  DCCircuitBreaker cb;
  cb.index = 8;
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.element_type = "dc_branch";
  cb.element_id = 1;
  cb.closed = false;
  cb.in_service = true;
  sys.dc.dc_circuit_breakers = {cb};

  const DCFaultResult r = dc_bus_fault_level(sys, 2);
  CHECK_FALSE(r.solved);
  CHECK(r.message.find("no resistive path") != std::string::npos);
  CHECK(r.dccb_blocked_branch_count == 1);
}

TEST_CASE("DC fault level: unassigned closed DCCB can act as standalone edge",
          "[analysis][dc][shortcircuit][dccb]") {
  HybridPowerSystem sys = radial_dc(0.5, 0.1, 1.0, 1.0);
  DCCircuitBreaker cb;
  cb.index = 9;
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.closed = true;
  cb.in_service = true;
  cb.r_ohm = 0.1;
  cb.rated_voltage_kv = 1.0;
  sys.dc.dc_circuit_breakers = {cb};

  DCFaultOptions with_edge;
  with_edge.dc_breakers_control_branches = false;
  with_edge.add_unassigned_closed_breaker_edges = true;
  const DCFaultResult r_edge = dc_bus_fault_level(sys, 2, with_edge);
  REQUIRE(r_edge.solved);

  DCFaultOptions no_edge = with_edge;
  no_edge.add_unassigned_closed_breaker_edges = false;
  const DCFaultResult r_no_edge = dc_bus_fault_level(sys, 2, no_edge);
  REQUIRE(r_no_edge.solved);

  CHECK(r_edge.r_thevenin_pu < r_no_edge.r_thevenin_pu);
  CHECK(r_edge.dccb_edges_used == 1);
  REQUIRE(r_edge.breaker_duties.size() == 1);
  CHECK(r_edge.breaker_duties.front().model == "closed_standalone_edge");
}

TEST_CASE("DC fault level contracts ideal branches instead of opening them",
          "[analysis][dc][shortcircuit][ideal-edge]") {
  const HybridPowerSystem sys = radial_dc(0.0, 0.1, 1.0, 1.0);
  const auto result = dc_bus_fault_level(sys, 3);
  REQUIRE(result.solved);
  CHECK_THAT(result.r_thevenin_pu, WithinAbs(0.1, 1e-9));
  CHECK_THAT(result.i_fault_pu, WithinAbs(10.0, 1e-9));
}

TEST_CASE("DC fault level uses actual DCCB branch current",
          "[analysis][dc][shortcircuit][dccb][current-recovery]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1, 1.0, 1.0);
  DCBranch spur;
  spur.index = 3;
  spur.from_bus = 2;
  spur.to_bus = 4;
  spur.r_pu = 0.1;
  sys.dc.branches.push_back(spur);
  DCBus open_end;
  open_end.index = 4;
  open_end.bus_type = DCBusType::DC_P;
  open_end.base_kv = 1.0;
  open_end.vm_pu = 1.0;
  sys.dc.buses.push_back(open_end);
  DCCircuitBreaker cb;
  cb.index = 30;
  cb.bus_from = 2;
  cb.bus_to = 4;
  cb.element_type = "dc_branch";
  cb.element_id = 3;
  cb.closed = true;
  cb.r_ohm = 0.01;
  cb.rated_voltage_kv = 1.0;
  sys.dc.dc_circuit_breakers = {cb};

  const auto result = dc_bus_fault_level(sys, 3);
  REQUIRE(result.solved);
  REQUIRE(result.breaker_duties.size() == 1);
  CHECK(result.i_fault_ka > 1.0);
  CHECK_THAT(result.breaker_duties.front().i_duty_ka, WithinAbs(0.0, 1e-9));
}

TEST_CASE("DC parallel breaker duties follow conductance division",
          "[analysis][dc][shortcircuit][dccb][parallel]") {
  HybridPowerSystem sys = radial_dc(0.09, 0.5, 1.0, 1.0);
  sys.dc.branches.resize(1);
  DCBranch parallel = sys.dc.branches.front();
  parallel.index = 2;
  parallel.r_pu = 0.19;
  sys.dc.branches.push_back(parallel);
  for (int i = 0; i < 2; ++i) {
    DCCircuitBreaker cb;
    cb.index = 40 + i;
    cb.bus_from = 1;
    cb.bus_to = 2;
    cb.element_type = "dc_branch";
    cb.element_id = i + 1;
    cb.r_ohm = 0.01;
    cb.rated_voltage_kv = 1.0;
    cb.closed = true;
    sys.dc.dc_circuit_breakers.push_back(cb);
  }
  const auto result = dc_bus_fault_level(sys, 2);
  REQUIRE(result.solved);
  REQUIRE(result.breaker_duties.size() == 2);
  CHECK_THAT(result.breaker_duties[0].i_duty_ka, WithinAbs(10.0, 1e-8));
  CHECK_THAT(result.breaker_duties[1].i_duty_ka, WithinAbs(5.0, 1e-8));
  CHECK_THAT(result.i_fault_ka, WithinAbs(15.0, 1e-8));
}

TEST_CASE("DC multiple breakers on one branch form a series chain",
          "[analysis][dc][shortcircuit][dccb][series-chain]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1, 1.0, 1.0);
  for (int i = 0; i < 2; ++i) {
    DCCircuitBreaker cb;
    cb.index = 50 + i;
    cb.bus_from = 1;
    cb.bus_to = 2;
    cb.element_type = "dc_branch";
    cb.element_id = 1;
    cb.r_ohm = 0.05;
    cb.rated_voltage_kv = 1.0;
    cb.closed = true;
    sys.dc.dc_circuit_breakers.push_back(cb);
  }
  const auto result = dc_bus_fault_level(sys, 2);
  REQUIRE(result.solved);
  CHECK_THAT(result.r_thevenin_pu, WithinAbs(0.2, 1e-9));
  REQUIRE(result.breaker_duties.size() == 2);
  for (const auto& duty : result.breaker_duties) {
    CHECK(duty.model == "closed_series_branch_chain");
    CHECK_THAT(duty.i_duty_ka, WithinAbs(5.0, 1e-8));
  }
}

TEST_CASE("DC ambiguous terminal-only breaker assignment is rejected",
          "[analysis][dc][shortcircuit][dccb][validation]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1);
  DCBranch parallel = sys.dc.branches.front();
  parallel.index = 99;
  sys.dc.branches.push_back(parallel);
  DCCircuitBreaker cb;
  cb.index = 60;
  cb.bus_from = 1;
  cb.bus_to = 2;
  cb.closed = true;
  sys.dc.dc_circuit_breakers = {cb};
  CHECK_THROWS_AS(dc_bus_fault_level(sys, 2), std::invalid_argument);
}

TEST_CASE("DC option validation rejects nonphysical and non-finite values",
          "[analysis][dc][shortcircuit][validation]") {
  const auto sys = radial_dc(0.1, 0.1);
  DCFaultOptions negative;
  negative.fault_resistance_pu = -0.1;
  CHECK_THROWS_AS(dc_bus_fault_level(sys, 2, negative), std::invalid_argument);
  DCFaultOptions nonfinite;
  nonfinite.source_voltage_pu = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(dc_bus_fault_level(sys, 2, nonfinite), std::invalid_argument);
}

TEST_CASE("DC batch reuses topology and observes cancellation",
          "[analysis][dc][shortcircuit][batch][cancel]") {
  const auto sys = radial_dc(0.1, 0.1);
  const auto results = dc_bus_fault_levels(sys, {2, 3});
  REQUIRE(results.size() == 2);
  CHECK(results[0].solved);
  CHECK(results[1].solved);
  DCFaultOptions cancelled;
  cancelled.cancellation_requested = [] { return true; };
  CHECK_THROWS(dc_bus_fault_levels(sys, {2, 3}, cancelled));
}

TEST_CASE("DC source-free islands do not singularize supplied components",
          "[analysis][dc][shortcircuit][island]") {
  HybridPowerSystem sys = radial_dc(0.1, 0.1);
  DCBus b4;
  b4.index = 4; b4.bus_type = DCBusType::DC_P; b4.base_kv = 0.75; b4.vm_pu = 1.0;
  DCBus b5 = b4; b5.index = 5;
  sys.dc.buses.push_back(b4);
  sys.dc.buses.push_back(b5);
  DCBranch island;
  island.index = 4; island.from_bus = 4; island.to_bus = 5; island.r_pu = 0.1;
  sys.dc.branches.push_back(island);
  const auto results = dc_bus_fault_levels(sys, {2, 5});
  REQUIRE(results.size() == 2);
  CHECK(results[0].solved);
  CHECK_FALSE(results[1].solved);
  CHECK(results[1].status == "unsupplied_island");
}

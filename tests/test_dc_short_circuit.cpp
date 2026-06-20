/// @file test_dc_short_circuit.cpp
/// @brief Resistive DC bolted-fault-level estimator (analysis/dc_short_circuit).
///
/// Tags: [analysis], [dc], [shortcircuit]

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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

#include <algorithm>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

namespace {

bool has_rule(const hacdcpf::PowerFlowResult& pf, const std::string& rule_id) {
  const auto& issues = pf.diagnostics.converter_coordination.issues;
  return std::any_of(issues.begin(), issues.end(), [&](const auto& issue) {
    return issue.rule_id == rule_id;
  });
}

bool report_has_rule(const hacdcpf::powerflow::ConverterCoordinationReport& report,
                     const std::string& rule_id) {
  return std::any_of(report.issues.begin(), report.issues.end(), [&](const auto& issue) {
    return issue.rule_id == rule_id;
  });
}

hacdcpf::DCBus dc_bus(int index, hacdcpf::DCBusType type = hacdcpf::DCBusType::DC_P) {
  hacdcpf::DCBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.vm_pu = 1.0;
  bus.in_service = true;
  return bus;
}

hacdcpf::DCBranch dc_branch(int index, int from, int to) {
  hacdcpf::DCBranch branch;
  branch.index = index;
  branch.from_bus = from;
  branch.to_bus = to;
  branch.r_pu = 0.01;
  branch.in_service = true;
  return branch;
}

hacdcpf::DCCircuitBreaker dc_breaker(int index, int from, int to, bool closed) {
  hacdcpf::DCCircuitBreaker breaker;
  breaker.index = index;
  breaker.bus_from = from;
  breaker.bus_to = to;
  breaker.closed = closed;
  breaker.in_service = true;
  return breaker;
}

hacdcpf::DCDCConverter dcdc_converter(int index,
                                      int bus_in,
                                      int bus_out,
                                      hacdcpf::DCDCControlMode mode) {
  hacdcpf::DCDCConverter dcdc;
  dcdc.index = index;
  dcdc.bus_in = bus_in;
  dcdc.bus_out = bus_out;
  dcdc.control_mode = mode;
  dcdc.p_ref_mw = 1.0;
  dcdc.v_ref_pu = 1.0;
  dcdc.k_droop = 0.05;
  dcdc.eta = 0.98;
  dcdc.in_service = true;
  return dcdc;
}

hacdcpf::VSCConverter vdc_vsc(int index, int dc_bus, double v_set_pu) {
  hacdcpf::VSCConverter conv;
  conv.index = index;
  conv.bus_ac = index + 1;
  conv.bus_dc = dc_bus;
  conv.control_mode = hacdcpf::ConverterMode::VDC_Q;
  conv.v_dc_set_pu = v_set_pu;
  conv.k_vdc = 0.1;
  conv.pmax_mw = 10.0;
  conv.pmin_mw = -10.0;
  conv.p_rated_mw = 10.0;
  conv.in_service = true;
  return conv;
}

}  // namespace

TEST_CASE("Converter coordination check blocks a reference-less fixed-P DC island",
          "[converter][coordination]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);

  REQUIRE_FALSE(sys.dc.buses.empty());
  REQUIRE_FALSE(sys.vsc_converters.empty());
  REQUIRE(sys.dc.buses.front().bus_type == hacdcpf::DCBusType::DC_P);
  REQUIRE(sys.vsc_converters.front().control_mode == hacdcpf::ConverterMode::PQ_MODE);

  hacdcpf::PowerFlowOptions opt;
  opt.enable_converter_coordination_check = true;
  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, opt);

  CHECK_FALSE(pf.converged);
  CHECK(pf.diagnostics.converter_coordination.enabled);
  CHECK_FALSE(pf.diagnostics.converter_coordination.feasible);
  CHECK(has_rule(pf, "DCISLAND-01"));
  CHECK(pf.diagnostics.termination_reason == "Converter coordination feasibility check failed");
}

TEST_CASE("Converter coordination check is opt-in for backward compatibility",
          "[converter][coordination]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);

  hacdcpf::PowerFlowOptions opt;
  opt.enable_converter_coordination_check = false;
  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, opt);

  CHECK_FALSE(pf.diagnostics.converter_coordination.enabled);
  CHECK(pf.diagnostics.converter_coordination.feasible);
}

TEST_CASE("VDC-controlled VSC supplies the DC island reference but releases P setpoint",
          "[converter][coordination]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  REQUIRE_FALSE(sys.vsc_converters.empty());

  sys.vsc_converters.front().control_mode = hacdcpf::ConverterMode::VDC_Q;
  sys.vsc_converters.front().p_set_mw = 5.0;
  sys.vsc_converters.front().pmax_mw = 100.0;
  sys.vsc_converters.front().pmin_mw = -100.0;

  hacdcpf::PowerFlowOptions opt;
  opt.enable_converter_coordination_check = true;
  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, opt);

  CHECK(pf.diagnostics.converter_coordination.enabled);
  CHECK(pf.diagnostics.converter_coordination.feasible);
  CHECK(has_rule(pf, "ACDC-03"));
  CHECK_FALSE(has_rule(pf, "DCISLAND-01"));
}

TEST_CASE("DC/DC converters do not merge DC voltage islands",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.dcdc_converters = {dcdc_converter(7, 1, 2, hacdcpf::DCDCControlMode::Power)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report.enabled);
  REQUIRE(report.dc_islands.size() == 2);
  CHECK(report.dc_islands[0].dc_buses == std::vector<int>{1});
  CHECK(report.dc_islands[1].dc_buses == std::vector<int>{2});
  CHECK_FALSE(report.feasible);
  CHECK(report_has_rule(report, "DCISLAND-01"));
}

TEST_CASE("Closed metallic DC paths merge voltage islands but DC/DC does not",
          "[converter][coordination][topology]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2), dc_bus(3)};
  sys.dc.dc_circuit_breakers = {dc_breaker(1, 1, 2, true)};
  sys.dc.dcdc_converters = {dcdc_converter(2, 2, 3, hacdcpf::DCDCControlMode::Power)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  REQUIRE(report.dc_islands.size() == 2);
  CHECK(report.dc_islands[0].dc_buses == std::vector<int>({1, 2}));
  CHECK(report.dc_islands[1].dc_buses == std::vector<int>({3}));
}

TEST_CASE("DC_V bus type is not accepted as a physical voltage source",
          "[converter][coordination][dc-bus]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1, hacdcpf::DCBusType::DC_V)};
  hacdcpf::PVArrayDC pv;
  pv.index = 1;
  pv.bus = 1;
  pv.p_set_mw = 0.5;
  pv.in_service = true;
  sys.dc.pv_arrays = {pv};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  REQUIRE(report.dc_islands.size() == 1);
  CHECK(report.dc_islands.front().declared_v_buses == std::vector<int>{1});
  CHECK(report.dc_islands.front().hard_vdc_sources == 0);
  CHECK(report_has_rule(report, "DCBUS-01"));
  CHECK(report_has_rule(report, "DCISLAND-01"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("Conflicting VDC setpoints in one DC island are fatal",
          "[converter][coordination][vsc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};
  sys.vsc_converters = {vdc_vsc(1, 1, 1.00), vdc_vsc(2, 2, 1.05)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  REQUIRE(report.dc_islands.size() == 1);
  CHECK(report.dc_islands.front().droop_sources == 2);
  CHECK(report_has_rule(report, "DCISLAND-05"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("DC/DC Voltage mode is blocked until the PF model enforces v_ref",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.dcdc_converters = {dcdc_converter(3, 1, 2, hacdcpf::DCDCControlMode::Voltage)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCDC-CTRL-01"));
  CHECK_FALSE(report.feasible);
  CHECK(report.blocking_count() > 0);
}

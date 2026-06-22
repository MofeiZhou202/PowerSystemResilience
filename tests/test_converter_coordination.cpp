#include <algorithm>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"

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

TEST_CASE("A reference-less fixed-P DC island is solved by auto-promoting the PQ VSC",
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

  // The unified solver auto-promotes the in-service PQ VSC to VDC_Q so it forms
  // the DC voltage reference.  The coordination check must mirror that and treat
  // the island as feasible (an informational note, not a blocking Fatal),
  // otherwise it would reject a system the solver can actually solve.
  CHECK(pf.converged);
  CHECK(pf.diagnostics.converter_coordination.enabled);
  CHECK(pf.diagnostics.converter_coordination.feasible);
  CHECK_FALSE(pf.diagnostics.converter_coordination.has_blocking_issue());
  CHECK_FALSE(has_rule(pf, "DCISLAND-01"));
  CHECK(has_rule(pf, "DCISLAND-PROMOTE-01"));
  // Stiff auto-promotion keeps Vdc within the bus voltage limits.
  REQUIRE_FALSE(pf.vdc.empty());
  CHECK(pf.vdc.front() >= sys.dc.buses.front().vmin_pu - 1e-6);
  CHECK(pf.vdc.front() <= sys.dc.buses.front().vmax_pu + 1e-6);
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

TEST_CASE("DC_V bus type alone is not accepted as a physical voltage source",
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
  CHECK(report.dc_islands.front().has_declared_v_bus);
  // A DC_V bus type is only a declaration, not a physical voltage-forming
  // device.  With no real VSC/ESS/source behind it the island is rejected as
  // reference-less (a bus pin would be a non-physical reference).
  CHECK(report.dc_islands.front().hard_vdc_sources == 0);
  CHECK(report.dc_islands.front().promotable_vsc_sources == 0);
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

TEST_CASE("A DC grid-forming converter cannot hard-constrain AC active power (ACDC-GFM-01)",
          "[converter][coordination][gfm]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1)};
  hacdcpf::VSCConverter conv = vdc_vsc(1, 1, 1.0);  // VDC_Q, k_vdc=0.1 -> forms Vdc
  conv.p_set_mw = 3.0;
  conv.p_is_hard_constraint = true;  // contradictory: forms Vdc yet pins AC P
  sys.vsc_converters = {conv};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "ACDC-GFM-01"));
  CHECK_FALSE(report_has_rule(report, "ACDC-03"));
  CHECK_FALSE(report.feasible);  // an Error blocks the solve
}

TEST_CASE("A DC grid-forming converter with a soft P schedule only warns (ACDC-03)",
          "[converter][coordination][gfm]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1)};
  hacdcpf::VSCConverter conv = vdc_vsc(1, 1, 1.0);
  conv.p_set_mw = 3.0;
  conv.p_is_hard_constraint = false;  // p_set_mw is only a schedule/initial value
  sys.vsc_converters = {conv};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "ACDC-03"));
  CHECK_FALSE(report_has_rule(report, "ACDC-GFM-01"));
  CHECK(report.feasible);  // a warning does not block; the VDC converter forms Vdc
}

TEST_CASE("Power-controlled DC/DC needs a voltage reference on both sides (DCDC-CTRL-05)",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  // Input bus 1 has a VDC VSC reference; output bus 2 has no reference.
  sys.vsc_converters = {vdc_vsc(5, 1, 1.0)};
  sys.dc.dcdc_converters = {dcdc_converter(1, 1, 2, hacdcpf::DCDCControlMode::Power)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCDC-CTRL-05"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("Output-droop DC/DC needs an input-side voltage reference (DCDC-CTRL-03)",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  // Droop DC/DC forms the output (bus 2) voltage, but the input island (bus 1)
  // has no source to supply the regulated output.
  sys.dc.dcdc_converters = {dcdc_converter(1, 1, 2, hacdcpf::DCDCControlMode::Droop)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCDC-CTRL-03"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("DC/DC Buck duty ratio feasibility is backed out from port voltages",
          "[converter][dcdc][duty]") {
  hacdcpf::DCDCConverter c;
  c.topology = hacdcpf::DCDCTopology::Buck;
  c.d_min = 0.05;
  c.d_max = 0.95;

  // Buck steps down: Vout < Vin -> D = Vout/Vin in (0,1), feasible.
  const auto step_down = hacdcpf::powerflow::dcdc_duty_ratio(c, 1.0, 0.5);
  CHECK(step_down.defined);
  CHECK(step_down.feasible);
  CHECK(std::abs(step_down.duty - 0.5) < 1e-9);

  // A Buck cannot step up: Vout > Vin -> D > 1, outside [d_min, d_max].
  const auto step_up = hacdcpf::powerflow::dcdc_duty_ratio(c, 1.0, 1.5);
  CHECK(step_up.defined);
  CHECK_FALSE(step_up.feasible);

  // Generic topology imposes no duty law.
  c.topology = hacdcpf::DCDCTopology::Generic;
  const auto generic = hacdcpf::powerflow::dcdc_duty_ratio(c, 1.0, 1.5);
  CHECK_FALSE(generic.defined);
  CHECK(generic.feasible);
}

TEST_CASE("Master-slave DC voltage group requires exactly one master (DCISLAND-MS-01)",
          "[converter][coordination][multisource]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};  // one metallic island
  auto m1 = vdc_vsc(1, 1, 1.0);
  m1.coordination_group_id = "grp";
  m1.is_master = true;
  auto m2 = vdc_vsc(2, 2, 1.0);
  m2.coordination_group_id = "grp";
  m2.is_master = true;  // two masters in one group -> illegal
  sys.vsc_converters = {m1, m2};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCISLAND-MS-01"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("Participation factors in a DC voltage group must sum to one (DCISLAND-PARTICIPATION-01)",
          "[converter][coordination][multisource]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};
  auto p1 = vdc_vsc(1, 1, 1.0);
  p1.coordination_group_id = "grp";
  p1.participation_factor = 0.3;
  auto p2 = vdc_vsc(2, 2, 1.0);
  p2.coordination_group_id = "grp";
  p2.participation_factor = 0.3;  // 0.3 + 0.3 = 0.6 != 1
  sys.vsc_converters = {p1, p2};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCISLAND-PARTICIPATION-01"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("A rigid Vdc source plus participation sharing is over-constrained (DCISLAND-PARTICIPATION-02)",
          "[converter][coordination][multisource]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};
  hacdcpf::Storage st;
  st.index = 1;
  st.bus = 1;
  st.in_service = true;
  st.control_mode = "DC_V";  // rigid DC voltage source
  sys.dc.storage = {st};
  auto p1 = vdc_vsc(2, 2, 1.0);
  p1.participation_factor = 1.0;  // participation sharing alongside a rigid source
  sys.vsc_converters = {p1};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCISLAND-PARTICIPATION-02"));
  CHECK_FALSE(report.feasible);
}

TEST_CASE("A balanced participation group with one master is accepted",
          "[converter][coordination][multisource]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};
  auto p1 = vdc_vsc(1, 1, 1.0);
  p1.coordination_group_id = "grp";
  p1.is_master = true;
  p1.participation_factor = 0.6;
  auto p2 = vdc_vsc(2, 2, 1.0);
  p2.coordination_group_id = "grp";
  p2.participation_factor = 0.4;  // exactly one master, factors sum to 1.0
  sys.vsc_converters = {p1, p2};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK_FALSE(report_has_rule(report, "DCISLAND-MS-01"));
  CHECK_FALSE(report_has_rule(report, "DCISLAND-PARTICIPATION-01"));
  CHECK_FALSE(report_has_rule(report, "DCISLAND-PARTICIPATION-02"));
  CHECK(report.feasible);
}

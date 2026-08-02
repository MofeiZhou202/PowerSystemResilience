#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_models/ac_pf_model_builder.hpp"
#include "hacdcpf/power_models/hybrid_opf_model_builder.hpp"

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

TEST_CASE("Distributed slack honors converter coordination blocking diagnostics",
          "[converter][coordination][distributed_slack]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  hacdcpf::ACBus ac;
  ac.index = 1;
  ac.bus_type = hacdcpf::BusType::SLACK;
  ac.vm_pu = 1.0;
  ac.in_service = true;
  sys.ac.buses = {ac};

  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  gen.in_service = true;
  gen.pmax_mw = 100.0;
  gen.pmin_mw = -100.0;
  sys.ac.generators = {gen};

  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 2, 1)};

  hacdcpf::DCLoad load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 0.25;
  load.in_service = true;
  sys.dc.loads = {load};

  hacdcpf::PVArrayDC pv;
  pv.index = 1;
  pv.bus = 2;
  pv.p_set_mw = 0.30;
  pv.in_service = true;
  sys.dc.pv_arrays = {pv};

  hacdcpf::VSCConverter conv;
  conv.index = 1;
  conv.bus_ac = 1;
  conv.bus_dc = 1;
  conv.control_mode = hacdcpf::ConverterMode::AC_PV;
  conv.v_ac_set_pu = 1.0;
  conv.p_set_mw = 0.05;
  conv.in_service = true;
  sys.vsc_converters = {conv};

  hacdcpf::DistributedSlack slack;
  slack.participating_buses = {1};
  slack.participation_factors = {1.0};
  slack.reference_bus = 1;

  hacdcpf::PowerFlowOptions opt;
  opt.enable_converter_coordination_check = true;

  const hacdcpf::DistributedSlackResult result =
      hacdcpf::solve_power_flow_distributed_slack_full(sys, slack, opt);

  CHECK_FALSE(result.converged);
  CHECK(result.vm.empty());
  CHECK(result.vdc.empty());
  CHECK(result.diagnostics.converter_coordination.enabled);
  CHECK_FALSE(result.diagnostics.converter_coordination.feasible);
  CHECK(result.diagnostics.converter_coordination.has_blocking_issue());
  CHECK(report_has_rule(result.diagnostics.converter_coordination, "DCISLAND-01"));
}

TEST_CASE("Dedicated DC storage participates in coordination island summaries",
          "[converter][coordination][dc_storage]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  sys.dc.branches = {dc_branch(1, 1, 2)};

  hacdcpf::DCLoad load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 0.25;
  load.scaling = 1.0;
  load.in_service = true;
  sys.dc.loads = {load};

  hacdcpf::PVArrayDC pv;
  pv.index = 1;
  pv.bus = 2;
  pv.p_set_mw = 0.30;
  pv.in_service = true;
  sys.dc.pv_arrays = {pv};

  hacdcpf::DCStorage st;
  st.index = 1;
  st.bus = 2;
  st.p_mw = 0.30;
  st.p_rated_mw = 10.0;
  st.pmax_mw = 10.0;
  st.pmin_mw = -10.0;
  st.soc_init = 0.5;
  st.soc_min = 0.1;
  st.soc_max = 0.9;
  st.controllable = true;
  st.in_service = true;
  sys.dc.dc_storage = {st};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  REQUIRE(report.dc_islands.size() == 1);
  CHECK(report.dc_islands[0].fixed_power_devices == 3);
  CHECK(std::abs(report.dc_islands[0].fixed_power_mw - 0.35) < 1e-12);
  CHECK(std::abs(report.dc_islands[0].flexible_up_mw - 9.70) < 1e-12);
  CHECK(std::abs(report.dc_islands[0].flexible_down_mw - 10.30) < 1e-12);
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

TEST_CASE("DC_V bus backed by controllable DC storage is accepted as a physical voltage source",
          "[converter][coordination][dc-bus]") {
  hacdcpf::HybridPowerSystem sys;
  hacdcpf::DCBus d;
  d.index = 2;
  d.bus_type = hacdcpf::DCBusType::DC_V;
  d.vm_pu = 1.0;
  d.in_service = true;
  sys.dc.buses = {d};

  hacdcpf::DCLoad ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 0.25;
  ld.in_service = true;
  sys.dc.loads = {ld};

  hacdcpf::DCStorage st;
  st.index = 1;
  st.bus = 2;
  st.p_mw = 0.3;
  st.p_rated_mw = 0.3;
  st.pmin_mw = -0.3;
  st.pmax_mw = 0.3;
  st.soc_init = 0.5;
  st.soc_min = 0.1;
  st.soc_max = 0.9;
  st.controllable = true;
  st.in_service = true;
  sys.dc.dc_storage = {st};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  REQUIRE(report.dc_islands.size() == 1);
  CHECK(report.dc_islands.front().declared_v_buses == std::vector<int>{2});
  CHECK(report.dc_islands.front().hard_vdc_sources == 1);
  REQUIRE(report.dc_islands.front().voltage_sources.size() == 1);
  CHECK(report.dc_islands.front().voltage_sources.front().component_type ==
        "dc_bus_storage_source");
  CHECK_FALSE(report_has_rule(report, "DCBUS-01"));
  CHECK_FALSE(report_has_rule(report, "DCISLAND-01"));
  CHECK(report.feasible);
}

TEST_CASE("VDC setpoint conflicts are scoped to the same DC bus",
          "[converter][coordination][vsc]") {
  SECTION("different buses may have different voltages across line resistance") {
    hacdcpf::HybridPowerSystem sys;
    sys.dc.buses = {dc_bus(1), dc_bus(2)};
    sys.dc.branches = {dc_branch(1, 1, 2)};
    sys.vsc_converters = {vdc_vsc(1, 1, 1.00), vdc_vsc(2, 2, 1.05)};

    const auto report =
        hacdcpf::powerflow::evaluate_converter_coordination(sys, true);
    REQUIRE(report.dc_islands.size() == 1);
    CHECK(report.dc_islands.front().droop_sources == 2);
    CHECK_FALSE(report_has_rule(report, "DCISLAND-05"));
    CHECK(report.feasible);
  }

  SECTION("two incompatible controls on one bus remain fatal") {
    hacdcpf::HybridPowerSystem sys;
    sys.dc.buses = {dc_bus(1)};
    sys.vsc_converters = {vdc_vsc(1, 1, 1.00), vdc_vsc(2, 1, 1.05)};

    const auto report =
        hacdcpf::powerflow::evaluate_converter_coordination(sys, true);
    REQUIRE(report.dc_islands.size() == 1);
    CHECK(report_has_rule(report, "DCISLAND-05"));
    CHECK_FALSE(report.feasible);
  }
}

TEST_CASE("DC/DC Voltage mode forms its output voltage and is feasible with an input reference",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  // Input island (bus 1) has a VDC VSC reference; the Voltage-mode DC/DC forms
  // the output island (bus 2) voltage and draws its regulated power from bus 1.
  sys.vsc_converters = {vdc_vsc(5, 1, 1.0)};
  sys.dc.dcdc_converters = {dcdc_converter(3, 1, 2, hacdcpf::DCDCControlMode::Voltage)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  // Voltage mode is now a true voltage-forming control: no longer blocked.
  CHECK_FALSE(report_has_rule(report, "DCDC-CTRL-01"));
  CHECK_FALSE(report_has_rule(report, "DCDC-CTRL-03"));  // input island has a reference
  CHECK(report.feasible);
  // The output island gained a hard Vdc reference contributed by the former.
  bool out_has_dcdc_source = false;
  for (const auto& isl : report.dc_islands) {
    for (const auto& vs : isl.voltage_sources) {
      if (vs.component_type == "dcdc_converter") out_has_dcdc_source = true;
    }
  }
  CHECK(out_has_dcdc_source);
}

TEST_CASE("DC/DC Voltage mode needs an input-side voltage reference (DCDC-CTRL-03)",
          "[converter][coordination][dcdc]") {
  hacdcpf::HybridPowerSystem sys;
  sys.dc.buses = {dc_bus(1), dc_bus(2)};
  // Voltage-mode DC/DC forms the output (bus 2) voltage, but the input island
  // (bus 1) has no source to supply the regulated output.
  sys.dc.dcdc_converters = {dcdc_converter(3, 1, 2, hacdcpf::DCDCControlMode::Voltage)};

  const auto report = hacdcpf::powerflow::evaluate_converter_coordination(sys, true);

  CHECK(report_has_rule(report, "DCDC-CTRL-03"));
  CHECK_FALSE(report.feasible);
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

  // Per-unit port voltages must be compared on their physical kV bases. A
  // 1.0 pu -> 1.0 pu Buck from 1.5 kV to 0.4 kV is a 0.2667 duty, not 1.0.
  c.vn_in_kv = 1.5;
  c.vn_out_kv = 0.4;
  const auto different_bases = hacdcpf::powerflow::dcdc_duty_ratio(c, 1.0, 1.0);
  CHECK(different_bases.defined);
  CHECK(different_bases.feasible);
  CHECK(std::abs(different_bases.duty - (0.4 / 1.5)) < 1e-9);

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

// ── Unified converter-model fixes (multi-converter model) ────────────────────

TEST_CASE("VSC r_conv_ac_pu couples AC conduction loss into the DC injection",
          "[converter][model][r_conv_ac]") {
  using hacdcpf::powerflow::converter_dc_injection;
  using hacdcpf::powerflow::converter_dc_jacobian_vm_ac;

  hacdcpf::VSCConverter conv;
  conv.index = 0;
  conv.bus_ac = 1;
  conv.bus_dc = 1;
  conv.control_mode = hacdcpf::ConverterMode::PQ_MODE;
  conv.p_set_mw = 50.0;
  conv.eta = 1.0;  // isolate the AC-conduction term from switching loss
  conv.in_service = true;

  const double base_mva = 100.0;
  Eigen::VectorXd vm(1), va(1), vdc(1);
  vm[0] = 0.95;
  va[0] = 0.0;
  vdc[0] = 1.0;

  // r_conv_ac_pu = 0 -> legacy behaviour: DC injection is just -pset (eta = 1).
  conv.r_conv_ac_pu = 0.0;
  const double pdc0 =
      converter_dc_injection(conv, vm, va, vdc, base_mva, hacdcpf::LossModelType::Linear);
  CHECK(std::abs(pdc0 - (-conv.p_set_mw / base_mva)) < 1e-12);

  // r_conv_ac_pu > 0 -> the DC bus additionally supplies r*pac^2 / Vm_AC^2.
  conv.r_conv_ac_pu = 0.05;
  const double pac = conv.p_set_mw / base_mva;
  const double expect_loss = conv.r_conv_ac_pu * pac * pac / (vm[0] * vm[0]);
  const double pdc1 =
      converter_dc_injection(conv, vm, va, vdc, base_mva, hacdcpf::LossModelType::Linear);
  CHECK(std::abs(pdc1 - (pdc0 - expect_loss)) < 1e-12);
  CHECK(pdc1 < pdc0);  // strictly more negative: extra power drawn from DC

  // The cross-block Jacobian matches d(pdc)/d(Vm_AC) = 2*r*pac^2 / Vm_AC^3.
  const auto jac = converter_dc_jacobian_vm_ac(conv, vm, pac, base_mva);
  const double expect_dvm =
      2.0 * conv.r_conv_ac_pu * pac * pac / (vm[0] * vm[0] * vm[0]);
  CHECK(std::abs(jac.dpdc_dvm_ac - expect_dvm) < 1e-12);
}

TEST_CASE("Power-flow result declares its converter model scope",
          "[converter][scope]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(pf.converged);

  const auto& sc = pf.converter_model_scope;
  CHECK(sc.model_scope == "steady-state-newton:vsc-3mode+dcdc-power-transfer");
  // Default compatibility mode reports physical-limit violations as warnings;
  // strict acceptance enforcement is opt-in.
  CHECK(sc.validity.vsc_loss_modelled);
  CHECK(sc.validity.vsc_vdc_control_modelled);
  CHECK(sc.validity.vsc_ac_conduction_loss_modelled);
  CHECK(sc.validity.dc_multisource_coordination_modelled);
  CHECK_FALSE(sc.validity.vsc_modulation_limits_enforced);
  CHECK_FALSE(sc.validity.vsc_current_limits_enforced);
}

TEST_CASE("Tight VSC DC-current limit raises an ACDC-PHYS-03 diagnostic",
          "[converter][limits]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  REQUIRE_FALSE(sys.vsc_converters.empty());

  // Baseline: no limits set -> no ACDC-PHYS-03 warning.
  const hacdcpf::PowerFlowResult base = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(base.converged);
  auto has_phys03 = [](const hacdcpf::PowerFlowResult& pf) {
    const auto& w = pf.diagnostics.warnings;
    return std::any_of(w.begin(), w.end(), [](const std::string& s) {
      return s.find("[ACDC-PHYS-03]") != std::string::npos;
    });
  };
  CHECK_FALSE(has_phys03(base));

  // Impose an unsatisfiably small DC current limit -> the post-solve check fires.
  for (auto& c : sys.vsc_converters) c.i_dc_max_pu = 1e-6;
  const hacdcpf::PowerFlowResult limited = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(limited.converged);  // diagnostic only, does not block convergence
  CHECK(has_phys03(limited));

  hacdcpf::PowerFlowOptions strict_options;
  strict_options.enforce_converter_physical_limits = true;
  const hacdcpf::PowerFlowResult strict =
      hacdcpf::solve_power_flow(sys, strict_options);
  CHECK_FALSE(strict.converged);
  CHECK(has_phys03(strict));
  CHECK(strict.diagnostics.termination_reason ==
        "Converter physical-limit feasibility check failed");
  CHECK(strict.converter_model_scope.validity.vsc_capacity_circle_enforced);
  CHECK(strict.converter_model_scope.validity.vsc_current_limits_enforced);
  CHECK(strict.converter_model_scope.validity.vsc_modulation_limits_enforced);
  CHECK(strict.converter_model_scope.validity.dcdc_duty_ratio_enforced);
}

TEST_CASE("Power-flow reports a structural equation-closure check",
          "[converter][closure]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(pf.converged);

  // The hybrid Newton system is square (n_eq == n_var) and structurally full
  // rank (no empty Jacobian row/column) for a well-posed case.
  CHECK(pf.diagnostics.equation_closure_checked);
  CHECK(pf.diagnostics.equation_closure_ok);
  CHECK(pf.diagnostics.empty_jacobian_rows == 0);
  CHECK(pf.diagnostics.empty_jacobian_cols == 0);
  CHECK(pf.diagnostics.n_variables == pf.diagnostics.n_equations);
  CHECK(pf.diagnostics.n_variables > 0);
  CHECK(pf.converter_model_scope.validity.equation_closure_checked);
}

TEST_CASE("A droop DC/DC forms its output DC island without merging islands",
          "[converter][island][dcdc]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);

  // Add a second DC bus fed ONLY through a droop DC/DC from DC bus 1 (no DC
  // branch links them).  DC/DC converters are power-coupling interfaces and must
  // not merge DC voltage islands (multi-converter model §5.2): bus 2 becomes its
  // own island, referenced by the droop DC/DC output port.
  hacdcpf::DCBus d2;
  d2.index = 2;
  d2.bus_type = hacdcpf::DCBusType::DC_P;
  d2.vm_pu = 1.0;
  d2.vmin_pu = 0.9;
  d2.vmax_pu = 1.1;
  d2.in_service = true;
  sys.dc.buses.push_back(d2);

  hacdcpf::DCDCConverter dd;
  dd.index = 0;
  dd.bus_in = 1;
  dd.bus_out = 2;
  dd.control_mode = hacdcpf::DCDCControlMode::Droop;
  dd.k_droop = 0.1;
  dd.v_ref_pu = 1.0;
  dd.p_ref_mw = 0.01;
  dd.eta = 0.98;
  dd.in_service = true;
  sys.dc.dcdc_converters.push_back(dd);

  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(pf.converged);

  // The droop DC/DC forms bus-2's island, so the planner must NOT fall back to a
  // non-physical pinned reference, and the system stays structurally full rank.
  const auto& w = pf.diagnostics.warnings;
  CHECK(std::none_of(w.begin(), w.end(), [](const std::string& s) {
    return s.find("non-physical reference") != std::string::npos;
  }));
  CHECK(pf.diagnostics.equation_closure_ok);
  CHECK(pf.diagnostics.empty_jacobian_cols == 0);
}

TEST_CASE("A declared master converter forms the Vdc reference over a larger one",
          "[converter][multisource]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  REQUIRE(sys.vsc_converters.size() == 1);

  // Converter 0: small rating, declared master.  Converter 1: larger rating, not
  // master, same DC bus.  Without the master flag the planner promotes the
  // larger one; with it, the declared master must form the reference.
  sys.vsc_converters[0].is_master = true;
  sys.vsc_converters[0].p_rated_mw = 0.1;
  hacdcpf::VSCConverter big = sys.vsc_converters[0];
  big.index = 1;
  big.name = "big";
  big.is_master = false;
  big.p_rated_mw = 1.0;
  big.p_set_mw = 0.0;
  big.control_mode = hacdcpf::ConverterMode::PQ_MODE;
  sys.vsc_converters.push_back(big);

  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(pf.converged);
  const auto& prom = pf.diagnostics.promoted_vsc_indices;
  CHECK(std::find(prom.begin(), prom.end(), 0) != prom.end());   // master promoted
  CHECK(std::find(prom.begin(), prom.end(), 1) == prom.end());   // larger one not
}

TEST_CASE("Participation factors reshape multi-source droop sharing",
          "[converter][multisource]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  REQUIRE(sys.vsc_converters.size() == 1);

  // Two Vdc droop converters on the same DC island, grouped with 0.7 / 0.3
  // participation factors.  Their effective droop stiffness must end up in the
  // 0.7 : 0.3 ratio (total preserved), so the island imbalance is shared per the
  // declared factors.
  auto& a = sys.vsc_converters[0];
  a.control_mode = hacdcpf::ConverterMode::VDC_Q;
  a.k_vdc = 1.0;
  a.coordination_group_id = "grp";
  a.participation_factor = 0.7;
  hacdcpf::VSCConverter b = a;
  b.index = 1;
  b.name = "b";
  b.participation_factor = 0.3;
  sys.vsc_converters.push_back(b);

  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys, {});
  REQUIRE(pf.converged);
  REQUIRE(pf.diagnostics.effective_converters.size() == 2);
  const double k0 = pf.diagnostics.effective_converters[0].k_vdc;
  const double k1 = pf.diagnostics.effective_converters[1].k_vdc;
  REQUIRE(k0 + k1 > 1e-9);
  CHECK(std::abs(k0 / (k0 + k1) - 0.7) < 1e-6);
  CHECK(std::abs(k1 / (k0 + k1) - 0.3) < 1e-6);
}

TEST_CASE("VSC transfer results report original AC bus after projection renumbering",
          "[converter][projection][powerflow]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus b1, b2, b3, b4;
  b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0; b1.in_service = true;
  b2.index = 2; b2.bus_type = BusType::PQ; b2.in_service = true;
  b3.index = 3; b3.bus_type = BusType::PQ; b3.in_service = true;  // removed as dead island
  b4.index = 4; b4.bus_type = BusType::PQ; b4.in_service = true;
  sys.ac.buses = {b1, b2, b3, b4};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.vg_pu = 1.0;
  g.pmax_mw = 500.0;
  g.pmin_mw = -500.0;
  g.qmax_mvar = 500.0;
  g.qmin_mvar = -500.0;
  g.in_service = true;
  sys.ac.generators = {g};

  ACBranch br12, br33, br14;
  br12.index = 1; br12.from_bus = 1; br12.to_bus = 2; br12.r_pu = 0.01; br12.x_pu = 0.05;
  br12.in_service = true;
  br33.index = 2; br33.from_bus = 3; br33.to_bus = 3; br33.r_pu = 0.01; br33.x_pu = 0.05;
  br33.in_service = true;
  br14.index = 3; br14.from_bus = 1; br14.to_bus = 4; br14.r_pu = 0.01; br14.x_pu = 0.05;
  br14.in_service = true;
  sys.ac.branches = {br12, br33, br14};

  DCBus d;
  d.index = 1;
  d.bus_type = DCBusType::DC_P;
  d.vm_pu = 1.0;
  d.vmin_pu = 0.8;
  d.vmax_pu = 1.2;
  d.in_service = true;
  sys.dc.buses = {d};

  DCLoad dl;
  dl.index = 1;
  dl.bus = 1;
  dl.p_mw = 1.0;
  dl.in_service = true;
  sys.dc.loads = {dl};

  VSCConverter v;
  v.index = 0;
  v.bus_ac = 4;
  v.bus_dc = 1;
  v.control_mode = ConverterMode::VDC_Q;
  v.k_vdc = 0.5;
  v.v_dc_set_pu = 1.0;
  v.pmax_mw = 100.0;
  v.pmin_mw = -100.0;
  v.p_rated_mw = 100.0;
  v.in_service = true;
  sys.vsc_converters = {v};

  PowerFlowOptions opt;
  const PowerFlowResult r = solve_power_flow(sys, opt);
  REQUIRE(r.converged);
  REQUIRE(r.vm.size() == 4);
  REQUIRE(r.branch_flows.size() == 3);
  CHECK(std::abs(r.branch_flows[1].pf_mw) < 1e-12);
  CHECK(std::abs(r.branch_flows[1].pt_mw) < 1e-12);
  CHECK(std::abs(r.branch_flows[2].pf_mw) > 1e-6);

  REQUIRE(r.vsc_transfers.size() == 1);
  CHECK(r.vsc_transfers[0].bus_ac == 4);
  REQUIRE(r.diagnostics.effective_converters.size() == 1);
  CHECK(r.diagnostics.effective_converters[0].bus_ac == 4);
}

namespace {
// Minimal hybrid AC/DC OPF fixture: AC slack generator feeds a DC load through a
// single VSC.  Hybrid cases route to the parity-IPM formulation.
hacdcpf::HybridPowerSystem build_hybrid_opf_case() {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;

  ACBus b;
  b.index = 1;
  b.bus_type = BusType::SLACK;
  b.vm_pu = 1.0;
  b.vmin_pu = 0.9;
  b.vmax_pu = 1.1;
  b.in_service = true;
  sys.ac.buses = {b};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.vg_pu = 1.0;
  g.pmax_mw = 100.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.ac.generators = {g};

  DCBus d;
  d.index = 1;
  d.bus_type = DCBusType::DC_P;
  d.vm_pu = 1.0;
  d.vmin_pu = 0.9;
  d.vmax_pu = 1.1;
  d.in_service = true;
  sys.dc.buses = {d};

  DCLoad dl;
  dl.index = 0;
  dl.bus = 1;
  dl.p_mw = 2.0;  // served via the converter (AC -> DC)
  dl.in_service = true;
  sys.dc.loads = {dl};

  VSCConverter v;
  v.index = 0;
  v.bus_ac = 1;
  v.bus_dc = 1;
  v.control_mode = ConverterMode::PQ_MODE;
  v.pmax_mw = 10.0;
  v.pmin_mw = -10.0;
  v.qmax_mvar = 10.0;
  v.qmin_mvar = -10.0;
  v.p_rated_mw = 10.0;
  v.eta = 0.98;
  v.in_service = true;
  sys.vsc_converters = {v};
  return sys;
}

double converter_iac_pu(const hacdcpf::opf::ACOPFResult& r, double base_mva) {
  return std::hypot(r.pac_mw[0] / base_mva, r.qac_mvar[0] / base_mva) /
         std::max(r.vm[0], 1e-6);
}
}  // namespace

TEST_CASE("Hybrid AC-OPF enforces the converter AC current limit (parity KKT)",
          "[converter][opf][limits]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();

  opf::ACOPFOptions opt;
  opt.verbose = false;

  const opf::ACOPFResult base = opf::solve_ac_opf(sys, opt);
  REQUIRE(base.converged);
  // The hybrid (parity-IPM) path now declares it enforces converter current limits.
  CHECK(base.converter_model_scope.validity.vsc_current_limits_enforced);
  REQUIRE(base.pac_mw.size() == 1);
  const double sb = sys.ac.base_mva;
  const double iac0 = converter_iac_pu(base, sb);
  REQUIRE(iac0 > 0.05);

  // Non-binding limit (above natural current): still feasible, and respected.
  sys.vsc_converters[0].i_ac_max_pu = iac0 * 1.5;
  const opf::ACOPFResult loose = opf::solve_ac_opf(sys, opt);
  REQUIRE(loose.converged);
  CHECK(converter_iac_pu(loose, sb) <= sys.vsc_converters[0].i_ac_max_pu + 1e-3);

  // Tight limit (below natural current): the IPM must NOT return a converged
  // point that violates the constraint — either it respects the limit or reports
  // infeasibility.  This is the enforcement guarantee.
  sys.vsc_converters[0].i_ac_max_pu = iac0 * 0.5;
  const opf::ACOPFResult tight = opf::solve_ac_opf(sys, opt);
  if (tight.converged) {
    CHECK(converter_iac_pu(tight, sb) <= sys.vsc_converters[0].i_ac_max_pu + 5e-3);
  }
}

TEST_CASE("Hybrid AC-OPF enforces the converter modulation limit (parity KKT)",
          "[converter][opf][limits]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  // m = Vac·Vn_ac / (Km·Vdc·Vn_dc); unit bases keep m ≈ Vac/Vdc.
  sys.vsc_converters[0].vn_ac_kv = 1.0;
  sys.vsc_converters[0].vn_dc_kv = 1.0;
  sys.vsc_converters[0].k_m_modulation = 1.0;

  opf::ACOPFOptions opt;
  opt.verbose = false;

  const opf::ACOPFResult base = opf::solve_ac_opf(sys, opt);
  REQUIRE(base.converged);
  CHECK(base.converter_model_scope.validity.vsc_modulation_limits_enforced);
  REQUIRE(!base.vdc.empty());
  auto modulation = [](const opf::ACOPFResult& r) {
    return (r.vm[0] * 1.0) / (1.0 * r.vdc[0] * 1.0);
  };
  const double m0 = modulation(base);
  REQUIRE(m0 > 0.5);

  // Upper modulation bound below the natural value: the IPM must not return a
  // converged point that exceeds it.
  sys.vsc_converters[0].m_max = m0 * 0.97;
  const opf::ACOPFResult tight = opf::solve_ac_opf(sys, opt);
  if (tight.converged) {
    CHECK(modulation(tight) <= sys.vsc_converters[0].m_max + 5e-3);
  }
}

namespace {
// Hybrid AC/DC OPF fixture exercising the DC/DC duty-ratio rows: the AC slack
// feeds DC bus 1 through a VDC-controlled VSC, and a Buck DC/DC steps DC bus 1
// down to DC bus 2, which now carries the DC load.
hacdcpf::HybridPowerSystem build_dcdc_opf_case() {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  sys.vsc_converters[0].control_mode = ConverterMode::VDC_Q;
  sys.vsc_converters[0].v_dc_set_pu = 1.0;
  sys.dc.buses[0].vmin_pu = 0.5;
  sys.dc.buses[0].vmax_pu = 1.2;

  DCBus d2;
  d2.index = 2;
  d2.bus_type = DCBusType::DC_P;
  d2.vm_pu = 0.8;
  d2.vmin_pu = 0.5;
  d2.vmax_pu = 1.2;
  d2.in_service = true;
  sys.dc.buses.push_back(d2);
  sys.dc.loads[0].bus = 2;  // served through the DC/DC

  DCDCConverter dc;
  dc.index = 0;
  dc.bus_in = 1;
  dc.bus_out = 2;
  dc.in_service = true;
  dc.control_mode = DCDCControlMode::Voltage;
  dc.v_ref_pu = 0.8;
  dc.sn_mva = 10.0;
  dc.eta = 1.0;
  dc.pmax_mw = 10.0;
  dc.pmin_mw = -10.0;
  dc.topology = DCDCTopology::Buck;  // Vout = D·Vin
  dc.d_min = 0.05;
  dc.d_max = 0.95;
  dc.n_ratio = 1.0;
  sys.dc.dcdc_converters = {dc};
  return sys;
}

// Buck duty backed out from the solved DC port voltages (D = Vout / Vin).
double dcdc_buck_duty(const hacdcpf::opf::ACOPFResult& r) {
  if (r.vdc.size() < 2 || r.vdc[0] < 1e-6) return 0.0;
  return r.vdc[1] / r.vdc[0];
}
}  // namespace

TEST_CASE("Hybrid AC-OPF enforces the DC/DC duty-ratio limit (parity KKT)",
          "[converter][opf][limits][dcdc]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_dcdc_opf_case();

  opf::ACOPFOptions opt;
  opt.verbose = false;

  const opf::ACOPFResult base = opf::solve_ac_opf(sys, opt);
  REQUIRE(base.converged);
  // The hybrid (parity-IPM) path declares it enforces DC/DC duty feasibility.
  CHECK(base.converter_model_scope.validity.dcdc_duty_ratio_enforced);
  REQUIRE(base.vdc.size() >= 2);
  const double d0 = dcdc_buck_duty(base);
  // The solved duty must lie inside the declared [d_min, d_max] window.
  CHECK(d0 >= sys.dc.dcdc_converters[0].d_min - 1e-3);
  CHECK(d0 <= sys.dc.dcdc_converters[0].d_max + 1e-3);

  // Disabling modulation enforcement clears the DC/DC duty scope flag.
  opt.enforce_converter_modulation_limits = false;
  const opf::ACOPFResult off = opf::solve_ac_opf(sys, opt);
  CHECK_FALSE(off.converter_model_scope.validity.dcdc_duty_ratio_enforced);

  // A tight upper duty bound below the natural value must not yield a converged
  // point that violates it (respect-or-infeasible guarantee, as for VSC limits).
  opt.enforce_converter_modulation_limits = true;
  sys.dc.dcdc_converters[0].d_max = std::min(0.9, d0 * 0.9);
  const opf::ACOPFResult tight = opf::solve_ac_opf(sys, opt);
  if (tight.converged && tight.vdc.size() >= 2) {
    CHECK(dcdc_buck_duty(tight) <= sys.dc.dcdc_converters[0].d_max + 5e-3);
  }
}

TEST_CASE("AC island without an angle reference is flagged (ACISLAND-REF-01)",
          "[converter][coordination][acisland]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  // Two AC buses joined by a branch, carrying load but with NO slack bus,
  // external grid, or AC grid-forming converter → no AC angle reference.
  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::PQ;
  b1.pd_mw = 5.0;
  b1.in_service = true;
  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.pd_mw = 3.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};
  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.x_pu = 0.1;
  br.in_service = true;
  sys.ac.branches = {br};

  const auto report = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(report, "ACISLAND-REF-01"));
  // ACISLAND-REF-01 is diagnostic only and must not block the solve.
  CHECK(report.feasible);

  // Promoting one bus to SLACK provides the angle reference → no warning.
  sys.ac.buses[0].bus_type = BusType::SLACK;
  const auto report2 = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(report2, "ACISLAND-REF-01"));
}

TEST_CASE("AC/DC converter cannot grid-form on both sides (ACDC-GFM-03)",
          "[converter][coordination][acgfm]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  // DC-side grid-forming (VDC control with droop) AND AC-side grid-forming.
  sys.vsc_converters[0].control_mode = ConverterMode::VDC_VAC;
  sys.vsc_converters[0].k_vdc = 0.1;
  sys.vsc_converters[0].ac_grid_forming = true;

  const auto report = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(report, "ACDC-GFM-03"));
  CHECK_FALSE(report.feasible);

  // The advanced dual-side case is allowed only with an explicit energy buffer.
  sys.vsc_converters[0].allow_dual_side_grid_forming = true;
  sys.vsc_converters[0].has_energy_buffer = true;
  const auto report2 = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(report2, "ACDC-GFM-03"));
}

TEST_CASE("AC grid-forming converter cannot hard-pin active power (ACDC-GFM-04)",
          "[converter][coordination][acgfm]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  sys.vsc_converters[0].control_mode = ConverterMode::PQ_MODE;  // not DC grid-forming
  sys.vsc_converters[0].ac_grid_forming = true;
  sys.vsc_converters[0].p_is_hard_constraint = true;

  const auto report = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(report, "ACDC-GFM-04"));
}

TEST_CASE("AC_PV converter holds its AC terminal voltage (power flow)",
          "[converter][acpv][powerflow]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;
  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.pd_mw = 5.0;
  b2.qd_mvar = 1.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.vg_pu = 1.0;
  g.pmax_mw = 500.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 500.0;
  g.qmin_mvar = -500.0;
  g.in_service = true;
  sys.ac.generators = {g};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.in_service = true;
  sys.ac.branches = {br};

  DCBus d;
  d.index = 1;
  d.bus_type = DCBusType::DC_P;
  d.vm_pu = 1.0;
  d.vmin_pu = 0.8;
  d.vmax_pu = 1.2;
  d.in_service = true;
  sys.dc.buses = {d};

  // VSC A forms the DC voltage on DC bus 1 (VDC droop).
  VSCConverter vA;
  vA.index = 0;
  vA.bus_ac = 1;
  vA.bus_dc = 1;
  vA.control_mode = ConverterMode::VDC_VAC;
  vA.k_vdc = 0.5;
  vA.v_dc_set_pu = 1.0;
  vA.pmax_mw = 100.0;
  vA.pmin_mw = -100.0;
  vA.p_rated_mw = 100.0;
  vA.eta = 0.99;
  vA.in_service = true;

  // VSC B is AC_PV: it holds Vac at bus 2 = 1.03 pu and injects Pac = 8 MW,
  // releasing its reactive power as the free balancing injection of that PV bus.
  VSCConverter vB;
  vB.index = 1;
  vB.bus_ac = 2;
  vB.bus_dc = 1;
  vB.control_mode = ConverterMode::AC_PV;
  vB.p_set_mw = 8.0;
  vB.v_ac_set_pu = 1.03;
  vB.pmax_mw = 100.0;
  vB.pmin_mw = -100.0;
  vB.p_rated_mw = 100.0;
  vB.eta = 0.99;
  vB.in_service = true;
  sys.vsc_converters = {vA, vB};

  PowerFlowOptions opt;
  const PowerFlowResult r = solve_power_flow(sys, opt);
  REQUIRE(r.converged);
  REQUIRE(r.vm.size() >= 2);
  // The AC_PV converter must hold its AC terminal (bus 2 → index 1) at its
  // voltage setpoint, with reactive power free.
  CHECK(std::abs(r.vm[1] - 1.03) < 2e-3);

  // It releases reactive power: the reported Qac is the bus's balancing reactive
  // (non-zero, since holding 1.03 pu against the load/branch needs reactive
  // support), not the placeholder q_set_mvar (0).
  const auto it = std::find_if(r.vsc_transfers.begin(), r.vsc_transfers.end(),
                               [](const VSCTransfer& t) { return t.index == 1; });
  REQUIRE(it != r.vsc_transfers.end());
  CHECK(std::isfinite(it->q_ac_mvar));
  CHECK(std::abs(it->q_ac_mvar) > 1e-3);
}

TEST_CASE("AC_PV mode coordination rules (ACDC-CTRL-03/04)",
          "[converter][coordination][acpv]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  auto& conv = sys.vsc_converters[0];
  conv.control_mode = ConverterMode::AC_PV;
  conv.v_ac_set_pu = 1.02;
  conv.q_set_mvar = 0.0;

  // Valid AC_PV: a positive voltage setpoint, no imposed reactive power.
  const auto ok = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(ok, "ACDC-CTRL-03"));
  CHECK_FALSE(report_has_rule(ok, "ACDC-CTRL-04"));

  // Missing voltage setpoint → ACDC-CTRL-03.
  conv.v_ac_set_pu = 0.0;
  const auto noV = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(noV, "ACDC-CTRL-03"));

  // Imposed reactive power in AC_PV → ACDC-CTRL-04 (released / ignored).
  conv.v_ac_set_pu = 1.02;
  conv.q_set_mvar = 5.0;
  const auto withQ = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(withQ, "ACDC-CTRL-04"));
}

// ── Seven-mode taxonomy: DeviceControlRole resolution ────────────────────────
TEST_CASE("resolve_device_control_role maps the seven VSC control modes",
          "[converter][role][sevenmode]") {
  using namespace hacdcpf;

  auto role_of = [](ConverterMode m) {
    VSCConverter c;
    c.control_mode = m;
    c.k_vdc = 0.1;  // active droop so DC-forming modes register
    return resolve_device_control_role(c);
  };

  // Mode 3 (AC_PQ): holds AC P and Q.
  const auto pq = role_of(ConverterMode::PQ_MODE);
  CHECK(pq.acdc_mode == ACDCControlMode::AC_PQ);
  CHECK(pq.controls_ac_p);
  CHECK(pq.controls_ac_q);
  CHECK_FALSE(pq.controls_ac_v);

  // Mode 2 (AC_PV): holds AC P and V, releases Q, provides a voltage reference.
  const auto pv = role_of(ConverterMode::AC_PV);
  CHECK(pv.acdc_mode == ACDCControlMode::AC_PV);
  CHECK(pv.controls_ac_p);
  CHECK(pv.controls_ac_v);
  CHECK(pv.ac_q_is_free);
  CHECK(pv.provides_ac_voltage_reference);
  CHECK_FALSE(pv.controls_ac_angle);

  // Mode 7 (DC_V_DROOP_AC_Q via VDC_Q): droop Udc + Q, releases AC P.
  const auto vdcq = role_of(ConverterMode::VDC_Q);
  CHECK(vdcq.acdc_mode == ACDCControlMode::DC_V_DROOP_AC_Q);
  CHECK(vdcq.controls_dc_v_droop);
  CHECK(vdcq.controls_ac_q);
  CHECK(vdcq.ac_p_is_free);
  CHECK(vdcq.is_dc_grid_forming);

  // Mode 6 (DC_V_DROOP_AC_V): droop Udc + Vs, releases AC P and Q, provides both
  // a DC voltage reference and an AC voltage reference.
  const auto m6 = role_of(ConverterMode::DC_V_DROOP_AC_V);
  CHECK(m6.acdc_mode == ACDCControlMode::DC_V_DROOP_AC_V);
  CHECK(m6.controls_dc_v_droop);
  CHECK(m6.controls_ac_v);
  CHECK(m6.ac_p_is_free);
  CHECK(m6.ac_q_is_free);
  CHECK(m6.provides_dc_v_reference);
  CHECK(m6.provides_ac_voltage_reference);
  CHECK(m6.is_dc_grid_forming);

  // Mode 1 (AC_GRID_FORMING): forms the AC reference (angle + magnitude).
  const auto m1 = role_of(ConverterMode::AC_GRID_FORMING);
  CHECK(m1.acdc_mode == ACDCControlMode::AC_GRID_FORMING);
  CHECK(m1.controls_ac_angle);
  CHECK(m1.controls_ac_v);
  CHECK(m1.is_ac_grid_forming);
  CHECK(m1.provides_ac_angle_reference);
  CHECK(m1.provides_ac_voltage_reference);
  CHECK(m1.ac_p_is_free);

  // String round-trip for the two new ConverterMode values.
  CHECK(converter_mode_from_str("AC_GRID_FORMING") == ConverterMode::AC_GRID_FORMING);
  CHECK(converter_mode_from_str("grid forming") == ConverterMode::AC_GRID_FORMING);
  CHECK(converter_mode_from_str("DC_V_AC_Q") == ConverterMode::VDC_Q);
  CHECK(converter_mode_from_str("dc-v-ac-v") == ConverterMode::VDC_VAC);
  CHECK(converter_mode_from_str("DC_V_DROOP_AC_V") == ConverterMode::DC_V_DROOP_AC_V);
  CHECK(converter_mode_str(ConverterMode::AC_GRID_FORMING) == "AC_GRID_FORMING");
  CHECK(converter_mode_str(ConverterMode::DC_V_DROOP_AC_V) == "DC_V_DROOP_AC_V");
  CHECK(dcdc_control_from_str("P") == DCDCControlMode::Power);
  CHECK(dcdc_control_from_str("constant power") == DCDCControlMode::Power);
  CHECK(dcdc_control_from_str("D") == DCDCControlMode::Droop);
  CHECK(dcdc_control_from_str("dc-v") == DCDCControlMode::Voltage);
  CHECK(dcdc_topology_from_str("buck-boost") == DCDCTopology::BuckBoost);
  CHECK(dcdc_topology_from_str("DAB") == DCDCTopology::Isolated);
}

// ── PF/OPF/transient consistency: the OPF DC-slack selection must agree with the
// shared control-role resolver (which the PF coordination, short-circuit and
// transient builders use). Previously the OPF builder only recognized VDC_Q as
// DC-voltage-forming, diverging from PF/transient for VDC_VAC / DC_V_DROOP_AC_V.
TEST_CASE("OPF DC-slack selection is consistent with the control-role resolver",
          "[converter][opf][role][consistency]") {
  using namespace hacdcpf;
  const std::vector<ConverterMode> modes = {
      ConverterMode::PQ_MODE,          ConverterMode::AC_PV,
      ConverterMode::VDC_Q,            ConverterMode::VDC_VAC,
      ConverterMode::DC_V_DROOP_AC_V,  ConverterMode::AC_GRID_FORMING};
  for (ConverterMode m : modes) {
    HybridPowerSystem sys = build_hybrid_opf_case();
    sys.vsc_converters[0].control_mode = m;
    sys.vsc_converters[0].v_dc_set_pu = 1.0;
    sys.vsc_converters[0].k_vdc = 0.1;  // active droop
    const bool expected =
        resolve_device_control_role(sys.vsc_converters[0]).provides_dc_v_reference;
    const auto data = power_models::to_acdcopf_data(sys);
    REQUIRE(data.converters.size() == 1);
    INFO("mode=" << converter_mode_str(m));
    // The converter and its DC bus carry the Vdc-slack flag iff the resolver says
    // the converter provides a DC voltage reference.
    CHECK(data.converters[0].is_vdc_slack == expected);
    bool bus_slack = false;
    for (const auto& bd : data.dc_buses)
      if (bd.id == data.converters[0].dc_bus_id) bus_slack = bd.is_vdc_slack;
    CHECK(bus_slack == expected);
  }

  // Regression guard for the specific fix: VDC_VAC and DC_V_DROOP_AC_V are now
  // recognized as DC-voltage-forming by the OPF builder (were previously missed).
  for (ConverterMode m : {ConverterMode::VDC_VAC, ConverterMode::DC_V_DROOP_AC_V}) {
    HybridPowerSystem sys = build_hybrid_opf_case();
    sys.vsc_converters[0].control_mode = m;
    const auto data = power_models::to_acdcopf_data(sys);
    INFO("mode=" << converter_mode_str(m));
    REQUIRE(data.converters.size() == 1);
    CHECK(data.converters[0].is_vdc_slack);
  }
}

TEST_CASE("Experimental AML OPF builders reject in-service LCC coupling",
          "[converter][opf][lcc][scope]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  LCCConverter lcc;
  lcc.index = 91;
  lcc.ac_bus = sys.ac.buses.front().index;
  lcc.dc_bus = sys.dc.buses.front().index;
  lcc.in_service = true;
  sys.lcc_converters.push_back(lcc);

  bool ac_rejected = false;
  try {
    (void)power_models::to_acopf_data(sys);
  } catch (const std::invalid_argument& error) {
    ac_rejected = std::string(error.what()).find("does not model LCC") !=
                  std::string::npos;
  }
  CHECK(ac_rejected);

  bool acdc_rejected = false;
  try {
    (void)power_models::to_acdcopf_data(sys);
  } catch (const std::invalid_argument& error) {
    acdc_rejected = std::string(error.what()).find("does not model LCC") !=
                    std::string::npos;
  }
  CHECK(acdc_rejected);

  sys.lcc_converters.front().in_service = false;
  CHECK_NOTHROW(power_models::to_acopf_data(sys));
  CHECK_NOTHROW(power_models::to_acdcopf_data(sys));
}

TEST_CASE("OPF AC reference matches the resolver for a grid-forming converter",
          "[converter][opf][role][consistency][acref]") {
  using namespace hacdcpf;

  // A DC-fed AC island: two PQ buses, NEITHER a slack, so the only possible AC
  // angle reference is the grid-forming converter at bus 1.
  auto build_conv_fed_island = []() {
    HybridPowerSystem sys;
    sys.base_mva = 10.0;
    sys.ac.base_mva = 10.0;
    ACBus b1; b1.index = 1; b1.bus_type = BusType::PQ; b1.vm_pu = 1.0;
    b1.vmin_pu = 0.9; b1.vmax_pu = 1.1; b1.in_service = true;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.vm_pu = 1.0;
    b2.vmin_pu = 0.9; b2.vmax_pu = 1.1; b2.pd_mw = 1.0; b2.in_service = true;
    sys.ac.buses = {b1, b2};
    ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.01;
    br.x_pu = 0.05; br.in_service = true;
    sys.ac.branches = {br};
    DCBus d; d.index = 1; d.bus_type = DCBusType::DC_V; d.vm_pu = 1.0;
    d.vmin_pu = 0.9; d.vmax_pu = 1.1; d.in_service = true;
    sys.dc.buses = {d};
    StaticGeneratorDC dg; dg.index = 1; dg.bus = 1; dg.p_set_mw = 1.5;
    dg.pmax_mw = 3.0; dg.in_service = true;
    sys.dc.dc_static_generators = {dg};
    VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 1;
    v.control_mode = ConverterMode::AC_GRID_FORMING; v.v_ac_set_pu = 1.02;
    v.v_ac_angle_set_deg = 0.0; v.pmax_mw = 10; v.pmin_mw = -10;
    v.qmax_mvar = 10; v.qmin_mvar = -10; v.p_rated_mw = 10; v.eta = 0.98;
    v.in_service = true;
    sys.vsc_converters = {v};
    return sys;
  };
  auto is_ref = [](const power_models::ACDCOPFData& data, const std::string& id) {
    for (const auto& b : data.ac.buses)
      if (b.id == id) return b.is_ref;
    return false;
  };
  auto count_ref = [](const power_models::ACDCOPFData& data) {
    int n = 0;
    for (const auto& b : data.ac.buses)
      if (b.is_ref) ++n;
    return n;
  };

  // 1) AC_GRID_FORMING mode: the resolver forms the AC reference, and (with no
  //    slack generator present) the OPF must anchor the converter's AC bus —
  //    matching what the power flow's apply_acpv_voltage_control does.
  {
    HybridPowerSystem sys = build_conv_fed_island();
    REQUIRE(resolve_device_control_role(sys.vsc_converters[0]).is_ac_grid_forming);
    const auto data = power_models::to_acdcopf_data(sys);
    CHECK(is_ref(data, "B1"));
    CHECK(count_ref(data) == 1);
  }

  // 2) The ac_grid_forming opt-in flag alone (control_mode left at PQ) also makes
  //    the converter form the AC reference — PF and OPF both honor the flag now.
  {
    HybridPowerSystem sys = build_conv_fed_island();
    sys.vsc_converters[0].control_mode = ConverterMode::PQ_MODE;
    sys.vsc_converters[0].ac_grid_forming = true;
    REQUIRE(resolve_device_control_role(sys.vsc_converters[0]).is_ac_grid_forming);
    const auto data = power_models::to_acdcopf_data(sys);
    CHECK(is_ref(data, "B1"));
    CHECK(count_ref(data) == 1);
  }

  // 3) An existing slack bus wins: the converter never pins a second AC angle
  //    reference in the same network (mirrors the power flow's "SLACK wins").
  {
    HybridPowerSystem sys = build_conv_fed_island();
    sys.ac.buses[1].bus_type = BusType::SLACK;  // bus 2 is now the slack
    const auto data = power_models::to_acdcopf_data(sys);
    CHECK(count_ref(data) == 1);
    CHECK(is_ref(data, "B2"));
    CHECK_FALSE(is_ref(data, "B1"));
  }
}

TEST_CASE("Hybrid AC/DC OPF models DC/DC converters in all three control modes",
          "[converter][opf][dcdc]") {
  using namespace hacdcpf;
  // AC slack -> VSC(VDC_Q) forms DC bus 1; DC bus 2 (load) is reached by BOTH a
  // DC line and the DC/DC converter, so Vdc2 is always well-determined.
  auto build_case = [](DCDCControlMode mode, double p_ref_mw, double v_ref_pu,
                       double k_droop, double load_mw) {
    HybridPowerSystem sys;
    sys.base_mva = 10.0;
    sys.ac.base_mva = 10.0;
    ACBus b; b.index = 1; b.bus_type = BusType::SLACK; b.vm_pu = 1.0;
    b.vmin_pu = 0.9; b.vmax_pu = 1.1; b.in_service = true;
    sys.ac.buses = {b};
    Generator g; g.index = 1; g.bus = 1; g.in_service = true; g.is_slack = true;
    g.vg_pu = 1.0; g.pmax_mw = 100; g.pmin_mw = 0; g.qmax_mvar = 100;
    g.qmin_mvar = -100; g.cost_c1 = 10.0;
    sys.ac.generators = {g};
    DCBus d1; d1.index = 1; d1.bus_type = DCBusType::DC_V; d1.vm_pu = 1.0;
    d1.vmin_pu = 0.9; d1.vmax_pu = 1.1; d1.in_service = true;
    DCBus d2; d2.index = 2; d2.bus_type = DCBusType::DC_P; d2.vm_pu = 1.0;
    d2.vmin_pu = 0.9; d2.vmax_pu = 1.1; d2.in_service = true;
    sys.dc.buses = {d1, d2};
    DCBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.05;
    br.in_service = true;
    sys.dc.branches = {br};
    DCLoad l; l.index = 1; l.bus = 2; l.p_mw = load_mw; l.in_service = true;
    sys.dc.loads = {l};
    VSCConverter v; v.index = 1; v.bus_ac = 1; v.bus_dc = 1;
    v.control_mode = ConverterMode::VDC_Q; v.v_dc_set_pu = 1.0; v.k_vdc = 0.1;
    v.pmax_mw = 50; v.pmin_mw = -50; v.qmax_mvar = 50; v.qmin_mvar = -50;
    v.p_rated_mw = 50; v.eta = 0.99; v.in_service = true;
    sys.vsc_converters = {v};
    DCDCConverter dc; dc.index = 1; dc.bus_in = 1; dc.bus_out = 2;
    dc.control_mode = mode; dc.p_ref_mw = p_ref_mw; dc.v_ref_pu = v_ref_pu;
    dc.k_droop = k_droop; dc.eta = 0.97; dc.pmax_mw = 20; dc.pmin_mw = -20;
    dc.r_eq_pu = 0.01;
    dc.in_service = true;
    sys.dc.dcdc_converters = {dc};
    return sys;
  };

  // The data builder captures the DC/DC converter (previously ignored entirely).
  {
    auto sys = build_case(DCDCControlMode::Voltage, 0.0, 1.03, 0.0, 1.5);
    const auto data = power_models::to_acdcopf_data(sys);
    REQUIRE(data.dcdc_converters.size() == 1);
    CHECK(data.dcdc_converters[0].in_bus_id == "DC1");
    CHECK(data.dcdc_converters[0].out_bus_id == "DC2");
    CHECK(data.dcdc_converters[0].forms_out_voltage);
    CHECK(data.dcdc_converters[0].r_eq_pu == Catch::Approx(0.01));
  }

  // Voltage mode: the DC/DC holds its output-bus voltage at v_ref.
  {
    auto sys = build_case(DCDCControlMode::Voltage, 0.0, 1.03, 0.0, 1.5);
    const auto res = power_models::solve_acdcopf(power_models::to_acdcopf_data(sys));
    REQUIRE(res.ac_result.solve_result.has_primal());
    REQUIRE(res.vdc_pu.count("DC2") == 1);
    CHECK(res.vdc_pu.at("DC2") == Catch::Approx(1.03).margin(1e-3));
  }

  // Power mode: the DC/DC output power equals its setpoint.
  {
    const double p_ref = 1.2;
    auto sys = build_case(DCDCControlMode::Power, p_ref, 1.0, 0.0, 1.5);
    const auto res = power_models::solve_acdcopf(power_models::to_acdcopf_data(sys));
    REQUIRE(res.ac_result.solve_result.has_primal());
    REQUIRE(res.pdcdc_mw.count("DCDC1") == 1);
    CHECK(res.pdcdc_mw.at("DCDC1") == Catch::Approx(p_ref).margin(1e-2));
    CHECK(res.max_dc_p_viol_pu < 1e-4);
  }

  // Droop mode: Pout = p_ref - k_droop·(Vdc_out − v_ref)  (negative feedback).
  {
    const double p_ref = 1.0, k = 2.0, v_ref = 1.0, Sb = 10.0;
    auto sys = build_case(DCDCControlMode::Droop, p_ref, v_ref, k, 1.5);
    const auto res = power_models::solve_acdcopf(power_models::to_acdcopf_data(sys));
    REQUIRE(res.ac_result.solve_result.has_primal());
    REQUIRE(res.pdcdc_mw.count("DCDC1") == 1);
    REQUIRE(res.vdc_pu.count("DC2") == 1);
    const double vdc2 = res.vdc_pu.at("DC2");
    const double expected_mw = p_ref - k * (vdc2 - v_ref) * Sb;
    CHECK(res.pdcdc_mw.at("DCDC1") == Catch::Approx(expected_mw).margin(2e-2));
  }
}

// ── Mode 6: droop Udc + AC voltage hold (genuine power flow) ──────────────────
TEST_CASE("DC_V_DROOP_AC_V converter forms Vdc by droop and holds its AC voltage",
          "[converter][mode6][powerflow]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus b1;  // main AC grid slack
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;
  ACBus b2;  // converter AC terminal + load
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.pd_mw = 5.0;
  b2.qd_mvar = 1.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.vg_pu = 1.0;
  g.pmax_mw = 500.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 500.0;
  g.qmin_mvar = -500.0;
  g.in_service = true;
  sys.ac.generators = {g};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.in_service = true;
  sys.ac.branches = {br};

  DCBus d;
  d.index = 1;
  d.bus_type = DCBusType::DC_P;
  d.vm_pu = 1.0;
  d.vmin_pu = 0.8;
  d.vmax_pu = 1.2;
  d.in_service = true;
  sys.dc.buses = {d};

  DCLoad dl;
  dl.index = 0;
  dl.bus = 1;
  dl.p_mw = 3.0;  // served by the converter from the AC side
  dl.in_service = true;
  sys.dc.loads = {dl};

  // A single converter that forms Vdc by droop AND holds its AC terminal voltage
  // (Mode 6).  It draws active power from the AC grid (slack at bus 1) to supply
  // the DC load, while holding Vac at bus 2 with free reactive power.
  VSCConverter v;
  v.index = 0;
  v.bus_ac = 2;
  v.bus_dc = 1;
  v.control_mode = ConverterMode::DC_V_DROOP_AC_V;
  v.k_vdc = 0.5;
  v.v_dc_set_pu = 1.0;
  v.v_ac_set_pu = 1.04;
  v.pmax_mw = 100.0;
  v.pmin_mw = -100.0;
  v.p_rated_mw = 100.0;
  v.eta = 0.99;
  v.in_service = true;
  sys.vsc_converters = {v};

  PowerFlowOptions opt;
  const PowerFlowResult r = solve_power_flow(sys, opt);
  REQUIRE(r.converged);
  REQUIRE(r.vm.size() >= 2);
  // Mode 6 holds the AC terminal voltage magnitude (bus 2 → index 1).
  CHECK(std::abs(r.vm[1] - 1.04) < 2e-3);
  // The DC voltage stays in a sensible band around its droop setpoint.
  REQUIRE(!r.vdc.empty());
  CHECK(r.vdc[0] > 0.8);
  CHECK(r.vdc[0] < 1.2);
  // Reactive power is released as the free balancing injection of the PV bus.
  const auto it = std::find_if(r.vsc_transfers.begin(), r.vsc_transfers.end(),
                               [](const VSCTransfer& t) { return t.index == 0; });
  REQUIRE(it != r.vsc_transfers.end());
  CHECK(std::isfinite(it->q_ac_mvar));
}

// ── Mode 1: AC grid-forming (genuine AC reference in the power flow) ──────────
TEST_CASE("AC_GRID_FORMING converter forms the islanded AC voltage and angle reference",
          "[converter][mode1][powerflow][math_audit][A1][D6]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  // Two AC areas linked only through a DC tie: the main grid (bus 1 slack, bus 2
  // = VSC A AC terminal) and an islanded microgrid (bus 3 = VSC B AC terminal,
  // bus 4 = microgrid load).  VSC B forms the microgrid's AC reference, so the
  // solver treats bus 3 as a secondary slack for that island.
  ACBus b1, b2, b3, b4;
  b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0; b1.in_service = true;
  b2.index = 2; b2.bus_type = BusType::PQ; b2.in_service = true;
  b3.index = 3; b3.bus_type = BusType::PQ; b3.in_service = true;
  b4.index = 4; b4.bus_type = BusType::PQ; b4.pd_mw = 6.0; b4.qd_mvar = 2.0;
  b4.in_service = true;
  sys.ac.buses = {b1, b2, b3, b4};

  Generator g;
  g.index = 1; g.bus = 1; g.is_slack = true; g.vg_pu = 1.0;
  g.pmax_mw = 500.0; g.pmin_mw = 0.0; g.qmax_mvar = 500.0; g.qmin_mvar = -500.0;
  g.in_service = true;
  sys.ac.generators = {g};

  ACBranch br12, br34;
  br12.index = 1; br12.from_bus = 1; br12.to_bus = 2; br12.r_pu = 0.01; br12.x_pu = 0.05;
  br12.in_service = true;
  br34.index = 2; br34.from_bus = 3; br34.to_bus = 4; br34.r_pu = 0.01; br34.x_pu = 0.05;
  br34.in_service = true;
  sys.ac.branches = {br12, br34};

  DCBus d1, d2;
  d1.index = 1; d1.bus_type = DCBusType::DC_P; d1.vm_pu = 1.0;
  d1.vmin_pu = 0.7; d1.vmax_pu = 1.3; d1.in_service = true;
  d2.index = 2; d2.bus_type = DCBusType::DC_P; d2.vm_pu = 1.0;
  d2.vmin_pu = 0.7; d2.vmax_pu = 1.3; d2.in_service = true;
  sys.dc.buses = {d1, d2};

  DCBranch dbr;
  dbr.index = 1; dbr.from_bus = 1; dbr.to_bus = 2; dbr.r_pu = 0.01; dbr.in_service = true;
  sys.dc.branches = {dbr};

  // VSC A rectifies the main grid into the DC tie and forms the DC voltage.
  VSCConverter vA;
  vA.index = 0; vA.bus_ac = 2; vA.bus_dc = 1;
  vA.control_mode = ConverterMode::VDC_Q;
  vA.k_vdc = 0.6; vA.v_dc_set_pu = 1.0;
  vA.pmax_mw = 200.0; vA.pmin_mw = -200.0; vA.p_rated_mw = 200.0; vA.eta = 0.99;
  vA.in_service = true;

  // VSC B inverts the DC tie into the islanded microgrid, forming its AC voltage
  // and angle reference (Mode 1).  Its DC island has VSC A as the Vdc reference,
  // satisfying ACDC-GFM-05.  p_set_mw is deliberately set far from the true
  // delivered power (the ~6 MW microgrid load) to prove the energy-conduit
  // coupling: the converter draws its ACTUAL AC slack power from the DC side, not
  // the (wrong) schedule.
  VSCConverter vB;
  vB.index = 1; vB.bus_ac = 3; vB.bus_dc = 2;
  vB.control_mode = ConverterMode::AC_GRID_FORMING;
  vB.v_ac_set_pu = 1.03;
  vB.v_ac_angle_set_deg = 3.0;
  vB.p_set_mw = 1.0;  // deliberately wrong (true draw ≈ 6 MW) — coupling overrides it
  vB.pmax_mw = 200.0; vB.pmin_mw = -200.0; vB.p_rated_mw = 200.0; vB.eta = 0.99;
  vB.in_service = true;
  sys.vsc_converters = {vA, vB};

  PowerFlowOptions opt;
  const PowerFlowResult r = solve_power_flow(sys, opt);
  REQUIRE(r.converged);
  REQUIRE(r.vm.size() >= 4);
  // Bus 3 (index 2) is the AC grid-forming terminal: it holds both the voltage
  // magnitude and the reference angle (3 deg → radians in the result).
  CHECK(std::abs(r.vm[2] - 1.03) < 3e-3);
  CHECK(std::abs(r.va[2] - 3.0 * 3.14159265358979 / 180.0) < 5e-3);
  // The microgrid load bus stays within a sensible band, energized by the former.
  CHECK(r.vm[3] > 0.9);

  // Energy-conduit coupling: the AC grid-forming converter injects ~+6 MW into
  // the microgrid (the load) and draws ~−6 MW from the DC side — tracking its
  // ACTUAL AC slack power, NOT the deliberately-wrong 1 MW schedule.
  const auto itB = std::find_if(r.vsc_transfers.begin(), r.vsc_transfers.end(),
                                [](const VSCTransfer& t) { return t.index == 1; });
  REQUIRE(itB != r.vsc_transfers.end());
  CHECK(itB->p_ac_mw > 5.9);   // supplies the 6 MW microgrid load (+ losses)
  CHECK(itB->p_dc_mw < -5.9);  // draws it from the DC link, not the 1 MW schedule
  // Per-converter energy balance holds: P_ac + P_dc + loss = 0.
  CHECK(std::abs(itB->p_ac_mw + itB->p_dc_mw + itB->loss_mw) < 1e-6);

  // A1 regression: validate the solved DC state and the upstream converter,
  // not only the post-processed GFM report. With 6 MW crossing a 0.01 pu DC
  // line the receiving voltage must move away from the flat 1.0 pu start, and
  // the upstream converter must actually supply the transfer.
  REQUIRE(r.vdc.size() == 2);
  CHECK(r.vdc[1] < 0.9999);
  const auto itA = std::find_if(r.vsc_transfers.begin(), r.vsc_transfers.end(),
                                [](const VSCTransfer& t) { return t.index == 0; });
  REQUIRE(itA != r.vsc_transfers.end());
  CHECK(std::abs(itA->p_ac_mw) > 5.9);

  // Coordination: no GFM-05 (DC support present) and no blocking issues.
  const auto report = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(report, "ACDC-GFM-05"));
  CHECK_FALSE(report.has_blocking_issue());

  // D6 regression: a fixed co-located injection belongs to that component,
  // not to the free-power GFM converter. Because bus 3 is a reference, adding
  // fixed Q there does not change the solved network voltage; it must reduce
  // only the converter's attributed Q by exactly the same amount.
  StaticGenerator co_located;
  co_located.index = 7;
  co_located.bus = 3;
  co_located.q_mvar = 1.5;
  co_located.in_service = true;
  sys.ac.static_generators.push_back(co_located);
  const PowerFlowResult with_co_located = solve_power_flow(sys, opt);
  REQUIRE(with_co_located.converged);
  const auto itB_fixed = std::find_if(
      with_co_located.vsc_transfers.begin(),
      with_co_located.vsc_transfers.end(),
      [](const VSCTransfer& transfer) { return transfer.index == 1; });
  REQUIRE(itB_fixed != with_co_located.vsc_transfers.end());
  CHECK(itB_fixed->q_ac_mvar == Catch::Approx(itB->q_ac_mvar - 1.5).margin(1e-8));
}

// ── ACDC-GFM-05: AC grid-forming needs DC-side support ───────────────────────
TEST_CASE("AC grid-forming converter without DC-side support raises ACDC-GFM-05",
          "[converter][coordination][gfm05]") {
  using namespace hacdcpf;
  HybridPowerSystem sys = build_hybrid_opf_case();
  // The lone converter forms the AC reference but its DC island (a single DC_P
  // bus with a fixed DC load) has no voltage reference, DC source, or buffer.
  auto& conv = sys.vsc_converters[0];
  conv.control_mode = ConverterMode::AC_GRID_FORMING;
  conv.v_ac_set_pu = 1.02;
  conv.has_energy_buffer = false;

  const auto bad = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(bad, "ACDC-GFM-05"));

  // Declaring an energy buffer on the converter satisfies the DC-side support
  // requirement and clears the rule.
  conv.has_energy_buffer = true;
  const auto ok = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(ok, "ACDC-GFM-05"));
}

// ── ACDC-GFM-06: DC grid-forming needs AC-side support ───────────────────────
TEST_CASE("DC grid-forming converter without AC-side support raises ACDC-GFM-06",
          "[converter][coordination][gfm06]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  // An AC island with only a load (no slack, generator, or external grid) cannot
  // supply the active power a DC-forming converter must exchange to hold Vdc.
  ACBus a;
  a.index = 1; a.bus_type = BusType::PQ; a.pd_mw = 4.0; a.in_service = true;
  sys.ac.buses = {a};

  DCBus d;
  d.index = 1; d.bus_type = DCBusType::DC_P; d.vm_pu = 1.0;
  d.vmin_pu = 0.8; d.vmax_pu = 1.2; d.in_service = true;
  sys.dc.buses = {d};

  VSCConverter v;
  v.index = 0; v.bus_ac = 1; v.bus_dc = 1;
  v.control_mode = ConverterMode::VDC_Q;  // DC voltage forming via droop
  v.k_vdc = 0.5; v.v_dc_set_pu = 1.0;
  v.pmax_mw = 100.0; v.pmin_mw = -100.0; v.p_rated_mw = 100.0;
  v.in_service = true;
  sys.vsc_converters = {v};

  const auto bad = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(bad, "ACDC-GFM-06"));

  // Adding an AC slack generator gives the island an active-power source.
  Generator g;
  g.index = 1; g.bus = 1; g.is_slack = true; g.vg_pu = 1.0;
  g.pmax_mw = 100.0; g.pmin_mw = 0.0; g.qmax_mvar = 100.0; g.qmin_mvar = -100.0;
  g.in_service = true;
  sys.ac.buses[0].bus_type = BusType::SLACK;
  sys.ac.generators = {g};

  const auto ok = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(ok, "ACDC-GFM-06"));
}

// ── ACISLAND-BALANCE-01: energized AC island needs an active-power source ────
TEST_CASE("Energized AC island with no active-power source raises ACISLAND-BALANCE-01",
          "[converter][coordination][acisland][balance]") {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  // Two AC buses with a load but no slack, generator, external grid, or
  // converter — the island is energized yet cannot balance active power.
  ACBus a, b;
  a.index = 1; a.bus_type = BusType::PQ; a.pd_mw = 3.0; a.in_service = true;
  b.index = 2; b.bus_type = BusType::PQ; b.in_service = true;
  sys.ac.buses = {a, b};

  ACBranch br;
  br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.01; br.x_pu = 0.05;
  br.in_service = true;
  sys.ac.branches = {br};

  const auto bad = powerflow::evaluate_converter_coordination(sys, true);
  CHECK(report_has_rule(bad, "ACISLAND-BALANCE-01"));

  // A generator gives the island an active-power balancing source.
  Generator g;
  g.index = 1; g.bus = 1; g.is_slack = true; g.vg_pu = 1.0;
  g.pmax_mw = 100.0; g.pmin_mw = 0.0; g.qmax_mvar = 100.0; g.qmin_mvar = -100.0;
  g.in_service = true;
  sys.ac.buses[0].bus_type = BusType::SLACK;
  sys.ac.generators = {g};

  const auto ok = powerflow::evaluate_converter_coordination(sys, true);
  CHECK_FALSE(report_has_rule(ok, "ACISLAND-BALANCE-01"));
}

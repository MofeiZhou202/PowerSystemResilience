#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
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
  // Honest fidelity declaration: losses + Vdc forming are modelled in the solve,
  // but modulation / current limits / multi-source coordination are not (yet).
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

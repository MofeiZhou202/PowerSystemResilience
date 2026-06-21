#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/analysis/distribution_power_flow.hpp"

namespace {

using hacdcpf::BusType;
using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::ThreePhaseRegulatorControl;
using hacdcpf::ThreePhaseTransformer;
using hacdcpf::analysis::ThreePhaseNROptions;
using hacdcpf::analysis::solve_three_phase_nr;

ThreePhaseACBus make_bus(
    int index,
    BusType type,
    PhaseMask mask,
    double base_kv = 7.2) {
  ThreePhaseACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.phase_mask = mask;
  bus.base_kv = base_kv;
  bus.in_service = true;
  bus.vm_a_pu = 1.0;
  bus.va_a_deg = 0.0;
  bus.vm_b_pu = 1.0;
  bus.va_b_deg = -120.0;
  bus.vm_c_pu = 1.0;
  bus.va_c_deg = 120.0;
  return bus;
}

ThreePhaseTransformer make_regulator_transformer(
    int index,
    int hv_bus,
    int lv_bus,
    PhaseMask mask) {
  ThreePhaseTransformer transformer;
  transformer.index = index;
  transformer.name = "reg" + std::to_string(index);
  transformer.hv_bus = hv_bus;
  transformer.lv_bus = lv_bus;
  transformer.in_service = true;
  transformer.hv_phase_mask = mask;
  transformer.lv_phase_mask = mask;
  transformer.sn_mva = 5.0;
  transformer.vn_hv_kv = 7.2;
  transformer.vn_lv_kv = 7.2;
  transformer.vk_percent = 1.0;
  transformer.vkr_percent = 0.1;
  transformer.vector_group = "YNyn0";
  transformer.tap_side = 1;
  transformer.tap_min = -16;
  transformer.tap_max = 16;
  transformer.tap_neutral = 0;
  transformer.tap_step_percent = 0.625;
  return transformer;
}

ThreePhaseACLine make_line(
    int index,
    int from_bus,
    int to_bus,
    PhaseMask mask,
    double r1_pu,
    double x1_pu) {
  ThreePhaseACLine line;
  line.index = index;
  line.from_bus = from_bus;
  line.to_bus = to_bus;
  line.phase_mask = mask;
  line.in_service = true;
  line.r1_pu = r1_pu;
  line.x1_pu = x1_pu;
  line.r0_pu = r1_pu;
  line.x0_pu = x1_pu;
  return line;
}

ThreePhaseRegulatorControl make_control(
    int index,
    int transformer_index,
    int monitored_bus,
    int monitored_node,
    double vreg_volts,
    double band_volts) {
  ThreePhaseRegulatorControl control;
  control.index = index;
  control.name = "control" + std::to_string(index);
  control.transformer_index = transformer_index;
  control.transformer_name = "reg" + std::to_string(transformer_index);
  control.winding = 2;
  control.tap_winding = 2;
  control.monitored_bus = monitored_bus;
  control.monitored_node = monitored_node;
  control.vreg_volts = vreg_volts;
  control.band_volts = band_volts;
  control.ptratio = 60.0;
  control.max_tap_change = 1;
  control.enabled = true;
  return control;
}

ThreePhaseNROptions nr_options() {
  ThreePhaseNROptions options;
  options.max_iter = 80;
  options.max_control_iter = 20;
  options.tol = 1e-10;
  return options;
}

const hacdcpf::analysis::ThreePhaseRegulatorControlState& get_state(
    const hacdcpf::analysis::ThreePhaseDPFResult& result,
    std::size_t pos) {
  REQUIRE(result.regulator_states.size() > pos);
  return result.regulator_states[pos];
}

const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation& get_observation(
    const hacdcpf::analysis::ThreePhaseDPFResult& result,
    std::size_t pos) {
  REQUIRE(result.transformer_terminal_observations.size() > pos);
  return result.transformer_terminal_observations[pos];
}

std::complex<double> phase_current_from_observation(
    const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation& observation,
    bool hv_side,
    int phase_index) {
  const auto& phasor = hv_side ? observation.hv_current_amps : observation.lv_current_amps;
  return {
      phasor.real[static_cast<std::size_t>(phase_index)],
      phasor.imag[static_cast<std::size_t>(phase_index)],
  };
}

ThreePhaseACSystem build_single_phase_raise_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 5.0;
  sys.name = "tp_nr_regulator_raise";
  sys.buses = {
      make_bus(1, BusType::SLACK, PhaseMask::a()),
      make_bus(2, BusType::PQ, PhaseMask::a()),
  };
  sys.transformers = {
      make_regulator_transformer(1, 1, 2, PhaseMask::a()),
  };
  sys.regulator_controls = {
      make_control(1, 1, 0, 1, 121.5, 1.0),
  };
  return sys;
}

ThreePhaseACSystem build_independent_bank_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 5.0;
  sys.name = "tp_nr_regulator_bank";
  sys.buses = {
      make_bus(1, BusType::SLACK, PhaseMask::abc()),
      make_bus(2, BusType::PQ, PhaseMask::abc()),
  };
  sys.transformers = {
      make_regulator_transformer(1, 1, 2, PhaseMask::a()),
      make_regulator_transformer(2, 1, 2, PhaseMask::b()),
      make_regulator_transformer(3, 1, 2, PhaseMask::c()),
  };
  sys.regulator_controls = {
      make_control(1, 1, 0, 1, 121.5, 1.0),
      make_control(2, 2, 0, 2, 120.75, 0.5),
      make_control(3, 3, 0, 3, 119.25, 0.5),
  };
  return sys;
}

ThreePhaseACSystem build_remote_ldc_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 5.0;
  sys.name = "tp_nr_regulator_remote_ldc";
  auto bus3 = make_bus(3, BusType::PQ, PhaseMask::a());
  bus3.pd_a_mw = 0.20;
  bus3.qd_a_mvar = 0.10;
  sys.buses = {
      make_bus(1, BusType::SLACK, PhaseMask::a()),
      make_bus(2, BusType::PQ, PhaseMask::a()),
      bus3,
  };
  sys.transformers = {
      make_regulator_transformer(1, 1, 2, PhaseMask::a()),
  };
  sys.lines = {
      make_line(1, 2, 3, PhaseMask::a(), 0.03, 0.06),
  };
  auto control = make_control(1, 1, 3, 1, 121.0, 1.0);
  control.ptratio = 55.0;
  control.remote_ptratio = 60.0;
  control.ct_primary_amps = 100.0;
  control.r_volts = 1.5;
  control.x_volts = 4.5;
  sys.regulator_controls = {control};
  return sys;
}

ThreePhaseACSystem build_tap_limit_case() {
  auto sys = build_single_phase_raise_case();
  sys.transformers[0].tap_max = 1;
  sys.regulator_controls[0].vreg_volts = 123.0;
  sys.regulator_controls[0].band_volts = 0.5;
  return sys;
}

ThreePhaseACSystem build_cycle_case() {
  auto sys = build_single_phase_raise_case();
  sys.transformers[0].tap_step_percent = 1.25;
  sys.regulator_controls[0].vreg_volts = 120.75;
  sys.regulator_controls[0].band_volts = 0.5;
  return sys;
}

ThreePhaseACSystem build_shared_tap_fail_close_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 5.0;
  sys.name = "tp_nr_regulator_shared_tap_fail_close";
  sys.buses = {
      make_bus(1, BusType::SLACK, PhaseMask::abc()),
      make_bus(2, BusType::PQ, PhaseMask::abc()),
  };
  sys.transformers = {
      make_regulator_transformer(1, 1, 2, PhaseMask::abc()),
  };
  sys.regulator_controls = {
      make_control(1, 1, 0, 1, 121.5, 1.0),
  };
  return sys;
}

}  // namespace

TEST_CASE("Three-phase NR regulator raises single-phase tap through the closed loop",
          "[integration][powerflow][three_phase_nr][regulator]") {
  const auto result = solve_three_phase_nr(build_single_phase_raise_case(), nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.control_converged);
  REQUIRE(result.regulator_trace.empty() == false);
  REQUIRE(result.transformer_terminal_observations.size() == 1);

  const auto& state = get_state(result, 0);
  CHECK(state.final_tap_pos == 2);
  CHECK(state.final_tap_number == 2);
  CHECK(std::abs(state.final_tap_pu - 1.0125) < 1e-9);
  CHECK(state.stop_reason == "within_band");
  CHECK(std::abs(result.bus_voltages[1].vm_a_pu - 1.0125) < 5e-4);
}

TEST_CASE("Three-phase NR regulator supports independent A/B/C bank behavior",
          "[integration][powerflow][three_phase_nr][regulator]") {
  const auto result = solve_three_phase_nr(build_independent_bank_case(), nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.control_converged);
  REQUIRE(result.regulator_states.size() == 3);

  CHECK(get_state(result, 0).final_tap_pos == 2);
  CHECK(get_state(result, 1).final_tap_pos == 1);
  CHECK(get_state(result, 2).final_tap_pos == -1);
  CHECK(result.bus_voltages[1].vm_a_pu > result.bus_voltages[1].vm_b_pu);
  CHECK(result.bus_voltages[1].vm_b_pu > result.bus_voltages[1].vm_c_pu);
}

TEST_CASE("Three-phase NR regulator uses transformer observation for remote PTRatio plus LDC",
          "[integration][powerflow][three_phase_nr][regulator][ldc]") {
  const auto result = solve_three_phase_nr(build_remote_ldc_case(), nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.control_converged);
  REQUIRE(result.transformer_terminal_observations.size() == 1);

  const auto& state = get_state(result, 0);
  const auto& observation = get_observation(result, 0);
  const std::complex<double> current_amps =
      phase_current_from_observation(observation, false, 0);
  const std::complex<double> expected_ldc =
      (current_amps / state.ct_primary_amps) *
      std::complex<double>(state.r_volts, state.x_volts);

  CHECK(state.used_remote_bus);
  CHECK(state.used_line_drop_compensation);
  CHECK(std::abs(state.remote_ptratio - 60.0) < 1e-12);
  CHECK(state.final_tap_pos > 0);
  CHECK(std::abs(state.line_drop_compensation_real_volts - expected_ldc.real()) < 1e-6);
  CHECK(std::abs(state.line_drop_compensation_imag_volts - expected_ldc.imag()) < 1e-6);
  CHECK(std::abs(state.line_drop_compensation_magnitude_volts - std::abs(expected_ldc)) < 1e-6);
  CHECK(std::abs(state.tap_winding_current_amps - std::abs(current_amps)) < 1e-6);
}

TEST_CASE("Three-phase NR regulator fail-closes tap limit, oscillation, and unsupported contracts",
          "[integration][powerflow][three_phase_nr][regulator][fail_close]") {
  SECTION("tap limit is reported through regulator state") {
    auto options = nr_options();
    options.max_control_iter = 6;
    const auto result = solve_three_phase_nr(build_tap_limit_case(), options);
    REQUIRE(result.control_converged == false);
    REQUIRE(result.converged == false);
    CHECK(get_state(result, 0).stop_reason == "tap_limit_reached_below_band");
    CHECK(result.regulator_trace.back().stop_reason == "tap_limit_reached_below_band");
  }

  SECTION("control oscillation is reported as cycle_detected") {
    auto options = nr_options();
    options.max_control_iter = 6;
    const auto result = solve_three_phase_nr(build_cycle_case(), options);
    REQUIRE(result.control_converged == false);
    REQUIRE(result.converged == false);
    REQUIRE(result.regulator_trace.size() >= 2);
    CHECK(get_state(result, 0).stop_reason == "cycle_detected");
    CHECK(result.regulator_trace.back().stop_reason == "cycle_detected");
  }

  SECTION("shared three-phase tap device remains unsupported") {
    const auto result = solve_three_phase_nr(build_shared_tap_fail_close_case(), nr_options());
    REQUIRE(result.control_converged == false);
    REQUIRE(result.converged == false);
    CHECK(get_state(result, 0).stop_reason == "regulator transformer 1 requires a single active phase");
  }
}

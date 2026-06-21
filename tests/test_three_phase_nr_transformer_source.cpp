#include <algorithm>
#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "hacdcpf/analysis/distribution_power_flow.hpp"

namespace {

using hacdcpf::BusType;
using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::ThreePhaseExternalGrid;
using hacdcpf::ThreePhaseTransformer;
using hacdcpf::analysis::ThreePhaseTransformerTerminalObservation;
using hacdcpf::analysis::ThreePhaseNROptions;
using hacdcpf::analysis::solve_three_phase_nr;

constexpr double kPi = 3.14159265358979323846;

ThreePhaseACBus make_bus(int index, BusType type) {
  ThreePhaseACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.phase_mask = PhaseMask::abc();
  bus.base_kv = 12.47;
  bus.in_service = true;
  bus.vm_a_pu = 1.0;
  bus.va_a_deg = 0.0;
  bus.vm_b_pu = 1.0;
  bus.va_b_deg = -120.0;
  bus.vm_c_pu = 1.0;
  bus.va_c_deg = 120.0;
  return bus;
}

ThreePhaseACLine make_line(
    int index,
    int from_bus,
    int to_bus,
    double r_pu,
    double x_pu) {
  ThreePhaseACLine line;
  line.index = index;
  line.from_bus = from_bus;
  line.to_bus = to_bus;
  line.phase_mask = PhaseMask::abc();
  line.in_service = true;
  line.r1_pu = r_pu;
  line.x1_pu = x_pu;
  line.r0_pu = 3.0 * r_pu;
  line.x0_pu = 3.0 * x_pu;
  line.rate_a_mva = 100.0;
  return line;
}

ThreePhaseExternalGrid make_source(
    int index,
    int bus,
    double vm_pu,
    double va_deg,
    double r1_pu,
    double x1_pu) {
  ThreePhaseExternalGrid source;
  source.index = index;
  source.bus = bus;
  source.in_service = true;
  source.phase_mask = PhaseMask::abc();
  source.vm_pu = vm_pu;
  source.va_deg = va_deg;
  source.r1_pu = r1_pu;
  source.x1_pu = x1_pu;
  source.r2_pu = r1_pu;
  source.x2_pu = x1_pu;
  source.r0_pu = 3.0 * r1_pu;
  source.x0_pu = 3.0 * x1_pu;
  return source;
}

ThreePhaseTransformer make_transformer(
    int index,
    const std::string& vector_group) {
  ThreePhaseTransformer transformer;
  transformer.index = index;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.in_service = true;
  transformer.hv_phase_mask = PhaseMask::abc();
  transformer.lv_phase_mask = PhaseMask::abc();
  transformer.sn_mva = 10.0;
  transformer.vn_hv_kv = 12.47;
  transformer.vn_lv_kv = 12.47;
  transformer.vk_percent = 8.0;
  transformer.vkr_percent = 0.5;
  transformer.vector_group = vector_group;
  return transformer;
}

ThreePhaseNROptions nr_options() {
  ThreePhaseNROptions opt;
  opt.max_iter = 60;
  opt.tol = 1e-9;
  return opt;
}

double wrapped_angle_error_deg(double actual_deg, double expected_deg) {
  double diff = std::fmod(actual_deg - expected_deg, 360.0);
  if (diff > 180.0) diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return std::abs(diff);
}

std::complex<double> phase_voltage(
    const hacdcpf::analysis::ThreePhaseBusVoltage& bus,
    int phase_index) {
  if (phase_index == 0) {
    return std::polar(bus.vm_a_pu, bus.va_a_deg * kPi / 180.0);
  }
  if (phase_index == 1) {
    return std::polar(bus.vm_b_pu, bus.va_b_deg * kPi / 180.0);
  }
  return std::polar(bus.vm_c_pu, bus.va_c_deg * kPi / 180.0);
}

std::complex<double> line_voltage(
    const hacdcpf::analysis::ThreePhaseBusVoltage& bus,
    int from_phase,
    int to_phase) {
  return phase_voltage(bus, from_phase) - phase_voltage(bus, to_phase);
}

double angle_deg(const std::complex<double>& value) {
  return std::arg(value) * 180.0 / kPi;
}

void set_source_slot_voltage(
    ThreePhaseExternalGrid& source,
    int slot,
    const std::complex<double>& value) {
  const double vm = std::abs(value);
  const double va = angle_deg(value);
  if (slot == 0) {
    source.vm_a_pu = vm;
    source.va_a_deg = va;
    return;
  }
  if (slot == 1) {
    source.vm_b_pu = vm;
    source.va_b_deg = va;
    return;
  }
  source.vm_c_pu = vm;
  source.va_c_deg = va;
}

void set_source_slot_impedance(
    ThreePhaseExternalGrid& source,
    int slot,
    double r_pu,
    double x_pu) {
  if (slot == 0) {
    source.r_a_pu = r_pu;
    source.x_a_pu = x_pu;
    return;
  }
  if (slot == 1) {
    source.r_b_pu = r_pu;
    source.x_b_pu = x_pu;
    return;
  }
  source.r_c_pu = r_pu;
  source.x_c_pu = x_pu;
}

double phasor_magnitude(double real, double imag) {
  return std::hypot(real, imag);
}

double terminal_phase_current_mag(
    const ThreePhaseTransformerTerminalObservation& observation,
    int phase_index,
    bool hv_side) {
  const auto& real = hv_side ? observation.hv_current_amps.real
                             : observation.lv_current_amps.real;
  const auto& imag = hv_side ? observation.hv_current_amps.imag
                             : observation.lv_current_amps.imag;
  return phasor_magnitude(real[phase_index], imag[phase_index]);
}

ThreePhaseACSystem build_grounded_wye_tap_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_grounded_wye_tap";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};

  auto transformer = make_transformer(1, "YNyn0");
  transformer.tap_side = 1;
  transformer.tap_pos = 4;
  transformer.tap_neutral = 0;
  transformer.tap_step_percent = 1.25;
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_ynyn6_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_ynyn6_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "YNyn6")};
  return sys;
}

ThreePhaseACSystem build_ynyn7_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_ynyn7_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "YNyn7")};
  return sys;
}

ThreePhaseACSystem build_ynyn0_zero_sequence_magnetizing_case(bool enable_mag0) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = enable_mag0 ? "tp_nr_ynyn0_zero_sequence_mag"
                         : "tp_nr_ynyn0_zero_sequence_open";

  auto slack = make_bus(1, BusType::SLACK);
  slack.va_b_deg = 0.0;
  slack.va_c_deg = 0.0;
  auto remote = make_bus(2, BusType::PQ);
  remote.va_b_deg = 0.0;
  remote.va_c_deg = 0.0;
  sys.buses = {slack, remote};

  auto transformer = make_transformer(1, "YNyn0");
  transformer.pfe_kw = 0.0;
  transformer.i0_percent = 0.0;
  transformer.vk0_percent = 10.0;
  transformer.vkr0_percent = 0.6;
  if (enable_mag0) {
    transformer.mag0_percent = 80.0;
    transformer.mag0_rx = 0.0;
    transformer.si0_hv_partial = 0.25;
  }
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_yny0_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_ynyn0_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_yny0_zero_sequence_mag"
                         : "tp_nr_yny0_zero_sequence_open";
  sys.transformers[0].vector_group = "YNy0";
  return sys;
}

ThreePhaseACSystem build_yy0_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_ynyn0_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_yy0_zero_sequence_mag"
                         : "tp_nr_yy0_zero_sequence_open";
  sys.transformers[0].vector_group = "Yy0";
  return sys;
}

ThreePhaseACSystem build_partial_ynyn0_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_ynyn0_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_partial_ynyn0_zero_sequence_mag"
                         : "tp_nr_partial_ynyn0_zero_sequence_open";
  sys.buses[0].phase_mask = PhaseMask::ab();
  sys.buses[1].phase_mask = PhaseMask::ab();
  sys.transformers[0].hv_phase_mask = PhaseMask::ab();
  sys.transformers[0].lv_phase_mask = PhaseMask::ab();
  return sys;
}

ThreePhaseACSystem build_ynd1_zero_sequence_magnetizing_case(bool enable_mag0) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = enable_mag0 ? "tp_nr_ynd1_zero_sequence_mag"
                         : "tp_nr_ynd1_zero_sequence_open";

  auto hv = make_bus(1, BusType::SLACK);
  hv.va_b_deg = 0.0;
  hv.va_c_deg = 0.0;
  auto lv = make_bus(2, BusType::PQ);
  lv.va_b_deg = 0.0;
  lv.va_c_deg = 0.0;
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "YNd1");
  transformer.pfe_kw = 0.0;
  transformer.i0_percent = 0.0;
  transformer.vk0_percent = 10.0;
  transformer.vkr0_percent = 0.6;
  if (enable_mag0) {
    transformer.mag0_percent = 80.0;
    transformer.mag0_rx = 0.0;
    transformer.si0_hv_partial = 0.25;
  }
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_yd1_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_ynd1_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_yd1_zero_sequence_mag"
                         : "tp_nr_yd1_zero_sequence_open";
  sys.transformers[0].vector_group = "Yd1";
  return sys;
}

ThreePhaseACSystem build_partial_ynd1_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_ynd1_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_partial_ynd1_zero_sequence_mag"
                         : "tp_nr_partial_ynd1_zero_sequence_open";
  sys.buses[0].phase_mask = PhaseMask::a();
  sys.buses[1].phase_mask = PhaseMask::ab();
  sys.transformers[0].hv_phase_mask = PhaseMask::a();
  sys.transformers[0].lv_phase_mask = PhaseMask::ab();
  return sys;
}

ThreePhaseACSystem build_dyn11_zero_sequence_magnetizing_case(bool enable_mag0) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = enable_mag0 ? "tp_nr_dyn11_zero_sequence_mag"
                         : "tp_nr_dyn11_zero_sequence_open";

  auto hv = make_bus(1, BusType::PQ);
  hv.va_b_deg = 0.0;
  hv.va_c_deg = 0.0;
  auto lv = make_bus(2, BusType::SLACK);
  lv.va_b_deg = 0.0;
  lv.va_c_deg = 0.0;
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Dyn11");
  transformer.pfe_kw = 0.0;
  transformer.i0_percent = 0.0;
  transformer.vk0_percent = 10.0;
  transformer.vkr0_percent = 0.6;
  if (enable_mag0) {
    transformer.mag0_percent = 80.0;
    transformer.mag0_rx = 0.0;
    transformer.si0_hv_partial = 0.25;
  }
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_partial_dyn11_zero_sequence_magnetizing_case(bool enable_mag0) {
  auto sys = build_dyn11_zero_sequence_magnetizing_case(enable_mag0);
  sys.name = enable_mag0 ? "tp_nr_partial_dyn11_zero_sequence_mag"
                         : "tp_nr_partial_dyn11_zero_sequence_open";
  sys.buses[0].phase_mask = PhaseMask::ab();
  sys.buses[1].phase_mask = PhaseMask::a();
  sys.transformers[0].hv_phase_mask = PhaseMask::ab();
  sys.transformers[0].lv_phase_mask = PhaseMask::a();
  return sys;
}

ThreePhaseACSystem build_dyn11_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_dyn11_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Dyn11")};
  return sys;
}

ThreePhaseACSystem build_ynd1_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_ynd1_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "YNd1")};
  return sys;
}

ThreePhaseACSystem build_dyn7_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_dyn7_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Dyn7")};
  return sys;
}

ThreePhaseACSystem build_dd0_tap_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_dd0_tap";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};

  auto transformer = make_transformer(1, "Dd0");
  transformer.tap_side = 1;
  transformer.tap_pos = 4;
  transformer.tap_neutral = 0;
  transformer.tap_step_percent = 1.25;
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_dd6_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_dd6_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Dd6")};
  return sys;
}

ThreePhaseACSystem build_dd5_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_dd5_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Dd5")};
  return sys;
}

ThreePhaseACSystem build_yd1_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_yd1_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Yd1")};
  return sys;
}

ThreePhaseACSystem build_yd5_no_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_yd5_noload";

  auto slack = make_bus(1, BusType::SLACK);
  auto load = make_bus(2, BusType::PQ);
  sys.buses = {slack, load};
  sys.transformers = {make_transformer(1, "Yd5")};
  return sys;
}

ThreePhaseACSystem build_partial_dyn11_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_dyn11";

  auto hv = make_bus(1, BusType::SLACK);
  hv.phase_mask = PhaseMask::ab();
  auto lv = make_bus(2, BusType::PQ);
  lv.phase_mask = PhaseMask::a();
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Dyn11");
  transformer.hv_phase_mask = PhaseMask::ab();
  transformer.lv_phase_mask = PhaseMask::a();
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_partial_yd1_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_yd1";

  auto hv = make_bus(1, BusType::SLACK);
  hv.phase_mask = PhaseMask::a();
  auto lv = make_bus(2, BusType::PQ);
  lv.phase_mask = PhaseMask::ab();
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Yd1");
  transformer.hv_phase_mask = PhaseMask::a();
  transformer.lv_phase_mask = PhaseMask::ab();
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_partial_dd0_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_dd0";

  auto hv = make_bus(1, BusType::SLACK);
  hv.phase_mask = PhaseMask::ab();
  auto lv = make_bus(2, BusType::PQ);
  lv.phase_mask = PhaseMask::ab();
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Dd0");
  transformer.hv_phase_mask = PhaseMask::ab();
  transformer.lv_phase_mask = PhaseMask::ab();
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_partial_dd6_case() {
  auto sys = build_partial_dd0_case();
  sys.name = "tp_nr_partial_dd6";
  sys.transformers[0].vector_group = "Dd6";
  return sys;
}

ThreePhaseACSystem build_open_delta_dd0_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_open_delta_dd0";

  auto hv = make_bus(1, BusType::SLACK);
  auto lv = make_bus(2, BusType::PQ);
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Dd0");
  transformer.hv_winding_topology = "AB,BC";
  transformer.lv_winding_topology = "AB,BC";
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_open_delta_dd5_case() {
  auto sys = build_open_delta_dd0_case();
  sys.name = "tp_nr_open_delta_dd5";
  sys.transformers[0].vector_group = "Dd5";
  return sys;
}

ThreePhaseACSystem build_explicit_closed_delta_dd0_tap_case() {
  auto sys = build_dd0_tap_case();
  sys.name = "tp_nr_explicit_closed_delta_dd0_tap";
  sys.transformers[0].hv_winding_topology = "AB,BC,CA";
  sys.transformers[0].lv_winding_topology = "AB,BC,CA";
  return sys;
}

ThreePhaseACSystem build_open_delta_yd1_ac_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_open_delta_yd1_ac";

  auto hv = make_bus(1, BusType::SLACK);
  auto lv = make_bus(2, BusType::PQ);
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Yd1");
  transformer.hv_winding_topology = "A,C";
  transformer.lv_winding_topology = "AB,CA";
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_open_delta_dyn11_bc_ordered_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_open_delta_dyn11_bc_ordered";

  auto hv = make_bus(1, BusType::SLACK);
  auto lv = make_bus(2, BusType::PQ);
  lv.phase_mask = PhaseMask::bc();
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Dyn11");
  transformer.lv_phase_mask = PhaseMask::bc();
  transformer.hv_winding_topology = "AB,BC";
  transformer.lv_winding_topology = "B,C";
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_open_delta_dyn11_bc_hv_topology_only_case() {
  auto sys = build_open_delta_dyn11_bc_ordered_case();
  sys.name = "tp_nr_open_delta_dyn11_bc_hv_topology_only";
  sys.transformers[0].lv_winding_topology.clear();
  return sys;
}

ThreePhaseACSystem build_open_delta_dyn11_bc_phase_mask_only_case() {
  auto sys = build_open_delta_dyn11_bc_hv_topology_only_case();
  sys.name = "tp_nr_open_delta_dyn11_bc_phase_mask_only";
  sys.transformers[0].hv_winding_topology.clear();
  return sys;
}

ThreePhaseACSystem build_open_delta_yd1_bc_ordered_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_open_delta_yd1_bc_ordered";

  auto hv = make_bus(1, BusType::SLACK);
  hv.phase_mask = PhaseMask::bc();
  auto lv = make_bus(2, BusType::PQ);
  sys.buses = {hv, lv};

  auto transformer = make_transformer(1, "Yd1");
  transformer.hv_phase_mask = PhaseMask::bc();
  transformer.hv_winding_topology = "B,C";
  transformer.lv_winding_topology = "AB,BC";
  sys.transformers = {transformer};
  return sys;
}

ThreePhaseACSystem build_open_delta_yd1_bc_lv_topology_only_case() {
  auto sys = build_open_delta_yd1_bc_ordered_case();
  sys.name = "tp_nr_open_delta_yd1_bc_lv_topology_only";
  sys.transformers[0].hv_winding_topology.clear();
  return sys;
}

ThreePhaseACSystem build_open_delta_yd1_bc_phase_mask_only_case() {
  auto sys = build_open_delta_yd1_bc_lv_topology_only_case();
  sys.name = "tp_nr_open_delta_yd1_bc_phase_mask_only";
  sys.transformers[0].lv_winding_topology.clear();
  return sys;
}

ThreePhaseACSystem build_single_source_case(bool weak_source) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = weak_source ? "tp_nr_single_source_weak" : "tp_nr_single_source_strong";

  auto bus1 = make_bus(1, BusType::PQ);
  auto bus2 = make_bus(2, BusType::PQ);
  bus2.pd_a_mw = 0.80;
  bus2.qd_a_mvar = 0.30;
  bus2.pd_b_mw = 0.70;
  bus2.qd_b_mvar = 0.26;
  bus2.pd_c_mw = 0.90;
  bus2.qd_c_mvar = 0.34;

  sys.buses = {bus1, bus2};
  sys.lines = {make_line(1, 1, 2, 0.03, 0.08)};
  sys.external_grids = {weak_source
                            ? make_source(1, 1, 1.0, 0.0, 0.08, 0.30)
                            : make_source(1, 1, 1.0, 0.0, 0.01, 0.05)};
  return sys;
}

ThreePhaseACSystem build_multi_source_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_multi_source";

  auto bus1 = make_bus(1, BusType::PQ);
  auto bus2 = make_bus(2, BusType::PQ);
  auto bus3 = make_bus(3, BusType::PQ);
  bus2.pd_a_mw = 1.05;
  bus2.qd_a_mvar = 0.42;
  bus2.pd_b_mw = 0.95;
  bus2.qd_b_mvar = 0.38;
  bus2.pd_c_mw = 1.10;
  bus2.qd_c_mvar = 0.44;

  sys.buses = {bus1, bus2, bus3};
  sys.lines = {
      make_line(1, 1, 2, 0.02, 0.06),
      make_line(2, 2, 3, 0.02, 0.06),
  };
  sys.external_grids = {
      make_source(1, 1, 1.0, 0.0, 0.02, 0.08),
      make_source(2, 3, 0.98, -2.0, 0.06, 0.20),
  };
  return sys;
}

ThreePhaseACSystem build_min_short_circuit_source_case() {
  auto sys = build_single_source_case(false);
  auto& source = sys.external_grids[0];
  source.r1_pu = 0.0;
  source.x1_pu = 0.0;
  source.r2_pu = 0.0;
  source.x2_pu = 0.0;
  source.r0_pu = 0.0;
  source.x0_pu = 0.0;
  source.s_sc_max_mva = 0.0;
  source.rx_max = 0.0;
  source.s_sc_min_mva = 35.0;
  source.rx_min = 0.35;
  return sys;
}

ThreePhaseACSystem build_per_phase_source_voltage_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_phase_source_voltage";

  auto bus = make_bus(1, BusType::PQ);
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.02, 0.08);
  source.use_phase_voltage_setpoint = true;
  source.vm_a_pu = 1.03;
  source.va_a_deg = 4.0;
  source.vm_b_pu = 0.98;
  source.va_b_deg = -121.5;
  source.vm_c_pu = 1.01;
  source.va_c_deg = 118.0;
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_partial_phase_source_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_phase_source";

  auto bus = make_bus(1, BusType::PQ);
  bus.phase_mask = PhaseMask::ab();
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.02, 0.08);
  source.phase_mask = PhaseMask::ab();
  source.use_phase_voltage_setpoint = true;
  source.vm_a_pu = 1.02;
  source.va_a_deg = 6.0;
  source.vm_b_pu = 0.97;
  source.va_b_deg = -117.0;
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_phase_impedance_source_case(bool explicit_phase_impedance) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = explicit_phase_impedance ? "tp_nr_phase_source_impedance_explicit"
                                      : "tp_nr_phase_source_impedance_legacy";

  auto bus = make_bus(1, BusType::PQ);
  bus.pd_a_mw = 0.80;
  bus.qd_a_mvar = 0.30;
  bus.pd_b_mw = 0.80;
  bus.qd_b_mvar = 0.30;
  bus.pd_c_mw = 0.80;
  bus.qd_c_mvar = 0.30;
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.04, 0.15);
  if (explicit_phase_impedance) {
    source.use_phase_impedance = true;
    source.r_a_pu = 0.08;
    source.x_a_pu = 0.30;
    source.r_b_pu = 0.04;
    source.x_b_pu = 0.15;
    source.r_c_pu = 0.01;
    source.x_c_pu = 0.05;
  }
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_phase_matrix_source_case() {
  auto sys = build_phase_impedance_source_case(false);
  sys.name = "tp_nr_phase_source_impedance_matrix";

  auto& source = sys.external_grids[0];
  source.use_phase_impedance_matrix = true;
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 0, 0, 0.08);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 0, 1, 0.03);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 1, 0, 0.03);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 1, 1, 0.04);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 1, 2, 0.02);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 2, 1, 0.02);
  hacdcpf::phase_matrix_set(source.r_matrix_pu, 2, 2, 0.01);

  hacdcpf::phase_matrix_set(source.x_matrix_pu, 0, 0, 0.30);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 0, 1, 0.10);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 1, 0, 0.10);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 1, 1, 0.15);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 1, 2, 0.06);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 2, 1, 0.06);
  hacdcpf::phase_matrix_set(source.x_matrix_pu, 2, 2, 0.05);
  return sys;
}

ThreePhaseACSystem build_explicit_wye_topology_source_case() {
  auto sys = build_per_phase_source_voltage_case();
  sys.name = "tp_nr_explicit_wye_source_topology";
  sys.external_grids[0].source_topology = "A,B,C";
  return sys;
}

ThreePhaseACSystem build_delta_topology_source_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_delta_source_topology";

  auto bus = make_bus(1, BusType::PQ);
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.02, 0.08);
  source.source_topology = "AB,BC,CA";
  source.use_phase_voltage_setpoint = true;
  source.use_phase_impedance = true;
  const std::array<std::complex<double>, 3> phase_reference = {
      std::polar(1.02, 5.0 * kPi / 180.0),
      std::polar(0.99, -121.0 * kPi / 180.0),
      std::polar(1.01, 118.0 * kPi / 180.0),
  };
  set_source_slot_voltage(source, 0, phase_reference[0] - phase_reference[1]);
  set_source_slot_voltage(source, 1, phase_reference[1] - phase_reference[2]);
  set_source_slot_voltage(source, 2, phase_reference[2] - phase_reference[0]);
  set_source_slot_impedance(source, 0, 0.025, 0.090);
  set_source_slot_impedance(source, 1, 0.020, 0.070);
  set_source_slot_impedance(source, 2, 0.030, 0.080);
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_delta_topology_source_sequence_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_delta_source_topology_sequence";

  auto bus = make_bus(1, BusType::PQ);
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.02, 0.08);
  source.r2_pu = 0.022;
  source.x2_pu = 0.084;
  source.r0_pu = 0.06;
  source.x0_pu = 0.24;
  source.source_topology = "AB,BC,CA";
  source.use_phase_voltage_setpoint = true;
  const std::array<std::complex<double>, 3> phase_reference = {
      std::polar(1.01, 3.0 * kPi / 180.0),
      std::polar(0.995, -120.5 * kPi / 180.0),
      std::polar(1.005, 119.0 * kPi / 180.0),
  };
  set_source_slot_voltage(source, 0, phase_reference[0] - phase_reference[1]);
  set_source_slot_voltage(source, 1, phase_reference[1] - phase_reference[2]);
  set_source_slot_voltage(source, 2, phase_reference[2] - phase_reference[0]);
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_partial_delta_topology_source_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_delta_source_topology";

  auto bus = make_bus(1, BusType::PQ);
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.02, 0.08);
  source.source_topology = "AB,BC";
  source.use_phase_voltage_setpoint = true;
  source.use_phase_impedance = true;
  set_source_slot_voltage(source, 0, std::polar(1.08, 12.0 * kPi / 180.0));
  set_source_slot_voltage(source, 1, std::polar(0.96, -111.0 * kPi / 180.0));
  set_source_slot_impedance(source, 0, 0.018, 0.060);
  set_source_slot_impedance(source, 1, 0.022, 0.075);
  sys.external_grids = {source};
  return sys;
}

ThreePhaseACSystem build_partial_delta_topology_source_sequence_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "tp_nr_partial_delta_source_topology_sequence";

  auto bus = make_bus(1, BusType::PQ);
  sys.buses = {bus};

  auto source = make_source(1, 1, 1.0, 0.0, 0.03, 0.10);
  source.r2_pu = 0.032;
  source.x2_pu = 0.105;
  source.r0_pu = 0.12;
  source.x0_pu = 0.35;
  source.source_topology = "AB,BC";
  source.use_phase_voltage_setpoint = true;
  set_source_slot_voltage(source, 0, std::polar(1.06, 10.0 * kPi / 180.0));
  set_source_slot_voltage(source, 1, std::polar(0.97, -112.0 * kPi / 180.0));
  sys.external_grids = {source};
  return sys;
}

}  // namespace

TEST_CASE("Three-phase NR stamps grounded-wye transformer tap into compact network", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_grounded_wye_tap_case(), nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 2);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.05) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.05) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.05) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 0.0) < 1e-3);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, -120.0) < 1e-3);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, 120.0) < 1e-3);
}

TEST_CASE("Three-phase NR stamps YNyn6 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_ynyn6_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 180.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 60.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -60.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps YNyn7 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_ynyn7_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 30.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -90.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps grounded-wye transformer zero-sequence magnetizing primitive in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto open = solve_three_phase_nr(
      build_ynyn0_zero_sequence_magnetizing_case(false),
      nr_options());
  const auto magnetized = solve_three_phase_nr(
      build_ynyn0_zero_sequence_magnetizing_case(true),
      nr_options());

  REQUIRE(open.converged);
  REQUIRE(magnetized.converged);
  REQUIRE(open.transformer_terminal_observations.size() == 1);
  REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

  const auto& open_obs = open.transformer_terminal_observations.front();
  const auto& mag_obs = magnetized.transformer_terminal_observations.front();
  const double open_hv_a = terminal_phase_current_mag(open_obs, 0, true);
  const double mag_hv_a = terminal_phase_current_mag(mag_obs, 0, true);
  const double mag_hv_b = terminal_phase_current_mag(mag_obs, 1, true);
  const double mag_hv_c = terminal_phase_current_mag(mag_obs, 2, true);

  CHECK(open_hv_a < 1e-6);
  CHECK(mag_hv_a > open_hv_a + 1e-3);
  CHECK(std::abs(mag_hv_b - mag_hv_a) < 1e-6 * std::max(1.0, mag_hv_a));
  CHECK(std::abs(mag_hv_c - mag_hv_a) < 1e-6 * std::max(1.0, mag_hv_a));

  const auto& open_lv = open.bus_voltages[1];
  const auto& mag_lv = magnetized.bus_voltages[1];
  CHECK(mag_lv.vm_a_pu < open_lv.vm_a_pu);
  CHECK(std::abs(mag_lv.vm_a_pu - mag_lv.vm_b_pu) < 5e-4);
  CHECK(std::abs(mag_lv.vm_b_pu - mag_lv.vm_c_pu) < 5e-4);
  CHECK(wrapped_angle_error_deg(mag_lv.va_a_deg, 0.0) < 2e-2);
  CHECK(wrapped_angle_error_deg(mag_lv.va_b_deg, 0.0) < 2e-2);
  CHECK(wrapped_angle_error_deg(mag_lv.va_c_deg, 0.0) < 2e-2);
}

TEST_CASE("Three-phase NR extends zero-sequence magnetizing semantics across blocked-port wye families", "[integration][powerflow][three_phase_nr][transformer]") {
  SECTION("grounded-wye to ungrounded-wye keeps zero-sequence current on the grounded side") {
    const auto open = solve_three_phase_nr(
        build_yny0_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_yny0_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) >
          terminal_phase_current_mag(open_obs, 0, true) + 1.0);
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 1, false) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 2, false) < 1e-6);
  }

  SECTION("ungrounded wye-wye treats zero-sequence magnetizing fields as externally blocked") {
    const auto open = solve_three_phase_nr(
        build_yy0_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_yy0_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) <
          terminal_phase_current_mag(open_obs, 0, true) + 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) <
          terminal_phase_current_mag(open_obs, 0, false) + 1e-6);

    const auto& open_lv = open.bus_voltages[1];
    const auto& mag_lv = magnetized.bus_voltages[1];
    CHECK(std::abs(mag_lv.vm_a_pu - open_lv.vm_a_pu) < 1e-9);
    CHECK(std::abs(mag_lv.vm_b_pu - open_lv.vm_b_pu) < 1e-9);
    CHECK(std::abs(mag_lv.vm_c_pu - open_lv.vm_c_pu) < 1e-9);
  }
}

TEST_CASE("Three-phase NR stamps partial-mask grounded-wye common-mode projector in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto open = solve_three_phase_nr(
      build_partial_ynyn0_zero_sequence_magnetizing_case(false),
      nr_options());
  const auto magnetized = solve_three_phase_nr(
      build_partial_ynyn0_zero_sequence_magnetizing_case(true),
      nr_options());

  REQUIRE(open.converged);
  REQUIRE(magnetized.converged);
  REQUIRE(open.transformer_terminal_observations.size() == 1);
  REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

  const auto& open_obs = open.transformer_terminal_observations.front();
  const auto& mag_obs = magnetized.transformer_terminal_observations.front();
  CHECK(terminal_phase_current_mag(mag_obs, 0, true) >
        terminal_phase_current_mag(open_obs, 0, true) + 1.0);
  CHECK(terminal_phase_current_mag(mag_obs, 1, true) >
        terminal_phase_current_mag(open_obs, 1, true) + 1.0);

  const auto& mag_lv = magnetized.bus_voltages[1];
  CHECK(std::abs(mag_lv.vm_a_pu - mag_lv.vm_b_pu) < 5e-4);
  CHECK(std::abs(mag_lv.vm_c_pu) <= 1e-12);
}

TEST_CASE("Three-phase NR stamps mixed grounded-delta zero-sequence magnetizing shunts in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  SECTION("grounded-wye/delta keeps zero-sequence current on the grounded side") {
    const auto open = solve_three_phase_nr(
        build_ynd1_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_ynd1_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) >
          terminal_phase_current_mag(open_obs, 0, true) + 50.0);
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 1, false) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 2, false) < 1e-6);
  }

  SECTION("delta/grounded-wye keeps zero-sequence current on the grounded side") {
    const auto open = solve_three_phase_nr(
        build_dyn11_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_dyn11_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) >
          terminal_phase_current_mag(open_obs, 0, false) + 50.0);
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 1, true) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 2, true) < 1e-6);
  }
}

TEST_CASE("Three-phase NR accepts blocked-port mixed zero-sequence magnetizing contracts without forking the primitive", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto open = solve_three_phase_nr(
      build_yd1_zero_sequence_magnetizing_case(false),
      nr_options());
  const auto magnetized = solve_three_phase_nr(
      build_yd1_zero_sequence_magnetizing_case(true),
      nr_options());

  REQUIRE(open.converged);
  REQUIRE(magnetized.converged);
  REQUIRE(open.transformer_terminal_observations.size() == 1);
  REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

  const auto& open_obs = open.transformer_terminal_observations.front();
  const auto& mag_obs = magnetized.transformer_terminal_observations.front();
  CHECK(terminal_phase_current_mag(mag_obs, 0, true) <
        terminal_phase_current_mag(open_obs, 0, true) + 1e-6);
  CHECK(terminal_phase_current_mag(mag_obs, 0, false) <
        terminal_phase_current_mag(open_obs, 0, false) + 1e-6);

  const auto& open_lv = open.bus_voltages[1];
  const auto& mag_lv = magnetized.bus_voltages[1];
  CHECK(std::abs(mag_lv.vm_a_pu - open_lv.vm_a_pu) < 1e-9);
  CHECK(std::abs(mag_lv.vm_b_pu - open_lv.vm_b_pu) < 1e-9);
  CHECK(std::abs(mag_lv.vm_c_pu - open_lv.vm_c_pu) < 1e-9);
}

TEST_CASE("Three-phase NR stamps partial-mask mixed grounded-delta common-mode projector in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  SECTION("partial grounded-wye/delta keeps zero-sequence current on the grounded side") {
    const auto open = solve_three_phase_nr(
        build_partial_ynd1_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_partial_ynd1_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) >
          terminal_phase_current_mag(open_obs, 0, true) + 1.0);
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 1, false) < 1e-6);
  }

  SECTION("partial delta/grounded-wye keeps zero-sequence current on the grounded side") {
    const auto open = solve_three_phase_nr(
        build_partial_dyn11_zero_sequence_magnetizing_case(false),
        nr_options());
    const auto magnetized = solve_three_phase_nr(
        build_partial_dyn11_zero_sequence_magnetizing_case(true),
        nr_options());

    REQUIRE(open.converged);
    REQUIRE(magnetized.converged);
    REQUIRE(open.transformer_terminal_observations.size() == 1);
    REQUIRE(magnetized.transformer_terminal_observations.size() == 1);

    const auto& open_obs = open.transformer_terminal_observations.front();
    const auto& mag_obs = magnetized.transformer_terminal_observations.front();
    CHECK(terminal_phase_current_mag(mag_obs, 0, false) >
          terminal_phase_current_mag(open_obs, 0, false) + 1.0);
    CHECK(terminal_phase_current_mag(mag_obs, 0, true) < 1e-6);
    CHECK(terminal_phase_current_mag(mag_obs, 1, true) < 1e-6);
  }
}

TEST_CASE("Three-phase NR stamps Dyn11 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_dyn11_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 30.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, -90.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, 150.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Dyn7 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_dyn7_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 30.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -90.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps YNd1 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_ynd1_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, -30.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, -150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, 90.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Yd1 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_yd1_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, -30.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, -150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, 90.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Yd5 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_yd5_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, -150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 90.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -30.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Dd0 transformer tap in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_dd0_tap_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.05) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.05) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.05) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 0.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, -120.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, 120.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Dd6 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_dd6_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, 180.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 60.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -60.0) < 1e-2);
}

TEST_CASE("Three-phase NR stamps Dd5 transformer phase shift in abc domain", "[integration][powerflow][three_phase_nr][transformer]") {
  const auto result = solve_three_phase_nr(build_dd5_no_load_case(), nr_options());
  REQUIRE(result.converged);

  const auto& lv = result.bus_voltages[1];
  CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_b_pu - 1.0) < 5e-4);
  CHECK(std::abs(lv.vm_c_pu - 1.0) < 5e-4);
  CHECK(wrapped_angle_error_deg(lv.va_a_deg, -150.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_b_deg, 90.0) < 1e-2);
  CHECK(wrapped_angle_error_deg(lv.va_c_deg, -30.0) < 1e-2);
}

TEST_CASE("Three-phase NR consumes explicit transformer winding topology without forking the shared primitive", "[integration][powerflow][three_phase_nr][transformer]") {
  SECTION("open-delta contract keeps explicit active windings on a full delta terminal") {
    const auto result = solve_three_phase_nr(build_open_delta_dd0_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = phase_voltage(hv, 0) - phase_voltage(hv, 1);
    const auto hv_vbc = phase_voltage(hv, 1) - phase_voltage(hv, 2);
    const auto lv_vab = phase_voltage(lv, 0) - phase_voltage(lv, 1);
    const auto lv_vbc = phase_voltage(lv, 1) - phase_voltage(lv, 2);

    CHECK(std::abs(std::abs(lv_vab) - std::abs(hv_vab)) < 5e-4);
    CHECK(std::abs(std::abs(lv_vbc) - std::abs(hv_vbc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vab) * 180.0 / kPi,
              std::arg(hv_vab) * 180.0 / kPi) < 1e-2);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vbc) * 180.0 / kPi,
              std::arg(hv_vbc) * 180.0 / kPi) < 1e-2);
  }

  SECTION("open-delta contract composes with same-connection 150 degree clock groups") {
    const auto result = solve_three_phase_nr(build_open_delta_dd5_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = phase_voltage(hv, 0) - phase_voltage(hv, 1);
    const auto hv_vbc = phase_voltage(hv, 1) - phase_voltage(hv, 2);
    const auto lv_vab = phase_voltage(lv, 0) - phase_voltage(lv, 1);
    const auto lv_vbc = phase_voltage(lv, 1) - phase_voltage(lv, 2);

    CHECK(std::abs(std::abs(lv_vab) - std::abs(hv_vab)) < 5e-4);
    CHECK(std::abs(std::abs(lv_vbc) - std::abs(hv_vbc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vab) * 180.0 / kPi,
              std::arg(hv_vab) * 180.0 / kPi - 150.0) < 1e-2);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vbc) * 180.0 / kPi,
              std::arg(hv_vbc) * 180.0 / kPi - 150.0) < 1e-2);
  }

  SECTION("explicit full delta topology remains compatible with the legacy vector-group path") {
    const auto result = solve_three_phase_nr(
        build_explicit_closed_delta_dd0_tap_case(),
        nr_options());
    REQUIRE(result.converged);

    const auto& lv = result.bus_voltages[1];
    CHECK(std::abs(lv.vm_a_pu - 1.05) < 5e-4);
    CHECK(std::abs(lv.vm_b_pu - 1.05) < 5e-4);
    CHECK(std::abs(lv.vm_c_pu - 1.05) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_a_deg, 0.0) < 1e-2);
    CHECK(wrapped_angle_error_deg(lv.va_b_deg, -120.0) < 1e-2);
    CHECK(wrapped_angle_error_deg(lv.va_c_deg, 120.0) < 1e-2);
  }

  SECTION("mixed wye-delta path accepts non-AB,BC open-delta subsets through the same embedding chain") {
    const auto result = solve_three_phase_nr(build_open_delta_yd1_ac_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto vab = line_voltage(lv, 0, 1);
    const auto vca = line_voltage(lv, 2, 0);
    const auto hv_va = phase_voltage(hv, 0);
    const auto hv_vc = phase_voltage(hv, 2);

    CHECK(std::abs(std::abs(vab) - std::sqrt(3.0) * std::abs(hv_va)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), angle_deg(hv_va)) < 1e-2);
    CHECK(std::abs(std::abs(vca) - std::sqrt(3.0) * std::abs(hv_vc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vca), angle_deg(hv_vc)) < 1e-2);
  }

  SECTION("mixed delta-wye path accepts explicit ordered winding correspondence beyond canonical label match") {
    const auto result = solve_three_phase_nr(build_open_delta_dyn11_bc_ordered_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = line_voltage(hv, 0, 1);
    const auto hv_vbc = line_voltage(hv, 1, 2);

    CHECK(std::abs(lv.vm_a_pu) <= 1e-12);
    CHECK(std::abs(lv.vm_b_pu - std::abs(hv_vab) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_b_deg, angle_deg(hv_vab)) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu - std::abs(hv_vbc) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_c_deg, angle_deg(hv_vbc)) < 1e-2);
  }

  SECTION("mixed delta-wye path accepts one-sided explicit winding correspondence when the wye order is implied by phase_mask") {
    const auto result =
        solve_three_phase_nr(build_open_delta_dyn11_bc_hv_topology_only_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = line_voltage(hv, 0, 1);
    const auto hv_vbc = line_voltage(hv, 1, 2);

    CHECK(std::abs(lv.vm_a_pu) <= 1e-12);
    CHECK(std::abs(lv.vm_b_pu - std::abs(hv_vab) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_b_deg, angle_deg(hv_vab)) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu - std::abs(hv_vbc) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_c_deg, angle_deg(hv_vbc)) < 1e-2);
  }

  SECTION("mixed delta-wye path infers canonical open-delta correspondence from connection, clock, and phase_mask alone") {
    const auto result =
        solve_three_phase_nr(build_open_delta_dyn11_bc_phase_mask_only_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vbc = line_voltage(hv, 1, 2);
    const auto hv_vca = line_voltage(hv, 2, 0);

    CHECK(std::abs(lv.vm_a_pu) <= 1e-12);
    CHECK(std::abs(lv.vm_b_pu - std::abs(hv_vbc) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_b_deg, angle_deg(hv_vbc)) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu - std::abs(hv_vca) / std::sqrt(3.0)) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_c_deg, angle_deg(hv_vca)) < 1e-2);
  }

  SECTION("mixed wye-delta path accepts explicit ordered winding correspondence beyond canonical label match") {
    const auto result = solve_three_phase_nr(build_open_delta_yd1_bc_ordered_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto vab = line_voltage(lv, 0, 1);
    const auto vbc = line_voltage(lv, 1, 2);
    const auto hv_vb = phase_voltage(hv, 1);
    const auto hv_vc = phase_voltage(hv, 2);

    CHECK(std::abs(std::abs(vab) - std::sqrt(3.0) * std::abs(hv_vb)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), angle_deg(hv_vb)) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - std::sqrt(3.0) * std::abs(hv_vc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), angle_deg(hv_vc)) < 1e-2);
  }

  SECTION("mixed wye-delta path infers canonical open-delta correspondence from connection, clock, and phase_mask alone") {
    const auto result =
        solve_three_phase_nr(build_open_delta_yd1_bc_phase_mask_only_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto vbc = line_voltage(lv, 1, 2);
    const auto vca = line_voltage(lv, 2, 0);
    const auto hv_vb = phase_voltage(hv, 1);
    const auto hv_vc = phase_voltage(hv, 2);

    CHECK(std::abs(std::abs(vbc) - std::sqrt(3.0) * std::abs(hv_vb)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), angle_deg(hv_vb)) < 1e-2);
    CHECK(std::abs(std::abs(vca) - std::sqrt(3.0) * std::abs(hv_vc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vca), angle_deg(hv_vc)) < 1e-2);
  }

  SECTION("mixed wye-delta path accepts one-sided explicit winding correspondence when the wye order is implied by phase_mask") {
    const auto result =
        solve_three_phase_nr(build_open_delta_yd1_bc_lv_topology_only_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto vab = line_voltage(lv, 0, 1);
    const auto vbc = line_voltage(lv, 1, 2);
    const auto hv_vb = phase_voltage(hv, 1);
    const auto hv_vc = phase_voltage(hv, 2);

    CHECK(std::abs(std::abs(vab) - std::sqrt(3.0) * std::abs(hv_vb)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), angle_deg(hv_vb)) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - std::sqrt(3.0) * std::abs(hv_vc)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), angle_deg(hv_vc)) < 1e-2);
  }
}

TEST_CASE("Three-phase NR stamps mixed-phase delta transformer primitives without full ABC assumptions", "[integration][powerflow][three_phase_nr][transformer]") {
  SECTION("delta-wye partial mask aligns canonical single winding") {
    const auto result = solve_three_phase_nr(build_partial_dyn11_case(), nr_options());
    REQUIRE(result.converged);

    const auto& lv = result.bus_voltages[1];
    CHECK(std::abs(lv.vm_a_pu - 1.0) < 5e-4);
    CHECK(wrapped_angle_error_deg(lv.va_a_deg, 30.0) < 1e-2);
    CHECK(std::abs(lv.vm_b_pu) <= 1e-12);
    CHECK(std::abs(lv.vm_c_pu) <= 1e-12);
  }

  SECTION("wye-delta partial mask aligns canonical single winding") {
    const auto result = solve_three_phase_nr(build_partial_yd1_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto vab = line_voltage(lv, 0, 1);
    const auto hv_va = phase_voltage(hv, 0);

    CHECK(std::abs(std::abs(vab) - std::sqrt(3.0) * std::abs(hv_va)) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), angle_deg(hv_va)) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu) <= 1e-12);
  }

  SECTION("delta-delta partial mask preserves the active line-to-line winding") {
    const auto result = solve_three_phase_nr(build_partial_dd0_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = phase_voltage(hv, 0) - phase_voltage(hv, 1);
    const auto lv_vab = phase_voltage(lv, 0) - phase_voltage(lv, 1);

    CHECK(std::abs(std::abs(lv_vab) - std::abs(hv_vab)) < 5e-4);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vab) * 180.0 / kPi,
              std::arg(hv_vab) * 180.0 / kPi) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu) <= 1e-12);
  }

  SECTION("delta-delta partial mask preserves the active line-to-line winding under 180 degree clock") {
    const auto result = solve_three_phase_nr(build_partial_dd6_case(), nr_options());
    REQUIRE(result.converged);

    const auto& hv = result.bus_voltages[0];
    const auto& lv = result.bus_voltages[1];
    const auto hv_vab = phase_voltage(hv, 0) - phase_voltage(hv, 1);
    const auto lv_vab = phase_voltage(lv, 0) - phase_voltage(lv, 1);

    CHECK(std::abs(std::abs(lv_vab) - std::abs(hv_vab)) < 5e-4);
    CHECK(wrapped_angle_error_deg(
              std::arg(lv_vab) * 180.0 / kPi,
              std::arg(hv_vab) * 180.0 / kPi + 180.0) < 1e-2);
    CHECK(std::abs(lv.vm_c_pu) <= 1e-12);
  }
}

TEST_CASE("Three-phase NR external-grid Norton stamping captures weak and multi-source behavior", "[integration][powerflow][three_phase_nr][source]") {
  const auto strong = solve_three_phase_nr(build_single_source_case(false), nr_options());
  const auto weak = solve_three_phase_nr(build_single_source_case(true), nr_options());
  const auto multi = solve_three_phase_nr(build_multi_source_case(), nr_options());
  const auto min_sc = solve_three_phase_nr(build_min_short_circuit_source_case(), nr_options());

  REQUIRE(strong.converged);
  REQUIRE(weak.converged);
  REQUIRE(multi.converged);
  REQUIRE(min_sc.converged);

  CHECK(weak.bus_voltages[0].vm_a_pu < strong.bus_voltages[0].vm_a_pu);
  CHECK(weak.bus_voltages[1].vm_a_pu < strong.bus_voltages[1].vm_a_pu);
  CHECK(weak.bus_voltages[0].vm_a_pu < 1.0);
  CHECK(multi.bus_voltages[1].vm_a_pu > weak.bus_voltages[1].vm_a_pu);
  CHECK(multi.bus_voltages[1].vm_a_pu < strong.bus_voltages[1].vm_a_pu);
  CHECK(min_sc.bus_voltages[1].vm_a_pu < 1.0);
  CHECK(min_sc.bus_voltages[1].vm_a_pu < strong.bus_voltages[1].vm_a_pu);
}

TEST_CASE("Three-phase NR external-grid source contract supports per-phase voltage setpoints and phase impedance overrides", "[integration][powerflow][three_phase_nr][source]") {
  SECTION("per-phase source voltage setpoint falls back to sequence impedance") {
    const auto result = solve_three_phase_nr(build_per_phase_source_voltage_case(), nr_options());
    REQUIRE(result.converged);

    const auto& bus = result.bus_voltages[0];
    CHECK(std::abs(bus.vm_a_pu - 1.03) < 5e-4);
    CHECK(wrapped_angle_error_deg(bus.va_a_deg, 4.0) < 1e-2);
    CHECK(std::abs(bus.vm_b_pu - 0.98) < 5e-4);
    CHECK(wrapped_angle_error_deg(bus.va_b_deg, -121.5) < 1e-2);
    CHECK(std::abs(bus.vm_c_pu - 1.01) < 5e-4);
    CHECK(wrapped_angle_error_deg(bus.va_c_deg, 118.0) < 1e-2);
  }

  SECTION("partial-mask source consumes per-phase setpoint without full ABC assumptions") {
    const auto result = solve_three_phase_nr(build_partial_phase_source_case(), nr_options());
    REQUIRE(result.converged);

    const auto& bus = result.bus_voltages[0];
    CHECK(std::abs(bus.vm_a_pu - 1.02) < 5e-4);
    CHECK(wrapped_angle_error_deg(bus.va_a_deg, 6.0) < 1e-2);
    CHECK(std::abs(bus.vm_b_pu - 0.97) < 5e-4);
    CHECK(wrapped_angle_error_deg(bus.va_b_deg, -117.0) < 1e-2);
    CHECK(std::abs(bus.vm_c_pu) <= 1e-12);
  }

  SECTION("explicit phase impedance overrides legacy balanced Norton path without forking solver logic") {
    const auto legacy = solve_three_phase_nr(build_phase_impedance_source_case(false), nr_options());
    const auto explicit_phase = solve_three_phase_nr(build_phase_impedance_source_case(true), nr_options());

    REQUIRE(legacy.converged);
    REQUIRE(explicit_phase.converged);

    const auto& legacy_bus = legacy.bus_voltages[0];
    const auto& phase_bus = explicit_phase.bus_voltages[0];
    CHECK(std::abs(legacy_bus.vm_a_pu - legacy_bus.vm_b_pu) < 5e-4);
    CHECK(std::abs(legacy_bus.vm_b_pu - legacy_bus.vm_c_pu) < 5e-4);
    CHECK(phase_bus.vm_a_pu < phase_bus.vm_b_pu);
    CHECK(phase_bus.vm_b_pu < phase_bus.vm_c_pu);
    CHECK(phase_bus.vm_a_pu < legacy_bus.vm_a_pu);
  }

  SECTION("full phase-domain source impedance matrix reuses the same Norton primitive") {
    const auto diagonal = solve_three_phase_nr(build_phase_impedance_source_case(true), nr_options());
    const auto matrix = solve_three_phase_nr(build_phase_matrix_source_case(), nr_options());

    REQUIRE(diagonal.converged);
    REQUIRE(matrix.converged);

    const auto& diagonal_bus = diagonal.bus_voltages[0];
    const auto& matrix_bus = matrix.bus_voltages[0];
    CHECK(std::abs(matrix_bus.vm_a_pu - diagonal_bus.vm_a_pu) > 1e-3);
    CHECK(std::abs(matrix_bus.vm_b_pu - diagonal_bus.vm_b_pu) > 1e-3);
    CHECK(matrix_bus.vm_a_pu < matrix_bus.vm_b_pu);
    CHECK(matrix_bus.vm_b_pu < matrix_bus.vm_c_pu);
  }
}

TEST_CASE("Three-phase NR external-grid source topology enters the shared winding-domain primitive", "[integration][powerflow][three_phase_nr][source]") {
  SECTION("explicit wye-like source_topology stays compatible with the legacy path") {
    const auto legacy = solve_three_phase_nr(build_per_phase_source_voltage_case(), nr_options());
    const auto explicit_topology =
        solve_three_phase_nr(build_explicit_wye_topology_source_case(), nr_options());

    REQUIRE(legacy.converged);
    REQUIRE(explicit_topology.converged);

    const auto& legacy_bus = legacy.bus_voltages[0];
    const auto& explicit_bus = explicit_topology.bus_voltages[0];
    CHECK(std::abs(legacy_bus.vm_a_pu - explicit_bus.vm_a_pu) < 1e-9);
    CHECK(std::abs(legacy_bus.vm_b_pu - explicit_bus.vm_b_pu) < 1e-9);
    CHECK(std::abs(legacy_bus.vm_c_pu - explicit_bus.vm_c_pu) < 1e-9);
    CHECK(wrapped_angle_error_deg(legacy_bus.va_a_deg, explicit_bus.va_a_deg) < 1e-9);
    CHECK(wrapped_angle_error_deg(legacy_bus.va_b_deg, explicit_bus.va_b_deg) < 1e-9);
    CHECK(wrapped_angle_error_deg(legacy_bus.va_c_deg, explicit_bus.va_c_deg) < 1e-9);
  }

  SECTION("closed-delta source_topology stamps non-diagonal voltage-source primitive") {
    const auto sys = build_delta_topology_source_case();
    const auto result = solve_three_phase_nr(sys, nr_options());
    REQUIRE(result.converged);

    const auto& source = sys.external_grids[0];
    const auto& bus = result.bus_voltages[0];
    const auto vab = line_voltage(bus, 0, 1);
    const auto vbc = line_voltage(bus, 1, 2);
    const auto vca = line_voltage(bus, 2, 0);

    CHECK(std::abs(std::abs(vab) - source.vm_a_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), source.va_a_deg) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - source.vm_b_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), source.va_b_deg) < 1e-2);
    CHECK(std::abs(std::abs(vca) - source.vm_c_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vca), source.va_c_deg) < 1e-2);
    CHECK(std::abs(vab + vbc + vca) < 1e-8);
  }

  SECTION("closed-delta source_topology falls back to sequence impedance inside the same primitive") {
    const auto sys = build_delta_topology_source_sequence_case();
    const auto result = solve_three_phase_nr(sys, nr_options());
    REQUIRE(result.converged);

    const auto& source = sys.external_grids[0];
    const auto& bus = result.bus_voltages[0];
    const auto vab = line_voltage(bus, 0, 1);
    const auto vbc = line_voltage(bus, 1, 2);
    const auto vca = line_voltage(bus, 2, 0);

    CHECK(std::abs(std::abs(vab) - source.vm_a_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), source.va_a_deg) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - source.vm_b_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), source.va_b_deg) < 1e-2);
    CHECK(std::abs(std::abs(vca) - source.vm_c_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vca), source.va_c_deg) < 1e-2);
  }

  SECTION("partial open-delta source_topology stays inside one shared primitive chain") {
    const auto sys = build_partial_delta_topology_source_case();
    const auto result = solve_three_phase_nr(sys, nr_options());
    REQUIRE(result.converged);

    const auto& source = sys.external_grids[0];
    const auto& bus = result.bus_voltages[0];
    const auto vab = line_voltage(bus, 0, 1);
    const auto vbc = line_voltage(bus, 1, 2);

    CHECK(std::abs(std::abs(vab) - source.vm_a_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), source.va_a_deg) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - source.vm_b_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), source.va_b_deg) < 1e-2);
    CHECK(bus.vm_a_pu > 1e-3);
    CHECK(bus.vm_b_pu > 1e-3);
    CHECK(bus.vm_c_pu > 1e-3);
  }

  SECTION("partial open-delta source_topology also falls back to sequence impedance") {
    const auto sys = build_partial_delta_topology_source_sequence_case();
    const auto result = solve_three_phase_nr(sys, nr_options());
    REQUIRE(result.converged);

    const auto& source = sys.external_grids[0];
    const auto& bus = result.bus_voltages[0];
    const auto vab = line_voltage(bus, 0, 1);
    const auto vbc = line_voltage(bus, 1, 2);

    CHECK(std::abs(std::abs(vab) - source.vm_a_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vab), source.va_a_deg) < 1e-2);
    CHECK(std::abs(std::abs(vbc) - source.vm_b_pu) < 5e-4);
    CHECK(wrapped_angle_error_deg(angle_deg(vbc), source.va_b_deg) < 1e-2);
  }
}

TEST_CASE("Three-phase NR fail-closes unsupported transformer and source contracts", "[unit][powerflow][three_phase_nr][fail_close]") {
  SECTION("unsupported transformer vector group") {
    auto sys = build_grounded_wye_tap_case();
    sys.transformers[0].vector_group = "Yd3";
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("+/-30 and +/-150 degree clock groups"));
  }

  SECTION("delta terminal requires at least two active phases") {
    auto sys = build_partial_dyn11_case();
    sys.buses[0].phase_mask = PhaseMask::a();
    sys.transformers[0].hv_phase_mask = PhaseMask::a();
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("delta terminal requires at least two active phases"));
  }

  SECTION("delta-wye still rejects phase masks whose canonical winding labels do not overlap") {
    auto sys = build_partial_dyn11_case();
    sys.buses[1].phase_mask = PhaseMask::b();
    sys.transformers[0].lv_phase_mask = PhaseMask::b();
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("cannot infer winding correspondence"));
  }

  SECTION("wye-delta still rejects phase masks whose canonical winding labels do not overlap") {
    auto sys = build_partial_yd1_case();
    sys.buses[0].phase_mask = PhaseMask::b();
    sys.transformers[0].hv_phase_mask = PhaseMask::b();
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("cannot infer winding correspondence"));
  }

  SECTION("same-connection path still rejects unsupported non-Julia clock groups") {
    auto sys = build_dd0_tap_case();
    sys.transformers[0].vector_group = "Dd3";
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("0, +/-30, +/-150, and 180 degree clock groups"));
  }

  SECTION("explicit winding topology must stay compatible with the active phase mask") {
    auto sys = build_open_delta_dd0_case();
    sys.buses[0].phase_mask = PhaseMask::ab();
    sys.transformers[0].hv_phase_mask = PhaseMask::ab();
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("hv_winding_topology is incompatible with current phase_mask"));
  }

  SECTION("blocked-port mixed zero-sequence magnetizing contracts no longer fail-close") {
    auto sys = build_yd1_no_load_case();
    sys.transformers[0].mag0_percent = 80.0;
    REQUIRE_NOTHROW(solve_three_phase_nr(sys, nr_options()));
  }

  SECTION("blocked-port wye-wye zero-sequence magnetizing contracts no longer fail-close") {
    auto sys = build_grounded_wye_tap_case();
    sys.transformers[0].vector_group = "Yy0";
    sys.transformers[0].mag0_percent = 80.0;
    REQUIRE_NOTHROW(solve_three_phase_nr(sys, nr_options()));
  }

  SECTION("grounded-wye zero-sequence magnetizing contract validates mag0_rx semantics") {
    auto sys = build_ynyn0_zero_sequence_magnetizing_case(true);
    sys.transformers[0].mag0_percent = 0.0;
    sys.transformers[0].mag0_rx = 0.2;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("mag0_rx requires positive mag0_percent"));
  }

  SECTION("mixed grounded-delta zero-sequence contract validates si0_hv_partial semantics") {
    auto sys = build_ynd1_zero_sequence_magnetizing_case(true);
    sys.transformers[0].si0_hv_partial = 1.5;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("si0_hv_partial must lie within [0, 1]"));
  }

  SECTION("external grid source requires compatible phase mask") {
    ThreePhaseACSystem sys;
    sys.base_mva = 10.0;
    auto bus = make_bus(1, BusType::PQ);
    bus.phase_mask = PhaseMask::a();
    sys.buses = {bus};
    sys.external_grids = {make_source(1, 1, 1.0, 0.0, 0.02, 0.08)};
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("mask is not a subset"));
  }

  SECTION("external grid source requires impedance semantics") {
    auto sys = build_single_source_case(false);
    sys.external_grids[0].r1_pu = 0.0;
    sys.external_grids[0].x1_pu = 0.0;
    sys.external_grids[0].r2_pu = 0.0;
    sys.external_grids[0].x2_pu = 0.0;
    sys.external_grids[0].r0_pu = 0.0;
    sys.external_grids[0].x0_pu = 0.0;
    sys.external_grids[0].s_sc_max_mva = 0.0;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("requires explicit impedance or short-circuit capacity"));
  }

  SECTION("external grid explicit phase voltage requires active-phase magnitude") {
    auto sys = build_per_phase_source_voltage_case();
    sys.external_grids[0].vm_b_pu = 0.0;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("requires explicit source voltage magnitude on active phase B"));
  }

  SECTION("external grid explicit phase impedance requires active-phase impedance") {
    auto sys = build_phase_impedance_source_case(true);
    sys.external_grids[0].r_b_pu = 0.0;
    sys.external_grids[0].x_b_pu = 0.0;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("requires explicit source impedance on active phase B"));
  }

  SECTION("external grid full phase-domain impedance matrix must stay inside phase mask") {
    auto sys = build_phase_matrix_source_case();
    sys.buses[0].phase_mask = PhaseMask::a();
    sys.external_grids[0].phase_mask = PhaseMask::a();
    hacdcpf::phase_matrix_set(sys.external_grids[0].r_matrix_pu, 0, 1, 0.01);
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("full phase-domain impedance matrix carries non-zero entries outside phase_mask"));
  }

  SECTION("closed-delta external grid voltage setpoint must stay compatible with topology") {
    auto sys = build_delta_topology_source_case();
    auto& source = sys.external_grids[0];
    source.vm_a_pu = 1.0;
    source.va_a_deg = 0.0;
    source.vm_b_pu = 1.0;
    source.va_b_deg = 0.0;
    source.vm_c_pu = 1.0;
    source.va_c_deg = 0.0;
    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("source voltage setpoint is incompatible with source_topology"));
  }
}

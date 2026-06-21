#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "hacdcpf/analysis/distribution_power_flow.hpp"

namespace {

using hacdcpf::BusType;
using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::ThreePhaseLoad;
using hacdcpf::analysis::ThreePhaseNROptions;
using hacdcpf::analysis::solve_three_phase_nr;

ThreePhaseACBus make_bus(int index, BusType type, PhaseMask mask) {
  ThreePhaseACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.phase_mask = mask;
  bus.in_service = true;
  bus.base_kv = mask.bits == PhaseMask::abc().bits ? 12.47 : 7.2;
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
    PhaseMask mask,
    double r_pu,
    double x_pu) {
  ThreePhaseACLine line;
  line.index = index;
  line.from_bus = from_bus;
  line.to_bus = to_bus;
  line.phase_mask = mask;
  line.in_service = true;
  line.r1_pu = r_pu;
  line.x1_pu = x_pu;
  line.r0_pu = r_pu;
  line.x0_pu = x_pu;
  line.rate_a_mva = 100.0;
  return line;
}

ThreePhaseLoad make_load(
    int index,
    int bus,
    PhaseMask mask,
    const std::string& connection) {
  ThreePhaseLoad load;
  load.index = index;
  load.bus = bus;
  load.phase_mask = mask;
  load.connection = connection;
  load.grounded = true;
  load.in_service = true;
  load.vmin_pu = 1e-6;
  load.vmax_pu = 2.0;
  load.zipv_cutoff_pu = 0.0;
  return load;
}

void set_legacy_zip(
    ThreePhaseLoad& load,
    double z_percent,
    double i_percent,
    double p_percent) {
  load.const_z_percent = z_percent;
  load.const_i_percent = i_percent;
  load.const_p_percent = p_percent;
}

void set_explicit_zip(
    ThreePhaseLoad& load,
    double p_z_percent,
    double p_i_percent,
    double p_p_percent,
    double q_z_percent,
    double q_i_percent,
    double q_p_percent) {
  load.p_const_z_percent = p_z_percent;
  load.p_const_i_percent = p_i_percent;
  load.p_const_p_percent = p_p_percent;
  load.q_const_z_percent = q_z_percent;
  load.q_const_i_percent = q_i_percent;
  load.q_const_p_percent = q_p_percent;
}

ThreePhaseACSystem build_wye_load_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.buses = {
      make_bus(1, BusType::SLACK, PhaseMask::abc()),
      make_bus(2, BusType::PQ, PhaseMask::abc()),
      make_bus(3, BusType::PQ, PhaseMask::abc()),
  };
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.10, 0.20),
      make_line(2, 2, 3, PhaseMask::abc(), 0.08, 0.15),
  };
  return sys;
}

ThreePhaseACSystem build_delta_load_case(PhaseMask load_mask = PhaseMask::abc()) {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.buses = {
      make_bus(1, BusType::SLACK, load_mask),
      make_bus(2, BusType::PQ, load_mask),
      make_bus(3, BusType::PQ, load_mask),
  };
  sys.lines = {
      make_line(1, 1, 2, load_mask, 0.10, 0.20),
      make_line(2, 2, 3, load_mask, 0.08, 0.15),
  };
  return sys;
}

ThreePhaseNROptions nr_options() {
  ThreePhaseNROptions options;
  options.max_iter = 80;
  options.tol = 1e-8;
  return options;
}

void set_source_vm(ThreePhaseACSystem& sys, double vm_pu) {
  sys.buses[0].vm_a_pu = vm_pu;
  sys.buses[0].vm_b_pu = vm_pu;
  sys.buses[0].vm_c_pu = vm_pu;
}

}  // namespace

TEST_CASE("Three-phase NR wye ZIP ordering follows impedance current power semantics",
          "[integration][powerflow][three_phase_nr][load]") {
  auto power_sys = build_wye_load_case();
  auto current_sys = build_wye_load_case();
  auto impedance_sys = build_wye_load_case();

  auto power_load = make_load(1, 3, PhaseMask::abc(), "wye");
  power_load.p_a_mw = 0.60; power_load.q_a_mvar = 0.24;
  power_load.p_b_mw = 0.48; power_load.q_b_mvar = 0.19;
  power_load.p_c_mw = 0.72; power_load.q_c_mvar = 0.29;
  set_legacy_zip(power_load, 0.0, 0.0, 100.0);

  auto current_load = power_load;
  set_legacy_zip(current_load, 0.0, 100.0, 0.0);

  auto impedance_load = power_load;
  set_legacy_zip(impedance_load, 100.0, 0.0, 0.0);

  power_sys.loads = {power_load};
  current_sys.loads = {current_load};
  impedance_sys.loads = {impedance_load};

  const auto power = solve_three_phase_nr(power_sys, nr_options());
  const auto current = solve_three_phase_nr(current_sys, nr_options());
  const auto impedance = solve_three_phase_nr(impedance_sys, nr_options());

  REQUIRE(power.converged);
  REQUIRE(current.converged);
  REQUIRE(impedance.converged);

  CHECK(impedance.bus_voltages[2].vm_a_pu > current.bus_voltages[2].vm_a_pu);
  CHECK(current.bus_voltages[2].vm_a_pu > power.bus_voltages[2].vm_a_pu);
  CHECK(impedance.bus_voltages[2].vm_b_pu > current.bus_voltages[2].vm_b_pu);
  CHECK(current.bus_voltages[2].vm_b_pu > power.bus_voltages[2].vm_b_pu);
  CHECK(impedance.bus_voltages[2].vm_c_pu > current.bus_voltages[2].vm_c_pu);
  CHECK(current.bus_voltages[2].vm_c_pu > power.bus_voltages[2].vm_c_pu);

  CHECK(impedance.branch_powers[1].p_a_mw < current.branch_powers[1].p_a_mw);
  CHECK(current.branch_powers[1].p_a_mw < power.branch_powers[1].p_a_mw);
}

TEST_CASE("Three-phase NR consumes separate active reactive ZIP weights and voltage windows",
          "[integration][powerflow][three_phase_nr][load]") {
  auto pure_power_sys = build_wye_load_case();
  auto split_zip_sys = build_wye_load_case();
  auto vmin_sys = build_wye_load_case();
  auto cutoff_sys = build_wye_load_case();
  auto no_load_sys = build_wye_load_case();

  auto load = make_load(1, 3, PhaseMask::abc(), "wye");
  load.p_a_mw = 0.62; load.q_a_mvar = 0.25;
  load.p_b_mw = 0.50; load.q_b_mvar = 0.20;
  load.p_c_mw = 0.58; load.q_c_mvar = 0.23;
  set_legacy_zip(load, 0.0, 0.0, 100.0);

  auto split = load;
  split.const_z_percent = 0.0;
  split.const_i_percent = 0.0;
  split.const_p_percent = 100.0;
  set_explicit_zip(split, 0.0, 0.0, 100.0, 100.0, 0.0, 0.0);

  auto vmin_load = load;
  vmin_load.vmin_pu = 1.03;

  auto cutoff_load = load;
  cutoff_load.zipv_cutoff_pu = 1.01;

  pure_power_sys.loads = {load};
  split_zip_sys.loads = {split};
  vmin_sys.loads = {vmin_load};
  cutoff_sys.loads = {cutoff_load};

  const auto pure_power = solve_three_phase_nr(pure_power_sys, nr_options());
  const auto split_zip = solve_three_phase_nr(split_zip_sys, nr_options());
  const auto vmin_result = solve_three_phase_nr(vmin_sys, nr_options());
  const auto cutoff_result = solve_three_phase_nr(cutoff_sys, nr_options());
  const auto no_load = solve_three_phase_nr(no_load_sys, nr_options());

  REQUIRE(pure_power.converged);
  REQUIRE(split_zip.converged);
  REQUIRE(vmin_result.converged);
  REQUIRE(cutoff_result.converged);
  REQUIRE(no_load.converged);

  CHECK(split_zip.bus_voltages[2].vm_a_pu > pure_power.bus_voltages[2].vm_a_pu);
  CHECK(split_zip.branch_powers[1].q_a_mvar < pure_power.branch_powers[1].q_a_mvar);

  CHECK(vmin_result.bus_voltages[2].vm_a_pu > pure_power.bus_voltages[2].vm_a_pu);
  CHECK(vmin_result.branch_powers[1].p_a_mw < pure_power.branch_powers[1].p_a_mw);

  CHECK(std::abs(cutoff_result.bus_voltages[2].vm_a_pu - no_load.bus_voltages[2].vm_a_pu) <
        1e-9);
  CHECK(std::abs(cutoff_result.bus_voltages[2].vm_b_pu - no_load.bus_voltages[2].vm_b_pu) <
        1e-9);
  CHECK(std::abs(cutoff_result.bus_voltages[2].vm_c_pu - no_load.bus_voltages[2].vm_c_pu) <
        1e-9);
  CHECK(std::abs(cutoff_result.branch_powers[1].p_a_mw - no_load.branch_powers[1].p_a_mw) <
        1e-9);
  CHECK(std::abs(cutoff_result.branch_powers[1].q_a_mvar - no_load.branch_powers[1].q_a_mvar) <
        1e-9);
}

TEST_CASE("Three-phase NR open-neutral wye honors missing return path and neutral shift",
          "[integration][powerflow][three_phase_nr][load][neutral]") {
  SECTION("single effective phase with open neutral disconnects") {
    auto open_sys = build_wye_load_case();
    auto no_load_sys = build_wye_load_case();

    auto load = make_load(1, 3, PhaseMask::abc(), "wye");
    load.grounded = false;
    load.p_a_mw = 0.55;
    load.q_a_mvar = 0.22;
    set_legacy_zip(load, 0.0, 0.0, 100.0);
    open_sys.loads = {load};

    const auto open_result = solve_three_phase_nr(open_sys, nr_options());
    const auto no_load_result = solve_three_phase_nr(no_load_sys, nr_options());

    REQUIRE(open_result.converged);
    REQUIRE(no_load_result.converged);

    CHECK(std::abs(open_result.bus_voltages[2].vm_a_pu - no_load_result.bus_voltages[2].vm_a_pu) <
          1e-9);
    CHECK(std::abs(open_result.branch_powers[1].p_a_mw - no_load_result.branch_powers[1].p_a_mw) <
          1e-9);
  }

  SECTION("two-phase open neutral differs from solid-grounded wye") {
    auto open_sys = build_wye_load_case();
    auto grounded_sys = build_wye_load_case();

    auto open_load = make_load(1, 3, PhaseMask::ab(), "wye");
    open_load.grounded = false;
    open_load.p_a_mw = 0.65;
    open_load.q_a_mvar = 0.26;
    open_load.p_b_mw = 0.28;
    open_load.q_b_mvar = 0.11;
    set_legacy_zip(open_load, 0.0, 0.0, 100.0);

    auto grounded_load = open_load;
    grounded_load.grounded = true;

    open_sys.loads = {open_load};
    grounded_sys.loads = {grounded_load};

    const auto open_result = solve_three_phase_nr(open_sys, nr_options());
    const auto grounded_result = solve_three_phase_nr(grounded_sys, nr_options());

    REQUIRE(open_result.converged);
    REQUIRE(grounded_result.converged);

    CHECK(std::abs(open_result.bus_voltages[2].vm_a_pu -
                   grounded_result.bus_voltages[2].vm_a_pu) > 1e-4);
    CHECK(std::abs(open_result.bus_voltages[2].vm_b_pu -
                   grounded_result.bus_voltages[2].vm_b_pu) > 1e-4);
    CHECK(std::abs(open_result.branch_powers[1].p_a_mw -
                   grounded_result.branch_powers[1].p_a_mw) > 1e-4);
  }
}

TEST_CASE("Three-phase NR impedance-grounded wye includes neutral loss path",
          "[integration][powerflow][three_phase_nr][load][neutral]") {
  auto impedance_sys = build_wye_load_case();
  auto solid_sys = build_wye_load_case();

  auto impedance_load = make_load(1, 3, PhaseMask::abc(), "wye");
  impedance_load.p_a_mw = 0.55;
  impedance_load.q_a_mvar = 0.22;
  impedance_load.r_neut_ohm = 5.0;
  impedance_load.x_neut_ohm = 1.5;
  set_legacy_zip(impedance_load, 0.0, 0.0, 100.0);

  auto solid_load = impedance_load;
  solid_load.r_neut_ohm = 0.0;
  solid_load.x_neut_ohm = 0.0;

  impedance_sys.loads = {impedance_load};
  solid_sys.loads = {solid_load};

  const auto impedance_result = solve_three_phase_nr(impedance_sys, nr_options());
  const auto solid_result = solve_three_phase_nr(solid_sys, nr_options());

  REQUIRE(impedance_result.converged);
  REQUIRE(solid_result.converged);

  CHECK(impedance_result.branch_powers[1].p_a_mw > solid_result.branch_powers[1].p_a_mw);
  CHECK(impedance_result.bus_voltages[2].vm_a_pu < solid_result.bus_voltages[2].vm_a_pu);
}

TEST_CASE("Three-phase NR honors vmax high-voltage load window",
          "[integration][powerflow][three_phase_nr][load][vmax]") {
  auto capped_sys = build_wye_load_case();
  auto uncapped_sys = build_wye_load_case();

  set_source_vm(capped_sys, 1.10);
  set_source_vm(uncapped_sys, 1.10);

  auto capped_load = make_load(1, 3, PhaseMask::abc(), "wye");
  capped_load.p_a_mw = 0.30; capped_load.q_a_mvar = 0.12;
  capped_load.p_b_mw = 0.30; capped_load.q_b_mvar = 0.12;
  capped_load.p_c_mw = 0.30; capped_load.q_c_mvar = 0.12;
  capped_load.vmax_pu = 1.01;
  set_legacy_zip(capped_load, 0.0, 0.0, 100.0);

  auto uncapped_load = capped_load;
  uncapped_load.vmax_pu = 2.0;

  capped_sys.loads = {capped_load};
  uncapped_sys.loads = {uncapped_load};

  const auto capped_result = solve_three_phase_nr(capped_sys, nr_options());
  const auto uncapped_result = solve_three_phase_nr(uncapped_sys, nr_options());

  REQUIRE(capped_result.converged);
  REQUIRE(uncapped_result.converged);

  CHECK(capped_result.branch_powers[1].p_a_mw > uncapped_result.branch_powers[1].p_a_mw);
  CHECK(capped_result.bus_voltages[2].vm_a_pu < uncapped_result.bus_voltages[2].vm_a_pu);
}

TEST_CASE("Three-phase NR delta power path is distinct from wye phase split",
          "[integration][powerflow][three_phase_nr][load]") {
  auto delta_sys = build_delta_load_case(PhaseMask::ab());
  auto wye_sys = build_delta_load_case(PhaseMask::ab());

  auto delta_load = make_load(1, 3, PhaseMask::ab(), "delta");
  delta_load.p_a_mw = 0.45;
  delta_load.q_a_mvar = 0.18;
  delta_load.p_b_mw = 0.30;
  delta_load.q_b_mvar = 0.12;
  set_legacy_zip(delta_load, 0.0, 0.0, 100.0);

  auto wye_load = make_load(1, 3, PhaseMask::ab(), "wye");
  wye_load.p_a_mw = delta_load.p_a_mw;
  wye_load.q_a_mvar = delta_load.q_a_mvar;
  wye_load.p_b_mw = delta_load.p_b_mw;
  wye_load.q_b_mvar = delta_load.q_b_mvar;
  set_legacy_zip(wye_load, 0.0, 0.0, 100.0);

  delta_sys.loads = {delta_load};
  wye_sys.loads = {wye_load};

  const auto delta = solve_three_phase_nr(delta_sys, nr_options());
  const auto wye = solve_three_phase_nr(wye_sys, nr_options());

  REQUIRE(delta.converged);
  REQUIRE(wye.converged);

  CHECK(std::abs(delta.bus_voltages[2].vm_a_pu - wye.bus_voltages[2].vm_a_pu) > 1e-3);
  CHECK(std::abs(delta.branch_powers[1].p_a_mw - wye.branch_powers[1].p_a_mw) > 1e-3);
  CHECK(std::abs(delta.branch_powers[1].q_a_mvar - wye.branch_powers[1].q_a_mvar) > 1e-3);

  CHECK(std::abs(delta.bus_voltages[2].vm_c_pu) <= 1e-12);
  CHECK(std::abs(delta.branch_powers[1].p_c_mw) <= 1e-12);
}

TEST_CASE("Three-phase NR delta ZIP ordering and cutoff follow line-line load semantics",
          "[integration][powerflow][three_phase_nr][load]") {
  auto power_sys = build_delta_load_case();
  auto impedance_sys = build_delta_load_case();
  auto cutoff_sys = build_delta_load_case();
  auto no_load_sys = build_delta_load_case();

  auto power = make_load(1, 3, PhaseMask::abc(), "delta");
  power.p_a_mw = 0.24; power.q_a_mvar = 0.10;
  power.p_b_mw = 0.27; power.q_b_mvar = 0.11;
  power.p_c_mw = 0.22; power.q_c_mvar = 0.09;
  set_legacy_zip(power, 0.0, 0.0, 100.0);

  auto impedance = power;
  set_legacy_zip(impedance, 100.0, 0.0, 0.0);

  auto cutoff = power;
  cutoff.zipv_cutoff_pu = 1.01;

  power_sys.loads = {power};
  impedance_sys.loads = {impedance};
  cutoff_sys.loads = {cutoff};

  const auto power_result = solve_three_phase_nr(power_sys, nr_options());
  const auto impedance_result = solve_three_phase_nr(impedance_sys, nr_options());
  const auto cutoff_result = solve_three_phase_nr(cutoff_sys, nr_options());
  const auto no_load_result = solve_three_phase_nr(no_load_sys, nr_options());

  REQUIRE(power_result.converged);
  REQUIRE(impedance_result.converged);
  REQUIRE(cutoff_result.converged);
  REQUIRE(no_load_result.converged);

  CHECK(impedance_result.bus_voltages[2].vm_a_pu > power_result.bus_voltages[2].vm_a_pu);
  CHECK(impedance_result.bus_voltages[2].vm_b_pu > power_result.bus_voltages[2].vm_b_pu);
  CHECK(impedance_result.bus_voltages[2].vm_c_pu > power_result.bus_voltages[2].vm_c_pu);

  CHECK(std::abs(cutoff_result.bus_voltages[2].vm_a_pu - no_load_result.bus_voltages[2].vm_a_pu) <
        1e-9);
  CHECK(std::abs(cutoff_result.branch_powers[1].p_a_mw - no_load_result.branch_powers[1].p_a_mw) <
        1e-9);
}

TEST_CASE("Three-phase NR fail-closes unsupported load contracts",
          "[unit][powerflow][three_phase_nr][load][fail_close]") {
  SECTION("legacy and explicit ZIP weights cannot be mixed") {
    auto sys = build_wye_load_case();
    auto load = make_load(1, 3, PhaseMask::abc(), "wye");
    load.p_a_mw = 0.5;
    load.q_a_mvar = 0.2;
    load.const_z_percent = 20.0;
    load.const_i_percent = 0.0;
    load.const_p_percent = 80.0;
    set_explicit_zip(load, 0.0, 0.0, 100.0, 0.0, 0.0, 100.0);
    sys.loads = {load};

    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("mixes legacy shared ZIP weights"));
  }

  SECTION("delta load requires at least two active phases") {
    auto sys = build_delta_load_case(PhaseMask::a());
    auto load = make_load(1, 3, PhaseMask::a(), "delta");
    load.p_a_mw = 0.5;
    load.q_a_mvar = 0.2;
    sys.loads = {load};

    REQUIRE_THROWS_WITH(
        solve_three_phase_nr(sys, nr_options()),
        Catch::Matchers::ContainsSubstring("requires at least two active phases"));
  }
}

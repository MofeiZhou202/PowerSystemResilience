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
using hacdcpf::ThreePhaseGenerator;
using hacdcpf::analysis::PhaseNodeIndexer;
using hacdcpf::analysis::ThreePhaseNROptions;
using hacdcpf::analysis::solve_three_phase_nr;

ThreePhaseACBus make_bus(int index, BusType type, PhaseMask mask) {
  ThreePhaseACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.phase_mask = mask;
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

ThreePhaseACLine make_full_matrix_line(
    int index,
    int from_bus,
    int to_bus,
    PhaseMask mask) {
  ThreePhaseACLine line;
  line.index = index;
  line.from_bus = from_bus;
  line.to_bus = to_bus;
  line.phase_mask = mask;
  line.in_service = true;
  line.use_phase_matrix = true;
  line.rate_a_mva = 100.0;

  hacdcpf::phase_matrix_set(line.r_matrix_pu, 0, 0, 0.035);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 0, 1, 0.011);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 0, 2, 0.009);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 1, 0, 0.011);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 1, 1, 0.038);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 1, 2, 0.010);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 2, 0, 0.009);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 2, 1, 0.010);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 2, 2, 0.036);

  hacdcpf::phase_matrix_set(line.x_matrix_pu, 0, 0, 0.110);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 0, 1, 0.035);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 0, 2, 0.031);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 1, 0, 0.035);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 1, 1, 0.116);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 1, 2, 0.033);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 2, 0, 0.031);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 2, 1, 0.033);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 2, 2, 0.112);

  return line;
}

ThreePhaseACSystem build_single_phase_lateral_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto bus1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto bus2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  bus2.pd_a_mw = 0.20; bus2.qd_a_mvar = 0.08;
  bus2.pd_b_mw = 0.20; bus2.qd_b_mvar = 0.08;
  bus2.pd_c_mw = 0.20; bus2.qd_c_mvar = 0.08;

  auto bus3 = make_bus(3, BusType::PQ, PhaseMask::a());
  bus3.pd_a_mw = 0.12;
  bus3.qd_a_mvar = 0.05;

  sys.buses = {bus1, bus2, bus3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.04, 0.08),
      make_line(2, 2, 3, PhaseMask::a(), 0.05, 0.10),
  };
  return sys;
}

ThreePhaseACSystem build_two_phase_branch_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto bus1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto bus2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  bus2.pd_a_mw = 0.10; bus2.qd_a_mvar = 0.04;
  bus2.pd_b_mw = 0.10; bus2.qd_b_mvar = 0.04;
  bus2.pd_c_mw = 0.10; bus2.qd_c_mvar = 0.04;

  auto bus3 = make_bus(3, BusType::PQ, PhaseMask::ab());
  bus3.pd_a_mw = 0.18; bus3.qd_a_mvar = 0.07;
  bus3.pd_b_mw = 0.11; bus3.qd_b_mvar = 0.05;

  sys.buses = {bus1, bus2, bus3};
  sys.lines = {
      make_line(1, 1, 2, PhaseMask::abc(), 0.03, 0.06),
      make_line(2, 2, 3, PhaseMask::ab(), 0.05, 0.09),
  };
  return sys;
}

ThreePhaseACSystem build_full_matrix_line_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto bus1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto bus2 = make_bus(2, BusType::PQ, PhaseMask::abc());
  bus2.pd_a_mw = 0.60; bus2.qd_a_mvar = 0.22;
  bus2.pd_b_mw = 0.35; bus2.qd_b_mvar = 0.14;
  bus2.pd_c_mw = 0.25; bus2.qd_c_mvar = 0.09;

  sys.buses = {bus1, bus2};
  sys.lines = {make_full_matrix_line(1, 1, 2, PhaseMask::abc())};
  return sys;
}

ThreePhaseACSystem build_two_phase_full_matrix_line_case() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;

  auto bus1 = make_bus(1, BusType::SLACK, PhaseMask::abc());
  auto bus2 = make_bus(2, BusType::PQ, PhaseMask::ab());
  bus2.pd_a_mw = 0.28; bus2.qd_a_mvar = 0.10;
  bus2.pd_b_mw = 0.19; bus2.qd_b_mvar = 0.07;

  auto line = make_full_matrix_line(1, 1, 2, PhaseMask::ab());
  line.r_matrix_pu = {};
  line.x_matrix_pu = {};
  line.b_matrix_pu = {};
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 0, 0, 0.041);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 0, 1, 0.013);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 1, 0, 0.013);
  hacdcpf::phase_matrix_set(line.r_matrix_pu, 1, 1, 0.044);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 0, 0, 0.121);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 0, 1, 0.037);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 1, 0, 0.037);
  hacdcpf::phase_matrix_set(line.x_matrix_pu, 1, 1, 0.126);

  sys.buses = {bus1, bus2};
  sys.lines = {line};
  return sys;
}

ThreePhaseNROptions nr_options() {
  ThreePhaseNROptions opt;
  opt.max_iter = 50;
  opt.tol = 1e-8;
  return opt;
}

}  // namespace

TEST_CASE("Three-phase NR builds compact nodes for single-phase lateral", "[integration][powerflow][three_phase_nr]") {
  const auto sys = build_single_phase_lateral_case();
  const auto indexer = PhaseNodeIndexer::build(sys);

  REQUIRE(indexer.total_nodes == 7);
  REQUIRE(indexer.has_node(2, 0));
  REQUIRE_FALSE(indexer.has_node(2, 1));
  REQUIRE_FALSE(indexer.has_node(2, 2));

  const auto result = solve_three_phase_nr(sys, nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 3);
  REQUIRE(result.branch_powers.size() == 2);

  REQUIRE(result.bus_voltages[2].vm_a_pu > 0.0);
  REQUIRE(std::abs(result.bus_voltages[2].vm_b_pu) <= 1e-12);
  REQUIRE(std::abs(result.bus_voltages[2].vm_c_pu) <= 1e-12);

  REQUIRE(std::abs(result.branch_powers[1].p_a_mw) > 1e-6);
  REQUIRE(std::abs(result.branch_powers[1].q_a_mvar) > 1e-6);
  REQUIRE(std::abs(result.branch_powers[1].p_b_mw) <= 1e-12);
  REQUIRE(std::abs(result.branch_powers[1].q_b_mvar) <= 1e-12);
  REQUIRE(std::abs(result.branch_powers[1].p_c_mw) <= 1e-12);
  REQUIRE(std::abs(result.branch_powers[1].q_c_mvar) <= 1e-12);
}

TEST_CASE("Three-phase NR supports two-phase branch without phantom phase-C node", "[integration][powerflow][three_phase_nr]") {
  const auto sys = build_two_phase_branch_case();
  const auto indexer = PhaseNodeIndexer::build(sys);

  REQUIRE(indexer.total_nodes == 8);
  REQUIRE(indexer.has_node(2, 0));
  REQUIRE(indexer.has_node(2, 1));
  REQUIRE_FALSE(indexer.has_node(2, 2));

  const auto result = solve_three_phase_nr(sys, nr_options());
  REQUIRE(result.converged);

  REQUIRE(result.bus_voltages[2].vm_a_pu > 0.0);
  REQUIRE(result.bus_voltages[2].vm_b_pu > 0.0);
  REQUIRE(std::abs(result.bus_voltages[2].vm_c_pu) <= 1e-12);

  REQUIRE(std::abs(result.branch_powers[1].p_a_mw) > 1e-6);
  REQUIRE(std::abs(result.branch_powers[1].p_b_mw) > 1e-6);
  REQUIRE(std::abs(result.branch_powers[1].p_c_mw) <= 1e-12);
  REQUIRE(std::abs(result.branch_powers[1].q_c_mvar) <= 1e-12);
}

TEST_CASE("Three-phase NR rejects generator aggregate and per-phase mixed dispatch", "[unit][powerflow][three_phase_nr]") {
  auto sys = build_single_phase_lateral_case();

  ThreePhaseGenerator generator;
  generator.index = 1;
  generator.bus = 2;
  generator.in_service = true;
  generator.phase_mask = PhaseMask::abc();
  generator.p_mw = 0.30;
  generator.q_mvar = 0.05;
  generator.p_a_mw = 0.10;
  generator.q_a_mvar = 0.02;
  sys.generators.push_back(generator);

  REQUIRE_THROWS_WITH(
      solve_three_phase_nr(sys, nr_options()),
      Catch::Matchers::ContainsSubstring("mixes aggregate and per-phase dispatch fields"));
}

TEST_CASE("Three-phase NR consumes full phase-domain line matrices on the compact NR path",
          "[integration][powerflow][three_phase_nr]") {
  const auto sys = build_full_matrix_line_case();

  REQUIRE(sys.lines[0].use_phase_matrix);
  REQUIRE(sys.lines[0].r1_pu == 0.0);
  REQUIRE(sys.lines[0].x1_pu == 0.0);
  REQUIRE(sys.lines[0].r0_pu == 0.0);
  REQUIRE(sys.lines[0].x0_pu == 0.0);

  const auto result = solve_three_phase_nr(sys, nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.branch_powers.size() == 1);

  const auto& bus2 = result.bus_voltages[1];
  CHECK(bus2.vm_a_pu < 1.0);
  CHECK(bus2.vm_b_pu < 1.0);
  CHECK(bus2.vm_c_pu < 1.0);
  CHECK(std::abs(bus2.vm_a_pu - bus2.vm_b_pu) > 1e-5);
  CHECK(std::abs(bus2.vm_b_pu - bus2.vm_c_pu) > 1e-5);

  const auto& branch = result.branch_powers[0];
  CHECK(std::abs(branch.p_a_mw) > 1e-4);
  CHECK(std::abs(branch.p_b_mw) > 1e-4);
  CHECK(std::abs(branch.p_c_mw) > 1e-4);
}

TEST_CASE("Three-phase NR extracts mixed-phase line submatrices from full phase-domain matrices",
          "[integration][powerflow][three_phase_nr]") {
  const auto sys = build_two_phase_full_matrix_line_case();
  const auto indexer = PhaseNodeIndexer::build(sys);

  REQUIRE(indexer.total_nodes == 5);
  REQUIRE(indexer.has_node(1, 0));
  REQUIRE(indexer.has_node(1, 1));
  REQUIRE_FALSE(indexer.has_node(1, 2));

  const auto result = solve_three_phase_nr(sys, nr_options());
  REQUIRE(result.converged);
  REQUIRE(result.bus_voltages.size() == 2);
  REQUIRE(result.branch_powers.size() == 1);

  CHECK(result.bus_voltages[1].vm_a_pu < 1.0);
  CHECK(result.bus_voltages[1].vm_b_pu < 1.0);
  CHECK(std::abs(result.bus_voltages[1].vm_c_pu) <= 1e-12);
  CHECK(std::abs(result.branch_powers[0].p_a_mw) > 1e-4);
  CHECK(std::abs(result.branch_powers[0].p_b_mw) > 1e-4);
  CHECK(std::abs(result.branch_powers[0].p_c_mw) <= 1e-12);
}

TEST_CASE("Three-phase NR rejects full line matrices that leak outside phase_mask",
          "[unit][powerflow][three_phase_nr]") {
  auto sys = build_two_phase_branch_case();
  sys.lines[1].use_phase_matrix = true;
  hacdcpf::phase_matrix_set(sys.lines[1].r_matrix_pu, 0, 0, 0.05);
  hacdcpf::phase_matrix_set(sys.lines[1].r_matrix_pu, 0, 1, 0.01);
  hacdcpf::phase_matrix_set(sys.lines[1].r_matrix_pu, 1, 0, 0.01);
  hacdcpf::phase_matrix_set(sys.lines[1].r_matrix_pu, 1, 1, 0.05);
  hacdcpf::phase_matrix_set(sys.lines[1].x_matrix_pu, 0, 0, 0.10);
  hacdcpf::phase_matrix_set(sys.lines[1].x_matrix_pu, 0, 1, 0.03);
  hacdcpf::phase_matrix_set(sys.lines[1].x_matrix_pu, 1, 0, 0.03);
  hacdcpf::phase_matrix_set(sys.lines[1].x_matrix_pu, 1, 1, 0.10);
  hacdcpf::phase_matrix_set(sys.lines[1].r_matrix_pu, 2, 2, 0.02);

  REQUIRE_THROWS_WITH(
      solve_three_phase_nr(sys, nr_options()),
      Catch::Matchers::ContainsSubstring("outside phase_mask"));
}

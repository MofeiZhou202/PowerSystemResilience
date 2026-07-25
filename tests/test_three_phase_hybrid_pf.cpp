#include <cmath>
// MinGW GCC in strict -std=c++20 mode does not define M_PI via <cmath>.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/power_flow/three_phase_hybrid.hpp"

namespace {

using Complex = std::complex<double>;
using hacdcpf::powerflow::ThreePhaseHybridPFCase;
using hacdcpf::powerflow::ThreePhaseHybridPFControlMode;
using hacdcpf::powerflow::ThreePhaseHybridPFConverter;
using hacdcpf::powerflow::ThreePhaseHybridPFOptions;
using hacdcpf::powerflow::solve_three_phase_hybrid_pf;

ThreePhaseHybridPFCase make_case(ThreePhaseHybridPFControlMode mode) {
  ThreePhaseHybridPFCase c;
  c.name = "two_bus_unbalanced_hybrid";
  c.base_mva = 1.0;
  constexpr int n = 6;
  c.y_ac.resize(n, n);
  std::vector<Eigen::Triplet<Complex>> trips;
  const Complex y = 1.0 / Complex{0.025, 0.08};
  for (int phase = 0; phase < 3; ++phase) {
    const int source = phase;
    const int load = 3 + phase;
    trips.emplace_back(source, source, y);
    trips.emplace_back(source, load, -y);
    trips.emplace_back(load, source, -y);
    trips.emplace_back(load, load, y);
  }
  c.y_ac.setFromTriplets(trips.begin(), trips.end());
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);
  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  c.p_load_pu.segment<3>(3) << 0.18, 0.12, 0.08;
  c.q_load_pu.segment<3>(3) << 0.06, 0.04, 0.025;
  const double angle[3] = {0.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0};
  c.voltage_start.resize(n);
  for (int phase = 0; phase < 3; ++phase) {
    c.voltage_start[phase] = std::polar(1.0, angle[phase]);
    c.voltage_start[3 + phase] = std::polar(0.98, angle[phase]);
  }
  c.ac_phase_index = {0, 1, 2, 0, 1, 2};
  c.reference_nodes = {0, 1, 2};
  c.reference_voltage.resize(3);
  for (int phase = 0; phase < 3; ++phase) {
    c.reference_voltage[phase] = std::polar(1.0, angle[phase]);
  }

  c.g_dc.resize(2, 2);
  const double conductance = 20.0;
  std::vector<Eigen::Triplet<double>> dc_trips = {
      {0, 0, conductance}, {0, 1, -conductance},
      {1, 0, -conductance}, {1, 1, conductance}};
  c.g_dc.setFromTriplets(dc_trips.begin(), dc_trips.end());
  c.p_dc_load_pu = Eigen::VectorXd::Zero(2);
  c.p_dc_load_pu[1] = 0.15;
  c.v_dc_start = Eigen::VectorXd::Ones(2);
  c.v_dc_start[1] = 0.992;

  ThreePhaseHybridPFConverter converter;
  converter.phase_nodes = {3, 4, 5};
  converter.dc_terminal = 0;
  converter.efficiency = 0.98;
  converter.control_mode = mode;
  converter.p_set_pu = -0.16;
  converter.q_set_pu = 0.0;
  converter.v_dc_set_pu = 1.0;
  converter.internal_voltage_positive = std::polar(0.985, -0.02);
  converter.virtual_r_pu = 0.01;
  converter.virtual_x_pu = 0.10;
  c.converters = {converter};
  return c;
}

ThreePhaseHybridPFOptions options() {
  ThreePhaseHybridPFOptions result;
  result.max_iterations = 80;
  result.tolerance = 1e-10;
  return result;
}

}  // namespace

TEST_CASE("Coupled unbalanced AC/DC PF solves GFL Vdc-Q control",
          "[power_flow][three_phase][hybrid][gfl]") {
  const auto result = solve_three_phase_hybrid_pf(
      make_case(ThreePhaseHybridPFControlMode::GridFollowingVdcQ), options());
  INFO(result.status);
  REQUIRE(result.converged);
  CHECK(result.residual < 1e-9);
  CHECK(result.ac_active_balance_residual < 1e-9);
  CHECK(result.ac_reactive_balance_residual < 1e-9);
  CHECK(result.dc_balance_residual < 1e-9);
  REQUIRE(result.converters.size() == 1);
  CHECK(result.converters.front().p_dc_pu > 0.14);
  CHECK(result.converters.front().current_vuf < 1e-9);
  CHECK(result.dc_voltage[0] == Catch::Approx(1.0).margin(1e-10));
  CHECK(result.voltage[3] != result.voltage[4]);
}

TEST_CASE("Coupled unbalanced AC/DC PF solves GFM Vdc-internal-voltage control",
          "[power_flow][three_phase][hybrid][gfm]") {
  const auto result = solve_three_phase_hybrid_pf(
      make_case(ThreePhaseHybridPFControlMode::GridFormingVdc), options());
  INFO(result.status);
  REQUIRE(result.converged);
  CHECK(result.residual < 1e-9);
  CHECK(result.dc_balance_residual < 1e-9);
  REQUIRE(result.converters.size() == 1);
  CHECK(result.converters.front().p_dc_pu > 0.14);
  CHECK(std::abs(result.converters.front().internal_voltage_positive) ==
        Catch::Approx(0.985).margin(1e-10));
  CHECK(result.converters.front().current_vuf > 1e-5);
  CHECK(result.dc_voltage[0] == Catch::Approx(1.0).margin(1e-10));
}

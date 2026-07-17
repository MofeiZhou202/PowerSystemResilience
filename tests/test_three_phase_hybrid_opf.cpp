#include <cmath>
#include <complex>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

namespace {

using hacdcpf::graph::SparseComplexMatrix;
using namespace hacdcpf::opf::phase_hybrid;
using Complex = std::complex<double>;

ThreePhaseHybridOPFCase make_small_hybrid_case() {
  ThreePhaseHybridOPFCase c;
  c.name = "three_bus_phase_hybrid";
  c.base_mva = 1.0;
  const int n = 9;
  std::vector<Eigen::Triplet<Complex>> ytrip;
  const auto add_branch = [&](int from, int to, Complex y) {
    ytrip.emplace_back(from, from, y);
    ytrip.emplace_back(to, to, y);
    ytrip.emplace_back(from, to, -y);
    ytrip.emplace_back(to, from, -y);
  };
  for (int phase = 0; phase < 3; ++phase) {
    add_branch(phase, 3 + phase, {8.0, -16.0});
    add_branch(3 + phase, 6 + phase, {6.0, -12.0});
  }
  c.y_ac.resize(n, n);
  c.y_ac.setFromTriplets(ytrip.begin(), ytrip.end());
  c.y_ac.makeCompressed();
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);

  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  for (int phase = 0; phase < 3; ++phase) {
    c.p_load_pu[6 + phase] = 0.08 + 0.01 * phase;
    c.q_load_pu[6 + phase] = 0.025 + 0.005 * phase;
  }
  c.v_min_pu = Eigen::VectorXd::Constant(n, 0.85);
  c.v_max_pu = Eigen::VectorXd::Constant(n, 1.10);
  c.voltage_start.resize(n);
  const double angles[3] = {0.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0};
  for (int bus = 0; bus < 3; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      c.voltage_start[3 * bus + phase] =
          std::polar(1.0 - 0.01 * bus, angles[phase]);
    }
  }
  c.reference_nodes = {0, 1, 2};
  c.reference_voltage.resize(3);
  for (int phase = 0; phase < 3; ++phase) {
    c.reference_voltage[phase] = std::polar(1.0, angles[phase]);
  }
  c.three_phase_bus_nodes = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
  c.vuf_max = 0.05;

  for (int phase = 0; phase < 3; ++phase) {
    PhaseGenerator generator;
    generator.phase_node = phase;
    generator.p_min_pu = -0.2;
    generator.p_max_pu = 1.0;
    generator.q_min_pu = -1.0;
    generator.q_max_pu = 1.0;
    generator.cost_c2 = 0.1;
    generator.cost_c1 = 30.0;
    c.generators.push_back(generator);
  }

  c.g_dc.resize(2, 2);
  std::vector<Eigen::Triplet<double>> gtrip{
      {0, 0, 50.0}, {0, 1, -50.0}, {1, 0, -50.0}, {1, 1, 50.0}};
  c.g_dc.setFromTriplets(gtrip.begin(), gtrip.end());
  c.p_dc_load_pu = Eigen::VectorXd::Zero(2);
  c.p_dc_load_pu[1] = 0.10;
  c.v_dc_start = Eigen::VectorXd(2);
  c.v_dc_start << 1.0, 0.997996;
  c.v_dc_min_pu = Eigen::VectorXd::Constant(2, 0.90);
  c.v_dc_max_pu = Eigen::VectorXd::Constant(2, 1.10);

  PhaseVSC converter;
  converter.phase_nodes = {6, 7, 8};
  converter.dc_terminal = 0;
  converter.efficiency = 0.98;
  converter.s_max_pu = 0.25;
  c.converters.push_back(converter);
  return c;
}

}  // namespace

TEST_CASE("Monolithic phase hybrid OPF Full and GR recover the same solution",
          "[opf][three_phase][hybrid][kron]") {
  const auto c = make_small_hybrid_case();
  ThreePhaseHybridOPFOptions full_options;
  full_options.variant = ModelVariant::Full;
  full_options.backend = SolverBackend::Ipopt;
  full_options.max_iterations = 300;
  full_options.tolerance = 1e-7;

  ThreePhaseHybridOPFOptions reduced_options = full_options;
  reduced_options.variant = ModelVariant::GraphReduced;
  reduced_options.reduction_options.max_front = 8;
  reduced_options.reduction_options.max_nnz_ratio = 10.0;

  const auto full = solve_three_phase_hybrid_opf(c, full_options);
  const auto reduced = solve_three_phase_hybrid_opf(c, reduced_options);
  INFO("full status=" << full.status << " residual=" << full.primal_residual);
  INFO("reduced status=" << reduced.status << " residual=" << reduced.primal_residual);
  REQUIRE(full.converged);
  REQUIRE(reduced.converged);
  CHECK(reduced.eliminated_phase_nodes == 3);
  CHECK(std::abs(full.objective - reduced.objective) <=
        1e-6 * std::max(1.0, std::abs(full.objective)));
  REQUIRE(full.full_voltage.size() == reduced.full_voltage.size());
  CHECK((full.full_voltage - reduced.full_voltage).cwiseAbs().maxCoeff() < 1e-6);
  CHECK(full.max_voltage_violation < 1e-8);
  CHECK(reduced.max_voltage_violation < 1e-8);
  CHECK(full.max_vuf <= c.vuf_max + 1e-7);
  CHECK(reduced.max_vuf <= c.vuf_max + 1e-7);
}

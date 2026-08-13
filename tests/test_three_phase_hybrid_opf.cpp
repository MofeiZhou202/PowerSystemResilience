#include <cmath>
// MinGW GCC in strict -std=c++20 mode does not define M_PI via <cmath>.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/dynamics/NetworkState.hpp"
#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_relaxation.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace {

using hacdcpf::graph::SparseComplexMatrix;
using namespace hacdcpf::opf::phase_hybrid;
using namespace hacdcpf::dynamics;
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
  c.ac_phase_index.resize(n);
  const double angles[3] = {0.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0};
  for (int bus = 0; bus < 3; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      c.voltage_start[3 * bus + phase] =
          std::polar(1.0 - 0.01 * bus, angles[phase]);
      c.ac_phase_index[static_cast<std::size_t>(3 * bus + phase)] = phase;
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
  c.dc_reference_terminals = {0};
  c.dc_reference_voltage_pu = Eigen::VectorXd::Ones(1);
  PhaseVSC converter;
  converter.phase_nodes = {6, 7, 8};
  converter.dc_terminal = 0;
  converter.efficiency = 0.98;
  converter.s_max_pu = 0.25;
  c.converters.push_back(converter);
  return c;
}

ThreePhaseHybridOPFCase make_large_sparse_hybrid_case(int bus_count) {
  ThreePhaseHybridOPFCase c;
  c.name = "large_sparse_phase_hybrid";
  c.base_mva = 1.0;
  const int n = 3 * bus_count;
  std::vector<Eigen::Triplet<Complex>> ytrip;
  ytrip.reserve(static_cast<std::size_t>(12 * bus_count));
  const Complex admittance{8.0, -16.0};
  for (int bus = 0; bus + 1 < bus_count; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      const int from = 3 * bus + phase;
      const int to = 3 * (bus + 1) + phase;
      ytrip.emplace_back(from, from, admittance);
      ytrip.emplace_back(to, to, admittance);
      ytrip.emplace_back(from, to, -admittance);
      ytrip.emplace_back(to, from, -admittance);
    }
  }
  c.y_ac.resize(n, n);
  c.y_ac.setFromTriplets(ytrip.begin(), ytrip.end());
  c.y_ac.makeCompressed();
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);
  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  for (int phase = 0; phase < 3; ++phase) {
    c.p_load_pu[n - 3 + phase] = 2e-4;
    c.q_load_pu[n - 3 + phase] = 5e-5;
  }
  c.v_min_pu = Eigen::VectorXd::Constant(n, 0.85);
  c.v_max_pu = Eigen::VectorXd::Constant(n, 1.10);
  c.voltage_start.resize(n);
  c.ac_phase_index.resize(n);
  const double angles[3] = {0.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0};
  for (int bus = 0; bus < bus_count; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      const int node = 3 * bus + phase;
      c.voltage_start[node] = std::polar(0.98, angles[phase]);
      c.ac_phase_index[static_cast<std::size_t>(node)] = phase;
    }
    c.three_phase_bus_nodes.push_back(
        {3 * bus, 3 * bus + 1, 3 * bus + 2});
  }
  c.reference_nodes = {0, 1, 2};
  c.reference_voltage.resize(3);
  for (int phase = 0; phase < 3; ++phase) {
    c.reference_voltage[phase] = std::polar(1.0, angles[phase]);
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
  c.vuf_max = 0.05;

  c.g_dc.resize(2, 2);
  std::vector<Eigen::Triplet<double>> gtrip{
      {0, 0, 50.0}, {0, 1, -50.0}, {1, 0, -50.0}, {1, 1, 50.0}};
  c.g_dc.setFromTriplets(gtrip.begin(), gtrip.end());
  c.p_dc_load_pu = Eigen::VectorXd::Zero(2);
  c.p_dc_load_pu[1] = 1e-3;
  c.v_dc_start = Eigen::VectorXd::Ones(2);
  c.v_dc_min_pu = Eigen::VectorXd::Constant(2, 0.90);
  c.v_dc_max_pu = Eigen::VectorXd::Constant(2, 1.10);
  c.dc_reference_terminals = {0};
  c.dc_reference_voltage_pu = Eigen::VectorXd::Ones(1);
  PhaseVSC converter;
  converter.phase_nodes = {n - 3, n - 2, n - 1};
  converter.dc_terminal = 0;
  converter.efficiency = 0.98;
  converter.s_max_pu = 0.25;
  c.converters.push_back(converter);
  return c;
}

}  // namespace

TEST_CASE("Monolithic phase hybrid OPF Full and GR recover the same solution",
          "[opf][three_phase][hybrid][kron]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
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
  CHECK(full.generator_active_power_pu.size() == c.generators.size());
  CHECK(full.generator_reactive_power_pu.size() == c.generators.size());
  CHECK(reduced.generator_active_power_pu.size() == c.generators.size());
  CHECK(reduced.generator_reactive_power_pu.size() == c.generators.size());
  CHECK(reduced.eliminated_phase_nodes == 3);
  CHECK(std::abs(full.objective - reduced.objective) <=
        1e-6 * std::max(1.0, std::abs(full.objective)));
  REQUIRE(full.full_voltage.size() == reduced.full_voltage.size());
  CHECK((full.full_voltage - reduced.full_voltage).cwiseAbs().maxCoeff() < 1e-6);
  CHECK(full.max_voltage_violation < 1e-8);
  CHECK(reduced.max_voltage_violation < 1e-8);
  CHECK(full.max_vuf <= c.vuf_max + 1e-7);
  CHECK(reduced.max_vuf <= c.vuf_max + 1e-7);
  CHECK(std::isfinite(reduced.max_converter_current_vuf));
  CHECK(reduced.max_converter_current_vuf > 0.0);
  CHECK(std::abs(full.max_converter_current_vuf -
                 reduced.max_converter_current_vuf) < 1e-6);
}

TEST_CASE("Graph-reduced OPF sequence retains every time-varying load node",
          "[opf][three_phase][hybrid][sequence][kron]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  auto first = make_small_hybrid_case();
  auto second = first;
  second.p_load_pu[3] = 0.035;
  second.q_load_pu[3] = 0.010;

  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::GraphReduced;
  options.backend = SolverBackend::Ipopt;
  options.max_iterations = 300;
  options.tolerance = 1e-7;
  options.reduction_options.max_front = 8;
  options.reduction_options.max_nnz_ratio = 10.0;

  const auto sequence =
      solve_three_phase_hybrid_opf_sequence({first, second}, options);
  const auto standalone = solve_three_phase_hybrid_opf(second, options);
  REQUIRE(sequence.size() == 2);
  REQUIRE(sequence[0].converged);
  REQUIRE(sequence[1].converged);
  REQUIRE(standalone.converged);
  CHECK(std::abs(sequence[1].objective - standalone.objective) <=
        1e-7 * std::max(1.0, std::abs(standalone.objective)));
  CHECK((sequence[1].full_voltage - standalone.full_voltage)
            .cwiseAbs()
            .maxCoeff() < 1e-6);
}

TEST_CASE("Sequence-aware VSC ports recover GFL and GFM dynamic equilibria",
          "[opf][three_phase][hybrid][converter][dynamics]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::GraphReduced;
  options.backend = SolverBackend::Ipopt;
  options.max_iterations = 500;
  options.tolerance = 1e-7;
  options.verify_derivatives = true;
  options.reduction_options.max_front = 8;
  options.reduction_options.max_nnz_ratio = 10.0;

  SECTION("grid-following PLL positive-sequence current equilibrium") {
    auto c = make_small_hybrid_case();
    c.converters.front().control_mode = PhaseVSCControlMode::GridFollowingPLL;
    const auto result = solve_three_phase_hybrid_opf(c, options);
    INFO(result.status);
    REQUIRE(result.converged);
    REQUIRE(result.converter_dynamic_equilibria.size() == 1);
    CHECK(result.max_converter_current_vuf < 1e-7);
    CHECK(result.max_converter_current_loading <= 1.0 + 1e-7);
    CHECK(result.max_dynamic_equilibrium_residual < 1e-7);
    CHECK(result.max_equality_jacobian_error < 1e-5);
    CHECK(result.max_lagrangian_hessian_error < 1e-5);

    NetworkState network;
    network.resize(3, 0);
    for (int phase = 0; phase < 3; ++phase) {
      network.Vac_abc[phase] = result.full_voltage[6 + phase];
    }
    hacdcpf::PowerFlowResult pf;
    const Complex a = std::polar(1.0, 2.0 * M_PI / 3.0);
    const Complex v1 = (network.Vac_abc[0] + a * network.Vac_abc[1] +
                        a * a * network.Vac_abc[2]) / 3.0;
    pf.vm = {std::abs(v1)};
    pf.va = {std::arg(v1)};
    GridFollowingInverterParams params;
    params.bus_pos = 0;
    params.base_mva = 1.0;
    params.p_ref_mw = result.converter_dynamic_equilibria.front()
                          .active_power_reference_pu;
    params.q_ref_mvar = result.converter_dynamic_equilibria.front()
                           .reactive_power_reference_pu;
    params.current_limit_pu = c.converters.front().phase_current_max_pu;
    params.pll_kp = c.converters.front().pll_kp;
    params.pll_ki = c.converters.front().pll_ki;
    GridFollowingInverter dynamic(params);
    int offset = 0;
    dynamic.assignStateIndices(offset);
    DynamicState state;
    state.resize(static_cast<std::size_t>(offset));
    dynamic.initializeFromPowerFlow(pf, state, network);
    dynamic.trimToNetworkEquilibrium(state, network);
    Eigen::VectorXd derivative = Eigen::VectorXd::Zero(offset);
    dynamic.computeDerivatives(0.0, state, network, derivative);
    CHECK(derivative.cwiseAbs().maxCoeff() < 1e-7);
  }

  SECTION("grid-forming droop Norton equilibrium") {
    auto c = make_small_hybrid_case();
    c.converters.front().control_mode = PhaseVSCControlMode::GridFormingDroop;
    c.converters.front().virtual_r_pu = 0.01;
    c.converters.front().virtual_x_pu = 0.10;
    const auto result = solve_three_phase_hybrid_opf(c, options);
    INFO(result.status);
    REQUIRE(result.converged);
    REQUIRE(result.converter_dynamic_equilibria.size() == 1);
    CHECK(result.max_converter_current_loading <= 1.0 + 1e-7);
    CHECK(result.max_dynamic_equilibrium_residual < 1e-7);
    CHECK(result.max_equality_jacobian_error < 1e-5);
    CHECK(result.max_lagrangian_hessian_error < 1e-5);
    CHECK(std::abs(result.converter_dynamic_equilibria.front()
                       .internal_voltage_positive) > 0.5);

    NetworkState network;
    network.resize(3, 0);
    for (int phase = 0; phase < 3; ++phase) {
      network.Vac_abc[phase] = result.full_voltage[6 + phase];
    }
    hacdcpf::PowerFlowResult pf;
    const Complex a = std::polar(1.0, 2.0 * M_PI / 3.0);
    const Complex v1 = (network.Vac_abc[0] + a * network.Vac_abc[1] +
                        a * a * network.Vac_abc[2]) / 3.0;
    pf.vm = {std::abs(v1)};
    pf.va = {std::arg(v1)};
    const auto& equilibrium = result.converter_dynamic_equilibria.front();
    GridFormingInverterParams params;
    params.bus_pos = 0;
    params.base_mva = 1.0;
    params.p_ref_mw = equilibrium.active_power_reference_pu;
    params.q_ref_mvar = equilibrium.reactive_power_reference_pu;
    params.v_ref_pu = equilibrium.voltage_reference_pu;
    params.virtual_r_pu = c.converters.front().virtual_r_pu;
    params.virtual_x_pu = c.converters.front().virtual_x_pu;
    params.current_limit_pu = c.converters.front().phase_current_max_pu;
    params.p_droop_pu = c.converters.front().p_droop_pu;
    params.q_droop_pu = c.converters.front().q_droop_pu;
    params.voltage_ki = c.converters.front().voltage_integral_gain;
    GridFormingInverter dynamic(params);
    int offset = 0;
    dynamic.assignStateIndices(offset);
    DynamicState state;
    state.resize(static_cast<std::size_t>(offset));
    dynamic.initializeFromPowerFlow(pf, state, network);
    dynamic.trimToNetworkEquilibrium(state, network);
    Eigen::VectorXd derivative = Eigen::VectorXd::Zero(offset);
    dynamic.computeDerivatives(0.0, state, network, derivative);
    CHECK(derivative.cwiseAbs().maxCoeff() < 1e-7);
  }
}

TEST_CASE("Independent coupled PF reproduces the OPF operating point",
          "[opf][power_flow][three_phase][hybrid][crosscheck]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::GraphReduced;
  options.backend = SolverBackend::Ipopt;
  options.max_iterations = 500;
  options.tolerance = 1e-8;
  options.reduction_options.max_front = 8;
  options.reduction_options.max_nnz_ratio = 10.0;

  for (const auto mode : {PhaseVSCControlMode::GridFollowingPLL,
                          PhaseVSCControlMode::GridFormingDroop}) {
    auto c = make_small_hybrid_case();
    c.converters.front().control_mode = mode;
    c.converters.front().virtual_r_pu = 0.01;
    c.converters.front().virtual_x_pu = 0.10;
    const auto opf = solve_three_phase_hybrid_opf(c, options);
    INFO(opf.status);
    REQUIRE(opf.converged);
    const auto pf_case = make_three_phase_hybrid_pf_case(c, opf);
    hacdcpf::powerflow::ThreePhaseHybridPFOptions pf_options;
    pf_options.max_iterations = 80;
    pf_options.tolerance = 1e-9;
    const auto pf =
        hacdcpf::powerflow::solve_three_phase_hybrid_pf(pf_case, pf_options);
    INFO(pf.status);
    REQUIRE(pf.converged);
    CHECK(pf.residual < 1e-8);
    REQUIRE(pf.voltage.size() == opf.full_voltage.size());
    REQUIRE(pf.dc_voltage.size() == opf.dc_voltage.size());
    CHECK((pf.voltage - opf.full_voltage).cwiseAbs().maxCoeff() < 2e-6);
    CHECK((pf.dc_voltage - opf.dc_voltage).cwiseAbs().maxCoeff() < 2e-6);
  }
}

TEST_CASE("Native Full certifies a graph-reduced primal-dual transport",
          "[opf][three_phase][hybrid][kron][native][warm_start]") {
  const auto c = make_small_hybrid_case();
  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::GraphReduced;
  options.backend = SolverBackend::NativeIPM;
  options.warm_start_with_ipopt = true;
  options.max_iterations = 500;
  options.tolerance = 1e-6;
  options.reduction_options.max_front = 8;
  options.reduction_options.max_nnz_ratio = 10.0;

  const auto reduced = solve_three_phase_hybrid_opf(c, options);
  INFO("reduced status=" << reduced.status
       << " p=" << reduced.primal_residual
       << " d=" << reduced.dual_residual
       << " c=" << reduced.complementarity);
  REQUIRE(reduced.converged);

  const int full_nv = static_cast<int>(c.y_ac.rows());
  const int reduced_nv = static_cast<int>(reduced.reduction.retained.size());
  const int ng = static_cast<int>(c.generators.size());
  const int ndc = static_cast<int>(c.g_dc.rows());
  int ncp = 0;
  int ngfm = 0;
  for (const auto& converter : c.converters) {
    ncp += static_cast<int>(converter.phase_nodes.size());
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      ++ngfm;
    }
  }
  const int nc = static_cast<int>(c.converters.size());
  const int device_variables = 2 * ng + ndc + 2 * ncp + 2 * ngfm + nc;

  REQUIRE(reduced.primal.size() == 2 * reduced_nv + device_variables);
  REQUIRE(reduced.full_voltage.size() == full_nv);
  options.variant = ModelVariant::Full;
  options.primal_start = Eigen::VectorXd::Zero(
      2 * full_nv + device_variables);
  for (int node = 0; node < full_nv; ++node) {
    options.primal_start[node] = std::real(reduced.full_voltage[node]);
    options.primal_start[full_nv + node] =
        std::imag(reduced.full_voltage[node]);
  }
  options.primal_start.tail(device_variables) =
      reduced.primal.tail(device_variables);

  const int full_equalities =
      reduced.equalities + 2 * (full_nv - reduced_nv);
  options.equality_dual_start = Eigen::VectorXd::Constant(
      full_equalities, std::numeric_limits<double>::quiet_NaN());
  for (int pos = 0; pos < reduced_nv; ++pos) {
    const int node =
        reduced.reduction.retained[static_cast<std::size_t>(pos)];
    options.equality_dual_start[node] = reduced.equality_dual[pos];
    options.equality_dual_start[full_nv + node] =
        reduced.equality_dual[reduced_nv + pos];
  }
  const int tail = reduced.equalities - 2 * reduced_nv;
  options.equality_dual_start.tail(tail) =
      reduced.equality_dual.tail(tail);
  options.nonlinear_inequality_dual_start =
      reduced.inequality_dual.head(reduced.inequalities);
  options.nonlinear_slack_start =
      reduced.inequality_slack.head(reduced.inequalities);

  ThreePhaseHybridOPFCase full_case = c;
  full_case.voltage_start = reduced.full_voltage;
  const auto full = solve_three_phase_hybrid_opf(full_case, options);
  INFO("full status=" << full.status
       << " initial=" << full.initial_primal_residual << "/"
       << full.initial_dual_residual
       << " final=" << full.primal_residual << "/"
       << full.dual_residual << "/" << full.complementarity);
  REQUIRE(full.converged);
  CHECK(full.initial_primal_residual <= 1e-6);
  CHECK(full.initial_dual_residual <= 1e-6);
  CHECK(full.phase_one_constraint_violation <= options.tolerance);
  CHECK(full.phase_one_primal_feasible);
  CHECK(full.phase_one_dual_initialized);
  CHECK_FALSE(full.phase_one_budget_exhausted);
  CHECK(full.phase_one_iterations <= options.phase_one_max_iterations);
  CHECK(full.phase_one_factorizations <=
        options.phase_one_max_factorizations);
  CHECK(full.phase_one_backtracks <=
        options.phase_one_max_iterations *
            options.phase_one_max_backtracks);
  CHECK(full.phase_one_runtime_ms >= 0.0);
  CHECK(full.phase_one_termination == "certified");
  CHECK(full.phase_two_start_requested);
  CHECK(full.phase_two_start_accepted);
  CHECK_FALSE(full.phase_two_linear_solver_backend.empty());
  CHECK(full.primal_residual <= 1e-6);
  CHECK(full.dual_residual <= 1e-6);
  CHECK(full.complementarity <= 1e-6);
  CHECK(std::abs(full.objective - reduced.objective) <=
        1e-6 * std::max(1.0, std::abs(full.objective)));
}

TEST_CASE("Native Phase I obeys a zero-factorization hard budget",
          "[opf][three_phase][hybrid][native][phase_one][budget]") {
  auto c = make_small_hybrid_case();
  c.voltage_start.array() *= 0.92;

  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::Full;
  options.backend = SolverBackend::NativeIPM;
  options.max_iterations = 1;
  options.tolerance = 1e-7;
  options.phase_one_max_iterations = 12;
  options.phase_one_max_factorizations = 0;
  options.phase_one_max_backtracks = 4;
  options.phase_one_time_limit_ms = 1000.0;

  const auto result = solve_three_phase_hybrid_opf(c, options);
  CHECK(result.phase_one_factorizations == 0);
  CHECK(result.phase_one_iterations == 0);
  CHECK(result.phase_one_budget_exhausted);
  CHECK(result.phase_one_termination == "factorization-budget");
  CHECK_FALSE(result.phase_one_dual_initialized);
  CHECK(result.phase_two_start_requested);
  CHECK_FALSE(result.phase_two_start_accepted);
}

TEST_CASE("Native Phase I permits a full step with no backtracking budget",
          "[opf][three_phase][hybrid][native][phase_one][budget]") {
  auto c = make_small_hybrid_case();
  c.voltage_start.array() *= 0.99;

  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::Full;
  options.backend = SolverBackend::NativeIPM;
  options.max_iterations = 0;
  options.tolerance = 1e-7;
  options.phase_one_max_iterations = 1;
  options.phase_one_max_factorizations = 3;
  options.phase_one_max_backtracks = 0;
  options.phase_one_time_limit_ms = 1000.0;

  const auto result = solve_three_phase_hybrid_opf(c, options);
  CHECK(result.phase_one_iterations == 1);
  CHECK(result.phase_one_backtracks == 0);
  CHECK(result.phase_one_factorizations <= 3);
  CHECK(result.phase_one_constraint_violation <
        result.phase_one_initial_violation);
}

TEST_CASE("Large Native Phase I remains sparse and budget bounded",
          "[opf][three_phase][hybrid][native][phase_one][large][sparse]") {
  const auto c = make_large_sparse_hybrid_case(360);
  const int phase_nodes = static_cast<int>(c.y_ac.rows());
  const int nvar = 2 * phase_nodes + 2 * static_cast<int>(c.generators.size()) +
                   static_cast<int>(c.g_dc.rows()) + 2 * 3 + 1;
  REQUIRE(nvar > 2000);

  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::Full;
  options.backend = SolverBackend::NativeIPM;
  options.max_iterations = 0;
  options.tolerance = 1e-6;
  options.phase_one_max_iterations = 2;
  options.phase_one_max_factorizations = 5;
  options.phase_one_max_backtracks = 4;
  options.phase_one_time_limit_ms = 5000.0;

  const auto result = solve_three_phase_hybrid_opf(c, options);
  INFO("termination=" << result.phase_one_termination
       << " solver=" << result.phase_one_linear_solver
       << " initial=" << result.phase_one_initial_violation
       << " final=" << result.phase_one_constraint_violation
       << " dual_full=" << result.initial_dual_residual
       << " dual_fit=" << result.phase_one_dual_fit_residual
       << " iterations=" << result.phase_one_iterations
       << " factorizations=" << result.phase_one_factorizations
       << " runtime_ms=" << result.phase_one_runtime_ms);
  CHECK(result.variables == nvar);
  CHECK(result.phase_one_iterations <= options.phase_one_max_iterations);
  CHECK(result.phase_one_factorizations <=
        options.phase_one_max_factorizations);
  CHECK((result.phase_one_linear_solver == "sparse-basis-lu" ||
         result.phase_one_linear_solver == "sparse-qr"));
  CHECK(std::isfinite(result.phase_one_constraint_violation));
  CHECK(result.phase_one_constraint_violation <=
        result.phase_one_initial_violation);
  CHECK(result.phase_one_primal_feasible);
  CHECK(result.phase_one_dual_initialized);
  CHECK(result.phase_one_dual_fit_residual <= 1e-6);
}

TEST_CASE("Exact constraint oracle preserves the graph-reduced OPF solution",
          "[opf][three_phase][hybrid][kron][constraint_oracle]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  const auto c = make_small_hybrid_case();
  ThreePhaseHybridOPFOptions all_rows_options;
  all_rows_options.variant = ModelVariant::GraphReduced;
  all_rows_options.backend = SolverBackend::Ipopt;
  all_rows_options.max_iterations = 300;
  all_rows_options.tolerance = 1e-7;
  all_rows_options.reduction_options.max_front = 8;
  all_rows_options.reduction_options.max_nnz_ratio = 10.0;

  ThreePhaseHybridOPFOptions oracle_options = all_rows_options;
  oracle_options.use_constraint_oracle = true;
  oracle_options.oracle_initial_margin = 1e-4;
  oracle_options.oracle_activation_margin = 1e-6;

  const auto all_rows = solve_three_phase_hybrid_opf(c, all_rows_options);
  const auto oracle = solve_three_phase_hybrid_opf(c, oracle_options);
  INFO("all-row status=" << all_rows.status
       << " residual=" << all_rows.primal_residual);
  INFO("oracle status=" << oracle.status
       << " residual=" << oracle.primal_residual
       << " rows=" << oracle.enforced_inequalities << '/'
       << oracle.inequalities
       << " max omitted=" << oracle.max_omitted_inequality);
  REQUIRE(all_rows.converged);
  REQUIRE(oracle.converged);
  CHECK(oracle.constraint_oracle_rounds >= 1);
  CHECK(oracle.enforced_inequalities < oracle.inequalities);
  CHECK(oracle.max_omitted_inequality <
        -oracle_options.oracle_activation_margin);
  CHECK(std::abs(all_rows.objective - oracle.objective) <=
        1e-7 * std::max(1.0, std::abs(all_rows.objective)));
  REQUIRE(all_rows.full_voltage.size() == oracle.full_voltage.size());
  CHECK((all_rows.full_voltage - oracle.full_voltage)
            .cwiseAbs().maxCoeff() < 1e-6);
  CHECK(oracle.primal_residual <= 1e-6);
  CHECK(oracle.dual_residual <= 1e-6);
  CHECK(oracle.complementarity <= 1e-6);

  ThreePhaseHybridOPFOptions seeded_options = oracle_options;
  seeded_options.oracle_seed_rows = oracle.enforced_inequality_rows;
  seeded_options.primal_start = oracle.primal;
  seeded_options.equality_dual_start = oracle.equality_dual;
  seeded_options.nonlinear_inequality_dual_start =
      oracle.inequality_dual.head(oracle.inequalities);
  seeded_options.nonlinear_slack_start =
      oracle.inequality_slack.head(oracle.inequalities);
  const auto seeded = solve_three_phase_hybrid_opf(c, seeded_options);
  INFO("seeded status=" << seeded.status
       << " residual=" << seeded.primal_residual
       << " rows=" << seeded.enforced_inequalities << '/'
       << seeded.inequalities);
  REQUIRE(seeded.converged);
  CHECK(seeded.constraint_oracle_added_rows == 0);
  CHECK(seeded.enforced_inequality_rows == oracle.enforced_inequality_rows);
  CHECK(std::abs(seeded.objective - oracle.objective) <=
        1e-8 * std::max(1.0, std::abs(oracle.objective)));
}

TEST_CASE("Lifted phase-hybrid relaxation gives a certified OPF lower bound",
          "[opf][three_phase][hybrid][relaxation]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  const auto c = make_small_hybrid_case();

  ThreePhaseHybridOPFOptions nonlinear_options;
  nonlinear_options.variant = ModelVariant::Full;
  nonlinear_options.backend = SolverBackend::Ipopt;
  nonlinear_options.max_iterations = 400;
  nonlinear_options.tolerance = 1e-8;
  const auto nonlinear = solve_three_phase_hybrid_opf(c, nonlinear_options);
  INFO("nonlinear status=" << nonlinear.status);
  REQUIRE(nonlinear.converged);

  ThreePhaseHybridRelaxationOptions coarse_options;
  coarse_options.max_outer_approximation_rounds = 0;
  const auto coarse =
      solve_three_phase_hybrid_opf_relaxation(c, coarse_options);
  INFO("coarse status=" << coarse.status);
  REQUIRE(coarse.solved);
  REQUIRE(coarse.outer_relaxation_valid);
  REQUIRE(coarse.dual_certificate_available);
  CHECK(coarse.objective_lower_bound <= nonlinear.objective + 1e-7);

  ThreePhaseHybridRelaxationOptions refined_options = coarse_options;
  refined_options.max_outer_approximation_rounds = 10;
  refined_options.cone_tolerance = 1e-7;
  const auto refined =
      solve_three_phase_hybrid_opf_relaxation(c, refined_options);
  INFO("refined status=" << refined.status
       << " lower=" << refined.objective_lower_bound
       << " nonlinear=" << nonlinear.objective
       << " cone=" << refined.max_soc_violation);
  REQUIRE(refined.solved);
  REQUIRE(refined.outer_relaxation_valid);
  REQUIRE(refined.dual_certificate_available);
  CHECK(refined.objective_lower_bound + 1e-8 >=
        coarse.objective_lower_bound);
  CHECK(refined.objective_lower_bound <= nonlinear.objective + 1e-7);
  CHECK(refined.lp_primal_objective + 1e-8 >=
        refined.objective_lower_bound);
  REQUIRE_FALSE(refined.round_lower_bounds.empty());
  for (std::size_t round = 1; round < refined.round_lower_bounds.size();
       ++round) {
    CHECK(refined.round_lower_bounds[round] + 1e-9 >=
          refined.round_lower_bounds[round - 1]);
  }
  CHECK(refined.max_soc_violation <= coarse.max_soc_violation + 1e-9);
}

TEST_CASE("Phase-hybrid relaxation rejects unsupported fixed-current loads",
          "[opf][three_phase][hybrid][relaxation][limitations]") {
  auto c = make_small_hybrid_case();
  c.i_ac_fixed[4] = Complex{0.01, -0.02};
  const auto result = solve_three_phase_hybrid_opf_relaxation(c);
  CHECK_FALSE(result.solved);
  CHECK_FALSE(result.outer_relaxation_valid);
  CHECK(result.status.find("fixed-current") != std::string::npos);
  CHECK_FALSE(result.model_limitations.empty());
  CHECK(result.runtime_ms >= 0.0);
}

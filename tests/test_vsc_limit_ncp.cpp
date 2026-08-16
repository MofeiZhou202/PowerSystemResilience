#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/LU>
#include <nlohmann/json.hpp>

#include "hacdcpf/power_flow/vsc_limit_ncp.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#ifdef HACDCPF_HAVE_OPENDSS
#include "hacdcpf/io/opendss_bridge.hpp"
#endif
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/model/gfm_norton_contract.hpp"
#include "hacdcpf/model/defaults.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/power_flow/ncp_functions.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/solvers/vsc_local_schur.hpp"

namespace {

using hacdcpf::ConverterMode;
using hacdcpf::LossModelType;
using hacdcpf::VSCCurrentLimitPriority;
using hacdcpf::VSCConverter;
using hacdcpf::powerflow::VSCLimitEvaluation;
using hacdcpf::powerflow::evaluate_vsc_limit_ncp;
using hacdcpf::powerflow::initialize_vsc_limit_state;

VSCConverter make_converter(VSCCurrentLimitPriority priority) {
  VSCConverter converter;
  converter.index = 17;
  converter.bus_ac = 1;
  converter.bus_dc = 1;
  converter.control_mode = ConverterMode::PQ_MODE;
  converter.p_set_mw = 90.0;
  converter.q_set_mvar = 60.0;
  converter.eta = 0.98;
  converter.i_ac_max_pu = 0.8;
  converter.enable_limit_ncp = true;
  converter.current_limit_priority = priority;
  return converter;
}

Eigen::Matrix<double, hacdcpf::powerflow::kVSCLimitStateSize, 1> solve_local(
                                        const VSCConverter& converter,
                                        double vm,
                                        double vdc) {
  auto state = initialize_vsc_limit_state(
      converter, vm, 0.0, vdc, 100.0, LossModelType::Linear);
  for (int iteration = 0; iteration < 20; ++iteration) {
    const VSCLimitEvaluation evaluation = evaluate_vsc_limit_ncp(
        converter, vm, 0.0, vdc, 100.0, LossModelType::Linear, state);
    if (evaluation.equation.cwiseAbs().maxCoeff() < 1e-12) return state;
    state -= evaluation.jacobian.fullPivLu().solve(evaluation.equation);
  }
  return state;
}

std::array<double, 2> projection_oracle(const VSCConverter& converter,
                                        double vm) {
  const double p_ref = converter.p_set_mw / 100.0;
  const double q_ref = converter.q_set_mvar / 100.0;
  const double radius = vm * converter.i_ac_max_pu;
  if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
    const double norm = std::hypot(p_ref, q_ref);
    const double scale = norm > radius ? radius / norm : 1.0;
    return {scale * p_ref, scale * q_ref};
  }
  if (converter.current_limit_priority == VSCCurrentLimitPriority::ActivePower) {
    const double p = std::clamp(p_ref, -radius, radius);
    const double q_cap = std::sqrt(std::max(0.0, radius * radius - p * p));
    return {p, std::clamp(q_ref, -q_cap, q_cap)};
  }
  const double q = std::clamp(q_ref, -radius, radius);
  const double p_cap = std::sqrt(std::max(0.0, radius * radius - q * q));
  return {std::clamp(p_ref, -p_cap, p_cap), q};
}

hacdcpf::powerflow::JacobianPattern make_schur_oracle_pattern(
    hacdcpf::powerflow::JacobianContext& context,
    Eigen::SparseMatrix<double>& matrix) {
  using hacdcpf::powerflow::JacobianPattern;
  using hacdcpf::powerflow::kVSCLimitStateSize;
  context.n = 1;
  context.ndc = 1;
  context.np = 1;
  context.nq = 1;
  context.ndc_eq = 1;
  context.network_nvar = 3;
  context.nvar = 3 + kVSCLimitStateSize;
  context.p_row = {0};
  context.q_row = {1};
  context.dc_row = {2};
  context.va_col = {0};
  context.vm_col = {1};
  context.vdc_col = {2};
  context.vsc_limit_blocks = {{0, 0, 0, 3}};

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      triplets.emplace_back(row, col, row == col ? 5.0 + row : 0.2);
    }
  }
  triplets.emplace_back(0, 3, -1.0);
  triplets.emplace_back(1, 4, -1.0);
  triplets.emplace_back(2, 5, -1.0);
  for (int row = 0; row < kVSCLimitStateSize; ++row) {
    for (int col = 0; col < kVSCLimitStateSize; ++col) {
      const double value = row == col ? 2.0 + 0.1 * row
                                      : 0.02 * (row + col + 1);
      triplets.emplace_back(3 + row, 3 + col, value);
    }
    for (int col = 0; col < 3; ++col) {
      triplets.emplace_back(3 + row, col,
                            0.03 * (row + 1) * (col + 1));
    }
  }
  matrix.resize(context.nvar, context.nvar);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();

  auto lookup = [&](int row, int col) {
    const int* outer = matrix.outerIndexPtr();
    const int* inner = matrix.innerIndexPtr();
    for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
      if (inner[nz] == row) return nz;
    }
    return -1;
  };

  JacobianPattern pattern;
  pattern.matrix = matrix;
  JacobianPattern::VSCLimitEntry entry;
  entry.block_index = 0;
  entry.p_network_nz = lookup(0, 3);
  entry.q_network_nz = lookup(1, 4);
  entry.dc_network_nz = lookup(2, 5);
  for (int row = 0; row < kVSCLimitStateSize; ++row) {
    for (int col = 0; col < kVSCLimitStateSize; ++col) {
      entry.local_nz[static_cast<size_t>(row * kVSCLimitStateSize + col)] =
          lookup(3 + row, 3 + col);
    }
    entry.va_nz[static_cast<size_t>(row)] = lookup(3 + row, 0);
    entry.vm_nz[static_cast<size_t>(row)] = lookup(3 + row, 1);
    entry.vdc_nz[static_cast<size_t>(row)] = lookup(3 + row, 2);
  }
  pattern.vsc_limit_entries = {entry};
  return pattern;
}

TEST_CASE("Local VSC Schur step is algebraically identical to full LU",
          "[power_flow][converter][ncp][schur][oracle]") {
  hacdcpf::powerflow::JacobianContext context;
  Eigen::SparseMatrix<double> matrix;
  const auto pattern = make_schur_oracle_pattern(context, matrix);
  hacdcpf::powerflow::VSCLocalSchurSolver solver;
  REQUIRE(solver.initialize(pattern, context));

  Eigen::VectorXd rhs(context.nvar);
  rhs << 0.5, -0.2, 0.1, 0.3, -0.4, 0.6, -0.1, 0.2, 0.7;
  Eigen::VectorXd schur_step;
  hacdcpf::powerflow::VSCLocalSchurReport report;
  REQUIRE(solver.solve(matrix, rhs, false, 1e-12, 1e-12,
                       schur_step, report));
  const Eigen::VectorXd full_step =
      Eigen::MatrixXd(matrix).fullPivLu().solve(rhs);
  const double relative_error =
      (schur_step - full_step).norm() / std::max(1.0, full_step.norm());
  const double residual = (matrix * schur_step - rhs).lpNorm<Eigen::Infinity>();
  INFO("relative_error=" << relative_error
                          << " backward_error=" << report.full_backward_error);
  CHECK(relative_error <= 1e-10);
  CHECK(report.full_backward_error <= 1e-12);
  CHECK(residual <= 1e-12);
  CHECK(report.full_dimension - report.reduced_dimension == 6);
  CHECK(report.full_structural_nnz - report.reduced_structural_nnz >= 48);
  CHECK(report.minimum_local_rcond > 1e-12);
}

TEST_CASE("Local VSC Schur rejects a singular selected local block",
          "[power_flow][converter][ncp][schur][bd_regular]") {
  hacdcpf::powerflow::JacobianContext context;
  Eigen::SparseMatrix<double> matrix;
  const auto pattern = make_schur_oracle_pattern(context, matrix);
  for (int col = 3; col < 9; ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it; ++it) {
      if (it.row() >= 3) it.valueRef() = 0.0;
    }
  }
  hacdcpf::powerflow::VSCLocalSchurSolver solver;
  REQUIRE(solver.initialize(pattern, context));
  Eigen::VectorXd step;
  hacdcpf::powerflow::VSCLocalSchurReport report;
  CHECK_FALSE(solver.solve(matrix, Eigen::VectorXd::Ones(9), false,
                           1e-12, 1e-12, step, report));
  CHECK(report.status == "local_block_not_regular");
  CHECK_FALSE(report.accepted);
}

TEST_CASE("Exact and smoothed Fischer-Burmeister primitives retain their contracts",
          "[power_flow][converter][ncp][smoothing][oracle]") {
  using hacdcpf::powerflow::fischer_burmeister;
  using hacdcpf::powerflow::fischer_burmeister_jacobian;
  using hacdcpf::powerflow::smooth_fb;
  using hacdcpf::powerflow::smooth_fb_derivatives;

  CHECK(fischer_burmeister(0.0, 0.0) == 0.0);
  const auto [origin_da, origin_db] =
      fischer_burmeister_jacobian(0.0, 0.0);
  CHECK(std::isfinite(origin_da));
  CHECK(std::isfinite(origin_db));
  CHECK(origin_da == Catch::Approx(origin_db).margin(1e-15));

  constexpr double a = 0.03;
  constexpr double b = 0.07;
  constexpr double mu = 1e-3;
  constexpr double step = 1e-7;
  const auto [smooth_da, smooth_db] = smooth_fb_derivatives(a, b, mu);
  const double fd_a =
      (smooth_fb(a + step, b, mu) - smooth_fb(a - step, b, mu)) /
      (2.0 * step);
  const double fd_b =
      (smooth_fb(a, b + step, mu) - smooth_fb(a, b - step, mu)) /
      (2.0 * step);
  CHECK(smooth_da == Catch::Approx(fd_a).margin(2e-9));
  CHECK(smooth_db == Catch::Approx(fd_b).margin(2e-9));
  CHECK(smooth_fb(a, b, 1e-12) ==
        Catch::Approx(fischer_burmeister(a, b)).margin(1e-12));
}

TEST_CASE("Numerical representation scales have one global source",
          "[numerics][defaults][machine_epsilon]") {
  CHECK(hacdcpf::NumericalConstants::kMachineEpsilon ==
        std::numeric_limits<double>::epsilon());
  const double squared =
      hacdcpf::NumericalConstants::kSqrtMachineEpsilon *
      hacdcpf::NumericalConstants::kSqrtMachineEpsilon;
  CHECK(std::abs(squared - hacdcpf::NumericalConstants::kMachineEpsilon) <=
        4.0 * hacdcpf::NumericalConstants::kMachineEpsilon *
            hacdcpf::NumericalConstants::kMachineEpsilon);
  CHECK(hacdcpf::Defaults::kVSCSchurLocalRcondTol ==
        hacdcpf::NumericalConstants::kSqrtMachineEpsilon);
}

TEST_CASE("VSC Schur policy rejects invalid hard-coded numerical values",
          "[numerics][defaults][schur][configuration]") {
  hacdcpf::powerflow::RobustNonlinearOptions options;
  options.vsc_schur_min_network_dimension = -7;
  options.vsc_schur_local_rcond_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  options.vsc_schur_backward_error_tolerance = 2.0;

  hacdcpf::powerflow::normalize_vsc_schur_policy(options);

  CHECK(options.vsc_schur_min_network_dimension == 0);
  CHECK(options.vsc_schur_local_rcond_tolerance ==
        hacdcpf::Defaults::kVSCSchurLocalRcondTol);
  CHECK(options.vsc_schur_backward_error_tolerance ==
        hacdcpf::Defaults::kVSCSchurBackwardErrorTol);
}

hacdcpf::HybridPowerSystem make_hybrid_case(
    VSCCurrentLimitPriority priority,
    bool enable_limit_ncp,
    double p_set_mw = 90.0,
    double q_set_mvar = 60.0) {
  using namespace hacdcpf;
  HybridPowerSystem system;
  system.base_mva = 100.0;
  system.ac.base_mva = 100.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  ACBus terminal;
  terminal.index = 2;
  terminal.bus_type = BusType::PQ;
  terminal.vm_pu = 1.0;
  system.ac.buses = {slack, terminal};

  ACBranch ac_line;
  ac_line.index = 11;
  ac_line.from_bus = 1;
  ac_line.to_bus = 2;
  ac_line.r_pu = 0.02;
  ac_line.x_pu = 0.15;
  system.ac.branches = {ac_line};

  Generator source;
  source.index = 21;
  source.bus = 1;
  source.is_slack = true;
  source.in_service = true;
  system.ac.generators = {source};

  DCBus dc_reference;
  dc_reference.index = 1;
  dc_reference.bus_type = DCBusType::DC_V;
  dc_reference.vm_pu = 1.0;
  DCBus dc_terminal;
  dc_terminal.index = 2;
  dc_terminal.bus_type = DCBusType::DC_P;
  dc_terminal.vm_pu = 1.0;
  system.dc.buses = {dc_reference, dc_terminal};

  DCBranch dc_line;
  dc_line.index = 31;
  dc_line.from_bus = 1;
  dc_line.to_bus = 2;
  dc_line.r_pu = 0.08;
  system.dc.branches = {dc_line};

  VSCConverter converter = make_converter(priority);
  converter.index = 77;
  converter.bus_ac = 2;
  converter.bus_dc = 2;
  converter.p_set_mw = p_set_mw;
  converter.q_set_mvar = q_set_mvar;
  converter.enable_limit_ncp = enable_limit_ncp;
  system.vsc_converters = {converter};
  return system;
}

hacdcpf::HybridPowerSystem make_two_converter_case() {
  auto system = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true);
  auto ac_terminal = system.ac.buses.back();
  ac_terminal.index = 3;
  system.ac.buses.push_back(ac_terminal);
  auto ac_line = system.ac.branches.back();
  ac_line.index = 12;
  ac_line.to_bus = 3;
  system.ac.branches.push_back(ac_line);

  auto dc_terminal = system.dc.buses.back();
  dc_terminal.index = 3;
  system.dc.buses.push_back(dc_terminal);
  auto dc_line = system.dc.branches.back();
  dc_line.index = 32;
  dc_line.to_bus = 3;
  system.dc.branches.push_back(dc_line);

  auto second = system.vsc_converters.front();
  second.index = 88;
  second.bus_ac = 3;
  second.bus_dc = 3;
  second.current_limit_priority = VSCCurrentLimitPriority::ActivePower;
  second.p_set_mw = 75.0;
  second.q_set_mvar = 70.0;
  system.vsc_converters.push_back(second);
  return system;
}

VSCConverter make_gfm_converter(VSCCurrentLimitPriority priority,
                                double current_limit = 0.8) {
  VSCConverter converter;
  converter.index = 91;
  converter.bus_ac = 2;
  converter.bus_dc = 2;
  converter.control_mode = ConverterMode::AC_GRID_FORMING;
  converter.enable_limit_ncp = true;
  converter.current_limit_priority = priority;
  converter.i_ac_max_pu = current_limit;
  converter.gfm_internal_voltage_set_pu = 1.08;
  converter.gfm_internal_angle_set_deg = 12.0;
  converter.gfm_virtual_r_pu = 0.04;
  converter.gfm_virtual_x_pu = 0.20;
  converter.eta = 0.98;
  return converter;
}

hacdcpf::HybridPowerSystem make_grid_connected_gfm_case(
    VSCCurrentLimitPriority priority, double current_limit = 0.8) {
  auto system = make_hybrid_case(priority, false, 0.0, 0.0);
  system.vsc_converters = {make_gfm_converter(priority, current_limit)};
  return system;
}

hacdcpf::HybridPowerSystem make_islanded_gfm_case(
    VSCCurrentLimitPriority priority = VSCCurrentLimitPriority::Magnitude,
    double current_limit = 1.2) {
  auto system = make_grid_connected_gfm_case(priority, current_limit);
  system.name = "islanded_gfm_norton";
  system.ac.buses.front().bus_type = hacdcpf::BusType::PQ;
  system.ac.buses.front().pd_mw = 28.0;
  system.ac.buses.front().qd_mvar = 9.0;
  system.ac.generators.clear();
  hacdcpf::Storage dc_source;
  dc_source.index = 501;
  dc_source.bus = 1;
  dc_source.p_rated_mw = 200.0;
  dc_source.pmin_mw = -200.0;
  dc_source.pmax_mw = 200.0;
  dc_source.e_rated_mwh = 1000.0;
  dc_source.soc_init = 0.5;
  dc_source.soc_min = 0.1;
  dc_source.soc_max = 0.9;
  dc_source.controllable = true;
  system.dc.storage.push_back(dc_source);
  return system;
}

hacdcpf::HybridPowerSystem make_two_islanded_gfm_case() {
  auto system = make_islanded_gfm_case();
  auto terminal = system.ac.buses.back();
  terminal.index = 3;
  terminal.vm_pu = 1.0;
  system.ac.buses.push_back(terminal);
  auto branch = system.ac.branches.back();
  branch.index = 12;
  branch.to_bus = 3;
  system.ac.branches.push_back(branch);
  auto dc_terminal = system.dc.buses.back();
  dc_terminal.index = 3;
  system.dc.buses.push_back(dc_terminal);
  auto dc_branch = system.dc.branches.back();
  dc_branch.index = 32;
  dc_branch.to_bus = 3;
  system.dc.branches.push_back(dc_branch);
  auto second = make_gfm_converter(VSCCurrentLimitPriority::ActivePower, 1.2);
  second.index = 92;
  second.bus_ac = 3;
  second.bus_dc = 3;
  second.gfm_internal_voltage_set_pu = 1.06;
  second.gfm_internal_angle_set_deg = 10.0;
  system.vsc_converters.push_back(second);
  return system;
}

hacdcpf::HybridPowerSystem make_two_gfm_case(double current_limit = 0.3) {
  auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude, current_limit);
  auto ac_terminal = system.ac.buses.back();
  ac_terminal.index = 3;
  system.ac.buses.push_back(ac_terminal);
  auto ac_line = system.ac.branches.back();
  ac_line.index = 12;
  ac_line.to_bus = 3;
  system.ac.branches.push_back(ac_line);
  auto dc_terminal = system.dc.buses.back();
  dc_terminal.index = 3;
  system.dc.buses.push_back(dc_terminal);
  auto dc_line = system.dc.branches.back();
  dc_line.index = 32;
  dc_line.to_bus = 3;
  system.dc.branches.push_back(dc_line);
  auto second = make_gfm_converter(
      VSCCurrentLimitPriority::ReactivePower, current_limit);
  second.index = 92;
  second.bus_ac = 3;
  second.bus_dc = 3;
  second.gfm_internal_angle_set_deg = -12.0;
  system.vsc_converters.push_back(second);
  return system;
}

#ifdef HACDCPF_HAVE_OPENDSS
struct OpenDSSGFMPoint {
  double voltage_magnitude_pu{0.0};
  double voltage_angle_rad{0.0};
  double terminal_kcl_residual_pu{0.0};
};

OpenDSSGFMPoint solve_opendss_gfm_circuit(
    const std::complex<double>& internal_voltage) {
  const auto directory = std::filesystem::temp_directory_path() /
      ("hacdcpf_gfm_opendss_" +
       std::to_string(static_cast<unsigned long long>(
           std::hash<std::string>{}(
               std::to_string(internal_voltage.real()) + ":" +
               std::to_string(internal_voltage.imag())))));
  std::filesystem::create_directories(directory);
  const auto master = directory / "Master.dss";
  const double magnitude = std::abs(internal_voltage);
  const double angle_deg =
      std::arg(internal_voltage) * 180.0 / std::numbers::pi;
  {
    std::ofstream output(master);
    REQUIRE(output.good());
    output << "Clear\n"
           << "New Circuit.GFMTest phases=3 bus1=grid_internal basekv=100 "
              "pu=1 angle=0 frequency=60 R1=1e-7 X1=1e-7 R0=1e-7 X0=1e-7\n"
           << "New Line.GridZ phases=3 bus1=grid_internal bus2=terminal "
              "r1=2 x1=15 r0=2 x0=15 c1=0 c0=0 length=1 units=km\n"
           << "New Vsource.GFM phases=3 bus1=gfm_internal basekv=100 pu="
           << magnitude << " angle=" << angle_deg
           << " frequency=60 R1=1e-7 X1=1e-7 R0=1e-7 X0=1e-7\n"
           << "New Line.GFMZ phases=3 bus1=gfm_internal bus2=terminal "
              "r1=4 x1=20 r0=4 x0=20 c1=0 c0=0 length=1 units=km\n"
           << "Set voltagebases=[100]\n"
           << "CalcVoltageBases\n"
           << "Set mode=snapshot controlmode=off tolerance=1e-12 "
              "maxiterations=100\n"
           << "Solve\n";
  }
  const auto snapshot = hacdcpf::io::solve_opendss_snapshot(master);
  std::filesystem::remove_all(directory);
  REQUIRE(snapshot.converged);
  const auto terminal = std::find_if(
      snapshot.node_voltages.begin(), snapshot.node_voltages.end(),
      [](const hacdcpf::io::OpenDSSNodeVoltage& voltage) {
        return voltage.bus_name == "terminal" && voltage.node == 1;
      });
  REQUIRE(terminal != snapshot.node_voltages.end());
  std::complex<double> terminal_power_kva{0.0, 0.0};
  for (const auto& element : snapshot.pd_element_results) {
    if (element.element_kind != hacdcpf::io::OpenDSSPDElementKind::Line ||
        (element.name != "gridz" && element.name != "gfmz")) {
      continue;
    }
    for (const auto& power : element.terminal_powers) {
      if (power.terminal == 2) {
        terminal_power_kva += std::complex<double>{
            power.power_kw_kvar.p_kw, power.power_kw_kvar.q_kvar};
      }
    }
  }
  return OpenDSSGFMPoint{
      terminal->vm_pu,
      terminal->va_deg * std::numbers::pi / 180.0,
      std::abs(terminal_power_kva) / 100000.0};
}
#endif

}  // namespace

TEST_CASE("GFM Norton limit block agrees with the frozen-terminal oracle",
          "[power_flow][converter][gfm][ncp][oracle]") {
  constexpr double vm = 0.96;
  constexpr double va = -0.07;
  const std::complex<double> voltage = std::polar(vm, va);
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    const VSCConverter converter = make_gfm_converter(priority);
    const auto state = initialize_vsc_limit_state(
        converter, vm, va, 1.0, 100.0, LossModelType::Linear);
    const auto evaluation = evaluate_vsc_limit_ncp(
        converter, vm, va, 1.0, 100.0, LossModelType::Linear, state);
    const std::complex<double> internal{state[3], state[4]};
    const std::complex<double> impedance{
        converter.gfm_virtual_r_pu, converter.gfm_virtual_x_pu};
    const std::complex<double> current = (internal - voltage) / impedance;
    const std::complex<double> power = voltage * std::conj(current);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    CHECK(state[0] == Catch::Approx(power.real()).margin(1e-12));
    CHECK(state[1] == Catch::Approx(power.imag()).margin(1e-12));
    CHECK(std::abs(current) <= converter.i_ac_max_pu + 1e-12);
    CHECK(evaluation.complementarity_residual <= 1e-12);
  }
}

TEST_CASE("Shared GFM Norton contract has one explicit and legacy precedence",
          "[power_flow][converter][gfm][contract]") {
  VSCConverter converter;
  converter.v_ac_set_pu = 1.01;
  converter.v_ac_angle_set_deg = -4.0;
  converter.r_conv_ac_pu = 0.015;
  converter.x_sc_pu = 0.11;

  auto resolved = hacdcpf::model::resolve_gfm_norton_parameters(converter);
  CHECK(resolved.internal_voltage_pu == Catch::Approx(1.01));
  CHECK(resolved.internal_angle_rad ==
        Catch::Approx(-4.0 * std::numbers::pi / 180.0));
  CHECK(resolved.virtual_r_pu == Catch::Approx(0.015));
  CHECK(resolved.virtual_x_pu == Catch::Approx(0.11));

  converter.gfm_internal_voltage_set_pu = 1.08;
  converter.gfm_internal_angle_set_deg = 12.0;
  converter.gfm_virtual_r_pu = 0.04;
  converter.gfm_virtual_x_pu = 0.20;
  resolved = hacdcpf::model::resolve_gfm_norton_parameters(converter);
  CHECK(resolved.internal_voltage_pu == Catch::Approx(1.08));
  CHECK(resolved.internal_angle_rad ==
        Catch::Approx(12.0 * std::numbers::pi / 180.0));
  CHECK(resolved.virtual_r_pu == Catch::Approx(0.04));
  CHECK(resolved.virtual_x_pu == Catch::Approx(0.20));
}

TEST_CASE("GFM Norton analytic generalized Jacobian matches finite differences",
          "[power_flow][converter][gfm][ncp][jacobian]") {
  constexpr double vm = 0.97;
  constexpr double va = -0.04;
  constexpr double vdc = 1.01;
  constexpr double step = 1e-7;
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    const VSCConverter converter = make_gfm_converter(priority);
    auto state = initialize_vsc_limit_state(
        converter, vm, va, vdc, 100.0, LossModelType::Linear);
    state[3] += 0.003;
    state[4] -= 0.002;
    state[5] += priority == VSCCurrentLimitPriority::Magnitude ? 0.02 : 0.0;
    const auto analytic = evaluate_vsc_limit_ncp(
        converter, vm, va, vdc, 100.0, LossModelType::Linear, state);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    for (int column = 0; column < hacdcpf::powerflow::kVSCLimitStateSize;
         ++column) {
      auto plus = state;
      auto minus = state;
      plus[column] += step;
      minus[column] -= step;
      const auto ep = evaluate_vsc_limit_ncp(
          converter, vm, va, vdc, 100.0, LossModelType::Linear, plus);
      const auto em = evaluate_vsc_limit_ncp(
          converter, vm, va, vdc, 100.0, LossModelType::Linear, minus);
      const auto finite_difference = (ep.equation - em.equation) / (2.0 * step);
      CHECK((finite_difference - analytic.jacobian.col(column))
                .cwiseAbs().maxCoeff() <= 5e-7);
    }
    const auto vm_plus = evaluate_vsc_limit_ncp(
        converter, vm + step, va, vdc, 100.0, LossModelType::Linear, state);
    const auto vm_minus = evaluate_vsc_limit_ncp(
        converter, vm - step, va, vdc, 100.0, LossModelType::Linear, state);
    CHECK(((vm_plus.equation - vm_minus.equation) / (2.0 * step) -
           analytic.derivative_vm).cwiseAbs().maxCoeff() <= 5e-7);
    const auto va_plus = evaluate_vsc_limit_ncp(
        converter, vm, va + step, vdc, 100.0, LossModelType::Linear, state);
    const auto va_minus = evaluate_vsc_limit_ncp(
        converter, vm, va - step, vdc, 100.0, LossModelType::Linear, state);
    CHECK(((va_plus.equation - va_minus.equation) / (2.0 * step) -
           analytic.derivative_va).cwiseAbs().maxCoeff() <= 5e-7);
  }
}

TEST_CASE("Grid-connected GFM terminal voltage is solved behind virtual impedance",
          "[power_flow][converter][gfm][ncp][integration]") {
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    auto data = hacdcpf::powerflow::make_solver_data(
        make_grid_connected_gfm_case(priority));
    CHECK(data.ac_buses[1].bus_type == hacdcpf::BusType::PQ);
    hacdcpf::PowerFlowOptions options;
    options.tol = 1e-10;
    options.max_iter = 100;
    hacdcpf::powerflow::NewtonSolver solver;
    const auto result = solver.solve(data, options);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    REQUIRE(result.converged);
    REQUIRE(result.vsc_limit_states.size() == 1);
    CHECK(result.vsc_limit_states.front().current_pu <= 0.8 + 1e-9);
    CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
    CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  }
}

TEST_CASE("Smooth continuation covers all current-limited GFM priorities",
          "[power_flow][converter][gfm][ncp][smoothing][integration]") {
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    auto data = hacdcpf::powerflow::make_solver_data(
        make_grid_connected_gfm_case(priority, 0.3));
    hacdcpf::PowerFlowOptions options;
    options.tol = 1e-9;
    options.max_iter = 40;
    options.robust_nonlinear.enable_smooth_ncp = true;
    options.robust_nonlinear.ncp_mu0 = 1e-2;
    options.robust_nonlinear.ncp_mu_min = 1e-12;
    hacdcpf::powerflow::NewtonSolver solver;
    const auto result = solver.solve(data, options);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    INFO("residual=" << result.residual
                      << " iterations=" << result.iterations
                      << " updates="
                      << result.profiling.smooth_ncp_continuation_updates);
    REQUIRE(result.converged);
    CHECK(result.iterations <= 40);
    CHECK(result.profiling.smooth_ncp_continuation_updates >= 1);
    CHECK(result.profiling.smooth_ncp_final_mu <= 1e-12);
    CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
    REQUIRE(result.vsc_limit_states.size() == 1);
    CHECK(result.vsc_limit_states.front().current_limit_active);
    CHECK(result.vsc_limit_states.front().current_pu <= 0.3 + 1e-9);
    CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
  }
}

TEST_CASE("GFM internal phasor anchors a slackless AC island",
          "[power_flow][converter][gfm][ncp][island]") {
  const auto system = make_islanded_gfm_case();
  const auto data = hacdcpf::powerflow::make_solver_data(system);
  REQUIRE(std::none_of(data.ac_buses.begin(), data.ac_buses.end(),
                       [](const hacdcpf::ACBus& bus) {
                         return bus.bus_type == hacdcpf::BusType::SLACK;
                       }));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  options.enable_converter_coordination_check = true;
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("residual=" << result.residual << " iterations=" << result.iterations);
  REQUIRE(result.converged);
  CHECK(result.iterations <= 20);
  REQUIRE(result.vsc_limit_states.size() == 1);
  CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
  CHECK(result.diagnostics.gfm_island_reference_vsc_indices ==
        std::vector<int>{91});
  CHECK(result.converter_model_scope.validity.vsc_gfm_island_reference_modelled);
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  CHECK(result.diagnostics.equation_closure_ok);
}

TEST_CASE("Slackless GFM island converges to the authored absolute angle from a rotated seed",
          "[power_flow][converter][gfm][ncp][island][initial_state]") {
  const auto system = make_islanded_gfm_case();
  hacdcpf::PowerFlowOptions baseline_options;
  baseline_options.tol = 1e-10;
  baseline_options.max_iter = 100;
  const auto baseline = hacdcpf::solve_power_flow(system, baseline_options);
  REQUIRE(baseline.converged);

  hacdcpf::PowerFlowOptions rotated_options = baseline_options;
  rotated_options.initial_state = hacdcpf::InitialState{
      baseline.vm, baseline.va, baseline.vdc};
  for (double& angle : rotated_options.initial_state->va) {
    angle += 0.05;
  }
  const auto rotated = hacdcpf::solve_power_flow(system, rotated_options);
  REQUIRE(rotated.converged);
  REQUIRE(rotated.va.size() == baseline.va.size());
  for (size_t bus = 0; bus < baseline.va.size(); ++bus) {
    CHECK(rotated.va[bus] == Catch::Approx(baseline.va[bus]).margin(1e-9));
    CHECK(rotated.vm[bus] == Catch::Approx(baseline.vm[bus]).margin(1e-10));
  }
}

TEST_CASE("Multiple Norton internal phasors share a slackless island without terminal overconstraint",
          "[power_flow][converter][gfm][ncp][island][multi_converter]") {
  const auto system = make_two_islanded_gfm_case();
  const auto coordination =
      hacdcpf::powerflow::evaluate_converter_coordination(system, true);
  std::ostringstream coordination_detail;
  for (const auto& issue : coordination.issues) {
    coordination_detail << issue.rule_id << ": " << issue.message << '\n';
  }
  INFO(coordination_detail.str());
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  options.enable_converter_coordination_check = true;
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("residual=" << result.residual << " iterations=" << result.iterations);
  INFO("termination=" << result.diagnostics.termination_reason);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 2);
  CHECK(result.diagnostics.gfm_island_reference_vsc_indices ==
        std::vector<int>{91, 92});
  CHECK(result.diagnostics.converter_coordination.feasible);
  CHECK(std::none_of(
      result.diagnostics.converter_coordination.issues.begin(),
      result.diagnostics.converter_coordination.issues.end(),
      [](const auto& issue) { return issue.rule_id == "ACISLAND-REF-02"; }));
  for (const auto& state : result.vsc_limit_states) {
    CHECK(state.complementarity_residual <= 1e-9);
  }
}

TEST_CASE("Adaptive multi-island solve preserves the GFM reference instead of manufacturing a slack",
          "[power_flow][converter][gfm][ncp][island][adaptive]") {
  auto system = make_islanded_gfm_case();
  system.ac.branches.front().index = 711;
  system.dc.branches.front().index = 731;
  hacdcpf::Load zero_load;
  zero_load.index = 761;
  zero_load.bus = 2;
  system.ac.loads.push_back(zero_load);
  hacdcpf::ACBus dead_bus;
  dead_bus.index = 77;
  dead_bus.bus_type = hacdcpf::BusType::PQ;
  dead_bus.vm_pu = 1.0;
  dead_bus.in_service = true;
  system.ac.buses.push_back(dead_bus);

  const auto detected = hacdcpf::powerflow::detect_islands(system);
  const auto gfm_island = std::find_if(
      detected.begin(), detected.end(), [](const hacdcpf::IslandInfo& island) {
        return !island.gfm_reference_vsc_indices.empty();
      });
  REQUIRE(gfm_island != detected.end());
  CHECK_FALSE(gfm_island->has_ac_slack);
  CHECK(gfm_island->has_ac_angle_reference);
  CHECK(gfm_island->has_generators);
  CHECK(gfm_island->gfm_reference_vsc_indices == std::vector<int>{91});

  const auto extracted =
      hacdcpf::powerflow::extract_island_subsystem(system, *gfm_island);
  REQUIRE(extracted.vsc_converters.size() == 1);
  CHECK(extracted.vsc_converters.front().index == 91);
  REQUIRE(extracted.ac.branches.size() == 1);
  CHECK(extracted.ac.branches.front().index == 711);
  REQUIRE(extracted.dc.branches.size() == 1);
  CHECK(extracted.dc.branches.front().index == 731);
  REQUIRE(extracted.ac.loads.size() == 1);
  CHECK(extracted.ac.loads.front().index == 761);
  REQUIRE(extracted.dc.storage.size() == 1);
  CHECK(extracted.dc.storage.front().index == 501);
  CHECK(std::none_of(extracted.ac.buses.begin(), extracted.ac.buses.end(),
                     [](const hacdcpf::ACBus& bus) {
                       return bus.bus_type == hacdcpf::BusType::SLACK;
                     }));

  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  options.enable_converter_coordination_check = true;
  const auto result = hacdcpf::solve_power_flow_adaptive(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.islands.size() == 2);
  CHECK(result.vm.back() == 0.0);
  CHECK(result.diagnostics.gfm_island_reference_vsc_indices ==
        std::vector<int>{91});
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
}

TEST_CASE("Production GFM Norton demo exposes a certified flat-start result",
          "[power_flow][converter][gfm][ncp][builtin]") {
  const auto system = hacdcpf::io::build_gfm_norton_limit_demo();
  REQUIRE(system.ac.buses.size() == 2);
  REQUIRE(system.dc.buses.size() == 2);
  REQUIRE(system.vsc_converters.size() == 2);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  options.enable_converter_coordination_check = true;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  CHECK(result.diagnostics.converter_coordination.feasible);
  REQUIRE(result.vsc_limit_states.size() == 1);
  const auto& state = result.vsc_limit_states.front();
  CHECK(state.index == 91);
  CHECK(state.gfm_norton_model);
  CHECK(state.current_pu <= 0.8 + 1e-9);
  CHECK(state.complementarity_residual <= 1e-9);
  CHECK(result.converter_model_scope.validity.vsc_gfm_norton_modelled);
  CHECK(result.converter_model_scope.validity.vsc_gfm_priority_limit_enforced);
}

TEST_CASE("Two grid-connected GFM Norton ports limit simultaneously",
          "[power_flow][converter][gfm][ncp][multi_converter]") {
  const auto system = make_two_gfm_case();
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 120;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 2);
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  for (const auto& state : result.vsc_limit_states) {
    CHECK(state.gfm_norton_model);
    CHECK(state.current_limit_active);
    CHECK(state.current_pu <= 0.3 + 1e-9);
    CHECK(state.complementarity_residual <= 1e-9);
  }
}

TEST_CASE("GFM priority blocks remain finite at a zero remaining-radius tie",
          "[power_flow][converter][gfm][ncp][degenerate]") {
  constexpr double vm = 1.0;
  constexpr double va = 0.0;
  constexpr double limit = 0.3;
  const std::complex<double> terminal{vm, 0.0};
  for (const auto priority : {VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    auto converter = make_gfm_converter(priority, limit);
    const std::complex<double> impedance{
        converter.gfm_virtual_r_pu, converter.gfm_virtual_x_pu};
    const std::complex<double> tie_current =
        priority == VSCCurrentLimitPriority::ActivePower
            ? std::complex<double>{limit, 0.0}
            : std::complex<double>{0.0, -limit};
    const std::complex<double> internal =
        terminal + impedance * tie_current;
    converter.gfm_internal_voltage_set_pu = std::abs(internal);
    converter.gfm_internal_angle_set_deg =
        std::arg(internal) * 180.0 / std::numbers::pi;
    const auto state = initialize_vsc_limit_state(
        converter, vm, va, 1.0, 100.0, LossModelType::Linear);
    const auto evaluation = evaluate_vsc_limit_ncp(
        converter, vm, va, 1.0, 100.0, LossModelType::Linear, state);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    CHECK(state.allFinite());
    CHECK(evaluation.jacobian.allFinite());
    CHECK(evaluation.current_pu == Catch::Approx(limit).margin(1e-10));
    CHECK(evaluation.complementarity_residual <= 1e-10);
  }
}

TEST_CASE("Weak-grid GFM retains a certified limited operating point",
          "[power_flow][converter][gfm][ncp][weak_grid]") {
  auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude, 0.3);
  system.ac.branches.front().r_pu = 0.08;
  system.ac.branches.front().x_pu = 0.55;
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;
  options.max_iter = 160;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  CHECK(result.vsc_limit_states.front().current_limit_active);
  CHECK(result.vsc_limit_states.front().current_pu <= 0.3 + 1e-9);
  CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
}

TEST_CASE("Infeasible weak-grid GFM point is not reported as converged",
          "[power_flow][converter][gfm][ncp][infeasible]") {
  auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude, 0.3);
  system.ac.buses[1].pd_mw = 300.0;
  system.ac.buses[1].qd_mvar = 180.0;
  system.ac.branches.front().r_pu = 0.1;
  system.ac.branches.front().x_pu = 0.8;
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;
  options.max_iter = 40;
  options.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
  const auto result = hacdcpf::solve_power_flow(system, options);
  CHECK_FALSE(result.converged);
  CHECK(result.residual > options.tol);
}

TEST_CASE("Quasi-steady snapshots preserve the GFM Norton limit contract",
          "[power_flow][converter][gfm][ncp][time_series]") {
  const auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude);
  hacdcpf::TimeSeriesData timeline;
  timeline.num_steps = 3;
  timeline.step_duration_hr = 0.25;
  hacdcpf::TimeSeriesPFOptions options;
  options.skip_uc = true;
  options.run_opf = false;
  options.pf_options.tol = 1e-10;
  options.pf_options.max_iter = 100;

  const auto result = hacdcpf::solve_time_series_pf(
      system, timeline, options);
  REQUIRE(result.num_converged == timeline.num_steps);
  REQUIRE(result.pf_results.size() ==
          static_cast<size_t>(timeline.num_steps));
  for (const auto& snapshot : result.pf_results) {
    REQUIRE(snapshot.vsc_limit_states.size() == 1);
    REQUIRE(snapshot.vsc_transfers.size() == 1);
    const auto& state = snapshot.vsc_limit_states.front();
    const auto& transfer = snapshot.vsc_transfers.front();
    CHECK(state.index == 91);
    CHECK(transfer.index == 91);
    CHECK(state.gfm_norton_model);
    CHECK(transfer.gfm_norton_model);
    CHECK(state.current_pu <= 0.8 + 1e-9);
    CHECK(state.complementarity_residual <= 1e-9);
    CHECK(snapshot.converter_model_scope.validity.vsc_gfm_norton_modelled);
    CHECK(snapshot.converter_model_scope.validity
              .vsc_gfm_priority_limit_enforced);
  }
}

TEST_CASE("OPF dispatch replay preserves and certifies the authored GFM port",
          "[power_flow][converter][gfm][ncp][opf_replay]") {
  auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::ReactivePower);
  hacdcpf::Storage dc_boundary;
  dc_boundary.index = 501;
  dc_boundary.bus = 1;
  dc_boundary.p_rated_mw = 200.0;
  dc_boundary.pmin_mw = -200.0;
  dc_boundary.pmax_mw = 200.0;
  dc_boundary.e_rated_mwh = 1000.0;
  dc_boundary.soc_init = 0.5;
  dc_boundary.soc_min = 0.1;
  dc_boundary.soc_max = 0.9;
  dc_boundary.controllable = true;
  system.dc.storage.push_back(dc_boundary);
  hacdcpf::TimeSeriesData timeline;
  timeline.num_steps = 1;
  timeline.step_duration_hr = 1.0;
  hacdcpf::TimeSeriesPFOptions options;
  options.skip_uc = true;
  options.run_opf = true;
  options.opf_options.ac_solver_backend =
      hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
  options.opf_options.allow_fallback = false;
  options.pf_options.tol = 1e-10;
  options.pf_options.max_iter = 100;

  const auto result = hacdcpf::solve_time_series_pf(
      system, timeline, options);
  REQUIRE(result.opf_results.size() == 1);
  INFO(result.opf_results.front().status);
  REQUIRE(result.opf_results.front().converged);
  CHECK_FALSE(result.opf_results.front().converter_model_scope.validity
                  .vsc_gfm_norton_modelled);
  CHECK_FALSE(result.opf_results.front().converter_model_scope.validity
                  .vsc_gfm_priority_limit_enforced);
  CHECK(std::any_of(
      result.opf_results.front().model_limitations.begin(),
      result.opf_results.front().model_limitations.end(),
      [](const std::string& limitation) {
        return limitation.find("priority NCP") != std::string::npos;
      }));

  REQUIRE(result.pf_results.size() == 1);
  const auto& replay = result.pf_results.front();
  REQUIRE(replay.converged);
  REQUIRE(replay.vsc_limit_states.size() == 1);
  REQUIRE(replay.vsc_transfers.size() == 1);
  CHECK(replay.vsc_limit_states.front().index == 91);
  CHECK(replay.vsc_limit_states.front().gfm_norton_model);
  CHECK(replay.vsc_transfers.front().gfm_norton_model);
  CHECK(replay.vsc_limit_states.front().current_pu <= 0.8 + 1e-9);
  CHECK(replay.vsc_limit_states.front().complementarity_residual <= 1e-9);
  CHECK(replay.converter_model_scope.validity.vsc_gfm_norton_modelled);
  CHECK(replay.converter_model_scope.validity
            .vsc_gfm_priority_limit_enforced);
}

#ifdef HACDCPF_HAVE_OPENDSS
TEST_CASE("OpenDSS independently reproduces the nonbinding GFM Norton root",
          "[power_flow][converter][gfm][ncp][opendss][cross_validation]") {
  auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude, 5.0);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-11;
  options.max_iter = 100;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  const auto& state = result.vsc_limit_states.front();
  REQUIRE_FALSE(state.current_limit_active);
  const std::complex<double> internal{
      state.internal_voltage_real_pu, state.internal_voltage_imag_pu};
  const auto external = solve_opendss_gfm_circuit(internal);
  CHECK(result.vm[1] ==
        Catch::Approx(external.voltage_magnitude_pu).margin(1e-4));
  CHECK(result.va[1] ==
        Catch::Approx(external.voltage_angle_rad)
            .margin(0.02 * std::numbers::pi / 180.0));

  const std::complex<double> terminal =
      std::polar(external.voltage_magnitude_pu, external.voltage_angle_rad);
  const std::complex<double> current =
      (internal - terminal) / std::complex<double>{0.04, 0.20};
  const std::complex<double> power = terminal * std::conj(current);
  CHECK(state.p_ac_pu == Catch::Approx(power.real()).margin(1e-3));
  CHECK(state.q_ac_pu == Catch::Approx(power.imag()).margin(1e-3));
}

TEST_CASE("OpenDSS frozen-source replay certifies the binding GFM circuit",
          "[power_flow][converter][gfm][ncp][opendss][frozen_replay]") {
  const auto system = make_grid_connected_gfm_case(
      VSCCurrentLimitPriority::Magnitude, 0.3);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-11;
  options.max_iter = 100;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  const auto& state = result.vsc_limit_states.front();
  REQUIRE(state.current_limit_active);
  const std::complex<double> internal{
      state.internal_voltage_real_pu, state.internal_voltage_imag_pu};
  const auto external = solve_opendss_gfm_circuit(internal);
  const std::complex<double> terminal =
      std::polar(external.voltage_magnitude_pu, external.voltage_angle_rad);
  const std::complex<double> converter_current =
      (internal - terminal) / std::complex<double>{0.04, 0.20};

  CHECK(result.vm[1] ==
        Catch::Approx(external.voltage_magnitude_pu).margin(1e-6));
  CHECK(result.va[1] ==
        Catch::Approx(external.voltage_angle_rad).margin(1e-6));
  CHECK(std::abs(converter_current) == Catch::Approx(0.3).margin(1e-6));
  CHECK(external.terminal_kcl_residual_pu <= 1e-6);
}
#endif

TEST_CASE("VSC limit NCP agrees with the exhaustive priority-mode oracle",
          "[power_flow][converter][ncp][oracle]") {
  constexpr double vm = 0.95;
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    const VSCConverter converter = make_converter(priority);
    const auto expected = projection_oracle(converter, vm);
    const auto state = solve_local(converter, vm, 1.0);
    const auto evaluation = evaluate_vsc_limit_ncp(
        converter, vm, 0.0, 1.0, 100.0, LossModelType::Linear, state);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    CHECK(state[0] == Catch::Approx(expected[0]).margin(1e-10));
    CHECK(state[1] == Catch::Approx(expected[1]).margin(1e-10));
    CHECK(evaluation.complementarity_residual < 1e-10);
    CHECK(evaluation.current_pu <= converter.i_ac_max_pu + 1e-10);
    CHECK(evaluation.current_limit_active);
  }
}

TEST_CASE("Nonbinding VSC NCP preserves the authored command",
          "[power_flow][converter][ncp][compatibility]") {
  VSCConverter converter = make_converter(VSCCurrentLimitPriority::Magnitude);
  converter.p_set_mw = 20.0;
  converter.q_set_mvar = -10.0;
  const auto state = solve_local(converter, 1.02, 1.0);
  const auto evaluation = evaluate_vsc_limit_ncp(
      converter, 1.02, 0.0, 1.0, 100.0, LossModelType::Linear, state);
  CHECK(state[0] == Catch::Approx(0.2).margin(1e-10));
  CHECK(state[1] == Catch::Approx(-0.1).margin(1e-10));
  CHECK_FALSE(evaluation.current_limit_active);
  CHECK(evaluation.complementarity_residual < 1e-10);
}

TEST_CASE("Vdc droop saturation and derivative are represented in the local block",
          "[power_flow][converter][ncp][droop]") {
  VSCConverter converter = make_converter(VSCCurrentLimitPriority::Magnitude);
  converter.control_mode = ConverterMode::VDC_Q;
  converter.k_vdc = 10.0;
  converter.droop_p_min_mw = -40.0;
  converter.droop_p_max_mw = 40.0;
  converter.q_set_mvar = 0.0;
  const auto state = solve_local(converter, 1.0, 1.2);
  const auto evaluation = evaluate_vsc_limit_ncp(
      converter, 1.0, 0.0, 1.2, 100.0, LossModelType::Linear, state);
  CHECK(state[0] == Catch::Approx(0.4).margin(1e-10));
  CHECK(evaluation.droop_saturated);
  CHECK(evaluation.derivative_vdc[0] == Catch::Approx(0.0));
}

TEST_CASE("VSC local generalized Jacobian matches finite differences off switching ties",
          "[power_flow][converter][ncp][jacobian]") {
  VSCConverter converter = make_converter(VSCCurrentLimitPriority::Magnitude);
  const double vm = 0.95;
  const double vdc = 1.0;
  auto state = solve_local(converter, vm, vdc);
  state[0] *= 0.97;
  state[1] *= 1.02;
  state[5] += 0.03;
  const auto analytic = evaluate_vsc_limit_ncp(
      converter, vm, 0.0, vdc, 100.0, LossModelType::Linear, state);
  constexpr double step = 1e-7;
  for (int column = 0; column < hacdcpf::powerflow::kVSCLimitStateSize; ++column) {
    auto plus = state;
    auto minus = state;
    plus[column] += step;
    minus[column] -= step;
    const auto ep = evaluate_vsc_limit_ncp(
        converter, vm, 0.0, vdc, 100.0, LossModelType::Linear, plus);
    const auto em = evaluate_vsc_limit_ncp(
        converter, vm, 0.0, vdc, 100.0, LossModelType::Linear, minus);
    const Eigen::Matrix<double, hacdcpf::powerflow::kVSCLimitStateSize, 1> finite_difference =
        (ep.equation - em.equation) / (2.0 * step);
    CHECK((finite_difference - analytic.jacobian.col(column))
              .cwiseAbs().maxCoeff() < 2e-7);
  }
}

TEST_CASE("Hybrid Newton VSC limit states agree with the priority oracle",
          "[power_flow][converter][ncp][integration][oracle]") {
  using hacdcpf::powerflow::NewtonSolver;
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    auto data = hacdcpf::powerflow::make_solver_data(
        make_hybrid_case(priority, true));
    hacdcpf::PowerFlowOptions options;
    options.tol = 1e-10;
    options.max_iter = 80;
    options.robust_nonlinear.vsc_schur_min_network_dimension = 0;
    NewtonSolver solver;
    const auto result = solver.solve(data, options);
    options.robust_nonlinear.enable_vsc_local_schur = false;
    NewtonSolver full_solver;
    const auto full_result = full_solver.solve(data, options);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    INFO("schur residual=" << result.residual
                            << " termination="
                            << result.diagnostics.termination_reason
                            << " status="
                            << result.profiling.vsc_schur_status
                            << " accepted="
                            << result.profiling.vsc_schur_accepted
                            << " fallbacks="
                            << result.profiling.vsc_schur_fallbacks
                            << " local_rcond="
                            << result.profiling.vsc_schur_minimum_local_rcond
                            << " full_backward="
                            << result.profiling.max_vsc_schur_full_backward_error);
    INFO("full residual=" << full_result.residual
                           << " termination="
                           << full_result.diagnostics.termination_reason);
    REQUIRE(full_result.converged);
    REQUIRE(result.converged);
    REQUIRE(result.vsc_limit_states.size() == 1);
    const auto& state = result.vsc_limit_states.front();
    const auto expected = projection_oracle(data.converters.front(), result.vm[1]);
    CHECK(state.index == 77);
    CHECK(state.p_ac_pu == Catch::Approx(expected[0]).margin(1e-8));
    CHECK(state.q_ac_pu == Catch::Approx(expected[1]).margin(1e-8));
    CHECK(state.current_pu <= data.converters.front().i_ac_max_pu + 1e-9);
    CHECK(state.complementarity_residual <= 1e-9);
    CHECK(state.current_limit_active);
    CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  }
}

TEST_CASE("Nonbinding hybrid VSC NCP preserves the legacy network root",
          "[power_flow][converter][ncp][integration][compatibility]") {
  auto legacy_data = hacdcpf::powerflow::make_solver_data(
      make_hybrid_case(VSCCurrentLimitPriority::Magnitude, false, 20.0, -10.0));
  auto ncp_data = hacdcpf::powerflow::make_solver_data(
      make_hybrid_case(VSCCurrentLimitPriority::Magnitude, true, 20.0, -10.0));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-11;
  options.max_iter = 80;
  hacdcpf::powerflow::NewtonSolver legacy_solver;
  hacdcpf::powerflow::NewtonSolver ncp_solver;
  const auto legacy = legacy_solver.solve(legacy_data, options);
  const auto ncp = ncp_solver.solve(ncp_data, options);
  REQUIRE(legacy.converged);
  REQUIRE(ncp.converged);
  REQUIRE(ncp.vsc_limit_states.size() == 1);
  for (size_t bus = 0; bus < legacy.vm.size(); ++bus) {
    CHECK(ncp.vm[bus] == Catch::Approx(legacy.vm[bus]).margin(1e-8));
    CHECK(ncp.va[bus] == Catch::Approx(legacy.va[bus]).margin(1e-8));
  }
  for (size_t bus = 0; bus < legacy.vdc.size(); ++bus) {
    CHECK(ncp.vdc[bus] == Catch::Approx(legacy.vdc[bus]).margin(1e-8));
  }
  CHECK(ncp.vsc_limit_states.front().complementarity_residual <= 1e-9);
  CHECK_FALSE(ncp.vsc_limit_states.front().current_limit_active);
}

TEST_CASE("Unified power-flow facade preserves solver-native VSC limit certificates",
          "[power_flow][converter][ncp][integration][facade]") {
  const auto system = make_hybrid_case(
      VSCCurrentLimitPriority::ReactivePower, true);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 80;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  REQUIRE(result.vsc_transfers.size() == 1);
  const auto& state = result.vsc_limit_states.front();
  const auto& transfer = result.vsc_transfers.front();
  CHECK(transfer.index == 77);
  CHECK(transfer.p_ac_mw == Catch::Approx(100.0 * state.p_ac_pu).margin(1e-8));
  CHECK(transfer.q_ac_mvar == Catch::Approx(100.0 * state.q_ac_pu).margin(1e-8));
  CHECK(transfer.p_dc_mw == Catch::Approx(100.0 * state.p_dc_pu).margin(1e-8));
  CHECK(transfer.limit_ncp_enabled);
  CHECK(transfer.current_limit_active == state.current_limit_active);
  CHECK(transfer.droop_saturated == state.droop_saturated);
  CHECK(transfer.current_limit_priority == "reactive_power");
  CHECK(transfer.effective_mode == "PQ");
  CHECK(transfer.complementarity_residual <= 1e-9);
  CHECK(result.converter_model_scope.model_scope.find("vsc-current-limit-ncp") !=
        std::string::npos);
  CHECK(result.converter_model_scope.validity.vsc_current_limits_enforced);
}

TEST_CASE("VSC limit-NCP configuration survives standard and JPC JSON round trips",
          "[power_flow][converter][ncp][io][roundtrip]") {
  const auto system = make_hybrid_case(
      VSCCurrentLimitPriority::ActivePower, true);
  auto gfm_system = system;
  gfm_system.vsc_converters.front() = make_gfm_converter(
      VSCCurrentLimitPriority::ReactivePower, 0.55);
  const auto standard = hacdcpf::io::from_json(hacdcpf::io::to_json(system, -1));
  const auto jpc =
      hacdcpf::io::from_jpc_json(hacdcpf::io::to_jpc_json(system, -1));
  for (const auto* restored : {&standard, &jpc}) {
    REQUIRE(restored->vsc_converters.size() == 1);
    const auto& converter = restored->vsc_converters.front();
    CHECK(converter.index == 77);
    CHECK(converter.enable_limit_ncp);
    CHECK(converter.current_limit_priority ==
          VSCCurrentLimitPriority::ActivePower);
    CHECK(converter.i_ac_max_pu == Catch::Approx(0.8));
    CHECK(converter.droop_p_min_mw == Catch::Approx(0.0));
    CHECK(converter.droop_p_max_mw == Catch::Approx(0.0));
  }
  const auto gfm_standard =
      hacdcpf::io::from_json(hacdcpf::io::to_json(gfm_system, -1));
  const auto gfm_jpc =
      hacdcpf::io::from_jpc_json(hacdcpf::io::to_jpc_json(gfm_system, -1));
  for (const auto* restored : {&gfm_standard, &gfm_jpc}) {
    REQUIRE(restored->vsc_converters.size() == 1);
    const auto& converter = restored->vsc_converters.front();
    CHECK(converter.control_mode == ConverterMode::AC_GRID_FORMING);
    CHECK(converter.enable_limit_ncp);
    CHECK(converter.current_limit_priority ==
          VSCCurrentLimitPriority::ReactivePower);
    CHECK(converter.i_ac_max_pu == Catch::Approx(0.55));
    CHECK(converter.gfm_internal_voltage_set_pu == Catch::Approx(1.08));
    CHECK(converter.gfm_internal_angle_set_deg == Catch::Approx(12.0));
    CHECK(converter.gfm_virtual_r_pu == Catch::Approx(0.04));
    CHECK(converter.gfm_virtual_x_pu == Catch::Approx(0.20));
  }
}

TEST_CASE("Unanchored AC islands and fixed-terminal VSC NCP requests fail closed",
          "[power_flow][converter][ncp][validation]") {
  hacdcpf::PowerFlowOptions options;
  hacdcpf::powerflow::NewtonSolver solver;

  auto unanchored = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true);
  for (auto& bus : unanchored.ac.buses) {
    bus.bus_type = hacdcpf::BusType::PQ;
  }
  unanchored.ac.generators.clear();
  const auto unanchored_data =
      hacdcpf::powerflow::make_solver_data(std::move(unanchored));
  CHECK_THROWS_AS(solver.solve(unanchored_data, options), std::invalid_argument);

  auto reference_terminal = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true);
  reference_terminal.vsc_converters.front().bus_ac = 1;
  const auto reference_data =
      hacdcpf::powerflow::make_solver_data(std::move(reference_terminal));
  CHECK_THROWS_AS(solver.solve(reference_data, options), std::invalid_argument);
}

TEST_CASE("Two VSC local blocks can activate simultaneously without rebuilding the pattern",
          "[power_flow][converter][ncp][integration][multi_converter]") {
  auto data = hacdcpf::powerflow::make_solver_data(make_two_converter_case());
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  hacdcpf::powerflow::NewtonSolver solver;
  const auto result = solver.solve(data, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 2);
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  for (const auto& state : result.vsc_limit_states) {
    const auto converter = std::find_if(
        data.converters.begin(), data.converters.end(),
        [&](const VSCConverter& item) { return item.index == state.index; });
    REQUIRE(converter != data.converters.end());
    const int ac_position = converter->bus_ac - 1;
    const auto expected = projection_oracle(*converter, result.vm[ac_position]);
    CHECK(state.p_ac_pu == Catch::Approx(expected[0]).margin(1e-8));
    CHECK(state.q_ac_pu == Catch::Approx(expected[1]).margin(1e-8));
    CHECK(state.current_pu <= converter->i_ac_max_pu + 1e-9);
    CHECK(state.complementarity_residual <= 1e-9);
    CHECK(state.current_limit_active);
  }
}

TEST_CASE("Priority NCP remains finite at a zero remaining-current-radius intersection",
          "[power_flow][converter][ncp][degenerate]") {
  for (const auto priority : {VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    VSCConverter converter = make_converter(priority);
    converter.p_set_mw = priority == VSCCurrentLimitPriority::ActivePower
                             ? 100.0
                             : 60.0;
    converter.q_set_mvar = priority == VSCCurrentLimitPriority::ReactivePower
                               ? 100.0
                               : 60.0;
    converter.i_ac_max_pu = 0.8;
    const auto state = solve_local(converter, 1.0, 1.0);
    const auto evaluation = evaluate_vsc_limit_ncp(
        converter, 1.0, 0.0, 1.0, 100.0, LossModelType::Linear, state);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    CHECK(state.allFinite());
    CHECK(evaluation.jacobian.allFinite());
    CHECK(evaluation.complementarity_residual <= 1e-10);
    CHECK(evaluation.current_pu == Catch::Approx(0.8).margin(1e-10));
    if (priority == VSCCurrentLimitPriority::ActivePower) {
      CHECK(state[0] == Catch::Approx(0.8).margin(1e-10));
      CHECK(state[1] == Catch::Approx(0.0).margin(1e-10));
    } else {
      CHECK(state[0] == Catch::Approx(0.0).margin(1e-10));
      CHECK(state[1] == Catch::Approx(0.8).margin(1e-10));
    }
  }
}

TEST_CASE("Magnitude NCP is exact at the biactive current-circle intersection",
          "[power_flow][converter][ncp][degenerate][oracle]") {
  VSCConverter converter = make_converter(VSCCurrentLimitPriority::Magnitude);
  converter.p_set_mw = 80.0;
  converter.q_set_mvar = 0.0;
  converter.i_ac_max_pu = 0.8;
  const auto state = initialize_vsc_limit_state(
      converter, 1.0, 0.0, 1.0, 100.0, LossModelType::Linear);
  const auto exact = evaluate_vsc_limit_ncp(
      converter, 1.0, 0.0, 1.0, 100.0, LossModelType::Linear, state);
  CHECK(state[5] == 0.0);
  CHECK(std::abs(exact.equation[5]) <= 2e-15);
  CHECK(exact.complementarity_residual <= 2e-15);
  CHECK(exact.jacobian.allFinite());

  constexpr double mu = 1e-3;
  constexpr double step = 1e-7;
  const auto smooth = evaluate_vsc_limit_ncp(
      converter, 1.0, 0.0, 1.0, 100.0, LossModelType::Linear, state, mu);
  for (const int column : {0, 1, 5}) {
    auto plus = state;
    auto minus = state;
    plus[column] += step;
    minus[column] -= step;
    const auto ep = evaluate_vsc_limit_ncp(
        converter, 1.0, 0.0, 1.0, 100.0,
        LossModelType::Linear, plus, mu);
    const auto em = evaluate_vsc_limit_ncp(
        converter, 1.0, 0.0, 1.0, 100.0,
        LossModelType::Linear, minus, mu);
    const auto finite_difference =
        (ep.equation - em.equation) / (2.0 * step);
    CHECK((finite_difference - smooth.jacobian.col(column))
              .cwiseAbs().maxCoeff() <= 5e-7);
  }
}

TEST_CASE("Smooth continuation covers all VSC priorities without changing the sparse layout",
          "[power_flow][converter][ncp][smoothing][integration]") {
  for (const auto priority : {VSCCurrentLimitPriority::Magnitude,
                              VSCCurrentLimitPriority::ActivePower,
                              VSCCurrentLimitPriority::ReactivePower}) {
    auto data = hacdcpf::powerflow::make_solver_data(
        make_hybrid_case(priority, true));
    hacdcpf::PowerFlowOptions options;
    options.tol = 1e-9;
    options.max_iter = 40;
    options.robust_nonlinear.enable_smooth_ncp = true;
    options.robust_nonlinear.ncp_mu0 = 1e-2;
    options.robust_nonlinear.ncp_mu_min = 1e-12;
    options.robust_nonlinear.ncp_mu_factor = 0.1;
    options.robust_nonlinear.ncp_mu_factor_coarse = 0.5;
    hacdcpf::powerflow::NewtonSolver solver;
    const auto result = solver.solve(data, options);
    INFO(hacdcpf::powerflow::vsc_current_limit_priority_str(priority));
    INFO("residual=" << result.residual
                      << " iterations=" << result.iterations
                      << " updates="
                      << result.profiling.smooth_ncp_continuation_updates);
    REQUIRE(result.converged);
    CHECK(result.iterations <= 40);
    CHECK(result.profiling.smooth_ncp_continuation_updates >= 1);
    CHECK(result.profiling.smooth_ncp_final_mu <= 1e-12);
    CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
    REQUIRE(result.vsc_limit_states.size() == 1);
    CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
  }
}

TEST_CASE("Weak-grid hybrid solve retains a certified current-limited root",
          "[power_flow][converter][ncp][integration][weak_grid]") {
  auto system = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true, -65.0, -45.0);
  system.ac.branches.front().r_pu = 0.08;
  system.ac.branches.front().x_pu = 0.55;
  auto data = hacdcpf::powerflow::make_solver_data(std::move(system));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;
  options.max_iter = 120;
  hacdcpf::powerflow::NewtonSolver solver;
  const auto result = solver.solve(data, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  CHECK(result.vm[1] < 0.9);
  CHECK(result.vsc_limit_states.front().current_limit_active);
  CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
}

TEST_CASE("Vdc droop saturation is enforced inside the full hybrid Newton system",
          "[power_flow][converter][ncp][integration][droop]") {
  auto system = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true, 0.0, 0.0);
  auto& converter = system.vsc_converters.front();
  converter.control_mode = ConverterMode::VDC_Q;
  converter.k_vdc = 10.0;
  converter.droop_p_min_mw = -40.0;
  converter.droop_p_max_mw = 40.0;
  system.dc.buses.front().vm_pu = 1.2;
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 100;
  const auto result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == 1);
  REQUIRE(result.vsc_transfers.size() == 1);
  CHECK(result.vsc_limit_states.front().p_ac_pu ==
        Catch::Approx(0.4).margin(1e-8));
  CHECK(result.vsc_limit_states.front().droop_saturated);
  CHECK(result.vsc_transfers.front().droop_saturated);
  CHECK(result.vsc_limit_states.front().complementarity_residual <= 1e-9);
  const auto serialized = nlohmann::json::parse(
      hacdcpf::io::power_flow_result_to_json(system, result));
  REQUIRE(serialized.at("vsc_transfers").size() == 1);
  CHECK(serialized.at("vsc_transfers").front().at("droop_saturated") == true);
}

TEST_CASE("An infeasible weak-grid operating point is not returned as converged",
          "[power_flow][converter][ncp][integration][infeasible]") {
  auto system = make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true, 0.0, 0.0);
  system.ac.buses[1].pd_mw = 300.0;
  system.ac.buses[1].qd_mvar = 180.0;
  system.ac.branches.front().r_pu = 0.1;
  system.ac.branches.front().x_pu = 0.8;
  auto data = hacdcpf::powerflow::make_solver_data(std::move(system));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;
  options.max_iter = 40;
  options.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
  hacdcpf::powerflow::NewtonSolver solver;
  const auto result = solver.solve(data, options);
  CHECK_FALSE(result.converged);
  CHECK(result.residual > options.tol);
}

TEST_CASE("Production Newton falls back to full LU when the Schur guard rejects",
          "[power_flow][converter][ncp][schur][fallback]") {
  auto data = hacdcpf::powerflow::make_solver_data(make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true, 45.0, 20.0));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 60;
  options.robust_nonlinear.vsc_schur_local_rcond_tolerance = 1.0;
  options.robust_nonlinear.vsc_schur_min_network_dimension = 0;
  options.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
  hacdcpf::powerflow::NewtonSolver solver;
  const auto result = solver.solve(data, options);
  REQUIRE(result.converged);
  CHECK(result.profiling.vsc_schur_attempts >= 1);
  CHECK(result.profiling.vsc_schur_accepted == 0);
  CHECK(result.profiling.vsc_schur_fallbacks ==
        result.profiling.vsc_schur_attempts);
  CHECK(result.profiling.factorization_calls >
        result.profiling.vsc_schur_sparse_factorizations);
  CHECK(result.profiling.max_refactor_backward_error <=
        options.robust_nonlinear.refactor_backward_error_tolerance);
}

TEST_CASE("Production Schur admission keeps small hybrid systems on full LU",
          "[power_flow][converter][ncp][schur][admission]") {
  auto data = hacdcpf::powerflow::make_solver_data(make_hybrid_case(
      VSCCurrentLimitPriority::Magnitude, true, 45.0, 20.0));
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  hacdcpf::powerflow::NewtonSolver solver;
  const auto result = solver.solve(data, options);
  REQUIRE(result.converged);
  CHECK(result.profiling.vsc_schur_attempts == 0);
  CHECK(result.profiling.vsc_schur_status == "not_attempted");
}

TEST_CASE("case300 limit-NCP benchmark is genuinely hybrid and structurally fixed",
          "[power_flow][converter][ncp][benchmark][case300]") {
  const auto system = hacdcpf::io::build_case300_acdc_vsc_limit_ncp();
  REQUIRE(system.ac.buses.size() == 300);
  REQUIRE(system.dc.buses.size() == 6);
  REQUIRE(system.vsc_converters.size() == 6);
  CHECK(std::all_of(system.dc.buses.begin(), system.dc.buses.end(),
                    [](const hacdcpf::DCBus& bus) {
                      return bus.bus_type == hacdcpf::DCBusType::DC_P;
                    }));
  CHECK(std::all_of(system.vsc_converters.begin(),
                    system.vsc_converters.end(),
                    [](const VSCConverter& converter) {
                      return converter.enable_limit_ncp &&
                             converter.i_ac_max_pu > 0.0;
                    }));

  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 120;
  options.robust_nonlinear.vsc_schur_min_network_dimension = 0;
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("termination=" << result.diagnostics.termination_reason
                       << " residual=" << result.residual
                       << " iterations=" << result.iterations);
  for (const auto& warning : result.diagnostics.warnings) INFO(warning);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == system.vsc_converters.size());
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  CHECK(result.profiling.vsc_schur_accepted >= 1);
  CHECK(result.profiling.vsc_schur_full_dimension -
            result.profiling.vsc_schur_reduced_dimension ==
        6 * static_cast<int>(system.vsc_converters.size()));
  CHECK(result.profiling.vsc_schur_full_structural_nnz -
            result.profiling.vsc_schur_reduced_structural_nnz >=
        48 * static_cast<std::int64_t>(system.vsc_converters.size()));
  CHECK(result.profiling.max_vsc_schur_full_backward_error <= 1e-8);
  CHECK(result.diagnostics.n_equations == result.diagnostics.n_variables);
  CHECK(result.diagnostics.equation_closure_ok);
  CHECK(std::count_if(result.vsc_limit_states.begin(),
                      result.vsc_limit_states.end(),
                      [](const hacdcpf::VSCLimitStateResult& state) {
                        return state.current_limit_active;
                      }) >= 3);
  for (const auto& state : result.vsc_limit_states) {
    CHECK(state.complementarity_residual <= 1e-9);
  }

  options.robust_nonlinear.enable_vsc_local_schur = false;
  const auto full_result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(full_result.converged);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.vm.data(), result.vm.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.vm.data(),
                                           full_result.vm.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.va.data(), result.va.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.va.data(),
                                           full_result.va.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.vdc.data(), result.vdc.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.vdc.data(),
                                           full_result.vdc.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);

  options.robust_nonlinear.enable_vsc_local_schur = true;
  options.robust_nonlinear.vsc_schur_min_network_dimension =
      hacdcpf::Defaults::kVSCSchurMinNetworkDimension;
  const auto admitted_result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(admitted_result.converged);
  CHECK(admitted_result.profiling.vsc_schur_attempts == 0);
}

TEST_CASE("case2000 limit-NCP benchmark uses the real hybrid source and fixed pattern",
          "[power_flow][converter][ncp][benchmark][case2000]") {
  const auto system = hacdcpf::io::build_case2000_acdc_vsc_limit_ncp();
  REQUIRE(system.ac.buses.size() == 2000);
  REQUIRE(system.dc.buses.size() == 8);
  REQUIRE(system.vsc_converters.size() == system.dc.buses.size());

  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 120;
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("termination=" << result.diagnostics.termination_reason
                       << " residual=" << result.residual
                       << " iterations=" << result.iterations);
  for (const auto& warning : result.diagnostics.warnings) INFO(warning);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == system.vsc_converters.size());
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  CHECK(result.profiling.vsc_schur_accepted >= 1);
  CHECK(result.profiling.vsc_schur_full_dimension -
            result.profiling.vsc_schur_reduced_dimension ==
        6 * static_cast<int>(system.vsc_converters.size()));
  CHECK(result.profiling.vsc_schur_full_structural_nnz -
            result.profiling.vsc_schur_reduced_structural_nnz >=
        48 * static_cast<std::int64_t>(system.vsc_converters.size()));
  CHECK(result.profiling.max_vsc_schur_full_backward_error <= 1e-8);
  CHECK(result.diagnostics.n_equations == result.diagnostics.n_variables);
  CHECK(result.diagnostics.equation_closure_ok);
  CHECK(std::count_if(result.vsc_limit_states.begin(),
                      result.vsc_limit_states.end(),
                      [](const hacdcpf::VSCLimitStateResult& state) {
                        return state.current_limit_active;
                      }) >= 1);
  for (const auto& state : result.vsc_limit_states) {
    CHECK(state.complementarity_residual <= 1e-9);
  }

  options.robust_nonlinear.enable_vsc_local_schur = false;
  const auto full_result = hacdcpf::solve_power_flow(system, options);
  REQUIRE(full_result.converged);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.vm.data(), result.vm.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.vm.data(),
                                           full_result.vm.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.va.data(), result.va.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.va.data(),
                                           full_result.va.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);
  CHECK((Eigen::Map<const Eigen::VectorXd>(result.vdc.data(), result.vdc.size()) -
         Eigen::Map<const Eigen::VectorXd>(full_result.vdc.data(),
                                           full_result.vdc.size()))
            .cwiseAbs()
            .maxCoeff() <= 1e-8);
}

TEST_CASE("case300 GFM benchmark embeds the certified MTDC operating point",
          "[power_flow][converter][gfm][ncp][benchmark][case300_gfm]") {
  const auto source_system = hacdcpf::io::build_case300_acdc_vsc_limit_ncp();
  hacdcpf::PowerFlowOptions source_options;
  source_options.tol = 1e-10;
  source_options.max_iter = 120;
  const auto source_result = hacdcpf::solve_power_flow(source_system, source_options);
  REQUIRE(source_result.converged);
  const auto system = hacdcpf::io::build_case300_acdc_gfm_limit_ncp();
  REQUIRE(system.ac.buses.size() == 300);
  REQUIRE(system.dc.buses.size() == 6);
  REQUIRE(system.vsc_converters.size() == 7);
  REQUIRE(system.vsc_converters.back().control_mode ==
          ConverterMode::AC_GRID_FORMING);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 160;
  options.enable_converter_mode_switching = false;
  options.initial_state = hacdcpf::InitialState{
      source_result.vm, source_result.va, source_result.vdc};
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("residual=" << result.residual << " iterations=" << result.iterations);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == system.vsc_converters.size());
  const auto gfm = std::find_if(
      result.vsc_limit_states.begin(), result.vsc_limit_states.end(),
      [](const hacdcpf::VSCLimitStateResult& state) {
        return state.gfm_norton_model;
      });
  REQUIRE(gfm != result.vsc_limit_states.end());
  CHECK(gfm->index == system.vsc_converters.back().index);
  CHECK(gfm->current_pu <= 1.25 + 1e-9);
  CHECK(gfm->complementarity_residual <= 1e-9);
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  CHECK(result.diagnostics.equation_closure_ok);
}

TEST_CASE("case2000 GFM benchmark uses the real ACTIVSg2000 MTDC source",
          "[power_flow][converter][gfm][ncp][benchmark][case2000_gfm]") {
  const auto source_system = hacdcpf::io::build_case2000_acdc_vsc_limit_ncp();
  hacdcpf::PowerFlowOptions source_options;
  source_options.tol = 1e-10;
  source_options.max_iter = 120;
  const auto source_result = hacdcpf::solve_power_flow(source_system, source_options);
  REQUIRE(source_result.converged);
  const auto system = hacdcpf::io::build_case2000_acdc_gfm_limit_ncp();
  REQUIRE(system.ac.buses.size() == 2000);
  REQUIRE(system.dc.buses.size() == 8);
  REQUIRE(system.vsc_converters.size() == 9);
  REQUIRE(system.vsc_converters.back().control_mode ==
          ConverterMode::AC_GRID_FORMING);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 180;
  options.enable_converter_mode_switching = false;
  options.initial_state = hacdcpf::InitialState{
      source_result.vm, source_result.va, source_result.vdc};
  const auto result = hacdcpf::solve_power_flow(system, options);
  INFO("residual=" << result.residual << " iterations=" << result.iterations);
  REQUIRE(result.converged);
  REQUIRE(result.vsc_limit_states.size() == system.vsc_converters.size());
  const auto gfm = std::find_if(
      result.vsc_limit_states.begin(), result.vsc_limit_states.end(),
      [](const hacdcpf::VSCLimitStateResult& state) {
        return state.gfm_norton_model;
      });
  REQUIRE(gfm != result.vsc_limit_states.end());
  CHECK(gfm->index == system.vsc_converters.back().index);
  CHECK(gfm->current_pu <= 1.25 + 1e-9);
  CHECK(gfm->complementarity_residual <= 1e-9);
  CHECK(result.profiling.jacobian_pattern_rebuilds == 1);
  CHECK(result.diagnostics.equation_closure_ok);
}

#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "hacdcpf/power_flow/three_phase_hybrid.hpp"

namespace {

using Complex = std::complex<double>;
using hacdcpf::powerflow::ThreePhaseHybridPFCase;
using hacdcpf::powerflow::ThreePhaseHybridPFControlMode;
using hacdcpf::powerflow::ThreePhaseHybridPFConverter;
using hacdcpf::powerflow::ThreePhaseHybridPFOptions;

ThreePhaseHybridPFCase make_case(ThreePhaseHybridPFControlMode mode) {
  ThreePhaseHybridPFCase c;
  c.name = "external_two_bus_unbalanced_hybrid";
  c.base_mva = 1.0;
  c.y_ac.resize(6, 6);
  std::vector<Eigen::Triplet<Complex>> trips;
  const Complex y = 1.0 / Complex{0.025, 0.08};
  for (int phase = 0; phase < 3; ++phase) {
    trips.emplace_back(phase, phase, y);
    trips.emplace_back(phase, 3 + phase, -y);
    trips.emplace_back(3 + phase, phase, -y);
    trips.emplace_back(3 + phase, 3 + phase, y);
  }
  c.y_ac.setFromTriplets(trips.begin(), trips.end());
  c.i_ac_fixed = Eigen::VectorXcd::Zero(6);
  c.p_load_pu = Eigen::VectorXd::Zero(6);
  c.q_load_pu = Eigen::VectorXd::Zero(6);
  c.p_load_pu.segment<3>(3) << 0.18, 0.12, 0.08;
  c.q_load_pu.segment<3>(3) << 0.06, 0.04, 0.025;
  const double angle[3] = {0.0, -2.0 * M_PI / 3.0, 2.0 * M_PI / 3.0};
  c.voltage_start.resize(6);
  c.reference_voltage.resize(3);
  for (int phase = 0; phase < 3; ++phase) {
    c.reference_voltage[phase] = std::polar(1.0, angle[phase]);
    c.voltage_start[phase] = c.reference_voltage[phase];
    c.voltage_start[3 + phase] = std::polar(0.98, angle[phase]);
  }
  c.ac_phase_index = {0, 1, 2, 0, 1, 2};
  c.reference_nodes = {0, 1, 2};

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

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path output = argc > 1
      ? std::filesystem::path(argv[1])
      : std::filesystem::path("output/benchmarks/paper_hybrid_pf_external_snapshot.csv");
  std::filesystem::create_directories(output.parent_path());
  std::ofstream out(output);
  if (!out) {
    std::cerr << "cannot open " << output << '\n';
    return 2;
  }
  out << "mode,phase,native_converged,native_iterations,native_residual,"
         "voltage_re_pu,voltage_im_pu,net_p_pu,net_q_pu,converter_p_pu,"
         "converter_q_pu,vdc_terminal_pu,vdc_load_pu,base_mva,base_kv_ll,"
         "line_r_pu,line_x_pu\n";
  out << std::setprecision(15);
  const std::pair<const char*, ThreePhaseHybridPFControlMode> modes[] = {
      {"gfl_vdc_q", ThreePhaseHybridPFControlMode::GridFollowingVdcQ},
      {"gfm_vdc_e", ThreePhaseHybridPFControlMode::GridFormingVdc}};
  bool all_converged = true;
  for (const auto& [label, mode] : modes) {
    const auto c = make_case(mode);
    ThreePhaseHybridPFOptions options;
    options.tolerance = 1e-11;
    const auto result = hacdcpf::powerflow::solve_three_phase_hybrid_pf(c, options);
    all_converged = all_converged && result.converged;
    if (!result.converged || result.converters.empty()) {
      std::cerr << label << ": " << result.status << '\n';
      continue;
    }
    for (int phase = 0; phase < 3; ++phase) {
      const Complex converter_power = result.converters.front()
                                          .phase_power_pu[static_cast<std::size_t>(phase)];
      const Complex net_load{
          c.p_load_pu[3 + phase] - std::real(converter_power),
          c.q_load_pu[3 + phase] - std::imag(converter_power)};
      out << label << ',' << phase << ",yes," << result.iterations << ','
          << result.residual << ',' << std::real(result.voltage[3 + phase]) << ','
          << std::imag(result.voltage[3 + phase]) << ',' << std::real(net_load)
          << ',' << std::imag(net_load) << ',' << std::real(converter_power)
          << ',' << std::imag(converter_power) << ',' << result.dc_voltage[0]
          << ',' << result.dc_voltage[1] << ',' << c.base_mva << ",12.47,0.025,0.08\n";
    }
    std::cout << label << ": iterations=" << result.iterations
              << ", residual=" << result.residual << '\n';
  }
  std::cout << "Wrote " << output << '\n';
  return all_converged ? 0 : 1;
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace {

double balanced_ac_load_mw(const hacdcpf::HybridPowerSystem& system) {
  double total = 0.0;
  for (const auto& load : system.ac.loads) {
    if (load.in_service) total += load.p_mw * load.scaling;
  }
  return total;
}

double phase_ac_load_mw(const hacdcpf::ThreePhaseACSystem& system) {
  double total = 0.0;
  for (const auto& load : system.loads) {
    if (load.in_service) {
      total += load.p_a_mw + load.p_b_mw + load.p_c_mw;
    }
  }
  return total;
}

}  // namespace

TEST_CASE("urban LVN primary-secondary benchmark has stable hierarchy",
          "[case_builder][urban_lvn][structure]") {
  const auto system = hacdcpf::io::build_urban_lvn_primary_secondary();

  CHECK(system.name == "Urban LVNTS Primary-Secondary AC/DC Benchmark");
  CHECK(system.ac.buses.size() == 338);
  CHECK(system.ac.branches.size() == 337);
  CHECK(system.ac.switches.size() == 4);
  CHECK(system.ac.loads.size() == 312);
  CHECK(system.ac.static_generators.size() == 12);
  CHECK(system.ac.storage.size() == 4);
  CHECK(system.dc.buses.size() == 3);
  CHECK(system.dc.branches.size() == 2);
  CHECK(system.dc.loads.size() == 2);
  CHECK(system.vsc_converters.size() == 1);

  REQUIRE(system.three_phase_ac.has_value());
  const auto& phase = *system.three_phase_ac;
  CHECK(phase.buses.size() == 338);
  CHECK(phase.lines.size() == 312);
  CHECK(phase.transformers.size() == 25);
  CHECK(phase.loads.size() == 312);
  CHECK(phase.generators.size() == 12);
  CHECK(phase.external_grids.size() == 1);

  CHECK(std::abs(balanced_ac_load_mw(system) - phase_ac_load_mw(phase)) <
        1e-10);
  const auto diagnostics = hacdcpf::validate_component_parameters(system);
  INFO("parameter diagnostics: errors=" << diagnostics.error_count()
                                         << " warnings="
                                         << diagnostics.warning_count());
  CHECK(diagnostics.ok());
}

TEST_CASE("urban LVN balanced hybrid power flow converges",
          "[case_builder][urban_lvn][power_flow][performance]") {
  const auto system = hacdcpf::io::build_urban_lvn_primary_secondary();
  hacdcpf::PowerFlowOptions options;
  options.max_iter = 100;
  options.tol = 1e-7;
  options.enable_converter_coordination_check = false;

  const auto started = std::chrono::steady_clock::now();
  const auto result = hacdcpf::solve_power_flow(system, options);
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started);

  INFO("balanced solve seconds=" << elapsed.count()
                                  << " iterations=" << result.iterations
                                  << " residual=" << result.residual);
  REQUIRE(result.converged);
  CHECK(result.iterations <= options.max_iter);
  CHECK(result.residual < options.tol * 10.0);
  REQUIRE(result.vm.size() == system.ac.buses.size());
  REQUIRE(result.vdc.size() == system.dc.buses.size());
  REQUIRE(result.vsc_transfers.size() == 1);
  const auto [minimum, maximum] =
      std::minmax_element(result.vm.begin(), result.vm.end());
  const auto [minimum_dc, maximum_dc] =
      std::minmax_element(result.vdc.begin(), result.vdc.end());
  CHECK(*minimum > 0.75);
  CHECK(*maximum < 1.15);
  CHECK(*minimum_dc > 0.90);
  CHECK(*maximum_dc < 1.10);

  const auto& transfer = result.vsc_transfers.front();
  CHECK(std::abs(transfer.p_ac_mw) > 0.20);
  CHECK(std::abs(transfer.p_dc_mw) > 0.20);
  CHECK(std::abs(transfer.p_ac_mw + transfer.p_dc_mw + transfer.loss_mw) <
        1e-8);
}

TEST_CASE("urban LVN compact phase-domain power flow converges",
          "[case_builder][urban_lvn][three_phase][performance]") {
  const auto system = hacdcpf::io::build_urban_lvn_primary_secondary();
  hacdcpf::analysis::RunPFPhaseOptions options;
  options.algorithm = hacdcpf::analysis::PhaseDomainSolverAlgorithm::Compact;
  options.max_iter = 200;
  options.max_control_iter = 50;
  options.tol = 1e-6;
  options.vmin_pu = 0.80;

  const auto started = std::chrono::steady_clock::now();
  const auto result = hacdcpf::analysis::runpf_phase(system, options);
  const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started);

  INFO("phase solve seconds=" << elapsed.count()
                               << " iterations=" << result.iterations
                               << " residual=" << result.residual);
  REQUIRE(result.success);
  CHECK(result.iterations <= options.max_iter);
  CHECK(result.residual < options.tol * 10.0);
  REQUIRE(result.bus_abc.size() == system.three_phase_ac->buses.size());

  double minimum_voltage = std::numeric_limits<double>::infinity();
  double maximum_voltage = 0.0;
  for (const auto& bus : result.bus_abc) {
    for (int phase = 0; phase < 3; ++phase) {
      if (!bus.has_phase[static_cast<std::size_t>(phase)]) continue;
      minimum_voltage = std::min(
          minimum_voltage, bus.vm_pu[static_cast<std::size_t>(phase)]);
      maximum_voltage = std::max(
          maximum_voltage, bus.vm_pu[static_cast<std::size_t>(phase)]);
    }
  }
  CHECK(minimum_voltage > 0.70);
  CHECK(maximum_voltage < 1.15);
}

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

#ifndef _WIN32
#include <unistd.h>
#endif

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/gridlabd_bridge.hpp"
#include "hacdcpf/io/gridlabd_validation.hpp"

namespace {

namespace fs = std::filesystem;

fs::path temp_dir(const std::string& name) {
  long pid = 0;
#ifndef _WIN32
  pid = static_cast<long>(::getpid());
#endif
  const auto dir = fs::temp_directory_path() / ("hacdcpf_" + name + "_" +
                                                std::to_string(pid));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

hacdcpf::HybridPowerSystem component_case_named(const std::string& name) {
  hacdcpf::io::GridLABDValidationMatrixOptions options;
  options.include_component_cases = true;
  options.include_distribution_cases = false;
  options.include_transmission_cases = false;
  const auto cases = hacdcpf::io::build_gridlabd_standard_cases(options);
  const auto it = std::find_if(
      cases.begin(), cases.end(),
      [&](const hacdcpf::io::GridLABDStandardCase& c) {
        return c.name == name;
      });
  REQUIRE(it != cases.end());
  return it->system;
}

}  // namespace

TEST_CASE("GridLAB-D bridge exports bus shunts as voltage-dependent impedance",
          "[gridlabd][validation][shunt]") {
  auto sys = component_case_named("two_bus_component");
  REQUIRE(sys.ac.buses.size() >= 2);
  sys.ac.buses[1].pd_mw = 0.0;
  sys.ac.buses[1].qd_mvar = 0.0;
  sys.ac.buses[1].gs_mw = 0.08;
  sys.ac.buses[1].bs_mvar = 0.04;

  hacdcpf::io::GridLABDExportOptions options;
  options.model_name = "shunt_impedance_export";
  options.include_shunt_admittance_as_impedance = true;
  const auto snapshot = hacdcpf::io::export_gridlabd_snapshot(
      sys, temp_dir("gridlabd_shunt_impedance"), options);
  const std::string glm = read_file(snapshot.glm_path);
  CHECK(glm.find("constant_impedance_A") != std::string::npos);
  CHECK(glm.find("constant_impedance_B") != std::string::npos);
  CHECK(glm.find("constant_impedance_C") != std::string::npos);
  CHECK(glm.find("constant_power_A") == std::string::npos);
}

TEST_CASE("GridLAB-D failure matrix builds comparable component scenarios",
          "[gridlabd][validation][matrix]") {
  hacdcpf::io::GridLABDValidationMatrixOptions options;
  options.include_component_cases = true;
  options.include_distribution_cases = false;
  options.include_transmission_cases = false;
  options.include_diagnostic_islanding_branch_trips = false;
  options.max_branch_trip_scenarios_per_case = 2;
  options.max_branch_close_scenarios_per_case = 2;
  options.max_load_scale_scenarios_per_case = 2;
  options.max_fault_shunt_scenarios_per_case = 1;

  const auto cases = hacdcpf::io::build_gridlabd_standard_cases(options);
  REQUIRE(cases.size() >= 4);
  const auto scenarios =
      hacdcpf::io::build_gridlabd_failure_scenarios(
          component_case_named("three_bus_open_tie"), options);

  auto has_kind = [&](hacdcpf::io::GridLABDFailureKind kind) {
    return std::any_of(scenarios.begin(), scenarios.end(),
                       [&](const hacdcpf::io::GridLABDFailureScenario& s) {
                         return s.kind == kind;
                       });
  };
  CHECK(has_kind(hacdcpf::io::GridLABDFailureKind::BaseCase));
  CHECK(has_kind(hacdcpf::io::GridLABDFailureKind::ACBranchClose));
  CHECK(has_kind(hacdcpf::io::GridLABDFailureKind::ACLoadScale));
  CHECK(has_kind(hacdcpf::io::GridLABDFailureKind::FaultShunt));
}

TEST_CASE("GridLAB-D external failure matrix gates exact component snapshots",
          "[gridlabd][validation][external]") {
  const auto gridlabd = hacdcpf::io::discover_gridlabd();
  if (!gridlabd.available) {
    SKIP("GridLAB-D executable not available for failure matrix validation");
  }

  hacdcpf::io::GridLABDValidationMatrixOptions options;
  options.include_component_cases = true;
  options.include_distribution_cases = false;
  options.include_transmission_cases = false;
  options.include_diagnostic_islanding_branch_trips = false;
  options.include_diagnostic_native_failures = false;
  options.require_gridlabd = true;
  options.max_branch_trip_scenarios_per_case = 1;
  options.max_branch_close_scenarios_per_case = 1;
  options.max_load_scale_scenarios_per_case = 1;
  options.max_generator_trip_scenarios_per_case = 1;
  options.max_fault_shunt_scenarios_per_case = 1;
  options.comparison_options.vm_tolerance_pu = 2e-2;
  options.comparison_options.va_tolerance_deg = 2.0;
  options.comparison_options.branch_p_tolerance_mw = 0.10;
  options.comparison_options.branch_q_tolerance_mvar = 0.10;
  options.comparison_options.run_options.timeout_seconds = 120;

  const auto report = hacdcpf::io::run_gridlabd_standard_validation_matrix(options);
  INFO(hacdcpf::io::gridlabd_validation_matrix_to_json(report, 2));
  CHECK(report.gridlabd_available);
  CHECK(report.exact_gate_count >= 8);
  CHECK(report.exact_gate_passed);
  CHECK(report.exact_gate_passed_count == report.exact_gate_count);
}

TEST_CASE("GridLAB-D long-duration sequence compares pre/post-failure samples",
          "[gridlabd][validation][long-duration]") {
  const auto gridlabd = hacdcpf::io::discover_gridlabd();
  if (!gridlabd.available) {
    SKIP("GridLAB-D executable not available for long-duration validation");
  }

  hacdcpf::io::GridLABDValidationMatrixOptions case_options;
  case_options.include_component_cases = true;
  case_options.include_distribution_cases = false;
  case_options.include_transmission_cases = false;
  const auto cases = hacdcpf::io::build_gridlabd_standard_cases(case_options);
  const auto it = std::find_if(
      cases.begin(), cases.end(),
      [](const hacdcpf::io::GridLABDStandardCase& c) {
        return c.name == "three_bus_meshed";
      });
  REQUIRE(it != cases.end());

  hacdcpf::io::GridLABDLongDurationOptions options;
  options.require_gridlabd = true;
  options.t_end_s = 10.0;
  options.sample_interval_s = 2.0;
  options.dynamic_vm_tolerance_pu = 0.10;
  options.dynamic_options.dt_s = 0.02;
  options.dynamic_options.use_adaptive_step = true;
  options.dynamic_options.abs_tol = 1e-7;
  options.dynamic_options.rel_tol = 1e-5;
  options.dynamic_options.max_step_halving = 12;
  options.comparison_options.vm_tolerance_pu = 2e-2;
  options.comparison_options.va_tolerance_deg = 2.0;
  options.comparison_options.branch_p_tolerance_mw = 0.10;
  options.comparison_options.branch_q_tolerance_mvar = 0.10;
  options.comparison_options.run_options.timeout_seconds = 120;

  const auto events =
      hacdcpf::io::build_gridlabd_long_duration_sequence(it->system, options);
  REQUIRE(events.size() >= 3);

  const auto report =
      hacdcpf::io::run_gridlabd_long_duration_validation(*it, events, options);
  INFO(hacdcpf::io::gridlabd_long_duration_report_to_json(report, 2));
  CHECK(report.gridlabd_available);
  CHECK(report.dynamic_success);
  CHECK(report.exact_gate_count >= 4);
  CHECK(report.exact_gate_passed);
  CHECK(report.exact_gate_passed_count == report.exact_gate_count);
}

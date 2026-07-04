#pragma once

#include <string>
#include <vector>

#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/io/gridlabd_bridge.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

enum class GridLABDFailureKind {
  BaseCase,
  ACBranchTrip,
  ACBranchClose,
  ACLoadScale,
  GeneratorTrip,
  StaticGeneratorTrip,
  RenewableTrip,
  PVTrip,
  StorageTrip,
  FaultShunt,
  DCBranchTrip,
  DCBranchClose,
  DCLoadScale,
  DCDCTrip,
  VSCTrip,
  DCStorageTrip
};

struct GridLABDStandardCase {
  std::string name;
  std::string category;
  std::string source;
  HybridPowerSystem system;
  bool large{false};
};

struct GridLABDFailureScenario {
  std::string name;
  GridLABDFailureKind kind{GridLABDFailureKind::BaseCase};
  int component_index{0};
  int bus{0};
  double value{0.0};
  double r_pu{0.0};
  double x_pu{0.0};
  double scale{1.0};
  bool expected_exact_ac_snapshot{true};
  std::string scope;
  std::vector<std::string> notes;
};

struct GridLABDValidationMatrixOptions {
  GridLABDComparisonOptions comparison_options;
  bool include_base_case{true};
  bool include_component_cases{true};
  bool include_distribution_cases{true};
  bool include_transmission_cases{true};
  bool include_large_cases{false};
  bool include_diagnostic_native_failures{true};
  bool include_diagnostic_islanding_branch_trips{true};
  bool include_fault_shunts{true};
  int max_branch_trip_scenarios_per_case{2};
  int max_branch_close_scenarios_per_case{2};
  int max_load_scale_scenarios_per_case{2};
  int max_generator_trip_scenarios_per_case{2};
  int max_fault_shunt_scenarios_per_case{1};
  double load_step_up_scale{1.20};
  double load_step_down_scale{0.80};
  double fault_r_pu{50.0};
  double fault_x_pu{0.0};
  bool require_gridlabd{false};
};

struct GridLABDScenarioApplication {
  HybridPowerSystem system;
  bool applied{false};
  bool comparable_with_gridlabd{true};
  bool exact_ac_snapshot{true};
  std::vector<std::string> warnings;
};

struct GridLABDValidationScenarioReport {
  std::string case_name;
  std::string case_category;
  GridLABDFailureScenario scenario;
  GridLABDScenarioApplication application;
  GridLABDComparisonReport comparison;
  bool gridlabd_attempted{false};
  bool gridlabd_solved{false};
  bool exact_gate_item{false};
  bool passed{false};
  std::vector<std::string> warnings;
};

struct GridLABDValidationMatrixReport {
  bool gridlabd_available{false};
  bool exact_gate_passed{false};
  int case_count{0};
  int scenario_count{0};
  int exact_gate_count{0};
  int exact_gate_passed_count{0};
  int diagnostic_count{0};
  int skipped_count{0};
  std::vector<GridLABDValidationScenarioReport> scenarios;
  std::vector<std::string> warnings;
};

struct GridLABDLongDurationEvent {
  double time_s{0.0};
  double duration_s{0.0};
  GridLABDFailureScenario scenario;
  std::string label;
};

struct GridLABDLongDurationOptions {
  GridLABDComparisonOptions comparison_options;
  dynamics::DynamicSolverOptions dynamic_options;
  double t_end_s{10.0};
  double sample_interval_s{1.0};
  double event_probe_offset_s{1e-4};
  double dynamic_vm_tolerance_pu{8e-2};
  bool include_event_probe_samples{true};
  bool compare_active_fault_samples{false};
  bool require_gridlabd{false};
};

struct GridLABDLongDurationSampleReport {
  double time_s{0.0};
  std::string stage;
  std::vector<std::string> active_events;
  bool dynamic_snapshot_found{false};
  double dynamic_min_ac_voltage_pu{0.0};
  double dynamic_max_ac_voltage_pu{0.0};
  GridLABDScenarioApplication application;
  GridLABDComparisonReport comparison;
  std::vector<GridLABDComparisonItem> dynamic_voltage_items;
  bool gridlabd_attempted{false};
  bool gridlabd_solved{false};
  bool exact_gate_item{false};
  bool passed{false};
  std::vector<std::string> warnings;
};

struct GridLABDLongDurationReport {
  bool gridlabd_available{false};
  bool dynamic_success{false};
  std::string dynamic_message;
  int dynamic_steps{0};
  int dynamic_rejected_steps{0};
  double dynamic_max_local_error_norm{0.0};
  bool exact_gate_passed{false};
  int sample_count{0};
  int exact_gate_count{0};
  int exact_gate_passed_count{0};
  int diagnostic_count{0};
  int skipped_count{0};
  std::vector<GridLABDLongDurationSampleReport> samples;
  std::vector<std::string> warnings;
};

std::string gridlabd_failure_kind_name(GridLABDFailureKind kind);

std::vector<GridLABDStandardCase> build_gridlabd_standard_cases(
    const GridLABDValidationMatrixOptions& options = {});

std::vector<GridLABDFailureScenario> build_gridlabd_failure_scenarios(
    const HybridPowerSystem& sys,
    const GridLABDValidationMatrixOptions& options = {});

GridLABDScenarioApplication apply_gridlabd_failure_scenario(
    const HybridPowerSystem& sys,
    const GridLABDFailureScenario& scenario);

GridLABDValidationMatrixReport run_gridlabd_validation_matrix(
    const std::vector<GridLABDStandardCase>& cases,
    const GridLABDValidationMatrixOptions& options = {});

GridLABDValidationMatrixReport run_gridlabd_standard_validation_matrix(
    const GridLABDValidationMatrixOptions& options = {});

std::string gridlabd_validation_matrix_to_json(
    const GridLABDValidationMatrixReport& report,
    int indent = 2);

std::vector<GridLABDLongDurationEvent> build_gridlabd_long_duration_sequence(
    const HybridPowerSystem& sys,
    const GridLABDLongDurationOptions& options = {});

GridLABDLongDurationReport run_gridlabd_long_duration_validation(
    const GridLABDStandardCase& test_case,
    const std::vector<GridLABDLongDurationEvent>& events,
    const GridLABDLongDurationOptions& options = {});

std::string gridlabd_long_duration_report_to_json(
    const GridLABDLongDurationReport& report,
    int indent = 2);

}  // namespace hacdcpf::io

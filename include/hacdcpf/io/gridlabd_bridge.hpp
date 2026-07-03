#pragma once

#include <complex>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::io {

struct GridLABDExecutable {
  bool available{false};
  std::filesystem::path path;
  std::vector<std::string> search_paths;
  std::vector<std::string> diagnostics;
};

struct GridLABDExportOptions {
  bool project_to_canonical{true};
  bool ac_scope_only{true};
  bool include_load_recorders{true};
  bool include_branch_recorders{true};
  bool include_metadata_comments{true};
  bool include_line_capacitance{true};
  bool include_converter_boundary_injections{false};
  bool merge_parallel_branches{true};
  int gridlabd_iteration_limit{100000};
  int gridlabd_nr_iteration_limit{1000};
  double minimum_base_kv{0.12};
  double nominal_frequency_hz{50.0};
  std::string model_name{"hacdcpf_gridlabd_snapshot"};
};

struct GridLABDRunOptions {
  std::optional<std::filesystem::path> executable;
  int timeout_seconds{120};
  bool keep_working_files{true};
};

struct GridLABDComparisonOptions {
  GridLABDExportOptions export_options;
  GridLABDRunOptions run_options;
  double vm_tolerance_pu{5e-3};
  double va_tolerance_deg{0.5};
  double branch_p_tolerance_mw{2e-2};
  double branch_q_tolerance_mvar{2e-2};
  bool compare_bus_voltages{true};
  bool compare_branch_flows{true};
  bool compare_transformer_branch_flows{false};
  bool require_gridlabd{false};
};

struct GridLABDBusMapping {
  int canonical_bus_index{0};
  int canonical_bus_position{0};
  std::string source_name;
  std::string gridlabd_name;
  double base_kv_ll{0.0};
  double nominal_voltage_ln_volts{0.0};
  bool is_swing{false};
};

struct GridLABDBranchMapping {
  int canonical_branch_index{0};
  int canonical_branch_position{0};
  std::string canonical_name;
  std::string origin_type{"ACBranch"};
  int origin_index{0};
  int from_bus{0};
  int to_bus{0};
  std::string gridlabd_name;
  std::string gridlabd_configuration_name;
  bool exported{false};
  bool exported_as_transformer{false};
  bool exported_as_parallel_equivalent{false};
  int merged_into_canonical_branch_index{0};
  std::vector<int> merged_canonical_branch_indices;
  std::string skip_reason;
};

struct GridLABDLoadMapping {
  int canonical_bus_index{0};
  std::string gridlabd_name;
  double p_mw{0.0};
  double q_mvar{0.0};
};

struct GridLABDExportedSnapshot {
  std::filesystem::path working_directory;
  std::filesystem::path glm_path;
  HybridPowerSystem canonical_system;
  std::vector<GridLABDBusMapping> bus_mappings;
  std::vector<GridLABDBranchMapping> branch_mappings;
  std::vector<GridLABDLoadMapping> load_mappings;
  std::vector<std::string> warnings;
};

struct GridLABDBusVoltage {
  int canonical_bus_index{0};
  std::string gridlabd_name;
  std::complex<double> voltage_a_v;
  std::complex<double> voltage_b_v;
  std::complex<double> voltage_c_v;
  double vm_a_pu{0.0};
  double va_a_deg{0.0};
};

struct GridLABDBranchPower {
  int canonical_branch_index{0};
  std::string gridlabd_name;
  std::complex<double> power_in_va;
  std::complex<double> power_out_va;
  std::complex<double> power_loss_va;
};

struct GridLABDRunResult {
  bool attempted{false};
  bool success{false};
  int exit_code{-1};
  std::filesystem::path executable;
  std::string command;
  std::string stdout_text;
  std::string stderr_text;
  std::vector<GridLABDBusVoltage> bus_voltages;
  std::vector<GridLABDBranchPower> branch_powers;
  std::vector<std::string> warnings;
};

struct GridLABDComparisonItem {
  std::string kind;
  std::string key;
  double hacdcpf_value{0.0};
  double gridlabd_value{0.0};
  double difference{0.0};
  double tolerance{0.0};
  bool passed{true};
  std::string detail;
};

struct GridLABDComparisonReport {
  bool gridlabd_available{false};
  bool gridlabd_run_attempted{false};
  bool gridlabd_run_success{false};
  bool hacdcpf_power_flow_converged{false};
  bool numerical_comparison_passed{false};
  bool equivalence_passed{false};
  bool passed{false};
  std::string equivalence_scope;
  std::string equivalence_claim;
  PowerFlowResult hacdcpf_power_flow;
  GridLABDExportedSnapshot exported_snapshot;
  GridLABDRunResult gridlabd_result;
  std::vector<GridLABDComparisonItem> items;
  std::vector<std::string> unsupported_features;
  std::vector<std::string> diagnostic_only_reasons;
  std::vector<std::string> warnings;
  std::vector<std::string> skipped;
};

GridLABDExecutable discover_gridlabd();

GridLABDExportedSnapshot export_gridlabd_snapshot(
    const HybridPowerSystem& sys,
    const std::filesystem::path& working_directory,
    const GridLABDExportOptions& options = {});

GridLABDRunResult run_gridlabd_snapshot(
    const GridLABDExportedSnapshot& snapshot,
    const GridLABDRunOptions& options = {});

GridLABDComparisonReport compare_gridlabd_snapshot(
    const HybridPowerSystem& sys,
    const GridLABDComparisonOptions& options = {});

}  // namespace hacdcpf::io

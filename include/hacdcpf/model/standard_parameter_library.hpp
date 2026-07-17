#pragma once

/// Editable engineering parameter profiles and model-quality diagnostics.
///
/// Importers remain source-faithful. Call validation after import to detect
/// unusable data, and apply_standard_parameter_library() only when the caller
/// explicitly wants missing values filled from the selected profile.

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf {

enum class ParameterIssueSeverity { Warning, Error };

const char* to_string(ParameterIssueSeverity severity);
ParameterIssueSeverity parameter_issue_severity_from_string(
    const std::string& value);

struct StandardParameterRule {
  std::string id;
  std::string component_type;
  std::string parameter;
  std::string label;
  double default_value{0.0};
  std::string unit;
  bool has_min{false};
  double min_value{0.0};
  bool has_max{false};
  double max_value{0.0};
  ParameterIssueSeverity severity{ParameterIssueSeverity::Warning};
  std::string source;
  std::string description;
  bool editable{true};
};

struct StandardParameterLibrary {
  std::string profile_id;
  std::string name;
  std::string version;
  std::string description;
  std::vector<StandardParameterRule> rules;

  const StandardParameterRule* find(const std::string& id) const;
  StandardParameterRule* find(const std::string& id);
};

struct StandardParameterProfile {
  std::string id;
  std::string name;
  std::string description;
};

struct ParameterDiagnostic {
  std::string code;
  ParameterIssueSeverity severity{ParameterIssueSeverity::Warning};
  std::string component_type;
  int component_index{0};
  std::string component_name;
  std::string parameter;
  double value{0.0};
  std::string message;
};

struct ParameterValidationReport {
  std::vector<ParameterDiagnostic> diagnostics;

  int error_count() const;
  int warning_count() const;
  bool ok() const { return error_count() == 0; }
};

struct StandardParameterApplyReport {
  int fields_changed{0};
  std::vector<std::string> applied_rule_ids;
};

/// Opt-in completion of physical AC-line parameters from published handbook
/// rules. Authored/manufacturer impedance is preserved unless it was previously
/// marked as an importer estimate.
struct DesignHandbookCompletionOptions {
  bool apply{false};
  bool overwrite_import_estimates{true};
  bool infer_switch_bindings{true};
  bool overwrite_inferred_switch_bindings{true};
  double default_lv_overhead_cross_section_mm2{35.0};
  double default_lv_cable_cross_section_mm2{50.0};
  double default_mv_cross_section_mm2{300.0};
  double cable_reactance_ohm_per_km{0.08};
  double overhead_reactance_ohm_per_km{0.35};
};

struct DesignHandbookSwitchBindingSuggestion {
  int switch_index{0};
  std::string switch_name;
  std::string switch_type;
  std::string old_role;
  std::string new_role;
  std::string controlled_element_type;
  int controlled_element_index{-1};
  int controlled_branch_index{-1};
  int protection_zone_id{0};
  int upstream_protective_switch_index{-1};
  std::string confidence;
  std::string action;
  std::string rationale;
  std::string source;
  bool ambiguous{false};
  bool applied{false};
};

struct DesignHandbookLineSuggestion {
  int branch_index{0};
  std::string branch_name;
  std::string conductor_model;
  std::string line_type;
  std::string conductor_material;
  std::string insulation;
  std::string confidence;
  std::string action;
  std::string source;
  double base_kv{0.0};
  double length_km{0.0};
  double cross_section_mm2{0.0};
  double conductor_temperature_c{0.0};
  double r20_ohm_per_km{0.0};
  double old_r_ohm_per_km{0.0};
  double old_x_ohm_per_km{0.0};
  double new_r_ohm_per_km{0.0};
  double new_x_ohm_per_km{0.0};
  double old_r_pu{0.0};
  double old_x_pu{0.0};
  double new_r_pu{0.0};
  double new_x_pu{0.0};
  double old_rate_a_mva{0.0};
  double new_rate_a_mva{0.0};
  bool cross_section_inferred{false};
  bool model_type_conflict{false};
  bool applied{false};
};

struct DesignHandbookCompletionReport {
  int branches_scanned{0};
  int candidates{0};
  int fields_changed{0};
  int skipped_authored{0};
  int skipped_missing_geometry{0};
  int switches_scanned{0};
  int switch_binding_candidates{0};
  int switch_binding_fields_changed{0};
  int skipped_ambiguous_bindings{0};
  std::vector<DesignHandbookLineSuggestion> suggestions;
  std::vector<DesignHandbookSwitchBindingSuggestion> switch_bindings;
  std::vector<std::string> warnings;
};

std::vector<StandardParameterProfile> standard_parameter_profiles();

StandardParameterLibrary make_standard_parameter_library(
    const std::string& profile_id = "distribution_50hz");

ParameterValidationReport validate_standard_parameter_library(
    const StandardParameterLibrary& library);

ParameterValidationReport validate_component_parameters(
    const HybridPowerSystem& system,
    const StandardParameterLibrary& library =
        make_standard_parameter_library());

StandardParameterApplyReport apply_standard_parameter_library(
    HybridPowerSystem& system,
    const StandardParameterLibrary& library =
        make_standard_parameter_library());

DesignHandbookCompletionReport complete_design_handbook_parameters(
    HybridPowerSystem& system,
    const DesignHandbookCompletionOptions& options = {});

}  // namespace hacdcpf

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

}  // namespace hacdcpf

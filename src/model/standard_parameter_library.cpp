#include "hacdcpf/model/standard_parameter_library.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace hacdcpf {
namespace {

constexpr double kMissing = 1e-12;

StandardParameterRule rule(std::string id,
                           std::string component_type,
                           std::string parameter,
                           std::string label,
                           double default_value,
                           std::string unit,
                           double min_value,
                           double max_value,
                           ParameterIssueSeverity severity,
                           std::string source,
                           std::string description) {
  StandardParameterRule out;
  out.id = std::move(id);
  out.component_type = std::move(component_type);
  out.parameter = std::move(parameter);
  out.label = std::move(label);
  out.default_value = default_value;
  out.unit = std::move(unit);
  out.has_min = true;
  out.min_value = min_value;
  out.has_max = true;
  out.max_value = max_value;
  out.severity = severity;
  out.source = std::move(source);
  out.description = std::move(description);
  return out;
}

double value_or(const StandardParameterLibrary& library,
                const std::string& id,
                double fallback) {
  const auto* item = library.find(id);
  return item != nullptr && std::isfinite(item->default_value)
             ? item->default_value
             : fallback;
}

bool missing_positive(double value) {
  return !std::isfinite(value) || value <= kMissing;
}

bool missing_impedance(double value) {
  return !std::isfinite(value) || std::abs(value) <= kMissing;
}

void note_change(StandardParameterApplyReport& report,
                 const std::string& rule_id) {
  ++report.fields_changed;
  if (std::find(report.applied_rule_ids.begin(), report.applied_rule_ids.end(),
                rule_id) == report.applied_rule_ids.end()) {
    report.applied_rule_ids.push_back(rule_id);
  }
}

void fill_positive(double& target,
                   double value,
                   const std::string& rule_id,
                   StandardParameterApplyReport& report) {
  if (!missing_positive(target) || !std::isfinite(value) || value <= kMissing) {
    return;
  }
  target = value;
  note_change(report, rule_id);
}

void fill_impedance(double& target,
                    double value,
                    const std::string& rule_id,
                    StandardParameterApplyReport& report) {
  if (!missing_impedance(target) || !std::isfinite(value) ||
      std::abs(value) <= kMissing) {
    return;
  }
  target = value;
  note_change(report, rule_id);
}

bool transformer_like(const ACBranch& branch) {
  return branch.sn_mva > kMissing || branch.vn_hv_kv > kMissing ||
         branch.vn_lv_kv > kMissing;
}

void add_issue(ParameterValidationReport& report,
               std::string code,
               ParameterIssueSeverity severity,
               std::string component_type,
               int component_index,
               std::string component_name,
               std::string parameter,
               double value,
               std::string message) {
  report.diagnostics.push_back(ParameterDiagnostic{
      std::move(code), severity, std::move(component_type), component_index,
      std::move(component_name), std::move(parameter), value,
      std::move(message)});
}

ParameterIssueSeverity severity_for(const StandardParameterLibrary& library,
                                    const std::string& id,
                                    ParameterIssueSeverity fallback) {
  const auto* item = library.find(id);
  return item == nullptr ? fallback : item->severity;
}

void check_range(ParameterValidationReport& report,
                 const StandardParameterLibrary& library,
                 const std::string& rule_id,
                 const std::string& component_type,
                 int index,
                 const std::string& name,
                 const std::string& parameter,
                 double value) {
  const auto* item = library.find(rule_id);
  if (item == nullptr) return;
  if (!std::isfinite(value)) {
    add_issue(report, "nonfinite_parameter", item->severity, component_type,
              index, name, parameter, value,
              component_type + " " + std::to_string(index) + " has non-finite " +
                  parameter + ".");
    return;
  }
  if ((item->has_min && value < item->min_value) ||
      (item->has_max && value > item->max_value)) {
    add_issue(report, "parameter_out_of_range", item->severity, component_type,
              index, name, parameter, value,
              component_type + " " + std::to_string(index) + " " + parameter +
                  " is outside the active parameter-library range.");
  }
}

}  // namespace

const char* to_string(ParameterIssueSeverity severity) {
  return severity == ParameterIssueSeverity::Error ? "error" : "warning";
}

ParameterIssueSeverity parameter_issue_severity_from_string(
    const std::string& value) {
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return lower == "error" ? ParameterIssueSeverity::Error
                          : ParameterIssueSeverity::Warning;
}

const StandardParameterRule* StandardParameterLibrary::find(
    const std::string& id) const {
  const auto it = std::find_if(rules.begin(), rules.end(),
                               [&](const auto& item) { return item.id == id; });
  return it == rules.end() ? nullptr : &*it;
}

StandardParameterRule* StandardParameterLibrary::find(const std::string& id) {
  const auto it = std::find_if(rules.begin(), rules.end(),
                               [&](const auto& item) { return item.id == id; });
  return it == rules.end() ? nullptr : &*it;
}

int ParameterValidationReport::error_count() const {
  return static_cast<int>(std::count_if(
      diagnostics.begin(), diagnostics.end(), [](const auto& item) {
        return item.severity == ParameterIssueSeverity::Error;
      }));
}

int ParameterValidationReport::warning_count() const {
  return static_cast<int>(std::count_if(
      diagnostics.begin(), diagnostics.end(), [](const auto& item) {
        return item.severity == ParameterIssueSeverity::Warning;
      }));
}

std::vector<StandardParameterProfile> standard_parameter_profiles() {
  return {
      {"distribution_50hz", "Distribution engineering baseline (50 Hz)",
       "General AC/DC distribution defaults with conservative plausibility guards."},
      {"lv_distribution_50hz", "Low-voltage distribution baseline (50 Hz)",
       "Lower-voltage defaults for LV feeders and converter-rich microgrids."},
  };
}

StandardParameterLibrary make_standard_parameter_library(
    const std::string& profile_id) {
  const bool lv = profile_id == "lv_distribution_50hz";
  if (!lv && profile_id != "distribution_50hz") {
    throw std::invalid_argument("Unknown standard parameter profile: " + profile_id);
  }

  StandardParameterLibrary library;
  library.profile_id = profile_id;
  library.name = lv ? "Low-voltage distribution baseline (50 Hz)"
                    : "Distribution engineering baseline (50 Hz)";
  library.version = "1.0";
  library.description =
      "Editable engineering defaults and plausibility limits. Values are not a "
      "substitute for manufacturer nameplates or project-specific studies.";
  const std::string baseline = "HACDCPF engineering baseline; verify against nameplate";
  const std::string voltage_source =
      "IEC 60038 voltage context; project-specific nominal voltage required";

  library.rules = {
      rule("system.base_mva", "System", "base_mva", "Power base", 100.0,
           "MVA", 0.1, 10000.0, ParameterIssueSeverity::Error, baseline,
           "Positive system power base used for per-unit conversion."),
      rule("system.frequency_hz", "System", "frequency_hz", "Nominal frequency",
           50.0, "Hz", 45.0, 65.0, ParameterIssueSeverity::Warning, baseline,
           "Nominal network frequency."),
      rule("ac_bus.base_kv", "AC bus", "base_kv", "Nominal line voltage",
           lv ? 0.4 : 10.0, "kV", 0.1, 1200.0, ParameterIssueSeverity::Error,
           voltage_source, "Positive line-to-line voltage base."),
      rule("ac_bus.vm_pu", "AC bus", "vm_pu", "Initial voltage", 1.0, "pu",
           0.5, 1.5, ParameterIssueSeverity::Warning, baseline,
           "Initial positive-sequence voltage magnitude."),
      rule("ac_bus.vmin_pu", "AC bus", "vmin_pu", "Minimum voltage", 0.9, "pu",
           0.5, 1.1, ParameterIssueSeverity::Warning, baseline,
           "Operational lower voltage bound."),
      rule("ac_bus.vmax_pu", "AC bus", "vmax_pu", "Maximum voltage", 1.1, "pu",
           0.9, 1.5, ParameterIssueSeverity::Warning, baseline,
           "Operational upper voltage bound."),
      rule("ac_branch.r_pu", "AC branch", "r_pu", "Series resistance",
           lv ? 0.02 : 0.01, "pu", 0.0, 2.0,
           ParameterIssueSeverity::Error, baseline,
           "Positive-sequence resistance on the system base."),
      rule("ac_branch.x_pu", "AC branch", "x_pu", "Series reactance",
           lv ? 0.03 : 0.04, "pu", 0.0, 3.0,
           ParameterIssueSeverity::Error, baseline,
           "Positive-sequence reactance on the system base."),
      rule("ac_branch.rate_a_mva", "AC branch", "rate_a_mva", "Continuous rating",
           lv ? 1.0 : 100.0, "MVA", 0.001, 10000.0,
           ParameterIssueSeverity::Warning, baseline,
           "Continuous apparent-power rating."),
      rule("transformer.r_pu", "Transformer", "r_pu", "Series resistance", 0.006,
           "pu", 0.0, 0.2, ParameterIssueSeverity::Error, baseline,
           "Transformer resistance on the active model base."),
      rule("transformer.x_pu", "Transformer", "x_pu", "Series reactance", 0.06,
           "pu", 0.0, 0.5, ParameterIssueSeverity::Error, baseline,
           "Transformer reactance on the active model base."),
      rule("transformer.sn_mva", "Transformer", "sn_mva", "Rated power",
           lv ? 0.5 : 10.0, "MVA", 0.001, 10000.0,
           ParameterIssueSeverity::Error, baseline,
           "Transformer nameplate apparent power."),
      rule("transformer.vk_percent", "Transformer", "vk_percent",
           "Short-circuit voltage", 6.0, "%", 0.1, 30.0,
           ParameterIssueSeverity::Error, baseline,
           "Transformer short-circuit voltage."),
      rule("transformer.vkr_percent", "Transformer", "vkr_percent",
           "Resistive short-circuit voltage", 0.6, "%", 0.01, 10.0,
           ParameterIssueSeverity::Warning, baseline,
           "Resistive part of transformer short-circuit voltage."),
      rule("dc_bus.base_kv", "DC bus", "base_kv", "Nominal DC voltage",
           lv ? 0.75 : 1.5, "kV", 0.05, 1000.0,
           ParameterIssueSeverity::Error, baseline,
           "Positive pole-to-pole or declared DC voltage base."),
      rule("dc_branch.r_pu", "DC branch", "r_pu", "Series resistance", 0.01,
           "pu", 1e-8, 5.0, ParameterIssueSeverity::Error, baseline,
           "DC branch resistance on the system base."),
      rule("vsc.p_rated_mw", "VSC", "p_rated_mw", "Rated active power",
           lv ? 0.25 : 1.0, "MW", 0.001, 10000.0,
           ParameterIssueSeverity::Warning, baseline,
           "Converter active-power nameplate rating."),
      rule("vsc.eta", "VSC", "eta", "Efficiency", 0.99, "pu", 0.5, 1.0,
           ParameterIssueSeverity::Error, baseline,
           "Steady-state conversion efficiency."),
      rule("vsc.r_sc_pu", "VSC", "r_sc_pu", "Short-circuit resistance", 0.01,
           "pu", 1e-6, 1.0, ParameterIssueSeverity::Warning, baseline,
           "Converter short-circuit resistance."),
      rule("vsc.x_sc_pu", "VSC", "x_sc_pu", "Short-circuit reactance", 0.15,
           "pu", 1e-6, 2.0, ParameterIssueSeverity::Warning, baseline,
           "Converter short-circuit reactance."),
      rule("vsc.i_max_pu", "VSC", "i_max_pu", "Current limit", 1.2, "pu",
           0.1, 5.0, ParameterIssueSeverity::Warning, baseline,
           "Converter short-circuit or protection current limit."),
      rule("dcdc.eta", "DC-DC converter", "eta", "Efficiency", 0.98, "pu",
           0.5, 1.0, ParameterIssueSeverity::Error, baseline,
           "Steady-state DC conversion efficiency."),
      rule("storage.eta_charge", "Storage", "eta_charge", "Charge efficiency",
           0.95, "pu", 0.5, 1.0, ParameterIssueSeverity::Error, baseline,
           "One-way charging efficiency."),
      rule("storage.eta_discharge", "Storage", "eta_discharge",
           "Discharge efficiency", 0.95, "pu", 0.5, 1.0,
           ParameterIssueSeverity::Error, baseline,
           "One-way discharging efficiency."),
  };
  return library;
}

ParameterValidationReport validate_standard_parameter_library(
    const StandardParameterLibrary& library) {
  ParameterValidationReport report;
  std::unordered_set<std::string> ids;
  for (const auto& item : library.rules) {
    if (item.id.empty() || !ids.insert(item.id).second) {
      add_issue(report, "invalid_rule_id", ParameterIssueSeverity::Error,
                "Parameter library", 0, library.name, item.id,
                item.default_value, "Parameter rule IDs must be non-empty and unique.");
    }
    if (!std::isfinite(item.default_value) ||
        (item.has_min && item.default_value < item.min_value) ||
        (item.has_max && item.default_value > item.max_value) ||
        (item.has_min && item.has_max && item.min_value > item.max_value)) {
      add_issue(report, "invalid_rule_range", ParameterIssueSeverity::Error,
                "Parameter library", 0, library.name, item.id,
                item.default_value,
                "Default and bounds are inconsistent for rule " + item.id + ".");
    }
  }
  return report;
}

ParameterValidationReport validate_component_parameters(
    const HybridPowerSystem& system,
    const StandardParameterLibrary& library) {
  ParameterValidationReport report = validate_standard_parameter_library(library);
  if (!report.ok()) return report;

  check_range(report, library, "system.base_mva", "System", 0, system.name,
              "base_mva", system.base_mva);
  check_range(report, library, "system.frequency_hz", "System", 0, system.name,
              "frequency_hz", system.ac.freq_hz);

  for (const auto& bus : system.ac.buses) {
    if (!bus.in_service) continue;
    check_range(report, library, "ac_bus.base_kv", "AC bus", bus.index,
                bus.name, "base_kv", bus.base_kv);
    check_range(report, library, "ac_bus.vm_pu", "AC bus", bus.index, bus.name,
                "vm_pu", bus.vm_pu);
    if (!std::isfinite(bus.vmin_pu) || !std::isfinite(bus.vmax_pu) ||
        bus.vmin_pu >= bus.vmax_pu) {
      add_issue(report, "invalid_voltage_limits", ParameterIssueSeverity::Error,
                "AC bus", bus.index, bus.name, "vmin_pu/vmax_pu", bus.vmin_pu,
                "AC bus voltage limits must be finite and vmin_pu < vmax_pu.");
    }
  }

  for (const auto& branch : system.ac.branches) {
    if (!branch.in_service) continue;
    const bool is_transformer = transformer_like(branch);
    const double impedance = std::hypot(branch.r_pu, branch.x_pu);
    if (!std::isfinite(impedance) || impedance <= kMissing) {
      add_issue(
          report,
          is_transformer ? "transformer_zero_series_impedance"
                         : "branch_zero_series_impedance",
          ParameterIssueSeverity::Error,
          is_transformer ? "Transformer branch" : "AC branch", branch.index,
          branch.name, "r_pu/x_pu", impedance,
          (is_transformer ? "Transformer branch " : "AC branch ") +
              std::to_string(branch.index) +
              " has zero or invalid series impedance; the admittance matrix may be singular.");
    } else {
      check_range(report, library,
                  is_transformer ? "transformer.r_pu" : "ac_branch.r_pu",
                  is_transformer ? "Transformer branch" : "AC branch",
                  branch.index, branch.name, "r_pu", branch.r_pu);
      check_range(report, library,
                  is_transformer ? "transformer.x_pu" : "ac_branch.x_pu",
                  is_transformer ? "Transformer branch" : "AC branch",
                  branch.index, branch.name, "x_pu", branch.x_pu);
    }
    if (is_transformer && missing_positive(branch.sn_mva) &&
        missing_positive(branch.rate_a_mva)) {
      add_issue(report, "transformer_missing_rating",
                severity_for(library, "transformer.sn_mva",
                             ParameterIssueSeverity::Warning),
                "Transformer branch", branch.index, branch.name,
                "sn_mva/rate_a_mva", branch.sn_mva,
                "Transformer branch has no positive nameplate or continuous rating.");
    }
  }

  for (const auto& transformer : system.ac.transformers_2w) {
    if (!transformer.in_service) continue;
    check_range(report, library, "transformer.sn_mva", "Transformer",
                transformer.index, transformer.name, "sn_mva", transformer.sn_mva);
    check_range(report, library, "transformer.vk_percent", "Transformer",
                transformer.index, transformer.name, "vk_percent",
                transformer.vk_percent);
    check_range(report, library, "transformer.vkr_percent", "Transformer",
                transformer.index, transformer.name, "vkr_percent",
                transformer.vkr_percent);
    if (transformer.vkr_percent > transformer.vk_percent + kMissing) {
      add_issue(report, "transformer_resistance_exceeds_impedance",
                ParameterIssueSeverity::Error, "Transformer", transformer.index,
                transformer.name, "vkr_percent", transformer.vkr_percent,
                "Transformer vkr_percent cannot exceed vk_percent.");
    }
  }

  for (const auto& bus : system.dc.buses) {
    if (!bus.in_service) continue;
    check_range(report, library, "dc_bus.base_kv", "DC bus", bus.index,
                bus.name, "base_kv", bus.base_kv);
  }
  for (const auto& branch : system.dc.branches) {
    if (!branch.in_service) continue;
    check_range(report, library, "dc_branch.r_pu", "DC branch", branch.index,
                branch.name, "r_pu", branch.r_pu);
  }
  for (const auto& converter : system.vsc_converters) {
    if (!converter.in_service) continue;
    check_range(report, library, "vsc.eta", "VSC", converter.index,
                converter.name, "eta", converter.eta);
    if (missing_positive(converter.p_rated_mw) &&
        missing_positive(converter.pmax_mw)) {
      add_issue(report, "vsc_missing_rating",
                severity_for(library, "vsc.p_rated_mw",
                             ParameterIssueSeverity::Warning),
                "VSC", converter.index, converter.name, "p_rated_mw/pmax_mw",
                converter.p_rated_mw, "VSC has no positive active-power rating.");
    }
  }
  for (const auto& converter : system.dc.dcdc_converters) {
    if (!converter.in_service) continue;
    check_range(report, library, "dcdc.eta", "DC-DC converter",
                converter.index, converter.name, "eta", converter.eta);
  }
  auto validate_storage = [&](const auto& storage) {
    if (!storage.in_service) return;
    check_range(report, library, "storage.eta_charge", "Storage", storage.index,
                storage.name, "eta_charge", storage.eta_charge);
    check_range(report, library, "storage.eta_discharge", "Storage",
                storage.index, storage.name, "eta_discharge",
                storage.eta_discharge);
    if (!std::isfinite(storage.soc_min) || !std::isfinite(storage.soc_max) ||
        storage.soc_min < 0.0 || storage.soc_max > 1.0 ||
        storage.soc_min >= storage.soc_max) {
      add_issue(report, "invalid_storage_soc_limits",
                ParameterIssueSeverity::Error, "Storage", storage.index,
                storage.name, "soc_min/soc_max", storage.soc_min,
                "Storage SOC limits must satisfy 0 <= soc_min < soc_max <= 1.");
    }
  };
  for (const auto& storage : system.ac.storage) validate_storage(storage);
  for (const auto& storage : system.dc.storage) validate_storage(storage);
  for (const auto& storage : system.dc.dc_storage) validate_storage(storage);

  if (system.three_phase_ac.has_value()) {
    const auto& phase = *system.three_phase_ac;
    for (const auto& bus : phase.buses) {
      if (!bus.in_service) continue;
      check_range(report, library, "ac_bus.base_kv", "Three-phase AC bus",
                  bus.index, bus.name, "base_kv", bus.base_kv);
      if (!std::isfinite(bus.vmin_pu) || !std::isfinite(bus.vmax_pu) ||
          bus.vmin_pu >= bus.vmax_pu) {
        add_issue(report, "invalid_voltage_limits", ParameterIssueSeverity::Error,
                  "Three-phase AC bus", bus.index, bus.name,
                  "vmin_pu/vmax_pu", bus.vmin_pu,
                  "Three-phase AC bus voltage limits must be finite and "
                  "vmin_pu < vmax_pu.");
      }
    }
    for (const auto& line : phase.lines) {
      if (!line.in_service) continue;
      double impedance = std::hypot(line.r1_pu, line.x1_pu);
      if (line.use_phase_matrix) {
        for (std::size_t i = 0; i < line.r_matrix_pu.size(); ++i) {
          impedance = std::max(
              impedance,
              std::hypot(line.r_matrix_pu[i], line.x_matrix_pu[i]));
        }
      }
      if (!std::isfinite(impedance) || impedance <= kMissing) {
        add_issue(report, "three_phase_line_zero_series_impedance",
                  ParameterIssueSeverity::Error, "Three-phase AC line",
                  line.index, line.name, "r1_pu/x1_pu or phase matrix",
                  impedance,
                  "Three-phase AC line has zero or invalid series impedance.");
      } else if (!line.use_phase_matrix) {
        check_range(report, library, "ac_branch.r_pu", "Three-phase AC line",
                    line.index, line.name, "r1_pu", line.r1_pu);
        check_range(report, library, "ac_branch.x_pu", "Three-phase AC line",
                    line.index, line.name, "x1_pu", line.x1_pu);
      }
    }
    for (const auto& transformer : phase.transformers) {
      if (!transformer.in_service) continue;
      check_range(report, library, "transformer.sn_mva",
                  "Three-phase transformer", transformer.index,
                  transformer.name, "sn_mva", transformer.sn_mva);
      check_range(report, library, "transformer.vk_percent",
                  "Three-phase transformer", transformer.index,
                  transformer.name, "vk_percent", transformer.vk_percent);
      check_range(report, library, "transformer.vkr_percent",
                  "Three-phase transformer", transformer.index,
                  transformer.name, "vkr_percent", transformer.vkr_percent);
      if (transformer.vkr_percent > transformer.vk_percent + kMissing) {
        add_issue(report, "transformer_resistance_exceeds_impedance",
                  ParameterIssueSeverity::Error, "Three-phase transformer",
                  transformer.index, transformer.name, "vkr_percent",
                  transformer.vkr_percent,
                  "Three-phase transformer vkr_percent cannot exceed "
                  "vk_percent.");
      }
    }
  }

  return report;
}

StandardParameterApplyReport apply_standard_parameter_library(
    HybridPowerSystem& system,
    const StandardParameterLibrary& library) {
  const auto library_validation = validate_standard_parameter_library(library);
  if (!library_validation.ok()) {
    throw std::invalid_argument("Cannot apply an invalid standard parameter library.");
  }

  StandardParameterApplyReport report;
  fill_positive(system.base_mva, value_or(library, "system.base_mva", 100.0),
                "system.base_mva", report);
  fill_positive(system.ac.base_mva, system.base_mva, "system.base_mva", report);
  fill_positive(system.dc.base_mva, system.base_mva, "system.base_mva", report);
  fill_positive(system.ac.freq_hz,
                value_or(library, "system.frequency_hz", 50.0),
                "system.frequency_hz", report);

  for (auto& bus : system.ac.buses) {
    fill_positive(bus.base_kv, value_or(library, "ac_bus.base_kv", 10.0),
                  "ac_bus.base_kv", report);
    fill_positive(bus.vm_pu, value_or(library, "ac_bus.vm_pu", 1.0),
                  "ac_bus.vm_pu", report);
    fill_positive(bus.vmin_pu, value_or(library, "ac_bus.vmin_pu", 0.9),
                  "ac_bus.vmin_pu", report);
    fill_positive(bus.vmax_pu, value_or(library, "ac_bus.vmax_pu", 1.1),
                  "ac_bus.vmax_pu", report);
    if (bus.vmin_pu >= bus.vmax_pu) {
      bus.vmin_pu = value_or(library, "ac_bus.vmin_pu", 0.9);
      bus.vmax_pu = value_or(library, "ac_bus.vmax_pu", 1.1);
      note_change(report, "ac_bus.vmin_pu");
      note_change(report, "ac_bus.vmax_pu");
    }
  }

  for (auto& branch : system.ac.branches) {
    const bool is_transformer = transformer_like(branch);
    const std::string r_id =
        is_transformer ? "transformer.r_pu" : "ac_branch.r_pu";
    const std::string x_id =
        is_transformer ? "transformer.x_pu" : "ac_branch.x_pu";
    if (missing_impedance(branch.r_pu) && missing_impedance(branch.x_pu)) {
      fill_impedance(branch.r_pu, value_or(library, r_id, 0.01), r_id, report);
      fill_impedance(branch.x_pu, value_or(library, x_id, 0.04), x_id, report);
    }
    fill_positive(branch.rate_a_mva,
                  is_transformer
                      ? value_or(library, "transformer.sn_mva", 10.0)
                      : value_or(library, "ac_branch.rate_a_mva", system.ac.base_mva),
                  is_transformer ? "transformer.sn_mva"
                                 : "ac_branch.rate_a_mva",
                  report);
    if (is_transformer) {
      fill_positive(branch.sn_mva, branch.rate_a_mva, "transformer.sn_mva", report);
    }
  }

  for (auto& transformer : system.ac.transformers_2w) {
    fill_positive(transformer.sn_mva,
                  value_or(library, "transformer.sn_mva", 10.0),
                  "transformer.sn_mva", report);
    fill_positive(transformer.vk_percent,
                  value_or(library, "transformer.vk_percent", 6.0),
                  "transformer.vk_percent", report);
    fill_positive(transformer.vkr_percent,
                  value_or(library, "transformer.vkr_percent", 0.6),
                  "transformer.vkr_percent", report);
  }

  for (auto& bus : system.dc.buses) {
    fill_positive(bus.base_kv, value_or(library, "dc_bus.base_kv", 0.75),
                  "dc_bus.base_kv", report);
  }
  for (auto& branch : system.dc.branches) {
    fill_positive(branch.r_pu, value_or(library, "dc_branch.r_pu", 0.01),
                  "dc_branch.r_pu", report);
  }
  for (auto& converter : system.vsc_converters) {
    fill_positive(converter.p_rated_mw,
                  value_or(library, "vsc.p_rated_mw", 1.0),
                  "vsc.p_rated_mw", report);
    if (!std::isfinite(converter.eta) || converter.eta <= kMissing ||
        converter.eta > 1.0) {
      converter.eta = value_or(library, "vsc.eta", 0.99);
      note_change(report, "vsc.eta");
    }
    fill_positive(converter.r_sc_pu, value_or(library, "vsc.r_sc_pu", 0.01),
                  "vsc.r_sc_pu", report);
    fill_positive(converter.x_sc_pu, value_or(library, "vsc.x_sc_pu", 0.15),
                  "vsc.x_sc_pu", report);
    fill_positive(converter.i_max_pu, value_or(library, "vsc.i_max_pu", 1.2),
                  "vsc.i_max_pu", report);
  }
  for (auto& converter : system.dc.dcdc_converters) {
    if (!std::isfinite(converter.eta) || converter.eta <= kMissing ||
        converter.eta > 1.0) {
      converter.eta = value_or(library, "dcdc.eta", 0.98);
      note_change(report, "dcdc.eta");
    }
  }
  auto fill_storage = [&](auto& storage) {
    if (!std::isfinite(storage.eta_charge) || storage.eta_charge <= kMissing ||
        storage.eta_charge > 1.0) {
      storage.eta_charge = value_or(library, "storage.eta_charge", 0.95);
      note_change(report, "storage.eta_charge");
    }
    if (!std::isfinite(storage.eta_discharge) ||
        storage.eta_discharge <= kMissing || storage.eta_discharge > 1.0) {
      storage.eta_discharge =
          value_or(library, "storage.eta_discharge", 0.95);
      note_change(report, "storage.eta_discharge");
    }
  };
  for (auto& storage : system.ac.storage) fill_storage(storage);
  for (auto& storage : system.dc.storage) fill_storage(storage);
  for (auto& storage : system.dc.dc_storage) fill_storage(storage);

  if (system.three_phase_ac.has_value()) {
    auto& phase = *system.three_phase_ac;
    fill_positive(phase.base_mva, system.ac.base_mva, "system.base_mva", report);
    fill_positive(phase.base_freq_hz,
                  value_or(library, "system.frequency_hz", 50.0),
                  "system.frequency_hz", report);
    for (auto& bus : phase.buses) {
      fill_positive(bus.base_kv, value_or(library, "ac_bus.base_kv", 10.0),
                    "ac_bus.base_kv", report);
    }
    for (auto& line : phase.lines) {
      if (missing_impedance(line.r1_pu) && missing_impedance(line.x1_pu)) {
        fill_impedance(line.r1_pu,
                       value_or(library, "ac_branch.r_pu", 0.01),
                       "ac_branch.r_pu", report);
        fill_impedance(line.x1_pu,
                       value_or(library, "ac_branch.x_pu", 0.04),
                       "ac_branch.x_pu", report);
      }
    }
    for (auto& transformer : phase.transformers) {
      fill_positive(transformer.sn_mva,
                    value_or(library, "transformer.sn_mva", 10.0),
                    "transformer.sn_mva", report);
      fill_positive(transformer.vk_percent,
                    value_or(library, "transformer.vk_percent", 6.0),
                    "transformer.vk_percent", report);
      fill_positive(transformer.vkr_percent,
                    value_or(library, "transformer.vkr_percent", 0.6),
                    "transformer.vkr_percent", report);
    }
  }

  return report;
}

}  // namespace hacdcpf

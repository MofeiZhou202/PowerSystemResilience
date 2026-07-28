#include "hacdcpf/model/standard_parameter_library.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "hacdcpf/model/enum_strings.hpp"

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

std::string uppercase_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::toupper(ch));
                 });
  return value;
}

bool contains_text(const std::string& value, const std::string& token) {
  return value.find(token) != std::string::npos;
}

bool source_is_import_estimate(const std::string& source) {
  return source.rfind("cim_", 0) == 0 ||
         source.rfind("svg_geometry_estimate", 0) == 0 ||
         source.rfind("svg_estimate", 0) == 0 ||
         source.rfind("design_handbook", 0) == 0;
}

std::vector<DesignHandbookReference> design_handbook_references() {
  return {
      {"GB/T 3956-2008 / IEC 60228:2004",
       "20 C conductor maximum DC resistance tables",
       "normative_parameter_table",
       "https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=149B3068D059EFD9BCFB8A8AF57E5B1C"},
      {"GB/T 12706.1-2020",
       "Extruded-insulation power-cable construction and thermal context",
       "normative_product_context",
       "https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=7593C7389ACDA76E0D40F6985E3A839D"},
      {"GB/T 1179-2017",
       "Round-wire concentric-lay overhead conductor product context",
       "normative_product_context", "https://openstd.samr.gov.cn/"},
      {"GB/T 14049-2008",
       "10 kV and 35 kV aerial insulated cable product context",
       "normative_product_context", "https://openstd.samr.gov.cn/"},
      {"DL/T 5220-2021",
       "10 kV and below overhead distribution-line design context",
       "industry_design_context", "https://www.nea.gov.cn/"},
      {"GB/T 1094.1-2013; GB/T 6451-2015",
       "Transformer nameplate, losses and impedance-voltage context",
       "normative_equipment_context", "https://openstd.samr.gov.cn/"},
      {"GB/T 15544.1-2013 / IEC 60909-0:2016",
       "Short-circuit source equivalent and impedance conversion",
       "normative_calculation_method", "https://openstd.samr.gov.cn/"},
      {"HACDCPF distribution screening assumptions",
       "Default reactance, current-density rating and missing asset metadata",
       "engineering_assumption", ""},
  };
}

double standard_r20_ohm_per_km(double area_mm2, bool aluminium) {
  struct Row { double area; double copper; double aluminium; };
  static constexpr std::array<Row, 20> rows{{
      {1.5, 12.1, 0.0}, {2.5, 7.41, 0.0}, {4.0, 4.61, 0.0},
      {6.0, 3.08, 0.0}, {10.0, 1.83, 3.08}, {16.0, 1.15, 1.91},
      {25.0, 0.727, 1.20}, {35.0, 0.524, 0.868},
      {50.0, 0.387, 0.641}, {70.0, 0.268, 0.443},
      {95.0, 0.193, 0.320}, {120.0, 0.153, 0.253},
      {150.0, 0.124, 0.206}, {185.0, 0.0991, 0.164},
      {240.0, 0.0754, 0.125}, {300.0, 0.0601, 0.100},
      {400.0, 0.0470, 0.0778}, {500.0, 0.0366, 0.0605},
      {630.0, 0.0283, 0.0469}, {800.0, 0.0221, 0.0367},
  }};
  for (const auto& row : rows) {
    if (std::abs(row.area - area_mm2) <= 1e-6) {
      const double value = aluminium ? row.aluminium : row.copper;
      if (value > 0.0) return value;
      break;
    }
  }
  return 0.0;
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

void check_opf_decision_bounds(ParameterValidationReport& report,
                               const std::string& component_type,
                               int index,
                               const std::string& name,
                               const std::string& variable,
                               double lower,
                               double upper,
                               double initial) {
  const std::string bounds = variable + "_min/" + variable + "_max";
  if (!std::isfinite(lower) || !std::isfinite(upper) ||
      !std::isfinite(initial)) {
    add_issue(report, "opf_decision_bounds_nonfinite",
              ParameterIssueSeverity::Error, component_type, index, name,
              bounds, initial,
              component_type + " " + std::to_string(index) + " has non-finite " +
                  variable + " OPF bounds or initial value.");
    return;
  }
  if (lower > upper) {
    add_issue(report, "opf_decision_bounds_reversed",
              ParameterIssueSeverity::Error, component_type, index, name,
              bounds, initial,
              component_type + " " + std::to_string(index) + " requires " +
                  variable + "_min <= " + variable + "_max for OPF.");
    return;
  }

  const double scale = std::max({1.0, std::abs(lower), std::abs(upper)});
  const double width_tol = 1e-12 * scale;
  if (upper - lower <= width_tol) {
    add_issue(report, "opf_decision_bounds_zero_width",
              ParameterIssueSeverity::Warning, component_type, index, name,
              bounds, initial,
              component_type + " " + std::to_string(index) + " has zero-width " +
                  variable + " bounds; strict-interior OPF solvers may become "
                             "numerically infeasible. Use a small physical capability "
                             "interval or model the value as a fixed parameter.");
  }

  const double outside_tol = 1e-10 * scale;
  if (initial < lower - outside_tol || initial > upper + outside_tol) {
    add_issue(report, "opf_initial_value_outside_bounds",
              ParameterIssueSeverity::Error, component_type, index, name,
              variable, initial,
              component_type + " " + std::to_string(index) + " initial " +
                  variable + " is outside its OPF decision bounds.");
  } else if (upper - lower > width_tol &&
             (std::abs(initial - lower) <= outside_tol ||
              std::abs(initial - upper) <= outside_tol)) {
    add_issue(report, "opf_initial_value_on_bound",
              ParameterIssueSeverity::Warning, component_type, index, name,
              variable, initial,
              component_type + " " + std::to_string(index) + " initial " +
                  variable + " lies exactly on an OPF bound; an interior warm start "
                             "is numerically safer.");
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
      "GB/T 156-2017 / IEC 60038 voltage context; project nominal voltage required";
  const std::string voltage_quality_source =
      "GB/T 12325-2008 voltage-deviation context; project operating limits required";
  const std::string frequency_source =
      "GB/T 15945-2008 power-system frequency-deviation context";
  const std::string transformer_source =
      "GB/T 1094.1-2013; GB/T 6451-2015; verify manufacturer nameplate";
  const std::string reliability_source =
      "HACDCPF comprehensive distribution reliability baseline; replace with utility statistics";

  library.rules = {
      rule("system.base_mva", "System", "base_mva", "Power base", 100.0,
           "MVA", 0.1, 10000.0, ParameterIssueSeverity::Error, baseline,
           "Positive system power base used for per-unit conversion."),
      rule("system.frequency_hz", "System", "frequency_hz", "Nominal frequency",
           50.0, "Hz", 45.0, 65.0, ParameterIssueSeverity::Warning,
           frequency_source,
           "Nominal network frequency."),
      rule("ac_bus.base_kv", "AC bus", "base_kv", "Nominal line voltage",
           lv ? 0.4 : 10.0, "kV", 0.1, 1200.0, ParameterIssueSeverity::Error,
           voltage_source, "Positive line-to-line voltage base."),
      rule("ac_bus.vm_pu", "AC bus", "vm_pu", "Initial voltage", 1.0, "pu",
           0.5, 1.5, ParameterIssueSeverity::Warning, voltage_quality_source,
           "Initial positive-sequence voltage magnitude."),
      rule("ac_bus.vmin_pu", "AC bus", "vmin_pu", "Minimum voltage", 0.9, "pu",
           0.5, 1.1, ParameterIssueSeverity::Warning, voltage_quality_source,
           "Operational lower voltage bound."),
      rule("ac_bus.vmax_pu", "AC bus", "vmax_pu", "Maximum voltage", 1.1, "pu",
           0.9, 1.5, ParameterIssueSeverity::Warning, voltage_quality_source,
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
           ParameterIssueSeverity::Error, transformer_source,
           "Transformer nameplate apparent power."),
      rule("transformer.vk_percent", "Transformer", "vk_percent",
           "Short-circuit voltage", 6.0, "%", 0.1, 30.0,
           ParameterIssueSeverity::Error, transformer_source,
           "Transformer short-circuit voltage."),
      rule("transformer.vkr_percent", "Transformer", "vkr_percent",
           "Resistive short-circuit voltage", 0.6, "%", 0.01, 10.0,
           ParameterIssueSeverity::Warning, transformer_source,
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
      rule("reliability.ac_overhead.failure_rate", "Reliability - overhead line",
           "failure_rate", "Failure rate per kilometre", 0.50, "occ/(km*yr)", 1e-6, 100.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Annual failure frequency per kilometre of overhead line; model completion multiplies by line length."),
      rule("reliability.ac_overhead.mttr_hr", "Reliability - overhead line",
           "mttr_hr", "Mean repair time", 8.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean time to restore an overhead-line component."),
      rule("reliability.ac_cable.failure_rate", "Reliability - cable",
           "failure_rate", "Failure rate per kilometre", 0.08, "occ/(km*yr)", 1e-6, 100.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Annual failure frequency per kilometre of underground cable; model completion multiplies by line length."),
      rule("reliability.ac_cable.mttr_hr", "Reliability - cable",
           "mttr_hr", "Mean repair time", 24.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean time to locate, excavate and repair a cable fault."),
      rule("reliability.ac_branch.failure_rate", "Reliability - other line",
           "failure_rate", "Failure rate per kilometre", 0.30, "occ/(km*yr)", 1e-6, 100.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Fallback annual failure frequency per kilometre for an unclassified AC branch; model completion multiplies by line length."),
      rule("reliability.ac_branch.mttr_hr", "Reliability - other line",
           "mttr_hr", "Mean repair time", 6.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Fallback mean repair time for an unclassified AC branch."),
      rule("reliability.transformer.mtbf_hours", "Reliability - transformer",
           "mtbf_hours", "MTBF", 300000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean operating time to transformer failure."),
      rule("reliability.transformer.mttr_hours", "Reliability - transformer",
           "mttr_hours", "MTTR", 200.0, "h", 0.01, 87600.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean transformer repair or replacement duration."),
      rule("reliability.switch.mtbf_hours", "Reliability - switchgear",
           "mtbf_hours", "MTBF", 175200.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "Switchgear MTBF, equivalent to approximately 0.05 failures/year."),
      rule("reliability.switch.mttr_hours", "Reliability - switchgear",
           "mttr_hours", "MTTR", 8.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean switchgear repair or replacement duration."),
      rule("reliability.generator.forced_outage_rate", "Reliability - generator",
           "forced_outage_rate", "Forced outage rate", 0.02, "pu", 1e-6, 0.99,
           ParameterIssueSeverity::Warning, reliability_source,
           "Forced-outage probability for a dispatchable generator."),
      rule("reliability.generator.mttr_hr", "Reliability - generator",
           "mttr_hr", "MTTR", 40.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean generator repair duration."),
      rule("reliability.static_generator.mtbf_hours", "Reliability - distributed generation",
           "mtbf_hours", "MTBF", 3000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "MTBF for inverter-based or engine-based distributed generation."),
      rule("reliability.static_generator.mttr_hours", "Reliability - distributed generation",
           "mttr_hours", "MTTR", 24.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean distributed-generator repair duration."),
      rule("reliability.renewable.mtbf_hours", "Reliability - renewable generation",
           "mtbf_hours", "MTBF", 4000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "MTBF for wind, hydro or renewable generation."),
      rule("reliability.renewable.mttr_hours", "Reliability - renewable generation",
           "mttr_hours", "MTTR", 48.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean renewable-generator repair duration."),
      rule("reliability.pv.mtbf_hours", "Reliability - PV",
           "mtbf_hours", "MTBF", 8000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "Aggregate PV-system MTBF."),
      rule("reliability.pv.mttr_hours", "Reliability - PV",
           "mttr_hours", "MTTR", 12.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Aggregate PV-system repair duration."),
      rule("reliability.storage.forced_outage_rate", "Reliability - storage",
           "forced_outage_rate", "Forced outage rate", 0.015, "pu", 1e-6, 0.99,
           ParameterIssueSeverity::Warning, reliability_source,
           "Forced-outage probability for battery storage and PCS."),
      rule("reliability.storage.mttr_hr", "Reliability - storage",
           "mttr_hr", "MTTR", 24.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean storage-system repair duration."),
      rule("reliability.vsc.forced_outage_rate", "Reliability - VSC",
           "forced_outage_rate", "Forced outage rate", 0.01, "pu", 1e-6, 0.99,
           ParameterIssueSeverity::Warning, reliability_source,
           "Forced-outage probability for a VSC converter."),
      rule("reliability.vsc.mttr_hr", "Reliability - VSC",
           "mttr_hr", "MTTR", 48.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean VSC repair duration."),
      rule("reliability.dcdc.mtbf_hours", "Reliability - DC/DC",
           "mtbf_hours", "MTBF", 20000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "DC/DC converter MTBF."),
      rule("reliability.dcdc.mttr_hours", "Reliability - DC/DC",
           "mttr_hours", "MTTR", 36.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean DC/DC converter repair duration."),
      rule("reliability.dc_branch.mtbf_hours", "Reliability - DC branch",
           "mtbf_hours", "MTBF", 50000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "DC branch MTBF."),
      rule("reliability.dc_branch.mttr_hours", "Reliability - DC branch",
           "mttr_hours", "MTTR", 24.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean DC branch repair duration."),
      rule("reliability.microgrid.mtbf_hours", "Reliability - microgrid",
           "mtbf_hours", "MTBF", 20000.0, "h", 1.0, 1e8,
           ParameterIssueSeverity::Warning, reliability_source,
           "MTBF of the microgrid islanding and supervisory function."),
      rule("reliability.microgrid.mttr_hours", "Reliability - microgrid",
           "mttr_hours", "MTTR", 8.0, "h", 0.01, 8760.0,
           ParameterIssueSeverity::Warning, reliability_source,
           "Mean time to restore microgrid islanding and supervisory control."),
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
    const double p_initial = std::abs(converter.p_initial_mw) > kMissing
                                 ? converter.p_initial_mw
                                 : converter.p_set_mw;
    check_opf_decision_bounds(report, "VSC", converter.index, converter.name,
                              "p_mw", converter.pmin_mw, converter.pmax_mw,
                              p_initial);
    check_opf_decision_bounds(report, "VSC", converter.index, converter.name,
                              "q_mvar", converter.qmin_mvar,
                              converter.qmax_mvar, converter.q_set_mvar);
  }
  for (const auto& converter : system.dc.dcdc_converters) {
    if (!converter.in_service) continue;
    check_range(report, library, "dcdc.eta", "DC-DC converter",
                converter.index, converter.name, "eta", converter.eta);
    check_opf_decision_bounds(report, "DC-DC converter", converter.index,
                              converter.name, "p_mw", converter.pmin_mw,
                              converter.pmax_mw, converter.p_ref_mw);
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

  for (const auto& generator : system.ac.generators) {
    if (!generator.in_service) continue;
    check_opf_decision_bounds(report, "Generator", generator.index,
                              generator.name, "p_mw", generator.pmin_mw,
                              generator.pmax_mw, generator.pg_mw);
    check_opf_decision_bounds(report, "Generator", generator.index,
                              generator.name, "q_mvar", generator.qmin_mvar,
                              generator.qmax_mvar, generator.qg_mvar);
  }
  for (const auto& storage : system.ac.storage) {
    if (!storage.in_service) continue;
    check_opf_decision_bounds(report, "Storage", storage.index, storage.name,
                              "p_mw", storage.pmin_mw, storage.pmax_mw,
                              storage.p_mw);
    check_opf_decision_bounds(report, "Storage", storage.index, storage.name,
                              "q_mvar", storage.qmin_mvar, storage.qmax_mvar,
                              storage.q_mvar);
  }
  auto check_dc_storage_bounds = [&](const auto& storage) {
    if (!storage.in_service) return;
    check_opf_decision_bounds(report, "DC storage", storage.index,
                              storage.name, "p_mw", storage.pmin_mw,
                              storage.pmax_mw, storage.p_mw);
  };
  for (const auto& storage : system.dc.storage)
    check_dc_storage_bounds(storage);
  for (const auto& storage : system.dc.dc_storage)
    check_dc_storage_bounds(storage);
  for (const auto& storage : system.mobile_storage) {
    if (!storage.in_service ||
        storage.status == MobileStorageStatus::InTransit) {
      continue;
    }
    check_opf_decision_bounds(report, "Mobile storage", storage.index,
                              storage.name, "p_mw", storage.pmin_mw,
                              storage.pmax_mw, storage.p_mw);
    check_opf_decision_bounds(report, "Mobile storage", storage.index,
                              storage.name, "q_mvar", storage.qmin_mvar,
                              storage.qmax_mvar, storage.q_mvar);
  }
  for (const auto& renewable : system.ac.renewable_gens) {
    if (!renewable.in_service || !renewable.curtailable) continue;
    check_opf_decision_bounds(report, "Renewable generator", renewable.index,
                              renewable.name, "p_mw", 0.0,
                              renewable.p_rated_mw, renewable.p_mw);
    check_opf_decision_bounds(report, "Renewable generator", renewable.index,
                              renewable.name, "q_mvar", renewable.qmin_mvar,
                              renewable.qmax_mvar, renewable.q_mvar);
  }
  for (const auto& pv : system.ac.pv_systems) {
    if (!pv.in_service || !pv.controllable) continue;
    check_opf_decision_bounds(report, "PV system", pv.index, pv.name, "p_mw",
                              0.0, std::max(pv.pmax_mw, pv.sn_mva), pv.p_mw);
    check_opf_decision_bounds(report, "PV system", pv.index, pv.name,
                              "q_mvar", pv.qmin_mvar, pv.qmax_mvar,
                              pv.q_mvar);
  }
  for (const auto& load : system.ac.flexible_loads) {
    if (!load.in_service || !load.controllable) continue;
    check_opf_decision_bounds(report, "Flexible load", load.index, load.name,
                              "p_mw", load.p_mw - load.flex_down_mw,
                              load.p_mw + load.flex_up_mw, load.p_mw);
  }
  for (const auto& router : system.energy_routers) {
    if (!router.in_service) continue;
    for (const auto& port : router.ports) {
      if (!port.in_service) continue;
      const std::string port_name = router.name + "/" + port.name;
      check_opf_decision_bounds(report, "Energy router port", port.index,
                                port_name, "p_mw", port.pmin_mw,
                                port.pmax_mw, port.p_mw);
      if (port.port_type == ERPortType::AC) {
        check_opf_decision_bounds(report, "Energy router port", port.index,
                                  port_name, "q_mvar", port.qmin_mvar,
                                  port.qmax_mvar, port.q_mvar);
      }
    }
  }
  for (const auto& vpp : system.vpps) {
    if (!vpp.in_service) continue;
    check_opf_decision_bounds(report, "Virtual power plant", vpp.index,
                              vpp.name, "p_mw", vpp.pmin_mw, vpp.pmax_mw,
                              vpp.p_output_mw);
  }

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

  // Reliability defaults are materialized on the model so every downstream
  // method sees the same data.  Existing utility/manufacturer values are never
  // overwritten by this missing-only parameter-library operation.
  for (auto& branch : system.ac.branches) {
    const std::string geometry = uppercase_ascii(
        branch.line_type + " " + branch.conductor_model);
    const bool overhead = contains_text(branch.line_type, "架空") ||
                          contains_text(geometry, "OVERHEAD") ||
                          contains_text(geometry, "OHL");
    const bool cable = contains_text(branch.line_type, "电缆") ||
                       contains_text(geometry, "CABLE") ||
                       contains_text(geometry, "YJV") ||
                       contains_text(geometry, "XLPE");
    const std::string prefix = overhead
        ? "reliability.ac_overhead"
        : (cable ? "reliability.ac_cable" : "reliability.ac_branch");
    const double rate_per_km =
        value_or(library, prefix + ".failure_rate", 0.30);
    const double exposure_km =
        std::isfinite(branch.length_km) && branch.length_km > kMissing
            ? branch.length_km
            : 1.0;
    fill_positive(branch.failure_rate, rate_per_km * exposure_km,
                  prefix + ".failure_rate", report);
    fill_positive(branch.mttr_hr,
                  value_or(library, prefix + ".mttr_hr", 6.0),
                  prefix + ".mttr_hr", report);
  }
  for (auto& transformer : system.ac.transformers_2w) {
    fill_positive(transformer.mtbf_hours,
                  value_or(library, "reliability.transformer.mtbf_hours", 300000.0),
                  "reliability.transformer.mtbf_hours", report);
    fill_positive(transformer.mttr_hours,
                  value_or(library, "reliability.transformer.mttr_hours", 200.0),
                  "reliability.transformer.mttr_hours", report);
  }
  for (auto& transformer : system.ac.transformers_3w) {
    fill_positive(transformer.mtbf_hours,
                  value_or(library, "reliability.transformer.mtbf_hours", 300000.0),
                  "reliability.transformer.mtbf_hours", report);
    fill_positive(transformer.mttr_hours,
                  value_or(library, "reliability.transformer.mttr_hours", 200.0),
                  "reliability.transformer.mttr_hours", report);
  }
  for (auto& sw : system.ac.switches) {
    fill_positive(sw.mtbf_hours,
                  value_or(library, "reliability.switch.mtbf_hours", 175200.0),
                  "reliability.switch.mtbf_hours", report);
    fill_positive(sw.mttr_hours,
                  value_or(library, "reliability.switch.mttr_hours", 8.0),
                  "reliability.switch.mttr_hours", report);
  }
  for (auto& generator : system.ac.generators) {
    fill_positive(generator.forced_outage_rate,
                  value_or(library, "reliability.generator.forced_outage_rate", 0.02),
                  "reliability.generator.forced_outage_rate", report);
    fill_positive(generator.mttr_hr,
                  value_or(library, "reliability.generator.mttr_hr", 40.0),
                  "reliability.generator.mttr_hr", report);
  }
  const auto fill_mtbf_pair = [&](auto& item, const std::string& prefix,
                                  double mtbf, double mttr) {
    fill_positive(item.mtbf_hours, value_or(library, prefix + ".mtbf_hours", mtbf),
                  prefix + ".mtbf_hours", report);
    fill_positive(item.mttr_hours, value_or(library, prefix + ".mttr_hours", mttr),
                  prefix + ".mttr_hours", report);
  };
  for (auto& source : system.ac.static_generators)
    fill_mtbf_pair(source, "reliability.static_generator", 3000.0, 24.0);
  for (auto& source : system.ac.renewable_gens)
    fill_mtbf_pair(source, "reliability.renewable", 4000.0, 48.0);
  for (auto& source : system.ac.pv_systems)
    fill_mtbf_pair(source, "reliability.pv", 8000.0, 12.0);
  const auto fill_storage_reliability = [&](auto& storage) {
    fill_positive(storage.forced_outage_rate,
                  value_or(library, "reliability.storage.forced_outage_rate", 0.015),
                  "reliability.storage.forced_outage_rate", report);
    fill_positive(storage.mttr_hr,
                  value_or(library, "reliability.storage.mttr_hr", 24.0),
                  "reliability.storage.mttr_hr", report);
  };
  for (auto& storage : system.ac.storage) fill_storage_reliability(storage);
  for (auto& storage : system.dc.storage) fill_storage_reliability(storage);
  for (auto& storage : system.dc.dc_storage) fill_storage_reliability(storage);
  for (auto& converter : system.vsc_converters) {
    fill_positive(converter.forced_outage_rate,
                  value_or(library, "reliability.vsc.forced_outage_rate", 0.01),
                  "reliability.vsc.forced_outage_rate", report);
    fill_positive(converter.mttr_hr,
                  value_or(library, "reliability.vsc.mttr_hr", 48.0),
                  "reliability.vsc.mttr_hr", report);
  }
  for (auto& converter : system.dc.dcdc_converters)
    fill_mtbf_pair(converter, "reliability.dcdc", 20000.0, 36.0);
  for (auto& branch : system.dc.branches)
    fill_mtbf_pair(branch, "reliability.dc_branch", 50000.0, 24.0);
  for (auto& source : system.dc.pv_arrays)
    fill_mtbf_pair(source, "reliability.pv", 8000.0, 12.0);
  for (auto& source : system.dc.static_generators)
    fill_mtbf_pair(source, "reliability.static_generator", 3000.0, 24.0);
  for (auto& source : system.dc.dc_static_generators)
    fill_mtbf_pair(source, "reliability.static_generator", 3000.0, 24.0);
  for (auto& microgrid : system.microgrids)
    fill_mtbf_pair(microgrid, "reliability.microgrid", 20000.0, 8.0);

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

DesignHandbookCompletionReport complete_design_handbook_parameters(
    HybridPowerSystem& system,
    const DesignHandbookCompletionOptions& options) {
  DesignHandbookCompletionReport report;
  report.references = design_handbook_references();
  const double base_mva = system.ac.base_mva > kMissing
                              ? system.ac.base_mva
                              : system.base_mva;
  int material_geometry_conflicts = 0;
  int low_confidence_count = 0;
  int missing_model_count = 0;

  const auto bus_base_kv = [&](int index) {
    const auto it = std::find_if(system.ac.buses.begin(), system.ac.buses.end(),
                                 [&](const auto& bus) {
                                   return bus.index == index;
                                 });
    return it == system.ac.buses.end() ? 0.0 : it->base_kv;
  };

  for (auto& branch : system.ac.branches) {
    if (!branch.in_service || transformer_like(branch)) continue;
    ++report.branches_scanned;

    const bool has_impedance =
        std::isfinite(branch.r_pu) && std::isfinite(branch.x_pu) &&
        std::hypot(branch.r_pu, branch.x_pu) > kMissing;
    const bool replaceable_estimate =
        options.overwrite_import_estimates &&
        source_is_import_estimate(branch.parameter_source);
    if (has_impedance && !replaceable_estimate) {
      ++report.skipped_authored;
      continue;
    }

    const double base_kv = bus_base_kv(branch.from_bus) > kMissing
                               ? bus_base_kv(branch.from_bus)
                               : bus_base_kv(branch.to_bus);
    if (!(base_mva > kMissing) || !(base_kv > kMissing) ||
        !(branch.length_km > kMissing)) {
      ++report.skipped_missing_geometry;
      continue;
    }

    const std::string model = uppercase_ascii(branch.conductor_model);
    const std::string geometry = uppercase_ascii(branch.line_type);
    const bool type_overhead = contains_text(branch.line_type, "架空") ||
                               contains_text(geometry, "OVERHEAD") ||
                               contains_text(geometry, "OHL");
    const bool model_overhead = contains_text(model, "LGJ") ||
                                contains_text(model, "JKL") ||
                                contains_text(model, "JL/") ||
                                contains_text(model, "JL-") ||
                                model == "LJ" || model.rfind("LJ", 0) == 0;
    const bool overhead = type_overhead || model_overhead;
    const bool aluminium_by_model = contains_text(model, "YJLV") ||
                                    contains_text(model, "BLV") ||
                                    contains_text(model, "JKLYJ") ||
                                    model_overhead;
    const bool copper_by_model = !model.empty() && !aluminium_by_model;
    const bool aluminium = aluminium_by_model || (!copper_by_model && overhead);
    const bool model_cable_family = contains_text(model, "BVV") ||
                                    contains_text(model, "YJV");
    const bool model_type_conflict = type_overhead && model_cable_family;

    double area = branch.cross_section_mm2;
    const bool inferred_area = branch.cross_section_inferred ||
                               !(area > kMissing);
    // An importer fallback is not authored geometry.  Re-evaluate it from the
    // actual voltage class so models imported by older versions (which used an
    // LV fallback on MV cable/overhead PSR types) are repaired in-place.
    if (inferred_area) {
      area = base_kv > 1.0
                 ? options.default_mv_cross_section_mm2
                 : (overhead
                        ? options.default_lv_overhead_cross_section_mm2
                        : options.default_lv_cable_cross_section_mm2);
    }
    if (!(area > kMissing)) {
      ++report.skipped_missing_geometry;
      continue;
    }

    const bool explicitly_insulated_overhead = contains_text(model, "JKL");
    const bool xlpe = contains_text(model, "YJ") ||
                      (model.empty() && !overhead) ||
                      explicitly_insulated_overhead;
    const double temperature_c = xlpe ? 90.0 : 70.0;
    const double alpha = aluminium ? 0.00403 : 0.00393;
    const double temperature_factor =
        1.0 + alpha * (temperature_c - 20.0);
    double r20 = standard_r20_ohm_per_km(area, aluminium);
    double resistance = 0.0;
    if (r20 > kMissing) {
      resistance = r20 * temperature_factor;
    } else {
      // Resistivity-based screening fallback for a non-tabulated area. This is
      // reported as an engineering assumption, not a product-standard value.
      resistance = (aluminium ? 37.6 : 23.7) / area;
      r20 = resistance / temperature_factor;
    }
    const double reactance = overhead
                                 ? options.overhead_reactance_ohm_per_km
                                 : options.cable_reactance_ohm_per_km;
    const double zbase = base_kv * base_kv / base_mva;
    const double new_r_pu = resistance * branch.length_km / zbase;
    const double new_x_pu = reactance * branch.length_km / zbase;
    const double new_rate_a_mva =
        std::sqrt(3.0) * base_kv * (2.0 * area) / 1000.0;

    DesignHandbookLineSuggestion suggestion;
    suggestion.branch_index = branch.index;
    suggestion.branch_name = branch.name;
    suggestion.conductor_model = branch.conductor_model;
    suggestion.line_type = branch.line_type;
    suggestion.conductor_material = aluminium ? "aluminium" : "copper";
    suggestion.insulation = xlpe ? "XLPE" : "PVC";
    suggestion.confidence = model.empty() && inferred_area
                                ? "low"
                                : ((model.empty() || inferred_area ||
                                    model_type_conflict)
                                       ? "medium"
                                       : "high");
    suggestion.action = has_impedance ? "replace_import_estimate"
                                      : "fill_missing";
    suggestion.source = overhead
        ? "GB/T 3956-2008; GB/T 1179-2017; GB/T 14049-2008; DL/T 5220-2021; engineering screening assumptions"
        : "GB/T 3956-2008; GB/T 12706.1-2020; engineering screening assumptions";
    suggestion.base_kv = base_kv;
    suggestion.length_km = branch.length_km;
    suggestion.cross_section_mm2 = area;
    suggestion.conductor_temperature_c = temperature_c;
    suggestion.r20_ohm_per_km = r20;
    suggestion.old_r_ohm_per_km = branch.r_ohm_per_km;
    suggestion.old_x_ohm_per_km = branch.x_ohm_per_km;
    suggestion.new_r_ohm_per_km = resistance;
    suggestion.new_x_ohm_per_km = reactance;
    suggestion.old_r_pu = branch.r_pu;
    suggestion.old_x_pu = branch.x_pu;
    suggestion.new_r_pu = new_r_pu;
    suggestion.new_x_pu = new_x_pu;
    suggestion.old_rate_a_mva = branch.rate_a_mva;
    suggestion.new_rate_a_mva = new_rate_a_mva;
    suggestion.cross_section_inferred = inferred_area;
    suggestion.model_type_conflict = model_type_conflict;

    const auto changed = [](double old_value, double new_value) {
      const double scale = std::max({1.0, std::abs(old_value),
                                     std::abs(new_value)});
      return std::abs(old_value - new_value) > 1e-10 * scale;
    };
    const bool any_change = changed(branch.r_ohm_per_km, resistance) ||
                            changed(branch.x_ohm_per_km, reactance) ||
                            changed(branch.r_pu, new_r_pu) ||
                            changed(branch.x_pu, new_x_pu) ||
                            changed(branch.rate_a_mva, new_rate_a_mva) ||
                            changed(branch.cross_section_mm2, area);
    ++report.candidates;
    if (model_type_conflict) ++material_geometry_conflicts;
    if (model.empty()) ++missing_model_count;
    if (suggestion.confidence == "low") ++low_confidence_count;

    if (options.apply && any_change) {
      const double old_r_pu = branch.r_pu;
      const double old_x_pu = branch.x_pu;
      const double old_rate_a_mva = branch.rate_a_mva;
      const bool rate_b_followed_a =
          !(branch.rate_b_mva > kMissing) ||
          !changed(branch.rate_b_mva, old_rate_a_mva);
      const bool rate_c_followed_a =
          !(branch.rate_c_mva > kMissing) ||
          !changed(branch.rate_c_mva, old_rate_a_mva);
      if (changed(branch.r_ohm_per_km, resistance)) ++report.fields_changed;
      if (changed(branch.x_ohm_per_km, reactance)) ++report.fields_changed;
      if (changed(branch.r_pu, new_r_pu)) ++report.fields_changed;
      if (changed(branch.x_pu, new_x_pu)) ++report.fields_changed;
      if (changed(branch.rate_a_mva, new_rate_a_mva)) ++report.fields_changed;
      if (changed(branch.cross_section_mm2, area)) ++report.fields_changed;
      branch.r_ohm_per_km = resistance;
      branch.x_ohm_per_km = reactance;
      branch.r_pu = new_r_pu;
      branch.x_pu = new_x_pu;
      branch.rate_a_mva = new_rate_a_mva;
      if (rate_b_followed_a) {
        if (changed(branch.rate_b_mva, new_rate_a_mva))
          ++report.fields_changed;
        branch.rate_b_mva = new_rate_a_mva;
      }
      if (rate_c_followed_a) {
        if (changed(branch.rate_c_mva, new_rate_a_mva))
          ++report.fields_changed;
        branch.rate_c_mva = new_rate_a_mva;
      }
      branch.cross_section_mm2 = area;
      branch.cross_section_inferred = inferred_area;
      branch.parameter_source =
          "design_handbook_gbt3956_distribution_standards";
      branch.parameters_inferred = true;
      suggestion.applied = true;

      if (system.three_phase_ac.has_value()) {
        for (auto& line : system.three_phase_ac->lines) {
          if (line.index != branch.index) continue;
          const bool zero_sequence_was_derived =
              std::abs(line.r0_pu - 3.0 * old_r_pu) <=
                  1e-8 * std::max(1.0, std::abs(line.r0_pu)) &&
              std::abs(line.x0_pu - 3.0 * old_x_pu) <=
                  1e-8 * std::max(1.0, std::abs(line.x0_pu));
          line.r1_ohm_per_km = resistance;
          line.x1_ohm_per_km = reactance;
          line.r1_pu = new_r_pu;
          line.x1_pu = new_x_pu;
          if (zero_sequence_was_derived) {
            line.r0_ohm_per_km = 3.0 * resistance;
            line.x0_ohm_per_km = 3.0 * reactance;
            line.r0_pu = 3.0 * new_r_pu;
            line.x0_pu = 3.0 * new_x_pu;
          }
          break;
        }
      }
    }
    report.suggestions.push_back(std::move(suggestion));
  }

  if (options.infer_switch_bindings && !system.ac.switches.empty()) {
    struct TopologyEdge {
      int to{-1};
      int switch_pos{-1};
    };
    std::unordered_map<int, int> bus_pos;
    for (int i = 0; i < static_cast<int>(system.ac.buses.size()); ++i)
      bus_pos[system.ac.buses[i].index] = i;
    std::vector<std::vector<TopologyEdge>> adjacency(system.ac.buses.size());
    const auto add_edge = [&](int from_bus, int to_bus, int switch_pos) {
      const auto from = bus_pos.find(from_bus);
      const auto to = bus_pos.find(to_bus);
      if (from == bus_pos.end() || to == bus_pos.end()) return;
      adjacency[from->second].push_back({to->second, switch_pos});
      adjacency[to->second].push_back({from->second, switch_pos});
    };
    for (const auto& branch : system.ac.branches)
      if (branch.in_service) add_edge(branch.from_bus, branch.to_bus, -1);
    for (int i = 0; i < static_cast<int>(system.ac.switches.size()); ++i) {
      const auto& sw = system.ac.switches[i];
      if (sw.in_service && effective_switch_normal_closed(sw))
        add_edge(sw.bus_from, sw.bus_to, i);
    }
    for (const auto& transformer : system.ac.transformers_2w)
      if (transformer.in_service)
        add_edge(transformer.hv_bus, transformer.lv_bus, -1);

    constexpr int kUnreached = std::numeric_limits<int>::max();
    std::vector<int> distance(system.ac.buses.size(), kUnreached);
    std::vector<int> parent(system.ac.buses.size(), -1);
    std::vector<int> parent_switch(system.ac.buses.size(), -1);
    std::vector<int> source_root(system.ac.buses.size(), -1);
    std::queue<int> queue;
    const auto add_source = [&](int bus) {
      const auto it = bus_pos.find(bus);
      if (it == bus_pos.end() || distance[it->second] != kUnreached) return;
      distance[it->second] = 0;
      source_root[it->second] = it->second;
      queue.push(it->second);
    };
    for (const auto& grid : system.ac.external_grids)
      if (grid.in_service) add_source(grid.bus);
    for (const auto& generator : system.ac.generators)
      if (generator.in_service && generator.is_slack) add_source(generator.bus);
    for (const auto& bus : system.ac.buses)
      if (bus.in_service && bus.bus_type == BusType::SLACK) add_source(bus.index);
    if (queue.empty() && !system.ac.buses.empty()) add_source(system.ac.buses.front().index);
    while (!queue.empty()) {
      const int at = queue.front();
      queue.pop();
      for (const auto& edge : adjacency[at]) {
        if (distance[edge.to] != kUnreached) continue;
        distance[edge.to] = distance[at] + 1;
        parent[edge.to] = at;
        parent_switch[edge.to] = edge.switch_pos;
        source_root[edge.to] = source_root[at];
        queue.push(edge.to);
      }
    }

    auto proposed = system.ac.switches;
    struct TargetCandidate {
      std::string type;
      int index{-1};
      int priority{0};
    };
    const auto infer_type_and_role = [&](Switch& sw) {
      if (sw.switch_type == SwitchType::Unknown) {
        if (contains_text(sw.name, "熔断")) sw.switch_type = SwitchType::Fuse;
        else if (contains_text(sw.name, "重合")) sw.switch_type = SwitchType::Recloser;
        else if (contains_text(sw.name, "分段")) sw.switch_type = SwitchType::Sectionalizer;
        else if (contains_text(sw.name, "隔离")) sw.switch_type = SwitchType::Disconnector;
        else if (contains_text(sw.name, "联络") || contains_text(sw.name, "环网"))
          sw.switch_type = SwitchType::LoadBreakSwitch;
      }
      if (sw.role == SwitchRole::Unspecified) {
        switch (sw.switch_type) {
          case SwitchType::CircuitBreaker:
          case SwitchType::Fuse:
          case SwitchType::Recloser:
            sw.role = SwitchRole::Protection;
            break;
          case SwitchType::Sectionalizer:
            sw.role = SwitchRole::Sectionalizing;
            break;
          case SwitchType::Disconnector:
            sw.role = SwitchRole::Isolation;
            break;
          case SwitchType::LoadBreakSwitch:
            sw.role = (contains_text(sw.name, "联络") ||
                       contains_text(sw.name, "环网"))
                          ? SwitchRole::Tie
                          : SwitchRole::Sectionalizing;
            break;
          case SwitchType::Unknown:
            break;
        }
      }
      if (!sw.capabilities_explicit && sw.switch_type != SwitchType::Unknown) {
        sw.capabilities = effective_switch_capabilities(sw);
        sw.capabilities_explicit = true;
      }
    };
    for (auto& sw : proposed) infer_type_and_role(sw);

    for (int pos = 0; pos < static_cast<int>(proposed.size()); ++pos) {
      auto& sw = proposed[pos];
      const auto& original = system.ac.switches[pos];
      ++report.switches_scanned;
      DesignHandbookSwitchBindingSuggestion suggestion;
      suggestion.switch_index = sw.index;
      suggestion.switch_name = sw.name;
      suggestion.switch_type = switch_type_str(sw.switch_type);
      suggestion.old_role = switch_role_str(original.role);
      suggestion.new_role = switch_role_str(sw.role);
      suggestion.source = "topology inference from normal switching state and source distance";

      const bool binding_present =
          original.controlled_element_index >= 0 ||
          original.controlled_branch_index >= 0;
      const bool may_replace_binding = !binding_present ||
          (options.overwrite_inferred_switch_bindings && original.binding_inferred);
      if (sw.role == SwitchRole::Tie) {
        const auto from = bus_pos.find(sw.bus_from);
        const auto to = bus_pos.find(sw.bus_to);
        if (from != bus_pos.end() && to != bus_pos.end() &&
            source_root[from->second] >= 0 && source_root[to->second] >= 0 &&
            source_root[from->second] != source_root[to->second])
          sw.synchronization_required = true;
        suggestion.action = "classify_tie";
        suggestion.confidence = "high";
        suggestion.rationale =
            "tie role is retained; no protected branch is assigned to a restoration tie";
      } else if (may_replace_binding &&
                 (sw.role == SwitchRole::Protection ||
                  sw.role == SwitchRole::Sectionalizing ||
                  sw.role == SwitchRole::Isolation)) {
        const auto from = bus_pos.find(sw.bus_from);
        const auto to = bus_pos.find(sw.bus_to);
        int downstream_bus = sw.bus_to;
        int downstream_pos = to == bus_pos.end() ? -1 : to->second;
        if (from != bus_pos.end() && to != bus_pos.end() &&
            distance[from->second] > distance[to->second]) {
          downstream_bus = sw.bus_from;
          downstream_pos = from->second;
        }
        std::vector<TargetCandidate> targets;
        const auto add_targets = [&](int bus, bool downstream_only) {
          for (const auto& branch : system.ac.branches) {
            if (!branch.in_service ||
                (branch.from_bus != bus && branch.to_bus != bus)) continue;
            const int other_bus = branch.from_bus == bus ? branch.to_bus
                                                         : branch.from_bus;
            const auto other = bus_pos.find(other_bus);
            const bool outgoing = downstream_pos >= 0 && other != bus_pos.end() &&
                distance[other->second] >= distance[downstream_pos];
            if (!downstream_only || outgoing)
              targets.push_back({"ac_branch", branch.index,
                                 downstream_only
                                     ? (sw.switch_type == SwitchType::Fuse ? 1 : 0)
                                     : 3});
          }
          for (const auto& transformer : system.ac.transformers_2w) {
            if (!transformer.in_service ||
                (transformer.hv_bus != bus && transformer.lv_bus != bus)) continue;
            const int priority = sw.switch_type == SwitchType::Fuse
                                     ? (downstream_only ? 0 : 2)
                                     : (downstream_only ? 1 : 4);
            targets.push_back({"transformer_2w", transformer.index, priority});
          }
        };
        add_targets(downstream_bus, true);
        if (targets.empty()) add_targets(downstream_bus, false);
        if (targets.empty()) {
          add_targets(sw.bus_from == downstream_bus ? sw.bus_to : sw.bus_from,
                      false);
          for (auto& target : targets) target.priority += 2;
        }
        std::sort(targets.begin(), targets.end(), [](const auto& lhs, const auto& rhs) {
          if (lhs.priority != rhs.priority) return lhs.priority < rhs.priority;
          if (lhs.type != rhs.type) return lhs.type < rhs.type;
          return lhs.index < rhs.index;
        });
        targets.erase(std::unique(targets.begin(), targets.end(), [](const auto& lhs,
                                                                     const auto& rhs) {
          return lhs.type == rhs.type && lhs.index == rhs.index;
        }), targets.end());
        const int best_priority = targets.empty() ? -1 : targets.front().priority;
        const int best_count = best_priority < 0 ? 0 : static_cast<int>(std::count_if(
            targets.begin(), targets.end(), [&](const auto& target) {
              return target.priority == best_priority;
            }));
        if (best_count == 1) {
          const auto& target = targets.front();
          sw.controlled_element_type = target.type;
          sw.controlled_element_index = target.index;
          sw.controlled_branch_index = target.type == "ac_branch" ? target.index : -1;
          sw.protection_zone_id = target.type == "ac_branch"
                                      ? target.index
                                      : 1000000 + target.index;
          sw.binding_inferred = true;
          sw.binding_source = "design_handbook_topology_inference_v1";
          suggestion.action = binding_present ? "replace_inferred_binding"
                                              : "fill_binding";
          suggestion.confidence = best_priority <= 1 ? "high" : "medium";
          suggestion.rationale =
              "unique nearest downstream equipment in the normal source-rooted topology";
        } else if (best_count > 1) {
          suggestion.action = "review_required";
          suggestion.confidence = "low";
          suggestion.ambiguous = true;
          suggestion.rationale =
              std::to_string(best_count) +
              " equally ranked downstream equipment candidates";
          ++report.skipped_ambiguous_bindings;
        } else {
          suggestion.action = "no_topology_target";
          suggestion.confidence = "low";
          suggestion.rationale = "no adjacent branch or transformer could be identified";
        }
      } else if (binding_present) {
        suggestion.action = "preserve_authored_binding";
        suggestion.confidence = "high";
        suggestion.rationale = "existing authored binding is preserved";
      } else {
        suggestion.action = "classification_only";
        suggestion.confidence = "low";
        suggestion.rationale = "device role does not define a protected equipment target";
      }
      suggestion.controlled_element_type = sw.controlled_element_type;
      suggestion.controlled_element_index = sw.controlled_element_index;
      suggestion.controlled_branch_index = sw.controlled_branch_index;
      suggestion.protection_zone_id = sw.protection_zone_id;
      report.switch_bindings.push_back(std::move(suggestion));
    }

    // Boundary devices inherit the nearest upstream fault-interrupting device
    // on the source-rooted normal-state parent path.  The generic dependency
    // is used by disconnectors as well as sectionalizing equipment; the
    // nested sectionalizer field remains a backward-compatible alias.
    for (int pos = 0; pos < static_cast<int>(proposed.size()); ++pos) {
      auto& sw = proposed[pos];
      const auto& original = system.ac.switches[pos];
      if (sw.role != SwitchRole::Sectionalizing &&
          sw.role != SwitchRole::Isolation)
        continue;
      if (sw.upstream_protective_switch_index < 0 &&
          sw.sectionalizer_protection.upstream_switch_index >= 0) {
        sw.upstream_protective_switch_index =
            sw.sectionalizer_protection.upstream_switch_index;
      }
      const auto from = bus_pos.find(sw.bus_from);
      const auto to = bus_pos.find(sw.bus_to);
      if (from == bus_pos.end() || to == bus_pos.end()) continue;
      const bool may_replace_upstream =
          sw.upstream_protective_switch_index < 0 ||
          (options.overwrite_inferred_switch_bindings &&
           original.binding_inferred);
      if (may_replace_upstream) {
        sw.upstream_protective_switch_index = -1;
        int upstream_bus = distance[from->second] <= distance[to->second]
                               ? sw.bus_from : sw.bus_to;
        if (distance[from->second] == kUnreached &&
            distance[to->second] == kUnreached &&
            sw.controlled_element_index >= 0) {
          if (sw.controlled_element_type == "ac_branch") {
            const auto target = std::find_if(
                system.ac.branches.begin(), system.ac.branches.end(),
                [&](const auto& branch) {
                  return branch.index == sw.controlled_element_index;
                });
            if (target != system.ac.branches.end()) {
              if (target->from_bus == sw.bus_from ||
                  target->to_bus == sw.bus_from)
                upstream_bus = sw.bus_to;
              else if (target->from_bus == sw.bus_to ||
                       target->to_bus == sw.bus_to)
                upstream_bus = sw.bus_from;
            }
          } else if (sw.controlled_element_type == "transformer_2w") {
            const auto target = std::find_if(
                system.ac.transformers_2w.begin(),
                system.ac.transformers_2w.end(), [&](const auto& transformer) {
                  return transformer.index == sw.controlled_element_index;
                });
            if (target != system.ac.transformers_2w.end()) {
              if (target->hv_bus == sw.bus_from ||
                  target->lv_bus == sw.bus_from)
                upstream_bus = sw.bus_to;
              else if (target->hv_bus == sw.bus_to ||
                       target->lv_bus == sw.bus_to)
                upstream_bus = sw.bus_from;
            }
          }
        }
        int at = bus_pos.at(upstream_bus);
        while (at >= 0 && parent[at] >= 0) {
          const int upstream_switch = parent_switch[at];
          if (upstream_switch >= 0 && upstream_switch != pos) {
            const auto& candidate = proposed[upstream_switch];
            if (candidate.role == SwitchRole::Protection &&
                effective_switch_capabilities(candidate)
                    .can_interrupt_fault_current) {
              sw.upstream_protective_switch_index = candidate.index;
              break;
            }
          }
          at = parent[at];
        }
        // A disconnected imported island has no source-rooted parent path.
        // Search locally from the equipment side opposite the controlled
        // element, without crossing the boundary device itself.
        if (sw.upstream_protective_switch_index < 0) {
          std::vector<bool> visited(system.ac.buses.size(), false);
          std::queue<int> local_queue;
          visited[at = bus_pos.at(upstream_bus)] = true;
          local_queue.push(at);
          while (!local_queue.empty() &&
                 sw.upstream_protective_switch_index < 0) {
            at = local_queue.front();
            local_queue.pop();
            for (const auto& edge : adjacency[at]) {
              if (edge.switch_pos == pos) continue;
              if (edge.switch_pos >= 0) {
                const auto& candidate = proposed[edge.switch_pos];
                if (candidate.role == SwitchRole::Protection &&
                    effective_switch_capabilities(candidate)
                        .can_interrupt_fault_current) {
                  sw.upstream_protective_switch_index = candidate.index;
                  break;
                }
              }
              if (!visited[edge.to]) {
                visited[edge.to] = true;
                local_queue.push(edge.to);
              }
            }
          }
        }
      }
      if (sw.role == SwitchRole::Sectionalizing)
        sw.sectionalizer_protection.upstream_switch_index =
            sw.upstream_protective_switch_index;
      report.switch_bindings[pos].upstream_protective_switch_index =
          sw.upstream_protective_switch_index;
    }

    for (int pos = 0; pos < static_cast<int>(proposed.size()); ++pos) {
      const auto& before = system.ac.switches[pos];
      auto& after = proposed[pos];
      int changed_fields = 0;
      changed_fields += before.switch_type != after.switch_type;
      changed_fields += before.role != after.role;
      changed_fields += before.capabilities_explicit != after.capabilities_explicit;
      changed_fields += before.controlled_element_type != after.controlled_element_type;
      changed_fields += before.controlled_element_index != after.controlled_element_index;
      changed_fields += before.controlled_branch_index != after.controlled_branch_index;
      changed_fields += before.protection_zone_id != after.protection_zone_id;
      changed_fields += before.upstream_protective_switch_index !=
                        after.upstream_protective_switch_index;
      changed_fields += before.sectionalizer_protection.upstream_switch_index !=
                        after.sectionalizer_protection.upstream_switch_index;
      changed_fields += before.synchronization_required != after.synchronization_required;
      changed_fields += before.binding_inferred != after.binding_inferred;
      changed_fields += before.binding_source != after.binding_source;
      if (changed_fields <= 0) continue;
      ++report.switch_binding_candidates;
      if (options.apply) {
        system.ac.switches[pos] = after;
        report.switch_bindings[pos].applied = true;
        report.switch_binding_fields_changed += changed_fields;
        report.fields_changed += changed_fields;
      }
    }
    if (report.skipped_ambiguous_bindings > 0) {
      report.warnings.push_back(
          std::to_string(report.skipped_ambiguous_bindings) +
          " switch binding(s) have multiple equally ranked topology targets; "
          "no controlled equipment was assigned.");
    }
  }

  if (material_geometry_conflicts > 0) {
    report.warnings.push_back(
        std::to_string(material_geometry_conflicts) +
        " line(s) have a cable-family model name but an overhead PSR type; "
        "material follows the model and reactance follows the PSR geometry.");
  }
  if (low_confidence_count > 0) {
    report.warnings.push_back(
        std::to_string(low_confidence_count) +
        " line(s) lack both a reliable model and an explicit cross-section; "
        "the configured voltage/type fallback was used.");
  }
  if (missing_model_count > low_confidence_count) {
    report.warnings.push_back(
        std::to_string(missing_model_count - low_confidence_count) +
        " additional line(s) have an explicit cross-section but no conductor "
        "model; material and insulation remain medium-confidence inferences.");
  }
  return report;
}

}  // namespace hacdcpf

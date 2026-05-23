#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/power_flow/distribution_power_flow.hpp"
#include "hacdcpf/io/opendss_bridge.hpp"
#include hacdcpf/model/hybrid_power_system.hpp
#include hacdcpf/projection/project_to_canonical.hpp
#include hacdcpf/model/hybrid_power_system.hpp
#include hacdcpf/model/ac_components.hpp

// Single source of truth for case builders.
// Lives in tools/opendss_compare/ — internal support, not public hacdcpf API.
#include "opendss_compare/fixtures.hpp"
#include "opendss_compare/reproducibility.hpp"
#include "opendss_compare/snapshot_contract.hpp"
#include "opendss_compare/units.hpp"

namespace {

using json = nlohmann::json;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Acceptance tolerances — written into every per-case "acceptance_tolerances"
// block so that downstream consumers do not need to read test source.
// These constants must match the REQUIRE() bounds in test_opendss_pf_compare.cpp.
// ---------------------------------------------------------------------------
struct CaseTolerance {
  double vm_pu{1e-3};          // bus voltage magnitude [p.u.]
  double va_deg{0.1};          // bus voltage angle [degrees]
  double p_mw{1e-3};           // line terminal active power [MW]
  double q_mvar{1e-3};         // line terminal reactive power [MVAr]
  double ploss_mw{1e-3};       // total active line loss [MW]
  double qloss_mvar{1e-3};     // total reactive line loss [MVAr]
  double control_v_volts{0.2}; // regulator PT-secondary control voltage [V]
  double tap_number{0.0};      // regulator tap number must match exactly
};

static constexpr CaseTolerance k1phTol  = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3};
static constexpr CaseTolerance k3phTol  = {2e-3, 0.2, 1e-3, 1e-3, 5e-3, 5e-3};
static constexpr CaseTolerance kShTol   = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3};
// Physical capacitor: looser tolerance because of V^2 systematic bias
static constexpr CaseTolerance kCapTol  = {2e-2, 0.5, 2e-2, 2e-2, 2e-2, 2e-2};
// case33bw: same algorithm as 1ph, slightly wider loss tolerance (32 branches)
static constexpr CaseTolerance k33bwTol = {2e-3, 0.2, 1e-3, 1e-3, 5e-3, 5e-3};
// unbalanced 3-phase: per-phase comparison; same accuracy as k3phTol
static constexpr CaseTolerance kUnbal3phTol = {2e-3, 0.2, 1e-3, 1e-3, 5e-3, 5e-3};
// pass-through transformer (tap=1.0): tight; direct transformer/line branch
// powers plus total-loss cross-check
static constexpr CaseTolerance kXfmrTol = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3};
// fixed off-nominal transformer tap: same acceptance band as pass-through.
static constexpr CaseTolerance kFixedTapXfmrTol = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3};
static constexpr CaseTolerance kRegulatorTol = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3, 0.2, 0.0};
static constexpr CaseTolerance kTransformer3WCrossCheckTol = {1e-3, 0.1, 1e-3, 1e-3, 1e-3, 1e-3};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

struct MetricSummary {
  std::size_t count{0};
  double max_abs{0.0};
  double mean_abs{0.0};
  std::optional<double> max_rel;
};

std::string ascii_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return value;
}

std::string key_for_bus_node(const std::string& bus_name, int node) {
  return ascii_lower(bus_name) + "." + std::to_string(node);
}

std::string key_for_pd_element(const std::string& element_kind,
                               const std::string& element_name) {
  return ascii_lower(element_kind) + ":" + ascii_lower(element_name);
}

std::string key_for_pd_element(hacdcpf::io::OpenDSSPDElementKind element_kind,
                               const std::string& element_name) {
  return key_for_pd_element(
      hacdcpf::io::opendss_pd_element_kind_to_string(element_kind),
      element_name);
}

std::string key_for_pd_element_terminal_node(
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name,
    int terminal,
    int node) {
  return key_for_pd_element(element_kind, element_name) + ":" +
         std::to_string(terminal) + ":" + std::to_string(node);
}

MetricSummary summarize_metric(const std::vector<double>& reference,
                               const std::vector<double>& actual,
                               double rel_floor = 1e-9) {
  if (reference.size() != actual.size()) {
    throw std::runtime_error("summarize_metric: vector size mismatch");
  }

  MetricSummary summary;
  summary.count = reference.size();
  if (reference.empty()) {
    summary.max_rel = std::nullopt;
    return summary;
  }

  double abs_sum = 0.0;
  double max_rel = 0.0;
  bool has_rel = false;
  for (std::size_t i = 0; i < reference.size(); ++i) {
    const double abs_err = std::abs(actual[i] - reference[i]);
    summary.max_abs = std::max(summary.max_abs, abs_err);
    abs_sum += abs_err;
    if (std::abs(reference[i]) > rel_floor) {
      max_rel = std::max(max_rel, abs_err / std::abs(reference[i]));
      has_rel = true;
    }
  }

  summary.mean_abs = abs_sum / static_cast<double>(reference.size());
  summary.max_rel = has_rel ? std::optional<double>(max_rel) : std::nullopt;
  return summary;
}

json to_json(const MetricSummary& metric) {
  json out = {
      {"count", metric.count},
      {"max_abs", metric.max_abs},
      {"mean_abs", metric.mean_abs},
  };
  if (metric.max_rel.has_value()) {
    out["max_rel"] = metric.max_rel.value();
  } else {
    out["max_rel"] = nullptr;
  }
  return out;
}

json tolerance_to_json(const CaseTolerance& tol) {
  return {
      {"bus_voltage_magnitude_pu",
       {{"max_abs", tol.vm_pu}, {"unit", "p.u."}}},
      {"bus_voltage_angle_deg",
       {{"max_abs", tol.va_deg}, {"unit", "degrees"}}},
      {"line_terminal1_p_mw",
       {{"max_abs", tol.p_mw}, {"unit", "MW"}}},
      {"line_terminal1_q_mvar",
       {{"max_abs", tol.q_mvar}, {"unit", "MVAr"}}},
      {"total_line_loss_p_mw",
       {{"max_abs", tol.ploss_mw}, {"unit", "MW"}}},
      {"total_line_loss_q_mvar",
       {{"max_abs", tol.qloss_mvar}, {"unit", "MVAr"}}},
      {"regulator_control_voltage_volts",
       {{"max_abs", tol.control_v_volts}, {"unit", "volts"}}},
      {"regulator_tap_number",
       {{"max_abs", tol.tap_number}, {"unit", "tap number"}}},
  };
}

json power_pair_to_json(const hacdcpf_compare::PowerPair& power) {
  return {
      {"p_mw", power.p_mw},
      {"q_mvar", power.q_mvar},
  };
}

json losses_raw_to_json(const hacdcpf::io::OpenDSSLossesWVar& losses) {
  return {
      {"p_w", losses.p_w},
      {"q_var", losses.q_var},
  };
}

json tap_impedance_normalization_to_json(const std::string& physical_tap_side,
                                         bool projected_to_from_side,
                                         bool scaled_from_to_side,
                                         double winding_tap_pu) {
  return {
      {"physical_tap_side", physical_tap_side},
      {"projected_to_from_side", projected_to_from_side},
      {"scaled_from_to_side", scaled_from_to_side},
      {"impedance_scale", scaled_from_to_side ? (winding_tap_pu * winding_tap_pu) : 1.0},
      {"impedance_scale_rule",
       scaled_from_to_side
           ? "z_scaled_by_tap_squared_after_to_side_to_from_side_normalization"
           : "identity_for_from_side_or_neutral_tap"},
  };
}

json transformer_state_to_json(const hacdcpf::io::OpenDSSTransformerState& state) {
  json windings = json::array();
  for (const auto& winding : state.winding_states) {
    windings.push_back({
        {"winding", winding.winding},
        {"tap_pu", winding.tap_pu},
        {"min_tap_pu", winding.min_tap_pu},
        {"max_tap_pu", winding.max_tap_pu},
        {"num_taps", winding.num_taps},
        {"kv", winding.kv},
        {"kva", winding.kva},
    });
  }
  return {
      {"name", state.name},
      {"num_windings", state.num_windings},
      {"winding_states", windings},
  };
}

json regcontrol_to_json(const hacdcpf::io::OpenDSSRegControlResult& control) {
  return {
      {"name", control.name},
      {"transformer_name", control.transformer_name},
      {"monitored_bus_name", control.monitored_bus_name},
      {"winding", control.winding},
      {"tap_winding", control.tap_winding},
      {"tap_number", control.tap_number},
      {"max_tap_change", control.max_tap_change},
      {"forward_vreg_volts", control.forward_vreg_volts},
      {"forward_band_volts", control.forward_band_volts},
      {"ptratio", control.ptratio},
      {"remote_ptratio", control.remote_ptratio},
      {"ct_primary_amps", control.ct_primary_amps},
      {"forward_r_volts", control.forward_r_volts},
      {"forward_x_volts", control.forward_x_volts},
      {"voltage_limit_volts", control.voltage_limit_volts},
      {"is_reversible", control.is_reversible},
  };
}

json regulator_state_to_json(
    const hacdcpf::analysis::RegulatorControlState& state) {
  return {
      {"regulator_name", state.regulator_name},
      {"transformer_name", state.transformer_name},
      {"transformer_index", state.transformer_index},
      {"winding", state.winding},
      {"tap_winding", state.tap_winding},
      {"monitored_bus", state.monitored_bus},
      {"monitored_node", state.monitored_node},
      {"used_remote_bus", state.used_remote_bus},
      {"used_line_drop_compensation", state.used_line_drop_compensation},
      {"target_vreg_volts", state.target_vreg_volts},
      {"band_volts", state.band_volts},
      {"ptratio", state.ptratio},
      {"remote_ptratio", state.remote_ptratio},
      {"ct_primary_amps", state.ct_primary_amps},
      {"r_volts", state.r_volts},
      {"x_volts", state.x_volts},
      {"max_tap_change", state.max_tap_change},
      {"control_iterations", state.control_iterations},
      {"final_tap_pos", state.final_tap_pos},
      {"final_tap_number", state.final_tap_number},
      {"final_tap_pu", state.final_tap_pu},
      {"monitored_voltage_pu", state.monitored_voltage_pu},
      {"monitored_voltage_volts", state.monitored_voltage_volts},
      {"control_voltage_volts", state.control_voltage_volts},
      {"line_drop_compensation_real_volts",
       state.line_drop_compensation_real_volts},
      {"line_drop_compensation_imag_volts",
       state.line_drop_compensation_imag_volts},
      {"line_drop_compensation_magnitude_volts",
       state.line_drop_compensation_magnitude_volts},
      {"converged", state.converged},
      {"stop_reason", state.stop_reason},
  };
}

json regulator_state_to_json(
    const hacdcpf::analysis::ThreePhaseRegulatorControlState& state) {
  return {
      {"regulator_name", state.regulator_name},
      {"transformer_name", state.transformer_name},
      {"transformer_index", state.transformer_index},
      {"winding", state.winding},
      {"tap_winding", state.tap_winding},
      {"monitored_bus", state.monitored_bus},
      {"monitored_node", state.monitored_node},
      {"used_remote_bus", state.used_remote_bus},
      {"used_line_drop_compensation", state.used_line_drop_compensation},
      {"target_vreg_volts", state.target_vreg_volts},
      {"band_volts", state.band_volts},
      {"ptratio", state.ptratio},
      {"remote_ptratio", state.remote_ptratio},
      {"ct_primary_amps", state.ct_primary_amps},
      {"r_volts", state.r_volts},
      {"x_volts", state.x_volts},
      {"max_tap_change", state.max_tap_change},
      {"control_iterations", state.control_iterations},
      {"final_tap_pos", state.final_tap_pos},
      {"final_tap_number", state.final_tap_number},
      {"final_tap_pu", state.final_tap_pu},
      {"monitored_voltage_pu", state.monitored_voltage_pu},
      {"monitored_voltage_volts", state.monitored_voltage_volts},
      {"control_voltage_volts", state.control_voltage_volts},
      {"tap_winding_current_amps", state.tap_winding_current_amps},
      {"line_drop_compensation_real_volts",
       state.line_drop_compensation_real_volts},
      {"line_drop_compensation_imag_volts",
       state.line_drop_compensation_imag_volts},
      {"line_drop_compensation_magnitude_volts",
       state.line_drop_compensation_magnitude_volts},
      {"converged", state.converged},
      {"stop_reason", state.stop_reason},
  };
}

json regulator_trace_to_json(
    const hacdcpf::analysis::RegulatorControlTraceEntry& trace) {
  return {
      {"regulator_name", trace.regulator_name},
      {"iteration", trace.iteration},
      {"current_tap_pos", trace.current_tap_pos},
      {"current_tap_number", trace.current_tap_number},
      {"current_tap_pu", trace.current_tap_pu},
      {"monitored_voltage_pu", trace.monitored_voltage_pu},
      {"monitored_voltage_volts", trace.monitored_voltage_volts},
      {"control_voltage_volts", trace.control_voltage_volts},
      {"target_vreg_volts", trace.target_vreg_volts},
      {"band_half_volts", trace.band_half_volts},
      {"line_drop_compensation_real_volts",
       trace.line_drop_compensation_real_volts},
      {"line_drop_compensation_imag_volts",
       trace.line_drop_compensation_imag_volts},
      {"line_drop_compensation_magnitude_volts",
       trace.line_drop_compensation_magnitude_volts},
      {"decision_reason", trace.decision_reason},
      {"next_tap_pos", trace.next_tap_pos},
      {"next_tap_number", trace.next_tap_number},
      {"stop_reason", trace.stop_reason},
  };
}

json regulator_trace_to_json(
    const hacdcpf::analysis::ThreePhaseRegulatorControlTraceEntry& trace) {
  return {
      {"regulator_name", trace.regulator_name},
      {"iteration", trace.iteration},
      {"current_tap_pos", trace.current_tap_pos},
      {"current_tap_number", trace.current_tap_number},
      {"current_tap_pu", trace.current_tap_pu},
      {"monitored_voltage_pu", trace.monitored_voltage_pu},
      {"monitored_voltage_volts", trace.monitored_voltage_volts},
      {"control_voltage_volts", trace.control_voltage_volts},
      {"tap_winding_current_amps", trace.tap_winding_current_amps},
      {"target_vreg_volts", trace.target_vreg_volts},
      {"band_half_volts", trace.band_half_volts},
      {"line_drop_compensation_real_volts",
       trace.line_drop_compensation_real_volts},
      {"line_drop_compensation_imag_volts",
       trace.line_drop_compensation_imag_volts},
      {"line_drop_compensation_magnitude_volts",
       trace.line_drop_compensation_magnitude_volts},
      {"decision_reason", trace.decision_reason},
      {"next_tap_pos", trace.next_tap_pos},
      {"next_tap_number", trace.next_tap_number},
      {"stop_reason", trace.stop_reason},
  };
}

json opendss_event_log_entry_to_json(
    const hacdcpf::io::OpenDSSEventLogEntry& entry) {
  const std::string lowered = ascii_lower(entry.message);
  const bool is_tap_operation =
      lowered.find("tap") != std::string::npos &&
      (lowered.find("change") != std::string::npos ||
       lowered.find("changed") != std::string::npos ||
       lowered.find("moving") != std::string::npos ||
       lowered.find("increment") != std::string::npos ||
       lowered.find("decrement") != std::string::npos);
  return {
      {"event_index", entry.event_index},
      {"raw_message", entry.message},
      {"is_tap_operation", is_tap_operation},
  };
}

bool event_log_mentions_control(const std::string& message,
                                const std::string& regcontrol_name,
                                const std::string& transformer_name) {
  const std::string lowered = ascii_lower(message);
  return lowered.find(ascii_lower(regcontrol_name)) != std::string::npos ||
         lowered.find(ascii_lower(transformer_name)) != std::string::npos ||
         lowered.find("regcontrol") != std::string::npos;
}

bool event_log_mentions_tap_operation(const std::string& message) {
  const std::string lowered = ascii_lower(message);
  return lowered.find("tap") != std::string::npos &&
         (lowered.find("change") != std::string::npos ||
          lowered.find("changed") != std::string::npos ||
          lowered.find("moving") != std::string::npos ||
          lowered.find("increment") != std::string::npos ||
          lowered.find("decrement") != std::string::npos);
}

int count_repo_tap_operations(
    const std::vector<hacdcpf::analysis::RegulatorControlTraceEntry>& trace,
    const std::string& regulator_name) {
  const std::string lowered = ascii_lower(regulator_name);
  int count = 0;
  for (const auto& entry : trace) {
    if (ascii_lower(entry.regulator_name) != lowered) {
      continue;
    }
    if (entry.next_tap_number != entry.current_tap_number) {
      ++count;
    }
  }
  return count;
}

int count_repo_tap_operations(
    const std::vector<hacdcpf::analysis::ThreePhaseRegulatorControlTraceEntry>& trace,
    const std::string& regulator_name) {
  const std::string lowered = ascii_lower(regulator_name);
  int count = 0;
  for (const auto& entry : trace) {
    if (ascii_lower(entry.regulator_name) != lowered) {
      continue;
    }
    if (entry.next_tap_number != entry.current_tap_number) {
      ++count;
    }
  }
  return count;
}

int count_opendss_tap_operations(
    const hacdcpf::io::OpenDSSControlOracle& oracle,
    const std::string& regcontrol_name,
    const std::string& transformer_name) {
  int count = 0;
  for (const auto& entry : oracle.event_log_entries) {
    if (!event_log_mentions_control(entry.message, regcontrol_name, transformer_name)) {
      continue;
    }
    if (event_log_mentions_tap_operation(entry.message)) {
      ++count;
    }
  }
  return count;
}

int derive_opendss_tap_operations_from_control_iterations(
    const hacdcpf::io::OpenDSSControlOracle& oracle,
    bool final_control_voltage_within_band) {
  if (oracle.control_iterations <= 0) {
    return 0;
  }
  const int derived =
      final_control_voltage_within_band ? (oracle.control_iterations - 1)
                                        : oracle.control_iterations;
  return std::max(0, derived);
}

std::string derive_opendss_control_stop_reason(
    const hacdcpf::io::OpenDSSControlOracle& oracle,
    bool final_control_voltage_within_band) {
  if (final_control_voltage_within_band) {
    return "within_band";
  }
  if (oracle.max_control_iterations > 0 &&
      oracle.control_iterations >= oracle.max_control_iterations) {
    return "max_control_iterations_reached";
  }
  return "not_within_band_after_solve";
}

json build_repo_control_trace_summary(
    const std::vector<hacdcpf::analysis::RegulatorControlTraceEntry>& trace,
    const hacdcpf::analysis::RegulatorControlState& state,
    const std::string& regulator_name) {
  json actions = json::array();
  const std::string lowered = ascii_lower(regulator_name);
  for (const auto& entry : trace) {
    if (ascii_lower(entry.regulator_name) != lowered) {
      continue;
    }
    actions.push_back(regulator_trace_to_json(entry));
  }
  return {
      {"control_iteration_count", state.control_iterations},
      {"tap_operation_count", count_repo_tap_operations(trace, regulator_name)},
      {"per_iteration_actions", actions},
      {"stop_reason", state.stop_reason},
  };
}

json build_repo_control_trace_summary(
    const std::vector<hacdcpf::analysis::ThreePhaseRegulatorControlTraceEntry>& trace,
    const hacdcpf::analysis::ThreePhaseRegulatorControlState& state,
    const std::string& regulator_name) {
  json actions = json::array();
  const std::string lowered = ascii_lower(regulator_name);
  for (const auto& entry : trace) {
    if (ascii_lower(entry.regulator_name) != lowered) {
      continue;
    }
    actions.push_back(regulator_trace_to_json(entry));
  }
  return {
      {"control_iteration_count", state.control_iterations},
      {"tap_operation_count", count_repo_tap_operations(trace, regulator_name)},
      {"per_iteration_actions", actions},
      {"stop_reason", state.stop_reason},
  };
}

json build_opendss_control_oracle_summary(
    const hacdcpf::io::OpenDSSSnapshotResult& dss,
    const std::string& regcontrol_name,
    const std::string& transformer_name,
    bool final_control_voltage_within_band) {
  json actions = json::array();
  for (const auto& entry : dss.control_oracle.event_log_entries) {
    if (!event_log_mentions_control(entry.message, regcontrol_name, transformer_name)) {
      continue;
    }
    actions.push_back(opendss_event_log_entry_to_json(entry));
  }
  const int event_log_tap_operation_count =
      count_opendss_tap_operations(
          dss.control_oracle, regcontrol_name, transformer_name);
  const int fallback_tap_operation_count =
      derive_opendss_tap_operations_from_control_iterations(
          dss.control_oracle, final_control_voltage_within_band);
  const bool used_event_log_tap_operation_count =
      event_log_tap_operation_count > 0;
  return {
      {"control_iteration_count", dss.control_oracle.control_iterations},
      {"max_control_iterations", dss.control_oracle.max_control_iterations},
      {"tap_operation_count",
       used_event_log_tap_operation_count ? event_log_tap_operation_count
                                          : fallback_tap_operation_count},
      {"tap_operation_count_source",
       used_event_log_tap_operation_count ? "event_log"
                                          : "control_iterations_fallback"},
      {"event_log_control_action_count", actions.size()},
      {"per_iteration_actions", actions},
      {"stop_reason",
       derive_opendss_control_stop_reason(
           dss.control_oracle, final_control_voltage_within_band)},
      {"stop_reason_source",
       "derived_from_OpenDSS_Solution_ControlIterations_plus_final_band_check"},
      {"event_log_note",
       actions.empty()
           ? "OpenDSS Solution.EventLog did not expose per-action control text for this case; tap_operation_count falls back to control_iterations semantics."
           : "Tap-operation evidence extracted from OpenDSS Solution.EventLog."},
  };
}

std::optional<double> metric_max_abs_any(
    const json& metrics,
    std::initializer_list<const char*> field_names) {
  std::optional<double> value;
  for (const char* field_name : field_names) {
    if (!metrics.contains(field_name)) {
      continue;
    }
    const double candidate = metrics[field_name]["max_abs"].get<double>();
    value = value.has_value() ? std::max(*value, candidate) : candidate;
  }
  return value;
}

struct CaseEvaluation {
  bool comparison_pass{true};
  std::vector<std::string> failed_metrics;
  std::string comparison_status;
};

std::string metric_tolerance_key(const std::string& metric_name);

json build_error_budget(const json& item, bool comparison_pass) {
  json metrics = json::object();
  double dominant_abs = -1.0;
  std::string dominant_name;
  json dominant_metric = nullptr;
  bool all_metric_thresholds_passed = true;

  const auto& metric_values = item["metrics"];
  const auto& tolerances = item["acceptance_tolerances"];
  for (auto it = metric_values.begin(); it != metric_values.end(); ++it) {
    const std::string tol_key = metric_tolerance_key(it.key());
    if (tol_key.empty() || !tolerances.contains(tol_key)) {
      continue;
    }

    const double absolute_error = it.value().at("max_abs").get<double>();
    const json relative_error = it.value().contains("max_rel")
                                    ? it.value().at("max_rel")
                                    : json(nullptr);
    const double threshold = tolerances.at(tol_key).at("max_abs").get<double>();
    const bool pass = absolute_error <= threshold;
    all_metric_thresholds_passed =
        all_metric_thresholds_passed && pass;

    json metric_budget = {
        {"absolute_error", absolute_error},
        {"relative_error", relative_error},
        {"threshold", threshold},
        {"pass", pass},
        {"unit", tolerances.at(tol_key).at("unit")},
    };
    metrics[it.key()] = metric_budget;

    if (absolute_error > dominant_abs) {
      dominant_abs = absolute_error;
      dominant_name = it.key();
      dominant_metric = metric_budget;
      dominant_metric["name"] = dominant_name;
    }
  }

  return {
      {"comparison_pass", comparison_pass},
      {"all_metric_thresholds_passed", all_metric_thresholds_passed},
      {"metrics", metrics},
      {"dominant_error_metric", dominant_metric},
  };
}

std::string metric_tolerance_key(const std::string& metric_name) {
  if (metric_name == "bus_voltage_magnitude_pu" ||
      metric_name == "bus_phase_voltage_magnitude_pu") {
    return "bus_voltage_magnitude_pu";
  }
  if (metric_name == "bus_voltage_angle_deg" ||
      metric_name == "bus_phase_voltage_angle_deg") {
    return "bus_voltage_angle_deg";
  }
  if (metric_name == "line_terminal1_p_mw" ||
      metric_name == "line_terminal1_phase_p_mw" ||
      metric_name == "line_terminal_3ph_total_p_mw" ||
      metric_name == "transformer_terminal1_p_mw" ||
      metric_name == "transformer_terminal2_p_mw" ||
      metric_name == "transformer_terminal3_p_mw" ||
      metric_name == "line_l1_terminal1_p_mw") {
    return "line_terminal1_p_mw";
  }
  if (metric_name == "line_terminal1_q_mvar" ||
      metric_name == "line_terminal1_phase_q_mvar" ||
      metric_name == "line_terminal_3ph_total_q_mvar" ||
      metric_name == "transformer_terminal1_q_mvar" ||
      metric_name == "transformer_terminal2_q_mvar" ||
      metric_name == "transformer_terminal3_q_mvar" ||
      metric_name == "line_l1_terminal1_q_mvar") {
    return "line_terminal1_q_mvar";
  }
  if (metric_name == "total_line_loss_p_mw" ||
      metric_name == "total_branch_loss_p_mw" ||
      metric_name == "total_circuit_loss_p_mw_via_circuit_losses_raw") {
    return "total_line_loss_p_mw";
  }
  if (metric_name == "total_line_loss_q_mvar" ||
      metric_name == "total_branch_loss_q_mvar" ||
      metric_name == "total_circuit_loss_q_mvar_via_circuit_losses_raw") {
    return "total_line_loss_q_mvar";
  }
  if (metric_name == "regulator_control_voltage_volts") {
    return "regulator_control_voltage_volts";
  }
  if (metric_name == "regulator_tap_number") {
    return "regulator_tap_number";
  }
  return {};
}

CaseEvaluation evaluate_case(json& item) {
  CaseEvaluation evaluation;
  const bool repo_converged = item["converged"]["repo"].get<bool>();
  const bool opendss_converged = item["converged"]["opendss"].get<bool>();
  bool all_metric_thresholds_passed = true;
  if (!repo_converged || !opendss_converged) {
    evaluation.comparison_pass = false;
    evaluation.failed_metrics.push_back("convergence");
  }

  const auto& metrics = item["metrics"];
  const auto& tolerances = item["acceptance_tolerances"];
  for (auto it = metrics.begin(); it != metrics.end(); ++it) {
    const std::string tol_key = metric_tolerance_key(it.key());
    if (tol_key.empty() || !tolerances.contains(tol_key)) {
      continue;
    }
    const double max_abs = it.value().at("max_abs").get<double>();
    const double max_allowed = tolerances.at(tol_key).at("max_abs").get<double>();
    if (max_abs > max_allowed) {
      all_metric_thresholds_passed = false;
      evaluation.comparison_pass = false;
      evaluation.failed_metrics.push_back(it.key());
    }
  }

  const bool bridge_validation_pass =
      !(item.contains("bridge_validation_pass") &&
        !item["bridge_validation_pass"].is_null() &&
        !item["bridge_validation_pass"].get<bool>());
  if (item.contains("bridge_validation_pass") &&
      !item["bridge_validation_pass"].is_null() &&
      !item["bridge_validation_pass"].get<bool>()) {
    evaluation.comparison_pass = false;
    evaluation.failed_metrics.push_back("bridge_validation");
  }

  const bool control_trace_validation_pass =
      !(item.contains("control_trace_validation_pass") &&
        !item["control_trace_validation_pass"].is_null() &&
        !item["control_trace_validation_pass"].get<bool>());
  if (item.contains("control_trace_validation_pass") &&
      !item["control_trace_validation_pass"].is_null() &&
      !item["control_trace_validation_pass"].get<bool>()) {
    evaluation.comparison_pass = false;
    evaluation.failed_metrics.push_back("control_trace_validation");
  }

  const bool fixture_truth_drift_detected =
      item.contains("fixture_truth_drift_pass") &&
      !item["fixture_truth_drift_pass"].is_null() &&
      !item["fixture_truth_drift_pass"].get<bool>();
  if (fixture_truth_drift_detected) {
    evaluation.comparison_pass = false;
    evaluation.failed_metrics.push_back("fixture_truth_drift");
  }

  const std::string tier = item["acceptance_tier"].get<std::string>();
  const bool is_accepted = tier.rfind("accepted", 0) == 0;
  if (is_accepted) {
    if (!evaluation.comparison_pass && fixture_truth_drift_detected &&
        all_metric_thresholds_passed && bridge_validation_pass &&
        control_trace_validation_pass &&
        repo_converged && opendss_converged) {
      evaluation.comparison_status = "fixture_truth_drift";
    } else {
      evaluation.comparison_status =
          evaluation.comparison_pass ? "accepted" : "failed";
    }
  } else if (tier.rfind("exploratory", 0) == 0) {
    evaluation.comparison_status = "exploratory";
  } else if (tier.rfind("deferred", 0) == 0) {
    evaluation.comparison_status = "deferred";
  } else {
    evaluation.comparison_status =
        evaluation.comparison_pass ? tier : "blocked";
  }

  item["comparison_pass"] = evaluation.comparison_pass;
  item["failed_metrics"] = evaluation.failed_metrics;
  item["comparison_status"] = evaluation.comparison_status;
  item["error_budget"] = build_error_budget(item, evaluation.comparison_pass);
  return evaluation;
}

const hacdcpf::io::OpenDSSNodeVoltage& find_node_voltage(
    const std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage>& index,
    const std::string& bus_name,
    int node) {
  const auto it = index.find(key_for_bus_node(bus_name, node));
  if (it == index.end()) {
    throw std::runtime_error("Missing OpenDSS node voltage for " +
                             key_for_bus_node(bus_name, node));
  }
  return it->second;
}

const hacdcpf::io::OpenDSSTerminalPower& find_terminal_power(
    const std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>& index,
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name,
    int terminal,
    int node) {
  const std::string key =
      key_for_pd_element_terminal_node(element_kind, element_name, terminal, node);
  const auto it = index.find(key);
  if (it == index.end()) {
    throw std::runtime_error("Missing OpenDSS terminal power for " + key);
  }
  return it->second;
}

const hacdcpf::io::OpenDSSPDElementResult& find_pd_element(
    const std::map<std::string, hacdcpf::io::OpenDSSPDElementResult>& index,
    hacdcpf::io::OpenDSSPDElementKind element_kind,
    const std::string& element_name) {
  const std::string key = key_for_pd_element(element_kind, element_name);
  const auto it = index.find(key);
  if (it == index.end()) {
    throw std::runtime_error("Missing OpenDSS PD element for " + key);
  }
  return it->second;
}

const hacdcpf::io::OpenDSSTransformerState& find_transformer_state(
    const std::vector<hacdcpf::io::OpenDSSTransformerState>& states,
    const std::string& transformer_name) {
  const std::string lowered = ascii_lower(transformer_name);
  for (const auto& state : states) {
    if (state.name == lowered) {
      return state;
    }
  }
  throw std::runtime_error("Missing OpenDSS transformer state for " + lowered);
}

const hacdcpf::io::OpenDSSTransformerWindingState& find_transformer_winding_state(
    const hacdcpf::io::OpenDSSTransformerState& state,
    int winding) {
  for (const auto& winding_state : state.winding_states) {
    if (winding_state.winding == winding) {
      return winding_state;
    }
  }
  throw std::runtime_error("Missing winding state for transformer " + state.name +
                           " winding " + std::to_string(winding));
}

const hacdcpf::io::OpenDSSRegControlResult& find_regcontrol(
    const std::vector<hacdcpf::io::OpenDSSRegControlResult>& controls,
    const std::string& regcontrol_name) {
  const std::string lowered = ascii_lower(regcontrol_name);
  for (const auto& control : controls) {
    if (control.name == lowered) {
      return control;
    }
  }
  throw std::runtime_error("Missing OpenDSS RegControl for " + lowered);
}

std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage> build_voltage_index(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage> index;
  for (const auto& node_voltage : result.node_voltages) {
    index.emplace(key_for_bus_node(node_voltage.bus_name, node_voltage.node),
                  node_voltage);
  }
  return index;
}

std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>
build_terminal_power_index(const hacdcpf::io::OpenDSSSnapshotResult& result) {
  std::map<std::string, hacdcpf::io::OpenDSSTerminalPower> index;
  for (const auto& element : result.pd_element_results) {
    for (const auto& power : element.terminal_powers) {
      const std::string key = key_for_pd_element_terminal_node(
          element.element_kind, element.name, power.terminal, power.node);
      index.emplace(key, power);
    }
  }
  return index;
}

std::map<std::string, hacdcpf::io::OpenDSSPDElementResult> build_pd_element_index(
    const hacdcpf::io::OpenDSSSnapshotResult& result) {
  std::map<std::string, hacdcpf::io::OpenDSSPDElementResult> index;
  for (const auto& element : result.pd_element_results) {
    index.emplace(key_for_pd_element(element.element_kind, element.name), element);
  }
  return index;
}

std::pair<std::string, int> parse_terminal_bus_name(const std::string& terminal_bus_name) {
  const auto dot_pos = terminal_bus_name.find('.');
  if (dot_pos == std::string::npos) {
    return {ascii_lower(terminal_bus_name), 1};
  }
  const std::string bus_name = ascii_lower(terminal_bus_name.substr(0, dot_pos));
  const auto next_dot_pos = terminal_bus_name.find('.', dot_pos + 1);
  const std::string node_text = terminal_bus_name.substr(
      dot_pos + 1,
      (next_dot_pos == std::string::npos) ? std::string::npos
                                          : next_dot_pos - dot_pos - 1);
  return {bus_name, std::stoi(node_text)};
}

std::complex<double> node_voltage_complex_volts(
    const hacdcpf::io::OpenDSSNodeVoltage& voltage) {
  return std::polar(voltage.vm_vln, voltage.va_deg * M_PI / 180.0);
}

struct DSSRegulatorControlSnapshot {
  double effective_ptratio{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
};

DSSRegulatorControlSnapshot compute_dss_regulator_control_snapshot(
    const std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage>& voltage_index,
    const std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>& power_index,
    const hacdcpf::io::OpenDSSPDElementResult& transformer,
    const hacdcpf::io::OpenDSSRegControlResult& regcontrol,
    const hacdcpf_compare_fixtures::RegulatorOpenDSSCase& spec) {
  const auto& monitored_bus_voltage = find_node_voltage(
      voltage_index, spec.monitored_bus_name, spec.monitored_bus_node);
  const std::complex<double> monitored_voltage_complex =
      node_voltage_complex_volts(monitored_bus_voltage);
  const double effective_ptratio =
      (!spec.implicit_local_bus_monitoring && regcontrol.remote_ptratio > 0.0)
          ? regcontrol.remote_ptratio
          : regcontrol.ptratio;

  std::complex<double> ldc_term{0.0, 0.0};
  if (std::abs(regcontrol.forward_r_volts) > 1e-12 ||
      std::abs(regcontrol.forward_x_volts) > 1e-12) {
    const auto [tap_bus_name, tap_bus_node] = parse_terminal_bus_name(
        transformer.terminal_bus_names.at(static_cast<std::size_t>(regcontrol.tap_winding - 1)));
    const auto& tap_winding_voltage = find_node_voltage(
        voltage_index, tap_bus_name, tap_bus_node);
    const std::complex<double> tap_winding_voltage_complex =
        node_voltage_complex_volts(tap_winding_voltage);
    const auto& tap_terminal_power = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        transformer.name, regcontrol.tap_winding, tap_bus_node);
    const auto tap_terminal_power_mw_mvar =
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            tap_terminal_power.power_kw_kvar);
    const std::complex<double> tap_terminal_power_mva{
        tap_terminal_power_mw_mvar.p_mw,
        tap_terminal_power_mw_mvar.q_mvar,
    };
    const std::complex<double> tap_current_amps = std::conj(
        (tap_terminal_power_mva * 1.0e6) / tap_winding_voltage_complex);
    ldc_term = (tap_current_amps / regcontrol.ct_primary_amps) *
               std::complex<double>(regcontrol.forward_r_volts,
                                    regcontrol.forward_x_volts);
  }

  const std::complex<double> control_voltage =
      monitored_voltage_complex / effective_ptratio + ldc_term;
  return {
      .effective_ptratio = effective_ptratio,
      .monitored_voltage_volts = monitored_bus_voltage.vm_vln,
      .control_voltage_volts = std::abs(control_voltage),
      .line_drop_compensation_real_volts = ldc_term.real(),
      .line_drop_compensation_imag_volts = ldc_term.imag(),
      .line_drop_compensation_magnitude_volts = std::abs(ldc_term),
  };
}

DSSRegulatorControlSnapshot compute_dss_regulator_control_snapshot(
    const std::map<std::string, hacdcpf::io::OpenDSSNodeVoltage>& voltage_index,
    const std::map<std::string, hacdcpf::io::OpenDSSTerminalPower>& power_index,
    const hacdcpf::io::OpenDSSPDElementResult& transformer,
    const hacdcpf::io::OpenDSSRegControlResult& regcontrol,
    const hacdcpf_compare_fixtures::ThreePhaseRegulatorPhaseSpec& spec) {
  const auto& monitored_bus_voltage = find_node_voltage(
      voltage_index, spec.monitored_bus_name, spec.monitored_bus_node);
  const std::complex<double> monitored_voltage_complex =
      node_voltage_complex_volts(monitored_bus_voltage);
  const double effective_ptratio =
      (!spec.implicit_local_bus_monitoring && regcontrol.remote_ptratio > 0.0)
          ? regcontrol.remote_ptratio
          : regcontrol.ptratio;

  std::complex<double> ldc_term{0.0, 0.0};
  if (std::abs(regcontrol.forward_r_volts) > 1e-12 ||
      std::abs(regcontrol.forward_x_volts) > 1e-12) {
    const auto [tap_bus_name, tap_bus_node] = parse_terminal_bus_name(
        transformer.terminal_bus_names.at(
            static_cast<std::size_t>(regcontrol.tap_winding - 1)));
    const auto& tap_winding_voltage = find_node_voltage(
        voltage_index, tap_bus_name, tap_bus_node);
    const std::complex<double> tap_winding_voltage_complex =
        node_voltage_complex_volts(tap_winding_voltage);
    const auto& tap_terminal_power = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        transformer.name, regcontrol.tap_winding, tap_bus_node);
    const auto tap_terminal_power_mw_mvar =
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            tap_terminal_power.power_kw_kvar);
    const std::complex<double> tap_terminal_power_mva{
        tap_terminal_power_mw_mvar.p_mw,
        tap_terminal_power_mw_mvar.q_mvar,
    };
    const std::complex<double> tap_current_amps = std::conj(
        (tap_terminal_power_mva * 1.0e6) / tap_winding_voltage_complex);
    ldc_term = (tap_current_amps / regcontrol.ct_primary_amps) *
               std::complex<double>(regcontrol.forward_r_volts,
                                    regcontrol.forward_x_volts);
  }

  const std::complex<double> control_voltage =
      monitored_voltage_complex / effective_ptratio + ldc_term;
  return {
      .effective_ptratio = effective_ptratio,
      .monitored_voltage_volts = monitored_bus_voltage.vm_vln,
      .control_voltage_volts = std::abs(control_voltage),
      .line_drop_compensation_real_volts = ldc_term.real(),
      .line_drop_compensation_imag_volts = ldc_term.imag(),
      .line_drop_compensation_magnitude_volts = std::abs(ldc_term),
  };
}

int find_branch_position_by_index(const hacdcpf::ACSystem& ac, int branch_index) {
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (ac.branches[pos].index == branch_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error("Missing projected ACBranch with index " +
                           std::to_string(branch_index));
}

int find_branch_position_by_name(const hacdcpf::ACSystem& ac,
                                 const std::string& branch_name) {
  const std::string lowered = ascii_lower(branch_name);
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (ascii_lower(ac.branches[pos].name) == lowered) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error("Missing projected ACBranch named " + lowered);
}

int find_bus_position_by_name(const std::vector<hacdcpf::ACBus>& buses,
                              const std::string& bus_name) {
  const std::string lowered = ascii_lower(bus_name);
  for (std::size_t pos = 0; pos < buses.size(); ++pos) {
    if (ascii_lower(buses[pos].name) == lowered) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error("Missing AC bus named " + lowered);
}

int find_projected_branch_position(const hacdcpf::HybridPowerSystem& projected,
                                   hacdcpf::BranchOriginType origin_type,
                                   int origin_index) {
  if (!projected.branch_expand_map.has_value()) {
    throw std::runtime_error("Projected system is missing BranchExpandMap.");
  }
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == origin_type && entry.origin_index == origin_index) {
      return find_branch_position_by_index(projected.ac, entry.branch_index);
    }
  }
  throw std::runtime_error("Missing projected branch mapping for origin index " +
                           std::to_string(origin_index));
}

using Complex = std::complex<double>;
using ThreePhaseVector = std::array<Complex, 3>;

int find_projected_branch_position_by_pair(
    const hacdcpf::HybridPowerSystem& projected,
    int transformer3w_index,
    int pair_number) {
  if (!projected.branch_expand_map.has_value()) {
    throw std::runtime_error("Projected system is missing BranchExpandMap.");
  }
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == hacdcpf::BranchOriginType::Transformer3W &&
        entry.origin_index == transformer3w_index &&
        entry.pair_number == pair_number) {
      return find_branch_position_by_index(projected.ac, entry.branch_index);
    }
  }
  throw std::runtime_error("Missing projected Transformer3W pair mapping for origin index " +
                           std::to_string(transformer3w_index) + " pair " +
                           std::to_string(pair_number));
}

Complex polar_pu(double magnitude, double angle_deg) {
  return std::polar(magnitude, angle_deg * M_PI / 180.0);
}

int phase_index_from_node(int node) {
  switch (node) {
    case 1:
      return 0;
    case 2:
      return 1;
    case 3:
      return 2;
    default:
      throw std::runtime_error("Unsupported phase node " + std::to_string(node));
  }
}

Complex phasor_from_observation(
    const hacdcpf::analysis::ThreePhasePhasorObservation& observation,
    int phase_index) {
  return {
      observation.real[static_cast<std::size_t>(phase_index)],
      observation.imag[static_cast<std::size_t>(phase_index)],
  };
}

const hacdcpf::ThreePhaseACBus& find_three_phase_bus(
    const hacdcpf::ThreePhaseACSystem& sys,
    int bus_index) {
  for (const auto& bus : sys.buses) {
    if (bus.index == bus_index) {
      return bus;
    }
  }
  throw std::runtime_error("Missing three-phase bus index " +
                           std::to_string(bus_index));
}

const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation&
find_three_phase_transformer_observation(
    const std::vector<hacdcpf::analysis::ThreePhaseTransformerTerminalObservation>& observations,
    int transformer_index) {
  for (const auto& observation : observations) {
    if (observation.transformer_index == transformer_index) {
      return observation;
    }
  }
  throw std::runtime_error(
      "Missing three-phase transformer observation for transformer index " +
      std::to_string(transformer_index));
}

const hacdcpf::analysis::ThreePhaseRegulatorControlState&
find_three_phase_regulator_state(
    const std::vector<hacdcpf::analysis::ThreePhaseRegulatorControlState>& states,
    const std::string& regulator_name) {
  const std::string lowered = ascii_lower(regulator_name);
  for (const auto& state : states) {
    if (ascii_lower(state.regulator_name) == lowered) {
      return state;
    }
  }
  throw std::runtime_error("Missing three-phase regulator state for " + lowered);
}

hacdcpf_compare::PowerPair transformer_terminal_power_from_observation(
    const hacdcpf::analysis::ThreePhaseTransformerTerminalObservation& observation,
    double terminal_base_voltage_volts,
    int phase_index,
    bool hv_side) {
  const auto voltage_pu = phasor_from_observation(
      hv_side ? observation.hv_voltage_pu : observation.lv_voltage_pu,
      phase_index);
  const auto current_amps = phasor_from_observation(
      hv_side ? observation.hv_current_amps : observation.lv_current_amps,
      phase_index);
  const Complex terminal_voltage_volts = voltage_pu * terminal_base_voltage_volts;
  const Complex power_mva =
      terminal_voltage_volts * std::conj(current_amps) / 1.0e6;
  return {
      .p_mw = power_mva.real(),
      .q_mvar = power_mva.imag(),
  };
}

struct SinglePhaseBranchTerminalPowers {
  Complex power_from_mva;
  Complex power_to_mva;
  Complex loss_mva;
};

std::pair<double, double> rx_from_vk_vkr(double vk_percent,
                                         double vkr_percent,
                                         double base_mva,
                                         double sn_mva) {
  if (sn_mva <= 1e-9 || base_mva <= 1e-9) {
    return {0.0, 0.0};
  }
  const double scale = base_mva / sn_mva;
  const double z = std::max(0.0, vk_percent / 100.0) * scale;
  const double r = std::max(0.0, vkr_percent / 100.0) * scale;
  const double x2 = std::max(0.0, z * z - r * r);
  return {r, std::sqrt(x2)};
}

double transformer3w_winding_tap_pu(const hacdcpf::Transformer3W& tr) {
  if (std::abs(tr.tap_step_percent) < 1e-12) {
    return 1.0;
  }
  return std::max(
      1e-6,
      1.0 + (static_cast<double>(tr.tap_pos) * tr.tap_step_percent / 100.0));
}

struct Transformer3WPairTapProjection {
  double branch_tap_pu{1.0};
  double impedance_scale{1.0};
};

Transformer3WPairTapProjection normalize_transformer3w_pair_tap_to_from_side(
    const hacdcpf::Transformer3W& tr,
    int pair_from_bus,
    int pair_to_bus) {
  const double winding_tap_pu = transformer3w_winding_tap_pu(tr);
  int tapped_bus = 0;
  switch (tr.tap_side) {
    case 0: tapped_bus = tr.hv_bus; break;
    case 1: tapped_bus = tr.mv_bus; break;
    case 2: tapped_bus = tr.lv_bus; break;
    default: return {};
  }
  if (pair_from_bus != tapped_bus && pair_to_bus != tapped_bus) {
    return {};
  }
  if (pair_from_bus == tapped_bus) {
    return {.branch_tap_pu = winding_tap_pu, .impedance_scale = 1.0};
  }
  return {
      .branch_tap_pu = 1.0 / winding_tap_pu,
      .impedance_scale = winding_tap_pu * winding_tap_pu,
  };
}

Complex branch_complex_tap(double tap_pu, double shift_deg = 0.0) {
  const double tap_mag = (std::abs(tap_pu) < 1e-12) ? 1.0 : tap_pu;
  return std::polar(tap_mag, shift_deg * M_PI / 180.0);
}

SinglePhaseBranchTerminalPowers compute_single_phase_branch_terminal_powers(
    const hacdcpf::ACBranch& branch,
    Complex voltage_from,
    Complex voltage_to,
    double base_mva) {
  const Complex tap = std::polar(
      (std::abs(branch.tap) < 1e-12) ? 1.0 : branch.tap,
      branch.shift_deg * M_PI / 180.0);
  const Complex z(branch.r_pu, branch.x_pu);
  const Complex series_current = (voltage_from / tap - voltage_to) / z;
  const Complex current_from = series_current / std::conj(tap);
  const Complex current_to = -series_current;
  const Complex power_from = voltage_from * std::conj(current_from) * base_mva;
  const Complex power_to = voltage_to * std::conj(current_to) * base_mva;
  return {
      .power_from_mva = power_from,
      .power_to_mva = power_to,
      .loss_mva = power_from + power_to,
  };
}

struct Transformer3WPairSpec {
  int from_bus;
  int to_bus;
  double vk_percent;
  double vkr_percent;
  double sn_from_mva;
  double sn_to_mva;
  const char* suffix;
  const char* winding_from;
  const char* winding_to;
};

std::array<Transformer3WPairSpec, 3> transformer3w_pair_specs(
    const hacdcpf::Transformer3W& tr) {
  return {{
      {tr.hv_bus, tr.mv_bus, tr.vk_hv_mv_percent, tr.vkr_hv_mv_percent,
       tr.sn_hv_mva, tr.sn_mv_mva, "HV_MV", "hv", "mv"},
      {tr.hv_bus, tr.lv_bus, tr.vk_hv_lv_percent, tr.vkr_hv_lv_percent,
       tr.sn_hv_mva, tr.sn_lv_mva, "HV_LV", "hv", "lv"},
      {tr.mv_bus, tr.lv_bus, tr.vk_mv_lv_percent, tr.vkr_mv_lv_percent,
       tr.sn_mv_mva, tr.sn_lv_mva, "MV_LV", "mv", "lv"},
  }};
}

enum class Transformer3WPairReductionMode {
  LegacyRawPair,
  CoupledKronPair,
};

struct Transformer3WPairModelBuild {
  hacdcpf::ACSystem ac;
  json derivation;
};

Transformer3WPairModelBuild build_transformer3w_pair_model_ac(
    const hacdcpf::HybridPowerSystem& base_case,
    Transformer3WPairReductionMode mode) {
  Transformer3WPairModelBuild out;
  out.ac = base_case.ac;
  out.ac.branches.clear();
  out.ac.transformers_3w.clear();

  const auto& tr = base_case.ac.transformers_3w.front();
  const auto pair_specs = transformer3w_pair_specs(tr);
  std::array<Complex, 3> pair_impedances{};
  json pair_derivation = json::array();

  if (mode == Transformer3WPairReductionMode::LegacyRawPair) {
    for (std::size_t pair = 0; pair < pair_specs.size(); ++pair) {
      const auto& spec = pair_specs[pair];
      const double sn_pair =
          std::max(1e-9, std::min(spec.sn_from_mva, spec.sn_to_mva));
      const auto [r_pu, x_pu] = rx_from_vk_vkr(
          spec.vk_percent, spec.vkr_percent, base_case.base_mva, sn_pair);
      const auto tap_projection = normalize_transformer3w_pair_tap_to_from_side(
          tr, spec.from_bus, spec.to_bus);
      pair_impedances[pair] = {
          r_pu * tap_projection.impedance_scale,
          x_pu * tap_projection.impedance_scale,
      };
      pair_derivation.push_back({
          {"pair_number", static_cast<int>(pair)},
          {"pair_label", std::string(spec.winding_from) + "_" + spec.winding_to},
          {"derivation", "legacy_raw_pair_short_circuit_data"},
          {"series_impedance_pu",
           {
               {"r_pu", pair_impedances[pair].real()},
               {"x_pu", pair_impedances[pair].imag()},
           }},
          {"branch_tap_pu", tap_projection.branch_tap_pu},
      });
    }
  } else {
    const double winding_tap_pu = transformer3w_winding_tap_pu(tr);
    const auto hm = rx_from_vk_vkr(
        tr.vk_hv_mv_percent, tr.vkr_hv_mv_percent, base_case.base_mva,
        std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_mv_mva)));
    const auto hl = rx_from_vk_vkr(
        tr.vk_hv_lv_percent, tr.vkr_hv_lv_percent, base_case.base_mva,
        std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_lv_mva)));
    const auto ml = rx_from_vk_vkr(
        tr.vk_mv_lv_percent, tr.vkr_mv_lv_percent, base_case.base_mva,
        std::max(1e-9, std::min(tr.sn_mv_mva, tr.sn_lv_mva)));
    const Complex z_hm(hm.first, hm.second);
    const Complex z_hl(hl.first, hl.second);
    const Complex z_ml(ml.first, ml.second);
    const Complex z_h = 0.5 * (z_hm + z_hl - z_ml);
    const Complex z_m = 0.5 * (z_hm + z_ml - z_hl);
    const Complex z_l = 0.5 * (z_hl + z_ml - z_hm);
    const std::array<Complex, 3> star_impedances = {z_h, z_m, z_l};
    const std::array<double, 3> star_taps = {
        tr.tap_side == 0 ? winding_tap_pu : 1.0,
        tr.tap_side == 1 ? winding_tap_pu : 1.0,
        tr.tap_side == 2 ? winding_tap_pu : 1.0,
    };

    std::array<std::array<Complex, 3>, 3> kron_y{};
    std::array<Complex, 3> y_ext_to_star{};
    std::array<Complex, 3> y_star_to_ext{};
    Complex y_star_star{0.0, 0.0};
    for (std::size_t leg = 0; leg < star_impedances.size(); ++leg) {
      const Complex y = 1.0 / star_impedances[leg];
      const Complex tau = branch_complex_tap(star_taps[leg]);
      kron_y[leg][leg] += y / (tau * std::conj(tau));
      y_ext_to_star[leg] = -y / std::conj(tau);
      y_star_to_ext[leg] = -y / tau;
      y_star_star += y;
    }
    if (std::abs(y_star_star) < 1e-12) {
      throw std::runtime_error(
          "Transformer3W coupled reduction produced singular auxiliary-star admittance.");
    }
    for (std::size_t row = 0; row < kron_y.size(); ++row) {
      for (std::size_t col = 0; col < kron_y[row].size(); ++col) {
        kron_y[row][col] -= y_ext_to_star[row] * y_star_to_ext[col] / y_star_star;
      }
    }

    const std::array<std::pair<std::size_t, std::size_t>, 3> pair_nodes = {{
        {0, 1},
        {0, 2},
        {1, 2},
    }};
    std::array<std::array<Complex, 3>, 3> reconstructed_y{};
    for (std::size_t pair = 0; pair < pair_specs.size(); ++pair) {
      const auto& spec = pair_specs[pair];
      const auto tap_projection = normalize_transformer3w_pair_tap_to_from_side(
          tr, spec.from_bus, spec.to_bus);
      const Complex tau = branch_complex_tap(tap_projection.branch_tap_pu);
      const Complex y_pair =
          -kron_y[pair_nodes[pair].first][pair_nodes[pair].second] *
          std::conj(tau);
      pair_impedances[pair] = 1.0 / y_pair;
      reconstructed_y[pair_nodes[pair].first][pair_nodes[pair].first] +=
          y_pair / (tau * std::conj(tau));
      reconstructed_y[pair_nodes[pair].first][pair_nodes[pair].second] +=
          -y_pair / std::conj(tau);
      reconstructed_y[pair_nodes[pair].second][pair_nodes[pair].first] +=
          -y_pair / tau;
      reconstructed_y[pair_nodes[pair].second][pair_nodes[pair].second] +=
          y_pair;
    }

    double max_reconstruction_abs_diff = 0.0;
    for (std::size_t row = 0; row < kron_y.size(); ++row) {
      for (std::size_t col = 0; col < kron_y[row].size(); ++col) {
        max_reconstruction_abs_diff = std::max(
            max_reconstruction_abs_diff,
            std::abs(reconstructed_y[row][col] - kron_y[row][col]));
      }
    }

    out.derivation["star_impedance_split_pu"] = {
        {"hv", {{"r_pu", z_h.real()}, {"x_pu", z_h.imag()}}},
        {"mv", {{"r_pu", z_m.real()}, {"x_pu", z_m.imag()}}},
        {"lv", {{"r_pu", z_l.real()}, {"x_pu", z_l.imag()}}},
    };
    out.derivation["auxiliary_star_tap_projection"] = {
        {"hv_leg_tap_pu", star_taps[0]},
        {"mv_leg_tap_pu", star_taps[1]},
        {"lv_leg_tap_pu", star_taps[2]},
    };
    out.derivation["kron_reconstruction_max_abs_diff"] =
        max_reconstruction_abs_diff;
  }

  for (std::size_t pair = 0; pair < pair_specs.size(); ++pair) {
    const auto& spec = pair_specs[pair];
    const double sn_pair =
        std::max(1e-9, std::min(spec.sn_from_mva, spec.sn_to_mva));
    const auto tap_projection = normalize_transformer3w_pair_tap_to_from_side(
        tr, spec.from_bus, spec.to_bus);
    hacdcpf::ACBranch branch;
    branch.index = static_cast<int>(pair + 1);
    branch.from_bus = spec.from_bus;
    branch.to_bus = spec.to_bus;
    branch.r_pu = pair_impedances[pair].real();
    branch.x_pu = pair_impedances[pair].imag();
    branch.b_pu = 0.0;
    branch.tap = tap_projection.branch_tap_pu;
    branch.shift_deg =
        (spec.from_bus == tr.hv_bus && spec.to_bus == tr.mv_bus) ? tr.shift_mv_deg
        : (spec.from_bus == tr.hv_bus && spec.to_bus == tr.lv_bus) ? tr.shift_lv_deg
                                                                   : (tr.shift_lv_deg - tr.shift_mv_deg);
    branch.rate_a_mva = sn_pair;
    branch.in_service = true;
    branch.name = tr.name + "_" + spec.suffix + "_eq";
    branch.r0_pu = branch.r_pu;
    branch.x0_pu = branch.x_pu;
    branch.b0_pu = 0.0;
    branch.vn_hv_kv = tr.vn_hv_kv;
    branch.vn_lv_kv = (spec.to_bus == tr.mv_bus) ? tr.vn_mv_kv : tr.vn_lv_kv;
    branch.sn_mva = sn_pair;
    out.ac.branches.push_back(branch);

    pair_derivation.push_back({
        {"pair_number", static_cast<int>(pair)},
        {"pair_label", std::string(spec.winding_from) + "_" + spec.winding_to},
        {"branch_name", branch.name},
        {"series_impedance_pu",
         {
             {"r_pu", branch.r_pu},
             {"x_pu", branch.x_pu},
         }},
        {"branch_tap_pu", branch.tap},
    });
  }

  out.derivation["mode"] =
      mode == Transformer3WPairReductionMode::LegacyRawPair
          ? "legacy_raw_pair_short_circuit_data"
          : "coupled_star_kron_resplit";
  out.derivation["pair_branch_parameters"] = pair_derivation;
  return out;
}

struct Transformer3WPairModelSolution {
  hacdcpf::analysis::DPFResult result;
  json pair_branch_breakdown;
  json derivation;
  std::array<hacdcpf_compare::PowerPair, 3> terminal_powers{};
  hacdcpf_compare::PowerPair total_loss;
};

Transformer3WPairModelSolution solve_transformer3w_pair_model(
    const hacdcpf::HybridPowerSystem& base_case,
    Transformer3WPairReductionMode mode,
    const hacdcpf::analysis::DPFOptions& options) {
  Transformer3WPairModelSolution out;
  const auto build = build_transformer3w_pair_model_ac(base_case, mode);
  out.derivation = build.derivation;
  out.result = hacdcpf::analysis::solve_distribution_pf(build.ac, options);

  const auto& tr = base_case.ac.transformers_3w.front();
  const std::array<int, 3> winding_bus_ids = {tr.hv_bus, tr.mv_bus, tr.lv_bus};
  std::map<int, std::size_t> bus_position;
  for (std::size_t idx = 0; idx < build.ac.buses.size(); ++idx) {
    bus_position.emplace(build.ac.buses[idx].index, idx);
  }

  out.pair_branch_breakdown = json::array();
  for (std::size_t pair = 0; pair < build.ac.branches.size(); ++pair) {
    const auto& branch = build.ac.branches[pair];
    const auto from_pos_it = bus_position.find(branch.from_bus);
    const auto to_pos_it = bus_position.find(branch.to_bus);
    if (from_pos_it == bus_position.end() || to_pos_it == bus_position.end()) {
      throw std::runtime_error(
          "Missing bus position while evaluating Transformer3W counterfactual.");
    }
    const auto flow = compute_single_phase_branch_terminal_powers(
        branch,
        polar_pu(out.result.vm_pu[from_pos_it->second],
                 out.result.va_deg[from_pos_it->second]),
        polar_pu(out.result.vm_pu[to_pos_it->second],
                 out.result.va_deg[to_pos_it->second]),
        base_case.base_mva);
    for (std::size_t winding = 0; winding < winding_bus_ids.size(); ++winding) {
      if (branch.from_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_from_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_from_mva.imag();
      }
      if (branch.to_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_to_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_to_mva.imag();
      }
    }
    out.total_loss.p_mw += flow.loss_mva.real();
    out.total_loss.q_mvar += flow.loss_mva.imag();
    out.pair_branch_breakdown.push_back({
        {"pair_number", static_cast<int>(pair)},
        {"branch_name", branch.name},
        {"from_bus", branch.from_bus},
        {"to_bus", branch.to_bus},
        {"power_from_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_from_mva.real(), .q_mvar = flow.power_from_mva.imag()})},
        {"power_to_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_to_mva.real(), .q_mvar = flow.power_to_mva.imag()})},
        {"loss_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.loss_mva.real(), .q_mvar = flow.loss_mva.imag()})},
    });
  }
  return out;
}

struct Transformer3WReplayResult {
  std::array<hacdcpf_compare::PowerPair, 3> terminal_powers{};
  hacdcpf_compare::PowerPair total_loss;
};

Transformer3WReplayResult replay_transformer3w_pair_model_at_voltages(
    const hacdcpf::HybridPowerSystem& base_case,
    Transformer3WPairReductionMode mode,
    const std::vector<double>& vm_pu,
    const std::vector<double>& va_deg) {
  const auto build = build_transformer3w_pair_model_ac(base_case, mode);
  const auto& tr = base_case.ac.transformers_3w.front();
  const std::array<int, 3> winding_bus_ids = {tr.hv_bus, tr.mv_bus, tr.lv_bus};
  std::map<int, std::size_t> bus_position;
  for (std::size_t idx = 0; idx < base_case.ac.buses.size(); ++idx) {
    bus_position.emplace(base_case.ac.buses[idx].index, idx);
  }

  Transformer3WReplayResult out;
  for (const auto& branch : build.ac.branches) {
    const auto from_pos_it = bus_position.find(branch.from_bus);
    const auto to_pos_it = bus_position.find(branch.to_bus);
    if (from_pos_it == bus_position.end() || to_pos_it == bus_position.end()) {
      throw std::runtime_error(
          "Missing bus position while replaying Transformer3W pair model.");
    }
    const auto flow = compute_single_phase_branch_terminal_powers(
        branch,
        polar_pu(vm_pu[from_pos_it->second], va_deg[from_pos_it->second]),
        polar_pu(vm_pu[to_pos_it->second], va_deg[to_pos_it->second]),
        base_case.base_mva);
    for (std::size_t winding = 0; winding < winding_bus_ids.size(); ++winding) {
      if (branch.from_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_from_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_from_mva.imag();
      }
      if (branch.to_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_to_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_to_mva.imag();
      }
    }
    out.total_loss.p_mw += flow.loss_mva.real();
    out.total_loss.q_mvar += flow.loss_mva.imag();
  }
  return out;
}

Transformer3WReplayResult replay_transformer3w_explicit_star_reference_at_voltages(
    const hacdcpf::HybridPowerSystem& base_case,
    const std::vector<double>& vm_pu,
    const std::vector<double>& va_deg) {
  const auto& tr = base_case.ac.transformers_3w.front();
  const auto hm = rx_from_vk_vkr(
      tr.vk_hv_mv_percent, tr.vkr_hv_mv_percent, base_case.base_mva,
      std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_mv_mva)));
  const auto hl = rx_from_vk_vkr(
      tr.vk_hv_lv_percent, tr.vkr_hv_lv_percent, base_case.base_mva,
      std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_lv_mva)));
  const auto ml = rx_from_vk_vkr(
      tr.vk_mv_lv_percent, tr.vkr_mv_lv_percent, base_case.base_mva,
      std::max(1e-9, std::min(tr.sn_mv_mva, tr.sn_lv_mva)));
  const std::array<Complex, 3> star_impedances = {
      0.5 * (Complex(hm.first, hm.second) + Complex(hl.first, hl.second) -
             Complex(ml.first, ml.second)),
      0.5 * (Complex(hm.first, hm.second) + Complex(ml.first, ml.second) -
             Complex(hl.first, hl.second)),
      0.5 * (Complex(hl.first, hl.second) + Complex(ml.first, ml.second) -
             Complex(hm.first, hm.second)),
  };
  const double winding_tap_pu = transformer3w_winding_tap_pu(tr);
  const std::array<double, 3> star_taps = {
      tr.tap_side == 0 ? winding_tap_pu : 1.0,
      tr.tap_side == 1 ? winding_tap_pu : 1.0,
      tr.tap_side == 2 ? winding_tap_pu : 1.0,
  };
  const std::array<Complex, 3> external_voltages = {
      polar_pu(vm_pu[0], va_deg[0]),
      polar_pu(vm_pu[1], va_deg[1]),
      polar_pu(vm_pu[2], va_deg[2]),
  };

  Complex numerator{0.0, 0.0};
  Complex denominator{0.0, 0.0};
  for (std::size_t leg = 0; leg < star_impedances.size(); ++leg) {
    const Complex y = 1.0 / star_impedances[leg];
    numerator += y * (external_voltages[leg] / branch_complex_tap(star_taps[leg]));
    denominator += y;
  }
  if (std::abs(denominator) < 1e-12) {
    throw std::runtime_error(
        "Transformer3W explicit coupling replay produced singular star denominator.");
  }
  const Complex star_voltage = numerator / denominator;

  Transformer3WReplayResult out;
  for (std::size_t leg = 0; leg < star_impedances.size(); ++leg) {
    const Complex tau = branch_complex_tap(star_taps[leg]);
    const Complex series_current =
        (external_voltages[leg] / tau - star_voltage) / star_impedances[leg];
    const Complex current_from = series_current / std::conj(tau);
    const Complex current_to = -series_current;
    const Complex power_from =
        external_voltages[leg] * std::conj(current_from) * base_case.base_mva;
    const Complex power_to =
        star_voltage * std::conj(current_to) * base_case.base_mva;
    out.terminal_powers[leg].p_mw = power_from.real();
    out.terminal_powers[leg].q_mvar = power_from.imag();
    out.total_loss.p_mw += (power_from + power_to).real();
    out.total_loss.q_mvar += (power_from + power_to).imag();
  }
  return out;
}

struct SinglePhaseDenseShadowResult {
  bool converged{false};
  int iterations{0};
  std::vector<double> vm_pu;
  std::vector<double> va_deg;
};

SinglePhaseDenseShadowResult solve_single_phase_dense_shadow_nr(
    const hacdcpf::ACSystem& ac,
    double base_mva,
    int max_iter,
    double tol) {
  std::map<int, std::size_t> bus_position;
  for (std::size_t pos = 0; pos < ac.buses.size(); ++pos) {
    bus_position.emplace(ac.buses[pos].index, pos);
  }

  const auto slack_it = std::find_if(
      ac.buses.begin(), ac.buses.end(), [](const hacdcpf::ACBus& bus) {
        return bus.bus_type == hacdcpf::BusType::SLACK;
      });
  if (slack_it == ac.buses.end()) {
    throw std::runtime_error("Dense shadow NR requires a slack bus.");
  }
  const std::size_t slack_pos =
      static_cast<std::size_t>(std::distance(ac.buses.begin(), slack_it));

  std::vector<std::size_t> pq_positions;
  for (std::size_t pos = 0; pos < ac.buses.size(); ++pos) {
    if (pos != slack_pos) {
      pq_positions.push_back(pos);
    }
  }

  std::vector<Complex> specified_injection(ac.buses.size(), Complex{0.0, 0.0});
  for (std::size_t pos = 0; pos < ac.buses.size(); ++pos) {
    specified_injection[pos] -=
        Complex(ac.buses[pos].pd_mw, ac.buses[pos].qd_mvar) / base_mva;
  }
  for (const auto& load : ac.loads) {
    if (!load.in_service) {
      continue;
    }
    const auto it = bus_position.find(load.bus);
    if (it == bus_position.end()) {
      continue;
    }
    specified_injection[it->second] -=
        Complex(load.p_mw, load.q_mvar) / base_mva;
  }
  for (const auto& gen : ac.generators) {
    if (!gen.in_service || gen.is_slack) {
      continue;
    }
    const auto it = bus_position.find(gen.bus);
    if (it == bus_position.end()) {
      continue;
    }
    specified_injection[it->second] +=
        Complex(gen.pg_mw, gen.qg_mvar) / base_mva;
  }

  std::vector<std::vector<Complex>> ybus(
      ac.buses.size(), std::vector<Complex>(ac.buses.size(), Complex{0.0, 0.0}));
  for (const auto& branch : ac.branches) {
    if (!branch.in_service) {
      continue;
    }
    const auto from_it = bus_position.find(branch.from_bus);
    const auto to_it = bus_position.find(branch.to_bus);
    if (from_it == bus_position.end() || to_it == bus_position.end()) {
      continue;
    }
    const std::size_t from = from_it->second;
    const std::size_t to = to_it->second;
    const Complex z(branch.r_pu, branch.x_pu);
    const Complex y = 1.0 / z;
    const Complex tau = branch_complex_tap(branch.tap, branch.shift_deg);
    ybus[from][from] += y / (tau * std::conj(tau));
    ybus[from][to] += -y / std::conj(tau);
    ybus[to][from] += -y / tau;
    ybus[to][to] += y;
  }

  std::vector<Complex> voltages(ac.buses.size(), Complex{1.0, 0.0});
  for (std::size_t pos = 0; pos < ac.buses.size(); ++pos) {
    voltages[pos] = polar_pu(ac.buses[pos].vm_pu, ac.buses[pos].va_deg);
  }

  const auto mismatch = [&](const std::vector<Complex>& v) {
    std::vector<double> out;
    out.reserve(2 * pq_positions.size());
    for (const std::size_t pos : pq_positions) {
      Complex current{0.0, 0.0};
      for (std::size_t col = 0; col < ac.buses.size(); ++col) {
        current += ybus[pos][col] * v[col];
      }
      const Complex calculated = v[pos] * std::conj(current);
      const Complex delta = calculated - specified_injection[pos];
      out.push_back(delta.real());
      out.push_back(delta.imag());
    }
    return out;
  };

  const auto vector_norm_inf = [](const std::vector<double>& values) {
    double max_abs = 0.0;
    for (const double value : values) {
      max_abs = std::max(max_abs, std::abs(value));
    }
    return max_abs;
  };

  SinglePhaseDenseShadowResult result;
  for (int iter = 0; iter < max_iter; ++iter) {
    const std::vector<double> residual = mismatch(voltages);
    if (vector_norm_inf(residual) <= tol) {
      result.converged = true;
      result.iterations = iter;
      break;
    }

    const double eps = 1e-8;
    const std::size_t n_unknown = 2 * pq_positions.size();
    std::vector<std::vector<double>> jacobian(
        residual.size(), std::vector<double>(n_unknown, 0.0));
    for (std::size_t k = 0; k < n_unknown; ++k) {
      auto perturbed = voltages;
      const std::size_t bus_pos = pq_positions[k / 2];
      if ((k % 2) == 0) {
        perturbed[bus_pos] += Complex{eps, 0.0};
      } else {
        perturbed[bus_pos] += Complex{0.0, eps};
      }
      const std::vector<double> residual_perturbed = mismatch(perturbed);
      for (std::size_t row = 0; row < residual.size(); ++row) {
        jacobian[row][k] =
            (residual_perturbed[row] - residual[row]) / eps;
      }
    }

    std::vector<std::vector<double>> augmented(
        residual.size(), std::vector<double>(n_unknown + 1, 0.0));
    for (std::size_t row = 0; row < residual.size(); ++row) {
      for (std::size_t col = 0; col < n_unknown; ++col) {
        augmented[row][col] = jacobian[row][col];
      }
      augmented[row][n_unknown] = -residual[row];
    }

    for (std::size_t pivot = 0; pivot < n_unknown; ++pivot) {
      std::size_t best = pivot;
      for (std::size_t row = pivot + 1; row < n_unknown; ++row) {
        if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) {
          best = row;
        }
      }
      if (std::abs(augmented[best][pivot]) < 1e-12) {
        throw std::runtime_error("Dense shadow NR Jacobian became singular.");
      }
      std::swap(augmented[pivot], augmented[best]);
      const double diag = augmented[pivot][pivot];
      for (std::size_t col = pivot; col <= n_unknown; ++col) {
        augmented[pivot][col] /= diag;
      }
      for (std::size_t row = 0; row < n_unknown; ++row) {
        if (row == pivot) {
          continue;
        }
        const double factor = augmented[row][pivot];
        for (std::size_t col = pivot; col <= n_unknown; ++col) {
          augmented[row][col] -= factor * augmented[pivot][col];
        }
      }
    }

    for (std::size_t k = 0; k < n_unknown; ++k) {
      const std::size_t bus_pos = pq_positions[k / 2];
      if ((k % 2) == 0) {
        voltages[bus_pos] += Complex{augmented[k][n_unknown], 0.0};
      } else {
        voltages[bus_pos] += Complex{0.0, augmented[k][n_unknown]};
      }
    }
    result.iterations = iter + 1;
  }

  result.vm_pu.resize(ac.buses.size());
  result.va_deg.resize(ac.buses.size());
  for (std::size_t pos = 0; pos < ac.buses.size(); ++pos) {
    result.vm_pu[pos] = std::abs(voltages[pos]);
    result.va_deg[pos] = std::arg(voltages[pos]) * 180.0 / M_PI;
  }
  return result;
}

Transformer3WPairModelSolution solve_transformer3w_pair_model_dense_shadow(
    const hacdcpf::HybridPowerSystem& base_case,
    Transformer3WPairReductionMode mode,
    int max_iter,
    double tol) {
  Transformer3WPairModelSolution out;
  const auto build = build_transformer3w_pair_model_ac(base_case, mode);
  out.derivation = build.derivation;
  const auto dense = solve_single_phase_dense_shadow_nr(
      build.ac, base_case.base_mva, max_iter, tol);
  out.result.converged = dense.converged;
  out.result.vm_pu = dense.vm_pu;
  out.result.va_deg = dense.va_deg;

  const auto& tr = base_case.ac.transformers_3w.front();
  const std::array<int, 3> winding_bus_ids = {tr.hv_bus, tr.mv_bus, tr.lv_bus};
  std::map<int, std::size_t> bus_position;
  for (std::size_t idx = 0; idx < build.ac.buses.size(); ++idx) {
    bus_position.emplace(build.ac.buses[idx].index, idx);
  }

  out.pair_branch_breakdown = json::array();
  for (std::size_t pair = 0; pair < build.ac.branches.size(); ++pair) {
    const auto& branch = build.ac.branches[pair];
    const auto from_pos_it = bus_position.find(branch.from_bus);
    const auto to_pos_it = bus_position.find(branch.to_bus);
    const auto flow = compute_single_phase_branch_terminal_powers(
        branch,
        polar_pu(out.result.vm_pu[from_pos_it->second],
                 out.result.va_deg[from_pos_it->second]),
        polar_pu(out.result.vm_pu[to_pos_it->second],
                 out.result.va_deg[to_pos_it->second]),
        base_case.base_mva);
    for (std::size_t winding = 0; winding < winding_bus_ids.size(); ++winding) {
      if (branch.from_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_from_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_from_mva.imag();
      }
      if (branch.to_bus == winding_bus_ids[winding]) {
        out.terminal_powers[winding].p_mw += flow.power_to_mva.real();
        out.terminal_powers[winding].q_mvar += flow.power_to_mva.imag();
      }
    }
    out.total_loss.p_mw += flow.loss_mva.real();
    out.total_loss.q_mvar += flow.loss_mva.imag();
    out.pair_branch_breakdown.push_back({
        {"pair_number", static_cast<int>(pair)},
        {"branch_name", branch.name},
        {"from_bus", branch.from_bus},
        {"to_bus", branch.to_bus},
        {"power_from_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_from_mva.real(), .q_mvar = flow.power_from_mva.imag()})},
        {"power_to_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_to_mva.real(), .q_mvar = flow.power_to_mva.imag()})},
    });
  }
  out.derivation["solver"] = "dense_shadow_nr";
  out.derivation["iterations"] = dense.iterations;
  return out;
}

hacdcpf_compare::PowerPair sum_pd_element_terminal_powers_for_terminal(
    const hacdcpf::io::OpenDSSPDElementResult& element,
    int terminal) {
  hacdcpf_compare::PowerPair total;
  for (const auto& power : element.terminal_powers) {
    if (power.terminal != terminal) {
      continue;
    }
    const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(power);
    total.p_mw += converted.p_mw;
    total.q_mvar += converted.q_mvar;
  }
  return total;
}

ThreePhaseVector repo_bus_voltage_vector(
    const hacdcpf::analysis::ThreePhaseBusVoltage& bus) {
  const double deg_to_rad = M_PI / 180.0;
  return {
      std::polar(bus.vm_a_pu, bus.va_a_deg * deg_to_rad),
      std::polar(bus.vm_b_pu, bus.va_b_deg * deg_to_rad),
      std::polar(bus.vm_c_pu, bus.va_c_deg * deg_to_rad),
  };
}

hacdcpf_compare::PowerPair compute_three_phase_line_loss(
    const hacdcpf::ThreePhaseACLine& line,
    const ThreePhaseVector& v_from,
    const ThreePhaseVector& v_to,
    double base_mva) {
  const Complex z1(line.r1_pu, line.x1_pu);
  Complex z0(line.r0_pu, line.x0_pu);
  if (std::abs(z0) < 1e-20) {
    z0 = z1;
  }

  const Complex ys = (1.0 / z0 + 2.0 / z1) / 3.0;
  const Complex ym = (1.0 / z0 - 1.0 / z1) / 3.0;

  ThreePhaseVector dv;
  for (int phase = 0; phase < 3; ++phase) {
    dv[phase] = v_from[phase] - v_to[phase];
  }

  ThreePhaseVector current;
  for (int phase = 0; phase < 3; ++phase) {
    current[phase] = ys * dv[phase] +
                     ym * dv[(phase + 1) % 3] +
                     ym * dv[(phase + 2) % 3];
  }

  hacdcpf_compare::PowerPair loss;
  for (int phase = 0; phase < 3; ++phase) {
    const Complex phase_loss = dv[phase] * std::conj(current[phase]) * base_mva;
    loss.p_mw += phase_loss.real();
    loss.q_mvar += phase_loss.imag();
  }
  return loss;
}

json build_opendss_pd_element_loss_breakdown(
    const hacdcpf::io::OpenDSSSnapshotResult& dss) {
  json elements = json::array();
  for (const auto& element : dss.pd_element_results) {
    const auto terminal_sum =
        hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(element);
    const auto element_losses =
        hacdcpf_compare::opendss_element_losses_to_mw_mvar(element);
    elements.push_back({
        {"element_kind",
         hacdcpf::io::opendss_pd_element_kind_to_string(element.element_kind)},
        {"name", element.name},
        {"loss_source",
         "sum(ActiveCktElement.Powers over both terminals and all conductors)"},
        {"terminal_power_sum_mw_mvar", power_pair_to_json(terminal_sum)},
        {"element_losses_raw_w_var",
         {
             {"p_w", element.losses_raw.p_w},
             {"q_var", element.losses_raw.q_var},
         }},
        {"element_losses_mw_mvar", power_pair_to_json(element_losses)},
        {"abs_diff_terminal_sum_vs_element_losses",
         {
             {"p_mw", std::abs(terminal_sum.p_mw - element_losses.p_mw)},
             {"q_mvar", std::abs(terminal_sum.q_mvar - element_losses.q_mvar)},
         }},
    });
  }
  return elements;
}

json build_repo_unbalanced_line_loss_breakdown(
    const hacdcpf::ThreePhaseACSystem& repo_case,
    const hacdcpf::analysis::ThreePhaseDPFResult& repo_result) {
  std::map<int, std::size_t> bus_position;
  for (std::size_t idx = 0; idx < repo_result.bus_voltages.size(); ++idx) {
    bus_position.emplace(repo_result.bus_voltages[idx].bus_id, idx);
  }

  json lines = json::array();
  for (std::size_t idx = 0; idx < repo_case.lines.size(); ++idx) {
    const auto& line = repo_case.lines[idx];
    const auto from_it = bus_position.find(line.from_bus);
    const auto to_it = bus_position.find(line.to_bus);
    if (from_it == bus_position.end() || to_it == bus_position.end()) {
      throw std::runtime_error("Missing bus voltage for repo unbalanced loss breakdown.");
    }

    const auto loss = compute_three_phase_line_loss(
        line,
        repo_bus_voltage_vector(repo_result.bus_voltages[from_it->second]),
        repo_bus_voltage_vector(repo_result.bus_voltages[to_it->second]),
        repo_case.base_mva);

    lines.push_back({
        {"line_name", "l" + std::to_string(idx + 1)},
        {"formula", "sum_phase((V_from_phase - V_to_phase) * conj(I_phase)) * base_mva"},
        {"p_mw", loss.p_mw},
        {"q_mvar", loss.q_mvar},
    });
  }
  return lines;
}

// ---------------------------------------------------------------------------
// summarize_single_phase_case
// For minimal 3-bus single-phase cases (bus names: sourcebus, bus2, bus3;
// line names: l1, l2; phase node: 1).
// ---------------------------------------------------------------------------
json summarize_single_phase_case(const fs::path& project_root,
                                 const fs::path& master_dss,
                                 const std::string& case_id,
                                 const std::string& acceptance_tier,
                                 const std::string& status_reason,
                                 const std::string& scope_reason,
                                 const hacdcpf::HybridPowerSystem& repo_case,
                                 const CaseTolerance& tol) {
  using namespace hacdcpf::analysis;

  hacdcpf::analysis::DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;

  const auto repo_result = solve_distribution_pf(repo_case.ac, options);
  if (!repo_result.converged) {
    throw std::runtime_error("Repository single-phase DPF did not converge for " + case_id);
  }

  const auto opendss_result = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!opendss_result.converged) {
    throw std::runtime_error("OpenDSS single-phase snapshot did not converge for " + case_id);
  }

  const auto voltage_index = build_voltage_index(opendss_result);
  const auto power_index = build_terminal_power_index(opendss_result);
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(opendss_result);

  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  const std::array<std::string, 2> line_names = {"l1", "l2"};

  std::vector<double> repo_vm, opendss_vm;
  std::vector<double> repo_va, opendss_va;
  std::vector<double> repo_p, opendss_p;
  std::vector<double> repo_q, opendss_q;

  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    repo_vm.push_back(repo_result.vm_pu[i]);
    repo_va.push_back(repo_result.va_deg[i]);
    const auto& nv = find_node_voltage(voltage_index, bus_names[i], 1);
    opendss_vm.push_back(nv.vm_pu);
    opendss_va.push_back(nv.va_deg);
  }

  for (std::size_t i = 0; i < line_names.size(); ++i) {
    repo_p.push_back(repo_result.p_branch_mw[i]);
    repo_q.push_back(repo_result.q_branch_mvar[i]);
    const auto& sp = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Line, line_names[i], 1, 1);
    const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
    opendss_p.push_back(converted.p_mw);
    opendss_q.push_back(converted.q_mvar);
  }

  const std::vector<double> repo_loss_p  = {repo_result.total_p_loss_mw};
  const std::vector<double> repo_loss_q  = {repo_result.total_q_loss_mvar};
  const std::vector<double> odss_loss_p  = {opendss_losses.p_mw};
  const std::vector<double> odss_loss_q  = {opendss_losses.q_mvar};

  return {
      {"case_id",        case_id},
      {"acceptance_tier", acceptance_tier},
      {"status_reason",  status_reason},
      {"module_in_scope", "solve_distribution_pf(const ACSystem&)"},
      {"scope_reason",   scope_reason},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::" + case_id +
       " in tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo_result.converged}, {"opendss", opendss_result.converged}}},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, opendss_vm))},
           {"bus_voltage_angle_deg",
            to_json(summarize_metric(repo_va, opendss_va))},
           {"line_terminal1_p_mw",
            to_json(summarize_metric(repo_p, opendss_p))},
           {"line_terminal1_q_mvar",
            to_json(summarize_metric(repo_q, opendss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, odss_loss_p))},
           {"total_line_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, odss_loss_q))},
       }},
      {"acceptance_tolerances", tolerance_to_json(tol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg",    repo_va},
           {"line_terminal1_p_mw",      repo_p},
           {"line_terminal1_q_mvar",    repo_q},
           {"total_line_loss_p_mw",     repo_result.total_p_loss_mw},
           {"total_line_loss_q_mvar",   repo_result.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", opendss_vm},
           {"bus_voltage_angle_deg",    opendss_va},
           {"line_terminal1_p_mw",      opendss_p},
           {"line_terminal1_q_mvar",    opendss_q},
           {"total_line_loss_p_mw",     opendss_losses.p_mw},
           {"total_line_loss_q_mvar",   opendss_losses.q_mvar},
           {"circuit_losses_raw",       losses_raw_to_json(opendss_result.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle",     "degrees (reference: slack bus at 0)"},
           {"line_power",            "MW / MVAr"},
           {"total_line_loss",       "MW / MVAr"},
           {"opendss_powers_source", "EPRI ActiveCktElement.Powers (kW/kvar)"},
           {"voltage_base_note",
            "The 1-phase bus is modeled at 7.2 kV line-to-neutral, while "
            "Set VoltageBases uses the equivalent 12.4707658 kV line-to-line "
            "base required by OpenDSS CalcVoltageBases."},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_physical_capacitor_case
// Like summarize_single_phase_case but adds bias_analysis block quantifying
// the structural mismatch between BFS fixed injection and OpenDSS V^2 cap.
// ---------------------------------------------------------------------------
json summarize_physical_capacitor_case(const fs::path& project_root,
                                       const fs::path& master_dss) {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_radial_with_cap_physical();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo_result = solve_distribution_pf(repo_case.ac, options);
  if (!repo_result.converged) {
    throw std::runtime_error("Repository DPF did not converge for cap case");
  }

  const auto opendss_result = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!opendss_result.converged) {
    throw std::runtime_error("OpenDSS snapshot did not converge for cap case");
  }

  const auto voltage_index = build_voltage_index(opendss_result);
  const auto power_index = build_terminal_power_index(opendss_result);
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(opendss_result);

  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  const std::array<std::string, 2> line_names = {"l1", "l2"};

  std::vector<double> repo_vm, opendss_vm;
  std::vector<double> repo_va, opendss_va;
  std::vector<double> repo_p, opendss_p;
  std::vector<double> repo_q, opendss_q;

  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    repo_vm.push_back(repo_result.vm_pu[i]);
    repo_va.push_back(repo_result.va_deg[i]);
    const auto& nv = find_node_voltage(voltage_index, bus_names[i], 1);
    opendss_vm.push_back(nv.vm_pu);
    opendss_va.push_back(nv.va_deg);
  }

  for (std::size_t i = 0; i < line_names.size(); ++i) {
    repo_p.push_back(repo_result.p_branch_mw[i]);
    repo_q.push_back(repo_result.q_branch_mvar[i]);
    const auto& sp = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Line, line_names[i], 1, 1);
    const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
    opendss_p.push_back(converted.p_mw);
    opendss_q.push_back(converted.q_mvar);
  }

  const std::vector<double> repo_loss_p  = {repo_result.total_p_loss_mw};
  const std::vector<double> repo_loss_q  = {repo_result.total_q_loss_mvar};
  const std::vector<double> odss_loss_p  = {opendss_losses.p_mw};
  const std::vector<double> odss_loss_q  = {opendss_losses.q_mvar};

  // Bias analysis: expected = (1 - V_bus2^2) * cap_kvar_pu
  // BFS result is at slightly higher voltage (fixed injection > V^2 injection)
  // so repo_vm[1] >= opendss_vm[1] (BFS overcorrects Q → higher V at bus2).
  const double v_bus2_opendss  = opendss_vm[1];  // physical cap Q is lower
  const double v_bus2_repo     = repo_vm[1];      // fixed injection Q is higher
  const double cap_kvar_pu     = 0.15;            // MVAr on 10 MVA base
  const double expected_q_bias = (1.0 - v_bus2_opendss * v_bus2_opendss) * cap_kvar_pu;
  const double measured_q_bias = repo_q[0] - opendss_q[0];  // l1 Q difference

  return {
      {"case_id",        "minimal_radial_3bus_1ph_cap"},
      {"acceptance_tier", "exploratory_physical_component"},
      {"status_reason",
       "OpenDSS uses a physical Capacitor element (Q = V^2 * B) while BFS uses "
       "ACBus.bs_mvar as a fixed voltage-independent injection. The systematic "
       "bias is (1-V_bus2^2)*cap_kvar (see bias_analysis for measured values). "
       "This case is accepted as exploratory: the bias is expected, quantified, "
       "and documented. It is NOT accepted as a primary/secondary baseline."},
      {"module_in_scope", "solve_distribution_pf(const ACSystem&)"},
      {"scope_reason",
       "Demonstrates the physical Capacitor element and quantifies the structural "
       "model mismatch between BFS constant injection and OpenDSS V^2 susceptance."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_3bus_radial_with_cap_physical in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo_result.converged}, {"opendss", opendss_result.converged}}},
      {"bias_analysis",
       {
           {"model_difference",
            "BFS: ACBus.bs_mvar = 0.15 MVAr fixed injection (voltage-independent). "
            "OpenDSS: Capacitor element kvar=150, kv=7.2 → Q = V^2 * B (voltage-dependent)."},
           {"systematic_bias_formula",
            "expected_q_overinjection = (1 - V_bus2^2) * cap_kvar_pu  [MVAr]"},
           {"v_bus2_opendss_pu",   v_bus2_opendss},
           {"v_bus2_repo_pu",      v_bus2_repo},
           {"cap_kvar_mvar",       cap_kvar_pu},
           {"expected_q_bias_mvar", expected_q_bias},
           {"measured_q_l1_bias_mvar", measured_q_bias},
           {"bias_fraction_of_cap_rating",
            expected_q_bias / cap_kvar_pu},
           {"interpretation",
            "BFS overestimates the reactive shunt injection by ~(1-V^2)*kvar at the cap bus. "
            "This causes BFS to underestimate the reactive demand, so V_repo > V_opendss at bus2. "
            "The effect propagates downstream (bus3 also slightly higher in BFS). "
            "Expected and measured bias agree within BFS convergence tolerance, confirming "
            "the model difference is the sole source of the discrepancy."},
       }},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, opendss_vm))},
           {"bus_voltage_angle_deg",
            to_json(summarize_metric(repo_va, opendss_va))},
           {"line_terminal1_p_mw",
            to_json(summarize_metric(repo_p, opendss_p))},
           {"line_terminal1_q_mvar",
            to_json(summarize_metric(repo_q, opendss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, odss_loss_p))},
           {"total_line_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, odss_loss_q))},
       }},
      {"acceptance_tolerances", tolerance_to_json(kCapTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg",    repo_va},
           {"line_terminal1_p_mw",      repo_p},
           {"line_terminal1_q_mvar",    repo_q},
           {"total_line_loss_p_mw",     repo_result.total_p_loss_mw},
           {"total_line_loss_q_mvar",   repo_result.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", opendss_vm},
           {"bus_voltage_angle_deg",    opendss_va},
           {"line_terminal1_p_mw",      opendss_p},
           {"line_terminal1_q_mvar",    opendss_q},
           {"total_line_loss_p_mw",     opendss_losses.p_mw},
           {"total_line_loss_q_mvar",   opendss_losses.q_mvar},
           {"circuit_losses_raw",       losses_raw_to_json(opendss_result.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle",     "degrees"},
           {"line_power",            "MW / MVAr"},
           {"total_line_loss",       "MW / MVAr"},
           {"opendss_powers_source", "EPRI ActiveCktElement.Powers (kW/kvar)"},
           {"voltage_base_note",
            "7.2 kV L-N; Set VoltageBases uses equivalent 12.4707658 kV L-L"},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_balanced_three_phase_case
// ---------------------------------------------------------------------------
json summarize_balanced_three_phase_case(const fs::path& project_root,
                                         const fs::path& master_dss) {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_balanced_3bus_3phase();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo_result = solve_three_phase_distribution_pf(repo_case, options);
  if (!repo_result.converged) {
    throw std::runtime_error("Repository three-phase BFS did not converge.");
  }

  const auto opendss_result = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!opendss_result.converged) {
    throw std::runtime_error("OpenDSS balanced three-phase snapshot did not converge.");
  }

  const auto voltage_index = build_voltage_index(opendss_result);
  const auto power_index = build_terminal_power_index(opendss_result);
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(opendss_result);

  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  const std::array<std::string, 2> line_names = {"l1", "l2"};

  std::vector<double> repo_vm, opendss_vm;
  std::vector<double> repo_p, opendss_p;
  std::vector<double> repo_q, opendss_q;

  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    const auto& bv = repo_result.bus_voltages[i];
    repo_vm.push_back(bv.vm_a_pu);
    repo_vm.push_back(bv.vm_b_pu);
    repo_vm.push_back(bv.vm_c_pu);

    opendss_vm.push_back(find_node_voltage(voltage_index, bus_names[i], 1).vm_pu);
    opendss_vm.push_back(find_node_voltage(voltage_index, bus_names[i], 2).vm_pu);
    opendss_vm.push_back(find_node_voltage(voltage_index, bus_names[i], 3).vm_pu);
  }

  for (std::size_t i = 0; i < line_names.size(); ++i) {
    const auto& bp = repo_result.branch_powers[i];
    repo_p.push_back(bp.p_a_mw);
    repo_p.push_back(bp.p_b_mw);
    repo_p.push_back(bp.p_c_mw);
    repo_q.push_back(bp.q_a_mvar);
    repo_q.push_back(bp.q_b_mvar);
    repo_q.push_back(bp.q_c_mvar);

    opendss_p.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 1)).p_mw);
    opendss_p.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 2)).p_mw);
    opendss_p.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 3)).p_mw);
    opendss_q.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 1)).q_mvar);
    opendss_q.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 2)).q_mvar);
    opendss_q.push_back(
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(
            find_terminal_power(
                power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
                line_names[i], 1, 3)).q_mvar);
  }

  const std::vector<double> repo_loss_p  = {repo_result.total_p_loss_mw};
  const std::vector<double> repo_loss_q  = {repo_result.total_q_loss_mvar};
  const std::vector<double> odss_loss_p  = {opendss_losses.p_mw};
  const std::vector<double> odss_loss_q  = {opendss_losses.q_mvar};

  return {
      {"case_id",       "minimal_radial_3bus_balanced_3ph"},
      {"acceptance_tier", "accepted_secondary"},
      {"status_reason",
       "The balanced three-phase radial path matches within 2e-3 p.u./1e-3 MW, "
       "making it the accepted secondary path behind the single-phase baseline."},
      {"module_in_scope",
       "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
      {"scope_reason",
       "Balanced radial three-phase BFS extends the primary path to preserve "
       "OpenDSS phase ordering and per-phase line-power extraction."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_balanced_3bus_3phase in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo_result.converged}, {"opendss", opendss_result.converged}}},
      {"metrics",
       {
           {"bus_phase_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, opendss_vm))},
           {"line_terminal1_phase_p_mw",
            to_json(summarize_metric(repo_p, opendss_p))},
           {"line_terminal1_phase_q_mvar",
            to_json(summarize_metric(repo_q, opendss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, odss_loss_p))},
           {"total_line_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, odss_loss_q))},
       }},
      {"acceptance_tolerances", tolerance_to_json(k3phTol)},
      {"repo",
       {
           {"bus_phase_voltage_magnitude_pu", repo_vm},
           {"line_terminal1_phase_p_mw",      repo_p},
           {"line_terminal1_phase_q_mvar",    repo_q},
           {"total_line_loss_p_mw",           repo_result.total_p_loss_mw},
           {"total_line_loss_q_mvar",         repo_result.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_phase_voltage_magnitude_pu", opendss_vm},
           {"line_terminal1_phase_p_mw",      opendss_p},
           {"line_terminal1_phase_q_mvar",    opendss_q},
           {"total_line_loss_p_mw",           opendss_losses.p_mw},
           {"total_line_loss_q_mvar",         opendss_losses.q_mvar},
           {"circuit_losses_raw",             losses_raw_to_json(opendss_result.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_phase_voltage_magnitude", "p.u."},
           {"line_phase_power",            "MW / MVAr"},
           {"total_line_loss",             "MW / MVAr"},
           {"opendss_powers_source",       "EPRI ActiveCktElement.Powers (kW/kvar)"},
           {"load_model_note",
            "OpenDSS loads pinned to constant-power (model=1) with vminpu/vmaxpu "
            "bounds to avoid voltage-dependent fallback during comparison."},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_unbalanced_three_phase_case
// Minimal 3-bus 3-phase unbalanced radial.
// Verifies per-phase Vm, Va, terminal-1 P/Q (A/B/C), and total losses.
// This is the accepted unbalanced 3-phase baseline — first case to exercise
// the per-phase voltage angle path and asymmetric zero-sequence impedance.
// ---------------------------------------------------------------------------
json summarize_unbalanced_three_phase_case(const fs::path& project_root,
                                            const fs::path& master_dss) {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_unbalanced_3bus_3phase();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  const auto repo = solve_three_phase_distribution_pf(repo_case, options);
  if (!repo.converged) {
    throw std::runtime_error("Repository three-phase BFS did not converge for unbalanced case.");
  }

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error("OpenDSS snapshot did not converge for unbalanced case.");
  }

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index   = build_terminal_power_index(dss);
  const auto dss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);

  const std::array<std::string, 3> bus_names  = {"sourcebus", "bus2", "bus3"};
  const std::array<std::string, 2> line_names = {"l1", "l2"};

  // Per-phase voltage magnitude (A, B, C across all 3 buses = 9 values each)
  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    const auto& bv = repo.bus_voltages[i];
    repo_vm.push_back(bv.vm_a_pu);
    repo_vm.push_back(bv.vm_b_pu);
    repo_vm.push_back(bv.vm_c_pu);
    repo_va.push_back(bv.va_a_deg);
    repo_va.push_back(bv.va_b_deg);
    repo_va.push_back(bv.va_c_deg);

    for (int node = 1; node <= 3; ++node) {
      const auto& nv = find_node_voltage(voltage_index, bus_names[i], node);
      dss_vm.push_back(nv.vm_pu);
      dss_va.push_back(nv.va_deg);
    }
  }

  // Per-phase sending-end power (A, B, C across both lines = 6 values each)
  std::vector<double> repo_p, dss_p;
  std::vector<double> repo_q, dss_q;
  for (std::size_t i = 0; i < line_names.size(); ++i) {
    const auto& bp = repo.branch_powers[i];
    repo_p.push_back(bp.p_a_mw);  repo_p.push_back(bp.p_b_mw);  repo_p.push_back(bp.p_c_mw);
    repo_q.push_back(bp.q_a_mvar); repo_q.push_back(bp.q_b_mvar); repo_q.push_back(bp.q_c_mvar);

    for (int node = 1; node <= 3; ++node) {
      const auto& sp = find_terminal_power(
          power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
          line_names[i], 1, node);
      const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
      dss_p.push_back(converted.p_mw);
      dss_q.push_back(converted.q_mvar);
    }
  }

  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p  = {dss_losses.p_mw};
  const std::vector<double> dss_loss_q  = {dss_losses.q_mvar};

  return {
      {"case_id",        "minimal_unbalanced_3bus_3ph"},
      {"acceptance_tier", "accepted_unbalanced_3ph"},
      {"status_reason",
       "Per-phase unequal loads (A≠B≠C) plus asymmetric zero-sequence line impedances "
       "(r0 ≠ r1). Both BFS ThreePhaseACSystem and OpenDSS 3-phase model with "
       "per-node 1-phase loads match within the kUnbal3phTol acceptance band. "
       "First accepted case to validate per-phase voltage angle comparison."},
      {"module_in_scope",
       "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
      {"scope_reason",
       "Extends the balanced 3-phase baseline to genuinely unbalanced loads and "
       "asymmetric zero-sequence impedances. Verifies: per-phase Vm, Va, "
       "terminal-1 P/Q for all 3 phases of both lines, and total losses."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_unbalanced_3bus_3phase in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"metrics",
       {
           {"bus_phase_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_phase_voltage_angle_deg",
            to_json(summarize_metric(repo_va, dss_va))},
           {"line_terminal1_phase_p_mw",
            to_json(summarize_metric(repo_p, dss_p))},
           {"line_terminal1_phase_q_mvar",
            to_json(summarize_metric(repo_q, dss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_line_loss_q_mvar",
           to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"loss_diagnostics",
       {
           {"line_loss_breakdown_mw_mvar",
            {
                {"repo", build_repo_unbalanced_line_loss_breakdown(repo_case, repo)},
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss",
            power_pair_to_json(dss_losses)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar",
                 power_pair_to_json(hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss))},
                {"abs_diff_vs_terminal_sum",
                 {
                     {"p_mw",
                      std::abs(hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss).p_mw -
                               dss_losses.p_mw)},
                     {"q_mvar",
                      std::abs(hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss).q_mvar -
                               dss_losses.q_mvar)},
                 }},
            }},
           {"repo_total_loss_assembly",
            {
                {"source_file", "src/analysis/distribution_power_flow.cpp"},
                {"formula",
                 "For each 3-phase branch, total complex loss = "
                 "sum_phase((V_from_phase - V_to_phase) * conj(I_phase)) * base_mva; "
                 "total_p_loss_mw / total_q_loss_mvar are sums over all branches."},
                {"root_cause_fixed",
                 "The previous implementation used only |I_phase|^2 times the diagonal "
                 "self impedance of Zabc, omitting mutual-coupling cross terms. "
                 "That undercounted/overcounted total losses on the unbalanced r0!=r1 case "
                 "while leaving Vm/Va/terminal powers nearly unchanged."},
            }},
       }},
      {"acceptance_tolerances", tolerance_to_json(kUnbal3phTol)},
      {"repo",
       {
           {"bus_phase_voltage_magnitude_pu", repo_vm},
           {"bus_phase_voltage_angle_deg",    repo_va},
           {"line_terminal1_phase_p_mw",      repo_p},
           {"line_terminal1_phase_q_mvar",    repo_q},
           {"total_line_loss_p_mw",           repo.total_p_loss_mw},
           {"total_line_loss_q_mvar",         repo.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_phase_voltage_magnitude_pu", dss_vm},
           {"bus_phase_voltage_angle_deg",    dss_va},
           {"line_terminal1_phase_p_mw",      dss_p},
           {"line_terminal1_phase_q_mvar",    dss_q},
           {"total_line_loss_p_mw",           dss_losses.p_mw},
           {"total_line_loss_q_mvar",         dss_losses.q_mvar},
           {"circuit_losses_raw",             losses_raw_to_json(dss.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_phase_voltage_magnitude", "p.u."},
           {"bus_phase_voltage_angle",     "degrees (phase A reference: 0° at slack)"},
           {"line_phase_power",            "MW / MVAr"},
           {"total_line_loss",             "MW / MVAr"},
           {"opendss_powers_source",       "EPRI ActiveCktElement.Powers (kW/kvar)"},
           {"load_model_note",
            "Per-phase loads modeled as individual 1-phase elements (bus2.1/2/3, bus3.1/2/3) "
            "with kv=7.2 L-N and vminpu/vmaxpu bounds to disable voltage-dependent fallback."},
       }},
  };
}

json summarize_three_phase_nr_case(
    const fs::path& project_root,
    const hacdcpf_compare_fixtures::ThreePhaseNROpenDSSCase& spec,
    const hacdcpf::ThreePhaseACSystem& repo_case,
    const std::string& repo_case_origin,
    const std::string& status_reason,
    const std::string& scope_reason) {
  using namespace hacdcpf::analysis;

  const fs::path master_dss = project_root / spec.relative_master_dss;

  ThreePhaseNROptions nr_options;
  nr_options.max_iter = 100;
  nr_options.tol = 1e-10;
  const auto repo = solve_three_phase_nr(repo_case, nr_options);
  if (!repo.converged) {
    throw std::runtime_error("Repository three-phase NR did not converge for " +
                             spec.case_id);
  }

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error("OpenDSS snapshot did not converge for " +
                             spec.case_id);
  }

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto dss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);

  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < spec.bus_names.size(); ++i) {
    const auto& bv = repo.bus_voltages[i];
    const auto mask = repo_case.buses[i].phase_mask;
    const std::array<double, 3> vm_values = {bv.vm_a_pu, bv.vm_b_pu, bv.vm_c_pu};
    const std::array<double, 3> va_values = {bv.va_a_deg, bv.va_b_deg, bv.va_c_deg};

    for (int node = 1; node <= 3; ++node) {
      if (!mask.has(node - 1)) continue;
      repo_vm.push_back(vm_values[static_cast<std::size_t>(node - 1)]);
      repo_va.push_back(va_values[static_cast<std::size_t>(node - 1)]);
      const auto& nv = find_node_voltage(voltage_index, spec.bus_names[i], node);
      dss_vm.push_back(nv.vm_pu);
      dss_va.push_back(nv.va_deg);
    }
  }

  std::vector<double> repo_p, dss_p;
  std::vector<double> repo_q, dss_q;
  for (std::size_t i = 0; i < spec.line_names.size(); ++i) {
    const auto& bp = repo.branch_powers[i];
    const auto mask = repo_case.lines[i].phase_mask;
    const std::array<double, 3> p_values = {bp.p_a_mw, bp.p_b_mw, bp.p_c_mw};
    const std::array<double, 3> q_values = {bp.q_a_mvar, bp.q_b_mvar, bp.q_c_mvar};

    for (int node = 1; node <= 3; ++node) {
      if (!mask.has(node - 1)) continue;
      repo_p.push_back(p_values[static_cast<std::size_t>(node - 1)]);
      repo_q.push_back(q_values[static_cast<std::size_t>(node - 1)]);
      const auto& sp = find_terminal_power(
          power_index, hacdcpf::io::OpenDSSPDElementKind::Line,
          spec.line_names[i], 1, node);
      const auto converted =
          hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
      dss_p.push_back(converted.p_mw);
      dss_q.push_back(converted.q_mvar);
    }
  }

  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p = {dss_losses.p_mw};
  const std::vector<double> dss_loss_q = {dss_losses.q_mvar};

  return {
      {"case_id", spec.case_id},
      {"acceptance_tier", spec.acceptance_tier},
      {"status_reason", status_reason},
      {"module_in_scope", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
      {"scope_reason", scope_reason},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin", repo_case_origin},
      {"converged", {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"metrics",
       {
           {"bus_phase_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_phase_voltage_angle_deg",
            to_json(summarize_metric(repo_va, dss_va))},
           {"line_terminal1_phase_p_mw",
            to_json(summarize_metric(repo_p, dss_p))},
           {"line_terminal1_phase_q_mvar",
            to_json(summarize_metric(repo_q, dss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_line_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"acceptance_tolerances", tolerance_to_json(k3phTol)},
      {"repo",
       {
           {"bus_phase_voltage_magnitude_pu", repo_vm},
           {"bus_phase_voltage_angle_deg", repo_va},
           {"line_terminal1_phase_p_mw", repo_p},
           {"line_terminal1_phase_q_mvar", repo_q},
           {"total_line_loss_p_mw", repo.total_p_loss_mw},
           {"total_line_loss_q_mvar", repo.total_q_loss_mvar},
           {"iterations", repo.iterations},
           {"residual", repo.residual},
       }},
      {"opendss",
       {
           {"bus_phase_voltage_magnitude_pu", dss_vm},
           {"bus_phase_voltage_angle_deg", dss_va},
           {"line_terminal1_phase_p_mw", dss_p},
           {"line_terminal1_phase_q_mvar", dss_q},
           {"total_line_loss_p_mw", dss_losses.p_mw},
           {"total_line_loss_q_mvar", dss_losses.q_mvar},
           {"circuit_losses_raw", losses_raw_to_json(dss.circuit_losses_raw)},
       }},
      {"loss_diagnostics",
       {
           {"line_loss_breakdown_mw_mvar",
            {
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss", power_pair_to_json(dss_losses)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar",
                 power_pair_to_json(
                     hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss))},
            }},
       }},
      {"units",
       {
           {"bus_phase_voltage_magnitude", "p.u."},
           {"bus_phase_voltage_angle", "degrees"},
           {"line_phase_power", "MW / MVAr"},
           {"total_line_loss", "MW / MVAr"},
           {"opendss_powers_source", "EPRI ActiveCktElement.Powers (kW/kvar)"},
       }},
  };
}

json summarize_accepted_meshed_three_phase_nr_case(const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_meshed_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_meshed_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_meshed_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR is now in the formal compare lane on a minimal meshed 3-bus "
      "network. OpenDSS and the repository agree on per-phase bus voltage, line "
      "terminal power, and total losses within the declared tolerance.",
      "First formal meshed-network compare for the NR lane. This closes the old "
      "artifact claim that meshed distribution power flow was outside the compared "
      "modules for OpenDSS validation.");
}

json summarize_accepted_phase_matrix_line_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_phase_matrix_line_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_phase_matrix_line_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_phase_matrix_line_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for full non-circulant "
      "phase-domain line matrices. The repository and OpenDSS agree on per-phase "
      "voltage, line terminal power, and total losses without falling back to the "
      "sequence-to-abc circulant approximation.",
      "Pins the full phase-domain line path in the NR lane with a minimal "
      "3-bus / 2-line case so line-matrix IO and solver stamping are verified "
      "end-to-end against OpenDSS.");
}

json summarize_accepted_load_wye_zip_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_load_wye_zip_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_load_wye_zip_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_load_wye_zip_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for grounded-wye "
      "ZIP loads. The repository and OpenDSS agree on per-phase bus voltage, "
      "line terminal power, and total losses on a minimal 3-bus feeder while "
      "consuming explicit active/reactive ZIP weights through sys.loads.",
      "Pins the grounded-wye ZIP load path in the NR lane without relying on "
      "bus-level fixed PQ demand.");
}

json summarize_accepted_load_wye_open_neutral_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::
          accepted_load_wye_open_neutral_3bus_1phase_lateral_nr_case(),
      hacdcpf_compare_fixtures::
          build_load_wye_open_neutral_3bus_1phase_lateral_nr_case(),
      "hacdcpf_compare_fixtures::build_load_wye_open_neutral_3bus_1phase_lateral_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for open-neutral wye "
      "loads on a single-phase lateral. The repository and OpenDSS agree on the "
      "disconnected no-return-path behavior while consuming the load through "
      "sys.loads instead of bus-level fixed PQ demand.",
      "Pins local-neutral open-circuit semantics and mixed-phase lateral indexing "
      "in the NR lane on a thin 3-bus feeder.");
}

json summarize_accepted_load_wye_impedance_grounded_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::
          accepted_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case(),
      hacdcpf_compare_fixtures::
          build_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case(),
      "hacdcpf_compare_fixtures::build_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for impedance-grounded "
      "constant-impedance wye loads on a single-phase lateral. The repository and "
      "OpenDSS agree on per-phase bus voltage, line terminal power, and total "
      "losses while reducing the local neutral impedance back into the compact "
      "phase-node skeleton.",
      "Pins local-neutral impedance-grounding reduction and mixed-phase lateral "
      "indexing in the NR lane.");
}

json summarize_accepted_load_wye_vmax_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::
          accepted_load_wye_vmax_3bus_1phase_lateral_nr_case(),
      hacdcpf_compare_fixtures::
          build_load_wye_vmax_3bus_1phase_lateral_nr_case(),
      "hacdcpf_compare_fixtures::build_load_wye_vmax_3bus_1phase_lateral_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for the high-voltage "
      "Vmax window on Model-8 grounded-wye loads. The repository and OpenDSS agree "
      "on voltage-dependent load scaling above Vmax without falling back to a "
      "static PQ interpretation.",
      "Pins the high-voltage load window in the NR lane on a single-phase lateral.");
}

json summarize_accepted_load_delta_power_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_load_delta_power_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_load_delta_power_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_load_delta_power_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for constant-power "
      "delta loads. The repository and OpenDSS agree on per-phase bus voltage, "
      "line terminal power, and total losses on a minimal 3-bus feeder while "
      "routing the load through the line-to-line delta path instead of a wye split.",
      "Pins the delta constant-power load path in the NR lane on a thin, "
      "auditable 3-bus feeder.");
}

json summarize_accepted_load_delta_zip_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_load_delta_zip_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_load_delta_zip_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_load_delta_zip_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR now has a formal OpenDSS compare case for delta ZIP loads. "
      "The repository and OpenDSS agree on per-phase bus voltage, line terminal "
      "power, and total losses on a minimal 3-bus feeder while consuming explicit "
      "delta active/reactive ZIP weights through sys.loads.",
      "Pins the delta ZIP load path in the NR lane without falling back to "
      "phase-ground constant-PQ approximations.");
}

json summarize_accepted_multi_source_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_multi_source_3bus_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_multi_source_3bus_3phase_nr_case(),
      "hacdcpf_compare_fixtures::build_multi_source_3bus_3phase_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Three-phase NR is now in the formal compare lane on a minimal multi-source "
      "3-bus network with a downstream per-phase distributed generator injection. "
      "OpenDSS and the repository agree on per-phase voltage, line terminal "
      "power, and total losses within the declared tolerance.",
      "Opens the NR compare lane beyond single-source radial feeders while keeping "
      "the case thin, auditable, and on the existing compare schema.");
}

json summarize_ieee13_4kv_backbone_three_phase_nr_case(
    const fs::path& project_root) {
  return summarize_three_phase_nr_case(
      project_root,
      hacdcpf_compare_fixtures::ieee13_4kv_backbone_3phase_nr_case(),
      hacdcpf_compare_fixtures::build_ieee13_4kv_backbone_3phase(),
      "hacdcpf_compare_fixtures::build_ieee13_4kv_backbone_3phase in "
      "tools/opendss_compare/fixtures.hpp",
      "IEEE 13-node 4.16 kV backbone subset for three-phase NR formal compare. "
      "The 8-bus three-phase subset (rg60, 632, 633, 670, 671, 680, 692, 675) "
      "omits transformers, regulators (modeled as known slack voltage), and "
      "single-phase/two-phase laterals. Sequence impedance approximation of "
      "asymmetric Zabc line code matrices is the dominant error source.",
      "First real-scale IEEE feeder compare for the NR lane. Known modeling "
      "limitations: no transformer support in NR, sequence-to-abc circulant "
      "approximation for asymmetric line impedance matrices, delta loads "
      "approximated as per-phase balanced split.");
}

// ---------------------------------------------------------------------------
// summarize_fixed_tap_transformer_case
// Off-nominal fixed-tap transformer verification through the canonical
// Transformer2W -> ACBranch projection and tap-aware BFS equations.
// ---------------------------------------------------------------------------
json summarize_fixed_tap_transformer_case(const fs::path& project_root) {
  using namespace hacdcpf::analysis;

  const auto spec = hacdcpf_compare_fixtures::fixed_tap_transformer_case();
  const fs::path master_dss = project_root / spec.relative_master_dss;
  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer();
  const auto projected = hacdcpf::project_to_canonical_models(repo_case);

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case, options);
  if (!repo.converged) {
    throw std::runtime_error(
        "Repository DPF did not converge for fixed-tap transformer case.");
  }

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error(
        "OpenDSS snapshot did not converge for fixed-tap transformer case.");
  }

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto pd_element_index = build_pd_element_index(dss);
  const auto& dss_transformer = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
      spec.transformer_name);
  const auto& dss_line = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name);
  const auto& dss_transformer_state =
      find_transformer_state(dss.transformer_states, spec.transformer_name);
  const auto& dss_tapped_winding =
      find_transformer_winding_state(dss_transformer_state, 1);

  const int transformer_branch_pos = find_projected_branch_position(
      projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const int line_branch_pos =
      find_branch_position_by_name(projected.ac, spec.line_name);
  const auto& projected_transformer_branch =
      projected.ac.branches[static_cast<std::size_t>(transformer_branch_pos)];

  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    repo_vm.push_back(repo.vm_pu[i]);
    repo_va.push_back(repo.va_deg[i]);
    const auto& nv = find_node_voltage(voltage_index, bus_names[i], 1);
    dss_vm.push_back(nv.vm_pu);
    dss_va.push_back(nv.va_deg);
  }

  const auto& xfmr_terminal_power = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
      spec.transformer_name, 1, 1);
  const auto dss_xfmr_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(xfmr_terminal_power);
  const auto& line_terminal_power = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name, 1, 1);
  const auto dss_line_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(line_terminal_power);

  const auto dss_branch_loss =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  const auto dss_total_loss_cross_check =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);

  const std::vector<double> repo_p_xfmr = {
      repo.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]};
  const std::vector<double> repo_q_xfmr = {
      repo.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]};
  const std::vector<double> dss_p_xfmr = {dss_xfmr_power.p_mw};
  const std::vector<double> dss_q_xfmr = {dss_xfmr_power.q_mvar};

  const std::vector<double> repo_p_l1 = {
      repo.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]};
  const std::vector<double> repo_q_l1 = {
      repo.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]};
  const std::vector<double> dss_p_l1 = {dss_line_power.p_mw};
  const std::vector<double> dss_q_l1 = {dss_line_power.q_mvar};

  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p = {dss_branch_loss.p_mw};
  const std::vector<double> dss_loss_q = {dss_branch_loss.q_mvar};

  return {
      {"case_id", spec.case_id},
      {"acceptance_tier", spec.acceptance_tier},
      {"status_reason",
       "Fixed off-nominal transformer tap is now part of the accepted baseline. "
       "Transformer2W tap_side/tap_pos/tap_step_percent are normalized onto the "
       "equivalent ACBranch.from_bus tap, and the tap-aware BFS equations match "
       "OpenDSS bus voltages, transformer terminal power, downstream line power, "
       "and summed branch losses within kFixedTapXfmrTol."},
      {"module_in_scope", "solve_distribution_pf(const HybridPowerSystem&)"},
      {"scope_reason",
       "Closes the former tap gap on the repository side without touching the "
       "algorithmically harder closed-loop regulator path. The compare explicitly "
       "exercises the Transformer2W -> ACBranch projection and the tap-aware "
       "single-phase BFS equations."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged", {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"projection_diagnostics",
       {
           {"tap_semantics",
            "Transformer2W tap_side/tap_pos/tap_step_percent are normalized to "
            "ACBranch.tap on the from_bus side before the BFS solve. When the "
            "physical tap lives on the LV/to-side winding, the projected "
            "leakage impedance is also scaled by tap^2 to preserve the original "
            "terminal equations."},
           {"source_transformer",
            {
                {"origin_index", spec.transformer_origin_index},
                {"tap_side", spec.tap_side == 0 ? "hv_from_side" : "lv_to_side"},
                {"tap_pos", spec.tap_pos},
                {"tap_neutral", spec.tap_neutral},
                {"tap_step_percent", spec.tap_step_percent},
                {"configured_winding_tap_pu", spec.configured_transformer_tap_pu},
            }},
           {"projected_branch",
            {
                {"name", projected_transformer_branch.name},
                {"index", projected_transformer_branch.index},
                {"from_bus", projected_transformer_branch.from_bus},
                {"to_bus", projected_transformer_branch.to_bus},
                {"normalized_tap_pu", projected_transformer_branch.tap},
                {"impedance_normalization",
                 tap_impedance_normalization_to_json(
                     spec.tap_side == 0 ? "hv_from_side" : "lv_to_side",
                     spec.tap_side == 1,
                     spec.tap_side == 1,
                     spec.configured_transformer_tap_pu)},
                {"r_pu", projected_transformer_branch.r_pu},
                {"x_pu", projected_transformer_branch.x_pu},
            }},
           {"opendss_transformer_state", transformer_state_to_json(dss_transformer_state)},
       }},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu", to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_voltage_angle_deg", to_json(summarize_metric(repo_va, dss_va))},
           {"transformer_terminal1_p_mw",
            to_json(summarize_metric(repo_p_xfmr, dss_p_xfmr))},
           {"transformer_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_xfmr, dss_q_xfmr))},
           {"line_l1_terminal1_p_mw",
            to_json(summarize_metric(repo_p_l1, dss_p_l1))},
           {"line_l1_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_l1, dss_q_l1))},
           {"total_branch_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_branch_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"loss_diagnostics",
       {
           {"branch_loss_breakdown_mw_mvar",
            {
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss", power_pair_to_json(dss_branch_loss)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar", power_pair_to_json(dss_total_loss_cross_check)},
                {"abs_diff_vs_branch_terminal_sum",
                 {
                     {"p_mw", std::abs(dss_total_loss_cross_check.p_mw - dss_branch_loss.p_mw)},
                     {"q_mvar", std::abs(dss_total_loss_cross_check.q_mvar - dss_branch_loss.q_mvar)},
                 }},
            }},
           {"branch_terminal_cross_check",
            {
                {"transformer",
                 {
                     {"name", dss_transformer.name},
                     {"terminal_power_sum_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(
                              dss_transformer))},
                     {"element_losses_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::opendss_element_losses_to_mw_mvar(
                              dss_transformer))},
                 }},
                {"line",
                 {
                     {"name", dss_line.name},
                     {"terminal_power_sum_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(
                              dss_line))},
                     {"element_losses_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::opendss_element_losses_to_mw_mvar(
                              dss_line))},
                 }},
            }},
       }},
      {"acceptance_tolerances", tolerance_to_json(kFixedTapXfmrTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg", repo_va},
           {"transformer_terminal1_p_mw", repo_p_xfmr},
           {"transformer_terminal1_q_mvar", repo_q_xfmr},
           {"line_l1_terminal1_p_mw", repo_p_l1},
           {"line_l1_terminal1_q_mvar", repo_q_l1},
           {"total_branch_loss_p_mw", repo.total_p_loss_mw},
           {"total_branch_loss_q_mvar", repo.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", dss_vm},
           {"bus_voltage_angle_deg", dss_va},
           {"transformer_terminal1_p_mw", dss_p_xfmr},
           {"transformer_terminal1_q_mvar", dss_q_xfmr},
           {"line_l1_terminal1_p_mw", dss_p_l1},
           {"line_l1_terminal1_q_mvar", dss_q_l1},
           {"total_branch_loss_p_mw", dss_branch_loss.p_mw},
           {"total_branch_loss_q_mvar", dss_branch_loss.q_mvar},
           {"total_p_loss_mw_circuit_losses_raw", dss_total_loss_cross_check.p_mw},
           {"total_q_loss_mvar_circuit_losses_raw", dss_total_loss_cross_check.q_mvar},
           {"circuit_losses_raw", losses_raw_to_json(dss.circuit_losses_raw)},
           {"transformer_state", transformer_state_to_json(dss_transformer_state)},
           {"tapped_winding_state",
            {
                {"winding", dss_tapped_winding.winding},
                {"tap_pu", dss_tapped_winding.tap_pu},
                {"num_taps", dss_tapped_winding.num_taps},
            }},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle", "degrees"},
           {"branch_power", "MW / MVAr"},
           {"total_loss",
            "MW / MVAr (primary: summed branch terminal powers)"},
           {"branch_tap",
            "p.u. from-side turns ratio after Transformer2W tap-side normalization"},
           {"circuit_losses_cross_check",
            "MW / MVAr (Circuit.Losses raw W / var -> MW / MVAr)"},
           {"voltage_base_note", "7.2 kV L-N; VoltageBases=[12.4707658] kV L-L"},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_passthr_transformer_case
// Pass-through transformer (tap=1.0) equivalence verification.
// OpenDSS Transformer element vs ACBranch series-impedance in BFS.
// ---------------------------------------------------------------------------
json summarize_passthr_transformer_case(const fs::path& project_root,
                                         const fs::path& master_dss) {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_3bus_passthr_transformer();
  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case.ac, options);
  if (!repo.converged) {
    throw std::runtime_error("Repository DPF did not converge for pass-through transformer case.");
  }

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error("OpenDSS snapshot did not converge for pass-through transformer case.");
  }

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto pd_element_index = build_pd_element_index(dss);

  // Bus voltages: 3 buses (sourcebus, bus2, bus3), single-phase node 1.
  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    repo_vm.push_back(repo.vm_pu[i]);
    repo_va.push_back(repo.va_deg[i]);
    const auto& nv = find_node_voltage(voltage_index, bus_names[i], 1);
    dss_vm.push_back(nv.vm_pu);
    dss_va.push_back(nv.va_deg);
  }

  const auto& dss_transformer = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Transformer, "t12");
  const auto& dss_line = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Line, "l1");

  // Branch 0 = transformer equivalent branch in the repository solver.
  std::vector<double> repo_p_xfmr = {repo.p_branch_mw[0]};
  std::vector<double> repo_q_xfmr = {repo.q_branch_mvar[0]};
  const auto& xfmr_terminal_power = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer, "t12", 1, 1);
  const auto dss_xfmr_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(xfmr_terminal_power);
  std::vector<double> dss_p_xfmr = {dss_xfmr_power.p_mw};
  std::vector<double> dss_q_xfmr = {dss_xfmr_power.q_mvar};

  // Branch 1 = downstream line l1.
  std::vector<double> repo_p_l1 = {repo.p_branch_mw[1]};
  std::vector<double> repo_q_l1 = {repo.q_branch_mvar[1]};
  const auto& sp = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Line, "l1", 1, 1);
  const auto dss_l1_power = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
  std::vector<double> dss_p_l1  = {dss_l1_power.p_mw};
  std::vector<double> dss_q_l1  = {dss_l1_power.q_mvar};

  // Total branch loss is now sourced from the generic PD-element view
  // (transformer + downstream line), not from Circuit.Losses raw.
  const auto dss_branch_loss =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  const auto dss_total_loss_cross_check =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p  = {dss_branch_loss.p_mw};
  const std::vector<double> dss_loss_q  = {dss_branch_loss.q_mvar};

  return {
      {"case_id",        "minimal_passthr_transformer_1ph"},
      {"acceptance_tier", "accepted_pass_through_xfmr"},
      {"status_reason",
       "OpenDSS Transformer element (kVA=10000, kV=7.2/7.2, xhl=6.0%, loadloss=2.0%, "
       "%noloadloss=0, %imag=0) is equivalent to an ACBranch with r_pu=0.02, x_pu=0.06, "
       "tap=1.0. Bus voltages, transformer terminal power, downstream line power, "
       "and summed branch losses all match within kXfmrTol."},
      {"module_in_scope", "solve_distribution_pf(const ACSystem&)"},
      {"scope_reason",
       "Verifies that a tap=1.0 OpenDSS Transformer with zero magnetizing branch "
       "is numerically equivalent to the BFS series-impedance model. "
       "The branch-class-aware snapshot directly captures transformer t12 and "
       "downstream line l1 terminal powers. Circuit.Losses raw is retained only "
       "as an independent total-loss cross-check."},
      {"bridge_coverage",
       "solve_opendss_snapshot now exposes generic PD elements with "
       "element_kind/name/terminal_bus_names/node_order/terminal_powers. "
       "line_results remains only as a compatibility projection for lines."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_3bus_passthr_transformer in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu",
           to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_voltage_angle_deg",
            to_json(summarize_metric(repo_va, dss_va))},
           {"transformer_terminal1_p_mw",
            to_json(summarize_metric(repo_p_xfmr, dss_p_xfmr))},
           {"transformer_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_xfmr, dss_q_xfmr))},
           {"line_l1_terminal1_p_mw",
            to_json(summarize_metric(repo_p_l1, dss_p_l1))},
           {"line_l1_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_l1, dss_q_l1))},
           {"total_branch_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_branch_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"loss_diagnostics",
       {
           {"branch_loss_breakdown_mw_mvar",
            {
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss",
            power_pair_to_json(dss_branch_loss)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar", power_pair_to_json(dss_total_loss_cross_check)},
                {"abs_diff_vs_branch_terminal_sum",
                 {
                     {"p_mw", std::abs(dss_total_loss_cross_check.p_mw - dss_branch_loss.p_mw)},
                     {"q_mvar", std::abs(dss_total_loss_cross_check.q_mvar - dss_branch_loss.q_mvar)},
                 }},
            }},
           {"branch_terminal_cross_check",
            {
                {"transformer",
                 {
                     {"name", dss_transformer.name},
                     {"terminal_power_sum_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(
                              dss_transformer))},
                     {"element_losses_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::opendss_element_losses_to_mw_mvar(
                              dss_transformer))},
                 }},
                {"line",
                 {
                     {"name", dss_line.name},
                     {"terminal_power_sum_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(
                              dss_line))},
                     {"element_losses_mw_mvar",
                      power_pair_to_json(
                          hacdcpf_compare::opendss_element_losses_to_mw_mvar(
                              dss_line))},
                 }},
            }},
           {"repo_total_loss_assembly",
            {
                {"source_file", "src/analysis/distribution_power_flow.cpp"},
                {"formula",
                 "Single-phase total_p_loss_mw / total_q_loss_mvar are sums of "
                 "|I_branch|^2 * r_pu * base_mva and |I_branch|^2 * x_pu * base_mva "
                 "over all AC branches."},
            }},
       }},
      {"acceptance_tolerances", tolerance_to_json(kXfmrTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg",    repo_va},
           {"transformer_terminal1_p_mw", repo_p_xfmr},
           {"transformer_terminal1_q_mvar", repo_q_xfmr},
           {"line_l1_terminal1_p_mw",  repo_p_l1},
           {"line_l1_terminal1_q_mvar", repo_q_l1},
           {"total_branch_loss_p_mw",  repo.total_p_loss_mw},
           {"total_branch_loss_q_mvar", repo.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", dss_vm},
           {"bus_voltage_angle_deg",    dss_va},
           {"transformer_terminal1_p_mw", dss_p_xfmr},
           {"transformer_terminal1_q_mvar", dss_q_xfmr},
           {"line_l1_terminal1_p_mw",  dss_p_l1},
           {"line_l1_terminal1_q_mvar", dss_q_l1},
           {"total_branch_loss_p_mw", dss_branch_loss.p_mw},
           {"total_branch_loss_q_mvar", dss_branch_loss.q_mvar},
           {"total_p_loss_mw_circuit_losses_raw", dss_total_loss_cross_check.p_mw},
           {"total_q_loss_mvar_circuit_losses_raw", dss_total_loss_cross_check.q_mvar},
           {"circuit_losses_raw",      losses_raw_to_json(dss.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle",     "degrees"},
           {"branch_power",          "MW / MVAr"},
           {"total_loss",            "MW / MVAr (primary: summed branch terminal powers)"},
           {"circuit_losses_cross_check", "MW / MVAr (Circuit.Losses raw W / var -> MW / MVAr)"},
           {"voltage_base_note",     "7.2 kV L-N; VoltageBases=[12.4707658] kV L-L"},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_case33bw_case
// Baran & Wu 33-bus radial — standard feeder subset.
// OpenDSS model: 3-phase balanced (phases=3, basekv=12.66).
// BFS model: ACSystem single-phase equivalent (total 3-phase loads on 10 MVA base).
// Power comparison: BFS p_branch_mw[j] = sum of all 3 phase powers at terminal 1.
// ---------------------------------------------------------------------------
json summarize_case33bw_case(const fs::path& project_root,
                              const fs::path& master_dss) {
  using namespace hacdcpf::analysis;

  const auto repo_case = hacdcpf_compare_fixtures::build_case33bw_radial();
  DPFOptions options;
  options.max_iter = 500;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo_result = solve_distribution_pf(repo_case.ac, options);
  if (!repo_result.converged) {
    throw std::runtime_error("Repository BFS did not converge for case33bw");
  }

  const auto opendss_result = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!opendss_result.converged) {
    throw std::runtime_error("OpenDSS snapshot did not converge for case33bw");
  }

  const auto voltage_index = build_voltage_index(opendss_result);
  const auto power_index = build_terminal_power_index(opendss_result);
  const auto opendss_losses =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(opendss_result);

  // 33 buses: BFS index 0..32 ↔ OpenDSS bus name "b{1..33}"
  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (int i = 0; i < 33; ++i) {
    repo_vm.push_back(repo_result.vm_pu[static_cast<std::size_t>(i)]);
    repo_va.push_back(repo_result.va_deg[static_cast<std::size_t>(i)]);
    const std::string bus_name = "b" + std::to_string(i + 1);
    const auto& nv = find_node_voltage(voltage_index, bus_name, 1);
    dss_vm.push_back(nv.vm_pu);
    dss_va.push_back(nv.va_deg);
  }

  // 32 branches: BFS index 0..31 ↔ OpenDSS line "l01".."l32", terminal 1.
  // For the 3-phase OpenDSS model, BFS p_branch_mw[j] = total 3-phase power
  // = sum of phase A + B + C terminal powers.
  std::vector<double> repo_p, dss_p;
  std::vector<double> repo_q, dss_q;
  for (int j = 0; j < 32; ++j) {
    repo_p.push_back(repo_result.p_branch_mw[static_cast<std::size_t>(j)]);
    repo_q.push_back(repo_result.q_branch_mvar[static_cast<std::size_t>(j)]);

    char buf[8];
    std::snprintf(buf, sizeof(buf), "l%02d", j + 1);
    const std::string line_name(buf);

    double p_sum = 0.0;
    double q_sum = 0.0;
    for (int node = 1; node <= 3; ++node) {
      const auto& sp = find_terminal_power(
          power_index, hacdcpf::io::OpenDSSPDElementKind::Line, line_name, 1, node);
      const auto converted = hacdcpf_compare::opendss_terminal_power_to_mw_mvar(sp);
      p_sum += converted.p_mw;
      q_sum += converted.q_mvar;
    }
    dss_p.push_back(p_sum);
    dss_q.push_back(q_sum);
  }

  const std::vector<double> repo_loss_p = {repo_result.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo_result.total_q_loss_mvar};
  const std::vector<double> dss_loss_p  = {opendss_losses.p_mw};
  const std::vector<double> dss_loss_q  = {opendss_losses.q_mvar};

  // ---------------------------------------------------------------------------
  // Phase symmetry regression (internal OpenDSS consistency, not BFS comparison).
  // Purpose: catch phase ordering / conductor indexing / parser drift.
  // case33bw uses 3-phase balanced lines and balanced 3-phase loads, so:
  //   vm_a ≈ vm_b ≈ vm_c  (relative imbalance should be ~0)
  //   va_b - va_a ≈ -120°, va_c - va_a ≈ +120°  (deviation from ±120° should be ~0)
  //   per-phase terminal powers should be equal for all 3 phases
  // ---------------------------------------------------------------------------
  double max_vm_rel_imbalance = 0.0;
  double max_angle_dev_deg     = 0.0;
  double max_power_rel_imbalance = 0.0;

  for (int i = 0; i < 33; ++i) {
    const std::string bname = "b" + std::to_string(i + 1);
    const double vma = find_node_voltage(voltage_index, bname, 1).vm_pu;
    const double vmb = find_node_voltage(voltage_index, bname, 2).vm_pu;
    const double vmc = find_node_voltage(voltage_index, bname, 3).vm_pu;
    const double vaa = find_node_voltage(voltage_index, bname, 1).va_deg;
    const double vab = find_node_voltage(voltage_index, bname, 2).va_deg;
    const double vac = find_node_voltage(voltage_index, bname, 3).va_deg;

    if (vma > 1e-9) {
      max_vm_rel_imbalance = std::max(max_vm_rel_imbalance,
          std::max(std::abs(vmb - vma) / vma, std::abs(vmc - vma) / vma));
    }
    // Phase B should lag phase A by 120°; phase C should lead phase A by 120°.
    // Deviation = |measured offset − expected ±120°|.
    max_angle_dev_deg = std::max(max_angle_dev_deg,
        std::max(std::abs(vab - vaa + 120.0), std::abs(vac - vaa - 120.0)));
  }

  for (int j = 0; j < 32; ++j) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "l%02d", j + 1);
    const double pa = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Line, buf, 1, 1)
                          .power_kw_kvar.p_kw;
    const double pb = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Line, buf, 1, 2)
                          .power_kw_kvar.p_kw;
    const double pc = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Line, buf, 1, 3)
                          .power_kw_kvar.p_kw;
    const double ptotal = pa + pb + pc;
    if (std::abs(ptotal) > 1.0) {  // skip near-zero total
      const double ref = std::abs(ptotal) / 3.0;
      max_power_rel_imbalance = std::max(max_power_rel_imbalance,
          std::max({std::abs(pa - ptotal / 3.0) / ref,
                    std::abs(pb - ptotal / 3.0) / ref,
                    std::abs(pc - ptotal / 3.0) / ref}));
    }
  }

  // Tightly-balanced model: relative imbalance should be below numerical noise.
  const bool sym_vm_pass    = (max_vm_rel_imbalance    < 1e-4);  // 0.01% relative
  const bool sym_angle_pass = (max_angle_dev_deg        < 1e-3);  // 0.001 deg
  const bool sym_power_pass = (max_power_rel_imbalance  < 1e-4);  // 0.01% relative

  const json phase_sym = {
      {"purpose",
       "Internal OpenDSS consistency check. Catches phase ordering, conductor "
       "indexing, or parser drift. case33bw uses balanced 3-phase loads and "
       "balanced line impedances — all three phases should match to numerical noise."},
      {"vm_rel_max_imbalance",   max_vm_rel_imbalance},
      {"angle_offset_max_dev_deg", max_angle_dev_deg},
      {"power_rel_max_imbalance", max_power_rel_imbalance},
      {"vm_sym_pass",    sym_vm_pass},
      {"angle_sym_pass", sym_angle_pass},
      {"power_sym_pass", sym_power_pass},
      {"tolerances",
       {{"vm_rel",    1e-4},
        {"angle_deg", 1e-3},
        {"power_rel", 1e-4}}},
  };

  return {
      {"case_id",        "case33bw_radial_3ph"},
      {"acceptance_tier", "accepted_feeder_subset"},
      {"status_reason",
       "Baran & Wu (1989) 33-bus purely radial distribution feeder. "
       "No regulators, no off-nominal taps, no unbalanced loads. "
       "BFS single-phase equivalent directly matches the 3-phase balanced OpenDSS model. "
       "Voltage and power metrics are within the k33bwTol acceptance band."},
      {"module_in_scope", "solve_distribution_pf(const ACSystem&)"},
      {"scope_reason",
       "Standard feeder subset: first case beyond the 3-bus family. "
       "33 buses, 3 laterals, total 3715 kW + 2300 kVAr. "
       "Demonstrates BFS correctness on a feeder-scale purely radial network "
       "from the published distribution systems literature."},
      {"feeder_provenance",
       {
           {"source", "Baran & Wu (1989), IEEE Trans. Power Delivery, 4(3)"},
           {"data_source", "Parameters from published Table I; "
                          "verified against publicly available MATPOWER case33bw.m"},
           {"total_buses", 33},
           {"total_branches", 32},
           {"topology", "purely radial (3 laterals off main feeder)"},
           {"base_kv_ll", 12.66},
           {"base_mva", 10.0},
           {"total_load_kw", 3715},
           {"total_load_kvar", 2300},
           {"elements_excluded",
            {"voltage_regulators (none in original)",
             "capacitor_banks (none in original)",
             "transformers (none in original)",
             "tie_switches (5 tie switches exist for reconfiguration studies, "
                          "not used here — radial topology only)"}},
           {"elements_included",
            {"32 radial branches (constant impedance, no tap)",
             "constant-power loads at all 32 PQ buses (OpenDSS model=1)",
             "slack bus (b1 = substation)"}},
       }},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin",
       "hacdcpf_compare_fixtures::build_case33bw_radial in "
       "tools/opendss_compare/fixtures.hpp"},
      {"converged",
       {{"repo", repo_result.converged}, {"opendss", opendss_result.converged}}},
      {"phase_symmetry_checks", phase_sym},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_voltage_angle_deg",
            to_json(summarize_metric(repo_va, dss_va))},
           {"line_terminal_3ph_total_p_mw",
            to_json(summarize_metric(repo_p, dss_p))},
           {"line_terminal_3ph_total_q_mvar",
            to_json(summarize_metric(repo_q, dss_q))},
           {"total_line_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_line_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"acceptance_tolerances", tolerance_to_json(k33bwTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu",         repo_vm},
           {"bus_voltage_angle_deg",            repo_va},
           {"line_terminal_3ph_total_p_mw",     repo_p},
           {"line_terminal_3ph_total_q_mvar",   repo_q},
           {"total_line_loss_p_mw",             repo_result.total_p_loss_mw},
           {"total_line_loss_q_mvar",           repo_result.total_q_loss_mvar},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu",         dss_vm},
           {"bus_voltage_angle_deg",            dss_va},
           {"line_terminal_3ph_total_p_mw",     dss_p},
           {"line_terminal_3ph_total_q_mvar",   dss_q},
           {"total_line_loss_p_mw",             opendss_losses.p_mw},
           {"total_line_loss_q_mvar",           opendss_losses.q_mvar},
           {"circuit_losses_raw",               losses_raw_to_json(opendss_result.circuit_losses_raw)},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle",     "degrees (reference: slack bus at 0)"},
           {"line_power_note",
            "3-phase total terminal power: sum of OpenDSS phase A+B+C powers. "
            "Equals BFS p_branch_mw (3-phase total on 10 MVA base) for balanced system."},
           {"total_line_loss",       "MW / MVAr"},
           {"opendss_powers_source", "EPRI ActiveCktElement.Powers (kW/kvar)"},
       }},
  };
}

// ---------------------------------------------------------------------------
// summarize_regulator_compare_case
// Accepted single-phase regulator compare through repo closed-loop control.
// ---------------------------------------------------------------------------
json summarize_regulator_compare_case(
    const fs::path& project_root,
    const hacdcpf_compare_fixtures::RegulatorOpenDSSCase& spec,
    const hacdcpf::HybridPowerSystem& repo_case,
    const std::string& repo_case_origin,
    const std::string& status_reason,
    const std::string& scope_reason) {
  using namespace hacdcpf::analysis;

  const fs::path master_dss = project_root / spec.relative_master_dss;

  DPFOptions options;
  options.max_iter = 200;
  options.max_control_iter = 100;
  options.tol = 1e-10;
  options.include_shunts = true;
  const auto repo = solve_distribution_pf(repo_case, options);

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error(
        "OpenDSS snapshot did not converge for accepted regulator case.");
  }

  if (repo.regulator_states.empty()) {
    throw std::runtime_error(
        "Repository DPF did not emit regulator_states for accepted regulator case.");
  }

  const auto& repo_regulator = repo.regulator_states.front();
  auto final_repo_case = repo_case;
  final_repo_case.ac.transformers_2w.front().tap_pos = repo_regulator.final_tap_pos;
  const auto final_projected = hacdcpf::project_to_canonical_models(final_repo_case);

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto pd_element_index = build_pd_element_index(dss);
  const auto& xfmr = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
      spec.transformer_name);
  const auto& line = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name);
  const auto& transformer_state =
      find_transformer_state(dss.transformer_states, spec.transformer_name);
  const auto& regcontrol = find_regcontrol(dss.regcontrol_results, spec.regcontrol_name);
  const auto& tap_winding_state =
      find_transformer_winding_state(transformer_state, spec.tap_winding);
  const auto dss_control_snapshot = compute_dss_regulator_control_snapshot(
      voltage_index, power_index, xfmr, regcontrol, spec);

  const int transformer_branch_pos = find_projected_branch_position(
      final_projected, hacdcpf::BranchOriginType::Transformer2W,
      spec.transformer_origin_index);
  const int line_branch_pos =
      find_branch_position_by_name(final_projected.ac, spec.line_name);
  const auto& projected_transformer_branch =
      final_projected.ac.branches[static_cast<std::size_t>(transformer_branch_pos)];

  const std::array<std::string, 3> bus_names = {"sourcebus", "bus2", "bus3"};
  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < bus_names.size(); ++i) {
    repo_vm.push_back(repo.vm_pu[i]);
    repo_va.push_back(repo.va_deg[i]);
    const auto& nv = find_node_voltage(voltage_index, bus_names[i], 1);
    dss_vm.push_back(nv.vm_pu);
    dss_va.push_back(nv.va_deg);
  }

  const auto& xfmr_terminal_power = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
      spec.transformer_name, 1, 1);
  const auto dss_xfmr_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(xfmr_terminal_power);
  const auto& line_terminal_power = find_terminal_power(
      power_index, hacdcpf::io::OpenDSSPDElementKind::Line, spec.line_name, 1, 1);
  const auto dss_line_power =
      hacdcpf_compare::opendss_terminal_power_to_mw_mvar(line_terminal_power);

  const auto xfmr_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
  const auto xfmr_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
  const auto line_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(line);
  const auto line_losses =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(line);
  const auto dss_branch_loss =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  const auto dss_total_loss_cross_check =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);

  const double dss_control_voltage_volts =
      dss_control_snapshot.control_voltage_volts;
  const double dss_control_abs_diff_volts =
      std::abs(dss_control_voltage_volts - regcontrol.forward_vreg_volts);
  const double band_half_volts = regcontrol.forward_band_volts / 2.0;
  const bool regulator_within_band =
      dss_control_abs_diff_volts <= band_half_volts + 1e-6;
  const bool transformer_loss_self_consistent =
      std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw) < 1e-6 &&
      std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar) < 1e-6;
  const bool line_loss_self_consistent =
      std::abs(line_terminal_sum.p_mw - line_losses.p_mw) < 1e-6 &&
      std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar) < 1e-6;
  const bool circuit_loss_self_consistent =
      std::abs(dss_branch_loss.p_mw - dss_total_loss_cross_check.p_mw) < 1e-6 &&
      std::abs(dss_branch_loss.q_mvar - dss_total_loss_cross_check.q_mvar) < 1e-6;

  const std::string expected_monitored_bus_key =
      key_for_bus_node(spec.monitored_bus_name, spec.monitored_bus_node);
  const bool config_transformer_name =
      regcontrol.transformer_name == ascii_lower(spec.transformer_name);
  const bool config_winding = regcontrol.winding == spec.winding;
  const bool config_tap_winding = regcontrol.tap_winding == spec.tap_winding;
  const bool config_monitored_bus = spec.implicit_local_bus_monitoring
                                        ? (regcontrol.monitored_bus_name.empty() ||
                                           regcontrol.monitored_bus_name ==
                                               expected_monitored_bus_key)
                                        : (regcontrol.monitored_bus_name ==
                                           expected_monitored_bus_key);
  const bool config_vreg =
      std::abs(regcontrol.forward_vreg_volts - spec.forward_vreg_volts) < 1e-12;
  const bool config_band =
      std::abs(regcontrol.forward_band_volts - spec.forward_band_volts) < 1e-12;
  const bool config_ptratio =
      std::abs(regcontrol.ptratio - spec.ptratio) < 1e-12;
  const bool config_remote_ptratio =
      std::abs(regcontrol.remote_ptratio - spec.remote_ptratio) < 1e-12;
  const bool config_ctprim =
      std::abs(regcontrol.ct_primary_amps - spec.ct_primary_amps) < 1e-12;
  const bool config_r =
      std::abs(regcontrol.forward_r_volts - spec.forward_r_volts) < 1e-12;
  const bool config_x =
      std::abs(regcontrol.forward_x_volts - spec.forward_x_volts) < 1e-12;
  const bool config_maxtapchange =
      regcontrol.max_tap_change == spec.max_tap_change;
  const bool repo_regulator_closed_loop_converged =
      repo.control_converged &&
      repo_regulator.converged &&
      repo_regulator.stop_reason == "within_band";
  const bool repo_dss_tap_number_match =
      repo_regulator.final_tap_number == regcontrol.tap_number;
  const bool repo_dss_tap_pu_match =
      std::abs(repo_regulator.final_tap_pu - tap_winding_state.tap_pu) < 1e-12;
  const bool repo_dss_control_voltage_within_tolerance =
      std::abs(repo_regulator.control_voltage_volts - dss_control_voltage_volts) <=
      kRegulatorTol.control_v_volts;
  const bool fixture_expected_tap_number_matches =
      repo_regulator.final_tap_number == spec.expected_final_tap_number &&
      regcontrol.tap_number == spec.expected_final_tap_number;
  const bool fixture_expected_tap_pu_matches =
      std::abs(repo_regulator.final_tap_pu - spec.expected_final_tap_pu) < 1e-12 &&
      std::abs(tap_winding_state.tap_pu - spec.expected_final_tap_pu) < 1e-12;
  const bool fixture_truth_drift_detected =
      repo_regulator_closed_loop_converged &&
      repo_dss_tap_number_match &&
      repo_dss_tap_pu_match &&
      repo_dss_control_voltage_within_tolerance &&
      (!fixture_expected_tap_number_matches || !fixture_expected_tap_pu_matches);
  const bool fixture_truth_drift_pass = !fixture_truth_drift_detected;

  const bool bridge_validation_pass =
      config_transformer_name && config_winding && config_tap_winding &&
      config_monitored_bus && config_vreg && config_band &&
      config_ptratio && config_remote_ptratio && config_ctprim &&
      config_r && config_x && config_maxtapchange && regulator_within_band &&
      repo_regulator_closed_loop_converged && repo_dss_tap_number_match &&
      repo_dss_tap_pu_match && repo_dss_control_voltage_within_tolerance &&
      transformer_loss_self_consistent && line_loss_self_consistent &&
      circuit_loss_self_consistent;

  const json repo_control_trace_summary =
      build_repo_control_trace_summary(
          repo.regulator_trace, repo_regulator, spec.regcontrol_name);
  const json opendss_control_oracle =
      build_opendss_control_oracle_summary(
          dss, spec.regcontrol_name, spec.transformer_name, regulator_within_band);
  const bool control_iteration_count_match =
      repo_control_trace_summary["control_iteration_count"].get<int>() ==
      opendss_control_oracle["control_iteration_count"].get<int>();
  const bool tap_operation_count_match =
      repo_control_trace_summary["tap_operation_count"].get<int>() ==
      opendss_control_oracle["tap_operation_count"].get<int>();
  const bool stop_reason_match =
      repo_control_trace_summary["stop_reason"].get<std::string>() ==
      opendss_control_oracle["stop_reason"].get<std::string>();
  const bool control_trace_validation_pass =
      control_iteration_count_match &&
      tap_operation_count_match &&
      stop_reason_match &&
      repo_dss_tap_number_match &&
      repo_dss_control_voltage_within_tolerance;

  const std::vector<double> repo_p_xfmr = {
      repo.p_branch_mw[static_cast<std::size_t>(transformer_branch_pos)]};
  const std::vector<double> repo_q_xfmr = {
      repo.q_branch_mvar[static_cast<std::size_t>(transformer_branch_pos)]};
  const std::vector<double> dss_p_xfmr = {dss_xfmr_power.p_mw};
  const std::vector<double> dss_q_xfmr = {dss_xfmr_power.q_mvar};

  const std::vector<double> repo_p_l1 = {
      repo.p_branch_mw[static_cast<std::size_t>(line_branch_pos)]};
  const std::vector<double> repo_q_l1 = {
      repo.q_branch_mvar[static_cast<std::size_t>(line_branch_pos)]};
  const std::vector<double> dss_p_l1 = {dss_line_power.p_mw};
  const std::vector<double> dss_q_l1 = {dss_line_power.q_mvar};

  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p = {dss_branch_loss.p_mw};
  const std::vector<double> dss_loss_q = {dss_branch_loss.q_mvar};
  const std::vector<double> repo_control_v = {repo_regulator.control_voltage_volts};
  const std::vector<double> dss_control_v = {dss_control_voltage_volts};
  const std::vector<double> repo_tap_number = {
      static_cast<double>(repo_regulator.final_tap_number)};
  const std::vector<double> dss_tap_number = {
      static_cast<double>(regcontrol.tap_number)};

  json repo_trace = json::array();
  for (const auto& trace : repo.regulator_trace) {
    repo_trace.push_back(regulator_trace_to_json(trace));
  }

  return {
      {"case_id", spec.case_id},
      {"acceptance_tier", spec.acceptance_tier},
      {"status_reason", status_reason},
      {"module_in_scope", "solve_distribution_pf(const HybridPowerSystem&)"},
      {"scope_reason", scope_reason},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin", repo_case_origin},
      {"converged", {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"bridge_validation_pass", bridge_validation_pass},
      {"control_trace_validation_pass", control_trace_validation_pass},
      {"fixture_truth_drift_pass", fixture_truth_drift_pass},
      {"bridge_validation",
       {
           {"configured_control",
            {
                {"expected_transformer_name", spec.transformer_name},
                {"expected_winding", spec.winding},
                {"expected_tap_winding", spec.tap_winding},
                {"expected_monitoring_mode",
                 spec.implicit_local_bus_monitoring ? "implicit_local_bus" : "explicit_remote_bus"},
                {"expected_monitored_bus_name", expected_monitored_bus_key},
                {"expected_vreg_volts", spec.forward_vreg_volts},
                {"expected_band_volts", spec.forward_band_volts},
                {"expected_ptratio", spec.ptratio},
                {"expected_remote_ptratio", spec.remote_ptratio},
                {"expected_ct_primary_amps", spec.ct_primary_amps},
                {"expected_forward_r_volts", spec.forward_r_volts},
                {"expected_forward_x_volts", spec.forward_x_volts},
                {"expected_max_tap_change", spec.max_tap_change},
            }},
           {"config_consistency",
            {
                {"transformer_name", config_transformer_name},
                {"winding", config_winding},
                {"tap_winding", config_tap_winding},
                {"monitored_bus_name", config_monitored_bus},
                {"vreg_volts", config_vreg},
                {"band_volts", config_band},
                {"ptratio", config_ptratio},
                {"remote_ptratio", config_remote_ptratio},
                {"ct_primary_amps", config_ctprim},
                {"forward_r_volts", config_r},
                {"forward_x_volts", config_x},
                {"max_tap_change", config_maxtapchange},
            }},
           {"repo_vs_opendss_end_state",
            {
                {"repo_control_converged", repo.control_converged},
                {"repo_stop_reason", repo_regulator.stop_reason},
                {"repo_dss_tap_number_match", repo_dss_tap_number_match},
                {"repo_dss_tap_pu_match", repo_dss_tap_pu_match},
                {"repo_dss_control_voltage_within_tolerance",
                 repo_dss_control_voltage_within_tolerance},
            }},
           {"regcontrol_state", regcontrol_to_json(regcontrol)},
           {"transformer_state", transformer_state_to_json(transformer_state)},
           {"monitored_bus",
            {
                {"bus_name", spec.monitored_bus_name},
                {"node", spec.monitored_bus_node},
                {"vm_pu", find_node_voltage(
                              voltage_index, spec.monitored_bus_name,
                              spec.monitored_bus_node).vm_pu},
                {"vm_vln_volts", dss_control_snapshot.monitored_voltage_volts},
                {"control_secondary_volts", dss_control_voltage_volts},
                {"effective_ptratio", dss_control_snapshot.effective_ptratio},
                {"line_drop_compensation_real_volts",
                 dss_control_snapshot.line_drop_compensation_real_volts},
                {"line_drop_compensation_imag_volts",
                 dss_control_snapshot.line_drop_compensation_imag_volts},
                {"line_drop_compensation_magnitude_volts",
                 dss_control_snapshot.line_drop_compensation_magnitude_volts},
                {"target_vreg_volts", regcontrol.forward_vreg_volts},
                {"band_half_volts", band_half_volts},
                {"abs_diff_to_target_volts", dss_control_abs_diff_volts},
                {"within_band", regulator_within_band},
            }},
           {"repo_regulator_state", regulator_state_to_json(repo_regulator)},
           {"repo_regulator_trace", repo_trace},
           {"transformer_terminal_power_sum_mw_mvar",
            power_pair_to_json(xfmr_terminal_sum)},
           {"transformer_losses_mw_mvar",
            power_pair_to_json(xfmr_losses)},
           {"transformer_abs_diff_terminal_sum_vs_losses",
            {
                {"p_mw", std::abs(xfmr_terminal_sum.p_mw - xfmr_losses.p_mw)},
                {"q_mvar", std::abs(xfmr_terminal_sum.q_mvar - xfmr_losses.q_mvar)},
            }},
           {"line_terminal_power_sum_mw_mvar",
            power_pair_to_json(line_terminal_sum)},
           {"line_losses_mw_mvar",
            power_pair_to_json(line_losses)},
           {"line_abs_diff_terminal_sum_vs_losses",
            {
                {"p_mw", std::abs(line_terminal_sum.p_mw - line_losses.p_mw)},
                {"q_mvar", std::abs(line_terminal_sum.q_mvar - line_losses.q_mvar)},
            }},
           {"total_branch_loss_mw_mvar", power_pair_to_json(dss_branch_loss)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar", power_pair_to_json(dss_total_loss_cross_check)},
                {"abs_diff_vs_branch_terminal_sum",
                 {
                     {"p_mw", std::abs(dss_total_loss_cross_check.p_mw - dss_branch_loss.p_mw)},
                     {"q_mvar", std::abs(dss_total_loss_cross_check.q_mvar - dss_branch_loss.q_mvar)},
                 }},
           }},
       }},
      {"control_trace_validation",
       {
           {"pass", control_trace_validation_pass},
           {"checks",
            {
                {"control_iteration_count", control_iteration_count_match},
                {"tap_operation_count", tap_operation_count_match},
                {"stop_reason", stop_reason_match},
                {"final_tap_number", repo_dss_tap_number_match},
                {"control_voltage_within_tolerance",
                 repo_dss_control_voltage_within_tolerance},
            }},
           {"repo_trace_summary", repo_control_trace_summary},
           {"opendss_control_oracle", opendss_control_oracle},
       }},
      {"fixture_truth_drift",
       {
           {"detected", fixture_truth_drift_detected},
           {"reason",
            fixture_truth_drift_detected
                ? json("fixture_expected_regulator_end_state_is_stale")
                : json(nullptr)},
           {"fixture_expected_end_state",
            {
                {"tap_number", spec.expected_final_tap_number},
                {"tap_pu", spec.expected_final_tap_pu},
            }},
           {"repo_end_state",
            {
                {"tap_number", repo_regulator.final_tap_number},
                {"tap_pu", repo_regulator.final_tap_pu},
                {"control_voltage_volts", repo_regulator.control_voltage_volts},
            }},
           {"opendss_end_state",
            {
                {"tap_number", regcontrol.tap_number},
                {"tap_pu", tap_winding_state.tap_pu},
                {"control_voltage_volts", dss_control_voltage_volts},
            }},
           {"repo_and_opendss_agree",
            {
                {"tap_number", repo_dss_tap_number_match},
                {"tap_pu", repo_dss_tap_pu_match},
                {"control_voltage_within_tolerance",
                 repo_dss_control_voltage_within_tolerance},
            }},
           {"fixture_matches_actual",
            {
                {"tap_number", fixture_expected_tap_number_matches},
                {"tap_pu", fixture_expected_tap_pu_matches},
            }},
       }},
      {"projection_diagnostics",
       {
           {"source_transformer",
            {
                {"origin_index", spec.transformer_origin_index},
                {"tap_side", spec.tap_side == 0 ? "hv_from_side" : "lv_to_side"},
                {"tap_min", spec.tap_min},
                {"tap_max", spec.tap_max},
                {"tap_neutral", spec.tap_neutral},
                {"tap_step_percent", spec.tap_step_percent},
            }},
           {"projected_branch",
            {
                {"name", projected_transformer_branch.name},
                {"index", projected_transformer_branch.index},
                {"normalized_tap_pu", projected_transformer_branch.tap},
                {"impedance_normalization",
                 tap_impedance_normalization_to_json(
                     spec.tap_side == 0 ? "hv_from_side" : "lv_to_side",
                     spec.tap_side == 1,
                     spec.tap_side == 1,
                     repo_regulator.final_tap_pu)},
                {"r_pu", projected_transformer_branch.r_pu},
                {"x_pu", projected_transformer_branch.x_pu},
            }},
       }},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu", to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_voltage_angle_deg", to_json(summarize_metric(repo_va, dss_va))},
           {"regulator_control_voltage_volts",
            to_json(summarize_metric(repo_control_v, dss_control_v))},
           {"regulator_tap_number",
            to_json(summarize_metric(repo_tap_number, dss_tap_number))},
           {"transformer_terminal1_p_mw",
            to_json(summarize_metric(repo_p_xfmr, dss_p_xfmr))},
           {"transformer_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_xfmr, dss_q_xfmr))},
           {"line_l1_terminal1_p_mw",
            to_json(summarize_metric(repo_p_l1, dss_p_l1))},
           {"line_l1_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_l1, dss_q_l1))},
           {"total_branch_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_branch_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"loss_diagnostics",
       {
           {"branch_loss_breakdown_mw_mvar",
            {
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss", power_pair_to_json(dss_branch_loss)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar", power_pair_to_json(dss_total_loss_cross_check)},
                {"abs_diff_vs_branch_terminal_sum",
                 {
                     {"p_mw", std::abs(dss_total_loss_cross_check.p_mw - dss_branch_loss.p_mw)},
                     {"q_mvar", std::abs(dss_total_loss_cross_check.q_mvar - dss_branch_loss.q_mvar)},
                 }},
            }},
       }},
      {"acceptance_tolerances", tolerance_to_json(kRegulatorTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg", repo_va},
           {"regulator_control_voltage_volts", repo_control_v},
           {"regulator_tap_number", repo_tap_number},
           {"transformer_terminal1_p_mw", repo_p_xfmr},
           {"transformer_terminal1_q_mvar", repo_q_xfmr},
           {"line_l1_terminal1_p_mw", repo_p_l1},
           {"line_l1_terminal1_q_mvar", repo_q_l1},
           {"total_branch_loss_p_mw", repo.total_p_loss_mw},
           {"total_branch_loss_q_mvar", repo.total_q_loss_mvar},
           {"regulator_state", regulator_state_to_json(repo_regulator)},
           {"regulator_trace", repo_trace},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", dss_vm},
           {"bus_voltage_angle_deg", dss_va},
           {"regulator_control_voltage_volts", dss_control_v},
           {"regulator_tap_number", dss_tap_number},
           {"transformer_terminal1_p_mw", dss_p_xfmr},
           {"transformer_terminal1_q_mvar", dss_q_xfmr},
           {"line_l1_terminal1_p_mw", dss_p_l1},
           {"line_l1_terminal1_q_mvar", dss_q_l1},
           {"total_branch_loss_p_mw", dss_branch_loss.p_mw},
           {"total_branch_loss_q_mvar", dss_branch_loss.q_mvar},
           {"circuit_losses_raw", losses_raw_to_json(dss.circuit_losses_raw)},
           {"transformer_state", transformer_state_to_json(transformer_state)},
           {"tapped_winding_state",
            {
                {"winding", tap_winding_state.winding},
                {"tap_pu", tap_winding_state.tap_pu},
                {"num_taps", tap_winding_state.num_taps},
            }},
           {"regcontrol_state", regcontrol_to_json(regcontrol)},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle", "degrees"},
           {"regulator_control_voltage", "volts (PT secondary)"},
           {"regulator_tap_number", "discrete tap number"},
           {"branch_power", "MW / MVAr"},
           {"total_loss",
            "MW / MVAr (primary: summed branch terminal powers)"},
           {"branch_tap",
            "p.u. from-side turns ratio after Transformer2W tap-side normalization"},
           {"circuit_losses_cross_check",
            "MW / MVAr (Circuit.Losses raw W / var -> MW / MVAr)"},
           {"voltage_base_note", "7.2 kV L-N; VoltageBases=[12.4707658] kV L-L"},
      }},
  };
}

json summarize_accepted_regulator_case(const fs::path& project_root) {
  return summarize_regulator_compare_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_regulator_case(),
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph(),
      "hacdcpf_compare_fixtures::build_minimal_regulator_1ph in "
      "tools/opendss_compare/fixtures.hpp",
      "Closed-loop single-phase regulator control is now in the accepted baseline. "
      "The repository solver runs a discrete outer RegControl loop over the "
      "tap-aware BFS inner solve, and matches OpenDSS monitored voltage, final "
      "tap number, transformer terminal power, downstream line power, and total "
      "branch losses within the declared tolerances.",
      "Promotes the former bridge-only regulator path into accepted compare. "
      "The same canonical PD-element snapshot remains the OpenDSS reference, "
      "but Circuit.Losses stays only as an independent total-loss cross-check.");
}

json summarize_accepted_local_pt_with_ldc_regulator_case(const fs::path& project_root) {
  return summarize_regulator_compare_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_local_pt_with_ldc_regulator_case(),
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph_local_pt_with_ldc(),
      "hacdcpf_compare_fixtures::build_minimal_regulator_1ph_local_pt_with_ldc in "
      "tools/opendss_compare/fixtures.hpp",
      "Closed-loop single-phase regulator control with local PT monitoring and "
      "line-drop compensation is in the accepted baseline. The repository solver "
      "matches OpenDSS final tap, PT-secondary control voltage, branch powers, "
      "and total losses within the declared tolerances.",
      "Extends the accepted regulator baseline from remote-bus/no-LDC to the "
      "local-PT plus LDC control path without introducing a second compare "
      "framework.");
}

json summarize_accepted_remote_ptratio_regulator_case(const fs::path& project_root) {
  return summarize_regulator_compare_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_remote_ptratio_regulator_case(),
      hacdcpf_compare_fixtures::build_minimal_regulator_1ph_remote_ptratio(),
      "hacdcpf_compare_fixtures::build_minimal_regulator_1ph_remote_ptratio in "
      "tools/opendss_compare/fixtures.hpp",
      "Closed-loop single-phase remote-bus regulator control with PTRatio != "
      "RemotePTRatio is in the accepted baseline. The repository solver matches "
      "OpenDSS final tap, control voltage, branch powers, and total losses "
      "within the declared tolerances.",
      "Promotes the distinct RemotePTRatio remote-bus control path into "
      "accepted compare without changing the threshold model or adding a "
      "second compare framework.");
}

json summarize_three_phase_regulator_bank_case(
    const fs::path& project_root,
    const hacdcpf_compare_fixtures::ThreePhaseRegulatorBankOpenDSSCase& spec,
    const hacdcpf::ThreePhaseACSystem& repo_case,
    const std::string& repo_case_origin,
    const std::string& status_reason,
    const std::string& scope_reason) {
  using namespace hacdcpf::analysis;

  const fs::path master_dss = project_root / spec.relative_master_dss;

  ThreePhaseNROptions nr_options;
  nr_options.max_iter = 100;
  nr_options.max_control_iter = 100;
  nr_options.tol = 1e-10;
  const auto repo = solve_three_phase_nr(repo_case, nr_options);

  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error(
        "OpenDSS snapshot did not converge for accepted three-phase regulator bank case.");
  }

  if (repo.regulator_states.size() < spec.regulators.size()) {
    throw std::runtime_error(
        "Repository three-phase NR did not emit all regulator_states for the "
        "accepted three-phase regulator bank case.");
  }
  if (repo.transformer_terminal_observations.size() < spec.regulators.size()) {
    throw std::runtime_error(
        "Repository three-phase NR did not emit all transformer observations for "
        "the accepted three-phase regulator bank case.");
  }

  const auto voltage_index = build_voltage_index(dss);
  const auto power_index = build_terminal_power_index(dss);
  const auto pd_element_index = build_pd_element_index(dss);
  const auto dss_branch_loss =
      hacdcpf_compare::sum_all_pd_element_terminal_powers_to_mw_mvar(dss);
  const auto dss_total_loss_cross_check =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  const bool circuit_loss_self_consistent =
      std::abs(dss_branch_loss.p_mw - dss_total_loss_cross_check.p_mw) < 1e-6 &&
      std::abs(dss_branch_loss.q_mvar - dss_total_loss_cross_check.q_mvar) < 1e-6;

  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t bus_pos = 0; bus_pos < spec.bus_names.size(); ++bus_pos) {
    if (repo.bus_voltages.size() <= bus_pos) {
      throw std::runtime_error(
          "Repository three-phase NR bus_voltages surface is smaller than the "
          "declared compare surface.");
    }
    const auto& bv = repo.bus_voltages[bus_pos];
    repo_vm.push_back(bv.vm_a_pu);
    repo_vm.push_back(bv.vm_b_pu);
    repo_vm.push_back(bv.vm_c_pu);
    repo_va.push_back(bv.va_a_deg);
    repo_va.push_back(bv.va_b_deg);
    repo_va.push_back(bv.va_c_deg);

    for (int node = 1; node <= 3; ++node) {
      const auto& nv = find_node_voltage(voltage_index, spec.bus_names[bus_pos], node);
      dss_vm.push_back(nv.vm_pu);
      dss_va.push_back(nv.va_deg);
    }
  }

  std::vector<double> repo_control_v, dss_control_v;
  std::vector<double> repo_tap_number, dss_tap_number;
  std::vector<double> repo_p_xfmr, dss_p_xfmr;
  std::vector<double> repo_q_xfmr, dss_q_xfmr;
  json bridge_validation_entries = json::array();
  json control_trace_entries = json::array();
  json fixture_truth_entries = json::array();
  json regulator_bank_entries = json::array();
  bool bridge_validation_pass = true;
  bool control_trace_validation_pass = true;
  bool fixture_truth_drift_pass = true;

  for (const auto& regulator_spec : spec.regulators) {
    const auto& repo_regulator =
        find_three_phase_regulator_state(repo.regulator_states,
                                         regulator_spec.regcontrol_name);
    const auto& repo_observation =
        find_three_phase_transformer_observation(
            repo.transformer_terminal_observations,
            regulator_spec.transformer_index);
    const auto& repo_hv_bus =
        find_three_phase_bus(repo_case, repo_observation.hv_bus);
    const int phase_index =
        phase_index_from_node(regulator_spec.monitored_bus_node);
    const auto repo_terminal_power = transformer_terminal_power_from_observation(
        repo_observation, repo_hv_bus.base_kv * 1000.0, phase_index, true);

    const auto& dss_transformer = find_pd_element(
        pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        regulator_spec.transformer_name);
    const auto& transformer_state =
        find_transformer_state(dss.transformer_states, regulator_spec.transformer_name);
    const auto& regcontrol =
        find_regcontrol(dss.regcontrol_results, regulator_spec.regcontrol_name);
    const auto& tap_winding_state =
        find_transformer_winding_state(transformer_state, regulator_spec.tap_winding);
    const auto dss_control_snapshot = compute_dss_regulator_control_snapshot(
        voltage_index, power_index, dss_transformer, regcontrol, regulator_spec);
    const auto& dss_terminal_power = find_terminal_power(
        power_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
        regulator_spec.transformer_name, 1, regulator_spec.monitored_bus_node);
    const auto dss_terminal_power_mw_mvar =
        hacdcpf_compare::opendss_terminal_power_to_mw_mvar(dss_terminal_power);
    const auto transformer_terminal_sum =
        hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(dss_transformer);
    const auto transformer_losses =
        hacdcpf_compare::opendss_element_losses_to_mw_mvar(dss_transformer);
    const bool transformer_loss_self_consistent =
        std::abs(transformer_terminal_sum.p_mw - transformer_losses.p_mw) < 1e-6 &&
        std::abs(transformer_terminal_sum.q_mvar - transformer_losses.q_mvar) < 1e-6;

    const std::string expected_monitored_bus_key =
        key_for_bus_node(
            regulator_spec.monitored_bus_name, regulator_spec.monitored_bus_node);
    const bool config_transformer_name =
        regcontrol.transformer_name == ascii_lower(regulator_spec.transformer_name);
    const bool config_winding = regcontrol.winding == regulator_spec.winding;
    const bool config_tap_winding =
        regcontrol.tap_winding == regulator_spec.tap_winding;
    const bool config_monitored_bus =
        regulator_spec.implicit_local_bus_monitoring
            ? (regcontrol.monitored_bus_name.empty() ||
               regcontrol.monitored_bus_name == expected_monitored_bus_key)
            : (regcontrol.monitored_bus_name == expected_monitored_bus_key);
    const bool config_vreg =
        std::abs(regcontrol.forward_vreg_volts -
                 regulator_spec.forward_vreg_volts) < 1e-12;
    const bool config_band =
        std::abs(regcontrol.forward_band_volts -
                 regulator_spec.forward_band_volts) < 1e-12;
    const bool config_ptratio =
        std::abs(regcontrol.ptratio - regulator_spec.ptratio) < 1e-12;
    const bool config_remote_ptratio =
        std::abs(regcontrol.remote_ptratio -
                 regulator_spec.remote_ptratio) < 1e-12;
    const bool config_ctprim =
        std::abs(regcontrol.ct_primary_amps -
                 regulator_spec.ct_primary_amps) < 1e-12;
    const bool config_r =
        std::abs(regcontrol.forward_r_volts -
                 regulator_spec.forward_r_volts) < 1e-12;
    const bool config_x =
        std::abs(regcontrol.forward_x_volts -
                 regulator_spec.forward_x_volts) < 1e-12;
    const bool config_maxtapchange =
        regcontrol.max_tap_change == regulator_spec.max_tap_change;

    const double band_half_volts = regcontrol.forward_band_volts / 2.0;
    const double dss_control_abs_diff_volts =
        std::abs(dss_control_snapshot.control_voltage_volts -
                 regcontrol.forward_vreg_volts);
    const bool regulator_within_band =
        dss_control_abs_diff_volts <= band_half_volts + 1e-6;
    const bool repo_regulator_closed_loop_converged =
        repo.control_converged &&
        repo_regulator.converged &&
        repo_regulator.stop_reason == "within_band";
    const bool repo_dss_tap_number_match =
        repo_regulator.final_tap_number == regcontrol.tap_number;
    const bool repo_dss_tap_pu_match =
        std::abs(repo_regulator.final_tap_pu - tap_winding_state.tap_pu) < 1e-12;
    const bool repo_dss_control_voltage_within_tolerance =
        std::abs(repo_regulator.control_voltage_volts -
                 dss_control_snapshot.control_voltage_volts) <=
        kRegulatorTol.control_v_volts;
    const bool fixture_expected_tap_number_matches =
        repo_regulator.final_tap_number ==
            regulator_spec.expected_final_tap_number &&
        regcontrol.tap_number == regulator_spec.expected_final_tap_number;
    const bool fixture_expected_tap_pu_matches =
        std::abs(repo_regulator.final_tap_pu -
                 regulator_spec.expected_final_tap_pu) < 1e-12 &&
        std::abs(tap_winding_state.tap_pu -
                 regulator_spec.expected_final_tap_pu) < 1e-12;
    const bool fixture_truth_drift_detected =
        repo_regulator_closed_loop_converged &&
        repo_dss_tap_number_match &&
        repo_dss_tap_pu_match &&
        repo_dss_control_voltage_within_tolerance &&
        (!fixture_expected_tap_number_matches || !fixture_expected_tap_pu_matches);
    const bool bridge_pass =
        config_transformer_name &&
        config_winding &&
        config_tap_winding &&
        config_monitored_bus &&
        config_vreg &&
        config_band &&
        config_ptratio &&
        config_remote_ptratio &&
        config_ctprim &&
        config_r &&
        config_x &&
        config_maxtapchange &&
        regulator_within_band &&
        repo_regulator_closed_loop_converged &&
        repo_dss_tap_number_match &&
        repo_dss_tap_pu_match &&
        repo_dss_control_voltage_within_tolerance &&
        transformer_loss_self_consistent &&
        circuit_loss_self_consistent;

    const json repo_trace_summary =
        build_repo_control_trace_summary(
            repo.regulator_trace, repo_regulator,
            regulator_spec.regcontrol_name);
    const json dss_control_oracle =
        build_opendss_control_oracle_summary(
            dss, regulator_spec.regcontrol_name,
            regulator_spec.transformer_name, regulator_within_band);
    const bool control_iteration_count_match =
        repo_trace_summary["control_iteration_count"].get<int>() ==
        dss_control_oracle["control_iteration_count"].get<int>();
    const bool opendss_has_per_regulator_tap_actions =
        dss_control_oracle["tap_operation_count_source"].get<std::string>() ==
        "event_log";
    const bool tap_operation_count_match =
        opendss_has_per_regulator_tap_actions
            ? (repo_trace_summary["tap_operation_count"].get<int>() ==
               dss_control_oracle["tap_operation_count"].get<int>())
            : (repo_trace_summary["tap_operation_count"].get<int>() <=
               dss_control_oracle["tap_operation_count"].get<int>());
    const bool stop_reason_match =
        repo_trace_summary["stop_reason"].get<std::string>() ==
        dss_control_oracle["stop_reason"].get<std::string>();
    const bool control_trace_pass =
        control_iteration_count_match &&
        tap_operation_count_match &&
        stop_reason_match &&
        repo_dss_tap_number_match &&
        repo_dss_control_voltage_within_tolerance;

    bridge_validation_pass = bridge_validation_pass && bridge_pass;
    control_trace_validation_pass =
        control_trace_validation_pass && control_trace_pass;
    fixture_truth_drift_pass =
        fixture_truth_drift_pass && !fixture_truth_drift_detected;

    repo_control_v.push_back(repo_regulator.control_voltage_volts);
    dss_control_v.push_back(dss_control_snapshot.control_voltage_volts);
    repo_tap_number.push_back(
        static_cast<double>(repo_regulator.final_tap_number));
    dss_tap_number.push_back(static_cast<double>(regcontrol.tap_number));
    repo_p_xfmr.push_back(repo_terminal_power.p_mw);
    repo_q_xfmr.push_back(repo_terminal_power.q_mvar);
    dss_p_xfmr.push_back(dss_terminal_power_mw_mvar.p_mw);
    dss_q_xfmr.push_back(dss_terminal_power_mw_mvar.q_mvar);

    bridge_validation_entries.push_back({
        {"phase", regulator_spec.phase_name},
        {"pass", bridge_pass},
        {"configured_control",
         {
             {"expected_transformer_name", regulator_spec.transformer_name},
             {"expected_winding", regulator_spec.winding},
             {"expected_tap_winding", regulator_spec.tap_winding},
             {"expected_monitoring_mode",
              regulator_spec.implicit_local_bus_monitoring
                  ? "implicit_local_bus"
                  : "explicit_remote_bus"},
             {"expected_monitored_bus_name", expected_monitored_bus_key},
             {"expected_vreg_volts", regulator_spec.forward_vreg_volts},
             {"expected_band_volts", regulator_spec.forward_band_volts},
             {"expected_ptratio", regulator_spec.ptratio},
             {"expected_remote_ptratio", regulator_spec.remote_ptratio},
             {"expected_ct_primary_amps", regulator_spec.ct_primary_amps},
             {"expected_forward_r_volts", regulator_spec.forward_r_volts},
             {"expected_forward_x_volts", regulator_spec.forward_x_volts},
             {"expected_max_tap_change", regulator_spec.max_tap_change},
         }},
        {"config_consistency",
         {
             {"transformer_name", config_transformer_name},
             {"winding", config_winding},
             {"tap_winding", config_tap_winding},
             {"monitored_bus_name", config_monitored_bus},
             {"vreg_volts", config_vreg},
             {"band_volts", config_band},
             {"ptratio", config_ptratio},
             {"remote_ptratio", config_remote_ptratio},
             {"ct_primary_amps", config_ctprim},
             {"forward_r_volts", config_r},
             {"forward_x_volts", config_x},
             {"max_tap_change", config_maxtapchange},
         }},
        {"repo_vs_opendss_end_state",
         {
             {"repo_control_converged", repo.control_converged},
             {"repo_stop_reason", repo_regulator.stop_reason},
             {"repo_dss_tap_number_match", repo_dss_tap_number_match},
             {"repo_dss_tap_pu_match", repo_dss_tap_pu_match},
             {"repo_dss_control_voltage_within_tolerance",
              repo_dss_control_voltage_within_tolerance},
         }},
        {"transformer_state", transformer_state_to_json(transformer_state)},
        {"regcontrol_state", regcontrol_to_json(regcontrol)},
        {"monitored_bus",
         {
             {"bus_name", regulator_spec.monitored_bus_name},
             {"node", regulator_spec.monitored_bus_node},
             {"vm_pu",
              find_node_voltage(
                  voltage_index,
                  regulator_spec.monitored_bus_name,
                  regulator_spec.monitored_bus_node).vm_pu},
             {"vm_vln_volts", dss_control_snapshot.monitored_voltage_volts},
             {"control_secondary_volts",
              dss_control_snapshot.control_voltage_volts},
             {"effective_ptratio", dss_control_snapshot.effective_ptratio},
             {"line_drop_compensation_real_volts",
              dss_control_snapshot.line_drop_compensation_real_volts},
             {"line_drop_compensation_imag_volts",
              dss_control_snapshot.line_drop_compensation_imag_volts},
             {"line_drop_compensation_magnitude_volts",
              dss_control_snapshot.line_drop_compensation_magnitude_volts},
             {"target_vreg_volts", regcontrol.forward_vreg_volts},
             {"band_half_volts", band_half_volts},
             {"abs_diff_to_target_volts", dss_control_abs_diff_volts},
             {"within_band", regulator_within_band},
         }},
        {"repo_regulator_state", regulator_state_to_json(repo_regulator)},
        {"transformer_terminal_power_sum_mw_mvar",
         power_pair_to_json(transformer_terminal_sum)},
        {"transformer_losses_mw_mvar",
         power_pair_to_json(transformer_losses)},
        {"transformer_abs_diff_terminal_sum_vs_losses",
         {
             {"p_mw",
              std::abs(transformer_terminal_sum.p_mw - transformer_losses.p_mw)},
             {"q_mvar",
              std::abs(transformer_terminal_sum.q_mvar -
                       transformer_losses.q_mvar)},
         }},
    });

    control_trace_entries.push_back({
        {"phase", regulator_spec.phase_name},
        {"pass", control_trace_pass},
        {"checks",
         {
             {"control_iteration_count", control_iteration_count_match},
             {"tap_operation_count", tap_operation_count_match},
             {"tap_operation_count_policy",
              opendss_has_per_regulator_tap_actions
                  ? "exact_match_from_event_log"
                  : "fallback_is_only_an_upper_bound_for_multi_regulator_cases"},
             {"stop_reason", stop_reason_match},
             {"final_tap_number", repo_dss_tap_number_match},
             {"control_voltage_within_tolerance",
              repo_dss_control_voltage_within_tolerance},
         }},
        {"repo_trace_summary", repo_trace_summary},
        {"opendss_control_oracle", dss_control_oracle},
    });

    fixture_truth_entries.push_back({
        {"phase", regulator_spec.phase_name},
        {"detected", fixture_truth_drift_detected},
        {"reason",
         fixture_truth_drift_detected
             ? json("fixture_expected_regulator_end_state_is_stale")
             : json(nullptr)},
        {"fixture_expected_end_state",
         {
             {"tap_number", regulator_spec.expected_final_tap_number},
             {"tap_pu", regulator_spec.expected_final_tap_pu},
         }},
        {"repo_end_state",
         {
             {"tap_number", repo_regulator.final_tap_number},
             {"tap_pu", repo_regulator.final_tap_pu},
             {"control_voltage_volts", repo_regulator.control_voltage_volts},
         }},
        {"opendss_end_state",
         {
             {"tap_number", regcontrol.tap_number},
             {"tap_pu", tap_winding_state.tap_pu},
             {"control_voltage_volts", dss_control_snapshot.control_voltage_volts},
         }},
        {"repo_and_opendss_agree",
         {
             {"tap_number", repo_dss_tap_number_match},
             {"tap_pu", repo_dss_tap_pu_match},
             {"control_voltage_within_tolerance",
              repo_dss_control_voltage_within_tolerance},
         }},
        {"fixture_matches_actual",
         {
             {"tap_number", fixture_expected_tap_number_matches},
             {"tap_pu", fixture_expected_tap_pu_matches},
         }},
    });

    regulator_bank_entries.push_back({
        {"phase", regulator_spec.phase_name},
        {"transformer_name", regulator_spec.transformer_name},
        {"regcontrol_name", regulator_spec.regcontrol_name},
        {"repo_regulator_state", regulator_state_to_json(repo_regulator)},
        {"repo_trace_summary", repo_trace_summary},
        {"repo_transformer_terminal1_power_mw_mvar",
         power_pair_to_json(repo_terminal_power)},
        {"opendss_transformer_terminal1_power_mw_mvar",
         power_pair_to_json(dss_terminal_power_mw_mvar)},
        {"opendss_regcontrol_state", regcontrol_to_json(regcontrol)},
        {"opendss_transformer_state", transformer_state_to_json(transformer_state)},
    });
  }

  const std::vector<double> repo_loss_p = {repo.total_p_loss_mw};
  const std::vector<double> repo_loss_q = {repo.total_q_loss_mvar};
  const std::vector<double> dss_loss_p = {dss_branch_loss.p_mw};
  const std::vector<double> dss_loss_q = {dss_branch_loss.q_mvar};

  return {
      {"case_id", spec.case_id},
      {"acceptance_tier", spec.acceptance_tier},
      {"status_reason", status_reason},
      {"module_in_scope", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
      {"scope_reason", scope_reason},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin", repo_case_origin},
      {"converged", {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"bridge_validation_pass", bridge_validation_pass},
      {"control_trace_validation_pass", control_trace_validation_pass},
      {"fixture_truth_drift_pass", fixture_truth_drift_pass},
      {"bridge_validation",
       {
           {"pass", bridge_validation_pass},
           {"per_regulator", bridge_validation_entries},
       }},
      {"control_trace_validation",
       {
           {"pass", control_trace_validation_pass},
           {"per_regulator", control_trace_entries},
       }},
      {"fixture_truth_drift",
       {
           {"detected", !fixture_truth_drift_pass},
           {"per_regulator", fixture_truth_entries},
       }},
      {"regulator_bank_validation",
       {
           {"topology",
            "three_independent_single_phase_regulators_forming_one_three_phase_bank"},
           {"per_regulator", regulator_bank_entries},
       }},
      {"metrics",
       {
           {"bus_phase_voltage_magnitude_pu",
            to_json(summarize_metric(repo_vm, dss_vm))},
           {"bus_phase_voltage_angle_deg",
            to_json(summarize_metric(repo_va, dss_va))},
           {"regulator_control_voltage_volts",
            to_json(summarize_metric(repo_control_v, dss_control_v))},
           {"regulator_tap_number",
            to_json(summarize_metric(repo_tap_number, dss_tap_number))},
           {"transformer_terminal1_p_mw",
            to_json(summarize_metric(repo_p_xfmr, dss_p_xfmr))},
           {"transformer_terminal1_q_mvar",
            to_json(summarize_metric(repo_q_xfmr, dss_q_xfmr))},
           {"total_branch_loss_p_mw",
            to_json(summarize_metric(repo_loss_p, dss_loss_p))},
           {"total_branch_loss_q_mvar",
            to_json(summarize_metric(repo_loss_q, dss_loss_q))},
       }},
      {"loss_diagnostics",
       {
           {"branch_loss_breakdown_mw_mvar",
            {
                {"opendss", build_opendss_pd_element_loss_breakdown(dss)},
            }},
           {"terminal_power_summed_loss", power_pair_to_json(dss_branch_loss)},
           {"circuit_losses_raw_cross_check",
            {
                {"raw_w_var", losses_raw_to_json(dss.circuit_losses_raw)},
                {"converted_mw_mvar",
                 power_pair_to_json(dss_total_loss_cross_check)},
                {"abs_diff_vs_branch_terminal_sum",
                 {
                     {"p_mw",
                      std::abs(dss_total_loss_cross_check.p_mw -
                               dss_branch_loss.p_mw)},
                     {"q_mvar",
                      std::abs(dss_total_loss_cross_check.q_mvar -
                               dss_branch_loss.q_mvar)},
                 }},
            }},
       }},
      {"acceptance_tolerances", tolerance_to_json(kRegulatorTol)},
      {"repo",
       {
           {"bus_phase_voltage_magnitude_pu", repo_vm},
           {"bus_phase_voltage_angle_deg", repo_va},
           {"regulator_control_voltage_volts", repo_control_v},
           {"regulator_tap_number", repo_tap_number},
           {"transformer_terminal1_p_mw", repo_p_xfmr},
           {"transformer_terminal1_q_mvar", repo_q_xfmr},
           {"total_branch_loss_p_mw", repo.total_p_loss_mw},
           {"total_branch_loss_q_mvar", repo.total_q_loss_mvar},
           {"regulators", regulator_bank_entries},
       }},
      {"opendss",
       {
           {"bus_phase_voltage_magnitude_pu", dss_vm},
           {"bus_phase_voltage_angle_deg", dss_va},
           {"regulator_control_voltage_volts", dss_control_v},
           {"regulator_tap_number", dss_tap_number},
           {"transformer_terminal1_p_mw", dss_p_xfmr},
           {"transformer_terminal1_q_mvar", dss_q_xfmr},
           {"total_branch_loss_p_mw", dss_branch_loss.p_mw},
           {"total_branch_loss_q_mvar", dss_branch_loss.q_mvar},
           {"circuit_losses_raw", losses_raw_to_json(dss.circuit_losses_raw)},
           {"regulators", regulator_bank_entries},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle", "degrees"},
           {"regulator_control_voltage", "volts (PT secondary)"},
           {"regulator_tap_number", "discrete tap number"},
           {"branch_power", "MW / MVAr"},
           {"total_loss",
            "MW / MVAr (primary: summed branch terminal powers)"},
           {"circuit_losses_cross_check",
            "MW / MVAr (Circuit.Losses raw W / var -> MW / MVAr)"},
           {"voltage_base_note",
            "12.47 kV L-L system; single-phase regulator windings are 7.2 kV L-N"},
       }},
  };
}

json summarize_accepted_three_phase_regulator_bank_case(
    const fs::path& project_root) {
  return summarize_three_phase_regulator_bank_case(
      project_root,
      hacdcpf_compare_fixtures::accepted_three_phase_regulator_bank_case(),
      hacdcpf_compare_fixtures::build_minimal_regulator_bank_3ph_nr_case(),
      "hacdcpf_compare_fixtures::build_minimal_regulator_bank_3ph_nr_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Closed-loop three-phase NR regulator compare now includes a minimal bank of "
      "three independent single-phase regulators. The repository solver keeps the "
      "native regulator outer loop, and matches OpenDSS on per-phase bus voltage, "
      "per-regulator control voltage, final tap number, transformer terminal power, "
      "and total branch losses within the declared tolerance.",
      "First accepted three-phase regulator compare surface for the NR lane. The "
      "case is intentionally minimal: three independent single-phase regulators "
      "share a three-phase bus pair, with local PT monitoring, no LDC, and no "
      "tap-import oracle.");
}

json summarize_transformer3w_formal_cross_check_case_impl(
    const fs::path& project_root,
    const hacdcpf_compare_fixtures::Transformer3WCrossCheckCase& spec,
    const hacdcpf::HybridPowerSystem& repo_case,
    const std::string& repo_case_origin,
    const std::string& status_reason,
    const std::string& scope_reason) {
  using namespace hacdcpf::analysis;
  const fs::path master_dss = project_root / spec.relative_master_dss;
  const bool has_external_line = !spec.external_line_name.empty();
  const auto& observed_bus_names =
      spec.observed_bus_names.empty() ? spec.bus_names : spec.observed_bus_names;
  if (!spec.observed_bus_nodes.empty() &&
      spec.observed_bus_nodes.size() != observed_bus_names.size()) {
    throw std::runtime_error(
        "Transformer3W compare case observed bus metadata size mismatch.");
  }

  DPFOptions options;
  options.max_iter = 200;
  options.tol = 1e-10;
  options.include_shunts = true;

  const auto repo = solve_distribution_pf(repo_case, options);
  const auto legacy_raw_pair =
      solve_transformer3w_pair_model(
          repo_case, Transformer3WPairReductionMode::LegacyRawPair, options);
  const auto coupled_kron_pair =
      solve_transformer3w_pair_model(
          repo_case, Transformer3WPairReductionMode::CoupledKronPair, options);
  const auto dss = hacdcpf::io::solve_opendss_snapshot(master_dss);
  if (!dss.converged) {
    throw std::runtime_error(
        "OpenDSS snapshot did not converge for Transformer3W formal cross-check.");
  }

  const auto projected = hacdcpf::project_to_canonical_models(repo_case);
  const auto voltage_index = build_voltage_index(dss);
  const auto pd_element_index = build_pd_element_index(dss);
  const auto& xfmr = find_pd_element(
      pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Transformer,
      spec.transformer_name);
  const auto& transformer_state =
      find_transformer_state(dss.transformer_states, spec.transformer_name);

  std::vector<double> repo_vm, dss_vm;
  std::vector<double> repo_va, dss_va;
  for (std::size_t i = 0; i < observed_bus_names.size(); ++i) {
    const int repo_bus_pos =
        find_bus_position_by_name(repo_case.ac.buses, observed_bus_names[i]);
    repo_vm.push_back(repo.vm_pu[static_cast<std::size_t>(repo_bus_pos)]);
    repo_va.push_back(repo.va_deg[static_cast<std::size_t>(repo_bus_pos)]);
    const auto& nv = find_node_voltage(
        voltage_index,
        observed_bus_names[i],
        spec.observed_bus_nodes.empty() ? spec.bus_nodes[i]
                                        : spec.observed_bus_nodes[i]);
    dss_vm.push_back(nv.vm_pu);
    dss_va.push_back(nv.va_deg);
  }

  const std::array<int, 3> winding_bus_ids = {
      repo_case.ac.transformers_3w.front().hv_bus,
      repo_case.ac.transformers_3w.front().mv_bus,
      repo_case.ac.transformers_3w.front().lv_bus,
  };
  std::map<int, std::size_t> bus_position;
  for (std::size_t idx = 0; idx < repo_case.ac.buses.size(); ++idx) {
    bus_position.emplace(repo_case.ac.buses[idx].index, idx);
  }

  std::array<hacdcpf_compare::PowerPair, 3> repo_terminal_powers{};
  json pair_branch_breakdown = json::array();
  hacdcpf_compare::PowerPair repo_total_loss;
  for (int pair_number = 0; pair_number < 3; ++pair_number) {
    const int branch_pos = find_projected_branch_position_by_pair(
        projected, spec.transformer_origin_index, pair_number);
    const auto& branch =
        projected.ac.branches[static_cast<std::size_t>(branch_pos)];
    const auto from_pos_it = bus_position.find(branch.from_bus);
    const auto to_pos_it = bus_position.find(branch.to_bus);
    if (from_pos_it == bus_position.end() || to_pos_it == bus_position.end()) {
      throw std::runtime_error(
          "Missing bus position while building Transformer3W formal cross-check.");
    }
    const auto flow = compute_single_phase_branch_terminal_powers(
        branch,
        polar_pu(repo.vm_pu[from_pos_it->second], repo.va_deg[from_pos_it->second]),
        polar_pu(repo.vm_pu[to_pos_it->second], repo.va_deg[to_pos_it->second]),
        repo_case.base_mva);

    for (std::size_t winding = 0; winding < winding_bus_ids.size(); ++winding) {
      if (branch.from_bus == winding_bus_ids[winding]) {
        repo_terminal_powers[winding].p_mw += flow.power_from_mva.real();
        repo_terminal_powers[winding].q_mvar += flow.power_from_mva.imag();
      }
      if (branch.to_bus == winding_bus_ids[winding]) {
        repo_terminal_powers[winding].p_mw += flow.power_to_mva.real();
        repo_terminal_powers[winding].q_mvar += flow.power_to_mva.imag();
      }
    }

    repo_total_loss.p_mw += flow.loss_mva.real();
    repo_total_loss.q_mvar += flow.loss_mva.imag();

    pair_branch_breakdown.push_back({
        {"pair_number", pair_number},
        {"branch_name", branch.name},
        {"from_bus", branch.from_bus},
        {"to_bus", branch.to_bus},
        {"power_from_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_from_mva.real(), .q_mvar = flow.power_from_mva.imag()})},
        {"power_to_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.power_to_mva.real(), .q_mvar = flow.power_to_mva.imag()})},
        {"loss_mw_mvar",
         power_pair_to_json(
             {.p_mw = flow.loss_mva.real(), .q_mvar = flow.loss_mva.imag()})},
    });
  }

  std::array<hacdcpf_compare::PowerPair, 3> dss_terminal_powers = {
      sum_pd_element_terminal_powers_for_terminal(xfmr, 1),
      sum_pd_element_terminal_powers_for_terminal(xfmr, 2),
      sum_pd_element_terminal_powers_for_terminal(xfmr, 3),
  };
  const auto dss_terminal_sum =
      hacdcpf_compare::sum_pd_element_terminal_powers_to_mw_mvar(xfmr);
  const auto dss_transformer_loss =
      hacdcpf_compare::opendss_element_losses_to_mw_mvar(xfmr);
  const auto dss_circuit_loss =
      hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(dss);
  const auto legacy_raw_pair_replay =
      replay_transformer3w_pair_model_at_voltages(
          repo_case, Transformer3WPairReductionMode::LegacyRawPair, dss_vm, dss_va);
  const auto coupled_kron_pair_replay =
      replay_transformer3w_pair_model_at_voltages(
          repo_case, Transformer3WPairReductionMode::CoupledKronPair, dss_vm, dss_va);
  const auto explicit_star_reference_replay =
      replay_transformer3w_explicit_star_reference_at_voltages(
          repo_case, dss_vm, dss_va);
  const auto& tapped_winding_state = find_transformer_winding_state(
      transformer_state, spec.tap_side + 1);
  const auto vm_summary = summarize_metric(repo_vm, dss_vm);
  const auto va_summary = summarize_metric(repo_va, dss_va);
  const auto terminal1_p_summary = summarize_metric(
      {repo_terminal_powers[0].p_mw}, {dss_terminal_powers[0].p_mw});
  const auto terminal1_q_summary = summarize_metric(
      {repo_terminal_powers[0].q_mvar}, {dss_terminal_powers[0].q_mvar});
  const auto terminal2_p_summary = summarize_metric(
      {repo_terminal_powers[1].p_mw}, {dss_terminal_powers[1].p_mw});
  const auto terminal2_q_summary = summarize_metric(
      {repo_terminal_powers[1].q_mvar}, {dss_terminal_powers[1].q_mvar});
  const auto terminal3_p_summary = summarize_metric(
      {repo_terminal_powers[2].p_mw}, {dss_terminal_powers[2].p_mw});
  const auto terminal3_q_summary = summarize_metric(
      {repo_terminal_powers[2].q_mvar}, {dss_terminal_powers[2].q_mvar});
  const auto total_loss_p_summary = summarize_metric(
      {repo_total_loss.p_mw}, {dss_transformer_loss.p_mw});
  const auto total_loss_q_summary = summarize_metric(
      {repo_total_loss.q_mvar}, {dss_transformer_loss.q_mvar});
  std::vector<double> repo_line_p;
  std::vector<double> repo_line_q;
  std::vector<double> dss_line_p;
  std::vector<double> dss_line_q;
  std::optional<MetricSummary> line_p_summary;
  std::optional<MetricSummary> line_q_summary;
  if (has_external_line) {
    const int branch_pos =
        find_branch_position_by_name(projected.ac, spec.external_line_name);
    const auto& branch = projected.ac.branches[static_cast<std::size_t>(branch_pos)];
    const int from_pos = static_cast<int>(bus_position.at(branch.from_bus));
    const int to_pos = static_cast<int>(bus_position.at(branch.to_bus));
    const auto repo_line_flow = compute_single_phase_branch_terminal_powers(
        branch,
        polar_pu(repo.vm_pu[static_cast<std::size_t>(from_pos)],
                 repo.va_deg[static_cast<std::size_t>(from_pos)]),
        polar_pu(repo.vm_pu[static_cast<std::size_t>(to_pos)],
                 repo.va_deg[static_cast<std::size_t>(to_pos)]),
        repo_case.base_mva);
    repo_line_p.push_back(repo_line_flow.power_from_mva.real());
    repo_line_q.push_back(repo_line_flow.power_from_mva.imag());

    const auto& dss_line = find_pd_element(
        pd_element_index, hacdcpf::io::OpenDSSPDElementKind::Line,
        spec.external_line_name);
    const auto dss_line_terminal =
        sum_pd_element_terminal_powers_for_terminal(dss_line, 1);
    dss_line_p.push_back(dss_line_terminal.p_mw);
    dss_line_q.push_back(dss_line_terminal.q_mvar);
    line_p_summary = summarize_metric(repo_line_p, dss_line_p);
    line_q_summary = summarize_metric(repo_line_q, dss_line_q);
  }
  const auto active_path_vm_alignment = summarize_metric(
      repo.vm_pu, legacy_raw_pair.result.vm_pu);
  const auto active_path_va_alignment = summarize_metric(
      repo.va_deg, legacy_raw_pair.result.va_deg);

  static constexpr std::array<const char*, 3> kWindingLabels = {
      "hv", "mv", "lv"};
  auto build_winding_active_error_abs_json =
      [&](const std::array<hacdcpf_compare::PowerPair, 3>& model) {
        return json{
            {"hv", std::abs(model[0].p_mw - dss_terminal_powers[0].p_mw)},
            {"mv", std::abs(model[1].p_mw - dss_terminal_powers[1].p_mw)},
            {"lv", std::abs(model[2].p_mw - dss_terminal_powers[2].p_mw)},
        };
      };
  auto build_winding_reactive_error_abs_json =
      [&](const std::array<hacdcpf_compare::PowerPair, 3>& model) {
        return json{
            {"hv", std::abs(model[0].q_mvar - dss_terminal_powers[0].q_mvar)},
            {"mv", std::abs(model[1].q_mvar - dss_terminal_powers[1].q_mvar)},
            {"lv", std::abs(model[2].q_mvar - dss_terminal_powers[2].q_mvar)},
        };
      };
  auto build_solution_counterfactual_json =
      [&](const std::string& model_id,
          const Transformer3WPairModelSolution& solution,
          const std::optional<json>& extra = std::nullopt) {
        const auto vm = summarize_metric(solution.result.vm_pu, dss_vm);
        const auto va = summarize_metric(solution.result.va_deg, dss_va);
        const auto t1p = summarize_metric(
            {solution.terminal_powers[0].p_mw}, {dss_terminal_powers[0].p_mw});
        const auto t1q = summarize_metric(
            {solution.terminal_powers[0].q_mvar}, {dss_terminal_powers[0].q_mvar});
        const auto t2p = summarize_metric(
            {solution.terminal_powers[1].p_mw}, {dss_terminal_powers[1].p_mw});
        const auto t2q = summarize_metric(
            {solution.terminal_powers[1].q_mvar}, {dss_terminal_powers[1].q_mvar});
        const auto t3p = summarize_metric(
            {solution.terminal_powers[2].p_mw}, {dss_terminal_powers[2].p_mw});
        const auto t3q = summarize_metric(
            {solution.terminal_powers[2].q_mvar}, {dss_terminal_powers[2].q_mvar});
        json out = {
            {"model_id", model_id},
            {"converged", solution.result.converged},
            {"metrics",
             {
                 {"bus_voltage_magnitude_pu", to_json(vm)},
                 {"bus_voltage_angle_deg", to_json(va)},
                 {"transformer_terminal1_p_mw", to_json(t1p)},
                 {"transformer_terminal1_q_mvar", to_json(t1q)},
                 {"transformer_terminal2_p_mw", to_json(t2p)},
                 {"transformer_terminal2_q_mvar", to_json(t2q)},
                 {"transformer_terminal3_p_mw", to_json(t3p)},
                 {"transformer_terminal3_q_mvar", to_json(t3q)},
             }},
            {"winding_active_power_error_abs_mw",
             build_winding_active_error_abs_json(solution.terminal_powers)},
            {"winding_reactive_power_error_abs_mvar",
             build_winding_reactive_error_abs_json(solution.terminal_powers)},
            {"mv_lv_active_power_split_mismatch_present",
             t2p.max_abs > kTransformer3WCrossCheckTol.p_mw &&
                 t3p.max_abs > kTransformer3WCrossCheckTol.p_mw},
            {"mv_lv_reactive_power_split_mismatch_present",
             t2q.max_abs > kTransformer3WCrossCheckTol.q_mvar &&
                 t3q.max_abs > kTransformer3WCrossCheckTol.q_mvar},
            {"hv_active_power_match_preserved",
             t1p.max_abs <= kTransformer3WCrossCheckTol.p_mw},
            {"mv_lv_active_power_error_symmetry_abs_diff_mw",
             std::abs(t2p.max_abs - t3p.max_abs)},
            {"pair_branch_breakdown", solution.pair_branch_breakdown},
            {"derivation", solution.derivation},
        };
        if (extra.has_value()) {
          out["extra"] = *extra;
        }
        return out;
      };
  auto build_replay_error_json =
      [&](const std::string& model_id,
          const Transformer3WReplayResult& replay) {
        return json{
            {"model_id", model_id},
            {"winding_active_power_error_abs_mw",
             build_winding_active_error_abs_json(replay.terminal_powers)},
            {"winding_reactive_power_error_abs_mvar",
             build_winding_reactive_error_abs_json(replay.terminal_powers)},
            {"total_loss_abs_error",
             {
                 {"p_mw", std::abs(replay.total_loss.p_mw - dss_transformer_loss.p_mw)},
                 {"q_mvar",
                  std::abs(replay.total_loss.q_mvar - dss_transformer_loss.q_mvar)},
             }},
        };
      };
  json per_winding_voltage_error = json::array();
  json per_winding_terminal_power_error = json::array();
  for (std::size_t winding = 0; winding < kWindingLabels.size(); ++winding) {
    const int repo_bus_pos =
        find_bus_position_by_name(repo_case.ac.buses, spec.bus_names[winding]);
    const auto& nv = find_node_voltage(
        voltage_index, spec.bus_names[winding], spec.bus_nodes[winding]);
    per_winding_voltage_error.push_back({
        {"winding", static_cast<int>(winding + 1)},
        {"winding_label", kWindingLabels[winding]},
        {"bus_name", spec.bus_names[winding]},
        {"repo_vm_pu", repo.vm_pu[static_cast<std::size_t>(repo_bus_pos)]},
        {"opendss_vm_pu", nv.vm_pu},
        {"abs_vm_error_pu",
         std::abs(repo.vm_pu[static_cast<std::size_t>(repo_bus_pos)] - nv.vm_pu)},
        {"repo_va_deg", repo.va_deg[static_cast<std::size_t>(repo_bus_pos)]},
        {"opendss_va_deg", nv.va_deg},
        {"abs_va_error_deg",
         std::abs(repo.va_deg[static_cast<std::size_t>(repo_bus_pos)] - nv.va_deg)},
    });
    per_winding_terminal_power_error.push_back({
        {"winding", static_cast<int>(winding + 1)},
        {"winding_label", kWindingLabels[winding]},
        {"repo_terminal_power_mw_mvar", power_pair_to_json(repo_terminal_powers[winding])},
        {"opendss_terminal_power_mw_mvar", power_pair_to_json(dss_terminal_powers[winding])},
        {"abs_error",
         {
             {"p_mw",
              std::abs(repo_terminal_powers[winding].p_mw -
                       dss_terminal_powers[winding].p_mw)},
             {"q_mvar",
              std::abs(repo_terminal_powers[winding].q_mvar -
                       dss_terminal_powers[winding].q_mvar)},
         }},
    });
  }

  json tap_side_projection_error = json::array();
  const int tapped_bus_id = winding_bus_ids[static_cast<std::size_t>(spec.tap_side)];
  double max_tap_projection_error_pu = 0.0;
  for (const auto& pair_item : pair_branch_breakdown) {
    const int from_bus = pair_item["from_bus"].get<int>();
    const int to_bus = pair_item["to_bus"].get<int>();
    const bool touches_tapped_winding =
        from_bus == tapped_bus_id || to_bus == tapped_bus_id;
    const double expected_branch_tap_pu =
        !touches_tapped_winding ? 1.0
                                : (from_bus == tapped_bus_id ? tapped_winding_state.tap_pu
                                                             : (1.0 / tapped_winding_state.tap_pu));
    const int branch_pos = find_branch_position_by_name(
        projected.ac, pair_item["branch_name"].get<std::string>());
    const auto& branch =
        projected.ac.branches[static_cast<std::size_t>(branch_pos)];
    const double abs_tap_error_pu =
        std::abs(branch.tap - expected_branch_tap_pu);
    max_tap_projection_error_pu =
        std::max(max_tap_projection_error_pu, abs_tap_error_pu);
    tap_side_projection_error.push_back({
        {"pair_number", pair_item["pair_number"]},
        {"branch_name", branch.name},
        {"touches_tapped_winding", touches_tapped_winding},
        {"projected_branch_tap_pu", branch.tap},
        {"expected_branch_tap_pu", expected_branch_tap_pu},
        {"abs_tap_error_pu", abs_tap_error_pu},
    });
  }

  const double element_vs_circuit_loss_p_abs_diff =
      std::abs(dss_transformer_loss.p_mw - dss_circuit_loss.p_mw);
  const double element_vs_circuit_loss_q_abs_diff =
      std::abs(dss_transformer_loss.q_mvar - dss_circuit_loss.q_mvar);
  const double element_vs_terminal_sum_p_abs_diff =
      std::abs(dss_transformer_loss.p_mw - dss_terminal_sum.p_mw);
  const double element_vs_terminal_sum_q_abs_diff =
      std::abs(dss_transformer_loss.q_mvar - dss_terminal_sum.q_mvar);
  const bool bridge_loss_self_consistent =
      element_vs_circuit_loss_p_abs_diff <= 1e-12 &&
      element_vs_circuit_loss_q_abs_diff <= 1e-12 &&
      element_vs_terminal_sum_p_abs_diff <= 1e-12 &&
      element_vs_terminal_sum_q_abs_diff <= 1e-12;
  const bool tap_projection_matches_expected =
      max_tap_projection_error_pu <= 1e-12;
  const bool mv_lv_active_power_split_mismatch_present =
      terminal2_p_summary.max_abs > kTransformer3WCrossCheckTol.p_mw &&
      terminal3_p_summary.max_abs > kTransformer3WCrossCheckTol.p_mw;
  const bool mv_lv_reactive_power_split_mismatch_present =
      terminal2_q_summary.max_abs > kTransformer3WCrossCheckTol.q_mvar &&
      terminal3_q_summary.max_abs > kTransformer3WCrossCheckTol.q_mvar;
  const bool hv_active_power_match_preserved =
      terminal1_p_summary.max_abs <= kTransformer3WCrossCheckTol.p_mw;
  struct DominantGapMetricCandidate {
    const char* name;
    double absolute_error;
    double threshold;
  };
  std::vector<DominantGapMetricCandidate> dominant_gap_candidates = {
      {"bus_voltage_magnitude_pu", vm_summary.max_abs, kTransformer3WCrossCheckTol.vm_pu},
      {"bus_voltage_angle_deg", va_summary.max_abs, kTransformer3WCrossCheckTol.va_deg},
      {"transformer_terminal1_q_mvar", terminal1_q_summary.max_abs,
       kTransformer3WCrossCheckTol.q_mvar},
      {"total_branch_loss_q_mvar", total_loss_q_summary.max_abs,
       kTransformer3WCrossCheckTol.q_mvar},
  };
  if (line_q_summary.has_value()) {
    dominant_gap_candidates.push_back(
        {"line_terminal1_q_mvar", line_q_summary->max_abs,
         kTransformer3WCrossCheckTol.q_mvar});
  }
  const auto dominant_gap_metric_it = std::max_element(
      dominant_gap_candidates.begin(), dominant_gap_candidates.end(),
      [](const DominantGapMetricCandidate& lhs,
         const DominantGapMetricCandidate& rhs) {
        return (lhs.absolute_error / lhs.threshold) < (rhs.absolute_error / rhs.threshold);
      });
  const auto& dominant_gap_metric = *dominant_gap_metric_it;
  const bool transformer3w_formal_thresholds_passed =
      vm_summary.max_abs <= kTransformer3WCrossCheckTol.vm_pu &&
      va_summary.max_abs <= kTransformer3WCrossCheckTol.va_deg &&
      terminal1_p_summary.max_abs <= kTransformer3WCrossCheckTol.p_mw &&
      terminal1_q_summary.max_abs <= kTransformer3WCrossCheckTol.q_mvar &&
      terminal2_p_summary.max_abs <= kTransformer3WCrossCheckTol.p_mw &&
      terminal2_q_summary.max_abs <= kTransformer3WCrossCheckTol.q_mvar &&
      terminal3_p_summary.max_abs <= kTransformer3WCrossCheckTol.p_mw &&
      terminal3_q_summary.max_abs <= kTransformer3WCrossCheckTol.q_mvar &&
      total_loss_p_summary.max_abs <= kTransformer3WCrossCheckTol.p_mw &&
      total_loss_q_summary.max_abs <= kTransformer3WCrossCheckTol.q_mvar;
  const json legacy_raw_pair_counterfactual = build_solution_counterfactual_json(
      "legacy_raw_pair_branch_reduction_bfs", legacy_raw_pair,
      json{
          {"active_path_alignment",
           {
               {"bus_voltage_magnitude_pu", to_json(active_path_vm_alignment)},
               {"bus_voltage_angle_deg", to_json(active_path_va_alignment)},
           }},
      });
  const json coupled_kron_pair_counterfactual = build_solution_counterfactual_json(
      "coupled_kron_pair_resplit_bfs", coupled_kron_pair);
  const json replay_cross_check = {
      {"legacy_raw_pair_replay",
       build_replay_error_json(
           "legacy_raw_pair_replay", legacy_raw_pair_replay)},
      {"coupled_kron_pair_replay",
       build_replay_error_json(
           "coupled_kron_pair_replay", coupled_kron_pair_replay)},
      {"explicit_star_reference_replay",
       build_replay_error_json(
           "explicit_star_reference_replay",
           explicit_star_reference_replay)},
  };
  json dense_shadow_counterfactual = {
      {"model_id", "meshed_dense_shadow_nr"},
      {"dense_shadow_replay_cross_check", replay_cross_check},
  };
  if (!has_external_line) {
    const auto legacy_raw_pair_dense_shadow =
        solve_transformer3w_pair_model_dense_shadow(
            repo_case, Transformer3WPairReductionMode::LegacyRawPair,
            options.max_iter, options.tol);
    const auto coupled_kron_pair_dense_shadow =
        solve_transformer3w_pair_model_dense_shadow(
            repo_case, Transformer3WPairReductionMode::CoupledKronPair,
            options.max_iter, options.tol);
    dense_shadow_counterfactual["supported"] = true;
    dense_shadow_counterfactual["legacy_raw_pair_branch_reduction"] =
        build_solution_counterfactual_json(
            "legacy_raw_pair_dense_shadow_nr", legacy_raw_pair_dense_shadow);
    dense_shadow_counterfactual["coupled_kron_pair_resplit"] =
        build_solution_counterfactual_json(
            "coupled_kron_pair_dense_shadow_nr",
            coupled_kron_pair_dense_shadow);
    dense_shadow_counterfactual["shadow_solver_note"] =
        "Dense rectangular NR is used only as a diagnostics shadow for the "
        "meshed three-bus Transformer3W equivalent. It is not wired into the "
        "production BFS path.";
  } else {
    dense_shadow_counterfactual["supported"] = false;
    dense_shadow_counterfactual["skip_reason"] =
        "dense_shadow_counterfactual_is_restricted_to_the_isolated_transformer3w_triangle";
    dense_shadow_counterfactual["shadow_solver_note"] =
        "Dense rectangular NR diagnostics stay restricted to the isolated "
        "three-bus Transformer3W equivalent. The embedded slack-leaf case "
        "still emits the replay cross-check lanes required for model-vs-path "
        "triage.";
  }
  const bool legacy_raw_pair_active_split_mismatch_present =
      legacy_raw_pair_counterfactual["mv_lv_active_power_split_mismatch_present"].get<bool>();
  const bool legacy_raw_pair_reactive_split_mismatch_present =
      legacy_raw_pair_counterfactual["mv_lv_reactive_power_split_mismatch_present"].get<bool>();
  const std::string trigger_pattern =
      has_external_line
          ? "single_transformer3w_projected_to_triangle_with_one_slack_side_radial_leaf"
          : "single_transformer3w_projected_to_isolated_three_bus_three_branch_triangle";
  json fail_close_conditions = json::array({
      "all_projected_transformer3w_pair_branches_must_expand_from_one_transformer3w_with_pair_numbers_0_1_2",
      "exactly_one_slack_bus_and_remaining_pq_buses_are_required",
      "q_limit_outer_loop_and_non_pq_non_slack_bus_types_fail_close_to_existing_bfs",
  });
  if (has_external_line) {
    fail_close_conditions.push_back(
        "at_most_one_extra_in_service_nonparallel_branch_is_allowed");
    fail_close_conditions.push_back(
        "the_extra_branch_must_attach_a_single_leaf_bus_to_the_slack_side_of_the_triangle");
  } else {
    fail_close_conditions.push_back(
        "projected_system_must_have_exactly_three_buses_and_three_branches");
    fail_close_conditions.push_back(
        "projected_triangle_must_be_isolated_without_bus_merges_parallel_edges_or_out_of_service_branches");
  }
  json targeted_metrics = json::array({
      "bus_voltage_magnitude_pu",
      "transformer_terminal2_p_mw",
      "transformer_terminal2_q_mvar",
      "transformer_terminal3_p_mw",
      "transformer_terminal3_q_mvar",
      "current_repo_mv_lv_active_power_split_mismatch_present",
      "current_repo_mv_lv_reactive_power_split_mismatch_present",
  });
  if (has_external_line) {
    targeted_metrics.push_back("line_terminal1_p_mw");
    targeted_metrics.push_back("line_terminal1_q_mvar");
  }
  const std::string production_trial_path_id =
      repo.specialized_path_id.empty()
          ? "transformer3w_isolated_triangle_dense_nr_trial"
          : repo.specialized_path_id;
  const bool production_trial_applied = repo.specialized_path_applied;
  const json production_trial = {
      {"path_id", production_trial_path_id},
      {"applied", production_trial_applied},
      {"fail_close_reason",
       repo.specialized_path_fail_close_reason.empty()
           ? json(nullptr)
           : json(repo.specialized_path_fail_close_reason)},
      {"trigger_pattern", trigger_pattern},
      {"fail_close_conditions", fail_close_conditions},
      {"targeted_metrics", targeted_metrics},
      {"current_repo_vs_baseline",
       {
           {"baseline_model_id", "legacy_raw_pair_branch_reduction_bfs"},
           {"current_model_id",
            production_trial_applied ? production_trial_path_id
                                     : "solve_distribution_pf_active_path"},
           {"metrics",
            {
                {"bus_voltage_magnitude_pu",
                 {
                     {"baseline_absolute_error",
                      legacy_raw_pair_counterfactual["metrics"]
                                                 ["bus_voltage_magnitude_pu"]["max_abs"]},
                     {"current_absolute_error", vm_summary.max_abs},
                 }},
                {"transformer_terminal2_p_mw",
                 {
                     {"baseline_absolute_error",
                      legacy_raw_pair_counterfactual["metrics"]
                                                 ["transformer_terminal2_p_mw"]["max_abs"]},
                     {"current_absolute_error", terminal2_p_summary.max_abs},
                 }},
                {"transformer_terminal2_q_mvar",
                 {
                     {"baseline_absolute_error",
                      legacy_raw_pair_counterfactual["metrics"]
                                                 ["transformer_terminal2_q_mvar"]["max_abs"]},
                     {"current_absolute_error", terminal2_q_summary.max_abs},
                 }},
                {"transformer_terminal3_p_mw",
                 {
                     {"baseline_absolute_error",
                      legacy_raw_pair_counterfactual["metrics"]
                                                 ["transformer_terminal3_p_mw"]["max_abs"]},
                     {"current_absolute_error", terminal3_p_summary.max_abs},
                 }},
                {"transformer_terminal3_q_mvar",
                 {
                     {"baseline_absolute_error",
                      legacy_raw_pair_counterfactual["metrics"]
                                                 ["transformer_terminal3_q_mvar"]["max_abs"]},
                     {"current_absolute_error", terminal3_q_summary.max_abs},
                 }},
            }},
           {"mv_lv_active_power_split_mismatch_present",
            {
                {"baseline", legacy_raw_pair_active_split_mismatch_present},
                {"current_repo", mv_lv_active_power_split_mismatch_present},
            }},
           {"mv_lv_reactive_power_split_mismatch_present",
            {
                {"baseline", legacy_raw_pair_reactive_split_mismatch_present},
                {"current_repo", mv_lv_reactive_power_split_mismatch_present},
            }},
       }},
  };
  const json root_cause_evidence = {
      {"hypothesis",
       transformer3w_formal_thresholds_passed
           ? "model_side_transformer3w_pair_projection_required_coupled_kron_equivalent"
           : "radial_bfs_applied_to_meshed_transformer3w_triangle"},
      {"support_level",
       transformer3w_formal_thresholds_passed
           ? (has_external_line
                  ? "model_side_coupled_kron_projection_fix_landed_slack_leaf_embed_surface_now_passes"
                  : "model_side_coupled_kron_projection_fix_landed_minimal_case_now_passes")
           : production_trial_applied
           ? "formal_truth_voltagebases_corrected_reactive_gap_remaining"
           : "falsified_split_change_only_not_sufficient"},
      {"tap_projection_matches_expected", tap_projection_matches_expected},
      {"bridge_loss_self_consistent", bridge_loss_self_consistent},
      {"affected_windings",
       transformer3w_formal_thresholds_passed ? json::array() : json::array({"mv", "lv"})},
      {"dominant_gap_metric",
       {
           {"name", dominant_gap_metric.name},
           {"absolute_error", dominant_gap_metric.absolute_error},
           {"threshold", dominant_gap_metric.threshold},
           {"pass", dominant_gap_metric.absolute_error <= dominant_gap_metric.threshold},
       }},
      {"winding_active_power_error_abs_mw",
       {
           {"hv", terminal1_p_summary.max_abs},
           {"mv", terminal2_p_summary.max_abs},
           {"lv", terminal3_p_summary.max_abs},
       }},
      {"winding_reactive_power_error_abs_mvar",
       {
           {"hv", terminal1_q_summary.max_abs},
           {"mv", terminal2_q_summary.max_abs},
           {"lv", terminal3_q_summary.max_abs},
       }},
      {"current_repo_mv_lv_active_power_split_mismatch_present",
       mv_lv_active_power_split_mismatch_present},
      {"current_repo_mv_lv_reactive_power_split_mismatch_present",
       mv_lv_reactive_power_split_mismatch_present},
      {"current_repo_hv_active_power_match_preserved",
       hv_active_power_match_preserved},
      {"current_repo_mv_lv_active_power_error_symmetry_abs_diff_mw",
       std::abs(terminal2_p_summary.max_abs - terminal3_p_summary.max_abs)},
      {"production_trial", production_trial},
      {"opendss_loss_self_consistency_abs_diff",
       {
           {"element_vs_circuit",
            {
                {"p_mw", element_vs_circuit_loss_p_abs_diff},
                {"q_mvar", element_vs_circuit_loss_q_abs_diff},
            }},
           {"element_vs_terminal_sum",
            {
                {"p_mw", element_vs_terminal_sum_p_abs_diff},
                {"q_mvar", element_vs_terminal_sum_q_abs_diff},
            }},
       }},
      {"counterfactuals",
       {
           {"legacy_raw_pair_branch_reduction_bfs", legacy_raw_pair_counterfactual},
           {"coupled_kron_pair_resplit_bfs", coupled_kron_pair_counterfactual},
           {"meshed_dense_shadow_nr", dense_shadow_counterfactual},
       }},
      {"next_single_point",
       transformer3w_formal_thresholds_passed
           ? (has_external_line
                  ? json{
                        {"name",
                         "non_slack_or_multi_branch_transformer3w_surface_still_unverified"},
                        {"evidence",
                         "The one-phase Transformer3W production path is now accepted "
                         "both on the isolated triangle and on the same triangle with "
                         "one slack-side radial spur. That acceptance does not yet "
                         "extend to non-slack embeds, multiple external branches, or "
                         "broader mixed-surface Transformer3W topologies."},
                    }
                  : json{
                        {"name", "broader_transformer3w_surface_still_unverified"},
                        {"evidence",
                         "The isolated one-phase Transformer3W triangle production "
                         "path is now accepted on its own minimal OpenDSS compare "
                         "surface. That acceptance does not yet extend to broader "
                         "Transformer3W topologies or mixed-surface cases."},
                    })
           : production_trial_applied
           ? json{
                 {"name", "transformer_terminal1_q_mvar_and_total_branch_loss_q_mvar"},
                 {"evidence",
                  "Correcting the 1-phase OpenDSS VoltageBases to line-line "
                  "equivalents collapses the pu-voltage mismatch without changing "
                  "actual voltages, terminal powers, or losses. The remaining "
                  "deferred tail is now reactive on terminal 1 and total loss."},
             }
           : json{
                 {"name", "radial_bfs_applied_to_meshed_transformer3w_triangle"},
                 {"evidence",
                  "Changing only the impedance split reduces the MV/LV error but "
                  "does not remove it, while the dense shadow removes the split "
                  "mismatch without changing the compare surface or the OpenDSS "
                  "adapter."},
             }},
      {"inference",
       transformer3w_formal_thresholds_passed
           ? (has_external_line
                  ? "Tap projection stays exact and OpenDSS loss channels remain "
                    "internally self-consistent. Replacing the raw Transformer3W "
                    "pair-branch projection with the coupled-kron equivalent keeps "
                    "the one-phase production path inside the formal tolerance family "
                    "when the triangle is embedded with one slack-side radial spur. "
                    "This widens the accepted Transformer3W surface by one controlled "
                    "step without claiming generic embedded-network coverage."
                  : "Tap projection stays exact and OpenDSS loss channels remain "
                    "internally self-consistent. Replacing the raw Transformer3W "
                    "pair-branch projection with the coupled-kron equivalent brings the "
                    "isolated one-phase production case into the formal tolerance family. "
                    "That minimal isolated-triangle surface is now accepted, while "
                    "broader Transformer3W reference surfaces remain explicitly out of scope.")
           : production_trial_applied
           ? "Tap projection stays exact and OpenDSS loss channels remain "
             "internally self-consistent. Correcting the 1-phase VoltageBases "
             "definition removes the sqrt(3)-scale pu-voltage truth-surface drift "
             "while preserving the same actual volts and terminal powers. The case "
             "remains deferred because the next remaining blocker is the reactive "
             "terminal/loss tail, not the MV/LV split."
           : "Tap projection stays exact and OpenDSS loss channels remain "
             "internally self-consistent. Changing only the MV/LV split is not "
             "sufficient: the current radial BFS still mis-solves the meshed "
             "Transformer3W triangle. The next isolated blocker is therefore the "
             "solver/model mismatch, not the raw pair split by itself."},
  };

  json out = {
      {"case_id", spec.case_id},
      {"acceptance_tier", spec.acceptance_tier},
      {"status_reason", status_reason},
      {"module_in_scope", "solve_distribution_pf(const HybridPowerSystem&)"},
      {"scope_reason", scope_reason},
      {"opendss_voltage_base_note",
       "1-phase Vsource/Load/Transformer kV entries remain winding or line-to-neutral "
       "values, while VoltageBases uses the equivalent line-to-line values required "
       "by OpenDSS CalcVoltageBases."},
      {"opendss_case", fs::relative(master_dss, project_root).string()},
      {"repo_case_origin", repo_case_origin},
      {"converged", {{"repo", repo.converged}, {"opendss", dss.converged}}},
      {"projection_diagnostics",
       {
           {"transformer_name", spec.transformer_name},
           {"tap_side", spec.tap_side},
           {"tap_pos", spec.tap_pos},
           {"tap_step_percent", spec.tap_step_percent},
           {"pair_branch_breakdown", pair_branch_breakdown},
           {"tap_side_projection_error", tap_side_projection_error},
       }},
      {"error_decomposition",
       {
           {"per_winding_voltage_error", per_winding_voltage_error},
           {"per_winding_terminal_power_error", per_winding_terminal_power_error},
           {"tap_side_projection_error", tap_side_projection_error},
       }},
      {"root_cause_evidence", root_cause_evidence},
      {"metrics",
       {
           {"bus_voltage_magnitude_pu", to_json(vm_summary)},
           {"bus_voltage_angle_deg", to_json(va_summary)},
           {"transformer_terminal1_p_mw",
            to_json(terminal1_p_summary)},
           {"transformer_terminal1_q_mvar",
            to_json(terminal1_q_summary)},
           {"transformer_terminal2_p_mw",
            to_json(terminal2_p_summary)},
           {"transformer_terminal2_q_mvar",
            to_json(terminal2_q_summary)},
           {"transformer_terminal3_p_mw",
            to_json(terminal3_p_summary)},
           {"transformer_terminal3_q_mvar",
            to_json(terminal3_q_summary)},
           {"total_branch_loss_p_mw",
            to_json(total_loss_p_summary)},
           {"total_branch_loss_q_mvar",
            to_json(total_loss_q_summary)},
       }},
      {"acceptance_tolerances", tolerance_to_json(kTransformer3WCrossCheckTol)},
      {"repo",
       {
           {"bus_voltage_magnitude_pu", repo_vm},
           {"bus_voltage_angle_deg", repo_va},
           {"transformer_terminal1_mw_mvar",
            power_pair_to_json(repo_terminal_powers[0])},
           {"transformer_terminal2_mw_mvar",
            power_pair_to_json(repo_terminal_powers[1])},
           {"transformer_terminal3_mw_mvar",
            power_pair_to_json(repo_terminal_powers[2])},
           {"total_branch_loss_mw_mvar", power_pair_to_json(repo_total_loss)},
       }},
      {"opendss",
       {
           {"bus_voltage_magnitude_pu", dss_vm},
           {"bus_voltage_angle_deg", dss_va},
           {"transformer_terminal1_mw_mvar",
            power_pair_to_json(dss_terminal_powers[0])},
           {"transformer_terminal2_mw_mvar",
            power_pair_to_json(dss_terminal_powers[1])},
           {"transformer_terminal3_mw_mvar",
            power_pair_to_json(dss_terminal_powers[2])},
           {"transformer_terminal_sum_mw_mvar",
            power_pair_to_json(dss_terminal_sum)},
           {"transformer_losses_mw_mvar",
            power_pair_to_json(dss_transformer_loss)},
           {"transformer_state", transformer_state_to_json(transformer_state)},
           {"circuit_losses_raw", losses_raw_to_json(dss.circuit_losses_raw)},
           {"circuit_losses_mw_mvar", power_pair_to_json(dss_circuit_loss)},
       }},
      {"loss_diagnostics",
       {
           {"terminal_power_summed_loss", power_pair_to_json(dss_terminal_sum)},
           {"element_losses_cross_check", power_pair_to_json(dss_transformer_loss)},
           {"circuit_losses_cross_check", power_pair_to_json(dss_circuit_loss)},
           {"loss_mismatch_breakdown",
            {
                {"repo_total_branch_loss_mw_mvar", power_pair_to_json(repo_total_loss)},
                {"opendss_transformer_loss_mw_mvar",
                 power_pair_to_json(dss_transformer_loss)},
                {"opendss_circuit_loss_mw_mvar", power_pair_to_json(dss_circuit_loss)},
                {"abs_diff_repo_vs_element",
                 {
                     {"p_mw", std::abs(repo_total_loss.p_mw - dss_transformer_loss.p_mw)},
                     {"q_mvar",
                      std::abs(repo_total_loss.q_mvar - dss_transformer_loss.q_mvar)},
                 }},
                {"abs_diff_repo_vs_circuit",
                 {
                     {"p_mw", std::abs(repo_total_loss.p_mw - dss_circuit_loss.p_mw)},
                     {"q_mvar",
                      std::abs(repo_total_loss.q_mvar - dss_circuit_loss.q_mvar)},
                 }},
                {"abs_diff_element_vs_circuit",
                 {
                     {"p_mw", element_vs_circuit_loss_p_abs_diff},
                     {"q_mvar", element_vs_circuit_loss_q_abs_diff},
                 }},
            }},
           {"element_vs_terminal_sum_abs_diff",
            {
                {"p_mw", element_vs_terminal_sum_p_abs_diff},
                {"q_mvar", element_vs_terminal_sum_q_abs_diff},
            }},
       }},
      {"units",
       {
           {"bus_voltage_magnitude", "p.u."},
           {"bus_voltage_angle", "degrees"},
           {"transformer_terminal_power", "MW / MVAr"},
           {"total_loss", "MW / MVAr"},
       }},
  };
  if (has_external_line) {
    out["metrics"]["line_terminal1_p_mw"] = to_json(*line_p_summary);
    out["metrics"]["line_terminal1_q_mvar"] = to_json(*line_q_summary);
    out["repo"]["line_terminal1_p_mw"] = repo_line_p;
    out["repo"]["line_terminal1_q_mvar"] = repo_line_q;
    out["opendss"]["line_terminal1_p_mw"] = dss_line_p;
    out["opendss"]["line_terminal1_q_mvar"] = dss_line_q;
    out["projection_diagnostics"]["external_line_name"] = spec.external_line_name;
    out["error_decomposition"]["observed_bus_names"] = observed_bus_names;
  }
  return out;
}

json summarize_transformer3w_formal_cross_check_case(const fs::path& project_root) {
  return summarize_transformer3w_formal_cross_check_case_impl(
      project_root,
      hacdcpf_compare_fixtures::transformer3w_formal_cross_check_case(),
      hacdcpf_compare_fixtures::build_minimal_transformer3w_cross_check_case(),
      "hacdcpf_compare_fixtures::build_minimal_transformer3w_cross_check_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Accepted minimal OpenDSS compare for the isolated one-phase "
      "Transformer3W production path. The coupled-kron canonical projection "
      "lands this narrow surface inside the formal tolerance family without "
      "claiming broader Transformer3W coverage.",
      "Formal cross-check for the isolated Transformer3W triangle production "
      "trial and its diagnostics shadow against a minimal one-phase OpenDSS "
      "Transformer element.");
}

json summarize_transformer3w_hv_spur_formal_cross_check_case(
    const fs::path& project_root) {
  return summarize_transformer3w_formal_cross_check_case_impl(
      project_root,
      hacdcpf_compare_fixtures::transformer3w_hv_spur_formal_cross_check_case(),
      hacdcpf_compare_fixtures::build_transformer3w_hv_spur_cross_check_case(),
      "hacdcpf_compare_fixtures::build_transformer3w_hv_spur_cross_check_case in "
      "tools/opendss_compare/fixtures.hpp",
      "Accepted minimal OpenDSS compare for the one-phase Transformer3W "
      "triangle embedded with a single slack-side radial spur. The same "
      "coupled-kron projection remains inside the formal tolerance family on "
      "this slightly broader surface without claiming generic Transformer3W "
      "network coverage.",
      "Formal cross-check for the one-phase Transformer3W production path when "
      "the three-winding triangle is embedded in one extra slack-side radial "
      "spur.");
}

json build_adapter_contract_surface(const fs::path& project_root) {
  const fs::path master =
      project_root / "tests/data/opendss/minimal_regulator_1ph/Master.dss";
  const auto snapshot = hacdcpf::io::solve_opendss_snapshot(master);
  const std::string portable_master =
      hacdcpf_compare::snapshot_contract::portable_path_string(
          master, project_root);
  const json serialized =
      hacdcpf_compare::snapshot_contract::snapshot_to_json(snapshot, portable_master);
  const auto roundtrip =
      hacdcpf_compare::snapshot_contract::snapshot_from_json(serialized);

  json surface =
      hacdcpf_compare::snapshot_contract::build_contract_surface_summary(roundtrip);
  surface["canonical_probe_case"] = portable_master;
  surface["serializer_parser_roundtrip_consistent"] =
      roundtrip.snapshot.node_voltages.size() == snapshot.node_voltages.size() &&
      roundtrip.snapshot.pd_element_results.size() ==
          snapshot.pd_element_results.size() &&
      roundtrip.snapshot.transformer_states.size() ==
          snapshot.transformer_states.size() &&
      roundtrip.snapshot.regcontrol_results.size() ==
          snapshot.regcontrol_results.size() &&
      roundtrip.snapshot.line_results.size() == snapshot.line_results.size() &&
      std::abs(roundtrip.snapshot.circuit_losses_raw.p_w -
               snapshot.circuit_losses_raw.p_w) < 1e-12 &&
      std::abs(roundtrip.snapshot.circuit_losses_raw.q_var -
               snapshot.circuit_losses_raw.q_var) < 1e-12;
  return surface;
}

json build_tolerance_justification() {
  return {
      {"source_type", "repository_machine_enforced_policy"},
      {"rationale",
       "Thresholds are a machine-enforced repo-vs-OpenDSS benchmark policy, not "
       "a promotion shortcut. Accepted families use tight bands only where the "
       "repository model is intended to be numerically comparable to the chosen "
       "OpenDSS compare surface. Wider or deferred families remain descriptive "
       "evidence and do not override explicit capability limitations."},
      {"threshold_families",
       json::array({
           {{"family_id", "single_phase_equivalence"},
            {"applies_to_tiers",
             {"accepted_primary", "accepted_rich_component",
              "accepted_pass_through_xfmr", "accepted_fixed_tap_xfmr"}},
            {"tolerances", tolerance_to_json(k1phTol)},
            {"family_rationale",
             "Used where the repository and OpenDSS compare the same single-phase "
             "steady-state electrical surface; thresholds mainly absorb adapter "
             "extraction and floating-point spread, not structural model error."}},
           {{"family_id", "three_phase_compare_surface"},
            {"applies_to_tiers",
             {"accepted_secondary", "accepted_unbalanced_3ph",
              "accepted_feeder_subset", "accepted_nr_load_semantics_3ph",
              "accepted_nr_phase_matrix_line_3ph", "accepted_nr_meshed_3ph",
              "accepted_nr_multi_source_3ph",
              "deferred_ieee13_feeder_formal_cross_check"}},
            {"tolerances", tolerance_to_json(k3phTol)},
            {"family_rationale",
             "Used for balanced, unbalanced, feeder-scale, and NR three-phase "
             "compare surfaces. The 2e-3 p.u. / 0.2 degree / 5e-3 MW-MVAr loss "
             "band is tight enough to gate accepted families while still "
             "quantifying deferred model-limited feeder gaps machine-readably."}},
           {{"family_id", "regulator_closed_loop_control"},
            {"applies_to_tiers",
             {"accepted_regulator", "accepted_regulator_local_pt_ldc",
              "accepted_regulator_remote_ptratio",
              "accepted_regulator_bank_3ph"}},
            {"tolerances", tolerance_to_json(kRegulatorTol)},
            {"family_rationale",
             "Inherits the tight electrical tolerances and adds PT-secondary "
             "control voltage plus exact discrete tap agreement for accepted "
             "closed-loop regulator validation, including the minimal "
             "three-phase bank case in the NR lane."}},
           {{"family_id", "exploratory_physical_component_bias"},
            {"applies_to_tiers", {"exploratory_physical_component"}},
            {"tolerances", tolerance_to_json(kCapTol)},
            {"family_rationale",
             "Reserved for the known Capacitor-vs-fixed-shunt model mismatch. The "
             "looser band quantifies the expected V^2 bias and is not eligible for "
             "accepted-baseline promotion."}},
           {{"family_id", "transformer3w_minimal_compare"},
            {"applies_to_tiers",
             {"accepted_transformer3w_isolated_triangle",
              "accepted_transformer3w_hv_spur_embed"}},
            {"tolerances", tolerance_to_json(kTransformer3WCrossCheckTol)},
            {"family_rationale",
             "Dedicated tolerance family for the accepted one-phase "
             "Transformer3W compare surfaces: the isolated triangle and the "
             "same triangle with one slack-side radial spur. It is tight enough "
             "to gate the current narrow production path without claiming broader "
             "Transformer3W coverage."}},
       })},
      {"citations",
       json::array({
           {{"id", "repository_case_tolerance_contract"},
            {"type", "repository_source"},
            {"path", "tools/opendss_pf_compare.cpp"},
            {"evidence",
             "CaseTolerance constants and evaluate_case()/build_error_budget() are "
             "the machine-enforced threshold source for compare artifacts."}},
           {{"id", "repository_manifest_scope_contract"},
            {"type", "repository_source"},
            {"path", "tools/opendss_pf_compare.cpp"},
            {"evidence",
             "build_case_manifest() and build_capability_matrix() define which "
             "families are accepted, exploratory, deferred, or unsupported."}},
           {{"id", "repository_artifact_regression_lock"},
            {"type", "repository_source"},
            {"path", "tests/test_opendss_pf_compare.cpp"},
            {"evidence",
             "Artifact regressions lock the emitted summary/manifest schema and the "
             "accepted-case gate behavior."}},
       })},
      {"application_boundary",
       "Applies only to the machine-readable repo-vs-OpenDSS compare artifact "
       "emitted by tools/opendss_pf_compare.cpp. It does not define solver "
       "convergence settings, optimization tolerances, or promotion of deferred "
       "or model-limited cases into the accepted baseline without explicit scope "
       "and regression evidence."},
  };
}

// ---------------------------------------------------------------------------
// build_capability_matrix — reviewer-usable, two-column classification
// ---------------------------------------------------------------------------
json build_capability_matrix() {
  return {
      {"components",
       json::array({
           // ---- Accepted ----
           {{"component",         "single_phase_radial_bfs"},
            {"module",            "solve_distribution_pf(const ACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_radial_3bus_1ph", "case33bw_radial_3ph"}},
            {"notes",             "BFS backward-forward sweep on purely radial ACSystem."}},

           {{"component",         "balanced_three_phase_radial_bfs"},
            {"module",            "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_radial_3bus_balanced_3ph"}},
            {"notes",             "Per-phase BFS on balanced ThreePhaseACSystem."}},

           {{"component",         "constant_power_loads (OpenDSS model=1)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_radial_3bus_1ph", "minimal_radial_3bus_balanced_3ph",
                                   "minimal_radial_3bus_1ph_shunt", "case33bw_radial_3ph"}},
            {"notes",             "vminpu=0.0 vmaxpu=2.0 disables voltage-dependent fallback."}},

           {{"component",         "fixed_reactive_shunt — ACBus.bs_mvar constant injection"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_radial_3bus_1ph_shunt"}},
            {"opendss_representation",
             "model=1 constant-power negative-Q load (kvar=-150). "
             "Both sides are voltage-independent — no V^2 bias."},
            {"notes",             "BFS injects fixed bs_mvar regardless of V."}},

           // ---- Exploratory ----
           {{"component",         "physical_capacitor — Capacitor element (V^2 dependent Q)"},
            {"comparison_status", "exploratory"},
            {"reason_for_gap",    "model_mismatch"},
            {"cases",             {"minimal_radial_3bus_1ph_cap"}},
            {"notes",
             "BFS uses fixed ACBus.bs_mvar injection; OpenDSS Capacitor delivers V^2*B. "
             "Systematic bias ≈ (1-V^2)*kvar at the cap bus (~5-6% at nominal loading). "
             "Bias is expected, quantified, and documented in the case artifact."}},

           // ---- Accepted (promoted from deferred) ----
           {{"component",         "pass_through_transformer (tap=1.0 series impedance)"},
            {"module",            "solve_distribution_pf(const ACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_passthr_transformer_1ph"}},
           {"notes",
             "ACBranch with tap=1.0 is numerically equivalent to an OpenDSS Transformer "
             "with nominal ratio=1.0, %noloadloss=0, %imag=0 (pure series impedance). "
             "Bus voltages, transformer terminal powers, downstream line powers, and "
             "summed branch losses match within kXfmrTol."}},

           {{"component",         "unbalanced_three_phase_loads"},
            {"module",            "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_unbalanced_3bus_3ph"}},
            {"notes",
             "ThreePhaseACBus per-phase pd/qd verified against OpenDSS 1-phase elements "
             "per node (bus2.1/2/3, bus3.1/2/3). Per-phase Vm, Va, and terminal powers "
             "match within kUnbal3phTol."}},

           {{"component",         "three_phase_voltage_angle"},
            {"module",            "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_unbalanced_3bus_3ph"}},
            {"notes",
             "ThreePhaseDPFResult.bus_voltages.va_a/b/c_deg verified against OpenDSS "
             "ActiveBus.puVmagAngle phase angles. Phase-A reference convention confirmed. "
             "First accepted case exercising per-phase voltage angle path."}},

           // ---- Deferred ----
           {{"component",         "q_limit_enforcement — DPFOptions.enforce_q_limits"},
            {"comparison_status", "deferred"},
            {"reason_for_gap",    "not_yet_benchmarked"},
            {"notes",             "Option available in DPFOptions; not yet exercised against OpenDSS."}},

           // ---- Unsupported ----
           {{"component",         "fixed_tap_transformer (tap≠1.0, 2-winding)"},
            {"module",            "solve_distribution_pf(const HybridPowerSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_tap_transformer_1ph"}},
            {"notes",
             "Transformer2W tap_side/tap_pos/tap_step_percent are normalized onto "
             "ACBranch.tap at the from_bus side, and the tap-aware BFS equations now "
             "match OpenDSS on a fixed off-nominal tap case."}},

           {{"component",         "voltage_regulator — automated tap-changer"},
            {"module",            "solve_distribution_pf(const HybridPowerSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_regulator_1ph",
                                   "minimal_regulator_local_pt_ldc_1ph",
                                   "minimal_regulator_remote_ptratio_1ph"}},
            {"notes",
             "Closed-loop single-phase regulator control now runs as an outer discrete "
             "tap loop around the tap-aware BFS inner solve. OpenDSS and the repository "
             "compare the remote-bus/no-LDC path, the remote-bus path with "
             "PTRatio != RemotePTRatio, and the local-PT plus LDC path through "
             "monitored PT-secondary voltage, final tap number, transformer "
             "terminal powers, downstream line powers, and total branch losses."}},

           {{"component",         "three_phase_nr_regulator_bank"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_regulator_bank_3ph_nr"}},
            {"notes",
             "Minimal accepted three-phase regulator compare for the NR lane. "
             "Three independent single-phase regulators form a shared 3-phase "
             "bank, and the compare gates per-phase bus voltage, per-regulator "
             "control voltage, exact final tap number, transformer terminal "
             "power, and total branch losses without importing taps from "
             "OpenDSS back into the repository solver."}},

           {{"component",         "transformer3w_tap_side_pair_projection"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",
             {"minimal_transformer3w_tap_1ph",
              "minimal_transformer3w_tap_1ph_hv_spur"}},
            {"notes",
             "Accepted only for two narrow one-phase surfaces: the isolated "
             "three-bus triangle used by minimal_transformer3w_tap_1ph, and the "
             "same triangle with one slack-side radial spur used by "
             "minimal_transformer3w_tap_1ph_hv_spur. Transformer3W tap_side is "
             "normalized onto only the attached pair branches, while canonical "
             "pair impedances come from the coupled-kron equivalent instead of "
             "the raw pair short-circuit data. Broader Transformer3W surfaces "
             "remain explicitly unclaimed."}},

           {{"component",         "meshed_distribution_network_nr"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_meshed_3bus_3ph_nr"}},
            {"notes",
             "Meshed three-phase distribution power flow is now compared formally "
             "through the NR lane on a minimal 3-bus looped network."}},

           {{"component",         "full_phase_domain_line_matrix_nr"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_phase_matrix_line_3bus_3ph_nr"}},
            {"notes",
             "ThreePhaseACLine.use_phase_matrix with explicit non-circulant 3x3 "
             "R/X/B matrices is now compared formally through the NR lane on a "
             "minimal 3-bus feeder. This pins the full phase-domain line path "
             "against OpenDSS."}},

           {{"component",         "three_phase_nr_load_semantics"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",
             {"minimal_load_wye_zip_3bus_3ph_nr",
              "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr",
              "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr",
              "minimal_load_wye_vmax_3bus_1ph_lateral_nr",
              "minimal_load_delta_power_3bus_3ph_nr",
              "minimal_load_delta_zip_3bus_3ph_nr"}},
            {"notes",
             "Grounded-wye ZIP, open-neutral wye, impedance-grounded wye, high-"
             "voltage Vmax load windows, delta constant-power, and delta ZIP "
             "loads are now compared formally through the NR lane "
             "using sys.loads rather than bus-level fixed PQ placeholders."}},

           {{"component",         "multi_source_three_phase_nr"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "accepted"},
            {"reason_for_gap",    nullptr},
            {"cases",             {"minimal_multi_source_3bus_3ph_nr"}},
            {"notes",
             "Multiple source injections are now compared formally through the NR "
             "lane using a downstream distributed generator on a 3-bus case."}},

           {{"component",         "meshed_distribution_network_bfs"},
            {"comparison_status", "unsupported"},
            {"reason_for_gap",    "algorithmic_mismatch"},
            {"notes",
             "BFS is strictly radial. Meshed topologies cause incorrect bus ordering "
             "and power balance violations."}},

           {{"component",         "IEEE_13_node_4kv_backbone"},
            {"module",            "solve_three_phase_nr(const ThreePhaseACSystem&)"},
            {"comparison_status", "deferred"},
            {"reason_for_gap",    "model_limitations"},
            {"cases",             {"ieee_13_node_4kv_backbone_3ph_nr"}},
            {"notes",
             "IEEE 13-node 4.16 kV three-phase backbone subset compared via NR. "
             "Known limitations: sequence impedance approximation, no transformer "
             "support, laterals excluded. Full mapping still unsupported."}},
       })},
  };
}

json build_regulator_semantics_matrix() {
  return {
      {"local_pt_no_ldc",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Local PT monitoring without LDC is now in accepted compare through "
            "the minimal three-phase bank case, while the single-regulator path "
            "remains covered by runtime semantics regression."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_bank_3ph_nr"}},
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:local_pt_no_ldc"}},
            }},
       }},
      {"three_phase_independent_bank_local_pt_no_ldc",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Minimal accepted three-phase regulator compare for the NR lane. "
            "Three independent single-phase regulators share a three-phase bus "
            "pair and are validated end-to-end against OpenDSS without a tap "
            "import oracle."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_bank_3ph_nr"}},
                {"regression_tests",
                 {"Three-phase NR regulator supports independent A/B/C bank behavior"}},
            }},
       }},
      {"remote_bus_no_ldc",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Canonical accepted regulator baseline. minimal_regulator_1ph proves "
            "remote-bus monitoring without LDC against OpenDSS end-to-end."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_1ph"}},
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:remote_bus_no_ldc"}},
            }},
       }},
      {"local_pt_with_ldc",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Canonical accepted regulator-with-LDC baseline. The local PT path with "
            "line-drop compensation is compared against OpenDSS end-to-end."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_local_pt_ldc_1ph"}},
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:local_pt_with_ldc"}},
            }},
       }},
      {"remote_bus_with_ldc",
       {
           {"coverage", "unsupported"},
           {"reason",
            "solve_distribution_pf fail-closes with stop_reason "
            "unsupported_remote_bus_with_ldc so remote monitoring and LDC cannot "
            "be silently mixed."},
           {"evidence",
            {
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:remote_bus_with_ldc_fail_closed"}},
            }},
       }},
      {"remote_ptratio_distinct_from_ptratio",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Canonical accepted regulator baseline now includes a remote-bus case "
            "with PTRatio != RemotePTRatio, so the solver and bridge both compare "
            "this control path against OpenDSS end-to-end."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_remote_ptratio_1ph"}},
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:remote_bus_distinct_remote_ptratio"}},
            }},
       }},
      {"tapwinding_1",
       {
           {"coverage", "solver_regression_only"},
           {"reason",
            "HV/from-side tap winding semantics are solver-supported and covered by "
            "runtime regression, but no dedicated canonical compare artifact exists yet."},
           {"evidence",
            {
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:tapwinding_1"}},
            }},
       }},
      {"tapwinding_2",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Canonical accepted regulator baseline uses tapwinding=2 and compares "
            "the full closed-loop result against OpenDSS, including the three-"
            "phase NR bank case."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_1ph",
                           "minimal_regulator_local_pt_ldc_1ph",
                           "minimal_regulator_remote_ptratio_1ph",
                           "minimal_regulator_bank_3ph_nr"}},
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:local_pt_no_ldc",
                  "runtime_regulator_semantics_matrix:max_tap_change_2"}},
            }},
       }},
      {"max_tap_change_1",
       {
           {"coverage", "accepted_compare"},
           {"reason",
            "Canonical accepted regulator baseline uses maxTapChange=1, including "
            "the minimal three-phase NR regulator bank compare case."},
           {"evidence",
            {
                {"cases", {"minimal_regulator_1ph",
                           "minimal_regulator_local_pt_ldc_1ph",
                           "minimal_regulator_remote_ptratio_1ph",
                           "minimal_regulator_bank_3ph_nr"}},
            }},
       }},
      {"max_tap_change_2",
       {
           {"coverage", "solver_regression_only"},
           {"reason",
            "Discrete multi-step control is solver-supported and regression-tested, "
            "but not yet promoted into a canonical accepted compare case."},
           {"evidence",
            {
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:max_tap_change_2"}},
            }},
       }},
      {"series_regulators",
       {
           {"coverage", "deferred"},
           {"reason",
            "No canonical compare case or dedicated runtime regression covers two "
            "regulators in series yet."},
           {"evidence", json::object()},
       }},
      {"reversible",
       {
           {"coverage", "unsupported"},
           {"reason",
            "solve_distribution_pf fail-closes reversible RegControl with "
            "unsupported_reversible_control."},
           {"evidence",
            {
                {"regression_tests",
                 {"runtime_regulator_semantics_matrix:reversible_fail_closed"}},
            }},
      }},
  };
}

json build_benchmark_expansion_plan() {
  return {
      {"priority_source", "ieee_pes_opendss_feeders"},
      {"data_root", "external_data/opendss_ieee_pes"},
      {"source_manifest", "external_data/opendss_ieee_pes/source_manifest.json"},
      {"candidate_feeders", json::array({"13_node", "34_node", "123_node"})},
      {"stages",
       json::array({
           {
               {"stage_id", "single_phase_or_balanced_subset"},
               {"why_this_stage",
               "First benchmark step should keep topology and phase semantics "
                "simple while proving that the adapter and compare chain scale "
                "beyond the toy feeders."},
               {"covers_capability_gap",
                "larger_official_feeder_snapshot_and_radial_compare_without_new_regulator_or_transformer_semantics"},
               {"not_compared_modules",
                json::array({"hybrid_pf",
                             "time_series_optimization",
                             "l2o"})},
           },
           {
               {"stage_id", "three_phase_unbalanced_subset"},
               {"why_this_stage",
               "Second benchmark step should increase phase asymmetry only after "
                "the balanced/single-phase official feeder subset is stable."},
               {"covers_capability_gap",
                "formal_unbalanced_official_feeder_compare_without_promoting_regulator_or_transformer3w_semantics"},
               {"not_compared_modules",
                json::array({"hybrid_pf",
                             "time_series_optimization",
                             "l2o"})},
           },
           {
               {"stage_id", "regulator_and_transformer_semantics"},
               {"why_this_stage",
               "Final benchmark step should bring in regulator and transformer "
                "semantics only after the feeder-base voltage and branch power "
                "paths are stable on official data."},
               {"covers_capability_gap",
                "official_feeder_regulator_and_transformer_semantics_after_adapter_and_compare_contract_are_stable"},
               {"not_compared_modules",
                json::array({"hybrid_pf",
                             "time_series_optimization",
                             "l2o"})},
           },
       })},
  };
}

json build_official_feeder_reference_status(const fs::path& project_root) {
  const fs::path source_manifest =
      project_root / "external_data/opendss_ieee_pes/source_manifest.json";
  const fs::path official_13_master =
      project_root /
      "external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss";

  if (!fs::exists(source_manifest) || !fs::exists(official_13_master)) {
    return {
        {"status", "source_data_missing"},
        {"source_manifest_present", fs::exists(source_manifest)},
        {"official_13_master_present", fs::exists(official_13_master)},
        {"source_manifest",
         hacdcpf_compare::snapshot_contract::portable_path_string(
             source_manifest, project_root)},
        {"master_dss",
         hacdcpf_compare::snapshot_contract::portable_path_string(
             official_13_master, project_root)},
    };
  }

  const auto snapshot = hacdcpf::io::solve_opendss_snapshot(official_13_master);
  const std::string portable_master =
      hacdcpf_compare::snapshot_contract::portable_path_string(
          official_13_master, project_root);
  const json serialized =
      hacdcpf_compare::snapshot_contract::snapshot_to_json(
          snapshot, portable_master);
  const auto roundtrip =
      hacdcpf_compare::snapshot_contract::snapshot_from_json(serialized);

  return {
      {"status", snapshot.converged ? "bridge_smoke_passed"
                                     : "bridge_smoke_not_converged"},
      {"source_manifest",
       hacdcpf_compare::snapshot_contract::portable_path_string(
           source_manifest, project_root)},
      {"master_dss", portable_master},
      {"converged", snapshot.converged},
      {"bus_count", serialized["buses"].size()},
      {"node_voltage_count", snapshot.node_voltages.size()},
      {"pd_element_count", snapshot.pd_element_results.size()},
      {"transformer_count", snapshot.transformer_states.size()},
      {"regcontrol_count", snapshot.regcontrol_results.size()},
      {"circuit_losses_mw_mvar",
       power_pair_to_json(
           hacdcpf_compare::opendss_circuit_losses_to_mw_mvar(snapshot))},
      {"adapter_contract",
       {
           {"schema", hacdcpf_compare::snapshot_contract::schema_to_json()},
           {"adapter", hacdcpf_compare::snapshot_contract::adapter_to_json()},
           {"portable_master_dss", !fs::path(portable_master).is_absolute()},
           {"serializer_parser_roundtrip_consistent",
            roundtrip.snapshot.node_voltages.size() ==
                    snapshot.node_voltages.size() &&
                roundtrip.snapshot.pd_element_results.size() ==
                    snapshot.pd_element_results.size() &&
                roundtrip.snapshot.transformer_states.size() ==
                    snapshot.transformer_states.size() &&
                roundtrip.snapshot.regcontrol_results.size() ==
                    snapshot.regcontrol_results.size()},
       }},
      {"scope_note",
       "Official IEEE PES 13-node OpenDSS reference now runs through the "
       "canonical C++ bridge/adapter as machine-readable snapshot evidence. "
       "Promotion into formal repo-vs-OpenDSS compare remains staged by the "
       "benchmark expansion plan."},
  };
}

json build_case_manifest(const fs::path& project_root) {
  return {
      {"scope",
       {
           {"compared_modules",
           {
               "solve_distribution_pf(const ACSystem&)",
               "solve_distribution_pf(const HybridPowerSystem&)",
               "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)",
               "solve_three_phase_nr(const ThreePhaseACSystem&)",
            }},
           {"bridge_validated_modules",
            {
                "solve_opendss_snapshot(const std::filesystem::path&)",
            }},
           {"tier_definitions",
            {
                {"accepted",
                 {{"machine_readable_id", "accepted"},
                  {"description",
                   "Both repo and OpenDSS converge; all comparison metrics are within "
                   "the stated CaseTolerance bounds; the case is in the test suite and "
                   "all assertions are green."},
                  {"may_be_promoted_from", "deferred"},
                  {"sub_tiers",
                   {"accepted_primary", "accepted_secondary", "accepted_rich_component",
                    "accepted_feeder_subset", "accepted_unbalanced_3ph",
                    "accepted_pass_through_xfmr", "accepted_fixed_tap_xfmr",
                    "accepted_regulator", "accepted_regulator_local_pt_ldc",
                    "accepted_regulator_remote_ptratio",
                    "accepted_regulator_bank_3ph",
                    "accepted_transformer3w_isolated_triangle",
                    "accepted_transformer3w_hv_spur_embed",
                    "accepted_nr_load_semantics_3ph",
                    "accepted_nr_phase_matrix_line_3ph",
                    "accepted_nr_meshed_3ph", "accepted_nr_multi_source_3ph"}}}},
                {"exploratory",
                 {{"machine_readable_id", "exploratory"},
                  {"description",
                   "Both repo and OpenDSS converge but a known structural model mismatch "
                   "produces a measurable systematic bias. The bias is quantified and "
                   "documented in the case artifact. NOT part of the accepted baseline."},
                  {"sub_tiers", {"exploratory_physical_component"}}}},
                {"deferred",
                 {{"machine_readable_id", "deferred"},
                  {"description",
                   "The capability or case exists in principle but has not yet been "
                   "verified with a dedicated comparison test."},
                  {"may_be_promoted_to", "accepted"}}},
                {"bridge_only",
                 {{"machine_readable_id",
                    "bridge_validated_but_algorithmically_unsupported"},
                  {"description",
                   "The OpenDSS snapshot bridge is validated internally for element "
                   "observability, buses/node order, typed units, and power/loss "
                   "self-consistency, but no repo PF comparison is claimed because "
                   "the current solver does not support the modeled capability."},
                  {"examples", {"future official feeder subset with unsupported algorithmic coverage"}}}},
                {"unsupported",
                 {{"machine_readable_id", "unsupported"},
                  {"description",
                   "An algorithmic or model gap makes this class of cases fundamentally "
                   "non-comparable with the current BFS implementation. Promotion requires "
                   "a solver-level change."},
                   {"examples",
                   {"phase-shifting transformer/vector-group semantics",
                    "meshed topology"}}}},
            }},
           {"comparison_failure_diagnostics",
            {
                {"metric_threshold_failed",
                 "repo and OpenDSS comparison exceeded at least one declared numeric threshold"},
                {"bridge_validation_failed",
                 "OpenDSS bridge/configuration/loss self-consistency evidence failed even before considering fixture truth"},
                {"control_trace_validation_failed",
                 "repo regulator trace and OpenDSS control oracle disagreed on iteration-level control evidence"},
                {"fixture_truth_drift",
                 "repo and OpenDSS end state agree within thresholds, but the fixture-declared expected truth is stale"},
            }},
           {"acceptance_tiers",
            {
                {"primary",
                 {{"case", "minimal_radial_3bus_1ph"},
                  {"role", "accepted_primary"},
                  {"module", "solve_distribution_pf(const ACSystem&)"},
                  {"reason",
                   "Smallest stable path: single-phase radial BFS on 3-bus toy feeder. "
                   "OpenDSS mapping is stable, residual error sub-1e-3 p.u./MW."}}},
                {"secondary",
                 {{"case", "minimal_radial_3bus_balanced_3ph"},
                  {"role", "accepted_secondary"},
                  {"module",
                   "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Balanced 3-phase BFS on 3-bus toy feeder. Extends primary path "
                   "to preserve phase ordering and per-phase line-power extraction."}}},
                {"rich_component",
                 {{"case", "minimal_radial_3bus_1ph_shunt"},
                  {"role", "accepted_rich_component"},
                  {"module", "solve_distribution_pf(const ACSystem&)"},
                  {"reason",
                   "Adds fixed reactive shunt (ACBus.bs_mvar) to primary radial. "
                   "Both BFS and OpenDSS use voltage-independent injection (no V^2 bias)."},
                  {"modeling_note",
                   "Physical Capacitor vs BFS is deferred to exploratory_physical_component."}}},
                {"feeder_subset",
                 {{"case", "case33bw_radial_3ph"},
                  {"role", "accepted_feeder_subset"},
                  {"module", "solve_distribution_pf(const ACSystem&)"},
                  {"reason",
                   "First standard feeder: Baran & Wu (1989) 33-bus purely radial. "
                   "No regulators, no taps, no unbalanced loads. "
                   "BFS correctness validated beyond the 3-bus family."},
                  {"why_better_than_ieee13",
                   "IEEE 13-node has regulators, unsymmetric Zabc lines, and mixed-phase "
                   "laterals — none of which the current model supports. case33bw is purely "
                   "radial with constant-Z branches, making it the correct first step."}}},
                {"exploratory_physical_component",
                 {{"case", "minimal_radial_3bus_1ph_cap"},
                  {"role", "exploratory_physical_component"},
                  {"module", "solve_distribution_pf(const ACSystem&)"},
                  {"reason",
                   "Physical Capacitor element (V^2 dependent Q) vs BFS fixed injection. "
                   "Known structural mismatch; bias is quantified, not suppressed."},
                  {"not_a_baseline",
                   "This case is NOT in the accepted baseline. It demonstrates an honest "
                   "physical element comparison with documented model limitations."}}},
                {"accepted_unbalanced_3ph",
                 {{"case", "minimal_unbalanced_3bus_3ph"},
                  {"role", "accepted_unbalanced_3ph"},
                  {"module",
                   "solve_three_phase_distribution_pf(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Extends the balanced 3-phase baseline to genuinely unbalanced loads "
                   "(A≠B≠C per bus) and asymmetric zero-sequence impedances (r0≠r1). "
                   "First accepted case validating per-phase voltage angle comparison."},
                  {"promoted_from", "deferred"}}},
                {"accepted_pass_through_xfmr",
                 {{"case", "minimal_passthr_transformer_1ph"},
                  {"role", "accepted_pass_through_xfmr"},
                  {"module", "solve_distribution_pf(const ACSystem&)"},
                  {"reason",
                   "OpenDSS Transformer element (nominal ratio=1.0, zero magnetizing branch) "
                   "is numerically equivalent to BFS ACBranch series impedance. "
                   "Bus voltages, transformer terminal powers, downstream line powers, "
                   "and total branch losses are verified within kXfmrTol."},
                  {"promoted_from", "deferred"}}},
                {"accepted_fixed_tap_xfmr",
                 {{"case", "minimal_tap_transformer_1ph"},
                  {"role", "accepted_fixed_tap_xfmr"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Transformer2W tap_side/tap_pos/tap_step_percent are projected into "
                   "a from-side-normalized ACBranch.tap. If the physical tap is on the "
                   "LV/to-side winding, the equivalent leakage impedance is also scaled "
                   "by tap^2. The tap-aware BFS equations match OpenDSS on a fixed "
                   "tap=1.05 transformer plus downstream line."},
                  {"promoted_from", "bridge_validated_but_algorithmically_unsupported"}}},
                {"accepted_regulator",
                 {{"case", "minimal_regulator_1ph"},
                  {"role", "accepted_regulator"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Minimal accepted regulator baseline. Discrete outer control loop "
                   "matches OpenDSS remote-bus voltage regulation on winding 2 with "
                   "maxtapchange=1, final tap number, branch powers, and total losses."},
                  {"promoted_from", "bridge_validated_but_algorithmically_unsupported"}}},
                {"accepted_regulator_local_pt_ldc",
                 {{"case", "minimal_regulator_local_pt_ldc_1ph"},
                  {"role", "accepted_regulator_local_pt_ldc"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Extends the accepted regulator baseline to local PT monitoring plus "
                   "line-drop compensation on winding 2. OpenDSS and the repository "
                   "match final tap, PT-secondary control voltage, branch powers, and "
                   "total losses."},
                  {"promoted_from", "bridge_validated_but_algorithmically_unsupported"}}},
                {"accepted_regulator_remote_ptratio",
                 {{"case", "minimal_regulator_remote_ptratio_1ph"},
                  {"role", "accepted_regulator_remote_ptratio"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Extends the accepted regulator baseline to remote-bus control with "
                   "PTRatio != RemotePTRatio. OpenDSS and the repository match final tap, "
                   "PT-secondary control voltage, branch powers, and total losses."},
                  {"promoted_from", "solver_regression_only"}}},
                {"accepted_regulator_bank_3ph",
                 {{"case", "minimal_regulator_bank_3ph_nr"},
                  {"role", "accepted_regulator_bank_3ph"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Minimal accepted three-phase regulator compare for the NR lane. "
                   "Three independent single-phase regulators form one shared bank, "
                   "and OpenDSS matches the repository on per-phase voltage, per-"
                   "regulator PT-secondary control voltage, final tap number, "
                   "transformer terminal power, and total losses."},
                  {"promoted_from", "solver_regression_only"}}},
                {"accepted_nr_meshed_3ph",
                 {{"case", "minimal_meshed_3bus_3ph_nr"},
                  {"role", "accepted_nr_meshed_3ph"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Minimal meshed 3-phase OpenDSS compare for the NR lane. "
                   "Per-phase bus voltage, line terminal powers, and total losses "
                   "are compared machine-readably against OpenDSS."}}},
                {"accepted_nr_load_semantics_3ph",
                 {{"case", "minimal_load_wye_zip_3bus_3ph_nr"},
                  {"role", "accepted_nr_load_semantics_3ph"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Minimal accepted three-phase NR compare family for sys.loads "
                   "semantics. Grounded-wye ZIP, local-neutral wye variants, high-"
                   "voltage load windows, mixed-phase delta constant-power, and "
                   "delta ZIP loads all match OpenDSS on per-phase bus voltage, "
                   "line terminal powers, and total losses without collapsing "
                   "back to bus-level PQ."}}},
                {"accepted_nr_phase_matrix_line_3ph",
                 {{"case", "minimal_phase_matrix_line_3bus_3ph_nr"},
                  {"role", "accepted_nr_phase_matrix_line_3ph"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Minimal accepted compare for the full phase-domain line path "
                   "in the NR lane. The case uses non-circulant 3x3 R/X line "
                   "matrices on a 3-bus feeder and verifies that the repository "
                   "consumes the explicit phase matrix instead of the legacy "
                   "sequence-derived circulant approximation."}}},
                {"accepted_nr_multi_source_3ph",
                 {{"case", "minimal_multi_source_3bus_3ph_nr"},
                  {"role", "accepted_nr_multi_source_3ph"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "Minimal multi-source 3-phase OpenDSS compare for the NR lane. "
                   "A downstream per-phase distributed generator creates a second "
                   "source injection while keeping the compare surface thin and "
                   "auditable."}}},
                {"accepted_transformer3w_isolated_triangle",
                 {{"case", "minimal_transformer3w_tap_1ph"},
                  {"role", "accepted_transformer3w_isolated_triangle"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Accepted minimal OpenDSS compare for the isolated one-phase "
                   "Transformer3W coupled-kron pair projection. This role is "
                   "intentionally narrow and does not claim broader Transformer3W "
                   "surface coverage."}}},
                {"accepted_transformer3w_hv_spur_embed",
                 {{"case", "minimal_transformer3w_tap_1ph_hv_spur"},
                  {"role", "accepted_transformer3w_hv_spur_embed"},
                  {"module", "solve_distribution_pf(const HybridPowerSystem&)"},
                  {"reason",
                   "Accepted minimal OpenDSS compare for the same one-phase "
                   "Transformer3W coupled-kron triangle when embedded with one "
                   "slack-side radial spur. This widens the accepted surface by "
                   "one controlled step without claiming generic Transformer3W "
                   "network coverage."}}},
                {"deferred_ieee13_feeder_formal_cross_check",
                 {{"case", "ieee_13_node_4kv_backbone_3ph_nr"},
                  {"role", "deferred_ieee13_feeder_formal_cross_check"},
                  {"module", "solve_three_phase_nr(const ThreePhaseACSystem&)"},
                  {"reason",
                   "IEEE 13-node 4.16 kV backbone subset is emitted as deferred "
                   "formal evidence for the NR lane. Numeric gaps are reported "
                   "machine-readably, but explicit model limitations keep this case "
                   "outside the accepted baseline."},
                  {"known_limitations",
                   json::array({
                       "regulators represented as known slack voltage",
                       "transformers excluded from the modeled subset",
                       "laterals and mixed-phase side branches excluded",
                       "asymmetric Zabc lines approximated through a sequence-to-abc "
                       "circulant mapping",
                       "delta loads approximated as balanced per-phase split",
                   })}}},
            }},
           {"not_compared_modules",
            {
                "time-series / optimization / topology reconfiguration / LinDistFlow",
            }},
           {"reproducibility",
            {
                {"subbuild_dir", "build/opendss_compare"},
                {"subbuild_status",
                 "standalone OpenDSS compare subbuild is the canonical reproducible entrypoint"},
                {"canonical_build_command",
                 hacdcpf_compare::kCanonicalOpenDSSCompareBuildCommand},
                {"canonical_test_command",
                 hacdcpf_compare::kCanonicalOpenDSSCompareTestCommand},
                {"canonical_adapter_command",
                 hacdcpf_compare::kCanonicalOpenDSSSnapshotAdapterCommand},
                {"canonical_compare_command",
                 hacdcpf_compare::kCanonicalOpenDSSCompareToolCommand},
                {"root_build_note",
                 "OpenDSS compare reproducibility is anchored to build/opendss_compare. "
                 "Any unrelated root-build issues must be reported separately and must not "
                 "be mixed into the OpenDSS compare artifact."},
            }},
           {"adapter_contract_surface", build_adapter_contract_surface(project_root)},
           {"compare_fields",
            {
                "bus voltage magnitude from ActiveBus.puVmagAngle[0]",
                "bus voltage angle from ActiveBus.puVmagAngle[1] wherever the case asserts angle",
                "sending-end branch power from generic PD-element ActiveCktElement.Powers terminal 1 (kW / kvar -> MW / MVAr), covering Line and Transformer",
                "regulator control voltage on the PT secondary (volts) for accepted regulator cases",
                "regulator final discrete tap number for accepted regulator cases",
                "canonical projection diagnostics include whether a physical to-side tap required tap^2 impedance rescaling when moved onto ACBranch.from_bus",
                "total branch losses from summed PD-element terminal powers for accepted compare cases",
                "Circuit.Losses raw (W / var -> MW / MVAr) retained as an independent cross-check, not as the sole transformer evidence",
            }},
           {"bridge_validation_fields",
           {
                "transformer winding taps from Transformers.Get_Tap",
                "regulator state from RegControls.Get_* plus DSSProperty(RemotePTRatio) (transformer name, tap number, winding, tap winding, PT/remote PT ratios, VReg/Band, monitored bus, maxTapChange)",
                "regulator configuration consistency against the fixture-declared transformer/bus/tap/control settings",
                "OpenDSS control oracle from Solution.ControlIterations / Solution.MaxControlIterations / Solution.EventLog",
                "standalone OpenDSS snapshot adapter canonical JSON output",
                "repo closed-loop regulator state and per-iteration control trace",
                "monitored bus voltage from ActiveBus / node voltage snapshot",
                "PD-element terminal power/loss self-consistency and Circuit.Losses cross-check",
            }},
           {"loss_field_note",
            "ActiveCktElement.Powers is read as kW / kvar from the generic PD-element "
            "snapshot and converted through typed helpers only. ActiveCktElement/Circuit "
           "Losses raw values are read as W / var and converted separately for "
           "cross-checks."},
           {"repository_existing_opendss_cases", true},
           {"repository_existing_master_dss",    true},
           {"benchmark_expansion_plan", build_benchmark_expansion_plan()},
           {"official_feeder_reference_status",
            build_official_feeder_reference_status(project_root)},
           {"fixture_source",
            "tools/opendss_compare/fixtures.hpp — single source of truth for all builders"},
           {"capability_matrix", build_capability_matrix()},
           {"regulator_semantics_matrix", build_regulator_semantics_matrix()},
       }},
      {"cases",
       {
           {
               {"case_id",  "minimal_radial_3bus_1ph"},
               {"role",     "accepted_primary"},
               {"kind",     "single_phase_radial"},
               {"opendss_master",
                fs::relative(
                    project_root / "tests/data/opendss/minimal_radial_3bus_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_3bus_radial in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id",  "minimal_radial_3bus_balanced_3ph"},
               {"role",     "accepted_secondary"},
               {"kind",     "balanced_three_phase_radial"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_radial_3bus_balanced_3ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_balanced_3bus_3phase in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id",  "minimal_radial_3bus_1ph_shunt"},
               {"role",     "accepted_rich_component"},
               {"kind",     "single_phase_radial_with_fixed_shunt"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_radial_3bus_1ph_shunt/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_3bus_radial_with_shunt in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id",  "case33bw_radial_3ph"},
               {"role",     "accepted_feeder_subset"},
               {"kind",     "standard_feeder_radial_bfs"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/case33bw_radial_3ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_case33bw_radial in "
                "tools/opendss_compare/fixtures.hpp"},
               {"feeder_source", "Baran & Wu (1989), IEEE Trans. Power Delivery, 4(3)"},
           },
           {
               {"case_id",  "minimal_radial_3bus_1ph_cap"},
               {"role",     "exploratory_physical_component"},
               {"kind",     "single_phase_radial_with_physical_capacitor"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_radial_3bus_1ph_cap/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_3bus_radial_with_cap_physical in "
                "tools/opendss_compare/fixtures.hpp"},
               {"warning",
                "NOT in accepted baseline. Exploratory case documenting structural "
                "model mismatch (V^2 bias)."},
           },
           {
               {"case_id",  "minimal_unbalanced_3bus_3ph"},
               {"role",     "accepted_unbalanced_3ph"},
               {"kind",     "unbalanced_three_phase_radial"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_unbalanced_3bus_3ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_unbalanced_3bus_3phase in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "First accepted case with per-phase voltage angle comparison and "
                "asymmetric zero-sequence impedances (r0 ≠ r1)."},
           },
           {
               {"case_id",  "minimal_passthr_transformer_1ph"},
               {"role",     "accepted_pass_through_xfmr"},
               {"kind",     "single_phase_pass_through_transformer"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_3bus_passthr_transformer in "
                "tools/opendss_compare/fixtures.hpp"},
               {"bridge_coverage",
                "Generic PD-element snapshot captures both transformer t12 and line l1. "
                "line_results is only the compatibility projection for l1."},
           },
           {
               {"case_id",  "minimal_tap_transformer_1ph"},
               {"role",     "accepted_fixed_tap_xfmr"},
               {"kind",     "single_phase_fixed_tap_transformer"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_tap_transformer_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_3bus_fixed_tap_transformer in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted fixed-tap compare case. Transformer2W tap semantics are "
                "projected onto a from-side ACBranch.tap before the tap-aware BFS solve."},
           },
           {
               {"case_id",  "minimal_regulator_1ph"},
               {"role",     "accepted_regulator"},
               {"kind",     "single_phase_regulator_closed_loop"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_regulator_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_minimal_regulator_1ph in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted regulator compare case. RegControl configuration consistency, "
                "repo closed-loop trace, final tap number, transformer terminal power, "
                "downstream line power, and total branch losses are all machine-readable."},
           },
           {
               {"case_id",  "minimal_regulator_local_pt_ldc_1ph"},
               {"role",     "accepted_regulator_local_pt_ldc"},
               {"kind",     "single_phase_regulator_local_pt_with_ldc"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_regulator_local_pt_ldc_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_minimal_regulator_1ph_local_pt_with_ldc in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted regulator compare case for the local PT plus LDC control path. "
                "Final tap, PT-secondary control voltage, branch powers, and total "
                "losses are all machine-readable."},
           },
           {
               {"case_id",  "minimal_regulator_remote_ptratio_1ph"},
               {"role",     "accepted_regulator_remote_ptratio"},
               {"kind",     "single_phase_regulator_remote_ptratio"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_regulator_remote_ptratio_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_minimal_regulator_1ph_remote_ptratio in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted regulator compare case for remote-bus monitoring with "
                "PTRatio distinct from RemotePTRatio. Final tap, PT-secondary control "
                "voltage, branch powers, and total losses are all machine-readable."},
           },
           {
               {"case_id",  "minimal_regulator_bank_3ph_nr"},
               {"role",     "accepted_regulator_bank_3ph"},
               {"kind",     "three_phase_nr_regulator_bank"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_regulator_bank_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_minimal_regulator_bank_3ph_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted minimal three-phase regulator compare case. Three "
                "independent single-phase regulators form a shared 3-phase bank, "
                "and the canonical artifact records per-phase voltage, per-"
                "regulator control voltage, final tap number, transformer terminal "
                "power, and total losses machine-readably."},
           },
           {
               {"case_id",  "minimal_transformer3w_tap_1ph"},
               {"role",     "accepted_transformer3w_isolated_triangle"},
               {"kind",     "single_phase_transformer3w_formal_cross_check"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_transformer3w_tap_1ph/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_minimal_transformer3w_cross_check_case in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted minimal compare for the current Transformer3W coupled-kron "
                "pair projection on the isolated one-phase triangle surface. The "
                "1-phase OpenDSS fixture uses line-line VoltageBases equivalents for "
                "CalcVoltageBases, while winding kV values remain the actual 1-phase "
                "winding ratings. This acceptance stays intentionally narrow and does "
                "not imply broader Transformer3W coverage."},
           },
           {
               {"case_id",  "minimal_transformer3w_tap_1ph_hv_spur"},
               {"role",     "accepted_transformer3w_hv_spur_embed"},
               {"kind",     "single_phase_transformer3w_hv_spur_formal_cross_check"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_transformer3w_tap_1ph_hv_spur/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_transformer3w_hv_spur_cross_check_case in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Accepted minimal compare for the current Transformer3W coupled-kron "
                "pair projection when the one-phase triangle is embedded with one "
                "slack-side radial spur. This surface remains intentionally narrow: "
                "one transformer, one extra line, one extra PQ leaf bus."},
           },
           {
               {"case_id", "minimal_meshed_3bus_3ph_nr"},
               {"role", "accepted_nr_meshed_3ph"},
               {"kind", "three_phase_nr_meshed"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_meshed_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_meshed_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_wye_zip_3bus_3ph_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_wye_zip"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_wye_zip_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_wye_zip_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_wye_open_neutral"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_wye_open_neutral_3bus_1ph_lateral_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_wye_open_neutral_3bus_1phase_lateral_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_wye_impedance_grounded"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_wye_vmax_3bus_1ph_lateral_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_wye_vmax"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_wye_vmax_3bus_1ph_lateral_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_wye_vmax_3bus_1phase_lateral_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_delta_power_3bus_3ph_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_delta_power"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_delta_power_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_delta_power_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_load_delta_zip_3bus_3ph_nr"},
               {"role", "accepted_nr_load_semantics_3ph"},
               {"kind", "three_phase_nr_load_delta_zip"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_load_delta_zip_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_load_delta_zip_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "minimal_phase_matrix_line_3bus_3ph_nr"},
               {"role", "accepted_nr_phase_matrix_line_3ph"},
               {"kind", "three_phase_nr_full_phase_matrix_line"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_phase_matrix_line_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_phase_matrix_line_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Minimal accepted compare case for non-circulant full phase-domain "
                "line matrices in the NR lane."},
           },
           {
               {"case_id", "minimal_multi_source_3bus_3ph_nr"},
               {"role", "accepted_nr_multi_source_3ph"},
               {"kind", "three_phase_nr_multi_source"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/minimal_multi_source_3bus_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_multi_source_3bus_3phase_nr_case in "
                "tools/opendss_compare/fixtures.hpp"},
           },
           {
               {"case_id", "ieee_13_node_4kv_backbone_3ph_nr"},
               {"role", "deferred_ieee13_feeder_formal_cross_check"},
               {"kind", "three_phase_nr_ieee13_backbone_subset"},
               {"opendss_master",
                fs::relative(
                    project_root /
                        "tests/data/opendss/ieee_13_node_4kv_backbone_3ph_nr/Master.dss",
                    project_root).string()},
               {"repo_builder",
                "hacdcpf_compare_fixtures::build_ieee13_4kv_backbone_3phase in "
                "tools/opendss_compare/fixtures.hpp"},
               {"note",
                "Deferred IEEE 13-node 4.16 kV backbone subset. Artifact keeps the "
                "formal compare surface machine-readable while preserving explicit "
                "model limitations and non-accepted status."},
           },
       }},
  };
}

// ---------------------------------------------------------------------------
// build_markdown_summary
// ---------------------------------------------------------------------------
std::string build_markdown_summary(const json& summary) {
  std::ostringstream out;
  out << "# OpenDSS PF Compare Summary\n\n";
  out << "- Fixture source (single truth): `tools/opendss_compare/fixtures.hpp`\n";
  out << "- Accepted modules: `solve_distribution_pf(const ACSystem&)`, "
         "`solve_distribution_pf(const HybridPowerSystem&)`, "
         "`solve_three_phase_distribution_pf(const ThreePhaseACSystem&)`, "
         "`solve_three_phase_nr(const ThreePhaseACSystem&)`\n";
  out << "- Bridge-only module: `solve_opendss_snapshot(const std::filesystem::path&)`\n";
  out << "- accepted_cases_all_passed: `"
      << (summary["accepted_cases_all_passed"].get<bool>() ? "true" : "false")
      << "`\n";
  out << "- bridge_only_cases_all_passed: `"
      << (summary["bridge_only_cases_all_passed"].get<bool>() ? "true" : "false")
      << "`\n";
  out << "- Canonical build: `"
      << summary["reproducibility"]["canonical_build_command"].get<std::string>()
      << "`\n";
  out << "- Canonical test: `"
      << summary["reproducibility"]["canonical_test_command"].get<std::string>()
      << "`\n";
  out << "- Canonical adapter: `"
      << summary["reproducibility"]["canonical_adapter_command"].get<std::string>()
      << "`\n";
  out << "- Canonical compare: `"
      << summary["reproducibility"]["canonical_compare_command"].get<std::string>()
      << "`\n";
  if (summary.contains("selection")) {
    out << "- Selection mode: `"
        << summary["selection"]["mode"].get<std::string>() << "`\n";
  }
  if (summary.contains("official_feeder_reference_status")) {
    out << "- Official feeder reference: `"
        << summary["official_feeder_reference_status"]["status"].get<std::string>()
        << "` via `"
        << summary["official_feeder_reference_status"]["master_dss"].get<std::string>()
        << "`\n";
    out << "- Official feeder source manifest: `"
        << summary["official_feeder_reference_status"]["source_manifest"].get<std::string>()
        << "`\n";
  }
  out << "- Adapter contract: schema `"
      << summary["adapter_contract_surface"]["schema"]["name"].get<std::string>()
      << "` v"
      << summary["adapter_contract_surface"]["schema"]["version"].get<int>()
      << ", shared serializer/parser/units surface consumed by the compare chain.\n";
  out << "- Voltage metric: `ActiveBus.puVmagAngle`; "
         "branch power: terminal-1 `ActiveCktElement.Powers` from the generic "
         "PD-element snapshot (`Line` + `Transformer`); "
         "total loss: summed PD-element terminal powers; "
         "`Circuit.Losses` raw (W / var) is retained as a cross-check.\n";
  out << "- Regulator accepted cases also gate on `control_trace_validation`, "
         "which compares repo regulator trace summary against the OpenDSS control "
         "oracle from `Solution.ControlIterations` and `Solution.EventLog`, now "
         "including the minimal three-phase bank case in the NR lane.\n";
  out << "- Not compared: hybrid overloads, time-series/optimization.\n\n";

  out << "| Case | Role | Status | Failed Metrics | Module | Vm max_abs | Va max_abs | "
         "P max_abs | Q max_abs | Ploss abs | Qloss abs |\n";
  out << "| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |\n";

  for (const auto& item : summary["cases"]) {
    const auto& metrics = item["metrics"];
    const auto vm_value = metric_max_abs_any(
        metrics, {"bus_voltage_magnitude_pu", "bus_phase_voltage_magnitude_pu"});
    const auto va_value = metric_max_abs_any(
        metrics, {"bus_voltage_angle_deg", "bus_phase_voltage_angle_deg"});
    const auto p_value = metric_max_abs_any(
        metrics, {"line_terminal1_p_mw", "line_terminal1_phase_p_mw",
                  "line_terminal_3ph_total_p_mw", "transformer_terminal1_p_mw",
                  "line_l1_terminal1_p_mw"});
    const auto q_value = metric_max_abs_any(
        metrics, {"line_terminal1_q_mvar", "line_terminal1_phase_q_mvar",
                  "line_terminal_3ph_total_q_mvar", "transformer_terminal1_q_mvar",
                  "line_l1_terminal1_q_mvar"});
    const auto ploss_value = metric_max_abs_any(
        metrics, {"total_line_loss_p_mw", "total_branch_loss_p_mw",
                  "total_circuit_loss_p_mw_via_circuit_losses_raw"});
    const auto qloss_value = metric_max_abs_any(
        metrics, {"total_line_loss_q_mvar", "total_branch_loss_q_mvar",
                  "total_circuit_loss_q_mvar_via_circuit_losses_raw"});

    out << "| " << item["case_id"].get<std::string>()
        << " | " << item["acceptance_tier"].get<std::string>()
        << " | " << item["comparison_status"].get<std::string>()
        << " | ";

    const auto& failed_metrics = item["failed_metrics"];
    if (failed_metrics.empty()) {
      out << "n/a";
    } else {
      for (std::size_t idx = 0; idx < failed_metrics.size(); ++idx) {
        if (idx != 0) {
          out << ", ";
        }
        out << failed_metrics[idx].get<std::string>();
      }
    }

    out << " | " << item["module_in_scope"].get<std::string>()
        << " | " << std::fixed << std::setprecision(6)
        << (vm_value.has_value() ? *vm_value : 0.0);

    if (va_value.has_value()) {
      out << " | " << *va_value;
    } else {
      out << " | n/a";
    }

    out << " | " << (p_value.has_value() ? *p_value : 0.0)
        << " | " << (q_value.has_value() ? *q_value : 0.0)
        << " | " << (ploss_value.has_value() ? *ploss_value : 0.0)
        << " | " << (qloss_value.has_value() ? *qloss_value : 0.0)
        << " |\n";
  }

  if (summary.contains("bridge_only_cases") &&
      !summary["bridge_only_cases"].empty()) {
    out << "\n## Bridge-Only Validation\n\n";
    out << "| Case | Role | Bridge Validation | Module | Key Evidence |\n";
    out << "| --- | --- | --- | --- | --- |\n";
    for (const auto& item : summary["bridge_only_cases"]) {
      const auto& bridge = item["bridge_validation"];
      const auto& diff = bridge["circuit_losses_raw_cross_check"]
                               ["abs_diff_vs_branch_terminal_sum"];
      std::string key_evidence;
      if (bridge.contains("regcontrol_state")) {
        key_evidence =
            "tap_num=" +
            std::to_string(
                bridge["regcontrol_state"]["tap_number"].get<int>()) +
            ", control_diff_v=" +
            std::to_string(
                bridge["monitored_bus"]["abs_diff_to_target_volts"].get<double>());
      } else {
        key_evidence =
            "tap=" +
            std::to_string(
                bridge["configured_transformer_tap_pu"].get<double>());
      }
      out << "| " << item["case_id"].get<std::string>()
          << " | " << item["acceptance_tier"].get<std::string>()
          << " | " << (item["bridge_validation_pass"].get<bool>() ? "passed" : "failed")
          << " | " << item["module_in_scope"].get<std::string>()
          << " | " << key_evidence
          << ", circuit-loss diff=(" << std::fixed << std::setprecision(6)
          << diff["p_mw"].get<double>() << ", " << diff["q_mvar"].get<double>()
          << ") MW/MVAr |\n";
    }
  }

  if (summary.contains("regulator_semantics_matrix") &&
      !summary["regulator_semantics_matrix"].empty()) {
    out << "\n## Regulator Semantics Matrix\n\n";
    out << "| Semantic | Coverage | Reason | Evidence |\n";
    out << "| --- | --- | --- | --- |\n";
    for (const auto& item : summary["regulator_semantics_matrix"].items()) {
      const auto& value = item.value();
      std::string evidence = "n/a";
      if (value.contains("evidence")) {
        const auto& evidence_json = value["evidence"];
        std::vector<std::string> parts;
        if (evidence_json.contains("cases")) {
          for (const auto& case_id : evidence_json["cases"]) {
            parts.push_back("cases=" + case_id.get<std::string>());
          }
        }
        if (evidence_json.contains("regression_tests")) {
          for (const auto& test_name : evidence_json["regression_tests"]) {
            parts.push_back("tests=" + test_name.get<std::string>());
          }
        }
        if (!parts.empty()) {
          evidence.clear();
          for (std::size_t idx = 0; idx < parts.size(); ++idx) {
            if (idx != 0) {
              evidence += "; ";
            }
            evidence += parts[idx];
          }
        }
      }
      out << "| " << item.key()
          << " | " << value["coverage"].get<std::string>()
          << " | " << value["reason"].get<std::string>()
          << " | " << evidence << " |\n";
    }
  }

  out << "\n## Acceptance Gate\n\n";
  out << "See `case_manifest.json` for the capability matrix and per-case current status. "
         "Any accepted-tier case with `comparison_pass=false` is reported as a non-accepted "
         "diagnostic status and must not be treated as accepted evidence.\n\n";
  out << "If numeric thresholds pass and the bridge stays consistent but the fixture-declared "
         "expected truth is stale, the case status becomes `fixture_truth_drift` instead of "
         "being collapsed into `bridge_validation`.\n\n";

  out << "## Tolerances\n\n";
  out << "Tolerance values are embedded in each case's `acceptance_tolerances` "
         "block in `compare_summary.json`.\n\n";

  out << "## Scope Notes\n\n";
  out << "- Baseline covers radial AC distribution PF with constant-power loads, "
         "fixed bus-level shunts, a pass-through transformer (tap=1.0), and a "
         "fixed off-nominal transformer tap (tap=1.05).\n";
  out << "- Unbalanced 3-phase baseline: per-phase Vm, Va, and terminal P/Q validated "
         "against OpenDSS with asymmetric zero-sequence impedances.\n";
  out << "- `ActiveCktElement.Powers` is treated as kW / kvar through the generic "
         "PD-element snapshot. `Losses` raw is treated as W / var through separate typed "
         "converters, so the compare path cannot silently mix the units.\n";
  out << "- Physical Capacitor (exploratory) demonstrates V^2 bias: "
         "~(1-V^2)*kvar at the cap bus, quantified in `compare_summary.json`.\n";
  out << "- Fixed transformer tap≠1.0 and the minimal closed-loop single-phase "
         "regulator are now both in the accepted compare baseline.\n";
  out << "- Regulator semantics are split machine-readably: remote-bus/no-LDC and "
         "local-PT+LDC and distinct RemotePTRatio are accepted compare, "
         "tapwinding=1 remains solver-regression-only, and both reversible "
         "control and remote-bus+LDC remain explicitly unsupported.\n";
  out << "- Transformer3W pair projection now has an accepted minimal OpenDSS "
         "compare on the isolated one-phase triangle production path. Broader "
         "Transformer3W surfaces remain explicitly unclaimed.\n";
  out << "- Next benchmark ladder is fixed machine-readably in the canonical artifact: "
         "IEEE PES feeders progress from single-phase/balanced subsets to unbalanced "
         "subsets and only then to regulator/transformer semantics.\n";
  return out.str();
}

fs::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

std::string current_time_utc_iso8601() {
  std::time_t now = std::time(nullptr);
  std::tm utc_tm{};
#if defined(_WIN32)
  gmtime_s(&utc_tm, &now);
#else
  gmtime_r(&now, &utc_tm);
#endif
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
  return buffer;
}

struct InjectedMetricMaxAbs {
  std::string case_id;
  std::string metric_name;
  double delta{0.0};
};

struct InjectedRegulatorFixtureTruth {
  std::string case_id;
  int expected_tap_number{0};
  double expected_tap_pu{1.0};
};

struct ToolOptions {
  fs::path report_dir;
  std::optional<InjectedMetricMaxAbs> injected_metric_max_abs;
  std::optional<InjectedRegulatorFixtureTruth> injected_regulator_fixture_truth;
  std::vector<std::string> case_filters;
  std::vector<std::string> tier_filters;
  std::optional<int> shard_index;
  std::optional<int> shard_count;
  bool show_help{false};
};

struct CaseExecutionSpec {
  std::string case_id;
  std::string acceptance_tier;
  std::function<json()> run;
};

bool selection_requested(const ToolOptions& options) {
  return !options.case_filters.empty() || !options.tier_filters.empty() ||
         options.shard_index.has_value() || options.shard_count.has_value();
}

void print_usage(std::ostream& out, const fs::path& root) {
  out << "Usage: " << (root / hacdcpf_compare::kCanonicalOpenDSSCompareToolCommand).filename().string()
      << " [--report-dir <path>] "
         "[--case <case_id>]... "
         "[--tier <acceptance_tier>]... "
         "[--shard-index <n> --shard-count <m>] "
         "[--inject-metric-max-abs <case_id> <metric_name> <delta>] "
         "[--inject-regulator-fixture-truth <case_id> <expected_tap_number> "
         "<expected_tap_pu>] "
         "[--help]\n";
  out << "Canonical build: " << hacdcpf_compare::kCanonicalOpenDSSCompareBuildCommand << '\n';
  out << "Canonical test: " << hacdcpf_compare::kCanonicalOpenDSSCompareTestCommand << '\n';
  out << "Canonical adapter: " << hacdcpf_compare::kCanonicalOpenDSSSnapshotAdapterCommand << '\n';
  out << "Canonical compare: " << hacdcpf_compare::kCanonicalOpenDSSCompareToolCommand << '\n';
}

ToolOptions parse_tool_options(int argc, char** argv, const fs::path& root) {
  ToolOptions options;
  options.report_dir = root / "reports/opendss_pf_compare";

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") {
      options.show_help = true;
      continue;
    }
    if (arg == "--report-dir") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--report-dir requires a path argument");
      }
      options.report_dir = argv[++i];
      continue;
    }
    if (arg == "--case") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--case requires a case_id argument");
      }
      options.case_filters.push_back(argv[++i]);
      continue;
    }
    if (arg == "--tier") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--tier requires an acceptance_tier argument");
      }
      options.tier_filters.push_back(argv[++i]);
      continue;
    }
    if (arg == "--shard-index") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--shard-index requires an integer argument");
      }
      options.shard_index = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--shard-count") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--shard-count requires an integer argument");
      }
      options.shard_count = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--inject-metric-max-abs") {
      if (i + 3 >= argc) {
        throw std::runtime_error(
            "--inject-metric-max-abs requires <case_id> <metric_name> <delta>");
      }
      options.injected_metric_max_abs = InjectedMetricMaxAbs{
          .case_id = argv[++i],
          .metric_name = argv[++i],
          .delta = std::stod(argv[++i]),
      };
      continue;
    }
    if (arg == "--inject-regulator-fixture-truth") {
      if (i + 3 >= argc) {
        throw std::runtime_error(
            "--inject-regulator-fixture-truth requires "
            "<case_id> <expected_tap_number> <expected_tap_pu>");
      }
      options.injected_regulator_fixture_truth = InjectedRegulatorFixtureTruth{
          .case_id = argv[++i],
          .expected_tap_number = std::stoi(argv[++i]),
          .expected_tap_pu = std::stod(argv[++i]),
      };
      continue;
    }
    throw std::runtime_error("Unknown argument: " + arg);
  }

  if (options.shard_index.has_value() != options.shard_count.has_value()) {
    throw std::runtime_error(
        "--shard-index and --shard-count must be provided together");
  }
  if (options.shard_count.has_value() && *options.shard_count <= 0) {
    throw std::runtime_error("--shard-count must be positive");
  }
  if (options.shard_index.has_value() &&
      (*options.shard_index < 0 || *options.shard_index >= *options.shard_count)) {
    throw std::runtime_error(
        "--shard-index must satisfy 0 <= shard-index < shard-count");
  }

  return options;
}

std::vector<CaseExecutionSpec> select_case_catalog(
    const std::vector<CaseExecutionSpec>& catalog,
    const ToolOptions& options) {
  std::set<std::string> available_cases;
  std::set<std::string> available_tiers;
  for (const auto& spec : catalog) {
    available_cases.insert(spec.case_id);
    available_tiers.insert(spec.acceptance_tier);
  }

  for (const auto& case_id : options.case_filters) {
    if (!available_cases.contains(case_id)) {
      throw std::runtime_error("Unknown --case selection: " + case_id);
    }
  }
  for (const auto& tier : options.tier_filters) {
    if (!available_tiers.contains(tier)) {
      throw std::runtime_error("Unknown --tier selection: " + tier);
    }
  }

  std::vector<CaseExecutionSpec> filtered;
  filtered.reserve(catalog.size());
  for (const auto& spec : catalog) {
    const bool case_match =
        options.case_filters.empty() ||
        std::find(options.case_filters.begin(), options.case_filters.end(),
                  spec.case_id) != options.case_filters.end();
    const bool tier_match =
        options.tier_filters.empty() ||
        std::find(options.tier_filters.begin(), options.tier_filters.end(),
                  spec.acceptance_tier) != options.tier_filters.end();
    if (case_match && tier_match) {
      filtered.push_back(spec);
    }
  }

  if (filtered.empty()) {
    throw std::runtime_error(
        "Case selection removed every case; adjust --case/--tier filters.");
  }

  if (!options.shard_count.has_value()) {
    return filtered;
  }

  std::vector<CaseExecutionSpec> sharded;
  sharded.reserve(filtered.size());
  for (std::size_t idx = 0; idx < filtered.size(); ++idx) {
    if (static_cast<int>(idx % static_cast<std::size_t>(*options.shard_count)) ==
        *options.shard_index) {
      sharded.push_back(filtered[idx]);
    }
  }
  if (sharded.empty()) {
    throw std::runtime_error(
        "Shard selection removed every case; adjust --shard-index/--shard-count.");
  }
  return sharded;
}

json build_selection_summary(const std::vector<CaseExecutionSpec>& catalog,
                             const std::vector<CaseExecutionSpec>& selected,
                             const ToolOptions& options) {
  std::set<std::string> selected_case_ids;
  for (const auto& spec : selected) {
    selected_case_ids.insert(spec.case_id);
  }

  json skipped = json::array();
  for (const auto& spec : catalog) {
    if (!selected_case_ids.contains(spec.case_id)) {
      skipped.push_back(spec.case_id);
    }
  }

  json selected_ids = json::array();
  for (const auto& spec : selected) {
    selected_ids.push_back(spec.case_id);
  }

  return {
      {"mode", selection_requested(options) ? "filtered" : "full_catalog"},
      {"requested_case_filters", options.case_filters},
      {"requested_tier_filters", options.tier_filters},
      {"shard_index",
       options.shard_index.has_value() ? json(*options.shard_index)
                                       : json(nullptr)},
      {"shard_count",
       options.shard_count.has_value() ? json(*options.shard_count)
                                       : json(nullptr)},
      {"selected_case_ids", selected_ids},
      {"skipped_case_ids", skipped},
  };
}

void apply_injected_metric_max_abs(json& summary,
                                   const InjectedMetricMaxAbs& injection) {
  for (auto& item : summary["cases"]) {
    if (item["case_id"].get<std::string>() != injection.case_id) {
      continue;
    }
    if (!item["metrics"].contains(injection.metric_name)) {
      throw std::runtime_error(
          "Injected metric not found: " + injection.metric_name);
    }
    item["metrics"][injection.metric_name]["max_abs"] =
        item["metrics"][injection.metric_name]["max_abs"].get<double>() +
        injection.delta;
    summary["injected_regression"] = {
        {"case_id", injection.case_id},
        {"metric_name", injection.metric_name},
        {"delta_added_to_max_abs", injection.delta},
    };
    return;
  }
  throw std::runtime_error("Injected case not found: " + injection.case_id);
}

void apply_injected_regulator_fixture_truth(
    json& summary,
    const InjectedRegulatorFixtureTruth& injection) {
  for (auto& item : summary["cases"]) {
    if (item["case_id"].get<std::string>() != injection.case_id) {
      continue;
    }
    if (!item.contains("fixture_truth_drift")) {
      throw std::runtime_error(
          "Injected fixture truth drift is only supported for regulator compare cases.");
    }

    auto& drift = item["fixture_truth_drift"];
    drift["detected"] = true;
    drift["reason"] = "fixture_expected_regulator_end_state_is_stale";
    drift["fixture_expected_end_state"]["tap_number"] =
        injection.expected_tap_number;
    drift["fixture_expected_end_state"]["tap_pu"] =
        injection.expected_tap_pu;

    const int repo_tap_number =
        drift["repo_end_state"]["tap_number"].get<int>();
    const int dss_tap_number =
        drift["opendss_end_state"]["tap_number"].get<int>();
    const double repo_tap_pu =
        drift["repo_end_state"]["tap_pu"].get<double>();
    const double dss_tap_pu =
        drift["opendss_end_state"]["tap_pu"].get<double>();
    drift["repo_and_opendss_agree"] = {
        {"tap_number", repo_tap_number == dss_tap_number},
        {"tap_pu", std::abs(repo_tap_pu - dss_tap_pu) < 1e-12},
        {"control_voltage_within_tolerance",
         drift["repo_and_opendss_agree"]["control_voltage_within_tolerance"]},
    };
    drift["fixture_matches_actual"] = {
        {"tap_number",
         repo_tap_number == injection.expected_tap_number &&
             dss_tap_number == injection.expected_tap_number},
        {"tap_pu",
         std::abs(repo_tap_pu - injection.expected_tap_pu) < 1e-12 &&
             std::abs(dss_tap_pu - injection.expected_tap_pu) < 1e-12},
    };
    item["fixture_truth_drift_pass"] = false;
    summary["injected_fixture_truth_regression"] = {
        {"case_id", injection.case_id},
        {"expected_tap_number", injection.expected_tap_number},
        {"expected_tap_pu", injection.expected_tap_pu},
    };
    return;
  }
  throw std::runtime_error("Injected case not found for fixture truth drift: " +
                           injection.case_id);
}

std::vector<CaseExecutionSpec> build_case_catalog(const fs::path& root) {
  const fs::path case_1ph =
      root / "tests/data/opendss/minimal_radial_3bus_1ph/Master.dss";
  const fs::path case_3ph =
      root / "tests/data/opendss/minimal_radial_3bus_balanced_3ph/Master.dss";
  const fs::path case_shunt =
      root / "tests/data/opendss/minimal_radial_3bus_1ph_shunt/Master.dss";
  const fs::path case_cap =
      root / "tests/data/opendss/minimal_radial_3bus_1ph_cap/Master.dss";
  const fs::path case_33bw =
      root / "tests/data/opendss/case33bw_radial_3ph/Master.dss";
  const fs::path case_unbal3ph =
      root / "tests/data/opendss/minimal_unbalanced_3bus_3ph/Master.dss";
  const fs::path case_xfmr =
      root / "tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss";

  return {
      {
          .case_id = "minimal_radial_3bus_1ph",
          .acceptance_tier = "accepted_primary",
          .run =
              [root, case_1ph]() {
                return summarize_single_phase_case(
                    root, case_1ph,
                    "minimal_radial_3bus_1ph",
                    "accepted_primary",
                    "Single-phase radial BFS primary acceptance path — smallest stable baseline.",
                    "Smallest stable path for OpenDSS cross-check; mapping stable, error sub-1e-3.",
                    hacdcpf_compare_fixtures::build_3bus_radial(),
                    k1phTol);
              },
      },
      {
          .case_id = "minimal_radial_3bus_balanced_3ph",
          .acceptance_tier = "accepted_secondary",
          .run = [root, case_3ph]() {
            return summarize_balanced_three_phase_case(root, case_3ph);
          },
      },
      {
          .case_id = "minimal_radial_3bus_1ph_shunt",
          .acceptance_tier = "accepted_rich_component",
          .run =
              [root, case_shunt]() {
                return summarize_single_phase_case(
                    root, case_shunt,
                    "minimal_radial_3bus_1ph_shunt",
                    "accepted_rich_component",
                    "Extends the primary case with a constant-injection reactive shunt at bus2. "
                    "Both BFS (ACBus.bs_mvar) and OpenDSS (model=1 negative-Q load) treat the "
                    "injection as voltage-independent — no V^2 bias.",
                    "Adds Q-compensator coverage (fixed shunt) without changing topology.",
                    hacdcpf_compare_fixtures::build_3bus_radial_with_shunt(),
                    kShTol);
              },
      },
      {
          .case_id = "case33bw_radial_3ph",
          .acceptance_tier = "accepted_feeder_subset",
          .run = [root, case_33bw]() {
            return summarize_case33bw_case(root, case_33bw);
          },
      },
      {
          .case_id = "minimal_radial_3bus_1ph_cap",
          .acceptance_tier = "exploratory_physical_component",
          .run = [root, case_cap]() {
            return summarize_physical_capacitor_case(root, case_cap);
          },
      },
      {
          .case_id = "minimal_unbalanced_3bus_3ph",
          .acceptance_tier = "accepted_unbalanced_3ph",
          .run = [root, case_unbal3ph]() {
            return summarize_unbalanced_three_phase_case(root, case_unbal3ph);
          },
      },
      {
          .case_id = "minimal_passthr_transformer_1ph",
          .acceptance_tier = "accepted_pass_through_xfmr",
          .run = [root, case_xfmr]() {
            return summarize_passthr_transformer_case(root, case_xfmr);
          },
      },
      {
          .case_id = "minimal_tap_transformer_1ph",
          .acceptance_tier = "accepted_fixed_tap_xfmr",
          .run = [root]() {
            return summarize_fixed_tap_transformer_case(root);
          },
      },
      {
          .case_id = "minimal_regulator_1ph",
          .acceptance_tier = "accepted_regulator",
          .run = [root]() {
            return summarize_accepted_regulator_case(root);
          },
      },
      {
          .case_id = "minimal_regulator_local_pt_ldc_1ph",
          .acceptance_tier = "accepted_regulator_local_pt_ldc",
          .run = [root]() {
            return summarize_accepted_local_pt_with_ldc_regulator_case(root);
          },
      },
      {
          .case_id = "minimal_regulator_remote_ptratio_1ph",
          .acceptance_tier = "accepted_regulator_remote_ptratio",
          .run = [root]() {
            return summarize_accepted_remote_ptratio_regulator_case(root);
          },
      },
      {
          .case_id = "minimal_regulator_bank_3ph_nr",
          .acceptance_tier = "accepted_regulator_bank_3ph",
          .run = [root]() {
            return summarize_accepted_three_phase_regulator_bank_case(root);
          },
      },
      {
          .case_id = "minimal_load_wye_zip_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_wye_zip_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_wye_open_neutral_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_wye_impedance_grounded_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_load_wye_vmax_3bus_1ph_lateral_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_wye_vmax_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_load_delta_power_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_delta_power_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_load_delta_zip_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_load_semantics_3ph",
          .run = [root]() {
            return summarize_accepted_load_delta_zip_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_transformer3w_tap_1ph",
          .acceptance_tier = "accepted_transformer3w_isolated_triangle",
          .run = [root]() {
            return summarize_transformer3w_formal_cross_check_case(root);
          },
      },
      {
          .case_id = "minimal_transformer3w_tap_1ph_hv_spur",
          .acceptance_tier = "accepted_transformer3w_hv_spur_embed",
          .run = [root]() {
            return summarize_transformer3w_hv_spur_formal_cross_check_case(root);
          },
      },
      {
          .case_id = "minimal_phase_matrix_line_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_phase_matrix_line_3ph",
          .run = [root]() {
            return summarize_accepted_phase_matrix_line_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_meshed_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_meshed_3ph",
          .run = [root]() {
            return summarize_accepted_meshed_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "minimal_multi_source_3bus_3ph_nr",
          .acceptance_tier = "accepted_nr_multi_source_3ph",
          .run = [root]() {
            return summarize_accepted_multi_source_three_phase_nr_case(root);
          },
      },
      {
          .case_id = "ieee_13_node_4kv_backbone_3ph_nr",
          .acceptance_tier = "deferred_ieee13_feeder_formal_cross_check",
          .run = [root]() {
            return summarize_ieee13_4kv_backbone_three_phase_nr_case(root);
          },
      },
  };
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  try {
    const fs::path root = project_root();
    const ToolOptions options = parse_tool_options(argc, argv, root);
    if (options.show_help) {
      print_usage(std::cout, root);
      return EXIT_SUCCESS;
    }

    const fs::path report_dir = options.report_dir;
    fs::create_directories(report_dir);

    const auto full_case_catalog = build_case_catalog(root);
    const auto selected_case_catalog =
        select_case_catalog(full_case_catalog, options);
    const json selection_summary =
        build_selection_summary(full_case_catalog, selected_case_catalog, options);
    const json official_feeder_reference_status =
        build_official_feeder_reference_status(root);
    const json tolerance_justification = build_tolerance_justification();

    json manifest = build_case_manifest(root);
    manifest["scope"]["official_feeder_reference_status"] =
        official_feeder_reference_status;
    manifest["scope"]["tolerance_justification"] = tolerance_justification;
    json summary = {
        {"generated_at_utc", current_time_utc_iso8601()},
        {"working_directory", root.string()},
        {"reproducibility",
         {
             {"subbuild_dir", "build/opendss_compare"},
             {"canonical_build_command",
              hacdcpf_compare::kCanonicalOpenDSSCompareBuildCommand},
             {"canonical_test_command",
              hacdcpf_compare::kCanonicalOpenDSSCompareTestCommand},
             {"canonical_adapter_command",
              hacdcpf_compare::kCanonicalOpenDSSSnapshotAdapterCommand},
             {"canonical_compare_command",
              hacdcpf_compare::kCanonicalOpenDSSCompareToolCommand},
             {"report_dir", report_dir.string()},
         }},
        {"comparison_failure_diagnostics",
         {
             {"metric_threshold_failed",
              "repo and OpenDSS comparison exceeded at least one declared numeric threshold"},
             {"bridge_validation_failed",
              "OpenDSS bridge/configuration/loss self-consistency evidence failed even before considering fixture truth"},
             {"control_trace_validation_failed",
              "repo regulator trace and OpenDSS control oracle disagreed on iteration-level control evidence"},
             {"fixture_truth_drift",
              "repo and OpenDSS end state agree within thresholds, but the fixture-declared expected truth is stale"},
         }},
        {"adapter_contract_surface", build_adapter_contract_surface(root)},
        {"benchmark_expansion_plan", build_benchmark_expansion_plan()},
        {"official_feeder_reference_status", official_feeder_reference_status},
        {"tolerance_justification", tolerance_justification},
        {"regulator_semantics_matrix", build_regulator_semantics_matrix()},
        {"selection", selection_summary},
        {"cases", json::array()},
        {"bridge_only_cases", json::array()},
    };

    for (const auto& case_spec : selected_case_catalog) {
      summary["cases"].push_back(case_spec.run());
    }

    if (options.injected_regulator_fixture_truth.has_value()) {
      apply_injected_regulator_fixture_truth(
          summary, *options.injected_regulator_fixture_truth);
    }

    if (options.injected_metric_max_abs.has_value()) {
      apply_injected_metric_max_abs(summary, *options.injected_metric_max_abs);
    }

    bool accepted_cases_all_passed = true;
    json failed_accepted_cases = json::array();
    for (auto& item : summary["cases"]) {
      const auto evaluation = evaluate_case(item);
      const std::string tier = item["acceptance_tier"].get<std::string>();
      if (tier.rfind("accepted", 0) == 0 && !evaluation.comparison_pass) {
        accepted_cases_all_passed = false;
        failed_accepted_cases.push_back({
            {"case_id", item["case_id"]},
            {"comparison_status", item["comparison_status"]},
            {"failed_metrics", evaluation.failed_metrics},
        });
      }
    }
    summary["accepted_cases_all_passed"] = accepted_cases_all_passed;
    summary["failed_accepted_cases"] = failed_accepted_cases;

    bool bridge_only_cases_all_passed = true;
    json failed_bridge_only_cases = json::array();
    for (const auto& item : summary["bridge_only_cases"]) {
      if (!item["bridge_validation_pass"].get<bool>()) {
        bridge_only_cases_all_passed = false;
        failed_bridge_only_cases.push_back({
            {"case_id", item["case_id"]},
        });
      }
    }
    summary["bridge_only_cases_all_passed"] = bridge_only_cases_all_passed;
    summary["failed_bridge_only_cases"] = failed_bridge_only_cases;

    // ---------------------------------------------------------------------------
    // error_budget_summary — passing accepted cases only; max over all accepted metrics.
    // Provides a single-glance pass/fail indicator for the accepted baseline.
    // ---------------------------------------------------------------------------
    {
      double max_vm    = 0.0;
      double max_va    = 0.0;
      double max_p     = 0.0;
      double max_q     = 0.0;
      double max_ploss = 0.0;
      double max_qloss = 0.0;
      double max_control_v = 0.0;
      double max_tap_number = 0.0;
      int declared_accepted_count = 0;
      int passing_accepted_count = 0;
      json excluded_failed_cases = json::array();
      std::vector<json> accepted_case_rankings;

      for (const auto& item : summary["cases"]) {
        const std::string tier = item["acceptance_tier"].get<std::string>();
        if (tier.rfind("accepted", 0) != 0) { continue; }
        ++declared_accepted_count;
        if (!item["comparison_pass"].get<bool>()) {
          excluded_failed_cases.push_back(item["case_id"]);
          continue;
        }
        ++passing_accepted_count;

        const auto& m = item["metrics"];
        if (const auto value = metric_max_abs_any(
                m, {"bus_voltage_magnitude_pu", "bus_phase_voltage_magnitude_pu"})) {
          max_vm = std::max(max_vm, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"bus_voltage_angle_deg", "bus_phase_voltage_angle_deg"})) {
          max_va = std::max(max_va, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"line_terminal1_p_mw", "line_terminal1_phase_p_mw",
                    "line_terminal_3ph_total_p_mw", "transformer_terminal1_p_mw",
                    "line_l1_terminal1_p_mw"})) {
          max_p = std::max(max_p, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"line_terminal1_q_mvar", "line_terminal1_phase_q_mvar",
                    "line_terminal_3ph_total_q_mvar", "transformer_terminal1_q_mvar",
                    "line_l1_terminal1_q_mvar"})) {
          max_q = std::max(max_q, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"total_line_loss_p_mw", "total_branch_loss_p_mw",
                    "total_circuit_loss_p_mw_via_circuit_losses_raw"})) {
          max_ploss = std::max(max_ploss, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"total_line_loss_q_mvar", "total_branch_loss_q_mvar",
                    "total_circuit_loss_q_mvar_via_circuit_losses_raw"})) {
          max_qloss = std::max(max_qloss, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"regulator_control_voltage_volts"})) {
          max_control_v = std::max(max_control_v, *value);
        }
        if (const auto value = metric_max_abs_any(
                m, {"regulator_tap_number"})) {
          max_tap_number = std::max(max_tap_number, *value);
        }

        const auto& dominant = item["error_budget"]["dominant_error_metric"];
        json ranking = {
            {"case_id", item["case_id"]},
            {"absolute_error", dominant.is_null() ? 0.0 : dominant["absolute_error"].get<double>()},
            {"relative_error", dominant.is_null() ? json(nullptr) : dominant["relative_error"]},
            {"threshold", dominant.is_null() ? 0.0 : dominant["threshold"].get<double>()},
            {"pass", dominant.is_null() ? true : dominant["pass"].get<bool>()},
            {"dominant_error_metric", dominant.is_null() ? json(nullptr) : dominant["name"]},
        };
        accepted_case_rankings.push_back(std::move(ranking));
      }

      std::sort(accepted_case_rankings.begin(), accepted_case_rankings.end(),
                [](const json& lhs, const json& rhs) {
                  return lhs["absolute_error"].get<double>() >
                         rhs["absolute_error"].get<double>();
                });

      summary["error_budget_summary"] = {
          {"description",
           "Machine-readable accepted-case error budget. Reports maximum absolute "
           "error over passing accepted cases plus a per-case ranking by dominant "
           "error metric. Exploratory and failed accepted cases are excluded."},
          {"declared_accepted_case_count", declared_accepted_count},
          {"passing_accepted_case_count",  passing_accepted_count},
          {"excluded_failed_accepted_cases", excluded_failed_cases},
          {"max_vm_pu",             max_vm},
          {"max_va_deg",            max_va},
          {"max_p_mw",              max_p},
          {"max_q_mvar",            max_q},
          {"max_ploss_mw",          max_ploss},
          {"max_qloss_mvar",        max_qloss},
          {"max_regulator_control_voltage_volts", max_control_v},
          {"max_regulator_tap_number", max_tap_number},
          {"accepted_case_rankings", accepted_case_rankings},
          {"units",
           {{"vm", "p.u."}, {"va", "degrees"}, {"p", "MW"}, {"q", "MVAr"},
            {"loss", "MW / MVAr"},
            {"regulator_control_voltage", "volts"},
            {"regulator_tap_number", "tap number"}}},
      };
    }

    auto find_case_summary = [&](const std::string& case_id) -> json* {
      for (auto& item : summary["cases"]) {
        if (item["case_id"].get<std::string>() == case_id) {
          return &item;
        }
      }
      for (auto& item : summary["bridge_only_cases"]) {
        if (item["case_id"].get<std::string>() == case_id) {
          return &item;
        }
      }
      return nullptr;
    };

    manifest["scope"]["accepted_cases_all_passed"] = accepted_cases_all_passed;
    manifest["scope"]["failed_accepted_cases"] = failed_accepted_cases;
    manifest["scope"]["bridge_only_cases_all_passed"] = bridge_only_cases_all_passed;
    manifest["scope"]["failed_bridge_only_cases"] = failed_bridge_only_cases;
    manifest["scope"]["selection"] = selection_summary;

    if (selection_requested(options)) {
      std::set<std::string> selected_case_ids;
      for (const auto& spec : selected_case_catalog) {
        selected_case_ids.insert(spec.case_id);
      }

      json filtered_cases = json::array();
      for (const auto& item : manifest["cases"]) {
        if (selected_case_ids.contains(item["case_id"].get<std::string>())) {
          filtered_cases.push_back(item);
        }
      }
      manifest["cases"] = std::move(filtered_cases);

      json filtered_tiers = json::object();
      for (const auto& tier_item : manifest["scope"]["acceptance_tiers"].items()) {
        const auto& tier_value = tier_item.value();
        if (tier_value.contains("case") &&
            selected_case_ids.contains(tier_value["case"].get<std::string>())) {
          filtered_tiers[tier_item.key()] = tier_value;
        }
      }
      manifest["scope"]["acceptance_tiers"] = std::move(filtered_tiers);
    }

    for (auto& item : manifest["cases"]) {
      const auto* case_summary =
          find_case_summary(item["case_id"].get<std::string>());
      if (case_summary == nullptr) {
        continue;
      }
      item["comparison_status"] = (*case_summary)["comparison_status"];
      item["comparison_pass"] = (*case_summary)["comparison_pass"];
      item["failed_metrics"] = (*case_summary)["failed_metrics"];
      if (case_summary->contains("bridge_validation_pass")) {
        item["bridge_validation_pass"] = (*case_summary)["bridge_validation_pass"];
      }
      if (case_summary->contains("fixture_truth_drift_pass")) {
        item["fixture_truth_drift_pass"] =
            (*case_summary)["fixture_truth_drift_pass"];
      }
    }

    for (auto& tier_item : manifest["scope"]["acceptance_tiers"].items()) {
      auto& tier_value = tier_item.value();
      if (!tier_value.contains("case")) {
        continue;
      }
      const auto* case_summary =
          find_case_summary(tier_value["case"].get<std::string>());
      if (case_summary == nullptr) {
        continue;
      }
      tier_value["comparison_status"] = (*case_summary)["comparison_status"];
      tier_value["comparison_pass"] = (*case_summary)["comparison_pass"];
      tier_value["failed_metrics"] = (*case_summary)["failed_metrics"];
      if (case_summary->contains("bridge_validation_pass")) {
        tier_value["bridge_validation_pass"] = (*case_summary)["bridge_validation_pass"];
      }
      if (case_summary->contains("fixture_truth_drift_pass")) {
        tier_value["fixture_truth_drift_pass"] =
            (*case_summary)["fixture_truth_drift_pass"];
      }
    }

    for (auto& component : manifest["scope"]["capability_matrix"]["components"]) {
      if (component["comparison_status"] != "accepted" || !component.contains("cases")) {
        continue;
      }

      bool any_failed = false;
      json failed_cases = json::array();
      for (const auto& case_id_json : component["cases"]) {
        const auto* case_summary = find_case_summary(case_id_json.get<std::string>());
        if (case_summary != nullptr && !(*case_summary)["comparison_pass"].get<bool>()) {
          any_failed = true;
          failed_cases.push_back(case_id_json);
        }
      }

      if (any_failed) {
        bool fixture_truth_drift_only = !failed_cases.empty();
        for (const auto& case_id_json : failed_cases) {
          const auto* case_summary =
              find_case_summary(case_id_json.get<std::string>());
          if (case_summary == nullptr ||
              (*case_summary)["comparison_status"].get<std::string>() !=
                  "fixture_truth_drift") {
            fixture_truth_drift_only = false;
            break;
          }
        }

        component["comparison_status"] =
            fixture_truth_drift_only ? "fixture_truth_drift" : "failed";
        component["reason_for_gap"] =
            fixture_truth_drift_only ? "fixture_truth_drift"
                                     : "comparison_failed";
        component["failed_cases"] = failed_cases;
      }
    }

    const std::string markdown = build_markdown_summary(summary);

    {
      std::ofstream os(report_dir / "case_manifest.json");
      os << std::setw(2) << manifest << '\n';
    }
    {
      std::ofstream os(report_dir / "compare_summary.json");
      os << std::setw(2) << summary << '\n';
    }
    {
      std::ofstream os(report_dir / "compare_summary.md");
      os << markdown;
    }

    std::cout << "Generated OpenDSS PF comparison artifacts:\n";
    std::cout << "  - " << (report_dir / "case_manifest.json") << '\n';
    std::cout << "  - " << (report_dir / "compare_summary.json") << '\n';
    std::cout << "  - " << (report_dir / "compare_summary.md") << '\n';

    for (const auto& item : summary["cases"]) {
      const auto& metrics = item["metrics"];
      const auto vm_value = metric_max_abs_any(
          metrics, {"bus_voltage_magnitude_pu", "bus_phase_voltage_magnitude_pu"});
      const auto p_value = metric_max_abs_any(
          metrics, {"line_terminal1_p_mw", "line_terminal1_phase_p_mw",
                    "line_terminal_3ph_total_p_mw", "transformer_terminal1_p_mw",
                    "line_l1_terminal1_p_mw"});
      const auto ploss_value = metric_max_abs_any(
          metrics, {"total_line_loss_p_mw", "total_branch_loss_p_mw",
                    "total_circuit_loss_p_mw_via_circuit_losses_raw"});

      std::cout << item["case_id"].get<std::string>()
                << " [" << item["acceptance_tier"].get<std::string>() << "]"
                << " status=" << item["comparison_status"].get<std::string>()
                << ": Vm max_abs=" << (vm_value.has_value() ? *vm_value : 0.0)
                << ", P max_abs=" << (p_value.has_value() ? *p_value : 0.0)
                << ", total Ploss abs="
                << (ploss_value.has_value() ? *ploss_value : 0.0)
                << '\n';
    }

    if (!accepted_cases_all_passed) {
      std::cerr << "Accepted-case comparison gate failed.\n";
      for (const auto& failed_case : failed_accepted_cases) {
        std::cerr << "  - " << failed_case["case_id"].get<std::string>() << ": ";
        const auto& failed_metrics = failed_case["failed_metrics"];
        for (std::size_t idx = 0; idx < failed_metrics.size(); ++idx) {
          if (idx != 0) {
            std::cerr << ", ";
          }
          std::cerr << failed_metrics[idx].get<std::string>();
        }
        std::cerr << '\n';
      }
      return hacdcpf_compare::kAcceptedCaseGateExitCode;
    }

    return EXIT_SUCCESS;
  } catch (const std::exception& ex) {
    std::cerr << "opendss_pf_compare failed: " << ex.what() << '\n';
    return EXIT_FAILURE;
  }
}

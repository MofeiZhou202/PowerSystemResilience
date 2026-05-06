#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace hacdcpf::io {

enum class OpenDSSPDElementKind {
  Line,
  Transformer,
};

struct OpenDSSNodeVoltage {
  std::string bus_name;
  int node{0};
  double vm_pu{0.0};
  double va_deg{0.0};
  double vm_vln{0.0};
};

struct OpenDSSPowerKWKvar {
  double p_kw{0.0};
  double q_kvar{0.0};
};

struct OpenDSSLossesWVar {
  double p_w{0.0};
  double q_var{0.0};
};

struct OpenDSSTerminalPower {
  int terminal{0};
  int conductor{0};
  int node{0};
  OpenDSSPowerKWKvar power_kw_kvar;
};

struct OpenDSSPDElementResult {
  OpenDSSPDElementKind element_kind{OpenDSSPDElementKind::Line};
  std::string element_name;
  std::string name;
  std::vector<std::string> terminal_bus_names;
  std::vector<int> node_order;
  int num_phases{0};
  int num_conductors{0};
  int num_terminals{0};
  std::vector<OpenDSSTerminalPower> terminal_powers;
  // Raw ActiveCktElement.Losses output for this element (W, var).
  OpenDSSLossesWVar losses_raw;
};

struct OpenDSSTransformerWindingState {
  int winding{0};
  double tap_pu{1.0};
  double min_tap_pu{1.0};
  double max_tap_pu{1.0};
  int num_taps{0};
  double kv{0.0};
  double kva{0.0};
};

struct OpenDSSTransformerState {
  std::string name;
  int num_windings{0};
  std::vector<OpenDSSTransformerWindingState> winding_states;
};

struct OpenDSSRegControlResult {
  std::string name;
  std::string transformer_name;
  std::string monitored_bus_name;
  int winding{0};
  int tap_winding{0};
  int tap_number{0};
  int max_tap_change{0};
  double forward_vreg_volts{0.0};
  double forward_band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double forward_r_volts{0.0};
  double forward_x_volts{0.0};
  double voltage_limit_volts{0.0};
  bool is_reversible{false};
};

struct OpenDSSEventLogEntry {
  int event_index{0};
  std::string message;
};

struct OpenDSSControlOracle {
  int control_iterations{0};
  int max_control_iterations{0};
  std::vector<OpenDSSEventLogEntry> event_log_entries;
};

struct OpenDSSLineResult {
  std::string element_name;
  std::string line_name;
  std::vector<std::string> terminal_bus_names;
  std::vector<int> node_order;
  int num_phases{0};
  int num_conductors{0};
  int num_terminals{0};
  std::vector<OpenDSSTerminalPower> terminal_powers;
  OpenDSSLossesWVar losses_raw;
};

struct OpenDSSSnapshotResult {
  std::string engine_version;
  bool converged{false};
  std::vector<OpenDSSNodeVoltage> node_voltages;
  // Generic PD-element snapshot. Supports at least Line and Transformer.
  std::vector<OpenDSSPDElementResult> pd_element_results;
  // Transformer operating state (e.g. solved winding taps) after Solve.
  std::vector<OpenDSSTransformerState> transformer_states;
  // RegControl state (e.g. final tap number and configured control settings).
  std::vector<OpenDSSRegControlResult> regcontrol_results;
  // Global OpenDSS control oracle for the solved snapshot.
  OpenDSSControlOracle control_oracle;
  // Backward-compatible projection of pd_element_results where element_kind=Line.
  std::vector<OpenDSSLineResult> line_results;
  // Raw Circuit.Losses output from OpenDSS C-API (W, var).
  OpenDSSLossesWVar circuit_losses_raw;
};

const char* opendss_pd_element_kind_to_string(OpenDSSPDElementKind kind);

OpenDSSSnapshotResult solve_opendss_snapshot(
    const std::filesystem::path& master_dss);

}  // namespace hacdcpf::io

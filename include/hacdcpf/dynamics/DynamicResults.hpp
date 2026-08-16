#pragma once

#include <complex>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct DynamicModelProfile {
  std::string standard;
  std::string profile;
  std::string model_name;
  std::string parameter_set;
  std::string source_id;
  std::string notes;
};

struct DynamicDeviceOutput {
  std::string name;
  std::string type;
  int component_index{0};
  int bus{0};
  std::string canvas_type;
  int canvas_index{-1};
  std::string component_domain;
  std::string source_type;
  std::string model_standard;
  std::string model_name;
  std::string parameter_set;
  std::vector<DynamicModelProfile> model_profiles;
  std::map<std::string, double> values;
};

struct DynamicAppliedEventRecord {
  double time_s{0.0};
  std::string type;
  std::string label;
  int component_index{0};
  int target_id{0};
  int bus{0};
  int phase{-1};
  double value{0.0};
  double duration_s{0.0};
  std::string component_type;
  std::string target_type;
  std::map<std::string, double> params;
};

struct DynamicResidualDiagnostic {
  std::string device_name;
  std::string device_type;
  int component_index{0};
  int state_index{-1};
  double residual{0.0};
};

// Per-island frequency observability (design doc §7). An "island" is a connected
// component of the live (in-service) AC network; its center-of-inertia (COI)
// frequency is the inertia-weighted average of the rotor/virtual speeds of the
// generation in that island.
struct DynamicIslandFrequency {
  int island_id{0};
  int n_ac_buses{0};
  bool has_anchor{false};          // island contains a machine/GFM/slack
  bool has_source{false};          // island contains any generation-capable device
  double coi_frequency_hz{0.0};    // inertia-weighted center-of-inertia frequency
  double coi_rocof_hz_s{0.0};      // inertia-weighted frequency derivative
  double total_inertia_mws{0.0};   // Sum of H*S over the island [MW*s]
};

struct DynamicInitializationSummary {
  bool power_flow_requested{true};
  bool power_flow_converged{false};
  bool fallback_voltage_setpoints{false};
  int iterations{0};
  double residual{0.0};
  bool dynamic_trim_converged{false};
  int dynamic_trim_iterations{0};
  double dynamic_initial_dxdt_inf_norm{0.0};
  double dynamic_fast_dxdt_inf_norm{0.0};
  double min_ac_voltage_pu{0.0};
  double max_ac_voltage_pu{0.0};
  double min_dc_voltage_pu{0.0};
  double max_dc_voltage_pu{0.0};
  bool gfm_pf_seed_checked{false};
  bool gfm_pf_seed_certified{false};
  int gfm_pf_seed_devices{0};
  double gfm_pf_seed_internal_voltage_error_pu{0.0};
  double gfm_pf_seed_current_error_pu{0.0};
  double gfm_pf_seed_power_error_pu{0.0};
  std::vector<DynamicResidualDiagnostic> dynamic_residual_diagnostics;
  std::vector<std::string> warnings;
};

struct DynamicSnapshot {
  double time_s{0.0};
  Eigen::VectorXd state;
  Eigen::VectorXcd vac_abc;
  Eigen::VectorXd vdc;
  double max_ac_voltage_pu{0.0};
  double min_ac_voltage_pu{0.0};
  double max_dc_voltage_pu{0.0};
  double min_dc_voltage_pu{0.0};
  double frequency_hz{0.0};
  double coi_frequency_hz{0.0};
  double coi_rocof_hz_s{0.0};
  std::vector<DynamicIslandFrequency> island_frequencies;
  // Measured (low-pass filtered) frequency per AC bus, derived from the bus
  // voltage-angle derivative (design doc §7 role 4). This is an output signal for
  // relays / ride-through logic / plotting; it is never fed back into the frame.
  std::vector<double> bus_frequency_hz;
  std::vector<DynamicDeviceOutput> device_outputs;
};

// Normalized participation of one differential state in a mode (design doc §18).
struct DynamicModalParticipation {
  int state_index{0};       // global differential-state index
  std::string state_label;  // "type#component:sLocal"
  double factor{0.0};       // |participation|, normalized per mode (0..1)
};

// One eigenvalue of the Schur-reduced state matrix, in the compact form attached
// to a transient result (design doc §18).
struct DynamicModalMode {
  double eigen_real{0.0};    // Re(lambda), 1/s
  double eigen_imag{0.0};    // Im(lambda), rad/s
  double frequency_hz{0.0};  // |Im| / (2*pi)
  double damping_ratio{0.0}; // -Re / |lambda|
  bool oscillatory{false};
  std::string dominant_state; // label of the highest-participation state
  // Significant state participations for this mode (normalized, descending),
  // truncated to the leading contributors. Empty unless the screen was computed.
  std::vector<DynamicModalParticipation> participation;
};

// Compact small-signal (modal) screen about the initialized operating point,
// populated when DynamicSolverOptions::compute_small_signal is set. The heavy
// artifacts (reduced Jacobian, full participation matrix) are omitted here; call
// small_signal_analysis() directly for those. Modes are least-damped first.
struct DynamicModalSummary {
  bool computed{false};   // analysis was requested
  bool success{false};    // analysis produced eigenvalues
  std::string message;
  int n_differential{0};
  int n_algebraic{0};
  bool stable{false};             // all Re(lambda) < margin
  double min_damping_ratio{0.0};  // damping of the most critical mode
  std::vector<DynamicModalMode> modes;
};

struct DynamicResults {
  bool success{false};
  std::string message;
  int steps{0};
  int failed_step{-1};
  int newton_iterations{0};
  int jacobian_evaluations{0};
  int jacobian_residual_evaluations{0};
  int jacobian_fd_columns{0};
  int jacobian_colored_groups{0};
  int linear_factorizations{0};
  int rejected_steps{0};
  double max_local_error_norm{0.0};
  double min_accepted_step_s{0.0};
  double max_accepted_step_s{0.0};

  std::vector<DynamicSnapshot> snapshots;
  std::vector<std::string> warnings;
  std::vector<std::string> applied_events;
  std::vector<DynamicAppliedEventRecord> applied_event_records;
  DynamicInitializationSummary initialization;
  DynamicModalSummary modal;

  [[nodiscard]] const DynamicSnapshot* final_snapshot() const noexcept {
    return snapshots.empty() ? nullptr : &snapshots.back();
  }
};

struct DynamicResultExportOptions {
  bool include_ac_voltage_magnitudes{true};
  bool include_dc_voltages{true};
  bool include_device_outputs{true};
  // Prepend the small-signal (modal) screen as a '#'-commented metadata block
  // (eigenvalues, damping, dominant state, participation) when the result carries
  // one. The comment prefix keeps the time-series table parseable as plain CSV.
  bool include_modal{true};
  char delimiter{','};
};

std::string to_csv(const DynamicResults& results,
                   const DynamicResultExportOptions& options = {});

// Render the small-signal (modal) screen as a human-readable Markdown report:
// a stability headline, state/variable counts, a table of the least-damped
// modes, and per-mode participation for the most critical ones (design doc §18).
// Returns an empty string when the screen was not computed.
std::string to_modal_report(const DynamicModalSummary& modal,
                            int max_modes = 12,
                            int max_participation_modes = 3);

}  // namespace hacdcpf::dynamics

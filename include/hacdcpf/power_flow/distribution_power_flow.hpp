#pragma once
#include <array>
#include <complex>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Phase-node indexing
// ---------------------------------------------------------------------------

/// Reference to a single phase-node (one of the 3n compact nodes).
struct PhaseNodeRef {
  int compact_index{-1}; ///< index in the compact Ybus
  int bus_offset{-1};    ///< position of bus in ThreePhaseACSystem::buses
  int bus_id{0};         ///< bus_id from ThreePhaseACBus
  int phase_index{0};    ///< 0=a, 1=b, 2=c
};

/// Maps from (bus_offset, phase_index) to compact node index and back.
struct PhaseNodeIndexer {
  int total_nodes{0};
  std::vector<PhaseNodeRef>         nodes;
  std::vector<std::array<int, 3>>   bus_phase_to_node; ///< [bus_offset][phase] → compact idx
  std::vector<PhaseMask>            bus_phase_masks;

  static PhaseNodeIndexer build(const ThreePhaseACSystem& sys);

  bool         has_node(int bus_offset, int phase_index) const;
  int          node_index(int bus_offset, int phase_index) const;
  const PhaseNodeRef& node_ref(int compact_index) const;
  PhaseMask    bus_phase_mask(int bus_offset) const;
};

// ---------------------------------------------------------------------------
// Three-phase result helper types
// ---------------------------------------------------------------------------

/// Phasor array (real/imag per phase) for transformer terminal observations.
struct ThreePhasePhasorObservation {
  std::array<double, 3> real{};
  std::array<double, 3> imag{};
};

/// Voltage and current at both terminals of a three-phase transformer.
struct ThreePhaseTransformerTerminalObservation {
  int  transformer_index{0};
  std::string transformer_name;
  int  hv_bus{0};
  int  lv_bus{0};
  PhaseMask hv_phase_mask{PhaseMask::none()};
  PhaseMask lv_phase_mask{PhaseMask::none()};
  ThreePhasePhasorObservation hv_voltage_pu;
  ThreePhasePhasorObservation lv_voltage_pu;
  ThreePhasePhasorObservation hv_current_amps;
  ThreePhasePhasorObservation lv_current_amps;
};

/// Per-iteration regulator control trace (three-phase).
struct ThreePhaseRegulatorControlTraceEntry {
  std::string regulator_name;
  int    iteration{0};
  int    current_tap_pos{0};
  int    current_tap_number{0};
  double current_tap_pu{1.0};
  double monitored_voltage_pu{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double tap_winding_current_amps{0.0};
  double target_vreg_volts{0.0};
  double band_half_volts{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
  std::string decision_reason;
  int    next_tap_pos{0};
  int    next_tap_number{0};
  std::string stop_reason;
};

/// Final state of each regulator after a three-phase power flow.
struct ThreePhaseRegulatorControlState {
  std::string regulator_name;
  std::string transformer_name;
  int    transformer_index{0};
  int    winding{0};
  int    tap_winding{0};
  int    monitored_bus{0};
  int    monitored_node{1};
  bool   used_remote_bus{false};
  bool   used_line_drop_compensation{false};
  double target_vreg_volts{0.0};
  double band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int    max_tap_change{1};
  int    control_iterations{0};
  int    final_tap_pos{0};
  int    final_tap_number{0};
  double final_tap_pu{1.0};
  double monitored_voltage_pu{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double tap_winding_current_amps{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
  bool   converged{true};
  std::string stop_reason;
};

// ---------------------------------------------------------------------------
// Three-phase per-bus and per-branch results
// ---------------------------------------------------------------------------

/// Per-phase bus voltage result (magnitude and angle).
struct ThreePhaseBusVoltage {
  int    bus_id{0};
  double vm_a_pu{1.0};  double va_a_deg{0.0};
  double vm_b_pu{1.0};  double va_b_deg{-120.0};
  double vm_c_pu{1.0};  double va_c_deg{120.0};
};

/// Per-phase branch power result (sending-end, from from_bus).
struct ThreePhaseBranchPower {
  int    line_index{0};
  double p_a_mw{0.0};   double q_a_mvar{0.0};
  double p_b_mw{0.0};   double q_b_mvar{0.0};
  double p_c_mw{0.0};   double q_c_mvar{0.0};
};

/// Full three-phase power flow result.
struct ThreePhaseDPFResult {
  std::vector<ThreePhaseBusVoltage>  bus_voltages;
  std::vector<ThreePhaseBranchPower> branch_powers;
  std::vector<ThreePhaseTransformerTerminalObservation> transformer_terminal_observations;
  std::vector<ThreePhaseRegulatorControlState>          regulator_states;
  std::vector<ThreePhaseRegulatorControlTraceEntry>     regulator_trace;

  double p_loss_a_mw{0.0};    double q_loss_a_mvar{0.0};
  double p_loss_b_mw{0.0};    double q_loss_b_mvar{0.0};
  double p_loss_c_mw{0.0};    double q_loss_c_mvar{0.0};
  double total_p_loss_mw{0.0};
  double total_q_loss_mvar{0.0};

  double max_vuf_percent{0.0};

  bool   control_converged{true};
  bool   converged{false};
  int    iterations{0};
  double residual{0.0};
};

// ---------------------------------------------------------------------------
// Three-phase Newton-Raphson options
// ---------------------------------------------------------------------------

/// Sparse matrix entry in phase-domain (complex).
struct PhaseDomainSparseEntry {
  int row{-1};
  int col{-1};
  std::complex<double> value{0.0, 0.0};
};

/// Options for the three-phase NR solver.
struct ThreePhaseNROptions {
  int    max_iter{50};
  int    max_control_iter{100};
  double tol{1e-6};
  bool   verbose{false};
  bool   include_shunts{true};
};

// ---------------------------------------------------------------------------
// Three-phase Newton-Raphson interface
// ---------------------------------------------------------------------------

/// Solve three-phase power flow using the Newton-Raphson method on the
/// abc-domain compact admittance matrix.
ThreePhaseDPFResult solve_three_phase_nr(
    const ThreePhaseACSystem& sys,
    const ThreePhaseNROptions& opt = {});

/// Convenience overload for HybridPowerSystem.
ThreePhaseDPFResult solve_three_phase_nr(
    const HybridPowerSystem& sys,
    const ThreePhaseNROptions& opt = {});

/// Build full (n×n) phase-domain Ybus entries for a ThreePhaseACSystem.
std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const ThreePhaseACSystem& sys,
    bool include_shunts = true);

std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const HybridPowerSystem& sys,
    bool include_shunts = true);

}  // namespace hacdcpf::analysis

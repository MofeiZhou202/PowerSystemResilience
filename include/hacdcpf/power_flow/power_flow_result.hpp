#pragma once

/// power_flow/power_flow_result.hpp
/// ==================================
/// Power flow result types and solver diagnostics.
/// Replaces: model/results.hpp + model/diagnostics.hpp.

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/ac_components.hpp"  // IslandInfo
#include "hacdcpf/model/converter_model_scope.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Per-equation residual breakdown
// ═══════════════════════════════════════════════════════════════════════
struct ResidualBreakdown {
  double ac_p_norm{0.0};
  double ac_q_norm{0.0};
  double dc_p_norm{0.0};
  double converter_p_norm{0.0};
  double converter_q_norm{0.0};
  double control_eq_norm{0.0};

  [[nodiscard]] double total_norm() const noexcept {
    double m = ac_p_norm;
    if (ac_q_norm        > m) m = ac_q_norm;
    if (dc_p_norm        > m) m = dc_p_norm;
    if (converter_p_norm > m) m = converter_p_norm;
    if (converter_q_norm > m) m = converter_q_norm;
    if (control_eq_norm  > m) m = control_eq_norm;
    return m;
  }
};

// ═══════════════════════════════════════════════════════════════════════
// Per-iteration log entry
// ═══════════════════════════════════════════════════════════════════════
struct IterationLogEntry {
  int    iteration{0};
  double residual_norm{0.0};
  double step_norm{0.0};
  double damping_factor{1.0};
  bool   jacobian_refactorized{false};
  bool   lm_fallback_used{false};
  ResidualBreakdown breakdown;
};

// ═══════════════════════════════════════════════════════════════════════
// High-level solver diagnostics
// ═══════════════════════════════════════════════════════════════════════
struct SolverDiagnostics {
  bool        converged{false};
  std::string termination_reason;

  int    iterations{0};
  double initial_mismatch_norm{0.0};
  double final_mismatch_norm{0.0};

  double max_p_mismatch_pu{0.0};
  double max_q_mismatch_pu{0.0};
  double max_i_violation_pu{0.0};
  double condition_estimate{0.0};

  // Pre-solve structural closure check (multi-converter model §7.4).
  // The hybrid Newton system is square by construction (n_equations ==
  // n_variables); equation_closure_ok additionally confirms the assembled
  // Jacobian pattern has no all-zero row or column (a necessary condition for
  // full structural rank).  equation_closure_checked records that the scan ran.
  bool   equation_closure_checked{false};
  bool   equation_closure_ok{true};
  int    n_equations{0};
  int    n_variables{0};
  int    empty_jacobian_rows{0};
  int    empty_jacobian_cols{0};

  ResidualBreakdown final_breakdown;

  std::vector<IterationLogEntry> iteration_log;
  std::vector<std::string>       warnings;
  powerflow::ConverterCoordinationReport converter_coordination;

  // VSC converters auto-promoted from PQ to a Vdc-regulating mode because their
  // DC island had no voltage reference (indices into the solver's converter list).
  std::vector<int>               promoted_vsc_indices;

  // The solver's final converter list after auto-promotion, stiff-gain Vdc
  // forming and any in-iteration mode switching.  Post-solve result
  // reconstruction (AC/DC transfers, losses) must use these — not the input
  // converters — so the reported converter powers match the solved network
  // state.  Empty when no Newton solve populated it.
  std::vector<VSCConverter>      effective_converters;
};

// ═══════════════════════════════════════════════════════════════════════
// Solver profiling counters
// ═══════════════════════════════════════════════════════════════════════
struct SolverProfiling {
  std::string linear_solver_backend;
  int ac_eval_threads{1};
  int jacobian_pattern_rebuilds{0};
  int jacobian_analyze_calls{0};
  int factorization_calls{0};
  int linear_solve_calls{0};
  int regularization_attempts{0};
  int line_search_evaluations{0};
  int rejected_steps{0};
  int pv_to_pq_switches{0};
  int pq_to_pv_switches{0};
  int converter_mode_switches{0};
  std::vector<double> residual_by_iter;
  std::vector<int> line_search_evals_by_iter;
  std::vector<double> eval_jacobian_ms_by_iter;
  std::vector<double> linear_solve_ms_by_iter;
  std::vector<double> line_search_ms_by_iter;
  double eval_jacobian_ms_total{0.0};
  double linear_solve_ms_total{0.0};
  double line_search_ms_total{0.0};

  double raw_residual_norm{0.0};
  double scaled_residual_norm{0.0};
  double condition_estimate{0.0};
  int regularization_count{0};
  std::string linear_solver_status{"not_run"};
};

// ═══════════════════════════════════════════════════════════════════════
// Component flow results
// ═══════════════════════════════════════════════════════════════════════
struct BranchFlow {
  double pf_mw{0.0};
  double qf_mvar{0.0};
  double pt_mw{0.0};
  double qt_mvar{0.0};
};

// Terminal flow of a two-terminal device (switch / circuit breaker) recovered
// after canonical projection collapsed it. Populated by power flow / OPF so
// merged-out breakers still report a flow and loading.
struct DeviceTerminalFlow {
  int    index{0};
  int    bus_from{0};
  int    bus_to{0};
  bool   closed{true};
  double pf_mw{0.0}, pt_mw{0.0}, qf_mvar{0.0}, qt_mvar{0.0};
  double rate_mva{0.0};
  double loading_pct{0.0};
};

struct VSCTransfer {
  int index{0};
  int bus_ac{0};
  int bus_dc{0};
  double p_ac_mw{0.0};
  double q_ac_mvar{0.0};
  double p_dc_mw{0.0};
  double loss_mw{0.0};
};

// LCC (line-commutated converter) quasi-steady station result (dat manual
// ch.4).  One entry per in-service LCCConverter, keyed by the stable
// component `.index`; bus_ac/bus_dc are AUTHORED component ids (restored
// across the canonical projection).  Units: kV, kA, MW, Mvar, degrees.
// Sign conventions (same as VSCTransfer): p_ac_mw > 0 delivers power to the
// AC grid (inverter), < 0 absorbs (rectifier); q_ac_mvar < 0 always (an LCC
// consumes reactive power); p_dc_mw > 0 injects into the DC network
// (rectifier), < 0 draws (inverter).
struct LCCTransfer {
  int index{0};          // LCCConverter.index (stable component id)
  int bus_ac{0};         // authored valve-side AC bus .index
  int bus_dc{0};         // authored DC bus .index
  int station_role{0};   // LCCStationRole: 0 = Rectifier, 1 = Inverter
  int control_mode{0};   // LCCControlMode of the station
  double alpha_deg{0.0}; // firing angle, back-calculated from the solved
                         // state (fixed taps: departs from the scheduled
                         // AlphaN a tap changer would hold)
  double gamma_deg{0.0}; // extinction angle: the CEA control value while the
                         // characteristic holds; back-calculated physical
                         // value when the rated-current limit binds (then
                         // gamma >= gamma_set — see id_at_limit)
  double ud0_kv{0.0};    // ideal no-load DC voltage (3*sqrt(2)/pi)*n_b*E at
                         // the solved valve-side voltage
  double ud_kv{0.0};     // solved DC terminal voltage
  double id_ka{0.0};     // DC current (positive in power direction)
  double p_ac_mw{0.0};
  double q_ac_mvar{0.0};
  double p_dc_mw{0.0};
  double transformer_tap{1.0};       // effective ACBranch.tap used by the solve
  double tap_target_angle_deg{0.0};  // alpha target (rectifier) or gamma target
  int tap_control_iterations{0};     // accepted tap changes in the outer loop
  bool tap_control_active{false};
  bool tap_control_converged{false};
  bool tap_at_limit{false};
  bool id_at_limit{false};           // DC current clamped at rated_current_a:
                                     // the current order/limit binds, so a
                                     // CEA/constant-alpha setpoint is NOT
                                     // held (gamma/alpha then float above
                                     // their minimum — physically safe)
  bool alpha_within_limits{true};  // alpha_min_deg <= alpha <= alpha_stop_deg
  bool gamma_within_limits{true};  // gamma >= gamma_min_deg (when specified)
};

struct DCDCTransfer {
  int index{0};
  int bus_in{0};
  int bus_out{0};
  double p_in_mw{0.0};
  double p_out_mw{0.0};
  double loss_mw{0.0};
  // Ideal CCM duty ratio backed out from the solved port voltages (or modulation
  // gain for the Isolated topology), and whether it lies in the converter's
  // [d_min, d_max] window.  duty_defined is false for the Generic topology.
  double duty{0.0};
  double voltage_ratio{0.0};
  bool duty_defined{false};
  bool duty_feasible{true};
};

struct Trafo3WFlow {
  int index{0};
  int hv_bus{0}, mv_bus{0}, lv_bus{0};
  double p_hv_mw{0.0}, q_hv_mvar{0.0};
  double p_mv_mw{0.0}, q_mv_mvar{0.0};
  double p_lv_mw{0.0}, q_lv_mvar{0.0};
  double loss_mw{0.0};
  double loading_pct{0.0};
  double rate_mva{0.0};
};

struct ERPortTransfer {
  int router_index{0};
  int port_index{0};
  int bus{0};
  bool is_ac{true};
  double p_mw{0.0};
  double q_mvar{0.0};
  double v_pu{1.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Power flow result types
// ═══════════════════════════════════════════════════════════════════════
struct PowerFlowResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  SolverProfiling profiling;
  SolverDiagnostics diagnostics;
  std::vector<BranchFlow> branch_flows;
  std::vector<VSCTransfer> vsc_transfers;
  std::vector<LCCTransfer> lcc_transfers;
  std::vector<DCDCTransfer> dcdc_transfers;
  std::vector<Trafo3WFlow> trafo3w_flows;
  std::vector<ERPortTransfer> er_port_transfers;

  // Switch / circuit-breaker terminal flows recovered across collapsed
  // (merged-out) zero-impedance devices. Empty when the system has none.
  std::vector<DeviceTerminalFlow> ac_switch_flows;
  std::vector<DeviceTerminalFlow> ac_circuit_breaker_flows;

  /// Declares which parts of the unified converter model this solve honored.
  ConverterModelScope converter_model_scope{};
};

struct DCPowerFlowResult {
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
};

struct AdaptiveSolveResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::vector<IslandInfo> islands;
  std::vector<BranchFlow> branch_flows;
};

struct IslandedSolveResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::vector<IslandInfo> islands;
};

struct DistributedSlackResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  SolverDiagnostics diagnostics;
  std::unordered_map<int, double> distributed_slack_p;
  std::vector<int> hit_limits;
};

}  // namespace hacdcpf

#pragma once

// Distribution Power Flow — Backward-Forward Sweep (BFS) and
// Three-Phase Newton-Raphson (NR) solvers.
//
// Single-phase BFS: Shirmohammadi et al. 1988. Radial networks only.
// Three-phase NR:   Polar-form Newton-Raphson on a compact abc-domain Ybus
//                   built only on active phase nodes. Handles radial and
//                   meshed networks, multiple sources.
//                   Uses existing SparseLinearSolver (KLU/UMFPACK/SparseLU).
//
// The three-phase NR is the primary solver for hybrid AC/DC systems
// with three-phase modeling, as BFS cannot handle multiple sources or
// meshed topologies efficiently.

#include <array>
#include <complex>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/system.hpp"
#include "hacdcpf/model/three_phase.hpp"

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Phase domain selector
// ---------------------------------------------------------------------------

/// Controls whether the DPF solver operates on a single-phase equivalent
/// (positive-sequence) model or a full per-phase (abc) formulation.
enum class PhaseDomain {
  PositiveSequence, ///< Single-phase equivalent BFS (default)
  ABC               ///< Three-phase abc BFS with 3×3 impedance coupling
};

struct PhaseNodeRef {
  int compact_index{-1};
  int bus_offset{-1};
  int bus_id{0};
  int phase_index{0};
};

struct PhaseNodeIndexer {
  int total_nodes{0};
  std::vector<PhaseNodeRef> nodes;
  std::vector<std::array<int, 3>> bus_phase_to_node;
  std::vector<PhaseMask> bus_phase_masks;

  static PhaseNodeIndexer build(const ThreePhaseACSystem& sys);

  bool has_node(int bus_offset, int phase_index) const;
  int node_index(int bus_offset, int phase_index) const;
  const PhaseNodeRef& node_ref(int compact_index) const;
  PhaseMask bus_phase_mask(int bus_offset) const;
};

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

struct DPFOptions {
  int    max_iter{100};              ///< Maximum BFS iterations
  int    max_control_iter{100};      ///< Maximum outer regulator-control iterations
  double tol{1e-6};                  ///< Convergence tolerance on max |ΔV| (p.u.)
  bool   verbose{false};             ///< Print per-iteration residual
  bool   include_shunts{true};       ///< Include ACBus shunt (gs/bs) in power balance
  bool   enforce_q_limits{false};    ///< Enforce generator reactive power limits (PV→PQ)
  int    q_limit_max_outer_iter{30}; ///< Maximum outer-loop iterations for Q-limit enforcement
};

// ---------------------------------------------------------------------------
// Results — indexed in the same order as ac_sys.buses
// ---------------------------------------------------------------------------

struct RegulatorControlTraceEntry {
  std::string regulator_name;
  int iteration{0};
  int current_tap_pos{0};
  int current_tap_number{0};
  double current_tap_pu{1.0};
  double monitored_voltage_pu{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double target_vreg_volts{0.0};
  double band_half_volts{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
  std::string decision_reason;
  int next_tap_pos{0};
  int next_tap_number{0};
  std::string stop_reason;
};

struct RegulatorControlState {
  std::string regulator_name;
  std::string transformer_name;
  int transformer_index{0};
  int winding{0};
  int tap_winding{0};
  int monitored_bus{0};
  int monitored_node{1};
  bool used_remote_bus{false};
  bool used_line_drop_compensation{false};
  double target_vreg_volts{0.0};
  double band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int max_tap_change{1};
  int control_iterations{0};
  int final_tap_pos{0};
  int final_tap_number{0};
  double final_tap_pu{1.0};
  double monitored_voltage_pu{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
  bool converged{true};
  std::string stop_reason;
};

struct ThreePhasePhasorObservation {
  std::array<double, 3> real{};
  std::array<double, 3> imag{};
};

struct ThreePhaseTransformerTerminalObservation {
  int transformer_index{0};
  std::string transformer_name;
  int hv_bus{0};
  int lv_bus{0};
  PhaseMask hv_phase_mask{PhaseMask::none()};
  PhaseMask lv_phase_mask{PhaseMask::none()};
  ThreePhasePhasorObservation hv_voltage_pu;
  ThreePhasePhasorObservation lv_voltage_pu;
  ThreePhasePhasorObservation hv_current_amps;
  ThreePhasePhasorObservation lv_current_amps;
};

struct ThreePhaseRegulatorControlTraceEntry {
  std::string regulator_name;
  int iteration{0};
  int current_tap_pos{0};
  int current_tap_number{0};
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
  int next_tap_pos{0};
  int next_tap_number{0};
  std::string stop_reason;
};

struct ThreePhaseRegulatorControlState {
  std::string regulator_name;
  std::string transformer_name;
  int transformer_index{0};
  int winding{0};
  int tap_winding{0};
  int monitored_bus{0};
  int monitored_node{1};
  bool used_remote_bus{false};
  bool used_line_drop_compensation{false};
  double target_vreg_volts{0.0};
  double band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double r_volts{0.0};
  double x_volts{0.0};
  int max_tap_change{1};
  int control_iterations{0};
  int final_tap_pos{0};
  int final_tap_number{0};
  double final_tap_pu{1.0};
  double monitored_voltage_pu{0.0};
  double monitored_voltage_volts{0.0};
  double control_voltage_volts{0.0};
  double tap_winding_current_amps{0.0};
  double line_drop_compensation_real_volts{0.0};
  double line_drop_compensation_imag_volts{0.0};
  double line_drop_compensation_magnitude_volts{0.0};
  bool converged{true};
  std::string stop_reason;
};

struct DPFResult {
  std::vector<double> vm_pu;        ///< Voltage magnitude [p.u.], size = n_buses
  std::vector<double> va_deg;       ///< Voltage angle [deg], size = n_buses
  std::vector<double> p_branch_mw;  ///< Active power sent from from_bus [MW], size = n_branches
  std::vector<double> q_branch_mvar;///< Reactive power sent from from_bus [MVAr], size = n_branches
  double total_p_loss_mw{0.0};      ///< Sum of active losses in all branches
  double total_q_loss_mvar{0.0};    ///< Sum of reactive losses in all branches
  std::vector<RegulatorControlState> regulator_states;
  std::vector<RegulatorControlTraceEntry> regulator_trace;
  bool   specialized_path_applied{false};
  std::string specialized_path_id;
  std::string specialized_path_fail_close_reason;
  bool   control_converged{true};
  bool   converged{false};
  int    iterations{0};
  double residual{0.0};             ///< max |ΔV| at last iteration [p.u.]
};

// ---------------------------------------------------------------------------
// Three-phase DPF result — per-phase voltage, branch power, and losses
// ---------------------------------------------------------------------------

/// Per-phase bus voltage result.
struct ThreePhaseBusVoltage {
  int bus_id{0};
  double vm_a_pu{1.0};  double va_a_deg{0.0};
  double vm_b_pu{1.0};  double va_b_deg{-120.0};
  double vm_c_pu{1.0};  double va_c_deg{120.0};
};

/// Per-phase branch power result (sending-end, from from_bus).
struct ThreePhaseBranchPower {
  int line_index{0};
  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};
};

/// Full three-phase DPF result.
struct ThreePhaseDPFResult {
  std::vector<ThreePhaseBusVoltage> bus_voltages;   ///< Per-bus per-phase |V|∠θ
  std::vector<ThreePhaseBranchPower> branch_powers;  ///< Per-branch per-phase S_from
  std::vector<ThreePhaseTransformerTerminalObservation> transformer_terminal_observations;
  std::vector<ThreePhaseRegulatorControlState> regulator_states;
  std::vector<ThreePhaseRegulatorControlTraceEntry> regulator_trace;

  // Per-phase total losses
  double p_loss_a_mw{0.0};   double q_loss_a_mvar{0.0};
  double p_loss_b_mw{0.0};   double q_loss_b_mvar{0.0};
  double p_loss_c_mw{0.0};   double q_loss_c_mvar{0.0};
  double total_p_loss_mw{0.0};
  double total_q_loss_mvar{0.0};

  // Unbalance metric: max voltage unbalance factor (VUF) across all buses
  double max_vuf_percent{0.0};

  bool   control_converged{true};
  bool   converged{false};
  int    iterations{0};
  double residual{0.0};             ///< max |ΔV| across all phases at last iteration

  std::string primary_solver;
  std::string solver_used;
  std::string result_source;
  bool fallback_used{false};
  std::string primary_solver_failed_reason;
};

// ---------------------------------------------------------------------------
// Phase-domain compatible shell
// ---------------------------------------------------------------------------

enum class PhaseDomainSolverAlgorithm {
  Newton,
  FixedPoint,
  Compact,
  OpenDSS,
};

enum class PhaseDomainBranchType {
  Line,
  Transformer,
  Regulator,
};

enum class PhaseDomainConnectionType {
  Unknown = 0,
  Wye = 1,
  Delta = 2,
  GroundedWye = 3,
};

struct PhaseDomainSparseEntry {
  int row{-1};
  int col{-1};
  std::complex<double> value{0.0, 0.0};
};

struct PhaseDomainBusRow {
  int bus_id{0};
  BusType bus_type{BusType::PQ};
  double base_kv{0.0};

  std::array<double, 3> vm_pu{1.0, 1.0, 1.0};
  std::array<double, 3> va_deg{0.0, -120.0, 120.0};
  std::array<double, 3> pd_mw{};
  std::array<double, 3> qd_mvar{};
  std::array<double, 3> pg_mw{};
  std::array<double, 3> qg_mvar{};
  std::array<bool, 3> has_phase{true, true, true};

  int zone{1};
  int area{1};

  std::array<double, 3> gs_mw{};
  std::array<double, 3> bs_mvar{};
};

struct PhaseDomainBranchRow {
  int from_bus{0};
  int to_bus{0};
  bool in_service{true};
  PhaseDomainBranchType branch_type{PhaseDomainBranchType::Line};

  PhaseMask from_phase_mask{PhaseMask::none()};
  PhaseMask to_phase_mask{PhaseMask::none()};

  std::array<std::array<double, 3>, 3> r_pu{};
  std::array<std::array<double, 3>, 3> x_pu{};
  std::array<double, 3> b_pu{};

  std::array<double, 3> tap_pu{1.0, 1.0, 1.0};
  std::array<double, 3> shift_deg{};

  PhaseDomainConnectionType from_connection{PhaseDomainConnectionType::Unknown};
  PhaseDomainConnectionType to_connection{PhaseDomainConnectionType::Unknown};

  std::string element_name;
  std::string vector_group;
  std::string from_winding_topology;
  std::string to_winding_topology;
  double length_km{0.0};
};

struct PhaseDomainDeltaLoadRow {
  int bus_row{-1};
  int bus_id{0};
  PhaseMask phase_mask{PhaseMask::none()};
  std::array<std::complex<double>, 3> line_power_mva{};
};

struct ThreePhaseJPCPhase {
  double base_mva{100.0};
  double base_freq_hz{50.0};

  std::vector<PhaseDomainBusRow> bus_abc;
  std::vector<PhaseDomainBranchRow> branch_abc;
  std::vector<PhaseDomainDeltaLoadRow> delta_loads;

  std::vector<PhaseDomainSparseEntry> ybus_abc_entries;
  int ybus_abc_dim{0};

  std::vector<int> ref_bus_rows;
  std::vector<int> pv_bus_rows;
  std::vector<int> pq_bus_rows;

  std::unordered_map<int, std::string> bus_id_to_name;
  std::unordered_map<std::string, int> bus_name_to_id;

  std::vector<std::complex<double>> v_abc;

  bool success{false};
  int iterations{0};
  double residual{0.0};

  std::string primary_solver;
  std::string solver_used;
  std::string result_source;
  bool fallback_used{false};
  std::string primary_solver_failed_reason;
};

struct OpenDSSSparseYMatrix {
  int dimension{0};
  std::vector<std::string> node_order;
  std::vector<PhaseDomainSparseEntry> entries;
};

struct ThreePhaseCompactPFData {
  PhaseNodeIndexer indexer;
  std::vector<int> fixed_indices;
  std::vector<int> variable_indices;
  std::vector<std::complex<double>> fixed_voltage;
  std::vector<std::complex<double>> fixed_current;
  std::vector<PhaseDomainSparseEntry> ybus_entries;
  std::vector<PhaseDomainSparseEntry> y_vv_entries;
  std::vector<PhaseDomainSparseEntry> y_vf_entries;
};

struct ThreePhaseFixedPointOptions {
  int max_iter{100};
  double tol{1e-6};
  bool verbose{false};
  bool include_shunts{true};
  double vmin_pu{0.95};
};

struct RunPFPhaseOptions {
  PhaseDomainSolverAlgorithm algorithm{PhaseDomainSolverAlgorithm::Newton};
  int max_iter{100};
  double tol{1e-6};
  bool verbose{false};
  bool include_shunts{true};
  double vmin_pu{0.95};
  std::unordered_map<std::string, double> reg_taps;
  std::filesystem::path dss_file_path;
};

// ---------------------------------------------------------------------------
// Primary interface — single-phase equivalent
// ---------------------------------------------------------------------------

/// Solve distribution power flow on a single-phase equivalent AC network using
/// the Backward-Forward Sweep algorithm (Shirmohammadi et al. 1988).
///
/// Requires exactly one SLACK bus. Non-radial networks (loops) are not
/// supported; call is_radial() from topology_analysis.hpp to verify first.
///
/// Results are indexed identically to ac_sys.buses, so:
///   result.vm_pu[i] corresponds to ac_sys.buses[i]
DPFResult solve_distribution_pf(const ACSystem& ac_sys,
                                 const DPFOptions& opt = {});

/// Overload accepting a full HybridPowerSystem, which includes VPP, Microgrid,
/// and MobileStorage injections at the AC bus level.
DPFResult solve_distribution_pf(const HybridPowerSystem& sys,
                                 const DPFOptions& opt = {});

// ---------------------------------------------------------------------------
// Three-phase BFS interface
// ---------------------------------------------------------------------------

/// Solve three-phase distribution power flow using per-phase BFS on
/// ThreePhaseACSystem.  Builds a 3×3 series impedance matrix from
/// positive- and zero-sequence parameters, then runs a coupled
/// backward-forward sweep in the abc domain.
///
/// The algorithm reduces to the single-phase BFS on balanced systems:
/// if all per-phase loads are equal and line Z0 = Z1, the per-phase
/// voltages converge to the balanced positive-sequence solution.
///
/// Requires exactly one SLACK bus and a radial network topology.
ThreePhaseDPFResult solve_three_phase_distribution_pf(
    const ThreePhaseACSystem& sys,
    const DPFOptions& opt = {});

/// Convenience overload: if the HybridPowerSystem has a three_phase_ac
/// subsystem, solves the three-phase BFS directly on it.
ThreePhaseDPFResult solve_three_phase_distribution_pf(
    const HybridPowerSystem& sys,
    const DPFOptions& opt = {});

// ---------------------------------------------------------------------------
// Three-phase Newton-Raphson interface (general networks)
// ---------------------------------------------------------------------------

/// Options specific to the three-phase NR solver.
struct ThreePhaseNROptions {
  int    max_iter{50};          ///< Maximum NR iterations
  int    max_control_iter{100}; ///< Maximum outer regulator-control iterations
  double tol{1e-6};             ///< Convergence tolerance on max |ΔP|,|ΔQ| mismatch (p.u.)
  bool   verbose{false};        ///< Print per-iteration residual
  bool   include_shunts{true};  ///< Include per-phase bus shunt admittance in Ybus
};

/// Solve three-phase power flow using the Newton-Raphson method on the
/// abc-domain compact admittance matrix built only on existing phase-nodes.
/// This solver:
///   - handles radial and meshed topologies,
///   - supports multiple generators/sources,
///   - uses polar-form state variables [θ_a,θ_b,θ_c,...,Vm_a,Vm_b,Vm_c,...],
///   - builds a compact Ybus from phase membership plus sequence-to-abc
///     impedance conversion,
///   - uses the existing sparse LU infrastructure (KLU/UMFPACK/SparseLU).
///
/// The result structure is the same ThreePhaseDPFResult for compatibility.
///
/// Requires at least one SLACK bus.
ThreePhaseDPFResult solve_three_phase_nr(
    const ThreePhaseACSystem& sys,
    const ThreePhaseNROptions& opt = {});

/// Convenience overload for HybridPowerSystem.
ThreePhaseDPFResult solve_three_phase_nr(
    const HybridPowerSystem& sys,
    const ThreePhaseNROptions& opt = {});

ThreePhaseACSystem load_three_phase_system_from_opendss(
    const std::filesystem::path& master_dss,
    double base_mva = 1.0);

std::unordered_map<std::string, double> get_opendss_regulator_taps(
    const std::filesystem::path& master_dss);

OpenDSSSparseYMatrix build_opendss_sparse_y_matrix(
    const std::filesystem::path& master_dss,
    const std::unordered_map<std::string, double>& reg_taps = {});

ThreePhaseJPCPhase case2jpc_phase(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<std::string, double>& reg_taps = {});

ThreePhaseJPCPhase case2jpc_phase(
    const HybridPowerSystem& sys,
    const std::unordered_map<std::string, double>& reg_taps = {});

void makeYbus_phase(
    ThreePhaseJPCPhase& jpc,
    const ThreePhaseACSystem& sys,
    bool include_shunts = true);

void makeYbus_phase(
    ThreePhaseJPCPhase& jpc,
    const HybridPowerSystem& sys,
    bool include_shunts = true);

std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const ThreePhaseACSystem& sys,
    bool include_shunts = true);

std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const HybridPowerSystem& sys,
    bool include_shunts = true);

ThreePhaseCompactPFData build_compact_pf_data(
    const ThreePhaseACSystem& sys,
    bool include_shunts = true);

ThreePhaseCompactPFData build_compact_pf_data(
    const HybridPowerSystem& sys,
    bool include_shunts = true);

ThreePhaseDPFResult solve_three_phase_compact_pf(
    const ThreePhaseACSystem& sys,
    const ThreePhaseFixedPointOptions& opt = {});

ThreePhaseDPFResult solve_three_phase_compact_pf(
    const HybridPowerSystem& sys,
    const ThreePhaseFixedPointOptions& opt = {});

ThreePhaseDPFResult solve_three_phase_fixed_point(
    const ThreePhaseACSystem& sys,
    const ThreePhaseFixedPointOptions& opt = {});

ThreePhaseDPFResult solve_three_phase_fixed_point(
    const HybridPowerSystem& sys,
    const ThreePhaseFixedPointOptions& opt = {});

ThreePhaseJPCPhase runpf_phase(
    const ThreePhaseACSystem& sys,
    const RunPFPhaseOptions& opt = {});

ThreePhaseJPCPhase runpf_phase(
    const HybridPowerSystem& sys,
    const RunPFPhaseOptions& opt = {});

}  // namespace hacdcpf::analysis

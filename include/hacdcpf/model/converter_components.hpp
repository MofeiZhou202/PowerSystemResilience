#pragma once

/// model/converter_components.hpp
/// ================================
/// VSC converters, DC-DC converters, and energy router types.
/// Replaces: converters.hpp.

#include <string>
#include <vector>

#include "hacdcpf/model/dynamic_model_profile.hpp"
#include "hacdcpf/model/enums/converter_enums.hpp"

namespace hacdcpf {

enum class VSCHarmonicTopology { Disabled, TwoLevel, MMC };

struct HarmonicPIController {
  double kp{0.0};
  double ki{0.0};
  double delay_s{0.0};
};

// Periodic steady-state converter data used by the harmonic state-space
// formulation. Zero/Disabled is an explicit opt-out, never an inferred model.
struct VSCHarmonicModel {
  VSCHarmonicTopology topology{VSCHarmonicTopology::Disabled};
  double switching_frequency_hz{0.0};
  double modulation_index{0.0};
  double modulation_phase_deg{0.0};
  double transfer_conductance_pu{0.0};
  double filter_resistance_ohm{0.0};
  double filter_inductance_h{0.0};
  double filter_capacitance_f{0.0};
  double dc_link_capacitance_f{0.0};
  double dc_link_esr_ohm{0.0};
  HarmonicPIController current_controller;
  HarmonicPIController dc_voltage_controller;

  // MMC arm-energy parameters. They are required when topology == MMC.
  int submodules_per_arm{0};
  double submodule_capacitance_f{0.0};
  double arm_inductance_h{0.0};
  double arm_resistance_ohm{0.0};
  double circulating_current_kp{0.0};
  double circulating_current_ki{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// VSC Converter (AC-DC coupling)
// ═══════════════════════════════════════════════════════════════════════
struct VSCConverter {
  int index{0};
  int bus_ac{0};
  int bus_dc{0};
  bool in_service{true};
  ConverterMode control_mode{ConverterMode::PQ_MODE};
  std::string type;

  double p_set_mw{0.0};
  // Active-power setpoint semantics (multi-converter model §16.3).  Historically
  // p_set_mw carried three meanings at once (hard constraint, dispatch schedule,
  // and initial guess), which is the root of the "grid-forming converter still
  // pins AC P" foot-gun.  These fields disambiguate it:
  //   * p_is_hard_constraint — when true, p_set_mw is a *binding* AC active-power
  //     constraint (typical AC_PQ converter).  A DC voltage-forming converter
  //     must release AC P, so combining this with DC grid-forming is rejected
  //     (rule ACDC-GFM-01).
  //   * p_schedule_mw — non-binding dispatch/schedule reference used when AC P is
  //     free (falls back to p_set_mw when left at 0).
  //   * p_initial_mw — solver warm-start hint for the released AC active power
  //     (falls back to p_set_mw when left at 0).
  bool p_is_hard_constraint{false};
  double p_schedule_mw{0.0};
  double p_initial_mw{0.0};
  double q_set_mvar{0.0};
  double v_dc_set_pu{1.0};
  double v_ac_set_pu{1.0};
  // Legacy AC voltage-angle reference in degrees. For the balanced GFM Norton
  // contract this is only the fallback internal-source angle; terminal Va is a
  // solved network state. New GFM data should use gfm_internal_angle_set_deg.
  double v_ac_angle_set_deg{0.0};

  // Authoritative balanced positive-sequence GFM port behind a virtual
  // impedance. PF, OPF adapters, and transient initialization resolve these
  // fields through model::resolve_gfm_norton_parameters. Zero values retain
  // the documented legacy fallback; module-local defaults are not permitted.
  double gfm_internal_voltage_set_pu{0.0};
  double gfm_internal_angle_set_deg{0.0};
  double gfm_virtual_r_pu{0.0};
  double gfm_virtual_x_pu{0.0};

  double eta{0.99};
  double loss_percent{0.0};
  double loss_mw{0.0};
  double k_vdc{0.1};

  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double p_rated_mw{0.0};
  double vn_ac_kv{0.0};
  double vn_dc_kv{0.0};

  // Steady-state physical feasibility limits (multi-converter model §3.1.4–3.1.7).
  // All are opt-in: a value of 0 disables the corresponding post-solve check.
  //   * i_ac_max_pu / i_dc_max_pu — AC/DC current limits (pu on system base).
  //   * k_m_modulation — modulation gain K_m in V_ac^model = K_m·m·V_dc.
  //   * m_min / m_max  — modulation-index feasibility window.
  // These are distinct from the IEC 60909 short-circuit i_max_pu below.
  double i_ac_max_pu{0.0};
  double i_dc_max_pu{0.0};
  double k_m_modulation{0.0};
  double m_min{0.0};
  double m_max{0.0};

  // Opt-in production power-flow complementarity model.  When enabled with a
  // positive i_ac_max_pu, the balanced Newton solve allocates a fixed local VSC
  // block and solves the current-limit operating point instead of merely
  // auditing the unconstrained root after convergence.
  bool enable_limit_ncp{false};
  VSCCurrentLimitPriority current_limit_priority{
      VSCCurrentLimitPriority::Magnitude};
  // Optional saturation of the Vdc-droop active-power command.  A zero lower
  // and upper pair means that pmin_mw/pmax_mw provide the effective bounds.
  double droop_p_min_mw{0.0};
  double droop_p_max_mw{0.0};

  double k_p{0.0};
  double k_q{0.0};
  double v_ref_pu{1.0};
  double f_ref_hz{50.0};

  bool controllable{true};

  // Steady-state AC-side conduction resistance (pu on base_mva / system base).
  // When > 0, the converter draws additional power from the DC bus to cover
  // I²·r losses on the AC transformer/reactor side:
  //   ploss_AC = r_conv_ac_pu * pac² / Vm_AC²
  // This couples the DC power-balance equation to the AC terminal voltage,
  // producing the cross-block Jacobian entry ∂P_DC / ∂V_AC_m.
  double r_conv_ac_pu{0.0};

  // Short-circuit impedance (IEC 60909)
  double r_sc_pu{0.0};
  double x_sc_pu{0.15};
  double r2_sc_pu{0.0};
  double x2_sc_pu{0.0};
  double i_max_pu{1.0};
  bool grid_forming{false};

  // AC-side grid-forming (multi-converter model r1 §2, VSC Mode 1): the
  // converter forms the AC voltage reference (angle + magnitude) at its AC
  // terminal.  This is distinct from grid_forming, which denotes DC-side voltage
  // forming.  An ordinary two-port converter cannot grid-form on both sides at
  // once; the advanced dual-side case is only valid with an explicit energy
  // buffer (allow_dual_side_grid_forming && has_energy_buffer gate ACDC-GFM-03).
  bool ac_grid_forming{false};
  bool allow_dual_side_grid_forming{false};
  bool has_energy_buffer{false};

  // Multi-source DC voltage coordination (multi-converter model §6.5–6.7).
  // Converters that share one DC island's voltage form a coordination group:
  //   * coordination_group_id — groups co-operating Vdc formers (empty = ungrouped).
  //   * is_master — master-slave designation; exactly one master per declared group.
  //   * participation_factor — share of the island power imbalance this source
  //     takes under participation-factor control; per group these must sum to 1.
  std::string coordination_group_id;
  bool is_master{false};
  double participation_factor{0.0};

  std::string name;

  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
  // Derived MTBF -- see Generator::mtbf_hr note for convention.
  double mtbf_hr{0.0};

  DynamicModelProfile dynamic_model;
  VSCHarmonicModel harmonic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// LCC Converter (line-commutated, thyristor HVDC — AC-DC coupling)
// ═══════════════════════════════════════════════════════════════════════
// Quasi-steady model of a classic grid-commutated converter station
// (dat card manual §4, BD/LD cards).  Physics carried per station:
//   U_d0 = (3*sqrt(2)/pi) * n_bridges * E          (E = valve-side no-load
//   rectifier: U_d = U_d0*cos(alpha) - (3/pi)*n_bridges*X_c*I_d - n_b*dU_v
//   inverter:  U_d = U_d0*cos(gamma) - (3/pi)*n_bridges*X_c*I_d + n_b*dU_v
// The commutation reactance X_c comes from the converter transformer leakage
// (T card); per the manual the T-card R/X is on the transformer Sn base, so
// x_comm_pu below is explicitly tied to x_comm_base_mva.
struct LCCConverter {
  int index{0};              // stable component id
  std::string name;
  int ac_bus{0};             // valve-side AC bus (ACBus.index)
  int dc_bus{0};             // DC bus (DCBus.index)
  bool in_service{true};
  LCCStationRole station_role{LCCStationRole::Rectifier};

  // ── Physical parameters (BD card) ──────────────────────────────────────
  int n_bridges{1};          // series 6-pulse bridges per pole
  double alpha_min_deg{5.0};   // minimum firing angle, rectifier limit (deg)
  double alpha_stop_deg{140.0};// maximum firing angle / inversion limit (deg)
  // Minimum extinction angle, inverter CEA limit (deg).  The BD card carries
  // no gamma_min (only BM cards do); 0 = unspecified by the data source.
  double gamma_min_deg{0.0};
  double v_drop_v{0.0};        // forward voltage drop per bridge valve (V)
  double rated_current_a{0.0}; // rated bridge (DC) current (A)
  // Per-bridge commutation reactance per phase, in pu on x_comm_base_mva at
  // the valve-side voltage. For BPA/DSP, one T-card branch is the AC-parallel
  // equivalent of the n_bridges series-connected DC bridge transformers; the
  // importer therefore multiplies the branch-equivalent leakage by
  // n_bridges to recover one bridge's X_c. x_comm_ohm is the same quantity in
  // ohms referred to the valve side, derived as
  // x_comm_pu * vn_ac_kv^2 / x_comm_base_mva.  0 = not available (the
  // converter transformer was not identified).
  double x_comm_pu{0.0};
  double x_comm_base_mva{0.0};
  double x_comm_ohm{0.0};
  double rated_dc_kv{0.0};     // rated DC voltage of the link (kV)
  double vn_ac_kv{0.0};        // valve-side AC base voltage (kV)
  // Smoothing reactor inductance (mH).  Dynamic simulations only; it does
  // not enter the quasi-steady power-flow equations.
  double smoothing_reactor_mh{0.0};

  // ── Control mode and setpoints (LD card) ───────────────────────────────
  // Exactly the setpoint matching control_mode is meaningful; the others are
  // informational defaults (0 = unset).
  LCCControlMode control_mode{LCCControlMode::ConstantPower};
  double p_set_mw{0.0};        // scheduled DC power at the control point (MW)
  double i_set_ka{0.0};        // DC current setpoint (kA)
  double alpha_set_deg{0.0};   // normal firing angle, rectifier (deg)
  double gamma_set_deg{0.0};   // normal extinction angle, inverter CEA (deg)
  double v_dc_set_kv{0.0};     // scheduled local DC-terminal voltage (kV)

  // Non-solving provenance/capability metadata. DSP may report compound
  // station codes such as PAAL (power+alpha) or VDGA (Vdc+gamma); these are
  // not interchangeable with control_mode, which selects the C++ equation
  // actually enforced. tap_control_modelled is true only when a converter-
  // transformer branch and a valid adjustable-tap range are available to the
  // unified power-flow tap-control outer loop.
  std::string external_control_code;
  bool tap_control_modelled{false};

  // Converter transformer reference: ACBranch.index of the T-card branch
  // between the primary AC bus and the valve-side bus; -1 = not identified.
  int converter_transformer_branch{-1};

  // Converter-transformer tap-control range in ACBranch::tap coordinates.
  // The BPA R-card winding limits are normalized during import, so the solver
  // need not reinterpret card orientation. transformer_tap_steps <= 1 means
  // continuous adjustment; otherwise it is the number of discrete positions.
  double transformer_tap_min_pu{0.0};
  double transformer_tap_max_pu{0.0};
  int transformer_tap_steps{0};
  int transformer_tap_winding{0};  // source winding number, 1 or 2

  // Honest-result bookkeeping (project convention): this is a quasi-steady
  // model with an approximate Q=P*tan(phi) relation and no commutation
  // overlap iteration; see model_limitations for source-specific coverage.
  std::string model_scope;
  std::string model_limitations;

  // Six-pulse switching-function harmonic model. The existing firing angle,
  // commutation reactance, bridge count and smoothing reactor remain the
  // physical source of truth.
  bool harmonic_model_enabled{false};
  double harmonic_transfer_conductance_pu{0.0};
  double dc_filter_capacitance_f{0.0};
  double dc_filter_esr_ohm{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC-DC Converter
// ═══════════════════════════════════════════════════════════════════════
struct DCDCConverter {
  int index{0};
  int bus_in{0};
  int bus_out{0};
  bool in_service{true};
  std::string name;

  DCDCControlMode control_mode{DCDCControlMode::Voltage};

  double p_ref_mw{0.0};
  double v_ref_pu{1.0};
  double sn_mva{0.0};
  double vn_in_kv{0.0};
  double vn_out_kv{0.0};
  double eta{0.98};
  double r_eq_pu{0.0};

  double pmax_mw{0.0};
  double pmin_mw{0.0};

  double k_droop{0.0};

  // Power-stage topology and duty-ratio feasibility window (multi-converter
  // model §3.2).  topology selects the ideal CCM voltage-conversion law used to
  // back out the duty ratio D from the solved port voltages; D must stay within
  // [d_min, d_max].  n_ratio is the transformer turns ratio for the Isolated
  // topology.  Generic topology imposes no duty constraint (legacy behavior).
  DCDCTopology topology{DCDCTopology::Generic};
  double d_min{0.05};
  double d_max{0.95};
  double n_ratio{1.0};

  bool controllable{true};
  double f_switching_hz{0.0};

  // Averaged PWM switching-function model for Buck, Boost and Isolated
  // topologies. Generic topology is rejected when this model is enabled.
  bool harmonic_model_enabled{false};
  double harmonic_transfer_conductance_pu{0.0};
  double duty_ratio{0.0};
  double inductance_h{0.0};
  double inductor_resistance_ohm{0.0};
  double input_capacitance_f{0.0};
  double output_capacitance_f{0.0};
  double capacitor_esr_ohm{0.0};
  HarmonicPIController voltage_controller;

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Energy Router Port (one port of a multi-port converter)
// ═══════════════════════════════════════════════════════════════════════
struct EnergyRouterPort {
  int index{0};
  std::string name;
  int bus{0};
  ERPortType port_type{ERPortType::AC};
  int side{0};

  double voltage_level_kv{0.0};
  double p_mw{0.0};
  double q_mvar{0.0};
  double v_pu{1.0};
  double eta{0.98};

  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  ERControlMode control_mode{ERControlMode::PQ};
  double p_set_mw{0.0};
  double q_set_mvar{0.0};
  double v_set_pu{1.0};
  bool in_service{true};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// Energy Router (multi-port power electronic converter)
// ═══════════════════════════════════════════════════════════════════════
struct EnergyRouter {
  int index{0};
  std::string name;
  bool in_service{true};

  std::string router_type;
  int num_ports{0};
  std::vector<EnergyRouterPort> ports;

  double p_rated_mw{0.0};
  double vn_ac_kv{0.0};
  double vn_dc_kv{0.0};
  double loss_percent{0.0};

  std::string control_mode;
  std::string power_dispatch_strategy;
  double t_scheduled_hr{0.0};

  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};

  DynamicModelProfile dynamic_model;
};

}  // namespace hacdcpf

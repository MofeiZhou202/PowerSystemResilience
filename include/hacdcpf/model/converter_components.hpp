#pragma once

/// model/converter_components.hpp
/// ================================
/// VSC converters, DC-DC converters, and energy router types.
/// Replaces: converters.hpp.

#include <string>
#include <vector>

#include "hacdcpf/model/enums/converter_enums.hpp"

namespace hacdcpf {

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
  // AC terminal angle reference (degrees) used when the converter forms the AC
  // voltage reference (AC_GRID_FORMING / Mode 1).  Defines the slack angle of
  // the AC island it energizes; ignored by every other control mode.
  double v_ac_angle_set_deg{0.0};

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

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
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
};

}  // namespace hacdcpf

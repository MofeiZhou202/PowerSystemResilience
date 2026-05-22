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
  double q_set_mvar{0.0};
  double v_dc_set_pu{1.0};
  double v_ac_set_pu{1.0};

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

  double k_p{0.0};
  double k_q{0.0};
  double v_ref_pu{1.0};
  double f_ref_hz{50.0};

  bool controllable{true};

  double r_sc_pu{0.0};
  double x_sc_pu{0.15};
  double r2_sc_pu{0.0};
  double x2_sc_pu{0.0};
  double i_max_pu{1.0};
  bool grid_forming{false};

  std::string name;

  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
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

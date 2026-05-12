#pragma once

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
  std::string type;                 // topology: "two_level", "mmc", "npc"

  double p_set_mw{0.0};            // active power setpoint (bus injection convention:
                                    //   positive = injection into AC bus, negative = withdrawal)
  double q_set_mvar{0.0};          // reactive power setpoint (bus injection convention:
                                    //   positive = injection into AC bus, negative = absorption)
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
  double vn_ac_kv{0.0};             // rated AC voltage
  double vn_dc_kv{0.0};             // rated DC voltage

  // Droop parameters (grid-forming mode)
  double k_p{0.0};                  // active power droop gain (%/Hz)
  double k_q{0.0};                  // reactive power droop gain (%/kV)
  double v_ref_pu{1.0};             // voltage reference
  double f_ref_hz{50.0};            // frequency reference

  bool controllable{true};

  // Short-circuit impedance (IEC 60909)
  double r_sc_pu{0.0};              // positive-seq SC resistance (pu on s_rated)
  double x_sc_pu{0.15};             // positive-seq SC reactance (pu on s_rated)
  double r2_sc_pu{0.0};             // negative-seq SC resistance (pu) — differs from Z1 for converters
  double x2_sc_pu{0.0};             // negative-seq SC reactance (pu) — 0 means use Z1
  double i_max_pu{1.0};             // max fault current injection (pu on I_rated)
  bool grid_forming{false};          // true = voltage-source model; false = current-source model

  std::string name;

  // Reliability
  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC-DC Converter
// ═══════════════════════════════════════════════════════════════════════
// DC-DC Converter
//
// Topology: bus_in and bus_out are two DC buses connected by the converter.
// For unidirectional converters the naming is physical (input → output).
// For bidirectional converters (e.g., BESS interfacing) the naming is a
// labelling convention — either assignment is correct.  The sign of
// p_ref_mw (or OPF variable p_in) determines the actual power flow
// direction at runtime:
//   p_ref > 0  →  forward:  bus_in supplies power, bus_out receives.
//   p_ref < 0  →  reverse:  bus_out supplies power, bus_in receives.
// Tip: for BESS, a common convention is bus_in = grid bus, bus_out =
//      battery bus, so p_ref > 0 means charging and p_ref < 0 discharging.
//      But the opposite is equally valid — just be consistent.
// ═══════════════════════════════════════════════════════════════════════
struct DCDCConverter {
  int index{0};
  int bus_in{0};                   // first DC bus  (labelled "input")
  int bus_out{0};                  // second DC bus (labelled "output")
  bool in_service{true};
  std::string name;

  DCDCControlMode control_mode{DCDCControlMode::Voltage};

  // Sign convention (正方向):
  //   p_ref_mw is OUTPUT-referenced (specifies the power delivered to bus_out).
  //   Positive direction: bus_in → bus_out.
  //   When p_ref_mw > 0 (forward):  bus_in supplies p_ref/η,  bus_out receives p_ref.
  //   When p_ref_mw < 0 (reverse):  bus_out supplies |p_ref|/η,  bus_in receives |p_ref|.
  //   The OPF variable is INPUT-referenced (p_in): p_in = p_ref / η (forward).
  double p_ref_mw{0.0};
  double v_ref_pu{1.0};           // output voltage reference
  double sn_mva{0.0};             // rated power
  double vn_in_kv{0.0};           // rated input voltage
  double vn_out_kv{0.0};          // rated output voltage
  double eta{0.98};               // efficiency
  double r_eq_pu{0.0};            // equivalent series resistance

  double pmax_mw{0.0};
  double pmin_mw{0.0};

  double k_droop{0.0};            // droop gain for droop mode

  bool controllable{true};
  double f_switching_hz{0.0};      // switching frequency

  // Reliability
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
  int bus{0};                      // connected AC bus
  ERPortType port_type{ERPortType::AC};
  int side{0};                     // 0 = Side A (left), 1 = Side B (right)

  double voltage_level_kv{0.0};
  double p_mw{0.0};               // active power (bus injection: positive = into bus)
  double q_mvar{0.0};             // reactive power (bus injection: positive = into bus)
  double v_pu{1.0};
  double eta{0.98};               // VSC efficiency for this port

  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  ERControlMode control_mode{ERControlMode::PQ};
  double p_set_mw{0.0};           // active power setpoint (bus injection: positive = into bus)
  double q_set_mvar{0.0};         // reactive power setpoint (bus injection: positive = into bus)
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

  std::string router_type;         // e.g. "SST", "UPFC"
  int num_ports{0};
  std::vector<EnergyRouterPort> ports;

  double p_rated_mw{0.0};
  double vn_ac_kv{0.0};
  double vn_dc_kv{0.0};
  double loss_percent{0.0};

  // Control strategy
  std::string control_mode;            // e.g. "autonomous", "centralized"
  std::string power_dispatch_strategy; // e.g. "priority", "proportional"
  double t_scheduled_hr{0.0};

  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
};

}  // namespace hacdcpf

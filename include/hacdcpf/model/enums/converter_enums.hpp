#pragma once

namespace hacdcpf {

// VSC converter control modes
enum class ConverterMode {
  PQ_MODE,
  VDC_Q,
  VDC_VAC,
  // AC-side PV control (multi-converter model r1 §1, VSC Mode 2): the converter
  // holds its AC terminal active power and AC voltage magnitude, releasing AC
  // reactive power as a free device unknown. It does NOT form the AC angle
  // reference. Appended last to keep the existing enum values stable.
  AC_PV,
  // AC-side grid-forming (multi-converter model r1 §2/§4.2, VSC Mode 1: δs+Vs):
  // the converter forms the AC voltage reference (angle + magnitude) at its AC
  // terminal — its AC bus is a slack bus and its power balances the AC island.
  AC_GRID_FORMING,
  // DC-voltage droop + AC voltage magnitude (multi-converter model r1 §4.7, VSC
  // Mode 6): DC power follows the Vdc droop (like VDC_VAC) while the AC terminal
  // voltage magnitude is held (PV bus), releasing AC reactive power.
  DC_V_DROOP_AC_V,
};

// Steady-state VSC AC-current limiting policy.  These policies are part of the
// balanced positive-sequence power-flow contract and are intentionally kept
// separate from the transient CurrentLimiterKind taxonomy.
enum class VSCCurrentLimitPriority {
  Magnitude = 0,
  ActivePower = 1,
  ReactivePower = 2,
};

// VSC seven-mode control taxonomy (multi-converter model r1 §6.1).  This is the
// human-facing classification of the seven typical VSC steady-state control
// modes; each maps onto a ConverterMode + AC-bus treatment for the power flow.
//   Mode 1 δs+Vs            -> AC_GRID_FORMING        (AC slack)
//   Mode 2 Ps+Vs            -> AC_PV                  (AC PV bus, P fixed)
//   Mode 3 Ps+Qs            -> AC_PQ                  (PQ injection)
//   Mode 4 Udc+Qs           -> DC_V_AC_Q             (Vdc droop/rigid, Q fixed)
//   Mode 5 Udc+Vs           -> DC_V_AC_V             (Vdc droop/rigid, V held)
//   Mode 6 Droop Udc+Vs     -> DC_V_DROOP_AC_V       (Vdc droop, V held)
//   Mode 7 Droop Udc+Qs     -> DC_V_DROOP_AC_Q       (Vdc droop, Q fixed)
enum class ACDCControlMode {
  AC_PQ,              // Mode 3
  AC_PV,              // Mode 2
  DC_V_AC_Q,          // Mode 4
  DC_V_AC_V,          // Mode 5
  DC_V_DROOP_AC_Q,    // Mode 7
  DC_V_DROOP_AC_V,    // Mode 6
  AC_GRID_FORMING,    // Mode 1
};

// Converter loss model
enum class LossModelType {
  Linear,
  CurrentBased,
};

// LCC (line-commutated converter) station role in a two-terminal HVDC link.
// For BPA/DSP imports the role follows the LD card terminal order: the first
// terminal is the rectifier, the second the inverter.
enum class LCCStationRole {
  Rectifier = 0,
  Inverter = 1,
};

// LCC quasi-steady control mode (BPA/DSP convention, dat card manual §4):
//   ConstantPower  — rectifier regulates the DC power at the control point
//                    (firing angle alpha is the free variable).
//   ConstantCurrent— constant DC current (current regulator).
//   ConstantAlpha  — constant firing angle (rectifier backup mode).
//   ConstantGamma  — constant extinction angle (CEA, the normal inverter mode).
enum class LCCControlMode {
  ConstantPower = 0,
  ConstantCurrent = 1,
  ConstantAlpha = 2,
  ConstantGamma = 3,
};

// DC-DC converter control mode
enum class DCDCControlMode {
  Voltage = 0,
  Power = 1,
  Droop = 2,
};

// DC-DC converter power-stage topology (sets the duty-ratio feasibility model,
// multi-converter model §3.2).  Generic keeps the legacy behavior with no
// duty-ratio constraint.
enum class DCDCTopology {
  Generic = 0,
  Buck = 1,
  Boost = 2,
  BuckBoost = 3,
  Isolated = 4,
};

// Energy router port type
enum class ERPortType {
  AC = 0,
  DC = 1,
};

// Energy router control mode (port-level)
enum class ERControlMode {
  PQ = 0,
  VF = 1,       // voltage-frequency (grid-forming)
  Droop = 2,
};

}  // namespace hacdcpf

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
};

// Converter loss model
enum class LossModelType {
  Linear,
  CurrentBased,
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

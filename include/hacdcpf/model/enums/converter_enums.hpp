#pragma once

namespace hacdcpf {

// VSC converter control modes
enum class ConverterMode {
  PQ_MODE,
  VDC_Q,
  VDC_VAC,
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

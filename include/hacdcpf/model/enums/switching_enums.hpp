#pragma once

namespace hacdcpf {

// Switch type
enum class SwitchType {
  CircuitBreaker = 0,
  Disconnector = 1,
  LoadBreakSwitch = 2,
  Fuse = 3,
  Recloser = 4,
  Sectionalizer = 5,
};

// Breaker type (matching Julia HighVoltageCircuitBreaker.type)
enum class BreakerType {
  CB = 0,   // Circuit Breaker
  LS = 1,   // Load Switch
  DS = 2,   // Disconnector (Isolating Switch)
};

}  // namespace hacdcpf

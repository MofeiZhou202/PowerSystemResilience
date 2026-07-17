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
  Unknown = 6,
};

// Functional role in protection and service-restoration studies.  The role is
// deliberately separate from SwitchType: a circuit breaker may be a feeder
// protection device, a bus coupler, or a normally-open tie.
enum class SwitchRole {
  Unspecified = 0,
  Protection = 1,
  Sectionalizing = 2,
  Tie = 3,
  Isolation = 4,
  Grounding = 5,
};

enum class SwitchOperatingMode {
  Manual = 0,
  Remote = 1,
  Automatic = 2,
};

// Breaker type
enum class BreakerType {
  CB = 0,   // Circuit Breaker
  LS = 1,   // Load Switch
  DS = 2,   // Disconnector (Isolating Switch)
};

}  // namespace hacdcpf

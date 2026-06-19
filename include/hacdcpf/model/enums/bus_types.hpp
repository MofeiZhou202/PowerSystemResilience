#pragma once

namespace hacdcpf {

// AC bus types
enum class BusType {
  PQ = 1,
  PV = 2,
  SLACK = 3,
  ISOLATED = 4,
};

// DC bus types
enum class DCBusType {
  DC_P = 1,         // DC active-power controlled bus (P bus)
  DC_V = 3,         // DC voltage-controlled bus (V bus / reference)
  DC_ISOLATED = 4,  // Disconnected/de-energized DC bus (removed from solve)
};

}  // namespace hacdcpf

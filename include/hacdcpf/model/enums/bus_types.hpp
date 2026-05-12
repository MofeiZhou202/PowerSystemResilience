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
  DC_P = 1,
  DC_V = 3,
};

}  // namespace hacdcpf

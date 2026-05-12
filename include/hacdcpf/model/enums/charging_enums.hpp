#pragma once

namespace hacdcpf {

// Charger type
enum class ChargerType {
  AC_L1 = 0,    // Level 1 AC (<=3.7 kW)
  AC_L2 = 1,    // Level 2 AC (<=22 kW)
  DC_Fast = 2,  // DC fast (<=350 kW)
};

}  // namespace hacdcpf

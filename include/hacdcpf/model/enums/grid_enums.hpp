#pragma once

namespace hacdcpf {

// Microgrid operating mode
enum class MicrogridMode {
  GridConnected = 0,
  Islanded = 1,
  Transition = 2,
};

}  // namespace hacdcpf

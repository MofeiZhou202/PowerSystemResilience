#pragma once

namespace hacdcpf {

// Storage operating mode
enum class StorageMode {
  Idle = 0,
  Charging = 1,
  Discharging = 2,
};

// Mobile storage status
enum class MobileStorageStatus {
  Stationary = 0,
  InTransit = 1,
  Deployed = 2,
};

}  // namespace hacdcpf

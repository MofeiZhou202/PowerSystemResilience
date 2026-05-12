#pragma once

namespace hacdcpf {

// Load models
enum class LoadModel {
  ConstantPower = 0,  // PQ (default)
  ZIP = 1,            // polynomial voltage-dependent
  Exponential = 2,    // P = P0 * V^alpha
};

// Load priority for shedding
enum class LoadPriority {
  Low = 1,
  Medium = 2,
  High = 3,
  Critical = 4,
};

}  // namespace hacdcpf

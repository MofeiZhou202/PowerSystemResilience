#pragma once

#include <map>
#include <string>

namespace hacdcpf::dynamics {

enum class DynamicEventType {
  ACBranchTrip,
  ACBranchClose,
  ACBranchImpedanceScale,
  DCBranchTrip,
  DCBranchClose,
  ACLoadScale,
  DCLoadScale,
  GeneratorTrip,
  VSCTrip,
  DCDCTrip,
  StoragePowerStep,
  DCStoragePowerStep,
  FaultShunt,
  ClearFault,
  Custom
};

struct DynamicEvent {
  double time_s{0.0};
  DynamicEventType type{DynamicEventType::Custom};
  int component_index{0};
  int target_id{0};
  int bus{0};
  int phase{-1};
  double value{0.0};
  double duration_s{0.0};
  std::string component_type;
  std::string target_type;
  std::string label;
  std::map<std::string, double> params;
  bool applied{false};
};

}  // namespace hacdcpf::dynamics

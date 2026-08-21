#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace mipsolvers::l2o {

enum class TraceEventType {
  TraceHeader,
  InstanceStart,
  RootLp,
  BranchDecision,
  NodeSelected,
  CutRound,
  Incumbent,
  SolveEnd,
  Custom
};

struct TraceEvent {
  TraceEventType type{TraceEventType::Custom};
  std::uint64_t sequence{0};
  std::string run_id;
  std::string instance_id;
  double wall_time_sec{0.0};
  nlohmann::json payload = nlohmann::json::object();
};

std::string to_string(TraceEventType type);
TraceEventType trace_event_type_from_string(std::string_view value);

void to_json(nlohmann::json& j, const TraceEvent& event);
void from_json(const nlohmann::json& j, TraceEvent& event);

TraceEvent make_trace_event(
    TraceEventType type,
    nlohmann::json payload = nlohmann::json::object(),
    std::string instance_id = {},
    double wall_time_sec = 0.0);

}  // namespace mipsolvers::l2o
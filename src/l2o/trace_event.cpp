#include "mipsolvers/l2o/trace_event.hpp"

#include <stdexcept>
#include <utility>

namespace mipsolvers::l2o {
namespace {

std::string lower_copy(std::string_view value) {
  std::string out(value);
  for (char& c : out) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return out;
}

}  // namespace

std::string to_string(TraceEventType type) {
  switch (type) {
    case TraceEventType::TraceHeader: return "trace_header";
    case TraceEventType::InstanceStart: return "instance_start";
    case TraceEventType::RootLp: return "root_lp";
    case TraceEventType::BranchDecision: return "branch_decision";
    case TraceEventType::NodeSelected: return "node_selected";
    case TraceEventType::CutRound: return "cut_round";
    case TraceEventType::Incumbent: return "incumbent";
    case TraceEventType::SolveEnd: return "solve_end";
    case TraceEventType::Custom: return "custom";
  }
  return "custom";
}

TraceEventType trace_event_type_from_string(std::string_view value) {
  const std::string v = lower_copy(value);
  if (v == "trace_header") return TraceEventType::TraceHeader;
  if (v == "instance_start") return TraceEventType::InstanceStart;
  if (v == "root_lp") return TraceEventType::RootLp;
  if (v == "branch_decision") return TraceEventType::BranchDecision;
  if (v == "node_selected") return TraceEventType::NodeSelected;
  if (v == "cut_round") return TraceEventType::CutRound;
  if (v == "incumbent") return TraceEventType::Incumbent;
  if (v == "solve_end") return TraceEventType::SolveEnd;
  if (v == "custom") return TraceEventType::Custom;
  throw std::invalid_argument("unknown L2O trace event type: " + std::string(value));
}

void to_json(nlohmann::json& j, const TraceEvent& event) {
  j = nlohmann::json{
      {"type", to_string(event.type)},
      {"sequence", event.sequence},
      {"run_id", event.run_id},
      {"instance_id", event.instance_id},
      {"wall_time_sec", event.wall_time_sec},
      {"payload", event.payload}};
}

void from_json(const nlohmann::json& j, TraceEvent& event) {
  event.type = trace_event_type_from_string(j.value("type", "custom"));
  event.sequence = j.value("sequence", std::uint64_t{0});
  event.run_id = j.value("run_id", "");
  event.instance_id = j.value("instance_id", "");
  event.wall_time_sec = j.value("wall_time_sec", 0.0);
  event.payload = j.value("payload", nlohmann::json::object());
}

TraceEvent make_trace_event(TraceEventType type,
                            nlohmann::json payload,
                            std::string instance_id,
                            double wall_time_sec) {
  TraceEvent event;
  event.type = type;
  event.instance_id = std::move(instance_id);
  event.wall_time_sec = wall_time_sec;
  event.payload = std::move(payload);
  return event;
}

}  // namespace mipsolvers::l2o
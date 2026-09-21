#include "mipsolvers/l2o/policy.hpp"

#include <stdexcept>

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

std::string to_string(PolicyMode mode) {
  switch (mode) {
    case PolicyMode::Off: return "off";
    case PolicyMode::TraceOnly: return "trace_only";
    case PolicyMode::Advisory: return "advisory";
    case PolicyMode::Guarded: return "guarded";
    case PolicyMode::Experimental: return "experimental";
  }
  return "off";
}

PolicyMode policy_mode_from_string(std::string_view value) {
  const std::string v = lower_copy(value);
  if (v == "off") return PolicyMode::Off;
  if (v == "trace_only" || v == "traceonly") return PolicyMode::TraceOnly;
  if (v == "advisory") return PolicyMode::Advisory;
  if (v == "guarded") return PolicyMode::Guarded;
  if (v == "experimental") return PolicyMode::Experimental;
  throw std::invalid_argument("unknown L2O policy mode: " + std::string(value));
}

void to_json(nlohmann::json& j, const PolicyMetadata& metadata) {
  j = nlohmann::json{
      {"name", metadata.name},
      {"version", metadata.version},
      {"model_hash", metadata.model_hash},
      {"feature_schema_version", metadata.feature_schema_version},
      {"training_distribution", metadata.training_distribution},
      {"max_inference_time_sec", metadata.max_inference_time_sec},
      {"mode", to_string(metadata.mode)},
      {"metadata", metadata.metadata}};
}

void from_json(const nlohmann::json& j, PolicyMetadata& metadata) {
  metadata.name = j.value("name", "noop");
  metadata.version = j.value("version", "0");
  metadata.model_hash = j.value("model_hash", "");
  metadata.feature_schema_version = j.value("feature_schema_version", "");
  metadata.training_distribution = j.value("training_distribution", "");
  metadata.max_inference_time_sec = j.value("max_inference_time_sec", 0.0);
  metadata.mode = policy_mode_from_string(j.value("mode", "off"));
  metadata.metadata = j.value("metadata", nlohmann::json::object());
}

}  // namespace mipsolvers::l2o
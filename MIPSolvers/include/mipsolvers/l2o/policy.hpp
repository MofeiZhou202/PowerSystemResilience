#pragma once

#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace mipsolvers::l2o {

enum class PolicyMode {
  Off,
  TraceOnly,
  Advisory,
  Guarded,
  Experimental
};

struct PolicyMetadata {
  std::string name{"noop"};
  std::string version{"0"};
  std::string model_hash;
  std::string feature_schema_version;
  std::string training_distribution;
  double max_inference_time_sec{0.0};
  PolicyMode mode{PolicyMode::Off};
  nlohmann::json metadata = nlohmann::json::object();
};

std::string to_string(PolicyMode mode);
PolicyMode policy_mode_from_string(std::string_view value);

void to_json(nlohmann::json& j, const PolicyMetadata& metadata);
void from_json(const nlohmann::json& j, PolicyMetadata& metadata);

}  // namespace mipsolvers::l2o
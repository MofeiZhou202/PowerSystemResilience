#pragma once

#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace mipsolvers::l2o {

enum class FeatureTarget {
  Instance,
  Variable,
  Row,
  Edge,
  BranchCandidate,
  Node,
  CutCandidate,
  Artifact
};

enum class FeatureDType {
  Float64,
  Int64,
  Boolean,
  Categorical,
  String
};

struct FeatureSpec {
  std::string name;
  FeatureTarget target{FeatureTarget::Instance};
  FeatureDType dtype{FeatureDType::Float64};
  std::string description;
  std::string unit;
  bool required{true};
  nlohmann::json default_value = nullptr;
  std::vector<std::string> categories;
};

struct FeatureSchema {
  std::string name{"mipsolvers.l2o.features"};
  int version{1};
  std::vector<FeatureSpec> features;
};

std::string to_string(FeatureTarget target);
std::string to_string(FeatureDType dtype);
FeatureTarget feature_target_from_string(std::string_view value);
FeatureDType feature_dtype_from_string(std::string_view value);

void to_json(nlohmann::json& j, const FeatureSpec& spec);
void from_json(const nlohmann::json& j, FeatureSpec& spec);
void to_json(nlohmann::json& j, const FeatureSchema& schema);
void from_json(const nlohmann::json& j, FeatureSchema& schema);

std::vector<std::string> validate_schema(const FeatureSchema& schema);
bool schema_has_feature(const FeatureSchema& schema,
                        FeatureTarget target,
                        std::string_view name);

}  // namespace mipsolvers::l2o
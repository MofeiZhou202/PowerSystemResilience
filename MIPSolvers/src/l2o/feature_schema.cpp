#include "mipsolvers/l2o/feature_schema.hpp"

#include <set>
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

std::string to_string(FeatureTarget target) {
  switch (target) {
    case FeatureTarget::Instance: return "instance";
    case FeatureTarget::Variable: return "variable";
    case FeatureTarget::Row: return "row";
    case FeatureTarget::Edge: return "edge";
    case FeatureTarget::BranchCandidate: return "branch_candidate";
    case FeatureTarget::Node: return "node";
    case FeatureTarget::CutCandidate: return "cut_candidate";
    case FeatureTarget::Artifact: return "artifact";
  }
  return "instance";
}

std::string to_string(FeatureDType dtype) {
  switch (dtype) {
    case FeatureDType::Float64: return "float64";
    case FeatureDType::Int64: return "int64";
    case FeatureDType::Boolean: return "bool";
    case FeatureDType::Categorical: return "categorical";
    case FeatureDType::String: return "string";
  }
  return "float64";
}

FeatureTarget feature_target_from_string(std::string_view value) {
  const std::string v = lower_copy(value);
  if (v == "instance") return FeatureTarget::Instance;
  if (v == "variable") return FeatureTarget::Variable;
  if (v == "row") return FeatureTarget::Row;
  if (v == "edge") return FeatureTarget::Edge;
  if (v == "branch_candidate") return FeatureTarget::BranchCandidate;
  if (v == "node") return FeatureTarget::Node;
  if (v == "cut_candidate") return FeatureTarget::CutCandidate;
  if (v == "artifact") return FeatureTarget::Artifact;
  throw std::invalid_argument("unknown L2O feature target: " + std::string(value));
}

FeatureDType feature_dtype_from_string(std::string_view value) {
  const std::string v = lower_copy(value);
  if (v == "float64" || v == "double") return FeatureDType::Float64;
  if (v == "int64" || v == "integer") return FeatureDType::Int64;
  if (v == "bool" || v == "boolean") return FeatureDType::Boolean;
  if (v == "categorical") return FeatureDType::Categorical;
  if (v == "string") return FeatureDType::String;
  throw std::invalid_argument("unknown L2O feature dtype: " + std::string(value));
}

void to_json(nlohmann::json& j, const FeatureSpec& spec) {
  j = nlohmann::json{
      {"name", spec.name},
      {"target", to_string(spec.target)},
      {"dtype", to_string(spec.dtype)},
      {"description", spec.description},
      {"unit", spec.unit},
      {"required", spec.required},
      {"default_value", spec.default_value},
      {"categories", spec.categories}};
}

void from_json(const nlohmann::json& j, FeatureSpec& spec) {
  spec.name = j.value("name", "");
  spec.target = feature_target_from_string(j.value("target", "instance"));
  spec.dtype = feature_dtype_from_string(j.value("dtype", "float64"));
  spec.description = j.value("description", "");
  spec.unit = j.value("unit", "");
  spec.required = j.value("required", true);
  spec.default_value = j.contains("default_value") ? j.at("default_value") : nlohmann::json(nullptr);
  spec.categories = j.value("categories", std::vector<std::string>{});
}

void to_json(nlohmann::json& j, const FeatureSchema& schema) {
  j = nlohmann::json{
      {"name", schema.name},
      {"version", schema.version},
      {"features", schema.features}};
}

void from_json(const nlohmann::json& j, FeatureSchema& schema) {
  schema.name = j.value("name", "mipsolvers.l2o.features");
  schema.version = j.value("version", 1);
  schema.features = j.value("features", std::vector<FeatureSpec>{});
}

std::vector<std::string> validate_schema(const FeatureSchema& schema) {
  std::vector<std::string> errors;
  if (schema.name.empty()) errors.push_back("feature schema name is empty");
  if (schema.version <= 0) errors.push_back("feature schema version must be positive");

  std::set<std::pair<FeatureTarget, std::string>> seen;
  for (const auto& feature : schema.features) {
    if (feature.name.empty()) {
      errors.push_back("feature name is empty");
      continue;
    }
    const auto key = std::make_pair(feature.target, feature.name);
    if (!seen.insert(key).second) {
      errors.push_back("duplicate feature name for target: " + to_string(feature.target) + "." + feature.name);
    }
    if (feature.dtype == FeatureDType::Categorical && feature.categories.empty()) {
      errors.push_back("categorical feature has no categories: " + feature.name);
    }
  }
  return errors;
}

bool schema_has_feature(const FeatureSchema& schema,
                        FeatureTarget target,
                        std::string_view name) {
  for (const auto& feature : schema.features) {
    if (feature.target == target && feature.name == name) return true;
  }
  return false;
}

}  // namespace mipsolvers::l2o
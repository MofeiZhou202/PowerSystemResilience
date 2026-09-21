#pragma once

#include <span>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace hacdcpf::server {

enum class Edition { Full, Trial, Resilience };
enum class EditionRouteAccess { Retained, Disabled, Unclassified };
enum class EditionRouteShape { Literal, StructuredV1 };
enum class V1AnalysisAccess { Enabled, KnownDisabled, Unknown };

struct EditionRouteRule {
  std::string_view method;
  std::string_view path_template;
  std::string_view feature;
  EditionRouteShape shape{EditionRouteShape::Literal};
  EditionRouteAccess trial_access{EditionRouteAccess::Unclassified};
  EditionRouteAccess resilience_access{EditionRouteAccess::Unclassified};
  std::string_view analysis_id{};
};

struct EditionRouteDecision {
  EditionRouteAccess access{EditionRouteAccess::Unclassified};
  std::string feature{"unclassified_api"};
};

struct V1AnalysisDecision {
  V1AnalysisAccess access{V1AnalysisAccess::Unknown};
  std::string feature{"unknown_analysis"};
};

Edition current_edition();
std::string_view edition_name(Edition edition);
bool edition_gating_enabled();
bool trial_edition_enabled();
bool resilience_edition_enabled();
nlohmann::json edition_profile_json();
nlohmann::json edition_analysis_plan_json(const nlohmann::json& request);
EditionRouteDecision classify_edition_route(std::string_view method,
                                            std::string_view path);
EditionRouteDecision classify_trial_route(std::string_view method,
                                          std::string_view path);
std::span<const EditionRouteRule> edition_route_manifest();
V1AnalysisDecision classify_v1_analysis(std::string_view analysis);
bool v1_executor_supports_analysis(std::string_view analysis);
nlohmann::json edition_analyses_json();
nlohmann::json edition_known_disabled_analyses_json();
nlohmann::json edition_analysis_catalog_json();

}  // namespace hacdcpf::server

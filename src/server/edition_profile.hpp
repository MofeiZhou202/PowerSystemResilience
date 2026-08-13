#pragma once

#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace hacdcpf::server {

enum class EditionRouteAccess { Retained, Disabled, Unclassified };

struct EditionRouteDecision {
  EditionRouteAccess access{EditionRouteAccess::Unclassified};
  std::string feature{"unclassified_api"};
};

bool trial_edition_enabled();
nlohmann::json edition_profile_json();
nlohmann::json edition_analysis_plan_json(const nlohmann::json& request);
EditionRouteDecision classify_trial_route(std::string_view method,
                                          std::string_view path);

}  // namespace hacdcpf::server

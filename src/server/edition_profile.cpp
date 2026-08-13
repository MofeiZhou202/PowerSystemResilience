#include "edition_profile.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hacdcpf::server {
namespace {

using json = nlohmann::json;

struct RoutePrefix {
  std::string_view prefix;
  std::string_view feature;
};

constexpr auto kRetainedRoutes = std::to_array<std::string_view>({
    "/api/edition",
    "/api/edition/analysis_plan",
    "/api/cases",
    "/api/matpower_files",
    "/api/io/model_compatibility",
    "/api/opf/parameter_contract",
    "/api/session/parameter_library",
    "/api/session/parameter_library/select",
    "/api/session/parameter_library/update",
    "/api/session/parameter_library/validate",
    "/api/session/parameter_library/apply",
    "/api/session/design_handbook_parameters/preview",
    "/api/session/design_handbook_parameters/apply",
    "/api/session/reliability/data_quality",
    "/api/session/reliability/configuration",
    "/api/session/reliability/configuration/validate",
    "/api/session/reliability/compare_results",
    "/api/session/run_reliability",
    "/api/session/load_builtin",
    "/api/session/load_matpower",
    "/api/session/load_json_string",
    "/api/session/load_gridlabd",
    "/api/session/load_opendss",
    "/api/session/new_empty",
    "/api/session/export_json",
    "/api/session/export_matpower",
    "/api/session/export_gridlabd",
    "/api/session/export_opendss",
    "/api/session/update_components",
    "/api/session/update_carbon_factors",
    "/api/session/cancel",
    "/api/session/status",
    "/api/session/pf",
    "/api/session/topology",
    "/api/session/opf",
    "/api/session/opf_ac",
    "/api/session/opf_parity",
    "/api/session/opf_dc",
    "/api/session/sc",
    "/api/session/sc_detailed",
    "/api/session/dc_sc",
    "/api/session/run_hosting_capacity",
    "/api/session/run_carbon",
    "/api/session/generate_scenarios",
    "/api/session/generate_typhoon_faults",
    "/api/session/run_distribution_resilience",
    "/api/session/run_reliability_nsq",
    "/api/session/run_reliability_seq",
    "/api/session/run_reliability_fmea",
    "/api/session/run_reliability_fd",
    "/api/session/run_reliability_three_stage",
    "/api/session/run_multidimensional_weak_links",
});

constexpr auto kRetainedLegacyRoutes = std::to_array<std::string_view>({
    "/api/load_case", "/api/pf", "/api/opf/ac",
    "/api/opf/dc", "/api/sc", "/api/dc_sc"});

constexpr auto kDisabledRoutes = std::to_array<RoutePrefix>({
    {"/api/dynamics/model_schema", "dynamics"},
    {"/api/dynamics/validate_profile", "dynamics"},
    {"/api/session/run_transient", "dynamics"},
    {"/api/session/transient/frame", "dynamics"},
    {"/api/session/small_signal", "dynamics"},
    {"/api/session/harmonics", "harmonics"},
    {"/api/session/harmonics_freqscan", "harmonics"},
    {"/api/session/harmonics_3ph", "harmonics"},
    {"/api/session/harmonics_metrics", "harmonics"},
    {"/api/session/harmonics_newton", "harmonics"},
    {"/api/session/rpo_inputs", "reactive_power_optimization"},
    {"/api/session/run_rpo", "reactive_power_optimization"},
    {"/api/session/set_ts_config", "time_series"},
    {"/api/session/run_ts_pf", "time_series"},
    {"/api/session/tspf/frame", "time_series"},
    {"/api/session/run_annual_sim", "time_series"},
    {"/api/session/run_lifecycle_sim", "time_series"},
    {"/api/session/run_lifecycle_compare", "time_series"},
    {"/api/session/run_dynamic_carbon", "time_series"},
    {"/api/session/run_uc", "market"},
    {"/api/session/run_market_clearing", "market"},
    {"/api/session/run_real_time_market", "market"},
    {"/api/session/run_repeated_market_game", "market"},
    {"/api/session/run_campus_ies", "integrated_energy"},
    {"/api/session/run_ev_traffic", "ev_traffic"},
    {"/api/session/run_reconfig", "network_reconfiguration"},
    {"/api/session/network_reduction", "network_reconfiguration"},
    {"/api/session/run_counterfactual_planning", "counterfactual_planning"},
    {"/api/session/sppt_guard", "sppt_agent"},
    {"/api/session/sppt_agent", "sppt_agent"},
    {"/api/session/load_bpa_dat", "advanced_io"},
    {"/api/session/export_bpa_dat", "advanced_io"},
    {"/api/session/load_etap_xml", "advanced_io"},
    {"/api/session/load_etap_xlsx", "advanced_io"},
});

constexpr auto kDisabledIoRoutes = std::to_array<RoutePrefix>({
    {"/api/session/export_etap", "advanced_io"},
    {"/api/session/export_etap_xml", "advanced_io"},
    {"/api/session/load_cim_dist", "advanced_io"},
    {"/api/session/export_cim_dist", "advanced_io"},
    {"/api/session/load_svg_distribution", "advanced_io"},
    {"/api/session/export_svg_distribution", "advanced_io"},
    {"/api/session/load_powersimulationsdynamics_julia", "advanced_io"},
    {"/api/session/export_powersimulationsdynamics", "advanced_io"},
    {"/api/session/export_powersimulationsdynamics_julia", "advanced_io"},
    {"/api/session/validate_scenario_bundle", "advanced_io"},
    {"/api/session/import_scenario_workbook", "advanced_io"},
    {"/api/session/export_scenario_workbook", "advanced_io"},
});

bool is_identifier(std::string_view value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return std::isalnum(c) || c == '_' || c == '-';
         });
}

bool is_decimal(std::string_view value) {
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char c) {
           return std::isdigit(c);
         });
}

std::vector<std::string_view> path_segments(std::string_view path) {
  std::vector<std::string_view> segments;
  std::size_t begin = 1;
  while (begin < path.size()) {
    const auto end = path.find('/', begin);
    segments.push_back(path.substr(begin, end - begin));
    if (end == std::string_view::npos) break;
    begin = end + 1;
  }
  return segments;
}

bool matches_retained_v1_route(std::string_view method, std::string_view path) {
  if (path == "/api/v1") return method == "GET";
  const auto segments = path_segments(path);
  if (segments.size() < 3 || segments[0] != "api" || segments[1] != "v1") {
    return false;
  }
  if (segments[2] == "sessions") {
    if (segments.size() == 3) return method == "GET" || method == "POST";
    if (!is_identifier(segments[3])) return false;
    if (segments.size() == 4) return method == "GET" || method == "DELETE";
    if (segments.size() != 5) return false;
    if (segments[4] == "model") return method == "GET" || method == "PUT";
    if (segments[4] == "topology" || segments[4] == "subgraph") {
      return method == "GET";
    }
    if (segments[4] == "jobs") return method == "GET" || method == "POST";
    return false;
  }
  if (segments[2] != "jobs" || segments.size() < 4 ||
      !is_identifier(segments[3])) {
    return false;
  }
  if (segments.size() == 4) return method == "GET" || method == "DELETE";
  if (segments.size() == 5 && segments[4] == "violations") {
    return method == "GET";
  }
  if (segments.size() == 5 && segments[4] == "cancel") {
    return method == "POST";
  }
  return segments.size() == 6 && segments[4] == "frames" &&
         is_decimal(segments[5]) && method == "GET";
}

template <typename Range>
EditionRouteDecision disabled_match(std::string_view path,
                                    const Range& routes) {
  for (const auto& item : routes) {
    if (path == item.prefix) {
      return {EditionRouteAccess::Disabled, std::string(item.feature)};
    }
  }
  return {};
}

json workflow_json() {
  return json::array({
      {{"id", "modeling"}, {"label", "模型建立"}},
      {{"id", "parameter_validation"}, {"label", "参数校核"}},
      {{"id", "indicator_design"}, {"label", "指标设计"}},
      {{"id", "panoramic_simulation"}, {"label", "全景仿真"}},
      {{"id", "weak_link_identification"}, {"label", "薄弱辨识"}},
  });
}

json indicators_json() {
  return json::array({
      {{"id", "system_economic"}, {"level", "system"},
       {"dimension", "economic"}, {"label", "系统经济性"}},
      {{"id", "user_economic"}, {"level", "user"},
       {"dimension", "economic"}, {"label", "用户经济性"}},
      {{"id", "system_carbon"}, {"level", "system"},
       {"dimension", "carbon"}, {"label", "系统碳指标"}},
      {{"id", "user_carbon"}, {"level", "user"},
       {"dimension", "carbon"}, {"label", "用户碳指标"}},
      {{"id", "system_reliability"}, {"level", "system"},
       {"dimension", "reliability"}, {"label", "系统可靠性"}},
      {{"id", "user_reliability"}, {"level", "user"},
       {"dimension", "reliability"}, {"label", "用户可靠性"}},
      {{"id", "system_resilience"}, {"level", "system"},
       {"dimension", "resilience"}, {"label", "系统弹性"}},
      {{"id", "user_resilience"}, {"level", "user"},
       {"dimension", "resilience"}, {"label", "用户弹性"}},
  });
}

}  // namespace

bool trial_edition_enabled() {
#ifdef HACDCPF_TRIAL_EDITION
  return true;
#else
  return false;
#endif
}

json edition_profile_json() {
  json profile{{"schema", "hacdcpf.edition-profile.v1"},
               {"edition", trial_edition_enabled() ? "trial" : "full"},
               {"product_name", trial_edition_enabled()
                                    ? "HySim-XJTU-HRPES Trial"
                                    : "HySim-XJTU-HRPES"},
               {"workflow", workflow_json()},
               {"indicators", indicators_json()}};
  if (!trial_edition_enabled()) {
    profile["enabled_modules"] = json::array({"*"});
    profile["frontend_modules"] = json::array({"*"});
    profile["enabled_io_formats"] = json::array({"*"});
    profile["disabled_features"] = json::array();
    return profile;
  }
  profile["enabled_modules"] = json::array(
      {"model_io", "parameter_validation", "topology_analysis",
       "indicator_design", "scenario_generation", "power_flow", "opf",
       "line_loss", "carbon_flow", "voltage_compliance", "hosting_capacity",
       "reliability", "resilience", "short_circuit",
       "multidimensional_weak_links"});
  profile["frontend_modules"] = json::array(
      {"modelIO", "parameterLibrary", "topologyAnalysis", "indicatorDesign",
       "scenarioGeneration", "powerFlow", "opf", "shortCircuit", "hosting",
       "reliability", "resilience", "carbonFlow", "weakLinks"});
  profile["enabled_io_formats"] =
      json::array({"json", "matpower", "gridlabd", "opendss"});
  profile["disabled_features"] = json::array(
      {"reactive_power_optimization", "harmonics", "dynamics", "market",
       "integrated_energy", "ev_traffic", "time_series",
       "network_reconfiguration", "counterfactual_planning", "sppt_agent",
       "advanced_io"});
  profile["route_policy"] =
      {{"mode", "fail_closed"}, {"unknown_api_routes", "disabled"}};
  return profile;
}

json edition_analysis_plan_json(const json& request) {
  if (!request.is_object()) {
    throw std::invalid_argument("analysis plan request must be an object");
  }
  const json selected = request.value("indicators", json::array());
  if (!selected.is_array()) {
    throw std::invalid_argument("indicators must be an array");
  }
  std::set<std::string> dimensions;
  for (const auto& value : selected) {
    if (!value.is_string()) {
      throw std::invalid_argument("indicator entries must be strings");
    }
    const std::string indicator_id = value.get<std::string>();
    const auto separator = indicator_id.find('_');
    if (separator == std::string::npos ||
        (indicator_id.substr(0, separator) != "system" &&
         indicator_id.substr(0, separator) != "user")) {
      throw std::invalid_argument("unsupported indicator: " + indicator_id);
    }
    std::string indicator = indicator_id.substr(separator + 1);
    if (indicator != "economic" && indicator != "carbon" &&
        indicator != "reliability" && indicator != "resilience") {
      throw std::invalid_argument("unsupported indicator: " + indicator_id);
    }
    dimensions.insert(std::move(indicator));
  }

  json steps = json::array();
  auto add = [&steps](const char* module, const char* label,
                      const char* scenario) {
    steps.push_back({{"module", module}, {"label", label},
                     {"scenario_family", scenario},
                     {"execution", "user_confirmed"}});
  };
  if (dimensions.contains("economic") || dimensions.contains("carbon")) {
    add("powerFlow", "潮流/线损/电压", "normal");
  }
  if (dimensions.contains("economic")) {
    add("opf", "最优潮流", "normal");
    add("hosting", "承载力", "normal");
  }
  if (dimensions.contains("carbon")) {
    add("carbonFlow", "碳流", "normal");
  }
  if (dimensions.contains("reliability") || dimensions.contains("resilience")) {
    add("scenarioGeneration", "场景生成", "fault");
  }
  if (dimensions.contains("reliability")) {
    add("reliability", "可靠性", "expected_fault");
  }
  if (dimensions.contains("resilience")) {
    add("resilience", "弹性", "unexpected_fault");
  }
  if (!dimensions.empty()) add("shortCircuit", "短路", "fault_evidence");
  if (dimensions.size() > 1) {
    add("weakLinks", "多维薄弱环节", "cross_scenario");
  }
  return {{"schema", "hacdcpf.edition-analysis-plan.v1"},
          {"indicators", selected},
          {"steps", std::move(steps)},
          {"automatic_execution", false},
          {"model_scope",
           "The plan orders existing analyses; each solver run requires explicit user confirmation."}};
}

EditionRouteDecision classify_trial_route(std::string_view method,
                                          std::string_view path) {
  if (method == "OPTIONS") return {EditionRouteAccess::Retained, "cors"};
  if (path != "/api" && !path.starts_with("/api/")) {
    return {EditionRouteAccess::Retained, "static_content"};
  }
  if (const auto decision = disabled_match(path, kDisabledRoutes);
      decision.access == EditionRouteAccess::Disabled) {
    return decision;
  }
  if (const auto decision = disabled_match(path, kDisabledIoRoutes);
      decision.access == EditionRouteAccess::Disabled) {
    return decision;
  }
  if (std::find(kRetainedRoutes.begin(), kRetainedRoutes.end(), path) !=
          kRetainedRoutes.end() ||
      std::find(kRetainedLegacyRoutes.begin(), kRetainedLegacyRoutes.end(),
                path) != kRetainedLegacyRoutes.end() ||
      matches_retained_v1_route(method, path)) {
    return {EditionRouteAccess::Retained, "retained"};
  }
  return {EditionRouteAccess::Unclassified, "unclassified_api"};
}

}  // namespace hacdcpf::server

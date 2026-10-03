#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <tuple>

#include "edition_profile.hpp"

using hacdcpf::server::Edition;
using hacdcpf::server::EditionRouteAccess;
using hacdcpf::server::EditionRouteShape;
using hacdcpf::server::V1AnalysisAccess;
using hacdcpf::server::classify_edition_route;
using hacdcpf::server::classify_trial_route;
using hacdcpf::server::classify_v1_analysis;
using hacdcpf::server::current_edition;
using hacdcpf::server::edition_analysis_catalog_json;
using hacdcpf::server::edition_analysis_plan_json;
using hacdcpf::server::edition_known_disabled_analyses_json;
using hacdcpf::server::edition_profile_json;
using hacdcpf::server::edition_route_manifest;
using hacdcpf::server::v1_executor_supports_analysis;

TEST_CASE("Edition profile matches the build mode", "[edition]") {
  const auto profile = edition_profile_json();
  const auto& indicators = profile.at("indicators");
#ifdef HACDCPF_TRIAL_EDITION
  CHECK(current_edition() == Edition::Trial);
  CHECK(profile.at("edition") == "trial");
  CHECK(profile.at("route_policy").at("mode") == "fail_closed");
#elif defined(HACDCPF_RESILIENCE_EDITION)
  CHECK(current_edition() == Edition::Resilience);
  CHECK(profile.at("edition") == "resilience");
  CHECK(profile.at("route_policy").at("mode") == "fail_closed");
  CHECK(profile.at("restoration_certification")
            .at("ordinary_feasibility_is_certified_safe") == false);
  CHECK(profile.at("enabled_io_formats") ==
        nlohmann::json::array({"json", "matpower"}));
  CHECK(profile.at("frontend_modules") ==
        nlohmann::json::array(
            {"modelIO", "parameterLibrary", "topologyAnalysis",
             "scenarioGeneration", "powerFlow", "opf", "resilience",
             "proactiveDefense", "rapidRecovery", "resilienceMetrics"}));
  CHECK(std::none_of(indicators.begin(), indicators.end(), [](const auto& row) {
    return row.at("dimension") == "carbon";
  }));
  CHECK(profile.at("disabled_features") ==
        nlohmann::json::array(
            {"reactive_power_optimization", "harmonics", "dynamics", "market",
             "carbon", "integrated_energy", "ev_traffic", "time_series",
             "hosting_capacity", "weak_links", "counterfactual_planning",
             "sppt_agent", "advanced_io"}));
#else
  CHECK(current_edition() == Edition::Full);
  CHECK(profile.at("edition") == "full");
  CHECK(profile.at("enabled_modules") == nlohmann::json::array({"*"}));
  CHECK_FALSE(profile.contains("route_policy"));
#endif
  CHECK(v1_executor_supports_analysis("power_flow"));
  CHECK(v1_executor_supports_analysis("optimal_power_flow"));
  CHECK_FALSE(v1_executor_supports_analysis("harmonics"));
  CHECK(profile.at("analysis_catalog") == edition_analysis_catalog_json());
#ifdef HACDCPF_RESILIENCE_EDITION
  const auto& metric_catalog = profile.at("resilience_metric_catalog");
  CHECK(metric_catalog.at("schema") == "resilience_metric_catalog_v1");
  CHECK(metric_catalog.at("definition_version") == "book_ch3_2026.2");
  REQUIRE(metric_catalog.at("entries").size() == 42);
  CHECK(profile.at("workflow") ==
        nlohmann::json::array({
            {{"id", "metric_selection"}, {"label", "指标选择"}},
            {{"id", "scenario_selection"}, {"label", "场景生成与选择"}},
            {{"id", "proactive_defense"}, {"label", "主动防御"}},
            {{"id", "rapid_recovery"}, {"label", "快速恢复"}},
            {{"id", "metric_output"}, {"label", "指标输出"}},
        }));
#endif
}

TEST_CASE("Edition route manifest is unique and strictly structured",
          "[edition]") {
  const auto manifest = edition_route_manifest();
  REQUIRE_FALSE(manifest.empty());
  std::set<std::tuple<std::string, std::string, int>> keys;
  for (const auto& rule : manifest) {
    INFO(rule.method << " " << rule.path_template);
    CHECK(keys.emplace(std::string(rule.method), std::string(rule.path_template),
                       static_cast<int>(rule.shape))
              .second);
    CHECK_FALSE(rule.method.empty());
    CHECK(rule.path_template.starts_with("/api/"));
    if (rule.shape == EditionRouteShape::StructuredV1) {
      CHECK(rule.path_template.starts_with("/api/v1"));
    } else {
      CHECK(rule.path_template.find('{') == std::string_view::npos);
      CHECK(rule.path_template.find('(') == std::string_view::npos);
    }
  }

  CHECK(classify_trial_route("GET", "/api/v1/sessions/session-1/model").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("PUT", "/api/v1/sessions/session_1/model").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("GET", "/api/v1/jobs/job-1/frames/19").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("GET", "/api/v1/jobs/job.1").access ==
        EditionRouteAccess::Unclassified);
  CHECK(classify_trial_route("GET", "/api/v1/jobs/job-1/frames/-1").access ==
        EditionRouteAccess::Unclassified);
  CHECK(classify_trial_route("GET", "/api/v1/jobs/job-1/frames/1/extra").access ==
        EditionRouteAccess::Unclassified);
  CHECK(classify_trial_route("POST", "/api/v1/jobs/job-1/frames/1").access ==
        EditionRouteAccess::Unclassified);
  CHECK(classify_trial_route("GET", "/api/session/market_forecast").feature ==
        "market");
  CHECK(classify_trial_route("GET", "/api/session/market_study").feature ==
        "market");
}

TEST_CASE("Edition analysis catalog matches canonical Python IDs",
          "[edition]") {
  static constexpr std::array<std::string_view, 69> expected_names{
      "power_flow", "optimal_power_flow", "opf_ac", "opf_parity", "opf_dc",
      "reactive_power_optimization", "rpo_inputs", "short_circuit",
      "detailed_short_circuit", "dc_short_circuit", "harmonics",
      "three_phase_harmonics", "harmonics_frequency_scan", "harmonics_hss",
      "harmonics_newton", "harmonics_metrics", "transient", "small_signal",
      "unit_commitment", "set_ts_config", "time_series_power_flow",
      "annual_production", "lifecycle_simulation", "lifecycle_compare",
      "carbon_flow", "dynamic_carbon_flow", "reliability_nonsequential",
      "reliability_sequential", "reliability_fmea", "reliability_fd",
      "reliability_three_stage", "reliability", "distribution_resilience",
      "market_clearing", "real_time_market", "repeated_market_game",
      "southern_market", "market_ptdf", "campus_ies", "ev_power_traffic",
      "reconfiguration", "hosting_capacity", "counterfactual_planning",
      "multidimensional_weak_links", "scenario_generation", "typhoon_faults",
      "sppt_guard", "topology", "network_reduction", "load_builtin",
      "load_matpower", "load_json_string", "new_empty", "update_components",
      "load_bpa_dat", "load_cim_dist", "load_gridlabd", "load_opendss",
      "load_etap_xml", "load_svg_distribution", "export_json",
      "export_matpower", "export_bpa_dat", "export_etap", "export_etap_xml",
      "export_cim_dist", "export_gridlabd", "export_opendss",
      "export_svg_distribution"};

  const auto catalog = edition_analysis_catalog_json();
  REQUIRE(catalog == nlohmann::json{{"schema", "hacdcpf.edition-analysis-catalog.v1"},
                                    {"entries", catalog.at("entries")}});
  const auto& entries = catalog.at("entries");
  REQUIRE(entries.size() == expected_names.size());
  std::set<std::string> actual_names;
  for (const auto& entry : entries) {
    INFO(entry.dump());
    CHECK(entry.size() == 4);
    CHECK(entry.contains("name"));
    CHECK(entry.contains("route"));
    CHECK(entry.contains("method"));
    CHECK(entry.contains("enabled"));
    CHECK(entry.at("method") == "POST");
    CHECK(entry.at("route").get<std::string>().starts_with("/api/session/"));
    CHECK(entry.at("enabled").is_boolean());
    CHECK(actual_names.insert(entry.at("name").get<std::string>()).second);
  }
  CHECK(actual_names ==
        std::set<std::string>(expected_names.begin(), expected_names.end()));

  const auto enabled = nlohmann::json::array(
      {"power_flow", "optimal_power_flow"});
  CHECK(hacdcpf::server::edition_analyses_json() == enabled);
  const auto disabled = edition_known_disabled_analyses_json();
  REQUIRE(disabled.is_array());
  REQUIRE(disabled.size() + enabled.size() == expected_names.size());
  std::set<std::string> v1_partition;
  for (const auto& name : enabled) {
    CHECK(v1_partition.insert(name.get<std::string>()).second);
  }
  for (const auto& name : disabled) {
    CHECK(v1_partition.insert(name.get<std::string>()).second);
  }
  CHECK(v1_partition == actual_names);
}

TEST_CASE("V1 analysis classification is tri-state and analyses stay v1-only",
          "[edition]") {
  CHECK(classify_v1_analysis("power_flow").access == V1AnalysisAccess::Enabled);
  CHECK(classify_v1_analysis("optimal_power_flow").access ==
        V1AnalysisAccess::Enabled);
  const auto disabled = classify_v1_analysis("harmonics_hss");
  CHECK(disabled.access == V1AnalysisAccess::KnownDisabled);
  CHECK(disabled.feature == "harmonics_hss");
  CHECK(classify_v1_analysis("not_a_registered_analysis").access ==
        V1AnalysisAccess::Unknown);
  CHECK(edition_profile_json().at("analyses") ==
        nlohmann::json::array({"power_flow", "optimal_power_flow"}));
}

TEST_CASE("Edition route policy uses method and path and fails closed",
          "[edition]") {
  CHECK(classify_edition_route("GET", "/xjtu/").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("GET", "/api/cases").access ==
        EditionRouteAccess::Retained);
  if (current_edition() != Edition::Full) {
    CHECK(classify_edition_route("POST", "/api/cases").access !=
          EditionRouteAccess::Retained);
    CHECK(classify_edition_route("GET", "/api/cases_extra").access !=
          EditionRouteAccess::Retained);
  }
  CHECK(classify_edition_route("POST", "/api/session/run_reliability").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("POST", "/api/session/run_reliability_fmea").access ==
        EditionRouteAccess::Retained);
  if (current_edition() == Edition::Resilience) {
    CHECK(classify_edition_route(
              "GET", "/api/session/resilience/metric_catalog")
              .access == EditionRouteAccess::Retained);
    CHECK(classify_edition_route("POST", "/api/session/resilience/metrics")
              .access == EditionRouteAccess::Retained);
    CHECK(classify_v1_analysis("resilience_metrics").access ==
          V1AnalysisAccess::Unknown);
  } else if (current_edition() == Edition::Trial) {
    CHECK(classify_edition_route(
              "GET", "/api/session/resilience/metric_catalog")
              .access == EditionRouteAccess::Disabled);
    CHECK(classify_edition_route("POST", "/api/session/resilience/metrics")
              .access == EditionRouteAccess::Disabled);
  }
  CHECK(classify_edition_route("POST", "/api/session/opf_parity").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("POST", "/api/opf/ac").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("POST", "/api/session/topology_window").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("POST", "/api/session/result_window").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("POST", "/api/v1/sessions").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_edition_route("GET", "/api/v1/jobs/job-1/frames/0").access ==
        EditionRouteAccess::Retained);
  if (current_edition() != Edition::Full) {
    CHECK(classify_edition_route("POST", "/api/v1/specialist_solver").access !=
          EditionRouteAccess::Retained);
  }

  CHECK(classify_trial_route("POST", "/api/session/run_hosting_capacity").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/session/run_reconfig").access ==
        EditionRouteAccess::Disabled);

  if (current_edition() == Edition::Full) return;

  const auto time_series =
      classify_edition_route("POST", "/api/session/set_ts_config");
  CHECK(time_series.access == EditionRouteAccess::Disabled);
  CHECK(time_series.feature == "time_series");
  CHECK(classify_edition_route("POST", "/api/session/harmonics_hss").feature ==
        "harmonics");
  CHECK(classify_edition_route("POST", "/api/session/market_ptdf").feature ==
        "market");
  CHECK(classify_edition_route("GET", "/api/session/market_forecast").feature ==
        "market");
  CHECK(classify_edition_route("POST", "/api/session/load_cim_dist").feature ==
        "advanced_io");
  CHECK(classify_edition_route("POST", "/api/session/load_gridlabd").feature ==
        "advanced_io");
  CHECK(classify_edition_route("POST", "/api/session/load_opendss").feature ==
        "advanced_io");
  CHECK(classify_edition_route("POST", "/api/session/export_gridlabd").feature ==
        "advanced_io");
  CHECK(classify_edition_route("POST", "/api/session/export_opendss").feature ==
        "advanced_io");

  const auto unknown =
      classify_edition_route("POST", "/api/session/new_specialist_solver");
  CHECK(unknown.access == EditionRouteAccess::Unclassified);
  CHECK(unknown.feature == "unclassified_api");

  if (current_edition() == Edition::Trial) {
    CHECK(classify_trial_route("POST", "/api/session/run_hosting_capacity").access ==
          EditionRouteAccess::Retained);
    CHECK(classify_edition_route("POST", "/api/session/run_reconfig").feature ==
          "network_reconfiguration");
  } else {
    CHECK(classify_edition_route("POST", "/api/session/run_hosting_capacity").feature ==
          "hosting_capacity");
    CHECK(classify_edition_route("POST", "/api/session/run_carbon").feature ==
          "carbon");
    CHECK(classify_edition_route(
              "POST", "/api/session/run_multidimensional_weak_links")
              .feature == "weak_links");
    CHECK(classify_edition_route("POST", "/api/session/run_reconfig").access ==
          EditionRouteAccess::Retained);
    CHECK(classify_edition_route("POST", "/api/session/network_reduction").access ==
          EditionRouteAccess::Retained);
  }
}

TEST_CASE("Edition analysis plan is backend ordered", "[edition]") {
  auto requested_indicators =
      nlohmann::json::array({"system_economic", "system_reliability"});
  if (current_edition() != Edition::Resilience) {
    requested_indicators.push_back("user_carbon");
  }
  const auto plan = edition_analysis_plan_json(
      {{"indicators", std::move(requested_indicators)}});
  REQUIRE(plan.at("schema") == "hacdcpf.edition-analysis-plan.v1");
  REQUIRE(plan.at("automatic_execution") == false);
  const auto& steps = plan.at("steps");
#ifdef HACDCPF_RESILIENCE_EDITION
  REQUIRE(steps.size() == 5);
  CHECK(steps.at(0).at("id") == "metric_selection");
  CHECK(steps.at(1).at("id") == "scenario_selection");
  CHECK(steps.at(2).at("id") == "proactive_defense");
  CHECK(steps.at(3).at("id") == "rapid_recovery");
  CHECK(steps.at(4).at("id") == "metric_output");
#else
  REQUIRE(steps.size() == 8);
  CHECK(steps.at(0).at("module") == "powerFlow");
  CHECK(steps.at(1).at("module") == "opf");
  CHECK(steps.at(2).at("module") == "hosting");
  CHECK(steps.at(3).at("module") == "carbonFlow");
  CHECK(steps.at(4).at("module") == "scenarioGeneration");
  CHECK(steps.at(5).at("module") == "reliability");
  CHECK(steps.at(6).at("module") == "shortCircuit");
  CHECK(steps.at(7).at("module") == "weakLinks");
#endif

  CHECK_THROWS_AS(edition_analysis_plan_json(
                      {{"indicators", {"internal_economic"}}}),
                  std::invalid_argument);
#ifdef HACDCPF_RESILIENCE_EDITION
  CHECK_THROWS_AS(edition_analysis_plan_json(
                      {{"indicators", {"user_carbon"}}}),
                  std::invalid_argument);
#endif
}

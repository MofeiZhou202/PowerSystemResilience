#include <catch2/catch_test_macros.hpp>

#include "edition_profile.hpp"

using hacdcpf::server::EditionRouteAccess;
using hacdcpf::server::classify_trial_route;
using hacdcpf::server::edition_analysis_plan_json;
using hacdcpf::server::edition_profile_json;

TEST_CASE("Edition profile matches the build mode", "[trial][edition]") {
#ifdef HACDCPF_TRIAL_EDITION
  const auto profile = edition_profile_json();
  CHECK(profile.at("edition") == "trial");
  CHECK(profile.at("route_policy").at("mode") == "fail_closed");
#else
  const auto profile = edition_profile_json();
  CHECK(profile.at("edition") == "full");
  CHECK(profile.at("enabled_modules") == nlohmann::json::array({"*"}));
  CHECK_FALSE(profile.contains("route_policy"));
#endif
}

TEST_CASE("Trial route policy fails closed", "[trial][edition]") {
  CHECK(classify_trial_route("GET", "/xjtu/").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("GET", "/api/cases").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("GET", "/api/cases_extra").access ==
        EditionRouteAccess::Unclassified);
  CHECK(classify_trial_route("POST", "/api/session/run_reliability").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/session/run_reliability_fmea").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/session/opf_parity").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/opf/ac").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/v1/sessions").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("GET", "/api/v1/jobs/job-1/frames/0").access ==
        EditionRouteAccess::Retained);
  CHECK(classify_trial_route("POST", "/api/v1/specialist_solver").access ==
        EditionRouteAccess::Unclassified);

  const auto time_series =
      classify_trial_route("POST", "/api/session/set_ts_config");
  CHECK(time_series.access == EditionRouteAccess::Disabled);
  CHECK(time_series.feature == "time_series");
  CHECK(classify_trial_route("POST", "/api/session/run_lifecycle_compare").feature ==
        "time_series");
  CHECK(classify_trial_route("POST", "/api/session/harmonics_freqscan").feature ==
        "harmonics");
  CHECK(classify_trial_route("POST", "/api/session/run_market_clearing").feature ==
        "market");
  CHECK(classify_trial_route("POST", "/api/session/load_cim_dist").feature ==
        "advanced_io");

  const auto unknown =
      classify_trial_route("POST", "/api/session/new_specialist_solver");
  CHECK(unknown.access == EditionRouteAccess::Unclassified);
  CHECK(unknown.feature == "unclassified_api");
}

TEST_CASE("Trial analysis plan is backend ordered", "[trial][edition]") {
  const auto plan = edition_analysis_plan_json(
      {{"indicators",
        {"system_economic", "user_carbon", "system_reliability"}}});
  REQUIRE(plan.at("schema") == "hacdcpf.edition-analysis-plan.v1");
  REQUIRE(plan.at("automatic_execution") == false);
  const auto& steps = plan.at("steps");
  REQUIRE(steps.size() == 8);
  CHECK(steps.at(0).at("module") == "powerFlow");
  CHECK(steps.at(1).at("module") == "opf");
  CHECK(steps.at(2).at("module") == "hosting");
  CHECK(steps.at(3).at("module") == "carbonFlow");
  CHECK(steps.at(4).at("module") == "scenarioGeneration");
  CHECK(steps.at(5).at("module") == "reliability");
  CHECK(steps.at(6).at("module") == "shortCircuit");
  CHECK(steps.at(7).at("module") == "weakLinks");

  CHECK_THROWS_AS(edition_analysis_plan_json(
                      {{"indicators", {"internal_economic"}}}),
                  std::invalid_argument);
}

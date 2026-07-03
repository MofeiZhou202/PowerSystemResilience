#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>

#include "hacdcpf/io/scenario_bundle_io.hpp"

using namespace hacdcpf::io;

TEST_CASE("Scenario bundle validation accepts v3 dimensionless multiplier schema", "[scenario_bundle]") {
  nlohmann::json bundle = {
      {"format", "generated_scenario_case_bundle_v3"},
      {"schema_version", 3},
      {"unit_space", "dimensionless_multiplier"},
      {"case_count", 1},
      {"cases", nlohmann::json::array({{
          {"case_id", "case_1"},
          {"system", {{"name", "demo"}, {"ac", nlohmann::json::object()}}},
          {"generated_scenario", {{"family", "regular"}, {"scenario_id", "s1"}}},
          {"standard_time_series", {
              {"unit_space", "dimensionless_multiplier"},
              {"num_steps", 3},
              {"step_duration_hr", 1.0},
              {"profiles", nlohmann::json::array({
                  {{"id", 0}, {"name", "scenario_load_scale"}, {"unit", "multiplier"}, {"values", nlohmann::json::array({0.9, 1.0, 1.1})}},
                  {{"id", 7}, {"name", "storage_soc"}, {"unit", "soc_fraction"}, {"values", nlohmann::json::array({0.5, 0.6, 0.5})}}
              })},
              {"binding", {{"assign_all_loads_to", 0}, {"load_profile_map", nlohmann::json::array()}}}
          }}
      }})}
  };

  ScenarioBundleIoReport report;
  CHECK_NOTHROW(validate_scenario_bundle(bundle, ScenarioBundleImportMode::Strict, report));
  CHECK(report.errors.empty());
}

TEST_CASE("Scenario bundle validation rejects invalid multiplier profiles", "[scenario_bundle]") {
  nlohmann::json bundle = {
      {"format", "generated_scenario_case_bundle_v3"},
      {"schema_version", 3},
      {"unit_space", "dimensionless_multiplier"},
      {"cases", nlohmann::json::array({{
          {"case_id", "case_1"},
          {"standard_time_series", {
              {"num_steps", 2},
              {"profiles", nlohmann::json::array({
                  {{"id", 0}, {"name", "scenario_load_scale"}, {"unit", "multiplier"}, {"values", nlohmann::json::array({1.0, -0.1})}},
                  {{"id", 0}, {"name", "duplicate"}, {"unit", "multiplier"}, {"values", nlohmann::json::array({1.0})}}
              })},
              {"binding", {{"load_profile_map", nlohmann::json::array({{{"profile_id", 99}}})}}}
          }}
      }})}
  };

  ScenarioBundleIoReport report;
  CHECK_THROWS(validate_scenario_bundle(bundle, ScenarioBundleImportMode::Strict, report));
  CHECK(!report.errors.empty());
}

TEST_CASE("Scenario bundle normalization upgrades v2 case shape", "[scenario_bundle]") {
  nlohmann::json v2 = {
      {"format", "generated_scenario_case_bundle_v2"},
      {"schema_version", 2},
      {"family", "regular"},
      {"cases", nlohmann::json::array({{
          {"name", "legacy"},
          {"ac", nlohmann::json::object()},
          {"_generated_scenario", {{"family", "regular"}, {"representative_id", "rep1"}}},
          {"_time_series", {{"num_steps", 1}, {"profiles", nlohmann::json::array({{{"id", 0}, {"name", "scenario_load_scale"}, {"values", nlohmann::json::array({1.0})}}})}}}
      }})}
  };

  ScenarioBundleIoReport report;
  const auto v3 = normalize_generated_scenario_bundle(v2, report);
  REQUIRE(v3.value("format", "") == "generated_scenario_case_bundle_v3");
  REQUIRE(v3.value("unit_space", "") == "dimensionless_multiplier");
  REQUIRE(v3["cases"].size() == 1);
  CHECK(v3["cases"][0].contains("system"));
  CHECK(v3["cases"][0].contains("standard_time_series"));
}

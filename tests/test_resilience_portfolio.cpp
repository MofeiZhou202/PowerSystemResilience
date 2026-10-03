#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/resilience/resilience_portfolio.hpp"

#include <algorithm>
#include <cmath>

using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {
HybridPowerSystem test_system() {
  io::set_case_data_root(HACDCPF_MATPOWER_DATA_DIR);
  return io::build_dist33_weather_mixed();
}

ScenarioCluster cluster(const std::string& id, int branch, double mass) {
  ScenarioCluster row;
  row.representative_id = id;
  row.probability = mass;
  row.representative.id = id;
  row.representative.resilience_event = ResilienceEventDefinition{};
  row.representative.resilience_event->hazard_type = "lightning";
  row.representative.resilience_event->faults.emplace_back(branch, 0.0, 2.0, id);
  row.representative.standard_time_series = {
      {"num_steps", 4}, {"step_duration_hr", 1.0},
      {"profiles", nlohmann::json::array({{{"id", 0}, {"values", {1.0, 1.0, 1.0, 1.0}}}})},
      {"binding", {{"load_profile_map", nlohmann::json::array()}}}};
  return row;
}
}  // namespace

TEST_CASE("A portfolio applies one stable resource configuration to every representative", "[resilience][portfolio]") {
  const auto system = test_system();
  REQUIRE(system.ac.branches.size() > 2);
  ResilienceScenarioResult generated;
  ResilienceIntensityScenarioGroup group;
  group.hazard_type = "lightning";
  group.intensity = TyphoonIntensityCategory::SuperTY;
  group.clusters.push_back(cluster("representative-A", system.ac.branches[0].index, 0.6));
  group.clusters.push_back(cluster("representative-B", system.ac.branches[1].index, 0.4));
  generated.intensities.push_back(group);
  ResilienceIntensityScenarioGroup second_group;
  second_group.hazard_type = "rainstorm";
  second_group.clusters.push_back(cluster("representative-C", system.ac.branches[2].index, 1.0));
  second_group.clusters.back().representative.resilience_event->hazard_type = "rainstorm";
  generated.intensities.push_back(second_group);

  const auto result = plan_resilience_portfolio(system, generated, true, true);
  REQUIRE(result.scenarios.size() == 3);
  CHECK(result.scenarios[0].scenario_id == "representative-A");
  CHECK(result.scenarios[1].scenario_id == "representative-B");
  CHECK(result.scenarios[2].scenario_id == "representative-C");
  CHECK(result.scenarios[0].design_weight == Catch::Approx(0.3));
  CHECK(result.scenarios[1].design_weight == Catch::Approx(0.2));
  CHECK(result.scenarios[2].design_weight == Catch::Approx(0.5));
  CHECK(std::isfinite(result.baseline_design_weighted_shed_mwh));
  CHECK(std::isfinite(result.planned_design_weighted_shed_mwh));
  CHECK(result.baseline_design_weighted_shed_mwh == Catch::Approx(
      0.3 * result.scenarios[0].baseline_shed_mwh +
      0.2 * result.scenarios[1].baseline_shed_mwh +
      0.5 * result.scenarios[2].baseline_shed_mwh));

  auto augmented = system;
  apply_resilience_portfolio_plan(augmented, result.plan);
  REQUIRE(augmented.ac.generators.size() == system.ac.generators.size() + 1);
  REQUIRE(augmented.mobile_storage.size() == system.mobile_storage.size() + 1);
  CHECK(augmented.ac.generators.back().index == result.plan.ac_generator_index);
  CHECK(augmented.ac.generators.back().bus == result.plan.ac_generator_bus);
  CHECK(augmented.mobile_storage.back().index == result.plan.mobile_storage_index);
  CHECK(augmented.mobile_storage.back().bus == result.plan.mobile_storage_bus);
  CHECK_THROWS_AS(apply_resilience_portfolio_plan(augmented, result.plan), std::invalid_argument);
}

TEST_CASE("Portfolio rejects incomplete generated evidence before planning", "[resilience][portfolio]") {
  const auto system = test_system();
  ResilienceScenarioResult generated;
  ResilienceIntensityScenarioGroup group;
  group.clusters.push_back(cluster("missing-event", system.ac.branches.front().index, 1.0));
  group.clusters.front().representative.resilience_event.reset();
  generated.intensities.push_back(group);
  CHECK_THROWS_AS(plan_resilience_portfolio(system, generated, true, false), std::invalid_argument);
  CHECK_THROWS_AS(plan_resilience_portfolio(system, generated, false, false), std::invalid_argument);
  group.clusters.front().representative.resilience_event = ResilienceEventDefinition{};
  group.clusters.push_back(group.clusters.front());
  generated.intensities = {group};
  CHECK_THROWS_AS(plan_resilience_portfolio(system, generated, true, false), std::invalid_argument);
}

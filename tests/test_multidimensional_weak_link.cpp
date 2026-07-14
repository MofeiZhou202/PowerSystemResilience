#include <catch2/catch_test_macros.hpp>

#include <algorithm>

#include "hacdcpf/analysis/multidimensional_weak_link.hpp"

using namespace hacdcpf::analysis;

namespace {

WeakLinkEntityEvidence entity(std::string key,
                              std::array<double, 4> pressure) {
  WeakLinkEntityEvidence row;
  row.key = std::move(key);
  row.name = row.key;
  for (size_t d = 0; d < pressure.size(); ++d) {
    row.dimensions[d].pressure = pressure[d];
  }
  return row;
}

}  // namespace

TEST_CASE("multidimensional weak-link case separates consensus and conflicts",
          "[weak-link][planning][internal-case]") {
  std::vector<WeakLinkEntityEvidence> entities = {
      entity("economic-carbon-corridor", {0.95, 0.85, 0.10, 0.10}),
      entity("reliability-resilience-feeder", {0.10, 0.10, 0.95, 0.85}),
      entity("integrated-upgrade-location", {0.80, 0.75, 0.82, 0.78}),
      entity("dominated-location", {0.20, 0.20, 0.20, 0.20}),
  };

  std::vector<WeakLinkPeriodEvidence> periods(3);
  periods[0].step = 0;
  periods[0].hour = 8.0;
  periods[0].dimensions[0].pressure = 0.90;
  periods[0].dimensions[1].pressure = 0.75;
  periods[1].step = 1;
  periods[1].hour = 16.0;
  periods[1].dimensions[0].pressure = 0.30;
  periods[1].dimensions[3].pressure = 0.35;
  periods[2].step = 2;
  periods[2].hour = 20.0;
  periods[2].dimensions[2].pressure = 0.88;
  periods[2].dimensions[3].pressure = 1.10;

  MultidimensionalWeakLinkOptions options;
  options.top_k = 10;
  options.minimum_dimensions = 2;
  const auto result = run_multidimensional_weak_link_assessment(
      entities, periods, options);

  REQUIRE(result.entities.size() == 4);
  CHECK(result.compatible_count == 1);
  CHECK(result.conflict_count == 2);
  CHECK(result.pareto_count == 3);
  CHECK(result.entities.front().key == "economic-carbon-corridor");

  const auto find = [&](const std::string& key) -> const WeakLinkEntityResult& {
    const auto it = std::find_if(result.entities.begin(), result.entities.end(),
                                 [&](const auto& row) { return row.key == key; });
    REQUIRE(it != result.entities.end());
    return *it;
  };
  CHECK(find("integrated-upgrade-location").relation == "compatible");
  CHECK(find("economic-carbon-corridor").relation == "conflict");
  CHECK(find("economic-carbon-corridor").dominant_dimension == "economic");
  CHECK_FALSE(find("dominated-location").pareto);
  REQUIRE(result.critical_periods.size() == 3);
  CHECK(result.critical_periods.front().hour == 20.0);
  CHECK(result.critical_periods.front().dominant_dimension == "resilience");
}

TEST_CASE("missing dimensions reduce evidence instead of becoming zero risk",
          "[weak-link][evidence]") {
  WeakLinkEntityEvidence row;
  row.key = "partial";
  row.dimensions[2].pressure = 2.0;

  MultidimensionalWeakLinkOptions options;
  options.minimum_dimensions = 2;
  const auto result = run_multidimensional_weak_link_assessment({row}, {}, options);

  REQUIRE(result.entities.size() == 1);
  CHECK(result.entities[0].evidence_dimensions == 1);
  CHECK(result.entities[0].relation == "insufficient_evidence");
  CHECK(result.entities[0].dimensions[0].available == false);
  CHECK(result.entities[0].severity == 2.0);
}

TEST_CASE("operation mode returns operational actions", "[weak-link][operation]") {
  MultidimensionalWeakLinkOptions options;
  options.mode = WeakLinkDecisionMode::Operation;
  const auto result = run_multidimensional_weak_link_assessment(
      {entity("security", {0.10, 0.10, 0.90, 0.85})}, {}, options);
  REQUIRE(result.entities.size() == 1);
  CHECK(result.entities[0].decision_hint == "emergency_reconfiguration");
}

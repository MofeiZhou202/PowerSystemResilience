#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>

#include "hacdcpf/analysis/counterfactual_planning.hpp"
#include "hacdcpf/io/case_builders.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {

CounterfactualMeasure measure(CounterfactualMeasureType type,
                              std::string id) {
  CounterfactualMeasure row;
  row.type = type;
  row.id = std::move(id);
  row.name = row.id;
  return row;
}

}  // namespace

TEST_CASE("counterfactual measures mutate only the cloned system",
          "[counterfactual][planning][internal-case]") {
  const auto baseline = io::build_cyber_physical_reliability_demo();
  REQUIRE(baseline.ac.branches.size() > 1);
  const auto original_rating = baseline.ac.branches[1].rate_a_mva;
  const auto original_resistance = baseline.ac.branches[1].r_pu;

  auto expanded = baseline;
  auto line = measure(CounterfactualMeasureType::LineCapacity, "line-test");
  line.target_index = baseline.ac.branches[1].index;
  line.expansion_factor = 2.0;
  REQUIRE(apply_counterfactual_measure(expanded, line).applied);
  CHECK(expanded.ac.branches[1].rate_a_mva == Catch::Approx(original_rating * 2.0));
  CHECK(expanded.ac.branches[1].r_pu == Catch::Approx(original_resistance / 2.0));
  CHECK(baseline.ac.branches[1].rate_a_mva == original_rating);
  CHECK(baseline.ac.branches[1].r_pu == original_resistance);

  auto with_storage = baseline;
  auto storage = measure(CounterfactualMeasureType::Storage, "storage-test");
  storage.bus = 2;
  storage.capacity_mw = 0.4;
  storage.energy_mwh = 1.6;
  storage.dispatch_fraction = 0.25;
  REQUIRE(apply_counterfactual_measure(with_storage, storage).applied);
  REQUIRE(with_storage.ac.storage.size() == baseline.ac.storage.size() + 1);
  CHECK(with_storage.ac.storage.back().grid_forming);
  CHECK(with_storage.ac.storage.back().p_rated_mw == Catch::Approx(0.4));

  auto with_der = baseline;
  auto der = measure(CounterfactualMeasureType::DistributedEnergyResource,
                     "der-test");
  der.bus = 2;
  der.capacity_mw = 0.5;
  der.capacity_factor = 0.4;
  REQUIRE(apply_counterfactual_measure(with_der, der).applied);
  REQUIRE(with_der.ac.static_generators.size() ==
          baseline.ac.static_generators.size() + 1);
  CHECK(with_der.ac.static_generators.back().p_mw == Catch::Approx(0.2));

  auto with_tie = baseline;
  with_tie.ac.branches.erase(std::remove_if(
      with_tie.ac.branches.begin(), with_tie.ac.branches.end(),
      [](const auto& branch) { return !branch.in_service; }),
      with_tie.ac.branches.end());
  const auto candidate_system = with_tie;
  const auto branch_count = with_tie.ac.branches.size();
  const auto switch_count = with_tie.ac.switches.size();
  auto tie = measure(CounterfactualMeasureType::TieSwitch, "tie-test");
  tie.from_bus = 1;
  tie.to_bus = 3;
  tie.capacity_mw = 1.2;
  REQUIRE(apply_counterfactual_measure(with_tie, tie).applied);
  REQUIRE(with_tie.ac.branches.size() == branch_count + 1);
  REQUIRE(with_tie.ac.switches.size() == switch_count + 1);
  CHECK_FALSE(with_tie.ac.branches.back().in_service);
  CHECK_FALSE(with_tie.ac.switches.back().closed);
  CHECK(with_tie.ac.switches.back().is_remote);

  CounterfactualPlanningOptions generated_options;
  generated_options.max_measures = 5;
  const auto generated = generate_counterfactual_measures(
      candidate_system, generated_options);
  REQUIRE(generated.size() == 5);
  for (const auto type : {CounterfactualMeasureType::LineCapacity,
                          CounterfactualMeasureType::Storage,
                          CounterfactualMeasureType::TieSwitch,
                          CounterfactualMeasureType::Automation,
                          CounterfactualMeasureType::DistributedEnergyResource}) {
    CHECK(std::any_of(generated.begin(), generated.end(),
                      [&](const auto& row) { return row.type == type; }));
  }

  auto automated = baseline;
  auto automation = measure(CounterfactualMeasureType::Automation,
                            "automation-test");
  automation.target_index = baseline.ac.branches[1].index;
  REQUIRE(apply_counterfactual_measure(automated, automation).applied);
  const auto switch_it = std::find_if(
      automated.ac.switches.begin(), automated.ac.switches.end(),
      [&](const auto& sw) { return sw.element_id == automation.target_index; });
  REQUIRE(switch_it != automated.ac.switches.end());
  CHECK(switch_it->is_automated);
  CHECK(switch_it->is_remote);
}

TEST_CASE("counterfactual planning recomputes benefits and pair synergy",
          "[counterfactual][planning][internal-case]") {
  auto system = io::build_cyber_physical_reliability_demo();
  REQUIRE_FALSE(system.ac.external_grids.empty());
  system.ac.external_grids.front().cost_c1 = 100.0;
  system.ac.external_grids.front().emission_factor_tco2_mwh = 0.8;

  REQUIRE(system.ac.branches.size() > 1);
  auto line = measure(CounterfactualMeasureType::LineCapacity, "line:1");
  line.target_index = system.ac.branches[1].index;
  line.expansion_factor = 2.0;

  auto storage = measure(CounterfactualMeasureType::Storage, "storage:2");
  storage.bus = 2;
  storage.capacity_mw = 0.4;
  storage.energy_mwh = 1.6;
  storage.dispatch_fraction = 0.25;

  CounterfactualPlanningOptions options;
  options.max_measures = 2;
  options.max_pairs = 1;
  options.include_pairs = true;
  options.resilience_horizon_hours = 4;
  options.resilience_fault_branch_indices = {system.ac.branches[1].index};
  options.reliability_physical_budget = 20;

  const auto result = run_counterfactual_planning_assessment(
      system, {line, storage}, options);

  CHECK(result.evaluated_systems == 4);
  CHECK(result.failed_systems == 0);
  REQUIRE(result.measures.size() == 2);
  REQUIRE(result.interactions.size() == 1);
  CHECK(result.baseline.values[2].has_value());
  CHECK(result.baseline.values[3].has_value());
  for (const auto& row : result.measures) {
    CHECK(row.applied);
    CHECK(row.benefit.absolute[2].has_value());
    CHECK(row.benefit.absolute[3].has_value());
  }
  REQUIRE(result.measures[0].benefit.absolute[0].has_value());
  CHECK(*result.measures[0].benefit.absolute[0] > 0.0);
  const auto& pair = result.interactions.front();
  CHECK(pair.applied);
  CHECK(pair.combined_benefit.absolute[2].has_value());
  CHECK(pair.combined_benefit.absolute[3].has_value());
  CHECK(pair.synergy.absolute[2].has_value());
  CHECK(pair.synergy.absolute[3].has_value());
  REQUIRE(pair.synergy.absolute[0].has_value());
  CHECK(std::abs(*pair.synergy.absolute[0]) > 1e-9);
}

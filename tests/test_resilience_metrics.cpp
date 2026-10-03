#include "hacdcpf/resilience/resilience_metrics.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace hacdcpf::analysis {
namespace {

ResilienceMetricRun recovery_run() {
  return ResilienceMetricRun{
      {{0.0, 100.0, 100.0, 0.0, 1.0, 0, 0},
       {1.0, 100.0, 50.0, 50.0, 0.5, 2, 0},
       {2.0, 100.0, 60.0, 40.0, 0.6, 1, 1},
       {2.5, 100.0, 80.0, 20.0, 0.8, 1, 1},
       {3.5, 100.0, 100.0, 0.0, 1.0, 0, 2}},
      1.5,
      2.0,
      false};
}

const ResilienceMetricResult& result_for(
    const std::vector<ResilienceMetricResult>& results, const std::string& id) {
  const auto it = std::find_if(results.begin(), results.end(),
                               [&](const auto& result) { return result.id == id; });
  REQUIRE(it != results.end());
  return *it;
}

}  // namespace

TEST_CASE("Resilience metrics expose the complete Chapter 3 catalog", "[resilience][metrics]") {
  const auto& catalog = resilience_metric_catalog();
  REQUIRE(catalog.size() == 42);

  std::vector<std::string> ids;
  ids.reserve(catalog.size());
  for (const auto& definition : catalog) {
    REQUIRE_FALSE(definition.id.empty());
    REQUIRE_FALSE(definition.symbol.empty());
    REQUIRE_FALSE(definition.formula_ref.empty());
    REQUIRE_FALSE(definition.unit.empty());
    REQUIRE_FALSE(definition.required_inputs.empty());
    auto inputs = definition.required_inputs;
    std::sort(inputs.begin(), inputs.end());
    CHECK(std::adjacent_find(inputs.begin(), inputs.end()) == inputs.end());
    ids.push_back(definition.id);
  }

  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.lolp") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.edns") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.alril") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.pcfd") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.psi") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.pin") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.gma") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.tmts") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.cllp") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.atcs") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.apda") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.ledsr") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.rlro") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.t_sp") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.arss") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.res") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.rei") == 1);
  REQUIRE(std::count(ids.begin(), ids.end(), "ch3.rse") == 1);
}

TEST_CASE("Resilience metrics distinguish unavailable values from zero", "[resilience][metrics]") {
  ResilienceMetricEvaluationOptions options;
  options.selected_metric_ids = {"ch3.lolp", "ch3.edns", "ch3.rei", "ch3.rse"};
  const auto results = evaluate_resilience_metrics(recovery_run(), options);

  for (const auto& result : results) {
    REQUIRE(result.status == ResilienceMetricStatus::Unavailable);
    REQUIRE_FALSE(result.value.has_value());
    REQUIRE_FALSE(result.reason_code.empty());
  }
  REQUIRE(result_for(results, "ch3.rei").reason_code == "FORMULA_REVIEW_REQUIRED");
  REQUIRE(result_for(results, "ch3.rse").reason_code ==
          "COST_DATA_MISSING_AND_FORMULA_IMAGE");
}

TEST_CASE("Resilience metrics calculate non-hourly threshold and recovery windows", "[resilience][metrics]") {
  ResilienceMetricEvaluationOptions options;
  options.selected_metric_ids = {"ch3.cllp", "ch3.ledsr", "ch3.rlro", "ch3.t_sp", "ch3.arss"};
  options.target_ratio = 0.9;
  const auto results = evaluate_resilience_metrics(recovery_run(), options);

  const auto& cllp = result_for(results, "ch3.cllp");
  REQUIRE(cllp.status == ResilienceMetricStatus::Computed);
  REQUIRE(cllp.value.has_value());
  REQUIRE(*cllp.value == Catch::Approx(0.0));

  const auto& ledsr = result_for(results, "ch3.ledsr");
  REQUIRE(ledsr.status == ResilienceMetricStatus::Computed);
  REQUIRE(*ledsr.value == Catch::Approx(0.5));

  const auto& rlro = result_for(results, "ch3.rlro");
  REQUIRE(rlro.status == ResilienceMetricStatus::Approximate);
  REQUIRE(rlro.approximate);
  REQUIRE(*rlro.value == Catch::Approx(0.75));

  const auto& tsp = result_for(results, "ch3.t_sp");
  REQUIRE(tsp.status == ResilienceMetricStatus::Computed);
  REQUIRE(*tsp.value == Catch::Approx(1.8));

  const auto& arss = result_for(results, "ch3.arss");
  REQUIRE(arss.status == ResilienceMetricStatus::Computed);
  REQUIRE(*arss.value == Catch::Approx(0.5833333333333333));
}

TEST_CASE("Resilience metrics reject invalid values and unknown selections", "[resilience][metrics]") {
  ResilienceMetricEvaluationOptions options;
  options.selected_metric_ids = {"ch3.cllp", "not_a_metric"};
  ResilienceMetricRun invalid{{{0.0, 100.0, std::numeric_limits<double>::quiet_NaN(),
                                0.0, 1.0, 0, 0}}};
  const auto results = evaluate_resilience_metrics(invalid, options);

  REQUIRE(result_for(results, "ch3.cllp").status == ResilienceMetricStatus::Invalid);
  REQUIRE(result_for(results, "ch3.cllp").reason_code == "INVALID_RUN_STEPS");
  REQUIRE(result_for(results, "not_a_metric").status == ResilienceMetricStatus::Invalid);
  REQUIRE(result_for(results, "not_a_metric").reason_code == "UNKNOWN_METRIC_ID");
}

TEST_CASE("Resilience metrics mark censored thresholds and missing recovery metadata", "[resilience][metrics]") {
  ResilienceMetricEvaluationOptions options;
  options.selected_metric_ids = {"ch3.t_sp", "ch3.ledsr"};
  options.target_ratio = 0.95;

  ResilienceMetricRun short_run{{{0.0, 100.0, 50.0, 50.0, 0.5, 1, 0},
                                 {0.5, 100.0, 60.0, 40.0, 0.6, 1, 0}},
                                std::nullopt, std::nullopt, false};
  const auto results = evaluate_resilience_metrics(short_run, options);
  REQUIRE(result_for(results, "ch3.t_sp").status == ResilienceMetricStatus::Unavailable);
  REQUIRE(result_for(results, "ch3.ledsr").status == ResilienceMetricStatus::Unavailable);
  REQUIRE(result_for(results, "ch3.ledsr").reason_code == "DISASTER_END_MISSING");

  short_run.disaster_end_hr = 0.0;
  const auto censored = evaluate_resilience_metrics(short_run, options);
  REQUIRE(result_for(censored, "ch3.t_sp").status == ResilienceMetricStatus::NotApplicable);
  REQUIRE(result_for(censored, "ch3.t_sp").censored);
  REQUIRE(result_for(censored, "ch3.t_sp").reason_code ==
          "TARGET_NOT_REACHED_WITHIN_WINDOW");
}

TEST_CASE("Resilience RES remains unavailable without explicit approximation consent", "[resilience][metrics]") {
  ResilienceMetricEvaluationOptions options;
  options.selected_metric_ids = {"ch3.res"};
  ResilienceMetricRun run = recovery_run();
  run.has_traceable_importance_mapping = true;
  for (auto& step : run.steps) step.weighted_shed_mw = 2.0 * step.shed_mw;

  const auto unavailable = evaluate_resilience_metrics(run, options);
  REQUIRE(unavailable.front().status == ResilienceMetricStatus::Unavailable);

  options.allow_res_approximation = true;
  const auto approximate = evaluate_resilience_metrics(run, options);
  REQUIRE(approximate.front().status == ResilienceMetricStatus::Approximate);
  REQUIRE(approximate.front().approximate);
  REQUIRE(approximate.front().value.has_value());
  run.steps[2].weighted_shed_mw = 80;
  run.steps[3].weighted_shed_mw = 20;
  run.steps[4].weighted_shed_mw = 0;
  const auto weighted = evaluate_resilience_metrics(run, options);
  REQUIRE(weighted.front().value);
  CHECK(*weighted.front().value == Catch::Approx(1.0 - 35.0 / 120.0));
}

TEST_CASE("Operational resilience metrics integrate complete nonuniform intervals", "[resilience][metrics]") {
  ResilienceMetricRun run;
  run.steps = {{0, 100, 50, 50, .5, 2, 0}, {0.5, 100, 80, 20, .8, 1, 1}, {2, 100, 100, 0, 1, 0, 2}};
  const double durations[] = {.5, 1.5, 1};
  for (std::size_t i = 0; i < run.steps.size(); ++i) {
    auto& step = run.steps[i]; step.duration_hr = durations[i];
    step.weighted_shed_mw = step.shed_mw * 2;
    step.shed_by_priority = {step.shed_mw * .1, step.shed_mw * .2, step.shed_mw * .3, step.shed_mw * .4};
    step.island_count = 3 - static_cast<int>(i); step.switch_actions = static_cast<int>(i);
  }
  run.mess_energy_delivered_mwh = 12.5; run.mess_travel_distance_km = 8;
  const auto results = evaluate_resilience_metrics(run);
  const std::vector<std::pair<std::string, double>> expected = {
    {"duration", 3}, {"demand_energy", 300}, {"served_energy", 245}, {"ens", 55},
    {"energy_supply_ratio", 245./300}, {"energy_loss_ratio", 55./300}, {"mean_shed", 55./3},
    {"peak_shed", 50}, {"minimum_supply_ratio", .5}, {"final_supply_ratio", 1},
    {"equivalent_outage_hours", .55}, {"interrupted_hours", 2}, {"below_90_hours", 2},
    {"weighted_ens", 110}, {"critical_ens", 5.5}, {"high_ens", 11}, {"medium_ens", 16.5},
    {"low_ens", 22}, {"peak_active_faults", 2}, {"repaired_faults", 2}, {"peak_islands", 3},
    {"switch_actions", 3}, {"mess_energy", 12.5}, {"mess_distance", 8}};
  for (const auto& [id, value] : expected) {
    const auto& r = result_for(results, "run." + id);
    INFO(id); REQUIRE(r.status == ResilienceMetricStatus::Computed);
    REQUIRE(r.value); CHECK(*r.value == Catch::Approx(value));
  }
  run.steps.back().weighted_shed_mw.reset();
  run.steps.back().shed_by_priority.clear();
  run.steps.back().island_count.reset();
  const auto missing = evaluate_resilience_metrics(run);
  for (const auto& id : {"run.weighted_ens", "run.critical_ens", "run.peak_islands"}) {
    CHECK(result_for(missing, id).status == ResilienceMetricStatus::Unavailable);
    CHECK_FALSE(result_for(missing, id).value);
  }
}

TEST_CASE("Metric windows reject invalid data and retain missing or failed evidence", "[resilience][metrics]") {
  auto run = recovery_run();
  SECTION("unordered samples") { run.steps[1].hour = 0; }
  SECTION("negative demand") { run.steps[1].demand_mw = -1; }
  SECTION("unbalanced power") { run.steps[1].shed_mw = 90; }
  SECTION("overlapping durations") { run.steps[0].duration_hr = 2; }
  SECTION("invalid event order") { run.disaster_end_hr = 3; }
  for (const auto& r : evaluate_resilience_metrics(run)) {
    CHECK(r.status == ResilienceMetricStatus::Invalid); CHECK_FALSE(r.value);
  }
}

TEST_CASE("Metric proxies and failure gates never fabricate probability or zero", "[resilience][metrics]") {
  auto run = recovery_run();
  ResilienceMetricEvaluationOptions options; options.selected_metric_ids = {"ch3.apda", "ch3.lolp"};
  CHECK_FALSE(evaluate_resilience_metrics(run, options).front().value);
  options.allow_apda_system_gap_approximation = true;
  const auto proxy = evaluate_resilience_metrics(run, options);
  CHECK(proxy.front().status == ResilienceMetricStatus::Approximate);
  REQUIRE(proxy.front().value); CHECK(*proxy.front().value == Catch::Approx(50));
  CHECK_FALSE(proxy.back().value);
  run.usable = false;
  for (const auto& r : evaluate_resilience_metrics(run)) { CHECK_FALSE(r.value); CHECK(r.reason_code == "RUN_NOT_FEASIBLE_OR_COMPLETE"); }
}

TEST_CASE("Zero load and truncated restoration have explicit status", "[resilience][metrics]") {
  ResilienceMetricRun zero; zero.steps = {{0, 0, 0, 0, 1, 0, 0}}; zero.steps[0].duration_hr = 1;
  const auto result = evaluate_resilience_metrics(zero);
  CHECK_FALSE(result_for(result, "run.energy_supply_ratio").value);
  CHECK_FALSE(result_for(result, "run.minimum_supply_ratio").value);
  REQUIRE(result_for(result, "run.ens").value); CHECK(*result_for(result, "run.ens").value == 0);
  auto run = recovery_run(); run.steps.pop_back();
  const auto truncated = evaluate_resilience_metrics(run);
  CHECK(result_for(truncated, "ch3.arss").censored);
  REQUIRE(result_for(truncated, "ch3.arss").value);
  CHECK(*result_for(truncated, "ch3.arss").value == Catch::Approx(.25));
}

}  // namespace hacdcpf::analysis

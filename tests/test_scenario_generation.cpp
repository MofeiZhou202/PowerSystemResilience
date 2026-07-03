#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>

#include "hacdcpf/analysis/scenario_generation.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using namespace hacdcpf;
using namespace hacdcpf::analysis;
using Catch::Approx;

namespace {

HybridPowerSystem make_regular_scenario_test_system() {
  HybridPowerSystem sys;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.in_service = true;

  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.vm_pu = 1.0;
  load_bus.in_service = true;

  sys.ac.buses = {slack, load_bus};

  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.in_service = true;
  branch.r_pu = 0.01;
  branch.x_pu = 0.01;
  sys.ac.branches = {branch};

  Load load;
  load.index = 1;
  load.bus = 2;
  load.in_service = true;
  load.p_mw = 10.0;
  load.scaling = 1.0;
  sys.ac.loads = {load};

  PVSystem pv;
  pv.index = 1;
  pv.bus = 2;
  pv.in_service = true;
  pv.p_mw = 4.0;
  pv.pmax_mw = 5.0;
  sys.ac.pv_systems = {pv};

  return sys;
}

PerturbationOptions deterministic_perturbation() {
  PerturbationOptions perturbation;
  perturbation.enable_load_perturbation = false;
  perturbation.enable_renewable_perturbation = false;
  perturbation.enable_storage_soc_perturbation = false;
  perturbation.enable_climate_perturbation = false;
  perturbation.seed = 7;
  perturbation.climate_seed = 2030;
  return perturbation;
}

std::map<std::string, ScenarioCandidate> representatives_by_id(const RegularScenarioResult& result) {
  std::map<std::string, ScenarioCandidate> reps;
  for (const auto& cluster : result.clusters) reps.emplace(cluster.representative.id, cluster.representative);
  return reps;
}

double feature(const ScenarioCandidate& candidate, const std::string& key) {
  const auto it = candidate.features.find(key);
  REQUIRE(it != candidate.features.end());
  return it->second;
}

std::vector<double> profile_values(const nlohmann::json& ts, const std::string& name) {
  for (const auto& p : ts.value("profiles", nlohmann::json::array())) {
    if (p.value("name", "") == name) return p.value("values", std::vector<double>{});
  }
  return {};
}

std::vector<double> profile_values(const TimeSeriesData& ts, const std::string& name) {
  for (const auto& p : ts.profiles) {
    if (p.name == name) return p.values;
  }
  return {};
}

std::vector<double> representative_feature_values(const RegularScenarioResult& result, const std::string& key) {
  std::vector<double> values;
  for (const auto& cluster : result.clusters) values.push_back(feature(cluster.representative, key));
  std::sort(values.begin(), values.end());
  return values;
}

}  // namespace

TEST_CASE("Regular scenario generation applies SSP/year climate morphing to load and PV", "[scenario_generation]") {
  const auto sys = make_regular_scenario_test_system();

  RegularScenarioOptions regular;
  regular.enabled = true;
  regular.ssp_levels = {"ssp126", "ssp585"};
  regular.years = {2050, 2080};
  regular.num_steps = 8760;
  regular.candidate_count = 4;
  regular.cluster_count = 1;

  ClusteringOptions clustering;
  clustering.compare_baseline = false;
  clustering.include_tail_anchors = false;
  clustering.method = "weighted_k_medoids";

  std::vector<std::string> warnings;
  const auto result = generate_regular_scenarios(sys, regular, deterministic_perturbation(), clustering, &warnings);
  REQUIRE(result.candidate_count == 4);
  REQUIRE(result.cluster_count == 4);

  const auto reps = representatives_by_id(result);
  REQUIRE(reps.count("regular:ssp126:2050:1") == 1);
  REQUIRE(reps.count("regular:ssp585:2080:1") == 1);

  const auto& low = reps.at("regular:ssp126:2050:1");
  const auto& high = reps.at("regular:ssp585:2080:1");

  CHECK(feature(high, "climate_annual_delta_T") > feature(low, "climate_annual_delta_T"));
  CHECK(feature(high, "climate_month_delta_T_2") > feature(low, "climate_month_delta_T_2"));
  CHECK(feature(high, "load_sum") > feature(low, "load_sum"));
  CHECK(feature(high, "pv_sum") != Approx(feature(low, "pv_sum")));
  CHECK(feature(low, "climate_data_found") == Approx(1.0));
  CHECK(feature(high, "climate_data_found") == Approx(1.0));
}

TEST_CASE("Scenario generation prepares standard multiplier time series internally", "[scenario_generation]") {
  const auto sys = make_regular_scenario_test_system();

  RegularScenarioOptions regular;
  regular.enabled = true;
  regular.ssp = "ssp245";
  regular.year = 2050;
  regular.num_steps = 24;
  regular.candidate_count = 1;
  regular.cluster_count = 1;

  ClusteringOptions clustering;
  clustering.compare_baseline = false;
  clustering.include_tail_anchors = false;
  clustering.method = "weighted_k_medoids";

  const auto result = generate_regular_scenarios(sys, regular, deterministic_perturbation(), clustering, nullptr);
  REQUIRE(result.cluster_count == 1);

  const auto& candidate = result.clusters.front().representative;
  REQUIRE(!candidate.standard_time_series.empty());
  const auto& ts = candidate.standard_time_series;
  CHECK(ts.value("unit_space", "") == "dimensionless_multiplier");
  const auto load_scale = profile_values(ts, "scenario_load_scale");
  const auto pv_scale = profile_values(ts, "scenario_pv_scale");
  REQUIRE(!load_scale.empty());
  REQUIRE(!pv_scale.empty());
  CHECK(load_scale.front() == Approx(profile_values(candidate.time_series, "total_load_mw").front() / 10.0));
  CHECK(pv_scale.front() == Approx(profile_values(candidate.time_series, "pv_mw").front() / 5.0));
  CHECK(ts["binding"].value("assign_all_pv_to", -1) == 2);
  CHECK(ts["normalization"].value("base_load_mw", 0.0) == Approx(10.0));
  CHECK(ts["normalization"].value("base_pv_mw", 0.0) == Approx(5.0));
}

TEST_CASE("Regular climate perturbation creates reproducible candidate variation", "[scenario_generation]") {
  const auto sys = make_regular_scenario_test_system();

  RegularScenarioOptions regular;
  regular.enabled = true;
  regular.ssp_levels = {"ssp245"};
  regular.years = {2050};
  regular.num_steps = 8760;
  regular.candidate_count = 3;
  regular.cluster_count = 3;

  auto perturbation = deterministic_perturbation();
  perturbation.enable_climate_perturbation = true;
  perturbation.climate_seed = 12345;

  ClusteringOptions clustering;
  clustering.compare_baseline = false;
  clustering.include_tail_anchors = false;
  clustering.method = "weighted_k_medoids";

  const auto first = generate_regular_scenarios(sys, regular, perturbation, clustering, nullptr);
  const auto second = generate_regular_scenarios(sys, regular, perturbation, clustering, nullptr);
  REQUIRE(first.cluster_count == 3);
  REQUIRE(second.cluster_count == 3);

  const auto loads = representative_feature_values(first, "load_sum");
  const auto pvs = representative_feature_values(first, "pv_sum");
  CHECK(std::abs(loads.front() - loads.back()) > 1e-6);
  CHECK(std::abs(pvs.front() - pvs.back()) > 1e-6);

  const auto loads_again = representative_feature_values(second, "load_sum");
  const auto pvs_again = representative_feature_values(second, "pv_sum");
  REQUIRE(loads_again.size() == loads.size());
  REQUIRE(pvs_again.size() == pvs.size());
  for (std::size_t i = 0; i < loads.size(); ++i) CHECK(loads_again[i] == Approx(loads[i]));
  for (std::size_t i = 0; i < pvs.size(); ++i) CHECK(pvs_again[i] == Approx(pvs[i]));
}

TEST_CASE("Regular climate morphing falls back safely for unknown SSP/year", "[scenario_generation]") {
  const auto sys = make_regular_scenario_test_system();

  RegularScenarioOptions regular;
  regular.enabled = true;
  regular.ssp = "ssp999";
  regular.year = 2090;
  regular.num_steps = 8760;
  regular.candidate_count = 1;
  regular.cluster_count = 1;

  ClusteringOptions clustering;
  clustering.compare_baseline = false;
  clustering.include_tail_anchors = false;
  clustering.method = "weighted_k_medoids";

  std::vector<std::string> warnings;
  const auto result = generate_regular_scenarios(sys, regular, deterministic_perturbation(), clustering, &warnings);
  REQUIRE(result.cluster_count == 1);
  REQUIRE(!warnings.empty());

  const auto& representative = result.clusters.front().representative;
  CHECK(feature(representative, "climate_data_found") == Approx(0.0));
  CHECK(feature(representative, "climate_annual_delta_T") == Approx(0.0));
  CHECK(feature(representative, "climate_annual_delta_GHI") == Approx(0.0));
}

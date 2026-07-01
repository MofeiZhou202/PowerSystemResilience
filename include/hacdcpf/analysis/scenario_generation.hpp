#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/analysis/typhoon_resilience.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::analysis {

enum class ScenarioFamily { Regular, Reliability, Resilience };

enum class ContingencyComponentType {
  ACBranch,
  DCBranch,
  VSCConverter,
  DCDCConverter,
  ACBus,
  DCBus,
  ACGenerator,
  ACStaticGenerator,
  ACRenewable,
  ACPVSystem,
  DCStaticGenerator,
  DCPVArray,
  ACLoad,
  DCLoad,
  ACStorage,
  DCStorage,
  MobileStorage,
  Transformer2W,
  Transformer3W,
  VPP,
  Microgrid,
};

struct RegularScenarioOptions {
  bool enabled{true};
  int candidate_count{100};
  int cluster_count{10};
  int num_steps{8760};
  std::string ssp{"ssp245"};
  int year{2050};
  std::vector<std::string> ssp_levels;
  std::vector<int> years;
};

struct ReliabilityScenarioOptions {
  bool enabled{true};
  int candidates_per_contingency{30};
  int cluster_count_per_contingency{5};
  int num_steps{1};

  // Scenario reliability contingencies intentionally follow the implemented
  // reliability FMEA catalog. Broader compatibility flags are kept for JSON/API
  // stability, but they do not expand the catalog beyond reliability_assessment.
  bool include_ac_branches{true};
  bool include_dc_branches{true};
  bool include_vsc_converters{true};
  bool include_dcdc_converters{true};
  bool include_ac_buses{true};
  bool include_dc_buses{true};
  bool include_generators{true};
  bool include_loads{true};
  bool include_storage{true};
  bool include_vpps{false};
  bool include_microgrids{false};
  int max_contingencies{0};
};

struct ResilienceScenarioOptions {
  bool enabled{true};
  std::vector<TyphoonIntensityCategory> intensity_levels{TyphoonIntensityCategory::TY};
  int candidates_per_intensity{30};
  int default_cluster_count{5};
  std::map<TyphoonIntensityCategory, int> cluster_count_by_intensity;
  int num_steps{48};
  int month{8};
};

struct PerturbationOptions {
  bool enable_load_perturbation{true};
  bool enable_renewable_perturbation{true};
  unsigned int seed{1};
  double load_sigma{0.08};
  double renewable_sigma{0.12};
  double load_min_multiplier{0.75};
  double load_max_multiplier{1.25};
  double renewable_min_multiplier{0.0};
  double renewable_max_multiplier{1.20};
  bool enable_storage_soc_perturbation{true};
  double storage_soc_sigma{0.10};
  double storage_soc_min_multiplier{0.75};
  double storage_soc_max_multiplier{1.25};
  bool enable_climate_perturbation{true};
  unsigned int climate_seed{2030};
  double climate_temp_sigma_c{0.7};
  double climate_ghi_sigma_pct{8.0};
  double climate_load_sigma_pct{3.0};
  int block_hours{24};
  double temporal_correlation{0.75};
  double load_renewable_correlation{-0.25};
};

struct TyphoonImpactOptions {
  int pv_transition_hours{3};
  double load_wind_start{25.0};
  double load_wind_full{50.0};
  double load_max_reduction{0.40};
  double wind_cut_in_ms{2.5};
  double wind_rated_ms{11.2};
  double wind_cut_out_ms{25.0};
  double wind_ramp_exponent{2.0};
};

struct ClusteringOptions {
  std::string method{"hybrid_kmedoids_tail_5pct"};
  int max_iterations{50};
  unsigned int seed{1};
  double continuous_weight{1.0};
  double outage_hamming_weight{1.0};
  bool robust_scale{true};
  bool include_tail_anchors{true};
  double regime_weight{1.0};
  double tail_fraction{0.05};
  double source_tail_quantile{0.95};
  double coupling_quantile{0.90};
  double boundary_fraction{0.02};
  bool freeze_anchors{true};
  bool compare_baseline{true};
};

struct ScenarioGenerationOptions {
  RegularScenarioOptions regular;
  ReliabilityScenarioOptions reliability;
  ResilienceScenarioOptions resilience;
  PerturbationOptions perturbation;
  TyphoonImpactOptions typhoon_impact;
  ClusteringOptions clustering;
};

struct ContingencyDefinition {
  std::string id;
  ContingencyComponentType type{ContingencyComponentType::ACBranch};
  int component_index{0};
  std::string display_name;
  std::vector<std::string> affected_ac_branches;
  std::vector<std::string> affected_dc_branches;
  std::vector<std::string> affected_converters;
  std::vector<std::string> affected_dcdc_converters;
  std::vector<std::string> affected_buses;
  std::vector<std::string> affected_generators;
  std::vector<std::string> affected_loads;
  std::vector<std::string> affected_storage;
};

struct ResilienceEventDefinition {
  std::string id;
  TyphoonIntensityCategory requested_intensity{TyphoonIntensityCategory::Unknown};
  TyphoonIntensityCategory selected_intensity{TyphoonIntensityCategory::Unknown};
  double selected_track_max_vmax_ms{0.0};
  std::string selected_sample_id;
  bool used_catalog_sample{false};
  bool used_category_fallback{false};
  int month{0};
  std::vector<DistributionResilienceFault> faults;
  std::vector<TyphoonTrackPoint> track;
  std::vector<TyphoonBranchRisk> branch_risks;
  std::vector<double> pv_typhoon_multiplier;
  std::vector<double> wind_typhoon_multiplier;
  std::vector<double> renewable_typhoon_multiplier;
  std::vector<double> load_typhoon_multiplier;
  std::optional<int> stage1_start_step;
  std::optional<int> stage1_end_step;
};

struct ScenarioCandidate {
  std::string id;
  ScenarioFamily family{ScenarioFamily::Regular};
  double probability{1.0};
  TimeSeriesData time_series;
  std::optional<ContingencyDefinition> contingency;
  std::optional<ResilienceEventDefinition> resilience_event;
  std::unordered_map<std::string, double> features;
  std::vector<int> outage_signature;
  std::string regime_label;
  double risk_score{0.0};
  std::unordered_map<std::string, double> risk_source_scores;
  std::vector<std::string> anchor_reasons;
  nlohmann::json component_load_profiles;
  nlohmann::json standard_time_series;
  bool tail_anchor{false};
  bool frozen_medoid{false};
};

struct ScenarioCluster {
  int cluster_id{0};
  std::string representative_id;
  ScenarioCandidate representative;
  double probability{0.0};
  int member_count{0};
  std::vector<std::string> member_ids;
  double average_distance{0.0};
  double max_distance{0.0};
  bool frozen_medoid{false};
  std::vector<std::string> anchor_reasons;
};

struct RegularScenarioResult {
  int candidate_count{0};
  int cluster_count{0};
  std::vector<ScenarioCluster> clusters;
  nlohmann::json audit = nlohmann::json::object();
};

struct ReliabilityContingencyScenarioGroup {
  ContingencyDefinition contingency;
  int candidate_count{0};
  int cluster_count{0};
  std::vector<ScenarioCluster> clusters;
  nlohmann::json audit = nlohmann::json::object();
};

struct ReliabilityScenarioResult {
  int contingency_count{0};
  int cluster_total{0};
  std::vector<ReliabilityContingencyScenarioGroup> contingencies;
  nlohmann::json audit = nlohmann::json::object();
};

struct ResilienceIntensityScenarioGroup {
  TyphoonIntensityCategory intensity{TyphoonIntensityCategory::Unknown};
  int candidate_count{0};
  int cluster_count{0};
  std::vector<ScenarioCluster> clusters;
  nlohmann::json audit = nlohmann::json::object();
};

struct ResilienceScenarioResult {
  int intensity_count{0};
  int cluster_total{0};
  std::vector<ResilienceIntensityScenarioGroup> intensities;
  nlohmann::json audit = nlohmann::json::object();
};

struct ScenarioGenerationResult {
  RegularScenarioResult regular;
  ReliabilityScenarioResult reliability;
  ResilienceScenarioResult resilience;
  std::vector<std::string> warnings;
  nlohmann::json summary = nlohmann::json::object();
};

std::vector<ContingencyDefinition> enumerate_n1_contingencies(
    const HybridPowerSystem& sys,
    const ReliabilityScenarioOptions& options = {});

RegularScenarioResult generate_regular_scenarios(
    const HybridPowerSystem& sys,
    const RegularScenarioOptions& options = {},
    const PerturbationOptions& perturbation = {},
    const ClusteringOptions& clustering = {},
    std::vector<std::string>* warnings = nullptr);

ReliabilityScenarioResult generate_reliability_scenarios(
    const HybridPowerSystem& sys,
    const ReliabilityScenarioOptions& options = {},
    const PerturbationOptions& perturbation = {},
    const ClusteringOptions& clustering = {},
    std::vector<std::string>* warnings = nullptr);

ResilienceScenarioResult generate_resilience_scenarios(
    const HybridPowerSystem& sys,
    const ResilienceScenarioOptions& options = {},
    const PerturbationOptions& perturbation = {},
    const TyphoonImpactOptions& typhoon_impact = {},
    const ClusteringOptions& clustering = {},
    std::vector<std::string>* warnings = nullptr);

ScenarioGenerationResult generate_scenarios(
    const HybridPowerSystem& sys,
    const ScenarioGenerationOptions& options = {});

ScenarioGenerationOptions scenario_generation_options_from_json(const nlohmann::json& j);
nlohmann::json scenario_generation_result_to_json(const ScenarioGenerationResult& result);

const char* to_string(ScenarioFamily family);
const char* to_string(ContingencyComponentType type);
ContingencyComponentType contingency_component_type_from_string(const std::string& value);

}  // namespace hacdcpf::analysis

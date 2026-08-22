#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/scenario_generation.hpp"
#include "hacdcpf/analysis/typhoon_traffic_impact.hpp"

namespace {

using nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

HybridPowerSystem make_system() {
  HybridPowerSystem sys;
  ACBus source;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  source.vm_pu = 1.0;
  source.latitude = 22.75;
  source.longitude = 113.55;
  source.in_service = true;
  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.vm_pu = 1.0;
  load_bus.latitude = 22.75;
  load_bus.longitude = 113.95;
  load_bus.in_service = true;
  sys.ac.buses = {source, load_bus};

  ACBranch branch;
  branch.index = 101;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.length_km = 2.0;
  branch.r_pu = 0.01;
  branch.x_pu = 0.02;
  branch.in_service = true;
  sys.ac.branches = {branch};

  Load load;
  load.index = 201;
  load.bus = 2;
  load.p_mw = 10.0;
  load.q_mvar = 2.0;
  load.scaling = 1.0;
  load.in_service = true;
  sys.ac.loads = {load};

  PVSystem pv;
  pv.index = 301;
  pv.bus = 2;
  pv.p_mw = 4.0;
  pv.pmax_mw = 5.0;
  pv.in_service = true;
  sys.ac.pv_systems = {pv};

  RenewableGen wind;
  wind.index = 302;
  wind.bus = 2;
  wind.type = RenewableType::Wind;
  wind.p_mw = 3.0;
  wind.p_rated_mw = 3.0;
  wind.in_service = true;
  sys.ac.renewable_gens = {wind};

  Generator generator;
  generator.index = 401;
  generator.bus = 1;
  generator.pg_mw = 10.0;
  generator.pmax_mw = 20.0;
  generator.in_service = true;
  sys.ac.generators = {generator};
  return sys;
}

TyphoonTrackPoint fixed_storm(double hour) {
  constexpr double omega = 7.2921e-5;
  constexpr double pi = 3.14159265358979323846;
  TyphoonTrackPoint point;
  point.hour = hour;
  point.latitude = 22.75;
  point.longitude = 113.55;
  point.fc = 2.0 * omega * std::sin(point.latitude * pi / 180.0);
  point.delta_p_hpa = 70.0;
  point.holland_b = 1.2;
  point.rmw_km = 40.0;
  point.heading_deg = 0.0;
  point.translation_speed_kmph = 18.0;
  point.vmax_ms = 45.0;
  return point;
}

json storm_json(const TyphoonTrackPoint& point) {
  return {{"hour", point.hour},
          {"latitude", point.latitude},
          {"longitude", point.longitude},
          {"fc", point.fc},
          {"delta_p_hpa", point.delta_p_hpa},
          {"holland_b", point.holland_b},
          {"rmw_km", point.rmw_km},
          {"heading_deg", point.heading_deg},
          {"translation_speed_kmph", point.translation_speed_kmph},
          {"vmax_ms", point.vmax_ms}};
}

double pearson(const json& samples, const std::string& x_key,
               const std::string& y_key) {
  std::vector<double> x;
  std::vector<double> y;
  for (const auto& sample : samples) {
    x.push_back(sample.at("features").at(x_key).get<double>());
    y.push_back(sample.at("features").at(y_key).get<double>());
  }
  const double mx =
      std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(x.size());
  const double my =
      std::accumulate(y.begin(), y.end(), 0.0) / static_cast<double>(y.size());
  double numerator = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double dx = x[i] - mx;
    const double dy = y[i] - my;
    numerator += dx * dy;
    sx += dx * dx;
    sy += dy * dy;
  }
  return numerator / std::sqrt(sx * sy);
}

double pearson(const std::vector<double>& x, const std::vector<double>& y) {
  if (x.size() != y.size() || x.size() < 2) {
    throw std::invalid_argument("pearson requires equal nontrivial vectors");
  }
  const double mx =
      std::accumulate(x.begin(), x.end(), 0.0) / static_cast<double>(x.size());
  const double my =
      std::accumulate(y.begin(), y.end(), 0.0) / static_cast<double>(y.size());
  double numerator = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const double dx = x[i] - mx;
    const double dy = y[i] - my;
    numerator += dx * dy;
    sx += dx * dx;
    sy += dy * dy;
  }
  return numerator / std::sqrt(sx * sy);
}

std::vector<double> profile_values(const TimeSeriesData& data,
                                   const std::string& name) {
  for (const auto& profile : data.profiles) {
    if (profile.name == name) return profile.values;
  }
  throw std::runtime_error("missing profile " + name);
}

json descriptive_statistics(const std::vector<double>& values) {
  if (values.empty()) throw std::invalid_argument("empty statistics input");
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                      static_cast<double>(values.size());
  double squared = 0.0;
  for (double value : values) squared += (value - mean) * (value - mean);
  const auto [minimum, maximum] =
      std::minmax_element(values.begin(), values.end());
  return {{"count", values.size()},
          {"mean", mean},
          {"sample_stddev",
           std::sqrt(squared / static_cast<double>(values.size() - 1))},
          {"minimum", *minimum},
          {"maximum", *maximum}};
}

std::vector<double> elementwise_ratio(const std::vector<double>& numerator,
                                      const std::vector<double>& denominator) {
  if (numerator.size() != denominator.size()) {
    throw std::invalid_argument("ratio vector size mismatch");
  }
  std::vector<double> ratio;
  ratio.reserve(numerator.size());
  for (std::size_t i = 0; i < numerator.size(); ++i) {
    if (std::abs(denominator[i]) <= 1e-12) {
      throw std::runtime_error("zero denominator in profile ratio");
    }
    ratio.push_back(numerator[i] / denominator[i]);
  }
  return ratio;
}

void write_json(const std::filesystem::path& path, const json& value) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot write " + path.string());
  output << value.dump(2) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      std::cerr << "usage: scenario_generation_review_case OUTPUT_JSON\n";
      return 2;
    }
    const auto sys = make_system();

    RegularScenarioOptions regular;
    regular.ssp = "ssp999";
    regular.year = 2090;
    regular.num_steps = 24;
    regular.candidate_count = 64;
    regular.cluster_count = 6;
    PerturbationOptions perturbation;
    perturbation.seed = 20260822U;
    perturbation.enable_climate_perturbation = false;
    perturbation.enable_storage_soc_perturbation = false;
    perturbation.load_sigma = 0.08;
    perturbation.renewable_sigma = 0.12;
    perturbation.load_min_multiplier = 0.5;
    perturbation.load_max_multiplier = 1.5;
    perturbation.renewable_min_multiplier = 0.5;
    perturbation.renewable_max_multiplier = 1.5;
    perturbation.block_hours = 1;
    perturbation.temporal_correlation = 0.75;
    perturbation.load_renewable_correlation = -0.25;
    ClusteringOptions clustering;
    clustering.method = "weighted_k_medoids";
    clustering.robust_scale = false;
    clustering.include_tail_anchors = false;
    clustering.compare_baseline = false;
    clustering.max_iterations = 50;
    clustering.seed = 17U;
    std::vector<std::string> warnings;
    const auto regular_result = generate_regular_scenarios(
        sys, regular, perturbation, clustering, &warnings);
    ScenarioGenerationResult aggregate;
    aggregate.regular = regular_result;
    aggregate.warnings = warnings;
    const auto regular_json = scenario_generation_result_to_json(aggregate);
    const auto& coverage = regular_result.audit.at("coverage_samples");

    // Box et al. (2015), stationary bivariate AR(1). A long single-candidate
    // run exposes the actual multiplier process after dividing out the exact
    // deterministic profile generated by the same model.
    RegularScenarioOptions correlation_options = regular;
    correlation_options.num_steps = 4096;
    correlation_options.candidate_count = 1;
    correlation_options.cluster_count = 1;
    PerturbationOptions correlation_perturbation = perturbation;
    correlation_perturbation.load_sigma = 0.05;
    correlation_perturbation.renewable_sigma = 0.08;
    const auto correlation_result = generate_regular_scenarios(
        sys, correlation_options, correlation_perturbation, clustering, nullptr);
    PerturbationOptions zero_perturbation = correlation_perturbation;
    zero_perturbation.enable_load_perturbation = false;
    zero_perturbation.enable_renewable_perturbation = false;
    const auto correlation_baseline = generate_regular_scenarios(
        sys, correlation_options, zero_perturbation, clustering, nullptr);
    const auto load_factors = elementwise_ratio(
        profile_values(correlation_result.clusters.front().representative.time_series,
                       "total_load_mw"),
        profile_values(correlation_baseline.clusters.front().representative.time_series,
                       "total_load_mw"));
    const auto perturbed_pv = profile_values(
        correlation_result.clusters.front().representative.time_series, "pv_mw");
    const auto baseline_pv = profile_values(
        correlation_baseline.clusters.front().representative.time_series, "pv_mw");
    const auto wind_factors = elementwise_ratio(
        profile_values(correlation_result.clusters.front().representative.time_series,
                       "wind_mw"),
        profile_values(correlation_baseline.clusters.front().representative.time_series,
                       "wind_mw"));
    std::vector<double> pv_factors;
    std::vector<double> daylight_load_factors;
    for (std::size_t i = 0; i < baseline_pv.size(); ++i) {
      if (baseline_pv[i] <= 1e-12) continue;
      pv_factors.push_back(perturbed_pv[i] / baseline_pv[i]);
      daylight_load_factors.push_back(load_factors[i]);
    }
    const std::vector<double> load_left(load_factors.begin(), load_factors.end() - 1);
    const std::vector<double> load_right(load_factors.begin() + 1, load_factors.end());

    ClusteringOptions hybrid_clustering;
    hybrid_clustering.method = "hybrid_kmedoids_tail_5pct";
    hybrid_clustering.seed = 17U;
    hybrid_clustering.boundary_fraction = 0.10;
    hybrid_clustering.regime_weight = 0.0;
    hybrid_clustering.compare_baseline = true;
    RegularScenarioOptions reduction_options = regular;
    reduction_options.candidate_count = 128;
    reduction_options.cluster_count = 12;
    const auto reduction_result = generate_regular_scenarios(
        sys, reduction_options, perturbation, hybrid_clustering, nullptr);
    const auto& reduction_audit =
        reduction_result.audit.at("clustering_audits").front();
    ScenarioGenerationResult reduction_aggregate;
    reduction_aggregate.regular = reduction_result;
    const auto reduction_json =
        scenario_generation_result_to_json(reduction_aggregate).at("regular");

    const auto repeat_result = generate_regular_scenarios(
        sys, regular, perturbation, clustering, nullptr);
    const bool deterministic_repeat =
        repeat_result.audit.at("coverage_samples") == coverage &&
        repeat_result.clusters.size() == regular_result.clusters.size();

    const auto storm = fixed_storm(0.0);
    const double site_latitude = 22.75;
    const double site_longitude = 113.95;
    const double native_wind =
        holland_wind_ms(storm, site_latitude, site_longitude);
    const double native_rain = typhoon_rainfall_mm_hr(
        storm, nullptr, site_latitude, site_longitude);

    TyphoonLineSegment review_segment;
    review_segment.branch_kind = ResilienceBranchKind::AC;
    review_segment.branch_index = 101;
    review_segment.segment_index = 0;
    review_segment.from_latitude = site_latitude;
    review_segment.from_longitude = site_longitude;
    review_segment.to_latitude = site_latitude;
    review_segment.to_longitude = site_longitude;
    review_segment.mid_latitude = site_latitude;
    review_segment.mid_longitude = site_longitude;
    review_segment.length_km = 2.0;
    TyphoonScenarioOptions fault_options;
    fault_options.horizon_hours = 2;
    fault_options.time_step_hr = 1.0;
    fault_options.seed = 20260822U;
    fault_options.use_precomputed_track = true;
    fault_options.precomputed_track = {fixed_storm(0.0), fixed_storm(1.0)};
    fault_options.min_fault_probability = 1.0;
    fault_options.staged_post_disaster_repair = false;
    fault_options.apply_pv_wind_derating = false;
    const auto fault_review = generate_typhoon_fault_sequence(
        sys, fault_options, std::vector<TyphoonLineSegment>{review_segment},
        false);
    if (fault_review.branch_risks.size() != 1) {
      throw std::runtime_error("review fragility case did not return one branch risk");
    }

    evpt::TrafficGraph traffic;
    traffic.nodes = {{1, "west", 113.93, 22.75},
                     {2, "east", 113.97, 22.75}};
    evpt::TrafficLink link;
    link.index = 501;
    link.from_node = 1;
    link.to_node = 2;
    link.length_km = 4.0;
    link.free_flow_time_hr = 0.08;
    link.capacity_veh_per_hr = 1200.0;
    link.available = true;
    traffic.links.push_back(link);
    std::vector<TyphoonTrackPoint> track;
    for (int hour = 0; hour <= 4; ++hour) {
      track.push_back(fixed_storm(static_cast<double>(hour)));
    }
    TyphoonTrafficImpactOptions traffic_options;
    traffic_options.num_steps = 4;
    traffic_options.time_step_hr = 1.0;
    traffic_options.coordinate_mode =
        TyphoonTrafficCoordinateMode::Geographic;
    traffic_options.rainfall_runoff_coefficient = 0.85;
    traffic_options.drainage_rate_mm_hr = 25.0;
    traffic_options.maximum_surface_water_mm = 500.0;
    traffic_options.flood_closure_depth_mm = 300.0;
    const auto traffic_result =
        apply_typhoon_traffic_impact(traffic, track, traffic_options);
    json traffic_steps = json::array();
    for (const auto& step : traffic_result.link_steps) {
      traffic_steps.push_back(
          {{"step", step.step},
           {"hour", step.hour},
           {"midpoint_latitude", step.midpoint_latitude},
           {"midpoint_longitude", step.midpoint_longitude},
           {"wind_ms", step.wind_ms},
           {"rainfall_mm_hr", step.rainfall_mm_hr},
           {"surface_water_mm", step.surface_water_mm},
           {"free_flow_speed_factor", step.free_flow_speed_factor},
           {"wind_capacity_factor", step.wind_capacity_factor},
           {"capacity_factor", step.capacity_factor},
           {"available", step.available}});
    }

    ReliabilityScenarioOptions reliability_options;
    const auto contingencies =
        enumerate_n1_contingencies(sys, reliability_options);
    json contingency_ids = json::array();
    for (const auto& contingency : contingencies) {
      contingency_ids.push_back(contingency.id);
    }

    const json result = {
        {"schema", "hacdcpf.scenario_generation.review.v1"},
        {"model_scope",
         "parameterized scenario-generation kernels and reduction; not a "
         "weather forecast or calibrated structural reliability model"},
        {"regular",
         {{"options",
           {{"candidate_count", regular.candidate_count},
            {"cluster_count", regular.cluster_count},
            {"num_steps", regular.num_steps},
            {"temporal_correlation", perturbation.temporal_correlation},
            {"load_renewable_correlation",
             perturbation.load_renewable_correlation},
            {"robust_scale", clustering.robust_scale}}},
          {"candidate_count", regular_result.candidate_count},
          {"cluster_count", regular_result.cluster_count},
          {"candidate_probability_sum",
           64.0 * coverage.front().at("probability").get<double>()},
          {"cluster_probability_sum",
           std::accumulate(
               regular_result.clusters.begin(), regular_result.clusters.end(),
               0.0, [](double sum, const ScenarioCluster& cluster) {
                 return sum + cluster.probability;
               })},
          {"load_pv_energy_correlation",
           pearson(coverage, "load_sum", "pv_sum")},
          {"deterministic_repeat", deterministic_repeat},
          {"ar1_process",
           {{"steps", load_factors.size()},
            {"target_temporal_correlation",
             correlation_perturbation.temporal_correlation},
            {"target_cross_correlation",
             correlation_perturbation.load_renewable_correlation},
            {"load_factor_statistics", descriptive_statistics(load_factors)},
            {"pv_factor_statistics", descriptive_statistics(pv_factors)},
            {"wind_factor_statistics", descriptive_statistics(wind_factors)},
            {"load_lag1_correlation", pearson(load_left, load_right)},
            {"load_wind_contemporaneous_correlation",
             pearson(load_factors, wind_factors)},
            {"load_pv_contemporaneous_correlation",
             pearson(daylight_load_factors, pv_factors)}}},
          {"reduction",
           {{"candidate_count", reduction_result.candidate_count},
            {"cluster_count", reduction_result.cluster_count},
            {"regime_weight", hybrid_clustering.regime_weight},
            {"audit", reduction_audit},
            {"result", reduction_json}}},
          {"result", regular_json.at("regular")}}},
        {"hazard",
         {{"storm", storm_json(storm)},
          {"site_latitude", site_latitude},
          {"site_longitude", site_longitude},
          {"holland_wind_ms", native_wind},
          {"rainfall_mm_hr", native_rain},
          {"fragility",
           {{"segment_length_km", review_segment.length_km},
            {"design_wind_ms", review_segment.fragility.design_wind_ms},
            {"collapse_wind_ms", review_segment.fragility.collapse_wind_ms},
            {"lognormal_median_wind_ms",
             review_segment.fragility.lognormal_median_wind_ms},
            {"lognormal_sigma", review_segment.fragility.lognormal_sigma},
            {"design_rain_mm_hr", review_segment.fragility.design_rain_mm_hr},
            {"seg_a_wind", review_segment.fragility.seg_a_wind},
            {"seg_b_rain", review_segment.fragility.seg_b_rain},
            {"seg_c_bias", review_segment.fragility.seg_c_bias},
            {"peak_failure_probability",
             fault_review.branch_risks.front().peak_failure_probability},
            {"sampling_gate", fault_options.min_fault_probability},
            {"fault_count", fault_review.faults.size()}}}}},
        {"traffic",
         {{"options",
           {{"time_step_hr", traffic_options.time_step_hr},
            {"rainfall_runoff_coefficient",
             traffic_options.rainfall_runoff_coefficient},
            {"drainage_rate_mm_hr", traffic_options.drainage_rate_mm_hr},
            {"maximum_surface_water_mm",
             traffic_options.maximum_surface_water_mm},
            {"depth_speed_quadratic",
             traffic_options.depth_speed_quadratic},
            {"depth_speed_linear", traffic_options.depth_speed_linear},
            {"depth_speed_intercept_km_hr",
             traffic_options.depth_speed_intercept_km_hr},
            {"minimum_open_speed_factor",
             traffic_options.minimum_open_speed_factor},
            {"wind_capacity_reduction_start_ms",
             traffic_options.wind_capacity_reduction_start_ms},
            {"wind_capacity_reduction_full_ms",
             traffic_options.wind_capacity_reduction_full_ms},
            {"minimum_wind_capacity_factor",
             traffic_options.minimum_wind_capacity_factor},
            {"capacity_speed_exponent",
             traffic_options.capacity_speed_exponent},
            {"minimum_open_capacity_factor",
             traffic_options.minimum_open_capacity_factor},
            {"flood_closure_depth_mm",
             traffic_options.flood_closure_depth_mm}}},
          {"steps", traffic_steps},
          {"closed_link_steps", traffic_result.closed_link_steps},
          {"model_scope", traffic_result.model_scope}}},
        {"reliability_catalog",
         {{"count", contingencies.size()}, {"ids", contingency_ids}}}};
    write_json(argv[1], result);
    std::cout << json{{"output", argv[1]},
                      {"candidate_count", regular_result.candidate_count},
                      {"cluster_count", regular_result.cluster_count},
                      {"holland_wind_ms", native_wind},
                      {"rainfall_mm_hr", native_rain}}
                     .dump(2)
              << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 2;
  }
}

#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hacdcpf/resilience/resilience_assessment.hpp"
#include "hacdcpf/model/system.hpp"

namespace hacdcpf::analysis {

enum class TyphoonAssetKind {
  OverheadLine,
  Cable,
  Mixed,
};

enum class TyphoonIntensityCategory {
  TD,
  TS,
  STS,
  TY,
  STY,
  SuperTY,
  Unknown,
};

struct TyphoonTrackPoint {
  double hour{0.0};
  double latitude{20.5};
  double longitude{114.5};
  double fc{0.0};
  double delta_p_hpa{44.0};
  double holland_b{1.2};
  double rmw_km{40.0};
  double heading_deg{0.0};
  double translation_speed_kmph{21.6};
  double vmax_ms{0.0};
  double sst_c{0.0};
};

struct TyphoonSegmentFragility {
  TyphoonAssetKind kind{TyphoonAssetKind::OverheadLine};
  double design_wind_ms{30.0};
  double collapse_wind_ms{55.0};
  double lognormal_median_wind_ms{45.0};
  double lognormal_sigma{0.25};

  // Segment wind-rain hazard model, inspired by the standalone typhoon
  // scenario generator. These defaults are intentionally conservative and
  // are used only when case data does not provide calibrated tower/segment
  // fragility tables.
  double design_rain_mm_hr{60.0};
  double seg_a_wind{3.0};
  double seg_b_rain{1.0};
  double seg_c_bias{-7.0};

  // Cable/rain hydrology model defaults.
  double curve_number_cn{75.0};
  double manning_n{0.035};
  double slope_s0{0.01};
  double soil_depth_z_m{1.0};
  double porosity_phi{0.45};
  double initial_moisture_theta{0.25};
  double base_cable_failure_p{1e-4};
};

struct TyphoonLineSegment {
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  int segment_index{0};
  double from_latitude{0.0};
  double from_longitude{0.0};
  double to_latitude{0.0};
  double to_longitude{0.0};
  double mid_latitude{0.0};
  double mid_longitude{0.0};
  double length_km{0.0};
  TyphoonSegmentFragility fragility;
};

struct TyphoonScenarioOptions {
  TyphoonScenarioOptions();

  int horizon_hours{48};
  double time_step_hr{1.0};
  unsigned int seed{2030};
  bool stochastic{false};

  bool use_precomputed_track{false};
  std::vector<TyphoonTrackPoint> precomputed_track;
  TyphoonIntensityCategory requested_category{TyphoonIntensityCategory::Unknown};
  TyphoonIntensityCategory selected_category{TyphoonIntensityCategory::Unknown};
  double selected_track_max_vmax_ms{0.0};
  std::string selected_sample_id;
  bool used_category_fallback{false};

  int month{8};
  bool use_month_defaults{false};
  bool use_sst_resource{true};
  std::string sst_resource_path;
  double initial_latitude{20.5};
  double initial_longitude{114.5};
  double initial_delta_p_hpa{44.0};
  double initial_rmw_km{0.0};
  double initial_heading_deg{0.0};
  double initial_translation_speed_kmph{21.6};

  // Auto segmentation.
  bool auto_segment_lines{true};
  double target_segment_length_km{0.5};
  int min_segments_per_branch{1};
  int max_segments_per_branch{40};
  double fallback_case_center_latitude{22.75};
  double fallback_case_center_longitude{113.55};
  double fallback_bus_spacing_km{0.8};

  // Asset mix defaults when the case lacks overhead/cable metadata.
  double default_overhead_fraction{1.0};
  double default_cable_fraction{0.0};
  TyphoonSegmentFragility default_overhead_fragility;
  TyphoonSegmentFragility default_cable_fragility;

  // Fault conversion.
  double min_fault_probability{0.05};
  double default_repair_time_hr{6.0};
  double repair_time_wind_factor{0.10};
  double max_repair_time_hr{24.0};
  bool staged_post_disaster_repair{true};
  double post_disaster_repair_delay_hr{1.0};
  double repair_crew_time_per_branch_hr{1.0};

  // Optional renewable impact.
  bool apply_pv_wind_derating{true};
  double pv_cutout_start_ms{25.0};
  double pv_cutout_full_ms{50.0};
  double pv_max_reduction{0.40};
  int pv_transition_hours{3};
};

struct TyphoonBranchRisk {
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  double peak_wind_ms{0.0};
  double peak_rain_mm_hr{0.0};
  double peak_failure_probability{0.0};
};

struct TyphoonTrackSample {
  std::string sample_id;
  int month{8};
  unsigned int seed{0};
  bool stochastic{true};
  double max_vmax_ms{0.0};
  TyphoonIntensityCategory category{TyphoonIntensityCategory::Unknown};
  double monthly_sst_c{0.0};
  std::string sst_resource_path;
  std::vector<TyphoonTrackPoint> track;
};

struct TyphoonCatalogOptions {
  int samples_per_month{100};
  int first_month{1};
  int last_month{12};
  unsigned int base_seed{203000};
  int horizon_hours{48};
  double time_step_hr{1.0};
  bool stochastic{true};
  bool use_month_defaults{true};
  bool use_sst_resource{true};
  std::string sst_resource_path;
  std::string catalog_path;
};

struct TyphoonCatalog {
  std::vector<TyphoonTrackSample> samples;
  std::map<TyphoonIntensityCategory, std::vector<std::size_t>> by_category;
  std::map<TyphoonIntensityCategory, int> category_counts;
  std::string source_path;
  bool loaded_from_disk{false};
};

struct TyphoonFaultSequenceResult {
  std::vector<TyphoonTrackPoint> track;
  std::vector<TyphoonLineSegment> generated_segments;
  std::vector<DistributionResilienceFault> faults;

  std::vector<int> branch_indices;
  std::vector<double> branch_peak_wind_ms;
  std::vector<double> branch_peak_rain_mm_hr;
  std::vector<double> branch_peak_failure_probability;
  std::vector<TyphoonBranchRisk> branch_risks;
  std::vector<double> renewable_profile_multiplier;

  bool used_fallback_coordinates{false};
  bool used_synthetic_segments{false};
  bool used_catalog_sample{false};
  bool used_category_fallback{false};
  bool used_approximate_repair_order{false};
  std::size_t repair_fault_count{0};
  TyphoonIntensityCategory requested_category{TyphoonIntensityCategory::Unknown};
  TyphoonIntensityCategory selected_category{TyphoonIntensityCategory::Unknown};
  double selected_track_max_vmax_ms{0.0};
  std::string selected_sample_id;
  double monthly_sst_c{0.0};
  std::string sst_resource_path;
  unsigned int seed{0};
  std::string status;
};

std::vector<TyphoonLineSegment> generate_typhoon_line_segments(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts = {},
    bool* used_fallback_coordinates = nullptr);

TyphoonFaultSequenceResult generate_typhoon_fault_sequence(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts = {});

TyphoonFaultSequenceResult generate_typhoon_fault_sequence(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts,
    const std::vector<TyphoonLineSegment>& precomputed_segments,
    bool used_fallback_coordinates);

TyphoonIntensityCategory classify_typhoon_intensity(double max_vmax_ms);
double holland_wind_ms(const TyphoonTrackPoint& storm, double site_lat, double site_lon);
double typhoon_rainfall_mm_hr(const TyphoonTrackPoint& storm,
                             const TyphoonTrackPoint* previous,
                             double site_lat,
                             double site_lon);
double max_track_vmax_ms(const std::vector<TyphoonTrackPoint>& track, bool skip_first_point = true);
TyphoonTrackSample generate_typhoon_track_sample(const TyphoonScenarioOptions& opts = {});
TyphoonCatalog build_typhoon_catalog(const TyphoonCatalogOptions& opts = {});
std::shared_ptr<const TyphoonCatalog> get_or_build_typhoon_catalog(
    const TyphoonCatalogOptions& opts = {});
std::shared_ptr<const TyphoonTrackSample> sample_typhoon_catalog(
    const std::shared_ptr<const TyphoonCatalog>& catalog,
    TyphoonIntensityCategory category,
    unsigned int selection_seed);
TyphoonIntensityCategory typhoon_intensity_category_from_string(const std::string& value);

const char* to_string(TyphoonAssetKind kind);
const char* to_string(TyphoonIntensityCategory category);
const char* to_zh_name(TyphoonIntensityCategory category);

}  // namespace hacdcpf::analysis

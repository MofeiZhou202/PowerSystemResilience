#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/analysis/typhoon_resilience.hpp"
#include "hacdcpf/ev_power_traffic/types.hpp"

namespace hacdcpf::analysis {

enum class TyphoonTrafficCoordinateMode {
  Auto,
  Geographic,
  AffineToStudyArea,
};

struct TyphoonTrafficImpactOptions {
  int num_steps{24};
  double time_step_hr{0.25};
  double start_hour{0.0};

  TyphoonTrafficCoordinateMode coordinate_mode{
      TyphoonTrafficCoordinateMode::Auto};
  double study_center_latitude{22.75};
  double study_center_longitude{113.55};
  double affine_network_span_km{20.0};
  double auto_geographic_track_distance_km{500.0};

  // Zero-dimensional surface-water balance on each road link:
  // d_{t+1} = max(0, d_t + (c_r r_t - q_d) Delta t).
  double rainfall_runoff_coefficient{0.85};
  double drainage_rate_mm_hr{25.0};
  double maximum_surface_water_mm{500.0};
  std::unordered_map<int, double> drainage_rate_by_link_mm_hr;
  std::unordered_map<int, double> runoff_multiplier_by_link;

  // Pregnolato et al. flood depth--speed curve, with depth in mm and speed
  // in km/h. The resulting factor is normalized by the zero-depth value and
  // applied to each link's own free-flow speed.
  double depth_speed_quadratic{0.0009};
  double depth_speed_linear{-0.5529};
  double depth_speed_intercept_km_hr{86.9448};
  double minimum_open_speed_factor{0.05};
  double flood_closure_depth_mm{300.0};

  // Wind represents debris, lane restrictions, and operational controls.
  double wind_capacity_reduction_start_ms{15.0};
  double wind_capacity_reduction_full_ms{35.0};
  double minimum_wind_capacity_factor{0.65};
  bool close_on_extreme_wind{false};
  double wind_closure_ms{45.0};

  bool apply_free_flow_time_profile{true};
  bool apply_capacity_profile{true};
  bool apply_availability_profile{true};
  double capacity_speed_exponent{1.0};
  double minimum_open_capacity_factor{0.02};
};

struct TyphoonTrafficLinkStep {
  int link_index{0};
  int step{0};
  double hour{0.0};
  double midpoint_latitude{0.0};
  double midpoint_longitude{0.0};
  double wind_ms{0.0};
  double rainfall_mm_hr{0.0};
  double surface_water_mm{0.0};
  double free_flow_speed_factor{1.0};
  double wind_capacity_factor{1.0};
  double capacity_factor{1.0};
  bool available{true};
};

struct TyphoonTrafficImpactResult {
  evpt::TrafficGraph impacted_traffic;
  std::vector<TyphoonTrafficLinkStep> link_steps;
  std::vector<int> links_closed_at_least_once;
  bool used_affine_georeferencing{false};
  double peak_wind_ms{0.0};
  double peak_rainfall_mm_hr{0.0};
  double peak_surface_water_mm{0.0};
  double minimum_speed_factor{1.0};
  double minimum_capacity_factor{1.0};
  int closed_link_steps{0};
  std::string model_scope;
};

TyphoonTrafficImpactResult apply_typhoon_traffic_impact(
    const evpt::TrafficGraph& traffic,
    const std::vector<TyphoonTrackPoint>& track,
    const TyphoonTrafficImpactOptions& options = {});

const char* to_string(TyphoonTrafficCoordinateMode mode);

}  // namespace hacdcpf::analysis

#include "hacdcpf/analysis/typhoon_traffic_impact.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace hacdcpf::analysis {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

double clamp01(double value) {
  return std::clamp(value, 0.0, 1.0);
}

double deg2rad(double value) {
  return value * kPi / 180.0;
}

double haversine_km(double lat1, double lon1, double lat2, double lon2) {
  constexpr double radius_km = 6371.0;
  const double dlat = deg2rad(lat2 - lat1);
  const double dlon = deg2rad(lon2 - lon1);
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
                       std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  return 2.0 * radius_km *
         std::asin(std::sqrt(std::clamp(a, 0.0, 1.0)));
}

bool finite_coordinate(const evpt::TrafficNode& node) {
  return std::isfinite(node.x) && std::isfinite(node.y);
}

bool plausible_geographic_coordinate(const evpt::TrafficNode& node) {
  return finite_coordinate(node) && node.x >= -180.0 && node.x <= 180.0 &&
         node.y >= -90.0 && node.y <= 90.0;
}

TyphoonTrackPoint interpolate_track(
    const std::vector<TyphoonTrackPoint>& track,
    double hour) {
  if (track.empty()) {
    throw std::invalid_argument(
        "apply_typhoon_traffic_impact requires a nonempty typhoon track");
  }
  if (hour <= track.front().hour) return track.front();
  if (hour >= track.back().hour) return track.back();
  const auto upper = std::upper_bound(
      track.begin(), track.end(), hour,
      [](double value, const TyphoonTrackPoint& point) {
        return value < point.hour;
      });
  const auto lower = std::prev(upper);
  const double span = std::max(kTol, upper->hour - lower->hour);
  const double weight = clamp01((hour - lower->hour) / span);
  auto lerp = [weight](double a, double b) {
    return a + weight * (b - a);
  };
  TyphoonTrackPoint point;
  point.hour = hour;
  point.latitude = lerp(lower->latitude, upper->latitude);
  point.longitude = lerp(lower->longitude, upper->longitude);
  point.fc = lerp(lower->fc, upper->fc);
  point.delta_p_hpa = lerp(lower->delta_p_hpa, upper->delta_p_hpa);
  point.holland_b = lerp(lower->holland_b, upper->holland_b);
  point.rmw_km = lerp(lower->rmw_km, upper->rmw_km);
  point.heading_deg = lerp(lower->heading_deg, upper->heading_deg);
  point.translation_speed_kmph =
      lerp(lower->translation_speed_kmph, upper->translation_speed_kmph);
  point.vmax_ms = lerp(lower->vmax_ms, upper->vmax_ms);
  point.sst_c = lerp(lower->sst_c, upper->sst_c);
  return point;
}

struct GeoPoint {
  double latitude{0.0};
  double longitude{0.0};
};

std::unordered_map<int, GeoPoint> georeference_nodes(
    const evpt::TrafficGraph& traffic,
    const std::vector<TyphoonTrackPoint>& track,
    const TyphoonTrafficImpactOptions& options,
    bool& used_affine) {
  if (traffic.nodes.empty()) {
    throw std::invalid_argument(
        "apply_typhoon_traffic_impact requires traffic nodes");
  }
  bool use_geographic =
      options.coordinate_mode == TyphoonTrafficCoordinateMode::Geographic;
  if (options.coordinate_mode == TyphoonTrafficCoordinateMode::Auto) {
    use_geographic = std::all_of(
        traffic.nodes.begin(), traffic.nodes.end(),
        plausible_geographic_coordinate);
    if (use_geographic && !track.empty()) {
      double minimum_distance = std::numeric_limits<double>::infinity();
      for (const auto& node : traffic.nodes) {
        for (const auto& storm : track) {
          minimum_distance = std::min(
              minimum_distance,
              haversine_km(node.y, node.x, storm.latitude, storm.longitude));
        }
      }
      use_geographic = minimum_distance <=
                       options.auto_geographic_track_distance_km;
    }
  }
  if (options.coordinate_mode ==
      TyphoonTrafficCoordinateMode::AffineToStudyArea) {
    use_geographic = false;
  }

  std::unordered_map<int, GeoPoint> result;
  if (use_geographic) {
    for (const auto& node : traffic.nodes) {
      if (!plausible_geographic_coordinate(node)) {
        throw std::invalid_argument(
            "geographic traffic mode requires longitude x and latitude y");
      }
      result[node.index] = {node.y, node.x};
    }
    used_affine = false;
    return result;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& node : traffic.nodes) {
    if (!finite_coordinate(node)) {
      throw std::invalid_argument(
          "affine traffic georeferencing requires finite node x/y values");
    }
    min_x = std::min(min_x, node.x);
    max_x = std::max(max_x, node.x);
    min_y = std::min(min_y, node.y);
    max_y = std::max(max_y, node.y);
  }
  const double center_x = 0.5 * (min_x + max_x);
  const double center_y = 0.5 * (min_y + max_y);
  const double coordinate_span =
      std::max({max_x - min_x, max_y - min_y, kTol});
  const double scale_km =
      std::max(kTol, options.affine_network_span_km) / coordinate_span;
  const double cos_lat = std::max(
      0.2, std::cos(deg2rad(options.study_center_latitude)));
  for (const auto& node : traffic.nodes) {
    const double east_km = (node.x - center_x) * scale_km;
    const double north_km = (node.y - center_y) * scale_km;
    result[node.index] = {
        options.study_center_latitude + north_km / 111.0,
        options.study_center_longitude + east_km / (111.0 * cos_lat)};
  }
  used_affine = true;
  return result;
}

double wind_capacity_factor(double wind_ms,
                            const TyphoonTrafficImpactOptions& options) {
  if (wind_ms <= options.wind_capacity_reduction_start_ms) return 1.0;
  const double span = std::max(
      kTol, options.wind_capacity_reduction_full_ms -
                options.wind_capacity_reduction_start_ms);
  const double severity = clamp01(
      (wind_ms - options.wind_capacity_reduction_start_ms) / span);
  return 1.0 - severity *
                   (1.0 - clamp01(options.minimum_wind_capacity_factor));
}

double flood_speed_factor(double depth_mm,
                          const TyphoonTrafficImpactOptions& options) {
  const double reference = std::max(
      kTol, options.depth_speed_intercept_km_hr);
  const double speed = options.depth_speed_quadratic * depth_mm * depth_mm +
                       options.depth_speed_linear * depth_mm +
                       options.depth_speed_intercept_km_hr;
  return std::clamp(speed / reference,
                    options.minimum_open_speed_factor, 1.0);
}

double map_value_or(const std::unordered_map<int, double>& values,
                    int index,
                    double fallback) {
  const auto it = values.find(index);
  return it == values.end() ? fallback : it->second;
}

}  // namespace

TyphoonTrafficImpactResult apply_typhoon_traffic_impact(
    const evpt::TrafficGraph& traffic,
    const std::vector<TyphoonTrackPoint>& track,
    const TyphoonTrafficImpactOptions& options) {
  if (options.num_steps <= 0 || options.time_step_hr <= 0.0) {
    throw std::invalid_argument(
        "typhoon traffic impact requires positive horizon and time step");
  }
  if (options.rainfall_runoff_coefficient < 0.0 ||
      options.drainage_rate_mm_hr < 0.0 ||
      options.flood_closure_depth_mm <= 0.0 ||
      options.depth_speed_intercept_km_hr <= 0.0) {
    throw std::invalid_argument(
        "typhoon traffic impact parameters are outside their physical range");
  }

  TyphoonTrafficImpactResult result;
  result.impacted_traffic = traffic;
  const auto coordinates = georeference_nodes(
      traffic, track, options, result.used_affine_georeferencing);

  std::vector<TyphoonTrackPoint> sampled_track;
  sampled_track.reserve(static_cast<std::size_t>(options.num_steps));
  for (int step = 0; step < options.num_steps; ++step) {
    sampled_track.push_back(interpolate_track(
        track, options.start_hour + step * options.time_step_hr));
  }

  std::set<int> closed_links;
  result.link_steps.reserve(
      result.impacted_traffic.links.size() *
      static_cast<std::size_t>(options.num_steps));
  for (auto& link : result.impacted_traffic.links) {
    const auto from = coordinates.find(link.from_node);
    const auto to = coordinates.find(link.to_node);
    if (from == coordinates.end() || to == coordinates.end()) {
      throw std::invalid_argument(
          "traffic link references a node without georeferenced coordinates");
    }
    const double midpoint_lat =
        0.5 * (from->second.latitude + to->second.latitude);
    const double midpoint_lon =
        0.5 * (from->second.longitude + to->second.longitude);
    const double base_time_hr = std::max(kTol, link.free_flow_time_hr);
    const double base_capacity = std::max(0.0, link.capacity_veh_per_hr);
    double surface_water_mm = 0.0;

    std::vector<double> time_profile(
        static_cast<std::size_t>(options.num_steps), base_time_hr);
    std::vector<double> capacity_profile(
        static_cast<std::size_t>(options.num_steps), base_capacity);
    std::vector<bool> availability_profile(
        static_cast<std::size_t>(options.num_steps), link.available);

    const double drainage = std::max(
        0.0, map_value_or(options.drainage_rate_by_link_mm_hr,
                          link.index, options.drainage_rate_mm_hr));
    const double runoff_multiplier = std::max(
        0.0, map_value_or(options.runoff_multiplier_by_link,
                          link.index, 1.0));

    for (int step = 0; step < options.num_steps; ++step) {
      const auto& storm = sampled_track[static_cast<std::size_t>(step)];
      const TyphoonTrackPoint* previous =
          step > 0 ? &sampled_track[static_cast<std::size_t>(step - 1)]
                   : nullptr;
      const double wind = holland_wind_ms(storm, midpoint_lat, midpoint_lon);
      const double rain = typhoon_rainfall_mm_hr(
          storm, previous, midpoint_lat, midpoint_lon);
      const double inflow_mm_hr =
          options.rainfall_runoff_coefficient * runoff_multiplier * rain;
      surface_water_mm = std::clamp(
          surface_water_mm + (inflow_mm_hr - drainage) * options.time_step_hr,
          0.0, options.maximum_surface_water_mm);

      const double speed_factor =
          flood_speed_factor(surface_water_mm, options);
      const double wind_factor = wind_capacity_factor(wind, options);
      const double flood_capacity_factor = std::pow(
          speed_factor, std::max(0.0, options.capacity_speed_exponent));
      const double capacity_factor = std::clamp(
          flood_capacity_factor * wind_factor,
          options.minimum_open_capacity_factor, 1.0);
      const bool flood_closed =
          surface_water_mm >= options.flood_closure_depth_mm;
      const bool wind_closed = options.close_on_extreme_wind &&
                               wind >= options.wind_closure_ms;
      const bool available = link.available && !flood_closed && !wind_closed;

      if (options.apply_free_flow_time_profile) {
        time_profile[static_cast<std::size_t>(step)] =
            base_time_hr / std::max(options.minimum_open_speed_factor,
                                    speed_factor);
      }
      if (options.apply_capacity_profile) {
        capacity_profile[static_cast<std::size_t>(step)] =
            available ? base_capacity * capacity_factor : 0.0;
      }
      if (options.apply_availability_profile) {
        availability_profile[static_cast<std::size_t>(step)] = available;
      }

      result.peak_wind_ms = std::max(result.peak_wind_ms, wind);
      result.peak_rainfall_mm_hr =
          std::max(result.peak_rainfall_mm_hr, rain);
      result.peak_surface_water_mm =
          std::max(result.peak_surface_water_mm, surface_water_mm);
      result.minimum_speed_factor =
          std::min(result.minimum_speed_factor, speed_factor);
      result.minimum_capacity_factor =
          std::min(result.minimum_capacity_factor, capacity_factor);
      if (!available) {
        ++result.closed_link_steps;
        closed_links.insert(link.index);
      }
      result.link_steps.push_back(
          {link.index,
           step,
           options.start_hour + step * options.time_step_hr,
           midpoint_lat,
           midpoint_lon,
           wind,
           rain,
           surface_water_mm,
           speed_factor,
           wind_factor,
           capacity_factor,
           available});
    }

    if (options.apply_free_flow_time_profile) {
      link.free_flow_time_profile_hr = std::move(time_profile);
    }
    if (options.apply_capacity_profile) {
      link.capacity_profile_veh_per_hr = std::move(capacity_profile);
    }
    if (options.apply_availability_profile) {
      link.availability_profile = std::move(availability_profile);
    }
  }

  result.links_closed_at_least_once.assign(
      closed_links.begin(), closed_links.end());
  result.model_scope =
      "Holland wind and parametric typhoon rainfall mapped to link exposure; "
      "zero-dimensional runoff-drainage storage; flood depth-speed disruption; "
      "wind/flood capacity reduction and threshold closure";
  if (result.used_affine_georeferencing) {
    result.model_scope +=
        "; benchmark topology affinely georeferenced to the study area";
  }
  return result;
}

const char* to_string(TyphoonTrafficCoordinateMode mode) {
  switch (mode) {
    case TyphoonTrafficCoordinateMode::Auto: return "auto";
    case TyphoonTrafficCoordinateMode::Geographic: return "geographic";
    case TyphoonTrafficCoordinateMode::AffineToStudyArea:
      return "affine_to_study_area";
  }
  return "unknown";
}

}  // namespace hacdcpf::analysis

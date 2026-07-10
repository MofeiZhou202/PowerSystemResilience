#include "hacdcpf/analysis/typhoon_resilience.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <cctype>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <set>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace hacdcpf::analysis {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEarthRadiusKm = 6371.0088;
constexpr double kOmega = 7.2921e-5;
constexpr double kAirDensity = 1.15;
constexpr double kGradientHeightM = 275.0;
constexpr double kAnemometerHeightM = 10.0;
constexpr double kPowerLawExponent = 1.0 / 7.0;
constexpr double kGustFactor = 1.75;
constexpr double kRainRadiusFloorKm = 1.0;
constexpr double kRainRatioMax = 1.1;
constexpr std::size_t kMaxExactStagedRepairFaults = 64;

struct BusGeo {
  double lat{0.0};
  double lon{0.0};
  bool fallback{false};
};

struct BranchRisk {
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  double peak_wind_ms{0.0};
  double peak_rain_mm_hr{0.0};
  double peak_probability{0.0};
  double first_fault_hr{0.0};
  bool faulted{false};
};

struct RepairCandidate {
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  double fault_start_hr{0.0};
  double impact_mw{0.0};
  double peak_probability{0.0};
  double peak_wind_ms{0.0};
};

struct SstClimatology {
  std::array<double, 12> monthly_mean_sst_c{};
  std::filesystem::path source_path;
};

struct BranchKey {
  ResilienceBranchKind kind{ResilienceBranchKind::AC};
  int index{0};

  bool operator<(const BranchKey& other) const {
    if (kind != other.kind) return kind < other.kind;
    return index < other.index;
  }
};

double deg2rad(double deg) { return deg * kPi / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / kPi; }

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

int clamp_int(int value, int lo, int hi) {
  return std::max(lo, std::min(hi, value));
}

std::filesystem::path project_root_path() {
#ifdef HACDCPF_PROJECT_ROOT
  return std::filesystem::path(HACDCPF_PROJECT_ROOT);
#else
  return std::filesystem::current_path();
#endif
}

std::filesystem::path resolve_sst_resource_path(const std::string& configured_path = {}) {
  namespace fs = std::filesystem;
  if (!configured_path.empty()) return fs::path(configured_path);

  const fs::path relative = fs::path("external_data") / "typhoon" / "sst_monthly_south_china_sea.json";
  std::vector<fs::path> candidates;
  candidates.push_back(project_root_path() / relative);
  candidates.push_back(fs::current_path() / relative);
  candidates.push_back(fs::current_path().parent_path() / relative);
  candidates.push_back(fs::current_path().parent_path().parent_path() / relative);

  for (const auto& candidate : candidates) {
    std::error_code ec;
    if (fs::is_regular_file(candidate, ec)) return candidate;
  }
  return candidates.front();
}

SstClimatology load_sst_climatology(const std::string& configured_path = {}) {
  const auto path = resolve_sst_resource_path(configured_path);
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Typhoon SST resource not found: " + path.string());
  }

  nlohmann::json j;
  in >> j;

  SstClimatology sst;
  sst.source_path = path;
  if (j.contains("monthly_mean_sst_c_array") && j["monthly_mean_sst_c_array"].is_array()) {
    const auto& arr = j["monthly_mean_sst_c_array"];
    if (arr.size() != sst.monthly_mean_sst_c.size()) {
      throw std::runtime_error("Typhoon SST resource must contain exactly 12 monthly values: " + path.string());
    }
    for (std::size_t i = 0; i < sst.monthly_mean_sst_c.size(); ++i) {
      sst.monthly_mean_sst_c[i] = arr.at(i).get<double>();
    }
  } else if (j.contains("monthly_mean_sst_c") && j["monthly_mean_sst_c"].is_object()) {
    const auto& obj = j["monthly_mean_sst_c"];
    for (std::size_t i = 0; i < sst.monthly_mean_sst_c.size(); ++i) {
      const auto key = std::to_string(i + 1);
      if (!obj.contains(key)) {
        throw std::runtime_error("Typhoon SST resource missing month " + key + ": " + path.string());
      }
      sst.monthly_mean_sst_c[i] = obj.at(key).get<double>();
    }
  } else {
    throw std::runtime_error("Typhoon SST resource missing monthly_mean_sst_c_array/monthly_mean_sst_c: " + path.string());
  }

  for (double value : sst.monthly_mean_sst_c) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Typhoon SST resource contains non-finite monthly value: " + path.string());
    }
  }
  return sst;
}

bool valid_coordinate(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) &&
         std::abs(lat) > 1e-9 && std::abs(lon) > 1e-9 &&
         lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

double haversine_km(double lat1, double lon1, double lat2, double lon2) {
  const double dlat = deg2rad(lat2 - lat1);
  const double dlon = deg2rad(lon2 - lon1);
  const double rlat1 = deg2rad(lat1);
  const double rlat2 = deg2rad(lat2);
  const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(rlat1) * std::cos(rlat2) *
                       std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
  return kEarthRadiusKm * c;
}

std::pair<double, double> offset_coordinate(double center_lat,
                                            double center_lon,
                                            double east_km,
                                            double north_km) {
  const double lat = center_lat + north_km / 111.0;
  const double cos_lat = std::max(0.2, std::cos(deg2rad(center_lat)));
  const double lon = center_lon + east_km / (111.0 * cos_lat);
  return {lat, lon};
}

std::unordered_map<int, const ACBus*> make_bus_map(const HybridPowerSystem& sys) {
  std::unordered_map<int, const ACBus*> out;
  for (const auto& bus : sys.ac.buses) out[bus.index] = &bus;
  return out;
}

std::map<int, BusGeo> build_bus_geo(const HybridPowerSystem& sys,
                                    const TyphoonScenarioOptions& opts,
                                    bool* used_fallback) {
  std::map<int, BusGeo> geo;
  bool any_valid = false;
  for (const auto& bus : sys.ac.buses) {
    if (valid_coordinate(bus.latitude, bus.longitude)) {
      any_valid = true;
      geo[bus.index] = {bus.latitude, bus.longitude, false};
    }
  }
  if (any_valid) {
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& bus = sys.ac.buses[i];
      if (geo.count(bus.index)) continue;
      const double angle = 2.0 * kPi * static_cast<double>(i) /
                           std::max<std::size_t>(1, sys.ac.buses.size());
      const double radius = opts.fallback_bus_spacing_km *
                            (1.0 + static_cast<double>(i) / 8.0);
      auto [lat, lon] = offset_coordinate(opts.fallback_case_center_latitude,
                                          opts.fallback_case_center_longitude,
                                          radius * std::cos(angle),
                                          radius * std::sin(angle));
      geo[bus.index] = {lat, lon, true};
      if (used_fallback) *used_fallback = true;
    }
    return geo;
  }

  if (used_fallback) *used_fallback = !sys.ac.buses.empty();
  const int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(sys.ac.buses.size())))));
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const int row = static_cast<int>(i) / cols;
    const int col = static_cast<int>(i) % cols;
    const double east = (static_cast<double>(col) - static_cast<double>(cols - 1) / 2.0) *
                        opts.fallback_bus_spacing_km;
    const double north = static_cast<double>(row) * opts.fallback_bus_spacing_km;
    auto [lat, lon] = offset_coordinate(opts.fallback_case_center_latitude,
                                        opts.fallback_case_center_longitude,
                                        east,
                                        north);
    geo[sys.ac.buses[i].index] = {lat, lon, true};
  }
  return geo;
}

std::map<int, BusGeo> build_dc_bus_geo(const HybridPowerSystem& sys,
                                       const TyphoonScenarioOptions& opts,
                                       bool* used_fallback) {
  std::map<int, BusGeo> geo;
  bool any_valid = false;
  for (const auto& bus : sys.dc.buses) {
    if (valid_coordinate(bus.latitude, bus.longitude)) {
      any_valid = true;
      geo[bus.index] = {bus.latitude, bus.longitude, false};
    }
  }
  const double dc_center_lat = opts.fallback_case_center_latitude + 0.05;
  const double dc_center_lon = opts.fallback_case_center_longitude + 0.05;
  if (any_valid) {
    for (std::size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const auto& bus = sys.dc.buses[i];
      if (geo.count(bus.index)) continue;
      const double angle = 2.0 * kPi * static_cast<double>(i) /
                           std::max<std::size_t>(1, sys.dc.buses.size());
      const double radius = opts.fallback_bus_spacing_km *
                            (1.0 + static_cast<double>(i) / 8.0);
      auto [lat, lon] = offset_coordinate(dc_center_lat,
                                          dc_center_lon,
                                          radius * std::cos(angle),
                                          radius * std::sin(angle));
      geo[bus.index] = {lat, lon, true};
      if (used_fallback) *used_fallback = true;
    }
    return geo;
  }

  if (used_fallback) *used_fallback = *used_fallback || !sys.dc.buses.empty();
  const int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(sys.dc.buses.size())))));
  for (std::size_t i = 0; i < sys.dc.buses.size(); ++i) {
    const int row = static_cast<int>(i) / cols;
    const int col = static_cast<int>(i) % cols;
    const double east = (static_cast<double>(col) - static_cast<double>(cols - 1) / 2.0) *
                        opts.fallback_bus_spacing_km;
    const double north = static_cast<double>(row) * opts.fallback_bus_spacing_km;
    auto [lat, lon] = offset_coordinate(dc_center_lat, dc_center_lon, east, north);
    geo[sys.dc.buses[i].index] = {lat, lon, true};
  }
  return geo;
}

TyphoonSegmentFragility cable_defaults() {
  TyphoonSegmentFragility f;
  f.kind = TyphoonAssetKind::Cable;
  f.design_wind_ms = 40.0;
  f.collapse_wind_ms = 70.0;
  f.lognormal_median_wind_ms = 60.0;
  f.lognormal_sigma = 0.35;
  f.design_rain_mm_hr = 50.0;
  f.seg_a_wind = 0.6;
  f.seg_b_rain = 2.5;
  f.seg_c_bias = -8.0;
  f.curve_number_cn = 85.0;
  f.manning_n = 0.035;
  f.slope_s0 = 0.005;
  f.soil_depth_z_m = 0.8;
  f.porosity_phi = 0.45;
  f.initial_moisture_theta = 0.30;
  f.base_cable_failure_p = 5e-4;
  return f;
}

TyphoonAssetKind choose_kind(double overhead_fraction, double cable_fraction, int segment_index, int n_segments) {
  const double total = std::max(0.0, overhead_fraction) + std::max(0.0, cable_fraction);
  if (total <= 1e-12) return TyphoonAssetKind::OverheadLine;
  const double cable_share = std::max(0.0, cable_fraction) / total;
  if (cable_share <= 1e-12) return TyphoonAssetKind::OverheadLine;
  if (cable_share >= 1.0 - 1e-12) return TyphoonAssetKind::Cable;
  const int cable_count = static_cast<int>(std::round(cable_share * n_segments));
  return segment_index >= n_segments - cable_count ? TyphoonAssetKind::Cable
                                                   : TyphoonAssetKind::OverheadLine;
}

double rmw_from_delta_p(double delta_p) {
  return std::exp(3.858 - 7.7e-5 * delta_p * delta_p);
}

double holland_b(double lat, double rmw_km) {
  return std::max(0.8, 1.881093 - 0.010917 * lat - 0.005567 * rmw_km);
}

double coriolis(double lat) {
  return 2.0 * kOmega * std::sin(deg2rad(lat));
}

double vmax_from_params(double delta_p_hpa, double b) {
  if (delta_p_hpa <= 0.0 || b <= 0.0) return 0.0;
  return std::sqrt(std::max(0.0, b * delta_p_hpa * 100.0 / kAirDensity)) * std::exp(-0.5);
}

void apply_stochastic_typhoon_perturbation(TyphoonScenarioOptions& opts, std::mt19937& rng) {
  std::normal_distribution<double> lat_jitter(0.0, 0.45);
  std::normal_distribution<double> lon_jitter(0.0, 0.45);
  std::normal_distribution<double> heading_jitter(0.0, 5.0);
  std::lognormal_distribution<double> delta_scale(-0.5 * 0.14 * 0.14, 0.14);
  std::lognormal_distribution<double> speed_scale(-0.5 * 0.18 * 0.18, 0.18);

  opts.initial_latitude = clamp_value(opts.initial_latitude + lat_jitter(rng), 12.0, 24.0);
  opts.initial_longitude = clamp_value(opts.initial_longitude + lon_jitter(rng), 109.5, 120.0);
  opts.initial_delta_p_hpa = clamp_value(opts.initial_delta_p_hpa * delta_scale(rng), 20.0, 75.0);
  opts.initial_heading_deg = clamp_value(opts.initial_heading_deg + heading_jitter(rng), -35.0, 35.0);
  opts.initial_translation_speed_kmph = clamp_value(opts.initial_translation_speed_kmph * speed_scale(rng), 8.0, 36.0);
}

TyphoonScenarioOptions apply_month_defaults(TyphoonScenarioOptions opts) {
  if (!opts.use_month_defaults) return opts;
  const int month = clamp_int(opts.month, 1, 12);
  // Deterministic seasonal defaults inspired by the standalone generator's
  // typhoon-season priors. They give users a one-field UI while keeping the
  // generated storm reproducible.
  const double season = clamp_value((static_cast<double>(month) - 6.0) / 5.0, 0.0, 1.0);
  opts.initial_latitude = 19.2 + 2.0 * season;
  opts.initial_longitude = 113.3 + 1.4 * std::sin((static_cast<double>(month) - 6.0) * kPi / 5.0);
  opts.initial_delta_p_hpa = 36.0 + 18.0 * std::sin(clamp_value((static_cast<double>(month) - 5.0) / 7.0, 0.0, 1.0) * kPi);
  opts.initial_heading_deg = -8.0 + 16.0 * season;
  opts.initial_translation_speed_kmph = 18.0 + 6.0 * season;
  return opts;
}

std::vector<TyphoonTrackPoint> build_track(const TyphoonScenarioOptions& raw_opts,
                                           double* monthly_sst_c = nullptr,
                                           std::string* sst_resource_path = nullptr) {
  auto opts = apply_month_defaults(raw_opts);
  std::mt19937 track_rng(opts.seed);
  if (opts.stochastic) {
    apply_stochastic_typhoon_perturbation(opts, track_rng);
  }
  const auto sst = load_sst_climatology(opts.sst_resource_path);
  const int month = clamp_int(opts.month, 1, 12);
  const double selected_sst = sst.monthly_mean_sst_c[static_cast<std::size_t>(month - 1)];
  const double annual_mean_sst = std::accumulate(sst.monthly_mean_sst_c.begin(),
                                                sst.monthly_mean_sst_c.end(), 0.0) /
                                 static_cast<double>(sst.monthly_mean_sst_c.size());
  const double sst_anomaly_c = selected_sst - annual_mean_sst;
  const double sst_intensity_scale = clamp_value(1.0 + 0.04 * sst_anomaly_c, 0.85, 1.20);
  const double sst_decay_scale = clamp_value(1.0 - 0.03 * sst_anomaly_c, 0.75, 1.25);
  if (monthly_sst_c) *monthly_sst_c = selected_sst;
  if (sst_resource_path) *sst_resource_path = sst.source_path.string();

  const int steps = std::max(1, static_cast<int>(std::ceil(opts.horizon_hours / std::max(opts.time_step_hr, 1e-6))));
  std::vector<TyphoonTrackPoint> track;
  track.reserve(steps);
  double lat = opts.initial_latitude;
  double lon = opts.initial_longitude;
  double delta_p = std::max(1.0, opts.initial_delta_p_hpa) * sst_intensity_scale;

  // A real typhoon track rarely follows a perfectly straight line.  Keep the
  // large-scale monthly steering flow from the original heading, but add a
  // deterministic low-frequency meander and weak northward recurvature so the
  // generated catalog and Web preview show a physically plausible wavy track.
  // The terms are seeded, so catalog samples remain reproducible.
  std::uniform_real_distribution<double> phase_dist(0.0, 2.0 * kPi);
  std::uniform_real_distribution<double> amp_dist(5.0, 12.0);
  std::uniform_real_distribution<double> speed_phase_dist(0.0, 2.0 * kPi);
  const double meander_phase = phase_dist(track_rng);
  const double speed_phase = speed_phase_dist(track_rng);
  const double meander_amp_deg = amp_dist(track_rng);
  const double meander_cycles = 1.15 + 0.35 * std::sin(static_cast<double>(opts.seed % 997U));
  const double recurvature_deg = clamp_value(4.0 + 0.45 * static_cast<double>(month), 5.0, 10.0);

  for (int i = 0; i < steps; ++i) {
    const double progress = steps > 1 ? static_cast<double>(i) / static_cast<double>(steps - 1) : 0.0;
    const double meander = meander_amp_deg * std::sin(2.0 * kPi * meander_cycles * progress + meander_phase);
    const double recurvature = recurvature_deg * std::sin(0.5 * kPi * progress) * std::sin(kPi * progress);
    const double current_heading_deg = opts.initial_heading_deg + meander + recurvature;
    const double current_speed_kmph = std::max(
        0.0,
        opts.initial_translation_speed_kmph *
            (1.0 + 0.08 * std::sin(2.0 * kPi * progress + speed_phase)));

    TyphoonTrackPoint p;
    p.hour = static_cast<double>(i) * opts.time_step_hr;
    p.latitude = lat;
    p.longitude = lon;
    p.delta_p_hpa = delta_p;
    p.rmw_km = opts.initial_rmw_km > 0.0 ? opts.initial_rmw_km : rmw_from_delta_p(delta_p);
    p.holland_b = holland_b(lat, p.rmw_km);
    p.fc = coriolis(lat);
    p.heading_deg = current_heading_deg;
    p.translation_speed_kmph = current_speed_kmph;
    p.vmax_ms = vmax_from_params(delta_p, p.holland_b);
    p.sst_c = selected_sst;
    track.push_back(p);

    const double distance_km = current_speed_kmph * opts.time_step_hr;
    const double heading = deg2rad(current_heading_deg);
    const double north_km = distance_km * std::cos(heading);
    const double east_km = -distance_km * std::sin(heading);
    lat += north_km / 111.0;
    const double cos_lat = std::max(0.2, std::cos(deg2rad(lat)));
    lon += east_km / (111.0 * cos_lat);
    if (lat >= 22.0) {
      const double decay = (0.006 + 0.00046 * delta_p * delta_p / std::max(1.0, p.rmw_km)) *
                           sst_decay_scale;
      delta_p *= std::exp(-decay * opts.time_step_hr);
    }
  }
  return track;
}

double holland_wind_ms_impl(const TyphoonTrackPoint& storm, double site_lat, double site_lon) {
  const double radius_km = std::max(1.0, haversine_km(storm.latitude, storm.longitude, site_lat, site_lon));
  const double rmw = std::max(1.0, storm.rmw_km);
  const double b = std::max(0.5, storm.holland_b);
  const double ratio = std::pow(rmw / radius_km, b);
  const double pressure = b * storm.delta_p_hpa * 100.0 * ratio * std::exp(-ratio) / kAirDensity;
  const double r_m = radius_km * 1000.0;
  const double tempc = storm.fc * r_m / 2.0;
  const double gradient = std::sqrt(std::max(0.0, pressure + tempc * tempc)) - tempc;
  const double v10 = kGustFactor * gradient * std::pow(kAnemometerHeightM / kGradientHeightM, kPowerLawExponent);
  return v10 < 0.01 ? 0.0 : std::max(0.0, v10);
}

double bearing_sector_deg(const TyphoonTrackPoint& storm, double lat, double lon) {
  const double y = std::sin(deg2rad(lon - storm.longitude)) * std::cos(deg2rad(lat));
  const double x = std::cos(deg2rad(storm.latitude)) * std::sin(deg2rad(lat)) -
                   std::sin(deg2rad(storm.latitude)) * std::cos(deg2rad(lat)) *
                       std::cos(deg2rad(lon - storm.longitude));
  double deg = rad2deg(std::atan2(y, x));
  if (deg < 0.0) deg += 360.0;
  deg += storm.heading_deg;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

double rain_sector_multiplier(int sector, double transspeed) {
  const bool fast = transspeed > 8.0;
  const bool slow = transspeed < 4.0;
  switch (sector) {
    case 0: return fast ? 1.15 : (slow ? 1.45 : 1.0);
    case 1: return fast ? 1.15 : (slow ? 1.05 : 1.0);
    case 2: return fast ? 1.35 : (slow ? 0.55 : 1.0);
    case 3: return fast ? 1.35 : (slow ? 0.65 : 1.0);
    case 4: return 0.85;
    case 5: return fast ? 0.65 : (slow ? 0.95 : 1.0);
    case 6: return fast ? 0.80 : (slow ? 1.15 : 1.0);
    case 7: return fast ? 0.95 : (slow ? 1.35 : 1.0);
    default: return 1.0;
  }
}

double typhoon_rain_mm_hr(const TyphoonTrackPoint& storm,
                          const TyphoonTrackPoint* previous,
                          double site_lat,
                          double site_lon) {
  const double radius = std::max(kRainRadiusFloorKm,
                                 haversine_km(storm.latitude, storm.longitude, site_lat, site_lon));
  const double tempratio = clamp_value(storm.rmw_km / radius, 0.0, kRainRatioMax);
  double rr = -5.5 + 110.0 * tempratio - 390.0 * std::pow(tempratio, 2) +
              550.0 * std::pow(tempratio, 3) - 250.0 * std::pow(tempratio, 4);
  rr = std::max(0.0, rr);
  const double dpdt = previous ? previous->delta_p_hpa - storm.delta_p_hpa : 1.0;
  const double k = std::max(1.0, 0.0319 * storm.delta_p_hpa - 0.0395);
  const double k1 = std::max(1.0, 1.0 - dpdt / 100.0);
  const int sector = static_cast<int>(std::floor(bearing_sector_deg(storm, site_lat, site_lon) / 45.0)) % 8;
  return std::max(0.0, k * k1 * rr * rain_sector_multiplier(sector, storm.translation_speed_kmph / 3.6));
}

double normal_cdf(double x) {
  return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

double lognormal_cdf(double value, double median, double sigma) {
  if (value <= 0.0 || median <= 0.0 || sigma <= 0.0) return 0.0;
  return clamp_value(normal_cdf((std::log(value) - std::log(median)) / sigma), 0.0, 1.0);
}

double equivalent_wind_ms(double wind_ms, double rain_mm_hr) {
  const double v10min = wind_ms / 1.42;
  const double f1 = std::exp(0.006462 * v10min) - 1.2486 * std::exp(-0.2769 * v10min);
  const double f2 = rain_mm_hr > 0.0 ? 0.09376 * std::pow(rain_mm_hr, 0.7087) : 0.0;
  return std::max(0.0, (v10min + std::max(0.0, f1 * f2)) * 1.42);
}

double overhead_segment_probability(const TyphoonLineSegment& seg,
                                    double wind_ms,
                                    double rain_mm_hr,
                                    double dt_hr) {
  const auto& f = seg.fragility;
  const double eq_wind = equivalent_wind_ms(wind_ms, rain_mm_hr);
  double p_wind = 0.0;
  if (eq_wind > f.design_wind_ms) {
    // The lognormal curve gives a fragility severity, not an independent full-line
    // outage probability for every synthetic segment. Scale it by segment exposure
    // length so auto-segmentation does not make long feeders fail deterministically.
    const double severity = lognormal_cdf(eq_wind, f.lognormal_median_wind_ms, f.lognormal_sigma);
    const double exposure_km = clamp_value(seg.length_km, 0.0, 5.0);
    p_wind = clamp_value(1.0 - std::exp(-0.015 * severity * exposure_km * std::max(0.0, dt_hr)), 0.0, 1.0);
  }
  const double wind_ratio = f.design_wind_ms > 0.0 ? wind_ms / f.design_wind_ms : 0.0;
  const double rain_ratio = f.design_rain_mm_hr > 0.0 ? rain_mm_hr / f.design_rain_mm_hr : 0.0;
  const double lambda = seg.length_km * seg.length_km *
                        std::exp(f.seg_a_wind * wind_ratio + f.seg_b_rain * rain_ratio + f.seg_c_bias);
  const double p_seg = wind_ms > f.design_wind_ms || rain_mm_hr > f.design_rain_mm_hr
                           ? clamp_value(1.0 - std::exp(-lambda * std::max(0.0, dt_hr)), 0.0, 1.0)
                           : 0.0;
  return clamp_value(1.0 - (1.0 - p_wind) * (1.0 - p_seg), 0.0, 1.0);
}

double cable_segment_probability(const TyphoonLineSegment& seg,
                                 double rain_mm_hr) {
  const auto& f = seg.fragility;
  const double cn = clamp_value(f.curve_number_cn, 1.0, 99.0);
  const double s = 25400.0 / cn - 254.0;
  const double ia = 0.2 * s;
  double runoff = 0.0;
  if (rain_mm_hr > ia) {
    const double excess = rain_mm_hr - ia;
    runoff = excess * excess / std::max(1e-9, excess + s);
  }
  const double infiltration = std::max(0.0, rain_mm_hr - runoff);
  const double q_rate = runoff / 1000.0 / 3600.0;
  const double h_t = std::pow(q_rate * f.manning_n / std::sqrt(std::max(1e-9, f.slope_s0)), 0.6);
  const double theta = std::min(f.porosity_phi,
                                f.initial_moisture_theta + infiltration / std::max(1e-9, f.soil_depth_z_m * 1000.0));
  const double p = f.base_cable_failure_p / 10.0 *
                   std::exp(130.0 * h_t + 8.0 * theta / std::max(1e-9, f.porosity_phi));
  return clamp_value(p, 0.0, 1.0);
}

double segment_probability(const TyphoonLineSegment& seg,
                           double wind_ms,
                           double rain_mm_hr,
                           double dt_hr) {
  if (seg.fragility.kind == TyphoonAssetKind::Cable) {
    return cable_segment_probability(seg, rain_mm_hr);
  }
  if (seg.fragility.kind == TyphoonAssetKind::Mixed) {
    const double p_over = overhead_segment_probability(seg, wind_ms, rain_mm_hr, dt_hr);
    const double p_cable = cable_segment_probability(seg, rain_mm_hr);
    return clamp_value(1.0 - (1.0 - p_over) * (1.0 - p_cable), 0.0, 1.0);
  }
  return overhead_segment_probability(seg, wind_ms, rain_mm_hr, dt_hr);
}

std::vector<double> smooth_profile(std::vector<double> values, int window) {
  if (values.empty() || window <= 1) return values;
  std::vector<double> out(values.size(), 1.0);
  const int half = window / 2;
  for (std::size_t i = 0; i < values.size(); ++i) {
    double sum = 0.0;
    int count = 0;
    for (int k = -half; k <= half; ++k) {
      const int idx = static_cast<int>(i) + k;
      if (idx < 0 || idx >= static_cast<int>(values.size())) continue;
      sum += values[static_cast<std::size_t>(idx)];
      ++count;
    }
    out[i] = count > 0 ? sum / static_cast<double>(count) : values[i];
  }
  return out;
}

constexpr std::array<TyphoonIntensityCategory, 6> kIntensityCategories{
    TyphoonIntensityCategory::TD,
    TyphoonIntensityCategory::TS,
    TyphoonIntensityCategory::STS,
    TyphoonIntensityCategory::TY,
    TyphoonIntensityCategory::STY,
    TyphoonIntensityCategory::SuperTY,
};

TyphoonCatalogOptions normalize_catalog_options(TyphoonCatalogOptions opts) {
  opts.samples_per_month = std::max(1, opts.samples_per_month);
  opts.first_month = clamp_int(opts.first_month, 1, 12);
  opts.last_month = clamp_int(opts.last_month, 1, 12);
  if (opts.first_month > opts.last_month) std::swap(opts.first_month, opts.last_month);
  opts.horizon_hours = std::max(1, opts.horizon_hours);
  opts.time_step_hr = std::max(1e-6, opts.time_step_hr);
  return opts;
}

std::string catalog_option_key(const TyphoonCatalogOptions& raw_opts) {
  const auto opts = normalize_catalog_options(raw_opts);
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os << "n=" << opts.samples_per_month
     << ";m=" << opts.first_month << '-' << opts.last_month
     << ";seed=" << opts.base_seed
     << ";h=" << opts.horizon_hours
     << ";dt=" << std::setprecision(6) << opts.time_step_hr
     << ";stoch=" << (opts.stochastic ? 1 : 0)
     << ";month_defaults=" << (opts.use_month_defaults ? 1 : 0)
     << ";sst=" << (opts.use_sst_resource ? 1 : 0)
     << ";sst_path=" << opts.sst_resource_path
     << ";catalog_path=" << opts.catalog_path;
  return os.str();
}

std::filesystem::path default_catalog_path(const TyphoonCatalogOptions& raw_opts) {
  namespace fs = std::filesystem;
  const auto opts = normalize_catalog_options(raw_opts);
  if (!opts.catalog_path.empty()) return fs::path(opts.catalog_path);
  std::ostringstream name;
  name.setf(std::ios::fixed);
  name << "typhoon_track_catalog_m" << opts.first_month << '-' << opts.last_month
       << "_n" << opts.samples_per_month
       << "_h" << opts.horizon_hours
       << "_dt" << std::setprecision(2) << opts.time_step_hr
       << "_seed" << opts.base_seed << ".json";
  return project_root_path() / "external_data" / "typhoon" / name.str();
}

nlohmann::json track_point_to_json(const TyphoonTrackPoint& p) {
  return nlohmann::json{{"hour", p.hour},
                        {"latitude", p.latitude},
                        {"longitude", p.longitude},
                        {"fc", p.fc},
                        {"delta_p_hpa", p.delta_p_hpa},
                        {"holland_b", p.holland_b},
                        {"rmw_km", p.rmw_km},
                        {"heading_deg", p.heading_deg},
                        {"translation_speed_kmph", p.translation_speed_kmph},
                        {"vmax_ms", p.vmax_ms},
                        {"sst_c", p.sst_c}};
}

TyphoonTrackPoint track_point_from_json(const nlohmann::json& j) {
  TyphoonTrackPoint p;
  p.hour = j.value("hour", 0.0);
  p.latitude = j.value("latitude", j.value("lat", p.latitude));
  p.longitude = j.value("longitude", j.value("lon", p.longitude));
  p.fc = j.value("fc", 0.0);
  p.delta_p_hpa = j.value("delta_p_hpa", p.delta_p_hpa);
  p.holland_b = j.value("holland_b", p.holland_b);
  p.rmw_km = j.value("rmw_km", p.rmw_km);
  p.heading_deg = j.value("heading_deg", p.heading_deg);
  p.translation_speed_kmph = j.value("translation_speed_kmph", p.translation_speed_kmph);
  p.vmax_ms = j.value("vmax_ms", 0.0);
  p.sst_c = j.value("sst_c", 0.0);
  return p;
}

nlohmann::json sample_to_json(const TyphoonTrackSample& sample) {
  nlohmann::json track = nlohmann::json::array();
  for (const auto& p : sample.track) track.push_back(track_point_to_json(p));
  return nlohmann::json{{"sample_id", sample.sample_id},
                        {"month", sample.month},
                        {"seed", sample.seed},
                        {"stochastic", sample.stochastic},
                        {"max_vmax_ms", sample.max_vmax_ms},
                        {"category", to_string(sample.category)},
                        {"monthly_sst_c", sample.monthly_sst_c},
                        {"sst_resource_path", sample.sst_resource_path},
                        {"track", std::move(track)}};
}

TyphoonTrackSample sample_from_json(const nlohmann::json& j) {
  TyphoonTrackSample sample;
  sample.sample_id = j.value("sample_id", std::string{});
  sample.month = j.value("month", 8);
  sample.seed = j.value("seed", 0U);
  sample.stochastic = j.value("stochastic", true);
  sample.max_vmax_ms = j.value("max_vmax_ms", 0.0);
  sample.category = typhoon_intensity_category_from_string(j.value("category", std::string{"Unknown"}));
  sample.monthly_sst_c = j.value("monthly_sst_c", 0.0);
  sample.sst_resource_path = j.value("sst_resource_path", std::string{});
  if (j.contains("track") && j["track"].is_array()) {
    sample.track.reserve(j["track"].size());
    for (const auto& item : j["track"]) sample.track.push_back(track_point_from_json(item));
  }
  if (!sample.track.empty()) {
    sample.max_vmax_ms = max_track_vmax_ms(sample.track, true);
    sample.category = classify_typhoon_intensity(sample.max_vmax_ms);
  }
  return sample;
}

nlohmann::json catalog_metadata_json(const TyphoonCatalogOptions& raw_opts) {
  const auto opts = normalize_catalog_options(raw_opts);
  return nlohmann::json{{"samples_per_month", opts.samples_per_month},
                        {"first_month", opts.first_month},
                        {"last_month", opts.last_month},
                        {"base_seed", opts.base_seed},
                        {"horizon_hours", opts.horizon_hours},
                        {"time_step_hr", opts.time_step_hr},
                        {"stochastic", opts.stochastic},
                        {"use_month_defaults", opts.use_month_defaults},
                        {"use_sst_resource", opts.use_sst_resource},
                        {"sst_resource_path", opts.sst_resource_path},
                        {"catalog_path", opts.catalog_path},
                        {"catalog_model_version", "intensity_balanced_wavy_v4"},
                        {"classification_skip_first_point", true}};
}

bool catalog_metadata_matches(const nlohmann::json& j, const TyphoonCatalogOptions& opts) {
  if (!j.contains("metadata") || !j["metadata"].is_object()) return false;
  return j["metadata"].dump() == catalog_metadata_json(opts).dump();
}

bool load_catalog_from_disk(const std::filesystem::path& path,
                            const TyphoonCatalogOptions& opts,
                            TyphoonCatalog* catalog) {
  std::ifstream in(path);
  if (!in) return false;
  nlohmann::json j;
  in >> j;
  if (!catalog_metadata_matches(j, opts)) return false;
  TyphoonCatalog loaded;
  loaded.source_path = path.string();
  loaded.loaded_from_disk = true;
  if (!j.contains("samples") || !j["samples"].is_array()) return false;
  loaded.samples.reserve(j["samples"].size());
  for (const auto& item : j["samples"]) {
    auto sample = sample_from_json(item);
    const auto idx = loaded.samples.size();
    loaded.samples.push_back(std::move(sample));
    const auto category = loaded.samples.back().category;
    if (category != TyphoonIntensityCategory::Unknown) {
      loaded.by_category[category].push_back(idx);
      loaded.category_counts[category] += 1;
    }
  }
  *catalog = std::move(loaded);
  return true;
}

void save_catalog_to_disk(const std::filesystem::path& path,
                          const TyphoonCatalogOptions& opts,
                          const TyphoonCatalog& catalog) {
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);
  nlohmann::json samples = nlohmann::json::array();
  for (const auto& sample : catalog.samples) samples.push_back(sample_to_json(sample));
  nlohmann::json counts;
  for (const auto category : kIntensityCategories) {
    auto it = catalog.category_counts.find(category);
    counts[to_string(category)] = it == catalog.category_counts.end() ? 0 : it->second;
  }
  nlohmann::json root{{"metadata", catalog_metadata_json(opts)},
                      {"category_counts", std::move(counts)},
                      {"samples", std::move(samples)}};
  std::ofstream out(path);
  if (out) out << root.dump(2) << '\n';
}

}  // namespace

double holland_wind_ms(const TyphoonTrackPoint& storm, double site_lat, double site_lon) {
  return holland_wind_ms_impl(storm, site_lat, site_lon);
}

TyphoonScenarioOptions::TyphoonScenarioOptions() {
  default_overhead_fragility = TyphoonSegmentFragility{};
  default_cable_fragility = cable_defaults();
}

const char* to_string(TyphoonAssetKind kind) {
  switch (kind) {
    case TyphoonAssetKind::OverheadLine: return "overhead";
    case TyphoonAssetKind::Cable: return "cable";
    case TyphoonAssetKind::Mixed: return "mixed";
  }
  return "unknown";
}

const char* to_string(TyphoonIntensityCategory category) {
  switch (category) {
    case TyphoonIntensityCategory::TD: return "TD";
    case TyphoonIntensityCategory::TS: return "TS";
    case TyphoonIntensityCategory::STS: return "STS";
    case TyphoonIntensityCategory::TY: return "TY";
    case TyphoonIntensityCategory::STY: return "STY";
    case TyphoonIntensityCategory::SuperTY: return "SuperTY";
    case TyphoonIntensityCategory::Unknown: return "Unknown";
  }
  return "Unknown";
}

const char* to_zh_name(TyphoonIntensityCategory category) {
  switch (category) {
    case TyphoonIntensityCategory::TD: return "热带低压";
    case TyphoonIntensityCategory::TS: return "热带风暴";
    case TyphoonIntensityCategory::STS: return "强热带风暴";
    case TyphoonIntensityCategory::TY: return "台风";
    case TyphoonIntensityCategory::STY: return "强台风";
    case TyphoonIntensityCategory::SuperTY: return "超强台风";
    case TyphoonIntensityCategory::Unknown: return "未知";
  }
  return "未知";
}

TyphoonIntensityCategory classify_typhoon_intensity(double max_vmax_ms) {
  if (!std::isfinite(max_vmax_ms) || max_vmax_ms < 10.8) return TyphoonIntensityCategory::Unknown;
  if (max_vmax_ms < 17.2) return TyphoonIntensityCategory::TD;
  if (max_vmax_ms < 24.5) return TyphoonIntensityCategory::TS;
  if (max_vmax_ms < 32.7) return TyphoonIntensityCategory::STS;
  if (max_vmax_ms < 41.5) return TyphoonIntensityCategory::TY;
  if (max_vmax_ms < 51.0) return TyphoonIntensityCategory::STY;
  return TyphoonIntensityCategory::SuperTY;
}

double max_track_vmax_ms(const std::vector<TyphoonTrackPoint>& track, bool skip_first_point) {
  if (track.empty()) return 0.0;
  const std::size_t start = skip_first_point && track.size() > 1 ? 1U : 0U;
  double vmax = 0.0;
  for (std::size_t i = start; i < track.size(); ++i) {
    if (std::isfinite(track[i].vmax_ms)) vmax = std::max(vmax, track[i].vmax_ms);
  }
  return vmax;
}

TyphoonIntensityCategory typhoon_intensity_category_from_string(const std::string& value) {
  std::string compact;
  compact.reserve(value.size());
  for (const unsigned char ch : value) {
    if (std::isalnum(ch)) compact.push_back(static_cast<char>(std::toupper(ch)));
  }
  if (compact == "TD") return TyphoonIntensityCategory::TD;
  if (compact == "TS") return TyphoonIntensityCategory::TS;
  if (compact == "STS") return TyphoonIntensityCategory::STS;
  if (compact == "TY") return TyphoonIntensityCategory::TY;
  if (compact == "STY") return TyphoonIntensityCategory::STY;
  if (compact == "SUPERTY" || compact == "SUPER") return TyphoonIntensityCategory::SuperTY;
  if (value == "热带低压") return TyphoonIntensityCategory::TD;
  if (value == "热带风暴") return TyphoonIntensityCategory::TS;
  if (value == "强热带风暴") return TyphoonIntensityCategory::STS;
  if (value == "台风") return TyphoonIntensityCategory::TY;
  if (value == "强台风") return TyphoonIntensityCategory::STY;
  if (value == "超强台风") return TyphoonIntensityCategory::SuperTY;
  return TyphoonIntensityCategory::Unknown;
}

TyphoonTrackSample generate_typhoon_track_sample(const TyphoonScenarioOptions& opts) {
  TyphoonTrackSample sample;
  sample.month = clamp_int(opts.month, 1, 12);
  sample.seed = opts.seed;
  sample.stochastic = opts.stochastic;
  sample.sst_resource_path = opts.sst_resource_path;
  sample.track = build_track(opts, &sample.monthly_sst_c, &sample.sst_resource_path);
  sample.max_vmax_ms = max_track_vmax_ms(sample.track, true);
  sample.category = classify_typhoon_intensity(sample.max_vmax_ms);
  std::ostringstream id;
  id << 'm' << std::setw(2) << std::setfill('0') << sample.month
     << "-s" << sample.seed << '-' << to_string(sample.category);
  sample.sample_id = id.str();
  return sample;
}

TyphoonCatalog build_typhoon_catalog(const TyphoonCatalogOptions& raw_opts) {
  const auto opts = normalize_catalog_options(raw_opts);
  TyphoonCatalog catalog;
  catalog.source_path = default_catalog_path(opts).string();
  const int months = opts.last_month - opts.first_month + 1;
  catalog.samples.reserve(static_cast<std::size_t>(months * opts.samples_per_month));
  for (int month = opts.first_month; month <= opts.last_month; ++month) {
    for (int i = 0; i < opts.samples_per_month; ++i) {
      TyphoonScenarioOptions scenario;
      scenario.month = month;
      scenario.horizon_hours = opts.horizon_hours;
      scenario.time_step_hr = opts.time_step_hr;
      scenario.seed = opts.base_seed + static_cast<unsigned int>(month * 10000 + i);
      scenario.stochastic = opts.stochastic;
      scenario.use_month_defaults = opts.use_month_defaults;
      scenario.use_sst_resource = opts.use_sst_resource;
      scenario.sst_resource_path = opts.sst_resource_path;
      if (opts.use_month_defaults) {
        scenario = apply_month_defaults(scenario);
        scenario.use_month_defaults = false;
      }
      if (opts.stochastic) {
        std::mt19937 intensity_rng(scenario.seed ^ 0x85ebca6bU);
        std::uniform_real_distribution<double> category_draw(0.0, 1.0);
        const double u = category_draw(intensity_rng);
        double target_vmax = 0.0;
        if (u < 0.12) {
          std::uniform_real_distribution<double> d(11.0, 16.8);
          target_vmax = d(intensity_rng);
        } else if (u < 0.27) {
          std::uniform_real_distribution<double> d(17.4, 24.0);
          target_vmax = d(intensity_rng);
        } else if (u < 0.47) {
          std::uniform_real_distribution<double> d(25.0, 32.2);
          target_vmax = d(intensity_rng);
        } else if (u < 0.72) {
          std::uniform_real_distribution<double> d(33.0, 41.0);
          target_vmax = d(intensity_rng);
        } else if (u < 0.90) {
          std::uniform_real_distribution<double> d(42.0, 50.5);
          target_vmax = d(intensity_rng);
        } else {
          std::uniform_real_distribution<double> d(51.2, 58.0);
          target_vmax = d(intensity_rng);
        }
        scenario.initial_delta_p_hpa = clamp_value(std::pow(target_vmax / 5.9, 2.0), 2.0, 90.0);
        scenario.stochastic = false;
      }
      auto sample = generate_typhoon_track_sample(scenario);
      const auto idx = catalog.samples.size();
      catalog.samples.push_back(std::move(sample));
      const auto category = catalog.samples.back().category;
      if (category != TyphoonIntensityCategory::Unknown) {
        catalog.by_category[category].push_back(idx);
        catalog.category_counts[category] += 1;
      }
    }
  }
  return catalog;
}

const TyphoonCatalog& get_or_build_typhoon_catalog(const TyphoonCatalogOptions& raw_opts) {
  static std::mutex mu;
  static std::string cached_key;
  static TyphoonCatalog cached;
  const auto opts = normalize_catalog_options(raw_opts);
  const auto key = catalog_option_key(opts);
  const auto path = default_catalog_path(opts);
  std::lock_guard<std::mutex> lk(mu);
  if (cached_key == key && !cached.samples.empty()) return cached;
  TyphoonCatalog loaded;
  if (load_catalog_from_disk(path, opts, &loaded)) {
    cached = std::move(loaded);
  } else {
    cached = build_typhoon_catalog(opts);
    cached.source_path = path.string();
    cached.loaded_from_disk = false;
    save_catalog_to_disk(path, opts, cached);
  }
  cached_key = key;
  return cached;
}

const TyphoonTrackSample* sample_typhoon_catalog(
    const TyphoonCatalog& catalog,
    TyphoonIntensityCategory category,
    unsigned int selection_seed) {
  auto it = catalog.by_category.find(category);
  if (it == catalog.by_category.end() || it->second.empty()) return nullptr;
  std::mt19937 rng(selection_seed);
  std::uniform_int_distribution<std::size_t> pick(0, it->second.size() - 1);
  return &catalog.samples[it->second[pick(rng)]];
}

std::vector<TyphoonLineSegment> generate_typhoon_line_segments(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts,
    bool* used_fallback_coordinates) {
  bool unused_fallback = false;
  const auto bus_geo = build_bus_geo(sys, opts, &unused_fallback);
  const auto dc_bus_geo = build_dc_bus_geo(sys, opts, &unused_fallback);
  if (used_fallback_coordinates) {
    *used_fallback_coordinates = unused_fallback;
  }
  std::vector<TyphoonLineSegment> segments;
  segments.reserve((sys.ac.branches.size() + sys.dc.branches.size()) * 2);

  for (std::size_t pos = 0; pos < sys.ac.branches.size(); ++pos) {
    const auto& br = sys.ac.branches[pos];
    if (!br.in_service) continue;
    auto it_from = bus_geo.find(br.from_bus);
    auto it_to = bus_geo.find(br.to_bus);
    if (it_from == bus_geo.end() || it_to == bus_geo.end()) continue;

    const int branch_index = br.index;
    double length_km = br.length_km;
    const double geo_length = haversine_km(it_from->second.lat, it_from->second.lon,
                                           it_to->second.lat, it_to->second.lon);
    if (length_km <= 1e-9 && geo_length > 1e-9) length_km = geo_length;
    if (length_km <= 1e-9) length_km = 0.8 + 0.15 * static_cast<double>(pos);

    const int min_segments = std::max(1, opts.min_segments_per_branch);
    const int max_segments = std::max(min_segments, opts.max_segments_per_branch);
    int nseg = static_cast<int>(std::ceil(length_km / std::max(0.05, opts.target_segment_length_km)));
    nseg = clamp_int(nseg, min_segments, max_segments);

    for (int s = 0; s < nseg; ++s) {
      const double a = static_cast<double>(s) / static_cast<double>(nseg);
      const double b = static_cast<double>(s + 1) / static_cast<double>(nseg);
      const double m = (a + b) / 2.0;
      TyphoonLineSegment seg;
      seg.branch_kind = ResilienceBranchKind::AC;
      seg.branch_index = branch_index;
      seg.segment_index = s;
      seg.from_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * a;
      seg.from_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * a;
      seg.to_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * b;
      seg.to_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * b;
      seg.mid_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * m;
      seg.mid_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * m;
      seg.length_km = length_km / static_cast<double>(nseg);
      const auto kind = choose_kind(opts.default_overhead_fraction, opts.default_cable_fraction, s, nseg);
      seg.fragility = kind == TyphoonAssetKind::Cable ? opts.default_cable_fragility
                                                      : opts.default_overhead_fragility;
      seg.fragility.kind = kind;
      segments.push_back(seg);
    }
  }

  for (std::size_t pos = 0; pos < sys.dc.branches.size(); ++pos) {
    const auto& br = sys.dc.branches[pos];
    if (!br.in_service) continue;
    auto it_from = dc_bus_geo.find(br.from_bus);
    auto it_to = dc_bus_geo.find(br.to_bus);
    if (it_from == dc_bus_geo.end() || it_to == dc_bus_geo.end()) continue;

    const int branch_index = br.index;
    double length_km = br.length_km;
    const double geo_length = haversine_km(it_from->second.lat, it_from->second.lon,
                                           it_to->second.lat, it_to->second.lon);
    if (length_km <= 1e-9 && geo_length > 1e-9) length_km = geo_length;
    if (length_km <= 1e-9) length_km = 0.8 + 0.15 * static_cast<double>(pos);

    const int min_segments = std::max(1, opts.min_segments_per_branch);
    const int max_segments = std::max(min_segments, opts.max_segments_per_branch);
    int nseg = static_cast<int>(std::ceil(length_km / std::max(0.05, opts.target_segment_length_km)));
    nseg = clamp_int(nseg, min_segments, max_segments);

    for (int s = 0; s < nseg; ++s) {
      const double a = static_cast<double>(s) / static_cast<double>(nseg);
      const double b = static_cast<double>(s + 1) / static_cast<double>(nseg);
      const double m = (a + b) / 2.0;
      TyphoonLineSegment seg;
      seg.branch_kind = ResilienceBranchKind::DC;
      seg.branch_index = branch_index;
      seg.segment_index = s;
      seg.from_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * a;
      seg.from_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * a;
      seg.to_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * b;
      seg.to_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * b;
      seg.mid_latitude = it_from->second.lat + (it_to->second.lat - it_from->second.lat) * m;
      seg.mid_longitude = it_from->second.lon + (it_to->second.lon - it_from->second.lon) * m;
      seg.length_km = length_km / static_cast<double>(nseg);
      const auto kind = choose_kind(opts.default_overhead_fraction, opts.default_cable_fraction, s, nseg);
      seg.fragility = kind == TyphoonAssetKind::Cable ? opts.default_cable_fragility
                                                      : opts.default_overhead_fragility;
      seg.fragility.kind = kind;
      segments.push_back(seg);
    }
  }
  return segments;
}

double typhoon_branch_length_km(const HybridPowerSystem& sys, ResilienceBranchKind kind, int branch_index) {
  if (kind == ResilienceBranchKind::DC) {
    for (std::size_t i = 0; i < sys.dc.branches.size(); ++i) {
      const auto& br = sys.dc.branches[i];
      if (br.index == branch_index) return std::max(0.0, br.length_km);
    }
    return 0.0;
  }
  for (std::size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    if (br.index == branch_index) return std::max(0.0, br.length_km);
  }
  return 0.0;
}

std::unordered_map<int, int> make_ac_bus_map_for_repair(const HybridPowerSystem& sys) {
  std::unordered_map<int, int> out;
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) out[sys.ac.buses[i].index] = static_cast<int>(i);
  return out;
}

std::unordered_map<int, int> make_dc_bus_map_for_repair(const HybridPowerSystem& sys, int ac_offset) {
  std::unordered_map<int, int> out;
  for (std::size_t i = 0; i < sys.dc.buses.size(); ++i) out[sys.dc.buses[i].index] = ac_offset + static_cast<int>(i);
  return out;
}

bool branch_outaged(const std::set<BranchKey>& outaged, ResilienceBranchKind kind, int branch_index) {
  return outaged.find(BranchKey{kind, branch_index}) != outaged.end();
}

double served_load_mw_for_repair_state(const HybridPowerSystem& sys,
                                       const std::set<BranchKey>& outaged) {
  const int ac_n = static_cast<int>(sys.ac.buses.size());
  const int dc_n = static_cast<int>(sys.dc.buses.size());
  const int n = ac_n + dc_n;
  if (n <= 0) return 0.0;
  const auto ac_bus = make_ac_bus_map_for_repair(sys);
  const auto dc_bus = make_dc_bus_map_for_repair(sys, ac_n);
  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
  auto add_edge = [&](int u, int v) {
    if (u < 0 || v < 0 || u >= n || v >= n) return;
    adj[static_cast<std::size_t>(u)].push_back(v);
    adj[static_cast<std::size_t>(v)].push_back(u);
  };

  for (std::size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    if (!br.in_service || branch_outaged(outaged, ResilienceBranchKind::AC, br.index)) continue;
    const auto f = ac_bus.find(br.from_bus);
    const auto t = ac_bus.find(br.to_bus);
    if (f != ac_bus.end() && t != ac_bus.end()) add_edge(f->second, t->second);
  }
  for (std::size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    if (!br.in_service || branch_outaged(outaged, ResilienceBranchKind::DC, br.index)) continue;
    const auto f = dc_bus.find(br.from_bus);
    const auto t = dc_bus.find(br.to_bus);
    if (f != dc_bus.end() && t != dc_bus.end()) add_edge(f->second, t->second);
  }
  for (const auto& t : sys.ac.transformers_2w) {
    if (!t.in_service) continue;
    const auto f = ac_bus.find(t.hv_bus);
    const auto l = ac_bus.find(t.lv_bus);
    if (f != ac_bus.end() && l != ac_bus.end()) add_edge(f->second, l->second);
  }
  for (const auto& c : sys.vsc_converters) {
    if (!c.in_service) continue;
    const auto a = ac_bus.find(c.bus_ac);
    const auto d = dc_bus.find(c.bus_dc);
    if (a != ac_bus.end() && d != dc_bus.end()) add_edge(a->second, d->second);
  }
  for (const auto& c : sys.dc.dcdc_converters) {
    if (!c.in_service) continue;
    const auto i = dc_bus.find(c.bus_in);
    const auto o = dc_bus.find(c.bus_out);
    if (i != dc_bus.end() && o != dc_bus.end()) add_edge(i->second, o->second);
  }

  std::vector<int> comp(static_cast<std::size_t>(n), -1);
  int cid = 0;
  std::vector<int> stack;
  for (int i = 0; i < n; ++i) {
    if (comp[static_cast<std::size_t>(i)] >= 0) continue;
    stack = {i};
    comp[static_cast<std::size_t>(i)] = cid;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      for (int v : adj[static_cast<std::size_t>(u)]) {
        if (comp[static_cast<std::size_t>(v)] >= 0) continue;
        comp[static_cast<std::size_t>(v)] = cid;
        stack.push_back(v);
      }
    }
    ++cid;
  }

  std::vector<bool> sourced(static_cast<std::size_t>(cid), false);
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (sys.ac.buses[i].in_service && sys.ac.buses[i].bus_type == BusType::SLACK) {
      sourced[static_cast<std::size_t>(comp[i])] = true;
    }
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = ac_bus.find(eg.bus);
    if (it != ac_bus.end()) sourced[static_cast<std::size_t>(comp[static_cast<std::size_t>(it->second)])] = true;
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service || !g.is_slack) continue;
    const auto it = ac_bus.find(g.bus);
    if (it != ac_bus.end()) sourced[static_cast<std::size_t>(comp[static_cast<std::size_t>(it->second)])] = true;
  }

  double served = 0.0;
  if (!sys.ac.loads.empty()) {
    for (const auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;
      const auto it = ac_bus.find(ld.bus);
      if (it == ac_bus.end()) continue;
      if (sourced[static_cast<std::size_t>(comp[static_cast<std::size_t>(it->second)])]) {
        served += std::max(0.0, ld.p_mw * (ld.scaling > 0.0 ? ld.scaling : 1.0));
      }
    }
  } else {
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& b = sys.ac.buses[i];
      if (!b.in_service || b.pd_mw <= 0.0) continue;
      if (sourced[static_cast<std::size_t>(comp[i])]) served += b.pd_mw;
    }
  }
  std::vector<bool> dc_explicit(sys.dc.buses.size(), false);
  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    const auto it = dc_bus.find(ld.bus);
    if (it == dc_bus.end()) continue;
    const int pos = it->second - ac_n;
    if (pos >= 0 && pos < static_cast<int>(dc_explicit.size())) dc_explicit[static_cast<std::size_t>(pos)] = true;
    if (sourced[static_cast<std::size_t>(comp[static_cast<std::size_t>(it->second)])]) {
      const double p = ld.p_mw > 0.0 ? ld.p_mw : ld.p_rated_mw;
      served += std::max(0.0, p * (ld.scaling > 0.0 ? ld.scaling : 1.0));
    }
  }
  for (std::size_t i = 0; i < sys.dc.buses.size(); ++i) {
    if (i < dc_explicit.size() && dc_explicit[i]) continue;
    const auto& b = sys.dc.buses[i];
    if (!b.in_service || b.pd_mw <= 0.0) continue;
    const int node = ac_n + static_cast<int>(i);
    if (sourced[static_cast<std::size_t>(comp[static_cast<std::size_t>(node)])]) served += b.pd_mw;
  }
  return served;
}

std::map<BranchKey, int> branch_source_depths(const HybridPowerSystem& sys) {
  const int ac_n = static_cast<int>(sys.ac.buses.size());
  const int dc_n = static_cast<int>(sys.dc.buses.size());
  const int n = ac_n + dc_n;
  std::map<BranchKey, int> depths;
  if (n <= 0) return depths;

  const auto ac_bus = make_ac_bus_map_for_repair(sys);
  const auto dc_bus = make_dc_bus_map_for_repair(sys, ac_n);
  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
  auto add_edge = [&](int u, int v) {
    if (u < 0 || v < 0 || u >= n || v >= n) return;
    adj[static_cast<std::size_t>(u)].push_back(v);
    adj[static_cast<std::size_t>(v)].push_back(u);
  };

  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    const auto f = ac_bus.find(br.from_bus);
    const auto t = ac_bus.find(br.to_bus);
    if (f != ac_bus.end() && t != ac_bus.end()) add_edge(f->second, t->second);
  }
  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    const auto f = dc_bus.find(br.from_bus);
    const auto t = dc_bus.find(br.to_bus);
    if (f != dc_bus.end() && t != dc_bus.end()) add_edge(f->second, t->second);
  }
  for (const auto& t : sys.ac.transformers_2w) {
    if (!t.in_service) continue;
    const auto f = ac_bus.find(t.hv_bus);
    const auto l = ac_bus.find(t.lv_bus);
    if (f != ac_bus.end() && l != ac_bus.end()) add_edge(f->second, l->second);
  }
  for (const auto& c : sys.vsc_converters) {
    if (!c.in_service) continue;
    const auto a = ac_bus.find(c.bus_ac);
    const auto d = dc_bus.find(c.bus_dc);
    if (a != ac_bus.end() && d != dc_bus.end()) add_edge(a->second, d->second);
  }
  for (const auto& c : sys.dc.dcdc_converters) {
    if (!c.in_service) continue;
    const auto i = dc_bus.find(c.bus_in);
    const auto o = dc_bus.find(c.bus_out);
    if (i != dc_bus.end() && o != dc_bus.end()) add_edge(i->second, o->second);
  }

  constexpr int kUnreachable = std::numeric_limits<int>::max() / 4;
  std::vector<int> distance(static_cast<std::size_t>(n), kUnreachable);
  std::queue<int> pending;
  auto add_source = [&](int node) {
    if (node < 0 || node >= n || distance[static_cast<std::size_t>(node)] == 0) return;
    distance[static_cast<std::size_t>(node)] = 0;
    pending.push(node);
  };
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (sys.ac.buses[i].in_service && sys.ac.buses[i].bus_type == BusType::SLACK) {
      add_source(static_cast<int>(i));
    }
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto it = ac_bus.find(eg.bus);
    if (it != ac_bus.end()) add_source(it->second);
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service || !g.is_slack) continue;
    const auto it = ac_bus.find(g.bus);
    if (it != ac_bus.end()) add_source(it->second);
  }
  while (!pending.empty()) {
    const int u = pending.front();
    pending.pop();
    for (const int v : adj[static_cast<std::size_t>(u)]) {
      if (distance[static_cast<std::size_t>(v)] <=
          distance[static_cast<std::size_t>(u)] + 1) {
        continue;
      }
      distance[static_cast<std::size_t>(v)] =
          distance[static_cast<std::size_t>(u)] + 1;
      pending.push(v);
    }
  }

  for (const auto& br : sys.ac.branches) {
    const auto f = ac_bus.find(br.from_bus);
    const auto t = ac_bus.find(br.to_bus);
    if (f == ac_bus.end() || t == ac_bus.end()) continue;
    depths[{ResilienceBranchKind::AC, br.index}] =
        std::min(distance[static_cast<std::size_t>(f->second)],
                 distance[static_cast<std::size_t>(t->second)]);
  }
  for (const auto& br : sys.dc.branches) {
    const auto f = dc_bus.find(br.from_bus);
    const auto t = dc_bus.find(br.to_bus);
    if (f == dc_bus.end() || t == dc_bus.end()) continue;
    depths[{ResilienceBranchKind::DC, br.index}] =
        std::min(distance[static_cast<std::size_t>(f->second)],
                 distance[static_cast<std::size_t>(t->second)]);
  }
  return depths;
}

void apply_staged_post_disaster_repairs(const HybridPowerSystem& sys,
                                        const TyphoonScenarioOptions& opts,
                                        const std::vector<BranchRisk>& risks,
                                        TyphoonFaultSequenceResult& result) {
  if (!opts.staged_post_disaster_repair || result.faults.empty()) return;
  std::unordered_map<std::string, const BranchRisk*> risk_by_key;
  for (const auto& r : risks) {
    risk_by_key[std::string(to_string(r.branch_kind)) + ":" + std::to_string(r.branch_index)] = &r;
  }
  std::vector<RepairCandidate> remaining;
  remaining.reserve(result.faults.size());
  std::set<BranchKey> outaged;
  double last_fault_hr = 0.0;
  for (const auto& f : result.faults) {
    last_fault_hr = std::max(last_fault_hr, f.outage_start_hr);
    const std::string key = std::string(to_string(f.branch_kind)) + ":" + std::to_string(f.branch_index);
    const auto it = risk_by_key.find(key);
    RepairCandidate c;
    c.branch_kind = f.branch_kind;
    c.branch_index = f.branch_index;
    c.fault_start_hr = f.outage_start_hr;
    if (it != risk_by_key.end()) {
      c.peak_probability = it->second->peak_probability;
      c.peak_wind_ms = it->second->peak_wind_ms;
    }
    remaining.push_back(c);
    outaged.insert({f.branch_kind, f.branch_index});
  }

  result.repair_fault_count = remaining.size();

  const double repair_ready_hr = last_fault_hr + std::max(0.0, opts.post_disaster_repair_delay_hr);
  const double crew_time_hr = std::max(opts.time_step_hr, opts.repair_crew_time_per_branch_hr);
  if (remaining.size() > kMaxExactStagedRepairFaults) {
    result.used_approximate_repair_order = true;
    const auto source_depth = branch_source_depths(sys);
    constexpr int kUnreachable = std::numeric_limits<int>::max() / 4;
    auto depth_of = [&](const RepairCandidate& candidate) {
      const auto it = source_depth.find(
          BranchKey{candidate.branch_kind, candidate.branch_index});
      return it == source_depth.end() ? kUnreachable : it->second;
    };
    std::stable_sort(remaining.begin(), remaining.end(),
                     [&](const RepairCandidate& a, const RepairCandidate& b) {
      const int ad = depth_of(a);
      const int bd = depth_of(b);
      if (ad != bd) return ad < bd;
      if (std::abs(a.fault_start_hr - b.fault_start_hr) > 1e-9) {
        return a.fault_start_hr < b.fault_start_hr;
      }
      if (std::abs(a.peak_probability - b.peak_probability) > 1e-12) {
        return a.peak_probability > b.peak_probability;
      }
      if (a.branch_kind != b.branch_kind) return a.branch_kind < b.branch_kind;
      return a.branch_index < b.branch_index;
    });
    for (std::size_t slot = 0; slot < remaining.size(); ++slot) {
      const auto& chosen = remaining[slot];
      const double completion_hr =
          repair_ready_hr + static_cast<double>(slot + 1) * crew_time_hr;
      for (auto& f : result.faults) {
        if (f.branch_kind == chosen.branch_kind &&
            f.branch_index == chosen.branch_index) {
          f.repair_duration_hr =
              std::max(opts.time_step_hr, completion_hr - f.outage_start_hr);
          break;
        }
      }
    }
    return;
  }

  for (std::size_t slot = 0; !remaining.empty(); ++slot) {
    const double base_served = served_load_mw_for_repair_state(sys, outaged);
    std::size_t best = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      auto trial = outaged;
      trial.erase(BranchKey{remaining[i].branch_kind, remaining[i].branch_index});
      const double impact = std::max(0.0, served_load_mw_for_repair_state(sys, trial) - base_served);
      remaining[i].impact_mw = impact;
      const double fallback = 1e-3 * remaining[i].peak_probability +
                              1e-5 * remaining[i].peak_wind_ms +
                              1e-6 * typhoon_branch_length_km(sys, remaining[i].branch_kind, remaining[i].branch_index);
      const double score = impact + fallback;
      if (score > best_score + 1e-12 ||
          (std::abs(score - best_score) <= 1e-12 &&
           remaining[i].fault_start_hr < remaining[best].fault_start_hr)) {
        best_score = score;
        best = i;
      }
    }

    const auto chosen = remaining[best];
    const double completion_hr = repair_ready_hr + static_cast<double>(slot + 1) * crew_time_hr;
    for (auto& f : result.faults) {
      if (f.branch_kind == chosen.branch_kind && f.branch_index == chosen.branch_index) {
        f.repair_duration_hr = std::max(opts.time_step_hr, completion_hr - f.outage_start_hr);
        break;
      }
    }
    outaged.erase(BranchKey{chosen.branch_kind, chosen.branch_index});
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
  }
}

TyphoonFaultSequenceResult generate_typhoon_fault_sequence(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts) {
  bool used_fallback_coordinates = false;
  const auto segments = generate_typhoon_line_segments(
      sys, opts, &used_fallback_coordinates);
  return generate_typhoon_fault_sequence(
      sys, opts, segments, used_fallback_coordinates);
}

TyphoonFaultSequenceResult generate_typhoon_fault_sequence(
    const HybridPowerSystem& sys,
    const TyphoonScenarioOptions& opts,
    const std::vector<TyphoonLineSegment>& precomputed_segments,
    bool used_fallback_coordinates) {
  TyphoonFaultSequenceResult result;
  result.used_fallback_coordinates = used_fallback_coordinates;
  result.used_synthetic_segments = opts.auto_segment_lines;
  result.seed = opts.seed;
  result.requested_category = opts.requested_category;
  result.used_category_fallback = opts.used_category_fallback;
  if (opts.use_precomputed_track && !opts.precomputed_track.empty()) {
    result.track = opts.precomputed_track;
    result.used_catalog_sample = true;
    result.selected_sample_id = opts.selected_sample_id;
    result.selected_track_max_vmax_ms = max_track_vmax_ms(result.track, true);
    result.selected_category = classify_typhoon_intensity(result.selected_track_max_vmax_ms);
    result.monthly_sst_c = result.track.front().sst_c;
    result.sst_resource_path = opts.sst_resource_path;
  } else {
    result.track = build_track(opts, &result.monthly_sst_c, &result.sst_resource_path);
    result.selected_track_max_vmax_ms = max_track_vmax_ms(result.track, true);
    result.selected_category = classify_typhoon_intensity(result.selected_track_max_vmax_ms);
  }
  result.generated_segments = precomputed_segments;

  if (sys.ac.branches.empty() && sys.dc.branches.empty()) {
    result.status = "No AC/DC branches available for typhoon fault generation";
    return result;
  }
  if (result.generated_segments.empty()) {
    result.status = "No typhoon line segments generated";
    return result;
  }

  std::map<BranchKey, std::vector<const TyphoonLineSegment*>> by_branch;
  for (const auto& seg : result.generated_segments) {
    by_branch[{seg.branch_kind, seg.branch_index}].push_back(&seg);
  }

  std::unordered_map<int, const ACBranch*> branch_map;
  for (std::size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& br = sys.ac.branches[i];
    branch_map[br.index] = &br;
  }
  std::unordered_map<int, const DCBranch*> dc_branch_map;
  for (std::size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& br = sys.dc.branches[i];
    dc_branch_map[br.index] = &br;
  }

  std::vector<BranchRisk> risks;
  risks.reserve(by_branch.size());
  std::vector<double> max_system_wind(result.track.size(), 0.0);
  std::mt19937 fault_rng(opts.seed ^ 0x9e3779b9U);
  std::uniform_real_distribution<double> unit_draw(0.0, 1.0);

  for (const auto& [branch_key, segs] : by_branch) {
    BranchRisk risk;
    risk.branch_kind = branch_key.kind;
    risk.branch_index = branch_key.index;
    for (std::size_t t = 0; t < result.track.size(); ++t) {
      double survival = 1.0;
      double hour_peak_wind = 0.0;
      double hour_peak_rain = 0.0;
      for (const auto* seg : segs) {
        const auto& storm = result.track[t];
        const TyphoonTrackPoint* prev = t > 0 ? &result.track[t - 1] : nullptr;
        const double wind = holland_wind_ms_impl(storm, seg->mid_latitude, seg->mid_longitude);
        const double rain = typhoon_rain_mm_hr(storm, prev, seg->mid_latitude, seg->mid_longitude);
        const double p = segment_probability(*seg, wind, rain, opts.time_step_hr);
        survival *= (1.0 - clamp_value(p, 0.0, 1.0));
        hour_peak_wind = std::max(hour_peak_wind, wind);
        hour_peak_rain = std::max(hour_peak_rain, rain);
      }
      const double p_branch = clamp_value(1.0 - survival, 0.0, 1.0);
      risk.peak_wind_ms = std::max(risk.peak_wind_ms, hour_peak_wind);
      risk.peak_rain_mm_hr = std::max(risk.peak_rain_mm_hr, hour_peak_rain);
      risk.peak_probability = std::max(risk.peak_probability, p_branch);
      max_system_wind[t] = std::max(max_system_wind[t], hour_peak_wind);
      if (!risk.faulted && unit_draw(fault_rng) < p_branch) {
        risk.faulted = true;
        risk.first_fault_hr = result.track[t].hour;
      }
    }
    risks.push_back(risk);
  }

  std::sort(risks.begin(), risks.end(), [](const auto& a, const auto& b) {
    if (a.faulted != b.faulted) return a.faulted > b.faulted;
    if (std::abs(a.first_fault_hr - b.first_fault_hr) > 1e-9) return a.first_fault_hr < b.first_fault_hr;
    if (std::abs(a.peak_probability - b.peak_probability) > 1e-12) return a.peak_probability > b.peak_probability;
    if (a.branch_kind != b.branch_kind) return a.branch_kind < b.branch_kind;
    return a.branch_index < b.branch_index;
  });

  for (const auto& risk : risks) {
    if (risk.branch_kind == ResilienceBranchKind::AC) {
      result.branch_indices.push_back(risk.branch_index);
      result.branch_peak_wind_ms.push_back(risk.peak_wind_ms);
      result.branch_peak_rain_mm_hr.push_back(risk.peak_rain_mm_hr);
      result.branch_peak_failure_probability.push_back(risk.peak_probability);
    }
    result.branch_risks.push_back({risk.branch_kind,
                                   risk.branch_index,
                                   risk.peak_wind_ms,
                                   risk.peak_rain_mm_hr,
                                   risk.peak_probability});
    if (!risk.faulted) continue;
    double repair = opts.default_repair_time_hr;
    if (risk.branch_kind == ResilienceBranchKind::DC) {
      auto it = dc_branch_map.find(risk.branch_index);
      if (it != dc_branch_map.end() && it->second->mttr_hours > 0.0) repair = it->second->mttr_hours;
    } else {
      auto it = branch_map.find(risk.branch_index);
      if (it != branch_map.end() && it->second->mttr_hr > 0.0) repair = it->second->mttr_hr;
    }
    repair *= 1.0 + opts.repair_time_wind_factor * std::max(0.0, risk.peak_wind_ms - 25.0) / 25.0;
    repair = clamp_value(repair, std::max(1e-6, opts.time_step_hr), opts.max_repair_time_hr);
    std::ostringstream name;
    name.setf(std::ios::fixed);
    name.precision(2);
    name << "Typhoon fault " << to_string(risk.branch_kind) << "-BR-" << risk.branch_index
         << " Vmax=" << risk.peak_wind_ms << "m/s Pmax=" << risk.peak_probability;
    DistributionResilienceFault fault;
    fault.ac_branch_index = risk.branch_kind == ResilienceBranchKind::AC ? risk.branch_index : 0;
    fault.branch_kind = risk.branch_kind;
    fault.branch_index = risk.branch_index;
    fault.outage_start_hr = risk.first_fault_hr;
    fault.repair_duration_hr = repair;
    fault.name = name.str();
    result.faults.push_back(fault);
  }

  apply_staged_post_disaster_repairs(sys, opts, risks, result);

  if (opts.apply_pv_wind_derating) {
    result.renewable_profile_multiplier.reserve(max_system_wind.size());
    for (const double wind : max_system_wind) {
      const double factor = clamp_value((wind - opts.pv_cutout_start_ms) /
                                            std::max(1e-9, opts.pv_cutout_full_ms - opts.pv_cutout_start_ms),
                                        0.0, 1.0);
      result.renewable_profile_multiplier.push_back(clamp_value(1.0 - opts.pv_max_reduction * factor, 0.0, 1.0));
    }
    result.renewable_profile_multiplier = smooth_profile(result.renewable_profile_multiplier,
                                                         std::max(1, opts.pv_transition_hours));
  }

  std::ostringstream status;
  status << "Generated " << result.faults.size() << " typhoon-driven branch faults from "
         << result.generated_segments.size() << " synthetic line segments";
  result.status = status.str();
  return result;
}

}  // namespace hacdcpf::analysis

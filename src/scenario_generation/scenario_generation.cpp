#include "hacdcpf/analysis/scenario_generation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace hacdcpf::analysis {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusKm = 6371.0;
constexpr double kGStcWm2 = 1000.0;
constexpr double kTStcC = 25.0;
constexpr double kPvTempCoeff = -0.004;
constexpr double kPvLossFactor = 0.85;
constexpr double kLoadTempBeta = 0.03;
constexpr std::array<int, 12> kMonthHours{744, 672, 744, 720, 744, 720, 744, 744, 720, 744, 720, 744};
constexpr std::uint64_t kMaxComponentPointsPerCandidate = 250000;
constexpr std::uint64_t kMaxComponentCandidateWorkPoints = 4000000;
constexpr std::uint64_t kMaxComponentRepresentativePoints = 500000;
constexpr std::uint64_t kMaxCoverageSamples = 5000;
constexpr std::uint64_t kMaxReliabilityCandidateCount = 12000;
constexpr std::uint64_t kMaxReliabilityRepresentativeCount = 5000;
constexpr std::uint64_t kMaxResilienceSpatialEvaluations = 8000000;
namespace fs = std::filesystem;

std::uint64_t saturating_product(std::initializer_list<std::uint64_t> values) {
  std::uint64_t result = 1;
  for (const auto value : values) {
    if (value == 0) return 0;
    if (result > std::numeric_limits<std::uint64_t>::max() / value) {
      return std::numeric_limits<std::uint64_t>::max();
    }
    result *= value;
  }
  return result;
}

struct ComponentProfilePlan {
  bool per_component{true};
  std::uint64_t points_per_candidate{0};
  std::uint64_t candidate_work_points{0};
  std::uint64_t representative_points{0};

  const char* granularity() const {
    return per_component ? "per_load_site" : "aggregate_system";
  }
};

ComponentProfilePlan component_profile_plan(std::size_t load_site_count,
                                            int steps,
                                            std::uint64_t total_candidates,
                                            std::uint64_t expected_representatives) {
  ComponentProfilePlan plan;
  const auto sites = static_cast<std::uint64_t>(load_site_count);
  const auto horizon = static_cast<std::uint64_t>(std::max(1, steps));
  plan.points_per_candidate = saturating_product({sites, horizon});
  plan.candidate_work_points =
      saturating_product({plan.points_per_candidate, total_candidates});
  plan.representative_points =
      saturating_product({plan.points_per_candidate, expected_representatives});
  plan.per_component =
      plan.points_per_candidate <= kMaxComponentPointsPerCandidate &&
      plan.candidate_work_points <= kMaxComponentCandidateWorkPoints &&
      plan.representative_points <= kMaxComponentRepresentativePoints;
  return plan;
}

void warn_component_profile_fallback(const char* family,
                                     const ComponentProfilePlan& plan,
                                     std::vector<std::string>* warnings) {
  if (plan.per_component || warnings == nullptr) return;
  warnings->push_back(
      std::string(family) +
      " scenario generation switched to aggregate load profiles because the "
      "projected per-component workload/output exceeds the bounded interactive budget.");
}

std::string lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::string component_id(const char* prefix, int index) {
  return std::string(prefix) + ":" + std::to_string(index);
}

std::string display_name(const std::string& name, const char* fallback, int index) {
  return name.empty() ? (std::string(fallback) + " " + std::to_string(index)) : name;
}

double clamp_value(double value, double lo, double hi) {
  if (hi < lo) std::swap(lo, hi);
  return std::clamp(value, lo, hi);
}

int typhoon_category_ordinal(TyphoonIntensityCategory category) {
  switch (category) {
    case TyphoonIntensityCategory::TD: return 0;
    case TyphoonIntensityCategory::TS: return 1;
    case TyphoonIntensityCategory::STS: return 2;
    case TyphoonIntensityCategory::TY: return 3;
    case TyphoonIntensityCategory::STY: return 4;
    case TyphoonIntensityCategory::SuperTY: return 5;
    case TyphoonIntensityCategory::Unknown: return 0;
  }
  return 0;
}

std::vector<TyphoonIntensityCategory> typhoon_category_order() {
  return {TyphoonIntensityCategory::TD,
          TyphoonIntensityCategory::TS,
          TyphoonIntensityCategory::STS,
          TyphoonIntensityCategory::TY,
          TyphoonIntensityCategory::STY,
          TyphoonIntensityCategory::SuperTY};
}

nlohmann::json typhoon_catalog_counts_to_json(const TyphoonCatalog& catalog) {
  nlohmann::json counts = nlohmann::json::object();
  for (const auto category : typhoon_category_order()) {
    const auto it = catalog.category_counts.find(category);
    counts[to_string(category)] = it == catalog.category_counts.end() ? 0 : it->second;
  }
  return counts;
}

nlohmann::json reliability_fmea_component_types_json() {
  return nlohmann::json::array({to_string(ContingencyComponentType::ACGenerator),
                                to_string(ContingencyComponentType::ACBranch),
                                to_string(ContingencyComponentType::DCBranch),
                                to_string(ContingencyComponentType::VSCConverter),
                                to_string(ContingencyComponentType::ACStaticGenerator),
                                to_string(ContingencyComponentType::ACRenewable),
                                to_string(ContingencyComponentType::ACStorage),
                                to_string(ContingencyComponentType::Transformer2W),
                                to_string(ContingencyComponentType::Transformer3W)});
}

bool contains_any(const std::string& text, std::initializer_list<const char*> needles) {
  for (const char* needle : needles) {
    if (text.find(needle) != std::string::npos) return true;
  }
  return false;
}

bool is_renewable_static_gen(const StaticGenerator& gen) {
  if (gen.sgen_type == SgenType::PV || gen.sgen_type == SgenType::Wind) return true;
  const auto name = lower_copy(gen.name);
  return contains_any(name, {"pv", "solar", "wind", "renew", "hydro"});
}

bool is_renewable_dc_static_gen(const StaticGeneratorDC& gen) {
  const auto type = lower_copy(gen.type);
  const auto name = lower_copy(gen.name);
  return contains_any(type, {"pv", "solar", "wind", "renew", "hydro"}) ||
         contains_any(name, {"pv", "solar", "wind", "renew", "hydro"});
}

bool is_pv_static_gen(const StaticGenerator& gen) {
  if (gen.sgen_type == SgenType::PV) return true;
  const auto name = lower_copy(gen.name);
  return contains_any(name, {"pv", "solar"});
}

bool is_wind_static_gen(const StaticGenerator& gen) {
  if (gen.sgen_type == SgenType::Wind) return true;
  const auto name = lower_copy(gen.name);
  return name.find("wind") != std::string::npos;
}

bool is_pv_dc_static_gen(const StaticGeneratorDC& gen) {
  const auto type = lower_copy(gen.type);
  const auto name = lower_copy(gen.name);
  return contains_any(type, {"pv", "solar"}) || contains_any(name, {"pv", "solar"});
}

bool is_wind_dc_static_gen(const StaticGeneratorDC& gen) {
  const auto type = lower_copy(gen.type);
  const auto name = lower_copy(gen.name);
  return type.find("wind") != std::string::npos || name.find("wind") != std::string::npos;
}

struct LoadSiteBaseline {
  std::string id;
  std::string name;
  bool is_dc{false};
  bool is_bus_fallback{false};
  int component_index{0};
  int bus{0};
  double base_mw{0.0};
  double latitude{0.0};
  double longitude{0.0};
  bool has_coordinates{false};
};

struct RenewableBreakdown {
  double pv_mw{0.0};
  double wind_mw{0.0};
  double other_mw{0.0};

  double total_mw() const { return pv_mw + wind_mw + other_mw; }
};

struct StorageSocDeviceBaseline {
  double capacity_mwh{0.0};
  double base_soc{0.0};
  double soc_min{0.0};
  double soc_max{1.0};
};

struct StorageSocBaseline {
  std::vector<StorageSocDeviceBaseline> devices;
  double total_capacity_mwh{0.0};

  bool empty() const { return devices.empty() || total_capacity_mwh <= 1e-9; }
  int count() const { return static_cast<int>(devices.size()); }
};

struct StorageSocProfiles {
  bool has_storage{false};
  int storage_count{0};
  double storage_capacity_mwh{0.0};
  double aggregate_soc{0.0};
  std::vector<double> storage_soc;
  std::vector<double> storage_energy_mwh;
};

struct WindResourceSite {
  double capacity_mw{0.0};
  double latitude{0.0};
  double longitude{0.0};
  bool has_coordinates{false};
};

double positive_capacity(double preferred, double fallback) {
  return std::max(0.0, preferred > 1e-9 ? preferred : fallback);
}

std::optional<std::pair<double, double>> ac_bus_coordinates(const HybridPowerSystem& sys, int bus_index) {
  for (const auto& bus : sys.ac.buses) {
    if (bus.index == bus_index && (std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9)) return std::make_pair(bus.latitude, bus.longitude);
  }
  return std::nullopt;
}

std::optional<std::pair<double, double>> dc_bus_coordinates(const HybridPowerSystem& sys, int bus_index) {
  for (const auto& bus : sys.dc.buses) {
    if (bus.index == bus_index && (std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9)) return std::make_pair(bus.latitude, bus.longitude);
  }
  return std::nullopt;
}

std::vector<LoadSiteBaseline> collect_load_sites(const HybridPowerSystem& sys) {
  std::vector<LoadSiteBaseline> sites;
  auto add_site = [&](LoadSiteBaseline site, std::optional<std::pair<double, double>> coords) {
    site.base_mw = std::max(0.0, site.base_mw);
    if (site.base_mw <= 1e-9) return;
    if (coords) {
      site.latitude = coords->first;
      site.longitude = coords->second;
      site.has_coordinates = true;
    }
    sites.push_back(std::move(site));
  };
  if (!sys.ac.loads.empty()) {
    for (std::size_t i = 0; i < sys.ac.loads.size(); ++i) {
      const auto& load = sys.ac.loads[i];
      if (!load.in_service) continue;
      LoadSiteBaseline site;
      site.id = component_id("ac_load", load.index);
      site.name = display_name(load.name, "AC Load", load.index);
      site.component_index = load.index;
      site.bus = load.bus;
      site.base_mw = load.p_mw * load.scaling;
      add_site(std::move(site), ac_bus_coordinates(sys, load.bus));
    }
  } else {
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const auto& bus = sys.ac.buses[i];
      if (!bus.in_service) continue;
      LoadSiteBaseline site;
      site.id = component_id("ac_bus_load", bus.index);
      site.name = display_name(bus.name, "AC Bus Load", bus.index);
      site.is_bus_fallback = true;
      site.component_index = bus.index;
      site.bus = bus.index;
      site.base_mw = bus.pd_mw;
      const bool has_coords = std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9;
      add_site(std::move(site), has_coords ? std::make_optional(std::make_pair(bus.latitude, bus.longitude)) : std::nullopt);
    }
  }
  if (!sys.dc.loads.empty()) {
    for (std::size_t i = 0; i < sys.dc.loads.size(); ++i) {
      const auto& load = sys.dc.loads[i];
      if (!load.in_service) continue;
      LoadSiteBaseline site;
      site.id = component_id("dc_load", load.index);
      site.name = display_name(load.name, "DC Load", load.index);
      site.is_dc = true;
      site.component_index = load.index;
      site.bus = load.bus;
      site.base_mw = load.p_mw * load.scaling;
      add_site(std::move(site), dc_bus_coordinates(sys, load.bus));
    }
  } else {
    for (std::size_t i = 0; i < sys.dc.buses.size(); ++i) {
      const auto& bus = sys.dc.buses[i];
      if (!bus.in_service) continue;
      LoadSiteBaseline site;
      site.id = component_id("dc_bus_load", bus.index);
      site.name = display_name(bus.name, "DC Bus Load", bus.index);
      site.is_dc = true;
      site.is_bus_fallback = true;
      site.component_index = bus.index;
      site.bus = bus.index;
      site.base_mw = bus.pd_mw;
      const bool has_coords = std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9;
      add_site(std::move(site), has_coords ? std::make_optional(std::make_pair(bus.latitude, bus.longitude)) : std::nullopt);
    }
  }
  return sites;
}

double sum_load_sites(const std::vector<LoadSiteBaseline>& sites) {
  return std::accumulate(sites.begin(), sites.end(), 0.0, [](double sum, const auto& site) { return sum + site.base_mw; });
}

double base_load_mw(const HybridPowerSystem& sys) {
  return sum_load_sites(collect_load_sites(sys));
}

RenewableBreakdown renewable_breakdown_mw(const HybridPowerSystem& sys) {
  RenewableBreakdown out;
  for (const auto& gen : sys.ac.renewable_gens) {
    if (!gen.in_service) continue;
    const double p = positive_capacity(gen.p_rated_mw, gen.p_mw);
    if (gen.type == RenewableType::SolarPV || gen.type == RenewableType::SolarCSP) out.pv_mw += p;
    else if (gen.type == RenewableType::Wind) out.wind_mw += p;
    else out.other_mw += p;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (pv.in_service) out.pv_mw += positive_capacity(pv.pmax_mw, pv.p_mw);
  }
  for (const auto& gen : sys.ac.static_generators) {
    if (!gen.in_service || !is_renewable_static_gen(gen)) continue;
    const double p = positive_capacity(gen.p_rated_mw, gen.pmax_mw > 1e-9 ? gen.pmax_mw : gen.p_mw) * gen.scaling;
    if (is_pv_static_gen(gen)) out.pv_mw += p;
    else if (is_wind_static_gen(gen)) out.wind_mw += p;
    else out.other_mw += p;
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (pv.in_service) out.pv_mw += std::max(0.0, pv.p_set_mw);
  }
  for (const auto& gen : sys.dc.dc_static_generators) {
    if (!gen.in_service || !is_renewable_dc_static_gen(gen)) continue;
    const double p = std::max(0.0, gen.p_set_mw * gen.scaling);
    if (is_pv_dc_static_gen(gen)) out.pv_mw += p;
    else if (is_wind_dc_static_gen(gen)) out.wind_mw += p;
    else out.other_mw += p;
  }
  return out;
}

double base_renewable_mw(const HybridPowerSystem& sys) {
  return renewable_breakdown_mw(sys).total_mw();
}

StorageSocBaseline storage_soc_baseline(const HybridPowerSystem& sys) {
  StorageSocBaseline baseline;
  auto add_storage = [&](const Storage& st) {
    if (!st.in_service || st.e_rated_mwh <= 1e-9) return;
    const double lo = clamp_value(std::min(st.soc_min, st.soc_max), 0.0, 1.0);
    const double hi = clamp_value(std::max(st.soc_min, st.soc_max), 0.0, 1.0);
    StorageSocDeviceBaseline dev;
    dev.capacity_mwh = st.e_rated_mwh;
    dev.base_soc = clamp_value(st.soc_init, lo, hi);
    dev.soc_min = lo;
    dev.soc_max = hi;
    baseline.total_capacity_mwh += dev.capacity_mwh;
    baseline.devices.push_back(dev);
  };
  for (const auto& st : sys.ac.storage) add_storage(st);
  for (const auto& st : sys.dc.storage) add_storage(st);
  return baseline;
}

double truncated_normal_multiplier(double sigma, double lo, double hi, std::mt19937& rng) {
  lo = std::max(0.0, lo);
  hi = std::max(0.0, hi);
  if (hi < lo) std::swap(lo, hi);
  if (sigma <= 1e-12) return clamp_value(1.0, lo, hi);
  std::normal_distribution<double> dist(1.0, sigma);
  double draw = 1.0;
  for (int attempt = 0; attempt < 128; ++attempt) {
    draw = dist(rng);
    if (draw >= lo && draw <= hi) return draw;
  }
  return clamp_value(draw, lo, hi);
}

StorageSocProfiles make_storage_soc_profiles(const StorageSocBaseline& baseline,
                                              int num_steps,
                                              const PerturbationOptions& opt,
                                              std::mt19937& rng) {
  StorageSocProfiles profiles;
  const int steps = std::max(1, num_steps);
  if (baseline.empty()) return profiles;
  profiles.has_storage = true;
  profiles.storage_count = baseline.count();
  profiles.storage_capacity_mwh = baseline.total_capacity_mwh;
  profiles.storage_soc.assign(static_cast<std::size_t>(steps), 0.0);
  profiles.storage_energy_mwh.assign(static_cast<std::size_t>(steps), 0.0);
  double actual_energy = 0.0;
  for (const auto& dev : baseline.devices) {
    const double multiplier = opt.enable_storage_soc_perturbation
        ? truncated_normal_multiplier(opt.storage_soc_sigma, opt.storage_soc_min_multiplier, opt.storage_soc_max_multiplier, rng)
        : 1.0;
    const double soc = clamp_value(dev.base_soc * multiplier, dev.soc_min, dev.soc_max);
    actual_energy += soc * dev.capacity_mwh;
  }
  profiles.aggregate_soc = actual_energy / baseline.total_capacity_mwh;
  std::fill(profiles.storage_energy_mwh.begin(), profiles.storage_energy_mwh.end(), actual_energy);
  std::fill(profiles.storage_soc.begin(), profiles.storage_soc.end(), profiles.aggregate_soc);
  return profiles;
}

std::vector<WindResourceSite> wind_resource_sites(const HybridPowerSystem& sys) {
  std::vector<WindResourceSite> sites;
  auto add_site = [&](double capacity, std::optional<std::pair<double, double>> coords) {
    if (capacity <= 1e-9) return;
    WindResourceSite site;
    site.capacity_mw = capacity;
    if (coords) {
      site.latitude = coords->first;
      site.longitude = coords->second;
      site.has_coordinates = true;
    }
    sites.push_back(site);
  };
  for (const auto& gen : sys.ac.renewable_gens) {
    if (gen.in_service && gen.type == RenewableType::Wind) add_site(positive_capacity(gen.p_rated_mw, gen.p_mw), ac_bus_coordinates(sys, gen.bus));
  }
  for (const auto& gen : sys.ac.static_generators) {
    if (gen.in_service && is_wind_static_gen(gen)) {
      add_site(positive_capacity(gen.p_rated_mw, gen.pmax_mw > 1e-9 ? gen.pmax_mw : gen.p_mw) * gen.scaling, ac_bus_coordinates(sys, gen.bus));
    }
  }
  for (const auto& gen : sys.dc.dc_static_generators) {
    if (gen.in_service && is_wind_dc_static_gen(gen)) add_site(std::max(0.0, gen.p_set_mw * gen.scaling), dc_bus_coordinates(sys, gen.bus));
  }
  return sites;
}

std::vector<double> synthesize_base_load(double base, int steps) {
  std::vector<double> values(std::max(1, steps), base);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int hour = t % 24;
    const int day = t / 24;
    const double daily = 0.84 + 0.18 * std::sin((static_cast<double>(hour) - 7.0) / 24.0 * 2.0 * kPi) +
                         0.10 * std::sin((static_cast<double>(hour) - 18.0) / 24.0 * 4.0 * kPi);
    const double season = steps >= 8760 ? 1.0 + 0.10 * std::cos((static_cast<double>(day) - 213.0) / 365.0 * 2.0 * kPi) : 1.0;
    values[static_cast<std::size_t>(t)] = base * std::max(0.2, daily) * season;
  }
  return values;
}

std::vector<std::vector<double>> synthesize_base_load_profiles(const std::vector<LoadSiteBaseline>& sites, int steps) {
  std::vector<std::vector<double>> profiles;
  profiles.reserve(sites.size());
  for (const auto& site : sites) profiles.push_back(synthesize_base_load(site.base_mw, steps));
  return profiles;
}

std::vector<double> sum_profiles(const std::vector<std::vector<double>>& component_profiles, int steps = 0) {
  int n = std::max(1, steps);
  for (const auto& profile : component_profiles) n = std::max(n, static_cast<int>(profile.size()));
  std::vector<double> total(static_cast<std::size_t>(n), 0.0);
  for (const auto& profile : component_profiles) {
    for (int t = 0; t < n; ++t) {
      if (t < static_cast<int>(profile.size())) total[static_cast<std::size_t>(t)] += profile[static_cast<std::size_t>(t)];
    }
  }
  return total;
}

std::vector<double> synthesize_base_pv(double base, int steps) {
  std::vector<double> values(std::max(1, steps), 0.0);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int hour = t % 24;
    const int day = t / 24;
    const double sun = std::max(0.0, std::sin((static_cast<double>(hour) - 6.0) / 12.0 * kPi));
    const double season = steps >= 8760 ? 0.90 + 0.18 * std::sin((static_cast<double>(day) - 80.0) / 365.0 * 2.0 * kPi) : 1.0;
    values[static_cast<std::size_t>(t)] = base * sun * std::max(0.15, season);
  }
  return values;
}

std::vector<double> synthesize_base_wind(double base, int steps) {
  std::vector<double> values(std::max(1, steps), 0.0);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int hour = t % 24;
    const int day = t / 24;
    const double diurnal = 0.95 + 0.05 * std::sin((static_cast<double>(hour) - 2.0) / 24.0 * 2.0 * kPi);
    const double synoptic = 1.0 + 0.12 * std::sin((static_cast<double>(day) + 20.0) / 9.0 * 2.0 * kPi);
    const double season = steps >= 8760 ? 0.82 + 0.18 * std::cos((static_cast<double>(day) - 25.0) / 365.0 * 2.0 * kPi) : 1.0;
    const double calm_weather = steps >= 8760
        ? clamp_value(0.55
                          + 0.35 * std::sin((static_cast<double>(day) + 3.0) / 6.5 * 2.0 * kPi)
                          + 0.25 * std::sin((static_cast<double>(day) + 11.0) / 21.0 * 2.0 * kPi),
                      0.0, 1.25)
        : 1.0;
    const double availability = diurnal * synoptic * season * calm_weather;
    values[static_cast<std::size_t>(t)] = availability < 0.08 ? 0.0 : base * clamp_value(availability, 0.0, 1.20);
  }
  return values;
}

std::vector<double> synthesize_base_other_renewable(double base, int steps) {
  std::vector<double> values(std::max(1, steps), 0.0);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int day = t / 24;
    const double season = steps >= 8760 ? 0.95 + 0.05 * std::sin((static_cast<double>(day) - 10.0) / 365.0 * 2.0 * kPi) : 1.0;
    values[static_cast<std::size_t>(t)] = base * std::max(0.0, season);
  }
  return values;
}

std::vector<double> synthesize_base_renewable(double base, int steps) {
  return synthesize_base_pv(base, steps);
}

std::vector<double> smooth_noise_factors(int steps, double sigma, double min_value, double max_value,
                                         int block_hours, std::mt19937& rng) {
  std::vector<double> factors(std::max(1, steps), 1.0);
  if (sigma <= 0.0) return factors;
  std::normal_distribution<double> normal(0.0, sigma);
  const int block = std::max(1, block_hours);
  double previous = normal(rng);
  double current = normal(rng);
  for (int t = 0; t < static_cast<int>(factors.size()); ++t) {
    if (t % block == 0) {
      previous = current;
      current = normal(rng);
    }
    const double alpha = static_cast<double>(t % block) / static_cast<double>(block);
    const double noise = previous * (1.0 - alpha) + current * alpha;
    factors[static_cast<std::size_t>(t)] = clamp_value(1.0 + noise, min_value, max_value);
  }
  return factors;
}

struct ClimateMorphFactors {
  std::array<double, 12> delta_t_c{};
  std::array<double, 12> delta_ghi_w_m2{};
  bool data_found{false};
  bool from_csv{false};
  std::string source;
};

std::string trim_copy(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char ch = line[i];
    if (ch == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (ch == ',' && !quoted) {
      fields.push_back(trim_copy(field));
      field.clear();
    } else {
      field.push_back(ch);
    }
  }
  fields.push_back(trim_copy(field));
  return fields;
}

std::optional<ClimateMorphFactors> parse_climate_grid_csv(const fs::path& path) {
  std::ifstream input(path);
  if (!input) return std::nullopt;
  std::unordered_map<std::string, std::array<double, 12>> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto row = split_csv_line(line);
    if (row.empty() || row[0].rfind("#", 0) == 0 || row[0] == "Variable" || row.size() < 26) continue;
    const std::string abbr = row[1];
    if (abbr.empty()) continue;
    std::array<double, 12> values{};
    bool ok = true;
    for (int month = 0; month < 12; ++month) {
      try {
        values[static_cast<std::size_t>(month)] = std::stod(row[static_cast<std::size_t>(3 + 2 * month)]);
      } catch (const std::exception&) {
        ok = false;
        break;
      }
    }
    if (ok) rows[abbr] = values;
  }
  const auto tas_it = rows.find("tas");
  const auto rsds_it = rows.find("rsds");
  if (tas_it == rows.end() || rsds_it == rows.end()) return std::nullopt;
  ClimateMorphFactors factors;
  factors.delta_t_c = tas_it->second;
  factors.delta_ghi_w_m2 = rsds_it->second;
  factors.data_found = true;
  factors.from_csv = true;
  factors.source = path.string();
  return factors;
}

std::vector<fs::path> climate_data_dir_candidates() {
  std::vector<fs::path> candidates;
#ifdef HACDCPF_PROJECT_ROOT
  candidates.push_back(fs::path(HACDCPF_PROJECT_ROOT) / "SenarioGeneration" / "offline_scenario_copilot" / "backend" / "data" / "climate_scenarios");
#endif
  const auto cwd = fs::current_path();
  candidates.push_back(cwd / "SenarioGeneration" / "offline_scenario_copilot" / "backend" / "data" / "climate_scenarios");
  candidates.push_back(cwd / "HybridACDCDistribtutionSystemsSimulation" / "SenarioGeneration" / "offline_scenario_copilot" / "backend" / "data" / "climate_scenarios");
  candidates.push_back(cwd.parent_path() / "SenarioGeneration" / "offline_scenario_copilot" / "backend" / "data" / "climate_scenarios");
  return candidates;
}

std::optional<ClimateMorphFactors> fallback_climate_morph_factors(const std::string& ssp, int year) {
  ClimateMorphFactors factors;
  factors.data_found = true;
  factors.from_csv = false;
  factors.source = "embedded_fallback:" + ssp + ":" + std::to_string(year);
  const auto set = [&](std::array<double, 12> dt, std::array<double, 12> dg) {
    factors.delta_t_c = dt;
    factors.delta_ghi_w_m2 = dg;
    return factors;
  };
  if (ssp == "ssp126" && year == 2050) return set({2.25, 2.95, 2.21, 2.07, 1.48, 1.38, 1.19, 1.13, 1.47, 1.85, 2.14, 2.30}, {15.77, 21.61, 17.78, 22.94, 13.52, 6.83, 9.97, 7.65, 14.06, 15.37, 15.92, 15.03});
  if (ssp == "ssp126" && year == 2080) return set({2.34, 3.22, 2.15, 2.59, 2.13, 1.74, 1.19, 1.39, 1.57, 2.37, 2.31, 1.96}, {16.45, 26.66, 20.55, 30.96, 22.78, 16.11, 12.12, 18.11, 12.80, 21.84, 21.79, 16.32});
  if (ssp == "ssp245" && year == 2050) return set({2.22, 2.60, 2.12, 1.82, 1.82, 1.59, 1.18, 1.22, 1.36, 1.85, 2.47, 2.43}, {2.38, 11.16, 11.35, 12.69, 11.63, 3.57, 2.35, 0.42, 1.10, 11.40, 14.58, 4.31});
  if (ssp == "ssp245" && year == 2080) return set({3.31, 3.56, 3.11, 2.98, 2.50, 2.31, 1.88, 2.09, 2.29, 3.08, 3.28, 2.97}, {2.19, 18.13, 20.54, 19.01, 12.68, 10.00, 7.86, 11.69, 12.43, 15.89, 20.91, 14.53});
  if (ssp == "ssp370" && year == 2050) return set({1.88, 2.36, 2.28, 2.11, 2.18, 1.96, 1.48, 1.55, 1.34, 1.24, 1.95, 1.43}, {-4.70, 1.28, -4.55, -1.11, -3.31, -2.37, -8.69, -4.99, -15.92, -2.42, -3.74, -4.59});
  if (ssp == "ssp370" && year == 2080) return set({3.70, 2.68, 3.10, 3.53, 3.36, 3.21, 2.80, 3.10, 2.79, 2.91, 3.70, 2.75}, {-5.02, 0.09, 2.62, 7.27, 0.29, -7.48, -7.43, -3.89, -16.04, -0.16, 3.76, -5.33});
  if (ssp == "ssp585" && year == 2050) return set({2.90, 3.88, 2.87, 2.28, 2.00, 2.05, 1.63, 1.60, 1.95, 2.09, 3.57, 2.65}, {5.37, 8.75, 13.32, 9.43, 6.55, 4.68, 0.24, 0.36, -1.86, 10.58, 11.27, 9.58});
  if (ssp == "ssp585" && year == 2080) return set({4.20, 5.41, 4.21, 4.58, 4.22, 3.69, 3.35, 3.63, 3.70, 4.58, 5.29, 4.50}, {11.31, 28.52, 16.37, 21.73, 16.82, -5.30, -0.28, 4.46, 2.63, 18.07, 18.14, 7.64});
  return std::nullopt;
}

ClimateMorphFactors load_climate_morph_factors(const std::string& ssp, int year, std::vector<std::string>* warnings) {
  const std::string filename = "CHN_-_GUANGZHOU_EC_Earth3_" + ssp + "_" + std::to_string(year) + "_GridPointVariables.csv";
  for (const auto& dir : climate_data_dir_candidates()) {
    const auto path = dir / filename;
    if (auto parsed = parse_climate_grid_csv(path)) return *parsed;
  }
  if (auto fallback = fallback_climate_morph_factors(ssp, year)) {
    if (warnings) warnings->push_back("Climate CSV for " + ssp + ":" + std::to_string(year) + " not found; using embedded monthly climate deltas.");
    return *fallback;
  }
  ClimateMorphFactors zeros;
  zeros.source = "zero_delta_missing:" + ssp + ":" + std::to_string(year);
  if (warnings) warnings->push_back("No climate morphing data found for " + ssp + ":" + std::to_string(year) + "; regular scenarios use zero climate deltas for this group.");
  return zeros;
}

int month_index_for_hour(int hour, int steps) {
  const int safe_steps = std::max(1, steps);
  const int annual_hour = std::clamp(static_cast<int>((static_cast<long long>(std::max(0, hour)) * 8760LL) / safe_steps), 0, 8759);
  int start = 0;
  for (int month = 0; month < 12; ++month) {
    start += kMonthHours[static_cast<std::size_t>(month)];
    if (annual_hour < start) return month;
  }
  return 11;
}

std::vector<double> synthesize_base_temperature_c(int steps) {
  std::vector<double> values(static_cast<std::size_t>(std::max(1, steps)), 0.0);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int hour = t % 24;
    const int day = t / 24;
    const double season = 23.5 + 7.0 * std::cos((static_cast<double>(day) - 205.0) / 365.0 * 2.0 * kPi);
    const double diurnal = 2.8 * std::sin((static_cast<double>(hour) - 8.0) / 24.0 * 2.0 * kPi);
    values[static_cast<std::size_t>(t)] = season + diurnal;
  }
  return values;
}

std::vector<double> synthesize_base_ghi_w_m2(int steps) {
  std::vector<double> values(static_cast<std::size_t>(std::max(1, steps)), 0.0);
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const int hour = t % 24;
    const int day = t / 24;
    const double sun = std::max(0.0, std::sin((static_cast<double>(hour) - 6.0) / 12.0 * kPi));
    const double season = steps >= 8760 ? 0.90 + 0.18 * std::sin((static_cast<double>(day) - 80.0) / 365.0 * 2.0 * kPi) : 1.0;
    values[static_cast<std::size_t>(t)] = 850.0 * sun * std::max(0.15, season);
  }
  return values;
}

std::array<double, 12> smooth_monthly_noise(double sigma, std::mt19937& rng) {
  std::array<double, 12> raw{};
  std::array<double, 12> smoothed{};
  if (sigma <= 0.0) return smoothed;
  std::normal_distribution<double> normal(0.0, sigma);
  for (double& value : raw) value = normal(rng);
  for (int month = 0; month < 12; ++month) {
    const int prev = (month + 11) % 12;
    const int next = (month + 1) % 12;
    smoothed[static_cast<std::size_t>(month)] = 0.25 * raw[static_cast<std::size_t>(prev)] + 0.5 * raw[static_cast<std::size_t>(month)] + 0.25 * raw[static_cast<std::size_t>(next)];
  }
  return smoothed;
}

std::vector<double> smooth_block_percent_factors(int steps, double sigma_pct, int block_hours, std::mt19937& rng) {
  std::vector<double> factors(static_cast<std::size_t>(std::max(1, steps)), 1.0);
  if (sigma_pct <= 0.0) return factors;
  const int block = std::max(1, block_hours);
  const int block_count = std::max(1, (static_cast<int>(factors.size()) + block - 1) / block);
  std::vector<double> raw(static_cast<std::size_t>(block_count), 0.0);
  std::normal_distribution<double> normal(0.0, sigma_pct / 100.0);
  for (double& value : raw) value = normal(rng);
  std::vector<double> smoothed(raw.size(), 0.0);
  for (int b = 0; b < block_count; ++b) {
    const int prev = std::max(0, b - 1);
    const int next = std::min(block_count - 1, b + 1);
    smoothed[static_cast<std::size_t>(b)] = 0.25 * raw[static_cast<std::size_t>(prev)] + 0.5 * raw[static_cast<std::size_t>(b)] + 0.25 * raw[static_cast<std::size_t>(next)];
  }
  for (int t = 0; t < static_cast<int>(factors.size()); ++t) {
    factors[static_cast<std::size_t>(t)] = std::max(0.0, 1.0 + smoothed[static_cast<std::size_t>(t / block)]);
  }
  return factors;
}

unsigned int stable_label_seed(const std::string& text) {
  unsigned int hash = 2166136261U;
  for (const unsigned char ch : text) {
    hash ^= ch;
    hash *= 16777619U;
  }
  return hash;
}

double pv_reference_pu(double air_temp_c, double ghi_w_m2) {
  const double ghi = std::max(0.0, ghi_w_m2);
  const double cell_temp = air_temp_c + 0.025 * ghi;
  const double efficiency = std::max(0.0, 1.0 + kPvTempCoeff * (cell_temp - kTStcC));
  return clamp_value((ghi / kGStcWm2) * efficiency * kPvLossFactor, 0.0, 1.0);
}

std::vector<double> morph_regular_load_profile(
    const std::vector<double>& base_load,
    const std::array<double, 12>& delta_t_c,
    const std::vector<double>& load_hourly_factor) {
  std::vector<double> load = base_load;
  load.resize(static_cast<std::size_t>(std::max(1, static_cast<int>(load.size()))), 0.0);
  const auto value_or = [](const std::vector<double>& values, int index, double fallback) {
    return index >= 0 && index < static_cast<int>(values.size()) ? values[static_cast<std::size_t>(index)] : fallback;
  };
  for (int t = 0; t < static_cast<int>(load.size()); ++t) {
    const int month = month_index_for_hour(t, static_cast<int>(load.size()));
    const double load_factor = 1.0 + kLoadTempBeta * delta_t_c[static_cast<std::size_t>(month)];
    load[static_cast<std::size_t>(t)] = std::max(0.0, load[static_cast<std::size_t>(t)] * load_factor * value_or(load_hourly_factor, t, 1.0));
  }
  return load;
}

std::vector<std::vector<double>> morph_regular_load_components(
    const std::vector<std::vector<double>>& base_load_components,
    const std::array<double, 12>& delta_t_c,
    const std::vector<std::vector<double>>& load_hourly_factors) {
  std::vector<std::vector<double>> load_components;
  load_components.reserve(base_load_components.size());
  for (std::size_t i = 0; i < base_load_components.size(); ++i) {
    const auto& factors = i < load_hourly_factors.size() ? load_hourly_factors[i] : std::vector<double>{};
    load_components.push_back(morph_regular_load_profile(base_load_components[i], delta_t_c, factors));
  }
  return load_components;
}

std::vector<double> morph_regular_pv(
    const std::vector<double>& base_pv,
    const std::vector<double>& base_temp_c,
    const std::vector<double>& base_ghi_w_m2,
    const std::array<double, 12>& delta_t_c,
    const std::array<double, 12>& delta_ghi_w_m2,
    const std::vector<double>& ghi_hourly_factor) {
  std::vector<double> pv = base_pv;
  pv.resize(static_cast<std::size_t>(std::max(1, static_cast<int>(pv.size()))), 0.0);
  const auto value_or = [](const std::vector<double>& values, int index, double fallback) {
    return index >= 0 && index < static_cast<int>(values.size()) ? values[static_cast<std::size_t>(index)] : fallback;
  };
  for (int t = 0; t < static_cast<int>(pv.size()); ++t) {
    const int month = month_index_for_hour(t, static_cast<int>(pv.size()));
    const double base_temp = value_or(base_temp_c, t, 25.0);
    const double base_ghi = value_or(base_ghi_w_m2, t, 0.0);
    const double future_temp = base_temp + delta_t_c[static_cast<std::size_t>(month)];
    const double future_ghi = std::max(0.0, base_ghi + delta_ghi_w_m2[static_cast<std::size_t>(month)]) * value_or(ghi_hourly_factor, t, 1.0);
    const double base_pu = pv_reference_pu(base_temp, base_ghi);
    const double future_pu = pv_reference_pu(future_temp, future_ghi);
    const double scale = base_pu > 1e-5 ? clamp_value(future_pu / base_pu, 0.0, 2.0) : 0.0;
    pv[static_cast<std::size_t>(t)] = std::max(0.0, pv[static_cast<std::size_t>(t)] * scale);
  }
  return pv;
}

std::pair<std::vector<double>, std::vector<double>> morph_regular_load_pv(
    const std::vector<double>& base_load,
    const std::vector<double>& base_pv,
    const std::vector<double>& base_temp_c,
    const std::vector<double>& base_ghi_w_m2,
    const std::array<double, 12>& delta_t_c,
    const std::array<double, 12>& delta_ghi_w_m2,
    const std::vector<double>& load_hourly_factor,
    const std::vector<double>& ghi_hourly_factor) {
  return {morph_regular_load_profile(base_load, delta_t_c, load_hourly_factor),
          morph_regular_pv(base_pv, base_temp_c, base_ghi_w_m2, delta_t_c, delta_ghi_w_m2, ghi_hourly_factor)};
}

double average_monthly(const std::array<double, 12>& values) {
  return std::accumulate(values.begin(), values.end(), 0.0) / 12.0;
}

std::vector<double> pv_transition_profile(int steps, std::optional<int> stage1_start, std::optional<int> stage1_end,
                                          int transition_hours) {
  std::vector<double> profile(std::max(1, steps), 1.0);
  if (!stage1_start || !stage1_end) return profile;
  const int start = std::clamp(*stage1_start, 1, steps);
  const int end = std::clamp(*stage1_end, start, steps);
  for (int h = start; h <= end; ++h) profile[static_cast<std::size_t>(h - 1)] = 0.0;
  if (transition_hours <= 0) return profile;
  for (int h = 1; h < start; ++h) {
    const int distance = start - h;
    if (distance < transition_hours) {
      profile[static_cast<std::size_t>(h - 1)] = 0.5 * (1.0 - std::cos(kPi * distance / transition_hours));
    }
  }
  for (int h = end + 1; h <= steps; ++h) {
    const int distance = h - end;
    if (distance < transition_hours) {
      profile[static_cast<std::size_t>(h - 1)] = 0.5 * (1.0 - std::cos(kPi * distance / transition_hours));
    }
  }
  return profile;
}

double smoothstep(double value) {
  const double clipped = std::clamp(value, 0.0, 1.0);
  return clipped * clipped * (3.0 - 2.0 * clipped);
}

double load_reduction(double wind_speed, const TyphoonImpactOptions& opt) {
  const double denom = std::max(1e-9, opt.load_wind_full - opt.load_wind_start);
  return opt.load_max_reduction * smoothstep((wind_speed - opt.load_wind_start) / denom);
}

double wind_power_curve(double wind_speed, const TyphoonImpactOptions& opt) {
  if (wind_speed <= opt.wind_cut_in_ms || wind_speed >= opt.wind_cut_out_ms) return 0.0;
  if (wind_speed >= opt.wind_rated_ms) return 1.0;
  const double denom = std::max(1e-9, opt.wind_rated_ms - opt.wind_cut_in_ms);
  const double x = clamp_value((wind_speed - opt.wind_cut_in_ms) / denom, 0.0, 1.0);
  return std::pow(x, std::max(0.1, opt.wind_ramp_exponent));
}

double approximate_site_wind(const TyphoonTrackPoint& p, double lat, double lon) {
  const double lat_rad = lat * kPi / 180.0;
  const double dx = kEarthRadiusKm * (lon - p.longitude) * kPi / 180.0 * std::cos(lat_rad);
  const double dy = kEarthRadiusKm * (lat - p.latitude) * kPi / 180.0;
  const double radius = std::sqrt(dx * dx + dy * dy);
  const double scale = std::max(25.0, p.rmw_km * 2.2);
  return std::max(0.0, p.vmax_ms * std::exp(-radius / scale));
}

std::vector<double> aggregate_bus_wind(const HybridPowerSystem& sys, const std::vector<TyphoonTrackPoint>& track, int steps) {
  std::vector<double> wind(std::max(1, steps), 0.0);
  if (track.empty()) return wind;
  std::vector<std::pair<double, double>> coords;
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service && (std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9)) coords.push_back({bus.latitude, bus.longitude});
  }
  for (const auto& bus : sys.dc.buses) {
    if (bus.in_service && (std::abs(bus.latitude) > 1e-9 || std::abs(bus.longitude) > 1e-9)) coords.push_back({bus.latitude, bus.longitude});
  }
  if (coords.empty()) {
    for (int t = 0; t < static_cast<int>(wind.size()); ++t) wind[static_cast<std::size_t>(t)] = track[std::min<std::size_t>(t, track.size() - 1)].vmax_ms * 0.55;
    return wind;
  }
  for (int t = 0; t < static_cast<int>(wind.size()); ++t) {
    const auto& p = track[std::min<std::size_t>(t, track.size() - 1)];
    double sum = 0.0;
    for (const auto& [lat, lon] : coords) sum += holland_wind_ms(p, lat, lon);
    wind[static_cast<std::size_t>(t)] = sum / static_cast<double>(coords.size());
  }
  return wind;
}

std::vector<double> site_wind_profile(const LoadSiteBaseline& site, const std::vector<TyphoonTrackPoint>& track, int steps) {
  std::vector<double> wind(static_cast<std::size_t>(std::max(1, steps)), 0.0);
  if (track.empty()) return wind;
  for (int t = 0; t < static_cast<int>(wind.size()); ++t) {
    const auto& p = track[std::min<std::size_t>(t, track.size() - 1)];
    wind[static_cast<std::size_t>(t)] = site.has_coordinates ? holland_wind_ms(p, site.latitude, site.longitude) : p.vmax_ms * 0.55;
  }
  return wind;
}

std::vector<double> wind_generation_from_track(const std::vector<WindResourceSite>& sites,
                                               const std::vector<TyphoonTrackPoint>& track,
                                               int steps,
                                               const TyphoonImpactOptions& opt) {
  std::vector<double> values(static_cast<std::size_t>(std::max(1, steps)), 0.0);
  if (sites.empty()) return values;
  const double total_capacity = std::accumulate(sites.begin(), sites.end(), 0.0, [](double sum, const auto& site) { return sum + site.capacity_mw; });
  if (total_capacity <= 1e-9) return values;
  for (int t = 0; t < static_cast<int>(values.size()); ++t) {
    const TyphoonTrackPoint* p = track.empty() ? nullptr : &track[std::min<std::size_t>(t, track.size() - 1)];
    double output = 0.0;
    for (const auto& site : sites) {
      double site_wind = p ? p->vmax_ms * 0.55 : 0.0;
      if (p && site.has_coordinates) site_wind = holland_wind_ms(*p, site.latitude, site.longitude);
      output += site.capacity_mw * wind_power_curve(site_wind, opt);
    }
    values[static_cast<std::size_t>(t)] = output;
  }
  return values;
}

std::optional<int> first_fault_step(const std::vector<DistributionResilienceFault>& faults) {
  if (faults.empty()) return std::nullopt;
  double first = std::numeric_limits<double>::infinity();
  for (const auto& f : faults) first = std::min(first, f.outage_start_hr);
  return std::max(1, static_cast<int>(std::floor(first)) + 1);
}

std::optional<int> last_new_fault_step(const std::vector<DistributionResilienceFault>& faults) {
  if (faults.empty()) return std::nullopt;
  double last = -std::numeric_limits<double>::infinity();
  for (const auto& f : faults) last = std::max(last, f.outage_start_hr);
  return std::max(1, static_cast<int>(std::floor(last)) + 1);
}

double series_value(const std::vector<double>& values, int index) {
  return index < static_cast<int>(values.size()) ? values[static_cast<std::size_t>(index)] : 0.0;
}

std::vector<double> sum_series(const std::vector<double>& a, const std::vector<double>& b, const std::vector<double>& c) {
  const int n = static_cast<int>(std::max({a.size(), b.size(), c.size()}));
  std::vector<double> out(static_cast<std::size_t>(std::max(1, n)), 0.0);
  for (int i = 0; i < static_cast<int>(out.size()); ++i) out[static_cast<std::size_t>(i)] = series_value(a, i) + series_value(b, i) + series_value(c, i);
  return out;
}

TimeSeriesProfile make_profile(int id, std::string name, std::vector<double> values) {
  TimeSeriesProfile profile;
  profile.id = id;
  profile.name = std::move(name);
  profile.values = std::move(values);
  return profile;
}

std::vector<double> profile_values(const TimeSeriesData& ts, const std::string& name);
bool has_profile_name(const TimeSeriesData& ts, const std::string& name);

std::vector<double> scale_profile_values(const std::vector<double>& values, double base) {
  std::vector<double> out;
  out.reserve(values.size());
  if (base <= 1e-9) return out;
  for (const double value : values) out.push_back(value / base);
  return out;
}

bool has_nonzero_value(const std::vector<double>& values) {
  return std::any_of(values.begin(), values.end(), [](double value) { return std::abs(value) > 1e-9; });
}

void add_multiplier_profile(nlohmann::json& profiles,
                            nlohmann::json& warnings,
                            int id,
                            const std::string& name,
                            const std::string& domain,
                            const std::vector<double>& raw_values,
                            double base,
                            const std::string& zero_base_warning) {
  if (raw_values.empty()) return;
  if (base <= 1e-9) {
    if (has_nonzero_value(raw_values)) warnings.push_back(zero_base_warning);
    return;
  }
  profiles.push_back({{"id", id},
                      {"name", name},
                      {"domain", domain},
                      {"scope", "aggregate"},
                      {"unit", "multiplier"},
                      {"values", scale_profile_values(raw_values, base)}});
}

nlohmann::json standard_time_series_from_candidate(const ScenarioCandidate& c,
                                                   const RenewableBreakdown& base_renewable,
                                                   double base_load) {
  nlohmann::json profiles = nlohmann::json::array();
  nlohmann::json warnings = nlohmann::json::array();

  const auto load = profile_values(c.time_series, "total_load_mw");
  const auto pv = profile_values(c.time_series, "pv_mw");
  const auto wind = profile_values(c.time_series, "wind_mw");
  const auto storage_soc = profile_values(c.time_series, "storage_soc");

  add_multiplier_profile(profiles, warnings, 0, "scenario_load_scale", "load", load, base_load,
                         "Base load is zero; load multiplier profile is omitted.");

  nlohmann::json load_profile_map = nlohmann::json::array();
  std::string component_source = "none";
  if (!c.component_load_profiles.is_null() && c.component_load_profiles.is_object()) {
    const auto& component_profiles = c.component_load_profiles.value("profiles", nlohmann::json::array());
    for (const auto& p : component_profiles) {
      if (!p.is_object() || !p.contains("values")) continue;
      nlohmann::json out = p;
      out["domain"] = "load";
      out["scope"] = "component";
      out["unit"] = "multiplier";
      profiles.push_back(std::move(out));
    }
    load_profile_map = c.component_load_profiles.value("load_profile_map", nlohmann::json::array());
    component_source = c.component_load_profiles.value("source", std::string("backend_per_load_site"));
  }

  add_multiplier_profile(profiles, warnings, 1, "scenario_wind_scale", "wind", wind, base_renewable.wind_mw,
                         "Base wind capacity is zero; wind multiplier profile is omitted.");
  add_multiplier_profile(profiles, warnings, 2, "scenario_pv_scale", "pv", pv, base_renewable.pv_mw,
                         "Base PV capacity is zero; PV multiplier profile is omitted.");

  if (has_profile_name(c.time_series, "storage_soc") && !storage_soc.empty()) {
    profiles.push_back({{"id", 7},
                        {"name", "storage_soc"},
                        {"domain", "storage"},
                        {"scope", "aggregate"},
                        {"unit", "soc_fraction"},
                        {"values", storage_soc}});
  }

  const bool has_load_profile = std::any_of(profiles.begin(), profiles.end(), [](const nlohmann::json& p) {
    return p.value("id", -1) == 0;
  });
  const bool has_wind_profile = std::any_of(profiles.begin(), profiles.end(), [](const nlohmann::json& p) {
    return p.value("id", -1) == 1;
  });
  const bool has_pv_profile = std::any_of(profiles.begin(), profiles.end(), [](const nlohmann::json& p) {
    return p.value("id", -1) == 2;
  });
  return {{"version", 1},
          {"unit_space", "dimensionless_multiplier"},
          {"profile_semantics", "source/load profiles are dimensionless multipliers against bound equipment base capacities"},
          {"num_steps", c.time_series.num_steps},
          {"step_duration_hr", c.time_series.step_duration_hr},
          {"profiles", profiles},
          {"binding", {{"assign_all_loads_to", load_profile_map.empty() ? (has_load_profile ? 0 : -1) : -1},
                       {"assign_all_pv_to", has_pv_profile ? 2 : -1},
                       {"load_profile_map", load_profile_map},
                       {"resilience_load_profile_id", has_load_profile ? 0 : -1},
                       {"resilience_wind_profile_id", has_wind_profile ? 1 : -1},
                       {"resilience_pv_profile_id", has_pv_profile ? 2 : -1},
                       {"resilience_renewable_profile_id", has_pv_profile ? 2 : (has_wind_profile ? 1 : -1)}}},
          {"normalization", {{"base_load_mw", base_load},
                              {"base_pv_mw", base_renewable.pv_mw},
                              {"base_wind_mw", base_renewable.wind_mw},
                              {"base_other_renewable_mw", base_renewable.other_mw},
                              {"base_renewable_mw", base_renewable.total_mw()},
                              {"profile_semantics", "dimensionless_multiplier"},
                              {"component_load_profile_source", component_source}}},
          {"warnings", warnings}};
}

void attach_standard_time_series(ScenarioCandidate& c, const RenewableBreakdown& base_renewable, double base_load) {
  c.standard_time_series = standard_time_series_from_candidate(c, base_renewable, base_load);
}

TimeSeriesData make_time_series(const std::vector<double>& load,
                                const std::vector<double>& pv,
                                const std::vector<double>& wind,
                                const std::vector<double>& other_renewable,
                                const StorageSocProfiles* storage = nullptr) {
  const auto renewable = sum_series(pv, wind, other_renewable);
  TimeSeriesData ts;
  ts.num_steps = static_cast<int>(std::max(load.size(), renewable.size()));
  ts.step_duration_hr = 1.0;
  std::vector<double> net(static_cast<std::size_t>(ts.num_steps), 0.0);
  for (int i = 0; i < ts.num_steps; ++i) net[static_cast<std::size_t>(i)] = series_value(load, i) - series_value(renewable, i);
  ts.profiles = {make_profile(1, "total_load_mw", load),
                 make_profile(2, "total_renewable_mw", renewable),
                 make_profile(3, "net_load_mw", std::move(net)),
                 make_profile(4, "pv_mw", pv),
                 make_profile(5, "wind_mw", wind),
                 make_profile(6, "other_renewable_mw", other_renewable)};
  if (storage && storage->has_storage) {
    ts.profiles.push_back(make_profile(7, "storage_soc", storage->storage_soc));
    ts.profiles.push_back(make_profile(8, "storage_energy_mwh", storage->storage_energy_mwh));
  }
  return ts;
}

TimeSeriesData make_time_series(const std::vector<double>& load, const std::vector<double>& renewable) {
  return make_time_series(load, renewable, std::vector<double>(renewable.size(), 0.0), std::vector<double>(renewable.size(), 0.0));
}

std::unordered_map<std::string, double> feature_summary(const std::vector<double>& load,
                                                        const std::vector<double>& pv,
                                                        const std::vector<double>& wind,
                                                        const std::vector<double>& other_renewable,
                                                        const StorageSocProfiles* storage = nullptr) {
  const auto renewable = sum_series(pv, wind, other_renewable);
  std::unordered_map<std::string, double> f;
  const int n = static_cast<int>(std::max(load.size(), renewable.size()));
  double load_sum = 0.0, ren_sum = 0.0, pv_sum = 0.0, wind_sum = 0.0, other_sum = 0.0;
  double peak_load = 0.0, net_sum = 0.0;
  double min_net = std::numeric_limits<double>::infinity();
  int negative_net_count = 0;
  double max_ramp = 0.0;
  double prev_net = 0.0;
  for (int i = 0; i < n; ++i) {
    const double l = series_value(load, i);
    const double p = series_value(pv, i);
    const double w = series_value(wind, i);
    const double o = series_value(other_renewable, i);
    const double r = p + w + o;
    const double net = l - r;
    load_sum += l;
    ren_sum += r;
    pv_sum += p;
    wind_sum += w;
    other_sum += o;
    net_sum += net;
    peak_load = std::max(peak_load, l);
    min_net = std::min(min_net, net);
    if (net < -1e-9) ++negative_net_count;
    if (i > 0) max_ramp = std::max(max_ramp, std::abs(net - prev_net));
    prev_net = net;
  }
  f["load_sum"] = load_sum;
  f["renewable_sum"] = ren_sum;
  f["pv_sum"] = pv_sum;
  f["wind_sum"] = wind_sum;
  f["other_renewable_sum"] = other_sum;
  f["peak_load"] = peak_load;
  f["net_load_sum"] = net_sum;
  f["min_net_load"] = std::isfinite(min_net) ? min_net : 0.0;
  f["negative_net_count"] = static_cast<double>(negative_net_count);
  f["max_net_ramp"] = max_ramp;
  f["renewable_share"] = load_sum > 1e-9 ? ren_sum / load_sum : 0.0;
  f["pv_share"] = load_sum > 1e-9 ? pv_sum / load_sum : 0.0;
  f["wind_share"] = load_sum > 1e-9 ? wind_sum / load_sum : 0.0;
  if (storage && storage->has_storage) {
    const double soc = storage->storage_soc.empty() ? storage->aggregate_soc : storage->storage_soc.front();
    const double energy = storage->storage_energy_mwh.empty() ? soc * storage->storage_capacity_mwh : storage->storage_energy_mwh.front();
    f["storage_count"] = static_cast<double>(storage->storage_count);
    f["storage_capacity_mwh"] = storage->storage_capacity_mwh;
    f["storage_soc"] = soc;
    f["storage_energy_mwh"] = energy;
  }
  if (n == 1) {
    f["load_mw"] = load.empty() ? 0.0 : load.front();
    f["renewable_mw"] = renewable.empty() ? 0.0 : renewable.front();
    f["pv_mw"] = pv.empty() ? 0.0 : pv.front();
    f["wind_mw"] = wind.empty() ? 0.0 : wind.front();
    f["other_renewable_mw"] = other_renewable.empty() ? 0.0 : other_renewable.front();
    f["net_load_mw"] = f["load_mw"] - f["renewable_mw"];
  }
  if (n >= 8760) {
    const int month_hours[12] = {744, 672, 744, 720, 744, 720, 744, 744, 720, 744, 720, 744};
    int start = 0;
    for (int m = 0; m < 12; ++m) {
      const int end = std::min(n, start + month_hours[m]);
      double ml = 0.0, mr = 0.0, mpv = 0.0, mwind = 0.0, mother = 0.0;
      for (int i = start; i < end; ++i) {
        const double p = series_value(pv, i);
        const double w = series_value(wind, i);
        const double o = series_value(other_renewable, i);
        ml += series_value(load, i);
        mpv += p;
        mwind += w;
        mother += o;
        mr += p + w + o;
      }
      f["month_load_" + std::to_string(m + 1)] = ml;
      f["month_renewable_" + std::to_string(m + 1)] = mr;
      f["month_pv_" + std::to_string(m + 1)] = mpv;
      f["month_wind_" + std::to_string(m + 1)] = mwind;
      f["month_other_renewable_" + std::to_string(m + 1)] = mother;
      start = end;
    }
  }
  return f;
}

std::unordered_map<std::string, double> feature_summary(const std::vector<double>& load, const std::vector<double>& renewable) {
  return feature_summary(load, renewable, std::vector<double>(renewable.size(), 0.0), std::vector<double>(renewable.size(), 0.0));
}

void apply_factor(std::vector<double>& values, const std::vector<double>& factors) {
  for (std::size_t i = 0; i < values.size(); ++i) values[i] *= i < factors.size() ? factors[i] : 1.0;
}

std::vector<std::vector<double>> perturb_load_components(const std::vector<std::vector<double>>& base_load_components,
                                                         const PerturbationOptions& opt,
                                                         std::mt19937& rng) {
  auto load_components = base_load_components;
  if (!opt.enable_load_perturbation) return load_components;
  for (auto& component : load_components) {
    const auto factors = smooth_noise_factors(static_cast<int>(component.size()), opt.load_sigma,
                                              opt.load_min_multiplier, opt.load_max_multiplier,
                                              opt.block_hours, rng);
    apply_factor(component, factors);
  }
  return load_components;
}

nlohmann::json component_load_profiles_to_json(const std::vector<LoadSiteBaseline>& sites,
                                               const std::vector<std::vector<double>>& component_profiles,
                                               int first_profile_id = 10) {
  nlohmann::json profiles = nlohmann::json::array();
  nlohmann::json map = nlohmann::json::array();
  const std::size_t count = std::min(sites.size(), component_profiles.size());
  for (std::size_t i = 0; i < count; ++i) {
    const auto& site = sites[i];
    const int profile_id = first_profile_id + static_cast<int>(i);
    std::vector<double> scale;
    scale.reserve(component_profiles[i].size());
    for (const double value : component_profiles[i]) scale.push_back(site.base_mw > 1e-9 ? value / site.base_mw : 0.0);
    const std::string kind = site.is_dc
        ? (site.is_bus_fallback ? "DC_BUS" : "DC_LOAD")
        : (site.is_bus_fallback ? "AC_BUS" : "AC_LOAD");
    const std::string prefix = site.is_dc ? (site.is_bus_fallback ? "scenario_dc_bus_" : "scenario_dc_load_")
                                          : (site.is_bus_fallback ? "scenario_ac_bus_" : "scenario_ac_load_");
    profiles.push_back({{"id", profile_id},
                        {"name", prefix + std::to_string(site.component_index) + "_scale"},
                        {"values", scale},
                        {"base_mw", site.base_mw},
                        {"source_site_id", site.id}});
    nlohmann::json row = {{"kind", kind}, {"profile_id", profile_id}, {"bus", site.bus}};
    if (!site.is_bus_fallback) row["load_index"] = site.component_index;
    map.push_back(std::move(row));
  }
  return {{"source", "backend_per_load_site"},
          {"profile_semantics", "dimensionless_multiplier"},
          {"site_count", count},
          {"profiles", profiles},
          {"load_profile_map", map}};
}

struct CandidateLoadRealization {
  ScenarioCandidate candidate;
  std::vector<std::vector<double>> load_components_after_perturbation;
};

CandidateLoadRealization make_candidate_with_load_components(const std::string& id, ScenarioFamily family,
                                                             const std::vector<LoadSiteBaseline>& load_sites,
                                                             const std::vector<std::vector<double>>& base_load_components,
                                                             const std::vector<double>& base_pv,
                                                             const std::vector<double>& base_wind,
                                                             const std::vector<double>& base_other_renewable,
                                                             const StorageSocBaseline& storage_baseline,
                                                             const PerturbationOptions& opt,
                                                             std::mt19937& rng) {
  auto load_components = perturb_load_components(base_load_components, opt, rng);
  const int renewable_steps = static_cast<int>(std::max({base_pv.size(), base_wind.size(), base_other_renewable.size(), std::size_t{1}}));
  int load_steps = renewable_steps;
  for (const auto& component : load_components) load_steps = std::max(load_steps, static_cast<int>(component.size()));
  auto load = sum_profiles(load_components, load_steps);
  auto renewable_factor = smooth_noise_factors(renewable_steps, opt.renewable_sigma,
                                               opt.renewable_min_multiplier, opt.renewable_max_multiplier,
                                               opt.block_hours, rng);
  std::vector<double> pv = base_pv;
  std::vector<double> wind = base_wind;
  std::vector<double> other = base_other_renewable;
  if (opt.enable_renewable_perturbation) {
    apply_factor(pv, renewable_factor);
    apply_factor(wind, renewable_factor);
    apply_factor(other, renewable_factor);
  }
  const auto storage = make_storage_soc_profiles(storage_baseline, static_cast<int>(std::max(load.size(), sum_series(pv, wind, other).size())), opt, rng);
  ScenarioCandidate c;
  c.id = id;
  c.family = family;
  c.time_series = make_time_series(load, pv, wind, other, &storage);
  c.features = feature_summary(load, pv, wind, other, &storage);
  c.component_load_profiles = component_load_profiles_to_json(load_sites, load_components);
  return {std::move(c), std::move(load_components)};
}

ScenarioCandidate make_candidate(const std::string& id, ScenarioFamily family,
                                 const std::vector<double>& base_load,
                                 const std::vector<double>& base_pv,
                                 const std::vector<double>& base_wind,
                                 const std::vector<double>& base_other_renewable,
                                 const StorageSocBaseline& storage_baseline,
                                 const PerturbationOptions& opt,
                                 std::mt19937& rng) {
  auto load_factor = smooth_noise_factors(static_cast<int>(base_load.size()), opt.load_sigma,
                                          opt.load_min_multiplier, opt.load_max_multiplier,
                                          opt.block_hours, rng);
  const int renewable_steps = static_cast<int>(std::max({base_pv.size(), base_wind.size(), base_other_renewable.size()}));
  auto renewable_factor = smooth_noise_factors(renewable_steps, opt.renewable_sigma,
                                               opt.renewable_min_multiplier, opt.renewable_max_multiplier,
                                               opt.block_hours, rng);
  std::vector<double> load = base_load;
  std::vector<double> pv = base_pv;
  std::vector<double> wind = base_wind;
  std::vector<double> other = base_other_renewable;
  if (opt.enable_load_perturbation) apply_factor(load, load_factor);
  if (opt.enable_renewable_perturbation) {
    apply_factor(pv, renewable_factor);
    apply_factor(wind, renewable_factor);
    apply_factor(other, renewable_factor);
  }
  const auto storage = make_storage_soc_profiles(storage_baseline, static_cast<int>(std::max(load.size(), sum_series(pv, wind, other).size())), opt, rng);
  ScenarioCandidate c;
  c.id = id;
  c.family = family;
  c.time_series = make_time_series(load, pv, wind, other, &storage);
  c.features = feature_summary(load, pv, wind, other, &storage);
  return c;
}

ScenarioCandidate make_candidate(const std::string& id, ScenarioFamily family,
                                 const std::vector<double>& base_load,
                                 const std::vector<double>& base_renewable,
                                 const StorageSocBaseline& storage_baseline,
                                 const PerturbationOptions& opt,
                                 std::mt19937& rng) {
  return make_candidate(id, family, base_load, base_renewable, std::vector<double>(base_renewable.size(), 0.0),
                        std::vector<double>(base_renewable.size(), 0.0), storage_baseline, opt, rng);
}

std::vector<std::string> feature_keys(const std::vector<ScenarioCandidate>& candidates) {
  std::set<std::string> keys;
  for (const auto& c : candidates) {
    for (const auto& [key, value] : c.features) {
      (void)value;
      keys.insert(key);
    }
  }
  return {keys.begin(), keys.end()};
}

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  return values.size() % 2 == 0 ? 0.5 * (values[mid - 1] + values[mid]) : values[mid];
}

std::vector<std::vector<double>> scaled_features(const std::vector<ScenarioCandidate>& candidates,
                                                 const std::vector<std::string>& keys,
                                                 bool robust_scale) {
  std::vector<std::vector<double>> x(candidates.size(), std::vector<double>(keys.size(), 0.0));
  for (std::size_t j = 0; j < keys.size(); ++j) {
    std::vector<double> col;
    col.reserve(candidates.size());
    for (const auto& c : candidates) {
      auto it = c.features.find(keys[j]);
      col.push_back(it == c.features.end() ? 0.0 : it->second);
    }
    double center = 0.0, scale = 1.0;
    if (robust_scale) {
      center = median(col);
      auto sorted = col;
      std::sort(sorted.begin(), sorted.end());
      const double q1 = sorted.empty() ? 0.0 : sorted[sorted.size() / 4];
      const double q3 = sorted.empty() ? 0.0 : sorted[(sorted.size() * 3) / 4];
      scale = std::max(1e-9, q3 - q1);
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) x[i][j] = robust_scale ? (col[i] - center) / scale : col[i];
  }
  return x;
}


bool hybrid_method(const std::string& method) {
  const auto m = lower_copy(method);
  return m == "hybrid_kmedoids_tail_5pct" || m == "hybrid_kmedoids_tail";
}

bool is_discrete_feature_key(const std::string& key) {
  return key == "regular_group" || key == "ssp_code" || key == "year" || key == "climate_data_found" ||
         key == "contingency_component_index" || key == "contingency_ordinal" || key == "typhoon_month";
}

std::vector<std::string> continuous_feature_keys(const std::vector<ScenarioCandidate>& candidates) {
  auto keys = feature_keys(candidates);
  keys.erase(std::remove_if(keys.begin(), keys.end(), is_discrete_feature_key), keys.end());
  return keys;
}

double quantile(std::vector<double> values, double q) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  q = clamp_value(q, 0.0, 1.0);
  const double pos = q * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = static_cast<std::size_t>(std::ceil(pos));
  if (lo == hi) return values[lo];
  const double alpha = pos - static_cast<double>(lo);
  return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

double feature_value(const ScenarioCandidate& c, const std::string& key) {
  const auto it = c.features.find(key);
  return it == c.features.end() ? 0.0 : it->second;
}

std::string id_field(const std::string& id, int field) {
  std::size_t start = 0;
  for (int i = 0; i < field; ++i) {
    start = id.find(':', start);
    if (start == std::string::npos) return {};
    ++start;
  }
  const auto end = id.find(':', start);
  return id.substr(start, end == std::string::npos ? std::string::npos : end - start);
}

std::string base_regime_label(const ScenarioCandidate& c) {
  if (c.family == ScenarioFamily::Regular) return "regular:" + id_field(c.id, 1) + ":" + id_field(c.id, 2);
  if (c.family == ScenarioFamily::Reliability) return std::string("reliability:") + (c.contingency ? std::string(to_string(c.contingency->type)) : "unknown");
  if (c.family == ScenarioFamily::Resilience) {
    const double faults = feature_value(c, "fault_count");
    const std::string bin = faults < 0.5 ? "faults_0" : faults < 3.5 ? "faults_1_3" : "faults_ge4";
    const std::string intensity = c.resilience_event ? std::string(to_string(c.resilience_event->selected_intensity)) : id_field(c.id, 1);
    return "resilience:" + intensity + ":" + bin;
  }
  return "unknown";
}

std::unordered_map<std::string, double> raw_risk_source_scores(const ScenarioCandidate& c) {
  std::unordered_map<std::string, double> scores;
  scores["load"] = std::max(feature_value(c, "load_sum"), feature_value(c, "peak_load"));
  scores["renewable_deficit"] = -feature_value(c, "renewable_sum") + feature_value(c, "net_load_sum");
  scores["pv_deficit"] = -feature_value(c, "pv_sum");
  scores["wind_deficit"] = -feature_value(c, "wind_sum");
  scores["net_load"] = feature_value(c, "net_load_sum") + feature_value(c, "max_net_ramp");
  if (feature_value(c, "storage_capacity_mwh") > 1e-9) {
    scores["storage_availability"] = 1.0 - feature_value(c, "storage_soc");
  }
  if (c.family == ScenarioFamily::Reliability) scores["network"] = 1.0 + static_cast<double>(c.outage_signature.size());
  if (c.family == ScenarioFamily::Resilience) {
    scores["hazard"] = feature_value(c, "selected_track_max_vmax_ms");
    scores["network"] = feature_value(c, "fault_count") + static_cast<double>(std::count(c.outage_signature.begin(), c.outage_signature.end(), 1));
    scores["failure_probability"] = feature_value(c, "peak_failure_probability");
    scores["renewable_loss"] = -feature_value(c, "renewable_sum");
  }
  return scores;
}

void enrich_candidate_risk_metadata(std::vector<ScenarioCandidate>& candidates, const ClusteringOptions& opt) {
  std::set<std::string> keys;
  std::vector<std::unordered_map<std::string, double>> raw;
  raw.reserve(candidates.size());
  for (const auto& c : candidates) {
    raw.push_back(raw_risk_source_scores(c));
    for (const auto& [k, v] : raw.back()) { (void)v; keys.insert(k); }
  }
  std::map<std::string, std::vector<double>> scaled_by_key;
  for (const auto& key : keys) {
    std::vector<double> col;
    for (const auto& r : raw) col.push_back(r.count(key) ? r.at(key) : 0.0);
    const double center = median(col);
    auto sorted = col;
    std::sort(sorted.begin(), sorted.end());
    const double scale = std::max(1e-9, sorted.empty() ? 1.0 : sorted[(sorted.size() * 3) / 4] - sorted[sorted.size() / 4]);
    auto& out = scaled_by_key[key];
    out.resize(candidates.size(), 0.0);
    for (std::size_t i = 0; i < candidates.size(); ++i) out[i] = (col[i] - center) / scale;
  }
  std::vector<double> risks(candidates.size(), 0.0);
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    double max_score = -std::numeric_limits<double>::infinity();
    double positive_norm = 0.0;
    std::vector<double> vals;
    for (const auto& key : keys) {
      const double v = scaled_by_key[key][i];
      candidates[i].risk_source_scores[key] = v;
      max_score = std::max(max_score, v);
      if (v > 1.0) positive_norm += (v - 1.0) * (v - 1.0);
      vals.push_back(v);
    }
    double mean = vals.empty() ? 0.0 : std::accumulate(vals.begin(), vals.end(), 0.0) / static_cast<double>(vals.size());
    double var = 0.0;
    for (double v : vals) var += (v - mean) * (v - mean);
    candidates[i].risk_score = (std::isfinite(max_score) ? max_score : 0.0) + 0.25 * std::sqrt(positive_norm) + 0.15 * (vals.empty() ? 0.0 : std::sqrt(var / static_cast<double>(vals.size())));
    risks[i] = candidates[i].risk_score;
  }
  const double global_q = quantile(risks, opt.source_tail_quantile);
  std::map<std::string, double> source_q95, source_q90;
  for (const auto& key : keys) {
    source_q95[key] = quantile(scaled_by_key[key], opt.source_tail_quantile);
    source_q90[key] = quantile(scaled_by_key[key], opt.coupling_quantile);
  }
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    int source_tail_count = 0;
    std::string first_tail;
    if (candidates[i].risk_score >= global_q) candidates[i].anchor_reasons.push_back("global_tail_q95");
    for (const auto& key : keys) {
      const double v = candidates[i].risk_source_scores[key];
      if (v >= source_q95[key]) { candidates[i].anchor_reasons.push_back("source_tail:" + key); if (first_tail.empty()) first_tail = key; }
      if (v >= source_q90[key]) ++source_tail_count;
    }
    if (source_tail_count >= 2) candidates[i].anchor_reasons.push_back("coupling_tail:q90");
    std::string suffix = "normal";
    if (std::find(candidates[i].anchor_reasons.begin(), candidates[i].anchor_reasons.end(), "coupling_tail:q90") != candidates[i].anchor_reasons.end()) suffix = "coupled_tail";
    else if (!first_tail.empty()) suffix = first_tail + "_tail";
    else if (candidates[i].risk_score >= global_q) suffix = "global_tail";
    candidates[i].regime_label = base_regime_label(candidates[i]) + ":" + suffix;
  }
}

double candidate_distance(const std::vector<double>& a, const std::vector<double>& b,
                          const ScenarioCandidate& ca, const ScenarioCandidate& cb,
                          const ClusteringOptions& opt) {
  double continuous = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) { const double d = a[i] - b[i]; continuous += d * d; }
  continuous = std::sqrt(continuous);
  double discrete = 0.0;
  const std::size_t n = std::max(ca.outage_signature.size(), cb.outage_signature.size());
  for (std::size_t i = 0; i < n; ++i) {
    const int va = i < ca.outage_signature.size() ? ca.outage_signature[i] : 0;
    const int vb = i < cb.outage_signature.size() ? cb.outage_signature[i] : 0;
    if (va != vb) discrete += 1.0;
  }
  const double regime = hybrid_method(opt.method) && !ca.regime_label.empty() && !cb.regime_label.empty() && ca.regime_label != cb.regime_label ? 1.0 : 0.0;
  return opt.continuous_weight * continuous + opt.outage_hamming_weight * discrete + opt.regime_weight * regime;
}

std::vector<int> select_tail_anchors(std::vector<ScenarioCandidate>& candidates, int k, const ClusteringOptions& opt) {
  std::vector<int> anchors;
  if (!opt.include_tail_anchors || candidates.empty() || k <= 0) return anchors;
  std::map<std::string, int> best_by_regime;
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    if (candidates[i].family == ScenarioFamily::Resilience && feature_value(candidates[i], "fault_count") > 0.5) {
      candidates[i].anchor_reasons.push_back("fault_sequence_nonzero");
    }
    if (!candidates[i].anchor_reasons.empty()) anchors.push_back(i);
    auto it = best_by_regime.find(candidates[i].regime_label);
    if (it == best_by_regime.end() || candidates[i].risk_score > candidates[it->second].risk_score) best_by_regime[candidates[i].regime_label] = i;
  }
  for (const auto& [regime, idx] : best_by_regime) {
    (void)regime;
    if (std::find(anchors.begin(), anchors.end(), idx) == anchors.end()) { candidates[idx].anchor_reasons.push_back("major_regime:" + candidates[idx].regime_label); anchors.push_back(idx); }
  }
  std::sort(anchors.begin(), anchors.end()); anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
  std::sort(anchors.begin(), anchors.end(), [&](int a, int b) {
    const bool af = candidates[a].family == ScenarioFamily::Resilience && feature_value(candidates[a], "fault_count") > 0.5;
    const bool bf = candidates[b].family == ScenarioFamily::Resilience && feature_value(candidates[b], "fault_count") > 0.5;
    if (af != bf) return af;
    return candidates[a].risk_score > candidates[b].risk_score;
  });
  if (static_cast<int>(anchors.size()) > k) anchors.resize(static_cast<std::size_t>(k));
  for (int idx : anchors) { candidates[idx].tail_anchor = true; candidates[idx].frozen_medoid = opt.freeze_anchors; }
  return anchors;
}

std::vector<int> initialize_medoids_kmeanspp(const std::vector<ScenarioCandidate>& candidates,
                                             const std::vector<std::vector<double>>& x,
                                             int k,
                                             const ClusteringOptions& opt,
                                             const std::vector<int>& anchors) {
  std::vector<int> medoids = anchors;
  std::sort(medoids.begin(), medoids.end()); medoids.erase(std::unique(medoids.begin(), medoids.end()), medoids.end());
  std::mt19937 rng(opt.seed);
  if (medoids.empty() && !candidates.empty()) medoids.push_back(0);
  while (static_cast<int>(medoids.size()) < k) {
    std::vector<double> weights(candidates.size(), 0.0); double total = 0.0;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
      if (std::find(medoids.begin(), medoids.end(), i) != medoids.end()) continue;
      double nearest = std::numeric_limits<double>::infinity();
      for (int m : medoids) nearest = std::min(nearest, candidate_distance(x[i], x[m], candidates[i], candidates[m], opt));
      weights[static_cast<std::size_t>(i)] = std::max(1e-12, candidates[i].probability) * nearest * nearest;
      total += weights[static_cast<std::size_t>(i)];
    }
    int chosen = -1;
    if (total > 1e-12) {
      std::uniform_real_distribution<double> dist(0.0, total); double draw = dist(rng);
      for (int i = 0; i < static_cast<int>(weights.size()); ++i) { draw -= weights[static_cast<std::size_t>(i)]; if (draw <= 0.0) { chosen = i; break; } }
    }
    if (chosen < 0) for (int i = 0; i < static_cast<int>(candidates.size()); ++i) if (std::find(medoids.begin(), medoids.end(), i) == medoids.end()) { chosen = i; break; }
    if (chosen < 0) break;
    medoids.push_back(chosen);
  }
  return medoids;
}

struct ReductionMetrics { double transport_mean{0.0}; double transport_max{0.0}; double regime_mass_l1{0.0}; double global_tail_coverage{0.0}; double coupling_tail_coverage{0.0}; std::unordered_map<std::string, double> source_tail_coverage; };

ReductionMetrics compute_reduction_metrics(const std::vector<ScenarioCandidate>& candidates, const std::vector<ScenarioCluster>& clusters) {
  ReductionMetrics m; std::set<std::string> retained; std::map<std::string, double> before, after;
  for (const auto& c : candidates) before[c.regime_label] += c.probability;
  for (const auto& cl : clusters) { retained.insert(cl.representative_id); after[cl.representative.regime_label] += cl.probability; m.transport_mean += cl.probability * cl.average_distance; m.transport_max = std::max(m.transport_max, cl.max_distance); }
  std::set<std::string> regimes; for (const auto& [r,v] : before) { (void)v; regimes.insert(r); } for (const auto& [r,v] : after) { (void)v; regimes.insert(r); }
  for (const auto& r : regimes) m.regime_mass_l1 += std::abs(before[r] - after[r]);
  auto coverage = [&](const std::string& reason) { int denom=0,numer=0; for (const auto& c : candidates) if (std::find(c.anchor_reasons.begin(), c.anchor_reasons.end(), reason) != c.anchor_reasons.end()) { ++denom; if (retained.count(c.id)) ++numer; } return denom ? static_cast<double>(numer)/denom : 1.0; };
  m.global_tail_coverage = coverage("global_tail_q95"); m.coupling_tail_coverage = coverage("coupling_tail:q90");
  std::set<std::string> source_reasons; for (const auto& c : candidates) for (const auto& r : c.anchor_reasons) if (r.rfind("source_tail:",0)==0) source_reasons.insert(r);
  for (const auto& r : source_reasons) m.source_tail_coverage[r.substr(12)] = coverage(r);
  return m;
}

nlohmann::json metrics_to_json(const ReductionMetrics& m) {
  nlohmann::json source = nlohmann::json::object(); for (const auto& [k,v] : m.source_tail_coverage) source[k]=v;
  return {{"transport_mean",m.transport_mean},{"transport_max",m.transport_max},{"regime_mass_l1",m.regime_mass_l1},{"global_tail_coverage",m.global_tail_coverage},{"coupling_tail_coverage",m.coupling_tail_coverage},{"source_tail_coverage",source}};
}

std::vector<ScenarioCluster> cluster_candidates_core(std::vector<ScenarioCandidate> candidates, int requested_k, const ClusteringOptions& opt, const std::vector<int>& anchors) {
  if (candidates.empty()) return {}; const int n=static_cast<int>(candidates.size()); const int k=std::clamp(requested_k,1,n);
  const auto keys = continuous_feature_keys(candidates); const auto x = scaled_features(candidates, keys, opt.robust_scale);
  auto medoids = initialize_medoids_kmeanspp(candidates, x, k, opt, anchors);
  std::vector<char> frozen(static_cast<std::size_t>(k), 0); for (int c=0;c<static_cast<int>(medoids.size());++c) frozen[static_cast<std::size_t>(c)] = candidates[medoids[c]].frozen_medoid ? 1 : 0;
  std::vector<int> assignment(n,0);
  for (int iter=0; iter<std::max(1,opt.max_iterations); ++iter) {
    bool changed=false;
    for (int i=0;i<n;++i) { double best=std::numeric_limits<double>::infinity(); int bc=0; for (int c=0;c<k;++c) { double d=candidate_distance(x[i],x[medoids[c]],candidates[i],candidates[medoids[c]],opt); if (d<best) {best=d; bc=c;} } if (assignment[i]!=bc) {assignment[i]=bc; changed=true;} }
    for (int c=0;c<k;++c) { if (frozen[static_cast<std::size_t>(c)]) continue; double best_sum=std::numeric_limits<double>::infinity(); int best_medoid=medoids[c]; for (int i=0;i<n;++i) { if (assignment[i]!=c) continue; double sum=0.0; for (int j=0;j<n;++j) if (assignment[j]==c) sum += candidates[j].probability*candidate_distance(x[i],x[j],candidates[i],candidates[j],opt); if (sum<best_sum) {best_sum=sum; best_medoid=i;} } if (medoids[c]!=best_medoid) {medoids[c]=best_medoid; changed=true;} }
    if (!changed) break;
  }
  std::vector<ScenarioCluster> clusters(static_cast<std::size_t>(k));
  for (int c=0;c<k;++c) { auto& cl=clusters[static_cast<std::size_t>(c)]; cl.cluster_id=c; cl.representative_id=candidates[medoids[c]].id; cl.representative=candidates[medoids[c]]; cl.frozen_medoid=candidates[medoids[c]].frozen_medoid; cl.anchor_reasons=candidates[medoids[c]].anchor_reasons; }
  for (int i=0;i<n;++i) { auto& cl=clusters[static_cast<std::size_t>(assignment[i])]; cl.member_ids.push_back(candidates[i].id); cl.probability += candidates[i].probability; double d=candidate_distance(x[i],x[medoids[assignment[i]]],candidates[i],candidates[medoids[assignment[i]]],opt); cl.average_distance += candidates[i].probability*d; cl.max_distance=std::max(cl.max_distance,d); }
  for (auto& cl:clusters) { cl.member_count=static_cast<int>(cl.member_ids.size()); if (cl.probability>1e-12) cl.average_distance/=cl.probability; }
  double ps=std::accumulate(clusters.begin(),clusters.end(),0.0,[](double s,const ScenarioCluster& c){return s+c.probability;}); if (ps>1e-12) for (auto& cl:clusters) cl.probability/=ps;
  return clusters;
}

std::vector<ScenarioCluster> cluster_candidates(std::vector<ScenarioCandidate> candidates, int requested_k, const ClusteringOptions& opt, nlohmann::json* audit = nullptr) {
  if (candidates.empty()) return {};
  enrich_candidate_risk_metadata(candidates, opt);
  std::vector<int> anchors;
  if (hybrid_method(opt.method)) anchors = select_tail_anchors(candidates, std::clamp(requested_k,1,static_cast<int>(candidates.size())), opt);
  auto clusters = cluster_candidates_core(candidates, requested_k, opt, anchors);
  auto metrics = compute_reduction_metrics(candidates, clusters);
  if (audit) {
    (*audit)["clustering_method"] = hybrid_method(opt.method) ? "hybrid_kmedoids_tail_5pct" : "weighted_k_medoids";
    (*audit)["screening_score"] = "cheap_consequence_proxy_v1";
    (*audit)["clearing_audit_status"] = "not_run";
    (*audit)["tail_anchor_count"] = anchors.size();
    (*audit)["frozen_medoid_count"] = std::count_if(clusters.begin(), clusters.end(), [](const auto& c){return c.frozen_medoid;});
    (*audit)["metrics"] = metrics_to_json(metrics);
    nlohmann::json anchor_ids=nlohmann::json::array(); for (int idx:anchors) anchor_ids.push_back(candidates[static_cast<std::size_t>(idx)].id); (*audit)["anchor_ids"] = anchor_ids;
    if (hybrid_method(opt.method) && opt.compare_baseline) { ClusteringOptions b=opt; b.method="weighted_k_medoids"; auto baseline_candidates=candidates; for (auto& c:baseline_candidates) { c.tail_anchor=false; c.frozen_medoid=false; } auto bc=cluster_candidates_core(baseline_candidates, requested_k, b, {}); auto bm=compute_reduction_metrics(candidates, bc); (*audit)["method_comparison"]={{"baseline_method","weighted_k_medoids"},{"hybrid_kmedoids_tail_5pct",metrics_to_json(metrics)},{"weighted_k_medoids",metrics_to_json(bm)},{"delta",{{"transport_mean",metrics.transport_mean-bm.transport_mean},{"regime_mass_l1",metrics.regime_mass_l1-bm.regime_mass_l1},{"global_tail_coverage",metrics.global_tail_coverage-bm.global_tail_coverage}}}}; }
  }
  return clusters;
}

void add_contingency(std::vector<ContingencyDefinition>& out, ContingencyDefinition c,
                     const ReliabilityScenarioOptions& options) {
  if (options.max_contingencies > 0 && static_cast<int>(out.size()) >= options.max_contingencies) return;
  out.push_back(std::move(c));
}

std::vector<double> profile_values(const TimeSeriesData& ts, const std::string& name) {
  for (const auto& p : ts.profiles) {
    if (p.name == name) return p.values;
  }
  return std::vector<double>(static_cast<std::size_t>(std::max(1, ts.num_steps)), 0.0);
}

bool has_profile_name(const TimeSeriesData& ts, const std::string& name) {
  return std::any_of(ts.profiles.begin(), ts.profiles.end(), [&](const auto& p) { return p.name == name; });
}

StorageSocProfiles storage_profiles_from_time_series(const TimeSeriesData& ts, const ScenarioCandidate& c) {
  if (!has_profile_name(ts, "storage_soc")) return StorageSocProfiles{};
  StorageSocProfiles storage;
  storage.has_storage = true;
  storage.storage_count = static_cast<int>(feature_value(c, "storage_count"));
  storage.storage_capacity_mwh = feature_value(c, "storage_capacity_mwh");
  storage.storage_soc = profile_values(ts, "storage_soc");
  storage.storage_energy_mwh = profile_values(ts, "storage_energy_mwh");
  storage.aggregate_soc = storage.storage_soc.empty() ? feature_value(c, "storage_soc") : storage.storage_soc.front();
  return storage;
}

nlohmann::json time_series_to_json(const TimeSeriesData& ts) {
  nlohmann::json profiles = nlohmann::json::array();
  for (const auto& p : ts.profiles) {
    profiles.push_back({{"id", p.id}, {"name", p.name}, {"values", p.values}});
  }
  return {{"num_steps", ts.num_steps}, {"step_duration_hr", ts.step_duration_hr}, {"profiles", profiles}};
}

nlohmann::json contingency_to_json(const ContingencyDefinition& c) {
  return {{"id", c.id},
          {"type", to_string(c.type)},
          {"component_index", c.component_index},
          {"display_name", c.display_name},
          {"affected_ac_branches", c.affected_ac_branches},
          {"affected_dc_branches", c.affected_dc_branches},
          {"affected_converters", c.affected_converters},
          {"affected_dcdc_converters", c.affected_dcdc_converters},
          {"affected_buses", c.affected_buses},
          {"affected_generators", c.affected_generators},
          {"affected_loads", c.affected_loads},
          {"affected_storage", c.affected_storage}};
}

nlohmann::json resilience_event_to_json(const ResilienceEventDefinition& e) {
  nlohmann::json faults = nlohmann::json::array();
  for (const auto& f : e.faults) {
    faults.push_back({{"branch_type", to_string(f.branch_kind)},
                     {"branch_index", f.branch_index},
                     {"start_hr", f.outage_start_hr},
                     {"repair_hr", f.repair_duration_hr},
                     {"repair_duration_hr", f.repair_duration_hr},
                     {"name", f.name}});
  }
  nlohmann::json track = nlohmann::json::array();
  for (const auto& p : e.track) {
    track.push_back({{"hour", p.hour}, {"lat", p.latitude}, {"lon", p.longitude}, {"vmax_ms", p.vmax_ms}});
  }
  nlohmann::json branch_risks = nlohmann::json::array();
  for (const auto& r : e.branch_risks) {
    branch_risks.push_back({{"branch_type", to_string(r.branch_kind)},
                            {"branch_index", r.branch_index},
                            {"peak_wind_ms", r.peak_wind_ms},
                            {"peak_rain_mm_hr", r.peak_rain_mm_hr},
                            {"peak_failure_probability", r.peak_failure_probability}});
  }
  return {{"id", e.id},
          {"requested_intensity", to_string(e.requested_intensity)},
          {"selected_intensity", to_string(e.selected_intensity)},
          {"selected_track_max_vmax_ms", e.selected_track_max_vmax_ms},
          {"selected_sample_id", e.selected_sample_id},
          {"used_catalog_sample", e.used_catalog_sample},
          {"used_category_fallback", e.used_category_fallback},
          {"used_approximate_repair_order", e.used_approximate_repair_order},
          {"repair_fault_count", e.repair_fault_count},
          {"month", e.month},
          {"stage1_start_step", e.stage1_start_step ? nlohmann::json(*e.stage1_start_step) : nlohmann::json(nullptr)},
          {"stage1_end_step", e.stage1_end_step ? nlohmann::json(*e.stage1_end_step) : nlohmann::json(nullptr)},
          {"faults", faults},
          {"track", track},
          {"branch_risks", branch_risks},
          {"pv_typhoon_multiplier", e.pv_typhoon_multiplier},
          {"wind_typhoon_multiplier", e.wind_typhoon_multiplier},
          {"renewable_typhoon_multiplier", e.renewable_typhoon_multiplier},
          {"load_typhoon_multiplier", e.load_typhoon_multiplier}};
}

nlohmann::json candidate_coverage_point(const ScenarioCandidate& c,
                                        bool compact_resilience_event = false) {
  nlohmann::json features = nlohmann::json::object();
  for (const auto& [key, value] : c.features) features[key] = value;
  nlohmann::json risk_source_scores = nlohmann::json::object();
  for (const auto& [key, value] : c.risk_source_scores) risk_source_scores[key] = value;
  nlohmann::json out = {{"id", c.id},
                        {"family", to_string(c.family)},
                        {"probability", c.probability},
                        {"features", features},
                        {"regime_label", c.regime_label},
                        {"risk_score", c.risk_score},
                        {"risk_source_scores", risk_source_scores},
                        {"tail_anchor", c.tail_anchor},
                        {"frozen_medoid", c.frozen_medoid},
                        {"anchor_reasons", c.anchor_reasons}};
  if (c.contingency) out["contingency"] = contingency_to_json(*c.contingency);
  if (c.resilience_event) {
    if (compact_resilience_event) {
      const auto& event = *c.resilience_event;
      out["resilience_event"] = {
          {"id", event.id},
          {"selected_intensity", to_string(event.selected_intensity)},
          {"selected_track_max_vmax_ms", event.selected_track_max_vmax_ms},
          {"selected_sample_id", event.selected_sample_id},
          {"month", event.month},
          {"used_approximate_repair_order",
           event.used_approximate_repair_order},
          {"repair_fault_count", event.repair_fault_count}};
    } else {
      out["resilience_event"] = resilience_event_to_json(*c.resilience_event);
    }
  }
  return out;
}

nlohmann::json candidate_to_json(const ScenarioCandidate& c) {
  nlohmann::json out = candidate_coverage_point(c);
  out["time_series"] = time_series_to_json(c.time_series);
  if (!c.standard_time_series.is_null() && !c.standard_time_series.empty()) {
    out["standard_time_series"] = c.standard_time_series;
  }
  if (!c.component_load_profiles.is_null() && !c.component_load_profiles.empty()) out["component_load_profiles"] = c.component_load_profiles;
  return out;
}

nlohmann::json cluster_to_json(const ScenarioCluster& c) {
  return {{"cluster_id", c.cluster_id},
          {"representative_id", c.representative_id},
          {"probability", c.probability},
          {"member_count", c.member_count},
          {"member_ids", c.member_ids},
          {"average_distance", c.average_distance},
          {"max_distance", c.max_distance},
          {"frozen_medoid", c.frozen_medoid},
          {"anchor_reasons", c.anchor_reasons},
          {"representative", candidate_to_json(c.representative)}};
}

}  // namespace

const char* to_string(ScenarioFamily family) {
  switch (family) {
    case ScenarioFamily::Regular: return "regular";
    case ScenarioFamily::Reliability: return "reliability";
    case ScenarioFamily::Resilience: return "resilience";
  }
  return "unknown";
}

const char* to_string(ContingencyComponentType type) {
  switch (type) {
    case ContingencyComponentType::ACBranch: return "ACBranch";
    case ContingencyComponentType::DCBranch: return "DCBranch";
    case ContingencyComponentType::VSCConverter: return "VSCConverter";
    case ContingencyComponentType::DCDCConverter: return "DCDCConverter";
    case ContingencyComponentType::ACBus: return "ACBus";
    case ContingencyComponentType::DCBus: return "DCBus";
    case ContingencyComponentType::ACGenerator: return "ACGenerator";
    case ContingencyComponentType::ACStaticGenerator: return "ACStaticGenerator";
    case ContingencyComponentType::ACRenewable: return "ACRenewable";
    case ContingencyComponentType::ACPVSystem: return "ACPVSystem";
    case ContingencyComponentType::DCStaticGenerator: return "DCStaticGenerator";
    case ContingencyComponentType::DCPVArray: return "DCPVArray";
    case ContingencyComponentType::ACLoad: return "ACLoad";
    case ContingencyComponentType::DCLoad: return "DCLoad";
    case ContingencyComponentType::ACStorage: return "ACStorage";
    case ContingencyComponentType::DCStorage: return "DCStorage";
    case ContingencyComponentType::MobileStorage: return "MobileStorage";
    case ContingencyComponentType::Transformer2W: return "Transformer2W";
    case ContingencyComponentType::Transformer3W: return "Transformer3W";
    case ContingencyComponentType::VPP: return "VPP";
    case ContingencyComponentType::Microgrid: return "Microgrid";
  }
  return "Unknown";
}

ContingencyComponentType contingency_component_type_from_string(const std::string& value) {
  const auto v = lower_copy(value);
  if (v == "dcbranch") return ContingencyComponentType::DCBranch;
  if (v == "vscconverter") return ContingencyComponentType::VSCConverter;
  if (v == "dcdcconverter") return ContingencyComponentType::DCDCConverter;
  if (v == "acbus") return ContingencyComponentType::ACBus;
  if (v == "dcbus") return ContingencyComponentType::DCBus;
  if (v == "acgenerator") return ContingencyComponentType::ACGenerator;
  if (v == "acload") return ContingencyComponentType::ACLoad;
  if (v == "dcload") return ContingencyComponentType::DCLoad;
  return ContingencyComponentType::ACBranch;
}

std::vector<ContingencyDefinition> enumerate_n1_contingencies(const HybridPowerSystem& sys,
                                                               const ReliabilityScenarioOptions& options) {
  std::vector<ContingencyDefinition> out;
  auto maybe_add = [&](ContingencyDefinition c) { add_contingency(out, std::move(c), options); };

  // Keep this catalog aligned with reliability_assessment.cpp::build_fmea_catalog:
  // AC generators, AC branches, DC branches, VSC converters, AC static gens,
  // AC renewable gens, AC storage, 2W transformers, and 3W transformers.
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    ContingencyDefinition c{component_id("ac_gen", g.index), ContingencyComponentType::ACGenerator, g.index, display_name(g.name, "Gen", g.index)};
    c.affected_generators.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    ContingencyDefinition c;
    c.id = component_id("ac_branch", br.index);
    c.type = ContingencyComponentType::ACBranch;
    c.component_index = br.index;
    c.display_name = br.name.empty() ? "ACBr_" + std::to_string(br.from_bus) + "-" + std::to_string(br.to_bus) : br.name;
    c.affected_ac_branches.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    ContingencyDefinition c;
    c.id = component_id("dc_branch", br.index);
    c.type = ContingencyComponentType::DCBranch;
    c.component_index = br.index;
    c.display_name = br.name.empty() ? "DCBr_" + std::to_string(br.from_bus) + "-" + std::to_string(br.to_bus) : br.name;
    c.affected_dc_branches.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& vsc : sys.vsc_converters) {
    if (!vsc.in_service) continue;
    ContingencyDefinition c{component_id("vsc", vsc.index), ContingencyComponentType::VSCConverter, vsc.index, display_name(vsc.name, "VSC", vsc.index)};
    c.affected_converters.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    ContingencyDefinition c{component_id("ac_sgen", sg.index), ContingencyComponentType::ACStaticGenerator, sg.index, display_name(sg.name, "SGen", sg.index)};
    c.affected_generators.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    ContingencyDefinition c{component_id("ac_ren", rg.index), ContingencyComponentType::ACRenewable, rg.index, display_name(rg.name, "RGen", rg.index)};
    c.affected_generators.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    ContingencyDefinition c{component_id("ac_storage", st.index), ContingencyComponentType::ACStorage, st.index, display_name(st.name, "BESS", st.index)};
    c.affected_storage.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& t : sys.ac.transformers_2w) {
    if (!t.in_service) continue;
    ContingencyDefinition c{component_id("trafo2w", t.index), ContingencyComponentType::Transformer2W, t.index, display_name(t.name, "Trafo2W", t.index)};
    c.affected_ac_branches.push_back(c.id);
    maybe_add(std::move(c));
  }
  for (const auto& t : sys.ac.transformers_3w) {
    if (!t.in_service) continue;
    ContingencyDefinition c{component_id("trafo3w", t.index), ContingencyComponentType::Transformer3W, t.index, display_name(t.name, "Trafo3W", t.index)};
    c.affected_ac_branches.push_back(c.id);
    maybe_add(std::move(c));
  }
  return out;
}

RegularScenarioResult generate_regular_scenarios(const HybridPowerSystem& sys,
                                                 const RegularScenarioOptions& options,
                                                 const PerturbationOptions& perturbation,
                                                 const ClusteringOptions& clustering,
                                                 std::vector<std::string>* warnings) {
  RegularScenarioResult result;
  if (!options.enabled) return result;
  const int steps = std::max(1, options.num_steps);
  const auto load_sites = collect_load_sites(sys);
  const double load0 = sum_load_sites(load_sites);
  const auto renewable0 = renewable_breakdown_mw(sys);
  if (warnings && load0 <= 1e-9) warnings->push_back("No positive load found; regular scenarios use zero load baseline.");
  if (warnings && renewable0.total_mw() <= 1e-9) warnings->push_back("No renewable resource found; regular scenarios use zero renewable baseline.");
  const auto base_load = synthesize_base_load(load0, steps);
  const auto base_pv = synthesize_base_pv(renewable0.pv_mw, steps);
  const auto base_wind = synthesize_base_wind(renewable0.wind_mw, steps);
  const auto base_other = synthesize_base_other_renewable(renewable0.other_mw, steps);
  const auto base_temp = synthesize_base_temperature_c(steps);
  const auto base_ghi = synthesize_base_ghi_w_m2(steps);
  const auto storage_baseline = storage_soc_baseline(sys);
  if (warnings && perturbation.enable_storage_soc_perturbation && storage_baseline.empty()) {
    warnings->push_back("Storage SOC perturbation enabled but no in-service storage with positive e_rated_mwh was found.");
  }
  const std::vector<std::string> ssps = options.ssp_levels.empty()
      ? std::vector<std::string>{options.ssp}
      : options.ssp_levels;
  const std::vector<int> years = options.years.empty()
      ? std::vector<int>{options.year}
      : options.years;
  const int group_count = std::max(1, static_cast<int>(ssps.size() * years.size()));
  const int per_group_candidates = std::max(options.cluster_count, std::max(1, options.candidate_count / group_count));
  const auto profile_plan = component_profile_plan(
      load_sites.size(), steps,
      static_cast<std::uint64_t>(per_group_candidates) * static_cast<std::uint64_t>(group_count),
      static_cast<std::uint64_t>(options.cluster_count) * static_cast<std::uint64_t>(group_count));
  warn_component_profile_fallback("Regular", profile_plan, warnings);
  const auto base_load_components = profile_plan.per_component
      ? synthesize_base_load_profiles(load_sites, steps)
      : std::vector<std::vector<double>>{};
  result.candidate_count = 0;
  nlohmann::json coverage_samples = nlohmann::json::array();
  int cluster_offset = 0;
  int group_index = 0;
  nlohmann::json clustering_audits = nlohmann::json::array();
  for (const auto& ssp : ssps) {
    for (const int year : years) {
      const auto climate = load_climate_morph_factors(ssp, year, warnings);
      std::vector<ScenarioCandidate> candidates;
      candidates.reserve(static_cast<std::size_t>(per_group_candidates));
      for (int i = 0; i < per_group_candidates; ++i) {
        std::array<double, 12> sampled_delta_t = climate.delta_t_c;
        std::array<double, 12> sampled_delta_ghi = climate.delta_ghi_w_m2;
        std::vector<std::vector<double>> climate_load_factors;
        std::vector<double> aggregate_climate_load_factor(
            static_cast<std::size_t>(steps), 1.0);
        if (profile_plan.per_component) {
          climate_load_factors.assign(
              load_sites.size(),
              std::vector<double>(static_cast<std::size_t>(steps), 1.0));
        }
        std::vector<double> climate_ghi_factor(static_cast<std::size_t>(steps), 1.0);
        if (perturbation.enable_climate_perturbation) {
          const std::string label = ssp + ":" + std::to_string(year);
          const unsigned int climate_seed = perturbation.climate_seed + stable_label_seed(label) * 1009U + static_cast<unsigned int>((i + 1) * 9176U);
          std::mt19937 climate_rng(climate_seed);
          const auto temp_noise = smooth_monthly_noise(std::max(0.0, perturbation.climate_temp_sigma_c), climate_rng);
          const auto ghi_noise_pct = smooth_monthly_noise(std::max(0.0, perturbation.climate_ghi_sigma_pct), climate_rng);
          for (int month = 0; month < 12; ++month) {
            sampled_delta_t[static_cast<std::size_t>(month)] += temp_noise[static_cast<std::size_t>(month)];
            sampled_delta_ghi[static_cast<std::size_t>(month)] *= 1.0 + ghi_noise_pct[static_cast<std::size_t>(month)] / 100.0;
          }
          climate_ghi_factor = smooth_block_percent_factors(steps, std::max(0.0, perturbation.climate_ghi_sigma_pct), perturbation.block_hours, climate_rng);
          if (profile_plan.per_component) {
            for (auto& factor : climate_load_factors) {
              factor = smooth_block_percent_factors(
                  steps, std::max(0.0, perturbation.climate_load_sigma_pct),
                  perturbation.block_hours, climate_rng);
            }
          } else {
            aggregate_climate_load_factor = smooth_block_percent_factors(
                steps, std::max(0.0, perturbation.climate_load_sigma_pct),
                perturbation.block_hours, climate_rng);
          }
        }
        auto regular_pv = morph_regular_pv(base_pv, base_temp, base_ghi, sampled_delta_t, sampled_delta_ghi, climate_ghi_factor);
        const unsigned int seed = perturbation.seed + static_cast<unsigned int>(group_index * 100003 + i * 7919 + 17);
        std::mt19937 rng(seed);
        const std::string candidate_id =
            "regular:" + ssp + ":" + std::to_string(year) + ":" +
            std::to_string(i + 1);
        ScenarioCandidate c;
        if (profile_plan.per_component) {
          auto regular_load_components = morph_regular_load_components(
              base_load_components, sampled_delta_t, climate_load_factors);
          auto realized = make_candidate_with_load_components(
              candidate_id, ScenarioFamily::Regular, load_sites,
              regular_load_components, regular_pv, base_wind, base_other,
              storage_baseline, perturbation, rng);
          c = std::move(realized.candidate);
        } else {
          auto regular_load = morph_regular_load_profile(
              base_load, sampled_delta_t, aggregate_climate_load_factor);
          c = make_candidate(candidate_id, ScenarioFamily::Regular, regular_load,
                             regular_pv, base_wind, base_other,
                             storage_baseline, perturbation, rng);
        }
        c.features["ssp_code"] = ssp == "ssp126" ? 126.0 : ssp == "ssp245" ? 245.0 : ssp == "ssp370" ? 370.0 : ssp == "ssp585" ? 585.0 : 0.0;
        c.features["year"] = static_cast<double>(year);
        c.features["regular_group"] = static_cast<double>(group_index);
        c.features["climate_annual_delta_T"] = average_monthly(sampled_delta_t);
        c.features["climate_annual_delta_GHI"] = average_monthly(sampled_delta_ghi);
        c.features["climate_data_found"] = climate.data_found ? 1.0 : 0.0;
        for (int month = 0; month < 12; ++month) {
          c.features["climate_month_delta_T_" + std::to_string(month + 1)] = sampled_delta_t[static_cast<std::size_t>(month)];
          c.features["climate_month_delta_GHI_" + std::to_string(month + 1)] = sampled_delta_ghi[static_cast<std::size_t>(month)];
        }
        c.probability = 1.0 / static_cast<double>(per_group_candidates);
        coverage_samples.push_back(candidate_coverage_point(c));
        candidates.push_back(std::move(c));
      }
      nlohmann::json cluster_audit = nlohmann::json::object();
      cluster_audit["group"] = ssp + ":" + std::to_string(year);
      cluster_audit["climate_data_found"] = climate.data_found;
      cluster_audit["climate_data_source"] = climate.source;
      cluster_audit["climate_data_from_csv"] = climate.from_csv;
      cluster_audit["climate_month_delta_T"] = climate.delta_t_c;
      cluster_audit["climate_month_delta_GHI"] = climate.delta_ghi_w_m2;
      cluster_audit["climate_annual_delta_T"] = average_monthly(climate.delta_t_c);
      cluster_audit["climate_annual_delta_GHI"] = average_monthly(climate.delta_ghi_w_m2);
      auto grouped = cluster_candidates(std::move(candidates), options.cluster_count, clustering, &cluster_audit);
      clustering_audits.push_back(cluster_audit);
      for (auto& cluster : grouped) {
        attach_standard_time_series(cluster.representative, renewable0, load0);
        cluster.cluster_id += cluster_offset;
        cluster.probability /= static_cast<double>(group_count);
        result.clusters.push_back(std::move(cluster));
      }
      cluster_offset += static_cast<int>(grouped.size());
      result.candidate_count += per_group_candidates;
      ++group_index;
    }
  }
  result.cluster_count = static_cast<int>(result.clusters.size());
  result.audit = {{"num_steps", steps},
                  {"grouping", "ssp_year"},
                  {"group_count", group_count},
                  {"requested_cluster_count_per_ssp_year", options.cluster_count},
                  {"expected_total_cluster_count", options.cluster_count * group_count},
                  {"ssp", options.ssp},
                  {"year", options.year},
                  {"ssp_levels", ssps},
                  {"years", years},
                  {"per_group_candidates", per_group_candidates},
                  {"clustering_method", clustering.method},
                  {"storage_soc_uncertainty_enabled", perturbation.enable_storage_soc_perturbation},
                  {"storage_count", storage_baseline.count()},
                  {"storage_capacity_mwh", storage_baseline.total_capacity_mwh},
                  {"storage_soc_sigma", perturbation.storage_soc_sigma},
                  {"storage_soc_min_multiplier", perturbation.storage_soc_min_multiplier},
                  {"storage_soc_max_multiplier", perturbation.storage_soc_max_multiplier},
                  {"load_processing_granularity", profile_plan.granularity()},
                  {"load_site_count", load_sites.size()},
                  {"component_profile_fallback", !profile_plan.per_component},
                  {"projected_component_points_per_candidate", profile_plan.points_per_candidate},
                  {"projected_component_candidate_work_points", profile_plan.candidate_work_points},
                  {"projected_component_representative_points", profile_plan.representative_points},
                  {"climate_morphing_enabled", true},
                  {"climate_formula", "tas_rsds_temperature_load_and_pv_reference_ratio"},
                  {"climate_perturbation_enabled", perturbation.enable_climate_perturbation},
                  {"climate_seed", perturbation.climate_seed},
                  {"climate_temp_sigma_c", perturbation.climate_temp_sigma_c},
                  {"climate_ghi_sigma_pct", perturbation.climate_ghi_sigma_pct},
                  {"climate_load_sigma_pct", perturbation.climate_load_sigma_pct},
                  {"clustering_audits", std::move(clustering_audits)},
                  {"coverage_samples", std::move(coverage_samples)}};
  return result;
}

ReliabilityScenarioResult generate_reliability_scenarios(const HybridPowerSystem& sys,
                                                         const ReliabilityScenarioOptions& options,
                                                         const PerturbationOptions& perturbation,
                                                         const ClusteringOptions& clustering,
                                                         std::vector<std::string>* warnings) {
  ReliabilityScenarioResult result;
  if (!options.enabled) return result;
  const auto contingencies = enumerate_n1_contingencies(sys, options);
  nlohmann::json coverage_samples = nlohmann::json::array();
  const auto load_sites = collect_load_sites(sys);
  const double load0 = sum_load_sites(load_sites);
  const auto renewable0 = renewable_breakdown_mw(sys);
  if (warnings && contingencies.empty()) warnings->push_back("No N-1 contingencies were enumerated for reliability scenarios.");
  const std::vector<double> base_pv{renewable0.pv_mw};
  const std::vector<double> base_wind{renewable0.wind_mw};
  const std::vector<double> base_other{renewable0.other_mw};
  const auto storage_baseline = storage_soc_baseline(sys);
  if (warnings && perturbation.enable_storage_soc_perturbation && storage_baseline.empty()) {
    warnings->push_back("Storage SOC perturbation enabled but no in-service storage with positive e_rated_mwh was found.");
  }
  const int requested_candidate_count =
      std::max(1, options.candidates_per_contingency);
  const int requested_cluster_count = std::clamp(
      options.cluster_count_per_contingency, 1, requested_candidate_count);
  const auto contingency_count =
      static_cast<std::uint64_t>(contingencies.size());
  const auto candidate_budget_per_contingency = contingency_count == 0
      ? static_cast<std::uint64_t>(requested_candidate_count)
      : std::max<std::uint64_t>(
            1, kMaxReliabilityCandidateCount / contingency_count);
  const int candidate_count = static_cast<int>(std::min<std::uint64_t>(
      static_cast<std::uint64_t>(requested_candidate_count),
      candidate_budget_per_contingency));
  const auto representative_budget_per_contingency = contingency_count == 0
      ? static_cast<std::uint64_t>(requested_cluster_count)
      : std::max<std::uint64_t>(
            1, kMaxReliabilityRepresentativeCount / contingency_count);
  const int cluster_count = static_cast<int>(std::min<std::uint64_t>(
      static_cast<std::uint64_t>(std::min(requested_cluster_count,
                                          candidate_count)),
      representative_budget_per_contingency));
  const bool candidate_budget_applied =
      candidate_count < requested_candidate_count;
  const bool representative_budget_applied =
      cluster_count < requested_cluster_count;
  if (warnings && (candidate_budget_applied || representative_budget_applied)) {
    warnings->push_back(
        "Reliability scenario generation preserved the complete N-1 catalog but "
        "reduced candidates from " +
        std::to_string(requested_candidate_count) + " to " +
        std::to_string(candidate_count) + " and representatives from " +
        std::to_string(requested_cluster_count) + " to " +
        std::to_string(cluster_count) +
        " per contingency to stay within the interactive workload budget.");
  }
  const auto profile_plan = component_profile_plan(
      load_sites.size(), 1,
      contingency_count *
          static_cast<std::uint64_t>(candidate_count),
      contingency_count * static_cast<std::uint64_t>(cluster_count));
  warn_component_profile_fallback("Reliability", profile_plan, warnings);
  const auto base_load_components = [&]() {
    std::vector<std::vector<double>> components;
    if (!profile_plan.per_component) return components;
    components.reserve(load_sites.size());
    for (const auto& site : load_sites) {
      components.push_back(std::vector<double>{site.base_mw});
    }
    return components;
  }();
  const std::vector<double> base_load{load0};
  const std::uint64_t total_coverage_candidates =
      static_cast<std::uint64_t>(contingencies.size()) *
      static_cast<std::uint64_t>(candidate_count);
  const std::uint64_t coverage_stride = std::max<std::uint64_t>(
      1, (total_coverage_candidates + kMaxCoverageSamples - 1) /
             kMaxCoverageSamples);
  std::uint64_t coverage_ordinal = 0;
  for (std::size_t ci = 0; ci < contingencies.size(); ++ci) {
    std::vector<ScenarioCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(candidate_count));
    for (int i = 0; i < candidate_count; ++i) {
      std::mt19937 rng(perturbation.seed + static_cast<unsigned int>(ci * 104729 + i * 1543 + 101));
      const std::string candidate_id =
          "reliability:" + contingencies[ci].id + ":" +
          std::to_string(i + 1);
      ScenarioCandidate c;
      if (profile_plan.per_component) {
        auto realized = make_candidate_with_load_components(
            candidate_id, ScenarioFamily::Reliability, load_sites,
            base_load_components, base_pv, base_wind, base_other,
            storage_baseline, perturbation, rng);
        c = std::move(realized.candidate);
      } else {
        c = make_candidate(candidate_id, ScenarioFamily::Reliability,
                           base_load, base_pv, base_wind, base_other,
                           storage_baseline, perturbation, rng);
      }
      c.probability = 1.0 / static_cast<double>(candidate_count);
      c.contingency = contingencies[ci];
      c.features["contingency_component_index"] = static_cast<double>(contingencies[ci].component_index);
      c.features["contingency_ordinal"] = static_cast<double>(ci + 1);
      if (coverage_ordinal % coverage_stride == 0) {
        coverage_samples.push_back(candidate_coverage_point(c));
      }
      ++coverage_ordinal;
      candidates.push_back(std::move(c));
    }
    ReliabilityContingencyScenarioGroup group;
    group.contingency = contingencies[ci];
    group.candidate_count = candidate_count;
    group.audit = {{"num_steps", 1},
                   {"requested_candidate_count", requested_candidate_count},
                   {"effective_candidate_count", candidate_count},
                   {"requested_cluster_count", requested_cluster_count},
                   {"effective_cluster_count", cluster_count},
                   {"load_processing_granularity", profile_plan.granularity()},
                   {"load_site_count", load_sites.size()}};
    group.clusters = cluster_candidates(
        std::move(candidates), cluster_count, clustering, &group.audit);
    for (auto& cluster : group.clusters) {
      attach_standard_time_series(cluster.representative, renewable0, load0);
    }
    group.cluster_count = static_cast<int>(group.clusters.size());
    result.cluster_total += group.cluster_count;
    result.contingencies.push_back(std::move(group));
  }
  result.contingency_count = static_cast<int>(result.contingencies.size());
  result.audit = {{"num_steps", 1},
                  {"requested_candidates_per_contingency", requested_candidate_count},
                  {"candidates_per_contingency", candidate_count},
                  {"requested_clusters_per_contingency", requested_cluster_count},
                  {"clusters_per_contingency", cluster_count},
                  {"candidate_budget", kMaxReliabilityCandidateCount},
                  {"representative_budget", kMaxReliabilityRepresentativeCount},
                  {"candidate_budget_applied", candidate_budget_applied},
                  {"representative_budget_applied", representative_budget_applied},
                  {"requested_candidate_total",
                   contingency_count *
                       static_cast<std::uint64_t>(requested_candidate_count)},
                  {"effective_candidate_total", total_coverage_candidates},
                  {"requested_representative_total",
                   contingency_count *
                       static_cast<std::uint64_t>(requested_cluster_count)},
                  {"effective_representative_total",
                   contingency_count * static_cast<std::uint64_t>(cluster_count)},
                  {"component_catalog", "reliability_fmea"},
                  {"component_types", reliability_fmea_component_types_json()},
                  {"load_processing_granularity", profile_plan.granularity()},
                  {"load_site_count", load_sites.size()},
                  {"component_profile_fallback", !profile_plan.per_component},
                  {"projected_component_points_per_candidate", profile_plan.points_per_candidate},
                  {"projected_component_candidate_work_points", profile_plan.candidate_work_points},
                  {"projected_component_representative_points", profile_plan.representative_points},
                  {"coverage_candidate_count", total_coverage_candidates},
                  {"coverage_sample_stride", coverage_stride},
                  {"coverage_samples_truncated", coverage_stride > 1},
                  {"storage_soc_uncertainty_enabled", perturbation.enable_storage_soc_perturbation},
                  {"storage_count", storage_baseline.count()},
                  {"storage_capacity_mwh", storage_baseline.total_capacity_mwh},
                  {"storage_soc_sigma", perturbation.storage_soc_sigma},
                  {"storage_soc_min_multiplier", perturbation.storage_soc_min_multiplier},
                  {"storage_soc_max_multiplier", perturbation.storage_soc_max_multiplier},
                  {"coverage_samples", std::move(coverage_samples)}};
  return result;
}

ResilienceScenarioResult generate_resilience_scenarios(const HybridPowerSystem& sys,
                                                       const ResilienceScenarioOptions& options,
                                                       const PerturbationOptions& perturbation,
                                                       const TyphoonImpactOptions& typhoon_impact,
                                                       const ClusteringOptions& clustering,
                                                       std::vector<std::string>* warnings) {
  ResilienceScenarioResult result;
  if (!options.enabled) return result;
  const int steps = std::max(1, options.num_steps);
  const auto load_sites = collect_load_sites(sys);
  const double load0 = sum_load_sites(load_sites);
  const auto renewable0 = renewable_breakdown_mw(sys);
  const auto wind_sites = wind_resource_sites(sys);
  const auto base_pv = synthesize_base_pv(renewable0.pv_mw, steps);
  const auto base_wind = synthesize_base_wind(renewable0.wind_mw, steps);
  const auto base_other = synthesize_base_other_renewable(renewable0.other_mw, steps);
  const auto storage_baseline = storage_soc_baseline(sys);
  if (warnings && perturbation.enable_storage_soc_perturbation && storage_baseline.empty()) {
    warnings->push_back("Storage SOC perturbation enabled but no in-service storage with positive e_rated_mwh was found.");
  }
  const int requested_candidate_count =
      std::max(1, options.candidates_per_intensity);
  const std::uint64_t intensity_count =
      static_cast<std::uint64_t>(options.intensity_levels.size());
  const std::uint64_t active_branch_count =
      static_cast<std::uint64_t>(std::count_if(
          sys.ac.branches.begin(), sys.ac.branches.end(),
          [](const auto& branch) { return branch.in_service; })) +
      static_cast<std::uint64_t>(std::count_if(
          sys.dc.branches.begin(), sys.dc.branches.end(),
          [](const auto& branch) { return branch.in_service; }));
  int minimum_candidates_per_intensity = 1;
  for (const auto intensity : options.intensity_levels) {
    const auto it = options.cluster_count_by_intensity.find(intensity);
    const int requested_clusters = it == options.cluster_count_by_intensity.end()
        ? options.default_cluster_count
        : it->second;
    minimum_candidates_per_intensity = std::max(
        minimum_candidates_per_intensity,
        std::clamp(requested_clusters, 1, requested_candidate_count));
  }
  const std::uint64_t spatial_sites_per_candidate =
      active_branch_count + static_cast<std::uint64_t>(load_sites.size());
  const std::uint64_t minimum_spatial_work_per_intensity =
      saturating_product(
          {spatial_sites_per_candidate,
           static_cast<std::uint64_t>(steps),
           std::max<std::uint64_t>(1, intensity_count)});
  const int budgeted_candidates_per_intensity =
      minimum_spatial_work_per_intensity == 0
          ? requested_candidate_count
          : static_cast<int>(std::max<std::uint64_t>(
                1, kMaxResilienceSpatialEvaluations /
                       minimum_spatial_work_per_intensity));
  const int candidate_count = std::min(
      requested_candidate_count,
      std::max(minimum_candidates_per_intensity,
               budgeted_candidates_per_intensity));
  const bool candidate_budget_applied =
      candidate_count < requested_candidate_count;
  std::uint64_t expected_representatives = 0;
  for (const auto intensity : options.intensity_levels) {
    const auto it = options.cluster_count_by_intensity.find(intensity);
    const int requested_clusters = it == options.cluster_count_by_intensity.end()
        ? options.default_cluster_count
        : it->second;
    expected_representatives += static_cast<std::uint64_t>(
        std::clamp(requested_clusters, 1, candidate_count));
  }
  const std::uint64_t effective_candidate_total =
      static_cast<std::uint64_t>(candidate_count) * intensity_count;
  const auto profile_plan = component_profile_plan(
      load_sites.size(), steps,
      effective_candidate_total,
      expected_representatives);
  warn_component_profile_fallback("Resilience", profile_plan, warnings);
  if (warnings && candidate_budget_applied) {
    warnings->push_back(
        "Resilience scenario generation reduced candidates from " +
        std::to_string(requested_candidate_count) + " to " +
        std::to_string(candidate_count) +
        " per intensity while preserving the requested representative count "
        "because the topology exceeds the interactive spatial-work budget.");
  }
  const auto base_load_components = profile_plan.per_component
      ? synthesize_base_load_profiles(load_sites, steps)
      : std::vector<std::vector<double>>{};
  const auto base_load = synthesize_base_load(load0, steps);

  const std::uint64_t candidate_time_points = saturating_product(
      {effective_candidate_total, static_cast<std::uint64_t>(steps)});
  const std::uint64_t load_spatial_evaluations = saturating_product(
      {static_cast<std::uint64_t>(load_sites.size()), candidate_time_points});
  const std::uint64_t available_segment_evaluations =
      load_spatial_evaluations >= kMaxResilienceSpatialEvaluations
          ? 0
          : kMaxResilienceSpatialEvaluations - load_spatial_evaluations;
  const std::uint64_t segment_count_budget = candidate_time_points == 0
      ? active_branch_count * 40
      : std::max<std::uint64_t>(
            active_branch_count,
            available_segment_evaluations / candidate_time_points);
  const int max_segments_per_branch = active_branch_count == 0
      ? 40
      : static_cast<int>(std::clamp<std::uint64_t>(
            segment_count_budget / active_branch_count, 1, 40));
  TyphoonScenarioOptions geometry_options;
  geometry_options.horizon_hours = steps;
  geometry_options.time_step_hr = 1.0;
  geometry_options.max_segments_per_branch = max_segments_per_branch;
  bool geometry_uses_fallback_coordinates = false;
  const auto precomputed_segments = generate_typhoon_line_segments(
      sys, geometry_options, &geometry_uses_fallback_coordinates);
  const bool segment_budget_applied = max_segments_per_branch < 40;
  const std::uint64_t effective_spatial_evaluations = saturating_product(
      {static_cast<std::uint64_t>(precomputed_segments.size()) +
           static_cast<std::uint64_t>(load_sites.size()),
       candidate_time_points});
  if (warnings && segment_budget_applied) {
    warnings->push_back(
        "Resilience hazard evaluation reused one precomputed topology and "
        "limited line segmentation to " +
        std::to_string(max_segments_per_branch) +
        " segment(s) per branch to bound first-run latency.");
  }

  TyphoonCatalogOptions catalog_opts;
  catalog_opts.samples_per_month = std::max(120, candidate_count * 2);
  catalog_opts.first_month = 1;
  catalog_opts.last_month = 12;
  catalog_opts.base_seed = perturbation.seed + 203000U;
  catalog_opts.horizon_hours = steps;
  catalog_opts.time_step_hr = 1.0;
  catalog_opts.stochastic = true;
  catalog_opts.use_month_defaults = true;
  catalog_opts.use_sst_resource = true;
  const auto catalog = get_or_build_typhoon_catalog(catalog_opts);
  const auto catalog_counts = typhoon_catalog_counts_to_json(*catalog);
  nlohmann::json coverage_samples = nlohmann::json::array();
  std::uint64_t approximate_repair_candidate_count = 0;
  std::unordered_map<int, std::size_t> ac_branch_pos;
  ac_branch_pos.reserve(sys.ac.branches.size());
  for (std::size_t bi = 0; bi < sys.ac.branches.size(); ++bi) {
    ac_branch_pos[sys.ac.branches[bi].index] = bi;
  }
  std::unordered_map<int, std::size_t> dc_branch_pos;
  dc_branch_pos.reserve(sys.dc.branches.size());
  for (std::size_t bi = 0; bi < sys.dc.branches.size(); ++bi) {
    dc_branch_pos[sys.dc.branches[bi].index] =
        sys.ac.branches.size() + bi;
  }

  for (auto intensity : options.intensity_levels) {
    const auto category_it = catalog->by_category.find(intensity);
    if (category_it == catalog->by_category.end() || category_it->second.empty()) {
      if (warnings) warnings->push_back("No typhoon catalog samples classified as " + std::string(to_string(intensity)) + "; resilience scenario group skipped.");
      continue;
    }
    const auto& category_indices = category_it->second;
    const int selected_count = std::min(candidate_count, static_cast<int>(category_indices.size()));
    if (warnings && selected_count < candidate_count) {
      warnings->push_back("Only " + std::to_string(selected_count) + " typhoon catalog samples classified as " +
                          std::string(to_string(intensity)) + "; clustering uses all available samples without cross-category fallback.");
    }
    const std::size_t start = category_indices.empty() ? 0U :
        (static_cast<std::size_t>(perturbation.seed + typhoon_category_ordinal(intensity) * 7919U) % category_indices.size());
    std::vector<ScenarioCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(selected_count));
    for (int i = 0; i < selected_count; ++i) {
      const unsigned int seed = perturbation.seed + static_cast<unsigned int>(typhoon_category_ordinal(intensity) * 100003 + i * 4099 + 7001);
      const auto& sample = catalog->samples[category_indices[(start + static_cast<std::size_t>(i)) % category_indices.size()]];
      std::mt19937 rng(seed);
      const std::string candidate_id =
          "resilience:" + std::string(to_string(intensity)) + ":" +
          std::to_string(i + 1);
      ScenarioCandidate c;
      std::vector<std::vector<double>> load_components;
      if (profile_plan.per_component) {
        auto realized = make_candidate_with_load_components(
            candidate_id, ScenarioFamily::Resilience, load_sites,
            base_load_components, base_pv, base_wind, base_other,
            storage_baseline, perturbation, rng);
        c = std::move(realized.candidate);
        load_components = std::move(realized.load_components_after_perturbation);
      } else {
        c = make_candidate(candidate_id, ScenarioFamily::Resilience,
                           base_load, base_pv, base_wind, base_other,
                           storage_baseline, perturbation, rng);
      }
      TyphoonScenarioOptions ty;
      ty.horizon_hours = steps;
      ty.time_step_hr = 1.0;
      ty.seed = sample.seed;
      ty.stochastic = sample.stochastic;
      ty.month = sample.month;
      ty.use_month_defaults = true;
      ty.use_sst_resource = true;
      ty.use_precomputed_track = true;
      ty.precomputed_track = sample.track;
      ty.requested_category = intensity;
      ty.selected_category = sample.category;
      ty.selected_track_max_vmax_ms = sample.max_vmax_ms;
      ty.selected_sample_id = sample.sample_id;
      ty.used_category_fallback = false;
      ty.apply_pv_wind_derating = true;
      const auto generated = generate_typhoon_fault_sequence(
          sys, ty, precomputed_segments,
          geometry_uses_fallback_coordinates);
      if (generated.selected_category != intensity) {
        if (warnings) warnings->push_back("Skipped typhoon sample " + generated.selected_sample_id + " because selected category " +
                                          std::string(to_string(generated.selected_category)) + " did not match requested category " +
                                          std::string(to_string(intensity)) + ".");
        continue;
      }

      ResilienceEventDefinition event;
      event.id = c.id + ":event";
      event.requested_intensity = intensity;
      event.selected_intensity = generated.selected_category;
      event.selected_track_max_vmax_ms = generated.selected_track_max_vmax_ms;
      event.selected_sample_id = generated.selected_sample_id;
      event.used_catalog_sample = generated.used_catalog_sample;
      event.used_category_fallback = generated.used_category_fallback;
      event.used_approximate_repair_order =
          generated.used_approximate_repair_order;
      event.repair_fault_count = generated.repair_fault_count;
      if (generated.used_approximate_repair_order) {
        ++approximate_repair_candidate_count;
      }
      event.month = sample.month;
      event.faults = generated.faults;
      event.track = generated.track;
      event.branch_risks = generated.branch_risks;
      event.stage1_start_step = first_fault_step(generated.faults);
      event.stage1_end_step = last_new_fault_step(generated.faults);
      event.pv_typhoon_multiplier = pv_transition_profile(steps, event.stage1_start_step, event.stage1_end_step,
                                                          typhoon_impact.pv_transition_hours);
      auto pv_wind_derating = generated.renewable_profile_multiplier;
      if (pv_wind_derating.empty()) pv_wind_derating.assign(static_cast<std::size_t>(steps), 1.0);
      if (static_cast<int>(pv_wind_derating.size()) < steps) pv_wind_derating.resize(static_cast<std::size_t>(steps), pv_wind_derating.back());
      const auto bus_wind = aggregate_bus_wind(sys, generated.track, steps);
      const auto wind_from_sites = wind_generation_from_track(wind_sites, generated.track, steps, typhoon_impact);
      event.load_typhoon_multiplier.resize(static_cast<std::size_t>(steps), 1.0);
      event.wind_typhoon_multiplier.resize(static_cast<std::size_t>(steps), 1.0);
      event.renewable_typhoon_multiplier.resize(static_cast<std::size_t>(steps), 1.0);

      std::vector<double> load_before_typhoon;
      std::vector<double> load;
      if (profile_plan.per_component) {
        std::vector<std::vector<double>> typhoon_load_components = load_components;
        load_before_typhoon = sum_profiles(load_components, steps);
        for (std::size_t site_idx = 0; site_idx < typhoon_load_components.size(); ++site_idx) {
          const auto wind_profile = site_idx < load_sites.size()
              ? site_wind_profile(load_sites[site_idx], generated.track, steps)
              : bus_wind;
          for (int t = 0; t < steps &&
                          t < static_cast<int>(typhoon_load_components[site_idx].size());
               ++t) {
            typhoon_load_components[site_idx][static_cast<std::size_t>(t)] *=
                1.0 - load_reduction(
                          wind_profile[static_cast<std::size_t>(t)],
                          typhoon_impact);
          }
        }
        load = sum_profiles(typhoon_load_components, steps);
        c.component_load_profiles =
            component_load_profiles_to_json(load_sites, typhoon_load_components);
      } else {
        load_before_typhoon = profile_values(c.time_series, "total_load_mw");
        load = load_before_typhoon;
        std::vector<double> aggregate_multiplier(
            static_cast<std::size_t>(steps), 1.0);
        if (load0 > 1e-9) {
          for (const auto& site : load_sites) {
            const double weight = std::max(0.0, site.base_mw) / load0;
            const auto wind_profile =
                site_wind_profile(site, generated.track, steps);
            for (int t = 0; t < steps; ++t) {
              aggregate_multiplier[static_cast<std::size_t>(t)] -=
                  weight * load_reduction(
                               series_value(wind_profile, t), typhoon_impact);
            }
          }
        }
        for (int t = 0; t < steps; ++t) {
          load[static_cast<std::size_t>(t)] *= std::clamp(
              aggregate_multiplier[static_cast<std::size_t>(t)], 0.0, 1.0);
        }
      }
      for (int t = 0; t < steps; ++t) {
        const double before = series_value(load_before_typhoon, t);
        event.load_typhoon_multiplier[static_cast<std::size_t>(t)] = before > 1e-9 ? series_value(load, t) / before : 1.0;
      }

      auto pv = profile_values(c.time_series, "pv_mw");
      auto wind = profile_values(c.time_series, "wind_mw");
      auto other = profile_values(c.time_series, "other_renewable_mw");
      for (int t = 0; t < steps; ++t) {
        const double pv0 = series_value(pv, t);
        const double wind0 = series_value(wind, t);
        const double other0 = series_value(other, t);
        const double renewable0_at_t = pv0 + wind0 + other0;
        pv[static_cast<std::size_t>(t)] = pv0 * event.pv_typhoon_multiplier[static_cast<std::size_t>(t)] *
                                          pv_wind_derating[static_cast<std::size_t>(t)];
        if (wind0 > 1e-9) {
          wind[static_cast<std::size_t>(t)] = wind_sites.empty()
              ? wind0 * wind_power_curve(bus_wind[static_cast<std::size_t>(t)], typhoon_impact)
              : series_value(wind_from_sites, t);
          event.wind_typhoon_multiplier[static_cast<std::size_t>(t)] = wind[static_cast<std::size_t>(t)] / wind0;
        } else {
          wind[static_cast<std::size_t>(t)] = 0.0;
        }
        const double renewable_after = pv[static_cast<std::size_t>(t)] + wind[static_cast<std::size_t>(t)] + other0;
        event.renewable_typhoon_multiplier[static_cast<std::size_t>(t)] = renewable0_at_t > 1e-9 ? renewable_after / renewable0_at_t : 1.0;
      }
      const auto storage = storage_profiles_from_time_series(c.time_series, c);
      c.time_series = make_time_series(load, pv, wind, other, &storage);
      c.features = feature_summary(load, pv, wind, other, &storage);
      c.features["fault_count"] = static_cast<double>(generated.faults.size());
      c.features["approximate_repair_order"] =
          generated.used_approximate_repair_order ? 1.0 : 0.0;
      c.features["peak_failure_probability"] = 0.0;
      c.features["selected_track_max_vmax_ms"] = generated.selected_track_max_vmax_ms;
      c.features["typhoon_month"] = static_cast<double>(sample.month);
      c.features["net_load_mw"] = c.features["net_load_sum"];
      for (const auto& risk : generated.branch_risks) c.features["peak_failure_probability"] = std::max(c.features["peak_failure_probability"], risk.peak_failure_probability);
      c.outage_signature.assign(static_cast<std::size_t>(sys.ac.branches.size() + sys.dc.branches.size()), 0);
      for (const auto& f : generated.faults) {
        const auto& pos_map = f.branch_kind == ResilienceBranchKind::AC ? ac_branch_pos : dc_branch_pos;
        auto it = pos_map.find(f.branch_index);
        if (it != pos_map.end() && it->second < c.outage_signature.size()) {
          c.outage_signature[it->second] = 1;
        }
      }
      c.resilience_event = std::move(event);
      c.probability = 1.0 / static_cast<double>(candidate_count);
      coverage_samples.push_back(candidate_coverage_point(c, true));
      candidates.push_back(std::move(c));
    }
    if (candidates.empty()) continue;
    for (auto& c : candidates) c.probability = 1.0 / static_cast<double>(candidates.size());
    ResilienceIntensityScenarioGroup group;
    group.intensity = intensity;
    group.candidate_count = static_cast<int>(candidates.size());
    const auto k_it = options.cluster_count_by_intensity.find(intensity);
    const int requested_k = k_it == options.cluster_count_by_intensity.end()
        ? options.default_cluster_count
        : k_it->second;
    const int k = std::clamp(
        requested_k, 1, static_cast<int>(candidates.size()));
    group.audit = {{"num_steps", steps},
                   {"requested_candidate_count", requested_candidate_count},
                   {"effective_candidate_count", group.candidate_count},
                   {"requested_cluster_count", requested_k},
                   {"effective_cluster_count", k},
                   {"load_processing_granularity", profile_plan.granularity()},
                   {"load_site_count", load_sites.size()},
                   {"catalog_first_month", catalog_opts.first_month},
                   {"catalog_last_month", catalog_opts.last_month},
                   {"catalog_sample_count", catalog->samples.size()},
                   {"catalog_counts", catalog_counts},
                   {"classification", "max_vmax_ms_skip_first_point"},
                   {"strict_category_sampling", true}};
    group.clusters = cluster_candidates(std::move(candidates), k, clustering, &group.audit);
    for (auto& cluster : group.clusters) {
      attach_standard_time_series(cluster.representative, renewable0, load0);
    }
    group.cluster_count = static_cast<int>(group.clusters.size());
    result.cluster_total += group.cluster_count;
    result.intensities.push_back(std::move(group));
  }
  result.intensity_count = static_cast<int>(result.intensities.size());
  result.audit = {{"num_steps", steps},
                  {"requested_candidates_per_intensity", requested_candidate_count},
                  {"candidates_per_intensity", candidate_count},
                  {"candidate_budget_applied", candidate_budget_applied},
                  {"spatial_evaluation_budget", kMaxResilienceSpatialEvaluations},
                  {"minimum_projected_requested_spatial_evaluations",
                   saturating_product(
                       {spatial_sites_per_candidate,
                        static_cast<std::uint64_t>(steps),
                        static_cast<std::uint64_t>(requested_candidate_count),
                        intensity_count})},
                  {"effective_spatial_evaluations",
                   effective_spatial_evaluations},
                  {"precomputed_topology_segments",
                   precomputed_segments.size()},
                  {"max_segments_per_branch", max_segments_per_branch},
                  {"segment_budget_applied", segment_budget_applied},
                  {"geometry_uses_fallback_coordinates",
                   geometry_uses_fallback_coordinates},
                  {"approximate_repair_candidate_count",
                   approximate_repair_candidate_count},
                  {"catalog_first_month", catalog_opts.first_month},
                  {"catalog_last_month", catalog_opts.last_month},
                  {"catalog_sample_count", catalog->samples.size()},
                  {"catalog_counts", catalog_counts},
                  {"classification", "max_vmax_ms_skip_first_point"},
                  {"strict_category_sampling", true},
                  {"load_processing_granularity", profile_plan.granularity()},
                  {"load_site_count", load_sites.size()},
                  {"component_profile_fallback", !profile_plan.per_component},
                  {"projected_component_points_per_candidate", profile_plan.points_per_candidate},
                  {"projected_component_candidate_work_points", profile_plan.candidate_work_points},
                  {"projected_component_representative_points", profile_plan.representative_points},
                  {"storage_soc_uncertainty_enabled", perturbation.enable_storage_soc_perturbation},
                  {"storage_count", storage_baseline.count()},
                  {"storage_capacity_mwh", storage_baseline.total_capacity_mwh},
                  {"storage_soc_sigma", perturbation.storage_soc_sigma},
                  {"storage_soc_min_multiplier", perturbation.storage_soc_min_multiplier},
                  {"storage_soc_max_multiplier", perturbation.storage_soc_max_multiplier},
                  {"coverage_samples", std::move(coverage_samples)}};
  if (warnings && sys.ac.buses.empty() && sys.dc.buses.empty()) warnings->push_back("No buses found for typhoon load impact location mapping.");
  return result;
}

ScenarioGenerationResult generate_scenarios(const HybridPowerSystem& sys, const ScenarioGenerationOptions& options) {
  ScenarioGenerationResult result;
  if (!options.regular.enabled && !options.reliability.enabled && !options.resilience.enabled) {
    throw std::invalid_argument("At least one scenario family must be enabled");
  }
  if (options.regular.enabled) {
    result.regular = generate_regular_scenarios(sys, options.regular, options.perturbation, options.clustering, &result.warnings);
  }
  if (options.reliability.enabled) {
    result.reliability = generate_reliability_scenarios(sys, options.reliability, options.perturbation, options.clustering, &result.warnings);
  }
  if (options.resilience.enabled) {
    result.resilience = generate_resilience_scenarios(sys, options.resilience, options.perturbation, options.typhoon_impact, options.clustering, &result.warnings);
  }
  result.summary = {{"regular_candidate_count", result.regular.candidate_count},
                    {"regular_cluster_count", result.regular.cluster_count},
                    {"reliability_contingency_count", result.reliability.contingency_count},
                    {"reliability_cluster_total", result.reliability.cluster_total},
                    {"resilience_intensity_count", result.resilience.intensity_count},
                    {"resilience_cluster_total", result.resilience.cluster_total}};
  return result;
}

ScenarioGenerationOptions scenario_generation_options_from_json(const nlohmann::json& j) {
  ScenarioGenerationOptions opt;
  if (j.contains("regular")) {
    const auto& r = j.at("regular");
    opt.regular.enabled = r.value("enabled", opt.regular.enabled);
    opt.regular.candidate_count = r.value("candidate_count", opt.regular.candidate_count);
    opt.regular.cluster_count = r.value("cluster_count", opt.regular.cluster_count);
    opt.regular.num_steps = r.value("num_steps", opt.regular.num_steps);
    opt.regular.ssp = r.value("ssp", opt.regular.ssp);
    opt.regular.year = r.value("year", opt.regular.year);
    if (r.contains("ssp_levels") && r.at("ssp_levels").is_array()) {
      opt.regular.ssp_levels.clear();
      for (const auto& item : r.at("ssp_levels")) opt.regular.ssp_levels.push_back(item.get<std::string>());
    }
    if (r.contains("years") && r.at("years").is_array()) {
      opt.regular.years.clear();
      for (const auto& item : r.at("years")) opt.regular.years.push_back(item.get<int>());
    }
  }
  if (j.contains("reliability")) {
    const auto& r = j.at("reliability");
    opt.reliability.enabled = r.value("enabled", opt.reliability.enabled);
    opt.reliability.candidates_per_contingency = r.value("candidates_per_contingency", opt.reliability.candidates_per_contingency);
    opt.reliability.cluster_count_per_contingency = r.value("cluster_count_per_contingency", opt.reliability.cluster_count_per_contingency);
    opt.reliability.num_steps = r.value("num_steps", 1);
    opt.reliability.include_ac_branches = r.value("include_ac_branches", opt.reliability.include_ac_branches);
    opt.reliability.include_dc_branches = r.value("include_dc_branches", opt.reliability.include_dc_branches);
    opt.reliability.include_vsc_converters = r.value("include_vsc_converters", opt.reliability.include_vsc_converters);
    opt.reliability.include_dcdc_converters = r.value("include_dcdc_converters", opt.reliability.include_dcdc_converters);
    opt.reliability.include_ac_buses = r.value("include_ac_buses", opt.reliability.include_ac_buses);
    opt.reliability.include_dc_buses = r.value("include_dc_buses", opt.reliability.include_dc_buses);
    opt.reliability.include_generators = r.value("include_generators", opt.reliability.include_generators);
    opt.reliability.include_loads = r.value("include_loads", opt.reliability.include_loads);
    opt.reliability.include_storage = r.value("include_storage", opt.reliability.include_storage);
    opt.reliability.include_vpps = r.value("include_vpps", opt.reliability.include_vpps);
    opt.reliability.include_microgrids = r.value("include_microgrids", opt.reliability.include_microgrids);
    opt.reliability.max_contingencies = r.value("max_contingencies", opt.reliability.max_contingencies);
  }
  if (j.contains("resilience")) {
    const auto& r = j.at("resilience");
    opt.resilience.enabled = r.value("enabled", opt.resilience.enabled);
    opt.resilience.candidates_per_intensity = r.value("candidates_per_intensity", opt.resilience.candidates_per_intensity);
    opt.resilience.default_cluster_count = r.value("default_cluster_count", r.value("cluster_count", opt.resilience.default_cluster_count));
    opt.resilience.num_steps = r.value("num_steps", opt.resilience.num_steps);
    opt.resilience.month = r.value("month", opt.resilience.month);
    if (r.contains("intensity_levels") && r.at("intensity_levels").is_array()) {
      opt.resilience.intensity_levels.clear();
      for (const auto& item : r.at("intensity_levels")) opt.resilience.intensity_levels.push_back(typhoon_intensity_category_from_string(item.get<std::string>()));
    }
    if (r.contains("cluster_count_by_intensity") && r.at("cluster_count_by_intensity").is_object()) {
      for (auto it = r.at("cluster_count_by_intensity").begin(); it != r.at("cluster_count_by_intensity").end(); ++it) {
        opt.resilience.cluster_count_by_intensity[typhoon_intensity_category_from_string(it.key())] = it.value().get<int>();
      }
    }
  }
  if (j.contains("perturbation")) {
    const auto& p = j.at("perturbation");
    opt.perturbation.enable_load_perturbation = p.value("enable_load_perturbation", opt.perturbation.enable_load_perturbation);
    opt.perturbation.enable_renewable_perturbation = p.value("enable_renewable_perturbation", opt.perturbation.enable_renewable_perturbation);
    opt.perturbation.seed = p.value("seed", opt.perturbation.seed);
    opt.perturbation.load_sigma = p.value("load_sigma", opt.perturbation.load_sigma);
    opt.perturbation.renewable_sigma = p.value("renewable_sigma", opt.perturbation.renewable_sigma);
    opt.perturbation.load_min_multiplier = p.value("load_min_multiplier", opt.perturbation.load_min_multiplier);
    opt.perturbation.load_max_multiplier = p.value("load_max_multiplier", opt.perturbation.load_max_multiplier);
    opt.perturbation.renewable_min_multiplier = p.value("renewable_min_multiplier", opt.perturbation.renewable_min_multiplier);
    opt.perturbation.renewable_max_multiplier = p.value("renewable_max_multiplier", opt.perturbation.renewable_max_multiplier);
    opt.perturbation.enable_storage_soc_perturbation = p.value("enable_storage_soc_perturbation", opt.perturbation.enable_storage_soc_perturbation);
    opt.perturbation.storage_soc_sigma = std::max(0.0, p.value("storage_soc_sigma", opt.perturbation.storage_soc_sigma));
    opt.perturbation.storage_soc_min_multiplier = std::max(0.0, p.value("storage_soc_min_multiplier", opt.perturbation.storage_soc_min_multiplier));
    opt.perturbation.storage_soc_max_multiplier = std::max(0.0, p.value("storage_soc_max_multiplier", opt.perturbation.storage_soc_max_multiplier));
    if (opt.perturbation.storage_soc_max_multiplier < opt.perturbation.storage_soc_min_multiplier) {
      std::swap(opt.perturbation.storage_soc_min_multiplier, opt.perturbation.storage_soc_max_multiplier);
    }
    opt.perturbation.enable_climate_perturbation = p.value("enable_climate_perturbation", opt.perturbation.enable_climate_perturbation);
    opt.perturbation.climate_seed = p.value("climate_seed", opt.perturbation.climate_seed);
    opt.perturbation.climate_temp_sigma_c = std::max(0.0, p.value("climate_temp_sigma_c", opt.perturbation.climate_temp_sigma_c));
    opt.perturbation.climate_ghi_sigma_pct = std::max(0.0, p.value("climate_ghi_sigma_pct", opt.perturbation.climate_ghi_sigma_pct));
    opt.perturbation.climate_load_sigma_pct = std::max(0.0, p.value("climate_load_sigma_pct", opt.perturbation.climate_load_sigma_pct));
    opt.perturbation.block_hours = p.value("block_hours", opt.perturbation.block_hours);
  }
  if (j.contains("typhoon_impact")) {
    const auto& t = j.at("typhoon_impact");
    opt.typhoon_impact.pv_transition_hours = t.value("pv_transition_hours", opt.typhoon_impact.pv_transition_hours);
    opt.typhoon_impact.load_wind_start = t.value("load_wind_start", opt.typhoon_impact.load_wind_start);
    opt.typhoon_impact.load_wind_full = t.value("load_wind_full", opt.typhoon_impact.load_wind_full);
    opt.typhoon_impact.load_max_reduction = t.value("load_max_reduction", opt.typhoon_impact.load_max_reduction);
    opt.typhoon_impact.wind_cut_in_ms = t.value("wind_cut_in_ms", opt.typhoon_impact.wind_cut_in_ms);
    opt.typhoon_impact.wind_rated_ms = t.value("wind_rated_ms", opt.typhoon_impact.wind_rated_ms);
    opt.typhoon_impact.wind_cut_out_ms = t.value("wind_cut_out_ms", opt.typhoon_impact.wind_cut_out_ms);
    opt.typhoon_impact.wind_ramp_exponent = t.value("wind_ramp_exponent", opt.typhoon_impact.wind_ramp_exponent);
  }
  if (j.contains("clustering")) {
    const auto& c = j.at("clustering");
    opt.clustering.method = c.value("method", opt.clustering.method);
    opt.clustering.max_iterations = c.value("max_iterations", opt.clustering.max_iterations);
    opt.clustering.seed = c.value("seed", opt.clustering.seed);
    opt.clustering.continuous_weight = std::max(0.0, c.value("continuous_weight", opt.clustering.continuous_weight));
    opt.clustering.outage_hamming_weight = std::max(0.0, c.value("outage_hamming_weight", opt.clustering.outage_hamming_weight));
    opt.clustering.robust_scale = c.value("robust_scale", opt.clustering.robust_scale);
    opt.clustering.include_tail_anchors = c.value("include_tail_anchors", opt.clustering.include_tail_anchors);
    opt.clustering.regime_weight = std::max(0.0, c.value("regime_weight", opt.clustering.regime_weight));
    opt.clustering.tail_fraction = clamp_value(c.value("tail_fraction", opt.clustering.tail_fraction), 0.0, 0.5);
    opt.clustering.source_tail_quantile = clamp_value(c.value("source_tail_quantile", opt.clustering.source_tail_quantile), 0.5, 0.999);
    opt.clustering.coupling_quantile = clamp_value(c.value("coupling_quantile", opt.clustering.coupling_quantile), 0.5, 0.999);
    opt.clustering.boundary_fraction = clamp_value(c.value("boundary_fraction", opt.clustering.boundary_fraction), 0.0, 0.5);
    opt.clustering.freeze_anchors = c.value("freeze_anchors", opt.clustering.freeze_anchors);
    opt.clustering.compare_baseline = c.value("compare_baseline", opt.clustering.compare_baseline);
  }
  opt.reliability.num_steps = 1;
  return opt;
}

nlohmann::json scenario_generation_result_to_json(const ScenarioGenerationResult& result) {
  nlohmann::json regular_clusters = nlohmann::json::array();
  for (const auto& c : result.regular.clusters) regular_clusters.push_back(cluster_to_json(c));
  nlohmann::json contingencies = nlohmann::json::array();
  for (const auto& group : result.reliability.contingencies) {
    nlohmann::json clusters = nlohmann::json::array();
    for (const auto& c : group.clusters) clusters.push_back(cluster_to_json(c));
    contingencies.push_back({{"contingency", contingency_to_json(group.contingency)},
                             {"candidate_count", group.candidate_count},
                             {"cluster_count", group.cluster_count},
                             {"clusters", clusters},
                             {"audit", group.audit}});
  }
  nlohmann::json intensities = nlohmann::json::array();
  for (const auto& group : result.resilience.intensities) {
    nlohmann::json clusters = nlohmann::json::array();
    for (const auto& c : group.clusters) clusters.push_back(cluster_to_json(c));
    intensities.push_back({{"intensity", to_string(group.intensity)},
                           {"candidate_count", group.candidate_count},
                           {"cluster_count", group.cluster_count},
                           {"clusters", clusters},
                           {"audit", group.audit}});
  }
  return {{"success", true},
          {"summary", result.summary},
          {"warnings", result.warnings},
          {"regular", {{"candidate_count", result.regular.candidate_count},
                         {"cluster_count", result.regular.cluster_count},
                         {"clusters", regular_clusters},
                         {"audit", result.regular.audit}}},
          {"reliability", {{"contingency_count", result.reliability.contingency_count},
                            {"cluster_total", result.reliability.cluster_total},
                            {"contingencies", contingencies},
                            {"audit", result.reliability.audit}}},
          {"resilience", {{"intensity_count", result.resilience.intensity_count},
                           {"cluster_total", result.resilience.cluster_total},
                           {"intensities", intensities},
                           {"audit", result.resilience.audit}}}};
}

}  // namespace hacdcpf::analysis

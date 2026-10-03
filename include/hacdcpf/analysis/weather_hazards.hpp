#pragma once

#include <nlohmann/json.hpp>
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace hacdcpf::analysis {

// Research defaults, not locally calibrated engineering design values.
struct RainstormOptions {
  double start_hr{4.0}, duration_hr{6.0}, total_mm{180.0};
  double peak_fraction{0.4}, shape_b_hr{0.5}, shape_n{0.7};
  double runoff{0.85}, drainage_mm_hr{15.0};
  // IEEE Std 4 wet-test water resistivity is 100 +/- 15 ohm-m. Remaining
  // coefficients are explicit synthetic demo calibration, not generic limits.
  double rainwater_resistivity_ohm_m{100.0}, altitude_m{500.0};
  double insulator_surface_c{0.01}, insulator_pressure_exponent{0.5};
  double transformer_oil_initial_ppm{15.0}, transformer_paper_initial_pct{1.0};
  double transformer_oil_a{0.025}, transformer_paper_a{0.00025};
  double transformer_oil_shutdown_ppm{60.0}, transformer_paper_shutdown_pct{3.0};
  double repair_hr{6.0}, severity_variation{0.2};
};
struct LightningOptions {
  double start_hr{4.0}, duration_hr{6.0}, density_km2_hr{2.0};
  double collection_width_m{100.0}, fallback_length_km{1.0};
  double median_current_ka{30.0}, log_current_sigma{0.6}, critical_current_ka{50.0};
  double permanent_fraction{0.2}, transient_duration_hr{1.0};
  double repair_hr{4.0}, severity_variation{0.2};
};
struct WeatherHazardOptions {
  std::string hazard_type{"typhoon"};
  RainstormOptions rainstorm;
  LightningOptions lightning;
};
struct WeatherHazardScenario {
  std::vector<DistributionResilienceFault> faults;
  nlohmann::json evidence;
  double peak_intensity{0.0};
  double peak_failure_probability{0.0};
};

nlohmann::json weather_hazard_schema();
WeatherHazardOptions weather_hazard_options_from_json(const nlohmann::json& resilience);
void validate_weather_hazard_options(const WeatherHazardOptions&, int horizon_hours);
WeatherHazardScenario generate_weather_hazard(const HybridPowerSystem&,
    const WeatherHazardOptions&, int horizon_hours, unsigned seed);
}  // namespace hacdcpf::analysis

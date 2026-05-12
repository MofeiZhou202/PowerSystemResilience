#pragma once
// hacdcdss/model/scenario_data.hpp
//
// Module: model
//
// Scenario and time-series representations used by power-flow simulations and
// planning studies.  A Scenario is a consistent realisation of all uncertain
// parameters (load levels, renewable output) for one operating period.

#include <cstdint>
#include <string>
#include <vector>

namespace hacdcdss::model {

// ── Time-series profile ───────────────────────────────────────────────────────
// One entry per time step (e.g. hourly, 15-minute).
struct Profile {
    std::string         name;
    std::vector<double> values;         // per-unit multipliers or MW absolutes
    bool                is_per_unit = true;
};

// ── Single scenario (one sample of uncertain parameters) ─────────────────────
struct Scenario {
    std::uint32_t        id          = 0;
    double               probability = 1.0;  // sum over all scenarios = 1.0

    // Per-bus and per-generator scaling factors (index-aligned with
    // NetworkModel::ac_buses and NetworkModel::generators respectively).
    std::vector<double>  load_scale;   // size = n_ac_buses
    std::vector<double>  pv_scale;     // size = n_renewable_generators
    std::vector<double>  wind_scale;   // size = n_renewable_generators
};

// ── Collection of scenarios for one planning / simulation period ─────────────
struct ScenarioSet {
    std::string           label;
    std::uint32_t         horizon_h = 8760;  // representative horizon [hours]
    std::vector<Scenario> scenarios;

    std::size_t size() const noexcept { return scenarios.size(); }

    /// Normalise scenario probabilities to sum to 1.0.
    void normalise_probabilities();

    /// Load from a JSON file (HACDCDSS native format).
    static ScenarioSet from_json(const std::string& path);
};

// ── Time-series data container ────────────────────────────────────────────────
struct TimeSeriesData {
    std::string           name;
    std::uint32_t         n_steps = 8760;  // total number of time steps
    double                dt_h    = 1.0;   // step duration [hours]

    std::vector<Profile>  load_profiles;   // one per AC bus
    std::vector<Profile>  pv_profiles;     // one per PV generator
    std::vector<Profile>  wind_profiles;   // one per wind generator
    std::vector<Profile>  price_profiles;  // one per market zone

    static TimeSeriesData from_csv(const std::string& path);
    static TimeSeriesData from_json(const std::string& path);
};

} // namespace hacdcdss::model

#pragma once
// hacdcdss/model/time_series_loader.hpp
//
// Module: model
//
// Lightweight I/O helpers to load load/generation profiles from CSV or JSON
// and build a TimeSeriesData object ready for power-flow simulations.

#include <hacdcdss/model/scenario_data.hpp>
#include <string>

namespace hacdcdss::model {

struct TimeSeriesLoaderOptions {
    char   csv_delimiter  = ',';
    bool   has_header_row = true;
    double missing_value  = 0.0;   // fill value for unparseable cells
};

/// Load time-series data from a CSV file.
/// Each column becomes a Profile.  Column headers (if has_header_row = true)
/// are used as profile names.  Columns whose name starts with "load_" are
/// placed in load_profiles; "pv_" → pv_profiles; "wind_" → wind_profiles;
/// "price_" → price_profiles; others → load_profiles as a fallback.
TimeSeriesData load_csv(const std::string& path,
                        const TimeSeriesLoaderOptions& opts = {});

/// Load time-series data from a JSON file (HACDCDSS native format).
TimeSeriesData load_json(const std::string& path);

/// Serialise a TimeSeriesData to JSON for archival.
void save_json(const TimeSeriesData& data, const std::string& path);

} // namespace hacdcdss::model

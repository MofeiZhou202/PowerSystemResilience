// src/model/scenario_data.cpp
//
// Module: model – ScenarioSet / TimeSeriesData I/O.

#include <hacdcdss/model/scenario_data.hpp>
#include <hacdcdss/model/time_series_loader.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace hacdcdss::model {

// ── ScenarioSet ───────────────────────────────────────────────────────────────

void ScenarioSet::normalise_probabilities()
{
    double total = 0.0;
    for (const auto& s : scenarios) total += s.probability;
    if (total <= 0.0)
        throw std::runtime_error(
            "ScenarioSet::normalise_probabilities: total probability is "
            "non-positive.");
    for (auto& s : scenarios) s.probability /= total;
}

ScenarioSet ScenarioSet::from_json(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error(
            "ScenarioSet::from_json: cannot open '" + path + "'");

    nlohmann::json j;
    f >> j;

    ScenarioSet ss;
    ss.label     = j.value("label",     "unnamed");
    ss.horizon_h = j.value("horizon_h", 8760u);

    for (const auto& js : j.value("scenarios", nlohmann::json::array())) {
        Scenario s;
        s.id          = js.at("id").get<std::uint32_t>();
        s.probability = js.value("probability", 1.0);

        if (js.contains("load_scale"))
            s.load_scale = js["load_scale"].get<std::vector<double>>();
        if (js.contains("pv_scale"))
            s.pv_scale   = js["pv_scale"].get<std::vector<double>>();
        if (js.contains("wind_scale"))
            s.wind_scale = js["wind_scale"].get<std::vector<double>>();

        ss.scenarios.push_back(std::move(s));
    }

    return ss;
}

// ── TimeSeriesData ────────────────────────────────────────────────────────────

TimeSeriesData TimeSeriesData::from_json(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error(
            "TimeSeriesData::from_json: cannot open '" + path + "'");

    nlohmann::json j;
    f >> j;

    TimeSeriesData ts;
    ts.name    = j.value("name",    "unnamed");
    ts.n_steps = j.value("n_steps", 8760u);
    ts.dt_h    = j.value("dt_h",    1.0);

    auto read_profiles = [](const nlohmann::json& arr) {
        std::vector<Profile> profiles;
        for (const auto& jp : arr) {
            Profile p;
            p.name        = jp.value("name", "");
            p.is_per_unit = jp.value("is_per_unit", true);
            if (jp.contains("values"))
                p.values = jp["values"].get<std::vector<double>>();
            profiles.push_back(std::move(p));
        }
        return profiles;
    };

    ts.load_profiles  = read_profiles(
        j.value("load_profiles",  nlohmann::json::array()));
    ts.pv_profiles    = read_profiles(
        j.value("pv_profiles",    nlohmann::json::array()));
    ts.wind_profiles  = read_profiles(
        j.value("wind_profiles",  nlohmann::json::array()));
    ts.price_profiles = read_profiles(
        j.value("price_profiles", nlohmann::json::array()));

    return ts;
}

TimeSeriesData TimeSeriesData::from_csv(const std::string& path)
{
    return load_csv(path);
}

// ── TimeSeriesLoader (CSV + JSON + save) ──────────────────────────────────────

TimeSeriesData load_csv(const std::string& path,
                        const TimeSeriesLoaderOptions& opts)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("load_csv: cannot open '" + path + "'");

    TimeSeriesData ts;
    ts.name = path;

    std::string line;
    bool first_line = true;
    std::vector<std::string> headers;
    std::vector<std::vector<double>> columns;

    while (std::getline(f, line)) {
        if (line.empty()) continue;

        std::istringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        while (std::getline(ss, cell, opts.csv_delimiter))
            row.push_back(cell);

        if (first_line && opts.has_header_row) {
            headers = row;
            columns.resize(headers.size());
            first_line = false;
            continue;
        }
        first_line = false;

        if (columns.empty()) columns.resize(row.size());

        for (std::size_t i = 0; i < row.size() && i < columns.size(); ++i) {
            try {
                columns[i].push_back(std::stod(row[i]));
            } catch (...) {
                columns[i].push_back(opts.missing_value);
            }
        }
    }

    ts.n_steps = columns.empty() ? 0u
               : static_cast<std::uint32_t>(columns[0].size());

    for (std::size_t i = 0; i < columns.size(); ++i) {
        Profile p;
        p.name   = (i < headers.size()) ? headers[i]
                                        : ("col_" + std::to_string(i));
        p.values = std::move(columns[i]);

        // Route profile to the appropriate collection based on name prefix.
        const std::string& n = p.name;
        if (n.rfind("pv_",    0) == 0) ts.pv_profiles.push_back(std::move(p));
        else if (n.rfind("wind_",  0) == 0) ts.wind_profiles.push_back(std::move(p));
        else if (n.rfind("price_", 0) == 0) ts.price_profiles.push_back(std::move(p));
        else                                ts.load_profiles.push_back(std::move(p));
    }

    return ts;
}

TimeSeriesData load_json(const std::string& path)
{
    return TimeSeriesData::from_json(path);
}

void save_json(const TimeSeriesData& data, const std::string& path)
{
    nlohmann::json j;
    j["name"]    = data.name;
    j["n_steps"] = data.n_steps;
    j["dt_h"]    = data.dt_h;

    auto write_profiles = [](const std::vector<Profile>& profiles) {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& p : profiles) {
            nlohmann::json jp;
            jp["name"]        = p.name;
            jp["is_per_unit"] = p.is_per_unit;
            jp["values"]      = p.values;
            arr.push_back(std::move(jp));
        }
        return arr;
    };

    j["load_profiles"]  = write_profiles(data.load_profiles);
    j["pv_profiles"]    = write_profiles(data.pv_profiles);
    j["wind_profiles"]  = write_profiles(data.wind_profiles);
    j["price_profiles"] = write_profiles(data.price_profiles);

    std::ofstream f(path);
    if (!f.is_open())
        throw std::runtime_error("save_json: cannot write to '" + path + "'");
    f << j.dump(2) << '\n';
}

} // namespace hacdcdss::model

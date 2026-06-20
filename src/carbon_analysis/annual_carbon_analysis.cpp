// Annual Carbon Flow Analysis — multi-timestep aggregation and GEC accounting.
// Ported from luosipeng/HybridACDCPowerSystemsPlanning (luosipeng branch).

#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {
namespace {

using json = nlohmann::json;

constexpr double kTol = 1e-12;

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------

json units_json() {
  return json{
      {"energy", "MWh"},
      {"emissions", "tCO2"},
      {"intensity", "tCO2/MWh"},
      {"duration", "h"},
  };
}

json number_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

std::string dump_json(const json& j, int indent) {
  return (indent >= 0) ? j.dump(indent) : j.dump();
}

void save_text_file(const std::string& path,
                    const std::string& content,
                    const std::string& kind) {
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Unable to open " + kind + " for writing: " + path);
  }
  out << content;
}

std::string read_text_file(const std::string& path, const std::string& kind) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Unable to open " + kind + ": " + path);
  }
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

double json_nonneg_double(const json& j,
                          const char* field,
                          double default_value = 0.0) {
  if (!j.contains(field) || j.at(field).is_null()) {
    return default_value;
  }
  const double value = j.at(field).get<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string("JSON field must be finite and non-negative: ") +
                                field);
  }
  return value;
}

bool json_bool_field(const json& j, const char* field) {
  if (!j.contains(field)) {
    throw std::invalid_argument(std::string("JSON missing required field: ") + field);
  }
  if (j.at(field).is_boolean()) {
    return j.at(field).get<bool>();
  }
  if (j.at(field).is_number_integer()) {
    const int value = j.at(field).get<int>();
    if (value == 0 || value == 1) return value == 1;
  }
  throw std::invalid_argument(std::string("JSON field must be boolean or 0/1: ") + field);
}

json annual_summary_json(const AnnualCarbonAnalysisResult& annual) {
  return json{
      {"num_steps", annual.num_steps},
      {"step_duration_hr", annual.step_duration_hr},
      {"num_pf_converged", annual.num_pf_converged},
      {"total_generation_emissions_tco2", annual.total_generation_emissions_tco2},
      {"total_load_emissions_tco2", annual.total_load_emissions_tco2},
      {"total_loss_emissions_tco2", annual.total_loss_emissions_tco2},
  };
}

json bus_summary_row_json(const AnnualBusCarbonStats& row) {
  return json{
      {"bus_index", row.bus_index},
      {"is_dc", row.is_dc},
      {"energy_mwh", row.energy_mwh},
      {"emissions_tco2", row.emissions_tco2},
      {"load_weighted_intensity_tco2_mwh", row.load_weighted_intensity_tco2_mwh},
      {"time_weighted_intensity_tco2_mwh", row.time_weighted_intensity_tco2_mwh},
      {"average_intensity_tco2_mwh", row.average_intensity_tco2_mwh},
      {"min_intensity_tco2_mwh", row.min_intensity_tco2_mwh},
      {"max_intensity_tco2_mwh", row.max_intensity_tco2_mwh},
  };
}

json load_summary_row_json(const AnnualLoadCarbonStats& row) {
  return json{
      {"load_index", row.load_index},
      {"bus", row.bus},
      {"is_dc", row.is_dc},
      {"energy_mwh", row.energy_mwh},
      {"emissions_tco2", row.emissions_tco2},
      {"average_intensity_tco2_mwh", row.average_intensity_tco2_mwh},
  };
}

json node_summary_row_json(const AnnualNodeCarbonStats& row) {
  return json{
      {"bus_index", row.bus_index},
      {"is_dc", row.is_dc},
      {"energy_mwh", row.energy_mwh},
      {"gross_emissions_tco2", row.gross_emissions_tco2},
      {"allocated_gec_mwh", row.allocated_gec_mwh},
      {"unused_gec_mwh", row.unused_gec_mwh},
      {"avoided_emissions_tco2", row.avoided_emissions_tco2},
      {"net_emissions_tco2", row.net_emissions_tco2},
      {"gross_intensity_tco2_mwh", row.gross_intensity_tco2_mwh},
      {"net_intensity_tco2_mwh", row.net_intensity_tco2_mwh},
      {"gec_coverage_ratio", row.gec_coverage_ratio},
  };
}

json node_hourly_row_json(int time_step, const AnnualNodeCarbonStepResult& row) {
  return json{
      {"time_step", time_step},
      {"bus_index", row.bus_index},
      {"is_dc", row.is_dc},
      {"energy_mwh", row.energy_mwh},
      {"gross_emissions_tco2", row.gross_emissions_tco2},
      {"allocated_gec_mwh", row.allocated_gec_mwh},
      {"net_emissions_tco2", row.net_emissions_tco2},
      {"gross_intensity_tco2_mwh", row.gross_intensity_tco2_mwh},
      {"net_intensity_tco2_mwh", row.net_intensity_tco2_mwh},
  };
}

json user_summary_row_json(const AnnualUserCarbonStats& row) {
  return json{
      {"user_id", row.user_id},
      {"energy_mwh", row.energy_mwh},
      {"gross_emissions_tco2", row.gross_emissions_tco2},
      {"allocated_gec_mwh", row.allocated_gec_mwh},
      {"unused_gec_mwh", row.unused_gec_mwh},
      {"avoided_emissions_tco2", row.avoided_emissions_tco2},
      {"net_emissions_tco2", row.net_emissions_tco2},
      {"gross_intensity_tco2_mwh", row.gross_intensity_tco2_mwh},
      {"net_intensity_tco2_mwh", row.net_intensity_tco2_mwh},
      {"gec_coverage_ratio", row.gec_coverage_ratio},
  };
}

json user_hourly_row_json(int time_step, const AnnualUserCarbonStepResult& row) {
  return json{
      {"time_step", time_step},
      {"user_id", row.user_id},
      {"energy_mwh", row.energy_mwh},
      {"gross_emissions_tco2", row.gross_emissions_tco2},
      {"allocated_gec_mwh", row.allocated_gec_mwh},
      {"net_emissions_tco2", row.net_emissions_tco2},
      {"gross_intensity_tco2_mwh", row.gross_intensity_tco2_mwh},
      {"net_intensity_tco2_mwh", row.net_intensity_tco2_mwh},
  };
}

json node_input_row_json(const AnnualNodeGECInput& row) {
  return json{
      {"bus_index", row.bus_index},
      {"is_dc", row.is_dc},
      {"annual_gec_mwh", row.annual_gec_mwh},
      {"gec_intensity_tco2_mwh", row.gec_intensity_tco2_mwh},
  };
}

json user_input_row_json(const AnnualUserGECInput& row) {
  json loads = json::array();
  for (const auto& load : row.loads) {
    loads.push_back(json{{"load_index", load.load_index}, {"is_dc", load.is_dc}});
  }
  return json{
      {"user_id", row.user_id},
      {"annual_gec_mwh", row.annual_gec_mwh},
      {"gec_intensity_tco2_mwh", row.gec_intensity_tco2_mwh},
      {"loads", std::move(loads)},
  };
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

double finite_or_zero(double value) {
  return std::isfinite(value) ? value : 0.0;
}

double step_generation_emissions(const CarbonAnalysisResult& ca) {
  if (ca.matrix_solved) {
    return ca.matrix_summary.total_generation_emissions_tco2;
  }
  return ca.tracing_summary.total_generation_emissions_tco2;
}

double step_load_emissions(const CarbonAnalysisResult& ca) {
  if (ca.matrix_solved) {
    return ca.matrix_summary.total_load_emissions_tco2;
  }
  return ca.tracing_summary.total_load_emissions_tco2;
}

double step_loss_emissions(const CarbonAnalysisResult& ca) {
  if (ca.matrix_solved) {
    return ca.matrix_summary.total_loss_emissions_tco2;
  }
  return ca.tracing_summary.total_loss_emissions_tco2;
}

struct BusKey {
  int index{0};
  bool is_dc{false};

  bool operator==(const BusKey& other) const {
    return index == other.index && is_dc == other.is_dc;
  }
};

struct BusKeyHash {
  size_t operator()(const BusKey& key) const {
    const auto a = static_cast<size_t>(static_cast<unsigned int>(key.index));
    return (a << 1U) ^ static_cast<size_t>(key.is_dc);
  }
};

struct LoadKey {
  int index{0};
  int bus{0};
  bool is_dc{false};

  bool operator==(const LoadKey& other) const {
    return index == other.index && bus == other.bus && is_dc == other.is_dc;
  }
};

struct LoadKeyHash {
  size_t operator()(const LoadKey& key) const {
    size_t h = static_cast<size_t>(static_cast<unsigned int>(key.index));
    h = h * 131U + static_cast<size_t>(static_cast<unsigned int>(key.bus));
    h = h * 131U + static_cast<size_t>(key.is_dc);
    return h;
  }
};

struct LoadRefKey {
  int index{0};
  bool is_dc{false};

  bool operator==(const LoadRefKey& other) const {
    return index == other.index && is_dc == other.is_dc;
  }
};

struct LoadRefKeyHash {
  size_t operator()(const LoadRefKey& key) const {
    const auto a = static_cast<size_t>(static_cast<unsigned int>(key.index));
    return (a << 1U) ^ static_cast<size_t>(key.is_dc);
  }
};

struct BusAccum {
  double intensity_time_hr{0.0};
  double intensity_sample_hr{0.0};
};

void add_bus_result(const BusCarbonResult& row,
                    bool is_dc,
                    double step_duration_hr,
                    std::vector<AnnualBusCarbonStats>& stats,
                    std::unordered_map<BusKey, size_t, BusKeyHash>& pos,
                    std::vector<BusAccum>& accum,
                    std::vector<double>* hourly_row) {
  const BusKey key{row.bus_index, is_dc};
  auto it = pos.find(key);
  if (it == pos.end()) {
    AnnualBusCarbonStats item;
    item.bus_index = row.bus_index;
    item.is_dc = is_dc;
    item.min_intensity_tco2_mwh = std::numeric_limits<double>::infinity();
    item.max_intensity_tco2_mwh = -std::numeric_limits<double>::infinity();
    const size_t new_pos = stats.size();
    stats.push_back(item);
    accum.push_back(BusAccum{});
    it = pos.emplace(key, new_pos).first;
  }

  auto& item = stats[it->second];
  auto& extra = accum[it->second];
  const double intensity = std::max(0.0, finite_or_zero(row.carbon_intensity_tco2_mwh));
  item.min_intensity_tco2_mwh = std::min(item.min_intensity_tco2_mwh, intensity);
  item.max_intensity_tco2_mwh = std::max(item.max_intensity_tco2_mwh, intensity);
  extra.intensity_time_hr += intensity * step_duration_hr;
  extra.intensity_sample_hr += step_duration_hr;

  if (hourly_row != nullptr) {
    if (it->second >= hourly_row->size()) {
      hourly_row->resize(it->second + 1, std::numeric_limits<double>::quiet_NaN());
    }
    (*hourly_row)[it->second] = intensity;
  }
}

void add_load_result(const LoadCarbonResult& row,
                     bool is_dc,
                     double step_duration_hr,
                     std::vector<AnnualLoadCarbonStats>& stats,
                     std::unordered_map<LoadKey, size_t, LoadKeyHash>& pos,
                     std::vector<AnnualBusCarbonStats>& bus_stats,
                     std::unordered_map<BusKey, size_t, BusKeyHash>& bus_pos,
                     std::vector<BusAccum>& bus_accum,
                     std::vector<double>* hourly_emissions_row,
                     std::vector<double>* hourly_energy_row) {
  const LoadKey key{row.load_index, row.bus, is_dc};
  auto it = pos.find(key);
  if (it == pos.end()) {
    AnnualLoadCarbonStats item;
    item.load_index = row.load_index;
    item.bus = row.bus;
    item.is_dc = is_dc;
    const size_t new_pos = stats.size();
    stats.push_back(item);
    it = pos.emplace(key, new_pos).first;
  }

  auto& item = stats[it->second];
  const double energy_mwh = std::max(0.0, finite_or_zero(row.demand_mw)) * step_duration_hr;
  const double emissions_tco2 =
      std::max(0.0, finite_or_zero(row.total_emissions_tco2)) * step_duration_hr;
  item.energy_mwh += energy_mwh;
  item.emissions_tco2 += emissions_tco2;

  const BusKey bus_key{row.bus, is_dc};
  auto bus_it = bus_pos.find(bus_key);
  if (bus_it == bus_pos.end()) {
    AnnualBusCarbonStats bus_item;
    bus_item.bus_index = row.bus;
    bus_item.is_dc = is_dc;
    bus_item.min_intensity_tco2_mwh = std::numeric_limits<double>::infinity();
    bus_item.max_intensity_tco2_mwh = -std::numeric_limits<double>::infinity();
    const size_t new_pos = bus_stats.size();
    bus_stats.push_back(bus_item);
    bus_accum.push_back(BusAccum{});
    bus_it = bus_pos.emplace(bus_key, new_pos).first;
  }
  bus_stats[bus_it->second].energy_mwh += energy_mwh;
  bus_stats[bus_it->second].emissions_tco2 += emissions_tco2;

  if (hourly_emissions_row != nullptr) {
    if (it->second >= hourly_emissions_row->size()) {
      hourly_emissions_row->resize(it->second + 1,
                                   std::numeric_limits<double>::quiet_NaN());
    }
    (*hourly_emissions_row)[it->second] = emissions_tco2;
  }
  if (hourly_energy_row != nullptr) {
    if (it->second >= hourly_energy_row->size()) {
      hourly_energy_row->resize(it->second + 1,
                                std::numeric_limits<double>::quiet_NaN());
    }
    (*hourly_energy_row)[it->second] = energy_mwh;
  }
}

void finalize_stats(AnnualCarbonAnalysisResult& result,
                    const std::vector<BusAccum>& bus_accum) {
  for (size_t i = 0; i < result.bus_stats.size(); ++i) {
    auto& row = result.bus_stats[i];
    if (row.energy_mwh > kTol) {
      row.load_weighted_intensity_tco2_mwh = row.emissions_tco2 / row.energy_mwh;
      row.average_intensity_tco2_mwh = row.load_weighted_intensity_tco2_mwh;
    }
    if (i < bus_accum.size() && bus_accum[i].intensity_sample_hr > kTol) {
      row.time_weighted_intensity_tco2_mwh =
          bus_accum[i].intensity_time_hr / bus_accum[i].intensity_sample_hr;
    }
    if (!std::isfinite(row.min_intensity_tco2_mwh)) {
      row.min_intensity_tco2_mwh = 0.0;
    }
    if (!std::isfinite(row.max_intensity_tco2_mwh)) {
      row.max_intensity_tco2_mwh = 0.0;
    }
  }

  for (auto& row : result.load_stats) {
    if (row.energy_mwh > kTol) {
      row.average_intensity_tco2_mwh = row.emissions_tco2 / row.energy_mwh;
    }
  }
}

void extend_hourly_width(std::vector<std::vector<double>>& rows, size_t width, double fill) {
  for (auto& row : rows) {
    if (row.size() < width) row.resize(width, fill);
  }
}

Storage* find_storage_by_index(std::vector<Storage>& storage, int index) {
  for (auto& item : storage) {
    if (item.index == index) return &item;
  }
  return nullptr;
}

const Storage* find_storage_by_index(const std::vector<Storage>& storage, int index) {
  for (const auto& item : storage) {
    if (item.index == index) return &item;
  }
  return nullptr;
}

double stored_energy_from_soc(const Storage& storage) {
  return std::max(storage.soc_init, 0.0) * std::max(storage.e_rated_mwh, 0.0);
}

double stored_energy_mwh(const Storage& storage) {
  if (storage.e_mwh > kTol) {
    return std::max(storage.e_mwh, 0.0);
  }
  return stored_energy_from_soc(storage);
}

std::unordered_map<LoadRefKey, size_t, LoadRefKeyHash> build_load_position_map(
    const std::vector<AnnualLoadCarbonStats>& load_stats) {
  std::unordered_map<LoadRefKey, size_t, LoadRefKeyHash> pos;
  pos.reserve(load_stats.size());
  for (size_t i = 0; i < load_stats.size(); ++i) {
    const LoadRefKey key{load_stats[i].load_index, load_stats[i].is_dc};
    if (pos.find(key) != pos.end()) {
      throw std::invalid_argument("load_index/is_dc mapping is ambiguous");
    }
    pos.emplace(key, i);
  }
  return pos;
}

void validate_user_gec_inputs(const AnnualCarbonAnalysisResult& annual,
                               const std::vector<AnnualUserGECInput>& users) {
  if (annual.hourly_load_energy_mwh.empty()) {
    throw std::invalid_argument(
        "annual.hourly_load_energy_mwh is required; enable keep_hourly_load_energy");
  }
  if (annual.hourly_load_emissions_tco2.empty()) {
    throw std::invalid_argument(
        "annual.hourly_load_emissions_tco2 is required; enable keep_hourly_load_emissions");
  }
  if (annual.hourly_load_energy_mwh.size() != static_cast<size_t>(annual.num_steps) ||
      annual.hourly_load_emissions_tco2.size() != static_cast<size_t>(annual.num_steps)) {
    throw std::invalid_argument("annual hourly load arrays must match num_steps");
  }
  for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
    if (annual.hourly_load_energy_mwh[t].size() != annual.load_stats.size() ||
        annual.hourly_load_emissions_tco2[t].size() != annual.load_stats.size()) {
      throw std::invalid_argument("annual hourly load arrays must match load_stats order");
    }
  }
  for (const auto& user : users) {
    if (user.loads.empty()) {
      throw std::invalid_argument("AnnualUserGECInput.loads must not be empty");
    }
    if (!std::isfinite(user.annual_gec_mwh) || user.annual_gec_mwh < -kTol) {
      throw std::invalid_argument("annual_gec_mwh must be finite and non-negative");
    }
    if (!std::isfinite(user.gec_intensity_tco2_mwh) ||
        user.gec_intensity_tco2_mwh < -kTol) {
      throw std::invalid_argument("gec_intensity_tco2_mwh must be finite and non-negative");
    }
  }

  std::unordered_map<LoadRefKey, int, LoadRefKeyHash> owner_by_load;
  for (const auto& user : users) {
    std::unordered_map<LoadRefKey, bool, LoadRefKeyHash> seen_in_user;
    seen_in_user.reserve(user.loads.size());
    for (const auto& ref : user.loads) {
      const LoadRefKey key{ref.load_index, ref.is_dc};
      if (seen_in_user.find(key) != seen_in_user.end()) {
        throw std::invalid_argument("AnnualUserGECInput.loads contains duplicate load refs");
      }
      seen_in_user.emplace(key, true);

      const auto owner_it = owner_by_load.find(key);
      if (owner_it != owner_by_load.end()) {
        throw std::invalid_argument("AnnualLoadRef is assigned to multiple users");
      }
      owner_by_load.emplace(key, user.user_id);
    }
  }
}

void carry_storage_carbon_state(const StorageCarbonResult& carbon,
                                const Storage& current,
                                Storage& next) {
  double next_intensity = current.soc_carbon_intensity_tco2_mwh;
  if (current.p_mw < -kTol) {
    const double e_start = stored_energy_mwh(current);
    const double e_end = stored_energy_mwh(next);
    const double d_e_stored = std::max(e_end - e_start, 0.0);
    const double bus_intensity =
        std::max(0.0, finite_or_zero(carbon.carbon_intensity_tco2_mwh));
    const double c_next =
        e_start * std::max(0.0, finite_or_zero(current.soc_carbon_intensity_tco2_mwh)) +
        d_e_stored * bus_intensity;
    next_intensity = (e_end > kTol) ? (c_next / e_end) : 0.0;
  }

  next.soc_carbon_intensity_tco2_mwh =
      std::max(0.0, finite_or_zero(next_intensity));
}

void carry_storage_carbon_state_from_terminal_snapshot(
    const StorageCarbonResult& carbon,
    const Storage& current,
    Storage& next,
    double step_duration_hr,
    const Storage* previous_terminal) {
  double next_intensity = current.soc_carbon_intensity_tco2_mwh;
  if (current.p_mw < -kTol) {
    const double e_end = stored_energy_mwh(current);
    double e_start = 0.0;
    if (previous_terminal != nullptr) {
      e_start = stored_energy_mwh(*previous_terminal);
    } else {
      const double eta_charge = (current.eta_charge > 0.0) ? current.eta_charge : 1.0;
      const double charged_energy =
          std::max(0.0, -current.p_mw * step_duration_hr * eta_charge);
      e_start = std::max(0.0, e_end - charged_energy);
    }
    const double d_e_stored = std::max(e_end - e_start, 0.0);
    const double bus_intensity =
        std::max(0.0, finite_or_zero(carbon.carbon_intensity_tco2_mwh));
    const double c_next =
        e_start * std::max(0.0, finite_or_zero(current.soc_carbon_intensity_tco2_mwh)) +
        d_e_stored * bus_intensity;
    next_intensity = (e_end > kTol) ? (c_next / e_end) : 0.0;
  }

  next.soc_carbon_intensity_tco2_mwh =
      std::max(0.0, finite_or_zero(next_intensity));
}

void update_storage_carbon_state(const HybridPowerSystem& current,
                                 HybridPowerSystem& next,
                                 const CarbonAnalysisResult& ca) {
  for (const auto& carbon : ca.storage_carbon) {
    if (carbon.is_dc) {
      const Storage* current_storage =
          find_storage_by_index(current.dc.storage, carbon.storage_index);
      Storage* next_storage = find_storage_by_index(next.dc.storage, carbon.storage_index);
      if (current_storage != nullptr && next_storage != nullptr) {
        carry_storage_carbon_state(carbon, *current_storage, *next_storage);
      }
    } else {
      const Storage* current_storage =
          find_storage_by_index(current.ac.storage, carbon.storage_index);
      Storage* next_storage = find_storage_by_index(next.ac.storage, carbon.storage_index);
      if (current_storage != nullptr && next_storage != nullptr) {
        carry_storage_carbon_state(carbon, *current_storage, *next_storage);
      }
    }
  }
}

void update_storage_carbon_state_from_terminal_snapshot(
    const HybridPowerSystem& current,
    const HybridPowerSystem* previous_terminal,
    HybridPowerSystem& next,
    const CarbonAnalysisResult& ca,
    double step_duration_hr) {
  for (const auto& carbon : ca.storage_carbon) {
    if (carbon.is_dc) {
      const Storage* current_storage =
          find_storage_by_index(current.dc.storage, carbon.storage_index);
      const Storage* previous_storage =
          (previous_terminal != nullptr)
              ? find_storage_by_index(previous_terminal->dc.storage, carbon.storage_index)
              : nullptr;
      Storage* next_storage = find_storage_by_index(next.dc.storage, carbon.storage_index);
      if (current_storage != nullptr && next_storage != nullptr) {
        carry_storage_carbon_state_from_terminal_snapshot(
            carbon, *current_storage, *next_storage, step_duration_hr, previous_storage);
      }
    } else {
      const Storage* current_storage =
          find_storage_by_index(current.ac.storage, carbon.storage_index);
      const Storage* previous_storage =
          (previous_terminal != nullptr)
              ? find_storage_by_index(previous_terminal->ac.storage, carbon.storage_index)
              : nullptr;
      Storage* next_storage = find_storage_by_index(next.ac.storage, carbon.storage_index);
      if (current_storage != nullptr && next_storage != nullptr) {
        carry_storage_carbon_state_from_terminal_snapshot(
            carbon, *current_storage, *next_storage, step_duration_hr, previous_storage);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Core implementation (shared across all compute_annual_carbon_analysis overloads)
// ---------------------------------------------------------------------------

AnnualCarbonAnalysisResult compute_annual_carbon_analysis_impl(
    const std::function<const HybridPowerSystem&(size_t)>& system_at,
    const std::vector<PowerFlowResult>& pf_results,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options,
    const std::function<void(size_t, const CarbonAnalysisResult&)>& after_step = {}) {
  if (!(step_duration_hr > 0.0) || !std::isfinite(step_duration_hr)) {
    throw std::invalid_argument("step_duration_hr must be positive and finite");
  }

  AnnualCarbonAnalysisResult result;
  result.num_steps = static_cast<int>(pf_results.size());
  result.step_duration_hr = step_duration_hr;
  result.step_results.resize(pf_results.size());

  std::unordered_map<BusKey, size_t, BusKeyHash> bus_pos;
  std::unordered_map<LoadKey, size_t, LoadKeyHash> load_pos;
  std::vector<BusAccum> bus_accum;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (options.keep_hourly_bus_intensity) {
    result.hourly_bus_intensity_tco2_mwh.resize(pf_results.size());
  }
  if (options.keep_hourly_load_emissions) {
    result.hourly_load_emissions_tco2.resize(pf_results.size());
  }
  if (options.keep_hourly_load_energy) {
    result.hourly_load_energy_mwh.resize(pf_results.size());
  }

  for (size_t t = 0; t < pf_results.size(); ++t) {
    auto& step = result.step_results[t];
    step.pf_converged = pf_results[t].converged;
    if (!pf_results[t].converged) {
      continue;
    }

    ++result.num_pf_converged;
    const CarbonAnalysisResult ca =
        compute_carbon_analysis(system_at(t), pf_results[t], options.carbon_options);

    step.total_generation_emissions_tco2 =
        std::max(0.0, finite_or_zero(step_generation_emissions(ca))) * step_duration_hr;
    step.total_load_emissions_tco2 =
        std::max(0.0, finite_or_zero(step_load_emissions(ca))) * step_duration_hr;
    step.total_loss_emissions_tco2 =
        std::max(0.0, finite_or_zero(step_loss_emissions(ca))) * step_duration_hr;

    result.total_generation_emissions_tco2 += step.total_generation_emissions_tco2;
    result.total_load_emissions_tco2 += step.total_load_emissions_tco2;
    result.total_loss_emissions_tco2 += step.total_loss_emissions_tco2;

    std::vector<double>* hourly_bus = nullptr;
    if (options.keep_hourly_bus_intensity) {
      extend_hourly_width(result.hourly_bus_intensity_tco2_mwh, result.bus_stats.size(), nan);
      result.hourly_bus_intensity_tco2_mwh[t].resize(result.bus_stats.size(), nan);
      hourly_bus = &result.hourly_bus_intensity_tco2_mwh[t];
    }

    for (const auto& row : ca.bus_carbon) {
      add_bus_result(row, false, step_duration_hr, result.bus_stats, bus_pos,
                     bus_accum, hourly_bus);
      if (options.keep_hourly_bus_intensity) {
        extend_hourly_width(result.hourly_bus_intensity_tco2_mwh, result.bus_stats.size(), nan);
      }
    }
    for (const auto& row : ca.dc_bus_carbon) {
      add_bus_result(row, true, step_duration_hr, result.bus_stats, bus_pos,
                     bus_accum, hourly_bus);
      if (options.keep_hourly_bus_intensity) {
        extend_hourly_width(result.hourly_bus_intensity_tco2_mwh, result.bus_stats.size(), nan);
      }
    }

    std::vector<double>* hourly_load = nullptr;
    if (options.keep_hourly_load_emissions) {
      extend_hourly_width(result.hourly_load_emissions_tco2, result.load_stats.size(), nan);
      result.hourly_load_emissions_tco2[t].resize(result.load_stats.size(), nan);
      hourly_load = &result.hourly_load_emissions_tco2[t];
    }
    std::vector<double>* hourly_load_energy = nullptr;
    if (options.keep_hourly_load_energy) {
      extend_hourly_width(result.hourly_load_energy_mwh, result.load_stats.size(), nan);
      result.hourly_load_energy_mwh[t].resize(result.load_stats.size(), nan);
      hourly_load_energy = &result.hourly_load_energy_mwh[t];
    }

    for (const auto& row : ca.load_carbon) {
      add_load_result(row, false, step_duration_hr, result.load_stats, load_pos,
                      result.bus_stats, bus_pos, bus_accum, hourly_load,
                      hourly_load_energy);
      if (options.keep_hourly_load_emissions) {
        extend_hourly_width(result.hourly_load_emissions_tco2, result.load_stats.size(), nan);
      }
      if (options.keep_hourly_load_energy) {
        extend_hourly_width(result.hourly_load_energy_mwh, result.load_stats.size(), nan);
      }
    }
    for (const auto& row : ca.dc_load_carbon) {
      add_load_result(row, true, step_duration_hr, result.load_stats, load_pos,
                      result.bus_stats, bus_pos, bus_accum, hourly_load,
                      hourly_load_energy);
      if (options.keep_hourly_load_emissions) {
        extend_hourly_width(result.hourly_load_emissions_tco2, result.load_stats.size(), nan);
      }
      if (options.keep_hourly_load_energy) {
        extend_hourly_width(result.hourly_load_energy_mwh, result.load_stats.size(), nan);
      }
    }

    if (after_step) {
      after_step(t, ca);
    }
  }

  if (options.keep_hourly_bus_intensity) {
    extend_hourly_width(result.hourly_bus_intensity_tco2_mwh, result.bus_stats.size(), nan);
  }
  if (options.keep_hourly_load_emissions) {
    extend_hourly_width(result.hourly_load_emissions_tco2, result.load_stats.size(), nan);
  }
  if (options.keep_hourly_load_energy) {
    extend_hourly_width(result.hourly_load_energy_mwh, result.load_stats.size(), nan);
  }

  finalize_stats(result, bus_accum);
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const HybridPowerSystem& sys,
    const std::vector<PowerFlowResult>& pf_results,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options) {
  return compute_annual_carbon_analysis_impl(
      [&](size_t) -> const HybridPowerSystem& { return sys; },
      pf_results,
      step_duration_hr,
      options);
}

AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const std::vector<HybridPowerSystem>& systems,
    const std::vector<PowerFlowResult>& pf_results,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options) {
  if (systems.size() != pf_results.size()) {
    throw std::invalid_argument("systems.size() must equal pf_results.size()");
  }
  std::vector<HybridPowerSystem> mutable_systems = systems;
  return compute_annual_carbon_analysis_impl(
      [&](size_t t) -> const HybridPowerSystem& { return mutable_systems[t]; },
      pf_results,
      step_duration_hr,
      options,
      [&](size_t t, const CarbonAnalysisResult& ca) {
        if (t + 1 < mutable_systems.size()) {
          update_storage_carbon_state(mutable_systems[t],
                                      mutable_systems[t + 1],
                                      ca);
        }
      });
}

AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const HybridPowerSystem& sys,
    const TimeSeriesPFResult& ts_result,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options) {
  if (ts_result.pf_system_snapshots.size() == ts_result.pf_results.size()) {
    return compute_annual_carbon_analysis(ts_result, step_duration_hr, options);
  }
  return compute_annual_carbon_analysis(sys, ts_result.pf_results, step_duration_hr, options);
}

AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const TimeSeriesPFResult& ts_result,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options) {
  if (ts_result.pf_system_snapshots.size() != ts_result.pf_results.size()) {
    throw std::invalid_argument(
        "ts_result.pf_system_snapshots.size() must equal ts_result.pf_results.size()");
  }
  std::vector<HybridPowerSystem> mutable_systems = ts_result.pf_system_snapshots;
  return compute_annual_carbon_analysis_impl(
      [&](size_t t) -> const HybridPowerSystem& { return mutable_systems[t]; },
      ts_result.pf_results,
      step_duration_hr,
      options,
      [&](size_t t, const CarbonAnalysisResult& ca) {
        if (t + 1 < mutable_systems.size()) {
          const HybridPowerSystem* previous_terminal =
              (t > 0) ? &mutable_systems[t - 1] : nullptr;
          update_storage_carbon_state_from_terminal_snapshot(
              mutable_systems[t],
              previous_terminal,
              mutable_systems[t + 1],
              ca,
              step_duration_hr);
        }
      });
}

AnnualUserGECResult compute_annual_user_gec_accounting(
    const AnnualCarbonAnalysisResult& annual,
    const std::vector<AnnualUserGECInput>& users,
    bool keep_hourly) {
  validate_user_gec_inputs(annual, users);

  AnnualUserGECResult result;
  result.num_steps = annual.num_steps;
  result.step_duration_hr = annual.step_duration_hr;
  result.user_stats.resize(users.size());
  if (keep_hourly) {
    result.hourly_user_results.resize(static_cast<size_t>(annual.num_steps));
    for (auto& row : result.hourly_user_results) {
      row.resize(users.size());
    }
  }

  const auto load_pos = build_load_position_map(annual.load_stats);

  for (size_t ui = 0; ui < users.size(); ++ui) {
    const auto& user = users[ui];
    auto& stats = result.user_stats[ui];
    stats.user_id = user.user_id;

    std::vector<size_t> load_columns;
    load_columns.reserve(user.loads.size());
    for (const auto& ref : user.loads) {
      const auto it = load_pos.find(LoadRefKey{ref.load_index, ref.is_dc});
      if (it == load_pos.end()) {
        throw std::invalid_argument("user load reference not found in annual.load_stats");
      }
      load_columns.push_back(it->second);
    }

    std::vector<double> step_energy(static_cast<size_t>(annual.num_steps), 0.0);
    std::vector<double> step_gross(static_cast<size_t>(annual.num_steps), 0.0);
    for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
      bool has_finite = false;
      for (size_t col : load_columns) {
        const double energy = annual.hourly_load_energy_mwh[t][col];
        const double emissions = annual.hourly_load_emissions_tco2[t][col];
        if (std::isfinite(energy)) {
          step_energy[t] += std::max(0.0, energy);
          has_finite = true;
        }
        if (std::isfinite(emissions)) {
          step_gross[t] += std::max(0.0, emissions);
        }
      }
      if (!has_finite) {
        step_energy[t] = 0.0;
        step_gross[t] = 0.0;
      }
      stats.energy_mwh += step_energy[t];
      stats.gross_emissions_tco2 += step_gross[t];
    }

    const double annual_gec =
        std::max(0.0, finite_or_zero(user.annual_gec_mwh));
    if (stats.energy_mwh <= kTol) {
      stats.unused_gec_mwh = annual_gec;
      if (keep_hourly) {
        for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
          result.hourly_user_results[t][ui].user_id = user.user_id;
        }
      }
      continue;
    }

    for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
      const double energy = step_energy[t];
      const double gross = step_gross[t];
      const double gross_intensity = (energy > kTol) ? (gross / energy) : 0.0;
      const double planned_gec = annual_gec * energy / stats.energy_mwh;
      const double allocated_gec = std::min(planned_gec, energy);
      const double gec_intensity =
          std::max(0.0, finite_or_zero(user.gec_intensity_tco2_mwh));
      const double net =
          (energy - allocated_gec) * gross_intensity + allocated_gec * gec_intensity;

      stats.allocated_gec_mwh += allocated_gec;
      stats.net_emissions_tco2 += net;

      if (keep_hourly) {
        auto& step = result.hourly_user_results[t][ui];
        step.user_id = user.user_id;
        step.energy_mwh = energy;
        step.gross_emissions_tco2 = gross;
        step.allocated_gec_mwh = allocated_gec;
        step.net_emissions_tco2 = net;
        step.gross_intensity_tco2_mwh = gross_intensity;
        step.net_intensity_tco2_mwh = (energy > kTol) ? (net / energy) : 0.0;
      }
    }

    stats.unused_gec_mwh = std::max(0.0, annual_gec - stats.allocated_gec_mwh);
    stats.avoided_emissions_tco2 =
        stats.gross_emissions_tco2 - stats.net_emissions_tco2;
    stats.gross_intensity_tco2_mwh =
        stats.gross_emissions_tco2 / stats.energy_mwh;
    stats.net_intensity_tco2_mwh =
        stats.net_emissions_tco2 / stats.energy_mwh;
    stats.gec_coverage_ratio = stats.allocated_gec_mwh / stats.energy_mwh;
  }

  return result;
}

std::vector<AnnualUserGECInput> make_one_load_per_user_gec_inputs(
    const AnnualCarbonAnalysisResult& annual,
    int first_user_id) {
  std::vector<AnnualUserGECInput> users;
  users.reserve(annual.load_stats.size());
  for (size_t i = 0; i < annual.load_stats.size(); ++i) {
    const auto& load = annual.load_stats[i];
    AnnualUserGECInput user;
    user.user_id = first_user_id + static_cast<int>(i);
    user.loads.push_back(AnnualLoadRef{load.load_index, load.is_dc});
    users.push_back(std::move(user));
  }
  return users;
}

// ---------------------------------------------------------------------------
// User GEC CSV/JSON output
// ---------------------------------------------------------------------------

std::string annual_user_gec_result_to_csv(const AnnualUserGECResult& result) {
  std::ostringstream out;
  out << "user_id,energy_mwh,gross_emissions_tco2,allocated_gec_mwh,"
         "unused_gec_mwh,avoided_emissions_tco2,net_emissions_tco2,"
         "gross_intensity_tco2_mwh,net_intensity_tco2_mwh,gec_coverage_ratio\n";
  out << std::setprecision(12);
  for (const auto& row : result.user_stats) {
    out << row.user_id << ','
        << row.energy_mwh << ','
        << row.gross_emissions_tco2 << ','
        << row.allocated_gec_mwh << ','
        << row.unused_gec_mwh << ','
        << row.avoided_emissions_tco2 << ','
        << row.net_emissions_tco2 << ','
        << row.gross_intensity_tco2_mwh << ','
        << row.net_intensity_tco2_mwh << ','
        << row.gec_coverage_ratio << '\n';
  }
  return out.str();
}

void save_annual_user_gec_result_csv(const AnnualUserGECResult& result,
                                     const std::string& path) {
  save_text_file(path, annual_user_gec_result_to_csv(result), "annual user GEC CSV");
}

std::string annual_user_gec_result_to_json(const AnnualUserGECResult& result,
                                           int indent) {
  json rows = json::array();
  for (const auto& row : result.user_stats) {
    rows.push_back(user_summary_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", {{"num_steps", result.num_steps},
                   {"step_duration_hr", result.step_duration_hr},
                   {"num_users", result.user_stats.size()}}},
      {"user_gec_summary", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_user_gec_result_json(const AnnualUserGECResult& result,
                                      const std::string& path,
                                      int indent) {
  save_text_file(path, annual_user_gec_result_to_json(result, indent),
                 "annual user GEC JSON");
}

std::string annual_user_gec_hourly_to_csv(const AnnualUserGECResult& result) {
  if (result.hourly_user_results.empty()) {
    throw std::invalid_argument(
        "hourly_user_results is empty; call compute_annual_user_gec_accounting with keep_hourly=true");
  }
  std::ostringstream out;
  out << "time_step,user_id,energy_mwh,gross_emissions_tco2,allocated_gec_mwh,"
         "net_emissions_tco2,gross_intensity_tco2_mwh,net_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  for (size_t t = 0; t < result.hourly_user_results.size(); ++t) {
    for (const auto& row : result.hourly_user_results[t]) {
      out << t << ','
          << row.user_id << ','
          << row.energy_mwh << ','
          << row.gross_emissions_tco2 << ','
          << row.allocated_gec_mwh << ','
          << row.net_emissions_tco2 << ','
          << row.gross_intensity_tco2_mwh << ','
          << row.net_intensity_tco2_mwh << '\n';
    }
  }
  return out.str();
}

void save_annual_user_gec_hourly_csv(const AnnualUserGECResult& result,
                                     const std::string& path) {
  save_text_file(path, annual_user_gec_hourly_to_csv(result), "hourly user GEC CSV");
}

std::string annual_user_gec_hourly_to_json(const AnnualUserGECResult& result,
                                           int indent) {
  if (result.hourly_user_results.empty()) {
    throw std::invalid_argument(
        "hourly_user_results is empty; call compute_annual_user_gec_accounting with keep_hourly=true");
  }
  json rows = json::array();
  for (size_t t = 0; t < result.hourly_user_results.size(); ++t) {
    for (const auto& row : result.hourly_user_results[t]) {
      rows.push_back(user_hourly_row_json(static_cast<int>(t), row));
    }
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", {{"num_steps", result.num_steps},
                   {"step_duration_hr", result.step_duration_hr},
                   {"num_users", result.user_stats.size()}}},
      {"user_gec_hourly", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_user_gec_hourly_json(const AnnualUserGECResult& result,
                                      const std::string& path,
                                      int indent) {
  save_text_file(path, annual_user_gec_hourly_to_json(result, indent),
                 "hourly user GEC JSON");
}

bool validate_annual_user_gec_result(const AnnualUserGECResult& result,
                                     double tol) {
  if (!std::isfinite(tol) || tol < 0.0) {
    return false;
  }
  const auto near = [tol](double a, double b) {
    return std::abs(a - b) <= tol;
  };
  for (const auto& stats : result.user_stats) {
    if (stats.energy_mwh < -tol ||
        stats.gross_emissions_tco2 < -tol ||
        stats.allocated_gec_mwh < -tol ||
        stats.unused_gec_mwh < -tol ||
        stats.net_emissions_tco2 < -tol ||
        stats.gec_coverage_ratio < -tol ||
        stats.gec_coverage_ratio > 1.0 + tol ||
        stats.allocated_gec_mwh > stats.energy_mwh + tol) {
      return false;
    }
  }

  if (result.hourly_user_results.empty()) {
    return true;
  }
  if (result.hourly_user_results.size() != static_cast<size_t>(result.num_steps)) {
    return false;
  }
  for (const auto& row : result.hourly_user_results) {
    if (row.size() != result.user_stats.size()) {
      return false;
    }
  }

  for (size_t ui = 0; ui < result.user_stats.size(); ++ui) {
    double energy = 0.0;
    double gross = 0.0;
    double allocated = 0.0;
    double net = 0.0;
    for (const auto& step_row : result.hourly_user_results) {
      const auto& step = step_row[ui];
      if (step.energy_mwh < -tol ||
          step.gross_emissions_tco2 < -tol ||
          step.allocated_gec_mwh < -tol ||
          step.net_emissions_tco2 < -tol ||
          step.allocated_gec_mwh > step.energy_mwh + tol) {
        return false;
      }
      energy += step.energy_mwh;
      gross += step.gross_emissions_tco2;
      allocated += step.allocated_gec_mwh;
      net += step.net_emissions_tco2;
    }
    const auto& stats = result.user_stats[ui];
    if (!near(energy, stats.energy_mwh) ||
        !near(gross, stats.gross_emissions_tco2) ||
        !near(allocated, stats.allocated_gec_mwh) ||
        !near(net, stats.net_emissions_tco2)) {
      return false;
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// Bus-level CSV/JSON helpers
// ---------------------------------------------------------------------------

std::string annual_bus_carbon_hourly_to_csv(const AnnualCarbonAnalysisResult& annual) {
  if (annual.hourly_bus_intensity_tco2_mwh.empty()) {
    throw std::invalid_argument(
        "hourly_bus_intensity_tco2_mwh is empty; "
        "enable keep_hourly_bus_intensity in AnnualCarbonAnalysisOptions");
  }
  std::ostringstream out;
  out << "time_step,bus_index,is_dc,carbon_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  const size_t num_buses = annual.bus_stats.size();
  for (size_t t = 0; t < annual.hourly_bus_intensity_tco2_mwh.size(); ++t) {
    const auto& row = annual.hourly_bus_intensity_tco2_mwh[t];
    for (size_t b = 0; b < num_buses; ++b) {
      const double intensity = (b < row.size()) ? row[b]
                                                : std::numeric_limits<double>::quiet_NaN();
      out << t << ',' << annual.bus_stats[b].bus_index << ','
          << (annual.bus_stats[b].is_dc ? 1 : 0) << ',';
      if (std::isnan(intensity)) {
        out << "nan";
      } else {
        out << intensity;
      }
      out << '\n';
    }
  }
  return out.str();
}

void save_annual_bus_carbon_hourly_csv(const AnnualCarbonAnalysisResult& annual,
                                       const std::string& path) {
  save_text_file(path, annual_bus_carbon_hourly_to_csv(annual), "bus hourly CSV");
}

std::string annual_bus_carbon_hourly_to_json(const AnnualCarbonAnalysisResult& annual,
                                             int indent) {
  if (annual.hourly_bus_intensity_tco2_mwh.empty()) {
    throw std::invalid_argument(
        "hourly_bus_intensity_tco2_mwh is empty; "
        "enable keep_hourly_bus_intensity in AnnualCarbonAnalysisOptions");
  }
  json rows = json::array();
  for (size_t t = 0; t < annual.hourly_bus_intensity_tco2_mwh.size(); ++t) {
    const auto& row = annual.hourly_bus_intensity_tco2_mwh[t];
    for (size_t b = 0; b < annual.bus_stats.size(); ++b) {
      const double intensity = (b < row.size()) ? row[b]
                                                : std::numeric_limits<double>::quiet_NaN();
      rows.push_back(json{
          {"time_step", static_cast<int>(t)},
          {"bus_index", annual.bus_stats[b].bus_index},
          {"is_dc", annual.bus_stats[b].is_dc},
          {"carbon_intensity_tco2_mwh", number_or_null(intensity)},
      });
    }
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", annual_summary_json(annual)},
      {"bus_carbon_hourly", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_bus_carbon_hourly_json(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path,
                                        int indent) {
  save_text_file(path, annual_bus_carbon_hourly_to_json(annual, indent), "bus hourly JSON");
}

std::string annual_bus_carbon_summary_to_csv(const AnnualCarbonAnalysisResult& annual) {
  std::ostringstream out;
  out << "bus_index,is_dc,energy_mwh,emissions_tco2,"
         "load_weighted_intensity_tco2_mwh,time_weighted_intensity_tco2_mwh,"
         "min_intensity_tco2_mwh,max_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  for (const auto& row : annual.bus_stats) {
    out << row.bus_index << ',' << (row.is_dc ? 1 : 0) << ','
        << row.energy_mwh << ',' << row.emissions_tco2 << ','
        << row.load_weighted_intensity_tco2_mwh << ','
        << row.time_weighted_intensity_tco2_mwh << ','
        << row.min_intensity_tco2_mwh << ','
        << row.max_intensity_tco2_mwh << '\n';
  }
  return out.str();
}

void save_annual_bus_carbon_summary_csv(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path) {
  save_text_file(path, annual_bus_carbon_summary_to_csv(annual), "bus summary CSV");
}

std::string annual_bus_carbon_summary_to_json(const AnnualCarbonAnalysisResult& annual,
                                              int indent) {
  json rows = json::array();
  for (const auto& row : annual.bus_stats) {
    rows.push_back(bus_summary_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", annual_summary_json(annual)},
      {"bus_carbon_summary", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_bus_carbon_summary_json(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path,
                                         int indent) {
  save_text_file(path, annual_bus_carbon_summary_to_json(annual, indent), "bus summary JSON");
}

// ---------------------------------------------------------------------------
// Load-level CSV/JSON helpers
// ---------------------------------------------------------------------------

std::string annual_load_carbon_hourly_to_csv(const AnnualCarbonAnalysisResult& annual) {
  if (annual.hourly_load_energy_mwh.empty()) {
    throw std::invalid_argument(
        "hourly_load_energy_mwh is empty; "
        "enable keep_hourly_load_energy in AnnualCarbonAnalysisOptions");
  }
  if (annual.hourly_load_emissions_tco2.empty()) {
    throw std::invalid_argument(
        "hourly_load_emissions_tco2 is empty; "
        "enable keep_hourly_load_emissions in AnnualCarbonAnalysisOptions");
  }
  std::ostringstream out;
  out << "time_step,load_index,bus,is_dc,energy_mwh,gross_emissions_tco2,"
         "gross_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  const size_t num_loads = annual.load_stats.size();
  const size_t num_steps = static_cast<size_t>(annual.num_steps);
  for (size_t t = 0; t < num_steps; ++t) {
    const auto& row_e = annual.hourly_load_energy_mwh[t];
    const auto& row_em = annual.hourly_load_emissions_tco2[t];
    for (size_t l = 0; l < num_loads; ++l) {
      const double energy = (l < row_e.size()) ? row_e[l]
                                               : std::numeric_limits<double>::quiet_NaN();
      const double emissions = (l < row_em.size()) ? row_em[l]
                                                    : std::numeric_limits<double>::quiet_NaN();
      double intensity;
      if (std::isnan(energy) || std::isnan(emissions)) {
        intensity = std::numeric_limits<double>::quiet_NaN();
      } else if (energy > kTol) {
        intensity = emissions / energy;
      } else {
        intensity = 0.0;
      }
      out << t << ',' << annual.load_stats[l].load_index << ','
          << annual.load_stats[l].bus << ','
          << (annual.load_stats[l].is_dc ? 1 : 0) << ',';
      auto write_val = [&out](double v) {
        if (std::isnan(v)) { out << "nan"; } else { out << v; }
      };
      write_val(energy); out << ',';
      write_val(emissions); out << ',';
      write_val(intensity); out << '\n';
    }
  }
  return out.str();
}

void save_annual_load_carbon_hourly_csv(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path) {
  save_text_file(path, annual_load_carbon_hourly_to_csv(annual), "load hourly CSV");
}

std::string annual_load_carbon_hourly_to_json(const AnnualCarbonAnalysisResult& annual,
                                              int indent) {
  if (annual.hourly_load_energy_mwh.empty()) {
    throw std::invalid_argument(
        "hourly_load_energy_mwh is empty; "
        "enable keep_hourly_load_energy in AnnualCarbonAnalysisOptions");
  }
  if (annual.hourly_load_emissions_tco2.empty()) {
    throw std::invalid_argument(
        "hourly_load_emissions_tco2 is empty; "
        "enable keep_hourly_load_emissions in AnnualCarbonAnalysisOptions");
  }
  json rows = json::array();
  const size_t num_steps = static_cast<size_t>(annual.num_steps);
  for (size_t t = 0; t < num_steps; ++t) {
    const auto& row_e = annual.hourly_load_energy_mwh[t];
    const auto& row_em = annual.hourly_load_emissions_tco2[t];
    for (size_t l = 0; l < annual.load_stats.size(); ++l) {
      const double energy = (l < row_e.size()) ? row_e[l]
                                               : std::numeric_limits<double>::quiet_NaN();
      const double emissions = (l < row_em.size()) ? row_em[l]
                                                    : std::numeric_limits<double>::quiet_NaN();
      double intensity = std::numeric_limits<double>::quiet_NaN();
      if (std::isfinite(energy) && std::isfinite(emissions)) {
        intensity = (energy > kTol) ? (emissions / energy) : 0.0;
      }
      rows.push_back(json{
          {"time_step", static_cast<int>(t)},
          {"load_index", annual.load_stats[l].load_index},
          {"bus", annual.load_stats[l].bus},
          {"is_dc", annual.load_stats[l].is_dc},
          {"energy_mwh", number_or_null(energy)},
          {"gross_emissions_tco2", number_or_null(emissions)},
          {"gross_intensity_tco2_mwh", number_or_null(intensity)},
      });
    }
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", annual_summary_json(annual)},
      {"load_carbon_hourly", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_load_carbon_hourly_json(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path,
                                         int indent) {
  save_text_file(path, annual_load_carbon_hourly_to_json(annual, indent), "load hourly JSON");
}

std::string annual_load_carbon_summary_to_csv(const AnnualCarbonAnalysisResult& annual) {
  std::ostringstream out;
  out << "load_index,bus,is_dc,energy_mwh,emissions_tco2,average_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  for (const auto& row : annual.load_stats) {
    out << row.load_index << ',' << row.bus << ','
        << (row.is_dc ? 1 : 0) << ','
        << row.energy_mwh << ',' << row.emissions_tco2 << ','
        << row.average_intensity_tco2_mwh << '\n';
  }
  return out.str();
}

void save_annual_load_carbon_summary_csv(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path) {
  save_text_file(path, annual_load_carbon_summary_to_csv(annual), "load summary CSV");
}

std::string annual_load_carbon_summary_to_json(const AnnualCarbonAnalysisResult& annual,
                                               int indent) {
  json rows = json::array();
  for (const auto& row : annual.load_stats) {
    rows.push_back(load_summary_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", annual_summary_json(annual)},
      {"load_carbon_summary", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_load_carbon_summary_json(const AnnualCarbonAnalysisResult& annual,
                                          const std::string& path,
                                          int indent) {
  save_text_file(path, annual_load_carbon_summary_to_json(annual, indent), "load summary JSON");
}

// ---------------------------------------------------------------------------
// Node-level GEC accounting
// ---------------------------------------------------------------------------

namespace {

void validate_node_gec_inputs(const AnnualCarbonAnalysisResult& annual,
                               const std::vector<AnnualNodeGECInput>& nodes) {
  if (annual.hourly_load_energy_mwh.empty()) {
    throw std::invalid_argument(
        "annual.hourly_load_energy_mwh is required; enable keep_hourly_load_energy");
  }
  if (annual.hourly_load_emissions_tco2.empty()) {
    throw std::invalid_argument(
        "annual.hourly_load_emissions_tco2 is required; enable keep_hourly_load_emissions");
  }
  if (annual.hourly_load_energy_mwh.size() != static_cast<size_t>(annual.num_steps) ||
      annual.hourly_load_emissions_tco2.size() != static_cast<size_t>(annual.num_steps)) {
    throw std::invalid_argument("annual hourly load arrays must match num_steps");
  }
  for (const auto& node : nodes) {
    if (!std::isfinite(node.annual_gec_mwh) || node.annual_gec_mwh < -kTol) {
      throw std::invalid_argument("annual_gec_mwh must be finite and non-negative");
    }
    if (!std::isfinite(node.gec_intensity_tco2_mwh) ||
        node.gec_intensity_tco2_mwh < -kTol) {
      throw std::invalid_argument("gec_intensity_tco2_mwh must be finite and non-negative");
    }
  }

  std::unordered_map<BusKey, int, BusKeyHash> seen;
  for (const auto& node : nodes) {
    const BusKey key{node.bus_index, node.is_dc};
    if (seen.find(key) != seen.end()) {
      throw std::invalid_argument("duplicate {bus_index,is_dc} in node GEC inputs");
    }
    seen.emplace(key, 1);
  }
}

void aggregate_load_by_bus(
    const AnnualCarbonAnalysisResult& annual,
    const std::vector<AnnualNodeGECInput>& nodes,
    std::vector<std::vector<double>>& node_step_energy,
    std::vector<std::vector<double>>& node_step_gross) {

  const size_t num_nodes = nodes.size();
  const size_t num_steps = static_cast<size_t>(annual.num_steps);
  node_step_energy.assign(num_nodes, std::vector<double>(num_steps, 0.0));
  node_step_gross.assign(num_nodes, std::vector<double>(num_steps, 0.0));

  std::unordered_map<BusKey, size_t, BusKeyHash> node_idx;
  node_idx.reserve(num_nodes);
  for (size_t ni = 0; ni < num_nodes; ++ni) {
    node_idx.emplace(BusKey{nodes[ni].bus_index, nodes[ni].is_dc}, ni);
  }

  for (size_t l = 0; l < annual.load_stats.size(); ++l) {
    const auto& ls = annual.load_stats[l];
    const BusKey key{ls.bus, ls.is_dc};
    const auto it = node_idx.find(key);
    if (it == node_idx.end()) continue;
    const size_t ni = it->second;

    for (size_t t = 0; t < num_steps; ++t) {
      const auto& row_e = annual.hourly_load_energy_mwh[t];
      const auto& row_em = annual.hourly_load_emissions_tco2[t];
      if (l < row_e.size() && std::isfinite(row_e[l])) {
        node_step_energy[ni][t] += std::max(0.0, row_e[l]);
      }
      if (l < row_em.size() && std::isfinite(row_em[l])) {
        node_step_gross[ni][t] += std::max(0.0, row_em[l]);
      }
    }
  }
}

}  // anonymous namespace

AnnualNodeGECResult compute_annual_node_gec_accounting(
    const AnnualCarbonAnalysisResult& annual,
    const std::vector<AnnualNodeGECInput>& nodes,
    bool keep_hourly) {
  validate_node_gec_inputs(annual, nodes);

  AnnualNodeGECResult result;
  result.num_steps = annual.num_steps;
  result.step_duration_hr = annual.step_duration_hr;
  result.node_stats.resize(nodes.size());
  if (keep_hourly) {
    result.hourly_node_results.resize(static_cast<size_t>(annual.num_steps));
    for (auto& row : result.hourly_node_results) {
      row.resize(nodes.size());
    }
  }

  std::vector<std::vector<double>> node_step_energy;
  std::vector<std::vector<double>> node_step_gross;
  aggregate_load_by_bus(annual, nodes, node_step_energy, node_step_gross);

  for (size_t ni = 0; ni < nodes.size(); ++ni) {
    const auto& node = nodes[ni];
    auto& stats = result.node_stats[ni];
    stats.bus_index = node.bus_index;
    stats.is_dc = node.is_dc;

    const auto& energies = node_step_energy[ni];
    const auto& grosses = node_step_gross[ni];

    double annual_energy = 0.0;
    for (double e : energies) annual_energy += e;
    for (double em : grosses) stats.gross_emissions_tco2 += em;
    stats.energy_mwh = annual_energy;

    if (keep_hourly) {
      for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
        result.hourly_node_results[t][ni].bus_index = node.bus_index;
        result.hourly_node_results[t][ni].is_dc = node.is_dc;
      }
    }

    const double annual_gec = std::max(0.0, finite_or_zero(node.annual_gec_mwh));
    if (annual_energy <= kTol) {
      stats.unused_gec_mwh = annual_gec;
      continue;
    }

    const double gec_intensity =
        std::max(0.0, finite_or_zero(node.gec_intensity_tco2_mwh));

    for (size_t t = 0; t < static_cast<size_t>(annual.num_steps); ++t) {
      const double energy = energies[t];
      const double gross = grosses[t];
      const double gross_intensity = (energy > kTol) ? (gross / energy) : 0.0;
      const double planned_gec = annual_gec * energy / annual_energy;
      const double allocated_gec = std::min(planned_gec, energy);
      const double net =
          (energy - allocated_gec) * gross_intensity + allocated_gec * gec_intensity;

      stats.allocated_gec_mwh += allocated_gec;
      stats.net_emissions_tco2 += net;

      if (keep_hourly) {
        auto& step = result.hourly_node_results[t][ni];
        step.energy_mwh = energy;
        step.gross_emissions_tco2 = gross;
        step.allocated_gec_mwh = allocated_gec;
        step.net_emissions_tco2 = net;
        step.gross_intensity_tco2_mwh = gross_intensity;
        step.net_intensity_tco2_mwh = (energy > kTol) ? (net / energy) : 0.0;
      }
    }

    stats.unused_gec_mwh = std::max(0.0, annual_gec - stats.allocated_gec_mwh);
    stats.avoided_emissions_tco2 =
        stats.gross_emissions_tco2 - stats.net_emissions_tco2;
    stats.gross_intensity_tco2_mwh =
        (annual_energy > kTol) ? (stats.gross_emissions_tco2 / annual_energy) : 0.0;
    stats.net_intensity_tco2_mwh =
        (annual_energy > kTol) ? (stats.net_emissions_tco2 / annual_energy) : 0.0;
    stats.gec_coverage_ratio = stats.allocated_gec_mwh / annual_energy;
  }

  return result;
}

// ---------------------------------------------------------------------------
// Node GEC CSV/JSON helpers
// ---------------------------------------------------------------------------

std::string annual_node_gec_result_to_csv(const AnnualNodeGECResult& result) {
  std::ostringstream out;
  out << "bus_index,is_dc,energy_mwh,gross_emissions_tco2,allocated_gec_mwh,"
         "unused_gec_mwh,avoided_emissions_tco2,net_emissions_tco2,"
         "gross_intensity_tco2_mwh,net_intensity_tco2_mwh,gec_coverage_ratio\n";
  out << std::setprecision(12);
  for (const auto& row : result.node_stats) {
    out << row.bus_index << ',' << (row.is_dc ? 1 : 0) << ','
        << row.energy_mwh << ',' << row.gross_emissions_tco2 << ','
        << row.allocated_gec_mwh << ',' << row.unused_gec_mwh << ','
        << row.avoided_emissions_tco2 << ',' << row.net_emissions_tco2 << ','
        << row.gross_intensity_tco2_mwh << ',' << row.net_intensity_tco2_mwh << ','
        << row.gec_coverage_ratio << '\n';
  }
  return out.str();
}

void save_annual_node_gec_result_csv(const AnnualNodeGECResult& result,
                                     const std::string& path) {
  save_text_file(path, annual_node_gec_result_to_csv(result), "node GEC result CSV");
}

std::string annual_node_gec_result_to_json(const AnnualNodeGECResult& result,
                                           int indent) {
  json rows = json::array();
  for (const auto& row : result.node_stats) {
    rows.push_back(node_summary_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", {{"num_steps", result.num_steps},
                   {"step_duration_hr", result.step_duration_hr},
                   {"num_nodes", result.node_stats.size()}}},
      {"node_gec_summary", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_node_gec_result_json(const AnnualNodeGECResult& result,
                                      const std::string& path,
                                      int indent) {
  save_text_file(path, annual_node_gec_result_to_json(result, indent),
                 "node GEC result JSON");
}

std::string annual_node_gec_hourly_to_csv(const AnnualNodeGECResult& result) {
  if (result.hourly_node_results.empty()) {
    throw std::invalid_argument(
        "hourly_node_results is empty; call compute_annual_node_gec_accounting with keep_hourly=true");
  }
  std::ostringstream out;
  out << "time_step,bus_index,is_dc,energy_mwh,gross_emissions_tco2,"
         "allocated_gec_mwh,net_emissions_tco2,"
         "gross_intensity_tco2_mwh,net_intensity_tco2_mwh\n";
  out << std::setprecision(12);
  for (size_t t = 0; t < result.hourly_node_results.size(); ++t) {
    for (const auto& row : result.hourly_node_results[t]) {
      out << t << ',' << row.bus_index << ',' << (row.is_dc ? 1 : 0) << ','
          << row.energy_mwh << ',' << row.gross_emissions_tco2 << ','
          << row.allocated_gec_mwh << ',' << row.net_emissions_tco2 << ','
          << row.gross_intensity_tco2_mwh << ',' << row.net_intensity_tco2_mwh << '\n';
    }
  }
  return out.str();
}

void save_annual_node_gec_hourly_csv(const AnnualNodeGECResult& result,
                                     const std::string& path) {
  save_text_file(path, annual_node_gec_hourly_to_csv(result), "node GEC hourly CSV");
}

std::string annual_node_gec_hourly_to_json(const AnnualNodeGECResult& result,
                                           int indent) {
  if (result.hourly_node_results.empty()) {
    throw std::invalid_argument(
        "hourly_node_results is empty; call compute_annual_node_gec_accounting with keep_hourly=true");
  }
  json rows = json::array();
  for (size_t t = 0; t < result.hourly_node_results.size(); ++t) {
    for (const auto& row : result.hourly_node_results[t]) {
      rows.push_back(node_hourly_row_json(static_cast<int>(t), row));
    }
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"summary", {{"num_steps", result.num_steps},
                   {"step_duration_hr", result.step_duration_hr},
                   {"num_nodes", result.node_stats.size()}}},
      {"node_gec_hourly", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_node_gec_hourly_json(const AnnualNodeGECResult& result,
                                      const std::string& path,
                                      int indent) {
  save_text_file(path, annual_node_gec_hourly_to_json(result, indent),
                 "node GEC hourly JSON");
}

bool validate_annual_node_gec_result(const AnnualNodeGECResult& result, double tol) {
  if (!std::isfinite(tol) || tol < 0.0) return false;
  const auto near = [tol](double a, double b) {
    return std::abs(a - b) <= tol;
  };
  for (const auto& stats : result.node_stats) {
    if (stats.energy_mwh < -tol ||
        stats.gross_emissions_tco2 < -tol ||
        stats.allocated_gec_mwh < -tol ||
        stats.unused_gec_mwh < -tol ||
        stats.net_emissions_tco2 < -tol ||
        stats.gec_coverage_ratio < -tol ||
        stats.gec_coverage_ratio > 1.0 + tol ||
        stats.allocated_gec_mwh > stats.energy_mwh + tol) {
      return false;
    }
  }

  if (result.hourly_node_results.empty()) return true;
  if (result.hourly_node_results.size() != static_cast<size_t>(result.num_steps)) {
    return false;
  }
  for (const auto& row : result.hourly_node_results) {
    if (row.size() != result.node_stats.size()) return false;
  }
  for (size_t ni = 0; ni < result.node_stats.size(); ++ni) {
    double energy = 0.0, gross = 0.0, allocated = 0.0, net = 0.0;
    for (const auto& step_row : result.hourly_node_results) {
      const auto& step = step_row[ni];
      if (step.energy_mwh < -tol || step.gross_emissions_tco2 < -tol ||
          step.allocated_gec_mwh < -tol || step.net_emissions_tco2 < -tol ||
          step.allocated_gec_mwh > step.energy_mwh + tol) {
        return false;
      }
      energy += step.energy_mwh;
      gross += step.gross_emissions_tco2;
      allocated += step.allocated_gec_mwh;
      net += step.net_emissions_tco2;
    }
    const auto& stats = result.node_stats[ni];
    if (!near(energy, stats.energy_mwh) ||
        !near(gross, stats.gross_emissions_tco2) ||
        !near(allocated, stats.allocated_gec_mwh) ||
        !near(net, stats.net_emissions_tco2)) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// CSV parser helpers
// ---------------------------------------------------------------------------

namespace {

std::string trim(const std::string& s) {
  const size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) return {};
  const size_t end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

std::vector<std::string> split_csv_row(const std::string& line) {
  std::vector<std::string> cols;
  std::istringstream ss(line);
  std::string col;
  while (std::getline(ss, col, ',')) {
    cols.push_back(trim(col));
  }
  return cols;
}

bool parse_bool_field(const std::string& s, const std::string& field_name) {
  const std::string lower = [&s]() {
    std::string r = s;
    for (auto& c : r) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return r;
  }();
  if (lower == "1" || lower == "true") return true;
  if (lower == "0" || lower == "false") return false;
  throw std::invalid_argument("Invalid boolean value for field '" + field_name + "': " + s);
}

double parse_nonneg_double(const std::string& s, const std::string& field_name) {
  try {
    size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size()) {
      throw std::invalid_argument("trailing chars");
    }
    if (!std::isfinite(v) || v < 0.0) {
      throw std::invalid_argument("must be finite and non-negative");
    }
    return v;
  } catch (...) {
    throw std::invalid_argument("Invalid non-negative double for field '" + field_name + "': " + s);
  }
}

}  // anonymous namespace

std::vector<AnnualNodeGECInput> parse_annual_node_gec_inputs_csv(
    const std::string& csv_text) {
  std::istringstream ss(csv_text);
  std::string line;

  const std::string expected_header =
      "bus_index,is_dc,annual_gec_mwh,gec_intensity_tco2_mwh";
  const std::string expected_header3 = "bus_index,is_dc,annual_gec_mwh";
  bool has_intensity_col = true;

  while (std::getline(ss, line)) {
    const std::string tl = trim(line);
    if (tl.empty()) continue;
    if (tl == expected_header) {
      has_intensity_col = true;
      break;
    }
    if (tl == expected_header3) {
      has_intensity_col = false;
      break;
    }
    throw std::invalid_argument(
        "Node GEC CSV: expected header '" + expected_header + "', got: " + tl);
  }

  std::vector<AnnualNodeGECInput> result;
  std::unordered_map<BusKey, int, BusKeyHash> seen;

  while (std::getline(ss, line)) {
    const std::string tl = trim(line);
    if (tl.empty()) continue;
    const auto cols = split_csv_row(tl);
    const size_t min_cols = has_intensity_col ? 4u : 3u;
    if (cols.size() < min_cols) {
      throw std::invalid_argument("Node GEC CSV: too few columns in row: " + tl);
    }

    AnnualNodeGECInput item;
    try {
      item.bus_index = std::stoi(cols[0]);
    } catch (...) {
      throw std::invalid_argument("Node GEC CSV: invalid bus_index: " + cols[0]);
    }
    item.is_dc = parse_bool_field(cols[1], "is_dc");
    item.annual_gec_mwh = parse_nonneg_double(cols[2], "annual_gec_mwh");
    if (has_intensity_col && cols.size() >= 4 && !cols[3].empty()) {
      item.gec_intensity_tco2_mwh = parse_nonneg_double(cols[3], "gec_intensity_tco2_mwh");
    }

    const BusKey key{item.bus_index, item.is_dc};
    if (seen.find(key) != seen.end()) {
      throw std::invalid_argument(
          "Node GEC CSV: duplicate {bus_index,is_dc}: " + std::to_string(item.bus_index));
    }
    seen.emplace(key, 1);
    result.push_back(item);
  }
  return result;
}

std::vector<AnnualNodeGECInput> load_annual_node_gec_inputs_csv(const std::string& path) {
  return parse_annual_node_gec_inputs_csv(
      read_text_file(path, "node GEC input CSV"));
}

std::string annual_node_gec_inputs_to_json(
    const std::vector<AnnualNodeGECInput>& nodes,
    int indent) {
  json rows = json::array();
  for (const auto& row : nodes) {
    rows.push_back(node_input_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"node_gec", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_node_gec_inputs_json(
    const std::vector<AnnualNodeGECInput>& nodes,
    const std::string& path,
    int indent) {
  save_text_file(path, annual_node_gec_inputs_to_json(nodes, indent),
                 "node GEC input JSON");
}

std::vector<AnnualNodeGECInput> parse_annual_node_gec_inputs_json(
    const std::string& json_text) {
  json doc;
  try {
    doc = json::parse(json_text);
  } catch (const json::exception& e) {
    throw std::invalid_argument(std::string("Node GEC JSON parse error: ") + e.what());
  }
  if (!doc.is_object() || !doc.contains("node_gec") || !doc.at("node_gec").is_array()) {
    throw std::invalid_argument("Node GEC JSON must contain array field 'node_gec'");
  }

  std::vector<AnnualNodeGECInput> result;
  std::unordered_map<BusKey, int, BusKeyHash> seen;
  for (const auto& row : doc.at("node_gec")) {
    if (!row.is_object()) {
      throw std::invalid_argument("Node GEC JSON row must be an object");
    }
    AnnualNodeGECInput item;
    item.bus_index = row.at("bus_index").get<int>();
    item.is_dc = json_bool_field(row, "is_dc");
    item.annual_gec_mwh = json_nonneg_double(row, "annual_gec_mwh");
    item.gec_intensity_tco2_mwh =
        json_nonneg_double(row, "gec_intensity_tco2_mwh", 0.0);

    const BusKey key{item.bus_index, item.is_dc};
    if (seen.find(key) != seen.end()) {
      throw std::invalid_argument(
          "Node GEC JSON: duplicate {bus_index,is_dc}: " +
          std::to_string(item.bus_index));
    }
    seen.emplace(key, 1);
    result.push_back(item);
  }
  return result;
}

std::vector<AnnualNodeGECInput> load_annual_node_gec_inputs_json(
    const std::string& path) {
  return parse_annual_node_gec_inputs_json(
      read_text_file(path, "node GEC input JSON"));
}

std::vector<AnnualUserGECInput> parse_annual_user_gec_inputs_csv(
    const std::string& csv_text) {
  std::istringstream ss(csv_text);
  std::string line;

  const std::string expected_header =
      "user_id,load_index,is_dc,annual_gec_mwh,gec_intensity_tco2_mwh";
  const std::string expected_header4 =
      "user_id,load_index,is_dc,annual_gec_mwh";
  bool has_intensity_col = true;

  while (std::getline(ss, line)) {
    const std::string tl = trim(line);
    if (tl.empty()) continue;
    if (tl == expected_header) {
      has_intensity_col = true;
      break;
    }
    if (tl == expected_header4) {
      has_intensity_col = false;
      break;
    }
    throw std::invalid_argument(
        "User GEC CSV: expected header '" + expected_header + "', got: " + tl);
  }

  struct UserMeta { size_t idx; double gec; double intensity; };
  std::unordered_map<int, UserMeta> user_map;
  std::vector<AnnualUserGECInput> result;

  while (std::getline(ss, line)) {
    const std::string tl = trim(line);
    if (tl.empty()) continue;
    const auto cols = split_csv_row(tl);
    const size_t min_cols = has_intensity_col ? 5u : 4u;
    if (cols.size() < min_cols) {
      throw std::invalid_argument("User GEC CSV: too few columns in row: " + tl);
    }

    int user_id = 0;
    try { user_id = std::stoi(cols[0]); }
    catch (...) { throw std::invalid_argument("User GEC CSV: invalid user_id: " + cols[0]); }

    int load_index = 0;
    try { load_index = std::stoi(cols[1]); }
    catch (...) { throw std::invalid_argument("User GEC CSV: invalid load_index: " + cols[1]); }

    const bool is_dc = parse_bool_field(cols[2], "is_dc");
    const double gec = parse_nonneg_double(cols[3], "annual_gec_mwh");
    double intensity = 0.0;
    if (has_intensity_col && cols.size() >= 5 && !cols[4].empty()) {
      intensity = parse_nonneg_double(cols[4], "gec_intensity_tco2_mwh");
    }

    const auto it = user_map.find(user_id);
    if (it == user_map.end()) {
      AnnualUserGECInput user;
      user.user_id = user_id;
      user.annual_gec_mwh = gec;
      user.gec_intensity_tco2_mwh = intensity;
      user.loads.push_back(AnnualLoadRef{load_index, is_dc});
      user_map.emplace(user_id, UserMeta{result.size(), gec, intensity});
      result.push_back(std::move(user));
    } else {
      const auto& meta = it->second;
      if (std::abs(meta.gec - gec) > kTol || std::abs(meta.intensity - intensity) > kTol) {
        throw std::invalid_argument(
            "User GEC CSV: conflicting annual_gec_mwh or gec_intensity for user_id " +
            std::to_string(user_id));
      }
      result[meta.idx].loads.push_back(AnnualLoadRef{load_index, is_dc});
    }
  }
  return result;
}

std::vector<AnnualUserGECInput> load_annual_user_gec_inputs_csv(const std::string& path) {
  return parse_annual_user_gec_inputs_csv(
      read_text_file(path, "user GEC input CSV"));
}

std::string annual_user_gec_inputs_to_json(
    const std::vector<AnnualUserGECInput>& users,
    int indent) {
  json rows = json::array();
  for (const auto& row : users) {
    rows.push_back(user_input_row_json(row));
  }
  const json doc{
      {"version", 1},
      {"units", units_json()},
      {"users", std::move(rows)},
  };
  return dump_json(doc, indent);
}

void save_annual_user_gec_inputs_json(
    const std::vector<AnnualUserGECInput>& users,
    const std::string& path,
    int indent) {
  save_text_file(path, annual_user_gec_inputs_to_json(users, indent),
                 "user GEC input JSON");
}

std::vector<AnnualUserGECInput> parse_annual_user_gec_inputs_json(
    const std::string& json_text) {
  json doc;
  try {
    doc = json::parse(json_text);
  } catch (const json::exception& e) {
    throw std::invalid_argument(std::string("User GEC JSON parse error: ") + e.what());
  }
  if (!doc.is_object() || !doc.contains("users") || !doc.at("users").is_array()) {
    throw std::invalid_argument("User GEC JSON must contain array field 'users'");
  }

  std::vector<AnnualUserGECInput> result;
  std::unordered_map<int, bool> seen_users;
  for (const auto& row : doc.at("users")) {
    if (!row.is_object()) {
      throw std::invalid_argument("User GEC JSON row must be an object");
    }
    AnnualUserGECInput user;
    user.user_id = row.at("user_id").get<int>();
    if (seen_users.find(user.user_id) != seen_users.end()) {
      throw std::invalid_argument("User GEC JSON: duplicate user_id " +
                                  std::to_string(user.user_id));
    }
    seen_users.emplace(user.user_id, true);
    user.annual_gec_mwh = json_nonneg_double(row, "annual_gec_mwh");
    user.gec_intensity_tco2_mwh =
        json_nonneg_double(row, "gec_intensity_tco2_mwh", 0.0);
    if (!row.contains("loads") || !row.at("loads").is_array() || row.at("loads").empty()) {
      throw std::invalid_argument("User GEC JSON user must contain non-empty array 'loads'");
    }
    for (const auto& load_row : row.at("loads")) {
      if (!load_row.is_object()) {
        throw std::invalid_argument("User GEC JSON load row must be an object");
      }
      AnnualLoadRef ref;
      ref.load_index = load_row.at("load_index").get<int>();
      ref.is_dc = json_bool_field(load_row, "is_dc");
      user.loads.push_back(ref);
    }
    result.push_back(std::move(user));
  }
  return result;
}

std::vector<AnnualUserGECInput> load_annual_user_gec_inputs_json(
    const std::string& path) {
  return parse_annual_user_gec_inputs_json(
      read_text_file(path, "user GEC input JSON"));
}

}  // namespace hacdcpf::analysis

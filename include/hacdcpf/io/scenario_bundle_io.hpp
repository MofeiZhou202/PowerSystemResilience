#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace hacdcpf::io {

enum class ScenarioBundleImportMode {
  Strict,
  Permissive,
};

struct ScenarioBundleIoReport {
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  std::vector<std::pair<std::string, int>> sheet_counts;

  int count(const std::string& sheet) const {
    for (const auto& kv : sheet_counts) {
      if (kv.first == sheet) return kv.second;
    }
    return 0;
  }
};

nlohmann::json validate_scenario_bundle(const nlohmann::json& bundle,
                                         ScenarioBundleImportMode mode,
                                         ScenarioBundleIoReport& report);

nlohmann::json normalize_generated_scenario_bundle(const nlohmann::json& input,
                                                   ScenarioBundleIoReport& report);

#ifdef HACDCPF_ENABLE_ETAP

void save_scenario_workbook(const nlohmann::json& scenario_bundle,
                            const std::string& path,
                            ScenarioBundleIoReport& report);

nlohmann::json load_scenario_workbook(const std::string& path,
                                      ScenarioBundleImportMode mode,
                                      ScenarioBundleIoReport& report);

#else

inline void save_scenario_workbook(const nlohmann::json&, const std::string&, ScenarioBundleIoReport&) {
  throw std::runtime_error("Scenario workbook I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

inline nlohmann::json load_scenario_workbook(const std::string&, ScenarioBundleImportMode, ScenarioBundleIoReport&) {
  throw std::runtime_error("Scenario workbook I/O not compiled (HACDCPF_ENABLE_ETAP not set)");
}

#endif

}  // namespace hacdcpf::io

// tests/baselines/load_baselines.hpp
// Utility to load JSON regression baselines from the tests/baselines/ directory.
#pragma once

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace baselines {

/// Return the path to tests/baselines/ relative to the calling source file.
inline std::filesystem::path baselines_dir() {
  // __FILE__ is resolved at the includer's compilation, e.g. tests/test_xxx.cpp
  // The baselines dir is tests/baselines/ (sibling of the test file).
  return std::filesystem::path(__FILE__).parent_path();
}

/// Load and parse a JSON baseline file by name (e.g. "solver_baselines.json").
inline nlohmann::json load(const std::string& filename) {
  auto path = baselines_dir() / filename;
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Cannot open baseline file: " + path.string());
  }
  return nlohmann::json::parse(ifs);
}

} // namespace baselines

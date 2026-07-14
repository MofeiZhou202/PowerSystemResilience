#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace hacdcpf::analysis {

enum class WeakLinkDecisionMode { Planning, Operation };

struct WeakLinkDimensionEvidence {
  std::optional<double> pressure;
  std::string detail;
};

struct WeakLinkEntityEvidence {
  std::string key;
  std::string name;
  std::string canvas_type;
  std::string comparison_group;
  int canvas_index{-1};
  std::string domain;
  int primary_bus{-1};
  int secondary_bus{-1};
  std::array<WeakLinkDimensionEvidence, 4> dimensions;
};

struct WeakLinkPeriodEvidence {
  int step{0};
  double hour{0.0};
  std::array<WeakLinkDimensionEvidence, 4> dimensions;
};

struct MultidimensionalWeakLinkOptions {
  WeakLinkDecisionMode mode{WeakLinkDecisionMode::Planning};
  std::array<double, 4> target_pressure{1.0, 1.0, 1.0, 1.0};
  int top_k{15};
  int minimum_dimensions{1};
  double high_pressure_threshold{0.75};
  double compatibility_threshold{0.50};
  double conflict_gap_threshold{0.60};
};

struct WeakLinkDimensionScore {
  bool available{false};
  double raw_pressure{0.0};
  double normalized_pressure{0.0};
  std::string detail;
};

struct WeakLinkEntityResult {
  std::string key;
  std::string name;
  std::string canvas_type;
  std::string comparison_group;
  int canvas_index{-1};
  std::string domain;
  int primary_bus{-1};
  int secondary_bus{-1};
  std::array<WeakLinkDimensionScore, 4> dimensions;
  int evidence_dimensions{0};
  double severity{0.0};
  double consensus{0.0};
  double disagreement{0.0};
  bool pareto{false};
  std::string dominant_dimension;
  std::string relation;
  std::string decision_hint;
};

struct WeakLinkPeriodResult {
  int step{0};
  double hour{0.0};
  std::array<WeakLinkDimensionScore, 4> dimensions;
  int evidence_dimensions{0};
  double severity{0.0};
  double consensus{0.0};
  std::string dominant_dimension;
};

struct MultidimensionalWeakLinkResult {
  std::string schema{"hacdcpf.multidimensional_weak_link.v1"};
  WeakLinkDecisionMode mode{WeakLinkDecisionMode::Planning};
  std::array<bool, 4> dimension_coverage{false, false, false, false};
  std::vector<WeakLinkEntityResult> entities;
  std::vector<WeakLinkPeriodResult> critical_periods;
  int pareto_count{0};
  int compatible_count{0};
  int conflict_count{0};
  int insufficient_count{0};
};

const char* weak_link_dimension_name(int dimension) noexcept;
const char* weak_link_mode_name(WeakLinkDecisionMode mode) noexcept;

MultidimensionalWeakLinkResult run_multidimensional_weak_link_assessment(
    const std::vector<WeakLinkEntityEvidence>& entities,
    const std::vector<WeakLinkPeriodEvidence>& periods,
    const MultidimensionalWeakLinkOptions& options = {});

}  // namespace hacdcpf::analysis

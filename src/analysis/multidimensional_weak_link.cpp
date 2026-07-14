#include "hacdcpf/analysis/multidimensional_weak_link.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hacdcpf::analysis {
namespace {

constexpr int kDimensionCount = 4;

double normalized_pressure(const WeakLinkDimensionEvidence& evidence,
                           double target) {
  if (!evidence.pressure || !std::isfinite(*evidence.pressure)) return 0.0;
  const double raw = std::max(0.0, *evidence.pressure);
  return raw / ((std::isfinite(target) && target > 0.0) ? target : 1.0);
}

template <typename Result, typename Evidence>
void populate_scores(Result& result, const Evidence& evidence,
                     const MultidimensionalWeakLinkOptions& options) {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = 0.0;
  for (int d = 0; d < kDimensionCount; ++d) {
    const auto& input = evidence.dimensions[static_cast<size_t>(d)];
    auto& output = result.dimensions[static_cast<size_t>(d)];
    output.available = input.pressure && std::isfinite(*input.pressure);
    output.raw_pressure = output.available ? std::max(0.0, *input.pressure) : 0.0;
    output.normalized_pressure = normalized_pressure(
        input, options.target_pressure[static_cast<size_t>(d)]);
    output.detail = input.detail;
    if (!output.available) continue;
    ++result.evidence_dimensions;
    minimum = std::min(minimum, output.normalized_pressure);
    if (output.normalized_pressure > maximum) {
      maximum = output.normalized_pressure;
      result.dominant_dimension = weak_link_dimension_name(d);
    }
  }
  result.severity = maximum;
  result.consensus = result.evidence_dimensions > 0 ? minimum : 0.0;
}

unsigned coverage_mask(const WeakLinkEntityResult& row) {
  unsigned mask = 0;
  for (int d = 0; d < kDimensionCount; ++d) {
    if (row.dimensions[static_cast<size_t>(d)].available) mask |= (1u << d);
  }
  return mask;
}

bool dominates(const WeakLinkEntityResult& lhs,
               const WeakLinkEntityResult& rhs) {
  if (coverage_mask(lhs) != coverage_mask(rhs)) return false;
  bool strictly_greater = false;
  for (int d = 0; d < kDimensionCount; ++d) {
    const auto& a = lhs.dimensions[static_cast<size_t>(d)];
    const auto& b = rhs.dimensions[static_cast<size_t>(d)];
    if (!a.available) continue;
    if (a.normalized_pressure + 1e-12 < b.normalized_pressure) return false;
    strictly_greater = strictly_greater ||
        a.normalized_pressure > b.normalized_pressure + 1e-12;
  }
  return strictly_greater;
}

std::string decision_hint(const WeakLinkEntityResult& row,
                          WeakLinkDecisionMode mode) {
  const auto score = [&](int d) {
    return row.dimensions[static_cast<size_t>(d)].normalized_pressure;
  };
  const double operations = std::max(score(0), score(1));
  const double security = std::max(score(2), score(3));
  if (row.relation == "insufficient_evidence") return "complete_evidence";
  if (mode == WeakLinkDecisionMode::Operation) {
    if (row.relation == "compatible") return "coordinated_dispatch";
    if (security > operations + 0.25) return "emergency_reconfiguration";
    if (operations > security + 0.25) return "redispatch_and_demand_response";
    if (row.relation == "conflict") return "operational_pareto_review";
    return "enhanced_monitoring";
  }
  if (row.relation == "compatible") return "integrated_upgrade";
  if (security > operations + 0.25) return "hardening_and_redundancy";
  if (operations > security + 0.25) return "dispatch_and_loss_reduction";
  if (row.relation == "conflict") return "pareto_tradeoff_review";
  return "monitor_and_targeted_upgrade";
}

}  // namespace

const char* weak_link_dimension_name(int dimension) noexcept {
  static constexpr const char* kNames[] = {
      "economic", "carbon", "reliability", "resilience"};
  return dimension >= 0 && dimension < kDimensionCount ? kNames[dimension]
                                                       : "unknown";
}

const char* weak_link_mode_name(WeakLinkDecisionMode mode) noexcept {
  return mode == WeakLinkDecisionMode::Operation ? "operation" : "planning";
}

MultidimensionalWeakLinkResult run_multidimensional_weak_link_assessment(
    const std::vector<WeakLinkEntityEvidence>& entities,
    const std::vector<WeakLinkPeriodEvidence>& periods,
    const MultidimensionalWeakLinkOptions& options) {
  MultidimensionalWeakLinkResult result;
  result.mode = options.mode;
  result.entities.reserve(entities.size());

  for (const auto& input : entities) {
    WeakLinkEntityResult row;
    row.key = input.key;
    row.name = input.name;
    row.canvas_type = input.canvas_type;
    row.canvas_index = input.canvas_index;
    row.domain = input.domain;
    row.primary_bus = input.primary_bus;
    row.secondary_bus = input.secondary_bus;
    populate_scores(row, input, options);
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = 0.0;
    for (int d = 0; d < kDimensionCount; ++d) {
      const auto& score = row.dimensions[static_cast<size_t>(d)];
      if (!score.available) continue;
      result.dimension_coverage[static_cast<size_t>(d)] = true;
      minimum = std::min(minimum, score.normalized_pressure);
      maximum = std::max(maximum, score.normalized_pressure);
    }
    row.disagreement = row.evidence_dimensions > 1 ? maximum - minimum : 0.0;
    if (row.evidence_dimensions < std::max(1, options.minimum_dimensions)) {
      row.relation = "insufficient_evidence";
      ++result.insufficient_count;
    } else if (row.evidence_dimensions >= 3 &&
               row.consensus >= options.compatibility_threshold) {
      row.relation = "compatible";
      ++result.compatible_count;
    } else if (row.evidence_dimensions >= 2 &&
               row.severity >= options.high_pressure_threshold &&
               row.disagreement >= options.conflict_gap_threshold) {
      row.relation = "conflict";
      ++result.conflict_count;
    } else if (row.severity >= options.high_pressure_threshold) {
      row.relation = "single_dimension";
    } else {
      row.relation = "watch";
    }
    row.decision_hint = decision_hint(row, options.mode);
    result.entities.push_back(std::move(row));
  }

  for (size_t i = 0; i < result.entities.size(); ++i) {
    auto& row = result.entities[i];
    if (row.evidence_dimensions < std::max(1, options.minimum_dimensions)) continue;
    row.pareto = true;
    for (size_t j = 0; j < result.entities.size(); ++j) {
      if (i != j && dominates(result.entities[j], row)) {
        row.pareto = false;
        break;
      }
    }
    if (row.pareto) ++result.pareto_count;
  }

  std::stable_sort(result.entities.begin(), result.entities.end(),
                   [](const auto& a, const auto& b) {
    if (a.pareto != b.pareto) return a.pareto > b.pareto;
    if (std::abs(a.severity - b.severity) > 1e-12)
      return a.severity > b.severity;
    if (std::abs(a.consensus - b.consensus) > 1e-12)
      return a.consensus > b.consensus;
    return a.key < b.key;
  });
  if (options.top_k > 0 &&
      result.entities.size() > static_cast<size_t>(options.top_k)) {
    result.entities.resize(static_cast<size_t>(options.top_k));
  }

  result.critical_periods.reserve(periods.size());
  for (const auto& input : periods) {
    WeakLinkPeriodResult row;
    row.step = input.step;
    row.hour = input.hour;
    populate_scores(row, input, options);
    result.critical_periods.push_back(std::move(row));
  }
  std::stable_sort(result.critical_periods.begin(), result.critical_periods.end(),
                   [](const auto& a, const auto& b) {
    if (std::abs(a.severity - b.severity) > 1e-12)
      return a.severity > b.severity;
    return a.hour < b.hour;
  });
  if (options.top_k > 0 &&
      result.critical_periods.size() > static_cast<size_t>(options.top_k)) {
    result.critical_periods.resize(static_cast<size_t>(options.top_k));
  }
  return result;
}

}  // namespace hacdcpf::analysis

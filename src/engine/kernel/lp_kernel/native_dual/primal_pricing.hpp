#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "model.hpp"

namespace mipsolvers::engine::native_dual::detail {

struct PrimalWeightChange {
  int col{-1};
  double value{1.0};
};

struct PrimalDevexFramework {
  std::vector<double> weight;
  std::vector<char> reference;
  int bad_weight_count{0};
  int iterations{0};
};

inline double primal_pricing_measure(double gain, double weight) {
  if (!(gain > 0.0) || !(weight > 0.0) || !std::isfinite(gain) ||
      !std::isfinite(weight)) {
    return 0.0;
  }
  return gain * gain / weight;
}

inline double primal_exact_edge_weight(const IndexedVector& direction) {
  long double squared_norm = 0.0L;
  for (double value : direction.value) {
    squared_norm += static_cast<long double>(value) * value;
  }
  const long double weight = 1.0L + squared_norm;
  return weight <= std::numeric_limits<double>::max()
             ? static_cast<double>(weight)
             : std::numeric_limits<double>::infinity();
}

inline double primal_pse_updated_weight(double old_weight, double lambda,
                                        double mu,
                                        double direction_squared_norm) {
  const long double l = lambda;
  const long double value =
      static_cast<long double>(old_weight) - 2.0L * l * mu + l * l *
          (static_cast<long double>(direction_squared_norm) + 1.0L);
  return std::abs(value) <= std::numeric_limits<double>::max()
             ? static_cast<double>(value)
             : std::numeric_limits<double>::infinity();
}

inline double primal_pse_leaving_weight(double direction_squared_norm,
                                        double pivot) {
  if (pivot == 0.0 || !std::isfinite(pivot)) {
    return std::numeric_limits<double>::infinity();
  }
  const long double denominator = static_cast<long double>(pivot) * pivot;
  const long double value =
      (1.0L + static_cast<long double>(direction_squared_norm)) / denominator;
  return value <= std::numeric_limits<double>::max()
             ? static_cast<double>(value)
             : std::numeric_limits<double>::infinity();
}

inline bool prepare_primal_pse_update(
    const std::vector<double>& weight, int entering_col, int leaving_col,
    double pivot, double direction_squared_norm,
    const IndexedVector& tableau_row, const IndexedVector& cross_products,
    std::vector<PrimalWeightChange>& changes) {
  changes.clear();
  if (entering_col < 0 || leaving_col < 0 || pivot == 0.0 ||
      !std::isfinite(direction_squared_norm)) {
    return false;
  }
  changes.reserve(tableau_row.index.size() + 2);
  for (std::size_t k = 0; k < tableau_row.index.size(); ++k) {
    const int col = tableau_row.index[k];
    if (col < 0 || col >= static_cast<int>(weight.size()) ||
        col == entering_col || col == leaving_col) {
      continue;
    }
    const double lambda = tableau_row.value[k] / pivot;
    const double updated = primal_pse_updated_weight(
        weight[static_cast<std::size_t>(col)], lambda,
        cross_products.at(col), direction_squared_norm);
    const double minimum = 1.0 + lambda * lambda;
    if (!std::isfinite(updated) || !std::isfinite(minimum)) {
      changes.clear();
      return false;
    }
    changes.push_back({col, std::max(updated, minimum)});
  }
  const double leaving_weight =
      primal_pse_leaving_weight(direction_squared_norm, pivot);
  if (!std::isfinite(leaving_weight)) {
    changes.clear();
    return false;
  }
  changes.push_back({leaving_col, leaving_weight});
  changes.push_back({entering_col, 1.0});
  return true;
}

inline void initialize_primal_devex(int n, const std::vector<char>& basic,
                                    PrimalDevexFramework& framework) {
  framework.weight.assign(static_cast<std::size_t>(n), 1.0);
  framework.reference.assign(static_cast<std::size_t>(n), 0);
  for (int col = 0; col < n; ++col) {
    if (col >= static_cast<int>(basic.size()) ||
        !basic[static_cast<std::size_t>(col)]) {
      framework.reference[static_cast<std::size_t>(col)] = 1;
    }
  }
  framework.bad_weight_count = 0;
  framework.iterations = 0;
}

inline double primal_devex_pivot_reference_weight(
    const PrimalDevexFramework& framework, const std::vector<int>& basis,
    int entering_col, const IndexedVector& direction) {
  if (entering_col < 0 ||
      entering_col >= static_cast<int>(framework.reference.size()) ||
      direction.dimension != static_cast<int>(basis.size())) {
    return std::numeric_limits<double>::infinity();
  }
  long double weight =
      framework.reference[static_cast<std::size_t>(entering_col)] ? 1.0L : 0.0L;
  for (std::size_t k = 0; k < direction.index.size(); ++k) {
    const int row = direction.index[k];
    if (row < 0 || row >= static_cast<int>(basis.size())) {
      return std::numeric_limits<double>::infinity();
    }
    const int basic_col = basis[static_cast<std::size_t>(row)];
    if (basic_col < 0 ||
        basic_col >= static_cast<int>(framework.reference.size())) {
      return std::numeric_limits<double>::infinity();
    }
    if (!framework.reference[static_cast<std::size_t>(basic_col)]) continue;
    const long double value = direction.value[k];
    weight += value * value;
  }
  return weight > 0.0L && weight <= std::numeric_limits<double>::max()
             ? static_cast<double>(weight)
             : std::numeric_limits<double>::infinity();
}

inline bool prepare_primal_devex_update(
    const PrimalDevexFramework& framework, int entering_col, int leaving_col,
    double pivot, double pivot_reference_weight,
    const IndexedVector& tableau_row, std::vector<PrimalWeightChange>& changes) {
  changes.clear();
  if (entering_col < 0 || leaving_col < 0 || pivot == 0.0 ||
      !std::isfinite(pivot_reference_weight) ||
      !(pivot_reference_weight > 0.0) ||
      framework.weight.size() != framework.reference.size()) {
    return false;
  }
  const long double normalized =
      static_cast<long double>(pivot_reference_weight) /
      (static_cast<long double>(pivot) * pivot);
  if (!(normalized > 0.0L) ||
      normalized > std::numeric_limits<double>::max()) {
    return false;
  }
  changes.reserve(tableau_row.index.size() + 2);
  for (std::size_t k = 0; k < tableau_row.index.size(); ++k) {
    const int col = tableau_row.index[k];
    if (col < 0 || col >= static_cast<int>(framework.weight.size()) ||
        col == entering_col || col == leaving_col) {
      continue;
    }
    const long double rho = tableau_row.value[k];
    const long double reference =
        framework.reference[static_cast<std::size_t>(col)] ? 1.0L : 0.0L;
    const long double candidate = reference + normalized * rho * rho;
    const long double updated =
        std::max<long double>(framework.weight[static_cast<std::size_t>(col)],
                              candidate);
    if (!(updated > 0.0L) || updated > std::numeric_limits<double>::max()) {
      changes.clear();
      return false;
    }
    changes.push_back({col, static_cast<double>(updated)});
  }
  changes.push_back(
      {leaving_col, static_cast<double>(std::max<long double>(1.0L, normalized))});
  changes.push_back({entering_col, 1.0});
  return true;
}

inline void commit_primal_weight_changes(
    PrimalDevexFramework& framework,
    const std::vector<PrimalWeightChange>& changes) {
  for (const PrimalWeightChange& change : changes) {
    framework.weight[static_cast<std::size_t>(change.col)] = change.value;
  }
  ++framework.iterations;
}

inline double primal_relative_pivot(double pivot,
                                    const IndexedVector& direction) {
  double norm_inf = 0.0;
  for (double value : direction.value) {
    norm_inf = std::max(norm_inf, std::abs(value));
  }
  return std::abs(pivot) / std::max(1.0, norm_inf);
}

inline bool primal_relative_pivot_acceptable(double pivot,
                                             const IndexedVector& direction) {
  return std::isfinite(pivot) &&
         primal_relative_pivot(pivot, direction) >
             std::sqrt(std::numeric_limits<double>::epsilon());
}

inline bool primal_pivot_identity_acceptable(double column_pivot,
                                             double row_pivot) {
  if (!std::isfinite(column_pivot) || !std::isfinite(row_pivot)) return false;
  const double scale = std::max(std::abs(column_pivot), std::abs(row_pivot));
  return scale > 0.0 &&
         std::abs(column_pivot - row_pivot) <=
             std::sqrt(std::numeric_limits<double>::epsilon()) * scale;
}

}  // namespace mipsolvers::engine::native_dual::detail

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace mipsolvers::engine::detail {

/// Two-component finite activity sum. Products are accumulated together with
/// their FMA residual so cancellation does not erase small row contributions.
class StableActivitySum {
 public:
  void reset() {
    sum_ = 0.0;
    correction_ = 0.0;
    magnitude_ = 0.0;
  }

  void add(double value) {
    add_component(value);
    magnitude_ += std::abs(value);
  }

  void add_product(double coefficient, double bound) {
    const double product = coefficient * bound;
    const double product_error = std::fma(coefficient, bound, -product);
    add_component(product);
    add_component(product_error);
    magnitude_ += std::abs(product) + std::abs(product_error);
  }

  void remove_product(double coefficient, double bound) {
    const double product = coefficient * bound;
    const double product_error = std::fma(coefficient, bound, -product);
    add_component(-product);
    add_component(-product_error);
    magnitude_ = std::max(
        0.0, magnitude_ - std::abs(product) - std::abs(product_error));
  }

  void replace_product(double coefficient, double old_bound,
                       double new_bound) {
    remove_product(coefficient, old_bound);
    add_product(coefficient, new_bound);
  }

  double value() const { return sum_ + correction_; }
  double magnitude() const { return magnitude_; }

  double replacing_product(double coefficient, double old_bound,
                           double new_bound) const {
    StableActivitySum replaced = *this;
    replaced.replace_product(coefficient, old_bound, new_bound);
    return replaced.value();
  }

  double upper_residual(double rhs, double coefficient, double bound,
                        double base_tolerance) const {
    StableActivitySum excluded = *this;
    excluded.remove_product(coefficient, bound);
    const auto residual = subtract(rhs, excluded);
    return residual.value() +
           roundoff_tolerance(base_tolerance, residual.value(),
                              magnitude_ + std::abs(rhs));
  }

  double lower_residual(double lhs, double coefficient, double bound,
                        double base_tolerance) const {
    StableActivitySum excluded = *this;
    excluded.remove_product(coefficient, bound);
    const auto residual = subtract(lhs, excluded);
    return residual.value() -
           roundoff_tolerance(base_tolerance, residual.value(),
                              magnitude_ + std::abs(lhs));
  }

  bool violates_upper(double rhs, double base_tolerance) const {
    const auto difference = subtract(*this, rhs);
    return difference.value() >
           roundoff_tolerance(base_tolerance, difference.value(),
                              magnitude_ + std::abs(rhs));
  }

  bool violates_lower(double lhs, double base_tolerance) const {
    const auto difference = subtract(lhs, *this);
    return difference.value() >
           roundoff_tolerance(base_tolerance, difference.value(),
                              magnitude_ + std::abs(lhs));
  }

  bool within_upper(double rhs, double base_tolerance) const {
    return !violates_upper(rhs, base_tolerance);
  }

  bool within_lower(double lhs, double base_tolerance) const {
    return !violates_lower(lhs, base_tolerance);
  }

  double comparison_tolerance(double rhs, double base_tolerance) const {
    const auto difference = subtract(*this, rhs);
    return roundoff_tolerance(base_tolerance, difference.value(),
                              magnitude_ + std::abs(rhs));
  }

 private:
  void add_component(double value) {
    if (!std::isfinite(value) || !std::isfinite(sum_) ||
        !std::isfinite(correction_)) {
      sum_ = std::numeric_limits<double>::quiet_NaN();
      correction_ = 0.0;
      magnitude_ = std::numeric_limits<double>::infinity();
      return;
    }
    const double next = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
      correction_ += (sum_ - next) + value;
    } else {
      correction_ += (value - next) + sum_;
    }
    sum_ = next;
  }

  static StableActivitySum subtract(double lhs,
                                    const StableActivitySum& rhs) {
    StableActivitySum result;
    result.add(lhs);
    result.add(-rhs.sum_);
    result.add(-rhs.correction_);
    return result;
  }

  static StableActivitySum subtract(const StableActivitySum& lhs,
                                    double rhs) {
    StableActivitySum result = lhs;
    result.add(-rhs);
    return result;
  }

  static double roundoff_tolerance(double base_tolerance, double residual,
                                   double input_magnitude) {
    const double epsilon = std::numeric_limits<double>::epsilon();
    const double second_order =
        64.0 * epsilon * epsilon * std::max(1.0, input_magnitude);
    const double result_roundoff =
        4.0 * epsilon * std::max(1.0, std::abs(residual));
    return std::max(std::abs(base_tolerance),
                    second_order + result_roundoff);
  }

  double sum_{0.0};
  double correction_{0.0};
  double magnitude_{0.0};
};

inline double bound_improvement_tolerance(double base_tolerance,
                                          double current_bound,
                                          double candidate_bound) {
  if (!std::isfinite(current_bound) || !std::isfinite(candidate_bound)) {
    return std::abs(base_tolerance);
  }
  const double scale =
      std::max({1.0, std::abs(current_bound), std::abs(candidate_bound)});
  return std::max(std::abs(base_tolerance),
                  4.0 * std::numeric_limits<double>::epsilon() * scale);
}

}  // namespace mipsolvers::engine::detail

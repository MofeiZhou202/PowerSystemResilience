#include "certificate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mipsolvers::engine::native_dual::detail {
namespace {

Certificate certificate_from_multiplier(const State& state,
                                        Eigen::VectorXd multiplier) {
  Certificate certificate;
  if (multiplier.size() != state.m || !multiplier.allFinite()) {
    return certificate;
  }
  certificate.multiplier = std::move(multiplier);
  certificate.rhs = certificate.multiplier.dot(state.sf->b);
  certificate.attainable_lower = 0.0;
  certificate.attainable_upper = 0.0;
  double absolute_sum = std::abs(certificate.rhs);
  const Bounds original = make_phase_two_bounds(*state.sf);
  for (int j = 0; j < state.n; ++j) {
    double coefficient = 0.0;
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A, j); it;
         ++it) {
      coefficient += certificate.multiplier[it.row()] * it.value();
    }
    const double lower = original.lower[j];
    const double upper = original.upper[j];
    if (coefficient >= 0.0) {
      certificate.attainable_lower += coefficient * lower;
      if (std::isfinite(upper)) {
        certificate.attainable_upper += coefficient * upper;
        absolute_sum += std::abs(coefficient * upper);
      } else if (coefficient > 0.0) {
        certificate.attainable_upper =
            std::numeric_limits<double>::infinity();
      }
    } else {
      certificate.attainable_upper += coefficient * lower;
      if (std::isfinite(upper)) {
        certificate.attainable_lower += coefficient * upper;
        absolute_sum += std::abs(coefficient * upper);
      } else {
        certificate.attainable_lower =
            -std::numeric_limits<double>::infinity();
      }
    }
  }
  certificate.error_bound =
      1024.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, absolute_sum);
  const double below = certificate.attainable_lower - certificate.rhs;
  const double above = certificate.rhs - certificate.attainable_upper;
  if (above > below) {
    certificate.multiplier = -certificate.multiplier;
    certificate.rhs = -certificate.rhs;
    const double old_lower = certificate.attainable_lower;
    certificate.attainable_lower = -certificate.attainable_upper;
    certificate.attainable_upper = -old_lower;
    certificate.margin = above;
  } else {
    certificate.margin = below;
  }
  certificate.valid = certificate.multiplier.allFinite() &&
                      std::isfinite(certificate.rhs) &&
                      certificate.margin > certificate.error_bound;
  return certificate;
}

}  // namespace

Certificate primal_infeasibility_certificate(const State& state,
                                              const Leaving& leaving) {
  return certificate_from_multiplier(state, leaving.row_ep);
}

Certificate phase_one_farkas_certificate(const State& state) {
  if (state.factor == nullptr || state.cost.size() != state.n ||
      static_cast<int>(state.basis.size()) != state.m) {
    return {};
  }
  Eigen::VectorXd basic_cost(state.m);
  for (int row = 0; row < state.m; ++row) {
    basic_cost[row] =
        state.cost[state.basis[static_cast<std::size_t>(row)]];
  }
  return certificate_from_multiplier(state, state.factor->btran(basic_cost));
}

}  // namespace mipsolvers::engine::native_dual::detail

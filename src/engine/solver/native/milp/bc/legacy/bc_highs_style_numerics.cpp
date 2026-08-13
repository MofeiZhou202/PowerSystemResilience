/// @file bc_highs_style_numerics.cpp
/// @brief Definitions for HiGHS-style numeric utilities used by the B&C engine.

#include "mipsolvers/engine/detail/bc_highs_style_numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mipsolvers/engine/detail/bc_types.hpp"  // kInf, is_integer_type
#include "util/HighsHash.h"                        // HighsHashHelpers, HighsInt

namespace mipsolvers::engine::detail {

ChangedColSides collect_changed_col_sides_with_tol(
    const Eigen::VectorXd& old_lb,
    const Eigen::VectorXd& old_ub,
    const Eigen::VectorXd& new_lb,
    const Eigen::VectorXd& new_ub,
    double tol) {
  ChangedColSides changed;
  if (old_lb.size() != new_lb.size() || old_ub.size() != new_ub.size()) {
    return changed;
  }
  changed.any.reserve(static_cast<std::size_t>(old_lb.size()));
  const double effective_tol = std::max(0.0, tol);
  for (int j = 0; j < old_lb.size(); ++j) {
    bool col_changed = false;
    const bool lower_tightened =
        (std::isfinite(new_lb[j]) && !std::isfinite(old_lb[j])) ||
        (std::isfinite(new_lb[j]) && std::isfinite(old_lb[j]) &&
         new_lb[j] - old_lb[j] > effective_tol);
    if (lower_tightened) {
      changed.lower.push_back(j);
      col_changed = true;
    }
    const bool upper_tightened =
        (std::isfinite(new_ub[j]) && !std::isfinite(old_ub[j])) ||
        (std::isfinite(new_ub[j]) && std::isfinite(old_ub[j]) &&
         old_ub[j] - new_ub[j] > effective_tol);
    if (upper_tightened) {
      changed.upper.push_back(j);
      col_changed = true;
    }
    if (col_changed) changed.any.push_back(j);
  }
  return changed;
}

std::int64_t highs_style_gcd(std::int64_t a, std::int64_t b) {
  if (a < 0) a = -a;
  if (b < 0) b = -b;
  if (a == 0) return b;
  if (b == 0) return a;
  while (b != 0) {
    const std::int64_t h = a % b;
    a = b;
    b = h;
  }
  return a;
}

std::uint64_t highs_style_pair_hash(std::uint32_t a,
                                    std::uint32_t b,
                                    int k) {
  static constexpr std::uint64_t c[] = {
      std::uint64_t{0xc8497d2a400d9551},
      std::uint64_t{0x80c8963be3e4c2f3},
      std::uint64_t{0x042d8680e260ae5b},
      std::uint64_t{0x8a183895eeac1536},
  };
  const int idx = 2 * k;
  return (static_cast<std::uint64_t>(a) + c[idx]) *
         (static_cast<std::uint64_t>(b) + c[idx + 1]);
}

std::uint64_t highs_style_hash_u64(std::uint64_t value) {
  const auto lo = static_cast<std::uint32_t>(value & 0xffffffffu);
  const auto hi = static_cast<std::uint32_t>(value >> 32);
  return highs_style_pair_hash(lo, hi, 1) ^
         (highs_style_pair_hash(lo, hi, 0) >> 32);
}

std::uint64_t highs_style_cut_hash(const Eigen::SparseVector<double>& coeff,
                                   double maxabscoef) {
  if (!(maxabscoef > 0.0) || !std::isfinite(maxabscoef)) return 0;
  std::vector<HighsInt> indices;
  std::vector<std::uint32_t> value_hash_codes;
  indices.reserve(static_cast<std::size_t>(coeff.nonZeros()));
  value_hash_codes.reserve(static_cast<std::size_t>(coeff.nonZeros()));
  const double scale = 1.0 / maxabscoef;
  for (Eigen::SparseVector<double>::InnerIterator it(coeff); it; ++it) {
    indices.push_back(static_cast<HighsInt>(it.index()));
    value_hash_codes.push_back(
        HighsHashHelpers::double_hash_code(scale * it.value()));
  }
  return HighsHashHelpers::vector_hash(indices.data(), indices.size()) ^
         (HighsHashHelpers::vector_hash(value_hash_codes.data(),
                                        value_hash_codes.size()) >>
          32);
}

std::int64_t highs_style_denominator(double x,
                                     double eps,
                                     std::int64_t max_denom) {
  std::int64_t ai = static_cast<std::int64_t>(x);
  std::int64_t m[] = {ai, 1, 1, 0};
  long double xi = x;
  long double fraction = xi - static_cast<double>(ai);
  while (fraction > eps) {
    xi = 1.0L / fraction;
    if (static_cast<double>(xi) > static_cast<double>(std::int64_t{1} << 53)) {
      break;
    }
    ai = static_cast<std::int64_t>(static_cast<double>(xi));
    std::int64_t t = m[2] * ai + m[3];
    if (t > max_denom) break;
    m[3] = m[2];
    m[2] = t;
    t = m[0] * ai + m[1];
    m[1] = m[0];
    m[0] = t;
    fraction = xi - static_cast<double>(ai);
  }
  ai = (max_denom - m[3]) / m[2];
  m[1] += m[0] * ai;
  m[3] += m[2] * ai;
  x = std::abs(x);
  const double x0 = static_cast<double>(m[0]) / static_cast<double>(m[2]);
  const double x1 = static_cast<double>(m[1]) / static_cast<double>(m[3]);
  return std::abs(x - x0) < std::abs(x - x1) ? m[2] : m[3];
}

double highs_style_integral_scale(const std::vector<double>& vals,
                                  double eps) {
  if (vals.empty()) return 0.0;
  auto minmax = std::minmax_element(
      vals.begin(), vals.end(),
      [](double a, double b) { return std::abs(a) < std::abs(b); });
  const double minval = *minmax.first;
  const double maxval = *minmax.second;

  int expshift = 0;
  if (minval < -eps || minval > eps) std::frexp(minval, &expshift);
  expshift = std::max(-expshift, 0) + 3;

  int exp_max_val = 0;
  std::frexp(maxval, &exp_max_val);
  exp_max_val = std::min(exp_max_val, 32);
  if (exp_max_val + expshift > 32) expshift = 32 - exp_max_val;
  if (expshift < 0) expshift = 0;

  std::uint64_t denom = std::uint64_t{75} << expshift;
  std::uint64_t start_denom = denom;
  auto scaled_floor_fraction =
      [&](double value, std::uint64_t scale, long double& downval) {
        const long double scaled =
            static_cast<long double>(scale) * static_cast<long double>(value);
        downval = std::floor(scaled + eps);
        return scaled - downval;
      };

  long double downval = 0.0L;
  long double fraction = scaled_floor_fraction(vals.front(), denom, downval);
  if (fraction > eps) {
    denom *= static_cast<std::uint64_t>(
        highs_style_denominator(static_cast<double>(fraction), eps, 1000));
    fraction = scaled_floor_fraction(vals.front(), denom, downval);
    if (fraction > eps) return 0.0;
  }

  auto abs_int64_from_floor = [](long double value) -> std::int64_t {
    if (value < 0.0L) value = -value;
    if (value > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      return std::numeric_limits<std::int64_t>::max();
    }
    return static_cast<std::int64_t>(static_cast<double>(value));
  };

  std::uint64_t curr_gcd =
      static_cast<std::uint64_t>(abs_int64_from_floor(downval));
  for (std::size_t i = 1; i < vals.size(); ++i) {
    fraction = scaled_floor_fraction(vals[i], denom, downval);
    if (fraction > eps) {
      long double start_down = 0.0L;
      const long double start_scaled =
          static_cast<long double>(start_denom) *
          static_cast<long double>(vals[i]);
      const long double start_fraction =
          start_scaled - std::floor(start_scaled);
      denom *= static_cast<std::uint64_t>(highs_style_denominator(
          static_cast<double>(start_fraction), eps, 1000));
      fraction = scaled_floor_fraction(vals[i], denom, downval);
      (void)start_down;
      if (fraction > eps) return 0.0;
    }
    if (curr_gcd != 1) {
      curr_gcd = static_cast<std::uint64_t>(highs_style_gcd(
          static_cast<std::int64_t>(curr_gcd),
          abs_int64_from_floor(downval)));
      if (denom > std::numeric_limits<unsigned int>::max()) {
        denom /= std::max<std::uint64_t>(1, curr_gcd);
        if (start_denom != 1) {
          start_denom /=
              static_cast<std::uint64_t>(highs_style_gcd(
                  static_cast<std::int64_t>(curr_gcd),
                  static_cast<std::int64_t>(start_denom)));
        }
        curr_gcd = 1;
      }
    }
  }

  return static_cast<double>(denom) /
         static_cast<double>(std::max<std::uint64_t>(1, curr_gcd));
}

double highs_style_objective_integral_scale(const LPModel& lp,
                                            double eps) {
  std::vector<double> objective_vals;
  objective_vals.reserve(static_cast<std::size_t>(lp.c.size()));
  for (std::size_t j = 0;
       j < lp.vars.size() && j < static_cast<std::size_t>(lp.c.size()); ++j) {
    const double cj = lp.c[static_cast<Eigen::Index>(j)];
    if (!std::isfinite(cj) || std::abs(cj) <= 1e-12) continue;
    if (!is_integer_type(lp.vars[j])) return 0.0;
    objective_vals.push_back(cj);
  }
  if (objective_vals.empty()) return 1.0;
  const double scale =
      highs_style_integral_scale(objective_vals, std::max(1e-12, eps));
  if (!(scale > 0.0) || scale * 1e-14 > std::max(1e-12, eps)) {
    return 0.0;
  }
  return scale;
}

double highs_style_incumbent_upper_limit(const LPModel& lp,
                                         double incumbent_obj,
                                         double feastol) {
  if (!std::isfinite(incumbent_obj)) return kInf;
  const double tol = std::max(1e-9, feastol);

  // Mirror HighsMipSolverData::computeNewUpperLimit(ub, 0, 0): the propagation
  // cutoff is the best objective value still worth searching, not the incumbent
  // objective itself. Fractional-but-scalable integer objectives such as UC
  // costs with 0.5 increments must use their integral scale.
  const double integral_scale = highs_style_objective_integral_scale(lp, tol);
  if (integral_scale > 0.0) {
    return (std::ceil(integral_scale * incumbent_obj - tol) - 1.0) /
               integral_scale +
           tol;
  }
  return std::min(incumbent_obj - tol,
                  std::nextafter(incumbent_obj, -kInf));
}

bool highs_style_root_source_eligible(int declared_integer_cols,
                                      int implied_integer_cols,
                                      double root_bound) {
  // Achterberg (2007), Constraint Integer Programming, Secs. 4.1-4.2:
  // tableau/path disjunctions arise from every fractional integer variable;
  // presolve-inferred integrality only enlarges that set.
  return std::isfinite(root_bound) &&
         (declared_integer_cols > 0 || implied_integer_cols > 0);
}

double presolved_objective_value(double original_objective,
                                 double presolved_objective_offset) {
  // HiGHS HighsLp::offset_: f_original = f_reduced + offset; derivation in
  // docs/native_milp_root_source_eligibility_2026-08-13.md.
  return original_objective - presolved_objective_offset;
}

bool highs_style_map_variable_bound_to_reduced(
    bool& is_upper, double& coef, double& constant, double target_scale,
    double target_shift, double trigger_scale, double trigger_shift) {
  if (!std::isfinite(coef) || !std::isfinite(constant) ||
      !std::isfinite(target_scale) || !std::isfinite(target_shift) ||
      !std::isfinite(trigger_scale) || !std::isfinite(trigger_shift) ||
      std::abs(target_scale) <= 1e-12) {
    return false;
  }
  // HiGHS HighsPostsolveStack::linearTransform and
  // docs/native_milp_presolve_varbound_coordinates_2026-08-13.md:
  // x_t <= a*x_z+b, x_i=s_i*y_i+c_i gives
  // y_t <= (a*s_z/s_t)y_z+(a*c_z+b-c_t)/s_t.
  constant =
      (coef * trigger_shift + constant - target_shift) / target_scale;
  coef = coef * trigger_scale / target_scale;
  if (target_scale < 0.0) is_upper = !is_upper;
  return std::isfinite(coef) && std::isfinite(constant);
}

bool highs_style_cutpool_row_tree_owned(bool alive, bool in_root_lp) {
  // HiGHS HighsCutPool::separate and HighsLpRelaxation::addCuts; derivation in
  // docs/native_milp_cutpool_proof_ownership_2026-08-13.md. A separator row
  // becomes a tree-global proof artifact only after successful LP admission.
  return alive && in_root_lp;
}

bool highs_style_cutpool_candidate_stays_root_owned(
    bool strict_highs_root_fixed_point, bool auto_highs_root_pipeline) {
  // HiGHS HighsCutPool::separate and HighsLpRelaxation::addCuts; derivation in
  // docs/native_milp_cutpool_proof_ownership_2026-08-13.md. Both HiGHS-style
  // execution modes defer tree ownership until root-LP admission.
  return strict_highs_root_fixed_point || auto_highs_root_pipeline;
}

}  // namespace mipsolvers::engine::detail

/// @file bc_highs_style_numerics.hpp
/// @brief HiGHS-style numeric utilities extracted from the legacy B&C engine.
///
/// Pure helpers for domain-change accounting, integral scaling, continued-
/// fraction denominator recovery, cut/value hashing, and incumbent cutoff
/// computation.  These were defined in the anonymous namespace of the
/// monolithic branch_and_cut translation unit; extracting them into a named
/// component makes them independently unit-testable and reusable without
/// altering behaviour (see tests/test_branch_and_cut.cpp).

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

/// @brief Columns whose lower and/or upper bound tightened between two domains.
struct ChangedColSides {
  std::vector<int> any;
  std::vector<int> lower;
  std::vector<int> upper;
};

/// @brief Collect columns whose bounds tightened by more than @p tol.
ChangedColSides collect_changed_col_sides_with_tol(
    const Eigen::VectorXd& old_lb, const Eigen::VectorXd& old_ub,
    const Eigen::VectorXd& new_lb, const Eigen::VectorXd& new_ub, double tol);

std::int64_t highs_style_gcd(std::int64_t a, std::int64_t b);

std::uint64_t highs_style_pair_hash(std::uint32_t a, std::uint32_t b, int k);

std::uint64_t highs_style_hash_u64(std::uint64_t value);

std::uint64_t highs_style_cut_hash(const Eigen::SparseVector<double>& coeff,
                                   double maxabscoef);

std::int64_t highs_style_denominator(double x, double eps,
                                     std::int64_t max_denom);

double highs_style_integral_scale(const std::vector<double>& vals, double eps);

double highs_style_objective_integral_scale(const LPModel& lp, double eps);

double highs_style_incumbent_upper_limit(const LPModel& lp, double incumbent_obj,
                                         double feastol);

/// Whether a finite root relaxation has integer disjunctions available to the
/// transformed-tableau/path separation loop. Inferred integer columns augment
/// the disjunction set; they are not a prerequisite for declared integers.
bool highs_style_root_source_eligible(int declared_integer_cols,
                                      int implied_integer_cols,
                                      double root_bound);

/// Convert an original internal-minimization objective into the presolved
/// objective space used by root bounds.
double presolved_objective_value(double original_objective,
                                 double presolved_objective_offset);

/// Transport x_target (<=/>=) coef*x_trigger+constant through retained affine
/// maps x_i=scale_i*y_i+shift_i. Returns false for a non-invertible mapping.
bool highs_style_map_variable_bound_to_reduced(
    bool& is_upper, double& coef, double& constant, double target_scale,
    double target_shift, double trigger_scale, double trigger_shift);

/// Whether a transformed root cut has crossed the proof-ownership boundary
/// required for export to tree-domain propagation.
bool highs_style_cutpool_row_tree_owned(bool alive, bool in_root_lp);

/// Whether generated transformed cuts remain private to the root cutpool
/// until the LP-ownership test can be applied.
bool highs_style_cutpool_candidate_stays_root_owned(
    bool strict_highs_root_fixed_point, bool auto_highs_root_pipeline);

}  // namespace mipsolvers::engine::detail

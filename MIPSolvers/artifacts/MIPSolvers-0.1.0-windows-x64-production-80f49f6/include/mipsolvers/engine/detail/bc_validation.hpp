/// @file bc_validation.hpp
/// @brief Incumbent validation and diagnostic formatting for B&C.

#pragma once

#include <string>

#include <Eigen/Core>

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/strategy/papilo_presolve.hpp"

namespace mipsolvers::engine::detail {

struct IncumbentValidationSummary {
  bool size_ok{true};
  bool bounds_ok{true};
  bool ineq_ok{true};
  bool eq_ok{true};
  bool integrality_ok{true};
  int expected_size{0};
  int actual_size{0};
  int bound_var{-1};
  int ineq_row{-1};
  int eq_row{-1};
  int frac_var{-1};
  double ineq_row_activity{0.0};
  double ineq_row_lhs{-kInf};
  double ineq_row_rhs{kInf};
  const char* ineq_row_side{"none"};
  double max_bound_violation{0.0};
  double max_ineq_violation{0.0};
  double max_eq_violation{0.0};
  double max_integrality_violation{0.0};

  bool ok() const {
    return size_ok && bounds_ok && ineq_ok && eq_ok && integrality_ok;
  }
};

IncumbentValidationSummary validate_incumbent_solution(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    double tol = 1e-6);

std::string format_incumbent_validation_failure(
    const char* stage,
    const LPModel& lp,
    const IncumbentValidationSummary& summary);

std::string format_ineq_row_contributors(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    int row,
    int max_terms = 8);

#ifdef MIPSOLVERS_HAVE_PAPILO
std::string format_papilo_reduced_row_mapping_detail(
    const PaPILOPresolveResult& ps,
    const LPModel& reduced_lp,
    const Eigen::VectorXd& x_reduced,
    int original_row);

std::string format_papilo_compressed_validation_detail(
    const PaPILOReducedValidationSummary& summary);
#endif

}  // namespace mipsolvers::engine::detail

#pragma once

#include <string>

namespace mipsolvers::aml {

/// Feature flags of a particular solver backend.
struct SolverCapabilities {
  std::string solver_name;

  // Problem classes
  bool supports_lp              = false;
  bool supports_milp            = false;
  bool supports_qp              = false;
  bool supports_qcp             = false;
  bool supports_nlp             = false;
  bool supports_minlp           = false;

  // Constraint extensions
  bool supports_sos1            = false;
  bool supports_sos2            = false;
  bool supports_indicator       = false;
  bool supports_piecewise       = false;   ///< native PWL cost
  bool supports_lazy_cuts       = false;
  bool supports_user_callbacks  = false;

  // Solution information
  bool supports_duals           = false;   ///< LP / QP optimal duals
  bool supports_reduced_costs   = false;
  bool supports_basis           = false;   ///< simplex basis
  bool supports_iis             = false;   ///< Irreducible Infeasible Subsystem
  bool supports_farkas          = false;   ///< infeasibility certificate
  bool supports_mip_duals       = false;   ///< root LP relaxation duals
};

/// Features detected in an AML model at compile time.
struct ProblemFeatures {
  bool has_integer_vars          = false;
  bool has_binary_vars           = false;
  bool has_quadratic_objective   = false;
  bool has_quadratic_constraints = false;
  bool has_nonlinear_objective   = false;
  bool has_nonlinear_constraints = false;
  bool has_indicator_constraints = false;
  bool has_sos1_constraints      = false;
  bool has_sos2_constraints      = false;
};

}  // namespace mipsolvers::aml

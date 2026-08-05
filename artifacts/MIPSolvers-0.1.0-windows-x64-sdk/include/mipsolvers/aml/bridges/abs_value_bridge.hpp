#pragma once

/// AbsValueBridge
/// ==============
/// Replaces  |expr|  with an auxiliary variable  t  by adding:
///
///   t >= expr
///   t >= -expr
///   t >= 0          (implicit from lb=0)
///
/// Usage:
///   auto [t_ref, c_ub, c_lb] = AbsValueBridge::apply(m, expr, "abs_cost");
///
/// The returned VarRef t can then be used directly in an objective or
/// further constraints.

#include <string>
#include <tuple>

#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/model.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml::bridges {

struct AbsValueResult {
  VarRef         aux;       ///< t ≥ |expr|  (auxiliary variable)
  ConstraintRef  c_upper;   ///< t ≥  expr
  ConstraintRef  c_lower;   ///< t ≥ -expr
};

class AbsValueBridge {
 public:
  /// Introduce |expr| into model `m`.
  /// @param m        The model to modify.
  /// @param expr     The expression whose absolute value is needed.
  /// @param name     Base name for the auxiliary variable and constraints.
  /// @param ub       Upper bound hint for the auxiliary variable (default 1e20).
  static AbsValueResult apply(Model& m,
                               const LinearExpr& expr,
                               const std::string& name,
                               double ub = 1e20) {
    // 1-element set so we can use add_var with a set domain
    auto& aux_set = m.add_set("__abs_set_" + name, {name});
    auto& t_arr   = m.add_var("__abs_" + name, aux_set,
                               VarType::Continuous, 0.0, ub);
    VarRef t = t_arr(Key::scalar(name));

    // t >= expr  ↔  t - expr >= 0  ↔  expr - t <= 0
    TempConstr cu = expr - static_cast<LinearExpr>(t) <= 0.0;
    auto c_upper  = m.add_constraint(cu, name + "_abs_upper");

    // t >= -expr  ↔  t + expr >= 0  ↔  -expr - t <= 0
    TempConstr cl = (-expr) - static_cast<LinearExpr>(t) <= 0.0;
    auto c_lower  = m.add_constraint(cl, name + "_abs_lower");

    return {t, c_upper, c_lower};
  }
};

}  // namespace mipsolvers::aml::bridges

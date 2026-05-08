#pragma once

/// IndicatorBridge
/// ===============
/// Big-M encoding of indicator constraints:
///
///   y = 1  →  lhs ≤ rhs      becomes   lhs - rhs ≤ M * (1 - y)
///   y = 1  →  lhs ≥ rhs      becomes   rhs - lhs ≤ M * (1 - y)
///   y = 1  →  lhs = rhs  is split into both of the above.
///
/// y must be a binary variable (type Binary) already in the model.
///
/// Usage:
///   auto c = IndicatorBridge::apply_leq(m, y, p_gen, 0.0, "uc_cap_off");
///   // Adds:  p_gen ≤ M*(1-y)  i.e. p_gen = 0 when unit is off

#include <string>

#include "mipsolvers/aml/constraint.hpp"
#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/model.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml::bridges {

class IndicatorBridge {
 public:
  /// Encode:  y = 1  →  lhs ≤ rhs
  ///   big-M relaxation:  lhs - rhs ≤ M*(1 - y)
  static ConstraintRef apply_leq(Model& m,
                                  VarRef y_binary,
                                  const LinearExpr& lhs,
                                  double rhs,
                                  const std::string& name,
                                  double big_m = 1.0e6) {
    // lhs - rhs ≤ M*(1-y)   →   lhs + M*y ≤ rhs + M
    LinearExpr row = lhs + big_m * static_cast<LinearExpr>(y_binary);
    return m.add_constraint(row <= rhs + big_m, name);
  }

  /// Encode:  y = 1  →  lhs ≥ rhs
  ///   big-M relaxation:  rhs - lhs ≤ M*(1 - y)
  static ConstraintRef apply_geq(Model& m,
                                  VarRef y_binary,
                                  const LinearExpr& lhs,
                                  double rhs,
                                  const std::string& name,
                                  double big_m = 1.0e6) {
    // rhs - lhs ≤ M*(1-y)   →   -lhs + M*y ≤ -rhs + M
    LinearExpr row = (-1.0) * lhs + big_m * static_cast<LinearExpr>(y_binary);
    return m.add_constraint(row <= -rhs + big_m, name);
  }

  /// Encode:  y = 1  →  lhs = rhs  (adds both leq and geq constraints).
  /// Returns the pair {leq_constraint, geq_constraint}.
  static std::pair<ConstraintRef, ConstraintRef>
  apply_eq(Model& m,
           VarRef y_binary,
           const LinearExpr& lhs,
           double rhs,
           const std::string& name,
           double big_m = 1.0e6) {
    return {
      apply_leq(m, y_binary, lhs, rhs, name + "_leq", big_m),
      apply_geq(m, y_binary, lhs, rhs, name + "_geq", big_m)
    };
  }

  /// Convenience: encode  y = 0  →  expr ≤ 0  (force to zero when off).
  /// Equivalent to  (1-y) = 1 → expr ≤ 0, but using y directly:
  ///   expr ≤ M * y
  static ConstraintRef apply_zero_when_off(Model& m,
                                            VarRef y_binary,
                                            const LinearExpr& expr,
                                            const std::string& name,
                                            double big_m = 1.0e6) {
    // expr - M*y ≤ 0
    LinearExpr row = expr + (-big_m) * static_cast<LinearExpr>(y_binary);
    return m.add_constraint(row <= 0.0, name);
  }
};

}  // namespace mipsolvers::aml::bridges

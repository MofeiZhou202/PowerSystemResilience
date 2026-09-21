#pragma once

/// MaxEpigraphBridge
/// =================
/// Introduces an epigraph variable  t  such that:
///
///   t >= expr_i   for each expression expr_i in a list
///
/// Minimizing t (or penalizing it in the objective) is equivalent to
/// minimizing max(expr_1, ..., expr_n).
///
/// Usage:
///   auto result = MaxEpigraphBridge::apply(m, {e1, e2, e3}, "peak_gen");
///   m.minimize(... + penalty * result.aux);

#include <string>
#include <vector>

#include "mipsolvers/aml/constraint.hpp"
#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/model.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml::bridges {

struct MaxEpigraphResult {
  VarRef                        aux;         ///< t ≥ max(expr_i)
  std::vector<ConstraintRef>    constraints; ///< t ≥ expr_i for each i
};

class MaxEpigraphBridge {
 public:
  /// Introduce max(exprs) epigraph into model `m`.
  /// @param m       The model to modify.
  /// @param exprs   Non-empty list of expressions to upper-bound.
  /// @param name    Base name for the auxiliary variable.
  /// @param lb      Lower bound for t (default -1e20; use 0 if all exprs >= 0).
  /// @param ub      Upper bound for t (default 1e20).
  static MaxEpigraphResult apply(Model& m,
                                  const std::vector<LinearExpr>& exprs,
                                  const std::string& name,
                                  double lb = -1e20,
                                  double ub =  1e20) {
    auto& aux_set = m.add_set("__epi_set_" + name, {name});
    auto& t_arr   = m.add_var("__epi_" + name, aux_set,
                               VarType::Continuous, lb, ub);
    VarRef t = t_arr(Key::scalar(name));

    MaxEpigraphResult res;
    res.aux = t;

    for (std::size_t i = 0; i < exprs.size(); ++i) {
      // t >= expr_i  ↔  expr_i - t <= 0
      TempConstr c =
          exprs[i] - static_cast<LinearExpr>(t) <= 0.0;
      auto cref = m.add_constraint(c, name + "_epi_" + std::to_string(i));
      res.constraints.push_back(cref);
    }
    return res;
  }
};

}  // namespace mipsolvers::aml::bridges

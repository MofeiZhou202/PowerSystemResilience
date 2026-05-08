#pragma once

/// PiecewiseLinearBridge
/// =====================
/// Approximates a univariate piecewise-linear function  y = f(x)  as a
/// convex combination of breakpoint values using SOS2 variables.
///
/// Given breakpoints (x_0 < x_1 < ... < x_n) and function values
/// (y_0, y_1, ..., y_n), the bridge introduces:
///
///   λ_k ≥ 0   for k = 0..n            (convex-combination weights)
///   Σ_k λ_k = 1                        (weights sum to 1)
///   x_var = Σ_k x_k * λ_k             (x recovered from weights)
///   y_var = Σ_k y_k * λ_k             (piecewise value)
///   {λ_k} are SOS2                     (at most two adjacent non-zero)
///
/// The SOS2 condition is enforced via binary variables (big-M):
///   λ_k ≤ b_{k-1} + b_k              b ∈ {0,1}^n, Σ b_k = 1
///   This guarantees at most two adjacent λ_k are positive.
///
/// Usage:
///   VarRef x = ...;   // the independent variable
///   auto res = PiecewiseLinearBridge::apply(m, x, {0,1,2,3}, {0,1,4,9}, "pwl_sq");
///   // res.y_var now equals a piecewise-linear approximation of x^2

#include <stdexcept>
#include <string>
#include <vector>

#include "mipsolvers/aml/constraint.hpp"
#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/model.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml::bridges {

struct PiecewiseLinearResult {
  VarRef                     x_var;         ///< input variable (same as provided)
  VarRef                     y_var;         ///< output: y = f(x)
  std::vector<VarRef>        lambda_vars;   ///< SOS2 convex-combination weights
  std::vector<ConstraintRef> constraints;   ///< all added constraints
};

class PiecewiseLinearBridge {
 public:
  /// @param m            Model to add variables/constraints into.
  /// @param x_var        The input variable (must already exist in m).
  /// @param breakpoints  Sorted x-coordinates of breakpoints (at least 2).
  /// @param values       f(breakpoint) values, same length as breakpoints.
  /// @param name         Base name for auxiliary variables.
  /// @param y_lb         Lower bound for y_var (default -1e20).
  /// @param y_ub         Upper bound for y_var (default  1e20).
  static PiecewiseLinearResult apply(Model& m,
                                      VarRef x_var,
                                      const std::vector<double>& breakpoints,
                                      const std::vector<double>& values,
                                      const std::string& name,
                                      double y_lb = -1e20,
                                      double y_ub =  1e20) {
    const int n = static_cast<int>(breakpoints.size());
    if (n < 2)
      throw std::invalid_argument("PiecewiseLinearBridge: need at least 2 breakpoints");
    if (static_cast<int>(values.size()) != n)
      throw std::invalid_argument("PiecewiseLinearBridge: breakpoints/values size mismatch");
    for (int i = 1; i < n; ++i) {
      if (breakpoints[i] <= breakpoints[i-1])
        throw std::invalid_argument("PiecewiseLinearBridge: breakpoints must be strictly increasing");
    }

    PiecewiseLinearResult res;
    res.x_var = x_var;

    // ── λ variables: convex-combination weights (n variables) ──────────
    auto& lam_set = m.add_set("__pwl_lam_set_" + name, {});
    // Add n atoms to the lambda set
    std::vector<std::string> lam_atoms;
    lam_atoms.reserve(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k) lam_atoms.push_back(name + "_k" + std::to_string(k));
    for (const auto& a : lam_atoms) lam_set.add_element(Key::scalar(a));

    auto& lam_arr = m.add_var("__pwl_lam_" + name, lam_set,
                               VarType::Continuous, 0.0, 1.0);
    res.lambda_vars.reserve(static_cast<std::size_t>(n));
    for (int k = 0; k < n; ++k)
      res.lambda_vars.push_back(lam_arr(Key::scalar(lam_atoms[static_cast<std::size_t>(k)])));

    // ── b variables: SOS2 selectors (n-1 binary segments) ──────────────
    auto& b_set = m.add_set("__pwl_b_set_" + name, {});
    std::vector<std::string> b_atoms;
    b_atoms.reserve(static_cast<std::size_t>(n - 1));
    for (int k = 0; k < n - 1; ++k) b_atoms.push_back(name + "_b" + std::to_string(k));
    for (const auto& a : b_atoms) b_set.add_element(Key::scalar(a));

    auto& b_arr = m.add_var("__pwl_b_" + name, b_set, VarType::Binary);

    // ── y output variable ───────────────────────────────────────────────
    auto& y_set = m.add_set("__pwl_y_set_" + name, {name + "_y"});
    auto& y_arr = m.add_var("__pwl_y_" + name, y_set,
                             VarType::Continuous, y_lb, y_ub);
    res.y_var = y_arr(Key::scalar(name + "_y"));

    auto add_con = [&](TempConstr tc, const std::string& cname) {
      res.constraints.push_back(m.add_constraint(tc, cname));
    };

    // ── Constraint: Σ λ_k = 1 ─────────────────────────────────────────
    {
      LinearExpr lam_sum;
      for (auto& lv : res.lambda_vars) lam_sum += static_cast<LinearExpr>(lv);
      add_con(lam_sum == 1.0, name + "_lam_sum");
    }

    // ── Constraint: Σ b_k = 1 ─────────────────────────────────────────
    {
      LinearExpr b_sum;
      for (int k = 0; k < n - 1; ++k)
        b_sum += static_cast<LinearExpr>(b_arr(Key::scalar(b_atoms[static_cast<std::size_t>(k)])));
      add_con(b_sum == 1.0, name + "_b_sum");
    }

    // ── Constraint: x = Σ bp_k * λ_k ─────────────────────────────────
    {
      LinearExpr x_rhs;
      for (int k = 0; k < n; ++k)
        x_rhs += breakpoints[static_cast<std::size_t>(k)] *
                 static_cast<LinearExpr>(res.lambda_vars[static_cast<std::size_t>(k)]);
      add_con(static_cast<LinearExpr>(x_var) == x_rhs, name + "_x_eq");
    }

    // ── Constraint: y = Σ val_k * λ_k ────────────────────────────────
    {
      LinearExpr y_rhs;
      for (int k = 0; k < n; ++k)
        y_rhs += values[static_cast<std::size_t>(k)] *
                 static_cast<LinearExpr>(res.lambda_vars[static_cast<std::size_t>(k)]);
      add_con(static_cast<LinearExpr>(res.y_var) == y_rhs, name + "_y_eq");
    }

    // ── SOS2: λ_k ≤ b_{k-1} + b_k  (b_{-1} = b_{n-1} = 0) ───────────
    // Segment binary b_k covers the interval [bp_k, bp_{k+1}], so:
    //   λ_0 ≤ b_0
    //   λ_k ≤ b_{k-1} + b_k   for k = 1..n-2
    //   λ_{n-1} ≤ b_{n-2}
    for (int k = 0; k < n; ++k) {
      LinearExpr rhs;
      if (k > 0)     rhs += static_cast<LinearExpr>(b_arr(Key::scalar(b_atoms[static_cast<std::size_t>(k-1)])));
      if (k < n - 1) rhs += static_cast<LinearExpr>(b_arr(Key::scalar(b_atoms[static_cast<std::size_t>(k)])));
      add_con(static_cast<LinearExpr>(res.lambda_vars[static_cast<std::size_t>(k)]) <= rhs,
              name + "_sos2_" + std::to_string(k));
    }

    return res;
  }
};

}  // namespace mipsolvers::aml::bridges

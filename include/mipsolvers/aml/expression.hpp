#pragma once

#include <functional>
#include <vector>

#include "mipsolvers/aml/ids.hpp"
#include "mipsolvers/aml/set.hpp"

namespace mipsolvers::aml {

// ── Forward declarations ───────────────────────────────────────────────────
struct TempConstr;
struct QuadExpr;

// ── CompareOp ─────────────────────────────────────────────────────────────
enum class CompareOp { LessEq, GreaterEq, Equal };

// ════════════════════════════════════════════════════════════════════════════
// LinearExpr — sparse affine expression:  constant  +  Σ coef_i * x_i
//
// Internal representation: vector<(VarId, double)> + double constant.
// Duplicate VarIds are permitted; the model compiler aggregates them.
// ════════════════════════════════════════════════════════════════════════════
struct LinearExpr {
  double                              constant = 0.0;
  std::vector<std::pair<VarId, double>> terms;

  // ---- Factories ---------------------------------------------------------
  static LinearExpr from_var(VarId v, double coef = 1.0) {
    LinearExpr e;
    e.terms.emplace_back(v, coef);
    return e;
  }
  static LinearExpr const_expr(double c) {
    LinearExpr e;
    e.constant = c;
    return e;
  }

  // ---- Arithmetic (in-place) --------------------------------------------
  LinearExpr& operator+=(const LinearExpr& o) {
    constant += o.constant;
    terms.insert(terms.end(), o.terms.begin(), o.terms.end());
    return *this;
  }
  LinearExpr& operator+=(double c) {
    constant += c;
    return *this;
  }
  LinearExpr& operator-=(const LinearExpr& o) {
    constant -= o.constant;
    for (const auto& [v, c] : o.terms) terms.emplace_back(v, -c);
    return *this;
  }
  LinearExpr& operator-=(double c) {
    constant -= c;
    return *this;
  }
  LinearExpr& operator*=(double s) {
    constant *= s;
    for (auto& [v, c] : terms) c *= s;
    return *this;
  }

  // ---- Arithmetic (value) -----------------------------------------------
  friend LinearExpr operator+(LinearExpr a, const LinearExpr& b) { return a += b; }
  friend LinearExpr operator+(LinearExpr a, double c)             { return a += c; }
  friend LinearExpr operator+(double c, LinearExpr a)             { return a += c; }
  friend LinearExpr operator-(LinearExpr a, const LinearExpr& b) { return a -= b; }
  friend LinearExpr operator-(LinearExpr a, double c)             { return a -= c; }
  friend LinearExpr operator-(double c, const LinearExpr& b) {
    LinearExpr r = LinearExpr::const_expr(c);
    r -= b;
    return r;
  }
  friend LinearExpr operator-(LinearExpr a) {
    a.constant = -a.constant;
    for (auto& [v, c] : a.terms) c = -c;
    return a;
  }
  friend LinearExpr operator*(double s, LinearExpr a) { return a *= s; }
  friend LinearExpr operator*(LinearExpr a, double s) { return a *= s; }

  // ---- Comparison: returns TempConstr, NOT LinearExpr -------------------
  [[nodiscard]] TempConstr operator<=(const LinearExpr& rhs) const;
  [[nodiscard]] TempConstr operator>=(const LinearExpr& rhs) const;
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  [[nodiscard]] TempConstr operator==(const LinearExpr& rhs) const;

  [[nodiscard]] TempConstr operator<=(double rhs) const;
  [[nodiscard]] TempConstr operator>=(double rhs) const;
  // NOLINTNEXTLINE(misc-unconventional-assign-operator)
  [[nodiscard]] TempConstr operator==(double rhs) const;

  // ---- Debug ------------------------------------------------------------
  [[nodiscard]] int num_terms() const { return static_cast<int>(terms.size()); }
  [[nodiscard]] bool empty()    const { return terms.empty() && constant == 0.0; }
};

// ════════════════════════════════════════════════════════════════════════════
// TempConstr — transient constraint handle produced by comparison operators.
// Never stored by the user; passed directly to Model::add_constraint.
// ════════════════════════════════════════════════════════════════════════════
struct TempConstr {
  LinearExpr lhs;
  LinearExpr rhs;
  CompareOp  sense = CompareOp::Equal;

  /// Normalize to:  (lhs - rhs) [sense] 0
  [[nodiscard]] LinearExpr canonical_lhs() const {
    LinearExpr cl = lhs;
    cl -= rhs;
    return cl;
  }
};

// ────────────────────────────────────────────────────────────────────────────
// Inline definitions: LinearExpr comparison operators
// ────────────────────────────────────────────────────────────────────────────
inline TempConstr LinearExpr::operator<=(const LinearExpr& rhs) const {
  return TempConstr{*this, rhs, CompareOp::LessEq};
}
inline TempConstr LinearExpr::operator>=(const LinearExpr& rhs) const {
  return TempConstr{*this, rhs, CompareOp::GreaterEq};
}
inline TempConstr LinearExpr::operator==(const LinearExpr& rhs) const {
  return TempConstr{*this, rhs, CompareOp::Equal};
}
inline TempConstr LinearExpr::operator<=(double rhs) const {
  return TempConstr{*this, LinearExpr::const_expr(rhs), CompareOp::LessEq};
}
inline TempConstr LinearExpr::operator>=(double rhs) const {
  return TempConstr{*this, LinearExpr::const_expr(rhs), CompareOp::GreaterEq};
}
inline TempConstr LinearExpr::operator==(double rhs) const {
  return TempConstr{*this, LinearExpr::const_expr(rhs), CompareOp::Equal};
}

// ────────────────────────────────────────────────────────────────────────────
// Aggregation helpers
// ────────────────────────────────────────────────────────────────────────────

/// Sum a vector of linear expressions.
inline LinearExpr sum(const std::vector<LinearExpr>& exprs) {
  LinearExpr result;
  for (const auto& e : exprs) result += e;
  return result;
}

/// Sum a function over all elements of a set.
/// fn: const Key& -> LinearExpr
template <typename Fn>
LinearExpr sum(const Set& S, Fn fn) {
  LinearExpr result;
  for (const auto& k : S.elements()) result += fn(k);
  return result;
}

// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
// QuadExpr — linear part + bilinear/quadratic terms  (Beta milestone)
//
// Internal form:  constant  +  Σ lin_coef_i * x_i  +  Σ quad_coef_ij * x_i * x_j
//
// Convention for QP compilation: the engine expects  0.5 * x' Q x + c' x.
// When assembling Q from quad_terms, diagonal term (i==j) with coef c maps
// to Q[i,i] = 2c; cross term (i≠j) with coef c maps to Q[i,j] = Q[j,i] = c
// (so that 0.5*(Q[i,j] + Q[j,i]) * x_i * x_j = c * x_i * x_j).
// ════════════════════════════════════════════════════════════════════════════
struct QuadTerm {
  VarId  i;
  VarId  j;
  double coef;
};

struct QuadExpr {
  LinearExpr            linear_part;
  std::vector<QuadTerm> quad_terms;

  // ---- Arithmetic -------------------------------------------------------
  QuadExpr& operator+=(const QuadExpr& o) {
    linear_part += o.linear_part;
    quad_terms.insert(quad_terms.end(), o.quad_terms.begin(), o.quad_terms.end());
    return *this;
  }
  QuadExpr& operator+=(const LinearExpr& o) {
    linear_part += o;
    return *this;
  }
  QuadExpr& operator+=(double c) {
    linear_part += c;
    return *this;
  }
  QuadExpr& operator-=(const QuadExpr& o) {
    linear_part -= o.linear_part;
    for (const auto& qt : o.quad_terms)
      quad_terms.push_back({qt.i, qt.j, -qt.coef});
    return *this;
  }
  QuadExpr& operator*=(double s) {
    linear_part *= s;
    for (auto& qt : quad_terms) qt.coef *= s;
    return *this;
  }

  friend QuadExpr operator+(QuadExpr a, const QuadExpr& b) { return a += b; }
  friend QuadExpr operator+(QuadExpr a, const LinearExpr& b) { return a += b; }
  friend QuadExpr operator+(const LinearExpr& a, QuadExpr b) { return b += a; }
  friend QuadExpr operator-(QuadExpr a, const QuadExpr& b) { return a -= b; }
  friend QuadExpr operator*(double s, QuadExpr a) { return a *= s; }
  friend QuadExpr operator*(QuadExpr a, double s) { return a *= s; }

  // ---- Factories --------------------------------------------------------
  /// Build x_i^2 term.
  static QuadExpr sq(VarId v, double coef = 1.0) {
    QuadExpr e;
    e.quad_terms.push_back({v, v, coef});
    return e;
  }
  /// Build x_i * x_j term.
  static QuadExpr bilinear(VarId vi, VarId vj, double coef = 1.0) {
    QuadExpr e;
    e.quad_terms.push_back({vi, vj, coef});
    return e;
  }
  /// Build from a LinearExpr (zero quadratic part).
  static QuadExpr from_linear(const LinearExpr& lin) {
    QuadExpr e;
    e.linear_part = lin;
    return e;
  }

  // ---- Debug ------------------------------------------------------------
  [[nodiscard]] bool is_linear() const { return quad_terms.empty(); }
  [[nodiscard]] int  num_quad_terms() const {
    return static_cast<int>(quad_terms.size());
  }
};

// ════════════════════════════════════════════════════════════════════════════
// NonlinearExpr — handle into the model's expression arena  (Advanced)
// ════════════════════════════════════════════════════════════════════════════

/// Node operation codes for the arena DAG.
enum class ExprOp {
  Constant,
  Variable,
  Add,
  Sub,
  Neg,
  Mul,
  Div,
  Pow2,      // x^2 (common special case)
  PowN,      // x^n; exponent encoded in lhs_id
  Exp,
  Log,
  Sqrt,
  Sin,
  Cos,
  Tan,
  Abs,
  Max2,
  Min2,
};

struct ExprNode {
  ExprOp op          = ExprOp::Constant;
  ExprId lhs_id      = kInvalidExpr;
  ExprId rhs_id      = kInvalidExpr;
  double value       = 0.0;           ///< Constant nodes
  VarId  var_id      = kInvalidVar;   ///< Variable nodes
};

/// Opaque handle into ModelImpl's arena vector.
class NonlinearExpr {
 public:
  NonlinearExpr() = default;
  explicit NonlinearExpr(ExprId id) : id_(id) {}

  [[nodiscard]] ExprId id()    const noexcept { return id_; }
  [[nodiscard]] bool is_null() const noexcept { return id_ == kInvalidExpr; }

  // Comparison operators intentionally not declared here;
  // they require Model& context to resolve variable IDs.

 private:
  ExprId id_ = kInvalidExpr;
};

}  // namespace mipsolvers::aml

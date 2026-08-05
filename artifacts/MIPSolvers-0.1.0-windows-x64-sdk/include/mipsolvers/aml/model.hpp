#pragma once

#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "mipsolvers/aml/constraint.hpp"
#include "mipsolvers/aml/expression.hpp"
#include "mipsolvers/aml/ids.hpp"
#include "mipsolvers/aml/key.hpp"
#include "mipsolvers/aml/parameter.hpp"
#include "mipsolvers/aml/set.hpp"
#include "mipsolvers/aml/solve_result.hpp"
#include "mipsolvers/aml/solver_capabilities.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml {

// ── Solve options forwarded to the engine ─────────────────────────────────
struct SolveOptions {
  std::string solver_name;    ///< "" = auto; "highs", "gurobi", "native", …
  double      time_limit_sec  = 1e30;
  double      mip_gap_tol     = 1e-4;
  int         verbosity       = 0;    ///< 0 = silent, 1 = summary, 2 = verbose
  bool        allow_fallback  = true; ///< If false, do not try alternative solvers on failure
};

// ────────────────────────────────────────────────────────────────────────────
// ModelImpl forward declaration (implementation detail hidden in .cpp)
// ────────────────────────────────────────────────────────────────────────────
class ModelImpl;

// ════════════════════════════════════════════════════════════════════════════
// Model — top-level user interface for building and solving AML problems.
// ════════════════════════════════════════════════════════════════════════════
class Model {
 public:
  explicit Model(const std::string& name = "");
  ~Model();
  Model(Model&&) noexcept;
  Model& operator=(Model&&) noexcept;

  // Non-copyable (ModelImpl owns heavy state)
  Model(const Model&)            = delete;
  Model& operator=(const Model&) = delete;

  [[nodiscard]] const std::string& name() const noexcept;

  // ── Set construction ───────────────────────────────────────────────────

  /// Create a 1-D explicit set with optional initial atoms.
  ExplicitSet& add_set(const std::string& name,
                       const std::vector<Atom>& atoms = {});

  /// Create an N-D explicit set with optional initial keys.
  ExplicitSet& add_set_nd(const std::string& name, int dim,
                           const std::vector<Key>& elems = {});

  /// Create a 1-D ordered set with optional initial atoms.
  OrderedSet& add_ordered_set(const std::string& name,
                               const std::vector<Atom>& atoms = {});

  // ── Parameter construction ─────────────────────────────────────────────

  /// Scalar parameter (dim == 0, stored at empty key).
  Parameter& add_param_scalar(const std::string& name,
                               const std::string& unit = "");

  /// N-D parameter with explicit domain dimension.
  Parameter& add_param(const std::string& name, int dim,
                        const std::string& unit = "");

  // ── Variable construction ──────────────────────────────────────────────

  /// Define a VarArray over a single domain set.
  VarArray& add_var(const std::string& name,
                    const Set& domain,
                    VarType type,
                    double lb = -1e20,
                    double ub =  1e20);

  /// Define a VarArray over the Cartesian product of two domain sets.
  VarArray& add_var(const std::string& name,
                    const Set& domain_a, const Set& domain_b,
                    VarType type,
                    double lb = -1e20,
                    double ub =  1e20);

  // ── Objective ─────────────────────────────────────────────────────────

  void minimize(const LinearExpr& obj);
  void maximize(const LinearExpr& obj);

  /// Set a quadratic objective (Beta milestone).
  /// Continuous models are treated as QPs. Integer variables combined with a
  /// quadratic objective are rejected at solve/export time because AML has no
  /// MIQP backend contract yet.
  void minimize(const QuadExpr& obj);
  void maximize(const QuadExpr& obj);

  /// Set a nonlinear objective (Advanced milestone).
  /// The model is dispatched to the NLP solver path.
  void minimize(const NonlinearExpr& obj);
  void maximize(const NonlinearExpr& obj);

  // ── Nonlinear expression builders ─────────────────────────────────────
  //
  // All builder methods allocate a node into the model's expression arena
  // and return an opaque NonlinearExpr handle.  Nodes with lower IDs are
  // always children of nodes with higher IDs (construction order invariant).

  /// Wrap a variable as a nonlinear expression leaf.
  NonlinearExpr nl_var(VarRef v);
  /// Wrap a scalar constant as a nonlinear expression leaf.
  NonlinearExpr nl_const(double c);

  /// Arithmetic
  NonlinearExpr nl_neg(NonlinearExpr a);
  NonlinearExpr nl_add(NonlinearExpr a, NonlinearExpr b);
  NonlinearExpr nl_sub(NonlinearExpr a, NonlinearExpr b);
  NonlinearExpr nl_mul(NonlinearExpr a, NonlinearExpr b);
  NonlinearExpr nl_div(NonlinearExpr a, NonlinearExpr b);

  /// Power / root
  NonlinearExpr nl_sq  (NonlinearExpr a);           ///< a²
  NonlinearExpr nl_pow (NonlinearExpr a, int n);    ///< aⁿ  (n ≥ 0 integer)
  NonlinearExpr nl_sqrt(NonlinearExpr a);           ///< √a

  /// Transcendental
  NonlinearExpr nl_exp(NonlinearExpr a);
  NonlinearExpr nl_log(NonlinearExpr a);
  NonlinearExpr nl_sin(NonlinearExpr a);
  NonlinearExpr nl_cos(NonlinearExpr a);
  NonlinearExpr nl_tan(NonlinearExpr a);

  /// Misc
  NonlinearExpr nl_abs(NonlinearExpr a);
  NonlinearExpr nl_max(NonlinearExpr a, NonlinearExpr b);
  NonlinearExpr nl_min(NonlinearExpr a, NonlinearExpr b);

  // ── Nonlinear constraint addition ──────────────────────────────────────

  /// Add a nonlinear constraint: expr [sense] rhs.
  /// Linear constraints added via add_constraint() are also forwarded to
  /// the NLP compiler when the model is dispatched to the NLP path.
  ConstraintRef add_nl_constraint(const std::string& name,
                                   NonlinearExpr expr,
                                   CompareOp       sense,
                                   double          rhs = 0.0);

  /// Provide an initial point for the NLP solver (optional; defaults to
  /// the midpoint of variable bounds when not set).
  void set_nlp_x0(const std::vector<double>& x0);

  // ── Constraint addition ────────────────────────────────────────────────

  /// Add a single anonymous constraint.
  ConstraintRef add_constraint(const TempConstr& c,
                                const std::string& name = "");

  /// Add an indexed family of constraints over a domain set.
  /// fn: const Key& -> TempConstr
  template <typename Fn>
  ConstraintArray& add_constraints(const std::string& family,
                                    const Set& domain, Fn fn) {
    auto& arr = create_constraint_array(family);
    for (const auto& k : domain.elements()) {
      TempConstr tc = fn(k);
      ConId cid = add_constraint_internal(tc, family + "[...]");
      arr.register_con(k, cid);
    }
    return arr;
  }

  // ── Conic constraint addition (SOCP / SDP) ──────────────────────────────

  /// Second-order cone constraint: ||xs||_2 <= t, with t and xs affine in the
  /// model variables. t >= 0 is implied by the cone.
  ConstraintRef add_soc_constraint(const LinearExpr& t,
                                    const std::vector<LinearExpr>& xs,
                                    const std::string& name = "");

  /// Rotated second-order cone: ||xs||_2^2 <= 2*a*b, a,b >= 0 implied.
  /// Reformulated into a standard SOC at compile time.
  ConstraintRef add_rotated_soc_constraint(const LinearExpr& a,
                                            const LinearExpr& b,
                                            const std::vector<LinearExpr>& xs,
                                            const std::string& name = "");

  /// Semidefinite constraint: the symmetric n-by-n matrix F(x) with lower-
  /// triangular entries F(i,j) = entries[i][j] (i >= j) is affine in the model
  /// variables; constraint is F(x) ⪰ 0. entries[i] must have exactly i+1
  /// elements.
  ConstraintRef add_psd_constraint(
      int order, const std::vector<std::vector<LinearExpr>>& entries,
      const std::string& name = "");

  // ── Compile / solve ────────────────────────────────────────────────────

  struct CompileOptions {
    bool remove_zero_terms    = true;   ///< drop |coef| < 1e-14
    bool aggregate_duplicates = true;   ///< sum duplicate VarIds per row
    int  verbosity            = 0;
  };

  /// Solve the model using the engine.
  SolveResult solve(const SolveOptions& opts = {}) const;

  // ── Inspection ────────────────────────────────────────────────────────

  [[nodiscard]] int num_vars()        const noexcept;
  [[nodiscard]] int num_constraints() const noexcept;

  void print_summary()        const;
  void check_bounds()         const;  ///< error if any lb > ub
  /// Validate registered parameter storage. Scalar parameters must be set,
  /// indexed parameter tables must contain at least one entry, and all stored
  /// values must be finite. Full domain coverage cannot be inferred because
  /// parameters are currently declared by dimension, not by a Set.
  void check_missing_params() const;

  // ── Export ────────────────────────────────────────────────────────────
  /// Export a linear LP or MILP. QP, NLP, MINLP and conic models are rejected.
  void write_lp  (const std::string& path) const;
  /// Export a linear LP or MILP in free MPS format. Unsupported nonlinear and
  /// quadratic model classes are rejected instead of being silently reduced.
  void write_mps (const std::string& path) const;
  void write_json(const std::string& path) const;  ///< Beta: JSON model export

 private:
  // Helper: create or retrieve ConstraintArray
  ConstraintArray& create_constraint_array(const std::string& name);
  ConId add_constraint_internal(const TempConstr& c, const std::string& name);

  std::unique_ptr<ModelImpl> impl_;
};

}  // namespace mipsolvers::aml

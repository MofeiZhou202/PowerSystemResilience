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
  /// The model is then treated as QP (or MIQP if integer vars are present).
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
  void check_missing_params() const;  ///< warn on unset domain keys

  // ── Export ────────────────────────────────────────────────────────────
  void write_lp  (const std::string& path) const;
  void write_mps (const std::string& path) const;
  void write_json(const std::string& path) const;  ///< Beta: JSON model export

 private:
  // Helper: create or retrieve ConstraintArray
  ConstraintArray& create_constraint_array(const std::string& name);
  ConId add_constraint_internal(const TempConstr& c, const std::string& name);

  std::unique_ptr<ModelImpl> impl_;
};

}  // namespace mipsolvers::aml

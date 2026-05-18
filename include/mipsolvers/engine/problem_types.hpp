#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace mipsolvers::engine {

enum class ProblemClass { LE, NLE, LP, QP, NLP, MILP, MINLP };

enum class VarType { Continuous, Integer, Binary };

enum class Sense { Minimize, Maximize };

enum class SymOp { Constant, Variable, Add, Sub, Mul, Neg, Pow2, PowN };

struct SymExpr {
  SymOp op{SymOp::Constant};
  double value{0.0};
  int var_index{-1};
  std::shared_ptr<SymExpr> lhs;
  std::shared_ptr<SymExpr> rhs;

  static std::shared_ptr<SymExpr> constant(double v) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Constant;
    e->value = v;
    return e;
  }

  static std::shared_ptr<SymExpr> variable(int idx) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Variable;
    e->var_index = idx;
    return e;
  }

  static std::shared_ptr<SymExpr> add(const std::shared_ptr<SymExpr>& a,
                                      const std::shared_ptr<SymExpr>& b) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Add;
    e->lhs = a;
    e->rhs = b;
    return e;
  }

  static std::shared_ptr<SymExpr> sub(const std::shared_ptr<SymExpr>& a,
                                      const std::shared_ptr<SymExpr>& b) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Sub;
    e->lhs = a;
    e->rhs = b;
    return e;
  }

  static std::shared_ptr<SymExpr> mul(const std::shared_ptr<SymExpr>& a,
                                      const std::shared_ptr<SymExpr>& b) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Mul;
    e->lhs = a;
    e->rhs = b;
    return e;
  }

  static std::shared_ptr<SymExpr> neg(const std::shared_ptr<SymExpr>& a) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Neg;
    e->lhs = a;
    return e;
  }

  static std::shared_ptr<SymExpr> pow2(const std::shared_ptr<SymExpr>& a) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::Pow2;
    e->lhs = a;
    return e;
  }

  /// Raise expression to an arbitrary positive integer power.
  static std::shared_ptr<SymExpr> powN(const std::shared_ptr<SymExpr>& a, int n) {
    auto e = std::make_shared<SymExpr>();
    e->op = SymOp::PowN;
    e->lhs = a;
    e->var_index = n;  // reuse var_index to store the exponent
    return e;
  }
};

enum class SymbolicSense { Equal, LessEqual, GreaterEqual };

struct SymbolicConstraint {
  std::string name;
  std::shared_ptr<SymExpr> expr;
  SymbolicSense sense{SymbolicSense::Equal};
  double rhs{0.0};
};

struct VariableMeta {
  VarType type{VarType::Continuous};
  double lb{-1e20};
  double ub{1e20};
  std::string name;
};

struct SparseLinSys {
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;
};

struct NonlinearSystem {
  int n{0};
  std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)> residual;
  std::function<void(const Eigen::VectorXd&, Eigen::SparseMatrix<double>&)> jacobian;
  Eigen::VectorXd x0;
  /// Optional projection callback: clamp state variables to physical bounds
  /// (e.g., Vm >= 0.05) after each line-search trial step.
  std::function<void(Eigen::VectorXd&)> project;
};

struct LPModel {
  Sense sense{Sense::Minimize};
  Eigen::VectorXd c;
  // Inequality kernel: row_lhs[i] <= A[i,:] * x <= b[i].
  // Historically LPModel only represented A*x <= b; an empty row_lhs keeps
  // that convention and means all inequality lower sides are -infinity.
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd row_lhs;
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> Aeq;
  Eigen::VectorXd beq;
  std::vector<VariableMeta> vars;
  // Optional source row order for HiGHS-presolved LPs.  The native Eigen
  // matrices sort row entries by column, but HiGHS separators consume rows in
  // the row-wise order returned by HighsLpRelaxation::getRow().
  std::vector<int> highs_row_to_native_row;
  std::vector<int> highs_row_start;
  std::vector<int> highs_row_index;
  std::vector<double> highs_row_value;
};

inline bool lp_has_row_lhs(const LPModel& lp) {
  return lp.row_lhs.size() == lp.A.rows();
}

inline double lp_row_lhs_or_neg_inf(const LPModel& lp, int row) {
  if (row >= 0 && row < lp.row_lhs.size() && lp_has_row_lhs(lp)) {
    return lp.row_lhs[row];
  }
  return -std::numeric_limits<double>::infinity();
}

inline bool lp_row_has_finite_lhs(const LPModel& lp, int row) {
  return std::isfinite(lp_row_lhs_or_neg_inf(lp, row));
}

/// Quadratic Programming model: min 0.5*x'Qx + c'x  s.t.  A*x <= b, Aeq*x = beq
/// This is LCQP (Linear Constraints Quadratic Programming) — convex QP.
struct QPModel {
  Sense sense{Sense::Minimize};
  Eigen::SparseMatrix<double> Q;  // Hessian of objective (symmetric, PSD)
  Eigen::VectorXd c;              // Linear term of objective
  Eigen::SparseMatrix<double> A;  // Inequality constraints A*x <= b
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> Aeq;  // Equality constraints Aeq*x = beq
  Eigen::VectorXd beq;
  std::vector<VariableMeta> vars;
};

struct NLPModel {
  Sense sense{Sense::Minimize};
  std::function<double(const Eigen::VectorXd&)> f;
  std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)> grad;
  std::function<void(const Eigen::VectorXd&, Eigen::SparseMatrix<double>&)> hess;
  // Optional exact Hessian of the effective minimization Lagrangian:
  //   L(x, lambda, nu) = f_eff(x) + lambda' g(x) + nu' h_nonlin(x)
  // where f_eff is already sense-normalized for minimization. The inequality
  // multipliers correspond only to the nonlinear inequality block, not box
  // bounds introduced by the solver.
  std::function<void(const Eigen::VectorXd&,
                     const Eigen::VectorXd&,
                     const Eigen::VectorXd*,
                     Eigen::SparseMatrix<double>&)> lagrangian_hess;
  std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)> g;
  std::function<void(const Eigen::VectorXd&, Eigen::SparseMatrix<double>&)> jac_g;
  std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)> h;
  std::function<void(const Eigen::VectorXd&, Eigen::SparseMatrix<double>&)> jac_h;

  // Optional symbolic representation for exportable solver adapters.
  std::shared_ptr<SymExpr> symbolic_objective;
  std::vector<SymbolicConstraint> symbolic_constraints;

  std::vector<VariableMeta> vars;
  Eigen::VectorXd x0;
};

struct MIPModel {
  LPModel linear_part;
  std::vector<int> integer_idx;
  std::vector<int> binary_idx;

  /// Optional warm-start: a feasible (or near-feasible) integer solution.
  /// When provided, the B&C solver uses it as the initial incumbent,
  /// enabling earlier pruning and drastically reducing node count.
  Eigen::VectorXd initial_solution;

  /// Unit-commitment hint: optional metadata for domain-specific experiments.
  /// Native B&C ignores it unless BCOptions::enable_domain_heuristics is true.
  struct UCGenHint {
    int ng{0};          ///< number of generators
    int T{0};           ///< number of commitment periods
    double period_hours{1.0};  ///< duration of one period in hours
    int ig_start{0};    ///< first index of IG(g,t) block (size ng×T)
    int su_start{0};    ///< first index of SU(g,t) block (size ng×T)
    int sd_start{0};    ///< first index of SD(g,t) block (size ng×T)
    int pg_start{-1};   ///< first index of PG(g,t) block (-1 if unknown)
    std::vector<int> min_up;    ///< min-up hours per generator (ng entries)
    std::vector<int> min_down;  ///< min-down hours per generator
    std::vector<int> ig0;       ///< initial commitment status (0 or 1)
    std::vector<double> pmin;   ///< minimum power per generator when committed
    std::vector<double> pmax;   ///< maximum power per generator
    std::vector<double> ramp;   ///< ramp rate limit per generator (MW/period)
    /// Per-element column maps for reduced model (size ng*T each, -1 = eliminated)
    std::vector<int> ig_cols;   ///< ig_cols[t*ng+g] = reduced col for u(g,t)
    std::vector<int> su_cols;   ///< su_cols[t*ng+g] = reduced col for su(g,t)
    std::vector<int> sd_cols;   ///< sd_cols[t*ng+g] = reduced col for sd(g,t)
    std::vector<int> pg_cols;   ///< pg_cols[t*ng+g] = reduced col for p(g,t)

    /// Constraint certificates for dynamic user cuts. Metadata by itself is
    /// only descriptive; a callback may generate a row only when the matching
    /// original-space constraint family is certified here.
    bool certifies_power_balance_rows{false};
    bool certifies_generation_capacity_rows{false};
    bool certifies_ramping_rows{false};
    bool certifies_min_up_down_rows{false};
    bool certifies_segment_bound_rows{false};
    bool certifies_system_reserve_rows{false};
    bool certifies_area_import_rows{false};
    bool certifies_hard_network_flow_rows{false};
    bool certifies_hard_section_flow_rows{false};
    bool certifies_storage_cycle_rows{false};

    /// Optional SCUC metadata for problem-specific dynamic user cuts.
    /// All indices remain in the original MIP column space. Empty vectors mean
    /// the corresponding family is not certified for this model and must be
    /// skipped by user-cut callbacks.
    std::vector<int> gen_bus;            ///< generator -> bus index, ng entries
    std::vector<double> demand;          ///< demand[t], T entries
    std::vector<double> reserve_requirement;  ///< optional reserve[t], T entries
    std::vector<double> spinning_requirement;  ///< spinning reserve req[t]
    std::vector<double> regulation_up_requirement;  ///< reg-up reserve req[t]
    std::vector<double> regulation_down_requirement;  ///< reg-down reserve req[t]
    std::vector<double> up_reserve_headroom_cap;  ///< max spin+reg-up per unit
    std::vector<double> spinning_reserve_cap;     ///< max spin per unit
    std::vector<double> regulation_up_cap;        ///< max reg-up per unit
    std::vector<double> regulation_down_cap;      ///< max reg-down per unit

    /// Optional balancing-area metadata. Area-indexed arrays use
    /// area*T + t ordering. area_import_capacity is a certified upper bound on
    /// net import into the area in the original model; leave it empty unless
    /// that bound follows from explicit original constraints.
    int n_areas{0};
    std::vector<int> gen_area;  ///< generator -> compact area index, ng entries
    std::vector<double> area_demand;
    std::vector<double> area_reserve_requirement;
    std::vector<double> area_import_capacity;
    std::vector<double> area_storage_discharge_capacity;

    /// Network deliverability hint for PTDF/GSF cuts. Coefficients are indexed
    /// as line_gsf[line*ng + g]. RHS arrays are line_fwd_rhs[line*T + t] and
    /// line_rev_rhs[line*T + t] for +GSF and -GSF line constraints.
    int network_line_count{0};
    std::vector<double> line_gsf;
    std::vector<double> line_fwd_rhs;
    std::vector<double> line_rev_rhs;

    /// Optional monitored-section / corridor metadata, with the same layout
    /// as line_gsf and line_*_rhs. A section row must be a nonnegative
    /// aggregation of explicit original network constraints to be valid as a
    /// dynamic user cut source.
    int section_count{0};
    std::vector<double> section_gsf;
    std::vector<double> section_fwd_rhs;
    std::vector<double> section_rev_rhs;

    /// Segment cumulative cuts. segment_cols[(t*ng + g)*n_segments + k] gives
    /// the original column of segment k for unit g,t. segment_cap[g*n_segments+k]
    /// gives the segment capacity.
    int n_segments{0};
    std::vector<int> segment_cols;
    std::vector<double> segment_cap;

    /// Storage cycle metadata. These are empty for the synthetic benchmark SCUC.
    int n_storage{0};
    std::vector<int> storage_charge_cols;     ///< storage_charge_cols[t*n_storage+s]
    std::vector<int> storage_discharge_cols;  ///< storage_discharge_cols[t*n_storage+s]
    std::vector<double> storage_energy_capacity;
    std::vector<double> storage_efficiency;
    std::vector<double> storage_initial_energy;
    std::vector<double> storage_cycle_limit;
  };
  std::optional<UCGenHint> uc_hint;

  /// Per-variable branching priority (higher = branch first).
  /// Empty means all variables have equal priority.
  /// When non-empty, must have size == linear_part.vars.size().
  std::vector<int> branching_priority;
};

struct MINLPModel {
  NLPModel nonlinear_part;
  std::vector<int> integer_idx;
  std::vector<int> binary_idx;
};

}  // namespace mipsolvers::engine

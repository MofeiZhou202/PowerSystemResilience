#include "mipsolvers/aml/model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/engine.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::aml {

// ════════════════════════════════════════════════════════════════════════════
// Per-row record stored during model building
// ════════════════════════════════════════════════════════════════════════════
struct RowRecord {
  LinearExpr lhs;
  LinearExpr rhs;
  CompareOp  sense = CompareOp::Equal;
  std::string name;
};

// ════════════════════════════════════════════════════════════════════════════
// ModelImpl — the real implementation (hidden from users)
// ════════════════════════════════════════════════════════════════════════════
class ModelImpl {
 public:
  std::string model_name;

  // ── Sets ────────────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<ExplicitSet>> explicit_sets;
  std::vector<std::unique_ptr<OrderedSet>>  ordered_sets;

  // ── Parameters ──────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<Parameter>> params;

  // ── Variables ───────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<VarArray>> var_arrays;
  std::vector<VarMeta> var_meta;   // indexed by VarId

  // ── Constraints ─────────────────────────────────────────────────────────
  std::vector<std::unique_ptr<ConstraintArray>> con_arrays;
  std::vector<RowRecord> rows;     // indexed by ConId

  // ── Objective ───────────────────────────────────────────────────────────
  LinearExpr           obj_expr;
  QuadExpr             quad_obj_expr;  // non-empty → QP mode
  bool                 has_quad_obj = false;
  bool                 has_nl_obj   = false;
  NonlinearExpr        nl_obj_expr_;  // valid when has_nl_obj
  engine::Sense        sense = engine::Sense::Minimize;

  // ── Nonlinear expression arena ──────────────────────────────────────────
  std::vector<ExprNode> expr_arena_;

  // ── Nonlinear constraint rows ──────────────────────────────────────────
  struct NLRowRecord {
    ExprId    expr_id;
    CompareOp sense;
    double    rhs;
    std::string name;
  };
  std::vector<NLRowRecord> nl_rows_;

  // ── NLP warm-start ──────────────────────────────────────────────────────
  std::vector<double> nlp_x0_;  // empty → use midpoint

  // ── Variable id counter ─────────────────────────────────────────────────
  VarId next_var_id = 0;
  ConId next_con_id = 0;

  // ── Helpers ─────────────────────────────────────────────────────────────

  VarArray& register_vars(std::unique_ptr<VarArray> arr,
                           const std::vector<Key>& keys) {
    VarArray& ref = *arr;
    for (const auto& k : keys) {
      VarMeta meta;
      meta.family = arr->name();
      meta.key    = k;
      meta.type   = arr->type();
      meta.lb     = arr->lb(k);
      meta.ub     = arr->ub(k);
      arr->register_var(k, next_var_id);
      var_meta.push_back(std::move(meta));
      ++next_var_id;
    }
    var_arrays.push_back(std::move(arr));
    return ref;
  }

  ConstraintArray& get_or_create_con_array(const std::string& name) {
    for (auto& p : con_arrays) {
      if (p->name() == name) return *p;
    }
    con_arrays.push_back(std::make_unique<ConstraintArray>(name));
    return *con_arrays.back();
  }

  ConId add_row(const TempConstr& tc, const std::string& name) {
    ConId id = next_con_id++;
    rows.push_back({tc.lhs, tc.rhs, tc.sense, name});
    return id;
  }

  // ── Three-pass compiler: build engine::LPModel or engine::MIPModel ──────

  void aggregate_terms(std::vector<std::pair<VarId, double>>& terms,
                       double* constant_out) const {
    // Sort by VarId, then sum
    std::sort(terms.begin(), terms.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<std::pair<VarId, double>> agg;
    for (const auto& [v, c] : terms) {
      if (!agg.empty() && agg.back().first == v) {
        agg.back().second += c;
      } else {
        agg.emplace_back(v, c);
      }
    }
    // Remove near-zero
    terms.clear();
    for (const auto& [v, c] : agg) {
      if (std::fabs(c) >= 1e-14) terms.emplace_back(v, c);
    }
    (void)constant_out;
  }

  engine::LPModel compile_lp() const {
    int nv = next_var_id;
    int nc_ineq = 0, nc_eq = 0;

    for (const auto& r : rows) {
      if (r.sense == CompareOp::Equal) ++nc_eq;
      else                             ++nc_ineq;
    }

    // Objective vector
    Eigen::VectorXd c = Eigen::VectorXd::Zero(nv);
    {
      // Obj = obj_expr.terms (canonical: lhs with rhs subtracted already baked in)
      // constant in objective is ignored (does not affect optimality)
      for (const auto& [v, coef] : obj_expr.terms) {
        c[v] += coef;
      }
    }

    // Build sparse triplets
    using T3 = Eigen::Triplet<double>;
    std::vector<T3> ineq_trip, eq_trip;
    Eigen::VectorXd b_ineq   = Eigen::VectorXd::Zero(nc_ineq);
    Eigen::VectorXd row_lhs  = Eigen::VectorXd::Constant(nc_ineq,
                                    -std::numeric_limits<double>::infinity());
    Eigen::VectorXd b_eq     = Eigen::VectorXd::Zero(nc_eq);

    int ii = 0, ei = 0;
    for (const auto& r : rows) {
      // canonical: lhs - rhs [sense] 0  → lhs_terms * x [sense] -canonical_constant
      LinearExpr cl = r.lhs;
      cl -= r.rhs;
      // aggregate
      auto terms = cl.terms;
      std::sort(terms.begin(), terms.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
      std::vector<std::pair<VarId, double>> agg;
      for (const auto& [v, coef] : terms) {
        if (!agg.empty() && agg.back().first == v)
          agg.back().second += coef;
        else
          agg.emplace_back(v, coef);
      }
      double rhs_val = -cl.constant;

      if (r.sense == CompareOp::Equal) {
        for (const auto& [v, coef] : agg)
          if (std::fabs(coef) >= 1e-14)
            eq_trip.emplace_back(ei, v, coef);
        b_eq[ei] = rhs_val;
        ++ei;
      } else {
        // LessEq:    row <= rhs_val  → standard upper-bound row
        // GreaterEq: row >= rhs_val  → flip to -row <= -rhs_val
        double sign = (r.sense == CompareOp::LessEq) ? 1.0 : -1.0;
        for (const auto& [v, coef] : agg)
          if (std::fabs(coef * sign) >= 1e-14)
            ineq_trip.emplace_back(ii, v, coef * sign);
        b_ineq[ii]  = rhs_val * sign;
        row_lhs[ii] = -std::numeric_limits<double>::infinity(); // no lower side
        ++ii;
      }
    }

    // Variable metadata — re-query bounds from var_arrays to pick up
    // any set_lb / set_ub / fix calls made after add_var().
    std::unordered_map<VarId, std::pair<double, double>> bound_overrides;
    bound_overrides.reserve(static_cast<std::size_t>(nv));
    for (const auto& arr : var_arrays) {
      for (const auto& [k, vid] : arr->key_to_id()) {
        bound_overrides[vid] = {arr->lb(k), arr->ub(k)};
      }
    }

    std::vector<engine::VariableMeta> vmeta(nv);
    for (int i = 0; i < nv; ++i) {
      const auto& m = var_meta[static_cast<std::size_t>(i)];
      // Binary variables always get [0,1] bounds regardless of user setting
      if (m.type == VarType::Binary) {
        vmeta[i].lb = 0.0;
        vmeta[i].ub = 1.0;
      } else {
        auto it = bound_overrides.find(i);
        if (it != bound_overrides.end()) {
          vmeta[i].lb = it->second.first;
          vmeta[i].ub = it->second.second;
        } else {
          vmeta[i].lb = m.lb;
          vmeta[i].ub = m.ub;
        }
      }
      vmeta[i].name = m.family + "[" + (m.key.values.empty() ? "" : m.key.values[0]) + "]";
      switch (m.type) {
        case VarType::Continuous: vmeta[i].type = engine::VarType::Continuous; break;
        case VarType::Integer:    vmeta[i].type = engine::VarType::Integer;    break;
        case VarType::Binary:     vmeta[i].type = engine::VarType::Binary;     break;
      }
    }

    engine::LPModel lp;
    lp.sense = sense;
    lp.c     = std::move(c);
    if (nc_ineq > 0) {
      lp.A.resize(nc_ineq, nv);
      lp.A.setFromTriplets(ineq_trip.begin(), ineq_trip.end());
      lp.b       = std::move(b_ineq);
      lp.row_lhs = std::move(row_lhs);
    } else {
      lp.A.resize(0, nv);
      lp.b.resize(0);
    }
    if (nc_eq > 0) {
      lp.Aeq.resize(nc_eq, nv);
      lp.Aeq.setFromTriplets(eq_trip.begin(), eq_trip.end());
      lp.beq = std::move(b_eq);
    } else {
      lp.Aeq.resize(0, nv);
      lp.beq.resize(0);
    }
    lp.vars = std::move(vmeta);
    return lp;
  }

  /// Compile a QP by starting from compile_lp() and adding the Q matrix.
  /// Convention: engine uses min 0.5*x'Qx + c'x.
  /// AML stores: Σ coef_ij * x_i * x_j; mapping:
  ///   diagonal (i==j): Q[i,i] += 2*coef
  ///   off-diagonal:    Q[i,j] += coef, Q[j,i] += coef
  engine::QPModel compile_qp() const {
    // Compile the LP shell (constraints, variable bounds, linear obj)
    engine::LPModel lp = compile_lp();
    const int nv = static_cast<int>(lp.c.size());

    // Override linear obj with the linear part of QuadExpr
    lp.c = Eigen::VectorXd::Zero(nv);
    for (const auto& [v, coef] : quad_obj_expr.linear_part.terms) {
      if (v >= 0 && v < nv) lp.c[v] += coef;
    }

    // Build symmetric Q matrix from quad_terms
    using T3 = Eigen::Triplet<double>;
    std::vector<T3> q_trip;
    q_trip.reserve(quad_obj_expr.quad_terms.size() * 2);
    for (const auto& qt : quad_obj_expr.quad_terms) {
      if (qt.i < 0 || qt.i >= nv || qt.j < 0 || qt.j >= nv) continue;
      if (qt.i == qt.j) {
        // diagonal: x_i^2 → Q[i,i] = 2*coef so that 0.5*Q[i,i]*x_i^2 = coef*x_i^2
        q_trip.emplace_back(qt.i, qt.i, 2.0 * qt.coef);
      } else {
        // cross: coef*x_i*x_j → Q[i,j] = Q[j,i] = coef (0.5*(Q+Q')*x_i*x_j = coef*x_i*x_j)
        q_trip.emplace_back(qt.i, qt.j, qt.coef);
        q_trip.emplace_back(qt.j, qt.i, qt.coef);
      }
    }

    engine::QPModel qp;
    qp.sense = lp.sense;
    qp.c     = std::move(lp.c);
    qp.Q.resize(nv, nv);
    qp.Q.setFromTriplets(q_trip.begin(), q_trip.end());
    qp.A    = std::move(lp.A);
    qp.b    = std::move(lp.b);
    qp.Aeq  = std::move(lp.Aeq);
    qp.beq  = std::move(lp.beq);
    qp.vars = std::move(lp.vars);
    return qp;
  }

  bool is_milp() const {
    for (const auto& m : var_meta) {
      if (m.type == VarType::Integer || m.type == VarType::Binary)
        return true;
    }
    return false;
  }

  // ── Nonlinear expression arena helpers ───────────────────────────────────

  ExprId push_node(ExprNode n) {
    ExprId id = static_cast<ExprId>(expr_arena_.size());
    expr_arena_.push_back(std::move(n));
    return id;
  }

  // ── AD: symbolic variable support (compile-time, DAG walk) ───────────────
  // Returns the sorted list of VarId leaves reachable from `id` in the arena.
  // Used to pre-compute which variables each constraint depends on, enabling
  // sparse Jacobian emission without scanning all nv variables.
  static void nl_col_support(ExprId id,
                              const std::vector<ExprNode>& arena,
                              std::vector<bool>& visited,
                              std::vector<int>& vars) {
    if (id < 0 || static_cast<std::size_t>(id) >= arena.size()) return;
    if (visited[static_cast<std::size_t>(id)]) return;
    visited[static_cast<std::size_t>(id)] = true;
    const ExprNode& n = arena[static_cast<std::size_t>(id)];
    switch (n.op) {
      case ExprOp::Constant: break;
      case ExprOp::Variable: vars.push_back(n.var_id); break;
      default:
        nl_col_support(n.lhs_id, arena, visited, vars);
        nl_col_support(n.rhs_id, arena, visited, vars);
        break;
    }
  }

  static std::vector<int> nl_col_support_of(ExprId id,
                                              const std::vector<ExprNode>& arena) {
    std::vector<bool> visited(arena.size(), false);
    std::vector<int> vars;
    nl_col_support(id, arena, visited, vars);
    std::sort(vars.begin(), vars.end());
    vars.erase(std::unique(vars.begin(), vars.end()), vars.end());
    return vars;
  }

  // ── AD: forward eval (recursive with memoisation cache) ─────────────────

  static double nl_eval(ExprId id,
                        const std::vector<ExprNode>& arena,
                        const Eigen::VectorXd& x,
                        std::vector<double>& cache) {
    // Cache is initialised to NaN as sentinel "not yet computed".
    double& slot = cache[static_cast<std::size_t>(id)];
    if (!std::isnan(slot)) return slot;

    const ExprNode& n = arena[static_cast<std::size_t>(id)];
    double v = 0.0;
    switch (n.op) {
      case ExprOp::Constant: v = n.value; break;
      case ExprOp::Variable: v = x[n.var_id]; break;
      case ExprOp::Add:  v = nl_eval(n.lhs_id,arena,x,cache) + nl_eval(n.rhs_id,arena,x,cache); break;
      case ExprOp::Sub:  v = nl_eval(n.lhs_id,arena,x,cache) - nl_eval(n.rhs_id,arena,x,cache); break;
      case ExprOp::Neg:  v = -nl_eval(n.lhs_id,arena,x,cache); break;
      case ExprOp::Mul:  v = nl_eval(n.lhs_id,arena,x,cache) * nl_eval(n.rhs_id,arena,x,cache); break;
      case ExprOp::Div:  v = nl_eval(n.lhs_id,arena,x,cache) / nl_eval(n.rhs_id,arena,x,cache); break;
      case ExprOp::Pow2: { double a = nl_eval(n.lhs_id,arena,x,cache); v = a * a; break; }
      case ExprOp::PowN: {
        double a = nl_eval(n.lhs_id,arena,x,cache);
        int    e = static_cast<int>(n.var_id);   // exponent stored in var_id
        v = std::pow(a, static_cast<double>(e));
        break;
      }
      case ExprOp::Exp:  v = std::exp (nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Log:  v = std::log (nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Sqrt: v = std::sqrt(nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Sin:  v = std::sin (nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Cos:  v = std::cos (nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Tan:  v = std::tan (nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Abs:  v = std::fabs(nl_eval(n.lhs_id,arena,x,cache)); break;
      case ExprOp::Max2: v = std::max(nl_eval(n.lhs_id,arena,x,cache),nl_eval(n.rhs_id,arena,x,cache)); break;
      case ExprOp::Min2: v = std::min(nl_eval(n.lhs_id,arena,x,cache),nl_eval(n.rhs_id,arena,x,cache)); break;
    }
    slot = v;
    return v;
  }

  // Convenience: eval and fill cache, return value.
  static double nl_eval_cached(ExprId id,
                                const std::vector<ExprNode>& arena,
                                const Eigen::VectorXd& x,
                                std::vector<double>& cache) {
    cache.assign(arena.size(), std::numeric_limits<double>::quiet_NaN());
    return nl_eval(id, arena, x, cache);
  }

  // ── AD: reverse-mode backward pass (accumulates gradients) ──────────────

  static void nl_backward(ExprId id,
                           double adj,
                           const std::vector<ExprNode>& arena,
                           const std::vector<double>& cache,
                           Eigen::VectorXd& grad) {
    const ExprNode& n = arena[static_cast<std::size_t>(id)];
    switch (n.op) {
      case ExprOp::Constant: break;
      case ExprOp::Variable: grad[n.var_id] += adj; break;

      case ExprOp::Add:
        nl_backward(n.lhs_id, adj, arena, cache, grad);
        nl_backward(n.rhs_id, adj, arena, cache, grad);
        break;
      case ExprOp::Sub:
        nl_backward(n.lhs_id,  adj, arena, cache, grad);
        nl_backward(n.rhs_id, -adj, arena, cache, grad);
        break;
      case ExprOp::Neg:
        nl_backward(n.lhs_id, -adj, arena, cache, grad);
        break;
      case ExprOp::Mul:
        nl_backward(n.lhs_id, adj * cache[static_cast<std::size_t>(n.rhs_id)], arena, cache, grad);
        nl_backward(n.rhs_id, adj * cache[static_cast<std::size_t>(n.lhs_id)], arena, cache, grad);
        break;
      case ExprOp::Div: {
        double rhs_val = cache[static_cast<std::size_t>(n.rhs_id)];
        double lhs_val = cache[static_cast<std::size_t>(n.lhs_id)];
        nl_backward(n.lhs_id, adj / rhs_val, arena, cache, grad);
        nl_backward(n.rhs_id, -adj * lhs_val / (rhs_val * rhs_val), arena, cache, grad);
        break;
      }
      case ExprOp::Pow2:
        nl_backward(n.lhs_id, adj * 2.0 * cache[static_cast<std::size_t>(n.lhs_id)], arena, cache, grad);
        break;
      case ExprOp::PowN: {
        int    e   = static_cast<int>(n.var_id);
        double a   = cache[static_cast<std::size_t>(n.lhs_id)];
        double da  = (e == 0) ? 0.0 : static_cast<double>(e) * std::pow(a, static_cast<double>(e - 1));
        nl_backward(n.lhs_id, adj * da, arena, cache, grad);
        break;
      }
      case ExprOp::Exp:
        nl_backward(n.lhs_id, adj * cache[static_cast<std::size_t>(id)], arena, cache, grad); // d/dx e^x = e^x
        break;
      case ExprOp::Log:
        nl_backward(n.lhs_id, adj / cache[static_cast<std::size_t>(n.lhs_id)], arena, cache, grad);
        break;
      case ExprOp::Sqrt:
        nl_backward(n.lhs_id, adj / (2.0 * cache[static_cast<std::size_t>(id)]), arena, cache, grad);
        break;
      case ExprOp::Sin:
        nl_backward(n.lhs_id, adj * std::cos(cache[static_cast<std::size_t>(n.lhs_id)]), arena, cache, grad);
        break;
      case ExprOp::Cos:
        nl_backward(n.lhs_id, adj * -std::sin(cache[static_cast<std::size_t>(n.lhs_id)]), arena, cache, grad);
        break;
      case ExprOp::Tan: {
        double c = std::cos(cache[static_cast<std::size_t>(n.lhs_id)]);
        nl_backward(n.lhs_id, adj / (c * c), arena, cache, grad);
        break;
      }
      case ExprOp::Abs: {
        double a = cache[static_cast<std::size_t>(n.lhs_id)];
        nl_backward(n.lhs_id, adj * (a >= 0.0 ? 1.0 : -1.0), arena, cache, grad);
        break;
      }
      case ExprOp::Max2:
        if (cache[static_cast<std::size_t>(n.lhs_id)] >= cache[static_cast<std::size_t>(n.rhs_id)])
          nl_backward(n.lhs_id, adj, arena, cache, grad);
        else
          nl_backward(n.rhs_id, adj, arena, cache, grad);
        break;
      case ExprOp::Min2:
        if (cache[static_cast<std::size_t>(n.lhs_id)] <= cache[static_cast<std::size_t>(n.rhs_id)])
          nl_backward(n.lhs_id, adj, arena, cache, grad);
        else
          nl_backward(n.rhs_id, adj, arena, cache, grad);
        break;
    }
  }

  // ── Gradient of one expression w.r.t. x (reverse-mode AD) ───────────────

  static void nl_gradient(ExprId id,
                           const std::vector<ExprNode>& arena,
                           const Eigen::VectorXd& x,
                           int /*nv*/,
                           double adj_scale,
                           Eigen::VectorXd& grad) {
    std::vector<double> cache(arena.size(), std::numeric_limits<double>::quiet_NaN());
    nl_eval(id, arena, x, cache);
    nl_backward(id, adj_scale, arena, cache, grad);
  }

  // ── compile_nlp: assemble engine::NLPModel from the arena ───────────────

  engine::NLPModel compile_nlp() const {
    const int nv = next_var_id;

    // Collect variable meta (reuse compile_lp logic for bounds)
    std::unordered_map<VarId, std::pair<double,double>> bound_overrides;
    for (const auto& arr : var_arrays) {
      for (const auto& [k, vid] : arr->key_to_id()) {
        bound_overrides[vid] = {arr->lb(k), arr->ub(k)};
      }
    }
    std::vector<engine::VariableMeta> vmeta(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) {
      const auto& m = var_meta[static_cast<std::size_t>(i)];
      auto it = bound_overrides.find(i);
      if (it != bound_overrides.end()) {
        vmeta[i].lb = it->second.first;
        vmeta[i].ub = it->second.second;
      } else {
        vmeta[i].lb = m.lb;
        vmeta[i].ub = m.ub;
      }
      vmeta[i].name = m.family;
      vmeta[i].type = engine::VarType::Continuous; // NLP only supports continuous
    }

    // Capture arena by copy for use in lambdas
    std::vector<ExprNode> arena = expr_arena_;

    engine::NLPModel nlp;
    nlp.vars  = std::move(vmeta);
    nlp.sense = sense;

    // ── Objective callbacks ───────────────────────────────────────────────
    const ExprId obj_id   = nl_obj_expr_.id();
    const engine::Sense obj_sense = sense;

    nlp.f = [arena, obj_id, obj_sense](const Eigen::VectorXd& x) -> double {
      std::vector<double> cache(arena.size(), std::numeric_limits<double>::quiet_NaN());
      double val = nl_eval(obj_id, arena, x, cache);
      return (obj_sense == engine::Sense::Maximize) ? -val : val;
    };

    nlp.grad = [arena, obj_id, obj_sense, nv](const Eigen::VectorXd& x,
                                               Eigen::VectorXd& g) {
      g = Eigen::VectorXd::Zero(nv);
      double adj = (obj_sense == engine::Sense::Maximize) ? -1.0 : 1.0;
      nl_gradient(obj_id, arena, x, nv, adj, g);
    };

    // ── Collect all equality and inequality constraint rows ───────────────
    // Linear rows (from `rows`) + nonlinear rows (from `nl_rows_`)

    // --- 1: build info vectors ---
    struct ConInfo {
      bool is_linear;
      // linear:
      std::vector<std::pair<VarId,double>> terms;
      double lin_rhs;
      // nonlinear:
      ExprId expr_id;
      double nl_rhs;
      CompareOp sense;
      // Symbolic variable support (compile-time DAG walk) — for sparse Jacobian.
      // For linear rows this is empty; support is read from `terms` directly.
      std::vector<int> nl_var_support;
    };

    std::vector<ConInfo> eq_cons, ineq_cons;

    // Process linear rows
    for (const auto& r : rows) {
      LinearExpr cl = r.lhs;
      cl -= r.rhs;
      // aggregate terms
      auto terms = cl.terms;
      std::sort(terms.begin(), terms.end(),
                [](const auto& a, const auto& b){ return a.first < b.first; });
      std::vector<std::pair<VarId,double>> agg;
      for (const auto& [v,c] : terms) {
        if (!agg.empty() && agg.back().first == v) agg.back().second += c;
        else agg.emplace_back(v,c);
      }
      double rhs_val = -cl.constant;

      ConInfo ci;
      ci.is_linear = true;
      ci.terms     = std::move(agg);
      ci.lin_rhs   = rhs_val;
      ci.sense     = r.sense;
      // for inequality: GreaterEq flipped to LessEq (negated row)
      if (r.sense == CompareOp::Equal) {
        eq_cons.push_back(std::move(ci));
      } else {
        if (r.sense == CompareOp::GreaterEq) {
          for (auto& [v,c] : ci.terms) c = -c;
          ci.lin_rhs = -ci.lin_rhs;
          ci.sense   = CompareOp::LessEq;
        }
        ineq_cons.push_back(std::move(ci));
      }
    }

    // Process nonlinear rows
    for (const auto& r : nl_rows_) {
      ConInfo ci;
      ci.is_linear      = false;
      ci.expr_id        = r.expr_id;
      ci.nl_rhs         = r.rhs;
      ci.sense          = r.sense;
      ci.nl_var_support = nl_col_support_of(r.expr_id, arena);
      if (r.sense == CompareOp::Equal) {
        eq_cons.push_back(std::move(ci));
      } else {
        ineq_cons.push_back(std::move(ci));
      }
    }

    const int meq   = static_cast<int>(eq_cons.size());
    const int mineq = static_cast<int>(ineq_cons.size());

    // --- 2: equality callbacks ---
    if (meq > 0) {
      nlp.g = [eq_cons, arena, meq](const Eigen::VectorXd& x,
                                         Eigen::VectorXd& geq) {
        geq.resize(meq);
        for (int i = 0; i < meq; ++i) {
          const auto& ci = eq_cons[static_cast<std::size_t>(i)];
          if (ci.is_linear) {
            double val = 0.0;
            for (const auto& [v,c] : ci.terms) val += c * x[v];
            geq[i] = val - ci.lin_rhs;
          } else {
            std::vector<double> cache(arena.size(), std::numeric_limits<double>::quiet_NaN());
            geq[i] = nl_eval(ci.expr_id, arena, x, cache) - ci.nl_rhs;
          }
        }
      };

      nlp.jac_g = [eq_cons, arena, nv, meq](const Eigen::VectorXd& x,
                                              Eigen::SparseMatrix<double>& J) {
        using T3 = Eigen::Triplet<double>;
        std::vector<T3> trip;
        trip.reserve(static_cast<std::size_t>(meq * 4));
        for (int i = 0; i < meq; ++i) {
          const auto& ci = eq_cons[static_cast<std::size_t>(i)];
          if (ci.is_linear) {
            for (const auto& [v,c] : ci.terms)
              if (std::fabs(c) >= 1e-14) trip.emplace_back(i, v, c);
          } else {
            // Sparse gradient: only compute for variables in the support set.
            // This is O(|support|) rather than O(nv), critical for large cases.
            Eigen::VectorXd row_grad = Eigen::VectorXd::Zero(nv);
            nl_gradient(ci.expr_id, arena, x, nv, 1.0, row_grad);
            // Emit all support vars (including structurally nonzero entries
            // that happen to be numerically zero, so the adapter's sparsity
            // pattern is complete at any point including flat-start).
            for (int v : ci.nl_var_support)
              trip.emplace_back(i, v, row_grad[v]);
          }
        }
        J.resize(meq, nv);
        J.setFromTriplets(trip.begin(), trip.end());
      };
    }

    // --- 3: inequality callbacks  (h(x) <= 0) ---
    if (mineq > 0) {
      nlp.h = [ineq_cons, arena, mineq](const Eigen::VectorXd& x,
                                              Eigen::VectorXd& hv) {
        hv.resize(mineq);
        for (int i = 0; i < mineq; ++i) {
          const auto& ci = ineq_cons[static_cast<std::size_t>(i)];
          if (ci.is_linear) {
            // row <= rhs  →  row - rhs <= 0  →  h[i] = Σ c*x - rhs
            double val = 0.0;
            for (const auto& [v,c] : ci.terms) val += c * x[v];
            hv[i] = val - ci.lin_rhs;
          } else {
            std::vector<double> cache(arena.size(), std::numeric_limits<double>::quiet_NaN());
            double eval_val = nl_eval(ci.expr_id, arena, x, cache);
            if (ci.sense == CompareOp::LessEq)    hv[i] =  eval_val - ci.nl_rhs;
            else /* GreaterEq */                  hv[i] = -eval_val + ci.nl_rhs;
          }
        }
      };

      nlp.jac_h = [ineq_cons, arena, nv, mineq](const Eigen::VectorXd& x,
                                                  Eigen::SparseMatrix<double>& J) {
        using T3 = Eigen::Triplet<double>;
        std::vector<T3> trip;
        trip.reserve(static_cast<std::size_t>(mineq * 4));
        for (int i = 0; i < mineq; ++i) {
          const auto& ci = ineq_cons[static_cast<std::size_t>(i)];
          if (ci.is_linear) {
            for (const auto& [v,c] : ci.terms)
              if (std::fabs(c) >= 1e-14) trip.emplace_back(i, v, c);
          } else {
            Eigen::VectorXd row_grad = Eigen::VectorXd::Zero(nv);
            double sign = (ci.sense == CompareOp::LessEq) ? 1.0 : -1.0;
            nl_gradient(ci.expr_id, arena, x, nv, sign, row_grad);
            for (int v : ci.nl_var_support)
              trip.emplace_back(i, v, row_grad[v]);
          }
        }
        J.resize(mineq, nv);
        J.setFromTriplets(trip.begin(), trip.end());
      };
    }

    // ── Initial point ────────────────────────────────────────────────────
    if (static_cast<int>(nlp_x0_.size()) == nv) {
      nlp.x0 = Eigen::Map<const Eigen::VectorXd>(nlp_x0_.data(), nv);
    } else {
      // default: midpoint clipped to bounds
      nlp.x0.resize(nv);
      for (int i = 0; i < nv; ++i) {
        double lo = nlp.vars[static_cast<std::size_t>(i)].lb;
        double hi = nlp.vars[static_cast<std::size_t>(i)].ub;
        if (!std::isfinite(lo)) lo = -1.0;
        if (!std::isfinite(hi)) hi =  1.0;
        nlp.x0[i] = 0.5 * (lo + hi);
      }
    }

    return nlp;
  }

  SolveResult build_result(const engine::api::Result& r,
                            bool milp_mode) const {
    SolveResult sr;
    sr.solve_time_sec = r.stats.runtime_sec;
    sr.solver_used    = r.stats.solver_name;

    if (r.stats.success) {
      sr.objective_value = r.stats.objective;
      sr.optimality_gap  = r.stats.mip_gap;
      sr.objective_bound = r.stats.objective - r.stats.mip_gap * std::max(1.0, std::fabs(r.stats.objective));

      sr.termination_status = TerminationStatus::Optimal;
      sr.primal_status      = PrimalStatus::Optimal;
      sr.dual_status = (!milp_mode && r.constraint_duals.size() > 0)
                           ? DualStatus::Optimal
                           : DualStatus::NoSolution;
    } else {
      // Map status string heuristically
      const auto& st = r.stats.status;
      if (st.find("infeasible") != std::string::npos ||
          st.find("Infeasible") != std::string::npos) {
        sr.termination_status = TerminationStatus::Infeasible;
        sr.primal_status      = PrimalStatus::Infeasible;
        sr.dual_status        = DualStatus::NoSolution;
      } else if (st.find("time") != std::string::npos ||
                 st.find("Time") != std::string::npos) {
        sr.termination_status = TerminationStatus::TimeLimit;
        // May still have a feasible incumbent
        if (r.x.size() == next_var_id) {
          sr.primal_status = PrimalStatus::Feasible;
          sr.objective_value = r.stats.objective;
        } else {
          sr.primal_status = PrimalStatus::NoSolution;
        }
        sr.dual_status = DualStatus::NoSolution;
      } else {
        // For NLP solvers that exceeded iteration count, accept the best iterate
        // if it is practically feasible (primal residual < 1e-4 p.u.).
        // This is analogous to Ipopt's "acceptable" convergence level.
        const bool near_feasible =
            !milp_mode &&
            r.x.size() == static_cast<Eigen::Index>(next_var_id) &&
            r.stats.primal_feas > 0.0 &&
            r.stats.primal_feas < 1e-4 &&
            std::isfinite(r.stats.objective);
        if (near_feasible) {
          sr.termination_status = TerminationStatus::Unknown;
          sr.primal_status      = PrimalStatus::Feasible;
          sr.objective_value    = r.stats.objective;
          sr.optimality_gap     = 1.0;  // optimality not certified
          sr.dual_status        = DualStatus::NoSolution;
        } else {
          sr.termination_status = TerminationStatus::Unknown;
          sr.primal_status      = PrimalStatus::Unknown;
          sr.dual_status        = DualStatus::Unknown;
        }
      }
    }

    // Fill primal values
    if (r.x.size() == next_var_id) {
      for (int i = 0; i < next_var_id; ++i) {
        sr.primal_vals[i] = r.x[i];
      }
    }

    // Fill duals (LP only)
    // Dual filling: only safe when all constraints are linear (LP path).
    // For pure-NLP models every constraint lives in nl_rows_ while rows is
    // empty; iterating up to next_con_id would index past rows.end().
    if (!milp_mode && r.constraint_duals.size() > 0 && !rows.empty()) {
      // Duals come in the same row order as we assembled (ineq first, then eq)
      // We stored rows in the order they were added; re-map here.
      // Re-count the ineq/eq split to map ConId → dual vector index.
      int ii = 0, ei = 0;
      int nc_ineq = 0;
      for (const auto& row : rows) {
        if (row.sense != CompareOp::Equal) ++nc_ineq;
      }
      const int n_linear = static_cast<int>(rows.size());
      for (ConId cid = 0; cid < n_linear; ++cid) {
        const auto& row = rows[static_cast<std::size_t>(cid)];
        int dual_idx;
        if (row.sense != CompareOp::Equal) {
          dual_idx = ii++;
        } else {
          dual_idx = nc_ineq + ei++;
        }
        if (dual_idx < static_cast<int>(r.constraint_duals.size())) {
          double raw = r.constraint_duals[dual_idx];
          // Flip sign for >= rows (we negated the row during compile)
          if (row.sense == CompareOp::GreaterEq) raw = -raw;
          sr.dual_vals[cid] = raw;
        }
      }
    }
    return sr;
  }
};

// ════════════════════════════════════════════════════════════════════════════
// Model public API implementation
// ════════════════════════════════════════════════════════════════════════════

Model::Model(const std::string& name)
    : impl_(std::make_unique<ModelImpl>()) {
  impl_->model_name = name;
}
Model::~Model() = default;
Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;

const std::string& Model::name() const noexcept { return impl_->model_name; }

// ── Sets ─────────────────────────────────────────────────────────────────

ExplicitSet& Model::add_set(const std::string& name,
                             const std::vector<Atom>& atoms) {
  impl_->explicit_sets.push_back(
      std::make_unique<ExplicitSet>(name, atoms));
  return *impl_->explicit_sets.back();
}

ExplicitSet& Model::add_set_nd(const std::string& name, int dim,
                                const std::vector<Key>& elems) {
  impl_->explicit_sets.push_back(
      std::make_unique<ExplicitSet>(name, dim, elems));
  return *impl_->explicit_sets.back();
}

OrderedSet& Model::add_ordered_set(const std::string& name,
                                    const std::vector<Atom>& atoms) {
  impl_->ordered_sets.push_back(
      std::make_unique<OrderedSet>(name, atoms));
  return *impl_->ordered_sets.back();
}

// ── Parameters ──────────────────────────────────────────────────────────

Parameter& Model::add_param_scalar(const std::string& name,
                                    const std::string& unit) {
  impl_->params.push_back(
      std::make_unique<Parameter>(name, 0, unit));
  return *impl_->params.back();
}

Parameter& Model::add_param(const std::string& name, int dim,
                              const std::string& unit) {
  impl_->params.push_back(
      std::make_unique<Parameter>(name, dim, unit));
  return *impl_->params.back();
}

// ── Variables ────────────────────────────────────────────────────────────

VarArray& Model::add_var(const std::string& name, const Set& domain,
                          VarType type, double lb, double ub) {
  auto arr = std::make_unique<VarArray>(name, type, lb, ub);
  return impl_->register_vars(std::move(arr), domain.elements());
}

VarArray& Model::add_var(const std::string& name,
                          const Set& domain_a, const Set& domain_b,
                          VarType type, double lb, double ub) {
  CartesianProductSet prod(
      // We need shared_ptrs for CartesianProductSet, but Set& are raw refs.
      // Make non-owning shared_ptrs that will never delete.
      std::shared_ptr<Set>(const_cast<Set*>(&domain_a), [](Set*) {}),
      std::shared_ptr<Set>(const_cast<Set*>(&domain_b), [](Set*) {}));
  auto arr = std::make_unique<VarArray>(name, type, lb, ub);
  return impl_->register_vars(std::move(arr), prod.elements());
}

// ── Objective ────────────────────────────────────────────────────────────

void Model::minimize(const LinearExpr& obj) {
  impl_->obj_expr      = obj;
  impl_->has_quad_obj  = false;
  impl_->sense         = engine::Sense::Minimize;
}
void Model::maximize(const LinearExpr& obj) {
  impl_->obj_expr      = obj;
  impl_->has_quad_obj  = false;
  impl_->sense         = engine::Sense::Maximize;
}

void Model::minimize(const QuadExpr& obj) {
  impl_->quad_obj_expr = obj;
  impl_->obj_expr      = obj.linear_part;   // keep for compile_lp linear vector
  impl_->has_quad_obj  = true;
  impl_->has_nl_obj    = false;
  impl_->sense         = engine::Sense::Minimize;
}
void Model::maximize(const QuadExpr& obj) {
  impl_->quad_obj_expr = obj;
  impl_->obj_expr      = obj.linear_part;
  impl_->has_quad_obj  = true;
  impl_->has_nl_obj    = false;
  impl_->sense         = engine::Sense::Maximize;
}

void Model::minimize(const NonlinearExpr& obj) {
  impl_->nl_obj_expr_ = obj;
  impl_->has_nl_obj   = true;
  impl_->has_quad_obj = false;
  impl_->sense        = engine::Sense::Minimize;
}
void Model::maximize(const NonlinearExpr& obj) {
  impl_->nl_obj_expr_ = obj;
  impl_->has_nl_obj   = true;
  impl_->has_quad_obj = false;
  impl_->sense        = engine::Sense::Maximize;
}

// ── Nonlinear expression builders ────────────────────────────────────────

namespace {
// Helper to push a unary node.
inline NonlinearExpr push1(ModelImpl* impl, ExprOp op, NonlinearExpr a) {
  ExprNode n;
  n.op     = op;
  n.lhs_id = a.id();
  return NonlinearExpr(impl->push_node(std::move(n)));
}
// Helper to push a binary node.
inline NonlinearExpr push2(ModelImpl* impl, ExprOp op, NonlinearExpr a, NonlinearExpr b) {
  ExprNode n;
  n.op     = op;
  n.lhs_id = a.id();
  n.rhs_id = b.id();
  return NonlinearExpr(impl->push_node(std::move(n)));
}
}  // namespace

NonlinearExpr Model::nl_var(VarRef v) {
  ExprNode n;
  n.op     = ExprOp::Variable;
  n.var_id = v.id();
  return NonlinearExpr(impl_->push_node(std::move(n)));
}

NonlinearExpr Model::nl_const(double c) {
  ExprNode n;
  n.op    = ExprOp::Constant;
  n.value = c;
  return NonlinearExpr(impl_->push_node(std::move(n)));
}

NonlinearExpr Model::nl_neg (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Neg,  a); }
NonlinearExpr Model::nl_add (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Add,  a, b); }
NonlinearExpr Model::nl_sub (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Sub,  a, b); }
NonlinearExpr Model::nl_mul (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Mul,  a, b); }
NonlinearExpr Model::nl_div (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Div,  a, b); }
NonlinearExpr Model::nl_sq  (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Pow2, a); }
NonlinearExpr Model::nl_sqrt(NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Sqrt, a); }
NonlinearExpr Model::nl_exp (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Exp,  a); }
NonlinearExpr Model::nl_log (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Log,  a); }
NonlinearExpr Model::nl_sin (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Sin,  a); }
NonlinearExpr Model::nl_cos (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Cos,  a); }
NonlinearExpr Model::nl_tan (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Tan,  a); }
NonlinearExpr Model::nl_abs (NonlinearExpr a)               { return push1(impl_.get(), ExprOp::Abs,  a); }
NonlinearExpr Model::nl_max (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Max2, a, b); }
NonlinearExpr Model::nl_min (NonlinearExpr a, NonlinearExpr b) { return push2(impl_.get(), ExprOp::Min2, a, b); }

NonlinearExpr Model::nl_pow(NonlinearExpr a, int n) {
  if (n == 2) return nl_sq(a);
  ExprNode nd;
  nd.op     = ExprOp::PowN;
  nd.lhs_id = a.id();
  nd.var_id = static_cast<VarId>(n);   // exponent stored in var_id slot
  return NonlinearExpr(impl_->push_node(std::move(nd)));
}

// ── Nonlinear constraints ─────────────────────────────────────────────────

ConstraintRef Model::add_nl_constraint(const std::string& name,
                                        NonlinearExpr expr,
                                        CompareOp     sense,
                                        double        rhs) {
  ConId id = impl_->next_con_id++;
  impl_->nl_rows_.push_back({expr.id(), sense, rhs, name});
  return ConstraintRef(id, name);
}

void Model::set_nlp_x0(const std::vector<double>& x0) {
  impl_->nlp_x0_ = x0;
}



// ── Constraints ──────────────────────────────────────────────────────────

ConstraintRef Model::add_constraint(const TempConstr& c,
                                     const std::string& name) {
  ConId id = impl_->add_row(c, name);
  return ConstraintRef(id, name);
}

ConstraintArray& Model::create_constraint_array(const std::string& name) {
  return impl_->get_or_create_con_array(name);
}

ConId Model::add_constraint_internal(const TempConstr& c,
                                      const std::string& name) {
  return impl_->add_row(c, name);
}

// ── Solve ────────────────────────────────────────────────────────────────

SolveResult Model::solve(const SolveOptions& opts) const {
  bool milp = impl_->is_milp();
  bool qp   = impl_->has_quad_obj;
  bool nlp  = impl_->has_nl_obj;

  engine::SolveOptions eng_opts;
  eng_opts.preferred_solver = opts.solver_name;
  eng_opts.allow_fallback   = opts.allow_fallback;

  engine::SolverEngine engine(/*register_defaults=*/true);

  if (nlp) {
    // NLP path: assemble NLPModel from arena and dispatch
    engine::NLPModel nlpm = impl_->compile_nlp();
    auto result = engine.solve_nlp(nlpm, eng_opts);
    return impl_->build_result(result, /*milp=*/false);
  } else if (!milp && qp) {
    // Pure QP (convex): dispatch to solve_qp
    engine::QPModel qpm = impl_->compile_qp();
    auto result = engine.solve_qp(qpm, eng_opts);
    return impl_->build_result(result, /*milp=*/false);
  } else if (milp) {
    engine::LPModel lp  = impl_->compile_lp();
    engine::MIPModel mp;
    mp.linear_part = std::move(lp);
    for (int i = 0; i < impl_->next_var_id; ++i) {
      const auto& m = impl_->var_meta[static_cast<std::size_t>(i)];
      if (m.type == VarType::Integer) mp.integer_idx.push_back(i);
      if (m.type == VarType::Binary)  mp.binary_idx.push_back(i);
    }
    auto result = engine.solve_milp(mp, eng_opts);
    return impl_->build_result(result, /*milp=*/true);
  } else {
    engine::LPModel lp = impl_->compile_lp();
    auto result = engine.solve_lp(lp, eng_opts);
    return impl_->build_result(result, /*milp=*/false);
  }
}

// ── Inspection ────────────────────────────────────────────────────────────

int Model::num_vars()        const noexcept { return impl_->next_var_id; }
int Model::num_constraints() const noexcept { return impl_->next_con_id; }

void Model::print_summary() const {
  std::cout << "Model '" << impl_->model_name << "'"
            << "  vars=" << num_vars()
            << "  cons=" << num_constraints() << "\n";
}

void Model::check_bounds() const {
  for (const auto& m : impl_->var_meta) {
    if (m.lb > m.ub) {
      throw std::domain_error(
          "Model '" + impl_->model_name + "': var '" + m.family +
          "' has lb=" + std::to_string(m.lb) +
          " > ub=" + std::to_string(m.ub));
    }
  }
}

void Model::check_missing_params() const {
  // Nothing to check unless users explicitly register domain keys with params.
  // This is a no-op stub; full checking is domain-builder responsibility.
}

void Model::write_lp(const std::string& path) const {
  // Minimal stub — LP file export is handled by engine adapters in the future.
  (void)path;
  throw std::logic_error("Model::write_lp: not yet implemented");
}
void Model::write_mps(const std::string& path) const {
  (void)path;
  throw std::logic_error("Model::write_mps: not yet implemented");
}

// ── JSON export (Beta milestone) ─────────────────────────────────────────
void Model::write_json(const std::string& path) const {
  std::ofstream f(path);
  if (!f.is_open())
    throw std::runtime_error("Model::write_json: cannot open " + path);

  auto escape = [](const std::string& s) {
    std::string r; r.reserve(s.size());
    for (char c : s) {
      if      (c == '"')  r += "\\\"";
      else if (c == '\\') r += "\\\\";
      else if (c == '\n') r += "\\n";
      else                r += c;
    }
    return r;
  };

  auto key_str = [](const Key& k) {
    std::string s;
    for (std::size_t i = 0; i < k.values.size(); ++i) {
      if (i) s += "|";
      s += k.values[i];
    }
    return s;
  };

  f << "{\n";
  f << "  \"name\": \"" << escape(impl_->model_name) << "\",\n";
  f << "  \"sense\": \""
    << (impl_->sense == engine::Sense::Minimize ? "minimize" : "maximize")
    << "\",\n";
  f << "  \"num_vars\": " << impl_->next_var_id << ",\n";
  f << "  \"num_constraints\": " << impl_->next_con_id << ",\n";

  // Variables
  f << "  \"variables\": [\n";
  bool first = true;
  for (const auto& arr : impl_->var_arrays) {
    for (const auto& [k, vid] : arr->key_to_id()) {
      if (!first) f << ",\n";
      const std::string type_str =
          (arr->type() == VarType::Binary)     ? "binary"     :
          (arr->type() == VarType::Integer)    ? "integer"    : "continuous";
      f << "    {\"id\":" << vid
        << ",\"name\":\"" << escape(arr->name()) << "\""
        << ",\"key\":\"" << escape(key_str(k)) << "\""
        << ",\"type\":\"" << type_str << "\""
        << ",\"lb\":" << arr->lb(k)
        << ",\"ub\":" << arr->ub(k) << "}";
      first = false;
    }
  }
  f << "\n  ],\n";

  // Sets
  f << "  \"sets\": [\n";
  first = true;
  for (const auto& s : impl_->explicit_sets) {
    if (!first) f << ",\n";
    f << "    {\"name\":\"" << escape(s->name()) << "\""
      << ",\"cardinality\":" << s->cardinality() << "}";
    first = false;
  }
  for (const auto& s : impl_->ordered_sets) {
    if (!first) f << ",\n";
    f << "    {\"name\":\"" << escape(s->name()) << "\""
      << ",\"cardinality\":" << s->cardinality()
      << ",\"ordered\":true}";
    first = false;
  }
  f << "\n  ],\n";

  // Objective terms
  f << "  \"objective\": {";
  f << "\"has_quadratic\":" << (impl_->has_quad_obj ? "true" : "false");
  f << ",\"num_linear_terms\":" << impl_->obj_expr.terms.size();
  if (impl_->has_quad_obj)
    f << ",\"num_quad_terms\":" << impl_->quad_obj_expr.quad_terms.size();
  f << "},\n";

  // Constraints (names only for compactness)
  f << "  \"constraints\": [\n";
  first = true;
  for (const auto& r : impl_->rows) {
    if (!first) f << ",\n";
    const std::string sense_str =
        (r.sense == CompareOp::LessEq)    ? "<=" :
        (r.sense == CompareOp::GreaterEq) ? ">=" : "==";
    f << "    {\"name\":\"" << escape(r.name) << "\""
      << ",\"sense\":\"" << sense_str << "\"}";
    first = false;
  }
  f << "\n  ]\n}\n";
}

}  // namespace mipsolvers::aml

// Optimized LCQP solver using primal-dual interior point method
// Solves: min 0.5*x'Qx + c'x  s.t. Aeq*x = beq, lb <= x <= ub
//
// Optimizations:
// - Symbolic factorization cached (pattern never changes, only D diagonal values)
// - Caller-visible Mehrotra centering policy: sigma=(mu_aff/mu)^p
//
// This is a high-performance implementation for DC OPF problems.

#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

namespace mipsolvers::engine {

namespace {
double minimum_safe_positive() {
  return std::sqrt(std::numeric_limits<double>::min());
}

double summation_roundoff_bound(int term_count, double absolute_term_sum) {
  if (term_count <= 0 || !(absolute_term_sum > 0.0)) return 0.0;
  const double accumulated_error =
      static_cast<double>(term_count) * std::numeric_limits<double>::epsilon();
  if (!(accumulated_error < 1.0)) {
    return std::numeric_limits<double>::infinity();
  }
  return accumulated_error / (1.0 - accumulated_error) * absolute_term_sum;
}

struct LCQPProfile {
  bool enabled{std::getenv("MIPSOLVERS_LCQP_PROF") != nullptr};
  std::chrono::steady_clock::time_point started{
      std::chrono::steady_clock::now()};
  double setup_seconds{0.0};
  double pattern_seconds{0.0};
  double analyze_seconds{0.0};
  double assembly_seconds{0.0};
  double factor_seconds{0.0};
  double solve_seconds{0.0};
  int dimension{0};
  int inequalities{0};
  int iterations{0};
  int factorizations{0};
  int solves{0};
  const char* backend{"none"};

  ~LCQPProfile() {
    if (!enabled) return;
    const double total = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::fprintf(stderr,
                 "[lcqp-prof] n=%d mineq=%d iter=%d backend=%s "
                 "total=%.6fs setup=%.6fs pattern=%.6fs analyze=%.6fs "
                 "assembly=%.6fs factor=%.6fs/%d solve=%.6fs/%d\n",
                 dimension, inequalities, iterations, backend, total,
                 setup_seconds, pattern_seconds, analyze_seconds,
                 assembly_seconds, factor_seconds, factorizations,
                 solve_seconds, solves);
  }
};

// Compute max step α s.t. v + α*dv > 0
double max_step_pos(const Eigen::VectorXd& v, const Eigen::VectorXd& dv,
                    double interior_fraction) {
  double alpha = 1.0;
  for (int i = 0; i < v.size(); ++i) {
    // Every negative component can hit the cone boundary.  Ignoring a small
    // negative direction is unsafe when the corresponding slack or dual is
    // even smaller, and can turn complementarity negative on the update.
    if (dv(i) < 0.0) {
      alpha = std::min(alpha, -interior_fraction * v(i) / dv(i));
    }
  }
  return std::clamp(alpha, 0.0, 1.0);
}

/// Ruiz equilibration for the quadratic, equality, and inequality blocks.
/// Returns column scaling vector d such that the scaled problem has better conditioning.
/// Scaled problem: variables x_s = D^{-1} x, where D = diag(d).
///   min 0.5 x_s' (D Q D) x_s + (D c)' x_s
///   s.t. (Aeq D) x_s = beq,   D^{-1} lb <= x_s <= D^{-1} ub
struct RuizScaling {
  Eigen::VectorXd col_scale;  // n-vector: x_original = diag(col_scale) * x_scaled
  Eigen::VectorXd equality_row_scale;
  Eigen::VectorXd inequality_row_scale;
};

RuizScaling compute_ruiz_scaling(const Eigen::SparseMatrix<double>& Q,
                                 const Eigen::SparseMatrix<double>& Aeq,
                                 const Eigen::SparseMatrix<double>& A,
                                 int n, int meq, int mineq,
                                 int iteration_budget) {
  Eigen::VectorXd col_scale = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd equality_row_scale = Eigen::VectorXd::Ones(meq);
  Eigen::VectorXd inequality_row_scale = Eigen::VectorXd::Ones(mineq);

  const double scaling_resolution =
      std::sqrt(std::numeric_limits<double>::epsilon());
  for (int r = 0; r < iteration_budget; ++r) {
    double maximum_log_update = 0.0;
    // Column scaling from the *effectively-scaled* matrices.
    // Scaled Q[i,j] = col_scale[i] * Q[i,j] * col_scale[j]
    // Scaled Aeq[i,j] = row_scale[i] * Aeq[i,j] * col_scale[j]
    Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);
    for (int col = 0; col < Q.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(Q, col); it; ++it) {
        double v = std::abs(it.value()) * col_scale[it.row()] * col_scale[col];
        col_max[col] = std::max(col_max[col], v);
      }
    }
    for (int col = 0; col < Aeq.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq, col); it; ++it) {
        double v = std::abs(it.value()) *
                   equality_row_scale[it.row()] * col_scale[col];
        col_max[col] = std::max(col_max[col], v);
      }
    }
    for (int col = 0; col < A.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, col); it; ++it) {
        double v = std::abs(it.value()) *
                   inequality_row_scale[it.row()] * col_scale[col];
        col_max[col] = std::max(col_max[col], v);
      }
    }
    for (int j = 0; j < n; ++j) {
      if (col_max[j] > minimum_safe_positive()) {
        double s = 1.0 / std::sqrt(col_max[j]);
        col_scale[j] *= s;
        maximum_log_update =
            std::max(maximum_log_update, std::abs(std::log(s)));
      }
    }

    // Row scaling for Aeq from *effectively-scaled* matrix.
    if (meq > 0) {
      Eigen::VectorXd row_max = Eigen::VectorXd::Zero(meq);
      for (int col = 0; col < Aeq.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq, col); it; ++it) {
          double v = std::abs(it.value()) *
                     equality_row_scale[it.row()] * col_scale[col];
          row_max[it.row()] = std::max(row_max[it.row()], v);
        }
      }
      for (int i = 0; i < meq; ++i) {
        if (row_max[i] > minimum_safe_positive()) {
          double s = 1.0 / std::sqrt(row_max[i]);
          equality_row_scale[i] *= s;
          maximum_log_update =
              std::max(maximum_log_update, std::abs(std::log(s)));
        }
      }
    }
    if (mineq > 0) {
      Eigen::VectorXd row_max = Eigen::VectorXd::Zero(mineq);
      for (int col = 0; col < A.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, col); it; ++it) {
          double v = std::abs(it.value()) *
                     inequality_row_scale[it.row()] * col_scale[col];
          row_max[it.row()] = std::max(row_max[it.row()], v);
        }
      }
      for (int i = 0; i < mineq; ++i) {
        if (row_max[i] > minimum_safe_positive()) {
          const double s = 1.0 / std::sqrt(row_max[i]);
          inequality_row_scale[i] *= s;
          maximum_log_update =
              std::max(maximum_log_update, std::abs(std::log(s)));
        }
      }
    }
    if (maximum_log_update <= scaling_resolution) break;
  }
  return {col_scale, equality_row_scale, inequality_row_scale};
}

double max_step_to_boundary(const Eigen::VectorXd& v,
                            const Eigen::VectorXd& dv) {
  double alpha = 1.0;
  for (int i = 0; i < v.size(); ++i) {
    if (dv[i] < 0.0) alpha = std::min(alpha, -v[i] / dv[i]);
  }
  return std::clamp(alpha, 0.0, 1.0);
}

}  // namespace

NativeLCQPAdapter::NativeLCQPAdapter(LCQPOptions opt) : opt_(std::move(opt)) {}

std::string NativeLCQPAdapter::name() const { return "NativeLCQP"; }

bool NativeLCQPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::QP || cls == ProblemClass::LP;
}

SolveResult NativeLCQPAdapter::solve_lp(const LPModel& prob) const {
  const int n       = static_cast<int>(prob.c.size());
  const int m_ineq  = static_cast<int>(prob.A.rows());

  if (m_ineq == 0) {
    // No inequality constraints — pass through directly.
    QPModel qp;
    qp.sense = prob.sense;
    qp.Q.resize(n, n);
    qp.c    = prob.c;
    qp.Aeq  = prob.Aeq;
    qp.beq  = prob.beq;
    qp.vars = prob.vars;
    return solve_qp(qp);
  }

  // Convert A*x <= b to equality form by adding slack variables s >= 0:
  //   [Aeq  0 ] [x]   [beq]
  //   [A    I ] [s] = [b  ]
  //   lb_s = 0,  ub_s = +inf
  const int n_aug = n + m_ineq;

  QPModel qp;
  qp.sense = prob.sense;
  qp.Q.resize(n_aug, n_aug);  // zero (LP → no Q)

  // Augmented cost: [c; 0] (no cost on slacks)
  qp.c.resize(n_aug);
  qp.c << prob.c, Eigen::VectorXd::Zero(m_ineq);

  // Build augmented equality constraint matrix [Aeq 0; A I]
  const int m_eq   = static_cast<int>(prob.Aeq.rows());
  const int m_all  = m_eq + m_ineq;
  qp.Aeq.resize(m_all, n_aug);
  {
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<size_t>(prob.Aeq.nonZeros() + prob.A.nonZeros() + m_ineq));
    // Top block: [Aeq | 0]
    for (int k = 0; k < prob.Aeq.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Aeq, k); it; ++it)
        trips.emplace_back(static_cast<int>(it.row()), static_cast<int>(it.col()), it.value());
    // Bottom block: [A | I]
    for (int k = 0; k < prob.A.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, k); it; ++it)
        trips.emplace_back(m_eq + static_cast<int>(it.row()), static_cast<int>(it.col()), it.value());
    for (int i = 0; i < m_ineq; ++i)
      trips.emplace_back(m_eq + i, n + i, 1.0);   // slack identity block
    qp.Aeq.setFromTriplets(trips.begin(), trips.end());
    qp.Aeq.makeCompressed();
  }

  // Augmented RHS: [beq; b]
  qp.beq.resize(m_all);
  if (m_eq > 0) qp.beq.head(m_eq) = prob.beq;
  qp.beq.tail(m_ineq) = prob.b;

  // Augmented variables: original vars + slack vars (lb=0, ub=+inf)
  qp.vars = prob.vars;
  for (int i = 0; i < m_ineq; ++i)
    qp.vars.push_back({VarType::Continuous, 0.0, std::numeric_limits<double>::infinity(), {}});

  SolveResult out = solve_qp(qp);
  // Strip slack variables from solution
  if (static_cast<int>(out.x.size()) == n_aug) {
    out.x.conservativeResize(n);
  }
  // Re-compute objective using original (un-augmented) cost vector c.
  // solve_qp already handles Maximize by negating c internally, so the
  // returned objective is the minimization value (possibly negated). We
  // recompute from scratch to get the user-facing value (c.dot(x) for min,
  // -c.dot(x) for max, to match the convention used throughout the engine).
  if (out.stats.success) {
    out.stats.objective = prob.c.dot(out.x);
  }
  return out;
}


SolveResult NativeLCQPAdapter::solve_qp(const QPModel& prob) const {
  SolveResult result;
  result.stats.solver_name = name();

  if (!std::isfinite(opt_.centering_exponent)) {
    result.stats.status = "LCQP centering exponent must be finite";
    return result;
  }
  const int n = static_cast<int>(prob.c.size());
  if (n == 0) {
    result.stats.success = true;
    result.stats.objective = 0.0;
    result.stats.status = "Empty";
    return result;
  }

  // Extract bounds
  Eigen::VectorXd lb(n), ub(n);
  for (int i = 0; i < n; ++i) {
    lb(i) = (i < static_cast<int>(prob.vars.size()))
        ? prob.vars[i].lb
        : -std::numeric_limits<double>::infinity();
    ub(i) = (i < static_cast<int>(prob.vars.size()))
        ? prob.vars[i].ub
        : std::numeric_limits<double>::infinity();
  }

  // Handle sense
  Eigen::SparseMatrix<double> Q = prob.Q;
  Eigen::VectorXd c = prob.c;
  if (prob.sense == Sense::Maximize) {
    Q = -Q;
    c = -c;
  }

  return solve_qp_ipm(Q, c, prob.A, prob.b, prob.Aeq, prob.beq, lb, ub);
}

// AUDIT-NAV: 凸 QP 内点主循环；KKT 模式只分析一次，每轮仅更新数值。
// 非凸 Q 不具有全局最优保证，终止必须联合检查可行度、驻点和互补度。
SolveResult NativeLCQPAdapter::solve_qp_ipm(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double>& Aeq,
    const Eigen::VectorXd& beq,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub) const {

  SolveResult result;
  result.stats.solver_name = name();
  LCQPProfile profile;
  const auto setup_started = std::chrono::steady_clock::now();
  const double centering_exponent =
      opt_.centering_exponent > 0.0 ? opt_.centering_exponent : 1.0;

  const int n = static_cast<int>(c.size());
  const int meq = static_cast<int>(Aeq.rows());
  const int mineq = static_cast<int>(A.rows());
  profile.dimension = n + meq;
  profile.inequalities = mineq;
  
  // Find finite bounds
  std::vector<int> idx_lb, idx_ub;
  for (int i = 0; i < n; ++i) {
    if (variable_has_finite_lower_bound(lb(i))) idx_lb.push_back(i);
    if (variable_has_finite_upper_bound(ub(i))) idx_ub.push_back(i);
  }
  const int nlb = static_cast<int>(idx_lb.size());
  const int nub = static_cast<int>(idx_ub.size());
  const int nslack = mineq + nlb + nub;

  if (nslack == 0 && meq == 0) {
    // Unconstrained
    if (Q.nonZeros() == 0) {
      if (c.squaredNorm() > 0.0) {
        result.stats.success = false;
        result.stats.status = "Unbounded";
        return result;
      }
      result.x = Eigen::VectorXd::Zero(n);
      result.stats.success = true;
      result.stats.objective = 0.0;
      result.stats.status = "Optimal";
      return result;
    }
    auto solver = make_default_sparse_solver();
    solver->analyze_pattern(Q);
    if (!solver->factorize(Q)) {
      result.stats.status = "Unconstrained Hessian factorization failed";
      return result;
    }
    Eigen::VectorXd neg_c = -c;
    if (!solver->solve(neg_c, result.x) || !result.x.allFinite()) {
      result.stats.status = "Unconstrained Hessian solve failed";
      return result;
    }
    result.stats.success = true;
    result.stats.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
    result.stats.status = "Optimal";
    return result;
  }

  // A caller may already have a structurally derived feasible descent point.
  // Audit it in the model's original coordinates before equilibration or KKT
  // construction; this status is deliberately not an optimality certificate.
  const bool has_initial_point =
      opt_.initial_point.size() == n && opt_.initial_point.allFinite();
  if (opt_.return_feasible_descent_candidate && has_initial_point) {
    const Eigen::VectorXd& candidate = opt_.initial_point;
    double primal_feasibility = 0.0;
    if (meq > 0) {
      primal_feasibility = std::max(
          primal_feasibility,
          (Aeq * candidate - beq).lpNorm<Eigen::Infinity>());
    }
    if (mineq > 0) {
      primal_feasibility = std::max(
          primal_feasibility,
          std::max(0.0, (A * candidate - b).maxCoeff()));
    }
    for (int variable = 0; variable < n; ++variable) {
      if (variable_has_finite_lower_bound(lb[variable])) {
        primal_feasibility = std::max(
            primal_feasibility, lb[variable] - candidate[variable]);
      }
      if (variable_has_finite_upper_bound(ub[variable])) {
        primal_feasibility = std::max(
            primal_feasibility, candidate[variable] - ub[variable]);
      }
    }
    const double objective =
        0.5 * candidate.dot(Q * candidate) + c.dot(candidate);
    if (std::isfinite(primal_feasibility) && std::isfinite(objective) &&
        primal_feasibility <= opt_.tol_primal &&
        objective <= opt_.candidate_objective_upper_bound) {
      result.x = candidate;
      result.stats.objective = objective;
      result.stats.primal_feas = primal_feasibility;
      result.stats.dual_feas = std::numeric_limits<double>::infinity();
      result.stats.complementarity = std::numeric_limits<double>::infinity();
      result.stats.status =
          "Feasible descent candidate; optimality not certified";
      return result;
    }
  }

  // ===========================================================================
  // Ruiz equilibration: scale the problem for better IPM conditioning.
  // Scaled variables: x_s = D^{-1} x  where D = diag(col_scale).
  // ===========================================================================
  auto scaling = compute_ruiz_scaling(
      Q, Aeq, A, n, meq, mineq, opt_.max_iter);
  const Eigen::VectorXd& d = scaling.col_scale;
  const Eigen::VectorXd& eq_rs = scaling.equality_row_scale;
  const Eigen::VectorXd& ineq_rs = scaling.inequality_row_scale;

  // Q_s = D * Q * D
  Eigen::SparseMatrix<double> Qs = Q;
  for (int col = 0; col < Qs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Qs, col); it; ++it) {
      it.valueRef() *= d[it.row()] * d[col];
    }
  }
  // c_s = D * c
  Eigen::VectorXd cs = c.cwiseProduct(d);
  // Aeq_s = R * Aeq * D  (R = diag(row_scale))
  Eigen::SparseMatrix<double> Aeqs = Aeq;
  for (int col = 0; col < Aeqs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Aeqs, col); it; ++it) {
      it.valueRef() *= eq_rs[it.row()] * d[col];
    }
  }
  // beq_s = R * beq
  Eigen::VectorXd beqs = beq.cwiseProduct(eq_rs);
  Eigen::SparseMatrix<double> As = A;
  for (int col = 0; col < As.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(As, col); it; ++it) {
      it.valueRef() *= ineq_rs[it.row()] * d[col];
    }
  }
  Eigen::VectorXd bs = b.cwiseProduct(ineq_rs);
  // Scaled bounds: lb_s = D^{-1} * lb,  ub_s = D^{-1} * ub
  Eigen::VectorXd lbs(n), ubs(n);
  for (int i = 0; i < n; ++i) {
    double di = d[i];
    lbs[i] = variable_has_finite_lower_bound(lb(i)) ? lb(i) / di :
        -std::numeric_limits<double>::infinity();
    ubs[i] = variable_has_finite_upper_bound(ub(i)) ? ub(i) / di :
        std::numeric_limits<double>::infinity();
  }

  // Re-identify finite bounds on scaled problem (indices unchanged)
  // Use Qs, cs, Aeqs, beqs, lbs, ubs from here on.
  // ===========================================================================
  // Primal-Dual IPM with Mehrotra Predictor-Corrector
  //
  // Variables: x (primal), y (eq dual), z_lb, z_ub (bound duals)
  // Slacks: s_lb = x - lb, s_ub = ub - x (for finite bounds)
  //
  // KKT:
  //   Qx + c + Aeq'y - z_lb + z_ub = 0   (stationarity)
  //   Aeq x = beq                         (primal feasibility)
  //   s_lb * z_lb = 0                     (complementarity)
  //   s_ub * z_ub = 0
  //   s_lb, s_ub, z_lb, z_ub >= 0
  // ===========================================================================

  // Initialize x from a caller-supplied structural estimate when available.
  // Scaling it here preserves the exact original-coordinate point.
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  if (has_initial_point) {
    x = opt_.initial_point.cwiseQuotient(d);
  } else {
    for (int i = 0; i < n; ++i) {
      const bool finite_lower = std::isfinite(lbs(i));
      const bool finite_upper = std::isfinite(ubs(i));
      if (finite_lower && finite_upper) {
        x(i) = std::midpoint(lbs(i), ubs(i));
      } else if (finite_lower) {
        x(i) = lbs(i) + std::sqrt(std::numeric_limits<double>::epsilon()) *
            std::max(1.0, std::abs(lbs(i)));
      } else if (finite_upper) {
        x(i) = ubs(i) - std::sqrt(std::numeric_limits<double>::epsilon()) *
            std::max(1.0, std::abs(ubs(i)));
      }
    }
  }

  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());

  // Resolve strict interiority at the local linearization resolution.
  Eigen::VectorXd s_ineq(mineq), z_ineq(mineq);
  if (mineq > 0) {
    Eigen::VectorXd row_reach = bs.cwiseAbs();
    Eigen::VectorXi row_term_count = Eigen::VectorXi::Ones(mineq);
    for (int col = 0; col < As.outerSize(); ++col) {
      const double variable_reach = std::max(1.0, std::abs(x[col]));
      for (Eigen::SparseMatrix<double>::InnerIterator it(As, col); it; ++it) {
        row_reach[it.row()] += std::abs(it.value()) * variable_reach;
        ++row_term_count[it.row()];
      }
    }
    Eigen::VectorXd row_resolution(mineq);
    for (int row = 0; row < mineq; ++row) {
      row_resolution[row] = std::max(
          minimum_safe_positive(),
          summation_roundoff_bound(row_term_count[row], row_reach[row]));
    }
    s_ineq = (bs - As * x).cwiseMax(row_resolution);
  }

  Eigen::VectorXd s_lb(nlb), z_lb(nlb);
  for (int k = 0; k < nlb; ++k) {
    const int variable = idx_lb[k];
    const double reach =
        std::max({1.0, std::abs(x[variable]), std::abs(lbs[variable])});
    s_lb(k) = std::max(
        x(variable) - lbs(variable), summation_roundoff_bound(2, reach));
  }

  Eigen::VectorXd s_ub(nub), z_ub(nub);
  for (int k = 0; k < nub; ++k) {
    const int variable = idx_ub[k];
    const double reach =
        std::max({1.0, std::abs(x[variable]), std::abs(ubs[variable])});
    s_ub(k) = std::max(
        ubs(variable) - x(variable), summation_roundoff_bound(2, reach));
  }

  // Ruiz scaling makes unit magnitude the local coordinate scale. These are
  // auxiliary QP cone duals, not OPF multipliers or a common OPF s*mu target.
  if (mineq > 0) z_ineq.setOnes();
  if (nlb > 0) z_lb.setOnes();
  if (nub > 0) z_ub.setOnes();

  Eigen::VectorXd y = Eigen::VectorXd::Zero(meq);

  profile.setup_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - setup_started).count();
  const auto pattern_started = std::chrono::steady_clock::now();

  // Build KKT pattern once
  const int kkt_dim = n + meq;
  double kkt_scale = 1.0;
  for (int col = 0; col < Qs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Qs, col); it; ++it) {
      kkt_scale = std::max(kkt_scale, std::abs(it.value()));
    }
  }
  for (int col = 0; col < Aeqs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Aeqs, col); it; ++it) {
      kkt_scale = std::max(kkt_scale, std::abs(it.value()));
    }
  }
  for (int col = 0; col < As.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(As, col); it; ++it) {
      kkt_scale = std::max(kkt_scale, std::abs(it.value()));
    }
  }
  const double reg = root_epsilon * kkt_scale;

  // Eliminating general-inequality slacks and duals contributes A' W A to
  // the primal block. Its graph is fixed because W=diag(z/s) stays positive;
  // build that graph once so symbolic analysis is reusable across iterations.
  Eigen::SparseMatrix<double> primal_pattern = Qs;
  if (mineq > 0) {
    Eigen::SparseMatrix<double> inequality_pattern = As;
    for (int col = 0; col < inequality_pattern.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               inequality_pattern, col);
           it; ++it) {
        it.valueRef() = 1.0;
      }
    }
    primal_pattern += inequality_pattern.transpose() * inequality_pattern;
  }
  primal_pattern.makeCompressed();
  
  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> trips;
  trips.reserve(primal_pattern.nonZeros() + n +
                2 * Aeqs.nonZeros() + meq);

  for (int col = 0; col < primal_pattern.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(primal_pattern, col);
         it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // Initial diagonal regularization. The barrier terms overwrite these
  // entries numerically on every iteration while preserving the KKT pattern.
  for (int i = 0; i < n; ++i) {
    trips.emplace_back(i, i, reg);
  }

  // Scaled Aeq and Aeq' blocks
  for (int col = 0; col < Aeqs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Aeqs, col); it; ++it) {
      trips.emplace_back(it.col(), n + it.row(), it.value());  // Aeq'
      trips.emplace_back(n + it.row(), it.col(), it.value());  // Aeq
    }
  }

  // -δI on (2,2) block
  for (int i = 0; i < meq; ++i) {
    trips.emplace_back(n + i, n + i, -reg);
  }

  Eigen::SparseMatrix<double> KKT(kkt_dim, kkt_dim);
  KKT.setFromTriplets(trips.begin(), trips.end());
  KKT.makeCompressed();

  // The no-equality condensed system is symmetric positive definite. Keep
  // only its lower triangle as the numerical assembly target; CHOLMOD consumes
  // that representation directly. The full matrix remains the symbolic
  // fallback pattern and is populated only if the symmetric factorization
  // rejects a numerical instance.
  Eigen::SparseMatrix<double> kkt_lower;
  if (meq == 0) {
    kkt_lower = KKT.triangularView<Eigen::Lower>();
    kkt_lower.makeCompressed();
  }
  Eigen::SparseMatrix<double>& numeric_kkt =
      meq == 0 ? kkt_lower : KKT;

  struct NormalContribution {
    Eigen::Index value_index;
    int inequality_row;
    double coefficient;
  };
  std::vector<NormalContribution> normal_contributions;
  if (mineq > 0) {
    using RowSparseMatrix =
        Eigen::SparseMatrix<double, Eigen::RowMajor>;
    const RowSparseMatrix as_rows = As;
    Eigen::Index contribution_count = 0;
    for (int row = 0; row < as_rows.rows(); ++row) {
      const Eigen::Index row_nnz =
          as_rows.outerIndexPtr()[row + 1] - as_rows.outerIndexPtr()[row];
      contribution_count += meq == 0
          ? row_nnz * (row_nnz + 1) / 2
          : row_nnz * row_nnz;
    }
    normal_contributions.reserve(
        static_cast<size_t>(contribution_count));
    const auto kkt_value_index = [&](Eigen::Index row, Eigen::Index col) {
      const Eigen::Index begin = numeric_kkt.outerIndexPtr()[col];
      const Eigen::Index end = numeric_kkt.outerIndexPtr()[col + 1];
      const auto* found = std::lower_bound(
          numeric_kkt.innerIndexPtr() + begin,
          numeric_kkt.innerIndexPtr() + end, row);
      eigen_assert(found != numeric_kkt.innerIndexPtr() + end &&
                   *found == row);
      return static_cast<Eigen::Index>(
          found - numeric_kkt.innerIndexPtr());
    };
    for (int row = 0; row < as_rows.rows(); ++row) {
      for (RowSparseMatrix::InnerIterator outer(as_rows, row); outer;
           ++outer) {
        for (RowSparseMatrix::InnerIterator inner(as_rows, row); inner;
             ++inner) {
          if (meq == 0 && inner.col() < outer.col()) continue;
          normal_contributions.push_back({
              kkt_value_index(inner.col(), outer.col()), row,
              inner.value() * outer.value()});
        }
      }
    }
  }

  std::vector<std::pair<Eigen::Index, double>> q_values;
  q_values.reserve(static_cast<size_t>(Qs.nonZeros()));
  for (int col = 0; col < Qs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Qs, col); it; ++it) {
      if (meq == 0 && it.row() < col) continue;
      const Eigen::Index begin = numeric_kkt.outerIndexPtr()[col];
      const Eigen::Index end = numeric_kkt.outerIndexPtr()[col + 1];
      const auto* found = std::lower_bound(
          numeric_kkt.innerIndexPtr() + begin,
          numeric_kkt.innerIndexPtr() + end, it.row());
      eigen_assert(found != numeric_kkt.innerIndexPtr() + end &&
                   *found == it.row());
      q_values.emplace_back(
          static_cast<Eigen::Index>(found - numeric_kkt.innerIndexPtr()),
          it.value());
    }
  }
  
  // With no equality rows the condensed matrix is SPD for a convex QP:
  // Q + A'WA + bound diagonal + regularization. Use symmetric CHOLMOD there;
  // retain the general sparse solver as the numerical fallback and for the
  // equality-constrained saddle-point form.
  CholmodLDLT cholmod_solver;
  bool use_cholmod = false;
  profile.pattern_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pattern_started).count();
  const auto analyze_started = std::chrono::steady_clock::now();
  if (meq == 0) {
    use_cholmod = cholmod_solver.analyze(
        kkt_dim, kkt_lower.outerIndexPtr(), kkt_lower.innerIndexPtr(),
        kkt_lower.valuePtr(), kkt_lower.nonZeros());
  }

  // Do symbolic factorization once (pattern never changes).
  auto kkt_solver = make_default_sparse_solver();
  bool general_solver_analyzed = false;
  if (!use_cholmod) {
    kkt_solver->analyze_pattern(KKT);
    general_solver_analyzed = true;
  }
  profile.analyze_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - analyze_started).count();
  profile.backend = use_cholmod ? "cholmod" : "general";

  // Scaling factor for dual feasibility: normalize by cost vector magnitude.
  // Degenerate variables at their bounds produce large dual multipliers (z ≈ c)
  // that inflate raw dual residuals. Scaling prevents false non-convergence.
  const double c_norm = cs.lpNorm<Eigen::Infinity>();
  const double dual_scale = 1.0 + c_norm;

  const int effective_max_iter = opt_.max_iter;
  const double interior_margin = std::max(
      std::sqrt(std::numeric_limits<double>::epsilon()),
      std::sqrt(std::clamp(opt_.tol_gap, 0.0, 1.0)));
  const double interior_fraction = 1.0 - interior_margin;

  for (int iter = 0; iter < effective_max_iter; ++iter) {
    profile.iterations = iter;
    // Compute residuals (all in scaled space)
    Eigen::VectorXd r_dual = cs;
    double dual_roundoff_scale =
        std::max(1.0, cs.lpNorm<Eigen::Infinity>());
    if (Qs.nonZeros() > 0) {
      const Eigen::VectorXd term = Qs * x;
      r_dual += term;
      dual_roundoff_scale =
          std::max(dual_roundoff_scale, term.lpNorm<Eigen::Infinity>());
    }
    if (meq > 0) {
      const Eigen::VectorXd term = Aeqs.transpose() * y;
      r_dual += term;
      dual_roundoff_scale =
          std::max(dual_roundoff_scale, term.lpNorm<Eigen::Infinity>());
    }
    if (mineq > 0) {
      const Eigen::VectorXd term = As.transpose() * z_ineq;
      r_dual += term;
      dual_roundoff_scale =
          std::max(dual_roundoff_scale, term.lpNorm<Eigen::Infinity>());
    }
    for (int k = 0; k < nlb; ++k) r_dual(idx_lb[k]) -= z_lb(k);
    for (int k = 0; k < nub; ++k) r_dual(idx_ub[k]) += z_ub(k);
    if (nlb > 0) {
      dual_roundoff_scale = std::max(
          dual_roundoff_scale, z_lb.lpNorm<Eigen::Infinity>());
    }
    if (nub > 0) {
      dual_roundoff_scale = std::max(
          dual_roundoff_scale, z_ub.lpNorm<Eigen::Infinity>());
    }

    Eigen::VectorXd r_eq = (meq > 0) ? Aeqs * x - beqs : Eigen::VectorXd();
    Eigen::VectorXd r_ineq =
        (mineq > 0) ? As * x + s_ineq - bs : Eigen::VectorXd();

    Eigen::VectorXd r_slb(nlb), r_sub(nub);
    for (int k = 0; k < nlb; ++k) r_slb(k) = x(idx_lb[k]) - lbs(idx_lb[k]) - s_lb(k);
    for (int k = 0; k < nub; ++k) r_sub(k) = ubs(idx_ub[k]) - x(idx_ub[k]) - s_ub(k);

    // Complementarity
    double mu = mineq > 0 ? s_ineq.dot(z_ineq) : 0.0;
    for (int k = 0; k < nlb; ++k) mu += s_lb(k) * z_lb(k);
    for (int k = 0; k < nub; ++k) mu += s_ub(k) * z_ub(k);
    if (nslack > 0) mu /= nslack;
    double complementarity_inf = 0.0;
    if (mineq > 0) {
      complementarity_inf = std::max(
          complementarity_inf,
          s_ineq.cwiseProduct(z_ineq).lpNorm<Eigen::Infinity>());
    }
    if (nlb > 0) {
      complementarity_inf = std::max(
          complementarity_inf,
          s_lb.cwiseProduct(z_lb).lpNorm<Eigen::Infinity>());
    }
    if (nub > 0) {
      complementarity_inf = std::max(
          complementarity_inf,
          s_ub.cwiseProduct(z_ub).lpNorm<Eigen::Infinity>());
    }

    // Convergence check — use scaled dual feasibility
    double pfeas = std::max(
        std::max(r_eq.lpNorm<Eigen::Infinity>(),
                 r_ineq.lpNorm<Eigen::Infinity>()),
        std::max(r_slb.lpNorm<Eigen::Infinity>(),
                 r_sub.lpNorm<Eigen::Infinity>()));
    double dfeas_raw = r_dual.lpNorm<Eigen::Infinity>();
    double dfeas = dfeas_raw / dual_scale;  // Scaled dual infeasibility
    const double dual_comparison_error =
        root_epsilon * dual_roundoff_scale;
    // Objective in scaled space (unscale for reporting: x_orig = D * x_scaled)
    Eigen::VectorXd x_orig = d.cwiseProduct(x);
    double obj = 0.5 * x_orig.dot(Q * x_orig) + c.dot(x_orig);

    if (opt_.verbose && iter % 5 == 0) {
      printf("LCQP %3d: pf=%.2e df=%.2e mu=%.2e obj=%.6e\n", iter, pfeas, dfeas, mu, obj);
    }

    // Primary convergence: all scaled criteria met
    const bool dual_within_backward_error =
        dfeas_raw <= opt_.tol_dual * dual_scale + dual_comparison_error;
    if (pfeas < opt_.tol_primal && dual_within_backward_error &&
        complementarity_inf >= 0.0 &&
        complementarity_inf < opt_.tol_gap) {
      result.x = x_orig;
      result.stats.success = true;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.complementarity = complementarity_inf;
      result.stats.status = dfeas < opt_.tol_dual
          ? "Optimal"
          : "Optimal (backward-error audited)";
      result.constraint_duals.resize(mineq + meq);
      if (mineq > 0) {
        result.constraint_duals.head(mineq) =
            z_ineq.cwiseProduct(ineq_rs);
      }
      if (meq > 0) {
        result.constraint_duals.tail(meq) = y.cwiseProduct(eq_rs);
      }
      return result;
    }

    const double quadratic_model_term = 0.5 * x_orig.dot(Q * x_orig);
    const double linear_model_term = c.dot(x_orig);
    const double objective_comparison_error = root_epsilon * std::max(
        {1.0, std::abs(quadratic_model_term), std::abs(linear_model_term)});
    if (opt_.return_feasible_descent_candidate &&
        pfeas < opt_.tol_primal &&
        obj + objective_comparison_error <=
            opt_.candidate_objective_upper_bound) {
      result.x = x_orig;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.complementarity = complementarity_inf;
      result.stats.status = "Feasible descent candidate; optimality not certified";
      result.constraint_duals.resize(mineq + meq);
      if (mineq > 0) {
        result.constraint_duals.head(mineq) =
            z_ineq.cwiseProduct(ineq_rs);
      }
      if (meq > 0) {
        result.constraint_duals.tail(meq) = y.cwiseProduct(eq_rs);
      }
      return result;
    }

    if (!std::isfinite(pfeas) || !std::isfinite(dfeas_raw) ||
        !std::isfinite(mu) || !std::isfinite(complementarity_inf) ||
        !std::isfinite(obj)) {
      result.x = x_orig;
      result.stats.success = false;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.status = "Non-finite iterate diagnostics";
      return result;
    }

    const auto assembly_started = std::chrono::steady_clock::now();
    Eigen::VectorXd ineq_ratio(mineq);
    for (int row = 0; row < mineq; ++row) {
      ineq_ratio[row] = z_ineq[row] / s_ineq[row];
    }

    // Build diagonal D = z_lb/s_lb + z_ub/s_ub.
    Eigen::VectorXd D = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < nlb; ++k) {
      D(idx_lb[k]) += z_lb(k) / s_lb(k);
    }
    for (int k = 0; k < nub; ++k) {
      D(idx_ub[k]) += z_ub(k) / s_ub(k);
    }

    // Update only numerical values in the fixed primal-block graph.  Each
    // general-inequality row contributes w_r a_r' a_r.  Its destination slots
    // are invariant, so the symbolic pass above precomputes them once instead
    // of rebuilding a sparse A' W A product on every IPM iteration.
    for (int col = 0; col < n; ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(numeric_kkt, col);
           it; ++it) {
        if (it.row() < n) it.valueRef() = 0.0;
      }
    }
    for (const auto& [value_index, value] : q_values) {
      numeric_kkt.valuePtr()[value_index] += value;
    }
    for (const NormalContribution& contribution : normal_contributions) {
      numeric_kkt.valuePtr()[contribution.value_index] +=
          ineq_ratio[contribution.inequality_row] *
          contribution.coefficient;
    }
    for (int i = 0; i < n; ++i) {
      numeric_kkt.coeffRef(i, i) += D(i) + reg;
    }
    profile.assembly_seconds += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - assembly_started).count();
    
    // Numeric factorization (reuses symbolic analysis).
    const auto factor_started = std::chrono::steady_clock::now();
    bool factorized = false;
    if (use_cholmod) {
      factorized = cholmod_solver.factorize(kkt_lower.valuePtr());
      if (!factorized) use_cholmod = false;
    }
    if (!use_cholmod) {
      if (meq == 0) {
        KKT = kkt_lower.selfadjointView<Eigen::Lower>();
        KKT.makeCompressed();
      }
      if (!general_solver_analyzed) {
        kkt_solver->analyze_pattern(KKT);
        general_solver_analyzed = true;
      }
      factorized = kkt_solver->factorize(KKT);
    }
    profile.factor_seconds += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - factor_started).count();
    ++profile.factorizations;
    profile.backend = use_cholmod ? "cholmod" : "general";
    if (!factorized) {
      result.stats.success = false;
      result.stats.status = "KKT factorization failed";
      result.stats.iterations = iter;
      return result;
    }

    struct Direction {
      Eigen::VectorXd dx, dy;
      Eigen::VectorXd ds_ineq, dz_ineq;
      Eigen::VectorXd ds_lb, dz_lb;
      Eigen::VectorXd ds_ub, dz_ub;
      bool valid{false};
    };
    const auto solve_direction = [&](double sigma,
                                     const Direction* affine) {
      Direction direction;
      Eigen::VectorXd rhs_x = -r_dual;
      Eigen::VectorXd comp_ineq(mineq);
      if (mineq > 0) {
        comp_ineq = Eigen::VectorXd::Constant(mineq, sigma * mu) -
                    s_ineq.cwiseProduct(z_ineq);
        if (affine != nullptr) {
          comp_ineq -= affine->ds_ineq.cwiseProduct(affine->dz_ineq);
        }
        rhs_x -= As.transpose() *
                 (comp_ineq.cwiseQuotient(s_ineq) +
                  ineq_ratio.cwiseProduct(r_ineq));
      }
      Eigen::VectorXd comp_lb(nlb), comp_ub(nub);
      for (int k = 0; k < nlb; ++k) {
        comp_lb[k] = sigma * mu - s_lb[k] * z_lb[k];
        if (affine != nullptr) comp_lb[k] -= affine->ds_lb[k] * affine->dz_lb[k];
        rhs_x(idx_lb[k]) +=
            comp_lb[k] / s_lb[k] - (z_lb[k] / s_lb[k]) * r_slb[k];
      }
      for (int k = 0; k < nub; ++k) {
        comp_ub[k] = sigma * mu - s_ub[k] * z_ub[k];
        if (affine != nullptr) comp_ub[k] -= affine->ds_ub[k] * affine->dz_ub[k];
        rhs_x(idx_ub[k]) -=
            comp_ub[k] / s_ub[k] - (z_ub[k] / s_ub[k]) * r_sub[k];
      }

      Eigen::VectorXd rhs(kkt_dim);
      rhs.head(n) = rhs_x;
      if (meq > 0) rhs.tail(meq) = -r_eq;
      Eigen::VectorXd sol;
      bool solved = false;
      const auto linear_solve_started = std::chrono::steady_clock::now();
      if (use_cholmod) {
        sol.resize(kkt_dim);
        solved = cholmod_solver.solve(rhs.data(), sol.data());
      } else {
        solved = kkt_solver->solve(rhs, sol);
      }
      profile.solve_seconds += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - linear_solve_started).count();
      ++profile.solves;
      if (!solved || !sol.allFinite()) return direction;
      direction.dx = sol.head(n);
      direction.dy = meq > 0 ? sol.tail(meq) : Eigen::VectorXd();
      direction.ds_ineq.resize(mineq);
      direction.dz_ineq.resize(mineq);
      if (mineq > 0) {
        direction.ds_ineq = -r_ineq - As * direction.dx;
        direction.dz_ineq =
            (comp_ineq - z_ineq.cwiseProduct(direction.ds_ineq))
                .cwiseQuotient(s_ineq);
      }
      direction.ds_lb.resize(nlb);
      direction.dz_lb.resize(nlb);
      for (int k = 0; k < nlb; ++k) {
        direction.ds_lb[k] = direction.dx[idx_lb[k]] + r_slb[k];
        direction.dz_lb[k] =
            (comp_lb[k] - z_lb[k] * direction.ds_lb[k]) / s_lb[k];
      }
      direction.ds_ub.resize(nub);
      direction.dz_ub.resize(nub);
      for (int k = 0; k < nub; ++k) {
        direction.ds_ub[k] = -direction.dx[idx_ub[k]] + r_sub[k];
        direction.dz_ub[k] =
            (comp_ub[k] - z_ub[k] * direction.ds_ub[k]) / s_ub[k];
      }
      direction.valid = direction.dx.allFinite() && direction.dy.allFinite() &&
                        direction.ds_ineq.allFinite() && direction.dz_ineq.allFinite() &&
                        direction.ds_lb.allFinite() && direction.dz_lb.allFinite() &&
                        direction.ds_ub.allFinite() && direction.dz_ub.allFinite();
      return direction;
    };

    const Direction affine = solve_direction(0.0, nullptr);
    if (!affine.valid) {
      result.stats.success = false;
      result.stats.status = "KKT affine solve NaN";
      result.stats.iterations = iter;
      return result;
    }
    double affine_primal_step = 1.0;
    double affine_dual_step = 1.0;
    affine_primal_step = std::min(affine_primal_step,
        max_step_to_boundary(s_ineq, affine.ds_ineq));
    affine_dual_step = std::min(affine_dual_step,
        max_step_to_boundary(z_ineq, affine.dz_ineq));
    affine_primal_step = std::min(affine_primal_step,
        max_step_to_boundary(s_lb, affine.ds_lb));
    affine_dual_step = std::min(affine_dual_step,
        max_step_to_boundary(z_lb, affine.dz_lb));
    affine_primal_step = std::min(affine_primal_step,
        max_step_to_boundary(s_ub, affine.ds_ub));
    affine_dual_step = std::min(affine_dual_step,
        max_step_to_boundary(z_ub, affine.dz_ub));
    double mu_affine = 0.0;
    if (mineq > 0) {
      mu_affine += (s_ineq + affine_primal_step * affine.ds_ineq).dot(
          z_ineq + affine_dual_step * affine.dz_ineq);
    }
    if (nlb > 0) {
      mu_affine += (s_lb + affine_primal_step * affine.ds_lb).dot(
          z_lb + affine_dual_step * affine.dz_lb);
    }
    if (nub > 0) {
      mu_affine += (s_ub + affine_primal_step * affine.ds_ub).dot(
          z_ub + affine_dual_step * affine.dz_ub);
    }
    mu_affine /= nslack;
    const double sigma = std::pow(
        std::clamp(mu_affine / std::max(mu, std::numeric_limits<double>::min()),
                   0.0, 1.0),
        centering_exponent);
    const Direction direction = solve_direction(sigma, &affine);
    if (!direction.valid) {
      result.stats.success = false;
      result.stats.status = "KKT corrector solve NaN";
      result.stats.iterations = iter;
      return result;
    }
    const Eigen::VectorXd& dx = direction.dx;
    const Eigen::VectorXd& dy = direction.dy;
    const Eigen::VectorXd& ds_ineq = direction.ds_ineq;
    const Eigen::VectorXd& dz_ineq = direction.dz_ineq;
    const Eigen::VectorXd& ds_lb = direction.ds_lb;
    const Eigen::VectorXd& dz_lb = direction.dz_lb;
    const Eigen::VectorXd& ds_ub = direction.ds_ub;
    const Eigen::VectorXd& dz_ub = direction.dz_ub;

    // Compute step lengths
    double alpha_p = 1.0, alpha_d = 1.0;
    if (mineq > 0) {
      alpha_p = std::min(
          alpha_p, max_step_pos(s_ineq, ds_ineq, interior_fraction));
      alpha_d = std::min(
          alpha_d, max_step_pos(z_ineq, dz_ineq, interior_fraction));
    }
    if (nlb > 0) {
      alpha_p = std::min(
          alpha_p, max_step_pos(s_lb, ds_lb, interior_fraction));
      alpha_d = std::min(
          alpha_d, max_step_pos(z_lb, dz_lb, interior_fraction));
    }
    if (nub > 0) {
      alpha_p = std::min(
          alpha_p, max_step_pos(s_ub, ds_ub, interior_fraction));
      alpha_d = std::min(
          alpha_d, max_step_pos(z_ub, dz_ub, interior_fraction));
    }

    const auto relative_displacement = [](const Eigen::VectorXd& state,
                                          const Eigen::VectorXd& step,
                                          double alpha) {
      if (state.size() == 0) return 0.0;
      return alpha * step.lpNorm<Eigen::Infinity>() /
             (1.0 + state.lpNorm<Eigen::Infinity>());
    };
    const double relative_step = std::max({
        relative_displacement(x, dx, alpha_p),
        relative_displacement(s_ineq, ds_ineq, alpha_p),
        relative_displacement(s_lb, ds_lb, alpha_p),
        relative_displacement(s_ub, ds_ub, alpha_p),
        relative_displacement(y, dy, alpha_d),
        relative_displacement(z_ineq, dz_ineq, alpha_d),
        relative_displacement(z_lb, dz_lb, alpha_d),
        relative_displacement(z_ub, dz_ub, alpha_d)});
    if (!(alpha_p > 0.0) || !(alpha_d > 0.0) ||
        !std::isfinite(alpha_p) || !std::isfinite(alpha_d) ||
        relative_step <= std::numeric_limits<double>::epsilon()) {
      result.x = d.cwiseProduct(x);
      result.stats.success = false;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.complementarity = mu;
      result.stats.status = "Step below machine resolution";
      return result;
    }

    // Update
    x += alpha_p * dx;
    if (meq > 0) y += alpha_d * dy;
    if (mineq > 0) {
      s_ineq += alpha_p * ds_ineq;
      z_ineq += alpha_d * dz_ineq;
    }
    if (nlb > 0) {
      s_lb += alpha_p * ds_lb;
      z_lb += alpha_d * dz_lb;
    }
    if (nub > 0) {
      s_ub += alpha_p * ds_ub;
      z_ub += alpha_d * dz_ub;
    }
    const bool cone_interior =
        (s_ineq.size() == 0 || (s_ineq.array() > 0.0).all()) &&
        (z_ineq.size() == 0 || (z_ineq.array() > 0.0).all()) &&
        (s_lb.size() == 0 || (s_lb.array() > 0.0).all()) &&
        (z_lb.size() == 0 || (z_lb.array() > 0.0).all()) &&
        (s_ub.size() == 0 || (s_ub.array() > 0.0).all()) &&
        (z_ub.size() == 0 || (z_ub.array() > 0.0).all());
    if (!cone_interior) {
      result.x = d.cwiseProduct(x);
      result.stats.success = false;
      result.stats.iterations = iter + 1;
      result.stats.objective =
          0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
      result.stats.status = "Positive-cone boundary lost to roundoff";
      return result;
    }
  }

  // Max iterations — unscale x before returning
  Eigen::VectorXd x_final_orig = d.cwiseProduct(x);
  result.x = x_final_orig;
  result.stats.success = false;
  result.stats.iterations = effective_max_iter;
  result.stats.objective = 0.5 * x_final_orig.dot(Q * x_final_orig) + c.dot(x_final_orig);
  result.stats.status = "MaxIterations";
  return result;
}

}  // namespace mipsolvers::engine

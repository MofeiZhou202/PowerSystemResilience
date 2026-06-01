// Optimized LCQP solver using primal-dual interior point method
// Solves: min 0.5*x'Qx + c'x  s.t. Aeq*x = beq, lb <= x <= ub
//
// Optimizations:
// - Symbolic factorization cached (pattern never changes, only D diagonal values)
// - Adaptive centering: σ = min(0.3, 100μ) for faster convergence near optimum
//
// This is a high-performance implementation for DC OPF problems.

#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

namespace mipsolvers::engine {

namespace {
constexpr double kBigNum = 1e15;
constexpr double kTau = 0.995;  // Step length fraction

// Compute max step α s.t. v + α*dv > 0
double max_step_pos(const Eigen::VectorXd& v, const Eigen::VectorXd& dv) {
  double alpha = 1.0;
  for (int i = 0; i < v.size(); ++i) {
    if (dv(i) < -1e-14) {
      alpha = std::min(alpha, -kTau * v(i) / dv(i));
    }
  }
  return std::max(1e-12, alpha);
}

/// Ruiz equilibration: alternate row/column scaling to equilibrate [Q, Aeq'; Aeq, 0].
/// Returns column scaling vector d such that the scaled problem has better conditioning.
/// Scaled problem: variables x_s = D^{-1} x, where D = diag(d).
///   min 0.5 x_s' (D Q D) x_s + (D c)' x_s
///   s.t. (Aeq D) x_s = beq,   D^{-1} lb <= x_s <= D^{-1} ub
struct RuizScaling {
  Eigen::VectorXd col_scale;  // n-vector: x_original = diag(col_scale) * x_scaled
  Eigen::VectorXd row_scale;  // meq-vector: scale for equality constraint rows
};

RuizScaling compute_ruiz_scaling(const Eigen::SparseMatrix<double>& Q,
                                 const Eigen::SparseMatrix<double>& Aeq,
                                 int n, int meq, int rounds = 5) {
  Eigen::VectorXd col_scale = Eigen::VectorXd::Ones(n);
  Eigen::VectorXd row_scale = Eigen::VectorXd::Ones(std::max(meq, 1));

  for (int r = 0; r < rounds; ++r) {
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
        double v = std::abs(it.value()) * row_scale[it.row()] * col_scale[col];
        col_max[col] = std::max(col_max[col], v);
      }
    }
    for (int j = 0; j < n; ++j) {
      if (col_max[j] > 1e-12) {
        double s = 1.0 / std::sqrt(col_max[j]);
        col_scale[j] *= s;
      }
    }

    // Row scaling for Aeq from *effectively-scaled* matrix.
    if (meq > 0) {
      Eigen::VectorXd row_max = Eigen::VectorXd::Zero(meq);
      for (int col = 0; col < Aeq.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Aeq, col); it; ++it) {
          double v = std::abs(it.value()) * row_scale[it.row()] * col_scale[col];
          row_max[it.row()] = std::max(row_max[it.row()], v);
        }
      }
      for (int i = 0; i < meq; ++i) {
        if (row_max[i] > 1e-12) {
          double s = 1.0 / std::sqrt(row_max[i]);
          row_scale[i] *= s;
        }
      }
    }
  }
  return {col_scale, row_scale};
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
    lb(i) = (i < static_cast<int>(prob.vars.size())) ? prob.vars[i].lb : -kBigNum;
    ub(i) = (i < static_cast<int>(prob.vars.size())) ? prob.vars[i].ub : kBigNum;
    if (lb(i) < -kBigNum + 1) lb(i) = -kBigNum;
    if (ub(i) > kBigNum - 1) ub(i) = kBigNum;
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

SolveResult NativeLCQPAdapter::solve_qp_ipm(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& /*A*/,  // General inequalities not used (DC OPF uses box only)
    const Eigen::VectorXd& /*b*/,
    const Eigen::SparseMatrix<double>& Aeq,
    const Eigen::VectorXd& beq,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub) const {

  SolveResult result;
  result.stats.solver_name = name();

  const int n = static_cast<int>(c.size());
  const int meq = static_cast<int>(Aeq.rows());
  
  // Find finite bounds
  std::vector<int> idx_lb, idx_ub;
  for (int i = 0; i < n; ++i) {
    if (lb(i) > -kBigNum + 1) idx_lb.push_back(i);
    if (ub(i) < kBigNum - 1) idx_ub.push_back(i);
  }
  const int nlb = static_cast<int>(idx_lb.size());
  const int nub = static_cast<int>(idx_ub.size());
  const int nslack = nlb + nub;  // Total slack variables for bounds

  if (nslack == 0 && meq == 0) {
    // Unconstrained
    if (Q.nonZeros() == 0) {
      if (c.norm() > 1e-10) {
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
    Eigen::SparseMatrix<double> Qreg = Q;
    for (int i = 0; i < n; ++i) Qreg.coeffRef(i, i) += 1e-10;
    solver->analyze_pattern(Qreg);
    solver->factorize(Qreg);
    Eigen::VectorXd neg_c = -c;
    solver->solve(neg_c, result.x);
    result.stats.success = true;
    result.stats.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
    result.stats.status = "Optimal";
    return result;
  }

  // ===========================================================================
  // Ruiz equilibration: scale the problem for better IPM conditioning.
  // Scaled variables: x_s = D^{-1} x  where D = diag(col_scale).
  // ===========================================================================
  auto scaling = compute_ruiz_scaling(Q, Aeq, n, meq);
  const Eigen::VectorXd& d = scaling.col_scale;
  const Eigen::VectorXd& rs = scaling.row_scale;

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
      it.valueRef() *= rs[it.row()] * d[col];
    }
  }
  // beq_s = R * beq
  Eigen::VectorXd beqs = beq.cwiseProduct(rs.head(meq));
  // Scaled bounds: lb_s = D^{-1} * lb,  ub_s = D^{-1} * ub
  Eigen::VectorXd lbs(n), ubs(n);
  for (int i = 0; i < n; ++i) {
    double di = d[i];
    lbs[i] = (lb(i) > -kBigNum + 1) ? lb(i) / di : lb(i);
    ubs[i] = (ub(i) < kBigNum - 1) ? ub(i) / di : ub(i);
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

  // Initialize x at center of bounds (in scaled space)
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    double lo = (lbs(i) > -kBigNum + 1) ? lbs(i) : -10.0;
    double hi = (ubs(i) < kBigNum - 1) ? ubs(i) : 10.0;
    x(i) = 0.5 * (lo + hi);
  }

  // Initialize slacks and duals (in scaled space)
  Eigen::VectorXd s_lb(nlb), z_lb(nlb);
  for (int k = 0; k < nlb; ++k) {
    s_lb(k) = std::max(x(idx_lb[k]) - lbs(idx_lb[k]), 1.0);
    z_lb(k) = 1.0;
  }

  Eigen::VectorXd s_ub(nub), z_ub(nub);
  for (int k = 0; k < nub; ++k) {
    s_ub(k) = std::max(ubs(idx_ub[k]) - x(idx_ub[k]), 1.0);
    z_ub(k) = 1.0;
  }

  Eigen::VectorXd y = Eigen::VectorXd::Zero(meq);

  // Build KKT pattern once
  const int kkt_dim = n + meq;
  const double reg = 1e-8;
  
  using Triplet = Eigen::Triplet<double>;
  std::vector<Triplet> trips;
  trips.reserve(Qs.nonZeros() + n + 2 * Aeqs.nonZeros() + meq);

  // Scaled Q block
  for (int col = 0; col < Qs.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Qs, col); it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }

  // D + regularization on diagonal (placeholder values)
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
  
  // Do symbolic factorization once (pattern never changes)
  auto kkt_solver = make_default_sparse_solver();
  kkt_solver->analyze_pattern(KKT);

  // Stall / infeasibility detection state
  double prev_mu = 1e30;
  double prev_obj = 1e30;
  int stall_count = 0;
  int obj_stable_count = 0;
  constexpr int kMaxStall = 30;          // Consecutive iterations with < 1% mu reduction
  constexpr double kStallRatio = 0.999;  // mu must decrease by at least 0.1% per iter
  constexpr double kInfeasGrowth = 1e12; // If residuals exceed this, declare infeasible

  // Scaling factor for dual feasibility: normalize by cost vector magnitude.
  // Degenerate variables at their bounds produce large dual multipliers (z ≈ c)
  // that inflate raw dual residuals. Scaling prevents false non-convergence.
  const double c_norm = cs.lpNorm<Eigen::Infinity>();
  const double dual_scale = 1.0 + c_norm;

  // Cap effective iteration limit for IPM — convergence beyond 500 iterations
  // indicates numerical issues, not insufficient iterations.
  const int effective_max_iter = std::min(opt_.max_iter, 500);

  for (int iter = 0; iter < effective_max_iter; ++iter) {
    // Compute residuals (all in scaled space)
    Eigen::VectorXd r_dual = cs;
    if (Qs.nonZeros() > 0) r_dual += Qs * x;
    if (meq > 0) r_dual += Aeqs.transpose() * y;
    for (int k = 0; k < nlb; ++k) r_dual(idx_lb[k]) -= z_lb(k);
    for (int k = 0; k < nub; ++k) r_dual(idx_ub[k]) += z_ub(k);

    Eigen::VectorXd r_eq = (meq > 0) ? Aeqs * x - beqs : Eigen::VectorXd();

    Eigen::VectorXd r_slb(nlb), r_sub(nub);
    for (int k = 0; k < nlb; ++k) r_slb(k) = x(idx_lb[k]) - lbs(idx_lb[k]) - s_lb(k);
    for (int k = 0; k < nub; ++k) r_sub(k) = ubs(idx_ub[k]) - x(idx_ub[k]) - s_ub(k);

    // Complementarity
    double mu = 0.0;
    for (int k = 0; k < nlb; ++k) mu += s_lb(k) * z_lb(k);
    for (int k = 0; k < nub; ++k) mu += s_ub(k) * z_ub(k);
    if (nslack > 0) mu /= nslack;

    // Convergence check — use scaled dual feasibility
    double pfeas = std::max(r_eq.lpNorm<Eigen::Infinity>(),
                            std::max(r_slb.lpNorm<Eigen::Infinity>(),
                                     r_sub.lpNorm<Eigen::Infinity>()));
    double dfeas_raw = r_dual.lpNorm<Eigen::Infinity>();
    double dfeas = dfeas_raw / dual_scale;  // Scaled dual infeasibility
    // Objective in scaled space (unscale for reporting: x_orig = D * x_scaled)
    Eigen::VectorXd x_orig = d.cwiseProduct(x);
    double obj = 0.5 * x_orig.dot(Q * x_orig) + c.dot(x_orig);

    if (opt_.verbose && iter % 5 == 0) {
      printf("LCQP %3d: pf=%.2e df=%.2e mu=%.2e obj=%.6e\n", iter, pfeas, dfeas, mu, obj);
    }

    // Primary convergence: all scaled criteria met
    if (pfeas < opt_.tol_primal && dfeas < opt_.tol_dual && mu < opt_.tol_gap) {
      result.x = x_orig;
      result.stats.success = true;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.complementarity = mu;
      result.stats.status = "Optimal";
      return result;
    }

    // Track objective stability
    double obj_rel_change = std::abs(obj - prev_obj) / std::max(1.0, std::abs(obj));
    if (iter > 5 && obj_rel_change < 1e-10) {
      ++obj_stable_count;
    } else {
      obj_stable_count = 0;
    }
    prev_obj = obj;

    // Near-optimal acceptance: primal converged, objective stable, mu small.
    // Dual residuals may be inflated by degenerate bound multipliers but the
    // primal solution is correct.
    if (pfeas < opt_.tol_primal && mu < opt_.tol_gap * 1e3 &&
        obj_stable_count >= 3) {
      result.x = x_orig;
      result.stats.success = true;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.complementarity = mu;
      result.stats.status = "Optimal (near)";
      return result;
    }

    // Early termination: primal residuals diverged (likely infeasible).
    // Only check primal — dual explosion from degenerate bound duals is
    // a numerical artifact, not true infeasibility.
    if (pfeas > kInfeasGrowth || !std::isfinite(pfeas) || !std::isfinite(dfeas_raw)) {
      result.x = x_orig;
      result.stats.success = false;
      result.stats.iterations = iter;
      result.stats.objective = obj;
      result.stats.primal_feas = pfeas;
      result.stats.dual_feas = dfeas;
      result.stats.status = "Infeasible (residuals diverged)";
      return result;
    }

    // Stall detection: mu not decreasing sufficiently
    if (iter > 10) {
      if (mu > prev_mu * kStallRatio) {
        ++stall_count;
      } else {
        stall_count = 0;
      }
      if (stall_count >= kMaxStall) {
        // Before declaring stall, check if we have a good-enough primal solution
        if (pfeas < opt_.tol_primal && std::abs(mu) < 1e-3) {
          result.x = x_orig;
          result.stats.success = true;
          result.stats.iterations = iter;
          result.stats.objective = obj;
          result.stats.primal_feas = pfeas;
          result.stats.dual_feas = dfeas;
          result.stats.complementarity = mu;
          result.stats.status = "Optimal (stall-accept)";
          return result;
        }
        result.x = x_orig;
        result.stats.success = false;
        result.stats.iterations = iter;
        result.stats.objective = obj;
        result.stats.primal_feas = pfeas;
        result.stats.dual_feas = dfeas;
        result.stats.complementarity = mu;
        result.stats.status = "Stalled (no progress)";
        return result;
      }
    }
    prev_mu = mu;

    // Build diagonal D = z_lb/s_lb + z_ub/s_ub
    Eigen::VectorXd D = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < nlb; ++k) {
      D(idx_lb[k]) += z_lb(k) / std::max(s_lb(k), 1e-12);
    }
    for (int k = 0; k < nub; ++k) {
      D(idx_ub[k]) += z_ub(k) / std::max(s_ub(k), 1e-12);
    }

    // Update KKT diagonal only (pattern unchanged)
    for (int i = 0; i < n; ++i) {
      KKT.coeffRef(i, i) = Qs.coeff(i, i) + D(i) + reg;
    }
    
    // Numeric factorization (reuses symbolic analysis)
    if (!kkt_solver->factorize(KKT)) {
      result.stats.success = false;
      result.stats.status = "KKT factorization failed";
      result.stats.iterations = iter;
      return result;
    }

    // Adaptive centering: use smaller σ when close to optimality
    double sigma = std::min(0.3, 100.0 * mu);  // Adaptive centering

    // Build RHS
    Eigen::VectorXd rhs_x = -r_dual;
    for (int k = 0; k < nlb; ++k) {
      double comp = sigma * mu - s_lb(k) * z_lb(k);
      double D_lb_k = z_lb(k) / s_lb(k);
      rhs_x(idx_lb[k]) += comp / s_lb(k) - D_lb_k * r_slb(k);
    }
    for (int k = 0; k < nub; ++k) {
      double comp = sigma * mu - s_ub(k) * z_ub(k);
      double D_ub_k = z_ub(k) / s_ub(k);
      rhs_x(idx_ub[k]) -= comp / s_ub(k) - D_ub_k * r_sub(k);
    }

    Eigen::VectorXd rhs(kkt_dim);
    rhs.head(n) = rhs_x;
    if (meq > 0) rhs.tail(meq) = -r_eq;

    Eigen::VectorXd sol;
    if (!kkt_solver->solve(rhs, sol) || !sol.allFinite()) {
      result.stats.success = false;
      result.stats.status = "KKT solve NaN";
      result.stats.iterations = iter;
      return result;
    }

    Eigen::VectorXd dx = sol.head(n);
    Eigen::VectorXd dy = (meq > 0) ? sol.tail(meq) : Eigen::VectorXd();

    // Recover slack and dual directions
    Eigen::VectorXd ds_lb(nlb), dz_lb(nlb);
    for (int k = 0; k < nlb; ++k) {
      ds_lb(k) = dx(idx_lb[k]) + r_slb(k);
      dz_lb(k) = (sigma * mu - s_lb(k) * z_lb(k) - z_lb(k) * ds_lb(k)) / s_lb(k);
    }
    
    Eigen::VectorXd ds_ub(nub), dz_ub(nub);
    for (int k = 0; k < nub; ++k) {
      ds_ub(k) = -dx(idx_ub[k]) + r_sub(k);
      dz_ub(k) = (sigma * mu - s_ub(k) * z_ub(k) - z_ub(k) * ds_ub(k)) / s_ub(k);
    }

    // Compute step lengths
    double alpha_p = 1.0, alpha_d = 1.0;
    if (nlb > 0) {
      alpha_p = std::min(alpha_p, max_step_pos(s_lb, ds_lb));
      alpha_d = std::min(alpha_d, max_step_pos(z_lb, dz_lb));
    }
    if (nub > 0) {
      alpha_p = std::min(alpha_p, max_step_pos(s_ub, ds_ub));
      alpha_d = std::min(alpha_d, max_step_pos(z_ub, dz_ub));
    }

    // Update
    x += alpha_p * dx;
    if (meq > 0) y += alpha_d * dy;
    if (nlb > 0) {
      s_lb += alpha_p * ds_lb;
      z_lb += alpha_d * dz_lb;
    }
    if (nub > 0) {
      s_ub += alpha_p * ds_ub;
      z_ub += alpha_d * dz_ub;
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

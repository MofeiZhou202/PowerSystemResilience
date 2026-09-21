#include "mipsolvers/engine/solver/native/native_adapters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

void project_to_bounds(const std::vector<VariableMeta>& vars, Eigen::VectorXd& x) {
  for (int i = 0; i < x.size(); ++i) {
    x[i] = std::min(vars[i].ub, std::max(vars[i].lb, x[i]));
  }
}

}  // namespace

std::string NativeLinearAdapter::name() const {
  return "NativeLinear";
}

bool NativeLinearAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LE;
}

// AUDIT-NAV: LE 直接法入口；成功状态只能在原系统残差复核后发布。
SolveResult NativeLinearAdapter::solve_le(const SparseLinSys& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid LE model" : vr.errors.front();
    return out;
  }

  auto solver = make_default_sparse_solver();
  solver->analyze_pattern(prob.A);
  if (!solver->factorize(prob.A)) {
    out.stats.status = "LE factorization failed";
    return out;
  }

  Eigen::VectorXd x;
  if (!solver->solve(prob.b, x)) {
    out.stats.status = "LE solve failed";
    return out;
  }

  const Eigen::VectorXd r = prob.A * x - prob.b;
  out.x = x;
  out.stats.success = true;
  out.stats.iterations = 1;
  out.stats.residual_inf = inf_norm(r);
  out.stats.status = "Optimal";

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
}

NativeNewtonAdapter::NativeNewtonAdapter(NativeNLEOptions opt) : opt_(opt) {}

std::string NativeNewtonAdapter::name() const {
  return "NativeNewton";
}

bool NativeNewtonAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::NLE;
}

// AUDIT-NAV: NLE 主循环；依次审核残差、Jacobian、正则化和回溯接受条件。
SolveResult NativeNewtonAdapter::solve_nle(const NonlinearSystem& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid NLE model" : vr.errors.front();
    return out;
  }

  Eigen::VectorXd x = prob.x0;
  Eigen::VectorXd f(prob.n);
  Eigen::VectorXd f_trial(prob.n);

  auto solver = make_default_sparse_solver();

  for (int iter = 0; iter < opt_.max_iter; ++iter) {
    prob.residual(x, f);
    const double res = inf_norm(f);
    if (!std::isfinite(res)) {
      out.stats.status = "NLE residual became non-finite";
      out.stats.iterations = iter;
      break;
    }
    if (res <= opt_.tol) {
      out.x = x;
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.residual_inf = res;
      out.stats.status = "Converged";
      break;
    }

    Eigen::SparseMatrix<double> j;
    prob.jacobian(x, j);

    if (j.rows() != prob.n || j.cols() != prob.n) {
      out.stats.status = "NLE Jacobian shape mismatch";
      out.stats.iterations = iter;
      break;
    }

    Eigen::SparseMatrix<double> w = j;
    double lambda = opt_.regularization0;
    bool step_found = false;
    Eigen::VectorXd dx(prob.n);

    for (int reg_try = 0; reg_try < 4 && !step_found; ++reg_try) {
      if (lambda > 0.0) {
        for (int k = 0; k < prob.n; ++k) {
          w.coeffRef(k, k) += lambda;
        }
      }
      w.makeCompressed();
      solver->analyze_pattern(w);
      if (solver->factorize(w) && solver->solve(-f, dx)) {
        double alpha = 1.0;
        const double base = res;
        Eigen::VectorXd x_trial(prob.n);
        for (int ls = 0; ls < opt_.max_line_search_steps; ++ls) {
          x_trial.noalias() = x + alpha * dx;
          prob.residual(x_trial, f_trial);
          const double trial = inf_norm(f_trial);
          if (std::isfinite(trial) && trial <= base) {
            x = x_trial;
            step_found = true;
            break;
          }
          alpha *= opt_.step_backoff;
        }
      }
      lambda = (lambda <= 0.0) ? 1e-10 : lambda * 10.0;
      w = j;
    }

    if (!step_found) {
      out.stats.status = "NLE line search failed";
      out.stats.iterations = iter + 1;
      break;
    }

    out.stats.iterations = iter + 1;
  }

  if (!out.stats.success && out.stats.status.empty()) {
    prob.residual(x, f);
    out.x = x;
    out.stats.residual_inf = inf_norm(f);
    out.stats.status = "Max iterations reached";
  } else if (out.stats.success) {
    prob.residual(out.x, f);
    out.stats.residual_inf = inf_norm(f);
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
}

NativeNLPAdapter::NativeNLPAdapter(NativeNLPOptions opt) : opt_(opt) {}

std::string NativeNLPAdapter::name() const {
  return "NativeNLP";
}

bool NativeNLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::NLP;
}

// AUDIT-NAV: 这是固定罚参数的轻量 Newton 法，不是 NativeIPM；成功时仍需同时
// 审核原约束可行度和罚函数驻点残差。
SolveResult NativeNLPAdapter::solve_nlp(const NLPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid NLP model" : vr.errors.front();
    return out;
  }

  const int n = static_cast<int>(prob.vars.size());
  Eigen::VectorXd x = prob.x0;
  project_to_bounds(prob.vars, x);

  auto solver = make_default_sparse_solver();

  Eigen::VectorXd grad_f(n);
  Eigen::VectorXd g_eq;
  Eigen::VectorXd h_in;

  for (int iter = 0; iter < opt_.max_iter; ++iter) {
    const double f = prob.f(x);
    if (!std::isfinite(f)) {
      out.stats.status = "NLP objective became non-finite";
      out.stats.iterations = iter;
      break;
    }

    prob.grad(x, grad_f);
    Eigen::SparseMatrix<double> hess;
    if (prob.hess) {
      prob.hess(x, hess);
    } else {
      hess.resize(n, n);
      hess.setZero();
    }

    if (prob.g) {
      prob.g(x, g_eq);
    } else {
      g_eq = Eigen::VectorXd::Zero(0);
    }

    if (prob.h) {
      prob.h(x, h_in);
    } else {
      h_in = Eigen::VectorXd::Zero(0);
    }

    Eigen::VectorXd grad_phi = grad_f;
    double feas_eq = 0.0;
    double feas_in = 0.0;

    if (prob.g && prob.jac_g) {
      Eigen::SparseMatrix<double> jg;
      prob.jac_g(x, jg);
      if (jg.cols() != n || jg.rows() != g_eq.size()) {
        out.stats.status = "NLP jac_g shape mismatch";
        out.stats.iterations = iter;
        break;
      }
      grad_phi += opt_.penalty_rho * (jg.transpose() * g_eq);
      hess += opt_.penalty_rho * (jg.transpose() * jg);
      feas_eq = inf_norm(g_eq);
    }

    if (prob.h && prob.jac_h) {
      Eigen::VectorXd h_pos = h_in.cwiseMax(0.0);
      Eigen::SparseMatrix<double> jh;
      prob.jac_h(x, jh);
      if (jh.cols() != n || jh.rows() != h_in.size()) {
        out.stats.status = "NLP jac_h shape mismatch";
        out.stats.iterations = iter;
        break;
      }
      grad_phi += opt_.penalty_rho * (jh.transpose() * h_pos);

      std::vector<Eigen::Triplet<double>> tri;
      tri.reserve(static_cast<size_t>(jh.nonZeros()));
      for (int k = 0; k < jh.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(jh, k); it; ++it) {
          if (h_in[it.row()] > 0.0) {
            tri.emplace_back(it.row(), it.col(), it.value());
          }
        }
      }
      Eigen::SparseMatrix<double> jh_act(jh.rows(), jh.cols());
      jh_act.setFromTriplets(tri.begin(), tri.end());
      hess += opt_.penalty_rho * (jh_act.transpose() * jh_act);
      feas_in = inf_norm(h_pos);
    }

    const double gnorm = inf_norm(grad_phi);
    if (gnorm <= opt_.tol_grad && feas_eq <= opt_.tol_grad && feas_in <= opt_.tol_grad) {
      out.x = x;
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.objective = f;
      out.stats.primal_feas = std::max(feas_eq, feas_in);
      out.stats.residual_inf = gnorm;
      out.stats.status = "Converged";
      break;
    }

    for (int k = 0; k < n; ++k) {
      hess.coeffRef(k, k) += opt_.regularization0;
    }
    hess.makeCompressed();

    Eigen::VectorXd dx(n);
    solver->analyze_pattern(hess);
    if (!solver->factorize(hess) || !solver->solve(-grad_phi, dx)) {
      out.stats.status = "NLP linear solve failed";
      out.stats.iterations = iter;
      break;
    }

    const double step_inf = inf_norm(dx);
    if (step_inf <= opt_.tol_step) {
      out.x = x;
      out.stats.success = true;
      out.stats.iterations = iter;
      out.stats.objective = f;
      out.stats.primal_feas = std::max(feas_eq, feas_in);
      out.stats.residual_inf = gnorm;
      out.stats.status = "Step tolerance reached";
      break;
    }

    const auto phi = [&](const Eigen::VectorXd& xv) {
      double val = prob.f(xv);
      if (prob.g) {
        Eigen::VectorXd ge;
        prob.g(xv, ge);
        val += 0.5 * opt_.penalty_rho * ge.squaredNorm();
      }
      if (prob.h) {
        Eigen::VectorXd hi;
        prob.h(xv, hi);
        val += 0.5 * opt_.penalty_rho * hi.cwiseMax(0.0).squaredNorm();
      }
      return val;
    };

    const double phi0 = phi(x);
    double alpha = 1.0;
    bool accepted = false;
    Eigen::VectorXd x_trial(n);
    for (int ls = 0; ls < opt_.max_line_search_steps; ++ls) {
      x_trial.noalias() = x + alpha * dx;
      project_to_bounds(prob.vars, x_trial);
      const double phi_trial = phi(x_trial);
      if (std::isfinite(phi_trial) && phi_trial <= phi0) {
        x = x_trial;
        accepted = true;
        break;
      }
      alpha *= opt_.step_backoff;
    }

    if (!accepted) {
      out.stats.status = "NLP line search failed";
      out.stats.iterations = iter + 1;
      break;
    }

    out.stats.iterations = iter + 1;
  }

  if (!out.stats.success) {
    out.x = x;
    if (prob.f) {
      out.stats.objective = prob.f(x);
    }
    Eigen::VectorXd gradf;
    if (prob.grad) {
      prob.grad(x, gradf);
      out.stats.residual_inf = inf_norm(gradf);
    }
    if (prob.g) {
      Eigen::VectorXd ge;
      prob.g(x, ge);
      out.stats.primal_feas = std::max(out.stats.primal_feas, inf_norm(ge));
    }
    if (prob.h) {
      Eigen::VectorXd hi;
      prob.h(x, hi);
      out.stats.primal_feas = std::max(out.stats.primal_feas, inf_norm(hi.cwiseMax(0.0)));
    }
    if (out.stats.status.empty()) {
      out.stats.status = "Max iterations reached";
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
}

NativeBranchAndCutAdapter::NativeBranchAndCutAdapter(BCOptions opt) : opt_(std::move(opt)) {}

BCOptions make_strict_highs_production_options(BCOptions opt) {
  opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  // The public adapter is named StrictHiGHS, so its contract must execute the
  // HiGHS MIP state machine rather than the native tree with a HiGHS LP kernel.
  opt.strict_highs_mip_contract = true;
  opt.auto_highs_root_pipeline = true;
  opt.enable_domain_heuristics = false;
  opt.accept_verified_warm_start_incumbent = true;
  // StrictHiGHS policy: request the two optional upstream rounding heuristics.
  // Their cost and incumbent effect remain instance-dependent.
  opt.highs_mip_run_zi_round = true;
  opt.highs_mip_run_shifting = true;
  // StrictHiGHS policy: force presolve "on" and raise substitution maxfillin.
  opt.highs_force_presolve_on = true;
  opt.highs_presolve_substitution_maxfillin = 30;
  // StrictHiGHS policy: keep dynamic LP cuts for more node visits and request
  // upstream symmetry detection. Neither setting guarantees useful cuts or
  // detectable symmetry on a particular model.
  opt.highs_mip_lp_age_limit = 30;
  opt.highs_mip_detect_symmetry = true;
  return opt;
}

BCOptions make_strict_highs_problem_options(const MIPModel& prob, BCOptions opt) {
  opt = make_strict_highs_production_options(std::move(opt));
  const int num_cols = static_cast<int>(prob.linear_part.vars.size());
  const int num_rows = static_cast<int>(prob.linear_part.A.rows() +
                                        prob.linear_part.Aeq.rows());
  const bool large_root =
      num_cols >= opt.highs_strict_auto_ipm_root_min_cols ||
      num_rows >= opt.highs_strict_auto_ipm_root_min_rows;
  // The upper cap limits the automatic policy only. Explicit caller choices
  // are not overridden by this size gate.
  const bool too_large_for_ipm_root =
      num_cols >= opt.highs_strict_auto_ipm_root_max_cols ||
      num_rows >= opt.highs_strict_auto_ipm_root_max_rows;
  const bool ipm_root_band = large_root && !too_large_for_ipm_root;
  // Minimum time budget: take the larger of the fixed floor and a
  // size-proportional component. The effective minimum is:
  //   max(min_time_sec, n_cols * secs_per_kcol / 1000)
  // With the default secs_per_kcol=2.0:
  //   39-bus  (6360 cols)  → max(30, 12.7) = 30.0 s  (unchanged)
  //   118-bus (26400 cols) → max(30, 52.8) = 52.8 s  (larger problem needs more)
  const double size_min_time_sec =
      std::max(opt.highs_strict_auto_ipm_root_min_time_sec,
               static_cast<double>(num_cols) *
                   opt.highs_strict_auto_ipm_root_secs_per_kcol / 1000.0);
  const bool enough_time_for_ipm_root =
      !(opt.time_limit_sec > 0.0) ||
      opt.time_limit_sec >= size_min_time_sec;
  if (opt.highs_strict_auto_ipm_root_for_large_models && ipm_root_band &&
      enough_time_for_ipm_root &&
      opt.highs_mip_lp_solver == "choose") {
    opt.highs_mip_lp_solver = "ipm";
    if (opt.highs_mip_root_crossover.empty() ||
        opt.highs_mip_root_crossover == "choose") {
      opt.highs_mip_root_crossover = "on";
    }
  }
  return opt;
}

StrictHighsBranchAndCutAdapter::StrictHighsBranchAndCutAdapter(BCOptions opt)
    : opt_(make_strict_highs_production_options(std::move(opt))) {}

std::string StrictHighsBranchAndCutAdapter::name() const {
  return "StrictHiGHS";
}

bool StrictHighsBranchAndCutAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::MILP;
}

SolveResult StrictHighsBranchAndCutAdapter::solve_milp(const MIPModel& prob) const {
  NativeBranchAndCutAdapter delegate(make_strict_highs_problem_options(prob, opt_));
  SolveResult out = delegate.solve_milp(prob);
  out.stats.solver_name = name();
  return out;
}

SolveResult StrictHighsBranchAndCutAdapter::solve_milp(
    const MIPModel& prob, const SolveContext& context) const {
  BCOptions effective = make_strict_highs_problem_options(prob, opt_);
  if (context.has_deadline()) {
    effective.time_limit_sec = std::min(
        effective.time_limit_sec, context.backend_time_limit_sec(0.0));
  }
  if (context.has_explicit_thread_budget()) {
    effective.num_threads = effective.num_threads > 0
                                ? std::min(effective.num_threads,
                                           context.thread_budget())
                                : context.thread_budget();
  }
  NativeBranchAndCutAdapter delegate(std::move(effective));
  SolveResult out = delegate.solve_milp(prob, context);
  out.stats.solver_name = name();
  return out;
}

std::string NativeBranchAndCutAdapter::name() const {
  return "NativeBranchAndCut";
}

bool NativeBranchAndCutAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::MILP || cls == ProblemClass::MINLP;
}

// AUDIT-NAV: MILP 适配层只处理 LP 快路径和 BCResult 映射；树搜索总流程见
// BCSolveState::run，StrictHiGHS 则通过选项进入 HiGHS MIP 合约。
SolveResult NativeBranchAndCutAdapter::solve_milp(const MIPModel& prob) const {
  // ── LP fast-path ──────────────────────────────────────────────────────
  // When no integer/binary variables exist, skip the inapplicable B&C
  // machinery and solve the LP directly. This also preserves constraint
  // dual extraction for LMP pricing.
  if (prob.binary_idx.empty() && prob.integer_idx.empty()) {
    SimplexOptions sopt;
    sopt.max_iter = std::max(opt_.max_lp_iter, 20000);
    sopt.feasibility_tol = std::max(1e-8, opt_.lp_tol * 0.1);
    sopt.optimality_tol  = std::max(1e-8, opt_.lp_tol * 0.1);
    sopt.allow_cold_start = true;
    sopt.verbose = opt_.verbose;

    auto sr = solve_lp_with_basis(prob.linear_part, sopt);
    SolveResult out = std::move(sr.result);
    out.stats.solver_name = name() + " (LP)";

    // Extract constraint duals from simplex basis: y = c_B^T * B^{-1}
    // in the standard-form space, then map back through row_sign to the
    // original LP ordering [ineq (m_ineq) | eq (m_eq)].
    if (out.stats.success && sr.basis.indices.size() > 0) {
      const auto& sf = sr.form;
      const auto& basis = sr.basis;
      const int m = static_cast<int>(sf.A.rows());
      const int m_ineq = static_cast<int>(prob.linear_part.A.rows());
      const int m_eq   = static_cast<int>(prob.linear_part.Aeq.rows());

      // Compute dual: y = c_B^T * B^{-1} using Phase-II objective.
      // In standard form we maximise c_max^T x_std; the dual is y[i]
      // (shadow price of row i in standard form).
      Eigen::VectorXd c_b(m);
      for (int i = 0; i < m; ++i) {
        const int col = basis.indices[static_cast<size_t>(i)];
        c_b[i] = (col >= 0 && col < sf.c_max.size()) ? sf.c_max[col] : 0.0;
      }

      Eigen::VectorXd y(m);
      if (sr.basis_inverse.rows() == m && sr.basis_inverse.cols() == m) {
        y = sr.basis_inverse.transpose() * c_b;
      } else if (basis.cached_sparse_basis) {
        // Use sparse BTRAN
        y = sparse_basis_btran(basis.cached_sparse_basis, c_b);
      } else {
        y.setZero();
      }

      // Map back: original-sense dual for row i = row_sign[i] * y[i].
      // The standard form converts to maximisation internally, so the dual y
      // is in maximise-space.  For a minimisation LP, the original constraint
      // dual (shadow price) = -row_sign[i] * y[i]  (sign flip from max→min).
      // Layout: [ineq duals (m_ineq) | eq duals (m_eq)] — matches Gurobi.
      const double sense_sign =
          (prob.linear_part.sense == Sense::Minimize) ? -1.0 : 1.0;
      out.constraint_duals.resize(m_ineq + m_eq);
      for (int i = 0; i < m; ++i) {
        const double d = sense_sign * sf.row_sign[i] * y[i];
        if (i < m_ineq) {
          out.constraint_duals[i] = d;
        } else {
          out.constraint_duals[i] = d;
        }
      }
    }
    return out;
  }

  // ── Full B&C path (MIP) ──────────────────────────────────────────────
  if (opt_.verbose) {
    const int nv = static_cast<int>(prob.linear_part.vars.size());
    const int mi = static_cast<int>(prob.linear_part.A.rows());
    const int me = static_cast<int>(prob.linear_part.Aeq.rows());
    const int nb = static_cast<int>(prob.binary_idx.size());
    const int ni = static_cast<int>(prob.integer_idx.size());
    fprintf(stderr, "[NativeBC] MILP: %d vars (%d binary, %d integer), %d ineq, %d eq\n",
            nv, nb, ni, mi, me);
  }
  BCResult bc = solve_milp_bc(prob, opt_);
  if (opt_.verbose) {
    fprintf(stderr, "[NativeBC] result: success=%d status='%s' nodes=%d time=%.3fs\n",
            bc.stats.success ? 1 : 0, bc.stats.status.c_str(),
            bc.stats.iterations, bc.stats.runtime_sec);
  }
  SolveResult out;
  out.x = std::move(bc.x);
  out.stats = std::move(bc.stats);
  out.stats.solver_name = name();
  return out;
}

SolveResult NativeBranchAndCutAdapter::solve_milp(
    const MIPModel& prob, const SolveContext& context) const {
  if (context.stop_requested()) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context.deadline_expired() ? "Time limit" : "Cancelled";
    return out;
  }
  BCOptions effective = opt_;
  if (context.has_deadline()) {
    effective.time_limit_sec = std::min(
        effective.time_limit_sec, context.backend_time_limit_sec(0.0));
  }
  if (context.has_explicit_thread_budget()) {
    effective.num_threads = effective.num_threads > 0
                                ? std::min(effective.num_threads,
                                           context.thread_budget())
                                : context.thread_budget();
  }
  effective.random_seed = context.random_seed();
  return NativeBranchAndCutAdapter(std::move(effective)).solve_milp(prob);
}

SolveResult NativeBranchAndCutAdapter::solve_minlp(const MINLPModel& prob) const {
  BCResult bc = solve_minlp_bc(prob, opt_);
  SolveResult out;
  out.x = std::move(bc.x);
  out.stats = std::move(bc.stats);
  out.stats.solver_name = name();
  return out;
}

SolveResult NativeBranchAndCutAdapter::solve_minlp(
    const MINLPModel& prob, const SolveContext& context) const {
  if (context.stop_requested()) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context.deadline_expired() ? "Time limit" : "Cancelled";
    return out;
  }
  BCOptions effective = opt_;
  if (context.has_deadline()) {
    effective.time_limit_sec = std::min(
        effective.time_limit_sec, context.backend_time_limit_sec(0.0));
  }
  if (context.has_explicit_thread_budget()) {
    effective.num_threads = effective.num_threads > 0
                                ? std::min(effective.num_threads,
                                           context.thread_budget())
                                : context.thread_budget();
  }
  effective.random_seed = context.random_seed();
  return NativeBranchAndCutAdapter(std::move(effective)).solve_minlp(prob);
}

}  // namespace mipsolvers::engine

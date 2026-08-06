/// test_ipm_solver.cpp
/// Tests for the interior-point LP/QP solver — exercises every registered LP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

// Return SolveOptions that hard-pins a specific solver.
static SolveOptions solver_opts(const std::string& name) {
  SolveOptions opts;
  opts.preferred_solver = name;
  opts.allow_fallback   = false;
  return opts;
}

// Tests use SolverEngine with preferred_solver="ipm" (or "lcqp" for QP).

// ─── IPM LP test ──────────────────────────────────────────────────────────
// min  -x - y
// s.t. x + y <= 6
//      2x + y <= 8
//      x, y  >= 0
// LP optimal: x=2, y=4, obj=-6
TEST_CASE("IPM: LP solved by barrier method", "[ipm][lp]") {
  SolverEngine eng;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -1.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  A.insert(1, 0) = 2.0; A.insert(1, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 6.0, 8.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  for (const auto& solver_name : eng.list_solvers(ProblemClass::LP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_lp(lp, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(-6.0).margin(1e-4));
      }
    }
  }
}


// ─── QP test via SolverEngine ─────────────────────────────────────────────
// min  x^2 + y^2   s.t. x + y >= 1,  x,y >= 0
// Optimal: x=0.5, y=0.5, obj=0.5
TEST_CASE("IPM/QP: convex QP solved", "[ipm][qp]") {
  SolverEngine eng;

  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c.resize(2);
  qp.c << 0.0, 0.0;  // no linear term

  // Q = 2*I (so 0.5 x'Qx = x'x)
  Eigen::SparseMatrix<double> Q(2, 2);
  Q.insert(0, 0) = 2.0;
  Q.insert(1, 1) = 2.0;
  Q.makeCompressed();
  qp.Q = Q;

  // x + y >= 1  →  -(x+y) <= -1
  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = -1.0; A.insert(0, 1) = -1.0;
  A.makeCompressed();
  qp.A = A;
  qp.b.resize(1);
  qp.b << -1.0;

  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SolveOptions opts;
  opts.preferred_solver = "ipm";
  opts.allow_fallback   = true;

  auto res = eng.solve(qp, opts);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(0.5).margin(1e-4));
  CHECK(res.x[0] == Approx(0.5).margin(1e-4));
  CHECK(res.x[1] == Approx(0.5).margin(1e-4));
}

// ─── IPM warm start / numerical robustness ────────────────────────────────
// Slightly larger LP to exercise IPM iterations.
// min  sum(x)
// s.t. x[i] >= 1 for i=0..4  (5 variables)
// Optimal: all x[i]=1, obj=5
TEST_CASE("IPM: 5-variable lower-bound LP", "[ipm][lp]") {
  SolverEngine eng;

  const int n = 5;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(n);

  // -x[i] <= -1
  Eigen::SparseMatrix<double> A(n, n);
  for (int i = 0; i < n; ++i) {
    A.insert(i, i) = -1.0;
  }
  A.makeCompressed();
  lp.A = A;
  lp.b = -Eigen::VectorXd::Ones(n);

  for (int i = 0; i < n; ++i) {
    lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  }

  SolveOptions opts;
  opts.preferred_solver = "ipm";
  opts.allow_fallback   = true;

  auto res = eng.solve_lp(lp, opts);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(5.0).margin(1e-4));
  for (int i = 0; i < n; ++i) {
    CHECK(res.x[i] == Approx(1.0).margin(1e-4));
  }
}

TEST_CASE("Filter IPM satisfies the full inequality Newton equations",
          "[ipm][nlp][complementarity]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 2.0;
    return 0.5 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] - 2.0;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& inequality) {
    inequality.resize(1);
    inequality[0] = x[0] - 1.0;
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.setIdentity();
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.use_inertia_correction = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);
  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.stats.primal_feas < 1e-8);
  CHECK(result.stats.dual_feas < 1e-8);
  CHECK(detail.complementarity < 1e-8);
}

TEST_CASE("Native IPM acceptable tolerance requires full KKT feasibility",
          "[ipm][nlp][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 10.0;
    return 0.5 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] - 10.0;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };

  IPMOptions options;
  options.globalization = Globalization::Merit;
  options.max_iter = 1;
  options.tol_primal = 1e-12;
  options.tol_dual = 1e-12;
  options.tol_complementarity = 1e-12;
  options.tol_accept = 1e-2;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.stats.primal_feas <= options.tol_accept);
  CHECK(result.stats.dual_feas > options.tol_accept);
  CHECK(result.stats.status.find("Ipopt") == std::string::npos);
}

TEST_CASE("Native IPM acceptable result reports its actual KKT residual",
          "[ipm][nlp][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Constant(1, 5e-3);
  nlp.f = [](const Eigen::VectorXd& x) { return 0.5 * x.squaredNorm(); };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient = x;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };

  IPMOptions options;
  options.globalization = Globalization::Merit;
  options.max_iter = 1;
  options.tol_primal = 1e-12;
  options.tol_dual = 1e-12;
  options.tol_complementarity = 1e-12;
  options.tol_accept = 1e-2;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.stats.status == "Converged (acceptable tolerance)");
  CHECK(result.stats.primal_feas <= options.tol_accept);
  CHECK(result.stats.dual_feas == Approx(5e-3).margin(1e-12));
  CHECK(result.stats.complementarity <= options.tol_accept);
}

TEST_CASE("Filter line search evaluates Jacobians only at accepted iterates",
          "[ipm][nlp][performance]") {
  int equality_calls = 0;
  int jacobian_calls = 0;
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) { return 0.5 * x.squaredNorm(); };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient = x;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };
  nlp.g = [&equality_calls](const Eigen::VectorXd& x,
                            Eigen::VectorXd& equality) {
    ++equality_calls;
    equality.resize(1);
    equality[0] = x[0] - 1.0;
  };
  nlp.jac_g = [&jacobian_calls](const Eigen::VectorXd&,
                                Eigen::SparseMatrix<double>& jacobian) {
    ++jacobian_calls;
    jacobian.resize(1, 1);
    jacobian.setIdentity();
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(equality_calls > jacobian_calls);
  CHECK(jacobian_calls <= result.stats.iterations + 2);
}

TEST_CASE("Filter accepts certified centrality-only multiplier steps",
          "[ipm][nlp][filter][regression]") {
  // Equality fixes x=0 and h(x)=-1 gives an interior inequality with s=1.
  // After stationarity is established, barrier reductions require only
  // lambda/mu updates. A (theta, phi)-only filter cannot see that progress.
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd&) { return 0.0; };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient = Eigen::VectorXd::Zero(1);
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 1.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
    equality.resize(1);
    equality[0] = x[0];
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = 1.0;
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& inequality) {
    inequality.resize(1);
    inequality[0] = x[0] - 1.0;
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = 1.0;
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.stats.primal_feas < options.tol_primal);
  CHECK(result.stats.dual_feas < options.tol_dual);
  CHECK(detail.complementarity < options.tol_complementarity);
}

TEST_CASE("Least-squares dual initialization removes a pure multiplier step",
          "[ipm][nlp][initialization]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 2.0;
    return 0.5 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] - 2.0;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 1.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
    equality.resize(1);
    equality[0] = x[0];
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = 1.0;
  };

  IPMOptions options;
  options.max_iter = 1;
  options.tol_primal = 1e-10;
  options.tol_dual = 1e-10;
  options.tol_complementarity = 1e-10;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.least_square_init_duals = true;
  NativeIPMAdapter initialized_solver(options);
  const auto [initialized_result, initialized_detail] =
      initialized_solver.solve_nlp_detail(nlp);

  REQUIRE(initialized_result.stats.success);
  CHECK(initialized_result.stats.solver_name == "NativeIPM");
  CHECK(initialized_result.stats.iterations == 1);
  REQUIRE(initialized_detail.lambda_eq.size() == 1);
  CHECK(initialized_detail.lambda_eq[0] == Approx(2.0).margin(1e-12));
  CHECK(initialized_result.stats.dual_feas < options.tol_dual);

  options.least_square_init_duals = false;
  NativeIPMAdapter zero_dual_solver(options);
  const SolveResult zero_dual_result = zero_dual_solver.solve_nlp(nlp);
  CHECK_FALSE(zero_dual_result.stats.success);
  CHECK(zero_dual_result.stats.dual_feas > options.tol_dual);
}

TEST_CASE("Filter IPM recovers from an infeasible inequality start",
          "[ipm][nlp][infeasible-start]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Constant(1, -4.0);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 0.5;
    return 0.5 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] - 0.5;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 1.0;
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& inequality) {
    inequality.resize(1);
    inequality[0] = 1.0 - x[0];
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = -1.0;
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);

  REQUIRE(result.stats.success);
  INFO(result.stats.status);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.stats.primal_feas < options.tol_primal);
  CHECK(result.stats.dual_feas < options.tol_dual);
  CHECK(detail.complementarity < options.tol_complementarity);
}

TEST_CASE("Filter IPM uses inertia correction on a nonconvex objective",
          "[ipm][nlp][nonconvex][inertia]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Constant(1, 0.25);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] * x[0] - 1.0;
    return 0.25 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] * (x[0] * x[0] - 1.0);
  };
  nlp.hess = [](const Eigen::VectorXd& x,
                 Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 3.0 * x[0] * x[0] - 1.0;
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-9;
  options.tol_dual = 1e-9;
  options.tol_complementarity = 1e-9;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.use_inertia_correction = true;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.x[0] == Approx(1.0).margin(1e-7));
  CHECK(result.stats.dual_feas < options.tol_dual);
}

TEST_CASE("Filter IPM handles rank-deficient equality multipliers",
          "[ipm][nlp][degenerate]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 2.0;
    return 0.5 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = x[0] - 2.0;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 1.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
    equality.resize(2);
    equality << x[0], 2.0 * x[0];
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(2, 1);
    jacobian.insert(0, 0) = 1.0;
    jacobian.insert(1, 0) = 2.0;
  };

  IPMOptions options;
  options.max_iter = 30;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);

  REQUIRE(result.stats.success);
  REQUIRE(detail.lambda_eq.size() == 2);
  CHECK(result.stats.primal_feas < options.tol_primal);
  CHECK(result.stats.dual_feas < options.tol_dual);
  CHECK((-2.0 + detail.lambda_eq[0] + 2.0 * detail.lambda_eq[1]) ==
        Approx(0.0).margin(options.tol_dual));
}

TEST_CASE("Filter IPM does not report inconsistent equalities as solved",
          "[ipm][nlp][infeasible][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) { return 0.5 * x.squaredNorm(); };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient = x;
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 1.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
    equality.resize(2);
    equality << x[0], x[0] - 1.0;
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(2, 1);
    jacobian.insert(0, 0) = 1.0;
    jacobian.insert(1, 0) = 1.0;
  };

  IPMOptions options;
  options.max_iter = 30;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.stats.primal_feas >= 0.5 - 1e-8);
  CHECK(result.stats.status.find("Ipopt") == std::string::npos);
}

TEST_CASE("Quasi-Newton IPM avoids startup finite-difference Hessian sweeps",
          "[ipm][nlp][performance]") {
  constexpr int n = 4;
  int gradient_calls = 0;
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.assign(n, {VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(n);
  const Eigen::VectorXd target =
      (Eigen::VectorXd(n) << 1.0, -2.0, 0.5, 3.0).finished();
  nlp.f = [target](const Eigen::VectorXd& x) {
    return 0.5 * (x - target).squaredNorm();
  };
  nlp.grad = [target, &gradient_calls](const Eigen::VectorXd& x,
                                      Eigen::VectorXd& gradient) {
    ++gradient_calls;
    gradient = x - target;
  };

  IPMOptions options;
  options.max_iter = 20;
  options.tol_primal = 1e-10;
  options.tol_dual = 1e-10;
  options.tol_complementarity = 1e-10;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const SolveResult result = solver.solve_nlp(nlp);

  REQUIRE(result.stats.success);
  CHECK((result.x - target).lpNorm<Eigen::Infinity>() < 1e-9);
  CHECK(gradient_calls <= result.stats.iterations + 2);
}

TEST_CASE("Quasi-Newton IPM handles nonlinear Lagrangian constraint curvature",
          "[ipm][nlp][quasi-newton]") {
  // min x0 on 100*x0^2 + x1^2 = 1. The objective gradient is constant, so
  // every nonzero secant comes from the nonlinear equality Jacobian. The
  // minimizer is (-0.1, 0), with a nonzero equality multiplier.
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.assign(2, {VarType::Continuous, -1e20, 1e20});
  nlp.x0 = (Eigen::VectorXd(2) << -0.05, 0.8).finished();
  nlp.f = [](const Eigen::VectorXd& x) { return x[0]; };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient.resize(2);
    gradient << 1.0, 0.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
    equality.resize(1);
    equality[0] = 100.0 * x[0] * x[0] + x[1] * x[1] - 1.0;
  };
  nlp.jac_g = [](const Eigen::VectorXd& x,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 2);
    jacobian.insert(0, 0) = 200.0 * x[0];
    jacobian.insert(0, 1) = 2.0 * x[1];
  };

  IPMOptions options;
  options.max_iter = 100;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(-0.1).margin(1e-7));
  CHECK(result.x[1] == Approx(0.0).margin(1e-6));
  CHECK(result.stats.primal_feas < options.tol_primal);
  CHECK(result.stats.dual_feas < options.tol_dual);
  CHECK(detail.lambda_eq.size() == 1);
  CHECK(detail.lambda_eq[0] == Approx(0.05).margin(1e-6));
}

TEST_CASE("Scaled filter IPM reports an unscaled self-consistent KKT result",
          "[ipm][nlp][scaling][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double residual = x[0] - 2.0;
    return 5000.0 * residual * residual;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.resize(1);
    gradient[0] = 10000.0 * (x[0] - 2.0);
  };
  nlp.hess = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.insert(0, 0) = 10000.0;
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& inequality) {
    inequality.resize(1);
    inequality[0] = 1000.0 * (x[0] - 1.0);
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = 1000.0;
  };

  IPMOptions options;
  options.max_iter = 150;
  options.tol_primal = 1e-6;
  options.tol_dual = 1e-6;
  options.tol_complementarity = 1e-6;
  options.tol_accept = 0.0;
  options.scale_problem = true;
  options.use_restoration_phase = false;
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(1.0).margin(1e-8));
  REQUIRE(detail.z_slack.size() == detail.mu_ineq.size());
  const double returned_complementarity =
      detail.z_slack.cwiseProduct(detail.mu_ineq)
          .lpNorm<Eigen::Infinity>();
  CHECK(result.stats.primal_feas <= options.tol_primal);
  CHECK(result.stats.dual_feas <= options.tol_dual);
  CHECK(result.stats.complementarity ==
        Approx(returned_complementarity).margin(1e-12));
}

TEST_CASE("Reduced sparse KKT block solve matches the regularized system",
          "[ipm][kkt][multi-rhs]") {
  Eigen::SparseMatrix<double> w(4, 4);
  w.insert(0, 0) = 5.0; w.insert(0, 1) = -1.0;
  w.insert(1, 0) = -1.0; w.insert(1, 1) = 4.0;
  w.insert(1, 2) = 0.5; w.insert(2, 1) = 0.5;
  w.insert(2, 2) = 3.0; w.insert(2, 3) = -0.25;
  w.insert(3, 2) = -0.25; w.insert(3, 3) = 2.0;
  w.makeCompressed();

  Eigen::SparseMatrix<double> jg(2, 4);
  jg.insert(0, 0) = 1.0; jg.insert(0, 2) = -0.5;
  jg.insert(1, 1) = 1.0; jg.insert(1, 3) = 0.75;
  jg.makeCompressed();

  Eigen::VectorXd rhs(6);
  rhs << 1.0, -2.0, 0.5, 3.0, -0.25, 1.5;
  Eigen::VectorXd dx, dlambda;
  constexpr double reg = 1e-8;
  REQUIRE(solve_kkt_reduced_sparse(w, jg, rhs, dx, dlambda,
                                   2, reg, reg));

  Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(6, 6);
  kkt.topLeftCorner(4, 4) = Eigen::MatrixXd(w);
  kkt.topLeftCorner(4, 4).diagonal().array() += reg;
  kkt.topRightCorner(4, 2) = Eigen::MatrixXd(jg.transpose());
  kkt.bottomLeftCorner(2, 4) = Eigen::MatrixXd(jg);
  kkt.bottomRightCorner(2, 2).diagonal().array() -= reg;
  Eigen::VectorXd solution(6);
  solution << dx, dlambda;
  CHECK((kkt * solution - rhs).lpNorm<Eigen::Infinity>() < 1e-10);

  CHECK_FALSE(solve_kkt_reduced_sparse(w, jg, rhs, dx, dlambda,
                                       1, reg, reg));
}

TEST_CASE("Sparse augmented KKT reuses one factor for multiple right-hand sides",
          "[ipm][kkt]") {
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = 4.0;
  w.insert(1, 1) = 3.0;
  w.insert(2, 2) = 2.0;
  w.insert(0, 1) = 0.25;
  w.insert(1, 0) = 0.25;
  w.makeCompressed();

  Eigen::SparseMatrix<double> jg(2, 3);
  jg.insert(0, 0) = 1.0;
  jg.insert(0, 1) = 1.0;
  jg.insert(1, 1) = 1.0;
  jg.insert(1, 2) = -1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  settings.mu = 1e-2;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  REQUIRE(status.correct);
  CHECK(status.reduced_space_certificate);
  CHECK(status.tangent_dimension == 1);
  CHECK(status.delta_w_used == Approx(0.0));
  CHECK(status.delta_c_used == Approx(0.0));
  CHECK(cache.augmented.numeric_factorizations == 1);

  const auto check_rhs = [&](const Eigen::VectorXd& rhs) {
    Eigen::VectorXd dx;
    Eigen::VectorXd dlambda;
    REQUIRE(solve_kkt_inertia_corrected_sparse(
        cache, rhs, dx, dlambda));
    Eigen::VectorXd residual(5);
    residual.head(3) =
        w * dx + status.delta_w_used * dx + jg.transpose() * dlambda;
    residual.tail(2) =
        jg * dx - status.delta_c_used * dlambda;
    CHECK((residual - rhs).lpNorm<Eigen::Infinity>() < 1e-11);
  };

  Eigen::VectorXd rhs1(5);
  rhs1 << 1.0, -2.0, 0.5, 0.25, -0.75;
  check_rhs(rhs1);
  const double* const solution_workspace =
      cache.augmented.solve_solution.data();
  const double* const residual_workspace =
      cache.augmented.solve_residual.data();
  Eigen::VectorXd rhs2(5);
  rhs2 << -0.5, 1.5, 2.0, -1.0, 0.5;
  check_rhs(rhs2);

  CHECK(cache.augmented.solve_solution.data() == solution_workspace);
  CHECK(cache.augmented.solve_residual.data() == residual_workspace);
  CHECK(cache.augmented.numeric_factorizations == 1);
  CHECK(cache.augmented.symbolic_analyses == 1);
  CHECK(cache.augmented.linear_solves >= 2);
}

TEST_CASE("Sparse augmented KKT reuses symbolic analysis by exact pattern",
          "[ipm][kkt]") {
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = 2.0;
  w.insert(1, 1) = 3.0;
  w.insert(2, 2) = 4.0;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 3);
  jg.insert(0, 0) = 1.0;
  jg.insert(0, 2) = -1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.reduced_space_certificate);
  CHECK(cache.tangent_partition_analyses == 1);
  CHECK(cache.tangent_basis.symbolic_analyses == 1);
  CHECK(cache.augmented.symbolic_analyses == 1);

  w.coeffRef(0, 0) = 5.0;
  w.coeffRef(1, 1) = 6.0;
  w.coeffRef(2, 2) = 7.0;
  w.makeCompressed();
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(cache.tangent_partition_analyses == 1);
  CHECK(cache.tangent_basis.symbolic_analyses == 1);
  CHECK(cache.augmented.symbolic_analyses == 1);
  CHECK(cache.tangent_basis.numeric_factorizations == 2);
  CHECK(cache.augmented.numeric_factorizations == 2);

  w.insert(0, 1) = 0.1;
  w.insert(1, 0) = 0.1;
  w.makeCompressed();
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(cache.tangent_partition_analyses == 1);
  CHECK(cache.tangent_basis.symbolic_analyses == 1);
  CHECK(cache.augmented.symbolic_analyses == 2);
}

TEST_CASE("KKT inertia uses reduced rather than full-space curvature",
          "[ipm][kkt][inertia]") {
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = 2.0;
  w.insert(1, 1) = 3.0;
  w.insert(2, 2) = -1e8;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 3);
  jg.insert(0, 2) = 1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  cache.preferred_free_columns = {0, 1};
  InertiaSettings settings;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.correct);
  CHECK(status.reduced_space_certificate);
  CHECK(status.tangent_dimension == 2);
  CHECK(status.min_reduced_curvature == Approx(2.0));
  CHECK(status.delta_w_used == Approx(0.0));
  CHECK(status.nullspace_residual < 1e-12);
  CHECK(cache.tangent_partition_strategy == 1);
  CHECK(cache.primal_numeric_factorizations == 0);

  Eigen::VectorXd rhs(4);
  rhs << 1.0, -2.0, 0.5, 0.25;
  Eigen::VectorXd dx;
  Eigen::VectorXd dlambda;
  REQUIRE(solve_kkt_inertia_corrected_sparse(
      cache, rhs, dx, dlambda));
  Eigen::VectorXd residual(4);
  residual.head(3) = w * dx + jg.transpose() * dlambda;
  residual.tail(1) = jg * dx;
  CHECK((residual - rhs).lpNorm<Eigen::Infinity>() < 1e-10);
}

TEST_CASE("KKT inertia shifts only genuine tangent negative curvature",
          "[ipm][kkt][inertia]") {
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = -1e8;
  w.insert(1, 1) = -4.0;
  w.insert(2, 2) = 3.0;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 3);
  jg.insert(0, 0) = 1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.reduced_space_certificate);
  CHECK(status.min_reduced_curvature == Approx(-4.0));
  CHECK(status.delta_w_used == Approx(4.0).margin(1e-5));
  CHECK(status.delta_w_used < 5.0);
}

TEST_CASE("Large-nullity KKT reads inertia from the symmetric factor",
          "[ipm][kkt][inertia][mumps]") {
#if MIPSOLVERS_HAVE_MUMPS
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = 2.0;
  w.insert(1, 1) = 3.0;
  w.insert(2, 2) = -1e8;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 3);
  jg.insert(0, 2) = 1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  settings.max_tangent_dimension = 0;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.correct);
  CHECK(status.direct_factor_inertia);
  CHECK_FALSE(status.reduced_space_certificate);
  CHECK(status.n_pos == 3);
  CHECK(status.n_neg == 1);
  CHECK(status.n_zero == 0);
  CHECK(status.delta_w_used == Approx(0.0));
  CHECK(cache.primal_numeric_factorizations == 0);

  Eigen::VectorXd rhs(4);
  rhs << 1.0, -2.0, 0.5, 0.25;
  Eigen::VectorXd dx;
  Eigen::VectorXd dlambda;
  REQUIRE(solve_kkt_inertia_corrected_sparse(
      cache, rhs, dx, dlambda));
  Eigen::VectorXd residual(4);
  residual.head(3) = w * dx + jg.transpose() * dlambda;
  residual.tail(1) = jg * dx;
  CHECK((residual - rhs).lpNorm<Eigen::Infinity>() < 1e-9);
#else
  SUCCEED("MUMPS not enabled in this build");
#endif
}

TEST_CASE("Direct factor inertia shifts genuine tangent negative curvature",
          "[ipm][kkt][inertia][mumps]") {
#if MIPSOLVERS_HAVE_MUMPS
  Eigen::SparseMatrix<double> w(3, 3);
  w.insert(0, 0) = -1e8;
  w.insert(1, 1) = -4.0;
  w.insert(2, 2) = 3.0;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 3);
  jg.insert(0, 0) = 1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  settings.max_tangent_dimension = 0;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.direct_factor_inertia);
  CHECK(status.n_neg == 1);
  CHECK(status.delta_w_used > 4.0);
  CHECK(status.delta_w_used < 6.0);
  CHECK(status.factorization_attempts > 1);
#else
  SUCCEED("MUMPS not enabled in this build");
#endif
}

TEST_CASE("Sparse augmented KKT regularizes rank-deficient equalities",
          "[ipm][kkt]") {
  Eigen::SparseMatrix<double> w(2, 2);
  w.setIdentity();
  Eigen::SparseMatrix<double> jg(2, 2);
  jg.insert(0, 0) = 1.0;
  jg.insert(1, 0) = 1.0;
  jg.makeCompressed();

  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  settings.mu = 1e-4;
  InertiaStatus status;
  double delta_w_last = 0.0;
  REQUIRE(factor_kkt_inertia_corrected_sparse(
      w, jg, settings, delta_w_last, cache, status));
  CHECK(status.correct);
  CHECK(status.delta_c_used > 0.0);
  CHECK(status.n_zero == 0);

  Eigen::VectorXd rhs(4);
  rhs << 1.0, -1.0, 0.25, 0.25;
  Eigen::VectorXd dx;
  Eigen::VectorXd dlambda;
  REQUIRE(solve_kkt_inertia_corrected_sparse(
      cache, rhs, dx, dlambda));
  Eigen::VectorXd residual(4);
  residual.head(2) =
      w * dx + status.delta_w_used * dx + jg.transpose() * dlambda;
  residual.tail(2) = jg * dx - status.delta_c_used * dlambda;
  CHECK((residual - rhs).lpNorm<Eigen::Infinity>() < 1e-9);
}

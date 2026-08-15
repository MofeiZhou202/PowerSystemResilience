/// test_ipm_solver.cpp
/// Tests for the interior-point LP/QP solver — exercises every registered LP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_filter.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_restoration.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_scaling.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

TEST_CASE("NLP scaling preserves diagnostic names",
          "[ipm][scaling][diagnostics]") {
  NLPModel original;
  original.vars = {
      {VarType::Continuous, 0.0, 2.0, "dispatch"},
      {VarType::Continuous, -1.0, 1.0, "voltage"}};
  original.nonlinear_inequality_names = {"thermal_limit"};

  ScalingFactors factors;
  factors.s_f = 0.5;
  factors.s_g.resize(0);
  factors.s_h = Eigen::VectorXd::Constant(1, 0.25);
  const NLPModel scaled = build_scaled_nlp_model(original, factors);

  REQUIRE(scaled.vars.size() == 2);
  CHECK(scaled.vars[0].name == "dispatch");
  CHECK(scaled.vars[1].name == "voltage");
  REQUIRE(scaled.nonlinear_inequality_names.size() == 1);
  CHECK(scaled.nonlinear_inequality_names[0] == "thermal_limit");
}

TEST_CASE("Filter feasibility wall is independent of dominance margins",
          "[ipm][filter][contract]") {
  Filter filter;
  const double contract_bound = std::nextafter(
      1.0, std::numeric_limits<double>::infinity());
  filter.reset_with_theta_upper_bound(contract_bound);

  CHECK(filter.is_acceptable(1.0, 0.0, 0.25, 0.25));
  CHECK_FALSE(filter.is_acceptable(contract_bound, 0.0, 0.25, 0.25));

  filter.add_entry(0.5, 1.0, 0.25, 0.25);
  filter.clear();
  CHECK(filter.size() == 0);
  CHECK(filter.is_acceptable(1.0, 0.0, 0.25, 0.25));
  CHECK_FALSE(filter.satisfies_theta_upper_bound(contract_bound));
}

TEST_CASE("Restoration warm start maps only original bound rows",
          "[ipm][restoration][structure]") {
  NLPModel original;
  original.vars = {
      {VarType::Continuous, 0.0, 2.0},
      {VarType::Continuous, -1.0, 1e20}};
  original.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x[0] + x[1] - 1.0;
  };
  original.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    h.resize(1);
    h[0] = x[0] * x[0] - 4.0;
  };

  Eigen::VectorXd x_reference(2);
  x_reference << 0.5, 0.5;
  const RestorationBuild build =
      build_restoration_nlp(original, x_reference, 0.0);
  REQUIRE(build.model.x0.size() == 4);
  CHECK(build.model.x0[2] > 0.0);
  CHECK(build.model.x0[3] > 0.0);
  Eigen::VectorXd restoration_initial_residual;
  build.model.g(build.model.x0, restoration_initial_residual);
  REQUIRE(restoration_initial_residual.size() == 1);
  CHECK(restoration_initial_residual[0] == 0.0);

  Eigen::VectorXd restoration_x(4);
  restoration_x << 0.75, 0.25, 0.1, 0.1;
  Eigen::VectorXd equality_dual(1);
  equality_dual << -0.5;
  Eigen::VectorXd inequality_dual(6);
  inequality_dual << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
  Eigen::VectorXd slack(6);
  slack << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  const RestorationWarmStart warm = recover_restoration_warm_start(
      original, build, restoration_x, equality_dual, inequality_dual, slack,
      1, {0, 1, 2, 3}, {0});

  REQUIRE(warm.valid);
  REQUIRE(warm.inequality_dual.size() == 4);
  CHECK(warm.x[0] == Approx(0.75));
  CHECK(warm.x[1] == Approx(0.25));
  CHECK(warm.equality_dual[0] == Approx(-0.5));
  CHECK(warm.inequality_dual[0] == Approx(10.0));
  CHECK(warm.inequality_dual[1] == Approx(20.0));
  CHECK(warm.inequality_dual[2] == Approx(30.0));
  CHECK(warm.inequality_dual[3] == Approx(60.0));
  CHECK(warm.slack[0] == Approx(1.0));
  CHECK(warm.slack[1] == Approx(2.0));
  CHECK(warm.slack[2] == Approx(3.0));
  CHECK(warm.slack[3] == Approx(6.0));
}

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
      INFO(solver_name << ": " << res.stats.status);
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

TEST_CASE("Native LCQP condenses general inequality slacks",
          "[ipm][qp][inequality][structure]") {
  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c = Eigen::VectorXd::Zero(2);
  qp.Q.resize(2, 2);
  qp.Q.insert(0, 0) = 2.0;
  qp.Q.insert(1, 1) = 2.0;
  qp.Q.makeCompressed();
  qp.A.resize(1, 2);
  qp.A.insert(0, 0) = -1.0;
  qp.A.insert(0, 1) = -1.0;
  qp.A.makeCompressed();
  qp.b = Eigen::VectorXd::Constant(1, -1.0);
  qp.Aeq.resize(0, 2);
  qp.beq.resize(0);
  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  const SolveResult result = NativeLCQPAdapter().solve_qp(qp);

  INFO("status=" << result.stats.status
       << " primal=" << result.stats.primal_feas
       << " dual=" << result.stats.dual_feas
       << " complementarity=" << result.stats.complementarity);
  REQUIRE(result.stats.success);
  REQUIRE(result.x.size() == 2);
  CHECK(result.x[0] == Approx(0.5).margin(1e-5));
  CHECK(result.x[1] == Approx(0.5).margin(1e-5));
  CHECK(result.stats.objective == Approx(0.5).margin(1e-5));
  CHECK((qp.A * result.x - qp.b).maxCoeff() <= 1e-7);
}

TEST_CASE("NativeLCQP consumes a complete model warm start",
          "[ipm][qp][warm-start]") {
  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c = Eigen::VectorXd::Zero(2);
  qp.Q.resize(2, 2);
  qp.Q.insert(0, 0) = 2.0;
  qp.Q.insert(1, 1) = 2.0;
  qp.Q.makeCompressed();
  qp.A.resize(0, 2);
  qp.b.resize(0);
  qp.Aeq.resize(1, 2);
  qp.Aeq.insert(0, 0) = 1.0;
  qp.Aeq.insert(0, 1) = 1.0;
  qp.Aeq.makeCompressed();
  qp.beq = Eigen::VectorXd::Constant(1, 1.4);
  qp.vars = {{VarType::Continuous, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0}};

  NativeLCQPAdapter solver;
  const SolveResult cold = solver.solve_qp(qp);
  REQUIRE(cold.stats.success);
  REQUIRE_FALSE(cold.stats.warm_start_used);
  REQUIRE(std::isfinite(cold.stats.initial_primal_feas));

  qp.x0 = Eigen::VectorXd::Constant(2, 0.7);
  const SolveResult warm = solver.solve_qp(qp);
  REQUIRE(warm.stats.success);
  CHECK(warm.stats.warm_start_used);
  CHECK(warm.stats.initial_primal_feas <= 1e-12);
  // The cold path may now reach the same equality-feasible point through its
  // minimum-norm projection; consuming x0 must remain no worse.
  CHECK(warm.stats.initial_primal_feas <= cold.stats.initial_primal_feas);
  CHECK(warm.x[0] == Approx(0.7).margin(1e-7));
  CHECK(warm.x[1] == Approx(0.7).margin(1e-7));

  qp.x0 = Eigen::VectorXd::Constant(1, 0.7);
  const SolveResult rejected = solver.solve_qp(qp);
  REQUIRE(rejected.stats.success);
  CHECK_FALSE(rejected.stats.warm_start_used);
  CHECK((rejected.x - cold.x).lpNorm<Eigen::Infinity>() <= 1e-12);
}

TEST_CASE("NativeLCQP time limit returns an uncertified finite iterate",
          "[ipm][qp][time-limit]") {
  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c = Eigen::VectorXd::Zero(2);
  qp.Q.resize(2, 2);
  qp.Q.insert(0, 0) = 2.0;
  qp.Q.insert(1, 1) = 2.0;
  qp.Q.makeCompressed();
  qp.A.resize(0, 2);
  qp.b.resize(0);
  qp.Aeq.resize(1, 2);
  qp.Aeq.insert(0, 0) = 1.0;
  qp.Aeq.insert(0, 1) = 1.0;
  qp.Aeq.makeCompressed();
  qp.beq = Eigen::VectorXd::Constant(1, 1.4);
  qp.vars = {{VarType::Continuous, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0}};

  LCQPOptions options;
  options.time_limit_sec = std::numeric_limits<double>::min();
  const SolveResult result = NativeLCQPAdapter(options).solve_qp(qp);

  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.status == "TimeLimit");
  REQUIRE(result.x.size() == 2);
  CHECK(result.x.allFinite());
  CHECK(result.stats.symbolic_analyze_calls == 1);
  CHECK(result.stats.factorization_calls == 0);
}

TEST_CASE("Native LCQP accepts a structural primal initial point",
          "[ipm][qp][initial-point]") {
  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c = Eigen::VectorXd::Zero(2);
  qp.Q.resize(2, 2);
  qp.Q.insert(0, 0) = 2.0;
  qp.Q.insert(1, 1) = 2.0;
  qp.Q.makeCompressed();
  qp.A.resize(1, 2);
  qp.A.insert(0, 0) = -1.0;
  qp.A.insert(0, 1) = -1.0;
  qp.A.makeCompressed();
  qp.b = Eigen::VectorXd::Constant(1, -1.0);
  qp.Aeq.resize(0, 2);
  qp.beq.resize(0);
  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  qp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  LCQPOptions options;
  options.initial_point.resize(2);
  options.initial_point << 0.75, 0.25;
  const SolveResult result = NativeLCQPAdapter(options).solve_qp(qp);

  INFO("status=" << result.stats.status
       << " primal=" << result.stats.primal_feas
       << " dual=" << result.stats.dual_feas
       << " complementarity=" << result.stats.complementarity);
  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(0.5).margin(1e-5));
  CHECK(result.x[1] == Approx(0.5).margin(1e-5));
  CHECK((qp.A * result.x - qp.b).maxCoeff() <= options.tol_primal);
}

TEST_CASE("Native LCQP returns an audited initial descent candidate before KKT",
          "[ipm][qp][initial-point][candidate]") {
  QPModel qp;
  qp.sense = Sense::Minimize;
  qp.c = Eigen::VectorXd::Zero(2);
  qp.Q.resize(2, 2);
  qp.Q.insert(0, 0) = 2.0;
  qp.Q.insert(1, 1) = 2.0;
  qp.Q.makeCompressed();
  qp.A.resize(1, 2);
  qp.A.insert(0, 0) = -1.0;
  qp.A.insert(0, 1) = -1.0;
  qp.A.makeCompressed();
  qp.b = Eigen::VectorXd::Constant(1, -1.0);
  qp.Aeq.resize(0, 2);
  qp.beq.resize(0);
  qp.vars.push_back({VarType::Continuous, 0.0, kVariableNoBound});
  qp.vars.push_back({VarType::Continuous, 0.0, kVariableNoBound});

  LCQPOptions options;
  options.max_iter = 0;
  options.initial_point.resize(2);
  options.initial_point << 0.75, 0.25;
  options.return_feasible_descent_candidate = true;
  options.candidate_objective_upper_bound =
      0.5 * options.initial_point.dot(qp.Q * options.initial_point) +
      qp.c.dot(options.initial_point);

  const SolveResult result = NativeLCQPAdapter(options).solve_qp(qp);

  CHECK_FALSE(result.stats.success);
  REQUIRE(result.x.size() == options.initial_point.size());
  CHECK(result.x == options.initial_point);
  CHECK(result.stats.iterations == 0);
  CHECK(result.stats.primal_feas <= options.tol_primal);
  CHECK(result.stats.status ==
        "Feasible descent candidate; optimality not certified");
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
  options.acceptable_iter = 2;
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
  options.acceptable_iter = 2;
  const SolveResult transient = NativeIPMAdapter(options).solve_nlp(nlp);
  CHECK_FALSE(transient.stats.success);
  CHECK_FALSE(transient.stats.acceptable_convergence);

  options.acceptable_iter = 1;
  const SolveResult result = NativeIPMAdapter(options).solve_nlp(nlp);

  REQUIRE(result.stats.success);
  CHECK(result.stats.status == "Converged (acceptable level)");
  CHECK(result.stats.acceptable_convergence);
  CHECK_FALSE(result.stats.strict_convergence);
  CHECK(result.stats.primal_feas <= options.tol_accept);
  CHECK(result.stats.dual_feas == Approx(5e-3).margin(1e-12));
  CHECK(result.stats.complementarity <= options.tol_accept);
}

TEST_CASE("IPM raw dual guard is explicit and complementarity remains raw",
          "[ipm][termination][regression]") {
  IPMOptions options;
  options.tol_primal = 1e-4;
  options.tol_dual = 1e-4;
  options.tol_complementarity = 1e-4;
  options.tol_overall = 1e-4;
  options.multiplier_scale_threshold = 1.0;
  options.multiplier_relative_stationarity = true;

  Eigen::VectorXd equality = Eigen::VectorXd::Constant(100, 1e6);
  Eigen::VectorXd inequality = Eigen::VectorXd::Constant(100, 1e6);
  const IPMTerminationMetrics scaled_dual = evaluate_ipm_termination(
      0.0, 1.0, 0.0, equality, inequality, 1.0, options);
  CHECK(scaled_dual.overall_error <= options.tol_overall);
  CHECK(scaled_dual.strict);

  options.raw_dual_guard = options.tol_dual;
  const IPMTerminationMetrics guarded_dual = evaluate_ipm_termination(
      0.0, 1.0, 0.0, equality, inequality, 1.0, options);
  CHECK(guarded_dual.overall_error <= options.tol_overall);
  CHECK_FALSE(guarded_dual.strict);

  const IPMTerminationMetrics raw_complementarity_guard =
      evaluate_ipm_termination(0.0, 0.0, 1.0, equality, inequality, 1.0,
                               options);
  CHECK(raw_complementarity_guard.overall_error <= options.tol_overall);
  CHECK_FALSE(raw_complementarity_guard.strict);
}

TEST_CASE("A single huge multiplier does not define the whole dual scale",
          "[ipm][termination][regression]") {
  IPMOptions options;
  options.multiplier_scale_threshold = 1.0;
  Eigen::VectorXd equality = Eigen::VectorXd::Zero(1000);
  equality[0] = 1e6;
  const IPMTerminationMetrics metrics = evaluate_ipm_termination(
      0.0, 1.0, 0.0, equality, Eigen::VectorXd(), 1.0, options);
  CHECK(metrics.dual_scale == Approx(1e3));
  CHECK(metrics.dual_scale < 1.0 + equality.lpNorm<Eigen::Infinity>());
}

TEST_CASE("Filter certifies a solution reached on the last allowed step",
          "[ipm][nlp][filter][iteration-budget][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Ones(1);
  nlp.f = [](const Eigen::VectorXd& x) { return 0.5 * x.squaredNorm(); };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient = x;
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };

  IPMOptions options;
  options.max_iter = 1;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.verbose = std::getenv("HACDCPF_OPF_TRACE") != nullptr;

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);

  INFO(result.stats.status);
  REQUIRE(result.stats.success);
  CHECK(result.stats.iterations == 1);
  CHECK(result.x[0] == Approx(0.0).margin(options.tol_dual));
  CHECK(result.stats.primal_feas <= options.tol_primal);
  CHECK(result.stats.dual_feas <= options.tol_dual);
  CHECK(result.stats.complementarity <= options.tol_complementarity);
  CHECK(detail.trial_value_evaluations ==
        detail.trial_full_derivative_evaluations +
            detail.trial_rejections_before_derivatives);
}

TEST_CASE("Failed last-step derivative audit preserves trial counters",
          "[ipm][nlp][filter][iteration-budget][regression]") {
  int gradient_calls = 0;
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
  nlp.x0 = Eigen::VectorXd::Ones(1);
  nlp.f = [](const Eigen::VectorXd& x) { return 0.5 * x.squaredNorm(); };
  nlp.grad = [&gradient_calls](const Eigen::VectorXd& x,
                               Eigen::VectorXd& gradient) {
    gradient = x;
    if (++gradient_calls > 1) {
      gradient[0] = std::numeric_limits<double>::quiet_NaN();
    }
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };

  IPMOptions options;
  options.max_iter = 1;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);

  CHECK_FALSE(result.stats.success);
  CHECK(gradient_calls >= 2);
  CHECK(detail.trial_value_evaluations ==
        detail.trial_full_derivative_evaluations +
            detail.trial_rejections_before_derivatives);
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
  CHECK(detail.complementarity <= options.tol_complementarity);
}

TEST_CASE("Auto Newton retries the unique equivalent formulation",
          "[ipm][nlp][newton][fallback][regression]") {
  // Jh is finite, while Jh^T(M/S)Jh is not representable for M/S=1.
  // The smaller condensed graph is therefore selected first but cannot earn
  // a numeric certificate; the uncondensed augmented entries remain finite.
  const double jacobian_scale =
      2.0 * std::sqrt(std::numeric_limits<double>::max());

  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back(
      {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd&) { return 0.0; };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient = Eigen::VectorXd::Zero(1);
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };
  nlp.h = [jacobian_scale](const Eigen::VectorXd& x,
                           Eigen::VectorXd& inequality) {
    inequality.resize(1);
    inequality[0] = jacobian_scale * x[0] - 1.0;
  };
  nlp.jac_h = [jacobian_scale](const Eigen::VectorXd&,
                                Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(1, 1);
    jacobian.insert(0, 0) = jacobian_scale;
  };

  IPMOptions options;
  options.max_iter = 1;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.newton_formulation = NewtonFormulation::Auto;
  options.slack_start = Eigen::VectorXd::Ones(1);
  options.inequality_dual_start = Eigen::VectorXd::Ones(1);

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);
  INFO(result.stats.status);
  CHECK(detail.condensed_dimension < detail.augmented_dimension);
  CHECK(detail.condensed_symbolic_flops < detail.augmented_symbolic_flops);
  CHECK(detail.condensed_symbolic_nonzeros <
        detail.augmented_symbolic_nonzeros);
  CHECK(detail.newton_formulation == "augmented");
  CHECK(detail.numeric_factorizations >= 2);
  CHECK(detail.numeric_factorizations ==
        detail.primary_factorizations +
            detail.inertia_retry_factorizations +
            detail.active_set_polish_factorizations);
}

TEST_CASE("Filter distinguishes a machine-resolution step from convergence",
          "[ipm][nlp][termination][stagnation][regression]") {
  const auto make_model = [](bool exact_hessian) {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back(
        {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
    nlp.x0 = Eigen::VectorXd::Zero(1);
    nlp.f = [exact_hessian](const Eigen::VectorXd& x) {
      return exact_hessian
          ? 0.5e20 * x[0] * x[0] + 1.0e10 * x[0]
          : 1.0e10 * x[0];
    };
    nlp.grad = [exact_hessian](const Eigen::VectorXd& x,
                               Eigen::VectorXd& gradient) {
      gradient.resize(1);
      gradient[0] = exact_hessian ? 1.0e20 * x[0] + 1.0e10 : 1.0e10;
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
      hessian.insert(0, 0) = 1.0e20;
    };
    return nlp;
  };

  IPMOptions options;
  options.max_iter = 10;
  options.tol_primal = 1.0e-12;
  options.tol_dual = 1.0e-12;
  options.tol_complementarity = 1.0e-12;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;

  const auto [stagnated, stagnated_detail] =
      NativeIPMAdapter(options).solve_nlp_detail(make_model(false));
  (void)stagnated_detail;
  CHECK_FALSE(stagnated.stats.success);
  CHECK(stagnated.stats.status.find("iterate stagnation") !=
        std::string::npos);
  CHECK(stagnated.stats.dual_feas > options.tol_dual);

  const auto [converged, converged_detail] =
      NativeIPMAdapter(options).solve_nlp_detail(make_model(true));
  (void)converged_detail;
  REQUIRE(converged.stats.success);
  CHECK(converged.stats.dual_feas <= options.tol_dual);
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
  const auto [zero_dual_result, zero_dual_detail] =
      zero_dual_solver.solve_nlp_detail(nlp);
  REQUIRE(zero_dual_result.stats.success);
  REQUIRE(zero_dual_detail.lambda_eq.size() == 1);
  CHECK(zero_dual_detail.lambda_eq[0] == Approx(2.0).margin(1e-12));
  CHECK(zero_dual_result.stats.dual_feas <= options.tol_dual);
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

  INFO(result.stats.status);
  REQUIRE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPM");
  CHECK(result.x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.stats.primal_feas < options.tol_primal);
  CHECK(result.stats.dual_feas < options.tol_dual);
  CHECK(detail.complementarity < options.tol_complementarity);
}

TEST_CASE("Native IPM preserves only an audited Phase-I primal start",
          "[ipm][nlp][phase1][initialization][regression]") {
  const auto make_model = [](double& first_objective_x) {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    nlp.x0 = Eigen::VectorXd::Constant(1, -5e-5);
    nlp.f = [&first_objective_x](const Eigen::VectorXd& x) {
      if (!std::isfinite(first_objective_x)) first_objective_x = x[0];
      return 0.5 * x.squaredNorm();
    };
    nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
      gradient = x;
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
      hessian.insert(0, 0) = 1.0;
    };
    // The callback equality is deliberately expressed in an internal row
    // coordinate. The independent original-coordinate audit must govern the
    // preserve decision, while Native continues to audit the box itself.
    nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
      equality = 100.0 * x;
    };
    nlp.jac_g = [](const Eigen::VectorXd&,
                   Eigen::SparseMatrix<double>& jacobian) {
      jacobian.resize(1, 1);
      jacobian.insert(0, 0) = 100.0;
    };
    nlp.original_constraint_violation = [](const Eigen::VectorXd& x) {
      return std::abs(x[0]);
    };
    return nlp;
  };

  IPMOptions options;
  options.primal_feasible_start = true;
  options.max_iter = 1;
  options.tol_primal = 1e-4;
  options.tol_dual = 1e-4;
  options.tol_complementarity = 1e-4;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;

  double accepted_first_x = std::numeric_limits<double>::quiet_NaN();
  const auto [accepted_result, accepted_detail] =
      NativeIPMAdapter(options).solve_nlp_detail(
          make_model(accepted_first_x));
  (void)accepted_result;
  CHECK(accepted_detail.primal_feasible_start_requested);
  CHECK(accepted_detail.primal_feasible_start_accepted);
  REQUIRE(std::isfinite(accepted_first_x));
  CHECK(accepted_first_x == Approx(-5e-5).margin(1e-15));

  options.tol_primal = 1e-6;
  double rejected_first_x = std::numeric_limits<double>::quiet_NaN();
  const auto [rejected_result, rejected_detail] =
      NativeIPMAdapter(options).solve_nlp_detail(
          make_model(rejected_first_x));
  (void)rejected_result;
  CHECK(rejected_detail.primal_feasible_start_requested);
  CHECK_FALSE(rejected_detail.primal_feasible_start_accepted);
  REQUIRE(std::isfinite(rejected_first_x));
  CHECK(rejected_first_x >= 0.0);
}

TEST_CASE("Native IPM can preserve an audited primal without feasible-start policy",
          "[ipm][nlp][phase1][initialization][regression]") {
  const auto first_evaluation = [](double tolerance) {
    double first_x = std::numeric_limits<double>::quiet_NaN();
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back({VarType::Continuous, 0.0, 1.0});
    nlp.x0 = Eigen::VectorXd::Constant(1, -5e-5);
    nlp.f = [&first_x](const Eigen::VectorXd& x) {
      if (!std::isfinite(first_x)) first_x = x[0];
      return 0.5 * x.squaredNorm();
    };
    nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
      gradient = x;
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
      hessian.insert(0, 0) = 1.0;
    };
    IPMOptions options;
    options.preserve_initial_point = true;
    options.max_iter = 1;
    options.tol_primal = tolerance;
    options.tol_dual = tolerance;
    options.tol_complementarity = tolerance;
    options.tol_accept = 0.0;
    options.scale_problem = false;
    options.use_restoration_phase = false;
    NativeIPMAdapter(options).solve_nlp_detail(nlp);
    return first_x;
  };

  CHECK(first_evaluation(1e-4) == Approx(-5e-5).margin(1e-15));
  CHECK(first_evaluation(1e-6) >= 0.0);
}

TEST_CASE("Automatic Phase-II initialization is row-scale covariant",
          "[ipm][nlp][phase2][initialization][scaling]") {
  const auto solve_scaled_row = [](double row_scale) {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back({VarType::Continuous, -1e20, 1e20});
    nlp.x0 = Eigen::VectorXd::Zero(1);
    nlp.f = [](const Eigen::VectorXd&) { return 0.0; };
    nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
      gradient = Eigen::VectorXd::Zero(1);
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
      hessian.setIdentity();
    };
    nlp.h = [row_scale](const Eigen::VectorXd& x,
                        Eigen::VectorXd& inequality) {
      inequality = Eigen::VectorXd::Constant(1, row_scale * x[0]);
    };
    nlp.jac_h = [row_scale](const Eigen::VectorXd&,
                            Eigen::SparseMatrix<double>& jacobian) {
      jacobian.resize(1, 1);
      jacobian.insert(0, 0) = row_scale;
    };

    IPMOptions options;
    options.primal_feasible_start = true;
    options.max_iter = 1;
    options.tol_primal = row_scale * 1e-6;
    options.tol_dual = 1e-6;
    options.tol_complementarity = 1e-6;
    options.tol_accept = 0.0;
    options.scale_problem = false;
    options.use_restoration_phase = false;
    options.mu_init = 0.0;
    options.mu_min = 0.0;
    return NativeIPMAdapter(options).solve_nlp_detail(nlp);
  };

  const auto [unit_result, unit_detail] = solve_scaled_row(1.0);
  const auto [scaled_result, scaled_detail] = solve_scaled_row(1e6);
  REQUIRE(unit_result.stats.success);
  REQUIRE(scaled_result.stats.success);
  REQUIRE(unit_detail.z_slack.size() == 1);
  REQUIRE(unit_detail.mu_ineq.size() == 1);
  REQUIRE(scaled_detail.z_slack.size() == 1);
  REQUIRE(scaled_detail.mu_ineq.size() == 1);
  CHECK(unit_detail.z_slack[0] > 0.0);
  CHECK(unit_detail.z_slack[0] < 1e-6);
  CHECK(scaled_detail.z_slack[0] / 1e6 ==
        Approx(unit_detail.z_slack[0]).epsilon(1e-12));
  CHECK(scaled_detail.mu_ineq[0] * 1e6 ==
        Approx(unit_detail.mu_ineq[0]).epsilon(1e-12));
}

TEST_CASE("Phase-I handoff does not impose a shared complementarity product",
          "[ipm][nlp][phase1][phase2][initialization][regression]") {
  constexpr int inequality_count = 10000;
  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());

  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back(
      {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd&) { return 0.0; };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient = Eigen::VectorXd::Zero(1);
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };
  nlp.h = [root_epsilon](const Eigen::VectorXd&,
                         Eigen::VectorXd& inequality) {
    inequality.resize(inequality_count);
    for (int row = 0; row < inequality_count; ++row) {
      const double relative_row = static_cast<double>(row) /
          static_cast<double>(inequality_count - 1);
      inequality[row] = -root_epsilon * (1.0 + relative_row);
    }
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(inequality_count, 1);
  };

  IPMOptions options;
  options.primal_feasible_start = true;
  options.max_iter = 1;
  options.tol_primal = 1e-4;
  options.tol_dual = 1e-4;
  options.tol_complementarity = 1e-4;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.mu_init = 0.0;
  options.mu_min = 0.0;

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);
  INFO(result.stats.status);
  REQUIRE(result.stats.success);
  REQUIRE(detail.z_slack.size() == inequality_count);
  REQUIRE(detail.mu_ineq.size() == inequality_count);
  const Eigen::VectorXd products =
      detail.z_slack.cwiseProduct(detail.mu_ineq);
  CHECK(products.maxCoeff() > products.minCoeff());
  CHECK(detail.mu_ineq.minCoeff() >=
        std::sqrt(std::numeric_limits<double>::min()));
}

TEST_CASE("Complete warm start recovers its current mean barrier",
          "[ipm][nlp][phase2][initialization][warm-start][regression]") {
  const auto solve = [](double initial_barrier) {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back({VarType::Continuous, 0.0, 2.0});
    nlp.x0 = Eigen::VectorXd::Constant(1, 0.5);
    nlp.f = [](const Eigen::VectorXd& x) { return x[0]; };
    nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
      gradient = Eigen::VectorXd::Ones(1);
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
    };

    IPMOptions options;
    options.primal_feasible_start = true;
    options.max_iter = 1;
    options.tol_primal = 1e-8;
    options.tol_dual = 1e-2;
    options.tol_complementarity = 1e-1;
    options.tol_accept = 0.0;
    options.scale_problem = false;
    options.use_restoration_phase = false;
    options.use_second_order_correction = false;
    options.mu_init = initial_barrier;
    options.mu_min = 0.0;
    options.inequality_dual_start.resize(2);
    options.inequality_dual_start << 2.0, 1.0;
    options.slack_start.resize(2);
    options.slack_start << 0.5, 1.5;
    return NativeIPMAdapter(options).solve_nlp_detail(nlp);
  };

  const double recovered_barrier = (0.5 * 2.0 + 1.5 * 1.0) / 2.0;
  const auto [automatic_result, automatic_detail] = solve(0.0);
  const auto [explicit_result, explicit_detail] = solve(recovered_barrier);

  INFO(automatic_result.stats.status);
  INFO(explicit_result.stats.status);
  REQUIRE(automatic_result.x.size() == explicit_result.x.size());
  REQUIRE(automatic_detail.mu_ineq.size() == explicit_detail.mu_ineq.size());
  REQUIRE(automatic_detail.z_slack.size() == explicit_detail.z_slack.size());
  CHECK(automatic_result.x.isApprox(explicit_result.x, 1e-12));
  CHECK(automatic_detail.mu_ineq.isApprox(explicit_detail.mu_ineq, 1e-12));
  CHECK(automatic_detail.z_slack.isApprox(explicit_detail.z_slack, 1e-12));
  CHECK(automatic_result.stats.dual_feas ==
        Approx(explicit_result.stats.dual_feas).epsilon(1e-12));
  CHECK(automatic_detail.complementarity ==
        Approx(explicit_detail.complementarity).epsilon(1e-12));
}

TEST_CASE("Scaled complementarity guard preserves a positive strict budget",
          "[ipm][nlp][scaling][complementarity][regression]") {
  const auto solve_at_boundary = [](double tolerance) {
    NLPModel nlp;
    nlp.sense = Sense::Minimize;
    nlp.vars.push_back(
        {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
    nlp.x0 = Eigen::VectorXd::Zero(1);
    nlp.f = [](const Eigen::VectorXd& x) { return 16.0 * x[0]; };
    nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
      gradient = Eigen::VectorXd::Constant(1, 16.0);
    };
    nlp.hess = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& hessian) {
      hessian.resize(1, 1);
    };
    nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& equality) {
      equality = x;
    };
    nlp.jac_g = [](const Eigen::VectorXd&,
                   Eigen::SparseMatrix<double>& jacobian) {
      jacobian.resize(1, 1);
      jacobian.setIdentity();
    };
    nlp.h = [](const Eigen::VectorXd&, Eigen::VectorXd& inequality) {
      inequality = Eigen::VectorXd::Constant(1, -1.0);
    };
    nlp.jac_h = [](const Eigen::VectorXd&,
                   Eigen::SparseMatrix<double>& jacobian) {
      jacobian.resize(1, 1);
    };

    IPMOptions options;
    options.max_iter = 1;
    options.tol_primal = tolerance;
    options.tol_dual = tolerance;
    options.tol_complementarity = tolerance;
    options.tol_accept = 0.0;
    options.scale_problem = true;
    options.use_restoration_phase = false;
    options.equality_dual_start = Eigen::VectorXd::Constant(1, -16.0);
    const double forward_resolution =
        std::sqrt(std::numeric_limits<double>::epsilon()) *
        std::max(1.0, tolerance);
    const double guard = std::min(forward_resolution, tolerance / 2.0);
    const double guarded_boundary = tolerance - guard;
    options.inequality_dual_start = Eigen::VectorXd::Constant(
        1, std::nextafter(guarded_boundary, 0.0));
    options.slack_start = Eigen::VectorXd::Ones(1);

    return NativeIPMAdapter(options).solve_nlp_detail(nlp);
  };

  for (const double tolerance : {1.0e-4, 1.0e-12}) {
    DYNAMIC_SECTION("tolerance=" << tolerance) {
      const auto [result, detail] = solve_at_boundary(tolerance);
      INFO(result.stats.status);
      REQUIRE(result.stats.success);
      CHECK(detail.numeric_factorizations == 0);
      CHECK(result.stats.complementarity < tolerance);
      CHECK(detail.complementarity <= tolerance);
    }
  }
}

TEST_CASE("Complete warm start is not replaced by the Phase-I dual selector",
          "[ipm][nlp][phase2][initialization][warm-start][regression]") {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, 0.0, 2.0});
  nlp.x0 = Eigen::VectorXd::Constant(1, 0.5);
  nlp.f = [](const Eigen::VectorXd&) { return 0.0; };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient = Eigen::VectorXd::Zero(1);
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
  };

  IPMOptions options;
  options.primal_feasible_start = true;
  options.max_iter = 1;
  options.tol_primal = 1e-8;
  options.tol_dual = 10.0;
  options.tol_complementarity = 10.0;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;
  options.mu_init = 0.0;
  options.mu_min = 0.0;
  options.inequality_dual_start.resize(2);
  options.inequality_dual_start << 10.0, 1.0;
  options.slack_start.resize(2);
  options.slack_start << 0.5, 1.5;

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);

  INFO(result.stats.status);
  REQUIRE(result.stats.success);
  REQUIRE(detail.mu_ineq.size() == options.inequality_dual_start.size());
  REQUIRE(detail.z_slack.size() == options.slack_start.size());
  CHECK(detail.mu_ineq.isApprox(options.inequality_dual_start, 0.0));
  CHECK(detail.z_slack.isApprox(options.slack_start, 0.0));
}

TEST_CASE("Phase-I many-row dual selector improves original stationarity",
          "[ipm][nlp][phase1][phase2][initialization][regression]") {
  constexpr int inequality_count = 10000;
  constexpr double primal_tolerance = 1e-4;
  constexpr double objective_gradient = 1e6;

  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back(
      {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
  nlp.x0 = Eigen::VectorXd::Zero(1);
  nlp.f = [](const Eigen::VectorXd& x) {
    return -objective_gradient * x[0];
  };
  nlp.grad = [](const Eigen::VectorXd&, Eigen::VectorXd& gradient) {
    gradient = Eigen::VectorXd::Constant(1, -objective_gradient);
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& inequality) {
    inequality.resize(inequality_count);
    for (int row = 0; row < inequality_count; ++row) {
      const double fraction = static_cast<double>(row + 1) /
          static_cast<double>(inequality_count);
      inequality[row] = x[0] - primal_tolerance * fraction;
    }
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                  Eigen::SparseMatrix<double>& jacobian) {
    jacobian.resize(inequality_count, 1);
    jacobian.reserve(inequality_count);
    for (int row = 0; row < inequality_count; ++row) {
      jacobian.insert(row, 0) = 1.0;
    }
  };

  for (const bool full_feasible_start_policy : {false, true}) {
    DYNAMIC_SECTION("full_feasible_start_policy="
                    << full_feasible_start_policy) {
      IPMOptions options;
      options.primal_feasible_start = full_feasible_start_policy;
      options.preserve_initial_point = !full_feasible_start_policy;
      options.max_iter = 1;
      options.tol_primal = primal_tolerance;
      options.tol_dual = 0.75 * objective_gradient;
      options.tol_complementarity = objective_gradient;
      options.tol_accept = 0.0;
      options.scale_problem = false;
      options.use_restoration_phase = false;
      options.mu_init = 0.0;
      options.mu_min = 0.0;

      const auto [result, detail] =
          NativeIPMAdapter(options).solve_nlp_detail(nlp);
      INFO(result.stats.status);
      REQUIRE(result.stats.success);
      REQUIRE(detail.z_slack.size() == inequality_count);
      REQUIRE(detail.mu_ineq.size() == inequality_count);
      REQUIRE((detail.mu_ineq.array() > 0.0).all());

      const double stationarity =
          std::abs(-objective_gradient + detail.mu_ineq.sum());
      CHECK(stationarity < objective_gradient);
      const double accumulation_condition =
          objective_gradient + detail.mu_ineq.lpNorm<1>();
      const double accumulation_gamma =
          static_cast<double>(inequality_count) *
          std::numeric_limits<double>::epsilon() /
          (1.0 - static_cast<double>(inequality_count) *
                     std::numeric_limits<double>::epsilon());
      CHECK(std::abs(stationarity - result.stats.dual_feas) <=
            accumulation_gamma * accumulation_condition);
      const Eigen::VectorXd products =
          detail.z_slack.cwiseProduct(detail.mu_ineq);
      CHECK(products.maxCoeff() > products.minCoeff());
      CHECK(detail.mu_ineq.maxCoeff() > 1.0);
    }
  }
}

TEST_CASE("Native IPM audits a complete central warm start",
          "[ipm][nlp][initialization][central-warm-start]") {
  double first_x = std::numeric_limits<double>::quiet_NaN();
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars.push_back({VarType::Continuous, 0.0, 1.0, "dispatch"});
  nlp.x0 = Eigen::VectorXd::Constant(1, -5e-3);
  nlp.f = [&first_x](const Eigen::VectorXd& x) {
    if (!std::isfinite(first_x)) first_x = x[0];
    return 0.5 * x.squaredNorm();
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient = x;
  };
  nlp.hess = [](const Eigen::VectorXd&,
                Eigen::SparseMatrix<double>& hessian) {
    hessian.resize(1, 1);
    hessian.setIdentity();
  };

  IPMOptions options;
  options.central_warm_start = true;
  options.central_warm_start_primal_tolerance = 1e-2;
  options.central_warm_start_centrality_tolerance = 0.5;
  options.equality_dual_start.resize(0);
  options.slack_start.resize(2);
  options.slack_start << 5e-3, 1.005;
  const double initial_barrier = 0.1;
  options.inequality_dual_start =
      initial_barrier * options.slack_start.cwiseInverse();
  options.max_iter = 1;
  options.tol_primal = 1e-8;
  options.tol_dual = 1e-8;
  options.tol_complementarity = 1e-8;
  options.tol_accept = 0.0;
  options.scale_problem = false;
  options.use_restoration_phase = false;

  const auto [result, detail] =
      NativeIPMAdapter(options).solve_nlp_detail(nlp);
  (void)result;

  CHECK(detail.central_warm_start_requested);
  CHECK(detail.central_warm_start_accepted);
  CHECK(detail.central_warm_start_rejection_reason.empty());
  CHECK(detail.central_warm_start_original_primal_violation == Approx(5e-3));
  CHECK(detail.central_warm_start_primal_residual == Approx(1e-2));
  CHECK(detail.central_warm_start_centrality == Approx(0.0).margin(1e-14));
  CHECK(detail.central_warm_start_mu == Approx(initial_barrier));
  CHECK(std::isfinite(detail.central_warm_start_dual_residual));
  CHECK(first_x == Approx(-5e-3).margin(1e-15));

  IPMOptions incomplete = options;
  incomplete.slack_start.resize(0);
  const auto [rejected_result, rejected_detail] =
      NativeIPMAdapter(incomplete).solve_nlp_detail(nlp);
  (void)rejected_result;
  CHECK_FALSE(rejected_detail.central_warm_start_accepted);
  CHECK(rejected_detail.central_warm_start_rejection_reason.find("slack") !=
        std::string::npos);
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

  INFO("status=" << result.stats.status
       << " primal=" << result.stats.primal_feas
       << " dual=" << result.stats.dual_feas
       << " complementarity=" << result.stats.complementarity);
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
  settings.max_tangent_dimension = static_cast<int>(w.rows());
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
  settings.max_tangent_dimension = static_cast<int>(w.rows());
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
  settings.max_tangent_dimension = static_cast<int>(w.rows());
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

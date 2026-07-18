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
  NativeIPMAdapter solver(options);
  const auto [result, detail] = solver.solve_nlp_detail(nlp);
  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.stats.primal_feas < 1e-8);
  CHECK(result.stats.dual_feas < 1e-8);
  CHECK(detail.complementarity < 1e-8);
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
  Eigen::VectorXd rhs2(5);
  rhs2 << -0.5, 1.5, 2.0, -1.0, 0.5;
  check_rhs(rhs2);

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
  CHECK(status.delta_w_used == Approx(4.0).margin(1e-7));
  CHECK(status.delta_w_used < 5.0);
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
  CHECK(status.n_zero == 2);

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

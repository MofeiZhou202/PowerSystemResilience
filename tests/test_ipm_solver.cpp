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

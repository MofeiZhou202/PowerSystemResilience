/// test_lp_solver.cpp
/// Tests for LP solving via the engine — exercises every registered LP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
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

// Helper: build a simple LP.
//   min  c' x
//   s.t. A x <= b,  Aeq x == beq,  lb <= x <= ub
static LPModel make_bounded_lp(
    Eigen::VectorXd c,
    Eigen::SparseMatrix<double> A,
    Eigen::VectorXd b,
    Eigen::SparseMatrix<double> Aeq,
    Eigen::VectorXd beq,
    Eigen::VectorXd lb,
    Eigen::VectorXd ub) {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c  = std::move(c);
  lp.A  = std::move(A);
  lp.b  = std::move(b);
  lp.Aeq  = std::move(Aeq);
  lp.beq  = std::move(beq);
  const int n = static_cast<int>(lp.c.size());
  for (int i = 0; i < n; ++i) {
    lp.vars.push_back({VarType::Continuous, lb[i], ub[i]});
  }
  return lp;
}

// ─── Trivial 1-variable LP ─────────────────────────────────────────────────
// min -x   s.t.  x <= 5,  x >= 0
// Optimal: x=5, obj=-5
TEST_CASE("LP: 1-variable bounded", "[lp]") {
  SolverEngine eng;

  Eigen::VectorXd c(1); c << -1.0;
  Eigen::SparseMatrix<double> A(1, 1); A.insert(0, 0) = 1.0; A.makeCompressed();
  Eigen::VectorXd b(1); b << 5.0;
  Eigen::SparseMatrix<double> Aeq(0, 1); Aeq.makeCompressed();
  Eigen::VectorXd beq(0);
  Eigen::VectorXd lb(1); lb << 0.0;
  Eigen::VectorXd ub(1); ub << 1e20;

  auto lp = make_bounded_lp(c, A, b, Aeq, beq, lb, ub);

  // Run with every registered LP solver.
  for (const auto& solver_name : eng.list_solvers(ProblemClass::LP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_lp(lp, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(-5.0).margin(1e-4));
        CHECK(res.x[0] == Approx(5.0).margin(1e-4));
      }
    }
  }
}

// ─── 2-variable LP: classic example ───────────────────────────────────────
// max  3x + 2y
// s.t. x + y  <= 4
//      x      <= 3
//      y      <= 3
//      x,y    >= 0
// Optimal: x=3, y=1, obj=11
TEST_CASE("LP: 2-variable production planning (max)", "[lp]") {
  SolverEngine eng;

  LPModel lp;
  lp.sense = Sense::Maximize;
  lp.c.resize(2);
  lp.c << 3.0, 2.0;

  Eigen::SparseMatrix<double> A(3, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;  // x+y <= 4
  A.insert(1, 0) = 1.0;                          // x   <= 3
  A.insert(2, 1) = 1.0;                          // y   <= 3
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(3);
  lp.b << 4.0, 3.0, 3.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  for (const auto& solver_name : eng.list_solvers(ProblemClass::LP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_lp(lp, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(11.0).margin(1e-4));
      }
    }
  }
}

// ─── Equality constrained LP ──────────────────────────────────────────────
// min  x + y
// s.t. x + y == 3
//      x, y  >= 0
// Optimal: any (x,y) with x+y=3, obj=3
TEST_CASE("LP: equality constrained", "[lp]") {
  SolverEngine eng;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;

  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 3.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  for (const auto& solver_name : eng.list_solvers(ProblemClass::LP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_lp(lp, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(3.0).margin(1e-4));
        CHECK(res.x.sum() == Approx(3.0).margin(1e-4));
      }
    }
  }
}

// ─── Infeasible LP ────────────────────────────────────────────────────────
// x + y <= 1  AND  x + y >= 2  (contradictory → infeasible)
// Use Gurobi or HiGHS explicitly: native IPM may not detect infeasibility.
TEST_CASE("LP: infeasible problem — Gurobi detects correctly", "[lp]") {
  SolverEngine eng;

  // Only run this check if Gurobi is registered.
  auto solvers = eng.list_solvers(ProblemClass::LP);
  bool has_gurobi = std::any_of(solvers.begin(), solvers.end(),
                                 [](const std::string& s) {
                                   return s.find("gurobi") != std::string::npos ||
                                          s.find("Gurobi") != std::string::npos;
                                 });
  if (!has_gurobi) {
    SUCCEED("Gurobi not available, skipping infeasibility test");
    return;
  }

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;   // x + y <= 1
  A.insert(1, 0) = -1.0; A.insert(1, 1) = -1.0; // x + y >= 2
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 1.0, -2.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SolveOptions opts;
  opts.preferred_solver = "gurobi";
  opts.allow_fallback = false;

  auto res = eng.solve_lp(lp, opts);
  CHECK_FALSE(res.stats.success);
  // status may be "Infeasible" or "Gurobi status=N" for inf-or-unbd
  CHECK_FALSE(res.stats.status.empty());
}

// ─── LP dual values ───────────────────────────────────────────────────────
// max  x + 2y
// s.t. x + y  <= 4   (binding at optimal)
//      x      >= 0
//      y      >= 0
// Optimal: x=0, y=4, obj=8; dual of first constraint should be 2.
TEST_CASE("LP: dual variables are returned", "[lp]") {
  SolverEngine eng;

  LPModel lp;
  lp.sense = Sense::Maximize;
  lp.c.resize(2);
  lp.c << 1.0, 2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 4.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto res = eng.solve_lp(lp);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(8.0).margin(1e-4));
  // constraint_duals should be non-empty for LP
  CHECK(res.constraint_duals.size() == 1);
}

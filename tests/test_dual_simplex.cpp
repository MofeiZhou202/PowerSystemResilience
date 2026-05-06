/// test_dual_simplex.cpp
/// Tests for the native dual-simplex LP kernel via solve_lp_with_basis().
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

// ─── 2-variable LP solved via dual simplex ────────────────────────────────
// min -3x - 2y  s.t.  x+y <= 4,  x >= 0,  y >= 0
// Optimal: x=4, y=0, obj=-12
TEST_CASE("DualSimplex: 2-variable LP", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0, -2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 4.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SimplexOptions opts;
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(-12.0).margin(1e-5));
  CHECK(res.result.x[0] == Approx(4.0).margin(1e-5));
  CHECK(res.result.x[1] == Approx(0.0).margin(1e-5));
}

// ─── 3-variable LP ────────────────────────────────────────────────────────
// min  x1 + x2 + x3
// s.t. x1 + x2 >= 2
//      x2 + x3 >= 2
//      all >= 0
// Optimal: x1=0, x2=2, x3=0, obj=2
TEST_CASE("DualSimplex: 3-variable LP with >= constraints", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(3);
  lp.c << 1.0, 1.0, 1.0;

  // Convert x1+x2 >= 2 and x2+x3 >= 2 to -x <= -2
  Eigen::SparseMatrix<double> A(2, 3);
  A.insert(0, 0) = -1.0; A.insert(0, 1) = -1.0;
  A.insert(1, 1) = -1.0; A.insert(1, 2) = -1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << -2.0, -2.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SimplexOptions opts;
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(2.0).margin(1e-5));
}

// ─── Equality constrained LP ──────────────────────────────────────────────
// min  x + y   s.t.  x + y = 4,  x,y >= 0
// Optimal: obj=4, x+y=4
TEST_CASE("DualSimplex: equality constrained LP", "[dual_simplex]") {
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
  lp.beq << 4.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SimplexOptions opts;
  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(4.0).margin(1e-5));
}

// ─── Basis round-trip: solve then warm-start ──────────────────────────────
TEST_CASE("DualSimplex: warm-start from previous basis", "[dual_simplex]") {
  // Same LP twice: second solve uses returned basis as hint.
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -2.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 2.0;
  A.insert(1, 0) = 2.0; A.insert(1, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 14.0, 14.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  SimplexOptions opts;
  auto first = solve_lp_with_basis(lp, opts);
  REQUIRE(first.result.stats.success);

  // Re-solve with basis hint – should reach optimality in 0 pivots.
  auto second = solve_lp_with_basis(lp, opts, &first.basis);
  REQUIRE(second.result.stats.success);
  CHECK(second.result.stats.objective ==
        Approx(first.result.stats.objective).margin(1e-6));
}

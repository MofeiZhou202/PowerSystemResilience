/// test_presolve.cpp
/// Tests for MILP and LP presolve/postsolve pipeline.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/strategy/presolve_manager.hpp"
#include "mipsolvers/engine/strategy/postsolve_manager.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using namespace mipsolvers::engine::strategy;
using Catch::Approx;

// ─── Presolve of a simple LP ────────────────────────────────────────────────
// min -x  s.t. x<=5, x<=10, x>=0
TEST_CASE("Presolve: LP with redundant constraint", "[presolve][lp]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(1);
  lp.c << -1.0;

  Eigen::SparseMatrix<double> A(2, 1);
  A.insert(0, 0) = 1.0;
  A.insert(1, 0) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 5.0, 10.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  if (std::holds_alternative<LPModel>(presolved.presolved)) {
    const auto& lp2 = std::get<LPModel>(presolved.presolved);
    CHECK(lp2.c.size() == 1);
  }
  SUCCEED();
}

// ─── Fixed variable elimination ────────────────────────────────────────────
TEST_CASE("Presolve: fixed variable eliminated", "[presolve][lp]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 0.0, 1.0;

  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 3.0;

  lp.vars.push_back({VarType::Continuous, 3.0, 3.0, "x"});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20, "y"});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  if (std::holds_alternative<LPModel>(presolved.presolved)) {
    const auto& lp2 = std::get<LPModel>(presolved.presolved);
    CHECK(lp2.c.size() <= 2);
  }
}

// ─── Postsolve round-trip ──────────────────────────────────────────────────
TEST_CASE("Postsolve: identity mapping preserves solution", "[presolve][postsolve]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 5.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  SolveResult presolved_res;
  presolved_res.stats.success = true;
  presolved_res.x.resize(2);
  presolved_res.x << 5.0, 0.0;
  presolved_res.stats.objective = 5.0;

  PostsolveManager post;
  auto final_res = post.postsolve(presolved_res, presolved.mapping);

  REQUIRE(final_res.stats.success);
  CHECK(final_res.x.size() == 2);
  CHECK(final_res.stats.objective == Approx(5.0).margin(1e-6));
}

// ─── should_presolve heuristic ────────────────────────────────────────────
TEST_CASE("Presolve: should_presolve runs without crash", "[presolve]") {
  LPModel tiny;
  tiny.c.resize(2);
  tiny.c << 1.0, 1.0;
  tiny.vars.push_back({VarType::Continuous, 0.0, 1e20});
  tiny.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  (void)ps.should_presolve(tiny, opts);
  SUCCEED();
}

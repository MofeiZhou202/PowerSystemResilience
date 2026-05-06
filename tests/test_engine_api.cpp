/// test_engine_api.cpp
/// Tests for the top-level SolverEngine API and basic problem types.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/adapter_registry.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

// ─── SolverEngine construction ────────────────────────────────────────────────
TEST_CASE("SolverEngine default construction registers adapters", "[engine][api]") {
  SolverEngine eng(/*register_defaults=*/true);

  // At least one adapter for LP should be registered given HiGHS is embedded.
  auto lp_adapters = eng.list_solvers(ProblemClass::LP);
  CHECK_FALSE(lp_adapters.empty());
}

TEST_CASE("SolverEngine empty construction registers no adapters", "[engine][api]") {
  SolverEngine eng(/*register_defaults=*/false);
  auto lp_adapters = eng.list_solvers(ProblemClass::LP);
  CHECK(lp_adapters.empty());
}

TEST_CASE("SolverEngine adapter registration", "[engine][api]") {
  SolverEngine eng(false);
  CHECK(eng.list_solvers(ProblemClass::LP).empty());

  eng.register_default_adapters();
  CHECK_FALSE(eng.list_solvers(ProblemClass::LP).empty());
}

// ─── Simple LP via SolverEngine ───────────────────────────────────────────────
// min  -x - y
// s.t. x + y <= 2
//      x     >= 0
//      y     >= 0
// Optimal: x=1, y=1, obj=-2  (or any convex combination summing to 2)
TEST_CASE("SolverEngine solves a simple LP", "[engine][api][lp]") {
  SolverEngine eng;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -1.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 2.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto result = eng.solve_lp(lp);
  REQUIRE(result.stats.success);
  CHECK(result.stats.objective == Approx(-2.0).margin(1e-4));
  CHECK(result.x.size() == 2);
  CHECK(result.x.sum() == Approx(2.0).margin(1e-4));
}

// ─── ProblemVariant dispatch ───────────────────────────────────────────────────
TEST_CASE("api::problem_class correctly identifies problem types", "[engine][api]") {
  using namespace mipsolvers::engine;

  LPModel lp;
  MIPModel mip;
  SparseLinSys le;

  CHECK(api::problem_class(LPModel{}) == ProblemClass::LP);
  CHECK(api::problem_class(MIPModel{}) == ProblemClass::MILP);
  CHECK(api::problem_class(SparseLinSys{}) == ProblemClass::LE);
  CHECK(api::problem_class(NLPModel{}) == ProblemClass::NLP);
  CHECK(api::problem_class(QPModel{}) == ProblemClass::QP);
  CHECK(api::problem_class(NonlinearSystem{}) == ProblemClass::NLE);
  CHECK(api::problem_class(MINLPModel{}) == ProblemClass::MINLP);
}

TEST_CASE("api::problem_class_name returns correct strings", "[engine][api]") {
  using namespace mipsolvers::engine::api;
  CHECK(problem_class_name(ProblemClass::LP)    == "LP");
  CHECK(problem_class_name(ProblemClass::MILP)  == "MILP");
  CHECK(problem_class_name(ProblemClass::LE)    == "LE");
  CHECK(problem_class_name(ProblemClass::NLE)   == "NLE");
  CHECK(problem_class_name(ProblemClass::NLP)   == "NLP");
  CHECK(problem_class_name(ProblemClass::QP)    == "QP");
  CHECK(problem_class_name(ProblemClass::MINLP) == "MINLP");
}

// ─── SolveOptions ─────────────────────────────────────────────────────────────
TEST_CASE("SolveOptions defaults are sane", "[engine][api]") {
  SolveOptions opts;
  CHECK(opts.allow_fallback);
  CHECK(opts.preferred_solver.empty());
  CHECK(opts.strategy_policy == StrategyPolicy::Auto);
}

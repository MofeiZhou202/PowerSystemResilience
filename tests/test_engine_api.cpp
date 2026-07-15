/// test_engine_api.cpp
/// Tests for the top-level SolverEngine API and basic problem types.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <stdexcept>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#include "mipsolvers/engine/solver/adapter_registry.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

#ifdef HACDCPF_HAVE_IPOPT
namespace {

NLPModel make_rosenbrock_nlp() {
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars = {
      VariableMeta{VarType::Continuous, -5.0, 5.0},
      VariableMeta{VarType::Continuous, -5.0, 5.0},
  };
  nlp.x0.resize(2);
  nlp.x0 << -1.2, 1.0;
  nlp.f = [](const Eigen::VectorXd& x) {
    const double a = x[1] - x[0] * x[0];
    const double b = 1.0 - x[0];
    return 100.0 * a * a + b * b;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    grad.resize(2);
    const double a = x[1] - x[0] * x[0];
    grad[0] = -400.0 * x[0] * a - 2.0 * (1.0 - x[0]);
    grad[1] = 200.0 * a;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x[0] + x[1] - 2.0;
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                 Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 1.0;
    jac.insert(0, 1) = 1.0;
    jac.makeCompressed();
  };
  return nlp;
}

}  // namespace
#endif

// ─── SolverEngine construction ────────────────────────────────────────────────
TEST_CASE("SolverEngine default construction registers adapters", "[engine][api]") {
  SolverEngine eng(/*register_defaults=*/true);

  // At least one adapter for LP should be registered given HiGHS is embedded.
  auto lp_adapters = eng.list_solvers(ProblemClass::LP);
  CHECK_FALSE(lp_adapters.empty());
}

#ifndef HACDCPF_HAVE_GUROBI
TEST_CASE("SolverEngine default adapters omit Gurobi when not compiled",
          "[engine][api][gurobi]") {
  SolverEngine eng(/*register_defaults=*/true);

  auto does_not_contain_gurobi = [](const std::vector<std::string>& solvers) {
    return std::find(solvers.begin(), solvers.end(), "Gurobi") == solvers.end();
  };

  CHECK(does_not_contain_gurobi(eng.list_solvers(ProblemClass::LP)));
  CHECK(does_not_contain_gurobi(eng.list_solvers(ProblemClass::QP)));
  CHECK(does_not_contain_gurobi(eng.list_solvers(ProblemClass::MILP)));
}
#endif

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

#ifdef HACDCPF_HAVE_IPOPT
TEST_CASE("IpoptAdapter honors NLPModel iteration and tolerance options",
          "[engine][api][ipopt][options]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  NLPModel capped = make_rosenbrock_nlp();
  capped.solver_options.max_iterations = 1;
  capped.solver_options.tolerance = 1e-12;
  capped.solver_options.acceptable_tolerance = 1e-12;
  const SolveResult short_run = ipopt.solve_nlp(capped);
  INFO("short Ipopt status=" << short_run.stats.status
       << " iterations=" << short_run.stats.iterations
       << " primal=" << short_run.stats.primal_feas
       << " dual=" << short_run.stats.dual_feas);
  CHECK_FALSE(short_run.stats.success);
  CHECK(short_run.stats.iterations <= 1);
  CHECK(short_run.stats.status == "Max iterations exceeded");

  NLPModel converged = make_rosenbrock_nlp();
  converged.solver_options.max_iterations = 500;
  converged.solver_options.tolerance = 1e-9;
  converged.solver_options.acceptable_tolerance = 1e-7;
  const SolveResult full_run = ipopt.solve_nlp(converged);
  INFO("full Ipopt status=" << full_run.stats.status
       << " iterations=" << full_run.stats.iterations
       << " primal=" << full_run.stats.primal_feas
       << " dual=" << full_run.stats.dual_feas);
  REQUIRE(full_run.stats.success);
  REQUIRE(full_run.x.size() == 2);
  CHECK(std::abs(full_run.x.sum() - 2.0) < 1e-7);
  CHECK(full_run.stats.objective < converged.f(converged.x0));
  CHECK(full_run.stats.iterations > short_run.stats.iterations);
  CHECK(full_run.stats.iterations <
        converged.solver_options.max_iterations);
}
#endif

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

TEST_CASE("SolverEngine rejects invalid LP at public API boundary", "[engine][api][validation]") {
  SolverEngine eng(/*register_defaults=*/false);

  LPModel lp;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  Eigen::SparseMatrix<double> A(1, 3);
  A.insert(0, 0) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 1.0;

  CHECK_THROWS_AS(eng.solve_lp(lp), std::invalid_argument);
}

TEST_CASE("SolverEngine normalizes unconstrained LP before validation", "[engine][api][validation]") {
  SolverEngine eng(/*register_defaults=*/false);

  LPModel lp;
  lp.c.resize(1);
  lp.c << 1.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  const auto result = eng.solve_lp(lp);
  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.status == "No adapter registered for class LP");
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

TEST_CASE("EigenSparseLU handles empty square systems", "[engine][api][linear-solver]") {
  EigenSparseLUSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  solver.analyze_pattern(a);
  CHECK(solver.factorize(a));

  Eigen::VectorXd rhs(0);
  Eigen::VectorXd x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

TEST_CASE("Default sparse solver handles empty square systems", "[engine][api][linear-solver]") {
  auto solver = make_default_sparse_solver();
  REQUIRE(solver != nullptr);

  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  solver->analyze_pattern(a);
  CHECK(solver->factorize(a));

  Eigen::VectorXd rhs(0);
  Eigen::VectorXd x;
  CHECK(solver->solve(rhs, x));
  CHECK(x.size() == 0);
}

// ─── Shared helper: run a 3×3 sparse solve through any SparseLinearSolver ────
// System:  [ 4  1  0 ] [x]   [9 ]
//          [ 1  3  1 ] [y] = [10]
//          [ 0  1  2 ] [z]   [7 ]
// Solution: x=1, y=2, z=2.5  (but exact fractions, tolerance 1e-10)
static void run_sparse_solve_3x3(SparseLinearSolver& solver) {
  Eigen::SparseMatrix<double> A(3, 3);
  A.insert(0, 0) = 4.0; A.insert(0, 1) = 1.0;
  A.insert(1, 0) = 1.0; A.insert(1, 1) = 3.0; A.insert(1, 2) = 1.0;
  A.insert(2, 1) = 1.0; A.insert(2, 2) = 2.0;
  A.makeCompressed();

  Eigen::VectorXd rhs(3);
  rhs << 6.0, 10.0, 7.0;  // rhs chosen so x=1, y=2, z=2.5

  // Correct rhs: A*[1,2,2.5]^T = [4+2,1+6+2.5,2+5] = [6,9.5,7] — recompute
  // A*[1,2,2.5]^T:
  //   row0: 4*1 + 1*2 + 0     = 6
  //   row1: 1*1 + 3*2 + 1*2.5 = 1+6+2.5 = 9.5
  //   row2: 0   + 1*2 + 2*2.5 = 2+5     = 7
  rhs << 6.0, 9.5, 7.0;

  // --- test with analyze_pattern then factorize (normal path) ---
  solver.analyze_pattern(A);
  REQUIRE(solver.factorize(A));
  Eigen::VectorXd x;
  REQUIRE(solver.solve(rhs, x));
  REQUIRE(x.size() == 3);
  CHECK(x[0] == Approx(1.0).epsilon(1e-10));
  CHECK(x[1] == Approx(2.0).epsilon(1e-10));
  CHECK(x[2] == Approx(2.5).epsilon(1e-10));
}

static void run_factorize_without_analyze(SparseLinearSolver& solver) {
  // Bug fix #1: factorize() called without prior analyze_pattern() must not
  // crash or assert-fail; it should auto-run analyzePattern internally.
  Eigen::SparseMatrix<double> A(3, 3);
  A.insert(0, 0) = 4.0; A.insert(0, 1) = 1.0;
  A.insert(1, 0) = 1.0; A.insert(1, 1) = 3.0; A.insert(1, 2) = 1.0;
  A.insert(2, 1) = 1.0; A.insert(2, 2) = 2.0;
  A.makeCompressed();

  Eigen::VectorXd rhs(3);
  rhs << 6.0, 9.5, 7.0;

  // Intentionally skip analyze_pattern()
  REQUIRE(solver.factorize(A));
  Eigen::VectorXd x;
  REQUIRE(solver.solve(rhs, x));
  CHECK(x[0] == Approx(1.0).epsilon(1e-10));
  CHECK(x[1] == Approx(2.0).epsilon(1e-10));
  CHECK(x[2] == Approx(2.5).epsilon(1e-10));
}

// ─── EigenSparseLU ────────────────────────────────────────────────────────────
TEST_CASE("EigenSparseLU: 3x3 sparse solve", "[engine][linear-solver][eigen]") {
  EigenSparseLUSolver solver;
  run_sparse_solve_3x3(solver);
}

TEST_CASE("EigenSparseLU: empty-system path (no analyze_pattern)", "[engine][linear-solver][eigen]") {
  EigenSparseLUSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  // factorize without analyze_pattern on empty system
  CHECK(solver.factorize(a));
  Eigen::VectorXd rhs(0), x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

// ─── Default solver (dispatches to best available backend) ───────────────────
TEST_CASE("Default sparse solver: 3x3 correctness + backend reported",
          "[engine][linear-solver][default]") {
  auto solver = make_default_sparse_solver();
  REQUIRE(solver != nullptr);
  INFO("Backend in use: " << solver->backend_name());
  run_sparse_solve_3x3(*solver);
}

TEST_CASE("Default sparse solver: factorize without analyze_pattern",
          "[engine][linear-solver][default]") {
  auto solver = make_default_sparse_solver();
  REQUIRE(solver != nullptr);
  // Only SuperLU and Pardiso have the auto-analyze fix; EigenSparseLU and
  // SuiteSparse wrappers rely on Eigen which handles this gracefully already.
  // This test exercises whichever backend is active.
  run_factorize_without_analyze(*solver);
}

// ─── SuperLU (compiled in only when HACDCPF_HAVE_SUPERLU is set) ─────────────
#ifdef HACDCPF_HAVE_SUPERLU
TEST_CASE("SuperLUSolver: 3x3 sparse solve", "[engine][linear-solver][superlu]") {
  SuperLUSolver solver;
  run_sparse_solve_3x3(solver);
}

TEST_CASE("SuperLUSolver: factorize without prior analyze_pattern",
          "[engine][linear-solver][superlu]") {
  SuperLUSolver solver;
  run_factorize_without_analyze(solver);
}

TEST_CASE("SuperLUSolver: empty square system", "[engine][linear-solver][superlu]") {
  SuperLUSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  solver.analyze_pattern(a);
  CHECK(solver.factorize(a));
  Eigen::VectorXd rhs(0), x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

TEST_CASE("SuperLUSolver: backend name is correct", "[engine][linear-solver][superlu]") {
  SuperLUSolver solver;
  CHECK(std::string(solver.backend_name()) == "SuperLU(Eigen)");
}
#endif  // HACDCPF_HAVE_SUPERLU

// ─── MKL PARDISO (compiled in only when HACDCPF_HAVE_MKL_PARDISO is set) ─────
#ifdef HACDCPF_HAVE_MKL_PARDISO
TEST_CASE("MKLPardisoSolver: 3x3 sparse solve", "[engine][linear-solver][pardiso]") {
  MKLPardisoSolver solver;
  run_sparse_solve_3x3(solver);
}

TEST_CASE("MKLPardisoSolver: factorize without prior analyze_pattern",
          "[engine][linear-solver][pardiso]") {
  MKLPardisoSolver solver;
  run_factorize_without_analyze(solver);
}

TEST_CASE("MKLPardisoSolver: empty square system", "[engine][linear-solver][pardiso]") {
  MKLPardisoSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  solver.analyze_pattern(a);
  CHECK(solver.factorize(a));
  Eigen::VectorXd rhs(0), x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

TEST_CASE("MKLPardisoSolver: backend name is correct", "[engine][linear-solver][pardiso]") {
  MKLPardisoSolver solver;
  CHECK(std::string(solver.backend_name()) == "Intel-MKL-PARDISO(Eigen)");
}
#endif  // HACDCPF_HAVE_MKL_PARDISO

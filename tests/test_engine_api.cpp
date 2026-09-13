/// test_engine_api.cpp
/// Tests for the top-level SolverEngine API and basic problem types.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"
#include "mipsolvers/engine/solver/adapter_registry.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

class RecordingLPAdapter final : public SolverAdapter {
 public:
  RecordingLPAdapter(std::string adapter_name, bool succeed,
                     std::shared_ptr<int> calls)
      : adapter_name_(std::move(adapter_name)),
        succeed_(succeed),
        calls_(std::move(calls)) {}

  std::string name() const override { return adapter_name_; }
  bool supports(ProblemClass cls) const override {
    return cls == ProblemClass::LP;
  }
  SolveResult solve_lp(const LPModel&) const override {
    ++*calls_;
    SolveResult out;
    out.stats.success = succeed_;
    out.stats.solver_name = adapter_name_;
    out.stats.status = succeed_ ? "Optimal" : "Synthetic failure";
    out.x = Eigen::VectorXd::Zero(1);
    return out;
  }

 private:
  std::string adapter_name_;
  bool succeed_{false};
  std::shared_ptr<int> calls_;
};

LPModel make_dispatch_lp() {
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  lp.A.resize(0, 1);
  lp.b.resize(0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  return lp;
}

}  // namespace

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

#ifndef HACDCPF_HAVE_CPLEX
TEST_CASE("SolverEngine default adapters omit CPLEX when not compiled",
          "[engine][api][cplex]") {
  SolverEngine eng(/*register_defaults=*/true);
  const auto solvers = eng.list_solvers(ProblemClass::MILP);
  CHECK(std::find(solvers.begin(), solvers.end(), "CPLEX") == solvers.end());
}
#else
TEST_CASE("SolverEngine registers CPLEX when its environment is available",
          "[engine][api][cplex]") {
  CplexAdapter adapter;
  SolverEngine eng(/*register_defaults=*/true);
  const auto solvers = eng.list_solvers(ProblemClass::MILP);
  const bool registered =
      std::find(solvers.begin(), solvers.end(), "CPLEX") != solvers.end();
  CHECK(registered == adapter.available());
}

TEST_CASE("CPLEX preserves a two-sided MILP row",
          "[engine][api][cplex][milp]") {
  CplexOptions options;
  options.time_limit_sec = 10.0;
  options.threads = 1;
  CplexAdapter adapter(options);
  if (!adapter.available()) SKIP("CPLEX environment or license unavailable");

  // min x+y subject to 1 <= x+y <= 2, x,y integer in [0,2].
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c = Eigen::Vector2d::Ones();
  mip.linear_part.A.resize(1, 2);
  mip.linear_part.A.insert(0, 0) = 1.0;
  mip.linear_part.A.insert(0, 1) = 1.0;
  mip.linear_part.A.makeCompressed();
  mip.linear_part.row_lhs = Eigen::VectorXd::Constant(1, 1.0);
  mip.linear_part.b = Eigen::VectorXd::Constant(1, 2.0);
  mip.linear_part.Aeq.resize(0, 2);
  mip.linear_part.beq.resize(0);
  mip.linear_part.vars = {
      VariableMeta{VarType::Integer, 0.0, 2.0},
      VariableMeta{VarType::Integer, 0.0, 2.0},
  };
  mip.integer_idx = {0, 1};

  const SolveResult result = adapter.solve_milp(mip);
  const CplexSolveInfo info = last_cplex_solve_info();
  INFO("status=" << result.stats.status);
  REQUIRE(result.stats.success);
  CHECK(info.optimal);
  CHECK(info.proven);
  // Fixed objective gate: integration derivation, Quantitative prediction.
  CHECK(result.stats.objective == Approx(1.0).margin(1e-4));
  REQUIRE(result.x.size() == 2);
  CHECK(result.x.sum() == Approx(1.0).margin(1e-4));
}

TEST_CASE("CPLEX maps equality, objective sense, integer types and MIP start",
          "[engine][api][cplex][milp]") {
  CplexOptions options;
  options.time_limit_sec = 10.0;
  options.threads = 1;
  CplexAdapter adapter(options);
  if (!adapter.available()) SKIP("CPLEX environment or license unavailable");

  // max 3x+2y subject to x+y=2, x binary, y integer in [0,2].
  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 3.0, 2.0;
  mip.linear_part.A.resize(0, 2);
  mip.linear_part.b.resize(0);
  mip.linear_part.Aeq.resize(1, 2);
  mip.linear_part.Aeq.insert(0, 0) = 1.0;
  mip.linear_part.Aeq.insert(0, 1) = 1.0;
  mip.linear_part.Aeq.makeCompressed();
  mip.linear_part.beq = Eigen::VectorXd::Constant(1, 2.0);
  mip.linear_part.vars = {
      VariableMeta{VarType::Binary, 0.0, 1.0},
      VariableMeta{VarType::Integer, 0.0, 2.0},
  };
  mip.binary_idx = {0};
  mip.integer_idx = {1};
  mip.initial_solution.resize(2);
  mip.initial_solution << 0.0, 2.0;

  const SolveResult result = adapter.solve_milp(mip);
  const CplexSolveInfo info = last_cplex_solve_info();
  INFO("status=" << result.stats.status);
  REQUIRE(result.stats.success);
  CHECK(info.optimal);
  REQUIRE(info.best_bound.has_value());
  CHECK(*info.best_bound == Approx(5.0).margin(1e-4));
  CHECK(result.stats.objective == Approx(5.0).margin(1e-4));
  REQUIRE(result.x.size() == 2);
  CHECK(result.x[0] == Approx(1.0).margin(1e-6));
  CHECK(result.x[1] == Approx(1.0).margin(1e-6));
}
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

TEST_CASE("Auto dispatch excludes Gurobi while explicit fallback remains available",
          "[engine][api][dispatch][fallback]") {
  auto gurobi_calls = std::make_shared<int>(0);
  auto native_calls = std::make_shared<int>(0);
  SolverEngine eng(false);
  eng.register_adapter(std::make_shared<RecordingLPAdapter>(
      "NativeIPMLP", true, native_calls));
  eng.register_adapter(std::make_shared<RecordingLPAdapter>(
      "Gurobi", false, gurobi_calls));

  const auto listed = eng.list_solvers(ProblemClass::LP);
  REQUIRE(listed.size() == 2);
  CHECK(listed[0] == "NativeIPMLP");
  CHECK(listed[1] == "Gurobi");

  SolveOptions options;
  options.preferred_solver = "Gurobi";
  options.allow_fallback = true;
  const auto result = eng.solve_lp(make_dispatch_lp(), options);
  REQUIRE(result.stats.success);
  CHECK(result.stats.solver_name == "NativeIPMLP");
  CHECK(*gurobi_calls == 1);
  CHECK(*native_calls == 1);
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
  REQUIRE(short_run.constraint_duals.size() == 1);
  REQUIRE(short_run.box_dual_lb.size() == 2);
  REQUIRE(short_run.box_dual_ub.size() == 2);
  Eigen::VectorXd short_gradient;
  capped.grad(short_run.x, short_gradient);
  const Eigen::VectorXd short_stationarity =
      short_gradient +
      Eigen::VectorXd::Constant(2, short_run.constraint_duals[0]) -
      short_run.box_dual_lb + short_run.box_dual_ub;
  const double short_multiplier_scale = 1.0 + std::max({
      short_run.constraint_duals.cwiseAbs().maxCoeff(),
      short_run.box_dual_lb.cwiseAbs().maxCoeff(),
      short_run.box_dual_ub.cwiseAbs().maxCoeff()});
  CHECK(short_run.stats.unscaled_dual_feas ==
        Approx(short_stationarity.cwiseAbs().maxCoeff() /
               short_multiplier_scale).margin(1e-8));

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

TEST_CASE("IpoptAdapter returns NLP multipliers in engine KKT convention",
          "[engine][api][ipopt][duals]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  // min 0.5*(x-2)^2 + 0.5*(y+1)^2
  // s.t. x+y-1=0, x-0.5<=0, y>=0.
  // The solution is (0.5, 0.5), with lambda_eq=-1.5,
  // mu_{x<=0.5}=3.0 and z_L(y)=0.
  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars = {
      VariableMeta{VarType::Continuous, -5.0, 5.0},
      VariableMeta{VarType::Continuous, 0.0, 5.0},
  };
  nlp.x0 = Eigen::Vector2d(0.25, 0.75);
  nlp.f = [](const Eigen::VectorXd& x) {
    return 0.5 * std::pow(x[0] - 2.0, 2) +
           0.5 * std::pow(x[1] + 1.0, 2);
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    grad.resize(2);
    grad << x[0] - 2.0, x[1] + 1.0;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g = Eigen::VectorXd::Constant(1, x[0] + x[1] - 1.0);
  };
  nlp.jac_g = [](const Eigen::VectorXd&,
                 Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 1.0;
    jac.insert(0, 1) = 1.0;
    jac.makeCompressed();
  };
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    h = Eigen::VectorXd::Constant(1, x[0] - 0.5);
  };
  nlp.jac_h = [](const Eigen::VectorXd&,
                 Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 1.0;
    jac.makeCompressed();
  };
  nlp.solver_options.tolerance = 1e-10;
  nlp.solver_options.acceptable_tolerance = 1e-9;

  const SolveResult result = ipopt.solve_nlp(nlp);
  REQUIRE(result.stats.success);
  REQUIRE(result.constraint_duals.size() == 2);
  REQUIRE(result.box_dual_lb.size() == 2);
  REQUIRE(result.box_dual_ub.size() == 2);
  CHECK(result.x[0] == Approx(0.5).margin(1e-7));
  CHECK(result.x[1] == Approx(0.5).margin(1e-7));
  CHECK(result.constraint_duals[0] == Approx(3.0).margin(1e-5));
  CHECK(result.constraint_duals[1] == Approx(-1.5).margin(1e-5));
  CHECK(result.box_dual_lb[1] >= 0.0);
  CHECK(result.box_dual_ub[0] >= 0.0);

  NLPModel warm = nlp;
  warm.x0 = result.x;
  warm.constraint_dual_start = result.constraint_duals;
  warm.box_dual_lb_start = result.box_dual_lb;
  warm.box_dual_ub_start = result.box_dual_ub;
  warm.solver_options.primal_dual_warm_start = true;
  const SolveResult repeated = ipopt.solve_nlp(warm);
  REQUIRE(repeated.stats.success);
  CHECK(repeated.stats.warm_start_used);
  CHECK(repeated.stats.iterations <= result.stats.iterations);
  CHECK(repeated.constraint_duals[0] == Approx(3.0).margin(1e-5));
  CHECK(repeated.constraint_duals[1] == Approx(-1.5).margin(1e-5));
}

TEST_CASE("IpoptAdapter unscales active variable-bound multipliers",
          "[engine][api][ipopt][duals][bounds]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  nlp.vars = {VariableMeta{VarType::Continuous, 0.0, 5.0}};
  nlp.x0 = Eigen::VectorXd::Constant(1, 1.0);
  nlp.f = [](const Eigen::VectorXd& x) {
    return 0.5 * std::pow(x[0] + 1.0, 2);
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    grad = Eigen::VectorXd::Constant(1, x[0] + 1.0);
  };
  nlp.solver_options.tolerance = 1e-10;
  nlp.solver_options.acceptable_tolerance = 1e-9;

  const SolveResult result = ipopt.solve_nlp(nlp);
  REQUIRE(result.stats.success);
  REQUIRE(result.box_dual_lb.size() == 1);
  REQUIRE(result.box_dual_ub.size() == 1);
  CHECK(result.x[0] == Approx(0.0).margin(1e-7));
  CHECK(result.box_dual_lb[0] == Approx(1.0).margin(1e-6));
  CHECK(result.box_dual_ub[0] == Approx(0.0).margin(1e-6));
}

TEST_CASE("IpoptAdapter reuses complete primal-dual NLP starts",
          "[engine][api][ipopt][warm-start]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  NLPModel base = make_rosenbrock_nlp();
  base.solver_options.tolerance = 1e-10;
  base.solver_options.acceptable_tolerance = 1e-9;
  const SolveResult solved = ipopt.solve_nlp(base);
  REQUIRE(solved.stats.success);
  REQUIRE(solved.constraint_duals.size() == 1);
  REQUIRE(solved.box_dual_lb.size() == 2);
  REQUIRE(solved.box_dual_ub.size() == 2);

  NLPModel warm = base;
  warm.x0 = solved.x;
  warm.constraint_dual_start = solved.constraint_duals;
  warm.box_dual_lb_start = solved.box_dual_lb;
  warm.box_dual_ub_start = solved.box_dual_ub;
  warm.solver_options.primal_dual_warm_start = true;
  const SolveResult repeated = ipopt.solve_nlp(warm);

  REQUIRE(repeated.stats.success);
  CHECK(repeated.stats.warm_start_used);
  CHECK(repeated.stats.iterations <= solved.stats.iterations);
  CHECK((repeated.x - solved.x).lpNorm<Eigen::Infinity>() <= 1e-7);
}

TEST_CASE("IpoptAdapter rejects incomplete primal-dual NLP starts",
          "[engine][api][ipopt][warm-start]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  NLPModel nlp = make_rosenbrock_nlp();
  nlp.solver_options.primal_dual_warm_start = true;
  nlp.constraint_dual_start = Eigen::VectorXd::Zero(1);
  const SolveResult result = ipopt.solve_nlp(nlp);

  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.status.find("Invalid NLP primal-dual warm start") !=
        std::string::npos);
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

  Eigen::MatrixXd rhs_many(0, 3), x_many;
  CHECK(solver.solve_many(rhs_many, x_many));
  CHECK(x_many.rows() == 0);
  CHECK(x_many.cols() == 3);
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

static void run_sparse_solve_many_3x3(SparseLinearSolver& solver) {
  Eigen::SparseMatrix<double> A(3, 3);
  A.insert(0, 0) = 4.0; A.insert(0, 1) = 1.0;
  A.insert(1, 0) = 1.0; A.insert(1, 1) = 3.0; A.insert(1, 2) = 1.0;
  A.insert(2, 1) = 1.0; A.insert(2, 2) = 2.0;
  A.makeCompressed();

  Eigen::MatrixXd expected(3, 3);
  expected << 1.0, -2.0, 0.5,
              2.0,  0.0, 1.5,
              2.5,  3.0, -1.0;
  const Eigen::MatrixXd rhs = A * expected;

  solver.analyze_pattern(A);
  REQUIRE(solver.factorize(A));
  Eigen::MatrixXd x;
  REQUIRE(solver.solve_many(rhs, x));
  REQUIRE(x.rows() == expected.rows());
  REQUIRE(x.cols() == expected.cols());
  CHECK((x - expected).cwiseAbs().maxCoeff() == Approx(0.0).margin(1e-10));

  Eigen::MatrixXd no_rhs(3, 0);
  REQUIRE(solver.solve_many(no_rhs, x));
  CHECK(x.rows() == 3);
  CHECK(x.cols() == 0);

  Eigen::MatrixXd wrong_rows = Eigen::MatrixXd::Ones(2, 2);
  CHECK_FALSE(solver.solve_many(wrong_rows, x));
}

TEST_CASE("SparseLinearSolver multi-RHS default preserves custom backends",
          "[engine][linear-solver][multi-rhs][fallback]") {
  class IdentitySolver final : public SparseLinearSolver {
   public:
    const char* backend_name() const override { return "test-identity"; }
    void analyze_pattern(const Eigen::SparseMatrix<double>& a) override {
      dimension = a.rows();
    }
    bool factorize(const Eigen::SparseMatrix<double>& a) override {
      dimension = a.rows();
      return a.rows() == a.cols();
    }
    bool solve(const Eigen::VectorXd& rhs, Eigen::VectorXd& x) override {
      if (rhs.size() != dimension) return false;
      ++solve_calls;
      x = rhs;
      return true;
    }

    Eigen::Index dimension{0};
    int solve_calls{0};
  } solver;

  Eigen::SparseMatrix<double> identity(2, 2);
  identity.setIdentity();
  REQUIRE(solver.factorize(identity));
  Eigen::MatrixXd rhs(2, 3);
  rhs << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0;
  Eigen::MatrixXd x;
  REQUIRE(solver.solve_many(rhs, x));
  CHECK(x == rhs);
  CHECK(solver.solve_calls == rhs.cols());
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

TEST_CASE("EigenSparseLU: multiple right-hand sides",
          "[engine][linear-solver][eigen][multi-rhs]") {
  EigenSparseLUSolver solver;
  run_sparse_solve_many_3x3(solver);
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

TEST_CASE("Default sparse solver: multiple right-hand sides",
          "[engine][linear-solver][default][multi-rhs]") {
  auto solver = make_default_sparse_solver();
  REQUIRE(solver != nullptr);
  INFO("Backend in use: " << solver->backend_name());
  run_sparse_solve_many_3x3(*solver);
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

#ifdef HACDCPF_HAVE_UMFPACK
TEST_CASE("EigenUmfPackSolver: multiple right-hand sides",
          "[engine][linear-solver][umfpack][multi-rhs]") {
  EigenUmfPackSolver solver;
  run_sparse_solve_many_3x3(solver);
}
#endif

#ifdef HACDCPF_HAVE_KLU
TEST_CASE("EigenKluSolver: multiple right-hand sides",
          "[engine][linear-solver][klu][multi-rhs]") {
  EigenKluSolver solver;
  run_sparse_solve_many_3x3(solver);
}
#endif

// ─── SuperLU (compiled in only when HACDCPF_HAVE_SUPERLU is set) ─────────────
#ifdef HACDCPF_HAVE_SUPERLU
TEST_CASE("SuperLUSolver: 3x3 sparse solve", "[engine][linear-solver][superlu]") {
  SuperLUSolver solver;
  run_sparse_solve_3x3(solver);
}

TEST_CASE("SuperLUSolver: multiple right-hand sides",
          "[engine][linear-solver][superlu][multi-rhs]") {
  SuperLUSolver solver;
  run_sparse_solve_many_3x3(solver);
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

TEST_CASE("MKLPardisoSolver: multiple right-hand sides",
          "[engine][linear-solver][pardiso][multi-rhs]") {
  MKLPardisoSolver solver;
  run_sparse_solve_many_3x3(solver);
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

TEST_CASE("MKLPardisoLLTSolver: SPD solve",
          "[engine][linear-solver][pardiso][llt]") {
  MKLPardisoLLTSolver solver;
  run_sparse_solve_3x3(solver);
}

TEST_CASE("MKLPardisoLLTSolver: multiple right-hand sides",
          "[engine][linear-solver][pardiso][llt][multi-rhs]") {
  MKLPardisoLLTSolver solver;
  run_sparse_solve_many_3x3(solver);
}

TEST_CASE("MKLPardisoLLTSolver: backend name is correct",
          "[engine][linear-solver][pardiso][llt]") {
  MKLPardisoLLTSolver solver;
  CHECK(std::string(solver.backend_name()) ==
        "Intel-MKL-PARDISO-LLT(Eigen)");
}

static void run_symmetric_indefinite_solve(SparseLinearSolver& solver,
                                            bool analyze_first) {
  // K = [ 2  1  0; 1 -2  1; 0  1  3 ] has both positive and negative
  // eigenvalues.  This exercises PARDISO's symmetric-indefinite mode rather
  // than merely testing LDLT on a positive-definite matrix.
  Eigen::SparseMatrix<double> k(3, 3);
  k.insert(0, 0) = 2.0;
  k.insert(0, 1) = 1.0;
  k.insert(1, 0) = 1.0;
  k.insert(1, 1) = -2.0;
  k.insert(1, 2) = 1.0;
  k.insert(2, 1) = 1.0;
  k.insert(2, 2) = 3.0;
  k.makeCompressed();
  Eigen::VectorXd expected(3);
  expected << 1.0, -2.0, 0.5;
  const Eigen::VectorXd rhs = k * expected;

  if (analyze_first) solver.analyze_pattern(k);
  REQUIRE(solver.factorize(k));
  Eigen::VectorXd x;
  REQUIRE(solver.solve(rhs, x));
  REQUIRE(x.size() == expected.size());
  CHECK((x - expected).lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-11));
}

TEST_CASE("MKLPardisoLDLTSolver: symmetric indefinite 3x3 solve",
          "[engine][linear-solver][pardiso][ldlt]") {
  MKLPardisoLDLTSolver solver;
  run_symmetric_indefinite_solve(solver, true);
}

TEST_CASE("MKLPardisoLDLTSolver: multiple right-hand sides",
          "[engine][linear-solver][pardiso][ldlt][multi-rhs]") {
  MKLPardisoLDLTSolver solver;
  run_sparse_solve_many_3x3(solver);
}

TEST_CASE("MKLPardisoLDLTSolver: factorize without prior analyze_pattern",
          "[engine][linear-solver][pardiso][ldlt]") {
  MKLPardisoLDLTSolver solver;
  run_symmetric_indefinite_solve(solver, false);
}

TEST_CASE("MKLPardisoLDLTSolver: empty square system",
          "[engine][linear-solver][pardiso][ldlt]") {
  MKLPardisoLDLTSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  solver.analyze_pattern(a);
  CHECK(solver.factorize(a));
  Eigen::VectorXd rhs(0), x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

TEST_CASE("MKLPardisoLDLTSolver: backend name is correct",
          "[engine][linear-solver][pardiso][ldlt]") {
  MKLPardisoLDLTSolver solver;
  CHECK(std::string(solver.backend_name()) ==
        "Intel-MKL-PARDISO-LDLT(Eigen)");
}

TEST_CASE("MKLPardisoAdaptiveSolver: symmetric indefinite 3x3 solve",
          "[engine][linear-solver][pardiso][adaptive]") {
  MKLPardisoAdaptiveSolver solver;
  run_symmetric_indefinite_solve(solver, true);
  CHECK(std::string(solver.backend_name()).find(
            "Intel-MKL-PARDISO-Adaptive[") == 0);
}

TEST_CASE("MKLPardisoAdaptiveSolver: factorize without analyze",
          "[engine][linear-solver][pardiso][adaptive]") {
  MKLPardisoAdaptiveSolver solver;
  run_symmetric_indefinite_solve(solver, false);
}

TEST_CASE("MKLPardisoAdaptiveSolver: empty square system",
          "[engine][linear-solver][pardiso][adaptive]") {
  MKLPardisoAdaptiveSolver solver;
  Eigen::SparseMatrix<double> a(0, 0);
  a.makeCompressed();
  CHECK(solver.factorize(a));
  Eigen::VectorXd rhs(0), x;
  CHECK(solver.solve(rhs, x));
  CHECK(x.size() == 0);
}

TEST_CASE("KKT explicit dual diagonal preserves the assembled equation",
          "[engine][linear-solver][pardiso][kkt]") {
  // [ 2  -3 ] [dx] = [  1]
  // [-3  -5 ] [dy]   [-11]
  // has the exact solution (2, 1).  The -5 dual entry represents a Schur
  // contribution supplied independently from the retained Jacobian pattern.
  Eigen::SparseMatrix<double> w(1, 1);
  w.insert(0, 0) = 2.0;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(1, 1);
  jg.insert(0, 0) = -3.0;
  jg.makeCompressed();
  Eigen::VectorXd dual_diagonal(1);
  dual_diagonal << -5.0;

  SparseKKTCache cache;
  cache.solver = std::make_unique<MKLPardisoSolver>();
  REQUIRE(factor_kkt_sparse_diagonal(cache, w, jg, 0.0,
                                     dual_diagonal));
  Eigen::VectorXd rhs(2);
  rhs << 1.0, -11.0;
  Eigen::VectorXd dx, dy;
  REQUIRE(solve_kkt_sparse(cache, rhs, dx, dy));
  REQUIRE(dx.size() == 1);
  REQUIRE(dy.size() == 1);
  CHECK(dx[0] == Approx(2.0).margin(1e-12));
  CHECK(dy[0] == Approx(1.0).margin(1e-12));
  CHECK((cache.kkt * (Eigen::VectorXd(2) << dx[0], dy[0]).finished() - rhs)
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-12));
}

TEST_CASE("KKT sparse dual Schur block preserves off-diagonal coupling",
          "[engine][linear-solver][pardiso][kkt]") {
  // The exact solution is (dx,dy1,dy2)=(1,2,-1).  The dual off-diagonal is
  // the fill edge produced when a degree-two primal column is condensed.
  Eigen::SparseMatrix<double> w(1, 1);
  w.insert(0, 0) = 2.0;
  w.makeCompressed();
  Eigen::SparseMatrix<double> jg(2, 1);
  jg.insert(0, 0) = -1.0;
  jg.insert(1, 0) = -2.0;
  jg.makeCompressed();
  Eigen::SparseMatrix<double> dual(2, 2);
  dual.insert(0, 0) = -3.0;
  dual.insert(1, 0) = -0.5;
  dual.insert(0, 1) = -0.5;
  dual.insert(1, 1) = -4.0;
  dual.makeCompressed();

  SparseKKTCache cache;
  cache.solver = std::make_unique<MKLPardisoSolver>();
  REQUIRE(factor_kkt_sparse_dual_block(cache, w, jg, 0.0, dual));
  Eigen::VectorXd rhs(3);
  rhs << 2.0, -6.5, 1.0;
  Eigen::VectorXd dx, dy;
  REQUIRE(solve_kkt_sparse(cache, rhs, dx, dy));
  REQUIRE(dx.size() == 1);
  REQUIRE(dy.size() == 2);
  CHECK(dx[0] == Approx(1.0).margin(1e-12));
  CHECK(dy[0] == Approx(2.0).margin(1e-12));
  CHECK(dy[1] == Approx(-1.0).margin(1e-12));
  Eigen::VectorXd solution(3);
  solution << dx[0], dy[0], dy[1];
  CHECK((cache.kkt * solution - rhs).lpNorm<Eigen::Infinity>() ==
        Approx(0.0).margin(1e-12));

  dual.valuePtr()[0] = -3.5;
  REQUIRE(factor_kkt_sparse_dual_block(cache, w, jg, 0.0, dual));
  CHECK(cache.symbolic_analyses == 1);
  CHECK(cache.numeric_factorizations == 2);
}
#endif  // HACDCPF_HAVE_MKL_PARDISO

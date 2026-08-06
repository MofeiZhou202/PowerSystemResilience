/// Numerical-stability regression matrix for the embedded Ipopt adapter.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"

using Catch::Approx;
using namespace mipsolvers::engine;

#ifdef HACDCPF_HAVE_IPOPT
namespace {

NLPModel make_well_conditioned_nlp() {
  NLPModel nlp;
  nlp.vars = {
      VariableMeta{VarType::Continuous, -10.0, 10.0},
      VariableMeta{VarType::Continuous, -10.0, 10.0},
  };
  nlp.x0 = Eigen::Vector2d(-4.0, 4.0);
  nlp.f = [](const Eigen::VectorXd& x) {
    return std::pow(x[0] - 1.0, 2) + std::pow(x[1] - 2.0, 2);
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    grad.resize(2);
    grad << 2.0 * (x[0] - 1.0), 2.0 * (x[1] - 2.0);
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x[0] + x[1] - 3.0;
  };
  nlp.jac_g = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 1.0;
    jac.insert(0, 1) = 1.0;
    jac.makeCompressed();
  };
  return nlp;
}

NLPModel make_rosenbrock_nlp() {
  NLPModel nlp;
  nlp.vars = {
      VariableMeta{VarType::Continuous, -5.0, 5.0},
      VariableMeta{VarType::Continuous, -5.0, 5.0},
  };
  nlp.x0 = Eigen::Vector2d(-1.2, 1.0);
  nlp.f = [](const Eigen::VectorXd& x) {
    const double a = x[1] - x[0] * x[0];
    const double b = 1.0 - x[0];
    return 100.0 * a * a + b * b;
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    const double a = x[1] - x[0] * x[0];
    grad.resize(2);
    grad[0] = -400.0 * x[0] * a - 2.0 * (1.0 - x[0]);
    grad[1] = 200.0 * a;
  };
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x[0] + x[1] - 2.0;
  };
  nlp.jac_g = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 1.0;
    jac.insert(0, 1) = 1.0;
    jac.makeCompressed();
  };
  return nlp;
}

NLPModel make_active_bound_nlp() {
  NLPModel nlp;
  nlp.vars = {VariableMeta{VarType::Continuous, 0.0, 10.0}};
  nlp.x0 = Eigen::VectorXd::Constant(1, 5.0);
  nlp.f = [](const Eigen::VectorXd& x) {
    return std::pow(x[0] + 2.0, 2);
  };
  nlp.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
    grad.resize(1);
    grad[0] = 2.0 * (x[0] + 2.0);
  };
  return nlp;
}

NLPModel make_scaled_nlp(double scale) {
  const double inverse_scale = 1.0 / scale;
  NLPModel nlp;
  nlp.vars = {
      VariableMeta{VarType::Continuous, -2.0 * scale, 2.0 * scale},
      VariableMeta{VarType::Continuous, -2.0 * inverse_scale,
                   2.0 * inverse_scale},
  };
  nlp.x0 = Eigen::Vector2d::Zero();
  nlp.f = [scale, inverse_scale](const Eigen::VectorXd& x) {
    const double a = x[0] * inverse_scale - 1.0;
    const double b = x[1] * scale - 1.0;
    return a * a + b * b;
  };
  nlp.grad = [scale, inverse_scale](const Eigen::VectorXd& x,
                                    Eigen::VectorXd& grad) {
    grad.resize(2);
    grad[0] = 2.0 * inverse_scale * (x[0] * inverse_scale - 1.0);
    grad[1] = 2.0 * scale * (x[1] * scale - 1.0);
  };
  nlp.g = [scale, inverse_scale](const Eigen::VectorXd& x,
                                 Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x[0] * inverse_scale + x[1] * scale - 2.0;
  };
  nlp.jac_g = [scale, inverse_scale](
                  const Eigen::VectorXd&, Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = inverse_scale;
    jac.insert(0, 1) = scale;
    jac.makeCompressed();
  };
  return nlp;
}

NLPModel make_nearly_dependent_nlp() {
  constexpr double epsilon = 1e-6;
  NLPModel nlp = make_well_conditioned_nlp();
  nlp.x0 = Eigen::Vector2d::Zero();
  nlp.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(2);
    g[0] = x[0] + x[1] - 2.0;
    g[1] = x[0] + (1.0 + epsilon) * x[1] - (2.0 + epsilon);
  };
  nlp.jac_g = [](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& jac) {
    jac.resize(2, 2);
    jac.insert(0, 0) = 1.0;
    jac.insert(0, 1) = 1.0;
    jac.insert(1, 0) = 1.0;
    jac.insert(1, 1) = 1.0 + epsilon;
    jac.makeCompressed();
  };
  return nlp;
}

void require_finite_stats(const SolveResult& result) {
  INFO("status=" << result.stats.status
       << " iterations=" << result.stats.iterations
       << " primal=" << result.stats.primal_feas
       << " dual=" << result.stats.dual_feas
       << " complementarity=" << result.stats.complementarity
       << " unscaled_primal=" << result.stats.unscaled_primal_feas
       << " unscaled_dual=" << result.stats.unscaled_dual_feas);
  CHECK(std::isfinite(result.stats.objective));
  CHECK(std::isfinite(result.stats.primal_feas));
  CHECK(std::isfinite(result.stats.dual_feas));
  CHECK(std::isfinite(result.stats.complementarity));
  CHECK(std::isfinite(result.stats.unscaled_primal_feas));
  CHECK(std::isfinite(result.stats.unscaled_dual_feas));
  CHECK(std::isfinite(result.stats.unscaled_complementarity));
  for (const double value : result.x) CHECK(std::isfinite(value));
}

SolveResult solve_with(IpoptAdapter& ipopt,
                       NLPModel nlp,
                       int max_iterations,
                       double tolerance,
                       double acceptable_tolerance) {
  nlp.solver_options.max_iterations = max_iterations;
  nlp.solver_options.tolerance = tolerance;
  nlp.solver_options.acceptable_tolerance = acceptable_tolerance;
  return ipopt.solve_nlp(nlp);
}

}  // namespace

TEST_CASE("Ipopt tolerance matrix remains finite and convergent",
          "[ipopt][stability][parameters]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  constexpr std::array<double, 4> strict_tolerances{
      1e-4, 1e-6, 1e-8, 1e-10};
  constexpr std::array<double, 3> acceptable_multipliers{1.0, 10.0, 1000.0};
  for (const double strict : strict_tolerances) {
    for (const double multiplier : acceptable_multipliers) {
      CAPTURE(strict, multiplier);
      const SolveResult result = solve_with(
          ipopt, make_well_conditioned_nlp(), 500, strict,
          strict * multiplier);
      require_finite_stats(result);
      REQUIRE(result.stats.success);
      REQUIRE(result.x.size() == 2);
      CHECK(result.x[0] == Approx(1.0).margin(2e-5));
      CHECK(result.x[1] == Approx(2.0).margin(2e-5));
      CHECK(result.stats.iterations < 500);
    }
  }
}

TEST_CASE("Ipopt iteration budgets are strict and monotone at endpoints",
          "[ipopt][stability][parameters]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  constexpr std::array<int, 5> budgets{1, 5, 20, 100, 500};
  std::vector<SolveResult> results;
  for (const int budget : budgets) {
    CAPTURE(budget);
    results.push_back(solve_with(
        ipopt, make_rosenbrock_nlp(), budget, 1e-9, 1e-7));
    require_finite_stats(results.back());
    CHECK(results.back().stats.iterations <= budget);
  }
  CHECK_FALSE(results.front().stats.success);
  CHECK(results.front().stats.status == "Max iterations exceeded");
  REQUIRE(results.back().stats.success);
  CHECK(results.back().stats.iterations < budgets.back());
  CHECK(results.back().stats.objective < results.front().stats.objective);
}

TEST_CASE("Ipopt normalizes malformed NLP options deterministically",
          "[ipopt][stability][parameters]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::array<double, 4> invalid_values{0.0, -1e-6, nan, inf};

  for (const double value : invalid_values) {
    CAPTURE(value);
    const SolveResult bad_strict = solve_with(
        ipopt, make_well_conditioned_nlp(), 100, value, 1e-6);
    require_finite_stats(bad_strict);
    CHECK(bad_strict.stats.success);
    CHECK(bad_strict.stats.status != "Ipopt invalid option");

    const SolveResult bad_acceptable = solve_with(
        ipopt, make_well_conditioned_nlp(), 100, 1e-8, value);
    require_finite_stats(bad_acceptable);
    CHECK(bad_acceptable.stats.success);
    CHECK(bad_acceptable.stats.status != "Ipopt invalid option");
  }

  const SolveResult inverted = solve_with(
      ipopt, make_well_conditioned_nlp(), 100, 1e-6, 1e-10);
  require_finite_stats(inverted);
  CHECK(inverted.stats.success);

  for (const int budget : {0, -50}) {
    CAPTURE(budget);
    const SolveResult result = solve_with(
        ipopt, make_rosenbrock_nlp(), budget, 1e-12, 1e-12);
    require_finite_stats(result);
    CHECK(result.stats.iterations <= 1);
    CHECK_FALSE(result.stats.success);
  }
}

TEST_CASE("Ipopt handles active bounds without synthetic Jacobian entries",
          "[ipopt][stability][bounds]") {
  IpoptAdapter ipopt;
  const SolveResult result = solve_with(
      ipopt, make_active_bound_nlp(), 200, 1e-9, 1e-7);
  require_finite_stats(result);
  REQUIRE(result.stats.success);
  REQUIRE(result.x.size() == 1);
  CHECK(result.x[0] == Approx(0.0).margin(1e-7));
}

TEST_CASE("Ipopt consumes an exact nonlinear Lagrangian Hessian",
          "[ipopt][stability][hessian]") {
  IpoptAdapter ipopt;
  REQUIRE(ipopt.available());

  auto hessian_calls = std::make_shared<int>(0);
  NLPModel nlp = make_well_conditioned_nlp();
  nlp.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    h.resize(1);
    h[0] = x.squaredNorm() - 10.0;
  };
  nlp.jac_h = [](const Eigen::VectorXd& x,
                 Eigen::SparseMatrix<double>& jac) {
    jac.resize(1, 2);
    jac.insert(0, 0) = 2.0 * x[0];
    jac.insert(0, 1) = 2.0 * x[1];
    jac.makeCompressed();
  };
  nlp.lagrangian_hess =
      [hessian_calls](const Eigen::VectorXd&,
                      const Eigen::VectorXd& lambda,
                      const Eigen::VectorXd* nu,
                      Eigen::SparseMatrix<double>& hessian) {
        REQUIRE(lambda.size() == 1);
        REQUIRE(nu != nullptr);
        REQUIRE(nu->size() == 1);
        ++*hessian_calls;
        const double diagonal = 2.0 + 2.0 * (*nu)[0];
        hessian.resize(2, 2);
        hessian.insert(0, 0) = diagonal;
        hessian.insert(1, 1) = diagonal;
        hessian.makeCompressed();
      };

  const SolveResult result = solve_with(ipopt, std::move(nlp), 200, 1e-9, 1e-7);
  require_finite_stats(result);
  REQUIRE(result.stats.success);
  CHECK(result.x[0] == Approx(1.0).margin(2e-5));
  CHECK(result.x[1] == Approx(2.0).margin(2e-5));
  // Two calls discover a fixed sparsity pattern. Any additional call proves
  // Ipopt requested numerical Hessian values during optimization.
  CHECK(*hessian_calls > 2);
}

TEST_CASE("Ipopt solves moderate scaling and reports extreme-scaling termination",
          "[ipopt][stability][scaling]") {
  IpoptAdapter ipopt;
  for (const double scale : {1.0, 10.0, 100.0, 1000.0}) {
    CAPTURE(scale);
    const SolveResult result = solve_with(
        ipopt, make_scaled_nlp(scale), 500, 1e-8, 1e-6);
    require_finite_stats(result);
    REQUIRE(result.stats.success);
    REQUIRE(result.x.size() == 2);
    CHECK(result.x[0] == Approx(scale).epsilon(2e-5));
    CHECK(result.x[1] == Approx(1.0 / scale).epsilon(2e-5));
    CHECK(result.stats.primal_feas < 1e-6);
  }

  NLPModel extreme_model = make_scaled_nlp(1e6);
  extreme_model.solver_options.dual_infeasibility_tolerance = 1e-8;
  extreme_model.solver_options.constraint_violation_tolerance = 1e-8;
  extreme_model.solver_options.complementarity_tolerance = 1e-8;
  extreme_model.solver_options.acceptable_dual_infeasibility_tolerance = 1e-6;
  extreme_model.solver_options.acceptable_constraint_violation_tolerance = 1e-6;
  extreme_model.solver_options.acceptable_complementarity_tolerance = 1e-6;
  const SolveResult extreme = solve_with(
      ipopt, std::move(extreme_model), 500, 1e-8, 1e-6);
  require_finite_stats(extreme);
  CHECK_FALSE(extreme.stats.success);
  CHECK(extreme.stats.status == "Ipopt search direction too small");
  CHECK(extreme.stats.unscaled_primal_feas <= 1e-8);
  CHECK(extreme.stats.unscaled_dual_feas <= 1e-8);
  CHECK(extreme.stats.unscaled_complementarity <= 1e-8);
}

TEST_CASE("Ipopt reports finite diagnostics for nearly dependent equalities",
          "[ipopt][stability][degeneracy]") {
  IpoptAdapter ipopt;
  const SolveResult result = solve_with(
      ipopt, make_nearly_dependent_nlp(), 500, 1e-8, 1e-6);
  require_finite_stats(result);
  REQUIRE(result.stats.success);
  REQUIRE(result.x.size() == 2);
  CHECK(result.x[0] == Approx(1.0).margin(2e-5));
  CHECK(result.x[1] == Approx(1.0).margin(2e-5));
  CHECK(result.stats.primal_feas < 1e-6);
}
#else
TEST_CASE("Ipopt stability matrix is skipped when Ipopt is unavailable",
          "[ipopt][stability]") {
  SUCCEED("HACDCPF_HAVE_IPOPT is not defined");
}
#endif

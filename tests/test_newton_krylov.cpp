#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <Eigen/Sparse>

#include "hacdcpf/power_flow/newton_krylov.hpp"
#include "hacdcpf/power_flow/solver_factory.hpp"
#include "hacdcpf/power_flow/workspace.hpp"
#include "power_flow_solver_test_utils.hpp"

TEST_CASE("Newton-Krylov GMRES solves a coupled AC-DC block system",
          "[power_flow][newton_krylov]") {
  Eigen::SparseMatrix<double> jacobian(3, 3);
  jacobian.insert(0, 0) = 4.0;
  jacobian.insert(0, 1) = 1.0;
  jacobian.insert(0, 2) = 0.5;
  jacobian.insert(1, 0) = 1.0;
  jacobian.insert(1, 1) = 3.0;
  jacobian.insert(1, 2) = 0.25;
  jacobian.insert(2, 2) = 2.0;
  jacobian.makeCompressed();
  const Eigen::Vector3d expected(1.0, -2.0, 0.5);
  const Eigen::VectorXd rhs = jacobian * expected;

  hacdcpf::powerflow::JacobianContext context;
  context.np = 1;
  context.nq = 1;
  context.ndc_eq = 1;
  context.nvar = 3;
  const auto result = hacdcpf::powerflow::newton_krylov_step(
      jacobian, rhs, context, true, 3, 5, 1e-12);
  REQUIRE(result.success);
  CHECK(result.used_schur);
  CHECK((result.step - expected).lpNorm<Eigen::Infinity>() < 1e-10);
}

TEST_CASE("Newton-Krylov Schur preconditioner reuses symbolic analysis",
          "[power_flow][newton_krylov][performance]") {
  Eigen::SparseMatrix<double> jacobian(3, 3);
  jacobian.insert(0, 0) = 4.0;
  jacobian.insert(0, 1) = 1.0;
  jacobian.insert(0, 2) = 0.5;
  jacobian.insert(1, 0) = 1.0;
  jacobian.insert(1, 1) = 3.0;
  jacobian.insert(1, 2) = 0.25;
  jacobian.insert(2, 2) = 2.0;
  jacobian.makeCompressed();

  hacdcpf::powerflow::JacobianContext context;
  context.np = 1;
  context.nq = 1;
  context.ndc_eq = 1;
  hacdcpf::powerflow::SchurBlockPreconditioner preconditioner;
  REQUIRE(preconditioner.build(jacobian, context));
  CHECK(preconditioner.symbolic_analysis_count() == 2);

  for (int k = 0; k < jacobian.nonZeros(); ++k) {
    jacobian.valuePtr()[k] *= 1.01;
  }
  REQUIRE(preconditioner.build(jacobian, context));
  CHECK(preconditioner.symbolic_analysis_count() == 2);
}

TEST_CASE("GMRES never accepts a small preconditioned residual as a true solution",
          "[power_flow][newton_krylov][math_audit][B7]") {
  const double scale = 1e6;
  const Eigen::Vector2d rhs(1.0, -2.0);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
  const auto stats = hacdcpf::powerflow::gmres_solve(
      [scale](const Eigen::VectorXd& value) { return scale * value; },
      [scale](const Eigen::VectorXd& value) { return value / scale; },
      rhs, x, 2, 2, 1e-4);

  REQUIRE(stats.converged);
  CHECK((rhs - scale * x).norm() / rhs.norm() < 1e-12);
  CHECK(stats.final_relres < 1e-12);
}

TEST_CASE("Unified solver factory returns linked, callable solvers",
          "[power_flow][solver_factory]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto problem = hacdcpf::build_power_flow_problem(sys);

  for (const auto method : {hacdcpf::PowerFlowMethod::Newton,
                            hacdcpf::PowerFlowMethod::FDPF,
                            hacdcpf::PowerFlowMethod::DC,
                            hacdcpf::PowerFlowMethod::Adaptive,
                            hacdcpf::PowerFlowMethod::Continuation,
                            hacdcpf::PowerFlowMethod::NewtonKrylov}) {
    std::unique_ptr<hacdcpf::IPowerFlowSolver> solver =
        hacdcpf::PowerFlowSolverFactory::create(method);
    REQUIRE(solver != nullptr);
    const auto result = solver->solve(problem);
    CHECK(result.converged);
  }
}

TEST_CASE("SolverWorkspace preparation clears retained values",
          "[power_flow][workspace]") {
  hacdcpf::powerflow::SolverWorkspace workspace;
  workspace.prepare_state(4, 2);
  workspace.prepare_equations(3, 2, 1);
  workspace.vm.setConstant(0.7);
  workspace.va.setOnes();
  workspace.vdc.setConstant(0.8);
  workspace.residual.setOnes();
  workspace.dx.setOnes();
  workspace.prepare_state(4, 2);
  workspace.prepare_equations(3, 2, 1);

  CHECK(workspace.nac == 4);
  CHECK(workspace.ndc == 2);
  CHECK(workspace.equation_count() == 6);
  CHECK(workspace.vm.isOnes());
  CHECK(workspace.va.isZero());
  CHECK(workspace.vdc.isOnes());
  CHECK(workspace.residual.isZero());
  CHECK(workspace.dx.isZero());
}

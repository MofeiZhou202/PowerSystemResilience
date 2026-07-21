#include "hacdcpf/power_flow/solver_factory.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "hacdcpf/power_flow/adaptive_solver.hpp"
#include "hacdcpf/power_flow/dc_solver.hpp"
#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/voltage_stability.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf {
namespace {

PowerFlowResult solve_newton_problem(const PowerFlowProblem& problem,
                                     PowerFlowOptions options) {
  powerflow::NewtonSolver solver;
  const InitialState* initial = options.initial_state ? &*options.initial_state : nullptr;
  return solver.solve(problem.network, options, initial);
}

}  // namespace

PowerFlowProblem build_power_flow_problem(const HybridPowerSystem& sys,
                                           const PowerFlowOptions& options) {
  PowerFlowProblem problem;
  problem.network = powerflow::make_solver_data(sys, options.loss_model);
  problem.options = options;
  problem.name = sys.name;
  problem.n_ac_buses = static_cast<int>(problem.network.ac_buses.size());
  problem.n_dc_buses = static_cast<int>(problem.network.dc_buses.size());
  problem.n_converters = static_cast<int>(problem.network.converters.size());
  return problem;
}

PowerFlowResult NewtonHybridSolver::solve(const PowerFlowProblem& problem) {
  return solve_newton_problem(problem, problem.options);
}

PowerFlowResult FDPFSolver::solve(const PowerFlowProblem& problem) {
  powerflow::FDPFSolver solver;
  return solver.solve(problem.network, problem.options);
}

PowerFlowResult DCPowerFlowSolver::solve(const PowerFlowProblem& problem) {
  powerflow::DCSolver solver;
  const InitialState* initial =
      problem.options.initial_state ? &*problem.options.initial_state : nullptr;
  const DCPowerFlowResult dc = solver.solve(problem.network, problem.options, initial);

  PowerFlowResult result;
  result.converged = dc.converged;
  result.iterations = dc.iterations;
  result.residual = dc.residual;
  result.vdc = dc.vdc;
  result.vm.reserve(problem.network.ac_buses.size());
  result.va.reserve(problem.network.ac_buses.size());
  for (const auto& bus : problem.network.ac_buses) {
    result.vm.push_back(bus.vm_pu);
    result.va.push_back(bus.va_deg * 3.14159265358979323846 / 180.0);
  }
  result.diagnostics.converged = result.converged;
  result.diagnostics.iterations = result.iterations;
  result.diagnostics.final_mismatch_norm = result.residual;
  result.diagnostics.termination_reason =
      result.converged ? "DC power flow converged" : "DC power flow failed";
  return result;
}

PowerFlowResult AdaptiveHybridSolver::solve(const PowerFlowProblem& problem) {
  PowerFlowResult result = solve_newton_problem(problem, problem.options);
  result.diagnostics.warnings.push_back(
      "Adaptive solver received an already canonical PowerFlowProblem; "
      "island-aware projection is unavailable, so the canonical Newton solver was used.");
  return result;
}

PowerFlowResult ContinuationPowerFlowSolver::solve(const PowerFlowProblem& problem) {
  powerflow::CpfSolver solver;
  solver.opts.trace_all_buses = true;
  const powerflow::CpfResult cpf = solver.solve(
      problem.network, powerflow::CpfDirection::proportional(problem.network));

  PowerFlowResult result;
  result.converged = !cpf.trace.empty();
  result.iterations = cpf.total_pf_solves;
  result.diagnostics.converged = result.converged;
  result.diagnostics.iterations = result.iterations;
  result.diagnostics.termination_reason = cpf.termination_reason;
  if (!cpf.trace.empty()) {
    const auto nose = std::max_element(
        cpf.trace.begin(), cpf.trace.end(),
        [](const powerflow::CpfPoint& a, const powerflow::CpfPoint& b) {
          return a.lambda < b.lambda;
        });
    result.vm = nose->vm;
    result.va = nose->va;
    result.vdc = nose->vdc;
  }
  return result;
}

PowerFlowResult NewtonKrylovSolver::solve(const PowerFlowProblem& problem) {
  PowerFlowOptions options = problem.options;
  options.robust_nonlinear.enable_newton_krylov_fallback = true;
  options.robust_nonlinear.nk_condition_trigger = 0.0;
  return solve_newton_problem(problem, std::move(options));
}

std::unique_ptr<IPowerFlowSolver> PowerFlowSolverFactory::create(
    PowerFlowMethod method) {
  switch (method) {
    case PowerFlowMethod::FDPF:
      return std::make_unique<FDPFSolver>();
    case PowerFlowMethod::DC:
      return std::make_unique<DCPowerFlowSolver>();
    case PowerFlowMethod::Adaptive:
      return std::make_unique<AdaptiveHybridSolver>();
    case PowerFlowMethod::Continuation:
      return std::make_unique<ContinuationPowerFlowSolver>();
    case PowerFlowMethod::NewtonKrylov:
      return std::make_unique<NewtonKrylovSolver>();
    case PowerFlowMethod::Newton:
    default:
      return std::make_unique<NewtonHybridSolver>();
  }
}

std::unique_ptr<IPowerFlowSolver> PowerFlowSolverFactory::create(
    const PowerFlowOptions&) {
  return create(PowerFlowMethod::Newton);
}

}  // namespace hacdcpf

#pragma once

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/kernel/ipm/conic_ipm_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

struct NativeNLEOptions {
  int max_iter{80};
  double tol{1e-8};
  double regularization0{1e-10};
  double step_backoff{0.5};
  int max_line_search_steps{20};
};

struct NativeNLPOptions {
  int max_iter{120};
  double tol_grad{1e-6};
  double tol_step{1e-8};
  double penalty_rho{100.0};
  double regularization0{1e-10};
  double step_backoff{0.5};
  int max_line_search_steps{20};
};

class NativeLinearAdapter final : public SolverAdapter {
 public:
  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_le(const SparseLinSys& prob) const override;
};

class NativeNewtonAdapter final : public SolverAdapter {
 public:
  explicit NativeNewtonAdapter(NativeNLEOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_nle(const NonlinearSystem& prob) const override;

 private:
  NativeNLEOptions opt_;
};

class NativeNLPAdapter final : public SolverAdapter {
 public:
  explicit NativeNLPAdapter(NativeNLPOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_nlp(const NLPModel& prob) const override;

 private:
  NativeNLPOptions opt_;
};

/// Native conic (LP/SOCP/SDP) interior-point solver adapter, cvxopt form.
class NativeConicIPMAdapter final : public SolverAdapter {
 public:
  explicit NativeConicIPMAdapter(ConicIPMOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_conic(const ConicModel& prob) const override;

 private:
  ConicIPMOptions opt_;
};

BCOptions make_strict_highs_production_options(BCOptions opt = {});
BCOptions make_strict_highs_problem_options(const MIPModel& prob,
                                            BCOptions opt = {});

class StrictHighsBranchAndCutAdapter final : public SolverAdapter {
 public:
  explicit StrictHighsBranchAndCutAdapter(BCOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_milp(const MIPModel& prob) const override;

 private:
  BCOptions opt_;
};

class NativeBranchAndCutAdapter final : public SolverAdapter {
 public:
  explicit NativeBranchAndCutAdapter(BCOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_milp(const MIPModel& prob) const override;
  SolveResult solve_minlp(const MINLPModel& prob) const override;

 private:
  BCOptions opt_;
};

}  // namespace mipsolvers::engine

#pragma once

#include <atomic>
#include <string>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// LP adapter wrapping the native dual-simplex kernel with full-exact dual
/// steepest-edge weights (the best-geomean NETLIB configuration measured on
/// 2026-08-11: Native-DualSimplex[ExactDSE](+HiGHS-presolve)).
class NativeDualSimplexLPAdapter final : public SolverAdapter {
 public:
  explicit NativeDualSimplexLPAdapter(
      double time_limit_sec = 0.0,
      const std::atomic<bool>* cancel_flag = nullptr);

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;

 private:
  double time_limit_sec_{0.0};
  const std::atomic<bool>* cancel_flag_{nullptr};
};

/// Concurrent LP portfolio (the default native LP path).
///
/// Races the dual-simplex-DSE kernel against the native IPM and returns the
/// first successful result. No native LP kernel dominates the NETLIB set
/// (dual simplex wins small/medium, IPM wins large/dense/wide) and the winner
/// is not predictable from sparsity structure alone, so the race realizes the
/// per-instance min(T_DSE, T_IPM) at ~2x CPU. Both branches are audited
/// kernels, so either result is correct. See
/// docs/lp_kernel_selector_2026-08-11.md for the design and the measured
/// alternatives (a cheap structural selector could not beat HiGHS).
class NativeAutoLPAdapter final : public SolverAdapter {
 public:
  explicit NativeAutoLPAdapter(double time_limit_sec = 0.0);

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;

 private:
  double time_limit_sec_{0.0};
};

}  // namespace mipsolvers::engine

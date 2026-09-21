// ═══════════════════════════════════════════════════════════════════════════
// NativeConicIPMAdapter — engine adapter around ConicIPMSolver.
//
// Runs the native conic IPM and maps ConicIPMResult onto SolveResult:
// constraint_duals = [z (m conic rows) | y (m_eq equality rows)].
// Non-optimal statuses return success = false with the best iterate kept.
// ═══════════════════════════════════════════════════════════════════════════

#include <chrono>

#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {

NativeConicIPMAdapter::NativeConicIPMAdapter(ConicIPMOptions opt)
    : opt_(opt) {}

std::string NativeConicIPMAdapter::name() const {
  return "NativeConicIPM";
}

bool NativeConicIPMAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::CONIC;
}

SolveResult NativeConicIPMAdapter::solve_conic(const ConicModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid conic model"
                                         : vr.errors.front();
    return out;
  }

  const ConicIPMSolver solver(opt_);
  const ConicIPMResult res = solver.solve(prob);

  out.x = res.x;
  out.stats.success = (res.status == "optimal");
  out.stats.status = res.status;
  out.stats.objective = res.primal_objective;  // already sense-adjusted
  out.stats.iterations = res.iterations;
  out.stats.primal_feas = res.primal_infeasibility;
  out.stats.dual_feas = res.dual_infeasibility;
  out.stats.complementarity = res.gap;
  out.stats.residual_inf =
      std::max(res.primal_infeasibility, res.dual_infeasibility);

  // Constraint duals: [z (m conic rows) | y (m_eq equality rows)].
  const int m = prob.dims.total();
  const int meq = static_cast<int>(prob.A.rows());
  out.constraint_duals.resize(m + meq);
  if (res.z.size() == m) {
    out.constraint_duals.head(m) = res.z;
  } else {
    out.constraint_duals.head(m).setZero();
  }
  if (meq > 0 && res.y.size() == meq) {
    out.constraint_duals.tail(meq) = res.y;
  } else if (meq > 0) {
    out.constraint_duals.tail(meq).setZero();
  }

  const auto t1 = std::chrono::steady_clock::now();
  out.stats.runtime_sec = std::chrono::duration<double>(t1 - t0).count();
  return out;
}

}  // namespace mipsolvers::engine

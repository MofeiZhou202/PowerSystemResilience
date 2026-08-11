#include "mipsolvers/engine/solver/native/native_lp_selector.hpp"

#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine {
namespace {

// Shared rendezvous for the concurrent LP portfolio. Held via shared_ptr by the
// caller and both worker threads so a detached loser can safely publish its
// result even after the caller has already returned the winner.
struct PortfolioSlot {
  std::mutex m;
  std::condition_variable cv;
  int finished{0};
  bool have_winner{false};
  SolveResult winner;
};

void publish(const std::shared_ptr<PortfolioSlot>& slot, SolveResult r) {
  std::lock_guard<std::mutex> lk(slot->m);
  ++slot->finished;
  // Take the first successful result; if both kernels fail, take the last.
  if (!slot->have_winner && (r.stats.success || slot->finished == 2)) {
    slot->winner = std::move(r);
    slot->have_winner = true;
  }
  slot->cv.notify_all();
}

}  // namespace

NativeDualSimplexLPAdapter::NativeDualSimplexLPAdapter(double time_limit_sec)
    : time_limit_sec_(time_limit_sec) {}

std::string NativeDualSimplexLPAdapter::name() const {
  return "NativeDualSimplex";
}

bool NativeDualSimplexLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeDualSimplexLPAdapter::solve_lp(const LPModel& prob) const {
  SimplexOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.dual_edge_weight_initialization =
      DualEdgeWeightInitialization::FullExact;  // ExactDSE
  opt.use_highs_presolve = true;  // best-geomean NETLIB configuration
  opt.time_limit_sec = time_limit_sec_;
  SolveResult r = solve_lp_with_basis(prob, opt).result;
  // solve_lp_with_basis reports stats.objective in the internal minimize
  // convention (objective_const - max_objective). The public adapter contract,
  // matching the HiGHS/IPM adapters, is the user objective c·x — independent of
  // Sense. Recompute it from the returned original-space solution.
  if (r.stats.success && r.x.size() == prob.c.size()) {
    r.stats.objective = prob.c.dot(r.x);
  }
  return r;
}

NativeAutoLPAdapter::NativeAutoLPAdapter(double time_limit_sec)
    : time_limit_sec_(time_limit_sec) {}

std::string NativeAutoLPAdapter::name() const { return "NativeAutoLP"; }

bool NativeAutoLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeAutoLPAdapter::solve_lp(const LPModel& prob) const {
  // Concurrent portfolio: race the dual-simplex-DSE kernel against the IPM and
  // return the first successful result. No native LP kernel dominates the
  // NETLIB set (dual simplex wins small/medium, IPM wins large/dense/wide) and
  // the winner is not predictable from sparsity structure alone, so the race
  // realizes the per-instance min(T_DSE, T_IPM). Both kernels are audited, so
  // either result is correct. See docs/lp_kernel_selector_2026-08-11.md.
  const double tl = time_limit_sec_;
  auto slot = std::make_shared<PortfolioSlot>();

  auto dse_worker = [slot, lp = prob, tl]() {
    SolveResult r;
    try {
      r = NativeDualSimplexLPAdapter(tl).solve_lp(lp);
    } catch (...) {
      r.stats.success = false;
      r.stats.status = "NativeDualSimplex exception";
    }
    publish(slot, std::move(r));
  };
  auto ipm_worker = [slot, lp = prob, tl]() {
    SolveResult r;
    try {
      IPMLPOptions opt;  // defaults == native-ipm-direct
      opt.time_limit_sec = tl;
      r = NativeIPMLPAdapter(opt).solve_lp(lp);
    } catch (...) {
      r.stats.success = false;
      r.stats.status = "NativeIPMLP exception";
    }
    publish(slot, std::move(r));
  };

  try {
    std::thread(dse_worker).detach();
    std::thread(ipm_worker).detach();
  } catch (const std::system_error&) {
    // Thread creation failed: fall back to a synchronous IPM solve (the prior
    // default LP path). Any worker already launched publishes harmlessly.
    IPMLPOptions opt;
    opt.time_limit_sec = tl;
    return NativeIPMLPAdapter(opt).solve_lp(prob);
  }

  std::unique_lock<std::mutex> lk(slot->m);
  slot->cv.wait(lk, [&] { return slot->have_winner; });
  SolveResult out = slot->winner;
  lk.unlock();

  if (const char* dbg = std::getenv("MIPSOLVERS_LP_SELECTOR_DEBUG");
      dbg && *dbg) {
    std::fprintf(stderr, "LP-PORTFOLIO winner=%s\n",
                 out.stats.solver_name.c_str());
  }
  return out;
}

}  // namespace mipsolvers::engine

#include "mipsolvers/engine/solver/native/native_lp_selector.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine {
namespace {

struct PortfolioSlot {
  std::mutex m;
  std::condition_variable cv;
  std::atomic<bool> cancel{false};
  int finished{0};
  bool have_winner{false};
  SolveResult winner;
};

void publish(const std::shared_ptr<PortfolioSlot>& slot, SolveResult r) {
  std::lock_guard<std::mutex> lk(slot->m);
  ++slot->finished;
  if (!slot->have_winner && (r.stats.success || slot->finished == 2)) {
    slot->winner = std::move(r);
    slot->have_winner = true;
    slot->cancel.store(true, std::memory_order_relaxed);
  }
  slot->cv.notify_all();
}

}  // namespace

NativeDualSimplexLPAdapter::NativeDualSimplexLPAdapter(
    double time_limit_sec, const std::atomic<bool>* cancel_flag)
    : time_limit_sec_(time_limit_sec), cancel_flag_(cancel_flag) {}

std::string NativeDualSimplexLPAdapter::name() const {
  return "NativeDualSimplex";
}

bool NativeDualSimplexLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeDualSimplexLPAdapter::solve_lp(const LPModel& prob) const {
  SimplexOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.dual_edge_weight_initialization = DualEdgeWeightInitialization::FullExact;
  opt.use_highs_presolve = true;
  opt.time_limit_sec = time_limit_sec_;
  opt.cancel_flag = cancel_flag_;
  SolveResult r = solve_lp_with_basis(prob, opt).result;
  // Public objective is c*x in the user's sense, unlike the internal minimum.
  if (r.stats.success && r.x.size() == prob.c.size())
    r.stats.objective = prob.c.dot(r.x);
  return r;
}

NativeAutoLPAdapter::NativeAutoLPAdapter(double time_limit_sec)
    : time_limit_sec_(time_limit_sec) {}

std::string NativeAutoLPAdapter::name() const { return "NativeAutoLP"; }

bool NativeAutoLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeAutoLPAdapter::solve_lp(const LPModel& prob) const {
  // R4 in docs/archive/windows_remediation_2026-09-11.md:
  // race against the reliable direct IPM path, with one call-wide deadline.
  // Cancellation is cooperative; an active library factorization must finish.
  const auto start = std::chrono::steady_clock::now();
  const bool limited = time_limit_sec_ > 0.0 && std::isfinite(time_limit_sec_);
  auto remaining = [&]() {
    return limited ? std::max(0.0, time_limit_sec_ -
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count())
        : 0.0;
  };
  auto timeout = [&]() {
    SolveResult r;
    r.stats.solver_name = name();
    r.stats.status = "Time limit";
    return r;
  };
  auto slot = std::make_shared<PortfolioSlot>();
  auto worker = [&](bool ipm) {
    SolveResult r;
    try {
      const double budget = remaining();
      if (limited && budget <= 0.0) {
        r = timeout();
      } else if (ipm) {
        IPMLPOptions opt;
        opt.presolve = false;
        opt.time_limit_sec = budget;
        opt.cancel_flag = &slot->cancel;
        r = NativeIPMLPAdapter(opt).solve_lp(prob);
      } else {
        r = NativeDualSimplexLPAdapter(budget, &slot->cancel).solve_lp(prob);
      }
    } catch (...) {
      r.stats.status = ipm ? "NativeIPMLP exception" : "NativeDualSimplex exception";
    }
    publish(slot, std::move(r));
  };
  std::vector<std::thread> workers;
  workers.reserve(2);
  SolveResult out;
  try {
    workers.emplace_back(worker, false);
    workers.emplace_back(worker, true);
  } catch (const std::system_error&) {
    slot->cancel.store(true, std::memory_order_relaxed);
    for (auto& thread : workers) thread.join();
    const double budget = remaining();
    if (limited && budget <= 0.0) return timeout();
    IPMLPOptions opt;
    opt.presolve = false;
    opt.time_limit_sec = budget;
    return NativeIPMLPAdapter(opt).solve_lp(prob);
  }
  {
    std::unique_lock<std::mutex> lock(slot->m);
    bool ready = true;
    if (limited) {
      ready = slot->cv.wait_until(lock,
          start + std::chrono::duration<double>(time_limit_sec_),
          [&] { return slot->have_winner; });
    } else {
      slot->cv.wait(lock, [&] { return slot->have_winner; });
    }
    out = ready ? slot->winner : timeout();
    slot->cancel.store(true, std::memory_order_relaxed);
  }
  // Both workers borrow this call's model and deadline; always join before return.
  for (auto& thread : workers) thread.join();
  out.stats.runtime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  if (const char* debug = std::getenv("MIPSOLVERS_LP_SELECTOR_DEBUG"); debug && *debug)
    std::fprintf(stderr, "LP-PORTFOLIO direct-ipm winner=%s\n", out.stats.solver_name.c_str());
  return out;
}

}  // namespace mipsolvers::engine

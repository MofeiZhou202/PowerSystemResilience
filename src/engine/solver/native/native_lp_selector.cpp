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
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
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

SolveResult NativeDualSimplexLPAdapter::solve_lp(
    const LPModel& prob, const SolveContext& context) const {
  if (context.stop_requested()) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context.deadline_expired() ? "Time limit" : "Cancelled";
    return out;
  }
  double limit = time_limit_sec_;
  if (context.has_deadline()) {
    const double remaining = context.backend_time_limit_sec(0.0);
    limit = limit > 0.0 ? std::min(limit, remaining) : remaining;
  }
  std::atomic<bool> cancelled{false};
  std::stop_callback callback(context.stop_token(), [&cancelled] {
    cancelled.store(true, std::memory_order_relaxed);
  });
  const ScopedMklThreadLimit thread_limit(
      context.has_explicit_thread_budget() ? context.thread_budget() : 0);
  return NativeDualSimplexLPAdapter(limit, &cancelled).solve_lp(prob);
}

NativeAutoLPAdapter::NativeAutoLPAdapter(double time_limit_sec)
    : time_limit_sec_(time_limit_sec) {}

std::string NativeAutoLPAdapter::name() const { return "NativeAutoLP"; }

bool NativeAutoLPAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::LP;
}

SolveResult NativeAutoLPAdapter::solve_lp(const LPModel& prob) const {
  return solve_with_context(prob, nullptr);
}

SolveResult NativeAutoLPAdapter::solve_lp(const LPModel& prob,
                                          const SolveContext& context) const {
  return solve_with_context(prob, &context);
}

SolveResult NativeAutoLPAdapter::solve_with_context(
    const LPModel& prob, const SolveContext* context) const {
  // R4 in docs/archive/windows_remediation_2026-09-11.md:
  // race against the reliable direct IPM path, with one call-wide deadline.
  // Cancellation is cooperative; an active library factorization must finish.
  const auto start = std::chrono::steady_clock::now();
  if (context && context->stop_requested()) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context->deadline_expired() ? "Time limit" : "Cancelled";
    return out;
  }
  double effective_limit = time_limit_sec_;
  if (context && context->has_deadline()) {
    const double remaining = context->backend_time_limit_sec(0.0);
    effective_limit = effective_limit > 0.0
                          ? std::min(effective_limit, remaining)
                          : remaining;
  }
  const bool limited = effective_limit > 0.0 && std::isfinite(effective_limit);
  auto remaining = [&]() {
    return limited ? std::max(0.0, effective_limit -
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
  std::stop_callback stop_callback(
      context ? context->stop_token() : std::stop_token{}, [slot] {
        slot->cancel.store(true, std::memory_order_relaxed);
        slot->cv.notify_all();
      });

  // Throughput mode and a one-thread budget use the reliable direct IPM path.
  // The latency race is admitted only when the call can pay for two workers.
  // See docs/archive/general_solver_performance_program_2026-09-13.md, R2.
  const bool single_worker = context &&
      (context->portfolio_mode() == PortfolioMode::Throughput ||
       context->thread_budget() == 1);
  if (single_worker) {
    IPMLPOptions opt;
    opt.presolve = false;
    opt.time_limit_sec = remaining();
    opt.cancel_flag = &slot->cancel;
    const ScopedMklThreadLimit thread_limit(context->thread_budget());
    SolveResult out = NativeIPMLPAdapter(opt).solve_lp(prob);
    out.stats.portfolio_workers = 1;
    out.stats.worker_thread_limit = context->thread_budget();
    out.stats.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    return out;
  }

  auto worker = [&](bool ipm) {
    const int worker_threads = context && context->thread_budget() > 0
                                   ? std::max(1, context->thread_budget() / 2)
                                   : 0;
    const ScopedMklThreadLimit thread_limit(worker_threads);
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
    const ScopedMklThreadLimit thread_limit(
        context ? context->thread_budget() : 0);
    SolveResult fallback = NativeIPMLPAdapter(opt).solve_lp(prob);
    fallback.stats.portfolio_workers = 1;
    fallback.stats.worker_thread_limit = context ? context->thread_budget() : 0;
    return fallback;
  }
  {
    std::unique_lock<std::mutex> lock(slot->m);
    bool ready = true;
    if (limited) {
      ready = slot->cv.wait_until(lock,
          start + std::chrono::duration<double>(effective_limit),
          [&] { return slot->have_winner || slot->cancel.load(std::memory_order_relaxed); });
    } else {
      slot->cv.wait(lock, [&] {
        return slot->have_winner || slot->cancel.load(std::memory_order_relaxed);
      });
    }
    if (ready && slot->have_winner) {
      out = slot->winner;
    } else {
      out = timeout();
      if (context && context->stop_token().stop_requested() &&
          !context->deadline_expired()) {
        out.stats.status = "Cancelled";
      }
    }
    slot->cancel.store(true, std::memory_order_relaxed);
  }
  // Both workers borrow this call's model and deadline; always join before return.
  for (auto& thread : workers) thread.join();
  out.stats.portfolio_workers = 2;
  out.stats.worker_thread_limit = context && context->thread_budget() > 0
                                      ? std::max(1, context->thread_budget() / 2)
                                      : 0;
  out.stats.runtime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  if (const char* debug = std::getenv("MIPSOLVERS_LP_SELECTOR_DEBUG"); debug && *debug)
    std::fprintf(stderr, "LP-PORTFOLIO direct-ipm winner=%s\n", out.stats.solver_name.c_str());
  return out;
}

}  // namespace mipsolvers::engine

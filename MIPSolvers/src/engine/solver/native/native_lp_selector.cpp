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
    double time_limit_sec, const std::atomic<bool>* cancel_flag,
    int kernel_threads)
    : time_limit_sec_(time_limit_sec),
      cancel_flag_(cancel_flag),
      kernel_threads_(kernel_threads) {}

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
  opt.lp_kernel_threads = kernel_threads_;
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
  return NativeDualSimplexLPAdapter(
             limit, &cancelled,
             context.has_explicit_thread_budget() ? context.thread_budget() : 0)
      .solve_lp(prob);
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
  // docs/archive/lp_kernel_selector_2026-08-11.md: race the two validated kernels for
  // latency. Cancellation is cooperative, so every worker is joined before
  // the call returns and noninterruptible factorization delay is measured.
  const auto start = std::chrono::steady_clock::now();
  if (context != nullptr && context->stop_requested()) {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context->deadline_expired() ? "Time limit" : "Cancelled";
    return out;
  }
  double limit = time_limit_sec_;
  if (context != nullptr && context->has_deadline()) {
    const double remaining = context->backend_time_limit_sec(0.0);
    limit = limit > 0.0 ? std::min(limit, remaining) : remaining;
  }
  const bool limited = limit > 0.0 && std::isfinite(limit);
  auto remaining = [&]() {
    if (!limited) return 0.0;
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    return std::max(0.0, limit - elapsed);
  };
  auto stopped_result = [&]() {
    SolveResult out;
    out.stats.solver_name = name();
    out.stats.status = context != nullptr &&
                               context->stop_token().stop_requested() &&
                               !context->deadline_expired()
                           ? "Cancelled"
                           : "Time limit";
    return out;
  };

  auto slot = std::make_shared<PortfolioSlot>();
  std::stop_callback stop_callback(
      context != nullptr ? context->stop_token() : std::stop_token{}, [slot] {
        slot->cancel.store(true, std::memory_order_relaxed);
        slot->cv.notify_all();
      });

  const bool single_worker =
      context != nullptr &&
      (context->portfolio_mode() == PortfolioMode::Throughput ||
       context->thread_budget() == 1);
  if (single_worker) {
    IPMLPOptions opt;
    // Windows direct-IPM policy: docs/archive/windows_remediation_2026-09-11.md, R4.
    opt.presolve = false;
    opt.time_limit_sec = remaining();
    opt.cancel_flag = &slot->cancel;
    const ScopedMklThreadLimit thread_limit(context->thread_budget());
    SolveResult out = NativeIPMLPAdapter(opt).solve_lp(prob);
    out.stats.portfolio_workers = 1;
    out.stats.worker_thread_limit = context->thread_budget();
    out.stats.portfolio_first_result_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    out.stats.runtime_sec = out.stats.portfolio_first_result_sec;
    return out;
  }

  auto worker = [&](bool use_ipm) {
    const int worker_threads = context != nullptr &&
                                       context->thread_budget() > 0
                                   ? std::max(1, context->thread_budget() / 2)
                                   : 0;
    const ScopedMklThreadLimit thread_limit(worker_threads);
    SolveResult r;
    try {
      const double budget = remaining();
      if (limited && budget <= 0.0) {
        r = stopped_result();
      } else if (use_ipm) {
        IPMLPOptions opt;
        // Windows direct-IPM policy: docs/archive/windows_remediation_2026-09-11.md, R4.
        opt.presolve = false;
        opt.time_limit_sec = budget;
        opt.cancel_flag = &slot->cancel;
        r = NativeIPMLPAdapter(opt).solve_lp(prob);
      } else {
        r = NativeDualSimplexLPAdapter(budget, &slot->cancel, worker_threads)
                .solve_lp(prob);
      }
    } catch (...) {
      r.stats.success = false;
      r.stats.solver_name = use_ipm ? "NativeIPMLP" : "NativeDualSimplex";
      r.stats.status = use_ipm ? "NativeIPMLP exception"
                               : "NativeDualSimplex exception";
    }
    publish(slot, std::move(r));
  };

  std::vector<std::thread> workers;
  workers.reserve(2);
  try {
    workers.emplace_back(worker, false);
    workers.emplace_back(worker, true);
  } catch (const std::system_error&) {
    slot->cancel.store(true, std::memory_order_relaxed);
    for (auto& thread : workers) thread.join();
    if ((limited && remaining() <= 0.0) ||
        (context != nullptr && context->stop_requested())) {
      return stopped_result();
    }
    IPMLPOptions opt;
    // Windows direct-IPM policy: docs/archive/windows_remediation_2026-09-11.md, R4.
    opt.presolve = false;
    opt.time_limit_sec = remaining();
    const int worker_threads =
        context != nullptr ? context->thread_budget() : 0;
    const ScopedMklThreadLimit thread_limit(worker_threads);
    SolveResult out = NativeIPMLPAdapter(opt).solve_lp(prob);
    out.stats.portfolio_workers = 1;
    out.stats.worker_thread_limit = worker_threads;
    out.stats.portfolio_first_result_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    out.stats.runtime_sec = out.stats.portfolio_first_result_sec;
    return out;
  }

  SolveResult out;
  {
    std::unique_lock<std::mutex> lock(slot->m);
    bool ready = true;
    const auto predicate = [&] {
      return slot->have_winner ||
             slot->cancel.load(std::memory_order_relaxed);
    };
    if (limited) {
      const auto deadline = start + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(std::chrono::duration<double>(limit));
      ready = slot->cv.wait_until(lock, deadline, predicate);
    } else {
      slot->cv.wait(lock, predicate);
    }
    if (ready && slot->have_winner) {
      out = std::move(slot->winner);
    } else {
      out = stopped_result();
    }
    out.stats.portfolio_first_result_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    slot->cancel.store(true, std::memory_order_relaxed);
  }

  const auto cancel_start = std::chrono::steady_clock::now();
  for (auto& thread : workers) thread.join();
  out.stats.portfolio_workers = 2;
  out.stats.worker_thread_limit = context != nullptr &&
                                          context->thread_budget() > 0
                                      ? std::max(1,
                                                 context->thread_budget() / 2)
                                      : 0;
  out.stats.portfolio_cancel_wait_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - cancel_start).count();
  out.stats.runtime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  if (const char* dbg = std::getenv("MIPSOLVERS_LP_SELECTOR_DEBUG");
      dbg && *dbg) {
    std::fprintf(stderr,
                 "LP-PORTFOLIO winner=%s first=%.6fs cancel_wait=%.6fs\n",
                 out.stats.solver_name.c_str(),
                 out.stats.portfolio_first_result_sec,
                 out.stats.portfolio_cancel_wait_sec);
  }
  return out;
}

}  // namespace mipsolvers::engine

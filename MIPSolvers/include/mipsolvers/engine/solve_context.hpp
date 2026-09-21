#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <stop_token>
#include <thread>

#include "mipsolvers/engine/api/options.hpp"

namespace mipsolvers::engine {

/// Immutable resource contract shared by a top-level solve and every fallback.
/// The absolute deadline prevents N fallback adapters from consuming N copies
/// of a relative time limit. See
/// docs/archive/general_solver_performance_program_2026-09-13.md, R1.
class SolveContext {
 public:
  using Clock = std::chrono::steady_clock;

  explicit SolveContext(const SolveOptions& options)
      : start_(Clock::now()),
        thread_budget_(options.threads > 0
                           ? options.threads
                           : static_cast<int>(std::min(
                                 std::max(1u, std::thread::hardware_concurrency()),
                                 static_cast<unsigned>(
                                 std::numeric_limits<int>::max())))),
        explicit_thread_budget_(options.threads > 0),
        memory_limit_bytes_(options.memory_limit_bytes),
        random_seed_(options.random_seed),
        portfolio_mode_(options.portfolio_mode),
        stop_token_(options.stop_token) {
    if (!std::isfinite(options.time_limit_sec) || options.time_limit_sec < 0.0) {
      throw std::invalid_argument("SolveOptions.time_limit_sec must be finite and non-negative");
    }
    if (options.threads < 0) {
      throw std::invalid_argument("SolveOptions.threads must be non-negative");
    }
    if (options.time_limit_sec > 0.0) {
      const double representable_sec = std::chrono::duration<double>(
          Clock::time_point::max() - start_).count();
      if (options.time_limit_sec > representable_sec) {
        throw std::invalid_argument(
            "SolveOptions.time_limit_sec exceeds the steady-clock range");
      }
      limited_ = true;
      deadline_ = start_ + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(options.time_limit_sec));
    }
  }

  bool has_deadline() const noexcept { return limited_; }
  bool stop_requested() const noexcept {
    return stop_token_.stop_requested() || deadline_expired();
  }
  bool deadline_expired() const noexcept {
    return limited_ && Clock::now() >= deadline_;
  }
  double elapsed_sec() const noexcept {
    return std::chrono::duration<double>(Clock::now() - start_).count();
  }
  double requested_time_limit_sec() const noexcept {
    return limited_ ? std::chrono::duration<double>(deadline_ - start_).count()
                    : 0.0;
  }
  double remaining_time_sec() const noexcept {
    if (!limited_) return 0.0;
    return std::max(0.0, std::chrono::duration<double>(deadline_ - Clock::now()).count());
  }
  /// Encodes a finite deadline for backends where zero means "unlimited".
  /// See general_solver_performance_program_2026-09-13.md, R1.
  double backend_time_limit_sec(double unlimited_value) const noexcept {
    if (!limited_) return unlimited_value;
    return std::max(std::numeric_limits<double>::min(), remaining_time_sec());
  }
  int thread_budget() const noexcept { return thread_budget_; }
  bool has_explicit_thread_budget() const noexcept {
    return explicit_thread_budget_;
  }
  std::size_t memory_limit_bytes() const noexcept { return memory_limit_bytes_; }
  std::uint32_t random_seed() const noexcept { return random_seed_; }
  PortfolioMode portfolio_mode() const noexcept { return portfolio_mode_; }
  const std::stop_token& stop_token() const noexcept { return stop_token_; }

 private:
  Clock::time_point start_;
  Clock::time_point deadline_{};
  bool limited_{false};
  int thread_budget_{0};
  bool explicit_thread_budget_{false};
  std::size_t memory_limit_bytes_{0};
  std::uint32_t random_seed_{0};
  PortfolioMode portfolio_mode_{PortfolioMode::Latency};
  std::stop_token stop_token_;
};

}  // namespace mipsolvers::engine

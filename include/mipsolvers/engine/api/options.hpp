#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <stop_token>
#include <string>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

enum class StrategyPolicy {
  Auto,
  NativeFirst,
  ExternalFirst,
};

/// Selects whether the native LP selector optimizes one-call latency or
/// aggregate throughput under a shared worker budget.
enum class PortfolioMode {
  Latency,
  Throughput,
};

struct SolveOptions {
  std::string preferred_solver;
  bool allow_fallback{true};
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
  /// Call-wide cooperative wall-clock budget. Zero means unlimited. Every
  /// fallback receives only the time remaining from the original call.
  double time_limit_sec{0.0};
  /// Maximum call-wide solver worker budget. Zero resolves once to the host's
  /// hardware concurrency; nested and portfolio workers divide that budget.
  int threads{0};
  /// Deterministic seed forwarded to backends that expose one.
  std::uint32_t random_seed{0};
  /// Advisory call-wide memory budget. Zero means unspecified. Backends that
  /// cannot enforce it must report that limitation rather than invent a value.
  std::size_t memory_limit_bytes{0};
  PortfolioMode portfolio_mode{PortfolioMode::Latency};
  /// Cooperative caller cancellation, checked together with the deadline.
  std::stop_token stop_token{};
};

namespace api {
using SolveOptions = ::mipsolvers::engine::SolveOptions;
using PortfolioMode = ::mipsolvers::engine::PortfolioMode;
}  // namespace api

}  // namespace mipsolvers::engine

#pragma once

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

enum class PortfolioMode {
  Latency,
  Throughput,
};

struct SolveOptions {
  std::string preferred_solver;
  bool allow_fallback{true};
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
  /// Call-wide cooperative wall-clock budget. Zero means unlimited.
  double time_limit_sec{0.0};
  /// Maximum call-wide worker budget. Zero leaves backend defaults unchanged.
  int threads{0};
  /// Deterministic backend seed. Zero leaves backend defaults unchanged.
  std::uint32_t random_seed{0};
  PortfolioMode portfolio_mode{PortfolioMode::Latency};
  std::stop_token stop_token{};
};

namespace api {
using SolveOptions = ::mipsolvers::engine::SolveOptions;
using PortfolioMode = ::mipsolvers::engine::PortfolioMode;
}  // namespace api

}  // namespace mipsolvers::engine

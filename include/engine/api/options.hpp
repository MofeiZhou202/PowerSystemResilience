#pragma once

#include <map>
#include <string>

#include "hacdcpf/engine/problem_types.hpp"

namespace hacdcpf::engine {

enum class StrategyPolicy {
  Auto,
  NativeFirst,
  ExternalFirst,
};

struct SolveOptions {
  std::string preferred_solver;
  bool allow_fallback{true};
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
};

namespace api {
using SolveOptions = ::hacdcpf::engine::SolveOptions;
}  // namespace api

}  // namespace hacdcpf::engine

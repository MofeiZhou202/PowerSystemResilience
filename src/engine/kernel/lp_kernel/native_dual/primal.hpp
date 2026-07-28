#pragma once

#include <chrono>

#include "state.hpp"

namespace mipsolvers::engine::native_dual::detail {

Result run_primal_phase(
    State& state, Statistics& statistics,
    const std::chrono::steady_clock::time_point& solve_start);

}  // namespace mipsolvers::engine::native_dual::detail

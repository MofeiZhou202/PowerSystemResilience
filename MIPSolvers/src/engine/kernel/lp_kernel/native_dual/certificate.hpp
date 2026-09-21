#pragma once

#include "pricing.hpp"

namespace mipsolvers::engine::native_dual::detail {

Certificate primal_infeasibility_certificate(const State& state,
                                              const Leaving& leaving);
Certificate phase_one_farkas_certificate(const State& state);

}  // namespace mipsolvers::engine::native_dual::detail

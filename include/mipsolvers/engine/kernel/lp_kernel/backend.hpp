#pragma once

namespace mipsolvers::engine {

/// Numerical kernel used for continuous LP solves owned by native algorithms.
/// This does not transfer ownership of decomposition, cuts, branching,
/// heuristics, or the MIP tree.
enum class LpKernelBackend {
  HiGHS = 0,
  ExperimentalNative = 1,
};

constexpr bool uses_highs_lp_kernel(LpKernelBackend backend) noexcept {
  return backend == LpKernelBackend::HiGHS;
}

}  // namespace mipsolvers::engine

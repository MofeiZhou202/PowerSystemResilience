/// @file bc_solve_state.hpp
/// @brief BCSolveState: shared state object for the branch_and_cut_lp
/// decomposition.
///
/// The historically monolithic branch_and_cut_lp body is being progressively
/// split into BCSolveState methods that share state through members instead of
/// a single function's locals and [&]-capturing lambdas. Co-located internal
/// header; included by branch_and_cut.cpp and the extracted method units.
#pragma once

#include <utility>

#include "mipsolvers/engine/branch_and_cut.hpp"

namespace mipsolvers::engine::detail {

// HiGHS-style default tolerances/limits used by the branch_and_cut_lp body.
// Kept here (rather than as branch_and_cut.cpp TU-local constants) so that
// extracted BCSolveState method units can reference them.
inline constexpr double kHighsDefaultMipFeasibilityTolerance = 1e-6;
inline constexpr double kHighsDefaultSmallMatrixValue = 1e-9;
inline constexpr int kHighsDefaultMipPoolAgeLimit = 30;
inline constexpr int kHighsDefaultMipPoolSoftLimit = 10000;
inline constexpr int kHighsDefaultMipLpAgeLimit = 10;

struct BCSolveState {
  const MIPModel& prob_;
  BCOptions opt_;
  const BCCallbacks* callbacks_;

  BCSolveState(const MIPModel& prob_in, BCOptions opt_in,
               const BCCallbacks* callbacks_in)
      : prob_(prob_in), opt_(std::move(opt_in)), callbacks_(callbacks_in) {}

  BCResult run();
};

}  // namespace mipsolvers::engine::detail

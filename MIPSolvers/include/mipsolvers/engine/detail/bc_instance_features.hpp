/// @file bc_instance_features.hpp
/// @brief Instance feature extraction for the public B&C solve entry point.
///
/// Computes the compact `BCInstanceFeatures` description handed to ML/policy
/// hooks.  Extracted from the monolithic branch_and_cut translation unit; the
/// matrix-statistic helpers it uses remain private to the source file.

#pragma once

#include "mipsolvers/engine/bc/ml_hooks.hpp"    // BCInstanceFeatures
#include "mipsolvers/engine/problem_types.hpp"  // MIPModel, MINLPModel

namespace mipsolvers::engine::detail {

BCInstanceFeatures compute_instance_features(const MIPModel& prob);
BCInstanceFeatures compute_instance_features(const MINLPModel& prob);

}  // namespace mipsolvers::engine::detail

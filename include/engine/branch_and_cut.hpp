#pragma once
/// \file branch_and_cut.hpp
/// \brief Umbrella header for the native branch-and-cut engine.
///
/// This file is a thin re-export; the real definitions live under
/// `hacdcpf/engine/bc/`:
///
///   * `bc/enums.hpp`            — BranchingStrategy, NodeSelection, CutType
///   * `bc/options.hpp`          — BCOptions + resolve_num_threads
///   * `bc/stats.hpp`            — BCStats, BCResult
///   * `bc/warmstart.hpp`        — BCWarmStart (primal/dual/basis hints)
///   * `bc/branching_prior.hpp`  — BCBranchingPrior (problem-specific hints)
///   * `bc/ml_hooks.hpp`         — BCCallbacks (ML-driven policies, tuning)
///   * `bc/api.hpp`              — solve_milp_bc / solve_minlp_bc
///
/// Existing code that #includes this header continues to work unchanged.
/// New code may prefer to include only the sub-header(s) it actually needs
/// to shorten compile times.

#include "hacdcpf/engine/bc/api.hpp"
#include "hacdcpf/engine/bc/branching_prior.hpp"
#include "hacdcpf/engine/bc/enums.hpp"
#include "hacdcpf/engine/bc/ml_hooks.hpp"
#include "hacdcpf/engine/bc/options.hpp"
#include "hacdcpf/engine/bc/stats.hpp"
#include "hacdcpf/engine/bc/warmstart.hpp"

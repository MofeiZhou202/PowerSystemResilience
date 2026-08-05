#pragma once
/// \file api.hpp
/// \brief Entry points to the native branch-and-cut engine.

#include "mipsolvers/engine/bc/ml_hooks.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/bc/warmstart.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

// ─── Simple (legacy) entry points ─────────────────────────────────────────

/// Solve a MILP via branch-and-cut.
/// LP relaxations use the vendored HiGHS dual simplex by default. The native
/// dual simplex remains an explicitly selected experimental backend; IPM can
/// be configured for root or node relaxations.
BCResult solve_milp_bc(const MIPModel& prob, const BCOptions& opt = {});

/// Solve a MINLP via spatial branch-and-bound.
/// NLP relaxations at each node are solved by the native Mehrotra IPM.
BCResult solve_minlp_bc(const MINLPModel& prob, const BCOptions& opt = {});

// ─── Extended entry points (warm-start + ML callbacks) ────────────────────

/// Solve a MILP with an optional primal warm start and policy hooks.
///
/// Equivalent to `solve_milp_bc(prob, opt)` when both `ws` and `cbs` are
/// empty. Primal hints, hyperparameter tuning, post-solve reporting, and the
/// callback modes documented in `ml_hooks.hpp` are validated before solving.
/// Unsupported callback/solver combinations raise `std::invalid_argument`
/// instead of being ignored. The caller owns callback
/// targets and is responsible for thread safety when `opt.num_threads > 1`.
BCResult solve_milp_bc(const MIPModel&    prob,
                       const BCOptions&   opt,
                       const BCWarmStart& ws,
                       const BCCallbacks& cbs);

/// MINLP supports primal initial points plus hyperparameter and post-solve
/// callbacks. Other callback modes raise `std::invalid_argument`.
BCResult solve_minlp_bc(const MINLPModel&  prob,
                        const BCOptions&   opt,
                        const BCWarmStart& ws,
                        const BCCallbacks& cbs);

}  // namespace mipsolvers::engine

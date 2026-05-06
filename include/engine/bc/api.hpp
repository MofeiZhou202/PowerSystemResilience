#pragma once
/// \file api.hpp
/// \brief Entry points to the native branch-and-cut engine.

#include "hacdcpf/engine/bc/ml_hooks.hpp"
#include "hacdcpf/engine/bc/options.hpp"
#include "hacdcpf/engine/bc/stats.hpp"
#include "hacdcpf/engine/bc/warmstart.hpp"
#include "hacdcpf/engine/problem_types.hpp"

namespace hacdcpf::engine {

// ─── Simple (legacy) entry points ─────────────────────────────────────────

/// Solve a MILP via branch-and-cut.
/// LP relaxations at each node are solved by the native dual simplex (default)
/// or the Mehrotra predictor-corrector IPM (when configured).
BCResult solve_milp_bc(const MIPModel& prob, const BCOptions& opt = {});

/// Solve a MINLP via spatial branch-and-bound.
/// NLP relaxations at each node are solved by the native Mehrotra IPM.
BCResult solve_minlp_bc(const MINLPModel& prob, const BCOptions& opt = {});

// ─── Extended entry points (warm-start + ML callbacks) ────────────────────

/// Solve a MILP with an optional warm-start payload and ML hooks.
///
/// Equivalent to `solve_milp_bc(prob, opt)` when both `ws` and `cbs` are
/// empty. Supplied callbacks are invoked by the engine internally;
/// the caller owns the std::function targets and is responsible for
/// making them thread-safe if `opt.num_threads > 1`.
BCResult solve_milp_bc(const MIPModel&    prob,
                       const BCOptions&   opt,
                       const BCWarmStart& ws,
                       const BCCallbacks& cbs);

/// Same as above for MINLP.
BCResult solve_minlp_bc(const MINLPModel&  prob,
                        const BCOptions&   opt,
                        const BCWarmStart& ws,
                        const BCCallbacks& cbs);

}  // namespace hacdcpf::engine

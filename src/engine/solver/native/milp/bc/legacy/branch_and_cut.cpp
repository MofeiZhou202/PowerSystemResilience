/// @file branch_and_cut.cpp
/// @brief Legacy branch-and-cut core implementation.
///
/// Public API entry points now live in solver/native/milp/bc/api.cpp and forward
/// here through detail::solve_*_legacy_core compatibility hooks. This file retains
/// the heavy orchestration logic (sequential/parallel loops and NLP fallback) while
/// preserving old behavior.

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/detail/bc/legacy_bridge.hpp"
#include "mipsolvers/engine/detail/bc_conformance_trace.hpp"
#include "mipsolvers/engine/detail/bc_concurrent_tree_state.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <climits>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <utility>

#include "mipsolvers/core/logging.hpp"
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fmt/format.h>

#include "../extern/pdqsort/pdqsort.h"
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#include "mip/HighsCutPool.h"
#include "mip/HighsLpRelaxation.h"
#include "presolve/HighsPostsolveStack.h"
#include "util/HVector.h"
#endif
#include "util/HighsCDouble.h"
#include "util/HighsRandom.h"

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_domain.hpp"
#include "mipsolvers/engine/detail/bc_domain_probe.hpp"
#include "mipsolvers/engine/detail/bc_highs_style_numerics.hpp"
#include "mipsolvers/engine/detail/bc_instance_features.hpp"
#include "mipsolvers/engine/detail/bc_root_redcost_lurking.hpp"
#include "mipsolvers/engine/detail/bc_strict_scuc_cuts.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"
#include "mipsolvers/engine/detail/bc_implied_bounds.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_minlp_legacy.hpp"
#include "mipsolvers/engine/detail/bc_objective_propagation.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"
#include "mipsolvers/engine/detail/bc_proof_artifacts.hpp"
#include "mipsolvers/engine/detail/bc_root_audit.hpp"
#include "mipsolvers/engine/detail/bc_restart.hpp"
#include "mipsolvers/engine/detail/bc_status.hpp"
#include "mipsolvers/engine/detail/bc_uc_trace.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_validation.hpp"
#include "mipsolvers/engine/detail/bc_cglp.hpp"
#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/detail/bc_parallel.hpp"
#include "mipsolvers/engine/detail/bc_solver_dispatch.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"
#include "mipsolvers/engine/strategy/papilo_presolve.hpp"
#include "mipsolvers/engine/solver/native/milp/bc/milp_presolve.hpp"
#include "util/HighsHash.h"

#include "bc_root_highs_oracle.hpp"
#include "bc_legacy_entry_helpers.hpp"
#include "bc_solve_state.hpp"

namespace mipsolvers::engine {
using namespace detail;
namespace bc_status = detail::bc_status;

// ════════════════════════════════════════════════════════════════════════════
// Main MILP B&C Entry Point
// ════════════════════════════════════════════════════════════════════════════

namespace detail {

BCResult branch_and_cut_lp(const MIPModel& prob, BCOptions opt,
                           const BCCallbacks* callbacks = nullptr) {
  BCSolveState state(prob, std::move(opt), callbacks);
  return state.run();
}

BCResult BCSolveState::run() {
#include "bc_run/01_setup_presolve_root_build.inc"
#include "bc_run/02_root_relaxation_a.inc"
#include "bc_run/03_root_relaxation_b.inc"
#include "bc_run/04_root_relaxation_c.inc"
#include "bc_run/05_root_relaxation_d.inc"
#include "bc_run/06_root_heuristics_a.inc"
#include "bc_run/07_root_heuristics_b.inc"
#include "bc_run/08_root_heuristics_c.inc"
#include "bc_run/09_tree_infrastructure.inc"
#include "bc_run/10_node_domain_conflict_a.inc"
#include "bc_run/11_node_domain_conflict_b.inc"
#include "bc_run/12_node_processing_dual_proofs.inc"
#include "bc_run/13_main_loop_sequential.inc"
#include "bc_run/14_parallel_path_and_result.inc"
}

}  // namespace detail

BCResult detail::solve_milp_bc_legacy_core(const MIPModel& prob, const BCOptions& opt) {
  // Auto-tune is now handled inside branch_and_cut_lp after presolve,
  // using the reduced model dimensions for a more accurate threshold.
  return branch_and_cut_lp(prob, opt);
}

BCResult detail::solve_minlp_bc_legacy_core(const MINLPModel& prob, const BCOptions& opt) {
  return branch_and_cut_nlp(prob, opt);
}

// ─── Extended (warm-start + callbacks) overloads ─────────────────────────


BCResult detail::solve_milp_bc_legacy_core(const MIPModel&    prob,
                                           const BCOptions&   opt_in,
                                           const BCWarmStart& ws,
                                           const BCCallbacks& cbs) {
  BCOptions opt = opt_in;
  MIPModel solve_prob = install_primal_warm_start(prob, ws);
  BCInstanceFeatures feat;
  bool have_feat = false;
  if (cbs.hyperparam_tuner) {
    feat = compute_instance_features(solve_prob);
    have_feat = true;
    opt = cbs.hyperparam_tuner(feat, opt, /*prev_stats=*/nullptr);
  }
  validate_milp_callbacks(cbs, opt);
  BCResult out = branch_and_cut_lp(solve_prob, opt, &cbs);
  if (cbs.post_solve) {
    if (!have_feat) feat = compute_instance_features(solve_prob);
    cbs.post_solve(feat, opt, out.bc_stats);
  }
  return out;
}

BCResult detail::solve_minlp_bc_legacy_core(const MINLPModel&  prob,
                                            const BCOptions&   opt_in,
                                            const BCWarmStart& ws,
                                            const BCCallbacks& cbs) {
  BCOptions opt = opt_in;
  validate_minlp_callbacks(cbs);
  MINLPModel solve_prob = install_primal_warm_start(prob, ws);
  BCInstanceFeatures feat;
  bool have_feat = false;
  if (cbs.hyperparam_tuner) {
    feat = compute_instance_features(solve_prob);
    have_feat = true;
    opt = cbs.hyperparam_tuner(feat, opt, /*prev_stats=*/nullptr);
  }
  BCResult out = branch_and_cut_nlp(solve_prob, opt);
  if (cbs.post_solve) {
    if (!have_feat) feat = compute_instance_features(solve_prob);
    cbs.post_solve(feat, opt, out.bc_stats);
  }
  return out;
}

}  // namespace mipsolvers::engine

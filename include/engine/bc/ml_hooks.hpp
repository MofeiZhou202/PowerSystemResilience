#pragma once
/// \file ml_hooks.hpp
/// \brief Callback surface for machine-learning-driven tuning & policies.
///
/// This header defines the integration surface between the B&C engine and
/// an external ML policy (for example, a learned branching agent, a cut
/// selector, or a Bayesian hyper-parameter optimizer). All hooks are
/// optional — the engine falls back to its built-in heuristics whenever
/// a hook is not attached.
///
/// The design goals are:
///
///   1. Zero run-time overhead when all hooks are `nullptr` (the usual case).
///   2. A pure-function call contract: the engine never owns the policy;
///      the caller is responsible for thread-safety of the callbacks.
///   3. Forward-compatibility: adding a new hook to this struct is a
///      source-compatible change (existing call sites use `{}`-init).
///
/// These hooks are intentionally kept generic so that both learned and
/// rule-based tuners can share the same infrastructure.

#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/engine/bc/branching_prior.hpp"

namespace hacdcpf::engine {

struct BCOptions;     // fwd (options.hpp)
struct BCStats;       // fwd (stats.hpp)
struct BCWarmStart;   // fwd (warmstart.hpp)

// ─── Instance features (input to ML policies) ─────────────────────────────
/// Feature vector computed from the presolved model. Supplied to policy
/// hooks that want a compact problem description (without re-walking the
/// LP matrix).
struct BCInstanceFeatures {
  int    n_vars{0};
  int    n_bin{0};
  int    n_int{0};
  int    n_rows{0};
  int    n_eq{0};
  int    n_nnz{0};
  double obj_density{0.0};        ///< nnz(c) / n_vars
  double row_density_median{0.0}; ///< median nnz per row
  double obj_range_ratio{0.0};    ///< max|c| / min|c| (nonzero entries)
  double rhs_range_ratio{0.0};
  double coeff_range_ratio{0.0};
  int    num_binary_blocks{0};    ///< SCUC-like structural heuristic
  int    max_clique_size{0};      ///< max clique in conflict graph (0 = not computed)
  std::vector<double> extra;      ///< caller-defined problem-family tags
};

// ─── Hook signatures ──────────────────────────────────────────────────────

/// Hyperparameter tuner: given instance features (and, for restart,
/// previous-run stats), return an adjusted `BCOptions`. The returned object
/// fully replaces the caller-supplied options.
using BCHyperparamTunerFn =
    std::function<BCOptions(const BCInstanceFeatures& feat,
                            const BCOptions& base,
                            const BCStats*   prev_stats /*nullable*/)>;

/// Node-selection advisor: given an open-node summary (lb, depth, estimate),
/// return a reordering permutation. Called at most every N nodes.
struct BCNodeSummary {
  int    node_id{0};
  int    depth{0};
  double lb{0.0};
  double estimate{0.0};
};
using BCNodeSelectorFn =
    std::function<void(const std::vector<BCNodeSummary>& open,
                       std::vector<int>& out_permutation)>;

/// Cut selector: given the list of candidate cuts and a caller-supplied
/// scoring budget, pick an accept/reject mask (same length as `candidates`).
struct BCCutCandidate {
  double efficacy{0.0};
  double density{0.0};
  double parallelism_max{0.0};  ///< max |cos(θ)| with existing cuts
  int    support_binary{0};
  int    support_total{0};
  double lp_violation{0.0};
  const char* family{""};       ///< "GMI" | "MIR" | "Cover" | ...
};
using BCCutSelectorFn =
    std::function<void(const std::vector<BCCutCandidate>& candidates,
                       std::vector<bool>& out_accept)>;

/// Post-solve event: fired once after the run completes. Useful to log
/// (features, options, final stats) triples for offline training.
using BCPostSolveFn =
    std::function<void(const BCInstanceFeatures& feat,
                       const BCOptions& opt_used,
                       const BCStats&   stats)>;

// ─── Aggregate hook record ────────────────────────────────────────────────
/// Aggregate of all ML / policy hooks. Pass `BCCallbacks{}` (or omit
/// entirely) to disable all hooks.
struct BCCallbacks {
  BCBranchingPriorFn   branching_prior;
  BCNodeSelectorFn     node_selector;
  BCCutSelectorFn      cut_selector;
  BCHyperparamTunerFn  hyperparam_tuner;
  BCPostSolveFn        post_solve;

  /// Optional human-readable tag used for log tracing (e.g. "gnn-v3").
  std::string policy_tag;

  /// True when all hooks are empty (fast path).
  bool empty() const {
    return !branching_prior && !node_selector && !cut_selector
        && !hyperparam_tuner && !post_solve;
  }
};

}  // namespace hacdcpf::engine

#pragma once
/// \file ml_hooks.hpp
/// \brief Callback surface for machine-learning-driven tuning & policies.
///
/// This header defines the integration surface between the B&C engine and
/// an external policy (for example, a learned branching agent, a dynamic-cut
/// generator, or a Bayesian hyper-parameter optimizer). All hooks are
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

#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mipsolvers/engine/bc/branching_prior.hpp"
#include "mipsolvers/engine/bc/enums.hpp"

namespace mipsolvers::engine {

struct BCOptions;     // fwd (options.hpp)
struct BCStats;       // fwd (stats.hpp)
struct BCWarmStart;   // fwd (warmstart.hpp)

// ─── Instance features (input to ML policies) ─────────────────────────────
/// Feature vector computed from the model presented to the public solve entry
/// point. Supplied to policy hooks that want a compact problem description.
/// Fields that do not apply to a problem class, such as RHS ranges for MINLP,
/// are zero by contract rather than fabricated estimates.
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
  /// Maximum clique size when a conflict graph collector has actually run.
  /// `nullopt` means unavailable; it is not encoded as a fabricated zero.
  std::optional<int> max_clique_size;
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

/// Dynamic node cut request used by the strict HiGHS lifecycle callback.
/// The row is interpreted as lower <= sum(values[k] * x[indices[k]]) <= upper.
struct BCDynamicNodeCut {
  enum class ColumnSpace {
    /// Indices are interpreted in the active HiGHS node LP column space.
    NodeLp,
    /// Indices are interpreted in the caller's original MIP model column space.
    /// The strict HiGHS bridge projects them through the current presolve
    /// postsolve stack before inserting the row into the node LP cutpool.
    Original
  };

  std::vector<int> indices;
  std::vector<double> values;
  /// Optional audit labels. They do not affect solver semantics, but are
  /// emitted by strict HiGHS projection checks when a dynamic cut is rejected.
  std::string audit_family;
  std::string audit_key;
  int audit_period{-1};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  bool integral{false};
  bool propagate{false};
  ColumnSpace column_space{ColumnSpace::NodeLp};
  /// Default is backward-compatible: existing callbacks generate global user
  /// cuts unless they explicitly mark a row as node-local or lazy.
  ValidityScope validity_scope{ValidityScope::GlobalCut};
};

/// Dynamic node cut context at the exact node LP lifecycle point:
/// after HiGHS resolves the active node LP and before incumbent/prune/branch.
struct BCDynamicNodeCutContext {
  std::string event;
  std::int64_t node_count{0};
  int depth{0};
  double lp_objective{0.0};
  /// Active node-LP solution in current HiGHS LP column space.
  std::vector<double> col_value;
  std::vector<double> col_lower;
  std::vector<double> col_upper;
  /// Current HiGHS LP column -> original model column. Empty if unavailable.
  std::vector<int> reduced_to_original_col;
  /// Original-space solution reconstructed for linearly transformable columns.
  /// Entries that cannot be reconstructed are NaN.
  std::vector<double> original_col_value;
};

using BCDynamicNodeCutFn =
    std::function<void(const BCDynamicNodeCutContext& ctx,
                       std::vector<BCDynamicNodeCut>& out_cuts)>;

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
  BCDynamicNodeCutFn   dynamic_node_cut;
  BCHyperparamTunerFn  hyperparam_tuner;
  BCPostSolveFn        post_solve;

  /// Optional human-readable tag used for log tracing (e.g. "gnn-v3").
  std::string policy_tag;

  /// True when all hooks are empty (fast path).
  bool empty() const {
    return !branching_prior && !dynamic_node_cut && !hyperparam_tuner
        && !post_solve;
  }
};

}  // namespace mipsolvers::engine

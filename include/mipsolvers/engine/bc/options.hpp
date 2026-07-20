#pragma once
/// \file options.hpp
/// \brief Option bundle for the native branch-and-cut engine.
///
/// Fields are grouped into logical sections by comment markers
/// (Core / Cuts / Heuristics / Parallel / Fallback / Auto-tune / ...).
/// The flat-field layout is preserved verbatim for backward compatibility:
/// existing code using e.g. `opt.num_threads` continues to compile unchanged.
///
/// When adding a new option, place it in the section that matches its
/// component. If a new section is warranted, add it here rather than in the
/// umbrella header `branch_and_cut.hpp`.

#include "mipsolvers/engine/bc/enums.hpp"

#include <memory>
#include <vector>

namespace mipsolvers::engine {

/// \brief Root cut pool for warm-starting HiGHS MIP solves on repeated
/// solves of the same (or structurally identical) problem.
///
/// All column indices are in the **original** (pre-presolve) LP column space
/// of the model passed to HiGHS.  Each row i represents:
///   lower[i] <= sum_{k=start[i]}^{start[i+1]-1} value[k] * x[index[k]] <= upper[i]
/// Use -kInf / +kInf (e.g. -1e30 / 1e30) for one-sided constraints.
struct BCRootCuts {
  std::vector<int>    start;   ///< row starts, size = n_cuts+1
  std::vector<int>    index;   ///< original column indices
  std::vector<double> value;   ///< coefficients rescaled to original space
  std::vector<double> lower;   ///< row lower bounds (-1e30 for pure ≤ cuts)
  std::vector<double> upper;   ///< row upper bounds (+1e30 for pure ≥ cuts)
  int  numCuts() const { return static_cast<int>(lower.size()); }
  bool empty()   const { return lower.empty(); }
};

/// \brief Original-space root simplex basis matching a BCRootCuts warm-start.
///
/// Status codes use HiGHSBasisStatus integer values, but this struct stays
/// independent of HiGHS headers so BCOptions remains a lightweight API type.
/// row_status covers the original model rows followed by the injected root-cut
/// rows in BCRootCuts order.
struct BCRootBasis {
  std::vector<int> col_status;
  std::vector<int> row_status;
  bool valid{false};
  bool empty() const { return !valid || col_status.empty() || row_status.empty(); }
};

/// \brief Pseudocost initialization for warm-starting branching decisions on
/// repeated solves of the same (or structurally identical) problem.
///
/// All arrays are in the **original** (pre-presolve) LP column space.
/// Populated by BCResult::highs_pseudocost_init after a solve; pass as
/// BCOptions::highs_pseudocost_warm_start to the next solve to seed the
/// HiGHS pseudocost table with data learned during the previous B&B.
struct BCPseudocostInit {
  std::vector<double> pseudocostup;
  std::vector<double> pseudocostdown;
  std::vector<int>    nsamplesup;
  std::vector<int>    nsamplesdown;
  std::vector<double> inferencesup;
  std::vector<double> inferencesdown;
  std::vector<int>    ninferencesup;
  std::vector<int>    ninferencesdown;
  std::vector<double> conflictscoreup;
  std::vector<double> conflictscoredown;
  double  cost_total{0.0};
  double  inferences_total{0.0};
  double  conflict_avg_score{0.0};
  int64_t nsamplestotal{0};
  int64_t ninferencestotal{0};
  int     n_orig_cols{0};   ///< Number of original LP columns (for validation)
  bool empty() const { return pseudocostup.empty(); }
};

}  // namespace mipsolvers::engine

namespace mipsolvers::engine {

/// Option bundle for the branch-and-cut algorithm.
struct BCOptions {
  // ═══════════════════════════════════════════════════════════════════════
  // Core limits and tolerances
  // ═══════════════════════════════════════════════════════════════════════
  int    max_nodes{50'000};         ///< Maximum B&B nodes before stopping
  int    max_lp_iter{500};          ///< IPM iteration limit per LP/NLP node solve
  double time_limit_sec{120.0};     ///< Wall-clock time limit
  double int_tol{1e-5};             ///< Integrality tolerance
  double gap_tol{1e-4};             ///< Relative primal-dual gap tolerance
  double lp_tol{1e-6};              ///< LP-relaxation convergence tolerance

  // ═══════════════════════════════════════════════════════════════════════
  // Cut generation
  // ═══════════════════════════════════════════════════════════════════════
  int    root_cut_rounds{10};       ///< Number of cut-generation rounds at the root
  int    cuts_per_round{20};        ///< Maximum new cuts per generation round/node
  int    max_cut_depth{0};          ///< Generate cuts at tree nodes up to this depth
  double gmi_min_efficacy{1e-3};    ///< Min normalized violation for GMI admission
  double gmi_max_density{0.35};     ///< Max nonzero ratio allowed for admitted GMI cuts
  double gmi_max_parallelism{0.90}; ///< P2.2: Reject near-parallel cuts
  double gmi_min_activity{0.0};     ///< Min alignment with highly fractional binary support
  double gmi_min_binary_support{0.0}; ///< Min share of nonzeros on binary vars
  double gmi_activity_weight{0.5};  ///< Activity bonus weight in GMI ranking score
  CutType cuts{CutType::All};       ///< Cut family selector

  // ═══════════════════════════════════════════════════════════════════════
  // Branching & node selection
  // ═══════════════════════════════════════════════════════════════════════
  BranchingStrategy branching{BranchingStrategy::Pseudocost};
  NodeSelection     node_sel{NodeSelection::Hybrid};

  // ═══════════════════════════════════════════════════════════════════════
  // LP solver selection
  // ═══════════════════════════════════════════════════════════════════════
  bool use_simplex_lp_nodes{true};
  bool use_ipm_root{false};
  bool use_ipm_nodes{false};
  /// Health-probe gate for the IPM root path: before paying for a full IPM
  /// root solve, run a few relaxed-tolerance iterations on the same LP.  A
  /// probe ending in NumericalError / Cholesky failed / ProblemTooLarge
  /// diagnoses the IPM as unhealthy on this instance: the root solve routes
  /// straight to simplex and the simplex-failure IPM repair is skipped
  /// (MaxIter / Time limit / success all count as healthy).  On healthy
  /// probes the full solve is seeded with the probe iterate, so probe work
  /// is not wasted.
  bool ipm_root_probe{true};
  int ipm_probe_max_iter{20};
  double ipm_probe_time_sec{5.0};
  /// Use the self-contained embedded HiGHS simplex kernel for native B&C LP
  /// relaxations.  The MIP tree, cuts, proof frontier, scheduling, and domain
  /// heuristics remain native; HiGHS is used only as the LP numerical kernel.
  bool use_vendored_highs_lp_kernel{false};
  /// Internal scheduling guard: keep root LP/cut frontier on native simplex
  /// while allowing vendored HiGHS for node/proof LPs. This prevents the
  /// vendored degenerate root frontier from starving root cuts.
  bool suppress_vendored_highs_root_frontier{false};
  // Dual-simplex factor backend selector passed through to SimplexOptions.
  // 0 = BackendA_UmfpackNative (default)
  // 1 = BackendB_HiGHSSafe
  // 2 = BackendB_ForceFT
  // 3 = BackendB_ShortChain
  int simplex_factor_backend{0};
  // Enable benchmark-only audits that compare the live branch choice against
  // a fresh-refactor branch score on sampled nodes.
  bool enable_ft_drift_telemetry{false};
  int ft_branch_audit_max_samples{0};
  int ft_branch_audit_depth_limit{6};

  // ═══════════════════════════════════════════════════════════════════════
  // Heuristics: feasibility pump, presolve
  // ═══════════════════════════════════════════════════════════════════════
  bool use_feasibility_pump{true};
  bool use_progressive_rounding{true};
  bool use_papilo_presolve{true};
  bool papilo_aggressive{false};
  bool verbose{false};
  /// Accept a caller-provided initial_solution once the fixed-integer
  /// dispatch LP verifies feasibility.  Even a weak incumbent is useful for
  /// objective cutoffs, reduced-cost fixing, and pruning.
  bool accept_verified_warm_start_incumbent{true};
  /// Reject candidate incumbents whose (minimize-convention) objective
  /// exceeds the best known dual bound by more than this relative factor:
  ///   obj - bound > factor * max(1, |bound|).
  /// Catches pathological repair-path incumbents (e.g. 1e6x the optimum)
  /// without blocking legitimately weak early heuristics.  Non-positive or
  /// non-finite disables the gate.
  double incumbent_quality_reject_factor{1e6};
  /// When the root LP relaxation of the PaPILO-presolved model fails or
  /// exhausts its (capped) budget, retry the whole solve once with PaPILO
  /// presolve disabled.  The presolved 118-bus SCUC root LP is unsolvable
  /// for the native simplex while the unpresolved one solves cleanly; this
  /// converts such failures into valid (if slower) solves.  Only fires when
  /// presolve actually reduced the model and enough wall-clock remains.
  bool root_presolve_fallback{true};
  /// Minimum remaining wall-clock (seconds) required to attempt the
  /// unpresolved-root fallback retry.
  double root_presolve_fallback_min_remaining_sec{5.0};
  /// Fraction of the total time limit reserved for the unpresolved-root
  /// fallback retry: the presolved root LP's budget is capped at
  /// (limit - max(min_remaining, fraction * limit)) so a hard presolved
  /// root cannot starve the retry.  0 disables the reserve (root LP may
  /// use the full remaining budget, as before).
  double root_presolve_fallback_budget_fraction{0.5};
  /// Generic HiGHS-style feasibility jump: locally flips binary variables to
  /// reduce row infeasibility, then repairs continuous variables by an LP with
  /// integers fixed.  No model-specific metadata is used.
  bool enable_feasibility_jump{true};
  int  feasibility_jump_max_flips{512};
  int  feasibility_jump_repair_attempts{3};
  /// Domain-specific heuristics are disabled by default so native B&C remains
  /// a general MILP solver.  When true, the solver may consume optional model
  /// hints such as UCGenHint and enable UC/SCUC-shaped shortcuts.
  bool enable_domain_heuristics{false};

  // ═══════════════════════════════════════════════════════════════════════
  // Cut & solution pools
  // ═══════════════════════════════════════════════════════════════════════
  int cut_pool_max_size{2000};
  int cut_pool_max_age{50};
  int solution_pool_size{10};

  // ═══════════════════════════════════════════════════════════════════════
  // Parallelism
  // ═══════════════════════════════════════════════════════════════════════
  int num_threads{-1};
  int cut_worker_queue_size{64};
  int cut_worker_queue_timeout_ms{50};
  int max_plunge_depth{10};
  int auto_parallel_min_threads{2};
  int auto_parallel_max_threads{0};

  // Parallel cross-thread info sharing (2026-Q2 improvements;
  // see docs/parallel_bc_sharing_improvements_2026q2.md).
  /// @brief Master switch for cross-thread conflict-clause / conflict-cut /
  /// binary-implication sharing. Keep OFF by default: these propagate any
  /// subtle scoping mistake across the whole forest and can cause a parallel
  /// run to prune regions that sequential / Gurobi would visit. When off,
  /// each explorer still uses its own local conflict pool and implication
  /// graph, so primal learning inside a thread is retained.
  bool share_conflict_learning_across_threads{false};
  /// @brief Publish locally-learned binary implications to a shared graph so
  /// sibling explorers can reuse them. Append-only, generation-counter sync.
  /// Effective only when share_conflict_learning_across_threads is true.
  bool share_implications_across_threads{true};
  /// @brief Per-thread publish-buffer size before flushing to the shared graph.
  int implication_publish_batch_size{8};
  /// @brief Use delta-aggregate pseudocost merge (sum counts/sums across
  /// threads) instead of the legacy keep-max merge. Strictly more samples
  /// retained, zero additional locking.
  bool pseudocost_delta_merge{true};
  /// @brief Do not launch the parallel tree until a finite incumbent is
  /// available. Before that point there is no primal bound for pruning, so
  /// multi-threaded search can expand strictly more nodes than the serial DFS
  /// route.
  bool parallel_delay_until_incumbent{true};
  /// @brief For large generic roots with very few fractional binaries, keep
  /// the proof/search phase serial. These cases usually benefit more from
  /// reduced-cost/domain fixing than from speculative parallel expansion.
  bool parallel_serial_on_low_fractionality_root{true};
  /// @brief If a finite incumbent exists but the root gap is not closed, allow
  /// the parallel tree to prove the remaining gap even on low-fractionality
  /// roots. This mirrors HiGHS' proof-parallel posture: primal discovery may
  /// be serial and fast, while the certificate tail is shared by workers.
  bool parallel_proof_on_incumbent_gap{true};
  /// Require an explicit live-tree certificate instead of pruning queued nodes
  /// solely because their lower bound is within the requested relative MIP gap.
  /// Benchmark conformance keeps this enabled so objective audits compare the
  /// same integer frontier as HiGHS/Gurobi rather than a tolerance-admissible
  /// incumbent.
  bool require_tree_exhaustion_certificate{true};
  /// @brief Experimental: after reduced-cost fixing, run one additional
  /// generic propagation pass on the tightened node domain. Disabled by
  /// default because the current legacy path would need an immediate LP
  /// re-solve before using the tightened-domain relaxation for branching.
  bool rc_fixing_followup_propagation{false};
  /// Learn objective-cutoff conflict clauses from reduced-cost fixings.  When
  /// a node LP proves that reversing a reduced-cost fixing cannot improve the
  /// incumbent, record branch_path + reversed_literal as a reusable conflict.
  bool enable_reduced_cost_conflict_learning{true};
  /// Master switch for reduced-cost domain fixing.  Keep true in production;
  /// audit-only simple B&B disables it to isolate plain LP-bound pruning.
  bool enable_reduced_cost_fixing{true};
  int  reduced_cost_conflict_max_per_node{8};
  /// Experimental proof-frontier conflict minimization. Disabled by default
  /// while transformed-source conformance is audited: it is not part of the
  /// HiGHS HighsLpAggregator -> HighsTransformedLp -> HighsCutGeneration
  /// source path, so leaving it on would mix proof-learning artifacts into the
  /// source-conformance counters.
  bool enable_reduced_cost_proof_conflict_minimization{false};
  /// Maximum branch-path length retained as a propagation-only reduced-cost
  /// cutoff conflict.  These clauses are not necessarily added as LP rows;
  /// they let descendants of the same path prune before solving another LP.
  int  reduced_cost_conflict_pool_max_literals{256};
  /// Maximum size of a binary no-good row admitted from a reduced-cost cutoff
  /// conflict.  Pairwise clauses still feed the implication graph; longer
  /// short clauses are kept sparse and help prune repeated branch prefixes.
  int  reduced_cost_conflict_cut_max_literals{32};
  /// Try to shrink long reduced-cost cutoff conflicts by taking a short suffix
  /// of the branch path plus the forbidden literal, then verifying it with a
  /// cutoff LP before learning.  This is budgeted because each successful or
  /// failed proof costs an LP solve, but it can turn otherwise unusable long
  /// paths into sparse propagation clauses.
  bool enable_verified_reduced_cost_conflict_minimization{false};
  int  reduced_cost_verified_conflict_max_literals{8};
  int  reduced_cost_verified_conflict_max_lps{32};
  int  reduced_cost_verified_conflict_max_per_node{1};
  /// After reduced-cost fixing tightens a child domain, optionally re-solve the
  /// child LP under the tightened bounds so the fixing can move the node bound
  /// immediately.  This is the proof-safe version of follow-up propagation:
  /// tightened domains are not used for branching until the LP relaxation has
  /// been refreshed.
  bool enable_reduced_cost_fixing_resolve{true};
  int  reduced_cost_fixing_resolve_min_fixings{1};
  int  reduced_cost_fixing_resolve_max_lps{64};
  /// Re-solve a small budget of nodes immediately after newly learned
  /// proof-frontier no-good rows are admitted at that node.  This is the
  /// bound-moving counterpart to propagation-only learning: the row is already
  /// proof-valid, and the refreshed LP bound can prune the node before it is
  /// branched further.  Keep the default budget small: on large structured
  /// MILPs many proof-frontier rows are valid but degenerate, so unbounded
  /// local re-solves can cost more than they save until the separator learns a
  /// truly bound-moving proof.
  bool enable_reduced_cost_proof_cut_resolve{true};
  int  reduced_cost_proof_cut_resolve_max_lps{4};

  /// @brief Deterministic-parallel mode.
  ///
  /// When enabled, explorer threads synchronize their node-queue pop and
  /// child-push operations through a round-robin turn token: at iteration
  /// number `k`, worker `(k mod n_explorers)` is granted the queue while
  /// the others block.  LP solves still run in parallel (the token is
  /// released between pop and push), so the wall-clock cost is modest,
  /// but the *order* in which nodes are popped becomes a pure function
  /// of the queue contents and the worker count — independent of OS
  /// scheduling jitter.
  ///
  /// Additionally, each explorer iteration takes a single atomic
  /// `SharedIncumbent` snapshot at the top of the loop and reuses it
  /// throughout, eliminating the torn read between `shared_inc.has()`
  /// and `shared_inc.get_obj()` that today causes two threads visiting
  /// the same node to reach different prune/keep decisions.
  ///
  /// Disabled by default.  Turn on for debugging, regression triage, or
  /// when reproducibility of the parallel search tree matters more than
  /// a few percent of throughput.
  bool deterministic_parallel{false};

  // ═══════════════════════════════════════════════════════════════════════
  // LP fallback recovery (Phase 1)
  // ═══════════════════════════════════════════════════════════════════════
  bool enable_lp_fallback{true};
  int fallback_l1_retries{3};
  double fallback_l1_perturbation{1e-7};
  int fallback_l3_retries{2};
  double fallback_l3_tol_mult{10.0};

  // ═══════════════════════════════════════════════════════════════════════
  // Auto-tuning thresholds (solver-mode gates)
  // ═══════════════════════════════════════════════════════════════════════
  int ipm_auto_threshold{8000};
  int large_problem_threshold{10000};
  /// P7.6: Raised from 5000→15000.  For m≤15000 simplex warm-start re-optimises
  /// in ≤30 pivots (~0.3 ms), while IPM Cholesky costs ≥1 ms.  IPMDiver is only
  /// beneficial when simplex cold-start would dominate (m>15000).
  int hybrid_ipm_threshold{15000};
  int xlarge_ipm_only_threshold{50000};

  // ═══════════════════════════════════════════════════════════════════════
  // Root heuristic budgets
  // ═══════════════════════════════════════════════════════════════════════
  int feasibility_pump_iters{5};
  int max_dive_lps{60};
  int max_probe_vars{50};
  int bound_propagation_rounds{3};

  // ═══════════════════════════════════════════════════════════════════════
  // Root cut control
  // ═══════════════════════════════════════════════════════════════════════
  double root_cut_stall_tol{1e-4};
  int    root_cut_max_stalls{2};
  /// Audit mode: force root separation through size/fractionality skip gates so
  /// we can diagnose whether the root cut pipeline is genuinely empty or merely
  /// bypassed by performance guards.  This is not intended as a production
  /// heuristic; benchmark runners enable it explicitly for root-only audits.
  bool root_cut_audit_force_separation{false};
  /// Validate root cut rows after each separation round.  The check is generic:
  /// finite coefficients/RHS, nonzero row norm, violation at the current root LP
  /// point, and preservation of a verified reduced-space warm start when one is
  /// available.  Invalid or incumbent-cutting rows are reverted immediately.
  bool root_cut_audit_validate_rows{true};
  /// Root-local rows must be termination-effective: after admission they must
  /// either tighten the root domain or improve the root LP lower bound after a
  /// re-solve.  This keeps proof-valid but non-moving rows out of the live LP.
  bool root_cut_reject_nonmoving_rows{true};
  double root_cut_min_bound_lift{1.0e-6};
  /// Experimental generic separator for rows of the form
  /// sum x_i = demand with variable upper bounds x_i <= U_i y_i.
  /// Kept off by default until it gives positive root-bound/time tradeoffs.
  bool enable_projected_capacity_cuts{false};
  /// Generic objective-cutoff conflict cuts for rows where a nonnegative
  /// continuous cost variable is implied by binary literals, e.g.
  /// s >= M*y or s >= M*(y-z).  These cuts are scoped to the current
  /// incumbent cutoff: they exclude only solutions that cannot improve the
  /// incumbent, and are therefore not ordinary global cover cuts.
  bool enable_objective_cutoff_conflict_cuts{true};
  int  objective_cutoff_conflict_max_cuts{64};
  /// Add sparse weighted objective-cutoff event rows of the form
  /// sum c_e(y_e-z_e) <= cutoff_budget for the most active implied-cost
  /// events. These are cutoff-valid and can move the LP bound directly.
  bool enable_objective_cutoff_weighted_event_cuts{true};
  int  objective_cutoff_weighted_event_max_terms{128};
  /// Use the same objective-cutoff event budget at tree nodes for local domain
  /// fixing and cutoff pruning.  This never adds node-context-dependent rows
  /// to the global LP; it only tightens the current node bounds when the
  /// incumbent cutoff proves a binary event cannot occur.
  bool enable_objective_cutoff_domain_fixing{true};
  /// Separate violated two-term implied-bound cuts from the binary implication
  /// graph, using the HiGHS-style formulas for x=0/1 => bound(y).
  bool enable_graph_implied_bound_cuts{true};
  int  graph_implied_bound_max_cuts{128};
  /// HiGHS-style local probing for dynamic implied-bound separation: for a
  /// small number of fractional binary columns at the current node, temporarily
  /// set x=0/1, replay domain propagation, and turn resulting local
  /// implications into violated two-term implied-bound rows.  Rows are local
  /// only; they are not inserted into the global cut pool.
  bool enable_dynamic_implied_bound_probing{true};
  int  dynamic_implied_bound_probing_max_vars{8};
  int  dynamic_implied_bound_probing_max_rows{8};
  /// Local dynamic implied-bound rows are expensive because they are produced
  /// by temporary propagation.  HiGHS only spends this work when the row can
  /// hit the live trail/queue or violate the current LP point, so keep probing
  /// near the cutoff frontier and reject non-moving rows early.
  double dynamic_implied_bound_frontier_gap_factor{10.0};
  double dynamic_implied_bound_min_violation{1.0e-7};
  double dynamic_implied_bound_min_bound_move{1.0e-6};
  /// Also admit sparse objective-cutoff conflict covers that do not cut the
  /// current root LP point. These can prune later nodes, but are experimental
  /// because they may add LP burden without moving the root bound.
  bool enable_nonviolated_cutoff_conflict_covers{false};
  /// Minimum mandatory event cost as a fraction of the incumbent/root gap
  /// before an event participates in cutoff conflict covers.
  double objective_cutoff_conflict_min_event_gap_fraction{0.05};
  /// Generic root RENS for low-fractionality roots.  HiGHS does not fix every
  /// root-LP-integral integer at once; it dives until a target fixing rate is
  /// reached, then solves a sub-MIP on the transformed/root LP context.
  bool enable_root_low_fractionality_rens{false};
  double root_low_fractionality_rens_target_fixing_rate{0.60};
  int  root_low_fractionality_rens_max_free_ints{4096};
  double root_low_fractionality_rens_time_limit{3.0};
  int  root_low_fractionality_rens_max_nodes{200};
  /// Root-only generic cut strengthening: detect continuous columns that are
  /// forced integral by equality rows over integer columns, and let root
  /// GMI/MIR separation treat them as integer support. This does not add them
  /// to branching candidates or incumbent integrality checks.
  bool enable_implied_integer_cut_strengthening{false};

  // ═══════════════════════════════════════════════════════════════════════
  // HiGHS-alignment controlled benchmark modes
  // ═══════════════════════════════════════════════════════════════════════
  /// Use implied-bound-first / clique-first separation ordering (HiGHS style).
  bool highs_style_root_separation{false};
  /// Use HiGHS-style pseudocost scoring (HighsPseudoCostState).
  bool highs_style_branch_scoring{false};
  /// Use HiGHS-style reliability probing schedule.
  bool highs_style_reliability_probing{false};

  // ═══════════════════════════════════════════════════════════════════════
  // Heuristic frequencies (sequential tree)
  // ═══════════════════════════════════════════════════════════════════════
  int crossover_heuristic_freq{100};
  int rins_frequency{200};

  // ═══════════════════════════════════════════════════════════════════════
  // P3.1: Large Neighborhood Search (LNS)
  // ═══════════════════════════════════════════════════════════════════════
  bool   enable_lns{true};
  double lns_fix_ratio{0.80};
  int    lns_node_limit{500};
  double lns_time_limit{5.0};
  int    lns_max_iters{3};
  /// Generic incumbent polishing: solve a tiny local-branching sub-MIP around
  /// the current incumbent.  This is domain-agnostic and only accepts a
  /// solution after validating it against the outer model.
  bool   enable_incumbent_local_branching{false};
  int    incumbent_local_branching_initial_radius{4};
  int    incumbent_local_branching_max_radius{16};
  int    incumbent_local_branching_max_iters{2};
  int    incumbent_local_branching_node_limit{250};
  double incumbent_local_branching_time_limit{0.75};

  // ═══════════════════════════════════════════════════════════════════════
  // P5.1: Work stealing (parallel only)
  // ═══════════════════════════════════════════════════════════════════════
  bool enable_work_stealing{true};
  int  work_steal_deque_size{256};

  // ═══════════════════════════════════════════════════════════════════════
  // Determinism
  // ═══════════════════════════════════════════════════════════════════════
  /// Seed for parallel RNGs (work-stealing victim selection, etc.).
  /// Deterministic default makes parallel runs reproducible across solves
  /// inside the same process; set to 0 to request a random_device seed.
  unsigned long long random_seed{0x9E3779B97F4A7C15ULL};

  // ═══════════════════════════════════════════════════════════════════════
  // Lift-and-project disjunctive cuts (CGLP)
  //
  // Scope (multi-phase rollout; see
  // /memories/repo/cglp-disjunctive-cuts-roadmap.md for the full plan):
  //
  //   Phase 1 (current): API + options + pipeline gate; stub implementation
  //                      that always returns "no cut generated".
  //   Phase 2: CGLP LP model building (constraint matrices for both sides
  //            of the 0-1 disjunction + normalization row).
  //   Phase 3: LP solve via dual simplex + cut extraction (α, β) with
  //            sign/efficacy validation.
  //   Phase 4: Cut strengthening (lifting, canonicalization, duplicate
  //            rejection) and integration into the global cut pool.
  //   Phase 5: Parallel CGLP generation across top-k fractional binaries;
  //            deployment beyond the root node if warranted.
  //
  // Default OFF — do not flip without finishing Phase 3 and re-validating
  // the unexpected-fault resilience benchmark against Gurobi.
  // ═══════════════════════════════════════════════════════════════════════
  /// Master switch. When false, the CGLP entry point is not invoked at all.
  bool enable_cglp_cuts{false};
  /// Maximum fractional binary variables to run CGLP on per root pass.
  int cglp_max_candidates{5};
  /// Minimum fractionality (|x_j - 0.5| distance from integer) required
  /// for a variable to be a CGLP candidate. Skip near-integer variables.
  double cglp_min_fractionality{0.05};
  /// Minimum efficacy (violation / ||α||) to accept a generated cut.
  /// Cuts below this threshold are discarded as numerically weak.
  double cglp_min_efficacy{1e-4};
  /// Time budget (seconds) for the entire CGLP pass in one root cut round.
  double cglp_time_limit_sec{2.0};
  /// Normalization choice for the CGLP master problem.
  /// 0 = standard (||multiplier vector||_1 = 1),
  /// 1 = alpha-normalized (||α||_1 + |β| = 1),
  /// 2 = trust-region around the incumbent fractional point.
  int cglp_normalization{0};
  /// [Scaffold — Phase 4 global dedup, NOT YET IMPLEMENTED]
  ///
  /// When true, admitted CGLP cuts will also be registered in the shared
  /// CutPool so they survive across root cut rounds and are re-separated at
  /// tree nodes (aligned with the SharedCutPool discipline in bc_parallel.cpp).
  ///
  /// Prerequisites before enabling:
  ///   1. A/B experiment on resilience benchmark confirms CGLP cuts close gap.
  ///   2. Add `add_sparse_rows_to_lp` + `CutPool::add()` calls in
  ///      `add_cglp_cuts_root()` guarded by this flag.
  ///   3. Regression suite passes with new parallel dedup paths.
  ///
  /// Keep FALSE until all three prerequisites are met.
  bool cglp_register_in_cut_pool{false};

  // ═══════════════════════════════════════════════════════════════════════
  // Probing
  // ═══════════════════════════════════════════════════════════════════════
  int    probe_reliability{2};
  int    probe_max_candidates{3};
  /// @brief At large low-fractionality roots, run a bounded strong-branching
  /// pass and use max_j min(LB(j=0), LB(j=1)) as a valid global integer
  /// lower-bound floor.  This is domain-agnostic split-bound strengthening.
  bool   root_split_bound_probing{true};
  int    root_split_bound_probe_max{128};
  double root_split_bound_probe_time_limit{1.5};
  double good_incumbent_gap{0.10};

  // ═══════════════════════════════════════════════════════════════════════
  // Pool cut separation
  // ═══════════════════════════════════════════════════════════════════════
  int pool_cut_row_threshold{500};
  int pool_cut_depth_limit{3};
  /// P7.4: Only scan the pool every N nodes (default 1 = every node).
  /// Set to 2 to halve violation-scan FLOPs with minimal impact on cut
  /// utilisation (cuts applied one iteration later at most).
  int pool_cut_scan_interval{1};

  // ═══════════════════════════════════════════════════════════════════════
  // Gap stagnation detection (tree phase)
  // ═══════════════════════════════════════════════════════════════════════
  /// Stagnation is diagnostic only by default.  A generic MILP solver may stop
  /// on time/node/gap/tree exhaustion, but a flat gap is not a proof of
  /// optimality and must not cut off post-incumbent queue certification.
  int    stall_node_window{0};
  double stall_gap_min_improvement{0.05};

  // ═══════════════════════════════════════════════════════════════════════
  // P8: Root phase enhancements (HiGHS-inspired)
  // ═══════════════════════════════════════════════════════════════════════
  /// P8.1: Spawn a background thread immediately before the root LP solve
  /// that computes an approximate analytic centre of the LP polytope via
  /// IPM with zero cost and no crossover.  The result is joined before the
  /// root heuristic phase and used by P8.2 linesearch rounding.
  /// Requires use_ipm_root = true.  Disabled by default.
  bool use_analytic_centre{false};

  /// P8.2: After the root LP (and analytic-centre thread join), slide a
  /// convex combination  x(α) = (1-α)*x_relax + α*x_centre  from α=0 to
  /// α=1, round integers at each step, and attempt feasibility repair.
  /// Falls back to plain linesearch between x_relax and rounded_x when no
  /// analytic centre is available.  Disabled by default.
  bool use_linesearch_rounding{false};

  /// P8.4: Bias simple rounding using LP column lock counts (HiGHS-style):
  ///   uplocks[j]==0  → always ceil  (zero up-locks: can only go up freely)
  ///   downlocks[j]==0 → always floor (zero down-locks: can only go down freely)
  ///   else           → nearest integer  (unchanged from baseline)
  /// Disabled by default.
  bool use_lock_count_rounding{false};

  /// Automatically enable the implemented HiGHS-style root incumbent pipeline
  /// on large generic roots: analytic-centre background LP, line-search
  /// rounding, and lock-count biased rounding. The trigger is based only on
  /// problem size, not on benchmark/domain metadata.
  bool auto_highs_root_pipeline{true};

  /// [D] Disable HiGHS symmetry detection (Nauty/Nauty-like).  UC/SCUC
  /// variables are not interchangeable (distinct cost curves, ramp rates,
  /// min up/down times), so symmetry detection finds nothing useful while
  /// adding Nauty preprocessing overhead.  Set to true to re-enable.
  bool highs_mip_detect_symmetry{false};

  /// [C] Root MIP heuristic effort fraction passed to HiGHS
  /// (mip_heuristic_effort).  HiGHS default is 0.05; 0.15 gives RENS and
  /// central rounding enough budget to find a good incumbent without
  /// significantly delaying the first B&B node.
  double highs_mip_heuristic_effort{0.15};

  /// [E] Pseudocost reliability threshold (mip_pscost_minreliable).  HiGHS
  /// default is 8; a value of 3 reaches statistical reliability faster,
  /// reducing expensive strong-branching LP solves at shallow nodes.
  int highs_mip_pscost_minreliable{3};

  /// [H] Maximum B&B nodes processed while the best bound has not improved
  /// (stall detection).  0 = disabled (HiGHS default).  When a good incumbent
  /// is injected via improvement A/B, setting this to 2000–5000 allows early
  /// termination on problems where the dual gap cannot be closed within
  /// the node budget, returning the best incumbent instead of timing out.
  int highs_mip_max_stall_nodes{0};

  /// [K1] Improvement (1): Extra HiGHS primal heuristics.
  /// ZI-Round and Shifting are cheap rounding heuristics that HiGHS disables
  /// by default but that frequently find good UC incumbents quickly.  They
  /// share the LP relaxation with the rest of root processing, so the cost is
  /// dominated by a few rounding passes per heuristic call.
  ///   highs_mip_run_zi_round: maps to HiGHS option mip_heuristic_run_zi_round.
  ///   highs_mip_run_shifting: maps to HiGHS option mip_heuristic_run_shifting.
  /// Both default to false here to preserve the previous strict-HiGHS contract;
  /// make_strict_highs_production_options() turns them on for the production
  /// StrictHiGHS adapter.
  bool highs_mip_run_zi_round{false};
  bool highs_mip_run_shifting{false};

  /// [K2] Improvement (1): Cutpool size knobs.  0 leaves HiGHS' defaults in
  /// place (mip_pool_age_limit=30, mip_pool_soft_limit=10000).  Non-zero
  /// values are passed through.  Use for very degenerate UC LPs where the
  /// default cutpool size is too small to retain effective MIR/cover cuts
  /// across the deep tree.
  int highs_mip_pool_age_limit{0};
  int highs_mip_pool_soft_limit{0};

  /// [K3] Improvement (2): Force HiGHS presolve to "on" (instead of
  /// "choose") for the StrictHiGHS path, and raise the substitution maxfillin
  /// threshold so degenerate UC blocks have more room to collapse.  These
  /// knobs trade a slightly longer root-presolve wall time for a tighter LP
  /// relaxation and stronger dual bound progression in the tree.
  ///   highs_force_presolve_on:               maps to HiGHS option presolve="on".
  ///   highs_presolve_substitution_maxfillin: maps to HiGHS option
  ///       presolve_substitution_maxfillin.  0 leaves the HiGHS default (10).
  /// Defaults preserve previous behaviour; make_strict_highs_production_options()
  /// turns them on for the production StrictHiGHS adapter.
  bool highs_force_presolve_on{false};
  int highs_presolve_substitution_maxfillin{0};

  /// [K4] Improvement (3): Dual-side knobs.  Raising mip_lp_age_limit keeps
  /// dynamic LP cuts in the relaxation for more node visits, tightening the
  /// per-node LP and propagating cuts across more of the tree before the
  /// pool deletion sweep.  Enabling mip_detect_symmetry turns on HiGHS' Nauty
  /// detection so UC time-shift symmetries between identical generators are
  /// folded out of the search tree.
  ///   highs_mip_lp_age_limit: maps to HiGHS option mip_lp_age_limit
  ///       (HiGHS default = 10).  0 means "leave HiGHS default".
  /// (Symmetry detection has its own boolean above: highs_mip_detect_symmetry.)
  int highs_mip_lp_age_limit{0};

  /// [I] LP solver for the MIP root-node LP relaxation (maps to HiGHS
  /// mip_lp_solver).  When no basis is available (root node), HiGHS routes
  /// the solve as follows:
  ///   "ipm"    — IPX/HiPO interior-point method; crossover behaviour is
  ///              controlled by highs_mip_root_crossover below.  Use for
  ///              large or highly degenerate UC LPs.
  ///   "choose" — Let HiGHS decide; currently defaults to simplex everywhere.
  ///              Default; safe for all problem sizes.
  /// When a basis IS available (every sub-tree node LP), HiGHS always uses
  /// dual simplex regardless of this setting.
  std::string highs_mip_lp_solver{"choose"};

  /// [I-xover] Crossover behaviour when mip_lp_solver="ipm".
  ///   "on"    — Standard IPM → crossover → simplex-basis handoff (default).
  ///             Crossover drives the interior point to a vertex so all
  ///             sub-tree node LPs can be warm-started with dual simplex.
  ///   "off"   — Pure IPM, no crossover.  Fastest for highly degenerate LPs
  ///             (e.g., 118-bus UC) where crossover itself incurs O(10k–100k)
  ///             degenerate simplex pivots.  No basis handoff: node LPs are
  ///             solved from scratch (IPM or simplex per mip_lp_solver).
  ///   "choose"— Let HiGHS decide (currently treated as "on").
  /// Has no effect when mip_lp_solver="choose" (simplex root, no IPM).
  std::string highs_mip_root_crossover{"on"};

  /// [I-fallback] Iteration cap for the first StrictHiGHS root simplex LP.
  /// When positive, the cap is passed to HiGHS' simplex_iteration_limit for
  /// the first root LP only.  If simplex reaches the cap, HiGHS' existing MIP
  /// relaxation logic falls back to IPM, obtains an IPM/crossover basis, and
  /// re-solves simplex from that basis.  0 leaves HiGHS' default infinite root
  /// simplex limit unchanged.
  int highs_mip_root_simplex_iteration_limit{0};

  /// Production StrictHiGHS policy: when the caller has left
  /// highs_mip_lp_solver="choose", route large root LP relaxations through
  /// HiGHS IPM.  Crossover remains controlled by highs_mip_root_crossover;
  /// the production default is "on" so the tree receives a simplex basis for
  /// following node LP kernels.  Explicit caller choices for highs_mip_lp_solver
  /// are respected and are not overwritten by this auto policy.
  bool highs_strict_auto_ipm_root_for_large_models{true};
  int highs_strict_auto_ipm_root_min_cols{10000};
  int highs_strict_auto_ipm_root_min_rows{10000};
  /// Upper cap on the auto root-IPM band.  Above this size the IPM root +
  /// crossover policy is *disabled* and the root LP stays on simplex.
  /// Rationale: on very large degenerate UC roots (e.g. IEEE 118-bus 24T,
  /// 26400 cols / ~35600 rows) the IPM crossover produces a strictly worse
  /// primal incumbent (167069 vs 159108, gap 15.8% vs 11.3%) for no dual-bound
  /// benefit, so simplex-root is preferred there.  Models in the band
  /// [min_cols, max_cols) keep the IPM-root policy.
  int highs_strict_auto_ipm_root_max_cols{20000};
  int highs_strict_auto_ipm_root_max_rows{20000};
  /// Fixed floor on the time budget required to activate auto root-IPM.
  double highs_strict_auto_ipm_root_min_time_sec{30.0};
  /// Size-proportional time scaling: require at least
  ///   max(min_time_sec, n_cols * secs_per_kcol / 1000)
  /// seconds of budget before engaging auto root-IPM.
  /// Rationale: crossover cost grows roughly linearly with LP column count
  /// (empirically ~728 pivots for 26k cols vs ~6 pivots for 6k cols).  For
  /// small problems (39-bus, 6360 cols) this evaluates to 12.7s < 30s floor,
  /// so the floor dominates.  For large problems (118-bus, 26400 cols) it
  /// evaluates to 52.8s > 30s floor, correctly requiring a larger budget.
  double highs_strict_auto_ipm_root_secs_per_kcol{2.0};

  /// [J] Native IPM crash-basis seeding for StrictHiGHS.
  /// When true, before calling Highs::run() on the MIP, the StrictHiGHS path
  /// runs a bounded auxiliary HiGHS IPM LP solve (with crossover disabled) on
  /// the root LP relaxation, recovers a native SimplexBasis from the IPM
  /// primal/dual partition signal,
  /// converts it to a HighsBasis (alien=true, useful=true), and injects it via
  /// Highs::setBasis() with label "MIPSOLVERS native IPM crash root basis".
  /// HiGHS MIP can then repair/dual-simplex warm-start the root LP from this
  /// basis instead of beginning from the default logical root basis.  Designed
  /// for large, highly degenerate UC LPs (e.g., 118-bus 24T) where HiGHS root
  /// crossover incurs O(30k+) pivots.
  /// Default off (opt-in); set to true to evaluate.
  bool highs_strict_seed_native_ipm_basis{false};

  /// Time budget for the auxiliary root-LP IPM solve used by
  /// highs_strict_seed_native_ipm_basis.  The StrictHiGHS path caps the seed
  /// solve by this value and by 25% of the caller's MIP time limit, then gives
  /// the remaining wall time back to Highs::run().
  double highs_strict_seed_native_ipm_basis_time_limit_sec{5.0};

  /// P8: Maximum IPM iterations for the analytic-centre sub-solve.
  int analytic_centre_max_iter{200};

  // ═══════════════════════════════════════════════════════════════════════
  // P9: Large-scale competitiveness (Forrest--Tomlin + symmetry breaking)
  // ═══════════════════════════════════════════════════════════════════════
  /// P9.1: Enable Forrest--Tomlin incremental LU updates in the dual
  /// simplex.  When true, sets simplex_factor_backend = 1
  /// (BackendB_HiGHSSafe) throughout cut LP / node LP / push / cleanup
  /// dispatches.  Requires CMake build flag
  /// MIPSOLVERS_ENABLE_FACTOR_BACKEND_B=ON; otherwise the backend id is
  /// silently clamped to 0 (Backend A) at compile time.  Disabled by
  /// default because of measured drift in certain MIP paths.
  bool use_forrest_tomlin_updates{false};

  /// P9.2: Orbit-based symmetry breaking.  At the root node, detect
  /// orbits of identical binary columns (matching cost, bounds, and full
  /// constraint-column vector) and append sum-lex ordering inequalities
  /// that preserve at least one optimum.  Disabled by default and additionally
  /// gated by enable_domain_heuristics in the legacy B&C path because current
  /// production thresholds were tuned on UC/SCUC benchmarks.
  bool use_symmetry_breaking{false};

  /// P9.2: Minimum orbit size to add lex cuts (must be >= 2; orbits of
  /// size 1 are trivially symmetric-free).
  int symmetry_min_orbit_size{2};

  /// P9.2: Maximum orbits to break.  Bounds the detection cost on very
  /// large instances.  Orbits are sorted by descending size so the
  /// largest (highest-yield) orbits are broken first.
  int symmetry_max_orbits{64};

  // ═══════════════════════════════════════════════════════════════════════
  // Internal: recursive sub-MIP flag
  // ═══════════════════════════════════════════════════════════════════════
  bool _is_stage3_submip{false};

  // ═══════════════════════════════════════════════════════════════════════
  // [K6] Root cut warm-start
  // ═══════════════════════════════════════════════════════════════════════
  /// When non-null, inject these pre-computed root cuts into the HiGHS model
  /// before calling Highs::run().  The cuts come from a previous solve of the
  /// same (or structurally identical) problem and are in the **original** LP
  /// column space (see BCRootCuts).  HiGHS presolve processes them normally;
  /// cuts that are made redundant by bounds are dropped, while tight cuts
  /// carry over and let HiGHS skip most of the root cutting loop, saving
  /// ~40s on large UC instances.
  std::shared_ptr<const BCRootCuts> highs_root_cut_warm_start;
  /// Optional simplex basis paired with highs_root_cut_warm_start.  When the
  /// row count matches the HiGHS model after cut injection, HiGHS maps this
  /// original-space basis through presolve and starts the augmented root LP
  /// from a warm vertex instead of a crash basis.
  std::shared_ptr<const BCRootBasis> highs_root_basis_warm_start;
  /// When >0, cap the number of HiGHS root-node cut separation rounds.
  /// Automatically set to 50 when highs_root_cut_warm_start is non-null
  /// and this field is 0 (i.e. not explicitly overridden).
  int highs_max_root_sepa_rounds{0};
  /// When non-null, seed the HiGHS pseudocost table with branching data
  /// learned during a previous solve of the same (or structurally identical)
  /// problem.  This lets the warm run make near-optimal branching decisions
  /// from the very first node, rather than spending ~50 nodes learning them.
  std::shared_ptr<const BCPseudocostInit> highs_pseudocost_warm_start;

  // ═══════════════════════════════════════════════════════════════════════
  // High-performance auto-tuning knobs (2026-Q2)
  // ═══════════════════════════════════════════════════════════════════════
  // Problem classification thresholds
  int huge_problem_n_threshold{90000};
  /// P7.7: Lowered from 10000→8000 and binary threshold from 2000→500 to
  /// include the 118-bus SCUC (m=10524, n_bin=1296) in the UC-like bypass
  /// that skips feasibility pump and progressive rounding.  Analysis shows
  /// that for any UC instance with tight ramp constraints (m_presolved>8000,
  /// n_bin>=500), rounding a fractional commitment schedule almost certainly
  /// produces an infeasible dispatch subproblem, so pump iterations are wasted.
  int large_uc_row_threshold{8000};
  int large_uc_binary_threshold{500};
  int large_uc_force_ipm_row_threshold{25000};
  int large_uc_force_ipm_bin_threshold{1000};
  int near_integral_n_threshold{10000};
  int near_integral_max_frac_bins{32};

  // SCUC-like auto-tuning
  int scuc_min_binaries{500};
  int scuc_min_frac_binaries{15};
  int scuc_min_rows{2000};
  int scuc_max_rows{20000};
  int scuc_min_cut_rounds{10};
  int scuc_min_cuts_per_round{30};

  // Size-stratified cut budgets
  int cut_budget_xlarge_row_threshold{35000};
  int cut_budget_very_large_row_threshold{15000};
  int cut_budget_large_row_threshold{12000};
  int cut_budget_medium_row_threshold{2000};
  int cuts_per_round_very_large{30};
  int cuts_per_round_large{40};
  int cuts_per_round_medium{60};
  // Generic large-root guard: if only a small fraction of binary variables are
  // fractional, full root cut scans and fixed-integer IPM repairs often cost
  // more than they return.  This is intentionally domain-agnostic.
  double large_generic_low_frac_ratio{0.06};
  int    large_generic_low_frac_abs{256};

  // IPM node-LP and cut re-solve budgets
  int    ipm_node_max_iter{200};
  double ipm_node_tol_multiplier{0.1};
  int    ipm_cut_resolve_row_threshold{10000};
  int    ipm_cut_resolve_xlarge_threshold{25000};

  // Uniform numerical tolerances
  double heuristic_feasibility_tol{1e-6};
  double simplex_tol_floor{1e-10};

  // Pump/repair wall budgets (large UC-like)
  double pump_wall_budget_cap_large{2.5};
  double pump_wall_budget_ratio_large{0.08};
  double pump_wall_budget_cap_default{6.0};
  double pump_wall_budget_ratio_default{0.10};
  double pump_stage3_budget_cap_large{3.0};
  double pump_stage3_budget_ratio_large{0.10};
};

/// Resolve a user-facing num_threads value into the concrete count.
/// If requested < 1, returns the hardware concurrency clamped to
/// [min_threads, max_threads] (where 0 means "no upper cap").
/// Otherwise returns requested unchanged.
int resolve_num_threads(int requested, int min_threads = 2, int max_threads = 0);

/// Convenience: resolve from a BCOptions object using its auto-parallel caps.
int resolve_num_threads(const BCOptions& opt);

}  // namespace mipsolvers::engine

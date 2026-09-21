/// @file bc_utils_ext.hpp
/// @brief Extended B&C separator/proof/propagation declarations (part 2 of the bc_utils.hpp split).
/// Included transitively via bc_utils.hpp; do not include directly.

#pragma once

#include "mipsolvers/engine/detail/bc_utils_core.hpp"

#include <functional>
#include <thread>
#include <utility>

namespace mipsolvers::engine::detail {

/// Join-on-destruction ownership for B&C background tasks.
/// ISO C++20 [thread.thread.destr]: destroying a joinable std::thread calls
/// std::terminate, so every early return of the owning scope must join first.
/// Destructor join makes that invariant structural ([thread.jthread]); crash
/// record and derivation in
/// docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
/// "Root Background-Task Ownership".
class ScopedJoinThread {
 public:
  ScopedJoinThread() = default;
  explicit ScopedJoinThread(std::thread t) : thread_(std::move(t)) {}
  ScopedJoinThread(const ScopedJoinThread&) = delete;
  ScopedJoinThread& operator=(const ScopedJoinThread&) = delete;
  ScopedJoinThread(ScopedJoinThread&& other) noexcept
      : thread_(std::move(other.thread_)) {}
  ScopedJoinThread& operator=(ScopedJoinThread&& other) {
    join();
    thread_ = std::move(other.thread_);
    return *this;
  }
  ~ScopedJoinThread() { join(); }
  bool joinable() const { return thread_.joinable(); }
  void join() {
    if (thread_.joinable()) thread_.join();
  }

 private:
  std::thread thread_;
};

/// @brief Add sparse rows (cuts) to an LP model's inequality system.
struct SeparatorStorageStats;
void add_sparse_rows_to_lp(LPModel& lp,
                            const std::vector<Eigen::SparseVector<double>>& rows,
                            const std::vector<double>& rhs_vals,
                            SeparatorStorageStats* storage_stats = nullptr);

/// @brief Check if x satisfies all constraints of lp within tolerance.
bool satisfies_lp(const LPModel& lp, const Eigen::VectorXd& x, double tol);

/// @brief Compute an estimate for a node from fractional variables and pseudocost history.
double compute_node_estimate(const std::vector<VariableMeta>& vars,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation =
                                 NodeEstimateAggregation::Sum);

/// Compute a node estimate by scanning only precomputed branchable columns.
/// This avoids an O(number-of-all-columns) pass at every B&B node when a large
/// model has comparatively few integer variables.
double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation =
                                 NodeEstimateAggregation::Sum);

double compute_node_estimate(const std::vector<int>& branchable_cols,
                             const Eigen::VectorXd& x,
                             const CompactPseudoCostTable& pc,
                             double int_tol,
                             double node_bound,
                             NodeEstimateAggregation aggregation =
                                 NodeEstimateAggregation::Sum);

/// @brief Append or strengthen a branch-domain reason on a node.
void append_branch_reason(Node& node,
                          int var_idx,
                          bool is_lb,
                          double value);

/// @brief Build a conservative binary no-good cut from a node domain.
/// Returns false if the reason trail is not a pure binary fixing conflict.
bool try_build_binary_conflict_cut(const LPModel& base_lp,
                                   const std::vector<BranchDomainLiteral>& reasons,
                                   PoolCut& out_cut,
                                   int max_literals = 64);

/// @brief Push a node onto the appropriate queue (PQ or DFS stack).
void push_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
               std::vector<Node>& dfs,
               const Node& node,
               NodeSelection mode,
               bool has_incumbent);

/// @brief Pop the next node from the queue.
/// @return true if a node was popped, false if queue is empty.
bool pop_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
              std::vector<Node>& dfs,
              NodeSelection mode,
              bool has_incumbent,
              Node& out);

/// @brief Compute the minimum lower bound across all live nodes.
double live_node_lower_bound(const std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
                             const std::vector<Node>& dfs);

// ── Branching functions (bc_branching.cpp) ───────────────────────────────

/// @brief Select branch variable by most-infeasible rule.
int choose_branch_var_most_infeasible(const std::vector<int>& cand,
                                      const Eigen::VectorXd& x);

/// @brief Select branch variable by pseudo-cost scoring (product rule).
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc);

/// @brief Select branch variable by pseudo-cost scoring with priority weighting.
/// Priority values > 0 give a soft multiplicative bonus to the pseudocost score.
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority);

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const CompactPseudoCostTable& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context);

/// @brief Select branch variable by pseudo-cost, static priority, and a dynamic prior callback.
int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context);

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const std::vector<PseudoCost>& pc,
                                const std::vector<int>* priority = nullptr);

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const CompactPseudoCostTable& pc,
                                const std::vector<int>* priority = nullptr);

/// Score one direction using current-node evidence when it is usable, falling
/// back to the directional pseudocost prediction for absent/failed evidence.
double compute_directional_branch_score(
    const PseudoCost& pseudocost,
    double branch_distance,
    bool is_up,
    const BranchDirectionalObservation* observation = nullptr,
    double prediction_offset = 0.0);

/// Product score shared by dense, compact, sequential, and parallel selectors.
double compute_branch_product_score(
    const PseudoCost& pseudocost,
    double fractionality,
    const BranchDirectionalObservation* down_observation = nullptr,
    const BranchDirectionalObservation* up_observation = nullptr);

/// Choose which child direction to evaluate first. Without an incumbent the
/// smaller predicted/observed child-bound lift is primal-oriented; with an
/// incumbent a uniquely proven cutoff takes precedence.
bool choose_up_branch_first(
    const PseudoCost& pseudocost,
    double fractionality,
    const BranchDirectionalObservation* down_observation,
    const BranchDirectionalObservation* up_observation,
    bool has_incumbent,
    bool existing_prefer_up = false,
    int advisory_direction = 0);

BranchEstimatorCalibration compute_branch_estimator_calibration(
    double parent_bound,
    double parent_estimate,
    bool down_prediction_available,
    double predicted_down_gain,
    bool up_prediction_available,
    double predicted_up_gain,
    bool down_bound_available,
    double down_bound,
    bool up_bound_available,
    double up_bound);

/// Apply one physical probe result to global history at most once. Unknown
/// failures make no update; certified cutoffs update only discrete history.
void update_pseudocost_from_branch_evidence(
    PseudoCost& pseudocost,
    bool is_up,
    BranchEvidenceState state,
    double gain,
    double branch_distance,
    bool certified_conflict = false);

/// Ordered signature of the active local-cut rows. Coefficients, right-hand
/// sides, order, and row-relevant flags all contribute to the signature.
std::uint64_t ordered_local_cut_signature(const std::vector<PoolCut>& cuts);

/// Classify whether a probe can replace the child LP solve, can only seed a
/// re-solve, or is structurally incompatible with the closed child state.
BranchProbeReuseKind classify_branch_probe_reuse(
    bool available,
    bool consumed,
    int probe_var,
    bool probe_is_up,
    int branch_var,
    bool child_is_up,
    const Eigen::VectorXd& probe_lb,
    const Eigen::VectorXd& probe_ub,
    const Eigen::VectorXd& child_lb,
    const Eigen::VectorXd& child_ub,
    std::uint64_t probe_row_signature,
    std::uint64_t child_row_signature,
    std::uint64_t probe_model_epoch,
    std::uint64_t child_model_epoch,
    std::uint64_t probe_objective_epoch,
    std::uint64_t child_objective_epoch,
    bool proof_usable,
    bool warm_state_usable,
    double tolerance = 1e-9);

double compute_branch_var_score_with_overlay(
    int j,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& pc,
    const std::vector<BranchDirectionalObservation>& observations);

double compute_branch_var_score_with_overlay(
    int j,
    const Eigen::VectorXd& x,
    const CompactPseudoCostTable& pc,
    const std::vector<BranchDirectionalObservation>& observations);

int choose_branch_var_pseudocost_with_overlay(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& pc,
    const std::vector<int>& priority,
    const std::vector<BranchDirectionalObservation>& observations,
    const BCBranchingPriorFn& dynamic_prior = {},
    const BCBranchContext& context = {});

/// Select from a candidate-aligned pseudocost snapshot. This avoids copying a
/// full model-sized table in parallel workers that only read fractional cols.
int choose_branch_var_pseudocost_candidates(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const std::vector<PseudoCost>& candidate_pc,
    const std::vector<int>& priority,
    const BCBranchingPriorFn& dynamic_prior = {},
    const BCBranchContext& context = {},
    const std::vector<BranchDirectionalObservation>& observations = {});

int choose_branch_var_pseudocost_with_overlay(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    const CompactPseudoCostTable& pc,
    const std::vector<int>& priority,
    const std::vector<BranchDirectionalObservation>& observations,
    const BCBranchingPriorFn& dynamic_prior = {},
    const BCBranchContext& context = {});

/// @brief Select branch variable using the configured strategy.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc);

/// @brief Select branch variable using the configured strategy with priority.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority);

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const CompactPseudoCostTable& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context);

/// @brief Select branch variable using static priorities and a dynamic prior callback.
int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context);

/// @brief Reliability branching: probe unreliable candidates with LP solves.
/// @details For candidates with insufficient pseudo-cost data (< reliability_limit
/// observations), solves child LPs to get real estimates. Only probes at depth <= 3.
/// @param cand Candidate fractional variable indices.
/// @param x Current LP relaxation solution.
/// @param pc Pseudo-cost table (updated in-place with probe results).
/// @param base_lp Base LP model.
/// @param base_sf Base standard-form LP for bound updates.
/// @param node_lb Current node lower bounds.
/// @param node_ub Current node upper bounds.
/// @param basis_hint Simplex basis for warm-starting probes.
/// @param simplex_opt Simplex solver options.
/// @param parent_bound Parent node's LP bound (for gain computation).
/// @param[in,out] lp_solves Counter incremented for each probe LP solve.
/// @param node_depth Current depth in the B&C tree.
/// @param reliability_limit Minimum observations before trusting pseudo-costs.
/// @param max_probes Maximum strong-branching probes per node.
/// @return Index of selected branching variable.
struct StrongBranchProbeStorageStats {
  std::uint64_t base_sf_materializations{0};
  std::uint64_t bound_transactions{0};
  std::uint64_t transaction_snapshot_values{0};
  std::uint64_t transaction_rollbacks{0};
  std::uint64_t transaction_failures{0};
  std::uint64_t backend_cold_solves{0};
  std::uint64_t backend_persistent_resolves{0};
};

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    std::vector<PseudoCost>& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit = 1,
    int max_probes = 4,
    const std::vector<int>& priority = {},
    const BCBranchingPriorFn& dynamic_prior = {},
    const BCBranchContext& context = {},
    std::vector<BranchDirectionalObservation>* selected_observations = nullptr,
    StrongBranchProbeStorageStats* storage_stats = nullptr);

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    CompactPseudoCostTable& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit = 1,
    int max_probes = 4,
    const std::vector<int>& priority = {},
    const BCBranchingPriorFn& dynamic_prior = {},
    const BCBranchContext& context = {},
    std::vector<BranchDirectionalObservation>* selected_observations = nullptr,
    StrongBranchProbeStorageStats* storage_stats = nullptr);

// ── Cut generation functions (bc_cuts.cpp) ───────────────────────────────

struct SeparatorStorageStats {
  std::uint64_t sparse_candidates_created{0};
  std::uint64_t sparse_candidate_entries_created{0};
  std::uint64_t peak_live_sparse_candidates{0};
  std::uint64_t peak_live_sparse_entries{0};
  std::uint64_t sparse_aggregation_snapshots{0};
  std::uint64_t sparse_aggregation_entries{0};
  std::uint64_t dense_workspace_materializations{0};
  std::uint64_t dense_workspace_values{0};
  std::uint64_t matrix_append_calls{0};
  std::uint64_t matrix_appended_rows{0};
  std::uint64_t matrix_appended_entries{0};
  std::uint64_t matrix_prior_entries_bypassing_triplet_rebuild{0};
  std::uint64_t matrix_storage_reallocations{0};
  std::uint64_t matrix_peak_spare_entries{0};

  void merge(const SeparatorStorageStats& other) {
    sparse_candidates_created += other.sparse_candidates_created;
    sparse_candidate_entries_created +=
        other.sparse_candidate_entries_created;
    peak_live_sparse_candidates =
        std::max(peak_live_sparse_candidates,
                 other.peak_live_sparse_candidates);
    peak_live_sparse_entries =
        std::max(peak_live_sparse_entries,
                 other.peak_live_sparse_entries);
    sparse_aggregation_snapshots += other.sparse_aggregation_snapshots;
    sparse_aggregation_entries += other.sparse_aggregation_entries;
    dense_workspace_materializations += other.dense_workspace_materializations;
    dense_workspace_values += other.dense_workspace_values;
    matrix_append_calls += other.matrix_append_calls;
    matrix_appended_rows += other.matrix_appended_rows;
    matrix_appended_entries += other.matrix_appended_entries;
    matrix_prior_entries_bypassing_triplet_rebuild +=
        other.matrix_prior_entries_bypassing_triplet_rebuild;
    matrix_storage_reallocations += other.matrix_storage_reallocations;
    matrix_peak_spare_entries =
        std::max(matrix_peak_spare_entries, other.matrix_peak_spare_entries);
  }
};

/// @brief Dispatch cut generation based on BCOptions::cuts.
/// @details Calls the appropriate cut family (GMI, MIR, cover, or all) within
/// a budget. Cuts are appended as new inequality rows to lp.
/// @param[in,out] lp LP model to add cuts to.
/// @param x Current fractional LP solution.
/// @param simplex Simplex result (needed for tableau-based cuts); nullptr to skip GMI.
/// @param opt B&C options controlling cut types and quality thresholds.
/// @param max_cuts Maximum cuts to generate.
/// @param sbasis Sparse basis factorization for BTRAN (optional, for GMI).
/// @param tracker Optional per-family efficacy tracker for adaptive cut generation.
/// @return Number of cuts added to the LP.
int add_cuts(LPModel& lp,
             const Eigen::VectorXd& x,
             const SimplexResult* simplex,
             const BCOptions& opt,
             int max_cuts,
             const std::shared_ptr<BasisOps>& sbasis = nullptr,
             CutFamilyTracker* tracker = nullptr,
             const class CliqueTable* clique_table = nullptr,
             SeparatorStorageStats* storage_stats = nullptr);

/// @brief Generate Gomory Mixed-Integer cuts from the simplex basis.
/// @details Constructs GMI cuts from the bounded-form simplex tableau for each
/// fractional basic integer variable. Filters by density, efficacy, activity,
/// parallelism, and binary support ratio. Batch-inserts selected cuts.
/// @param[in,out] lp LP model to add cuts to.
/// @param x Current fractional LP solution.
/// @param simplex Simplex result with basis and tableau data.
/// @param opt B&C options (GMI quality thresholds).
/// @param max_cuts Maximum cuts to select.
/// @param sbasis Sparse basis factorization for BTRAN (optional).
/// @return Number of cuts added.
int add_basis_gomory_cuts(LPModel& lp,
                          const Eigen::VectorXd& x,
                          const SimplexResult& simplex,
                          const BCOptions& opt,
                          int max_cuts,
                          const std::shared_ptr<BasisOps>& sbasis = nullptr,
                          SeparatorStorageStats* storage_stats = nullptr);

/// @brief Generate HiGHS-style transformed tableau CMIR cuts at the root.
/// @details Aggregates LP rows with a basis-inverse row, converts the
/// aggregate into the bounded shifted standard space, generates a CMIR cut,
/// untransforms through the exact standard-form row coefficients, and appends
/// only rows violated by the current original-space LP solution.
int add_transformed_tableau_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::shared_ptr<BasisOps>& sbasis = nullptr,
	    const std::vector<char>* implied_integer_cols = nullptr,
	    const BinaryImplicationGraph* implication_graph = nullptr,
	    const VariableBoundTable* variable_bound_table = nullptr,
	    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
	    double cut_generation_feastol = 1e-8,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
	    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt,
        SeparatorStorageStats* storage_stats = nullptr);

/// @brief Generate HiGHS-style transformed path aggregation CMIR cuts.
/// @details Starts from active row sides, projects through short row paths that
/// cancel continuous columns in transformed space, then uses the same bounded
/// transformed cut generator and exact untransform as transformed tableau cuts.
int add_transformed_path_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
	    const std::vector<char>* implied_integer_cols = nullptr,
	    const BinaryImplicationGraph* implication_graph = nullptr,
	    const VariableBoundTable* variable_bound_table = nullptr,
	    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
	    double cut_generation_feastol = 1e-8,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
	    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt,
	    const std::function<int(int)>* highs_path_randint = nullptr,
        SeparatorStorageStats* storage_stats = nullptr);

/// @brief Generate HiGHS-style mod-k cuts from transformed active rows.
/// @details Mirrors HighsModkSeparator: skip rows containing continuous
/// variables with nonzero transformed bound distance, build a GF(k) row system
/// from active integral rows, and add generated cuts directly to the cutpool.
int add_transformed_modk_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    const std::vector<char>* implied_integer_cols = nullptr,
    const BinaryImplicationGraph* implication_graph = nullptr,
    const VariableBoundTable* variable_bound_table = nullptr,
    std::vector<PoolCut>* generated_cutpool_rows = nullptr,
    double cut_generation_feastol = 1e-8,
    const std::function<int(PoolCut&&)>* cutpool_acceptor = nullptr,
    std::optional<std::uint64_t> highs_cutgen_seed = std::nullopt,
    SeparatorStorageStats* storage_stats = nullptr);

// ── Relaxation functions (bc_relaxation.cpp) ─────────────────────────────

/// @brief Solve the LP relaxation at a B&C node.
/// @details Uses simplex (if opt.use_simplex_lp_nodes) or IPM fallback.
/// Returns primal solution, dual bound, and basis hint for warm-starting children.
/// @param lp LP model with node-specific bounds already applied.
/// @param x0 Optional starting point (for IPM path).
/// @param basis_hint Simplex basis from parent node (for warm-start).
/// @param opt B&C options.
/// @return Primal result, dual bound, and basis hint.
LPRelaxationResult solve_lp_relaxation(const LPModel& lp,
                                       const Eigen::VectorXd* x0,
                                       const SimplexBasis* basis_hint,
                                       const BCOptions& opt);

/// @brief Recover a crash basis from a primal point, optionally constrained by
/// IPM dual/complementarity signals and seeded from an already valid basis.
/// @details When seed_basis is provided, only rows currently using
/// slack/artificial columns are considered for structural swaps. This keeps a
/// known-valid basis intact and only repairs the newly introduced rows.
CrashBasisRecoveryStats recover_primal_activity_basis(const StandardFormLP& sf,
                                                      const Eigen::VectorXd& x_orig,
                                                      const SolveResult* ipm_res,
                                                      const SimplexBasis* seed_basis,
                                                      SimplexBasis& out_basis);

/// @brief Solve the NLP relaxation at a B&C node (for MINLP).
/// @param nlp_base Base NLP model (bounds updated per node).
/// @param lb Node lower bounds.
/// @param ub Node upper bounds.
/// @param x0 Optional starting point.
/// @param opt B&C options.
/// @return Primal solution and convergence info.
SolveResult solve_nlp_relaxation(const NLPModel& nlp_base,
                                 const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub,
                                 const Eigen::VectorXd* x0,
                                 const BCOptions& opt);

/// @brief Lightweight rounding heuristic for child nodes.
/// @return true if a new incumbent was found.
bool try_rounding_heuristic(const LPModel& base_lp,
                            const Eigen::VectorXd& x_relax,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub,
                            double int_tol,
                            bool& has_incumbent,
                            double& incumbent_obj,
                            Eigen::VectorXd& incumbent_x,
                            SolutionPool* sol_pool = nullptr);

/// @brief Check if x satisfies all constraints using explicit bounds (avoids LPModel copy).
/// @details Uses base_lp constraint matrices but checks bounds from node_lb/node_ub.
bool satisfies_with_bounds(const LPModel& lp,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& node_lb,
                           const Eigen::VectorXd& node_ub,
                           double tol);

/// @brief Fix integer variables whose reduced costs prove they can't improve the incumbent.
/// @details After solving LP at a node, for each non-basic integer variable j:
///   If j is at lower bound and rc[j] >= gap: fix j at lb (ub[j] = lb[j])
///   If j is at upper bound and |rc[j]| >= gap: fix j at ub (lb[j] = ub[j])
/// Variables whose primal value is not on the reduced-cost-implied bound side
/// are skipped; basis membership alone is not a sufficient side certificate.
/// @return Number of variables fixed.
int reduced_cost_fixing(const std::vector<VariableMeta>& vars,
                        const Eigen::VectorXd& x_relax,
                        const Eigen::VectorXd& reduced_costs,
                        const std::vector<int>& basis_indices,
                        double node_bound,
                        double incumbent_obj,
                        double int_tol,
                        Eigen::VectorXd& node_lb,
                        Eigen::VectorXd& node_ub,
                        std::vector<BranchDomainLiteral>* forbidden_literals = nullptr,
                        const Eigen::VectorXd* sf_col_scale = nullptr,
                        std::vector<DomainReasonBound>* reason_bounds = nullptr,
                        const std::vector<BranchDomainLiteral>* reason_frontier = nullptr,
                        int reason_depth = 0,
                        int reason_trail_offset = 0);

/// @brief Full LP row-dual objective-cutoff proof row.
///
/// For the effective minimization LP and incumbent cutoff U, the row is
///     coeff^T x <= rhs
/// with
///     coeff = c - A^T y,   rhs = U - b^T y,
/// where y is recovered from the simplex basis row duals and mapped back to
/// the original LP rows. This is the native analogue of HiGHS
/// HighsLpRelaxation::computeDualProof().
struct DualProofRow {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
  double min_activity{0.0};
  double max_activity{0.0};
  double lp_activity{0.0};
  double expected_lp_gap{0.0};
  double actual_lp_gap{0.0};
  double gap_error{0.0};
  double max_stationarity_error{0.0};
  double max_scaled_stationarity_error{0.0};
  double max_cost_mapping_error{0.0};
  int nnz{0};
  int row_dual_nnz{0};
  int ignored_row_duals{0};
  int removed_bound_coefficients{0};
  int tightened_coefficients{0};
  bool used_solver_row_duals{false};
  bool valid{false};
  std::string reject_reason;
};

/// @brief Build a HiGHS-style full row-dual cutoff proof from a simplex result.
bool build_full_dual_proof_row(const LPModel& lp,
                               const SimplexResult& simplex,
                               const Eigen::VectorXd& root_lb,
                               const Eigen::VectorXd& root_ub,
                               const Eigen::VectorXd& x_lp,
                               double lp_objective,
                               double incumbent_obj,
                               DualProofRow& out,
                               double tol = 1e-8,
                               const StandardFormLP* form_override = nullptr);

/// @brief Reduced-cost-mass objective-cutoff proof row.
///
/// This is a stricter source row for conflict/reconvergence target selection:
/// it keeps only non-basic integer original columns whose reduced-cost
/// coefficient is active away from the global/root side.  All other
/// coefficients are shifted to the safe root-bound side, so the resulting
/// row remains a valid local objective-cutoff proof while exposing frontier
/// terms that carry actual reduced-cost mass.
bool build_reduced_cost_mass_dual_proof_row(
    const LPModel& lp,
    const SimplexResult& simplex,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_lp,
    double lp_objective,
    double incumbent_obj,
    DualProofRow& out,
    double tol = 1e-8,
    const StandardFormLP* form_override = nullptr,
    const std::vector<char>* implied_integer_cols = nullptr,
    const std::vector<char>* force_keep_cols = nullptr);

/// @brief Consume a cutoff dual proof as direct domain fixing.
///
/// For a valid proof row `coeff^T x <= rhs`, if the minimum activity over the
/// current local domain already exceeds `rhs`, the node is cutoff.  Otherwise
/// each coefficient gives the strongest one-variable bound that can still keep
/// the proof row feasible.  This is the proof-row analogue of HiGHS
/// reduced-cost/objective propagation: first obtain a real LP certificate, then
/// turn it into local bound changes before re-solving the node LP.
///
/// @return -1 if the domain is cutoff, otherwise the number of bounds changed.
int apply_dual_proof_domain_fixing(
    const std::vector<VariableMeta>& vars,
    const DualProofRow& proof,
    double int_tol,
    double lp_tol,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    std::vector<BoundChangeInfo>* changes_out = nullptr);

/// @brief Build a shortened reduced-cost cutoff conflict from the LP proof row.
///
/// The returned clause is a proof-cover version of
/// `branch_path + forbidden_literal`: using the current node reduced costs, it
/// keeps only branch-domain bounds whose local-bound activity is sufficient to
/// make the forbidden literal violate the incumbent cutoff row.  This is the
/// native analogue of HiGHS resolving a dual/objective proof against domain
/// reasons; if the proof cover cannot be certified, the function returns false
/// and callers should fall back to the conservative full-path clause.
bool build_reduced_cost_cutoff_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    double node_bound,
    double incumbent_obj,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const BranchDomainLiteral& forbidden,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin = nullptr,
    std::vector<std::vector<BranchDomainLiteral>>* reconvergence_clauses = nullptr,
    PoolCut* violated_local_proof_cover = nullptr,
    const DualProofRow* dual_proof = nullptr);

/// @brief Structured output for target-bound row-dual explanation.
///
/// HiGHS stores reconvergence cuts as `flip(target)` plus the resolved proof
/// frontier.  Keeping these fields separate lets callers admit true binary
/// implications only when the frontier itself resolves to one literal.
struct DualProofTargetExplanation {
  bool valid{false};
  bool cutoff_conflict{false};
  bool has_proved_target_bound{false};
  BranchDomainLiteral proved_target_bound;
  BranchDomainLiteral flipped_target;
  std::vector<BranchDomainLiteral> initial_frontier;
  std::vector<BranchDomainLiteral> resolved_frontier;
  std::vector<BranchDomainLiteral> clause;
  double proof_margin{0.0};
  double proof_budget{0.0};
  double frontier_lp_activity{0.0};
  double local_proof_cover_violation{0.0};
  int local_proof_cover_nnz{0};
  bool has_proof_activity_audit{false};
  double proof_activity{0.0};
  double proof_required_activity{0.0};
};

/// @brief Explain a generic target bound change with a full row-dual proof.
///
/// This is the native analogue of HiGHS
/// HighsDomain::conflictAnalyzeReconvergence(domchg, proof row): a local
/// target bound is proved from the objective-cutoff row using only prior
/// branch/propagation reasons, then emitted as a reason-side frontier plus the
/// flipped target literal.
bool build_dual_proof_target_bound_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin = nullptr,
    PoolCut* violated_local_proof_cover = nullptr,
    const DualProofRow* dual_proof = nullptr,
    DualProofTargetExplanation* explanation = nullptr,
    std::vector<PoolCut>* additional_local_proof_covers = nullptr,
    const DualProofRow* priority_proof = nullptr);

/// @brief Propagate bound changes through constraints to tighten other variable bounds.
/// @details Performs constraint-based bound tightening at tree nodes.
/// @return Number of bounds tightened.
int node_bound_propagation(const LPModel& lp,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds = 2);

/// @brief Overload accepting pre-computed row-major sparse matrices to avoid
/// per-call format conversion overhead in the B&C loop.
int node_bound_propagation(const LPModel& lp,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds = 2);

/// @brief Tracked variant: records every bound change as BoundChangeInfo.
/// Used for incremental standard-form updates (avoids full O(n) scan).
int node_bound_propagation_tracked(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    std::vector<BoundChangeInfo>& changes_out);

/// @brief Persistent binary implication graph built from root probing.
/// @details Stores implications of the form `(x_j = 0/1) => bound(k)`.
/// Propagation is event-driven: binary literals fixed by branching, clique
/// propagation, or row tightening are pushed into a queue, and all reachable
/// implications are applied until closure.
class BinaryImplicationGraph {
 public:
    struct Implication {
        int var_idx{-1};
        double value{0.0};
        bool is_lb{false};
    };

    BinaryImplicationGraph() = default;
    explicit BinaryImplicationGraph(int n_vars) { reset(n_vars); }

    void reset(int n_vars);
    bool empty() const { return num_implications_ == 0; }
    std::size_t size() const { return num_implications_; }

    bool add_implication(int trigger_var,
                                             bool trigger_value_one,
                                             int implied_var,
                                             bool implied_is_lb,
                                             double implied_value);

    std::pair<const Implication*, const Implication*> implications(int trigger_var,
                                                                   bool trigger_value_one) const {
        if (trigger_var < 0 || trigger_var >= n_vars_ || adjacency_.empty()) {
            return {nullptr, nullptr};
        }
        const auto& bucket = adjacency_[literal_index(trigger_var, trigger_value_one)];
        return {bucket.data(), bucket.data() + bucket.size()};
    }

    bool implies_binary_value(int trigger_var,
                              bool trigger_value_one,
                              int implied_var,
                              bool implied_value_one,
                              double tol = 1e-9) const {
        auto range = implications(trigger_var, trigger_value_one);
        for (const Implication* p = range.first; p != range.second; ++p) {
            if (p == nullptr || p->var_idx != implied_var) continue;
            if (implied_value_one && p->is_lb && p->value >= 1.0 - tol) {
                return true;
            }
            if (!implied_value_one && !p->is_lb && p->value <= tol) {
                return true;
            }
        }
        return false;
    }

    int propagate(const std::vector<VariableMeta>& vars,
	                                Eigen::VectorXd& lb,
	                                Eigen::VectorXd& ub,
	                                std::vector<BoundChangeInfo>* changes_out = nullptr,
	                                double tol = 1e-9) const;

 private:
    static std::size_t literal_index(int var_idx, bool value_one) {
        return static_cast<std::size_t>(2 * var_idx + (value_one ? 1 : 0));
    }

    int n_vars_{0};
    std::size_t num_implications_{0};
    std::vector<std::vector<Implication>> adjacency_;
};

/// @brief First-class variable-bound table, mirroring HiGHS' VUB/VLB store.
/// @details Stores source relations of the form
/// `x_col <= coef * y_trigger + constant` (VUB) and
/// `x_col >= coef * y_trigger + constant` (VLB), where `y_trigger` is a
/// binary trigger.  These objects are stronger source material than the
/// downstream implications obtained by evaluating the row at y=0 and y=1:
/// transformed tableau/path separation can substitute the whole affine bound.
class VariableBoundTable {
 public:
    struct VarBound {
        double coef{0.0};
        double constant{0.0};

        double min_value() const {
            return constant + std::min(coef, 0.0);
        }
        double max_value() const {
            return constant + std::max(coef, 0.0);
        }
    };

    struct Entry {
        int trigger_col{-1};
        VarBound bound;
    };

    struct SourceStats {
        std::uint64_t vub_attempts{0};
        std::uint64_t vub_accepted{0};
        std::uint64_t vub_replaced{0};
        std::uint64_t vlb_attempts{0};
        std::uint64_t vlb_accepted{0};
        std::uint64_t vlb_replaced{0};
        std::uint64_t mir_attempts{0};
        std::uint64_t mir_strengthened{0};
        std::uint64_t redundant{0};
    };

    VariableBoundTable() = default;
    explicit VariableBoundTable(int n_vars) { reset(n_vars); }

    void reset(int n_vars);
    bool empty() const { return num_varbounds_ == 0; }
    std::size_t size() const { return num_varbounds_; }
    int num_vars() const { return n_vars_; }
    const SourceStats& stats() const { return stats_; }

    const std::vector<Entry>& vubs(int col) const;
    const std::vector<Entry>& vlbs(int col) const;

    bool add_vub(int col, int trigger_col, double coef, double constant,
                 double col_upper_bound, bool col_is_integral);
    bool add_vlb(int col, int trigger_col, double coef, double constant,
                 double col_lower_bound, bool col_is_integral);

    std::uint64_t export_to_implication_graph(
        BinaryImplicationGraph& implication_graph,
        double tol = 1e-9) const;

 private:
    bool valid_indices(int col, int trigger_col) const {
        return col >= 0 && trigger_col >= 0 && col < n_vars_ &&
               trigger_col < n_vars_ && col != trigger_col;
    }

    static bool strengthen_var_bound(VarBound& bound, int multiplier);

    int n_vars_{0};
    std::size_t num_varbounds_{0};
    std::vector<std::vector<Entry>> vubs_;
    std::vector<std::vector<Entry>> vlbs_;
    SourceStats stats_;
};

inline int NodeQueue::prune_by_implications(
    const std::vector<VariableMeta>& vars,
    const BinaryImplicationGraph& implication_graph,
    const ConflictPool* conflict_pool,
    bool has_incumbent,
    double incumbent_obj,
	    double tol,
	    std::uint64_t* tightened_out,
	    double bound_band,
	    int max_nodes,
	    const Eigen::VectorXd* root_lb,
	    const Eigen::VectorXd* root_ub) {
    if (tightened_out != nullptr) *tightened_out = 0;
    if (implication_graph.empty()) return 0;
    if (nodes_.empty()) return 0;
    DomainMaterializationGuard domain_guard(*this);
    std::vector<std::int64_t> doomed;
    doomed.reserve(nodes_.size());

    std::vector<std::int64_t> candidate_ids;
    if (std::isfinite(bound_band) || max_nodes > 0) {
        std::set<std::int64_t> seen_ids;
        const double queue_lb = lower_bound();
        const double threshold =
            std::isfinite(queue_lb) && std::isfinite(bound_band)
                ? queue_lb + std::max(0.0, bound_band)
                : kInf;
        auto collect_ids = [&](const std::multiset<QueueKey>& source) {
            for (auto it = source.begin(); it != source.end(); ++it) {
                if (std::isfinite(threshold) && it->first > threshold) break;
                if (seen_ids.insert(it->second).second) {
                    candidate_ids.push_back(it->second);
                    if (max_nodes > 0 &&
                        static_cast<int>(candidate_ids.size()) >= max_nodes) {
                        break;
                    }
                }
            }
        };
        collect_ids(bounds_);
        if (max_nodes <= 0 ||
            static_cast<int>(candidate_ids.size()) < max_nodes) {
            collect_ids(dfs_bounds_);
        }
    } else {
        candidate_ids.reserve(nodes_.size());
        for (const auto& [id, node] : nodes_) {
            (void)node;
            candidate_ids.push_back(id);
        }
    }

    for (std::int64_t id : candidate_ids) {
        auto node_find = nodes_.find(id);
        if (node_find == nodes_.end()) continue;
        Node& node = node_find->second;
        if (has_incumbent &&
            incumbent_prunes_node(true, incumbent_obj, node.bound)) {
            doomed.push_back(id);
            continue;
        }
        Eigen::VectorXd propagated_lb = node.lb;
        Eigen::VectorXd propagated_ub = node.ub;
        std::vector<BoundChangeInfo> changes;
        implication_graph.propagate(vars, propagated_lb, propagated_ub,
                                    &changes, tol);
        if (!bounds_consistent(propagated_lb, propagated_ub) ||
            (conflict_pool != nullptr &&
             conflict_pool->has_conflict(propagated_lb, propagated_ub))) {
            doomed.push_back(id);
            continue;
        }
        // HiGHS keeps propagation objects live in the domain.  Do the same for
        // queued native nodes: root/global implications are valid in every
        // descendant, so carrying the tightened domain forward lets subsequent
        // certificate lower-bound passes and the eventual node LP see the same
        // first-class bounds instead of rediscovering them later.
        if (!changes.empty()) {
            const auto index_membership = detach_queue_indices(id, node);
            node.lb = std::move(propagated_lb);
            node.ub = std::move(propagated_ub);
            if (node.x_seed.size() == node.lb.size()) {
                node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
            }
            node.lp_refresh_needed = true;
            rebuild_local_domain_trail(
                node, static_cast<int>(node.lb.size()), root_lb, root_ub);
            if (!refresh_domain_signature_after_update(id)) {
                erase_detached_node(id);
                continue;
            }
            reattach_queue_indices(id, index_membership);
            if (tightened_out != nullptr) {
                *tightened_out += static_cast<std::uint64_t>(changes.size());
            }
        }
    }
    for (std::int64_t id : doomed) {
        erase_id(id);
    }
    return static_cast<int>(doomed.size());
}

/// @brief Variable-to-row index for event-driven bound propagation.
/// @details Instead of rescanning all rows after every bound change, the
/// propagator revisits only rows incident to variables whose bounds changed.
class RowPropagationIndex {
 public:
    void build(int n_vars,
               const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
               const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row);

    const std::vector<int>& ineq_rows_for(int var_idx) const {
        static const std::vector<int> kEmpty;
        if (var_idx < 0 || var_idx >= static_cast<int>(ineq_rows_by_var_.size())) return kEmpty;
        return ineq_rows_by_var_[static_cast<std::size_t>(var_idx)];
    }

    const std::vector<int>& eq_rows_for(int var_idx) const {
        static const std::vector<int> kEmpty;
        if (var_idx < 0 || var_idx >= static_cast<int>(eq_rows_by_var_.size())) return kEmpty;
        return eq_rows_by_var_[static_cast<std::size_t>(var_idx)];
    }

 private:
    std::vector<std::vector<int>> ineq_rows_by_var_;
    std::vector<std::vector<int>> eq_rows_by_var_;
};

struct PropagationProfile {
    std::uint64_t calls{0};
    std::uint64_t active_passes{0};
    std::uint64_t changed_var_visits{0};
    std::uint64_t ineq_rows_visited{0};
    std::uint64_t eq_rows_visited{0};
    std::uint64_t baseline_full_scan_rows{0};
    std::uint64_t bound_tightenings{0};
    std::uint64_t reason_clauses_materialized{0};
    /// Cumulative wall-time spent in propagation (nanoseconds).
    std::uint64_t wall_ns{0};
};

/// Identifies the mechanism that produced a node-domain bound change.
enum class DomainPropagationSource {
    Unknown,
    ConflictPool,
    Clique,
    Implication,
    InequalityRow,
    EqualityRow,
};

const char* domain_propagation_source_name(
    DomainPropagationSource source) noexcept;

/// Optional audit record for one bound change made by unified propagation.
/// The production path does not materialize these records unless requested.
struct DomainPropagationEvent {
    DomainPropagationSource source{DomainPropagationSource::Unknown};
    int variable{-1};
    int row{-1};
    int trigger_variable{-1};
    bool trigger_value_one{false};
    bool is_lb{false};
    double old_value{0.0};
    double new_value{0.0};
    double coefficient{0.0};
    double rhs{0.0};
    double activity_bound{0.0};
    std::vector<BranchDomainLiteral> source_clause;
};

/// Optional audit detail for the propagation event that made a domain
/// inconsistent. A pool conflict with no final bound change may have no
/// variable index.
struct DomainPropagationFailure {
    bool available{false};
    DomainPropagationEvent event;
};

void reset_propagation_profile();
PropagationProfile get_propagation_profile();

/// @brief Derive binary-trigger implications from a learned conflict clause.
/// @details For a 2-literal conflict `(l_1 \land l_2)` infeasible, every
/// binary literal among `l_1,l_2` can trigger the negation of the other bound.
/// If both literals are binary, both directed implications are inserted.
bool add_binary_implications_from_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const std::vector<BranchDomainLiteral>& clause,
    BinaryImplicationGraph& implication_graph);

/// @brief Unified fixed-point node propagation engine.
/// @details Replays learned conflict clauses, then alternates clique-table
/// implications, cached binary implications, and one round of row-based bound
/// tightening until no new bound change is produced. This is the in-tree
/// domain propagation hot path used before every node LP solve.
bool propagate_node_domain(
        const LPModel& lp,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
        const RowPropagationIndex& row_index,
        Eigen::VectorXd& node_lb,
        Eigen::VectorXd& node_ub,
        int max_rounds,
        const ConflictPool* conflict_pool,
        const CliqueTable* clique_table,
        const BinaryImplicationGraph* implication_graph,
        std::vector<BoundChangeInfo>& changes_out,
        const std::vector<BranchDomainLiteral>& branch_reasons = {},
        std::vector<BranchDomainLiteral>* learned_conflict = nullptr,
        int* total_tightened = nullptr,
        std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
        const std::vector<DomainReasonBound>* existing_reason_bounds = nullptr,
        std::vector<DomainPropagationEvent>* propagation_events = nullptr,
        DomainPropagationFailure* propagation_failure = nullptr,
        const std::function<bool()>* stop_requested = nullptr,
        bool* interrupted = nullptr);

bool propagate_node_domain(
        const LPModel& lp,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
        const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
        const RowPropagationIndex& row_index,
        Eigen::VectorXd& node_lb,
        Eigen::VectorXd& node_ub,
        int max_rounds,
        const SharedConflictPool* conflict_pool,
        const CliqueTable* clique_table,
        const BinaryImplicationGraph* implication_graph,
        std::vector<BoundChangeInfo>& changes_out,
        const std::vector<BranchDomainLiteral>& branch_reasons = {},
        std::vector<BranchDomainLiteral>* learned_conflict = nullptr,
        int* total_tightened = nullptr,
        std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
        const std::vector<DomainReasonBound>* existing_reason_bounds = nullptr,
        std::vector<DomainPropagationEvent>* propagation_events = nullptr,
        DomainPropagationFailure* propagation_failure = nullptr,
        const std::function<bool()>* stop_requested = nullptr,
        bool* interrupted = nullptr);

}  // namespace mipsolvers::engine::detail

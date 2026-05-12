#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

class Highs;

namespace mipsolvers::engine {

struct SimplexBasis;  // forward declaration
struct SimplexOptions;
struct StandardFormLP;
struct SimplexResult;
struct SparseFactorTelemetry;

enum class BasisOpsKind {
  NativeSparse = 0,
  VendoredHighs = 1,
};

/// First-class standard-form basis operations used by cut/proof code.
///
/// Implementations must live in the exact same canonical StandardFormLP column
/// and row space as the SimplexResult that owns them.  Native SparseBasis uses
/// the in-house LU/FT stack; VendoredHighsBasis delegates to HiGHS' basis
/// inverse/tableau APIs while retaining the solved HiGHS instance.
struct BasisOps {
  virtual ~BasisOps() = default;
  virtual BasisOpsKind kind() const = 0;
  virtual Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const = 0;
  virtual Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const = 0;
  /// Return row i of B^{-1} in the owner LP row space.  HiGHS' tableau
  /// separator uses this exact object as rowEp before aggregation.
  virtual bool basis_inverse_row(int row, Eigen::VectorXd& out) const = 0;
  virtual bool basis_inverse_row_sparse_entries(
      int,
      std::vector<std::pair<int, double>>&) const {
    return false;
  }
  virtual bool tableau_row(int row, Eigen::RowVectorXd& out) const = 0;
  virtual void clear_etas() {}
  virtual void truncate_etas_to(int) {}
  virtual int eta_count() const { return 0; }
  virtual int generation() const { return -1; }
  virtual SparseFactorTelemetry factor_telemetry() const;
  virtual void rebind_A(const Eigen::SparseMatrix<double>&) {}
  virtual bool bound_to_A(const Eigen::SparseMatrix<double>&) const {
    return false;
  }
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
  virtual std::shared_ptr<Highs> highs_handle() const { return nullptr; }
#endif
  virtual bool delete_rows_cols_and_resolve(
      const StandardFormLP&,
      const SimplexBasis*,
      const std::vector<int>&,
      const std::vector<int>&,
      const SimplexOptions&,
      SimplexResult&) {
    return false;
  }
};

// Basis-factor backend selector for dual simplex SparseBasis internals.
//
// BackendA_UmfpackNative: current production path (UMFPACK numeric + native
// sparse triangular solves; FT update path available but currently gated off in
// production policy).
// BackendB_HiGHSSafe / ForceFT / ShortChain: experimental HiGHS-like FT-first
// modes that differ only in how aggressively they keep FT active before
// refactorization or eta fallback.
enum class SimplexFactorBackend {
  BackendA_UmfpackNative = 0,
  BackendB_HiGHSSafe = 1,
  BackendB_ForceFT = 2,
  BackendB_ShortChain = 3,
};

inline SimplexFactorBackend simplex_factor_backend_from_id(int id) {
  switch (id) {
    case 3:
      return SimplexFactorBackend::BackendB_ShortChain;
    case 2:
      return SimplexFactorBackend::BackendB_ForceFT;
    case 1:
      return SimplexFactorBackend::BackendB_HiGHSSafe;
    case 0:
    default:
      return SimplexFactorBackend::BackendA_UmfpackNative;
  }
}

struct SparseFactorTelemetry {
  int backend_id{0};
  bool ft_backend{false};
  bool ft_valid{false};
  int ft_updates{0};
  int eta_updates{0};
  double min_pivot{0.0};
  double max_growth{1.0};
  double u_fill_ratio{1.0};
  int refactor_generation{0};
};

struct SimplexOptions {
  int max_iter{2000};
  double feasibility_tol{1e-8};
  double optimality_tol{1e-8};
  bool prefer_dual_simplex_reopt{true};
  bool verbose{false};
  // When false, skip expensive cold-start for large problems when warm-start
  // fails.  Set to false for tree node LPs (B&C prunes instead).
  // Keep true for root LP / root cut re-solves where cold-start is required.
  bool allow_cold_start{true};
  // Pointer to atomic incumbent bound for early termination in parallel B&C.
  // When non-null, simplex checks every incumbent_check_interval iterations and
  // aborts if obj >= *incumbent_bound (node will be pruned by caller).
  const std::atomic<double>* incumbent_bound{nullptr};
  int incumbent_check_interval{50};
  // When true, primal simplex uses sector-based partial pricing for large LPs
  // (n > 2000).  Disable for crossover cleanup where full pricing produces
  // a more consistent basis for downstream Gomory cut generation.
  bool use_partial_pricing{true};
  // When true, sparse primal simplex applies the legacy 1e-6 primal
  // perturbation to degenerate basic variables.  The first-class root LP path
  // keeps this false and relies on deterministic pivot tie-breaking instead,
  // since perturbing x_B changes the degenerate vertex/frontier consumed by
  // cut and proof generation.
  bool perturb_degenerate_primal{false};
  // Fallback basis (e.g., root basis) to try warm-starting from before
  // expensive cold-start.  When PATH_A fails with the primary hint, this
  // basis is tried next.  Set by B&C dispatcher for tree node LPs.
  const SimplexBasis* fallback_basis{nullptr};
  // When true, reuse the cached SparseBasis LU factorization from the hint
  // without refactorizing.  Safe when only bounds/RHS changed (A unchanged).
  // Saves ~30ms per LP at m≈14000 by skipping UMFPACK numeric factorization.
  bool reuse_factorization{false};
  // When true, initialize dual steepest-edge weights exactly from the current
  // basis by BTRAN instead of starting from unit weights when no persisted DSE
  // state is available.  This is expensive, so callers use it only for
  // root-LP conformance and HiGHS-style DSE/edge-weight diagnostics.
  bool exact_dse_initialization{false};
  // When true, perform the guarded zero-step remap of degenerate original
  // basic variables onto logical columns after optimality.  This is separate
  // from DSE initialization: xpool append/refactor conformance should compare
  // the native pivot state directly, while the first-class root LP path may
  // explicitly conform the degenerate frontier consumed by cut generation.
  bool enable_degenerate_frontier_remap{false};
  // Explicitly suppress post-optimal degenerate frontier remap even when
  // global conformance diagnostics are enabled.  Xpool append/refactor/full
  // A/B solves use this so the consume gate observes the raw native pivot
  // frontier instead of a repeatedly conformed diagnostic state.
  bool suppress_degenerate_frontier_remap{false};
  // Opt-in for root cut-pool resolves where the caller has appended only
  // <= rows to a scale-compatible standard form and made the new row slacks
  // basic.  In that case the new basis is block triangular around the parent
  // basis, so SparseBasis can reuse the parent factor for FTRAN/BTRAN instead
  // of rebuilding a large factorization.
  bool allow_incremental_row_append_factor{false};
  // Use the vendored HiGHS simplex kernel for this already-built canonical
  // standard-form LP. HiGHS is used only as an LP kernel; B&C ownership stays
  // in native code. The result is accepted only after the SF audit passes.
  bool allow_vendored_highs_sf_backend{false};
  // When true, do not fall back to native simplex if the vendored SF backend
  // rejects the solve. Root sub-MIP heuristics use this to keep the LP oracle
  // and basis/frontier state identical to the main vendored HiGHS path.
  bool require_vendored_highs_sf_backend{false};
  // Selects which basis-factor backend SparseBasis should use.
  // Defaults to current production backend (A).
  SimplexFactorBackend factor_backend{SimplexFactorBackend::BackendA_UmfpackNative};
};

struct SimplexBasis {
  std::vector<int> indices;
  // Optional shared backing storage for basis indices.
  // When engaged, `indices` may be empty to avoid per-node duplicate buffers.
  std::shared_ptr<const std::vector<int>> shared_indices;
  int rows{0};
  int cols{0};
  // Cached basis inverse from the parent solve.  When present, child nodes
  // can skip the O(m³) B.inverse() and reuse this directly (A is unchanged,
  // only b/bounds differ between parent and child).
  std::shared_ptr<const Eigen::MatrixXd> cached_inverse;
  // Cached reduced costs (depend only on basis/binv/c, not b).
  std::shared_ptr<const Eigen::VectorXd> cached_reduced_costs;
  // Column scale used by cached_reduced_costs. Native simplex stores reduced
  // costs in scaled standard-form maximization convention; HiGHS MIP
  // propagation consumes unscaled minimization col_dual, recovered as
  // col_dual[j] = -reduced_costs[j] / cached_col_scale[j].
  std::shared_ptr<const Eigen::VectorXd> cached_col_scale;
  // Optional live simplex state for first-class root LP debugging.  These are
  // deliberately optional because tree-node basis hints are numerous; root
  // xpool code fills them only when it wants to continue from the same vertex
  // after appending slack-basic rows.
  std::shared_ptr<const Eigen::VectorXd> cached_x_basic;
  std::shared_ptr<const Eigen::VectorXd> cached_x_std;
  double cached_max_objective{0.0};
  bool has_cached_max_objective{false};
  // Non-basic variable status: 1 = at upper bound, 0 = at lower bound.
  std::vector<char> at_upper;
  // Cached max objective from dual variables (c_B^T * B^{-1} contribution
  // that is independent of b, used to compute obj = this->dot(b)).
  double cached_obj_offset{0.0};
  bool has_cached_obj_offset{false};
  // Cached first-class standard-form basis operations.
  // Persisted across B&C node solves to avoid re-factorization when basis
  // indices are unchanged (only bounds differ between parent and child).
  std::shared_ptr<BasisOps> cached_sparse_basis;
  // Eta count at the time cached_sparse_basis was persisted.
  // Siblings can detect mutation: if current eta_count > this, another consumer modified it.
  int persist_eta_count{0};
  // Standard-form column counts from the solve that produced this basis.
  // Used to correctly remap columns when extending the basis for cut rows.
  int sf_n_slack{-1};
  int sf_n_surplus{-1};
  int sf_n_artificial{-1};

  /// Returns basis indices regardless of local/shared storage mode.
  const std::vector<int>& basis_indices() const {
    return shared_indices ? *shared_indices : indices;
  }

  /// Number of basis indices available.
  std::size_t index_count() const {
    return basis_indices().size();
  }

  /// Ensures `indices` owns mutable storage (materializes from shared mode).
  void ensure_owned_indices() {
    if (!shared_indices) return;
    indices = *shared_indices;
    shared_indices.reset();
  }

  /// Compacts local `indices` into shared immutable storage.
  void compact_indices_storage() {
    if (shared_indices || indices.empty()) return;
    shared_indices = std::make_shared<const std::vector<int>>(std::move(indices));
    indices.clear();
    indices.shrink_to_fit();
  }

  /// If basis indices are identical, reuse the same shared backing buffer.
  void try_share_indices_from(const SimplexBasis& other) {
    const auto& mine = basis_indices();
    const auto& theirs = other.basis_indices();
    if (mine.size() != theirs.size() || mine != theirs) return;
    if (other.shared_indices) {
      shared_indices = other.shared_indices;
    } else {
      shared_indices = std::make_shared<const std::vector<int>>(theirs);
    }
    indices.clear();
    indices.shrink_to_fit();
  }
};

struct StandardFormLP {
  Eigen::SparseMatrix<double> A;                        // column-major (fast col access)
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row;   // row-major (fast row access for GMI)
  Eigen::VectorXd b;
  Eigen::VectorXd c_max;
  Eigen::VectorXd lb_shift;
  // Upper bounds for each column in shifted space (var_ub[j] = ub_j - lb_shift_j).
  // +inf for unbounded variables and slack/surplus/artificial columns.
  Eigen::VectorXd var_ub;
  double objective_const{0.0};
  int n_original{0};
  int n_slack{0};
  int n_surplus{0};
  int n_artificial{0};
  std::vector<VarType> original_types;
  std::vector<int> row_to_artificial_col;
  std::vector<int> row_to_slack_col;
  std::vector<int> row_to_surplus_col;
  std::vector<int> row_sign;    // +1 or -1: sign flip applied at construction
  // Bound value used to form each standard-form RHS before lb_shift:
  //   row_sign * (row_rhs_value - A_i * lb_shift).
  // For ranged rows oriented from their lower side this is row_lhs[i], not b[i].
  Eigen::VectorXd row_rhs_value;
  std::vector<int> ub_row_var;  // For upper-bound rows: var index; -1 otherwise
  std::vector<int> source_highs_row;  // Native SF row -> HiGHS presolved row.
  std::vector<int> source_row_start;
  std::vector<int> source_row_index;
  std::vector<double> source_row_value;
  // Ruiz equilibration scale factors (populated by ruiz_scale_standard_form).
  // Scaled LP: D_r * A * D_c,  D_r * b,  D_c * c_max,  var_ub / D_c.
  Eigen::VectorXd row_scale;    // length m; empty if unscaled
  Eigen::VectorXd col_scale;    // length n; empty if unscaled
};

struct SimplexResult {
  SolveResult result;
  StandardFormLP form;
  SimplexBasis basis;
  Eigen::MatrixXd basis_inverse;
  // Shared basis inverse (avoids copy when parent basis yields primal feasibility).
  std::shared_ptr<const Eigen::MatrixXd> shared_binv;
  Eigen::VectorXd x_std;
  Eigen::VectorXd x_basic;
  Eigen::VectorXd reduced_costs;
  double max_objective{0.0};
  bool solved_from_hint{false};
  bool dual_reoptimized{false};
  bool exact_optimal{false};
};

StandardFormLP build_standard_form_lp(const LPModel& lp);

// Append <= rows, expressed in original variable space, to an existing
// StandardFormLP while preserving all old row/column scaling.  The appended
// rows keep their logical <= orientation even if the shifted RHS is negative:
// a first-class LP object may start from the resulting primal-infeasible
// slack-basic state and repair it by dual simplex.  This differs deliberately
// from a cold standard-form rebuild, which may flip negative-RHS rows.
bool append_leq_rows_to_standard_form(const StandardFormLP& base_sf,
                                      const std::vector<Eigen::SparseVector<double>>& rows,
                                      const std::vector<double>& rhs,
                                      StandardFormLP& out,
                                      double feasibility_tol = 1e-12);

// Append <= rows using the same canonical row orientation and Ruiz scaling
// policy as a fresh build_standard_form_lp(lp)+ruiz_scale_standard_form(lp)
// rebuild.  This is the first-class root-LP path for cut admission: old rows
// keep their membership/order, new inequality rows are inserted before the
// equality block, negative shifted RHS rows are flipped to >= with
// surplus/artificial columns, and the whole resulting standard form is scaled
// by the native Ruiz policy.
bool append_leq_rows_to_canonical_standard_form(
    const StandardFormLP& base_sf,
    const std::vector<Eigen::SparseVector<double>>& rows,
    const std::vector<double>& rhs,
    StandardFormLP& out,
    int ruiz_rounds = 10,
    double feasibility_tol = 1e-12);

// Apply Ruiz equilibration scaling to a StandardFormLP in-place.
// Iteratively balances row/column infinity norms of A toward 1.0.
// Populates sf.row_scale and sf.col_scale; scales A, A_row, b, c_max, var_ub.
void ruiz_scale_standard_form(StandardFormLP& sf, int rounds = 10);

// Rebuild only the bounds-dependent parts of a StandardFormLP (b, lb_shift,
// objective_const) while reusing the constraint matrix A.  The structural
// flipping decisions (row sign, slack/surplus assignment) are taken from
// base_sf; only numeric values are updated for the new variable bounds.
void update_standard_form_bounds(StandardFormLP& sf,
                                 const LPModel& lp,
                                 const Eigen::VectorXd& node_lb,
                                 const Eigen::VectorXd& node_ub);

// Rewrite only the cost vector (c_max) and objective_const of an existing
// StandardFormLP in place. `new_c` has length == sf.n_original and is
// expressed in the user (LPModel) sense given by `sense`. The constraint
// matrix A, RHS b, bounds var_ub, row/col signs and Ruiz scale factors are
// left untouched, so any SparseBasis / LU factorization cached on a
// SimplexBasis hint referencing this sf remains structurally valid.
//
// Callers reusing a basis hint after a cost change MUST clear its stale
// reduced-cost cache (hint.cached_reduced_costs.reset();
// hint.has_cached_obj_offset = false;) because those depend on c_max.
// After a cost-only change, the basis is generally primal feasible but
// dual-infeasible, so the primal simplex path is the natural re-optimizer
// (pass SimplexOptions::prefer_dual_simplex_reopt = false).
void update_standard_form_cost(StandardFormLP& sf,
                               Sense sense,
                               const Eigen::VectorXd& new_c);

SimplexResult solve_lp_with_basis(const LPModel& lp,
                                  const SimplexOptions& opt = {},
                                  const SimplexBasis* basis_hint = nullptr);

/// Describes a bound change for incremental warm-start in B&C.
struct BoundChangeInfo {
  int var_idx;       ///< Original variable index
  double delta;      ///< Change in bound value (new - old)
  bool is_lb;        ///< true = lower bound changed, false = upper bound changed
};

// Fast-path incremental update when exact bound changes are known.
// Avoids the O(n) scan to detect which bounds changed.
void update_standard_form_bounds_incremental(
    StandardFormLP& sf,
    const LPModel& lp,
    const std::vector<BoundChangeInfo>& changes);

// Solve an LP from a pre-built StandardFormLP (avoids rebuilding from LPModel).
SimplexResult solve_lp_from_sf(const StandardFormLP& sf,
                               const SimplexOptions& opt = {},
                               const SimplexBasis* basis_hint = nullptr,
                               const std::vector<BoundChangeInfo>* bound_changes = nullptr);

// Thread-local B&C integration switch for vendored HiGHS standard-form LPs.
// Returns the previous value so callers can restore it with RAII.
bool set_vendored_highs_sf_backend_thread_enabled(bool enabled);
bool vendored_highs_sf_backend_thread_enabled();

/// Dump and reset per-LP solve counters (warm-start / cold-start statistics).
void dump_solve_lp_counters();

// Perform BTRAN (B^{-T} * rhs) using a type-erased SparseBasis.
// Returns zero vector if cached_sparse_basis is null.
Eigen::VectorXd sparse_basis_btran(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                                   const Eigen::VectorXd& rhs);

/// Return row i of B^{-1}; this is the HiGHS rowEp object used by tableau
/// separation before row aggregation.
bool sparse_basis_inverse_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::VectorXd& out);

bool sparse_basis_inverse_row_sparse_entries(
  const std::shared_ptr<BasisOps>& cached_sparse_basis,
  int row,
  std::vector<std::pair<int, double>>& out);

/// Return a tableau row e_i^T B^{-1} A in the owning StandardFormLP space.
bool sparse_basis_tableau_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::RowVectorXd& out);

/// Clear accumulated eta vectors from a type-erased SparseBasis, restoring
/// it to the state at its last refactorization.  Used by root probing to
/// keep the SparseBasis clean between probes so each probe can warm-start
/// without re-factorization.
void clear_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb);

/// Truncate eta vectors in a type-erased SparseBasis back to a saved count.
/// Used between sibling node solves: save the eta count before child1's solve,
/// then truncate back to that count before child2's solve, so child2 sees the
/// same clean factorization state that child1 started from.
void truncate_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb, int target_count);

/// Return the current eta count of a type-erased SparseBasis.
/// Returns 0 if sb is null.
int sparse_basis_eta_count(const std::shared_ptr<BasisOps>& sb);

/// Return the refactorization generation counter of a type-erased SparseBasis.
/// Returns -1 if sb is null.  Used to detect mid-solve refactorizations:
/// if the generation changes across a solve, clearing etas is unsafe.
int sparse_basis_generation(const std::shared_ptr<BasisOps>& sb);

/// Return the current factor/update telemetry of a type-erased SparseBasis.
/// Returns default-initialized telemetry when sb is null.
SparseFactorTelemetry get_sparse_basis_factor_telemetry(
  const std::shared_ptr<BasisOps>& sb);

/// Rebind a type-erased SparseBasis to a StandardFormLP matrix that is owned
/// by a persistent SimplexResult/form object.  Cached SparseBasis instances
/// store a non-owning matrix reference; callers that persist a simplex result
/// after solving from a stack-local StandardFormLP must rebind before carrying
/// the factor into the next warm start.
void rebind_sparse_basis_matrix(const std::shared_ptr<BasisOps>& sb,
                                const Eigen::SparseMatrix<double>& A);

/// Return true iff the cached SparseBasis is currently bound to this exact
/// matrix object and has matching row dimension.
bool sparse_basis_bound_to_matrix(const std::shared_ptr<BasisOps>& sb,
                                  const Eigen::SparseMatrix<double>& A);

}  // namespace mipsolvers::engine

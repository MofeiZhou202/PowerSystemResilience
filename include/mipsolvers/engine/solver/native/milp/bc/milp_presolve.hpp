/// @file milp_presolve.hpp
/// @brief Comprehensive MILP presolve routines modeled after HiGHS/commercial solvers.
///
/// Implements the following reduction techniques:
///   1. Fixed variable substitution and removal
///   2. Empty row/column removal
///   3. Singleton row (bound tightening) and column (implied free substitution)
///   4. Doubleton equality substitution
///   5. Forcing and dominated rows
///   6. Dominated columns (cost-based fixing)
///   7. Parallel/duplicate row detection
///   8. Coefficient strengthening for MIP
///   9. Probing on binary variables with implication propagation
///
/// All reductions operate in-place on the LPModel via a unified sparse
/// representation that tracks row/column nonzero counts for O(1) singleton
/// detection.  Redundant rows and fixed variables are physically removed
/// when rebuild_model() constructs the reduced LPModel.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Statistics from presolve reductions.
struct PresolveStats {
  int rows_removed{0};
  int cols_removed{0};
  int bounds_tightened{0};
  int coefficients_strengthened{0};
  int singletons_removed{0};
  int doubletons_removed{0};
  int forcing_rows{0};
  int dominated_cols{0};
  int parallel_rows{0};
  int probing_fixings{0};
  int rounds{0};
  double presolve_time_sec{0.0};

  int orig_rows{0}, orig_cols{0}, orig_nnz{0};
  int final_rows{0}, final_cols{0}, final_nnz{0};
};

/// Options controlling presolve behavior.
struct PresolveOptions {
  int max_rounds{20};
  // NOTE: singleton-row, forcing-row and probing reductions are currently
  // UNSOUND on SCUC MIPs (they remove the integer optimum — validated via
  // native_kernel_comparison --check: 39bus_24T 1452678.4038 -> 1455085/1453758)
  // and default OFF pending a soundness fix.  The remaining reductions (fixed,
  // empty, singleton-column, tighten, doubleton, dominated, parallel, coeff
  // strengthen) are validated sound.
  bool do_singleton_rows{false};
  bool do_forcing_rows{false};
  bool do_probing{false};
  int max_probing_candidates{500};
  int probing_depth{3};
  bool do_coefficient_strengthen{true};
  bool do_doubleton{true};
  bool do_parallel_rows{true};
  bool do_dominated{true};
  double zero_tol{1e-10};
  double bound_tol{1e-9};
  bool verbose{false};
};

/// Main MILP presolve class.
///
/// Usage:
///   MILPPresolve ps(opts);
///   auto stats = ps.run(lp, binary_idx, integer_idx);
///   // solve reduced lp ...
///   Eigen::VectorXd x_orig = ps.postsolve(x_reduced);
class MILPPresolve {
 public:
  explicit MILPPresolve(PresolveOptions opts = {});

  /// Run presolve on an LP/MIP model.
  PresolveStats run(LPModel& lp,
                    std::vector<int>& binary_idx,
                    std::vector<int>& integer_idx);

  /// Recover original-space solution from reduced-space solution.
  Eigen::VectorXd postsolve(const Eigen::VectorXd& x_reduced) const;

  /// Forward-map an original-space (or intermediate) solution to reduced space.
  /// Extracts values at active column indices. Eliminated columns are dropped.
  Eigen::VectorXd forward_map(const Eigen::VectorXd& x_input) const;

  const PresolveStats& stats() const { return stats_; }

  /// Column mapping accessors (valid after run()).
  const std::vector<int>& orig_to_reduced_col() const { return orig_to_reduced_col_; }
  const std::vector<int>& reduced_to_orig_col() const { return reduced_to_orig_col_; }

  /// Row mapping accessor (valid after run()).
  /// orig_to_reduced_row()[r] is -1 if row r was eliminated, else the index in
  /// the reduced model.  The reduced model lays rows out as [ineq | eq],
  /// matching the input layout of [A-rows | Aeq-rows].
  const std::vector<int>& orig_to_reduced_row() const { return orig_to_reduced_row_; }

  // ── Internal sparse representation ──
  // Unified format: all constraints stored as  row_lb <= a^T x <= row_ub.
  // Equalities have row_lb == row_ub.  Pure inequalities (Ax<=b) have row_lb = -inf.
  struct Entry { int col; double val; };

 private:

  PresolveOptions opts_;
  PresolveStats stats_;

  int n_orig_{0};
  int m_orig_{0};

  // Per-row data
  std::vector<std::vector<Entry>> rows_;   // row -> list of (col, val)
  std::vector<double> row_lb_;
  std::vector<double> row_ub_;
  std::vector<bool> row_deleted_;

  // Per-column data (transpose index for column operations)
  struct CEntry { int row; double val; };
  std::vector<std::vector<CEntry>> cols_;  // col -> list of (row, val)
  std::vector<double> col_lb_;
  std::vector<double> col_ub_;
  std::vector<double> col_cost_;
  std::vector<VarType> col_type_;
  std::vector<bool> col_deleted_;

  // Snapshot of original column bounds (pre-reduction) for postsolve diagnostics.
  std::vector<double> col_lb_orig_;
  std::vector<double> col_ub_orig_;

  // Cached row activity bounds
  struct ActivityCache {
    double min_act{0.0}, max_act{0.0};
    int n_inf_min{0}, n_inf_max{0};  // count of infinite contributors
  };
  std::vector<ActivityCache> row_activity_;

  // Postsolve undo stack
  enum class UndoType {
    FixedCol,        // col fixed to value
    SubstitutedCol,  // col eliminated by equality (doubleton)
    RedundantRow,    // row removed
  };
  struct UndoRecord {
    UndoType type;
    int col{-1};
    double value{0.0};      // for FixedCol: fixed value
    // for SubstitutedCol: x_col = (rhs - sum a_k * x_k) / pivot_coeff
    int pivot_row{-1};
    double pivot_coeff{0.0};
    double rhs{0.0};
    std::vector<std::pair<int, double>> row_entries;  // other variables in the row
  };
  std::vector<UndoRecord> undo_stack_;

  // Original-to-reduced and reduced-to-original column maps
  std::vector<int> orig_to_reduced_col_;
  std::vector<int> reduced_to_orig_col_;
  std::vector<int> orig_to_reduced_row_;

  // How many ineq rows in the original model (first m_ineq entries = A, rest = Aeq)
  int m_ineq_orig_{0};

  // ── Initialization ──
  void init_from_lp(const LPModel& lp);
  void rebuild_model(LPModel& lp, std::vector<int>& binary_idx,
                     std::vector<int>& integer_idx);
  void compute_all_activities();
  void update_activity(int row);

  // ── Reductions (each returns number of changes) ──
  int remove_fixed_variables();
  int remove_empty_rows_and_cols();
  int process_singleton_rows();
  int process_singleton_columns();
  int process_doubleton_equations();
  int tighten_bounds();
  int detect_forcing_rows();
  int detect_dominated_columns();
  int detect_parallel_rows();
  int strengthen_coefficients();
  int run_probing();

  // ── Helpers ──
  bool is_integer_var(int col) const {
    return col_type_[col] == VarType::Binary || col_type_[col] == VarType::Integer;
  }
  bool is_fixed(int col) const {
    return std::abs(col_ub_[col] - col_lb_[col]) < opts_.bound_tol;
  }
  void fix_variable(int col, double val);
  void delete_row(int row);
  void remove_entry(int row, int col);
  void add_entry(int row, int col, double val);
  double get_entry(int row, int col) const;
};

/// Convenience: run presolve on a MIPModel's linear_part.
/// Physically removes variables and rows; requires postsolve to recover solution.
PresolveStats presolve_mip(LPModel& lp,
                           std::vector<int>& binary_idx,
                           std::vector<int>& integer_idx,
                           const PresolveOptions& opts = {});

/// In-place presolve: tightens bounds, zeros fixed variable columns,
/// and removes redundant rows, but keeps the same variable count.
/// Compatible with B&C which assumes stable variable indices.
PresolveStats presolve_inplace(LPModel& lp, const PresolveOptions& opts = {});

}  // namespace mipsolvers::engine

/// @file papilo_presolve.hpp
/// @brief PaPILO-based presolve wrapper for MILP problems.

#pragma once

#include "mipsolvers/engine/problem_types.hpp"

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <vector>

namespace mipsolvers::engine {

/// Opaque storage for PaPILO postsolve data.
struct PaPILOPostsolveData;

/// O(nnz) numerical-conditioning summary of an LPModel's constraint data.
/// Used to compare matrix scaling before/after presolve (P0(b) diagnostics:
/// the PaPILO-presolved 118-bus SCUC root LP is harder for the native
/// simplex Phase I than the unpresolved one; these stats attribute why).
struct MatrixScalingStats {
  int rows{0};
  int cols{0};
  long long nnz{0};
  /// Smallest / largest nonzero |A| entry (0 when the matrix is empty).
  double abs_min{0.0};
  double abs_max{0.0};
  /// abs_max / abs_min; 1 when empty.
  double dynamic_range{1.0};
  int max_row_nnz{0};
  int max_col_nnz{0};
  /// Rows whose intra-row |coefficient| dynamic range exceeds 1e8.
  int rows_with_large_range{0};
  /// Row with the largest intra-row range (combined index: inequality rows
  /// first, then equality rows; -1 when no nonzeros) and that range.
  int worst_row{-1};
  double worst_row_range{1.0};
  /// Largest |RHS| over b, finite row_lhs, and beq; largest |c|.
  double b_abs_max{0.0};
  double c_abs_max{0.0};
};

/// Compute MatrixScalingStats for an LPModel in one O(nnz) sweep.
MatrixScalingStats compute_matrix_scaling_stats(const LPModel& lp);

/// Result of PaPILO presolve including mappings for postsolve.
struct PaPILOPresolveResult {
  LPModel reduced_lp;          ///< Reduced problem (fewer rows/cols)
  std::vector<int> binary_idx; ///< Binary variable indices in reduced
  std::vector<int> integer_idx;///< Integer variable indices in reduced

  /// Column mapping: reduced col j → original col
  std::vector<int> reduced_to_orig_col;
  /// Column mapping: original col j → reduced col (or -1 if eliminated)
  std::vector<int> orig_to_reduced_col;
  /// Row mapping for diagnostics: native reduced inequality row -> original row.
  std::vector<int> reduced_ineq_to_orig_row;
  /// True when a native reduced inequality row is the negated lower side of
  /// a PaPILO row.
  std::vector<char> reduced_ineq_negated;
  /// Row mapping for diagnostics: native reduced equality row -> original row.
  std::vector<int> reduced_eq_to_orig_row;

  /// Presolve statistics
  int orig_rows{0}, orig_cols{0}, orig_nnz{0};
  int reduced_rows{0}, reduced_cols{0}, reduced_nnz{0};
  /// Constant objective contribution introduced by eliminated columns, in
  /// the solver's internal minimization convention.
  double objective_offset{0.0};
  /// Number of active PaPILO rows before the native LPModel expands finite
  /// lower/upper row sides into separate <= rows.
  int reduced_compact_rows{0};
  /// Active non-equation rows with both finite sides.  Each contributes one
  /// extra <= row in the current native LPModel representation.
  int reduced_two_sided_rows_expanded{0};
  /// Native reduced inequality rows that keep a finite lower side.
  int reduced_rows_with_finite_lhs{0};
  double presolve_time_sec{0.0};

  /// Total bound changes and coefficient changes detected during presolve.
  /// Non-zero values indicate the instance still had work to do and may
  /// benefit from a follow-up run with an aggressive profile.
  int total_bnd_chg{0};
  int total_coef_chg{0};

  bool success{false};
  bool infeasible{false};

  /// Matrix-conditioning snapshots of the input and reduced models
  /// (after_stats only populated on a successful reducing run).
  MatrixScalingStats before_stats;
  MatrixScalingStats after_stats;

  /// Opaque postsolve data (PaPILO's PostsolveStorage + Num)
  std::shared_ptr<PaPILOPostsolveData> postsolve_data;
};

/// Diagnostic summary for checking a native active-column solution against
/// PaPILO's final compressed problem before postsolve undo.
struct PaPILOReducedValidationSummary {
  bool available{false};
  bool size_ok{true};
  int expected_active_size{0};
  int actual_size{0};
  int compressed_cols{0};
  int compressed_rows{0};
  int fixed_cols_populated{0};

  bool col_bounds_ok{true};
  int worst_col{-1};
  int worst_orig_col{-1};
  double worst_col_value{0.0};
  double worst_col_lb{-std::numeric_limits<double>::infinity()};
  double worst_col_ub{std::numeric_limits<double>::infinity()};
  double max_col_violation{0.0};

  bool rows_ok{true};
  int worst_row{-1};
  int worst_orig_row{-1};
  bool worst_row_eq{false};
  const char* worst_row_side{"none"};
  double worst_row_activity{0.0};
  double worst_row_lhs{-std::numeric_limits<double>::infinity()};
  double worst_row_rhs{std::numeric_limits<double>::infinity()};
  double max_row_violation{0.0};
  int worst_row_nonactive_terms{0};
  double worst_row_nonactive_abs_activity{0.0};

  bool ok() const {
    return available && size_ok && col_bounds_ok && rows_ok;
  }
};

/// One term in an exact affine expression over PaPILO active reduced columns.
struct PaPILOAffineTerm {
  int col{-1};
  double coef{0.0};
};

/// Exact affine reconstruction of an original-space column from PaPILO's
/// postsolve stack.  Terms use native reduced LP column indices, not PaPILO's
/// compressed internal indices.  `available=false` means the column either was
/// not reconstructed or passed through a non-affine postsolve operation such as
/// parallel-column splitting.
struct PaPILOAffineExpression {
  bool available{false};
  bool exact{false};
  bool too_dense{false};
  bool unsupported{false};
  int original_col{-1};
  double constant{0.0};
  std::vector<PaPILOAffineTerm> terms;
};

/// Presolve aggressiveness profile.
///   kDefault    — PaPILO's stock abort/fill-in thresholds
///   kAggressive — tight abort factors + deeper substitution & lin-dep
///                 detection.  Useful on instances where the default early-
///                 exit fires before bound/coefficient tightening stabilises
///                 (e.g. SCUC, network-constrained planning).
enum class PaPILOProfile {
  kDefault,
  kAggressive,
};

/// Run PaPILO presolve on a MIP model.
PaPILOPresolveResult papilo_presolve_mip(const LPModel& lp, bool verbose = false,
                                          PaPILOProfile profile = PaPILOProfile::kDefault);

/// Recover original-space solution from a reduced-space solution.
/// Uses PaPILO's native Postsolve::undo() for correct handling of
/// substitutions, aggregations, etc.
Eigen::VectorXd papilo_postsolve(const PaPILOPresolveResult& ps,
                                  const Eigen::VectorXd& x_reduced);

/// Validate an active-column reduced solution against PaPILO's own final
/// compressed problem.  This is diagnostic-only: it distinguishes a native
/// reduced LP extraction mismatch from a postsolve undo/mapping issue.
PaPILOReducedValidationSummary papilo_validate_reduced_solution(
    const PaPILOPresolveResult& ps,
    const Eigen::VectorXd& x_reduced);

/// Forward-map an original-space solution to the PaPILO reduced space.
/// For active (non-fixed, non-substituted) columns, the value is copied
/// directly since PaPILO does not transform active variable values.
/// Eliminated columns are simply dropped.
/// Returns a vector of size reduced_cols (active columns only).
Eigen::VectorXd papilo_forward_map(const PaPILOPresolveResult& ps,
                                    const Eigen::VectorXd& x_orig);

/// Export exact affine expressions for original columns in terms of active
/// reduced columns.  This is diagnostic/source-state plumbing: callers should
/// consume only sparse exact expressions and reject unsupported expressions.
std::vector<PaPILOAffineExpression> papilo_export_affine_expressions(
    const PaPILOPresolveResult& ps,
    int max_terms = 8,
    double tol = 1e-10);

}  // namespace mipsolvers::engine

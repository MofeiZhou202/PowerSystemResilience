/// @file highs_presolve_side_state.hpp
/// @brief In-process HiGHS presolve/conformance bridge.

#pragma once

#include "mipsolvers/engine/problem_types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace mipsolvers::engine {

struct StandardFormLP;

struct HiGHSPresolveBridgeInfo {
  bool available{false};
  bool embedded_source{false};
  std::string version;
  std::string githash;
  std::string source;
};

struct HiGHSPresolvedModelStats {
  bool available{false};
  bool pass_ok{false};
  bool presolve_ok{false};
  std::string bridge_source;
  std::string highs_status;
  bool side_state_available{false};
  int rows{0};
  int cols{0};
  int nnz{0};
  int ranged_rows{0};
  int binary_cols{0};
  int integer_cols{0};
  int implied_integer_cols{0};
  int continuous_cols{0};
  int fixed_cols{0};
  int vub_count{0};
  int vlb_count{0};
  int vub_attempts{0};
  int vub_accepted{0};
  int vub_replaced{0};
  int vlb_attempts{0};
  int vlb_accepted{0};
  int vlb_replaced{0};
  int probing_calls{0};
  int probing_conflicts{0};
  int probing_reductions{0};
  int probing_substitutions{0};
  std::uint64_t vub_hash{0};
  std::uint64_t vlb_hash{0};
  bool presolved_lp_available{false};
  double presolved_objective_offset{0.0};
  LPModel presolved_lp;
  std::vector<double> presolved_row_lower;
  std::vector<double> presolved_row_upper;
  std::vector<double> presolved_col_lower;
  std::vector<double> presolved_col_upper;
  std::vector<int> presolved_col_orig;
  std::vector<unsigned char> presolved_col_type;
  std::vector<double> presolved_col_scale;
  std::vector<double> presolved_col_constant;
  std::vector<unsigned char> presolved_col_linearly_transformable;
  std::vector<int> presolved_a_start;
  std::vector<int> presolved_a_index;
  std::vector<double> presolved_a_value;
  struct VarBoundRecord {
    int target_col{-1};
    int trigger_col{-1};
    int target_orig_col{-1};
    int trigger_orig_col{-1};
    double coef{0.0};
    double constant{0.0};
    double target_scale{1.0};
    double target_constant{0.0};
    double trigger_scale{1.0};
    double trigger_constant{0.0};
    bool upper{true};
    bool target_linearly_transformable{false};
    bool trigger_linearly_transformable{false};
  };
  std::vector<VarBoundRecord> var_bounds;
};

struct HiGHSRootLpStateStats {
  bool available{false};
  bool pass_ok{false};
  bool solve_ok{false};
  bool basis_valid{false};
  std::string bridge_source;
  std::string model_status;
  int rows{0};
  int cols{0};
  int nnz{0};
  int iterations{0};
  double objective{0.0};
  int basic_cols{0};
  int nonbasic_lower{0};
  int nonbasic_upper{0};
  int nonbasic_zero{0};
  int nonbasic_other{0};
  int fixed_cols{0};
  int integer_like_total{0};
  int nonbasic_integer_like{0};
  int fractional_total{0};
  int fractional_binary{0};
  int fractional_integer{0};
  int fractional_implied{0};
  std::uint64_t status_hash{0};
  std::uint64_t frontier_hash{0};
  std::string frontier_sample;
  std::string side_sample;
  // Per-column LP-state trace.  These vectors are diagnostic-only and are used
  // to find the first basis/frontier divergence between native and HiGHS.
  std::vector<char> col_status;
  std::vector<double> col_value;
  std::vector<double> col_dual;
};

/// Return the compile/link status of the in-process HiGHS bridge.  This is the
/// first conformance gate: native code must be able to call the bundled HiGHS
/// API before HPresolve side-state can be compared in process.
HiGHSPresolveBridgeInfo highs_presolve_bridge_info();

/// Run HiGHS presolve in-process and return presolved model side-state counts.
/// This is intentionally diagnostic-only: it does not replace native presolve
/// or change B&C behavior.
HiGHSPresolvedModelStats highs_presolve_model_stats(const LPModel& lp);

/// Solve the supplied LP relaxation with HiGHS, presolve disabled, and return a
/// diagnostic LP-state/frontier signature.  This is intentionally diagnostic
/// only: it does not replace native LP solves or change B&C behavior.
HiGHSRootLpStateStats highs_root_lp_state_stats(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double int_tol,
    int max_terms = 32);

/// Solve the supplied native StandardFormLP with HiGHS and project the
/// diagnostic LP-state/frontier signature back to original columns.  This is a
/// conformance-only bridge used to distinguish ranged-row LP-state divergence
/// from native standard-form/simplex-state divergence.
HiGHSRootLpStateStats highs_standard_form_lp_state_stats(
    const StandardFormLP& sf,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double int_tol,
    int max_terms = 32);

// ─────────────────────────────────────────────────────────────────────────────
// Adaptive HiGHS presolve wrapper for the native LP kernels
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for the adaptive HiGHS presolve that wraps the native LP
/// kernels (dual simplex + interior point).  Presolve is only attempted when it
/// is likely to pay off, so a barely-reducible or very large model is solved
/// directly without paying the presolve cost.
struct HighsLpPresolveConfig {
  bool enabled{false};      ///< Master switch (off by default; opt-in per solve).
  long nnz_floor{5000};     ///< Skip presolve when the original nnz is below this
                            ///< (overhead not worth it for tiny models).
  long nnz_cap{300000};     ///< Skip presolve when the original nnz exceeds this.
  double min_shrink{1.0};   ///< Use the reduced LP whenever
                            ///< reduced_nnz < min_shrink * original_nnz.
  bool verbose{false};      ///< Emit a one-line [HIGHS-PRESOLVE] summary.
};

/// Overlay MIPSOLVERS_PRESOLVE* environment overrides on top of @p base:
///   MIPSOLVERS_PRESOLVE            "0"/"1" -> force disable / enable
///   MIPSOLVERS_PRESOLVE_NNZ_FLOOR  integer -> nnz_floor
///   MIPSOLVERS_PRESOLVE_NNZ_CAP    integer -> nnz_cap
///   MIPSOLVERS_PRESOLVE_MIN_SHRINK double  -> min_shrink
///   MIPSOLVERS_PRESOLVE_VERBOSE    present -> verbose
HighsLpPresolveConfig highs_lp_presolve_config_from_env(HighsLpPresolveConfig base);

/// Outcome of an adaptive HiGHS presolve pass over a native LP.  Holds the HiGHS
/// instance internally so a subsequent primal postsolve can undo the reductions.
struct HighsLpPresolveResult {
  bool attempted{false};          ///< Presolve ran (passed the adaptive gate).
  bool use_reduced{false};        ///< Solve `reduced`, then postsolve the primal.
  bool solved_by_presolve{false}; ///< Presolve reduced the model to empty.
  bool infeasible{false};         ///< Presolve proved infeasibility.
  LPModel reduced;                ///< Native reduced LP (valid iff use_reduced).
  std::string status;             ///< HiGHS presolve status name.
  int orig_rows{0};
  int orig_cols{0};
  long orig_nnz{0};
  int reduced_rows{0};
  int reduced_cols{0};
  long reduced_nnz{0};
  double presolve_ms{0.0};
  std::shared_ptr<void> impl;     ///< Retained HiGHS instance for postsolve.
};

/// Run adaptive HiGHS presolve over @p lp.  Never throws; on any error or when
/// the adaptive gate rejects the model, returns a result with use_reduced=false
/// and solved_by_presolve=false (the caller then solves the original LP).
HighsLpPresolveResult highs_presolve_lp(const LPModel& lp,
                                        const HighsLpPresolveConfig& cfg);

/// Recover the original-space primal from a reduced-space primal @p x_reduced
/// via HiGHS primal-only postsolve (needs no basis, so it fits both the simplex
/// and interior-point kernels).  Returns an empty vector on failure.
Eigen::VectorXd highs_postsolve_primal(const HighsLpPresolveResult& ps,
                                       const Eigen::VectorXd& x_reduced);

/// Postsolve @p x_reduced, then audit the recovered primal against the original
/// LP (bounds + rows, sentinel-aware).  On success fills @p x_orig_out and
/// @p objective_out (= lp.c . x_orig) and returns true; on postsolve failure or
/// an audit violation > audit_tol * scale returns false.
bool highs_presolve_recover_primal(const LPModel& lp,
                                   const HighsLpPresolveResult& ps,
                                   const Eigen::VectorXd& x_reduced,
                                   double audit_tol,
                                   Eigen::VectorXd& x_orig_out,
                                   double& objective_out);

}  // namespace mipsolvers::engine

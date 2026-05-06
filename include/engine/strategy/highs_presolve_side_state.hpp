/// @file highs_presolve_side_state.hpp
/// @brief In-process HiGHS presolve/conformance bridge.

#pragma once

#include "hacdcpf/engine/problem_types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace hacdcpf::engine {

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
  struct VarBoundRecord {
    int target_col{-1};
    int trigger_col{-1};
    double coef{0.0};
    double constant{0.0};
    bool upper{true};
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

}  // namespace hacdcpf::engine

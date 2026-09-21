/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef MIP_HIGHS_PRESOLVE_SIDE_STATE_H_
#define MIP_HIGHS_PRESOLVE_SIDE_STATE_H_

#include <cstdint>
#include <vector>

#include "lp_data/HConst.h"

struct HighsPresolveVarBoundRecord {
  HighsInt target_col = -1;
  HighsInt trigger_col = -1;
  HighsInt target_orig_col = -1;
  HighsInt trigger_orig_col = -1;
  double coef = 0.0;
  double constant = 0.0;
  double target_scale = 1.0;
  double target_constant = 0.0;
  double trigger_scale = 1.0;
  double trigger_constant = 0.0;
  bool upper = true;
  bool target_linearly_transformable = false;
  bool trigger_linearly_transformable = false;
};

struct HighsPresolveSideState {
  bool available = false;
  HighsInt rows = 0;
  HighsInt cols = 0;
  HighsInt nnz = 0;
  HighsInt ranged_rows = 0;
  HighsInt binary_cols = 0;
  HighsInt integer_cols = 0;
  HighsInt implied_integer_cols = 0;
  HighsInt continuous_cols = 0;
  HighsInt fixed_cols = 0;
  HighsInt vub_count = 0;
  HighsInt vlb_count = 0;
  HighsInt vub_attempts = 0;
  HighsInt vub_accepted = 0;
  HighsInt vub_replaced = 0;
  HighsInt vlb_attempts = 0;
  HighsInt vlb_accepted = 0;
  HighsInt vlb_replaced = 0;
  HighsInt probing_calls = 0;
  HighsInt probing_conflicts = 0;
  HighsInt probing_reductions = 0;
  HighsInt probing_substitutions = 0;
  std::uint64_t vub_hash = 0;
  std::uint64_t vlb_hash = 0;
  std::vector<double> presolved_row_lower;
  std::vector<double> presolved_row_upper;
  std::vector<double> presolved_col_lower;
  std::vector<double> presolved_col_upper;
  std::vector<HighsInt> presolved_col_orig;
  std::vector<HighsVarType> presolved_col_type;
  std::vector<double> presolved_col_scale;
  std::vector<double> presolved_col_constant;
  std::vector<unsigned char> presolved_col_linearly_transformable;
  std::vector<HighsInt> presolved_a_start;
  std::vector<HighsInt> presolved_a_index;
  std::vector<double> presolved_a_value;
  std::vector<HighsPresolveVarBoundRecord> var_bounds;

  void clear() {
    available = false;
    rows = 0;
    cols = 0;
    nnz = 0;
    ranged_rows = 0;
    binary_cols = 0;
    integer_cols = 0;
    implied_integer_cols = 0;
    continuous_cols = 0;
    fixed_cols = 0;
    vub_count = 0;
    vlb_count = 0;
    vub_attempts = 0;
    vub_accepted = 0;
    vub_replaced = 0;
    vlb_attempts = 0;
    vlb_accepted = 0;
    vlb_replaced = 0;
    probing_calls = 0;
    probing_conflicts = 0;
    probing_reductions = 0;
    probing_substitutions = 0;
    vub_hash = 0;
    vlb_hash = 0;
    presolved_row_lower.clear();
    presolved_row_upper.clear();
    presolved_col_lower.clear();
    presolved_col_upper.clear();
    presolved_col_orig.clear();
    presolved_col_type.clear();
    presolved_col_scale.clear();
    presolved_col_constant.clear();
    presolved_col_linearly_transformable.clear();
    presolved_a_start.clear();
    presolved_a_index.clear();
    presolved_a_value.clear();
    var_bounds.clear();
  }
};

#endif

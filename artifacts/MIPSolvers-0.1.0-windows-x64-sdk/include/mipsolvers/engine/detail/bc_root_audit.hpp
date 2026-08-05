/// @file bc_root_audit.hpp
/// @brief Root-cut and root-reduced-cost audit helpers for B&C diagnostics.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/branch_and_cut.hpp"

namespace mipsolvers::engine::detail {

struct RootCutRowAudit {
  int rows{0};
  int invalid_rows{0};
  int nonviolated_at_lp{0};
  int incumbent_violations{0};
  double max_lp_violation{0.0};
  double max_incumbent_violation{0.0};
};

struct RootCutStandardFormAudit {
  int rows{0};
  int flipped_rows{0};
  int mismatches{0};
  double max_coeff_error{0.0};
  double max_rhs_error{0.0};
};

struct RootReducedCostAudit {
  std::uint64_t rc_nonzero{0};
  std::uint64_t rc_integer_nonzero{0};
  std::uint64_t rc_implied_integer_nonzero{0};
  std::uint64_t rc_at_lower{0};
  std::uint64_t rc_at_upper{0};
  std::uint64_t cutoff_fix_candidates{0};
  double rc_abs_sum{0.0};
  double rc_abs_max{0.0};
  double gap{0.0};
};

struct RootSubsolveTrace {
  std::string name;
  int calls{0};
  int successes{0};
  double total_ms{0.0};
  int incumbent_hits{0};
  int lp_solves{0};
};

void truncate_ineq_rows(LPModel& lp, int keep_rows);

RootCutRowAudit audit_root_cut_rows(const LPModel& lp,
                                    int first_new_row,
                                    const Eigen::VectorXd& lp_point,
                                    const Eigen::VectorXd* incumbent_point,
                                    double tol);

RootCutStandardFormAudit audit_root_cut_standard_form_rows(const LPModel& lp,
                                                           int first_new_row,
                                                           double tol);

RootReducedCostAudit audit_root_reduced_costs(
    const std::vector<VariableMeta>& vars,
    const std::vector<char>* implied_integer_cols,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    double node_bound,
    double incumbent_obj,
    double int_tol);

void note_root_subsolve_trace(std::vector<RootSubsolveTrace>& traces,
                              bool enabled,
                              const char* name,
                              double ms,
                              bool success,
                              bool incumbent_hit = false,
                              int lp_solves = 0);

void print_root_subsolve_traces(const std::vector<RootSubsolveTrace>& traces,
                                int root_frac_bin_count,
                                const std::string& root_incumbent_source);

void print_root_phase_timing(std::chrono::steady_clock::time_point t0,
                             std::chrono::steady_clock::time_point t_root0,
                             std::chrono::steady_clock::time_point t_root1,
                             std::chrono::steady_clock::time_point t_cuts_done,
                             std::chrono::steady_clock::time_point t_pump_done,
                             std::chrono::steady_clock::time_point t_prog_done,
                             std::chrono::steady_clock::time_point t_dive_done,
                             std::chrono::steady_clock::time_point t_lns_done,
                             bool has_incumbent,
                             bool trace_root_subsolves,
                             const std::vector<RootSubsolveTrace>& traces,
                             int root_frac_bin_count,
                             const std::string& root_incumbent_source);

}  // namespace mipsolvers::engine::detail

/// @file bc_legacy_helpers.hpp
/// @brief Small support helpers shared by the legacy branch-and-cut modules.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/branch_and_cut.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#endif

namespace mipsolvers::engine::detail {

bool bc_env_flag_enabled(const char* name);
int bc_conformance_trace_terms(const char* env_name, int default_value = 32);
bool bc_frontier_conformance_enabled();
bool bc_first_class_lp_state_conformance_enabled();
bool bc_vendored_highs_lp_kernel_enabled(const BCOptions& opt);
bool bc_vendored_highs_root_frontier_enabled(const BCOptions& opt);

/// Allocate the root-LP share of the remaining global wall-clock budget.
/// A fallback reserve is meaningful only when the reserved minimum can still
/// be reached; otherwise the root owns the full remaining budget.
double bc_root_lp_budget_sec(double remaining_sec,
                             double total_limit_sec,
                             double fallback_min_remaining_sec,
                             double fallback_budget_fraction,
                             bool fallback_available,
                             bool short_budget_ipm_root);

/// Allocate one optional nested solve without violating either the outer
/// wall-clock deadline or the stage-wide allowance. Returns zero when the
/// remaining usable budget is too small to start another solve.
double bc_optional_subsolve_budget_sec(double per_call_limit_sec,
                                       double global_remaining_sec,
                                       double finalization_reserve_sec,
                                       double stage_remaining_sec,
                                       double minimum_useful_sec = 0.001);

void apply_bc_first_class_simplex_state(SimplexOptions& opt,
                                        bool allow_frontier_remap);

struct BcStrictHighsContractState {
  bool requested_vendored_highs_lp{false};
  bool strict_highs_lp_contract{false};
  bool allow_vendored_root_frontier{true};
  bool domain_heuristics{false};
};

BcStrictHighsContractState apply_bc_strict_highs_contract(BCOptions& opt);

struct BcDeclaredIntegrality {
  std::vector<char> integer_cols;
  std::vector<char> binary_cols;
};

BcDeclaredIntegrality normalize_declared_integrality(const MIPModel& prob,
                                                     LPModel& base_lp);

std::shared_ptr<SimplexBasis> persist_bc_node_basis_from_simplex(
    SimplexResult& simplex,
    const std::shared_ptr<SimplexBasis>& previous_basis);

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
const char* bc_highs_model_status_label(HighsModelStatus status);

bool bc_native_basis_to_highs_basis(const LPModel& lp,
                                    const SimplexBasis* basis_hint,
                                    HighsBasis& hbasis);

bool bc_pass_mip_model_to_highs(Highs& highs,
                                const LPModel& lp,
                                const Eigen::VectorXd* lb_override,
                                const Eigen::VectorXd* ub_override);

#endif

}  // namespace mipsolvers::engine::detail

/// @file bc_conformance_trace.hpp
/// @brief LP/frontier conformance diagnostics shared by the legacy B&C core.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

namespace mipsolvers::engine::detail {

class VariableBoundTable;

const char* bc_var_type_name(const VariableMeta& v);
std::vector<char> bc_original_basic_mask(int n_orig,
                                         const SimplexBasis* basis_hint);
char bc_native_basis_status(int col,
                            const std::vector<char>& basic,
                            const SimplexBasis* basis_hint);
double bc_native_reduced_cost(int col, const SimplexBasis* basis_hint);

std::uint64_t bc_model_side_state_signature(const LPModel& lp);
const HiGHSPresolvedModelStats& cached_highs_presolve_side_state(
    const LPModel& lp,
    bool* cache_hit = nullptr);

std::uint64_t bc_trace_hash_mix(std::uint64_t seed, std::uint64_t value);
std::uint64_t bc_trace_hash_double(double value);

void trace_native_frontier_conformance(
    const char* phase,
    const LPModel& lp,
    const StandardFormLP* native_sf,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double objective,
    double int_tol);

void trace_native_frontier_conformance(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double objective,
    double int_tol);

void trace_native_repair_bounds(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& root_x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    double int_tol,
    const char* detail = nullptr);

void trace_native_repair_candidate_order(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const SimplexBasis* basis_hint,
    const std::vector<int>& ordered_cols,
    double int_tol,
    const char* detail = nullptr);

void trace_native_presolve_state_conformance(
    const char* phase,
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const VariableBoundTable& variable_bound_table);

}  // namespace mipsolvers::engine::detail

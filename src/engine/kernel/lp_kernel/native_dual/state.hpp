#pragma once

#include "factor.hpp"

namespace mipsolvers::engine::native_dual::detail {

Bounds make_phase_two_bounds(const StandardFormLP& sf);
Bounds make_primal_phase_one_bounds(const StandardFormLP& sf);
Eigen::VectorXd make_primal_phase_one_cost(const StandardFormLP& sf);
bool is_artificial(const StandardFormLP& sf, int col);
bool build_logical_basis(const StandardFormLP& sf, std::vector<int>& basis,
                         std::string& failure);
int apply_certified_singleton_crash(const StandardFormLP& sf,
                                    std::vector<int>& basis);

bool initialize(State& state, const StandardFormLP& sf,
                const SimplexOptions& options, Phase phase,
                const SimplexBasis* hint, Statistics& statistics,
                std::string& failure);
bool rebuild_membership(State& state, std::string& failure);
bool reconstruct(State& state, std::string& failure);
bool correct_canonical_primal_residual(State& state, std::string& failure);
bool normalize_nonbasic_moves(State& state, std::string& failure);
bool initialize_exact_edge_weights(State& state, Statistics& statistics,
                                   std::string& failure);
void initialize_devex_framework(State& state, Statistics& statistics);
bool initialize_stabilized_cost(State& state, Statistics& statistics,
                                std::string& failure);
bool major_rebuild(State& state, RebuildReason reason, bool reinvert,
                   Statistics& statistics, std::string& failure);
void restore_original_cost(State& state);
bool working_cost_is_original(const State& state);
void initialize_cycle_guard(State& state);
bool is_taboo_change(const State& state, int leaving_col, int entering_col,
                     int* expires_after = nullptr);
bool is_taboo_row(const State& state, int leaving_col,
                  int* expires_after = nullptr);
void add_taboo_row(State& state, int leaving_col, Statistics& statistics);
void release_taboo_row(State& state, int leaving_col,
                       Statistics& statistics);
void record_cycle_departure(State& state, int leaving_col, int entering_col);
bool record_cycle_arrival(State& state, Statistics& statistics);
// Incremental cycle-signature maintenance (see State::cycle_signature_live_*).
void resync_cycle_signature(State& state);
void cycle_signature_apply_basis_swap(State& state, int row, int old_col,
                                      int new_col);
// XOR-toggles the nonbasic move token (col, move_sign): adds it when absent,
// removes it when present.  A move-side change is two toggles.
void cycle_signature_apply_move_toggle(State& state, int col, int move_sign);

Eigen::VectorXd multiply_A(const Eigen::SparseMatrix<double>& A,
                           const Eigen::VectorXd& x);
Eigen::VectorXd multiply_AT(const Eigen::SparseMatrix<double>& A,
                            const Eigen::VectorXd& y);
double equation_residual_inf(const Eigen::SparseMatrix<double>& A,
                             const Eigen::VectorXd& x,
                             const Eigen::VectorXd& rhs);

Eigen::VectorXd full_primal(const State& state);
double primal_infeasibility(const State& state, int row, int& side);
double dual_infeasibility(const State& state, int col);
Audit audit(const State& state, bool require_primal, bool require_dual,
            bool require_artificial_zero);

SimplexBasis export_basis(const State& state);
Result make_result(const State& state, Status status, std::string message,
                   Statistics statistics);

}  // namespace mipsolvers::engine::native_dual::detail

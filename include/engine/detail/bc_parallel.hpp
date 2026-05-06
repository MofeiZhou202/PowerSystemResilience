/// @file bc_parallel.hpp
/// @brief Declarations for parallel B&C thread functions.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/engine/detail/bc_types.hpp"
#include "hacdcpf/engine/detail/bc_threading.hpp"
#include "hacdcpf/engine/detail/bc_fallback.hpp"
#include "hacdcpf/engine/detail/bc_solver_dispatch.hpp"
#include "hacdcpf/engine/detail/bc_clique_table.hpp"
#include "hacdcpf/engine/detail/bc_utils.hpp"

namespace hacdcpf::engine::detail {

/// @brief Explorer thread: pops nodes, solves LP relaxations, branches.
void explorer_thread(
    int thread_id,
    WorkerRole role,
    ThreadSafeNodeQueue& node_queue,
    CutRequestQueue& cut_queue,
    SharedCutPool& shared_cp,
    SharedConflictPool& shared_conflicts,
    SharedSolutionPool& shared_sp,
    SharedIncumbent& shared_inc,
    const LPModel& base_lp,
    const LPModel& pre_cut_lp,
    const StandardFormLP& base_sf,
    const CliqueTable& clique_table,
    const BinaryImplicationGraph& implication_graph,
    const SimplexOptions& simplex_opt,
    const BCOptions& opt,
    std::vector<PseudoCost>& pc,
    std::mutex& pc_mtx,
    AtomicBCStats& stats,
    std::atomic<bool>& should_stop,
    std::atomic<int>& active_explorers,
    ActiveNodeBounds& active_bounds,
    FallbackLogger& fallback_logger,
    std::shared_ptr<const SimplexBasis> root_basis,
    WorkStealingPool* steal_pool,
    SharedImplicationGraph* shared_impl_graph = nullptr,
    DeterministicTurnToken* det_token = nullptr,
    const std::atomic<double>* optimality_limit = nullptr);

/// @brief Cut worker thread: generates GMI cuts from LP results.
void cut_worker_thread(
    CutRequestQueue& cut_queue,
    SharedCutPool& shared_cp,
    SharedSolutionPool& shared_sp,
    SharedIncumbent& shared_inc,
    const LPModel& base_lp,
    const BCOptions& opt,
    AtomicBCStats& stats,
    std::atomic<bool>& should_stop);

}  // namespace hacdcpf::engine::detail

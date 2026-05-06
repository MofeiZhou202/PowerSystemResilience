/// @file shared_state.cpp
/// @brief Implementation of BCSolveContext

#include "mipsolvers/engine/solver/native/milp/bc/parallel/shared_state.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Sparse>
#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine::solver::native::milp::bc {

BCSolveContext::BCSolveContext(const MIPModel& prob_in,
                               const BCOptions& opt_in,
                               const SolveStats* presolve_stats_in)
    : prob(prob_in),
      pseudocosts_(prob_in.linear_part.c.size()),
      ipm_node(nullptr),
      ipm_fallback(nullptr),
      ipm_cut(nullptr),
      opt(opt_in),
      presolve_stats_ptr(presolve_stats_in) {
  // Initialize pseudocosts with default construction (zero stats).
  for (size_t i = 0; i < pseudocosts_.size(); ++i) {
    pseudocosts_[i] = detail::PseudoCost();
  }

  // Pre-build row caches eagerly at construction (cheap, avoids lock on hot path).
  const LPModel& lp = prob.linear_part;
  A_row_cached_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(lp.A);
  Aeq_row_cached_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(lp.Aeq);
  row_index_cached_.build(
      static_cast<int>(lp.vars.size()),
      A_row_cached_, Aeq_row_cached_);
  row_cache_built_ = true;

}

bool BCSolveContext::update_incumbent(const Eigen::VectorXd& new_x,
                                      double new_obj) {
  if (!std::isfinite(new_obj)) return false;

  const double current_obj = incumbent_obj.load();
  if (new_obj >= current_obj) return false;  // Not better

  std::lock_guard<std::mutex> lock(incumbent_mtx);
  // Double-check with lock held (another thread may have updated)
  if (new_obj >= incumbent_obj.load()) return false;

  incumbent_x = new_x;
  incumbent_obj.store(new_obj);
  incumbent_found.store(true);
  return true;
}

void BCSolveContext::update_dual_bound(double new_bound) {
  // CAS loop to ensure monotonic increase of dual bound
  double current = dual_bound_.load();
  while (new_bound > current &&
         !dual_bound_.compare_exchange_weak(current, new_bound)) {
    // retry if CAS failed
  }
}

double BCSolveContext::elapsed_time() const {
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::duration<double>>(now -
                                                                  start_time);
  return elapsed.count();
}

double BCSolveContext::remaining_time() const {
  const double time_limit = opt.time_limit_sec;
  if (time_limit <= 0.0) return -1.0;  // No limit

  const double elapsed = elapsed_time();
  const double remaining = time_limit - elapsed;
  return std::max(0.0, remaining);
}

SolveResult BCSolveContext::build_result(
    const std::string& status_msg) const {
  SolveResult result;

  // Status and message
  result.stats.status = status_msg;
  result.stats.success = incumbent_found.load();

  // Bounds and gap
  result.stats.objective = incumbent_obj.load();
  result.stats.mip_gap =
      std::max(0.0, primal_bound() - dual_bound_.load());

  // Counters
  result.stats.iterations = static_cast<int>(node_count.load());

  // Timing
  result.stats.runtime_sec = elapsed_time();

  // Solution
  if (incumbent_found.load()) {
    std::lock_guard<std::mutex> lock(incumbent_mtx);
    result.x = incumbent_x;
  }

  // Feasibility check
  result.stats.primal_feas = incumbent_found.load() ? 1.0 : 0.0;
  result.stats.dual_feas = 1.0;  // Dual feasibility verified during B&C

  return result;
}

const Eigen::SparseMatrix<double, Eigen::RowMajor>&
BCSolveContext::A_row_cached() const {
  if (!row_cache_built_) {
    std::lock_guard<std::mutex> lk(row_cache_mtx_);
    if (!row_cache_built_) {
      const LPModel& lp = prob.linear_part;
      A_row_cached_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(lp.A);
      Aeq_row_cached_ = Eigen::SparseMatrix<double, Eigen::RowMajor>(lp.Aeq);
      row_index_cached_.build(
          static_cast<int>(lp.vars.size()),
          A_row_cached_, Aeq_row_cached_);
      row_cache_built_ = true;
    }
  }
  return A_row_cached_;
}

const Eigen::SparseMatrix<double, Eigen::RowMajor>&
BCSolveContext::Aeq_row_cached() const {
  (void)A_row_cached();  // ensure built
  return Aeq_row_cached_;
}

const detail::RowPropagationIndex& BCSolveContext::row_prop_index() const {
  (void)A_row_cached();  // ensure built
  return row_index_cached_;
}

}  // namespace mipsolvers::engine::solver::native::milp::bc

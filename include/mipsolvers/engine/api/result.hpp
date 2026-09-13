#pragma once

#include <limits>
#include <cstddef>
#include <string>

#include <Eigen/Core>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine::api {

struct Stats {
  bool success{false};
  bool strict_convergence{false};
  bool acceptable_convergence{false};
  int iterations{0};
  double objective{0.0};
  double residual_inf{0.0};
  double primal_feas{0.0};
  double dual_feas{0.0};
  double complementarity{0.0};
  double barrier_parameter{0.0};
  double unscaled_primal_feas{0.0};
  double unscaled_dual_feas{0.0};
  double unscaled_complementarity{0.0};
  double initial_primal_feas{std::numeric_limits<double>::quiet_NaN()};
  bool warm_start_used{false};
  double relative_primal_residual{std::numeric_limits<double>::quiet_NaN()};
  double relative_dual_residual{std::numeric_limits<double>::quiet_NaN()};
  double relative_gap{std::numeric_limits<double>::quiet_NaN()};
  double dual_objective{std::numeric_limits<double>::quiet_NaN()};
  double mip_gap{0.0};
  double runtime_sec{0.0};
  int thread_budget{0};
  int portfolio_workers{0};
  int worker_thread_limit{0};
  std::size_t memory_limit_bytes{0};
  bool memory_limit_enforced{false};
  bool hard_deadline_enforced{false};
  double deadline_overrun_sec{0.0};
  bool persistent_backend_reused{false};
  std::size_t incremental_update_count{0};
  std::string status;
  std::string solver_name;
  int cglp_cuts_added{0};
  Eigen::VectorXd farkas_ray;
  Eigen::VectorXd farkas_ray_eq;
  bool has_farkas_certificate{false};
};

struct Result {
  Eigen::VectorXd x;
  Stats stats;
  Eigen::VectorXd constraint_duals;
  Eigen::VectorXd box_dual_lb;
  Eigen::VectorXd box_dual_ub;
};

inline bool is_success(const Result& result) {
  return result.stats.success;
}

}  // namespace mipsolvers::engine::api

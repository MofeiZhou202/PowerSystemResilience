#pragma once

#include <memory>

#include <Eigen/Core>

#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Persistent LP model with transactional O(n) objective/bound and O(m) RHS
/// updates. With preferred_solver="HiGHS" and fallback disabled, one resident
/// HiGHS model receives incremental change calls and retains compatible basis
/// state. Other policies reuse the resident public model and dispatch normally.
/// Instances are not safe for concurrent mutation or solve calls. See
/// docs/archive/general_solver_performance_program_2026-09-13.md, R4.
class LPModelSession {
 public:
  explicit LPModelSession(LPModel model, bool register_default_adapters = true);
  ~LPModelSession();

  LPModelSession(LPModelSession&&) noexcept;
  LPModelSession& operator=(LPModelSession&&) noexcept;
  LPModelSession(const LPModelSession&) = delete;
  LPModelSession& operator=(const LPModelSession&) = delete;

  void update_objective(Eigen::VectorXd objective);
  void update_variable_bounds(Eigen::VectorXd lower, Eigen::VectorXd upper);
  void update_inequality_rhs(Eigen::VectorXd lower, Eigen::VectorXd upper);
  void update_equality_rhs(Eigen::VectorXd rhs);

  api::Result solve(const SolveOptions& options = {}) const;
  const LPModel& model() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

namespace api {
using LPModelSession = ::mipsolvers::engine::LPModelSession;
}

}  // namespace mipsolvers::engine

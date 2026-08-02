#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/detail/bc_domain.hpp"

namespace mipsolvers::engine::detail {

class CliqueTable;

enum class BCDomainProbeStatus { Feasible, Infeasible, Incomplete };

struct BCDomainProbeDelta {
  int col{-1};
  double old_lb{0.0};
  double old_ub{0.0};
  double new_lb{0.0};
  double new_ub{0.0};
};

struct BCDomainProbeOutcome {
  BCDomainProbeStatus status{BCDomainProbeStatus::Incomplete};
  std::vector<BCDomainProbeDelta> deltas;
};

struct BCDomainProbeTelemetry {
  std::uint64_t workspace_initializations{0};
  std::uint64_t worlds{0};
  std::uint64_t trail_pushes{0};
  std::uint64_t rollbacks{0};
  std::uint64_t failures{0};
  std::uint64_t rows_processed{0};
  std::uint64_t changed_columns{0};
};

class BCDomainProbeWorkspace {
 public:
  BCDomainProbeWorkspace(const LPModel& lp,
                         const Eigen::VectorXd& lb,
                         const Eigen::VectorXd& ub,
                         const CliqueTable* clique_table,
                         double integrality_tolerance);

  BCDomainProbeStatus initialize(std::vector<BCDomainProbeDelta>& deltas);
  BCDomainProbeOutcome probe(int col, bool value_one);
  BCDomainProbeStatus commit(int col,
                             bool value_one,
                             std::vector<BCDomainProbeDelta>& deltas);

  const Eigen::VectorXd& lb() const { return domain_.lb(); }
  const Eigen::VectorXd& ub() const { return domain_.ub(); }
  const BCDomainProbeTelemetry& telemetry() const { return telemetry_; }

 private:
  BCDomainProbeStatus propagate(std::size_t trail_start,
                                bool seed_existing_fixed_literals);
  void collect_deltas(std::size_t trail_start,
                      std::vector<BCDomainProbeDelta>& deltas);
  bool is_integral(int col) const;
  void next_generation(std::vector<std::uint32_t>& stamps,
                       std::uint32_t& generation);

  const LPModel* lp_{nullptr};
  const CliqueTable* clique_table_{nullptr};
  double integrality_tolerance_{1e-9};
  BCDomain domain_;
  std::vector<std::uint32_t> changed_col_stamp_;
  std::vector<std::uint32_t> literal_stamp_;
  std::uint32_t changed_col_generation_{0};
  std::uint32_t literal_generation_{0};
  BCDomainProbeTelemetry telemetry_;
};

}  // namespace mipsolvers::engine::detail

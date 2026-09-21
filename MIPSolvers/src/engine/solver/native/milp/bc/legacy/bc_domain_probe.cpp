#include "mipsolvers/engine/detail/bc_domain_probe.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"

namespace mipsolvers::engine::detail {

BCDomainProbeWorkspace::BCDomainProbeWorkspace(
    const LPModel& lp,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const CliqueTable* clique_table,
    double integrality_tolerance)
    : lp_(&lp),
      clique_table_(clique_table),
      integrality_tolerance_(std::max(1e-9, integrality_tolerance)),
      changed_col_stamp_(lp.vars.size(), 0),
      literal_stamp_(2 * lp.vars.size(), 0) {
  domain_.init(lp, lb, ub);
  telemetry_.workspace_initializations = 1;
}

bool BCDomainProbeWorkspace::is_integral(int col) const {
  if (col < 0 || col >= static_cast<int>(lp_->vars.size())) return false;
  const VarType type = lp_->vars[static_cast<std::size_t>(col)].type;
  return type == VarType::Integer || type == VarType::Binary;
}

void BCDomainProbeWorkspace::next_generation(
    std::vector<std::uint32_t>& stamps,
    std::uint32_t& generation) {
  ++generation;
  if (generation == 0) {
    std::fill(stamps.begin(), stamps.end(), 0);
    generation = 1;
  }
}

void BCDomainProbeWorkspace::collect_deltas(
    std::size_t trail_start,
    std::vector<BCDomainProbeDelta>& deltas) {
  deltas.clear();
  next_generation(changed_col_stamp_, changed_col_generation_);
  for (std::size_t pos = trail_start; pos < domain_.trail_size(); ++pos) {
    const BCDomain::TrailEntry& entry = domain_.trail_entry(pos);
    const int col = entry.col;
    if (col < 0 || col >= domain_.num_vars()) continue;
    auto& stamp = changed_col_stamp_[static_cast<std::size_t>(col)];
    if (stamp == changed_col_generation_) continue;
    stamp = changed_col_generation_;
    deltas.push_back(BCDomainProbeDelta{col, entry.old_lb, entry.old_ub,
                                        domain_.lb()[col], domain_.ub()[col]});
  }
}

BCDomainProbeStatus BCDomainProbeWorkspace::propagate(
    std::size_t trail_start,
    bool seed_existing_fixed_literals) {
  next_generation(literal_stamp_, literal_generation_);
  std::size_t clique_cursor = trail_start;
  int closure_passes = 0;

  auto propagate_fixed_literal = [&](int col) {
    if (clique_table_ == nullptr || clique_table_->empty() ||
        !is_integral(col)) {
      return;
    }
    bool value_one = false;
    if (domain_.lb()[col] >= 1.0 - integrality_tolerance_ &&
        domain_.ub()[col] >= 1.0 - integrality_tolerance_) {
      value_one = true;
    } else if (domain_.ub()[col] <= integrality_tolerance_ &&
               domain_.lb()[col] <= integrality_tolerance_) {
      value_one = false;
    } else {
      return;
    }

    const int literal = 2 * col + (value_one ? 1 : 0);
    if (literal < 0 || literal >= 2 * domain_.num_vars()) return;
    auto& stamp = literal_stamp_[static_cast<std::size_t>(literal)];
    if (stamp == literal_generation_) return;
    stamp = literal_generation_;

    const auto neighbours = clique_table_->literal_neighbours(col, value_one);
    for (const int* pos = neighbours.first; pos != neighbours.second; ++pos) {
      const int forbidden_literal = *pos;
      const int implied_col = forbidden_literal / 2;
      const bool forbidden_one = (forbidden_literal % 2) != 0;
      if (implied_col < 0 || implied_col >= domain_.num_vars()) continue;
      if (forbidden_one) {
        (void)domain_.change_bound(implied_col, domain_.lb()[implied_col], 0.0);
      } else {
        (void)domain_.change_bound(implied_col, 1.0, domain_.ub()[implied_col]);
      }
      if (domain_.infeasible()) return;
    }
  };

  if (seed_existing_fixed_literals) {
    for (int col = 0; col < domain_.num_vars() && !domain_.infeasible(); ++col) {
      propagate_fixed_literal(col);
    }
  }

  while (!domain_.infeasible() && closure_passes++ < 32) {
    const std::uint64_t rows_before = domain_.rows_processed();
    (void)domain_.propagate();
    telemetry_.rows_processed += domain_.rows_processed() - rows_before;
    if (domain_.infeasible()) return BCDomainProbeStatus::Infeasible;

    while (clique_cursor < domain_.trail_size() && !domain_.infeasible()) {
      propagate_fixed_literal(domain_.trail_entry(clique_cursor++).col);
    }
    if (domain_.infeasible()) return BCDomainProbeStatus::Infeasible;
    if (domain_.propagation_complete() && clique_cursor == domain_.trail_size()) {
      return BCDomainProbeStatus::Feasible;
    }
  }
  return BCDomainProbeStatus::Incomplete;
}

BCDomainProbeStatus BCDomainProbeWorkspace::initialize(
    std::vector<BCDomainProbeDelta>& deltas) {
  const BCDomainProbeStatus status =
      propagate(/*trail_start=*/0, /*seed_existing_fixed_literals=*/true);
  collect_deltas(/*trail_start=*/0, deltas);
  if (status != BCDomainProbeStatus::Feasible) ++telemetry_.failures;
  return status;
}

BCDomainProbeOutcome BCDomainProbeWorkspace::probe(int col, bool value_one) {
  BCDomainProbeOutcome outcome;
  const BCDomain::Savepoint savepoint = domain_.savepoint();
  const std::size_t trail_start = domain_.trail_size();
  ++telemetry_.worlds;

  if (!domain_.fix_col(col, value_one ? 1.0 : 0.0)) {
    outcome.status = domain_.infeasible() ? BCDomainProbeStatus::Infeasible
                                          : BCDomainProbeStatus::Incomplete;
  } else {
    outcome.status =
        propagate(trail_start, /*seed_existing_fixed_literals=*/false);
  }
  collect_deltas(trail_start, outcome.deltas);
  telemetry_.trail_pushes += domain_.trail_size() - trail_start;
  telemetry_.changed_columns += outcome.deltas.size();

  domain_.restore(savepoint);
  ++telemetry_.rollbacks;
  bool rollback_ok = true;
  for (const BCDomainProbeDelta& delta : outcome.deltas) {
    if (std::abs(domain_.lb()[delta.col] - delta.old_lb) > 1e-12 ||
        std::abs(domain_.ub()[delta.col] - delta.old_ub) > 1e-12) {
      rollback_ok = false;
      break;
    }
  }
  if (!rollback_ok || outcome.status == BCDomainProbeStatus::Incomplete) {
    ++telemetry_.failures;
    outcome.status = BCDomainProbeStatus::Incomplete;
  }
  return outcome;
}

BCDomainProbeStatus BCDomainProbeWorkspace::commit(
    int col,
    bool value_one,
    std::vector<BCDomainProbeDelta>& deltas) {
  const BCDomain::Savepoint savepoint = domain_.savepoint();
  const std::size_t trail_start = domain_.trail_size();
  BCDomainProbeStatus status;
  if (!domain_.fix_col(col, value_one ? 1.0 : 0.0)) {
    status = domain_.infeasible() ? BCDomainProbeStatus::Infeasible
                                  : BCDomainProbeStatus::Incomplete;
  } else {
    status = propagate(trail_start, /*seed_existing_fixed_literals=*/false);
  }
  collect_deltas(trail_start, deltas);
  if (status == BCDomainProbeStatus::Feasible) return status;

  domain_.restore(savepoint);
  if (status == BCDomainProbeStatus::Incomplete) ++telemetry_.failures;
  return status;
}

}  // namespace mipsolvers::engine::detail

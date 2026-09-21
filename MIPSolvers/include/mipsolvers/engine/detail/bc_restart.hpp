#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "mipsolvers/engine/bc/options.hpp"

namespace mipsolvers::engine::detail {

/// Stateful trigger for incumbent-driven sequential-tree restarts.
/// The controller decides when a restart is due; queue replacement and proof
/// state ownership remain explicit at the branch-and-cut call site.
class IncumbentTreeRestartController {
 public:
  explicit IncumbentTreeRestartController(BCTreeRestartOptions options)
      : options_(std::move(options)) {}

  void note_incumbent(double objective, int node, const char* source) {
    if (!std::isfinite(objective)) return;
    const double tol =
        1e-10 * std::max({1.0, std::abs(objective),
                          std::isfinite(pending_objective_)
                              ? std::abs(pending_objective_)
                              : 1.0});
    if (pending_ && objective >= pending_objective_ - tol) return;
    pending_ = true;
    pending_objective_ = objective;
    pending_node_ = std::max(0, node);
    pending_source_ = source != nullptr ? source : "tree_incumbent";
  }

  bool ready(int current_node, int open_nodes, bool proof_valid,
             double remaining_time_sec) const {
    if (!options_.enabled || !pending_ || !proof_valid ||
        restart_count_ >= std::max(0, options_.max_restarts) ||
        open_nodes < std::max(1, options_.min_open_nodes) ||
        current_node - last_restart_node_ <
            std::max(0, options_.min_nodes_since_restart) ||
        remaining_time_sec <
            std::max(0.0, options_.min_remaining_time_sec)) {
      return false;
    }
    if (!std::isfinite(last_restart_objective_)) return true;
    const double improvement = last_restart_objective_ - pending_objective_;
    const double relative =
        improvement / std::max(1.0, std::abs(last_restart_objective_));
    return relative + 1e-15 >=
           std::max(0.0, options_.min_relative_incumbent_improvement);
  }

  void commit(int current_node) {
    if (!pending_) return;
    ++restart_count_;
    last_restart_node_ = std::max(0, current_node);
    last_restart_objective_ = pending_objective_;
    pending_ = false;
  }

  bool pending() const { return pending_; }
  int restart_count() const { return restart_count_; }
  int pending_node() const { return pending_node_; }
  double pending_objective() const { return pending_objective_; }
  const std::string& pending_source() const { return pending_source_; }

 private:
  BCTreeRestartOptions options_;
  bool pending_{false};
  int restart_count_{0};
  int last_restart_node_{0};
  int pending_node_{-1};
  double last_restart_objective_{
      std::numeric_limits<double>::infinity()};
  double pending_objective_{std::numeric_limits<double>::infinity()};
  std::string pending_source_;
};

}  // namespace mipsolvers::engine::detail

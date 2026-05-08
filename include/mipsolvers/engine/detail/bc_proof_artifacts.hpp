/// @file bc_proof_artifacts.hpp
/// @brief Objective/proof artifact extraction helpers for legacy B&C.

#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

class BinaryImplicationGraph;
class CliqueTable;
class ConflictPool;
class SharedConflictPool;
struct DualProofRow;
struct ObjectivePropagationState;

struct ObjectiveCutoffArtifactStats {
  std::uint64_t candidates{0};
  std::uint64_t implied_event_candidates{0};
  std::uint64_t conflicts_seen{0};
  std::uint64_t conflict_clauses_added{0};
  std::uint64_t implications_added{0};
  std::uint64_t clique_edges_added{0};
  std::uint64_t unary_conflicts_added{0};
  std::uint64_t pair_conflicts_added{0};
  double raw_objective_lower{0.0};
  double cutoff_capacity{kInf};
  double max_delta{0.0};
};

struct DualProofLiteralArtifactStats {
  std::uint64_t candidates{0};
  std::uint64_t conflicts_seen{0};
  std::uint64_t conflict_clauses_added{0};
  std::uint64_t implications_added{0};
  std::uint64_t clique_edges_added{0};
  std::uint64_t frontier_priority_updates{0};
  double proof_capacity{kInf};
  double max_delta{0.0};
};

struct ObjectiveCutoffCliqueStats {
  std::uint64_t candidates{0};
  std::uint64_t pair_tests{0};
  std::uint64_t pair_conflicts{0};
  std::uint64_t cliques_generated{0};
  std::uint64_t clique_edges_added{0};
  std::uint64_t conflict_clauses_added{0};
  double raw_objective_lower{0.0};
  double cutoff_capacity{kInf};
  double max_pair_excess{0.0};
};

ObjectiveCutoffCliqueStats extract_objective_cutoff_cliques(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    double upper_limit,
    CliqueTable& clique_table,
    double int_tol,
    const ObjectivePropagationState* objective_propagation = nullptr,
    ConflictPool* conflict_pool = nullptr,
    SharedConflictPool* shared_conflict_pool = nullptr);

ObjectiveCutoffArtifactStats extract_objective_cutoff_artifacts(
    const LPModel& lp,
	    const std::vector<char>& implied_integer_cols,
	    const Eigen::VectorXd& lb,
	    const Eigen::VectorXd& ub,
	    double upper_limit,
    BinaryImplicationGraph& implication_graph,
    CliqueTable& clique_table,
    double int_tol,
    const ObjectivePropagationState* objective_propagation = nullptr,
    ConflictPool* conflict_pool = nullptr,
    SharedConflictPool* shared_conflict_pool = nullptr,
    std::uint64_t max_implications = 250000);

DualProofLiteralArtifactStats extract_dual_proof_literal_artifacts(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const DualProofRow& proof,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    BinaryImplicationGraph& implication_graph,
    CliqueTable& clique_table,
    double int_tol,
    ConflictPool* conflict_pool = nullptr,
    SharedConflictPool* shared_conflict_pool = nullptr,
    std::vector<int>* proof_frontier_priority = nullptr,
    std::uint64_t max_implications = 250000);

int append_graph_implied_bound_cuts(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const BinaryImplicationGraph& graph,
    int max_cuts,
    std::vector<Eigen::SparseVector<double>>& rows,
    std::vector<double>& rhs);

}  // namespace mipsolvers::engine::detail

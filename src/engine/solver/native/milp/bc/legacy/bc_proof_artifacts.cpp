/// @file bc_proof_artifacts.cpp
/// @brief Objective/proof artifact extraction helpers for legacy B&C.

#include "mipsolvers/engine/detail/bc_proof_artifacts.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Sparse>
#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_objective_propagation.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine::detail {

ObjectiveCutoffCliqueStats extract_objective_cutoff_cliques(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    double upper_limit,
    CliqueTable& clique_table,
    double int_tol,
    const ObjectivePropagationState* objective_propagation,
    ConflictPool* conflict_pool,
    SharedConflictPool* shared_conflict_pool) {
  ObjectiveCutoffCliqueStats stats;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 1 || lp.sense != Sense::Minimize || !std::isfinite(upper_limit) ||
      lb.size() < n || ub.size() < n) {
    return stats;
  }

  const double tol = std::max(1e-9, int_tol);
  auto binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > tol || var.ub < 1.0 - tol) return false;
    if (var.type == VarType::Binary) return true;
    return j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0 &&
           std::abs(var.lb) <= tol && std::abs(var.ub - 1.0) <= tol;
  };

  struct Candidate {
    int col{-1};
    bool literal_one{false};
    double delta{0.0};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(n));

  int ninf = 0;
  std::vector<double> cutoff_coeff(static_cast<std::size_t>(n), 0.0);
  for (int j = 0; j < n; ++j) {
    const double c = lp.c[j];
    if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
    cutoff_coeff[static_cast<std::size_t>(j)] = c;
  }
  double cutoff_rhs = upper_limit;
  if (objective_propagation != nullptr &&
      !objective_propagation->partitions.empty()) {
    for (const auto& part : objective_propagation->partitions) {
      double largest = 0.0;
      int clique_rhs = 1;
      for (const auto& term : part.terms) {
        if (term.col < 0 || term.col >= n || !std::isfinite(term.cost) ||
            std::abs(term.cost) <= 1e-12) {
          continue;
        }
        if (term.cost > 0.0) {
          --clique_rhs;
          if (lb[term.col] < 1.0 - tol) {
            largest = std::max(largest, term.cost);
          }
        } else if (ub[term.col] > tol) {
          largest = std::max(largest, -term.cost);
        }
      }
      if (!(largest > 1e-12) || !std::isfinite(largest)) continue;
      cutoff_rhs += largest * static_cast<double>(clique_rhs);
      for (const auto& term : part.terms) {
        if (term.col < 0 || term.col >= n || !std::isfinite(term.cost) ||
            std::abs(term.cost) <= 1e-12) {
          continue;
        }
        cutoff_coeff[static_cast<std::size_t>(term.col)] =
            term.cost - std::copysign(largest, term.cost);
      }
    }
  }

  for (int j = 0; j < n; ++j) {
    const double c = cutoff_coeff[static_cast<std::size_t>(j)];
    if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
    if (c > 0.0) {
      if (!std::isfinite(lb[j])) {
        ++ninf;
      } else {
        stats.raw_objective_lower += c * lb[j];
      }
      if (binary_like(j) && ub[j] >= 1.0 - tol) {
        const double delta = c * std::max(0.0, 1.0 - lb[j]);
        if (delta > 1e-12 && std::isfinite(delta)) {
          candidates.push_back(Candidate{j, true, delta});
        }
      }
    } else {
      if (!std::isfinite(ub[j])) {
        ++ninf;
      } else {
        stats.raw_objective_lower += c * ub[j];
      }
      if (binary_like(j) && lb[j] <= tol) {
        const double delta = (-c) * std::max(0.0, ub[j]);
        if (delta > 1e-12 && std::isfinite(delta)) {
          candidates.push_back(Candidate{j, false, delta});
        }
      }
    }
  }

  stats.candidates = static_cast<std::uint64_t>(candidates.size());
  if (ninf != 0 || candidates.size() <= 1) return stats;

  stats.cutoff_capacity = cutoff_rhs - stats.raw_objective_lower;
  if (!std::isfinite(stats.cutoff_capacity)) return stats;

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (std::abs(a.delta - b.delta) > 1e-12) {
                return a.delta > b.delta;
              }
              if (a.col != b.col) return a.col < b.col;
              return a.literal_one > b.literal_one;
            });

  if (candidates[0].delta + candidates[1].delta <=
      stats.cutoff_capacity + tol) {
    return stats;
  }

  std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
      literal_edges;
  literal_edges.reserve(1024);
  auto publish_conflict = [&](const Candidate& a, const Candidate& b) {
    if (a.col == b.col) return;
    ++stats.pair_conflicts;
    literal_edges.emplace_back(CliqueTable::Literal{a.col, a.literal_one},
                               CliqueTable::Literal{b.col, b.literal_one});
    std::vector<BranchDomainLiteral> clause{
        BranchDomainLiteral{a.col, a.literal_one ? 1.0 : 0.0, a.literal_one},
        BranchDomainLiteral{b.col, b.literal_one ? 1.0 : 0.0, b.literal_one}};
    canonicalize_branch_literals(clause);
    bool added = false;
    if (conflict_pool != nullptr) added = conflict_pool->add(clause) || added;
    if (shared_conflict_pool != nullptr) {
      added = shared_conflict_pool->add(clause) || added;
    }
    if (added) ++stats.conflict_clauses_added;
  };

  const int nbin = static_cast<int>(candidates.size());
  for (int k = nbin - 1; k > 0; --k) {
    const Candidate& tail = candidates[static_cast<std::size_t>(k)];
    const double min_clique_val = stats.cutoff_capacity - tail.delta + tol;
    auto clique_end = std::partition_point(
        candidates.begin(), candidates.begin() + k,
        [&](const Candidate& candidate) {
          return candidate.delta > min_clique_val;
        });
    if (clique_end == candidates.begin()) continue;

    const int old_edges = static_cast<int>(literal_edges.size());
    for (auto it = candidates.begin(); it != clique_end; ++it) {
      ++stats.pair_tests;
      publish_conflict(*it, tail);
    }
    if (static_cast<int>(literal_edges.size()) > old_edges) {
      ++stats.cliques_generated;
      stats.max_pair_excess = std::max(
          stats.max_pair_excess,
          candidates.front().delta + tail.delta - stats.cutoff_capacity);
    }
    if (clique_end == candidates.begin() + k) break;
  }

  stats.clique_edges_added = clique_table.add_literal_edges(lp, literal_edges);
  return stats;
}

ObjectiveCutoffArtifactStats extract_objective_cutoff_artifacts(
    const LPModel& lp,
	    const std::vector<char>& implied_integer_cols,
	    const Eigen::VectorXd& lb,
	    const Eigen::VectorXd& ub,
	    double upper_limit,
    BinaryImplicationGraph& implication_graph,
    CliqueTable& clique_table,
    double int_tol,
    const ObjectivePropagationState* objective_propagation,
    ConflictPool* conflict_pool,
    SharedConflictPool* shared_conflict_pool,
    std::uint64_t max_implications) {
  ObjectiveCutoffArtifactStats stats;
  const int n = static_cast<int>(lp.vars.size());
	  if (n <= 1 || lp.sense != Sense::Minimize ||
	      !std::isfinite(upper_limit) || lb.size() < n || ub.size() < n) {
    return stats;
  }

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > int_tol || var.ub < 1.0 - int_tol) return false;
    if (var.type == VarType::Binary) return true;
    return is_implied_integer(j) && std::abs(var.lb) <= int_tol &&
           std::abs(var.ub - 1.0) <= int_tol;
  };

  struct Candidate {
    int col{-1};
    bool literal_one{false};
    double delta{0.0};
    std::unordered_map<int, double> contribution_by_target;
  };
  auto candidate_literal = [](const Candidate& candidate) {
    return BranchDomainLiteral{candidate.col,
                               candidate.literal_one ? 1.0 : 0.0,
                               candidate.literal_one};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(n));
  std::unordered_map<int, std::size_t> candidate_by_literal;
  candidate_by_literal.reserve(static_cast<std::size_t>(n));
  // Different proof sources may describe the same objective target. Charge
  // each target once per literal, while retaining sums across distinct targets.
  auto merge_candidate_contribution = [](Candidate& candidate, int target,
                                         double contribution) {
    auto [it, inserted] =
        candidate.contribution_by_target.emplace(target, contribution);
    if (inserted) {
      candidate.delta += contribution;
    } else if (contribution > it->second) {
      candidate.delta += contribution - it->second;
      it->second = contribution;
    }
  };
  auto candidate_for_literal = [&](int col, bool literal_one) -> Candidate& {
    const int key = 2 * col + (literal_one ? 1 : 0);
    const auto [it, inserted] =
        candidate_by_literal.emplace(key, candidates.size());
    if (inserted) {
      candidates.push_back(Candidate{col, literal_one, 0.0, {}});
    }
    return candidates[it->second];
  };
  auto add_candidate_contribution = [&](int col, bool literal_one, int target,
                                        double contribution) {
    Candidate& candidate = candidate_for_literal(col, literal_one);
    merge_candidate_contribution(candidate, target, contribution);
  };

  const double cutoff = upper_limit;

  std::vector<double> cutoff_coeff(static_cast<std::size_t>(n), 0.0);
  for (int j = 0; j < n; ++j) {
    const double c = lp.c[j];
    if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
    cutoff_coeff[static_cast<std::size_t>(j)] = c;
  }
  double cutoff_rhs = cutoff;
  if (objective_propagation != nullptr &&
      !objective_propagation->partitions.empty()) {
    for (const auto& part : objective_propagation->partitions) {
      double largest = 0.0;
      int clique_rhs = 1;
      for (const auto& term : part.terms) {
        if (term.col < 0 || term.col >= n ||
            !std::isfinite(term.cost) || std::abs(term.cost) <= 1e-12) {
          continue;
        }
        if (term.cost > 0.0) {
          --clique_rhs;
          if (lb[term.col] < 1.0 - int_tol) {
            largest = std::max(largest, term.cost);
          }
        } else if (ub[term.col] > int_tol) {
          largest = std::max(largest, -term.cost);
        }
      }
      if (!(largest > 1e-12) || !std::isfinite(largest)) continue;
      cutoff_rhs += largest * static_cast<double>(clique_rhs);
      for (const auto& term : part.terms) {
        if (term.col < 0 || term.col >= n ||
            !std::isfinite(term.cost) || std::abs(term.cost) <= 1e-12) {
          continue;
        }
        cutoff_coeff[static_cast<std::size_t>(term.col)] =
            term.cost - std::copysign(largest, term.cost);
      }
    }
  }

  int inf_count = 0;
  for (int j = 0; j < n; ++j) {
    const double c = cutoff_coeff[static_cast<std::size_t>(j)];
    if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
    if (c > 0.0) {
      if (!std::isfinite(lb[j])) {
        ++inf_count;
      } else {
        stats.raw_objective_lower += c * lb[j];
      }
      if (binary_like(j) && ub[j] >= 1.0 - int_tol) {
        const double delta = c * std::max(0.0, 1.0 - lb[j]);
        if (delta > 1e-9) {
          add_candidate_contribution(j, true, j, delta);
        }
      }
    } else {
      if (!std::isfinite(ub[j])) {
        ++inf_count;
      } else {
        stats.raw_objective_lower += c * ub[j];
      }
      if (binary_like(j) && lb[j] <= int_tol) {
        const double delta = (-c) * std::max(0.0, ub[j]);
        if (delta > 1e-9) {
          add_candidate_contribution(j, false, j, delta);
        }
      }
    }
  }

  auto add_forbidden_literal = [&](const Candidate& trigger,
                                   const Candidate& forbidden) {
    if (trigger.col == forbidden.col) return false;
    bool added = false;
    if (forbidden.literal_one) {
      added = implication_graph.add_implication(
          trigger.col, trigger.literal_one, forbidden.col,
          /*implied_is_lb=*/false, 0.0);
    } else {
      added = implication_graph.add_implication(
          trigger.col, trigger.literal_one, forbidden.col,
          /*implied_is_lb=*/true, 1.0);
    }
    if (added) ++stats.implications_added;
    return added;
  };
  auto publish_conflict_clause =
      [&](const std::vector<BranchDomainLiteral>& raw_clause) {
        std::vector<BranchDomainLiteral> clause = raw_clause;
        canonicalize_branch_literals(clause);
        if (clause.empty()) return false;
        bool added = false;
        if (conflict_pool != nullptr) {
          added = conflict_pool->add(clause) || added;
        }
        if (shared_conflict_pool != nullptr) {
          added = shared_conflict_pool->add(clause) || added;
        }
        if (added) ++stats.conflict_clauses_added;
        return added;
      };
  auto publish_pair_conflict = [&](const Candidate& a, const Candidate& b) {
    if (a.col == b.col) return false;
    ++stats.conflicts_seen;
    const bool added_ab = add_forbidden_literal(a, b);
    const bool added_ba = add_forbidden_literal(b, a);
    if (publish_conflict_clause(
            std::vector<BranchDomainLiteral>{candidate_literal(a),
                                             candidate_literal(b)})) {
      ++stats.pair_conflicts_added;
    }
    return added_ab || added_ba;
  };

  struct AggregateEvent {
    double contribution{0.0};
    std::unordered_map<int, double> best_by_target;
  };
  std::unordered_map<int, AggregateEvent> implied_aggregates;
  if (objective_propagation != nullptr &&
      !objective_propagation->implied_events.empty()) {
    const double tol = std::max(1e-9, int_tol);
    auto literal_active = [&](const ObjectivePropagationState::Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (lb[lit.col] >= 1.0 - tol)
                            : (ub[lit.col] <= tol));
    };
    auto literal_possible = [&](const ObjectivePropagationState::Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (ub[lit.col] >= 1.0 - tol)
                            : (lb[lit.col] <= tol));
    };

    implied_aggregates.reserve(std::min<std::size_t>(
        objective_propagation->implied_events.size(),
        static_cast<std::size_t>(1024)));
    for (const auto& event : objective_propagation->implied_events) {
      const int target = event.target_col;
      if (target < 0 || target >= n ||
          objective_propagation->objective_partition_owns_col(target)) {
        continue;
      }
      const bool target_tightens = event.target_is_lower_bound
          ? (event.target_bound > lb[target] + tol)
          : (event.target_bound < ub[target] - tol);
      if (!target_tightens) continue;
      const double contribution = event.target_is_lower_bound
          ? event.target_cost * (event.target_bound - lb[target])
          : (-event.target_cost) * (ub[target] - event.target_bound);
      if (!(contribution > 1e-9) || !std::isfinite(contribution)) {
        continue;
      }

      int missing_key = -1;
      bool impossible = false;
      for (const auto& lit : event.literals) {
        if (literal_active(lit)) continue;
        if (literal_possible(lit)) {
          const int key = 2 * lit.col + (lit.value_one ? 1 : 0);
          if (missing_key >= 0 && missing_key != key) {
            missing_key = -2;
            break;
          }
          missing_key = key;
        } else {
          impossible = true;
          break;
        }
      }
      if (impossible || missing_key < 0) continue;

      AggregateEvent& aggregate = implied_aggregates[missing_key];
      auto [it, inserted] =
          aggregate.best_by_target.emplace(target, contribution);
      if (inserted) {
        aggregate.contribution += contribution;
      } else if (contribution > it->second + 1e-9) {
        aggregate.contribution += contribution - it->second;
        it->second = contribution;
      }
    }

    for (const auto& [key, aggregate] : implied_aggregates) {
      if (!(aggregate.contribution > 1e-9) ||
          !std::isfinite(aggregate.contribution)) {
        continue;
      }
      const int col = key / 2;
      const bool value_one = (key % 2) != 0;
      if (!binary_like(col)) continue;
      Candidate& candidate = candidate_for_literal(col, value_one);
      for (const auto& [target, contribution] : aggregate.best_by_target) {
        merge_candidate_contribution(candidate, target, contribution);
      }
      ++stats.implied_event_candidates;
    }
  }

  stats.candidates = static_cast<std::uint64_t>(candidates.size());
  for (const Candidate& candidate : candidates) {
    stats.max_delta = std::max(stats.max_delta, candidate.delta);
  }
  if (inf_count != 0) return stats;

  stats.cutoff_capacity = cutoff_rhs - stats.raw_objective_lower;
  if (!std::isfinite(stats.cutoff_capacity)) return stats;

  const double tol = std::max(1e-9, int_tol);
  for (const Candidate& candidate : candidates) {
    if (candidate.delta > stats.cutoff_capacity + tol) {
      const BranchDomainLiteral forbidden = candidate_literal(candidate);
      if (publish_conflict_clause(std::vector<BranchDomainLiteral>{forbidden})) {
        ++stats.unary_conflicts_added;
      }
    }
  }
  if (candidates.size() <= 1) return stats;

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (a.delta != b.delta) return a.delta > b.delta;
              if (a.col != b.col) return a.col < b.col;
              return a.literal_one > b.literal_one;
            });
  if (candidates[0].delta + candidates[1].delta <=
      stats.cutoff_capacity + 1e-9) {
    return stats;
  }

  std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
      literal_edges;
  literal_edges.reserve(1024);
  auto combined_delta = [](const Candidate& a, const Candidate& b) {
    double delta = a.delta;
    for (const auto& [target, contribution] : b.contribution_by_target) {
      const auto it = a.contribution_by_target.find(target);
      if (it == a.contribution_by_target.end()) {
        delta += contribution;
      } else if (contribution > it->second) {
        delta += contribution - it->second;
      }
    }
    return delta;
  };
  for (int k = static_cast<int>(candidates.size()) - 1; k > 0; --k) {
    const Candidate& b = candidates[static_cast<std::size_t>(k)];
    const double threshold = stats.cutoff_capacity - b.delta + tol;
    for (int i = 0; i < k; ++i) {
      const Candidate& a = candidates[static_cast<std::size_t>(i)];
      if (a.delta <= threshold) break;
      if (a.col == b.col) continue;
      if (combined_delta(a, b) <= stats.cutoff_capacity + tol) continue;
      const bool added = publish_pair_conflict(a, b);
      literal_edges.emplace_back(
          CliqueTable::Literal{a.col, a.literal_one},
          CliqueTable::Literal{b.col, b.literal_one});
      if (!added &&
          stats.implications_added >= max_implications) {
        return stats;
      }
      if (stats.implications_added >= max_implications) {
        stats.clique_edges_added =
            clique_table.add_literal_edges(lp, literal_edges);
        return stats;
      }
    }
  }

  stats.clique_edges_added = clique_table.add_literal_edges(lp, literal_edges);
  return stats;
}

DualProofLiteralArtifactStats extract_dual_proof_literal_artifacts(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    const DualProofRow& proof,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    BinaryImplicationGraph& implication_graph,
    CliqueTable& clique_table,
    double int_tol,
    ConflictPool* conflict_pool,
    SharedConflictPool* shared_conflict_pool,
    std::vector<int>* proof_frontier_priority,
    std::uint64_t max_implications) {
  DualProofLiteralArtifactStats stats;
  const int n = static_cast<int>(lp.vars.size());
  if (!proof.valid || proof.coeff.size() < n || n <= 1 ||
      lb.size() < n || ub.size() < n || !std::isfinite(proof.rhs) ||
      !std::isfinite(proof.min_activity)) {
    return stats;
  }

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > int_tol || var.ub < 1.0 - int_tol) return false;
    if (var.type == VarType::Binary) return true;
    return is_implied_integer(j) && std::abs(var.lb) <= int_tol &&
           std::abs(var.ub - 1.0) <= int_tol;
  };

  struct Candidate {
    int col{-1};
    bool literal_one{false};
    double delta{0.0};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(proof.coeff.nonZeros()));

  for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff); it; ++it) {
    const int j = static_cast<int>(it.index());
    if (!binary_like(j)) continue;
    const double a = it.value();
    if (a > 1e-12 && std::isfinite(lb[j]) && ub[j] >= 1.0 - int_tol) {
      const double delta = a * std::max(0.0, 1.0 - lb[j]);
      if (delta > 1e-9) {
        candidates.push_back(Candidate{j, true, delta});
        stats.max_delta = std::max(stats.max_delta, delta);
      }
    } else if (a < -1e-12 && std::isfinite(ub[j]) &&
               lb[j] <= int_tol) {
      const double delta = (-a) * std::max(0.0, ub[j]);
      if (delta > 1e-9) {
        candidates.push_back(Candidate{j, false, delta});
        stats.max_delta = std::max(stats.max_delta, delta);
      }
    }
  }
  stats.candidates = static_cast<std::uint64_t>(candidates.size());
  if (candidates.size() <= 1) return stats;

  stats.proof_capacity = proof.rhs - proof.min_activity;
  if (!std::isfinite(stats.proof_capacity)) return stats;
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (a.delta != b.delta) return a.delta > b.delta;
              return a.col < b.col;
            });
  if (candidates[0].delta + candidates[1].delta <=
      stats.proof_capacity + 1e-9) {
    return stats;
  }

  std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
      literal_edges;
  literal_edges.reserve(1024);
  auto add_forbidden_literal = [&](const Candidate& trigger,
                                   const Candidate& forbidden) {
    if (trigger.col == forbidden.col) return false;
    bool added = false;
    if (forbidden.literal_one) {
      added = implication_graph.add_implication(
          trigger.col, trigger.literal_one, forbidden.col,
          /*implied_is_lb=*/false, 0.0);
    } else {
      added = implication_graph.add_implication(
          trigger.col, trigger.literal_one, forbidden.col,
          /*implied_is_lb=*/true, 1.0);
    }
    if (added) ++stats.implications_added;
    return added;
  };
  auto candidate_literal = [](const Candidate& candidate) {
    return BranchDomainLiteral{candidate.col,
                               candidate.literal_one ? 1.0 : 0.0,
                               candidate.literal_one};
  };
  auto publish_conflict_clause = [&](const Candidate& a,
                                     const Candidate& b) {
    std::vector<BranchDomainLiteral> clause;
    clause.reserve(2);
    clause.push_back(candidate_literal(a));
    clause.push_back(candidate_literal(b));
    canonicalize_branch_literals(clause);
    if (clause.size() != 2) return false;
    bool added = false;
    if (conflict_pool != nullptr) {
      added = conflict_pool->add(clause) || added;
    }
    if (shared_conflict_pool != nullptr) {
      added = shared_conflict_pool->add(clause) || added;
    }
    if (added) {
      ++stats.conflict_clauses_added;
    }
    return added;
  };
  auto note_frontier_priority = [&](const Candidate& a,
                                    const Candidate& b) {
    if (proof_frontier_priority == nullptr) return;
    if (a.col < 0 || b.col < 0 ||
        a.col >= static_cast<int>(proof_frontier_priority->size()) ||
        b.col >= static_cast<int>(proof_frontier_priority->size()) ||
        a.col == b.col) {
      return;
    }
    auto bump = [&](int col) {
      int& priority = (*proof_frontier_priority)[static_cast<std::size_t>(col)];
      if (priority < std::numeric_limits<int>::max() / 2) {
        ++priority;
        ++stats.frontier_priority_updates;
      }
    };
    bump(a.col);
    bump(b.col);
  };

  const double tol = std::max(1e-9, int_tol);
  for (int k = static_cast<int>(candidates.size()) - 1; k > 0; --k) {
    const Candidate& b = candidates[static_cast<std::size_t>(k)];
    const double threshold = stats.proof_capacity - b.delta + tol;
    for (int i = 0; i < k; ++i) {
      const Candidate& a = candidates[static_cast<std::size_t>(i)];
      if (a.delta <= threshold) break;
      if (a.col == b.col) continue;
      ++stats.conflicts_seen;
      note_frontier_priority(a, b);
      publish_conflict_clause(a, b);
      add_forbidden_literal(a, b);
      add_forbidden_literal(b, a);
      literal_edges.emplace_back(
          CliqueTable::Literal{a.col, a.literal_one},
          CliqueTable::Literal{b.col, b.literal_one});
      if (stats.implications_added >= max_implications) {
        stats.clique_edges_added =
            clique_table.add_literal_edges(lp, literal_edges);
        return stats;
      }
    }
  }
  stats.clique_edges_added = clique_table.add_literal_edges(lp, literal_edges);
  return stats;
}

int append_graph_implied_bound_cuts(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const BinaryImplicationGraph& graph,
    int max_cuts,
    std::vector<Eigen::SparseVector<double>>& rows,
    std::vector<double>& rhs) {
  if (graph.empty() || max_cuts <= 0) return 0;
  const int n = std::min({static_cast<int>(lp.vars.size()),
                          static_cast<int>(x.size()),
                          static_cast<int>(node_lb.size()),
                          static_cast<int>(node_ub.size())});
  if (n <= 0) return 0;

  int added = 0;
  std::unordered_set<std::string> seen;
  constexpr double kTol = 1e-7;

  auto add_if_violated = [&](int trigger, bool trigger_one,
                             const BinaryImplicationGraph::Implication& arc) {
    if (added >= max_cuts) return;
    const int k = arc.var_idx;
    if (trigger < 0 || trigger >= n || k < 0 || k >= n || trigger == k) return;
    if (!std::isfinite(x[trigger]) || !std::isfinite(x[k]) ||
        !std::isfinite(arc.value)) {
      return;
    }

    const double lb = node_lb[k];
    const double ub = node_ub[k];
    if (arc.is_lb && !std::isfinite(lb)) return;
    if (!arc.is_lb && !std::isfinite(ub)) return;

    double rhs_val = 0.0;
    double coeff_trigger = 0.0;
    double coeff_implied = arc.is_lb ? -1.0 : 1.0;
    if (trigger_one) {
      if (arc.is_lb) {
        if (!(arc.value > lb + 1e-9)) return;
        // x_t = 1 => x_k >= b:
        //   -x_k + (b-lb_k) x_t <= -lb_k.
        coeff_trigger = arc.value - lb;
        rhs_val = -lb;
      } else {
        if (!(arc.value < ub - 1e-9)) return;
        // x_t = 1 => x_k <= b:
        //   x_k + (ub_k-b) x_t <= ub_k.
        coeff_trigger = ub - arc.value;
        rhs_val = ub;
      }
    } else {
      if (arc.is_lb) {
        if (!(arc.value > lb + 1e-9)) return;
        // x_t = 0 => x_k >= b:
        //   -x_k + (lb_k-b) x_t <= -b.
        coeff_trigger = lb - arc.value;
        rhs_val = -arc.value;
      } else {
        if (!(arc.value < ub - 1e-9)) return;
        // x_t = 0 => x_k <= b:
        //   x_k + (b-ub_k) x_t <= b.
        coeff_trigger = arc.value - ub;
        rhs_val = arc.value;
      }
    }

    if (!std::isfinite(coeff_trigger) || !std::isfinite(rhs_val)) return;
    const double lhs = coeff_implied * x[k] + coeff_trigger * x[trigger];
    const double scale = std::max({1.0, std::abs(rhs_val),
                                   std::abs(coeff_implied),
                                   std::abs(coeff_trigger)});
    const double violation = lhs - rhs_val;
    if (!(violation > kTol * scale)) return;

    const std::string key = fmt::format("{}:{}:{}:{}:{:.12g}",
                                        trigger, trigger_one ? 1 : 0,
                                        k, arc.is_lb ? 1 : 0, arc.value);
    if (!seen.insert(key).second) return;

    Eigen::SparseVector<double> cut(n);
    cut.reserve(2);
    cut.coeffRef(k) += coeff_implied;
    cut.coeffRef(trigger) += coeff_trigger;
    rows.push_back(std::move(cut));
    rhs.push_back(rhs_val);
    ++added;
  };

  for (int j = 0; j < n && added < max_cuts; ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    const bool binary_like =
        is_integer_type(v) && node_lb[j] >= -1e-9 && node_ub[j] <= 1.0 + 1e-9;
    if (!binary_like) continue;
    for (bool trigger_one : {false, true}) {
      auto range = graph.implications(j, trigger_one);
      for (const auto* arc = range.first;
           arc != range.second && added < max_cuts; ++arc) {
        add_if_violated(j, trigger_one, *arc);
      }
    }
  }
  return added;
}

}  // namespace mipsolvers::engine::detail

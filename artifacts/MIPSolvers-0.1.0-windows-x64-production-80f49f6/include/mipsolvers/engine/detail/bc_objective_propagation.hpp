/// @file bc_objective_propagation.hpp
/// @brief Objective propagation and objective-event proof helpers for legacy B&C.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

struct ObjectiveCutoffEvent {
  int row{-1};
  int s{-1};
  int y{-1};
  int z{-1};
  double mandatory_s{0.0};
  double cost{0.0};
  double activity{0.0};
};

struct ObjectivePropagationState {
  struct BuildPolicy {
    bool build_row_implied_events{true};
    bool build_pair_row_implied_events{true};
    bool strengthen_row_events_with_mir{true};
    bool build_implication_events{true};

    static BuildPolicy native_enriched() { return BuildPolicy{}; }

    static BuildPolicy highs_domain_closure() {
      BuildPolicy policy;
      policy.build_row_implied_events = false;
      policy.build_pair_row_implied_events = false;
      policy.strengthen_row_events_with_mir = false;
      policy.build_implication_events = false;
      return policy;
    }

    static BuildPolicy none() {
      BuildPolicy policy;
      policy.build_row_implied_events = false;
      policy.build_pair_row_implied_events = false;
      policy.strengthen_row_events_with_mir = false;
      policy.build_implication_events = false;
      return policy;
    }
  };

  struct Term {
    int col{-1};
    double cost{0.0};
    bool integer{false};
  };
  struct PartitionTerm {
    int col{-1};
    double cost{0.0};
    bool favorable_one{false};
    double favorable_contribution{0.0};
    double nonfavorable_contribution{0.0};
    double replacement_delta{0.0};
  };
  struct Partition {
    std::vector<PartitionTerm> terms;
  };
  struct Literal {
    int col{-1};
    bool value_one{false};
  };
  struct ImpliedContributionEvent {
    int target_col{-1};
    double target_bound{0.0};
    double target_cost{0.0};
    bool target_is_lower_bound{true};
    std::vector<Literal> literals;
  };

  std::vector<Term> terms;
  std::vector<int> partition_of_col;
  std::vector<Partition> partitions;
  std::vector<ImpliedContributionEvent> implied_events;
  std::vector<char> implied_event_target_cols;
  std::vector<std::vector<int>> implied_events_by_literal;
  std::uint64_t clique_edges{0};
  std::uint64_t mixed_rows_scanned{0};
  double max_replacement_delta{0.0};
  double max_implied_event_delta{0.0};
  double max_implied_event_aggregate_delta{0.0};
  std::uint64_t implied_event_vlb_events{0};
  std::uint64_t implied_event_vub_events{0};
  std::uint64_t implied_event_integral_rounds{0};
  std::uint64_t implied_event_mir_attempts{0};
  std::uint64_t implied_event_mir_strengthened{0};
  double implied_event_mir_gain_sum{0.0};
  double implied_event_mir_gain_max{0.0};
  std::uint64_t objective_clique_candidate_literals{0};
  std::uint64_t objective_clique_pair_tests{0};
  std::uint64_t objective_clique_pair_conflicts{0};
  std::uint64_t objective_clique_literal_conflicts{0};
  std::uint64_t objective_clique_implication_conflicts{0};

  void setup(const LPModel& lp,
             const std::vector<char>* implied_integer_cols = nullptr,
             const CliqueTable* clique_table = nullptr,
             const BinaryImplicationGraph* implication_graph = nullptr,
             BuildPolicy build_policy = BuildPolicy::native_enriched()) {
    terms.clear();
    partitions.clear();
    implied_events.clear();
    clique_edges = 0;
    mixed_rows_scanned = 0;
    max_replacement_delta = 0.0;
    max_implied_event_delta = 0.0;
    max_implied_event_aggregate_delta = 0.0;
    implied_event_vlb_events = 0;
    implied_event_vub_events = 0;
    implied_event_integral_rounds = 0;
    implied_event_mir_attempts = 0;
    implied_event_mir_strengthened = 0;
    implied_event_mir_gain_sum = 0.0;
    implied_event_mir_gain_max = 0.0;
    objective_clique_candidate_literals = 0;
    objective_clique_pair_tests = 0;
    objective_clique_pair_conflicts = 0;
    objective_clique_literal_conflicts = 0;
    objective_clique_implication_conflicts = 0;
    const int n = static_cast<int>(lp.vars.size());
    partition_of_col.assign(static_cast<std::size_t>(n), -1);
    implied_event_target_cols.assign(static_cast<std::size_t>(n), 0);
    implied_events_by_literal.assign(static_cast<std::size_t>(2 * n), {});
    terms.reserve(static_cast<std::size_t>(n));
    auto implied_integer = [&](int j) {
      return implied_integer_cols != nullptr &&
             j >= 0 && j < static_cast<int>(implied_integer_cols->size()) &&
             (*implied_integer_cols)[static_cast<std::size_t>(j)];
    };
    for (int j = 0; j < n; ++j) {
      const double c = lp.c[j];
      if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
      terms.push_back(Term{
          j, c, is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
                    implied_integer(j)});
    }
    build_objective_clique_partitions(lp, implied_integer_cols);
    build_objective_clique_partitions_from_implications(
        lp, implied_integer_cols, clique_table, implication_graph);
    if (build_policy.build_row_implied_events) {
      build_implied_contribution_events(lp, implied_integer_cols, build_policy);
    }
    if (build_policy.build_implication_events) {
      build_implied_contribution_events_from_implications(lp, implication_graph);
    }
    if (!implied_events.empty()) {
      recompute_max_implied_event_aggregate_delta(lp);
    }
  }

  bool empty() const { return terms.empty(); }

  bool objective_partition_owns_col(int col) const {
    return col >= 0 &&
           col < static_cast<int>(partition_of_col.size()) &&
           partition_of_col[static_cast<std::size_t>(col)] >= 0;
  }

  static bool finite_model_bound(double value) {
    return std::isfinite(value) && std::abs(value) < 1e19;
  }

  static bool strengthen_binary_var_bound_mir(double& coef,
                                              double& constant,
                                              bool lower_bound) {
    if (!std::isfinite(coef) || !std::isfinite(constant) ||
        std::abs(coef) >= 1e19 || std::abs(constant) >= 1e19) {
      return false;
    }
    constexpr double f0min = 0.005;
    constexpr double f0max = 0.995;
    constexpr double tiny = 1e-12;
    const double multiplier = lower_bound ? -1.0 : 1.0;
    const double downrhs = std::floor(multiplier * constant);
    const double f0 = multiplier * constant - downrhs;
    if (f0 < f0min || f0 > f0max) return false;
    const double downaj = std::floor(-multiplier * coef + tiny);
    const double fj = -multiplier * coef - downaj;
    constant = multiplier * downrhs;
    coef = -multiplier *
           (downaj + std::max(fj - f0, 0.0) / (1.0 - f0));
    return true;
  }

  void build_objective_clique_partitions(
      const LPModel& lp,
      const std::vector<char>* implied_integer_cols = nullptr) {
    const int n = static_cast<int>(lp.vars.size());
    auto binary_like = [&](int j) {
      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      if (var.type == VarType::Binary) return true;
      const bool implied_integer =
          implied_integer_cols != nullptr &&
          j >= 0 && j < static_cast<int>(implied_integer_cols->size()) &&
          (*implied_integer_cols)[static_cast<std::size_t>(j)];
      return implied_integer &&
             std::abs(var.lb) <= 1e-9 && std::abs(var.ub - 1.0) <= 1e-9;
    };
    struct Candidate {
      int col{-1};
      double cost{0.0};
      bool favorable_one{false};
      double penalty{0.0};
    };
    std::vector<Candidate> candidates;
    std::vector<int> cand_pos(static_cast<std::size_t>(n), -1);
    for (int j = 0; j < n; ++j) {
      const double c = lp.c[j];
      if (std::abs(c) <= 1e-12 || !std::isfinite(c)) continue;
      if (!binary_like(j)) continue;
      if (lp.vars[static_cast<std::size_t>(j)].lb > 1e-9 ||
          lp.vars[static_cast<std::size_t>(j)].ub < 1.0 - 1e-9) {
        continue;
      }
      const int pos = static_cast<int>(candidates.size());
      cand_pos[static_cast<std::size_t>(j)] = pos;
      candidates.push_back(Candidate{j, c, c < 0.0, std::abs(c)});
    }
    if (candidates.size() <= 1) return;
    objective_clique_candidate_literals +=
        static_cast<std::uint64_t>(candidates.size());

    std::vector<std::vector<int>> adj(candidates.size());
    auto add_edge = [&](int a, int b) {
      if (a < 0 || b < 0 || a == b) return;
      adj[static_cast<std::size_t>(a)].push_back(b);
      adj[static_cast<std::size_t>(b)].push_back(a);
      ++clique_edges;
    };

    auto scan_leq_row = [&](const std::vector<std::pair<int, double>>& row,
                            double rhs) {
      if (row.size() < 2 || row.size() > 512 || !std::isfinite(rhs)) return;
      double min_activity = 0.0;
      struct RowCand {
        int cand{-1};
        double lift{0.0};
        double penalty{0.0};
      };
      std::vector<RowCand> row_cands;
      row_cands.reserve(row.size());
      bool has_continuous_or_general = false;
      for (const auto& [col, a] : row) {
        if (col < 0 || col >= n || std::abs(a) <= 1e-12 || !std::isfinite(a)) {
          continue;
        }
        const auto& var = lp.vars[static_cast<std::size_t>(col)];
        if (var.type != VarType::Binary) has_continuous_or_general = true;
        const double activity_bound = (a >= 0.0) ? var.lb : var.ub;
        if (!finite_model_bound(activity_bound)) return;
        const double min_contrib = a * activity_bound;
        min_activity += min_contrib;
        const int cp = cand_pos[static_cast<std::size_t>(col)];
        if (cp < 0) continue;
        const double fav_value =
            candidates[static_cast<std::size_t>(cp)].favorable_one ? 1.0 : 0.0;
        const double lift = a * fav_value - min_contrib;
        if (lift > 1e-12) {
          row_cands.push_back(
              RowCand{cp, lift, candidates[static_cast<std::size_t>(cp)].penalty});
        }
      }
      if (row_cands.size() < 2) return;
      if (has_continuous_or_general) ++mixed_rows_scanned;
      if (row_cands.size() > 256) {
        std::sort(row_cands.begin(), row_cands.end(),
                  [](const RowCand& a, const RowCand& b) {
                    if (a.penalty != b.penalty) return a.penalty > b.penalty;
                    return a.lift > b.lift;
                  });
        row_cands.resize(256);
      }
      const int k = static_cast<int>(row_cands.size());
      for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j < k; ++j) {
          ++objective_clique_pair_tests;
          if (min_activity + row_cands[static_cast<std::size_t>(i)].lift +
                  row_cands[static_cast<std::size_t>(j)].lift >
              rhs + 1e-9) {
            ++objective_clique_pair_conflicts;
            add_edge(row_cands[static_cast<std::size_t>(i)].cand,
                     row_cands[static_cast<std::size_t>(j)].cand);
          }
        }
      }
    };

    Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
    std::vector<std::pair<int, double>> row;
    row.reserve(128);
    for (int r = 0; r < Arow.rows(); ++r) {
      row.clear();
      bool too_large = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
           it; ++it) {
        const int col = it.col();
        if (col < 0 || col >= n) continue;
        row.emplace_back(col, it.value());
        if (row.size() > 512) {
          too_large = true;
          break;
        }
      }
      if (!too_large) scan_leq_row(row, lp.b[r]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, r);
      if (!too_large && std::isfinite(lhs)) {
        for (auto& entry : row) entry.second = -entry.second;
        scan_leq_row(row, -lhs);
      }
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeqrow = lp.Aeq;
    for (int r = 0; r < Aeqrow.rows(); ++r) {
      row.clear();
      bool too_large = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeqrow, r);
           it; ++it) {
        const int col = it.col();
        if (col < 0 || col >= n) continue;
        row.emplace_back(col, it.value());
        if (row.size() > 512) {
          too_large = true;
          break;
        }
      }
      if (too_large) continue;
      scan_leq_row(row, lp.beq[r]);
      for (auto& entry : row) entry.second = -entry.second;
      scan_leq_row(row, -lp.beq[r]);
    }

    for (auto& nbs : adj) {
      std::sort(nbs.begin(), nbs.end());
      nbs.erase(std::unique(nbs.begin(), nbs.end()), nbs.end());
    }

    std::vector<int> order(candidates.size());
    for (int i = 0; i < static_cast<int>(order.size()); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      const auto& ca = candidates[static_cast<std::size_t>(a)];
      const auto& cb = candidates[static_cast<std::size_t>(b)];
      if (ca.penalty != cb.penalty) return ca.penalty > cb.penalty;
      return ca.col < cb.col;
    });
    std::vector<char> used(candidates.size(), 0);
    for (int seed : order) {
      if (used[static_cast<std::size_t>(seed)] ||
          adj[static_cast<std::size_t>(seed)].empty()) {
        continue;
      }
      std::vector<int> clique{seed};
      used[static_cast<std::size_t>(seed)] = 1;
      for (int cand : order) {
        if (used[static_cast<std::size_t>(cand)] || cand == seed) continue;
        bool all_conflict = true;
        for (int member : clique) {
          const auto& nbs = adj[static_cast<std::size_t>(cand)];
          if (!std::binary_search(nbs.begin(), nbs.end(), member)) {
            all_conflict = false;
            break;
          }
        }
        if (!all_conflict) continue;
        clique.push_back(cand);
        used[static_cast<std::size_t>(cand)] = 1;
      }
      if (clique.size() < 2) {
        used[static_cast<std::size_t>(seed)] = 0;
        continue;
      }
      const int pid = static_cast<int>(partitions.size());
      Partition part;
      part.terms.reserve(clique.size());
      for (int cp : clique) {
        const Candidate& c = candidates[static_cast<std::size_t>(cp)];
        const double fav = c.favorable_one ? c.cost : 0.0;
        const double nonfav = c.favorable_one ? 0.0 : c.cost;
        part.terms.push_back(
            PartitionTerm{c.col, c.cost, c.favorable_one, fav, nonfav,
                          nonfav - fav});
        max_replacement_delta =
            std::max(max_replacement_delta, std::max(0.0, nonfav - fav));
        partition_of_col[static_cast<std::size_t>(c.col)] = pid;
      }
      partitions.push_back(std::move(part));
    }
  }

  void build_objective_clique_partitions_from_implications(
      const LPModel& lp,
      const std::vector<char>* implied_integer_cols,
      const CliqueTable* clique_table,
      const BinaryImplicationGraph* implication_graph) {
    const int n = static_cast<int>(lp.vars.size());
    if (n <= 1 || (clique_table == nullptr && implication_graph == nullptr)) {
      return;
    }

    auto binary_like = [&](int j) {
      if (j < 0 || j >= n) return false;
      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      if (var.lb > 1e-9 || var.ub < 1.0 - 1e-9) return false;
      if (var.type == VarType::Binary) return true;
      const bool implied_integer =
          implied_integer_cols != nullptr &&
          j < static_cast<int>(implied_integer_cols->size()) &&
          (*implied_integer_cols)[static_cast<std::size_t>(j)];
      return implied_integer && std::abs(var.lb) <= 1e-9 &&
             std::abs(var.ub - 1.0) <= 1e-9;
    };

    struct Candidate {
      int col{-1};
      double cost{0.0};
      bool favorable_one{false};
      double penalty{0.0};
    };
    std::vector<Candidate> candidates;
    candidates.reserve(terms.size());
    for (const Term& term : terms) {
      const int j = term.col;
      if (!binary_like(j)) continue;
      if (j < static_cast<int>(partition_of_col.size()) &&
          partition_of_col[static_cast<std::size_t>(j)] >= 0) {
        continue;
      }
      candidates.push_back(Candidate{j, term.cost, term.cost < 0.0,
                                     std::abs(term.cost)});
    }
    if (candidates.size() <= 1) return;
    objective_clique_candidate_literals +=
        static_cast<std::uint64_t>(candidates.size());

    auto literals_conflict = [&](const Candidate& a,
                                 const Candidate& b,
                                 bool& via_literal,
                                 bool& via_implication) {
      via_literal = false;
      via_implication = false;
      if (a.col == b.col) return false;

      if (clique_table != nullptr &&
          clique_table->has_literal_edge(a.col, a.favorable_one,
                                         b.col, b.favorable_one)) {
        via_literal = true;
        return true;
      }

      if (implication_graph == nullptr || implication_graph->empty()) {
        return false;
      }
      const bool b_opposite = !b.favorable_one;
      const bool a_opposite = !a.favorable_one;
      via_implication =
          implication_graph->implies_binary_value(
              a.col, a.favorable_one, b.col, b_opposite) ||
          implication_graph->implies_binary_value(
              b.col, b.favorable_one, a.col, a_opposite);
      return via_implication;
    };

    std::vector<std::vector<int>> adj(candidates.size());
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
      for (int j = i + 1; j < static_cast<int>(candidates.size()); ++j) {
        ++objective_clique_pair_tests;
        bool via_literal = false;
        bool via_implication = false;
        if (!literals_conflict(candidates[static_cast<std::size_t>(i)],
                               candidates[static_cast<std::size_t>(j)],
                               via_literal, via_implication)) {
          continue;
        }
        ++objective_clique_pair_conflicts;
        if (via_literal) ++objective_clique_literal_conflicts;
        if (via_implication) ++objective_clique_implication_conflicts;
        adj[static_cast<std::size_t>(i)].push_back(j);
        adj[static_cast<std::size_t>(j)].push_back(i);
        ++clique_edges;
      }
    }
    for (auto& nbs : adj) {
      std::sort(nbs.begin(), nbs.end());
      nbs.erase(std::unique(nbs.begin(), nbs.end()), nbs.end());
    }

    std::vector<int> order(candidates.size());
    for (int i = 0; i < static_cast<int>(order.size()); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      const Candidate& ca = candidates[static_cast<std::size_t>(a)];
      const Candidate& cb = candidates[static_cast<std::size_t>(b)];
      if (ca.penalty != cb.penalty) return ca.penalty > cb.penalty;
      return ca.col < cb.col;
    });

    std::vector<char> used(candidates.size(), 0);
    for (int seed : order) {
      if (used[static_cast<std::size_t>(seed)] ||
          adj[static_cast<std::size_t>(seed)].empty()) {
        continue;
      }

      std::vector<int> clique{seed};
      used[static_cast<std::size_t>(seed)] = 1;
      for (int cand : order) {
        if (cand == seed || used[static_cast<std::size_t>(cand)]) continue;
        bool all_conflict = true;
        const auto& cand_adj = adj[static_cast<std::size_t>(cand)];
        for (int member : clique) {
          if (!std::binary_search(cand_adj.begin(), cand_adj.end(), member)) {
            all_conflict = false;
            break;
          }
        }
        if (!all_conflict) continue;
        clique.push_back(cand);
        used[static_cast<std::size_t>(cand)] = 1;
      }
      if (clique.size() < 2) {
        used[static_cast<std::size_t>(seed)] = 0;
        continue;
      }

      const int pid = static_cast<int>(partitions.size());
      Partition part;
      part.terms.reserve(clique.size());
      for (int cp : clique) {
        const Candidate& c = candidates[static_cast<std::size_t>(cp)];
        const double fav = c.favorable_one ? c.cost : 0.0;
        const double nonfav = c.favorable_one ? 0.0 : c.cost;
        part.terms.push_back(
            PartitionTerm{c.col, c.cost, c.favorable_one, fav, nonfav,
                          nonfav - fav});
        max_replacement_delta =
            std::max(max_replacement_delta, std::max(0.0, nonfav - fav));
        partition_of_col[static_cast<std::size_t>(c.col)] = pid;
      }
      partitions.push_back(std::move(part));
    }
  }

  void build_implied_contribution_events(
      const LPModel& lp,
      const std::vector<char>* implied_integer_cols,
      const BuildPolicy& build_policy) {
    const int n = static_cast<int>(lp.vars.size());
    constexpr int kMaxRowNnz = 32;
    constexpr int kMaxRowLiterals = 24;
    constexpr std::size_t kMaxEvents = 100000;

    auto is_implied_integer = [&](int j) {
      return implied_integer_cols != nullptr &&
             j >= 0 && j < static_cast<int>(implied_integer_cols->size()) &&
             (*implied_integer_cols)[static_cast<std::size_t>(j)];
    };
    auto is_integral_or_implied = [&](int j) {
      if (j < 0 || j >= n) return false;
      return is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
             is_implied_integer(j);
    };
    auto integral_bounds = [&](int j) {
      if (j < 0 || j >= n) return false;
      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      auto integral_if_finite = [](double value) {
        return !std::isfinite(value) ||
               std::abs(value - std::round(value)) <= 1e-7;
      };
      return integral_if_finite(var.lb) && integral_if_finite(var.ub);
    };
    auto binary_like = [&](int j) {
      if (j < 0 || j >= n) return false;
      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      if (var.lb > 1e-9 || var.ub < 1.0 - 1e-9) return false;
      if (var.type == VarType::Binary) return true;
      return is_implied_integer(j) && std::abs(var.lb) <= 1e-9 &&
             std::abs(var.ub - 1.0) <= 1e-9;
    };
    auto rounded_integral_bound = [&](int j, bool lower_bound,
                                      double value) {
      if (!is_integral_or_implied(j)) return value;
      const double rounded = lower_bound ? std::ceil(value - 1e-9)
                                         : std::floor(value + 1e-9);
      if (lower_bound ? (rounded > value + 1e-9)
                      : (rounded < value - 1e-9)) {
        ++implied_event_integral_rounds;
      }
      return rounded;
    };

    auto scan_leq_row = [&](const std::vector<std::pair<int, double>>& row,
                            double rhs) {
      if (row.size() < 2 || row.size() > kMaxRowNnz || !std::isfinite(rhs) ||
          implied_events.size() >= kMaxEvents) {
        return;
      }

      for (const auto& [target, target_a] : row) {
        if (target < 0 || target >= n || std::abs(target_a) <= 1e-12 ||
            target >= lp.c.size()) {
          continue;
        }
        const double target_cost = lp.c[target];
        if (!std::isfinite(target_cost)) continue;
        const auto& target_var = lp.vars[static_cast<std::size_t>(target)];
        const bool target_lower_event =
            target_a < -1e-12 && target_cost > 1e-12 &&
            finite_model_bound(target_var.lb);
        const bool target_upper_event =
            target_a > 1e-12 && target_cost < -1e-12 &&
            finite_model_bound(target_var.ub);
        if (!target_lower_event && !target_upper_event) continue;
        const bool target_is_lower_bound = target_lower_event;
        const double denom = std::abs(target_a);

        struct RowLiteral {
          Literal lit;
          double lift{0.0};
          double row_coeff{0.0};
          double min_contrib{0.0};
        };
        std::vector<RowLiteral> literals;
        literals.reserve(row.size() * 2);
        double min_rest_activity = 0.0;
        bool valid = true;

        for (const auto& [col, a] : row) {
          if (col < 0 || col >= n || col == target ||
              std::abs(a) <= 1e-12 || !std::isfinite(a)) {
            continue;
          }
          const auto& var = lp.vars[static_cast<std::size_t>(col)];
          const double min_bound = (a >= 0.0) ? var.lb : var.ub;
          if (!finite_model_bound(min_bound)) {
            valid = false;
            break;
          }
          const double min_contrib = a * min_bound;
          min_rest_activity += min_contrib;

          if (!binary_like(col)) continue;
          const double lift_zero = -min_contrib;
          const double lift_one = a - min_contrib;
          if (lift_zero > 1e-9) {
            literals.push_back(
                RowLiteral{Literal{col, false}, lift_zero, a, min_contrib});
          }
          if (lift_one > 1e-9) {
            literals.push_back(
                RowLiteral{Literal{col, true}, lift_one, a, min_contrib});
          }
        }
        if (!valid || literals.empty()) continue;
        if (literals.size() > kMaxRowLiterals) {
          std::sort(literals.begin(), literals.end(),
                    [](const RowLiteral& a, const RowLiteral& b) {
                      return a.lift > b.lift;
                    });
          literals.resize(kMaxRowLiterals);
        }

        auto add_event = [&](std::vector<Literal> lits, double lift_sum,
                             const RowLiteral* single_literal) {
          if (implied_events.size() >= kMaxEvents) return;
          double implied_bound = target_is_lower_bound
              ? (min_rest_activity + lift_sum - rhs) / denom
              : (rhs - min_rest_activity - lift_sum) / denom;
          if (!std::isfinite(implied_bound)) return;

          const bool target_integral =
              is_integral_or_implied(target) && integral_bounds(target);
          if (build_policy.strengthen_row_events_with_mir && target_integral &&
              single_literal != nullptr) {
            ++implied_event_mir_attempts;
            const double trigger_value =
                single_literal->lit.value_one ? 1.0 : 0.0;
            const double other_min =
                min_rest_activity - single_literal->min_contrib;
            double vbd_constant = target_is_lower_bound
                ? (other_min - rhs) / denom
                : (rhs - other_min) / denom;
            double vbd_coef = target_is_lower_bound
                ? single_literal->row_coeff / denom
                : -single_literal->row_coeff / denom;
            const double raw_active_bound = vbd_constant +
                                            vbd_coef * trigger_value;
            const bool mir_ok = strengthen_binary_var_bound_mir(
                vbd_coef, vbd_constant, target_is_lower_bound);
            if (mir_ok) {
              const double mir_active_bound = vbd_constant +
                                              vbd_coef * trigger_value;
              const double mir_gain = target_is_lower_bound
                  ? (mir_active_bound - raw_active_bound)
                  : (raw_active_bound - mir_active_bound);
              if (mir_gain > 1e-9 && std::isfinite(mir_gain)) {
                implied_bound = mir_active_bound;
                ++implied_event_mir_strengthened;
                implied_event_mir_gain_sum += mir_gain;
                implied_event_mir_gain_max =
                    std::max(implied_event_mir_gain_max, mir_gain);
              }
            }
          }
          implied_bound =
              rounded_integral_bound(target, target_is_lower_bound,
                                     implied_bound);
          // Clamp the implied bound to the variable's model domain. An implied
          // bound beyond the domain would assert infeasibility-on-trigger;
          // pinning it at the domain boundary yields a valid (weaker) lower
          // bound on the forced objective contribution and prevents
          // numerically degenerate (tiny-denominator) blow-ups from producing
          // unbounded, unsound contributions.
          if (target_is_lower_bound) {
            if (finite_model_bound(target_var.ub)) {
              implied_bound = std::min(implied_bound, target_var.ub);
            }
          } else {
            if (finite_model_bound(target_var.lb)) {
              implied_bound = std::max(implied_bound, target_var.lb);
            }
          }
          double contribution = 0.0;
          if (target_is_lower_bound) {
            if (implied_bound <= target_var.lb + 1e-8) return;
            contribution = target_cost * (implied_bound - target_var.lb);
          } else {
            if (implied_bound >= target_var.ub - 1e-8) return;
            contribution = (-target_cost) * (target_var.ub - implied_bound);
          }
          if (contribution <= 1e-7 || !std::isfinite(contribution)) return;
          max_implied_event_delta =
              std::max(max_implied_event_delta, contribution);
          const int event_idx = static_cast<int>(implied_events.size());
          implied_event_target_cols[static_cast<std::size_t>(target)] = 1;
          implied_events.push_back(ImpliedContributionEvent{
              target, implied_bound, target_cost, target_is_lower_bound,
              std::move(lits)});
          if (target_is_lower_bound) {
            ++implied_event_vlb_events;
          } else {
            ++implied_event_vub_events;
          }
          for (const Literal& lit : implied_events.back().literals) {
            if (lit.col < 0 || lit.col >= n) continue;
            const std::size_t key =
                static_cast<std::size_t>(2 * lit.col + (lit.value_one ? 1 : 0));
            implied_events_by_literal[key].push_back(event_idx);
          }
        };

        for (const RowLiteral& lit : literals) {
          add_event(std::vector<Literal>{lit.lit}, lit.lift, &lit);
        }
        if (build_policy.build_pair_row_implied_events) {
          for (std::size_t i = 0; i < literals.size(); ++i) {
            for (std::size_t j = i + 1; j < literals.size(); ++j) {
              if (literals[i].lit.col == literals[j].lit.col) continue;
              add_event(std::vector<Literal>{literals[i].lit, literals[j].lit},
                        literals[i].lift + literals[j].lift, nullptr);
            }
          }
        }
      }
    };

    Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
    std::vector<std::pair<int, double>> row;
    row.reserve(kMaxRowNnz);
    for (int r = 0; r < Arow.rows(); ++r) {
      row.clear();
      bool too_large = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
           it; ++it) {
        const int col = it.col();
        if (col < 0 || col >= n) continue;
        row.emplace_back(col, it.value());
        if (row.size() > kMaxRowNnz) {
          too_large = true;
          break;
        }
      }
      if (!too_large) scan_leq_row(row, lp.b[r]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, r);
      if (!too_large && std::isfinite(lhs)) {
        for (auto& entry : row) entry.second = -entry.second;
        scan_leq_row(row, -lhs);
      }
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeqrow = lp.Aeq;
    for (int r = 0; r < Aeqrow.rows(); ++r) {
      row.clear();
      bool too_large = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeqrow, r);
           it; ++it) {
        const int col = it.col();
        if (col < 0 || col >= n) continue;
        row.emplace_back(col, it.value());
        if (row.size() > kMaxRowNnz) {
          too_large = true;
          break;
        }
      }
      if (too_large) continue;
      scan_leq_row(row, lp.beq[r]);
      for (auto& entry : row) entry.second = -entry.second;
      scan_leq_row(row, -lp.beq[r]);
    }
  }

  void build_implied_contribution_events_from_implications(
      const LPModel& lp,
      const BinaryImplicationGraph* implication_graph) {
    if (implication_graph == nullptr || implication_graph->empty()) return;
    const int n = static_cast<int>(lp.vars.size());
    if (n <= 0 || implied_events_by_literal.size() < static_cast<std::size_t>(2 * n)) {
      return;
    }
    constexpr std::size_t kMaxEvents = 100000;
    auto binary_like = [&](int j) {
      if (j < 0 || j >= n) return false;
      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      return var.lb <= 1e-9 && var.ub >= 1.0 - 1e-9 &&
             var.type == VarType::Binary;
    };
    auto add_graph_event = [&](int trigger, bool trigger_one,
                               const BinaryImplicationGraph::Implication& imp) {
      if (implied_events.size() >= kMaxEvents) return;
      const int target = imp.var_idx;
      if (trigger < 0 || trigger >= n || target < 0 || target >= n ||
          trigger == target || !binary_like(trigger)) {
        return;
      }
      const double target_cost = lp.c[target];
      if (!std::isfinite(target_cost) || std::abs(target_cost) <= 1e-12 ||
          !std::isfinite(imp.value)) {
        return;
      }
      const auto& target_var = lp.vars[static_cast<std::size_t>(target)];
      const bool lower_event =
          imp.is_lb && target_cost > 1e-12 &&
          finite_model_bound(target_var.lb) &&
          imp.value > target_var.lb + 1e-8;
      const bool upper_event =
          !imp.is_lb && target_cost < -1e-12 &&
          finite_model_bound(target_var.ub) &&
          imp.value < target_var.ub - 1e-8;
      if (!lower_event && !upper_event) return;
      // Clamp the implied bound to the variable's model domain (see the
      // matching rationale in build_implied_contribution_events); an
      // out-of-domain implied bound must not inflate the forced objective
      // contribution.
      double bound_value = imp.value;
      if (lower_event) {
        if (finite_model_bound(target_var.ub)) {
          bound_value = std::min(bound_value, target_var.ub);
        }
        if (bound_value <= target_var.lb + 1e-8) return;
      } else {
        if (finite_model_bound(target_var.lb)) {
          bound_value = std::max(bound_value, target_var.lb);
        }
        if (bound_value >= target_var.ub - 1e-8) return;
      }
      const double contribution = lower_event
          ? target_cost * (bound_value - target_var.lb)
          : (-target_cost) * (target_var.ub - bound_value);
      if (!(contribution > 1e-7) || !std::isfinite(contribution)) return;
      max_implied_event_delta =
          std::max(max_implied_event_delta, contribution);
      const int event_idx = static_cast<int>(implied_events.size());
      implied_event_target_cols[static_cast<std::size_t>(target)] = 1;
      implied_events.push_back(ImpliedContributionEvent{
          target, bound_value, target_cost, lower_event,
          std::vector<Literal>{Literal{trigger, trigger_one}}});
      if (lower_event) {
        ++implied_event_vlb_events;
      } else {
        ++implied_event_vub_events;
      }
      const std::size_t key =
          static_cast<std::size_t>(2 * trigger + (trigger_one ? 1 : 0));
      implied_events_by_literal[key].push_back(event_idx);
    };

    for (int trigger = 0; trigger < n; ++trigger) {
      for (bool trigger_one : {false, true}) {
        auto range = implication_graph->implications(trigger, trigger_one);
        for (const auto* p = range.first; p != range.second; ++p) {
          if (p == nullptr) break;
          add_graph_event(trigger, trigger_one, *p);
          if (implied_events.size() >= kMaxEvents) return;
        }
      }
    }
  }

  void recompute_max_implied_event_aggregate_delta(const LPModel& lp) {
    max_implied_event_aggregate_delta = max_implied_event_delta;
    const int n = static_cast<int>(lp.vars.size());
    if (n <= 0 || implied_events.empty() ||
        implied_events_by_literal.empty()) {
      return;
    }
    for (const auto& event_ids : implied_events_by_literal) {
      if (event_ids.empty()) continue;
      std::unordered_map<int, double> best_by_target;
      best_by_target.reserve(event_ids.size());
      for (int event_id : event_ids) {
        if (event_id < 0 ||
            event_id >= static_cast<int>(implied_events.size())) {
          continue;
        }
        const ImpliedContributionEvent& event =
            implied_events[static_cast<std::size_t>(event_id)];
        const int target = event.target_col;
        if (target < 0 || target >= n ||
            objective_partition_owns_col(target)) {
          continue;
        }
        const auto& target_var = lp.vars[static_cast<std::size_t>(target)];
        const double contribution = event.target_is_lower_bound
            ? event.target_cost * (event.target_bound - target_var.lb)
            : (-event.target_cost) * (target_var.ub - event.target_bound);
        if (!(contribution > 1e-9) || !std::isfinite(contribution)) continue;
        auto [it, inserted] = best_by_target.emplace(target, contribution);
        if (!inserted && contribution > it->second) {
          it->second = contribution;
        }
      }
      double aggregate = 0.0;
      for (const auto& [target, contribution] : best_by_target) {
        (void)target;
        aggregate += contribution;
      }
      max_implied_event_aggregate_delta =
          std::max(max_implied_event_aggregate_delta, aggregate);
    }
  }

  struct Activity {
    double lower{0.0};
    int inf_count{0};
    int inf_col{-1};
    std::vector<double> min_contribution;
    std::vector<double> contribution_delta;
    double max_delta{0.0};
  };

  struct CapacityConformanceAudit {
    bool valid{false};
    double raw_lower{kInf};
    double raw_capacity{kInf};
    double proof_lower{kInf};
    double proof_capacity{kInf};
    double max_domain_delta{0.0};
    double max_objective_delta{0.0};
    double max_aggregate_delta{0.0};
    std::uint64_t target_tightening_events{0};
    std::uint64_t active_events{0};
    std::uint64_t active_tightening_events{0};
    std::uint64_t one_missing_events{0};
    std::uint64_t one_missing_keys{0};
    std::uint64_t one_missing_raw_exceed_keys{0};
    std::uint64_t one_missing_proof_exceed_keys{0};
    double max_one_missing_contribution{0.0};
    double max_raw_excess{0.0};
    double max_proof_excess{0.0};
  };

  struct ProofCapacityContext {
    const Eigen::SparseVector<double>* proof_coeff{nullptr};
    const Eigen::VectorXd* x_relax{nullptr};
    const Eigen::VectorXd* reduced_costs{nullptr};
    double proof_lower_bound{kInf};
	    double upper_limit{kInf};
    double int_tol{1e-9};
    double lp_tol{1e-9};
    int depth{0};
    int trail_offset{0};
  };

  struct ProofCapacityPropagationStats {
    std::uint64_t candidates{0};
    std::uint64_t fixings{0};
    std::uint64_t prunes{0};
    std::uint64_t skip_coeff{0};
    std::uint64_t skip_budget{0};
    std::uint64_t skip_no_lp_or_queue_hit{0};
    std::uint64_t rc_frontier_candidates{0};
    std::uint64_t rc_frontier_fixings{0};
    std::uint64_t rc_frontier_skip_budget{0};
    double max_proof_lift{0.0};
    double max_proof_excess{0.0};
  };

  Activity compute_activity(const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub,
                            int n) const {
    Activity a;
    a.min_contribution.assign(static_cast<std::size_t>(n), 0.0);
    a.contribution_delta.assign(static_cast<std::size_t>(n), 0.0);
    for (const Partition& part : partitions) {
      double nonfav_sum = 0.0;
      double best_delta = -1.0;
      int best_col = -1;
      bool fixed_favorable = false;
      double partition_lower = 0.0;
      for (const PartitionTerm& t : part.terms) {
        const bool fav_possible = t.favorable_one
            ? (ub[t.col] >= 1.0 - 1e-9)
            : (lb[t.col] <= 1e-9);
        const bool fav_forced = t.favorable_one
            ? (lb[t.col] >= 1.0 - 1e-9)
            : (ub[t.col] <= 1e-9);
        nonfav_sum += t.nonfavorable_contribution;
        if (fav_forced) {
          fixed_favorable = true;
          partition_lower += t.favorable_contribution;
        } else {
          partition_lower += t.nonfavorable_contribution;
          if (fav_possible && t.replacement_delta > best_delta) {
            best_delta = t.replacement_delta;
            best_col = t.col;
          }
        }
      }
      if (!fixed_favorable && best_col >= 0) {
        partition_lower = nonfav_sum - std::max(0.0, best_delta);
      }
      a.lower += partition_lower;
      for (const PartitionTerm& t : part.terms) {
        a.min_contribution[static_cast<std::size_t>(t.col)] = partition_lower;
        a.contribution_delta[static_cast<std::size_t>(t.col)] =
            std::max(0.0, t.replacement_delta);
        a.max_delta = std::max(a.max_delta, std::max(0.0, t.replacement_delta));
      }
    }
    for (const Term& t : terms) {
      if (t.col < 0 || t.col >= n) continue;
      if (t.col < static_cast<int>(partition_of_col.size()) &&
          partition_of_col[static_cast<std::size_t>(t.col)] >= 0) {
        continue;
      }
      const double c = t.cost;
      if (c > 0.0) {
        if (!std::isfinite(lb[t.col])) {
          ++a.inf_count;
          a.inf_col = t.col;
          continue;
        }
        const double contrib = c * lb[t.col];
        a.min_contribution[static_cast<std::size_t>(t.col)] = contrib;
        a.lower += contrib;
        if (std::isfinite(ub[t.col])) {
          const double delta = std::max(0.0, c * (ub[t.col] - lb[t.col]));
          a.contribution_delta[static_cast<std::size_t>(t.col)] = delta;
          a.max_delta = std::max(a.max_delta, delta);
        }
      } else {
        if (!std::isfinite(ub[t.col])) {
          ++a.inf_count;
          a.inf_col = t.col;
          continue;
        }
        const double contrib = c * ub[t.col];
        a.min_contribution[static_cast<std::size_t>(t.col)] = contrib;
        a.lower += contrib;
        if (std::isfinite(lb[t.col])) {
          const double delta = std::max(0.0, c * (lb[t.col] - ub[t.col]));
          a.contribution_delta[static_cast<std::size_t>(t.col)] = delta;
          a.max_delta = std::max(a.max_delta, delta);
        }
      }
    }
    return a;
  }

  double objective_lower_bound(const LPModel& lp,
                               const Eigen::VectorXd& lb,
                               const Eigen::VectorXd& ub) const {
    if (lp.sense != Sense::Minimize) return -kInf;
    const int n = static_cast<int>(lp.vars.size());
    if (n < 0 || lb.size() < n || ub.size() < n) return -kInf;
    const Activity act = compute_activity(lb, ub, n);
    if (act.inf_count != 0 || !std::isfinite(act.lower)) return -kInf;
    return act.lower;
  }

  CapacityConformanceAudit audit_capacity_conformance(
      const LPModel& lp,
      const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub,
	      double upper_limit,
      double proof_lower_bound,
      double int_tol) const {
    CapacityConformanceAudit audit;
	    if (empty() || lp.sense != Sense::Minimize ||
	        !std::isfinite(upper_limit)) {
      return audit;
    }
    const int n = static_cast<int>(lp.vars.size());
    if (lb.size() < n || ub.size() < n) return audit;
	    const double obj_cutoff = upper_limit;
    const double tol = std::max(1e-9, int_tol);
    const Activity act = compute_activity(lb, ub, n);
    audit.valid = true;
    audit.raw_lower = act.lower;
    audit.raw_capacity = obj_cutoff - act.lower;
    audit.proof_lower = proof_lower_bound;
    audit.proof_capacity = std::isfinite(proof_lower_bound)
        ? obj_cutoff - proof_lower_bound
        : kInf;
    audit.max_domain_delta = act.max_delta;
    audit.max_aggregate_delta = max_implied_event_aggregate_delta;
    audit.max_objective_delta =
        std::max({act.max_delta, max_implied_event_delta,
                  max_implied_event_aggregate_delta});

    auto literal_active = [&](const Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (lb[lit.col] >= 1.0 - tol)
                            : (ub[lit.col] <= tol));
    };
    auto literal_possible = [&](const Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (ub[lit.col] >= 1.0 - tol)
                            : (lb[lit.col] <= tol));
    };

    struct AggregateEvent {
      double contribution{0.0};
      std::unordered_map<int, double> best_by_target;
    };
    std::unordered_map<int, AggregateEvent> aggregates;
    aggregates.reserve(std::min<std::size_t>(
        implied_events.size(), static_cast<std::size_t>(512)));

    for (const ImpliedContributionEvent& event : implied_events) {
      const int target = event.target_col;
      if (target < 0 || target >= n ||
          objective_partition_owns_col(target)) {
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
      ++audit.target_tightening_events;

      int missing_key = -1;
      bool impossible = false;
      bool all_active = true;
      for (const Literal& lit : event.literals) {
        if (literal_active(lit)) continue;
        all_active = false;
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

      if (all_active) {
        ++audit.active_events;
        ++audit.active_tightening_events;
        continue;
      }
      if (impossible || missing_key < 0) continue;

      ++audit.one_missing_events;
      AggregateEvent& aggregate = aggregates[missing_key];
      auto [it, inserted] = aggregate.best_by_target.emplace(target, contribution);
      if (inserted) {
        aggregate.contribution += contribution;
      } else if (contribution > it->second + 1e-9) {
        aggregate.contribution += contribution - it->second;
        it->second = contribution;
      }
    }

    audit.one_missing_keys = static_cast<std::uint64_t>(aggregates.size());
    for (const auto& [key, aggregate] : aggregates) {
      (void)key;
      audit.max_one_missing_contribution =
          std::max(audit.max_one_missing_contribution,
                   aggregate.contribution);
      if (std::isfinite(audit.raw_capacity) &&
          aggregate.contribution > audit.raw_capacity + 1e-9) {
        ++audit.one_missing_raw_exceed_keys;
        audit.max_raw_excess =
            std::max(audit.max_raw_excess,
                     aggregate.contribution - audit.raw_capacity);
      }
      if (std::isfinite(audit.proof_capacity) &&
          aggregate.contribution > audit.proof_capacity + 1e-9) {
        ++audit.one_missing_proof_exceed_keys;
        audit.max_proof_excess =
            std::max(audit.max_proof_excess,
                     aggregate.contribution - audit.proof_capacity);
      }
    }
    return audit;
  }

  bool propagate_proof_capacity_one_missing(
      const LPModel& lp,
      const ProofCapacityContext& ctx,
      Eigen::VectorXd& lb,
      Eigen::VectorXd& ub,
      std::vector<DomainReasonBound>* reasons_out,
      int& tightened,
      int& pruned,
      ProofCapacityPropagationStats* stats = nullptr) const {
    tightened = 0;
    pruned = 0;
    if (empty() || lp.sense != Sense::Minimize ||
        ctx.proof_coeff == nullptr ||
        ctx.proof_coeff->size() < static_cast<Eigen::Index>(lp.vars.size()) ||
	        !std::isfinite(ctx.upper_limit) ||
        !std::isfinite(ctx.proof_lower_bound)) {
      return true;
    }
    const int n = static_cast<int>(lp.vars.size());
    if (lb.size() < n || ub.size() < n) return true;
    const double tol = std::max(1e-9, ctx.int_tol);
    const double proof_tol =
        std::max({1e-7, ctx.lp_tol,
	                  1e-10 * std::max(1.0, std::abs(ctx.upper_limit))});
	    const double obj_cutoff = ctx.upper_limit;
    const double capacity = obj_cutoff - ctx.proof_lower_bound;
    if (!std::isfinite(capacity)) return true;
    if (capacity < -proof_tol) {
      ++pruned;
      if (stats != nullptr) ++stats->prunes;
      return false;
    }

    auto literal_active = [&](const Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (lb[lit.col] >= 1.0 - tol)
                            : (ub[lit.col] <= tol));
    };
    auto literal_possible = [&](const Literal& lit) {
      return lit.col >= 0 && lit.col < n &&
             (lit.value_one ? (ub[lit.col] >= 1.0 - tol)
                            : (lb[lit.col] <= tol));
    };
    auto literal_reason = [](const Literal& lit) {
      return BranchDomainLiteral{lit.col, lit.value_one ? 1.0 : 0.0,
                                 lit.value_one};
    };
    auto add_reason_literal = [](std::vector<BranchDomainLiteral>& reason,
                                 const BranchDomainLiteral& literal) {
      const auto found = std::find_if(
          reason.begin(), reason.end(), [&](const BranchDomainLiteral& r) {
            return r.var_idx == literal.var_idx && r.is_lb == literal.is_lb &&
                   std::abs(r.value - literal.value) <= 1e-9;
          });
      if (found == reason.end()) reason.push_back(literal);
    };
	    auto record_reason = [&](const BranchDomainLiteral& bound,
	                             std::vector<BranchDomainLiteral> reason,
	                             const BranchDomainLiteral& forbidden,
	                             double proof_delta) {
	      if (reasons_out == nullptr) return;
	      canonicalize_branch_literals(reason);
	      const int trail_pos =
	          ctx.trail_offset + static_cast<int>(reasons_out->size());
      DomainReasonBound rb;
      rb.bound = bound;
      rb.reason = std::move(reason);
      rb.trail_pos = trail_pos;
      rb.depth = ctx.depth;
	      rb.source_conflict_literal = forbidden;
	      rb.has_source_conflict_literal = true;
	      rb.source_conflict_clause = rb.reason;
	      rb.source_conflict_clause.push_back(forbidden);
	      canonicalize_branch_literals(rb.source_conflict_clause);
	      rb.has_source_conflict_clause = true;
	      if (std::isfinite(proof_delta)) {
	        rb.has_proof_activity_audit = true;
	        rb.proof_activity_margin = proof_delta - capacity;
	        rb.proof_activity = ctx.proof_lower_bound + proof_delta;
	        rb.proof_required_activity = obj_cutoff;
	      }
	      reasons_out->push_back(std::move(rb));
	    };

    for (int round = 0; round < 4; ++round) {
      int round_tightened = 0;

      // LP-active reduced-cost frontier source.  This mirrors the first
      // HiGHS consumption step: use the current node LP certificate to forbid
      // movement away from an active integer bound when the reduced-cost
      // increase alone exceeds the incumbent capacity.  It is deliberately
      // tied to the node's own x/RC pair, not to the offline VLB event list.
      if (ctx.x_relax != nullptr && ctx.x_relax->size() >= n &&
          ctx.reduced_costs != nullptr && ctx.reduced_costs->size() >= n) {
        const double active_tol = std::max({tol, proof_tol, ctx.lp_tol});
        for (int j = 0; j < n; ++j) {
          if (!is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
            continue;
          }
          if (!std::isfinite(lb[j]) || !std::isfinite(ub[j]) ||
              ub[j] <= lb[j] + tol || !std::isfinite((*ctx.x_relax)[j])) {
            continue;
          }
          const double rc = std::abs((*ctx.reduced_costs)[j]);
          if (!(rc > 1e-12) || !std::isfinite(rc)) continue;
          const double xj = (*ctx.x_relax)[j];
          if (std::abs(xj - lb[j]) <= active_tol) {
            const double proof_delta = rc * (ub[j] - lb[j]);
            if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
              continue;
            }
            if (stats != nullptr) {
              ++stats->candidates;
              ++stats->rc_frontier_candidates;
              stats->max_proof_lift =
                  std::max(stats->max_proof_lift, proof_delta);
            }
            if (!(proof_delta > capacity + proof_tol)) {
              if (stats != nullptr) {
                ++stats->skip_budget;
                ++stats->rc_frontier_skip_budget;
              }
              continue;
            }
            double new_ub = std::floor(lb[j] + capacity / rc + tol);
            new_ub = std::min(ub[j], std::max(lb[j], new_ub));
            if (new_ub < ub[j] - tol) {
              ub[j] = new_ub;
              ++round_tightened;
              ++tightened;
              if (stats != nullptr) {
                ++stats->fixings;
                ++stats->rc_frontier_fixings;
                stats->max_proof_excess =
                    std::max(stats->max_proof_excess,
                             proof_delta - capacity);
              }
	              record_reason(
	                  BranchDomainLiteral{j, new_ub, false}, {},
	                  BranchDomainLiteral{j, std::floor(new_ub + tol) + 1.0,
	                                      true},
	                  proof_delta);
            }
          } else if (std::abs(xj - ub[j]) <= active_tol) {
            const double proof_delta = rc * (ub[j] - lb[j]);
            if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
              continue;
            }
            if (stats != nullptr) {
              ++stats->candidates;
              ++stats->rc_frontier_candidates;
              stats->max_proof_lift =
                  std::max(stats->max_proof_lift, proof_delta);
            }
            if (!(proof_delta > capacity + proof_tol)) {
              if (stats != nullptr) {
                ++stats->skip_budget;
                ++stats->rc_frontier_skip_budget;
              }
              continue;
            }
            double new_lb = std::ceil(ub[j] - capacity / rc - tol);
            new_lb = std::max(lb[j], std::min(ub[j], new_lb));
            if (new_lb > lb[j] + tol) {
              lb[j] = new_lb;
              ++round_tightened;
              ++tightened;
              if (stats != nullptr) {
                ++stats->fixings;
                ++stats->rc_frontier_fixings;
                stats->max_proof_excess =
                    std::max(stats->max_proof_excess,
                             proof_delta - capacity);
              }
	              record_reason(
	                  BranchDomainLiteral{j, new_lb, true}, {},
	                  BranchDomainLiteral{j, std::ceil(new_lb - tol) - 1.0,
	                                      false},
	                  proof_delta);
            }
          }
        }
      }

      // LP-frontier source: if the current proof coefficient on a binary
      // frontier variable already proves that one side of the binary exceeds
      // the cutoff capacity, consume it directly as a scoped objective fixing.
      // This is source-side filtering, not looser admission: the fixing must
      // cut the queued LP point and must be explained by positive proof mass.
      if (ctx.x_relax != nullptr && ctx.x_relax->size() >= n) {
        for (int j = 0; j < n; ++j) {
          if (lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
            continue;
          }
          if (lb[j] > tol || ub[j] < 1.0 - tol ||
              !std::isfinite((*ctx.x_relax)[j])) {
            continue;
          }
          const double xj = (*ctx.x_relax)[j];
          const double a = ctx.proof_coeff->coeff(j);
          if (!std::isfinite(a) || std::abs(a) <= 1e-12) continue;

          auto try_forbid_one = [&]() -> bool {
            if (!(a > 1e-12) || xj <= std::max(proof_tol, ctx.lp_tol) ||
                ub[j] <= tol) {
              return true;
            }
            const double proof_delta = a * (1.0 - lb[j]);
            if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
              return true;
            }
            if (stats != nullptr) {
              ++stats->candidates;
              stats->max_proof_lift =
                  std::max(stats->max_proof_lift, proof_delta);
            }
            if (!(proof_delta > capacity + proof_tol)) {
              if (stats != nullptr) ++stats->skip_budget;
              return true;
            }
            if (0.0 < lb[j] - tol) {
              ++pruned;
              if (stats != nullptr) ++stats->prunes;
              return false;
            }
            if (ub[j] > tol) {
              ub[j] = 0.0;
              ++round_tightened;
              ++tightened;
              if (stats != nullptr) {
                ++stats->fixings;
                stats->max_proof_excess =
                    std::max(stats->max_proof_excess,
                             proof_delta - capacity);
              }
	              record_reason(BranchDomainLiteral{j, 0.0, false}, {},
	                            BranchDomainLiteral{j, 1.0, true},
	                            proof_delta);
            }
            return true;
          };

          auto try_forbid_zero = [&]() -> bool {
            if (!(a < -1e-12) ||
                xj >= 1.0 - std::max(proof_tol, ctx.lp_tol) ||
                lb[j] >= 1.0 - tol) {
              return true;
            }
            const double proof_delta = (-a) * (ub[j] - 0.0);
            if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
              return true;
            }
            if (stats != nullptr) {
              ++stats->candidates;
              stats->max_proof_lift =
                  std::max(stats->max_proof_lift, proof_delta);
            }
            if (!(proof_delta > capacity + proof_tol)) {
              if (stats != nullptr) ++stats->skip_budget;
              return true;
            }
            if (1.0 > ub[j] + tol) {
              ++pruned;
              if (stats != nullptr) ++stats->prunes;
              return false;
            }
            if (lb[j] < 1.0 - tol) {
              lb[j] = 1.0;
              ++round_tightened;
              ++tightened;
              if (stats != nullptr) {
                ++stats->fixings;
                stats->max_proof_excess =
                    std::max(stats->max_proof_excess,
                             proof_delta - capacity);
              }
	              record_reason(BranchDomainLiteral{j, 1.0, true}, {},
	                            BranchDomainLiteral{j, 0.0, false},
	                            proof_delta);
            }
            return true;
          };

          if (!try_forbid_one() || !try_forbid_zero()) return false;
        }
      }

      struct AggregateProofEvent {
        double proof_delta{0.0};
        double objective_delta{0.0};
        std::unordered_map<int, double> best_by_target;
        std::vector<BranchDomainLiteral> reason;
      };
      std::unordered_map<int, AggregateProofEvent> aggregates;
      aggregates.reserve(std::min<std::size_t>(
          implied_events.size(), static_cast<std::size_t>(512)));

      for (const ImpliedContributionEvent& event : implied_events) {
        const int target = event.target_col;
        if (target < 0 || target >= n ||
            objective_partition_owns_col(target)) {
          continue;
        }
        const double a = ctx.proof_coeff->coeff(target);
        double proof_delta = 0.0;
        double objective_delta = 0.0;
        if (event.target_is_lower_bound) {
          if (event.target_bound <= lb[target] + tol) continue;
          if (a > 1e-12) {
            proof_delta = a * (event.target_bound - lb[target]);
          }
          objective_delta = std::max(0.0, event.target_cost) *
                            (event.target_bound - lb[target]);
        } else {
          if (event.target_bound >= ub[target] - tol) continue;
          if (a < -1e-12) {
            proof_delta = (-a) * (ub[target] - event.target_bound);
          }
          objective_delta = std::max(0.0, -event.target_cost) *
                            (ub[target] - event.target_bound);
        }

        int missing_key = -1;
        bool impossible = false;
        std::vector<BranchDomainLiteral> active_reason;
        active_reason.reserve(event.literals.size());
        for (const Literal& lit : event.literals) {
          if (literal_active(lit)) {
            active_reason.push_back(literal_reason(lit));
          } else if (literal_possible(lit)) {
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
        const int missing_col = missing_key / 2;
        if (missing_col < 0 || missing_col >= n) continue;
        if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
          if (stats != nullptr) ++stats->skip_coeff;
          continue;
        }
        AggregateProofEvent& aggregate = aggregates[missing_key];
        auto [best_it, inserted] =
            aggregate.best_by_target.emplace(target, proof_delta);
        if (inserted) {
          aggregate.proof_delta += proof_delta;
          aggregate.objective_delta += std::max(0.0, objective_delta);
          for (const BranchDomainLiteral& r : active_reason) {
            add_reason_literal(aggregate.reason, r);
          }
        } else if (proof_delta > best_it->second + 1e-9) {
          aggregate.proof_delta += proof_delta - best_it->second;
          aggregate.objective_delta += std::max(0.0, objective_delta);
          best_it->second = proof_delta;
          for (const BranchDomainLiteral& r : active_reason) {
            add_reason_literal(aggregate.reason, r);
          }
        }
      }

      for (const auto& [missing_key, aggregate] : aggregates) {
        if (!(aggregate.proof_delta > 1e-9) ||
            !std::isfinite(aggregate.proof_delta)) {
          continue;
        }
        if (stats != nullptr) {
          ++stats->candidates;
          stats->max_proof_lift =
              std::max(stats->max_proof_lift, aggregate.proof_delta);
        }
        if (!(aggregate.proof_delta > capacity + proof_tol)) {
          if (stats != nullptr) ++stats->skip_budget;
          continue;
        }
        if (stats != nullptr) {
          stats->max_proof_excess =
              std::max(stats->max_proof_excess,
                       aggregate.proof_delta - capacity);
        }
        const int col = missing_key / 2;
        const bool value_one = (missing_key % 2) != 0;
        if (col < 0 || col >= n) continue;
        const BranchDomainLiteral forbidden{
            col, value_one ? 1.0 : 0.0, value_one};
        if (value_one) {
          if (0.0 < lb[col] - tol) {
            ++pruned;
            if (stats != nullptr) ++stats->prunes;
            return false;
          }
	            if (ub[col] > tol) {
	              ub[col] = 0.0;
	              ++round_tightened;
	              ++tightened;
	              if (stats != nullptr) ++stats->fixings;
	              record_reason(BranchDomainLiteral{col, 0.0, false},
	                          aggregate.reason, forbidden,
	                          aggregate.proof_delta);
	            }
        } else {
          if (1.0 > ub[col] + tol) {
            ++pruned;
            if (stats != nullptr) ++stats->prunes;
            return false;
          }
	            if (lb[col] < 1.0 - tol) {
	              lb[col] = 1.0;
	              ++round_tightened;
	              ++tightened;
	              if (stats != nullptr) ++stats->fixings;
	              record_reason(BranchDomainLiteral{col, 1.0, true},
	                          aggregate.reason, forbidden,
	                          aggregate.proof_delta);
	            }
        }
      }
      if (round_tightened == 0) break;
    }

    return bounds_consistent(lb, ub);
  }

  bool propagate(const LPModel& lp,
	                 Eigen::VectorXd& lb,
	                 Eigen::VectorXd& ub,
	                 double upper_limit,
                 double int_tol,
                 int depth,
                 int trail_offset,
                 std::vector<DomainReasonBound>* reasons_out,
                 int& tightened,
                 int& pruned) const {
    tightened = 0;
    pruned = 0;
	    if (empty() || lp.sense != Sense::Minimize ||
	        !std::isfinite(upper_limit)) {
      return true;
    }
    const int n = static_cast<int>(lp.vars.size());
    if (lb.size() < n || ub.size() < n) return true;
	    const double obj_cutoff = upper_limit;
    const double tol = std::max(1e-9, int_tol);

	    auto record_reason =
	        [&](int col, bool is_lb, double value,
	            std::vector<BranchDomainLiteral> reason = {},
	            const BranchDomainLiteral* source_conflict_literal = nullptr,
            double proof_delta = 0.0,
            double objective_lower = 0.0,
            double required_activity = 0.0,
            double capacity = 0.0) {
      if (reasons_out == nullptr) return;
      const int trail_pos = trail_offset + static_cast<int>(reasons_out->size());
	      DomainReasonBound rb;
	      rb.bound = BranchDomainLiteral{col, value, is_lb};
	      canonicalize_branch_literals(reason);
	      rb.reason = std::move(reason);
	      rb.trail_pos = trail_pos;
	      rb.depth = depth;
	      if (source_conflict_literal != nullptr &&
	          source_conflict_literal->var_idx == col &&
          source_conflict_literal->is_lb != is_lb &&
          std::isfinite(source_conflict_literal->value) &&
          std::isfinite(proof_delta) && std::isfinite(objective_lower) &&
          std::isfinite(required_activity) && std::isfinite(capacity)) {
	        rb.source_conflict_literal = *source_conflict_literal;
	        rb.has_source_conflict_literal = true;
	        rb.source_conflict_clause = rb.reason;
	        rb.source_conflict_clause.push_back(*source_conflict_literal);
	        canonicalize_branch_literals(rb.source_conflict_clause);
	        rb.has_source_conflict_clause = true;
	        rb.has_proof_activity_audit = true;
	        rb.proof_activity_margin = proof_delta - capacity;
	        rb.proof_activity = objective_lower + proof_delta;
	        rb.proof_required_activity = required_activity;
	      }
      reasons_out->push_back(std::move(rb));
    };
    auto literal_active = [&](const Literal& lit) {
      return lit.value_one ? (lb[lit.col] >= 1.0 - tol) : (ub[lit.col] <= tol);
    };
    auto literal_possible = [&](const Literal& lit) {
      return lit.value_one ? (ub[lit.col] >= 1.0 - tol) : (lb[lit.col] <= tol);
    };
    auto literal_reason = [](const Literal& lit) {
      return BranchDomainLiteral{lit.col, lit.value_one ? 1.0 : 0.0,
                                 lit.value_one};
    };
	    auto flip_bound_literal = [&](const BranchDomainLiteral& bound,
	                                  BranchDomainLiteral& flipped) {
      if (bound.var_idx < 0 || bound.var_idx >= n ||
          !std::isfinite(bound.value)) {
        return false;
      }
      const bool integer_var =
          is_integer_type(lp.vars[static_cast<std::size_t>(bound.var_idx)]);
      if (bound.is_lb) {
        flipped = BranchDomainLiteral{
            bound.var_idx,
            integer_var ? std::ceil(bound.value - tol) - 1.0
                        : bound.value - std::max(1e-7, 10.0 * tol),
            false};
      } else {
        flipped = BranchDomainLiteral{
            bound.var_idx,
            integer_var ? std::floor(bound.value + tol) + 1.0
                        : bound.value + std::max(1e-7, 10.0 * tol),
            true};
      }
	      return true;
	    };
	    auto objective_activity_reason =
	        [&](int exclude_col) -> std::vector<BranchDomainLiteral> {
	      std::vector<BranchDomainLiteral> reason;
	      if (reasons_out == nullptr) return reason;
	      auto add_bound = [&](int col, bool is_lb, double value) {
	        if (col < 0 || col >= n || col == exclude_col ||
	            !std::isfinite(value)) {
	          return;
	        }
	        const auto& var = lp.vars[static_cast<std::size_t>(col)];
	        const double model_bound = is_lb ? var.lb : var.ub;
	        if (std::isfinite(model_bound)) {
	          const bool is_model_bound =
	              is_lb ? (value <= model_bound + tol)
	                    : (value >= model_bound - tol);
	          if (is_model_bound) return;
	        }
	        reason.push_back(BranchDomainLiteral{col, value, is_lb});
	      };
	      for (const Partition& part : partitions) {
	        for (const PartitionTerm& pt : part.terms) {
	          if (pt.col < 0 || pt.col >= n || pt.col == exclude_col) {
	            continue;
	          }
	          const bool fav_possible =
	              pt.favorable_one ? (ub[pt.col] >= 1.0 - tol)
	                               : (lb[pt.col] <= tol);
	          const bool fav_forced =
	              pt.favorable_one ? (lb[pt.col] >= 1.0 - tol)
	                               : (ub[pt.col] <= tol);
	          if (fav_forced) {
	            add_bound(pt.col, pt.favorable_one, pt.favorable_one ? 1.0 : 0.0);
	          } else if (!fav_possible) {
	            add_bound(pt.col, !pt.favorable_one,
	                      pt.favorable_one ? 0.0 : 1.0);
	          }
	        }
	      }
	      for (const Term& t : terms) {
	        const int col = t.col;
	        if (col < 0 || col >= n || col == exclude_col) continue;
	        if (col < static_cast<int>(partition_of_col.size()) &&
	            partition_of_col[static_cast<std::size_t>(col)] >= 0) {
	          continue;
	        }
	        if (t.cost > 0.0) {
	          add_bound(col, true, lb[col]);
	        } else if (t.cost < 0.0) {
	          add_bound(col, false, ub[col]);
	        }
	      }
	      canonicalize_branch_literals(reason);
	      return reason;
	    };

    std::vector<int> active_event_ids;
    active_event_ids.reserve(
        std::min<std::size_t>(implied_events.size(), 256));
    std::vector<std::uint32_t> event_stamps(implied_events.size(), 0);
    std::uint32_t event_stamp = 0;
    auto collect_active_event_ids = [&]() {
      active_event_ids.clear();
      const std::size_t expected_literal_buckets =
          static_cast<std::size_t>(2 * n);
      bool index_valid =
          implied_events_by_literal.size() == expected_literal_buckets;
      if (index_valid) {
        ++event_stamp;
        if (event_stamp == 0) {
          std::fill(event_stamps.begin(), event_stamps.end(), 0);
          event_stamp = 1;
        }
        auto append_bucket = [&](std::size_t key) {
          for (int event_id : implied_events_by_literal[key]) {
            if (event_id < 0 ||
                event_id >= static_cast<int>(implied_events.size())) {
              index_valid = false;
              return;
            }
            auto& stamp = event_stamps[static_cast<std::size_t>(event_id)];
            if (stamp == event_stamp) continue;
            stamp = event_stamp;
            active_event_ids.push_back(event_id);
          }
        };
        for (int col = 0; col < n && index_valid; ++col) {
          if (ub[col] <= tol) {
            append_bucket(static_cast<std::size_t>(2 * col));
          }
          if (index_valid && lb[col] >= 1.0 - tol) {
            append_bucket(static_cast<std::size_t>(2 * col + 1));
          }
        }
      }
      if (!index_valid) {
        active_event_ids.clear();
        active_event_ids.reserve(implied_events.size());
        for (int event_id = 0;
             event_id < static_cast<int>(implied_events.size()); ++event_id) {
          active_event_ids.push_back(event_id);
        }
      }
    };

	    for (int round = 0; round < 4; ++round) {
      Activity act = compute_activity(lb, ub, n);
      int round_tightened = 0;
      collect_active_event_ids();
      for (int event_id : active_event_ids) {
        const ImpliedContributionEvent& event =
            implied_events[static_cast<std::size_t>(event_id)];
        bool all_active = true;
        std::vector<BranchDomainLiteral> reason;
        reason.reserve(event.literals.size());
        for (const Literal& lit : event.literals) {
          if (!literal_active(lit)) {
            all_active = false;
            break;
          }
          reason.push_back(literal_reason(lit));
        }
        if (!all_active) continue;
        if (event.target_col < 0 || event.target_col >= n) continue;
        if (event.target_is_lower_bound) {
          if (event.target_bound > ub[event.target_col] + tol) {
            ++pruned;
            return false;
          }
          if (event.target_bound > lb[event.target_col] + tol) {
            lb[event.target_col] = event.target_bound;
            ++round_tightened;
            ++tightened;
            record_reason(event.target_col, /*is_lb=*/true, event.target_bound,
                          std::move(reason));
          }
        } else {
          if (event.target_bound < lb[event.target_col] - tol) {
            ++pruned;
            return false;
          }
          if (event.target_bound < ub[event.target_col] - tol) {
            ub[event.target_col] = event.target_bound;
            ++round_tightened;
            ++tightened;
            record_reason(event.target_col, /*is_lb=*/false,
                          event.target_bound, std::move(reason));
          }
        }
      }
      if (round_tightened > 0) continue;

      if (act.inf_count == 0) {
        const double preliminary_capacity = obj_cutoff - act.lower;
        const double max_objective_delta =
            std::max({act.max_delta, max_implied_event_delta,
                      max_implied_event_aggregate_delta});
        if (preliminary_capacity > max_objective_delta + 1e-9) {
          return true;
        }
      }

      if (act.inf_count > 1) return true;
      if (act.inf_count == 0 && act.lower > obj_cutoff + 1e-7) {
        ++pruned;
        return false;
      }
      double capacity = obj_cutoff - act.lower;
      if (act.inf_count == 0 && capacity > act.max_delta + 1e-9) {
        if (capacity > max_implied_event_aggregate_delta + 1e-9) {
          return true;
        }
      }

      if (act.inf_count == 1) {
        const int j = act.inf_col;
        if (j < 0 || j >= n) return true;
        const double c = lp.c[j];
        if (c > 0.0) {
          if (!std::isfinite(ub[j])) return true;
          double implied_ub = capacity / c;
          if (is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
            implied_ub = std::floor(implied_ub + tol);
          }
          if (implied_ub < lb[j] - tol) {
            ++pruned;
            return false;
          }
	          if (implied_ub < ub[j] - tol) {
	            const BranchDomainLiteral bound{j, implied_ub, false};
	            BranchDomainLiteral forbidden;
	            const BranchDomainLiteral* forbidden_ptr =
	                flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	            const double proof_delta =
	                forbidden_ptr != nullptr && std::isfinite(lb[j])
	                    ? c * (forbidden.value - lb[j])
	                    : capacity;
	            ub[j] = implied_ub;
	            ++round_tightened;
	            ++tightened;
	            record_reason(j, /*is_lb=*/false, implied_ub,
	                          objective_activity_reason(j), forbidden_ptr,
	                          proof_delta, act.lower, obj_cutoff, capacity);
	          }
	        } else if (c < 0.0) {
          if (!std::isfinite(lb[j])) return true;
          double implied_lb = capacity / c;
          if (is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
            implied_lb = std::ceil(implied_lb - tol);
          }
          if (implied_lb > ub[j] + tol) {
            ++pruned;
            return false;
          }
	          if (implied_lb > lb[j] + tol) {
	            const BranchDomainLiteral bound{j, implied_lb, true};
	            BranchDomainLiteral forbidden;
	            const BranchDomainLiteral* forbidden_ptr =
	                flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	            const double proof_delta =
	                forbidden_ptr != nullptr && std::isfinite(ub[j])
	                    ? (-c) * (ub[j] - forbidden.value)
	                    : capacity;
	            lb[j] = implied_lb;
	            ++round_tightened;
	            ++tightened;
	            record_reason(j, /*is_lb=*/true, implied_lb,
	                          objective_activity_reason(j), forbidden_ptr,
	                          proof_delta, act.lower, obj_cutoff, capacity);
	          }
	        }
      } else {
        struct AggregateEvent {
          double contribution{0.0};
          std::unordered_map<int, double> best_by_target;
          std::vector<BranchDomainLiteral> reason;
        };
        std::unordered_map<int, AggregateEvent> aggregates;
        aggregates.reserve(std::min<std::size_t>(
            implied_events.size(), static_cast<std::size_t>(256)));
        auto add_reason_literal = [](std::vector<BranchDomainLiteral>& reason,
                                     const BranchDomainLiteral& literal) {
          const auto found = std::find_if(
              reason.begin(), reason.end(), [&](const BranchDomainLiteral& r) {
                return r.var_idx == literal.var_idx &&
                       r.is_lb == literal.is_lb &&
                       std::abs(r.value - literal.value) <= 1e-9;
              });
          if (found == reason.end()) reason.push_back(literal);
        };
        for (const ImpliedContributionEvent& event : implied_events) {
          if (event.target_col < 0 || event.target_col >= n ||
              objective_partition_owns_col(event.target_col) ||
              (event.target_is_lower_bound
                   ? event.target_bound <= lb[event.target_col] + tol
                   : event.target_bound >= ub[event.target_col] - tol)) {
            continue;
          }
          const double contribution = event.target_is_lower_bound
              ? event.target_cost *
                    (event.target_bound - lb[event.target_col])
              : (-event.target_cost) *
                    (ub[event.target_col] - event.target_bound);
          if (!(contribution > 1e-9) || !std::isfinite(contribution)) {
            continue;
          }

          int missing_key = -1;
          bool impossible = false;
          std::vector<BranchDomainLiteral> active_reason;
          active_reason.reserve(event.literals.size());
          for (const Literal& lit : event.literals) {
            if (literal_active(lit)) {
              active_reason.push_back(literal_reason(lit));
            } else if (literal_possible(lit)) {
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

          AggregateEvent& aggregate = aggregates[missing_key];
          auto [best_it, inserted] =
              aggregate.best_by_target.emplace(event.target_col, contribution);
          if (inserted) {
            aggregate.contribution += contribution;
            for (const BranchDomainLiteral& r : active_reason) {
              add_reason_literal(aggregate.reason, r);
            }
          } else if (contribution > best_it->second + 1e-9) {
            aggregate.contribution += contribution - best_it->second;
            best_it->second = contribution;
            for (const BranchDomainLiteral& r : active_reason) {
              add_reason_literal(aggregate.reason, r);
            }
          }
        }
        for (const auto& [missing_key, aggregate] : aggregates) {
          if (!(aggregate.contribution > capacity + 1e-9)) continue;
          const int col = missing_key / 2;
          const bool value_one = (missing_key % 2) != 0;
          if (col < 0 || col >= n) continue;
          if (value_one) {
            if (0.0 < lb[col] - tol) {
              ++pruned;
              return false;
            }
            if (ub[col] > tol) {
              ub[col] = 0.0;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral forbidden{
                  col, value_one ? 1.0 : 0.0, value_one};
              record_reason(BranchDomainLiteral{col, 0.0, false}.var_idx,
                            /*is_lb=*/false, 0.0, aggregate.reason,
                            &forbidden, aggregate.contribution, act.lower,
                            obj_cutoff, capacity);
            }
          } else {
            if (1.0 > ub[col] + tol) {
              ++pruned;
              return false;
            }
            if (lb[col] < 1.0 - tol) {
              lb[col] = 1.0;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral forbidden{
                  col, value_one ? 1.0 : 0.0, value_one};
              record_reason(BranchDomainLiteral{col, 1.0, true}.var_idx,
                            /*is_lb=*/true, 1.0, aggregate.reason,
                            &forbidden, aggregate.contribution, act.lower,
                            obj_cutoff, capacity);
            }
          }
        }
        if (round_tightened > 0) continue;

        for (const ImpliedContributionEvent& event : implied_events) {
          if (event.target_col < 0 || event.target_col >= n ||
              objective_partition_owns_col(event.target_col) ||
              (event.target_is_lower_bound
                   ? event.target_bound <= lb[event.target_col] + tol
                   : event.target_bound >= ub[event.target_col] - tol)) {
            continue;
          }
          const double contribution = event.target_is_lower_bound
              ? event.target_cost *
                    (event.target_bound - lb[event.target_col])
              : (-event.target_cost) *
                    (ub[event.target_col] - event.target_bound);
          if (!(contribution > capacity + 1e-9)) continue;
          int missing = -1;
          bool impossible = false;
          std::vector<BranchDomainLiteral> reason;
          reason.reserve(event.literals.size());
          for (int li = 0; li < static_cast<int>(event.literals.size()); ++li) {
            const Literal& lit = event.literals[static_cast<std::size_t>(li)];
            if (literal_active(lit)) {
              reason.push_back(literal_reason(lit));
            } else if (literal_possible(lit)) {
              if (missing >= 0) {
                missing = -2;
                break;
              }
              missing = li;
            } else {
              impossible = true;
              break;
            }
          }
          if (impossible || missing < 0) continue;
          const Literal& lit = event.literals[static_cast<std::size_t>(missing)];
          if (lit.value_one) {
            if (0.0 < lb[lit.col] - tol) {
              ++pruned;
              return false;
            }
            if (ub[lit.col] > tol) {
              ub[lit.col] = 0.0;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral forbidden = literal_reason(lit);
              record_reason(lit.col, /*is_lb=*/false, 0.0, std::move(reason),
                            &forbidden, contribution, act.lower, obj_cutoff,
                            capacity);
            }
          } else {
            if (1.0 > ub[lit.col] + tol) {
              ++pruned;
              return false;
            }
            if (lb[lit.col] < 1.0 - tol) {
              lb[lit.col] = 1.0;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral forbidden = literal_reason(lit);
              record_reason(lit.col, /*is_lb=*/true, 1.0, std::move(reason),
                            &forbidden, contribution, act.lower, obj_cutoff,
                            capacity);
            }
          }
        }
        for (const Partition& part : partitions) {
          struct PossibleFav {
            const PartitionTerm* term{nullptr};
            double delta{0.0};
          };
          std::vector<PossibleFav> possible;
          possible.reserve(part.terms.size());
          bool has_fixed_favorable = false;
          int fixed_favorable_col = -1;
          double best_delta = -1.0;
          const PartitionTerm* best_term = nullptr;
          for (const PartitionTerm& pt : part.terms) {
            const bool fav_possible = pt.favorable_one
                ? (ub[pt.col] >= 1.0 - 1e-9)
                : (lb[pt.col] <= 1e-9);
            const bool fav_forced = pt.favorable_one
                ? (lb[pt.col] >= 1.0 - 1e-9)
                : (ub[pt.col] <= 1e-9);
            if (fav_forced) {
              has_fixed_favorable = true;
              fixed_favorable_col = pt.col;
            }
            if (fav_possible) {
              possible.push_back(PossibleFav{&pt, pt.replacement_delta});
              if (pt.replacement_delta > best_delta) {
                best_delta = pt.replacement_delta;
                best_term = &pt;
              }
            }
          }

          auto force_favorable = [&](const PartitionTerm& pt,
                                     double proof_delta) -> bool {
            if (pt.favorable_one) {
              if (1.0 > ub[pt.col] + tol) return false;
              if (lb[pt.col] < 1.0 - tol) {
                lb[pt.col] = 1.0;
                ++round_tightened;
                ++tightened;
                const BranchDomainLiteral bound{pt.col, 1.0, true};
                BranchDomainLiteral forbidden;
	                const BranchDomainLiteral* forbidden_ptr =
	                    flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	                record_reason(pt.col, /*is_lb=*/true, 1.0,
	                              objective_activity_reason(pt.col),
	                              forbidden_ptr, proof_delta, act.lower,
	                              obj_cutoff, capacity);
	              }
	            } else {
              if (0.0 < lb[pt.col] - tol) return false;
              if (ub[pt.col] > tol) {
                ub[pt.col] = 0.0;
                ++round_tightened;
                ++tightened;
                const BranchDomainLiteral bound{pt.col, 0.0, false};
                BranchDomainLiteral forbidden;
	                const BranchDomainLiteral* forbidden_ptr =
	                    flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	                record_reason(pt.col, /*is_lb=*/false, 0.0,
	                              objective_activity_reason(pt.col),
	                              forbidden_ptr, proof_delta, act.lower,
	                              obj_cutoff, capacity);
	              }
	            }
            return true;
          };
          auto force_nonfavorable = [&](const PartitionTerm& pt,
                                        double proof_delta) -> bool {
            if (pt.favorable_one) {
              if (0.0 < lb[pt.col] - tol) return false;
              if (ub[pt.col] > tol) {
                ub[pt.col] = 0.0;
                ++round_tightened;
                ++tightened;
                const BranchDomainLiteral bound{pt.col, 0.0, false};
                BranchDomainLiteral forbidden;
	                const BranchDomainLiteral* forbidden_ptr =
	                    flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	                record_reason(pt.col, /*is_lb=*/false, 0.0,
	                              objective_activity_reason(pt.col),
	                              forbidden_ptr, proof_delta, act.lower,
	                              obj_cutoff, capacity);
	              }
	            } else {
              if (1.0 > ub[pt.col] + tol) return false;
              if (lb[pt.col] < 1.0 - tol) {
                lb[pt.col] = 1.0;
                ++round_tightened;
                ++tightened;
                const BranchDomainLiteral bound{pt.col, 1.0, true};
                BranchDomainLiteral forbidden;
	                const BranchDomainLiteral* forbidden_ptr =
	                    flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	                record_reason(pt.col, /*is_lb=*/true, 1.0,
	                              objective_activity_reason(pt.col),
	                              forbidden_ptr, proof_delta, act.lower,
	                              obj_cutoff, capacity);
	              }
	            }
            return true;
          };

          if (has_fixed_favorable) {
            for (const PartitionTerm& pt : part.terms) {
              if (pt.col == fixed_favorable_col) continue;
                if (!force_nonfavorable(pt, capacity + 1e-9)) {
                  ++pruned;
                  return false;
                }
            }
            continue;
          }
          if (best_term == nullptr || best_delta <= 0.0) continue;
          double second_delta = -1.0;
          for (const PossibleFav& fav : possible) {
            if (fav.term == best_term) continue;
            second_delta = std::max(second_delta, fav.delta);
          }
          const double best_replacement_loss =
              best_delta - std::max(0.0, second_delta);
          if (best_replacement_loss > capacity + 1e-9) {
            if (!force_favorable(*best_term, best_replacement_loss)) {
              ++pruned;
              return false;
            }
            continue;
          }
          for (const PossibleFav& fav : possible) {
            if (fav.term == best_term) continue;
            if (best_delta - fav.delta > capacity + 1e-9) {
              if (!force_nonfavorable(*fav.term, best_delta - fav.delta)) {
                ++pruned;
                return false;
              }
            }
          }
        }
        for (const Term& t : terms) {
          const int j = t.col;
          if (j < 0 || j >= n) continue;
          if (j < static_cast<int>(partition_of_col.size()) &&
              partition_of_col[static_cast<std::size_t>(j)] >= 0) {
            continue;
          }
          const double c = t.cost;
          if (std::abs(c) <= 1e-12) continue;
          const double other_activity =
              act.lower - act.min_contribution[static_cast<std::size_t>(j)];
          double implied = (obj_cutoff - other_activity) / c;
          if (!std::isfinite(implied)) continue;
          if (c > 0.0) {
            if (t.integer) implied = std::floor(implied + tol);
            if (implied < lb[j] - tol) {
              ++pruned;
              return false;
            }
            if (implied < ub[j] - tol) {
              ub[j] = implied;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral bound{j, implied, false};
              BranchDomainLiteral forbidden;
              const BranchDomainLiteral* forbidden_ptr =
                  flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	              const double proof_delta =
	                  forbidden_ptr != nullptr && std::isfinite(lb[j])
	                      ? c * (forbidden.value - lb[j])
	                      : capacity;
	              record_reason(j, /*is_lb=*/false, implied,
	                            objective_activity_reason(j), forbidden_ptr,
	                            proof_delta, act.lower, obj_cutoff, capacity);
	            }
	          } else {
            if (t.integer) implied = std::ceil(implied - tol);
            if (implied > ub[j] + tol) {
              ++pruned;
              return false;
            }
            if (implied > lb[j] + tol) {
              lb[j] = implied;
              ++round_tightened;
              ++tightened;
              const BranchDomainLiteral bound{j, implied, true};
              BranchDomainLiteral forbidden;
              const BranchDomainLiteral* forbidden_ptr =
                  flip_bound_literal(bound, forbidden) ? &forbidden : nullptr;
	              const double proof_delta =
	                  forbidden_ptr != nullptr && std::isfinite(ub[j])
	                      ? (-c) * (ub[j] - forbidden.value)
	                      : capacity;
	              record_reason(j, /*is_lb=*/true, implied,
	                            objective_activity_reason(j), forbidden_ptr,
	                            proof_delta, act.lower, obj_cutoff, capacity);
	            }
	          }
        }
      }
      if (round_tightened == 0) break;
    }
    return true;
  }
};

struct PublishedObjectiveEventStats {
  std::uint64_t single_literal_events{0};
  std::uint64_t implications_added{0};
  std::uint64_t clique_edges_added{0};
};

inline PublishedObjectiveEventStats publish_objective_implied_event_source(
    const LPModel& lp,
    const ObjectivePropagationState& objective_propagation,
    BinaryImplicationGraph& implication_graph,
    CliqueTable& clique_table,
    double int_tol) {
  PublishedObjectiveEventStats stats;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0 || objective_propagation.implied_events.empty()) {
    return stats;
  }

  auto binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.type == VarType::Binary) return true;
    return std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -std::max(1e-9, int_tol) &&
           var.ub <= 1.0 + std::max(1e-9, int_tol);
  };

  std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
      literal_edges;
  literal_edges.reserve(1024);

  for (const auto& event : objective_propagation.implied_events) {
    if (event.literals.size() != 1 || event.target_col < 0 ||
        event.target_col >= n || !std::isfinite(event.target_bound)) {
      continue;
    }
    const auto& lit = event.literals.front();
    if (!binary_like(lit.col) || lit.col == event.target_col) continue;

    const auto& target_var = lp.vars[static_cast<std::size_t>(event.target_col)];
    bool improving = false;
    if (event.target_is_lower_bound) {
      improving = event.target_bound > target_var.lb + 1e-9;
    } else {
      improving = event.target_bound < target_var.ub - 1e-9;
    }
    if (!improving) continue;

    ++stats.single_literal_events;
    if (implication_graph.add_implication(
            lit.col, lit.value_one, event.target_col,
            event.target_is_lower_bound, event.target_bound)) {
      ++stats.implications_added;
    }

    if (!binary_like(event.target_col)) continue;
    if (event.target_is_lower_bound &&
        event.target_bound >= 1.0 - std::max(1e-9, int_tol)) {
      literal_edges.emplace_back(
          CliqueTable::Literal{lit.col, lit.value_one},
          CliqueTable::Literal{event.target_col, false});
    } else if (!event.target_is_lower_bound &&
               event.target_bound <= std::max(1e-9, int_tol)) {
      literal_edges.emplace_back(
          CliqueTable::Literal{lit.col, lit.value_one},
          CliqueTable::Literal{event.target_col, true});
    }
  }

  stats.clique_edges_added = clique_table.add_literal_edges(lp, literal_edges);
  return stats;
}

}  // namespace mipsolvers::engine::detail

/// @file bc_cuts.cpp
/// @brief Cut generation routines for the B&C solver.
///
/// Implements GMI (Gomory Mixed-Integer) cuts, single-row complemented MIR cuts,
/// cover cuts, implied bound cuts,
/// and the unified add_cuts dispatch. Includes scoring helpers for cut selection.

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fmt/format.h>

#include "../extern/pdqsort/pdqsort.h"
#include "util/HighsCDouble.h"
#include "mip/HighsGFkSolve.h"
#include "util/HighsRandom.h"
#include "util/HighsIntegers.h"

#include "bc_cuts_common.hpp"

namespace mipsolvers::engine::detail {

namespace {

int add_mir_like_cuts(LPModel& lp,
                      const Eigen::VectorXd& x,
                      CutType type,
                      int max_cuts,
                      SeparatorRowWorkspace& row_workspace,
                      SeparatorStorageStats* storage_stats) {
  int added = 0;
  const int n = static_cast<int>(lp.vars.size());

  std::vector<Eigen::SparseVector<double>> cut_rows;
  std::vector<double> cut_rhs;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  for (int r = 0; r < row_workspace.inequality_rows() && added < max_cuts;
       ++r) {
    const double frac_rhs = frac_part(lp.b[r]);
    if (frac_rhs <= 1e-8) {
      continue;
    }

    std::vector<std::pair<int, double>> terms;
    bool has_int = false;
    double lhs = 0.0;
    row_workspace.visit_inequality_row(r, [&](int j, double a) {
      if (std::abs(a) <= 1e-12 || !is_integer_type(lp.vars[j])) {
        return;
      }
      has_int = true;
      const double fj = frac_part(a);
      if (fj <= 1e-12) {
        return;
      }
      double coeff = 0.0;
      if (type == CutType::IntRounding) {
        coeff = -fj;
      } else {
        const double lb = lp.vars[j].lb;
        const double ub = lp.vars[j].ub;
        if (std::isfinite(lb) && std::isfinite(ub) && ub > lb + 1e-9) {
          coeff = -fj / (ub - lb);
        } else {
          coeff = -fj;
        }
      }
      terms.emplace_back(j, coeff);
      lhs += coeff * x[j];
    });

    if (!has_int || terms.empty()) {
      continue;
    }
    const double rhs = -frac_rhs;
    const double viol = lhs - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    Eigen::SparseVector<double> cut(n);
    cut.reserve(static_cast<int>(terms.size()));
    for (const auto& [col, value] : terms) cut.insertBack(col) = value;
    cut_rows.push_back(std::move(cut));
    storage_tracker.record(cut_rows.back(), cut_rows.size());
    cut_rhs.push_back(rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_sparse_rows_to_lp(lp, cut_rows, cut_rhs, storage_stats);
  }
  return added;
}

int add_cover_cuts(LPModel& lp,
                   const Eigen::VectorXd& x,
                   int max_cuts,
                   SeparatorRowWorkspace& row_workspace,
                   SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0) {
    return 0;
  }

  int added = 0;
  const int n = static_cast<int>(lp.vars.size());

  std::vector<Eigen::SparseVector<double>> cut_rows;
  std::vector<double> cut_rhs;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);

  for (int r = 0; r < row_workspace.inequality_rows() && added < max_cuts;
       ++r) {
    if (lp.b[r] <= 0.0) {
      continue;
    }

    std::vector<int> cand;
    cand.reserve(static_cast<size_t>(n));
    std::vector<double> cand_coeff;  // parallel to cand, cached coefficients
    cand_coeff.reserve(static_cast<size_t>(n));
    bool valid_row = true;
    row_workspace.visit_inequality_row(r, [&](int j, double a) {
      if (std::abs(a) <= 1e-12) {
        return;
      }
      if (j < 0 || j >= n) { valid_row = false; return; }
      if (a < 0.0 || lp.vars[j].type != VarType::Binary) {
        valid_row = false;
        return;
      }
      cand.push_back(j);
      cand_coeff.push_back(a);
    });

    if (!valid_row || cand.size() < 2) {
      continue;
    }

    // Sort cand by coefficient descending using cached values (avoids repeated sparse lookups).
    std::vector<int> order(cand.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return cand_coeff[a] > cand_coeff[b];
    });
    std::vector<int> sorted_cand(cand.size());
    std::vector<double> sorted_coeff(cand.size());
    for (int i = 0; i < static_cast<int>(order.size()); ++i) {
      sorted_cand[i] = cand[order[i]];
      sorted_coeff[i] = cand_coeff[order[i]];
    }
    cand = sorted_cand;
    cand_coeff = sorted_coeff;

    std::vector<int> cover;
    cover.reserve(cand.size());
    double sum_w = 0.0;
    for (int i = 0; i < static_cast<int>(cand.size()); ++i) {
      cover.push_back(cand[i]);
      sum_w += cand_coeff[i];
      if (sum_w > lp.b[r] + 1e-9) {
        break;
      }
    }

    if (sum_w <= lp.b[r] + 1e-9 || cover.size() < 2) {
      continue;
    }

    const int cover_size = static_cast<int>(cover.size());
    double rhs = static_cast<double>(cover_size) - 1.0;

    double lhs = 0.0;
    for (int j : cover) lhs += x[j];
    const double viol = lhs - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    cut_rows.push_back(sparse_indicator_cut(n, std::move(cover)));
    storage_tracker.record(cut_rows.back(), cut_rows.size());
    cut_rhs.push_back(rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_sparse_rows_to_lp(lp, cut_rows, cut_rhs, storage_stats);
  }
  return added;
}

struct SparseMIRCandidate {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
  double violation{0.0};
  double norm{0.0};
};

bool build_sparse_complemented_mir(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& x,
    const std::vector<std::pair<int, double>>& row_terms,
    double row_rhs,
    SparseMIRCandidate& candidate) {
  const int n = static_cast<int>(vars.size());
  if (x.size() < n || !std::isfinite(row_rhs)) return false;

  bool has_integer = false;
  double complemented_rhs = row_rhs;
  std::vector<unsigned char> complemented(row_terms.size(), 0);
  for (std::size_t k = 0; k < row_terms.size(); ++k) {
    const int col = row_terms[k].first;
    const double value = row_terms[k].second;
    if (col < 0 || col >= n || !std::isfinite(value) ||
        std::abs(value) <= 1e-12) {
      continue;
    }
    if (!is_integer_type(vars[static_cast<std::size_t>(col)])) continue;
    has_integer = true;
    const double lb = vars[static_cast<std::size_t>(col)].lb;
    const double ub = vars[static_cast<std::size_t>(col)].ub;
    if (std::isfinite(lb) && std::isfinite(ub) &&
        x[col] > 0.5 * (lb + ub)) {
      complemented[k] = 1;
      complemented_rhs -= value * ub;
    }
  }
  if (!has_integer) return false;

  const double f0 = frac_part(complemented_rhs);
  if (f0 <= 1e-8 || f0 >= 1.0 - 1e-8) return false;

  std::vector<std::pair<int, double>> cut_terms;
  cut_terms.reserve(row_terms.size());
  double rhs = std::floor(complemented_rhs);
  double lhs = 0.0;
  double norm_sq = 0.0;
  for (std::size_t k = 0; k < row_terms.size(); ++k) {
    const int col = row_terms[k].first;
    if (col < 0 || col >= n) continue;
    double value = row_terms[k].second;
    if (!std::isfinite(value) || std::abs(value) <= 1e-12) continue;
    if (complemented[k] != 0) value = -value;

    double mir_value = 0.0;
    if (is_integer_type(vars[static_cast<std::size_t>(col)])) {
      const double fj = frac_part(value);
      mir_value = fj <= f0 + 1e-10
                      ? std::floor(value)
                      : std::floor(value) + (fj - f0) / (1.0 - f0);
    } else if (value < 0.0) {
      mir_value = value / (1.0 - f0);
    }
    if (std::abs(mir_value) <= 1e-12) continue;

    double original_value = mir_value;
    if (complemented[k] != 0) {
      original_value = -mir_value;
      rhs -= mir_value * vars[static_cast<std::size_t>(col)].ub;
    }
    cut_terms.emplace_back(col, original_value);
    lhs += original_value * x[col];
    norm_sq += original_value * original_value;
  }
  if (cut_terms.empty() || norm_sq <= 1e-24) return false;

  const double violation = lhs - rhs;
  if (violation <= 1e-7 || !std::isfinite(violation) ||
      !std::isfinite(rhs)) {
    return false;
  }

  Eigen::SparseVector<double> cut(n);
  cut.reserve(static_cast<int>(cut_terms.size()));
  for (const auto& [col, value] : cut_terms) cut.insertBack(col) = value;
  candidate.coeff = std::move(cut);
  candidate.rhs = rhs;
  candidate.violation = violation;
  candidate.norm = std::sqrt(norm_sq);
  return true;
}

int add_basis_mir_cuts(LPModel& lp,
                       const Eigen::VectorXd& x,
                       [[maybe_unused]] const SimplexResult& simplex,
                       [[maybe_unused]] const BCOptions& opt,
                       int max_cuts,
                       [[maybe_unused]] const std::shared_ptr<BasisOps>& sbasis,
                       SeparatorRowWorkspace& row_workspace,
                       SeparatorStorageStats* storage_stats) {
  // True single-row complemented MIR cuts.
  // For each inequality row: sum_j a_j x_j <= b
  //   Separate integer vars I and continuous vars C.
  //   Complement integer vars near their upper bound: x'_j = ub_j - x_j
  //   Apply MIR rounding: let f0 = frac(b'), for integer j with f_j > f0:
  //     coeff_j = floor(a_j) + (frac(a_j) - f0)/(1-f0)
  //   For continuous vars with a_j > 0: coeff_j = a_j / (1-f0)
  //   For continuous vars with a_j < 0: coeff_j = 0 (dropped)
  if (max_cuts <= 0) {
    return 0;
  }

  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = row_workspace.inequality_rows();
  const int m_eq = row_workspace.equality_rows();

  std::vector<SparseMIRCandidate> candidates;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  candidates.reserve(static_cast<size_t>(std::min(m_ineq + m_eq, max_cuts * 4)));

  auto try_mir_row = [&](auto&& visit_nonzeros, double b_val, int /*row_idx*/) {
    std::vector<std::pair<int, double>> row_terms;
    visit_nonzeros([&](int j, double val) {
      if (j >= 0 && j < n && std::abs(val) > 1e-12) {
        row_terms.emplace_back(j, val);
      }
    });
    SparseMIRCandidate candidate;
    if (!build_sparse_complemented_mir(lp.vars, x, row_terms, b_val,
                                       candidate)) {
      return;
    }
    const double efficacy = candidate.violation / candidate.norm;
    if (efficacy < 5e-4) return;

    // Scale the density cap down for large problems.
    const int nnz = static_cast<int>(candidate.coeff.nonZeros());
    const double mir_density_cap = (n > 20000) ? 0.10 : (n > 10000) ? 0.30 : 0.60;
    if (n > 0 && static_cast<double>(nnz) / n > mir_density_cap) return;

    candidates.push_back(std::move(candidate));
    storage_tracker.record(candidates.back().coeff, candidates.size());
  };

  // Process inequality rows
  for (int r = 0; r < m_ineq; ++r) {
    try_mir_row([&](auto&& f) {
      row_workspace.visit_inequality_row(r, f);
    }, lp.b[r], r);
  }
  // Process equality rows as two inequalities (relaxed as <=)
  for (int r = 0; r < m_eq; ++r) {
    try_mir_row([&](auto&& f) {
      row_workspace.visit_equality_row(r, f);
    }, lp.beq[r], m_ineq + r);
    // Also try negated form
    try_mir_row([&](auto&& f) {
      row_workspace.visit_equality_row(
          r, [&](int col, double value) { f(col, -value); });
    }, -lp.beq[r], m_ineq + m_eq + r);
  }

  if (candidates.empty()) return 0;

  // Sort by violation (efficacy) and select diverse cuts
  std::sort(candidates.begin(), candidates.end(),
            [](const SparseMIRCandidate& a, const SparseMIRCandidate& b) {
              return a.violation > b.violation;
            });

  int added = 0;
  std::vector<Eigen::SparseVector<double>> cut_rows;
  std::vector<double> cut_rhs;
  std::vector<double> cut_norms;

  for (const auto& cand : candidates) {
    if (added >= max_cuts) break;
    // Parallelism filter against already selected
    // Use the configured parallelism threshold to preserve orthogonality.
    bool parallel = false;
    for (size_t si = 0; si < cut_rows.size(); ++si) {
      double cos_val =
          std::abs(sparse_sparse_dot(cand.coeff, cut_rows[si])) /
          (cand.norm * cut_norms[si]);
      if (cos_val > 0.90) { parallel = true; break; }
    }
    if (parallel) continue;

    cut_norms.push_back(cand.norm);
    cut_rows.push_back(cand.coeff);
    cut_rhs.push_back(cand.rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_sparse_rows_to_lp(lp, cut_rows, cut_rhs, storage_stats);
  }
  return added;
}

int add_projected_capacity_cover_cuts(LPModel& lp,
                                      const Eigen::VectorXd& x,
                                      int max_cuts,
                                      SeparatorRowWorkspace& row_workspace,
                                      SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0 || row_workspace.equality_rows() == 0 ||
      row_workspace.inequality_rows() == 0) {
    return 0;
  }
  const int n = static_cast<int>(lp.vars.size());
  if (x.size() < n) return 0;

  struct Link {
    int bin{-1};
    double ub{0.0};
  };
  std::vector<Link> vub(static_cast<std::size_t>(n));

  for (int r = 0; r < row_workspace.inequality_rows(); ++r) {
    int cont = -1;
    int bin = -1;
    double a_cont = 0.0;
    double a_bin = 0.0;
    int nnz = 0;
    row_workspace.visit_inequality_row(r, [&](int j, double value) {
      if (std::abs(value) <= 1e-12) return;
      ++nnz;
      if (j < 0 || j >= n) return;
      if (!is_integer_type(lp.vars[j]) && value > 0.0) {
        cont = j;
        a_cont = value;
      } else if (lp.vars[j].type == VarType::Binary && value < 0.0) {
        bin = j;
        a_bin = value;
      }
    });
    if (nnz == 2 && cont >= 0 && bin >= 0 && a_cont > 1e-12 &&
        a_bin < -1e-12 && std::abs(lp.b[r]) <= 1e-8) {
      const double ub = -a_bin / a_cont;
      if (std::isfinite(ub) && ub > 1e-9) {
        vub[static_cast<std::size_t>(cont)] = Link{bin, ub};
      }
    }
  }

  struct Item {
    int bin;
    double cap;
    double value;
  };

  std::vector<Eigen::SparseVector<double>> rows;
  std::vector<double> rhs;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  rows.reserve(static_cast<std::size_t>(max_cuts));
  rhs.reserve(static_cast<std::size_t>(max_cuts));

  for (int r = 0; r < row_workspace.equality_rows() &&
                  static_cast<int>(rows.size()) < max_cuts;
       ++r) {
    const double demand = lp.beq[r];
    if (!(demand > 1e-9) || !std::isfinite(demand)) continue;

    bool usable = true;
    std::vector<Item> items;
    double total_cap = 0.0;
    row_workspace.visit_equality_row(r, [&](int j, double a) {
      if (!usable || j < 0 || j >= n || std::abs(a) <= 1e-12) return;
      if (a <= 0.0 || is_integer_type(lp.vars[j])) {
        usable = false;
        return;
      }
      const Link link = vub[static_cast<std::size_t>(j)];
      if (link.bin < 0) {
        usable = false;
        return;
      }
      const double cap = a * link.ub;
      if (!(cap > 1e-9) || !std::isfinite(cap)) {
        usable = false;
        return;
      }
      items.push_back(Item{link.bin, cap, x[link.bin]});
      total_cap += cap;
    });
    if (!usable || items.size() < 2 || total_cap <= demand + 1e-8) continue;

    {
      std::vector<double> caps;
      caps.reserve(items.size());
      double activity = 0.0;
      for (const auto& item : items) {
        caps.push_back(item.cap);
        activity += item.value;
      }
      std::sort(caps.begin(), caps.end(), std::greater<double>());
      double cap_prefix = 0.0;
      int min_count = 0;
      while (min_count < static_cast<int>(caps.size()) &&
             cap_prefix + 1e-8 < demand) {
        cap_prefix += caps[static_cast<std::size_t>(min_count)];
        ++min_count;
      }
      if (cap_prefix + 1e-8 >= demand && min_count > 0 &&
          activity < static_cast<double>(min_count) - 1e-7 &&
          static_cast<int>(rows.size()) < max_cuts) {
        std::vector<int> indices;
        indices.reserve(items.size());
        for (const auto& item : items) indices.push_back(item.bin);
        rows.push_back(sparse_indicator_cut(n, std::move(indices), -1.0));
        storage_tracker.record(rows.back(), rows.size());
        rhs.push_back(-static_cast<double>(min_count));
      }
    }
    if (static_cast<int>(rows.size()) >= max_cuts) break;

    const double cover_threshold = total_cap - demand;
    std::sort(items.begin(), items.end(), [](const Item& lhs, const Item& rhs) {
      const double rl = lhs.value / std::max(1e-9, lhs.cap);
      const double rr = rhs.value / std::max(1e-9, rhs.cap);
      if (std::abs(rl - rr) > 1e-12) return rl < rr;
      return lhs.value < rhs.value;
    });

    const int seed_limit = std::min<int>(static_cast<int>(items.size()), 8);
    for (int seed = 0; seed < seed_limit && static_cast<int>(rows.size()) < max_cuts; ++seed) {
      double cover_cap = 0.0;
      double cover_activity = 0.0;
      std::vector<int> cover_bins;
      auto take = [&](const Item& item) {
        if (std::find(cover_bins.begin(), cover_bins.end(), item.bin) != cover_bins.end()) return;
        cover_cap += item.cap;
        cover_activity += item.value;
        cover_bins.push_back(item.bin);
      };
      take(items[static_cast<std::size_t>(seed)]);
      for (int pos = 0; pos < static_cast<int>(items.size()) &&
                        cover_cap <= cover_threshold + 1e-8; ++pos) {
        take(items[static_cast<std::size_t>(pos)]);
      }
      if (cover_cap <= cover_threshold + 1e-8) continue;
      if (cover_activity >= 1.0 - 1e-7) continue;

      std::sort(cover_bins.begin(), cover_bins.end());
      cover_bins.erase(std::unique(cover_bins.begin(), cover_bins.end()), cover_bins.end());
      if (cover_bins.empty()) continue;

      rows.push_back(sparse_indicator_cut(n, std::move(cover_bins), -1.0));
      storage_tracker.record(rows.back(), rows.size());
      rhs.push_back(-1.0);
    }
  }

  if (!rows.empty()) {
    add_sparse_rows_to_lp(lp, rows, rhs, storage_stats);
  }
  return static_cast<int>(rows.size());
}

/// Basis-free MIR cuts that work without simplex basis (e.g., after IPM root).
/// Applies single-row complemented MIR directly to LP constraints.
int add_row_mir_cuts(LPModel& lp,
                     const Eigen::VectorXd& x,
                     int max_cuts,
                     SeparatorRowWorkspace& row_workspace,
                     SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = row_workspace.inequality_rows();
  const int m_eq = row_workspace.equality_rows();

  std::vector<SparseMIRCandidate> candidates;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);

  auto try_row = [&](auto&& visit_nonzeros, double b_val) {
    std::vector<std::pair<int, double>> row_terms;
    visit_nonzeros([&](int j, double val) {
      if (j >= 0 && j < n && std::abs(val) > 1e-12) {
        row_terms.emplace_back(j, val);
      }
    });
    SparseMIRCandidate candidate;
    if (build_sparse_complemented_mir(lp.vars, x, row_terms, b_val,
                                      candidate)) {
      candidates.push_back(std::move(candidate));
      storage_tracker.record(candidates.back().coeff, candidates.size());
    }
  };

  for (int r = 0; r < m_ineq; ++r) {
    try_row([&](auto&& f) {
      row_workspace.visit_inequality_row(r, f);
    }, lp.b[r]);
  }
  for (int r = 0; r < m_eq; ++r) {
    try_row([&](auto&& f) {
      row_workspace.visit_equality_row(r, f);
    }, lp.beq[r]);
    try_row([&](auto&& f) {
      row_workspace.visit_equality_row(
          r, [&](int col, double value) { f(col, -value); });
    }, -lp.beq[r]);
  }

  if (candidates.empty()) return 0;
  std::sort(candidates.begin(), candidates.end(),
            [](const SparseMIRCandidate& a,
               const SparseMIRCandidate& b) {
              return a.violation > b.violation;
            });

  int added = 0;
  std::vector<Eigen::SparseVector<double>> rows;
  std::vector<double> rhs_vec;
  for (const auto& c : candidates) {
    if (added >= max_cuts) break;
    rows.push_back(c.coeff);
    rhs_vec.push_back(c.rhs);
    ++added;
  }
  if (!rows.empty()) {
    add_sparse_rows_to_lp(lp, rows, rhs_vec, storage_stats);
  }
  return added;
}


}  // anonymous namespace

int add_basis_gomory_cuts(LPModel& lp,
                          const Eigen::VectorXd& x,
                          const SimplexResult& simplex,
                          const BCOptions& opt,
                          int max_cuts,
                          const std::shared_ptr<BasisOps>& sbasis,
                          SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0 || !simplex.exact_optimal) {
    return 0;
  }
  if (!basis_tableau_cuts_admissible(simplex, opt, sbasis)) {
    return 0;
  }

  const int m = static_cast<int>(simplex.basis.indices.size());
  if (!sbasis && simplex.basis_inverse.rows() != m) {
    return 0;
  }

  struct CandidateCut {
    Eigen::SparseVector<double> coeff;
    double rhs{0.0};
    double score{0.0};
    double efficacy{0.0};
    double activity{0.0};
    double norm{0.0};
    int nnz{0};
  };

  const int n = static_cast<int>(x.size());
  const double density_cap = std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz = std::max(2, static_cast<int>(std::ceil(density_cap * static_cast<double>(n))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);
  const double min_activity = std::min(1.0, std::max(0.0, opt.gmi_min_activity));
  const double min_binary_support = std::min(1.0, std::max(0.0, opt.gmi_min_binary_support));
  const double activity_weight = std::max(0.0, opt.gmi_activity_weight);

  std::vector<CandidateCut> candidates;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  candidates.reserve(static_cast<size_t>(max_cuts * 3));

  // Pre-filter: identify fractional integer basis rows and sort by
  // fractionality (most fractional first = closest to 0.5).  Only
  // evaluate the top candidates to limit expensive BTRAN computations.
  // For large m (e.g. 6884), this reduces BTRAN count from ~200 to ~60,
  // saving ~150ms in cut generation.
  struct FracRow { int row; double frac; };
  std::vector<FracRow> frac_rows;
  frac_rows.reserve(static_cast<size_t>(m));
  for (int row = 0; row < m; ++row) {
    const int basic_col = simplex.basis.indices[static_cast<size_t>(row)];
    if (!transformed_original_var_is_integer(simplex, basic_col)) continue;
    const double f = frac_part(
        original_space_value(simplex, basic_col, simplex.x_basic[row]));
    if (f <= 1e-8 || f >= 1.0 - 1e-8) continue;
    frac_rows.push_back({row, f});
  }
  // Sort by distance from 0.5 ascending (most fractional first).
  std::sort(frac_rows.begin(), frac_rows.end(),
            [](const FracRow& a, const FracRow& b) {
              return std::abs(a.frac - 0.5) < std::abs(b.frac - 0.5);
            });
  // Evaluate at most 3× budget candidates (enough for good diversity).
  const int eval_limit = std::min(static_cast<int>(frac_rows.size()),
                                  std::max(max_cuts * 3, 30));
  if (static_cast<int>(frac_rows.size()) > eval_limit)
    frac_rows.resize(static_cast<size_t>(eval_limit));

  // Build is_basic once here and pass it down to avoid one O(n_std) allocation
  // + fill per fractional row (can be hundreds per cut round).
  const int n_std_gmi = static_cast<int>(simplex.form.A.cols());
  std::vector<char> is_basic_cache(static_cast<size_t>(n_std_gmi), 0);
  for (int idx : simplex.basis.indices) {
    if (idx >= 0 && idx < n_std_gmi)
      is_basic_cache[static_cast<size_t>(idx)] = 1;
  }
  const std::vector<char>* is_basic_ptr = &is_basic_cache;

  for (const auto& fr : frac_rows) {
    Eigen::VectorXd cut;
    double rhs = 0.0;
    if (!build_bounded_form_gmi_cut(simplex, fr.row, cut, rhs, sbasis, is_basic_ptr)) {
      continue;
    }
    record_dense_workspace(storage_stats,
                           static_cast<std::size_t>(cut.size()));
    const double viol = cut.dot(x) - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    const int nnz = count_nonzeros(cut);
    if (nnz > max_nnz) {
      continue;
    }

    const double norm = std::max(1e-12, cut.norm());
    const double efficacy = viol / norm;
    if (efficacy < min_efficacy) {
      continue;
    }

    const double activity = gmi_binary_activity_score(simplex, x, cut);
    if (activity < min_activity) {
      continue;
    }

    const double binary_support = gmi_binary_support_ratio(simplex, cut);
    if (binary_support < min_binary_support) {
      continue;
    }

    const double score = efficacy * (1.0 + activity_weight * activity);
    candidates.push_back(CandidateCut{dense_to_sparse_cut(cut), rhs, score,
                                      efficacy, activity, norm, nnz});
    storage_tracker.record(candidates.back().coeff, candidates.size());

  }

  std::sort(candidates.begin(), candidates.end(), [](const CandidateCut& a, const CandidateCut& b) {
    if (std::abs(a.score - b.score) > 1e-12) {
      return a.score > b.score;
    }
    return a.rhs < b.rhs;
  });

  int added = 0;
  std::vector<CandidateCut> selected;
  selected.reserve(static_cast<size_t>(max_cuts));

  for (const auto& cand : candidates) {
    if (added >= max_cuts) {
      break;
    }

    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm, keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (near_parallel) {
      continue;
    }

    selected.push_back(cand);
    ++added;
  }

  if (!selected.empty()) {
    std::vector<Eigen::SparseVector<double>> cut_rows;
    std::vector<double> cut_rhs;
    cut_rows.reserve(selected.size());
    cut_rhs.reserve(selected.size());
    for (const auto& s : selected) {
      cut_rows.push_back(s.coeff);
      cut_rhs.push_back(s.rhs);
    }
    add_sparse_rows_to_lp(lp, cut_rows, cut_rhs, storage_stats);
  }

  return added;
}


int add_clique_cuts(LPModel& lp,
                    const Eigen::VectorXd& x,
                    int max_cuts,
                    SeparatorRowWorkspace& row_workspace,
                    SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m = row_workspace.inequality_rows();

  // Skip for very large problems: conflict graph construction is O(m * k^2)
  // and rebuilds each cut round. Raised from 8000→15000 to enable clique
  // cuts on SCUC-scale problems (m ≈ 9000-12000).
  if (m > 15000) return 0;

  // Build conflict adjacency: conflict[i] = set of j where some row forces
  // x_i + x_j <= 1 (i.e. a_i + a_j > b).
  // Limit to rows with at most 200 binary vars to avoid O(n^2) blowup.
  // Build conflict adjacency: conflict[i] = set of j where some row forces
  // x_i + x_j <= 1 (i.e. a_i + a_j > b).
  // Limit to rows with at most 200 binary vars to avoid O(n^2) blowup.
  std::vector<std::vector<int>> conflicts(n);

  for (int r = 0; r < m; ++r) {
    if (lp.b[r] <= 0.0) continue;

    std::vector<std::pair<int,double>> bin_vars;
    bool valid = true;
    row_workspace.visit_inequality_row(r, [&](int j, double a) {
      if (!valid || std::abs(a) <= 1e-12) return;
      if (j < 0 || j >= n) { valid = false; return; }
      if (a < 0.0 || lp.vars[j].type != VarType::Binary) {
        valid = false;
        return;
      }
      bin_vars.push_back({j, a});
    });
    if (!valid || bin_vars.size() < 2 || bin_vars.size() > 200) continue;

    const double b_r = lp.b[r];
    for (size_t i = 0; i < bin_vars.size(); ++i) {
      for (size_t k = i + 1; k < bin_vars.size(); ++k) {
        if (bin_vars[i].second + bin_vars[k].second > b_r + 1e-9) {
          conflicts[bin_vars[i].first].push_back(bin_vars[k].first);
          conflicts[bin_vars[k].first].push_back(bin_vars[i].first);
        }
      }
    }
  }

  // Sort and deduplicate conflict lists
  for (int j = 0; j < n; ++j) {
    auto& v = conflicts[j];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }

  // Find cliques violated by current LP solution, ordered by fractional sum
  struct CliqueCandidate {
    std::vector<int> clique;
    double violation;
  };
  std::vector<CliqueCandidate> candidates;

  // Seed cliques from high-fractional binary variables
  std::vector<int> seeds;
  for (int j = 0; j < n; ++j) {
    if (lp.vars[j].type != VarType::Binary) continue;
    if (conflicts[j].empty()) continue;
    if (x[j] > 0.1 && x[j] < 0.9) seeds.push_back(j);
  }
  // Sort seeds by fractional value descending (most violated first)
  std::sort(seeds.begin(), seeds.end(), [&](int a, int b) {
    return x[a] > x[b];
  });
  if (seeds.size() > 200) seeds.resize(200);

  std::vector<bool> used_as_seed(n, false);
  for (int seed : seeds) {
    if (used_as_seed[seed]) continue;
    used_as_seed[seed] = true;

    // Greedy clique extension: start with seed, add the neighbor with
    // highest x-value that is in conflict with ALL current clique members.
    std::vector<int> clique = {seed};
    std::vector<bool> in_clique(n, false);
    in_clique[seed] = true;

    // Candidate pool: neighbors of seed
    std::vector<int> pool = conflicts[seed];

    while (!pool.empty()) {
      // Pick candidate with highest LP value
      int best = -1;
      double best_val = -1.0;
      for (int c : pool) {
        if (!in_clique[c] && x[c] > best_val) {
          best_val = x[c];
          best = c;
        }
      }
      if (best < 0 || best_val < 1e-6) break;

      // Check: best must conflict with ALL clique members
      bool all_conflict = true;
      for (int cm : clique) {
        if (!std::binary_search(conflicts[best].begin(), conflicts[best].end(), cm)) {
          all_conflict = false;
          break;
        }
      }
      if (!all_conflict) {
        // O(1) removal: swap rejected candidate to the end and shrink.
        const auto it = std::find(pool.begin(), pool.end(), best);
        if (it != pool.end()) {
          *it = pool.back();
          pool.pop_back();
        }
        continue;
      }

      clique.push_back(best);
      in_clique[best] = true;

      // Intersect pool with neighbors of best
      std::vector<int> new_pool;
      for (int c : pool) {
        if (!in_clique[c] && std::binary_search(conflicts[best].begin(), conflicts[best].end(), c)) {
          new_pool.push_back(c);
        }
      }
      pool = std::move(new_pool);
    }

    if (clique.size() < 2) continue;

    // Check violation: sum(x_i, i in clique) > 1
    double lhs_val = 0.0;
    for (int c : clique) lhs_val += x[c];
    double viol = lhs_val - 1.0;
    if (viol > 1e-7) {
      candidates.push_back({std::move(clique), viol});
    }
  }

  // Sort by violation descending
  std::sort(candidates.begin(), candidates.end(), [](const CliqueCandidate& a, const CliqueCandidate& b) {
    return a.violation > b.violation;
  });

  int added = 0;
  std::vector<Eigen::SparseVector<double>> cut_rows;
  std::vector<double> cut_rhs;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);

  for (const auto& cand : candidates) {
    if (added >= max_cuts) break;
    cut_rows.push_back(sparse_indicator_cut(n, cand.clique));
    storage_tracker.record(cut_rows.back(), cut_rows.size());
    cut_rhs.push_back(1.0);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_sparse_rows_to_lp(lp, cut_rows, cut_rhs, storage_stats);
  }
  return added;
}

// ════════════════════════════════════════════════════════════════════════════
// Zero-half cuts — Caprara-Fischetti-Letchford style {0,1/2}-cuts
// ════════════════════════════════════════════════════════════════════════════
//
// For rows Ax ≤ b with integer variables, combine pairs of rows with
// multiplier 1/2. If the aggregated integer-variable coefficients are all
// integer (the "zero-half" property), floor the RHS to get a valid cut.
//
// We support mixed-integer rows by: (a) ensuring the continuous-variable
// coefficients sum to integers after aggregation, OR (b) bounding the
// continuous contribution via variable bounds and absorbing it into the RHS.
int add_zero_half_cuts(LPModel& lp,
                       const Eigen::VectorXd& x,
                       int max_cuts,
                       SeparatorRowWorkspace& row_workspace,
                       SeparatorStorageStats* storage_stats) {
  if (max_cuts <= 0) return 0;

  const int m = row_workspace.inequality_rows();
  const int n = static_cast<int>(lp.vars.size());
  if (m < 2) return 0;

  // Step 1: For each row, build a "mod-2 signature" of integer variable
  // coefficients. Two rows can be combined for a zero-half cut if their
  // odd-integer-coefficient positions match (XOR to zero in GF(2)).
  // Also track whether continuous variables have integer coefficients.
  struct RowInfo {
    int row;
    double slack;              // b_r - a_r^T x
    int b_rounded;             // round(b_r) — needed for parity check
    std::vector<int> odd_cols; // sorted positions of odd-integer-coeff integer vars
    uint64_t fp;               // hash fingerprint of odd_cols
    bool cont_all_int;         // all continuous-var coefficients are near-integer
  };

  std::vector<RowInfo> rows;
  rows.reserve(static_cast<size_t>(m));

  for (int r = 0; r < m; ++r) {
    double activity = 0.0;
    std::vector<int> odd;
    bool has_int_var = false;
    bool cont_ok = true;

    row_workspace.visit_inequality_row(r, [&](int j, double a) {
      if (j < 0 || j >= n) return;
      activity += a * x[j];

      if (lp.vars[j].type == VarType::Integer || lp.vars[j].type == VarType::Binary) {
        has_int_var = true;
        const int a_int = static_cast<int>(std::round(a));
        if (std::abs(a - a_int) < 1e-6 && (a_int & 1)) {
          odd.push_back(j);
        }
      } else {
        // Continuous variable: check if coefficient is near-integer
        if (std::abs(a - std::round(a)) > 1e-6) cont_ok = false;
      }
    });

    if (!has_int_var || odd.empty()) continue;

    // Guard against non-finite or very large RHS to prevent int overflow below.
    if (!std::isfinite(lp.b[r]) || std::abs(lp.b[r]) >= 2e9) continue;

    const double slack = lp.b[r] - activity;
    if (slack < -1e-6 || slack > 0.99) continue; // too violated or too loose

    std::sort(odd.begin(), odd.end());

    // FNV-1a hash fingerprint of odd column positions
    uint64_t h = 14695981039346656037ULL;
    for (int c : odd) {
      h ^= static_cast<uint64_t>(c);
      h *= 1099511628211ULL;
    }

    rows.push_back({r, slack,
                     static_cast<int>(std::round(lp.b[r])),
                     std::move(odd), h, cont_ok});
  }

  if (rows.size() < 2) return 0;

  // Step 2: Group rows by fingerprint for O(m) pair matching.
  std::unordered_map<uint64_t, std::vector<int>> groups;
  groups.reserve(rows.size());
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    groups[rows[i].fp].push_back(i);
  }

  // Step 3: For each group, try pairwise combinations.
  // The cut from combining rows r1, r2 (each with multiplier 1/2):
  //   (a_r1 + a_r2)/2 · x ≤ floor((b_r1 + b_r2)/2)
  // Violation = (b_r1+b_r2)/2 - floor(...)  - (slack_r1+slack_r2)/2
  //           = frac((b_r1+b_r2)/2) - (slack_r1+slack_r2)/2
  // For this to be positive, need (b_r1+b_r2) odd and small slacks.
  struct CutCand {
    int i1, i2;
    double violation;
  };
  std::vector<CutCand> cands;

  for (auto& [fp, idxs] : groups) {
    if (idxs.size() < 2) continue;

    // Sort by slack (tightest first) for best violations.
    std::sort(idxs.begin(), idxs.end(), [&](int a, int b) {
      return rows[a].slack < rows[b].slack;
    });

    // Limit pairs per group to avoid O(k^2) blowup in large groups.
    const int k = std::min(static_cast<int>(idxs.size()), 30);
    for (int i = 0; i < k; ++i) {
      for (int j = i + 1; j < k; ++j) {
        auto& r1 = rows[idxs[i]];
        auto& r2 = rows[idxs[j]];

        // Verify real match (not just hash collision).
        if (r1.odd_cols != r2.odd_cols) continue;

        // RHS parity: sum must be odd for fractional aggregated RHS.
        if (((r1.b_rounded + r2.b_rounded) & 1) == 0) continue;

        // Both rows must have integer continuous-var coefficients for safe rounding.
        if (!r1.cont_all_int || !r2.cont_all_int) continue;

        // Violation estimate.
        double viol = 0.5 - 0.5 * (r1.slack + r2.slack);
        if (viol < 1e-4) continue;

        cands.push_back({idxs[i], idxs[j], viol});
      }
    }
  }

  if (cands.empty()) return 0;

  // Sort by violation (best first).
  std::sort(cands.begin(), cands.end(),
            [](const CutCand& a, const CutCand& b) { return a.violation > b.violation; });

  // Step 4: Build and add the cuts.
  int added = 0;
  std::vector<Eigen::SparseVector<double>> cut_rows_out;
  std::vector<double> cut_rhs_out;
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  // Track norms for parallelism filtering.
  std::vector<double> cut_norms;

  for (const auto& cc : cands) {
    if (added >= max_cuts) break;

    auto& ri = rows[cc.i1];
    auto& rj = rows[cc.i2];

    // Floor RHS.
    double rhs = std::floor(0.5 * (lp.b[ri.row] + lp.b[rj.row]) + 1e-9);

    const auto terms_i = row_workspace.inequality_row_terms(ri.row);
    const auto terms_j = row_workspace.inequality_row_terms(rj.row);
    Eigen::SparseVector<double> coeff(n);
    coeff.reserve(static_cast<int>(terms_i.size() + terms_j.size()));
    std::size_t pos_i = 0;
    std::size_t pos_j = 0;
    double lhs = 0.0;
    double norm_sq = 0.0;
    bool cut_invalid = false;
    while (pos_i < terms_i.size() || pos_j < terms_j.size()) {
      const int col_i =
          pos_i < terms_i.size() ? terms_i[pos_i].first : n;
      const int col_j =
          pos_j < terms_j.size() ? terms_j[pos_j].first : n;
      const int col = std::min(col_i, col_j);
      if (col < 0 || col >= n) {
        if (col_i == col) ++pos_i;
        if (col_j == col) ++pos_j;
        continue;
      }
      double value = 0.0;
      if (col_i == col) {
        value += 0.5 * terms_i[pos_i].second;
        ++pos_i;
      }
      if (col_j == col) {
        value += 0.5 * terms_j[pos_j].second;
        ++pos_j;
      }
      if (is_integer_type(lp.vars[static_cast<std::size_t>(col)])) {
        value = std::round(value);
      } else if (std::abs(value) > 1e-9) {
        // SOUNDNESS: a zero-half (0-1/2 Chvatal-Gomory) cut floors the
        // aggregated RHS, which is valid only if the aggregated LHS is
        // integer-valued for every feasible point. A continuous variable with a
        // nonzero aggregated coefficient (e.g. 0.5*(1+0)=0.5 when the two source
        // rows have odd coefficient sum) leaves a non-integer term in the LHS,
        // so flooring the RHS over-tightens and can cut off a feasible optimum
        // (MIPLIB3 `gen`: the cut 0.5*x598 + ... <= -1 excluded the true
        // optimum, whose activity was -0.5). The `cont_all_int` pre-filter only
        // checks the ORIGINAL row coefficients, not the aggregate, so it does
        // not catch this. Reject the whole cut. (See Caprara & Fischetti,
        // "0-1/2 Chvatal-Gomory cuts", Math. Prog. 1996 — the construction is
        // pure-integer; continuous columns must aggregate to zero.)
        cut_invalid = true;
        break;
      }
      if (std::abs(value) <= 1e-12) continue;
      coeff.insertBack(col) = value;
      lhs += value * x[col];
      norm_sq += value * value;
    }
    if (cut_invalid) continue;

    double actual_viol = lhs - rhs;
    if (actual_viol < 1e-4) continue;

    // Parallelism filter: reject if too similar to existing cuts.
    // Use the configured parallelism threshold to preserve orthogonality.
    double cnorm = std::sqrt(norm_sq);
    if (cnorm < 1e-12) continue;
    bool parallel = false;
    for (int ci = 0; ci < static_cast<int>(cut_norms.size()); ++ci) {
      double dot = sparse_sparse_dot(coeff, cut_rows_out[ci]);
      if (std::abs(dot) > 0.90 * cnorm * cut_norms[ci]) {
        parallel = true;
        break;
      }
    }
    if (parallel) continue;

    cut_norms.push_back(cnorm);
    cut_rows_out.push_back(std::move(coeff));
    storage_tracker.record(cut_rows_out.back(), cut_rows_out.size());
    cut_rhs_out.push_back(rhs);
    ++added;
  }

  if (!cut_rows_out.empty()) {
    add_sparse_rows_to_lp(lp, cut_rows_out, cut_rhs_out, storage_stats);
  }
  return added;
}

int add_cuts(LPModel& lp,
             const Eigen::VectorXd& x,
             const SimplexResult* simplex,
             const BCOptions& opt,
             int max_cuts,
             const std::shared_ptr<BasisOps>& sbasis,
             CutFamilyTracker* tracker,
             const CliqueTable* clique_table,
             SeparatorStorageStats* storage_stats) {
  if (opt.cuts == CutType::None || max_cuts <= 0) {
    return 0;
  }

  int budget = max_cuts;
  int total = 0;
  SeparatorRowWorkspace row_workspace(lp);

  // Helper to compute average efficacy of cuts added to LP rows [start_row, end_row).
  // Efficacy = violation / ||coeff|| where violation = coeff*x - rhs > 0.
  auto compute_efficacy = [&](int start_row, int end_row) -> double {
    if (end_row <= start_row) return 0.0;
    const int n = static_cast<int>(x.size());
    const int num_rows = end_row - start_row;
    Eigen::VectorXd lhs = Eigen::VectorXd::Zero(num_rows);
    Eigen::VectorXd norm_sq = Eigen::VectorXd::Zero(num_rows);
    for (int col = 0; col < lp.A.outerSize(); ++col) {
      if (col >= n) break;
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
        const int row = static_cast<int>(it.row());
        if (row < start_row || row >= end_row) continue;
        const int local_row = row - start_row;
        lhs[local_row] += it.value() * x[col];
        norm_sq[local_row] += it.value() * it.value();
      }
    }
    double sum = 0.0;
    int count = 0;
    for (int i = start_row; i < end_row; ++i) {
      const int local_row = i - start_row;
      if (norm_sq[local_row] > 1e-12) {
        double violation = lhs[local_row] - lp.b[i];
        double efficacy =
            std::max(0.0, violation) / std::sqrt(norm_sq[local_row]);
        sum += efficacy;
        ++count;
      }
    }
    return (count > 0) ? (sum / count) : 0.0;
  };

  // Helper to spend budget on a cut family and track efficacy.
  auto spend_tracked = [&](CutFamily family, const std::function<int(int)>& f, int family_budget) {
    if (budget <= 0) return;
    if (tracker && tracker->should_skip(family)) {
      tracker->get(family).skipped_rounds += 1;
      return;
    }
    
    const int start_rows = static_cast<int>(lp.A.rows());
    const int actual_budget = std::min(budget, family_budget);
    if (tracker) {
      tracker->get(family).attempts += 1;
      tracker->get(family).budget_total += actual_budget;
    }
    const int add = std::max(0, f(actual_budget));
    row_workspace.sync_inequalities(lp);
    const int end_rows = static_cast<int>(lp.A.rows());

    if (tracker && add > 0) {
      double eff = compute_efficacy(start_rows, end_rows);
      tracker->get(family).add_round(eff);
      tracker->get(family).cuts_total += add;
      tracker->get(family).efficacy_sum += eff * add;
    } else if (tracker) {
      tracker->get(family).zero_rounds += 1;
    }

    total += add;
    budget -= add;
  };

  auto spend = [&](const std::function<int(int)>& f) {
    if (budget <= 0) {
      return;
    }
    const int add = std::max(0, f(budget));
    row_workspace.sync_inequalities(lp);
    total += add;
    budget -= add;
  };

  if (opt.cuts == CutType::IntRounding) {
    spend([&](int b) {
      return add_mir_like_cuts(lp, x, CutType::IntRounding, b,
                               row_workspace, storage_stats);
    });
    return total;
  }
  if (opt.cuts == CutType::MIR) {
    // True MIR: first try basis-based MIR, then row-based MIR
    if (simplex != nullptr) {
      spend_tracked(CutFamily::MIR, [&](int b) {
        return add_basis_mir_cuts(lp, x, *simplex, opt, b, sbasis,
                                  row_workspace, storage_stats);
      }, budget);
    }
    spend_tracked(CutFamily::MIR, [&](int b) {
      return add_row_mir_cuts(lp, x, b, row_workspace, storage_stats);
    }, budget);
    return total;
  }
  if (opt.cuts == CutType::Gomory) {
    if (simplex != nullptr) {
      spend_tracked(CutFamily::Gomory,
                    [&](int b) {
	                      return add_transformed_tableau_cuts(
	                          lp, x, *simplex, opt, b, sbasis, nullptr, nullptr,
	                          nullptr, nullptr, 1e-8, nullptr, std::nullopt,
                              storage_stats);
                    },
                    budget);
    }
    return total;
  }
  if (opt.cuts == CutType::Cover) {
    spend_tracked(CutFamily::Cover, [&](int b) {
      return add_cover_cuts(lp, x, b, row_workspace, storage_stats);
    }, budget);
    if (opt.enable_projected_capacity_cuts) {
      spend_tracked(CutFamily::Cover, [&](int b) {
        return add_projected_capacity_cover_cuts(
            lp, x, b, row_workspace, storage_stats);
      }, budget);
    }
    return total;
  }

  // CutType::All — generate all applicable cut families.
  // Gomory gets full budget first (strongest cuts). Supplementary cuts
  // share the remaining budget to add complementary strength.
  // Skip families with low moving-average efficacy. Reduce supplementary cut
  // budgets and disable expensive families
  //        on very large problems (n > 20000).
  const bool xlarge = (static_cast<int>(x.size()) > 20000);
  const bool large = (static_cast<int>(x.size()) > 10000);

  // Prefer violated cliques from the persistent clique table when available.
  // The table is built once at root; this amortises the O(m*k^2) conflict
  // graph construction across every cut round.
  auto add_clique_cuts_from_table = [&](int b) -> int {
    if (b <= 0 || clique_table == nullptr || clique_table->empty()) return 0;
    const int nx = static_cast<int>(x.size());
    auto cliques = clique_table->find_violated_cliques(x, b, 1e-7, xlarge ? 128 : 512);
    if (cliques.empty()) return 0;
    std::vector<Eigen::SparseVector<double>> rows;
    std::vector<double> rhs;
    SeparatorCandidateStorageTracker storage_tracker(storage_stats);
    rows.reserve(cliques.size());
    rhs.reserve(cliques.size());
    for (const auto& cq : cliques) {
      rows.push_back(sparse_indicator_cut(nx, cq.members));
      storage_tracker.record(rows.back(), rows.size());
      rhs.push_back(1.0);
    }
    add_sparse_rows_to_lp(lp, rows, rhs, storage_stats);
    return static_cast<int>(rows.size());
  };

  // 1. Gomory cuts (strongest, need simplex basis) — full budget
  if (simplex != nullptr) {
    spend_tracked(CutFamily::Gomory,
                  [&](int b) {
	                    return add_transformed_tableau_cuts(
	                        lp, x, *simplex, opt, b, sbasis, nullptr, nullptr,
	                        nullptr, nullptr, 1e-8, nullptr, std::nullopt,
                            storage_stats);
                  },
                  budget);
  }
  // 2. Basis-MIR cuts (complemented single-row MIR, stronger than row-MIR, needs simplex basis)
  if (simplex != nullptr && !xlarge) {
    spend_tracked(CutFamily::MIR, [&](int b) {
      return add_basis_mir_cuts(lp, x, *simplex, opt, b, sbasis,
                                row_workspace, storage_stats);
    }, xlarge ? 0 : (large ? 2 : 4));
  }
  // 3. Row-based MIR cuts (works without basis, complementary to Gomory)
  spend_tracked(CutFamily::MIR, [&](int b) {
    return add_row_mir_cuts(lp, x, b, row_workspace, storage_stats);
  }, xlarge ? 2 : (large ? 3 : 5));
  // 3. Unlifted knapsack cover cuts
  if (opt.enable_projected_capacity_cuts) {
    spend_tracked(CutFamily::Cover,
                  [&](int b) {
                    return add_projected_capacity_cover_cuts(
                        lp, x, b, row_workspace, storage_stats);
                  },
                  budget);
  }
  spend_tracked(CutFamily::Cover, [&](int b) {
    return add_cover_cuts(lp, x, b, row_workspace, storage_stats);
  }, xlarge ? 2 : (large ? 3 : 5));

  // Clique cuts: always attempt when the persistent table is available
  // (near-zero cost when table is empty; fast lookup otherwise).
  if (clique_table != nullptr && !clique_table->empty()) {
    spend_tracked(CutFamily::Clique, add_clique_cuts_from_table,
                  xlarge ? 2 : (large ? 3 : 5));
  }
  // Remaining expensive supplementary families only on smaller problems.
  if (!large) {
    // Fallback clique cuts (on-the-fly conflict graph) when no table present.
    if (clique_table == nullptr) {
      spend_tracked(CutFamily::Clique, [&](int b) {
        return add_clique_cuts(lp, x, b, row_workspace, storage_stats);
      }, 5);
    }
    // Zero-half cuts (pairwise {0,1/2}-aggregation) — disable on very large
    if (!xlarge) {
      spend_tracked(CutFamily::ZeroHalf, [&](int b) {
        return add_zero_half_cuts(lp, x, b, row_workspace, storage_stats);
      }, 5);
    }
  }
  return total;
}


}  // namespace mipsolvers::engine::detail

/// @file bc_clique_table.cpp
/// @brief Persistent clique table implementation.

#include "hacdcpf/engine/detail/bc_clique_table.hpp"
#include "hacdcpf/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_set>
#include <utility>

namespace hacdcpf::engine::detail {

void CliqueTable::ensure_storage(int n) {
  if (n_ == n && !offsets_.empty() && !lit_offsets_.empty()) return;
  n_ = std::max(0, n);
  n_edges_ = 0;
  n_literal_edges_ = 0;
  offsets_.assign(static_cast<std::size_t>(n_) + 1, 0);
  adj_.clear();
  lit_offsets_.assign(static_cast<std::size_t>(2 * n_) + 1, 0);
  lit_adj_.clear();
}

void CliqueTable::rebuild_variable_csr(
    const std::vector<std::vector<int>>& adj) {
  std::size_t total = 0;
  for (int j = 0; j < n_; ++j) total += adj[static_cast<std::size_t>(j)].size();

  adj_.resize(total);
  offsets_.assign(static_cast<std::size_t>(n_) + 1, 0);
  offsets_[0] = 0;
  for (int j = 0; j < n_; ++j) {
    const auto& v = adj[static_cast<std::size_t>(j)];
    std::size_t off = offsets_[static_cast<std::size_t>(j)];
    for (int nb : v) adj_[off++] = nb;
    offsets_[static_cast<std::size_t>(j) + 1] = off;
  }
  n_edges_ = total / 2;
}

void CliqueTable::rebuild_literal_csr(
    const std::vector<std::vector<int>>& adj) {
  const int nlits = 2 * n_;
  std::size_t total = 0;
  for (int j = 0; j < nlits; ++j) total += adj[static_cast<std::size_t>(j)].size();

  lit_adj_.resize(total);
  lit_offsets_.assign(static_cast<std::size_t>(nlits) + 1, 0);
  lit_offsets_[0] = 0;
  for (int j = 0; j < nlits; ++j) {
    const auto& v = adj[static_cast<std::size_t>(j)];
    std::size_t off = lit_offsets_[static_cast<std::size_t>(j)];
    for (int nb : v) lit_adj_[off++] = nb;
    lit_offsets_[static_cast<std::size_t>(j) + 1] = off;
  }
  n_literal_edges_ = total / 2;
}

std::size_t CliqueTable::build(const LPModel& lp,
                               int max_row_nnz,
                               const std::vector<char>* binary_like_override) {
  ensure_storage(static_cast<int>(lp.vars.size()));
  n_edges_ = 0;
  n_literal_edges_ = 0;
  offsets_.assign(static_cast<std::size_t>(n_) + 1, 0);
  adj_.clear();
  lit_offsets_.assign(static_cast<std::size_t>(2 * n_) + 1, 0);
  lit_adj_.clear();

  if (n_ <= 0) return 0;

  // Row-major views for efficient per-row iteration.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;
  const int m = static_cast<int>(A_row.rows());

  // Collect conflict pairs deduplicated with a per-variable sorted insert.
  // Use adjacency lists in std::vector, then compact to CSR at the end.
  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n_));
  std::vector<std::vector<int>> lit_adj(static_cast<std::size_t>(2 * n_));

  auto add_literal_edge = [&](int col_a, bool val_a, int col_b, bool val_b) {
    const int ia = literal_index(col_a, val_a);
    const int ib = literal_index(col_b, val_b);
    if (ia < 0 || ib < 0 || ia == ib || ia >= 2 * n_ || ib >= 2 * n_) {
      return;
    }
    lit_adj[static_cast<std::size_t>(ia)].push_back(ib);
    lit_adj[static_cast<std::size_t>(ib)].push_back(ia);
    if (val_a && val_b) {
      adj[static_cast<std::size_t>(col_a)].push_back(col_b);
      adj[static_cast<std::size_t>(col_b)].push_back(col_a);
    }
  };

  // Scratch buffer reused across rows.
  struct LiteralTerm {
    int col{-1};
    bool value_one{false};
    double coeff{0.0};
  };
  std::vector<LiteralTerm> row_entries;
  row_entries.reserve(static_cast<std::size_t>(max_row_nnz));

  const double kCoeffEps = 1e-12;

  auto scan_side = [&](int r, double side_sign, double rhs) {
    if (!std::isfinite(rhs)) return;

    row_entries.clear();
    bool usable = true;
    double transformed_rhs = rhs;
    int transformed_nnz = 0;

    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r);
         it; ++it) {
      const double v = side_sign * it.value();
      if (std::abs(v) <= kCoeffEps) continue;
      const int j = it.col();
      if (j < 0 || j >= n_) {
        usable = false;
        break;
      }
      ++transformed_nnz;
      if (transformed_nnz > max_row_nnz) {
        usable = false;
        break;
      }

      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      const bool binary_like =
          var.type == VarType::Binary ||
          ((var.type == VarType::Integer) && std::isfinite(var.lb) &&
           std::isfinite(var.ub) && var.lb >= -1e-9 &&
           var.ub <= 1.0 + 1e-9) ||
          (binary_like_override != nullptr &&
           j < static_cast<int>(binary_like_override->size()) &&
           (*binary_like_override)[static_cast<std::size_t>(j)] != 0 &&
           std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9);

      if (v > 0.0) {
        if (!std::isfinite(var.lb)) {
          usable = false;
          break;
        }
        transformed_rhs -= v * var.lb;
        if (binary_like) {
          row_entries.push_back(LiteralTerm{j, true, v});
        }
      } else {
        if (!std::isfinite(var.ub)) {
          usable = false;
          break;
        }
        transformed_rhs -= v * var.ub;
        if (binary_like) {
          row_entries.push_back(LiteralTerm{j, false, -v});
        }
      }
    }

    if (!usable || row_entries.size() < 2) return;
    if (!(transformed_rhs > -1e-12)) return;

    std::sort(row_entries.begin(), row_entries.end(),
              [](const LiteralTerm& a, const LiteralTerm& b) {
                if (std::abs(a.coeff - b.coeff) > 1e-12) {
                  return a.coeff > b.coeff;
                }
                if (a.col != b.col) return a.col < b.col;
                return a.value_one > b.value_one;
              });

    if (row_entries.size() >= 2 &&
        row_entries[0].coeff + row_entries[1].coeff <=
            transformed_rhs + 1e-9) {
      return;
    }

    const int k = static_cast<int>(row_entries.size());
    for (int a = 0; a < k; ++a) {
      const auto& ta = row_entries[static_cast<std::size_t>(a)];
      if (ta.coeff + row_entries[0].coeff <= transformed_rhs + 1e-9 &&
          a > 0) {
        break;
      }
      for (int c = a + 1; c < k; ++c) {
        const auto& tc = row_entries[static_cast<std::size_t>(c)];
        if (ta.col == tc.col) continue;
        if (ta.coeff + tc.coeff <= transformed_rhs + 1e-9) {
          break;
        }
        add_literal_edge(ta.col, ta.value_one, tc.col, tc.value_one);
      }
    }
  };

  for (int r = 0; r < m; ++r) {
    scan_side(r, 1.0, lp.b[r]);
    const double lhs = lp_row_lhs_or_neg_inf(lp, r);
    if (std::isfinite(lhs)) scan_side(r, -1.0, -lhs);
  }
  auto scan_eq_side = [&](int r, double side_sign, double rhs) {
    if (!std::isfinite(rhs)) return;

    row_entries.clear();
    bool usable = true;
    double transformed_rhs = rhs;
    int transformed_nnz = 0;

    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r);
         it; ++it) {
      const double v = side_sign * it.value();
      if (std::abs(v) <= kCoeffEps) continue;
      const int j = it.col();
      if (j < 0 || j >= n_) {
        usable = false;
        break;
      }
      ++transformed_nnz;
      if (transformed_nnz > max_row_nnz) {
        usable = false;
        break;
      }

      const auto& var = lp.vars[static_cast<std::size_t>(j)];
      const bool binary_like =
          var.type == VarType::Binary ||
          ((var.type == VarType::Integer) && std::isfinite(var.lb) &&
           std::isfinite(var.ub) && var.lb >= -1e-9 &&
           var.ub <= 1.0 + 1e-9) ||
          (binary_like_override != nullptr &&
           j < static_cast<int>(binary_like_override->size()) &&
           (*binary_like_override)[static_cast<std::size_t>(j)] != 0 &&
           std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9);

      if (v > 0.0) {
        if (!std::isfinite(var.lb)) {
          usable = false;
          break;
        }
        transformed_rhs -= v * var.lb;
        if (binary_like) row_entries.push_back(LiteralTerm{j, true, v});
      } else {
        if (!std::isfinite(var.ub)) {
          usable = false;
          break;
        }
        transformed_rhs -= v * var.ub;
        if (binary_like) row_entries.push_back(LiteralTerm{j, false, -v});
      }
    }

    if (!usable || row_entries.size() < 2) return;
    if (!(transformed_rhs > -1e-12)) return;

    std::sort(row_entries.begin(), row_entries.end(),
              [](const LiteralTerm& a, const LiteralTerm& b) {
                if (std::abs(a.coeff - b.coeff) > 1e-12) {
                  return a.coeff > b.coeff;
                }
                if (a.col != b.col) return a.col < b.col;
                return a.value_one > b.value_one;
              });

    if (row_entries[0].coeff + row_entries[1].coeff <=
        transformed_rhs + 1e-9) {
      return;
    }

    const int k = static_cast<int>(row_entries.size());
    for (int a = 0; a < k; ++a) {
      const auto& ta = row_entries[static_cast<std::size_t>(a)];
      if (ta.coeff + row_entries[0].coeff <= transformed_rhs + 1e-9 &&
          a > 0) {
        break;
      }
      for (int c = a + 1; c < k; ++c) {
        const auto& tc = row_entries[static_cast<std::size_t>(c)];
        if (ta.col == tc.col) continue;
        if (ta.coeff + tc.coeff <= transformed_rhs + 1e-9) break;
        add_literal_edge(ta.col, ta.value_one, tc.col, tc.value_one);
      }
    }
  };
  for (int r = 0; r < Aeq_row.rows(); ++r) {
    scan_eq_side(r, 1.0, lp.beq[r]);
    scan_eq_side(r, -1.0, -lp.beq[r]);
  }

  for (int j = 0; j < n_; ++j) {
    auto& v = adj[static_cast<std::size_t>(j)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  for (int j = 0; j < 2 * n_; ++j) {
    auto& v = lit_adj[static_cast<std::size_t>(j)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  rebuild_variable_csr(adj);
  rebuild_literal_csr(lit_adj);
  return n_edges_;
}

std::size_t CliqueTable::add_edges(
    const LPModel& lp, const std::vector<std::pair<int, int>>& edges) {
  if (edges.empty()) return 0;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0) return 0;
  ensure_storage(n);

  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n_));
  for (int j = 0; j < n_; ++j) {
    auto rng = neighbours(j);
    for (const int* p = rng.first; p != rng.second; ++p) {
      adj[static_cast<std::size_t>(j)].push_back(*p);
    }
  }
  std::vector<std::vector<int>> lit_adj(static_cast<std::size_t>(2 * n_));
  for (int lit = 0; lit < 2 * n_; ++lit) {
    if (static_cast<std::size_t>(lit + 1) >= lit_offsets_.size()) break;
    const int* base = lit_adj_.data();
    const auto begin = lit_offsets_[static_cast<std::size_t>(lit)];
    const auto end = lit_offsets_[static_cast<std::size_t>(lit) + 1];
    for (std::size_t p = begin; p < end; ++p) {
      lit_adj[static_cast<std::size_t>(lit)].push_back(base[p]);
    }
  }

  std::size_t before = 0;
  for (auto& v : adj) before += v.size();

  auto binary_like = [&](int j) {
    if (j < 0 || j >= n_) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.type == VarType::Binary) return true;
    return std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9;
  };

  for (auto [a, b] : edges) {
    if (a < 0 || b < 0 || a >= n_ || b >= n_ || a == b) continue;
    if (!binary_like(a) || !binary_like(b)) continue;
    adj[static_cast<std::size_t>(a)].push_back(b);
    adj[static_cast<std::size_t>(b)].push_back(a);
    const int la = literal_index(a, true);
    const int lb = literal_index(b, true);
    lit_adj[static_cast<std::size_t>(la)].push_back(lb);
    lit_adj[static_cast<std::size_t>(lb)].push_back(la);
  }

  for (int j = 0; j < n_; ++j) {
    auto& v = adj[static_cast<std::size_t>(j)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  for (int lit = 0; lit < 2 * n_; ++lit) {
    auto& v = lit_adj[static_cast<std::size_t>(lit)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  std::size_t total = 0;
  for (const auto& v : adj) total += v.size();
  rebuild_variable_csr(adj);
  rebuild_literal_csr(lit_adj);
  return total > before ? (total - before) / 2 : 0;
}

std::size_t CliqueTable::add_literal_edges(
    const LPModel& lp,
    const std::vector<std::pair<Literal, Literal>>& edges) {
  if (edges.empty()) return 0;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0) return 0;
  ensure_storage(n);

  std::vector<std::vector<int>> adj(static_cast<std::size_t>(n_));
  for (int j = 0; j < n_; ++j) {
    auto rng = neighbours(j);
    for (const int* p = rng.first; p != rng.second; ++p) {
      adj[static_cast<std::size_t>(j)].push_back(*p);
    }
  }

  std::vector<std::vector<int>> lit_adj(static_cast<std::size_t>(2 * n_));
  for (int lit = 0; lit < 2 * n_; ++lit) {
    if (static_cast<std::size_t>(lit + 1) >= lit_offsets_.size()) break;
    const int* base = lit_adj_.data();
    const auto begin = lit_offsets_[static_cast<std::size_t>(lit)];
    const auto end = lit_offsets_[static_cast<std::size_t>(lit) + 1];
    for (std::size_t p = begin; p < end; ++p) {
      lit_adj[static_cast<std::size_t>(lit)].push_back(base[p]);
    }
  }

  std::size_t before = 0;
  for (auto& v : lit_adj) before += v.size();

  auto binary_like = [&](int j) {
    if (j < 0 || j >= n_) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.type == VarType::Binary) return true;
    return std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9;
  };

  for (const auto& edge : edges) {
    const Literal a = edge.first;
    const Literal b = edge.second;
    if (a.col < 0 || b.col < 0 || a.col >= n_ || b.col >= n_ ||
        a.col == b.col || !binary_like(a.col) || !binary_like(b.col)) {
      continue;
    }
    const int ia = literal_index(a.col, a.value_one);
    const int ib = literal_index(b.col, b.value_one);
    if (ia < 0 || ib < 0 || ia == ib) continue;
    lit_adj[static_cast<std::size_t>(ia)].push_back(ib);
    lit_adj[static_cast<std::size_t>(ib)].push_back(ia);
    if (a.value_one && b.value_one) {
      adj[static_cast<std::size_t>(a.col)].push_back(b.col);
      adj[static_cast<std::size_t>(b.col)].push_back(a.col);
    }
  }

  for (int j = 0; j < n_; ++j) {
    auto& v = adj[static_cast<std::size_t>(j)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  for (int lit = 0; lit < 2 * n_; ++lit) {
    auto& v = lit_adj[static_cast<std::size_t>(lit)];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  std::size_t total = 0;
  for (const auto& v : lit_adj) total += v.size();
  rebuild_variable_csr(adj);
  rebuild_literal_csr(lit_adj);
  return total > before ? (total - before) / 2 : 0;
}

std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
CliqueTable::extract_literal_edges_from_cut(
    const LPModel& lp,
    const Eigen::SparseVector<double>& row_le,
    double rhs_le,
    int max_row_nnz,
    const std::vector<char>* binary_like_override,
    const BinaryImplicationGraph* implication_graph,
    CutCliqueExtractionStats* stats,
    const Eigen::VectorXd* lp_solution) {
  if (stats != nullptr) ++stats->rows_scanned;
  std::vector<std::pair<Literal, Literal>> edges;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0 || row_le.size() != n || !std::isfinite(rhs_le) ||
      max_row_nnz < 2) {
    return edges;
  }

  struct LiteralTerm {
    int col{-1};
    bool value_one{false};
    double coeff{0.0};
  };
  std::vector<LiteralTerm> terms;
  terms.reserve(static_cast<std::size_t>(std::min(max_row_nnz, 256)));
  constexpr double kCoeffEps = 1e-12;
  double residual_capacity = rhs_le;

  auto binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    const bool unit_bounds =
        std::isfinite(var.lb) && std::isfinite(var.ub) &&
        var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9;
    if ((var.type == VarType::Binary ||
         (var.type == VarType::Integer && unit_bounds)) &&
        unit_bounds) {
      return true;
    }
    return binary_like_override != nullptr &&
           j < static_cast<int>(binary_like_override->size()) &&
           (*binary_like_override)[static_cast<std::size_t>(j)] != 0 &&
           unit_bounds;
  };

  struct VarBoundExpr {
    bool valid{false};
    int trigger_col{-1};
    bool trigger_value_one{false};
    double constant{0.0};
    double coef{0.0};
    double improvement{0.0};
  };

  std::vector<VarBoundExpr> best_vlb(static_cast<std::size_t>(n));
  std::vector<VarBoundExpr> best_vub(static_cast<std::size_t>(n));
  std::vector<std::vector<VarBoundExpr>> vlb_choices(static_cast<std::size_t>(n));
  std::vector<std::vector<VarBoundExpr>> vub_choices(static_cast<std::size_t>(n));
  auto insert_choice = [](std::vector<VarBoundExpr>& choices,
                          const VarBoundExpr& expr) {
    if (!expr.valid || expr.trigger_col < 0) return;
    for (auto& existing : choices) {
      if (existing.trigger_col != expr.trigger_col) continue;
      if (expr.improvement > existing.improvement + 1e-9) {
        existing = expr;
      }
      return;
    }
    choices.push_back(expr);
    std::sort(choices.begin(), choices.end(),
              [](const VarBoundExpr& a, const VarBoundExpr& b) {
                return a.improvement > b.improvement;
              });
    constexpr std::size_t kMaxChoices = 6;
    if (choices.size() > kMaxChoices) choices.resize(kMaxChoices);
  };
  if (implication_graph != nullptr && !implication_graph->empty()) {
    for (int trigger = 0; trigger < n; ++trigger) {
      if (!binary_like(trigger)) continue;
      for (bool trigger_one : {false, true}) {
        auto range = implication_graph->implications(trigger, trigger_one);
        for (const auto* p = range.first; p != range.second; ++p) {
          if (p == nullptr || p->var_idx < 0 || p->var_idx >= n ||
              p->var_idx == trigger || binary_like(p->var_idx) ||
              !std::isfinite(p->value)) {
            continue;
          }
          const auto& implied_var =
              lp.vars[static_cast<std::size_t>(p->var_idx)];
          if (p->is_lb) {
            if (!std::isfinite(implied_var.lb) ||
                p->value <= implied_var.lb + 1e-9) {
              continue;
            }
            const double delta = p->value - implied_var.lb;
            VarBoundExpr expr;
            expr.valid = true;
            expr.trigger_col = trigger;
            expr.trigger_value_one = trigger_one;
            expr.improvement = delta;
            if (trigger_one) {
              // x >= lb + delta*y
              expr.constant = implied_var.lb;
              expr.coef = delta;
            } else {
              // x >= lb + delta*(1-y) = (lb+delta) - delta*y
              expr.constant = p->value;
              expr.coef = -delta;
            }
            auto& best = best_vlb[static_cast<std::size_t>(p->var_idx)];
            if (!best.valid || expr.improvement > best.improvement + 1e-9) {
              best = expr;
            }
            insert_choice(vlb_choices[static_cast<std::size_t>(p->var_idx)],
                          expr);
          } else {
            if (!std::isfinite(implied_var.ub) ||
                p->value >= implied_var.ub - 1e-9) {
              continue;
            }
            const double delta = implied_var.ub - p->value;
            VarBoundExpr expr;
            expr.valid = true;
            expr.trigger_col = trigger;
            expr.trigger_value_one = trigger_one;
            expr.improvement = delta;
            if (trigger_one) {
              // x <= ub - delta*y
              expr.constant = implied_var.ub;
              expr.coef = -delta;
            } else {
              // x <= ub - delta*(1-y) = (ub-delta) + delta*y
              expr.constant = p->value;
              expr.coef = delta;
            }
            auto& best = best_vub[static_cast<std::size_t>(p->var_idx)];
            if (!best.valid || expr.improvement > best.improvement + 1e-9) {
              best = expr;
            }
            insert_choice(vub_choices[static_cast<std::size_t>(p->var_idx)],
                          expr);
          }
        }
      }
    }
  }

  std::vector<double> trigger_mass(static_cast<std::size_t>(n), 0.0);
  auto choose_bound_expression =
      [&](int target_col,
          const std::vector<VarBoundExpr>& choices,
          double row_coeff,
          bool lower_bound) -> const VarBoundExpr* {
    const VarBoundExpr* selected = nullptr;
    double best_score = -std::numeric_limits<double>::infinity();
    const bool have_lp_solution =
        lp_solution != nullptr && lp_solution->size() >= n;
    for (const VarBoundExpr& expr : choices) {
      if (!expr.valid || expr.trigger_col < 0 || expr.trigger_col >= n) {
        continue;
      }
      const double beta = row_coeff * expr.coef;
      if (std::abs(beta) <= kCoeffEps) {
        continue;
      }
      double score = 0.0;
      if (have_lp_solution) {
        const double target_value = (*lp_solution)[target_col];
        const double trigger_value = std::clamp((*lp_solution)[expr.trigger_col],
                                                0.0, 1.0);
        const double bound_value =
            expr.constant + expr.coef * trigger_value;
        const double dist = lower_bound
            ? std::max(0.0, target_value - bound_value)
            : std::max(0.0, bound_value - target_value);
        const double y_dist = 1e-9 + (lower_bound
            ? (expr.coef > 0.0 ? trigger_value : 1.0 - trigger_value)
            : (expr.coef > 0.0 ? 1.0 - trigger_value : trigger_value));
        const double norm2 = 1.0 + expr.coef * expr.coef;
        if (dist * dist > y_dist * y_dist * norm2 + 1e-12) {
          continue;
        }
        const double endpoint_a = expr.constant;
        const double endpoint_b = expr.constant + expr.coef;
        const double extremal_bound = lower_bound
            ? std::max(endpoint_a, endpoint_b)
            : -std::min(endpoint_a, endpoint_b);
        score = -dist + 1e-9 * extremal_bound;
      } else {
        const double used =
            trigger_mass[static_cast<std::size_t>(expr.trigger_col)];
        const double novelty = used <= kCoeffEps ? 1.0 : 1.0 / (1.0 + used);
        score = std::abs(beta) * novelty +
                1e-9 * std::max(0.0, expr.improvement);
      }
      if (score > best_score) {
        best_score = score;
        selected = &expr;
      }
    }
    return selected;
  };

  auto add_bound_expression = [&](double constant, double beta, int trigger) {
    residual_capacity -= constant;
    if (trigger < 0 || trigger >= n || std::abs(beta) <= kCoeffEps ||
        !binary_like(trigger)) {
      return;
    }
    if (beta > 0.0) {
      terms.push_back(LiteralTerm{trigger, true, beta});
    } else {
      residual_capacity -= beta;
      terms.push_back(LiteralTerm{trigger, false, -beta});
    }
    trigger_mass[static_cast<std::size_t>(trigger)] += std::abs(beta);
  };

  int nnz = 0;
  for (Eigen::SparseVector<double>::InnerIterator it(row_le); it; ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (std::abs(a) <= kCoeffEps) continue;
    if (j < 0 || j >= n || !std::isfinite(a)) return edges;
    if (++nnz > max_row_nnz) return edges;

    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (a > 0.0) {
      if (binary_like(j)) {
        if (!std::isfinite(var.lb)) return edges;
        residual_capacity -= a * var.lb;
        terms.push_back(LiteralTerm{j, true, a});
        trigger_mass[static_cast<std::size_t>(j)] += std::abs(a);
      } else if (!vlb_choices[static_cast<std::size_t>(j)].empty()) {
        const auto* expr = choose_bound_expression(
            j, vlb_choices[static_cast<std::size_t>(j)], a,
            /*lower_bound=*/true);
        if (expr == nullptr) {
          if (!std::isfinite(var.lb)) return edges;
          residual_capacity -= a * var.lb;
          continue;
        }
        add_bound_expression(a * expr->constant, a * expr->coef,
                             expr->trigger_col);
      } else {
        if (!std::isfinite(var.lb)) return edges;
        residual_capacity -= a * var.lb;
      }
    } else {
      if (binary_like(j)) {
        if (!std::isfinite(var.ub)) return edges;
        residual_capacity -= a * var.ub;
        terms.push_back(LiteralTerm{j, false, -a});
        trigger_mass[static_cast<std::size_t>(j)] += std::abs(a);
      } else if (!vub_choices[static_cast<std::size_t>(j)].empty()) {
        const auto* expr = choose_bound_expression(
            j, vub_choices[static_cast<std::size_t>(j)], a,
            /*lower_bound=*/false);
        if (expr == nullptr) {
          if (!std::isfinite(var.ub)) return edges;
          residual_capacity -= a * var.ub;
          continue;
        }
        add_bound_expression(a * expr->constant, a * expr->coef,
                             expr->trigger_col);
      } else {
        if (!std::isfinite(var.ub)) return edges;
        residual_capacity -= a * var.ub;
      }
    }
  }

  if (terms.size() < 2) return edges;

  // Multiple implied-bound substitutions can contribute to the same trigger
  // literal.  Treat them as one literal mass before testing pair excess; using
  // two copies of the same trigger would create a fake pair that cannot become
  // a conflict edge.
  std::sort(terms.begin(), terms.end(),
            [](const LiteralTerm& a, const LiteralTerm& b) {
              if (a.col != b.col) return a.col < b.col;
              return a.value_one > b.value_one;
            });
  int write = 0;
  for (int read = 0; read < static_cast<int>(terms.size()); ++read) {
    const auto& cur = terms[static_cast<std::size_t>(read)];
    if (std::abs(cur.coeff) <= kCoeffEps) continue;
    if (write > 0) {
      auto& prev = terms[static_cast<std::size_t>(write - 1)];
      if (prev.col == cur.col && prev.value_one == cur.value_one) {
        prev.coeff += cur.coeff;
        continue;
      }
    }
    terms[static_cast<std::size_t>(write++)] = cur;
  }
  terms.resize(static_cast<std::size_t>(write));
  terms.erase(std::remove_if(terms.begin(), terms.end(),
                             [](const LiteralTerm& t) {
                               return std::abs(t.coeff) <= 1e-12;
                             }),
              terms.end());
  if (terms.size() < 2) return edges;

  if (stats != nullptr) ++stats->rows_with_binary_literals;
  if (residual_capacity < 0.0) residual_capacity = 0.0;
  if (!std::isfinite(residual_capacity)) return edges;

  std::sort(terms.begin(), terms.end(),
            [](const LiteralTerm& a, const LiteralTerm& b) {
              if (std::abs(a.coeff - b.coeff) > 1e-12) {
                return a.coeff > b.coeff;
              }
              if (a.col != b.col) return a.col < b.col;
              return a.value_one > b.value_one;
            });

  const int k = static_cast<int>(terms.size());
  const double unit_excess = terms[0].coeff - residual_capacity;
  if (stats != nullptr) {
    stats->best_unit_margin =
        std::max(stats->best_unit_margin, unit_excess);
  }
  if (stats != nullptr && unit_excess > 1e-9) {
    ++stats->rows_with_unit_excess;
    stats->max_unit_excess =
        std::max(stats->max_unit_excess, unit_excess);
  }
  double top_excess = 0.0;
  double best_pair_margin = -1e100;
  for (int a = 0; a < k; ++a) {
    const auto& ta = terms[static_cast<std::size_t>(a)];
    for (int b = a + 1; b < k; ++b) {
      const auto& tb = terms[static_cast<std::size_t>(b)];
      if (ta.col == tb.col) continue;
      const double excess = ta.coeff + tb.coeff - residual_capacity;
      best_pair_margin = std::max(best_pair_margin, excess);
      if (excess <= 1e-9) break;
      top_excess = std::max(top_excess, excess);
      edges.emplace_back(Literal{ta.col, ta.value_one},
                         Literal{tb.col, tb.value_one});
    }
  }
  if (stats != nullptr && best_pair_margin > -1e90) {
    stats->best_pair_margin =
        std::max(stats->best_pair_margin, best_pair_margin);
  }
  if (edges.empty()) return edges;
  if (stats != nullptr) {
    ++stats->rows_with_pair_excess;
    stats->max_pair_excess = std::max(stats->max_pair_excess, top_excess);
    stats->literal_edges_generated += edges.size();
  }
  return edges;
}

std::size_t CliqueTable::add_literal_edges_from_cut(
    const LPModel& lp,
    const Eigen::SparseVector<double>& row_le,
    double rhs_le,
    int max_row_nnz,
    const std::vector<char>* binary_like_override,
    const BinaryImplicationGraph* implication_graph,
    CutCliqueExtractionStats* stats,
    const Eigen::VectorXd* lp_solution) {
  auto edges = extract_literal_edges_from_cut(
      lp, row_le, rhs_le, max_row_nnz, binary_like_override,
      implication_graph, stats, lp_solution);
  return add_literal_edges(lp, edges);
}

int CliqueTable::propagate(Eigen::VectorXd& lb,
                           Eigen::VectorXd& ub,
                           std::vector<BoundChangeInfo>* changes_out) const {
  if (n_edges_ == 0 && n_literal_edges_ == 0) return 0;
  const int nx = std::min(n_, static_cast<int>(lb.size()));
  int tightened = 0;
  if (n_literal_edges_ > 0) {
    std::deque<int> queue;
    std::vector<char> seen(static_cast<std::size_t>(2 * nx), 0);
    auto enqueue_literal = [&](int col, bool value_one) {
      if (col < 0 || col >= nx) return;
      const int idx = literal_index(col, value_one);
      if (idx < 0 || idx >= 2 * nx) return;
      auto& flag = seen[static_cast<std::size_t>(idx)];
      if (flag) return;
      flag = 1;
      queue.push_back(idx);
    };
    for (int i = 0; i < nx; ++i) {
      if (lb[i] >= 0.5 && ub[i] >= 0.5) enqueue_literal(i, true);
      if (ub[i] <= 0.5 && lb[i] <= 0.5) enqueue_literal(i, false);
    }
    while (!queue.empty()) {
      const int lit = queue.front();
      queue.pop_front();
      if (static_cast<std::size_t>(lit + 1) >= lit_offsets_.size()) continue;
      const int* base = lit_adj_.data();
      const auto begin = lit_offsets_[static_cast<std::size_t>(lit)];
      const auto end = lit_offsets_[static_cast<std::size_t>(lit) + 1];
      for (std::size_t p = begin; p < end; ++p) {
        const int forbidden = base[p];
        const int j = forbidden / 2;
        const bool forbidden_one = (forbidden % 2) != 0;
        if (j < 0 || j >= nx) continue;
        if (forbidden_one) {
          if (ub[j] > 0.5) {
            if (changes_out != nullptr) changes_out->push_back({j, -ub[j], false});
            ub[j] = 0.0;
            ++tightened;
            enqueue_literal(j, false);
          }
        } else {
          if (lb[j] < 0.5) {
            if (changes_out != nullptr) changes_out->push_back({j, 1.0 - lb[j], true});
            lb[j] = 1.0;
            ++tightened;
            enqueue_literal(j, true);
          }
        }
      }
    }
    return tightened;
  }

  for (int i = 0; i < nx; ++i) {
    // Triggered only when x_i is hard-fixed to 1.
    if (lb[i] < 0.5) continue;
    if (ub[i] < 0.5) continue;  // infeasible combo, caller will detect
    auto range = neighbours(i);
    for (const int* p = range.first; p != range.second; ++p) {
      const int j = *p;
      if (j >= ub.size()) continue;
      if (ub[j] > 0.5) {
        if (changes_out != nullptr) {
          changes_out->push_back({j, -ub[j], false});
        }
        ub[j] = 0.0;
        ++tightened;
        if (lb[j] > 0.5) {
          // Conflict: x_i = x_j = 1 is impossible; leave ub[j] < lb[j] so
          // caller sees inconsistent bounds.
        }
      }
    }
  }
  return tightened;
}

std::vector<CliqueTable::ViolatedClique>
CliqueTable::find_violated_cliques(const Eigen::VectorXd& x,
                                   int max_cliques,
                                   double tol,
                                   int max_seeds) const {
  std::vector<ViolatedClique> out;
  if (max_cliques <= 0 || n_edges_ == 0) return out;
  const int nx = std::min(n_, static_cast<int>(x.size()));

  // Seed candidates: fractional binaries with at least one conflict.
  std::vector<int> seeds;
  seeds.reserve(static_cast<std::size_t>(nx));
  for (int j = 0; j < nx; ++j) {
    if (degree(j) == 0) continue;
    const double v = x[j];
    if (v > 0.05 && v < 0.95) seeds.push_back(j);
  }
  std::sort(seeds.begin(), seeds.end(),
            [&](int a, int b) { return x[a] > x[b]; });
  if (static_cast<int>(seeds.size()) > max_seeds) seeds.resize(static_cast<std::size_t>(max_seeds));

  std::vector<char> in_clique(static_cast<std::size_t>(n_), 0);

  for (int seed : seeds) {
    if (static_cast<int>(out.size()) >= max_cliques) break;
    std::vector<int> clique = {seed};
    in_clique[static_cast<std::size_t>(seed)] = 1;

    // Candidate pool: neighbours of seed that are also fractional / positive.
    std::vector<int> pool;
    {
      auto rng = neighbours(seed);
      pool.reserve(static_cast<std::size_t>(rng.second - rng.first));
      for (const int* p = rng.first; p != rng.second; ++p) {
        if (*p < nx && x[*p] > 1e-6) pool.push_back(*p);
      }
    }

    double clique_sum = x[seed];

    while (!pool.empty()) {
      // Pick best candidate by x-value.
      int best = -1;
      double best_val = -1.0;
      int best_idx = -1;
      for (std::size_t k = 0; k < pool.size(); ++k) {
        const int c = pool[k];
        if (in_clique[static_cast<std::size_t>(c)]) continue;
        if (x[c] > best_val) {
          best_val = x[c];
          best = c;
          best_idx = static_cast<int>(k);
        }
      }
      if (best < 0 || best_val < 1e-6) break;

      // Remove selected from pool (swap-and-pop).
      pool[static_cast<std::size_t>(best_idx)] = pool.back();
      pool.pop_back();

      // `best` must conflict with every current clique member.
      bool all_conflict = true;
      for (int cm : clique) {
        if (!has_edge(best, cm)) { all_conflict = false; break; }
      }
      if (!all_conflict) continue;

      clique.push_back(best);
      in_clique[static_cast<std::size_t>(best)] = 1;
      clique_sum += best_val;

      // Intersect pool with neighbours of `best`.
      auto rng = neighbours(best);
      std::vector<int> new_pool;
      new_pool.reserve(pool.size());
      for (int c : pool) {
        if (!in_clique[static_cast<std::size_t>(c)] &&
            std::binary_search(rng.first, rng.second, c)) {
          new_pool.push_back(c);
        }
      }
      pool.swap(new_pool);
    }

    // Reset in_clique scratch.
    for (int c : clique) in_clique[static_cast<std::size_t>(c)] = 0;

    if (clique.size() < 2) continue;
    const double viol = clique_sum - 1.0;
    if (viol > tol) {
      std::sort(clique.begin(), clique.end());
      out.push_back({std::move(clique), viol});
    }
  }

  std::sort(out.begin(), out.end(),
            [](const ViolatedClique& a, const ViolatedClique& b) {
              return a.violation > b.violation;
            });
  return out;
}

}  // namespace hacdcpf::engine::detail

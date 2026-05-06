/// @file bc_pools.hpp
/// @brief Cut pool and solution pool for the branch-and-cut solver.
///
/// CutPool stores globally generated cutting planes for cheap re-separation.
/// SolutionPool keeps the top-K feasible solutions for guided rounding heuristics.

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/detail/bc_types.hpp"

namespace hacdcpf::engine::detail {

/// @brief Compute a structural hash for a sparse vector (for fast duplicate rejection).
inline std::size_t sparse_cut_hash(const Eigen::SparseVector<double>& v) {
  std::size_t h = 0;
  const std::size_t kPrime = 0x9e3779b97f4a7c15ULL;
  for (Eigen::SparseVector<double>::InnerIterator it(v); it; ++it) {
    // Combine index and quantized value into hash.
    std::size_t idx_hash = std::hash<int>{}(it.index());
    // Quantize to 4 decimal places to tolerate small perturbations.
    int64_t qval = static_cast<int64_t>(it.value() * 1e4);
    std::size_t val_hash = std::hash<int64_t>{}(qval);
    h ^= (idx_hash * kPrime + val_hash) + kPrime + (h << 6) + (h >> 2);
  }
  return h;
}

/// @brief Convert a dense vector to a sparse vector, dropping near-zero entries.
inline Eigen::SparseVector<double> dense_to_sparse_cut(const Eigen::VectorXd& dense, double tol = 1e-15) {
  Eigen::SparseVector<double> sp(dense.size());
  int nnz = 0;
  for (int i = 0; i < dense.size(); ++i) {
    if (std::abs(dense[i]) > tol) ++nnz;
  }
  sp.reserve(nnz);
  for (int i = 0; i < dense.size(); ++i) {
    if (std::abs(dense[i]) > tol) sp.insertBack(i) = dense[i];
  }
  return sp;
}

/// @brief Dot product of a sparse vector with a dense vector.
inline double sparse_dot(const Eigen::SparseVector<double>& sp, const Eigen::VectorXd& dense) {
  double sum = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(sp); it; ++it) {
    if (it.index() < 0 || it.index() >= dense.size() ||
        !std::isfinite(it.value())) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    sum += it.value() * dense[it.index()];
  }
  return sum;
}

/// @brief Dot product of two sparse vectors.
inline double sparse_sparse_dot(const Eigen::SparseVector<double>& a, const Eigen::SparseVector<double>& b) {
  double sum = 0.0;
  Eigen::SparseVector<double>::InnerIterator ia(a);
  Eigen::SparseVector<double>::InnerIterator ib(b);
  while (ia && ib) {
    if (ia.index() < ib.index()) ++ia;
    else if (ia.index() > ib.index()) ++ib;
    else { sum += ia.value() * ib.value(); ++ia; ++ib; }
  }
  return sum;
}

/// @brief Structural hash for a branch-domain literal.
inline std::size_t branch_literal_hash(const BranchDomainLiteral& literal) {
  std::size_t h = std::hash<int>{}(literal.var_idx);
  const std::size_t bound_hash = std::hash<bool>{}(literal.is_lb);
  const long long qval = static_cast<long long>(std::llround(literal.value * 1e6));
  const std::size_t value_hash = std::hash<long long>{}(qval);
  h ^= bound_hash + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  h ^= value_hash + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

/// @brief Canonicalize a literal list by removing dominated duplicates.
inline void canonicalize_branch_literals(std::vector<BranchDomainLiteral>& literals,
                                         double tol = 1e-9) {
  std::sort(literals.begin(), literals.end(), [](const BranchDomainLiteral& a,
                                                 const BranchDomainLiteral& b) {
    if (a.var_idx != b.var_idx) return a.var_idx < b.var_idx;
    if (a.is_lb != b.is_lb) return a.is_lb < b.is_lb;
    return a.value < b.value;
  });

  std::vector<BranchDomainLiteral> compact;
  compact.reserve(literals.size());
  for (const auto& literal : literals) {
    if (compact.empty() || compact.back().var_idx != literal.var_idx ||
        compact.back().is_lb != literal.is_lb) {
      compact.push_back(literal);
      continue;
    }

    auto& prev = compact.back();
    if (literal.is_lb) {
      if (literal.value > prev.value + tol) prev.value = literal.value;
    } else {
      if (literal.value < prev.value - tol) prev.value = literal.value;
    }
  }

  literals.swap(compact);
}

/// @brief Hash for a canonicalized conflict clause.
inline std::size_t conflict_clause_hash(const std::vector<BranchDomainLiteral>& literals) {
  std::size_t h = 0;
  for (const auto& literal : literals) {
    const std::size_t literal_hash = branch_literal_hash(literal);
    h ^= literal_hash + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  }
  return h;
}

/// @brief True when the first literal is at least as strong as the second.
inline bool branch_literal_stronger_or_equal(const BranchDomainLiteral& lhs,
                                             const BranchDomainLiteral& rhs,
                                             double tol = 1e-9) {
  if (lhs.var_idx != rhs.var_idx || lhs.is_lb != rhs.is_lb) {
    return false;
  }
  if (lhs.is_lb) {
    return lhs.value >= rhs.value - tol;
  }
  return lhs.value <= rhs.value + tol;
}

/// @brief True when satisfying all lhs literals implies all rhs literals.
inline bool conflict_clause_implies(const std::vector<BranchDomainLiteral>& lhs,
                                    const std::vector<BranchDomainLiteral>& rhs,
                                    double tol = 1e-9) {
  for (const auto& target : rhs) {
    bool covered = false;
    for (const auto& source : lhs) {
      if (branch_literal_stronger_or_equal(source, target, tol)) {
        covered = true;
        break;
      }
    }
    if (!covered) {
      return false;
    }
  }
  return true;
}

/// @brief Stores learned branch-domain conflicts for pruning and bound propagation.
class ConflictPool {
  std::vector<ConflictClause> clauses_;
  int max_pool_size_;
  int max_literals_;

  bool redundant_given_pool(const std::vector<BranchDomainLiteral>& literals,
                            int skip_index = -1) const {
    for (int i = 0; i < static_cast<int>(clauses_.size()); ++i) {
      if (i == skip_index) continue;
      if (conflict_clause_implies(literals, clauses_[i].literals)) {
        return true;
      }
    }
    return false;
  }

  void minimize_clause(std::vector<BranchDomainLiteral>& literals) const {
    canonicalize_branch_literals(literals);
    bool changed = true;
    while (changed && literals.size() > 1) {
      changed = false;
      for (std::size_t idx = 0; idx < literals.size(); ++idx) {
        std::vector<BranchDomainLiteral> reduced = literals;
        reduced.erase(reduced.begin() + static_cast<std::ptrdiff_t>(idx));
        if (redundant_given_pool(reduced)) {
          literals.swap(reduced);
          changed = true;
          break;
        }
      }
    }
  }

  static bool is_binary_fixed_zero(const std::vector<VariableMeta>& vars,
                                   const BranchDomainLiteral& literal,
                                   double tol = 1e-9) {
    return literal.var_idx >= 0 &&
           literal.var_idx < static_cast<int>(vars.size()) &&
           vars[static_cast<std::size_t>(literal.var_idx)].type == VarType::Binary &&
           !literal.is_lb &&
           literal.value <= tol;
  }

  static bool is_binary_fixed_one(const std::vector<VariableMeta>& vars,
                                  const BranchDomainLiteral& literal,
                                  double tol = 1e-9) {
    return literal.var_idx >= 0 &&
           literal.var_idx < static_cast<int>(vars.size()) &&
           vars[static_cast<std::size_t>(literal.var_idx)].type == VarType::Binary &&
           literal.is_lb &&
           literal.value >= 1.0 - tol;
  }

  static bool binary_literals_complementary(const std::vector<VariableMeta>& vars,
                                            const BranchDomainLiteral& a,
                                            const BranchDomainLiteral& b,
                                            double tol = 1e-9) {
    if (a.var_idx != b.var_idx) return false;
    return (is_binary_fixed_zero(vars, a, tol) &&
            is_binary_fixed_one(vars, b, tol)) ||
           (is_binary_fixed_one(vars, a, tol) &&
            is_binary_fixed_zero(vars, b, tol));
  }

  static bool has_binary_complement_pair(
      const std::vector<VariableMeta>& vars,
      const std::vector<BranchDomainLiteral>& literals,
      double tol = 1e-9) {
    for (std::size_t i = 0; i < literals.size(); ++i) {
      for (std::size_t k = i + 1; k < literals.size(); ++k) {
        if (binary_literals_complementary(vars, literals[i], literals[k], tol)) {
          return true;
        }
      }
    }
    return false;
  }

 public:
  explicit ConflictPool(int max_size = 1024, int max_literals = 64)
      : max_pool_size_(max_size), max_literals_(max_literals) {}

  bool empty() const { return clauses_.empty(); }

  bool add(std::vector<BranchDomainLiteral> literals) {
    minimize_clause(literals);
    if (literals.empty() || static_cast<int>(literals.size()) > max_literals_) {
      return false;
    }

    if (redundant_given_pool(literals)) {
      return false;
    }

    const std::size_t h = conflict_clause_hash(literals);
    for (const auto& clause : clauses_) {
      if (clause.hash != h || clause.literals.size() != literals.size()) continue;
      bool same = true;
      for (std::size_t i = 0; i < literals.size(); ++i) {
        const auto& a = clause.literals[i];
        const auto& b = literals[i];
        if (a.var_idx != b.var_idx || a.is_lb != b.is_lb ||
            std::abs(a.value - b.value) > 1e-9) {
          same = false;
          break;
        }
      }
      if (same) return false;
    }

    clauses_.erase(std::remove_if(clauses_.begin(), clauses_.end(),
        [&](const ConflictClause& clause) {
          return conflict_clause_implies(clause.literals, literals);
        }),
        clauses_.end());

    if (static_cast<int>(clauses_.size()) >= max_pool_size_) {
      clauses_.erase(clauses_.begin());
    }
    clauses_.push_back(ConflictClause{std::move(literals), h});
    return true;
  }

  int add_with_binary_resolution(
      const std::vector<VariableMeta>& vars,
      std::vector<BranchDomainLiteral> literals,
      int max_resolvents,
      std::vector<std::vector<BranchDomainLiteral>>* added_clauses = nullptr) {
    if (added_clauses != nullptr) added_clauses->clear();
    canonicalize_branch_literals(literals);
    if (literals.empty() || static_cast<int>(literals.size()) > max_literals_) {
      return 0;
    }

    const int old_count = static_cast<int>(clauses_.size());
    if (!add(literals)) {
      return 0;
    }
    int added = 1;
    if (added_clauses != nullptr && !clauses_.empty()) {
      added_clauses->push_back(clauses_.back().literals);
    }
    if (max_resolvents <= 0) {
      return added;
    }

    const std::vector<BranchDomainLiteral> base_clause =
        added_clauses != nullptr && !added_clauses->empty()
            ? added_clauses->front()
            : clauses_.back().literals;
    for (int ci = 0; ci < old_count && added <= max_resolvents; ++ci) {
      if (ci >= static_cast<int>(clauses_.size())) break;
      const auto& other = clauses_[static_cast<std::size_t>(ci)].literals;
      int base_complement = -1;
      int other_complement = -1;
      int complement_count = 0;
      for (int a = 0; a < static_cast<int>(base_clause.size()); ++a) {
        for (int b = 0; b < static_cast<int>(other.size()); ++b) {
          if (binary_literals_complementary(vars, base_clause[static_cast<std::size_t>(a)],
                                            other[static_cast<std::size_t>(b)])) {
            base_complement = a;
            other_complement = b;
            ++complement_count;
            if (complement_count > 1) break;
          }
        }
        if (complement_count > 1) break;
      }
      if (complement_count != 1) continue;

      std::vector<BranchDomainLiteral> resolvent;
      resolvent.reserve(base_clause.size() + other.size() - 2);
      for (int a = 0; a < static_cast<int>(base_clause.size()); ++a) {
        if (a != base_complement) resolvent.push_back(base_clause[static_cast<std::size_t>(a)]);
      }
      for (int b = 0; b < static_cast<int>(other.size()); ++b) {
        if (b != other_complement) resolvent.push_back(other[static_cast<std::size_t>(b)]);
      }
      canonicalize_branch_literals(resolvent);
      if (resolvent.empty() ||
          static_cast<int>(resolvent.size()) > max_literals_ ||
          resolvent.size() >= base_clause.size() ||
          resolvent.size() >= other.size() ||
          has_binary_complement_pair(vars, resolvent)) {
        continue;
      }
      if (add(resolvent)) {
        ++added;
        if (added_clauses != nullptr && !clauses_.empty()) {
          added_clauses->push_back(clauses_.back().literals);
        }
      }
    }
    return added;
  }

  bool has_conflict(const Eigen::VectorXd& node_lb,
                    const Eigen::VectorXd& node_ub,
                    double tol = 1e-9) const {
    for (const auto& clause : clauses_) {
      bool all_satisfied = true;
      for (const auto& literal : clause.literals) {
        const bool satisfied = literal.is_lb
            ? (node_lb[literal.var_idx] >= literal.value - tol)
            : (node_ub[literal.var_idx] <= literal.value + tol);
        if (!satisfied) {
          all_satisfied = false;
          break;
        }
      }
      if (all_satisfied) return true;
    }
    return false;
  }

  bool propagate(const std::vector<VariableMeta>& vars,
                 Eigen::VectorXd& node_lb,
                 Eigen::VectorXd& node_ub,
                 int* tightened = nullptr,
                 std::vector<BoundChangeInfo>* changes_out = nullptr,
                 double tol = 1e-9) const {
    return propagate_impl(vars, node_lb, node_ub, tightened, changes_out,
                          nullptr, tol);
  }

  bool propagate_with_reasons(const std::vector<VariableMeta>& vars,
                              Eigen::VectorXd& node_lb,
                              Eigen::VectorXd& node_ub,
                              int* tightened,
                              std::vector<BoundChangeInfo>* changes_out,
                              std::vector<DomainReasonBound>* reason_bounds_out,
                              double tol = 1e-9) const {
    return propagate_impl(vars, node_lb, node_ub, tightened, changes_out,
                          reason_bounds_out, tol);
  }

 private:
  bool propagate_impl(const std::vector<VariableMeta>& vars,
                      Eigen::VectorXd& node_lb,
                      Eigen::VectorXd& node_ub,
                      int* tightened,
                      std::vector<BoundChangeInfo>* changes_out,
                      std::vector<DomainReasonBound>* reason_bounds_out,
                      double tol) const {
    int local_tightened = 0;
    bool changed = true;
    while (changed) {
      changed = false;
      for (const auto& clause : clauses_) {
        int unsatisfied_count = 0;
        const BranchDomainLiteral* remaining = nullptr;
        std::vector<BranchDomainLiteral> reason;
        reason.reserve(clause.literals.size());
        for (const auto& literal : clause.literals) {
          const bool satisfied = literal.is_lb
              ? (node_lb[literal.var_idx] >= literal.value - tol)
              : (node_ub[literal.var_idx] <= literal.value + tol);
          if (!satisfied) {
            ++unsatisfied_count;
            remaining = &literal;
            if (unsatisfied_count > 1) break;
          } else {
            reason.push_back(literal);
          }
        }

        if (unsatisfied_count == 0) {
          if (tightened) *tightened += local_tightened;
          return false;
        }
        if (unsatisfied_count != 1 || remaining == nullptr) {
          continue;
        }

        const int j = remaining->var_idx;
        if (j < 0 || j >= static_cast<int>(vars.size())) {
          continue;
        }
        const bool integer_var = is_integer_type(vars[static_cast<std::size_t>(j)]);

        if (remaining->is_lb) {
          const double new_ub = integer_var
              ? std::ceil(remaining->value - tol) - 1.0
              : remaining->value - std::max(1e-7, 10.0 * tol);
          if (new_ub < node_ub[j] - tol) {
            if (changes_out != nullptr) {
              changes_out->push_back({j, new_ub - node_ub[j], false});
            }
            node_ub[j] = new_ub;
            if (reason_bounds_out != nullptr) {
              reason_bounds_out->push_back(
                  {BranchDomainLiteral{j, new_ub, false}, reason, -1, 0,
                   *remaining, true});
            }
            ++local_tightened;
            changed = true;
          }
        } else {
          const double new_lb = integer_var
              ? std::floor(remaining->value + tol) + 1.0
              : remaining->value + std::max(1e-7, 10.0 * tol);
          if (new_lb > node_lb[j] + tol) {
            if (changes_out != nullptr) {
              changes_out->push_back({j, new_lb - node_lb[j], true});
            }
            node_lb[j] = new_lb;
            if (reason_bounds_out != nullptr) {
              reason_bounds_out->push_back(
                  {BranchDomainLiteral{j, new_lb, true}, reason, -1, 0,
                   *remaining, true});
            }
            ++local_tightened;
            changed = true;
          }
        }

        if (node_lb[j] > node_ub[j] + tol) {
          if (tightened) *tightened += local_tightened;
          return false;
        }
      }
    }

    if (tightened) *tightened += local_tightened;
    return true;
  }

 public:
  int size() const { return static_cast<int>(clauses_.size()); }

  const ConflictClause& operator[](int i) const { return clauses_[i]; }
};

/// @brief Stores globally generated cuts for cheap re-separation at tree nodes.
/// @details Cuts are evicted by age (consecutive non-violations) and duplicates
/// are rejected via cosine similarity.
class CutPool {
  std::vector<PoolCut> cuts_;
  int max_pool_size_;
  int max_age_;
  int expected_size_;

  bool valid_cut_input(const Eigen::SparseVector<double>& coeff,
                       double rhs,
                       double* norm_out = nullptr) const {
    if (!std::isfinite(rhs)) return false;
    if (expected_size_ >= 0 && coeff.size() != expected_size_) return false;
    double norm2 = 0.0;
    for (Eigen::SparseVector<double>::InnerIterator it(coeff); it; ++it) {
      if (it.index() < 0 || it.index() >= coeff.size() ||
          !std::isfinite(it.value())) {
        return false;
      }
      norm2 += it.value() * it.value();
      if (!std::isfinite(norm2)) return false;
    }
    const double norm = std::sqrt(norm2);
    if (!(norm >= 1e-12) || !std::isfinite(norm)) return false;
    if (norm_out != nullptr) *norm_out = norm;
    return true;
  }

public:
  /// @brief Construct a cut pool with given capacity and age limit.
  /// @param max_size Maximum cuts stored in the pool.
  /// @param max_age Purge cuts after this many consecutive non-violations.
  explicit CutPool(int max_size = 2000, int max_age = 50,
                   int expected_size = -1)
      : max_pool_size_(max_size),
        max_age_(max_age),
        expected_size_(expected_size) {}

  void set_expected_size(int expected_size) {
    expected_size_ = expected_size;
  }

  /// @brief Add a sparse cut to the pool.
  /// @return false if duplicate (hash match + cosine > 0.95).
  bool add(Eigen::SparseVector<double> coeff, double rhs) {
    double norm = 0.0;
    if (!valid_cut_input(coeff, rhs, &norm)) return false;

    const std::size_t h = sparse_cut_hash(coeff);

    // Fast duplicate rejection: only compute cosine for cuts with matching hash.
    for (const auto& pc : cuts_) {
      if (pc.hash == h) {
        double dot = sparse_sparse_dot(pc.coeff, coeff);
        if (std::abs(dot) / (pc.norm * norm) > 0.95) {
          return false;
        }
      }
    }

    if (static_cast<int>(cuts_.size()) >= max_pool_size_) {
      auto worst = std::max_element(cuts_.begin(), cuts_.end(),
          [](const PoolCut& a, const PoolCut& b) { return a.age < b.age; });
      if (worst != cuts_.end()) {
        *worst = PoolCut{std::move(coeff), rhs, 0, 0.0, norm, h};
        return true;
      }
    }

    cuts_.push_back(PoolCut{std::move(coeff), rhs, 0, 0.0, norm, h});
    return true;
  }

  /// @brief Convenience overload: add a dense cut (converts to sparse).
  bool add(const Eigen::VectorXd& coeff_dense, double rhs) {
    return add(dense_to_sparse_cut(coeff_dense), rhs);
  }

  /// @brief Find indices of pool cuts violated by x (violation > min_viol).
  /// @return Indices sorted by violation (descending).
  std::vector<int> find_violated(const Eigen::VectorXd& x, double min_viol = 1e-4) const {
    std::vector<std::pair<double, int>> violated;
    for (int i = 0; i < static_cast<int>(cuts_.size()); ++i) {
      if (cuts_[i].coeff.size() != x.size()) continue;
      const double activity = sparse_dot(cuts_[i].coeff, x);
      if (!std::isfinite(activity) || !std::isfinite(cuts_[i].rhs)) continue;
      const double viol = activity - cuts_[i].rhs;
      if (viol > min_viol) {
        violated.push_back({viol, i});
      }
    }
    std::sort(violated.begin(), violated.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<int> result;
    result.reserve(violated.size());
    for (const auto& p : violated) result.push_back(p.second);
    return result;
  }

  /// @brief Age all cuts by 1. Reset age for violated ones. Purge old cuts.
  void age_and_purge(const std::vector<int>& violated_indices) {
    for (int idx : violated_indices) {
      if (idx >= 0 && idx < static_cast<int>(cuts_.size())) {
        cuts_[idx].age = 0;
      }
    }
    for (int i = 0; i < static_cast<int>(cuts_.size()); ++i) {
      bool was_violated = false;
      for (int idx : violated_indices) {
        if (idx == i) { was_violated = true; break; }
      }
      if (!was_violated) {
        ++cuts_[i].age;
      }
    }
    cuts_.erase(std::remove_if(cuts_.begin(), cuts_.end(),
        [this](const PoolCut& c) { return c.age > max_age_; }),
        cuts_.end());
  }

  /// @brief Access a pool cut by index.
  const PoolCut& operator[](int i) const { return cuts_[i]; }
  /// @brief Number of cuts in the pool.
  int size() const { return static_cast<int>(cuts_.size()); }
};

/// @brief Keeps top-K feasible solutions for guided rounding heuristics.
class SolutionPool {
  std::vector<PoolSolution> solutions_;
  int max_size_;
public:
  /// @brief Construct with given capacity.
  explicit SolutionPool(int max_size = 10) : max_size_(max_size) {}

  /// @brief Add a solution. Returns true if it entered the pool.
  bool add(const Eigen::VectorXd& x, double obj) {
    for (const auto& s : solutions_) {
      if (std::abs(s.obj - obj) < 1e-10) return false;
    }
    if (static_cast<int>(solutions_.size()) < max_size_) {
      solutions_.push_back({x, obj});
      std::sort(solutions_.begin(), solutions_.end(),
          [](const PoolSolution& a, const PoolSolution& b) { return a.obj < b.obj; });
      return true;
    }
    if (obj < solutions_.back().obj) {
      solutions_.back() = {x, obj};
      std::sort(solutions_.begin(), solutions_.end(),
          [](const PoolSolution& a, const PoolSolution& b) { return a.obj < b.obj; });
      return true;
    }
    return false;
  }

  /// @brief Best (lowest objective) solution.
  const PoolSolution& best() const { return solutions_.front(); }
  /// @brief Number of solutions in the pool.
  int size() const { return static_cast<int>(solutions_.size()); }
  /// @brief Access a solution by index (sorted by objective).
  const PoolSolution& operator[](int i) const { return solutions_[i]; }

  /// @brief Guided rounding: round fractional integer variables in the direction
  /// that the majority of pool solutions agree on.
  /// @param x_relax LP relaxation solution to round.
  /// @param vars Variable metadata.
  /// @return Rounded solution vector.
  Eigen::VectorXd guided_rounding(const Eigen::VectorXd& x_relax,
                                   const std::vector<VariableMeta>& vars) const {
    Eigen::VectorXd xr = x_relax;
    const int n = static_cast<int>(vars.size());
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(vars[i])) continue;
      if (is_integral(x_relax[i], 1e-5)) {
        xr[i] = std::round(x_relax[i]);
        continue;
      }
      int up_votes = 0, down_votes = 0;
      const double floor_val = std::floor(x_relax[i]);
      const double ceil_val = std::ceil(x_relax[i]);
      for (const auto& sol : solutions_) {
        if (sol.x.size() > i) {
          const double sv = std::round(sol.x[i]);
          if (sv >= ceil_val - 0.5) ++up_votes;
          else ++down_votes;
        }
      }
      xr[i] = (up_votes > down_votes) ? ceil_val : floor_val;
    }
    return xr;
  }
};

}  // namespace hacdcpf::engine::detail

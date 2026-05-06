#pragma once

#include <cstddef>
#include <vector>

namespace mipsolvers::engine {

/// Filter entry: one (θ, φ) pair previously accepted by the algorithm.
/// New trial points are compared against the filter to enforce global
/// convergence in the Wächter–Biegler sense: a trial point is rejected if
/// there exists an entry that dominates it after the prescribed margins
/// (γ_θ θ, γ_φ θ) are applied.
struct FilterEntry {
  double theta{0.0};
  double phi{0.0};
};

class Filter {
 public:
  Filter() = default;

  /// Whether the given (θ, φ) pair is acceptable to the filter.
  /// The trial is rejected when there exists an entry (θ_k, φ_k) such that
  ///   θ_trial ≥ (1 − γ_θ) θ_k  AND  φ_trial ≥ φ_k − γ_φ θ_k.
  /// Note: γ_θ and γ_φ here are applied to the *existing* filter entries
  /// (the region they forbid expands with their own θ), matching IPOPT.
  bool is_acceptable(double theta_trial, double phi_trial,
                     double gamma_theta, double gamma_phi) const;

  /// Add an entry with the Wächter–Biegler margin semantics. Dominated
  /// existing entries are pruned so the filter size stays bounded.
  void add_entry(double theta, double phi,
                 double gamma_theta, double gamma_phi);

  /// Initialize the filter with a hard upper bound on θ. Used at problem
  /// startup to forbid iterates with very large constraint violation.
  void reset_with_theta_upper_bound(double theta_max);

  /// Clear all entries. Called when the barrier parameter μ is decreased.
  void clear();

  std::size_t size() const { return entries_.size(); }
  const std::vector<FilterEntry>& entries() const { return entries_; }

 private:
  std::vector<FilterEntry> entries_;
};

}  // namespace mipsolvers::engine

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>

#include <Eigen/Core>

namespace hacdcpf::powerflow {

/// Result type returned by NonmonotoneLineSearch::search().
struct LineSearchResult {
  bool accepted{false};      ///< True if an acceptable step was found.
  double alpha{0.0};         ///< Accepted step length (0 if rejected).
  int trials{0};             ///< Number of residual evaluations performed.
  double phi_accepted{0.0};  ///< Merit value at the accepted point.
  double phi_reference{0.0}; ///< Nonmonotone reference φ_k^max.
};

/// Nonmonotone Armijo line search (Grippo–Lampariello–Lucidi, 1986).
///
/// Accepts step α if:
///   φ(x + α·dx) ≤ φ_k^max + c · α · ∇φ(x)ᵀ·dx
/// where φ_k^max = max(φ_{k-j}: 0 ≤ j ≤ M−1) over the last M iterates.
///
/// When M = 1 this reduces to the standard monotone Armijo search.
///
/// Usage:
/// @code
///   NonmonotoneLineSearch ls(window, c, beta);
///   ls.push_merit(phi_current);
///   auto result = ls.search(x, dx, phi_current, dir_deriv, eval_fn);
/// @endcode
class NonmonotoneLineSearch {
 public:
  /// @param window  Sliding window size M ≥ 1.
  /// @param c       Armijo constant c₁ ∈ (0, 1).
  /// @param beta    Step-length reduction β ∈ (0, 1) per trial.
  explicit NonmonotoneLineSearch(int window = 5,
                                  double c = 1e-4,
                                  double beta = 0.5)
      : window_(std::max(1, window)), c_(c), beta_(beta) {}

  /// Push the current merit value into the sliding window.
  void push_merit(double phi) {
    history_.push_back(phi);
    while (static_cast<int>(history_.size()) > window_) {
      history_.pop_front();
    }
  }

  /// Maximum merit value over the current window (the nonmonotone reference).
  double reference_merit() const {
    if (history_.empty()) return std::numeric_limits<double>::infinity();
    return *std::max_element(history_.begin(), history_.end());
  }

  /// Perform the line search.
  ///
  /// @param phi_current        Merit value φ(x) = ½‖F(x)‖₂² at current x.
  /// @param directional_deriv  ∇φ(x)ᵀ·dx = Fᵀ·J·dx (negative for descent).
  /// @param max_trials         Maximum number of backtrack trials.
  /// @param eval_phi           Callable: eval_phi(alpha) → φ(x + alpha·dx).
  /// @return                   LineSearchResult.
  template <typename EvalFn>
  LineSearchResult search(double phi_current,
                          double directional_deriv,
                          int max_trials,
                          EvalFn&& eval_phi) const {
    LineSearchResult result;
    result.phi_reference = reference_merit();

    // Fall back to the current merit if history is empty.
    if (!std::isfinite(result.phi_reference)) {
      result.phi_reference = phi_current;
    }

    double alpha = 1.0;
    for (int trial = 0; trial < max_trials; ++trial) {
      result.trials += 1;
      const double phi_trial = eval_phi(alpha);

      // Nonmonotone Armijo condition:
      //   φ(x + α·dx) ≤ φ_k^max + c · α · ∇φᵀ·dx
      // Note: directional_deriv should be ≤ 0 for a descent direction.
      const double threshold = result.phi_reference + c_ * alpha * directional_deriv;
      if (std::isfinite(phi_trial) && phi_trial <= threshold) {
        result.accepted = true;
        result.alpha = alpha;
        result.phi_accepted = phi_trial;
        return result;
      }
      alpha *= beta_;
    }
    // Not accepted — return the smallest tried alpha for information only.
    result.alpha = alpha;
    return result;
  }

  void clear() { history_.clear(); }

 private:
  int window_;
  double c_;
  double beta_;
  std::deque<double> history_;
};

}  // namespace hacdcpf::powerflow

#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::powerflow {

/// Result of an LM-step computation.
struct LMResult {
  Eigen::VectorXd step;    ///< The LM step Δx (in physical units).
  double lambda_used{0.0}; ///< Damping parameter λ actually applied.
  bool solved{false};      ///< True if the linear solve succeeded.
};

/// Acceptance status for an LM step.
enum class LMAccept {
  Accept,         ///< ρ ≥ 0.1 and actual reduction > 0.
  AcceptExpand,   ///< ρ ≥ 0.75 → accept and decrease λ.
  Reject,         ///< ρ < 0.1 → reject and increase λ.
  ModelInvalid,   ///< pred ≤ 0 → model is invalid; increase λ.
};

/// Adaptive Levenberg–Marquardt damping parameter controller.
///
/// Given the ratio ρ = ared/pred:
///   ρ ≥ 0.75  → accept step, λ ← max(λ/3, λ_min)
///   0.1 ≤ ρ < 0.75  → accept step, keep λ
///   ρ < 0.1  → reject step, λ ← min(λ*10, λ_max)
///   pred ≤ 0  → model invalid, λ ← min(λ*10, λ_max)
///
/// Typical usage:
/// @code
///   LMController ctrl(lambda0, lambda_min, lambda_max);
///   while (...) {
///     auto result = compute_lm_step(Jhat, Fhat, ctrl.lambda());
///     double pred = predict_reduction(Fhat, Jhat, result.step);
///     double ared = actual_reduction(F_old, F_new);
///     ctrl.update(ared, pred);
///   }
/// @endcode
class LMController {
 public:
  explicit LMController(double lambda0 = 1e-4,
                         double lambda_min = 1e-12,
                         double lambda_max = 1e8)
      : lambda_(lambda0), lambda_min_(lambda_min), lambda_max_(lambda_max) {}

  double lambda() const { return lambda_; }

  /// Update λ based on actual vs. predicted reduction.
  /// @return Whether to accept the step.
  bool update(double actual_reduction, double predicted_reduction) {
    if (predicted_reduction <= 0.0) {
      lambda_ = std::min(lambda_ * 10.0, lambda_max_);
      return false;
    }
    const double rho = actual_reduction / predicted_reduction;
    if (rho >= 0.75) {
      lambda_ = std::max(lambda_ / 3.0, lambda_min_);
      return true;
    } else if (rho >= 0.1) {
      // Accept without adjusting λ.
      return true;
    } else {
      lambda_ = std::min(lambda_ * 10.0, lambda_max_);
      return false;
    }
  }

  void reset(double lambda0) { lambda_ = lambda0; }

 private:
  double lambda_;
  double lambda_min_;
  double lambda_max_;
};

/// Adaptive PTC-SER pseudo-timestep controller.
///
/// Implements the Self-Equalising Residual (SER) update with exponent γ:
///   δt_{k+1} = clamp( δt_k · (r_{k-1}/r_k)^γ , δt_min, δt_max )
///
/// As δt → ∞ the modified Newton system (J + I/δt)·Δx = -F approaches
/// the pure Newton step.
class PtcSerController {
 public:
  explicit PtcSerController(double dt0 = 0.1,
                             double dt_min = 1e-4,
                             double dt_max = 1e6,
                             double gamma = 0.7)
      : dt_(dt0), dt_min_(dt_min), dt_max_(dt_max), gamma_(gamma) {}

  double dt() const { return dt_; }

  /// Update pseudo-timestep given two consecutive residual norms.
  void update(double resid_prev, double resid_curr) {
    if (resid_curr > 1e-30) {
      const double ratio = resid_prev / resid_curr;
      dt_ *= std::pow(ratio, gamma_);
    } else {
      dt_ *= 2.0;  // Pure Newton territory; grow freely.
    }
    dt_ = std::clamp(dt_, dt_min_, dt_max_);
  }

  void reset(double dt0) { dt_ = dt0; }

 private:
  double dt_;
  double dt_min_;
  double dt_max_;
  double gamma_;
};

}  // namespace hacdcpf::powerflow

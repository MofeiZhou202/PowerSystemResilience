#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/power_flow/jacobian_builder.hpp"
#include "hacdcpf/power_flow/robust_nonlinear_options.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

/// Precomputed row/column scale vectors for the Newton system.
///
/// The scaled system is:
///   J_hat * dx_hat = -F_hat
/// where:
///   F_hat    = D_F  * F          (residual_scale .* residual)
///   J_hat    = D_F  * J * D_x⁻¹ (row × col scaled Jacobian)
///   dx_hat   = D_x  * dx         (scaled Newton step)
///   dx       = D_x⁻¹ * dx_hat    (physical Newton step, via unscale_step)
///
/// With default scaling disabled the vectors are ones and all operations are
/// identity.
class NonlinearScaling {
 public:
  /// Construct a no-op scaling (all scale factors = 1).
  explicit NonlinearScaling() = default;

  /// Construct from pre-built scale vectors.
  ///
  /// @param res_scale  Per-row (equation) scale factors D_F (element-wise
  ///                   multiply applied to residual/Jacobian rows).
  /// @param var_scale  Per-column (variable) scale factors D_x.  The Jacobian
  ///                   columns are divided by D_x; the Newton step is also
  ///                   divided by D_x to recover the physical increment.
  NonlinearScaling(Eigen::VectorXd res_scale, Eigen::VectorXd var_scale)
      : residual_scale_(std::move(res_scale)),
        variable_scale_(std::move(var_scale)) {}

  /// Apply equation scaling: F_hat = D_F * F.
  Eigen::VectorXd apply_residual_scaling(const Eigen::VectorXd& residual) const {
    if (residual_scale_.size() == 0) return residual;
    return residual_scale_.cwiseProduct(residual);
  }

  /// Apply full J_hat = D_F * J * D_x⁻¹ row/column scaling.
  ///
  /// Sparse row scaling is done by iterating non-zeros; column scaling divides
  /// each non-zero by the matching variable scale factor.
  Eigen::SparseMatrix<double> apply_jacobian_scaling(
      const Eigen::SparseMatrix<double>& jacobian) const {
    if (residual_scale_.size() == 0 && variable_scale_.size() == 0) return jacobian;
    Eigen::SparseMatrix<double> result = jacobian;
    const bool has_row = (residual_scale_.size() == jacobian.rows());
    const bool has_col = (variable_scale_.size() == jacobian.cols());
    for (int col = 0; col < result.outerSize(); ++col) {
      const double col_inv = has_col ? (1.0 / variable_scale_[col]) : 1.0;
      for (Eigen::SparseMatrix<double>::InnerIterator it(result, col); it; ++it) {
        const double row_scale = has_row ? residual_scale_[it.row()] : 1.0;
        it.valueRef() *= row_scale * col_inv;
      }
    }
    return result;
  }

  /// Recover physical Newton step: dx = D_x⁻¹ * dx_hat.
  Eigen::VectorXd unscale_step(const Eigen::VectorXd& step_scaled) const {
    if (variable_scale_.size() == 0) return step_scaled;
    return step_scaled.cwiseQuotient(variable_scale_);
  }

  // Accessors for diagnostics.
  const Eigen::VectorXd& residual_scale() const { return residual_scale_; }
  const Eigen::VectorXd& variable_scale() const { return variable_scale_; }

 private:
  Eigen::VectorXd residual_scale_;  ///< D_F vector (empty = no row scaling)
  Eigen::VectorXd variable_scale_;  ///< D_x vector (empty = no col scaling)
};

/// Build NonlinearScaling from solver state.
///
/// Residual (row) scale factors:
///   - P-equation i:   1 / max(|p_spec[non_slack[i]]|, 1)
///   - Q-equation i:   1 / max(|q_spec[pq[i]]|,       1)
///   - Dc-equation i:  1 / max(|pdc_spec[dc_ns[i]]|,  1)
///
/// Variable (column) scale factors:
///   - θ variables (cols 0..np-1):       π  (normalise angle to [-1,1])
///   - V variables (cols np..np+nq-1):   vm[pq[i]]   (nominal voltage)
///   - Vdc variables:                    vdc[dc_ns[i]] (nominal DC voltage)
///
/// All scale factors are clamped to [options.min_scale, options.max_scale].
NonlinearScaling build_nonlinear_scaling(
    const JacobianContext& ctx,
    const SolverData& /*data*/,
    const Eigen::VectorXd& p_spec,
    const Eigen::VectorXd& q_spec,
    const Eigen::VectorXd& pdc_spec,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& vdc,
    const RobustNonlinearOptions& options);

/// Refine the nominal scale factors using the numerical Jacobian. The
/// transformed linear system is algebraically equivalent to the original one;
/// the recovered physical Newton step and nonlinear equations are unchanged.
NonlinearScaling equilibrate_nonlinear_scaling(
    const NonlinearScaling& nominal,
    const Eigen::SparseMatrix<double>& jacobian,
    const RobustNonlinearOptions& options);

}  // namespace hacdcpf::powerflow

#include "hacdcpf/power_flow/nonlinear_scaling.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::powerflow {

namespace {

/// Return the D_F scale element: 1 / clamp(raw, lo, hi).
inline double clamp_inv_scale(double raw, double lo, double hi) {
  const double c = std::clamp(raw, lo, hi);
  return (c > 0.0) ? (1.0 / c) : 1.0;
}

/// Return the D_x scale element: clamp(raw, lo, hi).
inline double clamp_var_scale(double raw, double lo, double hi) {
  return std::clamp(raw, lo, hi);
}

}  // namespace

NonlinearScaling build_nonlinear_scaling(
    const JacobianContext& ctx,
    const SolverData& /*data*/,
    const Eigen::VectorXd& p_spec,
    const Eigen::VectorXd& q_spec,
    const Eigen::VectorXd& pdc_spec,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& vdc,
    const RobustNonlinearOptions& options) {

  const int nvar = ctx.np + ctx.nq + ctx.ndc_eq;
  if (nvar == 0) return NonlinearScaling{};

  const double lo = options.min_scale;
  const double hi = options.max_scale;
  constexpr double kPi = 3.14159265358979323846;

  // ── Residual (row) scale vector D_F ──────────────────────────────
  // D_F[i] = 1 / max(|F_i^spec|, 1)  so that D_F * F ~ O(1).
  Eigen::VectorXd res_scale = Eigen::VectorXd::Ones(nvar);

  if (options.enable_residual_scaling) {
    // P-equations (rows 0..np-1).
    for (int k = 0; k < ctx.np; ++k) {
      const int bus = ctx.non_slack[static_cast<size_t>(k)];
      const double raw = (bus < p_spec.size())
                             ? std::max(std::abs(p_spec[bus]), 1.0)
                             : 1.0;
      res_scale[k] = clamp_inv_scale(raw, lo, hi);
    }
    // Q-equations (rows np..np+nq-1).
    for (int k = 0; k < ctx.nq; ++k) {
      const int bus = ctx.pq[static_cast<size_t>(k)];
      const double raw = (bus < q_spec.size())
                             ? std::max(std::abs(q_spec[bus]), 1.0)
                             : 1.0;
      res_scale[ctx.np + k] = clamp_inv_scale(raw, lo, hi);
    }
    // DC-equations (rows np+nq..nvar-1).
    for (int k = 0; k < ctx.ndc_eq; ++k) {
      const int bus = ctx.dc_non_slack[static_cast<size_t>(k)];
      const double raw = (bus < pdc_spec.size())
                             ? std::max(std::abs(pdc_spec[bus]), 1.0)
                             : 1.0;
      res_scale[ctx.np + ctx.nq + k] = clamp_inv_scale(raw, lo, hi);
    }
  }

  // ── Variable (column) scale vector D_x ───────────────────────────
  // D_x[j] = nominal magnitude of variable j so that dx/D_x ~ O(1).
  Eigen::VectorXd var_scale = Eigen::VectorXd::Ones(nvar);

  if (options.enable_variable_scaling) {
    // θ (phase angles): nominal range ≈ ±π  → normalise by π.
    for (int k = 0; k < ctx.np; ++k) {
      var_scale[k] = clamp_var_scale(kPi, lo, hi);
    }
    // V (AC voltage magnitudes): scale by current vm (typically ~1 pu).
    for (int k = 0; k < ctx.nq; ++k) {
      const int bus = ctx.pq[static_cast<size_t>(k)];
      const double v_nom = (bus < vm.size() && vm[bus] > 1e-3) ? vm[bus] : 1.0;
      var_scale[ctx.np + k] = clamp_var_scale(v_nom, lo, hi);
    }
    // Vdc (DC voltages): scale by current vdc.
    for (int k = 0; k < ctx.ndc_eq; ++k) {
      const int bus = ctx.dc_non_slack[static_cast<size_t>(k)];
      const double vdc_nom = (bus < vdc.size() && vdc[bus] > 1e-3) ? vdc[bus] : 1.0;
      var_scale[ctx.np + ctx.nq + k] = clamp_var_scale(vdc_nom, lo, hi);
    }
  }

  return NonlinearScaling(std::move(res_scale), std::move(var_scale));
}

NonlinearScaling equilibrate_nonlinear_scaling(
    const NonlinearScaling& nominal,
    const Eigen::SparseMatrix<double>& jacobian,
    const RobustNonlinearOptions& options) {
  if (jacobian.rows() == 0 || jacobian.cols() == 0) return nominal;

  Eigen::VectorXd row_scale = nominal.residual_scale();
  Eigen::VectorXd variable_scale = nominal.variable_scale();
  if (row_scale.size() != jacobian.rows()) {
    row_scale = Eigen::VectorXd::Ones(jacobian.rows());
  }
  if (variable_scale.size() != jacobian.cols()) {
    variable_scale = Eigen::VectorXd::Ones(jacobian.cols());
  }

  Eigen::SparseMatrix<double> balanced =
      nominal.apply_jacobian_scaling(jacobian);
  constexpr int kPasses = 4;
  constexpr double kTiny = 1e-30;
  const double scale_min = std::max(options.min_scale, kTiny);
  const double scale_max = std::max(options.max_scale, scale_min);

  for (int pass = 0; pass < kPasses; ++pass) {
    Eigen::VectorXd row_max = Eigen::VectorXd::Zero(balanced.rows());
    for (int col = 0; col < balanced.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(balanced, col); it;
           ++it) {
        row_max[it.row()] =
            std::max(row_max[it.row()], std::abs(it.value()));
      }
    }
    Eigen::VectorXd row_factor = Eigen::VectorXd::Ones(balanced.rows());
    for (int row = 0; row < row_max.size(); ++row) {
      if (row_max[row] <= kTiny) continue;
      const double requested = 1.0 / std::sqrt(row_max[row]);
      const double updated =
          std::clamp(row_scale[row] * requested, scale_min, scale_max);
      row_factor[row] = updated / row_scale[row];
      row_scale[row] = updated;
    }
    for (int col = 0; col < balanced.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(balanced, col); it;
           ++it) {
        it.valueRef() *= row_factor[it.row()];
      }
    }

    Eigen::VectorXd col_max = Eigen::VectorXd::Zero(balanced.cols());
    for (int col = 0; col < balanced.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(balanced, col); it;
           ++it) {
        col_max[col] = std::max(col_max[col], std::abs(it.value()));
      }
    }
    for (int col = 0; col < col_max.size(); ++col) {
      if (col_max[col] <= kTiny) continue;
      const double requested = 1.0 / std::sqrt(col_max[col]);
      const double updated = std::clamp(variable_scale[col] / requested,
                                        scale_min, scale_max);
      const double effective = variable_scale[col] / updated;
      variable_scale[col] = updated;
      for (Eigen::SparseMatrix<double>::InnerIterator it(balanced, col); it;
           ++it) {
        it.valueRef() *= effective;
      }
    }
  }

  return NonlinearScaling(std::move(row_scale),
                          std::move(variable_scale));
}

}  // namespace hacdcpf::powerflow


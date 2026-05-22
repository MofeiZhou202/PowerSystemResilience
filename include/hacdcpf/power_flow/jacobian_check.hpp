#pragma once

/// Jacobian Finite-Difference Consistency Check
/// ==============================================
/// Verifies that an analytic Jacobian function J(x) matches the
/// finite-difference approximation ∂F/∂x at a given point x₀.
///
/// Typical usage in a unit test:
///
///   SolverData data = make_solver_data(sys, LossModelType::Linear);
///   auto residual_fn = [&](const Eigen::VectorXd& x) {
///       return evaluate_residual(data, x);
///   };
///   auto jacobian_fn = [&](const Eigen::VectorXd& x) {
///       return build_jacobian(data, x);
///   };
///
///   auto report = hacdcpf::check_jacobian(residual_fn, jacobian_fn, x0);
///   CHECK(report.max_relative_error < 1e-5);

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf {

// ── Report ────────────────────────────────────────────────────────────────────

struct JacobianCheckReport {
    bool   passed{false};           ///< true if max_relative_error < tolerance
    double tolerance{1e-5};         ///< threshold used for this check
    double max_absolute_error{0.0}; ///< max |J_analytic - J_fd|
    double max_relative_error{0.0}; ///< max |J_analytic - J_fd| / (|J_analytic| + eps)

    /// Location of the worst mismatch.
    int worst_row{-1};
    int worst_col{-1};
    double analytic_value{0.0};
    double fd_value{0.0};

    std::vector<std::string> warnings; ///< additional diagnostic messages
};

// ── Core check function ───────────────────────────────────────────────────────

using ResidualFn = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
using AnalyticJacFn =
    std::function<Eigen::SparseMatrix<double>(const Eigen::VectorXd&)>;

/// Compare the analytic Jacobian against a central-difference approximation.
///
/// @param residual   Function F: ℝⁿ → ℝᵐ
/// @param jacobian   Analytic J: ℝⁿ → ℝᵐˣⁿ  (returned as sparse)
/// @param x0         Evaluation point
/// @param h          Finite-difference step size (default 1e-6)
/// @param tolerance  Pass/fail threshold on max relative error (default 1e-5)
[[nodiscard]] JacobianCheckReport check_jacobian(
    const ResidualFn&       residual,
    const AnalyticJacFn&    jacobian,
    const Eigen::VectorXd&  x0,
    double h         = 1e-6,
    double tolerance = 1e-5);

// ── Dense overload ────────────────────────────────────────────────────────────

using DenseJacFn =
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;

[[nodiscard]] JacobianCheckReport check_jacobian_dense(
    const ResidualFn&       residual,
    const DenseJacFn&       jacobian,
    const Eigen::VectorXd&  x0,
    double h         = 1e-6,
    double tolerance = 1e-5);

}  // namespace hacdcpf

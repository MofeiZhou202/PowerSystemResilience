// src/power_flow/jacobian_check.cpp
//
// Central-difference Jacobian consistency check.

#include "hacdcpf/power_flow/jacobian_check.hpp"

#include <cmath>
#include <limits>

namespace hacdcpf {

// ── Sparse overload ──────────────────────────────────────────────────────────

JacobianCheckReport check_jacobian(
    const ResidualFn&       residual,
    const AnalyticJacFn&    jacobian,
    const Eigen::VectorXd&  x0,
    double h,
    double tolerance)
{
    const int n = static_cast<int>(x0.size());
    const Eigen::VectorXd F0 = residual(x0);
    const int m = static_cast<int>(F0.size());
    const Eigen::SparseMatrix<double> J_analytic = jacobian(x0);

    JacobianCheckReport report;
    report.tolerance = tolerance;

    constexpr double eps = std::numeric_limits<double>::epsilon();

    for (int j = 0; j < n; ++j) {
        Eigen::VectorXd xp = x0; xp[j] += h;
        Eigen::VectorXd xm = x0; xm[j] -= h;
        Eigen::VectorXd fd_col = (residual(xp) - residual(xm)) / (2.0 * h);

        for (int i = 0; i < m; ++i) {
            const double a = J_analytic.coeff(i, j);
            const double f = fd_col[i];
            const double abs_err = std::fabs(a - f);
            const double rel_err = abs_err / (std::fabs(a) + eps);

            if (abs_err > report.max_absolute_error) {
                report.max_absolute_error = abs_err;
            }
            if (rel_err > report.max_relative_error) {
                report.max_relative_error = rel_err;
                report.worst_row = i;
                report.worst_col = j;
                report.analytic_value = a;
                report.fd_value = f;
            }
        }
    }

    report.passed = (report.max_relative_error <= tolerance);
    return report;
}

// ── Dense overload ───────────────────────────────────────────────────────────

JacobianCheckReport check_jacobian_dense(
    const ResidualFn&       residual,
    const DenseJacFn&       jacobian,
    const Eigen::VectorXd&  x0,
    double h,
    double tolerance)
{
    // Wrap the dense Jacobian as sparse and reuse the sparse implementation.
    auto sparse_wrapper = [&](const Eigen::VectorXd& x) -> Eigen::SparseMatrix<double> {
        return jacobian(x).sparseView();
    };
    return check_jacobian(residual, sparse_wrapper, x0, h, tolerance);
}

}  // namespace hacdcpf

/// @file bc_instance_features.cpp
/// @brief Definitions for B&C instance feature extraction.

#include "mipsolvers/engine/detail/bc_instance_features.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace mipsolvers::engine::detail {
namespace {

double finite_nonzero_range(const Eigen::VectorXd& values) {
  double smallest = std::numeric_limits<double>::infinity();
  double largest = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    const double value = std::abs(values[i]);
    if (!std::isfinite(value) || value == 0.0) continue;
    smallest = std::min(smallest, value);
    largest = std::max(largest, value);
  }
  return std::isfinite(smallest) ? largest / smallest : 0.0;
}

double combined_matrix_range(const Eigen::SparseMatrix<double>& first,
                             const Eigen::SparseMatrix<double>& second) {
  double smallest = std::numeric_limits<double>::infinity();
  double largest = 0.0;
  auto scan = [&](const Eigen::SparseMatrix<double>& matrix) {
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it;
           ++it) {
        const double value = std::abs(it.value());
        if (!std::isfinite(value) || value == 0.0) continue;
        smallest = std::min(smallest, value);
        largest = std::max(largest, value);
      }
    }
  };
  scan(first);
  scan(second);
  return std::isfinite(smallest) ? largest / smallest : 0.0;
}

double median_row_nnz(const Eigen::SparseMatrix<double>& first,
                      const Eigen::SparseMatrix<double>& second) {
  const int first_rows = static_cast<int>(first.rows());
  const int rows = first_rows + static_cast<int>(second.rows());
  if (rows == 0) return 0.0;
  std::vector<int> counts(static_cast<std::size_t>(rows), 0);
  auto scan = [&](const Eigen::SparseMatrix<double>& matrix, int offset) {
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it;
           ++it) {
        if (it.value() != 0.0) {
          ++counts[static_cast<std::size_t>(offset + it.row())];
        }
      }
    }
  };
  scan(first, 0);
  scan(second, first_rows);
  std::sort(counts.begin(), counts.end());
  const std::size_t middle = counts.size() / 2;
  if (counts.size() % 2 != 0) return counts[middle];
  return 0.5 * static_cast<double>(counts[middle - 1] + counts[middle]);
}

double lp_rhs_range(const LPModel& lp) {
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(lp.b.size() + lp.beq.size() +
                                          lp.row_lhs.size()));
  for (int i = 0; i < lp.b.size(); ++i) values.push_back(lp.b[i]);
  for (int i = 0; i < lp.beq.size(); ++i) values.push_back(lp.beq[i]);
  for (int i = 0; i < lp.row_lhs.size(); ++i) {
    if (std::isfinite(lp.row_lhs[i])) values.push_back(lp.row_lhs[i]);
  }
  if (values.empty()) return 0.0;
  Eigen::Map<const Eigen::VectorXd> mapped(values.data(), values.size());
  return finite_nonzero_range(mapped);
}

}  // namespace

BCInstanceFeatures compute_instance_features(const MIPModel& prob) {
  const LPModel& lp = prob.linear_part;
  BCInstanceFeatures f;
  f.n_vars = static_cast<int>(lp.c.size());
  f.n_rows = static_cast<int>(lp.A.rows()) + static_cast<int>(lp.Aeq.rows());
  f.n_eq   = static_cast<int>(lp.Aeq.rows());
  f.n_nnz  = static_cast<int>(lp.A.nonZeros() + lp.Aeq.nonZeros());
  f.n_bin  = static_cast<int>(prob.binary_idx.size());
  f.n_int  = static_cast<int>(prob.integer_idx.size());
  if (prob.uc_hint) {
    const auto& hint = *prob.uc_hint;
    f.num_binary_blocks = hint.ng * hint.T;
    f.extra = {1.0,
               static_cast<double>(hint.ng),
               static_cast<double>(hint.T),
               static_cast<double>(hint.n_segments),
               static_cast<double>(hint.n_storage),
               static_cast<double>(hint.network_line_count)};
  }
  if (f.n_vars > 0) {
    int cnnz = 0;
    for (int i = 0; i < lp.c.size(); ++i) if (lp.c[i] != 0.0) ++cnnz;
    f.obj_density = static_cast<double>(cnnz) / static_cast<double>(f.n_vars);
  }
  f.row_density_median = median_row_nnz(lp.A, lp.Aeq);
  f.obj_range_ratio = finite_nonzero_range(lp.c);
  f.rhs_range_ratio = lp_rhs_range(lp);
  f.coeff_range_ratio = combined_matrix_range(lp.A, lp.Aeq);
  return f;
}

BCInstanceFeatures compute_instance_features(const MINLPModel& prob) {
  const NLPModel& nlp = prob.nonlinear_part;
  BCInstanceFeatures f;
  f.n_vars = static_cast<int>(nlp.vars.size());
  f.n_bin = static_cast<int>(prob.binary_idx.size());
  f.n_int = static_cast<int>(prob.integer_idx.size());

  Eigen::VectorXd x = nlp.x0;
  if (x.size() != f.n_vars) x = Eigen::VectorXd::Zero(f.n_vars);
  Eigen::VectorXd inequalities;
  Eigen::VectorXd equalities;
  if (nlp.g) nlp.g(x, inequalities);
  if (nlp.h) nlp.h(x, equalities);
  f.n_rows = static_cast<int>(inequalities.size() + equalities.size());
  f.n_eq = static_cast<int>(equalities.size());

  Eigen::SparseMatrix<double> jac_g(inequalities.size(), f.n_vars);
  Eigen::SparseMatrix<double> jac_h(equalities.size(), f.n_vars);
  if (nlp.jac_g) nlp.jac_g(x, jac_g);
  if (nlp.jac_h) nlp.jac_h(x, jac_h);
  f.n_nnz = static_cast<int>(jac_g.nonZeros() + jac_h.nonZeros());
  f.row_density_median = median_row_nnz(jac_g, jac_h);
  f.coeff_range_ratio = combined_matrix_range(jac_g, jac_h);

  if (nlp.grad && f.n_vars > 0) {
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(f.n_vars);
    nlp.grad(x, gradient);
    if (gradient.size() == f.n_vars) {
      int nonzeros = 0;
      for (int i = 0; i < gradient.size(); ++i) {
        if (gradient[i] != 0.0) ++nonzeros;
      }
      f.obj_density = static_cast<double>(nonzeros) / f.n_vars;
      f.obj_range_ratio = finite_nonzero_range(gradient);
    }
  }
  return f;
}

}  // namespace mipsolvers::engine::detail

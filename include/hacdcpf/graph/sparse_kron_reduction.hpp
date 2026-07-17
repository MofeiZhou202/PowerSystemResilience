#pragma once

#include <complex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::graph {

using SparseComplexMatrix =
    Eigen::SparseMatrix<std::complex<double>, Eigen::ColMajor, int>;

struct SparseKronOptions {
  int max_front{12};
  double max_nnz_ratio{2.0};
  double pivot_tolerance{1e-12};
  double drop_tolerance{1e-13};
};

struct SparseKronRecoveryStep {
  int node{-1};
  std::vector<std::pair<int, std::complex<double>>> coefficients;
};

struct SparseKronResult {
  SparseComplexMatrix reduced;
  std::vector<int> retained;
  std::vector<SparseKronRecoveryStep> recovery_steps;
  int original_size{0};
  int original_nonzeros{0};
  int max_front{0};
  double elapsed_ms{0.0};
  std::string error;

  [[nodiscard]] bool valid() const {
    return error.empty() && original_size >= 0 &&
           reduced.rows() == static_cast<int>(retained.size());
  }
};

/// Sequential sparse Kron elimination with a minimum-front ordering.
/// Only nodes marked true in `eligible` may be removed. Rejected candidates
/// remain retained when their pivot, front, or predicted fill violates a cap.
SparseKronResult reduce_sparse_kron(
    const SparseComplexMatrix& ybus,
    const std::vector<bool>& eligible,
    const SparseKronOptions& options = {});

/// Apply the reverse recovery tape to a retained complex state.
Eigen::VectorXcd recover_sparse_kron_state(
    const SparseKronResult& reduction,
    const Eigen::VectorXcd& retained_state);

/// Materialize the linear recovery operator T such that x_full = T*x_retained.
/// Intended for small diagnostics; the reverse tape is preferable at scale.
Eigen::SparseMatrix<std::complex<double>> sparse_kron_recovery_operator(
    const SparseKronResult& reduction,
    double drop_tolerance = 1e-14);

}  // namespace hacdcpf::graph

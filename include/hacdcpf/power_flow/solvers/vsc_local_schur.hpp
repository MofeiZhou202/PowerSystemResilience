#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"
#include "hacdcpf/power_flow/assembly/jacobian_builder.hpp"

namespace hacdcpf::powerflow {

/// Numerical certificate for one exact local-VSC Schur-complement solve.
///
/// The certificate concerns the generalized-Jacobian element assembled at the
/// current iterate. It is not a proof that every element of the Clarke
/// generalized Jacobian is nonsingular.
struct VSCLocalSchurReport {
  bool attempted{false};
  bool accepted{false};
  bool numeric_refactor_attempted{false};
  bool numeric_refactor_accepted{false};
  int local_factorization_attempts{0};
  int sparse_factorization_attempts{0};
  int sparse_solve_attempts{0};
  int local_blocks{0};
  int full_dimension{0};
  int reduced_dimension{0};
  std::int64_t full_structural_nnz{0};
  std::int64_t reduced_structural_nnz{0};
  std::int64_t reduced_factor_nonzeros{-1};
  std::int64_t reduced_factor_work{-1};
  double minimum_local_rcond{0.0};
  double reduced_backward_error{0.0};
  double full_backward_error{0.0};
  std::string status{"not_attempted"};
};

/// Exact block Gaussian elimination for fixed six-state VSC limit blocks.
///
/// With network variables x and converter-local variables z, the assembled
/// Newton matrix has the form
///
///   J = [ A  B ],   D = blkdiag(D_1, ..., D_m),  D_i in R^{6x6}.
///       [ C  D ]
///
/// The solver factors S = A - B D^{-1} C, solves the reduced network system,
/// and recovers every local step by back substitution. The reduced sparse
/// pattern is analyzed once and retained across Newton iterations.
class VSCLocalSchurSolver {
 public:
  VSCLocalSchurSolver() = default;
  VSCLocalSchurSolver(const VSCLocalSchurSolver&) = delete;
  VSCLocalSchurSolver& operator=(const VSCLocalSchurSolver&) = delete;
  VSCLocalSchurSolver(VSCLocalSchurSolver&&) noexcept = default;
  VSCLocalSchurSolver& operator=(VSCLocalSchurSolver&&) noexcept = default;

  bool initialize(const JacobianPattern& full_pattern,
                  const JacobianContext& context);

  bool solve(const Eigen::SparseMatrix<double>& full_jacobian,
             const Eigen::VectorXd& rhs,
             bool enable_numeric_refactor,
             double local_rcond_tolerance,
             double backward_error_tolerance,
             Eigen::VectorXd& step,
             VSCLocalSchurReport& report);

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] int reduced_dimension() const noexcept { return network_nvar_; }
  [[nodiscard]] std::int64_t reduced_structural_nnz() const noexcept {
    return reduced_matrix_.nonZeros();
  }

 private:
  struct BlockPattern {
    int offset{0};
    std::array<int, 3> network_rows{};
    std::array<int, 3> network_cols{};
    std::array<int, 36> local_nz{};
    std::array<int, 6> va_nz{};
    std::array<int, 6> vm_nz{};
    std::array<int, 6> vdc_nz{};
    std::array<int, 3> b_nz{};
    std::array<int, 9> schur_nz{};
  };

  int full_nvar_{0};
  int network_nvar_{0};
  bool initialized_{false};
  bool has_numeric_factorization_{false};
  Eigen::SparseMatrix<double> reduced_matrix_;
  std::vector<int> full_a_to_reduced_nz_;
  std::vector<BlockPattern> blocks_;
  std::unique_ptr<hacdcpf::engine::SparseLinearSolver> solver_;
};

}  // namespace hacdcpf::powerflow

#include "hacdcpf/power_flow/solvers/vsc_local_schur.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

#include <Eigen/LU>

#include "hacdcpf/power_flow/vsc_limit_ncp.hpp"

namespace hacdcpf::powerflow {
namespace {

using Matrix6 = Eigen::Matrix<double, kVSCLimitStateSize,
                              kVSCLimitStateSize>;
using Matrix63 = Eigen::Matrix<double, kVSCLimitStateSize, 3>;
using Vector6 = Eigen::Matrix<double, kVSCLimitStateSize, 1>;

std::uint64_t coordinate_key(int row, int col) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32U) |
         static_cast<std::uint32_t>(col);
}

double normwise_backward_error(const Eigen::SparseMatrix<double>& matrix,
                               const Eigen::VectorXd& rhs,
                               const Eigen::VectorXd& solution) {
  if (!solution.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  Eigen::VectorXd row_sum = Eigen::VectorXd::Zero(matrix.rows());
  for (int col = 0; col < matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, col); it; ++it) {
      row_sum[it.row()] += std::abs(it.value());
    }
  }
  const double matrix_inf =
      row_sum.size() == 0 ? 0.0 : row_sum.maxCoeff();
  const double rhs_inf =
      rhs.size() == 0 ? 0.0 : rhs.cwiseAbs().maxCoeff();
  const double solution_inf =
      solution.size() == 0 ? 0.0 : solution.cwiseAbs().maxCoeff();
  const double residual_inf =
      rhs.size() == 0
          ? 0.0
          : (rhs - matrix * solution).cwiseAbs().maxCoeff();
  // Higham (2002), Accuracy and Stability of Numerical Algorithms, sec. 7.1.
  const double denominator = matrix_inf * solution_inf + rhs_inf;
  return denominator > 0.0 ? residual_inf / denominator : residual_inf;
}

}  // namespace

bool VSCLocalSchurSolver::initialize(const JacobianPattern& full_pattern,
                                     const JacobianContext& context) {
  initialized_ = false;
  has_numeric_factorization_ = false;
  solver_.reset();
  blocks_.clear();
  reduced_matrix_.resize(0, 0);
  full_a_to_reduced_nz_.clear();

  if (context.vsc_limit_blocks.empty() || context.network_nvar <= 0 ||
      context.nvar <= context.network_nvar ||
      full_pattern.matrix.rows() != context.nvar ||
      full_pattern.matrix.cols() != context.nvar ||
      full_pattern.vsc_limit_entries.size() !=
          context.vsc_limit_blocks.size()) {
    return false;
  }

  full_nvar_ = context.nvar;
  network_nvar_ = context.network_nvar;
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(full_pattern.matrix.nonZeros()));
  for (int col = 0; col < full_pattern.matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(full_pattern.matrix, col);
         it; ++it) {
      if (it.row() < network_nvar_ && it.col() < network_nvar_) {
        triplets.emplace_back(it.row(), it.col(), 1.0);
      }
    }
  }

  blocks_.reserve(context.vsc_limit_blocks.size());
  for (size_t block_index = 0;
       block_index < context.vsc_limit_blocks.size(); ++block_index) {
    const auto& context_block = context.vsc_limit_blocks[block_index];
    const auto& entry = full_pattern.vsc_limit_entries[block_index];
    BlockPattern block;
    block.offset = context_block.offset;
    block.network_rows = {
        context.p_row[static_cast<size_t>(context_block.ac_bus)],
        context.q_row[static_cast<size_t>(context_block.ac_bus)],
        context.dc_row[static_cast<size_t>(context_block.dc_bus)]};
    block.network_cols = {
        context.va_col[static_cast<size_t>(context_block.ac_bus)],
        context.vm_col[static_cast<size_t>(context_block.ac_bus)],
        context.vdc_col[static_cast<size_t>(context_block.dc_bus)]};
    if (std::any_of(block.network_rows.begin(), block.network_rows.end(),
                    [this](int index) {
                      return index < 0 || index >= network_nvar_;
                    }) ||
        std::any_of(block.network_cols.begin(), block.network_cols.end(),
                    [this](int index) {
                      return index < 0 || index >= network_nvar_;
                    })) {
      return false;
    }
    block.local_nz = entry.local_nz;
    block.va_nz = entry.va_nz;
    block.vm_nz = entry.vm_nz;
    block.vdc_nz = entry.vdc_nz;
    block.b_nz = {entry.p_network_nz, entry.q_network_nz,
                  entry.dc_network_nz};
    if (std::any_of(block.local_nz.begin(), block.local_nz.end(),
                    [](int nz) { return nz < 0; }) ||
        std::any_of(block.b_nz.begin(), block.b_nz.end(),
                    [](int nz) { return nz < 0; }) ||
        std::any_of(block.va_nz.begin(), block.va_nz.end(),
                    [](int nz) { return nz < 0; }) ||
        std::any_of(block.vm_nz.begin(), block.vm_nz.end(),
                    [](int nz) { return nz < 0; }) ||
        std::any_of(block.vdc_nz.begin(), block.vdc_nz.end(),
                    [](int nz) { return nz < 0; })) {
      return false;
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        triplets.emplace_back(block.network_rows[static_cast<size_t>(row)],
                              block.network_cols[static_cast<size_t>(col)],
                              1.0);
      }
    }
    blocks_.push_back(block);
  }

  reduced_matrix_.resize(network_nvar_, network_nvar_);
  reduced_matrix_.setFromTriplets(triplets.begin(), triplets.end());
  reduced_matrix_.makeCompressed();

  std::unordered_map<std::uint64_t, int> reduced_coordinate_to_nz;
  reduced_coordinate_to_nz.reserve(
      static_cast<size_t>(reduced_matrix_.nonZeros()));
  const int* reduced_outer = reduced_matrix_.outerIndexPtr();
  const int* reduced_inner = reduced_matrix_.innerIndexPtr();
  for (int col = 0; col < reduced_matrix_.outerSize(); ++col) {
    for (int nz = reduced_outer[col]; nz < reduced_outer[col + 1]; ++nz) {
      reduced_coordinate_to_nz.emplace(
          coordinate_key(reduced_inner[nz], col), nz);
    }
  }

  full_a_to_reduced_nz_.assign(
      static_cast<size_t>(full_pattern.matrix.nonZeros()), -1);
  const int* full_outer = full_pattern.matrix.outerIndexPtr();
  const int* full_inner = full_pattern.matrix.innerIndexPtr();
  for (int col = 0; col < full_pattern.matrix.outerSize(); ++col) {
    for (int nz = full_outer[col]; nz < full_outer[col + 1]; ++nz) {
      const int row = full_inner[nz];
      if (row >= network_nvar_ || col >= network_nvar_) {
        continue;
      }
      const auto found = reduced_coordinate_to_nz.find(
          coordinate_key(row, col));
      if (found == reduced_coordinate_to_nz.end()) {
        return false;
      }
      full_a_to_reduced_nz_[static_cast<size_t>(nz)] = found->second;
    }
  }
  for (auto& block : blocks_) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const auto found = reduced_coordinate_to_nz.find(coordinate_key(
            block.network_rows[static_cast<size_t>(row)],
            block.network_cols[static_cast<size_t>(col)]));
        if (found == reduced_coordinate_to_nz.end()) {
          return false;
        }
        block.schur_nz[static_cast<size_t>(3 * row + col)] = found->second;
      }
    }
  }

  solver_ = hacdcpf::engine::make_default_sparse_solver();
  if (!solver_) {
    return false;
  }
  solver_->analyze_pattern(reduced_matrix_);
  initialized_ = true;
  return true;
}

bool VSCLocalSchurSolver::solve(
    const Eigen::SparseMatrix<double>& full_jacobian,
    const Eigen::VectorXd& rhs,
    bool enable_numeric_refactor,
    double local_rcond_tolerance,
    double backward_error_tolerance,
    Eigen::VectorXd& step,
    VSCLocalSchurReport& report) {
  report = VSCLocalSchurReport{};
  report.attempted = true;
  report.local_blocks = static_cast<int>(blocks_.size());
  report.full_dimension = full_nvar_;
  report.reduced_dimension = network_nvar_;
  report.full_structural_nnz = full_jacobian.nonZeros();
  report.reduced_structural_nnz = reduced_matrix_.nonZeros();
  report.minimum_local_rcond = std::numeric_limits<double>::infinity();

  if (!initialized_ || !solver_ || full_jacobian.rows() != full_nvar_ ||
      full_jacobian.cols() != full_nvar_ || rhs.size() != full_nvar_) {
    report.status = "invalid_layout";
    return false;
  }

  std::fill(reduced_matrix_.valuePtr(),
            reduced_matrix_.valuePtr() + reduced_matrix_.nonZeros(), 0.0);
  const double* full_values = full_jacobian.valuePtr();
  for (size_t full_nz = 0; full_nz < full_a_to_reduced_nz_.size(); ++full_nz) {
    const int reduced_nz = full_a_to_reduced_nz_[full_nz];
    if (reduced_nz >= 0) {
      reduced_matrix_.valuePtr()[reduced_nz] = full_values[full_nz];
    }
  }

  struct BlockNumeric {
    Matrix63 d_inverse_c{Matrix63::Zero()};
    Vector6 d_inverse_rhs{Vector6::Zero()};
  };
  std::vector<BlockNumeric> numeric(blocks_.size());
  Eigen::VectorXd reduced_rhs = rhs.head(network_nvar_);
  const double accepted_rcond = std::max(0.0, local_rcond_tolerance);

  // Golub & Van Loan (2013), Matrix Computations, 4th ed., block Gaussian
  // elimination. D^{-1} is never formed: each 6x6 factor solves [C_i, r_i].
  for (size_t block_index = 0; block_index < blocks_.size(); ++block_index) {
    const auto& block = blocks_[block_index];
    Matrix6 d;
    for (int row = 0; row < kVSCLimitStateSize; ++row) {
      for (int col = 0; col < kVSCLimitStateSize; ++col) {
        d(row, col) = full_values[block.local_nz[static_cast<size_t>(
            row * kVSCLimitStateSize + col)]];
      }
    }
    Matrix63 c;
    for (int row = 0; row < kVSCLimitStateSize; ++row) {
      c(row, 0) = full_values[block.va_nz[static_cast<size_t>(row)]];
      c(row, 1) = full_values[block.vm_nz[static_cast<size_t>(row)]];
      c(row, 2) = full_values[block.vdc_nz[static_cast<size_t>(row)]];
    }
    Eigen::Matrix<double, 3, kVSCLimitStateSize> b =
        Eigen::Matrix<double, 3, kVSCLimitStateSize>::Zero();
    b(0, 0) = full_values[block.b_nz[0]];
    b(1, 1) = full_values[block.b_nz[1]];
    b(2, 2) = full_values[block.b_nz[2]];

    report.local_factorization_attempts += 1;
    Eigen::FullPivLU<Matrix6> local_lu(d);
    const double rcond = local_lu.rcond();
    report.minimum_local_rcond =
        std::min(report.minimum_local_rcond, rcond);
    if (!d.allFinite() || !std::isfinite(rcond) || rcond <= accepted_rcond) {
      report.status = "local_block_not_regular";
      return false;
    }

    Eigen::Matrix<double, kVSCLimitStateSize, 4> combined_rhs;
    combined_rhs.leftCols<3>() = c;
    combined_rhs.col(3) = rhs.segment<kVSCLimitStateSize>(block.offset);
    const auto combined_solution = local_lu.solve(combined_rhs);
    if (!combined_solution.allFinite()) {
      report.status = "local_block_solve_failed";
      return false;
    }
    numeric[block_index].d_inverse_c = combined_solution.leftCols<3>();
    numeric[block_index].d_inverse_rhs = combined_solution.col(3);

    const Eigen::Matrix3d update = b * numeric[block_index].d_inverse_c;
    const Eigen::Vector3d rhs_update = b * numeric[block_index].d_inverse_rhs;
    for (int row = 0; row < 3; ++row) {
      reduced_rhs[block.network_rows[static_cast<size_t>(row)]] -=
          rhs_update[row];
      for (int col = 0; col < 3; ++col) {
        reduced_matrix_.valuePtr()[block.schur_nz[static_cast<size_t>(
            3 * row + col)]] -= update(row, col);
      }
    }
  }

  const double accepted_backward_error =
      std::max(0.0, backward_error_tolerance);
  Eigen::VectorXd network_step;
  bool reduced_solved = false;
  if (enable_numeric_refactor && has_numeric_factorization_ &&
      solver_->supports_numeric_refactor()) {
    report.numeric_refactor_attempted = true;
    report.sparse_factorization_attempts += 1;
    if (solver_->refactorize(reduced_matrix_)) {
      report.sparse_solve_attempts += 1;
      reduced_solved = solver_->solve(reduced_rhs, network_step) &&
                       network_step.allFinite();
      report.reduced_backward_error =
          reduced_solved
              ? normwise_backward_error(reduced_matrix_, reduced_rhs,
                                        network_step)
              : std::numeric_limits<double>::infinity();
      if (reduced_solved &&
          report.reduced_backward_error <= accepted_backward_error) {
        report.numeric_refactor_accepted = true;
      } else {
        reduced_solved = false;
      }
    }
  }
  if (!reduced_solved) {
    report.sparse_factorization_attempts += 1;
    if (!solver_->factorize(reduced_matrix_)) {
      has_numeric_factorization_ = false;
      report.status = "reduced_factorization_failed";
      return false;
    }
    has_numeric_factorization_ = true;
    report.sparse_solve_attempts += 1;
    reduced_solved = solver_->solve(reduced_rhs, network_step) &&
                     network_step.allFinite();
    report.reduced_backward_error =
        reduced_solved
            ? normwise_backward_error(reduced_matrix_, reduced_rhs,
                                      network_step)
            : std::numeric_limits<double>::infinity();
    if (!reduced_solved ||
        report.reduced_backward_error > accepted_backward_error) {
      report.status = "reduced_backward_error_failed";
      return false;
    }
  }

  step = Eigen::VectorXd::Zero(full_nvar_);
  step.head(network_nvar_) = network_step;
  for (size_t block_index = 0; block_index < blocks_.size(); ++block_index) {
    const auto& block = blocks_[block_index];
    Eigen::Vector3d terminal_step;
    for (int col = 0; col < 3; ++col) {
      terminal_step[col] =
          network_step[block.network_cols[static_cast<size_t>(col)]];
    }
    step.segment<kVSCLimitStateSize>(block.offset) =
        numeric[block_index].d_inverse_rhs -
        numeric[block_index].d_inverse_c * terminal_step;
  }

  report.full_backward_error =
      normwise_backward_error(full_jacobian, rhs, step);
  report.reduced_factor_nonzeros = solver_->factor_nonzeros();
  report.reduced_factor_work = solver_->factor_work();
  if (!step.allFinite() ||
      report.full_backward_error > accepted_backward_error) {
    report.status = "full_backward_error_failed";
    return false;
  }

  report.accepted = true;
  report.status = report.numeric_refactor_accepted
                      ? "accepted_numeric_refactor"
                      : "accepted_fresh_factorization";
  return true;
}

}  // namespace hacdcpf::powerflow

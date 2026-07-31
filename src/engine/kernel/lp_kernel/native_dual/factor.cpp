#include "factor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <utility>

namespace mipsolvers::engine::native_dual::detail {
namespace {

constexpr double kBackwardErrorMultiplier = 256.0;

long double gamma_n(std::size_t operations) {
  const long double product =
      static_cast<long double>(operations) *
      std::numeric_limits<long double>::epsilon();
  return product < 0.5L ? product / (1.0L - product) : 1.0L;
}

}  // namespace

std::vector<int> logical_columns(const StandardFormLP& sf) {
  std::vector<int> logical(static_cast<std::size_t>(sf.A.rows()), -1);
  for (int row = 0; row < sf.A.rows(); ++row) {
    if (row < static_cast<int>(sf.row_to_slack_col.size()) &&
        sf.row_to_slack_col[static_cast<std::size_t>(row)] >= 0) {
      logical[static_cast<std::size_t>(row)] =
          sf.row_to_slack_col[static_cast<std::size_t>(row)];
    } else if (row < static_cast<int>(sf.row_to_artificial_col.size()) &&
               sf.row_to_artificial_col[static_cast<std::size_t>(row)] >= 0) {
      logical[static_cast<std::size_t>(row)] =
          sf.row_to_artificial_col[static_cast<std::size_t>(row)];
    }
  }
  return logical;
}

BasisFactor::BasisFactor(const StandardFormLP& sf,
                         std::vector<int> logical_by_row)
    : A_(&sf.A), logical_by_row_(std::move(logical_by_row)) {}

bool BasisFactor::rebuild(std::vector<int>& basis, int& rank_repairs,
                          std::string& failure) {
  if (A_ == nullptr || static_cast<int>(basis.size()) != A_->rows()) {
    failure = "basis dimension does not match A";
    return false;
  }
  std::vector<char> seen(static_cast<std::size_t>(A_->cols()), 0);
  for (int col : basis) {
    if (col < 0 || col >= A_->cols()) {
      failure = "basis contains an out-of-range column";
      return false;
    }
    if (seen[static_cast<std::size_t>(col)]) {
      failure = "basis contains a duplicate column";
      return false;
    }
    seen[static_cast<std::size_t>(col)] = 1;
  }

  std::vector<int> repaired;
  const std::vector<int> requested_basis = basis;
  if (!rank_factor_.factorize_with_logicals(
          *A_, basis.data(), static_cast<int>(basis.size()), logical_by_row_,
          repaired)) {
    failure = "rank-revealing factorization could not repair the basis";
    return false;
  }
  if (rank_factor_.rank_deficiency == 0 && repaired != requested_basis) {
    failure = "rank factor changed basis order without repairing a column";
    return false;
  }
  rank_repairs += rank_factor_.rank_deficiency;
  basis = std::move(repaired);
  basis_ = basis;

  rebuild_norms();
  ++generation_;
  ++rebuild_count_;
  return true;
}

bool BasisFactor::update(int pivot_row, int entering_col,
                         const Eigen::VectorXd& direction,
                         const Eigen::VectorXd& row_ep,
                         std::string& failure) {
  if (A_ == nullptr || pivot_row < 0 || pivot_row >= A_->rows() ||
      entering_col < 0 || entering_col >= A_->cols() ||
      direction.size() != A_->rows() || row_ep.size() != A_->rows() ||
      !direction.allFinite() || !row_ep.allFinite()) {
    failure = "basis update dimensions are invalid";
    return false;
  }
  const int leaving_col = basis_[static_cast<std::size_t>(pivot_row)];
  if (!rank_factor_.update(pivot_row, entering_col, direction.data(),
                           row_ep.data())) {
    failure = "Forrest-Tomlin basis update rejected a non-invertible exchange";
    return false;
  }
  for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, leaving_col); it;
       ++it) {
    row_abs_sum_[static_cast<std::size_t>(it.row())] -= std::abs(it.value());
  }
  double entering_col_sum = 0.0;
  for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, entering_col); it;
       ++it) {
    const double magnitude = std::abs(it.value());
    row_abs_sum_[static_cast<std::size_t>(it.row())] += magnitude;
    entering_col_sum += magnitude;
  }
  basis_[static_cast<std::size_t>(pivot_row)] = entering_col;
  col_abs_sum_[static_cast<std::size_t>(pivot_row)] = entering_col_sum;
  norm_B_inf_ = row_abs_sum_.empty()
                    ? 0.0
                    : *std::max_element(row_abs_sum_.begin(), row_abs_sum_.end());
  norm_Bt_inf_ = col_abs_sum_.empty()
                     ? 0.0
                     : *std::max_element(col_abs_sum_.begin(), col_abs_sum_.end());
  ++generation_;
  return true;
}

bool BasisFactor::update_indexed(int pivot_row, int entering_col,
                                 std::string& failure) {
  if (A_ == nullptr || pivot_row < 0 || pivot_row >= A_->rows() ||
      entering_col < 0 || entering_col >= A_->cols() ||
      !rank_factor_.update_captured(pivot_row, entering_col)) {
    failure = "Forrest-Tomlin basis update rejected the packed exchange";
    return false;
  }
  basis_[static_cast<std::size_t>(pivot_row)] = entering_col;
  ++generation_;
  return true;
}

PivotEvidence BasisFactor::pivot_evidence(
    int pivot_row, int entering_col, const Eigen::VectorXd& direction,
    const Eigen::VectorXd& row_ep) const {
  PivotEvidence evidence;
  if (A_ == nullptr || pivot_row < 0 || pivot_row >= A_->rows() ||
      entering_col < 0 || entering_col >= A_->cols() ||
      direction.size() != A_->rows() || row_ep.size() != A_->rows()) {
    return evidence;
  }

  std::vector<long double> column_residual(
      static_cast<std::size_t>(A_->rows()), 0.0L);
  for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, entering_col); it;
       ++it) {
    column_residual[static_cast<std::size_t>(it.row())] =
        static_cast<long double>(it.value());
  }
  for (int position = 0; position < static_cast<int>(basis_.size());
       ++position) {
    const long double value =
        static_cast<long double>(direction[position]);
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             *A_, basis_[static_cast<std::size_t>(position)]);
         it; ++it) {
      column_residual[static_cast<std::size_t>(it.row())] -=
          static_cast<long double>(it.value()) * value;
    }
  }
  for (long double residual : column_residual) {
    evidence.column_residual_inf =
        std::max(evidence.column_residual_inf, std::abs(residual));
  }

  long double row_pivot = 0.0L;
  long double residual_identity_bound = 0.0L;
  for (Eigen::SparseMatrix<double>::InnerIterator it(*A_, entering_col); it;
       ++it) {
    row_pivot += static_cast<long double>(row_ep[it.row()]) *
                 static_cast<long double>(it.value());
  }
  for (int row = 0; row < A_->rows(); ++row) {
    residual_identity_bound +=
        std::abs(static_cast<long double>(row_ep[row]) *
                 column_residual[static_cast<std::size_t>(row)]);
  }
  for (int position = 0; position < static_cast<int>(basis_.size());
       ++position) {
    long double transpose_residual = position == pivot_row ? 1.0L : 0.0L;
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             *A_, basis_[static_cast<std::size_t>(position)]);
         it; ++it) {
      transpose_residual -=
          static_cast<long double>(it.value()) *
          static_cast<long double>(row_ep[it.row()]);
    }
    residual_identity_bound +=
        std::abs(transpose_residual *
                 static_cast<long double>(direction[position]));
    evidence.row_residual_inf =
        std::max(evidence.row_residual_inf, std::abs(transpose_residual));
  }

  const long double column_pivot =
      static_cast<long double>(direction[pivot_row]);
  const long double arithmetic_guard =
      64.0L * std::numeric_limits<long double>::epsilon() *
      std::max({1.0L, std::abs(row_pivot), std::abs(column_pivot),
                residual_identity_bound});
  evidence.row_pivot = row_pivot;
  evidence.column_pivot = column_pivot;
  evidence.discrepancy = row_pivot - column_pivot;
  evidence.residual_envelope = residual_identity_bound;
  evidence.arithmetic_guard = arithmetic_guard;
  evidence.accepted =
      std::abs(evidence.discrepancy) <=
          evidence.residual_envelope + evidence.arithmetic_guard &&
      column_pivot != 0.0L;
  return evidence;
}

bool BasisFactor::row_solve_consistent(
    int pivot_row, const Eigen::VectorXd& rhs,
    const Eigen::VectorXd& solution, const Eigen::VectorXd& row_ep,
    long double& discrepancy, long double& residual_envelope) const {
  discrepancy = std::numeric_limits<long double>::infinity();
  residual_envelope = 0.0L;
  if (A_ == nullptr || pivot_row < 0 || pivot_row >= A_->rows() ||
      rhs.size() != A_->rows() || solution.size() != A_->rows() ||
      row_ep.size() != A_->rows() || !rhs.allFinite() ||
      !solution.allFinite() || !row_ep.allFinite()) {
    return false;
  }

  std::vector<long double> primal_residual(static_cast<std::size_t>(A_->rows()));
  std::vector<long double> primal_absolute_sum(
      static_cast<std::size_t>(A_->rows()));
  std::vector<std::size_t> primal_terms(static_cast<std::size_t>(A_->rows()), 1);
  for (int row = 0; row < A_->rows(); ++row) {
    primal_residual[static_cast<std::size_t>(row)] =
        static_cast<long double>(rhs[row]);
    primal_absolute_sum[static_cast<std::size_t>(row)] =
        std::abs(static_cast<long double>(rhs[row]));
  }
  for (int position = 0; position < static_cast<int>(basis_.size());
       ++position) {
    const long double value = static_cast<long double>(solution[position]);
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             *A_, basis_[static_cast<std::size_t>(position)]);
         it; ++it) {
      primal_residual[static_cast<std::size_t>(it.row())] -=
          static_cast<long double>(it.value()) * value;
      primal_absolute_sum[static_cast<std::size_t>(it.row())] +=
          std::abs(static_cast<long double>(it.value()) * value);
      primal_terms[static_cast<std::size_t>(it.row())] += 2;
    }
  }

  long double projected_rhs = 0.0L;
  long double projected_absolute_sum = 0.0L;
  long double primal_dot_absolute_sum = 0.0L;
  for (int row = 0; row < A_->rows(); ++row) {
    projected_rhs += static_cast<long double>(row_ep[row]) *
                     static_cast<long double>(rhs[row]);
    projected_absolute_sum +=
        std::abs(static_cast<long double>(row_ep[row]) *
                 static_cast<long double>(rhs[row]));
    const long double residual_bound =
        std::abs(primal_residual[static_cast<std::size_t>(row)]) +
        gamma_n(primal_terms[static_cast<std::size_t>(row)]) *
            primal_absolute_sum[static_cast<std::size_t>(row)];
    residual_envelope +=
        std::abs(static_cast<long double>(row_ep[row])) * residual_bound;
    primal_dot_absolute_sum +=
        std::abs(static_cast<long double>(row_ep[row]) *
                 primal_residual[static_cast<std::size_t>(row)]);
  }
  long double adjoint_dot_absolute_sum = 0.0L;
  for (int position = 0; position < static_cast<int>(basis_.size());
       ++position) {
    long double residual = position == pivot_row ? 1.0L : 0.0L;
    long double absolute_sum = position == pivot_row ? 1.0L : 0.0L;
    std::size_t terms = 1;
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             *A_, basis_[static_cast<std::size_t>(position)]);
         it; ++it) {
      residual -= static_cast<long double>(it.value()) *
                  static_cast<long double>(row_ep[it.row()]);
      absolute_sum +=
          std::abs(static_cast<long double>(it.value()) *
                   static_cast<long double>(row_ep[it.row()]));
      terms += 2;
    }
    const long double residual_bound =
        std::abs(residual) + gamma_n(terms) * absolute_sum;
    residual_envelope +=
        std::abs(static_cast<long double>(solution[position])) * residual_bound;
    adjoint_dot_absolute_sum +=
        std::abs(residual * static_cast<long double>(solution[position]));
  }

  discrepancy = projected_rhs - static_cast<long double>(solution[pivot_row]);
  residual_envelope +=
      gamma_n(2 * static_cast<std::size_t>(A_->rows())) *
      (projected_absolute_sum + primal_dot_absolute_sum +
       adjoint_dot_absolute_sum);
  residual_envelope +=
      gamma_n(1) *
      (std::abs(projected_rhs) +
       std::abs(static_cast<long double>(solution[pivot_row])));
  return std::abs(discrepancy) <= residual_envelope;
}

double BasisFactor::residual_norm(const Eigen::VectorXd& rhs,
                                  const Eigen::VectorXd& solution,
                                  bool transpose) const {
  // Guard against an inconsistent basis/matrix binding (e.g. after rebind_A to a
  // differently-shaped matrix): indexing residual[pos]/solution[pos] or a stale
  // basis_ column into A_ would read out of bounds.  Returning a large residual
  // makes backward_error_acceptable reject the solve instead of crashing.
  if (A_ == nullptr || rhs.size() != A_->rows() ||
      solution.size() != A_->rows() ||
      static_cast<int>(basis_.size()) != A_->rows()) {
    return std::numeric_limits<double>::infinity();
  }
  for (const int col : basis_) {
    if (col < 0 || col >= A_->cols()) {
      return std::numeric_limits<double>::infinity();
    }
  }
  Eigen::VectorXd residual = rhs;
  if (transpose) {
    for (int pos = 0; pos < static_cast<int>(basis_.size()); ++pos) {
      double product = 0.0;
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               *A_, basis_[static_cast<std::size_t>(pos)]);
           it; ++it) {
        product += it.value() * solution[it.row()];
      }
      residual[pos] -= product;
    }
  } else {
    for (int pos = 0; pos < static_cast<int>(basis_.size()); ++pos) {
      const double value = solution[pos];
      if (value == 0.0) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               *A_, basis_[static_cast<std::size_t>(pos)]);
           it; ++it) {
        residual[it.row()] -= it.value() * value;
      }
    }
  }
  return residual.lpNorm<Eigen::Infinity>();
}

Eigen::VectorXd BasisFactor::residual_vector(
    const Eigen::VectorXd& rhs, const Eigen::VectorXd& solution,
    bool transpose) const {
  if (A_ == nullptr || rhs.size() != A_->rows() ||
      solution.size() != A_->rows() ||
      static_cast<int>(basis_.size()) != A_->rows()) {
    return {};
  }
  for (const int col : basis_) {
    if (col < 0 || col >= A_->cols()) {
      return {};
    }
  }
  Eigen::VectorXd residual(rhs.size());
  if (transpose) {
    for (int position = 0; position < static_cast<int>(basis_.size());
         ++position) {
      long double value = static_cast<long double>(rhs[position]);
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               *A_, basis_[static_cast<std::size_t>(position)]);
           it; ++it) {
        value -= static_cast<long double>(it.value()) *
                 static_cast<long double>(solution[it.row()]);
      }
      residual[position] = static_cast<double>(value);
    }
  } else {
    std::vector<long double> values(static_cast<std::size_t>(rhs.size()));
    for (int row = 0; row < rhs.size(); ++row) {
      values[static_cast<std::size_t>(row)] =
          static_cast<long double>(rhs[row]);
    }
    for (int position = 0; position < static_cast<int>(basis_.size());
         ++position) {
      const long double x = static_cast<long double>(solution[position]);
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               *A_, basis_[static_cast<std::size_t>(position)]);
           it; ++it) {
        values[static_cast<std::size_t>(it.row())] -=
            static_cast<long double>(it.value()) * x;
      }
    }
    for (int row = 0; row < rhs.size(); ++row) {
      residual[row] = static_cast<double>(values[static_cast<std::size_t>(row)]);
    }
  }
  return residual;
}

void BasisFactor::rebuild_norms() {
  if (A_ == nullptr) return;
  row_abs_sum_.assign(static_cast<std::size_t>(A_->rows()), 0.0);
  col_abs_sum_.assign(basis_.size(), 0.0);
  for (int pos = 0; pos < static_cast<int>(basis_.size()); ++pos) {
    double col_sum = 0.0;
    for (Eigen::SparseMatrix<double>::InnerIterator it(
             *A_, basis_[static_cast<std::size_t>(pos)]);
         it; ++it) {
      const double magnitude = std::abs(it.value());
      row_abs_sum_[static_cast<std::size_t>(it.row())] += magnitude;
      col_sum += magnitude;
    }
    col_abs_sum_[static_cast<std::size_t>(pos)] = col_sum;
  }
  norm_B_inf_ = row_abs_sum_.empty()
                    ? 0.0
                    : *std::max_element(row_abs_sum_.begin(), row_abs_sum_.end());
  norm_Bt_inf_ = col_abs_sum_.empty()
                     ? 0.0
                     : *std::max_element(col_abs_sum_.begin(), col_abs_sum_.end());
}

SolveEvidence BasisFactor::solve_checked(const Eigen::VectorXd& rhs,
                                         bool transpose,
                                         bool capture_update,
                                         bool verify,
                                         const std::vector<int>* rhs_pattern) const {
  SolveEvidence evidence;
  if (A_ == nullptr || rhs.size() != A_->rows() || !rhs.allFinite()) {
    return evidence;
  }
  auto solve_current_factor = [&]() {
    Eigen::VectorXd solution(rhs.size());
    std::vector<int> pattern;
    if (transpose) {
      if (capture_update) {
        rank_factor_.btran_for_update(rhs.data(), solution.data(), rhs_pattern,
                                      &pattern);
      } else {
        rank_factor_.btran(rhs.data(), solution.data(), rhs_pattern, &pattern);
      }
    } else {
      if (capture_update) {
        rank_factor_.ftran_for_update(rhs.data(), solution.data(), rhs_pattern,
                                      &pattern);
      } else {
        rank_factor_.ftran(rhs.data(), solution.data(), rhs_pattern, &pattern);
      }
    }
    return std::make_pair(std::move(solution), std::move(pattern));
  };
  if (!rank_factor_.valid) return evidence;
  last_solve_transpose_ = transpose;
  last_solve_refined_ = false;
  auto [solution, pattern] = solve_current_factor();
  if (solution.allFinite()) {
    if (!verify) {
      evidence.solution = std::move(solution);
      evidence.pattern = std::move(pattern);
      evidence.pattern_known = true;
      evidence.accepted = true;
      return evidence;
    }
    if (backward_error_acceptable(rhs, solution, transpose)) {
      evidence.solution = std::move(solution);
      evidence.pattern = std::move(pattern);
      evidence.pattern_known = true;
      evidence.accepted = true;
      evidence.residual = last_residual_;
      evidence.error_limit = last_error_limit_;
      return evidence;
    }
  }

  return refine_checked(rhs, solution, transpose);
}

SolveEvidence BasisFactor::refine_checked(const Eigen::VectorXd& rhs,
                                          const Eigen::VectorXd& initial,
                                          bool transpose) const {
  SolveEvidence evidence;
  evidence.needs_rebuild = true;
  evidence.refined = true;
  last_solve_transpose_ = transpose;
  last_solve_refined_ = true;
  if (A_ == nullptr || rhs.size() != A_->rows() ||
      initial.size() != A_->rows() || !rhs.allFinite() ||
      !initial.allFinite() || !rank_factor_.valid) {
    return evidence;
  }

  // Refinement reuses the current factor and never changes the basis
  // representation. If this is a pivotal solve, the driver must not use the
  // raw captured FT pack: it commits the exchange and INVERTs the new basis.
  Eigen::VectorXd solution = initial;
  const Eigen::VectorXd residual = residual_vector(rhs, solution, transpose);
  if (residual.size() != rhs.size() || !residual.allFinite()) return evidence;
  Eigen::VectorXd correction(rhs.size());
  if (transpose) {
    rank_factor_.btran(residual.data(), correction.data());
  } else {
    rank_factor_.ftran(residual.data(), correction.data());
  }
  if (!correction.allFinite()) return evidence;
  solution += correction;
  if (!solution.allFinite() ||
      !backward_error_acceptable(rhs, solution, transpose)) {
    evidence.residual = last_residual_;
    evidence.error_limit = last_error_limit_;
    return evidence;
  }
  evidence.solution = std::move(solution);
  evidence.pattern.reserve(static_cast<std::size_t>(evidence.solution.size()));
  for (int row = 0; row < evidence.solution.size(); ++row) {
    if (evidence.solution[row] != 0.0) evidence.pattern.push_back(row);
  }
  evidence.pattern_known = true;
  evidence.accepted = true;
  evidence.needs_rebuild = false;
  evidence.residual = last_residual_;
  evidence.error_limit = last_error_limit_;
  return evidence;
}

bool BasisFactor::backward_error_acceptable(
    const Eigen::VectorXd& rhs, const Eigen::VectorXd& solution,
    bool transpose) const {
  const double matrix_norm = transpose ? norm_Bt_inf_ : norm_B_inf_;
  const double scale = matrix_norm * solution.lpNorm<Eigen::Infinity>() +
                       rhs.lpNorm<Eigen::Infinity>();
  const double limit =
      kBackwardErrorMultiplier * std::numeric_limits<double>::epsilon() *
      std::max(1.0, scale);
  const double residual = residual_norm(rhs, solution, transpose);
  last_residual_ = residual;
  last_error_limit_ = limit;
  last_matrix_norm_ = matrix_norm;
  last_solution_norm_ = solution.lpNorm<Eigen::Infinity>();
  last_rhs_norm_ = rhs.lpNorm<Eigen::Infinity>();
  return residual <= limit;
}

std::string BasisFactor::last_solve_diagnostics() const {
  std::ostringstream message;
  message << std::setprecision(17)
          << "solve=" << (last_solve_transpose_ ? "BTRAN" : "FTRAN")
          << ", residual_inf=" << last_residual_
          << ", limit=" << last_error_limit_
          << ", norm_B=" << last_matrix_norm_
          << ", norm_solution_inf=" << last_solution_norm_
          << ", norm_rhs_inf=" << last_rhs_norm_
          << ", ft_updates=" << rank_factor_.n_updates
          << ", refined=" << (last_solve_refined_ ? 1 : 0)
          << ", generation=" << generation_;
  return message.str();
}

Eigen::VectorXd BasisFactor::ftran(const Eigen::VectorXd& rhs) const {
  return checked_ftran(rhs).solution;
}

Eigen::VectorXd BasisFactor::btran(const Eigen::VectorXd& rhs) const {
  return checked_btran(rhs).solution;
}

Eigen::VectorXd BasisFactor::ftran_for_update(
    const Eigen::VectorXd& rhs) const {
  return checked_ftran(rhs, true).solution;
}

Eigen::VectorXd BasisFactor::btran_for_update(
    const Eigen::VectorXd& rhs) const {
  return checked_btran(rhs, true).solution;
}

SolveEvidence BasisFactor::checked_ftran(const Eigen::VectorXd& rhs,
                                         bool capture_update,
                                         bool verify,
                                         const std::vector<int>* rhs_pattern) const {
  return solve_checked(rhs, false, capture_update, verify, rhs_pattern);
}

SolveEvidence BasisFactor::checked_btran(const Eigen::VectorXd& rhs,
                                         bool capture_update,
                                         bool verify,
                                         const std::vector<int>* rhs_pattern) const {
  return solve_checked(rhs, true, capture_update, verify, rhs_pattern);
}

IndexedSolveEvidence BasisFactor::indexed_ftran(const IndexedVector& rhs,
                                                bool capture_update) const {
  IndexedSolveEvidence evidence;
  evidence.solution.clear(rhs.dimension);
  if (A_ == nullptr || rhs.dimension != A_->rows() || !rhs.finite() ||
      !rank_factor_.valid) {
    return evidence;
  }
  // The backend rejects non-finite results during export, so an accepted
  // solve is already known finite.
  evidence.accepted = rank_factor_.ftran_indexed(
      rhs.index, rhs.value, evidence.solution.index, evidence.solution.value,
      evidence.solution.lookup_slot, capture_update);
  return evidence;
}

IndexedSolveEvidence BasisFactor::indexed_btran(const IndexedVector& rhs,
                                                bool capture_update) const {
  IndexedSolveEvidence evidence;
  evidence.solution.clear(rhs.dimension);
  if (A_ == nullptr || rhs.dimension != A_->rows() || !rhs.finite() ||
      !rank_factor_.valid) {
    return evidence;
  }
  // The backend rejects non-finite results during export, so an accepted
  // solve is already known finite.
  evidence.accepted = rank_factor_.btran_indexed(
      rhs.index, rhs.value, evidence.solution.index, evidence.solution.value,
      evidence.solution.lookup_slot, capture_update);
  return evidence;
}

SolveEvidence BasisFactor::refine_ftran(
    const Eigen::VectorXd& rhs, const Eigen::VectorXd& solution) const {
  return refine_checked(rhs, solution, false);
}

EdgeWeightEvidence BasisFactor::compute_exact_edge_weights() const {
  EdgeWeightEvidence batch;
  if (A_ == nullptr || !rank_factor_.valid || A_->rows() <= 0) return batch;

  const int order = A_->rows();
  batch.weights.resize(static_cast<std::size_t>(order));
  Eigen::VectorXd unit = Eigen::VectorXd::Zero(order);
  for (int row = 0; row < order; ++row) {
    unit[row] = 1.0;
    SolveEvidence solve = checked_btran(unit);
    unit[row] = 0.0;
    batch.max_residual = std::max(batch.max_residual, solve.residual);
    batch.max_error_limit =
        std::max(batch.max_error_limit, solve.error_limit);
    if (!solve.accepted) {
      batch.needs_rebuild = solve.needs_rebuild;
      batch.weights.clear();
      return batch;
    }
    if (solve.refined) ++batch.refinements;
    const double weight = solve.solution.squaredNorm();
    if (!(weight > 0.0) || !std::isfinite(weight)) {
      batch.weights.clear();
      return batch;
    }
    batch.weights[static_cast<std::size_t>(row)] = weight;
  }
  batch.accepted = true;
  return batch;
}

bool BasisFactor::basis_inverse_row(int row, Eigen::VectorXd& out) const {
  if (A_ == nullptr || row < 0 || row >= A_->rows()) return false;
  Eigen::VectorXd unit = Eigen::VectorXd::Zero(A_->rows());
  unit[row] = 1.0;
  out = btran(unit);
  return out.size() == A_->rows();
}

bool BasisFactor::basis_inverse_row_sparse_entries(
    int row, std::vector<std::pair<int, double>>& out) const {
  Eigen::VectorXd dense;
  if (!basis_inverse_row(row, dense)) return false;
  out.clear();
  for (int i = 0; i < dense.size(); ++i) {
    if (dense[i] != 0.0) out.emplace_back(i, dense[i]);
  }
  return true;
}

bool BasisFactor::tableau_row(int row, Eigen::RowVectorXd& out) const {
  Eigen::VectorXd row_ep;
  if (!basis_inverse_row(row, row_ep) || A_ == nullptr) return false;
  out = row_ep.transpose() * *A_;
  return out.allFinite();
}

SparseFactorTelemetry BasisFactor::factor_telemetry() const {
  SparseFactorTelemetry telemetry;
  telemetry.ft_valid = false;
  telemetry.ft_updates = rank_factor_.n_updates;
  return telemetry;
}

void BasisFactor::rebind_A(const Eigen::SparseMatrix<double>& A) {
  A_ = &A;
  // Only treat the factor as usable if the stored basis is dimensionally
  // compatible with the newly bound matrix.  rebuild() enforces the same
  // invariants (basis.size()==A.rows() and every basis column in range); a
  // rebind to a differently-shaped matrix would otherwise leave basis_ indexing
  // out of range and crash the next solve/residual/norm.  On mismatch, drop the
  // factor's validity so solves return empty evidence (a rejected dual proof)
  // instead of reading out of bounds.
  if (static_cast<int>(basis_.size()) != A.rows()) {
    rank_factor_.valid = false;
    return;
  }
  for (const int col : basis_) {
    if (col < 0 || col >= A.cols()) {
      rank_factor_.valid = false;
      return;
    }
  }
  rebuild_norms();
}

bool BasisFactor::bound_to_A(const Eigen::SparseMatrix<double>& A) const {
  return A_ == &A;
}

}  // namespace mipsolvers::engine::native_dual::detail

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "model.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"

namespace mipsolvers::engine::native_dual::detail {

struct SolveEvidence {
  Eigen::VectorXd solution;
  std::vector<int> pattern;
  bool pattern_known{false};
  bool accepted{false};
  bool needs_rebuild{false};
  bool refined{false};
  double residual{0.0};
  double error_limit{0.0};
};

struct IndexedSolveEvidence {
  IndexedVector solution;
  bool accepted{false};
};

struct EdgeWeightEvidence {
  std::vector<double> weights;
  bool accepted{false};
  bool needs_rebuild{false};
  int refinements{0};
  double max_residual{0.0};
  double max_error_limit{0.0};
};

struct PivotEvidence {
  bool accepted{false};
  long double row_pivot{0.0L};
  long double column_pivot{0.0L};
  long double discrepancy{0.0L};
  long double residual_envelope{0.0L};
  long double arithmetic_guard{0.0L};
  long double column_residual_inf{0.0L};
  long double row_residual_inf{0.0L};
};

class BasisFactor final : public BasisOps {
 public:
  BasisFactor(const StandardFormLP& sf, std::vector<int> logical_by_row);

  bool rebuild(std::vector<int>& basis, int& rank_repairs,
               std::string& failure);
  bool update(int pivot_row, int entering_col,
              const Eigen::VectorXd& direction,
              const Eigen::VectorXd& row_ep, std::string& failure);
  bool update_indexed(int pivot_row, int entering_col, std::string& failure);
  PivotEvidence pivot_evidence(int pivot_row, int entering_col,
                               const Eigen::VectorXd& direction,
                               const Eigen::VectorXd& row_ep) const;
  bool row_solve_consistent(int pivot_row, const Eigen::VectorXd& rhs,
                            const Eigen::VectorXd& solution,
                            const Eigen::VectorXd& row_ep,
                            long double& discrepancy,
                            long double& residual_envelope) const;

  BasisOpsKind kind() const override { return BasisOpsKind::NativeSparse; }
  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override;
  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override;
  Eigen::VectorXd ftran_for_update(const Eigen::VectorXd& rhs) const;
  Eigen::VectorXd btran_for_update(const Eigen::VectorXd& rhs) const;
  SolveEvidence checked_ftran(const Eigen::VectorXd& rhs,
                              bool capture_update = false,
                              bool verify = true,
                              const std::vector<int>* rhs_pattern = nullptr) const;
  SolveEvidence checked_btran(const Eigen::VectorXd& rhs,
                              bool capture_update = false,
                              bool verify = true,
                              const std::vector<int>* rhs_pattern = nullptr) const;
  IndexedSolveEvidence indexed_ftran(const IndexedVector& rhs,
                                     bool capture_update = false) const;
  IndexedSolveEvidence indexed_btran(const IndexedVector& rhs,
                                     bool capture_update = false) const;
  SolveEvidence refine_ftran(const Eigen::VectorXd& rhs,
                             const Eigen::VectorXd& solution) const;
  EdgeWeightEvidence compute_exact_edge_weights() const;
  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override;
  bool basis_inverse_row_sparse_entries(
      int row, std::vector<std::pair<int, double>>& out) const override;
  bool tableau_row(int row, Eigen::RowVectorXd& out) const override;
  int generation() const override { return generation_; }
  SparseFactorTelemetry factor_telemetry() const override;
  void rebind_A(const Eigen::SparseMatrix<double>& A) override;
  bool bound_to_A(const Eigen::SparseMatrix<double>& A) const override;

  const std::vector<int>& basis() const { return basis_; }
  std::string last_solve_diagnostics() const;
  bool needs_rebuild() const { return rank_factor_.needs_refactorise(); }
  int update_count() const { return rank_factor_.n_updates; }
  bool last_solve_refined() const { return last_solve_refined_; }
  int rebuild_count() const { return rebuild_count_; }

 private:
  SolveEvidence solve_checked(const Eigen::VectorXd& rhs,
                              bool transpose, bool capture_update,
                              bool verify = true,
                              const std::vector<int>* rhs_pattern = nullptr) const;
  SolveEvidence refine_checked(const Eigen::VectorXd& rhs,
                               const Eigen::VectorXd& solution,
                               bool transpose) const;
  bool backward_error_acceptable(const Eigen::VectorXd& rhs,
                                 const Eigen::VectorXd& solution,
                                 bool transpose) const;
  double residual_norm(const Eigen::VectorXd& rhs,
                       const Eigen::VectorXd& solution,
                       bool transpose) const;
  Eigen::VectorXd residual_vector(const Eigen::VectorXd& rhs,
                                  const Eigen::VectorXd& solution,
                                  bool transpose) const;
  void rebuild_norms();

  const Eigen::SparseMatrix<double>* A_{nullptr};
  std::vector<int> logical_by_row_;
  std::vector<int> basis_;
  std::vector<double> row_abs_sum_;
  std::vector<double> col_abs_sum_;
  mutable HFactorBackend rank_factor_;
  double norm_B_inf_{0.0};
  double norm_Bt_inf_{0.0};
  mutable int generation_{0};
  int rebuild_count_{0};
  mutable double last_residual_{0.0};
  mutable double last_error_limit_{0.0};
  mutable double last_matrix_norm_{0.0};
  mutable double last_solution_norm_{0.0};
  mutable double last_rhs_norm_{0.0};
  mutable bool last_solve_transpose_{false};
  mutable bool last_solve_refined_{false};
};

std::vector<int> logical_columns(const StandardFormLP& sf);

}  // namespace mipsolvers::engine::native_dual::detail

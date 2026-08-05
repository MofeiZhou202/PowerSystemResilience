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

class BasisFactor final : public BasisOps {
 public:
  BasisFactor(const StandardFormLP& sf, std::vector<int> logical_by_row);

  bool rebuild(std::vector<int>& basis, int& rank_repairs,
               std::string& failure);
  bool update(int pivot_row, int entering_col,
              const Eigen::VectorXd& direction,
              const Eigen::VectorXd& row_ep, std::string& failure);
  bool update_indexed(int pivot_row, int entering_col, std::string& failure);

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
  // Same FTRAN as indexed_ftran but writes the packed result into a
  // caller-owned vector, reusing its capacity across pivots. Numerically
  // identical to indexed_ftran (same ftran_indexed call); it only removes the
  // per-pivot allocation of the result vectors. Returns whether the solve was
  // accepted.
  bool indexed_ftran_into(const IndexedVector& rhs, IndexedVector& out,
                          bool capture_update = false) const;
  bool indexed_ftran_at_captured_pattern(
      const IndexedVector& rhs, std::vector<double>& result_value) const;
  // O(1) read of the entering-column FTRAN (col_aq) at a single row, from the
  // factor-resident update_vec_aq backing. Bit-identical to indexed_ftran's
  // packed value at that row (the pivotal `column_pivot`); false if the capture
  // is stale. Avoids building a lookup over the packed image for one element.
  bool captured_aq_value(int external_row, double& out) const;
  IndexedSolveEvidence indexed_btran(const IndexedVector& rhs,
                                     bool capture_update = false) const;
  EdgeWeightEvidence compute_exact_edge_weights() const;
  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override;
  bool basis_inverse_row_sparse_entries(
      int row, std::vector<std::pair<int, double>>& out) const override;
  bool tableau_row(int row, Eigen::RowVectorXd& out) const override;
  int generation() const override { return generation_; }
  SparseFactorTelemetry factor_telemetry() const override;
  void rebind_A(const StandardColumnMatrix& A) override;
  bool bound_to_A(const StandardColumnMatrix& A) const override;

  const std::vector<int>& basis() const { return basis_; }
  std::string last_solve_diagnostics() const;
  bool needs_rebuild() const { return rank_factor_.needs_refactorise(); }
  int update_count() const { return rank_factor_.n_updates; }
  bool last_solve_refined() const { return last_solve_refined_; }
  int rebuild_count() const { return rebuild_count_; }
  void set_indexed_solve_profiling(bool enabled) {
    rank_factor_.set_indexed_solve_profiling(enabled);
  }
  double profiled_indexed_solve_time_sec() const {
    return rank_factor_.profiled_indexed_solve_time_sec();
  }
  double profiled_indexed_export_time_sec() const {
    return rank_factor_.profiled_indexed_export_time_sec();
  }
  double profiled_indexed_export_btran_time_sec() const {
    return rank_factor_.profiled_indexed_export_btran_time_sec();
  }
  double profiled_indexed_solve_synthetic_tick() const {
    return rank_factor_.profiled_indexed_solve_synthetic_tick();
  }
  std::uint64_t profiled_indexed_solve_count() const {
    return rank_factor_.profiled_indexed_solve_count();
  }
  double build_synthetic_tick() const {
    return rank_factor_.build_synthetic_tick();
  }

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
  void update_norms_after_exchange(int pivot_row, int leaving_col,
                                   int entering_col);

  const StandardColumnMatrix* A_{nullptr};
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

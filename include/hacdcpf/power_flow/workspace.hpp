#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::powerflow {

struct SolverWorkspace {
  int nac{0};
  int ndc{0};
  int np{0};
  int nq{0};
  int ndc_eq{0};
  int nf{0};

  std::vector<int> pq_idx;
  std::vector<int> pv_idx;
  std::vector<int> slack_idx;
  std::vector<int> non_slack_idx;
  std::vector<int> dc_non_slack_idx;

  Eigen::VectorXd vm;
  Eigen::VectorXd va;
  Eigen::VectorXd vdc;

  Eigen::VectorXd residual;
  Eigen::VectorXd dx;

  Eigen::SparseMatrix<double> jacobian;

  /// Resize state vectors for a network while retaining Eigen allocations when
  /// dimensions are unchanged. Values are reset to the standard flat start.
  void prepare_state(int ac_bus_count, int dc_bus_count);

  /// Resize equation/step storage for the current Jacobian layout.
  void prepare_equations(int p_equations, int q_equations,
                         int dc_equations, int extra_equations = 0);

  /// Reset iteration vectors without changing their dimensions.
  void reset_iteration() noexcept;

  [[nodiscard]] int equation_count() const noexcept {
    return np + nq + ndc_eq + nf;
  }
};

}  // namespace hacdcpf::powerflow

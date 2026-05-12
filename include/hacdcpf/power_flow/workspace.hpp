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
};

}  // namespace hacdcpf::powerflow

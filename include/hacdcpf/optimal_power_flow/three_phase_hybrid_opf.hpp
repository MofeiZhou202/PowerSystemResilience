#pragma once

#include <complex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/graph/sparse_kron_reduction.hpp"

namespace hacdcpf::opf::phase_hybrid {

struct PhaseGenerator {
  int phase_node{-1};
  double p_min_pu{0.0};
  double p_max_pu{0.0};
  double q_min_pu{0.0};
  double q_max_pu{0.0};
  double cost_c2{0.0};
  double cost_c1{0.0};
};

struct PhaseVSC {
  std::vector<int> phase_nodes;
  int dc_terminal{-1};
  double efficiency{0.98};
  double s_max_pu{0.0};
  bool fixed_unity_power_factor{false};
};

struct ThreePhaseHybridOPFCase {
  std::string name;
  double base_mva{1.0};
  graph::SparseComplexMatrix y_ac;
  Eigen::VectorXcd i_ac_fixed;
  Eigen::VectorXd p_load_pu;
  Eigen::VectorXd q_load_pu;
  Eigen::VectorXd v_min_pu;
  Eigen::VectorXd v_max_pu;
  Eigen::VectorXcd voltage_start;

  std::vector<int> reference_nodes;
  Eigen::VectorXcd reference_voltage;
  std::vector<std::vector<int>> three_phase_bus_nodes;
  double vuf_max{0.03};

  std::vector<PhaseGenerator> generators;
  std::vector<PhaseVSC> converters;
  Eigen::SparseMatrix<double> g_dc;
  Eigen::VectorXd p_dc_load_pu;
  Eigen::VectorXd v_dc_start;
  Eigen::VectorXd v_dc_min_pu;
  Eigen::VectorXd v_dc_max_pu;
};

enum class ModelVariant {
  Full,
  GraphReduced,
};

enum class SolverBackend {
  Ipopt,
  NativeIPM,
};

struct ThreePhaseHybridOPFOptions {
  ModelVariant variant{ModelVariant::Full};
  SolverBackend backend{SolverBackend::Ipopt};
  graph::SparseKronOptions reduction_options{};
  int max_iterations{300};
  double tolerance{1e-7};
  bool warm_start_with_ipopt{false};
  bool verify_derivatives{false};
  Eigen::VectorXd primal_start;
};

struct ThreePhaseHybridOPFResult {
  bool converged{false};
  std::string status;
  std::string solver;
  int iterations{0};
  int variables{0};
  int equalities{0};
  int inequalities{0};
  int equality_jacobian_nonzeros{0};
  int eliminated_phase_nodes{0};
  double objective{0.0};
  double runtime_ms{0.0};
  double initial_primal_residual{0.0};
  int initial_worst_equality{-1};
  int initial_worst_inequality{-1};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double complementarity{0.0};
  double max_voltage_violation{0.0};
  double max_vuf{0.0};
  double max_converter_violation{0.0};
  double max_equality_jacobian_error{0.0};
  double max_inequality_jacobian_error{0.0};
  double max_lagrangian_hessian_error{0.0};
  Eigen::VectorXd primal;
  Eigen::VectorXd equality_dual;
  Eigen::VectorXd inequality_dual;
  Eigen::VectorXcd full_voltage;
  graph::SparseKronResult reduction;
};

ThreePhaseHybridOPFResult solve_three_phase_hybrid_opf(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFOptions& options = {});

}  // namespace hacdcpf::opf::phase_hybrid

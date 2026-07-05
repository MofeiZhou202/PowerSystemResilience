#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/dynamics/DynamicSystem.hpp"

namespace hacdcpf::dynamics {

struct DynamicDaeStateInfo {
  int index{0};
  int source_state_index{-1};
  std::string kind;
  std::string label;
  std::string device_type;
  int component_index{0};
};

struct DynamicDaeDiagnosticOptions {
  // Export a PSD-style positive-sequence object:
  // [bus Vr..., bus Vi..., selected differential states...].
  bool positive_sequence_projection{true};

  // Match PSD Test 03 state convention for SimpleMarconato + AVRTypeI:
  // eq_p, ed_p, eq_pp, ed_pp, delta, omega, Vf, Vr1, Vr2, Vm per generator.
  // HACDCPF's fixed tau_m placeholders are excluded from this diagnostic object.
  bool psd_simple_marconato_state_order{false};

  bool build_jacobian{true};
  double finite_difference_step{1e-3};
};

struct DynamicDaeDiagnostics {
  bool success{false};
  std::string message;
  std::string coordinate_system;

  int n_variables{0};
  int n_differential{0};
  int n_algebraic{0};

  Eigen::VectorXd state;
  Eigen::VectorXd residual;
  Eigen::VectorXd mass_diag;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd reduced_jacobian;
  Eigen::VectorXcd eigenvalues;

  std::vector<DynamicDaeStateInfo> states;

  double residual_inf_norm{0.0};
  double jacobian_inf_norm{0.0};
  double reduced_jacobian_inf_norm{0.0};
};

DynamicDaeDiagnostics dynamic_dae_diagnostics(
    DynamicSystem& system,
    const DynamicDaeDiagnosticOptions& options = {});

}  // namespace hacdcpf::dynamics

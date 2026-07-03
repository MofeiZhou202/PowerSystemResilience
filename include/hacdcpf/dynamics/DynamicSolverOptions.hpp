#pragma once

#include <string>

#include "hacdcpf/power_flow/power_flow_options.hpp"

namespace hacdcpf::dynamics {

enum class DynamicSolverType {
  PartitionedEuler,
  PartitionedHeun,
  PartitionedRK4,
  BackwardEulerNewton,
  TrapezoidalNewton,
  RosenbrockEuler
};

enum class DynamicLinearSolverType {
  EigenSparseLU,
  EigenSparseQR,
  EigenBiCGSTAB,
  KLU,
  UMFPACK,
  Pardiso,
  PETSc
};

struct DynamicSolverOptions {
  DynamicSolverType solver_type{DynamicSolverType::PartitionedHeun};
  DynamicLinearSolverType linear_solver{DynamicLinearSolverType::EigenSparseLU};

  double t_start_s{0.0};
  double t_end_s{1.0};
  double dt_s{0.01};

  double abs_tol{1e-8};
  double rel_tol{1e-6};
  int max_newton_iters{20};
  double newton_tol{1e-8};
  double newton_damping_min{1e-3};

  bool use_adaptive_step{false};
  bool use_analytic_jacobian{false};
  bool use_numerical_jacobian{false};

  bool run_power_flow_initialization{true};
  bool trim_dynamic_initial_conditions{true};
  bool project_to_canonical{true};
  bool synthesize_three_phase_if_absent{true};
  bool include_constant_power_as_admittance{true};
  bool dynamic_dc_link{false};
  bool record_every_step{true};
  bool record_initial_state{true};
  bool verbose{false};
  int max_step_halving{8};
  int max_dynamic_trim_iters{12};
  int output_every_steps{1};
  double output_interval_s{0.0};
  int max_recorded_snapshots{0};
  double dynamic_trim_tol{1e-7};
  int algebraic_network_max_iters{4};
  double algebraic_network_tol{1e-9};

  double source_stiffness_pu{1e4};
  double inverter_virtual_reactance_pu{0.10};
  double dc_link_capacitance_s{0.10};
  double dc_link_coupling_conductance_pu{20.0};
  double min_branch_impedance_pu{1e-6};
  double singular_regularization_pu{1e-8};

  PowerFlowOptions power_flow_options{};
};

}  // namespace hacdcpf::dynamics

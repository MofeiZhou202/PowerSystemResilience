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
  RosenbrockEuler,
  // Simultaneous mass-matrix DAE: bus voltages are algebraic states solved
  // together with device states in one sparse Newton system per step (no nested
  // network solve). See docs/dynamics_psid_parity_upgrade_plan.md, Phase 1.
  MassMatrixDae
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

enum class DynamicDaeStepMethod {
  BackwardEuler,
  Trapezoidal
};

enum class DynamicDaeJacobianMode {
  FiniteDifference,
  HybridAnalytic,
  HybridAnalyticColored
};

struct DynamicSolverOptions {
  DynamicSolverType solver_type{DynamicSolverType::PartitionedHeun};
  DynamicLinearSolverType linear_solver{DynamicLinearSolverType::EigenSparseLU};
  DynamicDaeStepMethod dae_step_method{DynamicDaeStepMethod::BackwardEuler};
  DynamicDaeJacobianMode dae_jacobian_mode{DynamicDaeJacobianMode::HybridAnalytic};

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
  bool dae_reuse_jacobian_factorization{true};
  bool dae_use_analytic_network_jacobian{true};
  bool dae_use_analytic_device_jacobian{true};
  bool dae_use_fd_current_jacobian_corrections{true};
  // Skip the O(n) global finite-difference Jacobian sweep when the analytic
  // network block and per-device (local finite-difference) Jacobian blocks are
  // active. Those local blocks already cover every device that implements
  // addJacobian, so the global sweep is redundant for them; skipping it makes
  // each Jacobian rebuild cost O(devices) instead of O(system size). Opt-in
  // because device types without an addJacobian override still need the global
  // sweep for a complete Jacobian.
  bool dae_skip_global_fd_when_analytic{false};
  int dae_jacobian_max_reuse_steps{8};
  double dae_jacobian_stale_residual_ratio{0.75};

  bool run_power_flow_initialization{true};
  bool trim_dynamic_initial_conditions{true};
  bool project_to_canonical{true};
  bool synthesize_three_phase_if_absent{true};
  bool include_constant_power_as_admittance{true};
  bool dynamic_dc_link{false};
  bool record_every_step{true};
  bool record_initial_state{true};
  bool record_device_outputs{true};
  bool verbose{false};
  int max_step_halving{12};
  int max_dynamic_trim_iters{12};
  int output_every_steps{1};
  double output_interval_s{0.0};
  int max_recorded_snapshots{0};
  double dynamic_trim_tol{1e-7};
  bool use_consistent_dynamic_initialization{true};
  int algebraic_network_max_iters{6};
  double algebraic_network_tol{1e-6};
  // When the partitioned Gauss/Picard network fixed-point stalls (linear
  // convergence on stiff constant-power / IBR mixes), fall back to a
  // finite-difference Newton solve of the algebraic network residual
  // g(V) = I_inj(V) - Y_eff*V = 0 before declaring non-convergence.
  bool network_newton_fallback{true};

  double min_accepted_step_s{1e-7};
  double voltage_collapse_min_ac_pu{0.05};
  double voltage_collapse_min_dc_pu{0.05};
  double voltage_blowup_max_ac_pu{2.50};
  double voltage_blowup_max_dc_pu{2.50};
  bool enforce_voltage_health_check{true};
  bool allow_low_voltage_during_active_fault{true};

  double source_stiffness_pu{1e4};
  double inverter_virtual_reactance_pu{0.10};
  double dc_link_capacitance_s{0.10};
  double dc_link_coupling_conductance_pu{20.0};
  double min_branch_impedance_pu{1e-6};
  double singular_regularization_pu{1e-8};
  // Time constant of the per-bus measured-frequency low-pass filter (design doc
  // §7 role 4). The filter tracks the bus voltage-angle derivative; larger values
  // smooth the signal more.
  double measured_frequency_filter_t_s{0.02};

  // Enable device-level DER protection (IEEE 1547 ride-through / trip / reconnect,
  // design doc §11.7). Opt-in: when true the solver evaluates each device's
  // protection state machine on measured quantities after every accepted step and
  // applies trips/reconnects through the network. Devices additionally gate on
  // their own per-device enable flag, so this defaults off and is fully
  // backward-compatible.
  bool enable_der_protection{false};

  // Compute a small-signal (modal) screen about the initialized operating point
  // and attach a compact summary to DynamicResults::modal (design doc §18).
  // Opt-in: linearizes the DAE once (Schur-reduced state Jacobian, eigenvalues,
  // damping) before time-stepping; does not affect the time-domain trajectory.
  bool compute_small_signal{false};

  PowerFlowOptions power_flow_options{};
};

}  // namespace hacdcpf::dynamics

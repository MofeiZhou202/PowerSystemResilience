#pragma once

/// power_flow/power_flow_options.hpp
/// ===================================
/// Power flow solver options and sub-structs.
/// Replaces: model/options.hpp.

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/defaults.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/power_flow/robust_nonlinear_options.hpp"

namespace hacdcpf {

struct InitialState {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
};

struct ReactiveLimit {
  double qmin{-0.5};  ///< Aggregate bus lower reactive limit [p.u. on system base]
  double qmax{0.5};   ///< Aggregate bus upper reactive limit [p.u. on system base]
};

struct DistributedSlack {
  std::vector<int> participating_buses;
  std::vector<double> participation_factors;
  int reference_bus{0};
  std::unordered_map<int, double> max_participation_p;
};

// ═══════════════════════════════════════════════════════════════════════
// PowerFlowOptions sub-structs
// ═══════════════════════════════════════════════════════════════════════

struct ConvergenceOptions {
  int    max_iter{Defaults::kPFMaxIter};
  double tol{Defaults::kPFTol};
  int    fdpf_max_iter{1000};
};

struct LinearSolverOptions {
  int    max_line_search_steps{10};
  int    max_regularization_steps{8};
  double regularization_lambda0{1e-6};
  double regularization_growth{10.0};
  double max_delta_va_rad{1.50};
  double max_delta_vm_pu{0.50};
  double max_delta_vdc_pu{0.50};
  bool   enable_coupled_jacobian{false};
  bool   enable_augmented_equations{false};
  bool   enable_semi_smooth_newton{false};
};

struct GlobalizationOptions {
  enum class Strategy { LineSearch, TrustRegion, PseudoTransient };
  Strategy strategy{Strategy::LineSearch};
  double   trust_region_radius0{1.0};
  double   trust_region_max{10.0};
  double   ptc_delta0{1.0};
  double   ptc_growth{2.0};
};

struct PVPQSwitchingOptions {
  double pv_q_hysteresis_pu{0.01};
  double pv_recover_vm_tol_pu{0.01};
  bool   enable_pv_pq_conversion{true};
  bool   enable_auto_swing_selection{true};
};

struct ConverterModeOptions {
  double          converter_vdc_switch_high_pu{0.03};
  double          converter_vdc_switch_low_pu{0.01};
  int             mode_hysteresis_iters{2};
  bool            enable_converter_mode_switching{true};
  bool            enable_converter_coordination_check{false};
  LossModelType   loss_model{LossModelType::Linear};
};

struct ZipLoadOptions {
  double pw[3]{1.0, 0.0, 0.0};
  double qw[3]{1.0, 0.0, 0.0};
};

struct RuntimeOptions {
  int  ac_eval_threads{0};
  bool enable_solver_profiling{false};
  bool enable_iteration_log{false};
  bool verbose{false};
};

// ═══════════════════════════════════════════════════════════════════════
// PowerFlowOptions (flat layout for backward compatibility)
// ═══════════════════════════════════════════════════════════════════════
struct PowerFlowOptions {
  int max_iter{Defaults::kPFMaxIter};
  double tol{Defaults::kPFTol};

  int ac_eval_threads{0};

  bool enable_solver_profiling{false};
  bool enable_iteration_log{false};

  int max_line_search_steps{10};
  int max_regularization_steps{8};
  double regularization_lambda0{1e-6};
  double regularization_growth{10.0};
  double max_delta_va_rad{1.50};
  double max_delta_vm_pu{0.50};
  double max_delta_vdc_pu{0.50};

  double pv_q_hysteresis_pu{0.01};
  double pv_recover_vm_tol_pu{0.01};

  double converter_vdc_switch_high_pu{0.03};
  double converter_vdc_switch_low_pu{0.01};
  int mode_hysteresis_iters{2};

  bool enable_pv_pq_conversion{true};
  bool enable_auto_swing_selection{true};
  bool enable_converter_mode_switching{true};
  bool enable_converter_coordination_check{false};
  // Treat converter capacity/current/modulation/duty limits as hard feasibility
  // constraints. A determined power flow has no dispatch freedom with which to
  // repair an incompatible setpoint, so a violating Newton root is rejected as
  // physically infeasible instead of being reported as converged. The default
  // warning-only mode preserves legacy behavior; use OPF when redispatch is
  // required to find a feasible operating point.
  bool enforce_converter_physical_limits{false};
  // When true, a converter that is the *sole* voltage former of a single-DC-bus
  // island holds its DC bus voltage rigidly at the setpoint (DC-slack converter
  // mode, multi-converter model §6.3) instead of forming it through a stiff
  // droop.  Off by default: the droop promotion keeps Vdc within ~0.1% and is
  // the established behavior.
  bool enable_rigid_vdc_former{false};
  bool verbose{false};
  LossModelType loss_model{LossModelType::Linear};

  double zip_pw[3]{1.0, 0.0, 0.0};
  double zip_qw[3]{1.0, 0.0, 0.0};

  int fdpf_max_iter{1000};

  bool enable_coupled_jacobian{false};
  bool enable_augmented_equations{false};
  bool enable_semi_smooth_newton{false};

  enum class GlobalizationStrategy { LineSearch, TrustRegion, PseudoTransient };
  GlobalizationStrategy globalization{GlobalizationStrategy::LineSearch};

  double trust_region_radius0{1.0};
  double trust_region_max{10.0};
  double ptc_delta0{1.0};
  double ptc_growth{2.0};

  std::optional<InitialState> initial_state;

  powerflow::RobustNonlinearOptions robust_nonlinear;

  static PowerFlowOptions from_parts(
      const ConvergenceOptions&      conv,
      const LinearSolverOptions&     linear    = {},
      const GlobalizationOptions&    glob      = {},
      const PVPQSwitchingOptions&    switching = {},
      const ConverterModeOptions&    converter = {},
      const ZipLoadOptions&          zip       = {},
      const RuntimeOptions&          runtime   = {});
};

}  // namespace hacdcpf

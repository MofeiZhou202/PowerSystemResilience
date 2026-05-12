#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/power_flow/robust_nonlinear_options.hpp"

namespace hacdcpf {

struct InitialState {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
};

struct ReactiveLimit {
  double qmin{-0.5};
  double qmax{0.5};
};

struct DistributedSlack {
  std::vector<int> participating_buses;
  std::vector<double> participation_factors;
  int reference_bus{0};
  std::unordered_map<int, double> max_participation_p;
};

struct PowerFlowOptions {
  int max_iter{50};
  double tol{1e-8};

  // Internal AC kernel parallelism for a single solve.
  // 0 => auto-detect hardware threads.
  int ac_eval_threads{1};

  // Enable detailed profiling/counters in PowerFlowResult::profiling.
  bool enable_solver_profiling{false};

  // Robust Newton globalization and fallback controls.
  int max_line_search_steps{10};
  int max_regularization_steps{8};
  double regularization_lambda0{1e-6};
  double regularization_growth{10.0};
  double max_delta_va_rad{1.50};
  double max_delta_vm_pu{0.50};
  double max_delta_vdc_pu{0.50};

  // PV/PQ switching hysteresis controls (per-unit on system base).
  double pv_q_hysteresis_pu{0.01};
  double pv_recover_vm_tol_pu{0.01};

  // Converter mode switching hysteresis controls.
  double converter_vdc_switch_high_pu{0.03};
  double converter_vdc_switch_low_pu{0.01};
  int mode_hysteresis_iters{2};

  bool enable_pv_pq_conversion{true};
  bool enable_auto_swing_selection{true};
  bool enable_converter_mode_switching{true};
  bool verbose{false};
  LossModelType loss_model{LossModelType::Linear};

  // ZIP load model weights [constant-P, constant-I, constant-Z].
  // Each triplet must sum to 1.0.  Default [1,0,0] = pure constant power.
  double zip_pw[3]{1.0, 0.0, 0.0};  // active power weights
  double zip_qw[3]{1.0, 0.0, 0.0};  // reactive power weights

  // FDPF-specific max iterations (decoupled P/Q sweeps).
  int fdpf_max_iter{1000};

  // ── Exact fully-coupled Newton upgrade (opt-in) ──────────────────────
  bool enable_coupled_jacobian{false};       // Direction 1: cross-coupling derivatives
  bool enable_augmented_equations{false};    // Direction 2: setpoint equations
  bool enable_semi_smooth_newton{false};     // Direction 3: NCP-based switching

  enum class GlobalizationStrategy { LineSearch, TrustRegion, PseudoTransient };
  GlobalizationStrategy globalization{GlobalizationStrategy::LineSearch};

  double trust_region_radius0{1.0};
  double trust_region_max{10.0};
  double ptc_delta0{1.0};
  double ptc_growth{2.0};

  // Optional initial voltage state for the solver.
  // If non-empty, overrides the default initialization from bus data.
  std::optional<InitialState> initial_state;

  // Robust nonlinear solve controls (Phase 1+).
  powerflow::RobustNonlinearOptions robust_nonlinear;
};

}  // namespace hacdcpf

#pragma once

#include <string>
#include <vector>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

// ─────────────────────────────────────────────────────────────────────────────
// Load/generation direction for the continuation parameter λ
// ─────────────────────────────────────────────────────────────────────────────

/// Per-bus load and generation scaling increments for the CPF parameter λ.
///
/// The parameterized power system is:
///   P_load_i(λ) = P_load_i^0 + λ · dp_load[i]
///   Q_load_i(λ) = Q_load_i^0 + λ · dq_load[i]
///   P_gen_i(λ)  = P_gen_i^0  + λ · dp_gen[i]   (non-slack buses only)
///
/// All quantities are in MW / Mvar (same physical units as SolverData).
/// λ = 0 corresponds to the base operating point; λ = 1 doubles all loads
/// when using the proportional direction.
struct CpfDirection {
  std::vector<double> dp_load;  ///< Active load increase per unit λ (MW/bus)
  std::vector<double> dq_load;  ///< Reactive load increase per unit λ (Mvar/bus)
  std::vector<double> dp_gen;   ///< Generator active increase per unit λ (MW, bus-indexed)

  /// Build a proportional direction: every bus load scales by its current
  /// value, and all non-slack generators share the increase in proportion
  /// to their current output.
  static CpfDirection proportional(const SolverData& data);

  /// Build a direction where only the specified buses participate in the
  /// load increase (uniform 1 MW/Mvar per bus in the list).
  static CpfDirection at_buses(const SolverData& data,
                               const std::vector<int>& bus_indices);
};

// ─────────────────────────────────────────────────────────────────────────────
// Options
// ─────────────────────────────────────────────────────────────────────────────

/// Options for the Continuation Power Flow (CPF) solver.
struct CpfOptions {
  /// Maximum loading factor λ to trace.  Tracing terminates when λ ≥ lambda_max
  /// even if the voltage-collapse point has not been reached.
  double lambda_max{5.0};

  /// Initial continuation step size Δλ.
  double step_init{0.05};

  /// Minimum step size.  When adaptive reduction falls below this threshold
  /// the solver reports the nose point reached and stops.
  double step_min{1e-5};

  /// Maximum step size (prevents skipping over nose-point features).
  double step_max{0.20};

  /// Step growth factor applied after a successful corrector solve.
  double step_grow{2.0};

  /// Step shrink factor applied after a corrector failure.
  double step_shrink{0.5};

  /// Maximum Newton iterations in each corrector solve.
  int corrector_max_iter{50};

  /// Convergence tolerance for the corrector Newton solve (p.u.).
  double corrector_tol{1e-8};

  /// Bus index to monitor on the P–V curve (0-based).
  /// -1 = auto-select as the bus with the largest base-case active demand.
  int monitor_bus{-1};

  /// Store vm/va vectors at every CPF step when true.
  /// When false, only the monitor-bus voltage is stored per step.
  bool trace_all_buses{false};

  /// Maximum total number of predictor–corrector steps (safety cap).
  int max_steps{2000};

  /// Stop if any bus voltage magnitude drops below this threshold (p.u.).
  double vm_min_pu{0.25};

  /// Use a secant (two-point extrapolation) predictor.
  /// When false, a constant predictor (warm-start from current solution) is used.
  bool use_secant_predictor{true};

  /// After the first natural-parameter step, augment the corrector with an
  /// arc-length hyperplane and solve lambda as an unknown. This permits the
  /// trace to pass the P-V nose instead of stopping when dV/dlambda is singular.
  bool enable_arc_length{true};

  /// Number of accepted lower-branch points to retain after lambda first turns
  /// downward. A small positive value makes nose detection observable in tests
  /// without tracing deep into the low-voltage branch.
  int lower_branch_steps{2};
};

// ─────────────────────────────────────────────────────────────────────────────
// Results
// ─────────────────────────────────────────────────────────────────────────────

/// A single point on the P–V curve.
struct CpfPoint {
  double lambda{0.0};        ///< Continuation (loading) parameter.
  double vm_monitor{0.0};    ///< Monitor-bus voltage magnitude (p.u.).
  double p_total_mw{0.0};    ///< Total active load power (MW).
  double q_total_mvar{0.0};  ///< Total reactive load power (Mvar).
  double residual{0.0};      ///< Physical power-flow infinity-norm residual (p.u.).

  /// All-bus voltage magnitudes (p.u.) — populated only when
  /// CpfOptions::trace_all_buses is set.
  std::vector<double> vm;

  /// All-bus voltage angles (radians) — populated only when
  /// CpfOptions::trace_all_buses is set.
  std::vector<double> va;

  /// All DC-bus voltage magnitudes (p.u.) — populated only when
  /// CpfOptions::trace_all_buses is set.
  std::vector<double> vdc;
};

/// Output of the Continuation Power Flow solver.
struct CpfResult {
  /// Ordered solution path from λ = 0 (base case) toward voltage collapse.
  std::vector<CpfPoint> trace;

  /// Maximum loadability factor λ at the nose point (or last converged step).
  double lambda_max{0.0};

  /// Total active load power at the nose point (MW).
  double p_max_mw{0.0};

  /// Monitor-bus voltage magnitude at the nose point (p.u.).
  double vm_at_nose{0.0};

  /// Index of the bus that was monitored (0-based).
  int monitor_bus{-1};

  /// True if the voltage-collapse (nose) point was found within lambda_max.
  bool nose_found{false};

  /// True only when every accepted continuation point enforced Newton's
  /// PV/PQ active-set switching. Arc-length continuation currently requires a
  /// fixed equation layout, so systems with PV buses use natural parameterization.
  bool q_limits_enforced{true};

  /// Whether the augmented arc-length corrector was actually used.
  bool arc_length_used{false};

  /// Machine-readable scope and explicit limitations of this trace.
  std::string model_scope{"cpf:unset"};
  std::string model_limitations;
  std::vector<std::string> warnings;

  /// Human-readable reason for termination.
  std::string termination_reason;

  /// Total number of corrector (Newton) power-flow solves performed.
  int total_pf_solves{0};
};

/// Summary voltage stability indices derived from a completed CpfResult.
struct VoltageStabilityIndex {
  /// Bus index that was monitored (0-based).
  int bus{-1};
  /// Maximum loadability factor at the nose.
  double lambda_max{0.0};
  /// Total active load power at the nose (MW).
  double p_max_mw{0.0};
  /// Loading margin from base case to the nose point (MW).
  double p_margin_mw{0.0};
  /// Monitor-bus voltage at the nose (p.u.).
  double vm_at_nose{0.0};
  /// Monitor-bus voltage at the base case λ = 0 (p.u.).
  double vm_base{0.0};
};

// ─────────────────────────────────────────────────────────────────────────────
// Solver
// ─────────────────────────────────────────────────────────────────────────────

/// Static voltage stability analysis via Continuation Power Flow (CPF).
///
/// Traces the P–V nose curve from the base operating point (λ = 0) toward
/// voltage collapse using an adaptive predictor–corrector scheme:
///
/// Predictor  — normalized secant tangent in augmented [state, lambda] space.
/// Corrector  — Newton solve of the power-flow residual plus an arc-length
///              hyperplane; lambda is an unknown after the first step.
/// Adaptation — step halved on corrector divergence; doubled on convergence
///              (bounded by [step_min, step_max]).
/// Nose detection — the first sign reversal of the lambda tangent identifies
///              the upper/lower-branch transition.
///
/// The solver reuses the existing NewtonSolver and SolverData infrastructure
/// without modification.  It supports pure AC systems, hybrid AC–DC systems
/// (DC voltages warm-started from the previous solution), and multi-area
/// systems (including multi-slack).
class CpfSolver {
 public:
  CpfOptions opts;

  /// Trace the P–V curve starting from base_data at λ = 0.
  ///
  /// @param base_data  Base-case power system (defines the λ = 0 operating point).
  /// @param direction  Load/generation increment direction for the continuation.
  /// @return           CPF trace and nose-point diagnostics.
  CpfResult solve(const SolverData& base_data,
                  const CpfDirection& direction) const;
};

/// Compute voltage stability indices from a completed CpfResult.
///
/// @param result     Completed CPF trace (from CpfSolver::solve).
/// @param base_data  Original base-case SolverData (for base loading).
VoltageStabilityIndex compute_vsi(const CpfResult& result,
                                   const SolverData& base_data);

}  // namespace hacdcpf::powerflow

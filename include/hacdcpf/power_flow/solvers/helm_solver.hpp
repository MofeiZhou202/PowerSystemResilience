#pragma once

#include <vector>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

/// Warm-start strategy for the HELM solver.
enum class HelmWarmStart {
  /// Flat start at s = 0 (default, mathematically exact).
  None,
  /// Solve a DC power flow first; use resulting angles as the initial
  /// Padé comparison baseline.  Reduces coefficient count for
  /// transmission cases with small angle differences.
  DCPF,
  /// Run a small number of Gauss–Seidel AC power flow iterations
  /// before launching the HELM series.  The GS voltage estimate is
  /// used as the initial Padé baseline, accelerating convergence
  /// detection on well-conditioned systems.
  GaussSeidel,
};

/// Options specific to the HELM power flow solver.
struct HelmOptions {
  /// Maximum number of power-series coefficients to compute.
  int max_coef{100};
  /// Padé convergence tolerance: solver stops when successive Padé
  /// approximants differ by less than this in both magnitude and angle.
  double mismatch{1e-6};
  /// Whether to enforce generator reactive-power limits (Q-limit switching).
  bool enforce_q_limits{true};

  // ── Sparse Ytrans ──────────────────────────────────────────────────────────
  /// Minimum number of buses at which the HELM coefficient system uses sparse
  /// LU. Smaller systems use dense full-pivot LU. Set to 0 to always use sparse LU.
  int sparse_threshold{200};

  // ── Warm start ─────────────────────────────────────────────────────────────
  /// Warm-start strategy (default: None = flat start).
  HelmWarmStart warm_start{HelmWarmStart::None};
  /// Number of Gauss–Seidel pre-iterations when warm_start == GaussSeidel.
  int gauss_iter{5};

  // ── Multi-slack / distributed slack ───────────────────────────────────────
  /// When true, all buses with BusType::SLACK are treated as fixed-voltage
  /// reference buses (identity rows in Ymod).  Multiple slack buses are
  /// supported and handled automatically.
  ///
  /// Distributed slack (load-following via K-factor participation factors)
  /// is a separate mechanism; it is triggered when participation_factors is
  /// non-empty.
  bool allow_multi_slack{true};

  /// Optional participation factors for distributed-slack (K-factor) mode.
  /// When non-empty, the remaining scheduled-power imbalance is distributed
  /// among PV generators according to these factors (indexed by bus position,
  /// 0-based). Entries must be finite and non-negative; entries for SLACK and
  /// PQ buses must be zero because their active injection cannot be prescribed
  /// by this embedding. Eligible PV factors are normalised internally.
  std::vector<double> participation_factors;

  // ── Convergence trace ──────────────────────────────────────────────────────
  /// If non-null, the solver appends the maximum Padé magnitude-delta across
  /// all buses at each convergence check step to this vector.  Entries are in
  /// the order they are computed (every 2 coefficients, starting at length 4).
  /// Useful for plotting the Padé convergence path.
  mutable std::vector<double>* pade_trace_out{nullptr};
};

/// Holomorphic Embedding Load-flow Method (HELM) AC power flow solver.
///
/// Computes the AC power flow solution by embedding the power flow equations
/// in a holomorphic function of a complex parameter s ∈ ℂ, then evaluating the
/// power series at s = 1 via Padé approximants.
///
/// Key properties:
///  - Exact no-load germ at s = 0 supports ideal transformer taps and shifts.
///  - Warm-start modes (DCPF / Gauss–Seidel) accelerate convergence detection.
///  - Multiple SLACK buses are supported; all receive identity rows in Ymod.
///  - Distributed slack via participation factors (K-factor method).
///  - Sparse Ytrans storage above sparse_threshold buses.
///  - Divergence of the Padé approximant is a certificate of infeasibility.
///
/// Bus models:
///  - Slack  : voltage magnitude and angle fixed (identity rows).
///  - PQ     : complex power injection fixed (load buses).
///  - PV     : active power and voltage magnitude fixed (Model 2: |V|² constraint).
///
/// Algorithm follows the HELMpy open-source reference (Molina & Ortega, ULA 2019,
/// AGPLv3).  The Padé matrix method is used for analytic continuation.
struct HelmSolver {
  HelmOptions helm_opts;

  /// Solve AC power flow on the given SolverData.
  /// Only the AC subsystem (ac_buses, ac_branches, generators) is used.
  PowerFlowResult solve(const SolverData& data,
                        const PowerFlowOptions& opt = {}) const;
};

}  // namespace hacdcpf::powerflow

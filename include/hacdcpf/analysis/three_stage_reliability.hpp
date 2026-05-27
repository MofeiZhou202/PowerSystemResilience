#pragma once

// =============================================================================
// three_stage_reliability.hpp
//
// Native C++ three-stage MILP fault-recovery reliability evaluator.  The
// implementation reads the existing HybridPowerSystem JSON schema and solves
// each staged load-restoration subproblem with the embedded MIPSolvers backend.
//
// Three-stage framework (per IEEE Std 1366-2012 / Chinese DL/T 836):
//   Stage 1  [0, τ_SW]:        Fault isolation   — protect healthy zones
//   Stage 2  [τ_SW, τ_TP]:     Post-fault reconfig — restore as many loads as
//                              possible with switching operations
//   Stage 3  [τ_TP, τ_RP]:     Post-repair reconfig — re-optimise topology
//                              after the faulted component is repaired
//
// No Julia runtime is required.
// =============================================================================

#include <climits>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hacdcpf::analysis {

// ─── Per-fault detail ────────────────────────────────────────────────────────

/// Per-fault detail record produced by the three-stage reliability solver.
struct ThreeStageFaultDetail {
  int line_id{0};
  std::string status;         ///< "success" | "success (approximate)" | "failed"
  std::string stage1_status;
  std::string stage2_status;
  std::string stage3_status;
  double stage1_mip_gap{0.0};
  double stage2_mip_gap{0.0};
  double stage3_mip_gap{0.0};
  double objective{0.0};
  double pls_stage1{0.0};     ///< load shed in Stage 1 (kW)
  double pls_stage2{0.0};     ///< load shed in Stage 2 (kW)
  double pls_stage3{0.0};     ///< load shed in Stage 3 (kW)
  double pls_total{0.0};      ///< pls_stage1 + 2 + 3 (kW)
  std::vector<double> psop1;  ///< SOP power per device, Stage 1 (kW)
  std::vector<double> psop2;
  std::vector<double> psop3;
};

// ─── SOP (Soft Open Point) configuration ────────────────────────────────────

/// SOP (Soft Open Point) configuration as echoed by the Julia engine.
struct ThreeStageSopConfig {
  int id{0};
  int node_a{0};
  int node_b{0};
  double pmax_kw{0.0};
  double qmax_kw{0.0};
  double efficiency{0.0};
};

// ─── Aggregate result ────────────────────────────────────────────────────────

/// Aggregate result of one three-stage reliability evaluation.
struct ThreeStageReliabilityResult {
  /// True only when the result is a verified exact evaluation inside the
  /// evaluator's full physical scope.  Hybrid DC/VSC/SOP cases currently
  /// return metrics with `ok == false` because they use the documented
  /// DC-connectivity fallback rather than a full physical restoration MILP.
  bool ok{false};
  std::string error;          ///< populated when ok == false

  // System-level IEEE Std 1366-2012 / DL/T 836 indices.
  double saifi{0.0};          ///< interruptions / customer / yr
  double saidi_min{0.0};      ///< outage duration (minutes / yr)
  double eens_kwh_yr{0.0};    ///< expected energy not supplied (kWh / yr)
  double eens_cost{0.0};      ///< EENS × ω (currency / yr)
  int    worst_line{0};       ///< line id with maximum total load shed

  // Per-load-node indices (length = nd).
  std::vector<double> nodal_eens_kwh_yr;
  std::vector<double> nodal_cif;      ///< customer interruption frequency
  std::vector<double> nodal_cid_min;  ///< customer interruption duration (min)

  // Per-contingency details (length = number of in-service simulated lines).
  std::vector<ThreeStageFaultDetail> faults;

  // SOP configuration (empty when nl_sop == 0).
  std::vector<ThreeStageSopConfig> sop_config;

  // Network counts reported by the native loader.
  int nb{0},    nb_ac{0},  nb_dc{0};
  int nl{0},    nl_ac{0},  nl_dc{0};
  int nl_vsc{0}, nl_sop{0};
  int nd{0},    ng{0},     nmg{0};

  /// Reserved for backward compatibility with the historical process bridge.
  std::filesystem::path result_json_path;

  /// Human-readable description of known modelling approximations in this
  /// result.  Empty when the model is operating within its designed scope.
  ///
  /// Current limitations always present:
  ///  - AC restoration is a finite-source LinDistFlow MILP with explicit
  ///    p_g/q_g capacity bounds, energized-bus indicators, strict radial forest
  ///    constraints, branch flow limits, voltage bounds, and continuous load shed.
  ///  - VSC converters and DC/DC converters are treated as lossless graph
  ///    edges only in the DC fallback; their power-flow setpoints are NOT
  ///    optimised.  psop vectors in FaultDetail are filled with zeros.
  ///  - DC loads and DC generation are handled by a connectivity/capacity
  ///    fallback, but no DC power-flow constraints are enforced.
  ///  - The N-1 contingency set enumerates in-service ACBranch and DCBranch
  ///    outages only.  VSC, DC/DC, switch, breaker, transformer, generator,
  ///    load, and storage outages are outside this evaluator's fault set.
  std::string model_limitations;

  /// Structured model-capability declaration.  Pure AC systems report
  /// "ac-lindistflow-milp".  Hybrid systems report
  /// "ac-lindistflow-milp+dc-connectivity-fallback" because DC/VSC/SOP
  /// physics are not co-optimised in the restoration MILP.
  std::string model_scope{"ac-lindistflow-milp+dc-connectivity-fallback"};

  /// Per-feature validity flags so downstream code can branch on whether a
  /// given physical constraint was actually enforced for the whole reported
  /// system.  AC-only runs set branch/voltage/radial/restoration flags true;
  /// hybrid runs keep them false because the DC/VSC/SOP portion is fallback-only.
  struct ValidityFlags {
    bool branch_flow_enforced{false};
    bool voltage_constraints_enforced{false};
    bool radial_topology_enforced{false};
    bool sop_dispatch_optimised{false};
    bool dc_power_flow_enforced{false};
    bool restoration_milp_solved{false};
  };
  ValidityFlags validity{};
};

// ─── Options ─────────────────────────────────────────────────────────────────

/// Backward-compatible options. The native C++ implementation ignores the
/// former Julia process-launch fields.
struct ThreeStageReliabilityOptions {
  /// Deprecated: ignored by the native C++ implementation.
  std::filesystem::path julia_executable;

  /// Deprecated: ignored by the native C++ implementation.
  std::filesystem::path julia_project_dir;

  /// Deprecated: ignored by the native C++ implementation.
  std::filesystem::path cli_script;

  /// Deprecated: ignored by the native C++ implementation.
  std::filesystem::path workdir;

  bool keep_workdir{false};

  /// Deprecated: ignored by the native C++ implementation.
  bool inherit_stdio{true};

  /// Maximum number of switching operations allowed during Stage 2 (post-fault
  /// switching restoration).  Each normally-open AC switch that is closed in
  /// Stage 2 to merge two distinct connected components counts as one
  /// operation.  Switches within the same component are free.
  /// Use INT_MAX (default) to impose no limit.  A value of 0 disables all
  /// Stage-2 normally-open switch closing.  Negative values are treated as 0.
  /// Typical field values are 1–5 operations per fault.
  int max_switch_operations{INT_MAX};
};

// ─── Entry points ────────────────────────────────────────────────────────────

/// Run the three-stage reliability evaluation against ``case_json`` and return
/// the parsed metrics. Never throws — failures are reflected via
/// ``ThreeStageReliabilityResult::ok == false`` with a human-readable
/// ``error`` string.
ThreeStageReliabilityResult run_three_stage_reliability(
    const std::filesystem::path& case_json,
    const ThreeStageReliabilityOptions& options = {});

/// Convenience overload: write the case JSON for the caller, then run.
ThreeStageReliabilityResult run_three_stage_reliability_from_string(
    const std::string& case_json_text,
    const ThreeStageReliabilityOptions& options = {});

}  // namespace hacdcpf::analysis

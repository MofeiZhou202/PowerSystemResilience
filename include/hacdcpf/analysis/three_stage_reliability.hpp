#pragma once

// =============================================================================
// three_stage_reliability.hpp
//
// C++ bridge for the Julia-implemented three-stage MILP fault-recovery
// reliability evaluator under julia/reliability/.  The Julia engine is invoked
// out-of-process via its CLI; data is exchanged through JSON files following
// the schemas in:
//   - julia/reliability/src/load_case_json.jl    (input case)
//   - julia/reliability/src/save_results_json.jl (output result)
//
// Three-stage framework (per IEEE Std 1366-2012 / Chinese DL/T 836):
//   Stage 1  [0, τ_SW]:        Fault isolation   — protect healthy zones
//   Stage 2  [τ_SW, τ_TP]:     Post-fault reconfig — restore as many loads as
//                              possible with switching operations
//   Stage 3  [τ_TP, τ_RP]:     Post-repair reconfig — re-optimise topology
//                              after the faulted component is repaired
//
// Runtime requirements (validated lazily):
//   * `julia` available on PATH (or pass an explicit path via options),
//   * `julia/reliability/Project.toml` resolved (Pkg.instantiate already run),
//   * Gurobi solver reachable from the Julia environment.
//
// This bridge is intentionally a thin wrapper — it does not duplicate the
// optimisation model in C++.
// =============================================================================

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hacdcpf::analysis {

// ─── Per-fault detail ────────────────────────────────────────────────────────

/// Per-fault detail record produced by the three-stage reliability solver.
struct ThreeStageFaultDetail {
  int line_id{0};
  std::string status;         ///< "success" | "failed"
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

  // Network counts as reported by the Julia loader.
  int nb{0},    nb_ac{0},  nb_dc{0};
  int nl{0},    nl_ac{0},  nl_dc{0};
  int nl_vsc{0}, nl_sop{0};
  int nd{0},    ng{0},     nmg{0};

  /// Path to the result JSON written by the Julia engine. Populated when
  /// ``ThreeStageReliabilityOptions::keep_workdir`` is true (or when a custom
  /// workdir is provided), otherwise empty.
  std::filesystem::path result_json_path;
};

// ─── Options ─────────────────────────────────────────────────────────────────

/// Options controlling how the Julia engine is invoked.
struct ThreeStageReliabilityOptions {
  /// Path to the `julia` executable. When empty, "julia" is looked up on PATH.
  std::filesystem::path julia_executable;

  /// Path to the Julia project directory containing Project.toml. Defaults to
  /// `<project-root>/julia/reliability` (resolved via HACDCPF_PROJECT_ROOT).
  std::filesystem::path julia_project_dir;

  /// Path to the CLI script. Defaults to `<julia_project_dir>/cli.jl`.
  std::filesystem::path cli_script;

  /// Working directory for the temp input/output JSON files. When empty an
  /// auto-generated directory under `std::filesystem::temp_directory_path()`
  /// is used and removed at the end (unless ``keep_workdir`` is true).
  std::filesystem::path workdir;

  bool keep_workdir{false};

  /// Echo the Julia stdout/stderr stream to the parent terminal (default true).
  bool inherit_stdio{true};
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

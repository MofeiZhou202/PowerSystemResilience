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
//   Stage 3  [τ_TP, τ_RP]:     Repair window — the faulted component is STILL
//                              out (it is only repaired at τ_RP), and the Stage-2
//                              reconfiguration is HELD.  This is the long window
//                              (τ_RP ≈ MTTR); load that switching could not
//                              restore is shed for its whole duration.  (Fixed:
//                              Stage 3 used to re-close the faulted component and
//                              re-open the ties, which zeroed the repair-window
//                              shed for topology-isolated load.)
//
// No external runtime is required.
// =============================================================================

#include <climits>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/reliability/failure_mode.hpp"
#include "hacdcpf/util/parallel_execution.hpp"

namespace hacdcpf::analysis {

struct ThreeStageSwitchAction {
  int sequence_order{0};
  int switch_index{-1};
  std::string switch_name;
  std::string switch_type;
  std::string action;  ///< "open", "trip", or "close"
  std::string purpose;
  int bus_from{0};
  int bus_to{0};
  double operation_time_s{0.0};
  bool validated{false};
  std::string validation_message;
};

// ─── Per-fault detail ────────────────────────────────────────────────────────

/// Per-fault detail record produced by the three-stage reliability solver.
struct ThreeStageFaultDetail {
  int line_id{0};
  std::string component_type;  ///< canonical component type, e.g. "ac_branch"
  int component_index{-1};     ///< position in the corresponding component vector
  bool ac{true};
  int from_bus{0};
  int to_bus{0};
  double failure_rate{0.0};    ///< occ / yr
  double initiating_failure_rate{0.0}; ///< physical initiating events / yr
  double scenario_probability{1.0};    ///< conditional probability after initiation
  std::string protection_scenario{"unconfigured"};
  std::string protection_id;
  std::string primary_device_id;
  std::string backup_device_id;
  double reclose_success_probability{0.0};
  double primary_failure_probability{0.0};
  double backup_failure_probability{0.0};
  double clearing_time_s{0.0};
  std::vector<std::string> protection_zone_component_ids;
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
  double raw_pls_stage1{0.0}; ///< total Stage-1 shed before N-0 qualification
  double raw_pls_stage2{0.0};
  double raw_pls_stage3{0.0};
  double n0_pls_stage1{0.0};  ///< healthy-state shed under matching stage conditions
  double n0_pls_stage2{0.0};
  double n0_pls_stage3{0.0};
  double tau_iso_hr{0.0};     ///< Stage-1 fault isolation duration
  double tau_sw_hr{0.0};      ///< Stage-2 switching/restoration duration
  double tau_rep_hr{0.0};     ///< Stage-3 repair-window duration
  double storage_energy_initial_mwh{0.0};
  double storage_energy_used_stage1_mwh{0.0};
  double storage_energy_used_stage2_mwh{0.0};
  double storage_energy_used_stage3_mwh{0.0};
  double storage_energy_remaining_mwh{0.0};
  double duration_hr{0.0};    ///< loss duration used for LOLE when this fault sheds load
  double ens_kwh{0.0};        ///< event energy not supplied before frequency weighting
  double eens_contribution_mwh_yr{0.0};
  double lole_contribution_hr_yr{0.0};
  double lolf_contribution_occ_yr{0.0};
  std::vector<ThreeStageSwitchAction> switching_sequence;
  bool protection_interlock_valid{true};
  bool restoration_milp_admitted{true};
  bool stage3_switch_plan_held{true};
  bool switching_sequence_valid{true};
  std::string switching_sequence_message;
  bool fault_isolation_explicit{false};
  std::string fault_isolation_message;
  std::vector<double> psop1;  ///< SOP power per device, Stage 1 (kW)
  std::vector<double> psop2;
  std::vector<double> psop3;
};

// ─── SOP (Soft Open Point) configuration ────────────────────────────────────

/// SOP (Soft Open Point) configuration echoed by the native reliability solver.
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
  ///  - The default N-1 contingency set enumerates in-service ACBranch and
  ///    DCBranch outages. Generator, two-winding transformer, VSC/DC-DC, and
  ///    switch/breaker faults are opt-in; DER, storage, and microgrid source
  ///    faults are included with the generator fault family. Load faults are
  ///    represented when an applied protection zone removes a load component.
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

  /// Fault-level outer-loop parallel execution diagnostics. Stage 1 -> 2 -> 3
  /// remains sequential inside each fault because Stage 3 holds the accepted
  /// Stage-2 switching plan.
  hacdcpf::util::ParallelExecutionInfo parallel_execution;

  /// Audit of session protection configuration consumed by the staged model.
  bool protection_configuration_applied{false};
  int protection_rows_applied{0};
  int protection_scenarios_generated{0};
  double initiating_fault_frequency_per_year{0.0};
  double sustained_fault_frequency_per_year{0.0};
  double transient_reclose_frequency_per_year{0.0};
  std::vector<std::string> protection_configuration_limitations;
};

// ─── Options ─────────────────────────────────────────────────────────────────

struct ThreeStageReliabilityOptions {
  /// Maximum number of switching operations allowed during Stage 2 (post-fault
  /// switching restoration).  Each normally-open AC switch that is closed in
  /// Stage 2 to merge two distinct connected components counts as one
  /// operation.  Switches within the same component are free.
  /// Use INT_MAX (default) to impose no limit.  A value of 0 disables all
  /// Stage-2 normally-open switch closing.  Negative values are treated as 0.
  /// Typical field values are 1–5 operations per fault.
  int max_switch_operations{INT_MAX};

  /// Switch indices (``Switch::index``) of normally-open ties that CANNOT be
  /// closed during Stage-2 restoration (deterministic fail-to-close).  Use this
  /// to evaluate the load-restoration impact of a tie switch's fail-to-close
  /// failure mode: run once with the tie available and once with its id listed
  /// here, then weight the load-shed difference by the switch's per-demand
  /// failure probability.  Empty (default) = every in-service tie may close.
  std::vector<int> unavailable_tie_switch_ids;

  /// Include generator forced-outage contingencies in the fault set.  Default
  /// off preserves the historical branch-only enumeration (so SAIFI/EENS and
  /// fault counts are unchanged unless explicitly enabled).
  bool include_generator_faults{false};

  /// Include 2-winding transformer outage contingencies.  When enabled the
  /// transformer outage disconnects the downstream zone. Healthy transformers
  /// are always modelled as capacity-limited topology edges; this option only
  /// controls whether their outages are added to the contingency set.
  bool include_transformer_faults{false};

  /// Include VSC and DC-DC converter outage contingencies.  A faulted converter
  /// is removed from the DC connectivity/capacity fallback for the event (its
  /// AC<->DC or DC<->DC coupling and transfer capacity are lost), so DC loads
  /// that depend on it are shed.  Default off preserves the branch-only model.
  bool include_converter_faults{false};

  /// Include AC switch and AC/DC circuit-breaker outage contingencies.  A
  /// faulted AC switch/breaker edge is forced open in all three stages (like a
  /// faulted branch); a faulted DC breaker is dropped from the DC connectivity
  /// fallback.  Default off preserves the branch-only model.
  bool include_switch_faults{false};

  /// Use a DC LinDistFlow power flow for the DC subnetwork (per-bus voltage
  /// bounds v in [vmin^2, vmax^2], resistive branch drop v_j = v_i - 2 r P, and
  /// per-branch thermal limits), coupled to the AC MILP through per-component VSC
  /// transfer budgets.  Default **on**: hybrid runs are physics-based by default,
  /// and the aggregate capacity fallback is used automatically only if the DC LP
  /// fails to solve.  Set false to force the legacy capacity-only fallback.
  bool include_dc_power_flow{true};

  /// Evaluate independent contingencies concurrently. The automatic worker
  /// count is conservatively capped by the implementation to limit the memory
  /// footprint of multiple simultaneous MILP models.
  bool enable_parallel{true};
  int parallel_threads{0};

  /// Re-solve the repair-window continuous model with the accepted Stage-2
  /// switch binaries fixed. Disabled by default because the fault remains out
  /// and all physical constraints/demand are identical, so the accepted
  /// Stage-2 solution is already an exact feasible optimum for Stage 3.
  bool revalidate_stage3_plan{false};

  /// Session-level protection definitions. The staged evaluator validates and
  /// resolves stable component references against the submitted system before
  /// generating mutually exclusive protection-conditioned contingencies.
  ReliabilityConfiguration reliability_configuration{};
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

#pragma once

/// reliability/failure_mode.hpp
/// ============================
/// Failure-mode-centric reliability model (per the rewritten
/// docs/reliability_assessment_code_review.md).
///
/// A rich component is NOT represented by one reliability tuple.  It expands
/// into one or more *failure modes*, each carrying its own activation class
/// (passive vs active-on-demand), cause class (physical / cyber / protection /
/// …), mapped network consequence, and resolved reliability parameters.
///
/// Dependency direction (no upward dependencies):
///   rich component -> component catalog -> failure-mode catalog
///     -> parameter resolver -> consequence operator -> consequence engine
///     -> metrics -> GUI/API.

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Layer 1: Component identity
// ═══════════════════════════════════════════════════════════════════════

/// Stable reliability component kind across AC, DC, and hybrid domains.
enum class ReliabilityComponentKind {
  ACGenerator,
  ACBranch,
  ACLoad,
  ACBusLoad,
  ACStaticGenerator,
  ACRenewableGenerator,
  ACStorage,
  ACPVSystem,
  ACTransformer2W,
  ACTransformer3W,
  ACSwitch,
  ACCircuitBreaker,
  ExternalGrid,
  DCBusLoad,
  DCBranch,
  DCLoad,
  DCStaticGenerator,
  DCStaticGeneratorAC,
  DCDCConverter,
  DCCircuitBreaker,
  DCStorage,
  DCPVArray,
  VSCConverter,
  EnergyRouter,
  EnergyRouterPort,
  Microgrid,
  MobileStorage,
  VirtualPowerPlant,
  FlexibleLoad,
  AsymmetricLoad,
  Shunt,
  Charger,
  ChargingStation,
  AsynchronousMotor,
  Unknown
};

/// Stable reference to a rich component.  The GUI/exported results should use
/// `stable_id`, not only the vector position.
struct ComponentRef {
  ReliabilityComponentKind kind{ReliabilityComponentKind::Unknown};
  int element_index{0};        ///< 0-based position within its container vector
  std::string element_name;    ///< business label, may be empty
  std::string domain;          ///< "AC", "DC", or "Hybrid"
  std::string stable_id;       ///< e.g. "ac_branch:5" or "vsc:VSC1"
};

// ═══════════════════════════════════════════════════════════════════════
// Layer 2: Failure-mode taxonomy
// ═══════════════════════════════════════════════════════════════════════

/// How a failure mode is activated.
enum class FailureActivation {
  Passive,        ///< occurs while simply in service (time-based hazard)
  ActiveOnDemand  ///< occurs when the component is required to act
};

/// Root cause class of a failure mode.
enum class FailureCause {
  Physical,
  CyberControl,
  Communication,
  Measurement,
  ProtectionLogic,
  HumanOperation,
  Scheduled
};

/// Network/control consequence kind a failure mode maps to.
enum class FailureConsequenceKind {
  ForcedOutage,
  Derating,
  StuckOpen,
  StuckClosed,
  FailToOpen,
  FailToClose,
  FailToTrip,
  NuisanceTrip,
  ControlUnavailable,
  SetpointFrozen,
  MeasurementBias,
  CommunicationLoss,
  GridFormingUnavailable,
  ProtectionZoneTrip
};

/// Identity of a single failure mode of a component.
struct FailureModeRef {
  ComponentRef component;
  std::string mode_id;        ///< stable id, e.g. "vsc:VSC1/grid_forming_lost"
  std::string display_name;   ///< human-readable label
  FailureActivation activation{FailureActivation::Passive};
  FailureCause cause{FailureCause::Physical};
  FailureConsequenceKind consequence{FailureConsequenceKind::ForcedOutage};
};

/// A failure mode plus its resolved reliability parameters and stage durations.
struct FailureModeReliability {
  FailureModeRef ref;
  ReliabilityParams params;                 ///< resolved by resolve_reliability_params
  double probability_given_initiated{1.0};  ///< q_m: P(consequence | initiated)
  double demand_frequency_per_year{0.0};    ///< nu_d for active modes
  double probability_per_demand{0.0};       ///< p_d for active modes
  double isolation_hr{0.0};                 ///< tau_iso (stage 1)
  double switching_hr{0.0};                 ///< tau_sw  (stage 2)
  double repair_hr{0.0};                    ///< tau_rep (stage 3, physical)
  double cyber_recovery_hr{0.0};            ///< recovery time for cyber modes
  double residual_capacity_factor{0.5};     ///< derating: surviving capacity fraction (0..1]
};

// ═══════════════════════════════════════════════════════════════════════
// String helpers (stable lower_snake_case tokens for API/JSON/GUI)
// ═══════════════════════════════════════════════════════════════════════
std::string to_string(ReliabilityComponentKind k);
std::string to_string(FailureActivation a);
std::string to_string(FailureCause c);
std::string to_string(FailureConsequenceKind c);

// ═══════════════════════════════════════════════════════════════════════
// Layer 2 catalog: build failure modes for every rich component
// ═══════════════════════════════════════════════════════════════════════

/// Filters that restrict which failure modes the catalog emits as enabled.
struct FailureModeCatalogOptions {
  // Activation filters (empty = all).
  bool include_passive{true};
  bool include_active_on_demand{true};

  // Cause filters.
  bool include_physical{true};
  bool include_cyber_control{true};
  bool include_communication{true};
  bool include_measurement{true};
  bool include_protection_logic{true};
  bool include_human_operation{true};
  bool include_scheduled{false};

  // Component-kind inclusion: when empty, all kinds are included.
  std::vector<ReliabilityComponentKind> include_kinds;

  // When true, only currently in-service components contribute modes.
  bool only_in_service{true};

  // Default switching/isolation times used when a mode does not specify them.
  double default_isolation_hr{0.5};
  double default_switching_hr{0.5};
};

/// One catalog entry: a mode plus enable/support diagnostics.
struct FailureModeCatalogEntry {
  FailureModeReliability mode;
  bool enabled{true};
  bool supported_by_selected_consequence_model{true};
  std::string disabled_reason;
  std::string unsupported_reason;
};

/// Coverage summary of the generated failure-mode catalog (for GUI/API).
struct FailureModeCoverage {
  int components_total{0};
  int modes_total{0};
  int modes_enabled{0};
  int modes_disabled{0};
  int modes_unsupported{0};
  int modes_case_data{0};
  int modes_template_or_default{0};
  int modes_missing_data{0};
  int modes_active{0};
  int modes_passive{0};
  int modes_physical{0};
  int modes_cyber_control{0};
  int modes_protection_logic{0};
};

/// Build the failure-mode catalog for the whole system.  Every rich component
/// is expanded into its default failure modes (subject to the option filters),
/// and each mode's reliability parameters are resolved with `data_policy`.
std::vector<FailureModeCatalogEntry> build_failure_mode_catalog(
    const HybridPowerSystem& sys,
    const FailureModeCatalogOptions& options,
    const ReliabilityDataPolicy& data_policy);

/// Summarize a catalog into a coverage record.
FailureModeCoverage summarize_failure_mode_coverage(
    const std::vector<FailureModeCatalogEntry>& catalog);

// ═══════════════════════════════════════════════════════════════════════
// Layer 4: Consequence operator  Phi_m  (failure mode -> network mutation)
// ═══════════════════════════════════════════════════════════════════════

/// Which physical/control effects the selected consequence engine can model.
/// Modes whose effect is outside these capabilities are reported as
/// "not modelled by selected analysis" rather than silently ignored.
struct ConsequenceModelCapabilities {
  bool supports_forced_outage{true};       ///< in-service topology removal
  bool supports_derating{true};            ///< capacity reduction
  bool supports_control_unavailable{true}; ///< dispatch/controllability removal
  bool supports_topology_restoration{false};   ///< switch/breaker restoration actions
  bool supports_protection_modeling{false};     ///< fail-to-trip / zone expansion
  bool supports_observation_cyber{false};       ///< measurement/communication only
};

/// The category a mutation belongs to (mirrors the doc's six mutation classes).
enum class MutationCategory {
  Topology,
  Capacity,
  Control,
  Protection,
  Observation,
  Restoration
};

/// A concrete mutation applied to the network/control problem.
enum class MutationKind {
  ForceOutOfService,        ///< topology: set in_service = false
  ForceLoadShed,            ///< load: interrupt this load point (counts as shed)
  ProtectionZoneExpansion,  ///< protection: breaker fail-to-trip expands the outage
  ForceClosedNoIsolation,   ///< topology: device cannot isolate (stuck closed)
  CapacityDerate,           ///< capacity: multiply rating/pmax by `factor`
  RemoveControllability,    ///< control: fix/freeze setpoint (non-dispatchable)
  RemoveGridForming,        ///< control: island voltage-source capability lost
  RestorationActionForbidden,  ///< restoration: a switch/close action is blocked
  ObservationDegraded       ///< observation: measurement/communication only
};

/// One network/control mutation produced by a failure mode.
struct ConsequenceMutation {
  MutationCategory category{MutationCategory::Topology};
  MutationKind kind{MutationKind::ForceOutOfService};
  ReliabilityComponentKind target_kind{ReliabilityComponentKind::Unknown};
  int target_index{0};
  double factor{1.0};        ///< capacity multiplier for CapacityDerate
  std::string note;
};

/// Consequence patch: the modified network/control problem `G_m = Phi_m(G_0,x_m)`
/// with full provenance from rich component -> mode -> mutations.
struct ConsequencePatch {
  FailureModeRef mode;
  std::vector<ConsequenceMutation> mutations;
  std::vector<std::string> affected_canonical_ids;
  std::vector<std::string> warnings;
  /// True when at least one mutation can be represented by the selected engine.
  bool representable_by_selected_model{true};
  std::string unsupported_reason;
};

std::string to_string(MutationKind k);
std::string to_string(MutationCategory c);

/// Build the consequence patch for one failure mode under the selected engine
/// capabilities (code-review Layer 4).  Maps the mode's FailureConsequenceKind
/// to one or more mutations and records whether the engine can represent them.
ConsequencePatch build_consequence_patch(
    const HybridPowerSystem& sys,
    const FailureModeReliability& mode,
    const ConsequenceModelCapabilities& capabilities);

/// Apply a patch to a COPY of `sys` and return the mutated system for the
/// representable mutations.  Non-representable mutations are skipped (their
/// effect is reported via the patch/result, not silently applied).
HybridPowerSystem apply_consequence_patch(const HybridPowerSystem& sys,
                                          const ConsequencePatch& patch);

/// Compose multiple patches on the same base system using the documented
/// precedence (hard outage dominates derating/control; forced-open and
/// forced-closed on the same element is a hard conflict).  Used by
/// multi-mode (N-k / Monte-Carlo) studies.
ConsequencePatch compose_consequence_patches(
    const std::vector<ConsequencePatch>& patches);

// ═══════════════════════════════════════════════════════════════════════
// Layer 5/6: Deterministic failure-mode FMEA engine + unified result
// ═══════════════════════════════════════════════════════════════════════

/// Per-failure-mode contingency result (one row per evaluated mode).
struct FailureModeContingency {
  FailureModeRef ref;
  std::string data_source;          ///< case / template / default / missing
  bool supported{true};             ///< representable by the selected engine
  std::string unsupported_reason;
  double frequency_per_year{0.0};   ///< f_m (lambda passive / nu*p active)
  double duration_hr{0.0};          ///< iso + sw + repair (event duration)
  double total_shed_mw{0.0};
  double eens_contribution{0.0};    ///< f_m * duration * shed
  double lole_contribution{0.0};    ///< f_m * duration * I(shed>eps)
  bool causes_loss{false};
  std::vector<double> nodal_shed_mw;
};

/// Options for the deterministic failure-mode FMEA engine.
struct FailureModeFMEAOptions {
  FailureModeCatalogOptions catalog{};
  ReliabilityDataPolicy data_policy{};
  ConsequenceModelCapabilities capabilities{};
  double load_scale_factor{1.0};
  double curtail_threshold_mw{0.01};
  bool verbose{false};
  /// Highest simultaneous-failure order to enumerate.  1 = single-mode FMEA
  /// (default).  2 = also enumerate pairwise co-failures (N-2) of supported
  /// modes on distinct components, composed via compose_consequence_patches and
  /// weighted by the independent second-order overlap probability U_i*U_j.
  int max_order{1};
  /// Skip a co-failure pair whose joint unavailability U_i*U_j is below this
  /// (keeps the O(M^2) enumeration tractable by dropping negligible overlaps).
  double min_pair_unavailability{1e-10};
  /// Hard cap on the number of pair evaluations (safety bound for large cases).
  int max_pairs_evaluated{50000};
  /// Parallel evaluation of independent modes / co-failure pairs.
  bool enable_parallel{true};
  int parallel_threads{0};  ///< 0 = hardware_concurrency()
};

/// One enumerated second-order (co-failure) state: two simultaneously-down
/// modes on distinct components, weighted by the independent overlap.
struct FailureModeCoContingency {
  FailureModeRef mode_a;
  FailureModeRef mode_b;
  double joint_frequency_per_year{0.0};  ///< overlap rate f_i*f_j*(d_i+d_j)/8760
  double joint_unavailability{0.0};      ///< U_i*U_j (both simultaneously down)
  double duration_hr{0.0};               ///< overlap mean duration d_i*d_j/(d_i+d_j)
  double total_shed_mw{0.0};             ///< S_ij with both modes applied
  double eens_contribution{0.0};         ///< U_i*U_j*8760*S_ij
  double lole_contribution{0.0};         ///< U_i*U_j*8760 (if S_ij>eps)
  bool causes_loss{false};
};

/// Aggregate result of a deterministic failure-mode FMEA.
struct FailureModeFMEAResult {
  std::string method{"failure_mode_fmea"};
  std::string model_scope;
  std::string model_limitations;
  double eens_mwh_yr{0.0};
  double edns_mw{0.0};
  double lole_hr_yr{0.0};
  double lolf_occ_yr{0.0};
  DistributionIndices distribution_idx;
  std::vector<double> nodal_eens_mwh_yr;
  std::vector<FailureModeContingency> contingencies;
  std::vector<FailureModeCoContingency> co_contingencies;  ///< N-2 (max_order>=2)
  int n_pairs_evaluated{0};                                ///< co-failure solves done
  FailureModeCoverage coverage;
  ReliabilityDataQuality data_quality;
  std::vector<std::string> warnings;
  bool parallel_effective{false};
  int parallel_workers{1};
  std::string parallel_mode{"serial"};
};

/// Run a deterministic failure-mode enumeration: build the catalog, map every
/// enabled mode to a consequence patch, evaluate the resulting network state
/// with the existing AC/DC shed evaluator, and accumulate frequency-weighted
/// reliability metrics (per the frozen mathematical spec).
FailureModeFMEAResult run_failure_mode_fmea(
    const HybridPowerSystem& sys,
    const FailureModeFMEAOptions& options);



}  // namespace hacdcpf::analysis

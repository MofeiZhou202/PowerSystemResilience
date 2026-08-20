#pragma once

#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/util/parallel_execution.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Unified Reliability Parameter Semantics  (code-review Finding 1 & 2)
// ═══════════════════════════════════════════════════════════════════════
// A single resolver converts the heterogeneous component reliability fields
// (ACBranch::failure_rate, Generator::forced_outage_rate, MTBF/MTTR pairs,
// VSC/Storage FOR, …) into one canonical parameter set so that every method
// (NSQ MC, SEQ MC, FMEA, three-stage) consumes identical lambda/repair/
// unavailability for identical components.

/// How a method should treat components whose case data is missing.
enum class ReliabilityDefaultPolicy {
  /// Use only the data present on the case.  Missing data is reported, never
  /// invented.  This is the honest GUI default.
  StrictCaseDataOnly,
  /// Fill ONLY missing fields from a named default/template library; existing
  /// case values are preserved and flagged as case-sourced.
  UseNamedTemplateForMissingOnly,
  /// Overwrite every component with the named template (legacy behaviour of
  /// apply_ieee24_reliability_data / apply_comprehensive_reliability_data).
  OverwriteWithNamedTemplate
};

/// Convention for interpreting a legacy `mtbf_hours` field.  MTBF is ambiguous
/// in industry (mean operating time to failure vs. full failure-repair cycle
/// time), so the resolver records which interpretation was applied.
enum class MtbfConvention {
  MtbfAsMttf,       ///< mtbf_hours is mean time TO failure: lambda = H / mtbf
  MtbfAsCycleTime,  ///< mtbf_hours is the cycle time: MTTF = max(mtbf - mttr, 0)
  Unspecified       ///< not declared; strict mode blocks, compat assumes MtbfAsMttf
};

/// Basis of a supplied `failure_rate_per_year` value.
enum class FailureRateBasis {
  /// Conditional failure intensity while the component is operating.
  OperatingTime,
  /// Observed failures per calendar/reporting year, including downtime.
  CalendarTime
};

/// Policy object threaded through every reliability method.
struct ReliabilityDataPolicy {
  ReliabilityDefaultPolicy default_policy{
      ReliabilityDefaultPolicy::UseNamedTemplateForMissingOnly};
  std::string template_name;                 // "ieee-rts-24", "comprehensive", …
  bool fail_on_missing_required_data{false}; // strict mode hard-stop
  bool report_defaulted_components{true};
  /// Reporting hours per year (H).  Normally 8760; some studies use 8736.
  double hours_per_year{8760.0};
  /// How to interpret legacy `mtbf_hours` fields (default = mean-time-to-failure).
  MtbfConvention mtbf_convention{MtbfConvention::MtbfAsMttf};
  /// How to interpret `failure_rate_per_year`.  The historical contract is an
  /// operating-time intensity; calendar observations require the explicit
  /// CalendarTime selection so the resolver can undo downtime exposure.
  FailureRateBasis failure_rate_basis{FailureRateBasis::OperatingTime};
};

/// Raw failure-mode reliability fields as stored on the model / case.  Any
/// field left at its zero/false default is treated as "not provided".
struct ReliabilityRawFields {
  // ── Passive (time-based) inputs ──
  double failure_rate_per_year{0.0};  // ACBranch::failure_rate (occ/yr)
  double mttr_hr{0.0};                // ACBranch/VSC/Storage repair time (hr)
  double mtbf_hours{0.0};            // legacy MTBF (hr) — see MtbfConvention
  double mttr_hours{0.0};            // transformer / DC component MTTR (hr)
  double mttf_hours{0.0};            // explicit mean time to failure (hr)
  double forced_outage_rate{0.0};   // Generator/VSC/Storage FOR (steady U)
  // ── Active-on-demand inputs ──
  bool   is_active{false};               // true if this is an active failure mode
  double probability_per_demand{0.0};    // p_d (probability of failure per demand)
  double demand_frequency_per_year{0.0}; // nu_d (demand events/yr)
  /// True when the active params above are catalog template defaults (not from
  /// the case/model).  Under StrictCaseDataOnly these resolve to "missing"
  /// rather than masquerading as case data (data-policy honesty).
  bool   active_params_are_template{false};
  // ── Cyber/control recovery time (used instead of physical repair when set) ──
  double cyber_recovery_hr{0.0};
};

/// Canonical resolved reliability parameters (failure-mode level).
struct ReliabilityParams {
  bool has_data{false};            // true if the case provided usable data
  bool used_default{false};        // true if a default/template filled gaps
  std::string data_source;         // "case" | "template" | "default" | "missing"
  double lambda_per_year{0.0};     // operating-time failure intensity (occ/up-year)
  double calendar_frequency_per_year{0.0}; // observed/expected events per calendar year
  double repair_hr{0.0};           // mean repair / recovery duration (hr)
  double unavailability{0.0};      // steady-state forced unavailability
  double mttf_hr{0.0};             // mean time to failure (hr)
  // ── Active-on-demand outputs ──
  bool   is_active{false};
  double probability_per_demand{0.0};
  double demand_frequency_per_year{0.0};
  double lambda_active_per_year{0.0};   // nu_d * p_d (equivalent annual freq)
  // ── Cyber/control recovery time, when applicable ──
  double cyber_recovery_hr{0.0};
  /// Which mtbf interpretation was actually applied (Unspecified if none used).
  MtbfConvention mtbf_convention_applied{MtbfConvention::Unspecified};
  std::vector<std::string> warnings;
};

/// Per-method data-quality summary surfaced to the API/GUI (Finding 4).
struct ReliabilityDataQuality {
  int components_total{0};
  int components_with_reliability_data{0};
  int components_defaulted{0};
  std::vector<std::string> missing_required_data;  // component display names
};

/// Single resolver used by every reliability method.  Applies the conversion
/// rules from the code review's "Reliability Parameter Semantics" section.
///
/// `default_lambda_per_year` / `default_repair_hr` are the named-template
/// fallback values for this component kind; they are only consulted when the
/// policy permits defaulting and the case data is incomplete.
ReliabilityParams resolve_reliability_params(
    const ReliabilityRawFields& raw,
    const ReliabilityDataPolicy& policy,
    double default_lambda_per_year = 0.0,
    double default_repair_hr = 0.0);

// ═══════════════════════════════════════════════════════════════════════
// Load Profile for Sequential Monte Carlo
// ═══════════════════════════════════════════════════════════════════════
struct LoadProfile {
  std::vector<double> factors;  // hourly load scaling factors [0, 1]

  // Optional, time-invariant spatial multipliers. Entries are positional and
  // align with the corresponding component vectors in HybridPowerSystem.
  // Missing entries default to 1.0, so the common temporal curve remains
  // backward compatible while individual load points can be weighted.
  std::vector<double> ac_bus_factors;
  std::vector<double> ac_load_factors;
  std::vector<double> dc_bus_factors;
  std::vector<double> dc_load_factors;
};

/// Build IEEE RTS-24 8736-hour annual load profile.
/// Load(t) = Weekly(w) * Daily(d) * Hourly(h, season, daytype)
LoadProfile build_ieee_rts24_load_profile(int hours_per_year = 8736);

// ═══════════════════════════════════════════════════════════════════════
// Reliability Assessment Options
// ═══════════════════════════════════════════════════════════════════════
struct ReliabilityOptions {
  int max_iterations{10000};          // NSQ: max samples, SEQ: max years
  double cov_threshold{0.05};         // Convergence criterion (CoV)
  double curtail_threshold_mw{0.01};  // MW threshold for a curtailment event
  unsigned int seed{0};               // Random seed (0 = non-deterministic)
  bool verbose{false};

  // Load scaling factor to stress the system (1.0 = no scaling)
  // Use > 1.0 to reduce reserve margin and increase loss probability
  double load_scale_factor{1.0};

  // Sequential-specific
  int hours_per_year{8736};

  // Reliability data policy (Finding 1 & 2).  Controls how missing component
  // reliability data is treated when assembling component unavailabilities.
  ReliabilityDataPolicy data_policy{};

  // DC-OPF solver options for state evaluation
  opf::DCOPFOptions opf_options{};

  // Progress callback: (iteration, eens, cov) → return false to cancel
  std::function<bool(int, double, double)> progress_callback;

  // ─── Advanced Options ───
  // Tail risk analysis
  bool compute_tail_risk{true};       // Compute VaR/CVaR metrics
  double var_confidence{0.95};        // VaR/CVaR confidence level (0.95 = 95%)

  // Distribution system indices (requires customer data in loads)
  bool compute_distribution_indices{false};  // Compute SAIFI/SAIDI/ASAI

  // Independent-Bernoulli odds twisting for non-sequential MC. Sequential MC
  // rejects this option because a path-space likelihood ratio is required.
  bool use_importance_sampling{false};
  double importance_lambda{2.0};      // Failure-odds twisting factor (> 0)

  // Parallel evaluation of independent sampled states / simulated years.
  bool enable_parallel{true};
  int parallel_threads{0};            // 0 = hardware_concurrency()
};

// ═══════════════════════════════════════════════════════════════════════
// Tail Risk Metrics (Advanced)
// ═══════════════════════════════════════════════════════════════════════
struct TailRiskMetrics {
  // Value at Risk: minimum loss exceeded with (1-confidence) probability
  double eens_var{0.0};       // VaR of EENS at confidence level
  double lole_var{0.0};       // VaR of LOLE at confidence level
  
  // Conditional Value at Risk (Expected Shortfall): empirical upper-tail mean
  // with fractional weighting of probability mass at the VaR atom.
  double eens_cvar{0.0};      // CVaR of EENS
  double lole_cvar{0.0};      // CVaR of LOLE
  
  // Full distribution data for plotting
  std::vector<double> eens_distribution;  // Per-sample/year EENS values
  std::vector<double> lole_distribution;  // Per-sample/year LOLE values
  
  // Percentiles (5th, 25th, 50th, 75th, 95th)
  std::vector<double> eens_percentiles;
  std::vector<double> lole_percentiles;
};

// ═══════════════════════════════════════════════════════════════════════
// Distribution System Reliability Indices (Advanced)
// ═══════════════════════════════════════════════════════════════════════
struct DistributionIndices {
  // IEEE Std 1366-2012 indices
  double saifi{0.0};   // System Average Interruption Frequency Index (int/cust/yr)
  double saidi{0.0};   // System Average Interruption Duration Index (hr/cust/yr)
  double caidi{0.0};   // Customer Average Interruption Duration Index (hr/int)
  double asai{0.0};    // Average Service Availability Index (fraction)
  double asui{0.0};    // Average Service Unavailability Index (fraction)
  
  // Per-node indices (indexed by bus)
  std::vector<double> nodal_cif;   // Customer Interruption Frequency
  std::vector<double> nodal_cid;   // Customer Interruption Duration
};

// ═══════════════════════════════════════════════════════════════════════
// Frequency & Duration Results (Advanced)
// ═══════════════════════════════════════════════════════════════════════
struct FrequencyDurationResult {
  // Capacity Outage Probability Table (COPT)
  std::vector<double> capacity_outage_levels;  // MW
  std::vector<double> state_probability;       // P(outage == X)
  std::vector<double> cumulative_probability;  // P(outage >= X)
  std::vector<double> cumulative_frequency;    // F(outage >= X) (occ/yr)
  
  // System-level F&D indices
  double lolp{0.0};             // Loss of Load Probability
  double lole_fd{0.0};          // LOLE from F&D method (hr/yr)
  double lolf_fd{0.0};          // LOLF from F&D method (occ/yr)
  double lold{0.0};             // Loss of Load Duration = LOLE/LOLF (hr/occ)
  bool probability_valid{false};
  bool frequency_valid{false};
  bool exact_capacity_states{false};
  std::vector<std::string> warnings;
};

/// Independent two-state component used by analytical F&D reduction.
/// `failure_rate_per_year` is the up-to-down transition intensity lambda;
/// `repair_time_hr` is 1/mu.  `unavailability` must be consistent with those
/// rates when all three are supplied.
struct TwoStateReliabilityComponent {
  double failure_rate_per_year{0.0};
  double repair_time_hr{0.0};
  double unavailability{0.0};
};

/// Exact steady-state frequency/duration equivalent of a coherent series or
/// parallel block under independent two-state component assumptions.
struct FrequencyDurationEquivalent {
  double availability{1.0};
  double unavailability{0.0};
  double failure_frequency_per_year{0.0};
  double mean_failure_duration_hr{0.0};
};

FrequencyDurationEquivalent reduce_series_frequency_duration(
    const std::vector<TwoStateReliabilityComponent>& components,
    double hours_per_year = 8760.0);

FrequencyDurationEquivalent reduce_parallel_frequency_duration(
    const std::vector<TwoStateReliabilityComponent>& components,
    double hours_per_year = 8760.0);

/// IEC 61508 low-demand dangerous-undetected failure model with periodic,
/// complete proof testing.  The exact cycle-average is
/// 1 - (1-exp(-lambda_DU*T_I))/(lambda_DU*T_I); lambda*T/2 is also returned
/// so callers can quantify the small-rate approximation error.
struct LowDemandPFDResult {
  double pfd_average{0.0};
  double first_order_pfd_average{0.0};
  double approximation_relative_error{0.0};
};

LowDemandPFDResult compute_low_demand_pfd(
    double dangerous_undetected_rate_per_hour,
    double proof_test_interval_hr);

/// Independent information-service component.  Availability and packet
/// delivery are separate probabilities but share the same component identity,
/// so a component used by multiple paths is counted once in joint events.
struct InformationServiceComponent {
  std::string id;
  double availability{1.0};
  double packet_delivery_probability{1.0};
  double latency_ms{0.0};
  double jitter_ms{0.0};
};

struct InformationServicePath {
  std::vector<size_t> component_indices;
};

struct InformationFunctionDefinition {
  std::string name;
  std::vector<InformationServicePath> alternative_paths;
  double max_latency_ms{0.0};  ///< <= 0 disables this QoS limit
  double max_jitter_ms{0.0};   ///< <= 0 disables this QoS limit
  double min_packet_delivery_probability{0.0};
};

struct InformationFunctionReliabilityResult {
  double availability{0.0};
  size_t valid_path_count{0};
  /// Minimal component-index sets whose simultaneous failure disables every
  /// QoS-valid success path.  Empty when an empty (failure-independent) path
  /// exists or when no valid success path exists.
  std::vector<std::vector<size_t>> minimal_cut_sets;
};

/// Exact independent-component structure-function evaluation.  Alternative
/// paths may share components; inclusion-exclusion counts each shared event
/// once.  Throws when the reduced path/component count exceeds the documented
/// exact-enumeration bound instead of silently assuming path independence.
InformationFunctionReliabilityResult evaluate_information_function_reliability(
    const std::vector<InformationServiceComponent>& components,
    const InformationFunctionDefinition& function);

/// Probability that every requested function is simultaneously available in
/// the same component state.  This is not the product of marginal function
/// availabilities when functions share infrastructure.
InformationFunctionReliabilityResult evaluate_joint_information_reliability(
    const std::vector<InformationServiceComponent>& components,
    const std::vector<InformationFunctionDefinition>& functions);

/// Independent physical component used by an explicit success-path network.
struct PhysicalReliabilityComponent {
  std::string stable_id;
  double availability{1.0};
};

struct PhysicalSuccessPath {
  std::vector<size_t> component_indices;
};

struct PhysicalNetworkReliabilityResult {
  double availability{0.0};
  double loss_probability{1.0};
  size_t reduced_success_path_count{0};
  std::vector<std::vector<size_t>> minimal_cut_set_indices;
  std::vector<std::vector<std::string>> minimal_cut_set_stable_ids;
  bool exact_independent_path_model{false};
};

/// Exact coherent physical-network evaluation from explicit success paths.
/// Shared components are counted once by inclusion-exclusion; minimal cut sets
/// are the minimal hitting sets of the reduced path family.
PhysicalNetworkReliabilityResult evaluate_physical_network_reliability(
    const std::vector<PhysicalReliabilityComponent>& components,
    const std::vector<PhysicalSuccessPath>& success_paths);

// ═══════════════════════════════════════════════════════════════════════
// Reliability Assessment Results
// ═══════════════════════════════════════════════════════════════════════
struct ReliabilityResult {
  bool converged{false};
  int iterations_used{0};
  double final_cov{0.0};

  // System-level reliability indices
  double eens_mwh_yr{0.0};   // Expected Energy Not Supplied (MWh/yr)
  double edns_mw{0.0};       // Expected Demand Not Supplied (MW)
  double baseline_eens_mwh_yr{0.0};   // N-0 curtailment annualized (MWh/yr)
  double baseline_edns_mw{0.0};       // N-0 curtailment (MW)
  double incremental_eens_mwh_yr{0.0}; // Outage-state excess above N-0 (MWh/yr)
  double incremental_edns_mw{0.0};     // Expected outage-state excess above N-0 (MW)
  // State-evaluation decomposition. These counters/energies are reported in
  // the same sampling measure as eens_mwh_yr (including likelihood weights for
  // importance sampling). They make conservative solver fallbacks auditable.
  long long evaluated_state_count{0};
  long long opf_failed_state_count{0};
  long long dead_island_state_count{0};
  double opf_failed_probability{0.0};
  double dead_island_probability{0.0};
  double opf_failed_eens_mwh_yr{0.0};
  double dead_island_eens_mwh_yr{0.0};
  double opf_shed_eens_mwh_yr{0.0};
  double lole_hr_yr{0.0};    // Loss of Load Expectation (hr/yr)
  double lolf_occ_yr{0.0};   // Loss of Load Frequency (occ/yr, SEQ only)
  double plc{0.0};           // Probability of Load Curtailment

  // Importance-sampling diagnostics (non-sequential MC only).
  bool importance_sampling_used{false};
  double importance_twisting_factor{1.0};
  double importance_effective_sample_size{0.0};
  double importance_mean_likelihood_ratio{1.0};

  // Convergence history
  std::vector<double> eens_history;
  std::vector<double> cov_history;

  // Per-bus EENS (MWh/yr)
  std::vector<double> nodal_eens_mwh_yr;

  // Weak point detection.
  // F16: these are CO-OCCURRENCE / attribution metrics (the share of loss-state
  // shed observed while a component is down), NOT a Birnbaum marginal importance
  // (d EENS / d U_c).  In a multi-failure state the whole state shed is attributed
  // to every down component, so contributions can sum to more than 100%.  Read
  // them as a rank, not as marginal risk.
	  struct ComponentImportance {
	    int index;            // within-type index (0-based within component_type group)
	    bool is_generator;    // true = generator, false = other
	    double importance;    // primary displayed rank metric: loss-weighted risk share
	    double conditional_down_given_loss{0.0};  // P(component down | system failure)
	    double loss_weighted_risk{0.0};           // share of loss-state curtailed MW
	    double associated_eens_mwh_yr{0.0};       // co-outage-associated EENS
	    std::string component_type;  // e.g. "Generator", "ACBranch", "VSCConverter"
	    std::string component_name;  // e.g. "Generator[2]", "ACBranch[5]"
	    size_t global_state_index{0}; // raw index into the flat component state vector
	  };
  std::vector<ComponentImportance> critical_components;

  // Per-year results (sequential MC only)
  std::vector<double> annual_eens;
  std::vector<double> annual_lole;
  std::vector<double> annual_lolf;

  // ─── Modelling Scope Limitations ───
  /// Non-empty when the evaluation could not model all components of the
  /// submitted system.  Callers should treat metrics as approximate when
  /// this field is set.  Example: "DC loads not modelled (AC-only OPF)".
  std::string model_limitations;

  /// Structured model-capability declaration. Pure AC systems use
  /// "ac-only-dcopf". Systems with represented DC/VSC assets use
  /// "hybrid-acdc-network-lp", including AC/DC active-power balance, DC load
  /// shedding, bounded branch transfer, and bounded converter transfer.
  /// Neither scope certifies nonlinear AC voltage/reactive feasibility; callers
  /// must inspect the validity flags before interpreting the indices.
  std::string model_scope{"ac-only-dcopf"};

  /// Per-feature validity flags so downstream code can branch on what the
  /// evaluator actually modelled, rather than guessing from `model_scope`.
  struct ValidityFlags {
    /// True only if DC load curtailment contributes to EENS/LOLE.
    /// True for the hybrid AC/DC network LP and false for AC-only DC-OPF.
    bool dc_load_curtailment_included{false};
    /// True if VSC/DC branch contingencies affect DC-side power balance.
    /// True for the hybrid AC/DC network LP and false for AC-only DC-OPF.
    bool vsc_dc_power_flow_modelled{false};
    /// True if AC load curtailment is computed via OPF (not all-or-nothing).
    /// True for NSQ/SEQ MC and FMEA paths that call solve_dc_opf.
    bool ac_opf_curtailment{true};
    /// True only when nonlinear AC voltage/reactive feasibility is certified.
    /// Reliability evaluators in this header use DC-OPF or linear hybrid LPs,
    /// so this remains false unless a caller adds a post-solve AC validation.
    bool ac_voltage_reactive_feasibility_certified{false};
  };
  ValidityFlags validity{};

  // Data-quality summary for the resolved reliability parameters (Finding 4).
  ReliabilityDataQuality data_quality;

  // Parallel execution diagnostics.
  bool parallel_effective{false};
  int parallel_workers{1};
  std::string parallel_mode{"serial"};
  hacdcpf::util::ParallelExecutionInfo parallel_execution;

  // ─── Advanced Results ───
  TailRiskMetrics tail_risk;             // VaR/CVaR metrics
  DistributionIndices distribution_idx;  // SAIFI/SAIDI/ASAI (if computed)
};

// ═══════════════════════════════════════════════════════════════════════
// Monte Carlo Reliability Assessment Functions
// ═══════════════════════════════════════════════════════════════════════

/// Non-sequential Monte Carlo reliability assessment.
/// Uses state sampling (Bernoulli trials) and DC-OPF evaluation.
/// Requires: Generator::forced_outage_rate and ACBranch::failure_rate
///           with corresponding mttr_hr fields set.
ReliabilityResult run_nonsequential_mc(
    const HybridPowerSystem& sys,
    const ReliabilityOptions& options = {});

/// Sequential Monte Carlo reliability assessment.
/// Uses chronological simulation with hourly load profile.
/// Requires: Generator::forced_outage_rate/mttr_hr and
///           ACBranch::failure_rate/mttr_hr fields set.
ReliabilityResult run_sequential_mc(
    const HybridPowerSystem& sys,
    const LoadProfile& load_profile,
    const ReliabilityOptions& options = {});

struct ExactReliabilitySensitivityItem {
  size_t global_state_index{0};
  size_t component_position{0};
  /// Public stable component .index, never the vector position.
  int component_index{0};
  std::string component_type;
  std::string component_name;
  double unavailability{0.0};
  double eens_if_forced_down_mwh_yr{0.0};
  double eens_if_forced_up_mwh_yr{0.0};
  double birnbaum_mwh_yr_per_unit_unavailability{0.0};
  double eens_derivative_mwh_yr_per_unit_unavailability{0.0};
  double fussell_vesely{0.0};
};

struct ExactReliabilitySensitivityResult {
  double baseline_eens_mwh_yr{0.0};
  double expected_incremental_eens_mwh_yr{0.0};
  std::vector<ExactReliabilitySensitivityItem> components;
  size_t states_evaluated{0};
  bool exact_independent_binary_model{false};
  std::string model_scope{"exact-independent-binary-state-enumeration"};
};

ExactReliabilitySensitivityResult compute_exact_reliability_sensitivity(
    const HybridPowerSystem& sys,
    const ReliabilityOptions& options = {},
    size_t maximum_components = 20);

/// Apply IEEE RTS-24 reliability data to the existing IEEE-24 system.
/// Sets forced_outage_rate and mttr_hr on generators, and
/// failure_rate and mttr_hr on branches.
void apply_ieee24_reliability_data(HybridPowerSystem& sys);

/// Apply typical distribution-level reliability data to the comprehensive
/// hybrid AC/DC test system. Sets failure rates, MTTR, customer counts,
/// and reliability data on all component types (generators, branches,
/// transformers, static generators, renewables, storage, VSC converters,
/// DC branches, DC-DC converters, PV systems, loads).
void apply_comprehensive_reliability_data(HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// FMEA (Failure-Mode Enumeration Analysis) for Distribution Systems
// ═══════════════════════════════════════════════════════════════════════

/// Optional Level-1 cyber-physical conditioning for staged FMEA.
///
/// The cyber state is reduced to two consequence-equivalent classes:
/// automation available and automation unavailable.  This is the scalar
/// interface-matrix model discussed in the archived design reference
/// docs/archive/theory/cyber_physical_reliability_extension.md;
/// it does not claim to model a communication topology or cyber-node power.
///
/// Class semantics (doc section 4.1):
///  - automation available: FLISR-speed switching, full remote restoration.
///  - automation unavailable: manual restoration.  The crew still performs
///    switch/tie reconfiguration during the repair stage, but remotely
///    dispatched resources (DER promotion, storage, grid-forming VSC support,
///    black start, microgrid islanding) are frozen when
///    `freeze_der_on_automation_loss` is set.
/// When enabled, the class switching times REPLACE the global
/// `FMEAOptions::switching_time_hr` for every contingency.
struct CyberPhysicalFMEAOptions {
  bool enabled{false};
  /// Information/service dimension.  When false, the scalar communication
  /// service factor is bypassed (treated as 1.0), while intelligent local
  /// functions may still be screened independently.
  bool information_enabled{true};
  double automation_availability{0.97};
  double automatic_switching_time_hr{0.05};  // 3 minutes
  /// Clamped to >= automatic_switching_time_hr at evaluation time (manual
  /// restoration cannot beat automatic restoration).
  double manual_switching_time_hr{1.0};
  bool freeze_der_on_automation_loss{true};

  struct AvailabilityOverride {
    std::string component_type;
    int component_index{-1};
    double availability{0.97};
  };
  std::vector<AvailabilityOverride> availability_overrides;

  /// Protection-security contribution from no-fault decision windows.  The
  /// chain is evaluated as nu_window * p_false_alarm * p_channel * p_breaker;
  /// its EENS/LOLE/LOLF is added to the system result instead of being exposed
  /// as an always-zero decomposition field.
  struct ProtectionMisoperationOptions {
    bool enabled{false};
    double no_fault_decision_windows_per_year{0.0};
    double false_trip_probability_per_window{0.0};
    double trip_channel_success_probability{1.0};
    double breaker_success_probability{1.0};
    double disconnected_load_mw{0.0};
    double restoration_duration_hr{0.0};
  } protection_misoperation{};

  /// Level-1+ intelligent-function screening.  Scalar inputs use a factorized
  /// model; joint_states can replace it with explicit joint class dependence.
  /// Neither mode models relay pickup or protection/FRT trajectories.
  /// A failed function is routed to the existing degraded/manual consequence
  /// class, so protection_success_probability is a consequence proxy rather
  /// than an explicit backup-zone calculation.
  struct IntelligentFunctionOptions {
    bool enabled{false};
    double detection_success_probability{0.98};
    double isolation_success_probability{0.97};
    double restoration_decision_valid_probability{0.98};
    double restoration_execution_success_probability{0.98};
    double protection_success_probability{0.995};

    /// Explicit mutually-exclusive joint states.  When non-empty, these states
    /// replace the independent scalar product above and may encode arbitrary
    /// dependence among information, detection, isolation, restoration, and
    /// protection success.  Probabilities must be finite, non-negative, and
    /// sum to one.
    struct JointFunctionState {
      double probability{0.0};
      bool information_service_available{true};
      bool detection_success{true};
      bool isolation_success{true};
      bool restoration_decision_valid{true};
      bool restoration_execution_success{true};
      bool protection_success{true};
    };
    std::vector<JointFunctionState> joint_states;
  } intelligent{};
};

/// Options for FMEA distribution reliability assessment.
struct FMEAOptions {
  double load_scale_factor{1.0};       // Load scaling (1.0 = no scaling)
  double switching_time_hr{0.5};       // Default switching/isolation time (hours)
  double voll{10000.0};               // Value of Lost Load ($/MWh)
  bool verbose{false};

  // When enabled, contingencies are evaluated island-by-island after fault
  // isolation instead of by a single global DC-OPF. This allows island-capable
  // microgrids to remain energized if they retain a local forming source.
  bool enable_microgrid_islanding{true};

  // When enabled, repair-stage evaluation may promote local controllable DERs
  // (for example backup static generators) to serve islanded load after
  // switching actions have isolated the fault.
  bool enable_repair_reconfiguration{true};

  // When enabled, repair-stage evaluation explicitly considers automated AC
  // switch/tie closure actions and selects the topology with the least shed.
  bool enable_switch_reconfiguration{true};

  // Maximum number of switch actions considered simultaneously in the repair
  // stage explicit reconfiguration search.
  int max_repair_switch_actions{2};

  // Maximum number of OPF evaluations in the repair-stage topology search
  // across all candidate action combinations.  When this budget is reached the
  // search stops and the best result found so far is returned with
  // FMEAContingencyDetail::repair_search_truncated set to true.
  // Set to 0 to disable the cap (unlimited — may be slow for large systems).
  int max_repair_opf_calls{200};

  // When enabled, storage is modeled as a dispatchable emergency source during
  // FMEA stage evaluation with stage-specific SOC tracking.
  bool enable_storage_dispatch{true};

  // When enabled, grid-forming VSC converters can anchor an energized island
  // and contribute bounded active-power support in stage evaluation.
  bool enable_grid_forming_vsc_support{true};

  // When enabled, controllable storage inside an islanding-capable microgrid is
  // treated as a black-start / grid-forming candidate for FMEA restoration.
  bool enable_black_start_storage{true};

  // Reliability data policy (Finding 1 & 2): how missing component reliability
  // data is treated when building the contingency catalog.  Default fills only
  // missing fields from built-in per-kind defaults; StrictCaseDataOnly reports
  // missing data instead of inventing values.
  ReliabilityDataPolicy data_policy{};

  // DC-OPF solver options for contingency evaluation
  opf::DCOPFOptions opf_options{};

  // Parallel evaluation of independent contingencies.
  bool enable_parallel{true};
  int parallel_threads{0};            // 0 = hardware_concurrency()

  // Level-1 cyber-physical interface-matrix conditioning. Disabled by default
  // so existing physical-only studies retain their established semantics.
  CyberPhysicalFMEAOptions cyber_physical{};
};

/// Per-contingency detail for FMEA.
struct FMEAContingencyDetail {
  int component_index{0};         // Index into the component type's vector
  std::string component_type;     // "generator", "ac_branch", "dc_branch", "vsc_converter"
  std::string component_name;     // Human-readable name
  double failure_rate{0.0};       // λ_k (occ/yr)

  // Switching stage results
  double tau_sw_hr{0.0};          // Switching/isolation duration (hours)
  double shed_sw_mw{0.0};         // Load shed during switching stage (MW)
  double ens_sw_mwh{0.0};         // Energy not supplied in switching stage (MWh per event)

  // Repair stage results
  double tau_rep_hr{0.0};         // Repair duration (hours)
  double shed_rep_mw{0.0};        // Load shed during repair stage (MW)
  double ens_rep_mwh{0.0};        // Energy not supplied in repair stage (MWh per event)

  // Contribution to annual indices (frequency-weighted)
  double eens_contribution{0.0};  // λ_k * (ens_sw + ens_rep) (MWh/yr)
  double lole_contribution{0.0};  // λ_k * Σ_s τ_{k,s} * I(shed > 0) (hr/yr)
  bool causes_loss_sw{false};     // Does switching stage cause load loss?
  bool causes_loss_rep{false};    // Does repair stage cause load loss?

  // Per-bus shedding for SAIFI/SAIDI computation
  std::vector<double> nodal_shed_sw_mw;   // Per-bus shed (switching)
  std::vector<double> nodal_shed_rep_mw;  // Per-bus shed (repair)

  struct SwitchActionDetail {
    int switch_index{0};
    std::string switch_name;
    std::string switch_type;
    std::string action;  // "trip", "open", or "close"
    int bus_from{0};
    int bus_to{0};
    int sequence_order{0};
    std::string purpose;
    double operation_time_s{0.0};
    bool validated{false};
    std::string validation_message;
  };

  // Explicit repair-stage switch operations selected by the topology search.
  std::vector<SwitchActionDetail> repair_switch_actions;
  bool repair_switch_sequence_valid{true};
  std::string repair_switch_sequence_message;
  bool fault_isolation_explicit{false};
  std::string fault_isolation_message;

  // True if the repair-stage topology search was stopped early by the OPF
  // call budget (max_repair_opf_calls). Results remain valid but may not
  // reflect the globally optimal switching sequence.
  bool repair_search_truncated{false};

  // Level-1 cyber class attribution.  These remain zero for physical-only runs.
  double automation_availability{1.0};
  double tau_sw_automatic_hr{0.0};
  double tau_sw_manual_hr{0.0};
  double shed_sw_automatic_mw{0.0};
  double shed_sw_manual_mw{0.0};
  double shed_rep_automatic_mw{0.0};
  double shed_rep_manual_mw{0.0};
  double eens_perfect_cyber_contribution{0.0};
  double eens_no_automation_contribution{0.0};
  double eens_cyber_duration_increment{0.0};
  double eens_cyber_control_increment{0.0};
};

/// System-level Level-1 cyber-physical attribution and bounding pair.
struct CyberPhysicalReliabilityMetrics {
  bool enabled{false};
  int level{0};
  std::string model_scope{"physical-only"};
  bool information_enabled{false};
  bool intelligent_enabled{false};
  bool independent_factorization{false};
  double information_service_availability{1.0};
  double intelligent_function_success_probability{1.0};
  double effective_automation_probability{1.0};
  double detection_success_probability{1.0};
  double isolation_success_probability{1.0};
  double restoration_decision_valid_probability{1.0};
  double restoration_execution_success_probability{1.0};
  double protection_success_probability{1.0};
  double automation_availability{1.0};
  double automatic_switching_time_hr{0.0};
  double manual_switching_time_hr{0.0};
  double eens_perfect_cyber_mwh_yr{0.0};
  double eens_no_automation_mwh_yr{0.0};
  double eens_adjusted_mwh_yr{0.0};
  double delta_cyber_duration_mwh_yr{0.0};
  double delta_cyber_control_mwh_yr{0.0};
  double delta_protection_misoperation_mwh_yr{0.0};
  double protection_misoperation_frequency_per_year{0.0};
  double protection_misoperation_lole_hr_yr{0.0};
  double protection_misoperation_lolf_occ_yr{0.0};
  double eens_decomposition_residual_mwh_yr{0.0};
  /// eta = (EENS_noAuto - EENS) / (EENS_noAuto - EENS_perfect), clamped [0,1].
  /// With a UNIFORM availability A this equals A by construction; it becomes
  /// informative once per-component availability_overrides differ.
  double automation_efficacy{1.0};
  double saidi_perfect_cyber_hr_cust_yr{0.0};
  double saidi_no_automation_hr_cust_yr{0.0};
  /// Share of SAIDI in EXCESS of the perfect-cyber bound:
  /// (SAIDI - SAIDI_perfect) / SAIDI, clamped [0,1].  This is the "how much of
  /// today's SAIDI is attributable to imperfect automation" reading, not the
  /// class-c2-weighted CID share.
  double cyber_caused_saidi_share{0.0};
};

/// Comprehensive FMEA result for distribution systems.
struct FMEAResult {
  int n_contingencies{0};         // Total N-1 contingencies evaluated
  int n_loss_contingencies{0};    // Contingencies causing any load loss

  // System-level reliability indices (frequency-weighted)
  double eens_mwh_yr{0.0};       // Expected Energy Not Supplied (MWh/yr)
  double edns_mw{0.0};           // Expected Demand Not Supplied (MW)
  double lole_hr_yr{0.0};        // Loss of Load Expectation (hr/yr)
  double lolf_occ_yr{0.0};       // Loss of Load Frequency (occ/yr)

  // Per-bus results
  std::vector<double> nodal_eens_mwh_yr;

  // Distribution indices (customer-based)
  DistributionIndices distribution_idx;

  // Data-quality summary for the resolved reliability parameters (Finding 4).
  ReliabilityDataQuality data_quality;

  // Parallel execution diagnostics.
  bool parallel_effective{false};
  int parallel_workers{1};
  std::string parallel_mode{"serial"};
  hacdcpf::util::ParallelExecutionInfo parallel_execution;

  // Per-contingency details (sorted by EENS contribution descending)
  std::vector<FMEAContingencyDetail> contingencies;

  // Populated when FMEAOptions::cyber_physical.enabled is true.
  CyberPhysicalReliabilityMetrics cyber_physical;

  // Non-empty when evaluation uses simplified physics.
  std::string model_limitations;

  /// Structured model-capability declaration.  Hybrid AC/DC systems use
  /// "hybrid-acdc-network-lp", which optimizes AC/DC branch transfer,
  /// DC load shedding, DC sources, DC/DC converters, and VSC active-power
  /// transfer.  AC-only systems use "ac-only-dcopf".
  std::string model_scope{"ac-only-dcopf"};

  /// Per-feature validity flags for the FMEA result.
  struct ValidityFlags {
    bool dc_load_curtailment_included{false};
    bool vsc_dc_power_flow_modelled{false};
    bool ac_opf_curtailment{true};
    bool ac_voltage_reactive_feasibility_certified{false};
    bool repair_ac_switch_reconfiguration_modelled{false};
    bool repair_dc_side_reconfiguration_modelled{false};
    bool cyber_topology_modelled{false};
    bool restoration_duration_cyber_conditioned{false};
    bool cyber_control_consequence_modelled{false};
    bool cyber_power_coupling_modelled{false};
    bool information_service_conditioned{false};
    bool intelligent_function_probabilities_modelled{false};
    bool joint_class_probability_modelled{false};
    bool protection_logic_modelled{false};
    bool protection_misoperation_modelled{false};
    bool protection_frt_reliability_coupled{false};
  };
  ValidityFlags validity{};
};

// ═══════════════════════════════════════════════════════════════════════
// Advanced Analytical Methods (Non-Simulation Based)
// ═══════════════════════════════════════════════════════════════════════

/// Frequency & Duration method (analytical).
/// Builds an exact-capacity-state COPT recursively and calculates LOLP, LOLE,
/// LOLF without simulation.  "Exact" means no capacity binning; states whose
/// floating-point capacities differ only by round-off are merged.  This is
/// faster than MC but assumes independent two-state generator failures and a
/// constant peak load.
FrequencyDurationResult run_frequency_duration_analysis(
    const HybridPowerSystem& sys,
    double peak_load_mw = 0.0);  // 0 = use sum of bus loads

/// Compute tail risk metrics from a distribution of values.
/// Calculates VaR and CVaR at the specified confidence level.
TailRiskMetrics compute_tail_risk(
    const std::vector<double>& eens_samples,
    const std::vector<double>& lole_samples,
    double confidence = 0.95);

/// Compute distribution system reliability indices.
/// Requires Load::num_customers field to be set for accurate results.
DistributionIndices compute_distribution_indices(
    const HybridPowerSystem& sys,
    const std::vector<double>& nodal_cif,  // interruption freq per bus
    const std::vector<double>& nodal_cid,  // interruption duration per bus
    int hours_per_year = 8760);

/// Scan every reliability-relevant component in the system and summarize how
/// many have usable case reliability data vs. how many would be defaulted or
/// are missing under the given policy.  Used for pre-run diagnostics and the
/// API/GUI data-quality panel (code-review Finding 4).
ReliabilityDataQuality summarize_reliability_data_quality(
    const HybridPowerSystem& sys,
    const ReliabilityDataPolicy& policy = {});

// ═══════════════════════════════════════════════════════════════════════
// FMEA Distribution Reliability Assessment
// ═══════════════════════════════════════════════════════════════════════

/// Deterministic N-1 failure-mode enumeration for distribution systems.
/// Enumerates every single-component outage, evaluates switching and repair
/// stages via DC-OPF, and aggregates frequency-weighted reliability indices.
/// This is an analytical alternative to Monte Carlo that avoids sampling
/// variance and is fast for small-to-medium systems.
FMEAResult run_distribution_fmea(
    const HybridPowerSystem& sys,
    const FMEAOptions& options = {});

// ═══════════════════════════════════════════════════════════════════════
// Failed-network-state shed evaluator (consequence-engine entry point)
// ═══════════════════════════════════════════════════════════════════════

/// Result of evaluating the load shedding of a (possibly failed) network state.
struct NetworkShedResult {
  double total_shed_mw{0.0};
  std::vector<double> nodal_shed_mw;  ///< [AC buses | DC buses] for hybrid systems
  bool is_loss{false};
  std::string model_scope;            ///< "hybrid-acdc-network-lp" | "ac-only-dcopf"
};

/// Evaluate the steady-state minimum load shedding of `sys` AS-IS.  Callers
/// apply component failures or a failure-mode consequence patch to `sys`
/// BEFORE calling.  Reuses the FMEA hybrid AC/DC network LP for hybrid systems
/// and AC-only DC-OPF otherwise — no new solver is introduced.  This is the
/// shared consequence-engine entry point used by the failure-mode FMEA.
NetworkShedResult evaluate_failed_network_state(
    const HybridPowerSystem& sys,
    const FMEAOptions& options = {});

}  // namespace hacdcpf::analysis

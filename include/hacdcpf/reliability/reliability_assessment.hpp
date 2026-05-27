#pragma once

#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Load Profile for Sequential Monte Carlo
// ═══════════════════════════════════════════════════════════════════════
struct LoadProfile {
  std::vector<double> factors;  // hourly load scaling factors [0, 1]
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

  // Importance sampling for variance reduction (experimental)
  bool use_importance_sampling{false};
  double importance_lambda{2.0};      // Importance sampling twisting factor
};

// ═══════════════════════════════════════════════════════════════════════
// Tail Risk Metrics (Advanced)
// ═══════════════════════════════════════════════════════════════════════
struct TailRiskMetrics {
  // Value at Risk: minimum loss exceeded with (1-confidence) probability
  double eens_var{0.0};       // VaR of EENS at confidence level
  double lole_var{0.0};       // VaR of LOLE at confidence level
  
  // Conditional Value at Risk (Expected Shortfall): expected loss given > VaR
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
  std::vector<double> cumulative_probability;  // P(outage >= X)
  std::vector<double> cumulative_frequency;    // F(outage >= X) (occ/yr)
  
  // System-level F&D indices
  double lolp{0.0};             // Loss of Load Probability
  double lole_fd{0.0};          // LOLE from F&D method (hr/yr)
  double lolf_fd{0.0};          // LOLF from F&D method (occ/yr)
  double lold{0.0};             // Loss of Load Duration = LOLE/LOLF (hr/occ)
};

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
  double lole_hr_yr{0.0};    // Loss of Load Expectation (hr/yr)
  double lolf_occ_yr{0.0};   // Loss of Load Frequency (occ/yr, SEQ only)
  double plc{0.0};           // Probability of Load Curtailment

  // Convergence history
  std::vector<double> eens_history;
  std::vector<double> cov_history;

  // Per-bus EENS (MWh/yr)
  std::vector<double> nodal_eens_mwh_yr;

  // Weak point detection
  struct ComponentImportance {
    int index;            // within-type index (0-based within component_type group)
    bool is_generator;    // true = generator, false = other
    double importance;    // P(component down | system failure)
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

  /// Structured model-capability declaration.  Always "ac-only-dcopf" for
  /// the Monte Carlo / FMEA evaluators in this file: the state evaluator
  /// calls `solve_dc_opf` on the AC network only — DC power-flow balance,
  /// DC load curtailment, and VSC re-dispatch are NOT enforced.  EENS/LOLE
  /// figures therefore do NOT include DC load interruptions and will
  /// underestimate total curtailment on hybrid AC/DC systems.
  ///
  /// Do NOT treat `eens_mwh_yr` or `lole_hr_yr` as full-system reliability
  /// indices when the submitted `HybridPowerSystem` contains DC components.
  /// Check `validity.dc_load_curtailment_included` before using these figures.
  std::string model_scope{"ac-only-dcopf"};

  /// Per-feature validity flags so downstream code can branch on what the
  /// evaluator actually modelled, rather than guessing from `model_scope`.
  struct ValidityFlags {
    /// True only if DC load curtailment contributes to EENS/LOLE.
    /// Always false for the current AC-only DC-OPF evaluator.
    bool dc_load_curtailment_included{false};
    /// True if VSC/DC branch contingencies affect DC-side power balance.
    /// Always false — failures only affect AC island topology, not DC flow.
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

  // DC-OPF solver options for contingency evaluation
  opf::DCOPFOptions opf_options{};
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
    std::string action;  // "open" or "close"
    int bus_from{0};
    int bus_to{0};
  };

  // Explicit repair-stage switch operations selected by the topology search.
  std::vector<SwitchActionDetail> repair_switch_actions;

  // True if the repair-stage topology search was stopped early by the OPF
  // call budget (max_repair_opf_calls). Results remain valid but may not
  // reflect the globally optimal switching sequence.
  bool repair_search_truncated{false};
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

  // Per-contingency details (sorted by EENS contribution descending)
  std::vector<FMEAContingencyDetail> contingencies;

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
  };
  ValidityFlags validity{};
};

// ═══════════════════════════════════════════════════════════════════════
// Advanced Analytical Methods (Non-Simulation Based)
// ═══════════════════════════════════════════════════════════════════════

/// Frequency & Duration method (analytical).
/// Builds COPT recursively and calculates LOLP, LOLE, LOLF without simulation.
/// This is faster than MC but assumes independent component failures.
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

}  // namespace hacdcpf::analysis

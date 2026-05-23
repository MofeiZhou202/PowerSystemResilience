#pragma once

#include <string>
#include <vector>

#include "hacdcpf/time_series/annual_production_sim.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Theoretical Error Bounds
// ═══════════════════════════════════════════════════════════════════════

/// Dispatch approximation error bound: ε_dispatch ≤ ē · Σ_t L_max · P_load · Δt
struct DispatchErrorBound {
  double max_loss_fraction{0.0};     // L_max: max P_loss / P_load
  double max_emission_factor{0.0};   // ē: max generator emission factor
  double bound_tco2{0.0};           // ε_dispatch upper bound
};

/// Stratified sampling error bound
struct SamplingErrorBound {
  int num_strata{0};
  int total_sampled_hours{0};
  double confidence_level{0.95};     // 1-α
  double z_score{1.96};             // z_{1-α/2}
  double variance_estimate{0.0};    // stratified variance
  double bound_tco2{0.0};          // ε_sampling upper bound
  /// Per-stratum details
  struct Stratum {
    std::string name;
    int population_size{0};    // N_m
    int sample_size{0};        // n_m
    double variance{0.0};      // σ_m²
  };
  std::vector<Stratum> strata;
};

/// Storage carbon propagation error bound
struct StorageCarbonErrorBound {
  double lipschitz_constant{0.0};    // K_w
  double max_gap_hours{0.0};         // Δ_max
  double bound_tco2_per_mwh{0.0};  // |ŵ - w| upper bound
  double bound_tco2{0.0};          // total storage carbon error bound
};

/// Combined theoretical error bounds for one year
struct YearlyErrorBounds {
  DispatchErrorBound dispatch;
  SamplingErrorBound sampling;
  StorageCarbonErrorBound storage_carbon;
  double total_bound_tco2{0.0};  // sum of three components
};

/// Cross-validation: compare estimates and validate bound tightness
struct BoundCrossValidation {
  // Dense (all-hours) carbon estimate from Tier 1 dispatch
  double dense_carbon_tco2{0.0};
  // Stratified-sampling estimate (Tier 2)
  double sampled_carbon_tco2{0.0};
  // Sampling estimator gap: |dense - sampled|
  double sampling_gap_tco2{0.0};
  // Sampling gap as % of dense
  double sampling_gap_pct{0.0};
  // Total theoretical bound / dense carbon as percentage
  double total_bound_pct{0.0};
  // Per-component bounds as % of dense carbon
  double dispatch_bound_pct{0.0};
  double sampling_bound_pct{0.0};
  double storage_bound_pct{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Lifecycle Events
// ═══════════════════════════════════════════════════════════════════════

struct ReplacementEvent {
  int year{0};
  int storage_index{0};
  std::string storage_name;
  double old_soh{0.0};
  double replacement_cost_usd{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Per-Year Result
// ═══════════════════════════════════════════════════════════════════════

/// Per-storage yearly state
struct StorageYearState {
  int storage_index{0};
  std::string name;
  double soh{1.0};
  double soh_cycle{1.0};
  double soh_calendar{1.0};
  double effective_capacity_mwh{0.0};
  double cycles_this_year{0.0};
  double cumulative_cycles{0.0};
  bool replaced{false};
};

/// Per-renewable yearly state
struct RenewableYearState {
  int ren_index{0};
  std::string name;
  double derated_capacity_mw{0.0};
  double original_capacity_mw{0.0};
  double derating_factor{1.0};
};

/// Result for a single simulation year
struct YearResult {
  int year{0};                       // 1-indexed year within lifecycle
  double load_growth_factor{1.0};    // cumulative load growth multiplier

  // Annual simulation result (Tier 1)
  double annual_cost{0.0};
  double annual_gen_mwh{0.0};
  double annual_load_mwh{0.0};
  double annual_renewable_mwh{0.0};
  double annual_curtailment_mwh{0.0};
  double annual_ens_mwh{0.0};
  double annual_loss_mwh{0.0};
  bool feasible{false};

  // Carbon results (Tier 2 + 3)
  double annual_carbon_tco2{0.0};
  double avg_carbon_intensity{0.0};

  // Lifecycle states
  std::vector<StorageYearState> storage_states;
  std::vector<RenewableYearState> renewable_states;

  // Error bounds
  YearlyErrorBounds bounds;

  // Cross-validation of bounds
  BoundCrossValidation cross_validation;

  // Replacement events this year
  std::vector<ReplacementEvent> replacements;
};

// ═══════════════════════════════════════════════════════════════════════
// Lifecycle Simulation Result
// ═══════════════════════════════════════════════════════════════════════

struct LifecycleSimResult {
  std::vector<YearResult> year_results;
  int num_years{0};

  // Aggregate metrics
  double npv_total_cost{0.0};        // NPV of total costs (discount rate applied)
  double total_carbon_tco2{0.0};
  double total_replacement_cost{0.0};
  int total_replacements{0};

  // All replacement events
  std::vector<ReplacementEvent> all_replacements;

  bool feasible{false};
  std::string summary_text;
};

// ═══════════════════════════════════════════════════════════════════════
// Options
// ═══════════════════════════════════════════════════════════════════════

struct LifecycleSimOptions {
  int num_years{20};                   // planning horizon
  double discount_rate{0.05};          // annual discount rate for NPV
  double load_growth_rate{0.02};       // annual load growth rate

  // PV derating
  double pv_annual_derating{0.005};    // 0.5% per year

  // Battery degradation
  double calendar_degradation_per_year{0.02};  // 2% SOH loss per calendar year
  double cycle_degradation_per_cycle{0.00004}; // SOH loss per cycle (25000 cycles → 100%)

  // Sampling configuration for Tier 2
  int tier2_samples_per_stratum{4};    // hours to sample per operating regime
  double confidence_level{0.95};       // for sampling error bounds

  // Loss proxy for dispatch error bound
  double loss_proxy_fraction{0.03};    // estimated system loss fraction

  // Annual simulation sub-options
  double step_duration_hr{6.0};        // time resolution (6h default for speed)

  bool verbose{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Capacity Scaling for Long-Term Planning
// ═══════════════════════════════════════════════════════════════════════

/// Scale factors applied to component capacities before lifecycle simulation.
/// 1.0 = original capacity, 2.0 = doubled, 0.5 = halved.
struct CapacityScaling {
  double pv_scale{1.0};              // PV system pmax_mw
  double wind_scale{1.0};            // Wind renewable_gen p_rated_mw
  double bess_power_scale{1.0};      // Battery p_rated_mw, pmax, pmin
  double bess_energy_scale{1.0};     // Battery e_rated_mwh
  double diesel_scale{1.0};          // Diesel/gas static_gen p_rated_mw, pmax
};

/// Apply capacity scaling to a system (modifies in place)
void apply_capacity_scaling(HybridPowerSystem& sys, const CapacityScaling& scaling);

/// Summary of one capacity comparison scenario
struct ScenarioResult {
  CapacityScaling scaling;
  std::string label;                 // human-readable label
  double npv_total_cost{0.0};
  double total_carbon_tco2{0.0};
  double total_replacement_cost{0.0};
  int total_replacements{0};
  bool feasible{false};
  // Per-year carbon and cost trajectories
  std::vector<double> yearly_carbon;
  std::vector<double> yearly_cost;
  // Installed capacities (after scaling, MW or MWh)
  double total_pv_mw{0.0};
  double total_wind_mw{0.0};
  double total_bess_mw{0.0};
  double total_bess_mwh{0.0};
  double total_diesel_mw{0.0};
};

/// Result of a capacity comparison sweep
struct LifecycleCompareResult {
  std::vector<ScenarioResult> scenarios;
  std::string sweep_parameter;       // which parameter was swept
  int num_scenarios{0};
};

// ═══════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════

/// Run multi-year lifecycle simulation with degradation, derating, and
/// theoretical error bounds.
LifecycleSimResult run_lifecycle_simulation(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const LifecycleSimOptions& opts = {});

/// Run capacity comparison: sweep one parameter across a range of scale factors
LifecycleCompareResult run_lifecycle_comparison(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const LifecycleSimOptions& opts,
    const std::string& sweep_param,      // "pv", "wind", "bess_power", "bess_energy", "diesel"
    const std::vector<double>& scale_values,
    const CapacityScaling& base_scaling = {});

}  // namespace hacdcpf::analysis

#pragma once

// Carbon Flow Analysis — Proportional Power Tracing + Matrix-based Carbon Intensity
// Ported from luosipeng/HybridACDCPowerSystemsPlanning (luosipeng branch).
//
// Two methods:
//   1. Proportional Tracing (BFS-based): traces each generator's contribution
//      to every load and branch loss proportionally to power flow directions.
//   2. Matrix-based: builds a linear system A·w = b where w is the carbon
//      intensity vector at each bus, solved via sparse LU.
//
// Both methods require a converged PowerFlowResult plus the HybridPowerSystem
// (generator emission factors, load locations, and hybrid topology).

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

struct CarbonAnalysisOptions {
  /// Minimum power contribution to track (MW).
  double min_contribution_mw{1e-6};

  /// Relative tolerance for allocation verification (load / loss balance).
  double verification_tol{1e-4};

  /// Maximum BFS tracing depth (0 = unlimited).
  int max_tracing_depth{0};

  /// Branch/converter loss allocation factor in the carbon matrix. alpha=1
  /// assigns loss carbon to the sending-side intensity, alpha=0 assigns it to
  /// the receiving-side intensity, and alpha=0.5 splits it evenly.
  double loss_allocation_alpha{0.5};

  /// Regularisation for near-singular system matrix (matrix method).
  double regularization_eps{1e-8};

  /// Maximum accepted row-scaled QR pivot condition estimate. Set <= 0 to
  /// disable this gate while retaining rank and residual checks.
  double max_matrix_condition_estimate{1e12};

  /// Print progress to stdout.
  bool verbose{false};
};

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

struct EmissionsSummary {
  double total_generation_emissions_tco2{0.0};
  double total_load_emissions_tco2{0.0};
  double total_loss_emissions_tco2{0.0};
  double balance_error_tco2{0.0};
  double balance_error_pct{0.0};
};

struct LoadCarbonResult {
  int load_index{0};
  int bus{0};
  double demand_mw{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  /// Carbon source ID -> MW supplied to this load.
  std::unordered_map<int, double> generator_supply_mw;
};

struct BranchCarbonResult {
  int branch_index{0};
  int from_bus{0};
  int to_bus{0};
  double loss_mw{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  /// Carbon source ID -> MW of loss attributable to this source.
  std::unordered_map<int, double> generator_loss_mw;
};

struct BusCarbonResult {
  int bus_index{0};
  double carbon_intensity_tco2_mwh{0.0};
};

struct CarbonSourceResult {
  int source_id{0};
  std::string source_type;
  int component_index{0};
  std::string label;
  int bus{0};
  bool is_dc{false};
  double scheduled_power_mw{0.0};
  double power_mw{0.0};
  double emission_factor_tco2_mwh{0.0};
  bool is_balancing{false};
};

struct NodePowerBalanceError {
  int bus_index{0};
  bool is_dc{false};
  /// Explicit source power minus the power required by solved terminal flows
  /// and sinks. Negative means missing source; positive means missing sink.
  double mismatch_mw{0.0};
  double source_mw{0.0};
  double required_mw{0.0};
};

struct VSCCarbonResult {
  int converter_index{0};
  int bus_ac{0};
  int bus_dc{0};
  bool ac_to_dc{false};
  double input_power_mw{0.0};
  double output_power_mw{0.0};
  double loss_mw{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::unordered_map<int, double> generator_loss_mw;
};

struct DCDCCarbonResult {
  int converter_index{0};
  int bus_in{0};
  int bus_out{0};
  bool input_to_output{false};
  double input_power_mw{0.0};
  double output_power_mw{0.0};
  double loss_mw{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::unordered_map<int, double> generator_loss_mw;
};

struct StorageCarbonResult {
  int storage_index{0};
  int bus{0};
  bool is_dc{false};
  double p_mw{0.0};                           ///< signed; positive = discharge
  double soc{0.0};
  double stored_energy_mwh{0.0};
  double soc_carbon_intensity_tco2_mwh{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::unordered_map<int, double> source_supply_mw;
};

struct EnergyRouterCarbonResult {
  int router_index{0};
  double input_power_mw{0.0};
  double output_power_mw{0.0};
  double loss_mw{0.0};
  int active_input_ports{0};
  int active_output_ports{0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::unordered_map<int, double> source_loss_mw;
};

struct CarbonAnalysisResult {
  /// Sources in the exact ID order used by all proportional allocations.
  std::vector<CarbonSourceResult>       carbon_sources;
  /// Per-load results (AC loads), indexed same order as ACSystem::loads.
  std::vector<LoadCarbonResult>         load_carbon;
  /// Per-load results for DC loads.
  std::vector<LoadCarbonResult>         dc_load_carbon;
  /// Per-branch loss results for AC branches.
  std::vector<BranchCarbonResult>       branch_carbon;
  /// Per-branch loss results for DC branches.
  std::vector<BranchCarbonResult>       dc_branch_carbon;
  /// Per-bus intensity from matrix method (AC buses).
  std::vector<BusCarbonResult>          bus_carbon;
  /// Per-bus intensity from matrix method (DC buses).
  std::vector<BusCarbonResult>          dc_bus_carbon;
  /// Per-VSC converter loss results.
  std::vector<VSCCarbonResult>          vsc_carbon;
  /// Per-DC/DC converter loss results.
  std::vector<DCDCCarbonResult>         dcdc_carbon;
  /// Per-storage charge/discharge carbon.
  std::vector<StorageCarbonResult>      storage_carbon;
  /// Per-energy-router loss results.
  std::vector<EnergyRouterCarbonResult> energy_router_carbon;

  /// Source id → total MW allocated to network / converter losses.
  std::unordered_map<int, double> generator_loss_allocation;

  /// System-level summary from proportional tracing.
  EmissionsSummary tracing_summary;
  /// System-level summary from matrix method.
  EmissionsSummary matrix_summary;

  /// Storage charging emissions (storage acts as a carbon sink), in tCO2.
  double total_storage_charge_emissions_tco2{0.0};
  /// Storage discharge emissions (storage acts as a carbon source), in tCO2.
  double total_storage_discharge_emissions_tco2{0.0};
  /// Active power exported through an explicit AC/DC boundary, in MW.
  double total_external_export_mw{0.0};
  /// Carbon carried out through explicit AC/DC boundaries, in tCO2.
  double total_external_export_emissions_tco2{0.0};

  bool tracing_verified{false};
  bool matrix_solved{false};
  double matrix_residual{0.0};
  double matrix_relative_residual{0.0};
  double matrix_condition_estimate{0.0};
  int matrix_rank{0};

  /// Whether every canonical node closes its active-power balance using
  /// explicitly modelled sources, sinks and solved terminal flows.
  bool power_balance_verified{false};
  /// Largest absolute nodal active-power mismatch after source reconstruction.
  double max_node_power_balance_error_mw{0.0};
  /// Sum of absolute nodal active-power mismatches.
  double total_power_balance_error_mw{0.0};
  /// Nodes whose active-power mismatch exceeds the verification tolerance.
  std::vector<NodePowerBalanceError> node_power_balance_errors;
};

// ---------------------------------------------------------------------------
// Primary interface
// ---------------------------------------------------------------------------

/// Run carbon flow analysis using both proportional tracing and the matrix
/// method.  Requires a converged PowerFlowResult with branch flows.
CarbonAnalysisResult compute_carbon_analysis(
    const HybridPowerSystem& sys,
    const PowerFlowResult& pf_result,
    const CarbonAnalysisOptions& opt = {});

/// Convenience overload: run power flow first, then carbon analysis.
CarbonAnalysisResult compute_carbon_analysis(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& pf_opt = {},
    const CarbonAnalysisOptions& ca_opt = {});

/// Backward-compatible wrapper: runs power flow with default options then
/// calls compute_carbon_analysis.  Prefer compute_carbon_analysis() for new code.
inline CarbonAnalysisResult run_carbon_analysis(
    const HybridPowerSystem& sys,
    const CarbonAnalysisOptions& opt = {}) {
  return compute_carbon_analysis(sys, PowerFlowOptions{}, opt);
}

}  // namespace hacdcpf::analysis

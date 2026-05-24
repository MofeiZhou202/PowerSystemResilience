#pragma once

#include <string>
#include <vector>

#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace hacdcpf::analysis {

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

struct AnnualCarbonAnalysisOptions {
  CarbonAnalysisOptions carbon_options;
  bool keep_hourly_bus_intensity{false};
  bool keep_hourly_load_emissions{false};
  bool keep_hourly_load_energy{false};
};

// ---------------------------------------------------------------------------
// Step and aggregate result types
// ---------------------------------------------------------------------------

struct AnnualCarbonStepResult {
  bool pf_converged{false};
  double total_generation_emissions_tco2{0.0};
  double total_load_emissions_tco2{0.0};
  double total_loss_emissions_tco2{0.0};
};

struct AnnualBusCarbonStats {
  int bus_index{0};
  bool is_dc{false};
  double energy_mwh{0.0};
  double emissions_tco2{0.0};
  double load_weighted_intensity_tco2_mwh{0.0};
  double time_weighted_intensity_tco2_mwh{0.0};
  double average_intensity_tco2_mwh{0.0};
  double min_intensity_tco2_mwh{0.0};
  double max_intensity_tco2_mwh{0.0};
};

struct AnnualLoadCarbonStats {
  int load_index{0};
  int bus{0};
  bool is_dc{false};
  double energy_mwh{0.0};
  double emissions_tco2{0.0};
  double average_intensity_tco2_mwh{0.0};
};

struct AnnualCarbonAnalysisResult {
  int num_steps{0};
  double step_duration_hr{1.0};
  int num_pf_converged{0};

  double total_generation_emissions_tco2{0.0};
  double total_load_emissions_tco2{0.0};
  double total_loss_emissions_tco2{0.0};

  std::vector<AnnualCarbonStepResult> step_results;
  std::vector<AnnualBusCarbonStats> bus_stats;
  std::vector<AnnualLoadCarbonStats> load_stats;

  // Matrix shape: row = time step, column = bus_stats order.
  // Non-converged steps are filled with NaN.
  std::vector<std::vector<double>> hourly_bus_intensity_tco2_mwh;

  // Matrix shape: row = time step, column = load_stats order.
  // Values are per-step emissions in tCO2; non-converged steps are NaN.
  std::vector<std::vector<double>> hourly_load_emissions_tco2;

  // Matrix shape: row = time step, column = load_stats order.
  // Values are per-step energy in MWh; non-converged steps are NaN.
  std::vector<std::vector<double>> hourly_load_energy_mwh;
};

// ---------------------------------------------------------------------------
// User GEC accounting types
// ---------------------------------------------------------------------------

struct AnnualLoadRef {
  int load_index{0};
  bool is_dc{false};
};

struct AnnualUserGECInput {
  int user_id{0};
  std::vector<AnnualLoadRef> loads;
  double annual_gec_mwh{0.0};
  double gec_intensity_tco2_mwh{0.0};
};

struct AnnualUserCarbonStepResult {
  int user_id{0};
  double energy_mwh{0.0};
  double gross_emissions_tco2{0.0};
  double allocated_gec_mwh{0.0};
  double net_emissions_tco2{0.0};
  double gross_intensity_tco2_mwh{0.0};
  double net_intensity_tco2_mwh{0.0};
};

struct AnnualUserCarbonStats {
  int user_id{0};
  double energy_mwh{0.0};
  double gross_emissions_tco2{0.0};
  double allocated_gec_mwh{0.0};
  double unused_gec_mwh{0.0};
  double avoided_emissions_tco2{0.0};
  double net_emissions_tco2{0.0};
  double gross_intensity_tco2_mwh{0.0};
  double net_intensity_tco2_mwh{0.0};
  double gec_coverage_ratio{0.0};
};

struct AnnualUserGECResult {
  int num_steps{0};
  double step_duration_hr{1.0};
  std::vector<AnnualUserCarbonStats> user_stats;

  // Optional matrix shape: row = time step, column = user_stats order.
  std::vector<std::vector<AnnualUserCarbonStepResult>> hourly_user_results;
};

// ---------------------------------------------------------------------------
// Node-level GEC accounting types
// ---------------------------------------------------------------------------

struct AnnualNodeGECInput {
  int bus_index{0};
  bool is_dc{false};
  double annual_gec_mwh{0.0};
  double gec_intensity_tco2_mwh{0.0};
};

struct AnnualNodeCarbonStepResult {
  int bus_index{0};
  bool is_dc{false};
  double energy_mwh{0.0};
  double gross_emissions_tco2{0.0};
  double allocated_gec_mwh{0.0};
  double net_emissions_tco2{0.0};
  double gross_intensity_tco2_mwh{0.0};
  double net_intensity_tco2_mwh{0.0};
};

struct AnnualNodeCarbonStats {
  int bus_index{0};
  bool is_dc{false};
  double energy_mwh{0.0};
  double gross_emissions_tco2{0.0};
  double allocated_gec_mwh{0.0};
  double unused_gec_mwh{0.0};
  double avoided_emissions_tco2{0.0};
  double net_emissions_tco2{0.0};
  double gross_intensity_tco2_mwh{0.0};
  double net_intensity_tco2_mwh{0.0};
  double gec_coverage_ratio{0.0};
};

struct AnnualNodeGECResult {
  int num_steps{0};
  double step_duration_hr{1.0};
  std::vector<AnnualNodeCarbonStats> node_stats;
  // Row = time step, column = node_stats order. Populated when keep_hourly=true.
  std::vector<std::vector<AnnualNodeCarbonStepResult>> hourly_node_results;
};

// ---------------------------------------------------------------------------
// Core annual carbon analysis functions
// ---------------------------------------------------------------------------

/// Annual carbon-flow post-processing for a static system and PF snapshots.
/// This keeps sys fixed for every timestep; use the per-step overload when
/// load, generation, OPF dispatch, or storage state changes by timestep.
AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const HybridPowerSystem& sys,
    const std::vector<PowerFlowResult>& pf_results,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options = {});

/// Annual carbon-flow post-processing for per-step system snapshots.
/// systems[t] must match pf_results[t].
AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const std::vector<HybridPowerSystem>& systems,
    const std::vector<PowerFlowResult>& pf_results,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options = {});

/// Static-system convenience overload for an existing time-series PF result.
/// This intentionally uses the supplied sys for every timestep.
AnnualCarbonAnalysisResult compute_annual_carbon_analysis(
    const HybridPowerSystem& sys,
    const TimeSeriesPFResult& ts_result,
    double step_duration_hr,
    const AnnualCarbonAnalysisOptions& options = {});

// ---------------------------------------------------------------------------
// User-level GEC accounting
// ---------------------------------------------------------------------------

/// User-side annual GEC accounting on top of gross annual carbon-flow results.
/// Callers must compute annual with keep_hourly_load_energy and
/// keep_hourly_load_emissions enabled.  This does not modify physical bus/load
/// carbon results.
AnnualUserGECResult compute_annual_user_gec_accounting(
    const AnnualCarbonAnalysisResult& annual,
    const std::vector<AnnualUserGECInput>& users,
    bool keep_hourly = false);

/// Build a default external user mapping where each load is treated as one user.
/// GEC inputs default to zero and can be filled by the caller.
std::vector<AnnualUserGECInput> make_one_load_per_user_gec_inputs(
    const AnnualCarbonAnalysisResult& annual,
    int first_user_id = 1);

/// Serialize annual user GEC accounting stats as a CSV table.
std::string annual_user_gec_result_to_csv(const AnnualUserGECResult& result);

/// Write annual user GEC accounting stats as a CSV table.
void save_annual_user_gec_result_csv(const AnnualUserGECResult& result,
                                     const std::string& path);

/// Serialize annual user GEC accounting stats as a JSON document.
std::string annual_user_gec_result_to_json(const AnnualUserGECResult& result,
                                           int indent = 2);

/// Write annual user GEC accounting stats as a JSON document.
void save_annual_user_gec_result_json(const AnnualUserGECResult& result,
                                      const std::string& path,
                                      int indent = 2);

/// Serialize hourly user GEC accounting results as a CSV table.
/// Requires result.hourly_user_results populated by keep_hourly=true.
std::string annual_user_gec_hourly_to_csv(const AnnualUserGECResult& result);

/// Write hourly user GEC accounting results as a CSV table.
void save_annual_user_gec_hourly_csv(const AnnualUserGECResult& result,
                                     const std::string& path);

/// Serialize hourly user GEC accounting results as a JSON document.
/// Requires result.hourly_user_results populated by keep_hourly=true.
std::string annual_user_gec_hourly_to_json(const AnnualUserGECResult& result,
                                           int indent = 2);

/// Write hourly user GEC accounting results as a JSON document.
void save_annual_user_gec_hourly_json(const AnnualUserGECResult& result,
                                      const std::string& path,
                                      int indent = 2);

/// Validate annual user totals.  If hourly results are present, they must
/// reconcile with annual stats; otherwise only annual non-negativity and
/// coverage bounds are checked.
bool validate_annual_user_gec_result(const AnnualUserGECResult& result,
                                     double tol = 1e-8);

// ---------------------------------------------------------------------------
// Bus-level annual carbon CSV/JSON output
// ---------------------------------------------------------------------------

/// Serialize per-step bus carbon intensity as a CSV table.
/// Requires annual.hourly_bus_intensity_tco2_mwh to be populated.
std::string annual_bus_carbon_hourly_to_csv(const AnnualCarbonAnalysisResult& annual);

/// Write per-step bus carbon intensity CSV to file.
void save_annual_bus_carbon_hourly_csv(const AnnualCarbonAnalysisResult& annual,
                                       const std::string& path);

/// Serialize per-step bus carbon intensity as a JSON document.
std::string annual_bus_carbon_hourly_to_json(const AnnualCarbonAnalysisResult& annual,
                                             int indent = 2);

/// Write per-step bus carbon intensity JSON to file.
void save_annual_bus_carbon_hourly_json(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path,
                                        int indent = 2);

/// Serialize annual bus carbon summary as a CSV table.
std::string annual_bus_carbon_summary_to_csv(const AnnualCarbonAnalysisResult& annual);

/// Write annual bus carbon summary CSV to file.
void save_annual_bus_carbon_summary_csv(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path);

/// Serialize annual bus carbon summary as a JSON document.
std::string annual_bus_carbon_summary_to_json(const AnnualCarbonAnalysisResult& annual,
                                              int indent = 2);

/// Write annual bus carbon summary JSON to file.
void save_annual_bus_carbon_summary_json(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path,
                                         int indent = 2);

// ---------------------------------------------------------------------------
// Load-level annual gross carbon CSV/JSON output
// ---------------------------------------------------------------------------

/// Serialize per-step per-load gross energy/emissions as a CSV table.
std::string annual_load_carbon_hourly_to_csv(const AnnualCarbonAnalysisResult& annual);

/// Write per-step per-load gross CSV to file.
void save_annual_load_carbon_hourly_csv(const AnnualCarbonAnalysisResult& annual,
                                        const std::string& path);

/// Serialize per-step per-load gross results as a JSON document.
std::string annual_load_carbon_hourly_to_json(const AnnualCarbonAnalysisResult& annual,
                                              int indent = 2);

/// Write per-step per-load gross JSON to file.
void save_annual_load_carbon_hourly_json(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path,
                                         int indent = 2);

/// Serialize annual load carbon summary as a CSV table.
std::string annual_load_carbon_summary_to_csv(const AnnualCarbonAnalysisResult& annual);

/// Write annual load carbon summary CSV to file.
void save_annual_load_carbon_summary_csv(const AnnualCarbonAnalysisResult& annual,
                                         const std::string& path);

/// Serialize annual load carbon summary as a JSON document.
std::string annual_load_carbon_summary_to_json(const AnnualCarbonAnalysisResult& annual,
                                               int indent = 2);

/// Write annual load carbon summary JSON to file.
void save_annual_load_carbon_summary_json(const AnnualCarbonAnalysisResult& annual,
                                          const std::string& path,
                                          int indent = 2);

// ---------------------------------------------------------------------------
// Node-level annual GEC accounting
// ---------------------------------------------------------------------------

/// Node-level annual GEC accounting.
AnnualNodeGECResult compute_annual_node_gec_accounting(
    const AnnualCarbonAnalysisResult& annual,
    const std::vector<AnnualNodeGECInput>& nodes,
    bool keep_hourly = false);

/// Serialize annual node GEC stats as a CSV table.
std::string annual_node_gec_result_to_csv(const AnnualNodeGECResult& result);

/// Write annual node GEC stats CSV to file.
void save_annual_node_gec_result_csv(const AnnualNodeGECResult& result,
                                     const std::string& path);

/// Serialize annual node GEC stats as a JSON document.
std::string annual_node_gec_result_to_json(const AnnualNodeGECResult& result,
                                           int indent = 2);

/// Write annual node GEC stats JSON to file.
void save_annual_node_gec_result_json(const AnnualNodeGECResult& result,
                                      const std::string& path,
                                      int indent = 2);

/// Serialize per-step node GEC results as a CSV table.
std::string annual_node_gec_hourly_to_csv(const AnnualNodeGECResult& result);

/// Write per-step node GEC CSV to file.
void save_annual_node_gec_hourly_csv(const AnnualNodeGECResult& result,
                                     const std::string& path);

/// Serialize per-step node GEC results as a JSON document.
std::string annual_node_gec_hourly_to_json(const AnnualNodeGECResult& result,
                                           int indent = 2);

/// Write per-step node GEC JSON to file.
void save_annual_node_gec_hourly_json(const AnnualNodeGECResult& result,
                                      const std::string& path,
                                      int indent = 2);

/// Validate node GEC result annual totals and hourly reconciliation.
bool validate_annual_node_gec_result(const AnnualNodeGECResult& result,
                                     double tol = 1e-8);

// ---------------------------------------------------------------------------
// GEC input CSV/JSON parsers
// ---------------------------------------------------------------------------

/// Parse node GEC inputs from CSV text.
/// Expected header: bus_index,is_dc,annual_gec_mwh,gec_intensity_tco2_mwh
std::vector<AnnualNodeGECInput> parse_annual_node_gec_inputs_csv(
    const std::string& csv_text);

/// Load node GEC inputs from a CSV file.
std::vector<AnnualNodeGECInput> load_annual_node_gec_inputs_csv(
    const std::string& path);

/// Serialize node GEC inputs as JSON text.
std::string annual_node_gec_inputs_to_json(
    const std::vector<AnnualNodeGECInput>& nodes,
    int indent = 2);

/// Write node GEC inputs as JSON.
void save_annual_node_gec_inputs_json(
    const std::vector<AnnualNodeGECInput>& nodes,
    const std::string& path,
    int indent = 2);

/// Parse node GEC inputs from JSON text.
std::vector<AnnualNodeGECInput> parse_annual_node_gec_inputs_json(
    const std::string& json_text);

/// Load node GEC inputs from a JSON file.
std::vector<AnnualNodeGECInput> load_annual_node_gec_inputs_json(
    const std::string& path);

/// Parse user GEC inputs from CSV text.
/// Expected header: user_id,load_index,is_dc,annual_gec_mwh,gec_intensity_tco2_mwh
std::vector<AnnualUserGECInput> parse_annual_user_gec_inputs_csv(
    const std::string& csv_text);

/// Load user GEC inputs from a CSV file.
std::vector<AnnualUserGECInput> load_annual_user_gec_inputs_csv(
    const std::string& path);

/// Serialize user GEC inputs as JSON text.
std::string annual_user_gec_inputs_to_json(
    const std::vector<AnnualUserGECInput>& users,
    int indent = 2);

/// Write user GEC inputs as JSON.
void save_annual_user_gec_inputs_json(
    const std::vector<AnnualUserGECInput>& users,
    const std::string& path,
    int indent = 2);

/// Parse user GEC inputs from JSON text.
std::vector<AnnualUserGECInput> parse_annual_user_gec_inputs_json(
    const std::string& json_text);

/// Load user GEC inputs from a JSON file.
std::vector<AnnualUserGECInput> load_annual_user_gec_inputs_json(
    const std::string& path);

}  // namespace hacdcpf::analysis

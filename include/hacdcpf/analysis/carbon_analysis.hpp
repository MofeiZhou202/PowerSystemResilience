#pragma once
#include <map>
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::analysis {

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
  std::map<int, double> generator_supply_mw;
};

struct BranchCarbonResult {
  int branch_index{0};
  int from_bus{0};
  int to_bus{0};
  double loss_mw{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::map<int, double> generator_loss_mw;
};

struct BusCarbonResult {
  int bus_index{0};
  double carbon_intensity_tco2_mwh{0.0};
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
  std::map<int, double> generator_loss_mw;
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
  std::map<int, double> generator_loss_mw;
};

struct StorageCarbonResult {
  int storage_index{0};
  int bus{0};
  bool is_dc{false};
  double p_mw{0.0};
  double soc{0.0};
  double stored_energy_mwh{0.0};
  double soc_carbon_intensity_tco2_mwh{0.0};
  double carbon_intensity_tco2_mwh{0.0};
  double total_emissions_tco2{0.0};
  std::map<int, double> source_supply_mw;
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
  std::map<int, double> source_loss_mw;
};

struct CarbonAnalysisResult {
  bool tracing_verified{false};
  bool matrix_solved{false};
  double matrix_residual{0.0};

  std::vector<LoadCarbonResult>         load_carbon;
  std::vector<LoadCarbonResult>         dc_load_carbon;
  std::vector<BranchCarbonResult>       branch_carbon;
  std::vector<BranchCarbonResult>       dc_branch_carbon;
  std::vector<BusCarbonResult>          bus_carbon;
  std::vector<BusCarbonResult>          dc_bus_carbon;
  std::vector<VSCCarbonResult>          vsc_carbon;
  std::vector<DCDCCarbonResult>         dcdc_carbon;
  std::vector<StorageCarbonResult>      storage_carbon;
  std::vector<EnergyRouterCarbonResult> energy_router_carbon;

  std::map<int, double> generator_loss_allocation;

  EmissionsSummary tracing_summary;
  EmissionsSummary matrix_summary;
};

struct CarbonAnalysisOptions {
  bool enable_tracing{true};
  bool enable_matrix{true};
  double base_mva{100.0};
};

CarbonAnalysisResult run_carbon_analysis(
    const HybridPowerSystem& sys,
    const CarbonAnalysisOptions& opt = {});

}  // namespace hacdcpf::analysis

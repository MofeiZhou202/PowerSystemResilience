#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

enum class CounterfactualMeasureType {
  LineCapacity,
  Storage,
  TieSwitch,
  Automation,
  DistributedEnergyResource,
};

struct CounterfactualMeasure {
  std::string id;
  std::string name;
  CounterfactualMeasureType type{CounterfactualMeasureType::LineCapacity};
  std::string domain{"AC"};
  int target_index{-1};
  int bus{-1};
  int from_bus{-1};
  int to_bus{-1};
  double expansion_factor{1.5};
  double capacity_mw{0.0};
  double energy_mwh{0.0};
  double dispatch_fraction{0.25};
  double capacity_factor{0.40};
  double capex{0.0};
};

struct CounterfactualPlanningOptions {
  bool enable_line_capacity{true};
  bool enable_storage{true};
  bool enable_tie_switch{true};
  bool enable_automation{true};
  bool enable_der{true};
  bool include_economic{true};
  bool include_carbon{true};
  bool include_reliability{true};
  bool include_resilience{true};
  bool include_pairs{true};
  int max_measures{5};
  int max_pairs{10};
  int reliability_physical_budget{40};
  int resilience_horizon_hours{12};
  int resilience_fault_count{1};
  double resilience_repair_time_hr{6.0};
  double annual_hours{8760.0};
  double energy_price_per_mwh{100.0};
  double line_expansion_factor{1.5};
  double storage_power_mw{0.0};
  double storage_duration_hr{4.0};
  double storage_dispatch_fraction{0.25};
  double der_capacity_mw{0.0};
  double der_capacity_factor{0.40};
  double tie_capacity_mw{0.0};
  std::vector<int> resilience_fault_branch_indices;
};

struct CounterfactualMetricSet {
  // economic operating cost ($/yr), carbon emissions (tCO2/yr), reliability
  // EENS (MWh/yr), and event-conditioned resilience weighted ENS (MWh).
  std::array<std::optional<double>, 4> values;
  std::array<std::string, 4> diagnostics;
};

struct CounterfactualBenefitVector {
  std::array<std::optional<double>, 4> absolute;
  std::array<std::optional<double>, 4> relative;
  int improved_dimensions{0};
  int worsened_dimensions{0};
  std::string relation;
};

struct CounterfactualMeasureResult {
  CounterfactualMeasure measure;
  bool applied{false};
  std::string apply_status;
  CounterfactualMetricSet metrics;
  CounterfactualBenefitVector benefit;
};

struct CounterfactualInteractionResult {
  std::string measure_a;
  std::string measure_b;
  bool applied{false};
  std::string apply_status;
  double combined_capex{0.0};
  CounterfactualMetricSet metrics;
  CounterfactualBenefitVector combined_benefit;
  CounterfactualBenefitVector synergy;
};

struct CounterfactualPlanningResult {
  std::string schema{"hacdcpf.counterfactual_planning.v1"};
  CounterfactualMetricSet baseline;
  std::vector<CounterfactualMeasureResult> measures;
  std::vector<CounterfactualInteractionResult> interactions;
  int evaluated_systems{0};
  int failed_systems{0};
  std::vector<int> resilience_fault_branch_indices;
  std::string methodology;
};

struct CounterfactualApplyResult {
  bool applied{false};
  std::string status;
};

const char* counterfactual_measure_type_name(CounterfactualMeasureType type) noexcept;
const char* counterfactual_metric_name(int dimension) noexcept;
const char* counterfactual_metric_unit(int dimension) noexcept;

std::vector<CounterfactualMeasure> generate_counterfactual_measures(
    const HybridPowerSystem& system,
    const CounterfactualPlanningOptions& options = {});

CounterfactualApplyResult apply_counterfactual_measure(
    HybridPowerSystem& system, const CounterfactualMeasure& measure);

CounterfactualPlanningResult run_counterfactual_planning_assessment(
    const HybridPowerSystem& system,
    const std::vector<CounterfactualMeasure>& measures = {},
    const CounterfactualPlanningOptions& options = {});

}  // namespace hacdcpf::analysis

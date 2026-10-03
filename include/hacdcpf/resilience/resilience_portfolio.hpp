#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/analysis/scenario_generation.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

// One shared, scenario-independent configuration. The generated faults remain
// unchanged; only the request-local system receives these resources.
struct ResiliencePortfolioPlan {
  std::string id;
  int ac_generator_bus{0};
  int ac_generator_index{0};
  double ac_generator_mw{0.0};
  int mobile_storage_bus{0};
  int mobile_storage_index{0};
  double mobile_storage_mw{0.0};
  double mobile_storage_mwh{0.0};
};

struct ResiliencePortfolioScenarioScore {
  std::string scenario_id;
  std::string group;
  double design_weight{0.0};
  double baseline_shed_mwh{0.0};
  double planned_shed_mwh{0.0};
  double baseline_weighted_unserved_mwh{0.0};
  double planned_weighted_unserved_mwh{0.0};
};

struct ResiliencePortfolioResult {
  std::string schema{"hacdcpf.resilience_portfolio.v1"};
  ResiliencePortfolioPlan plan;
  std::vector<ResiliencePortfolioScenarioScore> scenarios;
  double baseline_design_weighted_shed_mwh{0.0};
  double planned_design_weighted_shed_mwh{0.0};
  double baseline_worst_shed_mwh{0.0};
  double planned_worst_shed_mwh{0.0};
  std::vector<std::string> limitations;
};

// Uses every generated cluster representative and conditional cluster weight.
// Each requested intensity group has equal design weight; these are not
// occurrence probabilities. The same heuristic portfolio is applied to every
// representative before the scenario-specific RA recovery assessment.
ResiliencePortfolioResult plan_resilience_portfolio(
    const HybridPowerSystem& system,
    const ResilienceScenarioResult& scenarios,
    bool add_generator,
    bool add_mobile_storage,
    const std::function<bool()>& should_cancel = {});

void apply_resilience_portfolio_plan(HybridPowerSystem& system,
                                     const ResiliencePortfolioPlan& plan);

}  // namespace hacdcpf::analysis

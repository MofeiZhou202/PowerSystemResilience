#pragma once
#include <map>
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::planning {

// ── Shared planning types ─────────────────────────────────────────────────────

enum class PlanningObjective { MinCost, MaxReliability, MinEmissions };

struct InvestmentCandidate {
  std::string type;  ///< "branch", "generator", "storage", "vsc"
  int from_bus{-1};
  int to_bus{-1};
  double capacity_mw{0.0};
  double capital_cost_usd{0.0};
  double annual_om_cost_usd{0.0};
  double lifetime_years{20.0};
  bool fixed{false};  ///< If true, always invested
};

struct PlanningOptions {
  PlanningObjective objective{PlanningObjective::MinCost};
  double time_horizon_years{20.0};
  double discount_rate{0.05};
  int num_scenarios{1};
  int max_iter{5000};
  double optimality_gap{1e-4};
  double time_limit_sec{3600.0};
  int num_threads{1};
  bool warm_start{false};
  bool verbose{false};
};

struct CandidateResult {
  int candidate_index{0};
  bool invested{false};
  double investment_cost_usd{0.0};
  double capacity_mw{0.0};
};

struct PlanningResult {
  bool converged{false};
  double objective_value{0.0};
  double lower_bound{0.0};
  double optimality_gap{0.0};
  int num_scenarios_solved{0};
  double solve_time_sec{0.0};
  std::vector<CandidateResult> investments;
  std::string status;
};

// ── Microgrid planning ────────────────────────────────────────────────────────

struct MicrogridPlanningInput {
  HybridPowerSystem base_system;
  std::vector<InvestmentCandidate> candidates;
  std::vector<double> load_profile;     ///< Hourly load factor (fraction)
  std::vector<double> solar_profile;    ///< Hourly solar factor (fraction)
  std::vector<double> wind_profile;     ///< Hourly wind factor (fraction)
  double peak_load_mw{0.0};
  double energy_price_usd_mwh{100.0};
};

PlanningResult solve_microgrid_planning(
    const MicrogridPlanningInput& input,
    const PlanningOptions& options = {});

PlanningResult solve_microgrid_planning_baseline(
    const MicrogridPlanningInput& input,
    const PlanningOptions& options = {});

PlanningResult solve_microgrid_planning_nested_bc(
    const MicrogridPlanningInput& input,
    const PlanningOptions& options = {});

}  // namespace hacdcpf::planning

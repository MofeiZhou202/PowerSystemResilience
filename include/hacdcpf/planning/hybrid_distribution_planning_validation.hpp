#pragma once
#include <map>
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"
#include "hacdcpf/planning/microgrid_planning_solver.hpp"

namespace hacdcpf::planning {

// ── Validation of planning results ───────────────────────────────────────────

struct HybridDistributionPlanningResult {
  bool converged{false};
  double objective_value{0.0};
  std::vector<CandidateResult> investments;
  std::string status;
};

struct HybridDistributionValidationOptions {
  double tolerance{1e-4};
  bool check_power_flow{true};
  bool check_voltage_limits{true};
  bool check_thermal_limits{true};
  bool check_reliability{false};
  bool verbose{false};
};

struct ValidationViolation {
  std::string type;    ///< "voltage", "thermal", "convergence", etc.
  int element_index{-1};
  std::string description;
  double value{0.0};
  double limit{0.0};
};

struct HybridDistributionValidationReport {
  bool valid{false};
  std::vector<ValidationViolation> violations;
  int num_scenarios_checked{0};
  double worst_voltage_pu{0.0};
  double worst_loading_pct{0.0};
  std::string status;
};

HybridDistributionValidationReport validate_hybrid_distribution_planning_result(
    const HybridPowerSystem& base_system,
    const HybridDistributionPlanningResult& result,
    const HybridDistributionValidationOptions& options = {});

// ── Replay a candidate solution ───────────────────────────────────────────────

struct HybridDistributionCandidateReplayOptions {
  bool run_power_flow{true};
  bool run_reliability{false};
  bool verbose{false};
};

struct HybridDistributionCandidateReplayResult {
  bool completed{false};
  HybridPowerSystem built_system;
  std::string status;
};

HybridDistributionCandidateReplayResult replay_hybrid_distribution_candidate(
    const HybridPowerSystem& base_system,
    const HybridDistributionPlanningResult& result,
    const HybridDistributionCandidateReplayOptions& options = {});

}  // namespace hacdcpf::planning

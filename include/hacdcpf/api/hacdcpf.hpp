#pragma once

#include <vector>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf.hpp"
#include "hacdcpf/optimal_power_flow/reactive_power_opt.hpp"
#include "hacdcpf/power_flow/ac_linearized_pf.hpp"
#include "hacdcpf/analysis/carbon_analysis.hpp"
#include "hacdcpf/analysis/time_series_pf.hpp"
#include "hacdcpf/analysis/reliability_assessment.hpp"
#include "hacdcpf/analysis/resilience_assessment.hpp"
#include "hacdcpf/market/market_simulation.hpp"
#include "hacdcpf/planning/hybrid_distribution_planning_validation.hpp"
#include "hacdcpf/planning/microgrid_planning_solver.hpp"

namespace hacdcpf {

struct SolverHandle;

PowerFlowResult solve_power_flow(const HybridPowerSystem& sys,
                                 const PowerFlowOptions& opt = {});

DCPowerFlowResult solve_dc_power_flow(const HybridPowerSystem& sys,
                                      const PowerFlowOptions& opt = {});

AdaptiveSolveResult solve_power_flow_adaptive(const HybridPowerSystem& sys,
                                              const PowerFlowOptions& opt = {});

IslandedSolveResult solve_power_flow_islanded(const HybridPowerSystem& sys,
                                              const PowerFlowOptions& opt = {});

DistributedSlackResult solve_power_flow_distributed_slack(
    const HybridPowerSystem& sys,
    const DistributedSlack& slack_cfg,
    const PowerFlowOptions& opt = {});

DistributedSlackResult solve_power_flow_distributed_slack_full(
    const HybridPowerSystem& sys,
    const DistributedSlack& slack_cfg,
    const PowerFlowOptions& opt = {});

SolverHandle* create_solver_handle(const HybridPowerSystem& sys,
                                   LossModelType loss_model = LossModelType::Linear);

void reset_solver_handle(SolverHandle* handle,
                         const HybridPowerSystem& sys,
                         LossModelType loss_model = LossModelType::Linear);

PowerFlowResult solve_handle(SolverHandle* handle,
                             const PowerFlowOptions& opt = {});

DCPowerFlowResult solve_dc_handle(SolverHandle* handle,
                                  const PowerFlowOptions& opt = {});

void destroy_solver_handle(SolverHandle* handle);

PowerFlowResult solve_power_flow_fdpf(const HybridPowerSystem& sys,
                                       const PowerFlowOptions& opt = {});

powerflow::ACLinearizedDCResult solve_ac_dc_power_flow(const HybridPowerSystem& sys,
                                                        const PowerFlowOptions& opt = {});

opf::ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                              const opf::ACOPFOptions& opt = {});

opf::DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                              const opf::DCOPFOptions& opt = {});

opf::RPOResult solve_rpo(const HybridPowerSystem& sys,
                         const opf::RPOOptions& opt = {});

analysis::DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& sys,
    const analysis::DistributionResilienceOptions& opt = {});

market::MarketClearingOutput run_market_clearing(
    const HybridPowerSystem& sys,
    const market::MarketConfig& config = {},
    const std::vector<market::GenCoBid>& bids = {},
    const market::MarketProfiles* profiles = nullptr,
    const market::InitialStatus* init = nullptr,
    const market::ScenarioConfig* scen_cfg = nullptr);

planning::PlanningResult solve_microgrid_planning(
    const planning::MicrogridPlanningInput& input,
    const planning::PlanningOptions& options = {});

planning::PlanningResult solve_microgrid_planning_baseline(
    const planning::MicrogridPlanningInput& input,
    const planning::PlanningOptions& options = {});

planning::PlanningResult solve_microgrid_planning_nested_bc(
    const planning::MicrogridPlanningInput& input,
    const planning::PlanningOptions& options = {});

planning::HybridDistributionCandidateReplayResult replay_hybrid_distribution_candidate(
    const HybridPowerSystem& base_system,
    const planning::HybridDistributionPlanningResult& result,
    const planning::HybridDistributionCandidateReplayOptions& options = {});

planning::HybridDistributionValidationReport validate_hybrid_distribution_planning_result(
    const HybridPowerSystem& base_system,
    const planning::HybridDistributionPlanningResult& result,
    const planning::HybridDistributionValidationOptions& options = {});

}  // namespace hacdcpf

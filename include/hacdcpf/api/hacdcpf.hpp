#pragma once

#include <vector>

#include "hacdcpf/api/solver_capabilities.hpp"
#include "hacdcpf/detail/internal_helpers.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/optimal_power_flow/opf_solver_interface.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/reactive_power_opt.hpp"
#include "hacdcpf/power_flow/ac_linearized_pf.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/time_series/annual_production_sim.hpp"
#include "hacdcpf/time_series/lifecycle_simulation.hpp"
#include "hacdcpf/integrated_energy/integrated_energy_optimizer.hpp"
#include "hacdcpf/market/market_simulation.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/validation/validate_system.hpp"
#include "hacdcpf/model/typical_parameters.hpp"
#include "hacdcpf/model/standard_parameter_library.hpp"

namespace hacdcpf {

struct SolverHandle;
// ── Capabilities ──────────────────────────────────────────────────────────────────

// get_solver_capabilities() is declared in api/solver_capabilities.hpp
// and implemented in src/api/hacdcpf.cpp.
// ── Validation ────────────────────────────────────────────────────────────────

/// Full model validation — returns a structured ValidationReport.
/// The existing validate() in network_utils.hpp remains for backward compatibility.
validation::ValidationReport validate_full(const HybridPowerSystem& sys);

// ── Power flow ────────────────────────────────────────────────────────────────

PowerFlowResult solve_power_flow(const HybridPowerSystem& sys,
                                 const PowerFlowOptions& opt = {});

/// Exception-free variant: returns Result<PowerFlowResult>.
/// Performs model validation before solving when `validate_input` is true.
Result<PowerFlowResult> safe_solve_power_flow(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& opt = {},
    bool validate_input = true);

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

// ── Transient dynamics ─────────────────────────────────────────────────────

dynamics::DynamicResults run_transient_simulation(
    const HybridPowerSystem& sys,
    const dynamics::DynamicSolverOptions& opt = {});

// ── OPF ───────────────────────────────────────────────────────────────────────

opf::ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                              const opf::ACOPFOptions& opt = {});

opf::DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                              const opf::DCOPFOptions& opt = {});

opf::RPOResult solve_rpo(const HybridPowerSystem& sys,
                         const opf::RPOOptions& opt = {});
/// Run an independent post-solve feasibility audit on an AC OPF result.
/// Checks power balance, voltage limits, branch limits, generator limits,
/// and objective consistency.  Populates result.audit and appends to
/// result.infeasibility_hints.
void verify_opf_result(const HybridPowerSystem& sys, opf::ACOPFResult& result);
// ── Analysis ──────────────────────────────────────────────────────────────────

analysis::DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& sys,
    const analysis::DistributionResilienceOptions& opt = {});

}  // namespace hacdcpf

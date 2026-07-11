#pragma once

#include <string>
#include <vector>

#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/util/parallel_execution.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Hierarchical Temporal Decomposition Levels
// ═══════════════════════════════════════════════════════════════════════

/// L0: Annual planning — maintenance windows, energy/fuel budgets per block.
struct AnnualPlanBlock {
  int block_id{0};                     // 0-indexed block (month or week)
  int start_step{0};                   // first global time index in this block
  int end_step{0};                     // one-past-end global time index
  std::vector<int> gen_maintenance;    // [g] 1=on maintenance, 0=available
  std::vector<double> gen_energy_budget_mwh;  // [g] max energy in this block
  double fuel_budget{1e30};            // aggregate fuel cap
  std::vector<double> storage_init_soc;       // [s] SOC at start of block [0,1]
  std::vector<double> storage_terminal_soc;   // [s] SOC target at end of block [0,1]
  double block_cost{0.0};             // schedule-stage cost for this block
};

/// L0 result: the annual plan produced by the coarsest level.
struct AnnualPlanResult {
  std::vector<AnnualPlanBlock> blocks;   // 12 months or 52 weeks
  double total_plan_cost{0.0};
  bool feasible{false};
  std::string solver_name;
};

/// L2: Weekly rolling UC schedule (inherits constraints from L0/L1).
struct WeeklySchedule {
  int week_id{0};
  int start_step{0};
  int num_steps{168};                  // binding window size
  int lookahead_steps{48};             // advisory look-ahead
  UCSchedule uc;                       // full UC result for binding window
  double remaining_energy_budget_mwh{0.0};  // from higher level
};

// ═══════════════════════════════════════════════════════════════════════
// Annual Production-Simulation Result
// ═══════════════════════════════════════════════════════════════════════

/// Per-step snapshot (kept lightweight for 8760 steps).
struct AnnualStepResult {
  double total_gen_mw{0.0};
  double total_load_mw{0.0};
  double total_renewable_mw{0.0};
  double total_curtailment_mw{0.0};
  double total_ess_mw{0.0};        // net: positive=discharge
  double external_grid_net_mw{0.0}; // positive=import, negative=export
  double total_loss_mw{0.0};
  double total_supply_mw{0.0};
  double total_demand_mw{0.0};
  double power_balance_error_mw{0.0};
  double load_shed_mw{0.0};
  bool pf_converged{false};
  bool opf_converged{false};
  double opf_cost{0.0};
};

/// Per-block (monthly/weekly) aggregated metrics.
struct BlockSummary {
  int block_id{0};
  int start_step{0};
  int end_step{0};
  double total_gen_mwh{0.0};
  double total_load_mwh{0.0};
  double total_renewable_mwh{0.0};
  double total_curtailment_mwh{0.0};
  double storage_discharge_mwh{0.0};
  double storage_charge_mwh{0.0};
  double external_grid_import_mwh{0.0};
  double external_grid_export_mwh{0.0};
  double total_supply_mwh{0.0};
  double total_demand_mwh{0.0};
  double power_balance_error_mwh{0.0};
  double total_loss_mwh{0.0};
  double total_ens_mwh{0.0};        // energy not served
  double total_cost{0.0};
  int num_pf_converged{0};
  int num_opf_converged{0};
  int num_steps{0};
};

/// Per-generator annual statistics.
struct GenAnnualStats {
  std::string name;
  int gen_index{0};
  double total_energy_mwh{0.0};
  double capacity_factor{0.0};
  int total_startups{0};
  int total_shutdowns{0};
  double total_hours_online{0.0};
};

/// Per-storage annual statistics.
struct StorageAnnualStats {
  std::string name;
  int storage_index{0};
  double total_charge_mwh{0.0};
  double total_discharge_mwh{0.0};
  double cycles{0.0};
};

/// Per-renewable annual statistics.
struct RenewableAnnualStats {
  std::string name;
  int ren_index{0};
  double total_energy_mwh{0.0};
  double total_curtailed_mwh{0.0};
  double capacity_factor{0.0};
  double curtailment_rate{0.0};     // curtailed / available
};

/// Sampled PF snapshot for dashboard visualization.
struct PFSnapshot {
  int global_step{0};
  std::vector<double> vm;                  // AC bus voltage magnitudes
  std::vector<double> vdc;                 // DC bus voltages
  std::vector<BranchFlow> branch_flows;    // AC branch flows
  std::vector<VSCTransfer> vsc_transfers;
  std::vector<DCDCTransfer> dcdc_transfers;
  bool converged{false};
};

/// Complete annual production-simulation result.
struct AnnualProductionSimResult {
  // Hierarchy outputs
  AnnualPlanResult annual_plan;         // L0 result
  std::vector<WeeklySchedule> weekly_schedules;  // L2 results

  // Per-step results (T_yr entries, one per hour or sub-hour)
  std::vector<AnnualStepResult> step_results;

  // Sampled PF snapshots for voltage/branch-flow visualization
  std::vector<PFSnapshot> pf_snapshots;

  // Block summaries (monthly aggregation)
  std::vector<BlockSummary> monthly_summaries;

  // Component-level annual statistics
  std::vector<GenAnnualStats> gen_stats;
  std::vector<StorageAnnualStats> storage_stats;
  std::vector<RenewableAnnualStats> renewable_stats;

  // Scalar annual metrics
  int num_steps{0};                    // T_yr
  double step_duration_hr{1.0};
  double total_cost{0.0};             // J^yr
  double total_gen_mwh{0.0};
  double total_load_mwh{0.0};
  double total_renewable_mwh{0.0};
  double total_curtailment_mwh{0.0};
  double storage_discharge_mwh{0.0};
  double storage_charge_mwh{0.0};
  double external_grid_import_mwh{0.0};
  double external_grid_export_mwh{0.0};
  double total_supply_mwh{0.0};
  double total_demand_mwh{0.0};
  double power_balance_error_mwh{0.0};
  double total_ens_mwh{0.0};          // energy not served
  double total_loss_mwh{0.0};
  int num_pf_converged{0};
  int num_opf_converged{0};
  bool feasible{false};
  std::string solver_name;
  bool parallel_daily_effective{false};
  int parallel_workers{1};
  std::string parallel_mode;
  util::ParallelExecutionInfo parallel_execution;

  /// Text summary for logging.
  std::string summary() const;
};

// ═══════════════════════════════════════════════════════════════════════
// Options
// ═══════════════════════════════════════════════════════════════════════

/// Decomposition granularity for the upper levels.
enum class AnnualBlockType {
  Monthly,    // 12 blocks — most common
  Weekly,     // 52 blocks — finer L0 resolution
};

/// Per-day simulation model for the parallel daily decomposition.
/// Each calendar day is an independent, energy-neutral horizon (cyclic SOC), so
/// all days can be solved concurrently.
enum class DailySimMode {
  SCUC,         // Security-constrained unit commitment: binary commitment + DC
                // network/line limits, then AC-OPF + PF replay.
  DynamicSCED,  // Dynamic security-constrained economic dispatch: commitment
                // fixed ON, multi-period ED with ramping + network limits.
  DynamicOPF,   // Dynamic optimal power flow: per-step AC-OPF + PF (no UC).
};

struct AnnualProductionSimOptions {
  // Pipeline options inherited per sub-horizon solve
  TimeSeriesPFOptions ts_pf_options;

  // Decomposition control
  AnnualBlockType block_type{AnnualBlockType::Monthly};
  int weekly_lookahead_hours{48};      // L2 look-ahead buffer
  int daily_window_hours{24};          // L3 daily sub-horizon

  // ── Parallel daily decomposition ───────────────────────────────────────
  // When enabled, the year is partitioned into independent calendar days
  // (each an energy-neutral horizon via per-day cyclic SOC) and the days are
  // solved concurrently on a thread pool, bypassing the sequential L0→L3 path.
  bool enable_parallel_daily{false};
  DailySimMode daily_mode{DailySimMode::SCUC};
  int parallel_threads{0};             // 0 = hardware_concurrency()
  bool enforce_daily_cyclic_soc{true}; // pin each day's terminal SOC to its initial
  // Dynamic-SCED commitment source: when true, a representative (peak-load) day
  // is first solved as SCUC and its commitment is reused across all SCED days;
  // when false, every in-service unit is simply forced ON.
  bool sced_reuse_scuc_commitment{true};

  // Cyclic storage boundary enforcement
  bool enforce_cyclic_soc{true};       // E_{s,T-1} = E_{s,0}

  // Iterative feedback (bottom-up → re-solve upper level)
  bool iterative_feedback{false};
  int max_feedback_iterations{3};
  double budget_violation_tol_mwh{10.0};

  // Optional: skip OPF/PF replay (schedule-only mode)
  bool skip_replay{false};

  // PF snapshot interval (store every N steps for dashboard voltage/flow)
  int pf_snapshot_interval{24};  // 0 = disabled

  // Curtailment and load-shedding penalty ($/MWh)
  double curtailment_penalty{50.0};
  double ens_penalty{10000.0};

  bool verbose{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════

/// Solve annual production simulation using hierarchical temporal decomposition.
/// L0 (annual planning) → L1 (monthly) → L2 (weekly UC) → L3 (daily UC + OPF/PF).
AnnualProductionSimResult solve_annual_production_simulation(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimOptions& opts = {});

}  // namespace hacdcpf::analysis

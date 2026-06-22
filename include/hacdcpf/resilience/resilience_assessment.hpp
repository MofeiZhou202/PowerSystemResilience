#pragma once

#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::analysis {

// ── Model selection ──────────────────────────────────────────────────────────

enum class DistributionResilienceModel {
  HeuristicSequential,
  MultiPeriodMIPLinDistFlow,
  RAStyleStageMILP,
};

enum class DistributionResilienceMIPSolver {
  Native,
  HiGHS,
  Gurobi,
};

enum class DistributionResilienceNativeNodeSelection {
  Hybrid,
  BestFirst,
};

// ── MIP options ──────────────────────────────────────────────────────────────

/// @brief Options for the strict multi-period restoration MIP skeleton.
struct DistributionResilienceMIPOptions {
  double v_min_pu{0.95};
  double v_max_pu{1.05};
  double big_m_voltage{4.0};
  double default_branch_rate_mva{10.0};

  double shed_penalty_critical{1.0e6};
  double shed_penalty_high{1.0e4};
  double shed_penalty_medium{1.0e2};
  double shed_penalty_low{1.0};
  double switching_cost{5.0};
  double mess_travel_cost_per_km{0.1};
  double renewable_curtailment_cost{10.0};

  bool allow_multi_root_forest{true};
  bool allow_grid_forming_storage_roots{true};
  bool allow_mess_black_start{true};

  double mip_gap{0.02};
  int max_time_s{120};
  int max_nodes{50000};
  /// <= 0 means auto: use hardware_concurrency capped at 8 threads.
  int num_threads{-1};
  DistributionResilienceMIPSolver solver{DistributionResilienceMIPSolver::Native};
  DistributionResilienceNativeNodeSelection native_node_selection{
      DistributionResilienceNativeNodeSelection::BestFirst};
  bool verbose{false};
  /// Enable CGLP lift-and-project disjunctive cuts in the native B&C solver.
  bool enable_cglp_cuts{false};
};

// ── Model statistics ─────────────────────────────────────────────────────────

struct DistributionResilienceModelStats {
  DistributionResilienceModel model{DistributionResilienceModel::HeuristicSequential};
  int num_variables{0};
  int num_binary_variables{0};
  int num_integer_variables{0};
  int num_eq_constraints{0};
  int num_ineq_constraints{0};
  double objective_value{0.0};
  double mip_gap{0.0};
  double runtime_sec{0.0};
  bool model_built{false};
  bool model_solved{false};
  std::string solver_name;
  std::string solver_status;
  std::string formulation_notes;
  /// Number of CGLP disjunctive cuts admitted.
  int cglp_cuts_added{0};

  /// Structured model-capability declaration.  Always "ac-only-lindistflow"
  /// for the strict MIP path; DC buses / branches / VSC / DC loads are NOT
  /// modelled in the restoration MILP and their contingencies cannot be
  /// scheduled here.  Use this string to gate downstream consumers.
  std::string model_scope{"ac-only-lindistflow"};

  /// Per-feature validity flags so consumers can branch on whether a given
  /// physical constraint was actually enforced.  These reflect what the
  /// MIP skeleton built by `build_mip_skeleton` *does* enforce.
  struct ValidityFlags {
    bool dc_network_modelled{false};
    bool vsc_dispatch_modelled{false};
    bool ac_branch_flow_limits_enforced{true};
    bool lindistflow_voltage_envelope_enforced{true};
    bool radial_topology_enforced{true};
    /// True when the MIP gap reported by the solver is within the requested
    /// tolerance (`DistributionResilienceMIPOptions::mip_gap`).  This does NOT
    /// mean the solution is the global optimum; it means the best known
    /// bound is within `mip_gap * 100`% of the incumbent.  A non-zero gap
    /// tolerance means the certificate is approximate.  Use `model_stats.mip_gap`
    /// for the exact achieved gap.  Renamed from `mip_solved_to_proven_optimum`
    /// which was misleading when users set a non-zero gap tolerance.
    bool mip_gap_within_tolerance{false};
  };
  ValidityFlags validity{};
};

// ── Fault / transport description ────────────────────────────────────────────

/// @brief Distinguishes AC vs DC branch faults (used by typhoon scenario
/// generation, which can place faults on either network).
enum class ResilienceBranchKind {
  AC,
  DC,
};

enum class DistributionDisasterStage {
  Normal,
  DisasterIsolation,
  DisasterPostFaultReconfig,
  PostDisasterRepair,
};

const char* to_string(ResilienceBranchKind kind);
const char* to_string(DistributionDisasterStage stage);
ResilienceBranchKind resilience_branch_kind_from_string(const std::string& value);

/// @brief Description of a single branch fault used to drive the resilience study.
struct DistributionResilienceFault {
  DistributionResilienceFault() = default;
  DistributionResilienceFault(int ac_index, double start_hr, double repair_hr,
                              std::string fault_name = {})
      : ac_branch_index(ac_index),
        branch_kind(ResilienceBranchKind::AC),
        branch_index(ac_index),
        outage_start_hr(start_hr),
        repair_duration_hr(repair_hr),
        name(std::move(fault_name)) {}

  // Legacy field: when branch_index is not set, this is interpreted as an AC branch.
  int ac_branch_index{0};
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  double outage_start_hr{0.0};
  double repair_duration_hr{6.0};
  std::string name;
};

/// @brief Explicit road-network edge for MESS routing.
struct TransportEdge {
  int from_bus{0};
  int to_bus{0};
  double distance_km{1.0};
  bool available{true};
};

// ── Options ──────────────────────────────────────────────────────────────────

/// @brief Configuration for a multi-hour distribution resilience study.
struct DistributionResilienceOptions {
  DistributionResilienceModel model{DistributionResilienceModel::HeuristicSequential};
  int horizon_hours{48};
  double time_step_hr{1.0};
  double load_scale_factor{1.0};
  bool allow_reconfiguration{true};
  bool allow_mess_dispatch{true};
  /// Maximum greedy reconfiguration iterations per step (safety limit).
  int max_reconfig_iterations{50};
  bool use_electrical_graph_as_transport_proxy{true};
  double mess_travel_speed_kmph{40.0};
  int default_fault_count{2};
  double default_repair_time_hr{6.0};
  /// Stagger interval (hr) between auto-generated faults (0 = simultaneous).
  double auto_fault_stagger_hr{0.0};
  /// Hour at which the first auto-generated fault occurs.
  double auto_fault_start_hr{0.0};

  /// Normalized hourly load multipliers (index = hour of day, values 0-1).
  /// If empty, a built-in 24-h residential curve is used.
  std::vector<double> load_profile;

  /// Normalized hourly RES (PV/wind) availability multipliers.
  /// If empty, a built-in daytime PV curve is used.
  std::vector<double> renewable_profile;

  std::vector<DistributionResilienceFault> faults;
  std::vector<TransportEdge> transport_edges;
  std::function<bool(int, double, double)> progress_callback;

  /// If true, run full AC power flow at each step to obtain bus voltages
  /// and branch flows (slower but gives physical validation).
  bool run_power_flow{false};

  /// Use RA-Validation-style disaster stages with switch-constrained topology
  /// MILPs for stage 1 fault isolation and stage 2 post-fault reconfiguration.
  bool use_ra_style_stage_milp{false};
  /// Preferred user-facing toggle for staged disaster isolation/reconfiguration.
  bool enable_disaster_stages{false};
  /// Preferred post-fault reconfiguration window. Kept alongside the legacy
  /// disaster_* spelling used by the standalone reference implementation.
  double post_fault_reconfig_window_hr{2.0};
  double disaster_post_fault_reconfig_window_hr{2.0};
  bool use_switch_based_fault_isolation{true};
  bool allow_stage1_open_switches{true};
  bool allow_stage2_close_ties{true};
  bool require_switch_for_nonfault_branch_operation{true};
  bool allow_branch_operation_without_switch{false};
  bool use_remote_switch_only{false};

  /// Strict restoration MIP options used when model ==
  /// MultiPeriodMIPLinDistFlow.
  DistributionResilienceMIPOptions mip;

  // Legacy fields kept for backwards compatibility with the old simplified API.
  double fault_duration_hr{1.0};
  double curtailment_penalty{1000.0};
  bool enable_islanding{true};
  bool enable_storage_dispatch{true};
  double max_restoration_hr{8.0};
  bool verbose{false};
};

// ── Per-step result types ─────────────────────────────────────────────────────

/// @brief Per-step snapshot of a single MESS unit.
struct MESSStateStep {
  int storage_index{0};
  int bus{0};
  int target_bus{0};
  std::string status;
  double dispatch_mw{0.0};
  double energy_mwh{0.0};
  double soc{0.0};
  double arrival_time_hr{0.0};
  double remaining_travel_hr{0.0};
};

struct BranchFlowStep {
  int branch_index{0};
  int from_bus{0};
  int to_bus{0};
  double pf_mw{0.0};
  double pt_mw{0.0};
  double qf_mvar{0.0};
  double qt_mvar{0.0};
  double loading_percent{0.0};
};

struct BusVoltageStep {
  int bus_index{0};
  double vm_pu{1.0};
  double va_deg{0.0};
};

/// @brief Results for a single time step of the resilience simulation.
struct DistributionResilienceStepResult {
  int step_index{0};
  double hour{0.0};
  double load_multiplier{1.0};
  double res_multiplier{0.0};
  double total_demand_mw{0.0};
  double served_mw{0.0};
  double shed_mw{0.0};
  double restoration_ratio{1.0};
  double weighted_shed_mw{0.0};
  double total_res_mw{0.0};
  int island_count{0};
  int active_faults{0};
  int repaired_faults{0};
  int switch_actions{0};
  std::string disaster_stage{"Normal"};
  std::vector<int> open_ac_branch_ids;
  std::vector<int> open_dc_branch_ids;
  std::vector<int> closed_tie_branch_ids;
  std::vector<int> closed_dc_tie_branch_ids;
  std::vector<int> isolation_open_ac_branch_ids;
  std::vector<int> isolation_open_dc_branch_ids;
  std::vector<int> open_switch_ids;
  std::vector<int> closed_switch_ids;
  std::vector<int> switched_open_ids;
  std::vector<int> switched_closed_ids;
  std::vector<int> closed_tie_switch_ids;
  std::vector<int> open_breaker_ids;
  std::vector<int> closed_tie_breaker_ids;
  int switch_open_actions{0};
  int switch_close_actions{0};
  int isolation_switch_actions{0};
  int reconfiguration_switch_actions{0};
  std::vector<int> fault_zone_ac_bus_ids;
  std::vector<int> fault_zone_dc_bus_ids;
  std::vector<MESSStateStep> mess_states;

  /// Shed MW broken down by priority tier [Critical, High, Medium, Low].
  std::vector<double> shed_by_priority;

  // Power flow validation results (populated when run_power_flow == true).
  bool pf_converged{false};
  double pf_residual{0.0};
  std::vector<BusVoltageStep> bus_voltages;
  std::vector<BranchFlowStep> branch_flows;

  /// Bus IDs of cut-vertices (articulation points) in the current topology.
  /// Removing any of these buses would split the connected network further.
  /// Useful for prioritizing repair crew dispatch.
  std::vector<int> cut_vertex_bus_ids;

  /// Branch IDs of bridges (cut-edges) in the current topology.
  /// Restoring a failed bridge reconnects the largest sub-tree downstream.
  std::vector<int> bridge_branch_ids;
};

struct FaultSequenceEntry {
  ResilienceBranchKind branch_kind{ResilienceBranchKind::AC};
  int branch_index{0};
  double start_hr{0.0};
  double repair_hr{6.0};
  std::string name;
};

// ── Aggregate result ──────────────────────────────────────────────────────────

/// @brief Aggregated results for the entire resilience assessment run.
struct DistributionResilienceResult {
  std::vector<DistributionResilienceStepResult> steps;
  /// Actual fault sequence used (auto-generated or user-specified).
  std::vector<FaultSequenceEntry> fault_sequence;

  DistributionResilienceModel model{DistributionResilienceModel::HeuristicSequential};
  DistributionResilienceModelStats model_stats;

  double total_demand_mwh{0.0};
  double total_served_mwh{0.0};
  double total_shed_mwh{0.0};
  double weighted_unserved_mwh{0.0};
  double resilience_index{0.0};
  double avg_restoration_ratio{0.0};
  double final_restoration_ratio{0.0};
  double peak_shed_mw{0.0};
  double mess_energy_delivered_mwh{0.0};
  double mess_travel_distance_km{0.0};
  int total_switch_actions{0};
  int total_repaired_faults{0};
  bool feasible{true};

  // Legacy fields kept for backward compatibility.
  bool completed{false};
  double total_ens_mwh{0.0};
  double max_curtailment_mw{0.0};
  double restoration_time_hr{0.0};

  std::string status;

  std::string summary() const;
};

// ── Functions ─────────────────────────────────────────────────────────────────

/// Fill missing resilience-relevant fields with plausible demo values.
void apply_distribution_resilience_demo_data(HybridPowerSystem& sys);

/// MIP-based multi-period restoration. Builds a binary MILP with branch-switching
/// variables, bus energisation variables, and commodity flow constraints, then
/// solves it with the registered MILP backend (Gurobi / HiGHS / NativeBranchAndCut).
/// Does NOT delegate to the heuristic path.
DistributionResilienceResult run_distribution_resilience_mip_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts = {});

/// RA-style staged disaster isolation / post-fault reconfiguration assessment.
DistributionResilienceResult run_distribution_resilience_stage_milp_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts = {});

/// Run the full multi-hour distribution resilience assessment.
DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& sys,
    const DistributionResilienceOptions& opts = {});

}  // namespace hacdcpf::analysis

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

// Time-series data types used by io and analysis modules.
// TimeSeriesData/TimeSeriesProfile are in the global hacdcpf namespace for
// backward-compatibility with the json_io serialisation functions.

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Time-Series Profile — per-time-step scaling factors
// ═══════════════════════════════════════════════════════════════════════
struct TimeSeriesProfile {
  int id{0};
  std::string name;
  std::vector<double> values;  // one per time step (scaling factors)
};

// ═══════════════════════════════════════════════════════════════════════
// Time-Series Input Data — bundles profiles with the system
// ═══════════════════════════════════════════════════════════════════════
struct TimeSeriesData {
  int num_steps{24};
  double step_duration_hr{1.0};
  std::vector<TimeSeriesProfile> profiles;
};

// ═══════════════════════════════════════════════════════════════════════
// UC Schedule — output from unit commitment (Phase II)
// ═══════════════════════════════════════════════════════════════════════
struct UCSchedule {
  std::vector<std::vector<double>> gen_dispatch;       // [g][t] MW
  std::vector<std::vector<int>>    gen_commit;         // [g][t] 0/1
  std::vector<std::vector<double>> ess_dispatch;       // [s][t] MW (+discharge)
  std::vector<std::vector<double>> ess_soc;            // [s][t] SOC [0,1]
  std::vector<std::vector<double>> renewable_dispatch; // [r][t] MW

  // DC-side component dispatch
  // - pv/static-gen/load: profile-driven exogenous series
  // - dc_ess_dispatch: UC-optimized when UC is enabled; otherwise fixed replay
  std::vector<std::vector<double>> dc_pv_dispatch;     // [k][t] MW
  std::vector<std::vector<double>> dc_ess_dispatch;    // [k][t] MW
  std::vector<std::vector<double>> dc_sgen_dispatch;   // [k][t] MW
  std::vector<std::vector<double>> dc_load_demand;     // [k][t] MW

  // Converter dispatch (populated when enable_dc_network_constraints=true)
  std::vector<std::vector<double>> vsc_dispatch;       // [c][t] MW (AC-side injection)
  std::vector<std::vector<double>> dcdc_dispatch;      // [dd][t] MW (bus_in→bus_out)

  double total_cost{0.0};
  bool feasible{false};
  std::string solver_name;
};

// ═══════════════════════════════════════════════════════════════════════
// Per-timestep cross-validation metrics (OPF vs PF)
// ═══════════════════════════════════════════════════════════════════════
struct CrossValStep {
  double max_vm_diff{0.0};     // max |Vm_opf - Vm_pf| across buses (pu)
  double max_va_diff{0.0};     // max |Va_opf - Va_pf| relative to slack (rad)
  double opf_objective{0.0};   // OPF cost for this step
  double opf_loss_mw{0.0};     // total losses from OPF
  double pf_loss_mw{0.0};      // total losses from validation PF
  double loss_diff_mw{0.0};    // pf_loss - opf_loss
  bool opf_converged{false};
  bool pf_converged{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Time-Series PF Options — controls for the entire UC→OPF→PF pipeline
// ═══════════════════════════════════════════════════════════════════════
enum class UCSolverChoice { Auto, Native, HiGHS, Gurobi };

struct TimeSeriesPFOptions {
  PowerFlowOptions pf_options;
  opf::ACOPFOptions opf_options;
  UCSolverChoice uc_solver{UCSolverChoice::Auto};
  bool skip_uc{false};
  // When true, UC uses nodal DC network constraints with line thermal limits.
  bool enable_network_constraints{false};
  // Spinning reserve requirement as fraction of total load per time step.
  // Example: 0.10 means 10% upward reserve.
  double reserve_requirement_fraction{0.0};
  // When true, models DC network nodal balance + VSC/DCDC converter coupling in UC MILP.
  // Requires enable_network_constraints=true. Adds DC bus voltage variables, converter
  // power variables, and explicit AC-DC coupling constraints.
  // Note: explicit DC branch thermal-limit constraints are NOT added — DC branch flows
  // are bounded implicitly through the per-converter power variable bounds, which serve
  // as the effective capacity constraint for each DC link.
  bool enable_dc_network_constraints{false};
  bool run_opf{true};         // when true: UC → OPF → PF validation pipeline
  // UC→OPF tracking band (around UC dispatch) for online units:
  //   pmin = max(original_pmin, p_uc - band)
  //   pmax = min(original_pmax, p_uc + band)
  //   band = max(uc_opf_tracking_abs_mw, uc_opf_tracking_rel * max(|p_uc|, 1.0))
  // Set enable_uc_opf_tracking_band=false to disable and let OPF freely redispatch.
  bool enable_uc_opf_tracking_band{true};
  double uc_opf_tracking_rel{0.20};
  double uc_opf_tracking_abs_mw{5.0};
  // Non-slack generators can be constrained more tightly than slack generators
  // to avoid large single-unit redispatch.
  bool enforce_non_slack_strict_tracking{true};
  double non_slack_tracking_rel{0.02};
  double non_slack_tracking_abs_mw{1.0};
  bool verbose{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Time-Series PF Result — full UC → OPF → PF validation pipeline
// ═══════════════════════════════════════════════════════════════════════
struct TimeSeriesPFResult {
  UCSchedule uc_schedule;

  // OPF results per timestep (empty if run_opf=false)
  std::vector<opf::ACOPFResult> opf_results;
  int num_opf_converged{0};

  // PF validation results per timestep
  std::vector<PowerFlowResult> pf_results;
  int num_converged{0};

  // Cross-validation metrics (OPF vs PF, one per timestep)
  std::vector<CrossValStep> crossval;

  int num_steps{0};
  double total_generation_cost{0.0};

  /// Wall-clock time for UC MILP solve only (seconds).
  double uc_solve_sec{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Public API
// ═══════════════════════════════════════════════════════════════════════

/// Phase II: Solve unit commitment (DC power flow based MILP).
UCSchedule solve_unit_commitment(const HybridPowerSystem& sys,
                                 const TimeSeriesData& ts_data,
                                 const TimeSeriesPFOptions& opts = {});

/// Full pipeline: UC → OPF (per step) → PF validation (per step).
/// When run_opf=false, behaves as before: UC → PF directly.
TimeSeriesPFResult solve_time_series_pf(const HybridPowerSystem& sys,
                                         const TimeSeriesData& ts_data,
                                         const TimeSeriesPFOptions& opts = {});

}  // namespace hacdcpf

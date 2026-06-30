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
  std::vector<std::vector<double>> dc_ess_soc;         // [k][t] SOC [0,1]
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
enum class UCSolverChoice { Auto, Native, HiGHS, SCIP, Gurobi };

// Unit-commitment objective selection.  All terms are linear, so any choice
// keeps the model a MILP solvable by every backend.  The reported $ cost is
// always recomputed from the resulting dispatch, independent of this choice.
//   * Cost          — minimize fuel + no-load + startup (classic, default).
//   * Carbon        — minimize generator CO2 (emission_factor_tco2_mwh).
//   * MinCurtailment— maximize renewable utilization (minimize curtailment).
//   * MinLoss       — generation-minimization proxy (true loss needs the
//                     explicit-DC model; the UC DC power flow is lossless).
//   * Weighted      — w_cost·cost + w_carbon·CO2 + w_loss·gen − w_curtailment·RES.
enum class UCObjective { Cost, Carbon, MinCurtailment, MinLoss, Weighted };

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
  // Per-horizon cyclic storage boundary: when true, every storage unit's SOC at
  // the final step is pinned to its initial SOC (e_{s,T-1}=soc_init).  This is
  // what makes day-decomposed production simulation horizons independent and
  // therefore safe to solve in parallel.  Implemented as a variable-bound tweak
  // on the last SOC variable (no extra constraint rows).
  bool enforce_terminal_soc_cyclic{false};
  // Fix unit commitment: force every in-service generator committed (u_{g,t}=1)
  // for the whole horizon, turning the SCUC MILP into a pure (security-constrained)
  // economic-dispatch problem.  Used by the "dynamic SCED" daily mode.
  bool fix_commitment{false};
  // Optional explicit commitment to pin (non-owning; must outlive the solve).
  // Indexed [g_active][t] over in-service generators in their natural order,
  // matching UCSchedule::gen_commit.  When set, each u_{g,t} is fixed to the
  // scheduled 0/1 value (overrides fix_commitment).  Lets dynamic SCED reuse a
  // representative SCUC day's commitment instead of forcing every unit ON.
  const std::vector<std::vector<int>>* fixed_commitment_schedule{nullptr};
  // Unit-commitment objective selection + weights for the Weighted mode.
  UCObjective objective_mode{UCObjective::Cost};
  double w_cost{1.0};
  double w_carbon{0.0};
  double w_loss{0.0};
  double w_curtailment{0.0};
  // Model external grids (substation ties) as price-aware exchange sources in
  // the UC: each in-service ExternalGrid gets a net-exchange variable
  // (+import / −export) priced by price_profile_id × cost_c1 (or static cost_c1).
  // Opt-in so existing cases (where generators cover all load) are unaffected.
  bool enable_external_grid{false};
  // Big-M cap on |external-grid exchange| (MW) when no explicit limit exists.
  double external_grid_cap_mw{1.0e5};
  // Demand response on FlexibleLoad: served demand may deviate from baseline by
  // up to flex_up_mw / flex_down_mw (× availability) via non-negative up/down
  // variables.  A discomfort penalty (w_demand_response · (up+down)) discourages
  // unnecessary shifting.  When dr_shiftable is true the per-load net deviation
  // over the horizon is constrained to zero (energy-conserving load shifting).
  // Opt-in so existing cases are unaffected (flexible loads stay at baseline).
  bool enable_demand_response{false};
  bool dr_shiftable{false};
  double w_demand_response{20.0};
  // Dispatchable (curtailable) distributed PV / static generation.  The
  // otherwise must-take sources (AC PVSystem, DC PVArrayDC, DC static gens) get
  // a curtailment "claw-back" variable in [0, available], priced by
  // pv_curtail_penalty, so the optimiser can curtail them under reverse-power /
  // oversupply / voltage limits instead of forcing must-take injection.
  // Opt-in; when off (default) the must-take behaviour is unchanged.
  bool enable_dispatchable_pv{false};
  double pv_curtail_penalty{50.0};
  // Microgrid PCC exchange + islanding.  Each in-service Microgrid gets a net
  // exchange variable at its PCC bus (+export / −import, bounded by
  // p_export_max / p_import_max) and a binary connection indicator o (1 =
  // grid-connected).  Exchange is gated to zero when islanded (o = 0); the
  // optimiser may island, penalised by w_microgrid_island per islanded step.
  // Microgrids without islanding_capability are pinned grid-connected.
  bool enable_microgrid{false};
  double w_microgrid_island{100.0};
  // Explicit DC branch flows + thermal limits (requires enable_dc_network_
  // constraints).  Adds a transport flow variable per DC branch bounded by its
  // rating (|f| ≤ rate_a_mva), entering the DC nodal balance so power can move
  // between DC buses up to the line capacity, replacing the per-bus self-balance
  // of the flat-voltage model.  Converter losses stay proportional (via eta).
  bool enable_dc_branch_flows{false};
  // Storage cycle-aging (degradation) cost.  Charges throughput at
  // replacement_cost / (2·max_cycles·E_rated) per MWh moved (via an absolute-
  // value auxiliary on the storage power), discouraging unnecessary cycling.
  // When a unit's daily_cycle_limit > 0, a hard cap Σ|p|·dt ≤ 2·limit·E is added
  // over the horizon.  Covers AC and DC storage.  Opt-in (default off).
  bool enable_storage_degradation{false};
  // Virtual power plant aggregate dispatch.  Each in-service VirtualPowerPlant
  // gets a net-output variable at its PCC bus bounded by [pmin_mw, pmax_mw] with
  // ramp limits, plus an aggregate energy state bounded by e_storage_sum_mwh
  // (the storage envelope) so its sustained output is energy-limited.  Opt-in.
  bool enable_vpp{false};
  // Energy router (multi-port converter) aggregate dispatch.  Each in-service
  // EnergyRouter that still carries explicit ports (i.e. has not been pre-
  // expanded into VSC + DC/DC converters during network projection) gets, per
  // port, a signed net injection at the port's bus modelled as an inflow/outflow
  // pair, coupled by a lossy internal power-conservation constraint
  // (Σ_j η_j·p_in_j = Σ_j p_out_j).  This realises the N-port generalisation of
  // the two-port VSC/DC-DC coupling.  Opt-in (default off); a no-op when no
  // router survives projection.
  bool enable_energy_router{false};
  // Mobile storage with an exogenous relocation schedule.  Each in-service
  // MobileStorage gets a signed power variable (+discharge) and an energy state
  // bounded by [soc_min, soc_max].  Its connection bus follows a schedule
  // derived from departure_time / arrival_time / status: it injects at its
  // current bus before departure, is disconnected (power forced to 0) while in
  // transit, and injects at target_bus after arrival.  The relocation timing is
  // taken as given input rather than co-optimised (no per-bus assignment
  // binaries), which keeps the model an LP-sized add-on.  Opt-in (default off).
  bool enable_mobile_storage{false};
  // Co-optimised mobile-storage relocation (requires enable_mobile_storage).
  // Replaces the fixed schedule with a decision: per step the unit is connected
  // at its origin bus, connected at target_bus, or in transit (binaries
  // z0/z1/w with z0+z1+w=1).  Power is injected only at the connected bus
  // (q0/q1 gated by z0/z1); leaving a bus forces a transit step; a per-transit
  // travel drain depletes the battery; and a minimum-stay (from t_stay_min_hr)
  // prevents oscillation.  A larger MILP (binaries per unit-step); default off.
  bool mobile_storage_corelocate{false};
  // Per-day parallel decomposition.  When set and the horizon spans more than one
  // scheduling day, the time-series solve is split into independent days (each
  // with cyclic terminal SOC) that are solved concurrently and stitched back
  // together — the same speedup the annual simulation uses, now available to a
  // plain multi-day run.  A single-day (or sub-day) horizon stays a coupled
  // solve.  Concurrency is dispatched lock-free only when the MILP backend is
  // instance-isolated (SCIP) or no MILP is used (skip_uc); otherwise it falls
  // back to a sequential day loop.  Default off (the coupled solve).
  bool parallel_daily{false};
  int parallel_threads{0};            // 0 = hardware_concurrency()
  bool keep_system_snapshots{false};
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

  // Actual system states used for PF validation when keep_system_snapshots is true.
  std::vector<HybridPowerSystem> pf_system_snapshots;

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

HybridPowerSystem build_time_series_system_snapshot(
    const HybridPowerSystem& base_sys,
    const TimeSeriesData& ts_data,
    const UCSchedule& schedule,
    int step,
    const TimeSeriesPFOptions& opts = {});

/// Full pipeline: UC → OPF (per step) → PF validation (per step).
/// When run_opf=false, behaves as before: UC → PF directly.
TimeSeriesPFResult solve_time_series_pf(const HybridPowerSystem& sys,
                                         const TimeSeriesData& ts_data,
                                         const TimeSeriesPFOptions& opts = {});

}  // namespace hacdcpf

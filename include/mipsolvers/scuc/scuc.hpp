#pragma once

/// Security-Constrained Unit Commitment (SCUC) / SCED / LMP module.
///
/// Implements the Southern China Regional Electricity Market Spot Trading
/// Rules 2025 V1.0, §2.6 day-ahead clearing formulation.
///
/// Standalone MILP formulation that accepts JSON input describing a
/// power-market day-ahead clearing problem and writes JSON output.
/// Any solver registered with mipsolvers::engine::SolverEngine can be
/// selected at runtime via the "solver" field in the JSON config block.
///
/// Usage:
///   auto input  = mipsolvers::scuc::scuc_from_json(json_string);
///   auto output = mipsolvers::scuc::scuc_solve(input);
///   std::string json = mipsolvers::scuc::scuc_output_to_json(output, input);

#include <string>
#include <utility>
#include <vector>

namespace mipsolvers::scuc {

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

/// One segment of a piecewise-linear generator bid curve.
struct BidSegment {
  double price{0.0};     ///< Marginal cost ($/MWh)
  double quantity{0.0};  ///< Capacity in this segment (MW above Pmin)
};

/// Thermal / dispatchable generator.
struct Generator {
  std::string name;
  int bus{0};                        ///< 0-based bus index
  double pmin{0.0};                  ///< MW
  double pmax{0.0};                  ///< MW
  double ramp_up_mw_min{0.0};        ///< MW/minute
  double ramp_dn_mw_min{0.0};        ///< MW/minute
  double min_up_time_hr{0.0};        ///< hours
  double min_dn_time_hr{0.0};        ///< hours
  bool must_run{false};
  int max_startups{0};               ///< 0 = unlimited
  int max_shutdowns{0};              ///< 0 = unlimited
  std::vector<BidSegment> bid_segments;

  /// Startup cost (§2.6.3.13): state-dependent hot/warm/cold.
  /// If startup_cost_hot = 0, falls back to startup_cost for all states.
  double startup_cost{0.0};          ///< $ per start (legacy / hot-start cost)
  double startup_cost_warm{0.0};     ///< $ per warm start (0 = use startup_cost)
  double startup_cost_cold{0.0};     ///< $ per cold start (0 = use startup_cost)
  double hot_start_threshold_hr{4.0};   ///< Max off-time for hot start (hours)
  double warm_start_threshold_hr{8.0};  ///< Max off-time for warm start (hours)

  /// Startup/shutdown trajectory durations (§2.6.3.8).
  int ud_periods{0};  ///< Startup process duration (periods from sync to Pmin)
  int dd_periods{0};  ///< Shutdown process duration (periods from Pmin to desync)

  double no_load_cost{0.0};          ///< $/h while committed (minimum-output cost)
  double spinning_reserve_price{0.0};
  double regulation_up_price{0.0};
  double regulation_down_price{0.0};

  /// Primary frequency regulation coefficient: max PFR = pfr_alpha * pmax (§2.6.3.4).
  double pfr_alpha{0.0};             ///< 0 = unit does not provide PFR

  /// Generator group membership: -1 = no group.
  int group_id{-1};
};

/// AC transmission branch (line or transformer).
struct Branch {
  int from{0};             ///< 0-based
  int to{0};               ///< 0-based
  double reactance{0.05};  ///< pu
  double rating_mw{1e6};   ///< thermal limit (MW)
  bool in_service{true};
};

/// Load (demand) at a bus.
struct Load {
  int bus{0};
  double p_mw{0.0};  ///< Base active demand (MW); scaled by profile at runtime
};

/// Wind generation unit.
struct WindUnit {
  int bus{0};
  double pmax{0.0};  ///< Rated capacity (MW); profile gives actual forecast MW
};

/// Solar (PV) generation unit.
struct SolarUnit {
  int bus{0};
  double pmax{0.0};
};

/// Battery / pumped-hydro storage unit.
struct StorageUnit {
  std::string name;
  int bus{0};
  double pmax_charge{0.0};       ///< Maximum charging rate (MW)
  double pmax_discharge{0.0};    ///< Maximum discharging rate (MW)
  double pmin_charge{0.0};       ///< Minimum charging rate when charging (MW)
  double pmin_discharge{0.0};    ///< Minimum discharging rate when discharging (MW)
  double energy_capacity_mwh{0.0};
  double efficiency{0.9};        ///< Round-trip efficiency (legacy, used if eta_charge/discharge < 0)

  /// Separate charging/discharging efficiencies (§2.6.3.16).
  /// Values < 0 → derive from sqrt(efficiency).
  double eta_charge{-1.0};       ///< Charging efficiency η^ch (0–1; <0 = use sqrt(efficiency))
  double eta_discharge{-1.0};    ///< Discharging efficiency η^dis (0–1; <0 = use sqrt(efficiency))

  double soc_init{0.5};          ///< Initial SOC (fraction of capacity, or MWh if > 1)
  double soc_min{-1.0};          ///< Minimum SOC fraction (<0 → use 0.10)
  double soc_final{-1.0};        ///< Required final SOC fraction (<0 → use soc_init)

  double charge_bid_price{0.0};      ///< λ^ch ($/MWh)
  double discharge_bid_price{0.0};   ///< λ^dis ($/MWh)
  double cycle_limit{0.0};           ///< N^cycle, max daily equivalent full-cycles (0 = unlimited)

  /// Use binary charge/discharge mode indicators (ξ+/ξ-) in SCUC MILP.
  /// Ensures mutual exclusion exactly; increases model size.
  bool use_binary_indicators{false};
};

/// VSC-HVDC or controllable DC line.
struct DCLine {
  int from{0};         ///< 0-based AC bus (injection point)
  int to{0};           ///< 0-based AC bus
  double pmin{-1e6};   ///< MW
  double pmax{1e6};    ///< MW
  double ramp_up{1e6};
  double ramp_dn{1e6};
};

/// Generator group: shared output or energy limits (§2.6.3.9–2.6.3.10).
struct GeneratorGroup {
  int id{-1};
  std::string name;
  std::vector<int> gen_indices;   ///< 0-based indices into SCUCInput::generators

  /// Per-period output limits [MW].  Empty → no constraint.
  std::vector<double> pmin_t;     ///< [T] or [1] (scalar applied to all periods)
  std::vector<double> pmax_t;     ///< [T] or [1]

  /// Horizon energy limits [MWh].  0 → no constraint.
  double emin{0.0};
  double emax{0.0};
};

/// Monitored network section (aggregate transmission corridor) (§2.6.3.15).
/// Section flow = weighted sum of constituent line flows.
struct Section {
  std::string name;
  /// Pairs of (0-based branch index, weight).
  /// Section PTDF_s = sum_l weight_l * PTDF_l
  std::vector<std::pair<int, double>> line_weights;
  double rating_fwd_mw{1e6};   ///< Forward (positive) flow limit [MW]
  double rating_rev_mw{1e6};   ///< Reverse (negative) flow limit [MW]
};

// ─────────────────────────────────────────────────────────────────────────────
// Solve configuration
// ─────────────────────────────────────────────────────────────────────────────

struct SCUCConfig {
  /// Solver name: "Auto", "StrictHiGHS", "Gurobi", "HiGHS", "SCIP", "NativeBranchAndCut"
  std::string solver{"Auto"};
  bool allow_fallback{true};  ///< Fall back to next available solver on failure

  int num_periods{24};              ///< Dispatch periods (T)
  double period_length_hr{1.0};     ///< Hours per dispatch period
  int n_segments{3};                ///< Bid curve segments per generator

  double mip_gap{0.001};           ///< Relative MIP optimality gap
  double time_limit_sec{300.0};
  bool verbose{false};

  // ── Reserve requirements ──────────────────────────────────────────────────
  /// Positive (upward) reserve requirement: fraction of system load (§2.6.3.2)
  double spinning_reserve_req{0.10};
  double regulation_up_req{0.05};
  double regulation_down_req{0.05};
  /// Negative (downward) reserve requirement: fraction of system load (§2.6.3.3).
  /// 0 = no negative reserve constraint.
  double neg_reserve_req{0.0};
  /// Primary frequency regulation reserve requirement [MW] (§2.6.3.4).
  /// 0 = no PFR constraint.
  double pfr_reserve_req_mw{0.0};

  // ── Penalty and cost parameters ───────────────────────────────────────────
  /// Value of lost load $/MWh (load-shedding penalty)
  double voll{10000.0};
  /// Value of over-generation curtailment $/MWh (system-level gen curtailment)
  double vocc{1000.0};
  /// Minimum renewable output enforced: P_w >= alpha * forecast (§2.6.3.20)
  double renewable_min_output_coeff{0.0};
  /// M2: renewable curtailment penalty ($/MW per period) (§2.6.3 objective)
  double M2_renewable_curtail_penalty{0.0};
  /// Wheeling fee (inter-provincial transmission cost) $/MWh (P_gwf in tex).
  /// Applied to all thermal generator output.  0 = disabled.
  double wheeling_fee_per_mwh{0.0};

  /// Add LP-valid pre-formulation cutting planes to SCUC
  bool enable_market_cuts{true};

  /// Big-M penalty for line/section flow slack variables (§2.6.3.14–2.6.3.15)
  double M1_line_slack_penalty{1.0e5};

  // ── Post-SCUC stages ──────────────────────────────────────────────────────
  bool solve_sced{true};  ///< Re-solve as LP with fixed commitment (§2.6.4)
  bool solve_lmp{true};   ///< Compute nodal LMP from SCED duals (§2.6.5–2.6.6)

  /// Delta-neighbourhood factor for LMP re-dispatch (§2.6.5.5)
  double lmp_delta{0.10};
};

// ─────────────────────────────────────────────────────────────────────────────
// Profiles and initial status
// ─────────────────────────────────────────────────────────────────────────────

/// Time-series profiles (per-unit or absolute MW depending on context).
struct SCUCProfiles {
  /// load[d][t] = per-unit multiplier applied to Load[d].p_mw at period t
  std::vector<std::vector<double>> load;
  /// wind[w][t] = actual forecast MW for WindUnit w at period t
  std::vector<std::vector<double>> wind;
  /// solar[s][t] = actual forecast MW for SolarUnit s at period t
  std::vector<std::vector<double>> solar;
};

/// Generator and storage state at the start of the planning horizon.
struct SCUCInitialStatus {
  std::vector<double> commitment;   ///< [ng] 0 or 1 — on/off status before period 1
  std::vector<double> dispatch;     ///< [ng] MW dispatched in period 0
  std::vector<double> storage_soc;  ///< [nstorage] fraction of capacity
  /// [ng] number of consecutive hours the unit has been in its current on/off state
  /// (positive = on-time, negative = off-time).  Used for hot/warm/cold start detection.
  std::vector<double> time_in_state;
};

// ─────────────────────────────────────────────────────────────────────────────
// Full problem input
// ─────────────────────────────────────────────────────────────────────────────

struct SCUCInput {
  SCUCConfig config;
  int num_buses{1};  ///< Total number of AC buses (auto-inferred if 0)
  std::vector<Generator>       generators;
  std::vector<Branch>          branches;
  std::vector<Load>            loads;
  std::vector<WindUnit>        wind;
  std::vector<SolarUnit>       solar;
  std::vector<StorageUnit>     storage;
  std::vector<DCLine>          dc_lines;
  std::vector<GeneratorGroup>  generator_groups;  ///< §2.6.3.9–2.6.3.10
  std::vector<Section>         sections;          ///< §2.6.3.15 monitored sections
  SCUCProfiles                 profiles;
  SCUCInitialStatus            initial_status;
};

// ─────────────────────────────────────────────────────────────────────────────
// Result types
// ─────────────────────────────────────────────────────────────────────────────

/// 2-D matrix stored as vector-of-rows [nrows][ncols].
using Matrix2D = std::vector<std::vector<double>>;

/// SCUC (or SCED) solve result.
struct SCUCSolveResult {
  bool converged{false};
  double objective{0.0};
  std::string solver_name;
  double solve_time_sec{0.0};
  double mip_gap{0.0};
  int n_cuts_added{0};  ///< Pre-formulation market cuts added

  // Commitment-horizon variables: [ng][T_commit]
  Matrix2D commitment;  ///< Binary on/off per generator per commit-period
  Matrix2D startup;     ///< Binary startup indicator
  Matrix2D shutdown;    ///< Binary shutdown indicator

  // Dispatch-horizon variables: [ng][T]
  Matrix2D dispatch;
  Matrix2D spinning_reserve;
  Matrix2D regulation_up;
  Matrix2D regulation_down;

  // Segment dispatch: segment_dispatch[k][ng][T]
  std::vector<Matrix2D> segment_dispatch;

  // Renewable generation: [nw or npv][T]
  Matrix2D wind_generation;
  Matrix2D solar_generation;
  /// Renewable curtailment: wind_curtailment[w][t] = forecast[w][t] - wind_generation[w][t]
  Matrix2D wind_curtailment;
  Matrix2D solar_curtailment;

  // Storage: [nstorage][T]
  Matrix2D storage_charging;
  Matrix2D storage_discharging;
  Matrix2D storage_soc;

  // Transmission: [nl][T]
  Matrix2D line_flows;
  /// Section flows [nsec][T]
  Matrix2D section_flows;

  // System-level [T]
  std::vector<double> load_shedding;
  std::vector<double> gen_curtailment;

  // Cost decomposition
  double energy_cost{0.0};
  double startup_cost{0.0};
  double no_load_cost{0.0};
  double reserve_cost{0.0};
  double penalty_cost{0.0};
  double total_cost{0.0};
};

/// Locational Marginal Price result.
struct SCUCLMPResult {
  bool converged{false};
  double solve_time_sec{0.0};
  Matrix2D nodal_lmp;      ///< [nb][T] $/MWh
  Matrix2D energy_lmp;     ///< [nb][T] system marginal price component
  Matrix2D congestion_lmp; ///< [nb][T] congestion rent component
  double avg_lmp{0.0};
  double max_lmp{0.0};
  double min_lmp{0.0};
};

/// Aggregated output for the full SCUC pipeline.
struct SCUCOutput {
  SCUCSolveResult scuc;  ///< Day-ahead unit commitment (MILP)
  SCUCSolveResult sced;  ///< Economic dispatch (LP, only if config.solve_sced)
  SCUCLMPResult   lmp;   ///< Nodal prices (only if config.solve_lmp)
};

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/// Parse SCUCInput from a JSON string (see docs/scuc_schema.md for format).
SCUCInput scuc_from_json(const std::string& json_str);

/// Solve the full SCUC → SCED → LMP pipeline.
/// Solver choice is taken from input.config.solver.
SCUCOutput scuc_solve(const SCUCInput& input);

/// Serialise SCUCOutput to a JSON string.
/// @param indent  JSON indent level (0 = compact, 2 = readable)
std::string scuc_output_to_json(const SCUCOutput& output,
                                const SCUCInput& input,
                                int indent = 2);

}  // namespace mipsolvers::scuc

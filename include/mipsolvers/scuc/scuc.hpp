#pragma once

/// Security-Constrained Unit Commitment (SCUC) / SCED / LMP module.
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
  double startup_cost{0.0};          ///< $ per start
  double no_load_cost{0.0};          ///< $/h while committed
  double spinning_reserve_price{0.0};
  double regulation_up_price{0.0};
  double regulation_down_price{0.0};
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
  double pmax{0.0};  ///< Rated capacity (MW); profile gives per-unit factor
};

/// Solar (PV) generation unit.
struct SolarUnit {
  int bus{0};
  double pmax{0.0};
};

/// Battery / pumped-hydro storage unit.
struct StorageUnit {
  int bus{0};
  double pmax_charge{0.0};       ///< Maximum charging rate (MW)
  double pmax_discharge{0.0};    ///< Maximum discharging rate (MW)
  double energy_capacity_mwh{0.0};
  double efficiency{0.9};        ///< Round-trip efficiency (0–1)
  double soc_init{0.5};          ///< Initial state of charge (fraction of capacity)
  double charge_bid_price{0.0};
  double discharge_bid_price{0.0};
  double cycle_limit{0.0};       ///< Max daily equivalent full-cycles (0 = unlimited)
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

// ─────────────────────────────────────────────────────────────────────────────
// Solve configuration
// ─────────────────────────────────────────────────────────────────────────────

struct SCUCConfig {
  /// Solver name: "Auto", "Gurobi", "HiGHS", "SCIP", "NativeBranchAndCut"
  std::string solver{"Auto"};
  bool allow_fallback{true};  ///< Fall back to next available solver on failure

  int num_periods{24};              ///< Dispatch periods (T)
  double period_length_hr{1.0};     ///< Hours per dispatch period
  int n_segments{3};                ///< Bid curve segments per generator

  double mip_gap{0.001};           ///< Relative MIP optimality gap
  double time_limit_sec{300.0};
  bool verbose{false};

  // Reserve requirements (fraction of system load)
  double spinning_reserve_req{0.10};
  double regulation_up_req{0.05};
  double regulation_down_req{0.05};

  // Curtailment penalties ($/MWh)
  double voll{10000.0};   ///< Value of lost load
  double vocc{1000.0};    ///< Value of over-generation curtailment

  /// Minimum renewable output enforced: P_w >= alpha * forecast
  double renewable_min_output_coeff{0.0};

  /// Add LP-valid pre-formulation cutting planes to SCUC
  bool enable_market_cuts{true};

  /// Big-M penalty for line/section flow slack variables (§2.6.3.14)
  double M1_line_slack_penalty{1.0e5};

  // Post-SCUC stages
  bool solve_sced{true};  ///< Re-solve as LP with fixed commitment
  bool solve_lmp{true};   ///< Compute nodal LMP from SCED duals

  /// Delta-neighbourhood factor for LMP re-dispatch (tex §2.6.5.5)
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
};

// ─────────────────────────────────────────────────────────────────────────────
// Full problem input
// ─────────────────────────────────────────────────────────────────────────────

struct SCUCInput {
  SCUCConfig config;
  int num_buses{1};  ///< Total number of AC buses (auto-inferred if 0)
  std::vector<Generator>   generators;
  std::vector<Branch>      branches;
  std::vector<Load>        loads;
  std::vector<WindUnit>    wind;
  std::vector<SolarUnit>   solar;
  std::vector<StorageUnit> storage;
  std::vector<DCLine>      dc_lines;
  SCUCProfiles             profiles;
  SCUCInitialStatus        initial_status;
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

  // Storage: [nstorage][T]
  Matrix2D storage_charging;
  Matrix2D storage_discharging;
  Matrix2D storage_soc;

  // Transmission: [nl][T]
  Matrix2D line_flows;

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

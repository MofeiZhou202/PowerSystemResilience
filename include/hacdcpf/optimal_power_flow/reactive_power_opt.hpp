#pragma once

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include <string>
#include <vector>

namespace hacdcpf::opf {

/// Objective function type for Reactive Power Optimization.
enum class RPOObjective {
  MinVoltageDeviation,   ///< minimise sum (V_i - 1)^2
  MinActiveLoss,         ///< minimise total active-power loss
  Combined,              ///< weighted combination of voltage deviation + loss
};

/// Inner continuous NLP objective, decoupled from the RPO objective that is
/// *evaluated* for the discrete ranking.  This separation exists because the
/// stripped physical objectives (voltage deviation / active loss) stall the
/// parity IPM on large/stiff grids, while an economic dispatch at unit
/// marginal cost (== min total generation == min loss) inherits the economic
/// path's robust machinery and converges.
enum class RPOInnerObjective {
  /// Inner IPM minimises the RPO objective directly.  Preferred on small /
  /// medium grids where it converges and lets generator reactive power help
  /// the voltage profile.
  MatchRPO,
  /// Inner IPM minimises total generation (economic dispatch at unit marginal
  /// cost).  Robust on large grids; the discrete tap/shunt search then
  /// corrects the voltage / loss profile on top of the feasible base point.
  LossEconomic,
};

/// Options for the MINLP Reactive Power Optimization solver.
struct RPOOptions {
  RPOObjective objective{RPOObjective::MinVoltageDeviation};

  /// Inner NLP objective (see RPOInnerObjective).  MinActiveLoss always uses
  /// LossEconomic (it is mathematically identical to loss minimisation).
  /// For MinVoltageDeviation / Combined the default MatchRPO is upgraded to
  /// LossEconomic automatically when the baseline fails to converge, provided
  /// robust_inner_on_failure is set.
  RPOInnerObjective inner_nlp_objective{RPOInnerObjective::MatchRPO};

  /// When the MatchRPO baseline fails on a vdev/Combined run, upgrade the
  /// inner objective to LossEconomic (robust) and re-solve instead of failing
  /// the whole RPO.  This is the mechanism that makes vdev-class RPO usable
  /// on 3000+-bus grids.
  bool robust_inner_on_failure{true};

  /// Above this AC-bus count a vdev/Combined run starts directly on the
  /// LossEconomic inner instead of attempting MatchRPO first.  Rationale:
  /// the stripped voltage-deviation objective does not converge on large /
  /// stiff grids, and *discovering* that failure is expensive (the parity IPM
  /// grinds to its iteration budget, then the Ipopt second leg grinds again,
  /// ~8 min at 3000 buses before the upgrade could trigger).  Starting on the
  /// robust inner avoids the doomed match attempt.  Below the threshold
  /// MatchRPO is still tried first (it converges and lets generator reactive
  /// power help the voltage profile), with the failure upgrade as backstop.
  /// Set to 0 to always start on MatchRPO; MinActiveLoss is unaffected.
  int robust_inner_min_buses{1000};

  /// Weight applied to the voltage-deviation term when objective is Combined.
  double vdev_weight{1.0};

  /// Weight applied to active-power loss (MW) for Combined/MinActiveLoss.
  double loss_weight{1.0};

  /// Target voltage magnitude (p.u.) for the deviation penalty.
  double v_target{1.0};

  // ---- Local discrete-search parameters ----
  /// Maximum number of continuous OPF evaluations.
  int    max_nodes{50'000};
  double time_limit_sec{120.0};
  /// Minimum objective improvement accepted by neighbourhood refinement.
  double gap_tol{1e-4};
  /// Retained for source compatibility; discrete values are represented as int.
  double int_tol{1e-5};

  // ---- Parallel discrete evaluation ----
  /// Threads used to evaluate independent discrete candidates (OLTC tap /
  /// switchable-shunt settings) in each search wave.  Each candidate is a full
  /// AC OPF solve; evaluating a batch in parallel is near-linear on the search
  /// phase.  0 = auto (hardware_concurrency, clamped to the batch size), 1 =
  /// sequential.  Forced to 1 on hybrid AC/DC grids: those drive MUMPS, which
  /// keeps mutable state in Fortran modules and is serialised by a process-wide
  /// mutex (correct but not faster), so threading only adds overhead there.
  /// Pure-AC grids use KLU (per-instance, thread-safe); the rare KLU→MUMPS
  /// escalation is serialised by the same mutex, so parallel stays safe.
  /// Note: parallel changes the search from Gauss-Seidel to Jacobi (incumbent
  /// updated between waves, not within), so the trajectory — and possibly the
  /// local optimum found — can differ slightly from the sequential path.
  int num_threads{0};

  // Retained for compatibility with older B&B-backed builds; the current local
  // search does not consume these strategy selectors.
  engine::BranchingStrategy branching{engine::BranchingStrategy::Pseudocost};
  engine::NodeSelection     node_sel{engine::NodeSelection::Hybrid};

  // ---- Inner IPM (NLP relaxation) parameters ----
  // Budget cap for each inner solve (does not force iterations).  Stiff
  // hybrid AC/DC baselines need ~2000 iterations for embedded Ipopt to reach
  // ipm_tol (measured: viol 5.2e-6 at 400 → 4.7e-8 at 2000 on
  // IEEE24-3area-expanded); easy cases finish far earlier and never consume
  // the budget.
  int    max_ipm_iter{2000};
  /// Fail-fast iteration cap for SEARCH evaluations (not the baseline).  A
  /// warm-chained perturbation that is a *good* candidate converges in a few
  /// iterations (measured 2–7 at 3000 buses); a candidate that grinds far past
  /// this cap is a bad/infeasible one, and the discrete search is better off
  /// rejecting it (obj = infeasible) than burning ~max_ipm_iter iterations on
  /// it — a single such grind (~150 s at 2000 iters) overruns the whole RPO
  /// time budget because the budget can only be checked *between* solves.
  /// Set ≤ 0 to leave search evaluations uncapped (use max_ipm_iter).
  int    search_ipm_iter{200};
  double ipm_tol{1e-6};
  /// Scaled KKT stationarity tolerance.  Kept separate from physical
  /// feasibility because large OPF objectives require a looser dual target.
  double stationarity_tol{1e-3};

  /// Backend for every inner AC OPF evaluation (baseline + sensitivity
  /// planes).  The native parity IPM is the default: it converges the stiff
  /// hybrid AC/DC cases where the embedded Ipopt exhausts its iteration
  /// budget (measured on IEEE24-3area-expanded), at a fraction of the time.
  /// Set to Ipopt for bit-exact comparison with older results.
  ACOPFSolverBackend inner_solver_backend{ACOPFSolverBackend::ParityIPM};

  /// One-time robust fallback for the economic seed on large/stiff grids
  /// (3000+ buses): when the direct parity seed does not converge, follow the
  /// economic objective homotopy to a certified endpoint (one-time cost ~1–2
  /// min at 3000 buses) instead of failing the whole RPO.
  bool seed_homotopy_on_failure{true};

  /// Limit each selected OLTC to this many positions above/below its current
  /// position.  A negative value exposes the full nameplate range; zero holds
  /// every OLTC fixed.  Library callers retain the full-range default; the GUI
  /// explicitly defaults this control to two positions in either direction.
  int max_tap_move{-1};

  /// When true, only vector positions listed in enabled_tap_indices enter the
  /// discrete decision vector.  This lets the GUI expose explicit per-device
  /// participation without mutating transformer nameplate data.
  bool restrict_tap_indices{false};
  std::vector<int> enabled_tap_indices;

  bool verbose{false};

  bool enforce_branch_limits{true};
  bool enforce_converter_capacity{true};
  bool enforce_converter_current_limits{true};
  bool enforce_converter_modulation_limits{true};
};

/// Per-transformer result entry.
struct TapResult {
  int    trafo_index{0};
  std::string name;
  int    tap_before{0};
  int    tap_after{0};
  double ratio_before{1.0};
  double ratio_after{1.0};
  double electrical_tap_before{1.0};
  double electrical_tap_after{1.0};
};

/// Per-shunt result entry.
struct ShuntResult {
  int    shunt_index{0};
  std::string name;
  int    step_before{0};
  int    step_after{0};
  double bs_mvar_before{0.0};
  double bs_mvar_after{0.0};
};

/// Auditable OLTC input row.  This is the single eligibility contract used by
/// both the solver and the GUI input preview.
struct RPOTapControlInput {
  int trafo_index{0};          ///< Position in ac.transformers_2w.
  int authored_index{0};       ///< User-facing component index.
  int source_branch_idx{0};    ///< Linked MATPOWER branch, when present.
  std::string name;
  int hv_bus{0};
  int lv_bus{0};
  int tap_side{0};             ///< 0 = HV, 1 = LV.
  bool in_service{false};
  bool adjustable{false};
  bool selected_for_optimization{false};
  std::string exclusion_reason;
  int tap_pos{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  int position_count{0};
  int optimization_tap_min{0};
  int optimization_tap_max{0};
  int optimization_position_count{0};
  double tap_step_percent{0.0};
  double ratio_current{1.0};
  double ratio_min{1.0};
  double ratio_max{1.0};
  double electrical_tap_current{1.0};
  double electrical_tap_min{1.0};
  double electrical_tap_max{1.0};
};

/// Auditable switchable-shunt input row.
struct RPOShuntControlInput {
  int shunt_index{0};          ///< Position in ac.shunts.
  int authored_index{0};
  std::string name;
  int bus{0};
  bool in_service{false};
  bool switchable{false};
  bool adjustable{false};
  std::string exclusion_reason;
  int current_step{0};
  int n_steps{0};
  int position_count{0};
  double bs_per_step_mvar{0.0};
  double bs_current_mvar{0.0};
};

struct RPOControlInventory {
  std::vector<RPOTapControlInput> taps;
  std::vector<RPOShuntControlInput> shunts;
};

struct RPOResult;

/// Inspect every candidate discrete control, including excluded devices and
/// the exact reason they do not enter the RPO decision vector.
RPOControlInventory inspect_rpo_controls(const HybridPowerSystem& sys,
                                         const RPOOptions& opt = {});

/// Apply a solved discrete RPO point back to the authored system, including
/// linked MATPOWER branch tap ratios and switchable-shunt susceptance.
void apply_rpo_discrete_solution(HybridPowerSystem& sys,
                                 const RPOResult& result);

/// Solution returned by the RPO solver.
struct RPOResult {
  bool   converged{false};
  double objective{0.0};
  double gap{1.0};
  int    nodes_explored{0};
  int    nlp_solves{0};
  double runtime_sec{0.0};
  std::string status;
  std::string algorithm{"discrete_coordinate_search_with_ac_opf"};
  bool globally_certified{false};
  bool optimality_gap_available{false};
  bool terminated_by_time_limit{false};
  bool terminated_by_evaluation_limit{false};
  std::vector<std::string> model_limitations;
  RPOInnerObjective effective_inner_nlp_objective{
      RPOInnerObjective::MatchRPO};

  // Bus-level results
  std::vector<double> vm_before;
  std::vector<double> vm_after;
  std::vector<double> va_before;
  std::vector<double> va_after;

  // Generator reactive dispatch
  std::vector<double> qg_mvar_before;
  std::vector<double> qg_mvar_after;
  std::vector<double> pg_mw_before;
  std::vector<double> pg_mw_after;

  // Discrete device results
  std::vector<TapResult>   taps;
  std::vector<ShuntResult> shunts;

  // System-level metrics
  double total_loss_before_mw{0.0};
  double total_loss_after_mw{0.0};
  double max_vdev_before{0.0};
  double max_vdev_after{0.0};

  // Legacy statistics envelope: counts local evaluations/sensitivity planes,
  // not a branch-and-bound or global-gap certificate.
  engine::BCStats bc_stats;

  /// Full inner OPF operating points retained for independent OPF/PF replay.
  ACOPFResult baseline_opf;
  ACOPFResult optimized_opf;
};

/// Solve the Reactive Power Optimization problem (MINLP).
///
/// Decision variables:
///   - Continuous: bus voltage magnitudes V_m, generator reactive output Q_g
///     (embedded inside the parity OPF formulation).
///   - Integer: transformer on-load tap-changer position  t_k ∈ [tap_min, tap_max],
///              switchable shunt step  s_j ∈ [0, n_steps].
///
/// The problem is solved by sensitivity-ranked discrete coordinate search and
/// pairwise neighbourhood refinement.  Each evaluated discrete setting is
/// completed by a nonlinear AC/DC OPF.  The method returns a feasible incumbent
/// but does not claim a globally valid MINLP optimality certificate.
RPOResult solve_rpo(const HybridPowerSystem& sys, const RPOOptions& opt = {});

}  // namespace hacdcpf::opf

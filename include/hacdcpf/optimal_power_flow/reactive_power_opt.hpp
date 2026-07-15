#pragma once

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/engine/branch_and_cut.hpp"
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

/// Options for the MINLP Reactive Power Optimization solver.
struct RPOOptions {
  RPOObjective objective{RPOObjective::MinVoltageDeviation};

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

  // Retained for compatibility with older B&B-backed builds; the current local
  // search does not consume these strategy selectors.
  engine::BranchingStrategy branching{engine::BranchingStrategy::Pseudocost};
  engine::NodeSelection     node_sel{engine::NodeSelection::Hybrid};

  // ---- Inner IPM (NLP relaxation) parameters ----
  int    max_ipm_iter{400};
  double ipm_tol{1e-6};

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

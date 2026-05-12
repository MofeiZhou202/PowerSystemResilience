#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/components.hpp"

namespace hacdcpf {

struct SolverProfiling {
  std::string linear_solver_backend;
  int ac_eval_threads{1};
  int jacobian_pattern_rebuilds{0};
  int jacobian_analyze_calls{0};
  int factorization_calls{0};
  int linear_solve_calls{0};
  int regularization_attempts{0};
  int line_search_evaluations{0};
  int rejected_steps{0};
  int pv_to_pq_switches{0};
  int pq_to_pv_switches{0};
  int converter_mode_switches{0};
  std::vector<double> residual_by_iter;
  std::vector<int> line_search_evals_by_iter;
  std::vector<double> eval_jacobian_ms_by_iter;
  std::vector<double> linear_solve_ms_by_iter;
  std::vector<double> line_search_ms_by_iter;
  double eval_jacobian_ms_total{0.0};
  double linear_solve_ms_total{0.0};
  double line_search_ms_total{0.0};

  // Phase 1 robust diagnostics (used by newton_solver).
  double raw_residual_norm{0.0};
  double scaled_residual_norm{0.0};
  double condition_estimate{0.0};
  int regularization_count{0};
  std::string linear_solver_status{"not_run"};
};

struct BranchFlow {
  double pf_mw{0.0};    // from-end active power
  double qf_mvar{0.0};  // from-end reactive power
  double pt_mw{0.0};    // to-end active power
  double qt_mvar{0.0};  // to-end reactive power
};

struct VSCTransfer {
  int index{0};
  int bus_ac{0};
  int bus_dc{0};
  double p_ac_mw{0.0};
  double q_ac_mvar{0.0};
  double p_dc_mw{0.0};
  double loss_mw{0.0};
};

struct DCDCTransfer {
  int index{0};
  int bus_in{0};
  int bus_out{0};
  double p_in_mw{0.0};
  double p_out_mw{0.0};
  double loss_mw{0.0};
};

struct Trafo3WFlow {
  int index{0};
  int hv_bus{0}, mv_bus{0}, lv_bus{0};
  double p_hv_mw{0.0}, q_hv_mvar{0.0};
  double p_mv_mw{0.0}, q_mv_mvar{0.0};
  double p_lv_mw{0.0}, q_lv_mvar{0.0};
  double loss_mw{0.0};
  double loading_pct{0.0};
  double rate_mva{0.0};
};

struct ERPortTransfer {
  int router_index{0};
  int port_index{0};
  int bus{0};
  bool is_ac{true};
  double p_mw{0.0};
  double q_mvar{0.0};
  double v_pu{1.0};
};

struct PowerFlowResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  SolverProfiling profiling;
  std::vector<BranchFlow> branch_flows;
  std::vector<VSCTransfer> vsc_transfers;
  std::vector<DCDCTransfer> dcdc_transfers;
  std::vector<Trafo3WFlow> trafo3w_flows;
  std::vector<ERPortTransfer> er_port_transfers;
};

struct DCPowerFlowResult {
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
};

struct AdaptiveSolveResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::vector<IslandInfo> islands;
  std::vector<BranchFlow> branch_flows;
};

struct IslandedSolveResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::vector<IslandInfo> islands;
};

struct DistributedSlackResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::unordered_map<int, double> distributed_slack_p;
  std::vector<int> hit_limits;
};

}  // namespace hacdcpf

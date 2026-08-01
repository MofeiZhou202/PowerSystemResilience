#pragma once
/// \file stats.hpp
/// \brief Runtime statistics and result record for the B&C engine.

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Detailed statistics accumulated during the B&C run.
struct BCStats {
  /// Provenance and availability for fields whose collectors are specific to
  /// the native tree. A zero counter means an observed zero only when the
  /// corresponding availability flag is true.
  std::string collection_scope{"native_full"};
  bool lp_solve_count_available{true};
  bool incumbent_timeline_available{true};
  bool cut_diagnostics_available{true};
  bool native_diagnostics_available{true};

  // ── Basic progress ─────────────────────────────────────────────────────
  int    nodes_explored{0};
  int    lp_solves{0};
  int    cuts_added{0};
  int    root_cuts_added{0};
  int    tree_cuts_added{0};
  int    root_gomory_cuts{0};
  int    root_mir_cuts{0};
  int    root_cover_cuts{0};
  int    root_clique_cuts{0};
  int    root_zerohalf_cuts{0};
  int    root_impliedbound_cuts{0};
  int    root_presolved_rows{0};
  int    root_presolved_cols{0};
  int    root_presolved_compact_rows{0};
  int    root_two_sided_rows_expanded{0};
  int    root_presolved_binary_cols{0};
  int    root_presolved_integer_cols{0};
  int    root_implied_integer_cols{0};
  int    root_presolved_continuous_cols{0};
  std::uint64_t root_clique_edges{0};
  std::uint64_t root_implication_arcs{0};
  std::uint64_t root_cut_rows_rejected_nonmoving{0};
  std::uint64_t root_cut_rounds_rejected_nonmoving{0};
  std::uint64_t root_cut_domain_tightenings{0};
  std::uint64_t root_cut_standard_form_mismatches{0};
  std::uint64_t root_cut_standard_form_flipped_rows{0};
  double root_cut_bound_lift{0.0};
  std::uint64_t root_reduced_cost_nonzero{0};
  std::uint64_t root_reduced_cost_integer_nonzero{0};
  std::uint64_t root_reduced_cost_implied_integer_nonzero{0};
  std::uint64_t root_reduced_cost_at_lower{0};
  std::uint64_t root_reduced_cost_at_upper{0};
  std::uint64_t root_reduced_cost_cutoff_fix_candidates{0};
  double root_reduced_cost_abs_sum{0.0};
  double root_reduced_cost_abs_max{0.0};
  double root_reduced_cost_gap{0.0};
  std::uint64_t root_objective_cutoff_domain_tightenings{0};
  std::uint64_t root_objective_cutoff_domain_prunes{0};
  std::uint64_t vendored_root_certificate_attempts{0};
  std::uint64_t vendored_root_certificate_accepted{0};
  std::uint64_t vendored_root_certificate_rejected{0};
  double vendored_root_certificate_time_ms{0.0};
  std::int64_t vendored_root_certificate_nodes{-1};
  std::int64_t vendored_root_certificate_simplex_iterations{-1};
  std::uint64_t root_split_bound_probe_candidates{0};
  std::uint64_t root_split_bound_probe_vars{0};
  std::uint64_t root_split_bound_probe_fixings{0};
  std::uint64_t root_split_bound_probe_cap_applied{0};
  std::uint64_t root_split_bound_probe_cap_from{0};
  std::uint64_t root_split_bound_probe_cap_to{0};
  double root_split_bound_probe_time_ms{0.0};
  double root_split_bound_probe_lift{0.0};
  double best_bound{-1e30};
  double best_obj{1e30};
  double gap{1e30};
  double root_only_bound{std::numeric_limits<double>::quiet_NaN()};
  double root_only_solver_bound{std::numeric_limits<double>::quiet_NaN()};
  double root_only_objective_offset{0.0};
  bool root_only_result{false};
  double runtime_sec{0.0};
  /// Number of unpresolved-root fallback retries taken after a root-LP
  /// failure on the PaPILO-presolved model (0 or 1; see
  /// BCOptions::root_presolve_fallback).
  int presolve_fallback_attempts{0};
  std::uint64_t incumbent_updates{0};
  int first_incumbent_node{-1};
  int last_incumbent_node{-1};
  int last_incumbent_depth{-1};
  int first_incumbent_lp_solves{-1};
  int last_incumbent_lp_solves{-1};
  int nodes_after_last_incumbent{0};
  int lp_solves_after_last_incumbent{0};
  double best_bound_at_last_incumbent{-1e30};
  double best_bound_lift_after_last_incumbent{0.0};
  std::uint64_t incumbent_queue_prune_passes{0};
  std::uint64_t incumbent_queue_prunes{0};
  std::uint64_t gap_suboptimal_queue_prune_passes{0};
  std::uint64_t gap_suboptimal_queue_prunes{0};
  std::uint64_t incumbent_certificate_queue_consumptions{0};
  std::uint64_t incumbent_repair_lp_attempts{0};
  std::uint64_t incumbent_repair_improvements{0};
  double incumbent_repair_time_ms{0.0};
  std::uint64_t incumbent_local_branching_attempts{0};
  std::uint64_t incumbent_local_branching_improvements{0};
  double incumbent_local_branching_time_ms{0.0};
  std::uint64_t reliability_branch_nodes{0};
  std::uint64_t strong_branch_candidates{0};
  std::uint64_t strong_branch_lp_solves{0};
  std::uint64_t strong_complete_probe_pairs{0};
  std::uint64_t strong_selected_exact{0};
  std::uint64_t strong_selected_unprobed{0};
  std::uint64_t strong_winner_changed_by_exact{0};
  std::uint64_t strong_probe_unknown_failures{0};
  std::uint64_t strong_probe_lp_iterations{0};
  double strong_probe_time_ms{0.0};
  std::uint64_t branch_direction_preferred_down{0};
  std::uint64_t branch_direction_preferred_up{0};
  std::uint64_t branch_direction_first_down{0};
  std::uint64_t branch_direction_first_up{0};
  std::uint64_t branch_first_child_incumbent_updates{0};
  std::uint64_t branch_first_child_cutoffs{0};
  std::uint64_t branch_second_child_cutoffs{0};
  std::uint64_t node_estimate_calibration_samples{0};
  double node_estimate_predicted_lift_sum{0.0};
  double node_estimate_realized_lift_sum{0.0};
  double node_estimate_abs_error_sum{0.0};
  double node_estimate_squared_error_sum{0.0};
  double node_estimate_predicted_sq_sum{0.0};
  double node_estimate_realized_sq_sum{0.0};
  double node_estimate_cross_sum{0.0};
  std::uint64_t directional_calibration_samples{0};
  double directional_predicted_gain_sum{0.0};
  double directional_realized_gain_sum{0.0};
  double directional_abs_error_sum{0.0};
  double directional_squared_error_sum{0.0};
  std::uint64_t directional_rank_samples{0};
  std::uint64_t directional_rank_concordant{0};
  std::uint64_t strong_branch_cache_exact_hits{0};
  std::uint64_t strong_branch_cache_warm_hits{0};
  std::uint64_t strong_branch_duplicate_lp_avoided{0};
  std::uint64_t branching_regret_samples{0};
  double branching_regret_sum{0.0};
  double branching_regret_max{0.0};
  std::uint64_t strong_branch_regret_samples{0};
  double strong_branch_regret_sum{0.0};
  double strong_branch_regret_max{0.0};
  int parallel_requested_threads{1};
  int parallel_effective_threads{1};
  int parallel_explorer_threads{0};
  bool parallel_tree_launched{false};
  std::string parallel_schedule_reason{"single_thread"};

  // ── Cut / heuristic diagnostics ────────────────────────────────────────
  int pool_cuts_separated{0};    ///< Cuts added from the global cut pool
  int crossover_incumbents{0};   ///< Incumbents found by crossover heuristic
  int cut_requests_submitted{0}; ///< Cut requests sent to the cut worker
  int cut_requests_dropped{0};   ///< Cut requests dropped (queue full)
  int cut_worker_cuts{0};        ///< Cuts generated by the cut worker
  int cut_worker_incumbents{0};  ///< Incumbents found by the cut worker

  // ── LP fallback recovery ───────────────────────────────────────────────
  int fallback_events{0};
  int fallback_recoveries{0};
  int fallback_deferrals{0};
  int fallback_prunes{0};
  int fallback_infeasible_prunes{0};
  int fallback_unbounded_prunes{0};
  int fallback_unsafe_prunes{0};
  int fallback_l1_success{0};
  int fallback_l2_success{0};
  int fallback_l3_success{0};

  // ── CGLP disjunctive cut statistics ──────────────────────────────────────
  int cglp_candidates{0};    ///< Binary vars evaluated as CGLP candidates (all rounds)
  int cglp_lp_solved{0};     ///< CGLP master LPs solved without falling back to interpolation
  int cglp_fallback_used{0}; ///< Calls to the interpolation-fallback separator
  int cglp_accepted{0};      ///< Cuts passing all filters and admitted to the LP
  int cglp_dedup_rejected{0};///< Candidate cuts dropped by the per-pass hash dedup

  // ── Work stealing ──────────────────────────────────────────────────────
  int work_steals{0};
  int steal_attempts{0};

  // ── Event-driven propagation profile ───────────────────────────────────
  std::uint64_t propagation_calls{0};
  std::uint64_t propagation_active_passes{0};
  std::uint64_t propagation_changed_var_visits{0};
  std::uint64_t propagation_ineq_rows_visited{0};
  std::uint64_t propagation_eq_rows_visited{0};
  std::uint64_t propagation_baseline_full_scan_rows{0};
  std::uint64_t propagation_bound_tightenings{0};
  std::uint64_t propagation_wall_ns{0};

  // ── Dynamic conflict / implied-bound learning diagnostics ─────────────
  std::uint64_t rc_fixings{0};
  std::uint64_t rc_forbidden_literals{0};
  std::uint64_t rc_conflict_clauses_learned{0};
  std::uint64_t rc_binary_implications_learned{0};
  std::uint64_t rc_conflict_cuts_added{0};
  std::uint64_t rc_conflict_queue_prunes{0};
  std::uint64_t rc_proof_conflict_clauses_learned{0};
  std::uint64_t rc_proof_conflict_literals_before{0};
  std::uint64_t rc_proof_conflict_literals_after{0};
  std::uint64_t rc_target_proof_candidates{0};
  std::uint64_t rc_target_proof_attempts{0};
  std::uint64_t rc_target_proof_successes{0};
  std::uint64_t rc_target_proof_clauses_learned{0};
  std::uint64_t rc_target_proof_diagnostic_samples{0};
  double rc_target_proof_budget_sum{0.0};
  double rc_target_proof_budget_max{0.0};
  double rc_target_frontier_lp_activity_sum{0.0};
  double rc_target_frontier_lp_activity_max{0.0};
  double rc_target_cover_violation_sum{0.0};
  double rc_target_cover_violation_max{0.0};
  std::uint64_t rc_proof_resolve_lift_samples{0};
  double rc_proof_resolve_lift_sum{0.0};
  double rc_proof_resolve_lift_max{0.0};
  std::uint64_t rc_conflict_verification_lps{0};
  std::uint64_t rc_verified_conflict_clauses_learned{0};
  std::uint64_t rc_fixing_resolve_lps{0};
  std::uint64_t rc_fixing_resolve_bound_improvements{0};
  std::uint64_t rc_fixing_resolve_prunes{0};
  std::uint64_t dynamic_implied_bound_rows_generated{0};
  std::uint64_t dynamic_implied_bound_rows_violated{0};
  std::uint64_t dynamic_implied_bound_rows_added{0};
  std::uint64_t dynamic_implied_bound_rows_bound_move{0};
  std::uint64_t dynamic_implied_bound_rows_rejected{0};
  std::uint64_t dynamic_implied_bound_rows_active_implication{0};
  std::uint64_t dynamic_implied_bound_rows_queue_hit{0};
  std::uint64_t dynamic_implied_bound_rows_skip_not_active{0};
  std::uint64_t dynamic_implied_bound_rows_skip_small_move{0};
  std::uint64_t dynamic_implied_bound_rows_skip_no_violation{0};
  std::uint64_t local_node_cuts_generated{0};
  std::uint64_t local_node_cuts_added{0};
  std::uint64_t global_cutpool_scope_rejections{0};
  std::uint64_t lazy_constraints_added{0};
  std::uint64_t late_replay_skipped_no_new_learning{0};
  std::uint64_t objective_clique_partitions{0};
  std::uint64_t objective_clique_terms{0};
  std::uint64_t objective_clique_edges{0};
  std::uint64_t objective_clique_mixed_rows{0};
  double objective_clique_max_replacement_delta{0.0};
  std::uint64_t objective_implied_events{0};
  std::uint64_t objective_implied_event_tightenings{0};
  std::uint64_t objective_implied_event_prunes{0};
  std::uint64_t objective_implied_event_lp_slack_hits{0};
  std::uint64_t objective_implied_event_proof_candidates{0};
  std::uint64_t objective_implied_event_proof_fixings{0};
  std::uint64_t objective_implied_event_proof_skip_coeff{0};
  std::uint64_t objective_implied_event_proof_skip_budget{0};
  std::uint64_t objective_implied_event_vlb_events{0};
  std::uint64_t objective_implied_event_vub_events{0};
  std::uint64_t variable_bound_table_size{0};
  std::uint64_t variable_bound_vub_attempts{0};
  std::uint64_t variable_bound_vub_accepted{0};
  std::uint64_t variable_bound_vub_replaced{0};
  std::uint64_t variable_bound_vlb_attempts{0};
  std::uint64_t variable_bound_vlb_accepted{0};
  std::uint64_t variable_bound_vlb_replaced{0};
  std::uint64_t variable_bound_mir_attempts{0};
  std::uint64_t variable_bound_mir_strengthened{0};
  std::uint64_t variable_bound_exported_implications{0};
  std::uint64_t variable_bound_cut_rows_scanned{0};
  std::uint64_t variable_bound_cut_mixed_rows{0};
  std::uint64_t variable_bound_cut_vub_candidates{0};
  std::uint64_t variable_bound_cut_vlb_candidates{0};
  std::uint64_t variable_bound_cut_exported_implications{0};
  std::uint64_t objective_implied_event_published_implications{0};
  std::uint64_t objective_implied_event_published_clique_edges{0};
  std::uint64_t objective_implied_event_integral_rounds{0};
  std::uint64_t objective_implied_event_mir_attempts{0};
  std::uint64_t objective_implied_event_mir_strengthened{0};
  double objective_implied_event_mir_gain_sum{0.0};
  double objective_implied_event_mir_gain_max{0.0};
  double objective_implied_event_max_delta{0.0};
  double objective_implied_event_max_aggregate_delta{0.0};
  std::uint64_t objective_domain_bound_lift_nodes{0};
  double objective_domain_bound_lift_sum{0.0};
  double objective_domain_bound_lift_max{0.0};
  std::uint64_t termination_effective_candidates{0};
  std::uint64_t termination_effective_admitted{0};
  std::uint64_t termination_effective_skip_no_lp_or_queue_hit{0};
  std::uint64_t transformed_frontier_source_attempts{0};
  std::uint64_t transformed_frontier_tableau_rows{0};
  std::uint64_t transformed_frontier_path_rows{0};
  std::uint64_t transformed_frontier_violated_rows{0};
  std::uint64_t transformed_frontier_admitted_rows{0};
  std::uint64_t transformed_frontier_rejected_rows{0};
  std::uint64_t transformed_frontier_fractional_coeff_hits{0};
  double transformed_frontier_max_fractional_coeff{0.0};
  double transformed_frontier_max_violation{0.0};
  std::uint64_t transformed_frontier_resolve_samples{0};
  std::uint64_t transformed_frontier_resolve_bound_moves{0};
  double transformed_frontier_resolve_lift_sum{0.0};
  double transformed_frontier_resolve_lift_max{0.0};
  std::uint64_t certificate_queue_passes{0};
  std::uint64_t certificate_queue_tightenings{0};
  std::uint64_t certificate_queue_prunes{0};
  double certificate_queue_lift_sum{0.0};
  double certificate_queue_lift_max{0.0};
  std::uint64_t certificate_queue_lower_bound_lift_nodes{0};
  std::uint64_t certificate_queue_lp_refresh_marked{0};
  std::uint64_t certificate_queue_frontier_bound_moves{0};
  double certificate_queue_frontier_bound_move_sum{0.0};
  double certificate_queue_frontier_bound_move_max{0.0};
  std::uint64_t certificate_queue_lb_audit_passes{0};
  std::uint64_t certificate_queue_lb_audit_failures{0};
  double certificate_queue_lb_audit_indexed_min{
      std::numeric_limits<double>::infinity()};
  double certificate_queue_lb_audit_computed_min{
      std::numeric_limits<double>::infinity()};
  double certificate_queue_lb_audit_max_error{0.0};
  std::int64_t certificate_queue_lb_audit_indexed_nodes{0};
  std::int64_t certificate_queue_lb_audit_live_nodes{0};
  std::uint64_t bound_lifting_certificates_published{0};
  std::uint64_t bound_lifting_certificates_rejected_audit{0};
  double bound_lifting_certificate_activity_margin_min{
      std::numeric_limits<double>::infinity()};
  double bound_lifting_certificate_activity_margin_max{0.0};
  std::uint64_t resolved_target_attempts{0};
  std::uint64_t resolved_target_success{0};
  std::uint64_t resolved_target_failed{0};
  std::uint64_t resolved_conflict_attempts{0};
  std::uint64_t resolved_conflict_success{0};
  std::uint64_t resolved_conflict_failed{0};
  std::uint64_t resolution_activity_failed{0};
  std::uint64_t resolution_missing_reason{0};
  std::uint64_t resolution_scope_blocked{0};
  std::uint64_t certificate_domain_passes{0};
  std::uint64_t certificate_domain_fixings{0};
  std::uint64_t certificate_domain_prunes{0};
  std::uint64_t certificate_domain_resolve_lps{0};
  std::uint64_t certificate_domain_bound_improvements{0};
  std::uint64_t invalid_cut_coefficients{0};

  // ── Source/propagation conformance instrumentation ───────────────────
  std::uint64_t conformance_source_phases{0};
  std::uint64_t conformance_source_nonempty_phases{0};
  std::uint64_t conformance_propagation_phases{0};
  std::uint64_t conformance_bound_change_phases{0};
  std::uint64_t conformance_bound_changes{0};
  std::uint64_t conformance_domain_prunes{0};
  std::uint64_t conformance_resolve_attempts{0};
  std::uint64_t conformance_resolve_failures{0};
  std::uint64_t conformance_resolve_bound_moves{0};
  double conformance_bound_lift_sum{0.0};
  double conformance_bound_lift_max{0.0};
  std::uint64_t conformance_queue_phases{0};
  std::uint64_t conformance_queue_tightenings{0};
  std::uint64_t conformance_queue_prunes{0};
  std::uint64_t conformance_queue_bound_moves{0};
  double conformance_queue_lift_sum{0.0};
  double conformance_queue_lift_max{0.0};
  std::uint64_t conformance_objective_capacity_phases{0};
  std::uint64_t conformance_objective_capacity_active_events{0};
  std::uint64_t conformance_objective_capacity_active_tight_events{0};
  std::uint64_t conformance_objective_capacity_one_missing_events{0};
  std::uint64_t conformance_objective_capacity_one_missing_keys{0};
  std::uint64_t conformance_objective_capacity_raw_exceed_keys{0};
  std::uint64_t conformance_objective_capacity_proof_exceed_keys{0};
  double conformance_objective_capacity_max_one_missing{0.0};
  double conformance_objective_capacity_max_proof_excess{0.0};

  std::uint64_t local_implications_learned{0};
  std::uint64_t local_implications_applied{0};
  std::uint64_t local_implication_prunes{0};
  std::uint64_t local_conflict_clauses_learned{0};
  std::uint64_t local_conflict_clause_tightenings{0};
  std::uint64_t local_conflict_clause_prunes{0};
  std::uint64_t scoped_conflict_objects_learned{0};
  std::uint64_t scoped_conflict_objects_active{0};
  std::uint64_t scoped_conflict_tightenings{0};
  std::uint64_t scoped_conflict_prunes{0};
  std::uint64_t cutoff_domain_tightenings{0};
  std::uint64_t cutoff_domain_prunes{0};
  std::uint64_t cutoff_conflict_clauses_learned{0};
  std::uint64_t cutoff_conflict_cuts_added{0};

  // ── FT drift telemetry (benchmark/audit only) ────────────────────────
  int ft_audit_samples{0};
  int ft_branch_choice_changes{0};
  int ft_live_ft_samples{0};
  std::uint64_t ft_audit_updates_sum{0};
  std::uint64_t ft_audit_updates_changed_sum{0};
  double ft_audit_growth_sum{0.0};
  double ft_audit_growth_changed_sum{0.0};
  double ft_audit_fill_sum{0.0};
  double ft_audit_fill_changed_sum{0.0};

  // ── IPM incremental LP telemetry ────────────────────────────────────────
  std::int64_t ipm_total_iterations{0};      ///< Sum of IPM iterations across all nodes
  int ipm_nodes_solved{0};                   ///< Number of nodes solved with IPM
  int ipm_iteration_min{INT_MAX};            ///< Min IPM iterations per node
  int ipm_iteration_max{0};                  ///< Max IPM iterations per node

  std::string status;
};

/// Extended result returned by the native branch-and-cut solvers.
struct BCResult {
  Eigen::VectorXd x;
  SolveStats      stats;     ///< Standard SolveStats (compatible with SolverAdapter)
  BCStats         bc_stats;  ///< Additional B&C diagnostics
  /// Non-empty solver-owned environment settings captured at solve entry.
  /// Nested solves and worker threads use this same immutable snapshot.
  std::vector<std::string> effective_environment;
  /// Root cuts extracted from this solve (HiGHS StrictHiGHS path only).
  /// Pass as BCOptions::highs_root_cut_warm_start on the next solve of the
  /// same problem to skip the ~40 s root cutting loop.
  std::shared_ptr<BCRootCuts> highs_root_cuts;
  /// Root simplex basis extracted from the same solve, in original row/column
  /// space.  Pass with highs_root_cuts to warm-start the augmented root LP.
  std::shared_ptr<BCRootBasis> highs_root_basis;
  /// Pseudocost data extracted from this solve's HiGHS or native B&C tree, in
  /// original column space.  Pass as BCOptions::highs_pseudocost_warm_start on
  /// the next solve of the same problem to seed branching decisions from node
  /// 1.  The field retains its historical name for API compatibility.
  std::shared_ptr<BCPseudocostInit> highs_pseudocost_init;
};

}  // namespace mipsolvers::engine

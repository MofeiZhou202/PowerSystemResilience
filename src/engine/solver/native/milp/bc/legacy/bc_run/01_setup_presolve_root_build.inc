// @file bc_run/01_setup_presolve_root_build.inc
// BCSolveState::run() segment 1/14: argument/env setup, integrality normalization, strict-HiGHS entry path, presolve (PaPILO + native MILPPresolve), and root model build (Node root).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
  const MIPModel& prob = prob_;
  BCOptions& opt = opt_;
  const BCCallbacks* callbacks = callbacks_;
  auto solve_environment = current_bc_env_options();
  if (!solve_environment) solve_environment = capture_bc_env_options();
  ScopedBcEnvOptions environment_scope(solve_environment);
  BCResult out;
  SeparatorStorageStats separator_storage_stats;
  enum class QueueStorageSource { Serial, Parallel, SubMip };
  auto accumulate_queue_storage =
      [&](const NodeQueueDomainStorageStats& storage,
          QueueStorageSource source) {
        out.bc_stats.node_queue_domain_compactions += storage.compactions;
        out.bc_stats.node_queue_domain_materializations +=
            storage.materializations;
        out.bc_stats.node_queue_dense_bound_values_released +=
            storage.dense_bound_values_released;
        out.bc_stats.node_queue_compact_entries_created +=
            storage.compact_entries_created;
        out.bc_stats.node_queue_terminal_compact_nodes +=
            storage.current_compact_nodes;
        out.bc_stats.node_queue_terminal_compact_entries +=
            storage.current_compact_entries;
        out.bc_stats.node_queue_peak_compact_nodes = std::max(
            out.bc_stats.node_queue_peak_compact_nodes,
            storage.peak_compact_nodes);
        out.bc_stats.node_queue_peak_compact_entries = std::max(
            out.bc_stats.node_queue_peak_compact_entries,
            storage.peak_compact_entries);
        out.bc_stats.node_queue_domain_compaction_failures +=
            storage.compaction_failures;
        out.bc_stats.node_queue_domain_materialization_failures +=
            storage.materialization_failures;
        switch (source) {
          case QueueStorageSource::Serial:
            out.bc_stats.node_queue_serial_compactions += storage.compactions;
            break;
          case QueueStorageSource::Parallel:
            out.bc_stats.node_queue_parallel_compactions += storage.compactions;
            break;
          case QueueStorageSource::SubMip:
            out.bc_stats.node_queue_submip_compactions += storage.compactions;
            break;
        }
      };
  auto accumulate_payload_sharing =
      [&](const NodePayloadSharingStats& sharing) {
        out.bc_stats.branch_payload_child_creations +=
            sharing.child_creations;
        out.bc_stats.branch_payload_shared_vectors +=
            sharing.shared_payload_vectors;
        out.bc_stats.branch_payload_shared_elements +=
            sharing.shared_payload_elements;
      };
  out.effective_environment = solve_environment->active_settings();
  out.stats.solver_name = "NativeBranchAndCut";
  out.stats.status = bc_status::kNotStarted;
  const auto highs_contract = apply_bc_strict_highs_contract(opt);
  const bool requested_vendored_highs_lp =
      highs_contract.requested_vendored_highs_lp;
  const bool strict_highs_lp_contract =
      highs_contract.strict_highs_lp_contract;
  const bool strict_highs_root_fixed_point =
      strict_highs_lp_contract && opt.auto_highs_root_pipeline;
  const double highs_mip_feastol = strict_highs_lp_contract
      ? kHighsDefaultMipFeasibilityTolerance
      : opt.int_tol;
  const bool allow_vendored_root_frontier =
      highs_contract.allow_vendored_root_frontier;
  bool domain_heuristics = highs_contract.domain_heuristics;

  // Forrest--Tomlin activation.
  // When use_forrest_tomlin_updates is true, force the simplex factor
  // backend to BackendB_HiGHSSafe (id=1) in all downstream LP dispatches.
  // The compile-time flag MIPSOLVERS_ENABLE_FACTOR_BACKEND_B gates the
  // actual FT codepath; when that flag is OFF, the backend silently
  // falls back to Backend A inside prepare_factor_backend_after_numeric.
  if (opt.use_forrest_tomlin_updates && opt.simplex_factor_backend < 1) {
    opt.simplex_factor_backend = 1;
  }

  static thread_local int propagation_profile_scope_depth = 0;
  const bool reset_propagation_profile_now = (propagation_profile_scope_depth++ == 0);
  if (reset_propagation_profile_now) {
    reset_propagation_profile();
  }

  auto sync_propagation_profile = [&]() {
    const auto profile = get_propagation_profile();
    out.bc_stats.propagation_calls = profile.calls;
    out.bc_stats.propagation_active_passes = profile.active_passes;
    out.bc_stats.propagation_changed_var_visits = profile.changed_var_visits;
    out.bc_stats.propagation_ineq_rows_visited = profile.ineq_rows_visited;
    out.bc_stats.propagation_eq_rows_visited = profile.eq_rows_visited;
    out.bc_stats.propagation_baseline_full_scan_rows = profile.baseline_full_scan_rows;
    out.bc_stats.propagation_bound_tightenings = profile.bound_tightenings;
    out.bc_stats.propagation_wall_ns = profile.wall_ns;
  };

  // Failure paths can leave BCStats at its defaults (best_bound -1e30,
  // best_obj/gap 1e30).  Those sentinels are finite, so downstream
  // isfinite() guards ingest them as real values and report nonsense gaps;
  // publish honest infinities instead.
  auto finalize_reported_stats_hygiene = [&]() {
    const double inf = std::numeric_limits<double>::infinity();
    if (std::abs(out.bc_stats.best_bound) >= 1e29) {
      out.bc_stats.best_bound =
          (prob.linear_part.sense == Sense::Minimize) ? -inf : inf;
    }
    if (out.bc_stats.best_obj >= 1e29) {
      out.bc_stats.best_obj = inf;
    }
    if (out.bc_stats.gap >= 1e29) {
      out.bc_stats.gap = inf;
    }
  };

  auto return_with_profile = [&]() {
    // A successful solve may terminate before the native tree is entered. In
    // that case imported pseudocosts are still the current learned state and
    // must not disappear from the round-trip API.
    if (out.stats.success && !out.highs_pseudocost_init &&
        opt.highs_pseudocost_warm_start &&
        opt.highs_pseudocost_warm_start->n_orig_cols ==
            prob.linear_part.c.size()) {
      out.highs_pseudocost_init = std::make_shared<BCPseudocostInit>(
          *opt.highs_pseudocost_warm_start);
    }
    finalize_reported_stats_hygiene();
    out.bc_stats.separator_sparse_candidates_created +=
        separator_storage_stats.sparse_candidates_created;
    out.bc_stats.separator_sparse_candidate_entries_created +=
        separator_storage_stats.sparse_candidate_entries_created;
    out.bc_stats.separator_peak_live_sparse_candidates = std::max(
        out.bc_stats.separator_peak_live_sparse_candidates,
        separator_storage_stats.peak_live_sparse_candidates);
    out.bc_stats.separator_peak_live_sparse_entries = std::max(
        out.bc_stats.separator_peak_live_sparse_entries,
        separator_storage_stats.peak_live_sparse_entries);
    out.bc_stats.separator_sparse_aggregation_snapshots +=
        separator_storage_stats.sparse_aggregation_snapshots;
    out.bc_stats.separator_sparse_aggregation_entries +=
        separator_storage_stats.sparse_aggregation_entries;
    out.bc_stats.separator_dense_workspace_materializations +=
        separator_storage_stats.dense_workspace_materializations;
    out.bc_stats.separator_dense_workspace_values +=
        separator_storage_stats.dense_workspace_values;
    out.bc_stats.separator_matrix_append_calls +=
        separator_storage_stats.matrix_append_calls;
    out.bc_stats.separator_matrix_appended_rows +=
        separator_storage_stats.matrix_appended_rows;
    out.bc_stats.separator_matrix_appended_entries +=
        separator_storage_stats.matrix_appended_entries;
    out.bc_stats.separator_matrix_prior_entries_bypassing_triplet_rebuild +=
        separator_storage_stats
            .matrix_prior_entries_bypassing_triplet_rebuild;
    out.bc_stats.separator_matrix_storage_reallocations +=
        separator_storage_stats.matrix_storage_reallocations;
    out.bc_stats.separator_matrix_peak_spare_entries = std::max(
        out.bc_stats.separator_matrix_peak_spare_entries,
        separator_storage_stats.matrix_peak_spare_entries);
    sync_propagation_profile();
    --propagation_profile_scope_depth;
    return out;
  };

  const auto t0 = std::chrono::steady_clock::now();
  auto bc_elapsed_sec = [&]() -> double {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
  };
  auto bc_remaining_sec = [&]() -> double {
    if (!(opt.time_limit_sec > 0.0) || !std::isfinite(opt.time_limit_sec)) {
      return std::numeric_limits<double>::infinity();
    }
    return opt.time_limit_sec - bc_elapsed_sec();
  };
  auto bc_time_limit_expired = [&]() -> bool {
    return opt.time_limit_sec > 0.0 && bc_elapsed_sec() >= opt.time_limit_sec;
  };
  // Root queue construction and proof bookkeeping have a measurable fixed
  // cost on large models. Stop optional root work slightly early so a short
  // solve limit includes that finalization instead of overrunning after the
  // last heuristic LP finishes.
  const double bc_post_root_finalize_reserve_sec =
      opt.time_limit_sec > 0.0 && std::isfinite(opt.time_limit_sec)
          ? std::min(1.0, std::max(0.05, 0.10 * opt.time_limit_sec))
          : 0.0;
  auto bc_optional_root_budget_expired = [&]() -> bool {
    return opt.time_limit_sec > 0.0 &&
           bc_remaining_sec() <= bc_post_root_finalize_reserve_sec;
  };

  // Reject pathologically poor candidate incumbents. A candidate whose
  // (minimize-convention) objective exceeds the reference dual bound by more
  // than incumbent_quality_reject_factor * max(1, |bound|) is a repair-path
  // artefact, not a usable solution.  Accepts when the gate is disabled or
  // no finite bound is available.
  auto incumbent_quality_acceptable = [&](double candidate_obj,
                                          double reference_bound) -> bool {
    const double factor = opt.incumbent_quality_reject_factor;
    if (!(factor > 0.0) || !std::isfinite(factor) ||
        !std::isfinite(reference_bound)) {
      return true;
    }
    return candidate_obj - reference_bound <=
           factor * std::max(1.0, std::abs(reference_bound));
  };

  if (prob.linear_part.vars.empty() || prob.linear_part.c.size() == 0) {
    out.stats.status = bc_status::kEmptyMilpModel;
    return return_with_profile();
  }

  LPModel base_lp = prob.linear_part;
  const int orig_n = static_cast<int>(base_lp.vars.size());

  std::vector<char> integer_index_seen(static_cast<std::size_t>(orig_n), 0);
  std::vector<char> binary_index_seen(static_cast<std::size_t>(orig_n), 0);
  for (int idx : prob.integer_idx) {
    if (idx < 0 || idx >= orig_n) {
      out.stats.status = bc_status::kInvalidIntegralityIndex;
      return return_with_profile();
    }
    if (integer_index_seen[static_cast<std::size_t>(idx)] != 0) {
      out.stats.status = bc_status::kConflictingIntegralityDeclaration;
      return return_with_profile();
    }
    integer_index_seen[static_cast<std::size_t>(idx)] = 1;
  }
  for (int idx : prob.binary_idx) {
    if (idx < 0 || idx >= orig_n) {
      out.stats.status = bc_status::kInvalidIntegralityIndex;
      return return_with_profile();
    }
    if (binary_index_seen[static_cast<std::size_t>(idx)] != 0 ||
        integer_index_seen[static_cast<std::size_t>(idx)] != 0) {
      out.stats.status = bc_status::kConflictingIntegralityDeclaration;
      return return_with_profile();
    }
    binary_index_seen[static_cast<std::size_t>(idx)] = 1;
  }

  const auto declared_integrality =
      normalize_declared_integrality(prob, base_lp);
  const auto& original_declared_integer_cols =
      declared_integrality.integer_cols;
  const auto& original_declared_binary_cols =
      declared_integrality.binary_cols;
  if (!strict_highs_root_fixed_point) {
    constexpr double kModelInfinityThreshold = 1e19;
    for (int j = 0; j < orig_n; ++j) {
      if (original_declared_integer_cols[static_cast<std::size_t>(j)] == 0) {
        continue;
      }
      const auto& var = base_lp.vars[static_cast<std::size_t>(j)];
      const bool lower_unbounded =
          !std::isfinite(var.lb) || var.lb <= -kModelInfinityThreshold;
      const bool upper_unbounded =
          !std::isfinite(var.ub) || var.ub >= kModelInfinityThreshold;
      if (lower_unbounded && upper_unbounded) {
        out.stats.status = bc_status::kUnsupportedFreeIntegerVariable;
        return return_with_profile();
      }
    }
  }
  // Deep-copied only in strict-HiGHS mode: every use below is guarded by
  // strict_highs_root_fixed_point (directly or via
  // strict_highs_evaluate_root_enabled), so non-strict runs skip the copy.
  const LPModel strict_highs_original_entry_lp =
      strict_highs_root_fixed_point ? base_lp : LPModel{};

  if (strict_highs_root_fixed_point &&
      !bc_env_options().root_separation_only) {
    out.bc_stats.collection_scope = "strict_highs_summary";
    out.bc_stats.lp_solve_count_available = false;
    out.bc_stats.incumbent_timeline_available = false;
    out.bc_stats.bound_event_stream_available = false;
    out.bc_stats.cut_diagnostics_available = false;
    out.bc_stats.native_diagnostics_available = false;
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
    const auto t_highs_full0 = std::chrono::steady_clock::now();
    Highs highs;
    const bool highs_timeline_log = bc_env_flag_enabled("HACDCPF_HIGHS_TIMELINE");
    highs.setOptionValue("output_flag", highs_timeline_log);
    highs.setOptionValue("log_to_console", highs_timeline_log);
    // Match HiGHS' own threading contract in strict full-solve mode:
    // threads=0 means automatic.  Forcing the native wrapper to one thread
    // while the reference HiGHS row uses automatic threading makes the
    // algorithm identical but the performance comparison invalid.
    const HighsInt strict_highs_threads =
        opt.num_threads >= 1 ? static_cast<HighsInt>(opt.num_threads)
                             : static_cast<HighsInt>(0);
    highs.setOptionValue("threads", strict_highs_threads);
    highs.setOptionValue("presolve", opt.highs_force_presolve_on ? "on" : "choose");
    if (opt.highs_presolve_substitution_maxfillin > 0) {
      highs.setOptionValue(
          "presolve_substitution_maxfillin",
          static_cast<HighsInt>(opt.highs_presolve_substitution_maxfillin));
    }
    highs.setOptionValue("solver", "choose");
    // [I] Root LP solver: "ipm" → IPX/HiPO at root (no basis).
    //     If highs_mip_root_crossover="on"  (default): crossover → simplex basis
    //       → dual simplex warm-starts at all sub-tree node LPs.
    //     If highs_mip_root_crossover="off": pure IPM, no crossover and no
    //       root basis handoff. The net solve effect is instance-dependent.
    highs.setOptionValue("mip_lp_solver", opt.highs_mip_lp_solver);
    highs.setOptionValue("mip_ipm_solver", "choose");  // pick IPX vs HiPO automatically
    highs.setOptionValue("run_crossover", opt.highs_mip_root_crossover);
    if (opt.highs_mip_root_simplex_iteration_limit > 0) {
      highs.setOptionValue(
          "simplex_iteration_limit",
          static_cast<HighsInt>(opt.highs_mip_root_simplex_iteration_limit));
    }
    highs.setOptionValue("mip_rel_gap", opt.gap_tol);
    highs.setOptionValue("time_limit", std::max(0.001, opt.time_limit_sec));
    highs.setOptionValue("mip_max_nodes",
                         static_cast<HighsInt>(std::max(1, opt.max_nodes)));
    highs.setOptionValue("mip_feasibility_tolerance",
                         std::max(1e-10, highs_mip_feastol));
    highs.setOptionValue("primal_feasibility_tolerance",
                         std::max(1e-10, opt.lp_tol));
    highs.setOptionValue("dual_feasibility_tolerance",
                         std::max(1e-10, opt.lp_tol));
    // [D] Symmetry detection: controlled by opt.highs_mip_detect_symmetry.
    highs.setOptionValue("mip_detect_symmetry", opt.highs_mip_detect_symmetry);
    // [C] Root heuristic effort: controlled by opt.highs_mip_heuristic_effort.
    highs.setOptionValue("mip_heuristic_effort", opt.highs_mip_heuristic_effort);
    // [E] Pseudocost reliability: controlled by opt.highs_mip_pscost_minreliable.
    highs.setOptionValue("mip_pscost_minreliable",
                         static_cast<HighsInt>(opt.highs_mip_pscost_minreliable));
    // [H] Stall node limit: controlled by opt.highs_mip_max_stall_nodes.
    if (opt.highs_mip_max_stall_nodes > 0) {
      highs.setOptionValue("mip_max_stall_nodes",
                           static_cast<HighsInt>(opt.highs_mip_max_stall_nodes));
    }

    // [K1] Improvement (1): extra HiGHS primal heuristics.  ZI-Round and
    // Shifting are cheap LP-rounding heuristics that HiGHS leaves off by
    // default; enabling them here is opt-in via BCOptions so callers that
    // need the historical HiGHS-default behaviour are unaffected.
    highs.setOptionValue("mip_heuristic_run_zi_round",
                         opt.highs_mip_run_zi_round);
    highs.setOptionValue("mip_heuristic_run_shifting",
                         opt.highs_mip_run_shifting);
    // [K2] Improvement (1): cutpool size overrides.  0 → keep HiGHS defaults.
    if (opt.highs_mip_pool_age_limit > 0) {
      highs.setOptionValue("mip_pool_age_limit",
                           static_cast<HighsInt>(opt.highs_mip_pool_age_limit));
    }
    if (opt.highs_mip_pool_soft_limit > 0) {
      highs.setOptionValue("mip_pool_soft_limit",
                           static_cast<HighsInt>(opt.highs_mip_pool_soft_limit));
    }
    // [K4] Improvement (3): raise mip_lp_age_limit to retain dynamic cuts in
    // the per-node LP relaxation longer, tightening the bound across deep
    // sub-trees.  0 leaves HiGHS' default (10).
    if (opt.highs_mip_lp_age_limit > 0) {
      highs.setOptionValue("mip_lp_age_limit",
                           static_cast<HighsInt>(opt.highs_mip_lp_age_limit));
    }

    bool strict_highs_ok =
        bc_pass_mip_model_to_highs(highs, strict_highs_original_entry_lp,
                                   nullptr, nullptr);
    std::string strict_highs_error;
    if (!strict_highs_ok) {
      strict_highs_error = "pass_model_failed";
    } else {
      // StrictHiGHS policy: pass caller-provided branching priorities.
      if (!prob.branching_priority.empty() &&
          static_cast<int>(prob.branching_priority.size()) == orig_n) {
        std::vector<HighsInt> hp(prob.branching_priority.begin(),
                                 prob.branching_priority.end());
        highs.hacdcpfSetColBranchingPriorities(
            static_cast<HighsInt>(hp.size()), hp.data());
      }
      // StrictHiGHS policy: inject caller-provided root cuts from a previous
      // solve. Each row is in original column space and is validated by the
      // same model/presolve path as any other input row.
      if (opt.highs_root_cut_warm_start &&
          !opt.highs_root_cut_warm_start->empty()) {
        const auto& rc = *opt.highs_root_cut_warm_start;
        const int nc = rc.numCuts();
        const int nnz = static_cast<int>(rc.index.size());
        // Convert BCRootCuts int indices to HighsInt if needed.
        std::vector<HighsInt> hi_start(rc.start.begin(), rc.start.end());
        std::vector<HighsInt> hi_index(rc.index.begin(), rc.index.end());
        highs.addRows(static_cast<HighsInt>(nc), rc.lower.data(),
                      rc.upper.data(), static_cast<HighsInt>(nnz),
                      hi_start.data(), hi_index.data(), rc.value.data());
        if (opt.highs_root_basis_warm_start &&
            !opt.highs_root_basis_warm_start->empty() &&
            opt.highs_root_basis_warm_start->col_status.size() ==
                static_cast<std::size_t>(highs.getNumCol()) &&
            opt.highs_root_basis_warm_start->row_status.size() ==
                static_cast<std::size_t>(highs.getNumRow())) {
          const auto& rb = *opt.highs_root_basis_warm_start;
          HighsBasis basis;
          basis.col_status.reserve(rb.col_status.size());
          basis.row_status.reserve(rb.row_status.size());
          bool basis_ok = true;
          auto append_status = [&](int raw,
                                   std::vector<HighsBasisStatus>& out) {
            if (raw < static_cast<int>(HighsBasisStatus::kLower) ||
                raw > static_cast<int>(HighsBasisStatus::kNonbasic)) {
              basis_ok = false;
              return;
            }
            out.push_back(static_cast<HighsBasisStatus>(raw));
          };
          for (int raw : rb.col_status) append_status(raw, basis.col_status);
          for (int raw : rb.row_status) append_status(raw, basis.row_status);
          if (basis_ok) {
            basis.valid = true;
            basis.useful = true;
            basis.alien = true;
            basis.debug_origin_name = "HACDCPF Baseline+Cuts root basis";
            highs.hacdcpfSetRootBasis(basis);
          }
        }
        // Keep upstream dynamic root separation available after injecting
        // static cuts. Use the explicit caller cap when present, otherwise the
        // wrapper policy value of 50 rounds.
        const int sepa_cap = (opt.highs_max_root_sepa_rounds > 0)
                             ? opt.highs_max_root_sepa_rounds : 50;
        highs.setOptionValue("hacdcpf_max_root_sepa_rounds",
                             static_cast<HighsInt>(sepa_cap));
        // When a warm incumbent will also be injected, suppress repeated
        // dive-loop RENS/RINS sub-MIPs so the budget goes to the search tree.
        // Skip the separate analytic-centre request on a simplex root when a
        // warm incumbent is already supplied. An IPM root may reuse its
        // interior point through the upstream implementation.
        if (prob.initial_solution.size() > 0) {
          if (opt.highs_mip_lp_solver != "ipm") {
            highs.hacdcpfSetSkipAnalyticCenter(true);
          }
          highs.hacdcpfSetDiveSubMipHeuristicLimit(4);
          highs.hacdcpfSetHeuristicSubMipNodeLimit(120);
        }
      } else if (opt.highs_max_root_sepa_rounds > 0) {
        highs.setOptionValue("hacdcpf_max_root_sepa_rounds",
                             static_cast<HighsInt>(opt.highs_max_root_sepa_rounds));
      }
      // [A] Warm-start injection: verify the caller-supplied initial solution
      // in the original LP space (no PaPILO reduction; dispatch LP fixes
      // integers and solves one IPM LP for the continuous dispatch) and pass
      // it to HiGHS via setSolution().  HighsMipSolver's constructor reads
      // solution_.value_valid, calls solutionFeasible(), sets
      // solution_objective_, and stores solution_.  HighsMipSolverData then
      // projects it to the presolved space via
      // postSolveStack.getReducedPrimalSolution(), calls addIncumbent(), and
      // sets upper_limit -- enabling pruning via
      // ObjectivePropagation::propagate() from the first B&B node.
      // [B] upper_limit is set automatically once setSolution() succeeds;
      // no separate objective_bound option call is needed for MIP.
      out.bc_stats.lp_solves += bc_strict_highs_inject_warm_start(
          highs, base_lp, prob.initial_solution, opt);

      // [J] Native crash-basis root seeding.
      // Solve a bounded auxiliary root LP with HiGHS IPM (no crossover) to get
      // an interior primal point quickly, recover a native SimplexBasis from
      // the IPM partition signal, convert it to a HighsBasis (alien=true,
      // useful=true), and inject it via Highs::setBasis().  This keeps the
      // expensive native IPM/push/cleanup phases out of the StrictHiGHS wall
      // clock while still giving HiGHS a root basis better than the default
      // logical crash basis on highly degenerate UC LPs.
      if (opt.highs_strict_seed_native_ipm_basis) {
        const bool seed_trace =
          opt.verbose || bc_env_flag_enabled("MIPSOLVERS_BC_TIMELINE");
        const auto t_seed0 = std::chrono::steady_clock::now();
        const double seed_time_limit = std::max(
            0.001, std::min(opt.highs_strict_seed_native_ipm_basis_time_limit_sec,
                            0.25 * opt.time_limit_sec));
        if (seed_trace) {
          std::fprintf(stderr,
            "[STRICT_HIGHS] Native IPM crash-basis seed start "
            "(vars=%d, rows=%d, lpTime=%.3fs)\n",
            static_cast<int>(strict_highs_original_entry_lp.vars.size()),
            static_cast<int>(strict_highs_original_entry_lp.A.rows() +
                     strict_highs_original_entry_lp.Aeq.rows()),
            seed_time_limit);
        }

        Highs seed_highs;
        seed_highs.setOptionValue("output_flag", false);
        seed_highs.setOptionValue("log_to_console", false);
        seed_highs.setOptionValue("threads", strict_highs_threads);
        seed_highs.setOptionValue("presolve", "choose");
        seed_highs.setOptionValue("solver", "ipm");
        seed_highs.setOptionValue("run_crossover", "off");
        seed_highs.setOptionValue("time_limit", seed_time_limit);
        seed_highs.setOptionValue("primal_feasibility_tolerance",
                                  std::max(1e-10, opt.lp_tol));
        seed_highs.setOptionValue("dual_feasibility_tolerance",
                                  std::max(1e-10, opt.lp_tol));
        LPModel seed_lp = strict_highs_original_entry_lp;
        for (auto& var : seed_lp.vars) var.type = VarType::Continuous;
        const bool seed_lp_passed =
            bc_pass_mip_model_to_highs(seed_highs, seed_lp, nullptr, nullptr);

        SolveResult seed_ipm_res;
        HighsModelStatus seed_model_status = HighsModelStatus::kNotset;
        HighsStatus seed_run_status = HighsStatus::kError;
        if (seed_lp_passed) {
          seed_run_status = seed_highs.run();
          seed_model_status = seed_highs.getModelStatus();
          const HighsSolution& seed_sol = seed_highs.getSolution();
          const int seed_n = static_cast<int>(strict_highs_original_entry_lp.vars.size());
          if (static_cast<int>(seed_sol.col_value.size()) >= seed_n &&
              (seed_sol.value_valid ||
               seed_model_status == HighsModelStatus::kOptimal ||
               seed_model_status == HighsModelStatus::kTimeLimit)) {
            seed_ipm_res.x = Eigen::VectorXd::Zero(seed_n);
            for (int j = 0; j < seed_n; ++j) {
              seed_ipm_res.x[j] = seed_sol.col_value[static_cast<std::size_t>(j)];
            }
            const int seed_rows = static_cast<int>(
                strict_highs_original_entry_lp.A.rows() +
                strict_highs_original_entry_lp.Aeq.rows());
            if (static_cast<int>(seed_sol.row_dual.size()) >= seed_rows) {
              seed_ipm_res.constraint_duals = Eigen::VectorXd::Zero(seed_rows);
              for (int i = 0; i < seed_rows; ++i) {
                seed_ipm_res.constraint_duals[i] =
                    seed_sol.row_dual[static_cast<std::size_t>(i)];
              }
            }
            seed_ipm_res.stats.success = seed_ipm_res.x.allFinite();
            seed_ipm_res.stats.status = bc_highs_model_status_label(seed_model_status);
            seed_ipm_res.stats.objective = objective_value(
                strict_highs_original_entry_lp.c, seed_ipm_res.x,
                strict_highs_original_entry_lp.sense);
          }
        }
        bool seed_injected = false;
        CrashBasisRecoveryStats seed_recovery;
        if (seed_ipm_res.stats.success &&
            seed_ipm_res.x.size() ==
                static_cast<int>(strict_highs_original_entry_lp.vars.size())) {
          StandardFormLP seed_sf = build_standard_form_lp(strict_highs_original_entry_lp);
          ruiz_scale_standard_form(seed_sf);
          SimplexBasis seed_basis;
          seed_recovery = recover_primal_activity_basis(
              seed_sf, seed_ipm_res.x, &seed_ipm_res,
              /*seed_basis=*/nullptr, seed_basis);
          seed_basis.sf_n_slack = seed_sf.n_slack;
          seed_basis.sf_n_surplus = seed_sf.n_surplus;
          seed_basis.sf_n_artificial = seed_sf.n_artificial;
          if (seed_recovery.selected_swaps > 0) {
            HighsBasis seed_hbasis;
            if (bc_native_basis_to_highs_basis(strict_highs_original_entry_lp,
                                               &seed_basis, seed_hbasis)) {
              (void)highs.setBasis(
                  seed_hbasis,
                  "MIPSOLVERS native IPM crash root basis");
              seed_injected = true;
            }
          }
        }
        const auto t_seed1 = std::chrono::steady_clock::now();
        const double seed_ms =
          std::chrono::duration<double, std::milli>(t_seed1 - t_seed0)
            .count();
        out.bc_stats.lp_solves += 1;
        if (seed_trace && seed_injected) {
          std::fprintf(stderr,
              "[STRICT_HIGHS] Native IPM crash-basis injected "
              "(time=%.1fms, obj=%.6g, swaps=%d, candRows=%d, screen=%d)\n",
              seed_ms, seed_ipm_res.stats.objective,
              seed_recovery.selected_swaps, seed_recovery.candidate_rows,
              seed_recovery.passes_screen ? 1 : 0);
        } else if (seed_trace) {
          std::fprintf(stderr,
              "[STRICT_HIGHS] Native IPM crash-basis seed skipped "
            "(pass=%d, run=%d, lpStatus=%s, ipm_success=%d, swaps=%d, "
            "time=%.1fms)\n",
            seed_lp_passed ? 1 : 0,
            static_cast<int>(seed_run_status),
            bc_highs_model_status_label(seed_model_status),
              seed_ipm_res.stats.success ? 1 : 0,
              seed_recovery.selected_swaps, seed_ms);
        }
        const double elapsed_sec =
            std::chrono::duration<double>(t_seed1 - t0).count();
        highs.setOptionValue(
            "time_limit", std::max(0.001, opt.time_limit_sec - elapsed_sec));
      }

      BCCallbacks strict_highs_effective_callbacks;
      const BCCallbacks* strict_highs_callbacks = callbacks;
      if (prob.uc_hint.has_value() &&
          strict_scuc_dynamic_cuts_enabled(*prob.uc_hint)) {
        if (callbacks != nullptr) strict_highs_effective_callbacks = *callbacks;
        auto external_dynamic_node_cut = strict_highs_effective_callbacks.dynamic_node_cut;
        strict_highs_effective_callbacks.dynamic_node_cut =
            [external_dynamic_node_cut, &prob, &opt](
                const BCDynamicNodeCutContext& ctx,
                std::vector<BCDynamicNodeCut>& rows) {
              if (external_dynamic_node_cut) {
                external_dynamic_node_cut(ctx, rows);
              }
              if (prob.uc_hint.has_value()) {
                append_strict_scuc_dynamic_cuts(*prob.uc_hint, opt, ctx,
                                                &prob.linear_part, rows);
              }
            };
        strict_highs_callbacks = &strict_highs_effective_callbacks;
      }
      const bool strict_highs_dynamic_callback_active =
          strict_highs_callbacks != nullptr &&
          static_cast<bool>(strict_highs_callbacks->dynamic_node_cut);
      StrictHighsNodeCutCallbackBridge callback_bridge{
          strict_highs_callbacks, &strict_highs_original_entry_lp};
      if (strict_highs_dynamic_callback_active) {
        highs.hacdcpfSetNodeCutCallback(
            [](const char* event, HighsLpRelaxation* lp,
               const HighsDomain* local_domain,
               const presolve::HighsPostsolveStack* postsolve_stack,
               HighsCutSet* cutset,
               int64_t node_count, HighsInt depth, void* user_data) {
              if (lp == nullptr || cutset == nullptr || user_data == nullptr) {
                return;
              }
              const auto* bridge =
                  static_cast<const StrictHighsNodeCutCallbackBridge*>(user_data);
              const BCCallbacks* cbs = bridge->callbacks;
              if (cbs == nullptr) return;
              if (!cbs->dynamic_node_cut) return;

              BCDynamicNodeCutContext ctx;
              ctx.event = event != nullptr ? event : "";
              ctx.node_count = node_count;
              ctx.depth = static_cast<int>(depth);
              ctx.lp_objective = lp->getObjective();
              const HighsSolution& sol = lp->getLpSolver().getSolution();
              ctx.col_value = sol.col_value;
              if (local_domain != nullptr) {
                ctx.col_lower = local_domain->col_lower_;
                ctx.col_upper = local_domain->col_upper_;
              }
              const HighsInt ncols = lp->numCols();
              if (postsolve_stack != nullptr) {
                const HighsInt norig = postsolve_stack->getOrigNumCol();
                if (norig > 0) {
                  HighsSolution original_sol;
                  original_sol.value_valid = true;
                  original_sol.dual_valid = false;
                  original_sol.col_value = sol.col_value;
                  original_sol.row_value.assign(
                      static_cast<std::size_t>(
                          std::max<HighsInt>(0, lp->numRows())),
                      0.0);
                  try {
                    auto* mutable_stack =
                        const_cast<presolve::HighsPostsolveStack*>(
                            postsolve_stack);
                    mutable_stack->undoPrimal(lp->getLpSolver().getOptions(),
                                              original_sol);
                    if (static_cast<HighsInt>(
                            original_sol.col_value.size()) >= norig) {
                      ctx.original_col_value.assign(
                          original_sol.col_value.begin(),
                          original_sol.col_value.begin() + norig);
                    }
                  } catch (...) {
                    ctx.original_col_value.assign(
                        static_cast<std::size_t>(norig),
                        std::numeric_limits<double>::quiet_NaN());
                  }
                  if (ctx.original_col_value.empty()) {
                    ctx.original_col_value.assign(
                        static_cast<std::size_t>(norig),
                        std::numeric_limits<double>::quiet_NaN());
                  }
                }
                ctx.reduced_to_original_col.reserve(
                    static_cast<std::size_t>(std::max<HighsInt>(0, ncols)));
                for (HighsInt col = 0; col < ncols; ++col) {
                  int orig_col = -1;
                  if (col < static_cast<HighsInt>(sol.col_value.size())) {
                    orig_col = static_cast<int>(
                        postsolve_stack->getOrigColIndex(col));
                  }
                  ctx.reduced_to_original_col.push_back(orig_col);
                }
              }

              std::vector<BCDynamicNodeCut> rows;
              cbs->dynamic_node_cut(ctx, rows);
              if (rows.empty()) return;

              cutset->clear();
              cutset->cutindices.reserve(rows.size() * 2);
              cutset->lower_.reserve(rows.size() * 2);
              cutset->upper_.reserve(rows.size() * 2);
              cutset->hacdcpf_integral_.reserve(rows.size() * 2);
              cutset->hacdcpf_propagate_.reserve(rows.size() * 2);
              cutset->hacdcpf_validity_scope_.reserve(rows.size() * 2);
              cutset->ARstart_.clear();
              cutset->ARstart_.push_back(0);
              const bool callback_cut_log =
                  bc_env_options().value("MIPSOLVERS_BENCH_DYNAMIC_MARKET_CUT_LOG") != nullptr ||
                  bc_env_options().value("MIPSOLVERS_HIGHS_USERCUT_LEDGER") != nullptr ||
                  bc_env_options().value("MIPSOLVERS_USERCUT_LEDGER") != nullptr;
              auto dynamic_cut_row_hash =
                  [&](const std::vector<HighsInt>& indices,
                      const std::vector<double>& values, double upper) {
                    std::uint64_t h = std::uint64_t{0x4841434455435554};
                    h = root_xpool_ledger_mix(
                        h, static_cast<std::uint64_t>(indices.size()));
                    h = root_xpool_ledger_mix(h,
                                              root_xpool_ledger_double(upper));
                    for (std::size_t k = 0; k < indices.size(); ++k) {
                      h = root_xpool_ledger_mix(
                          h, static_cast<std::uint64_t>(indices[k]));
                      h = root_xpool_ledger_mix(
                          h, root_xpool_ledger_double(values[k]));
                    }
                    return h;
                  };
              auto log_projection =
                  [&](const char* status, const std::string& reason,
                      const BCDynamicNodeCut& row, std::size_t projected_len,
                      double projected_upper) {
                    if (!callback_cut_log) return;
                    std::cerr << "[HIGHS-CB-CUT-PROJECT] status=" << status
                              << " reason=" << reason
                              << " event=" << ctx.event
                              << " node=" << node_count
                              << " depth=" << depth
                              << " family=" << row.audit_family
                              << " key=" << row.audit_key
                              << " period=" << row.audit_period
                              << " origLen=" << row.indices.size()
                              << " projectedLen=" << projected_len
                              << " upper=" << projected_upper << "\n";
                  };
              auto log_projection_audit =
                  [&](const char* status, const std::string& reason,
                      const BCDynamicNodeCut& row, std::uint64_t row_hash,
                      double original_activity, double projected_activity,
                      double original_upper, double projected_upper,
                      double diff, double tolerance) {
                    if (!callback_cut_log) return;
                    std::cerr << "[HIGHS-CB-CUT-PROJECT-AUDIT]"
                              << " status=" << status
                              << " reason=" << reason
                              << " event=" << ctx.event
                              << " node=" << node_count
                              << " depth=" << depth
                              << " family=" << row.audit_family
                              << " key=" << row.audit_key
                              << " period=" << row.audit_period
                              << " rowHash=" << std::hex << row_hash
                              << std::dec
                              << " origActivity=" << original_activity
                              << " projectedActivity=" << projected_activity
                              << " origUpper=" << original_upper
                              << " projectedUpper=" << projected_upper
                              << " diff=" << diff
                              << " tol=" << tolerance << "\n";
                  };
              auto row_activity = [](const std::vector<HighsInt>& indices,
                                     const std::vector<double>& values,
                                     const std::vector<double>& solution,
                                     double* norm_out) {
                double activity = 0.0;
                double norm_sq = 0.0;
                if (indices.size() != values.size()) {
                  if (norm_out != nullptr) *norm_out = 0.0;
                  return std::numeric_limits<double>::quiet_NaN();
                }
                for (std::size_t k = 0; k < indices.size(); ++k) {
                  const HighsInt col = indices[k];
                  const double value = values[k];
                  if (col < 0 ||
                      col >= static_cast<HighsInt>(solution.size()) ||
                      !std::isfinite(value) ||
                      !std::isfinite(solution[static_cast<std::size_t>(col)])) {
                    if (norm_out != nullptr) *norm_out = 0.0;
                    return std::numeric_limits<double>::quiet_NaN();
                  }
                  activity += value * solution[static_cast<std::size_t>(col)];
                  norm_sq += value * value;
                }
                if (norm_out != nullptr) *norm_out = std::sqrt(norm_sq);
                return activity;
              };
              auto projection_audit_passes =
                  [&](const BCDynamicNodeCut& row,
                      const std::vector<HighsInt>& orig_indices,
                      const std::vector<double>& orig_values,
                      double original_upper,
                      const presolve::HacdcpfRowProjectionResult& projected,
                      std::uint64_t row_hash) {
                    double orig_norm = 0.0;
                    const double original_activity =
                        row_activity(orig_indices, orig_values,
                                     ctx.original_col_value, &orig_norm);
                    double projected_norm = 0.0;
                    const double projected_activity =
                        row_activity(projected.indices, projected.values,
                                     sol.col_value, &projected_norm);
                    const double diff =
                        std::fabs(original_activity - projected_activity);
                    const double upper_diff =
                        std::fabs(original_upper - projected.upper);
                    const double scale =
                        std::max({1.0, std::fabs(original_activity),
                                  std::fabs(projected_activity),
                                  std::fabs(original_upper),
                                  std::fabs(projected.upper), orig_norm,
                                  projected_norm});
                    const double tolerance = 1e-8 * scale;
                    const bool ok =
                        std::isfinite(original_activity) &&
                        std::isfinite(projected_activity) &&
                        std::isfinite(original_upper) &&
                        std::isfinite(projected.upper) &&
                        diff <= tolerance && upper_diff <= tolerance;
                    log_projection_audit(
                        ok ? "exact" : "reject",
                        ok ? "activity_rhs_match"
                           : "activity_or_rhs_mismatch",
                        row, row_hash, original_activity, projected_activity,
                        original_upper, projected.upper,
                        std::max(diff, upper_diff), tolerance);
                    return ok;
                  };
              auto append_projected_cutpool_upper_row =
                  [&](const BCDynamicNodeCut& row,
                      const std::vector<HighsInt>& indices,
                      const std::vector<double>& values, double upper) {
                    if (indices.size() != values.size()) return false;
                    if (!std::isfinite(upper)) return false;
                    const HighsInt start =
                        static_cast<HighsInt>(cutset->ARindex_.size());
                    std::unordered_map<int, double> merged_terms;
                    merged_terms.reserve(indices.size());
                    for (std::size_t k = 0; k < indices.size(); ++k) {
                      const HighsInt col = indices[k];
                      const double val = values[k];
                      if (col < 0 || col >= ncols || !std::isfinite(val) ||
                          val == 0.0) {
                        continue;
                      }
                      merged_terms[static_cast<int>(col)] += val;
                    }
                    if (merged_terms.empty()) return false;
                    for (const auto& [col, val] : merged_terms) {
                      if (std::abs(val) <= 0.0 || !std::isfinite(val)) continue;
                      cutset->ARindex_.push_back(static_cast<HighsInt>(col));
                      cutset->ARvalue_.push_back(val);
                    }
                    const HighsInt end =
                        static_cast<HighsInt>(cutset->ARindex_.size());
                    if (end == start) return false;
                    cutset->cutindices.push_back(-2);
                    cutset->lower_.push_back(-kHighsInf);
                    cutset->upper_.push_back(upper);
                    cutset->hacdcpf_integral_.push_back(row.integral ? 1 : 0);
                    cutset->hacdcpf_propagate_.push_back(row.propagate ? 1 : 0);
                    unsigned char scope =
                        HighsCutSet::kHacdcpfGlobalCut;
                    if (row.validity_scope == ValidityScope::LocalNodeCut) {
                      scope = HighsCutSet::kHacdcpfLocalNodeCut;
                    } else if (row.validity_scope ==
                               ValidityScope::LazyConstraint) {
                      scope = HighsCutSet::kHacdcpfLazyConstraint;
                    }
                    cutset->hacdcpf_validity_scope_.push_back(scope);
                    cutset->hacdcpf_proof_family_.push_back(row.audit_family);
                    cutset->hacdcpf_proof_key_.push_back(row.audit_key);
                    cutset->ARstart_.push_back(end);
                    return true;
                  };
              auto append_cutpool_upper_row =
                  [&](const BCDynamicNodeCut& row, double sign,
                      double upper) {
                    if (row.column_space ==
                        BCDynamicNodeCut::ColumnSpace::Original) {
                      if (postsolve_stack == nullptr) {
                        log_projection("reject", "missing_postsolve_stack",
                                       row, 0, upper);
                        return;
                      }
                      std::vector<HighsInt> orig_indices;
                      std::vector<double> orig_values;
                      orig_indices.reserve(row.indices.size());
                      orig_values.reserve(row.values.size());
                      for (std::size_t k = 0; k < row.indices.size(); ++k) {
                        orig_indices.push_back(
                            static_cast<HighsInt>(row.indices[k]));
                        orig_values.push_back(sign * row.values[k]);
                      }
                      const std::uint64_t row_hash =
                          dynamic_cut_row_hash(orig_indices, orig_values,
                                               upper);
                      presolve::HacdcpfRowProjectionResult projected;
                      if (!postsolve_stack
                               ->hacdcpfProjectOriginalUpperRowToPresolved(
                                   orig_indices, orig_values, upper,
                                   projected)) {
                        log_projection_audit(
                            "reject", projected.reason, row, row_hash,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(), upper,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(), 0.0);
                        log_projection("reject", projected.reason, row, 0,
                                       upper);
                        return;
                      }
                      if (!projected.ok || !projected.exact) {
                        log_projection_audit(
                            "reject", "projection_not_exact", row, row_hash,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(), upper,
                            projected.upper,
                            std::numeric_limits<double>::quiet_NaN(), 0.0);
                        log_projection("reject", "projection_not_exact", row,
                                       projected.indices.size(),
                                       projected.upper);
                        return;
                      }
                      if (!projection_audit_passes(row, orig_indices,
                                                   orig_values, upper,
                                                   projected, row_hash)) {
                        log_projection("reject",
                                       "activity_or_rhs_mismatch", row,
                                       projected.indices.size(),
                                       projected.upper);
                        return;
                      }
                      const bool appended = append_projected_cutpool_upper_row(
                          row, projected.indices, projected.values,
                          projected.upper);
                      log_projection(appended ? "exact" : "reject",
                                     appended ? projected.reason
                                              : "empty_after_projection",
                                     row, projected.indices.size(),
                                     projected.upper);
                      return;
                    }

                    std::vector<HighsInt> lp_indices;
                    std::vector<double> lp_values;
                    lp_indices.reserve(row.indices.size());
                    lp_values.reserve(row.values.size());
                    for (std::size_t k = 0; k < row.indices.size(); ++k) {
                      lp_indices.push_back(
                          static_cast<HighsInt>(row.indices[k]));
                      lp_values.push_back(sign * row.values[k]);
                    }
                    (void)append_projected_cutpool_upper_row(
                        row, lp_indices, lp_values, upper);
                  };
              for (const BCDynamicNodeCut& row : rows) {
                if (row.indices.size() != row.values.size()) continue;
                const bool has_lower = std::isfinite(row.lower);
                const bool has_upper = std::isfinite(row.upper);
                if (!has_lower && !has_upper) continue;
                if (has_upper) append_cutpool_upper_row(row, 1.0, row.upper);
                if (has_lower) append_cutpool_upper_row(row, -1.0, -row.lower);
              }
              if (cutset->cutindices.empty()) {
                cutset->clear();
              }
            },
            &callback_bridge);
      } else {
        highs.hacdcpfSetNodeCutCallback(nullptr, nullptr);
      }

      (void)highs.setOptionValue(
          "hacdcpf_retain_root_lp_relaxation",
          bc_env_options().strict_full_retain_root_lp);
      // [K7] Inject pseudocost warm-start (original → presolved space mapping
      // is handled inside HighsPseudocost constructor using postSolveStack).
      if (opt.highs_pseudocost_warm_start &&
          !opt.highs_pseudocost_warm_start->empty() &&
          opt.highs_pseudocost_warm_start->n_orig_cols == highs.getNumCol()) {
        const auto& bp = *opt.highs_pseudocost_warm_start;
        auto hi_pscost = std::make_shared<HighsPseudocostInitialization>();
        hi_pscost->pseudocostup.assign(bp.pseudocostup.begin(),
                                       bp.pseudocostup.end());
        hi_pscost->pseudocostdown.assign(bp.pseudocostdown.begin(),
                                         bp.pseudocostdown.end());
        hi_pscost->nsamplesup.assign(bp.nsamplesup.begin(),
                                     bp.nsamplesup.end());
        hi_pscost->nsamplesdown.assign(bp.nsamplesdown.begin(),
                                       bp.nsamplesdown.end());
        hi_pscost->inferencesup.assign(bp.inferencesup.begin(),
                                        bp.inferencesup.end());
        hi_pscost->inferencesdown.assign(bp.inferencesdown.begin(),
                                          bp.inferencesdown.end());
        hi_pscost->ninferencesup.assign(bp.ninferencesup.begin(),
                                         bp.ninferencesup.end());
        hi_pscost->ninferencesdown.assign(bp.ninferencesdown.begin(),
                                           bp.ninferencesdown.end());
        hi_pscost->conflictscoreup.assign(bp.conflictscoreup.begin(),
                                           bp.conflictscoreup.end());
        hi_pscost->conflictscoredown.assign(bp.conflictscoredown.begin(),
                                             bp.conflictscoredown.end());
        hi_pscost->cost_total         = bp.cost_total;
        hi_pscost->inferences_total   = bp.inferences_total;
        hi_pscost->conflict_avg_score = bp.conflict_avg_score;
        hi_pscost->nsamplestotal      = bp.nsamplestotal;
        hi_pscost->ninferencestotal   = bp.ninferencestotal;
        highs.hacdcpfSetPseudocostInitialization(std::move(hi_pscost));
      }
      const HighsStatus run_status = highs.run();
      // [K6] Extract root cuts immediately after run() for warm-starting
      // future solves of this problem.  Convert HacdcpfRootCuts (HighsInt
      // column indices) to BCRootCuts (int column indices).
      {
        HacdcpfRootCuts hacdcpf_cuts;
        highs.hacdcpfExtractRootCuts(hacdcpf_cuts);
        if (!hacdcpf_cuts.empty()) {
          auto bc_cuts = std::make_shared<BCRootCuts>();
          bc_cuts->start.assign(hacdcpf_cuts.start.begin(),
                                hacdcpf_cuts.start.end());
          bc_cuts->index.assign(hacdcpf_cuts.index.begin(),
                                hacdcpf_cuts.index.end());
          bc_cuts->value = std::move(hacdcpf_cuts.value);
          bc_cuts->lower = std::move(hacdcpf_cuts.lower);
          bc_cuts->upper = std::move(hacdcpf_cuts.upper);
          out.highs_root_cuts = std::move(bc_cuts);
        }
        HighsBasis hacdcpf_basis;
        highs.hacdcpfExtractRootBasis(hacdcpf_basis);
        if (hacdcpf_basis.valid && !hacdcpf_basis.col_status.empty() &&
            !hacdcpf_basis.row_status.empty()) {
          auto bc_basis = std::make_shared<BCRootBasis>();
          bc_basis->valid = true;
          bc_basis->col_status.reserve(hacdcpf_basis.col_status.size());
          bc_basis->row_status.reserve(hacdcpf_basis.row_status.size());
          for (HighsBasisStatus status : hacdcpf_basis.col_status) {
            bc_basis->col_status.push_back(static_cast<int>(status));
          }
          for (HighsBasisStatus status : hacdcpf_basis.row_status) {
            bc_basis->row_status.push_back(static_cast<int>(status));
          }
          out.highs_root_basis = std::move(bc_basis);
        }
      }
      // [K7] Extract pseudocost data for warm-starting branching in the next
      // solve.  HighsPseudocostInitialization maps presolved → original space.
      {
        HighsPseudocostInitialization hacdcpf_pscost;
        if (highs.hacdcpfExtractPseudocostInitialization(hacdcpf_pscost)) {
          auto bc_pscost = std::make_shared<BCPseudocostInit>();
          bc_pscost->n_orig_cols = static_cast<int>(
              hacdcpf_pscost.pseudocostup.size());
          bc_pscost->pseudocostup.assign(hacdcpf_pscost.pseudocostup.begin(),
                                         hacdcpf_pscost.pseudocostup.end());
          bc_pscost->pseudocostdown.assign(
              hacdcpf_pscost.pseudocostdown.begin(),
              hacdcpf_pscost.pseudocostdown.end());
          bc_pscost->nsamplesup.assign(hacdcpf_pscost.nsamplesup.begin(),
                                       hacdcpf_pscost.nsamplesup.end());
          bc_pscost->nsamplesdown.assign(hacdcpf_pscost.nsamplesdown.begin(),
                                         hacdcpf_pscost.nsamplesdown.end());
          bc_pscost->inferencesup.assign(hacdcpf_pscost.inferencesup.begin(),
                                          hacdcpf_pscost.inferencesup.end());
          bc_pscost->inferencesdown.assign(
              hacdcpf_pscost.inferencesdown.begin(),
              hacdcpf_pscost.inferencesdown.end());
          bc_pscost->ninferencesup.assign(hacdcpf_pscost.ninferencesup.begin(),
                                           hacdcpf_pscost.ninferencesup.end());
          bc_pscost->ninferencesdown.assign(
              hacdcpf_pscost.ninferencesdown.begin(),
              hacdcpf_pscost.ninferencesdown.end());
          bc_pscost->conflictscoreup.assign(
              hacdcpf_pscost.conflictscoreup.begin(),
              hacdcpf_pscost.conflictscoreup.end());
          bc_pscost->conflictscoredown.assign(
              hacdcpf_pscost.conflictscoredown.begin(),
              hacdcpf_pscost.conflictscoredown.end());
          bc_pscost->cost_total        = hacdcpf_pscost.cost_total;
          bc_pscost->inferences_total  = hacdcpf_pscost.inferences_total;
          bc_pscost->conflict_avg_score = hacdcpf_pscost.conflict_avg_score;
          bc_pscost->nsamplestotal     = hacdcpf_pscost.nsamplestotal;
          bc_pscost->ninferencestotal  = hacdcpf_pscost.ninferencestotal;
          out.highs_pseudocost_init    = std::move(bc_pscost);
        }
      }
      const auto t_highs_full1 = std::chrono::steady_clock::now();
      const double runtime =
          std::chrono::duration<double>(t_highs_full1 - t0).count();
      const double highs_runtime =
          std::chrono::duration<double>(t_highs_full1 - t_highs_full0).count();
      const HighsModelStatus model_status = highs.getModelStatus();
      const HighsInfo& info = highs.getInfo();
      const HighsSolution& sol = highs.getSolution();
      const int nsol = static_cast<int>(std::min<std::size_t>(
          sol.col_value.size(), static_cast<std::size_t>(orig_n)));
      if (nsol > 0) {
        out.x = Eigen::VectorXd::Zero(orig_n);
        for (int j = 0; j < nsol; ++j) {
          out.x[j] = sol.col_value[static_cast<std::size_t>(j)];
        }
      }

      const bool has_solution =
          sol.value_valid && out.x.size() == orig_n &&
          (model_status == HighsModelStatus::kOptimal ||
           model_status == HighsModelStatus::kObjectiveBound ||
           model_status == HighsModelStatus::kObjectiveTarget ||
           model_status == HighsModelStatus::kTimeLimit ||
           model_status == HighsModelStatus::kIterationLimit ||
           model_status == HighsModelStatus::kSolutionLimit);
      const bool optimal = model_status == HighsModelStatus::kOptimal;
      const bool feasible_limit =
          has_solution && (model_status == HighsModelStatus::kTimeLimit ||
                           model_status == HighsModelStatus::kIterationLimit ||
                           model_status == HighsModelStatus::kSolutionLimit ||
                           model_status == HighsModelStatus::kObjectiveBound ||
                           model_status == HighsModelStatus::kObjectiveTarget);
      double obj = kInf;
      if (has_solution) {
        obj = objective_value(prob.linear_part.c, out.x, prob.linear_part.sense);
      }
      double dual_bound = info.mip_dual_bound;
      if (prob.linear_part.sense == Sense::Maximize &&
          std::isfinite(dual_bound)) {
        dual_bound = -dual_bound;
      }
      double gap = info.mip_gap;
      if (!std::isfinite(gap) && has_solution && std::isfinite(dual_bound)) {
        if (prob.linear_part.sense == Sense::Minimize) {
          gap = std::max(0.0, (obj - dual_bound) /
                                  std::max(1.0, std::abs(obj)));
        } else {
          gap = std::max(0.0, (dual_bound - obj) /
                                  std::max(1.0, std::abs(obj)));
        }
      }

      // obj and dual_bound are in the internal minimization convention:
      // for Maximize problems, objective_value() returns -c.dot(x) and
      // dual_bound was negated from info.mip_dual_bound.  Convert to the
      // user-facing (sense-correct) values before writing to public fields.
      const double user_facing_obj =
          (prob.linear_part.sense == Sense::Minimize) ? obj : -obj;
      const double user_facing_bound =
          (prob.linear_part.sense == Sense::Minimize) ? dual_bound : -dual_bound;

      out.stats.solver_name = "NativeBranchAndCut[StrictHiGHSStateMachine]";
      out.stats.runtime_sec = runtime;
      out.stats.iterations = static_cast<int>(std::min<int64_t>(
          info.simplex_iteration_count, std::numeric_limits<int>::max()));
      out.stats.success = optimal || feasible_limit;
      out.stats.status = fmt::format("StrictHiGHS {} run={}",
                                     bc_highs_model_status_label(model_status),
                                     static_cast<int>(run_status));
      out.stats.objective = std::isfinite(user_facing_obj) ? user_facing_obj : 0.0;
      out.stats.mip_gap = std::isfinite(gap) ? gap : kInf;
      out.stats.primal_feas = has_solution ? 0.0 : kInf;
      out.stats.residual_inf = has_solution ? 0.0 : kInf;

      out.bc_stats.runtime_sec = runtime;
      out.bc_stats.nodes_explored = static_cast<int>(std::min<int64_t>(
          info.mip_node_count, std::numeric_limits<int>::max()));
      // HiGHS 1.14 exposes simplex iterations and MIP nodes, but not the
      // number of node-LP solves. nodes+1 is not an equivalent statistic.
      out.bc_stats.lp_solves = 0;
      out.bc_stats.best_bound = std::isfinite(user_facing_bound) ? user_facing_bound : -kInf;
      out.bc_stats.best_obj = std::isfinite(user_facing_obj) ? user_facing_obj : kInf;
      out.bc_stats.gap = out.stats.mip_gap;
      out.bc_stats.status = out.stats.status;
      out.bc_stats.parallel_requested_threads =
          static_cast<int>(strict_highs_threads);
      // Vendored HiGHS 1.14 has a serial MIP tree. Its `threads` option may
      // affect LP internals, but cannot be reported as parallel MIP workers.
      out.bc_stats.parallel_effective_threads = 1;
      out.bc_stats.parallel_explorer_threads = 0;
      out.bc_stats.parallel_schedule_reason = "strict_highs_1_14_serial_mip";

      if (opt.verbose || bc_env_flag_enabled("MIPSOLVERS_BC_TIMELINE")) {
        fmt::print(stderr,
                   "[B&C-STRICT-HIGHS-FULL] status={} run={} nodes={} "
                   "iters={} ipm={} xover={} obj={:.17g} bound={:.17g} gap={:.6g} "
                   "wall={:.3f}s highsWall={:.3f}s dynamicNodeCut={}\n",
                   bc_highs_model_status_label(model_status),
                   static_cast<int>(run_status), out.bc_stats.nodes_explored,
                   out.stats.iterations,
                   static_cast<long long>(info.ipm_iteration_count),
                   static_cast<long long>(info.crossover_iteration_count),
                   out.stats.objective,
                   out.bc_stats.best_bound, out.stats.mip_gap, runtime,
                   highs_runtime,
                   strict_highs_dynamic_callback_active ? 1 : 0);
      }
      return return_with_profile();
    }

    out.stats.solver_name = "NativeBranchAndCut[StrictHiGHSStateMachine]";
    out.stats.status = fmt::format("StrictHiGHS {}", strict_highs_error);
    out.bc_stats.status = out.stats.status;
    return return_with_profile();
#else
    out.stats.solver_name = "NativeBranchAndCut[StrictHiGHSStateMachine]";
    out.stats.status = "StrictHiGHS unavailable";
    out.bc_stats.status = out.stats.status;
    return return_with_profile();
#endif
  }

  // ── Pre-presolve: verify heuristic via IPM dispatch ─────────────────
  // If the caller provides an initial_solution (e.g. UC commitment), fix
  // the integer variables and solve one IPM LP for continuous dispatch in
  // the ORIGINAL variable space. This verifies feasibility and records the
  // objective for future use (greedy shutdown, etc).
  //
  // NOTE: The verified solution cannot be mapped through PaPILO's reduced
  // space because PaPILO performs variable substitutions that are
  // one-directional (reduced→original via postsolve, but not the reverse).
  // The objective is stored for reference; the incumbent is found via the
  // root heuristics (progressive rounding, feasibility pump) which operate
  // natively in the reduced space.
  double heuristic_obj = std::numeric_limits<double>::infinity();
  Eigen::VectorXd heuristic_x_orig;  // verified original-space solution
  const bool warmstart_provided = prob.initial_solution.size() == orig_n;
  const bool skip_huge_heuristic_verification = !opt.use_ipm_nodes && orig_n >= 90000;
  if (warmstart_provided && !strict_highs_lp_contract &&
      !skip_huge_heuristic_verification) {
    auto t_heur0 = std::chrono::steady_clock::now();
    Eigen::VectorXd x0 = prob.initial_solution;
    Eigen::VectorXd olb(orig_n), oub(orig_n);
    for (int i = 0; i < orig_n; ++i) {
      olb[i] = base_lp.vars[i].lb;
      oub[i] = base_lp.vars[i].ub;
    }
    x0 = project_integer_solution(base_lp.vars, x0, olb, oub);

    // Eliminate fixed variables (integers + fixed continuous), build reduced LP
    std::vector<int> free_cols, fixed_cols;
    for (int i = 0; i < orig_n; ++i) {
      if (is_integer_type(base_lp.vars[i]) || olb[i] >= oub[i] - 1e-12)
        fixed_cols.push_back(i);
      else
        free_cols.push_back(i);
    }
    const int n_free = static_cast<int>(free_cols.size());
    static constexpr int kMaxDispatchFreeVars = 8000;

    Eigen::VectorXd x0_checked = clamp_to_bounds(x0, olb, oub);
    if (satisfies_with_bounds(base_lp, x0_checked, olb, oub, 1e-6)) {
      heuristic_obj = objective_value(base_lp.c, x0_checked, base_lp.sense);
      heuristic_x_orig = std::move(x0_checked);
      if (opt.verbose) {
        auto dt = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_heur0).count();
        fmt::print(stderr,
                   "[B&C] heuristic accepted directly: obj={:.2f} ({:.1f}ms)\n",
                   heuristic_obj, dt);
      }
    } else if (n_free > kMaxDispatchFreeVars) {
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C] heuristic dispatch LP skipped: nfree={} > {}\n",
                   n_free, kMaxDispatchFreeVars);
      }
    } else if (n_free > 0) {
      Eigen::VectorXd b_adj = base_lp.b;
      Eigen::VectorXd beq_adj = base_lp.beq;
      for (int j : fixed_cols) {
        if (std::abs(x0[j]) < 1e-15) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, j); it; ++it)
          b_adj[it.row()] -= it.value() * x0[j];
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, j); it; ++it)
          beq_adj[it.row()] -= it.value() * x0[j];
      }

      std::vector<Eigen::Triplet<double>> a_trips, aeq_trips;
      for (int k = 0; k < n_free; ++k) {
        int j = free_cols[k];
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, j); it; ++it)
          a_trips.emplace_back(it.row(), k, it.value());
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, j); it; ++it)
          aeq_trips.emplace_back(it.row(), k, it.value());
      }

      LPModel dispatch_lp;
      dispatch_lp.sense = base_lp.sense;
      dispatch_lp.vars.resize(n_free);
      dispatch_lp.c = Eigen::VectorXd::Zero(n_free);
      for (int k = 0; k < n_free; ++k) {
        int j = free_cols[k];
        dispatch_lp.vars[k] = base_lp.vars[j];
        dispatch_lp.vars[k].type = VarType::Continuous;
        dispatch_lp.c[k] = base_lp.c[j];
      }
      Eigen::SparseMatrix<double> Ar(static_cast<int>(base_lp.A.rows()), n_free);
      Ar.setFromTriplets(a_trips.begin(), a_trips.end());
      dispatch_lp.A = std::move(Ar);
      dispatch_lp.b = b_adj;
      Eigen::SparseMatrix<double> Aeqr(static_cast<int>(base_lp.Aeq.rows()), n_free);
      Aeqr.setFromTriplets(aeq_trips.begin(), aeq_trips.end());
      dispatch_lp.Aeq = std::move(Aeqr);
      dispatch_lp.beq = beq_adj;

      Eigen::VectorXd x0_free(n_free);
      for (int k = 0; k < n_free; ++k) x0_free[k] = x0[free_cols[k]];

      IPMLPOptions disp_opt;
      disp_opt.max_iter = 50;  // enough for feasible LPs, fast fail on infeasible
      disp_opt.tol_primal = 1e-6;
      disp_opt.tol_dual = 1e-6;
      disp_opt.tol_gap = 1e-6;
      disp_opt.time_limit_sec = std::max(0.001, bc_remaining_sec());
      disp_opt.verbose = false;
      NativeIPMLPAdapter disp_ipm(disp_opt);
      auto disp_res = disp_ipm.solve_lp(dispatch_lp, x0_free);
      out.bc_stats.lp_solves += 1;

      if (disp_res.stats.success && disp_res.x.size() >= n_free) {
        Eigen::VectorXd xr = x0;
        for (int k = 0; k < n_free; ++k)
          xr[free_cols[k]] = disp_res.x[k];
        xr = clamp_to_bounds(xr, olb, oub);
        for (int j : fixed_cols) xr[j] = x0[j];
        if (satisfies_with_bounds(base_lp, xr, olb, oub, 1e-6)) {
          heuristic_obj = objective_value(base_lp.c, xr, base_lp.sense);
          heuristic_x_orig = xr;
          if (opt.verbose) {
            auto dt = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_heur0).count();
            fmt::print(stderr, "[B&C] heuristic verified: obj={:.2f} ({:.1f}ms)\n",
                       heuristic_obj, dt);
          }
        }
      } else if (opt.verbose) {
        fmt::print(stderr, "[B&C] heuristic dispatch LP {}\n",
                   disp_res.stats.success ? "clamped infeasible" : "infeasible");
      }
    }
  } else if (warmstart_provided && opt.verbose &&
             (strict_highs_lp_contract || skip_huge_heuristic_verification)) {
    fmt::print(stderr,
               "[B&C] heuristic verification skipped (strict_highs_lp={} n={})\n",
               strict_highs_lp_contract ? 1 : 0, orig_n);
  }

  // Best dual bound known at publish time (minimize convention).  NaN until
  // the root relaxation produces one.  Note: after PaPILO presolve the bound
  // lives in the reduced space while heuristic_obj is original-space; a
  // presolve objective offset can therefore shift the comparison, which is
  // acceptable for the 1e6-relative quality gate and for honest (if slightly
  // approximate) gap reporting on failure paths.
  double best_known_root_bound = std::numeric_limits<double>::quiet_NaN();

  auto publish_verified_warm_start_time_limit = [&]() {
    out.stats.status = bc_status::kTimeLimitReached;
    out.bc_stats.status = out.stats.status;
    out.stats.runtime_sec = bc_elapsed_sec();
    out.bc_stats.runtime_sec = out.stats.runtime_sec;
    out.stats.solver_name = "NativeBranchAndCut";
    // The dual bound stands on its own certificate; publish it even when no
    // incumbent is publishable (or the quality gate below rejects one), so a
    // timed-out run reports a real bound instead of -inf.
    if (std::isfinite(best_known_root_bound)) {
      out.bc_stats.best_bound = best_known_root_bound;
    }
    if (heuristic_x_orig.size() == orig_n && std::isfinite(heuristic_obj)) {
      if (!incumbent_quality_acceptable(heuristic_obj, best_known_root_bound)) {
        if (opt.verbose) {
          fmt::print(stderr,
                     "[B&C] verified warm start rejected by quality gate: "
                     "obj={:.6g} vs bound={:.6g}\n",
                     heuristic_obj, best_known_root_bound);
        }
        return;
      }
      if (opt.verbose && !std::isfinite(best_known_root_bound) &&
          std::abs(heuristic_obj) > 1e10) {
        fmt::print(stderr,
                   "[B&C] warning: publishing verified warm-start incumbent "
                   "with large objective {:.6g} and no dual bound; solution "
                   "quality is unknown\n",
                   heuristic_obj);
      }
      out.x = heuristic_x_orig;
      out.stats.success = true;
      // SolveStats is public and always reports the model's original objective
      // direction. heuristic_obj uses the internal minimization convention.
      out.stats.objective = prob.linear_part.c.dot(out.x);
      out.stats.primal_feas = 0.0;
      out.stats.residual_inf = 0.0;
      out.bc_stats.best_obj = heuristic_obj;
      if (std::isfinite(best_known_root_bound)) {
        out.bc_stats.best_bound = best_known_root_bound;
        out.bc_stats.gap = std::max(
            0.0, (heuristic_obj - best_known_root_bound) /
                     std::max(1.0, std::abs(heuristic_obj)));
      }
      out.bc_stats.incumbent_updates = std::max<std::uint64_t>(
          out.bc_stats.incumbent_updates, 1);
      out.bc_stats.first_incumbent_node = 0;
      out.bc_stats.last_incumbent_node = 0;
      out.bc_stats.last_incumbent_depth = 0;
    }
  };

  if (bc_time_limit_expired()) {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }

  std::vector<NativeVariableBoundSourceEntry> original_presolve_vb_sources;
  if (!strict_highs_root_fixed_point) {
    LPModel original_source_lp = base_lp;
    std::vector<char> original_implied_integer_cols =
        detect_implied_integer_columns(original_source_lp);
    tighten_integral_row_sides(original_source_lp, original_implied_integer_cols,
                               opt.int_tol);
    VariableBoundTable original_vb_table(
        static_cast<int>(original_source_lp.vars.size()));
    build_variable_bound_table_from_rows(original_source_lp,
                                         original_implied_integer_cols,
                                         original_vb_table, nullptr, 512);
    original_presolve_vb_sources =
        collect_variable_bound_sources(original_vb_table);
    if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) &&
        !original_presolve_vb_sources.empty()) {
      fmt::print(stderr,
                 "[B&C-PRESOLVE-VB-SIDE] original_sources={} vars={} rows={}\n",
                 original_presolve_vb_sources.size(),
                 static_cast<int>(original_source_lp.vars.size()),
                 static_cast<int>(original_source_lp.A.rows()) +
                     static_cast<int>(original_source_lp.Aeq.rows()));
    }
  }

  // ── PaPILO presolve ──
  PaPILOPresolveResult papilo_ps;
  // True iff the working model below is the PaPILO-reduced one; gates the
  // unpresolved-root fallback retry (root_presolve_fallback).
  bool papilo_model_reduced = false;
#ifdef MIPSOLVERS_HAVE_PAPILO
  if (opt.use_papilo_presolve && !strict_highs_root_fixed_point) {
    papilo_ps = papilo_presolve_mip(base_lp, opt.verbose,
                                    opt.papilo_aggressive
                                        ? PaPILOProfile::kAggressive
                                        : PaPILOProfile::kDefault);
    // Auto-escalate: if the default run found bound or coefficient changes,
    // the instance still has presolvable structure. Re-run with aggressive
    // profile to extract tighter reductions (deeper substitution, tighter
    // abort factors). Applies only when caller did not already request aggressive.
    if (papilo_ps.success && !papilo_ps.infeasible && !opt.papilo_aggressive &&
        (papilo_ps.total_bnd_chg > 0 || papilo_ps.total_coef_chg > 0)) {
      if (opt.verbose)
        fprintf(stderr,
                "[PAPILO-AUTO] default run: bnd_chg=%d coef_chg=%d — escalating to aggressive\n",
                papilo_ps.total_bnd_chg, papilo_ps.total_coef_chg);
      papilo_ps = papilo_presolve_mip(base_lp, opt.verbose, PaPILOProfile::kAggressive);
      // The auto-escalated aggressive result is what the
      // solver actually uses below — attribute its conditioning explicitly so
      // hardening can be traced to this swap rather than the default run.
      if ((opt.verbose ||
           bc_env_flag_enabled("MIPSOLVERS_PRESOLVE_SCALING_DIAG")) &&
          papilo_ps.success && papilo_ps.reduced_cols > 0) {
        const auto& as = papilo_ps.after_stats;
        fprintf(stderr,
                "[PRESOLVE-SCALING] stage=aggressive-rerun-swap rows=%d "
                "cols=%d nnz=%lld dyn=%.3e rows_range_gt_1e8=%d "
                "worst_range=%.3e\n",
                as.rows, as.cols, as.nnz, as.dynamic_range,
                as.rows_with_large_range, as.worst_row_range);
      }
    }
    if (papilo_ps.success && !papilo_ps.infeasible &&
        papilo_ps.reduced_cols > 0) {
      base_lp = std::move(papilo_ps.reduced_lp);
      papilo_model_reduced = true;
    } else if (papilo_ps.infeasible) {
      out.stats.status = bc_status::kInfeasiblePapiloPresolve;
      return return_with_profile();
    }
  }
#endif

  // ── Native MILPPresolve top-level presolve (default ON; disable with
  //    MIPSOLVERS_NATIVE_TOPLEVEL_PRESOLVE_OFF) ──
  // Reduces base_lp so every node/root LP solve AND the objective-propagation
  // machinery operate on the smaller system; the incumbent is postsolved back to
  // original space before the objective recompute, and uc_hint / branching_
  // priority / initial_solution are remapped through the presolve column map.
  // MILPPresolve runs its full validated-sound reduction set (native_kernel_
  // comparison --check keeps objectives exact).
  PresolveOptions native_presolve_options;
  native_presolve_options.do_probing = opt.native_presolve_probing;
  MILPPresolve native_ps(native_presolve_options);
  bool native_model_reduced = false;
  if (!strict_highs_root_fixed_point && !papilo_model_reduced &&
      !bc_env_flag_enabled("MIPSOLVERS_NATIVE_TOPLEVEL_PRESOLVE_OFF")) {
    out.bc_stats.native_presolve_attempted = true;
    std::vector<int> nps_bin, nps_int;
    // run() rewrites the LPModel in place, so presolve on a copy and only adopt
    // the reduced model when it still has columns to branch on.  When presolve
    // fully solves the problem (final_cols == 0) the reduced model is empty and
    // the B&C cannot operate on it; fall back to the untouched base_lp (all
    // postsolve/mapping hooks are gated on native_model_reduced, so leaving it
    // false keeps the original model authoritative and correct).
    LPModel presolved_lp = base_lp;
    const PresolveStats nps_stats = native_ps.run(presolved_lp, nps_bin, nps_int);
    out.bc_stats.native_presolve_orig_rows = nps_stats.orig_rows;
    out.bc_stats.native_presolve_final_rows = nps_stats.final_rows;
    out.bc_stats.native_presolve_orig_cols = nps_stats.orig_cols;
    out.bc_stats.native_presolve_final_cols = nps_stats.final_cols;
    out.bc_stats.native_presolve_time_ms = 1000.0 * nps_stats.presolve_time_sec;
    out.bc_stats.native_presolve_probing_trail_pushes =
        nps_stats.probing_trail_pushes;
    out.bc_stats.native_presolve_probing_rows_processed =
        nps_stats.probing_rows_processed;
    out.bc_stats.native_presolve_probing_implications_learned =
        nps_stats.probing_implications_learned;
    out.bc_stats.native_presolve_probing_max_touched_cols =
        nps_stats.probing_max_touched_cols;
    out.bc_stats.native_presolve_probing_truncated =
        nps_stats.probing_truncated;
    if (nps_stats.infeasible) {
      out.stats.status = bc_status::kInfeasibleNativePresolve;
      if (opt.verbose) {
        fmt::print(stderr, "[NATIVE-PRESOLVE] infeasible: {}\n",
                   nps_stats.infeasibility_reason);
      }
      return return_with_profile();
    }
    const bool native_presolve_changed_model =
        nps_stats.cols_removed > 0 || nps_stats.rows_removed > 0 ||
        nps_stats.bounds_tightened > 0 ||
        !native_ps.probing_implications().empty();
    if (nps_stats.final_cols > 0 && native_presolve_changed_model) {
      base_lp = std::move(presolved_lp);
      native_model_reduced = true;
      out.bc_stats.native_presolve_adopted = true;
      if (opt.verbose || bc_env_flag_enabled("MIPSOLVERS_PRESOLVE_VERBOSE"))
        fmt::print(stderr,
                   "[NATIVE-PRESOLVE] rows {}->{} cols {}->{} nnz {}->{} "
                   "{:.1f}ms\n",
                   nps_stats.orig_rows, nps_stats.final_rows,
                   nps_stats.orig_cols, nps_stats.final_cols,
                   nps_stats.orig_nnz, nps_stats.final_nnz,
                   nps_stats.presolve_time_sec * 1000.0);
    }
  }

  if (bc_time_limit_expired()) {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }

  HiGHSPresolvedModelStats highs_presolve_side_state;
  bool strict_highs_presolved_working_space = false;
  double strict_highs_presolved_objective_offset = 0.0;
  const bool highs_presolve_vb_source_disabled =
      bc_env_flag_enabled("MIPSOLVERS_DISABLE_HIGHS_PRESOLVE_VB_SOURCE");
  const bool highs_presolve_vb_source_requested =
      !highs_presolve_vb_source_disabled ||
      strict_highs_root_fixed_point ||
      bc_env_flag_enabled("MIPSOLVERS_REPLAY_HIGHS_PRESOLVE_VB") ||
      bc_env_flag_enabled("MIPSOLVERS_HIGHS_PRESOLVE_STATS");
  if (highs_presolve_vb_source_requested) {
    bool highs_presolve_cache_hit = false;
    highs_presolve_side_state =
        cached_highs_presolve_side_state(
            base_lp, std::max(0.001, bc_remaining_sec()),
            &highs_presolve_cache_hit);
    if (strict_highs_root_fixed_point) {
      if (highs_presolve_side_state.presolve_ok &&
          highs_presolve_side_state.highs_status == "reduced" &&
          highs_presolve_side_state.presolved_lp_available &&
          !highs_presolve_side_state.presolved_lp.vars.empty()) {
        base_lp = highs_presolve_side_state.presolved_lp;
        strict_highs_presolved_working_space = true;
        strict_highs_presolved_objective_offset =
            highs_presolve_side_state.presolved_objective_offset;
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
            bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
          fmt::print(stderr,
                     "[B&C-HIGHS-PRESOLVED-WORKING-LP] enabled=1 "
                     "cache={} status={} rows={} cols={} nnz={} "
                     "native_ineq={} native_eq={} native_nnz={} "
                     "offset={:.17g}\n",
                     highs_presolve_cache_hit ? "hit" : "miss",
                     highs_presolve_side_state.highs_status,
                     highs_presolve_side_state.rows,
                     highs_presolve_side_state.cols,
                     highs_presolve_side_state.nnz,
                     static_cast<int>(base_lp.A.rows()),
                     static_cast<int>(base_lp.Aeq.rows()),
                     static_cast<int>(base_lp.A.nonZeros()) +
                         static_cast<int>(base_lp.Aeq.nonZeros()),
                     strict_highs_presolved_objective_offset);
        }
      } else {
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
            bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
          fmt::print(stderr,
                     "[B&C-HIGHS-PRESOLVED-WORKING-LP] enabled=0 "
                     "cache={} available={} pass={} presolve={} "
                     "lp={} status={} rows={} cols={} nnz={}\n",
                     highs_presolve_cache_hit ? "hit" : "miss",
                     highs_presolve_side_state.available ? 1 : 0,
                     highs_presolve_side_state.pass_ok ? 1 : 0,
                     highs_presolve_side_state.presolve_ok ? 1 : 0,
                     highs_presolve_side_state.presolved_lp_available ? 1 : 0,
                     highs_presolve_side_state.highs_status,
                     highs_presolve_side_state.rows,
                     highs_presolve_side_state.cols,
                     highs_presolve_side_state.nnz);
        }
      }
    }
    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
        bc_env_options().value("MIPSOLVERS_HIGHS_PRESOLVE_STATS") != nullptr) {
      fmt::print(stderr,
                 "[B&C-HIGHS-VB-SIDE] available={} pass={} presolve={} "
                 "side={} cache={} status={} rows={} cols={} nnz={} ranged={} "
                 "bin={} impl={} cont={} fixed={} vub={}:0x{:016x} "
                 "vlb={}:0x{:016x} records={} lp={} work={} "
                 "stats=vub{}/{}/{}:vlb{}/{}/{}:probe{}/{}/{}/{}\n",
                 highs_presolve_side_state.available ? 1 : 0,
                 highs_presolve_side_state.pass_ok ? 1 : 0,
                 highs_presolve_side_state.presolve_ok ? 1 : 0,
                 highs_presolve_side_state.side_state_available ? 1 : 0,
                 highs_presolve_cache_hit ? "hit" : "miss",
                 highs_presolve_side_state.highs_status,
                 highs_presolve_side_state.rows, highs_presolve_side_state.cols,
                 highs_presolve_side_state.nnz,
                 highs_presolve_side_state.ranged_rows,
                 highs_presolve_side_state.binary_cols,
                 highs_presolve_side_state.implied_integer_cols,
                 highs_presolve_side_state.continuous_cols,
                 highs_presolve_side_state.fixed_cols,
                 highs_presolve_side_state.vub_count,
                 highs_presolve_side_state.vub_hash,
                 highs_presolve_side_state.vlb_count,
                 highs_presolve_side_state.vlb_hash,
                 highs_presolve_side_state.var_bounds.size(),
                 highs_presolve_side_state.presolved_lp_available ? 1 : 0,
                 strict_highs_presolved_working_space ? 1 : 0,
                 highs_presolve_side_state.vub_accepted,
                 highs_presolve_side_state.vub_attempts,
                 highs_presolve_side_state.vub_replaced,
                 highs_presolve_side_state.vlb_accepted,
                 highs_presolve_side_state.vlb_attempts,
                 highs_presolve_side_state.vlb_replaced,
                 highs_presolve_side_state.probing_calls,
                 highs_presolve_side_state.probing_conflicts,
                 highs_presolve_side_state.probing_reductions,
                 highs_presolve_side_state.probing_substitutions);
    }
  }
  if (bc_time_limit_expired() ||
      highs_presolve_side_state.highs_status == "timeout") {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }
  if (strict_highs_root_fixed_point && !strict_highs_presolved_working_space) {
    out.stats.status = bc_status::kHighsPresolvedWorkingLpUnavailable;
    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
        bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
      fmt::print(stderr,
                 "[B&C-HIGHS-PRESOLVED-WORKING-LP] fatal=1 "
                 "reason=required_for_strict_root status={} rows={} cols={} "
                 "nnz={} lp={}\n",
                 highs_presolve_side_state.highs_status,
                 highs_presolve_side_state.rows,
                 highs_presolve_side_state.cols,
                 highs_presolve_side_state.nnz,
                 highs_presolve_side_state.presolved_lp_available ? 1 : 0);
    }
    return return_with_profile();
  }

  const bool use_huge_warmstart_root_shortcut =
      !opt.use_ipm_nodes && warmstart_provided &&
      static_cast<int>(base_lp.vars.size()) >= 90000;

  if (use_huge_warmstart_root_shortcut && opt.verbose) {
    fprintf(stderr,
            "[PAPILO-PRESOLVE] warm-start shortcut active (rows=%d cols=%d)\n",
            static_cast<int>(base_lp.A.rows()) + static_cast<int>(base_lp.Aeq.rows()),
            static_cast<int>(base_lp.vars.size()));
  }

  const int n = static_cast<int>(base_lp.vars.size());
  std::vector<char> declared_branchable_cols(static_cast<std::size_t>(n), 0);
  std::vector<char> declared_binary_cols(static_cast<std::size_t>(n), 0);
  if (strict_highs_presolved_working_space &&
      highs_presolve_side_state.side_state_available &&
      static_cast<int>(highs_presolve_side_state.presolved_col_type.size()) >=
          n) {
    for (int j = 0; j < n; ++j) {
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
      const auto type = static_cast<HighsVarType>(
          highs_presolve_side_state.presolved_col_type[
              static_cast<std::size_t>(j)]);
      if (type != HighsVarType::kContinuous &&
          type != HighsVarType::kSemiContinuous) {
        declared_branchable_cols[static_cast<std::size_t>(j)] = 1;
        if (type == HighsVarType::kInteger &&
            std::abs(base_lp.vars[static_cast<std::size_t>(j)].lb) <=
                highs_mip_feastol &&
            std::abs(base_lp.vars[static_cast<std::size_t>(j)].ub - 1.0) <=
                highs_mip_feastol) {
          declared_binary_cols[static_cast<std::size_t>(j)] = 1;
        }
      }
#endif
    }
  } else {
    for (int j = 0; j < n; ++j) {
      int orig_j = j;
#ifdef MIPSOLVERS_HAVE_PAPILO
      if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
        if (j >= static_cast<int>(papilo_ps.reduced_to_orig_col.size())) {
          continue;
        }
        orig_j = papilo_ps.reduced_to_orig_col[static_cast<std::size_t>(j)];
      }
#endif
      if (native_model_reduced) {
        const auto& r2o = native_ps.reduced_to_orig_col();
        if (j >= static_cast<int>(r2o.size())) continue;
        orig_j = r2o[static_cast<std::size_t>(j)];
      }
      if (orig_j >= 0 && orig_j < orig_n &&
          original_declared_integer_cols[static_cast<std::size_t>(orig_j)] !=
              0) {
        declared_branchable_cols[static_cast<std::size_t>(j)] = 1;
        if (original_declared_binary_cols[static_cast<std::size_t>(orig_j)] !=
            0) {
          declared_binary_cols[static_cast<std::size_t>(j)] = 1;
        }
      }
    }
  }
  std::vector<int> declared_branchable_indices;
  declared_branchable_indices.reserve(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    if (declared_branchable_cols[static_cast<std::size_t>(j)] != 0) {
      declared_branchable_indices.push_back(j);
    }
  }
  auto fractional_branchable_indices =
      [&](const Eigen::VectorXd& x, const Eigen::VectorXd& lb,
          const Eigen::VectorXd& ub, double int_tol,
          std::vector<int>& out) -> bool {
    out.clear();
    const int limit =
        std::min(n, static_cast<int>(std::min<std::size_t>(
                        declared_branchable_cols.size(),
                        static_cast<std::size_t>(std::min(
                            static_cast<int>(x.size()),
                            std::min(static_cast<int>(lb.size()),
                                     static_cast<int>(ub.size())))))));
    const double split_tol = std::max(1e-9, int_tol);
    for (int j : declared_branchable_indices) {
      if (j >= limit) break;
      if (ub[j] - lb[j] <= split_tol) {
        continue;
      }
      const double v = std::min(ub[j], std::max(lb[j], x[j]));
      if (is_integral(v, int_tol)) {
        continue;
      }
      const double down_ub = std::min(ub[j], std::floor(v));
      const double up_lb = std::max(lb[j], std::ceil(v));
      if (down_ub < lb[j] - split_tol || up_lb > ub[j] + split_tol) {
        continue;
      }
      out.push_back(j);
    }
    return !out.empty();
  };
  auto refresh_node_fractional_branchables =
      [&](Node& node, double int_tol) -> bool {
    node.fractional_branchables_valid = true;
    node.fractional_branchables_tol = int_tol;
    return fractional_branchable_indices(
        node.x_relax, node.lb, node.ub, int_tol,
        node.fractional_branchables);
  };
  auto node_fractional_branchable_indices =
      [&](Node& node, double int_tol, std::vector<int>& out) -> bool {
    const double tol_scale =
        std::max({1.0, std::abs(int_tol),
                  std::abs(node.fractional_branchables_tol)});
    const bool cache_matches =
        node.fractional_branchables_valid && !node.lp_refresh_needed &&
        std::abs(node.fractional_branchables_tol - int_tol) <=
            1e-15 * tol_scale;
    if (!cache_matches) {
      refresh_node_fractional_branchables(node, int_tol);
    }
    out = node.fractional_branchables;
    return !out.empty();
  };
  auto round_declared_branchables = [&](Eigen::VectorXd& x,
                                        const Eigen::VectorXd& lb,
                                        const Eigen::VectorXd& ub) {
    const int limit =
        std::min(n, static_cast<int>(std::min<std::size_t>(
                        declared_branchable_cols.size(),
                        static_cast<std::size_t>(x.size()))));
    for (int j : declared_branchable_indices) {
      if (j >= limit) break;
      x[j] = std::min(ub[j], std::max(lb[j], std::round(x[j])));
    }
  };
  if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_TIMELINE") != nullptr) {
    const int model_integer_cols = static_cast<int>(std::count_if(
        base_lp.vars.begin(), base_lp.vars.end(),
        [](const VariableMeta& v) { return is_integer_type(v); }));
    const int declared_branchable_count = static_cast<int>(std::count(
        declared_branchable_cols.begin(), declared_branchable_cols.end(), 1));
    const int declared_binary_count = static_cast<int>(
        std::count(declared_binary_cols.begin(), declared_binary_cols.end(), 1));
    const char* presolve_tag =
        strict_highs_presolved_working_space ? " presolve=highs" :
#ifdef MIPSOLVERS_HAVE_PAPILO
        (papilo_ps.success ? " presolve=papilo" : "");
    const char* mapped_tag =
        papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()
            ? " mapped=1"
            : " mapped=0";
#else
        "";
    const char* mapped_tag = "";
#endif
    fmt::print(stderr,
               "[B&C-INTEGRALITY] model_int={} declared_branchable={} "
               "declared_binary={} cols={}{}{}\n",
               model_integer_cols, declared_branchable_count,
               declared_binary_count, n, presolve_tag, mapped_tag);
  }
  if (bc_time_limit_expired()) {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }
  // Historically, large models (n>10000) with a short time budget (<=10s)
  // short-circuited to the verified warm start instead of running the tree.
  // That warm start can be a high-penalty value (e.g. IEEE-118 returns ~2.1e11
  // with bound=-inf and nodes=0), so bailing produces a useless result.  With
  // top-level presolve + the direct-HiGHS root LP the root now solves fast
  // enough to make real tree progress on a short budget, so this shortcut is
  // OFF by default (re-enable with MIPSOLVERS_BC_SHORT_LIMIT_WARMSTART_BAIL).
  if (!strict_highs_lp_contract && opt.time_limit_sec <= 10.0 && n > 10000 &&
      heuristic_x_orig.size() == orig_n && std::isfinite(heuristic_obj) &&
      bc_env_flag_enabled("MIPSOLVERS_BC_SHORT_LIMIT_WARMSTART_BAIL")) {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }
  // ── Auto-tune IPM for large (post-presolve) problems ──
  // Use presolved dimensions for a more accurate threshold. Presolve can
  // dramatically reduce row count (e.g., 10320→4709 for IEEE-118).
  const int presolved_m = static_cast<int>(base_lp.A.rows()) +
                          static_cast<int>(base_lp.Aeq.rows());
  const bool allow_auto_native_ipm_root =
      strict_highs_lp_contract || opt.time_limit_sec > 10.0;
  if (allow_auto_native_ipm_root && presolved_m > opt.ipm_auto_threshold &&
      !opt.use_ipm_root) {
    opt.use_ipm_root = true;
  }
  if (allow_auto_native_ipm_root && presolved_m > opt.ipm_auto_threshold &&
      !opt.use_ipm_nodes && !opt.use_simplex_lp_nodes) {
    opt.use_ipm_nodes = true;
  }

  // In IPM-only mode on very large models, Gomory cuts need a simplex basis
  // which we won't have. Auto-switch to All cuts so MIR/cover/clique can fire.
  if (presolved_m > opt.xlarge_ipm_only_threshold && opt.cuts == CutType::Gomory) {
    opt.cuts = CutType::All;
  }

  const bool large_generic_presolved_root =
      !domain_heuristics &&
      (presolved_m > std::min(opt.large_problem_threshold, 8000)) &&
      (n > 5000);
  if (!strict_highs_lp_contract && opt.auto_highs_root_pipeline &&
      large_generic_presolved_root) {
    opt.use_lock_count_rounding = true;
    opt.use_linesearch_rounding = true;
    opt.use_analytic_centre = true;
    opt.analytic_centre_max_iter = std::min(opt.analytic_centre_max_iter, 100);
    // The root sub-MIP reuses the same LP numerical oracle as node solves.
    // Keep this as a first-class primal source before the MIP-gap proof gate
    // is allowed to drain the frontier. Originally vendored-HiGHS-only; the
    // native dual simplex now converges on large degenerate SCUC roots (the
    // BFRT exact-cover tolerance fixes), so both kernels qualify.
    opt.enable_root_low_fractionality_rens = true;
  }

  // Auto-tune crossover-skip threshold for UC-like problems.
  // UC (≥1000 binaries, ≥50% integer ratio) at 25k+ rows spends most root
  // time in the IPM-to-simplex push phase. The shape gate selects IPM-only for
  // these cases; it does not assert an end-to-end timing result.
  {
    int n_bin_total = 0;
    for (int j = 0; j < n; ++j) {
      if (is_integer_type(base_lp.vars[j]) && base_lp.vars[j].lb >= -0.5 &&
          base_lp.vars[j].ub <= 1.5) {
        ++n_bin_total;
      }
    }
    const bool big_uc_like = (n_bin_total >= 1000) &&
                             (2 * n_bin_total > n) &&
                             (presolved_m >= 25000);
    if (big_uc_like && opt.xlarge_ipm_only_threshold > 20000) {
      opt.xlarge_ipm_only_threshold = 20000;  // force IPM-only path
    }
    if (opt.verbose) {
      fprintf(stderr, "[B&C-AUTOTUNE] big_uc_like=%d n_bin=%d n=%d presolved_m=%d xlarge_thr=%d\n",
              big_uc_like ? 1 : 0, n_bin_total, n, presolved_m,
              opt.xlarge_ipm_only_threshold);
    }
  }

  // Determine the reduced-space column mapping: original → PaPILO-reduced.
  auto map_original_col_to_reduced = [&](int orig_j) -> int {
    int j = orig_j;
#ifdef MIPSOLVERS_HAVE_PAPILO
    if (papilo_ps.success && !papilo_ps.orig_to_reduced_col.empty()) {
      if (j < 0 || j >= static_cast<int>(papilo_ps.orig_to_reduced_col.size()))
        return -1;
      j = papilo_ps.orig_to_reduced_col[j];
      if (j < 0) return -1;
    }
#endif
    if (native_model_reduced) {
      const auto& o2r = native_ps.orig_to_reduced_col();
      if (j < 0 || j >= static_cast<int>(o2r.size())) return -1;
      j = o2r[static_cast<std::size_t>(j)];
      if (j < 0) return -1;
    }
    return j;
  };

  Eigen::VectorXd reduced_initial_solution;
  bool reduced_warmstart_available = false;
  if (warmstart_provided) {
    Eigen::VectorXd mapped = prob.initial_solution;
#ifdef MIPSOLVERS_HAVE_PAPILO
    if (papilo_ps.success && !papilo_ps.orig_to_reduced_col.empty()) {
      mapped = papilo_forward_map(papilo_ps, mapped);
    }
#endif
    if (native_model_reduced) {
      mapped = native_ps.forward_map(mapped);
    }
    if (mapped.size() == n) {
      reduced_initial_solution = std::move(mapped);
      reduced_warmstart_available = true;
    }
  }

  // ── Build reduced-space UC hint (remap column indices through PaPILO) ──
  // prob.uc_hint has indices in original space. We map through PaPILO to get
  // reduced-space indices.
  std::optional<MIPModel::UCGenHint> reduced_uc_hint;
  if (domain_heuristics && prob.uc_hint.has_value()) {
    const auto& uc = *prob.uc_hint;
    const int block_size = uc.ng * uc.T;

    // Check if all IG (commitment binary) columns survive presolve.
    // SU/SD/PG columns may be eliminated — they get -1 in per-column maps.
    bool ig_survive = true;
    const bool has_orig_ig_cols = !uc.ig_cols.empty();
    for (int k = 0; k < block_size && ig_survive; ++k) {
      int orig_col = has_orig_ig_cols ? uc.ig_cols[static_cast<size_t>(k)] : (uc.ig_start + k);
      if (orig_col < 0 || map_original_col_to_reduced(orig_col) < 0) ig_survive = false;
    }

    if (ig_survive && block_size > 0) {
      MIPModel::UCGenHint ruc;
      ruc.ng = uc.ng;
      ruc.T = uc.T;
      ruc.ig_start = map_original_col_to_reduced(uc.ig_start);
      ruc.su_start = (map_original_col_to_reduced(uc.su_start) >= 0)
                         ? map_original_col_to_reduced(uc.su_start)
                         : -1;
      ruc.sd_start = (map_original_col_to_reduced(uc.sd_start) >= 0)
                         ? map_original_col_to_reduced(uc.sd_start)
                         : -1;
      if (uc.pg_start >= 0) ruc.pg_start = map_original_col_to_reduced(uc.pg_start);
      ruc.min_up = uc.min_up;
      ruc.min_down = uc.min_down;
      ruc.ig0 = uc.ig0;
      ruc.pmin = uc.pmin;
      ruc.pmax = uc.pmax;
      ruc.ramp = uc.ramp;
      ruc.certifies_power_balance_rows = uc.certifies_power_balance_rows;
      ruc.certifies_generation_capacity_rows = uc.certifies_generation_capacity_rows;
      ruc.certifies_ramping_rows = uc.certifies_ramping_rows;
      ruc.certifies_min_up_down_rows = uc.certifies_min_up_down_rows;
      ruc.certifies_segment_bound_rows = uc.certifies_segment_bound_rows;
      ruc.certifies_system_reserve_rows = uc.certifies_system_reserve_rows;
      ruc.certifies_area_import_rows = uc.certifies_area_import_rows;
      ruc.certifies_hard_network_flow_rows = uc.certifies_hard_network_flow_rows;
      ruc.certifies_hard_section_flow_rows = uc.certifies_hard_section_flow_rows;
      ruc.certifies_storage_cycle_rows = uc.certifies_storage_cycle_rows;
      // Build per-element column maps (eliminated cols get -1)
      ruc.ig_cols.resize(static_cast<size_t>(block_size));
      ruc.su_cols.resize(static_cast<size_t>(block_size), -1);
      ruc.sd_cols.resize(static_cast<size_t>(block_size), -1);
      ruc.pg_cols.resize(static_cast<size_t>(block_size), -1);
      int ig_ok = 0, su_ok = 0, sd_ok = 0, pg_ok = 0;
      const bool has_orig_ig_cols = !uc.ig_cols.empty();
      const bool has_orig_su_cols = !uc.su_cols.empty();
      const bool has_orig_sd_cols = !uc.sd_cols.empty();
      const bool has_orig_pg_cols = !uc.pg_cols.empty();
      for (int k = 0; k < block_size; ++k) {
        // Use per-element source cols if provided, else ig_start+k (contiguous block)
        int orig_ig = has_orig_ig_cols ? uc.ig_cols[static_cast<size_t>(k)] : (uc.ig_start + k);
        int orig_su = has_orig_su_cols ? uc.su_cols[static_cast<size_t>(k)] : (uc.su_start + k);
        int orig_sd = has_orig_sd_cols ? uc.sd_cols[static_cast<size_t>(k)] : (uc.sd_start + k);
        int orig_pg = has_orig_pg_cols ? uc.pg_cols[static_cast<size_t>(k)] : (uc.pg_start >= 0 ? uc.pg_start + k : -1);
        ruc.ig_cols[static_cast<size_t>(k)] = (orig_ig >= 0) ? map_original_col_to_reduced(orig_ig) : -1;
        if (ruc.ig_cols[static_cast<size_t>(k)] >= 0) ++ig_ok;
        ruc.su_cols[static_cast<size_t>(k)] = (orig_su >= 0) ? map_original_col_to_reduced(orig_su) : -1;
        if (ruc.su_cols[static_cast<size_t>(k)] >= 0) ++su_ok;
        ruc.sd_cols[static_cast<size_t>(k)] = (orig_sd >= 0) ? map_original_col_to_reduced(orig_sd) : -1;
        if (ruc.sd_cols[static_cast<size_t>(k)] >= 0) ++sd_ok;
        ruc.pg_cols[static_cast<size_t>(k)] = (orig_pg >= 0) ? map_original_col_to_reduced(orig_pg) : -1;
        if (ruc.pg_cols[static_cast<size_t>(k)] >= 0) ++pg_ok;
      }
      reduced_uc_hint = std::move(ruc);
      if (opt.verbose)
        fprintf(stderr, "[UC-HINT] reduced: ng=%d T=%d ig=%d/%d su=%d/%d sd=%d/%d pg=%d/%d\n",
                reduced_uc_hint->ng, reduced_uc_hint->T,
                ig_ok, block_size, su_ok, block_size, sd_ok, block_size, pg_ok, block_size);
    } else if (opt.verbose) {
      fprintf(stderr, "[UC-HINT] ig_survive=false, no reduced hint\n");
    }
  }

  // ── Build final working-space to original-space branching metadata ──
  std::vector<int> branch_original_cols(static_cast<std::size_t>(n), -1);
  for (int j = 0; j < n; ++j) {
    int original = j;
    if (strict_highs_presolved_working_space) {
      if (original < 0 ||
          original >= static_cast<int>(
                          highs_presolve_side_state.presolved_col_orig.size())) {
        continue;
      }
      original = highs_presolve_side_state
                     .presolved_col_orig[static_cast<std::size_t>(original)];
    }
    if (native_model_reduced) {
      const auto& reduced_to_input = native_ps.reduced_to_orig_col();
      if (original < 0 ||
          original >= static_cast<int>(reduced_to_input.size())) {
        continue;
      }
      original = reduced_to_input[static_cast<std::size_t>(original)];
    }
#ifdef MIPSOLVERS_HAVE_PAPILO
    if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
      if (original < 0 ||
          original >= static_cast<int>(papilo_ps.reduced_to_orig_col.size())) {
        continue;
      }
      original =
          papilo_ps.reduced_to_orig_col[static_cast<std::size_t>(original)];
    }
#endif
    if (original >= 0 && original < orig_n) {
      branch_original_cols[static_cast<std::size_t>(j)] = original;
    }
  }

  std::vector<int> branch_priority;
  if (!prob.branching_priority.empty()) {
    branch_priority.resize(static_cast<size_t>(n), 0);
    for (int j = 0; j < n; ++j) {
      const int original = branch_original_cols[static_cast<std::size_t>(j)];
      if (original >= 0 &&
          original < static_cast<int>(prob.branching_priority.size())) {
        branch_priority[static_cast<std::size_t>(j)] =
            prob.branching_priority[static_cast<std::size_t>(original)];
      }
    }
  }
  auto ensure_branch_priority = [&]() -> std::vector<int>& {
    if (branch_priority.size() != static_cast<std::size_t>(n)) {
      branch_priority.assign(static_cast<std::size_t>(n), 0);
    }
    return branch_priority;
  };
  const std::vector<int> parallel_branch_priority = branch_priority;

    auto build_prog_round_focus_trace = [&](const Eigen::VectorXd& x_relax,
                                            const Eigen::VectorXd& lb,
                                            const Eigen::VectorXd& ub) {
      if (domain_heuristics && reduced_uc_hint.has_value()) {
        return make_prog_round_focus_ig_trace(reduced_uc_hint, x_relax, lb, ub);
      }

      constexpr int kFocusGenerators[] = {53, 45, 13};
      constexpr int kFocusPeriods[] = {0, 1, 2, 3, 4, 23};
      std::vector<ProgRoundTraceFocusVar> out;
      if (!domain_heuristics || !prob.uc_hint.has_value()) return out;

      const auto& uc = *prob.uc_hint;
      const int n_focus = static_cast<int>(x_relax.size());
      const bool use_cols = !uc.ig_cols.empty();
      out.reserve(static_cast<size_t>(sizeof(kFocusGenerators) / sizeof(kFocusGenerators[0]) *
                                      sizeof(kFocusPeriods) / sizeof(kFocusPeriods[0])));

      for (int g : kFocusGenerators) {
        if (g < 0 || g >= uc.ng) continue;
        for (int t : kFocusPeriods) {
          if (t < 0 || t >= uc.T) continue;
          const size_t pos = static_cast<size_t>(t * uc.ng + g);
          int orig_col = -1;
          if (use_cols) {
            if (pos < uc.ig_cols.size()) orig_col = uc.ig_cols[pos];
          } else if (uc.ig_start >= 0) {
            orig_col = uc.ig_start + static_cast<int>(pos);
          }
          const int col = map_original_col_to_reduced(orig_col);

          ProgRoundTraceFocusVar focus;
          focus.g = g;
          focus.t = t;
          focus.col = col;
          focus.available = (col >= 0 && col < n_focus);
          if (focus.available) {
            focus.root_val = x_relax[col];
            focus.last_fixed = (ub[col] - lb[col] < 1e-9);
            focus.last_value = focus.last_fixed ? lb[col] : x_relax[col];
          }
          out.push_back(focus);
        }
      }
      return out;
    };

  // Probing budget: scale down for large problems where each LP solve
  // is expensive.  The dense simplex basis inverse makes each probe LP
  // O(m^2) per pivot, so probing dominates for m > 3000.  For very large
  // For very large problems (m > 10000), the sparse simplex per-LP cost is
  // high enough (~50-100ms) that probing LP overhead dominates. Skip probing
  // at non-root nodes to reduce LP count.
  // For medium problems (m 5000-10000), per-LP is modest (~5-10ms) and
  // branching quality from probing outweighs the LP cost.
  // Root always gets full probing for branching quality (applied per-node below).
  const int base_m = static_cast<int>(base_lp.A.rows()) +
                     static_cast<int>(base_lp.Aeq.rows());
  const bool huge_column_model = n >= 100000;
  const bool disable_uc_tree_probing = reduced_uc_hint.has_value() && (base_m > 2000);
  const int probe_reliability = huge_column_model ? 0
                         : (base_m > opt.large_problem_threshold) ? 0
                         : (base_m > 12000) ? 1 : opt.probe_reliability;
  const int probe_max = huge_column_model ? 0
                         : disable_uc_tree_probing ? 0
                         : (base_m > opt.large_problem_threshold) ? 0
                         : (base_m > 12000) ? 1
                         : (base_m > 2000) ? std::max(1, opt.probe_max_candidates - 1)
                         : opt.probe_max_candidates;
  const int root_probe_max = huge_column_model || base_m > 12000
                                 ? 0
                                 : opt.probe_max_candidates;

  CompactPseudoCostTable pc(n, declared_branchable_indices);

  Node root;
  root.lb = Eigen::VectorXd(n);
  root.ub = Eigen::VectorXd(n);
  for (int i = 0; i < n; ++i) {
    root.lb[i] = base_lp.vars[i].lb;
    root.ub[i] = base_lp.vars[i].ub;
  }
  root.x_seed = Eigen::VectorXd::Zero(n);
  if (reduced_warmstart_available && !opt.use_ipm_root) {
    // Simplex mode: seed root LP with initial solution for warm-start basis.
    // IPM mode: use midpoint interior point — seeding IPM with an integer
    // solution (e.g. all-generators-on UC warm-start) biases it to a nearly-
    // integral LP vertex with few fractional variables, which drastically
    // shrinks the B&C tree and prevents finding the true optimum.
    root.x_seed = clamp_to_bounds(reduced_initial_solution, root.lb, root.ub);
  } else {
    for (int i = 0; i < n; ++i) {
      root.x_seed[i] = 0.5 * (root.lb[i] + root.ub[i]);
      if (!std::isfinite(root.x_seed[i])) {
        root.x_seed[i] = 0.0;
      }
    }
  }

  if (!bounds_consistent(root.lb, root.ub)) {
    out.stats.status = bc_status::kInfeasibleVariableBounds;
    return return_with_profile();
  }

  // Orbit-based symmetry breaking.
  // Detect orbits of identical binary columns in base_lp and append
  // lex-ordering inequalities  x_{g_{j+1}} - x_{g_j} <= 0  as new rows of
  // base_lp.A.  Two binary columns (j, k) are in the same orbit iff they
  // have matching cost, bounds, integer type, and identical (A, Aeq)
  // column entries.  The gating heuristic mirrors the SCUC pre-check:
  // at least `scuc_min_binaries` binaries and `scuc_min_rows` rows.
  int p9_sym_orbits_added = 0;
  int p9_sym_cuts_added   = 0;
  if (domain_heuristics && opt.use_symmetry_breaking) {
    // Cheap pre-check: gate on problem shape (same as is_scuc_like rough cut).
    int n_binary_total = 0;
    for (int j = 0; j < n; ++j) {
      if (base_lp.vars[j].type == VarType::Binary) ++n_binary_total;
    }
    const int total_rows_pre = static_cast<int>(base_lp.A.rows()) +
                               static_cast<int>(base_lp.Aeq.rows());
    const bool sym_gate_ok =
        n_binary_total >= opt.scuc_min_binaries &&
        total_rows_pre >= opt.scuc_min_rows &&
        opt.symmetry_min_orbit_size >= 2;
    if (sym_gate_ok) {
      // Build a 64-bit fingerprint of each binary column spanning
      // cost, bounds, and the (row_id, value) pattern of both A and Aeq.
      // Equal fingerprints → candidate orbit; verified by explicit
      // column-comparison before emitting cuts.
      auto hash_combine = [](uint64_t h, uint64_t v) -> uint64_t {
        v ^= v >> 33; v *= 0xff51afd7ed558ccdULL;
        v ^= v >> 33; v *= 0xc4ceb9fe1a85ec53ULL;
        v ^= v >> 33;
        return h ^ (v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
      };
      auto double_bits = [](double x) -> uint64_t {
        uint64_t u; std::memcpy(&u, &x, sizeof(u));
        return u;
      };
      // Extract a compact column representation for each binary.
      struct ColKey {
        int j;
        uint64_t sig;
      };
      std::vector<ColKey> keys;
      keys.reserve(static_cast<size_t>(n_binary_total));
      // Precompute column -> (row, value) lists for fast compare.
      std::vector<std::vector<std::pair<int, double>>> col_a(n), col_e(n);
      // A: column-major iteration.
      for (int k = 0; k < base_lp.A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, k); it; ++it) {
          const int jj = static_cast<int>(it.col());
          if (jj >= 0 && jj < n && base_lp.vars[jj].type == VarType::Binary) {
            col_a[jj].emplace_back(static_cast<int>(it.row()), it.value());
          }
        }
      }
      for (int k = 0; k < base_lp.Aeq.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, k); it; ++it) {
          const int jj = static_cast<int>(it.col());
          if (jj >= 0 && jj < n && base_lp.vars[jj].type == VarType::Binary) {
            col_e[jj].emplace_back(static_cast<int>(it.row()), it.value());
          }
        }
      }
      for (int j = 0; j < n; ++j) {
        if (base_lp.vars[j].type != VarType::Binary) continue;
        // Sort entries by row id for order-independent fingerprint.
        auto sort_by_row = [](std::vector<std::pair<int, double>>& v) {
          std::sort(v.begin(), v.end(),
                    [](const auto& a, const auto& b) { return a.first < b.first; });
        };
        sort_by_row(col_a[j]);
        sort_by_row(col_e[j]);
        uint64_t sig = 0xcbf29ce484222325ULL;
        sig = hash_combine(sig, double_bits(base_lp.c[j]));
        sig = hash_combine(sig, double_bits(base_lp.vars[j].lb));
        sig = hash_combine(sig, double_bits(base_lp.vars[j].ub));
        sig = hash_combine(sig, static_cast<uint64_t>(col_a[j].size()));
        for (const auto& [row, val] : col_a[j]) {
          sig = hash_combine(sig, static_cast<uint64_t>(row));
          sig = hash_combine(sig, double_bits(val));
        }
        sig = hash_combine(sig, static_cast<uint64_t>(col_e[j].size()) | (1ULL << 32));
        for (const auto& [row, val] : col_e[j]) {
          sig = hash_combine(sig, static_cast<uint64_t>(row));
          sig = hash_combine(sig, double_bits(val));
        }
        keys.push_back({j, sig});
      }
      // Bucket by signature, then verify exact column equality.
      std::sort(keys.begin(), keys.end(),
                [](const ColKey& a, const ColKey& b) {
                  return a.sig < b.sig || (a.sig == b.sig && a.j < b.j);
                });
      auto cols_equal = [&](int a, int b) -> bool {
        if (base_lp.c[a] != base_lp.c[b]) return false;
        if (base_lp.vars[a].lb != base_lp.vars[b].lb) return false;
        if (base_lp.vars[a].ub != base_lp.vars[b].ub) return false;
        if (col_a[a].size() != col_a[b].size()) return false;
        if (col_e[a].size() != col_e[b].size()) return false;
        for (size_t k = 0; k < col_a[a].size(); ++k) {
          if (col_a[a][k].first  != col_a[b][k].first)  return false;
          if (col_a[a][k].second != col_a[b][k].second) return false;
        }
        for (size_t k = 0; k < col_e[a].size(); ++k) {
          if (col_e[a][k].first  != col_e[b][k].first)  return false;
          if (col_e[a][k].second != col_e[b][k].second) return false;
        }
        return true;
      };
      // Walk buckets with the same signature; split into actual orbits.
      std::vector<std::vector<int>> orbits;
      size_t i = 0;
      while (i < keys.size()) {
        size_t i2 = i + 1;
        while (i2 < keys.size() && keys[i2].sig == keys[i].sig) ++i2;
        // Candidates [i, i2) share a signature; verify pairwise.
        std::vector<int> cand(i2 - i);
        for (size_t k = 0; k < cand.size(); ++k) cand[k] = keys[i + k].j;
        std::vector<char> used(cand.size(), 0);
        for (size_t a = 0; a < cand.size(); ++a) {
          if (used[a]) continue;
          std::vector<int> orb{cand[a]};
          used[a] = 1;
          for (size_t b = a + 1; b < cand.size(); ++b) {
            if (used[b]) continue;
            if (cols_equal(cand[a], cand[b])) {
              orb.push_back(cand[b]);
              used[b] = 1;
            }
          }
          if (static_cast<int>(orb.size()) >= opt.symmetry_min_orbit_size) {
            std::sort(orb.begin(), orb.end());
            orbits.push_back(std::move(orb));
          }
        }
        i = i2;
      }
      // Sort orbits by descending size; cap at symmetry_max_orbits.
      std::sort(orbits.begin(), orbits.end(),
                [](const std::vector<int>& a, const std::vector<int>& b) {
                  return a.size() > b.size();
                });
      const int max_orb = std::max(0, opt.symmetry_max_orbits);
      if (static_cast<int>(orbits.size()) > max_orb) orbits.resize(max_orb);

      // Count cuts to add: (|orbit| - 1) per orbit.
      int total_cuts = 0;
      for (const auto& orb : orbits) total_cuts += static_cast<int>(orb.size()) - 1;
      if (total_cuts > 0) {
        // Append total_cuts rows to base_lp.A with entries:
        //   row r: +1 at col g_{j+1}, -1 at col g_j, RHS b_r = 0
        // representing x_{g_{j+1}} - x_{g_j} <= 0.
        const int old_rows = static_cast<int>(base_lp.A.rows());
        const int new_rows = old_rows + total_cuts;
        // Rebuild sparse matrix from triplets (preserve existing entries).
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(static_cast<size_t>(base_lp.A.nonZeros()) + 2 * static_cast<size_t>(total_cuts));
        for (int k = 0; k < base_lp.A.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, k); it; ++it) {
            trips.emplace_back(static_cast<int>(it.row()),
                               static_cast<int>(it.col()), it.value());
          }
        }
        int row_cursor = old_rows;
        for (const auto& orb : orbits) {
          for (size_t j = 1; j < orb.size(); ++j) {
            trips.emplace_back(row_cursor, orb[j],      1.0);
            trips.emplace_back(row_cursor, orb[j - 1], -1.0);
            ++row_cursor;
          }
        }
        Eigen::SparseMatrix<double> A_new(new_rows, base_lp.A.cols());
        A_new.setFromTriplets(trips.begin(), trips.end());
        A_new.makeCompressed();
        Eigen::VectorXd b_new = Eigen::VectorXd::Zero(new_rows);
        b_new.head(old_rows) = base_lp.b;
        Eigen::VectorXd lhs_new;
        const bool had_row_lhs = lp_has_row_lhs(base_lp);
        if (had_row_lhs || base_lp.row_lhs.size() > 0) {
          lhs_new = Eigen::VectorXd::Constant(
              new_rows, -std::numeric_limits<double>::infinity());
          if (had_row_lhs && old_rows > 0) lhs_new.head(old_rows) = base_lp.row_lhs;
        }
        // Remaining entries default to 0 (our RHS).
        base_lp.A = std::move(A_new);
        base_lp.b = std::move(b_new);
        if (lhs_new.size() > 0) base_lp.row_lhs = std::move(lhs_new);
        p9_sym_orbits_added = static_cast<int>(orbits.size());
        p9_sym_cuts_added   = total_cuts;
        if (opt.verbose) {
          fmt::print(stderr, "[B&C-SYMMETRY] {} orbits detected, {} lex cuts appended "
                             "(binaries={}, rows {} -> {})\n",
                     p9_sym_orbits_added, p9_sym_cuts_added, n_binary_total,
                     old_rows, new_rows);
        }
      }
    }
  }

  const std::vector<VariableMeta> root_global_vars = base_lp.vars;
  LPModel root_lp = base_lp;
  apply_node_bounds(root_lp.vars, root.lb, root.ub);

  const bool skip_large_short_limit_root_implied_detection =
      !strict_highs_lp_contract && n > 10000 && opt.time_limit_sec <= 10.0;
  	  std::vector<char> root_implied_integer_cols =
      skip_large_short_limit_root_implied_detection
          ? std::vector<char>(static_cast<std::size_t>(n), 0)
	      : detect_implied_integer_columns(root_lp);
      const int root_highs_native_rows =
          static_cast<int>(root_lp.A.rows()) +
          static_cast<int>(root_lp.Aeq.rows());
      const int root_highs_native_nnz =
          static_cast<int>(root_lp.A.nonZeros()) +
          static_cast<int>(root_lp.Aeq.nonZeros());
      bool root_highs_presolved_space_same_shape = false;
      bool root_highs_presolved_space_same_nnz = false;
      bool root_highs_presolved_col_identity = false;
      int root_highs_presolved_nonidentity_cols = 0;
      if (highs_presolve_side_state.side_state_available) {
        root_highs_presolved_space_same_shape =
            highs_presolve_side_state.cols == n &&
            highs_presolve_side_state.rows == root_highs_native_rows;
        root_highs_presolved_space_same_nnz =
            highs_presolve_side_state.nnz == root_highs_native_nnz;
        root_highs_presolved_col_identity =
            static_cast<int>(highs_presolve_side_state.presolved_col_orig.size()) ==
                highs_presolve_side_state.cols &&
            static_cast<int>(highs_presolve_side_state.presolved_col_scale.size()) ==
                highs_presolve_side_state.cols &&
            static_cast<int>(highs_presolve_side_state.presolved_col_constant.size()) ==
                highs_presolve_side_state.cols &&
            static_cast<int>(
                highs_presolve_side_state
                .presolved_col_linearly_transformable.size()) ==
                highs_presolve_side_state.cols &&
            highs_presolve_side_state.cols == n;
        if (strict_highs_presolved_working_space &&
            root_highs_presolved_col_identity) {
          root_highs_presolved_col_identity = true;
        } else
        if (root_highs_presolved_col_identity) {
          for (int hcol = 0; hcol < highs_presolve_side_state.cols; ++hcol) {
            const int orig =
                highs_presolve_side_state
                    .presolved_col_orig[static_cast<std::size_t>(hcol)];
            const double scale =
                highs_presolve_side_state
                    .presolved_col_scale[static_cast<std::size_t>(hcol)];
            const double constant =
                highs_presolve_side_state
                    .presolved_col_constant[static_cast<std::size_t>(hcol)];
            const bool identity =
                orig == hcol &&
                highs_presolve_side_state
                        .presolved_col_linearly_transformable
                            [static_cast<std::size_t>(hcol)] != 0 &&
                std::isfinite(scale) && std::isfinite(constant) &&
                std::abs(scale - 1.0) <= 1e-12 &&
                std::abs(constant) <= 1e-12;
            if (!identity) ++root_highs_presolved_nonidentity_cols;
          }
          root_highs_presolved_col_identity =
              root_highs_presolved_nonidentity_cols == 0;
        }
      }
      const bool root_highs_presolved_space_exact =
          strict_highs_root_fixed_point &&
          highs_presolve_side_state.side_state_available &&
          root_highs_presolved_space_same_shape &&
          root_highs_presolved_space_same_nnz &&
          root_highs_presolved_col_identity;
      const bool root_highs_presolved_side_state_first_class =
          !strict_highs_root_fixed_point || root_highs_presolved_space_exact;
      if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
           bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) &&
          strict_highs_root_fixed_point &&
          highs_presolve_side_state.side_state_available) {
        fmt::print(stderr,
                   "[B&C-HIGHS-SPACE{}] first_class={} exact={} "
                   "native_cols={} native_rows={} native_nnz={} "
                   "native_ineq={} native_eq={} "
                   "highs_cols={} highs_rows={} highs_nnz={} "
                   "same_shape={} same_nnz={} col_identity={} "
                   "nonidentity_cols={} status={}\n",
                   root_highs_presolved_space_exact ? "" : "-MISMATCH",
                   root_highs_presolved_side_state_first_class ? 1 : 0,
                   root_highs_presolved_space_exact ? 1 : 0, n,
                   root_highs_native_rows, root_highs_native_nnz,
                   static_cast<int>(root_lp.A.rows()),
                   static_cast<int>(root_lp.Aeq.rows()),
                   highs_presolve_side_state.cols,
                   highs_presolve_side_state.rows,
                   highs_presolve_side_state.nnz,
                   root_highs_presolved_space_same_shape ? 1 : 0,
                   root_highs_presolved_space_same_nnz ? 1 : 0,
                   root_highs_presolved_col_identity ? 1 : 0,
                   root_highs_presolved_nonidentity_cols,
                   highs_presolve_side_state.highs_status);
      }
      if (strict_highs_root_fixed_point && !root_highs_presolved_space_exact) {
        out.stats.status = bc_status::kHighsPresolvedWorkingLpMismatch;
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
            bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
          fmt::print(stderr,
                     "[B&C-HIGHS-SPACE-MISMATCH] fatal=1 "
                     "reason=required_for_strict_root native_cols={} "
                     "native_rows={} native_nnz={} highs_cols={} "
                     "highs_rows={} highs_nnz={} same_shape={} same_nnz={} "
                     "col_identity={} nonidentity_cols={} status={}\n",
                     n, root_highs_native_rows, root_highs_native_nnz,
                     highs_presolve_side_state.cols,
                     highs_presolve_side_state.rows,
                     highs_presolve_side_state.nnz,
                     root_highs_presolved_space_same_shape ? 1 : 0,
                     root_highs_presolved_space_same_nnz ? 1 : 0,
                     root_highs_presolved_col_identity ? 1 : 0,
                     root_highs_presolved_nonidentity_cols,
                     highs_presolve_side_state.highs_status);
        }
        return return_with_profile();
      }
	  std::vector<char> root_highs_presolved_integral_cols(
	      static_cast<std::size_t>(n), 0);
	  bool root_highs_presolved_integral_mask_valid = false;
	  std::vector<char> root_highs_presolved_implied_integer_cols(
	      static_cast<std::size_t>(n), 0);
	  int root_highs_presolved_implied_integer_count = 0;

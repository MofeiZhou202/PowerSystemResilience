// @file bc_run/06_root_heuristics_a.inc
// BCSolveState::run() segment 6/14: persistent-LP transition and root heuristics — part 1 (IPM re-solve, feasibility pump).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
          root_relax = std::move(incremental_relax);
          root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
          root.basis_hint = root_relax.basis_hint;
          // A committed live append changes the basis dimension; publish the
          // matching owner before any tableau query. See
          // docs/miplib2017_benchmark_protocol_2026-08-25.md,
          // "Stability-gate mismatch and re-derivation".
          if (root_relax.simplex &&
              root_relax.simplex->basis.cached_sparse_basis) {
            root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
          } else {
            root_sbasis.reset();
          }
          root.bound = root_relax.dual_bound;
          refresh_root_redcost_lurking_bounds("root_cut_round_incremental");
          persistent_cut_resolved = true;
          prof_gen_ms += std::chrono::duration<double, std::milli>(
              t_cut_gen - t_cut_start).count();
          prof_solve_ms += root_relax.primal.stats.runtime_sec * 1000.0;
          ++prof_rounds;
        } else {
          (void)root_cut_transaction->rollback_lp_transaction();
          root_cut_transaction.reset();
        }
      }

      // IPM re-solve path: when use_ipm_root is active, avoid rebuilding and
      // renumbering a simplex standard form. Trade-off: the IPM result has no
      // simplex basis, so tableau cuts require an earlier crossover basis.
      if (persistent_cut_resolved) {
        // State publication is deferred until the admission test below.
      } else if (root_cut_ipm) {
        const double cut_remaining =
            std::min(bc_remaining_sec() - bc_post_root_finalize_reserve_sec,
                     cut_stage_remaining_sec());
        if (cut_remaining <= 0.0) break;
        root_cut_ipm->set_solve_time_limit(cut_remaining);
        auto t_cut_solve0 = std::chrono::steady_clock::now();
        auto ipm_res = root_cut_ipm->solve_lp(root_lp, root.x_relax);
        auto t_cut_solve1 = std::chrono::steady_clock::now();
        if (opt.verbose) {
          fmt::print(stderr, "[B&C CUT-IPM] round {}: gen={:.1f}ms solve={:.1f}ms added={}\n",
                  k,
                  std::chrono::duration<double, std::milli>(t_cut_gen - t_cut_start).count(),
                  std::chrono::duration<double, std::milli>(t_cut_solve1 - t_cut_solve0).count(),
                  added);
        }
        out.bc_stats.lp_solves += 1;
        if (!ipm_res.stats.success || ipm_res.x.size() != n ||
            !std::isfinite(ipm_res.stats.objective)) {
          if (opt.verbose) {
            MIPSOLVERS_LOG_DEBUG("[B&C] cut round {}: IPM re-solve FAILED — reverting {} cuts", k, added);
          }
          truncate_ineq_rows(root_lp, prev_ineq_rows);
          out.bc_stats.cuts_added -= actual_added_rows;
          root.lb = round_lb;
          root.ub = round_ub;
          apply_node_bounds(root_lp.vars, root.lb, root.ub);
          root.basis_hint = round_basis_hint;
          root.x_relax = round_x_relax;
          root.bound = round_bound;
          root_relax = round_relax;
          root_sbasis = round_sbasis;
          break;
        }
        // Update root solution from IPM result.
        root.x_relax = clamp_to_bounds(ipm_res.x, root.lb, root.ub);
        root.bound = ipm_res.stats.objective;
        // No simplex basis from IPM → Gomory won't fire in subsequent rounds.
        // Basis-free cuts (MIR, cover, clique) still work.
        root_relax.simplex.reset();
        root_relax.primal = ipm_res;
        root_relax.dual_bound = ipm_res.stats.objective;
        // IPM cleared simplex basis — subsequent rounds generate only
        // basis-free cuts.  Skip further rounds on very large problems
        // where scanning is expensive (~1s) and rarely useful.
        if (presolved_m > opt.ipm_cut_resolve_row_threshold) {
          break;
        }
      } else {
      // ── Simplex re-solve path (original code) ──
      StandardFormLP cut_sf = build_standard_form_lp(root_lp);
      ruiz_scale_standard_form(cut_sf);
      auto t_cut_sf = std::chrono::steady_clock::now();
      // The added cut rows create new slack columns; put them in the basis
      // so the warm-started dual simplex only needs to restore optimality.
      const int cut_sf_m = static_cast<int>(cut_sf.A.rows());
      const int cut_sf_n = static_cast<int>(cut_sf.A.cols());
      std::shared_ptr<SimplexBasis> extended_hint;
      if (root.basis_hint && root.basis_hint->rows < cut_sf_m &&
          root.basis_hint->cols < cut_sf_n) {
        extended_hint = std::make_shared<SimplexBasis>(*root.basis_hint);
        extended_hint->ensure_owned_indices();
        const int old_m = extended_hint->rows;
        const int old_n = extended_hint->cols;

        // build_standard_form_lp lays out rows as [ineq | eq].  Adding ≤ cut
        // rows expands the ineq block, shifting the equality rows DOWN.
        // We must: (1) insert new basis entries (slacks for cuts) at the
        // boundary between the old inequality and equality blocks,
        // (2) adjust column indices for surplus/artificial column shifts.
        const int n_new_rows = cut_sf_m - old_m;
        const int n_orig = cut_sf.n_original;

        // Figure out how many of the old rows were inequality vs equality.
        // IMPORTANT: use prev_ineq_rows (captured BEFORE add_cuts) since
        // root_lp.A.rows() now includes the cut rows.
        const int m_eq = old_m - prev_ineq_rows;
        const int old_m_ineq = old_m - m_eq;  // == prev_ineq_rows

        // Build the new basis index array with correct row ordering:
        // [old_ineq_basis (0..old_m_ineq-1)] [new_cut_slacks] [old_eq_basis]
        std::vector<int> new_indices(static_cast<size_t>(cut_sf_m));
        std::vector<char> new_at_upper(static_cast<size_t>(cut_sf_n), 0);

        // Copy at_upper for old columns.
        if (!extended_hint->at_upper.empty()) {
          const int copy_n = std::min(old_n, cut_sf_n);
          for (int j = 0; j < copy_n; ++j) {
            new_at_upper[static_cast<size_t>(j)] = extended_hint->at_upper[static_cast<size_t>(j)];
          }
        }

        // Fix column renumbering: surplus/artificial columns shift right.
        // Use saved SF counts from the root solve (not cut SF counts which differ
        // when cut rows with negative RHS get flipped from L→G).
        const int n_slack_old = (root.basis_hint->sf_n_slack >= 0)
            ? root.basis_hint->sf_n_slack
            : old_n - n_orig - cut_sf.n_surplus - cut_sf.n_artificial;
        const int n_surplus_old = (root.basis_hint->sf_n_surplus >= 0)
            ? root.basis_hint->sf_n_surplus
            : cut_sf.n_surplus;
        const int delta_slack = cut_sf.n_slack - n_slack_old;
        const int delta_surplus = cut_sf.n_surplus - n_surplus_old;
        auto adjust_col = [&](int idx) -> int {
          if (idx < n_orig + n_slack_old) return idx;  // original or old slack
          // surplus columns: shift right by delta_slack
          if (idx < n_orig + n_slack_old + n_surplus_old) {
            return idx + delta_slack;
          }
          // artificial columns: shift right by delta_slack + delta_surplus
          return idx + delta_slack + delta_surplus;
        };

        // Old inequality rows (0..old_m_ineq-1) → new rows 0..old_m_ineq-1.
        for (int i = 0; i < old_m_ineq; ++i) {
          new_indices[static_cast<size_t>(i)] = adjust_col(extended_hint->indices[static_cast<size_t>(i)]);
        }
        // New cut rows → new rows old_m_ineq..old_m_ineq+n_new_rows-1.
        for (int k = 0; k < n_new_rows; ++k) {
          const int r = old_m_ineq + k;  // row index in new SF
          const int slack = cut_sf.row_to_slack_col[static_cast<size_t>(r)];
          const int art = cut_sf.row_to_artificial_col[static_cast<size_t>(r)];
          new_indices[static_cast<size_t>(r)] = (slack >= 0) ? slack : (art >= 0) ? art : -1;
        }
        // Old equality rows (old_m_ineq..old_m-1) → new rows old_m_ineq+n_new_rows..
        for (int i = old_m_ineq; i < old_m; ++i) {
          const int new_row = i + n_new_rows;
          new_indices[static_cast<size_t>(new_row)] = adjust_col(extended_hint->indices[static_cast<size_t>(i)]);
        }

        extended_hint->indices = std::move(new_indices);
        extended_hint->at_upper = std::move(new_at_upper);
        extended_hint->rows = cut_sf_m;
        extended_hint->cols = cut_sf_n;
        // Invalidate cached data that doesn't match new dimensions.
        extended_hint->cached_sparse_basis.reset();
        extended_hint->cached_reduced_costs.reset();

        // Validate: all basis indices must be in [0, cut_sf_n) and no duplicates.
        // If invalid, discard the hint entirely (cold-start simplex is safer).
        {
          bool valid = true;
          std::vector<char> used(static_cast<size_t>(cut_sf_n), 0);
          for (int i = 0; i < cut_sf_m; ++i) {
            int idx = extended_hint->indices[static_cast<size_t>(i)];
            if (idx < 0 || idx >= cut_sf_n) { valid = false; break; }
            if (used[static_cast<size_t>(idx)]) { valid = false; break; }
            used[static_cast<size_t>(idx)] = 1;
          }
          if (!valid) {
            extended_hint.reset();
          }
        }
      }

      SimplexOptions simplex_opt;
      simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
      simplex_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
      simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
      simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
      simplex_opt.verbose = false;
      const double cut_remaining =
          std::min(bc_remaining_sec() - bc_post_root_finalize_reserve_sec,
                   cut_stage_remaining_sec());
      if (cut_remaining <= 0.0) break;
      simplex_opt.time_limit_sec = cut_remaining;
      auto t_cut_solve0 = std::chrono::steady_clock::now();
      // Use extended hint if available and valid. If the old basis hint's
      // dimensions match cut_sf exactly, use it directly. Otherwise, cold-start.
      const SimplexBasis* hint_ptr = nullptr;
      if (extended_hint) {
        hint_ptr = extended_hint.get();
      } else if (root.basis_hint &&
                 root.basis_hint->rows == cut_sf_m &&
                 root.basis_hint->cols == cut_sf_n) {
        hint_ptr = root.basis_hint.get();
      }
      auto simplex_res = std::make_shared<SimplexResult>(
          solve_lp_from_sf(cut_sf, simplex_opt, hint_ptr));
      simplex_res->form = cut_sf;
      if (simplex_res->basis.cached_sparse_basis) {
        rebind_sparse_basis_matrix(simplex_res->basis.cached_sparse_basis,
                                   simplex_res->form.A);
      }
      auto t_cut_solve1 = std::chrono::steady_clock::now();
      prof_gen_ms += std::chrono::duration<double, std::milli>(t_cut_gen - t_cut_start).count();
      prof_sf_ms += std::chrono::duration<double, std::milli>(t_cut_sf - t_cut_gen).count();
      prof_solve_ms += std::chrono::duration<double, std::milli>(t_cut_solve1 - t_cut_solve0).count();
      ++prof_rounds;
      if (opt.verbose) {
        fmt::print(stderr, "[B&C CUT] round {}: gen={:.1f}ms sf={:.1f}ms solve={:.1f}ms added={}\n",
                k,
                std::chrono::duration<double, std::milli>(t_cut_gen - t_cut_start).count(),
                std::chrono::duration<double, std::milli>(t_cut_sf - t_cut_gen).count(),
                std::chrono::duration<double, std::milli>(t_cut_solve1 - t_cut_solve0).count(),
                added);
      }

      out.bc_stats.lp_solves += 1;
      if (!simplex_res->result.stats.success ||
          simplex_res->result.x.size() != n ||
          !std::isfinite(simplex_res->result.stats.objective)) {
        if (opt.verbose) {
          MIPSOLVERS_LOG_DEBUG("[B&C] cut round {}: re-solve FAILED — reverting {} cuts", k, added);
        }
        truncate_ineq_rows(root_lp, prev_ineq_rows);
        out.bc_stats.cuts_added -= actual_added_rows;
        root.lb = round_lb;
        root.ub = round_ub;
        apply_node_bounds(root_lp.vars, root.lb, root.ub);
        root.basis_hint = round_basis_hint;
        root.x_relax = round_x_relax;
        root.bound = round_bound;
        root_relax = round_relax;
        root_sbasis = round_sbasis;
        break;
      }

      root_relax.primal = simplex_res->result;
      root_relax.dual_bound = simplex_res->result.stats.objective;
      root_relax.simplex = simplex_res;
      auto cut_basis = std::make_shared<SimplexBasis>(simplex_res->basis);
      cut_basis->cached_reduced_costs =
          std::make_shared<const Eigen::VectorXd>(simplex_res->reduced_costs);
      if (simplex_res->basis.cached_sparse_basis) {
        cut_basis->cached_sparse_basis = simplex_res->basis.cached_sparse_basis;
      }
      root_relax.basis_hint = cut_basis;

	      root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	      root.basis_hint = root_relax.basis_hint;
	      root.bound = root_relax.dual_bound;
	      refresh_root_redcost_lurking_bounds("root_cut_round_resolve");
	      }  // end simplex re-solve path

      const double candidate_bound_after_round = root.bound;
      const double round_lift = candidate_bound_after_round - round_bound;
      const double min_lift = std::max(
          opt.root_cut_min_bound_lift,
          1e-10 * std::max(1.0, std::abs(round_bound)));
      std::vector<int> round_frac_after;
      const bool has_fractional_after_round =
          fractional_branchable_indices(root.x_relax, root.lb, root.ub,
                                        opt.int_tol,
                                        round_frac_after);
      const bool termination_effective =
          highs_root_pipeline_active ||
          !opt.root_cut_reject_nonmoving_rows || round_domain_tightened > 0 ||
          round_lift > min_lift;
      if (!termination_effective) {
        if (root_cut_transaction) {
          (void)root_cut_transaction->rollback_lp_transaction();
          root_cut_transaction.reset();
        }
        truncate_ineq_rows(root_lp, prev_ineq_rows);
        out.bc_stats.cuts_added -= actual_added_rows;
        root.lb = round_lb;
        root.ub = round_ub;
        apply_node_bounds(root_lp.vars, root.lb, root.ub);
        root.basis_hint = round_basis_hint;
        root.x_relax = round_x_relax;
        root.bound = round_bound;
        root_relax = round_relax;
        root_sbasis = round_sbasis;
        out.bc_stats.root_cut_rows_rejected_nonmoving +=
            static_cast<std::uint64_t>(actual_added_rows);
        ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
        if (opt.verbose || force_root_cut_audit) {
          fmt::print(stderr,
                     "[CUT-AUDIT] rejected nonmoving root cut round {}: "
                     "rows={} bound={:.2f}->{:.2f} lift={:.3e} "
                     "domain_tight={}\n",
                     k, actual_added_rows, round_bound,
                     candidate_bound_after_round,
                     round_lift, round_domain_tightened);
        }
        break;
      }
      if (root_cut_transaction) {
        (void)root_cut_transaction->commit_lp_transaction();
        root_cut_transaction.reset();
      }
      if (round_domain_tightened > 0) {
        out.bc_stats.root_cut_domain_tightenings +=
            static_cast<std::uint64_t>(round_domain_tightened);
      }

      const double improvement = std::abs(root.bound - prev_bound);
      const double denom = std::max(1.0, std::abs(root.bound));
      if (highs_root_pipeline_active) {
        const bool highs_progress_stalled =
            (root.bound - pre_cut_bound) <=
            std::max(prev_bound - pre_cut_bound, opt.lp_tol) * 1.01;
        if (highs_progress_stalled) {
          ++stall_count;
        } else {
          stall_count = 0;
        }
        if (!has_fractional_after_round || stall_count >= 3) {
          if (opt.verbose) {
            fmt::print(stderr,
                       "[B&C CUT-HIGHS] stop root separation after round {} "
                       "frac={} stall={} bound={:.10g}\n",
                       k + 1, static_cast<int>(round_frac_after.size()),
                       stall_count, root.bound);
          }
          break;
        }
      } else if (improvement / denom < opt.root_cut_stall_tol) {
        ++stall_count;
        if (stall_count >= opt.root_cut_max_stalls) {
          if (opt.verbose) {
            MIPSOLVERS_LOG_DEBUG("[B&C] root cuts stalled after {} rounds (improvement {:.6f}%)",
                    k + 1, 100.0 * improvement / denom);
          }
          break;
        }
      } else {
        stall_count = 0;
      }
      prev_bound = root.bound;
    }
    if (prof_rounds > 0) {
      MIPSOLVERS_LOG_DEBUG("[B&C] Root cuts: {} rounds, gen={:.1f}ms sf={:.1f}ms solve={:.1f}ms total={:.1f}ms",
                        prof_rounds, prof_gen_ms, prof_sf_ms, prof_solve_ms,
                        prof_gen_ms + prof_sf_ms + prof_solve_ms);
    }
  }

  // Diagnostic: root LP bound before and after cuts
  out.bc_stats.root_cut_bound_lift = std::max(0.0, root.bound - pre_cut_bound);
  if (opt.verbose) {
    fmt::print(stderr, "[B&C DIAG] root_lp_bound_pre_cut={:.2f}  root_lp_bound_post_cut={:.2f}  cuts_added={}  gap_closed_by_cuts={:.4f}% rejected_nonmoving_rows={} domain_tight={}\n",
               pre_cut_bound, root.bound, out.bc_stats.cuts_added,
               (std::abs(root.bound) > 1e-6) ? 100.0 * (root.bound - pre_cut_bound) / std::abs(root.bound) : 0.0,
               out.bc_stats.root_cut_rows_rejected_nonmoving,
               out.bc_stats.root_cut_domain_tightenings);
  }

  // Per-family cut diagnostic.
  if (opt.verbose) {
    const char* family_names[] = {"Gomory", "MIR", "Cover", "Clique", "ZeroHalf"};
    bool any = false;
    for (int f = 0; f < static_cast<int>(CutFamily::Count); ++f) {
      if (cut_family_tracker.families[f].cuts_total > 0) { any = true; break; }
    }
    if (any) {
      fmt::print(stderr, "[CUT-DIAG] per-family root cuts:");
      for (int f = 0; f < static_cast<int>(CutFamily::Count); ++f) {
        const auto& fs = cut_family_tracker.families[f];
        if (fs.cuts_total > 0)
          fmt::print(stderr, " {}={}", family_names[f], fs.cuts_total);
      }
      fmt::print(stderr, "\n");
    } else if (effective_cut_rounds == 0) {
      fmt::print(stderr, "[CUT-DIAG] root cuts skipped. cuts={} opt.cuts={} "
                         "effective_cut_rounds=0 reason={}\n",
                 out.bc_stats.cuts_added, static_cast<int>(opt.cuts),
                 root_cut_skip_reason.empty() ? "disabled_or_budget_zero"
                                              : root_cut_skip_reason);
    } else {
      fmt::print(stderr, "[CUT-DIAG] no cuts added at root (all families returned 0). "
                         "cuts={} opt.cuts={} effective_cut_rounds={}\n",
                 out.bc_stats.cuts_added, static_cast<int>(opt.cuts), effective_cut_rounds);
      fmt::print(stderr, "[CUT-DIAG] family admission attempts:");
      for (int f = 0; f < static_cast<int>(CutFamily::Count); ++f) {
        const auto& fs = cut_family_tracker.families[f];
        if (fs.attempts > 0 || fs.zero_rounds > 0 || fs.skipped_rounds > 0) {
          fmt::print(stderr, " {}(try={},zero={},skip={},budget={})",
                     family_names[f], fs.attempts, fs.zero_rounds,
                     fs.skipped_rounds, fs.budget_total);
        }
      }
      fmt::print(stderr, "\n");
    }
  }

  if (out.bc_stats.cuts_added > 0) {
    base_lp = root_lp;
  } else {
    base_lp = pre_cut_root_lp;
  }
  base_lp.vars = root_global_vars;
  if (use_objective_propagation) {
    objective_propagation.setup(base_lp, &root_implied_integer_cols,
                                &clique_table, &implication_graph,
                                objective_propagation_build_policy());
    const auto published = publish_objective_implied_event_source(
        base_lp, objective_propagation, implication_graph, clique_table,
        opt.int_tol);
    out.bc_stats.objective_implied_event_published_implications +=
        published.implications_added;
	    out.bc_stats.objective_implied_event_published_clique_edges +=
	        published.clique_edges_added;
	    if (published.implications_added > 0 || published.clique_edges_added > 0) {
	      const std::uint64_t published_total =
	          published.implications_added + published.clique_edges_added;
	      note_global_domain_learning(published_total);
	      note_objective_artifact_learning(published_total);
	      objective_propagation.setup(base_lp, &root_implied_integer_cols,
	                                  &clique_table, &implication_graph,
	                                  objective_propagation_build_policy());
	    }
	  }
  out.bc_stats.objective_clique_partitions =
      static_cast<std::uint64_t>(objective_propagation.partitions.size());
  for (const auto& part : objective_propagation.partitions) {
    out.bc_stats.objective_clique_terms +=
        static_cast<std::uint64_t>(part.terms.size());
  }
  out.bc_stats.objective_clique_edges = objective_propagation.clique_edges;
  out.bc_stats.objective_clique_mixed_rows =
      objective_propagation.mixed_rows_scanned;
  out.bc_stats.objective_clique_max_replacement_delta =
      objective_propagation.max_replacement_delta;
  out.bc_stats.objective_implied_events =
      static_cast<std::uint64_t>(objective_propagation.implied_events.size());
  out.bc_stats.objective_implied_event_vlb_events =
      objective_propagation.implied_event_vlb_events;
  out.bc_stats.objective_implied_event_vub_events =
      objective_propagation.implied_event_vub_events;
  out.bc_stats.objective_implied_event_integral_rounds =
      objective_propagation.implied_event_integral_rounds;
  out.bc_stats.objective_implied_event_mir_attempts =
      objective_propagation.implied_event_mir_attempts;
  out.bc_stats.objective_implied_event_mir_strengthened =
      objective_propagation.implied_event_mir_strengthened;
  out.bc_stats.objective_implied_event_mir_gain_sum =
      objective_propagation.implied_event_mir_gain_sum;
  out.bc_stats.objective_implied_event_mir_gain_max =
      objective_propagation.implied_event_mir_gain_max;
  out.bc_stats.objective_implied_event_max_delta =
      objective_propagation.max_implied_event_delta;
  out.bc_stats.objective_implied_event_max_aggregate_delta =
      objective_propagation.max_implied_event_aggregate_delta;
	  auto refresh_objective_artifact_stats = [&]() {
	    out.bc_stats.root_clique_edges = clique_table.n_literal_edges();
	    out.bc_stats.root_implication_arcs =
	        static_cast<std::uint64_t>(implication_graph.size());
    out.bc_stats.objective_clique_partitions =
        static_cast<std::uint64_t>(objective_propagation.partitions.size());
    out.bc_stats.objective_clique_terms = 0;
    for (const auto& part : objective_propagation.partitions) {
      out.bc_stats.objective_clique_terms +=
          static_cast<std::uint64_t>(part.terms.size());
    }
    out.bc_stats.objective_clique_edges = objective_propagation.clique_edges;
    out.bc_stats.objective_clique_mixed_rows =
        objective_propagation.mixed_rows_scanned;
    out.bc_stats.objective_clique_max_replacement_delta =
        objective_propagation.max_replacement_delta;
    out.bc_stats.objective_implied_events =
        static_cast<std::uint64_t>(objective_propagation.implied_events.size());
    out.bc_stats.objective_implied_event_vlb_events =
        objective_propagation.implied_event_vlb_events;
    out.bc_stats.objective_implied_event_vub_events =
        objective_propagation.implied_event_vub_events;
    out.bc_stats.objective_implied_event_integral_rounds =
        objective_propagation.implied_event_integral_rounds;
    out.bc_stats.objective_implied_event_mir_attempts =
        objective_propagation.implied_event_mir_attempts;
    out.bc_stats.objective_implied_event_mir_strengthened =
        objective_propagation.implied_event_mir_strengthened;
    out.bc_stats.objective_implied_event_mir_gain_sum =
        objective_propagation.implied_event_mir_gain_sum;
    out.bc_stats.objective_implied_event_mir_gain_max =
        objective_propagation.implied_event_mir_gain_max;
    out.bc_stats.objective_implied_event_max_delta =
        objective_propagation.max_implied_event_delta;
	    out.bc_stats.objective_implied_event_max_aggregate_delta =
	        objective_propagation.max_implied_event_aggregate_delta;
	  };

	  struct ObjectiveDomainSourceRefreshStats {
	    std::uint64_t published_implications{0};
	    std::uint64_t published_clique_edges{0};
	    std::uint64_t setup_passes{0};

	    std::uint64_t total_published() const {
	      return published_implications + published_clique_edges;
	    }
	  };

	  std::uint64_t last_objective_source_refresh_domain_epoch =
	      std::numeric_limits<std::uint64_t>::max();
	  std::uint64_t last_objective_source_refresh_objective_epoch =
	      std::numeric_limits<std::uint64_t>::max();
	  std::uint64_t last_objective_source_refresh_clique_edges =
	      std::numeric_limits<std::uint64_t>::max();
	  std::uint64_t last_objective_source_refresh_implications =
	      std::numeric_limits<std::uint64_t>::max();
	  Eigen::Index last_objective_source_refresh_ineq_rows = -1;
	  Eigen::Index last_objective_source_refresh_eq_rows = -1;
	  auto refresh_objective_domain_sources =
	      [&](const char* source,
	          bool force = false) -> ObjectiveDomainSourceRefreshStats {
	    ObjectiveDomainSourceRefreshStats stats;
	    if (!use_objective_propagation) {
	      return stats;
	    }

	    // HiGHS keeps objective propagation inside the same domain closure, but
	    // its source structure is tied to the objective, clique table and
	    // implication graph, not to every learned bound certificate.  Rebuild
	    // only when that structural source changes; ordinary domain learning is
	    // consumed by propagation using the current node bounds.
	    const std::uint64_t requested_objective_epoch =
	        current_objective_artifact_epoch();
	    const std::uint64_t requested_clique_edges =
	        static_cast<std::uint64_t>(clique_table.n_literal_edges());
	    const std::uint64_t requested_implications =
	        static_cast<std::uint64_t>(implication_graph.size());
	    const Eigen::Index requested_ineq_rows = base_lp.A.rows();
	    const Eigen::Index requested_eq_rows = base_lp.Aeq.rows();
	    if (!force &&
	        last_objective_source_refresh_objective_epoch ==
	            requested_objective_epoch &&
	        last_objective_source_refresh_clique_edges ==
	            requested_clique_edges &&
	        last_objective_source_refresh_implications ==
	            requested_implications &&
	        last_objective_source_refresh_ineq_rows == requested_ineq_rows &&
	        last_objective_source_refresh_eq_rows == requested_eq_rows) {
	      return stats;
	    }

	    constexpr int kMaxObjectiveSourceRefreshPasses = 16;
	    bool final_setup_needed = false;
		    for (int pass = 0; pass < kMaxObjectiveSourceRefreshPasses; ++pass) {
		      ++stats.setup_passes;
		      objective_propagation.setup(base_lp, &root_implied_integer_cols,
		                                  &clique_table, &implication_graph,
		                                  objective_propagation_build_policy());
	      const auto published = publish_objective_implied_event_source(
	          base_lp, objective_propagation, implication_graph, clique_table,
	          opt.int_tol);
	      out.bc_stats.objective_implied_event_published_implications +=
	          published.implications_added;
	      out.bc_stats.objective_implied_event_published_clique_edges +=
	          published.clique_edges_added;

	      const std::uint64_t published_total =
	          published.implications_added + published.clique_edges_added;
	      if (published_total == 0) {
	        final_setup_needed = false;
	        break;
	      }

	      stats.published_implications += published.implications_added;
	      stats.published_clique_edges += published.clique_edges_added;
	      note_global_domain_learning(published_total);
	      note_objective_artifact_learning(published_total);
	      final_setup_needed = true;
	    }

		    if (final_setup_needed) {
		      ++stats.setup_passes;
		      objective_propagation.setup(base_lp, &root_implied_integer_cols,
		                                  &clique_table, &implication_graph,
		                                  objective_propagation_build_policy());
		    }
	    refresh_objective_artifact_stats();
	    last_objective_source_refresh_domain_epoch =
	        current_domain_learning_epoch();
	    last_objective_source_refresh_objective_epoch =
	        current_objective_artifact_epoch();
	    last_objective_source_refresh_clique_edges =
	        static_cast<std::uint64_t>(clique_table.n_literal_edges());
	    last_objective_source_refresh_implications =
	        static_cast<std::uint64_t>(implication_graph.size());
	    last_objective_source_refresh_ineq_rows = base_lp.A.rows();
	    last_objective_source_refresh_eq_rows = base_lp.Aeq.rows();

	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C-OBJ-SRC-REFRESH] source={} setup={} pub={}/{} "
	                 "domainEpoch={} objectiveEpoch={} objParts={} objEvents={} "
	                 "clqEdges={} arcs={}\n",
	                 source != nullptr ? source : "domain_source",
	                 stats.setup_passes, stats.published_implications,
	                 stats.published_clique_edges,
	                 current_domain_learning_epoch(),
	                 current_objective_artifact_epoch(),
	                 objective_propagation.partitions.size(),
	                 objective_propagation.implied_events.size(),
	                 clique_table.n_literal_edges(), implication_graph.size());
	    }
	    return stats;
	  };

	  record_conformance_source("root_postcut_source");

  [[maybe_unused]] auto t_postcut0 = std::chrono::steady_clock::now();

  // IPM root cut path: final simplex crossover to get a basis for tree
  // exploration. The IPM re-solves don't produce a simplex basis, so we
  // extend the pre-cut crossover basis with slack columns for all added cuts,
  // then warm-start simplex from there (same mechanism as per-round extension).
  // Skip post-cut crossover on very large problems and use IPM-only mode.
  if (opt.use_ipm_root && !root_relax.simplex && out.bc_stats.cuts_added > 0 &&
      pre_cut_basis_hint && presolved_m <= opt.xlarge_ipm_only_threshold) {
    StandardFormLP xover_sf = build_standard_form_lp(base_lp);
    ruiz_scale_standard_form(xover_sf);
    [[maybe_unused]] auto t_postcut_sf = std::chrono::steady_clock::now();
    const int xo_m = static_cast<int>(xover_sf.A.rows());
    const int xo_n = static_cast<int>(xover_sf.A.cols());

    // Extend pre-cut basis with slack columns for new cut rows.
    std::shared_ptr<SimplexBasis> extended_hint;
    if (pre_cut_basis_hint->rows < xo_m && pre_cut_basis_hint->cols < xo_n) {
      extended_hint = std::make_shared<SimplexBasis>(*pre_cut_basis_hint);
      const int old_m = extended_hint->rows;
      const int old_n = extended_hint->cols;
      const int n_new_rows = xo_m - old_m;
      const int n_orig = xover_sf.n_original;
      const int pre_cut_ineq = static_cast<int>(pre_cut_root_lp.A.rows());
      const int old_m_ineq = pre_cut_ineq;

      std::vector<int> new_indices(static_cast<size_t>(xo_m));
      std::vector<char> new_at_upper(static_cast<size_t>(xo_n), 0);
      if (!extended_hint->at_upper.empty()) {
        const int copy_n = std::min(old_n, xo_n);
        for (int j = 0; j < copy_n; ++j)
          new_at_upper[static_cast<size_t>(j)] = extended_hint->at_upper[static_cast<size_t>(j)];
      }
      const int n_slack_old = (pre_cut_basis_hint->sf_n_slack >= 0)
          ? pre_cut_basis_hint->sf_n_slack
          : old_n - n_orig - xover_sf.n_surplus - xover_sf.n_artificial;
      const int n_surplus_old = (pre_cut_basis_hint->sf_n_surplus >= 0)
          ? pre_cut_basis_hint->sf_n_surplus
          : xover_sf.n_surplus;
      const int delta_slack = xover_sf.n_slack - n_slack_old;
      const int delta_surplus = xover_sf.n_surplus - n_surplus_old;
      auto adjust_col = [&](int idx) -> int {
        if (idx < n_orig + n_slack_old) return idx;
        if (idx < n_orig + n_slack_old + n_surplus_old) return idx + delta_slack;
        return idx + delta_slack + delta_surplus;
      };
      for (int i = 0; i < old_m_ineq; ++i)
        new_indices[static_cast<size_t>(i)] = adjust_col(extended_hint->indices[static_cast<size_t>(i)]);
      for (int kk = 0; kk < n_new_rows; ++kk) {
        const int r = old_m_ineq + kk;
        const int slack = xover_sf.row_to_slack_col[static_cast<size_t>(r)];
        const int art = xover_sf.row_to_artificial_col[static_cast<size_t>(r)];
        new_indices[static_cast<size_t>(r)] = (slack >= 0) ? slack : (art >= 0) ? art : -1;
      }
      for (int i = old_m_ineq; i < old_m; ++i) {
        const int new_row = i + n_new_rows;
        new_indices[static_cast<size_t>(new_row)] = adjust_col(extended_hint->indices[static_cast<size_t>(i)]);
      }
      extended_hint->indices = std::move(new_indices);
      extended_hint->at_upper = std::move(new_at_upper);
      extended_hint->rows = xo_m;
      extended_hint->cols = xo_n;
      extended_hint->cached_sparse_basis.reset();
      extended_hint->cached_reduced_costs.reset();
      // Validate basis
      bool valid = true;
      std::vector<char> used(static_cast<size_t>(xo_n), 0);
      for (int i = 0; i < xo_m; ++i) {
        int idx = extended_hint->indices[static_cast<size_t>(i)];
        if (idx < 0 || idx >= xo_n) { valid = false; break; }
        if (used[static_cast<size_t>(idx)]) { valid = false; break; }
        used[static_cast<size_t>(idx)] = 1;
      }
      if (!valid) extended_hint.reset();
    }

    std::shared_ptr<SimplexBasis> recovered_hint;
    CrashBasisRecoveryStats recovery_stats;
    if (extended_hint && root.x_relax.size() == n && std::isfinite(root_relax.primal.stats.objective)) {
      SimplexBasis recovered_basis;
      recovery_stats = recover_primal_activity_basis(
          xover_sf, root.x_relax, &root_relax.primal, extended_hint.get(), recovered_basis);
      if (recovery_stats.passes_screen) {
        recovered_hint = std::make_shared<SimplexBasis>(std::move(recovered_basis));
      }
      if (opt.verbose || xo_m > 10000) {
        fmt::print(stderr,
                   "[BC_RELAX] post-cut basis recovery: candidates={} swaps={} part={:.3f} rowdual={:.3f} activity={:.3e} screen={}\n",
                   recovery_stats.candidate_rows,
                   recovery_stats.selected_swaps,
                   recovery_stats.mean_partition_score,
                   recovery_stats.mean_row_dual_score,
                   recovery_stats.mean_activity_score,
                   recovery_stats.passes_screen ? 1 : 0);
      }
    }

    SimplexOptions xo_opt;
    xo_opt.lp_kernel_backend = opt.lp_kernel_backend;
    xo_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
    xo_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    xo_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    xo_opt.verbose = false;
    xo_opt.allow_cold_start = true;
    const SimplexBasis* hint_ptr = recovered_hint ? recovered_hint.get()
        : extended_hint ? extended_hint.get() : nullptr;
    auto xover_res = std::make_shared<SimplexResult>(
        solve_lp_from_sf(xover_sf, xo_opt, hint_ptr));
    [[maybe_unused]] auto t_postcut_solve = std::chrono::steady_clock::now();
    if (opt.verbose || xo_m > 10000) {
      fmt::print(stderr, "[BC_RELAX] simplex {} basis: path={} sf_build={:.1f}ms simplex={:.1f}ms iters={} success={}\n",
              hint_ptr ? "with" : "without",
              recovered_hint ? "post-cut-recovered" : "post-cut",
              std::chrono::duration<double, std::milli>(t_postcut_sf - t_postcut0).count(),
              std::chrono::duration<double, std::milli>(t_postcut_solve - t_postcut_sf).count(),
              xover_res->result.stats.iterations,
              xover_res->result.stats.success ? 1 : 0);
    }

      if (xover_res->result.stats.success && xover_res->result.x.size() == n) {
      root_relax.simplex = xover_res;
      root_relax.primal = xover_res->result;
      root_relax.dual_bound = xover_res->result.stats.objective;
      if (xover_res->form.A.rows() == 0 && xover_sf.A.rows() > 0)
        xover_res->form = std::move(xover_sf);
      if (xover_res->basis.cached_sparse_basis) {
        rebind_sparse_basis_matrix(xover_res->basis.cached_sparse_basis,
                                   xover_res->form.A);
      }
      auto xo_basis = std::make_shared<SimplexBasis>(xover_res->basis);
      xo_basis->sf_n_slack = xover_res->form.n_slack;
      xo_basis->sf_n_surplus = xover_res->form.n_surplus;
      xo_basis->sf_n_artificial = xover_res->form.n_artificial;
      xo_basis->cached_reduced_costs =
          std::make_shared<const Eigen::VectorXd>(xover_res->reduced_costs);
      if (xover_res->basis.cached_sparse_basis) {
        xo_basis->cached_sparse_basis = xover_res->basis.cached_sparse_basis;
      }
	      root_relax.basis_hint = xo_basis;
	      root.x_relax = clamp_to_bounds(xover_res->result.x, root.lb, root.ub);
	      root.basis_hint = root_relax.basis_hint;
	      root.bound = root_relax.dual_bound;
	      refresh_root_redcost_lurking_bounds("root_postcut_crossover");
	    }
  }

  // Generic medium-size IPM root polish: when the root cut path adds no cuts,
  // one simplex root solve can cheaply recover a strong root vertex and basis.
  // This is gated only by problem size and root state, not model semantics.
  if (opt.use_ipm_root && out.bc_stats.cuts_added == 0 &&
      presolved_m > 5000 && presolved_m <= 10000) {
    SimplexOptions handoff_opt;
    handoff_opt.lp_kernel_backend = opt.lp_kernel_backend;
    handoff_opt.max_iter = std::max(opt.max_lp_iter * 10, 4000);
    handoff_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    handoff_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    handoff_opt.verbose = false;
    handoff_opt.allow_cold_start = true;

    auto handoff_res = std::make_shared<SimplexResult>(
        solve_lp_with_basis(base_lp, handoff_opt, nullptr));
    out.bc_stats.lp_solves += 1;

    if (handoff_res->result.stats.success && handoff_res->result.x.size() == n &&
        std::isfinite(handoff_res->result.stats.objective)) {
      root_relax.simplex = handoff_res;
      root_relax.primal = handoff_res->result;
      root_relax.dual_bound = handoff_res->result.stats.objective;
      if (handoff_res->basis.cached_sparse_basis) {
        rebind_sparse_basis_matrix(handoff_res->basis.cached_sparse_basis,
                                   handoff_res->form.A);
      }

      auto handoff_basis = std::make_shared<SimplexBasis>(handoff_res->basis);
      handoff_basis->sf_n_slack = handoff_res->form.n_slack;
      handoff_basis->sf_n_surplus = handoff_res->form.n_surplus;
      handoff_basis->sf_n_artificial = handoff_res->form.n_artificial;
      handoff_basis->cached_reduced_costs =
          std::make_shared<const Eigen::VectorXd>(handoff_res->reduced_costs);
      if (handoff_res->basis.cached_sparse_basis) {
        handoff_basis->cached_sparse_basis = handoff_res->basis.cached_sparse_basis;
      }

	      root_relax.basis_hint = handoff_basis;
	      root.x_relax = clamp_to_bounds(handoff_res->result.x, root.lb, root.ub);
	      root.basis_hint = handoff_basis;
	      root.bound = root_relax.dual_bound;
	      refresh_root_redcost_lurking_bounds("root_ipm_handoff_simplex");
	    }
  }

  // The root cut rounds create SparseBasis objects that reference local
  // StandardFormLP matrices (cut_sf) which are now destroyed. Clear the
  // dangling cached_sparse_basis before building the fresh base_sf.
  if (root.basis_hint && root.basis_hint->cached_sparse_basis) {
    root.basis_hint->cached_sparse_basis.reset();
  }
  if (root_relax.basis_hint && root_relax.basis_hint->cached_sparse_basis) {
    root_relax.basis_hint->cached_sparse_basis.reset();
  }
  if (root_relax.simplex) {
    root_relax.simplex->basis.cached_sparse_basis.reset();
  }
  root_sbasis.reset();

  [[maybe_unused]] auto t_final_sf0 = std::chrono::steady_clock::now();
  // ──────────────────────────────────────────────────────────────────────
  // HiGHS-aligned PersistentLPState (B redesign, U.7.116)
  //
  // Replaces the local `base_sf_ptr` + `ensure_base_sf` lambda. The state
  // owns the cut-free `StandardFormLP` (`base_sf`) and two basis snapshots:
  //   - `base_basis_`  ≡ HiGHS' firstrootbasis (pre-cut, dimension-matches
  //                       `base_sf`). Pump and dive read from this so they
  //                       never face a row-count mismatch and never fall
  //                       back to IPM cold-start.
  //   - `cut_basis_`   the latest basis from the cut-augmented LP. Used by
  //                       xpool re-solve and tree-node warm starts.
  //
  // `pre_cut_basis_hint` (captured earlier in this scope, just before the
  // cut loop ran) is the HiGHS firstrootbasis analog. We hand it to the
  // persistent state so downstream phases can grab it through a single
  // typed entry point.
  // ──────────────────────────────────────────────────────────────────────
  PersistentLPState persistent_lp_state;
  auto ensure_base_sf = [&]() -> StandardFormLP& {
    return persistent_lp_state.ensure_base_sf(base_lp);
  };
  if (pre_cut_basis_hint) {
    persistent_lp_state.set_base_basis(pre_cut_basis_hint);
  }
  if (root.basis_hint) {
    persistent_lp_state.set_cut_basis(root.basis_hint);
  }
  [[maybe_unused]] auto t_cuts_done = std::chrono::steady_clock::now();
  if (opt.verbose) {
    fmt::print(stderr, "[B&C ROOT] root_lp={:.1f}ms binv={:.1f}ms cuts={:.1f}ms (postcut_overhead={:.1f}ms final_sf={:.1f}ms)\n",
            std::chrono::duration<double, std::milli>(t_root1 - t_root0).count(),
            std::chrono::duration<double, std::milli>(t_binv1 - t_root1).count(),
            std::chrono::duration<double, std::milli>(t_cuts_done - t_binv1).count(),
            std::chrono::duration<double, std::milli>(t_final_sf0 - t_postcut0).count(),
            std::chrono::duration<double, std::milli>(t_cuts_done - t_final_sf0).count());
  }

	  SolutionPool solution_pool(opt.solution_pool_size);

	      if (out.bc_stats.cuts_added > 0) {
        if (strict_highs_root_fixed_point) {
          // The strict root fixed-point has already managed transformed rows
          // through the private HiGHS-style cutpool state machine
          // (addCut -> separate -> addCuts/resolve -> performAging).  Re-seeding
          // those root rows into the native global pool here would give them a
          // second available/LP-visible lifecycle that HiGHS does not have.
        } else {
		    const int orig_rows = static_cast<int>(pre_cut_root_lp.A.rows());
		    const int total_rows = static_cast<int>(base_lp.A.rows());
	    Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_seed = base_lp.A;
    int seeded_root_lp_cutpool_rows = 0;
    for (int r = orig_rows; r < total_rows; ++r) {
      Eigen::SparseVector<double> row_sparse(n);
      int nnz = 0;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_seed, r); it; ++it) {
        if (it.col() < n) ++nnz;
      }
      row_sparse.reserve(nnz);
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_seed, r); it; ++it) {
        if (it.col() < n) {
          row_sparse.insertBack(it.col()) = it.value();
        }
      }
      bool new_domain_source = false;
      if (cut_pool.add(std::move(row_sparse), base_lp.b[r],
                       /*domain_only=*/false, &new_domain_source) &&
          new_domain_source) {
        ++seeded_root_lp_cutpool_rows;
      }
    }
    if (seeded_root_lp_cutpool_rows > 0) {
      note_global_domain_learning(
          static_cast<std::uint64_t>(seeded_root_lp_cutpool_rows));
      if (static_cast<std::size_t>(cut_pool.size()) <=
          last_cutpool_clique_scan_size) {
        last_cutpool_clique_scan_size = 0;
      }
	      (void)synchronize_root_source_cutpool_domain_sources(
	          "root_lp_cut_seed");
	    }
        }
	  }

  bool has_incumbent = false;
  double incumbent_obj = kInf;
  Eigen::VectorXd incumbent_x = Eigen::VectorXd::Zero(n);
  std::vector<ObjectiveCutoffEvent> objective_cutoff_events;
  double objective_cutoff_rest_lb = kInf;

  std::unique_ptr<ConcurrentTreeState> cts;
  bool concurrent_tree_launched = false;
  const bool trace_root_subsolves = opt.verbose;

  std::vector<RootSubsolveTrace> root_subsolve_traces;
  std::string root_incumbent_source = "none";
  bool root_gap_certificate_ready = true;
  std::string root_gap_certificate_pending_source;
  const bool trace_bc_timeline = bc_env_flag_enabled("MIPSOLVERS_BC_TIMELINE");

  auto bc_elapsed_ms = [&]() -> double {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  };

  auto note_bound_event = [&](double primal_bound, double dual_bound,
                              bool retain_incumbent) {
    const double primal = std::isfinite(primal_bound)
                              ? primal_bound
                              : std::numeric_limits<double>::quiet_NaN();
    const double dual = std::isfinite(dual_bound)
                            ? dual_bound
                            : std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(primal) && !std::isfinite(dual)) return;

    BCBoundEvent event;
    event.time_sec = 0.001 * bc_elapsed_ms();
    event.primal_bound = primal;
    event.dual_bound = dual;
    event.nodes = out.bc_stats.nodes_explored;
    event.lp_solves = out.bc_stats.lp_solves;
    if (retain_incumbent && incumbent_x.size() == n) {
      event.incumbent = incumbent_x;
    }

    if (!out.bc_stats.bound_events.empty()) {
      const auto& previous = out.bc_stats.bound_events.back();
      const bool same_primal =
          (!std::isfinite(previous.primal_bound) && !std::isfinite(primal)) ||
          previous.primal_bound == primal;
      const bool same_dual =
          (!std::isfinite(previous.dual_bound) && !std::isfinite(dual)) ||
          previous.dual_bound == dual;
      if (same_primal && same_dual && !retain_incumbent) return;
    }
    out.bc_stats.bound_events.push_back(std::move(event));
  };

  auto note_incumbent_update = [&](int depth, double bound_snapshot) {
    ++out.bc_stats.incumbent_updates;
    const int node_now = out.bc_stats.nodes_explored;
    const int lp_now = out.bc_stats.lp_solves;
    if (out.bc_stats.first_incumbent_node < 0) {
      out.bc_stats.first_incumbent_node = node_now;
      out.bc_stats.first_incumbent_lp_solves = lp_now;
    }
    out.bc_stats.last_incumbent_node = node_now;
    out.bc_stats.last_incumbent_depth = depth;
    out.bc_stats.last_incumbent_lp_solves = lp_now;
    if (std::isfinite(bound_snapshot)) {
      out.bc_stats.best_bound_at_last_incumbent = bound_snapshot;
    }
    note_bound_event(incumbent_obj, bound_snapshot,
                     /*retain_incumbent=*/true);
    if (trace_bc_timeline) {
      const double denom = std::max(1.0, std::abs(incumbent_obj));
      const double gap =
          std::isfinite(bound_snapshot)
              ? std::max(0.0, (incumbent_obj - bound_snapshot) / denom)
              : kInf;
      fmt::print(stderr,
                 "[B&C-TL] phase=incumbent time_ms={:.0f} node={} lp={} "
                 "depth={} ub={:.10g} lb_snapshot={:.10g} gap={:.6g} "
                 "updates={}\n",
                 bc_elapsed_ms(), node_now, lp_now, depth, incumbent_obj,
                 bound_snapshot, gap, out.bc_stats.incumbent_updates);
    }
  };

	  auto synchronize_cutpool_clique_objective_sources =
	      [&](const char* source) -> std::uint64_t {
    if (last_cutpool_clique_scan_size >
        static_cast<std::size_t>(cut_pool.size())) {
      last_cutpool_clique_scan_size = 0;
    }
    std::uint64_t edges_added_total = 0;
    std::uint64_t generated_edges_total = 0;
    std::uint64_t rows_scanned = 0;
    std::uint64_t rows_with_literals = 0;
    std::uint64_t rows_with_pair_excess = 0;
    double max_pair_excess = 0.0;
    double best_pair_margin = -1e100;
    const auto& cuts = cut_pool.cuts();
    for (std::size_t i = last_cutpool_clique_scan_size; i < cuts.size(); ++i) {
      const PoolCut& cut = cuts[i];
      if (cut.coeff.size() != n || cut.coeff.nonZeros() <= 0 ||
          !std::isfinite(cut.rhs)) {
        continue;
      }
      CliqueTable::CutCliqueExtractionStats stats;
      const std::size_t added = clique_table.add_literal_edges_from_cut(
          base_lp, cut.coeff, cut.rhs, 512, &root_implied_integer_cols,
          &implication_graph, &stats, nullptr);
      edges_added_total += static_cast<std::uint64_t>(added);
      generated_edges_total += stats.literal_edges_generated;
      rows_scanned += stats.rows_scanned;
      rows_with_literals += stats.rows_with_binary_literals;
      rows_with_pair_excess += stats.rows_with_pair_excess;
      max_pair_excess = std::max(max_pair_excess, stats.max_pair_excess);
      best_pair_margin = std::max(best_pair_margin, stats.best_pair_margin);
    }
    last_cutpool_clique_scan_size = cuts.size();
    if (edges_added_total == 0) {
      if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) &&
          rows_scanned > 0) {
        fmt::print(stderr,
                   "[B&C-CUTPOOL-CLIQUE] source={} scan={} litRows={} "
                   "pairRows={} genEdges={} added=0 bestMargin={:.6g} "
                   "maxExcess={:.6g} clqEdges={}\n",
                   source != nullptr ? source : "cutpool", rows_scanned,
                   rows_with_literals, rows_with_pair_excess,
                   generated_edges_total, best_pair_margin, max_pair_excess,
                   clique_table.n_literal_edges());
      }
      return 0;
    }

	    note_global_domain_learning(edges_added_total);
	    note_objective_artifact_learning(edges_added_total);
	    const auto refresh_stats =
	        refresh_objective_domain_sources(source, /*force=*/true);
	    const std::uint64_t published_total =
	        refresh_stats.total_published();
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C-CUTPOOL-CLIQUE] source={} scan={} litRows={} "
	                 "pairRows={} genEdges={} added={} pub={}/{} "
                 "bestMargin={:.6g} maxExcess={:.6g} objParts={} "
                 "clqEdges={} arcs={}\n",
	                 source != nullptr ? source : "cutpool", rows_scanned,
	                 rows_with_literals, rows_with_pair_excess,
	                 generated_edges_total, edges_added_total,
	                 refresh_stats.published_implications,
	                 refresh_stats.published_clique_edges,
	                 best_pair_margin, max_pair_excess,
	                 objective_propagation.partitions.size(),
	                 clique_table.n_literal_edges(), implication_graph.size());
	    }
	    return edges_added_total + published_total;
  };

	  auto add_cut_to_global_domain_pool =
	      [&](Eigen::SparseVector<double> coeff, double rhs,
	          const char* source, bool domain_only = false,
	          ValidityScope validity_scope = ValidityScope::GlobalCut) -> bool {
    if (validity_scope != ValidityScope::GlobalCut) {
      ++out.bc_stats.global_cutpool_scope_rejections;
      if (opt.verbose || trace_bc_timeline) {
        fmt::print(stderr,
                   "[B&C-CUTPOOL-DOMAIN] source={} rejected_scope={}\n",
                   source != nullptr ? source : "cutpool",
                   validity_scope == ValidityScope::LocalNodeCut
                       ? "LocalNodeCut"
                       : "LazyConstraint");
      }
      return false;
    }
    if (coeff.size() != n || coeff.nonZeros() <= 0 || !std::isfinite(rhs)) {
      return false;
    }
    bool new_domain_source = false;
    const bool added =
        cut_pool.add(std::move(coeff), rhs, domain_only, &new_domain_source,
                     validity_scope);
    if (added) {
      if (new_domain_source) {
        note_global_domain_learning();
        if (static_cast<std::size_t>(cut_pool.size()) <=
            last_cutpool_clique_scan_size) {
          last_cutpool_clique_scan_size = 0;
        }
        (void)synchronize_cutpool_clique_objective_sources(source);
      }
      if (opt.verbose || trace_bc_timeline) {
        fmt::print(stderr,
                   "[B&C-CUTPOOL-DOMAIN] source={} size={} domain_only={} "
                   "new_source={} epoch={}\n",
                   source != nullptr ? source : "cutpool", cut_pool.size(),
                   domain_only ? 1 : 0, new_domain_source ? 1 : 0,
                   current_domain_learning_epoch());
      }
    }
	    return added;
	  };
	  bool root_transformed_cutpool_in_global_domain_pool = false;
	  auto seed_root_transformed_cutpool_domain_rows = [&]() -> int {
	    int added = 0;
	    int duplicates = 0;
	    int provisional_rows = 0;
	    int domain_only_rows = 0;
	    int lp_visible_rows = 0;
	    for (int i = 0; i < static_cast<int>(root_transformed_cutpool.size()); ++i) {
	      const PoolCut& cut = root_transformed_cutpool[static_cast<std::size_t>(i)];
	      if (cut.coeff.size() != n || cut.coeff.nonZeros() <= 0 ||
	          !std::isfinite(cut.rhs)) {
	        continue;
	      }
	      const bool alive =
	          i >= static_cast<int>(root_transformed_cut_alive.size()) ||
	          root_transformed_cut_alive[static_cast<std::size_t>(i)] != 0;
	      const bool in_root_lp =
	          i < static_cast<int>(root_transformed_cut_in_lp.size()) &&
	          root_transformed_cut_in_lp[static_cast<std::size_t>(i)] != 0;
	      if (!highs_style_cutpool_row_tree_owned(alive, in_root_lp)) {
	        ++provisional_rows;
	        continue;
	      }
	      Eigen::SparseVector<double> coeff = cut.coeff;
		      const bool domain_only = true;
	      if (add_cut_to_global_domain_pool(
                  std::move(coeff), cut.rhs,
                  "root_transformed_cutpool_seed", domain_only)) {
	        ++added;
	        if (domain_only) {
	          ++domain_only_rows;
	        } else {
	          ++lp_visible_rows;
	        }
	      } else {
	        ++duplicates;
	      }
	    }
	    root_transformed_cutpool_in_global_domain_pool =
	        !root_transformed_cutpool.empty();
	    if (added > 0) {
	      (void)refresh_objective_domain_sources(
	          "root_transformed_cutpool_seed", /*force=*/true);
	    }
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
	        bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C-CUTPOOL-DOMAIN] source=root_transformed_cutpool_seed "
	                 "poolRows={} added={} dup={} provisional={} "
	                 "domain_only={} lp_visible={} cutpool={}\n",
	                 static_cast<int>(root_transformed_cutpool.size()), added,
	                 duplicates, provisional_rows, domain_only_rows,
	                 lp_visible_rows,
	                 cut_pool.size());
	    }
	    return added;
	  };
	  (void)seed_root_transformed_cutpool_domain_rows();
	  (void)synchronize_cutpool_clique_objective_sources("root_cutpool_seed");
  auto note_root_subsolve = [&](const char* name, double ms, bool success,
                                bool incumbent_hit = false,
                                int lp_solves = 0) {
    note_root_subsolve_trace(root_subsolve_traces, trace_root_subsolves, name,
                             ms, success, incumbent_hit, lp_solves);
  };

	  bool incumbent_proof_validated = false;
	  double incumbent_proof_validated_obj = kInf;
	  auto invalidate_incumbent_proof_validation = [&]() {
	    incumbent_proof_validated = false;
	    incumbent_proof_validated_obj = kInf;
	  };
	  auto validate_root_candidate_for_proof =
	      [&](const char* source, const Eigen::VectorXd& x) -> bool {
	    if (x.size() != n) {
	      if (opt.verbose || trace_bc_timeline) {
	        fmt::print(stderr,
	                   "[B&C-INC-GATE] rejected source={} reason=size "
	                   "actual={} expected={}\n",
	                   source != nullptr ? source : "incumbent",
	                   static_cast<int>(x.size()), n);
	      }
	      return false;
	    }
	    const auto reduced_validation =
	        validate_incumbent_solution(base_lp, x, kNodeFeasibilityTol);
	    if (!reduced_validation.ok()) {
	      if (opt.verbose || trace_bc_timeline) {
	        fmt::print(stderr,
	                   "[B&C-INC-GATE] rejected source={} {}\n",
	                   source != nullptr ? source : "incumbent",
	                   format_incumbent_validation_failure(
	                       "reduced-proof-gate", base_lp,
	                       reduced_validation));
	      }
	      return false;
	    }
	    if (!satisfies_with_bounds(base_lp, x, root.lb, root.ub,
	                               kNodeFeasibilityTol)) {
	      if (opt.verbose || trace_bc_timeline) {
	        fmt::print(stderr,
	                   "[B&C-INC-GATE] rejected source={} "
	                   "reason=root_domain_bounds\n",
	                   source != nullptr ? source : "incumbent");
	      }
	      return false;
	    }
	    return true;
	  };
	  auto current_incumbent_valid_for_proof = [&](const char* source) -> bool {
	    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
	        incumbent_x.size() != n) {
	      invalidate_incumbent_proof_validation();
	      return false;
	    }
	    if (incumbent_proof_validated &&
	        std::abs(incumbent_obj - incumbent_proof_validated_obj) <=
	            std::max(1e-9, 1e-12 * std::max(1.0, std::abs(incumbent_obj)))) {
	      return true;
	    }
	    const bool ok = validate_root_candidate_for_proof(source, incumbent_x);
	    incumbent_proof_validated = ok;
	    incumbent_proof_validated_obj = ok ? incumbent_obj : kInf;
	    return ok;
	  };

		  auto root_domain_signature = [&]() -> std::size_t {
		    std::size_t h = 0;
		    auto mix = [&](std::size_t v) {
		      h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
		    };
		    auto quantized_bound = [](double value) -> long long {
		      if (!std::isfinite(value)) {
		        return value < 0.0 ? std::numeric_limits<long long>::min()
		                           : std::numeric_limits<long long>::max();
		      }
		      const double scaled = value * 1e6;
		      if (scaled <= static_cast<double>(std::numeric_limits<long long>::min())) {
		        return std::numeric_limits<long long>::min() + 1;
		      }
		      if (scaled >= static_cast<double>(std::numeric_limits<long long>::max())) {
		        return std::numeric_limits<long long>::max() - 1;
		      }
		      return static_cast<long long>(std::llround(scaled));
		    };
		    const int m = root.lb.size() == root.ub.size()
		                      ? static_cast<int>(root.lb.size())
		                      : 0;
		    mix(std::hash<int>{}(m));
		    for (int j = 0; j < m; ++j) {
		      mix(std::hash<int>{}(j));
		      mix(std::hash<long long>{}(quantized_bound(root.lb[j])));
		      mix(std::hash<long long>{}(quantized_bound(root.ub[j])));
		    }
		    return h;
		  };

		  auto current_incumbent_valid_for_cutoff_propagation =
		      [&](const char* source) -> bool {
		    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
		        incumbent_x.size() != n) {
		      return false;
		    }
		    const auto reduced_validation =
		        validate_incumbent_solution(base_lp, incumbent_x,
		                                     kNodeFeasibilityTol);
		    if (!reduced_validation.ok()) {
		      if (opt.verbose || trace_bc_timeline) {
		        fmt::print(stderr,
		                   "[B&C-INC-GATE] rejected source={} {}\n",
		                   source != nullptr ? source : "incumbent_cutoff",
		                   format_incumbent_validation_failure(
		                       "cutoff-propagation-gate", base_lp,
		                       reduced_validation));
		      }
		      return false;
		    }
		    return true;
		  };

		  double last_objective_cutoff_artifact_obj = kInf;
		  std::uint64_t last_objective_cutoff_artifact_domain_epoch =
		      std::numeric_limits<std::uint64_t>::max();
		  std::size_t last_objective_cutoff_artifact_root_signature =
		      std::numeric_limits<std::size_t>::max();
		  auto consume_objective_cutoff_artifacts = [&](const char* reason) -> std::uint64_t {
		    if (!use_objective_propagation || !has_incumbent ||
		        !std::isfinite(incumbent_obj) || base_lp.sense != Sense::Minimize ||
		        root.lb.size() != n || root.ub.size() != n) {
		      return 0;
		    }
		    if (!current_incumbent_valid_for_cutoff_propagation(reason)) {
		      return 0;
		    }
	    const double upper_limit =
	        highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                         opt.int_tol);
	    if (!std::isfinite(upper_limit)) {
	      return 0;
	    }
		    const double improve_tol =
	        std::max(1e-7, 1e-10 * std::max(1.0, std::abs(incumbent_obj)));
	    const std::uint64_t domain_epoch = current_domain_learning_epoch();
	    const std::size_t domain_signature = root_domain_signature();
	    const bool cutoff_improved =
	        !std::isfinite(last_objective_cutoff_artifact_obj) ||
	        incumbent_obj < last_objective_cutoff_artifact_obj - improve_tol;
	    const bool domain_changed =
	        domain_epoch != last_objective_cutoff_artifact_domain_epoch ||
	        domain_signature != last_objective_cutoff_artifact_root_signature;
	    if (!cutoff_improved && !domain_changed) {
	      return 0;
	    }

	    const auto clique_stats = extract_objective_cutoff_cliques(
        base_lp, root_implied_integer_cols, root.lb, root.ub, upper_limit,
        clique_table, opt.int_tol, &objective_propagation,
        &conflict_pool, cts ? &cts->shared_conflicts : nullptr);
	    const auto stats = extract_objective_cutoff_artifacts(
        base_lp, root_implied_integer_cols, root.lb, root.ub, upper_limit,
        implication_graph, clique_table, opt.int_tol,
        &objective_propagation, &conflict_pool,
        cts ? &cts->shared_conflicts : nullptr);
	    if (stats.implications_added == 0 && stats.clique_edges_added == 0 &&
	        stats.conflict_clauses_added == 0 &&
	        clique_stats.clique_edges_added == 0 &&
	        clique_stats.conflict_clauses_added == 0) {
	      last_objective_cutoff_artifact_obj = incumbent_obj;
	      last_objective_cutoff_artifact_domain_epoch = current_domain_learning_epoch();
	      last_objective_cutoff_artifact_root_signature = root_domain_signature();
	      record_conformance_source("incumbent_objective_source_none");
      if (bc_conf_diag_enabled()) {
	      fmt::print(stderr,
                   "[B&C OBJ-CLIQUE] {} none cand={} eventCand={} cap={:.6g} "
                   "raw_lb={:.2f} max_delta={:.6g} cutClqCand={} "
                   "cutClqPairs={} cutClqConf={} cutClqAdded={}\n",
                   reason != nullptr ? reason : "incumbent",
                   stats.candidates, stats.implied_event_candidates,
                   stats.cutoff_capacity,
                   stats.raw_objective_lower, stats.max_delta,
                   clique_stats.candidates, clique_stats.pair_tests,
                   clique_stats.pair_conflicts,
                   clique_stats.clique_edges_added);
      }
      return 0;
    }

    out.bc_stats.rc_binary_implications_learned +=
        stats.implications_added;
    const std::uint64_t extracted_objective_artifacts =
        stats.implications_added + stats.clique_edges_added +
        stats.conflict_clauses_added + clique_stats.clique_edges_added +
        clique_stats.conflict_clauses_added;
	    out.bc_stats.cutoff_conflict_clauses_learned +=
	        stats.conflict_clauses_added + clique_stats.conflict_clauses_added;
	    note_global_domain_learning(extracted_objective_artifacts);
	    note_objective_artifact_learning(extracted_objective_artifacts);
	    (void)refresh_objective_domain_sources(reason, /*force=*/true);
		    last_objective_cutoff_artifact_obj = incumbent_obj;
		    last_objective_cutoff_artifact_domain_epoch = current_domain_learning_epoch();
		    last_objective_cutoff_artifact_root_signature = root_domain_signature();
	    record_conformance_source("incumbent_objective_source");
    if (bc_conf_diag_enabled()) {
      fmt::print(stderr,
                 "[B&C OBJ-CLIQUE] {} cand={} eventCand={} conflicts={} "
                 "confQ={} unary={} pair={} impl={} clique_edges={} "
                 "cutClqCand={} cutClqPairs={} cutClqConf={} "
                 "cutClqAdded={} cutClqClauses={} cap={:.6g} raw_lb={:.2f} "
                 "obj_parts={} obj_terms={} arcs={}\n",
                 reason != nullptr ? reason : "incumbent",
                 stats.candidates, stats.implied_event_candidates,
	                 stats.conflicts_seen, stats.conflict_clauses_added,
	                 stats.unary_conflicts_added, stats.pair_conflicts_added,
	                 stats.implications_added, stats.clique_edges_added,
	                 clique_stats.candidates, clique_stats.pair_tests,
	                 clique_stats.pair_conflicts, clique_stats.clique_edges_added,
                 clique_stats.conflict_clauses_added,
                 stats.cutoff_capacity, stats.raw_objective_lower,
                 objective_propagation.partitions.size(),
	    out.bc_stats.objective_clique_terms,
	                 implication_graph.size());
	    }
	    return extracted_objective_artifacts;
	  };

		  std::function<void(const char*)> root_incumbent_fixed_point_hook;
		  auto adopt_root_incumbent = [&](const char* source,
		                                  const Eigen::VectorXd& candidate_x,
		                                  double candidate_obj) -> bool {
	    (void)candidate_obj;
	    const double actual_obj =
	        objective_value(base_lp.c, candidate_x, base_lp.sense);
	    if (!std::isfinite(actual_obj)) {
	      return false;
	    }
	    if (!incumbent_quality_acceptable(actual_obj, root.bound)) {
	      if (opt.verbose) {
	        fmt::print(stderr,
	                   "[B&C] root incumbent from {} rejected by quality gate: "
	                   "obj={:.6g} vs root bound={:.6g}\n",
	                   source, actual_obj, root.bound);
	      }
	      return false;
	    }
	    if (has_incumbent && !(actual_obj + 1e-9 < incumbent_obj)) {
	      return false;
	    }
	    if (!validate_root_candidate_for_proof(source, candidate_x)) {
	      return false;
	    }
	    has_incumbent = true;
	    incumbent_x = candidate_x;
	    incumbent_obj = actual_obj;
	    note_incumbent_cutoff_learning();
	    incumbent_proof_validated = true;
	    incumbent_proof_validated_obj = incumbent_obj;
	    solution_pool.add(candidate_x, actual_obj);
	    if (cts) {
	      cts->shared_inc.try_update(candidate_x, actual_obj);
	      cts->shared_sp.add(candidate_x, actual_obj);
	    }
		    root_incumbent_source = source;
		    note_incumbent_update(/*depth=*/0, root.bound);
		    consume_objective_cutoff_artifacts(source);
		    // An audited incumbent equal to the rigorous root bound is already an
		    // exact optimality certificate; incumbent-driven closure cannot add
		    // proof value. See Achterberg (2007), Sec. 3.1, and the mismatch
		    // derivation in docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
		    const bool exact_root_certificate =
		        std::isfinite(root.bound) &&
		        incumbent_obj - root.bound <=
		            1e-9 * std::max(1.0, std::abs(incumbent_obj));
        // Audited primal + rigorous root bound already certify the requested
        // relative gap. docs/solvers.md, Fixed-Integer Root Repair Kernel and Budget.
        const bool requested_root_certificate =
            !opt.auto_highs_root_pipeline && !opt.require_tree_exhaustion_certificate &&
            std::isfinite(root.bound) && std::isfinite(opt.gap_tol) && opt.gap_tol >= 0.0 &&
            std::max(0.0, incumbent_obj-root.bound) /
                std::max(1.0, std::abs(incumbent_obj)) <= opt.gap_tol;
		    if (root_incumbent_fixed_point_hook && !exact_root_certificate && !requested_root_certificate) {
		      root_incumbent_fixed_point_hook(source);
		    }
		    return true;
		  };

  auto root_incumbent_match_tol = [&]() -> double {
    return std::max(1e-6, 1e-10 * std::max(1.0, std::abs(incumbent_obj)));
  };

		  auto root_bound_gap_certifies_incumbent = [&]() -> bool {
		    if (opt.require_tree_exhaustion_certificate ||
		        !has_incumbent || !std::isfinite(incumbent_obj) ||
		        !std::isfinite(root.bound) || !std::isfinite(opt.gap_tol) ||
		        opt.gap_tol < 0.0) {
		      return false;
		    }
		    if (!current_incumbent_valid_for_proof("root_bound_gap")) {
		      return false;
		    }
		    const double rel_gap =
		        std::max(0.0, (incumbent_obj - root.bound) /
		                            std::max(1.0, std::abs(incumbent_obj)));
	    return rel_gap <= opt.gap_tol;
	  };

	  // Exact-optimality predicate usable regardless of
	  // require_tree_exhaustion_certificate: root.bound is a rigorous lower bound
	  // on the optimum, so a valid incumbent meeting it (gap exactly 0) is proven
	  // optimal and all further proof / bound-lifting certificate work is
	  // redundant.  This lets root-closed instances stop immediately instead of
	  // grinding the certificate tail to the time limit.
	  auto incumbent_proven_optimal_exact = [&]() -> bool {
	    return has_incumbent && std::isfinite(incumbent_obj) &&
	           std::isfinite(root.bound) &&
	           incumbent_obj - root.bound <=
	               1e-9 * std::max(1.0, std::abs(incumbent_obj));
	  };

	  auto publish_root_bound_gap_certificate = [&](const char* source) -> bool {
	    if (!root_bound_gap_certifies_incumbent()) {
	      return false;
	    }
	    const bool log_transition =
	        !root_gap_certificate_ready ||
	        !root_gap_certificate_pending_source.empty();
	    root_gap_certificate_ready = true;
	    root_gap_certificate_pending_source.clear();
	    if (log_transition &&
	        (trace_bc_timeline || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
	         opt.verbose)) {
	      fmt::print(stderr,
	                 "[B&C ROOT-CERT] certified source=root_lp_bound "
	                 "candidate_source={} incumbent={:.10g} root_bound={:.10g}\n",
	                 source != nullptr ? source : "root_local_frontier",
	                 incumbent_obj, root.bound);
	    }
	    return true;
	  };

	  auto require_root_gap_frontier_certificate = [&](const char* source) {
	    if (publish_root_bound_gap_certificate(source)) {
	      return;
	    }
	    root_gap_certificate_ready = false;
	    root_gap_certificate_pending_source = source != nullptr ? source : "root_local_frontier";
    if (trace_bc_timeline || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
        opt.verbose) {
      fmt::print(stderr,
                 "[B&C ROOT-CERT] pending source={} incumbent={:.10g} "
                 "root_bound={:.10g}\n",
                 root_gap_certificate_pending_source, incumbent_obj,
                 root.bound);
    }
  };

	  auto certify_root_gap_frontier_incumbent =
	      [&](const char* source, double certified_obj) {
	    if (publish_root_bound_gap_certificate(source)) {
	      return;
	    }
	    if (root_gap_certificate_ready || !has_incumbent ||
	        !std::isfinite(certified_obj) || !std::isfinite(incumbent_obj)) {
	      return;
	    }
    if (certified_obj <= incumbent_obj + root_incumbent_match_tol()) {
      root_gap_certificate_ready = true;
      if (trace_bc_timeline || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
          opt.verbose) {
        fmt::print(stderr,
                   "[B&C ROOT-CERT] certified source={} pending={} "
                   "obj={:.10g} incumbent={:.10g}\n",
                   source != nullptr ? source : "frontier",
                   root_gap_certificate_pending_source, certified_obj,
                   incumbent_obj);
      }
      root_gap_certificate_pending_source.clear();
    }
  };

  // A disabled root pipeline has no pending polishing obligation; same gap
  // certificate contract as above, independent of the selected LP kernel.
  bool root_primal_polish_complete = !requested_vendored_highs_lp || !opt.auto_highs_root_pipeline;
  auto root_proof_tail_polish_required = [&]() -> bool {
    return requested_vendored_highs_lp && opt.auto_highs_root_pipeline &&
           opt.require_tree_exhaustion_certificate &&
           !root_primal_polish_complete && root_frac_bin_count > 0 &&
           std::isfinite(root.bound);
  };

	  // ── Helper: check if incumbent is within gap_tol of root bound ──
		  // Used to skip expensive root heuristics once gap is already closed.
		  // When concurrent tree is running, also check tree's SharedIncumbent.
		  auto root_gap_closed = [&]() -> bool {
		    (void)publish_root_bound_gap_certificate("root_gap_closed");
		    if (!root_primal_polish_complete) {
		      return false;
		    }
	    if (!root_gap_certificate_ready) {
	      return false;
    }
    double best_obj = has_incumbent ? incumbent_obj : kInf;
    // Pull better incumbent from concurrent tree threads if available.
    if (cts && cts->shared_inc.has()) {
      double tree_obj = cts->shared_inc.get_obj();
      if (tree_obj < best_obj) best_obj = tree_obj;
    }
    if (!std::isfinite(best_obj) || !std::isfinite(root.bound)) return false;
    const double denom = std::max(1.0, std::abs(best_obj));
    const double rel_gap = (best_obj - root.bound) / denom;
    if (!opt.require_tree_exhaustion_certificate &&
        rel_gap <= opt.gap_tol) {
      // Signal concurrent tree to stop immediately — gap is closed.
      if (cts && !cts->should_stop.load(std::memory_order_relaxed)) {
        cts->should_stop.store(true, std::memory_order_release);
      }
      return true;
    }
    return false;
  };

  auto incumbent_root_rel_gap = [&]() -> double {
    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
        !std::isfinite(root.bound)) {
      return kInf;
    }
    return (incumbent_obj - root.bound) /
           std::max(1.0, std::abs(incumbent_obj));
  };

	  auto incumbent_upper_limit = [&]() -> double {
	    return highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                             opt.int_tol);
	  };
	  auto incumbent_upper_limit_prune_tol = [&](double upper_limit) -> double {
	    return std::max({1e-9, opt.lp_tol,
	                     kIncumbentPruneTol *
	                         std::max(1.0, std::abs(upper_limit))});
	  };

		  auto root_incumbent_good_enough = [&]() -> bool {
		    (void)publish_root_bound_gap_certificate("root_incumbent_good_enough");
		    if (!root_primal_polish_complete) {
		      return false;
		    }
	    if (!root_gap_certificate_ready) {
	      return false;
    }
    return has_incumbent && std::isfinite(incumbent_obj) &&
           std::isfinite(root.bound) &&
           !opt.require_tree_exhaustion_certificate &&
           incumbent_root_rel_gap() <= opt.gap_tol;
  };

  auto trace_root_primal_gate =
      [&](const char* phase, const char* decision, const char* detail = "") {
    if (!trace_bc_timeline && bc_env_options().value("MIPSOLVERS_BC_CONF") == nullptr &&
        !opt.verbose) {
      return;
    }
	    const double gate_gap = incumbent_root_rel_gap();
		    const bool gate_cert_ready =
		        root_gap_certificate_ready || root_bound_gap_certifies_incumbent();
		    const bool gate_closed = has_incumbent && std::isfinite(gate_gap) &&
		                             root_primal_polish_complete &&
		                             gate_cert_ready &&
		                             gate_gap <= opt.gap_tol;
    fmt::print(stderr,
               "[B&C ROOT-HEUR] phase={} decision={} has_inc={} "
               "gap={:.6g} proof_gap={:.6g} tree_good_gap={:.6g} root_closed={} "
               "low_frac={} frac_bin={} total_bin={} m={} {}\n",
               phase != nullptr ? phase : "heuristic",
               decision != nullptr ? decision : "unknown",
               has_incumbent ? 1 : 0, gate_gap,
               opt.gap_tol, opt.good_incumbent_gap, gate_closed ? 1 : 0,
               large_generic_low_frac_root ? 1 : 0,
               root_frac_bin_count, root_total_bin_count, presolved_m,
               detail != nullptr ? detail : "");
	  };

			  DualProofRow cached_queue_dual_proof;
			  double cached_queue_dual_proof_upper = kInf;
			  bool have_cached_queue_dual_proof = false;
			  bool cached_queue_dual_proof_is_global = false;
			  std::vector<BranchDomainLiteral> cached_queue_dual_proof_scope;
			  DualProofRow cached_queue_objective_event_proof;
			  bool have_cached_queue_objective_event_proof = false;
			  bool cached_queue_objective_event_proof_is_global = false;
			  double cached_queue_objective_event_proof_upper = kInf;
			  std::vector<BranchDomainLiteral> cached_queue_objective_event_proof_scope;
			  DualProofRow root_global_dual_proof;
			  double root_global_dual_proof_upper = kInf;
			  bool have_root_global_dual_proof = false;
			  std::uint64_t root_dual_proof_epoch = 0;

				  double last_root_incumbent_propagation_obj = kInf;
			  std::uint64_t last_root_incumbent_propagation_domain_epoch =
			      std::numeric_limits<std::uint64_t>::max();
			  std::uint64_t last_root_incumbent_propagation_objective_epoch =
			      std::numeric_limits<std::uint64_t>::max();
			  std::uint64_t last_root_incumbent_propagation_proof_epoch =
			      std::numeric_limits<std::uint64_t>::max();
			  std::size_t last_root_incumbent_propagation_domain_signature =
			      std::numeric_limits<std::size_t>::max();
			  std::function<void(const char*)> run_root_incumbent_propagation;
			  run_root_incumbent_propagation = [&](const char* reason) {
	    if (!has_incumbent ||
	        !std::isfinite(incumbent_obj) || !std::isfinite(root.bound) ||
        !use_objective_propagation || base_lp.sense != Sense::Minimize) {
      return;
    }
	    // Achterberg (2007), Sec. 3.1: an audited primal meeting the rigorous
	    // root dual bound is already an exact certificate. Incumbent-driven
	    // closure cannot strengthen it; call-graph mismatch analysis is in
	    // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
	    if (incumbent_obj - root.bound <=
	        1e-9 * std::max(1.0, std::abs(incumbent_obj))) {
	      return;
	    }
	    const double upper_limit = incumbent_upper_limit();
	    if (!std::isfinite(upper_limit)) {
	      return;
	    }
    const double improve_tol =
        std::max(1e-7, 1e-10 * std::max(1.0, std::abs(incumbent_obj)));
	    const std::uint64_t domain_epoch = current_domain_learning_epoch();
	    const std::uint64_t objective_epoch = current_objective_artifact_epoch();
	    const std::size_t domain_signature = root_domain_signature();
	    const bool cutoff_improved =
	        !std::isfinite(last_root_incumbent_propagation_obj) ||
	        incumbent_obj < last_root_incumbent_propagation_obj - improve_tol;
		    const bool domain_event_changed =
		        domain_epoch != last_root_incumbent_propagation_domain_epoch ||
		        objective_epoch != last_root_incumbent_propagation_objective_epoch ||
		        root_dual_proof_epoch !=
		            last_root_incumbent_propagation_proof_epoch ||
		        domain_signature != last_root_incumbent_propagation_domain_signature;
	    if (!cutoff_improved && !domain_event_changed) {
	      return;
	    }
		    consume_objective_cutoff_artifacts(reason);
    if (!root_cutoff_domain_commit_allowed) {
      if (opt.verbose || trace_bc_timeline ||
          bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
        fmt::print(stderr,
                   "[B&C ROOT-INC-PROP] {} scoped-only cutoff closure; "
                   "root LP domain unchanged inc={:.10g} bound={:.10g}\n",
                   reason != nullptr ? reason : "incumbent", incumbent_obj,
                   root.bound);
	      }
      return;
    }

	    const double old_bound = root.bound;
    const Eigen::VectorXd old_lb = root.lb;
    const Eigen::VectorXd old_ub = root.ub;

    const Eigen::SparseMatrix<double, Eigen::RowMajor> prop_A_row = base_lp.A;
    const Eigen::SparseMatrix<double, Eigen::RowMajor> prop_Aeq_row = base_lp.Aeq;

    int objective_tightened = 0;
    int objective_pruned = 0;
    int row_tightened = 0;
    int clique_tightened = 0;
	    int implication_tightened = 0;
	    int conflict_tightened = 0;
      int cutpool_tightened = 0;
      int redcost_tightened = 0;
	      int redcost_pruned = 0;
	      int redcost_active = 0;
	      int proof_capacity_tightened = 0;
	      int proof_capacity_pruned = 0;
	      std::uint64_t proof_capacity_candidates = 0;
		      std::uint64_t proof_capacity_fixings = 0;
		      std::uint64_t proof_capacity_skip_coeff = 0;
		      std::uint64_t proof_capacity_skip_budget = 0;
		      std::uint64_t objective_artifacts_learned = 0;
		      std::uint64_t source_artifacts_learned = 0;
		      int reasons_added = 0;
		      int closure_passes = 0;
		      bool pruned = false;

    auto duplicate_root_reason_bound = [&](const DomainReasonBound& rb) {
      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
      const double tol = std::max(1e-9, opt.int_tol);
      for (const auto& existing : root.domain_reason_bounds) {
        if (conflict_clause_hash(existing.reason) == reason_hash &&
            domain_reason_bounds_equal(existing, rb, tol)) {
          return true;
        }
      }
      return false;
    };
    auto append_root_reason_bounds =
        [&](std::vector<DomainReasonBound>& reason_bounds) {
      for (auto& rb : reason_bounds) {
        if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
            !std::isfinite(rb.bound.value) ||
            duplicate_root_reason_bound(rb)) {
          continue;
        }
        append_domain_reason_bound(root, std::move(rb), n, &old_lb, &old_ub);
        ++reasons_added;
      }
      reason_bounds.clear();
    };

    record_objective_capacity_conformance(
        "root_incumbent_objective_capacity", root.lb, root.ub,
        upper_limit, old_bound);

    // HiGHS' HighsDomain::propagate() is a single monotone domain closure:
    // objective cutoff, conflict pool, model rows, cutpool rows and implication
    // sources keep reactivating each other until no bound changes remain.
	    bool redcost_lurking_propagated = false;
	    const std::size_t max_root_closure_passes =
	        std::max<std::size_t>(8, 2 * static_cast<std::size_t>(n) + 8);
    for (std::size_t pass = 0; pass < max_root_closure_passes; ++pass) {
      ++closure_passes;
      const Eigen::VectorXd pass_lb = root.lb;
      const Eigen::VectorXd pass_ub = root.ub;
      const std::size_t pass_reason_count = root.domain_reason_bounds.size();
	      const int pass_conflicts = conflict_pool.size();
	      const int pass_cutpool_size = cut_pool.size();
	      const std::uint64_t pass_implications =
	          static_cast<std::uint64_t>(implication_graph.size());
	      const std::uint64_t pass_clique_edges =
	          static_cast<std::uint64_t>(clique_table.n_literal_edges());
	      const std::uint64_t pass_domain_epoch =
	          current_domain_learning_epoch();
	      const std::uint64_t pass_objective_epoch =
	          current_objective_artifact_epoch();
	      const std::uint64_t pass_closure_epoch =
	          current_domain_closure_epoch();

	      source_artifacts_learned +=
	          synchronize_root_source_cutpool_domain_sources(
	              "root_incumbent_closure");
	      const auto root_refresh =
	          refresh_objective_domain_sources("root_incumbent_closure");
	      source_artifacts_learned += root_refresh.total_published();

	      int pass_obj_tight = 0;
	      int pass_obj_pruned = 0;
	      std::vector<DomainReasonBound> objective_reasons;
      const bool obj_ok = objective_propagation.propagate(
          base_lp, root.lb, root.ub, upper_limit, opt.int_tol,
          /*depth=*/0,
          static_cast<int>(root.domain_reason_bounds.size()),
          &objective_reasons, pass_obj_tight, pass_obj_pruned);
      objective_tightened += std::max(0, pass_obj_tight);
      objective_pruned += std::max(0, pass_obj_pruned);
      append_root_reason_bounds(objective_reasons);
		      if (!obj_ok || pass_obj_pruned > 0 ||
		          !bounds_consistent(root.lb, root.ub)) {
		        pruned = true;
		        break;
		      }

		      auto proof_domain_lower_bound =
		          [&](const DualProofRow& proof, double proof_upper) -> double {
		        if (!proof.valid || proof.coeff.size() < n ||
		            proof.coeff.nonZeros() <= 0 || !std::isfinite(proof_upper) ||
		            !std::isfinite(proof.rhs) || root.lb.size() < n ||
		            root.ub.size() < n) {
		          return kInf;
		        }
		        double min_activity = 0.0;
		        for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff);
		             it; ++it) {
		          const int col = static_cast<int>(it.index());
		          const double a = it.value();
		          if (col < 0 || col >= n || !std::isfinite(a)) return kInf;
		          if (a > 0.0) {
		            if (!std::isfinite(root.lb[col])) return kInf;
		            min_activity += a * root.lb[col];
		          } else if (a < 0.0) {
		            if (!std::isfinite(root.ub[col])) return kInf;
		            min_activity += a * root.ub[col];
		          }
		        }
		        if (!std::isfinite(min_activity)) return kInf;
		        return proof_upper - proof.rhs + min_activity;
		      };
		      if (have_root_global_dual_proof && root_global_dual_proof.valid &&
		          root_global_dual_proof.coeff.size() >= n &&
		          root_global_dual_proof.coeff.nonZeros() > 0 &&
		          std::isfinite(root_global_dual_proof_upper)) {
		        const double proof_lower = proof_domain_lower_bound(
		            root_global_dual_proof, root_global_dual_proof_upper);
		        if (std::isfinite(proof_lower)) {
		          std::vector<DomainReasonBound> proof_reasons;
		          ObjectivePropagationState::ProofCapacityPropagationStats proof_stats;
		          int pass_proof_tightened = 0;
		          int pass_proof_pruned = 0;
		          const Eigen::VectorXd* root_reduced_costs = nullptr;
		          if (root.basis_hint && root.basis_hint->cached_reduced_costs &&
		              root.basis_hint->cached_reduced_costs->size() >= n) {
		            root_reduced_costs = root.basis_hint->cached_reduced_costs.get();
		          }
		          const bool proof_ok =
		              objective_propagation.propagate_proof_capacity_one_missing(
		                  base_lp,
		                  ObjectivePropagationState::ProofCapacityContext{
		                      &root_global_dual_proof.coeff,
		                      root.x_relax.size() >= n ? &root.x_relax : nullptr,
		                      root_reduced_costs, proof_lower, upper_limit,
		                      opt.int_tol, std::max(1e-9, opt.lp_tol),
		                      /*depth=*/0,
		                      static_cast<int>(root.domain_reason_bounds.size())},
		                  root.lb, root.ub, &proof_reasons,
		                  pass_proof_tightened, pass_proof_pruned,
		                  &proof_stats);
		          proof_capacity_tightened += std::max(0, pass_proof_tightened);
		          proof_capacity_pruned += std::max(0, pass_proof_pruned);
		          proof_capacity_candidates += proof_stats.candidates;
		          proof_capacity_fixings += proof_stats.fixings;
		          proof_capacity_skip_coeff += proof_stats.skip_coeff;
		          proof_capacity_skip_budget += proof_stats.skip_budget;
		          out.bc_stats.objective_implied_event_proof_candidates +=
		              proof_stats.candidates;
		          out.bc_stats.objective_implied_event_proof_fixings +=
		              proof_stats.fixings;
		          out.bc_stats.objective_implied_event_proof_skip_coeff +=
		              proof_stats.skip_coeff;
		          out.bc_stats.objective_implied_event_proof_skip_budget +=
		              proof_stats.skip_budget;
		          out.bc_stats.termination_effective_candidates +=
		              proof_stats.candidates;
		          out.bc_stats.termination_effective_admitted +=
		              proof_stats.fixings + proof_stats.prunes;
		          append_root_reason_bounds(proof_reasons);
		          if (!proof_ok || pass_proof_pruned > 0 ||
		              !bounds_consistent(root.lb, root.ub)) {
		            pruned = true;
		            break;
		          }
		        }
		      }

				    if (!redcost_lurking_propagated &&
				        root_redcost_lurking_bounds.size() > 0) {
			        redcost_lurking_propagated = true;
			        std::vector<DomainReasonBound> redcost_reasons;
			        const auto rc_stats = root_redcost_lurking_bounds.propagate(
		            upper_limit, root.bound, root.lb, root.ub, &redcost_reasons,
	            /*reason_depth=*/0,
	            static_cast<int>(root.domain_reason_bounds.size()),
	            std::max(1e-9, opt.int_tol));
	        redcost_tightened += std::max(0, rc_stats.tightened);
	        redcost_pruned += std::max(0, rc_stats.pruned);
	        redcost_active += std::max(0, rc_stats.records_active);
	        append_root_reason_bounds(redcost_reasons);
	        if (rc_stats.pruned > 0 || !bounds_consistent(root.lb, root.ub)) {
	          pruned = true;
	          break;
	        }
	      }

	      if (!conflict_pool.empty()) {
        int pass_conflict_tight = 0;
        std::vector<BoundChangeInfo> conflict_changes;
        std::vector<DomainReasonBound> conflict_reasons;
        const bool conflict_ok = conflict_pool.propagate_with_reasons(
            base_lp.vars, root.lb, root.ub, &pass_conflict_tight,
            &conflict_changes, &conflict_reasons, opt.int_tol);
        (void)conflict_changes;
        conflict_tightened += std::max(0, pass_conflict_tight);
        append_root_reason_bounds(conflict_reasons);
        if (!conflict_ok || !bounds_consistent(root.lb, root.ub)) {
          pruned = true;
          break;
        }
      }

      if (!implication_graph.empty()) {
	        implication_tightened += implication_graph.propagate(
	            base_lp.vars, root.lb, root.ub, nullptr,
	            std::max(1e-9, opt.int_tol));
        if (!bounds_consistent(root.lb, root.ub)) {
          pruned = true;
          break;
        }
      }
      if (!clique_table.empty()) {
        clique_tightened += clique_table.propagate(root.lb, root.ub);
        if (!bounds_consistent(root.lb, root.ub)) {
          pruned = true;
          break;
        }
      }

      const int pass_row_tightened = node_bound_propagation(
          base_lp, prop_A_row, prop_Aeq_row, root.lb, root.ub,
          opt.bound_propagation_rounds);
      row_tightened += std::max(0, pass_row_tightened);
      if (!bounds_consistent(root.lb, root.ub)) {
        pruned = true;
        break;
      }

      auto propagate_root_cutpool_family =
          [&](const std::vector<Eigen::SparseVector<double>>& rows,
              const std::vector<double>& rhs) -> bool {
        if (rows.empty()) return true;
        SparseCutpoolPropagationStats stats;
        std::vector<DomainReasonBound> cut_reasons;
        const bool ok = propagate_sparse_cutpool_rows(
            rows, rhs, root.lb, root.ub, opt.bound_propagation_rounds,
            &stats, &cut_reasons, /*reason_depth=*/0,
            static_cast<int>(root.domain_reason_bounds.size()));
        cutpool_tightened += std::max(0, stats.tightened);
        append_root_reason_bounds(cut_reasons);
        return ok && bounds_consistent(root.lb, root.ub);
      };
      if ((!root_transformed_cutpool_in_global_domain_pool &&
           !propagate_root_cutpool_family(root_transformed_cutpool_audit_rows,
                                          root_transformed_cutpool_audit_rhs)) ||
          !propagate_root_cutpool_family(variable_bound_cutpool_rows,
                                         variable_bound_cutpool_rhs)) {
        pruned = true;
        break;
      }

      if (cut_pool.size() > 0) {
        int pass_cutpool_tight = 0;
        std::vector<BoundChangeInfo> cutpool_changes;
        std::vector<DomainReasonBound> cutpool_reasons;
        CutPoolPropagationStats cutpool_stats;
        const bool cutpool_ok = cut_pool.propagate_with_reasons(
            base_lp.vars, root.lb, root.ub, opt.bound_propagation_rounds,
            &pass_cutpool_tight, &cutpool_changes, &cutpool_reasons,
            &cutpool_stats, nullptr, &root_implied_integer_cols,
            /*reason_depth=*/0,
            static_cast<int>(root.domain_reason_bounds.size()),
            std::max(1e-9, opt.lp_tol));
        (void)cutpool_changes;
        cutpool_tightened += std::max(0, pass_cutpool_tight);
        append_root_reason_bounds(cutpool_reasons);
        if (!cutpool_ok || !bounds_consistent(root.lb, root.ub)) {
          pruned = true;
          break;
        }
      }

      apply_domain_objective_lower_bound(root, "root_incumbent_closure");

      // HiGHS reaches a fixed point over domain propagation before and after
      // root reduced-cost fixing, then extracts objective cliques from the
      // strengthened global domain.  Re-run the objective-cutoff artifact
      // extractor whenever this pass moved the root domain so conflict/clique
      // sources join the same monotone closure instead of arriving one
      // incumbent later.
      objective_artifacts_learned +=
          consume_objective_cutoff_artifacts("root_incumbent_fixed_point");

      const auto pass_delta =
          count_bound_changes(pass_lb, pass_ub, root.lb, root.ub);
      const bool pass_changed =
          pass_delta.first > 0 ||
	          root.domain_reason_bounds.size() != pass_reason_count ||
	          conflict_pool.size() != pass_conflicts ||
	          cut_pool.size() != pass_cutpool_size ||
	          static_cast<std::uint64_t>(implication_graph.size()) !=
	              pass_implications ||
	          static_cast<std::uint64_t>(clique_table.n_literal_edges()) !=
	              pass_clique_edges ||
	          current_domain_learning_epoch() != pass_domain_epoch ||
	          current_objective_artifact_epoch() != pass_objective_epoch ||
	          current_domain_closure_epoch() != pass_closure_epoch;
	      if (!pass_changed) break;
	    }

		    const int total_tightened = objective_tightened + row_tightened +
		        clique_tightened + implication_tightened + conflict_tightened +
		        cutpool_tightened + redcost_tightened +
		        proof_capacity_tightened;
		    const auto root_inc_delta =
		        count_bound_changes(old_lb, old_ub, root.lb, root.ub);
		    const std::uint64_t root_inc_domain_moves =
		        static_cast<std::uint64_t>(root_inc_delta.first) +
		        static_cast<std::uint64_t>(root_inc_delta.second);
		    if (root_inc_domain_moves > 0) {
		      note_global_domain_learning(root_inc_domain_moves);
		    }
		    const std::uint64_t root_inc_sources = conformance_source_total();
		    const std::string root_inc_detail = fmt::format(
			        "reason={} pass={} obj={} conflict={} impl={} row={} clique={} "
			        "cutpool={} rc={} rc_active={} proof={} proofCand={} "
			        "proofFix={} proofSkip={}/{} reasons={} obj_pruned={} "
			        "rc_pruned={} proof_pruned={} srcLearned={} objLearned={}",
		        reason != nullptr ? reason : "incumbent", closure_passes,
		        objective_tightened, conflict_tightened, implication_tightened,
		        row_tightened, clique_tightened, cutpool_tightened,
		        redcost_tightened, redcost_active, proof_capacity_tightened,
		        proof_capacity_candidates, proof_capacity_fixings,
		        proof_capacity_skip_coeff, proof_capacity_skip_budget, reasons_added,
		        objective_pruned, redcost_pruned, proof_capacity_pruned,
		        source_artifacts_learned, objective_artifacts_learned);
	    auto mark_root_incumbent_closure_consumed = [&]() {
	      last_root_incumbent_propagation_obj = incumbent_obj;
	      last_root_incumbent_propagation_domain_epoch =
	          current_domain_learning_epoch();
		      last_root_incumbent_propagation_objective_epoch =
		          current_objective_artifact_epoch();
		      last_root_incumbent_propagation_proof_epoch =
		          root_dual_proof_epoch;
		      last_root_incumbent_propagation_domain_signature =
		          root_domain_signature();
		    };
	    if (pruned) {
	      mark_root_incumbent_closure_consumed();
	      root.bound = incumbent_obj;
      persistent_lp_state.invalidate_base_sf();
      record_conformance_propagation(
          "root_incumbent_propagation", root_inc_sources,
          root_inc_delta.first, root_inc_delta.second, 1,
          /*resolved=*/false, /*resolve_ok=*/true, old_bound, root.bound,
          root_inc_detail);
      if (bc_conf_diag_enabled() || trace_bc_timeline) {
		        fmt::print(stderr,
		                   "[B&C ROOT-INC-PROP] {} pruned root "
		                   "pass={} obj={} conflict={} impl={} row={} clique={} "
			                   "cutpool={} rc={} rc_active={} proof={} "
			                   "proofCand={} proofFix={} proofSkip={}/{} "
			                   "srcArtifacts={} objArtifacts={} reasons={} obj_pruned={} rc_pruned={} "
			                   "proof_pruned={} bound={:.2f}->{:.2f}\n",
			                   reason != nullptr ? reason : "incumbent",
			                   closure_passes, objective_tightened, conflict_tightened,
			                   implication_tightened, row_tightened, clique_tightened,
			                   cutpool_tightened, redcost_tightened, redcost_active,
			                   proof_capacity_tightened, proof_capacity_candidates,
			                   proof_capacity_fixings, proof_capacity_skip_coeff,
			                   proof_capacity_skip_budget, source_artifacts_learned,
			                   objective_artifacts_learned, reasons_added, objective_pruned, redcost_pruned,
			                   proof_capacity_pruned, old_bound, root.bound);
		      }
	      return;
	    }

	    if (total_tightened <= 0) {
	      mark_root_incumbent_closure_consumed();
	      record_conformance_propagation(
          "root_incumbent_propagation", root_inc_sources,
          root_inc_delta.first, root_inc_delta.second, 0,
          /*resolved=*/false, /*resolve_ok=*/true, old_bound, old_bound,
          root_inc_detail);
	      if (opt.verbose) {
			        fmt::print(stderr,
			                   "[B&C ROOT-INC-PROP] {} no tightening "
			                   "cutoff={:.2f} upper_limit={:.2f} bound={:.2f} pass={} "
			                   "srcArtifacts={} objArtifacts={} reasons={} rc_active={} proofCand={} "
			                   "proofSkip={}/{}\n",
			                   reason != nullptr ? reason : "incumbent",
			                   incumbent_obj, upper_limit, root.bound, closure_passes,
			                   source_artifacts_learned, objective_artifacts_learned,
			                   reasons_added, redcost_active,
			                   proof_capacity_candidates, proof_capacity_skip_coeff,
			                   proof_capacity_skip_budget);
	      }
	      return;
	    }

	    const LPModel root_lp_before_resolve = root_lp;
	    const LPRelaxationResult root_relax_before_resolve = root_relax;
	    const auto root_basis_before_resolve = root.basis_hint;
	    const Eigen::VectorXd root_x_before_resolve = root.x_relax;
	    const Eigen::VectorXd root_x_seed_before_resolve = root.x_seed;
	    const double root_bound_before_resolve = root.bound;
	    apply_node_bounds(root_lp.vars, root.lb, root.ub);
	    const auto t_inc_resolve0 = std::chrono::steady_clock::now();
	    LPRelaxationResult inc_relax = solve_lp_relaxation(
	        root_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
	    const auto t_inc_resolve1 = std::chrono::steady_clock::now();
	    out.bc_stats.lp_solves += 1;
	    const bool resolve_ok =
	        inc_relax.primal.stats.success && inc_relax.primal.x.size() == n &&
	        std::isfinite(inc_relax.dual_bound);
	    if (!resolve_ok) {
	      root_lp = root_lp_before_resolve;
	      root_relax = root_relax_before_resolve;
	      root.basis_hint = root_basis_before_resolve;
	      root.x_relax = root_x_before_resolve;
	      root.x_seed = root_x_seed_before_resolve;
	      root.bound = root_bound_before_resolve;
	      root.lp_refresh_needed = true;
	      record_conformance_propagation(
	          "root_incumbent_propagation", root_inc_sources,
	          root_inc_delta.first, root_inc_delta.second, 0,
	          /*resolved=*/true, /*resolve_ok=*/false, old_bound, root.bound,
	          root_inc_detail + " root_resolve_failed=1");
        if (bc_conf_diag_enabled()) {
	        fmt::print(stderr,
	                   "[B&C ROOT-INC-PROP] {} pass={} root_resolve_failed "
	                   "status={} bound={:.2f}->{:.2f}\n",
	                   reason != nullptr ? reason : "incumbent",
	                   closure_passes, inc_relax.primal.stats.status,
	                   old_bound, root.bound);
	      }
	      return;
	    }

	    root_relax = std::move(inc_relax);
	    root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	    root.x_seed = root.x_relax;
	    root.basis_hint = root_relax.basis_hint;
	    if (root.basis_hint) root.basis_hint->compact_indices_storage();
	    root.bound = root_relax.dual_bound;
	    if (root.estimate < root.bound) root.estimate = root.bound;
	    root.lp_refresh_needed = false;
	    root.domain_learning_epoch = current_domain_learning_epoch();
	    root.objective_artifact_epoch = current_objective_artifact_epoch();
	    root.domain_closure_epoch = current_domain_closure_epoch();
		    if (maintain_native_xpool_sf_cache && root_relax.simplex &&
		        root_relax.simplex->form.A.rows() > 0) {
		      root_first_class_unscaled_sf = build_standard_form_lp(root_lp);
		      root_first_class_sf_valid = true;
		    } else {
	      root_first_class_sf_valid = false;
	    }
	    if (root_relax.simplex &&
	        root_relax.simplex->basis.cached_sparse_basis) {
	      root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
	      root.has_live_factor_telemetry = true;
	      root.live_factor_telemetry =
	          get_sparse_basis_factor_telemetry(
	              root_relax.simplex->basis.cached_sparse_basis);
	    } else {
	      root_sbasis.reset();
	    }
	    if (root.basis_hint) {
	      persistent_lp_state.set_cut_basis(root.basis_hint);
	    }
	    persistent_lp_state.invalidate_base_sf();
	    mark_root_incumbent_closure_consumed();
	    const auto rc_add_stats =
	        refresh_root_redcost_lurking_bounds(
	            "root_incumbent_propagation_resolve");
	    record_conformance_propagation(
	        "root_incumbent_propagation", root_inc_sources,
	        root_inc_delta.first, root_inc_delta.second, 0,
	        /*resolved=*/true, /*resolve_ok=*/true, old_bound, root.bound,
	        root_inc_detail +
	            fmt::format(" root_resolve=1 rcAdded={}",
	                        rc_add_stats.records_added));
    if (bc_conf_diag_enabled() || trace_bc_timeline) {
	      fmt::print(stderr,
	                 "[B&C ROOT-INC-PROP] {} pass={} obj={} conflict={} impl={} "
	                 "row={} clique={} cutpool={} rc={} rc_active={} "
	                 "proof={} proofCand={} proofFix={} proofSkip={}/{} "
	                 "srcArtifacts={} objArtifacts={} reasons={} "
	                 "bound={:.2f}->{:.2f} root_resolve=1 "
	                 "resolve={:.1f}ms rcAdded={}\n",
	                 reason != nullptr ? reason : "incumbent", closure_passes,
	                 objective_tightened, conflict_tightened,
	                 implication_tightened, row_tightened, clique_tightened,
	                 cutpool_tightened, redcost_tightened, redcost_active,
	                 proof_capacity_tightened, proof_capacity_candidates,
	                 proof_capacity_fixings, proof_capacity_skip_coeff,
	                 proof_capacity_skip_budget, source_artifacts_learned,
	                 objective_artifacts_learned, reasons_added, old_bound,
	                 root.bound,
	                 std::chrono::duration<double, std::milli>(
	                     t_inc_resolve1 - t_inc_resolve0).count(),
	                 rc_add_stats.records_added);
	    }
	  };

	  root_incumbent_fixed_point_hook = run_root_incumbent_propagation;

	  note_root_subsolve(
	      "root_relax",
	      std::chrono::duration<double, std::milli>(t_root1 - t_root0).count(),
	      root_relax.primal.stats.success && root_relax.primal.x.size() == n);

	  // An integral optimal root LP is already a globally valid MIP incumbent.
	  // Register it before repair heuristics: their feasibility objectives may
	  // return a different, worse integral point even when no repair is needed.
	  bool integral_root_lp = certified_root_primal_x.size() == n;
	  for (int col : declared_branchable_indices) {
	    if (col < 0 || col >= n ||
	        std::abs(certified_root_primal_x[col] -
	                 std::round(certified_root_primal_x[col])) >
	            opt.int_tol) {
	      integral_root_lp = false;
	      break;
	    }
	  }
	  if (integral_root_lp &&
	      satisfies_with_bounds(base_lp, certified_root_primal_x, root.lb, root.ub,
	                            std::max(1e-7, opt.heuristic_feasibility_tol))) {
	    const double root_incumbent_obj =
	        objective_value(base_lp.c, certified_root_primal_x, base_lp.sense);
	    const bool adopted_integral_root = adopt_root_incumbent(
	        "integral_root_lp", certified_root_primal_x, root_incumbent_obj);
	    if (trace_bc_timeline) {
	      fmt::print(stderr,
	                 "[B&C-INTEGRAL-ROOT] adopted={} obj={:.12g} "
	                 "current_bound={:.12g}\n",
	                 adopted_integral_root ? 1 : 0, root_incumbent_obj,
	                 root.bound);
	    }
	  } else if (trace_bc_timeline) {
	    fmt::print(stderr,
	               "[B&C-INTEGRAL-ROOT] adopted=0 integral={} feasible=0 "
	               "bounds0=[{:.12g},{:.12g}] bounds1=[{:.12g},{:.12g}]\n",
	               integral_root_lp ? 1 : 0,
	               n > 0 ? root.lb[0] : 0.0, n > 0 ? root.ub[0] : 0.0,
	               n > 1 ? root.lb[1] : 0.0, n > 1 ? root.ub[1] : 0.0);
	  }

	  if (pending_highs_root_primal.size() == n) {
	    const double highs_root_primal_obj = objective_value(
	        base_lp.c, pending_highs_root_primal, base_lp.sense);
	    const bool adopted_highs_root_primal = adopt_root_incumbent(
	        "highs_root_primal", pending_highs_root_primal,
	        highs_root_primal_obj);
	    note_root_subsolve("highs_root_primal", 0.0, true,
	                       adopted_highs_root_primal);
	    if (opt.verbose) {
	      fmt::print(stderr,
	                 "[B&C-HIGHS-ROOT-PRIMAL] available=1 adopted={} "
	                 "obj={:.12g}\n",
	                 adopted_highs_root_primal ? 1 : 0,
	                 highs_root_primal_obj);
	    }
	  }

	  if (!strict_highs_lp_contract &&
      has_verified_root_cutoff && verified_root_cutoff_x.size() == n) {
	    const bool improved = adopt_root_incumbent(
	        verified_root_cutoff_source.c_str(), verified_root_cutoff_x,
	        verified_root_cutoff_obj);
    note_root_subsolve(verified_root_cutoff_source.c_str(), 0.0,
                       true, improved);
    if (opt.verbose && improved) {
      fmt::print(stderr,
                 "[B&C] verified root cutoff incumbent: obj={:.2f} source={}\n",
                 verified_root_cutoff_obj, verified_root_cutoff_source);
    }
  }

  // Join the analytic-centre background thread.
  // The thread ran concurrently with root LP + root cuts.  Join here (after
  // all cut rounds) before we enter the root heuristic phase that uses it.
  // If the thread completed early, join() returns immediately (no stall).
  if (ac_thread.joinable()) {
    [[maybe_unused]] auto t_ac_join0 = std::chrono::steady_clock::now();
    ac_thread.join();
    [[maybe_unused]] auto t_ac_join1 = std::chrono::steady_clock::now();
    if (opt.verbose) {
      fmt::print(stderr, "[B&C-ANALYTIC-CENTRE] {} join_wait={:.1f}ms\n",
                 analytic_centre.has_value() ? "ok" : "failed",
                 std::chrono::duration<double, std::milli>(t_ac_join1 - t_ac_join0).count());
    }
  }

  auto apply_forward_mapped_heuristic = [&]() {
    if (!(heuristic_x_orig.size() > 0 && std::isfinite(heuristic_obj))) {
      return;
    }

    Eigen::VectorXd x_fwd = heuristic_x_orig;

    if (highs_mip_model_reduced) {
      x_fwd = highs_presolve_forward_map(highs_mip_ps, x_fwd);
    }

#ifdef MIPSOLVERS_HAVE_PAPILO
    if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
      x_fwd = papilo_forward_map(papilo_ps, x_fwd);
    }
#endif

    if (native_model_reduced) {
      x_fwd = native_ps.forward_map(x_fwd);
    }

    if (x_fwd.size() != n) {
      return;
    }

    for (int i = 0; i < n; ++i) {
      x_fwd[i] = std::min(root.ub[i], std::max(root.lb[i], x_fwd[i]));
      if (is_integer_type(base_lp.vars[i])) {
        x_fwd[i] = std::round(x_fwd[i]);
      }
    }

    if (satisfies_with_bounds(base_lp, x_fwd, root.lb, root.ub, 1e-6)) {
      const double fwd_obj = objective_value(base_lp.c, x_fwd, base_lp.sense);
      // A verified warm start is a legal incumbent even when it is weak.  For
      // HiGHS-style root processing the first incumbent mainly supplies an
      // objective cutoff for reduced-cost fixing, conflict cuts, and pruning.
      // Keep the historical 10% quality gate only when the new generic early
      // incumbent path is explicitly disabled.
      if (!opt.accept_verified_warm_start_incumbent &&
          root.bound > -kInf && std::isfinite(root.bound) && std::isfinite(fwd_obj)) {
        const double lp_gap_frac =
            (fwd_obj - root.bound) / std::max(1.0, std::abs(root.bound));
        if (lp_gap_frac > 0.10) {
          if (opt.verbose) {
            fmt::print(stderr,
              "[B&C] warm-start incumbent rejected (lp_gap={:.1f}%): "
              "obj={:.2f} root_lb={:.2f}\n",
              lp_gap_frac * 100.0, fwd_obj, root.bound);
          }
          return;
        }
      }
      if (adopt_root_incumbent("forward_mapped_heuristic", x_fwd, fwd_obj)) {
        if (opt.verbose) {
          fmt::print(stderr,
            "[B&C] forward-mapped heuristic incumbent: obj={:.2f}\n", fwd_obj);
        }
      }
    } else if (opt.verbose) {
      int n_ineq_viol = 0, n_eq_viol = 0, n_bnd_viol = 0;
      double max_ineq_viol = 0, max_eq_viol = 0, max_bnd_viol = 0;
      for (int i = 0; i < n; ++i) {
        double v = std::max(0.0, std::max(root.lb[i] - x_fwd[i], x_fwd[i] - root.ub[i]));
        if (v > 1e-6) { ++n_bnd_viol; max_bnd_viol = std::max(max_bnd_viol, v); }
      }
      Eigen::VectorXd Ax = base_lp.A * x_fwd;
      for (int i = 0; i < static_cast<int>(base_lp.A.rows()); ++i) {
        double v = Ax[i] - base_lp.b[i];
        if (v > 1e-6) { ++n_ineq_viol; max_ineq_viol = std::max(max_ineq_viol, v); }
      }
      if (base_lp.Aeq.rows() > 0) {
        Eigen::VectorXd Aeqx = base_lp.Aeq * x_fwd;
        for (int i = 0; i < static_cast<int>(base_lp.Aeq.rows()); ++i) {
          double v = std::abs(Aeqx[i] - base_lp.beq[i]);
          if (v > 1e-6) { ++n_eq_viol; max_eq_viol = std::max(max_eq_viol, v); }
        }
      }
      fmt::print(stderr,
        "[B&C] forward-mapped heuristic infeasible: bnd={} ({:.2e}) ineq={} ({:.2e}) eq={} ({:.2e})\n",
        n_bnd_viol, max_bnd_viol, n_ineq_viol, max_ineq_viol, n_eq_viol, max_eq_viol);
    }
  };

  [[maybe_unused]] auto t_phase = std::chrono::steady_clock::now();
  const bool skip_large_near_integral_quick_round =
      large_near_integral_root && root.basis_hint;
  const bool skip_large_near_integral_forward_map =
      large_near_integral_root && root.basis_hint;

  // Large UC-like detection (reuse hoisted flag from before root cuts).
  // pump + repair + stage3 all fail deterministically on tight-ramp UC
  // (rounding a large binary schedule violates ramp/power-balance rows).
  // Keep the large-UC skip for mid/large roots, but allow bounded root
  // heuristics on xlarge roots. On xlarge UC we often need an incumbent
  // early; skipping all root heuristics can leave parallel search unseeded.
  const bool skip_pump_large_uc_like =
      large_uc_like_root && !is_xlarge_root && !opt.auto_highs_root_pipeline;
  if (skip_pump_large_uc_like && opt.verbose) {
    fprintf(stderr, "[B&C] skip root repair/pump on large UC-like (m=%d n_bin=%d) — "
                    "yielding to parallel tree\n",
            presolved_m, bc_skip_n_bin);
  }

  if (!skip_large_near_integral_forward_map) {
    // Delay consuming the caller/forward-mapped incumbent until after the
    // root LP point has had the same chance HiGHS gives it through root
    // randomized rounding. A weak warm start is useful, but it should not
    // mask a stronger LP-derived root incumbent source.
  }

  // IPM adapter for root heuristics (pump, diving, probing, and HiGHS-style
  // fixed-integer repair). Lazily initialized on first use to avoid setup cost
  // when simplex warm-starting handles all root heuristic LPs successfully.
  std::unique_ptr<NativeIPMLPAdapter> root_heur_ipm;
  auto ensure_root_heur_ipm = [&](bool force_for_root_repair = false)
      -> NativeIPMLPAdapter* {
    if (!root_heur_ipm && (opt.use_ipm_root || force_for_root_repair)) {
      IPMLPOptions rh_opt;
      rh_opt.max_iter =
          (presolved_m > 25000) ? 200 : (presolved_m > 10000) ? 150 : 100;
      rh_opt.tol_primal = 1e-7;
      rh_opt.tol_dual = 1e-7;
      rh_opt.tol_gap = 1e-7;
      rh_opt.verbose = false;
      root_heur_ipm = std::make_unique<NativeIPMLPAdapter>(rh_opt);
      root_heur_ipm->prepare_for_node_solves(base_lp);
    }
    return root_heur_ipm.get();
  };

  // Stateful, incremental domain propagation analogous to HiGHS' HighsDomain
  // (the `localdom` reused inside HighsPrimalHeuristics::linesearchRounding).
  // Initialised once with the root box; every `try_fixed_integer_repair`
  // call probes via `fix_col + propagate` and then `restore` to a savepoint,
  // so the per-alpha-step propagation work is O(touched_cols * row_nnz)
  // instead of the prior O(nnz) full-rescan.
  using ::mipsolvers::engine::detail::BCDomain;
  BCDomain repair_domain;
  // HiGHS HighsPrimalHeuristics::tryRoundedPoint copies the structural
  // HighsDomain; generated separation rows retain separate cut-pool ownership.
  // The cut-augmented base_lp is still used by repair and incumbent audit.
  // Derivation in
  // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
  repair_domain.init(root_lp, root.lb, root.ub,
                     BCDomain::InitialPropagation::AlreadyClosed);
  if (bc_env_flag_enabled("MIPSOLVERS_ROOT_REPAIR_ROW_ZERO_TRACE")) {
    const Eigen::SparseMatrix<double, Eigen::RowMajor> trace_rows = root_lp.A;
    fmt::print(stderr,
               "[NATIVE-ROOT-REPAIR-ROW-ZERO] lhs={:.17g} rhs={:.17g} "
               "terms=",
               lp_row_lhs_or_neg_inf(root_lp, 0), root_lp.b[0]);
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             trace_rows, 0);
         it; ++it) {
      fmt::print(stderr, "{}{}:{:.17g}", it.index() == 0 ? "" : ",",
                 it.col(), it.value());
    }
    fmt::print(stderr, "\n");
  }
  // The root box has already passed root-domain closure. This snapshot mirrors
  // HiGHS' copy of its live domain and therefore starts with an empty queue.
  const bool repair_domain_initial_propagation = repair_domain.propagate();
  if (bc_env_flag_enabled("MIPSOLVERS_ROOT_REPAIR_CONFLICT_TRACE")) {
    fmt::print(stderr,
               "[NATIVE-ROOT-REPAIR-INIT] ok={} infeasible={} complete={} "
               "rows={}\n",
               repair_domain_initial_propagation ? 1 : 0,
               repair_domain.infeasible() ? 1 : 0,
               repair_domain.propagation_complete() ? 1 : 0,
               repair_domain.rows_processed());
  }
  if (bc_env_flag_enabled("MIPSOLVERS_ROOT_REPAIR_DOMAIN_TRACE")) {
    const Eigen::VectorXd& trace_lb = repair_domain.lb();
    const Eigen::VectorXd& trace_ub = repair_domain.ub();
    for (int j = 0; j < n; ++j) {
      fmt::print(stderr,
                 "[NATIVE-ROOT-REPAIR-DOMAIN] col={} lb={:.17g} "
                 "ub={:.17g}\n",
                 j, trace_lb[j], trace_ub[j]);
    }
  }
  bool traced_first_incomplete_repair = false;

  auto try_fixed_integer_repair = [&](const char* source,
                                      Eigen::VectorXd xr) -> bool {
    if (xr.size() != n || bc_optional_root_budget_expired()) return false;
    for (int i = 0; i < n; ++i) {
      xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
      if (is_integer_type(base_lp.vars[i])) {
        xr[i] = std::round(xr[i]);
        xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
      }
    }
    if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub, 1e-6)) {
      const double obj = objective_value(base_lp.c, xr, base_lp.sense);
      const bool improved = adopt_root_incumbent(source, xr, obj);
      note_root_subsolve(source, 0.0, true, improved);
      return improved;
    }

    // HiGHS tryRoundedPoint analog: project the rounded preferences through
    // the evolving persistent domain in lock-priority order. The state
    // machine is savepoint -> {clamp, fix_col, propagate}* -> restore.
    // If propagation proves infeasibility we never reach the LP solver — this
    // is the short-circuit that bounds linesearchRounding on large UC roots.
    const BCDomain::Savepoint domain_sp = repair_domain.savepoint();
    // HiGHS HighsPrimalHeuristics::tryRoundedPoint; Achterberg (2007),
    // Secs. 3.1 and 9.2. Derivation in
    // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
    const bool prop_infeasible = !repair_domain.project_integer_preferences(
        repair_integer_order, xr);
    if (prop_infeasible) {
      if (bc_env_flag_enabled("MIPSOLVERS_ROOT_REPAIR_CONFLICT_TRACE")) {
        if (!repair_domain.infeasible() &&
            !repair_domain.propagation_complete() &&
            !traced_first_incomplete_repair) {
          traced_first_incomplete_repair = true;
          for (int chunk = 1;
               chunk <= 8 && !repair_domain.infeasible() &&
               !repair_domain.propagation_complete();
               ++chunk) {
            const std::uint64_t rows_before = repair_domain.rows_processed();
            (void)repair_domain.propagate();
            fmt::print(stderr,
                       "[NATIVE-ROOT-REPAIR-INCOMPLETE] chunk={} col={} "
                       "rows={} trail={} pending={} infeasible={} complete={}\n",
                       chunk, repair_domain.last_preference_col(),
                       repair_domain.rows_processed() - rows_before,
                       repair_domain.trail_size(),
                       repair_domain.pending_row_count(),
                       repair_domain.infeasible() ? 1 : 0,
                       repair_domain.propagation_complete() ? 1 : 0);
          }
        }
        const auto& cert = repair_domain.infeasibility_certificate();
        fmt::print(stderr,
                   "[NATIVE-ROOT-REPAIR-CONFLICT] source={} kind={} row={} "
                   "col={} activity={:.17g} magnitude={:.17g} inf={} "
                   "side={:.17g} requested=[{:.17g},{:.17g}] "
                   "scratch={:.17g} scratch_inf={}\n",
                   source, cert.kind, cert.row, cert.col, cert.activity,
                   cert.magnitude, cert.infinity_count, cert.row_side,
                   cert.requested_lb, cert.requested_ub,
                   cert.scratch_activity, cert.scratch_infinity_count);
      }
      repair_domain.restore(domain_sp);
      note_root_subsolve(source, 0.0, false);
      return false;
    }
    Eigen::VectorXd repair_lb = repair_domain.lb();
    Eigen::VectorXd repair_ub = repair_domain.ub();

    // RAII guard so every subsequent exit path rolls the domain back to the
    // savepoint we just took. This keeps the persistent state aligned with
    // root.lb/root.ub between calls (HiGHS' localdom.backtrack() analog).
    struct DomainRestoreGuard {
      BCDomain* d; BCDomain::Savepoint sp; bool armed;
      ~DomainRestoreGuard() { if (armed) d->restore(sp); }
    } domain_guard{&repair_domain, domain_sp, true};

    auto t_repair0 = std::chrono::steady_clock::now();
    bool repair_ok = false;
    Eigen::VectorXd repaired;
    const int total_cons = static_cast<int>(base_lp.A.rows() + base_lp.Aeq.rows());
    const bool highs_style_fixed_integer_repair =
        opt.auto_highs_root_pipeline && opt.use_linesearch_rounding;
    if (bc_optional_root_budget_expired()) return false;
    if (opt.use_simplex_lp_nodes && bc_vendored_highs_lp_kernel_enabled(opt)) {
      // Same fixed-integer LP, same selected kernel, remaining wall budget.
      // Derivation: docs/solvers.md, Fixed-Integer Root Repair Kernel and Budget.
      LPModel repair_lp = base_lp;
      for (int i = 0; i < n; ++i) {
        repair_lp.vars[i].lb = repair_lb[i];
        repair_lp.vars[i].ub = repair_ub[i];
      }
      if (bc_optional_root_budget_expired()) return false;
      BCOptions repair_opt = opt;
      const double remaining = bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
      if (remaining <= 0.0) return false;
      repair_opt.time_limit_sec = std::isfinite(remaining) ? remaining : 0.0;
      auto repair_res = solve_lp_relaxation(repair_lp, &xr, nullptr, repair_opt);
      ++out.bc_stats.lp_solves;
      repair_ok = repair_res.primal.stats.success && repair_res.primal.x.size() == n;
      if (repair_ok) repaired = std::move(repair_res.primal.x);
      if (opt.verbose || trace_bc_timeline) {
        fmt::print(stderr,
                   "[B&C-ROOT-REPAIR-LP] source={} kernel=highs budget={:.3f}s "
                   "status={} success={}\n", source, repair_opt.time_limit_sec,
                   repair_res.primal.stats.status, repair_ok ? 1 : 0);
      }
    } else if (total_cons > 10000 &&
        (opt.use_ipm_root || highs_style_fixed_integer_repair)) {
      NativeIPMLPAdapter* repair_ipm =
          ensure_root_heur_ipm(highs_style_fixed_integer_repair);
      if (repair_ipm != nullptr) {
        auto repair_res =
            repair_ipm->solve_node_lp(base_lp, repair_lb, repair_ub, xr);
        ++out.bc_stats.lp_solves;
        repair_ok = repair_res.stats.success && repair_res.x.size() >= n;
        if (repair_ok) repaired = repair_res.x.head(n);
      }
    } else {
      StandardFormLP repair_sf = ensure_base_sf();
      update_standard_form_bounds(repair_sf, base_lp, repair_lb, repair_ub);
      SimplexOptions repair_opt;
      repair_opt.lp_kernel_backend = opt.lp_kernel_backend;
      repair_opt.max_iter = std::min(300, std::max(50, static_cast<int>(repair_sf.A.rows()) / 8));
      repair_opt.feasibility_tol = 1e-7;
      repair_opt.optimality_tol = 1e-7;
      repair_opt.verbose = false;
      repair_opt.allow_cold_start = false;
      // PersistentLPState exposes a basis hint that is dimensionally aligned
      // with the shared base StandardFormLP (HiGHS firstrootbasis analog).
      auto repair_basis_hint = persistent_lp_state.basis_hint_for_base();
      auto repair_res =
          solve_lp_from_sf(repair_sf, repair_opt, repair_basis_hint.get());
      ++out.bc_stats.lp_solves;
      repair_ok = repair_res.result.stats.success && repair_res.result.x.size() >= n;
      if (repair_ok) repaired = repair_res.result.x.head(n);
    }
    auto t_repair1 = std::chrono::steady_clock::now();
    note_root_subsolve(
        source,
        std::chrono::duration<double, std::milli>(t_repair1 - t_repair0).count(),
        repair_ok);
    if (!repair_ok || repaired.size() != n) return false;

    repaired = clamp_to_bounds(repaired, root.lb, root.ub);
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(base_lp.vars[i])) continue;
      repaired[i] = std::round(repaired[i]);
      repaired[i] = std::min(root.ub[i], std::max(root.lb[i], repaired[i]));
    }
    if (!satisfies_with_bounds(base_lp, repaired, root.lb, root.ub, 1e-6)) {
      return false;
    }
    const double obj = objective_value(base_lp.c, repaired, base_lp.sense);
    const bool improved = adopt_root_incumbent(source, repaired, obj);
    if (improved) note_root_subsolve(source, 0.0, true, true);
    return improved;
  };

  auto run_highs_randomized_rounding = [&]() -> bool {
    if (!opt.auto_highs_root_pipeline || root_gap_closed() ||
        skip_pump_large_uc_like || root.x_relax.size() != n) {
      return false;
    }
    Eigen::VectorXd xr = root.x_relax;
    std::mt19937_64 rng(opt.random_seed);
    std::uniform_real_distribution<double> pick(0.1, 0.9);
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(base_lp.vars[i])) continue;
      double rounded = xr[i];
      if (i < static_cast<int>(uplocks.size()) && uplocks[i] == 0) {
        rounded = std::ceil(xr[i] - opt.int_tol);
      } else if (i < static_cast<int>(downlocks.size()) &&
                 downlocks[i] == 0) {
        rounded = std::floor(xr[i] + opt.int_tol);
      } else {
        rounded = std::floor(xr[i] + pick(rng));
      }
      xr[i] = std::min(root.ub[i], std::max(root.lb[i], rounded));
    }
    return try_fixed_integer_repair("randomized_round", xr);
  };

  auto t_pp_a = std::chrono::steady_clock::now();
  if (!strict_highs_lp_contract) {
    run_highs_randomized_rounding();
  }
  auto t_pp_b = std::chrono::steady_clock::now();

	  if (!strict_highs_lp_contract && !skip_large_near_integral_forward_map) {
	    apply_forward_mapped_heuristic();
	  }
  auto t_pp_c = std::chrono::steady_clock::now();

	  auto run_feasibility_jump = [&]() -> bool {
	    if (bc_optional_root_budget_expired()) return false;
	    if (!opt.enable_feasibility_jump ||
	        has_incumbent || root.x_relax.size() != n) {
	      trace_root_primal_gate(
	          "feasibility_jump", "skip",
	          !opt.enable_feasibility_jump
	              ? "disabled=1"
	              : (root.x_relax.size() != n ? "bad_root_x=1"
	                                         : "verified_incumbent_exists=1"));
	      return false;
	    }
	    trace_root_primal_gate("feasibility_jump", "run");

    std::vector<int> bin_cands;
    bin_cands.reserve(static_cast<size_t>(std::min(n, 2048)));
    for (int j = 0; j < n; ++j) {
      const auto& v = base_lp.vars[static_cast<size_t>(j)];
      const bool binary_like = is_integer_type(v) &&
          root.lb[j] >= -1e-9 && root.ub[j] <= 1.0 + 1e-9 &&
          root.ub[j] > root.lb[j] + 1e-9;
      if (!binary_like) continue;
      bin_cands.push_back(j);
    }
    if (bin_cands.empty()) return false;

    std::sort(bin_cands.begin(), bin_cands.end(), [&](int a, int b) {
      const double fa = std::abs(root.x_relax[a] - std::round(root.x_relax[a]));
      const double fb = std::abs(root.x_relax[b] - std::round(root.x_relax[b]));
      if (std::abs(fa - fb) > 1e-12) return fa > fb;
      return std::abs(base_lp.c[a]) > std::abs(base_lp.c[b]);
    });
    const int cand_cap = std::min<int>(static_cast<int>(bin_cands.size()), 2048);
    bin_cands.resize(static_cast<size_t>(cand_cap));

    auto row_merit = [&](double lhs, double rhs) {
      return std::max(0.0, lhs - rhs);
    };
    auto eq_merit = [](double lhs, double rhs) {
      return std::abs(lhs - rhs);
    };

	    const int attempts = std::max(1, opt.feasibility_jump_repair_attempts);
	    for (int attempt = 0; attempt < attempts; ++attempt) {
	      if (has_incumbent || bc_optional_root_budget_expired()) break;
	      Eigen::VectorXd xj = root.x_relax;
      for (int j = 0; j < n; ++j) {
        if (!is_integer_type(base_lp.vars[j])) continue;
        double val = std::round(xj[j]);

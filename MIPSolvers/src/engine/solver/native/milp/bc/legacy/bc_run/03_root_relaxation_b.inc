// @file bc_run/03_root_relaxation_b.inc
// BCSolveState::run() segment 3/14: root LP relaxation / cut loop — part 2 (HiGHS implied-bound separation, tableau).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
        root_transformed_cutpool_audit_rhs.push_back(rhs);
	        return true;
	      };
      auto root_transformed_cutpool_propagate_requested =
          [&](const PoolCut& row, const char* family) -> bool {
        if (!strict_highs_root_fixed_point) return true;
        if (family != nullptr && std::strcmp(family, "implied_bound") == 0) {
          // HiGHS HighsImplications::separateImpliedBounds adds infeasible-side
          // unit cuts with propagate=true, but ordinary two-term implied-bound
          // cuts with propagate=false.  They stay in the cutpool for
          // separate()/aging, not in the domain propagation matrix.
          return row.coeff.nonZeros() == 1;
        }
        return true;
      };
	      auto propagate_sparse_cutpool_rows =
	          [&](const std::vector<Eigen::SparseVector<double>>& rows,
	              const std::vector<double>& rhs,
	              Eigen::VectorXd& lb,
	              Eigen::VectorXd& ub,
	              int max_rounds,
	              SparseCutpoolPropagationStats* stats,
	              std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
	              int reason_depth = 0,
	              int reason_trail_offset = 0,
	              const std::vector<char>* pending_cols = nullptr) -> bool {
	        const int n_local = static_cast<int>(lb.size());
	        const std::size_t nrows = std::min(rows.size(), rhs.size());
	        if (n_local != n || ub.size() != n_local || nrows == 0) return true;
	        const double tol = std::max(1e-9, opt.lp_tol);
	        const int rounds = std::max(1, max_rounds);
        for (int round = 0; round < rounds; ++round) {
          int round_tightened = 0;
          for (std::size_t row_id = 0; row_id < nrows; ++row_id) {
            const auto& row = rows[row_id];
	            if (row.size() != n || row.nonZeros() <= 0 ||
	                !std::isfinite(rhs[row_id])) {
	              continue;
	            }
	            if (pending_cols != nullptr) {
	              bool touches_pending = false;
	              for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
	                const int j = static_cast<int>(it.index());
	                if (j >= 0 && j < static_cast<int>(pending_cols->size()) &&
	                    (*pending_cols)[static_cast<std::size_t>(j)] != 0) {
	                  touches_pending = true;
	                  break;
	                }
	              }
	              if (!touches_pending) continue;
	            }
	            if (stats != nullptr) ++stats->rows_scanned;
            double min_activity = 0.0;
            int ninf_min = 0;
            double capacity_threshold = tol;
            for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
              const int j = static_cast<int>(it.index());
              const double a = it.value();
              if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
              const double bound = a > 0.0 ? lb[j] : ub[j];
              if (!std::isfinite(bound)) {
                ++ninf_min;
              } else {
                min_activity += a * bound;
              }
              const double range =
                  std::isfinite(lb[j]) && std::isfinite(ub[j])
                      ? std::max(0.0, ub[j] - lb[j])
                      : kInf;
              if (std::isfinite(range)) {
                capacity_threshold =
                  std::max(capacity_threshold, std::abs(a) * range);
              }
            }
            if (!std::isfinite(min_activity) || ninf_min > 1) continue;
            const double capacity = rhs[row_id] - min_activity;
            if (stats != nullptr) {
              stats->max_capacity = std::max(stats->max_capacity, capacity);
              stats->min_capacity = std::min(stats->min_capacity, capacity);
            }
            if (ninf_min == 0 && capacity < -tol) {
              if (stats != nullptr) ++stats->pruned;
              return false;
            }
            if (ninf_min == 0 && capacity > capacity_threshold + tol) continue;
            if (stats != nullptr) ++stats->active_rows;

            const Eigen::VectorXd row_reason_lb = lb;
            const Eigen::VectorXd row_reason_ub = ub;
            for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
              const int j = static_cast<int>(it.index());
              const double a = it.value();
              if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
              const double min_bound =
                  a > 0.0 ? row_reason_lb[j] : row_reason_ub[j];
              const bool target_is_inf_min = !std::isfinite(min_bound);
              double min_residual_activity = min_activity;
              if (ninf_min == 1) {
                if (!target_is_inf_min) continue;
              } else {
                if (target_is_inf_min) continue;
                min_residual_activity -= a * min_bound;
              }
              const double residual = rhs[row_id] - min_residual_activity;
              auto make_reason = [&](const BranchDomainLiteral& bound,
                                     const BranchDomainLiteral& forbidden) {
                if (reason_bounds_out == nullptr) return;
                DomainReasonBound rb;
                rb.bound = bound;
                rb.reason.reserve(static_cast<std::size_t>(
                    std::max<Eigen::Index>(0, row.nonZeros() - 1)));
                for (Eigen::SparseVector<double>::InnerIterator jt(row); jt;
                     ++jt) {
                  const int k = static_cast<int>(jt.index());
                  const double ak = jt.value();
                  if (k < 0 || k >= n || k == j ||
                      std::abs(ak) <= 1e-12) {
                    continue;
                  }
                  if (ak > 0.0) {
                    if (std::isfinite(row_reason_lb[k])) {
                      rb.reason.push_back(
                          BranchDomainLiteral{k, row_reason_lb[k], true});
                    }
                  } else if (std::isfinite(row_reason_ub[k])) {
                    rb.reason.push_back(
                        BranchDomainLiteral{k, row_reason_ub[k], false});
                  }
                }
                canonicalize_branch_literals(rb.reason);
                rb.trail_pos =
                    reason_trail_offset +
                    static_cast<int>(reason_bounds_out->size());
                rb.depth = reason_depth;
                rb.source_conflict_literal = forbidden;
                rb.has_source_conflict_literal = true;
                rb.source_conflict_clause = rb.reason;
                rb.source_conflict_clause.push_back(forbidden);
                canonicalize_branch_literals(rb.source_conflict_clause);
                rb.has_source_conflict_clause = true;
                const double source_activity =
                    min_residual_activity + a * forbidden.value;
                rb.has_proof_activity_audit =
                    std::isfinite(source_activity) &&
                    std::isfinite(rhs[row_id]);
                rb.proof_activity = source_activity;
                rb.proof_required_activity = rhs[row_id];
                rb.proof_activity_margin = source_activity - rhs[row_id];
                reason_bounds_out->push_back(std::move(rb));
              };
              if (a > 0.0) {
                double new_ub = residual / a;
                bool accept_ub = false;
                if (strict_highs_root_fixed_point) {
                  accept_ub =
                      root_highs_adjusted_ub(j, new_ub, lb, ub, new_ub);
                } else {
                  if (is_root_integral_or_implied(j)) {
                    new_ub = std::floor(new_ub + opt.int_tol);
                  }
                  accept_ub =
                      std::isfinite(new_ub) && new_ub < ub[j] - tol;
                }
                if (accept_ub) {
                  make_reason(BranchDomainLiteral{j, new_ub, false},
                              BranchDomainLiteral{
                                  j,
                                  is_root_integral_or_implied(j)
                                      ? std::floor(new_ub + opt.int_tol) + 1.0
                                      : new_ub + std::max(1e-7, 10.0 * tol),
                                  true});
                  ub[j] = new_ub;
                  ++round_tightened;
                  if (stats != nullptr) ++stats->tightened;
                }
              } else {
                double new_lb = residual / a;
                bool accept_lb = false;
                if (strict_highs_root_fixed_point) {
                  accept_lb =
                      root_highs_adjusted_lb(j, new_lb, lb, ub, new_lb);
                } else {
                  if (is_root_integral_or_implied(j)) {
                    new_lb = std::ceil(new_lb - opt.int_tol);
                  }
                  accept_lb =
                      std::isfinite(new_lb) && new_lb > lb[j] + tol;
                }
                if (accept_lb) {
                  make_reason(BranchDomainLiteral{j, new_lb, true},
                              BranchDomainLiteral{
                                  j,
                                  is_root_integral_or_implied(j)
                                      ? std::ceil(new_lb - opt.int_tol) - 1.0
                                      : new_lb - std::max(1e-7, 10.0 * tol),
                                  false});
                  lb[j] = new_lb;
                  ++round_tightened;
                  if (stats != nullptr) ++stats->tightened;
                }
              }
            }
            if (!bounds_consistent(lb, ub)) {
              if (stats != nullptr) ++stats->pruned;
              return false;
            }
          }
          if (round_tightened == 0) break;
        }
        return true;
      };
		  auto capture_variable_bound_cutpool_rows =
		      [&](const LPModel& with_rows, int first_row, int last_row,
		          const char* phase) {
        if (!derive_dynamic_root_lp_rows_as_vb_source) return;
		    first_row = std::max(0, first_row);
	    last_row = std::min<int>(last_row, static_cast<int>(with_rows.A.rows()));
	    if (first_row >= last_row) return;
	    Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = with_rows.A;
	    Arow.makeCompressed();
	    const std::size_t old_count = variable_bound_cutpool_rows.size();
	    for (int r = first_row; r < last_row; ++r) {
	      if (!std::isfinite(with_rows.b[r])) continue;
	      Eigen::SparseVector<double> row(n);
	      row.reserve(Arow.outerIndexPtr()[r + 1] - Arow.outerIndexPtr()[r]);
	      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
	           it; ++it) {
	        if (std::abs(it.value()) <= 1e-12) continue;
	        row.coeffRef(static_cast<int>(it.col())) = it.value();
	      }
	      row.finalize();
	      if (row.nonZeros() == 0) continue;
          append_variable_bound_sparse_cutpool_row(row, with_rows.b[r]);
	    }
	    if (variable_bound_cutpool_rows.size() != old_count) {
	      refresh_variable_bound_table(root_lp, phase);
	    }
	  };
		  auto audit_variable_bound_cutpool_row_range =
		      [&](const LPModel& with_rows, int first_row, int last_row,
		          const char* phase) {
	    if (bc_env_options().value("MIPSOLVERS_BC_CONF") == nullptr) return;
	    VariableBoundTable scratch_table = variable_bound_table;
	    BinaryImplicationGraph scratch_graph = implication_graph;
	    NativeVariableBoundSourceStats audit_stats =
	        augment_variable_bound_table_from_cut_rows(
	            with_rows, root_implied_integer_cols, first_row, last_row,
	            scratch_table, &scratch_graph, 100, std::max(1e-9, opt.lp_tol));
	    fmt::print(stderr,
	               "[B&C-VB-SRC-AUDIT] phase={} rows={} mixed={} cutVUB={} "
	               "cutVLB={} liveVarbounds={} auditVarbounds={} "
	               "auditImplArcs={}\n",
	               phase != nullptr ? phase : "cutpool_audit",
	               audit_stats.cut_rows_scanned, audit_stats.cut_mixed_rows,
	               audit_stats.cut_vub_candidates,
	               audit_stats.cut_vlb_candidates,
	               variable_bound_table.size(), scratch_table.size(),
	               scratch_graph.size());
	  };
  RootImpliedIntegerArtifactStats artifact_stats;
  {
    auto t_cq0 = std::chrono::steady_clock::now();
    const std::size_t edges =
        clique_table.build(root_lp, 256, &root_implied_integer_cols);
		    if (!strict_highs_root_fixed_point) {
		      variable_bound_source_stats = build_variable_bound_table_from_rows(
		          root_lp, root_implied_integer_cols, variable_bound_table,
		          &implication_graph, 512);
		    }
        replay_root_presolve_varbound_sources();
		    sync_variable_bound_stats();
	        trace_native_presolve_state_conformance(
            "root_initial_source", root_lp, root_implied_integer_cols,
            variable_bound_table);
    if (!strict_highs_root_fixed_point) {
      artifact_stats = build_implied_integer_row_artifacts(
          root_lp, root_implied_integer_cols, clique_table,
          implication_graph, 256, 96, 250000, &root.x_relax);
    }
    if (!strict_highs_root_fixed_point && use_objective_propagation) {
      objective_propagation.setup(root_lp, &root_implied_integer_cols,
                                  &clique_table, &implication_graph,
                                  objective_propagation_build_policy());
      const auto published = publish_objective_implied_event_source(
          root_lp, objective_propagation, implication_graph, clique_table,
          opt.int_tol);
      out.bc_stats.objective_implied_event_published_implications +=
          published.implications_added;
      out.bc_stats.objective_implied_event_published_clique_edges +=
          published.clique_edges_added;
      if (published.implications_added > 0 || published.clique_edges_added > 0) {
        objective_propagation.setup(root_lp, &root_implied_integer_cols,
                                    &clique_table, &implication_graph,
                                    objective_propagation_build_policy());
      }
    }
    if (opt.verbose && reduced_initial_solution.size() == n) {
      const double witness_tol =
          std::max(1e-9, 10.0 * std::max(opt.int_tol, opt.lp_tol));
      for (int trigger = 0; trigger < n; ++trigger) {
        const double trigger_value = reduced_initial_solution[trigger];
        if (std::abs(trigger_value) > witness_tol &&
            std::abs(trigger_value - 1.0) > witness_tol) {
          continue;
        }
        const bool trigger_one = trigger_value > 0.5;
        auto implications =
            implication_graph.implications(trigger, trigger_one);
        for (const auto* arc = implications.first;
             arc != implications.second; ++arc) {
          if (arc == nullptr || arc->var_idx < 0 || arc->var_idx >= n) continue;
          const double target_value = reduced_initial_solution[arc->var_idx];
          const bool valid = arc->is_lb
              ? target_value >= arc->value - witness_tol
              : target_value <= arc->value + witness_tol;
          if (valid) continue;
          fmt::print(stderr,
                     "[B&C-WITNESS-INVALID-GLOBAL-IMPLICATION] trigger={} "
                     "trigger_value={} target={} side={} bound={:.17g} "
                     "witness_target={:.17g}\n",
                     trigger, trigger_one ? 1 : 0, arc->var_idx,
                     arc->is_lb ? "lb" : "ub", arc->value, target_value);
        }
      }
    }
    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
      auto t_cq1 = std::chrono::steady_clock::now();
      fmt::print(stderr,
                 "[B&C CLIQUE] n_cols={} n_edges={} lit_edges={} base_edges={} "
	                 "impl_rows={} impl_lit_rows={} impl_bound_cand={} "
	                 "impl_bound_imp={} impl_conflicts={} impl_arcs={} "
	                 "impl_clique_edges={} lit_pair={}/{} obj_lit_pair={}/{} "
	                 "varbounds={} vb_src={} vb_single={} vb_cand={} "
                 "cutRows={} cutMixed={} cutVUB={} cutVLB={} "
	                 "vub={}/{} vlb={}/{} vb_exp={} cutExp={} "
                 "build={:.1f}ms\n",
                 clique_table.n_cols(), clique_table.n_edges(),
                 clique_table.n_literal_edges(), edges,
                 artifact_stats.row_sides_scanned,
                 artifact_stats.row_sides_with_binary_literals,
                 artifact_stats.bound_candidates,
                 artifact_stats.bound_improvements,
                 artifact_stats.conflicts_added,
                 artifact_stats.implications_added,
                 artifact_stats.clique_edges_added,
                 artifact_stats.literal_pair_conflicts,
                 artifact_stats.literal_pair_tests,
                 artifact_stats.objective_literal_pair_conflicts,
                 artifact_stats.objective_literal_pair_tests,
                 variable_bound_table.size(),
	                 variable_bound_source_stats.row_sides_scanned,
	                 variable_bound_source_stats.single_binary_row_sides,
	                 variable_bound_source_stats.candidates,
	                 variable_bound_source_stats.cut_rows_scanned,
	                 variable_bound_source_stats.cut_mixed_rows,
	                 variable_bound_source_stats.cut_vub_candidates,
	                 variable_bound_source_stats.cut_vlb_candidates,
	                 variable_bound_table.stats().vub_accepted,
	                 variable_bound_table.stats().vub_attempts,
	                 variable_bound_table.stats().vlb_accepted,
	                 variable_bound_table.stats().vlb_attempts,
	                 variable_bound_source_stats.exported_implications,
	                 variable_bound_source_stats.cut_exported_implications,
                 std::chrono::duration<double, std::milli>(t_cq1 - t_cq0).count());
    }
  }

  Eigen::VectorXd verified_root_cutoff_x;
  double verified_root_cutoff_obj = kInf;
  bool has_verified_root_cutoff = false;
  std::string verified_root_cutoff_source = "none";
  auto try_register_root_cutoff =
      [&](const char* source, const Eigen::VectorXd& candidate) {
    if (candidate.size() != n || base_lp.sense != Sense::Minimize) return;
    Eigen::VectorXd x = clamp_to_bounds(candidate, root.lb, root.ub);
    const double feas_tol =
        std::max(1e-7, 10.0 * opt.heuristic_feasibility_tol);
    if (!satisfies_with_bounds(base_lp, x, root.lb, root.ub, feas_tol)) {
      return;
    }
    const double obj = objective_value(base_lp.c, x, base_lp.sense);
    if (!std::isfinite(obj)) return;
    // Apply the incumbent quality contract before learning from the cutoff.
    // A repair artefact rejected for publication must not tighten the root
    // domain or populate implication/conflict structures beforehand.
    if (!incumbent_quality_acceptable(obj, root.bound)) {
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C] root cutoff rejected by quality gate: obj={:.6g} "
                   "vs bound={:.6g}\n",
                   obj, root.bound);
      }
      return;
    }
    if (!has_verified_root_cutoff || obj + 1e-9 < verified_root_cutoff_obj) {
      verified_root_cutoff_x = std::move(x);
      verified_root_cutoff_obj = obj;
      has_verified_root_cutoff = true;
      verified_root_cutoff_source = source != nullptr ? source : "root_cutoff";
    }
  };

		  if (reduced_initial_solution.size() == n) {
	    // HiGHS runSetup() accepts a feasible MIP start before evaluateRootNode()
	    // and exposes it as upper_limit for objective propagation and root
	    // reduced-cost fixing.  In strict tree-exhaustion mode this cutoff is
	    // propagation-only: it is not a root gap certificate or termination
	    // shortcut.
	    try_register_root_cutoff("reduced_initial_solution",
	                             reduced_initial_solution);
		  }
	  if (!strict_highs_lp_contract && heuristic_x_orig.size() > 0 &&
      std::isfinite(heuristic_obj)) {
	    Eigen::VectorXd x_fwd = heuristic_x_orig;
    if (highs_mip_model_reduced) {
      x_fwd = highs_presolve_forward_map(highs_mip_ps, x_fwd);
    }
#ifdef MIPSOLVERS_HAVE_PAPILO
    if (papilo_ps.success && !papilo_ps.orig_to_reduced_col.empty()) {
      x_fwd = papilo_forward_map(papilo_ps, x_fwd);
    }
#endif
    if (native_model_reduced) {
      x_fwd = native_ps.forward_map(x_fwd);
    }
    try_register_root_cutoff("forward_mapped_heuristic", x_fwd);
  }
		  const bool has_root_cutoff_for_propagation =
		      has_verified_root_cutoff;
  const double root_cutoff_raw_for_propagation =
      has_verified_root_cutoff ? verified_root_cutoff_obj : heuristic_obj;
  const double root_cutoff_for_propagation =
      has_root_cutoff_for_propagation
          ? highs_style_incumbent_upper_limit(base_lp,
                                             root_cutoff_raw_for_propagation,
                                             opt.int_tol)
          : kInf;
	  const std::string root_cutoff_source_for_propagation =
		      has_verified_root_cutoff ? verified_root_cutoff_source
		                               : std::string("none");
    root_redcost_lurking_cutoff_available =
        has_root_cutoff_for_propagation &&
        std::isfinite(root_cutoff_for_propagation);
    if (root_redcost_lurking_cutoff_available) {
      (void)refresh_root_redcost_lurking_bounds("root_cutoff_available");
    }
	  if (root_cutoff_domain_commit_allowed &&
	      has_root_cutoff_for_propagation &&
	      std::isfinite(root_cutoff_for_propagation) &&
	      use_objective_propagation && base_lp.sense == Sense::Minimize) {
	    note_incumbent_cutoff_learning();
	    const auto clique_stats = extract_objective_cutoff_cliques(
	        base_lp, root_implied_integer_cols, root.lb, root.ub,
	        root_cutoff_for_propagation, clique_table, opt.int_tol,
	        &objective_propagation, &conflict_pool, nullptr);
	    const auto stats = extract_objective_cutoff_artifacts(
	        base_lp, root_implied_integer_cols, root.lb, root.ub,
	        root_cutoff_for_propagation, implication_graph, clique_table,
	        opt.int_tol, &objective_propagation, &conflict_pool, nullptr);
	    const std::uint64_t extracted =
	        stats.implications_added + stats.clique_edges_added +
	        stats.conflict_clauses_added + clique_stats.clique_edges_added +
	        clique_stats.conflict_clauses_added;
	    if (extracted > 0) {
	      out.bc_stats.rc_binary_implications_learned +=
	          stats.implications_added;
	      out.bc_stats.cutoff_conflict_clauses_learned +=
	          stats.conflict_clauses_added + clique_stats.conflict_clauses_added;
	      note_global_domain_learning(extracted);
	      note_objective_artifact_learning(extracted);
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
	      if (published_total > 0) {
	        note_global_domain_learning(published_total);
	        note_objective_artifact_learning(published_total);
	        objective_propagation.setup(base_lp, &root_implied_integer_cols,
	                                    &clique_table, &implication_graph,
	                                    objective_propagation_build_policy());
	      }
	    }
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
	        extracted > 0) {
	      fmt::print(stderr,
	                 "[B&C ROOT-CUTOFF-SOURCE] source={} cap={:.6g} "
	                 "cand={} eventCand={} conflicts={} confQ={} impl={} "
	                 "clq={} cutClq={} cutConf={} arcs={} clqEdges={}\n",
	                 root_cutoff_source_for_propagation,
	                 stats.cutoff_capacity, stats.candidates,
	                 stats.implied_event_candidates, stats.conflicts_seen,
	                 stats.conflict_clauses_added +
	                     clique_stats.conflict_clauses_added,
	                 stats.implications_added, stats.clique_edges_added,
	                 clique_stats.clique_edges_added,
	                 clique_stats.conflict_clauses_added,
	                 implication_graph.size(), clique_table.n_literal_edges());
	    }
	  }

	  out.bc_stats.root_presolved_rows =
	      static_cast<int>(root_lp.A.rows()) + static_cast<int>(root_lp.Aeq.rows());
  out.bc_stats.root_presolved_cols = n;
#ifdef MIPSOLVERS_HAVE_PAPILO
  if (papilo_ps.success && papilo_ps.reduced_compact_rows > 0) {
    out.bc_stats.root_presolved_compact_rows = papilo_ps.reduced_compact_rows;
    out.bc_stats.root_two_sided_rows_expanded =
        papilo_ps.reduced_two_sided_rows_expanded;
  } else
#endif
  {
    out.bc_stats.root_presolved_compact_rows =
        out.bc_stats.root_presolved_rows;
    out.bc_stats.root_two_sided_rows_expanded = 0;
  }
  out.bc_stats.root_implied_integer_cols = root_implied_integer_count;
  for (const auto& v : root_lp.vars) {
    if (v.type == VarType::Binary) {
      ++out.bc_stats.root_presolved_binary_cols;
    } else if (v.type == VarType::Integer) {
      ++out.bc_stats.root_presolved_integer_cols;
    } else {
      ++out.bc_stats.root_presolved_continuous_cols;
    }
  }
  out.bc_stats.root_clique_edges = clique_table.n_literal_edges();
  out.bc_stats.root_implication_arcs =
      static_cast<std::uint64_t>(implication_graph.size());
  out.bc_stats.variable_bound_table_size =
      static_cast<std::uint64_t>(variable_bound_table.size());
  out.bc_stats.variable_bound_vub_attempts =
      variable_bound_table.stats().vub_attempts;
  out.bc_stats.variable_bound_vub_accepted =
      variable_bound_table.stats().vub_accepted;
  out.bc_stats.variable_bound_vub_replaced =
      variable_bound_table.stats().vub_replaced;
  out.bc_stats.variable_bound_vlb_attempts =
      variable_bound_table.stats().vlb_attempts;
  out.bc_stats.variable_bound_vlb_accepted =
      variable_bound_table.stats().vlb_accepted;
  out.bc_stats.variable_bound_vlb_replaced =
      variable_bound_table.stats().vlb_replaced;
  out.bc_stats.variable_bound_mir_attempts =
      variable_bound_table.stats().mir_attempts;
	  out.bc_stats.variable_bound_mir_strengthened =
	      variable_bound_table.stats().mir_strengthened;
	  out.bc_stats.variable_bound_exported_implications =
	      variable_bound_source_stats.exported_implications;
	  out.bc_stats.variable_bound_cut_rows_scanned =
	      variable_bound_source_stats.cut_rows_scanned;
	  out.bc_stats.variable_bound_cut_mixed_rows =
	      variable_bound_source_stats.cut_mixed_rows;
	  out.bc_stats.variable_bound_cut_vub_candidates =
	      variable_bound_source_stats.cut_vub_candidates;
	  out.bc_stats.variable_bound_cut_vlb_candidates =
	      variable_bound_source_stats.cut_vlb_candidates;
	  out.bc_stats.variable_bound_cut_exported_implications =
	      variable_bound_source_stats.cut_exported_implications;

	  std::uint64_t root_row_dual_nnz = 0;
  double root_row_dual_abs_sum = 0.0;
  double root_row_dual_abs_max = 0.0;
  if (root_relax.row_duals.size() > 0) {
    for (int i = 0; i < root_relax.row_duals.size(); ++i) {
      const double a = std::abs(root_relax.row_duals[i]);
      if (!std::isfinite(a) || a <= std::max(1e-12, opt.lp_tol)) continue;
      ++root_row_dual_nnz;
      root_row_dual_abs_sum += a;
      root_row_dual_abs_max = std::max(root_row_dual_abs_max, a);
    }
  }
  RootReducedCostAudit root_rc_audit;
  if (root.basis_hint && root.basis_hint->cached_reduced_costs &&
      root.basis_hint->cached_reduced_costs->size() >= n) {
    root_rc_audit = audit_root_reduced_costs(
        root_lp.vars, &root_implied_integer_cols, root.x_relax,
        *root.basis_hint->cached_reduced_costs,
        root.basis_hint->basis_indices(), root.lb, root.ub, root.bound,
        has_root_cutoff_for_propagation ? root_cutoff_for_propagation : kInf,
        opt.int_tol);
    out.bc_stats.root_reduced_cost_nonzero = root_rc_audit.rc_nonzero;
    out.bc_stats.root_reduced_cost_integer_nonzero =
        root_rc_audit.rc_integer_nonzero;
    out.bc_stats.root_reduced_cost_implied_integer_nonzero =
        root_rc_audit.rc_implied_integer_nonzero;
    out.bc_stats.root_reduced_cost_at_lower = root_rc_audit.rc_at_lower;
    out.bc_stats.root_reduced_cost_at_upper = root_rc_audit.rc_at_upper;
    out.bc_stats.root_reduced_cost_cutoff_fix_candidates =
        root_rc_audit.cutoff_fix_candidates;
    out.bc_stats.root_reduced_cost_abs_sum = root_rc_audit.rc_abs_sum;
    out.bc_stats.root_reduced_cost_abs_max = root_rc_audit.rc_abs_max;
    out.bc_stats.root_reduced_cost_gap = root_rc_audit.gap;
  }
  if (opt.verbose) {
    fmt::print(stderr,
               "[ROOT-AUDIT] native presolved rows={} compact_rows={} "
               "two_sided_expanded={} finite_lhs={} cols={} bin={} int={} implied_int={} "
               "cont={} obj_clq_part={} obj_clq_terms={} clq_edges={} "
               "impl_arcs={} varbounds={} vub={}/{} vlb={}/{} "
               "cutRows={} cutMixed={} cutVUB={} cutVLB={} "
               "cutoff={} source={}\n",
               out.bc_stats.root_presolved_rows,
               out.bc_stats.root_presolved_compact_rows,
               out.bc_stats.root_two_sided_rows_expanded,
#ifdef MIPSOLVERS_HAVE_PAPILO
               papilo_ps.success ? papilo_ps.reduced_rows_with_finite_lhs : 0,
#else
               0,
#endif
               out.bc_stats.root_presolved_cols,
               out.bc_stats.root_presolved_binary_cols,
               out.bc_stats.root_presolved_integer_cols,
               out.bc_stats.root_implied_integer_cols,
               out.bc_stats.root_presolved_continuous_cols,
               static_cast<std::uint64_t>(objective_propagation.partitions.size()),
               std::accumulate(
                   objective_propagation.partitions.begin(),
                   objective_propagation.partitions.end(), std::uint64_t{0},
                   [](std::uint64_t s,
                      const ObjectivePropagationState::Partition& p) {
                     return s + static_cast<std::uint64_t>(p.terms.size());
                   }),
               out.bc_stats.root_clique_edges, out.bc_stats.root_implication_arcs,
               out.bc_stats.variable_bound_table_size,
               out.bc_stats.variable_bound_vub_accepted,
               out.bc_stats.variable_bound_vub_attempts,
               out.bc_stats.variable_bound_vlb_accepted,
               out.bc_stats.variable_bound_vlb_attempts,
               out.bc_stats.variable_bound_cut_rows_scanned,
               out.bc_stats.variable_bound_cut_mixed_rows,
               out.bc_stats.variable_bound_cut_vub_candidates,
               out.bc_stats.variable_bound_cut_vlb_candidates,
               has_root_cutoff_for_propagation
                 ? fmt::format("{:.2f}", root_cutoff_for_propagation)
                 : std::string("none"),
               root_cutoff_source_for_propagation);
  }
  trace_highs_native_varbound_diff("root_audit", highs_presolve_side_state,
                                   &root_lp,
                                   variable_bound_table,
                                   bc_conformance_trace_terms(
                                       "MIPSOLVERS_HIGHS_VB_DIFF_TERMS", 8));
  if (opt.verbose) {
    fmt::print(stderr,
               "[OBJ-CLQ-AUDIT] cand_lit={} pair_tests={} pair_conf={} "
               "literal_conf={} implication_conf={} artifact_pair={}/{} "
               "artifact_obj_pair={}/{}\n",
               objective_propagation.objective_clique_candidate_literals,
               objective_propagation.objective_clique_pair_tests,
               objective_propagation.objective_clique_pair_conflicts,
               objective_propagation.objective_clique_literal_conflicts,
               objective_propagation.objective_clique_implication_conflicts,
               artifact_stats.literal_pair_conflicts,
               artifact_stats.literal_pair_tests,
               artifact_stats.objective_literal_pair_conflicts,
               artifact_stats.objective_literal_pair_tests);
    fmt::print(stderr,
               "[ROOT-RC-AUDIT] rc_nz={} rc_int_nz={} rc_implint_nz={} "
               "at_lb={} at_ub={} "
               "cutoff_fix_candidates={} rc_abs_sum={:.3e} rc_abs_max={:.3e} "
               "gap={} rowdual_nz={} rowdual_abs_sum={:.3e} "
               "rowdual_abs_max={:.3e}\n",
               root_rc_audit.rc_nonzero, root_rc_audit.rc_integer_nonzero,
               root_rc_audit.rc_implied_integer_nonzero,
               root_rc_audit.rc_at_lower, root_rc_audit.rc_at_upper,
               root_rc_audit.cutoff_fix_candidates, root_rc_audit.rc_abs_sum,
               root_rc_audit.rc_abs_max,
               std::isfinite(root_rc_audit.gap)
                   ? fmt::format("{:.3e}", root_rc_audit.gap)
                   : std::string("inf"),
               root_row_dual_nnz, root_row_dual_abs_sum, root_row_dual_abs_max);
  }

  auto conformance_objective_terms = [&]() -> std::uint64_t {
    return std::accumulate(
        objective_propagation.partitions.begin(),
        objective_propagation.partitions.end(), std::uint64_t{0},
        [](std::uint64_t sum,
           const ObjectivePropagationState::Partition& part) {
          return sum + static_cast<std::uint64_t>(part.terms.size());
        });
  };

  auto conformance_source_total = [&]() -> std::uint64_t {
    return static_cast<std::uint64_t>(std::max(0, root_implied_integer_count)) +
           clique_table.n_literal_edges() +
           static_cast<std::uint64_t>(implication_graph.size()) +
           static_cast<std::uint64_t>(variable_bound_table.size()) +
           static_cast<std::uint64_t>(objective_propagation.partitions.size()) +
           conformance_objective_terms() +
           static_cast<std::uint64_t>(
               objective_propagation.implied_events.size());
  };

  auto bc_conf_diag_enabled = [&]() -> bool {
    return opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr;
  };

  auto record_conformance_source = [&](const char* phase) -> std::uint64_t {
    const std::uint64_t obj_terms = conformance_objective_terms();
    const std::uint64_t sources = conformance_source_total();
    ++out.bc_stats.conformance_source_phases;
    if (sources > 0) ++out.bc_stats.conformance_source_nonempty_phases;
    if (bc_conf_diag_enabled()) {
      fmt::print(stderr,
                 "[B&C-CONF] phase={} source implInt={} clqEdges={} "
                 "implArcs={} varbounds={} objClq={}/{} objEvt={} evtMax={:.6g} "
                 "evtAggMax={:.6g} total={}\n",
                 phase != nullptr ? phase : "source",
                 root_implied_integer_count, clique_table.n_literal_edges(),
                 implication_graph.size(), variable_bound_table.size(),
                 objective_propagation.partitions.size(), obj_terms,
                 objective_propagation.implied_events.size(),
                 objective_propagation.max_implied_event_delta,
                 objective_propagation.max_implied_event_aggregate_delta,
                 sources);
    }
    return sources;
  };

  auto count_bound_changes = [&](const Eigen::VectorXd& old_lb,
                                 const Eigen::VectorXd& old_ub,
                                 const Eigen::VectorXd& new_lb,
                                 const Eigen::VectorXd& new_ub)
      -> std::pair<std::uint64_t, double> {
    if (old_lb.size() != new_lb.size() || old_ub.size() != new_ub.size()) {
      return {0, 0.0};
    }
    const double tol = std::max(1e-9, opt.int_tol);
    std::uint64_t count = 0;
    double max_move = 0.0;
    for (int j = 0; j < old_lb.size(); ++j) {
      const double lb_move = new_lb[j] - old_lb[j];
      if (std::isfinite(lb_move) && lb_move > tol) {
        ++count;
        max_move = std::max(max_move, lb_move);
      }
      const double ub_move = old_ub[j] - new_ub[j];
      if (std::isfinite(ub_move) && ub_move > tol) {
        ++count;
        max_move = std::max(max_move, ub_move);
      }
    }
    return {count, max_move};
  };
  auto count_changed_cols = [&](const Eigen::VectorXd& old_lb,
                                const Eigen::VectorXd& old_ub,
                                const Eigen::VectorXd& new_lb,
                                const Eigen::VectorXd& new_ub)
      -> std::pair<std::uint64_t, double> {
    if (old_lb.size() != new_lb.size() || old_ub.size() != new_ub.size()) {
      return {0, 0.0};
    }
    const double tol = std::max(1e-9, opt.int_tol);
    std::uint64_t count = 0;
    double max_move = 0.0;
    for (int j = 0; j < old_lb.size(); ++j) {
      bool changed_col = false;
      const double lb_move = new_lb[j] - old_lb[j];
      if (std::isfinite(lb_move) && lb_move > tol) {
        changed_col = true;
        max_move = std::max(max_move, lb_move);
      }
      const double ub_move = old_ub[j] - new_ub[j];
      if (std::isfinite(ub_move) && ub_move > tol) {
        changed_col = true;
        max_move = std::max(max_move, ub_move);
      }
      if (changed_col) ++count;
    }
    return {count, max_move};
  };
  auto collect_changed_col_sides =
      [&](const Eigen::VectorXd& old_lb, const Eigen::VectorXd& old_ub,
          const Eigen::VectorXd& new_lb,
          const Eigen::VectorXd& new_ub) -> ChangedColSides {
    const double tol = std::max(1e-9, opt.int_tol);
    return collect_changed_col_sides_with_tol(old_lb, old_ub, new_lb, new_ub,
                                              tol);
  };

  auto record_conformance_propagation =
      [&](const char* phase,
          std::uint64_t sources,
          std::uint64_t bound_changes,
          double max_bound_move,
          std::uint64_t prunes,
          bool resolved,
          bool resolve_ok,
          double old_bound,
          double new_bound,
          const std::string& detail) {
    ++out.bc_stats.conformance_propagation_phases;
    if (bound_changes > 0) ++out.bc_stats.conformance_bound_change_phases;
    out.bc_stats.conformance_bound_changes += bound_changes;
    out.bc_stats.conformance_domain_prunes += prunes;
    if (resolved) {
      ++out.bc_stats.conformance_resolve_attempts;
      if (!resolve_ok) ++out.bc_stats.conformance_resolve_failures;
    }
    const double lift =
        (std::isfinite(old_bound) && std::isfinite(new_bound))
            ? std::max(0.0, new_bound - old_bound)
            : 0.0;
    if (lift > std::max(1e-8, opt.lp_tol)) {
      ++out.bc_stats.conformance_resolve_bound_moves;
      out.bc_stats.conformance_bound_lift_sum += lift;
      out.bc_stats.conformance_bound_lift_max =
          std::max(out.bc_stats.conformance_bound_lift_max, lift);
    }
    if (bc_conf_diag_enabled()) {
      fmt::print(stderr,
                 "[B&C-CONF] phase={} propagate sources={} "
                 "bound_changes={} max_move={:.6g} prunes={} "
                 "resolved={} resolve_ok={} bound={:.10g}->{:.10g} "
                 "lift={:.6g} {}\n",
                 phase != nullptr ? phase : "propagation", sources,
                 bound_changes, max_bound_move, prunes, resolved ? 1 : 0,
                 resolve_ok ? 1 : 0, old_bound, new_bound, lift, detail);
    }
  };

  const bool conformance_diag_enabled =
      opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
      bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr;

  auto record_objective_capacity_conformance =
      [&](const char* phase,
          const Eigen::VectorXd& lb,
          const Eigen::VectorXd& ub,
          double incumbent,
          double proof_lower_bound) {
    if (!conformance_diag_enabled) {
      return ObjectivePropagationState::CapacityConformanceAudit{};
    }
    const auto audit = objective_propagation.audit_capacity_conformance(
        base_lp, lb, ub, incumbent, proof_lower_bound, opt.int_tol);
    if (!audit.valid) return audit;

    ++out.bc_stats.conformance_objective_capacity_phases;
    out.bc_stats.conformance_objective_capacity_active_events +=
        audit.active_events;
    out.bc_stats.conformance_objective_capacity_active_tight_events +=
        audit.active_tightening_events;
    out.bc_stats.conformance_objective_capacity_one_missing_events +=
        audit.one_missing_events;
    out.bc_stats.conformance_objective_capacity_one_missing_keys +=
        audit.one_missing_keys;
    out.bc_stats.conformance_objective_capacity_raw_exceed_keys +=
        audit.one_missing_raw_exceed_keys;
    out.bc_stats.conformance_objective_capacity_proof_exceed_keys +=
        audit.one_missing_proof_exceed_keys;
    out.bc_stats.conformance_objective_capacity_max_one_missing =
        std::max(out.bc_stats.conformance_objective_capacity_max_one_missing,
                 audit.max_one_missing_contribution);
    out.bc_stats.conformance_objective_capacity_max_proof_excess =
        std::max(out.bc_stats.conformance_objective_capacity_max_proof_excess,
                 audit.max_proof_excess);

    if (conformance_diag_enabled || audit.active_tightening_events > 0 ||
        audit.one_missing_raw_exceed_keys > 0 ||
        audit.one_missing_proof_exceed_keys > 0 ||
        audit.one_missing_events > 0) {
      fmt::print(stderr,
                 "[B&C-CONF] phase={} objcap raw_lb={:.10g} "
                 "raw_cap={:.10g} proof_lb={:.10g} proof_cap={:.10g} "
                 "maxD={:.6g} aggD={:.6g} tgt={} active={}/{} "
                 "one={}/{} rawEx={} proofEx={} maxOne={:.6g} "
                 "proofExMax={:.6g}\n",
                 phase != nullptr ? phase : "objective_capacity",
                 audit.raw_lower, audit.raw_capacity, audit.proof_lower,
                 audit.proof_capacity, audit.max_objective_delta,
                 audit.max_aggregate_delta, audit.target_tightening_events,
                 audit.active_tightening_events, audit.active_events,
                 audit.one_missing_events, audit.one_missing_keys,
                 audit.one_missing_raw_exceed_keys,
                 audit.one_missing_proof_exceed_keys,
                 audit.max_one_missing_contribution, audit.max_proof_excess);
    }
    return audit;
  };

  std::function<double(const Node&)> domain_objective_lower_bound =
      [&](const Node& node) -> double {
    return objective_propagation.objective_lower_bound(base_lp, node.lb,
                                                       node.ub);
  };

  auto apply_domain_objective_lower_bound =
      [&](Node& node, const char* phase) -> double {
    const double domain_lb = domain_objective_lower_bound(node);
    if (!std::isfinite(domain_lb)) return 0.0;
    const double old_bound = node.bound;
    if (std::isfinite(old_bound) && domain_lb <= old_bound) return 0.0;
    node.bound = domain_lb;
    if (node.estimate < node.bound) node.estimate = node.bound;
    const double lift =
        std::isfinite(old_bound) ? std::max(0.0, node.bound - old_bound) : 0.0;
    if (lift > std::max(1e-8, opt.lp_tol)) {
      ++out.bc_stats.objective_domain_bound_lift_nodes;
      out.bc_stats.objective_domain_bound_lift_sum += lift;
      out.bc_stats.objective_domain_bound_lift_max =
          std::max(out.bc_stats.objective_domain_bound_lift_max, lift);
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C-OBJ-LB] phase={} depth={} bound={:.10g}->{:.10g} "
                   "lift={:.6g}\n",
                   phase != nullptr ? phase : "domain_objective", node.depth,
                   old_bound, node.bound, lift);
      }
    }
    return lift;
  };

  record_conformance_source("root_initial_source");

  bool root_propagation_local_cutoff_tightened = false;
  bool root_propagation_pruned = false;
  {
    const Eigen::VectorXd root_lb_before_prop = root.lb;
    const Eigen::VectorXd root_ub_before_prop = root.ub;
    const LPModel root_lp_before_prop = root_lp;
    const auto root_basis_before_prop = root.basis_hint;
    const auto root_x_before_prop = root.x_relax;
    const double root_bound_before_prop = root.bound;
    const LPRelaxationResult root_relax_before_prop = root_relax;

	    int row_tightened = strict_highs_root_fixed_point
            ? propagate_root_highs_presolved_domain(
                  root.lb, root.ub, opt.bound_propagation_rounds)
            : node_bound_propagation(base_lp, root.lb, root.ub,
                                     opt.bound_propagation_rounds);
	    int clique_tightened = 0;
	    if (!clique_table.empty()) {
	      clique_tightened = clique_table.propagate(root.lb, root.ub);
	    }
	    int conflict_tightened = 0;
	    std::vector<DomainReasonBound> root_conflict_reasons;
	    if (!conflict_pool.empty()) {
	      int pass_conflict_tight = 0;
	      std::vector<BoundChangeInfo> conflict_changes;
	      const bool conflict_ok = conflict_pool.propagate_with_reasons(
	          base_lp.vars, root.lb, root.ub, &pass_conflict_tight,
	          &conflict_changes, &root_conflict_reasons, opt.int_tol);
	      (void)conflict_changes;
	      conflict_tightened += std::max(0, pass_conflict_tight);
	      if (!conflict_ok) {
	        root_propagation_pruned = true;
	      }
	    }
	    int implication_tightened = 0;
	    if (!root_propagation_pruned && !implication_graph.empty()) {
	      implication_tightened = implication_graph.propagate(
	          base_lp.vars, root.lb, root.ub, nullptr,
	          std::max(1e-9, opt.int_tol));
	    }
		    int objective_tightened = 0;
		    int objective_pruned = 0;
		    int proof_target_tightened = 0;
		    int proof_target_pruned = 0;
		    std::vector<DomainReasonBound> root_objective_reasons;
	    if (root_cutoff_domain_commit_allowed &&
          has_root_cutoff_for_propagation && use_objective_propagation) {
      record_objective_capacity_conformance(
          "root_initial_objective_capacity", root.lb, root.ub,
          root_cutoff_for_propagation, root_bound_before_prop);
      if (!root_propagation_pruned) {
        const bool ok = objective_propagation.propagate(
            base_lp, root.lb, root.ub, root_cutoff_for_propagation, opt.int_tol,
            /*depth=*/0, /*trail_offset=*/0, &root_objective_reasons,
            objective_tightened, objective_pruned);
	        if (!ok) {
	          root_propagation_pruned = true;
	        }
	      }
	      root_propagation_local_cutoff_tightened =
          proof_target_tightened > 0 || proof_target_pruned > 0 ||
	          objective_tightened > 0 || objective_pruned > 0;
	    }
	    auto append_initial_root_reason_bounds =
	        [&](std::vector<DomainReasonBound>& reason_bounds) {
	      for (auto& rb : reason_bounds) {
	        if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
	            !std::isfinite(rb.bound.value)) {
	          continue;
	        }
	        append_domain_reason_bound(root, std::move(rb), n,
	                                   &root_lb_before_prop,
	                                   &root_ub_before_prop);
	      }
	      reason_bounds.clear();
	    };
	    append_initial_root_reason_bounds(root_conflict_reasons);
	    append_initial_root_reason_bounds(root_objective_reasons);

	    int followup_row_tightened = 0;
	    int followup_clique_tightened = 0;
	    if (!root_propagation_pruned &&
	        (row_tightened + clique_tightened + conflict_tightened +
	         implication_tightened + objective_tightened) > 0 &&
	        bounds_consistent(root.lb, root.ub)) {
	      followup_row_tightened = strict_highs_root_fixed_point
              ? propagate_root_highs_presolved_domain(
                    root.lb, root.ub, opt.bound_propagation_rounds)
              : node_bound_propagation(base_lp, root.lb, root.ub,
                                       opt.bound_propagation_rounds);
	      if (!clique_table.empty()) {
        followup_clique_tightened = clique_table.propagate(root.lb, root.ub);
      }
    }

	    const int total_root_tightened = row_tightened + clique_tightened +
	        conflict_tightened + implication_tightened +
	        proof_target_tightened + objective_tightened +
	        followup_row_tightened + followup_clique_tightened;
    out.bc_stats.root_objective_cutoff_domain_tightenings +=
        static_cast<std::uint64_t>(std::max(0, proof_target_tightened) +
                                   std::max(0, objective_tightened));
    out.bc_stats.root_objective_cutoff_domain_prunes +=
        static_cast<std::uint64_t>(std::max(0, proof_target_pruned) +
                                   std::max(0, objective_pruned));
    if (!root_propagation_pruned && !bounds_consistent(root.lb, root.ub)) {
      root_propagation_pruned = true;
    }

    const auto root_bound_delta = count_bound_changes(
        root_lb_before_prop, root_ub_before_prop, root.lb, root.ub);
    const std::uint64_t root_conf_sources = conformance_source_total();
	    const std::string root_conf_detail = fmt::format(
	        "row={} clique={} conflict={} impl={} proof_target={} obj={} "
	        "follow_row={} follow_clique={} objective_pruned={}",
	        row_tightened, clique_tightened, conflict_tightened,
	        implication_tightened, proof_target_tightened, objective_tightened,
	        followup_row_tightened, followup_clique_tightened, objective_pruned);

    if (root_propagation_pruned) {
      root.bound = has_root_cutoff_for_propagation
          ? root_cutoff_for_propagation
          : kInf;
      if (has_verified_root_cutoff) root.x_relax = verified_root_cutoff_x;
      record_conformance_propagation(
          "root_initial_propagation", root_conf_sources,
          root_bound_delta.first, root_bound_delta.second, 1,
          /*resolved=*/false, /*resolve_ok=*/true, root_bound_before_prop,
          root.bound, root_conf_detail);
      if (bc_conf_diag_enabled()) {
	        fmt::print(stderr,
	                   "[B&C ROOT-PROP] pruned root by propagation "
	                   "row={} clique={} conflict={} impl={} proof_target={} obj={} "
	                   "follow_row={} follow_clique={} "
	                   "target_pruned={} obj_pruned={} cutoff={:.2f}\n",
	                   row_tightened, clique_tightened, conflict_tightened,
	                   implication_tightened, proof_target_tightened,
	                   objective_tightened,
	                   followup_row_tightened, followup_clique_tightened,
	                   proof_target_pruned, objective_pruned,
	                   root_cutoff_for_propagation);
      }
    } else if (total_root_tightened > 0) {
      apply_node_bounds(root_lp.vars, root.lb, root.ub);
      const double reuse_tol = std::max(1e-7, 10.0 * opt.lp_tol);
      double tightened_bound_violation = 0.0;
      bool root_point_survives_tightening = root.x_relax.size() == n;
      if (root_point_survives_tightening) {
        for (int j = 0; j < n; ++j) {
          if (!std::isfinite(root.x_relax[j])) {
            root_point_survives_tightening = false;
            tightened_bound_violation =
                std::numeric_limits<double>::infinity();
            break;
          }
          tightened_bound_violation = std::max(
              tightened_bound_violation,
              std::max(root.lb[j] - root.x_relax[j],
                       root.x_relax[j] - root.ub[j]));
        }
        root_point_survives_tightening =
            tightened_bound_violation <= reuse_tol;
      }

      // Constraints and objective are unchanged. If the old optimum remains
      // feasible in the tightened box, its primal objective and retained dual
      // bound prove the tightened relaxation has the same optimum.
      if (root_point_survives_tightening) {
        root.x_seed = root.x_relax;
        record_conformance_propagation(
            "root_initial_propagation", root_conf_sources,
            root_bound_delta.first, root_bound_delta.second, 0,
            /*resolved=*/false, /*resolve_ok=*/true, root_bound_before_prop,
            root.bound, root_conf_detail + " reused_root_optimum=1");
        if (opt.verbose) {
          fmt::print(stderr,
                     "[B&C ROOT-PROP] retained root optimum after {} domain "
                     "tightenings (max bound violation {:.3g})\n",
                     total_root_tightened, tightened_bound_violation);
        }
      } else {
        const auto t_prop_resolve0 = std::chrono::steady_clock::now();
        root_solve_opt.time_limit_sec =
            std::max(0.001, bc_remaining_sec());
        LPRelaxationResult prop_relax = solve_lp_relaxation(
            root_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
        const auto t_prop_resolve1 = std::chrono::steady_clock::now();
        out.bc_stats.lp_solves += 1;

	      if (prop_relax.primal.stats.success && prop_relax.primal.x.size() == n &&
	          std::isfinite(prop_relax.dual_bound)) {
	        root_relax = std::move(prop_relax);
	        root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	        root.x_seed = root.x_relax;
	        refresh_root_lp_source_solution();
	        root.basis_hint = root_relax.basis_hint;
        if (root.basis_hint) root.basis_hint->compact_indices_storage();
	        root.bound = root_relax.dual_bound;
	        refresh_root_redcost_lurking_bounds("root_initial_propagation_resolve");
	        if (root_relax.simplex &&
	            root_relax.simplex->basis.cached_sparse_basis) {
          root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
        } else {
          root_sbasis.reset();
        }
        if (root_relax.simplex &&
            root_relax.simplex->basis.cached_sparse_basis) {
          root.has_live_factor_telemetry = true;
          root.live_factor_telemetry =
              get_sparse_basis_factor_telemetry(
                  root_relax.simplex->basis.cached_sparse_basis);
        }
        record_conformance_propagation(
            "root_initial_propagation", root_conf_sources,
            root_bound_delta.first, root_bound_delta.second, 0,
            /*resolved=*/true, /*resolve_ok=*/true, root_bound_before_prop,
            root.bound, root_conf_detail);
          if (bc_conf_diag_enabled()) {
	          fmt::print(stderr,
	                     "[B&C ROOT-PROP] row={} clique={} conflict={} impl={} "
	                     "proof_target={} obj={} "
	                     "follow_row={} follow_clique={} bound={:.2f}->{:.2f} "
	                     "resolve={:.1f}ms cutoff={} source={}\n",
	                     row_tightened, clique_tightened, conflict_tightened,
	                     implication_tightened, proof_target_tightened,
	                     objective_tightened,
	                     followup_row_tightened, followup_clique_tightened,
	                     root_bound_before_prop, root.bound,
                     std::chrono::duration<double, std::milli>(
                         t_prop_resolve1 - t_prop_resolve0).count(),
                     has_root_cutoff_for_propagation
                         ? fmt::format("{:.2f}", root_cutoff_for_propagation)
                         : std::string("none"),
                     root_cutoff_source_for_propagation);
        }
      } else {
        root.lb = root_lb_before_prop;
        root.ub = root_ub_before_prop;
        root_lp = root_lp_before_prop;
        root.basis_hint = root_basis_before_prop;
        root.x_relax = root_x_before_prop;
        root.bound = root_bound_before_prop;
        root_relax = root_relax_before_prop;
        if (root_relax.simplex &&
            root_relax.simplex->basis.cached_sparse_basis) {
          root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
        }
        root_propagation_local_cutoff_tightened = false;
        record_conformance_propagation(
            "root_initial_propagation", root_conf_sources,
            root_bound_delta.first, root_bound_delta.second, 0,
            /*resolved=*/true, /*resolve_ok=*/false, root_bound_before_prop,
            root_bound_before_prop, root_conf_detail + " reverted=1");
        if (opt.verbose) {
          fmt::print(stderr,
                     "[B&C ROOT-PROP] re-solve failed after domain "
                     "tightening; reverted root propagation\n");
        }
      }
      }
    } else if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C ROOT-PROP] no root domain tightening "
                 "(cutoff={} source={})\n",
                 has_root_cutoff_for_propagation
                     ? fmt::format("{:.2f}", root_cutoff_for_propagation)
                     : std::string("none"),
                 root_cutoff_source_for_propagation);
      record_conformance_propagation(
          "root_initial_propagation", root_conf_sources,
          root_bound_delta.first, root_bound_delta.second, 0,
          /*resolved=*/false, /*resolve_ok=*/true, root_bound_before_prop,
          root_bound_before_prop, root_conf_detail);
    } else {
      record_conformance_propagation(
          "root_initial_propagation", root_conf_sources,
          root_bound_delta.first, root_bound_delta.second, 0,
          /*resolved=*/false, /*resolve_ok=*/true, root_bound_before_prop,
          root_bound_before_prop, root_conf_detail);
    }
  }

		  // HiGHS keeps the root cutpool attached to the active propagation domain
		  // during separation.  Native root source rows used to become
		  // queue-visible only after root separation had already finished, which
		  // meant the root fixed point could not consume them before the tree
		  // opened.  Keep this pool live for the whole root separation path.
			  CutPool cut_pool(
			      strict_highs_root_fixed_point ? kHighsDefaultMipPoolSoftLimit
			                                    : opt.cut_pool_max_size,
			      strict_highs_root_fixed_point ? kHighsDefaultMipPoolAgeLimit
			                                    : opt.cut_pool_max_age,
			      n);

		  auto run_root_probed_implied_bound_resolve = [&]() -> bool {
    if (!opt.enable_dynamic_implied_bound_probing ||
        !opt.enable_graph_implied_bound_cuts ||
        opt.dynamic_implied_bound_probing_max_rows <= 0 ||
        root.x_relax.size() != n ||
        root_propagation_local_cutoff_tightened) {
      return false;
    }

    struct RootProbeCandidate {
      int col{-1};
      double score{0.0};
      double dist_to_half{0.0};
	    };
	    std::vector<RootProbeCandidate> candidates;
	    candidates.reserve(64);
        auto root_probe_binary_like = [&](int j) {
          if (j < 0 || j >= n) return false;
          const auto& var = root_lp.vars[static_cast<std::size_t>(j)];
          if (root.lb[j] > 1e-9 || root.ub[j] < 1.0 - 1e-9) return false;
          if (var.type == VarType::Binary ||
              (var.type == VarType::Integer && std::abs(root.lb[j]) <= 1e-9 &&
               std::abs(root.ub[j] - 1.0) <= 1e-9)) {
            return true;
          }
          return j < static_cast<int>(root_implied_integer_cols.size()) &&
                 root_implied_integer_cols[static_cast<std::size_t>(j)] != 0 &&
                 std::abs(root.lb[j]) <= 1e-9 &&
                 std::abs(root.ub[j] - 1.0) <= 1e-9;
        };
	    for (int j = 0; j < n; ++j) {
          if (!root_probe_binary_like(j)) continue;
          if (j < static_cast<int>(root_implied_bound_probe_cache.size()) &&
              root_implied_bound_probe_cache[static_cast<std::size_t>(j)] ==
                  0x3) {
            continue;
          }
	      const double xj = root.x_relax[j];
	      if (!std::isfinite(xj) || xj <= opt.int_tol ||
	          xj >= 1.0 - opt.int_tol) {
        continue;
      }
      const double fractionality = std::min(xj, 1.0 - xj);
      const double obj_mass = std::abs(base_lp.c[j]);
      candidates.push_back(
          RootProbeCandidate{j, fractionality + 1e-9 * obj_mass,
                             std::abs(xj - 0.5)});
    }
    if (candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end(),
              [](const RootProbeCandidate& a, const RootProbeCandidate& b) {
                if (a.score != b.score) return a.score > b.score;
                if (a.dist_to_half != b.dist_to_half) {
                  return a.dist_to_half < b.dist_to_half;
                }
                return a.col < b.col;
          });

    const int configured_probe_vars =
        std::max(0, opt.dynamic_implied_bound_probing_max_vars);
    const int highs_style_root_probe_vars =
        large_generic_presolved_root && candidates.size() <= 256
            ? static_cast<int>(candidates.size())
            : configured_probe_vars;
    const int max_vars =
        candidates.empty()
            ? 0
            : std::min<int>(candidates.size(),
                            std::max(configured_probe_vars,
                                     highs_style_root_probe_vars));
    const int row_budget = std::max(
        std::max(0, opt.dynamic_implied_bound_probing_max_rows),
        std::min(std::max(0, opt.graph_implied_bound_max_cuts), 64));
    if (row_budget <= 0) return false;

    BinaryImplicationGraph probed_graph(n);
    std::vector<Eigen::SparseVector<double>> direct_rows;
    std::vector<double> direct_rhs;
    direct_rows.reserve(static_cast<std::size_t>(row_budget));
    direct_rhs.reserve(static_cast<std::size_t>(row_budget));
    struct RootImpliedBoundRow {
      Eigen::SparseVector<double> row;
      double rhs{0.0};
      double violation{0.0};
      double score{0.0};
      int trigger{-1};
      bool trigger_one{false};
      int implied{-1};
      bool implied_is_lb{true};
      double implied_value{0.0};
    };
    std::unordered_set<std::string> seen_root_ib_rows;
    int probes = 0;
    int infeasible_probes = 0;
	    int propagated_implications = 0;
	    int objective_event_rows_admitted = 0;
	    int clique_implication_rows_admitted = 0;
	    int graph_rows_admitted = 0;
        int probe_varbound_sources_accepted = 0;
        int probe_global_tightenings = 0;
        Eigen::VectorXd probe_global_lb = root.lb;
        Eigen::VectorXd probe_global_ub = root.ub;

        auto tighten_probe_global_lb = [&](int col, double value) {
          if (col < 0 || col >= n || !std::isfinite(value)) return;
          if (is_integer_type(root_lp.vars[static_cast<std::size_t>(col)]) ||
              (col < static_cast<int>(root_implied_integer_cols.size()) &&
               root_implied_integer_cols[static_cast<std::size_t>(col)] != 0)) {
            value = std::ceil(value - std::max(1e-9, opt.int_tol));
          }
          if (value > probe_global_lb[col] + std::max(1e-9, opt.int_tol)) {
            probe_global_lb[col] = value;
            ++probe_global_tightenings;
            ++variable_bound_source_stats.probe_global_tightenings;
          }
        };
        auto tighten_probe_global_ub = [&](int col, double value) {
          if (col < 0 || col >= n || !std::isfinite(value)) return;
          if (is_integer_type(root_lp.vars[static_cast<std::size_t>(col)]) ||
              (col < static_cast<int>(root_implied_integer_cols.size()) &&
               root_implied_integer_cols[static_cast<std::size_t>(col)] != 0)) {
            value = std::floor(value + std::max(1e-9, opt.int_tol));
          }
          if (value < probe_global_ub[col] - std::max(1e-9, opt.int_tol)) {
            probe_global_ub[col] = value;
            ++probe_global_tightenings;
            ++variable_bound_source_stats.probe_global_tightenings;
          }
        };

        auto add_probe_endpoint_varbound =
            [&](int trigger, bool value_one, int implied, bool implied_is_lb,
                double implied_value) {
          if (trigger < 0 || trigger >= n || implied < 0 || implied >= n ||
              trigger == implied || !std::isfinite(implied_value)) {
            return;
          }
          if (implied_is_lb) {
            const double base_lb = root.lb[implied];
            if (!std::isfinite(base_lb) || implied_value <= base_lb + 1e-9) {
              return;
            }
            const double coef = value_one ? implied_value - base_lb
                                          : base_lb - implied_value;
            const double constant = value_one ? base_lb : implied_value;
            if (add_root_probe_varbound_source(
                    /*is_vub=*/false, implied, trigger, coef, constant)) {
              ++probe_varbound_sources_accepted;
            }
          } else {
            const double base_ub = root.ub[implied];
            if (!std::isfinite(base_ub) || implied_value >= base_ub - 1e-9) {
              return;
            }
            const double coef = value_one ? implied_value - base_ub
                                          : base_ub - implied_value;
            const double constant = value_one ? base_ub : implied_value;
            if (add_root_probe_varbound_source(
                    /*is_vub=*/true, implied, trigger, coef, constant)) {
              ++probe_varbound_sources_accepted;
            }
          }
        };

    auto replay_root_propagation =
        [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub) -> bool {
      for (int round = 0; round < 4; ++round) {
        const int row_changes = strict_highs_root_fixed_point
            ? propagate_root_highs_presolved_domain(
                  lb, ub, opt.bound_propagation_rounds)
            : node_bound_propagation(root_lp, lb, ub,
                                     opt.bound_propagation_rounds);
        int clique_changes = 0;
        if (!clique_table.empty()) {
          clique_changes = clique_table.propagate(lb, ub);
        }
        int implication_changes = 0;
        if (!implication_graph.empty()) {
	          implication_changes = implication_graph.propagate(
	              base_lp.vars, lb, ub, nullptr, std::max(1e-9, opt.int_tol));
        }
        if (!bounds_consistent(lb, ub)) return false;
        if (row_changes + clique_changes + implication_changes == 0) break;
      }
      return bounds_consistent(lb, ub);
    };

    auto add_direct_bound_row = [&](int trigger, bool forbid_one) {
      if (static_cast<int>(direct_rows.size()) >= row_budget) return;
      const std::string key =
          fmt::format("direct:{}:{}", trigger, forbid_one ? 1 : 0);
      if (!seen_root_ib_rows.insert(key).second) return;
      Eigen::SparseVector<double> row(n);
      row.reserve(1);
      double rhs = 0.0;
      if (forbid_one) {
        row.coeffRef(trigger) = 1.0;
        rhs = 0.0;
      } else {
        row.coeffRef(trigger) = -1.0;
        rhs = -1.0;
      }
      const double viol = sparse_dot(row, root.x_relax) - rhs;
      ++out.bc_stats.dynamic_implied_bound_rows_generated;
      if (viol <= opt.dynamic_implied_bound_min_violation) {
        ++out.bc_stats.dynamic_implied_bound_rows_rejected;
        ++out.bc_stats.dynamic_implied_bound_rows_skip_no_violation;
        return;
      }
      ++out.bc_stats.dynamic_implied_bound_rows_violated;
      ++out.bc_stats.dynamic_implied_bound_rows_added;
      ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
      direct_rows.push_back(std::move(row));
      direct_rhs.push_back(rhs);
    };

    auto collect_implied_bound_row =
        [&](int trigger, bool trigger_one, int implied, bool implied_is_lb,
            double implied_value, double source_score,
            std::vector<RootImpliedBoundRow>& out_rows) {
      if (trigger < 0 || trigger >= n || implied < 0 || implied >= n ||
          trigger == implied || !std::isfinite(implied_value) ||
          !std::isfinite(root.x_relax[trigger]) ||
          !std::isfinite(root.x_relax[implied])) {
        return;
      }
      if (base_lp.vars[static_cast<std::size_t>(trigger)].type !=
          VarType::Binary) {
        return;
      }
      const double lb = root.lb[implied];
      const double ub = root.ub[implied];
      if (implied_is_lb && !std::isfinite(lb)) return;
      if (!implied_is_lb && !std::isfinite(ub)) return;

      double rhs_val = 0.0;
      double coeff_trigger = 0.0;
      const double coeff_implied = implied_is_lb ? -1.0 : 1.0;
      if (trigger_one) {
        if (implied_is_lb) {
          if (!(implied_value > lb + 1e-9)) return;
          coeff_trigger = implied_value - lb;
          rhs_val = -lb;
        } else {
          if (!(implied_value < ub - 1e-9)) return;
          coeff_trigger = ub - implied_value;
          rhs_val = ub;
        }
      } else {
        if (implied_is_lb) {
          if (!(implied_value > lb + 1e-9)) return;
          coeff_trigger = lb - implied_value;
          rhs_val = -implied_value;
        } else {
          if (!(implied_value < ub - 1e-9)) return;
          coeff_trigger = implied_value - ub;
          rhs_val = implied_value;
        }
      }
      if (!std::isfinite(coeff_trigger) || !std::isfinite(rhs_val)) return;
      const double lhs =
          coeff_implied * root.x_relax[implied] +
          coeff_trigger * root.x_relax[trigger];
      const double scale = std::max({1.0, std::abs(rhs_val),
                                     std::abs(coeff_implied),
                                     std::abs(coeff_trigger)});
      const double violation = lhs - rhs_val;
      ++out.bc_stats.dynamic_implied_bound_rows_generated;
      if (violation <= opt.dynamic_implied_bound_min_violation * scale) {
        ++out.bc_stats.dynamic_implied_bound_rows_rejected;
        ++out.bc_stats.dynamic_implied_bound_rows_skip_no_violation;
        return;
      }
      const std::string key =
          fmt::format("{}:{}:{}:{}:{:.12g}", trigger, trigger_one ? 1 : 0,
                      implied, implied_is_lb ? 1 : 0, implied_value);
      if (!seen_root_ib_rows.insert(key).second) return;

      Eigen::SparseVector<double> row(n);
      row.reserve(2);
      row.coeffRef(implied) += coeff_implied;
      row.coeffRef(trigger) += coeff_trigger;
      const double obj_mass = std::abs(base_lp.c[implied]);
      const double bound_move = implied_is_lb
          ? std::max(0.0, implied_value - lb)
          : std::max(0.0, ub - implied_value);
      out_rows.push_back(RootImpliedBoundRow{
          std::move(row), rhs_val, violation,
          (1.0 + obj_mass) * violation + source_score +
              1e-6 * obj_mass * bound_move,
          trigger, trigger_one, implied, implied_is_lb, implied_value});
    };

    if (!objective_propagation.implied_events.empty()) {
      std::vector<RootImpliedBoundRow> objective_rows;
      objective_rows.reserve(std::min<std::size_t>(
          objective_propagation.implied_events.size(), 4 * row_budget));
      for (const auto& event : objective_propagation.implied_events) {
        if (event.literals.size() != 1 || event.target_col < 0 ||
            event.target_col >= n) {
          continue;
        }
        const auto& lit = event.literals.front();
        if (event.target_is_lower_bound) {
          if (event.target_bound <= root.lb[event.target_col] + 1e-9) {
            continue;
          }
          const double source_score =
              std::max(0.0, event.target_cost) *
              std::max(0.0, event.target_bound - root.lb[event.target_col]);
          collect_implied_bound_row(lit.col, lit.value_one, event.target_col,
                                    /*implied_is_lb=*/true,
                                    event.target_bound, source_score,
                                    objective_rows);
        } else {
          if (event.target_bound >= root.ub[event.target_col] - 1e-9) {
            continue;
          }
          const double source_score =
              std::max(0.0, -event.target_cost) *
              std::max(0.0, root.ub[event.target_col] - event.target_bound);
          collect_implied_bound_row(lit.col, lit.value_one, event.target_col,
                                    /*implied_is_lb=*/false,
                                    event.target_bound, source_score,
                                    objective_rows);
        }
      }
      std::sort(objective_rows.begin(), objective_rows.end(),
                [](const RootImpliedBoundRow& a,
                   const RootImpliedBoundRow& b) {
                  if (a.score != b.score) return a.score > b.score;
                  if (a.violation != b.violation) {
                    return a.violation > b.violation;
                  }
                  return a.trigger < b.trigger;
                });
      const int take =
          std::min<int>(row_budget, objective_rows.size());
      for (int i = 0; i < take; ++i) {
        auto& cand = objective_rows[static_cast<std::size_t>(i)];
        implication_graph.add_implication(cand.trigger, cand.trigger_one,
                                          cand.implied, cand.implied_is_lb,
                                          cand.implied_value);
        ++out.bc_stats.dynamic_implied_bound_rows_violated;
        ++out.bc_stats.dynamic_implied_bound_rows_added;
        ++out.bc_stats.dynamic_implied_bound_rows_active_implication;
        ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
        direct_rows.push_back(std::move(cand.row));
        direct_rhs.push_back(cand.rhs);
        ++objective_event_rows_admitted;
      }
    }

    if (!clique_table.empty() &&
        static_cast<int>(direct_rows.size()) < row_budget) {
      std::vector<RootImpliedBoundRow> clique_rows;
      clique_rows.reserve(static_cast<std::size_t>(
          std::max(0, row_budget - static_cast<int>(direct_rows.size()))));
      for (int j = 0; j < n; ++j) {
        if (base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
          continue;
        }
        const double xj = root.x_relax[j];
        if (!(xj > 1e-6 && xj < 1.0 - 1e-6)) continue;
        auto range = clique_table.neighbours(j);
        for (const int* p = range.first; p != range.second; ++p) {
          const int k = *p;
          if (k <= j || k >= n ||
              base_lp.vars[static_cast<std::size_t>(k)].type != VarType::Binary) {
            continue;
          }
          const double xk = root.x_relax[k];
          const double violation = xj + xk - 1.0;
          if (violation <= opt.dynamic_implied_bound_min_violation) continue;
          const double source_score =
              violation * (1.0 + std::abs(base_lp.c[j]) + std::abs(base_lp.c[k]));
          collect_implied_bound_row(j, /*trigger_one=*/true, k,
                                    /*implied_is_lb=*/false, 0.0,
                                    source_score, clique_rows);
        }
      }
      std::sort(clique_rows.begin(), clique_rows.end(),
                [](const RootImpliedBoundRow& a,
                   const RootImpliedBoundRow& b) {
                  if (a.score != b.score) return a.score > b.score;
                  if (a.violation != b.violation) {
                    return a.violation > b.violation;
                  }
                  if (a.trigger != b.trigger) return a.trigger < b.trigger;
                  return a.implied < b.implied;
                });
      const int remaining =
          std::max(0, row_budget - static_cast<int>(direct_rows.size()));
      const int take = std::min<int>(remaining, clique_rows.size());
      for (int i = 0; i < take; ++i) {
        auto& cand = clique_rows[static_cast<std::size_t>(i)];
        implication_graph.add_implication(cand.trigger, cand.trigger_one,
                                          cand.implied, cand.implied_is_lb,
                                          cand.implied_value);
        ++out.bc_stats.dynamic_implied_bound_rows_violated;
        ++out.bc_stats.dynamic_implied_bound_rows_added;
        ++out.bc_stats.dynamic_implied_bound_rows_active_implication;
        ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
        direct_rows.push_back(std::move(cand.row));
        direct_rhs.push_back(cand.rhs);
        ++clique_implication_rows_admitted;
      }
    }

	    for (int ci = 0; ci < max_vars; ++ci) {
	      const int trigger = candidates[static_cast<std::size_t>(ci)].col;
          struct ProbeWorld {
            bool value_one{false};
            bool feasible{false};
            Eigen::VectorXd lb;
            Eigen::VectorXd ub;
          };
          std::array<ProbeWorld, 2> worlds{
              ProbeWorld{false, false, Eigen::VectorXd(), Eigen::VectorXd()},
              ProbeWorld{true, false, Eigen::VectorXd(), Eigen::VectorXd()}};

          for (ProbeWorld& world : worlds) {
	        world.lb = root.lb;
	        world.ub = root.ub;
	        if (world.value_one) {
	          world.lb[trigger] = std::max(world.lb[trigger], 1.0);
	        } else {
	          world.ub[trigger] = std::min(world.ub[trigger], 0.0);
	        }
            if (trigger >= 0 &&
                trigger < static_cast<int>(
                              root_implied_bound_probe_cache.size())) {
              root_implied_bound_probe_cache[static_cast<std::size_t>(
                  trigger)] |= world.value_one ? 0x2 : 0x1;
            }
	        ++probes;
	        if (!bounds_consistent(world.lb, world.ub) ||
	            !replay_root_propagation(world.lb, world.ub)) {
	          ++infeasible_probes;
	          add_direct_bound_row(trigger, world.value_one);
              if (world.value_one) {
                tighten_probe_global_ub(trigger, 0.0);
              } else {
                tighten_probe_global_lb(trigger, 1.0);
              }
	          continue;
	        }
            world.feasible = true;

	        for (int k = 0; k < n; ++k) {
	          if (k == trigger) continue;
	          if (world.lb[k] > root.lb[k] + 1e-9) {
                add_probe_endpoint_varbound(trigger, world.value_one, k,
                                            /*implied_is_lb=*/true,
                                            world.lb[k]);
	            if (probed_graph.add_implication(trigger, world.value_one, k, true,
	                                             world.lb[k])) {
	              implication_graph.add_implication(trigger, world.value_one, k, true,
	                                                world.lb[k]);
	              ++propagated_implications;
	            }
	          }
	          if (world.ub[k] < root.ub[k] - 1e-9) {
                add_probe_endpoint_varbound(trigger, world.value_one, k,
                                            /*implied_is_lb=*/false,
                                            world.ub[k]);
	            if (probed_graph.add_implication(trigger, world.value_one, k, false,
	                                             world.ub[k])) {
	              implication_graph.add_implication(trigger, world.value_one, k, false,
	                                                world.ub[k]);
	              ++propagated_implications;
	            }
	          }
	        }
	      }

          if (worlds[0].feasible && worlds[1].feasible) {
            for (int k = 0; k < n; ++k) {
              if (k == trigger) continue;
              const double common_lb =
                  std::min(worlds[0].lb[k], worlds[1].lb[k]);
              const double common_ub =
                  std::max(worlds[0].ub[k], worlds[1].ub[k]);
              if (common_lb > root.lb[k] + 1e-9) {
                tighten_probe_global_lb(k, common_lb);
              }
              if (common_ub < root.ub[k] - 1e-9) {
                tighten_probe_global_ub(k, common_ub);
              }
            }
          }
	    }

        if (!root_probe_varbound_sources.empty()) {
          const std::uint64_t exported =
              strict_highs_root_fixed_point
                  ? 0
                  : variable_bound_table.export_to_implication_graph(
                        implication_graph);
          variable_bound_source_stats.probe_exported_implications += exported;
          variable_bound_source_stats.exported_implications += exported;
          sync_variable_bound_stats();
          if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
            fmt::print(stderr,
                       "[B&C-VB-SRC] phase=after_root_probe_sources "
                       "varbounds={} probeVUB={} probeVLB={} accepted={} "
                       "probeTight={} exp={} totalImpl={}\n",
                       variable_bound_table.size(),
                       variable_bound_source_stats.probe_vub_candidates,
                       variable_bound_source_stats.probe_vlb_candidates,
                       probe_varbound_sources_accepted, probe_global_tightenings,
                       exported, implication_graph.size());
          }
        }

    std::vector<Eigen::SparseVector<double>> ib_rows;
    std::vector<double> ib_rhs;
    const int remaining_rows =
        std::max(0, row_budget - static_cast<int>(direct_rows.size()));
    if (!implication_graph.empty() && remaining_rows > 0) {
      ib_rows.reserve(static_cast<std::size_t>(remaining_rows));
      ib_rhs.reserve(static_cast<std::size_t>(remaining_rows));
      graph_rows_admitted = append_graph_implied_bound_cuts(
          root_lp, root.x_relax, root.lb, root.ub, implication_graph,
          remaining_rows, ib_rows, ib_rhs);
    }

    std::vector<Eigen::SparseVector<double>> rows;
    std::vector<double> rhs;
    rows.reserve(direct_rows.size() + ib_rows.size());
    rhs.reserve(direct_rhs.size() + ib_rhs.size());
    for (std::size_t i = 0; i < direct_rows.size(); ++i) {
      rows.push_back(std::move(direct_rows[i]));
      rhs.push_back(direct_rhs[i]);
    }
    for (std::size_t i = 0; i < ib_rows.size(); ++i) {
      ++out.bc_stats.dynamic_implied_bound_rows_generated;
      ++out.bc_stats.dynamic_implied_bound_rows_violated;
      ++out.bc_stats.dynamic_implied_bound_rows_added;
      ++out.bc_stats.dynamic_implied_bound_rows_active_implication;
      ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
      rows.push_back(std::move(ib_rows[i]));
      rhs.push_back(ib_rhs[i]);
    }

    if (!rows.empty()) {
      std::vector<Eigen::SparseVector<double>> unique_rows;
      std::vector<double> unique_rhs;
      unique_rows.reserve(rows.size());
      unique_rhs.reserve(rhs.size());
      for (std::size_t i = 0; i < rows.size(); ++i) {
        if (!root_implied_bound_cuts.insert(rows[i], rhs[i])) {
          continue;
        }
        unique_rows.push_back(std::move(rows[i]));
        unique_rhs.push_back(rhs[i]);
      }
      rows = std::move(unique_rows);
      rhs = std::move(unique_rhs);
    }

	    if (rows.empty() && probe_global_tightenings == 0) {
	      if (opt.verbose) {
	        fmt::print(stderr,
	                   "[B&C ROOT-IB-PROP] probes={} infeas={} impl={} "
	                   "obj_rows={} clique_rows={} graph_rows={} "
                       "probe_vb={} cuts=0\n",
	                   probes, infeasible_probes, propagated_implications,
	                   objective_event_rows_admitted,
	                   clique_implication_rows_admitted, graph_rows_admitted,
                       probe_varbound_sources_accepted);
	      }
	      return false;
	    }

    if (!rows.empty() && probe_global_tightenings == 0 &&
        opt.auto_highs_root_pipeline) {
      int deferred_rows = 0;
      for (std::size_t i = 0; i < rows.size(); ++i) {
        const double norm =
            std::sqrt(std::max(0.0, rows[i].squaredNorm()));
        if (!(norm > 1e-12) || !std::isfinite(norm) ||
            !std::isfinite(rhs[i])) {
          continue;
        }
        const std::size_t hash = sparse_cut_hash(rows[i]);
        root_deferred_implied_bound_cutpool_rows.push_back(
            PoolCut{std::move(rows[i]), rhs[i], 0, 0.0, norm, hash});
        ++deferred_rows;
      }
      if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
        fmt::print(stderr,
                   "[B&C ROOT-IB-PROP] deferred cutpool rows={} probes={} "
                   "infeas={} impl={} obj_rows={} clique_rows={} "
                   "graph_rows={} probe_vb={}\n",
                   deferred_rows, probes, infeasible_probes,
                   propagated_implications, objective_event_rows_admitted,
                   clique_implication_rows_admitted, graph_rows_admitted,
                   probe_varbound_sources_accepted);
      }
      return false;
    }

    const int prev_rows = static_cast<int>(root_lp.A.rows());
    const double old_bound = root.bound;
    const Eigen::VectorXd old_x = root.x_relax;
    const auto old_basis = root.basis_hint;
    const LPRelaxationResult old_relax = root_relax;
    LPModel trial_lp = root_lp;
    add_sparse_rows_to_lp(trial_lp, rows, rhs);
	    Eigen::VectorXd trial_lb = probe_global_lb;
	    Eigen::VectorXd trial_ub = probe_global_ub;
	    int trial_domain_tightened = probe_global_tightenings;
	    if (opt.root_cut_reject_nonmoving_rows) {
	      const Eigen::SparseMatrix<double, Eigen::RowMajor> trial_A_row =
	          trial_lp.A;
	      const Eigen::SparseMatrix<double, Eigen::RowMajor> trial_Aeq_row =
	          trial_lp.Aeq;
	      const int propagated_domain_tightened = node_bound_propagation(
	          trial_lp, trial_A_row, trial_Aeq_row, trial_lb, trial_ub,
	          opt.bound_propagation_rounds);
	      if (!bounds_consistent(trial_lb, trial_ub)) {
	        trial_domain_tightened = -1;
	      } else if (propagated_domain_tightened > 0) {
            trial_domain_tightened += propagated_domain_tightened;
          }
	    }
    if (trial_domain_tightened > 0) {
      apply_node_bounds(trial_lp.vars, trial_lb, trial_ub);
    } else {
      apply_node_bounds(trial_lp.vars, root.lb, root.ub);
    }

    const auto t_ib_resolve0 = std::chrono::steady_clock::now();
    LPRelaxationResult ib_relax = solve_lp_relaxation(
        trial_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
    const auto t_ib_resolve1 = std::chrono::steady_clock::now();
    out.bc_stats.lp_solves += 1;

    if (ib_relax.primal.stats.success && ib_relax.primal.x.size() == n &&
        std::isfinite(ib_relax.dual_bound) &&
        ib_relax.dual_bound >= old_bound -
            std::max(1e-6, 1e-9 * std::max(1.0, std::abs(old_bound)))) {
      const double lift = ib_relax.dual_bound - old_bound;
      const double min_lift = std::max(
          opt.root_cut_min_bound_lift,
          1e-10 * std::max(1.0, std::abs(old_bound)));
      const bool termination_effective =
          !opt.root_cut_reject_nonmoving_rows || trial_domain_tightened > 0 ||
          lift > min_lift;
      if (!termination_effective) {
        auto sub_counter = [](std::uint64_t& counter, std::uint64_t value) {
          counter = counter > value ? counter - value : 0;
        };
        const auto rejected_rows =
            static_cast<std::uint64_t>(rows.size());
        out.bc_stats.dynamic_implied_bound_rows_rejected += rejected_rows;
        sub_counter(out.bc_stats.dynamic_implied_bound_rows_added,
                    rejected_rows);
        sub_counter(out.bc_stats.dynamic_implied_bound_rows_bound_move,
                    rejected_rows);
        ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
        out.bc_stats.root_cut_rows_rejected_nonmoving += rejected_rows;
        if (opt.verbose || rejected_rows > 0) {
	          fmt::print(stderr,
	                     "[B&C ROOT-IB-PROP] rejected nonmoving candidate rows={} "
	                     "probes={} impl={} bound={:.2f}->{:.2f} lift={:.3e} "
	                     "domain_tight={} probe_vb={}\n",
	                     static_cast<int>(rows.size()), probes,
	                     propagated_implications, old_bound, ib_relax.dual_bound,
	                     lift, trial_domain_tightened,
                         probe_varbound_sources_accepted);
	        }
        return false;
      }
	      if (trial_domain_tightened > 0) {
	        root.lb = std::move(trial_lb);
	        root.ub = std::move(trial_ub);
	        out.bc_stats.root_cut_domain_tightenings +=
	            static_cast<std::uint64_t>(trial_domain_tightened);
	      }
	      root_lp = std::move(trial_lp);
	      capture_variable_bound_cutpool_rows(root_lp, prev_rows,
	                                          static_cast<int>(root_lp.A.rows()),
	                                          "after_root_ib_prop");
	      root_relax = std::move(ib_relax);
	      root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	      root.x_seed = root.x_relax;
	      refresh_root_lp_source_solution();
		      root.basis_hint = root_relax.basis_hint;
	      if (root.basis_hint) root.basis_hint->compact_indices_storage();
	      root.bound = root_relax.dual_bound;
	      refresh_root_redcost_lurking_bounds("root_implied_bound_resolve");
	      if (root_relax.simplex &&
	          root_relax.simplex->basis.cached_sparse_basis) {
        root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
      } else {
        root_sbasis.reset();
      }
      const int admitted = static_cast<int>(root_lp.A.rows()) - prev_rows;
      out.bc_stats.cuts_added += admitted;
      out.bc_stats.root_cuts_added += admitted;
      out.bc_stats.root_impliedbound_cuts += admitted;
	      fmt::print(stderr,
	                 "[B&C ROOT-IB-PROP] probes={} infeas={} impl={} cuts={} "
	                 "obj_rows={} clique_rows={} graph_rows={} probe_vb={} "
                     "probe_tight={} bound={:.2f}->{:.2f} resolve={:.1f}ms\n",
	                 probes, infeasible_probes, propagated_implications, admitted,
	                 objective_event_rows_admitted,
	                 clique_implication_rows_admitted, graph_rows_admitted,
                     probe_varbound_sources_accepted, trial_domain_tightened,
	                 old_bound, root.bound,
	                 std::chrono::duration<double, std::milli>(
	                     t_ib_resolve1 - t_ib_resolve0).count());
      return true;
    } else {
      root.x_relax = old_x;
      root.basis_hint = old_basis;
      root.bound = old_bound;
      root_relax = old_relax;
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C ROOT-IB-PROP] re-solve failed/degraded; reverted "
                   "probes={} infeas={} impl={} obj_rows={} clique_rows={} "
                   "graph_rows={} candidate_cuts={}\n",
                   probes, infeasible_probes, propagated_implications,
                   objective_event_rows_admitted,
                   clique_implication_rows_admitted, graph_rows_admitted,
                   static_cast<int>(rows.size()));
      }
    }
    return false;
	  };

	  [[maybe_unused]] auto run_root_tableau_source_resolve = [&]() -> bool {
		    const Eigen::VectorXd& source_x = root_source_solution();
		    if (!root_relax.simplex || source_x.size() != n ||
		        root_implied_integer_count <= 0) {
      return false;
    }
    std::shared_ptr<BasisOps> tableau_sbasis = root_sbasis;
    if (root_relax.simplex->basis.cached_sparse_basis) {
      tableau_sbasis = root_relax.simplex->basis.cached_sparse_basis;
    }
    SimplexResult tableau_simplex = *root_relax.simplex;
    const int n_form =
        static_cast<int>(tableau_simplex.form.original_types.size());
    int implied_marked = 0;
    for (int j = 0; j < std::min(n, n_form); ++j) {
      if (root_implied_integer_cols[static_cast<std::size_t>(j)] &&
          tableau_simplex.form.original_types[static_cast<std::size_t>(j)] ==
              VarType::Continuous) {
        tableau_simplex.form.original_types[static_cast<std::size_t>(j)] =
            VarType::Integer;
        ++implied_marked;
      }
    }
    if (implied_marked == 0) return false;

    LPModel trial_lp = root_lp;
    const int prev_rows = static_cast<int>(trial_lp.A.rows());
    const bool large_transformed_source_root =
        presolved_m > opt.cut_budget_large_row_threshold;
    const int max_tableau_cuts = large_transformed_source_root
        ? std::max(8, std::min(512, std::max(opt.cuts_per_round,
                                             20 * opt.cuts_per_round)))
        : std::max(8, std::min(64, std::max(opt.cuts_per_round,
                                            2 * opt.cuts_per_round)));
    const int added = add_transformed_tableau_cuts(
        trial_lp, source_x, tableau_simplex, opt, max_tableau_cuts,
        tableau_sbasis, &root_implied_integer_cols, &implication_graph,
        &variable_bound_table, nullptr, 1e-8, nullptr, std::nullopt,
        &separator_storage_stats);
    if (added <= 0) {
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C ROOT-XTAB] no violated transformed implied-integer tableau "
                   "cuts (marked={})\n",
                   implied_marked);
      }
      return false;
    }

    apply_node_bounds(trial_lp.vars, root.lb, root.ub);
    const double old_bound = root.bound;
    LPRelaxationResult tableau_relax = solve_lp_relaxation(
        trial_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
    ++out.bc_stats.lp_solves;
    if (!tableau_relax.primal.stats.success ||
        tableau_relax.primal.x.size() != n ||
        !std::isfinite(tableau_relax.dual_bound) ||
        tableau_relax.dual_bound < old_bound -
            std::max(1e-6, 1e-9 * std::max(1.0, std::abs(old_bound)))) {
      if (opt.verbose || added > 0) {
        fmt::print(stderr,
                   "[B&C ROOT-XTAB] rejected cuts={} rows={} "
                   "old_bound={:.2f} status={}\n",
                   added, static_cast<int>(trial_lp.A.rows()) - prev_rows,
                   old_bound, tableau_relax.primal.stats.status);
      }
      return false;
    }
    const double lift = tableau_relax.dual_bound - old_bound;
    const double min_lift =
        std::max(opt.root_cut_min_bound_lift,
                 1e-10 * std::max(1.0, std::abs(old_bound)));
	    if (opt.root_cut_reject_nonmoving_rows && lift <= min_lift) {
	      audit_variable_bound_cutpool_row_range(
	          trial_lp, prev_rows, static_cast<int>(trial_lp.A.rows()),
	          "rejected_root_xtab_source");
	      if (opt.verbose || added > 0) {
	        fmt::print(stderr,
		                   "[B&C ROOT-XTAB] rejected nonmoving cuts={} "
	                   "bound={:.2f}->{:.2f} lift={:.3e}\n",
	                   added, old_bound, tableau_relax.dual_bound, lift);
	      }
		      return false;
	    }

	    root_lp = std::move(trial_lp);
	    capture_variable_bound_cutpool_rows(root_lp, prev_rows,
	                                        static_cast<int>(root_lp.A.rows()),
	                                        "after_root_xtab");
    root_relax = std::move(tableau_relax);
    root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
    root.x_seed = root.x_relax;
	    root.basis_hint = root_relax.basis_hint;
	    if (root.basis_hint) root.basis_hint->compact_indices_storage();
	    root.bound = root_relax.dual_bound;
	    refresh_root_redcost_lurking_bounds("root_tableau_resolve");
	    if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
      root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
	    } else {
	      root_sbasis.reset();
	    }
	    refresh_root_lp_source_solution();
	    out.bc_stats.cuts_added += added;
    out.bc_stats.root_cuts_added += added;
    out.bc_stats.root_gomory_cuts += added;
    if (lift > opt.root_cut_min_bound_lift) {
      out.bc_stats.root_cut_bound_lift += std::max(0.0, lift);
    }
    fmt::print(stderr,
	               "[B&C ROOT-XTAB] implied_int={} cuts={} rows={} "
	               "bound={:.2f}->{:.2f} lift={:.3e}\n",
	               implied_marked, added,
	               static_cast<int>(root_lp.A.rows()) - prev_rows,
	               old_bound, root.bound, lift);
    return true;
	  };

			  auto count_fractional_source_integers = [&]() -> int {
			    return static_cast<int>(
              compute_root_highs_fractional_frontier().cols.size());
		  };

	  struct RootTransformedPoolStats {
	    int available{0};
	    int violated{0};
	    int active{0};
        int efficacious{0};
	    int selected{0};
	    int parallel{0};
        int aged_deleted{0};
        int age_limit{0};
        int num_lp_cuts{0};
        int selected_nnz{0};
        std::uint64_t selected_index_hash{0};
        std::uint64_t selected_content_hash{0};
	    double best_score{0.0};
        double min_score_factor{0.0};
	    double max_violation{0.0};
	  };

	  struct RootHighsXpoolAppendReplay {
	    int selected{0};
	    int pre_rows{0};
	    int post_rows{0};
	    int pre_nnz{0};
	    int post_nnz{0};
	    int selected_nnz{0};
	    int appended_nnz{0};
	    std::uint64_t selected_payload_hash{0};
	    std::uint64_t appended_payload_hash{0};
	    int success{0};
	    double append_ms{0.0};
	  };

	  struct RootHighsXpoolResolveReplay {
	    std::string status;
	    double obj{0.0};
	    int rows{0};
	    int cols{0};
	    int nnz{0};
	    int iter_delta{0};
	    int iter_total{0};
	    int frac{0};
	    std::uint64_t frontier_hash{0};
	    std::uint64_t status_hash{0};
	    std::uint64_t basis_hash{0};
	    int basic_cols{0};
	    int lower_cols{0};
	    int upper_cols{0};
	    int zero_cols{0};
	    int other_cols{0};
	    std::string sample;
	    double resolve_ms{0.0};
	  };

	  struct RootHighsXpoolResolveReplayRound {
	    RootHighsXpoolAppendReplay append;
	    RootHighsXpoolResolveReplay resolve;
	    std::vector<char> col_status;
	    std::vector<double> col_value;
	    std::vector<char> row_status;
	    std::vector<double> row_value;
	  };

	  struct RootHighsLiveXpoolRound {
	    std::string select_event;
	    RootXpoolPayloadSignature selected;
	    RootHighsXpoolAppendReplay append;
	    RootHighsXpoolResolveReplay resolve;
	    std::vector<char> col_status;
	    std::vector<double> col_value;
	    std::vector<char> row_status;
	    std::vector<double> row_value;
	    std::shared_ptr<Highs> highs_owner;
	    Highs* lpsolver{nullptr};
	    int append_iter_before{0};
	    int resolve_events{0};
	    bool have_select{false};
	    bool have_append{false};
	    bool have_resolve{false};
	    bool append_verified{false};
	    bool resolve_verified{false};
	    std::string verify_error;
	  };

	  struct RootHighsLiveXpoolOracle {
	    std::vector<RootHighsLiveXpoolRound> rounds;
	    bool ok{true};
	    std::string error;
	  };

	  auto root_highs_live_resolve_status_name =
	      [](HighsLpRelaxation::Status status) -> std::string {
	    switch (status) {
	      case HighsLpRelaxation::Status::kNotSet:
	        return "not_set";
	      case HighsLpRelaxation::Status::kOptimal:
	        return "optimal";
	      case HighsLpRelaxation::Status::kInfeasible:
	        return "infeasible";
	      case HighsLpRelaxation::Status::kUnscaledDualFeasible:
	        return "unscaled_dual_feasible";
	      case HighsLpRelaxation::Status::kUnscaledPrimalFeasible:
	        return "unscaled_primal_feasible";
	      case HighsLpRelaxation::Status::kUnscaledInfeasible:
	        return "unscaled_infeasible";
	      case HighsLpRelaxation::Status::kUnbounded:
	        return "unbounded";
	      case HighsLpRelaxation::Status::kError:
	        return "error";
	    }
	    return "unknown";
	  };

	  auto root_highs_live_state_from_lp =
	      [&](const HighsLpRelaxation& lp) -> RootHighsXpoolResolveReplay {
	    RootHighsXpoolResolveReplay resolve;
	    const Highs& highs = lp.getLpSolver();
	    const HighsLp& h_lp = highs.getLp();
	    const HighsSolution& sol = highs.getSolution();
	    const HighsBasis& basis = highs.getBasis();
	    resolve.status = root_highs_live_resolve_status_name(lp.getStatus());
	    resolve.obj = lp.getObjective();
	    resolve.rows = static_cast<int>(h_lp.num_row_);
	    resolve.cols = static_cast<int>(h_lp.num_col_);
	    resolve.nnz = static_cast<int>(h_lp.a_matrix_.numNz());
	    resolve.iter_total =
	        static_cast<int>(std::min<int64_t>(
	            lp.getNumLpIterations(), std::numeric_limits<int>::max()));
	    const RootXpoolStateSignature sig = root_xpool_state_signature_from_highs(
	        root_lp, sol, basis, n, root_implied_integer_cols,
	        highs_mip_feastol);
	    resolve.frac = sig.fractional;
	    resolve.frontier_hash = sig.frontier_hash;
	    resolve.status_hash = sig.status_hash;
	    resolve.basis_hash = sig.basis_hash;
	    resolve.basic_cols = sig.basic_cols;
	    resolve.lower_cols = sig.lower_cols;
	    resolve.upper_cols = sig.upper_cols;
	    resolve.zero_cols = sig.zero_cols;
	    resolve.other_cols = sig.other_cols;
	    return resolve;
	  };

	  auto root_highs_collect_live_basis =
	      [&](const HighsLpRelaxation& lp, RootHighsLiveXpoolRound& round) {
	    const Highs& highs = lp.getLpSolver();
	    const HighsLp& h_lp = highs.getLp();
	    const HighsSolution& sol = highs.getSolution();
	    const HighsBasis& basis = highs.getBasis();
	    const int cols = static_cast<int>(h_lp.num_col_);
	    const int rows = static_cast<int>(h_lp.num_row_);
	    round.col_status.assign(static_cast<std::size_t>(std::max(0, cols)),
	                            '?');
	    round.col_value.assign(
	        static_cast<std::size_t>(std::max(0, cols)),
	        std::numeric_limits<double>::quiet_NaN());
	    for (int j = 0; j < cols; ++j) {
	      const HighsBasisStatus st =
	          basis.valid && j < static_cast<int>(basis.col_status.size())
	              ? basis.col_status[static_cast<std::size_t>(j)]
	              : HighsBasisStatus::kNonbasic;
	      round.col_status[static_cast<std::size_t>(j)] =
	          root_highs_status_char(st);
	      if (j < static_cast<int>(sol.col_value.size())) {
	        round.col_value[static_cast<std::size_t>(j)] =
	            sol.col_value[static_cast<std::size_t>(j)];
	      }
	    }
	    round.row_status.assign(static_cast<std::size_t>(std::max(0, rows)),
	                            '?');
	    round.row_value.assign(
	        static_cast<std::size_t>(std::max(0, rows)),
	        std::numeric_limits<double>::quiet_NaN());
	    for (int r = 0; r < rows; ++r) {
	      const HighsBasisStatus st =
	          basis.valid && r < static_cast<int>(basis.row_status.size())
	              ? basis.row_status[static_cast<std::size_t>(r)]
	              : HighsBasisStatus::kNonbasic;
	      round.row_status[static_cast<std::size_t>(r)] =
	          root_highs_status_char(st);
	      if (r < static_cast<int>(sol.row_value.size())) {
	        round.row_value[static_cast<std::size_t>(r)] =
	            sol.row_value[static_cast<std::size_t>(r)];
	      }
	    }
	  };

		  std::shared_ptr<Highs> root_xpool_authority_highs_owner;
		  Highs* root_xpool_authority_lpsolver = nullptr;
		  RootHighsLiveXpoolOracle root_xpool_live_oracle;
		  double strict_highs_root_objective_offset_for_bound =
		      strict_highs_presolved_working_space
		          ? strict_highs_presolved_objective_offset
		          : 0.0;

	  auto import_root_highs_oracle_state =
	      [&](const RootHighsXpoolResolveReplayRound& oracle,
	          const LPModel& oracle_lp,
	          const Eigen::VectorXd& oracle_lb,
	          const Eigen::VectorXd& oracle_ub) -> bool {
	    (void)oracle_lb;
	    (void)oracle_ub;
	    const int rows = static_cast<int>(oracle_lp.A.rows() + oracle_lp.Aeq.rows());
	    const int cols = static_cast<int>(oracle_lp.vars.size());
	    if (oracle.resolve.rows != rows || oracle.resolve.cols != cols ||
	        static_cast<int>(oracle.col_status.size()) < cols ||
	        static_cast<int>(oracle.col_value.size()) < cols ||
	        static_cast<int>(oracle.row_status.size()) < rows ||
	        static_cast<int>(oracle.row_value.size()) < rows) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 reason=incomplete_ledger "
	                 "ledgerRows={} rows={} ledgerCols={} cols={} cstate={} "
	                 "rstate={}\n",
	                 oracle.resolve.rows, rows, oracle.resolve.cols, cols,
	                 static_cast<int>(oracle.col_status.size()),
	                 static_cast<int>(oracle.row_status.size()));
	      return false;
	    }
	    if (root_xpool_authority_lpsolver == nullptr) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                 "reason=missing_same_highslprelaxation_lpsolver; "
	                 "strict root refuses shadow Direct LP handle\n");
	      return false;
	    }
	    std::shared_ptr<Highs> highs(
	        root_xpool_authority_highs_owner,
	        root_xpool_authority_lpsolver);
	    if (!highs) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                 "reason=expired_same_highslprelaxation_lpsolver\n");
	      return false;
	    }
	    const HighsLp& authority_lp = highs->getLp();
	    if (authority_lp.num_row_ != rows || authority_lp.num_col_ != cols) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                 "reason=authority_shape_mismatch rows={}/{} cols={}/{}\n",
	                 static_cast<int>(authority_lp.num_row_), rows,
	                 static_cast<int>(authority_lp.num_col_), cols);
	      return false;
	    }
	    HighsBasis hbasis;
	    hbasis = highs->getBasis();
	    if (!hbasis.valid ||
	        static_cast<int>(hbasis.col_status.size()) < cols ||
	        static_cast<int>(hbasis.row_status.size()) < rows) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                 "reason=authority_basis_incomplete cstate={} rstate={}\n",
	                 static_cast<int>(hbasis.col_status.size()),
	                 static_cast<int>(hbasis.row_status.size()));
	      return false;
	    }
	    const HighsSolution& hsol_ref = highs->getSolution();
	    HighsSolution oracle_solution;
	    oracle_solution.value_valid = true;
	    oracle_solution.col_value = hsol_ref.col_value;
	    const RootXpoolStateSignature imported_sig =
	        root_xpool_state_signature_from_highs(
	            oracle_lp, oracle_solution, hbasis, cols,
	            root_implied_integer_cols, highs_mip_feastol);
	    const bool state_hash_match =
	        imported_sig.fractional == oracle.resolve.frac &&
	        imported_sig.frontier_hash == oracle.resolve.frontier_hash &&
	        imported_sig.status_hash == oracle.resolve.status_hash &&
	        imported_sig.basis_hash == oracle.resolve.basis_hash &&
	        imported_sig.basic_cols == oracle.resolve.basic_cols &&
	        imported_sig.lower_cols == oracle.resolve.lower_cols &&
	        imported_sig.upper_cols == oracle.resolve.upper_cols &&
	        imported_sig.zero_cols == oracle.resolve.zero_cols &&
	        imported_sig.other_cols == oracle.resolve.other_cols;
	    fmt::print(stderr,
	               "[B&C-XPOOL-STATE-VERIFY] ok={} frac={}/{} "
	               "frontierHash={:016x}/{:016x} "
	               "statusHash={:016x}/{:016x} "
	               "basisHash={:016x}/{:016x} "
	               "basis=basic{}/{}:lower{}/{}:upper{}/{}:"
	               "zero{}/{}:other{}/{} src=highs_root_live_snapshot\n",
	               state_hash_match ? 1 : 0, imported_sig.fractional,
	               oracle.resolve.frac, imported_sig.frontier_hash,
	               oracle.resolve.frontier_hash, imported_sig.status_hash,
	               oracle.resolve.status_hash, imported_sig.basis_hash,
	               oracle.resolve.basis_hash, imported_sig.basic_cols,
	               oracle.resolve.basic_cols, imported_sig.lower_cols,
	               oracle.resolve.lower_cols, imported_sig.upper_cols,
	               oracle.resolve.upper_cols, imported_sig.zero_cols,
	               oracle.resolve.zero_cols, imported_sig.other_cols,
	               oracle.resolve.other_cols);
	    if (!state_hash_match) {
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                 "reason=oracle_ledger_hash_mismatch\n");
	      return false;
	    }

		    LPRelaxationResult imported;
		    imported.primal.stats.solver_name = "HighsRootOracleReplay";
		    imported.primal.stats.status = oracle.resolve.status;
		    imported.primal.stats.success = oracle.resolve.status == "optimal";
		    const double oracle_obj_with_offset =
		        oracle.resolve.obj + strict_highs_root_objective_offset_for_bound;
		    imported.primal.stats.objective = oracle_obj_with_offset;
		    imported.primal.stats.iterations = oracle.resolve.iter_delta;
		    imported.primal.x = Eigen::VectorXd::Zero(cols);
	    for (int j = 0; j < cols; ++j) {
	      imported.primal.x[j] = hsol_ref.col_value[static_cast<std::size_t>(j)];
	    }
		    imported.dual_bound = oracle_obj_with_offset;
	    auto simplex = std::make_shared<SimplexResult>();
	    simplex->result = imported.primal;
	    simplex->result.x = imported.primal.x;
	    simplex->form.n_original = cols;
	    simplex->form.original_types.reserve(static_cast<std::size_t>(cols));
	    simplex->form.lb_shift = Eigen::VectorXd::Zero(cols);
	    for (int j = 0; j < cols; ++j) {
	      simplex->form.original_types.push_back(
	          oracle_lp.vars[static_cast<std::size_t>(j)].type);
	      simplex->form.lb_shift[j] =
	          oracle_lp.vars[static_cast<std::size_t>(j)].lb;
	    }
	    simplex->form.n_slack = rows;
	    simplex->form.n_surplus = 0;
	    simplex->form.n_artificial = 0;
	    simplex->form.row_to_slack_col.assign(static_cast<std::size_t>(rows), -1);
	    simplex->form.row_to_surplus_col.assign(static_cast<std::size_t>(rows), -1);
	    simplex->form.row_to_artificial_col.assign(static_cast<std::size_t>(rows), -1);
	    simplex->form.row_sign.assign(static_cast<std::size_t>(rows), 1);
	    simplex->form.row_rhs_value = Eigen::VectorXd::Zero(rows);
	    for (int r = 0; r < rows; ++r) {
	      simplex->form.row_to_slack_col[static_cast<std::size_t>(r)] = cols + r;
	      simplex->form.row_rhs_value[r] =
	          r < oracle_lp.b.size() ? oracle_lp.b[r] : 0.0;
	    }
	    std::vector<Eigen::Triplet<double>> triplets;
	    triplets.reserve(static_cast<std::size_t>(oracle_lp.A.nonZeros() + rows));
	    for (int col = 0; col < oracle_lp.A.outerSize(); ++col) {
	      for (Eigen::SparseMatrix<double>::InnerIterator it(oracle_lp.A, col);
	           it; ++it) {
	        if (std::abs(it.value()) > 1e-15) {
	          triplets.emplace_back(static_cast<int>(it.row()), col, it.value());
	        }
	      }
	    }
	    for (int r = 0; r < rows; ++r) triplets.emplace_back(r, cols + r, -1.0);
	    simplex->form.A.resize(rows, cols + rows);
	    simplex->form.A.setFromTriplets(triplets.begin(), triplets.end());
	    simplex->form.A.makeCompressed();
	    simplex->form.A_row = simplex->form.A;
	    simplex->form.b = Eigen::VectorXd::Zero(rows);
	    simplex->form.c_max = Eigen::VectorXd::Zero(cols + rows);
	    const Eigen::VectorXd c_min =
	        oracle_lp.sense == Sense::Minimize ? oracle_lp.c : (-oracle_lp.c);
	    simplex->form.c_max.head(cols) = -c_min;
	    simplex->form.var_ub = Eigen::VectorXd::Constant(
	        cols + rows, std::numeric_limits<double>::infinity());
	    for (int j = 0; j < cols; ++j) {
	      if (std::isfinite(oracle_lp.vars[static_cast<std::size_t>(j)].ub) &&
	          std::isfinite(oracle_lp.vars[static_cast<std::size_t>(j)].lb)) {
	        simplex->form.var_ub[j] =
	            oracle_lp.vars[static_cast<std::size_t>(j)].ub -
	            oracle_lp.vars[static_cast<std::size_t>(j)].lb;
	      }
	    }
	    for (int r = 0; r < rows; ++r) {
	      const double lhs = lp_row_lhs_or_neg_inf(oracle_lp, r);
	      const double rhs = oracle_lp.b[r];
	      if (std::isfinite(lhs) && std::isfinite(rhs)) {
	        simplex->form.var_ub[cols + r] = std::max(0.0, rhs - lhs);
	      }
	    }
	    simplex->x_std = Eigen::VectorXd::Zero(cols + rows);
	    for (int j = 0; j < cols; ++j) {
	      simplex->x_std[j] =
	          oracle.col_value[static_cast<std::size_t>(j)] -
	          oracle_lp.vars[static_cast<std::size_t>(j)].lb;
	    }
	    for (int r = 0; r < rows; ++r) {
	      const double lhs = lp_row_lhs_or_neg_inf(oracle_lp, r);
	      const double rhs = oracle_lp.b[r];
	      const double row_value =
	          r < static_cast<int>(hsol_ref.row_value.size())
	              ? hsol_ref.row_value[static_cast<std::size_t>(r)]
	              : oracle.row_value[static_cast<std::size_t>(r)];
	      if (std::isfinite(lhs)) {
	        simplex->x_std[cols + r] = row_value - lhs;
	      } else if (std::isfinite(rhs)) {
	        simplex->x_std[cols + r] = row_value - rhs;
	      } else {
	        simplex->x_std[cols + r] = row_value;
	      }
	    }
	    simplex->basis.rows = rows;
	    simplex->basis.cols = cols + rows;
	    simplex->basis.indices.assign(static_cast<std::size_t>(rows), -1);
	    simplex->basis.at_upper.assign(static_cast<std::size_t>(cols + rows), 0);
	    simplex->basis.sf_n_slack = rows;
	    simplex->basis.sf_n_surplus = 0;
	    simplex->basis.sf_n_artificial = 0;
	    const HighsBasis& factored_basis = hbasis;
	    std::vector<HighsInt> basic(static_cast<std::size_t>(rows), 0);
	    bool have_basic_variables = rows <= 0 ||
	        highs->getBasicVariables(basic.data()) == HighsStatus::kOk;
	    if (have_basic_variables) {
	      for (int r = 0; r < rows; ++r) {
	        const HighsInt hbasic = basic[static_cast<std::size_t>(r)];
	        const int col = hbasic >= 0 ? static_cast<int>(hbasic)
	                                    : cols + static_cast<int>(-hbasic - 1);
	        if (col < 0 || col >= cols + rows) {
	          fmt::print(stderr,
	                     "[B&C-XPOOL-STATE-IMPORT] ok=0 reason=bad_basic col={}\n",
	                     col);
	          return false;
	        }
	        simplex->basis.indices[static_cast<std::size_t>(r)] = col;
	      }
	    } else {
	      std::vector<unsigned char> used(static_cast<std::size_t>(cols + rows), 0);
	      int slot = 0;
	      auto add_basic_col = [&](int col) {
	        if (slot >= rows || col < 0 || col >= cols + rows ||
	            used[static_cast<std::size_t>(col)] != 0) {
	          return;
	        }
	        simplex->basis.indices[static_cast<std::size_t>(slot++)] = col;
	        used[static_cast<std::size_t>(col)] = 1;
	      };
	      for (int j = 0; j < cols; ++j) {
	        if (j < static_cast<int>(factored_basis.col_status.size()) &&
	            factored_basis.col_status[static_cast<std::size_t>(j)] ==
	                HighsBasisStatus::kBasic) {
	          add_basic_col(j);
	        }
	      }
	      for (int r = 0; r < rows; ++r) {
	        if (r < static_cast<int>(factored_basis.row_status.size()) &&
	            factored_basis.row_status[static_cast<std::size_t>(r)] ==
	                HighsBasisStatus::kBasic) {
	          add_basic_col(cols + r);
	        }
	      }
	      for (int r = 0; slot < rows && r < rows; ++r) add_basic_col(cols + r);
	      for (int j = 0; slot < rows && j < cols; ++j) add_basic_col(j);
	      if (slot != rows) {
	        fmt::print(stderr,
	                   "[B&C-XPOOL-STATE-IMPORT] ok=0 "
	                   "reason=synth_basic_incomplete rows={} filled={}\n",
	                   rows, slot);
	        return false;
	      }
	      fmt::print(stderr,
	                 "[B&C-XPOOL-STATE-IMPORT] warning=synth_basic_indices "
	                 "rows={} src=oracle_status getBasicVariables=failed\n",
	                 rows);
	    }
	    for (int j = 0; j < cols; ++j) {
	      const HighsBasisStatus st =
	          j < static_cast<int>(factored_basis.col_status.size())
	              ? factored_basis.col_status[static_cast<std::size_t>(j)]
	              : HighsBasisStatus::kNonbasic;
	      simplex->basis.at_upper[static_cast<std::size_t>(j)] =
	          st == HighsBasisStatus::kUpper ? 1 : 0;
	    }
	    for (int r = 0; r < rows; ++r) {
	      const HighsBasisStatus st =
	          r < static_cast<int>(factored_basis.row_status.size())
	              ? factored_basis.row_status[static_cast<std::size_t>(r)]
	              : HighsBasisStatus::kNonbasic;
	      simplex->basis.at_upper[static_cast<std::size_t>(cols + r)] =
	          st == HighsBasisStatus::kUpper ? 1 : 0;
	    }
	    simplex->x_basic = Eigen::VectorXd::Zero(rows);
	    for (int r = 0; r < rows; ++r) {
	      const int col = simplex->basis.indices[static_cast<std::size_t>(r)];
	      simplex->x_basic[r] =
	          col >= 0 && col < simplex->x_std.size() ? simplex->x_std[col] : 0.0;
	    }
	    if (have_basic_variables) {
	      simplex->basis.cached_sparse_basis =
	          std::make_shared<RootHighsOracleBasisOps>(
	              highs, rows, cols + rows, &simplex->form.A);
	    }
	    auto bh = std::make_shared<SimplexBasis>(simplex->basis);
	    if (simplex->basis.cached_sparse_basis) {
	      bh->cached_sparse_basis = simplex->basis.cached_sparse_basis;
	    }
	    imported.simplex = simplex;
	    imported.basis_hint = bh;
		    root_lp = oracle_lp;
		    root.x_relax = imported.primal.x;
		    root.x_seed = root.x_relax;
		    root.bound = imported.dual_bound;
		    out.bc_stats.root_only_solver_bound = oracle.resolve.obj;
		    out.bc_stats.root_only_objective_offset =
		        strict_highs_root_objective_offset_for_bound;
		    root_relax = std::move(imported);
	    root.basis_hint = root_relax.basis_hint;
	    root_sbasis = simplex->basis.cached_sparse_basis;
	    root_first_class_sf_valid = false;
	    refresh_root_lp_source_solution();
		    fmt::print(stderr,
		               "[B&C-XPOOL-STATE-IMPORT] ok=1 rows={} cols={} "
		               "solverObj={:.17g} offset={:.17g} bound={:.17g} "
		               "frontierHash={:016x} basisHash={:016x} "
		               "src=highs_root_live_snapshot\n",
		               rows, cols, oracle.resolve.obj,
		               strict_highs_root_objective_offset_for_bound, root.bound,
		               oracle.resolve.frontier_hash,
		               oracle.resolve.basis_hash);
	    return true;
	  };

	  auto root_native_lp_from_highs_lp =
	      [&](const HighsLp& highs_lp, Sense native_sense) -> LPModel {
	    LPModel out_lp;
	    out_lp.sense = native_sense;
	    const int cols = static_cast<int>(highs_lp.num_col_);
	    const int rows = static_cast<int>(highs_lp.num_row_);
	    out_lp.vars.resize(static_cast<std::size_t>(std::max(0, cols)));
	    out_lp.c = Eigen::VectorXd::Zero(std::max(0, cols));
	    for (int j = 0; j < cols; ++j) {
	      const double lb =
	          j < static_cast<int>(highs_lp.col_lower_.size())
	              ? highs_lp.col_lower_[static_cast<std::size_t>(j)]
	              : -std::numeric_limits<double>::infinity();
	      const double ub =
	          j < static_cast<int>(highs_lp.col_upper_.size())
	              ? highs_lp.col_upper_[static_cast<std::size_t>(j)]
	              : std::numeric_limits<double>::infinity();

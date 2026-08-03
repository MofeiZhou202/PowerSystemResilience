// @file bc_run/14_parallel_path_and_result.inc
// BCSolveState::run() segment 14/14: heterogeneous parallel exploration path and final result assembly / postsolve.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
                  ? static_cast<int>(std::floor(
                        probe_budget_remaining /
                        (2.2 * measured_ipm_solve_sec)))
                  : 0;
          const int eff_max = (cur.depth == 0)
                                  ? (!cur.basis_hint ||
                                             skip_short_basis_free_root_probing
                                         ? 0
                                         : std::min(
                                               {opt.probe_max_candidates, 3,
                                                root_batch_candidate_capacity}))
                                  : allow_tree_probe ? 1 : 0;
          const int eff_rel = (cur.depth == 0) ? opt.probe_reliability
                              : allow_tree_probe ? 1 : 0;
          std::vector<int> unreliable;
          std::vector<StrongBranchProbeResult> ipm_probe_results;
          if (eff_max > 0) {
            for (int jj : frac) {
              if (pc[jj].down_cnt < eff_rel || pc[jj].up_cnt < eff_rel)
                unreliable.push_back(jj);
            }
          }
          const int provisional = choose_pseudocost_branch_var_for_node(frac);
          initial_probe_winner = provisional;
          if (std::find(unreliable.begin(), unreliable.end(), provisional) ==
              unreliable.end()) {
            unreliable.clear();
          }
          if (!unreliable.empty()) {
            const int probes = std::min(eff_max, static_cast<int>(unreliable.size()));
            ipm_probe_results.reserve(static_cast<std::size_t>(2 * probes));
            auto classify_ipm_probe = [&](const SolveResult& res) {
              if (res.stats.success && res.x.size() == n &&
                  std::isfinite(res.stats.objective)) {
                return LPFailureType::Success;
              }
              if (res.stats.has_farkas_certificate) {
                return LPFailureType::Infeasible;
              }
              const std::string& status = res.stats.status;
              if (status.find("Time limit") != std::string::npos ||
                  status.find("MaxIter") != std::string::npos) {
                return LPFailureType::Timeout;
              }
              if (status.find("Unbounded") != std::string::npos ||
                  status.find("unbounded") != std::string::npos) {
                return LPFailureType::Unbounded;
              }
              return LPFailureType::NumericalFailure;
            };
            auto make_ipm_probe = [&](int jj, bool is_up,
                                      bool materialize_domain) {
              const double xjj = cur.x_relax[jj];
              StrongBranchProbeResult pr;
              pr.available = true;
              pr.from_ipm = true;
              pr.var = jj;
              pr.is_up = is_up;
              pr.local_row_signature =
                  ordered_local_cut_signature(cur.local_cuts);
              pr.model_epoch = current_domain_closure_epoch();
              pr.objective_epoch = current_objective_artifact_epoch();
              if (!materialize_domain) return pr;
              pr.lb = cur.lb;
              pr.ub = cur.ub;
              if (is_up) {
                pr.lb[jj] =
                    std::max(pr.lb[jj], std::floor(xjj) + 1.0);
              } else {
                pr.ub[jj] = std::min(pr.ub[jj], std::floor(xjj));
              }
              return pr;
            };
            auto record_ipm_probe = [&](StrongBranchProbeResult pr) {
              local_probe_lp_iterations += static_cast<std::uint64_t>(
                  std::max(0, pr.ipm_result.stats.iterations));
              const double frac_j =
                  cur.x_relax[pr.var] - std::floor(cur.x_relax[pr.var]);
              const double distance = pr.is_up ? 1.0 - frac_j : frac_j;
              if (pr.failure_type == LPFailureType::Success) {
                pr.evidence_state = BranchEvidenceState::Measured;
                pr.observed_gain = std::max(
                    0.0, pr.ipm_result.stats.objective - cur.bound);
              } else if (pr.failure_type == LPFailureType::Infeasible) {
                pr.evidence_state = BranchEvidenceState::ProvenCutoff;
              } else {
                pr.evidence_state = BranchEvidenceState::UnknownFailure;
              }
              update_pseudocost_from_branch_evidence(
                  pc[pr.var], pr.is_up, pr.evidence_state, pr.observed_gain,
                  distance,
                  pr.failure_type == LPFailureType::Infeasible &&
                      pr.ipm_result.stats.has_farkas_certificate);
              ipm_probe_results.push_back(std::move(pr));
            };

            int candidates_probed = 0;
            while (candidates_probed < probes) {
              const int jj = choose_with_probe_overlay(frac, ipm_probe_results);
              if (jj < 0) break;
              const auto already_observed = [&](bool is_up) {
                return std::any_of(
                    ipm_probe_results.begin(), ipm_probe_results.end(),
                    [&](const auto& result) {
                      return result.var == jj && result.is_up == is_up;
                    });
              };
              const bool need_down =
                  pc[jj].down_cnt < eff_rel && !already_observed(false);
              const bool need_up =
                  pc[jj].up_cnt < eff_rel && !already_observed(true);
              if (!need_down && !need_up) break;

            if (cur.local_cuts.empty()) {
              std::vector<StrongBranchProbeResult> pending;
              std::vector<IPMNodeBoundChange> changes;
              pending.reserve(2);
              changes.reserve(2);
              for (bool is_up : {false, true}) {
                  if ((is_up && !need_up) || (!is_up && !need_down)) continue;
                  auto pr = make_ipm_probe(jj, is_up,
                                           /*materialize_domain=*/false);
                  const double xjj = cur.x_relax[jj];
                  IPMNodeBoundChange change;
                  change.variable = jj;
                  if (is_up) {
                    change.lower_bound =
                        std::max(cur.lb[jj], std::floor(xjj) + 1.0);
                    change.upper_bound = cur.ub[jj];
                  } else {
                    change.lower_bound = cur.lb[jj];
                    change.upper_bound =
                        std::min(cur.ub[jj], std::floor(xjj));
                  }
                  if (change.lower_bound > change.upper_bound) {
                    pr.failure_type = LPFailureType::Infeasible;
                    record_ipm_probe(std::move(pr));
                    continue;
                  }
                  pending.push_back(std::move(pr));
                  changes.push_back(change);
              }

              const double batch_budget =
                  bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
              if (batch_budget <= 0.001) {
                for (auto& pr : pending) {
                  pr.failure_type = LPFailureType::Timeout;
                  record_ipm_probe(std::move(pr));
                }
              } else {
                const auto batch_start = std::chrono::steady_clock::now();
                auto batch = ipm_node_solver->solve_cached_bound_change_batch(
                    cur.lb, cur.ub, cur.x_relax, changes, batch_budget);
                int batch_attempted = 0;
                int batch_succeeded = 0;
                for (std::size_t i = 0; i < pending.size(); ++i) {
                  auto& pr = pending[i];
                  if (i < batch.size()) {
                    if (batch[i].attempted) {
                      ++local_lp_solves;
                      ++batch_attempted;
                    }
                    if (batch[i].result.stats.success) ++batch_succeeded;
                    pr.ipm_result = std::move(batch[i].result);
                    pr.failure_type = classify_ipm_probe(pr.ipm_result);
                  } else {
                    pr.failure_type = LPFailureType::Timeout;
                  }
                  record_ipm_probe(std::move(pr));
                }
                if (opt.verbose || trace_bc_timeline) {
                  const double batch_ms =
                      std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - batch_start)
                          .count();
                  fmt::print(stderr,
                             "[B&C-IPM-BATCH] depth={} entries={} "
                             "attempted={} success={} wall={:.1f}ms "
                             "budget={:.3f}s\n",
                             cur.depth, changes.size(), batch_attempted,
                             batch_succeeded, batch_ms, batch_budget);
                }
              }
            } else {
              int remaining_probe_solves =
                  static_cast<int>(need_down) + static_cast<int>(need_up);
              for (bool is_up : {false, true}) {
                  if ((is_up && !need_up) || (!is_up && !need_down)) continue;
                  auto pr = make_ipm_probe(jj, is_up,
                                           /*materialize_domain=*/true);
                  if (!bounds_consistent(pr.lb, pr.ub)) {
                    pr.failure_type = LPFailureType::Infeasible;
                  } else {
                    const double solve_remaining =
                        bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
                    if (solve_remaining <= 0.001) {
                      pr.failure_type = LPFailureType::Timeout;
                    } else {
                      ipm_cut_solver.set_solve_time_limit(std::max(
                          0.001, solve_remaining /
                                     std::max(1, remaining_probe_solves)));
                      Node probe_node = cur;
                      probe_node.lb = pr.lb;
                      probe_node.ub = pr.ub;
                      LPModel probe_lp =
                          build_node_lp_with_local_cuts(probe_node);
                      pr.ipm_result =
                          ipm_cut_solver.solve_lp(probe_lp, cur.x_relax);
                      ++local_lp_solves;
                      pr.failure_type =
                          classify_ipm_probe(pr.ipm_result);
                    }
                  }
                  --remaining_probe_solves;
                  record_ipm_probe(std::move(pr));
              }
            }
              ++candidates_probed;
              ++local_strong_branch_candidates;
            }
          }
	          j = choose_with_probe_overlay(frac, ipm_probe_results);
	          record_strong_branch_regret(ipm_probe_results, j);
          record_probe_selection_telemetry(ipm_probe_results, j);
          for (auto& pr : ipm_probe_results) {
            if (pr.var != j) continue;
            if (pr.from_ipm && pr.lb.size() != n) {
              pr.lb = cur.lb;
              pr.ub = cur.ub;
              const double xjj = cur.x_relax[pr.var];
              if (pr.is_up) {
                pr.lb[pr.var] =
                    std::max(pr.lb[pr.var], std::floor(xjj) + 1.0);
              } else {
                pr.ub[pr.var] =
                    std::min(pr.ub[pr.var], std::floor(xjj));
              }
            }
            if (pr.is_up) retained_up_probe = std::move(pr);
            else retained_down_probe = std::move(pr);
          }
	        } else {
	          // HiGHS strong-branching is a proof-bearing node evaluation.  Keep
	          // the child LP states produced by probing so the selected branch
	          // directions can consume their x/basis/reduced-cost frontier instead
	          // of immediately solving the same child LP a second time.
	          const int eff_reliability =
	              (cur.depth == 0) ? opt.probe_reliability : probe_reliability;
	          const int eff_max = (cur.depth == 0) ? root_probe_max : probe_max;
	          std::vector<StrongBranchProbeResult> probe_results;
	          if (cur.depth <= 6 && eff_max != 0) {
	            std::vector<int> unreliable;
	            unreliable.reserve(frac.size());
	            for (int jj : frac) {
	              const int limit =
	                  (pc[jj].down_cnt >= 1 && pc[jj].up_cnt >= 1)
	                      ? std::max(2, eff_reliability / 2)
	                      : eff_reliability;
	              if (pc[jj].down_cnt < limit || pc[jj].up_cnt < limit) {
	                unreliable.push_back(jj);
	              }
	            }
	            const int provisional =
	                choose_pseudocost_branch_var_for_node(frac);
	            initial_probe_winner = provisional;
	            if (std::find(unreliable.begin(), unreliable.end(), provisional) ==
	                unreliable.end()) {
	              unreliable.clear();
	            }
	            if (!unreliable.empty()) {
	              const int probes =
	                  std::min(cur.depth == 0 ? eff_max : std::max(2, eff_max / 2),
	                           static_cast<int>(unreliable.size()));
	              probe_results.reserve(static_cast<std::size_t>(2 * probes));

	              auto update_pc_from_probe =
	                  [&](StrongBranchProbeResult& pr) {
	                if (pr.var < 0 || pr.var >= static_cast<int>(pc.size())) return;
	                const double frac_j = cur.x_relax[pr.var] -
	                                      std::floor(cur.x_relax[pr.var]);
	                const double distance = pr.is_up ? 1.0 - frac_j : frac_j;
	                if (pr.failure_type == LPFailureType::Success) {
	                  pr.evidence_state = pr.simplex.exact_optimal
	                                          ? BranchEvidenceState::ExactOptimal
	                                          : BranchEvidenceState::Measured;
	                  pr.observed_gain = std::max(
	                      0.0, pr.simplex.result.stats.objective - cur.bound);
	                } else if (pr.failure_type == LPFailureType::ObjectiveCutoff ||
	                           (pr.failure_type == LPFailureType::Infeasible &&
	                            (pr.simplex.result.stats.has_farkas_certificate ||
	                             !bounds_consistent(pr.lb, pr.ub)))) {
	                  pr.evidence_state = BranchEvidenceState::ProvenCutoff;
	                } else {
	                  pr.evidence_state = BranchEvidenceState::UnknownFailure;
	                }
	                update_pseudocost_from_branch_evidence(
	                    pc[pr.var], pr.is_up, pr.evidence_state,
	                    pr.observed_gain, distance,
	                    pr.failure_type == LPFailureType::Infeasible &&
	                        pr.simplex.result.stats.has_farkas_certificate);
	              };

	              auto collect_probe_integral_candidate =
	                  [&](const StrongBranchProbeResult& pr) {
	                if (pr.failure_type != LPFailureType::Success ||
	                    pr.simplex.result.x.size() != n) {
	                  return;
	                }
	                Eigen::VectorXd x_probe =
	                    clamp_to_bounds(pr.simplex.result.x, pr.lb, pr.ub);
	                std::vector<int> probe_frac;
		                if (fractional_branchable_indices(
		                        x_probe, pr.lb, pr.ub,
		                        requested_vendored_highs_lp
		                            ? std::max(1e-9, opt.lp_tol)
		                            : opt.int_tol,
		                        probe_frac)) {
	                  return;
	                }
		                round_declared_branchables(x_probe, pr.lb, pr.ub);
	                if (!satisfies_with_bounds(
	                        base_lp, x_probe, pr.lb, pr.ub,
	                        requested_vendored_highs_lp
	                            ? std::max(kNodeFeasibilityTol, opt.lp_tol)
	                            : kNodeFeasibilityTol)) {
	                  return;
	                }
	                const double obj =
	                    objective_value(base_lp.c, x_probe, base_lp.sense);
	                if (adopt_tree_incumbent("strong_branch_probe", x_probe, obj,
	                                         cur.depth + 1, global_lb)) {
	                  incumbent_changed_by_probe = true;
	                }
	              };

	              std::optional<LPModel> active_probe_lp;
	              std::optional<StandardFormLP> active_probe_sf;
	              auto ensure_active_probe_sf = [&]() -> StandardFormLP& {
	                if (active_probe_sf.has_value()) return *active_probe_sf;
	                if (cur.local_cuts.empty()) {
	                  active_probe_sf.emplace(tree_base_sf);
	                  update_standard_form_bounds(
	                      *active_probe_sf, base_lp, cur.lb, cur.ub);
	                } else {
	                  active_probe_lp.emplace(
	                      build_node_lp_with_local_cuts(cur));
	                  active_probe_sf.emplace(
	                      build_standard_form_lp(*active_probe_lp));
	                  ruiz_scale_standard_form(*active_probe_sf);
	                  update_standard_form_bounds(
	                      *active_probe_sf, *active_probe_lp,
	                      cur.lb, cur.ub);
	                }
	                ++local_probe_storage.base_sf_materializations;
	                return *active_probe_sf;
	              };

	              auto run_probe = [&](int jj, bool is_up) {
	                const double xjj = cur.x_relax[jj];
	                Eigen::VectorXd plb = cur.lb;
	                Eigen::VectorXd pub = cur.ub;
	                if (is_up) {
	                  plb[jj] = std::max(plb[jj], std::floor(xjj) + 1.0);
	                } else {
	                  pub[jj] = std::min(pub[jj], std::floor(xjj));
	                }
	                StrongBranchProbeResult pr;
		                pr.available = true;
		                pr.var = jj;
		                pr.is_up = is_up;
		                pr.local_row_signature =
		                    ordered_local_cut_signature(cur.local_cuts);
		                pr.model_epoch = current_domain_closure_epoch();
		                pr.objective_epoch =
		                    current_objective_artifact_epoch();
	                pr.lb = plb;
	                pr.ub = pub;
	                if (!bounds_consistent(plb, pub)) {
	                  pr.failure_type = LPFailureType::Infeasible;
	                  update_pc_from_probe(pr);
	                  probe_results.push_back(std::move(pr));
	                  return;
	                }
	                const double probe_remaining =
	                    bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
	                if (probe_remaining <= 0.001) {
	                  simplex_node_deadline_hit = true;
	                  pr.failure_type = LPFailureType::Timeout;
	                  update_pc_from_probe(pr);
	                  probe_results.push_back(std::move(pr));
	                  return;
	                }
			                StandardFormLP& probe_sf = ensure_active_probe_sf();
			                const LPModel& probe_lp = active_probe_lp.has_value()
			                    ? *active_probe_lp : base_lp;
			                const double old_value =
			                    is_up ? cur.lb[jj] : cur.ub[jj];
			                const double new_value =
			                    is_up ? plb[jj] : pub[jj];
			                const std::vector<BoundChangeInfo> branch_change{
			                    {jj, 0.0, is_up, old_value, new_value}};
			                StandardFormBoundTransaction probe_transaction(probe_sf);
			                if (!probe_transaction.apply(probe_lp, branch_change)) {
			                  ++local_probe_storage.transaction_failures;
			                  pr.failure_type = LPFailureType::NumericalFailure;
			                  update_pc_from_probe(pr);
			                  probe_results.push_back(std::move(pr));
			                  return;
			                }
			                ++local_probe_storage.bound_transactions;
			                local_probe_storage.transaction_snapshot_values +=
			                    probe_transaction.snapshot_value_count();
	                seq_dispatcher.set_time_limit(
	                    probe_remaining, &simplex_node_deadline_hit);
		                pr.simplex = seq_dispatcher.solve(
		                    probe_sf, cur.basis_hint.get(), SolveContext::Probing,
		                    seq_node_id++, cur.depth + 1, n, pr.deferred);
		                ++local_lp_solves;
		                if (pr.simplex.result.stats.solver_name ==
		                    "VendoredHighsPersistentLpKernel") {
		                  ++local_probe_storage.backend_persistent_resolves;
		                } else {
		                  ++local_probe_storage.backend_cold_solves;
		                }
		                if (probe_transaction.rollback()) {
		                  ++local_probe_storage.transaction_rollbacks;
		                } else {
		                  ++local_probe_storage.transaction_failures;
		                  active_probe_sf.reset();
		                }
	                pr.failure_type = classify_lp_result(pr.simplex, n);
	                if (lp_wall_time_limit_reached(pr.simplex)) {
	                  simplex_node_deadline_hit = true;
	                }
		                local_probe_lp_iterations += static_cast<std::uint64_t>(
		                    std::max(0, pr.simplex.result.stats.iterations));
		                detach_probe_transient_basis_state(pr.simplex);
		                update_pc_from_probe(pr);
		                collect_probe_integral_candidate(pr);
		                probe_results.push_back(std::move(pr));
	              };

	              int candidates_probed = 0;
	              while (candidates_probed < probes) {
	                const int jj = choose_with_probe_overlay(frac, probe_results);
	                if (jj < 0) break;
	                const int limit =
	                    (pc[jj].down_cnt >= 1 && pc[jj].up_cnt >= 1)
	                        ? std::max(2, eff_reliability / 2)
	                        : eff_reliability;
	                const auto already_observed = [&](bool is_up) {
	                  return std::any_of(
	                      probe_results.begin(), probe_results.end(),
	                      [&](const auto& result) {
	                        return result.var == jj && result.is_up == is_up;
	                      });
	                };
	                const bool need_down =
	                    pc[jj].down_cnt < limit && !already_observed(false);
	                const bool need_up =
	                    pc[jj].up_cnt < limit && !already_observed(true);
	                if (!need_down && !need_up) break;
	                if (need_down) run_probe(jj, /*is_up=*/false);
	                if (need_up && !simplex_node_deadline_hit) {
	                  run_probe(jj, /*is_up=*/true);
	                }
	                ++candidates_probed;
	                ++local_strong_branch_candidates;
	                if (simplex_node_deadline_hit) break;
	              }
	            }
	          }
	                j = choose_with_probe_overlay(frac, probe_results);
	                record_strong_branch_regret(probe_results, j);
	                record_probe_selection_telemetry(probe_results, j);
		          if (j >= 0) {
		            for (auto& pr : probe_results) {
		              if (pr.var != j) continue;
		              if (pr.is_up) retained_up_probe = std::move(pr);
		              else retained_down_probe = std::move(pr);
		            }
		            if ((retained_down_probe.available || retained_up_probe.available) &&
		                (opt.verbose || trace_bc_timeline)) {
		              fmt::print(stderr,
		                         "[B&C-SB-SELECT] depth={} j={} from_probe={} "
		                         "down={} up={}\n",
			                         cur.depth, j,
			                         retained_down_probe.available ||
			                                 retained_up_probe.available
			                             ? 1
			                             : 0,
		                         retained_down_probe.available ? 1 : 0,
		                         retained_up_probe.available ? 1 : 0);
		            }
		          }
		        }
	      } else {
            j = choose_branch_var_for_node(frac);
	      }
	      if (incumbent_changed_by_probe) {
	        highs_style_post_incumbent_domain_closure(
	            tree_incumbent_domain_closure_source.c_str(),
	            tree_incumbent_domain_closure_depth);
	        prune_queue_by_gap_optimality_limit("strong_branch_probe_incumbent");
	      }
		      out.bc_stats.lp_solves += local_lp_solves;
		      out.bc_stats.strong_branch_lp_solves += local_lp_solves;
		      out.bc_stats.strong_probe_lp_iterations +=
		          local_probe_lp_iterations;
		      out.bc_stats.strong_probe_base_sf_materializations +=
		          local_probe_storage.base_sf_materializations;
		      out.bc_stats.strong_probe_bound_transactions +=
		          local_probe_storage.bound_transactions;
		      out.bc_stats.strong_probe_transaction_snapshot_values +=
		          local_probe_storage.transaction_snapshot_values;
		      out.bc_stats.strong_probe_transaction_rollbacks +=
		          local_probe_storage.transaction_rollbacks;
		      out.bc_stats.strong_probe_transaction_failures +=
		          local_probe_storage.transaction_failures;
		      out.bc_stats.strong_probe_backend_cold_solves +=
		          local_probe_storage.backend_cold_solves;
		      out.bc_stats.strong_probe_backend_persistent_resolves +=
		          local_probe_storage.backend_persistent_resolves;
		      if (local_lp_solves > 0) {
		        out.bc_stats.strong_probe_time_ms +=
		            std::chrono::duration<double, std::milli>(
		                std::chrono::steady_clock::now() - strong_probe_start)
		                .count();
		      }
		      out.bc_stats.strong_branch_candidates +=
		          local_strong_branch_candidates;
		      if (local_strong_branch_candidates > 0) {
		        ++out.bc_stats.reliability_branch_nodes;
		      }
      if (opt.verbose && local_lp_solves > 0) {
        fmt::print(stderr, "[B&C] node={} depth={} probing_lps={}\n",
                   out.bc_stats.nodes_explored, cur.depth, local_lp_solves);
	      }
	      }
	      if (ipm_node_deadline_hit || simplex_node_deadline_hit) {
	        out.stats.status = bc_status::kTimeLimitReached;
	        break;
	      }
    if (j < 0) continue;
    {
      const int selected_priority =
          (j < static_cast<int>(branch_priority.size()))
              ? branch_priority[static_cast<std::size_t>(j)]
              : 0;
      const double selected_score = compute_branch_var_score(
          j, cur.x_relax, pc,
          branch_priority.empty() ? nullptr : &branch_priority);
      double best_score = selected_score;
      for (int cand : frac) {
        const int priority =
            (cand < static_cast<int>(branch_priority.size()))
                ? branch_priority[static_cast<std::size_t>(cand)]
                : 0;
        if (priority != selected_priority) continue;
        best_score = std::max(
            best_score,
            compute_branch_var_score(
                cand, cur.x_relax, pc,
                branch_priority.empty() ? nullptr : &branch_priority));
      }
      const double regret = std::max(0.0, best_score - selected_score);
      ++out.bc_stats.branching_regret_samples;
      out.bc_stats.branching_regret_sum += regret;
      out.bc_stats.branching_regret_max =
          std::max(out.bc_stats.branching_regret_max, regret);
    }
    if (cur.has_live_factor_telemetry) {
      live_factor_telemetry = cur.live_factor_telemetry;
    } else if (cur.basis_hint && cur.basis_hint->cached_sparse_basis) {
      live_factor_telemetry =
          get_sparse_basis_factor_telemetry(cur.basis_hint->cached_sparse_basis);
    }

    if (opt.enable_ft_drift_telemetry &&
        ft_audit_samples_taken < std::max(0, opt.ft_branch_audit_max_samples) &&
        cur.depth <= opt.ft_branch_audit_depth_limit &&
        !use_ipm_nodes &&
        cur.basis_hint &&
        (cur.has_live_factor_telemetry || cur.basis_hint->cached_sparse_basis) &&
        !frac.empty()) {
      ++ft_audit_samples_taken;
      ++out.bc_stats.ft_audit_samples;
      if (live_factor_telemetry.ft_valid) ++out.bc_stats.ft_live_ft_samples;
      out.bc_stats.ft_audit_updates_sum +=
          static_cast<std::uint64_t>(std::max(0, live_factor_telemetry.ft_updates));
      out.bc_stats.ft_audit_growth_sum += live_factor_telemetry.max_growth;
      out.bc_stats.ft_audit_fill_sum += live_factor_telemetry.u_fill_ratio;

      StandardFormLP audit_sf = ensure_base_sf();
      update_standard_form_bounds(audit_sf, base_lp, cur.lb, cur.ub);
      SimplexOptions audit_opt = simplex_opt;
      audit_opt.allow_cold_start = true;
      audit_opt.incumbent_bound = nullptr;
      auto audit_res = solve_tree_lp(audit_sf, audit_opt, cur.basis_hint.get());
      if (audit_res.result.stats.success &&
          audit_res.result.x.size() == cur.x_relax.size()) {
        double best_score = -1.0;
        int best_priority = std::numeric_limits<int>::min();
        int fresh_j = -1;
        for (int cand : frac) {
          const double score = compute_branch_var_score(
              cand, audit_res.result.x, pc,
              branch_priority.empty() ? nullptr : &branch_priority);
          const int prio = (cand >= 0 &&
                            cand < static_cast<int>(branch_priority.size()))
                               ? branch_priority[static_cast<std::size_t>(cand)]
                               : 0;
          if (prio > best_priority ||
              (prio == best_priority && score > best_score)) {
            best_score = score;
            best_priority = prio;
            fresh_j = cand;
          }
        }
        if (fresh_j >= 0 && fresh_j != j) {
          ++out.bc_stats.ft_branch_choice_changes;
          out.bc_stats.ft_audit_updates_changed_sum +=
              static_cast<std::uint64_t>(std::max(0, live_factor_telemetry.ft_updates));
          out.bc_stats.ft_audit_growth_changed_sum += live_factor_telemetry.max_growth;
          out.bc_stats.ft_audit_fill_changed_sum += live_factor_telemetry.u_fill_ratio;
        }
      }
    }
    const auto loop_t2 = std::chrono::steady_clock::now();

    const double xj = cur.x_relax[j];
    const double parent_bound = cur.bound;
    const double parent_estimate = cur.estimate;
    const int parent_depth = cur.depth;
    const bool run_crossover_this_node =
        solution_pool.size() >= 2 && opt.crossover_heuristic_freq > 0 &&
        out.bc_stats.nodes_explored % opt.crossover_heuristic_freq == 0;
    const bool run_rins_this_node =
        has_incumbent && opt.rins_frequency > 0 &&
        out.bc_stats.nodes_explored % opt.rins_frequency == 0;
    const bool retain_parent_for_heuristics =
        run_crossover_this_node || run_rins_this_node;
	    Eigen::VectorXd parent_x_relax = retain_parent_for_heuristics
	        ? cur.x_relax
	        : std::move(cur.x_relax);
	    const std::uint64_t parent_domain_values =
	        static_cast<std::uint64_t>(cur.lb.size() + cur.ub.size());
	    Node child_down = cur.branch_child(&serial_payload_sharing);
	    ++out.bc_stats.branch_domain_dense_copies;
	    out.bc_stats.branch_domain_dense_values_copied +=
	        parent_domain_values;
    child_down.ub[j] = std::min(child_down.ub[j], std::floor(xj));
    child_down.x_seed = clamp_to_bounds(parent_x_relax, child_down.lb, child_down.ub);
    append_branch_reason(child_down, j, false, child_down.ub[j]);

    // The popped parent is dead after branching. Move its O(n) domain and
    // proof state into one sibling instead of copying both lb and ub again.
    Node child_up;
	    if (retain_parent_for_heuristics) {
	      child_up = cur.branch_child(&serial_payload_sharing);
	      ++out.bc_stats.branch_domain_dense_copies;
	      out.bc_stats.branch_domain_dense_values_copied +=
	          parent_domain_values;
	    } else {
	      child_up = std::move(cur);
	      ++out.bc_stats.branch_domain_moves;
      child_up.x_relax.resize(0);
      child_up.x_seed.resize(0);
      child_up.ipm_iterations = 0;
      child_up.lp_refresh_needed = false;
      child_up.depth = parent_depth + 1;
    }
    child_up.fractional_branchables_valid = false;
    child_up.lb[j] = std::max(child_up.lb[j], std::ceil(xj));
    child_up.x_seed = clamp_to_bounds(parent_x_relax, child_up.lb, child_up.ub);
    append_branch_reason(child_up, j, true, child_up.lb[j]);

    if (use_ipm_nodes && parent_depth == 0) {
      first_branch_ipm_solve_slots = 2;
    }

    BranchDirectionalObservation down_direction_observation;
    BranchDirectionalObservation up_direction_observation;
    const BranchDirectionalObservation* down_direction_ptr = nullptr;
    const BranchDirectionalObservation* up_direction_ptr = nullptr;
    if (retained_down_probe.available) {
      down_direction_observation = {
          j, false, retained_down_probe.evidence_state,
          retained_down_probe.observed_gain};
      down_direction_ptr = &down_direction_observation;
    }
    if (retained_up_probe.available) {
      up_direction_observation = {
          j, true, retained_up_probe.evidence_state,
          retained_up_probe.observed_gain};
      up_direction_ptr = &up_direction_observation;
    }
    const double branch_fractionality = xj - std::floor(xj);
    const auto history_prediction_usable =
        [](const BranchDirectionalObservation* observation) {
          return observation == nullptr ||
                 observation->state == BranchEvidenceState::Predicted ||
                 observation->state == BranchEvidenceState::UnknownFailure;
        };
    const bool down_prediction_available =
        history_prediction_usable(down_direction_ptr);
    const bool up_prediction_available =
        history_prediction_usable(up_direction_ptr);
    const double predicted_down_gain = compute_directional_branch_score(
        pc[j], branch_fractionality, false, nullptr);
    const double predicted_up_gain = compute_directional_branch_score(
        pc[j], 1.0 - branch_fractionality, true, nullptr);
    const bool prefer_up = choose_up_branch_first(
        pc[j], branch_fractionality, down_direction_ptr, up_direction_ptr,
        has_incumbent);
    if (prefer_up) {
      ++out.bc_stats.branch_direction_preferred_up;
      ++out.bc_stats.branch_direction_first_up;
    } else {
      ++out.bc_stats.branch_direction_preferred_down;
      ++out.bc_stats.branch_direction_first_down;
    }

    bool down_valid = false;
    bool up_valid = false;
    bool incumbent_changed_this_node = false;
    bool child_domain_source_added = false;
    auto process_direction = [&](bool is_up, int ordinal) {
      Node& child = is_up ? child_up : child_down;
      bool& child_valid = is_up ? up_valid : down_valid;
      StrongBranchProbeResult* retained_probe =
          is_up ? &retained_up_probe : &retained_down_probe;
      const double branch_distance =
          is_up ? std::ceil(xj) - xj : xj - std::floor(xj);
      const char* direction = is_up ? "up" : "down";
      const char* solution_source =
          is_up ? "up_child_solution" : "down_child_solution";
      const char* cut_source =
          is_up ? "up_child_new_cut" : "down_child_new_cut";

      // Each direction reuses one workspace but starts from pristine rows.
      if (node_sf_cuts_augmented) {
        node_sf = tree_base_sf;
        node_sf_cuts_augmented = false;
      }
      auto [valid, pcu, sols, ncuts, lps, ca, ps] = process_single_child(
          child, is_up, j, branch_distance, parent_bound, node_sf, cut_pool,
          incumbent_obj, has_incumbent, retained_probe);
      child_valid = valid;
      if (trace_bc_timeline) {
        fmt::print(stderr,
                   "[B&C-CHILD] dir={} ordinal={} depth={} var={} x={:.10g} "
                   "valid={} bound={:.10g} sols={} lps={} lb={:.10g} "
                   "ub={:.10g}\n",
                   direction, ordinal, child.depth, j, xj,
                   child_valid ? 1 : 0, child.bound, sols.size(), lps,
                   child.lb[j], child.ub[j]);
      }
      out.bc_stats.lp_solves += lps;
      out.bc_stats.cuts_added += ca;
      out.bc_stats.pool_cuts_separated += ps;
      const bool cutoff = std::any_of(
          pcu.begin(), pcu.end(),
          [](const PCUpdate& update) { return update.cutoff_count > 0; });
      if (cutoff) {
        if (ordinal == 0) ++out.bc_stats.branch_first_child_cutoffs;
        else ++out.bc_stats.branch_second_child_cutoffs;
      }
      for (const auto& update : pcu) update.apply(pc[update.var]);

      bool incumbent_changed_in_direction = false;
      for (const auto& solution : sols) {
        if (adopt_tree_incumbent(solution_source, solution.x, solution.obj,
                                 child.depth, global_lb)) {
          incumbent_changed_in_direction = true;
          incumbent_changed_this_node = true;
        }
      }
      if (ordinal == 0 && incumbent_changed_in_direction) {
        ++out.bc_stats.branch_first_child_incumbent_updates;
      }
      for (auto& cut : ncuts) {
        if (valid_pool_cut(cut, nullptr, cut_source)) {
          child_domain_source_added =
              add_cut_to_global_domain_pool(
                  std::move(cut.coeff), cut.rhs, cut_source, cut.domain_only,
                  cut.validity_scope) ||
              child_domain_source_added;
        }
      }
    };
    const std::array<bool, 2> direction_order{prefer_up, !prefer_up};
    for (int ordinal = 0; ordinal < 2; ++ordinal) {
      if (ipm_node_deadline_hit || simplex_node_deadline_hit) break;
      process_direction(direction_order[static_cast<std::size_t>(ordinal)],
                        ordinal);
    }
    const auto calibration = compute_branch_estimator_calibration(
        parent_bound, parent_estimate, down_prediction_available,
        predicted_down_gain, up_prediction_available, predicted_up_gain,
        down_valid, child_down.bound, up_valid, child_up.bound);
    out.bc_stats.node_estimate_calibration_samples += calibration.node_samples;
    out.bc_stats.node_estimate_predicted_lift_sum +=
        calibration.node_predicted_lift_sum;
    out.bc_stats.node_estimate_realized_lift_sum +=
        calibration.node_realized_lift_sum;
    out.bc_stats.node_estimate_abs_error_sum +=
        calibration.node_abs_error_sum;
    out.bc_stats.node_estimate_squared_error_sum +=
        calibration.node_squared_error_sum;
    out.bc_stats.node_estimate_predicted_sq_sum +=
        calibration.node_predicted_sq_sum;
    out.bc_stats.node_estimate_realized_sq_sum +=
        calibration.node_realized_sq_sum;
    out.bc_stats.node_estimate_cross_sum += calibration.node_cross_sum;
    out.bc_stats.directional_calibration_samples +=
        calibration.directional_samples;
    out.bc_stats.directional_predicted_gain_sum +=
        calibration.directional_predicted_gain_sum;
    out.bc_stats.directional_realized_gain_sum +=
        calibration.directional_realized_gain_sum;
    out.bc_stats.directional_abs_error_sum +=
        calibration.directional_abs_error_sum;
    out.bc_stats.directional_squared_error_sum +=
        calibration.directional_squared_error_sum;
    out.bc_stats.directional_rank_samples +=
        calibration.directional_rank_samples;
    out.bc_stats.directional_rank_concordant +=
        calibration.directional_rank_concordant;
			    const auto loop_t3 = std::chrono::steady_clock::now();
    first_branch_ipm_solve_slots = 0;

			    // HiGHS treats newly learned conflict/cutpool rows as live
			    // propagation rows.  Once a child publishes one, drive the proof
			    // frontier to the same domain fixed point before leaving stale
			    // queued nodes behind.
			    if (child_domain_source_added && !node_queue.empty()) {
			      (void)prune_queue_after_domain_learning(
			          proof_tail_queue_mode_active(), "child_domain_source");
			    }
		    if (incumbent_changed_this_node) {
		      highs_style_post_incumbent_domain_closure(
	          "child_solution", parent_depth);
	      maybe_run_post_incumbent_local_branching(parent_depth);
	    }

		    if (has_incumbent) {
		      const double child_queue_upper_limit =
		          highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
		                                             opt.int_tol);
		      if (down_valid && incumbent_prunes_node(
		                            std::isfinite(child_queue_upper_limit),
		                            child_queue_upper_limit, child_down.bound,
		                            std::max(1e-9, opt.lp_tol))) down_valid = false;
		      if (up_valid && incumbent_prunes_node(
		                          std::isfinite(child_queue_upper_limit),
		                          child_queue_upper_limit, child_up.bound,
		                          std::max(1e-9, opt.lp_tol))) up_valid = false;
              const double gap_limit = gap_optimality_limit();
              const double gap_limit_tol = gap_optimality_prune_tol(gap_limit);
	              if (!opt.require_tree_exhaustion_certificate &&
	                  std::isfinite(gap_limit)) {
                if (down_valid && std::isfinite(child_down.bound) &&
                    child_down.bound >= gap_limit - gap_limit_tol) {
                  down_valid = false;
                  gap_limit_lower_bound_certificate =
                      std::max(gap_limit_lower_bound_certificate, gap_limit);
                  ++out.bc_stats.gap_suboptimal_queue_prunes;
                }
                if (up_valid && std::isfinite(child_up.bound) &&
                    child_up.bound >= gap_limit - gap_limit_tol) {
                  up_valid = false;
                  gap_limit_lower_bound_certificate =
                      std::max(gap_limit_lower_bound_certificate, gap_limit);
                  ++out.bc_stats.gap_suboptimal_queue_prunes;
                }
              }
		    }
	    if (down_valid && conflict_pool.has_conflict(child_down.lb, child_down.ub)) {
	      down_valid = false;
	      ++out.bc_stats.rc_conflict_queue_prunes;
	    }
	    if (up_valid && conflict_pool.has_conflict(child_up.lb, child_up.ub)) {
	      up_valid = false;
	      ++out.bc_stats.rc_conflict_queue_prunes;
	    }

    replay_child_domain_before_queue(child_down, down_valid,
                                     "down_child_queue");
    replay_child_domain_before_queue(child_up, up_valid, "up_child_queue");
    refresh_child_lp_before_queue(child_down, down_valid,
                                  "down_child_queue_refresh");
    refresh_child_lp_before_queue(child_up, up_valid,
                                  "up_child_queue_refresh");

    if (trace_bc_timeline) {
      fmt::print(stderr,
                 "[B&C-CHILD-FINAL] var={} down_valid={} up_valid={} "
                 "down_bound={:.10g} up_bound={:.10g} inc={:.10g} "
                 "qsize={}\n",
                 j, down_valid ? 1 : 0, up_valid ? 1 : 0,
                 child_down.bound, child_up.bound,
                 has_incumbent ? incumbent_obj : kInf, node_queue.size());
    }

    if (down_valid) {
      child_down.estimate = compute_node_estimate(child_down.fractional_branchables, child_down.x_relax,
                                                  pc, opt.int_tol, child_down.bound,
                                                  opt.node_estimate_aggregation);
    }
    if (up_valid) {
      child_up.estimate = compute_node_estimate(child_up.fractional_branchables, child_up.x_relax,
                                                pc, opt.int_tol, child_up.bound,
                                                opt.node_estimate_aggregation);
    }

    // The next branch reconstructs a seed from x_relax. Do not retain a second
    // full primal vector in the live-node frontier.
    if (down_valid) child_down.x_seed.resize(0);
    if (up_valid) child_up.x_seed.resize(0);

	    if (down_valid && up_valid) {
	      const bool push_proof_queue_mode = hybrid_queue_mode_active();
	      const bool proof_tail_queue_mode = proof_tail_queue_mode_active();
      if (proof_tail_queue_mode) {
	        node_queue.push(std::move(child_down), true);
	        node_queue.push(std::move(child_up), true);
      } else if (!prefer_up) {
        node_queue.push(std::move(child_up), push_proof_queue_mode);
        node_queue.push_dfs(std::move(child_down));
      } else {
        node_queue.push(std::move(child_down), push_proof_queue_mode);
        node_queue.push_dfs(std::move(child_up));
      }
	    } else if (down_valid) {
	      if (proof_tail_queue_mode_active()) {
	        node_queue.push(std::move(child_down), true);
	      } else {
        node_queue.push_dfs(std::move(child_down));
      }
	    } else if (up_valid) {
	      if (proof_tail_queue_mode_active()) {
	        node_queue.push(std::move(child_up), true);
	      } else {
        node_queue.push_dfs(std::move(child_up));
      }
    }

    if (ipm_node_deadline_hit || simplex_node_deadline_hit) {
      out.stats.status = bc_status::kTimeLimitReached;
      break;
    }

    // Crossover heuristic: every 100 nodes.
    if (run_crossover_this_node) {
      const auto& s0 = solution_pool[0];
      const auto& s1 = solution_pool[1];
      Eigen::VectorXd xc = s0.x;
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        xc[i] = std::round(s0.x[i]);
      }
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        if (!is_integral(cur.x_relax[i], opt.int_tol)) {
          xc[i] = std::round(s1.x[i]);
        }
      }
      for (int i = 0; i < n; ++i) {
        xc[i] = std::min(cur.ub[i], std::max(cur.lb[i], xc[i]));
      }
	      if (satisfies_with_bounds(base_lp, xc, cur.lb, cur.ub, 1e-6)) {
	        const double obj = objective_value(base_lp.c, xc, base_lp.sense);
	        if (adopt_tree_incumbent("crossover", xc, obj, cur.depth,
	                                 global_lb)) {
	          ++out.bc_stats.crossover_incumbents;
		          highs_style_post_incumbent_domain_closure("crossover",
		                                                    cur.depth);
	          maybe_run_post_incumbent_local_branching(cur.depth);
	        }
      }
    }

    // Cheap RINS-like rounded-point check. This path does not solve a RINS
    // sub-MIP; it only rounds variables using LP/incumbent agreement.
    if (run_rins_this_node) {
      Eigen::VectorXd xr = cur.x_relax;
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        const double lp_rounded = std::round(cur.x_relax[i]);
        const double inc_rounded = std::round(incumbent_x[i]);
        // Where LP and incumbent agree, fix to that value.
        if (std::abs(lp_rounded - inc_rounded) < 0.5) {
          xr[i] = inc_rounded;
        } else {
          // Disagree: round LP towards better objective (try LP value first).
          xr[i] = lp_rounded;
        }
        xr[i] = std::min(cur.ub[i], std::max(cur.lb[i], xr[i]));
      }
	      if (satisfies_with_bounds(base_lp, xr, cur.lb, cur.ub, 1e-6)) {
	        const double obj = objective_value(base_lp.c, xr, base_lp.sense);
	        if (adopt_tree_incumbent("rins_rounding", xr, obj, cur.depth,
	                                 global_lb)) {
		          highs_style_post_incumbent_domain_closure("rins_rounding",
		                                                    cur.depth);
	          maybe_run_post_incumbent_local_branching(cur.depth);
	        }
      }
    }

    refresh_queue_lower_bound();
    {
      const auto loop_t4 = std::chrono::steady_clock::now();
      loop_timing.pop_us += loop_us(loop_t0, loop_t1);
      loop_timing.branch_us += loop_us(loop_t1, loop_t2);
      loop_timing.child_us += loop_us(loop_t2, loop_t3);
      loop_timing.post_us += loop_us(loop_t3, loop_t4);
      ++loop_timing.count;
      loop_timing.dump(opt.verbose);
    }

    if (has_incumbent && std::isfinite(global_lb)) {
      const double denom = std::max(1.0, std::abs(incumbent_obj));
      double rel_gap = std::max(0.0, (incumbent_obj - global_lb) / denom);
      out.bc_stats.gap = rel_gap;
      if (opt.verbose && out.bc_stats.nodes_explored % 500 == 0) {
        double tree_elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        fprintf(stderr, "[TREE-PROGRESS] N=%d lb=%.2f ub=%.2f gap=%.2e time=%.0fms\n",
                static_cast<int>(out.bc_stats.nodes_explored), global_lb,
                incumbent_obj, rel_gap, tree_elapsed);
      }
      if (root_primal_polish_complete && root_gap_certificate_ready &&
          !opt.require_tree_exhaustion_certificate &&
          rel_gap <= opt.gap_tol) {
        out.stats.status = bc_status::kOptimalityGapReached;
        break;
      }
      // Gap stagnation detection: if gap hasn't improved enough over the
      // last stall_node_window nodes, stop early.  This prevents wasting
      // time when the bound/incumbent plateau (e.g. gap stuck at 1.3e-4
      // for 2000+ nodes).
      // BUG FIX: When the gap is already small (< 10× gap_tol), the
      // relative improvement metric is too strict — even tiny absolute
      // progress is invisible.  Use a larger effective window to give the
      // tree more time when the gap is within striking distance of gap_tol.
      if (opt.stall_node_window > 0) {
        // Widen the stagnation window modestly when the gap is near the target
        // (2× instead of 4× — the old 4× multiplier could waste 20+ seconds
        // of tree search on problems where the bound/incumbent are truly frozen).
        const int effective_window = large_generic_low_frac_root
            ? opt.stall_node_window
            : ((rel_gap < 10.0 * opt.gap_tol)
                   ? opt.stall_node_window * 2
                   : opt.stall_node_window);
        const int nodes_now = out.bc_stats.nodes_explored;
        if (nodes_now >= stall_checkpoint_node + effective_window) {
          if (stall_gap_checkpoint < 1e29) {
            // Check relative improvement: (old_gap - new_gap) / old_gap
            const double improvement = (stall_gap_checkpoint - rel_gap) /
                                       std::max(1e-15, stall_gap_checkpoint);
            if (improvement < opt.stall_gap_min_improvement) {
              if (opt.verbose) {
                fprintf(stderr, "[B&C] gap stagnation: gap=%.2e improved only %.1f%% "
                        "over %d nodes (threshold %.0f%%), stopping\n",
                        rel_gap, 100.0 * improvement, opt.stall_node_window,
                        100.0 * opt.stall_gap_min_improvement);
              }
              out.stats.status = bc_status::kGapStagnation;
              break;
            }
          }
          stall_gap_checkpoint = rel_gap;
          stall_checkpoint_node = nodes_now;
        }
      }
    }
  }
  // ── END SEQUENTIAL PATH ──────────────────────────────────────────────────
  } else {
  // ── HETEROGENEOUS PARALLEL PATH ──────────────────────────────────────────
  // For concurrent mode (par_mode), tree threads were launched before
  // root heuristics.  CTS (ConcurrentTreeState) holds all shared state.
  // Push any final heuristic incumbent to CTS, then enter monitor loop.
  //
  // For non-concurrent parallel (shouldn't happen, but as a safety fallback),
  // create CTS here and launch threads.
  if (!concurrent_tree_launched) {
    // Late launch: create CTS and launch threads (same logic as early launch).
    const int num_threads_late = std::max(1, resolve_num_threads(opt));
	    cts = std::make_unique<ConcurrentTreeState>(opt, t0, n);
    for (int col : declared_branchable_indices) cts->pseudocosts[col] = pc[col];
    cts->shared_queue.configure_domain_signature(
        declared_branchable_cols, root.lb, root.ub, std::max(1e-9, opt.int_tol));
	    if (out.bc_stats.cuts_added > 0) {
      const int orig_rows = static_cast<int>(pre_cut_root_lp.A.rows());
      const int total_rows = static_cast<int>(base_lp.A.rows());
      Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_late = base_lp.A;
      for (int r = orig_rows; r < total_rows; ++r) {
        Eigen::SparseVector<double> row_sparse(n);
        int nnz = 0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_late, r); it; ++it)
          if (it.col() < n) ++nnz;
        row_sparse.reserve(nnz);
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_late, r); it; ++it)
          if (it.col() < n) row_sparse.insertBack(it.col()) = it.value();
        cts->shared_cp.seed(std::move(row_sparse), base_lp.b[r]);
      }
    }
    if (has_incumbent) {
      cts->shared_inc.try_update(incumbent_x, incumbent_obj);
      cts->shared_sp.seed(incumbent_x, incumbent_obj);
    }
    cts->shared_queue.push(root, has_incumbent);
    cts->optimality_limit.store(
        root_gap_certificate_ready && !opt.require_tree_exhaustion_certificate
            ? gap_optimality_limit()
            : kInf,
        std::memory_order_release);
    cts->par_simplex_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
    cts->par_simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    cts->par_simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    cts->par_simplex_opt.allow_cold_start = false;
    // Parallel node LPs are proof-bearing.  Let the B&C layer prune against
    // the shared incumbent after a complete LP solve; an in-kernel objective
    // cutoff only exposes an intermediate objective, not a certified bound.
    cts->par_simplex_opt.incumbent_bound = nullptr;
    cts->par_simplex_opt.incumbent_check_interval = 0;
    cts->par_simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
    cts->par_simplex_opt.factor_backend =
        simplex_factor_backend_from_id(opt.simplex_factor_backend);
    apply_bc_first_class_simplex_state(cts->par_simplex_opt,
                                       /*allow_frontier_remap=*/true);
    cts->use_cut_worker = (opt.cuts != CutType::None && opt.max_cut_depth > 0);
    const int n_exp = cts->use_cut_worker ? num_threads_late - 1 : num_threads_late;
    cts->n_explorers = n_exp;
    out.bc_stats.parallel_effective_threads = num_threads_late;
    out.bc_stats.parallel_explorer_threads = n_exp;
    out.bc_stats.parallel_tree_launched = true;
    out.bc_stats.parallel_schedule_reason = "late_tree";
    const int par_base_m = static_cast<int>(base_lp.A.rows()) +
                           static_cast<int>(base_lp.Aeq.rows());
    const bool use_hybrid_ipm = (n_exp >= 2 && par_base_m > opt.hybrid_ipm_threshold);
    if (opt.enable_work_stealing && n_exp >= 2)
      cts->steal_pool = std::make_unique<WorkStealingPool>(
          n_exp, opt.work_steal_deque_size, opt.random_seed);
    if (opt.deterministic_parallel && n_exp >= 2)
      cts->det_token = std::make_unique<DeterministicTurnToken>(n_exp);
    cts->explorers.reserve(n_exp);
    for (int tid = 0; tid < n_exp; ++tid) {
      WorkerRole role = WorkerRole::Diver;
      if (use_hybrid_ipm && tid == 0) role = WorkerRole::IPMDiver;
      else if (tid == n_exp - 1 && n_exp >= 2) role = WorkerRole::Prover;
      cts->explorers.emplace_back(explorer_thread,
          solve_environment, tid, role,
          std::ref(cts->shared_queue), std::ref(cts->cut_req_queue),
          std::ref(cts->shared_cp), std::ref(cts->shared_conflicts),
	          std::ref(cts->shared_sp), std::ref(cts->shared_inc),
	          std::cref(base_lp), std::cref(pre_cut_root_lp), std::cref(ensure_base_sf()),
	          std::cref(declared_branchable_cols),
	          std::cref(parallel_branch_priority),
	          std::cref(branch_original_cols), orig_n,
	          callbacks && callbacks->branching_prior
	              ? &callbacks->branching_prior
	              : nullptr,
	          std::cref(clique_table), std::cref(implication_graph),
          std::cref(cts->par_simplex_opt), std::cref(opt),
	          std::ref(cts->pseudocosts), std::ref(cts->pc_mtx),
          std::ref(cts->atomic_stats), std::ref(cts->should_stop),
          std::ref(cts->progress_event),
          std::ref(cts->active_explorers), std::ref(cts->active_bounds),
          std::ref(cts->fallback_logger),
          root.basis_hint, cts->steal_pool.get(),
          &cts->shared_impl_graph, cts->det_token.get(),
          &cts->optimality_limit);
    }
    if (cts->use_cut_worker) {
      cts->cut_worker = std::thread(cut_worker_thread,
          solve_environment, std::ref(cts->cut_req_queue),
          std::ref(cts->shared_cp),
          std::ref(cts->shared_sp), std::ref(cts->shared_inc),
          std::cref(base_lp), std::cref(opt),
          std::ref(cts->atomic_stats), std::ref(cts->should_stop));
    }
    concurrent_tree_launched = true;
  } else {
    // Early-launched: push final heuristic incumbent to shared state.
    if (has_incumbent) {
      cts->shared_inc.try_update(incumbent_x, incumbent_obj);
    }
  }

  // ── Monitor loop (shared by early and late launch) ──
  double par_stall_gap_checkpoint = 1e30;
  int    par_stall_checkpoint_node = 0;
  double par_certified_bound_at_stop = kInf;
  double par_gap_at_stop = kInf;
  const bool trace_parallel_tail =
      trace_bc_timeline || bc_env_flag_enabled("MIPSOLVERS_BC_PAR_TRACE");
  std::uint64_t par_progress_generation = cts->progress_event.generation();

  while (!cts->should_stop.load(std::memory_order_relaxed)) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - t0).count();
    const int nodes = cts->atomic_stats.nodes_explored.load(std::memory_order_relaxed);

    if (elapsed > opt.time_limit_sec) {
      out.stats.status = bc_status::kTimeLimitReached;
      cts->should_stop.store(true, std::memory_order_release);
      break;
    }
    if (nodes >= opt.max_nodes) {
      out.stats.status = bc_status::kNodeLimitReached;
      cts->should_stop.store(true, std::memory_order_release);
      break;
    }

    if (cts->shared_inc.has()) {
      const double inc_obj = cts->shared_inc.get_obj();
      const double par_opt_limit =
          root_gap_certificate_ready && !opt.require_tree_exhaustion_certificate &&
                  std::isfinite(opt.gap_tol) && opt.gap_tol >= 0.0
              ? inc_obj - opt.gap_tol * std::max(1.0, std::abs(inc_obj))
              : kInf;
      cts->optimality_limit.store(par_opt_limit, std::memory_order_release);
      double live_lb = cts->shared_queue.lower_bound();
      if (cts->steal_pool) {
        live_lb = std::min(live_lb, cts->steal_pool->lower_bound());
      }
      live_lb = std::min(live_lb, cts->active_bounds.lower_bound());
      if (std::isfinite(live_lb) && std::isfinite(inc_obj)) {
        const double denom = std::max(1.0, std::abs(inc_obj));
        const double rel_gap = std::max(0.0, (inc_obj - live_lb) / denom);
        // Exact optimality (mirrors the sequential skip_tree path): a certified
        // dual bound meeting the incumbent proves optimality independent of
        // require_tree_exhaustion_certificate.
        const bool proven_optimal_exact =
            root_gap_certificate_ready && rel_gap <= 1e-9;
        if (proven_optimal_exact ||
            (root_gap_certificate_ready &&
             !opt.require_tree_exhaustion_certificate &&
             rel_gap <= opt.gap_tol)) {
          par_certified_bound_at_stop = live_lb;
          par_gap_at_stop = rel_gap;
          out.stats.status = bc_status::kOptimalityGapReached;
          cts->should_stop.store(true, std::memory_order_release);
          break;
        }
        // Gap stagnation detection (parallel). 2× widening near target
        // (was 4× — too patient, wasted real wall-clock time).
        const int par_effective_window = large_generic_low_frac_root
            ? opt.stall_node_window
            : ((rel_gap < 10.0 * opt.gap_tol)
                   ? opt.stall_node_window * 2
                   : opt.stall_node_window);
        if (opt.stall_node_window > 0 &&
            nodes >= par_stall_checkpoint_node + par_effective_window) {
          if (par_stall_gap_checkpoint < 1e29) {
            const double improvement = (par_stall_gap_checkpoint - rel_gap) /
                                       std::max(1e-15, par_stall_gap_checkpoint);
            if (improvement < opt.stall_gap_min_improvement) {
              out.stats.status = bc_status::kGapStagnation;
              cts->should_stop.store(true, std::memory_order_release);
              break;
            }
          }
          par_stall_gap_checkpoint = rel_gap;
          par_stall_checkpoint_node = nodes;
        }
      }
    }

    const std::uint64_t empty_check_generation =
        cts->progress_event.generation();
    const int deque_nodes = cts->steal_pool ? cts->steal_pool->total_size() : 0;
    if (cts->shared_queue.empty() && deque_nodes == 0 &&
        cts->active_explorers.load(std::memory_order_acquire) == 0) {
      const int deque_nodes2 = cts->steal_pool ? cts->steal_pool->total_size() : 0;
      const bool queue_still_empty = cts->shared_queue.empty();
      const bool explorers_still_idle =
          cts->active_explorers.load(std::memory_order_acquire) == 0;
      const bool no_intervening_progress =
          cts->progress_event.generation() == empty_check_generation;
      if (queue_still_empty && deque_nodes2 == 0 && explorers_still_idle &&
          no_intervening_progress) {
        out.stats.status = bc_status::kSearchQueueExhausted;
        cts->should_stop.store(true, std::memory_order_release);
        if (trace_parallel_tail) {
          fmt::print(stderr,
                     "[B&C-PAR-STOP] reason=queue_exhausted nodes={} q={} "
                     "deques={} active={} qlb={:.10g} dlb={:.10g} "
                     "alb={:.10g} inc={:.10g}\n",
                     nodes, cts->shared_queue.size(), deque_nodes2,
                     cts->active_explorers.load(std::memory_order_acquire),
                     cts->shared_queue.lower_bound(),
                     cts->steal_pool ? cts->steal_pool->lower_bound() : kInf,
                     cts->active_bounds.lower_bound(),
                     cts->shared_inc.has() ? cts->shared_inc.get_obj() : kInf);
        }
        break;
      }
    }

    if (opt.verbose && nodes > 0 && nodes % 500 < 10) {
      const double inc_obj = cts->shared_inc.get_obj();
      std::string inc_msg = cts->shared_inc.has() ? fmt::format(" obj={:.2f}", inc_obj) : "";
      MIPSOLVERS_LOG_DEBUG("[B&C-PAR] nodes={} q={} inc={}{} cuts_req={}/{} cw_cuts={}",
              nodes, cts->shared_queue.size(),
              cts->shared_inc.has() ? "yes" : "no", inc_msg,
              cts->atomic_stats.cut_requests_submitted.load(std::memory_order_relaxed),
              cts->atomic_stats.cut_requests_dropped.load(std::memory_order_relaxed),
              cts->atomic_stats.cut_worker_cuts.load(std::memory_order_relaxed));
    }

    auto wake_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    if (std::isfinite(opt.time_limit_sec)) {
      const auto time_deadline =
          t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(
                       std::max(0.0, opt.time_limit_sec)));
      wake_deadline = std::min(wake_deadline, time_deadline);
    }
    (void)cts->progress_event.wait_until_change(par_progress_generation,
                                                 wake_deadline);
  }

  // Shutdown and join threads.
  if (trace_parallel_tail) {
    fmt::print(stderr,
               "[B&C-PAR-BEFORE-JOIN] status='{}' should_stop={} nodes={} "
               "q={} qlb={:.10g} deques={} dlb={:.10g} active={} "
               "alb={:.10g} inc={:.10g}\n",
               out.stats.status,
               cts->should_stop.load(std::memory_order_acquire) ? 1 : 0,
               cts->atomic_stats.nodes_explored.load(),
               cts->shared_queue.size(), cts->shared_queue.lower_bound(),
               cts->steal_pool ? cts->steal_pool->total_size() : 0,
               cts->steal_pool ? cts->steal_pool->lower_bound() : kInf,
               cts->active_explorers.load(std::memory_order_acquire),
               cts->active_bounds.lower_bound(),
               cts->shared_inc.has() ? cts->shared_inc.get_obj() : kInf);
  }
  cts->shutdown_and_join();

  out.bc_stats.nodes_explored = cts->atomic_stats.nodes_explored.load();
  out.bc_stats.lp_solves += cts->atomic_stats.lp_solves.load();
  out.bc_stats.cuts_added += cts->atomic_stats.cuts_added.load();
  out.bc_stats.pool_cuts_separated += cts->atomic_stats.pool_cuts_separated.load();
  out.bc_stats.crossover_incumbents += cts->atomic_stats.crossover_incumbents.load();
  out.bc_stats.cut_requests_submitted = cts->atomic_stats.cut_requests_submitted.load();
  out.bc_stats.cut_requests_dropped = cts->atomic_stats.cut_requests_dropped.load();
  out.bc_stats.cut_worker_cuts = cts->atomic_stats.cut_worker_cuts.load();
  out.bc_stats.cut_worker_incumbents = cts->atomic_stats.cut_worker_incumbents.load();
  out.bc_stats.rc_fixings += cts->atomic_stats.rc_fixings.load();
  out.bc_stats.rc_forbidden_literals += cts->atomic_stats.rc_forbidden_literals.load();
  out.bc_stats.rc_conflict_clauses_learned +=
      cts->atomic_stats.rc_conflict_clauses_learned.load();
  out.bc_stats.rc_binary_implications_learned +=
      cts->atomic_stats.rc_binary_implications_learned.load();
  out.bc_stats.rc_conflict_cuts_added +=
      cts->atomic_stats.rc_conflict_cuts_added.load();
  out.bc_stats.rc_proof_conflict_clauses_learned +=
      cts->atomic_stats.rc_proof_conflict_clauses_learned.load();
  out.bc_stats.rc_proof_conflict_literals_before +=
      cts->atomic_stats.rc_proof_conflict_literals_before.load();
  out.bc_stats.rc_proof_conflict_literals_after +=
      cts->atomic_stats.rc_proof_conflict_literals_after.load();
  out.bc_stats.rc_conflict_verification_lps +=
      cts->atomic_stats.rc_conflict_verification_lps.load();
  out.bc_stats.rc_verified_conflict_clauses_learned +=
      cts->atomic_stats.rc_verified_conflict_clauses_learned.load();
  out.bc_stats.gap_suboptimal_queue_prune_passes +=
      cts->atomic_stats.gap_suboptimal_queue_prune_passes.load();
  out.bc_stats.gap_suboptimal_queue_prunes +=
      cts->atomic_stats.gap_suboptimal_queue_prunes.load();
  out.bc_stats.branch_direction_preferred_down +=
      cts->atomic_stats.branch_direction_preferred_down.load();
  out.bc_stats.branch_direction_preferred_up +=
      cts->atomic_stats.branch_direction_preferred_up.load();
  out.bc_stats.branch_direction_first_down +=
      cts->atomic_stats.branch_direction_first_down.load();
  out.bc_stats.branch_direction_first_up +=
      cts->atomic_stats.branch_direction_first_up.load();
  out.bc_stats.branch_first_child_incumbent_updates +=
      cts->atomic_stats.branch_first_child_incumbent_updates.load();
  out.bc_stats.branch_first_child_cutoffs +=
      cts->atomic_stats.branch_first_child_cutoffs.load();
  out.bc_stats.branch_second_child_cutoffs +=
      cts->atomic_stats.branch_second_child_cutoffs.load();
  out.bc_stats.node_estimate_calibration_samples +=
      cts->atomic_stats.node_estimate_calibration_samples.load();
  out.bc_stats.node_estimate_predicted_lift_sum +=
      cts->atomic_stats.node_estimate_predicted_lift_sum.load();
  out.bc_stats.node_estimate_realized_lift_sum +=
      cts->atomic_stats.node_estimate_realized_lift_sum.load();
  out.bc_stats.node_estimate_abs_error_sum +=
      cts->atomic_stats.node_estimate_abs_error_sum.load();
  out.bc_stats.node_estimate_squared_error_sum +=
      cts->atomic_stats.node_estimate_squared_error_sum.load();
  out.bc_stats.node_estimate_predicted_sq_sum +=
      cts->atomic_stats.node_estimate_predicted_sq_sum.load();
  out.bc_stats.node_estimate_realized_sq_sum +=
      cts->atomic_stats.node_estimate_realized_sq_sum.load();
  out.bc_stats.node_estimate_cross_sum +=
      cts->atomic_stats.node_estimate_cross_sum.load();
  out.bc_stats.directional_calibration_samples +=
      cts->atomic_stats.directional_calibration_samples.load();
  out.bc_stats.directional_predicted_gain_sum +=
      cts->atomic_stats.directional_predicted_gain_sum.load();
  out.bc_stats.directional_realized_gain_sum +=
      cts->atomic_stats.directional_realized_gain_sum.load();
  out.bc_stats.directional_abs_error_sum +=
      cts->atomic_stats.directional_abs_error_sum.load();
  out.bc_stats.directional_squared_error_sum +=
      cts->atomic_stats.directional_squared_error_sum.load();
  out.bc_stats.directional_rank_samples +=
      cts->atomic_stats.directional_rank_samples.load();
  out.bc_stats.directional_rank_concordant +=
      cts->atomic_stats.directional_rank_concordant.load();

  if (cts->steal_pool) {
    out.bc_stats.work_steals = cts->steal_pool->get_total_steals();
    out.bc_stats.steal_attempts = cts->steal_pool->get_total_steals() + cts->steal_pool->get_failed_steals();
  }

		  if (cts->shared_inc.has()) {
		    Eigen::VectorXd shared_x = cts->shared_inc.get_x();
		    const double shared_actual_obj =
		        objective_value(base_lp.c, shared_x, base_lp.sense);
		    const bool parallel_improved =
		        std::isfinite(shared_actual_obj) &&
		        (!has_incumbent || shared_actual_obj + 1e-9 < incumbent_obj);
			    if (parallel_improved &&
			        validate_root_candidate_for_proof("parallel_shared_incumbent",
			                                          shared_x)) {
			      has_incumbent = true;
			      incumbent_obj = shared_actual_obj;
			      incumbent_x = std::move(shared_x);
			      note_incumbent_cutoff_learning();
			      incumbent_proof_validated = true;
		      incumbent_proof_validated_obj = incumbent_obj;
		      note_incumbent_update(/*depth=*/-1, global_lb);
		    } else if (!has_incumbent) {
		      invalidate_incumbent_proof_validation();
		    }
		  }

  const double live_lb = cts->shared_queue.lower_bound();
  const double deque_lb = cts->steal_pool ? cts->steal_pool->lower_bound() : kInf;
  const double active_lb = cts->active_bounds.lower_bound();
  const double combined_lb = std::min(live_lb, std::min(deque_lb, active_lb));
  const auto par_fb_sum = cts->fallback_logger.summary();
  if (std::isfinite(par_certified_bound_at_stop)) {
    global_lb = par_certified_bound_at_stop;
  } else if (par_fb_sum.nodes_pruned_unsafe > 0) {
    global_lb = root.bound;
  } else if (out.stats.status == bc_status::kSearchQueueExhausted) {
    global_lb = has_incumbent ? incumbent_obj : root.bound;
  } else if (std::isfinite(combined_lb)) {
    global_lb = combined_lb;
  } else if (has_incumbent) {
    global_lb = incumbent_obj;
  }
  out.bc_stats.best_bound = global_lb;

  if (std::isfinite(par_gap_at_stop)) {
    out.bc_stats.gap = par_gap_at_stop;
  } else if (has_incumbent && std::isfinite(global_lb)) {
    const double denom = std::max(1.0, std::abs(incumbent_obj));
    out.bc_stats.gap = std::max(0.0, (incumbent_obj - global_lb) / denom);
  }
  // ── END HETEROGENEOUS PARALLEL PATH ──────────────────────────────────────

  // Merge parallel fallback stats.
  {
    auto fb_sum = cts->fallback_logger.summary();
    out.bc_stats.fallback_events += fb_sum.total_events;
    out.bc_stats.fallback_recoveries += fb_sum.nodes_recovered;
    out.bc_stats.fallback_deferrals += fb_sum.nodes_deferred;
    out.bc_stats.fallback_prunes += fb_sum.nodes_pruned;
    out.bc_stats.fallback_infeasible_prunes += fb_sum.nodes_pruned_infeasible;
    out.bc_stats.fallback_unbounded_prunes += fb_sum.nodes_pruned_unbounded;
    out.bc_stats.fallback_unsafe_prunes += fb_sum.nodes_pruned_unsafe;
    out.bc_stats.fallback_l1_success += fb_sum.l1_successes;
    out.bc_stats.fallback_l2_success += fb_sum.l2_successes;
    out.bc_stats.fallback_l3_success += fb_sum.l3_successes;
  }

  }

  const auto t1 = std::chrono::steady_clock::now();
  const double runtime = std::chrono::duration<double>(t1 - t0).count();
  note_bound_event(has_incumbent ? incumbent_obj : kInf, global_lb,
                   /*retain_incumbent=*/has_incumbent);

  sync_propagation_profile();

  if (out.bc_stats.last_incumbent_node >= 0) {
    out.bc_stats.nodes_after_last_incumbent =
        std::max(0, out.bc_stats.nodes_explored -
                        out.bc_stats.last_incumbent_node);
    out.bc_stats.lp_solves_after_last_incumbent =
        std::max(0, out.bc_stats.lp_solves -
                        out.bc_stats.last_incumbent_lp_solves);
    if (std::isfinite(out.bc_stats.best_bound) &&
        std::isfinite(out.bc_stats.best_bound_at_last_incumbent)) {
      out.bc_stats.best_bound_lift_after_last_incumbent =
          std::max(0.0, out.bc_stats.best_bound -
                            out.bc_stats.best_bound_at_last_incumbent);
    }
  }

  if (opt.verbose) {
    fmt::print(stderr, "[B&C] done: nodes={} lp_solves={} cuts={} pool_sep={} crossover={} time={:.2f}s status={}\n",
            out.bc_stats.nodes_explored, out.bc_stats.lp_solves,
            out.bc_stats.cuts_added, out.bc_stats.pool_cuts_separated,
            out.bc_stats.crossover_incumbents, runtime, out.stats.status);
    dump_solve_lp_counters();
    MIPSOLVERS_LOG_DEBUG("[B&C] done: nodes={} lp_solves={} cuts={} pool_sep={} crossover={} time={:.2f}s status={}",
            out.bc_stats.nodes_explored, out.bc_stats.lp_solves,
            out.bc_stats.cuts_added, out.bc_stats.pool_cuts_separated,
            out.bc_stats.crossover_incumbents, runtime, out.stats.status);
    fmt::print(stderr,
               "[B&C-DYNAMIC] rc_fix={} rc_lits={} rc_clauses={} rc_impl={} "
               "rc_cuts={} rc_proof={} rc_proof_len={}/{} "
	               "rc_verify_lps={} rc_verified={} rc_resolve_lps={} "
	               "rc_resolve_up={} rc_resolve_prunes={} dynIB={}/{}/{}/{} "
	               "objClq={}/{}/{} mixRows={} maxDel={:.3g} "
	               "objEvt={}/{}/{} "
	               "objSrc=vlb/vub:{}/{} pub={}/{} round={} mir={}/{} "
	               "mir_gain_avg/max={:.3g}/{:.3g} evtMax={:.3g}/{:.3g} "
	               "objDomLb={}/{:.3g}/{:.3g} "
	               "dynIB_active={} dynIB_skip={}/{}/{} late_skip={} "
	               "locImp={}/{}/{} locCl={}/{}/{} scoped={}/{}/{}/{} "
	               "cutoff_tight={} cutoff_prunes={} "
	               "cutoff_clauses={} cutoff_cuts={} "
	               "certQ={}/{}/{} lift_avg/max={:.3g}/{:.3g} "
	               "certMove=count{} sum/max={:.3g}/{:.3g} "
	               "lb_nodes={} lp_refresh={} certPub/Rej={}/{} "
	               "audit_margin=min/max={:.3g}/{:.3g} "
                 "resTarget={}/{}/{} resConflict={}/{}/{} "
                 "resFail=activity/missing/scope {}/{}/{} "
                 "qlbAudit={}/{} last={:.10g}/{:.10g} maxErr={:.3g} "
	               "nodes={}/{} "
	               "certD={}/{}/{}/{} termEff={}/{}/{} "
	               "xfront=att{} tab{} path{} viol/admit/rej={}/{}/{} "
	               "coeff/viol={:.3g}/{:.3g} xres={}/{} avg/max={:.3g}/{:.3g} "
	               "proof_diag_n={} budget_avg/max={:.3g}/{:.3g} "
	               "activity_avg/max={:.3g}/{:.3g} viol_avg/max={:.3g}/{:.3g} "
	               "resolve_lift_n={} avg/max={:.3g}/{:.3g}\n",
               out.bc_stats.rc_fixings,
               out.bc_stats.rc_forbidden_literals,
               out.bc_stats.rc_conflict_clauses_learned,
               out.bc_stats.rc_binary_implications_learned,
               out.bc_stats.rc_conflict_cuts_added,
               out.bc_stats.rc_proof_conflict_clauses_learned,
               out.bc_stats.rc_proof_conflict_literals_before,
               out.bc_stats.rc_proof_conflict_literals_after,
               out.bc_stats.rc_conflict_verification_lps,
               out.bc_stats.rc_verified_conflict_clauses_learned,
               out.bc_stats.rc_fixing_resolve_lps,
               out.bc_stats.rc_fixing_resolve_bound_improvements,
               out.bc_stats.rc_fixing_resolve_prunes,
               out.bc_stats.dynamic_implied_bound_rows_generated,
               out.bc_stats.dynamic_implied_bound_rows_violated,
               out.bc_stats.dynamic_implied_bound_rows_added,
               out.bc_stats.dynamic_implied_bound_rows_rejected,
               out.bc_stats.objective_clique_partitions,
               out.bc_stats.objective_clique_terms,
               out.bc_stats.objective_clique_edges,
               out.bc_stats.objective_clique_mixed_rows,
               out.bc_stats.objective_clique_max_replacement_delta,
               out.bc_stats.objective_implied_events,
               out.bc_stats.objective_implied_event_tightenings,
               out.bc_stats.objective_implied_event_prunes,
               out.bc_stats.objective_implied_event_vlb_events,
               out.bc_stats.objective_implied_event_vub_events,
               out.bc_stats.objective_implied_event_published_implications,
               out.bc_stats.objective_implied_event_published_clique_edges,
               out.bc_stats.objective_implied_event_integral_rounds,
               out.bc_stats.objective_implied_event_mir_attempts,
               out.bc_stats.objective_implied_event_mir_strengthened,
               out.bc_stats.objective_implied_event_mir_strengthened > 0
                   ? out.bc_stats.objective_implied_event_mir_gain_sum /
                         static_cast<double>(
                             out.bc_stats
                                 .objective_implied_event_mir_strengthened)
                   : 0.0,
               out.bc_stats.objective_implied_event_mir_gain_max,
               out.bc_stats.objective_implied_event_max_delta,
               out.bc_stats.objective_implied_event_max_aggregate_delta,
               out.bc_stats.objective_domain_bound_lift_nodes,
               out.bc_stats.objective_domain_bound_lift_sum,
               out.bc_stats.objective_domain_bound_lift_max,
               out.bc_stats.dynamic_implied_bound_rows_active_implication,
               out.bc_stats.dynamic_implied_bound_rows_skip_not_active,
               out.bc_stats.dynamic_implied_bound_rows_skip_small_move,
               out.bc_stats.dynamic_implied_bound_rows_skip_no_violation,
               out.bc_stats.late_replay_skipped_no_new_learning,
               out.bc_stats.local_implications_learned,
               out.bc_stats.local_implications_applied,
               out.bc_stats.local_implication_prunes,
               out.bc_stats.local_conflict_clauses_learned,
               out.bc_stats.local_conflict_clause_tightenings,
               out.bc_stats.local_conflict_clause_prunes,
               out.bc_stats.scoped_conflict_objects_learned,
               out.bc_stats.scoped_conflict_objects_active,
               out.bc_stats.scoped_conflict_tightenings,
               out.bc_stats.scoped_conflict_prunes,
	               out.bc_stats.cutoff_domain_tightenings,
	               out.bc_stats.cutoff_domain_prunes,
	               out.bc_stats.cutoff_conflict_clauses_learned,
	               out.bc_stats.cutoff_conflict_cuts_added,
	               out.bc_stats.certificate_queue_passes,
	               out.bc_stats.certificate_queue_tightenings,
	               out.bc_stats.certificate_queue_prunes,
	               out.bc_stats.certificate_queue_tightenings > 0
	                   ? out.bc_stats.certificate_queue_lift_sum /
	                         static_cast<double>(
	                             out.bc_stats.certificate_queue_tightenings)
	                   : 0.0,
	               out.bc_stats.certificate_queue_lift_max,
	               out.bc_stats.certificate_queue_frontier_bound_moves,
	               out.bc_stats.certificate_queue_frontier_bound_move_sum,
	               out.bc_stats.certificate_queue_frontier_bound_move_max,
	               out.bc_stats.certificate_queue_lower_bound_lift_nodes,
	               out.bc_stats.certificate_queue_lp_refresh_marked,
	               out.bc_stats.bound_lifting_certificates_published,
	               out.bc_stats.bound_lifting_certificates_rejected_audit,
	               std::isfinite(
	                   out.bc_stats.bound_lifting_certificate_activity_margin_min)
	                   ? out.bc_stats
	                         .bound_lifting_certificate_activity_margin_min
	                   : 0.0,
	               out.bc_stats.bound_lifting_certificate_activity_margin_max,
                 out.bc_stats.resolved_target_attempts,
                 out.bc_stats.resolved_target_success,
                 out.bc_stats.resolved_target_failed,
                 out.bc_stats.resolved_conflict_attempts,
                 out.bc_stats.resolved_conflict_success,
                 out.bc_stats.resolved_conflict_failed,
                 out.bc_stats.resolution_activity_failed,
                 out.bc_stats.resolution_missing_reason,
                 out.bc_stats.resolution_scope_blocked,
                 out.bc_stats.certificate_queue_lb_audit_passes,
                 out.bc_stats.certificate_queue_lb_audit_failures,
                 std::isfinite(out.bc_stats.certificate_queue_lb_audit_indexed_min)
                     ? out.bc_stats.certificate_queue_lb_audit_indexed_min
                     : kInf,
                 std::isfinite(out.bc_stats.certificate_queue_lb_audit_computed_min)
                     ? out.bc_stats.certificate_queue_lb_audit_computed_min
                     : kInf,
                 out.bc_stats.certificate_queue_lb_audit_max_error,
	               out.bc_stats.certificate_queue_lb_audit_indexed_nodes,
	               out.bc_stats.certificate_queue_lb_audit_live_nodes,
	               out.bc_stats.certificate_domain_passes,
	               out.bc_stats.certificate_domain_fixings,
	               out.bc_stats.certificate_domain_prunes,
	               out.bc_stats.certificate_domain_bound_improvements,
	               out.bc_stats.termination_effective_candidates,
	               out.bc_stats.termination_effective_admitted,
	               out.bc_stats.termination_effective_skip_no_lp_or_queue_hit,
	               out.bc_stats.transformed_frontier_source_attempts,
	               out.bc_stats.transformed_frontier_tableau_rows,
	               out.bc_stats.transformed_frontier_path_rows,
	               out.bc_stats.transformed_frontier_violated_rows,
	               out.bc_stats.transformed_frontier_admitted_rows,
	               out.bc_stats.transformed_frontier_rejected_rows,
	               out.bc_stats.transformed_frontier_max_fractional_coeff,
	               out.bc_stats.transformed_frontier_max_violation,
	               out.bc_stats.transformed_frontier_resolve_samples,
	               out.bc_stats.transformed_frontier_resolve_bound_moves,
	               out.bc_stats.transformed_frontier_resolve_samples > 0
	                   ? out.bc_stats.transformed_frontier_resolve_lift_sum /
	                         static_cast<double>(
	                             out.bc_stats.transformed_frontier_resolve_samples)
	                   : 0.0,
	               out.bc_stats.transformed_frontier_resolve_lift_max,
	               out.bc_stats.rc_target_proof_diagnostic_samples,
	               out.bc_stats.rc_target_proof_diagnostic_samples > 0
	                   ? out.bc_stats.rc_target_proof_budget_sum /
	                         static_cast<double>(out.bc_stats.rc_target_proof_diagnostic_samples)
	                   : 0.0,
	               out.bc_stats.rc_target_proof_budget_max,
	               out.bc_stats.rc_target_proof_diagnostic_samples > 0
	                   ? out.bc_stats.rc_target_frontier_lp_activity_sum /
	                         static_cast<double>(out.bc_stats.rc_target_proof_diagnostic_samples)
	                   : 0.0,
	               out.bc_stats.rc_target_frontier_lp_activity_max,
	               out.bc_stats.rc_target_proof_diagnostic_samples > 0
	                   ? out.bc_stats.rc_target_cover_violation_sum /
	                         static_cast<double>(out.bc_stats.rc_target_proof_diagnostic_samples)
	                   : 0.0,
	               out.bc_stats.rc_target_cover_violation_max,
	               out.bc_stats.rc_proof_resolve_lift_samples,
	               out.bc_stats.rc_proof_resolve_lift_samples > 0
	                   ? out.bc_stats.rc_proof_resolve_lift_sum /
	                         static_cast<double>(out.bc_stats.rc_proof_resolve_lift_samples)
	                   : 0.0,
	               out.bc_stats.rc_proof_resolve_lift_max);
    if (num_threads <= 1) {
      MIPSOLVERS_LOG_DEBUG("[B&C TIMING] usf={:.1f}ms lp={:.1f}ms ({} calls, avg={:.0f}us)",
              usf_total_us / 1000.0, lp_total_us / 1000.0, timing_lp_count,
              timing_lp_count > 0 ? lp_total_us / timing_lp_count : 0.0);
      MIPSOLVERS_LOG_DEBUG("[B&C POOLS] cut_pool={} solution_pool={}",
              cut_pool.size(), solution_pool.size());
    } else {
      MIPSOLVERS_LOG_DEBUG("[B&C-PAR] threads={} (1 cut_worker + {} explorers)",
              num_threads, num_threads - 1);
      MIPSOLVERS_LOG_DEBUG("[B&C-PAR] cut_requests={}/{} cw_cuts={} cw_incumbents={}",
              out.bc_stats.cut_requests_submitted,
              out.bc_stats.cut_requests_dropped,
              out.bc_stats.cut_worker_cuts,
              out.bc_stats.cut_worker_incumbents);
    }
  }

  if (bc_conf_diag_enabled() && out.bc_stats.conformance_source_phases > 0) {
    fmt::print(stderr,
               "[B&C-CONF-SUMMARY] source={}/{} prop={} "
               "bound_change_phases={} bound_changes={} prunes={} "
               "resolve={}/fail{} bound_moves={} lift_sum/max={:.6g}/{:.6g} "
               "queue={} tight={} prunes={} bound_moves={} "
               "qlift_sum/max={:.6g}/{:.6g} "
               "objcap_phases={} active={}/{} one={}/{} "
               "rawEx={} proofEx={} maxOne={:.6g} proofExMax={:.6g}\n",
               out.bc_stats.conformance_source_nonempty_phases,
               out.bc_stats.conformance_source_phases,
               out.bc_stats.conformance_propagation_phases,
               out.bc_stats.conformance_bound_change_phases,
               out.bc_stats.conformance_bound_changes,
               out.bc_stats.conformance_domain_prunes,
               out.bc_stats.conformance_resolve_attempts,
               out.bc_stats.conformance_resolve_failures,
               out.bc_stats.conformance_resolve_bound_moves,
               out.bc_stats.conformance_bound_lift_sum,
               out.bc_stats.conformance_bound_lift_max,
               out.bc_stats.conformance_queue_phases,
               out.bc_stats.conformance_queue_tightenings,
               out.bc_stats.conformance_queue_prunes,
               out.bc_stats.conformance_queue_bound_moves,
               out.bc_stats.conformance_queue_lift_sum,
               out.bc_stats.conformance_queue_lift_max,
               out.bc_stats.conformance_objective_capacity_phases,
               out.bc_stats.conformance_objective_capacity_active_tight_events,
               out.bc_stats.conformance_objective_capacity_active_events,
               out.bc_stats.conformance_objective_capacity_one_missing_events,
               out.bc_stats.conformance_objective_capacity_one_missing_keys,
               out.bc_stats.conformance_objective_capacity_raw_exceed_keys,
               out.bc_stats.conformance_objective_capacity_proof_exceed_keys,
               out.bc_stats.conformance_objective_capacity_max_one_missing,
               out.bc_stats.conformance_objective_capacity_max_proof_excess);
  }

  if (out.bc_stats.propagation_calls > 0 &&
      (opt.verbose ||
       bc_env_flag_enabled("MIPSOLVERS_BC_PROPAGATION_PROFILE"))) {
    const std::uint64_t actual_rows = out.bc_stats.propagation_ineq_rows_visited +
                                      out.bc_stats.propagation_eq_rows_visited;
    const double visit_ratio = out.bc_stats.propagation_baseline_full_scan_rows > 0
        ? static_cast<double>(actual_rows) /
              static_cast<double>(out.bc_stats.propagation_baseline_full_scan_rows)
        : 0.0;
    fmt::print(stderr,
               "[PROP-PROFILE] calls={} passes={} changed_vars={} rows={} ineq_rows={} eq_rows={} baseline_rows={} row_visit_ratio={:.3f} tightenings={} wall_ms={:.1f}\n",
               out.bc_stats.propagation_calls,
               out.bc_stats.propagation_active_passes,
               out.bc_stats.propagation_changed_var_visits,
               actual_rows,
               out.bc_stats.propagation_ineq_rows_visited,
               out.bc_stats.propagation_eq_rows_visited,
               out.bc_stats.propagation_baseline_full_scan_rows,
               visit_ratio,
               out.bc_stats.propagation_bound_tightenings,
               out.bc_stats.propagation_wall_ns / 1.0e6);
  }

  const double objective_report_offset =
      strict_highs_presolved_working_space
          ? strict_highs_presolved_objective_offset
          : (papilo_model_reduced ? papilo_ps.objective_offset : 0.0);
  auto report_objective_value = [&](double internal_obj) {
    if (!std::isfinite(internal_obj)) return internal_obj;
    const double with_offset = internal_obj + objective_report_offset;
    return (base_lp.sense == Sense::Minimize) ? with_offset : -with_offset;
  };

  out.bc_stats.runtime_sec = runtime;
  out.bc_stats.best_bound = report_objective_value(global_lb);
  out.bc_stats.best_obj = has_incumbent ? report_objective_value(incumbent_obj) : kInf;
  if (std::isfinite(out.bc_stats.best_bound_at_last_incumbent)) {
    out.bc_stats.best_bound_at_last_incumbent =
        report_objective_value(out.bc_stats.best_bound_at_last_incumbent);
  }
  if (out.bc_stats.last_incumbent_node >= 0 &&
      std::isfinite(out.bc_stats.best_bound) &&
      std::isfinite(out.bc_stats.best_bound_at_last_incumbent)) {
    const double bound_delta =
        (base_lp.sense == Sense::Minimize)
            ? (out.bc_stats.best_bound - out.bc_stats.best_bound_at_last_incumbent)
            : (out.bc_stats.best_bound_at_last_incumbent - out.bc_stats.best_bound);
    out.bc_stats.best_bound_lift_after_last_incumbent = std::max(0.0, bound_delta);
  }

  out.stats.runtime_sec = runtime;
  out.stats.iterations = out.bc_stats.nodes_explored;
  out.stats.success = has_incumbent;
  out.stats.solver_name = "NativeBranchAndCut";

  if (has_incumbent) {
    out.x = incumbent_x;
    out.stats.objective = report_objective_value(incumbent_obj);
    out.stats.primal_feas = 0.0;
    out.stats.residual_inf = 0.0;

    if (!std::isfinite(out.bc_stats.gap) && std::isfinite(global_lb)) {
      const double denom = std::max(1.0, std::abs(incumbent_obj));
      out.bc_stats.gap = std::max(0.0, (incumbent_obj - global_lb) / denom);
    }
    if (out.stats.status == bc_status::kSearchQueueExhausted) {
      if (out.bc_stats.fallback_unsafe_prunes == 0 &&
          (!std::isfinite(out.bc_stats.gap) || out.bc_stats.gap <= opt.gap_tol)) {
        // Tree fully explored with no solver failures and a valid proof gap.
        out.bc_stats.gap = 0.0;
        out.stats.status = bc_status::kOptimalTreeExhausted;
        // The exhaustion proof makes the incumbent optimal; report the bound
        // at the incumbent instead of a stale weaker tree bound (which used
        // to show up as a 7-21% "gap" alongside an Optimal status).
        out.bc_stats.best_bound = out.bc_stats.best_obj;
      }
      // Otherwise keep the measured gap — queue exhaustion without a closed
      // proof gap is a solver/frontier failure, not an optimality proof.
    }
    out.stats.mip_gap = std::isfinite(out.bc_stats.gap) ? out.bc_stats.gap : 0.0;
  out.stats.cglp_cuts_added = out.bc_stats.cglp_accepted;

    if (out.stats.status.empty()) {
      out.stats.status = bc_status::kFeasibleIncumbentFound;
    }
  } else {
    out.x = Eigen::VectorXd::Zero(n);
    out.stats.objective = 0.0;
    out.stats.mip_gap = kInf;
    if (out.stats.status.empty()) {
      out.stats.status = bc_status::kNoFeasibleIntegerSolutionFound;
    }
  }

  if (has_incumbent) {
    // Search rows can be valid only below the incumbent cutoff and may
    // intentionally exclude the incumbent itself. Feasibility belongs to the
    // frozen reduced source model; original-space validation follows below.
    const auto reduced_validation =
        validate_incumbent_solution(pre_cut_root_lp, out.x, 1e-6);
    if (!reduced_validation.ok()) {
      if (opt.verbose) {
        fprintf(stderr,
                "[B&C-VALIDATE-FAIL] reduced-space failed: %s "
                "(source=%s cuts=%d)\n",
                format_incumbent_validation_failure("reduced", pre_cut_root_lp,
                                                    reduced_validation).c_str(),
                root_incumbent_source.c_str(),
                static_cast<int>(out.bc_stats.cuts_added));
      }
      has_incumbent = false;
      out.x = Eigen::VectorXd::Zero(n);
      out.stats.objective = 0.0;
      out.stats.mip_gap = kInf;
      out.stats.success = false;
      out.stats.status = fmt::format(
          "{} ({})",
          bc_status::kInvalidReducedIncumbent,
          format_incumbent_validation_failure("reduced", pre_cut_root_lp,
                                              reduced_validation));
    }
  }

  // ── PaPILO postsolve (undo presolve reduction) ──
#ifdef MIPSOLVERS_HAVE_PAPILO
  if (has_incumbent && papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
    const Eigen::VectorXd x_reduced_before_postsolve = out.x;
    const auto papilo_reduced_validation =
        papilo_validate_reduced_solution(papilo_ps, x_reduced_before_postsolve);
    if (opt.verbose && !papilo_reduced_validation.ok()) {
      fprintf(stderr,
              "[B&C-VALIDATE-FAIL] papilo-reduced-compressed failed before postsolve:%s\n",
              format_papilo_compressed_validation_detail(
                  papilo_reduced_validation).c_str());
    }
    out.x = papilo_postsolve(papilo_ps, out.x);
    const auto papilo_validation =
        validate_incumbent_solution(prob.linear_part, out.x, 1e-6);
    if (!papilo_validation.ok()) {
      if (opt.verbose) {
        const std::string row_detail =
            format_ineq_row_contributors(prob.linear_part, out.x,
                                         papilo_validation.ineq_row);
        const std::string reduced_detail =
            format_papilo_reduced_row_mapping_detail(
                papilo_ps, base_lp, x_reduced_before_postsolve,
                papilo_validation.ineq_row);
        const std::string compressed_detail =
            format_papilo_compressed_validation_detail(papilo_reduced_validation);
        const std::string compressed_native_row_detail =
            papilo_reduced_validation.worst_orig_row >= 0
                ? format_papilo_reduced_row_mapping_detail(
                      papilo_ps, base_lp, x_reduced_before_postsolve,
                      papilo_reduced_validation.worst_orig_row)
                : std::string();
        fprintf(stderr,
                "[B&C-VALIDATE-FAIL] papilo-postsolve failed: %s%s%s%s compressed_native%s\n",
                format_incumbent_validation_failure("papilo-postsolve",
                                                    prob.linear_part,
                                                    papilo_validation).c_str(),
                row_detail.c_str(),
                reduced_detail.c_str(),
                compressed_detail.c_str(),
                compressed_native_row_detail.c_str());
      }
      has_incumbent = false;
      out.x = Eigen::VectorXd::Zero(static_cast<int>(prob.linear_part.vars.size()));
      out.stats.objective = 0.0;
      out.stats.mip_gap = kInf;
      out.stats.success = false;
      out.stats.status = fmt::format(
          "{} ({})",
          bc_status::kInvalidPapiloPostsolveIncumbent,
          format_incumbent_validation_failure("papilo-postsolve",
                                              prob.linear_part,
                                              papilo_validation));
    }
  }
#endif

  // ── Native MILPPresolve postsolve (undo presolve reduction) ──
  // Restores the original-space incumbent (fixed/substituted columns) so the
  // objective recompute below evaluates orig_lp.c · out.x correctly.
  if (has_incumbent && native_model_reduced) {
    out.x = native_ps.postsolve(out.x);
    const auto native_validation =
        validate_incumbent_solution(prob.linear_part, out.x, 1e-6);
    if (!native_validation.ok()) {
      if (opt.verbose) {
        fprintf(stderr, "[B&C-VALIDATE-FAIL] native-postsolve failed: %s\n",
                format_incumbent_validation_failure("native-postsolve",
                                                    prob.linear_part,
                                                    native_validation)
                    .c_str());
      }
      has_incumbent = false;
      out.x = Eigen::VectorXd::Zero(
          static_cast<int>(prob.linear_part.vars.size()));
      out.stats.objective = 0.0;
      out.stats.mip_gap = kInf;
      out.stats.success = false;
      out.stats.status = fmt::format("{} (native-postsolve)",
                                     bc_status::kInvalidReducedIncumbent);
    }
  }

  // Recompute objective in original space after all postsolve passes.
  if (has_incumbent && !strict_highs_presolved_working_space) {
    const auto& orig_lp = prob.linear_part;
    double obj_val = 0.0;
    const int orig_nc = static_cast<int>(orig_lp.c.size());
    const int x_sz = static_cast<int>(out.x.size());
    for (int j = 0; j < orig_nc && j < x_sz; ++j)
      obj_val += orig_lp.c[j] * out.x[j];
    // The reduced model folds constants out of the objective (presolve
    // reductions), so the internal-space best_obj/best_bound diverge from
    // the original-space objective by that constant — showing up as a spurious
    // 7-21% "gap" on proven-optimal solves.  Shift the reported bookkeeping
    // by the same delta to keep all three in one (original) space.
    const double obj_delta = obj_val - out.stats.objective;
    if (obj_delta != 0.0) {
      if (std::isfinite(out.bc_stats.best_obj))
        out.bc_stats.best_obj += obj_delta;
      if (std::isfinite(out.bc_stats.best_bound))
        out.bc_stats.best_bound += obj_delta;
      if (std::isfinite(out.bc_stats.best_bound_at_last_incumbent))
        out.bc_stats.best_bound_at_last_incumbent += obj_delta;
    }
    out.stats.objective = obj_val;
  }

  // Convert every progress event to original model space. Incumbent snapshots
  // are postsolved and audited individually; one invalid intermediate
  // candidate invalidates the complete trajectory instead of leaving an
  // unaudited primal value available for PDI computation.
  if (out.bc_stats.bound_event_stream_available) {
    // Queue lower bounds observed during search are provisional until every
    // node that justified the frontier transition has a retained certificate.
    // If recovery failures force the final result back to a weaker certified
    // bound, discard stronger transient values instead of publishing a
    // regressing or uncertified dual trajectory.
    if (std::isfinite(global_lb)) {
      const double dual_tol =
          1e-9 * std::max(1.0, std::abs(global_lb));
      for (auto& event : out.bc_stats.bound_events) {
        if (std::isfinite(event.dual_bound) &&
            event.dual_bound > global_lb + dual_tol) {
          event.dual_bound = std::numeric_limits<double>::quiet_NaN();
          ++out.bc_stats.bound_events_dropped_uncertified_dual;
        }
      }
    }
    bool stream_valid = true;
    bool shift_known = false;
    double original_objective_shift = 0.0;
    for (auto& event : out.bc_stats.bound_events) {
      if (event.incumbent.size() == 0) continue;
      if (event.incumbent.size() != n ||
          !std::isfinite(event.primal_bound)) {
        stream_valid = false;
        break;
      }

      Eigen::VectorXd original_x = event.incumbent;
#ifdef MIPSOLVERS_HAVE_PAPILO
      if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
        original_x = papilo_postsolve(papilo_ps, original_x);
      }
#endif
      if (native_model_reduced) {
        original_x = native_ps.postsolve(original_x);
      }
      const auto event_validation =
          validate_incumbent_solution(prob.linear_part, original_x, 1e-6);
      if (!event_validation.ok() ||
          original_x.size() != prob.linear_part.c.size()) {
        stream_valid = false;
        break;
      }

      const double original_obj = prob.linear_part.c.dot(original_x);
      const double reported_internal =
          report_objective_value(event.primal_bound);
      if (!std::isfinite(original_obj) ||
          !std::isfinite(reported_internal)) {
        stream_valid = false;
        break;
      }
      const double event_shift = original_obj - reported_internal;
      const double shift_tol =
          1e-7 * std::max({1.0, std::abs(original_obj),
                           std::abs(reported_internal)});
      if (shift_known &&
          std::abs(event_shift - original_objective_shift) > shift_tol) {
        stream_valid = false;
        break;
      }
      if (!shift_known) {
        original_objective_shift = event_shift;
        shift_known = true;
      }
      event.primal_bound = original_obj;
      event.incumbent = std::move(original_x);
    }

    if (stream_valid) {
      for (auto& event : out.bc_stats.bound_events) {
        if (event.incumbent.size() == 0 &&
            std::isfinite(event.primal_bound)) {
          event.primal_bound =
              std::numeric_limits<double>::quiet_NaN();
        }
        if (std::isfinite(event.dual_bound)) {
          event.dual_bound = report_objective_value(event.dual_bound) +
                             (shift_known ? original_objective_shift : 0.0);
        }
        event.time_sec = std::clamp(event.time_sec, 0.0, runtime);
      }
    }
    if (!stream_valid) {
      out.bc_stats.bound_event_stream_available = false;
      out.bc_stats.bound_events.clear();
    } else {
      std::erase_if(out.bc_stats.bound_events, [](const BCBoundEvent& event) {
        return !std::isfinite(event.primal_bound) &&
               !std::isfinite(event.dual_bound);
      });
    }
  }

  // Export native pseudocost learning in original column space.  The same
  // payload is already used by the strict HiGHS path; keeping one public
  // representation lets repeated callers warm either LP/MIP backend.
  {
    auto exported = std::make_shared<BCPseudocostInit>();
    exported->n_orig_cols = orig_n;
    exported->pseudocostup.assign(static_cast<size_t>(orig_n), 0.0);
    exported->pseudocostdown.assign(static_cast<size_t>(orig_n), 0.0);
    exported->nsamplesup.assign(static_cast<size_t>(orig_n), 0);
    exported->nsamplesdown.assign(static_cast<size_t>(orig_n), 0);
    exported->inferencesup.assign(static_cast<size_t>(orig_n), 0.0);
    exported->inferencesdown.assign(static_cast<size_t>(orig_n), 0.0);
    exported->ninferencesup.assign(static_cast<size_t>(orig_n), 0);
    exported->ninferencesdown.assign(static_cast<size_t>(orig_n), 0);
    exported->conflictscoreup.assign(static_cast<size_t>(orig_n), 0.0);
    exported->conflictscoredown.assign(static_cast<size_t>(orig_n), 0.0);
    for (int col : declared_branchable_indices) {
      int orig_col = col;
#ifdef MIPSOLVERS_HAVE_PAPILO
      if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
        if (col < 0 ||
            col >= static_cast<int>(papilo_ps.reduced_to_orig_col.size()))
          continue;
        orig_col = papilo_ps.reduced_to_orig_col[static_cast<size_t>(col)];
      }
#endif
      if (native_model_reduced) {
        const auto& r2o = native_ps.reduced_to_orig_col();
        if (col < 0 || col >= static_cast<int>(r2o.size())) continue;
        orig_col = r2o[static_cast<std::size_t>(col)];
      }
      if (orig_col < 0 || orig_col >= orig_n) continue;
      const PseudoCost& p = pc[col];
      exported->pseudocostup[static_cast<size_t>(orig_col)] = p.up_avg();
      exported->pseudocostdown[static_cast<size_t>(orig_col)] = p.down_avg();
      exported->nsamplesup[static_cast<size_t>(orig_col)] = p.up_cnt;
      exported->nsamplesdown[static_cast<size_t>(orig_col)] = p.down_cnt;
      exported->inferencesup[static_cast<size_t>(orig_col)] =
          p.up_inference_avg();
      exported->inferencesdown[static_cast<size_t>(orig_col)] =
          p.down_inference_avg();
      exported->ninferencesup[static_cast<size_t>(orig_col)] =
          p.up_inference_cnt;
      exported->ninferencesdown[static_cast<size_t>(orig_col)] =
          p.down_inference_cnt;
      exported->conflictscoreup[static_cast<size_t>(orig_col)] =
          p.up_conflict_score;
      exported->conflictscoredown[static_cast<size_t>(orig_col)] =
          p.down_conflict_score;
      exported->nsamplestotal += p.up_cnt + p.down_cnt;
      exported->ninferencestotal +=
          p.up_inference_cnt + p.down_inference_cnt;
    }
    if (exported->nsamplestotal > 0)
      out.highs_pseudocost_init = std::move(exported);
  }

  accumulate_queue_storage(node_queue.domain_storage_stats(),
                           QueueStorageSource::Serial);
  accumulate_payload_sharing(serial_payload_sharing);
  if (cts) {
    accumulate_queue_storage(cts->shared_queue.domain_storage_stats(),
                             QueueStorageSource::Parallel);
    out.bc_stats.branch_payload_child_creations +=
        cts->atomic_stats.branch_payload_child_creations.load();
    out.bc_stats.branch_payload_shared_vectors +=
        cts->atomic_stats.branch_payload_shared_vectors.load();
	    out.bc_stats.branch_payload_shared_elements +=
	        cts->atomic_stats.branch_payload_shared_elements.load();
	    out.bc_stats.branch_domain_dense_copies +=
	        cts->atomic_stats.branch_domain_dense_copies.load();
	    out.bc_stats.branch_domain_dense_values_copied +=
	        cts->atomic_stats.branch_domain_dense_values_copied.load();
	    out.bc_stats.branch_domain_moves +=
	        cts->atomic_stats.branch_domain_moves.load();
	    out.bc_stats.strong_probe_base_sf_materializations +=
	        cts->atomic_stats.strong_probe_base_sf_materializations.load();
	    out.bc_stats.strong_probe_bound_transactions +=
	        cts->atomic_stats.strong_probe_bound_transactions.load();
	    out.bc_stats.strong_probe_transaction_snapshot_values +=
	        cts->atomic_stats.strong_probe_transaction_snapshot_values.load();
	    out.bc_stats.strong_probe_transaction_rollbacks +=
	        cts->atomic_stats.strong_probe_transaction_rollbacks.load();
	    out.bc_stats.strong_probe_transaction_failures +=
	        cts->atomic_stats.strong_probe_transaction_failures.load();
	    out.bc_stats.strong_probe_backend_cold_solves +=
	        cts->atomic_stats.strong_probe_backend_cold_solves.load();
	    out.bc_stats.strong_probe_backend_persistent_resolves +=
	        cts->atomic_stats.strong_probe_backend_persistent_resolves.load();
    out.bc_stats.separator_sparse_candidates_created +=
        cts->atomic_stats.separator_sparse_candidates_created.load();
    out.bc_stats.separator_sparse_candidate_entries_created +=
        cts->atomic_stats.separator_sparse_candidate_entries_created.load();
    out.bc_stats.separator_peak_live_sparse_candidates = std::max(
        out.bc_stats.separator_peak_live_sparse_candidates,
        cts->atomic_stats.separator_peak_live_sparse_candidates.load());
    out.bc_stats.separator_peak_live_sparse_entries = std::max(
        out.bc_stats.separator_peak_live_sparse_entries,
        cts->atomic_stats.separator_peak_live_sparse_entries.load());
    out.bc_stats.separator_sparse_aggregation_snapshots +=
        cts->atomic_stats.separator_sparse_aggregation_snapshots.load();
    out.bc_stats.separator_sparse_aggregation_entries +=
        cts->atomic_stats.separator_sparse_aggregation_entries.load();
    out.bc_stats.separator_dense_workspace_materializations +=
        cts->atomic_stats.separator_dense_workspace_materializations.load();
    out.bc_stats.separator_dense_workspace_values +=
        cts->atomic_stats.separator_dense_workspace_values.load();
    out.bc_stats.separator_matrix_append_calls +=
        cts->atomic_stats.separator_matrix_append_calls.load();
    out.bc_stats.separator_matrix_appended_rows +=
        cts->atomic_stats.separator_matrix_appended_rows.load();
    out.bc_stats.separator_matrix_appended_entries +=
        cts->atomic_stats.separator_matrix_appended_entries.load();
    out.bc_stats.separator_matrix_prior_entries_bypassing_triplet_rebuild +=
        cts->atomic_stats
            .separator_matrix_prior_entries_bypassing_triplet_rebuild.load();
    out.bc_stats.separator_matrix_storage_reallocations +=
        cts->atomic_stats.separator_matrix_storage_reallocations.load();
    out.bc_stats.separator_matrix_peak_spare_entries = std::max(
        out.bc_stats.separator_matrix_peak_spare_entries,
        cts->atomic_stats.separator_matrix_peak_spare_entries.load());
  }

  out.bc_stats.status = out.stats.status;

  // Merge fallback statistics from sequential path.
  {
    auto fb_sum = seq_fallback_logger.summary();
    out.bc_stats.fallback_events += fb_sum.total_events;
    out.bc_stats.fallback_recoveries += fb_sum.nodes_recovered;
    out.bc_stats.fallback_deferrals += fb_sum.nodes_deferred;
    out.bc_stats.fallback_prunes += fb_sum.nodes_pruned;
    out.bc_stats.fallback_infeasible_prunes += fb_sum.nodes_pruned_infeasible;
    out.bc_stats.fallback_unbounded_prunes += fb_sum.nodes_pruned_unbounded;
    out.bc_stats.fallback_unsafe_prunes += fb_sum.nodes_pruned_unsafe;
    out.bc_stats.fallback_l1_success += fb_sum.l1_successes;
    out.bc_stats.fallback_l2_success += fb_sum.l2_successes;
    out.bc_stats.fallback_l3_success += fb_sum.l3_successes;
  }

  if (opt.verbose && out.bc_stats.fallback_events > 0) {
    MIPSOLVERS_LOG_DEBUG("[B&C] fallback events={} recovered={} deferred={} pruned={} "
                      "(inf={} unb={} unsafe={}) L1={} L2={} L3={}",
            out.bc_stats.fallback_events,
            out.bc_stats.fallback_recoveries,
            out.bc_stats.fallback_deferrals,
            out.bc_stats.fallback_prunes,
            out.bc_stats.fallback_infeasible_prunes,
            out.bc_stats.fallback_unbounded_prunes,
            out.bc_stats.fallback_unsafe_prunes,
            out.bc_stats.fallback_l1_success,
            out.bc_stats.fallback_l2_success,
            out.bc_stats.fallback_l3_success);
  }

  return return_with_profile();

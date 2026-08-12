// @file bc_run/13_main_loop_sequential.inc
// BCSolveState::run() segment 13/14: node-processing helpers, main B&C loop setup, and the sequential branch-and-bound loop.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
	          } else {
	            if (child.lb[missing_col] >= 1.0 - tol) return;
	            fixing = BranchDomainLiteral{missing_col, 1.0, true};
	          }
	          ++out.bc_stats.termination_effective_candidates;
	          const double fixing_lp_violation = fixing.is_lb
	              ? std::max(0.0, fixing.value - child.x_relax[fixing.var_idx])
	              : std::max(0.0, child.x_relax[fixing.var_idx] -
	                                  fixing.value);
	          if (fixing_lp_violation <= lp_violation_tol) {
	            ++out.bc_stats.termination_effective_skip_no_lp_or_queue_hit;
	            return;
	          }
		          struct AggregateProofEvent {
		            double proof_delta{0.0};
		            double objective_delta{0.0};
		            std::unordered_map<int, double> best_by_target;
		            std::unordered_map<int, DomainReasonBound> best_target_bound;
		            std::vector<BranchDomainLiteral> reason;
		          };
		          AggregateProofEvent aggregate;
		          auto add_reason_literal =
		              [](std::vector<BranchDomainLiteral>& reason,
		                 const BranchDomainLiteral& literal) {
	            const auto found = std::find_if(
	                reason.begin(), reason.end(),
	                [&](const BranchDomainLiteral& r) {
	                  return r.var_idx == literal.var_idx &&
	                         r.is_lb == literal.is_lb &&
	                         std::abs(r.value - literal.value) <= 1e-9;
	                });
	            if (found == reason.end()) reason.push_back(literal);
	          };
	          for (int event_id :
	               objective_propagation.implied_events_by_literal[key]) {
	            if (event_id < 0 ||
	                event_id >=
	                    static_cast<int>(objective_propagation
	                                         .implied_events.size())) {
	              continue;
	            }
	            const auto& event =
	                objective_propagation
	                    .implied_events[static_cast<std::size_t>(event_id)];
	            const int target = event.target_col;
	            if (target < 0 || target >= n) {
	              continue;
	            }
	            const double a = proof->coeff.coeff(target);
	            double proof_delta = 0.0;
	            double objective_delta = 0.0;
	            if (event.target_is_lower_bound) {
	              if (event.target_bound <= child.lb[target] + tol) continue;
	              if (!(a > 1e-12)) {
	                ++skip_coeff;
	                continue;
	              }
	              proof_delta = a * (event.target_bound - child.lb[target]);
	              objective_delta =
	                  std::max(0.0, event.target_cost) *
	                  (event.target_bound - child.lb[target]);
	            } else {
	              if (event.target_bound >= child.ub[target] - tol) continue;
	              if (!(a < -1e-12)) {
	                ++skip_coeff;
	                continue;
	              }
	              proof_delta = (-a) * (child.ub[target] - event.target_bound);
	              objective_delta =
	                  std::max(0.0, -event.target_cost) *
	                  (child.ub[target] - event.target_bound);
	            }
	            if (!(proof_delta > 1e-9) || !std::isfinite(proof_delta)) {
	              continue;
	            }
	            int missing = -1;
	            bool inactive = false;
	            std::vector<BranchDomainLiteral> active_reason;
	            for (int li = 0;
	                 li < static_cast<int>(event.literals.size()); ++li) {
	              const auto& lit =
	                  event.literals[static_cast<std::size_t>(li)];
	              if (lit.col < 0 || lit.col >= n) {
	                inactive = true;
	                break;
	              }
	              if (lit.col == missing_col &&
	                  lit.value_one == missing_value_one) {
	                if (missing >= 0 || !literal_possible(lit)) {
	                  inactive = true;
	                  break;
	                }
	                missing = li;
	                continue;
	              }
	              if (!literal_active(lit)) {
	                inactive = true;
	                break;
	              }
		              active_reason.push_back(literal_reason(lit));
		            }
		            if (inactive || missing < 0) continue;
			            DomainReasonBound event_target;
			            event_target.bound =
			                BranchDomainLiteral{target, event.target_bound,
			                                    event.target_is_lower_bound};
			            event_target.reason = active_reason;
			            event_target.trail_pos =
			                child.local_domain_trail.empty()
			                    ? static_cast<int>(child.branch_reasons.size() +
			                                       child.domain_reason_bounds.size() +
			                                       aggregate.best_by_target.size())
			                    : static_cast<int>(child.local_domain_trail.size() +
			                                       aggregate.best_by_target.size());
			            event_target.depth = child.depth;
			            event_target.source_conflict_literal =
			                event.target_is_lower_bound
			                    ? BranchDomainLiteral{
			                          target,
			                          is_integer_type(base_lp.vars[
			                              static_cast<std::size_t>(target)])
			                              ? std::ceil(event.target_bound - tol) - 1.0
			                              : event.target_bound -
			                                    std::max(1e-7, 10.0 * tol),
			                          false}
			                    : BranchDomainLiteral{
			                          target,
			                          is_integer_type(base_lp.vars[
			                              static_cast<std::size_t>(target)])
			                              ? std::floor(event.target_bound + tol) + 1.0
			                              : event.target_bound +
			                                    std::max(1e-7, 10.0 * tol),
			                          true};
			            event_target.has_source_conflict_literal = true;
		            auto [best_it, inserted] =
		                aggregate.best_by_target.emplace(target, proof_delta);
		            if (inserted) {
		              aggregate.proof_delta += proof_delta;
		              aggregate.objective_delta += std::max(0.0, objective_delta);
		              aggregate.best_target_bound.emplace(target,
		                                                  std::move(event_target));
		              for (const BranchDomainLiteral& r : active_reason) {
		                add_reason_literal(aggregate.reason, r);
		              }
		            } else if (proof_delta > best_it->second + 1e-9) {
		              aggregate.proof_delta += proof_delta - best_it->second;
		              aggregate.objective_delta += std::max(0.0, objective_delta);
		              best_it->second = proof_delta;
		              aggregate.best_target_bound[target] =
		                  std::move(event_target);
		              for (const BranchDomainLiteral& r : active_reason) {
		                add_reason_literal(aggregate.reason, r);
		              }
	            }
	          }
		          if (!(aggregate.proof_delta > 1e-9)) return;
		          ++proof_candidates;
			          const double score =
			              aggregate.proof_delta + 1e-3 * aggregate.objective_delta;
		          for (const auto& [target, raw_event_target] :
		               aggregate.best_target_bound) {
		            (void)target;
		            push_objective_event_target(candidates, raw_event_target, score);
		          }
			          if (!(node_proof_activity + aggregate.proof_delta >
			                proof_budget + proof_tol)) {
			            ++skip_budget;
			            return;
			          }
			          DomainReasonBound fixing_target;
			          fixing_target.bound = fixing;
			          fixing_target.reason = aggregate.reason;
			          fixing_target.trail_pos =
			              child.local_domain_trail.empty()
			                  ? static_cast<int>(child.branch_reasons.size() +
			                                     child.domain_reason_bounds.size() +
			                                     proof_target_bounds.size() +
			                                     candidates.size())
			                  : static_cast<int>(child.local_domain_trail.size() +
			                                     proof_target_bounds.size() +
			                                     candidates.size());
			          fixing_target.depth = child.depth;
			          fixing_target.source_conflict_literal = forbidden;
			          fixing_target.has_source_conflict_literal = true;
		          if (!duplicate_candidate_bound(candidates, fixing_target)) {
		            candidates.push_back(Candidate{std::move(fixing_target), score});
		          }
		        };
	        for (int col = 0; col < n; ++col) {
	          if (base_lp.vars[static_cast<std::size_t>(col)].type !=
	              VarType::Binary) {
	            continue;
	          }
	          if (child.x_relax[col] > lp_violation_tol &&
	              child.ub[col] >= 1.0 - tol &&
	              child.lb[col] < 1.0 - tol) {
	            process_missing_literal_events(col, true);
	          }
	          if (child.x_relax[col] < 1.0 - lp_violation_tol &&
	              child.lb[col] <= tol &&
	              child.ub[col] > tol) {
	            process_missing_literal_events(col, false);
	          }
	        }
	        std::sort(candidates.begin(), candidates.end(),
	                  [](const Candidate& a, const Candidate& b) {
	                    if (a.score != b.score) return a.score > b.score;
	                    return a.rb.reason.size() < b.rb.reason.size();
	                  });
	        const int take = std::min<int>(candidates.size(), 1);
	        for (int i = 0; i < take; ++i) {
	          proof_target_bounds.push_back(
	              std::move(candidates[static_cast<std::size_t>(i)].rb));
	        }
	        out.bc_stats.termination_effective_admitted +=
	            static_cast<std::uint64_t>(take);
	        out.bc_stats.objective_implied_event_proof_candidates +=
	            proof_candidates;
	        out.bc_stats.objective_implied_event_proof_fixings +=
	            static_cast<std::uint64_t>(take);
	        out.bc_stats.objective_implied_event_proof_skip_coeff += skip_coeff;
	        out.bc_stats.objective_implied_event_proof_skip_budget += skip_budget;
	        return take;
	      };
	      int objective_proof_fixings =
	          learn_objective_event_proof_fixings(target_dual_proof);
	      if (objective_proof_fixings == 0 && fallback_dual_proof != nullptr &&
	          fallback_dual_proof != target_dual_proof) {
	        objective_proof_fixings =
	            learn_objective_event_proof_fixings(fallback_dual_proof);
	      }
	      if (objective_proof_fixings > 0) {
	        learned_targets += objective_proof_fixings;
	        out.bc_stats.objective_implied_event_tightenings +=
	            static_cast<std::uint64_t>(objective_proof_fixings);
	      }
	      if (objective_event_bound_lifts > 0 && !node_queue.empty()) {
	        learned_targets += objective_event_bound_lifts;
	        const int queue_pruned = prune_queue_after_domain_learning(
	            /*force_frontier_drain=*/true,
	            "objective_event_bound_lift");
	        if (queue_pruned > 0) {
	          out.bc_stats.rc_conflict_queue_prunes +=
	              static_cast<std::uint64_t>(queue_pruned);
	        }
	        forced_certificate_queue_passes =
	            std::max(forced_certificate_queue_passes, 1);
	      }
	      const int local_trail_bound_lifts =
	          publish_local_trail_bound_lifting_certificates(
	              child, target_dual_proof, priority_dual_proof,
	              "node_local_trail_bound_lift");
	      if (local_trail_bound_lifts > 0) {
	        learned_targets += local_trail_bound_lifts;
	        branch_obs.has_inference = true;
	        branch_obs.conflict_score +=
	            static_cast<double>(local_trail_bound_lifts);
	        out.bc_stats.termination_effective_admitted +=
	            static_cast<std::uint64_t>(local_trail_bound_lifts);
	      }
	      int local_trail_cutoff_conflicts = 0;
	      if (has_inc_snapshot && std::isfinite(inc_obj_snapshot)) {
	        std::array<const DualProofRow*, 3> proof_rows{
	            target_dual_proof, fallback_dual_proof,
	            child_dual_proof_ok ? &child_objective_event_proof : nullptr};
	        std::unordered_set<const DualProofRow*> seen_proof_rows;
	        for (const DualProofRow* proof_row : proof_rows) {
	          if (proof_row == nullptr ||
	              !seen_proof_rows.insert(proof_row).second ||
	              !proof_row->valid) {
	            continue;
	          }
	          local_trail_cutoff_conflicts +=
	              publish_local_trail_cutoff_conflict_certificates(
	                  child, proof_row, proof_row->rhs,
	                  "node_local_trail_cutoff_conflict");
	        }
	      }
	      if (local_trail_cutoff_conflicts > 0) {
	        learned_targets += local_trail_cutoff_conflicts;
	        branch_obs.has_inference = true;
	        branch_obs.conflict_score +=
	            static_cast<double>(local_trail_cutoff_conflicts);
	        out.bc_stats.termination_effective_admitted +=
	            static_cast<std::uint64_t>(local_trail_cutoff_conflicts);
	        if (!node_queue.empty()) {
	          const int queue_pruned = prune_queue_after_domain_learning(
	              /*force_frontier_drain=*/true,
	              "node_local_trail_cutoff_conflict");
	          if (queue_pruned > 0) {
	            out.bc_stats.rc_conflict_queue_prunes +=
	                static_cast<std::uint64_t>(queue_pruned);
	          }
	        }
	      }
	      if (learned_targets > 0) {
	        branch_obs.has_inference = true;
	        branch_obs.inference_count += static_cast<double>(learned_targets);
	        branch_obs.conflict_score += static_cast<double>(learned_targets);
	      }
	      if (local_trail_bound_lifts > 0 && !node_queue.empty()) {
	        const int queue_pruned = prune_queue_after_domain_learning(
	            /*force_frontier_drain=*/true,
	            "node_local_trail_bound_lift");
	        if (queue_pruned > 0) {
	          out.bc_stats.rc_conflict_queue_prunes +=
	              static_cast<std::uint64_t>(queue_pruned);
	        }
	      }
	      if (!proof_target_bounds.empty()) {
	        has_new_proof_target_bounds = true;
	        bool target_tightened = false;
		        const bool allow_target_tightening =
		            allow_certificate_domain_replay ||
                    rc_proof_cut_resolve_lps_left > 0;
	        auto active_literal = [&](const BranchDomainLiteral& lit) {
	          if (lit.var_idx < 0 || lit.var_idx >= n) return false;
	          const double tol = std::max(1e-9, opt.int_tol);
	          return lit.is_lb ? (child.lb[lit.var_idx] >= lit.value - tol)
	                           : (child.ub[lit.var_idx] <= lit.value + tol);
	        };
	        auto reason_active = [&](const std::vector<BranchDomainLiteral>& reason) {
	          for (const auto& lit : reason) {
	            if (!active_literal(lit)) return false;
	          }
	          return true;
	        };
		        auto remember_explained_bound = [&](const DomainReasonBound& rb) {
		          for (const auto& existing : child.domain_reason_bounds) {
		            if (conflict_clause_hash(existing.reason) ==
		                    conflict_clause_hash(rb.reason) &&
		                domain_reason_bounds_equal(existing, rb)) {
		              return;
		            }
		          }
			          append_domain_reason_bound(child, rb, n, &root.lb, &root.ub);
			          add_scoped_target_bound_certificate(child, rb);
			          if (publish_bound_lifting_certificate &&
			              rb.has_source_conflict_literal &&
			              !rb.reason.empty()) {
			            publish_bound_lifting_certificate(
			                rb, nullptr, nullptr,
			                "proof_target_bound_lift");
			          }
			          if (rb.reason.size() == 1) {
	            const auto& trig = rb.reason.front();
	            const bool trig_one =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                trig.is_lb && trig.value >= 1.0 - opt.int_tol;
	            const bool trig_zero =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                !trig.is_lb && trig.value <= opt.int_tol;
	            if (trig_one || trig_zero) {
	              add_local_binary_implication(child, trig.var_idx, trig_one,
	                                           rb.bound.var_idx,
	                                           rb.bound.is_lb, rb.bound.value);
	            }
	          }
	        };
		        for (const auto& rb : proof_target_bounds) {
		          const int j = rb.bound.var_idx;
		          if (j < 0 || j >= n || !std::isfinite(rb.bound.value)) continue;
		          add_scoped_target_bound_certificate(child, rb);
		          const bool frontier_active =
		              rb.reason.empty() || reason_active(rb.reason);
	          if (rb.bound.is_lb) {
	            if (frontier_active &&
	                rb.bound.value > child.ub[j] + std::max(1e-9, opt.int_tol)) {
	              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	            if (allow_target_tightening &&
	                frontier_active &&
	                rb.bound.value > child.lb[j] + std::max(1e-9, opt.int_tol)) {
	              child.lb[j] = rb.bound.value;
	              remember_explained_bound(rb);
	              target_tightened = true;
	              ++out.bc_stats.scoped_conflict_tightenings;
	            } else if (frontier_active || active_literal(rb.bound)) {
	              remember_explained_bound(rb);
	            }
	          } else {
	            if (frontier_active &&
	                rb.bound.value < child.lb[j] - std::max(1e-9, opt.int_tol)) {
	              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	            if (allow_target_tightening &&
	                frontier_active &&
	                rb.bound.value < child.ub[j] - std::max(1e-9, opt.int_tol)) {
	              child.ub[j] = rb.bound.value;
	              remember_explained_bound(rb);
	              target_tightened = true;
	              ++out.bc_stats.scoped_conflict_tightenings;
	            } else if (frontier_active || active_literal(rb.bound)) {
	              remember_explained_bound(rb);
	            }
	          }
	        }
			        if (target_tightened &&
	                    (allow_certificate_domain_replay ||
	                     rc_proof_cut_resolve_lps_left > 0)) {
		          std::vector<BoundChangeInfo> target_prop_changes;
		          NodeDomainClosureStats target_closure_stats;
		          const std::vector<DomainReasonBound> no_target_reasons;
		          if (!run_node_domain_closure(
		                  child, no_target_reasons, target_prop_changes,
		                  &new_cuts, &cut_count, inc_obj_snapshot,
		                  has_inc_snapshot, "proof_target_closure",
		                  &target_closure_stats)) {
		            branch_obs.cutoff_count = 1;
		            pc_upd.push_back(branch_obs);
		            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		          }
		          if (!target_prop_changes.empty()) {
		            branch_obs.has_inference = true;
		            branch_obs.inference_count +=
		                static_cast<double>(target_prop_changes.size());
	          }
	          const bool allow_target_lp_refresh =
	              allow_native_proof_cut_rows &&
	              rc_proof_cut_resolve_lps_left > 0;
	          if (allow_target_lp_refresh) {
	          node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
	          SimplexOptions target_resolve_opt = simplex_opt;
	          target_resolve_opt.allow_cold_start = true;
	          auto target_res = solve_tree_lp(node_sf_ref, target_resolve_opt,
	                                          child.basis_hint.get());
	          --rc_proof_cut_resolve_lps_left;
	          ++lp_count;
	          ++out.bc_stats.rc_fixing_resolve_lps;
	          const auto target_type = classify_lp_result(target_res, n);
		          if (target_type == LPFailureType::Infeasible ||
		              target_type == LPFailureType::ObjectiveCutoff) {
		            if (target_type == LPFailureType::ObjectiveCutoff) {
		              child.bound = std::max(child.bound,
		                                     target_res.result.stats.objective);
		            }
		            publish_child_bound_exceeding_conflict("proof_target_resolve_cutoff");
		            ++out.bc_stats.rc_fixing_resolve_prunes;
	            branch_obs.cutoff_count = 1;
	            pc_upd.push_back(branch_obs);
	            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	          }
	          if (target_type == LPFailureType::Success &&
	              target_res.result.x.size() == n &&
	              std::isfinite(target_res.result.stats.objective)) {
	            const double old_bound = child.bound;
	            child.x_relax = clamp_to_bounds(target_res.result.x, child.lb, child.ub);
		            child.x_seed = child.x_relax;
		            child.bound = std::max(child.bound,
		                                   target_res.result.stats.objective);
		            {
		              const double lift = std::max(0.0, child.bound - old_bound);
		              ++out.bc_stats.rc_proof_resolve_lift_samples;
		              out.bc_stats.rc_proof_resolve_lift_sum += lift;
		              out.bc_stats.rc_proof_resolve_lift_max =
		                  std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
		            }
		            child.basis_hint =
		                persist_bc_node_basis_from_simplex(target_res,
		                                                child.basis_hint);
		            if (child.bound > old_bound + 1e-7) {
		              ++out.bc_stats.rc_fixing_resolve_bound_improvements;
		              branch_obs.has_gain = true;
	              branch_obs.gain = std::max(
	                  branch_obs.gain,
		                  std::max(0.0, child.bound - parent_bound));
		            }
		            if (collect_integral_lp_candidate("proof_target_resolve")) {
		              pc_upd.push_back(branch_obs);
		              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		            }
			            if (incumbent_prunes_node(has_child_upper_limit,
			                                      child_upper_limit_snapshot,
			                                      child.bound,
			                                      std::max(1e-9, opt.lp_tol))) {
		              publish_child_bound_exceeding_conflict("proof_target_resolve_bound");
		              ++out.bc_stats.rc_fixing_resolve_prunes;
	              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	            if (prune_child_by_gap_optimality_limit(
	                    "proof_target_resolve_optimality_limit")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	          }
	        } else {
	          if (child.x_relax.size() == n) {
	            child.x_relax = clamp_to_bounds(child.x_relax, child.lb, child.ub);
	          }
	          child.lp_refresh_needed = true;
	        }
	        }
	      }
	    }

		    if (opt.enable_reduced_cost_proof_conflict_minimization &&
		        (child_closure_stats.reasons_added > 0 ||
		         has_new_proof_target_bounds)) {
	      bool learned_scoped_reconvergence = false;
	      if (learn_short_reconvergence_implication(child.branch_reasons,
	                                                child.domain_reason_bounds,
	                                                child.x_relax.size() == n
	                                                    ? &child.x_relax
	                                                    : nullptr,
                                                &child.lb,
	                                                &child.ub,
	                                                &local_proof_cuts,
	                                                new_cuts, cut_count, &child)) {
	        learned_scoped_reconvergence = true;
	        branch_obs.has_inference = true;
	        branch_obs.inference_count += 1.0;
	        branch_obs.conflict_score += 1.0;
	      }
		      if (learned_scoped_reconvergence &&
                  (allow_certificate_domain_replay ||
                   rc_proof_cut_resolve_lps_left > 0)) {
	        std::vector<BoundChangeInfo> scoped_post_changes;
	        if (!explain_active_scoped_reason_bounds(child, scoped_post_changes)) {
	          branch_obs.cutoff_count = 1;
	          pc_upd.push_back(branch_obs);
	          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	        }
	        if (!propagate_scoped_conflict_objects(child, scoped_post_changes)) {
	          branch_obs.cutoff_count = 1;
	          pc_upd.push_back(branch_obs);
	          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	        }
	        if (!scoped_post_changes.empty()) {
	          rebuild_local_domain_trail(child, n, &root.lb, &root.ub);
	          const bool allow_scoped_lp_refresh =
	              allow_native_proof_cut_rows &&
	              rc_proof_cut_resolve_lps_left > 0;
	          if (allow_scoped_lp_refresh) {
	          node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
	          SimplexOptions scoped_resolve_opt = simplex_opt;
	          scoped_resolve_opt.allow_cold_start = true;
	          auto scoped_res = solve_tree_lp(node_sf_ref, scoped_resolve_opt,
	                                          child.basis_hint.get());
	          --rc_proof_cut_resolve_lps_left;
	          ++lp_count;
	          ++out.bc_stats.rc_fixing_resolve_lps;
	          const auto scoped_type = classify_lp_result(scoped_res, n);
		          if (scoped_type == LPFailureType::Infeasible ||
		              scoped_type == LPFailureType::ObjectiveCutoff) {
		            if (scoped_type == LPFailureType::ObjectiveCutoff) {
		              child.bound = std::max(child.bound,
		                                     scoped_res.result.stats.objective);
		            }
		            publish_child_bound_exceeding_conflict("scoped_resolve_cutoff");
		            ++out.bc_stats.rc_fixing_resolve_prunes;
	            branch_obs.cutoff_count = 1;
	            pc_upd.push_back(branch_obs);
	            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	          }
	          if (scoped_type == LPFailureType::Success &&
	              scoped_res.result.x.size() == n &&
	              std::isfinite(scoped_res.result.stats.objective)) {
	            const double old_bound = child.bound;
	            child.x_relax = clamp_to_bounds(scoped_res.result.x, child.lb, child.ub);
		            child.x_seed = child.x_relax;
		            child.bound = std::max(child.bound,
		                                   scoped_res.result.stats.objective);
		            {
		              const double lift = std::max(0.0, child.bound - old_bound);
		              ++out.bc_stats.rc_proof_resolve_lift_samples;
		              out.bc_stats.rc_proof_resolve_lift_sum += lift;
		              out.bc_stats.rc_proof_resolve_lift_max =
		                  std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
		            }
		            child.basis_hint =
		                persist_bc_node_basis_from_simplex(scoped_res,
		                                                child.basis_hint);
		            if (child.bound > old_bound + 1e-7) {
		              ++out.bc_stats.rc_fixing_resolve_bound_improvements;
		              branch_obs.has_gain = true;
	              branch_obs.gain = std::max(
	                  branch_obs.gain,
		                  std::max(0.0, child.bound - parent_bound));
		            }
		            if (collect_integral_lp_candidate("scoped_resolve")) {
		              pc_upd.push_back(branch_obs);
		              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		            }
			            if (incumbent_prunes_node(has_child_upper_limit,
			                                      child_upper_limit_snapshot,
			                                      child.bound,
			                                      std::max(1e-9, opt.lp_tol))) {
		              publish_child_bound_exceeding_conflict("scoped_resolve_bound");
		              ++out.bc_stats.rc_fixing_resolve_prunes;
	              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	            if (prune_child_by_gap_optimality_limit(
	                    "scoped_resolve_optimality_limit")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	          }
	        } else {
	          if (child.x_relax.size() == n) {
	            child.x_relax = clamp_to_bounds(child.x_relax, child.lb, child.ub);
	          }
	          child.lp_refresh_needed = true;
	        }
	        }
	      }
	    }

			    if (allow_native_proof_cut_rows &&
			        rc_proof_cut_resolve_lps_left > 0 &&
		        child.x_relax.size() == n) {
		      const int scoped_rows = separate_active_scoped_conflict_rows(
		          child, child.x_relax, local_proof_cuts,
		          std::max(0, opt.reduced_cost_conflict_max_per_node));
		      if (scoped_rows > 0) {
		        branch_obs.has_inference = true;
		        branch_obs.conflict_score += static_cast<double>(scoped_rows);
		      }
		    }

			    if (opt.enable_graph_implied_bound_cuts &&
			        allow_native_proof_cut_rows &&
        !implication_graph.empty() &&
        child.x_relax.size() == n) {
      std::vector<Eigen::SparseVector<double>> ib_rows;
      std::vector<double> ib_rhs;
      const int ib_budget = std::max(0, opt.graph_implied_bound_max_cuts);
      const int ib_added = append_graph_implied_bound_cuts(
          base_lp, child.x_relax, child.lb, child.ub, implication_graph,
          ib_budget, ib_rows, ib_rhs);
      for (int i = 0; i < ib_added; ++i) {
        const std::size_t h = sparse_cut_hash(ib_rows[i]);
        if (contains_identical_local_cut(child, local_proof_cuts, ib_rows[i],
                                         ib_rhs[i])) {
          continue;
        }
        const double norm = std::sqrt(std::max(0.0, ib_rows[i].squaredNorm()));
        ++out.bc_stats.local_node_cuts_generated;
        PoolCut cut{std::move(ib_rows[i]), ib_rhs[i], 0, 0.0, norm, h};
        local_proof_cuts.push_back(
            make_local_node_cut(std::move(cut)));
      }
    }

    separate_local_probed_implied_bound_cuts(child, local_proof_cuts,
                                             inc_obj_snapshot,
                                             has_inc_snapshot);

    if (std::isfinite(root_domain_bound_floor) &&
        child.bound < root_domain_bound_floor) {
      child.bound = root_domain_bound_floor;
    }
    apply_domain_objective_lower_bound(child, "child_pre_cutoff");

    const double gain = std::max(0.0, child.bound - parent_bound);
    branch_obs.has_gain = true;
    branch_obs.gain = gain;

	    if (incumbent_prunes_node(has_child_upper_limit,
	                              child_upper_limit_snapshot, child.bound,
	                              std::max(1e-9, opt.lp_tol))) {
	      publish_child_bound_exceeding_conflict("node_bound_exceeding");
	      branch_obs.cutoff_count = 1;
	      pc_upd.push_back(branch_obs);
	      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
    }

    // Reduced-cost variable fixing: fix integer variables whose reduced costs
    // prove they can't improve the incumbent.
    // Only available when simplex was used (provides reduced costs + basis).
	    if (opt.enable_reduced_cost_fixing &&
	        !use_ipm_nodes && has_child_upper_limit && child.basis_hint &&
        child.basis_hint->cached_reduced_costs &&
        child.basis_hint->cached_reduced_costs->size() >= n) {
      std::vector<BranchDomainLiteral> rc_forbidden_literals =
          rc_forbidden_literals_preproof;
      Eigen::VectorXd rc_proof_lb = rc_proof_lb_preproof;
      Eigen::VectorXd rc_proof_ub = rc_proof_ub_preproof;
      int rc_fixed = rc_fixed_preproof;
      if (!rc_fixing_preproof_applied) {
        if (opt.enable_reduced_cost_conflict_learning) {
          rc_forbidden_literals.reserve(
              static_cast<std::size_t>(
                  std::max(0, opt.reduced_cost_conflict_max_per_node)));
          if (opt.enable_reduced_cost_proof_conflict_minimization) {
            rc_proof_lb = child.lb;
            rc_proof_ub = child.ub;
          }
        }
        std::vector<DomainReasonBound> rc_fixing_reasons;
		        rc_fixed = reduced_cost_fixing(
		            base_lp.vars, child.x_relax, *child.basis_hint->cached_reduced_costs,
		            child.basis_hint->basis_indices(), child.bound,
		            child_upper_limit_snapshot,
		            opt.int_tol, child.lb, child.ub,
	            opt.enable_reduced_cost_conflict_learning ? &rc_forbidden_literals
	                                                      : nullptr,
	            child.basis_hint->cached_col_scale.get(),
            &rc_fixing_reasons, child.branch_reasons.read_ptr(), child.depth,
            static_cast<int>(child.branch_reasons.size() +
                             child.domain_reason_bounds.size()));
        for (auto& rb : rc_fixing_reasons) {
          append_domain_reason_bound(child, std::move(rb), n, &root.lb,
                                     &root.ub);
        }
	      }
      if (rc_fixed > 0 && !rc_fixing_preproof_applied) {
        out.bc_stats.rc_fixings += static_cast<std::uint64_t>(rc_fixed);
        out.bc_stats.rc_forbidden_literals +=
            static_cast<std::uint64_t>(rc_forbidden_literals.size());
      }
      std::size_t rc_proof_cut_begin = new_cuts.size();
      if (rc_fixed > 0 && opt.enable_reduced_cost_conflict_learning) {
        int rc_conflicts_learned = 0;
        const Eigen::VectorXd& proof_lb =
            rc_proof_lb.size() == n ? rc_proof_lb : child.lb;
        const Eigen::VectorXd& proof_ub =
            rc_proof_ub.size() == n ? rc_proof_ub : child.ub;
        learn_cutoff_conflicts_from_forbidden_literals(
            child.branch_reasons, rc_forbidden_literals,
            proof_lb, proof_ub, child.x_relax,
            *child.basis_hint->cached_reduced_costs,
	            child.basis_hint->basis_indices(), child.bound, inc_obj_snapshot,
	            child.domain_reason_bounds,
	            target_dual_proof,
	            &local_proof_cuts,
            new_cuts, cut_count,
            std::max(0, opt.reduced_cost_conflict_max_per_node),
            rc_conflicts_learned);
        if (rc_conflicts_learned == 0 &&
            opt.enable_verified_reduced_cost_conflict_minimization &&
            rc_verified_conflict_lps_left > 0 &&
            !child.branch_reasons.empty() &&
            !rc_forbidden_literals.empty() &&
            has_inc_snapshot &&
            std::isfinite(inc_obj_snapshot)) {
          const int max_literals =
              std::max(1, opt.reduced_cost_verified_conflict_max_literals);
          const int max_verified =
              std::max(0, opt.reduced_cost_verified_conflict_max_per_node);
          int verified_this_node = 0;

          auto apply_literal_bounds = [](const BranchDomainLiteral& lit,
                                         Eigen::VectorXd& lb,
                                         Eigen::VectorXd& ub) {
            if (lit.var_idx < 0 || lit.var_idx >= lb.size()) return;
            if (lit.is_lb) lb[lit.var_idx] = std::max(lb[lit.var_idx], lit.value);
            else ub[lit.var_idx] = std::min(ub[lit.var_idx], lit.value);
          };

          auto verify_clause_by_cutoff_lp =
              [&](const std::vector<BranchDomainLiteral>& clause) -> bool {
            if (rc_verified_conflict_lps_left <= 0 || clause.empty()) return false;
            Eigen::VectorXd vlb = root.lb;
            Eigen::VectorXd vub = root.ub;
            for (const auto& lit : clause) apply_literal_bounds(lit, vlb, vub);
            if (!bounds_consistent(vlb, vub)) return true;

            StandardFormLP verify_sf = ensure_base_sf();
            update_standard_form_bounds(verify_sf, base_lp, vlb, vub);
            SimplexOptions verify_opt = simplex_opt;
            verify_opt.allow_cold_start = true;
            verify_opt.max_iter = std::max(1000, opt.max_lp_iter);
            auto verify_res = solve_tree_lp(
                verify_sf, verify_opt,
                (root.basis_hint && root.basis_hint->rows == verify_sf.A.rows() &&
                 root.basis_hint->cols == verify_sf.A.cols())
                    ? root.basis_hint.get()
                    : nullptr);
            --rc_verified_conflict_lps_left;
            ++lp_count;
            ++out.bc_stats.rc_conflict_verification_lps;

            const auto vtype = classify_lp_result(verify_res, n);
            if (vtype == LPFailureType::Infeasible ||
                vtype == LPFailureType::ObjectiveCutoff) {
              return true;
            }
            if (vtype == LPFailureType::Success &&
                std::isfinite(verify_res.result.stats.objective) &&
                incumbent_prunes_node(has_child_upper_limit,
                                      child_upper_limit_snapshot,
                                      verify_res.result.stats.objective,
                                      std::max(1e-9, opt.lp_tol))) {
              return true;
            }
            return false;
          };

          auto learn_verified_clause =
              [&](std::vector<BranchDomainLiteral> clause) -> bool {
            canonicalize_branch_literals(clause);
            if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
              return false;
            }
            if (!verify_clause_by_cutoff_lp(clause)) return false;

            const bool pool_added = add_global_conflict(
                clause, "verified_reduced_cost_conflict");
            const bool impl_added =
                add_binary_implications_from_conflict_clause(base_lp.vars,
                                                             clause,
                                                             implication_graph);
            if (impl_added) {
              ++out.bc_stats.rc_binary_implications_learned;
              note_global_domain_learning();
            }
            if (!strict_highs_lp_contract &&
                (pool_added || impl_added) && clause.size() <= 16) {
              PoolCut conflict_cut;
              if (try_build_binary_conflict_cut(base_lp, clause, conflict_cut, 16)) {
                new_cuts.push_back(std::move(conflict_cut));
                ++cut_count;
                ++out.bc_stats.rc_conflict_cuts_added;
              }
            }
            if (pool_added || impl_added) {
              note_global_domain_learning();
              ++out.bc_stats.rc_conflict_clauses_learned;
              ++out.bc_stats.rc_verified_conflict_clauses_learned;
              ++rc_conflicts_learned;
              return true;
            }
            return false;
          };

          std::vector<int> suffix_lengths;
          suffix_lengths.push_back(0);
          suffix_lengths.push_back(1);
          suffix_lengths.push_back(2);
          suffix_lengths.push_back(4);
          suffix_lengths.push_back(max_literals - 1);
          std::sort(suffix_lengths.begin(), suffix_lengths.end());
          suffix_lengths.erase(std::unique(suffix_lengths.begin(),
                                           suffix_lengths.end()),
                               suffix_lengths.end());

          for (const auto& forbidden : rc_forbidden_literals) {
            if (verified_this_node >= max_verified ||
                rc_verified_conflict_lps_left <= 0) {
              break;
            }
            for (int suffix_len : suffix_lengths) {
              if (verified_this_node >= max_verified ||
                  rc_verified_conflict_lps_left <= 0) {
                break;
              }
              if (suffix_len < 0 || suffix_len + 1 > max_literals) continue;
              const int available =
                  static_cast<int>(child.branch_reasons.size());
              const int take = std::min(suffix_len, available);
              std::vector<BranchDomainLiteral> clause;
              clause.reserve(static_cast<std::size_t>(take + 1));
              for (int q = available - take; q < available; ++q) {
                if (q >= 0) clause.push_back(child.branch_reasons[static_cast<std::size_t>(q)]);
              }
              clause.push_back(forbidden);
              if (learn_verified_clause(std::move(clause))) {
                ++verified_this_node;
                break;
              }
            }
          }
        }
        if (rc_conflicts_learned > 0) {
          branch_obs.conflict_score += static_cast<double>(rc_conflicts_learned);
          branch_obs.has_inference = true;
          branch_obs.inference_count += static_cast<double>(rc_fixed);
        }
      }
      if (rc_fixed > 0 && opt.rc_fixing_followup_propagation) {
        std::vector<BoundChangeInfo> rc_prop_changes;
        std::vector<BranchDomainLiteral> rc_conflict;
        int rc_tightenings = 0;
        if (!propagate_node_domain(base_lp, A_row_cached, Aeq_row_cached,
                                   row_index_cached,
                                   child.lb, child.ub,
                                   opt.bound_propagation_rounds,
                                   &conflict_pool, &clique_table,
                                   &implication_graph,
                                   rc_prop_changes,
                                   child.branch_reasons,
                                   &rc_conflict,
                                   &rc_tightenings)) {
          branch_obs.cutoff_count = 1;
          pc_upd.push_back(branch_obs);
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
      }
      if (!bounds_consistent(child.lb, child.ub)) {
        branch_obs.cutoff_count = 1;
        pc_upd.push_back(branch_obs);
        return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
      }
      if (!transformed_frontier_cuts.empty()) {
        (void)append_local_node_cuts(child, transformed_frontier_cuts,
                                     &child.x_relax,
                                     "node_transformed_frontier");
      }
      if (!local_proof_cuts.empty()) {
        (void)append_local_node_cuts(child, local_proof_cuts, &child.x_relax,
                                     "node_local_proof");
      }
      bool rc_resolved_this_node = false;
	      if (allow_native_proof_cut_rows &&
	          rc_proof_cut_resolve_lps_left > 0 &&
          (new_cuts.size() > rc_proof_cut_begin ||
           !child.local_cuts.empty())) {
        std::vector<Eigen::SparseVector<double>> proof_rows;
        std::vector<double> proof_rhs;
        proof_rows.reserve(new_cuts.size() - rc_proof_cut_begin +
                           child.local_cuts.size());
        proof_rhs.reserve(new_cuts.size() - rc_proof_cut_begin +
                          child.local_cuts.size());
        for (const auto& cut : child.local_cuts) {
          if (valid_pool_cut(cut, &child.x_relax,
                             "resolve_local_node_cut") &&
              sparse_dot(cut.coeff, child.x_relax) > cut.rhs + 1e-7) {
            proof_rows.push_back(cut.coeff);
            proof_rhs.push_back(cut.rhs);
          }
        }
        for (std::size_t ci = rc_proof_cut_begin; ci < new_cuts.size(); ++ci) {
          const auto& cut = new_cuts[ci];
          if (valid_pool_cut(cut, &child.x_relax, "resolve_new_proof_cut") &&
              sparse_dot(cut.coeff, child.x_relax) > cut.rhs + 1e-7) {
            proof_rows.push_back(cut.coeff);
            proof_rhs.push_back(cut.rhs);
          }
        }
        if (proof_rows.empty()) {
          rc_resolved_this_node = false;
        } else {
          LPModel proof_lp = build_node_lp_with_local_cuts(child);
          add_sparse_rows_to_lp(proof_lp, proof_rows, proof_rhs);
          apply_tree_ipm_deadline(ipm_cut_solver);
          auto proof_res = ipm_cut_solver.solve_lp(proof_lp, child.x_relax);
          --rc_proof_cut_resolve_lps_left;
          ++lp_count;
          ++out.bc_stats.rc_fixing_resolve_lps;
          rc_resolved_this_node = true;

          const bool proof_lp_infeasible =
              proof_res.stats.has_farkas_certificate ||
              proof_res.stats.status.find("Infeasible") != std::string::npos ||
              proof_res.stats.status.find("infeasible") != std::string::npos;
          if (proof_lp_infeasible) {
            ++out.bc_stats.rc_fixing_resolve_prunes;
            branch_obs.cutoff_count = 1;
            pc_upd.push_back(branch_obs);
            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
          }

          if (proof_res.stats.success &&
              proof_res.x.size() == n &&
              std::isfinite(proof_res.stats.objective)) {
            const double old_bound = child.bound;
            const double resolved_bound =
                std::max(child.bound, proof_res.stats.objective);
            const double lift = std::max(0.0, resolved_bound - old_bound);
            const double transformed_lift_tol = std::max(
                {10.0 * std::max(1e-7, opt.lp_tol),
                 1e-6 * std::max(1.0, std::abs(old_bound)),
                 has_inc_snapshot && std::isfinite(inc_obj_snapshot)
                     ? 1e-3 * std::max(0.0, inc_obj_snapshot - old_bound)
                     : 0.0});
            const bool consume_resolved_relaxation =
                local_transformed_frontier_rows_pending <= 0 ||
                lift > transformed_lift_tol;
            ++out.bc_stats.rc_proof_resolve_lift_samples;
            out.bc_stats.rc_proof_resolve_lift_sum += lift;
            out.bc_stats.rc_proof_resolve_lift_max =
                std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
            if (local_transformed_frontier_rows_pending > 0) {
              ++out.bc_stats.transformed_frontier_resolve_samples;
              out.bc_stats.transformed_frontier_resolve_lift_sum += lift;
              out.bc_stats.transformed_frontier_resolve_lift_max =
                  std::max(out.bc_stats.transformed_frontier_resolve_lift_max,
                           lift);
              if (lift > (local_transformed_frontier_rows_pending > 0
                              ? transformed_lift_tol
                              : std::max(1e-7, opt.lp_tol))) {
                ++out.bc_stats.transformed_frontier_resolve_bound_moves;
              }
            }
            if (consume_resolved_relaxation) {
              child.x_relax = clamp_to_bounds(proof_res.x, child.lb, child.ub);
              child.x_seed = child.x_relax;
              child.bound = resolved_bound;
            }
	          if (child.bound > old_bound + 1e-7) {
	            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
	            branch_obs.has_gain = true;
	            branch_obs.gain = std::max(branch_obs.gain,
	                                       std::max(0.0, child.bound - parent_bound));
	          }
	          if (collect_integral_lp_candidate("rc_fixing_resolve")) {
	            pc_upd.push_back(branch_obs);
	            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	          }
		          if (incumbent_prunes_node(has_child_upper_limit,
		                                    child_upper_limit_snapshot,
		                                    child.bound,
		                                    std::max(1e-9, opt.lp_tol))) {
		            publish_child_bound_exceeding_conflict("rc_proof_resolve_bound");
		            ++out.bc_stats.rc_fixing_resolve_prunes;
	              branch_obs.cutoff_count = 1;
              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
            if (prune_child_by_gap_optimality_limit(
                    "rc_proof_resolve_optimality_limit")) {
              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
          }
        }
      }
      if (!rc_resolved_this_node &&
          opt.enable_reduced_cost_fixing_resolve &&
          rc_fixing_resolve_lps_left > 0 &&
          rc_fixed >= std::max(1, opt.reduced_cost_fixing_resolve_min_fixings)) {
        const double old_bound = child.bound;
        node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
        SimplexOptions rc_resolve_opt = simplex_opt;
        rc_resolve_opt.allow_cold_start = true;
        auto rc_res = solve_tree_lp(node_sf_ref, rc_resolve_opt,
                                    child.basis_hint.get());
        --rc_fixing_resolve_lps_left;
        ++lp_count;
        ++out.bc_stats.rc_fixing_resolve_lps;
        const auto rc_res_type = classify_lp_result(rc_res, n);
	        if (rc_res_type == LPFailureType::Infeasible ||
	            rc_res_type == LPFailureType::ObjectiveCutoff) {
	          if (rc_res_type == LPFailureType::ObjectiveCutoff) {
	            child.bound = std::max(child.bound, rc_res.result.stats.objective);
	          }
	          publish_child_bound_exceeding_conflict("rc_fixing_resolve_cutoff");
	          ++out.bc_stats.rc_fixing_resolve_prunes;
          branch_obs.cutoff_count = 1;
          pc_upd.push_back(branch_obs);
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
        if (rc_res_type == LPFailureType::Success &&
            rc_res.result.x.size() == n &&
            std::isfinite(rc_res.result.stats.objective)) {
          child.x_relax = clamp_to_bounds(rc_res.result.x, child.lb, child.ub);
          child.x_seed = child.x_relax;
          child.bound = std::max(child.bound, rc_res.result.stats.objective);
          child.basis_hint =
              persist_bc_node_basis_from_simplex(rc_res, child.basis_hint);
          if (child.bound > old_bound + 1e-7) {
            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
            branch_obs.has_gain = true;
            branch_obs.gain = std::max(branch_obs.gain,
                                       std::max(0.0, child.bound - parent_bound));
          }
	          if (incumbent_prunes_node(has_child_upper_limit,
	                                    child_upper_limit_snapshot, child.bound,
	                                    std::max(1e-9, opt.lp_tol))) {
	            publish_child_bound_exceeding_conflict("rc_fixing_resolve_bound");
	            ++out.bc_stats.rc_fixing_resolve_prunes;
	            branch_obs.cutoff_count = 1;
            pc_upd.push_back(branch_obs);
            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
          }
          if (prune_child_by_gap_optimality_limit(
                  "rc_fixing_resolve_optimality_limit")) {
            pc_upd.push_back(branch_obs);
            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
          }
        }
      }
    }

    // Pool cut separation.
    // For large LPs (>500 rows) or deep nodes (>3), skip the expensive
    // cold-start re-solve (LPModel copy + StandardFormLP rebuild + LP solve).
    // Root cuts are already in base_lp; pool cuts at deep nodes provide
    // diminishing returns relative to the per-node rebuild cost.
    if (static_cast<int>(base_lp.A.rows()) <= opt.pool_cut_row_threshold && child.depth <= opt.pool_cut_depth_limit) {
      auto violated_cuts = cp_ref.select_violated_and_age(
          child.x_relax, 1e-4, opt.cuts_per_round);
      if (!violated_cuts.empty()) {
        const int pool_budget = static_cast<int>(violated_cuts.size());
        std::vector<Eigen::SparseVector<double>> pool_rows;
        std::vector<double> pool_rhs_vec;
        pool_rows.reserve(pool_budget);
        pool_rhs_vec.reserve(pool_budget);
        for (int pi = 0; pi < pool_budget; ++pi) {
          PoolCut& cut = violated_cuts[static_cast<std::size_t>(pi)];
          pool_rows.push_back(std::move(cut.coeff));
          pool_rhs_vec.push_back(cut.rhs);
        }
        LPModel pool_lp = build_node_lp_with_local_cuts(child);
        add_sparse_rows_to_lp(pool_lp, pool_rows, pool_rhs_vec);
        pool_sep += pool_budget;

        if (use_ipm_nodes && ipm_node_solver) {
          apply_tree_ipm_deadline(*ipm_node_solver);
          auto pool_res = ipm_node_solver->solve_lp(pool_lp, child.x_relax);
          ++lp_count;
          if (pool_res.stats.success && pool_res.x.size() == n) {
	            child.x_relax = clamp_to_bounds(pool_res.x, child.lb, child.ub);
	            child.x_seed = child.x_relax;
	            child.bound = std::max(child.bound, pool_res.stats.objective);
	            if (collect_integral_lp_candidate("pool_cut_ipm_resolve")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
		          if (incumbent_prunes_node(has_child_upper_limit,
		                                    child_upper_limit_snapshot,
		                                    child.bound,
		                                    std::max(1e-9, opt.lp_tol))) {
		            publish_child_bound_exceeding_conflict("pool_cut_ipm_bound");
		              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
            if (prune_child_by_gap_optimality_limit(
                    "pool_cut_ipm_optimality_limit")) {
              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
          }
        } else {
          // Use IPM for pool-cut re-solve: only needs primal for bound
          // tightening, avoids build_standard_form_lp + dense simplex.
          apply_tree_ipm_deadline(ipm_cut_solver);
          auto pool_res = ipm_cut_solver.solve_lp(pool_lp, child.x_relax);
          ++lp_count;
          if (pool_res.stats.success && pool_res.x.size() == n) {
	            child.x_relax = clamp_to_bounds(pool_res.x, child.lb, child.ub);
	            child.x_seed = child.x_relax;
	            child.bound = std::max(child.bound, pool_res.stats.objective);
	            if (collect_integral_lp_candidate("pool_cut_resolve")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
		          if (incumbent_prunes_node(has_child_upper_limit,
		                                    child_upper_limit_snapshot,
		                                    child.bound,
		                                    std::max(1e-9, opt.lp_tol))) {
		            publish_child_bound_exceeding_conflict("pool_cut_resolve_bound");
		              branch_obs.cutoff_count = 1;
	              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
            if (prune_child_by_gap_optimality_limit(
                    "pool_cut_resolve_optimality_limit")) {
              pc_upd.push_back(branch_obs);
              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
            }
          }
        }
      }
    }

    // Skip node cuts for very large IPM-only problems: no simplex basis
    // means Gomory can't fire, and MIR/cover find 0 cuts on UC. The
    // add_cuts scan + potential re-solve costs ~600ms per node.
    const bool skip_node_cuts = use_ipm_nodes && (presolved_m > opt.xlarge_ipm_only_threshold);
    if (opt.cuts != CutType::None && child.depth <= opt.max_cut_depth && !skip_node_cuts) {
      LPModel node_lp = build_node_lp_with_local_cuts(child);
      // For cut generation, pass nullptr for simplex result in IPM mode
      // (basis-free cuts: MIR and cover still work).
      // Pass the per-family efficacy tracker.
      const int added = add_cuts(
          node_lp, child.x_relax, nullptr, opt, opt.cuts_per_round, nullptr,
          &cut_family_tracker, &clique_table, &separator_storage_stats);
      if (added > 0) {
        cut_count += added;
        const int first_added_row = static_cast<int>(node_lp.A.rows()) - added;
        if (first_added_row >= 0 && first_added_row < node_lp.A.rows()) {
          Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = node_lp.A;
          for (int r = first_added_row; r < node_lp.A.rows(); ++r) {
            if (r >= node_lp.b.size()) break;
            PoolCut local_cut;
            local_cut.coeff = Eigen::SparseVector<double>(n);
            local_cut.coeff.reserve(Arow.innerVector(r).nonZeros());
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
                 it; ++it) {
              const int col = static_cast<int>(it.col());
              if (col >= 0 && col < n && std::isfinite(it.value()) &&
                  std::abs(it.value()) > 1e-12) {
                local_cut.coeff.insertBack(col) = it.value();
              }
            }
            local_cut.coeff.finalize();
            local_cut.rhs = node_lp.b[r];
            local_cut.hash = sparse_cut_hash(local_cut.coeff);
            local_cut.norm =
                std::sqrt(std::max(0.0, local_cut.coeff.squaredNorm()));
            local_cut.validity_scope = ValidityScope::LocalNodeCut;
            ++out.bc_stats.local_node_cuts_generated;
            (void)append_unique_local_node_cut(child, std::move(local_cut),
                                               &child.x_relax,
                                               "child_node_cut");
          }
        }
        auto pre_cut_basis = child.basis_hint;

        const int orig = static_cast<int>(base_lp.A.rows());
        const int total_rows = static_cast<int>(node_lp.A.rows());
        (void)orig;
        (void)total_rows;
        // Child-node cuts tighten only the current node re-solve. They are not
        // globally valid once branch bounds are removed, so do not export them
        // to the global cut pool.

        // Always use IPM for child-cut re-solve: only needs primal for
        // bound tightening, avoids build_standard_form_lp + dense simplex.
        {
          apply_tree_ipm_deadline(ipm_cut_solver);
          auto child_cut_res = ipm_cut_solver.solve_lp(node_lp, child.x_relax);
          ++lp_count;
          if (child_cut_res.stats.success && child_cut_res.x.size() == n) {
	            child.x_relax = clamp_to_bounds(child_cut_res.x, child.lb, child.ub);
	            child.x_seed = child.x_relax;
	            child.bound = std::max(child.bound, child_cut_res.stats.objective);
	            if (collect_integral_lp_candidate("child_cut_resolve")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
			          if (incumbent_prunes_node(has_child_upper_limit,
			                                    child_upper_limit_snapshot,
			                                    child.bound,
			                                    std::max(1e-9, opt.lp_tol))) {
			            publish_child_bound_exceeding_conflict("child_cut_resolve_bound");
			              branch_obs.cutoff_count = 1;
		              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	            if (prune_child_by_gap_optimality_limit(
	                    "child_cut_resolve_optimality_limit")) {
	              pc_upd.push_back(branch_obs);
	              return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	            }
	          }
	        }
	      }
    }

    auto _t3b = std::chrono::steady_clock::now();

    // Try rounding heuristic.  Always attempt — the satisfies_with_bounds
    // check is O(nnz) and cheap relative to LP solves.
    if (!strict_highs_lp_contract) {
      const Eigen::VectorXd xr = project_integer_solution(base_lp.vars, child.x_relax,
                                                          child.lb, child.ub);
      if (satisfies_with_bounds(base_lp, xr, child.lb, child.ub, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, xr, base_lp.sense);
        sols.push_back({xr, obj});
      } else if (!has_inc_snapshot && sols.empty() && !use_ipm_nodes && !is_up_branch
                 && rounding_lp_attempts < max_rounding_lp_attempts) {
        // Simple rounding failed and we have no incumbent yet.
        // Fix all integer variables to their rounded values and re-solve the
        // LP for continuous variables.  Progressive iteration limit: shallow
        // depths are unlikely to converge, so use fewer iterations.
        ++rounding_lp_attempts;
        Eigen::VectorXd fix_lb = child.lb, fix_ub = child.ub;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          double rv = std::round(child.x_relax[i]);
          rv = std::min(child.ub[i], std::max(child.lb[i], rv));
          fix_lb[i] = rv;
          fix_ub[i] = rv;
        }
        node_sf_ref = build_node_sf_with_bounds(child, fix_lb, fix_ub);
        node_sf_cuts_augmented = true;
        SimplexOptions round_opt;
        round_opt.lp_kernel_backend = opt.lp_kernel_backend;
        // Progressive iter budget: shallow nodes get fewer iters since the
        // rounding LP rarely converges before enough branching decisions.
        round_opt.max_iter = (child.depth < 6) ? 50 :
                             (child.depth < 11) ? 150 : 500;
        round_opt.allow_cold_start = false;
        round_opt.feasibility_tol = 1e-8;
        round_opt.optimality_tol = 1e-8;
        round_opt.verbose = false;
        round_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
        apply_bc_first_class_simplex_state(round_opt, /*allow_frontier_remap=*/true);
        bool round_deferred = false;
        auto round_res = seq_dispatcher.solve_with_options(
            node_sf_ref, child.basis_hint.get(), round_opt,
            seq_node_id, child.depth, n, round_deferred);
        ++lp_count;
        bool found_int = false;
        (void)found_int;
        if (round_res.result.stats.success && round_res.result.x.size() == n) {
          const Eigen::VectorXd xf = project_integer_solution(base_lp.vars, round_res.result.x,
                                                              fix_lb, fix_ub);
          if (satisfies_with_bounds(base_lp, xf, child.lb, child.ub, kNodeFeasibilityTol)) {
            const double obj = objective_value(base_lp.c, xf, base_lp.sense);
            sols.push_back({xf, obj});
            found_int = true;
            rounding_lp_attempts = 0;  // reset on success
          }
        }
        // Restore node_sf_ref to child's actual bounds for subsequent use.
        node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
      } else if (has_inc_snapshot && sols.empty() && !use_ipm_nodes &&
                 !is_up_branch && incumbent_repair_lps_left > 0 &&
                 incumbent_x.size() == n) {
        Eigen::VectorXd fix_lb = child.lb;
        Eigen::VectorXd fix_ub = child.ub;
        int changed_ints = 0;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          double rv = std::round(child.x_relax[i]);
          rv = std::min(child.ub[i], std::max(child.lb[i], rv));
          if (std::abs(rv - std::round(incumbent_x[i])) > opt.int_tol) {
            ++changed_ints;
          }
          fix_lb[i] = rv;
          fix_ub[i] = rv;
        }
        if (changed_ints > 0) {
          --incumbent_repair_lps_left;
          ++out.bc_stats.incumbent_repair_lp_attempts;
          const auto rep0 = std::chrono::steady_clock::now();
          node_sf_ref = build_node_sf_with_bounds(child, fix_lb, fix_ub);
        node_sf_cuts_augmented = true;
          SimplexOptions repair_opt;
          repair_opt.lp_kernel_backend = opt.lp_kernel_backend;
          repair_opt.max_iter = (child.depth < 16) ? 120 : 300;
          repair_opt.allow_cold_start = false;
          repair_opt.feasibility_tol = 1e-8;
          repair_opt.optimality_tol = 1e-8;
          repair_opt.verbose = false;
          repair_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
          apply_bc_first_class_simplex_state(repair_opt, /*allow_frontier_remap=*/true);
          bool repair_deferred = false;
          auto repair_res = seq_dispatcher.solve_with_options(
              node_sf_ref, child.basis_hint.get(), repair_opt,
              seq_node_id, child.depth, n, repair_deferred);
          ++lp_count;
          const auto rep1 = std::chrono::steady_clock::now();
          out.bc_stats.incumbent_repair_time_ms +=
              std::chrono::duration<double, std::milli>(rep1 - rep0).count();
          if (repair_res.result.stats.success &&
              repair_res.result.x.size() == n) {
            const Eigen::VectorXd xf = project_integer_solution(
                base_lp.vars, repair_res.result.x, fix_lb, fix_ub);
            if (satisfies_with_bounds(base_lp, xf, child.lb, child.ub,
                                      kNodeFeasibilityTol)) {
              const double obj = objective_value(base_lp.c, xf, base_lp.sense);
              const double improve_tol =
                  std::max(1e-6, 1e-10 * std::max(1.0, std::abs(inc_obj_snapshot)));
              if (obj + improve_tol < inc_obj_snapshot) {
                ++out.bc_stats.incumbent_repair_improvements;
                sols.push_back({xf, obj});
              }
            }
          }
          node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
        }
      } else if (!has_inc_snapshot && sols.empty() && use_ipm_nodes && ipm_node_solver
                 && !is_up_branch
                 && rounding_lp_attempts < max_rounding_lp_attempts
                 && (rounding_lp_attempts < 3 || (rounding_lp_attempts % 4 == 0))) {
        // IPM-based rounding repair for xlarge use_ipm_nodes mode.
        // B&C can't find incumbents via simple round(x_relax) when UC has
        // tight coupling. Fix integers to rounded values (UC-aware) and
        // solve the continuous LP with IPM to check feasibility.
        ++rounding_lp_attempts;
        Eigen::VectorXd xr_try = child.x_relax;
        if (reduced_uc_hint.has_value()) {
          uc_round_commitment(xr_try, *reduced_uc_hint, reduced_uc_hint->ng);
        }
        Eigen::VectorXd fix_lb = child.lb, fix_ub = child.ub;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          double rv = std::round(xr_try[i]);
          rv = std::min(child.ub[i], std::max(child.lb[i], rv));
          fix_lb[i] = rv;
          fix_ub[i] = rv;
        }
        SolveResult ipm_repair_res;
        if (child.local_cuts.empty()) {
          apply_tree_ipm_deadline(*ipm_node_solver);
          ipm_repair_res = ipm_node_solver->solve_node_lp(
              base_lp, fix_lb, fix_ub, child.x_seed);
        } else {
          Node repair_node = child;
          repair_node.lb = fix_lb;
          repair_node.ub = fix_ub;
          LPModel repair_lp = build_node_lp_with_local_cuts(repair_node);
          apply_tree_ipm_deadline(ipm_cut_solver);
          ipm_repair_res = ipm_cut_solver.solve_lp(repair_lp, child.x_seed);
        }
        ++lp_count;
        if (ipm_repair_res.stats.success && ipm_repair_res.x.size() == n) {
          Eigen::VectorXd xf = project_integer_solution(base_lp.vars,
                                  ipm_repair_res.x, fix_lb, fix_ub);
          if (satisfies_with_bounds(base_lp, xf, child.lb, child.ub,
                                     kNodeFeasibilityTol)) {
            const double obj = objective_value(base_lp.c, xf, base_lp.sense);
            sols.push_back({xf, obj});
            rounding_lp_attempts = 0;  // reset on success
          }
        }
      }
    }

    // Guided rounding from solution pool (sequential path).
    if (!strict_highs_lp_contract && solution_pool.size() >= 2) {
      Eigen::VectorXd xg = solution_pool.guided_rounding(child.x_relax, base_lp.vars);
      xg = clamp_to_bounds(xg, child.lb, child.ub);
      if (satisfies_with_bounds(base_lp, xg, child.lb, child.ub, kNodeFeasibilityTol)) {
        const double obj = objective_value(base_lp.c, xg, base_lp.sense);
        sols.push_back({xg, obj});
      }
    }

    auto _t4 = std::chrono::steady_clock::now();
    if (opt.verbose && n > 100) {
      // Aggregate NODE-TIMING (print summary every 500 children).
      thread_local struct {
        int64_t bp_us{0}, sf_us{0}, lp_us{0}, cuts_us{0}, round_us{0};
        int count{0};
        void add(int64_t bp, int64_t sf, int64_t lp, int64_t cuts, int64_t round) {
          bp_us += bp; sf_us += sf; lp_us += lp; cuts_us += cuts; round_us += round;
          ++count;
        }
        void dump_if(int /*m*/) {
          if (count > 0 && count % 1000 == 0) {
            fprintf(stderr, "[NODE-TIMING] n=%d avg: bp=%.2f sf=%.2f lp=%.2f cuts=%.2f round=%.2f total=%.2fms\n",
                    count,
                    bp_us * 0.001 / count, sf_us * 0.001 / count,
                    lp_us * 0.001 / count, cuts_us * 0.001 / count,
                    round_us * 0.001 / count,
                    (bp_us + sf_us + lp_us + cuts_us + round_us) * 0.001 / count);
          }
        }
      } nt;
      auto us = [](auto a, auto b) { return std::chrono::duration_cast<std::chrono::microseconds>(b-a).count(); };
      nt.add(us(_t0, _t1), us(_t1, _t2), us(_t2, _t3), us(_t3, _t3b), us(_t3b, _t4));
      nt.dump_if(n);
    }
    if (opt.verbose && n > 10000) {
      fprintf(stderr, "[NODE-TIMING] d=%d bp=%.1f sf=%.1f lp=%.1f cuts=%.1f round=%.1fms up=%d\n",
              child.depth,
              std::chrono::duration<double,std::milli>(_t1-_t0).count(),
              std::chrono::duration<double,std::milli>(_t2-_t1).count(),
              std::chrono::duration<double,std::milli>(_t3-_t2).count(),
              std::chrono::duration<double,std::milli>(_t3b-_t3).count(),
              std::chrono::duration<double,std::milli>(_t4-_t3b).count(),
              is_up_branch ? 1 : 0);
    }

    pc_upd.push_back(branch_obs);

    return {true, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
  };

	  auto replay_child_domain_before_queue =
	      [&](Node& child, bool& valid, const char* source) -> bool {
    if (!valid) return false;
    if (!bounds_consistent(child.lb, child.ub)) {
      valid = false;
      return false;
    }

    const bool needs_replay =
        child.lp_refresh_needed ||
        child.domain_learning_epoch < current_domain_learning_epoch() ||
        child.objective_artifact_epoch < current_objective_artifact_epoch() ||
        child.domain_closure_epoch < current_domain_closure_epoch();
    if (!needs_replay) return true;

    Eigen::VectorXd lb_before = child.lb;
    Eigen::VectorXd ub_before = child.ub;
    std::uint64_t replay_tightened = 0;
    std::uint64_t replay_reasons = 0;
    bool replay_pruned = false;
    const std::vector<DomainReasonBound> no_new_reasons;
    if (!replay_queue_reason_trail_certificates(
            child, no_new_reasons, replay_tightened, replay_reasons,
            replay_pruned) ||
        replay_pruned || !bounds_consistent(child.lb, child.ub)) {
      valid = false;
      ++out.bc_stats.rc_conflict_queue_prunes;
      ++out.bc_stats.certificate_queue_prunes;
      return false;
    }

	    if (conflict_pool.has_conflict(child.lb, child.ub) ||
	        !bounds_consistent(child.lb, child.ub)) {
	      valid = false;
	      ++out.bc_stats.rc_conflict_queue_prunes;
      ++out.bc_stats.certificate_queue_prunes;
      return false;
    }

		    const bool domain_changed =
		        replay_tightened > 0 ||
		        !same_domain_bounds_for_probe(lb_before, ub_before, child.lb,
		                                      child.ub);
		    const bool trail_changed = domain_changed || replay_reasons > 0;
		    if (trail_changed) {
		      rebuild_local_domain_trail(child, n, &root.lb, &root.ub);
		    }
		    if (domain_changed) {
	      if (child.x_seed.size() == n) {
	        child.x_seed = clamp_to_bounds(child.x_seed, child.lb, child.ub);
	      }
	      if (child.x_relax.size() == n) {
	        child.x_relax = clamp_to_bounds(child.x_relax, child.lb, child.ub);
	      }
	      child.lp_refresh_needed = true;
		      if (apply_domain_objective_lower_bound(child, source) >
		          std::max(1e-8, opt.lp_tol)) {
		        child.lp_refresh_needed = true;
		      }
	      out.bc_stats.certificate_domain_fixings +=
	          replay_tightened;
	      if (opt.verbose || trace_bc_timeline) {
	        fmt::print(stderr,
	                   "[B&C-PREQ-REPLAY] source={} depth={} tight={} "
	                   "reasons={} row_tight={} refresh=1\n",
	                   source != nullptr ? source : "child_queue",
	                   child.depth, replay_tightened, replay_reasons,
	                   0);
	      }
	    }
	    child.domain_learning_epoch = current_domain_learning_epoch();
	    child.objective_artifact_epoch = current_objective_artifact_epoch();
	    child.domain_closure_epoch = current_domain_closure_epoch();
	    return true;
	  };

	  auto refresh_child_lp_before_queue =
	      [&](Node& child, bool& valid, const char* source) -> bool {
	    if (!valid || !child.lp_refresh_needed) return valid;
	    if (child.lb.size() != n || child.ub.size() != n ||
	        !bounds_consistent(child.lb, child.ub)) {
	      valid = false;
	      return false;
	    }
	    StandardFormLP refresh_sf = build_node_sf_with_local_cuts(child);
	    SimplexOptions refresh_opt = simplex_opt;
	    refresh_opt.allow_cold_start = true;
	    auto refresh_res =
	        solve_tree_lp(refresh_sf, refresh_opt, child.basis_hint.get());
	    ++out.bc_stats.lp_solves;
	    ++out.bc_stats.certificate_domain_resolve_lps;
	    const auto refresh_type = classify_lp_result(refresh_res, n);
	    if (refresh_type == LPFailureType::Infeasible ||
	        refresh_type == LPFailureType::ObjectiveCutoff) {
	      if (refresh_type == LPFailureType::ObjectiveCutoff) {
	        child.bound =
	            std::max(child.bound, refresh_res.result.stats.objective);
	      }
	      valid = false;
	      ++out.bc_stats.certificate_domain_prunes;
	      ++out.bc_stats.rc_fixing_resolve_prunes;
	      return false;
	    }
	    if (refresh_type != LPFailureType::Success ||
	        refresh_res.result.x.size() != n ||
	        !std::isfinite(refresh_res.result.stats.objective)) {
	      return valid;
	    }
	    const double old_bound = child.bound;
	    child.x_relax =
	        clamp_to_bounds(refresh_res.result.x, child.lb, child.ub);
	    child.x_seed = child.x_relax;
	    child.bound = std::max(child.bound, refresh_res.result.stats.objective);
	    if (child.estimate < child.bound) child.estimate = child.bound;
	    child.basis_hint =
	        persist_bc_node_basis_from_simplex(refresh_res, child.basis_hint);
	    child.lp_refresh_needed = false;
	    child.domain_learning_epoch = current_domain_learning_epoch();
	    child.objective_artifact_epoch = current_objective_artifact_epoch();
	    child.domain_closure_epoch = current_domain_closure_epoch();
	    const double lift = std::max(0.0, child.bound - old_bound);
	    if (lift > std::max(1e-7, opt.lp_tol)) {
	      ++out.bc_stats.rc_fixing_resolve_bound_improvements;
	      ++out.bc_stats.certificate_domain_bound_improvements;
	      out.bc_stats.rc_proof_resolve_lift_sum += lift;
	      out.bc_stats.rc_proof_resolve_lift_max =
	          std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
	    }
		    if (opt.verbose || trace_bc_timeline) {
		      fmt::print(stderr,
		                 "[B&C-PREQ-LP] source={} depth={} bound={:.10g}->{:.10g} "
		                 "lift={:.6g}\n",
		                 source != nullptr ? source : "child_queue",
		                 child.depth, old_bound, child.bound, lift);
		    }
      if (opt.enable_reduced_cost_fixing && has_incumbent &&
          std::isfinite(incumbent_obj) && child.basis_hint &&
          child.basis_hint->cached_reduced_costs &&
          child.basis_hint->cached_reduced_costs->size() >= n) {
        bool rc_pruned = false;
        int rc_fixed_total = 0;
        int rc_passes = 0;
        for (;;) {
          const double upper_limit =
              highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
                                                opt.int_tol);
          if (!std::isfinite(upper_limit)) break;
          std::vector<DomainReasonBound> rc_handoff_reasons;
	          const int rc_fixed = reduced_cost_fixing(
	              base_lp.vars, child.x_relax,
	              *child.basis_hint->cached_reduced_costs,
	              child.basis_hint->basis_indices(), child.bound, upper_limit,
	              opt.int_tol, child.lb, child.ub, nullptr,
	              child.basis_hint->cached_col_scale.get(), &rc_handoff_reasons,
              child.branch_reasons.read_ptr(), child.depth,
              static_cast<int>(child.branch_reasons.size() +
                               child.domain_reason_bounds.size()));
	          if (rc_fixed <= 0) break;
          rc_fixed_total += rc_fixed;
          ++rc_passes;
          out.bc_stats.rc_fixings += static_cast<std::uint64_t>(rc_fixed);
          for (auto& rb : rc_handoff_reasons) {
            append_domain_reason_bound(child, std::move(rb), n, &root.lb,
                                       &root.ub);
          }
	          rebuild_local_domain_trail(child, n, &root.lb, &root.ub);
          std::vector<BoundChangeInfo> rc_changes;
          NodeDomainClosureStats rc_closure_stats;
          const std::vector<DomainReasonBound> no_rc_reasons;
          if (!run_node_domain_closure(
                  child, no_rc_reasons, rc_changes, nullptr, nullptr,
                  incumbent_obj, true, "prequeue_rc_closure",
                  &rc_closure_stats) ||
              !bounds_consistent(child.lb, child.ub)) {
            valid = false;
            rc_pruned = true;
            ++out.bc_stats.certificate_domain_prunes;
            break;
          }
          refresh_sf = build_node_sf_with_local_cuts(child);
          auto rc_res =
              solve_tree_lp(refresh_sf, refresh_opt, child.basis_hint.get());
          ++out.bc_stats.lp_solves;
          ++out.bc_stats.certificate_domain_resolve_lps;
          const auto rc_type = classify_lp_result(rc_res, n);
          if (rc_type == LPFailureType::Infeasible ||
              rc_type == LPFailureType::ObjectiveCutoff) {
            if (rc_type == LPFailureType::ObjectiveCutoff) {
              child.bound = std::max(child.bound, rc_res.result.stats.objective);
            }
            valid = false;
            rc_pruned = true;
            ++out.bc_stats.certificate_domain_prunes;
            ++out.bc_stats.rc_fixing_resolve_prunes;
            break;
          }
          if (rc_type != LPFailureType::Success || rc_res.result.x.size() != n ||
              !std::isfinite(rc_res.result.stats.objective)) {
            child.lp_refresh_needed = true;
            break;
          }
          const double rc_old_bound = child.bound;
          child.x_relax = clamp_to_bounds(rc_res.result.x, child.lb, child.ub);
          child.x_seed = child.x_relax;
          child.bound = std::max(child.bound, rc_res.result.stats.objective);
          if (child.estimate < child.bound) child.estimate = child.bound;
          const double rc_lift = std::max(0.0, child.bound - rc_old_bound);
          if (rc_lift > std::max(1e-7, opt.lp_tol)) {
            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
            ++out.bc_stats.certificate_domain_bound_improvements;
            out.bc_stats.rc_proof_resolve_lift_sum += rc_lift;
            out.bc_stats.rc_proof_resolve_lift_max =
                std::max(out.bc_stats.rc_proof_resolve_lift_max, rc_lift);
          }
          child.basis_hint =
              persist_bc_node_basis_from_simplex(rc_res, child.basis_hint);
          child.lp_refresh_needed = false;
          child.domain_learning_epoch = current_domain_learning_epoch();
          child.objective_artifact_epoch = current_objective_artifact_epoch();
          child.domain_closure_epoch = current_domain_closure_epoch();
        }
        if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) &&
            rc_fixed_total > 0) {
          fmt::print(stderr,
                     "[B&C-RC-FIXPOINT] phase=prequeue_refresh depth={} "
                     "pass={} fix={} bound={:.10g} pruned={}\n",
                     child.depth, rc_passes, rc_fixed_total, child.bound,
                     rc_pruned ? 1 : 0);
        }
        if (rc_pruned) return false;
      }
		    std::vector<int> prequeue_frac;
	    if (!node_fractional_branchable_indices(
	            child,
	            requested_vendored_highs_lp ? std::max(1e-9, opt.lp_tol)
	                                        : opt.int_tol,
	            prequeue_frac)) {
	      Eigen::VectorXd x_int = child.x_relax;
	      round_declared_branchables(x_int, child.lb, child.ub);
	      const double feas_tol =
	          requested_vendored_highs_lp ? std::max(kNodeFeasibilityTol, opt.lp_tol)
	                                      : kNodeFeasibilityTol;
	      if (satisfies_with_bounds(base_lp, x_int, child.lb, child.ub,
	                                feas_tol)) {
	        const double obj = objective_value(base_lp.c, x_int, base_lp.sense);
	        if (adopt_tree_incumbent("prequeue_refresh", x_int, obj,
	                                 child.depth, child.bound)) {
	          if (opt.verbose || trace_bc_timeline) {
	            fmt::print(stderr,
	                       "[B&C LP-INTEGRAL] phase=prequeue_refresh "
	                       "depth={} obj={:.10g} bound={:.10g}\n",
	                       child.depth, obj, child.bound);
	          }
	        }
	        valid = false;
	        return false;
	      }
	    }
	    const double prequeue_upper_limit =
	        has_incumbent && std::isfinite(incumbent_obj)
	            ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                               opt.int_tol)
	            : kInf;
	    if (incumbent_prunes_node(
	            has_incumbent && std::isfinite(prequeue_upper_limit),
	            prequeue_upper_limit, child.bound, std::max(1e-9, opt.lp_tol))) {
	      valid = false;
	      return false;
	    }
	    return valid;
	  };

  // ════════════════════════════════════════════════════════════════════════
  // Main B&C loop — sequential (num_threads==1) or heterogeneous parallel
  // ════════════════════════════════════════════════════════════════════════
  int num_threads = parallel_tree_allowed ? std::max(1, resolved_threads) : 1;
  if (opt.parallel_delay_until_incumbent && !has_incumbent) {
    num_threads = 1;
    if (parallel_tree_allowed) {
      parallel_schedule_reason = "serial_until_incumbent";
    }
  }
  if (opt.verbose && par_mode && num_threads <= 1 && !concurrent_tree_launched) {
    fmt::print(stderr, "[B&C-PAR] parallel tree disabled: reason={}\n",
               parallel_schedule_reason);
  }
  out.bc_stats.parallel_effective_threads = num_threads;
  out.bc_stats.parallel_schedule_reason =
      (num_threads > 1) ? "enabled" : parallel_schedule_reason;

  if (opt.verbose) {
    print_root_phase_timing(t0, t_root0, t_root1, t_cuts_done, t_pump_done,
                            t_prog_done, t_dive_done, t_lns_done, has_incumbent,
                            trace_root_subsolves, root_subsolve_traces,
                            root_frac_bin_count, root_incumbent_source);
  }
	  if (opt.verbose) {
	    const double root_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	    fmt::print(stderr, "[B&C] root done: {:.3f}s inc={} root_bound={:.2f}\n",
	               root_time, has_incumbent ? 1 : 0, root.bound);
	  }

			  // HiGHS consumes a usable incumbent immediately against the live queue:
			  // objective/reduced-cost proof objects are propagation sources, not a
			  // side channel that waits for a later incumbent.  Do the same once the
			  // sequential root node has been inserted, before the first gap check.
			  if (!concurrent_tree_launched && !node_queue.empty()) {
			    const int root_cert_published =
			        publish_root_domain_closure_certificates(
			            "root_pre_tree_domain_closure");
	      if (root_cert_published > 0) {
	        int root_cert_pruned = prune_queue_after_domain_learning(
	            /*force_frontier_drain=*/true,
	            "root_pre_tree_domain_closure");
	        if (!node_queue.empty()) {
	          root_cert_pruned += inherit_root_domain_to_queue(
	              "root_pre_tree_domain_closure",
	              /*force_frontier_drain=*/true);
	          root_cert_pruned += prune_queue_after_domain_learning(
	              /*force_frontier_drain=*/true,
	              "root_pre_tree_domain_inherit");
	        }
	        refresh_queue_lower_bound();
			      if (opt.verbose ||
			          bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
			        fmt::print(stderr,
			                   "[B&C ROOT-CERT-CLOSURE] published={} "
			                   "pruned={} qsize={} qlb={:.10g}\n",
			                   root_cert_published, root_cert_pruned,
			                   node_queue.size(), global_lb);
			      }
			    }
			  }
			  if (!concurrent_tree_launched && has_incumbent && !node_queue.empty()) {
			    int root_fixed_point_pruned = prune_queue_after_domain_learning(
			        /*force_frontier_drain=*/true, "root_incumbent_fixed_point");
	    if (!node_queue.empty()) {
	      root_fixed_point_pruned += inherit_root_domain_to_queue(
	          "root_incumbent_fixed_point", /*force_frontier_drain=*/true);
	      root_fixed_point_pruned += prune_queue_after_domain_learning(
	          /*force_frontier_drain=*/true,
	          "root_incumbent_domain_inherit");
	    }
		    refresh_queue_lower_bound();
		    if (opt.verbose) {
		      fmt::print(stderr,
		                 "[B&C-POSTINC] phase=root_fixed_point pruned={} "
		                 "qsize={} qlb={:.10g} ub={:.10g}\n",
		                 root_fixed_point_pruned, node_queue.size(), global_lb,
		                 incumbent_obj);
		    }
		  }
		  if (!concurrent_tree_launched && has_incumbent && !node_queue.empty() &&
		      (have_cached_queue_dual_proof || have_root_global_dual_proof)) {
		    const int post_root_pruned = consume_cached_dual_proof_after_incumbent();
	    refresh_queue_lower_bound();
	    if (opt.verbose) {
	      fmt::print(stderr,
	                 "[B&C-POSTINC] phase=root_queue_consume pruned={} "
	                 "qsize={} qlb={:.10g} ub={:.10g}\n",
	                 post_root_pruned, node_queue.size(), global_lb,
	                 incumbent_obj);
	    }
	  }
      if (!concurrent_tree_launched && requested_vendored_highs_lp &&
          has_incumbent && !node_queue.empty()) {
        maybe_run_post_incumbent_local_branching(/*depth=*/0);
        refresh_queue_lower_bound();
      }
      if (!concurrent_tree_launched && has_incumbent && !node_queue.empty()) {
        prune_queue_by_gap_optimality_limit("root_gap_limit");
      }

	  // ── Early gap check: skip tree if root heuristic already closed the gap ──
  // For concurrent mode, also pull tree-found incumbent from SharedIncumbent.
  bool skip_tree = root_tree_setup_budget_expired;
	  if (root_tree_setup_budget_expired) {
	    out.stats.status = bc_status::kTimeLimitReached;
	  }
	  if (concurrent_tree_launched && cts && cts->shared_inc.has()) {
	    Eigen::VectorXd tree_x = cts->shared_inc.get_x();
	    const double tree_actual_obj =
	        objective_value(base_lp.c, tree_x, base_lp.sense);
	    if (std::isfinite(tree_actual_obj) &&
	        (!has_incumbent || tree_actual_obj + 1e-9 < incumbent_obj)) {
      if (validate_root_candidate_for_proof("shared_tree_incumbent",
	                                            tree_x)) {
	        has_incumbent = true;
	        incumbent_obj = tree_actual_obj;
	        incumbent_x = std::move(tree_x);
	        note_incumbent_cutoff_learning();
	        incumbent_proof_validated = true;
	        incumbent_proof_validated_obj = incumbent_obj;
	      } else if (!has_incumbent) {
	        invalidate_incumbent_proof_validation();
	      }
	    }
	  }
	  if (has_incumbent) {
	    refresh_queue_lower_bound();
	    const double live_lb = concurrent_tree_launched ? root.bound : global_lb;
	    const double lb = std::isfinite(live_lb) ? live_lb : root.bound;
    if (std::isfinite(lb)) {
      const double denom = std::max(1.0, std::abs(incumbent_obj));
      const double rel_gap = std::max(0.0, (incumbent_obj - lb) / denom);
      out.bc_stats.gap = rel_gap;
      out.bc_stats.best_bound = lb;
	      // Exact optimality: when a certified dual bound meets the incumbent
	      // (rel_gap ~ 0) the incumbent is proven optimal and every open node is
	      // bound-dominated.  require_tree_exhaustion_certificate governs only
	      // *tolerance*-based pruning of within-gap nodes, so it must not block
	      // termination once the gap is exactly closed -- HiGHS/Gurobi likewise
	      // stop as soon as best_bound meets the incumbent.
	      const bool incumbent_proof_valid =
	          current_incumbent_valid_for_proof("early_gap_check");
	      const bool proven_optimal_exact =
	          incumbent_proof_valid && rel_gap <= 1e-9;
	      if (proven_optimal_exact ||
	          (root_gap_certificate_ready && incumbent_proof_valid &&
	           !opt.require_tree_exhaustion_certificate &&
	           rel_gap <= opt.gap_tol)) {
	        out.stats.status = bc_status::kOptimalRootGapClosed;
        skip_tree = true;
      }
    }
  }

  if (skip_tree) {
    // Root closed the gap — nothing to do in either path.
    // If concurrent tree was running, shut it down.
    if (concurrent_tree_launched && cts) {
      cts->shutdown_and_join();
      // Collect any stats accumulated by tree threads.
      out.bc_stats.nodes_explored = cts->atomic_stats.nodes_explored.load();
      out.bc_stats.lp_solves += cts->atomic_stats.lp_solves.load();
      out.bc_stats.cuts_added += cts->atomic_stats.cuts_added.load();
      out.bc_stats.pool_cuts_separated += cts->atomic_stats.pool_cuts_separated.load();
      out.bc_stats.crossover_incumbents += cts->atomic_stats.crossover_incumbents.load();
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
      auto par_fb_sum = cts->fallback_logger.summary();
      out.bc_stats.fallback_events += par_fb_sum.total_events;
      out.bc_stats.fallback_recoveries += par_fb_sum.nodes_recovered;
    }
  } else if (num_threads <= 1) {
  // ── SEQUENTIAL PATH ──────────────────────────────────────────────────────

  // Main-loop timing aggregation.
  thread_local struct {
    int64_t pop_us{0}, frac_us{0}, branch_us{0}, child_us{0}, post_us{0};
    int count{0};
    void dump(bool verbose) {
      if (verbose && count > 0 && count % 500 == 0) {
        fprintf(stderr, "[LOOP-TIMING] N=%d avg: pop=%.2f frac=%.2f branch=%.2f child=%.2f post=%.2fms total=%.2fms\n",
                count,
                pop_us * 0.001 / count, frac_us * 0.001 / count,
                branch_us * 0.001 / count, child_us * 0.001 / count,
                post_us * 0.001 / count,
                (pop_us + frac_us + branch_us + child_us + post_us) * 0.001 / count);
      }
    }
  } loop_timing;
  auto loop_us = [](auto a, auto b) { return std::chrono::duration_cast<std::chrono::microseconds>(b-a).count(); };

  // Gap stagnation tracking: record gap at checkpoint nodes and detect plateau.
  double stall_gap_checkpoint = 1e30;    // gap at last checkpoint
  int    stall_checkpoint_node = 0;      // node count at last checkpoint
  int ft_audit_samples_taken = 0;
  bool proof_tail_mode_was_active = false;

  while (true) {
    const auto loop_t0 = std::chrono::steady_clock::now();
    const auto now = loop_t0;
    const double elapsed = std::chrono::duration<double>(now - t0).count();
    if (elapsed > opt.time_limit_sec) {
      out.stats.status = bc_status::kTimeLimitReached;
      break;
    }
    if (out.bc_stats.nodes_explored >= opt.max_nodes) {
      out.stats.status = bc_status::kNodeLimitReached;
      break;
    }
    if (maybe_restart_tree_after_incumbent()) {
      continue;
    }
    if (use_ipm_nodes && out.bc_stats.nodes_explored > 0 &&
        !node_queue.empty()) {
      const double pair_budget =
          bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
      const double pair_estimate =
          2.2 * std::max(0.05, root_relax.primal.stats.runtime_sec);
      if (pair_budget < pair_estimate) {
        refresh_queue_lower_bound();
        out.stats.status = bc_status::kTimeLimitReached;
        if (opt.verbose || trace_bc_timeline) {
          fmt::print(stderr,
                     "[B&C-IPM-ADMIT] stop before pop: remaining={:.3f}s "
                     "pair_estimate={:.3f}s qsize={} qlb={:.10g}\n",
                     pair_budget, pair_estimate, node_queue.size(), global_lb);
        }
        break;
      }
    }

    const bool proof_tail_mode_now = proof_tail_queue_mode_active();
	    if (proof_tail_mode_now && !proof_tail_mode_was_active) {
      const bool cached_proof_already_consumed =
          cached_dual_proof_consumed_for_current_epoch();
      if (!cached_proof_already_consumed) {
        forced_certificate_queue_passes =
            std::max(forced_certificate_queue_passes, 1);
      }
	      const int tail_pruned =
	          cached_proof_already_consumed
	              ? 0
	              : highs_style_post_incumbent_domain_closure(
	                    "proof_tail_activate", out.bc_stats.last_incumbent_depth);
      const bool has_new_replay_learning =
          current_domain_closure_epoch() > last_queue_domain_replay_epoch;
		      const int replay_pruned = has_new_replay_learning
              ? prune_queue_after_domain_learning(
		            /*force_frontier_drain=*/true, "proof_tail_activate")
              : 0;
          const int gap_pruned =
              prune_queue_by_gap_optimality_limit("proof_tail_activate");
	      refresh_queue_lower_bound();
	      if (opt.verbose) {
	        fmt::print(stderr,
	                   "[B&C-PROOF-TAIL] activated node={} last_inc={} "
	                   "qsize={} qlb={:.10g} ub={:.10g} pruned={}\n",
	                   out.bc_stats.nodes_explored,
	                   out.bc_stats.last_incumbent_node, node_queue.size(),
	                   global_lb, incumbent_obj,
                       tail_pruned + replay_pruned + gap_pruned);
	      }
          print_bc_timeline_state("proof_tail_activate", "tail_gate",
                                  node_queue.lower_bound(), node_queue.size(),
                                  tail_pruned + replay_pruned + gap_pruned);
	    }
    proof_tail_mode_was_active = proof_tail_mode_now;

    if (proof_tail_mode_now) {
      int frontier_contract_pruned = 0;
      for (int guard = 0; guard < 8; ++guard) {
        const double refresh_band =
            std::max(termination_bound_band(), 2.0 * opt.lp_tol);
        const int marked_frontier_nodes =
            node_queue.count_lp_refresh_needed_near_lower_bound(
                refresh_band, /*max_scan=*/0);
        if (marked_frontier_nodes <= 0) break;
	        frontier_contract_pruned += refresh_marked_queue_nodes(
	            "proof_tail_frontier_contract", marked_frontier_nodes,
	            refresh_band, /*mandatory_frontier_contract=*/true);
	        if (last_marked_queue_refresh_attempts <= 0 ||
	            last_marked_queue_refresh_remaining <= 0 ||
	            (last_marked_queue_refresh_pruned <= 0 &&
	             last_marked_queue_refresh_bound_moves <= 0 &&
	             last_marked_queue_refresh_queue_lift <=
	                 std::max(1e-8, opt.lp_tol))) {
	          break;
	        }
	      }
      if (frontier_contract_pruned > 0) {
        refresh_queue_lower_bound();
        const double frontier_upper_limit =
            has_incumbent && std::isfinite(incumbent_obj)
                ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
                                                   opt.int_tol)
                : kInf;
        if (incumbent_prunes_node(
                has_incumbent && std::isfinite(frontier_upper_limit),
                frontier_upper_limit, node_queue.lower_bound(),
                std::max(1e-9, opt.lp_tol))) {
          (void)prune_queue_by_incumbent();
          refresh_queue_lower_bound();
        }
      }
    }

    Node cur;
    const bool proof_queue_mode = hybrid_queue_mode_active();
	    const bool popped_node = proof_tail_mode_now
	                                 ? node_queue.pop_bestbound(cur)
	                                 : node_queue.pop(cur, proof_queue_mode);
	    if (!popped_node) {
          refresh_queue_lower_bound();
          if (has_incumbent && std::isfinite(global_lb)) {
            const double denom = std::max(1.0, std::abs(incumbent_obj));
            const double rel_gap =
                std::max(0.0, (incumbent_obj - global_lb) / denom);
            out.bc_stats.gap = rel_gap;
            out.stats.status = !opt.require_tree_exhaustion_certificate &&
                                       root_gap_certificate_ready &&
                                       rel_gap <= opt.gap_tol
                                   ? bc_status::kOptimalityGapReached
                                   : bc_status::kSearchQueueExhausted;
          } else {
            out.stats.status = bc_status::kSearchQueueExhausted;
          }
	      break;
	    }
    if (cur.x_relax.size() != n) {
      cur.lp_refresh_needed = true;
      cur.fractional_branchables_valid = false;
    }
    if (conflict_pool.has_conflict(cur.lb, cur.ub)) {
      ++out.bc_stats.rc_conflict_queue_prunes;
      continue;
    }
	    const double cur_upper_limit =
	        has_incumbent && std::isfinite(incumbent_obj)
	            ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                               opt.int_tol)
	            : kInf;
	    if (incumbent_prunes_node(
	            has_incumbent && std::isfinite(cur_upper_limit),
	            cur_upper_limit, cur.bound, std::max(1e-9, opt.lp_tol))) {
	      continue;
	    }
        if (!opt.require_tree_exhaustion_certificate) {
          const double gap_limit = gap_optimality_limit();
          const double gap_limit_tol = gap_optimality_prune_tol(gap_limit);
          if (std::isfinite(gap_limit) && std::isfinite(cur.bound) &&
              cur.bound >= gap_limit - gap_limit_tol) {
            gap_limit_lower_bound_certificate =
                std::max(gap_limit_lower_bound_certificate, gap_limit);
            ++out.bc_stats.gap_suboptimal_queue_prunes;
            continue;
          }
        }

    // Clauses learned after a node entered the queue must still be first-class
    // propagation objects.  Apply them when the node is popped, and only commit
    // tightened domains after a refreshed LP proves the node relaxation under
    // those domains.  This keeps the old basis/x_relax from being used with a
    // stronger domain than the LP that produced it.
    const bool late_replay_has_new_learning =
        cur.lp_refresh_needed ||
        cur.domain_learning_epoch < current_domain_learning_epoch() ||
        cur.objective_artifact_epoch < current_objective_artifact_epoch() ||
        cur.domain_closure_epoch < current_domain_closure_epoch();
			    if ((cur.lp_refresh_needed ||
			         (allow_certificate_domain_replay &&
			          (!conflict_pool.empty() || !implication_graph.empty() ||
			           cut_pool.size() > 0 ||
			           !global_bound_lifting_certificates.empty()))) &&
			        late_replay_has_new_learning) {
		      Node replay_node = cur;
		      std::vector<BoundChangeInfo> late_prop_changes;
	      NodeDomainClosureStats late_closure_stats;
	      const std::vector<DomainReasonBound> no_late_reasons;
	      bool prop_ok = run_node_domain_closure(
	          replay_node, no_late_reasons, late_prop_changes, nullptr,
	          nullptr, has_incumbent ? incumbent_obj : kInf, has_incumbent,
	          "pop_late_replay", &late_closure_stats);
		      if (!prop_ok) {
	        ++out.bc_stats.rc_conflict_queue_prunes;
	        continue;
	      }
	      if (!bounds_consistent(replay_node.lb, replay_node.ub)) {
	        ++out.bc_stats.rc_conflict_queue_prunes;
	        continue;
	      }
		      const bool replay_domain_changed =
		          !late_prop_changes.empty() ||
		          replay_node.domain_reason_bounds.size() !=
		              cur.domain_reason_bounds.size() ||
		          replay_node.scoped_conflict_clauses.size() !=
		              cur.scoped_conflict_clauses.size() ||
	          replay_node.local_conflict_clauses.size() !=
		              cur.local_conflict_clauses.size() ||
		          replay_node.local_binary_implications.size() !=
		              cur.local_binary_implications.size();
			      if ((!late_prop_changes.empty() ||
			           cur.lp_refresh_needed) &&
		          bounds_consistent(replay_node.lb, replay_node.ub)) {
	        node_sf = build_node_sf_with_local_cuts(replay_node);
        node_sf_cuts_augmented = true;
	        SimplexOptions late_opt = simplex_opt;
	        late_opt.allow_cold_start = true;
	        auto late_res = solve_tree_lp(node_sf, late_opt, cur.basis_hint.get());
	        if (rc_proof_cut_resolve_lps_left > 0) {
	          --rc_proof_cut_resolve_lps_left;
	        }
        ++out.bc_stats.lp_solves;
        ++out.bc_stats.rc_fixing_resolve_lps;
        const auto late_type = classify_lp_result(late_res, n);
        if (late_type == LPFailureType::Infeasible ||
            late_type == LPFailureType::ObjectiveCutoff) {
          if (late_type == LPFailureType::ObjectiveCutoff) {
            cur.bound = std::max(cur.bound, late_res.result.stats.objective);
          }
          ++out.bc_stats.rc_fixing_resolve_prunes;
          ++out.bc_stats.rc_conflict_queue_prunes;
          continue;
        }
	        if (late_type == LPFailureType::Success &&
	            late_res.result.x.size() == n &&
	            std::isfinite(late_res.result.stats.objective)) {
	          const double old_bound = cur.bound;
	          cur = std::move(replay_node);
	          cur.fractional_branchables_valid = false;
		          cur.x_relax = clamp_to_bounds(late_res.result.x, cur.lb, cur.ub);
	          cur.x_seed = cur.x_relax;
	          cur.bound = std::max(cur.bound, late_res.result.stats.objective);
	          {
	            const double lift = std::max(0.0, cur.bound - old_bound);
	            ++out.bc_stats.rc_proof_resolve_lift_samples;
	            out.bc_stats.rc_proof_resolve_lift_sum += lift;
	            out.bc_stats.rc_proof_resolve_lift_max =
	                std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
	          }
	          cur.basis_hint =
	              persist_bc_node_basis_from_simplex(late_res, cur.basis_hint);
          if (cur.bound > old_bound + 1e-7) {
            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
          }
	          if (incumbent_prunes_node(
	                  has_incumbent && std::isfinite(cur_upper_limit),
	                  cur_upper_limit, cur.bound,
	                  std::max(1e-9, opt.lp_tol))) {
	            ++out.bc_stats.rc_fixing_resolve_prunes;
	            continue;
	          }
	        } else {
              if (simplex_node_deadline_hit || bc_time_limit_expired()) {
                // Genuine wall/node deadline: stop the tree search cleanly and
                // preserve the node for a later resume.
                replay_node.lp_refresh_needed = true;
                replay_node.fractional_branchables_valid = false;
                node_queue.push_priority(std::move(replay_node));
                out.stats.status = bc_status::kTimeLimitReached;
                out.bc_stats.status = out.stats.status;
                break;
              }
              // Non-deadline hard-LP outcome (e.g. the dual simplex exhausting
              // its anti-cycling ladder on a numerically degenerate node, or an
              // iteration-limit stop): this OPTIONAL reduced-cost / certificate
              // resolve could not reach optimality. Abandoning the whole tree
              // search here would forfeit every other queued node, so instead
              // skip the resolve and branch on `cur` with its existing valid
              // relaxation and bound. cur.bound remains a sound lower bound and
              // the learned conflicts/cuts stay in the global pools for the
              // children; only this one node is left un-tightened. Control falls
              // through to the lp_refresh_needed = false reset below, so the
              // node is branched (not re-queued for a resolve that would fail
              // identically). node_sf is a scratch buffer rebuilt before its
              // next use, so leaving it in replay_node's form is harmless.
	        }
	      } else if (replay_domain_changed &&
	                 bounds_consistent(replay_node.lb, replay_node.ub)) {
	        cur = std::move(replay_node);
	        cur.fractional_branchables_valid = false;
	      }
      cur.domain_learning_epoch = current_domain_learning_epoch();
      cur.objective_artifact_epoch = current_objective_artifact_epoch();
      cur.domain_closure_epoch = current_domain_closure_epoch();
      cur.lp_refresh_needed = false;
		    } else if ((!conflict_pool.empty() || !implication_graph.empty() ||
		                cut_pool.size() > 0 ||
		                !global_bound_lifting_certificates.empty()) &&
		               allow_certificate_domain_replay &&
		               !late_replay_has_new_learning) {
	      ++out.bc_stats.late_replay_skipped_no_new_learning;
	    }

    if (allow_native_proof_cut_rows &&
        rc_proof_cut_resolve_lps_left > 0 &&
        cut_pool.size() > 0) {
      auto violated_cuts = cut_pool.select_violated_and_age(
          cur.x_relax, 1e-4, opt.cuts_per_round);
      if (!violated_cuts.empty()) {
        const int pool_budget = static_cast<int>(violated_cuts.size());
        LPModel pool_lp = build_node_lp_with_local_cuts(cur);
        std::vector<Eigen::SparseVector<double>> pool_rows;
        std::vector<double> pool_rhs_vec;
        pool_rows.reserve(pool_budget);
        pool_rhs_vec.reserve(pool_budget);
        for (int pi = 0; pi < pool_budget; ++pi) {
          PoolCut& pc_cut = violated_cuts[static_cast<std::size_t>(pi)];
          pool_rows.push_back(std::move(pc_cut.coeff));
          pool_rhs_vec.push_back(pc_cut.rhs);
        }
        add_sparse_rows_to_lp(pool_lp, pool_rows, pool_rhs_vec);
        apply_tree_ipm_deadline(ipm_cut_solver);
        auto pool_res = ipm_cut_solver.solve_lp(pool_lp, cur.x_relax);
        --rc_proof_cut_resolve_lps_left;
        ++out.bc_stats.lp_solves;
        out.bc_stats.pool_cuts_separated += pool_budget;
        ++out.bc_stats.rc_fixing_resolve_lps;
        if (pool_res.stats.success && pool_res.x.size() == n &&
            std::isfinite(pool_res.stats.objective)) {
          const double old_bound = cur.bound;
	          cur.x_relax = clamp_to_bounds(pool_res.x, cur.lb, cur.ub);
	          cur.fractional_branchables_valid = false;
	          cur.x_seed = cur.x_relax;
	          cur.bound = std::max(cur.bound, pool_res.stats.objective);
	          {
	            const double lift = std::max(0.0, cur.bound - old_bound);
	            ++out.bc_stats.rc_proof_resolve_lift_samples;
	            out.bc_stats.rc_proof_resolve_lift_sum += lift;
	            out.bc_stats.rc_proof_resolve_lift_max =
	                std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
	          }
	          if (cur.bound > old_bound + 1e-7) {
            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
          }
          if (incumbent_prunes_node(
                  has_incumbent && std::isfinite(cur_upper_limit),
                  cur_upper_limit, cur.bound, std::max(1e-9, opt.lp_tol))) {
            ++out.bc_stats.rc_fixing_resolve_prunes;
            continue;
          }
        }
      }
    }

    ++out.bc_stats.nodes_explored;

    if (opt.verbose && (out.bc_stats.nodes_explored % 500 == 0 || out.bc_stats.nodes_explored <= 5)) {
      std::string inc_msg = has_incumbent ? fmt::format(" obj={:.2f}", incumbent_obj) : "";
      MIPSOLVERS_LOG_DEBUG("[B&C] node={} d={} bnd={:.2f} q={}+{} inc={}{}",
              out.bc_stats.nodes_explored, cur.depth, cur.bound,
              node_queue.priority_size(), node_queue.dfs_size(),
              has_incumbent ? 1 : 0, inc_msg);
    }

    std::vector<int> frac;
		    if (!node_fractional_branchable_indices(cur, opt.int_tol, frac)) {
	      if (satisfies_with_bounds(base_lp, cur.x_relax, cur.lb, cur.ub, 1e-6)) {
	        const double obj = objective_value(base_lp.c, cur.x_relax, base_lp.sense);
	        if (adopt_tree_incumbent("tree_integral_node", cur.x_relax, obj,
	                                 cur.depth, global_lb)) {
	          if (opt.verbose && !has_incumbent)
	            fprintf(stderr, "[B&C] first incumbent at node=%d obj=%.2f\n",
	                    static_cast<int>(out.bc_stats.nodes_explored), obj);
		          highs_style_post_incumbent_domain_closure(
		              "tree_integral_node", cur.depth);
	          maybe_run_post_incumbent_local_branching(cur.depth);
	        }
	      }
	      continue;
	    }

    // Priority-based candidate scoring: when branching priorities are
    // set, pass them to pseudocost scoring for soft priority weighting.
    // (Hard filtering removed — it caused excess numerical failures.)
    const auto loop_t1 = std::chrono::steady_clock::now();

    struct BranchPriorContextState {
      BCBranchContext ctx;
      std::vector<int> original_candidates;
      Eigen::VectorXd original_lp_x;
    };
    auto make_branch_context = [&](const std::vector<int>& candidates) {
      BranchPriorContextState state;
      state.ctx.node_id = static_cast<int>(out.bc_stats.nodes_explored);
      state.ctx.depth = cur.depth;
      state.ctx.node_lb = cur.bound;
      state.ctx.best_obj = has_incumbent
                               ? incumbent_obj
                               : std::numeric_limits<double>::infinity();
      state.ctx.candidates = &candidates;
      state.ctx.lp_x = cur.x_relax.data();
      state.ctx.lp_x_size = static_cast<std::size_t>(cur.x_relax.size());
#ifdef MIPSOLVERS_HAVE_PAPILO
      if (papilo_ps.success && !papilo_ps.reduced_to_orig_col.empty()) {
        const auto& reduced_to_orig = papilo_ps.reduced_to_orig_col;
        bool mapping_ok = true;
        state.original_candidates.reserve(candidates.size());
        for (int red_col : candidates) {
          if (red_col < 0 || red_col >= static_cast<int>(reduced_to_orig.size())) {
            mapping_ok = false;
            break;
          }
          const int orig_col = reduced_to_orig[static_cast<std::size_t>(red_col)];
          if (orig_col < 0 || orig_col >= orig_n) {
            mapping_ok = false;
            break;
          }
          state.original_candidates.push_back(orig_col);
        }
        if (mapping_ok && state.original_candidates.size() == candidates.size()) {
          state.original_lp_x = Eigen::VectorXd::Zero(orig_n);
          const int n_red = std::min(static_cast<int>(cur.x_relax.size()),
                                     static_cast<int>(reduced_to_orig.size()));
          for (int red_col = 0; red_col < n_red; ++red_col) {
            const int orig_col = reduced_to_orig[static_cast<std::size_t>(red_col)];
            if (orig_col >= 0 && orig_col < orig_n) {
              state.original_lp_x[orig_col] = cur.x_relax[red_col];
            }
          }
          state.ctx.candidates = &state.original_candidates;
          state.ctx.lp_x = state.original_lp_x.data();
          state.ctx.lp_x_size = static_cast<std::size_t>(state.original_lp_x.size());
        }
      }
#endif
      return state;
    };
    auto choose_branch_var_for_node = [&](const std::vector<int>& candidates) {
      if (callbacks && callbacks->branching_prior) {
        auto branch_context = make_branch_context(candidates);
        return choose_branch_var(opt, candidates, cur.x_relax, pc,
                                 branch_priority, callbacks->branching_prior,
                                 branch_context.ctx);
      }
      return choose_branch_var(opt, candidates, cur.x_relax, pc, branch_priority);
    };
    auto choose_pseudocost_branch_var_for_node =
        [&](const std::vector<int>& candidates) {
      if (callbacks && callbacks->branching_prior) {
        auto branch_context = make_branch_context(candidates);
        return choose_branch_var_pseudocost(
            candidates, cur.x_relax, pc, branch_priority,
            callbacks->branching_prior, branch_context.ctx);
      }
      return branch_priority.empty()
                 ? choose_branch_var_pseudocost(candidates, cur.x_relax, pc)
                 : choose_branch_var_pseudocost(candidates, cur.x_relax, pc,
                                                branch_priority);
    };

	    int j = -1;
	    StrongBranchProbeResult retained_down_probe;
	    StrongBranchProbeResult retained_up_probe;
	    SparseFactorTelemetry live_factor_telemetry{};
		    {
		      int local_lp_solves = 0;
		      int local_strong_branch_candidates = 0;
		      std::uint64_t local_probe_lp_iterations = 0;
		      StrongBranchProbeStorageStats local_probe_storage;
		      int initial_probe_winner = -1;
		      const auto strong_probe_start = std::chrono::steady_clock::now();
		      bool incumbent_changed_by_probe = false;
		      auto observations_from_probes =
		          [](const std::vector<StrongBranchProbeResult>& probes) {
		        std::vector<BranchDirectionalObservation> observations;
		        observations.reserve(probes.size());
		        for (const auto& probe : probes) {
		          if (!probe.available) continue;
		          observations.push_back({probe.var, probe.is_up,
		                                  probe.evidence_state,
		                                  probe.observed_gain});
		        }
		        return observations;
		      };
		      auto choose_with_probe_overlay =
		          [&](const std::vector<int>& candidates,
		              const std::vector<StrongBranchProbeResult>& probes) {
		        const auto observations = observations_from_probes(probes);
		        if (callbacks && callbacks->branching_prior) {
		          auto branch_context = make_branch_context(candidates);
		          return choose_branch_var_pseudocost_with_overlay(
		              candidates, cur.x_relax, pc, branch_priority, observations,
		              callbacks->branching_prior, branch_context.ctx);
		        }
		        return choose_branch_var_pseudocost_with_overlay(
		            candidates, cur.x_relax, pc, branch_priority, observations);
		      };
		      auto record_probe_selection_telemetry =
		          [&](const std::vector<StrongBranchProbeResult>& probes,
		              int selected) {
		        for (const auto& down : probes) {
		          if (!down.available || down.is_up) continue;
		          const auto up = std::find_if(
		              probes.begin(), probes.end(), [&](const auto& probe) {
		                return probe.available && probe.is_up &&
		                       probe.var == down.var;
		              });
		          if (up == probes.end()) continue;
		          const auto complete = [](BranchEvidenceState state) {
		            return state != BranchEvidenceState::Predicted &&
		                   state != BranchEvidenceState::UnknownFailure;
		          };
		          if (complete(down.evidence_state) &&
		              complete(up->evidence_state)) {
		            ++out.bc_stats.strong_complete_probe_pairs;
		          }
		        }
		        out.bc_stats.strong_probe_unknown_failures +=
		            static_cast<std::uint64_t>(std::count_if(
		                probes.begin(), probes.end(), [](const auto& probe) {
		                  return probe.evidence_state ==
		                         BranchEvidenceState::UnknownFailure;
		                }));
		        const auto selected_down = std::find_if(
		            probes.begin(), probes.end(), [&](const auto& probe) {
		              return probe.available && !probe.is_up &&
		                     probe.var == selected;
		            });
		        const auto selected_up = std::find_if(
		            probes.begin(), probes.end(), [&](const auto& probe) {
		              return probe.available && probe.is_up &&
		                     probe.var == selected;
		            });
		        if (!probes.empty() && selected_down == probes.end() &&
		            selected_up == probes.end()) {
		          ++out.bc_stats.strong_selected_unprobed;
		          return;
		        }
		        const bool selected_exact =
		            selected_down != probes.end() && selected_up != probes.end() &&
		            selected_down->evidence_state ==
		                BranchEvidenceState::ExactOptimal &&
		            selected_up->evidence_state ==
		                BranchEvidenceState::ExactOptimal;
		        if (selected_exact) ++out.bc_stats.strong_selected_exact;
		        const auto usable = [](BranchEvidenceState state) {
		          return state == BranchEvidenceState::Measured ||
		                 state == BranchEvidenceState::ExactOptimal;
		        };
		        if (initial_probe_winner >= 0 && selected != initial_probe_winner &&
		            selected_down != probes.end() && selected_up != probes.end() &&
		            usable(selected_down->evidence_state) &&
		            usable(selected_up->evidence_state)) {
		          ++out.bc_stats.strong_winner_changed_by_exact;
		        }
		      };
		      auto record_strong_branch_regret =
		          [&](const std::vector<StrongBranchProbeResult>& probes,
		              int selected) {
		        const auto observations = observations_from_probes(probes);
		        double best = -1.0;
		        double chosen = -1.0;
		        for (const auto& down : probes) {
		          if (!down.available || down.is_up) continue;
		          const auto up = std::find_if(
		              probes.begin(), probes.end(), [&](const auto& pr) {
		                return pr.available && pr.is_up && pr.var == down.var;
		              });
		          if (up == probes.end()) continue;
		          const bool down_usable =
		              down.evidence_state == BranchEvidenceState::Measured ||
		              down.evidence_state == BranchEvidenceState::ExactOptimal;
		          const bool up_usable =
		              up->evidence_state == BranchEvidenceState::Measured ||
		              up->evidence_state == BranchEvidenceState::ExactOptimal;
		          if (!down_usable || !up_usable) continue;
		          const double score = compute_branch_var_score_with_overlay(
		              down.var, cur.x_relax, pc, observations);
		          best = std::max(best, score);
		          if (down.var == selected) chosen = score;
		        }
		        if (best >= 0.0 && chosen >= 0.0) {
		          const double regret = std::max(0.0, best - chosen);
		          ++out.bc_stats.strong_branch_regret_samples;
		          out.bc_stats.strong_branch_regret_sum += regret;
		          out.bc_stats.strong_branch_regret_max =
		              std::max(out.bc_stats.strong_branch_regret_max, regret);
		        }
		      };
	      if (opt.branching == BranchingStrategy::Pseudocost) {
	        if (proof_tail_mode_now && has_incumbent) {
            j = choose_branch_var_for_node(frac);
	        } else
        // When using IPM for node solves, skip expensive simplex probing
        // and use IPM-based probing or pure pseudocost instead.
        if (use_ipm_nodes && ipm_node_solver) {
          // IPM-based reliability branching. Retain the selected candidate's
          // two solved child states so normal child processing consumes them
          // instead of immediately repeating the same IPM solves.
          const bool allow_tree_probe = !(disable_uc_tree_probing && cur.depth > 0);
          const double probe_budget_remaining =
              bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
          const double measured_ipm_solve_sec = std::max(
              0.05, root_relax.primal.stats.runtime_sec);
          // Admit only complete down/up pairs. A bound-change solve is often
          // cheaper than the cold root, but reserving 1.1x the measured root
          // time avoids splitting a short batch into uniformly useless
          // sub-second attempts on large degenerate roots.
          const int root_batch_candidate_capacity =
              probe_budget_remaining > 0.0

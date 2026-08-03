// @file bc_run/11_node_domain_conflict_b.inc
// BCSolveState::run() segment 11/14: node domain propagation / conflict learning — part 2, queue management.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
	        continue;
	      }
	      local_cut.hash = h;
	      ++out.bc_stats.local_node_cuts_generated;
	      local_proof_cuts.push_back(
	          make_local_node_cut(std::move(local_cut)));
	      ++rows;
	      if (rows >= max_rows) break;
		    }
		    return rows;
		  };

		  auto propagate_local_conflict_clauses =
		      [&](Node& node, std::vector<BoundChangeInfo>& changes_out) -> bool {
    if (node.local_conflict_clauses.empty()) return true;
    const double tol = std::max(1e-9, opt.int_tol);
    bool changed = true;
    while (changed) {
      changed = false;
      for (const auto& clause : node.local_conflict_clauses) {
        int unsatisfied_count = 0;
        const BranchDomainLiteral* remaining = nullptr;
        for (const auto& lit : clause) {
          if (lit.var_idx < 0 || lit.var_idx >= n) {
            unsatisfied_count = 2;
            break;
          }
          const bool active = lit.is_lb
              ? (node.lb[lit.var_idx] >= lit.value - tol)
              : (node.ub[lit.var_idx] <= lit.value + tol);
          if (!active) {
            ++unsatisfied_count;
            remaining = &lit;
            if (unsatisfied_count > 1) break;
          }
        }
        if (unsatisfied_count == 0) {
          ++out.bc_stats.local_conflict_clause_prunes;
          return false;
        }
        if (unsatisfied_count != 1 || remaining == nullptr) continue;
        const int j = remaining->var_idx;
        const bool integer_var =
            is_integer_type(base_lp.vars[static_cast<std::size_t>(j)]);
        if (remaining->is_lb) {
          const double new_ub =
              integer_var ? std::ceil(remaining->value - tol) - 1.0
                          : remaining->value - 1e-7;
          if (new_ub < node.lb[j] - tol) {
            ++out.bc_stats.local_conflict_clause_prunes;
            return false;
          }
          if (new_ub < node.ub[j] - tol) {
            const double old = node.ub[j];
            node.ub[j] = new_ub;
            changes_out.push_back({j, new_ub - old, false, old, new_ub});
            ++out.bc_stats.local_conflict_clause_tightenings;
            changed = true;
          }
        } else {
          const double new_lb =
              integer_var ? std::floor(remaining->value + tol) + 1.0
                          : remaining->value + 1e-7;
          if (new_lb > node.ub[j] + tol) {
            ++out.bc_stats.local_conflict_clause_prunes;
            return false;
          }
          if (new_lb > node.lb[j] + tol) {
            const double old = node.lb[j];
            node.lb[j] = new_lb;
            changes_out.push_back({j, new_lb - old, true, old, new_lb});
            ++out.bc_stats.local_conflict_clause_tightenings;
            changed = true;
          }
        }
      }
    }
    return bounds_consistent(node.lb, node.ub);
  };

	  auto propagate_local_binary_implications =
	      [&](Node& node, std::vector<BoundChangeInfo>& changes_out) -> bool {
    if (node.local_binary_implications.empty()) return true;
    const double tol = std::max(1e-9, opt.int_tol);
    std::deque<std::pair<int, bool>> queue;
    std::vector<char> seen_zero(static_cast<std::size_t>(n), 0);
    std::vector<char> seen_one(static_cast<std::size_t>(n), 0);

    auto enqueue_literal = [&](int var, bool value_one) {
      if (var < 0 || var >= n) return;
      if (base_lp.vars[static_cast<std::size_t>(var)].type !=
          VarType::Binary) {
        return;
      }
      auto& seen = value_one ? seen_one[static_cast<std::size_t>(var)]
                             : seen_zero[static_cast<std::size_t>(var)];
      if (seen) return;
      seen = 1;
      queue.emplace_back(var, value_one);
    };

    for (int j = 0; j < n; ++j) {
      if (base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
        continue;
      }
      if (node.lb[j] >= 1.0 - tol) enqueue_literal(j, true);
      if (node.ub[j] <= tol) enqueue_literal(j, false);
    }

    while (!queue.empty()) {
      const auto [trigger, trigger_one] = queue.front();
      queue.pop_front();
      for (const auto& arc : node.local_binary_implications) {
        if (arc.trigger_var != trigger ||
            arc.trigger_value_one != trigger_one) {
          continue;
        }
        const int j = arc.implied_var;
        if (j < 0 || j >= n) continue;
        double implied_value = arc.implied_value;
        if (is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
          implied_value = arc.implied_is_lb
                              ? std::ceil(implied_value - tol)
                              : std::floor(implied_value + tol);
        }
        if (arc.implied_is_lb) {
          if (implied_value > node.ub[j] + tol) {
            ++out.bc_stats.local_implication_prunes;
            return false;
          }
          if (implied_value > node.lb[j] + tol) {
            const double old = node.lb[j];
            node.lb[j] = implied_value;
            changes_out.push_back(
                {j, implied_value - old, true, old, implied_value});
            ++out.bc_stats.local_implications_applied;
            if (base_lp.vars[static_cast<std::size_t>(j)].type ==
                    VarType::Binary &&
                node.lb[j] >= 1.0 - tol) {
              enqueue_literal(j, true);
            }
          }
        } else {
          if (implied_value < node.lb[j] - tol) {
            ++out.bc_stats.local_implication_prunes;
            return false;
          }
          if (implied_value < node.ub[j] - tol) {
            const double old = node.ub[j];
            node.ub[j] = implied_value;
            changes_out.push_back(
                {j, implied_value - old, false, old, implied_value});
            ++out.bc_stats.local_implications_applied;
            if (base_lp.vars[static_cast<std::size_t>(j)].type ==
                    VarType::Binary &&
                node.ub[j] <= tol) {
              enqueue_literal(j, false);
            }
          }
        }
      }
	    }
	    return bounds_consistent(node.lb, node.ub);
	  };

	  struct NodeDomainClosureStats {
	    std::uint64_t tightened{0};
	    std::uint64_t reasons_added{0};
	    std::uint64_t global_certificate_reasons{0};
	    std::uint64_t cutpool_tightened{0};
	    std::uint64_t objective_tightened{0};
	    std::uint64_t objective_proof_candidates{0};
	    std::uint64_t objective_proof_fixings{0};
	    std::uint64_t objective_proof_prunes{0};
	    std::uint64_t objective_proof_skip_coeff{0};
	    std::uint64_t objective_proof_skip_budget{0};
		    std::uint64_t objective_proof_skip_no_hit{0};
		    std::uint64_t conflict_tightened{0};
		    std::uint64_t row_tightened{0};
		    std::uint64_t redcost_tightened{0};
		    std::uint64_t redcost_active{0};
		    std::uint64_t redcost_scanned{0};
		    std::uint64_t cutpool_rows_scanned{0};
		    std::uint64_t cutpool_active_rows{0};
		    std::uint64_t cutpool_skip_capacity{0};
		    std::uint64_t cutpool_skip_infinite{0};
		    std::uint64_t cutpool_candidate_bounds{0};
		    std::uint64_t cutpool_rejected_weak_bounds{0};
		    std::uint64_t closure_passes{0};
			    std::uint64_t global_learning_events{0};
			    int conflicts_learned{0};
			    int cuts_learned{0};
			    bool domain_changed{false};
			    bool propagation_source_changed{false};
		    bool pruned{false};
		  };

  auto run_node_domain_closure =
      [&](Node& node,
          const std::vector<DomainReasonBound>& incoming_reasons,
          std::vector<BoundChangeInfo>& changes_out,
          std::vector<PoolCut>* new_cuts,
          int* cut_count,
          double inc_obj_snapshot,
          bool has_inc_snapshot,
          const char* phase,
          NodeDomainClosureStats* stats) -> bool {
    if (node.lb.size() != n || node.ub.size() != n) {
      return true;
    }
		    NodeDomainClosureStats local_stats;
		    NodeDomainClosureStats& st = stats != nullptr ? *stats : local_stats;
		    const double tol = std::max(1e-9, opt.int_tol);
		    const double upper_limit_snapshot =
		        has_inc_snapshot
		            ? highs_style_incumbent_upper_limit(base_lp, inc_obj_snapshot,
		                                               opt.int_tol)
		            : kInf;
		    const bool has_propagation_cutoff =
		        has_inc_snapshot && std::isfinite(upper_limit_snapshot);
			    const std::size_t initial_reason_count = node.domain_reason_bounds.size();
			    const std::size_t initial_scoped_count = node.scoped_conflict_clauses.size();
			    const std::size_t initial_changes = changes_out.size();
			    std::size_t processed_changes_for_cutpool = initial_changes;
			    std::uint64_t scanned_domain_source_epoch =
			        node.domain_learning_epoch;
			    auto build_pending_cols =
			        [&](std::size_t begin) -> std::vector<char> {
			      std::vector<char> pending(static_cast<std::size_t>(n), 0);
			      for (std::size_t i = begin; i < changes_out.size(); ++i) {
			        const int j = changes_out[i].var_idx;
			        if (j >= 0 && j < n) pending[static_cast<std::size_t>(j)] = 1;
			      }
			      return pending;
			    };
			    auto pending_has_any = [](const std::vector<char>& pending) {
			      return std::any_of(pending.begin(), pending.end(),
			                         [](char v) { return v != 0; });
			    };
			    bool inherited_root_domain = false;
		    if (root.lb.size() == n && root.ub.size() == n) {
		      for (int col = 0; col < n; ++col) {
		        if (std::isfinite(root.lb[col]) &&
		            node.lb[col] < root.lb[col] - tol) {
		          const double old = node.lb[col];
		          node.lb[col] = root.lb[col];
		          changes_out.push_back(
		              {col, node.lb[col] - old, true, old, node.lb[col]});
		          inherited_root_domain = true;
		        }
		        if (std::isfinite(root.ub[col]) &&
		            node.ub[col] > root.ub[col] + tol) {
		          const double old = node.ub[col];
		          node.ub[col] = root.ub[col];
		          changes_out.push_back(
		              {col, node.ub[col] - old, false, old, node.ub[col]});
		          inherited_root_domain = true;
		        }
		      }
			      if (!bounds_consistent(node.lb, node.ub)) {
			        st.pruned = true;
			        return false;
			      }
			    }
			    if (inherited_root_domain) st.domain_changed = true;

	    auto publish_closure_scratch_cuts =
	        [&](std::vector<PoolCut>& cuts, const char* source) -> int {
	      int added = 0;
	      for (auto& cut : cuts) {
	        if (!valid_pool_cut(cut, nullptr, source)) {
	          continue;
	        }
		        if (add_cut_to_global_domain_pool(std::move(cut.coeff), cut.rhs,
		                                          source, cut.domain_only,
		                                          cut.validity_scope)) {
	          ++added;
	        }
	      }
	      cuts.clear();
	      if (added > 0) {
	        st.cuts_learned += added;
	      }
	      return added;
	    };

	    auto duplicate_reason_bound = [&](const DomainReasonBound& rb) -> bool {
	      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
	      for (const auto& existing : node.domain_reason_bounds) {
        if (conflict_clause_hash(existing.reason) == reason_hash &&
            domain_reason_bounds_equal(existing, rb, tol)) {
          return true;
        }
      }
      return false;
    };

	    auto append_reason_and_certificate =
	        [&](DomainReasonBound rb, bool apply_bound) -> bool {
      if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
          !std::isfinite(rb.bound.value)) {
        return true;
      }
      bool bound_changed = false;
      if (apply_bound) {
        const int j = rb.bound.var_idx;
        if (rb.bound.is_lb) {
          if (rb.bound.value > node.ub[j] + tol) return false;
          if (rb.bound.value > node.lb[j] + tol) {
            const double old = node.lb[j];
            node.lb[j] = rb.bound.value;
            changes_out.push_back(
                {j, rb.bound.value - old, true, old, rb.bound.value});
            bound_changed = true;
          }
        } else {
          if (rb.bound.value < node.lb[j] - tol) return false;
          if (rb.bound.value < node.ub[j] - tol) {
            const double old = node.ub[j];
            node.ub[j] = rb.bound.value;
            changes_out.push_back(
                {j, rb.bound.value - old, false, old, rb.bound.value});
            bound_changed = true;
          }
        }
      }
      if (!duplicate_reason_bound(rb)) {
        append_domain_reason_bound(node, std::move(rb), n, &root.lb, &root.ub);
        ++st.reasons_added;
      }
	      if (bound_changed) ++st.tightened;
	      return true;
	    };

	    auto publish_closure_reason_target =
	        [&](const DomainReasonBound& rb, const char* tag) -> int {
	      (void)add_scoped_target_bound_certificate(node, rb);
	      if (publish_bound_lifting_certificate == nullptr ||
	          !strict_target_certificate_ok(rb)) {
	        return 0;
	      }
	      const int published = publish_bound_lifting_certificate(
	          rb, nullptr, nullptr, tag != nullptr ? tag : "domain_closure");
	      if (published > 0) {
	        st.global_certificate_reasons +=
	            static_cast<std::uint64_t>(published);
	      }
	      return published;
	    };

			    for (const auto& rb : incoming_reasons) {
			      if (!append_reason_and_certificate(rb, /*apply_bound=*/true)) {
			        st.pruned = true;
			        return false;
			      }
			      (void)publish_closure_reason_target(rb, "closure_incoming");
			    }
			    if (changes_out.size() > initial_changes) {
			      st.domain_changed = true;
			    }

		    auto run_objective_closure_step = [&]() -> bool {
		      if (!use_objective_propagation || !has_propagation_cutoff) {
	        return true;
	      }
	      Eigen::VectorXd objective_change_base_lb = node.lb;
	      Eigen::VectorXd objective_change_base_ub = node.ub;
	      apply_domain_objective_lower_bound(node, phase);
	      std::vector<PoolCut> scratch_cuts;
	      int scratch_cut_count = 0;
	      int objective_conflicts = 0;
	      int objective_tightened = 0;
	      int objective_reasons_added = 0;
	      std::vector<PoolCut>& cuts_ref =
	          new_cuts != nullptr ? *new_cuts : scratch_cuts;
	      int& cut_count_ref =
	          cut_count != nullptr ? *cut_count : scratch_cut_count;
	      const std::size_t objective_reason_begin =
	          node.domain_reason_bounds.size();
		      if (!apply_objective_cutoff_domain_fixing(
		              node, upper_limit_snapshot, has_inc_snapshot, cuts_ref,
	              cut_count_ref, objective_conflicts, objective_tightened,
	              &objective_reasons_added)) {
	        if (new_cuts == nullptr) {
	          (void)publish_closure_scratch_cuts(
	              scratch_cuts, "objective_closure_cutoff_prune");
	        }
		        for (std::size_t i = objective_reason_begin;
		             i < node.domain_reason_bounds.size(); ++i) {
		          (void)publish_closure_reason_target(
		              node.domain_reason_bounds[i],
		              "objective_closure_cutoff_prune");
		        }
	        st.objective_tightened +=
	            static_cast<std::uint64_t>(std::max(0, objective_tightened));
	        st.reasons_added +=
	            static_cast<std::uint64_t>(std::max(0, objective_reasons_added));
	        st.conflicts_learned += objective_conflicts;
	        st.pruned = true;
	        return false;
	      }
	      if (new_cuts == nullptr) {
	        (void)publish_closure_scratch_cuts(scratch_cuts,
	                                           "objective_closure_cutoff");
	      }
		      for (std::size_t i = objective_reason_begin;
		           i < node.domain_reason_bounds.size(); ++i) {
		        (void)publish_closure_reason_target(
		            node.domain_reason_bounds[i], "objective_closure_raw");
		      }
	      st.objective_tightened +=
	          static_cast<std::uint64_t>(std::max(0, objective_tightened));
	      st.reasons_added +=
	          static_cast<std::uint64_t>(std::max(0, objective_reasons_added));
	      st.conflicts_learned += objective_conflicts;
	      apply_domain_objective_lower_bound(node, phase);
	      auto append_objective_bound_changes = [&]() {
	        if (objective_change_base_lb.size() != n ||
	            objective_change_base_ub.size() != n) {
	          return;
	        }
	        for (int col = 0; col < n; ++col) {
	          if (node.lb[col] > objective_change_base_lb[col] + tol) {
	            changes_out.push_back(
	                {col, node.lb[col] - objective_change_base_lb[col], true});
	          }
	          if (node.ub[col] < objective_change_base_ub[col] - tol) {
	            changes_out.push_back(
	                {col, node.ub[col] - objective_change_base_ub[col], false});
	          }
	        }
	        objective_change_base_lb = node.lb;
	        objective_change_base_ub = node.ub;
	      };
	      if (objective_tightened > 0) {
	        append_objective_bound_changes();
	      }

	      const DualProofRow* objective_proof =
	          allow_certificate_domain_replay
	              ? select_objective_event_proof_for_node(
	                    node, /*require_proof_tail=*/true)
	              : nullptr;
	      if (objective_proof == nullptr) return true;
      const double objective_proof_upper =
          objective_event_proof_upper(objective_proof);
      const double objective_proof_lower =
          dual_proof_domain_lower_bound(objective_proof, objective_proof_upper,
                                        node.lb, node.ub);
      if (!std::isfinite(objective_proof_lower)) return true;

	      std::vector<DomainReasonBound> proof_reasons;
	      ObjectivePropagationState::ProofCapacityPropagationStats proof_stats;
	      int proof_tightened = 0;
	      int proof_pruned = 0;
	      const Eigen::VectorXd* node_reduced_costs = nullptr;
	      if (node.basis_hint && node.basis_hint->cached_reduced_costs &&
	          node.basis_hint->cached_reduced_costs->size() >= n) {
	        node_reduced_costs = node.basis_hint->cached_reduced_costs.get();
	      }
	      const bool proof_ok =
	          objective_propagation.propagate_proof_capacity_one_missing(
	                      base_lp,
	                      ObjectivePropagationState::ProofCapacityContext{
	                  &objective_proof->coeff,
	                  node.x_relax.size() >= n ? &node.x_relax : nullptr,
	                  node_reduced_costs,
	                  objective_proof_lower, upper_limit_snapshot, opt.int_tol,
	                  std::max(1e-9, opt.lp_tol), node.depth,
	                  static_cast<int>(node.branch_reasons.size() +
	                                   node.domain_reason_bounds.size())},
	              node.lb, node.ub, &proof_reasons, proof_tightened,
	              proof_pruned, &proof_stats);
	      st.objective_proof_candidates += proof_stats.candidates;
	      st.objective_proof_fixings += proof_stats.fixings;
	      st.objective_proof_prunes += proof_stats.prunes;
	      st.objective_proof_skip_coeff += proof_stats.skip_coeff;
	      st.objective_proof_skip_budget += proof_stats.skip_budget;
	      st.objective_proof_skip_no_hit +=
	          proof_stats.skip_no_lp_or_queue_hit;
	      out.bc_stats.termination_effective_candidates +=
	          proof_stats.candidates;
	      out.bc_stats.termination_effective_admitted +=
	          proof_stats.fixings + proof_stats.prunes;
	      out.bc_stats.termination_effective_skip_no_lp_or_queue_hit +=
	          proof_stats.skip_no_lp_or_queue_hit;
	      out.bc_stats.objective_implied_event_proof_candidates +=
	          proof_stats.candidates;
	      out.bc_stats.objective_implied_event_proof_fixings +=
	          proof_stats.fixings;
	      out.bc_stats.objective_implied_event_proof_skip_coeff +=
	          proof_stats.skip_coeff;
	      out.bc_stats.objective_implied_event_proof_skip_budget +=
	          proof_stats.skip_budget;
	      out.bc_stats.objective_implied_event_tightenings +=
	          static_cast<std::uint64_t>(std::max(0, proof_tightened));
	      if (proof_pruned > 0) {
	        out.bc_stats.objective_implied_event_prunes +=
	            static_cast<std::uint64_t>(proof_pruned);
	      }
	      st.objective_tightened +=
	          static_cast<std::uint64_t>(std::max(0, proof_tightened));
	      if (proof_tightened > 0) {
	        append_objective_bound_changes();
	      }
		      for (auto& rb : proof_reasons) {
		        DomainReasonBound rb_copy = rb;
		        const bool duplicate_before = duplicate_reason_bound(rb);
		        if (!append_reason_and_certificate(std::move(rb),
		                                           /*apply_bound=*/false)) {
	          st.pruned = true;
		          return false;
		        }
		        (void)publish_closure_reason_target(
		            duplicate_before ? rb_copy : node.domain_reason_bounds.back(),
		            "objective_closure_proof_capacity");
		      }
	      if (!proof_ok || proof_pruned > 0 ||
	          !bounds_consistent(node.lb, node.ub)) {
	        st.pruned = true;
	        return false;
	      }
		      return true;
		    };

		    // HiGHS' local domain propagation treats conflict pool, cut pool, objective
	    // cutoff propagation and row propagation as one monotone closure.  The
    // outer loop here repeats only when one source changed the local domain or
    // published a new reason/certificate that can feed another source.
    const std::size_t max_closure_passes =
        std::max<std::size_t>(8, 2 * static_cast<std::size_t>(n) + 8);
	    bool changed = inherited_root_domain || !incoming_reasons.empty();
    for (std::size_t pass = 0; pass < max_closure_passes; ++pass) {
      ++st.closure_passes;
      const std::size_t pass_changes_before = changes_out.size();
      const std::size_t pass_reasons_before = node.domain_reason_bounds.size();
      const std::size_t pass_scoped_before = node.scoped_conflict_clauses.size();
	      const std::size_t pass_conflict_pool_before =
	          static_cast<std::size_t>(conflict_pool.size());
	      const int pass_cut_pool_before = cut_pool.size();
	      const std::uint64_t pass_impl_before =
	          static_cast<std::uint64_t>(implication_graph.size());
	      const std::uint64_t pass_clique_edges_before =
	          static_cast<std::uint64_t>(clique_table.n_literal_edges());
	      const std::size_t pass_cuts_before =
	          new_cuts != nullptr ? new_cuts->size() : 0;
		      const std::uint64_t pass_domain_epoch_before =
		          current_domain_learning_epoch();
		      const std::size_t pass_global_certificates_before =
		          global_bound_lifting_certificates.size();
		      const std::size_t pass_pending_begin = processed_changes_for_cutpool;

		      const auto source_refresh =
		          refresh_objective_domain_sources(
		              phase != nullptr ? phase : "node_domain_closure");
		      st.global_learning_events += source_refresh.total_published();

		      std::uint64_t global_cert_reasons = 0;
		      if (!propagate_global_bound_lifting_certificates(
		              node, changes_out, &global_cert_reasons)) {
        st.pruned = true;
        return false;
      }
	      st.global_certificate_reasons += global_cert_reasons;
	      st.reasons_added += global_cert_reasons;

		      if (!run_objective_closure_step()) {
		        return false;
		      }

		      if (!propagate_local_conflict_clauses(node, changes_out) ||
	          !explain_active_scoped_reason_bounds(node, changes_out) ||
          !propagate_scoped_conflict_objects(node, changes_out) ||
          !propagate_local_binary_implications(node, changes_out)) {
        st.pruned = true;
        return false;
      }

	      std::vector<BranchDomainLiteral> learned_conflict;
      std::vector<DomainReasonBound> row_reason_bounds;
      int row_tightenings = 0;
      auto audit_witness_in_box = [&](const Eigen::VectorXd& lb,
                                      const Eigen::VectorXd& ub) {
        if (!opt.verbose || reduced_initial_solution.size() != n ||
            lb.size() != n || ub.size() != n) {
          return false;
        }
        const double witness_tol =
            std::max(1e-9, 10.0 * std::max(opt.int_tol, opt.lp_tol));
        for (int column = 0; column < n; ++column) {
          if (reduced_initial_solution[column] < lb[column] - witness_tol ||
              reduced_initial_solution[column] > ub[column] + witness_tol) {
            return false;
          }
        }
        return true;
      };
      const bool witness_in_domain_before =
          audit_witness_in_box(node.lb, node.ub);
      const bool collect_propagation_audit =
          opt.verbose && reduced_initial_solution.size() == n;
      const Eigen::VectorXd witness_audit_lb =
          collect_propagation_audit ? node.lb : Eigen::VectorXd{};
      const Eigen::VectorXd witness_audit_ub =
          collect_propagation_audit ? node.ub : Eigen::VectorXd{};
      std::vector<DomainPropagationEvent> propagation_audit_events;
      DomainPropagationFailure propagation_audit_failure;
      const bool node_domain_ok = propagate_node_domain(
              base_lp, A_row_cached, Aeq_row_cached, row_index_cached, node.lb,
              node.ub, opt.bound_propagation_rounds, &conflict_pool,
              &clique_table, &implication_graph, changes_out,
              node.branch_reasons, &learned_conflict, &row_tightenings,
	              &row_reason_bounds,
	              node.domain_reason_bounds.read_ptr(),
              collect_propagation_audit ? &propagation_audit_events : nullptr,
              collect_propagation_audit ? &propagation_audit_failure : nullptr);
      if (witness_in_domain_before &&
          !audit_witness_in_box(node.lb, node.ub)) {
        const double witness_tol =
            std::max(1e-9, 10.0 * std::max(opt.int_tol, opt.lp_tol));
        for (int column = 0; column < n; ++column) {
          const double value = reduced_initial_solution[column];
          if (value < node.lb[column] - witness_tol ||
              value > node.ub[column] + witness_tol) {
            const DomainPropagationEvent* cause = nullptr;
            for (auto event_it = propagation_audit_events.rbegin();
                 event_it != propagation_audit_events.rend(); ++event_it) {
              if (event_it->variable != column) continue;
              if ((event_it->is_lb && value < event_it->new_value - witness_tol) ||
                  (!event_it->is_lb && value > event_it->new_value + witness_tol)) {
                cause = &*event_it;
                break;
              }
            }
            const DomainPropagationEvent fallback_cause =
                propagation_audit_failure.available
                    ? propagation_audit_failure.event
                    : DomainPropagationEvent{};
            if (cause == nullptr) cause = &fallback_cause;
            fmt::print(stderr,
                       "[B&C-WITNESS-VIOLATION] source={} "
                       "phase={} depth={} col={} x={:.17g} "
                       "before=[{:.17g},{:.17g}] "
                       "after=[{:.17g},{:.17g}] ok={} "
                       "event_var={} side={} old={:.17g} new={:.17g} "
                       "row={} trigger={} trigger_value={} coeff={:.17g} "
                       "rhs={:.17g} activity_bound={:.17g} "
                       "failure_source={} failure_var={}\n",
                       domain_propagation_source_name(cause->source),
                       phase != nullptr ? phase : "unknown", node.depth,
                       column, value, witness_audit_lb[column],
                       witness_audit_ub[column], node.lb[column],
                       node.ub[column], node_domain_ok ? 1 : 0,
                       cause->variable, cause->is_lb ? "lb" : "ub",
                       cause->old_value, cause->new_value, cause->row,
                       cause->trigger_variable,
                       cause->trigger_value_one ? 1 : 0, cause->coefficient,
                       cause->rhs, cause->activity_bound,
                       propagation_audit_failure.available
                           ? domain_propagation_source_name(
                                 propagation_audit_failure.event.source)
                           : "none",
                       propagation_audit_failure.available
                           ? propagation_audit_failure.event.variable
                           : -1);
            for (std::size_t literal_index = 0;
                 literal_index < cause->source_clause.size();
                 ++literal_index) {
              const auto& literal = cause->source_clause[literal_index];
              const bool valid_literal =
                  literal.var_idx >= 0 && literal.var_idx < n;
              const bool active_at_node =
                  valid_literal &&
                  (literal.is_lb
                       ? node.lb[literal.var_idx] >= literal.value - witness_tol
                       : node.ub[literal.var_idx] <= literal.value + witness_tol);
              const bool active_at_witness =
                  valid_literal &&
                  (literal.is_lb
                       ? reduced_initial_solution[literal.var_idx] >=
                             literal.value - witness_tol
                       : reduced_initial_solution[literal.var_idx] <=
                             literal.value + witness_tol);
              fmt::print(stderr,
                         "[B&C-WITNESS-CLAUSE] index={} var={} side={} "
                         "value={:.17g} node_active={} witness_active={} "
                         "witness_value={:.17g}\n",
                         literal_index, literal.var_idx,
                         literal.is_lb ? "lb" : "ub", literal.value,
                         active_at_node ? 1 : 0, active_at_witness ? 1 : 0,
                         valid_literal
                             ? reduced_initial_solution[literal.var_idx]
                             : std::numeric_limits<double>::quiet_NaN());
            }
          }
        }
      }
      if (!node_domain_ok) {
        if (learned_conflict.empty()) learned_conflict = node.branch_reasons;
        if (!learned_conflict.empty()) {
          if (opt.verbose && conflict_clause_hits_witness(learned_conflict)) {
            const auto& failure_event = propagation_audit_failure.event;
            fmt::print(stderr,
                       "[B&C-WITNESS-INVALID-CLOSURE-CAUSE] phase={} depth={} "
                       "witness_in_before={} failure_available={} "
                       "failure_source={} var={} row={} "
                       "side={} old={:.17g} new={:.17g} trigger={} "
                       "trigger_value={} coeff={:.17g} rhs={:.17g} "
                       "events={}\n",
                       phase != nullptr ? phase : "unknown", node.depth,
                       witness_in_domain_before ? 1 : 0,
                       propagation_audit_failure.available ? 1 : 0,
                       propagation_audit_failure.available
                           ? domain_propagation_source_name(failure_event.source)
                           : "none",
                       failure_event.variable, failure_event.row,
                       failure_event.is_lb ? "lb" : "ub",
                       failure_event.old_value, failure_event.new_value,
                       failure_event.trigger_variable,
                       failure_event.trigger_value_one ? 1 : 0,
                       failure_event.coefficient, failure_event.rhs,
                       propagation_audit_events.size());
            if (propagation_audit_failure.available &&
                failure_event.source ==
                    DomainPropagationSource::EqualityRow &&
                failure_event.row >= 0 &&
                failure_event.row < Aeq_row_cached.rows()) {
              double witness_activity = 0.0;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator
                       row_it(Aeq_row_cached, failure_event.row);
                   row_it; ++row_it) {
                witness_activity +=
                    row_it.value() * reduced_initial_solution[row_it.col()];
              }
              fmt::print(stderr,
                         "[B&C-WITNESS-EQUALITY-CAUSE] row={} rhs={:.17g} "
                         "witness_activity={:.17g} residual={:.17g}\n",
                         failure_event.row, base_lp.beq[failure_event.row],
                         witness_activity,
                         witness_activity - base_lp.beq[failure_event.row]);
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator
                       row_it(Aeq_row_cached, failure_event.row);
                   row_it; ++row_it) {
                const int col = static_cast<int>(row_it.col());
                fmt::print(stderr,
                           "[B&C-WITNESS-EQUALITY-TERM] row={} col={} "
                           "a={:.17g} before=[{:.17g},{:.17g}] "
                           "after=[{:.17g},{:.17g}] witness={:.17g}\n",
                           failure_event.row, col, row_it.value(),
                           witness_audit_lb[col], witness_audit_ub[col],
                           node.lb[col], node.ub[col],
                           reduced_initial_solution[col]);
              }
            }
          }
          const bool pool_added = add_global_conflict(
              learned_conflict, "node_domain_closure");
          const bool impl_added = add_binary_implications_from_conflict_clause(
              base_lp.vars, learned_conflict, implication_graph);
          if (pool_added || impl_added) {
            note_global_domain_learning(1 + (impl_added ? 1 : 0));
            ++st.conflicts_learned;
          }
	          if (!strict_highs_lp_contract) {
	            PoolCut conflict_cut;
	            if (try_build_binary_conflict_cut(
	                    base_lp, learned_conflict, conflict_cut,
	                    opt.reduced_cost_conflict_cut_max_literals) &&
	                valid_pool_cut(conflict_cut, nullptr,
	                               "node_closure_conflict_cut")) {
	              if (new_cuts != nullptr && cut_count != nullptr) {
	                new_cuts->push_back(std::move(conflict_cut));
	                ++(*cut_count);
	                ++st.cuts_learned;
	              } else if (add_cut_to_global_domain_pool(
	                             std::move(conflict_cut.coeff), conflict_cut.rhs,
	                             "node_closure_conflict_cut",
	                             conflict_cut.domain_only,
	                             conflict_cut.validity_scope)) {
	                ++st.cuts_learned;
	              }
	            }
	          }
	        }
        st.pruned = true;
        return false;
      }
	      st.row_tightened +=
	          static_cast<std::uint64_t>(std::max(0, row_tightenings));
	      for (auto& rb : row_reason_bounds) {
	        DomainReasonBound rb_copy = rb;
	        const bool duplicate_before = duplicate_reason_bound(rb);
	        if (!append_reason_and_certificate(std::move(rb),
	                                           /*apply_bound=*/false)) {
	          st.pruned = true;
	          return false;
	        }
	        (void)publish_closure_reason_target(
	            duplicate_before ? rb_copy : node.domain_reason_bounds.back(),
	            "row_domain_closure");
	      }

      if (!row_reason_bounds.empty()) {
        if (!explain_active_scoped_reason_bounds(node, changes_out) ||
            !propagate_scoped_conflict_objects(node, changes_out)) {
          st.pruned = true;
          return false;
        }
      }

	      auto propagate_cutpool_family =
	          [&](const std::vector<Eigen::SparseVector<double>>& rows,
	              const std::vector<double>& rhs,
	              const char* source) -> bool {
	        if (rows.empty()) return true;
	        const bool force_source_scan =
	            current_domain_learning_epoch() > scanned_domain_source_epoch;
	        const std::vector<char> pending_cols =
	            build_pending_cols(pass_pending_begin);
	        if (!force_source_scan && !pending_has_any(pending_cols)) return true;
	        const Eigen::VectorXd old_lb = node.lb;
	        const Eigen::VectorXd old_ub = node.ub;
	        const std::size_t cutpool_changes_begin = changes_out.size();
	        SparseCutpoolPropagationStats cutpool_stats;
	        std::vector<DomainReasonBound> cutpool_reason_bounds;
	        const bool ok = propagate_sparse_cutpool_rows(
	            rows, rhs, node.lb, node.ub, opt.bound_propagation_rounds,
	            &cutpool_stats, &cutpool_reason_bounds, node.depth,
	            static_cast<int>(node.branch_reasons.size() +
	                             node.domain_reason_bounds.size()),
	            force_source_scan ? nullptr : &pending_cols);
        if (!ok || !bounds_consistent(node.lb, node.ub)) {
          st.cutpool_tightened +=
              static_cast<std::uint64_t>(std::max(0, cutpool_stats.tightened));
          (void)source;
          return false;
        }
        if (cutpool_stats.tightened > 0) {
          st.cutpool_tightened += static_cast<std::uint64_t>(
              std::max(0, cutpool_stats.tightened));
          for (int col = 0; col < n; ++col) {
            if (node.lb[col] > old_lb[col] + tol) {
              changes_out.push_back(
                  {col, node.lb[col] - old_lb[col], true, old_lb[col],
                   node.lb[col]});
            }
            if (node.ub[col] < old_ub[col] - tol) {
              changes_out.push_back(
                  {col, node.ub[col] - old_ub[col], false, old_ub[col],
                   node.ub[col]});
            }
	          }
	          for (auto& rb : cutpool_reason_bounds) {
	            DomainReasonBound rb_copy = rb;
	            const bool duplicate_before = duplicate_reason_bound(rb);
	            if (!append_reason_and_certificate(std::move(rb),
	                                               /*apply_bound=*/false)) {
	              return false;
	            }
	            (void)publish_closure_reason_target(
	                duplicate_before ? rb_copy : node.domain_reason_bounds.back(),
	                source);
	          }
	          processed_changes_for_cutpool =
	              std::min(processed_changes_for_cutpool,
	                       cutpool_changes_begin);
	        }
        return true;
      };
      if ((!root_transformed_cutpool_in_global_domain_pool &&
           !propagate_cutpool_family(root_transformed_cutpool_audit_rows,
                                     root_transformed_cutpool_audit_rhs,
                                     "root_transformed_cutpool")) ||
          !propagate_cutpool_family(variable_bound_cutpool_rows,
                                    variable_bound_cutpool_rhs,
                                    "variable_bound_cutpool")) {
        st.pruned = true;
        return false;
      }

	      if (cut_pool.size() > 0 || (new_cuts != nullptr && !new_cuts->empty())) {
	        int global_cutpool_tightened = 0;
	        std::vector<BoundChangeInfo> global_cutpool_changes;
	        std::vector<DomainReasonBound> global_cutpool_reason_bounds;
	        CutPoolPropagationStats global_cutpool_stats;
	        const std::vector<char> pending_cols =
	            build_pending_cols(pass_pending_begin);
	        const bool force_source_scan =
	            current_domain_learning_epoch() > scanned_domain_source_epoch;
	        const bool scan_existing_cutpool = pending_has_any(pending_cols);
	        const bool have_pending_extra_cuts =
	            new_cuts != nullptr && !new_cuts->empty();
	        if (!force_source_scan && !scan_existing_cutpool &&
	            !have_pending_extra_cuts) {
	          processed_changes_for_cutpool = changes_out.size();
	        } else {
	          const bool global_cutpool_ok = cut_pool.propagate_with_reasons(
	              base_lp.vars, node.lb, node.ub, opt.bound_propagation_rounds,
	              &global_cutpool_tightened, &global_cutpool_changes,
	              &global_cutpool_reason_bounds, &global_cutpool_stats,
	              new_cuts, &root_implied_integer_cols, node.depth,
	              static_cast<int>(node.branch_reasons.size() +
	                               node.domain_reason_bounds.size()),
	              std::max(1e-9, opt.lp_tol),
	              force_source_scan ? nullptr : &pending_cols);
	          st.cutpool_rows_scanned += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.rows_scanned));
	          st.cutpool_active_rows += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.active_rows));
	          st.cutpool_skip_capacity += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.skipped_capacity));
	          st.cutpool_skip_infinite += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.skipped_infinite_min));
	          st.cutpool_candidate_bounds += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.candidate_bounds));
	          st.cutpool_rejected_weak_bounds += static_cast<std::uint64_t>(
	              std::max(0, global_cutpool_stats.rejected_weak_bounds));
	          if (!global_cutpool_ok || !bounds_consistent(node.lb, node.ub)) {
	            st.cutpool_tightened += static_cast<std::uint64_t>(
	                std::max(0, global_cutpool_stats.tightened));
	            st.pruned = true;
	            return false;
	          }
	          if (global_cutpool_tightened > 0) {
	            const std::size_t global_cutpool_changes_begin =
	                changes_out.size();
	            changes_out.insert(changes_out.end(), global_cutpool_changes.begin(),
	                               global_cutpool_changes.end());
	            st.cutpool_tightened += static_cast<std::uint64_t>(
	                std::max(0, global_cutpool_tightened));
	            for (auto& rb : global_cutpool_reason_bounds) {
	              DomainReasonBound rb_copy = rb;
	              const bool duplicate_before = duplicate_reason_bound(rb);
	              if (!append_reason_and_certificate(std::move(rb),
	                                                 /*apply_bound=*/false)) {
	                st.pruned = true;
	                return false;
	              }
	              (void)publish_closure_reason_target(
	                  duplicate_before ? rb_copy : node.domain_reason_bounds.back(),
	                  "global_cutpool_closure");
	            }
	            processed_changes_for_cutpool =
	                std::min(processed_changes_for_cutpool,
	                         global_cutpool_changes_begin);
	          }
	          if (global_cutpool_tightened <= 0) {
	            processed_changes_for_cutpool = changes_out.size();
	          }
	        }
	      }
	      scanned_domain_source_epoch =
	          std::max(scanned_domain_source_epoch,
	                   current_domain_learning_epoch());

      if (node.domain_reason_bounds.size() != pass_reasons_before) {
        std::vector<PoolCut> scratch_cuts;
        int scratch_cut_count = 0;
        std::vector<PoolCut>& cuts_ref =
            new_cuts != nullptr ? *new_cuts : scratch_cuts;
        int& cut_count_ref =
            cut_count != nullptr ? *cut_count : scratch_cut_count;
        const std::size_t clauses_before =
            static_cast<std::size_t>(conflict_pool.size());
	        if (learn_short_reconvergence_implication(
	                node.branch_reasons, node.domain_reason_bounds, nullptr,
	                nullptr, nullptr, nullptr, cuts_ref, cut_count_ref, &node)) {
	          ++st.conflicts_learned;
	        }
	        if (new_cuts == nullptr) {
	          (void)publish_closure_scratch_cuts(
	              scratch_cuts, "node_closure_reconvergence_cut");
	        }
	        const std::size_t clauses_after =
	            static_cast<std::size_t>(conflict_pool.size());
        if (clauses_after > clauses_before) {
          note_global_domain_learning(static_cast<std::uint64_t>(
              clauses_after - clauses_before));
        }
      }

		      const bool pass_domain_changed =
		          changes_out.size() != pass_changes_before;
		      const bool pass_source_changed =
		          static_cast<std::size_t>(conflict_pool.size()) !=
		              pass_conflict_pool_before ||
		          cut_pool.size() != pass_cut_pool_before ||
		          static_cast<std::uint64_t>(implication_graph.size()) !=
		              pass_impl_before ||
		          static_cast<std::uint64_t>(clique_table.n_literal_edges()) !=
		              pass_clique_edges_before ||
		          (new_cuts != nullptr && new_cuts->size() != pass_cuts_before) ||
		          source_refresh.total_published() > 0;
		      const bool pass_metadata_changed =
		          node.domain_reason_bounds.size() != pass_reasons_before ||
		          node.scoped_conflict_clauses.size() != pass_scoped_before ||
		          global_bound_lifting_certificates.size() !=
		              pass_global_certificates_before;
		      const bool pass_changed =
		          pass_domain_changed || pass_source_changed ||
		          pass_metadata_changed;
		      st.domain_changed = st.domain_changed || pass_domain_changed;
		      st.propagation_source_changed =
		          st.propagation_source_changed || pass_source_changed;
		      if (current_domain_learning_epoch() != pass_domain_epoch_before) {
		        st.global_learning_events += current_domain_learning_epoch() -
		                                     pass_domain_epoch_before;
		      }
		      changed = changed || pass_changed || pass_metadata_changed;
		      if (!pass_changed) break;
		    }

    if (!bounds_consistent(node.lb, node.ub)) {
      st.pruned = true;
      return false;
    }
    st.tightened =
        static_cast<std::uint64_t>(changes_out.size() - initial_changes);
    const bool metadata_changed =
        node.domain_reason_bounds.size() != initial_reason_count ||
        node.scoped_conflict_clauses.size() != initial_scoped_count;
    const bool trail_changed = changed || metadata_changed;
    const bool domain_moved =
        st.tightened > 0 || st.conflict_tightened > 0 || st.row_tightened > 0 ||
        st.cutpool_tightened > 0 || st.objective_tightened > 0 ||
        st.redcost_tightened > 0 || st.cuts_learned > 0;
    if (trail_changed) {
      rebuild_local_domain_trail(node, n, &root.lb, &root.ub);
      node.domain_learning_epoch = current_domain_learning_epoch();
      node.objective_artifact_epoch = current_objective_artifact_epoch();
      node.domain_closure_epoch = current_domain_closure_epoch();
    }
    if (domain_moved) {
      node.lp_refresh_needed = true;
      if (node.x_seed.size() == n) {
        node.x_seed = clamp_to_bounds(node.x_seed, node.lb, node.ub);
      }
      if (node.x_relax.size() == n) {
        node.x_relax = clamp_to_bounds(node.x_relax, node.lb, node.ub);
      }
      if (apply_domain_objective_lower_bound(node, phase) >
          std::max(1e-8, opt.lp_tol)) {
        node.lp_refresh_needed = true;
      }
    }
	    if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) &&
		        (st.tightened > 0 || st.reasons_added > 0 || st.cutpool_tightened > 0 ||
		         st.objective_tightened > 0 || st.redcost_tightened > 0 ||
		         st.conflict_tightened > 0 || st.row_tightened > 0 ||
		         st.conflicts_learned > 0 ||
		         st.objective_proof_candidates > 0 ||
		         st.objective_proof_fixings > 0 ||
		         st.objective_proof_prunes > 0 ||
		         st.cutpool_rows_scanned > 0)) {
		      fmt::print(stderr,
		                 "[B&C-DOM-CLOSURE] phase={} depth={} pass={} tight={} "
			                 "reasons={} certReasons={} conflictTight={} rowTight={} "
			                 "cutpoolTight={} objTight={} rcTight={} rcActive={} "
			                 "cutRows={}/{} skipCap={} skipInf={} cand/rej={}/{} "
			                 "learnEvt={} objProof={}/{}/{} "
			                 "skip={}/{}/{} learnedConf={} learnedCuts={} refresh={}\n",
			                 phase != nullptr ? phase : "node", node.depth,
			                 st.closure_passes, st.tightened, st.reasons_added,
			                 st.global_certificate_reasons, st.conflict_tightened,
			                 st.row_tightened, st.cutpool_tightened,
			                 st.objective_tightened, st.redcost_tightened,
			                 st.redcost_active, st.cutpool_active_rows,
			                 st.cutpool_rows_scanned, st.cutpool_skip_capacity,
			                 st.cutpool_skip_infinite,
			                 st.cutpool_candidate_bounds,
			                 st.cutpool_rejected_weak_bounds,
			                 st.global_learning_events,
			                 st.objective_proof_candidates,
			                 st.objective_proof_fixings, st.objective_proof_prunes,
	                 st.objective_proof_skip_coeff,
	                 st.objective_proof_skip_budget,
	                 st.objective_proof_skip_no_hit, st.conflicts_learned,
	                 st.cuts_learned, node.lp_refresh_needed ? 1 : 0);
	    }
    return true;
  };

	  replay_queue_reason_trail_certificates =
	      [&](Node& node, const std::vector<DomainReasonBound>& new_reasons,
	          std::uint64_t& tightened_out, std::uint64_t& reasons_out,
	          bool& pruned_out) -> bool {
	    tightened_out = 0;
	    reasons_out = 0;
	    pruned_out = false;
	    if (node.lb.size() != n || node.ub.size() != n) {
	      return true;
	    }
	    std::vector<BoundChangeInfo> changes;
	    NodeDomainClosureStats closure_stats;
	    const bool ok = run_node_domain_closure(
	        node, new_reasons, changes, nullptr, nullptr,
	        has_incumbent ? incumbent_obj : kInf, has_incumbent,
	        "queue_reason_replay", &closure_stats);
		    tightened_out =
		        closure_stats.tightened + closure_stats.conflict_tightened +
		        closure_stats.row_tightened + closure_stats.cutpool_tightened +
		        closure_stats.objective_tightened +
		        closure_stats.redcost_tightened;
	    reasons_out = closure_stats.reasons_added +
	                  closure_stats.global_certificate_reasons;
	    pruned_out = !ok || closure_stats.pruned ||
	                 !bounds_consistent(node.lb, node.ub);
	    return ok && !pruned_out;
	  };

	  double gap_limit_lower_bound_certificate = -kInf;
  double timeline_last_reported_lb = -kInf;

  auto timeline_gap_limit_value = [&]() -> double {
    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
        !std::isfinite(opt.gap_tol) || opt.gap_tol < 0.0) {
      return kInf;
    }
    const double denom = std::max(1.0, std::abs(incumbent_obj));
    return incumbent_obj - opt.gap_tol * denom;
  };

  auto print_bc_timeline_state = [&](const char* phase, const char* reason,
                                     double qlb_raw, int qsize,
                                     int pruned) {
    if (!trace_bc_timeline) return;
    const double gap =
        has_incumbent && std::isfinite(global_lb)
            ? std::max(0.0, (incumbent_obj - global_lb) /
                                  std::max(1.0, std::abs(incumbent_obj)))
            : kInf;
    fmt::print(stderr,
               "[B&C-TL] phase={} reason={} time_ms={:.0f} node={} lp={} "
               "qsize={} qlb_raw={:.10g} lb={:.10g} ub={:.10g} "
               "gap_limit={:.10g} gap={:.6g} pruned={} gap_cert={:.10g}\n",
               phase != nullptr ? phase : "state",
               reason != nullptr ? reason : "none", bc_elapsed_ms(),
               out.bc_stats.nodes_explored, out.bc_stats.lp_solves, qsize,
               qlb_raw, global_lb, has_incumbent ? incumbent_obj : kInf,
               timeline_gap_limit_value(), gap, pruned,
               gap_limit_lower_bound_certificate);
  };

  auto refresh_queue_lower_bound = [&]() -> double {
    const double live_lb = node_queue.lower_bound();
    if (std::isfinite(live_lb)) {
      global_lb = std::isfinite(gap_limit_lower_bound_certificate)
                      ? std::min(live_lb, gap_limit_lower_bound_certificate)
                      : live_lb;
    } else if (std::isfinite(gap_limit_lower_bound_certificate)) {
      global_lb = has_incumbent
                      ? std::min(incumbent_obj,
                                 gap_limit_lower_bound_certificate)
                      : gap_limit_lower_bound_certificate;
    } else if (has_incumbent) {
      global_lb = incumbent_obj;
    } else if (std::isfinite(root.bound)) {
      global_lb = root.bound;
    }
    out.bc_stats.best_bound = global_lb;
    note_bound_event(has_incumbent ? incumbent_obj : kInf, global_lb,
                     /*retain_incumbent=*/false);
    if (trace_bc_timeline && std::isfinite(global_lb) &&
        (!std::isfinite(timeline_last_reported_lb) ||
         global_lb > timeline_last_reported_lb + 1e-6)) {
      timeline_last_reported_lb = global_lb;
      print_bc_timeline_state("lb_update", "refresh_queue", live_lb,
                              node_queue.size(), 0);
    }
    return global_lb;
  };

	  auto record_queue_lower_bound_audit = [&](const char* phase,
	                                            const char* source,
	                                            double before_lb,
	                                            double after_lb) {
    const QueueLowerBoundAudit audit =
        node_queue.audit_lower_bound_index(std::max(1e-9, opt.lp_tol));
    ++out.bc_stats.certificate_queue_lb_audit_passes;
    if (!audit.ok) {
      ++out.bc_stats.certificate_queue_lb_audit_failures;
    }
    out.bc_stats.certificate_queue_lb_audit_indexed_min = audit.indexed_min;
    out.bc_stats.certificate_queue_lb_audit_computed_min = audit.computed_min;
    out.bc_stats.certificate_queue_lb_audit_max_error =
        std::max(out.bc_stats.certificate_queue_lb_audit_max_error,
                 audit.abs_error);
    out.bc_stats.certificate_queue_lb_audit_indexed_nodes =
        audit.indexed_nodes;
    out.bc_stats.certificate_queue_lb_audit_live_nodes = audit.live_nodes;
    if (opt.verbose || !audit.ok) {
      fmt::print(stderr,
                 "[B&C-QUEUE-LB] phase={} source={} ok={} "
                 "nodes={}/{} indexed={:.10g} computed={:.10g} "
                 "err={:.6g} before={:.10g} after={:.10g}\n",
                 phase != nullptr ? phase : "unknown",
                 source != nullptr ? source : "unknown", audit.ok ? 1 : 0,
                 audit.indexed_nodes, audit.live_nodes, audit.indexed_min,
                 audit.computed_min, audit.abs_error, before_lb, after_lb);
    }
	    return audit.ok;
	  };

	  auto inherit_root_domain_to_queue = [&](const char* source,
	                                          bool force_frontier_drain) -> int {
	    if (node_queue.empty() || root.lb.size() != n || root.ub.size() != n) {
	      return 0;
	    }
	    const double qlb_before = node_queue.lower_bound();
	    std::uint64_t inherited_tightened = 0;
	    std::uint64_t refresh_marked = 0;
	    std::uint64_t lift_nodes = 0;
		    double lift_sum = 0.0;
		    double lift_max = 0.0;
		    const double root_queue_upper_limit =
		        has_incumbent && std::isfinite(incumbent_obj)
		            ? incumbent_upper_limit()
		            : kInf;
		    const bool has_root_queue_upper_limit =
		        has_incumbent && std::isfinite(root_queue_upper_limit);
		    const double root_queue_prune_tol =
		        has_root_queue_upper_limit
		            ? incumbent_upper_limit_prune_tol(root_queue_upper_limit)
		            : kIncumbentPruneTol;
		    const int pruned = node_queue.inherit_root_domain(
		        root.lb, root.ub, &domain_objective_lower_bound,
		        has_root_queue_upper_limit, root_queue_upper_limit,
		        root_queue_prune_tol,
		        &inherited_tightened, &refresh_marked, &lift_nodes, &lift_sum,
		        &lift_max);
	    if (inherited_tightened > 0 || refresh_marked > 0 || pruned > 0 ||
	        lift_sum > 0.0) {
	      note_global_domain_learning(inherited_tightened);
	      out.bc_stats.certificate_queue_tightenings += inherited_tightened;
	      out.bc_stats.certificate_domain_fixings += inherited_tightened;
	      out.bc_stats.certificate_queue_lp_refresh_marked += refresh_marked;
	      out.bc_stats.certificate_queue_lower_bound_lift_nodes += lift_nodes;
	      out.bc_stats.certificate_queue_lift_sum += lift_sum;
	      out.bc_stats.certificate_queue_lift_max =
	          std::max(out.bc_stats.certificate_queue_lift_max, lift_max);
	      if (pruned > 0) {
	        out.bc_stats.certificate_queue_prunes +=
	            static_cast<std::uint64_t>(pruned);
	      }
	      refresh_queue_lower_bound();
	      record_queue_lower_bound_audit("queue_root_domain",
	                                     source != nullptr
	                                         ? source
	                                         : "root_domain_inherit",
	                                     qlb_before, node_queue.lower_bound());
	      if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	        fmt::print(stderr,
	                   "[B&C-QUEUE-ROOT-DOMAIN] source={} tight={} "
	                   "refresh={} pruned={} lift_nodes={} "
	                   "lift_sum/max={:.6g}/{:.6g} qlb={:.10g}->{:.10g}\n",
	                   source != nullptr ? source : "root_domain_inherit",
	                   inherited_tightened, refresh_marked, pruned, lift_nodes,
	                   lift_sum, lift_max, qlb_before, node_queue.lower_bound());
	      }
	    }
	    if (force_frontier_drain && inherited_tightened > 0) {
	      return pruned;
	    }
	    return pruned;
	  };
	  std::function<int(const char*, int)>
	      highs_style_post_incumbent_domain_closure;

  auto prune_queue_by_incumbent = [&]() -> int {
    if (!has_incumbent || !std::isfinite(incumbent_obj)) {
      return 0;
    }
    if (!current_incumbent_valid_for_proof("queue_incumbent_prune")) {
      return 0;
    }
    const double upper_limit = incumbent_upper_limit();
	    if (!std::isfinite(upper_limit)) {
	      return 0;
	    }
	    ++out.bc_stats.incumbent_queue_prune_passes;
	    const double prune_tol = incumbent_upper_limit_prune_tol(upper_limit);
    const int pruned =
        node_queue.prune_by_lower_bound_limit(upper_limit, prune_tol);
    if (pruned > 0) {
      out.bc_stats.incumbent_queue_prunes +=
          static_cast<std::uint64_t>(pruned);
      if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
        fmt::print(stderr,
                   "[B&C-QUEUE-BOUND] upper_limit={:.10g} incumbent={:.10g} "
                   "pruned={} qsize={}\n",
                   upper_limit, incumbent_obj, pruned, node_queue.size());
      }
    }
    refresh_queue_lower_bound();
    return pruned;
  };

	  auto gap_optimality_limit = [&]() -> double {
	    if (!root_gap_certificate_ready) {
	      return kInf;
	    }
	    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
	        !std::isfinite(opt.gap_tol) || opt.gap_tol < 0.0) {
	      return kInf;
	    }
	    if (!current_incumbent_valid_for_proof("gap_optimality_limit")) {
	      return kInf;
	    }
	    const double denom = std::max(1.0, std::abs(incumbent_obj));
	    return incumbent_obj - opt.gap_tol * denom;
	  };

  auto gap_optimality_prune_tol = [&](double limit) -> double {
    // This tolerance is in objective units.  Do not scale integrality tolerance
    // by |objective| here: doing so turns a 1e-4 MIP gap into a materially
    // wider pruning limit on million-scale UC objectives.
    return std::max({1e-7, 10.0 * std::max(0.0, opt.lp_tol),
                     1e-9 * std::max(1.0, std::abs(limit))});
  };

  auto prune_queue_by_gap_optimality_limit = [&](const char* phase) -> int {
    if (opt.require_tree_exhaustion_certificate) {
      return 0;
    }
    const double limit = gap_optimality_limit();
    if (!std::isfinite(limit) || node_queue.empty()) return 0;

    ++out.bc_stats.gap_suboptimal_queue_prune_passes;
    const double prune_tol = gap_optimality_prune_tol(limit);
    const double qlb_before = node_queue.lower_bound();
    const int qsize_before = node_queue.size();
    const int pruned = node_queue.prune_by_lower_bound_limit(limit, prune_tol);
    if (pruned > 0) {
      gap_limit_lower_bound_certificate =
          std::max(gap_limit_lower_bound_certificate, limit);
      out.bc_stats.gap_suboptimal_queue_prunes +=
          static_cast<std::uint64_t>(pruned);
      refresh_queue_lower_bound();
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C-GAP-QUEUE] phase={} limit={:.10g} qsize={}->{} "
                   "qlb={:.10g}->{:.10g} pruned={}\n",
                   phase != nullptr ? phase : "gap_limit", limit,
                   qsize_before, node_queue.size(), qlb_before, global_lb,
                   pruned);
      }
      print_bc_timeline_state("gap_queue_prune", phase, node_queue.lower_bound(),
                              node_queue.size(), pruned);
    }
    return pruned;
  };

	  int last_marked_queue_refresh_attempts = 0;
	  int last_marked_queue_refresh_remaining = 0;
	  int last_marked_queue_refresh_pruned = 0;
	  int last_marked_queue_refresh_bound_moves = 0;
	  double last_marked_queue_refresh_queue_lift = 0.0;
	  auto refresh_marked_queue_nodes =
	      [&](const char* source, int max_lps, double bound_band,
	          bool mandatory_frontier_contract) -> int {
	    last_marked_queue_refresh_attempts = 0;
	    last_marked_queue_refresh_remaining = 0;
	    last_marked_queue_refresh_pruned = 0;
	    last_marked_queue_refresh_bound_moves = 0;
	    last_marked_queue_refresh_queue_lift = 0.0;
	    if ((!mandatory_frontier_contract &&
             (!opt.enable_reduced_cost_fixing_resolve ||
              rc_fixing_resolve_lps_left <= 0 || max_lps <= 0)) ||
	        node_queue.empty()) {
	      return 0;
	    }
    const double qlb_for_refresh = node_queue.lower_bound();
    const double frontier_band =
        std::isfinite(qlb_for_refresh)
            ? std::max({1e-8, opt.lp_tol,
                        1e-10 * std::max(1.0, std::abs(qlb_for_refresh))})
            : bound_band;
    const double movement_band =
        std::isfinite(bound_band) ? std::min(bound_band, frontier_band)
                                  : frontier_band;
    const int scan_limit = mandatory_frontier_contract ? 0 : 8192;
    const int marked_frontier_nodes =
        node_queue.count_lp_refresh_needed_near_lower_bound(
            movement_band, scan_limit);
    // HiGHS' currentNodeToQueue/openNodesToQueue contract runs local-domain
    // propagation before a node is left in the queue; conflicts are published
    // immediately and the node lower bound reflects the propagated domain.
    // Native stores certificate effects on queued nodes, so the equivalent
    // operation is to refresh every marked lower-bound-frontier LP as soon as
    // a proof/certificate pass marks it.  Delaying this until a large tail or
    // an incumbent-age gate leaves stale LP bounds in the proof frontier.
    if (marked_frontier_nodes <= 0) {
      return 0;
    }

    int effective_max_lps = max_lps;
    if (mandatory_frontier_contract) {
      effective_max_lps = marked_frontier_nodes;
    } else if (marked_frontier_nodes > 0) {
      effective_max_lps =
          std::max(effective_max_lps,
                   std::min(marked_frontier_nodes,
                            std::max(1, opt.reduced_cost_fixing_resolve_max_lps / 2)));
    }
    const int budget = mandatory_frontier_contract
                           ? effective_max_lps
                           : std::min(effective_max_lps,
                                      std::max(0, rc_fixing_resolve_lps_left));
    const double qlb_before = node_queue.lower_bound();
    int attempts = 0;
    int failures = 0;
    int pruned = 0;
	    int requeued = 0;
	    int bound_moves = 0;
	    int incumbent_updates = 0;
    LPFailureType last_refresh_type = LPFailureType::Unknown;
    std::string last_refresh_status;
    bool last_refresh_farkas = false;
    double last_refresh_objective = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t frontier_effective_nodes = 0;
	    double node_lift_sum = 0.0;
	    double node_lift_max = 0.0;

	    for (; attempts < budget &&
               (mandatory_frontier_contract ||
                rc_fixing_resolve_lps_left > 0); ++attempts) {
      const double qlb_at_pop = node_queue.lower_bound();
	      Node qnode;
	      bool qnode_was_dfs = false;
	      if (!node_queue.pop_lp_refresh_node(qnode, movement_band,
	                                          mandatory_frontier_contract
                                                  ? 0
                                                  : 4096,
	                                          &qnode_was_dfs)) {
	        break;
	      }
      const bool popped_lower_bound_frontier =
          std::isfinite(qlb_at_pop) && std::isfinite(qnode.bound) &&
          qnode.bound <= qlb_at_pop + frontier_band;

      auto requeue_refreshed_node = [&](Node node) {
        if (qnode_was_dfs) {
          node_queue.push_dfs(std::move(node));
        } else {
          node_queue.push_priority(std::move(node));
        }
      };

      std::vector<BoundChangeInfo> refresh_changes;
      NodeDomainClosureStats refresh_closure_stats;
      const std::vector<DomainReasonBound> no_refresh_reasons;
      bool domain_ok = run_node_domain_closure(
          qnode, no_refresh_reasons, refresh_changes, nullptr, nullptr,
          has_incumbent ? incumbent_obj : kInf, has_incumbent,
          "queue_refresh_pre_lp", &refresh_closure_stats);
      if (!domain_ok || !bounds_consistent(qnode.lb, qnode.ub)) {
        ++pruned;
        ++out.bc_stats.rc_conflict_queue_prunes;
        ++out.bc_stats.certificate_queue_prunes;
        ++out.bc_stats.certificate_domain_prunes;
        continue;
      }

      node_sf = build_node_sf_with_local_cuts(qnode);
      node_sf_cuts_augmented = true;
      SimplexOptions refresh_opt = simplex_opt;
      refresh_opt.allow_cold_start = true;
      auto refresh_res =
          solve_tree_lp(node_sf, refresh_opt, qnode.basis_hint.get());
      if (rc_fixing_resolve_lps_left > 0) {
        --rc_fixing_resolve_lps_left;
      }
      ++out.bc_stats.lp_solves;
      ++out.bc_stats.rc_fixing_resolve_lps;
      ++out.bc_stats.certificate_domain_resolve_lps;
      ++out.bc_stats.conformance_resolve_attempts;

      const auto refresh_type = classify_lp_result(refresh_res, n);
      last_refresh_type = refresh_type;
      last_refresh_status = refresh_res.result.stats.status;
      last_refresh_farkas = refresh_res.result.stats.has_farkas_certificate;
      last_refresh_objective = refresh_res.result.stats.objective;
      if (refresh_type == LPFailureType::Infeasible ||
          refresh_type == LPFailureType::ObjectiveCutoff) {
        if (refresh_type == LPFailureType::ObjectiveCutoff) {
          qnode.bound = std::max(qnode.bound,
                                 refresh_res.result.stats.objective);
        }
        ++pruned;
	        ++out.bc_stats.rc_fixing_resolve_prunes;
	        ++out.bc_stats.certificate_queue_prunes;
	        ++out.bc_stats.certificate_domain_prunes;
        if (popped_lower_bound_frontier) ++frontier_effective_nodes;
	        continue;
	      }
      if (refresh_type != LPFailureType::Success ||
          refresh_res.result.x.size() != n ||
          !std::isfinite(refresh_res.result.stats.objective)) {
        ++failures;
        ++out.bc_stats.conformance_resolve_failures;
        requeue_refreshed_node(std::move(qnode));
        break;
      }

      const double old_bound = qnode.bound;
      qnode.x_relax = clamp_to_bounds(refresh_res.result.x, qnode.lb,
                                      qnode.ub);
      qnode.x_seed = qnode.x_relax;
      qnode.bound = std::max(qnode.bound, refresh_res.result.stats.objective);
      if (qnode.estimate < qnode.bound) qnode.estimate = qnode.bound;
      const double lift = std::max(0.0, qnode.bound - old_bound);
      ++out.bc_stats.rc_proof_resolve_lift_samples;
      out.bc_stats.rc_proof_resolve_lift_sum += lift;
      out.bc_stats.rc_proof_resolve_lift_max =
          std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
      node_lift_sum += lift;
      node_lift_max = std::max(node_lift_max, lift);
	      if (lift > std::max(1e-7, opt.lp_tol)) {
	        ++bound_moves;
        if (popped_lower_bound_frontier) ++frontier_effective_nodes;
	        ++out.bc_stats.rc_fixing_resolve_bound_improvements;
        ++out.bc_stats.certificate_domain_bound_improvements;
        ++out.bc_stats.conformance_resolve_bound_moves;
        out.bc_stats.conformance_bound_lift_sum += lift;
        out.bc_stats.conformance_bound_lift_max =
            std::max(out.bc_stats.conformance_bound_lift_max, lift);
      }

	      qnode.basis_hint =
	          persist_bc_node_basis_from_simplex(refresh_res, qnode.basis_hint);
		      qnode.domain_learning_epoch = current_domain_learning_epoch();
		      qnode.objective_artifact_epoch = current_objective_artifact_epoch();
		      qnode.domain_closure_epoch = current_domain_closure_epoch();
		      qnode.lp_refresh_needed = false;

      bool rc_refresh_deferred = false;
      bool rc_refresh_pruned = false;
      int queue_rc_fixed = 0;
      int queue_rc_passes = 0;
      for (;;) {
        const double queue_upper_limit =
            has_incumbent && std::isfinite(incumbent_obj)
                ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
                                                   opt.int_tol)
                : kInf;
        if (!opt.enable_reduced_cost_fixing ||
            (!mandatory_frontier_contract && rc_fixing_resolve_lps_left <= 0) ||
            !std::isfinite(queue_upper_limit) || qnode.basis_hint == nullptr ||
            !qnode.basis_hint->cached_reduced_costs ||
            qnode.basis_hint->cached_reduced_costs->size() < n) {
          break;
        }
        std::vector<DomainReasonBound> queue_rc_reasons;
        const int rc_fixed = reduced_cost_fixing(
            base_lp.vars, qnode.x_relax,
            *qnode.basis_hint->cached_reduced_costs,
            qnode.basis_hint->basis_indices(), qnode.bound,
            queue_upper_limit, opt.int_tol, qnode.lb, qnode.ub, nullptr,
            qnode.basis_hint->cached_col_scale.get(), &queue_rc_reasons,
            qnode.branch_reasons.read_ptr(), qnode.depth,
            static_cast<int>(qnode.branch_reasons.size() +
                             qnode.domain_reason_bounds.size()));
        if (rc_fixed <= 0) break;
        queue_rc_fixed += rc_fixed;
        ++queue_rc_passes;
        out.bc_stats.rc_fixings += static_cast<std::uint64_t>(rc_fixed);
        for (auto& rb : queue_rc_reasons) {
          append_domain_reason_bound(qnode, std::move(rb), n, &root.lb,
                                     &root.ub);
        }
        rebuild_local_domain_trail(qnode, n, &root.lb, &root.ub);
        qnode.lp_refresh_needed = true;

        std::vector<BoundChangeInfo> rc_refresh_changes;
        NodeDomainClosureStats rc_refresh_closure_stats;
        const std::vector<DomainReasonBound> no_rc_refresh_reasons;
        const bool rc_domain_ok = run_node_domain_closure(
            qnode, no_rc_refresh_reasons, rc_refresh_changes, nullptr, nullptr,
            has_incumbent ? incumbent_obj : kInf, has_incumbent,
            "queue_refresh_rc_closure", &rc_refresh_closure_stats);
        if (!rc_domain_ok || !bounds_consistent(qnode.lb, qnode.ub)) {
          ++pruned;
          ++out.bc_stats.rc_fixing_resolve_prunes;
          ++out.bc_stats.certificate_queue_prunes;
          ++out.bc_stats.certificate_domain_prunes;
          rc_refresh_deferred = false;
          rc_refresh_pruned = true;
          break;
        }

        node_sf = build_node_sf_with_local_cuts(qnode);
      node_sf_cuts_augmented = true;
        SimplexOptions rc_refresh_opt = simplex_opt;
        rc_refresh_opt.allow_cold_start = true;
        auto rc_refresh_res =
            solve_tree_lp(node_sf, rc_refresh_opt, qnode.basis_hint.get());
        if (rc_fixing_resolve_lps_left > 0) {
          --rc_fixing_resolve_lps_left;
        }
        ++out.bc_stats.lp_solves;
        ++out.bc_stats.rc_fixing_resolve_lps;
        ++out.bc_stats.certificate_domain_resolve_lps;
        ++out.bc_stats.conformance_resolve_attempts;

        const auto rc_refresh_type = classify_lp_result(rc_refresh_res, n);
        last_refresh_type = rc_refresh_type;
        last_refresh_status = rc_refresh_res.result.stats.status;
        last_refresh_farkas = rc_refresh_res.result.stats.has_farkas_certificate;
        last_refresh_objective = rc_refresh_res.result.stats.objective;
        if (rc_refresh_type == LPFailureType::Infeasible ||
            rc_refresh_type == LPFailureType::ObjectiveCutoff) {
          if (rc_refresh_type == LPFailureType::ObjectiveCutoff) {
            qnode.bound =
                std::max(qnode.bound, rc_refresh_res.result.stats.objective);
          }
          ++pruned;
          ++out.bc_stats.rc_fixing_resolve_prunes;
          ++out.bc_stats.certificate_queue_prunes;
          ++out.bc_stats.certificate_domain_prunes;
          if (popped_lower_bound_frontier) ++frontier_effective_nodes;
          rc_refresh_deferred = false;
          rc_refresh_pruned = true;
          break;
        }
        if (rc_refresh_type != LPFailureType::Success ||
            rc_refresh_res.result.x.size() != n ||
            !std::isfinite(rc_refresh_res.result.stats.objective)) {
          ++failures;
          ++out.bc_stats.conformance_resolve_failures;
          qnode.lp_refresh_needed = true;
          rc_refresh_deferred = true;
          break;
        }

        const double rc_old_bound = qnode.bound;
        qnode.x_relax = clamp_to_bounds(rc_refresh_res.result.x, qnode.lb,
                                        qnode.ub);
        qnode.x_seed = qnode.x_relax;
        qnode.bound =
            std::max(qnode.bound, rc_refresh_res.result.stats.objective);
        if (qnode.estimate < qnode.bound) qnode.estimate = qnode.bound;
        const double rc_lift = std::max(0.0, qnode.bound - rc_old_bound);
        ++out.bc_stats.rc_proof_resolve_lift_samples;
        out.bc_stats.rc_proof_resolve_lift_sum += rc_lift;
        out.bc_stats.rc_proof_resolve_lift_max =
            std::max(out.bc_stats.rc_proof_resolve_lift_max, rc_lift);
        node_lift_sum += rc_lift;
        node_lift_max = std::max(node_lift_max, rc_lift);
        if (rc_lift > std::max(1e-7, opt.lp_tol)) {
          ++bound_moves;
          if (popped_lower_bound_frontier) ++frontier_effective_nodes;
          ++out.bc_stats.rc_fixing_resolve_bound_improvements;
          ++out.bc_stats.certificate_domain_bound_improvements;
          ++out.bc_stats.conformance_resolve_bound_moves;
          out.bc_stats.conformance_bound_lift_sum += rc_lift;
          out.bc_stats.conformance_bound_lift_max =
              std::max(out.bc_stats.conformance_bound_lift_max, rc_lift);
        }
        qnode.basis_hint =
            persist_bc_node_basis_from_simplex(rc_refresh_res, qnode.basis_hint);
        qnode.domain_learning_epoch = current_domain_learning_epoch();
        qnode.objective_artifact_epoch = current_objective_artifact_epoch();
        qnode.domain_closure_epoch = current_domain_closure_epoch();
        qnode.lp_refresh_needed = false;
      }
      if (queue_rc_fixed > 0 &&
          (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr)) {
        fmt::print(stderr,
                   "[B&C-RC-FIXPOINT] phase=queue_refresh depth={} pass={} "
                   "fix={} bound={:.10g} deferred={}\n",
                   qnode.depth, queue_rc_passes, queue_rc_fixed, qnode.bound,
                   rc_refresh_deferred ? 1 : 0);
      }
      if (rc_refresh_deferred) {
        requeue_refreshed_node(std::move(qnode));
        ++requeued;
        break;
      }
      if (rc_refresh_pruned ||
          (queue_rc_fixed > 0 && !bounds_consistent(qnode.lb, qnode.ub))) {
        continue;
      }

		      std::vector<int> refresh_frac;
      if (!fractional_branchable_indices(
              qnode.x_relax, qnode.lb, qnode.ub,
              requested_vendored_highs_lp ? std::max(1e-9, opt.lp_tol)
                                          : opt.int_tol,
              refresh_frac) &&
          satisfies_with_bounds(base_lp, qnode.x_relax, qnode.lb, qnode.ub,
                                requested_vendored_highs_lp
                                    ? std::max(1e-6, opt.lp_tol)
                                    : 1e-6)) {
        Eigen::VectorXd x_int = qnode.x_relax;
	        round_declared_branchables(x_int, qnode.lb, qnode.ub);
	        const double obj = objective_value(base_lp.c, x_int, base_lp.sense);
	        if (adopt_tree_incumbent("queue_refresh", x_int, obj, qnode.depth,
	                                 global_lb)) {
	          ++incumbent_updates;
	          if (opt.verbose) {
	            fmt::print(stderr,
                       "[B&C LP-INTEGRAL] phase=queue_refresh depth={} "
                       "obj={:.10g} bound={:.10g}\n",
                       qnode.depth, obj, qnode.bound);
          }
	        }
	        ++pruned;
        if (popped_lower_bound_frontier) ++frontier_effective_nodes;
	        continue;
	      }

	      const double queue_refresh_prune_limit =
	          has_incumbent && std::isfinite(incumbent_obj)
	              ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                                 opt.int_tol)
	              : kInf;
	      if (incumbent_prunes_node(
	              has_incumbent && std::isfinite(queue_refresh_prune_limit),
	              queue_refresh_prune_limit, qnode.bound,
	              std::max(1e-9, opt.lp_tol))) {
	        ++pruned;
	        ++out.bc_stats.rc_fixing_resolve_prunes;
	        ++out.bc_stats.certificate_queue_prunes;
        if (popped_lower_bound_frontier) ++frontier_effective_nodes;
	        continue;
	      }

	      requeue_refreshed_node(std::move(qnode));
	      ++requeued;
      // A zero-lift LP refresh proves only this queued node's current
      // frontier bound.  Other marked frontier nodes may still lift or prune,
      // so strict proof drain must continue through the available refresh
      // budget instead of stopping on the first degenerate/no-move node.
      (void)popped_lower_bound_frontier;
    }

    if (incumbent_updates > 0) {
      pruned += prune_queue_by_incumbent();
    }
    refresh_queue_lower_bound();
    const double qlb_after = node_queue.lower_bound();
    record_queue_lower_bound_audit("queue_resolve",
                     source != nullptr ? source
                             : "domain_refresh",
                     qlb_before, qlb_after);
    const double queue_lift =
        (std::isfinite(qlb_before) && std::isfinite(qlb_after))
            ? std::max(0.0, qlb_after - qlb_before)
            : 0.0;
    if (queue_lift > std::max(1e-8, opt.lp_tol)) {
      ++out.bc_stats.conformance_queue_bound_moves;
      out.bc_stats.conformance_queue_lift_sum += queue_lift;
      out.bc_stats.conformance_queue_lift_max =
          std::max(out.bc_stats.conformance_queue_lift_max, queue_lift);
      out.bc_stats.certificate_queue_lower_bound_lift_nodes +=
          frontier_effective_nodes;
    }
	    last_marked_queue_refresh_attempts = attempts;
	    last_marked_queue_refresh_remaining =
	        node_queue.count_lp_refresh_needed_near_lower_bound(
	            movement_band, scan_limit);
	    last_marked_queue_refresh_pruned = pruned;
	    last_marked_queue_refresh_bound_moves = bound_moves;
	    last_marked_queue_refresh_queue_lift = queue_lift;
	    if (bc_conf_diag_enabled() &&
          (opt.verbose || attempts > 0 || pruned > 0 || queue_lift > 0.0)) {
	      fmt::print(stderr,
	                 "[B&C-QUEUE-RESOLVE] source={} attempts={} requeued={} "
	                 "pruned={} failures={} inc={} bound_moves={} "
	                 "frontier_marked={} lb_nodes={} lift_avg/max={:.6g}/{:.6g} "
	                 "qlb={:.10g}->{:.10g} budget_left={} mandatory={} "
                     "frontier_stale={} last_type={} last_status='{}' "
                     "last_farkas={} last_obj={:.10g}\n",
	                 source != nullptr ? source : "domain_refresh", attempts,
	                 requeued, pruned, failures, incumbent_updates, bound_moves,
	                 marked_frontier_nodes, frontier_effective_nodes,
	                 attempts > 0 ? node_lift_sum / static_cast<double>(attempts)
	                              : 0.0,
	                 node_lift_max, qlb_before, qlb_after,
                 rc_fixing_resolve_lps_left,
                 mandatory_frontier_contract ? 1 : 0,
                 last_marked_queue_refresh_remaining,
                 lp_failure_type_str(last_refresh_type), last_refresh_status,
                 last_refresh_farkas ? 1 : 0, last_refresh_objective);
    }
    (void)bound_moves;
    return pruned;
  };

	  auto prune_queue_after_domain_learning =
	      [&](bool force_frontier_drain = false, const char* reason = nullptr)
	          -> int {
	    const auto queue_source_refresh = refresh_objective_domain_sources(
	        reason != nullptr ? reason : "queue_domain_learning");
	    const double cutoff_improve_tol =
	        std::max(1e-7, has_incumbent && std::isfinite(incumbent_obj)
	                            ? 1e-10 * std::max(1.0, std::abs(incumbent_obj))
	                            : 1e-7);
    const bool cutoff_strictly_improved =
        has_incumbent && std::isfinite(incumbent_obj) &&
        (!std::isfinite(last_queue_domain_replay_incumbent) ||
         incumbent_obj < last_queue_domain_replay_incumbent - cutoff_improve_tol);
    const std::uint64_t closure_epoch = current_domain_closure_epoch();
	    const bool has_new_global_learning =
	        closure_epoch > last_queue_domain_replay_epoch;
	    if (!force_frontier_drain && !cutoff_strictly_improved &&
	        !has_new_global_learning &&
	        queue_source_refresh.total_published() == 0) {
	      return 0;
	    }
	    // HiGHS' HighsDomain::propagate() is driven by pending propagation rows:
	    // once a conflict/cut/objective artifact is learned, the domain closure
	    // must run regardless of how many previous queue replays were empty.
	    // Stale replay suppression may only apply when there is no new source,
	    // which is already handled by the early return above.
    const double queue_incumbent_upper_limit =
        has_incumbent && std::isfinite(incumbent_obj)
            ? incumbent_upper_limit()
            : kInf;
    const bool has_queue_upper_limit =
        has_incumbent && std::isfinite(queue_incumbent_upper_limit);
    const double queue_upper_limit_prune_tol =
        has_queue_upper_limit
            ? incumbent_upper_limit_prune_tol(queue_incumbent_upper_limit)
            : kIncumbentPruneTol;

    const double queue_lb_before = node_queue.lower_bound();
    const int queue_size_before = node_queue.size();
    const std::uint64_t queue_sources = conformance_source_total();
    std::uint64_t closure_tightened_total = 0;
    std::uint64_t cutpool_tightened_total = 0;
    std::uint64_t objective_tightened_total = 0;
    int direct_incumbent_pruned = 0;
	    int closure_pruned_total = 0;
	    int cutpool_pruned_total = 0;
	    int objective_pruned_total = 0;
	    int pruned = prune_queue_by_incumbent();
	    direct_incumbent_pruned = pruned;
    const int direct_gap_pruned =
        prune_queue_by_gap_optimality_limit("queue_domain_learning");
    pruned += direct_gap_pruned;
    int learned_pruned = 0;
    if (!global_bound_lifting_certificates.empty()) {
      std::uint64_t certificate_tightened = 0;
      std::uint64_t certificate_reasons = 0;
      std::uint64_t certificate_refresh_marked = 0;
      double certificate_move_sum = 0.0;
      double certificate_move_max = 0.0;
      const bool frontier_limited_cert =
          opt.require_tree_exhaustion_certificate &&
          (proof_tail_queue_mode_active() || force_frontier_drain);
      const double cert_replay_band =
          frontier_limited_cert
              ? std::max(termination_bound_band(), 2.0 * opt.lp_tol)
              : kInf;
      const int cert_replay_max_nodes =
          frontier_limited_cert
              ? (force_frontier_drain ? 2048 : 512)
              : 0;
      const int certificate_pruned =
          node_queue.propagate_bound_lifting_certificates(
              base_lp.vars, global_bound_lifting_certificates,
              has_queue_upper_limit, queue_incumbent_upper_limit,
              queue_upper_limit_prune_tol,
              &certificate_tightened, &certificate_reasons,
              &certificate_refresh_marked, &certificate_move_sum,
              &certificate_move_max, cert_replay_band,
              cert_replay_max_nodes, &root.lb, &root.ub,
              &domain_objective_lower_bound,
              &replay_queue_reason_trail_certificates);
      out.bc_stats.certificate_queue_lp_refresh_marked +=
          certificate_refresh_marked;
      if (certificate_move_sum > 0.0) {
        out.bc_stats.certificate_queue_frontier_bound_moves +=
            certificate_tightened;
        out.bc_stats.certificate_queue_frontier_bound_move_sum +=
            certificate_move_sum;
        out.bc_stats.certificate_queue_frontier_bound_move_max =
            std::max(out.bc_stats.certificate_queue_frontier_bound_move_max,
                     certificate_move_max);
      }
      if (certificate_tightened > 0 || certificate_reasons > 0) {
        closure_tightened_total += certificate_tightened;
        out.bc_stats.certificate_queue_tightenings += certificate_tightened;
        out.bc_stats.certificate_domain_fixings += certificate_tightened;
        out.bc_stats.scoped_conflict_objects_learned += certificate_reasons;
        out.bc_stats.termination_effective_admitted +=
            certificate_tightened + certificate_reasons;
      }
      if (certificate_pruned > 0) {
        learned_pruned += certificate_pruned;
        out.bc_stats.certificate_queue_prunes +=
            static_cast<std::uint64_t>(certificate_pruned);
      }
    }
						    const bool objective_replay_allowed =
					        cutoff_strictly_improved ||
		                    current_objective_artifact_epoch() >
	                            last_queue_objective_replay_epoch ||
	                        (!proof_tail_queue_mode_active() &&
	                         has_new_global_learning);
		    const double queue_upper_limit =
		        has_incumbent && std::isfinite(incumbent_obj)
		            ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
		                                               opt.int_tol)
		            : kInf;
		    if (objective_replay_allowed &&
		        use_objective_propagation && has_incumbent &&
	        std::isfinite(queue_upper_limit) &&
	        (!objective_propagation.partitions.empty() ||
         !objective_propagation.implied_events.empty() ||
         objective_propagation.max_implied_event_delta > 0.0 ||
         objective_propagation.max_replacement_delta > 0.0)) {
      std::uint64_t objective_tightened = 0;
      std::uint64_t queue_objcap_nodes = 0;
      std::uint64_t queue_objcap_active = 0;
      std::uint64_t queue_objcap_active_tight = 0;
      std::uint64_t queue_objcap_one_events = 0;
      std::uint64_t queue_objcap_one_keys = 0;
      std::uint64_t queue_objcap_raw_exceed = 0;
      std::uint64_t queue_objcap_proof_exceed = 0;
      std::uint64_t objective_reasons_added = 0;
      ObjectivePropagationState::ProofCapacityPropagationStats
          queue_proof_stats;
      double queue_objcap_min_raw_cap = kInf;
      double queue_objcap_min_proof_cap = kInf;
      double queue_objcap_max_one = 0.0;
      double queue_objcap_max_proof_excess = 0.0;
      const DualProofRow* global_queue_objective_proof = nullptr;
      if (have_cached_queue_objective_event_proof &&
          cached_queue_objective_event_proof_is_global &&
          cached_queue_objective_event_proof.valid &&
          cached_queue_objective_event_proof.coeff.size() >= n &&
          cached_queue_objective_event_proof.coeff.nonZeros() > 0) {
        global_queue_objective_proof = &cached_queue_objective_event_proof;
      } else if (have_root_global_dual_proof && root_global_dual_proof.valid &&
                 root_global_dual_proof.coeff.size() >= n &&
                 root_global_dual_proof.coeff.nonZeros() > 0) {
        global_queue_objective_proof = &root_global_dual_proof;
      }
	      if (!proof_tail_queue_mode_active() ||
              !dual_proof_has_objective_event_target(
                  global_queue_objective_proof)) {
	        global_queue_objective_proof = nullptr;
	      }
	      const bool allow_scoped_queue_proof = proof_tail_queue_mode_active();
      const bool have_scoped_queue_objective_proof =
          allow_scoped_queue_proof &&
          have_cached_queue_objective_event_proof &&
          !cached_queue_objective_event_proof_is_global &&
          cached_queue_objective_event_proof.valid &&
          cached_queue_objective_event_proof.coeff.size() >= n &&
          cached_queue_objective_event_proof.coeff.nonZeros() > 0 &&
          dual_proof_has_objective_event_target(
              &cached_queue_objective_event_proof);
      auto scope_active_for_node =
          [&](const Node& node,
              const std::vector<BranchDomainLiteral>& scope) {
        const double tol = std::max(1e-9, opt.int_tol);
        for (const auto& lit : scope) {
          if (lit.var_idx < 0 || lit.var_idx >= n ||
              lit.var_idx >= node.lb.size() ||
              lit.var_idx >= node.ub.size()) {
            return false;
          }
          const bool active = lit.is_lb
              ? (node.lb[lit.var_idx] >= lit.value - tol)
              : (node.ub[lit.var_idx] <= lit.value + tol);
          if (!active) return false;
        }
        return true;
      };
      const bool proof_tail_frontier_only = proof_tail_queue_mode_active();
      const double objective_queue_bound_band =
          proof_tail_frontier_only
              ? std::max(termination_bound_band(), 2.0 * opt.lp_tol)
              : kInf;
      const int objective_queue_max_nodes =
          proof_tail_frontier_only ? 512 : 0;
      const int objective_pruned =
          node_queue.propagate_by_domain_object_with_reasons(
          [&](const Node& node,
              Eigen::VectorXd& lb,
              Eigen::VectorXd& ub,
              std::vector<DomainReasonBound>& reasons,
              int& tightened,
              int& pruned) {
            if (opt.verbose) {
              const auto audit =
                  objective_propagation.audit_capacity_conformance(
                      base_lp, lb, ub, queue_upper_limit, node.bound,
                      opt.int_tol);
              if (audit.valid) {
                ++queue_objcap_nodes;
                queue_objcap_active += audit.active_events;
                queue_objcap_active_tight += audit.active_tightening_events;
                queue_objcap_one_events += audit.one_missing_events;
                queue_objcap_one_keys += audit.one_missing_keys;
                queue_objcap_raw_exceed += audit.one_missing_raw_exceed_keys;
                queue_objcap_proof_exceed += audit.one_missing_proof_exceed_keys;
                queue_objcap_min_raw_cap =
                    std::min(queue_objcap_min_raw_cap, audit.raw_capacity);
                queue_objcap_min_proof_cap =
                    std::min(queue_objcap_min_proof_cap, audit.proof_capacity);
                queue_objcap_max_one =
                    std::max(queue_objcap_max_one,
                             audit.max_one_missing_contribution);
                queue_objcap_max_proof_excess =
                    std::max(queue_objcap_max_proof_excess,
                             audit.max_proof_excess);
              }
            }
            const int trail_base = static_cast<int>(
                node.branch_reasons.size() + node.domain_reason_bounds.size());
            int raw_tightened = 0;
            int raw_pruned = 0;
            const bool raw_ok = objective_propagation.propagate(
                base_lp, lb, ub, queue_upper_limit, opt.int_tol, node.depth,
                trail_base, &reasons, raw_tightened, raw_pruned);
            tightened = raw_tightened;
            pruned = raw_pruned;
            if (!raw_ok || pruned > 0) return raw_ok;
            const DualProofRow* queue_objective_proof = nullptr;
            if (have_scoped_queue_objective_proof &&
                scope_active_for_node(
                    node, cached_queue_objective_event_proof_scope)) {
              queue_objective_proof = &cached_queue_objective_event_proof;
            }
            if (queue_objective_proof == nullptr) {
              queue_objective_proof = global_queue_objective_proof;
            }
	            if (queue_objective_proof != nullptr &&
	                allow_certificate_domain_replay) {
              const double queue_objective_proof_upper =
                  objective_event_proof_upper(queue_objective_proof);
              const double queue_objective_proof_lower =
                  dual_proof_domain_lower_bound(queue_objective_proof,
                                                queue_objective_proof_upper,
                                                lb, ub);
              if (!std::isfinite(queue_objective_proof_lower)) {
                return bounds_consistent(lb, ub);
              }
              const bool using_scoped_queue_proof =
                  queue_objective_proof ==
                      &cached_queue_objective_event_proof &&
                  !cached_queue_objective_event_proof_is_global;
              const std::size_t scoped_reason_begin = reasons.size();
	              ObjectivePropagationState::ProofCapacityPropagationStats
	                  local_proof_stats;
	              int proof_tightened = 0;
	              int proof_pruned = 0;
              const Eigen::VectorXd* node_reduced_costs = nullptr;
              if (raw_tightened == 0 && node.basis_hint &&
                  node.basis_hint->cached_reduced_costs &&
                  node.basis_hint->cached_reduced_costs->size() >= n) {
                node_reduced_costs = node.basis_hint->cached_reduced_costs.get();
              }
	              const bool proof_ok =
	                  objective_propagation.propagate_proof_capacity_one_missing(
	                      base_lp,
	                      ObjectivePropagationState::ProofCapacityContext{
	                          &queue_objective_proof->coeff, &node.x_relax,
                          node_reduced_costs,
	                          queue_objective_proof_lower, queue_upper_limit,
                          opt.int_tol,
	                          std::max(1e-9, opt.lp_tol), node.depth,
	                          trail_base + static_cast<int>(reasons.size())},
                      lb, ub, &reasons, proof_tightened, proof_pruned,
                      &local_proof_stats);
              // The cached child proof scope is a consumption gate.  HiGHS
              // resolves the local trail and publishes only the reconvergence
              // frontier; appending the generating child branch here would turn
              // a cross-frontier certificate back into a local artifact.
              (void)using_scoped_queue_proof;
              (void)scoped_reason_begin;
              queue_proof_stats.candidates += local_proof_stats.candidates;
              queue_proof_stats.fixings += local_proof_stats.fixings;
              queue_proof_stats.prunes += local_proof_stats.prunes;
              queue_proof_stats.skip_coeff += local_proof_stats.skip_coeff;
              queue_proof_stats.skip_budget += local_proof_stats.skip_budget;
	              queue_proof_stats.skip_no_lp_or_queue_hit +=
	                  local_proof_stats.skip_no_lp_or_queue_hit;
              queue_proof_stats.rc_frontier_candidates +=
                  local_proof_stats.rc_frontier_candidates;
              queue_proof_stats.rc_frontier_fixings +=
                  local_proof_stats.rc_frontier_fixings;
              queue_proof_stats.rc_frontier_skip_budget +=
                  local_proof_stats.rc_frontier_skip_budget;
	              queue_proof_stats.max_proof_lift =
	                  std::max(queue_proof_stats.max_proof_lift,
	                           local_proof_stats.max_proof_lift);
              queue_proof_stats.max_proof_excess =
                  std::max(queue_proof_stats.max_proof_excess,
                           local_proof_stats.max_proof_excess);
              tightened += proof_tightened;
              pruned += proof_pruned;
              if (!proof_ok || pruned > 0) return proof_ok;
            }
            return bounds_consistent(lb, ub);
          },
		          has_queue_upper_limit, queue_incumbent_upper_limit,
		          queue_upper_limit_prune_tol,
	          &objective_tightened, &objective_reasons_added,
	          objective_queue_bound_band, objective_queue_max_nodes,
	          &root.lb, &root.ub, &domain_objective_lower_bound, nullptr,
	          nullptr, nullptr, &replay_queue_reason_trail_certificates);
      if (objective_tightened > 0 || objective_reasons_added > 0) {
        refresh_queue_lower_bound();
      }
      objective_tightened_total = objective_tightened;
      objective_pruned_total = objective_pruned;
      if (objective_tightened > 0) {
        out.bc_stats.cutoff_domain_tightenings += objective_tightened;
        out.bc_stats.objective_implied_event_tightenings +=
            objective_tightened;
        out.bc_stats.local_implications_applied += objective_tightened;
      }
      if (queue_proof_stats.candidates > 0 ||
          queue_proof_stats.fixings > 0 ||
          queue_proof_stats.skip_coeff > 0 ||
          queue_proof_stats.skip_budget > 0 ||
          queue_proof_stats.skip_no_lp_or_queue_hit > 0) {
        out.bc_stats.termination_effective_candidates +=
            queue_proof_stats.candidates;
        out.bc_stats.termination_effective_admitted +=
            queue_proof_stats.fixings + queue_proof_stats.prunes;
        out.bc_stats.termination_effective_skip_no_lp_or_queue_hit +=
            queue_proof_stats.skip_no_lp_or_queue_hit;
        out.bc_stats.objective_implied_event_proof_candidates +=
            queue_proof_stats.candidates;
        out.bc_stats.objective_implied_event_proof_fixings +=
            queue_proof_stats.fixings;
        out.bc_stats.objective_implied_event_proof_skip_coeff +=
            queue_proof_stats.skip_coeff;
        out.bc_stats.objective_implied_event_proof_skip_budget +=
            queue_proof_stats.skip_budget;
      }
      if (objective_pruned > 0) {
        learned_pruned += objective_pruned;
        out.bc_stats.objective_implied_event_prunes +=
            static_cast<std::uint64_t>(objective_pruned);
      }
      last_queue_objective_replay_epoch = current_objective_artifact_epoch();
      if (queue_objcap_nodes > 0) {
        out.bc_stats.conformance_objective_capacity_phases +=
            queue_objcap_nodes;
        out.bc_stats.conformance_objective_capacity_active_events +=
            queue_objcap_active;
        out.bc_stats.conformance_objective_capacity_active_tight_events +=
            queue_objcap_active_tight;
        out.bc_stats.conformance_objective_capacity_one_missing_events +=
            queue_objcap_one_events;
        out.bc_stats.conformance_objective_capacity_one_missing_keys +=
            queue_objcap_one_keys;
        out.bc_stats.conformance_objective_capacity_raw_exceed_keys +=
            queue_objcap_raw_exceed;
        out.bc_stats.conformance_objective_capacity_proof_exceed_keys +=
            queue_objcap_proof_exceed;
        out.bc_stats.conformance_objective_capacity_max_one_missing =
            std::max(out.bc_stats.conformance_objective_capacity_max_one_missing,
                     queue_objcap_max_one);
        out.bc_stats.conformance_objective_capacity_max_proof_excess =
            std::max(
                out.bc_stats.conformance_objective_capacity_max_proof_excess,
                queue_objcap_max_proof_excess);
        if (bc_conf_diag_enabled()) {
          fmt::print(stderr,
                     "[B&C-CONF] phase=queue_objective_capacity nodes={} "
                     "raw_cap_min={:.10g} proof_cap_min={:.10g} "
                     "active={}/{} one={}/{} rawEx={} proofEx={} "
                     "maxOne={:.6g} proofExMax={:.6g}\n",
                     queue_objcap_nodes, queue_objcap_min_raw_cap,
                     queue_objcap_min_proof_cap, queue_objcap_active_tight,
                     queue_objcap_active, queue_objcap_one_events,
                     queue_objcap_one_keys, queue_objcap_raw_exceed,
                     queue_objcap_proof_exceed, queue_objcap_max_one,
                     queue_objcap_max_proof_excess);
        }
      }
        if ((bc_conf_diag_enabled() || trace_bc_timeline) &&
	          (queue_proof_stats.candidates > 0 ||
	           queue_proof_stats.skip_coeff > 0 ||
	           queue_proof_stats.skip_budget > 0 ||
	           queue_proof_stats.skip_no_lp_or_queue_hit > 0 ||
                   queue_proof_stats.rc_frontier_candidates > 0)) {
	        fmt::print(stderr,
	                   "[B&C-CONF] phase=queue_objective_proof_capacity "
	                   "candidates={} fixings={} reasons={} prunes={} "
	                   "skipCoeff={} skipBudget={} skipNoHit={} maxLift={:.6g} "
	                   "maxExcess={:.6g} rcFrontier={}/{} rcSkipBudget={}\n",
	                   queue_proof_stats.candidates,
	                   queue_proof_stats.fixings, objective_reasons_added,
	                   queue_proof_stats.prunes, queue_proof_stats.skip_coeff,
	                   queue_proof_stats.skip_budget,
	                   queue_proof_stats.skip_no_lp_or_queue_hit,
	                   queue_proof_stats.max_proof_lift,
	                   queue_proof_stats.max_proof_excess,
	                   queue_proof_stats.rc_frontier_fixings,
                           queue_proof_stats.rc_frontier_candidates,
                           queue_proof_stats.rc_frontier_skip_budget);
	      }
    }
		    if (learned_pruned > 0) {
	      out.bc_stats.rc_conflict_queue_prunes +=
	          static_cast<std::uint64_t>(learned_pruned);
    }
    pruned += learned_pruned;
    const std::uint64_t queue_tightenings_for_refresh =
        closure_tightened_total + objective_tightened_total;
    const bool mandatory_frontier_refresh =
        proof_tail_queue_mode_active() || force_frontier_drain ||
        cutoff_strictly_improved;
    if (queue_tightenings_for_refresh > 0 &&
        (mandatory_frontier_refresh || rc_fixing_resolve_lps_left > 0)) {
      const double refresh_band =
          std::max(termination_bound_band(), 2.0 * opt.lp_tol);
      const int marked_frontier_nodes =
          node_queue.count_lp_refresh_needed_near_lower_bound(
              refresh_band, /*max_scan=*/8192);
      const int pass_budget = mandatory_frontier_refresh
          ? marked_frontier_nodes
          : std::min(
                rc_fixing_resolve_lps_left,
                marked_frontier_nodes > 0
                    ? std::min(marked_frontier_nodes,
                               std::max(1,
                                        opt.reduced_cost_fixing_resolve_max_lps /
                                            2))
                    : std::max(1,
                               opt.reduced_cost_fixing_resolve_max_lps / 8));
		      const int refresh_pruned = refresh_marked_queue_nodes(
		          reason != nullptr ? reason : "queue_domain_learning", pass_budget,
		          refresh_band, mandatory_frontier_refresh);
		      pruned += refresh_pruned;
		    }
    refresh_queue_lower_bound();
    const double queue_lb_after = node_queue.lower_bound();
    record_queue_lower_bound_audit("queue_domain_learning",
                     reason != nullptr ? reason
                             : "domain_learning",
                     queue_lb_before, queue_lb_after);
    const int queue_size_after = node_queue.size();
    const std::uint64_t queue_tightenings =
        closure_tightened_total + cutpool_tightened_total +
        objective_tightened_total;
    const double queue_lift =
        (std::isfinite(queue_lb_before) && std::isfinite(queue_lb_after))
            ? std::max(0.0, queue_lb_after - queue_lb_before)
            : 0.0;
    ++out.bc_stats.conformance_queue_phases;
    out.bc_stats.conformance_queue_tightenings += queue_tightenings;
    out.bc_stats.conformance_queue_prunes += static_cast<std::uint64_t>(
        std::max(0, pruned));
    if (queue_lift > std::max(1e-8, opt.lp_tol)) {
      ++out.bc_stats.conformance_queue_bound_moves;
      out.bc_stats.conformance_queue_lift_sum += queue_lift;
      out.bc_stats.conformance_queue_lift_max =
          std::max(out.bc_stats.conformance_queue_lift_max, queue_lift);
    }
    if (bc_conf_diag_enabled()) {
      fmt::print(stderr,
                 "[B&C-CONF] phase=queue_domain_learning sources={} "
                 "qsize={}->{} qlb={:.10g}->{:.10g} lift={:.6g} "
                 "direct_pruned={} closure_tight={} closure_pruned={} "
                 "cutpool_tight={} cutpool_pruned={} gap_pruned={} "
                 "obj_tight={} obj_pruned={} total_pruned={}\n",
                 queue_sources, queue_size_before, queue_size_after,
                 queue_lb_before, queue_lb_after, queue_lift,
                 direct_incumbent_pruned, closure_tightened_total,
                 closure_pruned_total, cutpool_tightened_total,
                 cutpool_pruned_total, direct_gap_pruned,
                 objective_tightened_total, objective_pruned_total, pruned);
    }
    if (has_incumbent && std::isfinite(incumbent_obj)) {
      last_queue_domain_replay_incumbent = incumbent_obj;
    }
    last_queue_domain_replay_epoch = current_domain_closure_epoch();
    const bool progressed =
        queue_tightenings > 0 || pruned > 0 ||
        queue_lift > std::max(1e-8, opt.lp_tol);
    if (force_frontier_drain || cutoff_strictly_improved || progressed) {
      stale_queue_domain_replay_passes = 0;
    } else {
      ++stale_queue_domain_replay_passes;
    }
    return pruned;
  };

  int post_incumbent_local_branching_attempts = 0;
  double last_local_branching_center_obj = kInf;
  auto maybe_run_post_incumbent_local_branching = [&](int depth) -> bool {
    if (opt._is_stage3_submip ||
        !has_incumbent || incumbent_x.size() != n ||
        !std::isfinite(incumbent_obj) || base_lp.sense != Sense::Minimize ||
        root_frac_bin_count <= 0 || root_frac_bin_count > 512) {
      return false;
    }
    if (requested_vendored_highs_lp && !opt.enable_incumbent_local_branching) {
      if (opt.verbose || trace_bc_timeline ||
          bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
        fmt::print(stderr,
                   "[B&C-LB-POLISH] depth={} skipped="
                   "proof_contract_local_branching_not_certificate\n",
                   depth);
      }
      return false;
    }

    refresh_queue_lower_bound();
    if (!std::isfinite(global_lb)) return false;
    const double rel_gap =
        std::max(0.0, (incumbent_obj - global_lb) /
                            std::max(1.0, std::abs(incumbent_obj)));
    if (rel_gap <= opt.gap_tol) {
      return false;
    }

    const double gap_limit_incumbent =
        global_lb / std::max(1e-12, 1.0 - std::max(0.0, opt.gap_tol));
    const double improvement_needed =
        std::isfinite(gap_limit_incumbent)
            ? std::max(0.0, incumbent_obj - gap_limit_incumbent)
            : kInf;
    const bool termination_polish =
        (std::isfinite(improvement_needed) && improvement_needed > 0.0 &&
         rel_gap <= std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5));
    if (!opt.enable_incumbent_local_branching && !termination_polish) {
      return false;
    }
    if (opt.enable_incumbent_local_branching && !termination_polish &&
        rel_gap <= std::max(2.0 * opt.gap_tol, 1.0e-4)) {
      return false;
    }

    const int configured_max_attempts =
        std::max(0, opt.incumbent_local_branching_max_iters);
    const int max_attempts = opt.enable_incumbent_local_branching
        ? configured_max_attempts
        : (termination_polish ? 1 : 0);
    if (post_incumbent_local_branching_attempts >= max_attempts) {
      return false;
    }
    if (std::isfinite(last_local_branching_center_obj) &&
        incumbent_obj >= last_local_branching_center_obj -
                             std::max(1e-7, 1e-10 * std::abs(incumbent_obj))) {
      return false;
    }

    const double remaining_s =
        std::max(0.0, opt.time_limit_sec -
                          std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t0).count());
    const double requested_budget =
        termination_polish && !opt.enable_incumbent_local_branching
            ? std::max(0.05, std::min(1.5, 0.02 * opt.time_limit_sec))
            : std::max(0.05, opt.incumbent_local_branching_time_limit);
    const double sub_budget = std::min(requested_budget, 0.04 * remaining_s);
    if (sub_budget < 0.05) return false;

    const int init_radius =
        std::max(1, opt.incumbent_local_branching_initial_radius);
    const int max_radius =
        std::max(init_radius, opt.incumbent_local_branching_max_radius);
    int radius = init_radius;
    if (termination_polish && !opt.enable_incumbent_local_branching) {
      radius = std::min(max_radius, std::max(init_radius, 8));
    }
    for (int k = 0; k < post_incumbent_local_branching_attempts; ++k) {
      radius = std::min(max_radius, radius * 2);
    }

    MIPModel sub_prob;
    sub_prob.linear_part = base_lp;
    for (int j = 0; j < n; ++j) {
      sub_prob.linear_part.vars[static_cast<std::size_t>(j)].lb = root.lb[j];
      sub_prob.linear_part.vars[static_cast<std::size_t>(j)].ub = root.ub[j];
    }

    Eigen::SparseVector<double> lb_row(n);
    int ones = 0;
    int active_bins = 0;
    for (int j = 0; j < n; ++j) {
      if (base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
        continue;
      }
      if (root.ub[j] - root.lb[j] <= 1e-9) continue;
      const double v = std::round(incumbent_x[j]);
      if (v >= 0.5) {
        lb_row.coeffRef(j) = -1.0;
        ++ones;
      } else {
        lb_row.coeffRef(j) = 1.0;
      }
      ++active_bins;
    }
    if (active_bins == 0) return false;
    lb_row.prune(0.0, 0.0);
    const double lb_rhs = static_cast<double>(radius - ones);

    Eigen::SparseVector<double> obj_cut(n);
    int obj_nnz = 0;
    for (int j = 0; j < n; ++j) {
      const double cj = base_lp.c[j];
      if (!std::isfinite(cj)) return false;
      if (std::abs(cj) > 0.0) {
        obj_cut.coeffRef(j) = cj;
        ++obj_nnz;
      }
    }
    if (obj_nnz == 0) return false;
    obj_cut.prune(0.0, 0.0);
    const double improve_tol =
        std::max(1e-6, 1e-10 * std::max(1.0, std::abs(incumbent_obj)));
    const double obj_rhs = incumbent_obj - improve_tol;

    std::vector<Eigen::SparseVector<double>> rows;
    std::vector<double> rhs;
    rows.reserve(2);
    rhs.reserve(2);
    rows.push_back(std::move(lb_row));
    rhs.push_back(lb_rhs);
    rows.push_back(std::move(obj_cut));
    rhs.push_back(obj_rhs);
    add_sparse_rows_to_lp(sub_prob.linear_part, rows, rhs);

    sub_prob.integer_idx.clear();
    sub_prob.binary_idx.clear();
    for (int j = 0; j < n; ++j) {
      const auto type =
          sub_prob.linear_part.vars[static_cast<std::size_t>(j)].type;
      if (type == VarType::Integer) {
        sub_prob.integer_idx.push_back(j);
      } else if (type == VarType::Binary) {
        sub_prob.binary_idx.push_back(j);
      }
    }
    sub_prob.initial_solution = incumbent_x;

    ++post_incumbent_local_branching_attempts;
    last_local_branching_center_obj = incumbent_obj;
    ++out.bc_stats.incumbent_local_branching_attempts;
    const auto lb0 = std::chrono::steady_clock::now();
    BCOptions sub_opt;
    sub_opt._is_stage3_submip = true;
    sub_opt.time_limit_sec = sub_budget;
    sub_opt.max_nodes =
        termination_polish && !opt.enable_incumbent_local_branching
            ? std::max(1, std::min(500,
                                   2 * opt.incumbent_local_branching_node_limit))
            : std::max(1, opt.incumbent_local_branching_node_limit);
    sub_opt.max_lp_iter = opt.max_lp_iter;
    sub_opt.int_tol = opt.int_tol;
    sub_opt.gap_tol = 0.0;
    sub_opt.lp_tol = opt.lp_tol;
    sub_opt.root_cut_rounds = 0;
    sub_opt.cuts_per_round = 0;
    sub_opt.max_cut_depth = 0;
    sub_opt.cuts = CutType::None;
    sub_opt.branching = opt.branching;
    sub_opt.node_sel = NodeSelection::Hybrid;
    sub_opt.use_simplex_lp_nodes = true;
    sub_opt.lp_kernel_backend = opt.lp_kernel_backend;
    sub_opt.use_ipm_root = false;
    sub_opt.use_ipm_nodes = false;
    sub_opt.use_feasibility_pump = false;
    sub_opt.enable_feasibility_jump = false;
    sub_opt.use_progressive_rounding = false;
    sub_opt.max_dive_lps = 0;
    sub_opt.max_probe_vars = 0;
    sub_opt.root_split_bound_probing = false;
    sub_opt.use_papilo_presolve = false;
    sub_opt.enable_lns = false;
    sub_opt.enable_incumbent_local_branching = false;
    sub_opt.enable_root_low_fractionality_rens = false;
    sub_opt.enable_objective_cutoff_conflict_cuts = false;
    sub_opt.enable_objective_cutoff_weighted_event_cuts = false;
    sub_opt.enable_objective_cutoff_domain_fixing = false;
    sub_opt.enable_dynamic_implied_bound_probing = false;
    sub_opt.enable_reduced_cost_proof_cut_resolve = false;
    sub_opt.num_threads = 1;
    sub_opt.verbose = false;

    BCResult sub_res = branch_and_cut_lp(sub_prob, sub_opt);
    const auto lb1 = std::chrono::steady_clock::now();
    const double lb_ms =
        std::chrono::duration<double, std::milli>(lb1 - lb0).count();
    out.bc_stats.incumbent_local_branching_time_ms += lb_ms;

    bool improved = false;
    if (sub_res.x.size() == n && std::isfinite(sub_res.stats.objective)) {
      Eigen::VectorXd xs = clamp_to_bounds(sub_res.x, root.lb, root.ub);
      for (int j = 0; j < n; ++j) {
        if (is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
          xs[j] = std::min(root.ub[j],
                           std::max(root.lb[j], std::round(xs[j])));
        }
      }
	      if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
	        const double obj = objective_value(base_lp.c, xs, base_lp.sense);
	        if (obj + improve_tol < incumbent_obj &&
	            adopt_root_incumbent("local_branching_incumbent", xs, obj)) {
	          seq_incumbent_bound.store(incumbent_obj, std::memory_order_relaxed);
	          note_incumbent_update(depth, global_lb);
	          tree_restart_controller.note_incumbent(
	              incumbent_obj, out.bc_stats.nodes_explored,
	              "local_branching_incumbent");
	          publish_root_bound_gap_certificate("local_branching_incumbent");
	          ++out.bc_stats.incumbent_local_branching_improvements;
	          improved = true;
	          prune_queue_after_domain_learning(
	              /*force_frontier_drain=*/false, "local_branching_incumbent");
	        }
	      }
    }
    if (opt.verbose || improved) {
      fmt::print(stderr,
                 "[B&C-LB-POLISH] radius={} gap={:.3g} need={:.6g} "
                 "budget={:.2f}s time={:.0f}ms improved={} obj={:.2f}\n",
                 radius, rel_gap, improvement_needed, sub_budget, lb_ms,
                 improved ? 1 : 0, incumbent_obj);
    }
    return improved;
  };

  int certificate_queue_passes_left = 48;
  int forced_certificate_queue_passes = 0;
  int stale_certificate_queue_passes = 0;
  double best_certificate_queue_gap = kInf;
  std::uint64_t last_certificate_queue_node = 0;
  double last_cached_dual_proof_consume_obj = kInf;
  std::uint64_t last_cached_dual_proof_consume_epoch =
      std::numeric_limits<std::uint64_t>::max();
  bool last_cached_dual_proof_consume_had_effect = false;
  auto cached_dual_proof_consumed_for_current_epoch = [&]() -> bool {
    if (!has_incumbent || !std::isfinite(incumbent_obj) ||
        !std::isfinite(last_cached_dual_proof_consume_obj)) {
      return false;
    }
    const double consume_obj_tol =
        std::max(1e-7, 1e-10 * std::max(1.0, std::abs(incumbent_obj)));
    return std::abs(incumbent_obj - last_cached_dual_proof_consume_obj) <=
               consume_obj_tol &&
           last_cached_dual_proof_consume_epoch ==
               current_domain_learning_epoch();
  };
  auto consume_dual_proof_for_queue =
      [&](const DualProofRow& proof, double proof_upper,
          const char* source,
          const std::vector<BranchDomainLiteral>* required_scope = nullptr)
          -> int {
    if (!proof.valid || proof.coeff.size() < n || proof.coeff.nonZeros() <= 0 ||
        !std::isfinite(proof_upper) || !std::isfinite(proof.rhs) ||
        node_queue.empty() ||
        (certificate_queue_passes_left <= 0 &&
         forced_certificate_queue_passes <= 0)) {
      return 0;
    }

    double proof_gap = proof.actual_lp_gap;
    if (!std::isfinite(proof_gap)) {
      proof_gap = proof.rhs - proof.lp_activity;
    }
    if (!std::isfinite(proof_gap)) proof_gap = kInf;
    proof_gap = std::max(0.0, proof_gap);
    const double band = 10.0 * termination_bound_band();
    const bool forced = forced_certificate_queue_passes > 0;
    const bool near_cutoff = proof_gap <= band;
    const bool meaningfully_better =
        proof_gap + 1e-7 < 0.90 * best_certificate_queue_gap;
    const bool periodic_near_cutoff =
        static_cast<std::uint64_t>(out.bc_stats.nodes_explored) >=
            last_certificate_queue_node + 256 &&
        proof_gap <= 4.0 * band;
    const bool spacing_ok =
        static_cast<std::uint64_t>(out.bc_stats.nodes_explored) >=
        last_certificate_queue_node + 256;
    if (!forced &&
        ((!near_cutoff && !meaningfully_better && !periodic_near_cutoff) ||
         (!spacing_ok && !meaningfully_better))) {
      return 0;
    }

    (void)required_scope;
	    int pruned = 0;
	    std::uint64_t scoped_domain_fixings = 0;
		    std::uint64_t scoped_certificates = 0;
		    std::uint64_t queue_proof_conflicts = 0;
		    std::uint64_t queue_lb_lift_nodes = 0;
		    std::uint64_t queue_lp_refresh_marked = 0;
		    double frontier_bound_move_sum = 0.0;
		    double frontier_bound_move_max = 0.0;
		    double lift_sum = 0.0;
		    double lift_max = 0.0;
	    const double frontier_band = forced ? kInf : std::max(band, 2.0 * proof_gap);
	    const int max_queue_nodes = forced ? 2048 : 256;
      const double queue_lb_before = node_queue.lower_bound();
	    const int queue_nodes_before = node_queue.size();
      const int tightened = 0;
				    const double queue_incumbent_upper_limit =
				        has_incumbent && std::isfinite(incumbent_obj)

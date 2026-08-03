// @file bc_run/12_node_processing_dual_proofs.inc
// BCSolveState::run() segment 12/14: single-child node processing (LP solve, pool separation, cut generation) and dual-proof consumption.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
				            ? incumbent_upper_limit()
				            : kInf;
				    const bool has_queue_upper_limit =
				        has_incumbent && std::isfinite(queue_incumbent_upper_limit);
				    const double queue_upper_limit_prune_tol =
				        has_queue_upper_limit
				            ? incumbent_upper_limit_prune_tol(
				                  queue_incumbent_upper_limit)
				            : kIncumbentPruneTol;
      const bool has_resolved_frontier_certificates =
          !global_bound_lifting_certificates.empty();
      // HiGHS does not let a raw row-dual proof scan the open queue.  The row
      // must first be explained through the local domain trail and republished
      // as a resolved frontier certificate or conflict.  This consumer is
      // therefore only a scheduling hook for already-published certificates.
    if (forced_certificate_queue_passes > 0) {
      --forced_certificate_queue_passes;
    }
	    if (tightened == 0 && pruned == 0 && scoped_domain_fixings == 0 &&
	        scoped_certificates == 0 && queue_proof_conflicts == 0 &&
	        queue_lb_lift_nodes == 0 && queue_lp_refresh_marked == 0 &&
	        frontier_bound_move_sum <= 0.0 &&
          !has_resolved_frontier_certificates) {
      ++out.bc_stats.termination_effective_skip_no_lp_or_queue_hit;
      if (certificate_queue_passes_left > 0) --certificate_queue_passes_left;
      last_certificate_queue_node =
          static_cast<std::uint64_t>(out.bc_stats.nodes_explored);
      ++out.bc_stats.certificate_queue_passes;
      last_queue_domain_replay_epoch = current_domain_closure_epoch();
      stale_queue_domain_replay_passes = 0;
      if (meaningfully_better) {
        best_certificate_queue_gap =
            std::min(best_certificate_queue_gap, proof_gap);
      }
      record_queue_lower_bound_audit("certificate_queue_noop",
                                     source != nullptr ? source
                                                       : "dual_proof",
                                     queue_lb_before, node_queue.lower_bound());
      return 0;
    }

    if (certificate_queue_passes_left > 0) --certificate_queue_passes_left;
    last_certificate_queue_node =
        static_cast<std::uint64_t>(out.bc_stats.nodes_explored);
    best_certificate_queue_gap =
        std::min(best_certificate_queue_gap, proof_gap);
    ++out.bc_stats.certificate_queue_passes;
    out.bc_stats.certificate_queue_tightenings +=
        static_cast<std::uint64_t>(tightened);
    out.bc_stats.certificate_queue_prunes +=
        static_cast<std::uint64_t>(pruned);
	    out.bc_stats.certificate_domain_fixings += scoped_domain_fixings;
	    out.bc_stats.scoped_conflict_objects_learned += scoped_certificates;
	    out.bc_stats.termination_effective_candidates +=
        static_cast<std::uint64_t>(
            std::max(0, std::min(queue_nodes_before, max_queue_nodes)));
		    out.bc_stats.termination_effective_admitted +=
		        scoped_domain_fixings + scoped_certificates +
		        queue_proof_conflicts +
		        static_cast<std::uint64_t>(std::max(0, pruned));
    out.bc_stats.certificate_queue_lift_sum += lift_sum;
    out.bc_stats.certificate_queue_lift_max =
        std::max(out.bc_stats.certificate_queue_lift_max, lift_max);
    out.bc_stats.certificate_queue_lower_bound_lift_nodes +=
        queue_lb_lift_nodes;
    out.bc_stats.certificate_queue_lp_refresh_marked +=
        queue_lp_refresh_marked;
    if (frontier_bound_move_sum > 0.0) {
      out.bc_stats.certificate_queue_frontier_bound_moves +=
          scoped_domain_fixings;
      out.bc_stats.certificate_queue_frontier_bound_move_sum +=
          frontier_bound_move_sum;
      out.bc_stats.certificate_queue_frontier_bound_move_max =
          std::max(out.bc_stats.certificate_queue_frontier_bound_move_max,
                   frontier_bound_move_max);
    }
    const double certificate_lift_tol =
        std::max(1e-6, 10.0 * std::max(opt.lp_tol, opt.int_tol));
    const bool termination_effective_pass =
        pruned > 0 || lift_max > certificate_lift_tol ||
        queue_lb_lift_nodes > 0 || queue_lp_refresh_marked > 0 ||
        frontier_bound_move_sum > 0.0;
    if (termination_effective_pass) {
      stale_certificate_queue_passes = 0;
    } else {
      ++stale_certificate_queue_passes;
      if (stale_certificate_queue_passes >= 3) {
        certificate_queue_passes_left = 0;
      }
    }
    if (pruned > 0) {
      out.bc_stats.rc_conflict_queue_prunes +=
          static_cast<std::uint64_t>(pruned);
    }
		    const bool have_bound_lifting_artifacts =
		        scoped_domain_fixings > 0 || scoped_certificates > 0 ||
		        queue_proof_conflicts > 0 ||
		        !global_bound_lifting_certificates.empty();
		    if (have_bound_lifting_artifacts &&
		        !global_bound_lifting_certificates.empty()) {
			      std::uint64_t cert_tightened = 0;
			      std::uint64_t cert_reasons = 0;
			      std::uint64_t cert_refresh_marked = 0;
			      double cert_move_sum = 0.0;
			      double cert_move_max = 0.0;
				      const int cert_pruned =
				          node_queue.propagate_bound_lifting_certificates(
			              base_lp.vars, global_bound_lifting_certificates,
			              has_queue_upper_limit, queue_incumbent_upper_limit,
			              queue_upper_limit_prune_tol,
			              &cert_tightened, &cert_reasons,
			              &cert_refresh_marked, &cert_move_sum,
			              &cert_move_max,
			              frontier_band, forced ? 2048 : 512,
			              &root.lb, &root.ub,
			              &domain_objective_lower_bound,
			              &replay_queue_reason_trail_certificates);
		      out.bc_stats.certificate_queue_lp_refresh_marked +=
		          cert_refresh_marked;
		      if (cert_move_sum > 0.0) {
		        out.bc_stats.certificate_queue_frontier_bound_moves +=
		            cert_tightened;
		        out.bc_stats.certificate_queue_frontier_bound_move_sum +=
		            cert_move_sum;
		        out.bc_stats.certificate_queue_frontier_bound_move_max =
		            std::max(out.bc_stats.certificate_queue_frontier_bound_move_max,
		                     cert_move_max);
		      }
		      if (cert_tightened > 0 || cert_reasons > 0) {
		        scoped_domain_fixings += cert_tightened;
		        scoped_certificates += cert_reasons;
		        out.bc_stats.certificate_queue_tightenings += cert_tightened;
		        out.bc_stats.certificate_domain_fixings += cert_tightened;
		        out.bc_stats.termination_effective_admitted +=
		            cert_tightened + cert_reasons;
		      }
		      if (cert_pruned > 0) {
		        pruned += cert_pruned;
		        out.bc_stats.rc_conflict_queue_prunes +=
		            static_cast<std::uint64_t>(cert_pruned);
		        out.bc_stats.certificate_queue_prunes +=
		            static_cast<std::uint64_t>(cert_pruned);
		        out.bc_stats.termination_effective_admitted +=
		            static_cast<std::uint64_t>(cert_pruned);
		      }
		    }
		    if (have_bound_lifting_artifacts && !conflict_pool.empty()) {
		      std::uint64_t conflict_tightened = 0;
		      std::uint64_t conflict_reasons = 0;
		      const int conflict_pruned =
		          node_queue.propagate_by_domain_object_with_reasons(
		              [&](const Node&,
		                  Eigen::VectorXd& lb,
		                  Eigen::VectorXd& ub,
		                  std::vector<DomainReasonBound>& reasons,
		                  int& tightened_by_conflict,
		                  int& pruned_by_conflict) {
		                tightened_by_conflict = 0;
		                pruned_by_conflict = 0;
		                std::vector<BoundChangeInfo> changes;
		                const bool ok = conflict_pool.propagate_with_reasons(
		                    base_lp.vars, lb, ub, &tightened_by_conflict,
		                    &changes, &reasons, opt.int_tol);
		                if (!ok || !bounds_consistent(lb, ub)) {
		                  pruned_by_conflict = 1;
		                  return false;
		                }
		                return true;
			              },
				              has_queue_upper_limit, queue_incumbent_upper_limit,
				              queue_upper_limit_prune_tol,
				              &conflict_tightened, &conflict_reasons,
			              frontier_band, forced ? 2048 : 512,
			              &root.lb, &root.ub,
			              &domain_objective_lower_bound, nullptr,
			              nullptr, nullptr,
			              &replay_queue_reason_trail_certificates);
		      if (conflict_tightened > 0 || conflict_reasons > 0) {
		        scoped_domain_fixings += conflict_tightened;
		        out.bc_stats.certificate_queue_tightenings +=
		            conflict_tightened;
		        out.bc_stats.certificate_domain_fixings +=
		            conflict_tightened;
		        out.bc_stats.termination_effective_admitted +=
		            conflict_tightened + conflict_reasons;
		        refresh_queue_lower_bound();
		      }
		      if (conflict_pruned > 0) {
		        pruned += conflict_pruned;
		        out.bc_stats.rc_conflict_queue_prunes +=
		            static_cast<std::uint64_t>(conflict_pruned);
		        out.bc_stats.certificate_queue_prunes +=
		            static_cast<std::uint64_t>(conflict_pruned);
		        out.bc_stats.termination_effective_admitted +=
		            static_cast<std::uint64_t>(conflict_pruned);
		      }
		    }
		    if ((scoped_domain_fixings > 0 || queue_proof_conflicts > 0) &&
		        !implication_graph.empty()) {
      std::uint64_t queued_tightened = 0;
		      const int replay_pruned = node_queue.prune_by_implications(
		          base_lp.vars, implication_graph,
		          conflict_pool.empty() ? nullptr : &conflict_pool,
		          has_queue_upper_limit, queue_incumbent_upper_limit, opt.int_tol,
		          &queued_tightened, frontier_band,
		          forced ? 2048 : 512, &root.lb, &root.ub);
      out.bc_stats.local_implications_applied += queued_tightened;
      if (replay_pruned > 0) {
        out.bc_stats.rc_conflict_queue_prunes +=
            static_cast<std::uint64_t>(replay_pruned);
        out.bc_stats.certificate_queue_prunes +=
            static_cast<std::uint64_t>(replay_pruned);
        pruned += replay_pruned;
      }
			    } else if ((scoped_domain_fixings > 0 || queue_proof_conflicts > 0) &&
			               !conflict_pool.empty()) {
	      const int replay_pruned = node_queue.prune_conflicts(conflict_pool);
	      if (replay_pruned > 0) {
	        out.bc_stats.rc_conflict_queue_prunes +=
            static_cast<std::uint64_t>(replay_pruned);
        out.bc_stats.certificate_queue_prunes +=
            static_cast<std::uint64_t>(replay_pruned);
	        pruned += replay_pruned;
	      }
		    }
	    const bool certificate_domain_moved =
	        scoped_domain_fixings > 0 || scoped_certificates > 0 ||
	        queue_proof_conflicts > 0 || queue_lb_lift_nodes > 0 ||
	        queue_lp_refresh_marked > 0 || frontier_bound_move_sum > 0.0;
	    if (have_bound_lifting_artifacts || certificate_domain_moved) {
	      // HiGHS treats conflict rows, cutpool rows and objective/cutoff
	      // propagation as one domain closure.  After a dual proof publishes
	      // resolved frontier certificates, replay the full queue-side closure
	      // immediately so new conflict/cutpool/objective activations are not
	      // left as a separate tail pass.
	      const int fixed_point_pruned = prune_queue_after_domain_learning(
	          proof_tail_queue_mode_active() || forced || certificate_domain_moved,
	          source != nullptr ? source : "certificate_queue_fixed_point");
	      if (fixed_point_pruned > 0) {
	        pruned += fixed_point_pruned;
	      }
	    }
		    if (have_bound_lifting_artifacts && rc_fixing_resolve_lps_left > 0) {
		      const int marked_frontier_nodes =
		          node_queue.count_lp_refresh_needed_near_lower_bound(
	              frontier_band, /*max_scan=*/8192);
      const int pass_budget = std::min(
          rc_fixing_resolve_lps_left,
          marked_frontier_nodes > 0
              ? std::min(marked_frontier_nodes,
                         proof_tail_queue_mode_active()
                             ? std::max(1, opt.reduced_cost_fixing_resolve_max_lps)
                             : std::max(1, opt.reduced_cost_fixing_resolve_max_lps / 2))
              : (forced ? 16
                        : std::max(1, opt.reduced_cost_fixing_resolve_max_lps / 8)));
	      pruned += refresh_marked_queue_nodes(source, pass_budget,
	                                           frontier_band,
                                               proof_tail_queue_mode_active() ||
                                                   forced);
	    }
	    refresh_queue_lower_bound();
    const double queue_lb_after = node_queue.lower_bound();
    record_queue_lower_bound_audit("certificate_queue",
                                   source != nullptr ? source
                                                     : "dual_proof",
                                   queue_lb_before, queue_lb_after);
    last_queue_domain_replay_epoch = current_domain_closure_epoch();
    stale_queue_domain_replay_passes = 0;
    const int certificate_effect =
        pruned + static_cast<int>(std::min<std::uint64_t>(
                     scoped_domain_fixings + scoped_certificates +
                         queue_proof_conflicts + queue_lb_lift_nodes +
                         queue_lp_refresh_marked,
                     static_cast<std::uint64_t>(
                         std::numeric_limits<int>::max())));
    if (opt.verbose) {
      fmt::print(stderr,
	                 "[B&C-CERT-QUEUE] source={} gap={:.6g} tightened={} "
		                 "domain_fix={} cert={} pruned={} lift_avg/max={:.6g}/{:.6g} "
		                 "proof_conf={} qlb={:.2f} stale={}\n",
		                 source != nullptr ? source : "dual_proof", proof_gap,
		                 tightened, scoped_domain_fixings, scoped_certificates, pruned,
	                 tightened > 0 ? lift_sum / static_cast<double>(tightened)
	                               : 0.0,
		                 lift_max, queue_proof_conflicts, global_lb,
		                 stale_certificate_queue_passes);
    }
    return certificate_effect;
  };

  auto consume_cached_dual_proof_after_incumbent = [&]() -> int {
    int incumbent_pruned = prune_queue_by_incumbent();
    const std::uint64_t objective_artifacts =
        consume_objective_cutoff_artifacts("incumbent");
    // HiGHS does not wait for newly learned artifacts before consuming the
    // improved cutoff.  A tighter incumbent can make already-known
    // objective/implication/conflict objects propagate, so replay them after
    // every incumbent improvement.
	    incumbent_pruned += prune_queue_after_domain_learning(
	        objective_artifacts > 0, "incumbent_cutoff");
    if (have_root_global_dual_proof &&
        std::isfinite(root_global_dual_proof_upper) &&
        std::isfinite(incumbent_obj) &&
        incumbent_obj + 1e-9 < root_global_dual_proof_upper) {
      DualProofRow tightened_proof = root_global_dual_proof;
      tightened_proof.rhs += incumbent_obj - root_global_dual_proof_upper;
      tightened_proof.actual_lp_gap =
          tightened_proof.rhs - tightened_proof.lp_activity;
      root_global_dual_proof = tightened_proof;
      root_global_dual_proof_upper = incumbent_obj;
      if (have_cached_queue_objective_event_proof &&
          cached_queue_objective_event_proof_is_global) {
        cached_queue_objective_event_proof = tightened_proof;
        cached_queue_objective_event_proof_upper = incumbent_obj;
      }
      const auto proof_artifacts = extract_dual_proof_literal_artifacts(
          base_lp, root_implied_integer_cols, root_global_dual_proof,
          root.lb, root.ub, implication_graph, clique_table, opt.int_tol,
          &conflict_pool, cts ? &cts->shared_conflicts : nullptr,
          &ensure_branch_priority());
      if (proof_artifacts.implications_added > 0 ||
          proof_artifacts.clique_edges_added > 0 ||
          proof_artifacts.conflict_clauses_added > 0) {
        out.bc_stats.rc_binary_implications_learned +=
            proof_artifacts.implications_added;
	        note_global_domain_learning(proof_artifacts.implications_added +
	                                    proof_artifacts.clique_edges_added +
	                                    proof_artifacts.conflict_clauses_added);
	        note_objective_artifact_learning(proof_artifacts.implications_added +
	                                         proof_artifacts.clique_edges_added);
	        (void)refresh_objective_domain_sources(
	            "incumbent_dual_proof_artifacts", /*force=*/true);
		        incumbent_pruned += prune_queue_after_domain_learning(
		            /*force_frontier_drain=*/true, "incumbent_dual_proof_artifacts");
	      }
      fmt::print(stderr,
                 "[B&C PROOF-SCOPED] incumbent full-proof rhs={:.6g} "
                 "gap={:.6g} proof_clq_cand={} conflicts={} confQ={} impl={} "
                 "clique_edges={} frontier_pri={} obj_parts={} arcs={}\n",
                 tightened_proof.rhs, tightened_proof.actual_lp_gap,
                 proof_artifacts.candidates,
                 proof_artifacts.conflicts_seen,
                 proof_artifacts.conflict_clauses_added,
                 proof_artifacts.implications_added,
                 proof_artifacts.clique_edges_added,
                 proof_artifacts.frontier_priority_updates,
                 objective_propagation.partitions.size(),
                 implication_graph.size());
    }
	    const double incumbent_upper_limit_now =
	        highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                         opt.int_tol);
	      if (have_cached_queue_dual_proof &&
	        std::isfinite(cached_queue_dual_proof_upper) &&
	        std::isfinite(incumbent_upper_limit_now) &&
	        incumbent_upper_limit_now + 1e-9 < cached_queue_dual_proof_upper) {
	      cached_queue_dual_proof.rhs +=
	          incumbent_upper_limit_now - cached_queue_dual_proof_upper;
	      cached_queue_dual_proof.actual_lp_gap =
	          cached_queue_dual_proof.rhs - cached_queue_dual_proof.lp_activity;
	      cached_queue_dual_proof_upper = incumbent_upper_limit_now;
      if (have_cached_queue_objective_event_proof &&
          cached_queue_objective_event_proof_is_global &&
          std::isfinite(cached_queue_objective_event_proof_upper) &&
          incumbent_upper_limit_now + 1e-9 <
              cached_queue_objective_event_proof_upper) {
        cached_queue_objective_event_proof.rhs +=
            incumbent_upper_limit_now -
            cached_queue_objective_event_proof_upper;
        cached_queue_objective_event_proof.actual_lp_gap =
            cached_queue_objective_event_proof.rhs -
            cached_queue_objective_event_proof.lp_activity;
        cached_queue_objective_event_proof_upper =
            incumbent_upper_limit_now;
      }
    } else if (!have_cached_queue_dual_proof && have_root_global_dual_proof) {
      cached_queue_dual_proof = root_global_dual_proof;
      cached_queue_dual_proof_upper = root_global_dual_proof_upper;
      have_cached_queue_dual_proof = true;
      cached_queue_dual_proof_is_global = true;
      cached_queue_dual_proof_scope.clear();
      cached_queue_objective_event_proof = root_global_dual_proof;
      have_cached_queue_objective_event_proof = true;
      cached_queue_objective_event_proof_is_global = true;
      cached_queue_objective_event_proof_upper = root_global_dual_proof_upper;
      cached_queue_objective_event_proof_scope.clear();
    }
	    if (!have_cached_queue_dual_proof || node_queue.empty() ||
	        !std::isfinite(cached_queue_dual_proof_upper)) {
	      return incumbent_pruned;
	    }
    if (cached_dual_proof_consumed_for_current_epoch()) {
      return incumbent_pruned;
    }
	    const bool strict_frontier_ready =
	        proof_tail_queue_mode_active();
	    const std::vector<BranchDomainLiteral>* cached_scope =
	        cached_queue_dual_proof_is_global ? nullptr
	                                          : &cached_queue_dual_proof_scope;
	    if (!cached_queue_dual_proof_is_global &&
	        (cached_scope == nullptr || cached_scope->empty())) {
	      return incumbent_pruned;
	    }
	    if (!strict_frontier_ready && !cached_queue_dual_proof_is_global) {
	      return incumbent_pruned;
	    }
	    forced_certificate_queue_passes =
	        std::max(forced_certificate_queue_passes, 1);
	    ++out.bc_stats.incumbent_certificate_queue_consumptions;
    const int proof_pruned =
        consume_dual_proof_for_queue(cached_queue_dual_proof,
                                     cached_queue_dual_proof_upper,
                                     "incumbent_cached_proof", cached_scope);
    last_cached_dual_proof_consume_obj = incumbent_obj;
	    last_cached_dual_proof_consume_epoch = current_domain_learning_epoch();
	    last_cached_dual_proof_consume_had_effect = proof_pruned > 0;
		    return incumbent_pruned + proof_pruned;
	  };

	  highs_style_post_incumbent_domain_closure =
	      [&](const char* source, int depth) -> int {
	    if (tree_incumbent_domain_closure_active) {
	      tree_incumbent_domain_closure_pending = true;
	      tree_incumbent_domain_closure_source = source != nullptr
	                                                 ? source
	                                                 : "tree_incumbent";
	      tree_incumbent_domain_closure_depth = depth;
	      return 0;
	    }
	    if (!has_incumbent || !std::isfinite(incumbent_obj)) {
	      tree_incumbent_domain_closure_pending = false;
	      return 0;
	    }
	    tree_incumbent_domain_closure_active = true;
	    const std::string source_string =
	        source != nullptr ? source : "tree_incumbent";
	    int pruned = 0;
	    tree_incumbent_domain_closure_pending = false;
	    const double root_bound_before = root.bound;
	    const Eigen::VectorXd root_lb_before = root.lb;
	    const Eigen::VectorXd root_ub_before = root.ub;
	    run_root_incumbent_propagation(source_string.c_str());
	    const auto root_domain_delta =
	        count_bound_changes(root_lb_before, root_ub_before, root.lb, root.ub);
		    if (root_domain_delta.first > 0 || root_domain_delta.second > 0 ||
		        root.bound > root_bound_before + std::max(1e-8, opt.lp_tol)) {
		      update_standard_form_bounds(tree_base_sf, base_lp, root.lb, root.ub);
		      update_standard_form_bounds(node_sf, base_lp, root.lb, root.ub);
		      pruned += inherit_root_domain_to_queue(source_string.c_str(),
		                                            /*force_frontier_drain=*/true);
		      pruned += prune_queue_after_domain_learning(
		          /*force_frontier_drain=*/true,
		          "tree_incumbent_domain_inherit");
		    }
		    pruned += prune_queue_after_domain_learning(
		        /*force_frontier_drain=*/true, source_string.c_str());
		    pruned += consume_cached_dual_proof_after_incumbent();
		    pruned += prune_queue_by_gap_optimality_limit(source_string.c_str());
		    refresh_queue_lower_bound();
		    const bool rerun_requested = tree_incumbent_domain_closure_pending;
		    const std::string rerun_source = tree_incumbent_domain_closure_source;
	    const int rerun_depth = tree_incumbent_domain_closure_depth;
	    tree_incumbent_domain_closure_pending = false;
	    tree_incumbent_domain_closure_active = false;
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C-POSTINC-CLOSURE] source={} depth={} root_delta={}/{} "
	                 "root_bound={:.10g}->{:.10g} pruned={} qsize={} "
	                 "qlb={:.10g} ub={:.10g}\n",
	                 source_string, depth, root_domain_delta.first,
		                 root_domain_delta.second, root_bound_before, root.bound,
		                 pruned, node_queue.size(), global_lb, incumbent_obj);
		    }
		    if (rerun_requested) {
		      pruned += highs_style_post_incumbent_domain_closure(
		          rerun_source.c_str(), rerun_depth);
		    }
		    return pruned;
		  };

	  auto maybe_restart_tree_after_incumbent = [&]() -> bool {
	    if (!tree_restart_controller.pending() || node_queue.empty()) {
	      return false;
	    }
	    const double remaining_time =
	        bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
	    if (!tree_restart_controller.ready(
	            out.bc_stats.nodes_explored, node_queue.size(),
	            current_incumbent_valid_for_proof("tree_restart"),
	            remaining_time)) {
	      return false;
	    }

	    const int open_nodes_before = node_queue.size();
	    const double queue_bound_before = node_queue.lower_bound();
	    const std::string restart_source =
	        tree_restart_controller.pending_source();
	    std::uint64_t pseudocost_observations = 0;
	    for (int col : pc.active_cols()) {
	      const PseudoCost& cost = pc[col];
	      pseudocost_observations += static_cast<std::uint64_t>(
	          std::max(0, cost.down_cnt) + std::max(0, cost.up_cnt));
	    }

	    Node restart_root = root;
	    restart_root.compact_domain.clear();
	    restart_root.compact_domain_size = 0;
	    // A restarted root may reuse only solve-global learning.  Make that
	    // ownership boundary explicit instead of relying on the current root
	    // object happening to have empty node-local payloads.
	    restart_root.branch_reasons.clear();
	    restart_root.domain_reason_bounds.clear();
	    restart_root.local_domain_trail.clear();
	    restart_root.local_branch_positions.clear();
	    restart_root.dynamic_probe_literal_keys.clear();
	    restart_root.local_binary_implications.clear();
	    restart_root.local_conflict_clauses.clear();
	    restart_root.scoped_conflict_clauses.clear();
	    restart_root.local_cuts.clear();
	    restart_root.nlp_down_scores.reset();
	    restart_root.nlp_up_scores.reset();
	    if (root.x_relax.size() == n) {
	      restart_root.x_seed =
	          clamp_to_bounds(root.x_relax, root.lb, root.ub);
	    } else if (incumbent_x.size() == n) {
	      restart_root.x_seed =
	          clamp_to_bounds(incumbent_x, root.lb, root.ub);
	    } else {
	      restart_root.x_seed.resize(0);
	    }
	    restart_root.x_relax.resize(0);
	    restart_root.fractional_branchables.clear();
	    restart_root.fractional_branchables_valid = false;
	    restart_root.has_live_factor_telemetry = false;
	    restart_root.live_factor_telemetry = {};
	    restart_root.ipm_iterations = 0;
	    restart_root.lp_refresh_needed = true;
	    restart_root.bound = root.bound;
	    restart_root.estimate = root.bound;
	    restart_root.depth = 0;
	    restart_root.domain_learning_epoch = current_domain_learning_epoch();
	    restart_root.objective_artifact_epoch =
	        current_objective_artifact_epoch();
	    restart_root.domain_closure_epoch = current_domain_closure_epoch();

	    const int discarded = node_queue.discard_all();
	    node_queue.configure_domain_signature(
	        declared_branchable_cols, root.lb, root.ub,
	        std::max(1e-9, opt.int_tol));
	    node_queue.push_priority(std::move(restart_root));
	    tree_base_sf = ensure_base_sf();
	    update_standard_form_bounds(tree_base_sf, base_lp, root.lb, root.ub);
	    node_sf = tree_base_sf;
	    node_sf_cuts_augmented = false;
	    gap_limit_lower_bound_certificate = -kInf;
	    tree_incumbent_domain_closure_pending = false;
	    tree_restart_controller.commit(out.bc_stats.nodes_explored);

	    ++out.bc_stats.tree_restarts;
	    out.bc_stats.tree_restart_nodes_discarded +=
	        static_cast<std::uint64_t>(std::max(0, discarded));
	    ++out.bc_stats.tree_restart_root_requeues;
	    out.bc_stats.tree_restart_cut_pool_rows_preserved +=
	        static_cast<std::uint64_t>(std::max(0, cut_pool.size()));
	    out.bc_stats.tree_restart_conflicts_preserved +=
	        static_cast<std::uint64_t>(std::max(0, conflict_pool.size()));
	    out.bc_stats.tree_restart_implications_preserved +=
	        static_cast<std::uint64_t>(implication_graph.size());
	    out.bc_stats.tree_restart_clique_edges_preserved +=
	        static_cast<std::uint64_t>(clique_table.n_literal_edges());
	    out.bc_stats.tree_restart_pseudocost_observations_preserved +=
	        pseudocost_observations;
	    out.bc_stats.tree_restart_last_node = out.bc_stats.nodes_explored;
	    out.bc_stats.tree_restart_last_source = restart_source;
	    refresh_queue_lower_bound();

	    if (opt.verbose || trace_bc_timeline) {
	      fmt::print(stderr,
	                 "[B&C-RESTART] count={} source={} node={} open={} "
	                 "discarded={} qlb_before={:.10g} root_bound={:.10g} "
	                 "cuts={} conflicts={} implications={} clique={} "
	                 "pc_obs={} remaining={:.3f}s\n",
	                 out.bc_stats.tree_restarts, restart_source,
	                 out.bc_stats.nodes_explored, open_nodes_before, discarded,
	                 queue_bound_before, root.bound, cut_pool.size(),
	                 conflict_pool.size(), implication_graph.size(),
	                 clique_table.n_literal_edges(), pseudocost_observations,
	                 remaining_time);
	    }
	    return true;
	  };

	  auto separate_local_probed_implied_bound_cuts =
      [&](Node& child,
          std::vector<PoolCut>& local_proof_cuts,
          double inc_obj_snapshot,
          bool has_inc_snapshot) -> int {
	    if (!allow_native_proof_cut_rows ||
	        !opt.enable_dynamic_implied_bound_probing ||
	        !opt.enable_graph_implied_bound_cuts ||
	        child.x_relax.size() != n ||
        opt.dynamic_implied_bound_probing_max_vars <= 0 ||
        opt.dynamic_implied_bound_probing_max_rows <= 0) {
      return 0;
    }
    if (!has_inc_snapshot || !std::isfinite(inc_obj_snapshot) ||
        !std::isfinite(child.bound)) {
      return 0;
    }
    const double rel_frontier_gap =
        std::max(0.0, (inc_obj_snapshot - child.bound) /
                          std::max(1.0, std::abs(inc_obj_snapshot)));
    const double frontier_limit =
        std::max(opt.gap_tol, opt.dynamic_implied_bound_frontier_gap_factor *
                              std::max(1e-12, opt.gap_tol));
    if (rel_frontier_gap > frontier_limit) {
      return 0;
    }

    struct ProbeCandidate {
      int var{-1};
      double dist_to_half{0.0};
    };
    std::vector<ProbeCandidate> candidates;
    candidates.reserve(32);
    for (int j = 0; j < n; ++j) {
      if (base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
        continue;
      }
      if (child.lb[j] > 1e-9 || child.ub[j] < 1.0 - 1e-9) continue;
      const double xj = child.x_relax[j];
      if (!std::isfinite(xj) || xj <= opt.int_tol ||
          xj >= 1.0 - opt.int_tol) {
        continue;
      }
      candidates.push_back({j, std::abs(xj - 0.5)});
    }
    if (candidates.empty()) return 0;
    std::sort(candidates.begin(), candidates.end(),
              [](const ProbeCandidate& a, const ProbeCandidate& b) {
                if (a.dist_to_half != b.dist_to_half) {
                  return a.dist_to_half < b.dist_to_half;
                }
                return a.var < b.var;
              });

    const int max_vars = std::min<int>(
        candidates.size(), std::max(0, opt.dynamic_implied_bound_probing_max_vars));
    const int row_budget =
        std::max(0, opt.dynamic_implied_bound_probing_max_rows);
    int rows_added = 0;
    std::unordered_set<std::string> seen_rows;
    std::unordered_set<std::uint64_t> seen_probe_literals(
        child.dynamic_probe_literal_keys.begin(),
        child.dynamic_probe_literal_keys.end());
    auto probe_literal_key = [](int var, bool value_one) -> std::uint64_t {
      return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(var)) << 1) |
             static_cast<std::uint64_t>(value_one);
    };

    auto add_single_bound_cut = [&](int trigger, bool forbid_one) {
      if (rows_added >= row_budget) return;
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
      const double viol = sparse_dot(row, child.x_relax) - rhs;
      ++out.bc_stats.dynamic_implied_bound_rows_generated;
      if (viol <= opt.dynamic_implied_bound_min_violation) {
        ++out.bc_stats.dynamic_implied_bound_rows_rejected;
        ++out.bc_stats.dynamic_implied_bound_rows_skip_no_violation;
        return;
      }
      const std::string key =
          fmt::format("single:{}:{}", trigger, forbid_one ? 1 : 0);
      if (!seen_rows.insert(key).second) return;
      const std::size_t h = sparse_cut_hash(row);
      if (contains_identical_local_cut(child, local_proof_cuts, row, rhs)) {
        ++out.bc_stats.dynamic_implied_bound_rows_rejected;
        ++out.bc_stats.dynamic_implied_bound_rows_skip_not_active;
        return;
      }
      ++out.bc_stats.dynamic_implied_bound_rows_violated;
      ++out.bc_stats.dynamic_implied_bound_rows_added;
      ++out.bc_stats.dynamic_implied_bound_rows_active_implication;
      ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
      ++out.bc_stats.local_node_cuts_generated;
      PoolCut cut{std::move(row), rhs, 0, viol, 1.0, h};
      local_proof_cuts.push_back(make_local_node_cut(std::move(cut)));
      ++rows_added;
    };

    BinaryImplicationGraph local_graph(n);
    for (int ci = 0; ci < max_vars && rows_added < row_budget; ++ci) {
      const int trigger = candidates[static_cast<std::size_t>(ci)].var;
      for (bool value_one : {false, true}) {
        if (rows_added >= row_budget) break;
        const std::uint64_t probe_key = probe_literal_key(trigger, value_one);
        if (!seen_probe_literals.insert(probe_key).second) {
          continue;
        }
        child.dynamic_probe_literal_keys.push_back(probe_key);
        Eigen::VectorXd probe_lb = child.lb;
        Eigen::VectorXd probe_ub = child.ub;
        if (value_one) {
          probe_lb[trigger] = std::max(probe_lb[trigger], 1.0);
        } else {
          probe_ub[trigger] = std::min(probe_ub[trigger], 0.0);
        }
        if (!bounds_consistent(probe_lb, probe_ub)) {
          add_single_bound_cut(trigger, value_one);
          continue;
        }

        std::vector<BoundChangeInfo> probe_changes;
        std::vector<BranchDomainLiteral> probe_conflict;
        int probe_tightened = 0;
        const bool ok = propagate_node_domain(
            base_lp, A_row_cached, Aeq_row_cached, row_index_cached,
            probe_lb, probe_ub, opt.bound_propagation_rounds,
            &conflict_pool, &clique_table, &implication_graph,
            probe_changes, child.branch_reasons, &probe_conflict,
            &probe_tightened, nullptr);
        if (!ok || !bounds_consistent(probe_lb, probe_ub)) {
          add_single_bound_cut(trigger, value_one);
          continue;
        }

        for (const auto& chg : probe_changes) {
          const int k = chg.var_idx;
          if (k < 0 || k >= n || k == trigger) continue;
          if (chg.is_lb) {
            if (probe_lb[k] > child.lb[k] + 1e-9) {
              local_graph.add_implication(trigger, value_one, k, true,
                                          probe_lb[k]);
              add_local_binary_implication(child, trigger, value_one, k, true,
                                           probe_lb[k]);
            }
          } else {
            if (probe_ub[k] < child.ub[k] - 1e-9) {
              local_graph.add_implication(trigger, value_one, k, false,
                                          probe_ub[k]);
              add_local_binary_implication(child, trigger, value_one, k, false,
                                           probe_ub[k]);
            }
          }
        }
      }
    }

    if (!local_graph.empty() && rows_added < row_budget) {
      std::vector<Eigen::SparseVector<double>> ib_rows;
      std::vector<double> ib_rhs;
      const int added = append_graph_implied_bound_cuts(
          base_lp, child.x_relax, child.lb, child.ub, local_graph,
          row_budget - rows_added, ib_rows, ib_rhs);
      for (int i = 0; i < added; ++i) {
        Eigen::SparseVector<double>& row = ib_rows[static_cast<std::size_t>(i)];
        const double rhs = ib_rhs[static_cast<std::size_t>(i)];
        ++out.bc_stats.dynamic_implied_bound_rows_generated;
        const double viol = sparse_dot(row, child.x_relax) - rhs;
        if (viol <= opt.dynamic_implied_bound_min_violation) {
          ++out.bc_stats.dynamic_implied_bound_rows_rejected;
          ++out.bc_stats.dynamic_implied_bound_rows_skip_no_violation;
          continue;
        }
        bool has_bound_move = false;
        for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
          if (std::abs(it.value()) >= opt.dynamic_implied_bound_min_bound_move) {
            has_bound_move = true;
            break;
          }
        }
        if (!has_bound_move) {
          ++out.bc_stats.dynamic_implied_bound_rows_rejected;
          ++out.bc_stats.dynamic_implied_bound_rows_skip_small_move;
          continue;
        }
        const std::size_t h = sparse_cut_hash(row);
        if (contains_identical_local_cut(child, local_proof_cuts, row, rhs)) {
          ++out.bc_stats.dynamic_implied_bound_rows_rejected;
          ++out.bc_stats.dynamic_implied_bound_rows_skip_not_active;
          continue;
        }
        const double norm =
            std::sqrt(std::max(0.0, row.squaredNorm()));
        ++out.bc_stats.dynamic_implied_bound_rows_violated;
        ++out.bc_stats.dynamic_implied_bound_rows_added;
        ++out.bc_stats.dynamic_implied_bound_rows_active_implication;
        ++out.bc_stats.dynamic_implied_bound_rows_bound_move;
        ++out.bc_stats.local_node_cuts_generated;
        PoolCut cut{std::move(row), rhs, 0,
                    viol / std::max(1.0, norm), norm, h};
        local_proof_cuts.push_back(
            make_local_node_cut(std::move(cut)));
        ++rows_added;
      }
    }

    return rows_added;
  };

  auto append_transformed_fractional_frontier_cuts =
      [&](Node& child,
          const SimplexResult& simplex,
          const std::shared_ptr<BasisOps>& sbasis,
          std::vector<PoolCut>& local_proof_cuts,
          int max_rows) -> int {
	    if (max_rows <= 0 || !simplex.exact_optimal ||
	        child.x_relax.size() != n ||
	        !allow_native_proof_cut_rows ||
	        rc_proof_cut_resolve_lps_left <= 0) {
      return 0;
    }

    std::vector<char> fractional_frontier(static_cast<std::size_t>(n), 0);
    int frontier_cols = 0;
    for (int j = 0; j < n; ++j) {
      const bool integral_like =
          is_integer_type(base_lp.vars[static_cast<std::size_t>(j)]) ||
          (j < static_cast<int>(root_implied_integer_cols.size()) &&
           root_implied_integer_cols[static_cast<std::size_t>(j)] != 0);
      if (!integral_like) continue;
      const double xj = child.x_relax[j];
      if (!std::isfinite(xj)) continue;
      const double nearest = std::round(xj);
      if (std::abs(xj - nearest) > opt.int_tol &&
          child.ub[j] - child.lb[j] > opt.int_tol) {
        fractional_frontier[static_cast<std::size_t>(j)] = 1;
        ++frontier_cols;
      }
    }
    if (frontier_cols == 0) return 0;

    ++out.bc_stats.transformed_frontier_source_attempts;

    int rows_added = 0;

    auto append_rows_from_trial =
        [&](const LPModel& trial_lp, int first_row,
            const char* family) -> int {
      if (first_row < 0 || first_row >= trial_lp.A.rows()) return 0;
      Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = trial_lp.A;
      int accepted = 0;
      for (int r = first_row; r < Arow.rows() && rows_added < max_rows; ++r) {
        if (r >= trial_lp.b.size()) break;
        Eigen::SparseVector<double> row(n);
        row.reserve(Arow.innerVector(r).nonZeros());
        double max_fractional_coeff = 0.0;
        int nnz = 0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                 Arow, r);
             it; ++it) {
          const int j = static_cast<int>(it.col());
          if (j < 0 || j >= n) continue;
          const double a = it.value();
          if (!std::isfinite(a) || std::abs(a) <= 1e-12) continue;
          row.insertBack(j) = a;
          ++nnz;
          if (fractional_frontier[static_cast<std::size_t>(j)] != 0) {
            max_fractional_coeff =
                std::max(max_fractional_coeff, std::abs(a));
          }
        }
        row.finalize();
        const double rhs = trial_lp.b[r];
        const double viol = sparse_dot(row, child.x_relax) - rhs;

        if (nnz == 0 || !std::isfinite(rhs) || !std::isfinite(viol) ||
            viol <= std::max(1e-7,
                             opt.dynamic_implied_bound_min_violation)) {
          ++out.bc_stats.transformed_frontier_rejected_rows;
          continue;
        }
        if (max_fractional_coeff <= 1e-10) {
          ++out.bc_stats.transformed_frontier_rejected_rows;
          continue;
        }
        ++out.bc_stats.transformed_frontier_violated_rows;
        ++out.bc_stats.transformed_frontier_fractional_coeff_hits;
        out.bc_stats.transformed_frontier_max_fractional_coeff =
            std::max(out.bc_stats.transformed_frontier_max_fractional_coeff,
                     max_fractional_coeff);
        out.bc_stats.transformed_frontier_max_violation =
            std::max(out.bc_stats.transformed_frontier_max_violation, viol);
        const std::size_t h = sparse_cut_hash(row);
        if (contains_identical_local_cut(child, local_proof_cuts, row, rhs)) {
          ++out.bc_stats.transformed_frontier_rejected_rows;
          continue;
        }
        PoolCut cut;
        cut.coeff = std::move(row);
        cut.rhs = rhs;
        const double norm =
            std::sqrt(std::max(0.0, cut.coeff.squaredNorm()));
        cut.norm = norm;
        cut.best_efficacy = viol / std::max(1.0, norm);
        cut.hash = h;
        if (!valid_pool_cut(cut, &child.x_relax, family)) {
          ++out.bc_stats.transformed_frontier_rejected_rows;
          continue;
        }
        ++out.bc_stats.local_node_cuts_generated;
        local_proof_cuts.push_back(make_local_node_cut(std::move(cut)));
        ++out.bc_stats.transformed_frontier_admitted_rows;
        ++accepted;
        ++rows_added;
      }
      return accepted;
    };

    LPModel trial_lp = base_lp;
    apply_node_bounds(trial_lp.vars, child.lb, child.ub);
    int remaining = max_rows - rows_added;
    const int tableau_budget = std::max(0, remaining);
    if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C-XFRONT-SRC] depth={} maxRows={} remaining={} "
	                 "tableauBudget={} exact={} xSize={} sfOrig={} localCuts={}\n",
                 child.depth, max_rows, remaining, tableau_budget,
                 simplex.exact_optimal ? 1 : 0,
                 static_cast<int>(child.x_relax.size()),
                 simplex.form.n_original,
                 static_cast<int>(local_proof_cuts.size()));
    }
    if (tableau_budget > 0) {
      const int prev_rows = static_cast<int>(trial_lp.A.rows());
      const int generated = add_transformed_tableau_cuts(
          trial_lp, child.x_relax, simplex, opt, tableau_budget, sbasis,
          &root_implied_integer_cols, &implication_graph,
          &variable_bound_table, nullptr, 1e-8, nullptr, std::nullopt,
          &separator_storage_stats);
      if (generated > 0) {
        out.bc_stats.transformed_frontier_tableau_rows +=
            static_cast<std::uint64_t>(generated);
        const int accepted =
            append_rows_from_trial(trial_lp, prev_rows, "node_xtab_frontier");
        (void)accepted;
        remaining = max_rows - rows_added;
      }
    }
    if (remaining > 0) {
      const int prev_rows = static_cast<int>(trial_lp.A.rows());
      const int generated = add_transformed_path_cuts(
          trial_lp, child.x_relax, simplex, opt, remaining,
          &root_implied_integer_cols, &implication_graph,
          &variable_bound_table, nullptr, 1e-8, nullptr, std::nullopt,
          nullptr, &separator_storage_stats);
      if (generated > 0) {
        out.bc_stats.transformed_frontier_path_rows +=
            static_cast<std::uint64_t>(generated);
        const int accepted =
            append_rows_from_trial(trial_lp, prev_rows, "node_xpath_frontier");
        (void)accepted;
      }
    }

    if (opt.verbose && (rows_added > 0 || frontier_cols > 0)) {
      fmt::print(stderr,
                 "[B&C-XFRONT] depth={} frontier={} rows={} "
                 "tabRows={} pathRows={} "
                 "maxCoeff={:.6g} maxViol={:.6g}\n",
                 child.depth, frontier_cols, rows_added,
                 out.bc_stats.transformed_frontier_tableau_rows,
                 out.bc_stats.transformed_frontier_path_rows,
                 out.bc_stats.transformed_frontier_max_fractional_coeff,
                 out.bc_stats.transformed_frontier_max_violation);
    }
    return rows_added;
  };

		  struct StrongBranchProbeResult {
	    bool available{false};
	    bool consumed{false};
	    bool from_ipm{false};
		    int var{-1};
		    bool is_up{false};
		    LPFailureType failure_type{LPFailureType::Unknown};
		    BranchEvidenceState evidence_state{BranchEvidenceState::UnknownFailure};
		    double observed_gain{0.0};
	    SimplexResult simplex;
	    SolveResult ipm_result;
	    Eigen::VectorXd lb;
	    Eigen::VectorXd ub;
	    std::uint64_t local_row_signature{0};
	    std::uint64_t model_epoch{0};
	    std::uint64_t objective_epoch{0};
		    bool deferred{false};
		  };

		  auto detach_probe_transient_basis_state = [](SimplexResult& simplex) {
		    // Probe solves use a temporary StandardFormLP object.  Native
		    // SparseBasis stores a pointer to the matrix it was factorized from,
		    // so only keep the value-state (basis indices, nonbasic side,
		    // reduced costs, x_std/form) when carrying the probe result into the
		    // child evaluator. VendoredHighsBasis owns an independent HiGHS
		    // handle, so it is safe and useful as a propagated-domain warm state.
		    if (simplex.basis.cached_sparse_basis &&
		        simplex.basis.cached_sparse_basis->kind() ==
		            BasisOpsKind::NativeSparse) {
		      simplex.basis.cached_sparse_basis.reset();
		    }
		  };

		  auto same_domain_bounds_for_probe =
		      [&](const Eigen::VectorXd& a_lb, const Eigen::VectorXd& a_ub,
		          const Eigen::VectorXd& b_lb, const Eigen::VectorXd& b_ub) -> bool {
	    if (a_lb.size() != n || a_ub.size() != n ||
	        b_lb.size() != n || b_ub.size() != n) {
	      return false;
	    }
	    const double tol = std::max(1e-12, 10.0 * std::max(opt.lp_tol, opt.int_tol));
	    for (int i = 0; i < n; ++i) {
	      const auto same_bound = [&](double a, double b) {
	        if (!std::isfinite(a) || !std::isfinite(b)) {
	          return std::isfinite(a) == std::isfinite(b);
	        }
	        return std::abs(a - b) <= tol;
	      };
	      if (!same_bound(a_lb[i], b_lb[i]) || !same_bound(a_ub[i], b_ub[i])) {
	        return false;
	      }
	    }
		    return true;
		  };

  // In short-budget basis-free mode the first two IPM solves are the actual
  // root children, not reliability probes. Split their remaining wall budget
  // fairly so the down solve cannot starve the up solve.
  int first_branch_ipm_solve_slots = 0;
  bool ipm_node_deadline_hit = false;

	  // ════════════════════════════════════════════════════════════════════════
	  // Helper: process a single child node (LP solve, pool separation,
	  // cut generation). Used by both sequential and parallel code paths.
	  // ════════════════════════════════════════════════════════════════════════
	  auto process_single_child = [&](Node& child, bool is_up_branch, int branch_var,
	                                   double branch_distance,
	                                   double parent_bound,
	                                   StandardFormLP& node_sf_ref,
	                                   CutPool& cp_ref,
	                                   double inc_obj_snapshot,
	                                   bool has_inc_snapshot,
	                                   StrongBranchProbeResult* probe_lp_state = nullptr) -> std::tuple<bool, std::vector<PCUpdate>,
	                                                                        std::vector<PoolSolution>,
	                                                                        std::vector<PoolCut>,
	                                                                        int, int, int> {
    std::vector<PCUpdate> pc_upd;
    std::vector<PoolSolution> sols;
    std::vector<PoolCut> new_cuts;
    std::vector<PoolCut> local_proof_cuts;
	    std::vector<PoolCut> transformed_frontier_cuts;
	    int local_transformed_frontier_rows_pending = 0;
	    bool rc_fixing_preproof_applied = false;
	    int rc_fixed_preproof = 0;
    std::vector<BranchDomainLiteral> rc_forbidden_literals_preproof;
	    Eigen::VectorXd rc_proof_lb_preproof;
	    Eigen::VectorXd rc_proof_ub_preproof;
	    std::shared_ptr<SimplexResult> child_final_simplex_for_proof;
	    const bool defer_child_lp_proof_until_closure = true;
	    const double child_upper_limit_snapshot =
	        has_inc_snapshot && std::isfinite(inc_obj_snapshot)
	            ? highs_style_incumbent_upper_limit(base_lp, inc_obj_snapshot,
	                                               opt.int_tol)
	            : kInf;
	    const bool has_child_upper_limit =
	        has_inc_snapshot && std::isfinite(child_upper_limit_snapshot);
	    int lp_count = 0, cut_count = 0, pool_sep = 0;
    PCUpdate branch_obs{branch_var, is_up_branch};
    branch_obs.branch_distance = branch_distance;

    if (!bounds_consistent(child.lb, child.ub)) {
      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
    }

    int conflict_tightenings = 0;
    std::vector<BoundChangeInfo> prop_changes;
    prop_changes.reserve(64);
    if (opt.enable_reduced_cost_proof_conflict_minimization) {
      child.domain_reason_bounds.reserve(child.domain_reason_bounds.size() + 32);
    }
    NodeDomainClosureStats child_closure_stats;
    const std::vector<DomainReasonBound> no_incoming_reasons;
    if (!run_node_domain_closure(child, no_incoming_reasons, prop_changes,
                                 &new_cuts, &cut_count, inc_obj_snapshot,
                                 has_inc_snapshot, "child_pre_lp",
                                 &child_closure_stats)) {
      branch_obs.has_inference = true;
      branch_obs.inference_count =
          static_cast<double>(child_closure_stats.tightened +
                              child_closure_stats.conflict_tightened +
	                              child_closure_stats.row_tightened +
	                              child_closure_stats.cutpool_tightened +
	                              child_closure_stats.objective_tightened +
	                              child_closure_stats.redcost_tightened +
	                              child_closure_stats.reasons_added);
      branch_obs.conflict_score =
          static_cast<double>(child_closure_stats.conflicts_learned);
      branch_obs.cutoff_count = 1;
      pc_upd.push_back(branch_obs);
      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
    }
    conflict_tightenings =
        static_cast<int>(std::min<std::uint64_t>(
            std::numeric_limits<int>::max(),
            child_closure_stats.tightened +
                child_closure_stats.conflict_tightened +
	                child_closure_stats.row_tightened +
	                child_closure_stats.cutpool_tightened +
	                child_closure_stats.objective_tightened +
	                child_closure_stats.redcost_tightened));
    if (conflict_tightenings > 0 || child_closure_stats.reasons_added > 0 ||
        child_closure_stats.conflicts_learned > 0) {
      branch_obs.has_inference = true;
      branch_obs.inference_count +=
          static_cast<double>(conflict_tightenings +
                              child_closure_stats.reasons_added);
      branch_obs.conflict_score +=
          static_cast<double>(child_closure_stats.conflicts_learned);
    }

	    // Unified node domain propagation has already replayed conflicts,
	    // clique implications, root-probed implications, and row tightening.
	    auto _t0 = std::chrono::steady_clock::now();
	    branch_obs.has_inference = branch_obs.has_inference ||
	                               conflict_tightenings > 0;
	    auto _t1 = std::chrono::steady_clock::now();

		    const bool probe_matches_direction =
		        probe_lp_state != nullptr && probe_lp_state->available &&
		        !probe_lp_state->consumed && probe_lp_state->var == branch_var &&
		        probe_lp_state->is_up == is_up_branch;
		    const bool probe_success =
		        probe_matches_direction &&
		        probe_lp_state->failure_type == LPFailureType::Success;
		    const bool probe_proof_usable =
		        probe_matches_direction &&
		        ((probe_lp_state->evidence_state ==
		              BranchEvidenceState::ExactOptimal &&
		          !probe_lp_state->from_ipm &&
		          probe_lp_state->simplex.exact_optimal) ||
		         (probe_lp_state->evidence_state ==
		              BranchEvidenceState::ProvenCutoff &&
		          !probe_lp_state->from_ipm));
		    const bool probe_warm_state_usable =
		        probe_success &&
		        (probe_lp_state->from_ipm
		             ? probe_lp_state->ipm_result.x.size() == n
		             : probe_lp_state->simplex.result.x.size() == n);
		    const auto probe_reuse = classify_branch_probe_reuse(
		        probe_lp_state != nullptr && probe_lp_state->available,
		        probe_lp_state != nullptr && probe_lp_state->consumed,
		        probe_lp_state != nullptr ? probe_lp_state->var : -1,
		        probe_lp_state != nullptr && probe_lp_state->is_up, branch_var,
		        is_up_branch,
		        probe_lp_state != nullptr ? probe_lp_state->lb : Eigen::VectorXd{},
		        probe_lp_state != nullptr ? probe_lp_state->ub : Eigen::VectorXd{},
		        child.lb, child.ub,
		        probe_lp_state != nullptr ? probe_lp_state->local_row_signature : 0,
		        ordered_local_cut_signature(child.local_cuts),
		        probe_lp_state != nullptr ? probe_lp_state->model_epoch : 0,
		        current_domain_closure_epoch(),
		        probe_lp_state != nullptr ? probe_lp_state->objective_epoch : 0,
		        current_objective_artifact_epoch(), probe_proof_usable,
		        probe_warm_state_usable,
		        std::max(1e-12, 10.0 * std::max(opt.lp_tol, opt.int_tol)));
		    const bool consume_probe_lp =
		        probe_reuse == BranchProbeReuseKind::Exact;
		    branch_obs.suppress_gain_update = consume_probe_lp;
		    if ((opt.verbose || trace_bc_timeline) && probe_matches_direction &&
		        probe_reuse == BranchProbeReuseKind::None) {
		      fmt::print(stderr,
		                 "[B&C-SB-SKIP] depth={} var={} up={} type={} "
		                 "row_sig={} model_epoch={} objective_epoch={}\n",
		                 child.depth, branch_var, is_up_branch ? 1 : 0,
		                 lp_failure_type_str(probe_lp_state->failure_type),
		                 probe_lp_state->local_row_signature,
		                 probe_lp_state->model_epoch,
		                 probe_lp_state->objective_epoch);
		    }
        if (probe_reuse == BranchProbeReuseKind::Warm &&
            probe_lp_state->from_ipm) {
          // Domain closure may add bounds after a structure-only probe. The
          // old objective is not a valid child bound, but its primal remains
          // a useful starting point once clamped into the closed domain.
          child.x_seed = clamp_to_bounds(probe_lp_state->ipm_result.x,
                                         child.lb, child.ub);
          ++out.bc_stats.strong_branch_cache_warm_hits;
          probe_lp_state->consumed = true;
          if (opt.verbose || trace_bc_timeline) {
            fmt::print(stderr,
                       "[B&C-SB-SEED] depth={} var={} up={} "
                       "closed_domain_warm_start=1\n",
                       child.depth, branch_var, is_up_branch ? 1 : 0);
          }
        }
        if (probe_reuse == BranchProbeReuseKind::Warm &&
            !probe_lp_state->from_ipm) {
          child.x_seed = clamp_to_bounds(probe_lp_state->simplex.result.x,
                                         child.lb, child.ub);
          const auto& live_basis =
              probe_lp_state->simplex.basis.cached_sparse_basis;
          if (live_basis && live_basis->kind() == BasisOpsKind::VendoredHighs) {
            auto warm_basis = std::make_shared<SimplexBasis>(
                probe_lp_state->simplex.basis);
            warm_basis->cached_reduced_costs =
                std::make_shared<const Eigen::VectorXd>(
                    probe_lp_state->simplex.reduced_costs);
            child.basis_hint = std::move(warm_basis);
          }
          ++out.bc_stats.strong_branch_cache_warm_hits;
          probe_lp_state->consumed = true;
        }

	    // Update standard form with child's bounds. When a child already carries
	    // inherited local cuts, build a node-local LP/SF so the relaxation rows
	    // match the local domain state.
	    if (child.local_cuts.empty()) {
	      update_standard_form_bounds(node_sf_ref, base_lp, child.lb, child.ub);
	    } else {
	      node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
	    }
	    auto _t2 = std::chrono::steady_clock::now();

	    SolveResult node_result;
		    bool node_lp_success = false;
		    DualProofRow child_dual_proof;
			    DualProofRow child_objective_event_proof;
			    DualProofRow child_rc_mass_proof;
			    bool child_dual_proof_ok = false;
			    bool child_rc_mass_proof_ok = false;
		    auto compact_child_full_dual_proof =
		        [&](DualProofRow& proof, const Eigen::VectorXd& x_lp) -> int {
		      if (proof.coeff.size() < n || x_lp.size() < n) return 0;
		      Eigen::VectorXd compact =
		          Eigen::VectorXd::Zero(proof.coeff.size());
		      for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff); it;
		           ++it) {
		        compact[static_cast<int>(it.index())] = it.value();
		      }
		      double compact_rhs = proof.rhs;
		      int compact_removed = 0;
		      const double compact_tol =
		          std::max(1e-9, std::max(1e-9, opt.lp_tol));
		      for (int j = 0; j < n; ++j) {
		        if (base_lp.vars[static_cast<std::size_t>(j)].type !=
		            VarType::Continuous) {
		          continue;
		        }
		        const bool fixed =
		            std::isfinite(root.lb[j]) && std::isfinite(root.ub[j]) &&
		            std::abs(root.ub[j] - root.lb[j]) <= compact_tol;
		        if (fixed) continue;
		        const double a = compact[j];
		        if (std::abs(a) <= compact_tol) continue;
		        if (a > 0.0 && std::isfinite(root.lb[j]) &&
		            x_lp[j] - root.lb[j] <=
		                std::max(1e-7, 10.0 * compact_tol)) {
		          compact_rhs -= a * root.lb[j];
		          compact[j] = 0.0;
		          ++compact_removed;
		        } else if (a < 0.0 && std::isfinite(root.ub[j]) &&
		                   root.ub[j] - x_lp[j] <=
		                       std::max(1e-7, 10.0 * compact_tol)) {
		          compact_rhs -= a * root.ub[j];
		          compact[j] = 0.0;
		          ++compact_removed;
		        }
		      }
		      if (compact_removed <= 0) return 0;
		      Eigen::SparseVector<double> compact_sparse(n);
		      compact_sparse.reserve(proof.coeff.nonZeros());
		      for (int j = 0; j < n; ++j) {
		        if (std::abs(compact[j]) >
		            std::max(1e-12, compact_tol * 1e-2)) {
		          compact_sparse.insertBack(j) = compact[j];
		        }
		      }
		      compact_sparse.finalize();
		      proof.coeff = std::move(compact_sparse);
		      proof.nnz = static_cast<int>(proof.coeff.nonZeros());
		      proof.rhs = compact_rhs;
		      proof.removed_bound_coefficients += compact_removed;
		      return compact_removed;
		    };

    auto collect_integral_lp_candidate = [&](const char* phase) -> bool {
      if (child.x_relax.size() != n) return false;
      const double mip_integrality_tol =
          requested_vendored_highs_lp ? std::max(1e-9, opt.lp_tol)
                                      : opt.int_tol;
      std::vector<int> frac_check;
	      if (node_fractional_branchable_indices(
	              child, mip_integrality_tol, frac_check)) {
        return false;
      }
      Eigen::VectorXd x_int = child.x_relax;
	      round_declared_branchables(x_int, child.lb, child.ub);
      const double feas_tol =
          requested_vendored_highs_lp ? std::max(kNodeFeasibilityTol, opt.lp_tol)
                                      : kNodeFeasibilityTol;
      if (!satisfies_with_bounds(base_lp, x_int, child.lb, child.ub,
                                 feas_tol)) {
        return false;
      }
      const double obj = objective_value(base_lp.c, x_int, base_lp.sense);
      bool duplicate = false;
      for (const auto& sol : sols) {
        if (std::abs(sol.obj - obj) <=
            std::max(1e-9, 1e-12 * std::max(1.0, std::abs(obj)))) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        sols.push_back({std::move(x_int), obj});
      }
      branch_obs.cutoff_count = 1;
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C LP-INTEGRAL] phase={} depth={} obj={:.10g} "
                   "bound={:.10g} inc={:.10g} up={}\n",
                   phase != nullptr ? phase : "lp", child.depth, obj,
                   child.bound, has_inc_snapshot ? inc_obj_snapshot : kInf,
                   is_up_branch ? 1 : 0);
      }
      return true;
    };

	    if (use_ipm_nodes && ipm_node_solver) {
      // IPM-LP path: solve node LP directly from LPModel + bounds and pass the
      // parent relaxation as an initial point when available.
      if (consume_probe_lp && probe_lp_state->from_ipm) {
        node_result = std::move(probe_lp_state->ipm_result);
        probe_lp_state->consumed = true;
        ++out.bc_stats.strong_branch_cache_exact_hits;
        ++out.bc_stats.strong_branch_duplicate_lp_avoided;
        if (opt.verbose || trace_bc_timeline) {
          fmt::print(stderr,
                     "[B&C-SB-CONSUME] depth={} var={} up={} type={} "
                     "bound={:.10g} solver=ipm\n",
                     child.depth, branch_var, is_up_branch ? 1 : 0,
                     lp_failure_type_str(probe_lp_state->failure_type),
                     node_result.stats.objective);
        }
      } else {
        const double solve_remaining =
            bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
        const double solve_limit = first_branch_ipm_solve_slots > 0
            ? solve_remaining / first_branch_ipm_solve_slots
            : solve_remaining;
        if (first_branch_ipm_solve_slots > 0) {
          --first_branch_ipm_solve_slots;
        }
        if (child.local_cuts.empty()) {
          ipm_node_solver->set_solve_time_limit(
              std::max(0.001, solve_limit));
          node_result = ipm_node_solver->solve_structure_aware_node_lp(
              child.lb, child.ub, child.x_seed);
        } else {
          ipm_cut_solver.set_solve_time_limit(
              std::max(0.001, solve_limit));
          LPModel active_node_lp = build_node_lp_with_local_cuts(child);
          node_result = ipm_cut_solver.solve_lp(active_node_lp, child.x_seed);
        }
        ++lp_count;
      }
      node_lp_success = node_result.stats.success &&
                        node_result.x.size() == n;
      if (!node_lp_success &&
          node_result.stats.status.starts_with("Time limit")) {
        ipm_node_deadline_hit = true;
      }
      if (node_lp_success) {
		        child.x_relax = clamp_to_bounds(node_result.x, child.lb, child.ub);
		        child.x_seed = child.x_relax;
		        child.bound = std::max(parent_bound, node_result.stats.objective);
		        child.lp_refresh_needed = false;
		        child.domain_learning_epoch = current_domain_learning_epoch();
		        child.objective_artifact_epoch = current_objective_artifact_epoch();
		        child.domain_closure_epoch = current_domain_closure_epoch();
		        child.ipm_iterations = node_result.stats.iterations;  // ← Capture IPM iterations
        child.has_live_factor_telemetry = false;
        child.live_factor_telemetry = {};
        // Track IPM iteration statistics
        if (use_ipm_nodes) {
          ++out.bc_stats.ipm_nodes_solved;
          out.bc_stats.ipm_total_iterations += child.ipm_iterations;
          out.bc_stats.ipm_iteration_min = std::min(out.bc_stats.ipm_iteration_min, child.ipm_iterations);
          out.bc_stats.ipm_iteration_max = std::max(out.bc_stats.ipm_iteration_max, child.ipm_iterations);
        }
        // No simplex basis available in IPM mode — child inherits parent's
        // basis_hint (already set from branching) for potential simplex fallback.
      }
	    } else {
	      // Simplex path: use StandardFormLP + warm-start basis.
	        bool deferred = false;
	        const int node_id = seq_node_id++;
	        bool node_lp_from_probe_cache = false;
	        SimplexResult simplex_res;
	        LPFailureType failure_type = LPFailureType::Unknown;
	        if (consume_probe_lp) {
	          simplex_res = std::move(probe_lp_state->simplex);
	          failure_type = probe_lp_state->failure_type;
	          deferred = probe_lp_state->deferred;
	          probe_lp_state->consumed = true;
	          node_lp_from_probe_cache = true;
	          ++out.bc_stats.strong_branch_cache_exact_hits;
	          ++out.bc_stats.strong_branch_duplicate_lp_avoided;
	          if (opt.verbose || trace_bc_timeline) {
	            fmt::print(stderr,
	                       "[B&C-SB-CONSUME] depth={} var={} up={} type={} "
	                       "bound={:.10g}\n",
	                       child.depth, branch_var, is_up_branch ? 1 : 0,
	                       lp_failure_type_str(failure_type),
	                       simplex_res.result.stats.objective);
	          }
	        } else {
	          const double node_lp_remaining =
	              bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
	          if (node_lp_remaining <= 0.001) {
	            simplex_node_deadline_hit = true;
	          }
	          seq_dispatcher.set_time_limit(
	              std::max(0.001, node_lp_remaining),
	              &simplex_node_deadline_hit);
	          simplex_res = seq_dispatcher.solve(
	            node_sf_ref, child.basis_hint.get(), SolveContext::NodeLP,
	            node_id, child.depth, n, deferred);
	          ++lp_count;
	          failure_type = classify_lp_result(simplex_res, n);
	          if (lp_wall_time_limit_reached(simplex_res)) {
	            simplex_node_deadline_hit = true;
	          }
	        }

	      if (failure_type != LPFailureType::Success) {
	        if (failure_type == LPFailureType::ObjectiveCutoff) {
	          child.bound = std::max(parent_bound, simplex_res.result.stats.objective);
          branch_obs.cutoff_count = 1;
          pc_upd.push_back(branch_obs);
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
        if (simplex_node_deadline_hit) {
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
        const bool has_farkas = simplex_res.result.stats.has_farkas_certificate;

        // IPM fallback: when simplex warm-start fails (including all fallback
        // retries), try IPM which doesn't need a simplex basis. This also
        // cross-checks Farkas-based infeasibility before pruning a subtree.
        if (ipm_fallback_solver && failure_type != LPFailureType::Unbounded) {
          SolveResult ipm_res;
          if (child.local_cuts.empty()) {
            apply_tree_ipm_deadline(*ipm_fallback_solver);
            ipm_res = ipm_fallback_solver->solve_node_lp(
                base_lp, child.lb, child.ub, child.x_seed);
          } else {
            LPModel active_node_lp = build_node_lp_with_local_cuts(child);
            apply_tree_ipm_deadline(ipm_cut_solver);
            ipm_res = ipm_cut_solver.solve_lp(active_node_lp, child.x_seed);
          }
          ++lp_count;
          if (!ipm_res.stats.success &&
              ipm_res.stats.status.starts_with("Time limit")) {
            ipm_node_deadline_hit = true;
          }
          if (ipm_res.stats.success && ipm_res.x.size() == n) {
            node_lp_success = true;
	            node_result = std::move(ipm_res);
		            child.x_relax = clamp_to_bounds(node_result.x, child.lb, child.ub);
		            child.x_seed = child.x_relax;
		            child.bound = std::max(parent_bound, node_result.stats.objective);
		            child.lp_refresh_needed = false;
		            child.domain_learning_epoch = current_domain_learning_epoch();
		            child.objective_artifact_epoch = current_objective_artifact_epoch();
		            child.domain_closure_epoch = current_domain_closure_epoch();
		            child.has_live_factor_telemetry = false;
            child.live_factor_telemetry = {};
            // No simplex basis — child keeps parent's basis_hint for future
            // simplex warm-start attempts at grandchild nodes.
	          }
	        }
	        if (!node_lp_success) {
	          if (failure_type == LPFailureType::Infeasible) {
	            seq_fallback_logger.log(node_id, child.depth,
	                                    LPFailureType::Infeasible, 5, "pruned");
	            branch_obs.cutoff_count = 1;
	            if (has_farkas) {
	              std::vector<BranchDomainLiteral> learned_conflict =
	                  child.branch_reasons;
	              branch_obs.conflict_score = 1.0;
	              if (!learned_conflict.empty()) {
		                const bool pool_added = add_global_conflict(
		                    learned_conflict, "node_lp_farkas");
		                const bool impl_added =
		                    add_binary_implications_from_conflict_clause(
		                        base_lp.vars, learned_conflict, implication_graph);
		                if (pool_added || impl_added) {
		                  note_global_domain_learning(1 + (impl_added ? 1 : 0));
		                }
	                if (!strict_highs_lp_contract) {
	                  PoolCut conflict_cut;
	                  if (try_build_binary_conflict_cut(base_lp, learned_conflict,
	                                                    conflict_cut)) {
	                    new_cuts.push_back(std::move(conflict_cut));
	                    ++cut_count;
	                  }
	                }
	              }
	            }
	            pc_upd.push_back(branch_obs);
          }
          if (deferred) {
            child.lp_refresh_needed = true;
            child.fractional_branchables_valid = false;
            const bool child_queue_proof_mode =
                has_inc_snapshot && std::isfinite(inc_obj_snapshot) &&
                std::isfinite(root.bound) &&
                (inc_obj_snapshot - root.bound) /
                        std::max(1.0, std::abs(inc_obj_snapshot)) <=
                    opt.gap_tol;
            node_queue.push(child, child_queue_proof_mode);
          }
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
	      } else {
	        node_lp_success = true;
	        child.x_relax = clamp_to_bounds(simplex_res.result.x, child.lb, child.ub);
	        child.x_seed = child.x_relax;
		        if (!defer_child_lp_proof_until_closure &&
		            has_child_upper_limit &&
	            (opt.enable_reduced_cost_proof_cut_resolve ||
	             opt.enable_reduced_cost_proof_conflict_minimization)) {
          LPModel proof_lp_scratch;
          const LPModel& proof_lp = active_lp_for_proof(child, proof_lp_scratch);
          const bool proof_is_local = !child.local_cuts.empty();
          const std::vector<BranchDomainLiteral> proof_scope =
              proof_is_local ? node_local_validity_scope(child)
                             : std::vector<BranchDomainLiteral>{};
	          child_dual_proof_ok = build_full_dual_proof_row(
	              proof_lp, simplex_res, root.lb, root.ub, child.x_relax,
	              simplex_res.result.stats.objective, child_upper_limit_snapshot,
	              child_dual_proof, std::max(1e-9, opt.lp_tol), &node_sf_ref);
	          if (child_dual_proof_ok) {
	            child_objective_event_proof = child_dual_proof;
	            cached_queue_objective_event_proof = child_objective_event_proof;
	            have_cached_queue_objective_event_proof = true;
	            // Do not expose a child objective-event proof as global direct
	            // objective propagation.  Cross-frontier use is allowed only
	            // after the generating node's local-trail resolver publishes a
	            // first-class certificate.
	            cached_queue_objective_event_proof_is_global = false;
	            cached_queue_objective_event_proof_upper =
	                child_upper_limit_snapshot;
	            cached_queue_objective_event_proof_scope = child.branch_reasons;
	            cached_queue_dual_proof = child_objective_event_proof;
	            cached_queue_dual_proof_upper = child_upper_limit_snapshot;
	            have_cached_queue_dual_proof = true;
	            cached_queue_dual_proof_is_global = !proof_is_local;
	            cached_queue_dual_proof_scope = proof_scope;
	            if (proof_tail_queue_mode_active()) {
	              // This only schedules replay of certificates already
	              // published by local-trail resolution; the raw row is not a
	              // queue propagation object.
	              consume_dual_proof_for_queue(child_objective_event_proof,
                                           child_upper_limit_snapshot,
	                                           "node_full_proof",
	                                           proof_is_local ? &proof_scope
	                                                          : nullptr);
	            }

		            // Keep the expanded full proof for objective-event
		            // explanations, but compact the generic proof row before it is
		            // used by the node-local resolver.
		            compact_child_full_dual_proof(child_dual_proof,
		                                          child.x_relax);
	          }
	          child_rc_mass_proof_ok = build_reduced_cost_mass_dual_proof_row(
	              proof_lp, simplex_res, root.lb, root.ub, child.x_relax,
	              simplex_res.result.stats.objective, child_upper_limit_snapshot,
	              child_rc_mass_proof, std::max(1e-9, opt.lp_tol),
	              &node_sf_ref, &root_implied_integer_cols,
	              &objective_propagation.implied_event_target_cols);
	          if (child_rc_mass_proof_ok) {
	            cached_queue_dual_proof = child_rc_mass_proof;
	            cached_queue_dual_proof_upper = child_upper_limit_snapshot;
	            have_cached_queue_dual_proof = true;
	            cached_queue_dual_proof_is_global = !proof_is_local;
	            cached_queue_dual_proof_scope = proof_scope;
	            if (proof_tail_queue_mode_active()) {
	              consume_dual_proof_for_queue(child_rc_mass_proof,
                                           child_upper_limit_snapshot,
	                                           "node_rc_mass_proof",
	                                           proof_is_local ? &proof_scope
	                                                          : nullptr);
	            }
	          }
	        }
	        if (has_child_upper_limit &&
            allow_native_proof_cut_rows &&
            rc_proof_cut_resolve_lps_left > 0 &&
            opt.dynamic_implied_bound_probing_max_rows > 0) {
          const int row_budget =
              std::max(0, opt.dynamic_implied_bound_probing_max_rows);
          const SimplexResult* source_simplex = &simplex_res;
          SimplexResult source_simplex_with_form;
          if ((simplex_res.form.n_original <= 0 ||
               simplex_res.form.A.cols() == 0) &&
              node_sf_ref.n_original == n) {
            source_simplex_with_form = simplex_res;
            source_simplex_with_form.form = node_sf_ref;
            source_simplex = &source_simplex_with_form;
          }
          local_transformed_frontier_rows_pending +=
              append_transformed_fractional_frontier_cuts(
                  child, *source_simplex,
                  source_simplex->basis.cached_sparse_basis,
                  transformed_frontier_cuts, row_budget);
	        }
        child_final_simplex_for_proof =
            std::make_shared<SimplexResult>(simplex_res);
        node_result = simplex_res.result;
        const auto node_basis_telemetry =
            child_final_simplex_for_proof->basis.cached_sparse_basis
                ? get_sparse_basis_factor_telemetry(
                      child_final_simplex_for_proof->basis.cached_sparse_basis)
                : SparseFactorTelemetry{};
        const bool have_node_basis_telemetry =
            static_cast<bool>(
                child_final_simplex_for_proof->basis.cached_sparse_basis);
        child.basis_hint =
            persist_bc_node_basis_from_simplex(simplex_res, child.basis_hint);
	        child.bound = std::max(parent_bound, node_result.stats.objective);
	        child.lp_refresh_needed = false;
	        child.domain_learning_epoch = current_domain_learning_epoch();
	        child.objective_artifact_epoch = current_objective_artifact_epoch();
	        child.domain_closure_epoch = current_domain_closure_epoch();
	        if (!node_lp_from_probe_cache) {
	          branch_obs.has_gain = true;
	          branch_obs.gain =
	              std::max(branch_obs.gain,
	                       std::max(0.0, child.bound - parent_bound));
	        }
	        if (have_node_basis_telemetry) {
          child.has_live_factor_telemetry = true;
          child.live_factor_telemetry = node_basis_telemetry;
        } else {
          child.has_live_factor_telemetry = false;
          child.live_factor_telemetry = {};
        }
      }
    }
    auto _t3 = std::chrono::steady_clock::now();

	    if (!node_lp_success) {
	      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	    }

    if (use_ipm_nodes && first_branch_ipm_solve_slots > 0) {
      // Establish the sibling frontier before spending time on optional
      // reduced-cost proofs, pool separation, cuts, or repair re-solves.
      branch_obs.has_gain = true;
      branch_obs.gain =
          std::max(0.0, child.bound - parent_bound);
      if (collect_integral_lp_candidate("first_branch_primary")) {
        pc_upd.push_back(branch_obs);
        return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
      }
      pc_upd.push_back(branch_obs);
      return {true, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
    }

	    if (opt.enable_reduced_cost_fixing &&
	        !use_ipm_nodes && has_child_upper_limit && child.basis_hint &&
        child.basis_hint->cached_reduced_costs &&
        child.basis_hint->cached_reduced_costs->size() >= n) {
      if (opt.enable_reduced_cost_conflict_learning) {
        rc_forbidden_literals_preproof.reserve(
            static_cast<std::size_t>(
                std::max(0, opt.reduced_cost_conflict_max_per_node)));
        if (opt.enable_reduced_cost_proof_conflict_minimization) {
          rc_proof_lb_preproof = child.lb;
          rc_proof_ub_preproof = child.ub;
        }
      }
      int rc_fixed_point_passes = 0;
      for (;;) {
        const std::size_t forbidden_begin =
            rc_forbidden_literals_preproof.size();
        std::vector<DomainReasonBound> rc_fixing_reasons;
			        const int rc_fixed_this_pass = reduced_cost_fixing(
			            base_lp.vars, child.x_relax, *child.basis_hint->cached_reduced_costs,
			            child.basis_hint->basis_indices(), child.bound,
			            child_upper_limit_snapshot,
			            opt.int_tol, child.lb, child.ub,
	            opt.enable_reduced_cost_conflict_learning
		                ? &rc_forbidden_literals_preproof
		                : nullptr,
		            child.basis_hint->cached_col_scale.get(),
            &rc_fixing_reasons, child.branch_reasons.read_ptr(), child.depth,
            static_cast<int>(child.branch_reasons.size() +
                             child.domain_reason_bounds.size()));
	        rc_fixing_preproof_applied = true;
	        if (rc_fixed_this_pass <= 0) break;
        rc_fixed_preproof += rc_fixed_this_pass;
        ++rc_fixed_point_passes;

	        out.bc_stats.rc_fixings +=
	            static_cast<std::uint64_t>(rc_fixed_this_pass);
	        out.bc_stats.rc_forbidden_literals +=
	            static_cast<std::uint64_t>(
	                rc_forbidden_literals_preproof.size() - forbidden_begin);
	        branch_obs.has_inference = true;
	        branch_obs.inference_count +=
	            static_cast<double>(rc_fixed_this_pass);
        for (auto& rb : rc_fixing_reasons) {
          append_domain_reason_bound(child, std::move(rb), n, &root.lb,
                                     &root.ub);
        }
	        rebuild_local_domain_trail(child, n, &root.lb, &root.ub);
        child.lp_refresh_needed = true;

	        node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
	        std::vector<BoundChangeInfo> rc_closure_changes;
        NodeDomainClosureStats rc_closure_stats;
        const std::vector<DomainReasonBound> no_rc_reasons;
        if (!run_node_domain_closure(
                child, no_rc_reasons, rc_closure_changes, &new_cuts,
                &cut_count, inc_obj_snapshot, has_inc_snapshot,
                "child_post_lp_rc_closure", &rc_closure_stats)) {
          branch_obs.cutoff_count = 1;
          pc_upd.push_back(branch_obs);
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
        node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;

        SimplexOptions rc_fp_resolve_opt = simplex_opt;
        rc_fp_resolve_opt.allow_cold_start = true;
        auto rc_fp_res =
            solve_tree_lp(node_sf_ref, rc_fp_resolve_opt,
                          child.basis_hint.get());
        ++lp_count;
        ++out.bc_stats.rc_fixing_resolve_lps;
        const auto rc_fp_type = classify_lp_result(rc_fp_res, n);
        if (rc_fp_type == LPFailureType::Infeasible ||
            rc_fp_type == LPFailureType::ObjectiveCutoff) {
          if (rc_fp_type == LPFailureType::ObjectiveCutoff) {
            child.bound =
                std::max(child.bound, rc_fp_res.result.stats.objective);
          }
          branch_obs.cutoff_count = 1;
          pc_upd.push_back(branch_obs);
          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
        }
        if (rc_fp_type != LPFailureType::Success ||
            rc_fp_res.result.x.size() != n ||
            !std::isfinite(rc_fp_res.result.stats.objective)) {
          break;
        }
        const double old_bound = child.bound;
        child.x_relax =
            clamp_to_bounds(rc_fp_res.result.x, child.lb, child.ub);
        child.x_seed = child.x_relax;
        child.bound = std::max(child.bound, rc_fp_res.result.stats.objective);
        node_result = rc_fp_res.result;
        child_final_simplex_for_proof =
            std::make_shared<SimplexResult>(rc_fp_res);
        child.basis_hint =
            persist_bc_node_basis_from_simplex(rc_fp_res, child.basis_hint);
        if (child.bound > old_bound + 1e-7) {
          ++out.bc_stats.rc_fixing_resolve_bound_improvements;
          branch_obs.has_gain = true;
          branch_obs.gain =
              std::max(branch_obs.gain,
                       std::max(0.0, child.bound - parent_bound));
        }
        child.lp_refresh_needed = false;
	        child.domain_learning_epoch = current_domain_learning_epoch();
	        child.objective_artifact_epoch = current_objective_artifact_epoch();
	        child.domain_closure_epoch = current_domain_closure_epoch();
	      }
      if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) &&
          rc_fixed_preproof > 0) {
        fmt::print(stderr,
                   "[B&C-RC-FIXPOINT] phase=child_post_lp depth={} pass={} "
                   "fix={} bound={:.10g} forbidden={}\n",
                   child.depth, rc_fixed_point_passes, rc_fixed_preproof,
                   child.bound, rc_forbidden_literals_preproof.size());
      }
	    }

	    if (collect_integral_lp_candidate("node_lp")) {
	      pc_upd.push_back(branch_obs);
	      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	    }

	    if (defer_child_lp_proof_until_closure &&
		        child_final_simplex_for_proof && !use_ipm_nodes &&
		        has_child_upper_limit &&
	        (opt.enable_reduced_cost_proof_cut_resolve ||
	         opt.enable_reduced_cost_proof_conflict_minimization)) {
	      SimplexResult& proof_simplex = *child_final_simplex_for_proof;
	      LPModel proof_lp_scratch;
	      const LPModel& proof_lp = active_lp_for_proof(child, proof_lp_scratch);
	      const bool proof_is_local = !child.local_cuts.empty();
	      const std::vector<BranchDomainLiteral> proof_scope =
	          proof_is_local ? node_local_validity_scope(child)
	                         : std::vector<BranchDomainLiteral>{};
	      child_dual_proof_ok = build_full_dual_proof_row(
	          proof_lp, proof_simplex, root.lb, root.ub, child.x_relax,
	          proof_simplex.result.stats.objective, child_upper_limit_snapshot,
	          child_dual_proof, std::max(1e-9, opt.lp_tol), &node_sf_ref);
      if (child_dual_proof_ok) {
        child_objective_event_proof = child_dual_proof;
        cached_queue_objective_event_proof = child_objective_event_proof;
        have_cached_queue_objective_event_proof = true;
        cached_queue_objective_event_proof_is_global = false;
        cached_queue_objective_event_proof_upper = child_upper_limit_snapshot;
        cached_queue_objective_event_proof_scope = child.branch_reasons;
	        cached_queue_dual_proof = child_objective_event_proof;
	        cached_queue_dual_proof_upper = child_upper_limit_snapshot;
	        have_cached_queue_dual_proof = true;
	        cached_queue_dual_proof_is_global = !proof_is_local;
	        cached_queue_dual_proof_scope = proof_scope;
	        if (proof_tail_queue_mode_active()) {
	          consume_dual_proof_for_queue(child_objective_event_proof,
	                                           child_upper_limit_snapshot,
	                                       "node_full_proof",
	                                       proof_is_local ? &proof_scope
	                                                      : nullptr);
	        }
	        compact_child_full_dual_proof(child_dual_proof, child.x_relax);
	      }
	      child_rc_mass_proof_ok = build_reduced_cost_mass_dual_proof_row(
	          proof_lp, proof_simplex, root.lb, root.ub, child.x_relax,
	          proof_simplex.result.stats.objective, child_upper_limit_snapshot,
	          child_rc_mass_proof, std::max(1e-9, opt.lp_tol),
          &node_sf_ref, &root_implied_integer_cols,
          &objective_propagation.implied_event_target_cols);
      if (child_rc_mass_proof_ok) {
	        cached_queue_dual_proof = child_rc_mass_proof;
	        cached_queue_dual_proof_upper = child_upper_limit_snapshot;
	        have_cached_queue_dual_proof = true;
	        cached_queue_dual_proof_is_global = !proof_is_local;
	        cached_queue_dual_proof_scope = proof_scope;
	        if (proof_tail_queue_mode_active()) {
	          consume_dual_proof_for_queue(child_rc_mass_proof,
	                                           child_upper_limit_snapshot,
	                                       "node_rc_mass_proof",
	                                       proof_is_local ? &proof_scope
	                                                      : nullptr);
	        }
      }
    }

		    // HiGHS separates the real incumbent cutoff (`upper_limit`) from the
		    // gap proof frontier (`optimality_limit`).  Once the node LP lower
		    // bound reaches the latter, the node is suboptimal for the requested
		    // MIP gap and must not spend more LPs on proof cuts, RC fixing, or
		    // pool-cut re-solves.  A dual conflict is still published against
		    // the incumbent cutoff when the current proof row supports it.
		    const DualProofRow* target_dual_proof =
		        child_rc_mass_proof_ok ? &child_rc_mass_proof
		                               : (child_dual_proof_ok ? &child_dual_proof : nullptr);
		    const DualProofRow* fallback_dual_proof =
		        (child_dual_proof_ok && target_dual_proof != &child_dual_proof)
		            ? &child_dual_proof
		            : nullptr;
		    const DualProofRow* priority_dual_proof =
		        child_rc_mass_proof_ok ? &child_rc_mass_proof : target_dual_proof;
			    auto publish_child_bound_exceeding_conflict =
			        [&](const char* source) -> int {
			      if (!has_inc_snapshot || !std::isfinite(inc_obj_snapshot) ||
			          !opt.enable_reduced_cost_proof_conflict_minimization) {
			        return 0;
			      }
			      const double gap_limit = gap_optimality_limit();
			      const bool use_gap_cutoff =
			          !opt.require_tree_exhaustion_certificate &&
			          std::isfinite(gap_limit);
			      std::array<const DualProofRow*, 4> proofs{
			          child_dual_proof_ok ? &child_objective_event_proof : nullptr,
			          child_rc_mass_proof_ok ? &child_rc_mass_proof : nullptr,
			          target_dual_proof,
			          fallback_dual_proof};
		      int published = 0;
		      std::unordered_set<const DualProofRow*> seen_proofs;
		      for (const DualProofRow* proof : proofs) {
		        if (proof == nullptr || !seen_proofs.insert(proof).second ||
		            !proof->valid) {
		          continue;
		        }
			        const double node_min_activity =
			            dual_proof_min_activity_for_node(*proof, child);
			        if (!std::isfinite(node_min_activity)) continue;
			        const double cutoff_rhs =
			            use_gap_cutoff ? proof->rhs + gap_limit - inc_obj_snapshot
			                           : proof->rhs;
			        published += publish_dual_proof_prune_conflict(
			            *proof, child, node_min_activity, cutoff_rhs,
			            /*proof_cutoff=*/true, /*incumbent_cutoff=*/true,
			            source != nullptr ? source : "node_bound_exceeding");
			        if (published > 0) break;
			      }
			      if (published > 0) {
			        branch_obs.conflict_score += static_cast<double>(published);
			        branch_obs.has_inference = true;
			        const int queue_pruned = prune_queue_after_domain_learning(
			            /*force_frontier_drain=*/true,
		            source != nullptr ? source : "node_bound_exceeding");
		        if (queue_pruned > 0) {
		          out.bc_stats.rc_conflict_queue_prunes +=
		              static_cast<std::uint64_t>(queue_pruned);
		        }
		        if (opt.verbose || trace_bc_timeline) {
		          fmt::print(stderr,
		                     "[B&C-BOUND-CONF] source={} depth={} "
		                     "published={} queue_pruned={} bound={:.10g} "
		                     "inc={:.10g}\n",
		                     source != nullptr ? source : "node_bound_exceeding",
		                     child.depth, published, queue_pruned, child.bound,
		                     inc_obj_snapshot);
		        }
			      }
			      return published;
			    };
				    auto consume_child_bound_exceeding_proof =
				        [&](const char* source) -> int {
				      if (!has_inc_snapshot || !std::isfinite(inc_obj_snapshot) ||
				          node_queue.empty()) {
				        return 0;
				      }
				      const double gap_limit = gap_optimality_limit();
				      const bool use_gap_cutoff =
				          !opt.require_tree_exhaustion_certificate &&
				          std::isfinite(gap_limit);
				      std::array<const DualProofRow*, 4> proofs{
				          child_dual_proof_ok ? &child_objective_event_proof : nullptr,
				          priority_dual_proof, target_dual_proof, fallback_dual_proof};
				      int consumed = 0;
				      std::unordered_set<const DualProofRow*> seen_proofs;
				      for (const DualProofRow* proof : proofs) {
				        if (proof == nullptr || !seen_proofs.insert(proof).second ||
				            !proof->valid) {
				          continue;
				        }
				        DualProofRow frontier_proof = *proof;
				        double proof_upper = child_upper_limit_snapshot;
				        if (!std::isfinite(proof_upper)) {
				          proof_upper = inc_obj_snapshot;
				        }
					        const std::vector<BranchDomainLiteral>* required_scope = nullptr;
				        if (use_gap_cutoff) {
				          frontier_proof.rhs += gap_limit - proof_upper;
				          frontier_proof.actual_lp_gap =
				              frontier_proof.rhs - frontier_proof.lp_activity;
				          frontier_proof.expected_lp_gap += gap_limit - proof_upper;
				          proof_upper = gap_limit;
				          required_scope = nullptr;
				        }
				        forced_certificate_queue_passes =
				            std::max(forced_certificate_queue_passes, 1);
				        consumed += consume_dual_proof_for_queue(
				            frontier_proof, proof_upper,
				            source != nullptr ? source : "node_bound_exceeding",
				            required_scope);
				        if (consumed > 0) break;
				      }
				      if (consumed > 0) {
			        branch_obs.conflict_score += static_cast<double>(consumed);
			        branch_obs.has_inference = true;
			      }
			      return consumed;
			    };
			    auto prune_child_by_gap_optimality_limit =
			        [&](const char* source) -> bool {
		      if (opt.require_tree_exhaustion_certificate ||
		          !std::isfinite(child.bound)) {
		        return false;
		      }
		      const double gap_limit = gap_optimality_limit();
		      if (!std::isfinite(gap_limit)) return false;
		      const double gap_limit_tol = gap_optimality_prune_tol(gap_limit);
		      if (child.bound < gap_limit - gap_limit_tol) return false;
			      publish_child_bound_exceeding_conflict(
			          source != nullptr ? source : "node_optimality_limit");
			      consume_child_bound_exceeding_proof(
			          source != nullptr ? source : "node_optimality_limit");
			      prune_queue_by_gap_optimality_limit(
			          source != nullptr ? source : "node_optimality_limit");
			      refresh_queue_lower_bound();
			      gap_limit_lower_bound_certificate =
			          std::max(gap_limit_lower_bound_certificate, gap_limit);
		      ++out.bc_stats.gap_suboptimal_queue_prunes;
		      branch_obs.cutoff_count = 1;
		      if (opt.verbose || trace_bc_timeline) {
		        fmt::print(stderr,
		                   "[B&C-GAP-NODE] source={} depth={} bound={:.10g} "
		                   "opt_limit={:.10g} inc={:.10g}\n",
		                   source != nullptr ? source : "node_optimality_limit",
		                   child.depth, child.bound, gap_limit,
		                   has_inc_snapshot ? inc_obj_snapshot : kInf);
		      }
		      print_bc_timeline_state("gap_node_prune",
		                              source != nullptr ? source
		                                                : "node_optimality_limit",
		                              node_queue.lower_bound(), node_queue.size(), 1);
		      return true;
		    };
		    if (prune_child_by_gap_optimality_limit("node_optimality_limit")) {
		      pc_upd.push_back(branch_obs);
		      return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		    }

		    // The proof source used for target/frontier construction must carry
		    // objective-effective mass.  The full row-dual proof is still a valid
		    // certificate, but it often spreads activity over rows that do not
		    // move the current LP point.  Prefer the reduced-cost-mass proof row
		    // for HiGHS-style reconvergence target selection, and keep the full
		    // proof as a fallback certificate source.
			    std::vector<DomainReasonBound> rc_direct_domain_targets;
	    if (target_dual_proof != nullptr && rc_fixed_preproof > 0 &&
	        !rc_forbidden_literals_preproof.empty()) {
	      rc_direct_domain_targets.reserve(rc_forbidden_literals_preproof.size());
	      const int base_trail =
	          static_cast<int>(child.branch_reasons.size() +
	                           child.domain_reason_bounds.size());
	      for (std::size_t k = 0; k < rc_forbidden_literals_preproof.size(); ++k) {
	        const auto& forbidden = rc_forbidden_literals_preproof[k];
	        const int j = forbidden.var_idx;
	        if (j < 0 || j >= n) continue;
	        BranchDomainLiteral target_bound;
	        if (forbidden.is_lb) {
	          if (!std::isfinite(child.ub[j])) continue;
	          target_bound = BranchDomainLiteral{j, child.ub[j], false};
	        } else {
	          if (!std::isfinite(child.lb[j])) continue;
	          target_bound = BranchDomainLiteral{j, child.lb[j], true};
	        }
	        DomainReasonBound rb;
	        rb.bound = target_bound;
	        rb.trail_pos = base_trail + static_cast<int>(k);
	        rb.depth = child.depth;
	        rb.source_conflict_literal = forbidden;
	        rb.has_source_conflict_literal = true;
	        rc_direct_domain_targets.push_back(std::move(rb));
	      }
	    }
		    std::vector<DomainReasonBound> objective_event_domain_targets;
		    std::vector<DomainReasonBound> direct_domain_targets;
	    if (!rc_direct_domain_targets.empty() ||
	        !objective_event_domain_targets.empty()) {
	      direct_domain_targets.reserve(rc_direct_domain_targets.size() +
	                                    objective_event_domain_targets.size());
	      direct_domain_targets.insert(direct_domain_targets.end(),
	                                   rc_direct_domain_targets.begin(),
	                                   rc_direct_domain_targets.end());
	      direct_domain_targets.insert(direct_domain_targets.end(),
	                                   objective_event_domain_targets.begin(),
	                                   objective_event_domain_targets.end());
	    }
	    if (target_dual_proof != nullptr && allow_certificate_domain_replay) {
	      const DualProofRow consumed_target_dual_proof =
	          target_dual_proof != nullptr ? *target_dual_proof : DualProofRow{};
	      const DualProofRow consumed_fallback_dual_proof =
	          fallback_dual_proof != nullptr ? *fallback_dual_proof : DualProofRow{};
	      const DualProofRow consumed_priority_dual_proof =
	          priority_dual_proof != nullptr ? *priority_dual_proof : DualProofRow{};
	      std::vector<BoundChangeInfo> proof_domain_changes;
	      Eigen::VectorXd proof_domain_lb = child.lb;
	      Eigen::VectorXd proof_domain_ub = child.ub;
	      ++out.bc_stats.certificate_domain_passes;
	      const int proof_domain_fixed = apply_dual_proof_domain_fixing(
	          base_lp.vars, *target_dual_proof, opt.int_tol, opt.lp_tol,
	          proof_domain_lb, proof_domain_ub, &proof_domain_changes);
		      if (proof_domain_fixed < 0) {
		        publish_child_bound_exceeding_conflict("proof_domain_cutoff");
		        ++out.bc_stats.certificate_domain_prunes;
		        ++out.bc_stats.rc_fixing_resolve_prunes;
		        branch_obs.cutoff_count = 1;
	        pc_upd.push_back(branch_obs);
	        return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	      }
	      if (proof_domain_fixed > 0) {
	        out.bc_stats.certificate_domain_fixings +=
	            static_cast<std::uint64_t>(proof_domain_fixed);
	        branch_obs.has_inference = true;
	        branch_obs.inference_count +=
	            static_cast<double>(proof_domain_fixed);
		        std::vector<DomainReasonBound> proof_domain_cert_targets;
		        proof_domain_cert_targets.reserve(proof_domain_changes.size());
		        for (const auto& chg : proof_domain_changes) {
		          const int j = chg.var_idx;
		          if (j < 0 || j >= n) continue;
	          const double value =
	              chg.is_lb ? proof_domain_lb[j] : proof_domain_ub[j];
	          if (!std::isfinite(value)) continue;
	          DomainReasonBound raw_target;
	          raw_target.bound = BranchDomainLiteral{j, value, chg.is_lb};
	          raw_target.trail_pos =
	              child.local_domain_trail.empty()
	                  ? static_cast<int>(child.branch_reasons.size() +
	                                     child.domain_reason_bounds.size() +
	                                     proof_domain_cert_targets.size())
	                  : static_cast<int>(child.local_domain_trail.size() +
	                                     proof_domain_cert_targets.size());
	          raw_target.depth = child.depth;
	          std::vector<BranchDomainLiteral> cert_clause;
	          DualProofTargetExplanation cert_explanation;
	          PoolCut cert_cover;
	          double cert_margin = 0.0;
	          if (!build_dual_proof_target_bound_conflict_clause(
	                  base_lp.vars, root.lb, root.ub, child.x_relax,
	                  opt.int_tol, child.branch_reasons,
	                  child.domain_reason_bounds,
	                  child.local_domain_trail.read_ptr(),
	                  child.local_branch_positions.read_ptr(), raw_target,
	                  std::max(std::max(2, opt.reduced_cost_conflict_cut_max_literals),
	                           opt.reduced_cost_conflict_pool_max_literals),
	                  cert_clause, &cert_margin, &cert_cover,
	                  target_dual_proof, &cert_explanation, nullptr,
	                  priority_dual_proof)) {
	            continue;
	          }
	          if (!cert_explanation.valid || cert_explanation.cutoff_conflict) {
	            continue;
	          }
	          DomainReasonBound explained = raw_target;
	          explained.bound = cert_explanation.has_proved_target_bound
	                                ? cert_explanation.proved_target_bound
	                                : raw_target.bound;
	          explained.reason = cert_explanation.resolved_frontier;
	          explained.source_conflict_literal =
	              cert_explanation.flipped_target;
	          explained.has_source_conflict_literal = true;
	          explained.has_proof_activity_audit =
	              cert_explanation.has_proof_activity_audit;
	          explained.proof_activity_margin =
	              cert_explanation.proof_margin;
	          explained.proof_activity = cert_explanation.proof_activity;
	          explained.proof_required_activity =
	              cert_explanation.proof_required_activity;
	          bool duplicate = false;
	          const std::size_t reason_hash =
	              conflict_clause_hash(explained.reason);
	          for (const auto& existing : child.domain_reason_bounds) {
	            if (conflict_clause_hash(existing.reason) == reason_hash &&
	                domain_reason_bounds_equal(existing, explained)) {
	              duplicate = true;
	              break;
	            }
	          }
	          if (duplicate) continue;
	          proof_domain_cert_targets.push_back(std::move(explained));
	        }
	        child.lb = std::move(proof_domain_lb);
	        child.ub = std::move(proof_domain_ub);
	        if (child.x_seed.size() == n) {
	          child.x_seed = clamp_to_bounds(child.x_seed, child.lb, child.ub);
	        }
		        for (const auto& explained : proof_domain_cert_targets) {
		          append_domain_reason_bound(child, explained, n, &root.lb, &root.ub);
		          add_scoped_target_bound_certificate(child, explained);
		          if (publish_bound_lifting_certificate &&
		              explained.has_source_conflict_literal &&
		              !explained.reason.empty()) {
		            publish_bound_lifting_certificate(
		                explained, nullptr, nullptr,
		                "proof_domain_bound_lift");
		          }
		        }
	        if (!proof_domain_cert_targets.empty()) {
	          out.bc_stats.termination_effective_candidates +=
	              static_cast<std::uint64_t>(proof_domain_cert_targets.size());
	          out.bc_stats.termination_effective_admitted +=
	              static_cast<std::uint64_t>(proof_domain_cert_targets.size());
	          branch_obs.conflict_score +=
	              static_cast<double>(proof_domain_cert_targets.size());
	        }
	        if (!propagate_local_binary_implications(child,
	                                                 proof_domain_changes) ||
	            !propagate_scoped_conflict_objects(child,
	                                               proof_domain_changes)) {
	          ++out.bc_stats.certificate_domain_prunes;
	          ++out.bc_stats.rc_fixing_resolve_prunes;
	          branch_obs.cutoff_count = 1;
	          pc_upd.push_back(branch_obs);
	          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	        }
	        std::vector<BranchDomainLiteral> proof_domain_conflict;
	        int proof_domain_prop_tightenings = 0;
	        if (!propagate_node_domain(base_lp, A_row_cached, Aeq_row_cached,
	                                   row_index_cached,
	                                   child.lb, child.ub,
	                                   opt.bound_propagation_rounds,
	                                   &conflict_pool, &clique_table,
	                                   &implication_graph,
	                                   proof_domain_changes,
	                                   child.branch_reasons,
	                                   &proof_domain_conflict,
	                                   &proof_domain_prop_tightenings,
	                                   child.domain_reason_bounds.write_ptr())) {
	          if (!proof_domain_conflict.empty()) {
		            const bool pool_added = add_global_conflict(
		                proof_domain_conflict, "proof_domain_propagation");
		            const bool impl_added =
		                add_binary_implications_from_conflict_clause(
		                    base_lp.vars, proof_domain_conflict, implication_graph);
		            if (pool_added || impl_added) {
		              note_global_domain_learning(1 + (impl_added ? 1 : 0));
		              prune_queue_after_domain_learning(
		                  proof_tail_queue_mode_active(),
		                  "proof_domain_conflict");
		            }
	          }
	          ++out.bc_stats.certificate_domain_prunes;
	          ++out.bc_stats.rc_fixing_resolve_prunes;
	          branch_obs.cutoff_count = 1;
		          pc_upd.push_back(branch_obs);
		          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		        }
		        rebuild_local_domain_trail(child, n, &root.lb, &root.ub);
	        const bool allow_proof_domain_lp_refresh =
	            allow_native_proof_cut_rows &&
	            rc_proof_cut_resolve_lps_left > 0;
	        if (allow_proof_domain_lp_refresh) {
		        node_sf_ref = build_node_sf_with_local_cuts(child);
        node_sf_cuts_augmented = true;
	        SimplexOptions proof_domain_resolve_opt = simplex_opt;
	        proof_domain_resolve_opt.allow_cold_start = true;
	        auto proof_domain_res = solve_tree_lp(
	            node_sf_ref, proof_domain_resolve_opt, child.basis_hint.get());
	        --rc_proof_cut_resolve_lps_left;
	        ++lp_count;
	        ++out.bc_stats.rc_fixing_resolve_lps;
	        ++out.bc_stats.certificate_domain_resolve_lps;
	        const auto proof_domain_type =
	            classify_lp_result(proof_domain_res, n);
		        if (proof_domain_type == LPFailureType::Infeasible ||
		            proof_domain_type == LPFailureType::ObjectiveCutoff) {
		          if (proof_domain_type == LPFailureType::ObjectiveCutoff) {
		            child.bound = std::max(
		                child.bound, proof_domain_res.result.stats.objective);
		          }
		          publish_child_bound_exceeding_conflict("proof_domain_resolve_cutoff");
		          ++out.bc_stats.certificate_domain_prunes;
		          ++out.bc_stats.rc_fixing_resolve_prunes;
	          branch_obs.cutoff_count = 1;
	          pc_upd.push_back(branch_obs);
	          return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
	        }
	        if (proof_domain_type == LPFailureType::Success &&
	            proof_domain_res.result.x.size() == n &&
	            std::isfinite(proof_domain_res.result.stats.objective)) {
	          const double old_bound = child.bound;
	          child.x_relax = clamp_to_bounds(proof_domain_res.result.x,
	                                          child.lb, child.ub);
	          child.x_seed = child.x_relax;
	          child.bound = std::max(child.bound,
	                                 proof_domain_res.result.stats.objective);
	          ++out.bc_stats.rc_proof_resolve_lift_samples;
	          const double lift = std::max(0.0, child.bound - old_bound);
	          out.bc_stats.rc_proof_resolve_lift_sum += lift;
	          out.bc_stats.rc_proof_resolve_lift_max =
	              std::max(out.bc_stats.rc_proof_resolve_lift_max, lift);
	          child.basis_hint =
	              persist_bc_node_basis_from_simplex(proof_domain_res,
	                                              child.basis_hint);
		          if (child.bound > old_bound + 1e-7) {
		            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
		            ++out.bc_stats.certificate_domain_bound_improvements;
	            branch_obs.has_gain = true;
	            branch_obs.gain = std::max(
	                branch_obs.gain,
		                std::max(0.0, child.bound - parent_bound));
		          }
		          if (collect_integral_lp_candidate("proof_domain_resolve")) {
		            pc_upd.push_back(branch_obs);
		            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		          }
		          if (incumbent_prunes_node(has_child_upper_limit,
				                                    child_upper_limit_snapshot,
				                                    child.bound,
				                                    std::max(1e-9, opt.lp_tol))) {
			            publish_child_bound_exceeding_conflict("proof_domain_resolve_bound");
			            ++out.bc_stats.certificate_domain_prunes;
			            ++out.bc_stats.rc_fixing_resolve_prunes;
		            branch_obs.cutoff_count = 1;
		            pc_upd.push_back(branch_obs);
		            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		          }
		          if (prune_child_by_gap_optimality_limit(
		                  "proof_domain_resolve_optimality_limit")) {
		            pc_upd.push_back(branch_obs);
		            return {false, pc_upd, sols, new_cuts, lp_count, cut_count, pool_sep};
		          }
		          // The old proof was consumed into actual domain changes.  Rebuild
		          // the proof from the re-solved LP immediately, then continue the
		          // HiGHS-style target/frontier resolution with a fresh local
		          // certificate instead of silently disabling the learner.
			          child_dual_proof = DualProofRow{};
			          child_objective_event_proof = DualProofRow{};
			          child_rc_mass_proof = DualProofRow{};
			          LPModel proof_lp_scratch;
			          const LPModel& proof_lp = active_lp_for_proof(child, proof_lp_scratch);
				          child_dual_proof_ok = build_full_dual_proof_row(
				              proof_lp, proof_domain_res, root.lb, root.ub, child.x_relax,
				              proof_domain_res.result.stats.objective,
			              child_upper_limit_snapshot,
			              child_dual_proof, std::max(1e-9, opt.lp_tol),
			              &node_sf_ref);
		          if (child_dual_proof_ok) {
		            child_objective_event_proof = child_dual_proof;
		            compact_child_full_dual_proof(child_dual_proof,
		                                          child.x_relax);
		          }
				          child_rc_mass_proof_ok = build_reduced_cost_mass_dual_proof_row(
				              proof_lp, proof_domain_res, root.lb, root.ub, child.x_relax,
				              proof_domain_res.result.stats.objective,
			              child_upper_limit_snapshot,
			              child_rc_mass_proof, std::max(1e-9, opt.lp_tol),
		              &node_sf_ref, &root_implied_integer_cols,
		              &objective_propagation.implied_event_target_cols);
		          target_dual_proof =
		              child_rc_mass_proof_ok
		                  ? &child_rc_mass_proof
		                  : (child_dual_proof_ok ? &child_dual_proof : nullptr);
		          fallback_dual_proof =
		              (child_dual_proof_ok && target_dual_proof != &child_dual_proof)
		                  ? &child_dual_proof
		                  : nullptr;
		          priority_dual_proof =
		              child_rc_mass_proof_ok ? &child_rc_mass_proof
		                                     : target_dual_proof;
		          if (target_dual_proof == nullptr &&
		              consumed_target_dual_proof.valid) {
		            child_rc_mass_proof = consumed_target_dual_proof;
		            child_rc_mass_proof_ok = true;
		            target_dual_proof = &child_rc_mass_proof;
		            if (consumed_fallback_dual_proof.valid) {
		              child_dual_proof = consumed_fallback_dual_proof;
		              child_dual_proof_ok = true;
		              fallback_dual_proof = &child_dual_proof;
		            }
		          }
		          if (priority_dual_proof == nullptr &&
		              consumed_priority_dual_proof.valid) {
		            child_objective_event_proof = consumed_priority_dual_proof;
		            priority_dual_proof = &child_objective_event_proof;
		          }
		          direct_domain_targets.clear();
	        }
	      } else {
	        if (child.x_relax.size() == n) {
	          child.x_relax = clamp_to_bounds(child.x_relax, child.lb, child.ub);
	        }
	        child.lp_refresh_needed = true;
	      }
	      }
	    }
	    bool has_new_proof_target_bounds = false;
	    if (target_dual_proof != nullptr &&
	        opt.enable_reduced_cost_proof_conflict_minimization) {
	      std::vector<DomainReasonBound> proof_target_bounds;
	      int learned_targets = learn_dual_proof_target_bound_conflicts(
	          child.branch_reasons, child.domain_reason_bounds,
	          child.local_domain_trail.read_ptr(),
	          child.local_branch_positions.read_ptr(),
	          child.lb, child.ub, child.x_relax, target_dual_proof,
	          priority_dual_proof,
	          target_dual_proof != nullptr && !direct_domain_targets.empty()
	              ? &direct_domain_targets
	              : nullptr,
	          &local_proof_cuts,
	          &proof_target_bounds, new_cuts, cut_count,
	          std::max(1, opt.reduced_cost_conflict_max_per_node));
	      if (learned_targets == 0 && fallback_dual_proof != nullptr &&
	          fallback_dual_proof != target_dual_proof) {
	        learned_targets = learn_dual_proof_target_bound_conflicts(
	            child.branch_reasons, child.domain_reason_bounds,
	            child.local_domain_trail.read_ptr(),
	            child.local_branch_positions.read_ptr(),
	            child.lb, child.ub, child.x_relax, fallback_dual_proof, nullptr,
	            nullptr,
	            &local_proof_cuts,
	            &proof_target_bounds, new_cuts, cut_count,
	            std::max(1, opt.reduced_cost_conflict_max_per_node));
	        if (learned_targets > 0) target_dual_proof = fallback_dual_proof;
	      }
	      int objective_event_bound_lifts = 0;
		      auto learn_objective_event_proof_fixings =
		          [&](const DualProofRow* proof) -> int {
		        if (strict_highs_lp_contract || proof == nullptr ||
		            !proof->valid || proof->coeff.size() < n ||
		            !std::isfinite(proof->rhs) ||
		            objective_propagation.implied_events.empty() ||
		            rc_proof_cut_resolve_lps_left <= 0) {
	          return 0;
	        }
	        if (!objective_propagation.implied_event_target_cols.empty()) {
	          bool proof_has_usable_event_target = false;
	          for (Eigen::SparseVector<double>::InnerIterator it(proof->coeff);
	               it; ++it) {
	            const int j = static_cast<int>(it.index());
	            if (j >= 0 && j < n && std::abs(it.value()) > 1e-12 &&
	                objective_propagation
	                    .implied_event_target_cols[static_cast<std::size_t>(j)]) {
	              proof_has_usable_event_target = true;
	              break;
	            }
	          }
	          if (!proof_has_usable_event_target) return 0;
	        }
	        auto proof_min_activity =
	            [&](const Eigen::VectorXd& lbv,
	                const Eigen::VectorXd& ubv) -> double {
	          double activity = 0.0;
	          for (Eigen::SparseVector<double>::InnerIterator it(proof->coeff);
	               it; ++it) {
	            const int j = static_cast<int>(it.index());
	            if (j < 0 || j >= n) {
	              return -std::numeric_limits<double>::infinity();
	            }
	            const double a = it.value();
	            if (a > 0.0) {
	              if (!std::isfinite(lbv[j])) {
	                return -std::numeric_limits<double>::infinity();
	              }
	              activity += a * lbv[j];
	            } else if (a < 0.0) {
	              if (!std::isfinite(ubv[j])) {
	                return -std::numeric_limits<double>::infinity();
	              }
	              activity += a * ubv[j];
	            }
	          }
	          return activity;
	        };
	        const double root_activity =
	            proof_min_activity(root.lb, root.ub);
	        const double node_activity =
	            proof_min_activity(child.lb, child.ub);
	        if (!std::isfinite(root_activity) ||
	            !std::isfinite(node_activity)) {
	          return 0;
	        }
	        const double proof_budget = proof->rhs - root_activity;
	        const double node_proof_activity = node_activity - root_activity;
	        if (!std::isfinite(proof_budget) ||
	            !std::isfinite(node_proof_activity)) {
	          return 0;
	        }
	        const double tol = std::max(1e-9, opt.int_tol);
	        const double proof_tol =
	            std::max(1e-7, 1e-10 * (1.0 + std::abs(proof_budget)));
	        const double lp_violation_tol =
	            std::max(1e-7, opt.dynamic_implied_bound_min_violation);
	        auto literal_active = [&](const ObjectivePropagationState::Literal& lit) {
	          return lit.value_one ? (child.lb[lit.col] >= 1.0 - tol)
	                               : (child.ub[lit.col] <= tol);
	        };
	        auto literal_possible =
	            [&](const ObjectivePropagationState::Literal& lit) {
	          return lit.value_one ? (child.ub[lit.col] >= 1.0 - tol)
	                               : (child.lb[lit.col] <= tol);
	        };
	        auto literal_reason =
	            [](const ObjectivePropagationState::Literal& lit) {
	          return BranchDomainLiteral{lit.col, lit.value_one ? 1.0 : 0.0,
	                                     lit.value_one};
	        };
			        struct Candidate {
			          DomainReasonBound rb;
			          double score{0.0};
			        };
			        auto push_objective_event_target =
		            [&](std::vector<Candidate>& target_candidates,
		                const DomainReasonBound& raw_target, double score) {
		          bool duplicate = false;
		          const std::size_t reason_hash =
		              conflict_clause_hash(raw_target.reason);
		          for (const auto& existing : proof_target_bounds) {
	            if (conflict_clause_hash(existing.reason) == reason_hash &&
	                domain_reason_bounds_equal(existing, raw_target,
	                                           std::max(1e-9, opt.int_tol))) {
		              duplicate = true;
		              break;
		            }
		          }
		          if (!duplicate) {
		            for (const auto& existing : target_candidates) {
	              if (conflict_clause_hash(existing.rb.reason) == reason_hash &&
	                  domain_reason_bounds_equal(existing.rb, raw_target,
	                                             std::max(1e-9, opt.int_tol))) {
		                duplicate = true;
		                break;
		              }
		            }
		          }
		          if (duplicate) return;

		          std::vector<BranchDomainLiteral> cert_clause;
		          DualProofTargetExplanation cert_explanation;
		          PoolCut cert_cover;
		          double cert_margin = 0.0;
		          const int max_pool_literals = std::max(
		              std::max(2, opt.reduced_cost_conflict_cut_max_literals),
		              opt.reduced_cost_conflict_pool_max_literals);
		          if (!build_dual_proof_target_bound_conflict_clause(
		                  base_lp.vars, root.lb, root.ub, child.x_relax,
		                  opt.int_tol, child.branch_reasons,
		                  child.domain_reason_bounds,
		                  child.local_domain_trail.read_ptr(),
		                  child.local_branch_positions.read_ptr(), raw_target,
		                  max_pool_literals, cert_clause, &cert_margin,
		                  &cert_cover, proof, &cert_explanation, nullptr,
		                  priority_dual_proof)) {
		            return;
		          }
		          if (!cert_explanation.valid || cert_explanation.cutoff_conflict) {
		            return;
		          }
		          DomainReasonBound explained = raw_target;
		          explained.bound = cert_explanation.has_proved_target_bound
		                                ? cert_explanation.proved_target_bound
		                                : raw_target.bound;
		          explained.reason = cert_explanation.resolved_frontier;
			          explained.source_conflict_literal =
			              cert_explanation.flipped_target;
			          explained.has_source_conflict_literal = true;
			          explained.has_proof_activity_audit =
			              cert_explanation.has_proof_activity_audit;
			          explained.proof_activity_margin =
			              cert_explanation.proof_margin;
			          explained.proof_activity =
			              cert_explanation.proof_activity;
			          explained.proof_required_activity =
			              cert_explanation.proof_required_activity;
			          const int published_reconvergence_bound =
		              publish_bound_lifting_certificate(
		                  explained, nullptr, nullptr,
		                  "objective_event_reconvergence_bound_lift");
		          if (published_reconvergence_bound > 0) {
		            objective_event_bound_lifts +=
		                published_reconvergence_bound;
		          }
			          target_candidates.push_back(Candidate{std::move(explained), score});
			        };
		        auto duplicate_candidate_bound =
		            [&](const std::vector<Candidate>& target_candidates,
		                const DomainReasonBound& raw_target) {
		          const std::size_t reason_hash =
		              conflict_clause_hash(raw_target.reason);
		          for (const auto& existing : proof_target_bounds) {
	            if (conflict_clause_hash(existing.reason) == reason_hash &&
	                domain_reason_bounds_equal(existing, raw_target,
	                                           std::max(1e-9, opt.int_tol))) {
		              return true;
		            }
		          }
		          for (const auto& existing : target_candidates) {
	            if (conflict_clause_hash(existing.rb.reason) == reason_hash &&
	                domain_reason_bounds_equal(existing.rb, raw_target,
	                                           std::max(1e-9, opt.int_tol))) {
		              return true;
		            }
		          }
		          return false;
		        };
			        std::vector<Candidate> candidates;
			        candidates.reserve(
			            static_cast<std::size_t>(
	                std::max(1, opt.reduced_cost_conflict_max_per_node)));
	        std::uint64_t proof_candidates = 0;
	        std::uint64_t skip_coeff = 0;
	        std::uint64_t skip_budget = 0;
	        auto process_missing_literal_events =
	            [&](int missing_col, bool missing_value_one) {
	          const std::size_t key = static_cast<std::size_t>(
	              2 * missing_col + (missing_value_one ? 1 : 0));
	          if (key >= objective_propagation.implied_events_by_literal.size()) {
	            return;
	          }
	          BranchDomainLiteral fixing;
	          BranchDomainLiteral forbidden{
	              missing_col, missing_value_one ? 1.0 : 0.0,
	              missing_value_one};
	          if (missing_value_one) {
	            if (child.ub[missing_col] <= tol) return;
	            fixing = BranchDomainLiteral{missing_col, 0.0, false};

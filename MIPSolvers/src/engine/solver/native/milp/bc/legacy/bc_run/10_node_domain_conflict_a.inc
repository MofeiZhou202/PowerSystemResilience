// @file bc_run/10_node_domain_conflict_a.inc
// BCSolveState::run() segment 10/14: node domain propagation and conflict/implication learning — part 1.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
      compact_trail_frontier(frontier);
      if (frontier.empty()) return std::vector<BranchDomainLiteral>{};

      int depth = scoped_branch_positions == nullptr
                      ? 0
                      : static_cast<int>(scoped_branch_positions->size());
      const double tol = std::max(1e-9, opt.int_tol);
      while (depth > 0) {
        const int branch_pos =
            (*scoped_branch_positions)[static_cast<std::size_t>(depth - 1)];
        if (branch_pos < 0 ||
            branch_pos >= static_cast<int>(scoped_trail->size())) {
          break;
        }
        const auto& branch_entry =
            (*scoped_trail)[static_cast<std::size_t>(branch_pos)];
        if (std::abs(branch_entry.bound.value -
                     branch_entry.prev_bound_value) > tol) {
          break;
        }
        --depth;
      }
      const int start_pos =
          depth <= 0 || scoped_branch_positions == nullptr ||
                  scoped_branch_positions->empty()
              ? 0
              : (*scoped_branch_positions)[static_cast<std::size_t>(depth - 1)] + 1;
      const int end_pos =
          depth >= static_cast<int>(scoped_branch_positions == nullptr
                                        ? 0
                                        : scoped_branch_positions->size())
              ? static_cast<int>(scoped_trail->size())
              : (*scoped_branch_positions)[static_cast<std::size_t>(depth)];
      const int max_resolves = std::min<int>(
          4096, static_cast<int>(scoped_trail->size()) +
                    4 * std::max(1, max_literals));
      HashBucketExactSet<std::vector<TrailFrontierEntry>,
                         decltype(trail_frontier_hash),
                         decltype(trail_frontiers_equal)>
          seen_states(trail_frontier_hash, trail_frontiers_equal);
      for (int iter = 0; iter < max_resolves; ++iter) {
        if (!seen_states.insert(frontier)) break;
        int latest_idx = -1;
        int latest_pos = -1;
        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
          const int pos = frontier[static_cast<std::size_t>(i)].trail_pos;
          if (pos < start_pos || pos < 0 ||
              pos >= static_cast<int>(scoped_trail->size()) ||
              pos > end_pos) {
            continue;
          }
          const auto& entry =
              (*scoped_trail)[static_cast<std::size_t>(pos)];
          if (entry.is_branch || entry.reason.empty()) continue;
          if (pos > latest_pos) {
            latest_pos = pos;
            latest_idx = i;
          }
        }
        if (latest_idx < 0) break;
        const auto& entry =
            (*scoped_trail)[static_cast<std::size_t>(latest_pos)];
        std::vector<TrailFrontierEntry> replacement;
        replacement.reserve(entry.reason.size());
        bool replacement_valid = true;
        for (const auto& reason_lit : entry.reason) {
          auto repl = trail_entry_for_literal(reason_lit, latest_pos);
          if (!repl.has_value()) {
            replacement_valid = false;
            break;
          }
          replacement.push_back(*repl);
        }
        if (!replacement_valid || replacement.empty()) break;
        frontier.erase(frontier.begin() + latest_idx);
        frontier.insert(frontier.end(),
                        std::make_move_iterator(replacement.begin()),
                        std::make_move_iterator(replacement.end()));
        compact_trail_frontier(frontier);
      }
      return trail_frontier_literals(frontier);
    };

    auto flip_bound = [&](const BranchDomainLiteral& lit,
                          BranchDomainLiteral& out) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      const bool integer_var =
          is_integer_type(base_lp.vars[static_cast<std::size_t>(lit.var_idx)]);
      if (lit.is_lb) {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::ceil(lit.value - 1e-9) - 1.0 : lit.value - 1e-7,
            false};
      } else {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::floor(lit.value + 1e-9) + 1.0 : lit.value + 1e-7,
            true};
      }
      return true;
    };

    auto local_binary_cut_violation =
        [&](const std::vector<BranchDomainLiteral>& clause) -> double {
      if (local_x == nullptr || local_x->size() != n) return 0.0;
      PoolCut cut;
      if (!try_build_binary_conflict_cut(base_lp, clause, cut, max_cut_literals)) {
        return 0.0;
      }
      return sparse_dot(cut.coeff, *local_x) - cut.rhs;
    };

    auto build_local_implication_proof_cut =
        [&](const std::vector<BranchDomainLiteral>& reason,
            const BranchDomainLiteral& source,
            PoolCut& out_cut) -> bool {
      if (local_x == nullptr || local_x->size() != n ||
          local_lb == nullptr || local_lb->size() != n ||
          local_ub == nullptr || local_ub->size() != n ||
          source.var_idx < 0 || source.var_idx >= n ||
          reason.empty()) {
        return false;
      }
      const int target = source.var_idx;
      const bool integer_target =
          is_integer_type(base_lp.vars[static_cast<std::size_t>(target)]);
      constexpr double tol = 1e-9;
      const bool implies_upper = source.is_lb;
      double target_bound = 0.0;
      double big_m = 0.0;
      if (implies_upper) {
        target_bound = integer_target
            ? std::ceil(source.value - tol) - 1.0
            : source.value - 1e-7;
        if (!std::isfinite((*local_ub)[target])) return false;
        big_m = (*local_ub)[target] - target_bound;
      } else {
        target_bound = integer_target
            ? std::floor(source.value + tol) + 1.0
            : source.value + 1e-7;
        if (!std::isfinite((*local_lb)[target])) return false;
        big_m = target_bound - (*local_lb)[target];
      }
      if (!std::isfinite(big_m) || big_m <= 1e-8) return false;

      Eigen::SparseVector<double> coeff(n);
      coeff.reserve(static_cast<int>(reason.size()) + 1);
      coeff.insertBack(target) = implies_upper ? 1.0 : -1.0;
      double rhs = implies_upper ? target_bound : -target_bound;
      double norm2 = 1.0;
      int fixed_one_terms = 0;
      for (const auto& lit : reason) {
        const int j = lit.var_idx;
        if (j < 0 || j >= n ||
            base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
          return false;
        }
        const bool fixed_zero = !lit.is_lb && lit.value <= tol;
        const bool fixed_one = lit.is_lb && lit.value >= 1.0 - tol;
        if (!fixed_zero && !fixed_one) return false;
        if (fixed_one) {
          coeff.coeffRef(j) += big_m;
          ++fixed_one_terms;
        } else {
          coeff.coeffRef(j) -= big_m;
        }
        norm2 += big_m * big_m;
      }
      rhs += big_m * static_cast<double>(fixed_one_terms);
      coeff.prune(0.0, 0.0);
      out_cut = PoolCut{std::move(coeff), rhs, 0, 0.0, std::sqrt(norm2), 0};
      return sparse_dot(out_cut.coeff, *local_x) > out_cut.rhs + 1e-7;
    };

    for (const auto& rb : reason_bounds) {
      if (rb.reason.empty()) continue;
      BranchDomainLiteral flipped;
      if (rb.has_source_conflict_literal) {
        flipped = rb.source_conflict_literal;
      } else if (!flip_bound(rb.bound, flipped)) {
        continue;
      }
	      std::vector<BranchDomainLiteral> clause = resolve_scoped_trail_frontier(
	          rb.reason,
	          rb.trail_pos >= 0
	              ? rb.trail_pos
	              : static_cast<int>(branch_reasons.size() + reason_bounds.size()));
	      PoolCut implication_cut;
	      const bool implication_viol =
	          !strict_highs_lp_contract &&
	          build_local_implication_proof_cut(clause, flipped, implication_cut);
	      clause.push_back(flipped);
	      canonicalize_branch_literals(clause);
	      if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
	        continue;
	      }
      if (!seen.insert(clause)) continue;
      const double cut_viol = local_binary_cut_violation(clause);
      const int short_limit = std::min(8, max_literals);
	      const int queue_hits =
	          static_cast<int>(clause.size()) <= short_limit
	              ? node_queue.count_clause_hits_near_lower_bound(
	                    clause, termination_bound_band(), true, 8)
	              : 0;
	      if (!strict_highs_lp_contract && cut_viol <= 1e-7 &&
	          queue_hits <= 0 && !implication_viol) {
	        continue;
	      }
      FrontierCandidate cand;
      cand.score = (cut_viol > 1e-7 ? 1000000.0 * cut_viol : 0.0) +
                   (implication_viol ? 500000.0 : 0.0) +
                   1000.0 * static_cast<double>(queue_hits) -
                   static_cast<double>(clause.size());
      cand.cut_violation = cut_viol;
      cand.queue_hits = queue_hits;
      cand.trail_pos = rb.trail_pos;
      cand.depth = rb.depth;
      cand.frontier_literals = resolve_scoped_trail_frontier(
          rb.reason,
          rb.trail_pos >= 0
              ? rb.trail_pos
              : static_cast<int>(branch_reasons.size() + reason_bounds.size()));
      canonicalize_branch_literals(cand.frontier_literals);
      cand.target_bound = rb.bound;
      cand.source_literal = flipped;
      cand.has_bound_lift =
          !cand.frontier_literals.empty() &&
          cand.target_bound.var_idx >= 0 && cand.target_bound.var_idx < n &&
          std::isfinite(cand.target_bound.value) &&
          cand.source_literal.var_idx >= 0 && cand.source_literal.var_idx < n &&
          std::isfinite(cand.source_literal.value) &&
          static_cast<int>(cand.frontier_literals.size() + 1) <= max_literals;
      if (implication_viol) cand.local_implication_cut = std::move(implication_cut);
      cand.clause = std::move(clause);
      candidates.push_back(std::move(cand));
    }

    if (candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) {
                if (a.score != b.score) return a.score > b.score;
                if (a.clause.size() != b.clause.size()) {
                  return a.clause.size() < b.clause.size();
                }
                return conflict_clause_hash(a.clause) <
                       conflict_clause_hash(b.clause);
              });

    bool learned_any = false;
    int learned_frontier = 0;
	    for (const auto& candidate : candidates) {
	      if (learned_frontier >= max_frontier_clauses) break;
	      if (bc_time_limit_expired()) break;
	      const auto& frontier_clause = candidate.clause;
	      if (!strict_highs_lp_contract && candidate.cut_violation > 1e-7 &&
	          local_proof_cuts != nullptr) {
	        PoolCut local_cut;
	        if (try_build_binary_conflict_cut(base_lp, frontier_clause, local_cut,
	                                          max_cut_literals) &&
	            local_x != nullptr &&
	            local_x->size() == n &&
            valid_pool_cut(local_cut, local_x, "reconv_binary_cut") &&
            sparse_dot(local_cut.coeff, *local_x) > local_cut.rhs + 1e-7) {
          ++out.bc_stats.local_node_cuts_generated;
          local_proof_cuts->push_back(
              make_local_node_cut(std::move(local_cut)));
        }
      }
		      if (!strict_highs_lp_contract &&
		          candidate.local_implication_cut.has_value() &&
		          local_proof_cuts != nullptr &&
		          valid_pool_cut(*candidate.local_implication_cut, local_x,
		                         "reconv_implication_cut")) {
	        PoolCut cut = *candidate.local_implication_cut;
	        ++out.bc_stats.local_node_cuts_generated;
	        local_proof_cuts->push_back(make_local_node_cut(std::move(cut)));
	      }
      int bound_lift_published = 0;
	      if (scoped_node != nullptr) {
	        bool scoped_added = false;
	        if (add_scoped_conflict_object(*scoped_node, frontier_clause, nullptr)) {
	          ++out.bc_stats.rc_proof_conflict_clauses_learned;
	          out.bc_stats.rc_proof_conflict_literals_before +=
	              static_cast<std::uint64_t>(frontier_clause.size());
	          out.bc_stats.rc_proof_conflict_literals_after +=
	              static_cast<std::uint64_t>(frontier_clause.size());
	          scoped_added = true;
	        }
		        // This path is a scoped implication learned from already-stored
		        // local reasons, not a row-resolved reconvergence proof.  Keep it
		        // node-local; globally consumable bound-lifting certificates are
		        // emitted only by the dual-proof local-trail resolver below.
		        (void)candidate;
	        std::vector<std::vector<BranchDomainLiteral>> learned_clauses;
	        conflict_pool.add_with_binary_resolution(
	            base_lp.vars, frontier_clause,
	            /*max_resolvents=*/4, &learned_clauses);
	        bool global_added = false;
	        for (const auto& learned_clause : learned_clauses) {
	          const bool impl_added =
	              add_binary_implications_from_conflict_clause(base_lp.vars,
	                                                           learned_clause,
	                                                           implication_graph);
	          if (impl_added) {
	            ++out.bc_stats.rc_binary_implications_learned;
	            note_global_domain_learning();
	          }
	          if (!strict_highs_lp_contract &&
	              static_cast<int>(learned_clause.size()) <= max_cut_literals) {
	            PoolCut conflict_cut;
	            if (try_build_binary_conflict_cut(base_lp, learned_clause,
	                                              conflict_cut, max_cut_literals) &&
	                valid_pool_cut(conflict_cut, local_x,
	                               "scoped_reconv_global_cut")) {
	              if (local_x == nullptr ||
	                  local_x->size() != n ||
	                  static_cast<int>(learned_clause.size()) <= 2 ||
	                  sparse_dot(conflict_cut.coeff, *local_x) >
	                      conflict_cut.rhs + 1e-7) {
	                new_cuts.push_back(std::move(conflict_cut));
	                ++cut_count;
	                ++out.bc_stats.rc_conflict_cuts_added;
	              }
	            }
	          }
	          ++out.bc_stats.rc_conflict_clauses_learned;
	          ++out.bc_stats.rc_proof_conflict_clauses_learned;
	          out.bc_stats.rc_proof_conflict_literals_before +=
	              static_cast<std::uint64_t>(frontier_clause.size());
	          out.bc_stats.rc_proof_conflict_literals_after +=
	              static_cast<std::uint64_t>(learned_clause.size());
	          global_added = true;
	        }
	        if (global_added) {
	          note_global_domain_learning(
	              static_cast<std::uint64_t>(learned_clauses.size()));
	        }
	        learned_any = learned_any || scoped_added || global_added ||
                          bound_lift_published > 0;
	        ++learned_frontier;
	        continue;
	      }

      std::vector<std::vector<BranchDomainLiteral>> learned_clauses;
      conflict_pool.add_with_binary_resolution(
          base_lp.vars, frontier_clause,
          /*max_resolvents=*/4, &learned_clauses);
      if (learned_clauses.empty()) {
        if (bound_lift_published > 0) ++learned_frontier;
        continue;
      }

      for (const auto& learned_clause : learned_clauses) {
        const bool impl_added =
            add_binary_implications_from_conflict_clause(base_lp.vars,
                                                         learned_clause,
                                                         implication_graph);
        if (impl_added) {
          ++out.bc_stats.rc_binary_implications_learned;
          note_global_domain_learning();
        }
        if (!strict_highs_lp_contract &&
            static_cast<int>(learned_clause.size()) <= max_cut_literals) {
          PoolCut conflict_cut;
          if (try_build_binary_conflict_cut(base_lp, learned_clause,
                                            conflict_cut, max_cut_literals)) {
            new_cuts.push_back(std::move(conflict_cut));
            ++cut_count;
            ++out.bc_stats.rc_conflict_cuts_added;
          }
        }
        ++out.bc_stats.rc_conflict_clauses_learned;
        ++out.bc_stats.rc_proof_conflict_clauses_learned;
        out.bc_stats.rc_proof_conflict_literals_before +=
            static_cast<std::uint64_t>(frontier_clause.size());
        out.bc_stats.rc_proof_conflict_literals_after +=
            static_cast<std::uint64_t>(learned_clause.size());
        learned_any = true;
      }
      ++learned_frontier;
    }
    return learned_any;
  };

	  auto learn_dual_proof_target_bound_conflicts =
	      [&](const std::vector<BranchDomainLiteral>& branch_reasons,
	          const std::vector<DomainReasonBound>& reason_bounds,
	          const std::vector<LocalDomainTrailEntry>* local_domain_trail,
	          const std::vector<int>* local_branch_positions,
	          const Eigen::VectorXd& node_lb,
	          const Eigen::VectorXd& node_ub,
	          const Eigen::VectorXd& x_relax,
	          const DualProofRow* dual_proof,
	          const DualProofRow* priority_proof,
	          const std::vector<DomainReasonBound>* direct_domain_targets,
	          std::vector<PoolCut>* local_proof_cuts,
	          std::vector<DomainReasonBound>* local_target_bounds,
	          std::vector<PoolCut>& new_cuts,
	          int& cut_count,
	          int max_to_learn) -> int {
    if (!opt.enable_reduced_cost_proof_conflict_minimization ||
        dual_proof == nullptr || !dual_proof->valid ||
	        (reason_bounds.empty() && branch_reasons.empty() &&
	         (direct_domain_targets == nullptr || direct_domain_targets->empty())) ||
	        node_lb.size() != n || node_ub.size() != n ||
	        max_to_learn <= 0) {
	      return 0;
	    }
    const int max_cut_literals =
        std::max(2, opt.reduced_cost_conflict_cut_max_literals);
    const int max_pool_literals =
        std::max(max_cut_literals,
                 opt.reduced_cost_conflict_pool_max_literals);
    const bool direct_target_mode =
        direct_domain_targets != nullptr && !direct_domain_targets->empty();
    const bool use_priority_proof =
        priority_proof != nullptr && priority_proof->valid &&
        priority_proof->coeff.size() >= n;
    auto target_priority_coeff = [&](int j) -> double {
      if (!use_priority_proof || j < 0 || j >= n) return 0.0;
      const double v = priority_proof->coeff.coeff(j);
      return std::isfinite(v) ? v : 0.0;
    };

	    struct TargetCandidate {
	      const DomainReasonBound* rb{nullptr};
	      double score{0.0};
	      double proof_activity{0.0};
	      bool direct_domain_target{false};
	    };
	    std::vector<DomainReasonBound> branch_targets;
	    branch_targets.reserve(branch_reasons.size());
    for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
      DomainReasonBound rb;
      rb.bound = branch_reasons[static_cast<std::size_t>(i)];
      rb.trail_pos = i;
      rb.depth = i;
      branch_targets.push_back(std::move(rb));
	    }
	    const bool have_local_domain_trail =
	        local_domain_trail != nullptr && !local_domain_trail->empty();
	    const int synthetic_target_trail_pos =
	        have_local_domain_trail
	            ? static_cast<int>(local_domain_trail->size())
	            : static_cast<int>(branch_reasons.size() + reason_bounds.size());
	    std::vector<DomainReasonBound> local_trail_targets;
	    if (have_local_domain_trail) {
	      local_trail_targets.reserve(local_domain_trail->size());
	      for (const auto& entry : *local_domain_trail) {
	        const int j = entry.bound.var_idx;
	        if (j < 0 || j >= n || !std::isfinite(entry.bound.value) ||
	            entry.pos < 0) {
	          continue;
	        }
	        const double a = dual_proof->coeff.coeff(j);
	        const bool target_side =
	            (entry.bound.is_lb && a < -1e-12) ||
	            (!entry.bound.is_lb && a > 1e-12);
	        if (!target_side) continue;
	        DomainReasonBound rb;
	        rb.bound = entry.bound;
	        rb.reason = entry.reason;
	        rb.trail_pos = entry.pos;
	        rb.depth = entry.depth;
	        rb.source_conflict_literal = entry.source_conflict_literal;
	        rb.has_source_conflict_literal = entry.has_source_conflict_literal;
	        rb.prev_bound_pos = entry.prev_bound_pos;
	        rb.prev_bound_value = entry.prev_bound_value;
	        local_trail_targets.push_back(std::move(rb));
	      }
	    }
	    auto compute_node_min_activity = [&]() -> double {
	      double activity = 0.0;
	      for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff);
	           it; ++it) {
	        const int j = static_cast<int>(it.index());
	        if (j < 0 || j >= n) return -std::numeric_limits<double>::infinity();
	        const double a = it.value();
	        if (a > 0.0) {
	          if (!std::isfinite(node_lb[j])) {
	            return -std::numeric_limits<double>::infinity();
	          }
	          activity += a * node_lb[j];
	        } else if (a < 0.0) {
	          if (!std::isfinite(node_ub[j])) {
	            return -std::numeric_limits<double>::infinity();
	          }
	          activity += a * node_ub[j];
	        }
	      }
	      return activity;
	    };
	    std::vector<DomainReasonBound> proof_activity_targets;
	    const double node_min_activity = compute_node_min_activity();
	    if (std::isfinite(node_min_activity)) {
	      struct ProofTargetTmp {
	        DomainReasonBound rb;
	        double score{0.0};
	        double proof_activity{0.0};
	      };
	      std::vector<ProofTargetTmp> tmp;
	      for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff);
	           it; ++it) {
	        const int j = static_cast<int>(it.index());
	        if (j < 0 || j >= n) continue;
	        const double a = it.value();
	        if (std::abs(a) <= 1e-12) continue;
	        double implied = 0.0;
	        bool is_lb = false;
	        double movement = 0.0;
	        if (a > 0.0) {
	          if (!std::isfinite(node_lb[j]) || !std::isfinite(node_ub[j])) continue;
	          const double min_other = node_min_activity - a * node_lb[j];
	          implied = (dual_proof->rhs - min_other) / a;
	          if (is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
	            implied = std::floor(implied + 1e-9);
	          }
	          if (!(implied < node_ub[j] - 1e-7)) continue;
	          is_lb = false;
	          movement = node_ub[j] - implied;
	        } else {
	          if (!std::isfinite(node_lb[j]) || !std::isfinite(node_ub[j])) continue;
	          const double min_other = node_min_activity - a * node_ub[j];
	          implied = (dual_proof->rhs - min_other) / a;
	          if (is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
	            implied = std::ceil(implied - 1e-9);
	          }
	          if (!(implied > node_lb[j] + 1e-7)) continue;
	          is_lb = true;
	          movement = implied - node_lb[j];
	        }
	        if (!std::isfinite(implied) || movement <= 1e-7) continue;
	        const double lp_violation = is_lb
	            ? std::max(0.0, implied - x_relax[j])
	            : std::max(0.0, x_relax[j] - implied);
	        const double proof_activity =
	            std::abs(a) * (movement + 10.0 * lp_violation);
	        const double priority_activity =
	            std::abs(target_priority_coeff(j)) *
	            (movement + 10.0 * lp_violation);
	        DomainReasonBound rb;
	        rb.bound = BranchDomainLiteral{j, implied, is_lb};
	        rb.trail_pos = synthetic_target_trail_pos;
	        rb.depth = static_cast<int>(branch_reasons.size());
	        tmp.push_back(ProofTargetTmp{
	            std::move(rb),
	            proof_activity + 4.0 * priority_activity +
	                1.0 / std::max(1.0, 1.0 + movement),
	            proof_activity});
	      }
	      std::sort(tmp.begin(), tmp.end(),
	                [](const ProofTargetTmp& a, const ProofTargetTmp& b) {
	                  if (a.score != b.score) return a.score > b.score;
	                  return a.rb.bound.var_idx < b.rb.bound.var_idx;
	                });
	      const int keep = std::min<int>(
	          tmp.size(), std::max(2, 2 * std::max(1, max_to_learn)));
	      proof_activity_targets.reserve(static_cast<std::size_t>(keep));
	      for (int i = 0; i < keep; ++i) {
	        proof_activity_targets.push_back(tmp[static_cast<std::size_t>(i)].rb);
	      }
	    }
	    std::vector<TargetCandidate> targets;
	    targets.reserve(reason_bounds.size() + branch_targets.size() +
	                    local_trail_targets.size() +
	                    proof_activity_targets.size() +
	                    (direct_domain_targets != nullptr
	                         ? direct_domain_targets->size()
	                         : 0));
	    auto add_target_candidate = [&](const DomainReasonBound& rb,
	                                    double activity_bonus = 0.0,
	                                    bool direct_domain_target = false) {
	      if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
	          !std::isfinite(rb.bound.value)) {
	        return;
      }
      const double a = dual_proof->coeff.coeff(rb.bound.var_idx);
      if ((rb.bound.is_lb && a >= -1e-12) ||
          (!rb.bound.is_lb && a <= 1e-12)) {
        return;
      }
      double movement = 0.0;
      double node_movement = 0.0;
      if (rb.bound.is_lb) {
        if (std::isfinite(root.lb[rb.bound.var_idx])) {
          movement = std::max(0.0, rb.bound.value - root.lb[rb.bound.var_idx]);
        }
        if (std::isfinite(node_lb[rb.bound.var_idx])) {
          node_movement =
              std::max(0.0, rb.bound.value - node_lb[rb.bound.var_idx]);
        }
      } else {
        if (std::isfinite(root.ub[rb.bound.var_idx])) {
          movement = std::max(0.0, root.ub[rb.bound.var_idx] - rb.bound.value);
        }
        if (std::isfinite(node_ub[rb.bound.var_idx])) {
          node_movement =
              std::max(0.0, node_ub[rb.bound.var_idx] - rb.bound.value);
        }
	      }
	      const double lp_violation = rb.bound.is_lb
	          ? std::max(0.0, rb.bound.value - x_relax[rb.bound.var_idx])
	          : std::max(0.0, x_relax[rb.bound.var_idx] - rb.bound.value);
	      const double compact_bonus =
	          1.0 / static_cast<double>(1 + rb.reason.size());
	      const double proof_mass =
	          std::abs(a) * (node_movement + 10.0 * lp_violation);
	      const double priority_bonus =
	          std::abs(target_priority_coeff(rb.bound.var_idx)) *
	          (node_movement + 10.0 * lp_violation);
	      targets.push_back(TargetCandidate{
	          &rb, proof_mass + 4.0 * priority_bonus +
	                   activity_bonus + 1e-3 * std::abs(a) * movement +
	                   compact_bonus,
	          activity_bonus, direct_domain_target});
	    };
	    if (direct_domain_targets != nullptr) {
	      for (const auto& rb : *direct_domain_targets) {
	        const double a = dual_proof->coeff.coeff(rb.bound.var_idx);
	        double movement = 0.0;
	        if (rb.bound.var_idx >= 0 && rb.bound.var_idx < n) {
	          if (rb.bound.is_lb) {
	            movement = std::max(0.0, rb.bound.value - node_lb[rb.bound.var_idx]);
	          } else {
	            movement = std::max(0.0, node_ub[rb.bound.var_idx] - rb.bound.value);
	          }
	        }
	        add_target_candidate(rb, 10.0 * std::abs(a) * movement,
	                             /*direct_domain_target=*/true);
	      }
	    }
	    if (!direct_target_mode) {
	      for (const auto& rb : reason_bounds) {
	        if (rb.reason.empty()) continue;
	        add_target_candidate(rb);
	      }
	      for (const auto& rb : branch_targets) {
	        add_target_candidate(rb);
	      }
	      for (const auto& rb : local_trail_targets) {
	        add_target_candidate(rb);
	      }
	    }
	    for (const auto& rb : proof_activity_targets) {
	      const double a = dual_proof->coeff.coeff(rb.bound.var_idx);
	      double movement = 0.0;
	      if (rb.bound.is_lb) {
	        movement = std::max(0.0, rb.bound.value - node_lb[rb.bound.var_idx]);
	      } else {
	        movement = std::max(0.0, node_ub[rb.bound.var_idx] - rb.bound.value);
	      }
	      add_target_candidate(rb, 25.0 * std::abs(a) * movement,
	                           /*direct_domain_target=*/true);
	    }
    out.bc_stats.rc_target_proof_candidates +=
        static_cast<std::uint64_t>(targets.size());
    if (targets.empty()) return 0;
    std::sort(targets.begin(), targets.end(),
              [](const TargetCandidate& a, const TargetCandidate& b) {
                if (a.score != b.score) return a.score > b.score;
                return a.rb->trail_pos > b.rb->trail_pos;
              });

	    const int max_attempts =
	        std::min<int>(targets.size(), std::max(4, 3 * max_to_learn));
    int learned = 0;
    int local_rows_added = 0;
	    BranchLiteralListSet seen;
	    for (int t = 0; t < max_attempts && learned < max_to_learn; ++t) {
	      if (bc_time_limit_expired()) break;
		      const DomainReasonBound& rb = *targets[static_cast<std::size_t>(t)].rb;
		      const bool direct_domain_target =
		          targets[static_cast<std::size_t>(t)].direct_domain_target;
	      bool target_bound_admitted = false;
		      std::vector<BranchDomainLiteral> clause;
      double proof_margin = 0.0;
      PoolCut proof_cover_cut;
      std::vector<PoolCut> extra_proof_cover_cuts;
      DualProofTargetExplanation explanation;
      ++out.bc_stats.rc_target_proof_attempts;
      if (!build_dual_proof_target_bound_conflict_clause(
              base_lp.vars, root.lb, root.ub, x_relax, opt.int_tol,
              branch_reasons, reason_bounds, local_domain_trail,
              local_branch_positions, rb, max_pool_literals, clause,
              &proof_margin, &proof_cover_cut, dual_proof, &explanation,
              &extra_proof_cover_cuts, priority_proof)) {
        continue;
      }
      ++out.bc_stats.rc_target_proof_successes;
      (void)proof_margin;
      if (explanation.valid &&
          std::isfinite(explanation.proof_budget) &&
          std::isfinite(explanation.frontier_lp_activity) &&
          std::isfinite(explanation.local_proof_cover_violation)) {
        ++out.bc_stats.rc_target_proof_diagnostic_samples;
        const double budget = std::max(0.0, explanation.proof_budget);
        const double activity = std::max(0.0, explanation.frontier_lp_activity);
        const double violation =
            std::max(0.0, explanation.local_proof_cover_violation);
        out.bc_stats.rc_target_proof_budget_sum += budget;
        out.bc_stats.rc_target_proof_budget_max =
            std::max(out.bc_stats.rc_target_proof_budget_max, budget);
        out.bc_stats.rc_target_frontier_lp_activity_sum += activity;
        out.bc_stats.rc_target_frontier_lp_activity_max =
            std::max(out.bc_stats.rc_target_frontier_lp_activity_max, activity);
        out.bc_stats.rc_target_cover_violation_sum += violation;
        out.bc_stats.rc_target_cover_violation_max =
            std::max(out.bc_stats.rc_target_cover_violation_max, violation);
      }
      int published_reconvergence_certificate = 0;
      if (explanation.valid && !explanation.cutoff_conflict &&
          publish_bound_lifting_certificate) {
        const BranchDomainLiteral effective_bound =
            explanation.has_proved_target_bound
                ? explanation.proved_target_bound
                : rb.bound;
        if (effective_bound.var_idx >= 0 && effective_bound.var_idx < n &&
            std::isfinite(effective_bound.value)) {
          DomainReasonBound lifted = rb;
          lifted.bound = effective_bound;
          lifted.reason = explanation.resolved_frontier;
          lifted.source_conflict_literal = explanation.flipped_target;
          lifted.has_source_conflict_literal = true;
          lifted.has_proof_activity_audit =
              explanation.has_proof_activity_audit;
          lifted.proof_activity_margin = explanation.proof_margin;
          lifted.proof_activity = explanation.proof_activity;
          lifted.proof_required_activity =
              explanation.proof_required_activity;
          lifted.trail_pos = std::max(
              lifted.trail_pos,
              static_cast<int>(branch_reasons.size() + reason_bounds.size()));
          published_reconvergence_certificate =
              publish_bound_lifting_certificate(
                  lifted, nullptr, nullptr,
                  "target_reconvergence_bound_lift");
        }
      }
	      auto admit_explained_unit_or_domain_bound = [&]() -> bool {
	        if (local_target_bounds == nullptr ||
	            !explanation.valid || explanation.cutoff_conflict ||
	            static_cast<int>(explanation.resolved_frontier.size()) >
	                std::min(8, max_pool_literals)) {
	          return false;
	        }
        const BranchDomainLiteral effective_bound =
            explanation.has_proved_target_bound
                ? explanation.proved_target_bound
                : rb.bound;
        if (effective_bound.var_idx < 0 || effective_bound.var_idx >= n) {
          return false;
        }
        ++out.bc_stats.termination_effective_candidates;
	        const double lp_bound_violation = effective_bound.is_lb
	            ? std::max(0.0, effective_bound.value -
	                                x_relax[effective_bound.var_idx])
	            : std::max(0.0, x_relax[effective_bound.var_idx] -
	                                effective_bound.value);
	        (void)lp_bound_violation;
        auto literal_active_at_node = [&](const BranchDomainLiteral& lit) {
          if (lit.var_idx < 0 || lit.var_idx >= n) return false;
          const double tol = std::max(1e-9, opt.int_tol);
          return lit.is_lb ? (node_lb[lit.var_idx] >= lit.value - tol)
                           : (node_ub[lit.var_idx] <= lit.value + tol);
        };
	        if (explanation.resolved_frontier.size() == 1) {
	          const auto& trig = explanation.resolved_frontier.front();
	          if (trig.var_idx < 0 || trig.var_idx >= n ||
	              base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type !=
	                  VarType::Binary) {
	            return false;
          }
          const bool trigger_one = trig.is_lb && trig.value >= 1.0 - opt.int_tol;
          const bool trigger_zero = !trig.is_lb && trig.value <= opt.int_tol;
	          if (!trigger_one && !trigger_zero) return false;
	          if (!literal_active_at_node(trig)) return false;
	        }
        for (const auto& existing : *local_target_bounds) {
          if (branch_literal_stronger_or_equal(existing.bound, effective_bound) ||
              branch_literal_stronger_or_equal(effective_bound, existing.bound)) {
            return false;
          }
        }
        if (static_cast<int>(local_target_bounds->size()) >=
            std::max(1, 2 * max_to_learn)) {
          return false;
        }
        DomainReasonBound explained = rb;
        explained.bound = effective_bound;
        explained.reason = explanation.resolved_frontier;
        explained.source_conflict_literal = explanation.flipped_target;
        explained.has_source_conflict_literal = true;
        explained.has_proof_activity_audit =
            explanation.has_proof_activity_audit;
        explained.proof_activity_margin = explanation.proof_margin;
        explained.proof_activity = explanation.proof_activity;
        explained.proof_required_activity =
            explanation.proof_required_activity;
        explained.trail_pos = std::max(
            explained.trail_pos,
            static_cast<int>(branch_reasons.size() + reason_bounds.size() +
                             local_target_bounds->size()));
	        local_target_bounds->push_back(std::move(explained));
	        ++out.bc_stats.termination_effective_admitted;
	        return true;
	      };
      bool proof_cover_admitted = false;
      const bool proof_cover_violated =
          local_proof_cuts != nullptr &&
          proof_cover_cut.coeff.size() == n &&
          proof_cover_cut.coeff.nonZeros() > 0 &&
          valid_pool_cut(proof_cover_cut, &x_relax, "target_proof_cover") &&
          sparse_dot(proof_cover_cut.coeff, x_relax) >
              proof_cover_cut.rhs + 1e-7;
      if (proof_cover_violated) {
        ++out.bc_stats.local_node_cuts_generated;
        local_proof_cuts->push_back(
            make_local_node_cut(std::move(proof_cover_cut)));
        proof_cover_admitted = true;
        for (auto& extra_cut : extra_proof_cover_cuts) {
          if (valid_pool_cut(extra_cut, &x_relax, "target_extra_proof_cover") &&
              extra_cut.coeff.size() == n && extra_cut.coeff.nonZeros() > 0 &&
              sparse_dot(extra_cut.coeff, x_relax) > extra_cut.rhs + 1e-7) {
            ++out.bc_stats.local_node_cuts_generated;
            local_proof_cuts->push_back(
                make_local_node_cut(std::move(extra_cut)));
          }
        }
      }
	      target_bound_admitted = admit_explained_unit_or_domain_bound();
      if (direct_domain_target) {
        if (target_bound_admitted || proof_cover_admitted ||
            published_reconvergence_certificate > 0) {
          ++learned;
        }
        continue;
      }
      canonicalize_branch_literals(clause);
	      if (clause.empty() || static_cast<int>(clause.size()) > max_pool_literals) {
	        continue;
	      }
	      if (!seen.insert(clause)) continue;
		      bool target_binary_impl_added = false;
      if (explanation.valid && !explanation.cutoff_conflict &&
          explanation.resolved_frontier.size() == 1 && clause.size() == 2) {
        target_binary_impl_added =
            add_binary_implications_from_conflict_clause(base_lp.vars, clause,
                                                         implication_graph);
        if (target_binary_impl_added) {
          ++out.bc_stats.rc_binary_implications_learned;
        }
      }

      const int queue_hits =
          static_cast<int>(clause.size()) <= std::min(8, max_pool_literals)
              ? node_queue.count_clause_hits_near_lower_bound(
                    clause, termination_bound_band(), true, 8)
              : 0;
      PoolCut direct_clause_cut;
      const bool direct_clause_cut_violated =
          local_proof_cuts != nullptr &&
          local_rows_added < max_to_learn &&
          static_cast<int>(clause.size()) <= max_cut_literals &&
          try_build_binary_conflict_cut(base_lp, clause, direct_clause_cut,
                                        max_cut_literals) &&
          valid_pool_cut(direct_clause_cut, &x_relax,
                         "target_direct_clause_cut") &&
          sparse_dot(direct_clause_cut.coeff, x_relax) >
              direct_clause_cut.rhs + 1e-7;
	      if (!proof_cover_violated && queue_hits <= 0 &&
	          !direct_clause_cut_violated && !target_binary_impl_added &&
	          !target_bound_admitted &&
	          published_reconvergence_certificate <= 0) {
	        continue;
	      }
      if (proof_cover_violated) {
        proof_cover_admitted = true;
      }
      if (direct_clause_cut_violated) {
        ++out.bc_stats.local_node_cuts_generated;
        local_proof_cuts->push_back(
            make_local_node_cut(std::move(direct_clause_cut)));
        ++local_rows_added;
      }

      std::vector<std::vector<BranchDomainLiteral>> learned_clauses;
      conflict_pool.add_with_binary_resolution(
          base_lp.vars, clause,
          /*max_resolvents=*/4, &learned_clauses);
      if (learned_clauses.empty()) {
        if (queue_hits > 0) {
          const int queue_pruned = node_queue.prune_conflicts(conflict_pool);
          if (queue_pruned > 0) {
            out.bc_stats.rc_conflict_queue_prunes +=
                static_cast<std::uint64_t>(queue_pruned);
          }
        }
        if (target_binary_impl_added) {
          note_global_domain_learning();
          ++learned;
          ++out.bc_stats.rc_target_proof_clauses_learned;
          ++out.bc_stats.rc_conflict_clauses_learned;
          ++out.bc_stats.rc_proof_conflict_clauses_learned;
          out.bc_stats.rc_proof_conflict_literals_before +=
              static_cast<std::uint64_t>(clause.size());
          out.bc_stats.rc_proof_conflict_literals_after +=
              static_cast<std::uint64_t>(clause.size());
        }
        if (published_reconvergence_certificate > 0) {
          ++learned;
          ++out.bc_stats.rc_target_proof_clauses_learned;
        }
        continue;
      }

      bool learned_any =
          target_binary_impl_added || published_reconvergence_certificate > 0;
      for (const auto& learned_clause : learned_clauses) {
        const bool impl_added =
            add_binary_implications_from_conflict_clause(base_lp.vars,
                                                         learned_clause,
                                                         implication_graph);
        if (impl_added) {
          ++out.bc_stats.rc_binary_implications_learned;
          note_global_domain_learning();
        }
        if (!strict_highs_lp_contract &&
            static_cast<int>(learned_clause.size()) <= max_cut_literals) {
          PoolCut conflict_cut;
          if (try_build_binary_conflict_cut(base_lp, learned_clause,
                                            conflict_cut,
                                            max_cut_literals) &&
              valid_pool_cut(conflict_cut, &x_relax,
                             "target_learned_conflict_cut") &&
              (static_cast<int>(learned_clause.size()) <= 2 ||
               sparse_dot(conflict_cut.coeff, x_relax) >
                   conflict_cut.rhs + 1e-7)) {
            new_cuts.push_back(std::move(conflict_cut));
            ++cut_count;
            ++out.bc_stats.rc_conflict_cuts_added;
          }
        }
        ++out.bc_stats.rc_conflict_clauses_learned;
        ++out.bc_stats.rc_proof_conflict_clauses_learned;
        out.bc_stats.rc_proof_conflict_literals_before +=
            static_cast<std::uint64_t>(branch_reasons.size() + 1);
        out.bc_stats.rc_proof_conflict_literals_after +=
            static_cast<std::uint64_t>(learned_clause.size());
        learned_any = true;
      }
      if (learned_any) ++learned;
      if (learned_any) ++out.bc_stats.rc_target_proof_clauses_learned;
    }
    return learned;
  };

  int rc_verified_conflict_lps_left =
      std::max(0, opt.reduced_cost_verified_conflict_max_lps);
  const bool large_stale_frontier_case = n >= 10000 || presolved_m >= 15000;
  int rc_fixing_resolve_lps_left = std::max(
      0, opt.reduced_cost_fixing_resolve_max_lps *
             (large_stale_frontier_case ? 4 : 1));
  int rc_proof_cut_resolve_lps_left =
      std::max(0, opt.reduced_cost_proof_cut_resolve_max_lps);
  const bool allow_certificate_domain_replay =
      opt.enable_reduced_cost_proof_conflict_minimization;
  const bool allow_native_proof_cut_rows =
      opt.enable_reduced_cost_proof_cut_resolve && !strict_highs_lp_contract;

  double last_queue_domain_replay_incumbent = kInf;
  std::uint64_t last_queue_domain_replay_epoch = 0;
  std::uint64_t last_queue_objective_replay_epoch = 0;
  int stale_queue_domain_replay_passes = 0;

	  auto local_implication_hash = [](int trigger, bool trigger_one, int implied,
	                                   bool implied_is_lb, double implied_value)
      -> std::size_t {
    std::size_t h = std::hash<int>{}(trigger);
    auto mix = [&](std::size_t v) {
      h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    };
    mix(std::hash<bool>{}(trigger_one));
    mix(std::hash<int>{}(implied));
    mix(std::hash<bool>{}(implied_is_lb));
    mix(std::hash<long long>{}(
        static_cast<long long>(std::llround(implied_value * 1e9))));
	    return h;
	  };

	  auto publish_conflict_clause_from_dual_proof =
	      [&](std::vector<BranchDomainLiteral> clause,
	          std::uint64_t source_literal_count,
	          const char* tag) -> int {
	    // Honor the wall-clock limit mid-node: this leaf drives the O(pool*L^2)
	    // binary-resolution learning and is called from several per-node loops.
	    // Also skip entirely once the incumbent is proven optimal (gap 0): the
	    // learned conflicts cannot change a certified-optimal outcome.
	    if (bc_time_limit_expired() || incumbent_proven_optimal_exact()) return 0;
	    canonicalize_branch_literals(clause);
	    if (clause.empty() ||
	        static_cast<int>(clause.size()) >
	            std::max(2, opt.reduced_cost_conflict_pool_max_literals)) {
	      return 0;
	    }
	    std::vector<std::vector<BranchDomainLiteral>> learned_clauses;
	    audit_conflict_publish_against_witness(
	        clause, tag != nullptr ? tag : "dual_proof_conflict");
	    const int added = conflict_pool.add_with_binary_resolution(
	        base_lp.vars, clause, /*max_resolvents=*/4, &learned_clauses);
	    if (added <= 0 || learned_clauses.empty()) return 0;

	    int published = 0;
	    const int max_cut_literals =
	        std::max(2, opt.reduced_cost_conflict_cut_max_literals);
	    for (const auto& learned_clause : learned_clauses) {
	      audit_conflict_publish_against_witness(
	          learned_clause,
	          tag != nullptr ? tag : "dual_proof_resolved_conflict");
	      const bool impl_added =
	          add_binary_implications_from_conflict_clause(
	              base_lp.vars, learned_clause, implication_graph);
	      if (impl_added) ++out.bc_stats.rc_binary_implications_learned;
	      if (!strict_highs_lp_contract &&
	          static_cast<int>(learned_clause.size()) <= max_cut_literals) {
	        PoolCut conflict_cut;
	        if (try_build_binary_conflict_cut(base_lp, learned_clause,
	                                          conflict_cut,
	                                          max_cut_literals) &&
	            valid_pool_cut(conflict_cut, nullptr,
	                           tag != nullptr ? tag
	                                          : "queue_dual_proof_conflict") &&
	            [&]() {
	              return add_cut_to_global_domain_pool(
	                  std::move(conflict_cut.coeff), conflict_cut.rhs,
	                  tag != nullptr ? tag : "queue_dual_proof_conflict",
	                  conflict_cut.domain_only,
	                  conflict_cut.validity_scope);
	            }()) {
	          ++out.bc_stats.rc_conflict_cuts_added;
	        }
	      }
	      ++out.bc_stats.rc_conflict_clauses_learned;
	      ++out.bc_stats.rc_proof_conflict_clauses_learned;
	      out.bc_stats.rc_proof_conflict_literals_before +=
	          source_literal_count == 0
	              ? static_cast<std::uint64_t>(clause.size())
	              : source_literal_count;
	      out.bc_stats.rc_proof_conflict_literals_after +=
	          static_cast<std::uint64_t>(learned_clause.size());
	      ++published;
	    }
	    if (published > 0) {
	      note_global_domain_learning(static_cast<std::uint64_t>(published));
	      out.bc_stats.termination_effective_admitted +=
	          static_cast<std::uint64_t>(published);
	    }
	    return published;
	  };

	  std::vector<BoundLiftingCertificate> global_bound_lifting_certificates;
	  publish_bound_lifting_certificate =
	      [&](const DomainReasonBound& rb,
	          const std::vector<BranchDomainLiteral>* extra_scope,
	          const std::vector<std::vector<BranchDomainLiteral>>*
	              reconvergence_clauses,
	          const char* tag) -> int {
	    // HiGHS' reconvergence cut stores the resolved frontier itself.  A
	    // native scoped gate may decide where the artifact is consumed, but it is
	    // not part of the published frontier unless the resolver selected it.
	    (void)extra_scope;
	    if (bc_time_limit_expired() || incumbent_proven_optimal_exact()) return 0;
	    if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
	        !std::isfinite(rb.bound.value) ||
	        !rb.has_source_conflict_literal ||
	        rb.source_conflict_literal.var_idx < 0 ||
	        rb.source_conflict_literal.var_idx >= n ||
	        !std::isfinite(rb.source_conflict_literal.value)) {
	      return 0;
	    }
	    std::vector<BranchDomainLiteral> frontier;
	    frontier.insert(frontier.end(), rb.reason.begin(), rb.reason.end());
	    canonicalize_branch_literals(frontier);
	    int integral_cols = 0;
	    for (const auto& var : base_lp.vars) {
	      if (is_integer_type(var)) ++integral_cols;
	    }
	    const int highs_reconvergence_limit =
	        std::max(32, (1000 + 3 * integral_cols) / 10);
	    const int max_bound_lift_literals =
	        std::max(std::max(2, opt.reduced_cost_conflict_pool_max_literals),
	                 highs_reconvergence_limit);
	    if (static_cast<int>(frontier.size() + 1) > max_bound_lift_literals) {
	      ++out.bc_stats.bound_lifting_certificates_rejected_audit;
	      return 0;
	    }
	    auto structural_certificate_ok =
	        [&](const std::vector<BranchDomainLiteral>& lits) -> bool {
	      std::vector<double> lower(static_cast<std::size_t>(n), -kInf);
	      std::vector<double> upper(static_cast<std::size_t>(n), kInf);
	      for (const auto& lit : lits) {
	        if (lit.var_idx < 0 || lit.var_idx >= n ||
	            !std::isfinite(lit.value)) {
	          return false;
	        }
	        if (lit.var_idx == rb.bound.var_idx) return false;
	        if (lit.is_lb) {
	          lower[static_cast<std::size_t>(lit.var_idx)] =
	              std::max(lower[static_cast<std::size_t>(lit.var_idx)],
	                       lit.value);
	        } else {
	          upper[static_cast<std::size_t>(lit.var_idx)] =
	              std::min(upper[static_cast<std::size_t>(lit.var_idx)],
	                       lit.value);
	        }
	        if (lower[static_cast<std::size_t>(lit.var_idx)] >
	            upper[static_cast<std::size_t>(lit.var_idx)] + opt.int_tol) {
	          return false;
	        }
	      }
	      return true;
	    };
	    const bool source_excludes_target =
	        rb.source_conflict_literal.var_idx == rb.bound.var_idx &&
	        rb.source_conflict_literal.is_lb != rb.bound.is_lb &&
	        (rb.bound.is_lb
	             ? rb.source_conflict_literal.value < rb.bound.value - 1e-9
	             : rb.source_conflict_literal.value > rb.bound.value + 1e-9);
          if (!rb.has_proof_activity_audit ||
              !std::isfinite(rb.proof_activity_margin) ||
              // SOUNDNESS: require a real violation margin exceeding the node
              // feasibility tolerance; a knife-edge margin (<= kNodeFeasibilityTol)
              // is within LP feasibility noise and yields an unsound certificate
              // that can exclude an LP-feasible optimum (MIPLIB3 gen). See
              // /memories/repo/mipsolvers-objcutoff-soundness-bug.md.
              !(rb.proof_activity_margin > kNodeFeasibilityTol) ||
              !source_excludes_target ||
              !structural_certificate_ok(frontier)) {
	      ++out.bc_stats.bound_lifting_certificates_rejected_audit;
	      return 0;
	    }

	    std::vector<BranchDomainLiteral> sig = frontier;
	    sig.push_back(rb.bound);
	    if (rb.has_source_conflict_literal) {
	      sig.push_back(rb.source_conflict_literal);
	    }
	    canonicalize_branch_literals(sig);
	    const std::size_t cert_hash = conflict_clause_hash(sig);
		    bool duplicate_certificate = false;
		    for (const auto& existing : global_bound_lifting_certificates) {
		      if (existing.hash != cert_hash ||
		          !branch_literal_lists_equal(existing.frontier_literals,
		                                      frontier) ||
		          !branch_literals_equal(existing.target_bound, rb.bound) ||
		          existing.has_source_conflict_literal !=
		              rb.has_source_conflict_literal ||
		          (rb.has_source_conflict_literal &&
		           !branch_literals_equal(existing.source_conflict_literal,
		                                  rb.source_conflict_literal))) {
		        continue;
		      }
		      duplicate_certificate = true;
		      break;
		    }
		    int published = 0;
		    if (!duplicate_certificate) {
		      BoundLiftingCertificate cert;
		      cert.frontier_literals = frontier;
		      cert.target_bound = rb.bound;
		      cert.source_conflict_literal = rb.source_conflict_literal;
	      cert.has_source_conflict_literal = rb.has_source_conflict_literal;
	      cert.hash = cert_hash;
	      cert.depth = rb.depth;
		      cert.has_proof_activity_audit = rb.has_proof_activity_audit;
		      cert.proof_activity_margin = rb.proof_activity_margin;
		      cert.proof_activity = rb.proof_activity;
		      cert.proof_required_activity = rb.proof_required_activity;
		      global_bound_lifting_certificates.push_back(std::move(cert));
		      note_global_domain_learning();
		      ++out.bc_stats.bound_lifting_certificates_published;
	      out.bc_stats.bound_lifting_certificate_activity_margin_min =
	          std::min(out.bc_stats.bound_lifting_certificate_activity_margin_min,
	                   rb.proof_activity_margin);
	      out.bc_stats.bound_lifting_certificate_activity_margin_max =
	          std::max(out.bc_stats.bound_lifting_certificate_activity_margin_max,
		                   rb.proof_activity_margin);
		      ++published;
		    }

	    std::vector<BranchDomainLiteral> reconv = frontier;
	    reconv.push_back(rb.source_conflict_literal);
	    published += publish_conflict_clause_from_dual_proof(
	        std::move(reconv),
	        static_cast<std::uint64_t>(frontier.size() + 1),
	        tag != nullptr ? tag : "bound_lifting_certificate");
	    if (reconvergence_clauses != nullptr) {
	      for (const auto& reconv : *reconvergence_clauses) {
	        std::vector<BranchDomainLiteral> scoped_reconv = reconv;
	        canonicalize_branch_literals(scoped_reconv);
	        published += publish_conflict_clause_from_dual_proof(
	            scoped_reconv,
	            static_cast<std::uint64_t>(scoped_reconv.size()),
	            tag != nullptr ? tag : "bound_lifting_reconvergence");
	      }
	    }
	    return published;
	  };

    auto record_dual_proof_resolution_failure =
        [&](DualProofResolutionStatus status) {
      if (dual_proof_resolution_missing_reason(status)) {
        ++out.bc_stats.resolution_missing_reason;
      } else if (dual_proof_resolution_scope_blocked(status)) {
        ++out.bc_stats.resolution_scope_blocked;
      } else {
        ++out.bc_stats.resolution_activity_failed;
      }
    };

    auto publish_local_trail_bound_lifting_certificates =
	      [&](const Node& node, const DualProofRow* proof,
	          const DualProofRow* priority_proof, const char* tag) -> int {
	    if (proof == nullptr || !proof->valid || proof->coeff.size() < n ||
	        proof->coeff.nonZeros() <= 0 || !std::isfinite(proof->rhs) ||
	        node.lb.size() < n || node.ub.size() < n ||
	        node.domain_reason_bounds.empty()) {
	      return 0;
	    }
	    int integral_cols = 0;
	    for (const auto& var : base_lp.vars) {
	      if (is_integer_type(var)) ++integral_cols;
	    }
	    const int highs_frontier_literal_limit =
	        std::max(32, (1000 + 3 * integral_cols) / 10);
	    const int max_literals = std::min(
	        std::max(2, opt.reduced_cost_conflict_pool_max_literals),
	        highs_frontier_literal_limit);
	    const Eigen::SparseVector<double>* priority_coeff =
	        priority_proof != nullptr && priority_proof->valid &&
	                priority_proof->coeff.size() >= n
	            ? &priority_proof->coeff
	            : nullptr;
	    int published = 0;
	    BranchLiteralListSet seen_targets;
	    for (const auto& rb : node.domain_reason_bounds) {
	      if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
	          !std::isfinite(rb.bound.value)) {
	        continue;
	      }
	      const bool source_excludes_target =
	          rb.has_source_conflict_literal &&
	          rb.source_conflict_literal.var_idx == rb.bound.var_idx &&
	          rb.source_conflict_literal.is_lb != rb.bound.is_lb &&
	          std::isfinite(rb.source_conflict_literal.value) &&
	          (rb.bound.is_lb
	               ? rb.source_conflict_literal.value < rb.bound.value - 1e-9
	               : rb.source_conflict_literal.value > rb.bound.value + 1e-9);
	      const bool first_class_proof_target =
	          rb.has_proof_activity_audit &&
	          std::isfinite(rb.proof_activity_margin) &&
	          std::isfinite(rb.proof_activity) &&
	          std::isfinite(rb.proof_required_activity) &&
	          rb.proof_activity_margin >= -std::max(1e-8, opt.lp_tol) &&
	          rb.has_source_conflict_literal &&
	          rb.source_conflict_literal.var_idx >= 0 &&
	          rb.source_conflict_literal.var_idx < n &&
	          source_excludes_target;
	      if (!first_class_proof_target) {
	        continue;
	      }
	      std::vector<BranchDomainLiteral> sig = rb.reason;
	      sig.push_back(rb.bound);
	      if (rb.has_source_conflict_literal) {
	        sig.push_back(rb.source_conflict_literal);
	      }
	      canonicalize_branch_literals(sig);
	      if (!seen_targets.insert(sig)) {
	        continue;
	      }
    	      DualProofTrailResolution resolution;
        ++out.bc_stats.resolved_target_attempts;
        const DualProofResolutionStatus resolve_status =
          resolve_dual_proof_target_bound_from_local_trail(
                base_lp.vars, root.lb, root.ub,
                node.x_relax.size() >= n ? &node.x_relax : nullptr,
                opt.int_tol, node.branch_reasons, node.domain_reason_bounds,
                node.local_domain_trail.read_ptr(),
                node.local_branch_positions.read_ptr(), rb,
                max_literals, proof->coeff, proof->rhs, resolution,
            priority_coeff);
        const bool resolved_target =
          dual_proof_resolution_success(resolve_status) && resolution.valid &&
          !resolution.cutoff_conflict;
        if (!resolved_target) {
          ++out.bc_stats.resolved_target_failed;
          record_dual_proof_resolution_failure(
            resolution.cutoff_conflict
              ? DualProofResolutionStatus::ScopeBlocked
              : resolve_status);
	        continue;
	      }
        ++out.bc_stats.resolved_target_success;
	      DomainReasonBound lifted = rb;
	      lifted.bound = resolution.has_proved_target_bound
	                         ? resolution.proved_target_bound
	                         : rb.bound;
	      lifted.reason = resolution.resolved_frontier;
      lifted.source_conflict_literal = resolution.flipped_target;
      lifted.has_source_conflict_literal = true;
      lifted.has_proof_activity_audit = true;
      lifted.proof_activity_margin = resolution.proof_margin;
      lifted.proof_activity = resolution.resolved_activity;
      lifted.proof_required_activity =
          resolution.resolved_activity - resolution.proof_margin;
      lifted.trail_pos = std::max(
	          lifted.trail_pos,
	          static_cast<int>(node.branch_reasons.size() +
	                           node.domain_reason_bounds.size()));
	      published += publish_bound_lifting_certificate(
	          lifted, nullptr, &resolution.reconvergence_clauses,
	          tag != nullptr ? tag : "local_trail_bound_lift");
	    }
	    return published;
	  };

    auto publish_local_trail_cutoff_conflict_certificates =
        [&](const Node& node, const DualProofRow* proof, double cutoff_rhs,
            const char* tag) -> int {
      if (proof == nullptr || !proof->valid || proof->coeff.size() < n ||
          proof->coeff.nonZeros() <= 0 || !std::isfinite(cutoff_rhs) ||
          node.lb.size() < n || node.ub.size() < n ||
          node.local_domain_trail.empty()) {
        return 0;
      }
      double node_min_activity = 0.0;
      for (Eigen::SparseVector<double>::InnerIterator it(proof->coeff); it;
           ++it) {
        const int j = static_cast<int>(it.index());
        const double a = it.value();
        if (j < 0 || j >= n || !std::isfinite(a)) return 0;
        if (a > 0.0) {
          if (!std::isfinite(node.lb[j])) return 0;
          node_min_activity += a * node.lb[j];
        } else if (a < 0.0) {
          if (!std::isfinite(node.ub[j])) return 0;
          node_min_activity += a * node.ub[j];
        }
      }
      const double activity_tol =
          std::max(1e-7, std::max(opt.lp_tol, opt.int_tol) *
                              std::max(10.0, std::abs(cutoff_rhs)));
      if (!std::isfinite(node_min_activity) ||
          node_min_activity <= cutoff_rhs + activity_tol) {
        return 0;
      }
      const int integral_cols_for_limit =
          static_cast<int>(std::count_if(
              base_lp.vars.begin(), base_lp.vars.end(),
              [](const VariableMeta& var) { return is_integer_type(var); }));
      const int proof_frontier_literal_limit = std::min(
          std::max(2, opt.reduced_cost_conflict_pool_max_literals),
          std::max(32, (1000 + 3 * integral_cols_for_limit) / 10));
      DualProofConflictResolution resolution;
      ++out.bc_stats.resolved_conflict_attempts;
      const DualProofResolutionStatus resolve_status =
          resolve_dual_proof_conflict_from_local_trail(
              base_lp.vars, root.lb, root.ub, node.lb, node.ub, opt.int_tol,
              opt.lp_tol, node.branch_reasons, node.domain_reason_bounds,
              node.local_domain_trail.read_ptr(),
              node.local_branch_positions.read_ptr(),
              proof->coeff, cutoff_rhs, proof_frontier_literal_limit,
              resolution);
      if (!dual_proof_resolution_success(resolve_status) ||
          !resolution.valid) {
        ++out.bc_stats.resolved_conflict_failed;
        record_dual_proof_resolution_failure(resolve_status);
        return 0;
      }
      ++out.bc_stats.resolved_conflict_success;
      int published = 0;
      for (const auto& clause : resolution.conflict_clauses) {
        published += publish_conflict_clause_from_dual_proof(
            clause,
            static_cast<std::uint64_t>(
                resolution.proof_frontier.empty()
                    ? clause.size()
                    : resolution.proof_frontier.size()),
            tag != nullptr ? tag : "local_trail_cutoff_conflict");
      }
      return published;
    };

	  auto publish_dual_proof_prune_conflict =
	      [&](const DualProofRow& proof, const Node& node, double node_min_activity,
	          double cutoff_rhs, bool proof_cutoff, bool incumbent_cutoff,
	          const char* source) -> int {
	    (void)proof_cutoff;
	    (void)incumbent_cutoff;
	    if (!opt.enable_reduced_cost_proof_conflict_minimization ||
	        !proof.valid || proof.coeff.size() < n || proof.coeff.nonZeros() <= 0 ||
	        !std::isfinite(cutoff_rhs) || node.lb.size() < n ||
	        node.ub.size() < n) {
	      return 0;
	    }
	    const double feastol = std::max(1e-9, opt.lp_tol);
	    const double eps = std::max(1e-9, opt.int_tol);
	    const double activity_tol =
	        std::max(1e-7, std::max(feastol, eps) *
	                            std::max(10.0, std::abs(cutoff_rhs)));
	    if (!std::isfinite(node_min_activity) ||
	        node_min_activity <= cutoff_rhs + activity_tol) {
	      return 0;
	    }

	    const int integral_cols_for_limit =
	        static_cast<int>(std::count_if(
	            base_lp.vars.begin(), base_lp.vars.end(),
	            [](const VariableMeta& var) { return is_integer_type(var); }));
	    const int proof_frontier_literal_limit = std::min(
	        std::max(2, opt.reduced_cost_conflict_pool_max_literals),
	        std::max(32, (1000 + 3 * integral_cols_for_limit) / 10));
	    DualProofConflictResolution row_resolution;
	    int row_published = 0;
      ++out.bc_stats.resolved_conflict_attempts;
  	    const DualProofResolutionStatus resolve_status =
        resolve_dual_proof_conflict_from_local_trail(
	            base_lp.vars, root.lb, root.ub, node.lb, node.ub, opt.int_tol,
	            opt.lp_tol, node.branch_reasons, node.domain_reason_bounds,
	            node.local_domain_trail.read_ptr(),
	            node.local_branch_positions.read_ptr(), proof.coeff,
          cutoff_rhs, proof_frontier_literal_limit, row_resolution);
      const bool resolved_conflict =
        dual_proof_resolution_success(resolve_status) && row_resolution.valid;
      if (resolved_conflict) {
        ++out.bc_stats.resolved_conflict_success;
	      for (const auto& clause : row_resolution.conflict_clauses) {
	        row_published += publish_conflict_clause_from_dual_proof(
	            clause, static_cast<std::uint64_t>(
	                        row_resolution.proof_frontier.empty()
	                            ? clause.size()
	                            : row_resolution.proof_frontier.size()),
	            source != nullptr ? source : "proof_row_local_trail");
	      }
      } else {
        ++out.bc_stats.resolved_conflict_failed;
	      record_dual_proof_resolution_failure(resolve_status);
	    }
	    if (row_published > 0) {
	      return row_published;
	    }
	    return 0;
	  };

	  auto dual_proof_min_activity_for_node =
	      [&](const DualProofRow& proof, const Node& node) -> double {
	    if (!proof.valid || proof.coeff.size() < n || node.lb.size() < n ||
	        node.ub.size() < n) {
	      return -kInf;
	    }
	    double activity = 0.0;
	    for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff); it; ++it) {
	      const int j = static_cast<int>(it.index());
	      const double a = it.value();
	      if (j < 0 || j >= n || !std::isfinite(a)) return -kInf;
	      if (a > 0.0) {
	        if (!std::isfinite(node.lb[j])) return -kInf;
	        activity += a * node.lb[j];
	      } else if (a < 0.0) {
	        if (!std::isfinite(node.ub[j])) return -kInf;
	        activity += a * node.ub[j];
	      }
	    }
	    return activity;
	  };

	  auto add_local_binary_implication =
	      [&](Node& node, int trigger, bool trigger_one, int implied,
          bool implied_is_lb, double implied_value) -> bool {
    if (trigger < 0 || trigger >= n || implied < 0 || implied >= n ||
        trigger == implied || !std::isfinite(implied_value)) {
      return false;
    }
    if (base_lp.vars[static_cast<std::size_t>(trigger)].type !=
        VarType::Binary) {
      return false;
    }
    const std::size_t h =
        local_implication_hash(trigger, trigger_one, implied, implied_is_lb,
                               implied_value);
    const LocalBinaryImplication candidate{
        trigger, trigger_one, implied, implied_is_lb, implied_value, h};
    for (const auto& arc : node.local_binary_implications) {
      if (arc.hash == h && local_binary_implications_equal(arc, candidate)) {
        return false;
      }
    }
    node.local_binary_implications.push_back(candidate);
    ++out.bc_stats.local_implications_learned;
    return true;
  };

  add_scoped_local_conflict_clause =
      [&](Node& node, const std::vector<BranchDomainLiteral>& learned_clause)
          -> bool {
    if (learned_clause.empty()) return false;
    std::vector<BranchDomainLiteral> local_clause;
    local_clause.reserve(learned_clause.size());
    for (const auto& lit : learned_clause) {
      bool scoped_active = false;
      for (const auto& br : node.branch_reasons) {
        if (branch_literal_stronger_or_equal(br, lit)) {
          scoped_active = true;
          break;
        }
      }
      if (!scoped_active) local_clause.push_back(lit);
    }
    canonicalize_branch_literals(local_clause);
    if (local_clause.empty() || static_cast<int>(local_clause.size()) > 32) {
      return false;
    }
    const std::size_t h = conflict_clause_hash(local_clause);
    for (const auto& clause : node.local_conflict_clauses) {
      if (conflict_clause_hash(clause) == h && clause.size() == local_clause.size()) {
        bool same = true;
        for (std::size_t i = 0; i < clause.size(); ++i) {
          if (clause[i].var_idx != local_clause[i].var_idx ||
              clause[i].is_lb != local_clause[i].is_lb ||
              std::abs(clause[i].value - local_clause[i].value) > 1e-9) {
            same = false;
            break;
          }
        }
        if (same) return false;
      }
    }
    node.local_conflict_clauses.push_back(std::move(local_clause));
	    ++out.bc_stats.local_conflict_clauses_learned;
	    return true;
	  };

	  auto strict_target_certificate_ok =
	      [&](const DomainReasonBound& rb) -> bool {
	    if (!strict_highs_lp_contract) return true;
	    if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
	        !std::isfinite(rb.bound.value) ||
	        !rb.has_source_conflict_literal ||
	        rb.source_conflict_literal.var_idx < 0 ||
	        rb.source_conflict_literal.var_idx >= n ||
	        !std::isfinite(rb.source_conflict_literal.value) ||
	        !rb.has_proof_activity_audit ||
	        !std::isfinite(rb.proof_activity_margin) ||
	        rb.proof_activity_margin < -std::max(1e-8, opt.lp_tol)) {
	      return false;
	    }
	    const bool source_excludes_target =
	        rb.source_conflict_literal.var_idx == rb.bound.var_idx &&
	        rb.source_conflict_literal.is_lb != rb.bound.is_lb &&
	        (rb.bound.is_lb
	             ? rb.source_conflict_literal.value < rb.bound.value - 1e-9
	             : rb.source_conflict_literal.value > rb.bound.value + 1e-9);
	    if (!source_excludes_target) return false;
	    std::vector<double> lower(static_cast<std::size_t>(n), -kInf);
	    std::vector<double> upper(static_cast<std::size_t>(n), kInf);
	    for (const auto& lit : rb.reason) {
	      if (lit.var_idx < 0 || lit.var_idx >= n ||
	          !std::isfinite(lit.value) || lit.var_idx == rb.bound.var_idx) {
	        return false;
	      }
	      if (lit.is_lb) {
	        lower[static_cast<std::size_t>(lit.var_idx)] =
	            std::max(lower[static_cast<std::size_t>(lit.var_idx)],
	                     lit.value);
	      } else {
	        upper[static_cast<std::size_t>(lit.var_idx)] =
	            std::min(upper[static_cast<std::size_t>(lit.var_idx)],
	                     lit.value);
	      }
	      if (lower[static_cast<std::size_t>(lit.var_idx)] >
	          upper[static_cast<std::size_t>(lit.var_idx)] +
	              std::max(1e-9, opt.int_tol)) {
	        return false;
	      }
	    }
	    return true;
	  };

		  add_scoped_conflict_object =
		      [&](Node& node, const std::vector<BranchDomainLiteral>& learned_clause,
		          const DomainReasonBound* target_bound)
		          -> bool {
	    if (learned_clause.empty() || learned_clause.size() > 16) return false;
	    if (strict_highs_lp_contract &&
	        (target_bound == nullptr ||
	         !strict_target_certificate_ok(*target_bound))) {
	      return false;
	    }
    std::vector<BranchDomainLiteral> scope;
    std::vector<BranchDomainLiteral> residual;
    scope.reserve(node.branch_reasons.size());
    residual.reserve(learned_clause.size());
    for (const auto& lit : learned_clause) {
      bool scoped_active = false;
      for (const auto& br : node.branch_reasons) {
        if (branch_literal_stronger_or_equal(br, lit)) {
          scoped_active = true;
          scope.push_back(lit);
          break;
        }
      }
      if (!scoped_active) residual.push_back(lit);
    }
    canonicalize_branch_literals(scope);
    canonicalize_branch_literals(residual);
    int integral_cols = 0;
    for (const auto& var : base_lp.vars) {
      if (is_integer_type(var)) ++integral_cols;
    }
    const int max_scoped_residual = std::max(
        1, (1000 + 3 * integral_cols) / 10);
	    int published_global = 0;
    if (target_bound == nullptr && scope.empty() && !residual.empty() &&
        static_cast<int>(residual.size()) <= max_scoped_residual) {
      published_global = publish_conflict_clause_from_dual_proof(
          residual, static_cast<std::uint64_t>(residual.size()),
          "scoped_conflict_global_reconvergence");
    }
    if ((!target_bound && residual.empty()) ||
        static_cast<int>(residual.size()) > max_scoped_residual ||
        scope.empty()) {
      return published_global > 0;
    }
    std::vector<BranchDomainLiteral> sig = scope;
    sig.insert(sig.end(), residual.begin(), residual.end());
    if (target_bound != nullptr) {
      sig.push_back(target_bound->bound);
      if (target_bound->has_source_conflict_literal) {
        sig.push_back(target_bound->source_conflict_literal);
      }
    }
    canonicalize_branch_literals(sig);
    ScopedConflictClause obj;
    obj.scope_literals = std::move(scope);
    obj.residual_literals = std::move(residual);
    obj.depth = node.depth;
	    if (target_bound != nullptr &&
	        target_bound->bound.var_idx >= 0 &&
	        target_bound->bound.var_idx < n &&
	        std::isfinite(target_bound->bound.value)) {
	      obj.has_target_bound = true;
	      obj.target_bound = target_bound->bound;
	      obj.source_conflict_literal = target_bound->source_conflict_literal;
	      obj.has_source_conflict_literal =
	          target_bound->has_source_conflict_literal;
	      obj.has_proof_activity_audit =
	          target_bound->has_proof_activity_audit;
	      obj.proof_activity_margin = target_bound->proof_activity_margin;
	      obj.proof_activity = target_bound->proof_activity;
	      obj.proof_required_activity =
	          target_bound->proof_required_activity;
	    }
	    // SOUNDNESS: a scoped conflict clause backed by a proof-activity
	    // certificate is a valid global infeasibility statement only if the
	    // forbidden literal makes the source row's min-activity exceed its rhs by
	    // MORE than the node feasibility tolerance kNodeFeasibilityTol (1e-6). A
	    // knife-edge margin (<= feas tol) is within LP feasibility noise, so the
	    // "violated" point is feasible by the solver's own tolerance and the
	    // clause can wrongly exclude an LP-feasible optimum (MIPLIB3 `gen`:
	    // margin 1.01e-7 pruned the true optimum). See
	    // /memories/repo/mipsolvers-objcutoff-soundness-bug.md.
	    if (obj.has_proof_activity_audit &&
	        !(obj.proof_activity_margin > kNodeFeasibilityTol)) {
	      return published_global > 0;
	    }
    const std::size_t h = conflict_clause_hash(sig);
    obj.hash = h;
    for (const auto& existing : node.scoped_conflict_clauses) {
      if (existing.hash == h && scoped_conflict_clauses_equal(existing, obj)) {
        return published_global > 0;
      }
    }
    node.scoped_conflict_clauses.push_back(
        std::move(obj));
    ++out.bc_stats.scoped_conflict_objects_learned;
    return true;
  };

				  add_scoped_target_bound_certificate =
				      [&](Node& node, const DomainReasonBound& target_bound) -> bool {
			    if (target_bound.bound.var_idx < 0 || target_bound.bound.var_idx >= n ||
			        !std::isfinite(target_bound.bound.value)) {
			      return false;
			    }
		    if (!strict_target_certificate_ok(target_bound)) {
		      return false;
		    }
		    std::vector<BranchDomainLiteral> scope;
		    std::vector<BranchDomainLiteral> residual = target_bound.reason;
		    canonicalize_branch_literals(scope);
		    canonicalize_branch_literals(residual);
		    int integral_cols = 0;
	    for (const auto& var : base_lp.vars) {
	      if (is_integer_type(var)) ++integral_cols;
	    }
    const int max_scoped_residual = std::max(
        1, (1000 + 3 * integral_cols) / 10);
    if (static_cast<int>(residual.size()) > max_scoped_residual) {
      return false;
    }
    std::vector<BranchDomainLiteral> sig = scope;
    sig.insert(sig.end(), residual.begin(), residual.end());
    sig.push_back(target_bound.bound);
    if (target_bound.has_source_conflict_literal) {
      sig.push_back(target_bound.source_conflict_literal);
    }
    canonicalize_branch_literals(sig);
    ScopedConflictClause obj;
    obj.scope_literals = std::move(scope);
    obj.residual_literals = std::move(residual);
    obj.depth = node.depth;
	    obj.has_target_bound = true;
	    obj.target_bound = target_bound.bound;
	    obj.source_conflict_literal = target_bound.source_conflict_literal;
	    obj.has_source_conflict_literal =
	        target_bound.has_source_conflict_literal;
	    obj.has_proof_activity_audit =
	        target_bound.has_proof_activity_audit;
	    obj.proof_activity_margin = target_bound.proof_activity_margin;
	    obj.proof_activity = target_bound.proof_activity;
	    obj.proof_required_activity =
	        target_bound.proof_required_activity;
	    // SOUNDNESS: reject knife-edge proof-activity certificates whose margin
	    // is within the node feasibility tolerance kNodeFeasibilityTol (1e-6).
	    // Such a "violation" is feasible by the solver's own tolerance, so the
	    // clause can wrongly exclude an LP-feasible optimum. See the note at the
	    // add_scoped_conflict_object gate above /
	    // /memories/repo/mipsolvers-objcutoff-soundness-bug.md.
	    if (obj.has_proof_activity_audit &&
	        !(obj.proof_activity_margin > kNodeFeasibilityTol)) {
	      return false;
	    }
    const std::size_t h = conflict_clause_hash(sig);
    obj.hash = h;
	    for (const auto& existing : node.scoped_conflict_clauses) {
	      if (existing.hash == h && scoped_conflict_clauses_equal(existing, obj)) {
	        return false;
	      }
	    }
	    node.scoped_conflict_clauses.push_back(std::move(obj));
	    ++out.bc_stats.scoped_conflict_objects_learned;
			    return true;
			  };

			  std::size_t root_domain_reason_certificate_cursor = 0;
			  auto publish_root_domain_closure_certificates =
			      [&](const char* tag) -> int {
			    if (!publish_bound_lifting_certificate ||
			        root.domain_reason_bounds.empty()) {
			      return 0;
			    }
			    if (root_domain_reason_certificate_cursor >
			        root.domain_reason_bounds.size()) {
			      root_domain_reason_certificate_cursor = 0;
			    }
			    int published = 0;
			    int candidates = 0;
			    int rejected = 0;
			    const std::size_t begin = root_domain_reason_certificate_cursor;
			    for (std::size_t i = begin; i < root.domain_reason_bounds.size(); ++i) {
			      const DomainReasonBound& rb = root.domain_reason_bounds[i];
			      const bool candidate =
			          rb.bound.var_idx >= 0 && rb.bound.var_idx < n &&
			          std::isfinite(rb.bound.value) &&
			          rb.has_source_conflict_literal &&
			          rb.source_conflict_literal.var_idx >= 0 &&
			          rb.source_conflict_literal.var_idx < n &&
			          std::isfinite(rb.source_conflict_literal.value) &&
			          rb.has_proof_activity_audit &&
			          std::isfinite(rb.proof_activity_margin);
			      if (!candidate) {
			        ++rejected;
			        continue;
			      }
			      ++candidates;
			      published += publish_bound_lifting_certificate(
			          rb, nullptr, nullptr,
			          tag != nullptr ? tag : "root_domain_closure");
			    }
			    root_domain_reason_certificate_cursor =
			        root.domain_reason_bounds.size();
			    if (published > 0) {
			      root.domain_learning_epoch = current_domain_learning_epoch();
			      root.objective_artifact_epoch = current_objective_artifact_epoch();
			      root.domain_closure_epoch = current_domain_closure_epoch();
			    }
			    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
			      fmt::print(stderr,
			                 "[B&C ROOT-CERT-PUBLISH] source={} range={}:{} "
			                 "candidates={} rejected={} published={} totalCert={}\n",
			                 tag != nullptr ? tag : "root_domain_closure",
			                 begin, root.domain_reason_bounds.size(), candidates,
			                 rejected, published,
			                 global_bound_lifting_certificates.size());
			    }
			    return published;
			  };

			  propagate_global_bound_lifting_certificates =
			      [&](Node& node, std::vector<BoundChangeInfo>& changes_out,
		          std::uint64_t* reasons_added_out) -> bool {
	    if (reasons_added_out != nullptr) *reasons_added_out = 0;
	    if (global_bound_lifting_certificates.empty() || node.lb.size() < n ||
	        node.ub.size() < n) {
	      return true;
	    }
	    const double tol = std::max(1e-9, opt.int_tol);
	    auto literal_active = [&](const BranchDomainLiteral& lit) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n ||
	          lit.var_idx >= node.lb.size() || lit.var_idx >= node.ub.size()) {
	        return false;
	      }
	      return lit.is_lb ? (node.lb[lit.var_idx] >= lit.value - tol)
	                       : (node.ub[lit.var_idx] <= lit.value + tol);
	    };
	    auto flip_clause_literal =
	        [&](const BranchDomainLiteral& lit,
	            BranchDomainLiteral& flipped) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n ||
	          !std::isfinite(lit.value)) {
	        return false;
	      }
	      const bool integer_var =
	          is_integer_type(base_lp.vars[static_cast<std::size_t>(lit.var_idx)]);
	      if (lit.is_lb) {
	        flipped = BranchDomainLiteral{
	            lit.var_idx,
	            integer_var ? std::ceil(lit.value - tol) - 1.0
	                        : lit.value - std::max(1e-7, 10.0 * tol),
	            false};
	      } else {
	        flipped = BranchDomainLiteral{
	            lit.var_idx,
	            integer_var ? std::floor(lit.value + tol) + 1.0
	                        : lit.value + std::max(1e-7, 10.0 * tol),
	            true};
	      }
	      return true;
	    };
	    auto duplicate_reason = [&](const DomainReasonBound& rb) -> bool {
	      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
	      for (const auto& existing : node.domain_reason_bounds) {
	        if (conflict_clause_hash(existing.reason) == reason_hash &&
	            domain_reason_bounds_equal(existing, rb, tol)) {
	          return true;
	        }
	      }
	      return false;
	    };

		    bool changed = false;
		    bool changed_this_pass = true;
		    std::uint64_t reasons_added = 0;
		    const std::size_t max_closure_passes =
		        global_bound_lifting_certificates.size() + 1;
		    for (std::size_t pass = 0;
		         pass < max_closure_passes && changed_this_pass; ++pass) {
		      changed_this_pass = false;
		      for (const auto& cert : global_bound_lifting_certificates) {
		        if (cert.target_bound.var_idx < 0 ||
		            cert.target_bound.var_idx >= n ||
		            !std::isfinite(cert.target_bound.value)) {
		          continue;
		        }
		        std::vector<BranchDomainLiteral> clause =
		            cert.frontier_literals;
		        BranchDomainLiteral source_lit = cert.source_conflict_literal;
		        if (!cert.has_source_conflict_literal ||
		            source_lit.var_idx < 0 || source_lit.var_idx >= n ||
		            !std::isfinite(source_lit.value)) {
		          if (!flip_clause_literal(cert.target_bound, source_lit)) {
		            continue;
		          }
		        }
		        clause.push_back(source_lit);

		        int missing_idx = -1;
		        int missing_count = 0;
		        std::vector<BranchDomainLiteral> reason;
		        reason.reserve(clause.size());
		        for (int ci = 0; ci < static_cast<int>(clause.size()); ++ci) {
		          const auto& lit = clause[static_cast<std::size_t>(ci)];
		          if (literal_active(lit)) {
		            reason.push_back(lit);
		          } else {
		            missing_idx = ci;
		            if (++missing_count > 1) break;
		          }
		        }
		        if (missing_count == 0) {
		          ++out.bc_stats.scoped_conflict_prunes;
		          return false;
		        }
		        if (missing_count != 1 || missing_idx < 0) continue;

			        const BranchDomainLiteral& missing =
			            clause[static_cast<std::size_t>(missing_idx)];
			        BranchDomainLiteral implied = cert.target_bound;
			        const bool missing_is_source =
			            missing.var_idx == source_lit.var_idx &&
			            missing.is_lb == source_lit.is_lb &&
			            std::abs(missing.value - source_lit.value) <= tol;
			        if (!missing_is_source &&
			            !flip_clause_literal(missing, implied)) {
			          continue;
			        }
		        if (implied.var_idx < 0 || implied.var_idx >= n ||
		            !std::isfinite(implied.value)) {
		          continue;
		        }
		        canonicalize_branch_literals(reason);

		        DomainReasonBound rb;
		        rb.bound = implied;
		        rb.reason = reason;
		        rb.trail_pos = static_cast<int>(
		            node.branch_reasons.size() + node.domain_reason_bounds.size());
			        rb.depth = cert.depth;
			        rb.source_conflict_literal = missing;
			        rb.has_source_conflict_literal = true;
			        rb.has_proof_activity_audit =
			            cert.has_proof_activity_audit;
			        rb.proof_activity_margin = cert.proof_activity_margin;
			        rb.proof_activity = cert.proof_activity;
			        rb.proof_required_activity =
			            cert.proof_required_activity;

		        const int j = implied.var_idx;
		        bool bound_changed = false;
		        if (implied.is_lb) {
		          if (implied.value > node.ub[j] + tol) {
		            ++out.bc_stats.scoped_conflict_prunes;
		            return false;
		          }
		          if (implied.value > node.lb[j] + tol) {
		            const double old = node.lb[j];
		            node.lb[j] = implied.value;
		            changes_out.push_back(
		                {j, implied.value - old, true, old, implied.value});
		            ++out.bc_stats.scoped_conflict_tightenings;
		            ++out.bc_stats.certificate_domain_fixings;
		            bound_changed = true;
		            changed = true;
		          }
		        } else {
		          if (implied.value < node.lb[j] - tol) {
		            ++out.bc_stats.scoped_conflict_prunes;
		            return false;
		          }
		          if (implied.value < node.ub[j] - tol) {
		            const double old = node.ub[j];
		            node.ub[j] = implied.value;
		            changes_out.push_back(
		                {j, implied.value - old, false, old, implied.value});
		            ++out.bc_stats.scoped_conflict_tightenings;
		            ++out.bc_stats.certificate_domain_fixings;
		            bound_changed = true;
		            changed = true;
		          }
		        }
			        bool reason_added = false;
			        if (bound_changed && !duplicate_reason(rb)) {
			          append_domain_reason_bound(node, std::move(rb), n, &root.lb,
			                                     &root.ub);
			          ++reasons_added;
			          reason_added = true;
			        }
		        if (bound_changed || reason_added) changed_this_pass = true;
		      }
		    }
	    if (reasons_added_out != nullptr) {
	      *reasons_added_out = reasons_added;
	    }
		    if (changed) {
		      node.lp_refresh_needed = true;
		      node.domain_learning_epoch = current_domain_learning_epoch();
		      node.objective_artifact_epoch = current_objective_artifact_epoch();
		      node.domain_closure_epoch = current_domain_closure_epoch();
		    }
	    return bounds_consistent(node.lb, node.ub);
	  };

				  auto propagate_scoped_conflict_objects =
					      [&](Node& node, std::vector<BoundChangeInfo>& changes_out) -> bool {
			    if (node.scoped_conflict_clauses.empty()) return true;
			    const double tol = std::max(1e-9, opt.int_tol);
	    auto literal_active = [&](const BranchDomainLiteral& lit) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n) return false;
	      return lit.is_lb ? (node.lb[lit.var_idx] >= lit.value - tol)
	                       : (node.ub[lit.var_idx] <= lit.value + tol);
	    };
	    auto flip_clause_literal =
	        [&](const BranchDomainLiteral& lit,
	            BranchDomainLiteral& flipped) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n ||
	          !std::isfinite(lit.value)) {
	        return false;
	      }
	      const bool integer_var =
	          is_integer_type(base_lp.vars[static_cast<std::size_t>(lit.var_idx)]);
	      if (lit.is_lb) {
	        flipped = BranchDomainLiteral{
	            lit.var_idx,
	            integer_var ? std::ceil(lit.value - tol) - 1.0
	                        : lit.value - std::max(1e-7, 10.0 * tol),
	            false};
	      } else {
	        flipped = BranchDomainLiteral{
	            lit.var_idx,
	            integer_var ? std::floor(lit.value + tol) + 1.0
	                        : lit.value + std::max(1e-7, 10.0 * tol),
	            true};
	      }
	      return true;
	    };
		    auto remember_scoped_reason_bound =
		        [&](const BranchDomainLiteral& bound,
		            std::vector<BranchDomainLiteral> reason,
		            const std::vector<BranchDomainLiteral>& scope_literals,
		            const BranchDomainLiteral& source_conflict_literal,
		            bool has_source_conflict_literal,
		            int depth,
		            bool has_proof_activity_audit,
		            double proof_activity_margin,
		            double proof_activity,
		            double proof_required_activity) {
	      (void)scope_literals;
	      if (bound.var_idx < 0 || bound.var_idx >= n ||
	          !std::isfinite(bound.value)) {
	        return;
	      }
	      canonicalize_branch_literals(reason);
	      DomainReasonBound rb;
	      rb.bound = bound;
	      rb.reason = std::move(reason);
	      rb.trail_pos = static_cast<int>(node.branch_reasons.size() +
	                                      node.domain_reason_bounds.size());
		      rb.depth = depth;
		      rb.source_conflict_literal = source_conflict_literal;
		      rb.has_source_conflict_literal = has_source_conflict_literal;
		      rb.has_proof_activity_audit = has_proof_activity_audit;
		      rb.proof_activity_margin = proof_activity_margin;
		      rb.proof_activity = proof_activity;
		      rb.proof_required_activity = proof_required_activity;
			      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
	      for (const auto& existing : node.domain_reason_bounds) {
	        if (conflict_clause_hash(existing.reason) == reason_hash &&
	            domain_reason_bounds_equal(existing, rb, tol)) {
          return;
        }
	      }
	      append_domain_reason_bound(node, std::move(rb), n, &root.lb, &root.ub);
	    };
		    auto remember_scoped_target_reason =
		        [&](const ScopedConflictClause& obj) {
	      if (!obj.has_target_bound) return;
	      remember_scoped_reason_bound(obj.target_bound, obj.residual_literals,
	                                   obj.scope_literals,
	                                   obj.source_conflict_literal,
	                                   obj.has_source_conflict_literal,
	                                   obj.depth,
	                                   obj.has_proof_activity_audit,
	                                   obj.proof_activity_margin,
	                                   obj.proof_activity,
	                                   obj.proof_required_activity);
		    };
		    bool changed = true;
		    while (changed) {
		      changed = false;
		      for (const auto& obj : node.scoped_conflict_clauses) {
		        bool scope_active = true;
	        for (const auto& lit : obj.scope_literals) {
	          if (!literal_active(lit)) {
	            scope_active = false;
	            break;
	          }
	        }
		        if (!scope_active) continue;
		        ++out.bc_stats.scoped_conflict_objects_active;
	
	        if (obj.has_target_bound) {
	          BranchDomainLiteral source_lit = obj.source_conflict_literal;
	          if (!obj.has_source_conflict_literal ||
	              source_lit.var_idx < 0 || source_lit.var_idx >= n ||
	              !std::isfinite(source_lit.value)) {
	            if (!flip_clause_literal(obj.target_bound, source_lit)) {
	              continue;
	            }
	          }

	          bool frontier_active = true;
	          std::vector<BranchDomainLiteral> frontier_reason;
	          frontier_reason.reserve(obj.residual_literals.size());
	          for (const auto& lit : obj.residual_literals) {
	            if (!literal_active(lit)) {
	              frontier_active = false;
	              break;
	            }
	            frontier_reason.push_back(lit);
	          }
	          if (frontier_active) {
	            const int j = obj.target_bound.var_idx;
	            if (j < 0 || j >= n ||
	                !std::isfinite(obj.target_bound.value)) {
	              continue;
	            }
	            bool target_changed = false;
	            if (obj.target_bound.is_lb) {
	              if (obj.target_bound.value > node.ub[j] + tol) {
		                remember_scoped_reason_bound(
		                    obj.target_bound, std::move(frontier_reason),
		                    obj.scope_literals,
		                    source_lit, /*has_source_conflict_literal=*/true,
		                    obj.depth, obj.has_proof_activity_audit,
		                    obj.proof_activity_margin, obj.proof_activity,
		                    obj.proof_required_activity);
	                ++out.bc_stats.scoped_conflict_prunes;
	                return false;
	              }
	              if (obj.target_bound.value > node.lb[j] + tol) {
	                const double old = node.lb[j];
	                node.lb[j] = obj.target_bound.value;
	                changes_out.push_back(
	                    {j, obj.target_bound.value - old, true});
	                ++out.bc_stats.scoped_conflict_tightenings;
	                target_changed = true;
	                changed = true;
	              }
	            } else {
	              if (obj.target_bound.value < node.lb[j] - tol) {
		                remember_scoped_reason_bound(
		                    obj.target_bound, std::move(frontier_reason),
		                    obj.scope_literals,
		                    source_lit, /*has_source_conflict_literal=*/true,
		                    obj.depth, obj.has_proof_activity_audit,
		                    obj.proof_activity_margin, obj.proof_activity,
		                    obj.proof_required_activity);
	                ++out.bc_stats.scoped_conflict_prunes;
	                return false;
	              }
	              if (obj.target_bound.value < node.ub[j] - tol) {
	                const double old = node.ub[j];
	                node.ub[j] = obj.target_bound.value;
	                changes_out.push_back(
	                    {j, obj.target_bound.value - old, false});
	                ++out.bc_stats.scoped_conflict_tightenings;
	                target_changed = true;
	                changed = true;
	              }
	            }
		            if (target_changed) {
			              remember_scoped_reason_bound(
			                  obj.target_bound, std::move(frontier_reason),
			                  obj.scope_literals,
			                  source_lit,
			                  /*has_source_conflict_literal=*/true, obj.depth,
			                  obj.has_proof_activity_audit,
			                  obj.proof_activity_margin, obj.proof_activity,
			                  obj.proof_required_activity);
		            }
		            if (target_changed) continue;
	          }

	          std::vector<BranchDomainLiteral> clause = obj.residual_literals;
	          clause.push_back(source_lit);
	          int missing_idx = -1;
	          int missing_count = 0;
	          std::vector<BranchDomainLiteral> reason;
	          reason.reserve(clause.size());
	          for (int ci = 0; ci < static_cast<int>(clause.size()); ++ci) {
	            const auto& lit = clause[static_cast<std::size_t>(ci)];
	            if (literal_active(lit)) {
	              reason.push_back(lit);
	            } else {
	              missing_idx = ci;
	              if (++missing_count > 1) break;
	            }
	          }
	          if (missing_count == 0) {
	            ++out.bc_stats.scoped_conflict_prunes;
	            return false;
	          }
	          if (missing_count == 1 && missing_idx >= 0) {
		            const BranchDomainLiteral& missing =
		                clause[static_cast<std::size_t>(missing_idx)];
		            BranchDomainLiteral implied = obj.target_bound;
		            const bool missing_is_source =
		                missing.var_idx == source_lit.var_idx &&
		                missing.is_lb == source_lit.is_lb &&
		                std::abs(missing.value - source_lit.value) <= tol;
		            if (!missing_is_source &&
		                !flip_clause_literal(missing, implied)) {
		              continue;
		            }
	            const int j = implied.var_idx;
	            if (j < 0 || j >= n || !std::isfinite(implied.value)) {
	              continue;
	            }
	            bool target_changed = false;
	            if (implied.is_lb) {
	              if (implied.value > node.ub[j] + tol) {
		                remember_scoped_reason_bound(
		                    implied, reason, obj.scope_literals, missing,
		                    /*has_source_conflict_literal=*/true, obj.depth,
		                    obj.has_proof_activity_audit,
		                    obj.proof_activity_margin, obj.proof_activity,
		                    obj.proof_required_activity);
	                ++out.bc_stats.scoped_conflict_prunes;
	                return false;
	              }
	              if (implied.value > node.lb[j] + tol) {
	                const double old = node.lb[j];
	                node.lb[j] = implied.value;
	                changes_out.push_back(
	                    {j, implied.value - old, true, old, implied.value});
	                ++out.bc_stats.scoped_conflict_tightenings;
	                target_changed = true;
	                changed = true;
	              }
	            } else {
	              if (implied.value < node.lb[j] - tol) {
		                remember_scoped_reason_bound(
		                    implied, reason, obj.scope_literals, missing,
		                    /*has_source_conflict_literal=*/true, obj.depth,
		                    obj.has_proof_activity_audit,
		                    obj.proof_activity_margin, obj.proof_activity,
		                    obj.proof_required_activity);
	                ++out.bc_stats.scoped_conflict_prunes;
	                return false;
	              }
	              if (implied.value < node.ub[j] - tol) {
	                const double old = node.ub[j];
	                node.ub[j] = implied.value;
	                changes_out.push_back(
	                    {j, implied.value - old, false, old, implied.value});
	                ++out.bc_stats.scoped_conflict_tightenings;
	                target_changed = true;
	                changed = true;
	              }
	            }
		            if (target_changed) {
			              remember_scoped_reason_bound(
			                  implied, std::move(reason), obj.scope_literals, missing,
			                  /*has_source_conflict_literal=*/true, obj.depth,
			                  obj.has_proof_activity_audit,
			                  obj.proof_activity_margin, obj.proof_activity,
			                  obj.proof_required_activity);
		            }
	            if (target_changed) continue;
	          }
	          continue;
	        }

	        bool frontier_active = true;
	        for (const auto& lit : obj.residual_literals) {
	          if (!literal_active(lit)) {
	            frontier_active = false;
	            break;
          }
        }
        if (frontier_active && obj.has_target_bound) {
          bool target_changed = false;
          const int j = obj.target_bound.var_idx;
          if (j < 0 || j >= n || !std::isfinite(obj.target_bound.value)) {
            continue;
          }
          if (obj.target_bound.is_lb) {
            if (obj.target_bound.value > node.ub[j] + tol) {
              remember_scoped_target_reason(obj);
              ++out.bc_stats.scoped_conflict_prunes;
              return false;
            }
            if (obj.target_bound.value > node.lb[j] + tol) {
              const double old = node.lb[j];
              node.lb[j] = obj.target_bound.value;
              changes_out.push_back(
                  {j, obj.target_bound.value - old, true, old,
                   obj.target_bound.value});
	              ++out.bc_stats.scoped_conflict_tightenings;
	              target_changed = true;
	              changed = true;
	            }
          } else {
            if (obj.target_bound.value < node.lb[j] - tol) {
              remember_scoped_target_reason(obj);
              ++out.bc_stats.scoped_conflict_prunes;
              return false;
            }
            if (obj.target_bound.value < node.ub[j] - tol) {
              const double old = node.ub[j];
              node.ub[j] = obj.target_bound.value;
              changes_out.push_back(
                  {j, obj.target_bound.value - old, false, old,
                   obj.target_bound.value});
	              ++out.bc_stats.scoped_conflict_tightenings;
	              target_changed = true;
	              changed = true;
	            }
          }
          if (target_changed) continue;
        }

	        int unsatisfied_count = 0;
	        const BranchDomainLiteral* remaining = nullptr;
	        for (const auto& lit : obj.residual_literals) {
	          if (lit.var_idx < 0 || lit.var_idx >= n) {
	            unsatisfied_count = 2;
	            break;
	          }
	          if (!literal_active(lit)) {
	            ++unsatisfied_count;
	            remaining = &lit;
	            if (unsatisfied_count > 1) break;
	          }
	        }
        if (unsatisfied_count == 0) {
          if (obj.has_target_bound) continue;
          ++out.bc_stats.scoped_conflict_prunes;
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
            ++out.bc_stats.scoped_conflict_prunes;
            return false;
          }
          if (new_ub < node.ub[j] - tol) {
            const double old = node.ub[j];
            node.ub[j] = new_ub;
            changes_out.push_back({j, new_ub - old, false, old, new_ub});
            ++out.bc_stats.scoped_conflict_tightenings;
            changed = true;
          }
        } else {
          const double new_lb =
              integer_var ? std::floor(remaining->value + tol) + 1.0
                          : remaining->value + 1e-7;
          if (new_lb > node.ub[j] + tol) {
            ++out.bc_stats.scoped_conflict_prunes;
            return false;
          }
          if (new_lb > node.lb[j] + tol) {
            const double old = node.lb[j];
            node.lb[j] = new_lb;
            changes_out.push_back({j, new_lb - old, true, old, new_lb});
            ++out.bc_stats.scoped_conflict_tightenings;
            changed = true;
          }
        }
      }
    }
	    return bounds_consistent(node.lb, node.ub);
	  };

	  auto explain_active_scoped_reason_bounds =
	      [&](Node& node, std::vector<BoundChangeInfo>& changes_out) -> bool {
	    if (node.scoped_conflict_clauses.empty() ||
	        node.domain_reason_bounds.empty()) {
	      return true;
	    }
	    const double tol = std::max(1e-9, opt.int_tol);
	    auto literal_active = [&](const BranchDomainLiteral& lit) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n) return false;
	      return lit.is_lb ? (node.lb[lit.var_idx] >= lit.value - tol)
	                       : (node.ub[lit.var_idx] <= lit.value + tol);
	    };
	    auto reason_active =
	        [&](const std::vector<BranchDomainLiteral>& reason) -> bool {
	      if (reason.empty()) return false;
	      for (const auto& lit : reason) {
	        if (!literal_active(lit)) return false;
	      }
	      return true;
	    };
			    auto apply_reason_bound = [&](const DomainReasonBound& rb) -> bool {
		      const int j = rb.bound.var_idx;
		      if (j < 0 || j >= n || !std::isfinite(rb.bound.value)) {
		        return true;
		      }
		      if (rb.bound.is_lb) {
	        if (rb.bound.value > node.ub[j] + tol) {
	          ++out.bc_stats.scoped_conflict_prunes;
	          return false;
	        }
	        if (rb.bound.value > node.lb[j] + tol) {
	          const double old = node.lb[j];
	          node.lb[j] = rb.bound.value;
	          changes_out.push_back(
	              {j, rb.bound.value - old, true, old, rb.bound.value});
	          ++out.bc_stats.scoped_conflict_tightenings;
	          if (rb.reason.size() == 1) {
	            const auto& trig = rb.reason.front();
	            const bool trig_one =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                trig.is_lb && trig.value >= 1.0 - tol;
	            const bool trig_zero =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                !trig.is_lb && trig.value <= tol;
	            if (trig_one || trig_zero) {
	              add_local_binary_implication(node, trig.var_idx, trig_one, j,
	                                           true, rb.bound.value);
	            }
	          }
	        }
	      } else {
	        if (rb.bound.value < node.lb[j] - tol) {
	          ++out.bc_stats.scoped_conflict_prunes;
	          return false;
	        }
	        if (rb.bound.value < node.ub[j] - tol) {
	          const double old = node.ub[j];
	          node.ub[j] = rb.bound.value;
	          changes_out.push_back(
	              {j, rb.bound.value - old, false, old, rb.bound.value});
	          ++out.bc_stats.scoped_conflict_tightenings;
	          if (rb.reason.size() == 1) {
	            const auto& trig = rb.reason.front();
	            const bool trig_one =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                trig.is_lb && trig.value >= 1.0 - tol;
	            const bool trig_zero =
	                trig.var_idx >= 0 && trig.var_idx < n &&
	                base_lp.vars[static_cast<std::size_t>(trig.var_idx)].type ==
	                    VarType::Binary &&
	                !trig.is_lb && trig.value <= tol;
	            if (trig_one || trig_zero) {
	              add_local_binary_implication(node, trig.var_idx, trig_one, j,
	                                           false, rb.bound.value);
	            }
	          }
	        }
	      }
		      return bounds_consistent(node.lb, node.ub);
		    };
		    auto duplicate_reason_bound = [&](const DomainReasonBound& rb) {
		      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
		      for (const auto& existing : node.domain_reason_bounds) {
	        if (conflict_clause_hash(existing.reason) == reason_hash &&
	            domain_reason_bounds_equal(existing, rb, tol)) {
		          return true;
		        }
		      }
		      return false;
		    };

	    bool changed = true;
	    while (changed) {
	      changed = false;
	      const std::size_t objects_this_pass = node.scoped_conflict_clauses.size();
	      for (std::size_t obj_idx = 0; obj_idx < objects_this_pass; ++obj_idx) {
	        const auto& obj = node.scoped_conflict_clauses[obj_idx];
	        bool scope_active = true;
	        for (const auto& lit : obj.scope_literals) {
	          if (!literal_active(lit)) {
	            scope_active = false;
	            break;
	          }
	        }
	        if (!scope_active) continue;
	        for (const auto& residual_lit : obj.residual_literals) {
	          if (literal_active(residual_lit)) continue;
	          const DomainReasonBound* best = nullptr;
	          for (const auto& rb : node.domain_reason_bounds) {
	            if (rb.reason.empty() ||
	                !branch_literal_stronger_or_equal(rb.bound, residual_lit,
	                                                  tol) ||
	                !reason_active(rb.reason)) {
	              continue;
	            }
	            if (best == nullptr ||
	                rb.reason.size() < best->reason.size() ||
	                (rb.reason.size() == best->reason.size() &&
	                 rb.trail_pos > best->trail_pos)) {
	              best = &rb;
	            }
	          }
	          if (best == nullptr) continue;
	          std::vector<BranchDomainLiteral> resolved_clause = obj.scope_literals;
	          std::vector<BranchDomainLiteral> resolved_residual;
	          resolved_residual.reserve(obj.residual_literals.size() +
	                                    best->reason.size());
	          for (const auto& keep : obj.residual_literals) {
	            if (!branch_literal_stronger_or_equal(keep, residual_lit, tol) ||
	                !branch_literal_stronger_or_equal(residual_lit, keep, tol)) {
	              resolved_clause.push_back(keep);
	              resolved_residual.push_back(keep);
	            }
	          }
	          resolved_clause.insert(resolved_clause.end(), best->reason.begin(),
	                                 best->reason.end());
	          resolved_residual.insert(resolved_residual.end(),
	                                   best->reason.begin(),
	                                   best->reason.end());
	          canonicalize_branch_literals(resolved_clause);
	          canonicalize_branch_literals(resolved_residual);
		          if (obj.has_target_bound) {
		            DomainReasonBound resolved_target;
		            resolved_target.bound = obj.target_bound;
		            resolved_target.reason = resolved_clause;
		            resolved_target.trail_pos =
		                static_cast<int>(node.branch_reasons.size() +
		                                 node.domain_reason_bounds.size());
		            resolved_target.depth = obj.depth;
	            resolved_target.source_conflict_literal =
	                obj.has_source_conflict_literal
	                    ? obj.source_conflict_literal
		                    : BranchDomainLiteral{};
			            resolved_target.has_source_conflict_literal =
			                obj.has_source_conflict_literal;
			            resolved_target.has_proof_activity_audit =
			                obj.has_proof_activity_audit;
			            resolved_target.proof_activity_margin =
			                obj.proof_activity_margin;
			            resolved_target.proof_activity = obj.proof_activity;
			            resolved_target.proof_required_activity =
			                obj.proof_required_activity;
			            const bool resolved_frontier_active =
			                reason_active(resolved_target.reason);
			            if (resolved_frontier_active) {
			              const double old_lb =
			                  resolved_target.bound.var_idx >= 0
			                      ? node.lb[resolved_target.bound.var_idx]
			                      : 0.0;
		              const double old_ub =
		                  resolved_target.bound.var_idx >= 0
		                      ? node.ub[resolved_target.bound.var_idx]
		                      : 0.0;
		              if (!apply_reason_bound(resolved_target)) return false;
		              const bool target_tightened =
		                  resolved_target.bound.var_idx >= 0 &&
			                  (resolved_target.bound.is_lb
			                       ? node.lb[resolved_target.bound.var_idx] >
			                             old_lb + tol
			                       : node.ub[resolved_target.bound.var_idx] <
			                             old_ub - tol);
			              if (target_tightened) {
			                if (!duplicate_reason_bound(resolved_target)) {
			                  append_domain_reason_bound(node, resolved_target, n,
			                                             &root.lb, &root.ub);
			                }
			                changed = true;
			              }
			            }
		          } else if (add_scoped_conflict_object(node, resolved_clause,
		                                                nullptr)) {
		            changed = true;
		          }
	          const double old_lb = node.lb[best->bound.var_idx];
	          const double old_ub = node.ub[best->bound.var_idx];
	          if (!apply_reason_bound(*best)) return false;
	          const bool tightened = best->bound.is_lb
	              ? (node.lb[best->bound.var_idx] > old_lb + tol)
	              : (node.ub[best->bound.var_idx] < old_ub - tol);
	          if (tightened) {
	            changed = true;
	            if (!propagate_scoped_conflict_objects(node, changes_out)) {
	              return false;
	            }
	          }
	        }
	      }
	    }
	    return true;
	  };

		  auto separate_active_scoped_conflict_rows =
		      [&](Node& node, const Eigen::VectorXd& x_relax,
		          std::vector<PoolCut>& local_proof_cuts, int max_rows) -> int {
	    if (max_rows <= 0 || x_relax.size() != n ||
	        node.scoped_conflict_clauses.empty()) {
	      return 0;
	    }
	    const double tol = std::max(1e-9, opt.int_tol);
	    auto literal_active = [&](const BranchDomainLiteral& lit) -> bool {
	      if (lit.var_idx < 0 || lit.var_idx >= n) return false;
	      return lit.is_lb ? (node.lb[lit.var_idx] >= lit.value - tol)
	                       : (node.ub[lit.var_idx] <= lit.value + tol);
	    };
	    int rows = 0;
	    for (const auto& obj : node.scoped_conflict_clauses) {
	      bool scope_active = true;
	      for (const auto& lit : obj.scope_literals) {
	        if (!literal_active(lit)) {
	          scope_active = false;
	          break;
	        }
	      }
	      if (!scope_active || obj.residual_literals.empty() ||
	          static_cast<int>(obj.residual_literals.size()) >
	              std::max(2, opt.reduced_cost_conflict_cut_max_literals)) {
	        continue;
	      }
	      PoolCut local_cut;
	      if (!try_build_binary_conflict_cut(
	              base_lp, obj.residual_literals, local_cut,
	              std::max(2, opt.reduced_cost_conflict_cut_max_literals))) {
	        continue;
	      }
	      const double viol = sparse_dot(local_cut.coeff, x_relax) - local_cut.rhs;
	      if (viol <= 1e-7) continue;
	      const std::size_t h = sparse_cut_hash(local_cut.coeff);
	      if (contains_identical_local_cut(node, local_proof_cuts,
	                                       local_cut.coeff, local_cut.rhs)) {

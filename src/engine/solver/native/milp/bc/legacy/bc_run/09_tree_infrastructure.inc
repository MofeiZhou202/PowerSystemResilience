// @file bc_run/09_tree_infrastructure.inc
// BCSolveState::run() segment 9/14: tree/node infrastructure: NodeQueue, global lower bound, node LP/StandardForm builders.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.

	      using ::mipsolvers::engine::detail::BCDomainProbeDelta;
	      using ::mipsolvers::engine::detail::BCDomainProbeOutcome;
	      using ::mipsolvers::engine::detail::BCDomainProbeStatus;
	      using ::mipsolvers::engine::detail::BCDomainProbeWorkspace;

	      BCDomainProbeWorkspace probe_domain(
	          base_lp, root.lb, root.ub,
	          clique_table.empty() ? nullptr : &clique_table, opt.int_tol);

	      auto commit_probe_domain_deltas =
	          [&](const std::vector<BCDomainProbeDelta>& deltas) {
	        for (const auto& delta : deltas) {
	          if (delta.col < 0 || delta.col >= n) continue;
	          if (delta.new_lb > root.lb[delta.col] +
	                                 bound_improvement_tolerance(
	                                     1e-9, root.lb[delta.col],
	                                     delta.new_lb)) {
	            root.lb[delta.col] = delta.new_lb;
	            ++out.bc_stats.root_domain_probe_committed_bound_changes;
	            root_domain_probe_changed_root = true;
	          }
	          if (delta.new_ub < root.ub[delta.col] -
	                                 bound_improvement_tolerance(
	                                     1e-9, root.ub[delta.col],
	                                     delta.new_ub)) {
	            root.ub[delta.col] = delta.new_ub;
	            ++out.bc_stats.root_domain_probe_committed_bound_changes;
	            root_domain_probe_changed_root = true;
	          }
	        }
	      };

	      std::vector<BCDomainProbeDelta> baseline_deltas;
	      const BCDomainProbeStatus baseline_status =
	          probe_domain.initialize(baseline_deltas);
	      bool probe_domain_ready =
	          baseline_status == BCDomainProbeStatus::Feasible;
	      if (probe_domain_ready) {
	        commit_probe_domain_deltas(baseline_deltas);
	      }

	      auto learn_probe_implications =
	          [&](int trigger_col, bool trigger_value_one,
	              const std::vector<BCDomainProbeDelta>& deltas) {
	        out.bc_stats.root_domain_probe_implication_export_columns_visited +=
	            static_cast<std::uint64_t>(deltas.size());
	        for (const auto& delta : deltas) {
	          const int implied_col = delta.col;
	          if (implied_col < 0 || implied_col >= n ||
	              implied_col == trigger_col) {
	            continue;
	          }
	          if (delta.new_lb > root.lb[implied_col] + 1e-9 &&
	              implication_graph.add_implication(
	                  trigger_col, trigger_value_one, implied_col, true,
	                  delta.new_lb)) {
	            ++dp_implication_sources;
	            ++out.bc_stats.root_domain_probe_implications_learned;
	          }
	          if (delta.new_ub < root.ub[implied_col] - 1e-9 &&
	              implication_graph.add_implication(
	                  trigger_col, trigger_value_one, implied_col, false,
	                  delta.new_ub)) {
	            ++dp_implication_sources;
	            ++out.bc_stats.root_domain_probe_implications_learned;
	          }
	        }
	      };

	      auto commit_probe_fixing = [&](int col, bool value_one) {
	        std::vector<BCDomainProbeDelta> committed_deltas;
	        const BCDomainProbeStatus status =
	            probe_domain.commit(col, value_one, committed_deltas);
	        if (status == BCDomainProbeStatus::Feasible) {
	          commit_probe_domain_deltas(committed_deltas);
	        }
	        return status;
	      };

	      for (int j = 0; j < n; ++j) {
	        if (!probe_domain_ready) break;
	        if (!is_integer_type(base_lp.vars[j])) continue;
	        if (std::abs(root.ub[j] - root.lb[j]) < 1e-9) continue;
	        if (root.lb[j] != 0.0 || root.ub[j] != 1.0) continue;

	        const BCDomainProbeOutcome down =
	            probe_domain.probe(j, /*value_one=*/false);
	        ++dp_count;
	        if (down.status == BCDomainProbeStatus::Infeasible) {
	          if (add_global_conflict(
	                  {BranchDomainLiteral{j, 0.0, false}},
	                  "root_domain_probe_down")) {
	            ++dp_conflict_sources;
	          }
	          const BCDomainProbeStatus commit_status =
	              commit_probe_fixing(j, /*value_one=*/true);
	          if (commit_status == BCDomainProbeStatus::Feasible) {
	            ++dp_fixed;
	          } else if (commit_status == BCDomainProbeStatus::Infeasible) {
	            root.lb[j] = 1.0;
	            root.ub[j] = 0.0;
	            root_domain_probe_changed_root = true;
	            probe_domain_ready = false;
	          }
	          continue;
	        }
	        if (down.status == BCDomainProbeStatus::Incomplete) continue;
	        learn_probe_implications(j, /*trigger_value_one=*/false, down.deltas);

	        const BCDomainProbeOutcome up =
	            probe_domain.probe(j, /*value_one=*/true);
	        ++dp_count;
	        if (up.status == BCDomainProbeStatus::Infeasible) {
	          if (add_global_conflict(
	                  {BranchDomainLiteral{j, 1.0, true}},
	                  "root_domain_probe_up")) {
	            ++dp_conflict_sources;
	          }
	          const BCDomainProbeStatus commit_status =
	              commit_probe_fixing(j, /*value_one=*/false);
	          if (commit_status == BCDomainProbeStatus::Feasible) {
	            ++dp_fixed;
	          } else if (commit_status == BCDomainProbeStatus::Infeasible) {
	            root.lb[j] = 1.0;
	            root.ub[j] = 0.0;
	            root_domain_probe_changed_root = true;
	            probe_domain_ready = false;
	          }
	          continue;
	        }
	        if (up.status == BCDomainProbeStatus::Feasible) {
	          learn_probe_implications(j, /*trigger_value_one=*/true, up.deltas);
	        }
	      }

	      const auto& domain_probe_telemetry = probe_domain.telemetry();
	      out.bc_stats.root_domain_probe_workspace_initializations +=
	          domain_probe_telemetry.workspace_initializations;
	      out.bc_stats.root_domain_probe_worlds +=
	          domain_probe_telemetry.worlds;
	      out.bc_stats.root_domain_probe_trail_pushes +=
	          domain_probe_telemetry.trail_pushes;
	      out.bc_stats.root_domain_probe_rollbacks +=
	          domain_probe_telemetry.rollbacks;
	      out.bc_stats.root_domain_probe_failures +=
	          domain_probe_telemetry.failures;
	      out.bc_stats.root_domain_probe_rows_processed +=
	          domain_probe_telemetry.rows_processed;
	      out.bc_stats.root_domain_probe_changed_columns +=
	          domain_probe_telemetry.changed_columns;
	      if (dp_conflict_sources > 0 || dp_implication_sources > 0) {
	        note_global_domain_learning(dp_conflict_sources +
	                                    dp_implication_sources);
	        note_objective_artifact_learning(dp_implication_sources);
	        (void)refresh_objective_domain_sources("root_domain_probing",
	                                               /*force=*/true);
	      }
	      fixed_by_probe += dp_fixed;
      auto t_dp_end = std::chrono::steady_clock::now();
      if (opt.verbose) {
        fmt::print(stderr, "[B&C PROBE] domain prop: {:.1f}ms probes={} fixed={}\n",
                   std::chrono::duration<double, std::milli>(t_dp_end - t_dp_start).count(),
                   dp_count, dp_fixed);
      }
    }

    // Phase 2: LP probing — probe top fractional unfixed binaries for pseudocosts.
    // When a good incumbent already exists (from warm-start), reduce probing budget
    // since reduced-cost fixing will handle most variable fixings more cheaply.
    int probed = 0;
    const bool has_good_incumbent = root_incumbent_good_enough();
    // Skip LP probing entirely for large problems with incumbent:
    // probes cost ~90-400ms each (IPM or simplex) and rarely fix variables.
    // Pseudocosts are already initialized from diving LP solves, and
    // reduced-cost fixing (below) is far more cost-effective.
    const int m_total = static_cast<int>(base_lp.A.rows() + base_lp.Aeq.rows());
    const bool split_bound_probe_root =
        opt.root_split_bound_probing && current_large_generic_low_frac_root() &&
        root_frac_bin_count > 0 &&
        !publish_root_bound_gap_certificate("root_split_bound_probe_gate") &&
        !(requested_vendored_highs_lp && root_primal_polish_complete &&
          has_incumbent && std::isfinite(incumbent_obj) &&
          std::isfinite(root.bound) &&
          incumbent_root_rel_gap() <=
              std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5));
    const int base_probe_budget = split_bound_probe_root
            ? std::min(opt.root_split_bound_probe_max, root_frac_bin_count)
            : ((m_total > 5000) ? 0  // skip ordinary LP probing for large problems
               : ((m_total > 3000) ? std::max(10, opt.max_probe_vars / 5) : opt.max_probe_vars));
    const int max_probe_vars = split_bound_probe_root
        ? std::min(base_probe_budget, n)
        : ((base_probe_budget == 0) ? 0  // respect large-problem skip
           : (has_incumbent && m_total > 10000) ? 0
           : (has_incumbent
              ? (has_good_incumbent ? std::min(3, n) : std::min(5, n))
              : std::min(base_probe_budget, n)));
    const double probe_time_budget = split_bound_probe_root
        ? std::min(opt.root_split_bound_probe_time_limit,
                   std::max(0.0, opt.time_limit_sec) * 0.10)
        : std::min(
            opt.time_limit_sec * (has_incumbent ? 0.01 : (m_total > 5000 ? 0.01 : 0.03)),
            m_total > opt.ipm_auto_threshold ? 0.15 : (m_total > 5000 ? 0.5 : 2.0));
    const double split_bound_probe_gate_bound = root.bound;
    const auto t_probe_start = std::chrono::steady_clock::now();
    // Lazy-initialize probe LP infrastructure only when actually needed.
    std::unique_ptr<StandardFormLP> probe_sf_ptr;
    SimplexOptions probe_opt;
    probe_opt.max_iter = std::max(opt.max_lp_iter * 5, 1000);
    probe_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    probe_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    probe_opt.verbose = false;
    probe_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
    probe_opt.lp_kernel_backend = opt.lp_kernel_backend;
    apply_bc_first_class_simplex_state(probe_opt, /*allow_frontier_remap=*/true);
    auto ensure_probe_sf = [&]() -> StandardFormLP& {
      if (!probe_sf_ptr) probe_sf_ptr = std::make_unique<StandardFormLP>(ensure_base_sf());
      return *probe_sf_ptr;
    };

    // HiGHS-style cutoff conflicts for implied-integer cost rows.
    //
    // Detect structural rows of the form
    //   a_s s + a_y y            <= rhs,  a_s < 0, y binary
    //   a_s s + a_y y + a_z z   <= rhs,  a_s < 0, a_y > 0, a_z < 0
    // where s is a nonnegative continuous variable with positive objective
    // cost.  For binary assignments y=1 (and z=0, when present), the row
    // implies a mandatory objective contribution from s.  We then solve one
    // auxiliary LP with these s-costs removed to get a valid lower bound on
    // the remaining objective.  Under the incumbent cutoff, any event cover
    // whose mandatory s-cost exceeds the remaining budget becomes a legal
    // conflict clause:
    //   sum y_i - sum z_i <= |cover|-1.
	    if (opt.enable_objective_cutoff_conflict_cuts && has_incumbent &&
	        base_lp.sense == Sense::Minimize && std::isfinite(incumbent_obj) &&
	        std::isfinite(root.bound) &&
	        opt.objective_cutoff_conflict_max_cuts > 0) {
	      const double incumbent_upper_limit_for_cutoff =
	          highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                             opt.int_tol);
	      if (std::isfinite(incumbent_upper_limit_for_cutoff)) {
	      std::vector<ObjectiveCutoffEvent> events;
      events.reserve(256);
      std::vector<char> is_cutoff_s(static_cast<size_t>(n), 0);
      const Eigen::SparseMatrix<double, Eigen::RowMajor> Arow_cutoff = base_lp.A;
      constexpr double kCoefTol = 1e-9;
      for (int r = 0; r < static_cast<int>(Arow_cutoff.rows()); ++r) {
        int s_col = -1;
        int y_col = -1;
        int z_col = -1;
        double a_s = 0.0;
        double a_y = 0.0;
        double a_z = 0.0;
        bool bad = false;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow_cutoff, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= kCoefTol || j < 0 || j >= n) continue;
          const auto& v = base_lp.vars[static_cast<size_t>(j)];
          const bool binary_like = is_integer_type(v) && v.lb >= -1e-9 && v.ub <= 1.0 + 1e-9;
          if (v.type == VarType::Continuous && a < -kCoefTol &&
              base_lp.c[j] > kCoefTol && v.lb >= -1e-9) {
            if (s_col >= 0) { bad = true; break; }
            s_col = j;
            a_s = a;
          } else if (binary_like && a > kCoefTol) {
            if (y_col >= 0) { bad = true; break; }
            y_col = j;
            a_y = a;
          } else if (binary_like && a < -kCoefTol) {
            if (z_col >= 0) { bad = true; break; }
            z_col = j;
            a_z = a;
          } else {
            bad = true;
            break;
          }
        }
        if (bad || s_col < 0 || y_col < 0) continue;
        if (z_col >= 0) {
          const double scale = std::max({1.0, std::abs(a_y), std::abs(a_z)});
          if (std::abs(std::abs(a_z) - std::abs(a_y)) > 1e-6 * scale) continue;
        }
        const double implied_s = (a_y - base_lp.b[r]) / (-a_s);
        if (!(implied_s > 1e-8) || !std::isfinite(implied_s)) continue;
        const double event_cost = base_lp.c[s_col] * implied_s;
        if (!(event_cost > 1e-8) || !std::isfinite(event_cost)) continue;
        const double act = root.x_relax.size() == n
            ? root.x_relax[y_col] - (z_col >= 0 ? root.x_relax[z_col] : 0.0)
            : 0.0;
        events.push_back(ObjectiveCutoffEvent{r, s_col, y_col, z_col, implied_s,
                                              event_cost, act});
        is_cutoff_s[static_cast<size_t>(s_col)] = 1;
      }

      if (!events.empty()) {
        const double root_gap = std::max(0.0, incumbent_obj - root.bound);
        const double min_event_cost =
            std::max(1e-8, opt.objective_cutoff_conflict_min_event_gap_fraction *
                               std::max(1.0, root_gap));

        LPModel rest_lp = base_lp;
        int n_cutoff_s = 0;
        double max_cutoff_cost = 0.0;
        for (int j = 0; j < n; ++j) {
          if (!is_cutoff_s[static_cast<size_t>(j)]) continue;
          ++n_cutoff_s;
          const double coeff = std::max(0.0, rest_lp.c[j]);
          rest_lp.c[j] = 0.0;
          if (std::isfinite(base_lp.vars[static_cast<size_t>(j)].ub)) {
            max_cutoff_cost += coeff * std::max(0.0, base_lp.vars[static_cast<size_t>(j)].ub);
          } else {
            max_cutoff_cost = kInf;
          }
        }

        StandardFormLP rest_sf = build_standard_form_lp(rest_lp);
        ruiz_scale_standard_form(rest_sf);
        const SimplexBasis* rest_hint = nullptr;
        if (root.basis_hint &&
            root.basis_hint->rows == static_cast<int>(rest_sf.A.rows()) &&
            root.basis_hint->cols == static_cast<int>(rest_sf.A.cols())) {
          rest_hint = root.basis_hint.get();
        }
        auto rest_res = solve_lp_from_sf(rest_sf, probe_opt, rest_hint);
        ++out.bc_stats.lp_solves;

        if (rest_res.result.stats.success &&
            std::isfinite(rest_res.result.stats.objective)) {
          const double rest_lb = rest_res.result.stats.objective;
          objective_cutoff_events = events;
          objective_cutoff_rest_lb = rest_lb;
	          const double cutoff_budget =
	              incumbent_upper_limit_for_cutoff - rest_lb;

          if (cutoff_budget > 0.0 && std::isfinite(cutoff_budget)) {
            std::vector<Eigen::SparseVector<double>> cutoff_rows;
            std::vector<double> cutoff_rhs;
            cutoff_rows.reserve(static_cast<size_t>(opt.objective_cutoff_conflict_max_cuts + 1));
            cutoff_rhs.reserve(static_cast<size_t>(opt.objective_cutoff_conflict_max_cuts + 1));

            // Do not materialize the objective cutoff budget as a model row.
            // That row is typically dense over many continuous cost variables
            // and makes every subsequent node LP slower.  HiGHS-style cutoff
            // processing uses the budget as solver state to derive sparse
            // conflicts; only those sparse conflicts should enter the LP.
            (void)max_cutoff_cost;

            int weighted_event_cuts = 0;
            if (opt.enable_objective_cutoff_weighted_event_cuts &&
                opt.objective_cutoff_weighted_event_max_terms > 0 &&
                root.x_relax.size() == n) {
              std::vector<int> best_event_for_s(static_cast<std::size_t>(n), -1);
              std::vector<double> best_score_for_s(static_cast<std::size_t>(n), 0.0);
              for (int i = 0; i < static_cast<int>(events.size()); ++i) {
                const auto& e = events[static_cast<std::size_t>(i)];
                if (e.s < 0 || e.s >= n) continue;
                const double activity = root.x_relax[e.y] -
                    (e.z >= 0 ? root.x_relax[e.z] : 0.0);
                const double score = e.cost * activity;
                if (!(score > best_score_for_s[static_cast<std::size_t>(e.s)] + 1e-9)) {
                  continue;
                }
                best_score_for_s[static_cast<std::size_t>(e.s)] = score;
                best_event_for_s[static_cast<std::size_t>(e.s)] = i;
              }

              std::vector<int> weighted_candidates;
              weighted_candidates.reserve(events.size());
              for (int s = 0; s < n; ++s) {
                const int idx = best_event_for_s[static_cast<std::size_t>(s)];
                if (idx >= 0) weighted_candidates.push_back(idx);
              }
              std::sort(weighted_candidates.begin(), weighted_candidates.end(),
                        [&](int a, int b) {
                          const auto& ea = events[static_cast<std::size_t>(a)];
                          const auto& eb = events[static_cast<std::size_t>(b)];
                          const double sa = ea.cost *
                              (root.x_relax[ea.y] - (ea.z >= 0 ? root.x_relax[ea.z] : 0.0));
                          const double sb = eb.cost *
                              (root.x_relax[eb.y] - (eb.z >= 0 ? root.x_relax[eb.z] : 0.0));
                          if (std::abs(sa - sb) > 1e-9) return sa > sb;
                          return ea.cost > eb.cost;
                        });

              const int max_terms =
                  std::min(opt.objective_cutoff_weighted_event_max_terms,
                           static_cast<int>(weighted_candidates.size()));
              Eigen::SparseVector<double> weighted_cut(n);
              weighted_cut.reserve(2 * max_terms);
              double lhs_at_root = 0.0;
              int terms = 0;
              for (int p = 0; p < max_terms; ++p) {
                const auto& e = events[static_cast<std::size_t>(weighted_candidates[static_cast<std::size_t>(p)])];
                const double activity = root.x_relax[e.y] -
                    (e.z >= 0 ? root.x_relax[e.z] : 0.0);
                const double contribution = e.cost * activity;
                if (!(contribution > 1e-8)) continue;
                weighted_cut.coeffRef(e.y) += e.cost;
                if (e.z >= 0) weighted_cut.coeffRef(e.z) -= e.cost;
                lhs_at_root += contribution;
                ++terms;
              }

              const double cut_tol =
                  1e-7 * std::max(1.0, std::max(std::abs(lhs_at_root),
                                                std::abs(cutoff_budget)));
              if (terms > 0 && lhs_at_root > cutoff_budget + cut_tol) {
                cutoff_rows.push_back(std::move(weighted_cut));
                cutoff_rhs.push_back(cutoff_budget);
                weighted_event_cuts = 1;
	        }
		      }

	            std::vector<ObjectiveCutoffEvent> cover_events;
            cover_events.reserve(events.size());
            for (const auto& e : events) {
              if (e.cost + 1e-9 < min_event_cost) continue;
              if (root.lb[e.y] > 1.0 - 1e-9) continue;
              if (e.z >= 0 && root.ub[e.z] < 1e-9) continue;
              cover_events.push_back(e);
            }
            std::sort(cover_events.begin(), cover_events.end(),
                      [](const ObjectiveCutoffEvent& a, const ObjectiveCutoffEvent& b) {
                        const double sa = std::max(0.0, a.activity) + 1e-6 * a.cost;
                        const double sb = std::max(0.0, b.activity) + 1e-6 * b.cost;
                        if (std::abs(sa - sb) > 1e-12) return sa > sb;
                        return a.cost > b.cost;
                      });

            std::unordered_set<std::string> seen_covers;
            for (int seed = 0;
                 seed < static_cast<int>(cover_events.size()) &&
                 static_cast<int>(cutoff_rows.size()) < opt.objective_cutoff_conflict_max_cuts + 1;
                 ++seed) {
              double sum_cost = 0.0;
              double lhs_at_root = 0.0;
              std::vector<int> chosen;
              chosen.reserve(16);
              for (int p = seed; p < static_cast<int>(cover_events.size()); ++p) {
                const auto& e = cover_events[static_cast<size_t>(p)];
                sum_cost += e.cost;
                lhs_at_root += e.activity;
                chosen.push_back(p);
                if (sum_cost > cutoff_budget + 1e-7) break;
                if (static_cast<int>(chosen.size()) >= 64) break;
              }
              if (sum_cost <= cutoff_budget + 1e-7 || chosen.empty()) continue;
              const double rhs = static_cast<double>(chosen.size()) - 1.0;
              // A cutoff cover is a conflict cut: it is useful even when the
              // current root LP point does not violate it, because it prevents
              // later nodes from spending LP solves on a combination of binary
              // events whose implied objective cost already exceeds the
              // incumbent cutoff.  HiGHS keeps this kind of nonviolated
              // conflict in the global conflict/cut machinery.
              const bool violated_at_root = lhs_at_root > rhs + 1e-7;
              const bool admit_nonviolated_cutoff_cover =
                  strict_highs_lp_contract ||
                  opt.enable_nonviolated_cutoff_conflict_covers;
              if (!violated_at_root && !admit_nonviolated_cutoff_cover) {
                continue;
              }
              if (!strict_highs_lp_contract && !violated_at_root &&
                  static_cast<int>(chosen.size()) > 16) {
                continue;
              }

              std::vector<int> key_ids;
              key_ids.reserve(chosen.size());
              for (int idx : chosen) key_ids.push_back(cover_events[static_cast<size_t>(idx)].row);
              std::sort(key_ids.begin(), key_ids.end());
              std::string key;
              for (int id : key_ids) {
                key += std::to_string(id);
                key.push_back(',');
              }
              if (!seen_covers.insert(std::move(key)).second) continue;

	              Eigen::SparseVector<double> cut(n);
	              cut.reserve(static_cast<int>(2 * chosen.size()));
	              std::vector<BranchDomainLiteral> cover_clause;
	              cover_clause.reserve(static_cast<std::size_t>(2 * chosen.size()));
	              for (int idx : chosen) {
	                const auto& e = cover_events[static_cast<size_t>(idx)];
	                cut.coeffRef(e.y) += 1.0;
	                cover_clause.push_back(BranchDomainLiteral{e.y, 1.0, true});
	                if (e.z >= 0) cut.coeffRef(e.z) -= 1.0;
	                if (e.z >= 0) {
	                  cover_clause.push_back(
	                      BranchDomainLiteral{e.z, 0.0, false});
	                }
	              }
	              canonicalize_branch_literals(cover_clause);
	              if (!cover_clause.empty()) {
	                const bool pool_added = add_global_conflict(
	                    cover_clause, "root_objective_cover");
	                const bool impl_added =
	                    add_binary_implications_from_conflict_clause(
	                        base_lp.vars, cover_clause, implication_graph);
	                if (pool_added || impl_added) {
	                  note_global_domain_learning(1 + (impl_added ? 1 : 0));
	                  ++out.bc_stats.cutoff_conflict_clauses_learned;
	                }
	              }
	              cutoff_rows.push_back(std::move(cut));
	              cutoff_rhs.push_back(rhs);
	            }

	            if (!cutoff_rows.empty()) {
	              const int added_cutoff = static_cast<int>(cutoff_rows.size());
	              if (strict_highs_lp_contract) {
	                int domain_rows_added = 0;
	                for (int i = 0; i < added_cutoff; ++i) {
	                  if (add_cut_to_global_domain_pool(
	                          std::move(cutoff_rows[static_cast<std::size_t>(i)]),
	                          cutoff_rhs[static_cast<std::size_t>(i)],
	                          "objective_cutoff_domain_row",
	                          /*domain_only=*/true)) {
	                    ++domain_rows_added;
	                    ++out.bc_stats.cutoff_conflict_cuts_added;
	                  }
	                }
	    if (domain_rows_added > 0) {
		                  (void)refresh_objective_domain_sources(
		                      "objective_cutoff_domain_row", /*force=*/true);
		                }
	                if (opt.verbose || trace_bc_timeline) {
	                  fmt::print(stderr,
	                             "[B&C CUTOFF-CONFLICT] events={} svars={} "
	                             "rest_lb={:.2f} budget={:.2f} cuts={} "
	                             "weighted={} domain_only=1 root_lp_rows=0 "
	                             "pool_added={}\n",
	                             static_cast<int>(events.size()), n_cutoff_s,
	                             rest_lb, cutoff_budget, added_cutoff,
	                             weighted_event_cuts, domain_rows_added);
		                }
		              } else {
		              const int prev_cutoff_rows = static_cast<int>(base_lp.A.rows());
              const double cutoff_old_bound = root.bound;
              const Eigen::VectorXd cutoff_old_x = root.x_relax;
              const auto cutoff_old_basis = root.basis_hint;
              const Eigen::VectorXd cutoff_old_lb = root.lb;
              const Eigen::VectorXd cutoff_old_ub = root.ub;
              add_sparse_rows_to_lp(base_lp, cutoff_rows, cutoff_rhs);
              out.bc_stats.cuts_added += added_cutoff;
              persistent_lp_state.invalidate_base_sf();
              probe_sf_ptr.reset();

              Eigen::VectorXd cutoff_trial_lb = root.lb;
              Eigen::VectorXd cutoff_trial_ub = root.ub;
              int cutoff_domain_tightened = 0;
              if (opt.root_cut_reject_nonmoving_rows) {
                const Eigen::SparseMatrix<double, Eigen::RowMajor> cutoff_A_row =
                    base_lp.A;
                const Eigen::SparseMatrix<double, Eigen::RowMajor> cutoff_Aeq_row =
                    base_lp.Aeq;
                cutoff_domain_tightened = node_bound_propagation(
                    base_lp, cutoff_A_row, cutoff_Aeq_row, cutoff_trial_lb,
                    cutoff_trial_ub, opt.bound_propagation_rounds);
                if (!bounds_consistent(cutoff_trial_lb, cutoff_trial_ub)) {
                  cutoff_domain_tightened = -1;
                }
              }
              StandardFormLP& cutoff_sf = ensure_probe_sf();
              update_standard_form_bounds(
                  cutoff_sf, base_lp,
                  cutoff_domain_tightened > 0 ? cutoff_trial_lb : root.lb,
                  cutoff_domain_tightened > 0 ? cutoff_trial_ub : root.ub);
              auto cutoff_res = solve_lp_from_sf(cutoff_sf, probe_opt, nullptr);
              ++out.bc_stats.lp_solves;
              bool cutoff_keep_rows = false;
              double cutoff_candidate_bound = root.bound;
              if (cutoff_res.result.stats.success && cutoff_res.result.x.size() == n &&
                  std::isfinite(cutoff_res.result.stats.objective)) {
                cutoff_candidate_bound =
                    std::min(incumbent_obj, cutoff_res.result.stats.objective);
                const double lift = cutoff_candidate_bound - cutoff_old_bound;
                cutoff_keep_rows =
                    !opt.root_cut_reject_nonmoving_rows ||
                    cutoff_domain_tightened > 0 ||
                    lift > std::max(opt.root_cut_min_bound_lift,
                                    1e-10 * std::max(1.0, std::abs(cutoff_old_bound)));
              } else if (cutoff_res.result.stats.status.find("Infeasible") != std::string::npos) {
                cutoff_candidate_bound = incumbent_obj;
                cutoff_keep_rows = true;
              }

              if (cutoff_keep_rows) {
                if (cutoff_domain_tightened > 0) {
                  root.lb = std::move(cutoff_trial_lb);
                  root.ub = std::move(cutoff_trial_ub);
                  out.bc_stats.root_cut_domain_tightenings +=
                      static_cast<std::uint64_t>(cutoff_domain_tightened);
                }
              }

              if (cutoff_keep_rows && cutoff_res.result.stats.success &&
                  cutoff_res.result.x.size() == n &&
                  std::isfinite(cutoff_res.result.stats.objective)) {
                root.x_relax = clamp_to_bounds(cutoff_res.result.x, root.lb, root.ub);
                root.x_seed = root.x_relax;
                root.bound = cutoff_candidate_bound;
                auto cutoff_basis = std::make_shared<SimplexBasis>(cutoff_res.basis);
                cutoff_basis->compact_indices_storage();
                cutoff_basis->cached_reduced_costs =
                    std::make_shared<const Eigen::VectorXd>(
                        cutoff_res.reduced_costs);
                root.basis_hint = cutoff_basis;
                root_relax.primal = cutoff_res.result;
                root_relax.dual_bound = cutoff_candidate_bound;
	                root_relax.basis_hint = root.basis_hint;
	                root_relax.simplex =
	                    std::make_shared<SimplexResult>(cutoff_res);
	                root_relax.simplex->form = cutoff_sf;
	                refresh_root_redcost_lurking_bounds("root_cutoff_conflict_resolve");
	              } else if (cutoff_keep_rows) {
                root.bound = incumbent_obj;
              } else {
                truncate_ineq_rows(base_lp, prev_cutoff_rows);
                out.bc_stats.cuts_added -= added_cutoff;
                root.lb = cutoff_old_lb;
                root.ub = cutoff_old_ub;
                root.x_relax = cutoff_old_x;
                root.basis_hint = cutoff_old_basis;
                root.bound = cutoff_old_bound;
                persistent_lp_state.invalidate_base_sf();
                probe_sf_ptr.reset();
                out.bc_stats.root_cut_rows_rejected_nonmoving +=
                    static_cast<std::uint64_t>(added_cutoff);
                ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
              }

              if (opt.verbose || !cutoff_keep_rows) {
                fmt::print(stderr,
                           "[B&C CUTOFF-CONFLICT] events={} svars={} rest_lb={:.2f} "
                           "budget={:.2f} cuts={} weighted={} root_lb={:.2f} "
                           "kept={} domain_tight={}\n",
                           static_cast<int>(events.size()), n_cutoff_s, rest_lb,
                           cutoff_budget, added_cutoff, weighted_event_cuts,
	                           cutoff_keep_rows ? root.bound : cutoff_candidate_bound,
	                           cutoff_keep_rows ? 1 : 0, cutoff_domain_tightened);
	              }
	              }
	            } else if (opt.verbose) {
              fmt::print(stderr,
                         "[B&C CUTOFF-CONFLICT] events={} svars={} rest_lb={:.2f} "
                         "budget={:.2f} cuts=0 weighted=0\n",
                         static_cast<int>(events.size()), n_cutoff_s, rest_lb,
                         cutoff_budget);
            }
	          }
	        }
	      }
	    }
	  }

	    const bool defer_root_graph_implied_bound_resolve =
        requested_vendored_highs_lp && has_incumbent &&
        current_large_generic_low_frac_root() &&
        std::isfinite(incumbent_root_rel_gap()) &&
        incumbent_root_rel_gap() <=
            std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5);
    if (!defer_root_graph_implied_bound_resolve &&
        opt.enable_graph_implied_bound_cuts &&
        opt.graph_implied_bound_max_cuts > 0 &&
        !implication_graph.empty() &&
        root.x_relax.size() == n) {
      std::vector<Eigen::SparseVector<double>> ib_rows;
      std::vector<double> ib_rhs;
      ib_rows.reserve(static_cast<std::size_t>(opt.graph_implied_bound_max_cuts));
      ib_rhs.reserve(static_cast<std::size_t>(opt.graph_implied_bound_max_cuts));
      const int ib_added = append_graph_implied_bound_cuts(
          base_lp, root.x_relax, root.lb, root.ub, implication_graph,
          opt.graph_implied_bound_max_cuts, ib_rows, ib_rhs);
      if (ib_added > 0) {
        if (requested_vendored_highs_lp && opt.auto_highs_root_pipeline &&
            has_incumbent) {
          int deferred_rows = 0;
          for (int i = 0; i < ib_added; ++i) {
            Eigen::SparseVector<double>& row =
                ib_rows[static_cast<std::size_t>(i)];
            const double rhs = ib_rhs[static_cast<std::size_t>(i)];
            const double norm =
                std::sqrt(std::max(0.0, row.squaredNorm()));
            if (!(norm > 1e-12) || !std::isfinite(norm) ||
                !std::isfinite(rhs)) {
              continue;
            }
            const std::size_t hash = sparse_cut_hash(row);
            root_deferred_implied_bound_cutpool_rows.push_back(
                PoolCut{std::move(row), rhs, 0, 0.0, norm, hash});
            ++deferred_rows;
          }
          out.bc_stats.root_cut_rows_rejected_nonmoving +=
              static_cast<std::uint64_t>(std::max(0, ib_added));
          ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
          if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
            fmt::print(stderr,
                       "[B&C IMPLIED-BOUND] graph_arcs={} cuts={} "
                       "deferred={} kept=0 domain_tight=0 "
                       "mode=proof_artifact\n",
                       implication_graph.size(), ib_added, deferred_rows);
          }
        } else {
        const int prev_ib_rows = static_cast<int>(base_lp.A.rows());
        const double ib_old_bound = root.bound;
        const Eigen::VectorXd ib_old_x = root.x_relax;
        const auto ib_old_basis = root.basis_hint;
        const Eigen::VectorXd ib_old_lb = root.lb;
        const Eigen::VectorXd ib_old_ub = root.ub;
        add_sparse_rows_to_lp(base_lp, ib_rows, ib_rhs);
        out.bc_stats.cuts_added += ib_added;
        persistent_lp_state.invalidate_base_sf();
        probe_sf_ptr.reset();

        Eigen::VectorXd ib_trial_lb = root.lb;
        Eigen::VectorXd ib_trial_ub = root.ub;
        int ib_domain_tightened = 0;
        if (opt.root_cut_reject_nonmoving_rows) {
          const Eigen::SparseMatrix<double, Eigen::RowMajor> ib_A_row = base_lp.A;
          const Eigen::SparseMatrix<double, Eigen::RowMajor> ib_Aeq_row = base_lp.Aeq;
          ib_domain_tightened = node_bound_propagation(
              base_lp, ib_A_row, ib_Aeq_row, ib_trial_lb, ib_trial_ub,
              opt.bound_propagation_rounds);
          if (!bounds_consistent(ib_trial_lb, ib_trial_ub)) {
            ib_domain_tightened = -1;
          }
        }
        StandardFormLP& ib_sf = ensure_probe_sf();
        update_standard_form_bounds(
            ib_sf, base_lp, ib_domain_tightened > 0 ? ib_trial_lb : root.lb,
            ib_domain_tightened > 0 ? ib_trial_ub : root.ub);
        auto ib_res = solve_lp_from_sf(ib_sf, probe_opt, nullptr);
        ++out.bc_stats.lp_solves;
        bool ib_keep_rows = false;
        double ib_candidate_bound = root.bound;
        if (ib_res.result.stats.success && ib_res.result.x.size() == n &&
            std::isfinite(ib_res.result.stats.objective)) {
          ib_candidate_bound = has_incumbent
              ? std::min(incumbent_obj, ib_res.result.stats.objective)
              : ib_res.result.stats.objective;
          const double lift = ib_candidate_bound - ib_old_bound;
          ib_keep_rows = !opt.root_cut_reject_nonmoving_rows ||
              ib_domain_tightened > 0 ||
              lift > std::max(opt.root_cut_min_bound_lift,
                              1e-10 * std::max(1.0, std::abs(ib_old_bound)));
        } else if (ib_res.result.stats.status.find("Infeasible") != std::string::npos) {
          ib_candidate_bound = has_incumbent ? incumbent_obj : kInf;
          ib_keep_rows = true;
        }

        if (ib_keep_rows && ib_domain_tightened > 0) {
          root.lb = std::move(ib_trial_lb);
          root.ub = std::move(ib_trial_ub);
          out.bc_stats.root_cut_domain_tightenings +=
              static_cast<std::uint64_t>(ib_domain_tightened);
        }

        if (ib_keep_rows && ib_res.result.stats.success &&
            ib_res.result.x.size() == n &&
            std::isfinite(ib_res.result.stats.objective)) {
          root.x_relax = clamp_to_bounds(ib_res.result.x, root.lb, root.ub);
          root.x_seed = root.x_relax;
          root.bound = ib_candidate_bound;
          auto ib_basis = std::make_shared<SimplexBasis>(ib_res.basis);
          ib_basis->compact_indices_storage();
          ib_basis->cached_reduced_costs =
              std::make_shared<const Eigen::VectorXd>(ib_res.reduced_costs);
          root.basis_hint = ib_basis;
          root_relax.primal = ib_res.result;
	          root_relax.dual_bound = ib_candidate_bound;
	          root_relax.basis_hint = root.basis_hint;
	          root_relax.simplex = std::make_shared<SimplexResult>(ib_res);
	          root_relax.simplex->form = ib_sf;
	          refresh_root_redcost_lurking_bounds("root_graph_implied_bound_resolve");
	        } else if (ib_keep_rows) {
          root.bound = ib_candidate_bound;
        } else {
          truncate_ineq_rows(base_lp, prev_ib_rows);
          out.bc_stats.cuts_added -= ib_added;
          root.lb = ib_old_lb;
          root.ub = ib_old_ub;
          root.x_relax = ib_old_x;
          root.basis_hint = ib_old_basis;
          root.bound = ib_old_bound;
          persistent_lp_state.invalidate_base_sf();
          probe_sf_ptr.reset();
          out.bc_stats.root_cut_rows_rejected_nonmoving +=
              static_cast<std::uint64_t>(ib_added);
          ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
        }

        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
          fmt::print(stderr,
                     "[B&C IMPLIED-BOUND] graph_arcs={} cuts={} root_lb={:.2f} "
                     "kept={} domain_tight={}\n",
                     implication_graph.size(), ib_added,
                     ib_keep_rows ? root.bound : ib_candidate_bound,
                     ib_keep_rows ? 1 : 0, ib_domain_tightened);
        }
        }
      } else if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C IMPLIED-BOUND] graph_arcs={} cuts=0\n",
                   implication_graph.size());
      }
    }

    const double split_bound_prior_lift =
        std::isfinite(split_bound_probe_gate_bound) && std::isfinite(root.bound)
            ? root.bound - split_bound_probe_gate_bound
            : 0.0;
    const bool cap_split_bound_lp_probe =
        split_bound_probe_root &&
        split_bound_prior_lift >
            std::max(opt.root_cut_min_bound_lift,
                     1e-10 * std::max(1.0, std::abs(split_bound_probe_gate_bound)));
    const int effective_max_probe_vars =
        cap_split_bound_lp_probe
            ? std::min(max_probe_vars, std::max(8, std::min(16, root_frac_bin_count)))
            : max_probe_vars;
    if (split_bound_probe_root) {
      out.bc_stats.root_split_bound_probe_cap_applied =
          cap_split_bound_lp_probe ? 1 : 0;
      out.bc_stats.root_split_bound_probe_cap_from =
          static_cast<std::uint64_t>(std::max(0, max_probe_vars));
      out.bc_stats.root_split_bound_probe_cap_to =
          static_cast<std::uint64_t>(std::max(0, effective_max_probe_vars));
    }
    if (cap_split_bound_lp_probe && opt.verbose) {
      fmt::print(stderr,
                 "[B&C PROBE] cap split-bound LP probing: prior root rows "
                 "lifted bound by {:.6g} ({:.2f}->{:.2f}); vars {}->{}\n",
                 split_bound_prior_lift, split_bound_probe_gate_bound,
                 root.bound, max_probe_vars, effective_max_probe_vars);
    }

    // Collect unfixed binary candidates, prioritize most fractional.
    std::vector<int> probe_cands;
    for (int j = 0; j < n; ++j) {
      if (!is_integer_type(base_lp.vars[j])) continue;
      if (std::abs(root.ub[j] - root.lb[j]) < 1e-9) continue;  // Already fixed.
      if (root.lb[j] != 0.0 || root.ub[j] != 1.0) continue;
      // Only probe variables that are actually fractional in the root relaxation.
      const double frac = root.x_relax[j] - std::floor(root.x_relax[j]);
      if (frac <= opt.int_tol || frac >= 1.0 - opt.int_tol) continue;
      probe_cands.push_back(j);
    }
    std::sort(probe_cands.begin(), probe_cands.end(), [&](int a, int b) {
      const double fa = std::abs(root.x_relax[a] - 0.5);
      const double fb = std::abs(root.x_relax[b] - 0.5);
      return fa < fb;
    });
    if (split_bound_probe_root) {
      out.bc_stats.root_split_bound_probe_candidates =
          static_cast<std::uint64_t>(probe_cands.size());
    }

	    const double probe_base_bound = root.bound;
	    double best_split_bound = root.bound;
	    int best_split_var = -1;
	    std::optional<Eigen::VectorXd> root_probe_lb_workspace;
	    std::optional<Eigen::VectorXd> root_probe_ub_workspace;
	    std::optional<SimplexBasis> root_probe_basis;
	    bool root_probe_sf_synced = false;
	    SimplexOptions root_probe_simplex_opt = probe_opt;
	    root_probe_simplex_opt.allow_persistent_lp_state = true;
	    auto ensure_root_probe_bound_workspace = [&]() {
	      if (!root_probe_lb_workspace.has_value()) {
	        root_probe_lb_workspace.emplace(root.lb);
	        root_probe_ub_workspace.emplace(root.ub);
	        ++out.bc_stats.root_lp_probe_bound_workspace_initializations;
	      }
	    };
	    auto ensure_root_probe_sf = [&]() -> StandardFormLP& {
	      StandardFormLP& sf = ensure_probe_sf();
	      if (!root_probe_sf_synced) {
	        update_standard_form_bounds(sf, base_lp, root.lb, root.ub);
	        root_probe_sf_synced = true;
	      }
	      return sf;
	    };
	    auto root_probe_basis_hint = [&]() -> const SimplexBasis* {
	      if (root_probe_basis.has_value()) return &*root_probe_basis;
	      if (!root.basis_hint) return nullptr;
	      root_probe_basis.emplace(*root.basis_hint);
	      // The node LP owner is mutable. A probe must establish its own owner
	      // before persistent resolves can be used across probe directions.
	      root_probe_basis->cached_sparse_basis.reset();
	      root_probe_basis->persist_eta_count = 0;
	      return &*root_probe_basis;
	    };
	    auto invalidate_root_probe_sf = [&]() {
	      probe_sf_ptr.reset();
	      root_probe_sf_synced = false;
	      root_probe_basis.reset();
	    };
	    auto commit_root_probe_bound = [&](int column, bool is_lb,
	                                       double new_value) {
	      const double old_value = is_lb ? root.lb[column] : root.ub[column];
	      if (is_lb) {
	        root.lb[column] = new_value;
	      } else {
	        root.ub[column] = new_value;
	      }
	      if (root_probe_lb_workspace.has_value()) {
	        if (is_lb) {
	          (*root_probe_lb_workspace)[column] = new_value;
	        } else {
	          (*root_probe_ub_workspace)[column] = new_value;
	        }
	      }
	      if (!root_probe_sf_synced) return;
	      const std::vector<BoundChangeInfo> committed_change{
	          {column, 0.0, is_lb, old_value, new_value}};
	      if (!update_standard_form_bounds_incremental(
	              *probe_sf_ptr, base_lp, committed_change)) {
	        invalidate_root_probe_sf();
	      }
	    };
	    for (int idx = 0; idx < static_cast<int>(probe_cands.size()) &&
	                      probed < effective_max_probe_vars; ++idx) {
      if (bc_optional_root_budget_expired()) break;
      const int j = probe_cands[idx];
      const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_probe_start).count();
      if (elapsed > probe_time_budget) break;

      bool down_success = false;
      bool down_proved_infeas = false;
      double down_bound = root.bound;
      int ipm_probe_directions_remaining = 2;
      auto prepare_root_probe_ipm_solve =
          [&](NativeIPMLPAdapter& solver) -> bool {
        const double phase_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_probe_start).count();
        const double remaining = std::min(
            probe_time_budget - phase_elapsed,
            bc_remaining_sec() - bc_post_root_finalize_reserve_sec);
        if (remaining <= 0.001) return false;
        solver.set_solve_time_limit(std::max(
            0.001, remaining /
                       std::max(1, ipm_probe_directions_remaining)));
	        --ipm_probe_directions_remaining;
	        return true;
	      };
	      auto run_root_probe_direction =
	          [&](bool is_up, bool& success, bool& proved_infeas,
	              double& direction_bound) {
	            ensure_root_probe_bound_workspace();
	            Eigen::VectorXd& plb = *root_probe_lb_workspace;
	            Eigen::VectorXd& pub = *root_probe_ub_workspace;
	            const double old_value = is_up ? plb[j] : pub[j];
	            const double new_value = is_up ? 1.0 : 0.0;
	            if (is_up) {
	              plb[j] = new_value;
	            } else {
	              pub[j] = new_value;
	            }
	            auto restore_workspace_scalar = [&]() {
	              if (is_up) {
	                plb[j] = old_value;
	              } else {
	                pub[j] = old_value;
	              }
	            };

	            if (auto* pipm = ensure_root_heur_ipm()) {
	              if (prepare_root_probe_ipm_solve(*pipm)) {
	                ++out.bc_stats.lp_solves;
	                auto res =
	                    pipm->solve_node_lp(base_lp, plb, pub, root.x_relax);
	                const std::string& status = res.stats.status;
	                const bool status_infeasible =
	                    status.find("Infeasible") != std::string::npos ||
	                    status.find("infeasible") != std::string::npos;
	                if (res.stats.success && res.x.size() == n &&
	                    std::isfinite(res.stats.objective)) {
	                  success = true;
	                  direction_bound = res.stats.objective;
	                } else if (res.stats.has_farkas_certificate ||
	                           status_infeasible) {
	                  proved_infeas = true;
	                }
	              }
	              restore_workspace_scalar();
	              return;
	            }

	            StandardFormLP& sf = ensure_root_probe_sf();
	            const std::vector<BoundChangeInfo> direction_change{
	                {j, 0.0, is_up, old_value, new_value}};
	            StandardFormBoundTransaction transaction(sf);
	            if (!transaction.apply(base_lp, direction_change)) {
	              ++out.bc_stats.root_lp_probe_transaction_failures;
	              restore_workspace_scalar();
	              invalidate_root_probe_sf();
	              return;
	            }
	            ++out.bc_stats.root_lp_probe_bound_transactions;
	            out.bc_stats.root_lp_probe_transaction_snapshot_values +=
	                transaction.snapshot_value_count();
	            ++out.bc_stats.lp_solves;
	            auto res = solve_lp_from_sf(sf, root_probe_simplex_opt,
	                                        root_probe_basis_hint());
	            if (res.result.stats.solver_name ==
	                "VendoredHighsPersistentLpKernel") {
	              ++out.bc_stats.root_lp_probe_backend_persistent_resolves;
	            } else {
	              ++out.bc_stats.root_lp_probe_backend_cold_solves;
	            }
	            if (res.basis.cached_sparse_basis) {
	              root_probe_basis = res.basis;
	            }
	            const auto type = classify_lp_result(res, n);
	            if (type == LPFailureType::Success &&
	                std::isfinite(res.result.stats.objective)) {
	              success = true;
	              direction_bound = res.result.stats.objective;
	            } else if (type == LPFailureType::Infeasible) {
	              proved_infeas = true;
	            }
	            if (transaction.rollback()) {
	              ++out.bc_stats.root_lp_probe_transaction_rollbacks;
	            } else {
	              ++out.bc_stats.root_lp_probe_transaction_failures;
	              invalidate_root_probe_sf();
	            }
	            restore_workspace_scalar();
	          };
	      run_root_probe_direction(/*is_up=*/false, down_success,
	                               down_proved_infeas, down_bound);

      bool up_success = false;
      bool up_proved_infeas = false;
      double up_bound = root.bound;
      if (bc_optional_root_budget_expired()) break;
	      run_root_probe_direction(/*is_up=*/true, up_success,
	                               up_proved_infeas, up_bound);

      ++probed;

	      if (down_proved_infeas && up_success) {
	        commit_root_probe_bound(j, /*is_lb=*/true, 1.0);
	        ++fixed_by_probe;
	      } else if (up_proved_infeas && down_success) {
	        commit_root_probe_bound(j, /*is_lb=*/false, 0.0);
	        ++fixed_by_probe;
      } else if (down_success && up_success) {
        const double frac_j = root.x_relax[j] - std::floor(root.x_relax[j]);
        pc[j].add_down(normalized_pseudocost_gain(
            std::max(0.0, down_bound - probe_base_bound), frac_j));
        pc[j].add_up(normalized_pseudocost_gain(
            std::max(0.0, up_bound - probe_base_bound), 1.0 - frac_j));
        if (split_bound_probe_root) {
          const double split_bound = std::min(down_bound, up_bound);
          if (std::isfinite(split_bound) && split_bound > best_split_bound) {
            best_split_bound = split_bound;
            best_split_var = j;
          }
        }

        // A probe bound that exceeds the incumbent is an objective-cutoff
        // certificate for that child, not an infeasibility proof for the
        // original root domain.  HiGHS keeps this as local proof/tree state.
        // Do not tighten root.lb/root.ub here: doing so would make the
        // re-solved restricted LP look like a global root lower bound and can
        // falsely close the MIP gap when the incumbent is still weak.
      }
    }
    if (split_bound_probe_root) {
      const double probe_elapsed_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - t_probe_start)
              .count();
      out.bc_stats.root_split_bound_probe_vars =
          static_cast<std::uint64_t>(std::max(0, probed));
      out.bc_stats.root_split_bound_probe_fixings =
          static_cast<std::uint64_t>(std::max(0, fixed_by_probe));
      out.bc_stats.root_split_bound_probe_time_ms = probe_elapsed_ms;
      out.bc_stats.root_split_bound_probe_lift =
          std::max(0.0, best_split_bound - probe_base_bound);
    }

    if (best_split_bound > root.bound + 1e-7) {
      root.bound = best_split_bound;
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C ROOT] split-bound probing: var={} strengthened root LB to {:.2f}\n",
                   best_split_var, root.bound);
      }
    }

    if (opt.verbose) {
      fmt::print(stderr, "[B&C PROBE] LP probing: probed={} fixed={} cands={} split_lb={:.2f}\n",
                 probed, fixed_by_probe, static_cast<int>(probe_cands.size()),
                 best_split_bound);
    }

    // Update root node with tightened bounds.
	    if (root_domain_probe_changed_root || fixed_by_probe > 0) {
      // Re-solve root LP with new fixed variables.
      ++out.bc_stats.lp_solves;
      if (auto* probe_ipm = ensure_root_heur_ipm()) {
        auto res = probe_ipm->solve_node_lp(base_lp, root.lb, root.ub, root.x_relax);
        if (res.stats.success && res.x.size() == n) {
          root.x_relax = clamp_to_bounds(res.x, root.lb, root.ub);
          root.x_seed = root.x_relax;
          root.bound = std::max(res.stats.objective, best_split_bound);
        }
      } else {
        update_standard_form_bounds(ensure_probe_sf(), base_lp, root.lb, root.ub);
        auto res = solve_lp_from_sf(ensure_probe_sf(), probe_opt, root.basis_hint.get());
        if (res.result.stats.success && res.result.x.size() == n) {
          root.x_relax = clamp_to_bounds(res.result.x, root.lb, root.ub);
          root.x_seed = root.x_relax;
          root.bound = std::max(res.result.stats.objective, best_split_bound);
          auto root_basis = std::make_shared<SimplexBasis>(std::move(res.basis));
          if (root.basis_hint) {
            root_basis->try_share_indices_from(*root.basis_hint);
          }
          root_basis->compact_indices_storage();
          root_basis->cached_reduced_costs =
              std::make_shared<const Eigen::VectorXd>(std::move(res.reduced_costs));
          root.basis_hint = root_basis;
        }
      }

      // Rebuild base_sf with the updated bounds.
      persistent_lp_state.invalidate_base_sf();
      (void)persistent_lp_state.ensure_base_sf(base_lp);
    }

    // Reduced cost fixing at root: if we have an incumbent, use reduced costs
    // to fix integer variables that can't improve the objective.
	    const double root_upper_limit_for_proof =
	        has_incumbent
	            ? highs_style_incumbent_upper_limit(base_lp, incumbent_obj,
	                                               opt.int_tol)
	            : kInf;
	    if (has_incumbent && std::isfinite(root_upper_limit_for_proof) &&
	        root_relax.simplex &&
	        opt.enable_reduced_cost_fixing &&
	        (opt.enable_reduced_cost_proof_cut_resolve ||
	         opt.enable_reduced_cost_proof_conflict_minimization)) {
      SimplexResult proof_simplex = *root_relax.simplex;
      if (proof_simplex.result.constraint_duals.size() != base_lp.A.rows() + base_lp.Aeq.rows() &&
          root_relax.row_duals.size() == base_lp.A.rows() + base_lp.Aeq.rows()) {
        proof_simplex.result.constraint_duals = root_relax.row_duals;
      }
	      DualProofRow full_proof;
	      const bool proof_ok = build_full_dual_proof_row(
	          base_lp, proof_simplex, root.lb, root.ub, root.x_relax,
	          root.bound, root_upper_limit_for_proof, full_proof,
	          std::max(1e-9, opt.lp_tol));
      if (opt.verbose) {
        if (proof_ok) {
          fmt::print(stderr,
                     "[B&C-DUALPROOF] full_row ok nnz={} rowdual={} ignrow={} "
                     "rm={} tight={} "
                     "min={:.6f} rhs={:.6f} max={:.6f} lp_gap={:.6f} "
                     "iderr={:.2e} staterr={:.2e} sfstat={:.2e} costmap={:.2e} src={}\n",
                     full_proof.nnz, full_proof.row_dual_nnz,
                     full_proof.ignored_row_duals,
                     full_proof.removed_bound_coefficients,
                     full_proof.tightened_coefficients,
                     full_proof.min_activity, full_proof.rhs,
                     full_proof.max_activity, full_proof.actual_lp_gap,
                     full_proof.gap_error,
                     full_proof.max_stationarity_error,
                     full_proof.max_scaled_stationarity_error,
                     full_proof.max_cost_mapping_error,
                     full_proof.used_solver_row_duals ? "solver" : "basis");
        } else {
          fmt::print(stderr, "[B&C-DUALPROOF] full_row skipped reason={}\n",
                     full_proof.reject_reason);
        }
      }
		      if (proof_ok) {
		        root_global_dual_proof = full_proof;
		        root_global_dual_proof_upper = root_upper_limit_for_proof;
	        have_root_global_dual_proof = true;
	        ++root_dual_proof_epoch;
        const auto proof_artifacts = extract_dual_proof_literal_artifacts(
            base_lp, root_implied_integer_cols, full_proof, root.lb, root.ub,
            implication_graph, clique_table, opt.int_tol, &conflict_pool,
            cts ? &cts->shared_conflicts : nullptr,
            &ensure_branch_priority());
        if (proof_artifacts.implications_added > 0 ||
            proof_artifacts.clique_edges_added > 0 ||
            proof_artifacts.conflict_clauses_added > 0) {
          out.bc_stats.rc_binary_implications_learned +=
              proof_artifacts.implications_added;
	          note_global_domain_learning(proof_artifacts.implications_added +
	                                      proof_artifacts.clique_edges_added +
	                                      proof_artifacts.conflict_clauses_added);
	          note_objective_artifact_learning(
	              proof_artifacts.implications_added +
	              proof_artifacts.clique_edges_added);
	          (void)refresh_objective_domain_sources(
	              "root_dual_proof_artifacts", /*force=*/true);
	        }
        if (bc_conf_diag_enabled() || trace_bc_timeline) {
          fmt::print(stderr,
                     "[B&C PROOF-IMPL] root cand={} conflicts={} confQ={} impl={} "
                     "clique_edges={} cap={:.6g} max_delta={:.6g} "
                     "frontier_pri={} obj_parts={} arcs={}\n",
                     proof_artifacts.candidates,
                     proof_artifacts.conflicts_seen,
                     proof_artifacts.conflict_clauses_added,
                     proof_artifacts.implications_added,
                     proof_artifacts.clique_edges_added,
                     proof_artifacts.proof_capacity,
                     proof_artifacts.max_delta,
                     proof_artifacts.frontier_priority_updates,
                     objective_propagation.partitions.size(),
                     implication_graph.size());
        }
	        cached_queue_dual_proof = full_proof;
	        cached_queue_dual_proof_upper = root_upper_limit_for_proof;
        have_cached_queue_dual_proof = true;
        cached_queue_dual_proof_is_global = true;
        cached_queue_dual_proof_scope.clear();
        cached_queue_objective_event_proof = full_proof;
        have_cached_queue_objective_event_proof = true;
        cached_queue_objective_event_proof_is_global = true;
		        cached_queue_objective_event_proof_upper = root_upper_limit_for_proof;
	        cached_queue_objective_event_proof_scope.clear();
        DualProofRow root_rc_mass_proof;
	        if (build_reduced_cost_mass_dual_proof_row(
	                base_lp, proof_simplex, root.lb, root.ub, root.x_relax,
	                root.bound, root_upper_limit_for_proof, root_rc_mass_proof,
	                std::max(1e-9, opt.lp_tol), nullptr,
	                &root_implied_integer_cols,
	                &objective_propagation.implied_event_target_cols)) {
	          cached_queue_dual_proof = root_rc_mass_proof;
	          cached_queue_dual_proof_upper = root_upper_limit_for_proof;
          have_cached_queue_dual_proof = true;
          cached_queue_dual_proof_is_global = true;
          cached_queue_dual_proof_scope.clear();
        }
        run_root_incumbent_propagation("after_root_dual_proof");
        // A full dual proof row is derived from the incumbent cutoff.  It is a
        // first-class proof object for local domains and queued frontier nodes,
        // but consuming it as root-domain fixing changes the global root LP
        // into the "solutions that can beat the incumbent" subproblem.  The
        // resulting restricted LP bound is not the original root lower bound,
        // so it must not be used to close the root gap.
      }
    }

				    if (root_current_redcost_domain_commit_allowed &&
			          opt.enable_reduced_cost_fixing &&
			        has_incumbent && std::isfinite(root_upper_limit_for_proof) &&
			        !root.lp_refresh_needed &&
			        root.basis_hint && root.basis_hint->cached_reduced_costs &&
			        root.basis_hint->cached_reduced_costs->size() >= n) {
      const Eigen::VectorXd rc_root_lb_before = root.lb;
      const Eigen::VectorXd rc_root_ub_before = root.ub;
      const auto rc_root_basis_before = root.basis_hint;
      const auto rc_root_x_before = root.x_relax;
      const double rc_root_bound_before = root.bound;
      const LPRelaxationResult rc_root_relax_before = root_relax;
      std::vector<DomainReasonBound> root_current_redcost_reasons;
	      int root_fixed = reduced_cost_fixing(
		          base_lp.vars, root.x_relax, *root.basis_hint->cached_reduced_costs,
		          root.basis_hint->basis_indices(), root.bound,
		          root_upper_limit_for_proof,
		          opt.int_tol, root.lb, root.ub, nullptr,
	          root.basis_hint->cached_col_scale.get(),
          &root_current_redcost_reasons, nullptr, /*reason_depth=*/0,
          static_cast<int>(root.domain_reason_bounds.size()));
      if (root_fixed > 0) {
        out.bc_stats.rc_fixings += static_cast<std::uint64_t>(root_fixed);
        out.bc_stats.root_cut_domain_tightenings +=
            static_cast<std::uint64_t>(root_fixed);
        update_standard_form_bounds(ensure_probe_sf(), base_lp, root.lb, root.ub);
        auto rc_res = std::make_shared<SimplexResult>(
            solve_lp_from_sf(ensure_probe_sf(), probe_opt,
                             rc_root_basis_before.get()));
        ++out.bc_stats.lp_solves;
        ++out.bc_stats.rc_fixing_resolve_lps;
        LPRelaxationResult rc_accept_relax;
        bool rc_accept = false;
        if (rc_res->result.stats.success && rc_res->result.x.size() == n &&
            std::isfinite(rc_res->result.stats.objective)) {
          rc_accept_relax.primal = rc_res->result;
          rc_accept_relax.dual_bound = rc_res->result.stats.objective;
          rc_accept_relax.simplex = rc_res;
          auto root_basis = std::make_shared<SimplexBasis>(rc_res->basis);
          if (rc_root_basis_before) {
            root_basis->try_share_indices_from(*rc_root_basis_before);
          }
          root_basis->compact_indices_storage();
          root_basis->cached_reduced_costs =
              std::make_shared<const Eigen::VectorXd>(rc_res->reduced_costs);
          rc_accept_relax.basis_hint = root_basis;
          rc_accept = true;
        } else if (requested_vendored_highs_lp) {
          LPModel rc_fallback_lp = base_lp;
          apply_node_bounds(rc_fallback_lp.vars, root.lb, root.ub);
          rc_accept_relax = solve_lp_relaxation(
              rc_fallback_lp, &rc_root_x_before, nullptr, root_solve_opt);
          ++out.bc_stats.lp_solves;
          ++out.bc_stats.rc_fixing_resolve_lps;
          rc_accept = rc_accept_relax.primal.stats.success &&
                      rc_accept_relax.primal.x.size() == n &&
                      std::isfinite(rc_accept_relax.dual_bound);
          if (opt.verbose ||
              bc_env_flag_enabled("MIPSOLVERS_BC_ROOT_RC_TRACE")) {
            fmt::print(stderr,
                       "[B&C ROOT-RC] warm re-solve failed status={}; "
                       "fallback status={} ok={}\n",
                       rc_res->result.stats.status,
                       rc_accept_relax.primal.stats.status,
                       rc_accept ? 1 : 0);
          }
        } else if (opt.verbose ||
                   bc_env_flag_enabled("MIPSOLVERS_BC_ROOT_RC_TRACE")) {
          fmt::print(stderr,
                     "[B&C ROOT-RC] warm re-solve failed status={}; "
                     "fallback disabled without vendored LP\n",
                     rc_res->result.stats.status);
        }
	        if (rc_accept) {
          for (auto& rb : root_current_redcost_reasons) {
            append_domain_reason_bound(root, std::move(rb), n,
                                       &rc_root_lb_before, &rc_root_ub_before);
          }
	          root.x_relax = clamp_to_bounds(rc_accept_relax.primal.x, root.lb, root.ub);
          root.x_seed = root.x_relax;
          root.bound = rc_accept_relax.dual_bound;
          root_relax = std::move(rc_accept_relax);
          root.basis_hint = root_relax.basis_hint;
          if (root.basis_hint) root.basis_hint->compact_indices_storage();
	          if (root_relax.simplex && root_relax.simplex->form.A.rows() == 0) {
	            root_relax.simplex->form = ensure_probe_sf();
	          }
	          refresh_root_redcost_lurking_bounds("root_rc_fixing_resolve");
	          if (root.bound > rc_root_bound_before +
                  std::max(1e-7, opt.root_cut_min_bound_lift)) {
            ++out.bc_stats.rc_fixing_resolve_bound_improvements;
          }
          if (opt.verbose ||
              bc_env_flag_enabled("MIPSOLVERS_BC_ROOT_RC_TRACE")) {
            fmt::print(stderr,
                       "[B&C ROOT-RC] fixed={} bound={:.2f}->{:.2f} "
                       "resolve_status={}\n",
                       root_fixed, rc_root_bound_before, root.bound,
                       root_relax.primal.stats.status);
          }
        } else {
          root.lb = rc_root_lb_before;
          root.ub = rc_root_ub_before;
          root.basis_hint = rc_root_basis_before;
          root.x_relax = rc_root_x_before;
          root.bound = rc_root_bound_before;
          root_relax = rc_root_relax_before;
          out.bc_stats.rc_fixings =
              out.bc_stats.rc_fixings > static_cast<std::uint64_t>(root_fixed)
                  ? out.bc_stats.rc_fixings -
                        static_cast<std::uint64_t>(root_fixed)
                  : 0;
          out.bc_stats.root_cut_domain_tightenings =
              out.bc_stats.root_cut_domain_tightenings >
                      static_cast<std::uint64_t>(root_fixed)
                  ? out.bc_stats.root_cut_domain_tightenings -
                        static_cast<std::uint64_t>(root_fixed)
                  : 0;
          if (opt.verbose ||
              bc_env_flag_enabled("MIPSOLVERS_BC_ROOT_RC_TRACE")) {
            fmt::print(stderr,
                       "[B&C ROOT-RC] re-solve failed after fixed={}; "
                       "reverted root reduced-cost fixing\n",
                       root_fixed);
          }
        }
      }
    }
  }

  [[maybe_unused]] auto t_probe_done = std::chrono::steady_clock::now();
  const bool root_tree_setup_budget_expired =
      bc_optional_root_budget_expired();
  if (opt.verbose) {
    fmt::print(stderr, "[B&C ROOT] probing={:.1f}ms has_incumbent={} (total root={:.1f}ms)\n",
            std::chrono::duration<double, std::milli>(t_probe_done - t_dive_done).count(),
            has_incumbent ? 1 : 0,
            std::chrono::duration<double, std::milli>(t_probe_done - t0).count());
  }

	  NodeQueue node_queue(opt.node_sel);
  NodePayloadSharingStats serial_payload_sharing;
  node_queue.configure_domain_signature(declared_branchable_cols, root.lb,
                                        root.ub, std::max(1e-9, opt.int_tol));
	  // For concurrent mode, root was already pushed to cts->shared_queue.
	  if (!concurrent_tree_launched) {
    // A queued node already owns x_relax; retaining an identical x_seed costs
    // another 8 bytes per column and is unnecessary for basis warm starts.
    root.x_seed.resize(0);
    node_queue.push(root, root_incumbent_good_enough());
  }

  double global_lb = node_queue.lower_bound();
  if (!std::isfinite(global_lb)) {
    global_lb = root.bound;
  }
  out.bc_stats.best_bound = global_lb;
  note_bound_event(has_incumbent ? incumbent_obj : kInf, global_lb,
                   /*retain_incumbent=*/false);

  auto hybrid_queue_mode_active = [&]() -> bool {
    return root_incumbent_good_enough();
  };

		  auto proof_tail_queue_mode_active = [&]() -> bool {
		    if (!has_incumbent || out.bc_stats.last_incumbent_node < 0) {
		      return false;
		    }
		    if (!std::isfinite(incumbent_obj) || !std::isfinite(root.bound) ||
		        !std::isfinite(opt.gap_tol) || opt.gap_tol < 0.0) {
		      return false;
		    }
		    return current_incumbent_valid_for_proof("proof_tail_queue_mode");
		  };

	  auto dual_proof_has_objective_event_target =
	      [&](const DualProofRow* proof) {
    if (proof == nullptr ||
        objective_propagation.implied_event_target_cols.empty()) {
      return false;
    }
    for (Eigen::SparseVector<double>::InnerIterator it(proof->coeff); it; ++it) {
      const int j = static_cast<int>(it.index());
      if (j >= 0 && j < n && std::abs(it.value()) > 1e-12 &&
          objective_propagation
              .implied_event_target_cols[static_cast<std::size_t>(j)]) {
        return true;
      }
	    }
	    return false;
	  };

	  auto scope_active_for_node =
	      [&](const Node& node, const std::vector<BranchDomainLiteral>& scope) {
	    const double tol = std::max(1e-9, opt.int_tol);
	    for (const auto& lit : scope) {
	      if (lit.var_idx < 0 || lit.var_idx >= n ||
	          lit.var_idx >= node.lb.size() || lit.var_idx >= node.ub.size()) {
	        return false;
	      }
	      const bool active = lit.is_lb
	          ? (node.lb[lit.var_idx] >= lit.value - tol)
	          : (node.ub[lit.var_idx] <= lit.value + tol);
	      if (!active) return false;
	    }
	    return true;
	  };

	  auto objective_event_proof_usable = [&](const DualProofRow* proof) {
	    return proof != nullptr && proof->valid && proof->coeff.size() >= n &&
	           proof->coeff.nonZeros() > 0 &&
	           dual_proof_has_objective_event_target(proof);
	  };

  auto dual_proof_domain_lower_bound =
      [&](const DualProofRow* proof, double proof_upper,
          const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) -> double {
    if (proof == nullptr || !proof->valid || proof->coeff.size() < n ||
        proof->coeff.nonZeros() <= 0 || !std::isfinite(proof_upper) ||
        !std::isfinite(proof->rhs) || lb.size() < n || ub.size() < n) {
      return kInf;
    }
    double min_activity = 0.0;
    for (Eigen::SparseVector<double>::InnerIterator it(proof->coeff); it;
         ++it) {
      const int j = static_cast<int>(it.index());
      const double a = it.value();
      if (j < 0 || j >= n || !std::isfinite(a)) return kInf;
      if (a > 0.0) {
        if (!std::isfinite(lb[j])) return kInf;
        min_activity += a * lb[j];
      } else if (a < 0.0) {
        if (!std::isfinite(ub[j])) return kInf;
        min_activity += a * ub[j];
      }
    }
    if (!std::isfinite(min_activity)) return kInf;
    return proof_upper - proof->rhs + min_activity;
  };

  auto objective_event_proof_upper = [&](const DualProofRow* proof) -> double {
    if (proof == nullptr) return kInf;
    if (have_cached_queue_objective_event_proof &&
        proof == &cached_queue_objective_event_proof &&
        std::isfinite(cached_queue_objective_event_proof_upper)) {
      return cached_queue_objective_event_proof_upper;
    }
    if (have_root_global_dual_proof && proof == &root_global_dual_proof &&
        std::isfinite(root_global_dual_proof_upper)) {
      return root_global_dual_proof_upper;
    }
    if (have_cached_queue_dual_proof &&
        proof == &cached_queue_dual_proof &&
        std::isfinite(cached_queue_dual_proof_upper)) {
      return cached_queue_dual_proof_upper;
    }
    return kInf;
  };

	  auto select_objective_event_proof_for_node =
	      [&](const Node& node, bool require_proof_tail) -> const DualProofRow* {
	    if (require_proof_tail && !proof_tail_queue_mode_active()) {
	      return nullptr;
	    }
	    // HiGHS consumes objective/cutoff propagation in the current local
	    // domain closure.  Prefer the scoped node LP proof when its trail scope
	    // is active; a root/global proof is only a fallback oracle.  Using the
	    // root proof first inflates the proof capacity on non-root frontiers and
	    // turns real one-missing objective events into skip_budget no-ops.
	    if (have_cached_queue_objective_event_proof &&
	        !cached_queue_objective_event_proof_is_global &&
	        objective_event_proof_usable(&cached_queue_objective_event_proof) &&
	        scope_active_for_node(node, cached_queue_objective_event_proof_scope)) {
	      return &cached_queue_objective_event_proof;
	    }
	    if (have_cached_queue_objective_event_proof &&
	        cached_queue_objective_event_proof_is_global &&
	        objective_event_proof_usable(&cached_queue_objective_event_proof)) {
	      return &cached_queue_objective_event_proof;
	    }
	    if (have_root_global_dual_proof &&
	        objective_event_proof_usable(&root_global_dual_proof)) {
	      return &root_global_dual_proof;
	    }
	    return nullptr;
	  };

	  auto termination_bound_band = [&]() {
    const double base =
        std::isfinite(global_lb) ? std::abs(global_lb)
                                 : std::max(1.0, std::abs(root.bound));
    return std::max(1e-6, 1e-4 * std::max(1.0, base));
  };
  auto valid_pool_cut =
      [&](const PoolCut& cut, const Eigen::VectorXd* x,
          const char* source) -> bool {
    bool ok = cut.coeff.size() == n &&
              cut.coeff.nonZeros() > 0 &&
              std::isfinite(cut.rhs);
    for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); ok && it;
         ++it) {
      const int j = static_cast<int>(it.index());
      ok = j >= 0 && j < n && std::isfinite(it.value());
    }
    if (ok && x != nullptr) {
      ok = x->size() == n && std::isfinite(sparse_dot(cut.coeff, *x));
    }
    if (!ok) {
      ++out.bc_stats.invalid_cut_coefficients;
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C-CUT-VALIDATION] rejected {} coeff_size={} nnz={} "
                   "rhs={:.6g} expected_n={}\n",
                   source != nullptr ? source : "cut",
                   cut.coeff.size(), cut.coeff.nonZeros(), cut.rhs, n);
      }
    }
    return ok;
  };

  auto mark_cut_scope = [](PoolCut& cut, ValidityScope scope) {
    cut.validity_scope = scope;
  };

  auto make_local_node_cut = [&](PoolCut&& cut) -> PoolCut {
    mark_cut_scope(cut, ValidityScope::LocalNodeCut);
    return std::move(cut);
  };

  auto append_unique_local_node_cut =
      [&](Node& node, PoolCut&& cut, const Eigen::VectorXd* x,
          const char* source) -> bool {
    mark_cut_scope(cut, ValidityScope::LocalNodeCut);
    if (!valid_pool_cut(cut, x, source)) return false;
    const std::size_t h = cut.hash != 0 ? cut.hash : sparse_cut_hash(cut.coeff);
    for (const auto& existing : node.local_cuts) {
      const std::size_t eh =
          existing.hash != 0 ? existing.hash : sparse_cut_hash(existing.coeff);
      if (eh == h && sparse_cut_identical(existing.coeff, existing.rhs,
                                          cut.coeff, cut.rhs)) {
        return false;
      }
    }
    cut.hash = h;
    node.local_cuts.push_back(std::move(cut));
    ++out.bc_stats.local_node_cuts_added;
    return true;
  };

  auto append_local_node_cuts =
      [&](Node& node, std::vector<PoolCut>& cuts, const Eigen::VectorXd* x,
          const char* source) -> int {
    int added = 0;
    for (auto& cut : cuts) {
      if (append_unique_local_node_cut(node, std::move(cut), x, source)) {
        ++added;
      }
    }
    cuts.clear();
    return added;
  };

  auto contains_identical_local_cut =
      [&](const Node& node, const std::vector<PoolCut>& pending,
          const Eigen::SparseVector<double>& row, double rhs) {
        const std::size_t hash = sparse_cut_hash(row);
        const auto contains = [&](const std::vector<PoolCut>& cuts) {
          for (const PoolCut& cut : cuts) {
            const std::size_t cut_hash =
                cut.hash != 0 ? cut.hash : sparse_cut_hash(cut.coeff);
            if (cut_hash == hash &&
                sparse_cut_identical(cut.coeff, cut.rhs, row, rhs)) {
              return true;
            }
          }
          return false;
        };
        return contains(node.local_cuts) || contains(pending);
      };

  auto build_node_lp_with_local_cuts = [&](const Node& node) -> LPModel {
    LPModel node_lp = base_lp;
    if (!node.local_cuts.empty()) {
      std::vector<Eigen::SparseVector<double>> rows;
      std::vector<double> rhs;
      rows.reserve(node.local_cuts.size());
      rhs.reserve(node.local_cuts.size());
      for (const PoolCut& cut : node.local_cuts) {
        if (cut.validity_scope != ValidityScope::LocalNodeCut) continue;
        if (!valid_pool_cut(cut, nullptr, "node_local_cut_build")) continue;
        rows.push_back(cut.coeff);
        rhs.push_back(cut.rhs);
      }
      if (!rows.empty()) add_sparse_rows_to_lp(node_lp, rows, rhs);
    }
    apply_node_bounds(node_lp.vars, node.lb, node.ub);
    return node_lp;
  };

  // Main tree LPs must start from a root-clean standard form.  Root heuristics
  // and probes reuse PersistentLPState scratch space and repeatedly rewrite
  // its bounds; using that object as the live tree LP lets sibling/probe state
  // leak into child bounds.  HiGHS keeps the active node LP state as a
  // first-class object and applies each node's domain before solving it.  Mirror
  // that here by keeping an immutable tree base and deriving node-local SF
  // workspaces from it.
  StandardFormLP tree_base_sf;
  StandardFormLP node_sf;
  if (!root_tree_setup_budget_expired) {
    tree_base_sf = build_standard_form_lp(base_lp);
    ruiz_scale_standard_form(tree_base_sf);
    update_standard_form_bounds(tree_base_sf, base_lp, root.lb, root.ub);
    node_sf = tree_base_sf;
  }
  // Shared node-SF workspace hygiene: process_single_child and the refresh/
  // replay paths reuse node_sf, and cuts-augmented solves replace it with a
  // locally-augmented SF.  This flag tracks that state so the next child can
  // restore the pristine tree base exactly once (avoids a full SF copy per
  // child in the common no-cuts path).
  bool node_sf_cuts_augmented = false;
  auto build_node_sf_with_local_cuts = [&](const Node& node) -> StandardFormLP {
    if (node.local_cuts.empty()) {
      StandardFormLP local_sf = tree_base_sf;
      update_standard_form_bounds(local_sf, base_lp, node.lb, node.ub);
      return local_sf;
    }
    LPModel node_lp = build_node_lp_with_local_cuts(node);
    StandardFormLP local_sf = build_standard_form_lp(node_lp);
    ruiz_scale_standard_form(local_sf);
    update_standard_form_bounds(local_sf, node_lp, node.lb, node.ub);
    return local_sf;
  };
  auto build_node_sf_with_bounds =
      [&](const Node& node, const Eigen::VectorXd& lb,
          const Eigen::VectorXd& ub) -> StandardFormLP {
    if (node.local_cuts.empty()) {
      StandardFormLP local_sf = tree_base_sf;
      update_standard_form_bounds(local_sf, base_lp, lb, ub);
      return local_sf;
    }
    Node tmp = node;
    tmp.lb = lb;
    tmp.ub = ub;
    return build_node_sf_with_local_cuts(tmp);
  };
  auto node_local_validity_scope =
      [&](const Node& node) -> std::vector<BranchDomainLiteral> {
    std::vector<BranchDomainLiteral> scope;
    const double tol = std::max(1e-9, opt.int_tol);
    if (node.lb.size() != n || node.ub.size() != n || root.lb.size() != n ||
        root.ub.size() != n) {
      scope = node.branch_reasons;
      canonicalize_branch_literals(scope);
      return scope;
    }
    scope.reserve(static_cast<std::size_t>(2 * n));
    for (int j = 0; j < n; ++j) {
      if (std::isfinite(node.lb[j]) &&
          (!std::isfinite(root.lb[j]) || node.lb[j] > root.lb[j] + tol)) {
        scope.push_back(BranchDomainLiteral{j, node.lb[j], true});
      }
      if (std::isfinite(node.ub[j]) &&
          (!std::isfinite(root.ub[j]) || node.ub[j] < root.ub[j] - tol)) {
        scope.push_back(BranchDomainLiteral{j, node.ub[j], false});
      }
    }
    canonicalize_branch_literals(scope);
    return scope;
  };
  // Returns the LP to use for proof construction.  When the node has no
  // local cuts this aliases base_lp (zero copy); otherwise the augmented
  // model is built into the caller-provided scratch.
  auto active_lp_for_proof = [&](const Node& node, LPModel& scratch) -> const LPModel& {
    return node.local_cuts.empty() ? base_lp
                                   : (scratch = build_node_lp_with_local_cuts(node));
  };
  const double root_domain_bound_floor =
      std::isfinite(root.bound) ? root.bound : -kInf;

  // Atomic incumbent bound kept for legacy sub-solves. Main tree node LPs must
  // finish with a basis/dual state so bound-exceeding prunes can be published as
  // HiGHS-style dual-proof conflicts.
  std::atomic<double> seq_incumbent_bound{has_incumbent ? incumbent_obj : kInf};

  SimplexOptions simplex_opt;
  simplex_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
  simplex_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
  simplex_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
  simplex_opt.allow_cold_start = false;
  simplex_opt.incumbent_bound = nullptr;
  simplex_opt.incumbent_check_interval = 0;
  simplex_opt.factor_backend = simplex_factor_backend_from_id(opt.simplex_factor_backend);
  simplex_opt.lp_kernel_backend = opt.lp_kernel_backend;
  apply_bc_first_class_simplex_state(simplex_opt, /*allow_frontier_remap=*/true);
  bool simplex_node_deadline_hit = false;
  auto solve_tree_lp = [&](const StandardFormLP& sf,
                           SimplexOptions solve_opt,
                           const SimplexBasis* basis_hint) {
    const double remaining =
        bc_remaining_sec() - bc_post_root_finalize_reserve_sec;
    if (remaining <= 0.001) simplex_node_deadline_hit = true;
    solve_opt.time_limit_sec = std::max(0.001, remaining);
    solve_opt.time_limit_hit = &simplex_node_deadline_hit;
    SimplexResult result = solve_lp_from_sf(sf, solve_opt, basis_hint);
    if (lp_wall_time_limit_reached(result)) {
      simplex_node_deadline_hit = true;
    }
    return result;
  };

  // Pre-compute row-major sparse matrices for bound propagation (avoids
  // O(nnz) format conversion at every node).
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_cached;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row_cached;
  RowPropagationIndex row_index_cached;
  if (!root_tree_setup_budget_expired) {
    A_row_cached = base_lp.A;
    Aeq_row_cached = base_lp.Aeq;
    row_index_cached.build(n, A_row_cached, Aeq_row_cached);
  }

  // Determine early if IPM fallback will be used for node LPs.
  // When simplex has a root basis (from crossover), simplex is primary solver
  // and IPM serves as fallback for failures. When no root basis exists, IPM
  // is the primary solver and no fallback is needed.
  const bool use_ipm_primary = opt.use_ipm_nodes && !root.basis_hint;
  const bool has_ipm_fallback = !use_ipm_primary;

  // Sequential fallback infrastructure. The dispatcher owns the fallback
  // contract; this legacy manager retains the configured retry parameters
  // without making a kernel timing claim.
  FallbackConfig seq_fb_config;
  seq_fb_config.l1_max_retries = opt.fallback_l1_retries;
  seq_fb_config.l1_perturbation = opt.fallback_l1_perturbation;
  seq_fb_config.l3_max_retries = opt.fallback_l3_retries;
  seq_fb_config.l3_tol_multiplier = opt.fallback_l3_tol_mult;
  FallbackLogger seq_fallback_logger(t0);
  FallbackManager seq_fallback_mgr(seq_fb_config, seq_fallback_logger, simplex_opt);
  int seq_node_id = 0;

  // Unified solver dispatcher for sequential path.
  // Always enable the simplex fallback chain; failures are rare and the
  // chain handles them without changing the node proof contract.
  DispatcherConfig disp_config = DispatcherConfig::from_bc_options(opt);
  disp_config.allow_persistent_lp_state = true;
  if (bc_first_class_lp_state_conformance_enabled()) {
    disp_config.exact_dse_initialization = true;
    disp_config.enable_degenerate_frontier_remap = true;
    disp_config.suppress_degenerate_frontier_remap = false;
    disp_config.disable_partial_pricing_for_conformance = true;
  }
	  SolverDispatcher seq_dispatcher(disp_config, &seq_fallback_mgr, &seq_incumbent_bound);
		  if (root.basis_hint) {
		    seq_dispatcher.set_root_basis(root.basis_hint);
		  }
		  IncumbentTreeRestartController tree_restart_controller(
		      opt.tree_restart);
		  bool tree_incumbent_domain_closure_pending = false;
		  std::string tree_incumbent_domain_closure_source;
		  int tree_incumbent_domain_closure_depth = 0;
		  bool tree_incumbent_domain_closure_active = false;
		  auto adopt_tree_incumbent =
		      [&](const char* source, const Eigen::VectorXd& candidate_x,
		          double candidate_obj, int depth, double bound_snapshot) -> bool {
	    (void)candidate_obj;
	    const double actual_obj =
	        objective_value(base_lp.c, candidate_x, base_lp.sense);
	    if (!std::isfinite(actual_obj)) return false;
	    if (has_incumbent && !(actual_obj + 1e-9 < incumbent_obj)) {
	      return false;
	    }
	    if (candidate_x.size() != n) return false;
	    const auto reduced_validation =
	        validate_incumbent_solution(base_lp, candidate_x,
	                                     kNodeFeasibilityTol);
	    if (!reduced_validation.ok()) {
	      if (opt.verbose || trace_bc_timeline) {
	        fmt::print(stderr,
	                   "[B&C-INC-GATE] rejected source={} {}\n",
	                   source != nullptr ? source : "tree_incumbent",
	                   format_incumbent_validation_failure(
	                       "tree-proof-gate", base_lp,
	                       reduced_validation));
	      }
	      return false;
	    }
	    has_incumbent = true;
	    incumbent_x = candidate_x;
	    incumbent_obj = actual_obj;
	    note_incumbent_cutoff_learning();
	    incumbent_proof_validated = true;
	    incumbent_proof_validated_obj = incumbent_obj;
	    solution_pool.add(candidate_x, actual_obj);
		    seq_incumbent_bound.store(incumbent_obj, std::memory_order_relaxed);
		    note_incumbent_update(depth, bound_snapshot);
		    tree_restart_controller.note_incumbent(
		        incumbent_obj, out.bc_stats.nodes_explored, source);
		    certify_root_gap_frontier_incumbent(source, actual_obj);
		    tree_incumbent_domain_closure_pending = true;
		    tree_incumbent_domain_closure_source =
		        source != nullptr ? source : "tree_incumbent";
		    tree_incumbent_domain_closure_depth = depth;
		    return true;
		  };

	  // Use IPM for node LP solves only when crossover supplied no simplex
  // basis. With a basis, the dispatcher uses same-structure simplex resolves.
  const bool use_ipm_nodes = use_ipm_primary;
  std::unique_ptr<NativeIPMLPAdapter> ipm_node_solver;
  if (use_ipm_nodes && !root_tree_setup_budget_expired) {
    IPMLPOptions ipm_opt;
    // Scale max_iter with problem size: warm-start converges in ~15-20 iters
    // for m<15k, but larger problems (m=34k for uc_500g) can need ~30-50.
    ipm_opt.max_iter = (presolved_m > 25000) ? 200 : 100;
    ipm_opt.tol_primal = 1e-6;  // looser tol sufficient for B&C bound decisions
    ipm_opt.tol_dual = 1e-6;
    ipm_opt.tol_gap = 1e-6;
    ipm_opt.verbose = false;
    ipm_node_solver = std::make_unique<NativeIPMLPAdapter>(ipm_opt);
    ipm_node_solver->prepare_for_node_solves(base_lp);
  }

  // IPM fallback solver: used when simplex warm-start fails at tree nodes.
  std::unique_ptr<NativeIPMLPAdapter> ipm_fallback_solver;
  if (has_ipm_fallback && !root_tree_setup_budget_expired) {
    IPMLPOptions fb_ipm_opt;
    fb_ipm_opt.max_iter = (presolved_m > 25000) ? 200 : 100;
    fb_ipm_opt.tol_primal = 1e-6;
    fb_ipm_opt.tol_dual = 1e-6;
    fb_ipm_opt.tol_gap = 1e-6;
    fb_ipm_opt.verbose = false;
    ipm_fallback_solver = std::make_unique<NativeIPMLPAdapter>(fb_ipm_opt);
    ipm_fallback_solver->prepare_for_node_solves(base_lp);
  }

  // IPM adapter for pool-cut / child-cut re-solves.  These re-solves only
  // need a primal solution for bound tightening (the simplex basis from the
  // initial node LP is kept for tree exploration).  IPM avoids the expensive
  // dense basis inverse maintained by our simplex and skips the full
  // build_standard_form_lp rebuild.
  IPMLPOptions cut_ipm_opt;
  cut_ipm_opt.max_iter = 150;
  cut_ipm_opt.tol_primal = std::max(1e-8, opt.lp_tol);
  cut_ipm_opt.tol_dual = std::max(1e-8, opt.lp_tol);
  cut_ipm_opt.tol_gap = std::max(1e-8, opt.lp_tol);
  cut_ipm_opt.verbose = false;
  NativeIPMLPAdapter ipm_cut_solver(cut_ipm_opt);
  auto apply_tree_ipm_deadline = [&](NativeIPMLPAdapter& solver) {
    solver.set_solve_time_limit(std::max(
        0.001, bc_remaining_sec() - bc_post_root_finalize_reserve_sec));
  };

  [[maybe_unused]] double usf_total_us = 0, lp_total_us = 0;
  [[maybe_unused]] int timing_lp_count = 0;

  if (opt.verbose) {
    std::string inc_msg = has_incumbent ? fmt::format(" inc_obj={:.2f}", incumbent_obj) : "";
    MIPSOLVERS_LOG_DEBUG("[B&C] root_bound={:.2f}  has_incumbent={}{}  n={}",
            root.bound, has_incumbent ? 1 : 0, inc_msg, n);
  std::vector<int> frac_tmp;
  fractional_branchable_indices(root.x_relax, root.lb, root.ub, opt.int_tol,
                                frac_tmp);
    MIPSOLVERS_LOG_DEBUG("[B&C] root fractional vars: {}", static_cast<int>(frac_tmp.size()));
  }

  // Rounding LP attempt budget: stop trying after max_rounding_lp_attempts
  // unsuccessful attempts (converged LP but no integer-feasible solution).
  // For medium problems, use a smaller budget since rounding LPs are less
  // likely to succeed with stronger coupling constraints.
  int rounding_lp_attempts = 0;
  const int max_rounding_lp_attempts =
      large_generic_root ? 2 : ((presolved_m > 10000) ? 20 : 5);
  // Post-incumbent fixed-integer LP repair is intentionally disabled by
  // default. A focused validation on uc_100g_24t spent ~3.4s in four repair
  // LPs with no incumbent improvement, so this remains an instrumented
  // experiment rather than production search.
  int incumbent_repair_lps_left = 0;

		  auto apply_objective_cutoff_domain_fixing =
	      [&](Node& node,
	          double cutoff_snapshot,
	          bool has_inc_snapshot,
	          std::vector<PoolCut>& new_cuts,
	          int& cut_count,
	          int& n_conflicts_learned,
	          int& n_tightened,
	          int* n_reasons_added = nullptr) -> bool {
    n_tightened = 0;
    n_conflicts_learned = 0;
	    if (n_reasons_added != nullptr) *n_reasons_added = 0;
	    if (!opt.enable_objective_cutoff_domain_fixing ||
	        !has_inc_snapshot ||
	        base_lp.sense != Sense::Minimize ||
	        !std::isfinite(cutoff_snapshot)) {
	      return true;
	    }

	    int objective_state_tightenings = 0;
	    int objective_state_prunes = 0;
	    std::vector<DomainReasonBound> objective_state_reasons;
	    auto append_objective_state_reason = [&](DomainReasonBound rb) {
	      const std::size_t reason_hash = conflict_clause_hash(rb.reason);
	      for (const auto& existing : node.domain_reason_bounds) {
	        if (conflict_clause_hash(existing.reason) == reason_hash &&
	            domain_reason_bounds_equal(existing, rb,
	                                       std::max(1e-9, opt.int_tol))) {
	          return false;
	        }
	      }
	      append_domain_reason_bound(node, std::move(rb), n, &root.lb, &root.ub);
	      if (n_reasons_added != nullptr) ++(*n_reasons_added);
	      return true;
	    };
		    if (!objective_propagation.propagate(
		            base_lp, node.lb, node.ub, cutoff_snapshot, opt.int_tol,
		            node.depth,
	            static_cast<int>(node.branch_reasons.size() +
	                             node.domain_reason_bounds.size()),
	            opt.enable_reduced_cost_proof_conflict_minimization
	                ? &objective_state_reasons
	                : nullptr,
	            objective_state_tightenings, objective_state_prunes)) {
	      for (auto& rb : objective_state_reasons) {
	        (void)append_objective_state_reason(std::move(rb));
	      }
	      n_tightened += objective_state_tightenings;
	      out.bc_stats.cutoff_domain_tightenings +=
	          static_cast<std::uint64_t>(objective_state_tightenings);
      out.bc_stats.objective_implied_event_tightenings +=
          static_cast<std::uint64_t>(objective_state_tightenings);
      out.bc_stats.cutoff_domain_prunes +=
          static_cast<std::uint64_t>(std::max(1, objective_state_prunes));
      out.bc_stats.objective_implied_event_prunes +=
          static_cast<std::uint64_t>(std::max(1, objective_state_prunes));
	      return false;
	    }
	    for (auto& rb : objective_state_reasons) {
	      (void)append_objective_state_reason(std::move(rb));
	    }
	    n_tightened += objective_state_tightenings;
    out.bc_stats.objective_implied_event_tightenings +=
        static_cast<std::uint64_t>(objective_state_tightenings);

    auto add_literal = [](std::vector<BranchDomainLiteral>& clause,
                          BranchDomainLiteral lit) {
      for (auto& existing : clause) {
        if (existing.var_idx != lit.var_idx || existing.is_lb != lit.is_lb) continue;
        if (lit.is_lb) existing.value = std::max(existing.value, lit.value);
        else existing.value = std::min(existing.value, lit.value);
        return;
      }
      clause.push_back(lit);
    };
    auto learn_cutoff_clause = [&](std::vector<BranchDomainLiteral> clause) {
      canonicalize_branch_literals(clause);
      if (clause.empty() || clause.size() > 64) return;
      const bool pool_added = add_global_conflict(
          clause, "objective_cutoff_domain_fixing");
      const bool impl_added =
          add_binary_implications_from_conflict_clause(base_lp.vars,
                                                       clause,
                                                       implication_graph);
      if (pool_added || impl_added) {
        note_global_domain_learning(1 + (impl_added ? 1 : 0));
        ++n_conflicts_learned;
        ++out.bc_stats.cutoff_conflict_clauses_learned;
	        if (!strict_highs_lp_contract && clause.size() <= 16) {
	          PoolCut conflict_cut;
	          if (try_build_binary_conflict_cut(base_lp, clause, conflict_cut, 16)) {
	            conflict_cut.domain_only = true;
	            new_cuts.push_back(std::move(conflict_cut));
	            ++cut_count;
	            ++out.bc_stats.cutoff_conflict_cuts_added;
	          }
	        }
      }
    };
	    auto append_objective_cutoff_reason =
	        [&](BranchDomainLiteral bound,
	            std::vector<BranchDomainLiteral> reason,
	            BranchDomainLiteral source_conflict_literal,
	            double mandatory_cost) {
	      if (bound.var_idx < 0 || bound.var_idx >= n ||
	          !std::isfinite(bound.value)) {
	        return false;
	      }
	      canonicalize_branch_literals(reason);
	      DomainReasonBound rb;
	      rb.bound = bound;
	      rb.reason = std::move(reason);
	      rb.trail_pos = static_cast<int>(node.branch_reasons.size() +
	                                      node.domain_reason_bounds.size());
	      rb.depth = node.depth;
	      rb.source_conflict_literal = source_conflict_literal;
	      rb.has_source_conflict_literal = true;
	      rb.source_conflict_clause = rb.reason;
	      rb.source_conflict_clause.push_back(source_conflict_literal);
	      canonicalize_branch_literals(rb.source_conflict_clause);
	      rb.has_source_conflict_clause = true;
	      if (std::isfinite(mandatory_cost) &&
	          std::isfinite(objective_cutoff_rest_lb)) {
	        rb.has_proof_activity_audit = true;
	        rb.proof_activity =
	            objective_cutoff_rest_lb + std::max(0.0, mandatory_cost);
	        rb.proof_required_activity = cutoff_snapshot;
	        rb.proof_activity_margin =
	            rb.proof_activity - rb.proof_required_activity;
	      }
	      return append_objective_state_reason(std::move(rb));
	    };

    if (objective_cutoff_events.empty() ||
        !std::isfinite(objective_cutoff_rest_lb)) {
      if (n_tightened > 0) {
        out.bc_stats.cutoff_domain_tightenings +=
            static_cast<std::uint64_t>(n_tightened);
      }
      const bool ok = bounds_consistent(node.lb, node.ub);
      if (!ok) ++out.bc_stats.cutoff_domain_prunes;
      return ok;
    }

	    const double cutoff_budget = cutoff_snapshot - objective_cutoff_rest_lb;
    if (!(cutoff_budget >= 0.0) || !std::isfinite(cutoff_budget)) {
      ++out.bc_stats.cutoff_domain_prunes;
      return false;
    }

    std::vector<char> forced_s(static_cast<std::size_t>(n), 0);
    std::vector<BranchDomainLiteral> forced_event_literals;
    forced_event_literals.reserve(16);

    double forced_cost = 0.0;
    for (const auto& e : objective_cutoff_events) {
      if (e.s < 0 || e.s >= n || e.y < 0 || e.y >= n) continue;
      const bool y_forced_one = node.lb[e.y] >= 1.0 - 1e-9;
      const bool z_forced_zero = e.z < 0 || node.ub[e.z] <= 1e-9;
      if (!y_forced_one || !z_forced_zero) continue;
      if (forced_s[static_cast<std::size_t>(e.s)]) continue;
      forced_s[static_cast<std::size_t>(e.s)] = 1;
      forced_cost += e.cost;
      add_literal(forced_event_literals, BranchDomainLiteral{e.y, 1.0, true});
      if (e.z >= 0 && e.z < n) {
        add_literal(forced_event_literals, BranchDomainLiteral{e.z, 0.0, false});
      }
      if (forced_cost > cutoff_budget + 1e-7) {
        learn_cutoff_clause(forced_event_literals);
        ++out.bc_stats.cutoff_domain_prunes;
        return false;
      }
    }

    const double remaining = cutoff_budget - forced_cost;
    const double tol = 1e-7 * std::max(1.0, std::abs(cutoff_budget));
    for (const auto& e : objective_cutoff_events) {
      if (e.y < 0 || e.y >= n) continue;
      if (e.cost <= remaining + tol) continue;
      if (e.z < 0) {
        std::vector<BranchDomainLiteral> clause = forced_event_literals;
        add_literal(clause, BranchDomainLiteral{e.y, 1.0, true});
        learn_cutoff_clause(std::move(clause));
        if (node.lb[e.y] >= 1.0 - 1e-9) {
          ++out.bc_stats.cutoff_domain_prunes;
          return false;
        }
        if (node.ub[e.y] > 1e-9) {
	          std::vector<BranchDomainLiteral> reason = forced_event_literals;
          node.ub[e.y] = 0.0;
          ++n_tightened;
	          (void)append_objective_cutoff_reason(
	              BranchDomainLiteral{e.y, 0.0, false}, std::move(reason),
	              BranchDomainLiteral{e.y, 1.0, true},
	              forced_cost + e.cost);
        }
      } else if (e.z >= 0 && e.z < n) {
        const bool y_forced_one = node.lb[e.y] >= 1.0 - 1e-9;
        const bool z_forced_zero = node.ub[e.z] <= 1e-9;
        if (y_forced_one && z_forced_zero) {
          std::vector<BranchDomainLiteral> clause = forced_event_literals;
          add_literal(clause, BranchDomainLiteral{e.y, 1.0, true});
          add_literal(clause, BranchDomainLiteral{e.z, 0.0, false});
          learn_cutoff_clause(std::move(clause));
          ++out.bc_stats.cutoff_domain_prunes;
          return false;
        }
        if (y_forced_one && node.lb[e.z] < 1.0 - 1e-9) {
          std::vector<BranchDomainLiteral> clause = forced_event_literals;
          add_literal(clause, BranchDomainLiteral{e.y, 1.0, true});
          add_literal(clause, BranchDomainLiteral{e.z, 0.0, false});
          learn_cutoff_clause(std::move(clause));
	          std::vector<BranchDomainLiteral> reason = forced_event_literals;
	          add_literal(reason, BranchDomainLiteral{e.y, 1.0, true});
          node.lb[e.z] = 1.0;
          ++n_tightened;
	          (void)append_objective_cutoff_reason(
	              BranchDomainLiteral{e.z, 1.0, true}, std::move(reason),
	              BranchDomainLiteral{e.z, 0.0, false},
	              forced_cost + e.cost);
        } else if (z_forced_zero && node.ub[e.y] > 1e-9) {
          std::vector<BranchDomainLiteral> clause = forced_event_literals;
          add_literal(clause, BranchDomainLiteral{e.y, 1.0, true});
          add_literal(clause, BranchDomainLiteral{e.z, 0.0, false});
          learn_cutoff_clause(std::move(clause));
	          std::vector<BranchDomainLiteral> reason = forced_event_literals;
	          add_literal(reason, BranchDomainLiteral{e.z, 0.0, false});
          node.ub[e.y] = 0.0;
          ++n_tightened;
	          (void)append_objective_cutoff_reason(
	              BranchDomainLiteral{e.y, 0.0, false}, std::move(reason),
	              BranchDomainLiteral{e.y, 1.0, true},
	              forced_cost + e.cost);
        }
      }
    }
    if (n_tightened > 0) {
      out.bc_stats.cutoff_domain_tightenings +=
          static_cast<std::uint64_t>(n_tightened);
    }
    const bool ok = bounds_consistent(node.lb, node.ub);
    if (!ok) ++out.bc_stats.cutoff_domain_prunes;
    return ok;
  };

	  auto learn_cutoff_conflicts_from_forbidden_literals =
	      [&](const std::vector<BranchDomainLiteral>& branch_reasons,
	          const std::vector<BranchDomainLiteral>& forbidden_literals,
	          const Eigen::VectorXd& proof_node_lb,
	          const Eigen::VectorXd& proof_node_ub,
	          const Eigen::VectorXd& x_relax,
	          const Eigen::VectorXd& reduced_costs,
	          const std::vector<int>& basis_indices,
	          double node_bound,
	          double incumbent_obj,
	          const std::vector<DomainReasonBound>& reason_bounds,
	          const DualProofRow* dual_proof,
	          std::vector<PoolCut>* local_proof_cuts,
	          std::vector<PoolCut>& new_cuts,
	          int& cut_count,
	          int max_to_learn,
	          int& n_learned) {
	    n_learned = 0;
	    if (!opt.enable_reduced_cost_conflict_learning ||
	        forbidden_literals.empty() ||
	        max_to_learn <= 0) {
	      return;
	    }
	    if (strict_highs_lp_contract) {
	      return;
	    }
	    const int max_cut_literals =
	        std::max(2, opt.reduced_cost_conflict_cut_max_literals);
	    const int max_pool_literals =
	        std::max(max_cut_literals,
	                 opt.reduced_cost_conflict_pool_max_literals);
	      for (const auto& forbidden : forbidden_literals) {
	        if (n_learned >= max_to_learn) break;
	        if (bc_time_limit_expired()) break;
	        if (forbidden.var_idx < 0 || forbidden.var_idx >= n) continue;
	        std::vector<BranchDomainLiteral> clause;
	        const std::uint64_t full_clause_len =
	            static_cast<std::uint64_t>(branch_reasons.size() + 1);
	        bool proof_clause = false;
	        std::vector<std::vector<BranchDomainLiteral>> reconvergence_clauses;
	        if (opt.enable_reduced_cost_proof_conflict_minimization) {
	          double proof_margin = 0.0;
	          PoolCut proof_cover_cut;
	          proof_clause = build_reduced_cost_cutoff_conflict_clause(
	            base_lp.vars, root.lb, root.ub, proof_node_lb, proof_node_ub,
	            x_relax, reduced_costs, basis_indices, node_bound,
	            incumbent_obj, opt.int_tol, branch_reasons, reason_bounds, forbidden,
	            max_pool_literals, clause, &proof_margin,
	            &reconvergence_clauses, &proof_cover_cut, dual_proof);
	        if (!proof_clause && dual_proof != nullptr && dual_proof->valid) {
	          reconvergence_clauses.clear();
	          proof_cover_cut = PoolCut{};
	          proof_clause = build_reduced_cost_cutoff_conflict_clause(
	              base_lp.vars, root.lb, root.ub, proof_node_lb, proof_node_ub,
	              x_relax, reduced_costs, basis_indices, node_bound,
	              incumbent_obj, opt.int_tol, branch_reasons, reason_bounds,
		              forbidden, max_pool_literals, clause, &proof_margin,
	              &reconvergence_clauses, &proof_cover_cut);
	        }
	        (void)proof_margin;
	        if (proof_clause && local_proof_cuts != nullptr &&
	            proof_cover_cut.coeff.size() == n &&
	            proof_cover_cut.coeff.nonZeros() > 0 &&
	            valid_pool_cut(proof_cover_cut, &x_relax,
	                           "rc_forbidden_proof_cover") &&
	            sparse_dot(proof_cover_cut.coeff, x_relax) >
	            proof_cover_cut.rhs + 1e-7) {
	          ++out.bc_stats.local_node_cuts_generated;
	          local_proof_cuts->push_back(
	              make_local_node_cut(std::move(proof_cover_cut)));
	        }
	      }
	      if (!proof_clause) {
	        clause = branch_reasons;
      bool merged = false;
      for (auto& lit : clause) {
        if (lit.var_idx != forbidden.var_idx || lit.is_lb != forbidden.is_lb) continue;
        if (lit.is_lb) lit.value = std::max(lit.value, forbidden.value);
        else lit.value = std::min(lit.value, forbidden.value);
        merged = true;
        break;
	      }
	      if (!merged) clause.push_back(forbidden);
	      }
	      canonicalize_branch_literals(clause);
	      if (clause.empty() ||
	          static_cast<int>(clause.size()) > max_pool_literals) {
	        continue;
	      }

	      std::vector<std::vector<BranchDomainLiteral>> clauses_to_learn;
	      clauses_to_learn.push_back(clause);
	      clauses_to_learn.insert(clauses_to_learn.end(),
	                              reconvergence_clauses.begin(),
	                              reconvergence_clauses.end());
	      std::vector<std::vector<BranchDomainLiteral>> learned_clauses;
	      for (const auto& clause_to_learn : clauses_to_learn) {
	        std::vector<std::vector<BranchDomainLiteral>> local_learned;
	        conflict_pool.add_with_binary_resolution(
	            base_lp.vars, clause_to_learn,
	            /*max_resolvents=*/4, &local_learned);
	        learned_clauses.insert(learned_clauses.end(),
	                               local_learned.begin(), local_learned.end());
	      }
	      bool learned_any = false;
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
	                             "rc_learned_conflict_cut") &&
	              (static_cast<int>(learned_clause.size()) <= 2 ||
	               sparse_dot(conflict_cut.coeff, x_relax) >
	                   conflict_cut.rhs + 1e-7)) {
	            new_cuts.push_back(std::move(conflict_cut));
	            ++cut_count;
	            ++out.bc_stats.rc_conflict_cuts_added;
	          }
	        }
	        if (proof_clause) {
	          ++out.bc_stats.rc_proof_conflict_clauses_learned;
	          out.bc_stats.rc_proof_conflict_literals_before += full_clause_len;
	          out.bc_stats.rc_proof_conflict_literals_after +=
	              static_cast<std::uint64_t>(learned_clause.size());
	        }
	        ++out.bc_stats.rc_conflict_clauses_learned;
	        learned_any = true;
	      }
	      if (learned_any) {
	        ++n_learned;
	      }
	    }
	  };

  std::function<bool(Node&, const std::vector<BranchDomainLiteral>&)>
      add_scoped_local_conflict_clause;
	  std::function<bool(Node&, const std::vector<BranchDomainLiteral>&,
	                     const DomainReasonBound*)>
	      add_scoped_conflict_object;
	  std::function<bool(Node&, const DomainReasonBound&)>
	      add_scoped_target_bound_certificate;
	  std::function<bool(Node&,
	                     const std::vector<DomainReasonBound>&,
	                     std::uint64_t&,
	                     std::uint64_t&,
	                     bool&)>
	      replay_queue_reason_trail_certificates;
	  std::function<bool(Node&, std::vector<BoundChangeInfo>&, std::uint64_t*)>
	      propagate_global_bound_lifting_certificates;
		  std::function<int(const DomainReasonBound&,
		                    const std::vector<BranchDomainLiteral>*,
		                    const std::vector<std::vector<BranchDomainLiteral>>*,
		                    const char*)>
		      publish_bound_lifting_certificate;

  auto learn_short_reconvergence_implication =
      [&](const std::vector<BranchDomainLiteral>& branch_reasons,
          const std::vector<DomainReasonBound>& reason_bounds,
          const Eigen::VectorXd* local_x,
          const Eigen::VectorXd* local_lb,
          const Eigen::VectorXd* local_ub,
          std::vector<PoolCut>* local_proof_cuts,
          std::vector<PoolCut>& new_cuts,
          int& cut_count,
          Node* scoped_node = nullptr) -> bool {
	    if (!opt.enable_reduced_cost_proof_conflict_minimization ||
	        reason_bounds.empty()) {
	      return false;
	    }
	    const int max_cut_literals =
	        std::max(2, opt.reduced_cost_conflict_cut_max_literals);
    const int max_literals = std::min(32, max_cut_literals);
    const int max_frontier_clauses =
        std::min(4, std::max(1, opt.reduced_cost_conflict_max_per_node));
    struct FrontierCandidate {
      std::vector<BranchDomainLiteral> clause;
      std::vector<BranchDomainLiteral> frontier_literals;
      BranchDomainLiteral target_bound;
      BranchDomainLiteral source_literal;
      double score{0.0};
      double cut_violation{0.0};
      int queue_hits{0};
      int trail_pos{-1};
      int depth{0};
      bool has_bound_lift{false};
      std::optional<PoolCut> local_implication_cut;
    };
    std::vector<FrontierCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(
        std::min<int>(reason_bounds.size(), max_frontier_clauses * 4)));
    BranchLiteralListSet seen;

    auto literal_pos = [&](const BranchDomainLiteral& lit) -> int {
      int best_pos = -1;
      for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
        if (branch_literal_stronger_or_equal(branch_reasons[static_cast<std::size_t>(i)],
                                             lit)) {
          best_pos = std::max(best_pos, i);
        }
      }
      for (const auto& rb : reason_bounds) {
        if (rb.trail_pos < 0) continue;
        if (branch_literal_stronger_or_equal(rb.bound, lit)) {
          best_pos = std::max(best_pos, rb.trail_pos);
        }
      }
      return best_pos;
    };

    auto find_reason_for_literal =
        [&](const BranchDomainLiteral& lit,
            int before_pos) -> const DomainReasonBound* {
      const DomainReasonBound* best = nullptr;
      for (const auto& rb : reason_bounds) {
        if (rb.trail_pos < 0 || rb.trail_pos >= before_pos) continue;
        if (!branch_literal_stronger_or_equal(rb.bound, lit)) continue;
        if (best == nullptr || rb.trail_pos > best->trail_pos) {
          best = &rb;
        }
      }
      return best;
    };

    auto uip_minimize_frontier =
        [&](std::vector<BranchDomainLiteral> frontier,
            int target_pos) {
      canonicalize_branch_literals(frontier);
      BranchLiteralListSet visited_frontiers;
      for (int iter = 0; iter < 16; ++iter) {
        if (!visited_frontiers.insert(frontier)) break;

        int best_idx = -1;
        int best_pos = -1;
        int resolvable_at_depth = 0;
        const int branch_depth_begin = static_cast<int>(branch_reasons.size());
        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
          const int pos = literal_pos(frontier[static_cast<std::size_t>(i)]);
          if (pos < branch_depth_begin || pos >= target_pos) continue;
          const DomainReasonBound* reason =
              find_reason_for_literal(frontier[static_cast<std::size_t>(i)],
                                      target_pos);
          if (reason == nullptr || reason->reason.empty()) continue;
          ++resolvable_at_depth;
          if (pos > best_pos) {
            best_pos = pos;
            best_idx = i;
          }
        }
        if (best_idx < 0 || resolvable_at_depth <= 1) break;
        const DomainReasonBound* reason =
            find_reason_for_literal(frontier[static_cast<std::size_t>(best_idx)],
                                    target_pos);
        if (reason == nullptr) break;
        frontier.erase(frontier.begin() + best_idx);
        frontier.insert(frontier.end(), reason->reason.begin(), reason->reason.end());
        canonicalize_branch_literals(frontier);
        if (frontier.empty() ||
            static_cast<int>(frontier.size()) > max_literals) {
          break;
        }
      }
      return frontier;
    };

    struct TrailFrontierEntry {
      BranchDomainLiteral lit;
      int trail_pos{-1};
      bool is_branch{true};
    };

    const bool have_scoped_trail =
        scoped_node != nullptr && !scoped_node->local_domain_trail.empty();
    const std::vector<LocalDomainTrailEntry>* scoped_trail =
        have_scoped_trail ? scoped_node->local_domain_trail.read_ptr() : nullptr;
    const std::vector<int>* scoped_branch_positions =
        have_scoped_trail ? scoped_node->local_branch_positions.read_ptr()
                          : nullptr;

    auto trail_entry_for_literal =
        [&](const BranchDomainLiteral& lit,
            int before_pos) -> std::optional<TrailFrontierEntry> {
      if (!have_scoped_trail || lit.var_idx < 0 || lit.var_idx >= n) {
        return std::nullopt;
      }
      const LocalDomainTrailEntry* best = nullptr;
      for (const auto& entry : *scoped_trail) {
        if (entry.pos < 0 || entry.pos >= before_pos ||
            entry.bound.var_idx != lit.var_idx ||
            entry.bound.is_lb != lit.is_lb) {
          continue;
        }
        if (!branch_literal_stronger_or_equal(entry.bound, lit,
                                              std::max(1e-9, opt.int_tol))) {
          continue;
        }
        if (best == nullptr || entry.pos > best->pos) best = &entry;
      }
      if (best == nullptr) return std::nullopt;
      return TrailFrontierEntry{lit, best->pos, best->is_branch};
    };

    auto compact_trail_frontier =
        [&](std::vector<TrailFrontierEntry>& frontier) {
      std::vector<TrailFrontierEntry> compact;
      compact.reserve(frontier.size());
      const double tol = std::max(1e-9, opt.int_tol);
      for (auto& entry : frontier) {
        if (entry.lit.var_idx < 0 || entry.lit.var_idx >= n ||
            !std::isfinite(entry.lit.value)) {
          continue;
        }
        bool merged = false;
        if (entry.trail_pos >= 0) {
          for (auto& cur : compact) {
            if (cur.trail_pos != entry.trail_pos) continue;
            if (cur.lit.var_idx == entry.lit.var_idx &&
                cur.lit.is_lb == entry.lit.is_lb) {
              if (entry.lit.is_lb) {
                cur.lit.value = std::max(cur.lit.value, entry.lit.value);
              } else {
                cur.lit.value = std::min(cur.lit.value, entry.lit.value);
              }
            }
            merged = true;
            break;
          }
        }
        if (!merged) {
          for (auto& cur : compact) {
            if (cur.trail_pos >= 0 || entry.trail_pos >= 0 ||
                cur.lit.var_idx != entry.lit.var_idx ||
                cur.lit.is_lb != entry.lit.is_lb) {
              continue;
            }
            const bool replace = entry.lit.is_lb
                ? entry.lit.value > cur.lit.value + tol
                : entry.lit.value < cur.lit.value - tol;
            if (replace) cur = std::move(entry);
            merged = true;
            break;
          }
        }
        if (!merged) compact.push_back(std::move(entry));
      }
      frontier.swap(compact);
      std::sort(frontier.begin(), frontier.end(),
                [](const TrailFrontierEntry& a,
                   const TrailFrontierEntry& b) {
          if (a.trail_pos != b.trail_pos) {
            if (a.trail_pos < 0) return false;
            if (b.trail_pos < 0) return true;
            return a.trail_pos < b.trail_pos;
          }
          if (a.lit.var_idx != b.lit.var_idx) {
            return a.lit.var_idx < b.lit.var_idx;
          }
          if (a.lit.is_lb != b.lit.is_lb) return a.lit.is_lb < b.lit.is_lb;
          return a.lit.value < b.lit.value;
        });
    };

    auto trail_frontier_hash =
        [&](const std::vector<TrailFrontierEntry>& frontier) {
      std::size_t h = 0;
      for (const auto& entry : frontier) {
        std::size_t eh = branch_literal_hash(entry.lit);
        eh ^= std::hash<int>{}(entry.trail_pos) + 0x9e3779b97f4a7c15ULL +
              (eh << 6) + (eh >> 2);
        h ^= eh + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
      }
      return h;
    };
    auto trail_frontiers_equal =
        [](const std::vector<TrailFrontierEntry>& lhs,
           const std::vector<TrailFrontierEntry>& rhs) {
          if (lhs.size() != rhs.size()) return false;
          for (std::size_t i = 0; i < lhs.size(); ++i) {
            if (lhs[i].trail_pos != rhs[i].trail_pos ||
                lhs[i].is_branch != rhs[i].is_branch ||
                !branch_literals_equal(lhs[i].lit, rhs[i].lit, 0.0)) {
              return false;
            }
          }
          return true;
        };

    auto trail_frontier_literals =
        [&](const std::vector<TrailFrontierEntry>& frontier) {
      std::vector<BranchDomainLiteral> lits;
      lits.reserve(frontier.size());
      for (const auto& entry : frontier) lits.push_back(entry.lit);
      canonicalize_branch_literals(lits);
      return lits;
    };

    auto resolve_scoped_trail_frontier =
        [&](std::vector<BranchDomainLiteral> raw_frontier,
            int target_pos) {
      if (!have_scoped_trail) {
        return uip_minimize_frontier(std::move(raw_frontier), target_pos);
      }
      std::vector<TrailFrontierEntry> frontier;
      frontier.reserve(raw_frontier.size());
      const int before_pos =
          target_pos >= 0
              ? std::min(target_pos, static_cast<int>(scoped_trail->size()))
              : static_cast<int>(scoped_trail->size());
      for (const auto& lit : raw_frontier) {
        auto entry = trail_entry_for_literal(lit, before_pos);
        if (entry.has_value()) {
          frontier.push_back(*entry);
        } else {
          frontier.push_back(
              TrailFrontierEntry{lit, literal_pos(lit), true});
        }
      }

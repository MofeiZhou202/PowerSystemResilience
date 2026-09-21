// @file bc_run/08_root_heuristics_c.inc
// BCSolveState::run() segment 8/14: root heuristics — part 3 (progressive rounding, LP diving, LNS, root probing).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
      // Sort by reduced-cost score (descending) when available,
      // otherwise by proximity to integer (least disruptive first).
      // Higher rc_score means larger expected objective impact per fix.
      if (rc_ptr) {
        std::sort(frac_vars.begin(), frac_vars.end(),
                  [](const FracVar& a, const FracVar& b) { return a.rc_score > b.rc_score; });
      } else {
        std::sort(frac_vars.begin(), frac_vars.end(),
                  [](const FracVar& a, const FracVar& b) { return a.dist < b.dist; });
      }

      // Batch-fix variables close to integrality before solving LP.
      // This reduces the total number of LP solves during diving.
      // Use wider threshold for very large problems to fix more per LP.
      // When stalled, increase threshold aggressively to break through.
      const double batch_thr = use_batch_fix ? 0.45
          : ((m_dive > 10000) ? 0.35
          : ((m_dive > opt.xlarge_ipm_only_threshold) ? 0.3 : 0.15));
      {
        int batch_fixed = 0;
        for (auto& fv : frac_vars) {
          if (fv.dist >= batch_thr) break;
          const double fixed_val = std::round(dive_x[fv.j]);
          dive_fixed_vars.push_back(fv.j);
          dive_saved_bounds.push_back({dive_lb[fv.j], dive_ub[fv.j]});
          dive_lb[fv.j] = std::max(dive_lb[fv.j], fixed_val);
          dive_ub[fv.j] = std::min(dive_ub[fv.j], fixed_val);
          dive_x[fv.j] = fixed_val;  // Keep dive_x consistent with bounds.
          ++batch_fixed;
        }
        if (batch_fixed > 1) {
          // Remove batch-fixed from frac_vars.
          frac_vars.erase(frac_vars.begin(), frac_vars.begin() + batch_fixed);
          if (frac_vars.empty()) continue;  // Re-evaluate after batch fix.
        }
      }

      // Try fixing the best remaining candidate.
      // For large problems, allow more candidate fixings before giving up,
      // subject to the shared dive budget.
      bool step_ok = false;
      const int max_attempts = xlarge_dive
          ? std::min(3, static_cast<int>(frac_vars.size()))
          : (m_dive > opt.xlarge_ipm_only_threshold)
          ? std::min(10, static_cast<int>(frac_vars.size()))
          : std::min(5, static_cast<int>(frac_vars.size()));
      for (int attempt = 0; attempt < max_attempts && !step_ok; ++attempt) {
        if (dive_budget_exceeded()) break;
        const int j = frac_vars[attempt].j;
        const double xval = dive_x[j];
        const double rc = frac_vars[attempt].rc;

        // Save bounds for potential backtrack.
        const double saved_lb = dive_lb[j];
        const double saved_ub = dive_ub[j];

        // Determine fix direction from the reduced cost when available.
        // For minimization: positive rc → fix to lower bound (0), negative rc → fix to upper bound (1).
        // Otherwise, round to nearest integer.
        double fixed_val;
        if (rc_ptr && std::abs(rc) > 1e-9) {
          // Fix in direction indicated by reduced cost sign.
          fixed_val = (rc > 0) ? std::floor(xval) : std::ceil(xval);
          // Clamp to variable bounds.
          fixed_val = std::max(base_lp.vars[j].lb, std::min(base_lp.vars[j].ub, fixed_val));
        } else {
          fixed_val = std::round(xval);
        }
        dive_lb[j] = std::max(dive_lb[j], fixed_val);
        dive_ub[j] = std::min(dive_ub[j], fixed_val);

        ++out.bc_stats.lp_solves;
        ++dive_lp_count;
        bool dive_lp_ok = false;
        double dive_lp_obj = 0.0;
        // Prefer dual simplex so accepted steps retain a reusable basis.
        {
          update_standard_form_bounds(dive_sf, base_lp, dive_lb, dive_ub);
          apply_dive_solve_budget();
          auto t_dive_lp0 = std::chrono::steady_clock::now();
          auto dive_res = solve_lp_from_sf(dive_sf, dive_opt, dive_basis.get());
          auto t_dive_lp1 = std::chrono::steady_clock::now();
          if (opt.verbose) {
            fmt::print(stderr, "[DIVE-LP] step={} attempt={} iters={} time={:.1f}ms success={} hint_rows={}\n",
                    dive_step, attempt, dive_res.result.stats.iterations,
                    std::chrono::duration<double, std::milli>(t_dive_lp1 - t_dive_lp0).count(),
                    dive_res.result.stats.success ? 1 : 0,
                    dive_basis ? dive_basis->rows : -1);
          }
          if (dive_res.result.stats.success && dive_res.result.x.size() == n) {
            dive_lp_ok = true;
            dive_lp_obj = dive_res.result.stats.objective;
            if (!has_incumbent || dive_lp_obj < incumbent_obj - 1e-9) {
              dive_x = clamp_to_bounds(dive_res.result.x, dive_lb, dive_ub);
              auto db = std::make_shared<SimplexBasis>(dive_res.basis);
              db->cached_reduced_costs =
                  std::make_shared<const Eigen::VectorXd>(std::move(dive_res.reduced_costs));
              dive_basis = db;
              dive_fixed_vars.push_back(j);
              dive_saved_bounds.push_back({saved_lb, saved_ub});
              step_ok = true;
            }
          } else if (presolved_m <= 5000) {
            // Keep the IPM fallback below its row-count policy boundary and
            // within the shared dive budget.
            if (auto* dive_ipm = ensure_root_heur_ipm()) {
              auto dive_ipm_res = dive_ipm->solve_node_lp(base_lp, dive_lb, dive_ub, dive_x);
              bool ipm_usable = dive_ipm_res.stats.success ||
                  (dive_ipm_res.stats.status == "MaxIter" && dive_ipm_res.stats.primal_feas < 1e-3);
              if (ipm_usable && dive_ipm_res.x.size() == n) {
                dive_lp_ok = true;
                dive_lp_obj = dive_ipm_res.stats.objective;
                if (!has_incumbent || dive_lp_obj < incumbent_obj - 1e-9) {
                  dive_x = clamp_to_bounds(dive_ipm_res.x, dive_lb, dive_ub);
                  dive_fixed_vars.push_back(j);
                  dive_saved_bounds.push_back({saved_lb, saved_ub});
                  step_ok = true;
                }
              }
            }
          }
        }

        if (dive_lp_ok && step_ok) {
          // Fixing succeeded — continue dive.
        } else {
          // Primary direction failed — try opposite direction.
          dive_lb[j] = saved_lb;
          dive_ub[j] = saved_ub;

          if (dive_budget_exceeded()) break;
          double alt_val = (xval > fixed_val) ? std::ceil(xval) : std::floor(xval);
          if (alt_val < base_lp.vars[j].lb || alt_val > base_lp.vars[j].ub) continue;
          if (std::abs(alt_val - fixed_val) < 0.5) continue;

          dive_lb[j] = std::max(dive_lb[j], alt_val);
          dive_ub[j] = std::min(dive_ub[j], alt_val);

          ++out.bc_stats.lp_solves;
          ++dive_lp_count;
          {
            update_standard_form_bounds(dive_sf, base_lp, dive_lb, dive_ub);
            apply_dive_solve_budget();
            auto alt_res = solve_lp_from_sf(dive_sf, dive_opt, dive_basis.get());

            if (alt_res.result.stats.success && alt_res.result.x.size() == n &&
                (!has_incumbent || alt_res.result.stats.objective < incumbent_obj - 1e-9)) {
              dive_x = clamp_to_bounds(alt_res.result.x, dive_lb, dive_ub);
              auto db = std::make_shared<SimplexBasis>(alt_res.basis);
              db->cached_reduced_costs =
                  std::make_shared<const Eigen::VectorXd>(std::move(alt_res.reduced_costs));
              dive_basis = db;
              dive_fixed_vars.push_back(j);
              dive_saved_bounds.push_back({saved_lb, saved_ub});
              step_ok = true;
            } else {
              dive_lb[j] = saved_lb;
              dive_ub[j] = saved_ub;
            }
          }
        }
      }

      if (!step_ok) {
        // Flip heuristic: undo the last few fixings and try opposite values.
        // This explores different commitment patterns when the current dive path
        // leads to infeasibility.
        bool flipped = false;
        const int flip_depth = std::min(3, static_cast<int>(dive_fixed_vars.size()));
        if (flip_depth > 0 && !dive_budget_exceeded()) {
          // Undo recent fixings
          std::vector<std::pair<int, double>> to_flip;
          for (int bt = 0; bt < flip_depth; ++bt) {
            const int bj = dive_fixed_vars.back();
            auto [orig_lb, orig_ub] = dive_saved_bounds.back();
            // Current fixed value
            double cur_val = dive_lb[bj];  // lb=ub after fix
            double flip_val = (cur_val < 0.5) ? 1.0 : 0.0;
            flip_val = std::max(orig_lb, std::min(orig_ub, flip_val));
            dive_lb[bj] = orig_lb;
            dive_ub[bj] = orig_ub;
            dive_fixed_vars.pop_back();
            dive_saved_bounds.pop_back();
            if (std::abs(flip_val - cur_val) > 0.5) {
              to_flip.push_back({bj, flip_val});
            }
          }
          // Try flipped values
          for (auto& [fj, fval] : to_flip) {
            if (dive_budget_exceeded()) break;
            double saved_lb = dive_lb[fj];
            double saved_ub = dive_ub[fj];
            dive_lb[fj] = fval;
            dive_ub[fj] = fval;
            {
              update_standard_form_bounds(dive_sf, base_lp, dive_lb, dive_ub);
              apply_dive_solve_budget();
              auto fl_res = solve_lp_from_sf(dive_sf, dive_opt, dive_basis.get());
              ++out.bc_stats.lp_solves;
              ++dive_lp_count;
              if (fl_res.result.stats.success && fl_res.result.x.size() == n) {
                dive_x = clamp_to_bounds(fl_res.result.x, dive_lb, dive_ub);
                auto db = std::make_shared<SimplexBasis>(fl_res.basis);
                db->cached_reduced_costs =
                    std::make_shared<const Eigen::VectorXd>(std::move(fl_res.reduced_costs));
                dive_basis = db;
                dive_fixed_vars.push_back(fj);
                dive_saved_bounds.push_back({saved_lb, saved_ub});
                flipped = true;
              } else {
                dive_lb[fj] = saved_lb;
                dive_ub[fj] = saved_ub;
              }
            }
          }
        }
        if (!flipped) {
          ++dive_fail_count;
          // For very large problems, abort on the first failed fixing so the
          // optional dive cannot consume the tree budget.
          if (xlarge_dive || dive_fail_count >= 2) {
            dive_failed = true;
            break;
          }
        } else {
          dive_fail_count = 0;  // Reset on successful flip.
        }
      }
    }
    if (opt.verbose) {
      auto t_dive_loop_done = std::chrono::steady_clock::now();
      fmt::print(stderr, "[B&C ROOT] diving_lps={} dive_loop={:.1f}ms post_setup_start\n", dive_lp_count,
              std::chrono::duration<double, std::milli>(t_dive_loop_done - t_pump_done).count());
    }

    // Post-dive rounding: round all remaining fractional binaries in the
    // dive solution and check feasibility directly. If feasible, use as incumbent.
    // Also try solving the LP for better continuous variable values.
    // Skip for very large problems: constructive dispatch + LP rounding is
    // expensive (~700ms) and almost never finds a feasible incumbent due to
    // complex coupling constraints (min-up/min-down/ramp).
    if (max_dive_lps > 0 && !has_incumbent && base_m <= 5000) {
      // Direct rounding check: round binaries in dive_x, repair continuous
      Eigen::VectorXd xr = dive_x;
      if (reduced_uc_hint.has_value()) {
        const auto& ruc = *reduced_uc_hint;
        uc_round_commitment(xr, ruc, ruc.ng);
        
        // Debug: print column mappings for g=152
        if (opt.verbose && !ruc.ig_cols.empty()) {
          for (int t = 6; t <= 9 && t < ruc.T; ++t) {
            size_t idx = static_cast<size_t>(t * ruc.ng + 152);
            if (idx < ruc.ig_cols.size())
              fprintf(stderr, "[DEBUG] g=152 t=%d ig_col=%d su_col=%d pg_col=%d ig_start_off=%d su_start_off=%d\n",
                      t, ruc.ig_cols[idx], ruc.su_cols[idx], ruc.pg_cols[idx],
                      ruc.ig_start + t * ruc.ng + 152,
                      ruc.su_start + t * ruc.ng + 152);
          }
        }
        
        // Constructive economic dispatch: given rounded commitment u[g,t],
        // dispatch power from scratch respecting Pmin/Pmax, ramp, and demand.
        if (!ruc.pg_cols.empty() && !ruc.ig_cols.empty() &&
            !ruc.pmin.empty() && !ruc.pmax.empty()) {
          auto pg_col = [&](int g, int t) -> int {
            size_t idx = static_cast<size_t>(t * ruc.ng + g);
            return idx < ruc.pg_cols.size() ? ruc.pg_cols[idx] : -1;
          };
          auto ig_col = [&](int g, int t) -> int {
            size_t idx = static_cast<size_t>(t * ruc.ng + g);
            return idx < ruc.ig_cols.size() ? ruc.ig_cols[idx] : -1;
          };
          auto su_col = [&](int g, int t) -> int {
            size_t idx = static_cast<size_t>(t * ruc.ng + g);
            return idx < ruc.su_cols.size() ? ruc.su_cols[idx] : -1;
          };
          const bool has_ramp = !ruc.ramp.empty();
          const int ng = ruc.ng, T = ruc.T;

          // Build set of power column indices for quick lookup
          std::vector<bool> is_pg_col(static_cast<size_t>(n), false);
          for (int g = 0; g < ng; ++g)
            for (int t = 0; t < T; ++t) {
              int pc = pg_col(g, t);
              if (pc >= 0 && pc < n) is_pg_col[static_cast<size_t>(pc)] = true;
            }

          if (opt.verbose)
            fprintf(stderr, "[DISPATCH] running constructive dispatch ng=%d T=%d\n", ng, T);
          // Set p=0 for decommitted generators, Pmin for committed
          for (int g = 0; g < ng; ++g) {
            for (int t = 0; t < T; ++t) {
              int pc = pg_col(g, t);
              int uc_ = ig_col(g, t);
              if (pc >= 0 && pc < n && uc_ >= 0 && uc_ < n)
                xr[pc] = (xr[uc_] > 0.5) ? ruc.pmin[static_cast<size_t>(g)] : 0.0;
            }
          }

          // Build merit order: sort generators by cost
          std::vector<int> merit(static_cast<size_t>(ng));
          std::iota(merit.begin(), merit.end(), 0);
          std::sort(merit.begin(), merit.end(), [&](int a, int b) {
            int pa = pg_col(a, 0), pb = pg_col(b, 0);
            return (pa >= 0 && pa < n && pb >= 0 && pb < n)
                ? base_lp.c[pa] < base_lp.c[pb] : a < b;
          });

          // Map each timestep to its Aeq row for demand target
          std::vector<int> t_to_eq_row(static_cast<size_t>(T), -1);
          for (int t = 0; t < T; ++t) {
            for (int g = 0; g < ng; ++g) {
              int pc = pg_col(g, t);
              if (pc < 0 || pc >= n) continue;
              for (Eigen::SparseMatrix<double>::InnerIterator it(pre_cut_root_lp.Aeq, pc); it; ++it) {
                t_to_eq_row[static_cast<size_t>(t)] = it.row();
                break;
              }
              if (t_to_eq_row[static_cast<size_t>(t)] >= 0) break;
            }
          }

          // Iterative forward-backward dispatch (converges ramp constraints)
          for (int fwd_bwd_iter = 0; fwd_bwd_iter < 3; ++fwd_bwd_iter) {

          // Forward dispatch respecting ramp-up from t-1 and ramp-down to t+1
          for (int t = 0; t < T; ++t) {
            // Use demand from equality constraint RHS (robust to presolve)
            double target = 0.0;
            int eq_row = t_to_eq_row[static_cast<size_t>(t)];
            if (eq_row >= 0) {
              target = pre_cut_root_lp.beq[eq_row];
            } else {
              for (int g = 0; g < ng; ++g) {
                int pc = pg_col(g, t);
                if (pc >= 0 && pc < n) target += dive_x[pc];
              }
            }

            struct GBound { int g; double lo, hi, p; };
            std::vector<GBound> committed;
            double must_gen = 0.0;
            for (int gm : merit) {
              int pc = pg_col(gm, t);
              int uc_ = ig_col(gm, t);
              if (pc < 0 || pc >= n || uc_ < 0 || uc_ >= n || xr[uc_] < 0.5) continue;
              double pm = ruc.pmin[static_cast<size_t>(gm)];
              double px = ruc.pmax[static_cast<size_t>(gm)];
              double lo = pm, hi = px;
              double r = has_ramp ? ruc.ramp[static_cast<size_t>(gm)] : 1e20;
              // Ramp-up from t-1
              if (has_ramp && t > 0) {
                int pc_prev = pg_col(gm, t - 1);
                if (pc_prev >= 0 && pc_prev < n) {
                  double prev_p = xr[pc_prev];
                  lo = std::max(lo, prev_p - r);
                  hi = std::min(hi, prev_p + r);
                }
              }
              // Ramp-down look-ahead: if gen shuts down at t+1, p[t] ≤ ramp
              if (has_ramp && t + 1 < T) {
                int uc_next = ig_col(gm, t + 1);
                if (uc_next >= 0 && uc_next < n && xr[uc_next] < 0.5) {
                  hi = std::min(hi, r);
                }
              }
              // Ramp-down to t+1: p[t] ≤ p[t+1] + ramp (use current xr value)
              if (has_ramp && t + 1 < T) {
                int pc_next = pg_col(gm, t + 1);
                if (pc_next >= 0 && pc_next < n)
                  hi = std::min(hi, xr[pc_next] + r);
              }
              if (lo > hi) { double mid = (lo + hi) * 0.5; lo = hi = mid; }
              committed.push_back({gm, lo, hi, lo});
              must_gen += lo;
            }

            double remaining = target - must_gen;
            for (auto& gb : committed) {
              if (remaining <= 0) break;
              double add = std::min(remaining, gb.hi - gb.lo);
              gb.p = gb.lo + add;
              remaining -= add;
            }

            for (auto& gb : committed) {
              int pc = pg_col(gb.g, t);
              if (pc >= 0 && pc < n) xr[pc] = gb.p;
            }
          }

          // Backward pass: enforce ramp-down p[t] ≤ p[t+1] + ramp,
          // then redistribute shortfall to other generators.
          if (has_ramp) {
            for (int t = T - 2; t >= 0; --t) {
              double shortfall = 0.0;
              for (int g = 0; g < ng; ++g) {
                int pc = pg_col(g, t);
                int pc_next = pg_col(g, t + 1);
                if (pc < 0 || pc >= n || pc_next < 0 || pc_next >= n) continue;
                double r = ruc.ramp[static_cast<size_t>(g)];
                double max_p = xr[pc_next] + r;
                if (xr[pc] > max_p + 1e-9) {
                  shortfall += xr[pc] - max_p;
                  xr[pc] = max_p;
                }
              }
              // Redistribute shortfall to generators with remaining capacity
              if (shortfall > 1e-9) {
                for (int gm : merit) {
                  if (shortfall <= 1e-9) break;
                  int pc = pg_col(gm, t);
                  int uc_ = ig_col(gm, t);
                  if (pc < 0 || pc >= n || uc_ < 0 || uc_ >= n || xr[uc_] < 0.5) continue;
                  double px = ruc.pmax[static_cast<size_t>(gm)];
                  double r = ruc.ramp[static_cast<size_t>(gm)];
                  double hi = px;
                  if (t > 0) {
                    int pc_prev = pg_col(gm, t - 1);
                    if (pc_prev >= 0 && pc_prev < n)
                      hi = std::min(hi, xr[pc_prev] + r);
                  }
                  if (t + 1 < T) {
                    int pc_next = pg_col(gm, t + 1);
                    if (pc_next >= 0 && pc_next < n)
                      hi = std::min(hi, xr[pc_next] + r);
                  }
                  double room = hi - xr[pc];
                  if (room > 1e-9) {
                    double add = std::min(shortfall, room);
                    xr[pc] += add;
                    shortfall -= add;
                  }
                }
              }
            }
          }

          } // end fwd_bwd_iter
          
          // Startup cost repair using per-column maps:
          // s[g,t] ≥ startup_cost * max(0, u[t]-u[t-1])
          // Read startup cost from constraint matrix for accuracy after presolve.
          for (int g = 0; g < ng; ++g) {
            for (int t = 0; t < T; ++t) {
              int sc = su_col(g, t);
              if (sc < 0 || sc >= n) continue;
              
              // Find required minimum from ineq constraints involving this column
              double s_min = 0.0;
              for (Eigen::SparseMatrix<double>::InnerIterator it(pre_cut_root_lp.A, sc); it; ++it) {
                int ri = it.row();
                double a_s = it.value();
                double row_rest = pre_cut_root_lp.A.row(ri).dot(xr) - a_s * xr[sc];
                if (a_s < -1e-12) {
                  double needed = (pre_cut_root_lp.b[ri] - row_rest) / a_s;
                  s_min = std::max(s_min, needed);
                }
              }
              xr[sc] = std::clamp(s_min, base_lp.vars[sc].lb,
                                  std::max(base_lp.vars[sc].ub, s_min));
            }
          }
        }
      }
      for (int i = 0; i < n; ++i) {
        if (is_integer_type(base_lp.vars[i]))
          xr[i] = std::round(xr[i]);
        xr[i] = std::clamp(xr[i], root.lb[i], root.ub[i]);
      }
	      if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub,
	                                kNodeFeasibilityTol)) {
	        const double obj = objective_value(base_lp.c, xr, base_lp.sense);
	        const bool improved = adopt_root_incumbent("direct_rounding", xr, obj);
	        if (opt.verbose)
	          fprintf(stderr,
	                  "[B&C ROOT] direct rounding found incumbent obj=%.1f "
	                  "improved=%d\n",
	                  obj, improved ? 1 : 0);
	      } else if (opt.verbose) {
        // Detailed violation diagnostics
        int n_bnd = 0, n_ineq = 0, n_eq = 0;
        double max_bnd = 0, max_ineq = 0, max_eq = 0;
        for (int i = 0; i < n; ++i) {
          double lb = std::max(pre_cut_root_lp.vars[i].lb, root.lb[i]);
          double ub = std::min(pre_cut_root_lp.vars[i].ub, root.ub[i]);
          if (xr[i] < lb - 1e-6) { ++n_bnd; max_bnd = std::max(max_bnd, lb - xr[i]); }
          if (xr[i] > ub + 1e-6) { ++n_bnd; max_bnd = std::max(max_bnd, xr[i] - ub); }
        }
        for (int i = 0; i < pre_cut_root_lp.A.rows(); ++i) {
          double v = pre_cut_root_lp.A.row(i).dot(xr) - pre_cut_root_lp.b[i];
          if (v > 1e-6) { ++n_ineq; max_ineq = std::max(max_ineq, v); }
        }
        // Print worst violated ineq rows for diagnostics
        {
          std::vector<std::pair<double,int>> viols;
          for (int i = 0; i < pre_cut_root_lp.A.rows(); ++i) {
            double v = pre_cut_root_lp.A.row(i).dot(xr) - pre_cut_root_lp.b[i];
            if (v > 1e-6) viols.push_back({v, i});
          }
          std::sort(viols.begin(), viols.end(), std::greater<>());
          for (int k = 0; k < std::min(3, static_cast<int>(viols.size())); ++k) {
            int ri = viols[static_cast<size_t>(k)].second;
            // Print the nonzero entries (col, coeff) and x values for this row
            std::string row_str;
            Eigen::SparseVector<double> row_vec = pre_cut_root_lp.A.row(ri);
            for (Eigen::SparseVector<double>::InnerIterator it(row_vec); it; ++it) {
              int col = it.index();
              double coeff = it.value();
              char buf[128];
              snprintf(buf, sizeof(buf), " c%d=%.1f*%.2f", col, coeff, xr[col]);
              row_str += buf;
            }
            fprintf(stderr, "[VIOL] row=%d viol=%.1f b=%.1f:%s\n",
                    ri, viols[static_cast<size_t>(k)].first,
                    pre_cut_root_lp.b[ri], row_str.c_str());
          }
        }
        for (int i = 0; i < pre_cut_root_lp.Aeq.rows(); ++i) {
          double v = std::abs(pre_cut_root_lp.Aeq.row(i).dot(xr) - pre_cut_root_lp.beq[i]);
          if (v > 1e-6) {
            ++n_eq; max_eq = std::max(max_eq, v);
            double gen = pre_cut_root_lp.Aeq.row(i).dot(xr);
            double dem = pre_cut_root_lp.beq[i];
            fprintf(stderr, "[EQ-VIOL] eq_row=%d gen=%.1f demand=%.1f diff=%.1f\n",
                    i, gen, dem, gen - dem);
          }
        }
        fprintf(stderr, "[B&C ROOT] direct rounding INFEASIBLE: bnd=%d(%.2e) ineq=%d(%.2e) eq=%d(%.2e)\n",
                n_bnd, max_bnd, n_ineq, max_ineq, n_eq, max_eq);
      }

      // If direct rounding failed, try LP-based rounding with dedicated solver.
      // Skip for very large problems: fixed-integer LPs are usually infeasible
      // due to equality constraint violations (dispatch mismatch) and waste
      // hundreds of ms on a doomed simplex solve.
      if (!has_incumbent && opt.use_ipm_root && base_m <= 5000) {
        Eigen::VectorXd rd_lb = dive_lb, rd_ub = dive_ub;
        if (reduced_uc_hint.has_value()) {
          Eigen::VectorXd xr2 = dive_x;
          uc_round_commitment(xr2, *reduced_uc_hint, reduced_uc_hint->ng);
          for (int i = 0; i < n; ++i) {
            if (!is_integer_type(base_lp.vars[i])) continue;
            if (std::abs(rd_ub[i] - rd_lb[i]) < 1e-9) continue;
            double rv = std::round(xr2[i]);
            rv = std::max(rd_lb[i], std::min(rd_ub[i], rv));
            rd_lb[i] = rv;
            rd_ub[i] = rv;
          }
        } else {
          for (int i = 0; i < n; ++i) {
            if (!is_integer_type(base_lp.vars[i])) continue;
            if (std::abs(rd_ub[i] - rd_lb[i]) < 1e-9) continue;
            double rv = std::round(dive_x[i]);
            rv = std::max(rd_lb[i], std::min(rd_ub[i], rv));
            rd_lb[i] = rv;
            rd_ub[i] = rv;
          }
        }
        if (bounds_consistent(rd_lb, rd_ub)) {
          // Try simplex first (exact), fall back to IPM
          update_standard_form_bounds(dive_sf, base_lp, rd_lb, rd_ub);
          apply_dive_solve_budget();
          auto rd_sx = solve_lp_from_sf(dive_sf, dive_opt, dive_basis.get());
          out.bc_stats.lp_solves += 1;
          bool rd_ok = rd_sx.result.stats.success;
          Eigen::VectorXd rd_x = rd_ok ? rd_sx.result.x : Eigen::VectorXd();
          double rd_pfeas = 0.0;
          std::string rd_status = rd_sx.result.stats.status;
          if (rd_ok && rd_x.size() == n) {
            auto db = std::make_shared<SimplexBasis>(rd_sx.basis);
            dive_basis = db;
          }
          if (!rd_ok) {
            // Skip IPM fallback for large problems — fixed-integer LPs have
            // near-singular KKT systems that make IPM hit MaxIter wastefully.
            if (base_m <= 5000) {
              if (auto* rd_ipm = ensure_root_heur_ipm()) {
                auto rd_res = rd_ipm->solve_node_lp(base_lp, rd_lb, rd_ub, dive_x);
                rd_ok = rd_res.stats.success ||
                    (rd_res.stats.status == "MaxIter" && rd_res.stats.primal_feas < 1e-4);
                rd_x = rd_res.x;
                rd_pfeas = rd_res.stats.primal_feas;
                rd_status = rd_res.stats.status;
              }
            }
          }
          if (opt.verbose)
            fprintf(stderr, "[B&C ROOT] post-dive LP rounding: success=%d status=%s pfeas=%.2e\n",
                    rd_ok ? 1 : 0, rd_status.c_str(), rd_pfeas);
          if (rd_x.size() == n) {
            Eigen::VectorXd xr2 = clamp_to_bounds(rd_x, rd_lb, rd_ub);
            for (int i = 0; i < n; ++i) {
              if (is_integer_type(base_lp.vars[i]))
                xr2[i] = std::round(xr2[i]);
            }
	            if (satisfies_with_bounds(base_lp, xr2, root.lb, root.ub,
	                                      kNodeFeasibilityTol)) {
	              const double obj = objective_value(base_lp.c, xr2, base_lp.sense);
	              const bool improved =
	                  adopt_root_incumbent("post_dive_lp_rounding", xr2, obj);
	              if (opt.verbose)
	                fprintf(stderr,
	                        "[B&C ROOT] post-dive LP rounding found incumbent "
	                        "obj=%.1f improved=%d\n",
	                        obj, improved ? 1 : 0);
	            }
          }
        }
      }
    }
  }

  [[maybe_unused]] auto t_dive_done = std::chrono::steady_clock::now();
  if (opt.verbose) {
    fmt::print(stderr, "[B&C ROOT] diving={:.1f}ms has_incumbent={}\n",
            std::chrono::duration<double, std::milli>(t_dive_done - t_pump_done).count(),
            has_incumbent ? 1 : 0);
  }

  // ════════════════════════════════════════════════════════════════════════
  // Large Neighborhood Search (LNS) around the incumbent.
  // Fix top 80% most-confident integer variables, solve restricted MILP.
  // Effective at finding better solutions on UC instances by exploiting
  // commitment structure (most generators stay on/off across periods).
  // On large problems, reduce LNS aggression to avoid spending too much
  // time on sub-MILP solves that may not improve the incumbent.
  // ════════════════════════════════════════════════════════════════════════
  [[maybe_unused]] auto t_lns_start = std::chrono::steady_clock::now();
  
  // Size-adaptive LNS parameters — skip entirely on very large problems
  // where the sub-MILP itself is expensive (~1s) and rarely improves.
  // Also skip when root gap is already closed.
  // For medium problems (m 5000-10000), limit to 1 iteration since each
  // sub-MILP solve costs ~400ms and rarely improves the incumbent.
  // For parallel mode we still run root LNS: the pre-LNS incumbent from
  // progressive rounding / feasibility pump is often far from optimal
  // (observed 2190914 vs LNS-polished 2167464 on SCUC IEEE-118), and the
  // ~500ms LNS call dominates the total runtime of tree threads that would
  // otherwise start from a poor incumbent.
  const bool skip_lns_large = (presolved_m > 30000) ||
                              large_generic_low_frac_root;
  const bool skip_lns_highs_root_pipeline =
      requested_vendored_highs_lp && opt.auto_highs_root_pipeline;
  const bool skip_lns_gap = root_gap_closed();
  const int effective_lns_max_iters =
      (skip_lns_large || skip_lns_highs_root_pipeline || skip_lns_gap) ? 0
      : ((presolved_m > 10000) ? 1
      : ((presolved_m > 5000) ? std::min(2, opt.lns_max_iters)
      : ((presolved_m > 2000) ? std::min(3, opt.lns_max_iters)
                              : opt.lns_max_iters)));
  const double effective_lns_time_limit = (presolved_m > 5000) ? 1.0
      : ((presolved_m > 2000) ? 1.5 : opt.lns_time_limit);
  const int effective_lns_node_limit = (presolved_m > 5000) ? 100
      : ((presolved_m > 2000) ? 200 : opt.lns_node_limit);
  const double effective_lns_stage_limit =
      effective_lns_time_limit * effective_lns_max_iters;
  
  trace_root_primal_gate(
      "lns", (opt.enable_lns && has_incumbent && effective_lns_max_iters > 0)
                 ? "run"
                 : "skip",
      skip_lns_gap ? "gap_closed=1"
                   : (skip_lns_highs_root_pipeline
                          ? "highs_root_pipeline_uses_solveSubMip=1"
                      : (skip_lns_large ? "large_or_good_low_frac=1"
                                     : (!has_incumbent ? "no_incumbent=1"
                                                       : "budget=1"))));
  if (opt.enable_lns && has_incumbent) {
    // Identify integer variables sorted by "confidence".
    // Use LP relaxation fractionality: variables that are integer in the LP
    // are likely correctly decided, while highly fractional LP variables are
    // uncertain and should be freed for LNS to explore.
    // BUG FIX: The old code used incumbent_x[j] which is always at integer
    // values after progressive rounding, giving confidence=1.0 for ALL vars
    // and making the sort meaningless.
    struct IntVarConf {
      int j;
      double value;
      double confidence;  // Higher = more likely correct → fix in LNS
    };
    std::vector<IntVarConf> int_vars;
    for (int j = 0; j < n; ++j) {
      if (!is_integer_type(base_lp.vars[j])) continue;
      if (std::abs(root.ub[j] - root.lb[j]) < 1e-9) continue;  // Already fixed
      double lp_val = root.x_relax[j];
      double lp_frac = std::abs(lp_val - std::round(lp_val));
      // Confidence: 1.0 when LP is exactly integer, 0.5 when LP = 0.5
      // Also penalize when LP and incumbent disagree on direction
      double conf = 1.0 - lp_frac;
      double inc_rounded = std::round(incumbent_x[j]);
      double lp_rounded = std::round(lp_val);
      if (std::abs(inc_rounded - lp_rounded) > 0.5) {
        // LP and incumbent disagree — this variable is suspicious, lower confidence
        conf *= 0.5;
      }
      int_vars.push_back({j, incumbent_x[j], conf});
    }

    if (!int_vars.empty()) {
      // Sort by confidence (highest first)
      std::sort(int_vars.begin(), int_vars.end(),
                [](const IntVarConf& a, const IntVarConf& b) {
                  return a.confidence > b.confidence;
                });

      // Use a higher fix ratio on very large problems to make the restricted
      // neighborhood smaller.
      const double effective_lns_fix_ratio = (presolved_m > opt.xlarge_ipm_only_threshold)
          ? std::max(opt.lns_fix_ratio, 0.92)
          : opt.lns_fix_ratio;
      const int n_to_fix = static_cast<int>(
          std::ceil(effective_lns_fix_ratio * static_cast<double>(int_vars.size())));

      for (int lns_iter = 0; lns_iter < effective_lns_max_iters; ++lns_iter) {
        const double lns_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_lns_start).count();
        double lns_sub_budget = bc_optional_subsolve_budget_sec(
            effective_lns_time_limit, bc_remaining_sec(),
            bc_post_root_finalize_reserve_sec,
            effective_lns_stage_limit - lns_elapsed);
        if (!(lns_sub_budget > 0.0)) break;

        // Create restricted problem: fix top n_to_fix variables
        MIPModel restricted;
        restricted.linear_part = base_lp;
        
        // Apply root bounds first
        for (int j = 0; j < n; ++j) {
          restricted.linear_part.vars[j].lb = root.lb[j];
          restricted.linear_part.vars[j].ub = root.ub[j];
        }

        // Fix confident variables to their incumbent values
        for (int k = 0; k < n_to_fix && k < static_cast<int>(int_vars.size()); ++k) {
          int j = int_vars[static_cast<size_t>(k)].j;
          double fixed_val = std::round(incumbent_x[j]);
          fixed_val = std::max(root.lb[j], std::min(root.ub[j], fixed_val));
          restricted.linear_part.vars[static_cast<size_t>(j)].lb = fixed_val;
          restricted.linear_part.vars[static_cast<size_t>(j)].ub = fixed_val;
        }

        // Build integer/binary index lists
        restricted.integer_idx.clear();
        restricted.binary_idx.clear();
        for (int j = 0; j < n; ++j) {
          if (restricted.linear_part.vars[j].type == VarType::Integer) {
            restricted.integer_idx.push_back(j);
          } else if (restricted.linear_part.vars[j].type == VarType::Binary) {
            restricted.binary_idx.push_back(j);
          }
        }

        // Warm-start with current incumbent (adjusted for fixed vars)
        restricted.initial_solution = incumbent_x;

        // Solve as a HiGHS-style primal-heuristic sub-MIP consumer: inherit the
        // active root domain and LP kernel, but do not restart the root
        // cut/heuristic/proof pipeline inside the neighborhood.
        BCOptions lns_opt;
        lns_opt._is_stage3_submip = true;
        lns_opt.max_nodes = effective_lns_node_limit;
        lns_opt.max_lp_iter = opt.max_lp_iter;
        lns_opt.int_tol = opt.int_tol;
        lns_opt.gap_tol = 0.0;
        lns_opt.lp_tol = opt.lp_tol;
        lns_opt.root_cut_rounds = 0;
        lns_opt.cuts_per_round = 0;
        lns_opt.max_cut_depth = 0;
        lns_opt.cuts = CutType::None;
        lns_opt.branching = opt.branching;
        lns_opt.node_sel = opt.node_sel;
        lns_opt.use_simplex_lp_nodes = true;
        lns_opt.lp_kernel_backend = opt.lp_kernel_backend;
        lns_opt.use_ipm_root = false;
        lns_opt.use_ipm_nodes = false;
        lns_opt.use_feasibility_pump = false;
        lns_opt.enable_feasibility_jump = false;
        lns_opt.use_progressive_rounding = false;
        lns_opt.max_dive_lps = 0;
        lns_opt.max_probe_vars = 0;
        lns_opt.root_split_bound_probing = false;
        lns_opt.use_papilo_presolve = false;
        lns_opt.enable_lns = false;
        lns_opt.enable_incumbent_local_branching = false;
        lns_opt.enable_root_low_fractionality_rens = false;
        lns_opt.enable_objective_cutoff_conflict_cuts = false;
        lns_opt.enable_objective_cutoff_weighted_event_cuts = false;
        lns_opt.enable_objective_cutoff_domain_fixing = false;
        lns_opt.enable_dynamic_implied_bound_probing = false;
        lns_opt.enable_reduced_cost_proof_cut_resolve = false;
        lns_opt.num_threads = 1;
        lns_opt.verbose = false;

        // Model construction is charged to the outer solve. Recompute the
        // child allowance immediately before entering the nested B&C call.
        const double pre_solve_lns_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_lns_start).count();
        lns_sub_budget = bc_optional_subsolve_budget_sec(
            effective_lns_time_limit, bc_remaining_sec(),
            bc_post_root_finalize_reserve_sec,
            effective_lns_stage_limit - pre_solve_lns_elapsed);
        if (!(lns_sub_budget > 0.0)) break;
        lns_opt.time_limit_sec = lns_sub_budget;

        const auto lns_sub0 = std::chrono::steady_clock::now();
        BCResult lns_result = solve_milp_bc(restricted, lns_opt);
        const auto lns_sub1 = std::chrono::steady_clock::now();
        const double lns_sub_ms =
            std::chrono::duration<double, std::milli>(lns_sub1 - lns_sub0)
                .count();
        note_root_subsolve("root_lns", lns_sub_ms, lns_result.stats.success,
                           false, lns_result.bc_stats.lp_solves);

        // Check if LNS found a better solution.
        // stats.success indicates feasibility; check objective directly for improvement.
        // CRITICAL: The inner B&C solves a restricted (presolved) subproblem.
        // Its incumbent may satisfy the *reduced* model but violate the outer
        // model's constraints (inner presolve may have removed rows). Always
        // re-validate against the outer base_lp before accepting.
        if (lns_result.stats.success &&
              lns_result.x.size() == n &&
              satisfies_with_bounds(base_lp, lns_result.x, root.lb, root.ub, 1e-6) &&
              lns_result.stats.objective < incumbent_obj - 1e-6) {
            // Recompute objective in outer model's cost vector for consistency
            const double outer_obj = objective_value(base_lp.c, lns_result.x, base_lp.sense);
	            if (outer_obj < incumbent_obj - 1e-6 &&
	                adopt_root_incumbent("root_lns", lns_result.x, outer_obj)) {
	              if (opt.verbose) {
	                fmt::print(stderr, "[B&C ROOT] LNS iter={}: improved to {:.6f}\n",
	                           lns_iter, incumbent_obj);
              }
            }
        }

        // Re-sort using LP fractionality + LP-incumbent agreement
        // Add iteration-based diversification: on iter > 0, hash-permute
        // the confidence scores to explore different neighborhoods.
        // This ensures LP-integer variables (normally confidence ~1.0 and
        // always fixed) get freed on some iterations.
        for (size_t vi = 0; vi < int_vars.size(); ++vi) {
          auto& iv = int_vars[vi];
          double lp_val = root.x_relax[iv.j];
          double lp_frac = std::abs(lp_val - std::round(lp_val));
          double conf = 1.0 - lp_frac;
          double inc_rounded = std::round(incumbent_x[iv.j]);
          double lp_rounded = std::round(lp_val);
          if (std::abs(inc_rounded - lp_rounded) > 0.5)
            conf *= 0.5;
          // Diversification: on iterations > 0, add a deterministic pseudo-random
          // perturbation so different variables are freed each time
          if (lns_iter > 0) {
            // Simple hash: (var_index * 2654435761 + iter * 40503) mod 2^32, scaled to [-0.4, 0.4]
            uint32_t hash = static_cast<uint32_t>(iv.j) * 2654435761u
                          + static_cast<uint32_t>(lns_iter) * 40503u;
            double noise = (static_cast<double>(hash) / 4294967296.0 - 0.5) * 0.8;
            conf += noise;
          }
          iv.value = incumbent_x[iv.j];
          iv.confidence = conf;
        }
        std::sort(int_vars.begin(), int_vars.end(),
                  [](const IntVarConf& a, const IntVarConf& b) {
                    return a.confidence > b.confidence;
                  });
      }
    }
  }
  [[maybe_unused]] auto t_lns_done = std::chrono::steady_clock::now();
  if (opt.verbose) {
    fmt::print(stderr, "[B&C ROOT] lns={:.1f}ms has_incumbent={}\n",
            std::chrono::duration<double, std::milli>(t_lns_done - t_lns_start).count(),
            has_incumbent ? 1 : 0);
  }

  // Generic local-branching incumbent polishing.  Unlike the older UC-shaped
  // repair rules, this uses only the binary incumbent vector and the matrix:
  //   sum_{x*_j=0} x_j + sum_{x*_j=1} (1 - x_j) <= k.
  // The row is added only to a small sub-MIP, and any candidate is validated
  // against the outer model before it can replace the incumbent.
  if (opt.enable_incumbent_local_branching &&
      !opt._is_stage3_submip &&
      has_incumbent &&
      incumbent_root_rel_gap() <= std::max(10.0 * opt.gap_tol, 1.0e-3) &&
      root_frac_bin_count > 0 &&
      root_frac_bin_count <= 512 &&
      std::isfinite(incumbent_obj) &&
      base_lp.sense == Sense::Minimize) {
    const int init_radius =
        std::max(1, opt.incumbent_local_branching_initial_radius);
    const int max_radius =
        std::max(init_radius, opt.incumbent_local_branching_max_radius);
    const int max_iters =
        std::max(0, opt.incumbent_local_branching_max_iters);
    const double per_iter_limit =
        std::max(0.05, opt.incumbent_local_branching_time_limit);

    for (int iter = 0; iter < max_iters; ++iter) {
      const double remaining_s =
          std::max(0.0, opt.time_limit_sec -
                        std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - t0).count());
      const double sub_budget = std::min(per_iter_limit, 0.05 * remaining_s);
      if (sub_budget < 0.05) break;

      int radius = init_radius;
      for (int k = 0; k < iter; ++k) radius = std::min(max_radius, radius * 2);
      radius = std::min(radius, max_radius);

      MIPModel sub_prob;
      sub_prob.linear_part = base_lp;
      for (int j = 0; j < n; ++j) {
        sub_prob.linear_part.vars[static_cast<size_t>(j)].lb = root.lb[j];
        sub_prob.linear_part.vars[static_cast<size_t>(j)].ub = root.ub[j];
      }

      Eigen::SparseVector<double> lb_row(n);
      int ones = 0;
      int active_bins = 0;
      for (int j = 0; j < n; ++j) {
        if (base_lp.vars[static_cast<size_t>(j)].type != VarType::Binary) continue;
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
      if (active_bins == 0) break;
      lb_row.prune(0.0, 0.0);  // explicit (ref, eps) avoids functor deduction in Eigen >= 3.4
      const double lb_rhs = static_cast<double>(radius - ones);
      add_sparse_rows_to_lp(sub_prob.linear_part,
                            std::vector<Eigen::SparseVector<double>>{lb_row},
                            std::vector<double>{lb_rhs});

      sub_prob.integer_idx.clear();
      sub_prob.binary_idx.clear();
      for (int j = 0; j < n; ++j) {
        if (sub_prob.linear_part.vars[static_cast<size_t>(j)].type == VarType::Integer) {
          sub_prob.integer_idx.push_back(j);
        } else if (sub_prob.linear_part.vars[static_cast<size_t>(j)].type == VarType::Binary) {
          sub_prob.binary_idx.push_back(j);
        }
      }
      sub_prob.initial_solution = incumbent_x;

      BCOptions sub_opt;
      sub_opt._is_stage3_submip = true;
      sub_opt.time_limit_sec = sub_budget;
      sub_opt.max_nodes = std::max(1, opt.incumbent_local_branching_node_limit);
      sub_opt.max_lp_iter = opt.max_lp_iter;
      sub_opt.int_tol = opt.int_tol;
      sub_opt.gap_tol = 0.0;
      sub_opt.lp_tol = opt.lp_tol;
      sub_opt.root_cut_rounds = 0;
      sub_opt.cuts_per_round = 0;
      sub_opt.max_cut_depth = 0;
      sub_opt.cuts = CutType::None;
      sub_opt.branching = opt.branching;
      sub_opt.node_sel = opt.node_sel;
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

      const auto t_lb0 = std::chrono::steady_clock::now();
      BCResult sub_res = branch_and_cut_lp(sub_prob, sub_opt);
      const auto t_lb1 = std::chrono::steady_clock::now();
      const double lb_ms =
          std::chrono::duration<double, std::milli>(t_lb1 - t_lb0).count();

      bool valid_candidate = false;
      bool improved = false;
      if (sub_res.x.size() == n && std::isfinite(sub_res.stats.objective)) {
        Eigen::VectorXd xs = clamp_to_bounds(sub_res.x, root.lb, root.ub);
        for (int j = 0; j < n; ++j) {
          if (is_integer_type(base_lp.vars[static_cast<size_t>(j)])) {
            xs[j] = std::min(root.ub[j], std::max(root.lb[j], std::round(xs[j])));
          }
        }
        if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
          valid_candidate = true;
          const double obj = objective_value(base_lp.c, xs, base_lp.sense);
          improved = adopt_root_incumbent("incumbent_local_branching", xs, obj);
          if (improved && opt.verbose) {
            fmt::print(stderr,
                       "[B&C ROOT] local-branching radius={} improved to {:.2f} ({:.0f}ms)\n",
                       radius, incumbent_obj, lb_ms);
          }
        }
      }
      note_root_subsolve("incumbent_local_branching", lb_ms, valid_candidate,
                         improved, sub_res.bc_stats.lp_solves);
      if (improved && root_gap_closed()) break;
    }
  }

		  struct RootSubMipReuseResult {
		    bool found{false};
		    bool frontier_exhausted{false};
		    bool root_gap_candidate{false};
		    bool frontier_handoff_candidate{false};
		    bool node_limit_hit{false};
		    bool leaf_limit_hit{false};
		    bool time_limit_hit{false};
	    bool stall_limit_hit{false};
	    Eigen::VectorXd x;
	    double obj{kInf};
	    int nodes{0};
    int leaves{0};
    int lp_solves{0};
    int pruned_by_bound{0};
    int pruned_by_domain{0};
    int objective_cutoffs{0};
    std::string last_lp_status;
  };

  auto solve_root_submip_reusing_parent_state =
      [&](const Eigen::VectorXd& sub_lb,
          const Eigen::VectorXd& sub_ub,
          const Eigen::VectorXd& seed_x,
          const std::shared_ptr<SimplexBasis>& seed_basis,
          const LPModel* lp_override,
	          double time_budget_sec,
	          int max_nodes,
	          int max_leaves,
	          int stall_nodes,
	          double fixing_rate,
			          bool local_trail_completion,
              bool stop_on_root_gap_candidate) -> RootSubMipReuseResult {
    RootSubMipReuseResult sub_out;
    if (sub_lb.size() != n || sub_ub.size() != n ||
        !bounds_consistent(sub_lb, sub_ub) || time_budget_sec <= 0.0) {
      return sub_out;
    }

    const auto sub_t0 = std::chrono::steady_clock::now();
    const bool has_lp_override = (lp_override != nullptr);
    std::atomic<double> sub_incumbent_bound{
        has_lp_override ? kInf : (has_incumbent ? incumbent_obj : kInf)};

	    SimplexOptions sub_lp_opt;
	    sub_lp_opt.max_iter = std::max(opt.max_lp_iter * 10, 2000);
	    sub_lp_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
	    sub_lp_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    sub_lp_opt.verbose = false;
    sub_lp_opt.allow_cold_start = true;
    sub_lp_opt.factor_backend =
        simplex_factor_backend_from_id(opt.simplex_factor_backend);
    sub_lp_opt.incumbent_bound = &sub_incumbent_bound;
    sub_lp_opt.incumbent_check_interval = 5;
	    sub_lp_opt.fallback_basis = seed_basis.get();
	    sub_lp_opt.lp_kernel_backend = opt.lp_kernel_backend;

	    const LPModel& sub_lp_source = has_lp_override ? *lp_override : base_lp;
	    if (static_cast<int>(sub_lp_source.vars.size()) != n) {
	      return sub_out;
	    }
    StandardFormLP sub_sf =
        has_lp_override ? build_standard_form_lp(sub_lp_source)
                        : ensure_base_sf();
    if (has_lp_override) {
      ruiz_scale_standard_form(sub_sf);
    }
	    const Eigen::SparseMatrix<double, Eigen::RowMajor> sub_A_row = sub_lp_source.A;
	    const Eigen::SparseMatrix<double, Eigen::RowMajor> sub_Aeq_row = sub_lp_source.Aeq;
	    RowPropagationIndex sub_row_index;
	    sub_row_index.build(n, sub_A_row, sub_Aeq_row);
	    CompactPseudoCostTable sub_pc = pc;
	    NodeQueue sub_queue(local_trail_completion ? NodeSelection::DepthFirst
	                                               : NodeSelection::Hybrid);
	    sub_queue.configure_domain_signature(declared_branchable_cols, sub_lb,
	                                         sub_ub,
	                                         std::max(1e-9, opt.int_tol));
    NodePayloadSharingStats sub_payload_sharing;

    Node sub_root;
    sub_root.lb = sub_lb;
    sub_root.ub = sub_ub;
    sub_root.x_seed = seed_x.size() == n ? clamp_to_bounds(seed_x, sub_lb, sub_ub)
                                         : clamp_to_bounds(root.x_relax, sub_lb, sub_ub);
    sub_root.basis_hint = seed_basis ? seed_basis : root.basis_hint;
    sub_root.bound = root.bound;
    sub_root.depth = 0;
	    sub_queue.push(std::move(sub_root),
                   local_trail_completion ? false
                                          : std::isfinite(sub_incumbent_bound.load(
                                                std::memory_order_relaxed)));

	    double local_best = has_lp_override
                                ? kInf
                                : (has_incumbent ? incumbent_obj : kInf);
		    Eigen::VectorXd local_x;
		    int non_improving_nodes = 0;
	    const double obj_prune_tol =
	        std::max(1e-7, 1e-10 * std::max(1.0, std::abs(local_best)));

    auto elapsed_sub_sec = [&]() -> double {
      return std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - sub_t0)
          .count();
    };

    auto accept_integral_candidate = [&](const Node& node) {
      if (node.x_relax.size() != n) return false;
      Eigen::VectorXd xs = clamp_to_bounds(node.x_relax, sub_lb, sub_ub);
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[static_cast<std::size_t>(i)])) {
          continue;
        }
        xs[i] = std::min(sub_ub[i],
                         std::max(sub_lb[i], std::round(xs[i])));
      }
      if (!satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
        return false;
      }
      const double obj = objective_value(base_lp.c, xs, base_lp.sense);
      if (!std::isfinite(obj) ||
          (std::isfinite(local_best) &&
           obj >= local_best - std::max(1e-7, 1e-10 * std::abs(local_best)))) {
        return false;
      }
	      local_best = obj;
	      local_x = std::move(xs);
	      sub_incumbent_bound.store(local_best, std::memory_order_relaxed);
		      non_improving_nodes = 0;
		      if (stop_on_root_gap_candidate && std::isfinite(root.bound) &&
		          std::isfinite(opt.gap_tol) && opt.gap_tol >= 0.0) {
		        const double rel_gap =
		            std::max(0.0, (local_best - root.bound) /
		                              std::max(1.0, std::abs(local_best)));
			        if (rel_gap <= opt.gap_tol) {
			          sub_out.root_gap_candidate = true;
			        }
			        const double frontier_handoff_gap =
			            std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5);
			        if (rel_gap <= frontier_handoff_gap) {
			          sub_out.frontier_handoff_candidate = true;
			        }
			        if (sub_out.root_gap_candidate ||
			            sub_out.frontier_handoff_candidate) {
			          return true;
			        }
			      }
			      return true;
			    };

    auto solve_with_vendored_highs_submip = [&]() -> bool {
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
      if (!requested_vendored_highs_lp || local_trail_completion) return false;
      if (base_lp.sense != Sense::Minimize) return false;

      const auto h0 = std::chrono::steady_clock::now();
      Highs highs;
      highs.setOptionValue("output_flag", false);
      highs.setOptionValue("log_to_console", false);
      highs.setOptionValue("threads", 1);
      highs.setOptionValue("presolve", "on");
      highs.setOptionValue("solver", "choose");
      highs.setOptionValue("mip_lp_solver", "choose");
      highs.setOptionValue("mip_ipm_solver", "choose");
      highs.setOptionValue("mip_max_leaves",
                           static_cast<HighsInt>(std::max(1, max_leaves)));
      highs.setOptionValue("mip_max_nodes",
                           static_cast<HighsInt>(std::max(1, max_nodes)));
      highs.setOptionValue("mip_max_stall_nodes",
                           static_cast<HighsInt>(std::max(1, stall_nodes)));
      highs.setOptionValue("mip_pscost_minreliable", static_cast<HighsInt>(0));
      highs.setOptionValue("mip_detect_symmetry", false);
      highs.setOptionValue("mip_heuristic_effort", 0.8);
      highs.setOptionValue("mip_rel_gap", 0.0);
      const double curr_abs_gap =
          has_incumbent && std::isfinite(incumbent_obj) &&
                  std::isfinite(root.bound)
              ? std::max(0.0, incumbent_obj - root.bound)
              : (std::isfinite(root.bound) ? std::abs(root.bound) : 0.0);
      highs.setOptionValue("mip_abs_gap",
                           std::max(1e-6, opt.lp_tol) *
                               std::max(curr_abs_gap, 1000.0));
      highs.setOptionValue("mip_feasibility_tolerance",
                           std::max(1e-6, opt.int_tol));
      highs.setOptionValue("primal_feasibility_tolerance",
                           std::max(1e-7, opt.lp_tol));
      highs.setOptionValue("dual_feasibility_tolerance",
                           std::max(1e-7, opt.lp_tol));
      highs.setOptionValue("time_limit", std::max(0.001, time_budget_sec));
      if (has_incumbent && std::isfinite(incumbent_obj)) {
        highs.setOptionValue("objective_bound", incumbent_obj);
      }

      if (!bc_pass_mip_model_to_highs(highs, sub_lp_source, &sub_lb, &sub_ub)) {
        sub_out.last_lp_status = "HiGHS pass_error";
        return false;
      }
      HighsBasis warm_basis;
      if (bc_native_basis_to_highs_basis(sub_lp_source, seed_basis.get(),
                                         warm_basis)) {
        (void)highs.setBasis(warm_basis, "MIPSOLVERS root sub-MIP basis hint");
      }
      if (seed_x.size() == n) {
        HighsSolution hsol;
        hsol.col_value.resize(static_cast<std::size_t>(n));
        for (int j = 0; j < n; ++j) {
          hsol.col_value[static_cast<std::size_t>(j)] =
              std::min(sub_ub[j], std::max(sub_lb[j], seed_x[j]));
        }
        hsol.value_valid = true;
        hsol.dual_valid = false;
        (void)highs.setSolution(hsol);
      }

      const HighsStatus run_status = highs.run();
      const HighsModelStatus model_status = highs.getModelStatus();
      const HighsInfo& info = highs.getInfo();
      sub_out.nodes =
          static_cast<int>(std::min<int64_t>(info.mip_node_count,
                                             std::numeric_limits<int>::max()));
	      // HiGHS exposes simplex_iteration_count, not a count of LP
	      // relaxations solved inside the sub-MIP.  Native's lp_solves metric is
	      // a solve count, so account one LP relaxation for the root plus one per
	      // processed MIP node instead of misclassifying iterations as LP solves.
	      sub_out.lp_solves =
	          std::max(1, sub_out.nodes + (sub_out.nodes == 0 ? 1 : 0));
      sub_out.last_lp_status =
          fmt::format("VendoredHiGHSSubMip {} run={}",
                      bc_highs_model_status_label(model_status),
                      static_cast<int>(run_status));
      sub_out.time_limit_hit = model_status == HighsModelStatus::kTimeLimit;
      sub_out.node_limit_hit = model_status == HighsModelStatus::kSolutionLimit &&
                               sub_out.nodes >= std::max(1, max_nodes);
      sub_out.leaf_limit_hit = model_status == HighsModelStatus::kSolutionLimit;
      sub_out.stall_limit_hit = model_status == HighsModelStatus::kSolutionLimit;
      sub_out.frontier_exhausted = model_status == HighsModelStatus::kOptimal ||
                                   model_status == HighsModelStatus::kInfeasible ||
                                   model_status == HighsModelStatus::kObjectiveBound;

      const HighsSolution& sol = highs.getSolution();
      if (static_cast<int>(sol.col_value.size()) >= n) {
        Eigen::VectorXd xs = Eigen::VectorXd::Zero(n);
        for (int j = 0; j < n; ++j) {
          xs[j] = sol.col_value[static_cast<std::size_t>(j)];
        }
        Node candidate;
        candidate.lb = sub_lb;
        candidate.ub = sub_ub;
        candidate.x_relax = std::move(xs);
        candidate.bound = std::isfinite(info.mip_dual_bound)
                              ? info.mip_dual_bound
                              : root.bound;
        const bool accepted = accept_integral_candidate(candidate);
        sub_out.found = sub_out.found || accepted;
      }

      if (local_x.size() == n && std::isfinite(local_best)) {
        sub_out.x = local_x;
        sub_out.obj = local_best;
        const double rel_gap =
            std::isfinite(root.bound)
                ? std::max(0.0, (local_best - root.bound) /
                                    std::max(1.0, std::abs(local_best)))
                : kInf;
        if (rel_gap <= opt.gap_tol) sub_out.root_gap_candidate = true;
        if (rel_gap <= std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5)) {
          sub_out.frontier_handoff_candidate = true;
        }
        return true;
      }

      if (model_status == HighsModelStatus::kInfeasible ||
          model_status == HighsModelStatus::kOptimal ||
          model_status == HighsModelStatus::kObjectiveBound) {
        sub_out.leaves = 1;
        return true;
      }
      const double hsec = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - h0)
                              .count();
      if (hsec >= time_budget_sec) sub_out.time_limit_hit = true;
      return true;
#else
      (void)fixing_rate;
      return false;
#endif
    };

    if (solve_with_vendored_highs_submip()) {
      accumulate_queue_storage(
          sub_queue.domain_storage_stats(), QueueStorageSource::SubMip);
      accumulate_payload_sharing(sub_payload_sharing);
      return sub_out;
    }

		    while (!sub_queue.empty()) {
	      if (sub_out.nodes >= std::max(1, max_nodes)) {
	        sub_out.node_limit_hit = true;
	        break;
	      }
	      if (sub_out.leaves >= std::max(1, max_leaves)) {
	        sub_out.leaf_limit_hit = true;
	        break;
	      }
	      if (elapsed_sub_sec() >= time_budget_sec) {
	        sub_out.time_limit_hit = true;
	        break;
	      }
			      if (!local_trail_completion && stall_nodes > 0 && std::isfinite(local_best) &&
			          non_improving_nodes >= stall_nodes) {
		        sub_out.stall_limit_hit = true;
		        break;
		      }

      Node cur;
	      if (!sub_queue.pop(cur,
	                         local_trail_completion ? false
	                                                : std::isfinite(local_best))) break;
		      if (!local_trail_completion && std::isfinite(local_best) &&
		          incumbent_prunes_node(true, local_best, cur.bound, obj_prune_tol)) {
		        ++sub_out.pruned_by_bound;
		        ++non_improving_nodes;
	        continue;
	      }

	      const Eigen::VectorXd pre_prop_lb = cur.lb;
	      const Eigen::VectorXd pre_prop_ub = cur.ub;
	      std::vector<BoundChangeInfo> sub_changes;
	      std::vector<BranchDomainLiteral> sub_conflict;
	      int sub_tightened = 0;
	      const bool domain_ok = propagate_node_domain(
	          sub_lp_source, sub_A_row, sub_Aeq_row, sub_row_index, cur.lb, cur.ub,
	          opt.bound_propagation_rounds,
	          conflict_pool.empty() ? nullptr : &conflict_pool,
	          clique_table.empty() ? nullptr : &clique_table,
	          implication_graph.empty() ? nullptr : &implication_graph,
	          sub_changes, cur.branch_reasons, &sub_conflict,
          &sub_tightened, nullptr);
      if (!domain_ok || !bounds_consistent(cur.lb, cur.ub)) {
	        ++sub_out.pruned_by_domain;
	        ++sub_out.leaves;
	        ++non_improving_nodes;
	        continue;
	      }

		      SimplexResult lp_res;
		      if (has_lp_override && requested_vendored_highs_lp) {
		        // HiGHS' root RENS solveSubMip consumes the active HighsLp and its
		        // root basis directly; it does not rebuild the native scaled SF or
		        // require the SF audit gate.  Keep that object boundary here: build
		        // a node LP in the same canonical row/column space and let the
		        // vendored HiGHS LP kernel publish the next basis/nonbasic side.
		        LPModel node_lp = sub_lp_source;
		        apply_node_bounds(node_lp.vars, cur.lb, cur.ub);
		        BCOptions node_lp_opt = root_solve_opt;
		        node_lp_opt.use_simplex_lp_nodes = true;
		        node_lp_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
		        node_lp_opt.use_ipm_root = false;
		        node_lp_opt.use_ipm_nodes = false;
		        node_lp_opt.suppress_vendored_highs_root_frontier = false;
		        node_lp_opt.max_lp_iter =
		            std::max(opt.max_lp_iter * 10, 2000);
		        node_lp_opt.lp_tol = opt.lp_tol;
		        node_lp_opt.verbose = false;
		        LPRelaxationResult relax_res = solve_lp_relaxation(
		            node_lp,
		            cur.x_seed.size() == n ? &cur.x_seed : nullptr,
		            cur.basis_hint.get(), node_lp_opt);
		        lp_res = relax_res.simplex ? *relax_res.simplex : SimplexResult{};
		        if (!relax_res.primal.stats.success) {
		          lp_res.result = relax_res.primal;
		        }
		        if (lp_res.result.stats.status.empty()) {
		          lp_res.result.stats.status = relax_res.primal.stats.status;
		        }
		      } else {
		        update_standard_form_bounds(sub_sf, sub_lp_source, cur.lb, cur.ub);
		        lp_res = solve_lp_from_sf(sub_sf, sub_lp_opt, cur.basis_hint.get());
		      }
			      ++sub_out.lp_solves;
			      auto failure_type = classify_lp_result(lp_res, n);
			      if (!requested_vendored_highs_lp &&
			          (failure_type == LPFailureType::Infeasible ||
		           (failure_type != LPFailureType::Success &&
		            failure_type != LPFailureType::ObjectiveCutoff))) {
		        const std::string cached_status = lp_res.result.stats.status;
			        LPModel fresh_lp = sub_lp_source;
			        apply_node_bounds(fresh_lp.vars, cur.lb, cur.ub);
			        SimplexResult fresh_res =
			            solve_lp_with_basis(fresh_lp, sub_lp_opt, cur.basis_hint.get());
		        ++sub_out.lp_solves;
		        const auto fresh_type = classify_lp_result(fresh_res, n);
		        if (fresh_type == LPFailureType::Success ||
		            fresh_type == LPFailureType::ObjectiveCutoff ||
		            fresh_type == LPFailureType::Infeasible) {
		          lp_res = std::move(fresh_res);
		          failure_type = fresh_type;
		          sub_out.last_lp_status =
		              fmt::format("fresh_after_cached_{}", cached_status);
		        } else if (bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
		          sub_out.last_lp_status =
		              fmt::format("cached_{} fresh_{}",
		                          cached_status, fresh_res.result.stats.status);
		        }
		      }
		      if (failure_type == LPFailureType::Infeasible && sub_tightened > 0) {
		        // HiGHS' solveSubMip consumes a conformance-checked implication
		        // and clique state.  Native's root source table can still contain
		        // proof-derived implications whose scope is safe for the main
		        // proof path but too strong for this heuristic neighbourhood.
		        // Since RENS is only an incumbent source, keep correctness by
			        // validating any returned candidate against the outer model, and
			        // do not discard a neighbourhood solely because optional
			        // propagation made its LP infeasible.
			        cur.lb = pre_prop_lb;
			        cur.ub = pre_prop_ub;
		        SimplexResult retry_res;
		        if (has_lp_override && requested_vendored_highs_lp) {
		          LPModel retry_lp = sub_lp_source;
		          apply_node_bounds(retry_lp.vars, cur.lb, cur.ub);
		          BCOptions retry_opt = root_solve_opt;
		          retry_opt.use_simplex_lp_nodes = true;
		          retry_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
		          retry_opt.use_ipm_root = false;
		          retry_opt.use_ipm_nodes = false;
		          retry_opt.suppress_vendored_highs_root_frontier = false;
		          retry_opt.max_lp_iter =
		              std::max(opt.max_lp_iter * 10, 2000);
		          retry_opt.lp_tol = opt.lp_tol;
		          retry_opt.verbose = false;
		          LPRelaxationResult retry_relax = solve_lp_relaxation(
		              retry_lp,
		              cur.x_seed.size() == n ? &cur.x_seed : nullptr,
		              cur.basis_hint.get(), retry_opt);
		          retry_res =
		              retry_relax.simplex ? *retry_relax.simplex : SimplexResult{};
		          if (!retry_relax.primal.stats.success) {
		            retry_res.result = retry_relax.primal;
		          }
		          if (retry_res.result.stats.status.empty()) {
		            retry_res.result.stats.status = retry_relax.primal.stats.status;
		          }
		        } else {
		          update_standard_form_bounds(sub_sf, sub_lp_source, cur.lb, cur.ub);
		          retry_res =
		              solve_lp_from_sf(sub_sf, sub_lp_opt, cur.basis_hint.get());
		        }
			        ++sub_out.lp_solves;
			        const auto retry_type = classify_lp_result(retry_res, n);
		        if (retry_type == LPFailureType::Success ||
		            retry_type == LPFailureType::ObjectiveCutoff) {
		          sub_out.last_lp_status =
		              fmt::format("rens_preprop_retry_after_tight{}_{}",
		                          sub_tightened, lp_res.result.stats.status);
		          lp_res = std::move(retry_res);
		          failure_type = retry_type;
		        }
		      }
		      ++sub_out.nodes;
			      if (failure_type == LPFailureType::ObjectiveCutoff) {
			        ++sub_out.objective_cutoffs;
			        ++sub_out.leaves;
		        ++non_improving_nodes;
		        continue;
		      }
	      if (failure_type != LPFailureType::Success ||
	          lp_res.result.x.size() != n ||
	          !std::isfinite(lp_res.result.stats.objective)) {
		        sub_out.last_lp_status = lp_res.result.stats.status;
		        ++sub_out.leaves;
		        ++non_improving_nodes;
		        continue;
		      }

	      cur.x_relax = clamp_to_bounds(lp_res.result.x, cur.lb, cur.ub);
	      cur.x_seed = cur.x_relax;
	      cur.bound = std::max(cur.bound, lp_res.result.stats.objective);
		      auto cur_basis =
		          std::make_shared<SimplexBasis>(std::move(lp_res.basis));
		      cur_basis->sf_n_slack = lp_res.form.n_slack;
		      cur_basis->sf_n_surplus = lp_res.form.n_surplus;
		      cur_basis->sf_n_artificial = lp_res.form.n_artificial;
		      if (cur.basis_hint) cur_basis->try_share_indices_from(*cur.basis_hint);
		      cur_basis->compact_indices_storage();
		      if (lp_res.reduced_costs.size() > 0) {
		        cur_basis->cached_reduced_costs =
		            std::make_shared<const Eigen::VectorXd>(
		                std::move(lp_res.reduced_costs));
		      }
		      cur.basis_hint = cur_basis;

		      if (!local_trail_completion && std::isfinite(local_best) &&
		          incumbent_prunes_node(true, local_best, cur.bound, obj_prune_tol)) {
		        ++sub_out.pruned_by_bound;
		        ++sub_out.leaves;
	        ++non_improving_nodes;
	        continue;
	      }

      std::vector<int> frac;
		      if (!fractional_branchable_indices(cur.x_relax, cur.lb, cur.ub,
		                                        opt.int_tol, frac)) {
		        const bool improved = accept_integral_candidate(cur);
		        sub_out.found = sub_out.found || improved;
		        ++sub_out.leaves;
		        if (!improved) ++non_improving_nodes;
			        if (sub_out.root_gap_candidate ||
			            sub_out.frontier_handoff_candidate) {
			          break;
			        }
		        continue;
		      }

	      int branch_col = -1;
	      if (local_trail_completion) {
	        double best_key = std::numeric_limits<double>::infinity();
	        double best_frac = -1.0;
	        const std::vector<char> basic =
	            bc_original_basic_mask(n, cur.basis_hint.get());
	        for (int cand : frac) {
	          const double rc = bc_native_reduced_cost(cand, cur.basis_hint.get());
	          const double rc_abs =
	              std::isfinite(rc) ? std::abs(rc)
	                                : std::numeric_limits<double>::infinity();
	          const bool is_basic =
	              cand >= 0 && cand < static_cast<int>(basic.size()) &&
	              basic[static_cast<std::size_t>(cand)] != 0;
	          const double value = cur.x_relax[cand];
	          const double frac_part = value - std::floor(value);
	          const double dist = std::min(frac_part, 1.0 - frac_part);
	          const double key = is_basic ? rc_abs + 1.0 : rc_abs;
	          if (key < best_key - 1e-12 ||
	              (std::abs(key - best_key) <= 1e-12 &&
	               (dist > best_frac + 1e-12 ||
	                (std::abs(dist - best_frac) <= 1e-12 &&
	                 (branch_col < 0 || cand < branch_col))))) {
	            best_key = key;
	            best_frac = dist;
	            branch_col = cand;
	          }
	        }
	      } else {
	        branch_col = choose_branch_var(
	            opt, frac, cur.x_relax, sub_pc, branch_priority);
	      }
	      if (branch_col < 0) {
	        ++sub_out.leaves;
	        ++non_improving_nodes;
	        continue;
	      }

	      const double xj = cur.x_relax[branch_col];
	      const int parent_depth = cur.depth;
	      const std::uint64_t parent_domain_values =
	          static_cast<std::uint64_t>(cur.lb.size() + cur.ub.size());
	      Eigen::VectorXd parent_x_relax = std::move(cur.x_relax);
	      Node down = cur.branch_child(&sub_payload_sharing);
	      ++out.bc_stats.branch_domain_dense_copies;
	      out.bc_stats.branch_domain_dense_values_copied +=
	          parent_domain_values;
	      down.ub[branch_col] = std::min(down.ub[branch_col], std::floor(xj));
	      down.x_seed = clamp_to_bounds(parent_x_relax, down.lb, down.ub);
	      append_branch_reason(down, branch_col, false, down.ub[branch_col]);
	      Node up = std::move(cur);
	      ++out.bc_stats.branch_domain_moves;
	      up.x_relax.resize(0);
	      up.x_seed.resize(0);
	      up.ipm_iterations = 0;
	      up.lp_refresh_needed = false;
	      up.depth = parent_depth + 1;
	      up.fractional_branchables_valid = false;
	      up.lb[branch_col] = std::max(up.lb[branch_col], std::ceil(xj));
	      up.x_seed = clamp_to_bounds(parent_x_relax, up.lb, up.ub);
	      append_branch_reason(up, branch_col, true, up.lb[branch_col]);

	      if (bounds_consistent(down.lb, down.ub)) {
	        sub_queue.push(std::move(down),
	                       local_trail_completion ? false
	                                              : std::isfinite(local_best));
	      } else {
	        ++sub_out.pruned_by_domain;
	      }
		      if (bounds_consistent(up.lb, up.ub)) {
		        sub_queue.push(std::move(up),
		                       local_trail_completion ? false
		                                              : std::isfinite(local_best));
		      } else {
	        ++sub_out.pruned_by_domain;
	      }
	      if (!local_trail_completion && std::isfinite(local_best)) {
	        ++non_improving_nodes;
	      }
		      (void)fixing_rate;
		    }
	    sub_out.frontier_exhausted = sub_queue.empty();

    if (local_x.size() == n && std::isfinite(local_best) &&
	        (!has_incumbent || local_best < incumbent_obj -
                              std::max(1e-7, 1e-10 * std::abs(incumbent_obj)))) {
      sub_out.found = true;
      sub_out.obj = local_best;
      sub_out.x = std::move(local_x);
    }
    accumulate_queue_storage(sub_queue.domain_storage_stats(),
                             QueueStorageSource::SubMip);
    accumulate_payload_sharing(sub_payload_sharing);
    return sub_out;
  };

  // Generic residual RENS special case.  After a strong root cut/source loop,
  // a large MILP can have only a tiny fractional integer frontier left.  In
  // that case, fixing the integer columns that are already integral in the
  // current root LP point and solving the residual frontier sub-MIP is a
  // HiGHS-style incumbent source: it consumes the transformed root LP, the
  // current root basis and the incumbent cutoff, but it never contributes to
  // the proof unless a returned candidate is validated in the outer model.
  if (root.x_relax.size() == n) {
    int current_frac_bin = 0;
    int current_total_bin = 0;
    for (int j = 0; j < n; ++j) {
      if (base_lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) {
        continue;
      }
      ++current_total_bin;
      const double v = root.x_relax[j];
      if (v > opt.int_tol && v < 1.0 - opt.int_tol) {
        ++current_frac_bin;
      }
    }
    root_frac_bin_count = current_frac_bin;
    root_total_bin_count = current_total_bin;
  }
  auto current_large_generic_low_frac_root = [&]() -> bool {
    return large_generic_root && root_total_bin_count > 0 &&
           root_frac_bin_count <= large_generic_low_frac_cap;
  };

  int root_frontier_local_branching_attempts = 0;
  double last_root_frontier_local_branching_center_obj = kInf;
  auto run_root_frontier_local_branching_polish =
      [&](const char* reason, bool center_on_incumbent) -> bool {
    if (requested_vendored_highs_lp && !opt.enable_incumbent_local_branching) {
      if (opt.verbose || trace_bc_timeline ||
          bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
        fmt::print(stderr,
                   "[B&C-ROOT-LB-POLISH] source={} skipped="
                   "proof_contract_local_branching_not_certificate\n",
                   reason != nullptr ? reason : "root_frontier_local_branching");
      }
      return false;
    }
    if (opt._is_stage3_submip || !requested_vendored_highs_lp ||
        !current_large_generic_low_frac_root() ||
        !has_incumbent || incumbent_x.size() != n ||
        !std::isfinite(incumbent_obj) || !std::isfinite(root.bound) ||
        base_lp.sense != Sense::Minimize || root.x_relax.size() != n ||
        root_frac_bin_count <= 0 || root_frac_bin_count > 512 ||
        root_gap_closed()) {
      return false;
    }
	    if (!current_large_generic_low_frac_root() &&
	        incumbent_root_rel_gap() >
	            std::max(3.0 * opt.gap_tol, opt.gap_tol + 5.0e-5)) {
	      return false;
	    }
	    const double rel_gap_now = incumbent_root_rel_gap();
	    if (!std::isfinite(rel_gap_now) ||
	        rel_gap_now > std::max(3.0 * opt.gap_tol,
	                                opt.gap_tol + 5.0e-5)) {
	      return false;
	    }
    const int max_attempts =
        opt.enable_incumbent_local_branching
            ? std::max(0, opt.incumbent_local_branching_max_iters)
            : 1;
    if (root_frontier_local_branching_attempts >= max_attempts) {
      return false;
    }
    if (std::isfinite(last_root_frontier_local_branching_center_obj) &&
        incumbent_obj >= last_root_frontier_local_branching_center_obj -
                             std::max(1e-7, 1e-10 * std::abs(incumbent_obj))) {
      return false;
    }

    const double remaining_s =
        std::max(0.0, opt.time_limit_sec -
                          std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t0)
                              .count());
    const double requested_budget =
        opt.enable_incumbent_local_branching
            ? std::max(0.05, opt.incumbent_local_branching_time_limit)
            : std::max(0.05, std::min(1.5, 0.02 * opt.time_limit_sec));
    const double sub_budget = std::min(requested_budget, 0.04 * remaining_s);
    if (sub_budget < 0.05) return false;

    const int init_radius =
        std::max(1, opt.incumbent_local_branching_initial_radius);
    const int max_radius =
        std::max(init_radius, opt.incumbent_local_branching_max_radius);
    int radius = std::min(max_radius, std::max(init_radius, 8));
    for (int k = 0; k < root_frontier_local_branching_attempts; ++k) {
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
      double v = center_on_incumbent ? incumbent_x[j] : root.x_relax[j];
      v = std::round(std::min(root.ub[j], std::max(root.lb[j], v)));
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
    sub_prob.initial_solution =
        center_on_incumbent ? incumbent_x : root.x_relax;

    ++root_frontier_local_branching_attempts;
    last_root_frontier_local_branching_center_obj = incumbent_obj;
    ++out.bc_stats.incumbent_local_branching_attempts;
    const auto lb0 = std::chrono::steady_clock::now();
    BCOptions sub_opt;
    sub_opt._is_stage3_submip = true;
    sub_opt.time_limit_sec = sub_budget;
    sub_opt.max_nodes = opt.enable_incumbent_local_branching
        ? std::max(1, opt.incumbent_local_branching_node_limit)
        : std::max(1, std::min(500,
                               2 * opt.incumbent_local_branching_node_limit));
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
    bool valid_candidate = false;
    if (sub_res.x.size() == n && std::isfinite(sub_res.stats.objective)) {
      Eigen::VectorXd xs = clamp_to_bounds(sub_res.x, root.lb, root.ub);
      for (int j = 0; j < n; ++j) {
        if (is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
          xs[j] =
              std::min(root.ub[j], std::max(root.lb[j], std::round(xs[j])));
        }
      }
      if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
        valid_candidate = true;
        const double obj = objective_value(base_lp.c, xs, base_lp.sense);
        improved = adopt_root_incumbent(reason, xs, obj);
        if (improved) {
          ++out.bc_stats.incumbent_local_branching_improvements;
          publish_root_bound_gap_certificate(reason);
        }
      }
    }
    note_root_subsolve(reason, lb_ms, valid_candidate, improved,
                       sub_res.bc_stats.lp_solves);
    if (opt.verbose || improved || trace_bc_timeline ||
        bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
      const double rel_gap = incumbent_root_rel_gap();
      const double gap_limit_incumbent =
          root.bound / std::max(1e-12, 1.0 - std::max(0.0, opt.gap_tol));
      const double improvement_needed =
          std::isfinite(gap_limit_incumbent)
              ? std::max(0.0, incumbent_obj - gap_limit_incumbent)
              : kInf;
      fmt::print(stderr,
                 "[B&C-ROOT-LB-POLISH] source={} center={} radius={} "
                 "gap={:.3g} need={:.6g} budget={:.2f}s time={:.0f}ms "
                 "improved={} obj={:.2f}\n",
                 reason != nullptr ? reason : "root_frontier_local_branching",
                 center_on_incumbent ? "incumbent" : "root_frontier",
                 radius, rel_gap, improvement_needed, sub_budget, lb_ms,
                 improved ? 1 : 0, incumbent_obj);
    }
    return improved;
  };

	  const bool residual_rens_large_root =
	      large_generic_root || n >= opt.near_integral_n_threshold;
	  const bool residual_rens_root_gap_certified =
	      publish_root_bound_gap_certificate("root_residual_rens_gate");
		  const bool native_residual_freeze_rens_allowed =
		      !(requested_vendored_highs_lp && opt.auto_highs_root_pipeline);
		  const bool residual_rens_needs_incumbent =
		      has_incumbent &&
		      native_residual_freeze_rens_allowed &&
		      !residual_rens_root_gap_certified &&
		      ((!root_primal_polish_complete && requested_vendored_highs_lp) ||
		       incumbent_root_rel_gap() > opt.gap_tol);
  const bool residual_rens_candidate =
      residual_rens_needs_incumbent && !opt._is_stage3_submip &&
      residual_rens_large_root && root_frac_bin_count > 0 &&
      root_frac_bin_count <= 128 && std::isfinite(root.bound) &&
      root.x_relax.size() == n && root.basis_hint;
  std::string residual_rens_gate_detail;
	  if (residual_rens_candidate) {
	    residual_rens_gate_detail = "frontier=1";
	  } else if (!native_residual_freeze_rens_allowed) {
	    residual_rens_gate_detail = "skip_native_freeze_rens_highs_root_state=1";
	  } else if (residual_rens_needs_incumbent) {
    residual_rens_gate_detail = fmt::format(
        "frontier_gate=0 stage3={} large={} frac={} finite={} x={} basis={}",
        opt._is_stage3_submip ? 1 : 0, residual_rens_large_root ? 1 : 0,
        root_frac_bin_count, std::isfinite(root.bound) ? 1 : 0,
        root.x_relax.size() == n ? 1 : 0, root.basis_hint ? 1 : 0);
  } else {
    residual_rens_gate_detail = "gap_closed_or_no_incumbent=1";
  }
  trace_root_primal_gate(
      "root_residual_rens",
      residual_rens_candidate ? "run" : "skip",
      residual_rens_gate_detail.c_str());
  if (residual_rens_candidate) {
    Eigen::VectorXd rr_lb = root.lb;
	    Eigen::VectorXd rr_ub = root.ub;
	    int rr_fixed = 0;
	    int rr_free = 0;
	    bool rr_bounds_ok = true;
	    const std::vector<char> rr_root_basic =
	        bc_original_basic_mask(n, root.basis_hint.get());
	    std::vector<std::pair<double, int>> rr_degenerate_side_cols;
	    for (int j = 0; j < n; ++j) {
	      if (!is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) continue;
	      if (rr_ub[j] - rr_lb[j] <= 1e-9) {
	        ++rr_fixed;
	        continue;
	      }
	      const double v = root.x_relax[j];
	      if (is_integral(v, opt.int_tol)) {
	        const bool is_basic =
	            j >= 0 && j < static_cast<int>(rr_root_basic.size()) &&
	            rr_root_basic[static_cast<std::size_t>(j)] != 0;
	        const bool at_side =
	            std::abs(v - rr_lb[j]) <= std::max(1e-7, 10.0 * opt.int_tol) ||
	            std::abs(v - rr_ub[j]) <= std::max(1e-7, 10.0 * opt.int_tol);
	        const double rc = bc_native_reduced_cost(j, root.basis_hint.get());
	        const double rc_abs = std::isfinite(rc) ? std::abs(rc)
	                                                : std::numeric_limits<double>::infinity();
	        if (!is_basic && at_side && rc_abs <= std::max(1e-7, opt.lp_tol)) {
	          rr_degenerate_side_cols.emplace_back(
	              std::make_pair(std::abs(base_lp.c[j]), j));
	          continue;
	        }
	        const double fixval =
	            std::min(rr_ub[j], std::max(rr_lb[j], std::round(v)));
	        rr_lb[j] = fixval;
	        rr_ub[j] = fixval;
	        ++rr_fixed;
      } else {
        ++rr_free;
      }
      if (rr_lb[j] > rr_ub[j] + 1e-9) {
        rr_bounds_ok = false;
        break;
	      }
	    }
	    Eigen::VectorXd rr_pure_lb = rr_lb;
	    Eigen::VectorXd rr_pure_ub = rr_ub;
	    int rr_pure_fixed = rr_fixed;
	    const int rr_pure_free = rr_free;
	    bool rr_pure_bounds_ok = rr_bounds_ok;
	    for (const auto& item : rr_degenerate_side_cols) {
	      const int j = item.second;
	      const double fixval =
	          std::min(rr_pure_ub[j],
	                   std::max(rr_pure_lb[j], std::round(root.x_relax[j])));
	      rr_pure_lb[j] = fixval;
	      rr_pure_ub[j] = fixval;
	      ++rr_pure_fixed;
	      if (rr_pure_lb[j] > rr_pure_ub[j] + 1e-9) {
	        rr_pure_bounds_ok = false;
	        break;
	      }
	    }
	    if (rr_free > 0 && rr_free < 128 && !rr_degenerate_side_cols.empty()) {
	      std::sort(rr_degenerate_side_cols.begin(), rr_degenerate_side_cols.end(),
	                [](const auto& a, const auto& b) {
	                  if (std::abs(a.first - b.first) > 1e-12) return a.first > b.first;
	                  return a.second < b.second;
	                });
	      const int extra =
	          std::min(static_cast<int>(rr_degenerate_side_cols.size()), 128 - rr_free);
	      for (int k = 0; k < extra; ++k) {
	        ++rr_free;
	      }
	      for (int k = extra; k < static_cast<int>(rr_degenerate_side_cols.size()); ++k) {
	        const int j = rr_degenerate_side_cols[static_cast<std::size_t>(k)].second;
	        const double fixval =
	            std::min(rr_ub[j], std::max(rr_lb[j], std::round(root.x_relax[j])));
	        rr_lb[j] = fixval;
	        rr_ub[j] = fixval;
	        ++rr_fixed;
	      }
	    } else {
	      for (const auto& item : rr_degenerate_side_cols) {
	        const int j = item.second;
	        const double fixval =
	          std::min(rr_ub[j], std::max(rr_lb[j], std::round(root.x_relax[j])));
	        rr_lb[j] = fixval;
	        rr_ub[j] = fixval;
	        ++rr_fixed;
	      }
	    }
	    const std::string residual_rens_pure_trace_detail =
	        fmt::format("free={} fixed={} bounds_ok={} deg_side={} mode=pure",
	                    rr_pure_free, rr_pure_fixed,
	                    rr_pure_bounds_ok ? 1 : 0,
	                    static_cast<int>(rr_degenerate_side_cols.size()));
    trace_native_repair_bounds(
        "root_residual_rens_pure_bounds", base_lp, root_implied_integer_cols,
        root.x_relax, rr_pure_lb, rr_pure_ub, root.basis_hint.get(),
        opt.int_tol, residual_rens_pure_trace_detail.c_str());

	    const std::string residual_rens_trace_detail =
	        fmt::format("free={} fixed={} bounds_ok={} deg_side={}", rr_free,
	                    rr_fixed, rr_bounds_ok ? 1 : 0,
	                    static_cast<int>(rr_degenerate_side_cols.size()));
    trace_native_repair_bounds(
        "root_residual_rens_bounds", base_lp, root_implied_integer_cols,
        root.x_relax, rr_lb, rr_ub, root.basis_hint.get(), opt.int_tol,
        residual_rens_trace_detail.c_str());

    auto run_residual_rens_attempt =
        [&](const char* label, const Eigen::VectorXd& attempt_lb,
            const Eigen::VectorXd& attempt_ub, int attempt_free,
	            int attempt_fixed, int max_nodes, int stall_nodes) -> bool {
      if (attempt_lb.size() != n || attempt_ub.size() != n ||
          !bounds_consistent(attempt_lb, attempt_ub) || attempt_free <= 0 ||
          attempt_free > 128) {
        return false;
      }
      const double remaining_s =
          std::max(0.0, opt.time_limit_sec -
                            std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - t0).count());
	      const double budget_by_frontier =
	          0.04 * static_cast<double>(attempt_free) + 0.0025 * static_cast<double>(max_nodes);
	      const double rr_budget =
	          std::min(std::max(1.5, 0.30 * remaining_s),
	                   std::max(1.5, budget_by_frontier));
	      const auto t_rr0 = std::chrono::steady_clock::now();
	      // HiGHS' root RENS/root-reduced-cost path calls
	      // HighsPrimalHeuristics::solveSubMip as a primal heuristic consumer:
	      // it inherits the root LP/basis/domain, sets objective_bound to the
	      // incumbent upper limit, and lets the sub-MIP stop by its own
	      // max-leaf/max-node/stall limits.  It is not a proof-frontier drain.
	      const bool highs_style_submip =
	          requested_vendored_highs_lp && !opt.require_tree_exhaustion_certificate;
	      const int residual_rens_node_budget = highs_style_submip
	          ? std::max(1, std::min(max_nodes, 200))
	          : std::max(1, max_nodes);
	      const int residual_rens_leaf_budget = highs_style_submip
	          ? std::max(1, std::min(500, std::max(max_nodes, residual_rens_node_budget)))
	          : std::max(1, max_nodes);
	      const int residual_rens_stall_budget = highs_style_submip
	          ? 12
	          : std::max(1, stall_nodes);
				      RootSubMipReuseResult rr_res =
				          solve_root_submip_reusing_parent_state(
					              attempt_lb, attempt_ub, root.x_relax, root.basis_hint,
	                              nullptr, rr_budget,
					              residual_rens_node_budget,
					              residual_rens_leaf_budget,
					              residual_rens_stall_budget,
						              static_cast<double>(attempt_fixed) /
						                  std::max(1.0, static_cast<double>(attempt_fixed + attempt_free)),
						              /*local_trail_completion=*/!highs_style_submip,
		                    /*stop_on_root_gap_candidate=*/false);
      const auto t_rr1 = std::chrono::steady_clock::now();
      const double rr_ms =
          std::chrono::duration<double, std::milli>(t_rr1 - t_rr0).count();
      out.bc_stats.lp_solves += rr_res.lp_solves;

      bool rr_ok = false;
      bool rr_improved = false;
      if (rr_res.x.size() == n && std::isfinite(rr_res.obj)) {
        Eigen::VectorXd xs = clamp_to_bounds(rr_res.x, root.lb, root.ub);
        for (int j = 0; j < n; ++j) {
          if (!is_integer_type(base_lp.vars[static_cast<std::size_t>(j)])) {
            continue;
          }
          xs[j] =
              std::min(root.ub[j], std::max(root.lb[j], std::round(xs[j])));
        }
        if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
          const double obj = objective_value(base_lp.c, xs, base_lp.sense);
	          const bool improved =
	              adopt_root_incumbent(label, xs, obj);
	          note_root_subsolve(label, rr_ms, true, improved,
	                             rr_res.lp_solves);
	          rr_ok = true;
	          rr_improved = improved;
			          if (improved && requested_vendored_highs_lp &&
			              !rr_res.frontier_exhausted) {
			            const double source_gap = incumbent_root_rel_gap();
			            if (std::isfinite(source_gap)) {
			              if (source_gap <= opt.gap_tol &&
			                  !root_bound_gap_certifies_incumbent()) {
			                require_root_gap_frontier_certificate(label);
			              } else if (source_gap <= std::max(3.0 * opt.gap_tol,
			                                                opt.gap_tol + 5.0e-5)) {
			                (void)run_root_frontier_local_branching_polish(
			                    "root_frontier_handoff_local_branching",
			                    /*center_on_incumbent=*/true);
			              }
			            }
			          }
	          if (opt.verbose || improved || trace_bc_timeline ||
	              bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	            fmt::print(stderr,
	                       "[B&C RESIDUAL-RENS] phase={} free={} fixed={} obj={:.2f} "
	                       "improved={} nodes={} lps={} leaves={} exhausted={} "
	                       "gapCert={} limits=n{} l{} t{} s{} status='{}' "
	                       "({:.0f}ms)\n",
	                       label, attempt_free, attempt_fixed, obj, improved ? 1 : 0,
	                       rr_res.nodes, rr_res.lp_solves, rr_res.leaves,
		                       rr_res.frontier_exhausted ? 1 : 0,
	                         rr_res.root_gap_candidate ? 1 : 0,
	                       rr_res.node_limit_hit ? 1 : 0,
	                       rr_res.leaf_limit_hit ? 1 : 0,
	                       rr_res.time_limit_hit ? 1 : 0,
	                       rr_res.stall_limit_hit ? 1 : 0,
	                       rr_res.last_lp_status, rr_ms);
	          }
        }
      }
      if (!rr_ok) {
        note_root_subsolve(label, rr_ms, false, false, rr_res.lp_solves);
        if (opt.verbose || trace_bc_timeline ||
            bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
          fmt::print(stderr,
                     "[B&C RESIDUAL-RENS] phase={} no incumbent free={} fixed={} "
                     "nodes={} lps={} leaves={} boundPrune={} domPrune={} "
                     "objCut={} status='{}' ({:.0f}ms)\n",
                     label, attempt_free, attempt_fixed, rr_res.nodes,
                     rr_res.lp_solves, rr_res.leaves, rr_res.pruned_by_bound,
                     rr_res.pruned_by_domain, rr_res.objective_cutoffs,
                     rr_res.last_lp_status, rr_ms);
        }
      }
	      return rr_res.root_gap_candidate ||
	             rr_res.frontier_handoff_candidate ||
	             (rr_improved && root_bound_gap_certifies_incumbent());
	    };

    bool rr_improved = false;
    if (rr_pure_bounds_ok && rr_pure_free > 0 && rr_pure_free <= 128) {
      rr_improved = run_residual_rens_attempt(
          "root_residual_rens_pure", rr_pure_lb, rr_pure_ub, rr_pure_free,
          rr_pure_fixed, /*max_nodes=*/1024, /*stall_nodes=*/1024);
    }
    if (!rr_improved && rr_bounds_ok && rr_free > 0 && rr_free <= 128) {
      const bool expanded_improved = run_residual_rens_attempt(
	          "root_residual_rens_expanded", rr_lb, rr_ub, rr_free, rr_fixed,
	          /*max_nodes=*/1024, /*stall_nodes=*/1024);
	      if (expanded_improved && rr_free > rr_pure_free) {
      }
    }
  }

  if (!root_gap_closed()) {
    (void)run_root_frontier_local_branching_polish(
        "root_post_residual_local_branching",
        /*center_on_incumbent=*/true);
  }

  // Generic low-fractionality RENS.  HiGHS does not freeze the whole
  // root-LP-integral complement.  It first performs a local relaxation-enforced
  // dive until a target fixing rate is reached, then solves a sub-MIP on the
  // current transformed/root LP context.  The native implementation mirrors
  // that shape: root_lp (with admitted transformed rows and root domain) is the
  // sub-MIP source, the fixing rate is about 60%, and the incumbent objective
  // is consumed as a local objective bound rather than by relaxing the proof
  // gap.
	  const double root_rens_gap_now =
	      has_incumbent ? incumbent_root_rel_gap() : kInf;
	  const bool highs_root_rens_slot =
	      requested_vendored_highs_lp && opt.auto_highs_root_pipeline &&
	      !root_primal_polish_complete;
	  const bool native_root_rens_needs_incumbent =
	      !requested_vendored_highs_lp &&
	      (!has_incumbent || root_rens_gap_now > opt.gap_tol);
	  const bool root_rens_should_run =
	      highs_root_rens_slot || native_root_rens_needs_incumbent ||
	      root_proof_tail_polish_required();
	  const std::string root_rens_gate_detail =
	      highs_root_rens_slot
	          ? "highs_root_heuristic_slot=1"
	          : (native_root_rens_needs_incumbent
	                 ? "native_needs_incumbent=1"
	                 : (root_proof_tail_polish_required()
	                        ? "strict_tree_polish=1"
	                        : (has_incumbent ? "incumbent_gap_closed=1"
	                                         : "no_highs_root_slot=1")));
	  trace_root_primal_gate(
	      "root_low_fractionality_rens",
	      (root_rens_should_run && !opt._is_stage3_submip &&
	       opt.enable_root_low_fractionality_rens &&
	       root_frac_bin_count > 0 &&
	       std::isfinite(root.bound))
	          ? "run"
	          : "skip",
	      opt.enable_root_low_fractionality_rens
	          ? root_rens_gate_detail.c_str()
	          : "disabled=1");
	  if (root_rens_should_run && !opt._is_stage3_submip &&
	      opt.enable_root_low_fractionality_rens &&
	      root_frac_bin_count > 0 &&
	      std::isfinite(root.bound)) {
    const double remaining_s =
        std::max(0.0, opt.time_limit_sec -
                      std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - t0).count());
    const double rens_budget =
        std::min(opt.root_low_fractionality_rens_time_limit, 0.05 * remaining_s);
    if (rens_budget >= 0.10) {
      const double initial_target_fixing_rate =
          std::min(0.95, std::max(0.10,
                                  opt.root_low_fractionality_rens_target_fixing_rate));
      double attempt_target_fixing_rate = initial_target_fixing_rate;
      const bool trace_rens =
          opt.verbose || trace_bc_timeline ||
          bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr;
      for (int rens_attempt = 0; rens_attempt < 3; ++rens_attempt) {
      bool rens_attempt_had_candidate = false;
      std::vector<int> eligible_ints;
      eligible_ints.reserve(static_cast<std::size_t>(root_total_bin_count));
	      for (int i = 0; i < n; ++i) {
	        if (!is_integer_type(base_lp.vars[static_cast<size_t>(i)])) continue;
	        if (root.ub[i] - root.lb[i] <= 1e-9) continue;
	        eligible_ints.push_back(i);
	      }
          auto literal_degree = [&](int col, bool value_one) -> int {
            auto range = clique_table.literal_neighbours(col, value_one);
            if (range.first == nullptr || range.second == nullptr) return 0;
            return static_cast<int>(range.second - range.first);
          };
          std::sort(eligible_ints.begin(), eligible_ints.end(),
                    [&](int a, int b) {
                      const double lock_a =
                          (opt.lp_tol + static_cast<double>(uplocks[a])) *
                          (opt.lp_tol + static_cast<double>(downlocks[a]));
                      const double lock_b =
                          (opt.lp_tol + static_cast<double>(uplocks[b])) *
                          (opt.lp_tol + static_cast<double>(downlocks[b]));
                      if (lock_a != lock_b) return lock_a > lock_b;
                      const double clq_a =
                          (opt.lp_tol + static_cast<double>(literal_degree(a, true))) *
                          (opt.lp_tol + static_cast<double>(literal_degree(a, false)));
                      const double clq_b =
                          (opt.lp_tol + static_cast<double>(literal_degree(b, true))) *
                          (opt.lp_tol + static_cast<double>(literal_degree(b, false)));
                      if (clq_a != clq_b) return clq_a > clq_b;
                      const std::uint64_t ha =
                          std::hash<std::uint64_t>{}(static_cast<std::uint64_t>(a));
                      const std::uint64_t hb =
                          std::hash<std::uint64_t>{}(static_cast<std::uint64_t>(b));
                      if (ha != hb) return ha > hb;
                      return a > b;
                    });

	      const int n_eligible = static_cast<int>(eligible_ints.size());
      const double target_fixing_rate = attempt_target_fixing_rate;
	      Eigen::VectorXd rens_lb = root.lb;
	      Eigen::VectorXd rens_ub = root.ub;
	      Eigen::VectorXd rens_x = root.x_relax;
	      std::shared_ptr<SimplexBasis> rens_basis_hint = root.basis_hint;
	      std::vector<char> rens_fixed(static_cast<std::size_t>(n), 0);
      int n_fixed = 0;

      auto mark_new_fixed = [&]() {
        for (int col : eligible_ints) {
          if (rens_fixed[static_cast<std::size_t>(col)]) continue;
          if (rens_ub[col] - rens_lb[col] <= 1e-9) {
            rens_fixed[static_cast<std::size_t>(col)] = 1;
            ++n_fixed;
          }
        }
      };

      auto tighten_rens_bound = [&](int col, double new_lb, double new_ub) {
        new_lb = std::max(new_lb, root.lb[col]);
        new_ub = std::min(new_ub, root.ub[col]);
        if (new_lb > new_ub + 1e-9) return false;
        bool changed = false;
        if (new_lb > rens_lb[col] + 1e-9) {
          rens_lb[col] = new_lb;
          changed = true;
        }
        if (new_ub < rens_ub[col] - 1e-9) {
          rens_ub[col] = new_ub;
          changed = true;
        }
        if (changed && rens_ub[col] - rens_lb[col] <= 1e-9) {
          rens_lb[col] = rens_ub[col] =
              std::min(root.ub[col], std::max(root.lb[col],
                                              std::round(0.5 * (rens_lb[col] + rens_ub[col]))));
        }
        return changed;
      };

      auto rens_fix_value = [&](int col, double fracval) {
        const double rootchange =
            (root.x_relax.size() == n) ? fracval - root.x_relax[col] : 0.0;
        double fixval = std::floor(fracval + 0.5);
        if (rootchange >= 0.4) {
          fixval = std::ceil(fracval);
        } else if (rootchange <= -0.4) {
          fixval = std::floor(fracval);
        } else if (base_lp.c[col] > 0.0) {
          fixval = std::ceil(fracval);
        } else if (base_lp.c[col] < 0.0) {
          fixval = std::floor(fracval);
        }
        return std::min(rens_ub[col], std::max(rens_lb[col], fixval));
      };

      // Berthold (2006), Sec. 3.5; Achterberg (2007), Sec. 9.4; derivation in
      // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
      // RENS fixes integer variables that are integral in the root relaxation
      // directly. Re-solving the unchanged relaxation between partial batches
      // does not define a stronger neighbourhood and is prohibitive on large
      // roots, so use LP rounds only when the integral set cannot reach the
      // target fixing rate.
      const int target_fixed = static_cast<int>(std::ceil(
          target_fixing_rate * static_cast<double>(n_eligible)));
      for (int col : eligible_ints) {
        if (n_fixed >= target_fixed) break;
        const double value = root.x_relax[col];
        if (!is_integral(value, opt.int_tol)) continue;
        const double fixed_value =
            std::min(rens_ub[col], std::max(rens_lb[col], std::round(value)));
        if (tighten_rens_bound(col, fixed_value, fixed_value)) {
          mark_new_fixed();
        }
      }

      LPModel rens_relax_lp = root_lp;
      apply_node_bounds(rens_relax_lp.vars, rens_lb, rens_ub);
      const Eigen::SparseMatrix<double, Eigen::RowMajor> rens_A_row = root_lp.A;
      const Eigen::SparseMatrix<double, Eigen::RowMajor> rens_Aeq_row = root_lp.Aeq;
      const auto t_rens_dive0 = std::chrono::steady_clock::now();
      const int max_rens_dive_rounds = 12;
      bool rens_domain_infeasible = false;
      int rens_dive_lps = 0;
      int rens_dive_rounds = 0;

      for (int round = 0; round < max_rens_dive_rounds && n_eligible > 0; ++round) {
        mark_new_fixed();
        const double fixing_rate =
            static_cast<double>(n_fixed) / static_cast<double>(n_eligible);
        if (fixing_rate >= target_fixing_rate) break;
        const double stop_rate =
            std::min(target_fixing_rate, 1.0 - (1.0 - fixing_rate) * 0.9);
        const int stop_fixed = std::max(
            n_fixed + 1,
            static_cast<int>(std::ceil(stop_rate * static_cast<double>(n_eligible))));

        int changed = 0;
        for (int col : eligible_ints) {
          if (n_fixed >= stop_fixed) break;
          if (rens_ub[col] - rens_lb[col] <= 1e-9) continue;
          const double v = (rens_x.size() == n) ? rens_x[col] : root.x_relax[col];
          double down = std::floor(v + opt.int_tol);
          double up = std::ceil(v - opt.int_tol);
          down = std::min(down, rens_ub[col]);
          up = std::max(up, rens_lb[col]);
          if (tighten_rens_bound(col, down, up)) {
            ++changed;
            mark_new_fixed();
          }
        }

        if (changed == 0) {
          std::vector<std::pair<double, int>> fractional;
          fractional.reserve(eligible_ints.size());
          for (int col : eligible_ints) {
            if (rens_ub[col] - rens_lb[col] <= 1e-9) continue;
            const double v = (rens_x.size() == n) ? rens_x[col] : root.x_relax[col];
            const double frac = std::abs(v - std::round(v));
            if (frac <= opt.int_tol) continue;
            const double fv = rens_fix_value(col, v);
            fractional.push_back({std::abs(fv - v), col});
          }
          std::sort(fractional.begin(), fractional.end(),
                    [](const auto& a, const auto& b) {
                      if (a.first != b.first) return a.first < b.first;
                      const std::uint64_t ha =
                          static_cast<std::uint64_t>(a.second) * 11400714819323198485ull;
                      const std::uint64_t hb =
                          static_cast<std::uint64_t>(b.second) * 11400714819323198485ull;
                      return ha < hb;
                    });
          std::vector<int> rens_candidate_cols;
          rens_candidate_cols.reserve(fractional.size());
          for (const auto& item : fractional) rens_candidate_cols.push_back(item.second);
          const std::string rens_candidate_trace_detail =
              fmt::format("attempt={} round={} fixing_rate={:.6g} stop_fixed={} "
                          "fixed={} eligible={}",
                          rens_attempt, round, fixing_rate, stop_fixed, n_fixed,
                          n_eligible);
          trace_native_repair_candidate_order(
              "root_low_fractionality_rens_candidates", base_lp,
              root_implied_integer_cols, rens_x, rens_lb, rens_ub,
              rens_basis_hint.get(), rens_candidate_cols, opt.int_tol,
              rens_candidate_trace_detail.c_str());
          double total_change = 0.0;
          for (const auto& [dist, col] : fractional) {
            (void)dist;
            if (n_fixed >= stop_fixed) break;
            const double v = (rens_x.size() == n) ? rens_x[col] : root.x_relax[col];
            const double fv = rens_fix_value(col, v);
            if (tighten_rens_bound(col, fv, fv)) {
              ++changed;
              total_change += std::abs(fv - v);
              mark_new_fixed();
            }
            if (total_change >= 0.5) break;
          }
        }

        if (changed == 0) break;
        apply_node_bounds(rens_relax_lp.vars, rens_lb, rens_ub);
        node_bound_propagation(root_lp, rens_A_row, rens_Aeq_row, rens_lb,
                               rens_ub, opt.bound_propagation_rounds);
        if (!clique_table.empty()) clique_table.propagate(rens_lb, rens_ub);
        if (!bounds_consistent(rens_lb, rens_ub)) {
          rens_domain_infeasible = true;
          break;
        }
        mark_new_fixed();
        apply_node_bounds(rens_relax_lp.vars, rens_lb, rens_ub);
        if (static_cast<double>(n_fixed) / static_cast<double>(n_eligible) >=
            target_fixing_rate) {
          break;
        }
        const double spent = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_rens_dive0).count();
        if (spent >= 0.35 * rens_budget) break;
	        LPRelaxationResult rens_lp_res = solve_lp_relaxation(
	            rens_relax_lp, &rens_x, root.basis_hint.get(), root_solve_opt);
	        ++out.bc_stats.lp_solves;
	        ++rens_dive_lps;
	        if (!rens_lp_res.primal.stats.success ||
	            rens_lp_res.primal.x.size() != n) {
              rens_domain_infeasible = true;
	          break;
	        }
	        rens_x = clamp_to_bounds(rens_lp_res.primal.x, rens_lb, rens_ub);
	        if (rens_lp_res.basis_hint) {
	          rens_basis_hint = rens_lp_res.basis_hint;
	          rens_basis_hint->compact_indices_storage();
	        }
	        ++rens_dive_rounds;
	      }
      mark_new_fixed();
      const int n_free = n_eligible - n_fixed;
      const std::string lowfrac_rens_trace_detail =
          fmt::format("attempt={} target={:.3g} free={} fixed={} eligible={} "
                      "dive_rounds={} lp_rounds={} infeas={}",
                      rens_attempt, target_fixing_rate, n_free, n_fixed,
                      n_eligible, rens_dive_rounds, rens_dive_lps,
                      rens_domain_infeasible ? 1 : 0);
      trace_native_repair_bounds(
          "root_low_fractionality_rens_bounds", base_lp,
          root_implied_integer_cols, root.x_relax, rens_lb, rens_ub,
          rens_basis_hint.get(), opt.int_tol,
          lowfrac_rens_trace_detail.c_str());

      if (trace_rens) {
        fmt::print(stderr,
                   "[B&C RENS] attempt={} dive target={:.2f} fixed={} free={} "
                   "eligible={} dive_rounds={} lp_rounds={} infeas={} budget={:.2f}s\n",
                   rens_attempt, target_fixing_rate, n_fixed, n_free, n_eligible,
                   rens_dive_rounds, rens_dive_lps,
                   rens_domain_infeasible ? 1 : 0,
                   rens_budget);
      }

	      if (!rens_domain_infeasible && n_free > 0 &&
	          (opt.root_low_fractionality_rens_max_free_ints <= 0 ||
	           n_free <= opt.root_low_fractionality_rens_max_free_ints)) {
	        const auto t_rens0 = std::chrono::steady_clock::now();
		        RootSubMipReuseResult sub_res =
			            solve_root_submip_reusing_parent_state(
			                rens_lb, rens_ub, rens_x, rens_basis_hint, &rens_relax_lp,
	                        rens_budget,
				                opt.root_low_fractionality_rens_max_nodes,
				                /*max_leaves=*/500, /*stall_nodes=*/12,
				                target_fixing_rate,
				                /*local_trail_completion=*/false,
                        /*stop_on_root_gap_candidate=*/false);
	        const auto t_rens1 = std::chrono::steady_clock::now();
	        const double rens_ms =
	            std::chrono::duration<double, std::milli>(t_rens1 - t_rens0).count();

	        bool rens_ok = false;
	        out.bc_stats.lp_solves += sub_res.lp_solves;
	        if (sub_res.x.size() == n && std::isfinite(sub_res.obj)) {
	          Eigen::VectorXd xs = clamp_to_bounds(sub_res.x, root.lb, root.ub);
	          for (int i = 0; i < n; ++i) {
	            if (!is_integer_type(base_lp.vars[static_cast<size_t>(i)])) continue;
	            xs[i] = std::min(root.ub[i], std::max(root.lb[i], std::round(xs[i])));
	          }
	          if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
	            const double obj = sub_res.obj;
	            const bool improved = adopt_root_incumbent("root_low_fractionality_rens", xs, obj);
	            note_root_subsolve("root_low_fractionality_rens", rens_ms, true,
	                               improved, sub_res.lp_solves);
	            rens_ok = true;
	            rens_attempt_had_candidate = true;
	            if (trace_rens) {
	              fmt::print(stderr,
	                         "[B&C RENS] attempt={} root candidate n_free={} "
	                         "n_fixed={} obj={:.2f} improved={} nodes={} "
	                         "lps={} leaves={} boundPrune={} domPrune={} "
	                         "objCut={} status='{}' ({:.0f}ms)\n",
	                         rens_attempt, n_free, n_fixed, obj,
	                         improved ? 1 : 0, sub_res.nodes,
	                         sub_res.lp_solves, sub_res.leaves,
	                         sub_res.pruned_by_bound,
	                         sub_res.pruned_by_domain,
	                         sub_res.objective_cutoffs,
	                         sub_res.last_lp_status, rens_ms);
	            }
	            if (improved || root_gap_closed()) break;
	          }
	        }
	        if (!rens_ok) {
	          note_root_subsolve("root_low_fractionality_rens", rens_ms, false,
	                             false, sub_res.lp_solves);
	          if (trace_rens) {
	            fmt::print(stderr,
	                       "[B&C RENS] attempt={} no incumbent n_free={} "
	                       "n_fixed={} nodes={} lps={} leaves={} "
	                       "boundPrune={} domPrune={} objCut={} status='{}' "
	                       "budget={:.2f}s ({:.0f}ms)\n",
	                       rens_attempt, n_free, n_fixed, sub_res.nodes,
	                       sub_res.lp_solves, sub_res.leaves,
	                       sub_res.pruned_by_bound,
	                       sub_res.pruned_by_domain,
	                       sub_res.objective_cutoffs,
	                       sub_res.last_lp_status, rens_budget, rens_ms);
	          }
	        }
      } else if (trace_rens && !rens_domain_infeasible) {
        fmt::print(stderr,
                   "[B&C RENS] attempt={} skipped sub-MIP free={} cap={} fixed={}\n",
                   rens_attempt, n_free,
                   opt.root_low_fractionality_rens_max_free_ints, n_fixed);
      }
      if (root_gap_closed() || rens_attempt_had_candidate ||
          attempt_target_fixing_rate <= 0.11) {
        break;
      }
      attempt_target_fixing_rate =
          std::max(0.10, 0.5 * attempt_target_fixing_rate);
      }
    }
  }
  root_primal_polish_complete = true;

  run_root_incumbent_propagation("after_root_incumbent");

	  // ════════════════════════════════════════════════════════════════════════
  // Root probing — two phases:
  //   Phase 1: Domain propagation probing (fast, all binaries).
  //            Fix x[j]=0/1, run bound propagation. If inconsistent → fix.
  //   Phase 2: LP probing (expensive, top fractional candidates only).
  //            Solve LPs to get pseudocost data and detect LP-infeasible fixings.
  // ════════════════════════════════════════════════════════════════════════
  // Skip probing when concurrent tree is running: probing modifies root.lb/ub
  // which would race with tree threads, and its bound-tightening benefits can't
  // propagate to tree nodes already created from the original root.
  const int root_probe_rows =
      static_cast<int>(base_lp.A.rows() + base_lp.Aeq.rows());
  // Savelsbergh (1994), Secs. 3-4; Achterberg (2007), Sec. 3.3; measured
  // entry-cost derivation in
  // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
  // The stateful domain workspace needs an O(nnz) baseline closure before its
  // first cancellable probing world, independently of simplex basis ownership.
  const bool skip_short_large_root_probing =
      root_probe_rows > 10000 &&
      opt.time_limit_sec > 0.0 && opt.time_limit_sec <= 10.0;
  if (skip_short_large_root_probing && opt.verbose) {
    fmt::print(stderr,
               "[B&C PROBE] skipped standalone probing on short-budget "
               "large root\n");
  }
  // This scope also contains reduced-cost/cutoff follow-up that consumes the
  // probe LP state. On a short large root, entering its propagation prefix can
  // still reactivate that coupled tail after the local probing budget expires.
  if (!skip_short_large_root_probing && !concurrent_tree_launched &&
      !root_gap_closed() &&
      !bc_optional_root_budget_expired())
  {
    int fixed_by_probe = 0;
    bool root_domain_probe_changed_root = false;

    // Phase 1: Domain propagation probing — probe ALL unfixed binaries.
    // Each probe: fix variable → run bound propagation → check infeasibility.
    // Skip when we already have an incumbent — the benefit is rarely worth the cost.
    // Also skip for large problems (>1000 binaries) where the O(nbinary × n) cost
    // dominates and empirically fixes 0 variables on UC-type formulations.
    const int n_unfixed_bins = [&]() {
      int c = 0;
      for (int j = 0; j < n; ++j) {
        if (!is_integer_type(base_lp.vars[j])) continue;
        if (std::abs(root.ub[j] - root.lb[j]) < 1e-9) continue;
        ++c;
      }
      return c;
    }();
	    if (!has_incumbent && n_unfixed_bins <= 1000) {
	      auto t_dp_start = std::chrono::steady_clock::now();
	      // Savelsbergh (1994), Secs. 3-4; Achterberg (2007), Sec. 3.3;
	      // deadline derivation in
	      // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
	      // A completed probing world is an independent valid deduction.  Bound
	      // optional root probing by 5% of the solve limit and stop only between
	      // worlds, leaving the existing post-root finalization reserve intact.
	      const double domain_probe_time_budget =
	          opt.time_limit_sec > 0.0 && std::isfinite(opt.time_limit_sec)
	              ? std::min(0.05 * opt.time_limit_sec,
	                         std::max(0.0, bc_remaining_sec() -
	                                           bc_post_root_finalize_reserve_sec))
	              : std::numeric_limits<double>::infinity();
	      auto domain_probe_budget_expired = [&]() {
	        return bc_optional_root_budget_expired() ||
	               std::chrono::duration<double>(
	                   std::chrono::steady_clock::now() - t_dp_start).count() >=
	                   domain_probe_time_budget;
	      };
	      int dp_fixed = 0;
	      int dp_count = 0;
	      std::uint64_t dp_conflict_sources = 0;
	      std::uint64_t dp_implication_sources = 0;

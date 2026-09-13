// @file bc_run/07_root_heuristics_b.inc
// BCSolveState::run() segment 7/14: root heuristics — part 2 (parent-state sub-MIP reuse, rounding drivers).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
        if (attempt == 1 && j < static_cast<int>(uplocks.size())) {
          if (uplocks[j] == 0 && downlocks[j] > 0) val = std::ceil(xj[j]);
          else if (downlocks[j] == 0 && uplocks[j] > 0) val = std::floor(xj[j]);
        } else if (attempt == 2) {
          val = (base_lp.c[j] >= 0.0) ? root.lb[j] : root.ub[j];
        }
        xj[j] = std::min(root.ub[j], std::max(root.lb[j], val));
      }

      Eigen::VectorXd row_act = base_lp.A * xj;
      Eigen::VectorXd eq_act = base_lp.Aeq * xj;
      const int max_flips = std::max(0, opt.feasibility_jump_max_flips);
      for (int flip = 0; flip < max_flips; ++flip) {
        // Optional root work must leave the existing finalization reserve.
        // Budget contract: docs/solvers.md, Fixed-Integer Root Repair Kernel and Budget.
        if (bc_optional_root_budget_expired()) return false;
        double best_gain = 1e-9;
        int best_j = -1;
        double best_delta = 0.0;
        for (int j : bin_cands) {
          const double lo = root.lb[j];
          const double hi = root.ub[j];
          const double mid = 0.5 * (lo + hi);
          const double new_val = (xj[j] <= mid) ? hi : lo;
          const double delta = new_val - xj[j];
          if (std::abs(delta) <= 1e-12) continue;

          double gain = 0.0;
          for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, j); it; ++it) {
            const int r = it.row();
            const double old_v = row_merit(row_act[r], base_lp.b[r]);
            const double new_v = row_merit(row_act[r] + it.value() * delta, base_lp.b[r]);
            gain += old_v - new_v;
          }
          for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, j); it; ++it) {
            const int r = it.row();
            const double old_v = eq_merit(eq_act[r], base_lp.beq[r]);
            const double new_v = eq_merit(eq_act[r] + it.value() * delta, base_lp.beq[r]);
            gain += old_v - new_v;
          }
          if (gain > best_gain) {
            best_gain = gain;
            best_j = j;
            best_delta = delta;
          }
        }
        if (best_j < 0) break;
        xj[best_j] += best_delta;
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, best_j); it; ++it) {
          row_act[it.row()] += it.value() * best_delta;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, best_j); it; ++it) {
          eq_act[it.row()] += it.value() * best_delta;
        }
      }

      if (try_fixed_integer_repair("feasibility_jump", xj)) {
        if (opt.verbose) {
          fmt::print(stderr, "[B&C] feasibility-jump incumbent: obj={:.2f}\n",
                     incumbent_obj);
        }
        return true;
      }
    }
    return false;
  };

  if (!strict_highs_lp_contract) {
    run_feasibility_jump();
  }
  auto t_pp_d = std::chrono::steady_clock::now();
  auto t_pp_d2 = t_pp_d;  // after quick_round
  auto t_pp_d3 = t_pp_d;  // after linesearch_rounding
  auto t_pp_d4 = t_pp_d;  // after fixed-pump (small LP) block

  // ── Quick rounding heuristic after root LP ─────────────────────────
  // Always attempt simple rounding regardless of pump setting — it's cheap.
  // When UC hints are available, use commitment-aware rounding first.
	  if (!strict_highs_lp_contract &&
      !root_gap_closed() && !skip_large_near_integral_quick_round &&
	      !skip_pump_large_uc_like) {
    Eigen::VectorXd xr = root.x_relax;

    // UC-aware rounding: round IG respecting min-up/down, derive SU/SD.
    if (reduced_uc_hint.has_value()) {
      uc_round_commitment(xr, *reduced_uc_hint, reduced_uc_hint->ng);
    }

    // Round remaining integer variables (non-UC binaries, general integers).
    // When lock-count rounding is enabled, apply HiGHS-style lock bias:
    //   uplocks[j]==0 && downlocks[j]>0  → ceil  (free to go up)
    //   downlocks[j]==0 && uplocks[j]>0  → floor (free to go down)
    //   otherwise                         → nearest (unchanged)
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(base_lp.vars[i])) continue;
      double rounded;
      if (opt.use_lock_count_rounding && i < static_cast<int>(uplocks.size())) {
        if (uplocks[i] == 0 && downlocks[i] > 0) {
          rounded = std::ceil(xr[i]);
        } else if (downlocks[i] == 0 && uplocks[i] > 0) {
          rounded = std::floor(xr[i]);
        } else {
          rounded = std::round(xr[i]);
        }
      } else {
        rounded = std::round(xr[i]);
      }
      xr[i] = std::min(root.ub[i], std::max(root.lb[i], rounded));
    }

    // Fix continuous variables by solving the LP with integers fixed.
    // Use direct simplex warm-start from root basis (avoid expensive
    // solve_lp_relaxation which goes through IPM→crossover→simplex pipeline).
    if (!satisfies_with_bounds(base_lp, xr, root.lb, root.ub, 1e-6)) {
      // Quick feasibility check: skip expensive LP if too many constraints violated.
      int n_viol = 0;
      {
        Eigen::VectorXd Ax = base_lp.A * xr;
        for (int r = 0; r < static_cast<int>(Ax.size()); ++r)
          if (Ax[r] > base_lp.b[r] + 1e-4) ++n_viol;
        Eigen::VectorXd Aeqx = base_lp.Aeq * xr;
        for (int r = 0; r < static_cast<int>(Aeqx.size()); ++r)
          if (std::abs(Aeqx[r] - base_lp.beq[r]) > 1e-4) ++n_viol;
      }
      const int total_cons = static_cast<int>(base_lp.A.rows() + base_lp.Aeq.rows());
      const bool repair_likely = n_viol < std::max(10, total_cons / 5);
      const bool skip_large_near_integral_quick_repair =
          large_near_integral_root && root.basis_hint;
      const bool skip_large_generic_quick_repair =
          large_generic_low_frac_root;
      if (repair_likely && !skip_large_near_integral_quick_repair &&
          !skip_large_generic_quick_repair) {
      // Build fixed-integer bounds.
      Eigen::VectorXd repair_lb = root.lb, repair_ub = root.ub;
      for (int i = 0; i < n; ++i) {
        if (is_integer_type(base_lp.vars[i])) {
          repair_lb[i] = xr[i];
          repair_ub[i] = xr[i];
        }
      }
      // For large problems, use IPM-based repair (handles lb==ub fixed vars).
      // For smaller problems, use a simplex warm-start from the root basis.
      // The elapsed-time guard reserves the remaining budget for tree search;
      // simplex repair remains available after the IPM branch is skipped.
      const double t_elapsed_pre_repair =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      const bool skip_ipm_repair_by_time =
          (t_elapsed_pre_repair > 0.25 * opt.time_limit_sec);
      if (total_cons > 10000 && opt.use_ipm_root && !skip_ipm_repair_by_time) {
        // IPM repair: solves with fixed-integer bounds via IPM which handles
        // lb==ub variables correctly (unlike simplex cold-start on large LPs).
        IPMLPOptions repair_ipm_opt;
        repair_ipm_opt.max_iter = (presolved_m > 25000) ? 200 : 100;
        repair_ipm_opt.tol_primal = 1e-7;
        repair_ipm_opt.tol_dual = 1e-7;
        repair_ipm_opt.tol_gap = 1e-7;
        repair_ipm_opt.verbose = false;
        NativeIPMLPAdapter repair_ipm(repair_ipm_opt);
        auto t_quick_repair0 = std::chrono::steady_clock::now();
        auto repair_res = repair_ipm.solve_node_lp(base_lp, repair_lb, repair_ub, root.x_relax);
        auto t_quick_repair1 = std::chrono::steady_clock::now();
        out.bc_stats.lp_solves += 1;
        note_root_subsolve(
            "quick_round_repair_ipm",
            std::chrono::duration<double, std::milli>(t_quick_repair1 - t_quick_repair0).count(),
            repair_res.stats.success && repair_res.x.size() >= n);
        if (repair_res.stats.success && repair_res.x.size() >= n) {
          xr = clamp_to_bounds(repair_res.x.head(n), root.lb, root.ub);
          for (int i = 0; i < n; ++i) {
            if (is_integer_type(base_lp.vars[i])) {
              xr[i] = std::round(xr[i]);
              xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
            }
          }
        }
      } else {
      // Warm-start simplex from root basis on the same standard form.
      StandardFormLP repair_sf = ensure_base_sf();
      update_standard_form_bounds(repair_sf, base_lp, repair_lb, repair_ub);
      SimplexOptions repair_opt;
      repair_opt.lp_kernel_backend = opt.lp_kernel_backend;
      repair_opt.max_iter = std::min(200, static_cast<int>(ensure_base_sf().A.rows()) / 10);
      repair_opt.feasibility_tol = 1e-7;
      repair_opt.optimality_tol = 1e-7;
      repair_opt.verbose = false;
      repair_opt.allow_cold_start = false;  // Fail fast; don't waste time on cold start
      auto t_quick_repair0 = std::chrono::steady_clock::now();
      auto repair_res = solve_lp_from_sf(repair_sf, repair_opt, root.basis_hint.get());
      auto t_quick_repair1 = std::chrono::steady_clock::now();
      out.bc_stats.lp_solves += 1;
      note_root_subsolve(
          "quick_round_repair_simplex",
          std::chrono::duration<double, std::milli>(t_quick_repair1 - t_quick_repair0).count(),
          repair_res.result.stats.success && repair_res.result.x.size() >= n);
      if (repair_res.result.stats.success && repair_res.result.x.size() >= n) {
        xr = clamp_to_bounds(repair_res.result.x.head(n), root.lb, root.ub);
        // Re-snap integers.
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            xr[i] = std::round(xr[i]);
            xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
          }
        }
      }
      }  // else simplex path
      }  // repair_likely
    }

    if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub, 1e-6)) {
      const double obj = objective_value(base_lp.c, xr, base_lp.sense);
      const bool improved = adopt_root_incumbent("quick_round_direct", xr, obj);
      note_root_subsolve("quick_round_direct", 0.0, true, improved);
      if (opt.verbose) {
        MIPSOLVERS_LOG_DEBUG("[B&C] UC-aware rounding incumbent, obj={:.4f}", obj);
      }
    }
  }
  t_pp_d2 = std::chrono::steady_clock::now();

  // Line-search central rounding.
  // HiGHS centralRounding calls linesearchRounding(), whose tryRoundedPoint()
  // fixes rounded integer columns, propagates the local domain, and solves an
  // LP with those integers fixed to repair continuous columns.  The important
  // conformance point is that this is a root incumbent source, not merely a
  // direct feasibility check of the rounded vector.
  //
  // When an analytic centre is available, it is used as x_anchor.
  // Otherwise a plain rounded copy of x_relax is used (effectively nearest-
  // integer rounding from two ends), which still provides diversity.
  if (opt.use_linesearch_rounding && !root_gap_closed() &&
      !skip_pump_large_uc_like) {
    // HiGHS HighsPrimalHeuristics::centralRounding uses firstlpsol as point1;
    // degeneracy-sensitive derivation and measurement in
    // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
    // The post-cut root point may have the same objective but different
    // coordinates and therefore a different rounding-breakpoint sequence.
    const Eigen::VectorXd& x_line_start =
        first_root_lp_x.size() == n ? first_root_lp_x : root.x_relax;
    const Eigen::VectorXd* x_anchor_ptr = nullptr;
    Eigen::VectorXd x_rounded_anchor;
    if (analytic_centre.has_value() && analytic_centre->size() == n) {
      x_anchor_ptr = &(*analytic_centre);
    } else {
      // Fallback: anchor = nearest-integer rounding of x_relax.
      x_rounded_anchor = root.x_relax;
      for (int i = 0; i < n; ++i) {
        if (is_integer_type(base_lp.vars[i])) {
          x_rounded_anchor[i] = std::round(x_line_start[i]);
          x_rounded_anchor[i] =
              std::min(root.ub[i], std::max(root.lb[i], x_rounded_anchor[i]));
        }
      }
      x_anchor_ptr = &x_rounded_anchor;
    }

    double alpha = 0.0;
    int tries = 0;
    constexpr int kMaxLineSearchTries = 64;
    while (alpha < 1.0 - 1e-12 && tries < kMaxLineSearchTries) {
      if (root_gap_closed()) break;
      ++tries;
      double next_alpha = 1.0;
      bool reached_anchor_rounding = true;
      Eigen::VectorXd xr = x_line_start;
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        double rounded;
        if (opt.use_lock_count_rounding && i < static_cast<int>(uplocks.size())) {
          if (uplocks[i] == 0) {
            rounded = std::ceil(
                std::max(x_line_start[i], (*x_anchor_ptr)[i]) - opt.int_tol);
          } else if (downlocks[i] == 0) {
            rounded = std::floor(
                std::min(x_line_start[i], (*x_anchor_ptr)[i]) + opt.int_tol);
          } else {
              const double convex =
                (1.0 - alpha) * x_line_start[i] + alpha * (*x_anchor_ptr)[i];
            const double anchor_round = std::floor((*x_anchor_ptr)[i] + 0.5);
            rounded = std::floor(convex + 0.5);
            if (std::abs(rounded - anchor_round) > opt.int_tol) {
              reached_anchor_rounding = false;
              const double denom = std::abs((*x_anchor_ptr)[i] - x_line_start[i]);
              if (denom > 1e-12) {
                const double cand_alpha =
                    (rounded + 0.5 + opt.int_tol - x_line_start[i]) / denom;
                if (cand_alpha > alpha + 1e-2 && cand_alpha < next_alpha) {
                  next_alpha = cand_alpha;
                }
              }
            }
          }
        } else {
          const double convex =
              (1.0 - alpha) * x_line_start[i] + alpha * (*x_anchor_ptr)[i];
          const double anchor_round = std::floor((*x_anchor_ptr)[i] + 0.5);
          rounded = std::floor(convex + 0.5);
          if (std::abs(rounded - anchor_round) > opt.int_tol) {
            reached_anchor_rounding = false;
            const double denom = std::abs((*x_anchor_ptr)[i] - x_line_start[i]);
            if (denom > 1e-12) {
              const double cand_alpha =
                  (rounded + 0.5 + opt.int_tol - x_line_start[i]) / denom;
              if (cand_alpha > alpha + 1e-2 && cand_alpha < next_alpha) {
                next_alpha = cand_alpha;
              }
            }
          }
        }
        xr[i] = std::min(root.ub[i], std::max(root.lb[i], rounded));
      }
      if (bc_env_flag_enabled("MIPSOLVERS_LINESEARCH_COORD_TRACE")) {
        fmt::print(stderr, "[B&C-LINESEARCH-COORD] alpha={:.17g} bits=", alpha);
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          fmt::print(stderr, "{}", static_cast<long long>(std::llround(xr[i])));
        }
        fmt::print(stderr, "\n");
      }
      if (try_fixed_integer_repair("linesearch_round", xr)) {
        if (opt.verbose) {
          fmt::print(stderr, "[B&C-LINESEARCH] alpha={:.2f} obj={:.4f}\n",
                     alpha, incumbent_obj);
        }
        break;
      }
      if (reached_anchor_rounding) break;
      alpha = next_alpha;
    }
  }
  t_pp_d3 = std::chrono::steady_clock::now();

	    const bool skip_large_generic_pump =
	        large_generic_root && !has_incumbent &&
	        !opt.auto_highs_root_pipeline;
    if (skip_large_generic_pump && opt.verbose) {
      fprintf(stderr,
              "[B&C] skip root feasibility pump on large generic root "
              "(m=%d): yield budget to tree search\n",
              presolved_m);
    }

    const bool skip_large_near_integral_pump = large_near_integral_root;
    if (opt.use_feasibility_pump && !root_gap_closed() &&
      !skip_large_near_integral_pump && !skip_pump_large_uc_like &&
      !skip_large_generic_pump) {
    // Simple rounding first (with UC-awareness if available).
    Eigen::VectorXd xr = root.x_relax;
    if (reduced_uc_hint.has_value()) {
      uc_round_commitment(xr, *reduced_uc_hint, reduced_uc_hint->ng);
    }
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(base_lp.vars[i])) continue;
      xr[i] = std::round(xr[i]);
      xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
    }
    if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub, 1e-6)) {
      const double obj = objective_value(base_lp.c, xr, base_lp.sense);
      const bool improved = adopt_root_incumbent("pump_direct_round", xr, obj);
      note_root_subsolve("pump_direct_round", 0.0, true, improved);
    }

    // Enhanced feasibility pump: iterate LP solve + rounding.
    // Fix integer variables to rounded values, re-solve LP for continuous.
    // Skip the pump on large problems because each pump LP is expensive.
    // (~400ms for m>10k) and dive heuristic is more effective at finding
    // incumbents via incremental bound fixing with warm-started simplex.
    const int pump_row_limit = std::min(opt.xlarge_ipm_only_threshold, 10000);
    if (!has_incumbent && presolved_m <= pump_row_limit) {
      Eigen::VectorXd x_fix = root.x_relax;
      for (int pump_iter = 0; pump_iter < opt.feasibility_pump_iters && !has_incumbent; ++pump_iter) {
        // Round integer variables (with UC-aware rounding).
        Eigen::VectorXd x_rounded = x_fix;
        if (reduced_uc_hint.has_value()) {
          uc_round_commitment(x_rounded, *reduced_uc_hint, reduced_uc_hint->ng);
        }
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          x_rounded[i] = std::round(x_fix[i]);
          x_rounded[i] = std::min(root.ub[i], std::max(root.lb[i], x_rounded[i]));
        }

        // Check feasibility of rounded solution.
        if (satisfies_with_bounds(base_lp, x_rounded, root.lb, root.ub, 1e-6)) {
          const double obj = objective_value(base_lp.c, x_rounded, base_lp.sense);
          const bool improved = adopt_root_incumbent("pump_fixed_round", x_rounded, obj);
          note_root_subsolve("pump_fixed_round", 0.0, true, improved);
          break;
        }

        // Build LP with integer variables fixed to rounded values.
        // Solve LP for continuous variables only.
        // For larger problems, use IPM which handles lb==ub fixed variables.
        // For smaller problems, use simplex warm-start from root basis.
        Eigen::VectorXd pump_lb = root.lb, pump_ub = root.ub;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          pump_lb[i] = x_rounded[i];
          pump_ub[i] = x_rounded[i];
        }
        if (!bounds_consistent(pump_lb, pump_ub)) break;

        SolveResult pump_result;
        auto t_pump_lp0 = std::chrono::steady_clock::now();
        if (presolved_m > 5000) {
          // Use IPM for pump LP on medium-large problems.
          auto* pump_ipm_ptr = ensure_root_heur_ipm();
          if (!pump_ipm_ptr) break;
          pump_result = pump_ipm_ptr->solve_node_lp(base_lp, pump_lb, pump_ub, x_fix);
        } else {
          // Simplex warm-start from the cut-free root basis (HiGHS' first
          // root basis pattern). PersistentLPState exposes the basis that
          // is dimensionally compatible with `base_sf` so we never fall
          // back to IPM cold-start due to a row-count mismatch.
          StandardFormLP pump_sf = ensure_base_sf();
          update_standard_form_bounds(pump_sf, base_lp, pump_lb, pump_ub);
          SimplexOptions pump_opt;
          pump_opt.lp_kernel_backend = opt.lp_kernel_backend;
          pump_opt.max_iter = std::max(opt.max_lp_iter * 5, 1000);
          pump_opt.feasibility_tol = 1e-7;
          pump_opt.optimality_tol = 1e-7;
          pump_opt.verbose = false;
          auto pump_basis_hint = persistent_lp_state.basis_hint_for_base();
          auto pump_res = solve_lp_from_sf(pump_sf, pump_opt,
                                           pump_basis_hint.get());
          pump_result = std::move(pump_res.result);
        }
        auto t_pump_lp1 = std::chrono::steady_clock::now();
        if (opt.verbose) {
          fmt::print(stderr, "[PUMP] iter={} lp={:.1f}ms ok={}\n",
                     pump_iter,
                     std::chrono::duration<double, std::milli>(t_pump_lp1 - t_pump_lp0).count(),
                     pump_result.stats.success ? 1 : 0);
        }
        out.bc_stats.lp_solves += 1;
        note_root_subsolve(
            presolved_m > 5000 ? "pump_fixed_ipm" : "pump_fixed_simplex",
            std::chrono::duration<double, std::milli>(t_pump_lp1 - t_pump_lp0).count(),
            pump_result.stats.success && pump_result.x.size() == n);

        if (!pump_result.stats.success || pump_result.x.size() != n) {
          break;  // LP infeasible with fixed integers — pump can't help.
        }

        // Update x_fix with the LP solution.
        x_fix = clamp_to_bounds(pump_result.x, root.lb, root.ub);
      }
    }
    t_pp_d4 = std::chrono::steady_clock::now();

    // Objective-based feasibility pump for large problems (Fischetti-Glover style).
    // Instead of fixing integer bounds (which makes IPM fail with many lb==ub),
    // we CHANGE THE OBJECTIVE to minimize L1 distance to the rounded point while
    // keeping all original constraints and bounds intact.
    // Each iteration: round → check feasibility → build pump objective → solve LP.
    // Covers the range m > pump_row_limit where the standard pump doesn't run.
    //
    // Backend: native primal simplex with warm-start basis reuse. Costs are
    // swapped via update_standard_form_cost() — A, b, bounds, row/col scales
    // and the persisted SparseBasis LU stay valid. After each cost change the
    // basis is primal feasible but dual-infeasible, so primal simplex
    // re-optimizes in O(few hundred pivots) instead of a full refactor.
    const double t_elapsed_pre_objpump =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const bool skip_objpump_by_time =
        (t_elapsed_pre_objpump > 0.20 * opt.time_limit_sec);
    if (opt.use_feasibility_pump && !has_incumbent &&
        presolved_m > pump_row_limit && !skip_objpump_by_time &&
        !skip_pump_large_uc_like) {
      // Build (or reuse) the shared StandardFormLP for the pump. We snapshot
      // it into a dedicated instance because we are going to mutate c_max.
      StandardFormLP pump_sf = ensure_base_sf();
      update_standard_form_bounds(pump_sf, base_lp, root.lb, root.ub);

      // Start warm basis = cut-free firstrootbasis (HiGHS analog). Deep
      // copy so we own it across iterations and can invalidate cached
      // reduced_costs after each cost swap. PersistentLPState guarantees
      // dimension match with `pump_sf` so this branch is taken even when
      // many cuts were committed earlier in the cut loop.
      std::shared_ptr<SimplexBasis> pump_basis;
      auto base_hint_for_pump = persistent_lp_state.basis_hint_for_base();
      const int pump_sf_rows = static_cast<int>(pump_sf.A.rows());
      const int pump_sf_cols = static_cast<int>(pump_sf.A.cols());
      const int hint_rows = base_hint_for_pump ? base_hint_for_pump->rows : -1;
      const int hint_cols = base_hint_for_pump ? base_hint_for_pump->cols : -1;
      if (base_hint_for_pump &&
          hint_rows == pump_sf_rows &&
          hint_cols == pump_sf_cols) {
        pump_basis = std::make_shared<SimplexBasis>(*base_hint_for_pump);
      }
      if (opt.verbose || bc_env_options().value("MIPSOLVERS_PUMP_DIAG")) {
        fmt::print(stderr,
                   "[OBJ-PUMP-DIAG] pump_sf={}x{} base_hint={}x{} (cut={}x{}) "
                   "warm={}\n",
                   pump_sf_rows, pump_sf_cols, hint_rows, hint_cols,
                   persistent_lp_state.has_cut_basis() ? persistent_lp_state.cut_basis()->rows : -1,
                   persistent_lp_state.has_cut_basis() ? persistent_lp_state.cut_basis()->cols : -1,
                   pump_basis ? 1 : 0);
      }

      // IPM fallback for when simplex warm-start fails (rare) or when we
      // don't have a valid starting basis.
      std::unique_ptr<NativeIPMLPAdapter> pump_ipm_fb;
      auto ensure_pump_ipm = [&]() -> NativeIPMLPAdapter* {
        if (!pump_ipm_fb) {
          IPMLPOptions io;
          io.max_iter = (presolved_m > 25000) ? 200 : 100;
          io.tol_primal = 1e-7;
          io.tol_dual = 1e-7;
          io.tol_gap = 1e-7;
          io.verbose = false;
          pump_ipm_fb = std::make_unique<NativeIPMLPAdapter>(io);
          pump_ipm_fb->prepare_for_node_solves(base_lp);
        }
        return pump_ipm_fb.get();
      };

      // Sense multiplier: pump cost must be in LP convention.
      const double sgn = (base_lp.sense == Sense::Maximize) ? -1.0 : 1.0;
      Eigen::VectorXd x_pump = root.x_relax;
      // Achterberg-Berthold Objective FP: keep a rolling history of the last
      // HIST rounded points for cycle detection (vanilla 1-step check misses
      // longer cycles). Use Zobrist-style 64-bit hash for fast compare.
      constexpr int HIST = 3;
      std::array<std::uint64_t, HIST> hist_hash{};
      int hist_idx = 0;

      // Pre-build integer index list to avoid O(n) scans each iter.
      std::vector<int> int_idx;
      int_idx.reserve(n);
      for (int i = 0; i < n; ++i)
        if (is_integer_type(base_lp.vars[i])) int_idx.push_back(i);

      // Zobrist hash table: random u64 per integer var, xor'd based on rounded value.
      std::mt19937_64 rng(0x9E3779B97F4A7C15ULL);
      std::vector<std::uint64_t> zobrist(n, 0);
      for (int i : int_idx) zobrist[i] = rng();

      // Achterberg alpha: geometric decay from 1.0 → push toward original
      // objective as iterations progress so we drift toward good basins.
      double alpha = 1.0;
      const double alpha_decay = 0.9;

      // Wall-time budget: cap pump phase to avoid starving B&C on large m.
      // On large UC-like MILPs the pump rarely finds an incumbent; cap
      // more tightly so the parallel tree gets time to dive instead.
      auto t_pump_phase0 = std::chrono::steady_clock::now();
      const bool pump_large_uc_like = domain_heuristics &&
          ((presolved_m > opt.large_uc_row_threshold) ||
           (static_cast<int>(int_idx.size()) > opt.large_uc_binary_threshold));
      const double pump_wall_budget_sec =
          pump_large_uc_like
              ? std::max(0.5, std::min(opt.pump_wall_budget_cap_large,
                                       opt.pump_wall_budget_ratio_large * opt.time_limit_sec))
              : std::max(1.0, std::min(opt.pump_wall_budget_cap_default,
                                       opt.pump_wall_budget_ratio_default * opt.time_limit_sec));

      // Track the best iterate seen (min n_frac) for last-chance fixing.
      int best_n_frac = std::numeric_limits<int>::max();
      Eigen::VectorXd best_x_pump = x_pump;
      Eigen::VectorXd best_x_rounded;

      const int max_pump_iters = 30;
      int stage3_restarts = 0;
      const int max_stage3 = 3;
      for (int pump_iter = 0; pump_iter < max_pump_iters && !has_incumbent; ++pump_iter) {
        // Budget check.
        const double t_pump_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_pump_phase0).count();
        if (t_pump_elapsed > pump_wall_budget_sec) {
          if (opt.verbose)
            fmt::print(stderr, "[OBJ-PUMP] budget exhausted at iter={} ({:.1f}s)\n",
                       pump_iter, t_pump_elapsed);
          break;
        }
        // Round integer variables (general-purpose, no domain knowledge).
        Eigen::VectorXd x_rounded = x_pump;
        std::uint64_t cur_hash = 0;
        for (int i : int_idx) {
          double r = std::round(x_pump[i]);
          r = std::min(root.ub[i], std::max(root.lb[i], r));
          x_rounded[i] = r;
          if (r > 0.5) cur_hash ^= zobrist[i];
        }

        // Check if rounded solution is feasible.
        if (satisfies_with_bounds(base_lp, x_rounded, root.lb, root.ub, 1e-6)) {
          const double obj = objective_value(base_lp.c, x_rounded, base_lp.sense);
          const bool improved = adopt_root_incumbent("obj_pump_round", x_rounded, obj);
          note_root_subsolve("obj_pump_round", 0.0, true, improved);
          if (opt.verbose)
            fmt::print(stderr, "[OBJ-PUMP] iter={} FEASIBLE obj={:.1f}\n", pump_iter, obj);
          break;
        }

        // Cycle detection against rolling history (HIST steps back).
        bool cycling = false;
        for (int h = 0; h < HIST; ++h) {
          if (hist_hash[h] != 0 && hist_hash[h] == cur_hash) {
            cycling = true;
            break;
          }
        }
        if (cycling) {
          // Fischetti-Salvagnin randomized perturbation: flip each integer
          // independently with probability proportional to its fractional
          // part. This escapes cycles better than deterministic top-K.
          std::uniform_real_distribution<double> unif(0.0, 1.0);
          int n_flipped = 0;
          for (int i : int_idx) {
            const double f = x_pump[i] - std::floor(x_pump[i]);
            const double frac_dist = std::min(f, 1.0 - f);
            // Flip probability: 0 if near-integral, up to 0.5 at frac=0.5.
            const double p_flip = 0.5 * frac_dist + 0.03;  // 3% baseline jitter
            if (unif(rng) < p_flip) {
              x_rounded[i] = (x_rounded[i] < 0.5) ? 1.0 : 0.0;
              x_rounded[i] = std::min(root.ub[i], std::max(root.lb[i], x_rounded[i]));
              ++n_flipped;
            }
          }
          if (n_flipped == 0 && !int_idx.empty()) {
            // Guarantee ≥1 flip: flip the single most-fractional.
            int best_i = int_idx[0];
            double best_f = -1.0;
            for (int i : int_idx) {
              const double f = x_pump[i] - std::floor(x_pump[i]);
              const double fd = std::min(f, 1.0 - f);
              if (fd > best_f) { best_f = fd; best_i = i; }
            }
            x_rounded[best_i] = (x_rounded[best_i] < 0.5) ? 1.0 : 0.0;
            n_flipped = 1;
          }
          // Rehash.
          cur_hash = 0;
          for (int i : int_idx)
            if (x_rounded[i] > 0.5) cur_hash ^= zobrist[i];
          if (opt.verbose)
            fmt::print(stderr, "[OBJ-PUMP] iter={} CYCLE, flipped {} vars (rand)\n",
                       pump_iter, n_flipped);
          // Stage-3: if we've cycled several times, reset alpha to 1.0 to
          // re-emphasize feasibility (and later we'd try sub-MIP).
          if (++stage3_restarts >= max_stage3) {
            if (opt.verbose)
              fmt::print(stderr, "[OBJ-PUMP] max stage-3 restarts reached; aborting pump\n");
            break;
          }
          alpha = 1.0;  // reset stage
        }
        hist_hash[hist_idx] = cur_hash;
        hist_idx = (hist_idx + 1) % HIST;

        // Achterberg OFP pump objective:
        //   pump_c = alpha * (L1-distance gradient) + (1 - alpha) * (scaled c)
        // Scale original cost to have same magnitude as distance gradient
        // (which is ±1 per binary) to avoid domination.
        double c_norm = 0.0;
        for (int i = 0; i < n; ++i) c_norm += base_lp.c[i] * base_lp.c[i];
        c_norm = std::sqrt(c_norm);
        const double c_scale =
            (c_norm > 1e-12) ? std::sqrt(static_cast<double>(std::max(1, (int)int_idx.size()))) / c_norm
                             : 0.0;
        Eigen::VectorXd pump_c(n);
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            const double dist_cost = sgn * (1.0 - 2.0 * x_rounded[i]);
            pump_c[i] = alpha * dist_cost + (1.0 - alpha) * c_scale * base_lp.c[i];
          } else {
            pump_c[i] = (1.0 - alpha) * c_scale * base_lp.c[i];
          }
        }
        // Solve the pump LP with primal simplex, reusing the warm basis.
        // Cost-only change: basis is primal feasible and dual-infeasible.
        // Pump cost is in LP user convention — pass
        // via update_standard_form_cost which applies sense + Ruiz scaling.
        update_standard_form_cost(pump_sf, base_lp.sense, pump_c);

        // Invalidate cached reduced costs on the basis (they depend on c_max).
        SimplexBasis* hint_raw = nullptr;
        if (pump_basis) {
          pump_basis->cached_reduced_costs.reset();
          hint_raw = pump_basis.get();
        }

        SimplexOptions pump_sopt;
        pump_sopt.lp_kernel_backend = opt.lp_kernel_backend;
        // Pump simplex: tight iter cap. After a cost swap the basis is
        // primal feasible / dual infeasible, and primal simplex must pivot
        // away every wrong-sign reduced cost. On large m this can be tens
        // of thousands of pivots — we'd rather fall back to IPM quickly.
        // Scale: ~2x number of integer variables as a rough estimate of
        // how many pivots are typically needed.
        const int pump_pivot_cap =
            std::max(500, std::min(5000, 2 * static_cast<int>(int_idx.size())));
        pump_sopt.max_iter = pump_pivot_cap;
        pump_sopt.feasibility_tol = 1e-7;
        pump_sopt.optimality_tol = 1e-7;
        pump_sopt.allow_cold_start = false;  // don't let simplex burn time on cold start
        pump_sopt.verbose = false;

        auto t_obj_pump0 = std::chrono::steady_clock::now();
        SimplexResult spx_res;
        bool tried_simplex = false;
        // Attempt simplex only with a reusable basis and below the row-count
        // policy boundary; otherwise use the separately budgeted IPM path.
        const bool simplex_pump_enabled =
            (hint_raw != nullptr) && (pump_sf.A.rows() < 25000);
        if (simplex_pump_enabled) {
          update_standard_form_cost(pump_sf, base_lp.sense, pump_c);
          hint_raw->cached_reduced_costs.reset();
          spx_res = solve_lp_from_sf(pump_sf, pump_sopt, hint_raw);
          tried_simplex = true;
        }
        auto t_obj_pump1 = std::chrono::steady_clock::now();
        out.bc_stats.lp_solves += (tried_simplex ? 1 : 0);

        // Convert to SolveResult-like fields used below.
        struct { SolveStats stats; Eigen::VectorXd x; } pump_res;
        if (tried_simplex) {
          pump_res.stats = spx_res.result.stats;
          pump_res.x = spx_res.result.x;
        } else {
          pump_res.stats.success = false;
          pump_res.stats.status = "no-warm-basis";
        }

        // Fallback to IPM if simplex failed / hit iter limit / had no warm basis.
        if (!pump_res.stats.success || pump_res.x.size() != n) {
          NativeIPMLPAdapter* ipm_fb = ensure_pump_ipm();
          if (ipm_fb) {
            ipm_fb->update_cached_cost(pump_c);
            auto ipm_res = ipm_fb->solve_cached_node_lp(root.lb, root.ub, x_pump);
            out.bc_stats.lp_solves += 1;
            pump_res.stats = ipm_res.stats;
            pump_res.x = ipm_res.x;
            // IPM solved from scratch — next iter will cold-start simplex again,
            // but the basis_hint we had may still help in later iters after
            // reduced costs settle. Keep pump_basis as is.
          }
        } else {
          // Simplex succeeded → update the warm basis for the next iteration.
          pump_basis = std::make_shared<SimplexBasis>(std::move(spx_res.basis));
        }

        note_root_subsolve(
          "obj_pump_spx",
          std::chrono::duration<double, std::milli>(t_obj_pump1 - t_obj_pump0).count(),
          pump_res.stats.success && pump_res.x.size() == n);

        if (!pump_res.stats.success || pump_res.x.size() != n) {
          if (opt.verbose)
            fmt::print(stderr, "[OBJ-PUMP] iter={} LP failed: {}\n",
                       pump_iter, pump_res.stats.status);
          break;
        }

        // Count fractional integer variables.
        int n_frac = 0;
        for (int i : int_idx) {
          const double f = pump_res.x[i] - std::floor(pump_res.x[i]);
          if (std::min(f, 1.0 - f) > 1e-6) ++n_frac;
        }

        if (opt.verbose)
          fmt::print(stderr, "[OBJ-PUMP] iter={} alpha={:.3f} lp={:.1f}ms n_frac={}\n",
                     pump_iter, alpha, pump_res.stats.runtime_sec * 1000.0, n_frac);

        // Track best iterate for last-chance fixing after the pump loop.
        if (n_frac < best_n_frac) {
          best_n_frac = n_frac;
          best_x_pump = pump_res.x;
          best_x_rounded = x_rounded;
        }

        x_pump = clamp_to_bounds(pump_res.x, root.lb, root.ub);
        alpha *= alpha_decay;  // geometric decay toward original objective
        if (alpha < 0.3) alpha = 1.0;  // restart: re-emphasize feasibility
      }

      // Last-chance feasibility: take the best (min-fractional) pump iterate,
      // round all integers, fix them, and solve the resulting LP. If feasible,
      // we have an incumbent even if the pump itself didn't converge.
      // Skip on the large UC-like policy branch, where fixing every integer at
      // once conflicts with the intended bounded heuristic budget.
      if (!has_incumbent && !pump_large_uc_like &&
          best_n_frac < std::numeric_limits<int>::max() &&
          best_x_rounded.size() == n) {
        Eigen::VectorXd lastch_lb = root.lb, lastch_ub = root.ub;
        for (int i : int_idx) {
          const double rv = std::min(root.ub[i], std::max(root.lb[i], best_x_rounded[i]));
          lastch_lb[i] = rv;
          lastch_ub[i] = rv;
        }
        auto t_lc0 = std::chrono::steady_clock::now();
        NativeIPMLPAdapter* ipm_fb = ensure_pump_ipm();
        bool lc_ok = false;
        Eigen::VectorXd lc_x;
        if (ipm_fb) {
          auto lc_res = ipm_fb->solve_node_lp(base_lp, lastch_lb, lastch_ub, best_x_pump);
          out.bc_stats.lp_solves += 1;
          lc_ok = lc_res.stats.success && lc_res.x.size() == n;
          if (lc_ok) lc_x = lc_res.x;
        }
        auto t_lc1 = std::chrono::steady_clock::now();
        const double lc_ms = std::chrono::duration<double, std::milli>(t_lc1 - t_lc0).count();
        if (lc_ok) {
          Eigen::VectorXd xlc = clamp_to_bounds(lc_x, root.lb, root.ub);
          for (int i : int_idx) {
            xlc[i] = std::min(root.ub[i], std::max(root.lb[i], std::round(xlc[i])));
          }
          if (satisfies_with_bounds(base_lp, xlc, root.lb, root.ub, 1e-6)) {
            const double obj = objective_value(base_lp.c, xlc, base_lp.sense);
            const bool improved = adopt_root_incumbent("obj_pump_lastchance", xlc, obj);
            note_root_subsolve("obj_pump_lastchance", lc_ms, true, improved);
            if (opt.verbose)
              fmt::print(stderr, "[OBJ-PUMP] last-chance FEASIBLE obj={:.1f} ({:.0f}ms)\n",
                         obj, lc_ms);
          } else {
            note_root_subsolve("obj_pump_lastchance", lc_ms, false);
          }
        } else {
          note_root_subsolve("obj_pump_lastchance", lc_ms, false);
        }
      }

      // ────────────────────────────────────────────────────────────────
      // Stage-3 sub-MIP fallback: the pump had a best-fractional iterate
      // but none of the direct-round / last-chance attempts produced a
      // feasible integer incumbent. Fix the integers that the pump kept
      // consistent (non-fractional at best_x_pump) and recurse on the
      // remaining fractional subset as a small MILP. This works in the
      // presolved variable space — the sub-MIP shares base_lp's rows and
      // root.lb/root.ub — so any incumbent returned is directly usable as
      // a root-level incumbent for the outer solve. Only invoke when the
      // fractional subset is small enough to be tractable in the budget.
      // ────────────────────────────────────────────────────────────────
      if (!has_incumbent && !opt._is_stage3_submip && !pump_large_uc_like &&
          best_x_pump.size() == n && best_n_frac > 0 &&
          best_n_frac <= 150) {
        const double remaining_s =
            std::max(0.0, opt.time_limit_sec -
                          std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t0).count());
        // Large UC-like models: this stage3 recurse almost always fails
        // (the sub-MIP has the same structural coupling), so cap tightly
        // and give the parallel tree the remaining budget instead.
        const bool s3_large_uc_like = domain_heuristics &&
            ((presolved_m > opt.large_uc_row_threshold) ||
             (static_cast<int>(int_idx.size()) > opt.large_uc_binary_threshold));
        const double submip_budget = large_generic_root
            ? std::min(1.0, 0.05 * remaining_s)
            : s3_large_uc_like
            ? std::min(opt.pump_stage3_budget_cap_large,
                       opt.pump_stage3_budget_ratio_large * remaining_s)
            : std::min(15.0, 0.25 * remaining_s);
        if (submip_budget >= (large_generic_root ? 0.25 : 2.0)) {
          MIPModel sub_prob;
          sub_prob.linear_part = base_lp;  // copies A/Aeq/b/beq/c/vars
          // Apply current node bounds and fix confident integers.
          for (int i = 0; i < n; ++i) {
            sub_prob.linear_part.vars[i].lb = root.lb[i];
            sub_prob.linear_part.vars[i].ub = root.ub[i];
          }
          std::vector<int> sub_int_idx, sub_bin_idx;
          int n_fixed = 0;
          for (int i : int_idx) {
            const double f = best_x_pump[i] - std::floor(best_x_pump[i]);
            const bool frac = (std::min(f, 1.0 - f) > 1e-6);
            if (frac) {
              sub_int_idx.push_back(i);
              if (base_lp.vars[i].type == VarType::Binary) sub_bin_idx.push_back(i);
            } else {
              const double rv = std::min(root.ub[i],
                                   std::max(root.lb[i], std::round(best_x_pump[i])));
              sub_prob.linear_part.vars[i].lb = rv;
              sub_prob.linear_part.vars[i].ub = rv;
              ++n_fixed;
            }
          }
          sub_prob.integer_idx = sub_int_idx;
          sub_prob.binary_idx = sub_bin_idx;

          BCOptions sub_opt;
          sub_opt._is_stage3_submip = true;
          sub_opt.time_limit_sec = submip_budget;
          sub_opt.max_nodes = large_generic_root ? 500 : 5000;
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
          sub_opt.enable_lns = false;
          sub_opt.use_papilo_presolve = false;
          sub_opt.enable_incumbent_local_branching = false;
          sub_opt.enable_root_low_fractionality_rens = false;
          sub_opt.enable_objective_cutoff_conflict_cuts = false;
          sub_opt.enable_objective_cutoff_weighted_event_cuts = false;
          sub_opt.enable_objective_cutoff_domain_fixing = false;
          sub_opt.enable_dynamic_implied_bound_probing = false;
          sub_opt.enable_reduced_cost_proof_cut_resolve = false;
          sub_opt.num_threads = 1;
          sub_opt.verbose = false;

          auto t_s30 = std::chrono::steady_clock::now();
          BCResult sub_res = branch_and_cut_lp(sub_prob, sub_opt);
          auto t_s31 = std::chrono::steady_clock::now();
          const double s3_ms =
              std::chrono::duration<double, std::milli>(t_s31 - t_s30).count();

          bool s3_ok = false;
          if (sub_res.x.size() == n && std::isfinite(sub_res.stats.objective)) {
            Eigen::VectorXd xs = clamp_to_bounds(sub_res.x, root.lb, root.ub);
            for (int i : int_idx) {
              xs[i] = std::min(root.ub[i], std::max(root.lb[i], std::round(xs[i])));
            }
            if (satisfies_with_bounds(base_lp, xs, root.lb, root.ub, 1e-6)) {
              const double obj = objective_value(base_lp.c, xs, base_lp.sense);
              const bool improved = adopt_root_incumbent("obj_pump_stage3_submip", xs, obj);
              note_root_subsolve("obj_pump_stage3_submip", s3_ms, true,
                                 improved, sub_res.bc_stats.lp_solves);
              s3_ok = true;
              if (opt.verbose)
                fmt::print(stderr,
                  "[OBJ-PUMP] stage3 sub-MIP FEASIBLE n_free={} n_fixed={} obj={:.1f} ({:.0f}ms)\n",
                  (int)sub_int_idx.size(), n_fixed, obj, s3_ms);
            }
          }
          if (!s3_ok) {
            note_root_subsolve("obj_pump_stage3_submip", s3_ms, false,
                               false, sub_res.bc_stats.lp_solves);
            if (opt.verbose)
              fmt::print(stderr,
                "[OBJ-PUMP] stage3 sub-MIP n_free={} n_fixed={} budget={:.1f}s -> infeas/none ({:.0f}ms)\n",
                (int)sub_int_idx.size(), n_fixed, submip_budget, s3_ms);
          }
        }
      }

      // Restore original cost for subsequent uses (though pump adapter is local).
      // No action needed — pump_ipm goes out of scope.
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // LP Diving Heuristic: greedily fix fractional binaries, re-solve LP.
  [[maybe_unused]] auto t_pump_done = std::chrono::steady_clock::now();
  if (opt.verbose) {
    fmt::print(stderr, "[B&C ROOT] feas_pump={:.1f}ms has_incumbent={}\n",
            std::chrono::duration<double, std::milli>(t_pump_done - t_cuts_done).count(),
            has_incumbent ? 1 : 0);
  }
  if (bc_env_options().value("MIPSOLVERS_PUMP_DIAG") != nullptr) {
    auto ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    fmt::print(stderr,
      "[PUMP-WINDOW] randR={:.0f} fwdMap={:.0f} feasJump={:.0f} "
      "quickRnd={:.0f} lineSrch={:.0f} fixedPump={:.0f} objPump={:.0f} "
      "total={:.0f}ms skip_pump_uc={} large_uc_like={} is_xlarge={} large_gen={}\n",
      ms(t_pp_a, t_pp_b), ms(t_pp_b, t_pp_c), ms(t_pp_c, t_pp_d),
      ms(t_pp_d, t_pp_d2), ms(t_pp_d2, t_pp_d3), ms(t_pp_d3, t_pp_d4),
      ms(t_pp_d4, t_pump_done),
      ms(t_cuts_done, t_pump_done),
      skip_pump_large_uc_like ? 1 : 0,
      large_uc_like_root ? 1 : 0,
      is_xlarge_root ? 1 : 0,
      large_generic_root ? 1 : 0);
  }

  // ════════════════════════════════════════════════════════════════════════
  // Concurrent tree launch (parallel mode): launch tree threads before
  // expensive root heuristics (prog-round, dive, LNS) so tree exploration
  // overlaps with heuristics.  The tree threads use SharedIncumbent for
  // pruning; any incumbent found by main thread heuristics is pushed to
  // SharedIncumbent so tree threads benefit immediately.
  //
  // Gate: only launch concurrent tree when we already have an incumbent
  // whose gap exceeds gap_tol.  Without an incumbent, tree exploration has
  // no bound for pruning and just adds CPU contention that slows heuristics.
  // Prog-round / dive / LNS will find an incumbent quickly, and the late-
  // launch parallel path (after root heuristics) handles tree if needed.
  // ════════════════════════════════════════════════════════════════════════
  const bool gap_needs_tree = has_incumbent && !root_gap_closed();
  if (!parallel_tree_allowed && par_mode &&
      opt.parallel_proof_on_incumbent_gap &&
      parallel_schedule_reason == "serial_low_fractionality_root" &&
      gap_needs_tree) {
    parallel_tree_allowed = true;
    parallel_schedule_reason = "parallel_proof_low_fractionality";
    if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C-PAR] enabling parallel proof on low-fractionality root "
                 "(frac_bin={}/{}, incumbent gap open)\n",
                 root_frac_bin_count, root_total_bin_count);
    }
  }
  const int parallel_launch_threads = std::max(1, resolve_num_threads(opt));
  const double parallel_launch_remaining_sec = bc_remaining_sec();
  const double parallel_launch_min_remaining_sec =
      opt.time_limit_sec > 0.0 && std::isfinite(opt.time_limit_sec)
          ? std::max(0.25, 0.25 * opt.time_limit_sec)
          : 0.25;
  const bool allow_concurrent_tree =
      parallel_tree_allowed && gap_needs_tree &&
      parallel_launch_threads >= 2 &&
      parallel_launch_remaining_sec >= parallel_launch_min_remaining_sec &&
      !bc_optional_root_budget_expired();
  if (parallel_tree_allowed && gap_needs_tree && !allow_concurrent_tree) {
    parallel_schedule_reason = "late_tree_stable_root";
    if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C-PAR] delaying parallel proof until root state is stable\n");
    }
  }
  if (allow_concurrent_tree) {
    const int num_threads = std::max(1, resolve_num_threads(opt));
    cts = std::make_unique<ConcurrentTreeState>(opt, t0, n);
    for (int col : declared_branchable_indices) cts->pseudocosts[col] = pc[col];

    // Seed shared cut pool with root cuts.
    if (out.bc_stats.cuts_added > 0) {
      const int orig_rows = static_cast<int>(pre_cut_root_lp.A.rows());
      const int total_rows = static_cast<int>(base_lp.A.rows());
      Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_cts = base_lp.A;
      for (int r = orig_rows; r < total_rows; ++r) {
        Eigen::SparseVector<double> row_sparse(n);
        int nnz = 0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_cts, r); it; ++it) {
          if (it.col() < n) ++nnz;
        }
        row_sparse.reserve(nnz);
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_cts, r); it; ++it) {
          if (it.col() < n) {
            row_sparse.insertBack(it.col()) = it.value();
          }
        }
        cts->shared_cp.seed(std::move(row_sparse), base_lp.b[r]);
      }
    }

    // Seed incumbent if available from quick rounding / pump.
    if (has_incumbent) {
      cts->shared_inc.try_update(incumbent_x, incumbent_obj);
      cts->shared_sp.seed(incumbent_x, incumbent_obj);
    }

	    // Push root node to shared queue.
    cts->shared_queue.configure_domain_signature(
        declared_branchable_cols, root.lb, root.ub, std::max(1e-9, opt.int_tol));
	    cts->shared_queue.push(root, has_incumbent);
    cts->optimality_limit.store(
        root_gap_certificate_ready && !opt.require_tree_exhaustion_certificate &&
                has_incumbent && std::isfinite(incumbent_obj) &&
                std::isfinite(opt.gap_tol) && opt.gap_tol >= 0.0
            ? incumbent_obj -
                  opt.gap_tol * std::max(1.0, std::abs(incumbent_obj))
            : kInf,
        std::memory_order_release);

    // Configure parallel simplex options.
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

    // Determine thread roles.
    cts->use_cut_worker = (opt.cuts != CutType::None && opt.max_cut_depth > 0);
    const int n_exp = cts->use_cut_worker ? num_threads - 1 : num_threads;
    cts->n_explorers = n_exp;
    out.bc_stats.parallel_effective_threads = num_threads;
    out.bc_stats.parallel_explorer_threads = n_exp;
    out.bc_stats.parallel_tree_launched = true;
    out.bc_stats.parallel_schedule_reason = "concurrent_tree";
    const int par_base_m = static_cast<int>(base_lp.A.rows()) +
                           static_cast<int>(base_lp.Aeq.rows());
    const bool use_hybrid_ipm = (n_exp >= 2 && par_base_m > opt.hybrid_ipm_threshold);

    // Work-stealing pool. Seed deterministically so that repeated solves
    // inside the same process do not drift purely due to RNG state.
    if (opt.enable_work_stealing && n_exp >= 2) {
      cts->steal_pool = std::make_unique<WorkStealingPool>(
          n_exp, opt.work_steal_deque_size, opt.random_seed);
    }

    // Deterministic-parallel token (opt-in via BCOptions::deterministic_parallel).
    // Serialises queue pops and child pushes across workers so node
    // exploration order is a pure function of worker count + queue keys,
    // independent of OS scheduling jitter.  LP solves between pop and
    // push still run in parallel, so throughput impact is bounded.
    if (opt.deterministic_parallel && n_exp >= 2) {
      cts->det_token = std::make_unique<DeterministicTurnToken>(n_exp);
    }

    // Launch explorer threads.
    cts->explorers.reserve(n_exp);
    for (int tid = 0; tid < n_exp; ++tid) {
      WorkerRole role;
      if (use_hybrid_ipm && tid == 0) {
        role = WorkerRole::IPMDiver;
      } else if (tid == n_exp - 1 && n_exp >= 2) {
        role = WorkerRole::Prover;
      } else {
        role = WorkerRole::Diver;
      }
      cts->explorers.emplace_back(explorer_thread,
          solve_environment, tid, role,
          std::ref(cts->shared_queue),
          std::ref(cts->cut_req_queue),
          std::ref(cts->shared_cp),
          std::ref(cts->shared_conflicts),
          std::ref(cts->shared_sp),
          std::ref(cts->shared_inc),
	          std::cref(base_lp),
	          std::cref(pre_cut_root_lp),
	          std::cref(ensure_base_sf()),
	          std::cref(declared_branchable_cols),
	          std::cref(parallel_branch_priority),
	          std::cref(branch_original_cols), orig_n,
	          callbacks && callbacks->branching_prior
	              ? &callbacks->branching_prior
	              : nullptr,
	          std::cref(clique_table),
	          std::cref(implication_graph),
          std::cref(cts->par_simplex_opt),
          std::cref(opt),
          std::ref(cts->pseudocosts),
          std::ref(cts->pc_mtx),
          std::ref(cts->atomic_stats),
          std::ref(cts->should_stop),
          std::ref(cts->progress_event),
          std::ref(cts->active_explorers),
          std::ref(cts->active_bounds),
          std::ref(cts->fallback_logger),
          root.basis_hint,
          cts->steal_pool.get(),
          &cts->shared_impl_graph,
          cts->det_token.get(),
          &cts->optimality_limit);
    }

    // Launch cut worker.
    if (cts->use_cut_worker) {
      cts->cut_worker = std::thread(cut_worker_thread,
          solve_environment, std::ref(cts->cut_req_queue),
          std::ref(cts->shared_cp),
          std::ref(cts->shared_sp),
          std::ref(cts->shared_inc),
          std::cref(base_lp),
          std::cref(opt),
          std::ref(cts->atomic_stats),
          std::ref(cts->should_stop));
    }

    concurrent_tree_launched = true;
    if (opt.verbose) {
      fprintf(stderr, "[B&C] concurrent tree launched with %d threads before prog-round\n",
              num_threads);
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // Progressive Rounding Heuristic: Fix integer variables in batches,
  // starting from those closest to integer values, and re-solve LP after
  // each batch.
  //
  // Two-phase strategy:
  //   Phase 1 (IPM): use IPM with warm-start and try fix-all before batches.
  //   Phase 2 (Simplex fallback): If IPM fails, uses a simplex warm start.
  // ════════════════════════════════════════════════════════════════════════
	  // Always run prog-round when the incumbent gap is proof-relevant.
	  // On low-fractionality roots a very small incumbent improvement can close
	  // the whole MILP, so use the solver proof tolerance instead of the legacy
	  // 0.5% "large gap" trigger.
	  const double prog_round_gap_threshold =
	      large_near_integral_root ? opt.gap_tol : 0.005;
	  bool need_prog_round = opt.use_progressive_rounding && !has_incumbent;
	  if (has_incumbent && std::isfinite(root.bound) && std::abs(root.bound) > 1e-9) {
	    double gap = (incumbent_obj - root.bound) / std::abs(root.bound);
	    if (opt.use_progressive_rounding && gap > prog_round_gap_threshold)
	      need_prog_round = true;
	  }
  // Skip ALL remaining heuristics if gap is already within tolerance.
  if (root_gap_closed()) need_prog_round = false;
	  // Do not use a UC-shaped label to suppress the generic progressive
	  // rounding path.  HiGHS keeps root primal heuristics available when the
	  // incumbent/root-bound gap is still proof-relevant; the generic guards
	  // below (gap, root size, basis availability, and time budget) decide
	  // whether this path is worth spending.
  // SCUC-like policy: when an incumbent already exists and LNS is enabled,
  // reserve the remaining optional heuristic budget for the restricted
  // sub-MIP rather than running progressive rounding first.
  if (need_prog_round && is_scuc_like && has_incumbent && opt.enable_lns) {
    if (opt.verbose) {
      fprintf(stderr, "[B&C] skip prog-round (SCUC-like, has_incumbent, LNS enabled)\n");
    }
    need_prog_round = false;
  }
  if (need_prog_round && large_generic_root) {
    double gap = std::numeric_limits<double>::infinity();
    if (has_incumbent && std::isfinite(root.bound) && std::abs(root.bound) > 1e-9) {
      gap = (incumbent_obj - root.bound) / std::abs(root.bound);
    }
	    if ((!has_incumbent && !(requested_vendored_highs_lp &&
	                             opt.auto_highs_root_pipeline)) ||
	        (large_generic_low_frac_root && root_incumbent_good_enough()) ||
	        gap <= opt.gap_tol) {
      if (opt.verbose) {
        fprintf(stderr,
                "[PROG-ROUND] skip on large generic root "
                "(has_inc=%d gap=%.3e low_frac=%d m=%d): tree search has better ROI\n",
                has_incumbent ? 1 : 0, gap,
                large_generic_low_frac_root ? 1 : 0, presolved_m);
      }
      trace_root_primal_gate("prog_round", "skip",
                             "large_generic_or_good_incumbent=1");
      need_prog_round = false;
    } else {
      trace_root_primal_gate("prog_round", "run",
                             "weak_incumbent_large_generic=1");
    }
  }
	  if (need_prog_round && requested_vendored_highs_lp &&
	      opt.auto_highs_root_pipeline) {
	    trace_root_primal_gate("prog_round", "run",
	                           "vendored_highs_root_pipeline_primal_source=1");
	  }
  // For parallel mode, the tree threads will also search for incumbents.
  // Reduce prog-round budget since each LP solve delays tree launch.
      if (need_prog_round && presolved_m > 5000 && root.basis_hint) {
    // Wall-time guard: when the user-supplied time_limit_sec is small (e.g.
    // inside a recursive stage-3 sub-MIP) the simplex-based prog-round can
    // spend far more than the remaining budget on a few slow LP solves.
    // Gate the whole block by a fraction of time_limit_sec so we preserve
    // time for the B&C tree.
    const double t_elapsed_before_prog_spx =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const bool skip_prog_spx_by_time =
        (t_elapsed_before_prog_spx > 0.40 * opt.time_limit_sec);
    if (skip_prog_spx_by_time) {
      if (opt.verbose)
        fprintf(stderr, "[PROG-ROUND] skip simplex prog-round (elapsed=%.1fs > 0.4*limit=%.1fs)\n",
                t_elapsed_before_prog_spx, 0.40 * opt.time_limit_sec);
      need_prog_round = false;
    }
  }
  if (need_prog_round && presolved_m > 5000 && root.basis_hint) {
    // Collect fractional integer variables sorted by fractionality (least first).
    struct FracInfo { int idx; double frac; double rounded; };
    std::vector<FracInfo> frac_ints;
    int already_integer = 0;
    (void)already_integer;
    for (int i = 0; i < n; ++i) {
      if (!is_integer_type(base_lp.vars[i])) continue;
      double val = root.x_relax[i];
      double rounded = std::round(val);
      rounded = std::min(root.ub[i], std::max(root.lb[i], rounded));
      double frac = std::abs(val - rounded);
      if (frac < opt.int_tol) {
        ++already_integer;
      } else {
        frac_ints.push_back({i, frac, rounded});
      }
    }
    std::sort(frac_ints.begin(), frac_ints.end(),
              [](const FracInfo& a, const FracInfo& b) { return a.frac < b.frac; });
    const std::string prog_round_trace_detail =
        fmt::format("n_frac={} already_int={}", frac_ints.size(),
                    already_integer);
    trace_native_repair_bounds(
        "prog_simplex_round_candidates", base_lp, root_implied_integer_cols,
        root.x_relax, root.lb, root.ub, root.basis_hint.get(), opt.int_tol,
        prog_round_trace_detail.c_str());

    std::vector<int> frac_cols;
    frac_cols.reserve(frac_ints.size());
    for (const auto& fi : frac_ints) frac_cols.push_back(fi.idx);
    trace_native_repair_candidate_order(
        "prog_simplex_round_candidate_order", base_lp,
        root_implied_integer_cols, root.x_relax, root.lb, root.ub,
        root.basis_hint.get(), frac_cols, opt.int_tol,
        prog_round_trace_detail.c_str());
    auto prog_round_focus = build_prog_round_focus_trace(
      root.x_relax, root.lb, root.ub);
    annotate_prog_round_focus_frac_ranks(prog_round_focus, frac_cols);
    if (opt.verbose) {
      print_prog_round_focus_initial("PROG-ROUND", prog_round_focus);
    }

    const int n_frac = static_cast<int>(frac_ints.size());
    if (opt.verbose)
      fprintf(stderr, "[PROG-ROUND] n_frac=%d already_int=%d presolved_m=%d\n",
              n_frac, already_integer, presolved_m);

    const bool use_fast_large_near_integral_prog =
        large_near_integral_root && n_frac > 0 && n_frac <= 64;
    if (use_fast_large_near_integral_prog) {
      StandardFormLP fast_prog_sf = ensure_base_sf();
      auto fast_basis = root.basis_hint;
      Eigen::VectorXd fast_lb = root.lb;
      Eigen::VectorXd fast_ub = root.ub;
      Eigen::VectorXd last_good_x = root.x_relax;
      int fast_prog_lp_count = 0;
      const int max_fast_prog_lps = 6;

      auto try_rounding_fast = [&]() -> bool {
        Eigen::VectorXd xf = last_good_x;
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            xf[i] = std::round(xf[i]);
            xf[i] = std::min(root.ub[i], std::max(root.lb[i], xf[i]));
          }
        }
        if (satisfies_with_bounds(base_lp, xf, root.lb, root.ub, 1e-6)) {
          const double obj = objective_value(base_lp.c, xf, base_lp.sense);
          if (adopt_root_incumbent("prog_simplex_round", xf, obj)) {
            note_root_subsolve("prog_simplex_round", 0.0, true, true);
            if (cts) cts->shared_inc.try_update(xf, obj);
            return true;
          }
        }
        return false;
      };

      auto adopt_fixed_fast_solution = [&]() -> bool {
        Eigen::VectorXd xf = clamp_to_bounds(last_good_x, root.lb, root.ub);
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            xf[i] = std::round(xf[i]);
            xf[i] = std::min(root.ub[i], std::max(root.lb[i], xf[i]));
          }
        }
        // Bug fix: verify feasibility against base_lp before adopting as
        // incumbent. Rounding of a scaled-SF LP solution can introduce
        // violations on power-balance / flow constraints when continuous
        // vars depend on the rounded integer values. Without this check
        // we previously adopted infeasible incumbents on uc_200g/500g.
        if (!satisfies_with_bounds(base_lp, xf, root.lb, root.ub, 1e-6)) {
          return false;
        }
        const double obj = objective_value(base_lp.c, xf, base_lp.sense);
        if (adopt_root_incumbent("prog_simplex_round", xf, obj)) {
          note_root_subsolve("prog_simplex_round", 0.0, true, true);
          if (cts) cts->shared_inc.try_update(xf, obj);
          return true;
        }
        return false;
      };

      auto solve_fast_prog_lp = [&]() -> bool {
        update_standard_form_bounds(fast_prog_sf, base_lp, fast_lb, fast_ub);
        SimplexOptions spx_opt;
        spx_opt.lp_kernel_backend = opt.lp_kernel_backend;
        spx_opt.max_iter = 2000;
        spx_opt.feasibility_tol = 1e-7;
        spx_opt.optimality_tol = 1e-7;
        spx_opt.verbose = false;
        spx_opt.allow_cold_start = false;
        auto t_fast_prog0 = std::chrono::steady_clock::now();
        auto res = solve_lp_from_sf(fast_prog_sf, spx_opt, fast_basis.get());
        auto t_fast_prog1 = std::chrono::steady_clock::now();
        ++fast_prog_lp_count;
        out.bc_stats.lp_solves += 1;
        note_root_subsolve(
            "prog_simplex",
            std::chrono::duration<double, std::milli>(t_fast_prog1 - t_fast_prog0).count(),
            res.result.stats.success && res.result.x.size() == n);
        if (!res.result.stats.success || res.result.x.size() != n) return false;
        last_good_x = clamp_to_bounds(res.result.x, root.lb, root.ub);
        auto nb = std::make_shared<SimplexBasis>(res.basis);
        nb->cached_reduced_costs =
            std::make_shared<const Eigen::VectorXd>(std::move(res.reduced_costs));
        fast_basis = nb;
        return true;
      };

      int cursor = 0;
      int batch_sz = n_frac;
      while (cursor < n_frac && need_prog_round && fast_prog_lp_count < max_fast_prog_lps) {
        const int end = std::min(cursor + batch_sz, n_frac);
        for (int k = cursor; k < end; ++k) {
          fast_lb[frac_ints[k].idx] = frac_ints[k].rounded;
          fast_ub[frac_ints[k].idx] = frac_ints[k].rounded;
        }
        if (solve_fast_prog_lp()) {
          cursor = end;
          if ((cursor >= n_frac ? adopt_fixed_fast_solution() : try_rounding_fast())) {
            if (root_gap_closed() ||
                (std::isfinite(root.bound) && std::abs(root.bound) > 1e-9 &&
                 (incumbent_obj - root.bound) / std::abs(root.bound) <= prog_round_gap_threshold)) {
              need_prog_round = false;
            }
          }
          if (cursor < n_frac) {
            batch_sz = std::min(batch_sz, n_frac - cursor);
          }
        } else {
          for (int k = cursor; k < end; ++k) {
            fast_lb[frac_ints[k].idx] = root.lb[frac_ints[k].idx];
            fast_ub[frac_ints[k].idx] = root.ub[frac_ints[k].idx];
          }
          if (batch_sz <= 1) {
            ++cursor;
            batch_sz = std::max(1, n_frac - cursor);
          } else {
            batch_sz = std::max(1, batch_sz / 2);
          }
        }
      }
      if (has_incumbent) {
        need_prog_round = false;
      }
    }

    if (need_prog_round && n_frac > 0 && n_frac <= 5000) {
      Eigen::VectorXd prog_lb = root.lb, prog_ub = root.ub;
      Eigen::VectorXd last_good_x = root.x_relax;

      // Fix near-integer variables (needed for LP tightening).
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        double val = root.x_relax[i];
        double rounded = std::round(val);
        rounded = std::min(root.ub[i], std::max(root.lb[i], rounded));
        if (std::abs(val - rounded) < opt.int_tol) {
          prog_lb[i] = rounded;
          prog_ub[i] = rounded;
        }
      }
      if (opt.verbose) {
        log_prog_round_focus_changes("PROG-ROUND", "direct-near-int",
                                     prog_round_focus, prog_lb, prog_ub);
      }

      // Helper: try rounding the current LP solution.
      auto try_rounding = [&]() -> bool {
        Eigen::VectorXd xf = last_good_x;
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            xf[i] = std::round(xf[i]);
            xf[i] = std::min(root.ub[i], std::max(root.lb[i], xf[i]));
          }
        }
        if (satisfies_with_bounds(base_lp, xf, root.lb, root.ub, 1e-6)) {
          const double obj = objective_value(base_lp.c, xf, base_lp.sense);
          if (adopt_root_incumbent("prog_simplex_round", xf, obj)) {
            note_root_subsolve("prog_simplex_round", 0.0, true, true);
            if (cts) cts->shared_inc.try_update(xf, obj);
            return true;
          }
        }
        return false;
      };

      // ── Constraint propagation for progressive rounding ──
      // Pre-compute row-major matrices once for repeated propagation calls.
      Eigen::SparseMatrix<double, Eigen::RowMajor> prop_A_rm = base_lp.A;
      Eigen::SparseMatrix<double, Eigen::RowMajor> prop_Aeq_rm = base_lp.Aeq;

      // Track which fractional vars are fixed by propagation (not by batch).
      std::vector<bool> prop_fixed(static_cast<size_t>(n_frac), false);

      // Initial propagation: exploit near-integer fixings to pre-fix fractional vars.
      {
        int pre_prop = node_bound_propagation(base_lp, prop_A_rm, prop_Aeq_rm,
                                              prog_lb, prog_ub, 3);
        int pre_fixed = 0;
        for (int k = 0; k < n_frac; ++k) {
          if (prog_ub[frac_ints[k].idx] - prog_lb[frac_ints[k].idx] < 1e-9) {
            prop_fixed[static_cast<size_t>(k)] = true;
            frac_ints[k].rounded = prog_lb[frac_ints[k].idx];
            ++pre_fixed;
          }
        }
        if (opt.verbose && (pre_prop > 0 || pre_fixed > 0)) {
          fprintf(stderr, "[PROG-ROUND] pre-propagation: tightened=%d frac_fixed=%d\n",
                  pre_prop, pre_fixed);
        }
        if (opt.verbose) {
          log_prog_round_focus_changes("PROG-ROUND", "propagation-pre",
                                       prog_round_focus, prog_lb, prog_ub);
        }
      }

      // Helper: fix/unfix ranges of frac_ints (skips propagation-fixed vars).
      auto fix_range = [&](int from, int to) {
        for (int k = from; k < to; ++k) {
          prog_lb[frac_ints[k].idx] = frac_ints[k].rounded;
          prog_ub[frac_ints[k].idx] = frac_ints[k].rounded;
        }
      };
      auto unfix_range = [&](int from, int to) {
        for (int k = from; k < to; ++k) {
          if (prop_fixed[static_cast<size_t>(k)]) continue;  // Keep propagation fixings
          prog_lb[frac_ints[k].idx] = root.lb[frac_ints[k].idx];
          prog_ub[frac_ints[k].idx] = root.ub[frac_ints[k].idx];
        }
      };

      // Helper: propagate after batch and mark newly fixed vars.
      auto propagate_and_skip = [&](int& cur) {
        node_bound_propagation(base_lp, prop_A_rm, prop_Aeq_rm,
                               prog_lb, prog_ub, 3);
        for (int k = cur; k < n_frac; ++k) {
          if (!prop_fixed[static_cast<size_t>(k)] &&
              prog_ub[frac_ints[k].idx] - prog_lb[frac_ints[k].idx] < 1e-9) {
            prop_fixed[static_cast<size_t>(k)] = true;
            frac_ints[k].rounded = prog_lb[frac_ints[k].idx];
          }
        }
        if (opt.verbose) {
          log_prog_round_focus_changes("PROG-ROUND", "propagation-post-batch",
                                       prog_round_focus, prog_lb, prog_ub);
        }
        while (cur < n_frac && prog_ub[frac_ints[cur].idx] - prog_lb[frac_ints[cur].idx] < 1e-9) ++cur;
      };

      // ── Simplex progressive rounding ──
      {
        StandardFormLP prog_sf = ensure_base_sf();
        auto prog_basis = root.basis_hint;
        last_good_x = root.x_relax;

        // Fix near-integer variables (needed for LP tightening).
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          double val = root.x_relax[i];
          double rounded = std::round(val);
          rounded = std::min(root.ub[i], std::max(root.lb[i], rounded));
          if (std::abs(val - rounded) < opt.int_tol) {
            prog_lb[i] = rounded;
            prog_ub[i] = rounded;
          }
        }

        int prog_spx_lp_count = 0;
        auto solve_prog_lp = [&]() -> bool {
          update_standard_form_bounds(prog_sf, base_lp, prog_lb, prog_ub);
          SimplexOptions spx_opt;
          spx_opt.lp_kernel_backend = opt.lp_kernel_backend;
          spx_opt.max_iter = 2000;
          spx_opt.feasibility_tol = 1e-7;
          spx_opt.optimality_tol = 1e-7;
          spx_opt.verbose = false;
          spx_opt.allow_cold_start = false;
            auto t_prog_lp0 = std::chrono::steady_clock::now();
          auto res = solve_lp_from_sf(prog_sf, spx_opt, prog_basis.get());
            auto t_prog_lp1 = std::chrono::steady_clock::now();
          ++prog_spx_lp_count;
          out.bc_stats.lp_solves += 1;
            note_root_subsolve(
              "prog_simplex",
              std::chrono::duration<double, std::milli>(t_prog_lp1 - t_prog_lp0).count(),
              res.result.stats.success && res.result.x.size() == n);
          if (!res.result.stats.success || res.result.x.size() != n) return false;
          last_good_x = clamp_to_bounds(res.result.x, root.lb, root.ub);
          auto nb = std::make_shared<SimplexBasis>(res.basis);
          nb->cached_reduced_costs =
              std::make_shared<const Eigen::VectorXd>(std::move(res.reduced_costs));
          prog_basis = nb;
          return true;
        };

        // ── Adaptive-batch progressive rounding with constraint propagation ──
        // After each successful batch fix, constraint propagation exploits
        // min-up/min-down time and other structural implications to fix
        // additional integer variables for free (no LP solve needed).

        // Scale down the LP-count budget as row count grows. In parallel mode,
        // reserve half of this optional budget for the tree workers.
        int max_prog_lps = (presolved_m > 20000) ? std::min(40, std::max(10, n_frac))
            : (presolved_m > 10000) ? std::min(50, std::max(15, n_frac))
            : std::min(200, std::max(40, n_frac * 2));
        if (par_mode) max_prog_lps = std::max(5, max_prog_lps / 2);
        if (large_generic_root) max_prog_lps = std::min(max_prog_lps, 6);
        // Stage-3 sub-MIP: budget is already tight; cap prog-round LPs to
        // avoid blowing past the parent's time_limit_sec.
        if (opt._is_stage3_submip) max_prog_lps = std::min(max_prog_lps, 20);
        int cursor = 0;
        // Skip vars already fixed by pre-propagation.
        while (cursor < n_frac && prog_ub[frac_ints[cursor].idx] - prog_lb[frac_ints[cursor].idx] < 1e-9) ++cursor;
        const int remaining = n_frac - cursor;
        // Start with n_frac/4 for medium-large sets to avoid 2-3 wasted
        // speculative fix-all LPs (each ~24ms simplex).  Only try fix-all
        // when n_frac ≤ 20 where the chance of success is high.
        int batch_sz;
        if (remaining <= 20) {
          batch_sz = std::max(1, remaining);  // Speculative fix-all
        } else {
          batch_sz = std::max(1, remaining / 4);
        }
        int last_success_sz = batch_sz;

        // Wall-time budget for this prog-round phase. Protects against a few
        // slow LPs (e.g. m>50k at 1-2s each) chewing through the whole budget.
        const auto prog_spx_phase_t0 = std::chrono::steady_clock::now();
        const double prog_spx_wall_budget_sec = large_generic_root
            ? std::max(1.0, std::min(2.0, 0.08 * opt.time_limit_sec))
            : std::max(1.0, std::min(30.0, 0.40 * opt.time_limit_sec));
        auto prog_spx_time_exhausted = [&]() {
          return std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - prog_spx_phase_t0).count()
                 > prog_spx_wall_budget_sec;
        };

        while (cursor < n_frac && need_prog_round && prog_spx_lp_count < max_prog_lps) {
          if (prog_spx_time_exhausted()) {
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND] wall-time %.1fs exhausted — aborting\n",
                      prog_spx_wall_budget_sec);
            break;
          }
          int end = std::min(cursor + batch_sz, n_frac);
          fix_range(cursor, end);
          if (opt.verbose) {
            log_prog_round_focus_changes(
                "PROG-ROUND",
                "direct-batch[" + std::to_string(cursor) + "," + std::to_string(end) + ")",
                prog_round_focus, prog_lb, prog_ub);
          }
          if (solve_prog_lp()) {
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND] batch [%d,%d) OK sz=%d lps=%d\n",
                      cursor, end, end - cursor, prog_spx_lp_count);
            last_success_sz = end - cursor;
            cursor = end;
            // Propagate: tighten bounds and skip newly implied fixings.
            propagate_and_skip(cursor);
            if (try_rounding()) {
              if (opt.verbose)
                fprintf(stderr, "[PROG-ROUND] ROOT INCUMBENT obj=%.2f lps=%d\n",
                        incumbent_obj, prog_spx_lp_count);
              // Stop prog-round when gap is tight enough
              if (root_gap_closed() ||
                  (std::isfinite(root.bound) && std::abs(root.bound) > 1e-9 &&
                   (incumbent_obj - root.bound) / std::abs(root.bound) <= prog_round_gap_threshold))
                need_prog_round = false;
            }
          } else {
            // Failed — unfix this batch and halve the batch size.
            unfix_range(cursor, end);
            if (opt.verbose) {
              log_prog_round_focus_changes(
                  "PROG-ROUND",
                  "revert-batch[" + std::to_string(cursor) + "," + std::to_string(end) + ")",
                  prog_round_focus, prog_lb, prog_ub);
            }
            if (batch_sz <= 1) {
              ++cursor;  // Skip this unfixable var.
              // Skip propagation-fixed vars.
              while (cursor < n_frac && prog_ub[frac_ints[cursor].idx] - prog_lb[frac_ints[cursor].idx] < 1e-9) ++cursor;
              // Restart from last successful batch size rather than n_frac/5
              // to avoid oscillation between too-large and too-small batches.
              batch_sz = std::max(1, std::min(last_success_sz, n_frac - cursor));
            } else {
              batch_sz = std::max(1, batch_sz / 2);
            }
          }
        }

        // Residual dive: try fixing any still-unfixed vars individually.
        if (need_prog_round) {
          for (int k = 0; k < n_frac && need_prog_round && prog_spx_lp_count < max_prog_lps; ++k) {
            if (prog_spx_time_exhausted()) break;
            const auto& fi = frac_ints[k];
            if (prog_lb[fi.idx] == prog_ub[fi.idx]) continue;  // Already fixed.
            double near = std::round(last_good_x[fi.idx]);
            double far = (near > last_good_x[fi.idx]) ? std::floor(last_good_x[fi.idx])
                                                       : std::ceil(last_good_x[fi.idx]);
            for (int attempt = 0; attempt < 2; ++attempt) {
              double v = (attempt == 0) ? near : far;
              v = std::min(root.ub[fi.idx], std::max(root.lb[fi.idx], v));
              prog_lb[fi.idx] = v;
              prog_ub[fi.idx] = v;
              if (opt.verbose) {
                log_prog_round_focus_changes(
                    "PROG-ROUND",
                    "direct-dive[k=" + std::to_string(k) + ",attempt=" +
                        std::to_string(attempt) + "]",
                    prog_round_focus, prog_lb, prog_ub);
              }
              if (solve_prog_lp()) {
                if (try_rounding()) {
                  if (opt.verbose)
                    fprintf(stderr, "[PROG-ROUND] ROOT INCUMBENT via dive obj=%.2f fixed=%d/%d lps=%d\n",
                            incumbent_obj, k + 1, n_frac, prog_spx_lp_count);
                  if (root_gap_closed() ||
                      (std::isfinite(root.bound) && std::abs(root.bound) > 1e-9 &&
                       (incumbent_obj - root.bound) / std::abs(root.bound) <= prog_round_gap_threshold))
                    need_prog_round = false;
                }
                break;
              }
              if (attempt == 0) {
                prog_lb[fi.idx] = root.lb[fi.idx];
                prog_ub[fi.idx] = root.ub[fi.idx];
                if (opt.verbose) {
                  log_prog_round_focus_changes(
                      "PROG-ROUND",
                      "revert-dive[k=" + std::to_string(k) + ",attempt=" +
                          std::to_string(attempt) + "]",
                      prog_round_focus, prog_lb, prog_ub);
                }
              }
            }
          }
        }

        if (opt.verbose)
          fprintf(stderr, "[PROG-ROUND] done: lps=%d incumbent=%d\n",
                  prog_spx_lp_count, has_incumbent ? 1 : 0);

        // ── Polishing: flip uncertain variables toward the opposite integer ──
        // After progressive rounding, variables with high LP fractionality
        // (>0.3) are most likely to have been rounded incorrectly.  Try
        // flipping them in batches to see if the incumbent improves.
        // Skip polishing when gap is already within tolerance or problem is large.
        if (has_incumbent && prog_spx_lp_count < max_prog_lps
            && !root_gap_closed()) {
          // Identify uncertain variables: those with LP frac > 0.3 where
          // the incumbent differs from the LP-opposite direction.
          struct FlipCandidate {
            int idx;
            double cur_val;  // current incumbent value
            double alt_val;  // alternative (opposite) value
            double lp_frac;  // LP fractionality (closer to 0.5 = more uncertain)
          };
          std::vector<FlipCandidate> flip_cands;
          for (const auto& fi : frac_ints) {
            double lp_val = root.x_relax[fi.idx];
            double lp_frac = std::abs(lp_val - std::round(lp_val));
            if (lp_frac < 0.3) continue;
            double cur = std::round(incumbent_x[fi.idx]);
            double alt = (cur < 0.5) ? 1.0 : 0.0;
            alt = std::max(root.lb[fi.idx], std::min(root.ub[fi.idx], alt));
            if (std::abs(alt - cur) < 0.5) continue;  // Can't flip
            flip_cands.push_back({fi.idx, cur, alt, lp_frac});
          }
          // Sort by decreasing uncertainty (highest frac first)
          std::sort(flip_cands.begin(), flip_cands.end(),
                    [](const FlipCandidate& a, const FlipCandidate& b) {
                      return a.lp_frac > b.lp_frac;
                    });

          if (!flip_cands.empty() && opt.verbose) {
            fprintf(stderr, "[PROG-ROUND-POLISH] %d flip candidates (frac > 0.3)\n",
                    static_cast<int>(flip_cands.size()));
          }

          // Build clean incumbent bound snapshot: integers fixed, continuous at root bounds
          Eigen::VectorXd inc_lb(n), inc_ub(n);
          for (int i = 0; i < n; ++i) {
            if (is_integer_type(base_lp.vars[i])) {
              double v = std::round(incumbent_x[i]);
              v = std::max(root.lb[i], std::min(root.ub[i], v));
              inc_lb[i] = v;
              inc_ub[i] = v;
            } else {
              inc_lb[i] = root.lb[i];
              inc_ub[i] = root.ub[i];
            }
          }

          // Cold-start-enabled LP solve for polishing (bounds differ greatly from batch phase)
          auto solve_polish_lp = [&]() -> bool {
            update_standard_form_bounds(prog_sf, base_lp, prog_lb, prog_ub);
            SimplexOptions spx_opt;
            spx_opt.lp_kernel_backend = opt.lp_kernel_backend;
            spx_opt.max_iter = 2000;
            spx_opt.feasibility_tol = 1e-7;
            spx_opt.optimality_tol = 1e-7;
            spx_opt.verbose = false;
            spx_opt.allow_cold_start = true;  // Polishing changes many bounds at once
            auto t_polish_lp0 = std::chrono::steady_clock::now();
            auto res = solve_lp_from_sf(prog_sf, spx_opt, prog_basis.get());
            auto t_polish_lp1 = std::chrono::steady_clock::now();
            ++prog_spx_lp_count;
            out.bc_stats.lp_solves += 1;
            note_root_subsolve(
              "prog_simplex_polish",
              std::chrono::duration<double, std::milli>(t_polish_lp1 - t_polish_lp0).count(),
              res.result.stats.success && res.result.x.size() == n);
            if (!res.result.stats.success || res.result.x.size() != n) return false;
            last_good_x = clamp_to_bounds(res.result.x, root.lb, root.ub);
            auto nb = std::make_shared<SimplexBasis>(res.basis);
            nb->cached_reduced_costs =
                std::make_shared<const Eigen::VectorXd>(std::move(res.reduced_costs));
            prog_basis = nb;
            return true;
          };

          // ── Group-flip polishing with binary search ──
          // Individual flips are too local to bridge the ~600 cost gap for SCUC.
          // Instead, try flipping ALL uncertain vars at once.  If infeasible,
          // binary-search: keep top half (highest fractionality) and retry.
          // This explores larger neighborhoods efficiently.
          int polish_ok = 0, polish_fail = 0;
          bool polish_improved = false;

          // Phase 1: try flipping ALL candidates at once
          {
            prog_lb = inc_lb;
            prog_ub = inc_ub;
            for (const auto& fc : flip_cands) {
              prog_lb[fc.idx] = fc.alt_val;
              prog_ub[fc.idx] = fc.alt_val;
            }
            if (solve_polish_lp()) {
              ++polish_ok;
              if (try_rounding()) {
                polish_improved = true;
                if (opt.verbose)
                  fprintf(stderr, "[PROG-ROUND-POLISH] IMPROVED obj=%.2f flips=%d lps=%d\n",
                          incumbent_obj, static_cast<int>(flip_cands.size()),
                          prog_spx_lp_count);
                for (int i = 0; i < n; ++i) {
                  if (is_integer_type(base_lp.vars[i])) {
                    double v = std::round(incumbent_x[i]);
                    v = std::max(root.lb[i], std::min(root.ub[i], v));
                    inc_lb[i] = v;
                    inc_ub[i] = v;
                  }
                }
              }
            } else {
              ++polish_fail;
            }
          }

          // Phase 2: binary-search if full flip failed/didn't improve
          if (!polish_improved && flip_cands.size() > 1 &&
              prog_spx_lp_count < max_prog_lps) {
            // Binary search on subset size: try top-k most fractional candidates
            int lo = 1, hi = static_cast<int>(flip_cands.size()) - 1;
            while (lo <= hi && prog_spx_lp_count < max_prog_lps) {
              int mid = (lo + hi) / 2;
              prog_lb = inc_lb;
              prog_ub = inc_ub;
              // Flip top-mid candidates (flip_cands is sorted by decreasing frac)
              for (int fi = 0; fi < mid; ++fi) {
                prog_lb[flip_cands[fi].idx] = flip_cands[fi].alt_val;
                prog_ub[flip_cands[fi].idx] = flip_cands[fi].alt_val;
              }
              if (solve_polish_lp()) {
                ++polish_ok;
                if (try_rounding()) {
                  polish_improved = true;
                  if (opt.verbose)
                    fprintf(stderr, "[PROG-ROUND-POLISH] IMPROVED obj=%.2f flips=%d lps=%d\n",
                            incumbent_obj, mid, prog_spx_lp_count);
                  for (int i = 0; i < n; ++i) {
                    if (is_integer_type(base_lp.vars[i])) {
                      double v = std::round(incumbent_x[i]);
                      v = std::max(root.lb[i], std::min(root.ub[i], v));
                      inc_lb[i] = v;
                      inc_ub[i] = v;
                    }
                  }
                  break;  // Found improvement — stop
                }
                // LP feasible but didn't improve — try more flips
                lo = mid + 1;
              } else {
                ++polish_fail;
                // Too many flips — try fewer
                hi = mid - 1;
              }
            }
          }

          // Phase 3: individual flips on remaining candidates (quick scan)
          if (!polish_improved && prog_spx_lp_count < max_prog_lps) {
            for (int fi = 0; fi < static_cast<int>(flip_cands.size()) &&
                 prog_spx_lp_count < max_prog_lps; ++fi) {
              prog_lb = inc_lb;
              prog_ub = inc_ub;
              prog_lb[flip_cands[fi].idx] = flip_cands[fi].alt_val;
              prog_ub[flip_cands[fi].idx] = flip_cands[fi].alt_val;
              if (solve_polish_lp()) {
                ++polish_ok;
                if (try_rounding()) {
                  if (opt.verbose)
                    fprintf(stderr, "[PROG-ROUND-POLISH] IMPROVED obj=%.2f var=%d lps=%d\n",
                            incumbent_obj, flip_cands[fi].idx, prog_spx_lp_count);
                  for (int i = 0; i < n; ++i) {
                    if (is_integer_type(base_lp.vars[i])) {
                      double v = std::round(incumbent_x[i]);
                      v = std::max(root.lb[i], std::min(root.ub[i], v));
                      inc_lb[i] = v;
                      inc_ub[i] = v;
                    }
                  }
                }
              } else {
                ++polish_fail;
              }
            }
          }

          if (opt.verbose && !flip_cands.empty()) {
            fprintf(stderr, "[PROG-ROUND-POLISH] done: ok=%d fail=%d lps=%d improved=%d\n",
                    polish_ok, polish_fail, prog_spx_lp_count,
                    polish_improved ? 1 : 0);
          }
          // Restore prog_lb/prog_ub to current incumbent for subsequent use
          prog_lb = inc_lb;
          prog_ub = inc_ub;
        }
      }

      if (has_incumbent && opt.verbose) {
        fprintf(stderr, "[PROG-ROUND] root incumbent obj=%.2f\n", incumbent_obj);
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // IPM-based progressive rounding for large problems without a simplex
  // basis. Near-integer variables are not all passed as lb=ub because mass
  // fixing can make the KKT system ill-conditioned. Propagation sees full
  // fixings; IPM sees only batch and propagation-implied fixings.
  // ════════════════════════════════════════════════════════════════════════
  // Cap this optional block to a fraction of the user time limit so it cannot
  // consume the budget reserved for branch-and-bound.
  const double t_elapsed_before_prog_ipm =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double prog_ipm_wall_budget =
      std::max(1.0, std::min(15.0, 0.30 * opt.time_limit_sec));
  const bool skip_prog_ipm_by_time =
      (t_elapsed_before_prog_ipm > 0.40 * opt.time_limit_sec);
  if (need_prog_round && presolved_m > 5000 && !root.basis_hint &&
      !skip_prog_ipm_by_time) {
    // Dedicated IPM with a fixed iteration and wall-time budget.
    std::unique_ptr<NativeIPMLPAdapter> prog_round_ipm_adapter;
    {
      IPMLPOptions pr_opt;
      pr_opt.max_iter = 80;
      pr_opt.tol_primal = 1e-7;
      pr_opt.tol_dual = 1e-7;
      pr_opt.tol_gap = 1e-7;
      pr_opt.verbose = false;
      prog_round_ipm_adapter = std::make_unique<NativeIPMLPAdapter>(pr_opt);
      prog_round_ipm_adapter->prepare_for_node_solves(base_lp);
    }
    auto* prog_ipm = prog_round_ipm_adapter.get();
    if (prog_ipm) {
      struct FracInfoIPM { int idx; double frac; double rounded; };
      std::vector<FracInfoIPM> frac_ints_ipm;
      int already_int_ipm = 0;
      std::vector<bool> near_int_var(static_cast<size_t>(n), false);
      for (int i = 0; i < n; ++i) {
        if (!is_integer_type(base_lp.vars[i])) continue;
        double val = root.x_relax[i];
        double rounded = std::round(val);
        rounded = std::min(root.ub[i], std::max(root.lb[i], rounded));
        double frac = std::abs(val - rounded);
        if (frac < opt.int_tol) {
          ++already_int_ipm;
          near_int_var[static_cast<size_t>(i)] = true;
        } else {
          frac_ints_ipm.push_back({i, frac, rounded});
        }
      }
      std::sort(frac_ints_ipm.begin(), frac_ints_ipm.end(),
                [](const FracInfoIPM& a, const FracInfoIPM& b) { return a.frac < b.frac; });

      std::vector<int> frac_cols_ipm;
      frac_cols_ipm.reserve(frac_ints_ipm.size());
      for (const auto& fi : frac_ints_ipm) frac_cols_ipm.push_back(fi.idx);
        auto prog_round_focus_ipm = build_prog_round_focus_trace(
          root.x_relax, root.lb, root.ub);
      annotate_prog_round_focus_frac_ranks(prog_round_focus_ipm, frac_cols_ipm);
      if (opt.verbose) {
        print_prog_round_focus_initial("PROG-ROUND-IPM", prog_round_focus_ipm);
      }

      const int n_frac_ipm = static_cast<int>(frac_ints_ipm.size());
      if (opt.verbose)
        fprintf(stderr, "[PROG-ROUND-IPM] n_frac=%d already_int=%d presolved_m=%d\n",
                n_frac_ipm, already_int_ipm, presolved_m);

      if (n_frac_ipm > 0 && n_frac_ipm <= 8000) {
        // Two-level bounds:
        //   prop_lb/ub: includes near-integer fixings (drives propagation)
        //   solve_lb/ub: NO near-integer fixings (avoids singular IPM KKT)
        Eigen::VectorXd prop_lb = root.lb, prop_ub = root.ub;
        Eigen::VectorXd solve_lb = root.lb, solve_ub = root.ub;
        Eigen::VectorXd last_good_x = root.x_relax;

        // Fix near-integer vars in propagation bounds (exact lb=ub).
        // In solve bounds, TIGHTEN to [v-0.5, v+0.5] instead of fixing —
        // this keeps the IPM KKT well-conditioned (no lb=ub) while ensuring
        // the var stays in the correct rounding half-plane.
        for (int i = 0; i < n; ++i) {
          if (near_int_var[static_cast<size_t>(i)]) {
            double rounded = std::round(root.x_relax[i]);
            rounded = std::min(root.ub[i], std::max(root.lb[i], rounded));
            prop_lb[i] = rounded;
            prop_ub[i] = rounded;
            // Tighten solve bounds: narrow enough to prevent drift (delta 0.01)
            // while keeping IPM KKT stable (range >> 0).  A near-int var at
            // 0.999 gets [0.99,1.0]; after IPM solve it stays ~0.995; rounding
            // perturbation is ~0.005 → constraint violation ≤ 0.5 per var.
            solve_lb[i] = std::max(root.lb[i], rounded - 0.01);
            solve_ub[i] = std::min(root.ub[i], rounded + 0.01);
          }
        }
        if (opt.verbose) {
          log_prog_round_focus_changes("PROG-ROUND-IPM", "direct-near-int",
                                       prog_round_focus_ipm, prop_lb, prop_ub);
        }

        auto try_rounding_ipm = [&]() -> bool {
          Eigen::VectorXd xf = last_good_x;
          if (reduced_uc_hint.has_value()) {
            uc_round_commitment(xf, *reduced_uc_hint, reduced_uc_hint->ng);
          }
          for (int i = 0; i < n; ++i) {
            if (is_integer_type(base_lp.vars[i])) {
              xf[i] = std::round(xf[i]);
              xf[i] = std::min(root.ub[i], std::max(root.lb[i], xf[i]));
            }
          }
          // Diagnostics: measure constraint violation after rounding.
          // Use matrix-vector products (col-major friendly) rather than
          // per-row .dot(), which materialises each row and costs O(m·nnz/n)
          // extra work — catastrophic on xlarge UC (59k rows).  Guard the
          // computation behind `verbose` since the result is only logged.
          if (opt.verbose) {
            double max_ineq_viol = 0, max_eq_viol = 0;
            int worst_ineq = -1, worst_eq = -1;
            const Eigen::VectorXd Ax = base_lp.A * xf;
            for (int i = 0; i < static_cast<int>(Ax.size()); ++i) {
              const double v = Ax[i] - base_lp.b[i];
              if (v > max_ineq_viol) { max_ineq_viol = v; worst_ineq = i; }
            }
            const Eigen::VectorXd Aeqx = base_lp.Aeq * xf;
            for (int i = 0; i < static_cast<int>(Aeqx.size()); ++i) {
              const double v = std::abs(Aeqx[i] - base_lp.beq[i]);
              if (v > max_eq_viol) { max_eq_viol = v; worst_eq = i; }
            }
            int n_nonint = 0;
            for (int i = 0; i < n; ++i) {
              if (is_integer_type(base_lp.vars[i]) && !is_integral(xf[i], 1e-6)) ++n_nonint;
            }
            fprintf(stderr, "[PROG-ROUND-IPM] rounding check: ineq_viol=%.2e(row%d) eq_viol=%.2e(row%d) nonint=%d\n",
                    max_ineq_viol, worst_ineq, max_eq_viol, worst_eq, n_nonint);
          }

          const double ipm_feas_tol = 1e-4;
          if (satisfies_with_bounds(base_lp, xf, root.lb, root.ub, ipm_feas_tol)) {
            const double obj = objective_value(base_lp.c, xf, base_lp.sense);
            if (adopt_root_incumbent("prog_ipm_round", xf, obj)) {
              note_root_subsolve("prog_ipm_round", 0.0, true, true);
              if (cts) cts->shared_inc.try_update(xf, obj);
              return true;
            }
          }

          // Repair LP: round created constraint violations — try IPM repair
          // with fixed integer bounds to find feasible continuous variables.
          {
            Eigen::VectorXd repair_lb_ipm = root.lb, repair_ub_ipm = root.ub;
            for (int i = 0; i < n; ++i) {
              if (is_integer_type(base_lp.vars[i])) {
                repair_lb_ipm[i] = xf[i];
                repair_ub_ipm[i] = xf[i];
              }
            }
            auto t_prog_repair0 = std::chrono::steady_clock::now();
            auto repair_res = prog_ipm->solve_node_lp(base_lp, repair_lb_ipm, repair_ub_ipm, xf);
            auto t_prog_repair1 = std::chrono::steady_clock::now();
            out.bc_stats.lp_solves += 1;
            note_root_subsolve(
              "prog_ipm_repair",
              std::chrono::duration<double, std::milli>(t_prog_repair1 - t_prog_repair0).count(),
              repair_res.stats.success && repair_res.x.size() >= n);
            if (repair_res.stats.success && repair_res.x.size() >= n) {
              Eigen::VectorXd xr = clamp_to_bounds(repair_res.x.head(n), root.lb, root.ub);
              // Re-snap integers.
              for (int i = 0; i < n; ++i) {
                if (is_integer_type(base_lp.vars[i])) {
                  xr[i] = std::round(xr[i]);
                  xr[i] = std::min(root.ub[i], std::max(root.lb[i], xr[i]));
                }
              }
              if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub, ipm_feas_tol)) {
                const double obj = objective_value(base_lp.c, xr, base_lp.sense);
                if (adopt_root_incumbent("prog_ipm_repair_round", xr, obj)) {
                  note_root_subsolve("prog_ipm_repair_round", 0.0, true, true);
                  if (cts) cts->shared_inc.try_update(xr, obj);
                  if (opt.verbose)
                    fprintf(stderr, "[PROG-ROUND-IPM] repair LP found incumbent obj=%.2f\n", obj);
                  return true;
                }
              }
            }
          }

          return false;
        };

        // Propagation structures (use prop bounds with near-integer fixings).
        Eigen::SparseMatrix<double, Eigen::RowMajor> iprop_A_rm = base_lp.A;
        Eigen::SparseMatrix<double, Eigen::RowMajor> iprop_Aeq_rm = base_lp.Aeq;
        std::vector<bool> iprop_fixed(static_cast<size_t>(n_frac_ipm), false);

        // Initial propagation from near-integer fixings.
        {
          int pre_prop = node_bound_propagation(base_lp, iprop_A_rm, iprop_Aeq_rm,
                                                prop_lb, prop_ub, 3);
          int pre_fixed = 0;
          for (int k = 0; k < n_frac_ipm; ++k) {
            if (prop_ub[frac_ints_ipm[k].idx] - prop_lb[frac_ints_ipm[k].idx] < 1e-9) {
              iprop_fixed[static_cast<size_t>(k)] = true;
              frac_ints_ipm[k].rounded = prop_lb[frac_ints_ipm[k].idx];
              // Propagation-implied fixings go into BOTH bound sets.
              solve_lb[frac_ints_ipm[k].idx] = prop_lb[frac_ints_ipm[k].idx];
              solve_ub[frac_ints_ipm[k].idx] = prop_ub[frac_ints_ipm[k].idx];
              ++pre_fixed;
            }
          }
          if (pre_prop > 0 || pre_fixed > 0)
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND-IPM] pre-propagation: tightened=%d frac_fixed=%d\n",
                      pre_prop, pre_fixed);
          if (opt.verbose) {
            log_prog_round_focus_changes("PROG-ROUND-IPM", "propagation-pre",
                                         prog_round_focus_ipm, prop_lb, prop_ub);
          }
        }

        // Fix/unfix helpers apply to BOTH bound sets.
        auto fix_range_ipm = [&](int from, int to) {
          for (int k = from; k < to; ++k) {
            const int idx = frac_ints_ipm[k].idx;
            const double v = frac_ints_ipm[k].rounded;
            prop_lb[idx] = v;  prop_ub[idx] = v;
            solve_lb[idx] = v; solve_ub[idx] = v;
          }
        };
        auto unfix_range_ipm = [&](int from, int to) {
          for (int k = from; k < to; ++k) {
            if (iprop_fixed[static_cast<size_t>(k)]) continue;
            const int idx = frac_ints_ipm[k].idx;
            prop_lb[idx] = root.lb[idx];  prop_ub[idx] = root.ub[idx];
            solve_lb[idx] = root.lb[idx]; solve_ub[idx] = root.ub[idx];
          }
        };
        auto propagate_and_skip_ipm = [&](int& cur) {
          node_bound_propagation(base_lp, iprop_A_rm, iprop_Aeq_rm,
                                 prop_lb, prop_ub, 3);
          for (int k = cur; k < n_frac_ipm; ++k) {
            if (!iprop_fixed[static_cast<size_t>(k)] &&
                prop_ub[frac_ints_ipm[k].idx] - prop_lb[frac_ints_ipm[k].idx] < 1e-9) {
              iprop_fixed[static_cast<size_t>(k)] = true;
              frac_ints_ipm[k].rounded = prop_lb[frac_ints_ipm[k].idx];
              solve_lb[frac_ints_ipm[k].idx] = prop_lb[frac_ints_ipm[k].idx];
              solve_ub[frac_ints_ipm[k].idx] = prop_ub[frac_ints_ipm[k].idx];
            }
          }
          if (opt.verbose) {
            log_prog_round_focus_changes("PROG-ROUND-IPM", "propagation-post-batch",
                                         prog_round_focus_ipm, prop_lb, prop_ub);
          }
          while (cur < n_frac_ipm &&
                 prop_ub[frac_ints_ipm[cur].idx] - prop_lb[frac_ints_ipm[cur].idx] < 1e-9)
            ++cur;
        };

        // IPM LP solver — uses solve_lb/ub (no near-integer fixings).
        int prog_ipm_lp_count = 0;
        // Scale the LP-count budget down as row count grows.
        const int max_prog_ipm_lps = (presolved_m > 25000) ? 20
            : (presolved_m > 15000) ? 25 : 30;
        // A cumulative wall-time cap prevents this optional block from
        // starving branch-and-bound regardless of per-LP runtime.
        const auto t_prog_ipm_block_start = std::chrono::steady_clock::now();
        auto prog_ipm_time_ok = [&]() {
          return std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - t_prog_ipm_block_start)
                     .count() < prog_ipm_wall_budget;
        };
        auto solve_prog_ipm_lp = [&]() -> bool {
          auto t_prog_ipm0 = std::chrono::steady_clock::now();
          auto res = prog_ipm->solve_node_lp(base_lp, solve_lb, solve_ub, last_good_x);
          auto t_prog_ipm1 = std::chrono::steady_clock::now();
          ++prog_ipm_lp_count;
          out.bc_stats.lp_solves += 1;
          note_root_subsolve(
            "prog_ipm",
            std::chrono::duration<double, std::milli>(t_prog_ipm1 - t_prog_ipm0).count(),
            res.stats.success && res.x.size() == n);
          if (!res.stats.success || res.x.size() != n) return false;
          last_good_x = clamp_to_bounds(res.x, root.lb, root.ub);
          return true;
        };

        // ── Adaptive-batch progressive rounding via IPM ──
        int cursor = 0;
        while (cursor < n_frac_ipm &&
               prop_ub[frac_ints_ipm[cursor].idx] - prop_lb[frac_ints_ipm[cursor].idx] < 1e-9)
          ++cursor;
        int batch_sz = std::max(1, n_frac_ipm - cursor);

        while (cursor < n_frac_ipm && need_prog_round && prog_ipm_lp_count < max_prog_ipm_lps) {
          if (!prog_ipm_time_ok()) {
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND-IPM] wall-time budget (%.1fs) exceeded — aborting\n",
                      prog_ipm_wall_budget);
            break;
          }
          int end = std::min(cursor + batch_sz, n_frac_ipm);
          fix_range_ipm(cursor, end);
          if (opt.verbose) {
            log_prog_round_focus_changes(
                "PROG-ROUND-IPM",
                "direct-batch[" + std::to_string(cursor) + "," + std::to_string(end) + ")",
                prog_round_focus_ipm, prop_lb, prop_ub);
          }
          if (solve_prog_ipm_lp()) {
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND-IPM] batch [%d,%d) OK sz=%d lps=%d\n",
                      cursor, end, end - cursor, prog_ipm_lp_count);
            cursor = end;
            propagate_and_skip_ipm(cursor);
            // Try rounding when most fractional vars are fixed (≥60%).
            // Even with some unfixed vars, rounding + repair LP often succeeds.
            const double fix_ratio = static_cast<double>(cursor) / std::max(1, n_frac_ipm);
            if (cursor >= n_frac_ipm || fix_ratio >= 0.60) {
              if (try_rounding_ipm()) {
                if (opt.verbose)
                  fprintf(stderr, "[PROG-ROUND-IPM] ROOT INCUMBENT obj=%.2f lps=%d fix_ratio=%.2f\n",
                          incumbent_obj, prog_ipm_lp_count, fix_ratio);
              }
            }
          } else {
            unfix_range_ipm(cursor, end);
            if (opt.verbose) {
              log_prog_round_focus_changes(
                  "PROG-ROUND-IPM",
                  "revert-batch[" + std::to_string(cursor) + "," + std::to_string(end) + ")",
                  prog_round_focus_ipm, prop_lb, prop_ub);
            }
            if (batch_sz <= 1) {
              ++cursor;
              while (cursor < n_frac_ipm &&
                     prop_ub[frac_ints_ipm[cursor].idx] - prop_lb[frac_ints_ipm[cursor].idx] < 1e-9)
                ++cursor;
              batch_sz = std::max(1, std::min(n_frac_ipm / 5, n_frac_ipm - cursor));
            } else {
              batch_sz = std::max(1, batch_sz / 2);
            }
          }
        }

        // Residual dive: fix remaining vars individually if budget allows.
        if (need_prog_round && prog_ipm_lp_count < max_prog_ipm_lps && prog_ipm_time_ok()) {
          for (int k = 0; k < n_frac_ipm && need_prog_round && prog_ipm_lp_count < max_prog_ipm_lps; ++k) {
            if (!prog_ipm_time_ok()) break;
            const auto& fi = frac_ints_ipm[k];
            if (solve_lb[fi.idx] == solve_ub[fi.idx]) continue;
            double near = std::round(last_good_x[fi.idx]);
            near = std::min(root.ub[fi.idx], std::max(root.lb[fi.idx], near));
            prop_lb[fi.idx] = near;  prop_ub[fi.idx] = near;
            solve_lb[fi.idx] = near; solve_ub[fi.idx] = near;
            if (opt.verbose) {
              log_prog_round_focus_changes(
                  "PROG-ROUND-IPM",
                  "direct-dive[k=" + std::to_string(k) + "]",
                  prog_round_focus_ipm, prop_lb, prop_ub);
            }
            if (solve_prog_ipm_lp()) {
              propagate_and_skip_ipm(cursor);
              if (try_rounding_ipm()) {
                if (opt.verbose)
                  fprintf(stderr, "[PROG-ROUND-IPM] ROOT INCUMBENT via dive obj=%.2f lps=%d\n",
                          incumbent_obj, prog_ipm_lp_count);
              }
            } else {
              prop_lb[fi.idx] = root.lb[fi.idx];  prop_ub[fi.idx] = root.ub[fi.idx];
              solve_lb[fi.idx] = root.lb[fi.idx]; solve_ub[fi.idx] = root.ub[fi.idx];
              if (opt.verbose) {
                log_prog_round_focus_changes(
                    "PROG-ROUND-IPM",
                    "revert-dive[k=" + std::to_string(k) + "]",
                    prog_round_focus_ipm, prop_lb, prop_ub);
              }
            }
          }
        }

        // Final rounding attempt — try even if <60% fixed.
        if (!has_incumbent && cursor > 0) {
          if (try_rounding_ipm()) {
            if (opt.verbose)
              fprintf(stderr, "[PROG-ROUND-IPM] FINAL ROUND found incumbent obj=%.2f\n",
                      incumbent_obj);
          }
        }

        if (opt.verbose)
          fprintf(stderr, "[PROG-ROUND-IPM] done: lps=%d incumbent=%d fixed=%d/%d\n",
                  prog_ipm_lp_count, has_incumbent ? 1 : 0, cursor, n_frac_ipm);
      }
    }
  }

  [[maybe_unused]] auto t_prog_done = std::chrono::steady_clock::now();

  // LP diving heuristic: follow the relaxation through successive bound
  // fixings and warm re-solves.
  // ════════════════════════════════════════════════════════════════════════
  {  // LP Diving Heuristic scope
    SimplexOptions dive_opt;
    dive_opt.lp_kernel_backend = opt.lp_kernel_backend;
    // A fixed iteration cap bounds the cost of a conflicting dive fixing.
    dive_opt.max_iter = 400;
    dive_opt.allow_cold_start = false;  // Prevents expensive cold start fallback.
    dive_opt.feasibility_tol = std::max(1e-10, opt.lp_tol * 0.1);
    dive_opt.optimality_tol = std::max(1e-10, opt.lp_tol * 0.1);
    dive_opt.verbose = false;

    Eigen::VectorXd dive_lb = root.lb, dive_ub = root.ub;
    Eigen::VectorXd dive_x = root.x_relax;
    auto dive_basis = root.basis_hint;
    bool dive_failed = false;

    // Budget: at most max_dive_lps LP solves or time budget.
    // Skip diving entirely if warm-start already provided a feasible incumbent.
    // Time is measured from dive start so root LP time doesn't eat the budget.
    // Use dual simplex so each accepted dive step can warm-start the next
    // same-structure re-solve from the preceding basis.
    const int m_dive = static_cast<int>(base_lp.A.rows() + base_lp.Aeq.rows());
    const bool large_dive = m_dive > 5000;
    const bool xlarge_dive = m_dive > 10000;
    // Simplex diving is a warm reoptimization heuristic. Without a root
    // basis, every fixing starts primal Phase I from scratch; on large roots
    // these bounded cold attempts cannot produce the first reusable basis.
    const bool skip_basis_free_large_dive = large_dive && !root.basis_hint;
    const bool skip_dive_good_incumbent =
        root_gap_closed() || root_incumbent_good_enough();
    // Low-fractionality rescue: on a large root whose LP relaxation is
    // almost integral and with no incumbent at all, warm bound-fix re-solves
    // can reuse the root basis, and diving may be the only remaining primal
    // source. This exception is still bounded by the normal dive limits.
    const bool dive_low_frac_rescue =
        large_generic_low_frac_root && !has_incumbent;
	    const int max_dive_lps =
	        (skip_dive_good_incumbent || skip_basis_free_large_dive ||
	         bc_optional_root_budget_expired()) ? 0
	        : opt.max_dive_lps <= 0 ? 0
	        : ((xlarge_dive || large_dive)
	               ? (dive_low_frac_rescue ? std::max(200, opt.max_dive_lps) : 0)
	           : std::max(500, opt.max_dive_lps * 5));
    int dive_lp_count = 0;
    int dive_fail_count = 0;  // Track consecutive failures for early abort
	    const double dive_time_budget =
	        (skip_dive_good_incumbent || skip_basis_free_large_dive) ? 0.0
	        : opt.max_dive_lps <= 0 ? 0.0
	        : ((xlarge_dive || large_dive) && dive_low_frac_rescue)
	              ? std::min(opt.time_limit_sec * 0.25, 5.0)
	        : (xlarge_dive ? std::min(opt.time_limit_sec * 0.01, 0.15)
	           : large_dive ? std::min(opt.time_limit_sec * 0.02, 0.5)
	           : std::min(opt.time_limit_sec * 0.40, 15.0));
    trace_root_primal_gate(
        "dive", max_dive_lps > 0 ? "run" : "skip",
	        skip_dive_good_incumbent
	            ? "good_incumbent_or_gap_closed=1"
	            : (skip_basis_free_large_dive ? "basis_free_large_root=1"
	            : (opt.max_dive_lps <= 0 ? "disabled=1"
	            : (dive_low_frac_rescue && (xlarge_dive || large_dive))
	                  ? "low_frac_rescue=1"
	            : (xlarge_dive ? "xlarge=1"
	                           : (large_dive ? "large=1" : "budget=1")))));
    StandardFormLP dive_sf;
    if (max_dive_lps > 0) {
      dive_sf = ensure_base_sf();
    }
    const auto t_dive_start = std::chrono::steady_clock::now();
    auto dive_budget_exceeded = [&]() {
      return bc_optional_root_budget_expired() ||
             dive_lp_count >= max_dive_lps ||
             std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - t_dive_start).count() > dive_time_budget;
    };
    auto apply_dive_solve_budget = [&]() {
      const double local_remaining = dive_time_budget -
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - t_dive_start).count();
      dive_opt.time_limit_sec = std::max(
          0.001,
          std::min(bc_remaining_sec() - bc_post_root_finalize_reserve_sec,
                   local_remaining));
    };

    // Track which variables were fixed during diving (for potential backtrack).
    std::vector<int> dive_fixed_vars;
    std::vector<std::pair<double, double>> dive_saved_bounds;  // (lb, ub) before fix

    // Stall detection: stop dive early if progress plateaus.
    int stall_check_frac = 0;   // fractional count at last stall check
    int stall_check_lp = 0;     // LP count at last stall check
    const int stall_window = 30; // check every N LPs

    for (int dive_step = 0; dive_step < n && !dive_failed; ++dive_step) {
      if (dive_budget_exceeded()) break;

      // Find fractional binary variables.
      // Reduced-cost diving ranks by |cbar_j| times fractionality when available.
      // This guides diving toward variables where fixing has predictable objective impact.
      struct FracVar {
        int j;
        double dist;       // proximity to integrality: min(frac, 1-frac)
        double frac;       // fractional part
        double rc_score;   // |reduced_cost| × fractionality (0 if no RC available)
        double rc;         // raw reduced cost (sign indicates fix direction)
      };
      std::vector<FracVar> frac_vars;
      
      // Check if reduced costs are available from the dive basis.
      const Eigen::VectorXd* rc_ptr = nullptr;
      if (dive_basis && dive_basis->cached_reduced_costs &&
          dive_basis->cached_reduced_costs->size() >= n) {
        rc_ptr = dive_basis->cached_reduced_costs.get();
      }
      
      for (int j = 0; j < n; ++j) {
        if (!is_integer_type(base_lp.vars[j])) continue;
        if (std::abs(dive_ub[j] - dive_lb[j]) < 1e-9) continue;
        const double frac = dive_x[j] - std::floor(dive_x[j]);
        if (frac > opt.int_tol && frac < 1.0 - opt.int_tol) {
          FracVar fv;
          fv.j = j;
          fv.dist = std::min(frac, 1.0 - frac);
          fv.frac = frac;
          if (rc_ptr) {
            fv.rc = (*rc_ptr)[j];
            fv.rc_score = std::abs(fv.rc) * fv.frac;
          } else {
            fv.rc = 0.0;
            fv.rc_score = 0.0;
          }
          frac_vars.push_back(fv);
        }
      }

      if (opt.verbose && (dive_step % 10 == 0 || frac_vars.size() < 10))
        fprintf(stderr, "[DIVE] step=%d frac=%d lps=%d\n", dive_step, (int)frac_vars.size(), dive_lp_count);

      // Stall detection: if fractional count hasn't decreased by 20% in the
      // last stall_window LPs, switch to batch fixing — fix multiple variables
      // per LP to break through the stall.
      bool use_batch_fix = false;
      if (dive_lp_count > 0 && dive_lp_count - stall_check_lp >= stall_window) {
        const int cur_frac = static_cast<int>(frac_vars.size());
        if (stall_check_frac > 0 && cur_frac > stall_check_frac * 4 / 5) {
          // Less than 20% progress in stall_window LPs — stalled.
          use_batch_fix = true;
          if (opt.verbose)
            fprintf(stderr, "[DIVE] stall detected: frac %d→%d in %d LPs, switching to batch\n",
                    stall_check_frac, cur_frac, dive_lp_count - stall_check_lp);
        }
        stall_check_frac = cur_frac;
        stall_check_lp = dive_lp_count;
      }
      if (stall_check_frac == 0 && !frac_vars.empty()) {
        stall_check_frac = static_cast<int>(frac_vars.size());
        stall_check_lp = dive_lp_count;
      }

      if (frac_vars.empty()) {
        // All integer variables appear integral (or fixed via bounds).
        // Verify that dive_x is truly integral for ALL integer variables,
        // not just the unfixed ones (batch-fix updates dive_x directly now).
        bool truly_integral = true;
        for (int i = 0; i < n; ++i) {
          if (!is_integer_type(base_lp.vars[i])) continue;
          double frac = dive_x[i] - std::floor(dive_x[i]);
          if (frac > opt.int_tol && frac < 1.0 - opt.int_tol) {
            truly_integral = false;
            break;
          }
        }
        if (!truly_integral) {
          // Batch-fixed vars have inconsistent dive_x — solve one LP to update.
          if (!dive_budget_exceeded()) {
            update_standard_form_bounds(dive_sf, base_lp, dive_lb, dive_ub);
            apply_dive_solve_budget();
            auto repair_res = solve_lp_from_sf(dive_sf, dive_opt, dive_basis.get());
            ++out.bc_stats.lp_solves;
            ++dive_lp_count;
            if (repair_res.result.stats.success && repair_res.result.x.size() == n) {
              dive_x = clamp_to_bounds(repair_res.result.x, dive_lb, dive_ub);
              auto db = std::make_shared<SimplexBasis>(repair_res.basis);
              db->cached_reduced_costs =
                  std::make_shared<const Eigen::VectorXd>(std::move(repair_res.reduced_costs));
              dive_basis = db;
            }
            // Re-check integrality — the LP relaxation may introduce new
            // fractional values, in which case this dive can't reach integral.
            truly_integral = true;
            for (int i = 0; i < n; ++i) {
              if (!is_integer_type(base_lp.vars[i])) continue;
              double frac = dive_x[i] - std::floor(dive_x[i]);
              if (frac > opt.int_tol && frac < 1.0 - opt.int_tol) {
                truly_integral = false;
                break;
              }
            }
          }
          if (!truly_integral) {
            if (opt.verbose)
              fprintf(stderr, "[DIVE] frac_vars empty but LP not integral, giving up\n");
            dive_failed = true;
            break;
          }
        }
        if (opt.verbose)
          fprintf(stderr, "[DIVE] integral! step=%d lps=%d\n", dive_step, dive_lp_count);
        Eigen::VectorXd xr = dive_x;
        for (int i = 0; i < n; ++i) {
          if (is_integer_type(base_lp.vars[i])) {
            xr[i] = std::round(xr[i]);
            xr[i] = std::min(dive_ub[i], std::max(dive_lb[i], xr[i]));
          }
        }
	        if (satisfies_with_bounds(base_lp, xr, root.lb, root.ub,
	                                  kNodeFeasibilityTol)) {
	          const double obj = objective_value(base_lp.c, xr, base_lp.sense);
	          if (adopt_root_incumbent("dive_integral", xr, obj)) {
	            if (opt.verbose)
	              fprintf(stderr, "[DIVE] incumbent found! obj=%.2f\n", obj);
	          }
        } else if (opt.verbose) {
          // Diagnose why the integral dive solution is infeasible.
          double max_ineq_viol = 0.0;
          int worst_row = -1;
          for (int i = 0; i < pre_cut_root_lp.A.rows(); ++i) {
            double v = pre_cut_root_lp.A.row(i).dot(xr) - pre_cut_root_lp.b[i];
            if (v > max_ineq_viol) { max_ineq_viol = v; worst_row = i; }
          }
          double max_eq_viol = 0.0;
          for (int i = 0; i < pre_cut_root_lp.Aeq.rows(); ++i) {
            double v = std::abs(pre_cut_root_lp.Aeq.row(i).dot(xr) - pre_cut_root_lp.beq[i]);
            if (v > max_eq_viol) max_eq_viol = v;
          }
          // Also check against base_lp (cut-augmented) and using dive_x (unrounded).
          double base_max_viol = 0.0;
          int base_worst = -1;
          for (int i = 0; i < base_lp.A.rows(); ++i) {
            double v = base_lp.A.row(i).dot(xr) - base_lp.b[i];
            if (v > base_max_viol) { base_max_viol = v; base_worst = i; }
          }
          double unrounded_viol = 0.0;
          if (worst_row >= 0) {
            unrounded_viol = pre_cut_root_lp.A.row(worst_row).dot(dive_x) - pre_cut_root_lp.b[worst_row];
          }
          fprintf(stderr, "[DIVE] integral but infeasible: max_ineq=%.2e (row %d) max_eq=%.2e "
                  "base_viol=%.2e (row %d) unrounded_viol=%.2e\n",
                  max_ineq_viol, worst_row, max_eq_viol,
                  base_max_viol, base_worst, unrounded_viol);
          // Find which variable(s) changed most during rounding in worst_row.
          if (worst_row >= 0) {
            Eigen::SparseVector<double> row_vec = pre_cut_root_lp.A.row(worst_row);
            double biggest_change = 0.0;
            int biggest_var = -1;
            double total_int_change = 0.0;
            [[maybe_unused]] double total_cont_diff = 0.0;
            int n_int_in_row = 0;
            for (Eigen::SparseVector<double>::InnerIterator it(row_vec); it; ++it) {
              int col = it.index();
              if (col >= n) continue;
              if (is_integer_type(base_lp.vars[col])) {
                double change = it.value() * (xr[col] - dive_x[col]);
                total_int_change += change;
                ++n_int_in_row;
                double abs_change = std::abs(change);
                if (abs_change > biggest_change) {
                  biggest_change = abs_change;
                  biggest_var = col;
                }
              }
            }
            // Also compute row values separately
            double row_xr = pre_cut_root_lp.A.row(worst_row).dot(xr);
            double row_dx = pre_cut_root_lp.A.row(worst_row).dot(dive_x);
            fprintf(stderr, "[DIVE] row %d: dot(xr)=%.6f dot(dive_x)=%.6f b=%.6f "
                    "total_int_rounding=%.2e n_int=%d\n",
                    worst_row, row_xr, row_dx, pre_cut_root_lp.b[worst_row],
                    total_int_change, n_int_in_row);
            if (biggest_var >= 0) {
              fprintf(stderr, "[DIVE] worst rounding: var=%d type=%d val=%.6f→%.6f coeff=%.1f "
                      "delta_contrib=%.2e dive_lb=%.1f dive_ub=%.1f\n",
                      biggest_var, static_cast<int>(base_lp.vars[biggest_var].type),
                      dive_x[biggest_var], xr[biggest_var],
                      pre_cut_root_lp.A.coeff(worst_row, biggest_var),
                      biggest_change,
                      dive_lb[biggest_var], dive_ub[biggest_var]);
            }
          }
        }
        break;
      }

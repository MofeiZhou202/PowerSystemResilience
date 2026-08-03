// @file bc_run/02_root_relaxation_a.inc
// BCSolveState::run() segment 2/14: root LP relaxation and root cut loop — part 1 (HiGHS side-state, implied bounds).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
	  if (opt.auto_highs_root_pipeline &&
          root_highs_presolved_side_state_first_class &&
	      highs_presolve_side_state.side_state_available &&
	      highs_presolve_side_state.presolved_col_orig.size() ==
	          highs_presolve_side_state.presolved_col_type.size()) {
	    const std::size_t mask_cols =
	        strict_highs_presolved_working_space
	            ? std::min<std::size_t>(
	                  highs_presolve_side_state.presolved_col_type.size(),
	                  static_cast<std::size_t>(std::max(0, n)))
	            : highs_presolve_side_state.presolved_col_orig.size();
	    for (std::size_t k = 0; k < mask_cols; ++k) {
	      int col = strict_highs_presolved_working_space
	                    ? static_cast<int>(k)
	                    : highs_presolve_side_state.presolved_col_orig[k];
	      // The HiGHS side-state is computed from the current root `base_lp`.
	      // Thus postSolveStack.getOrigColIndex() is already in this column
	      // space.  Applying the PaPILO original->reduced map here would map a
	      // reduced column a second time and corrupt the HiGHS integral mask.
	      if (col < 0 || col >= n) continue;
	      const auto type = static_cast<HighsVarType>(
	          highs_presolve_side_state.presolved_col_type[k]);
	      if (type != HighsVarType::kContinuous &&
	          type != HighsVarType::kSemiContinuous) {
	        root_highs_presolved_integral_cols[static_cast<std::size_t>(col)] =
	            1;
	      }
	      if (type == HighsVarType::kImplicitInteger) {
	        root_highs_presolved_implied_integer_cols[
	            static_cast<std::size_t>(col)] = 1;
	      }
	    }
	    root_highs_presolved_integral_mask_valid =
	        std::any_of(root_highs_presolved_integral_cols.begin(),
	                    root_highs_presolved_integral_cols.end(),
	                    [](char v) { return v != 0; });
	    root_highs_presolved_implied_integer_count = static_cast<int>(
	        std::count(root_highs_presolved_implied_integer_cols.begin(),
	                   root_highs_presolved_implied_integer_cols.end(), 1));
	    if (root_highs_presolved_implied_integer_count > 0) {
	      int removed = 0;
	      int added = 0;
	      for (int j = 0; j < n; ++j) {
	        const bool native_impl =
	            root_implied_integer_cols[static_cast<std::size_t>(j)] != 0;
	        const bool highs_impl =
	            root_highs_presolved_implied_integer_cols[
	                static_cast<std::size_t>(j)] != 0;
	        if (native_impl && !highs_impl) ++removed;
	        if (!native_impl && highs_impl) ++added;
	        root_implied_integer_cols[static_cast<std::size_t>(j)] =
	            highs_impl ? 1 : 0;
	      }
	      if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
	          bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	        fmt::print(stderr,
	                   "[B&C-HIGHS-IMPL-MASK] source=presolved_col_type "
	                   "mapped={} native_removed={} native_added={} "
	                   "side_impl={} side_cols={}\n",
	                   root_highs_presolved_implied_integer_count, removed,
	                   added, highs_presolve_side_state.implied_integer_cols,
	                   highs_presolve_side_state.cols);
	      }
	    }
	    if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
	         bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) &&
	        root_highs_presolved_integral_mask_valid) {
	      fmt::print(stderr,
	                 "[B&C-HIGHS-INTEGRAL-MASK] source=presolved_col_type "
	                 "mapped={} side_bin={} side_int={} side_impl={} "
	                 "side_fixed={} side_cols={}\n",
	                 static_cast<int>(std::count(
	                     root_highs_presolved_integral_cols.begin(),
	                     root_highs_presolved_integral_cols.end(), 1)),
	                 highs_presolve_side_state.binary_cols,
	                 highs_presolve_side_state.integer_cols,
	                 highs_presolve_side_state.implied_integer_cols,
	                 highs_presolve_side_state.fixed_cols,
	                 highs_presolve_side_state.cols);
	    }
	  }
#endif
      struct RootHighsPresolvedDomainRows {
        LPModel lp;
        Eigen::SparseMatrix<double, Eigen::RowMajor> A_row;
        Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row;
        bool valid{false};
        int mapped_rows{0};
        int dropped_rows{0};
        int finite_lower_rows{0};
        int finite_upper_rows{0};
      };
      auto build_root_highs_presolved_domain_rows =
          [&]() -> RootHighsPresolvedDomainRows {
        RootHighsPresolvedDomainRows out_rows;
        if (!root_highs_presolved_space_exact ||
            !highs_presolve_side_state.side_state_available ||
            highs_presolve_side_state.rows <= 0 ||
            highs_presolve_side_state.cols <= 0) {
          return out_rows;
        }
        const int hrows = highs_presolve_side_state.rows;
        const int hcols = highs_presolve_side_state.cols;
        const bool direct_presolved_space =
            strict_highs_presolved_working_space;
        if (static_cast<int>(highs_presolve_side_state.presolved_row_lower.size()) != hrows ||
            static_cast<int>(highs_presolve_side_state.presolved_row_upper.size()) != hrows ||
            static_cast<int>(highs_presolve_side_state.presolved_a_start.size()) != hrows + 1) {
          return out_rows;
        }
        if (!direct_presolved_space &&
            (static_cast<int>(
                 highs_presolve_side_state.presolved_col_orig.size()) != hcols ||
             static_cast<int>(
                 highs_presolve_side_state.presolved_col_scale.size()) != hcols ||
             static_cast<int>(
                 highs_presolve_side_state.presolved_col_constant.size()) != hcols ||
             static_cast<int>(
                 highs_presolve_side_state
                     .presolved_col_linearly_transformable.size()) != hcols)) {
          return out_rows;
        }
        const int matrix_end =
            highs_presolve_side_state.presolved_a_start[static_cast<std::size_t>(hrows)];
        if (matrix_end < 0 ||
            static_cast<int>(highs_presolve_side_state.presolved_a_value.size()) < matrix_end ||
            static_cast<int>(highs_presolve_side_state.presolved_a_index.size()) < matrix_end) {
          return out_rows;
        }
        out_rows.lp.sense = base_lp.sense;
        out_rows.lp.vars = base_lp.vars;
        out_rows.lp.c = Eigen::VectorXd::Zero(n);
        for (int j = 0; j < n &&
                        j < static_cast<int>(root_implied_integer_cols.size()); ++j) {
          if (root_implied_integer_cols[static_cast<std::size_t>(j)] != 0 &&
              out_rows.lp.vars[static_cast<std::size_t>(j)].type ==
                  VarType::Continuous) {
            out_rows.lp.vars[static_cast<std::size_t>(j)].type =
                VarType::Integer;
          }
        }

        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(static_cast<std::size_t>(
            std::max(0, highs_presolve_side_state.nnz)));
        std::vector<double> lower;
        std::vector<double> upper;
        lower.reserve(static_cast<std::size_t>(hrows));
        upper.reserve(static_cast<std::size_t>(hrows));
        auto highs_finite_lower = [](double value) {
          return std::isfinite(value) && value > -1e29;
        };
        auto highs_finite_upper = [](double value) {
          return std::isfinite(value) && value < 1e29;
        };
        for (int r = 0; r < hrows; ++r) {
          const int start =
              highs_presolve_side_state.presolved_a_start[static_cast<std::size_t>(r)];
          const int end =
              highs_presolve_side_state.presolved_a_start[static_cast<std::size_t>(r + 1)];
          if (start < 0 || end < start || end > matrix_end) {
            ++out_rows.dropped_rows;
            continue;
          }
          std::vector<std::pair<int, double>> row_terms;
          row_terms.reserve(static_cast<std::size_t>(end - start));
          double row_shift = 0.0;
          bool ok = true;
          for (int p = start; p < end; ++p) {
            const int hcol =
                highs_presolve_side_state.presolved_a_index[static_cast<std::size_t>(p)];
            const double a =
                highs_presolve_side_state.presolved_a_value[static_cast<std::size_t>(p)];
            if (hcol < 0 || hcol >= hcols || !std::isfinite(a)) {
              ok = false;
              break;
            }
            if (direct_presolved_space) {
              if (std::abs(a) > 1e-15) {
                row_terms.emplace_back(hcol, a);
              }
              continue;
            }
            if (
                highs_presolve_side_state
                        .presolved_col_linearly_transformable[static_cast<std::size_t>(hcol)] == 0) {
              ok = false;
              break;
            }
            const int native_col =
                highs_presolve_side_state.presolved_col_orig[static_cast<std::size_t>(hcol)];
            const double scale =
                highs_presolve_side_state.presolved_col_scale[static_cast<std::size_t>(hcol)];
            const double constant =
                highs_presolve_side_state.presolved_col_constant[static_cast<std::size_t>(hcol)];
            if (native_col < 0 || native_col >= n || !std::isfinite(scale) ||
                !std::isfinite(constant) || std::abs(scale) <= 1e-12) {
              ok = false;
              break;
            }
            const double mapped_a = a / scale;
            row_shift += mapped_a * constant;
            if (std::abs(mapped_a) > 1e-15) {
              row_terms.emplace_back(native_col, mapped_a);
            }
          }
          if (!ok) {
            ++out_rows.dropped_rows;
            continue;
          }
          std::sort(row_terms.begin(), row_terms.end(),
                    [](const auto& lhs, const auto& rhs) {
                      return lhs.first < rhs.first;
                    });
          std::vector<std::pair<int, double>> compact_terms;
          compact_terms.reserve(row_terms.size());
          for (const auto& term : row_terms) {
            if (!compact_terms.empty() &&
                compact_terms.back().first == term.first) {
              compact_terms.back().second += term.second;
            } else {
              compact_terms.push_back(term);
            }
          }
          compact_terms.erase(
              std::remove_if(compact_terms.begin(), compact_terms.end(),
                             [](const auto& term) {
                               return std::abs(term.second) <= 1e-15;
                             }),
              compact_terms.end());
          const double row_lower =
              highs_presolve_side_state.presolved_row_lower[static_cast<std::size_t>(r)];
          const double row_upper =
              highs_presolve_side_state.presolved_row_upper[static_cast<std::size_t>(r)];
          const bool has_lower = highs_finite_lower(row_lower);
          const bool has_upper = highs_finite_upper(row_upper);
          if (compact_terms.empty() || (!has_lower && !has_upper)) {
            ++out_rows.dropped_rows;
            continue;
          }
          const int mapped_row = static_cast<int>(lower.size());
          for (const auto& term : compact_terms) {
            trips.emplace_back(mapped_row, term.first, term.second);
          }
          lower.push_back(has_lower ? row_lower + row_shift
                                    : -std::numeric_limits<double>::infinity());
          upper.push_back(has_upper ? row_upper + row_shift
                                    : std::numeric_limits<double>::infinity());
          out_rows.finite_lower_rows += has_lower ? 1 : 0;
          out_rows.finite_upper_rows += has_upper ? 1 : 0;
        }
        out_rows.mapped_rows = static_cast<int>(lower.size());
        if (out_rows.mapped_rows <= 0) return out_rows;
        out_rows.lp.row_lhs = Eigen::VectorXd::Zero(out_rows.mapped_rows);
        out_rows.lp.b = Eigen::VectorXd::Zero(out_rows.mapped_rows);
        for (int r = 0; r < out_rows.mapped_rows; ++r) {
          out_rows.lp.row_lhs[r] = lower[static_cast<std::size_t>(r)];
          out_rows.lp.b[r] = upper[static_cast<std::size_t>(r)];
        }
        out_rows.lp.A.resize(out_rows.mapped_rows, n);
        out_rows.lp.A.setFromTriplets(trips.begin(), trips.end());
        out_rows.lp.A.makeCompressed();
        out_rows.lp.Aeq.resize(0, n);
        out_rows.lp.beq = Eigen::VectorXd::Zero(0);
        out_rows.A_row = out_rows.lp.A;
        out_rows.Aeq_row = out_rows.lp.Aeq;
        out_rows.valid = true;
        return out_rows;
      };
      RootHighsPresolvedDomainRows root_highs_presolved_domain_rows =
          build_root_highs_presolved_domain_rows();
      auto root_highs_integral_col = [&](int col) -> bool {
        if (col < 0 || col >= n) return false;
        if (strict_highs_root_fixed_point &&
                root_highs_presolved_space_exact &&
            root_highs_presolved_integral_mask_valid) {
	          return root_highs_presolved_integral_cols[
	                     static_cast<std::size_t>(col)] != 0;
	        }
        return is_integer_type(root_lp.vars[static_cast<std::size_t>(col)]) ||
               (col < static_cast<int>(root_implied_integer_cols.size()) &&
                root_implied_integer_cols[static_cast<std::size_t>(col)] != 0);
      };
      auto root_highs_bound_range =
          [&](int col, double upper_bound, double lower_bound) -> double {
        const double range = upper_bound - lower_bound;
        const bool continuous = !root_highs_integral_col(col);
        if (!std::isfinite(range)) {
          return continuous ? std::numeric_limits<double>::quiet_NaN()
                            : range;
        }
        return range -
               (continuous
                    ? std::max(0.3 * range,
                               1000.0 * highs_mip_feastol)
                    : highs_mip_feastol);
      };
      auto root_highs_capacity_threshold_with_term =
          [&](double threshold, int col, double coeff,
              double upper_bound, double lower_bound) -> double {
        const double range =
            root_highs_bound_range(col, upper_bound, lower_bound);
        const double term = std::abs(coeff) * range;
        if (std::isnan(term)) {
          return std::max(threshold, highs_mip_feastol);
        }
        return std::max({threshold, term, highs_mip_feastol});
      };
      auto root_highs_adjusted_ub =
          [&](int col, double bound_val, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub, double& adjusted) -> bool {
        if (col < 0 || col >= n || !std::isfinite(bound_val)) return false;
        if (root_highs_integral_col(col)) {
          adjusted = std::floor(bound_val + highs_mip_feastol);
          return adjusted < ub[col] &&
                 ub[col] - adjusted >
                     1000.0 * highs_mip_feastol * std::abs(adjusted);
        }
        adjusted = std::abs(bound_val - lb[col]) <= kHighsDefaultSmallMatrixValue
                       ? lb[col]
                       : bound_val;
        if (!std::isfinite(ub[col])) return std::isfinite(adjusted);
        if (!(adjusted + 1000.0 * highs_mip_feastol < ub[col])) return false;
        double relative_improve = ub[col] - adjusted;
        if (std::isfinite(lb[col])) {
          const double width = ub[col] - lb[col];
          if (!(width > 0.0)) return false;
          relative_improve /= width;
        } else {
          const double denom = std::max(std::abs(ub[col]), std::abs(adjusted));
          if (!(denom > 0.0)) return false;
          relative_improve /= denom;
        }
        return relative_improve >= 0.3;
      };
      auto root_highs_adjusted_lb =
          [&](int col, double bound_val, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub, double& adjusted) -> bool {
        if (col < 0 || col >= n || !std::isfinite(bound_val)) return false;
        if (root_highs_integral_col(col)) {
          adjusted = std::ceil(bound_val - highs_mip_feastol);
          return adjusted > lb[col] &&
                 adjusted - lb[col] >
                     1000.0 * highs_mip_feastol * std::abs(adjusted);
        }
        adjusted = std::abs(ub[col] - bound_val) <= kHighsDefaultSmallMatrixValue
                       ? ub[col]
                       : bound_val;
        if (!std::isfinite(lb[col])) return std::isfinite(adjusted);
        if (!(adjusted - 1000.0 * highs_mip_feastol > lb[col])) return false;
        double relative_improve = adjusted - lb[col];
        if (std::isfinite(ub[col])) {
          const double width = ub[col] - lb[col];
          if (!(width > 0.0)) return false;
          relative_improve /= width;
        } else {
          const double denom = std::max(std::abs(lb[col]), std::abs(adjusted));
          if (!(denom > 0.0)) return false;
          relative_improve /= denom;
        }
        return relative_improve >= 0.3;
      };
      auto apply_root_highs_presolved_col_bounds =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub) -> int {
        if (!root_highs_presolved_space_exact ||
            !highs_presolve_side_state.side_state_available ||
            static_cast<int>(highs_presolve_side_state.presolved_col_lower.size()) !=
                highs_presolve_side_state.cols ||
            static_cast<int>(highs_presolve_side_state.presolved_col_upper.size()) !=
                highs_presolve_side_state.cols) {
          return 0;
        }
        const bool direct_presolved_space =
            strict_highs_presolved_working_space;
        if (!direct_presolved_space &&
            (static_cast<int>(
                 highs_presolve_side_state.presolved_col_orig.size()) !=
                 highs_presolve_side_state.cols ||
             static_cast<int>(
                 highs_presolve_side_state.presolved_col_scale.size()) !=
                 highs_presolve_side_state.cols ||
             static_cast<int>(
                 highs_presolve_side_state.presolved_col_constant.size()) !=
                 highs_presolve_side_state.cols ||
             static_cast<int>(
                 highs_presolve_side_state
                     .presolved_col_linearly_transformable.size()) !=
                 highs_presolve_side_state.cols)) {
          return 0;
        }
        int tightened = 0;
        const double tol = std::max(1e-9, highs_mip_feastol);
        auto highs_finite_lower = [](double value) {
          return std::isfinite(value) && value > -1e29;
        };
        auto highs_finite_upper = [](double value) {
          return std::isfinite(value) && value < 1e29;
        };
        for (int hcol = 0; hcol < highs_presolve_side_state.cols; ++hcol) {
          if (!direct_presolved_space &&
              highs_presolve_side_state
                  .presolved_col_linearly_transformable[static_cast<std::size_t>(hcol)] == 0) {
            continue;
          }
          const int native_col =
              direct_presolved_space
                  ? hcol
                  : highs_presolve_side_state
                        .presolved_col_orig[static_cast<std::size_t>(hcol)];
          if (native_col < 0 || native_col >= n) continue;
          const double scale =
              direct_presolved_space
                  ? 1.0
                  : highs_presolve_side_state
                        .presolved_col_scale[static_cast<std::size_t>(hcol)];
          const double constant =
              direct_presolved_space
                  ? 0.0
                  : highs_presolve_side_state
                        .presolved_col_constant[static_cast<std::size_t>(hcol)];
          const double plb =
              highs_presolve_side_state.presolved_col_lower[static_cast<std::size_t>(hcol)];
          const double pub =
              highs_presolve_side_state.presolved_col_upper[static_cast<std::size_t>(hcol)];
          if (!std::isfinite(scale) || !std::isfinite(constant) ||
              std::abs(scale) <= 1e-12) {
            continue;
          }
          if (highs_finite_lower(plb)) {
            const double mapped = scale * plb + constant;
            if (scale > 0.0) {
              if (mapped > lb[native_col] + tol) {
                lb[native_col] = mapped;
                ++tightened;
              }
            } else if (mapped < ub[native_col] - tol) {
              ub[native_col] = mapped;
              ++tightened;
            }
          }
          if (highs_finite_upper(pub)) {
            const double mapped = scale * pub + constant;
            if (scale > 0.0) {
              if (mapped < ub[native_col] - tol) {
                ub[native_col] = mapped;
                ++tightened;
              }
            } else if (mapped > lb[native_col] + tol) {
              lb[native_col] = mapped;
              ++tightened;
            }
          }
        }
        return tightened;
      };
      struct RootHighsDomainPropagationQueue {
        std::vector<std::vector<int>> col_to_rows;
        std::vector<unsigned char> pending;
        std::vector<int> queue;
        bool initialized{false};
      };
      RootHighsDomainPropagationQueue root_highs_domain_queue;
      auto root_highs_domain_row_propagatable =
          [&](int row, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> bool {
        if (!root_highs_presolved_domain_rows.valid) return false;
        const auto& domain_lp = root_highs_presolved_domain_rows.lp;
        const auto& A_row = root_highs_presolved_domain_rows.A_row;
        if (row < 0 || row >= static_cast<int>(A_row.rows())) return false;
        const auto check_upper_side = [&](double row_sign, double rhs) {
          if (!std::isfinite(rhs)) return false;
          double min_activity = 0.0;
          int ninf_min = 0;
          double capacity_threshold = highs_mip_feastol;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                   A_row, row);
               it; ++it) {
            const int j = static_cast<int>(it.col());
            const double a = row_sign * it.value();
            if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
            const double min_bound = a > 0.0 ? lb[j] : ub[j];
            const double max_bound = a > 0.0 ? ub[j] : lb[j];
            if (!std::isfinite(min_bound)) {
              ++ninf_min;
            } else {
              min_activity += a * min_bound;
            }
            if (min_bound != max_bound) {
              capacity_threshold =
                  root_highs_capacity_threshold_with_term(
                      capacity_threshold, j, a, ub[j], lb[j]);
            }
          }
          return ninf_min == 1 ||
                 (ninf_min == 0 &&
                  rhs - min_activity <= capacity_threshold);
        };
        bool propagate = false;
        if (row < domain_lp.b.size()) {
          propagate = propagate ||
                      check_upper_side(1.0, domain_lp.b[row]);
        }
        const double lhs = lp_row_lhs_or_neg_inf(domain_lp, row);
        if (std::isfinite(lhs)) {
          propagate = propagate || check_upper_side(-1.0, -lhs);
        }
        return propagate;
      };
      auto mark_root_highs_domain_row_pending =
          [&](int row, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> void {
        if (!root_highs_presolved_domain_rows.valid) return;
        if (row < 0 ||
            row >= static_cast<int>(root_highs_domain_queue.pending.size())) {
          return;
        }
        if (root_highs_domain_queue.pending[static_cast<std::size_t>(row)] !=
            0) {
          return;
        }
        if (!root_highs_domain_row_propagatable(row, lb, ub)) return;
        root_highs_domain_queue.pending[static_cast<std::size_t>(row)] = 1;
        root_highs_domain_queue.queue.push_back(row);
      };
      auto initialize_root_highs_domain_queue =
          [&](const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) -> void {
        (void)lb;
        (void)ub;
        if (root_highs_domain_queue.initialized ||
            !root_highs_presolved_domain_rows.valid) {
          return;
        }
        const auto& A_row = root_highs_presolved_domain_rows.A_row;
        const int m = static_cast<int>(A_row.rows());
        root_highs_domain_queue.col_to_rows.assign(
            static_cast<std::size_t>(n), {});
        root_highs_domain_queue.pending.assign(static_cast<std::size_t>(m), 0);
        root_highs_domain_queue.queue.clear();
        for (int r = 0; r < m; ++r) {
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                   A_row, r);
               it; ++it) {
            const int j = static_cast<int>(it.col());
            if (j >= 0 && j < n && std::abs(it.value()) > 1e-12) {
              root_highs_domain_queue.col_to_rows[static_cast<std::size_t>(j)]
                  .push_back(r);
            }
          }
        }
        // HiGHS seeds all model rows once in HighsDomain::computeRowActivities()
        // during MIP setup, then consumes that queue before root separation.
        // A separation-round propagate() is changed-column/event driven: rows
        // become pending only through changeBound() updates (or through cutpool
        // cutAdded/lpCutRemoved events).  Build the incidence index here, but
        // do not enqueue every row again at root.
        root_highs_domain_queue.initialized = true;
      };
      auto mark_root_highs_domain_rows_for_changed_cols =
          [&](const std::vector<int>& cols, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> void {
        initialize_root_highs_domain_queue(lb, ub);
        if (!root_highs_domain_queue.initialized) return;
        for (int col : cols) {
          if (col < 0 ||
              col >= static_cast<int>(root_highs_domain_queue.col_to_rows.size())) {
            continue;
          }
          for (int row :
               root_highs_domain_queue.col_to_rows[static_cast<std::size_t>(
                   col)]) {
            mark_root_highs_domain_row_pending(row, lb, ub);
          }
        }
      };
      auto propagate_root_highs_presolved_domain_rows =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub, int max_rounds) -> int {
        if (!root_highs_presolved_domain_rows.valid) return 0;
        const auto& domain_lp = root_highs_presolved_domain_rows.lp;
        const auto& A_row = root_highs_presolved_domain_rows.A_row;
        const int m = static_cast<int>(A_row.rows());
        const int rounds = std::max(1, max_rounds);
        int total_tightened = 0;
        for (int round = 0; round < rounds; ++round) {
          int round_tightened = 0;
          for (int r = 0; r < m; ++r) {
            auto propagate_upper_side = [&](double row_sign, double rhs) {
              if (!std::isfinite(rhs)) return;
              double min_activity = 0.0;
              int ninf_min = 0;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double bound = a > 0.0 ? lb[j] : ub[j];
                if (!std::isfinite(bound)) {
                  ++ninf_min;
                } else {
                  min_activity += a * bound;
                }
              }
              if (!std::isfinite(min_activity) || ninf_min > 1) return;
              if (ninf_min == 0 && min_activity > rhs + highs_mip_feastol) {
                if (lb.size() > 0) lb[0] = ub[0] + 1.0;
                return;
              }
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double min_bound = a > 0.0 ? lb[j] : ub[j];
                double residual_activity = min_activity;
                if (ninf_min == 1) {
                  if (std::isfinite(min_bound)) continue;
                } else {
                  if (!std::isfinite(min_bound)) continue;
                  residual_activity -= a * min_bound;
                }
                const double residual = rhs - residual_activity;
                double adjusted = 0.0;
                if (a > 0.0) {
                  if (root_highs_adjusted_ub(j, residual / a, lb, ub,
                                             adjusted)) {
                    ub[j] = adjusted;
                    ++round_tightened;
                  }
                } else if (root_highs_adjusted_lb(j, residual / a, lb, ub,
                                                  adjusted)) {
                  lb[j] = adjusted;
                  ++round_tightened;
                }
              }
            };
            if (r < domain_lp.b.size()) propagate_upper_side(1.0, domain_lp.b[r]);
            const double lhs = lp_row_lhs_or_neg_inf(domain_lp, r);
            if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
            if (!bounds_consistent(lb, ub)) return total_tightened + round_tightened;
          }
          total_tightened += round_tightened;
          if (round_tightened == 0) break;
        }
        return total_tightened;
      };
      auto propagate_root_highs_presolved_domain_pending =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub) -> int {
        if (!root_highs_presolved_domain_rows.valid) return 0;
        initialize_root_highs_domain_queue(lb, ub);
        const auto& domain_lp = root_highs_presolved_domain_rows.lp;
        const auto& A_row = root_highs_presolved_domain_rows.A_row;
        int total_tightened = 0;
        while (!root_highs_domain_queue.queue.empty()) {
          std::vector<int> rows;
          rows.swap(root_highs_domain_queue.queue);
          for (int r : rows) {
            if (r < 0 ||
                r >= static_cast<int>(root_highs_domain_queue.pending.size())) {
              continue;
            }
            root_highs_domain_queue.pending[static_cast<std::size_t>(r)] = 0;
            auto propagate_upper_side = [&](double row_sign, double rhs) {
              if (!std::isfinite(rhs)) return;
              double min_activity = 0.0;
              int ninf_min = 0;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double bound = a > 0.0 ? lb[j] : ub[j];
                if (!std::isfinite(bound)) {
                  ++ninf_min;
                } else {
                  min_activity += a * bound;
                }
              }
              if (!std::isfinite(min_activity) || ninf_min > 1) return;
              if (ninf_min == 0 && min_activity > rhs + highs_mip_feastol) {
                if (lb.size() > 0) lb[0] = ub[0] + 1.0;
                return;
              }
              const Eigen::VectorXd before_lb = lb;
              const Eigen::VectorXd before_ub = ub;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double min_bound = a > 0.0 ? before_lb[j] : before_ub[j];
                double residual_activity = min_activity;
                if (ninf_min == 1) {
                  if (std::isfinite(min_bound)) continue;
                } else {
                  if (!std::isfinite(min_bound)) continue;
                  residual_activity -= a * min_bound;
                }
                const double residual = rhs - residual_activity;
                double adjusted = 0.0;
                if (a > 0.0) {
                  if (root_highs_adjusted_ub(j, residual / a, lb, ub,
                                             adjusted)) {
                    const Eigen::VectorXd col_old_lb = lb;
                    const Eigen::VectorXd col_old_ub = ub;
                    ub[j] = adjusted;
                    ++total_tightened;
                    const ChangedColSides changed =
                        collect_changed_col_sides_with_tol(
                            col_old_lb, col_old_ub, lb, ub,
                            std::max(1e-9, opt.int_tol));
                    mark_root_highs_domain_rows_for_changed_cols(changed.any,
                                                                 lb, ub);
                  }
                } else if (root_highs_adjusted_lb(j, residual / a, lb, ub,
                                                  adjusted)) {
                  const Eigen::VectorXd col_old_lb = lb;
                  const Eigen::VectorXd col_old_ub = ub;
                  lb[j] = adjusted;
                  ++total_tightened;
                  const ChangedColSides changed =
                      collect_changed_col_sides_with_tol(
                          col_old_lb, col_old_ub, lb, ub,
                          std::max(1e-9, opt.int_tol));
                  mark_root_highs_domain_rows_for_changed_cols(changed.any,
                                                               lb, ub);
                }
                if (!bounds_consistent(lb, ub)) return;
              }
            };
            if (r < domain_lp.b.size()) {
              propagate_upper_side(1.0, domain_lp.b[r]);
            }
            const double lhs = lp_row_lhs_or_neg_inf(domain_lp, r);
            if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
            if (!bounds_consistent(lb, ub)) return total_tightened;
          }
        }
        return total_tightened;
      };
      auto propagate_root_highs_presolved_domain_from_changed_cols =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub,
              const ChangedColSides& initial_changed) -> int {
        if (!root_highs_presolved_domain_rows.valid ||
            initial_changed.any.empty()) {
          return 0;
        }
        initialize_root_highs_domain_queue(lb, ub);
        const auto& domain_lp = root_highs_presolved_domain_rows.lp;
        const auto& A_row = root_highs_presolved_domain_rows.A_row;
        std::vector<unsigned char> local_pending(
            root_highs_domain_queue.pending.size(), 0);
        std::vector<int> local_queue;
        auto mark_local_row = [&](int row) {
          if (row < 0 || row >= static_cast<int>(local_pending.size())) {
            return;
          }
          if (local_pending[static_cast<std::size_t>(row)] != 0) return;
          if (!root_highs_domain_row_propagatable(row, lb, ub)) return;
          local_pending[static_cast<std::size_t>(row)] = 1;
          local_queue.push_back(row);
        };
        auto mark_local_cols = [&](const std::vector<int>& cols) {
          for (int col : cols) {
            if (col < 0 ||
                col >= static_cast<int>(
                           root_highs_domain_queue.col_to_rows.size())) {
              continue;
            }
            for (int row :
                 root_highs_domain_queue.col_to_rows[static_cast<std::size_t>(
                     col)]) {
              mark_local_row(row);
            }
          }
        };
        mark_local_cols(initial_changed.any);

        int total_tightened = 0;
        while (!local_queue.empty()) {
          std::vector<int> rows;
          rows.swap(local_queue);
          for (int r : rows) {
            if (r < 0 || r >= static_cast<int>(local_pending.size())) {
              continue;
            }
            local_pending[static_cast<std::size_t>(r)] = 0;
            auto propagate_upper_side = [&](double row_sign, double rhs) {
              if (!std::isfinite(rhs)) return;
              double min_activity = 0.0;
              int ninf_min = 0;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double bound = a > 0.0 ? lb[j] : ub[j];
                if (!std::isfinite(bound)) {
                  ++ninf_min;
                } else {
                  min_activity += a * bound;
                }
              }
              if (!std::isfinite(min_activity) || ninf_min > 1) return;
              if (ninf_min == 0 && min_activity > rhs + highs_mip_feastol) {
                if (lb.size() > 0) lb[0] = ub[0] + 1.0;
                return;
              }
              const Eigen::VectorXd before_row_lb = lb;
              const Eigen::VectorXd before_row_ub = ub;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
                       A_row, r);
                   it; ++it) {
                const int j = static_cast<int>(it.col());
                const double a = row_sign * it.value();
                if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
                const double min_bound =
                    a > 0.0 ? before_row_lb[j] : before_row_ub[j];
                double residual_activity = min_activity;
                if (ninf_min == 1) {
                  if (std::isfinite(min_bound)) continue;
                } else {
                  if (!std::isfinite(min_bound)) continue;
                  residual_activity -= a * min_bound;
                }
                const double residual = rhs - residual_activity;
                double adjusted = 0.0;
                if (a > 0.0) {
                  if (root_highs_adjusted_ub(j, residual / a, lb, ub,
                                             adjusted)) {
                    const Eigen::VectorXd col_old_lb = lb;
                    const Eigen::VectorXd col_old_ub = ub;
                    ub[j] = adjusted;
                    ++total_tightened;
                    const ChangedColSides changed =
                        collect_changed_col_sides_with_tol(
                            col_old_lb, col_old_ub, lb, ub,
                            std::max(1e-9, opt.int_tol));
                    mark_local_cols(changed.any);
                  }
                } else if (root_highs_adjusted_lb(j, residual / a, lb, ub,
                                                  adjusted)) {
                  const Eigen::VectorXd col_old_lb = lb;
                  const Eigen::VectorXd col_old_ub = ub;
                  lb[j] = adjusted;
                  ++total_tightened;
                  const ChangedColSides changed =
                      collect_changed_col_sides_with_tol(
                          col_old_lb, col_old_ub, lb, ub,
                          std::max(1e-9, opt.int_tol));
                  mark_local_cols(changed.any);
                }
                if (!bounds_consistent(lb, ub)) return;
              }
            };
            if (r < domain_lp.b.size()) {
              propagate_upper_side(1.0, domain_lp.b[r]);
            }
            const double lhs = lp_row_lhs_or_neg_inf(domain_lp, r);
            if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
            if (!bounds_consistent(lb, ub)) return total_tightened;
          }
        }
        return total_tightened;
      };
      auto propagate_root_highs_presolved_domain =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub, int max_rounds) -> int {
        if (!root_highs_presolved_space_exact) return 0;
        const Eigen::VectorXd before_col_lb = lb;
        const Eigen::VectorXd before_col_ub = ub;
        int tightened = apply_root_highs_presolved_col_bounds(lb, ub);
        if (!bounds_consistent(lb, ub)) return tightened;
        if (strict_highs_root_fixed_point) {
          const ChangedColSides changed = collect_changed_col_sides_with_tol(
              before_col_lb, before_col_ub, lb, ub,
              std::max(1e-9, opt.int_tol));
          tightened +=
              propagate_root_highs_presolved_domain_from_changed_cols(
                  lb, ub, changed);
          return tightened;
        }
        tightened += propagate_root_highs_presolved_domain_rows(lb, ub,
                                                                max_rounds);
        return tightened;
      };
      if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
           bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) &&
          strict_highs_root_fixed_point) {
        fmt::print(stderr,
                   "[B&C-HIGHS-DOMAIN] mappedRows={} droppedRows={} "
                   "finiteLower={} finiteUpper={} nnz={} cols={} valid={} "
                   "spaceExact={}\n",
                   root_highs_presolved_domain_rows.mapped_rows,
                   root_highs_presolved_domain_rows.dropped_rows,
                   root_highs_presolved_domain_rows.finite_lower_rows,
                   root_highs_presolved_domain_rows.finite_upper_rows,
                   static_cast<int>(
                       root_highs_presolved_domain_rows.lp.A.nonZeros()),
                   highs_presolve_side_state.cols,
                   root_highs_presolved_domain_rows.valid ? 1 : 0,
                   root_highs_presolved_space_exact ? 1 : 0);
      }
	  RootRedcostLurkingBounds root_redcost_lurking_bounds;
	  root_redcost_lurking_bounds.ensure_size(n);
	  int root_implied_integer_count = static_cast<int>(
	      std::count(root_implied_integer_cols.begin(),
                 root_implied_integer_cols.end(), 1));
  const IntegralRowTighteningStats root_integral_row_tightening =
      tighten_integral_row_sides(root_lp, root_implied_integer_cols,
                                 highs_mip_feastol);
  const int root_implied_integer_cut_support_count =
      opt.enable_implied_integer_cut_strengthening ? root_implied_integer_count : 0;
  if (opt.verbose && root_implied_integer_count > 0) {
    fmt::print(stderr,
               "[B&C] root implied-integer artifact: {} continuous columns"
               "{}; integral rows={} row_side_tighten={}/{} "
               "max_delta={:.3g}/{:.3g}\n",
               root_implied_integer_count,
               root_implied_integer_cut_support_count > 0
                   ? " (enabled for root cut support)"
                   : " (audit/propagation only)",
               root_integral_row_tightening.integral_rows,
               root_integral_row_tightening.lower_tightened,
               root_integral_row_tightening.upper_tightened,
               root_integral_row_tightening.max_lower_delta,
               root_integral_row_tightening.max_upper_delta);
  }

  BCOptions root_solve_opt = opt;
  if (requested_vendored_highs_lp && !allow_vendored_root_frontier) {
    root_solve_opt.suppress_vendored_highs_root_frontier = true;
  }
  if (reduced_warmstart_available && orig_n >= 90000) {
    root_solve_opt.xlarge_ipm_only_threshold = std::numeric_limits<int>::max();
  }
  if (use_huge_warmstart_root_shortcut && !root_solve_opt.use_ipm_root) {
    root_solve_opt.use_ipm_root = true;
  }
  if (bc_time_limit_expired()) {
    publish_verified_warm_start_time_limit();
    return return_with_profile();
  }
  const double root_remaining_before_reserve = bc_remaining_sec();
  root_solve_opt.time_limit_sec =
      std::max(0.001, root_remaining_before_reserve);
  // When the unpresolved-root fallback is available,
  // cap the presolved root LP so a hard root cannot burn the whole limit
  // before the retry gets a chance (the presolved 118-bus root otherwise
  // consumes the entire budget in the simplex escalation chain and the
  // fallback has nothing left to run with).
  const bool short_budget_ipm_root =
      root_solve_opt.use_ipm_root && opt.time_limit_sec > 0.0 &&
      opt.time_limit_sec <= 10.0;
  const bool short_budget_root =
      opt.time_limit_sec > 0.0 && opt.time_limit_sec <= 10.0;
  const bool root_fallback_available =
      papilo_model_reduced && opt.root_presolve_fallback;
  root_solve_opt.time_limit_sec = bc_root_lp_budget_sec(
      root_remaining_before_reserve, opt.time_limit_sec,
      opt.root_presolve_fallback_min_remaining_sec,
      opt.root_presolve_fallback_budget_fraction,
      root_fallback_available, short_budget_ipm_root);
  if (opt.verbose && root_fallback_available) {
    fmt::print(stderr,
               "[B&C-ROOT-BUDGET] remaining={:.6f}s root={:.6f}s "
               "fallback_min={:.6f}s fraction={:.3f} ipm_short={}\n",
               root_remaining_before_reserve, root_solve_opt.time_limit_sec,
               opt.root_presolve_fallback_min_remaining_sec,
               opt.root_presolve_fallback_budget_fraction,
               short_budget_ipm_root ? 1 : 0);
  }

  // Background analytic-centre thread. It owns a full LPModel copy so its
  // zero-objective interior solve cannot race with root processing. The
  // resulting interior point is used by line-search rounding.
  //
  // This is independent of the root LP algorithm.  HiGHS computes the analytic
  // centre in parallel with a simplex root and later consumes it through
  // central rounding, so native must not gate it on use_ipm_root.
  // Disabled automatically for trivially small problems (n < 100).
  std::optional<Eigen::VectorXd> analytic_centre;
  std::unique_ptr<std::thread> ac_thread_ptr;
  {
    const double remaining_before_root = bc_remaining_sec();
    const bool spawn_analytic_centre =
        opt.use_analytic_centre && !short_budget_root && n >= 100 &&
        remaining_before_root > 2.0;
    if (spawn_analytic_centre) {
      LPModel ac_lp_copy = root_lp;  // copy with node bounds
      ac_lp_copy.c.setZero();        // zero cost → interior-point solution
      // Make vars continuous so the IPM doesn't refuse to solve it.
      for (auto& v : ac_lp_copy.vars) {
        if (v.type == VarType::Binary || v.type == VarType::Integer) {
          v.type = VarType::Continuous;
        }
      }
      const int ac_iter = std::max(
          1, std::min(opt.analytic_centre_max_iter,
                      static_cast<int>(remaining_before_root * 20.0)));
      const int ac_n = n;
      ac_thread_ptr = std::make_unique<std::thread>(
          [ac_lp = std::move(ac_lp_copy), &analytic_centre, ac_iter, ac_n,
           remaining_before_root, solve_environment]() mutable {
            ScopedBcEnvOptions environment_scope(solve_environment);
            IPMLPOptions ac_opt;
            ac_opt.max_iter   = ac_iter;
            ac_opt.time_limit_sec = std::max(0.001, 0.5 * remaining_before_root);
            ac_opt.tol_primal = 1e-5;
            ac_opt.tol_dual   = 1e-5;
            ac_opt.tol_gap    = 1e-5;
            ac_opt.crossover  = false;  // keep interior; no vertex purification
            ac_opt.verbose    = false;
            NativeIPMLPAdapter ac_ipm(ac_opt);
            auto res = ac_ipm.solve_lp(ac_lp);
            if (res.stats.success && res.x.size() == ac_n) {
              analytic_centre = std::move(res.x);
            }
          });
    }
  }
  auto join_root_background_tasks = [&]() {
    if (ac_thread_ptr && ac_thread_ptr->joinable()) {
      [[maybe_unused]] auto t_ac_join0 = std::chrono::steady_clock::now();
      ac_thread_ptr->join();
      [[maybe_unused]] auto t_ac_join1 = std::chrono::steady_clock::now();
      if (opt.verbose) {
        fmt::print(stderr, "[B&C-ANALYTIC-CENTRE] {} join_wait={:.1f}ms\n",
                   analytic_centre.has_value() ? "ok" : "failed",
                   std::chrono::duration<double, std::milli>(
                       t_ac_join1 - t_ac_join0)
                       .count());
      }
    }
  };

  [[maybe_unused]] auto t_root0 = std::chrono::steady_clock::now();
  const Eigen::VectorXd* root_lp_seed = reduced_warmstart_available ? &root.x_seed : nullptr;
  LPRelaxationResult root_relax = solve_lp_relaxation(root_lp, root_lp_seed, nullptr, root_solve_opt);
  [[maybe_unused]] auto t_root1 = std::chrono::steady_clock::now();
  if (opt.verbose) {
    MIPSOLVERS_LOG_DEBUG("[B&C] Root LP: {:.1f}ms (m={}, n={})",
                      std::chrono::duration<double, std::milli>(t_root1 - t_root0).count(),
                      static_cast<int>(root_lp.A.rows()) + static_cast<int>(root_lp.Aeq.rows()),
                      static_cast<int>(root_lp.vars.size()));
  }
  out.bc_stats.lp_solves += 1;
  // Test seam: deterministically exercise the unpresolved-root fallback.
  // Fails only the presolved attempt (the retry runs with presolve off), so
  // the hook is reusable across test cases without hidden static state.
  if (opt.use_papilo_presolve &&
      bc_env_flag_enabled("MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL")) {
    root_relax.primal.stats.success = false;
    root_relax.primal.stats.status = "Forced root failure (test seam)";
  }
  if (!root_relax.primal.stats.success || root_relax.primal.x.size() != n) {
    out.stats.status = root_relax.primal.stats.status.starts_with("Time limit")
                           ? bc_status::kTimeLimitReached
                           : bc_status::kRootRelaxationFailed;
    // Keep bc_stats.status in sync: callers reading BCResult::bc_stats.status
    // must not see an empty string on the root-relaxation failure path.
    out.bc_stats.status = out.stats.status;
    // Retry once without PaPILO presolve instead of entering the repair path
    // with no usable root relaxation. A root-stage time limit
    // also qualifies: with the budget reserve above it means the presolved
    // root exhausted its capped share, not the whole budget (a genuinely
    // expired limit leaves <= min_remaining and is filtered by the guard).
    const bool test_forced_fallback =
        opt.use_papilo_presolve &&
        bc_env_flag_enabled("MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL");
    if ((out.stats.status == bc_status::kRootRelaxationFailed ||
         out.stats.status == bc_status::kTimeLimitReached) &&
        opt.root_presolve_fallback &&
        (papilo_model_reduced || test_forced_fallback) &&
        bc_remaining_sec() > opt.root_presolve_fallback_min_remaining_sec) {
      // ac_thread captures references to locals of this frame — join before
      // anything else.
      join_root_background_tasks();
      if (opt.verbose) {
        fmt::print(stderr,
                   "[B&C] root relaxation failed on the presolved model — "
                   "retrying without PaPILO presolve ({:.1f}s remaining)\n",
                   bc_remaining_sec());
      }
      BCOptions retry_opt = opt;
      retry_opt.use_papilo_presolve = false;
      // Inner call has presolve off, so the fallback condition cannot
      // re-fire: recursion depth is bounded at 2 by construction.
      if (opt.time_limit_sec > 0.0) {
        retry_opt.time_limit_sec = std::max(0.001, bc_remaining_sec());
      }
      BCResult retry =
          branch_and_cut_lp(prob, retry_opt, callbacks);
      // The retry result is already original-space (no presolve), so no
      // postsolve mapping is needed; merge the outer attempt's counters.
      retry.bc_stats.lp_solves += out.bc_stats.lp_solves;
      retry.bc_stats.presolve_fallback_attempts += 1;
      retry.stats.runtime_sec = bc_elapsed_sec();
      retry.bc_stats.runtime_sec = retry.stats.runtime_sec;
      // Balance this frame's propagation-profile scope (the retry synced its
      // own stats at depth 2; the thread-local depth still needs our
      // decrement, and `out` is abandoned).
      sync_propagation_profile();
      --propagation_profile_scope_depth;
      return retry;
    }
    if (out.stats.status == bc_status::kTimeLimitReached) {
      // A timed-out root relaxation can still carry a kernel-certified dual
      // bound (see LPRelaxationResult::dual_bound on the simplex time-limit
      // path).  Feed it to the publisher so the result reports a real bound
      // and the incumbent quality gate compares against something honest.
      if (std::isfinite(root_relax.dual_bound)) {
        best_known_root_bound = root_relax.dual_bound;
      }
      publish_verified_warm_start_time_limit();
    }
    join_root_background_tasks();
    return return_with_profile();
	  }

  // A bounded IPM crossover may return a certified interior solution without
  // a simplex basis. Make that solver-mode transition explicit so later root
  // and node LPs never interpret the missing basis as a cold-start request.
  if (root_relax.requires_ipm_nodes) {
    opt.use_simplex_lp_nodes = false;
    opt.use_ipm_nodes = true;
    root_solve_opt.use_simplex_lp_nodes = false;
    root_solve_opt.use_ipm_nodes = true;
    if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C] root crossover unavailable; switched downstream LPs "
                 "to IPM\n");
    }
  }

	  root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	  const Eigen::VectorXd certified_root_primal_x = root.x_relax;
	  Eigen::VectorXd root_lp_source_x = root_relax.primal.x;
	  auto refresh_root_lp_source_solution = [&]() {
	    // The basis handle may own the scaled canonical standard-form model;
	    // its first n columns are not an original-space primal vector. The LP
	    // kernel publication audit already reconstructs root_relax.primal.x,
	    // which is the only valid source for branching and heuristics.
	    if (root_relax.primal.stats.success &&
	        root_relax.primal.x.size() == n) {
	      root_lp_source_x = root_relax.primal.x;
	    } else {
	      root_lp_source_x = root.x_relax;
	    }
	  };
		  auto root_source_solution = [&]() -> const Eigen::VectorXd& {
		    return root_lp_source_x.size() == n ? root_lp_source_x : root.x_relax;
		  };
		  refresh_root_lp_source_solution();
      struct RootHighsFractionalFrontier {
        std::vector<int> cols;
        std::vector<char> mask;
        int declared{0};
        int implied{0};
        int binary{0};
      };
      auto compute_root_highs_fractional_frontier =
          [&]() -> RootHighsFractionalFrontier {
        RootHighsFractionalFrontier frontier;
        frontier.mask.assign(static_cast<std::size_t>(std::max(0, n)), 0);
        const Eigen::VectorXd& source_x = root_source_solution();
        if (source_x.size() != n || root.lb.size() != n ||
            root.ub.size() != n) {
          return frontier;
        }
        frontier.cols.reserve(64);
        for (int j = 0; j < n; ++j) {
          const bool declared_integer =
              is_integer_type(root_lp.vars[static_cast<std::size_t>(j)]);
          const bool implied_integer =
              j < static_cast<int>(root_implied_integer_cols.size()) &&
              root_implied_integer_cols[static_cast<std::size_t>(j)] != 0;
          if (!declared_integer && !implied_integer) continue;

          const double lbj = root.lb[j];
          const double ubj = root.ub[j];
          const double raw_xj = source_x[j];
          if (!std::isfinite(raw_xj) || std::isnan(lbj) || std::isnan(ubj) ||
              ubj <= lbj) {
            continue;
          }

          const double xj = std::min(ubj, std::max(lbj, raw_xj));
          if (std::abs(xj - std::round(xj)) <= highs_mip_feastol) continue;

          frontier.mask[static_cast<std::size_t>(j)] = 1;
          frontier.cols.push_back(j);
          if (declared_integer) ++frontier.declared;
          if (implied_integer) ++frontier.implied;
          if (std::abs(lbj) <= highs_mip_feastol &&
              std::abs(ubj - 1.0) <= highs_mip_feastol) {
            ++frontier.binary;
          }
        }
        if (strict_highs_root_fixed_point && frontier.cols.size() > 1) {
          HighsHashTable<HighsInt> highs_fractional_order(0);
          for (int col : frontier.cols) {
            highs_fractional_order.insert(static_cast<HighsInt>(col));
          }
          frontier.cols.clear();
          frontier.cols.reserve(highs_fractional_order.size());
          for (const auto& entry : highs_fractional_order) {
            frontier.cols.push_back(static_cast<int>(entry.key()));
          }
        }
        return frontier;
      };
		  if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
	    rebind_sparse_basis_matrix(root_relax.simplex->basis.cached_sparse_basis,
	                               root_relax.simplex->form.A);
    if (root_relax.basis_hint) {
      root_relax.basis_hint->cached_sparse_basis =
          root_relax.simplex->basis.cached_sparse_basis;
    }
  }
	  root.basis_hint = root_relax.basis_hint;
	  if (root.basis_hint) {
	    root.basis_hint->compact_indices_storage();
	  }
  if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
    root.has_live_factor_telemetry = true;
    root.live_factor_telemetry =
        get_sparse_basis_factor_telemetry(root_relax.simplex->basis.cached_sparse_basis);
  }
  root.bound = root_relax.dual_bound;
  if (!std::isfinite(root.bound)) {
    root.bound = objective_value(base_lp.c, root.x_relax, base_lp.sense);
  }
	  if (bc_env_flag_enabled("MIPSOLVERS_BC_TIMELINE")) {
	    fmt::print(stderr, "[B&C-ROOT-PRIMAL] bound={:.12g} x=[", root.bound);
	    for (int j = 0; j < std::min(n, 8); ++j) {
	      fmt::print(stderr, "{}{}", j == 0 ? "" : ",", root.x_relax[j]);
	    }
	    fmt::print(stderr, "]\n");
	  }
	  if (!std::isfinite(root.bound)) {
	    out.stats.status = bc_status::kRootRelaxationNanObjective;
	    join_root_background_tasks();
	    return return_with_profile();
	  }
	  best_known_root_bound = root.bound;
	  root.depth = 0;
	  if (bc_optional_root_budget_expired()) {
	    publish_verified_warm_start_time_limit();
	    join_root_background_tasks();
	    return return_with_profile();
	  }
	  const bool highs_root_cutpool_original_lp =
	      strict_highs_root_fixed_point;
	      // HiGHS' HighsDomain::propagate() commits objective-cutoff and current-LP
	      // reduced-cost bound changes as first-class domain changes, then
	      // immediately re-solves the LP when the domain moved.  The root red-cost
      // lurking table is a first-class root LP source: it is built only from the
      // audited root LP basis/dual state and activated by the HiGHS upper_limit
      // (not the raw incumbent objective).
      const bool root_cutoff_domain_commit_allowed = true;
      const bool root_current_redcost_domain_commit_allowed = true;
      const bool root_lurking_redcost_domain_commit_allowed = true;
      bool root_redcost_lurking_cutoff_available = false;
      auto root_redcost_lurking_recording_allowed =
          [&]() -> bool {
        if (!root_lurking_redcost_domain_commit_allowed ||
            !opt.enable_reduced_cost_fixing) {
          return false;
        }
        // HiGHS records root reduced-cost state inside the root domain
        // fixed point, but actual reduced-cost bound changes are activated by
        // a finite upper_limit.  The strict root pipeline must therefore not
        // inflate a multi-threshold lurking table before a verified cutoff is
        // available; that turns a lightweight LP state into an extra Native
        // proof-source graph and dominates root separation time.
        if (strict_highs_root_fixed_point &&
            !root_redcost_lurking_cutoff_available) {
          return false;
        }
        return true;
      };
		  auto refresh_root_redcost_lurking_bounds =
		      [&](const char* phase) -> RootRedcostLurkingBounds::AddStats {
		    RootRedcostLurkingBounds::AddStats stats;
			    if (!root_redcost_lurking_recording_allowed() ||
            base_lp.sense != Sense::Minimize ||
		        !root.basis_hint || !root.basis_hint->cached_reduced_costs ||
		        root.basis_hint->cached_reduced_costs->size() < n ||
		        root.x_relax.size() != n || root.lb.size() != n ||
	        root.ub.size() != n || !std::isfinite(root.bound)) {
	      stats.records_total = root_redcost_lurking_bounds.size();
	      return stats;
	    }
	    stats = root_redcost_lurking_bounds.add_from_root_lp(
	        base_lp.vars, &root_implied_integer_cols, root.x_relax,
	        *root.basis_hint->cached_reduced_costs,
	        root.basis_hint->cached_col_scale.get(),
	        root.basis_hint->basis_indices(),
	        root.lb, root.ub, root.bound, opt.int_tol);
      if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C ROOT-RC-LURK] phase={} cols={} added={} total={} "
	                 "sideReject={} lpobj={:.10g}\n",
	                 phase != nullptr ? phase : "root_lp", stats.columns_seen,
	                 stats.records_added, stats.records_total,
	                 stats.side_rejected, root.bound);
	    }
	    return stats;
	  };
	  refresh_root_redcost_lurking_bounds("root_lp_initial");
	  const StandardFormLP* root_trace_sf =
	      root_relax.simplex ? &root_relax.simplex->form : nullptr;
  trace_native_frontier_conformance(
      "root_lp_initial", root_lp, root_trace_sf, root_implied_integer_cols,
      root.x_relax, root.lb, root.ub, root.basis_hint.get(), root.bound,
      highs_mip_feastol);
	  StandardFormLP root_first_class_unscaled_sf;
	  bool root_first_class_sf_valid = false;
	  const bool maintain_native_xpool_sf_cache =
	      !highs_root_cutpool_original_lp;
	  if (maintain_native_xpool_sf_cache && root_relax.simplex &&
	      root_relax.simplex->form.A.rows() > 0) {
	    root_first_class_unscaled_sf = build_standard_form_lp(root_lp);
	    root_first_class_sf_valid = true;
	  }

  // ─────────────────────────────────────────────────────────────────────
  // Root pseudocost seeding (cold-start priors).
  //
  // Before any probing, every integer variable has `down_cnt=up_cnt=0` and
  // `down_avg()=up_avg()=1.0`, so `pseudocost_branch_score = frac*(1-frac)*1*1`
  // collapses all cold candidates to near-identical scores.  This makes the
  // branching decision degenerate into "first fractional index wins", which
  // is sensitive to tiny numerical differences in the LP relaxation across
  // LP factor backends.
  //
  // Seed each integer variable with a single synthetic observation whose
  // magnitude equals `|c[j]|` (the objective coefficient magnitude — a
  // backend-independent, data-driven prior).  Keeps `down_cnt=up_cnt=1` so
  // the variable is still considered "unreliable" by `probe_reliability=2`
  // and gets refined by in-tree probing, but now cold scores differ in a
  // way that reflects the true cost structure.
  // ─────────────────────────────────────────────────────────────────────
  {
    std::vector<double> abs_c;
    abs_c.reserve(declared_branchable_indices.size());
    for (int i : declared_branchable_indices) {
      const double a = std::abs(base_lp.c[i]);
      if (a > 0.0) abs_c.push_back(a);
    }
    double cost_median = 1.0;
    if (!abs_c.empty()) {
      std::nth_element(abs_c.begin(), abs_c.begin() + abs_c.size() / 2, abs_c.end());
      cost_median = std::max(1e-6, abs_c[abs_c.size() / 2]);
    }
    for (int i : declared_branchable_indices) {
      const double cj = std::abs(base_lp.c[i]);
      const double prior = std::max(1e-6, 0.5 * cost_median + cj);
      pc[i].add_down(prior);
      pc[i].add_up(prior);
    }
  }

  // Repeated native solves with unchanged original columns can import learned
  // directional pseudocosts
  // through the PaPILO original-to-reduced map instead of relearning them.
  if (opt.highs_pseudocost_warm_start &&
      !opt.highs_pseudocost_warm_start->empty()) {
    const auto& seed = *opt.highs_pseudocost_warm_start;
    const auto full_size = [orig_n](const auto& values) {
      return static_cast<int>(values.size()) == orig_n;
    };
    if (seed.n_orig_cols == orig_n && full_size(seed.pseudocostup) &&
        full_size(seed.pseudocostdown) && full_size(seed.nsamplesup) &&
        full_size(seed.nsamplesdown)) {
      for (int orig_col = 0; orig_col < orig_n; ++orig_col) {
        const int col = map_original_col_to_reduced(orig_col);
        if (col < 0 || col >= n || !is_integer_type(base_lp.vars[col])) continue;
        PseudoCost& p = pc[col];
        p.up_cnt = std::max(0, seed.nsamplesup[static_cast<size_t>(orig_col)]);
        p.down_cnt =
            std::max(0, seed.nsamplesdown[static_cast<size_t>(orig_col)]);
        const double up = seed.pseudocostup[static_cast<size_t>(orig_col)];
        const double down = seed.pseudocostdown[static_cast<size_t>(orig_col)];
        p.up_sum = std::max(0.0, up) * p.up_cnt;
        p.down_sum = std::max(0.0, down) * p.down_cnt;
        p.up_sq = std::max(0.0, up * up) * p.up_cnt;
        p.down_sq = std::max(0.0, down * down) * p.down_cnt;
        if (full_size(seed.inferencesup) && full_size(seed.ninferencesup)) {
          p.up_inference_cnt =
              std::max(0, seed.ninferencesup[static_cast<size_t>(orig_col)]);
          p.up_inference_sum =
              std::max(0.0, seed.inferencesup[static_cast<size_t>(orig_col)]) *
              p.up_inference_cnt;
        }
        if (full_size(seed.inferencesdown) && full_size(seed.ninferencesdown)) {
          p.down_inference_cnt =
              std::max(0, seed.ninferencesdown[static_cast<size_t>(orig_col)]);
          p.down_inference_sum =
              std::max(0.0, seed.inferencesdown[static_cast<size_t>(orig_col)]) *
              p.down_inference_cnt;
        }
        if (full_size(seed.conflictscoreup))
          p.up_conflict_score =
              std::max(0.0, seed.conflictscoreup[static_cast<size_t>(orig_col)]);
        if (full_size(seed.conflictscoredown))
          p.down_conflict_score = std::max(
              0.0, seed.conflictscoredown[static_cast<size_t>(orig_col)]);
      }
    }
  }

  root.estimate = compute_node_estimate(declared_branchable_indices, root.x_relax, pc,
                                        opt.int_tol, root.bound,
                                        opt.node_estimate_aggregation);

  // Count fractional binaries at root
  if (opt.verbose) {
    int n_frac = 0, n_bin = 0;
    for (int i = 0; i < n; ++i) {
      if (base_lp.vars[i].type == VarType::Binary) {
        ++n_bin;
        double v = root.x_relax[i];
        if (v > 1e-6 && v < 1.0 - 1e-6) ++n_frac;
      }
    }
    fmt::print(stderr, "[B&C DIAG] root_lp_obj={:.2f}  n_frac_binary={}/{}\n",
               root.bound, n_frac, n_bin);
  }

  ObjectivePropagationState objective_propagation;
  const bool use_objective_propagation =
      opt.enable_objective_cutoff_conflict_cuts ||
      opt.enable_objective_cutoff_weighted_event_cuts ||
      opt.enable_objective_cutoff_domain_fixing ||
      (opt.enable_reduced_cost_fixing &&
       opt.enable_reduced_cost_proof_conflict_minimization);
  auto objective_propagation_build_policy =
      [&]() -> ObjectivePropagationState::BuildPolicy {
    return strict_highs_lp_contract
        ? ObjectivePropagationState::BuildPolicy::highs_domain_closure()
        : ObjectivePropagationState::BuildPolicy::native_enriched();
  };
	  if (use_objective_propagation) {
	    objective_propagation.setup(root_lp, &root_implied_integer_cols,
	                                nullptr, nullptr,
	                                objective_propagation_build_policy());
	  }

		  ConflictPool conflict_pool(
		      opt.cut_pool_max_size,
		      std::max(64, opt.reduced_cost_conflict_pool_max_literals),
		      opt.enable_reduced_cost_proof_conflict_minimization);
		  auto conflict_clause_hits_witness =
		      [&](const std::vector<BranchDomainLiteral>& clause) {
		        if (reduced_initial_solution.size() != n || clause.empty()) {
		          return false;
		        }
		        const double witness_tol =
		            std::max(1e-9, 10.0 * std::max(opt.int_tol, opt.lp_tol));
		        for (const auto& literal : clause) {
		          if (literal.var_idx < 0 || literal.var_idx >= n) return false;
		          const double value = reduced_initial_solution[literal.var_idx];
		          const bool active = literal.is_lb
		              ? value >= literal.value - witness_tol
		              : value <= literal.value + witness_tol;
		          if (!active) return false;
		        }
		        return true;
		      };
		  auto audit_conflict_publish_against_witness =
		      [&](const std::vector<BranchDomainLiteral>& clause,
		          const char* source) {
		        if (!opt.verbose || !conflict_clause_hits_witness(clause)) return;
		        fmt::print(stderr,
		                   "[B&C-WITNESS-INVALID-CONFLICT-PUBLISH] source={} "
		                   "literals={}\n",
		                   source != nullptr ? source : "unknown", clause.size());
		        for (std::size_t i = 0; i < clause.size(); ++i) {
		          const auto& literal = clause[i];
		          fmt::print(stderr,
		                     "[B&C-WITNESS-PUBLISHED-CLAUSE] index={} var={} "
		                     "side={} value={:.17g} witness_value={:.17g}\n",
		                     i, literal.var_idx,
		                     literal.is_lb ? "lb" : "ub", literal.value,
		                     reduced_initial_solution[literal.var_idx]);
		        }
		      };
		  auto add_global_conflict =
		      [&](std::vector<BranchDomainLiteral> clause,
		          const char* source) {
		        audit_conflict_publish_against_witness(clause, source);
		        return conflict_pool.add(std::move(clause));
		      };
		  std::uint64_t global_domain_learning_epoch = 0;
		  std::uint64_t objective_artifact_epoch = 0;
		  std::uint64_t incumbent_cutoff_epoch = 0;
		  auto current_domain_learning_epoch = [&]() -> std::uint64_t {
		    return global_domain_learning_epoch;
		  };
		  auto current_objective_artifact_epoch = [&]() -> std::uint64_t {
		    return objective_artifact_epoch;
		  };
		  auto current_domain_closure_epoch = [&]() -> std::uint64_t {
		    return global_domain_learning_epoch + objective_artifact_epoch +
		           incumbent_cutoff_epoch;
		  };
		  auto note_global_domain_learning =
		      [&](std::uint64_t added = 1) -> void {
		    if (added > 0) {
		      global_domain_learning_epoch += added;
		    }
		  };
		  auto note_objective_artifact_learning =
		      [&](std::uint64_t added = 1) -> void {
		    if (added > 0) {
		      objective_artifact_epoch += added;
		    }
		  };
		  auto note_incumbent_cutoff_learning = [&]() -> void {
		    ++incumbent_cutoff_epoch;
		  };

	  // When crossover was skipped (no simplex basis), Gomory cuts can't fire.
	  // Switch to All so MIR/cover still run.
  if (!root_relax.simplex && opt.cuts == CutType::Gomory) {
    opt.cuts = CutType::All;
  }

  auto t_binv0 = std::chrono::steady_clock::now();
  auto t_binv1 = t_binv0;
  std::shared_ptr<BasisOps> root_sbasis;
  if (root_relax.simplex && opt.cuts != CutType::None && opt.cuts != CutType::Cover) {
    if (root_relax.simplex->basis.cached_sparse_basis) {
      root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
    }
  }
  t_binv1 = std::chrono::steady_clock::now();

  // Build the persistent clique table once from the post-presolve root LP.
  // This amortises conflict-graph construction across every cut round and
  // also powers probing-phase propagation (x_i=1 -> x_j=0 cascade).
  CliqueTable clique_table;
  BinaryImplicationGraph implication_graph(n);
	  if (native_model_reduced) {
	    for (const auto& implication : native_ps.probing_implications()) {
	      if (implication_graph.add_implication(
	              implication.trigger_col, implication.trigger_value_one,
	              implication.implied_col, implication.implied_is_lb,
	              implication.implied_value)) {
	        ++out.bc_stats.native_presolve_probing_implications_imported;
	      }
	    }
	  }
	  VariableBoundTable variable_bound_table(n);
	  NativeVariableBoundSourceStats variable_bound_source_stats;
	  std::vector<Eigen::SparseVector<double>> variable_bound_cutpool_rows;
	  std::vector<double> variable_bound_cutpool_rhs;
      std::unordered_set<std::string> variable_bound_cutpool_row_keys;
      std::vector<Eigen::SparseVector<double>>
          root_transformed_cutpool_audit_rows;
      std::vector<double> root_transformed_cutpool_audit_rhs;
      std::unordered_set<std::string> root_transformed_cutpool_audit_row_keys;
	      struct SparseCutpoolPropagationStats {
        int pending_rows{0};
	        int rows_scanned{0};
	        int active_rows{0};
	        int tightened{0};
        int pruned{0};
        double max_capacity{0.0};
        double min_capacity{kInf};
      };
      std::vector<PoolCut> root_transformed_cutpool;
      std::vector<unsigned char> root_transformed_cut_in_lp;
      std::vector<unsigned char> root_transformed_cut_alive;
      std::vector<unsigned char> root_transformed_cut_propagate;
      std::vector<unsigned char> root_transformed_cut_prop_pending;
      std::vector<int> root_transformed_cut_prop_queue;
      std::vector<std::vector<int>> root_transformed_cut_col_to_cuts(
          static_cast<std::size_t>(n));
      std::vector<int> root_transformed_cut_age;
      struct RootTransformedLpCutRow {
        int lp_row{-1};
        int cut_index{-1};
        int age{0};
        bool alive{false};
      };
      std::vector<RootTransformedLpCutRow> root_transformed_lp_cut_rows;
      std::uint64_t root_transformed_lp_cut_age_epochs = 0;
      std::unordered_multimap<std::uint64_t, int>
          root_transformed_cutpool_hash_to_cut;
      const int root_transformed_cutpool_age_limit =
          opt.auto_highs_root_pipeline ? kHighsDefaultMipPoolAgeLimit
                                       : std::max(0, opt.cut_pool_max_age);
      const int root_transformed_cutpool_soft_limit =
          opt.auto_highs_root_pipeline ? kHighsDefaultMipPoolSoftLimit
                                       : std::max(1, opt.cut_pool_max_size);
      const int root_transformed_cutpool_initial_age =
          std::max(0, root_transformed_cutpool_age_limit - 5);
      double root_transformed_cutpool_best_observed_score = 0.0;
      double root_transformed_cutpool_min_score_factor = 0.9;
      int root_transformed_cutpool_pending_source_rows = 0;
      auto root_transformed_cut_prop_is_live = [&](int cut_index) -> bool {
        if (cut_index < 0 ||
            cut_index >= static_cast<int>(root_transformed_cutpool.size())) {
          return false;
        }
        if (cut_index < static_cast<int>(root_transformed_cut_alive.size()) &&
            root_transformed_cut_alive[static_cast<std::size_t>(cut_index)] ==
                0) {
          return false;
        }
        if (cut_index >=
                static_cast<int>(root_transformed_cut_propagate.size()) ||
            root_transformed_cut_propagate[static_cast<std::size_t>(cut_index)] ==
                0) {
          return false;
        }
        const PoolCut& cut =
            root_transformed_cutpool[static_cast<std::size_t>(cut_index)];
        return cut.coeff.size() == n && cut.coeff.nonZeros() > 0 &&
               std::isfinite(cut.rhs);
      };
      auto root_transformed_cut_capacity_threshold =
          [&](int cut_index, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> double {
        if (!root_transformed_cut_prop_is_live(cut_index)) {
          return -highs_mip_feastol;
        }
        const PoolCut& cut =
            root_transformed_cutpool[static_cast<std::size_t>(cut_index)];
        double threshold = -highs_mip_feastol;
        for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it;
             ++it) {
          const int j = static_cast<int>(it.index());
          const double a = it.value();
          if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
          if (lb[j] == ub[j]) continue;
          threshold = root_highs_capacity_threshold_with_term(
              threshold, j, a, ub[j], lb[j]);
        }
        return threshold;
      };
      auto root_transformed_cut_should_propagate =
          [&](int cut_index, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> bool {
        if (!root_transformed_cut_prop_is_live(cut_index)) return false;
        const PoolCut& cut =
            root_transformed_cutpool[static_cast<std::size_t>(cut_index)];
        double min_activity = 0.0;
        int ninf_min = 0;
        for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it;
             ++it) {
          const int j = static_cast<int>(it.index());
          const double a = it.value();
          if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
          const double bound = a > 0.0 ? lb[j] : ub[j];
          if (!std::isfinite(bound)) {
            ++ninf_min;
          } else {
            min_activity += a * bound;
          }
        }
        if (!std::isfinite(min_activity)) return false;
        return ninf_min == 1 ||
               (ninf_min == 0 &&
                cut.rhs - min_activity <=
                    root_transformed_cut_capacity_threshold(cut_index, lb, ub));
      };
      auto mark_root_transformed_cut_prop_pending =
          [&](int cut_index, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> void {
        if (!root_transformed_cut_prop_is_live(cut_index)) return;
        if (cut_index >=
            static_cast<int>(root_transformed_cut_prop_pending.size())) {
          return;
        }
        if (root_transformed_cut_prop_pending[static_cast<std::size_t>(
                cut_index)] != 0) {
          return;
        }
        if (!root_transformed_cut_should_propagate(cut_index, lb, ub)) {
          return;
        }
        root_transformed_cut_prop_pending[static_cast<std::size_t>(
            cut_index)] = 1;
        root_transformed_cut_prop_queue.push_back(cut_index);
      };
      auto mark_root_transformed_cut_prop_for_changed_sides =
          [&](const ChangedColSides& changed, const Eigen::VectorXd& lb,
              const Eigen::VectorXd& ub) -> void {
        auto mark_col = [&](int col, bool lower_side) {
          if (col < 0 ||
              col >=
                  static_cast<int>(root_transformed_cut_col_to_cuts.size())) {
            return;
          }
          for (int cut_index :
               root_transformed_cut_col_to_cuts[static_cast<std::size_t>(
                   col)]) {
            if (cut_index < 0 ||
                cut_index >= static_cast<int>(root_transformed_cutpool.size())) {
              continue;
            }
            const PoolCut& cut =
                root_transformed_cutpool[static_cast<std::size_t>(cut_index)];
            const double coeff = cut.coeff.coeff(col);
            if ((lower_side && coeff <= 1e-12) ||
                (!lower_side && coeff >= -1e-12)) {
              continue;
            }
            mark_root_transformed_cut_prop_pending(cut_index, lb, ub);
          }
        };
        for (int col : changed.lower) mark_col(col, true);
        for (int col : changed.upper) mark_col(col, false);
      };
      [[maybe_unused]] auto collect_root_transformed_cutpool_propagation_rows =
          [&](std::vector<Eigen::SparseVector<double>>& rows,
              std::vector<double>& rhs) -> int {
        rows.clear();
        rhs.clear();
        if (strict_highs_root_fixed_point) {
          std::vector<int> pending;
          pending.swap(root_transformed_cut_prop_queue);
          rows.reserve(pending.size());
          rhs.reserve(pending.size());
          for (int i : pending) {
            if (i >= 0 &&
                i < static_cast<int>(
                        root_transformed_cut_prop_pending.size())) {
              root_transformed_cut_prop_pending[static_cast<std::size_t>(i)] =
                  0;
            }
            if (!root_transformed_cut_prop_is_live(i)) continue;
            const PoolCut& cut =
                root_transformed_cutpool[static_cast<std::size_t>(i)];
            rows.push_back(cut.coeff);
            rhs.push_back(cut.rhs);
          }
          return static_cast<int>(rows.size());
        }
        rows.reserve(root_transformed_cutpool.size());
        rhs.reserve(root_transformed_cutpool.size());
        for (int i = 0; i < static_cast<int>(root_transformed_cutpool.size());
             ++i) {
          if (!root_transformed_cut_prop_is_live(i)) continue;
          if (i < static_cast<int>(root_transformed_cut_in_lp.size()) &&
              root_transformed_cut_in_lp[static_cast<std::size_t>(i)] != 0) {
            continue;
          }
          const PoolCut& cut =
              root_transformed_cutpool[static_cast<std::size_t>(i)];
          rows.push_back(cut.coeff);
          rhs.push_back(cut.rhs);
        }
        return static_cast<int>(rows.size());
      };
      auto propagate_root_transformed_cutpool_pending =
          [&](Eigen::VectorXd& lb, Eigen::VectorXd& ub,
              SparseCutpoolPropagationStats* stats,
              std::vector<DomainReasonBound>* reason_bounds_out = nullptr,
              int reason_depth = 0,
              int reason_trail_offset = 0) -> bool {
	        if (!strict_highs_root_fixed_point ||
	            root_transformed_cut_prop_queue.empty()) {
	          return true;
	        }
        if (stats != nullptr) {
          stats->pending_rows +=
              static_cast<int>(root_transformed_cut_prop_queue.size());
        }
	        while (!root_transformed_cut_prop_queue.empty()) {
	          std::vector<int> pending;
	          pending.swap(root_transformed_cut_prop_queue);
          for (int cut_index : pending) {
            if (cut_index >= 0 &&
                cut_index <
                    static_cast<int>(
                        root_transformed_cut_prop_pending.size())) {
              root_transformed_cut_prop_pending[static_cast<std::size_t>(
                  cut_index)] = 0;
            }
            if (!root_transformed_cut_prop_is_live(cut_index)) continue;
            const PoolCut& cut =
                root_transformed_cutpool[static_cast<std::size_t>(cut_index)];
            if (stats != nullptr) ++stats->rows_scanned;
            double min_activity = 0.0;
            int ninf_min = 0;
            for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it;
                 ++it) {
              const int j = static_cast<int>(it.index());
              const double a = it.value();
              if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
              const double bound = a > 0.0 ? lb[j] : ub[j];
              if (!std::isfinite(bound)) {
                ++ninf_min;
              } else {
                min_activity += a * bound;
              }
            }
            if (!std::isfinite(min_activity) || ninf_min > 1) continue;
            const double capacity = cut.rhs - min_activity;
            if (stats != nullptr) {
              stats->max_capacity = std::max(stats->max_capacity, capacity);
              stats->min_capacity = std::min(stats->min_capacity, capacity);
            }
            if (ninf_min == 0 && capacity < -highs_mip_feastol) {
              if (stats != nullptr) ++stats->pruned;
              return false;
            }
            const double capacity_threshold =
                root_transformed_cut_capacity_threshold(cut_index, lb, ub);
            if (ninf_min == 0 && capacity > capacity_threshold) continue;
            if (stats != nullptr) ++stats->active_rows;

            std::vector<std::tuple<int, bool, double>> bound_changes;
            bound_changes.reserve(static_cast<std::size_t>(cut.coeff.nonZeros()));
            const Eigen::VectorXd reason_lb = lb;
            const Eigen::VectorXd reason_ub = ub;
            for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it;
                 ++it) {
              const int j = static_cast<int>(it.index());
              const double a = it.value();
              if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
              const double min_bound = a > 0.0 ? reason_lb[j] : reason_ub[j];
              double residual_activity = min_activity;
              if (ninf_min == 1) {
                if (std::isfinite(min_bound)) continue;
              } else {
                if (!std::isfinite(min_bound)) continue;
                residual_activity -= a * min_bound;
              }
              const double residual = cut.rhs - residual_activity;
              double adjusted = 0.0;
              if (a > 0.0) {
                if (root_highs_adjusted_ub(j, residual / a, lb, ub,
                                           adjusted)) {
                  bound_changes.emplace_back(j, false, adjusted);
                }
              } else if (root_highs_adjusted_lb(j, residual / a, lb, ub,
                                                adjusted)) {
                bound_changes.emplace_back(j, true, adjusted);
              }
            }
            if (bound_changes.empty()) continue;
            for (const auto& change : bound_changes) {
              const int j = std::get<0>(change);
              const bool is_lb = std::get<1>(change);
              const double value = std::get<2>(change);
              const Eigen::VectorXd before_lb = lb;
              const Eigen::VectorXd before_ub = ub;
              if (is_lb) {
                if (!(value > lb[j])) continue;
                if (reason_bounds_out != nullptr) {
                  DomainReasonBound rb;
                  rb.bound = BranchDomainLiteral{j, value, true};
                  rb.reason.reserve(static_cast<std::size_t>(
                      std::max<Eigen::Index>(0, cut.coeff.nonZeros() - 1)));
                  for (Eigen::SparseVector<double>::InnerIterator jt(
                           cut.coeff);
                       jt; ++jt) {
                    const int k = static_cast<int>(jt.index());
                    const double ak = jt.value();
                    if (k < 0 || k >= n || k == j ||
                        std::abs(ak) <= 1e-12) {
                      continue;
                    }
                    if (ak > 0.0) {
                      if (std::isfinite(reason_lb[k])) {
                        rb.reason.push_back(
                            BranchDomainLiteral{k, reason_lb[k], true});
                      }
                    } else if (std::isfinite(reason_ub[k])) {
                      rb.reason.push_back(
                          BranchDomainLiteral{k, reason_ub[k], false});
                    }
                  }
                  canonicalize_branch_literals(rb.reason);
                  rb.trail_pos =
                      reason_trail_offset +
                      static_cast<int>(reason_bounds_out->size());
                  rb.depth = reason_depth;
                  rb.source_conflict_literal =
                      BranchDomainLiteral{
                          j,
                          root_highs_integral_col(j)
                              ? std::ceil(value - opt.int_tol) - 1.0
                              : value - std::max(1e-7, 10.0 * highs_mip_feastol),
                          false};
                  rb.has_source_conflict_literal = true;
                  rb.source_conflict_clause = rb.reason;
                  rb.source_conflict_clause.push_back(rb.source_conflict_literal);
                  canonicalize_branch_literals(rb.source_conflict_clause);
                  rb.has_source_conflict_clause = true;
                  reason_bounds_out->push_back(std::move(rb));
                }
                lb[j] = value;
              } else {
                if (!(value < ub[j])) continue;
                if (reason_bounds_out != nullptr) {
                  DomainReasonBound rb;
                  rb.bound = BranchDomainLiteral{j, value, false};
                  rb.reason.reserve(static_cast<std::size_t>(
                      std::max<Eigen::Index>(0, cut.coeff.nonZeros() - 1)));
                  for (Eigen::SparseVector<double>::InnerIterator jt(
                           cut.coeff);
                       jt; ++jt) {
                    const int k = static_cast<int>(jt.index());
                    const double ak = jt.value();
                    if (k < 0 || k >= n || k == j ||
                        std::abs(ak) <= 1e-12) {
                      continue;
                    }
                    if (ak > 0.0) {
                      if (std::isfinite(reason_lb[k])) {
                        rb.reason.push_back(
                            BranchDomainLiteral{k, reason_lb[k], true});
                      }
                    } else if (std::isfinite(reason_ub[k])) {
                      rb.reason.push_back(
                          BranchDomainLiteral{k, reason_ub[k], false});
                    }
                  }
                  canonicalize_branch_literals(rb.reason);
                  rb.trail_pos =
                      reason_trail_offset +
                      static_cast<int>(reason_bounds_out->size());
                  rb.depth = reason_depth;
                  rb.source_conflict_literal =
                      BranchDomainLiteral{
                          j,
                          root_highs_integral_col(j)
                              ? std::floor(value + opt.int_tol) + 1.0
                              : value + std::max(1e-7, 10.0 * highs_mip_feastol),
                          true};
                  rb.has_source_conflict_literal = true;
                  rb.source_conflict_clause = rb.reason;
                  rb.source_conflict_clause.push_back(rb.source_conflict_literal);
                  canonicalize_branch_literals(rb.source_conflict_clause);
                  rb.has_source_conflict_clause = true;
                  reason_bounds_out->push_back(std::move(rb));
                }
                ub[j] = value;
              }
              if (stats != nullptr) ++stats->tightened;
              const ChangedColSides changed =
                  collect_changed_col_sides_with_tol(
                      before_lb, before_ub, lb, ub,
                      std::max(1e-9, opt.int_tol));
              if (!changed.any.empty() &&
                  cut_index <
                      static_cast<int>(root_transformed_cut_age.size()) &&
                  root_transformed_cut_age[static_cast<std::size_t>(
                      cut_index)] >= 0) {
                root_transformed_cut_age[static_cast<std::size_t>(
                    cut_index)] = 0;
              }
              mark_root_highs_domain_rows_for_changed_cols(changed.any, lb,
                                                           ub);
              mark_root_transformed_cut_prop_for_changed_sides(changed, lb,
                                                               ub);
              if (!bounds_consistent(lb, ub)) {
                if (stats != nullptr) ++stats->pruned;
                return false;
              }
            }
          }
        }
        return bounds_consistent(lb, ub);
      };
      // HiGHS keeps cutpool rows as live domain-propagation rows.  Deriving
      // extra variable-bound separator sources from those rows changes the
      // transformed source graph and is intentionally diagnostic/opt-in.
      const bool root_transformed_cutpool_live_vb_source =
          !strict_highs_root_fixed_point &&
          bc_env_flag_enabled("MIPSOLVERS_XTAB_CUTPOOL_VB_SOURCE");
      const bool derive_dynamic_root_lp_rows_as_vb_source =
          !strict_highs_root_fixed_point ||
          bc_env_flag_enabled("MIPSOLVERS_XTAB_DYNAMIC_ROW_VB_SOURCE");
      struct RootProbeVariableBoundSource {
        bool is_vub{true};
        int col{-1};
        int trigger_col{-1};
        double coef{0.0};
        double constant{0.0};
      };
      std::vector<RootProbeVariableBoundSource> root_presolve_varbound_sources;
      std::vector<RootProbeVariableBoundSource> root_highs_varbound_sources;
      std::vector<RootProbeVariableBoundSource> root_probe_varbound_sources;
	      const bool use_highs_presolve_varbounds =
	          !highs_presolve_vb_source_disabled &&
              root_highs_presolved_side_state_first_class &&
	          highs_presolve_side_state.side_state_available &&
	          !highs_presolve_side_state.var_bounds.empty();
      const bool replay_highs_presolve_varbounds =
          use_highs_presolve_varbounds ||
          (root_highs_presolved_side_state_first_class &&
           bc_env_flag_enabled("MIPSOLVERS_REPLAY_HIGHS_PRESOLVE_VB"));
      std::unordered_set<std::string> root_probe_varbound_source_keys;
      std::vector<unsigned char> root_implied_bound_probe_cache(
          static_cast<std::size_t>(std::max(0, n)), 0);
      SparseCutSet root_implied_bound_cuts;
      std::vector<PoolCut> root_deferred_implied_bound_cutpool_rows;
      const int root_cut_varbound_source_start_row =
          static_cast<int>(root_lp.A.rows());
      auto is_root_integral_or_implied = [&](int col) {
        return root_highs_integral_col(col);
      };
      auto add_root_probe_varbound_source =
          [&](bool is_vub, int col, int trigger_col, double coef,
              double constant) -> bool {
        if (col < 0 || col >= n || trigger_col < 0 || trigger_col >= n ||
            col == trigger_col || !std::isfinite(coef) ||
            !std::isfinite(constant)) {
          return false;
        }
        const std::string key = fmt::format(
            "{}:{}:{}:{:.17g}:{:.17g}", is_vub ? 1 : 0, col, trigger_col,
            coef, constant);
        if (!root_probe_varbound_source_keys.insert(key).second) return false;
        root_probe_varbound_sources.push_back(
            RootProbeVariableBoundSource{is_vub, col, trigger_col, coef,
                                         constant});
        const bool target_integral = is_root_integral_or_implied(col);
        if (is_vub) {
          ++variable_bound_source_stats.probe_vub_candidates;
          return variable_bound_table.add_vub(
              col, trigger_col, coef, constant, root.ub[col], target_integral);
        }
        ++variable_bound_source_stats.probe_vlb_candidates;
        return variable_bound_table.add_vlb(
            col, trigger_col, coef, constant, root.lb[col], target_integral);
      };
      if (replay_highs_presolve_varbounds) {
        std::unordered_set<std::string> highs_source_keys;
        std::uint64_t highs_seen = 0;
        std::uint64_t highs_kept = 0;
        std::uint64_t highs_vub_kept = 0;
        std::uint64_t highs_vlb_kept = 0;
        std::uint64_t highs_bad_col = 0;
	        std::uint64_t highs_bad_trigger = 0;
	        std::uint64_t highs_nonbinary_trigger = 0;
	        std::uint64_t highs_duplicate = 0;
	        std::uint64_t highs_self = 0;
	        std::uint64_t highs_mapped_reduced = 0;
	        std::uint64_t highs_mapped_original = 0;
	        std::uint64_t highs_unmapped_original = 0;
	        std::uint64_t highs_nontransformable = 0;
	        std::uint64_t highs_bad_transform = 0;
	        const bool highs_side_match =
	            highs_presolve_side_state.side_state_available &&
                root_highs_presolved_side_state_first_class &&
	            highs_presolve_side_state.cols == n &&
	            !highs_presolve_side_state.var_bounds.empty();
	        if (use_highs_presolve_varbounds) {
	          for (const auto& rec : highs_presolve_side_state.var_bounds) {
	            ++highs_seen;
	            bool mapped_from_original = false;
	            int col = rec.target_col;
	            int trigger_col = rec.trigger_col;
	            bool is_vub = rec.upper;
	            double coef = rec.coef;
	            double constant = rec.constant;
	            if (!highs_side_match) {
	              if (!rec.target_linearly_transformable ||
	                  !rec.trigger_linearly_transformable) {
	                ++highs_nontransformable;
	                continue;
	              }
	              if (rec.target_orig_col < 0 || rec.trigger_orig_col < 0) {
	                ++highs_unmapped_original;
	                continue;
	              }
	              col = rec.target_orig_col;
	              trigger_col = rec.trigger_orig_col;
	              if (col < 0 || trigger_col < 0) {
	                ++highs_unmapped_original;
	                continue;
	              }
	              const double target_scale = rec.target_scale;
	              const double trigger_scale = rec.trigger_scale;
	              const double target_shift = rec.target_constant;
	              const double trigger_shift = rec.trigger_constant;
	              if (!std::isfinite(target_scale) ||
	                  !std::isfinite(trigger_scale) ||
	                  !std::isfinite(target_shift) ||
	                  !std::isfinite(trigger_shift) ||
	                  std::abs(target_scale) <= 1e-12 ||
	                  std::abs(trigger_scale) <= 1e-12) {
	                ++highs_bad_transform;
	                continue;
	              }
	              coef = rec.coef * trigger_scale / target_scale;
	              constant =
	                  (rec.coef * trigger_shift + rec.constant - target_shift) /
	                  target_scale;
	              if (target_scale < 0.0) is_vub = !is_vub;
	              mapped_from_original = true;
	            }
	            if (col == trigger_col) {
	              ++highs_self;
	              continue;
	            }
            if (col < 0 || col >= n) {
              ++highs_bad_col;
              continue;
            }
            if (trigger_col < 0 || trigger_col >= n) {
              ++highs_bad_trigger;
              continue;
            }
            const auto& trigger_var =
                root_lp.vars[static_cast<std::size_t>(trigger_col)];
            if (!(trigger_var.type == VarType::Binary ||
                  (trigger_var.type == VarType::Integer &&
                   std::abs(root.lb[trigger_col]) <= 1e-9 &&
                   std::abs(root.ub[trigger_col] - 1.0) <= 1e-9))) {
              ++highs_nonbinary_trigger;
              continue;
            }
	            if (!std::isfinite(coef) || !std::isfinite(constant)) {
	              ++highs_bad_col;
	              continue;
	            }
	            const std::string key = fmt::format(
	                "{}:{}:{}:{:.17g}:{:.17g}", is_vub ? 1 : 0, col,
	                trigger_col, coef, constant);
	            if (!highs_source_keys.insert(key).second) {
	              ++highs_duplicate;
	              continue;
	            }
	            root_highs_varbound_sources.push_back(
	                RootProbeVariableBoundSource{is_vub, col, trigger_col, coef,
	                                             constant});
	            ++highs_kept;
	            if (mapped_from_original)
	              ++highs_mapped_original;
	            else
	              ++highs_mapped_reduced;
	            if (is_vub)
	              ++highs_vub_kept;
	            else
	              ++highs_vlb_kept;
	          }
	        }
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
            bc_env_options().value("MIPSOLVERS_HIGHS_PRESOLVE_STATS") != nullptr) {
          fmt::print(stderr,
                     "[B&C-HIGHS-VB-MAP] mode={} side_match={} "
                     "first_class={} hs_cols={} native_cols={} hs_vub={} hs_vlb={} "
                     "records={} seen={} kept={} kept_vub={} kept_vlb={} "
	                     "bad_col={} bad_trigger={} nonbin={} dup={} self={} "
	                     "redMap={} origMap={} unmapOrig={} nonxform={} "
	                     "badXform={} "
	                     "src_stats=vub{}/{}/{}:vlb{}/{}/{}\n",
                     use_highs_presolve_varbounds ? "first_class"
                                                  : "diagnostic",
                     highs_side_match ? 1 : 0,
                     root_highs_presolved_side_state_first_class ? 1 : 0,
                     highs_presolve_side_state.cols,
                     n, highs_presolve_side_state.vub_count,
                     highs_presolve_side_state.vlb_count,
                     highs_presolve_side_state.var_bounds.size(), highs_seen,
                     highs_kept, highs_vub_kept, highs_vlb_kept,
	                     highs_bad_col, highs_bad_trigger,
	                     highs_nonbinary_trigger, highs_duplicate, highs_self,
	                     highs_mapped_reduced, highs_mapped_original,
	                     highs_unmapped_original, highs_nontransformable,
	                     highs_bad_transform,
	                     highs_presolve_side_state.vub_accepted,
                     highs_presolve_side_state.vub_attempts,
                     highs_presolve_side_state.vub_replaced,
                     highs_presolve_side_state.vlb_accepted,
                     highs_presolve_side_state.vlb_attempts,
                     highs_presolve_side_state.vlb_replaced);
        }
      } else if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
                 bc_env_options().value("MIPSOLVERS_HIGHS_PRESOLVE_STATS") != nullptr) {
	          fmt::print(stderr,
	                   "[B&C-HIGHS-VB-MAP] mode=disabled side_match=0 "
	                   "first_class={} hs_cols={} native_cols={} hs_vub={} hs_vlb={} "
	                   "records={} seen=0 kept=0 kept_vub=0 kept_vlb=0 "
	                   "bad_col=0 bad_trigger=0 nonbin=0 dup=0 self=0 "
	                   "redMap=0 origMap=0 unmapOrig=0 nonxform=0 badXform=0 "
	                   "src_stats=vub{}/{}/{}:vlb{}/{}/{}\n",
                   root_highs_presolved_side_state_first_class ? 1 : 0,
                   highs_presolve_side_state.cols, n,
                   highs_presolve_side_state.vub_count,
                   highs_presolve_side_state.vlb_count,
                   highs_presolve_side_state.var_bounds.size(),
                   highs_presolve_side_state.vub_accepted,
                   highs_presolve_side_state.vub_attempts,
                   highs_presolve_side_state.vub_replaced,
                   highs_presolve_side_state.vlb_accepted,
                   highs_presolve_side_state.vlb_attempts,
                   highs_presolve_side_state.vlb_replaced);
      }
      if (!original_presolve_vb_sources.empty()) {
        std::unordered_set<std::string> presolve_source_keys;
        std::vector<PaPILOAffineExpression> papilo_affine_exprs;
#ifdef MIPSOLVERS_HAVE_PAPILO
        if (papilo_ps.success && !papilo_ps.orig_to_reduced_col.empty()) {
          papilo_affine_exprs =
              papilo_export_affine_expressions(papilo_ps, 8, 1e-10);
        }
#endif
        std::uint64_t presolve_map_seen = 0;
        std::uint64_t presolve_map_elim_col = 0;
        std::uint64_t presolve_map_elim_trigger = 0;
        std::uint64_t presolve_map_bad_col = 0;
        std::uint64_t presolve_map_bad_trigger = 0;
        std::uint64_t presolve_map_nonbinary_trigger = 0;
        std::uint64_t presolve_map_duplicate = 0;
        std::uint64_t presolve_map_kept = 0;
        std::uint64_t presolve_map_projected_col = 0;
        std::uint64_t presolve_map_project_no_expr = 0;
        std::uint64_t presolve_map_project_inexact = 0;
        std::uint64_t presolve_map_project_multi = 0;
        std::uint64_t presolve_map_project_bad_alpha = 0;
        std::uint64_t presolve_map_project_self = 0;
        for (const auto& source : original_presolve_vb_sources) {
          ++presolve_map_seen;
          int col = source.col;
          int trigger_col = source.trigger_col;
          bool is_vub = source.is_vub;
          double coef = source.coef;
          double constant = source.constant;
#ifdef MIPSOLVERS_HAVE_PAPILO
          if (papilo_ps.success && !papilo_ps.orig_to_reduced_col.empty()) {
            if (col < 0 ||
                col >= static_cast<int>(papilo_ps.orig_to_reduced_col.size()) ||
                trigger_col < 0 ||
                trigger_col >=
                    static_cast<int>(papilo_ps.orig_to_reduced_col.size())) {
              ++presolve_map_bad_col;
              continue;
            }
            const int mapped_col =
                papilo_ps.orig_to_reduced_col[static_cast<std::size_t>(col)];
            const int mapped_trigger =
                papilo_ps.orig_to_reduced_col[static_cast<std::size_t>(trigger_col)];
            if (mapped_col < 0) {
              if (col < 0 ||
                  col >= static_cast<int>(papilo_affine_exprs.size())) {
                ++presolve_map_elim_col;
                ++presolve_map_project_no_expr;
                continue;
              }
              const auto& expr =
                  papilo_affine_exprs[static_cast<std::size_t>(col)];
              if (!expr.available) {
                ++presolve_map_elim_col;
                ++presolve_map_project_no_expr;
                continue;
              }
              if (!expr.exact) {
                ++presolve_map_elim_col;
                ++presolve_map_project_inexact;
                continue;
              }
              if (expr.terms.size() != 1) {
                ++presolve_map_elim_col;
                ++presolve_map_project_multi;
                continue;
              }
              const double alpha = expr.terms.front().coef;
              if (!std::isfinite(alpha) || std::abs(alpha) <= 1e-10) {
                ++presolve_map_elim_col;
                ++presolve_map_project_bad_alpha;
                continue;
              }
              col = expr.terms.front().col;
              coef /= alpha;
              constant = (constant - expr.constant) / alpha;
              if (alpha < 0.0) is_vub = !is_vub;
              ++presolve_map_projected_col;
            } else {
              col = mapped_col;
            }
            if (mapped_trigger < 0) {
              ++presolve_map_elim_trigger;
              continue;
            }
            trigger_col = mapped_trigger;
          }
#endif
          if (col == trigger_col) {
            ++presolve_map_project_self;
            continue;
          }
          if (col < 0 || col >= n) {
            ++presolve_map_bad_col;
            continue;
          }
          if (trigger_col < 0 || trigger_col >= n) {
            ++presolve_map_bad_trigger;
            continue;
          }
          const auto& trigger_var = root_lp.vars[static_cast<std::size_t>(trigger_col)];
          if (!(trigger_var.type == VarType::Binary ||
                (trigger_var.type == VarType::Integer &&
                 std::abs(root.lb[trigger_col]) <= 1e-9 &&
                 std::abs(root.ub[trigger_col] - 1.0) <= 1e-9))) {
            ++presolve_map_nonbinary_trigger;
            continue;
          }
          const std::string key = fmt::format(
              "{}:{}:{}:{:.17g}:{:.17g}", is_vub ? 1 : 0, col,
              trigger_col, coef, constant);
          if (!presolve_source_keys.insert(key).second) {
            ++presolve_map_duplicate;
            continue;
          }
          root_presolve_varbound_sources.push_back(
              RootProbeVariableBoundSource{is_vub, col, trigger_col, coef,
                                           constant});
          ++presolve_map_kept;
        }
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
          fmt::print(stderr,
                     "[B&C-PRESOLVE-VB-MAP] seen={} kept={} elim_col={} "
                     "elim_trigger={} bad_col={} bad_trigger={} nonbin={} "
                     "dup={} projected={} proj_no_expr={} proj_inexact={} "
                     "proj_multi={} proj_bad_alpha={} proj_self={}\n",
                     presolve_map_seen, presolve_map_kept,
                     presolve_map_elim_col, presolve_map_elim_trigger,
                     presolve_map_bad_col, presolve_map_bad_trigger,
                     presolve_map_nonbinary_trigger, presolve_map_duplicate,
                     presolve_map_projected_col, presolve_map_project_no_expr,
                     presolve_map_project_inexact, presolve_map_project_multi,
                     presolve_map_project_bad_alpha,
                     presolve_map_project_self);
        }
      }
      auto replay_varbound_sources =
          [&](const std::vector<RootProbeVariableBoundSource>& sources,
              const char* tag) -> std::uint64_t {
        if (sources.empty()) return 0;
        std::uint64_t replayed = 0;
        std::uint64_t vub_attempts = 0;
        std::uint64_t vlb_attempts = 0;
        for (const auto& source : sources) {
          const bool target_integral = is_root_integral_or_implied(source.col);
          bool changed = false;
          if (source.is_vub) {
            ++vub_attempts;
            ++variable_bound_source_stats.presolve_vub_candidates;
            changed = variable_bound_table.add_vub(
                source.col, source.trigger_col, source.coef, source.constant,
                root.ub[source.col], target_integral);
          } else {
            ++vlb_attempts;
            ++variable_bound_source_stats.presolve_vlb_candidates;
            changed = variable_bound_table.add_vlb(
                source.col, source.trigger_col, source.coef, source.constant,
                root.lb[source.col], target_integral);
          }
          if (changed) ++replayed;
        }
        variable_bound_source_stats.presolve_varbounds_replayed += replayed;
        const std::uint64_t exported =
            strict_highs_root_fixed_point
                ? 0
                : variable_bound_table.export_to_implication_graph(
                      implication_graph);
        variable_bound_source_stats.presolve_exported_implications += exported;
        variable_bound_source_stats.exported_implications += exported;
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
            bc_env_options().value("MIPSOLVERS_HIGHS_PRESOLVE_STATS") != nullptr) {
          fmt::print(stderr,
                     "[B&C-VB-REPLAY] tag={} sources={} vub={} vlb={} "
                     "accepted={} table={} exported={} graph={}\n",
                     tag != nullptr ? tag : "presolve", sources.size(),
                     vub_attempts, vlb_attempts, replayed,
                     variable_bound_table.size(), exported,
                     implication_graph.size());
        }
        return replayed;
      };
      auto replay_root_presolve_varbound_sources = [&]() {
        replay_varbound_sources(root_highs_varbound_sources, "highs");
        if (!strict_highs_root_fixed_point) {
          replay_varbound_sources(root_presolve_varbound_sources, "native");
        }
      };
      auto replay_root_probe_varbound_sources = [&]() {
        if (root_probe_varbound_sources.empty()) return;
        std::uint64_t replayed = 0;
        for (const auto& source : root_probe_varbound_sources) {
          const bool target_integral = is_root_integral_or_implied(source.col);
          bool changed = false;
          if (source.is_vub) {
            changed = variable_bound_table.add_vub(
                source.col, source.trigger_col, source.coef, source.constant,
                root.ub[source.col], target_integral);
          } else {
            changed = variable_bound_table.add_vlb(
                source.col, source.trigger_col, source.coef, source.constant,
                root.lb[source.col], target_integral);
          }
          if (changed) ++replayed;
        }
        variable_bound_source_stats.probe_varbounds_replayed += replayed;
        const std::uint64_t exported =
            strict_highs_root_fixed_point
                ? 0
                : variable_bound_table.export_to_implication_graph(
                      implication_graph);
        variable_bound_source_stats.probe_exported_implications += exported;
        variable_bound_source_stats.exported_implications += exported;
      };
	  auto sync_variable_bound_stats = [&]() {
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
	  };
	  auto refresh_variable_bound_table = [&](const LPModel& source_lp,
	                                          const char* phase) {
	    const std::size_t old_size = variable_bound_table.size();
	    if (!strict_highs_root_fixed_point) {
	      variable_bound_source_stats = build_variable_bound_table_from_rows(
	          source_lp, root_implied_integer_cols, variable_bound_table,
	          &implication_graph, 512);
	    }
		    if (!strict_highs_root_fixed_point &&
            derive_dynamic_root_lp_rows_as_vb_source &&
            source_lp.A.rows() > root_cut_varbound_source_start_row) {
	      NativeVariableBoundSourceStats cut_stats =
	          augment_variable_bound_table_from_cut_rows(
	              source_lp, root_implied_integer_cols,
	              root_cut_varbound_source_start_row,
	              static_cast<int>(source_lp.A.rows()), variable_bound_table,
	              &implication_graph, 100, std::max(1e-9, opt.lp_tol));
		      accumulate_variable_bound_source_stats(variable_bound_source_stats,
		                                             cut_stats);
		    }
		    if (!strict_highs_root_fixed_point &&
            !variable_bound_cutpool_rows.empty()) {
		      NativeVariableBoundSourceStats cutpool_stats =
		          augment_variable_bound_table_from_sparse_cut_rows(
		              source_lp, variable_bound_cutpool_rows,
		              variable_bound_cutpool_rhs, root_implied_integer_cols,
		              variable_bound_table, &implication_graph, 100,
		              std::max(1e-9, opt.lp_tol));
		      accumulate_variable_bound_source_stats(variable_bound_source_stats,
		                                             cutpool_stats);
		    }
        replay_root_presolve_varbound_sources();
        replay_root_probe_varbound_sources();
		    sync_variable_bound_stats();
		    if (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr) {
		      fmt::print(stderr,
	                 "[B&C-VB-SRC] phase={} old={} new={} rows={} single={} "
	                 "cand={} cutRows={} cutMixed={} cutVUB={} cutVLB={} "
	                 "preVUB={} preVLB={} preReplay={} "
                         "implBnd={}p/{}/{} "
                         "probeVUB={} probeVLB={} probeReplay={} "
                         "probeTight={} vub={}/{} vlb={}/{} exp={} cutExp={} "
                         "preExp={} probeExp={}\n",
		                 phase != nullptr ? phase : "refresh", old_size,
		                 variable_bound_table.size(),
		                 variable_bound_source_stats.row_sides_scanned,
		                 variable_bound_source_stats.single_binary_row_sides,
		                 variable_bound_source_stats.candidates,
	                 variable_bound_source_stats.cut_rows_scanned,
	                 variable_bound_source_stats.cut_mixed_rows,
		                 variable_bound_source_stats.cut_vub_candidates,
	                 variable_bound_source_stats.cut_vlb_candidates,
                         variable_bound_source_stats.presolve_vub_candidates,
                         variable_bound_source_stats.presolve_vlb_candidates,
                         variable_bound_source_stats.presolve_varbounds_replayed,
                         variable_bound_source_stats.implied_bound_passes,
                         variable_bound_source_stats.implied_bound_lower,
                         variable_bound_source_stats.implied_bound_upper,
                         variable_bound_source_stats.probe_vub_candidates,
                         variable_bound_source_stats.probe_vlb_candidates,
                         variable_bound_source_stats.probe_varbounds_replayed,
                         variable_bound_source_stats.probe_global_tightenings,
		                 variable_bound_table.stats().vub_accepted,
		                 variable_bound_table.stats().vub_attempts,
		                 variable_bound_table.stats().vlb_accepted,
		                 variable_bound_table.stats().vlb_attempts,
		                 variable_bound_source_stats.exported_implications,
		                 variable_bound_source_stats.cut_exported_implications,
                         variable_bound_source_stats.presolve_exported_implications,
	                         variable_bound_source_stats.probe_exported_implications);
			    }
            trace_native_presolve_state_conformance(
                phase != nullptr ? phase : "refresh", source_lp,
                root_implied_integer_cols, variable_bound_table);
			  };
      auto append_variable_bound_sparse_cutpool_row =
          [&](const Eigen::SparseVector<double>& sparse_row,
              double rhs) -> bool {
        if (sparse_row.size() != n || sparse_row.nonZeros() <= 0 ||
            sparse_row.nonZeros() > 100 || !std::isfinite(rhs)) {
          return false;
        }
        const std::size_t hash = sparse_cut_hash(sparse_row);
        const std::string key =
            fmt::format("{}:{}:{:.17g}", hash, sparse_row.nonZeros(), rhs);
        if (!variable_bound_cutpool_row_keys.insert(key).second) return false;
        variable_bound_cutpool_rows.push_back(sparse_row);
        variable_bound_cutpool_rhs.push_back(rhs);
        return true;
      };
      auto extract_variable_bounds_from_cutpool_row =
          [&](const Eigen::SparseVector<double>& sparse_row,
              double rhs) -> bool {
        if (sparse_row.size() != n || sparse_row.nonZeros() <= 0 ||
            sparse_row.nonZeros() > 100 || !std::isfinite(rhs)) {
          return false;
        }
        std::vector<Eigen::SparseVector<double>> rows;
        std::vector<double> rhs_values;
        rows.push_back(sparse_row);
        rhs_values.push_back(rhs);
        NativeVariableBoundSourceStats cut_stats =
            augment_variable_bound_table_from_sparse_cut_rows(
                root_lp, rows, rhs_values, root_implied_integer_cols,
            variable_bound_table, nullptr, 100,
                std::max(1e-9, opt.lp_tol));
        const bool changed = cut_stats.cut_vub_candidates > 0 ||
                             cut_stats.cut_vlb_candidates > 0 ||
                             cut_stats.cut_exported_implications > 0;
        if (changed) {
          accumulate_variable_bound_source_stats(variable_bound_source_stats,
                                                 cut_stats);
          sync_variable_bound_stats();
        }
        return changed;
      };
      const double root_highs_cutpool_avg_model_nnz =
          root_lp.A.rows() > 0
              ? static_cast<double>(root_lp.A.nonZeros()) /
                    static_cast<double>(root_lp.A.rows())
              : 0.0;
      const double root_highs_cutpool_min_density_limit =
          0.1 * static_cast<double>(std::max(1, n));
      const double root_highs_cutpool_propagate_nnz_limit = std::max(
          2.0 * root_highs_cutpool_avg_model_nnz,
          root_highs_cutpool_min_density_limit);
      auto append_root_transformed_cutpool_audit_row =
          [&](const Eigen::SparseVector<double>& sparse_row,
              double rhs) -> bool {
        if (sparse_row.size() != n || sparse_row.nonZeros() <= 0 ||
            !std::isfinite(rhs)) {
          return false;
        }
        // HiGHS CutPool::addCut(..., propagate=true) keeps cut rows in the
        // propagation matrix unless their actual length exceeds the same
        // density gate used by HighsCutPool.  The legacy non-strict path keeps
        // the old sparse-audit cap to avoid changing its source model.
        if (strict_highs_root_fixed_point) {
          if (static_cast<double>(sparse_row.nonZeros()) >=
              root_highs_cutpool_propagate_nnz_limit) {
            return false;
          }
        } else if (sparse_row.nonZeros() > 100) {
          return false;
        }
        const std::size_t hash = sparse_cut_hash(sparse_row);
        const std::string key =
            fmt::format("{}:{}:{:.17g}", hash, sparse_row.nonZeros(), rhs);
        if (!root_transformed_cutpool_audit_row_keys.insert(key).second) {
          return false;
        }
        root_transformed_cutpool_audit_rows.push_back(sparse_row);

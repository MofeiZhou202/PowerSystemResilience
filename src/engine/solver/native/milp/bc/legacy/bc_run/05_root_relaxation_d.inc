// @file bc_run/05_root_relaxation_d.inc
// BCSolveState::run() segment 5/14: root LP relaxation / cut loop — part 4 (crossover, root finalization).
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
		          return env != nullptr && env[0] != '\0' && env[0] != '0';
		        };
		        auto xpool_state_trace_terms = []() {
		          const char* env = bc_env_options().value("MIPSOLVERS_XPOOL_STATE_TRACE_TERMS");
		          if (env == nullptr || env[0] == '\0') return 24;
		          char* end = nullptr;
		          const long parsed = std::strtol(env, &end, 10);
		          if (end == env || parsed < 0) return 24;
		          return static_cast<int>(std::min<long>(parsed, 512));
		        };
		        auto xpool_state_bits = [](double value) -> std::uint64_t {
		          std::uint64_t bits = 0;
		          static_assert(sizeof(bits) == sizeof(value));
		          std::memcpy(&bits, &value, sizeof(bits));
		          return bits;
		        };
	        auto xpool_rows_payload_hash =
	            [&](const std::vector<Eigen::SparseVector<double>>& hash_rows,
	                const std::vector<double>& hash_rhs,
	                int* nnz_out) -> std::uint64_t {
	          std::uint64_t hash = std::uint64_t{0x4849474858504150};
	          int nnz = 0;
	          const int k = static_cast<int>(
	              std::min(hash_rows.size(), hash_rhs.size()));
	          for (int i = 0; i < k; ++i) {
	            const auto& row = hash_rows[static_cast<std::size_t>(i)];
	            const int len = static_cast<int>(row.nonZeros());
	            nnz += len;
	            std::uint64_t row_hash = std::uint64_t{0x4849474858505257};
	            row_hash = xpool_ledger_mix(row_hash,
	                                        static_cast<std::uint64_t>(i));
	            row_hash = xpool_ledger_mix(row_hash,
	                                        static_cast<std::uint64_t>(len));
	            row_hash = xpool_ledger_mix(
	                row_hash,
	                xpool_ledger_double(-std::numeric_limits<double>::infinity()));
	            row_hash = xpool_ledger_mix(
	                row_hash,
	                xpool_ledger_double(hash_rhs[static_cast<std::size_t>(i)]));
	            for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
	              row_hash = xpool_ledger_mix(
	                  row_hash, static_cast<std::uint64_t>(it.index()));
	              row_hash =
	                  xpool_ledger_mix(row_hash, xpool_ledger_double(it.value()));
	            }
	            hash = xpool_ledger_mix(hash, row_hash);
	          }
	          if (nnz_out != nullptr) *nnz_out = nnz;
	          return hash;
	        };
	        auto xpool_lp_appended_payload_hash =
	            [&](const LPModel& lp, int first_row, int num_rows,
	                int* nnz_out) -> std::uint64_t {
	          std::uint64_t hash = std::uint64_t{0x4849474858504150};
	          int nnz = 0;
	          if (num_rows <= 0 || first_row < 0 ||
	              first_row + num_rows > lp.A.rows()) {
	            if (nnz_out != nullptr) *nnz_out = 0;
	            return hash;
	          }
	          std::vector<std::vector<std::pair<int, double>>> row_terms(
	              static_cast<std::size_t>(num_rows));
	          for (int col = 0; col < lp.A.outerSize(); ++col) {
	            for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col);
	                 it; ++it) {
	              const int row = static_cast<int>(it.row());
	              if (row < first_row || row >= first_row + num_rows) continue;
	              row_terms[static_cast<std::size_t>(row - first_row)]
	                  .push_back({static_cast<int>(it.col()), it.value()});
	            }
	          }
	          for (int i = 0; i < num_rows; ++i) {
	            auto& terms = row_terms[static_cast<std::size_t>(i)];
	            std::sort(terms.begin(), terms.end(),
	                      [](const auto& a, const auto& b) {
	                        return a.first < b.first;
	                      });
	            const int len = static_cast<int>(terms.size());
	            nnz += len;
	            std::uint64_t row_hash = std::uint64_t{0x4849474858505257};
	            row_hash = xpool_ledger_mix(row_hash,
	                                        static_cast<std::uint64_t>(i));
	            row_hash = xpool_ledger_mix(row_hash,
	                                        static_cast<std::uint64_t>(len));
	            row_hash = xpool_ledger_mix(
	                row_hash,
	                xpool_ledger_double(lp_row_lhs_or_neg_inf(lp, first_row + i)));
	            row_hash = xpool_ledger_mix(
	                row_hash, xpool_ledger_double(lp.b[first_row + i]));
	            for (const auto& [col, value] : terms) {
	              row_hash = xpool_ledger_mix(
	                  row_hash, static_cast<std::uint64_t>(col));
	              row_hash =
	                  xpool_ledger_mix(row_hash, xpool_ledger_double(value));
	            }
	            hash = xpool_ledger_mix(hash, row_hash);
	          }
	          if (nnz_out != nullptr) *nnz_out = nnz;
	          return hash;
	        };
	        auto xpool_log_resolve_ledger =
	            [&](const char* path,
	                const LPModel& ledger_lp,
	                const LPRelaxationResult& relax,
	                const Eigen::VectorXd& x,
	                const SimplexBasis* basis,
	                int iter_delta,
	                double resolve_ms) {
	          if (!xpool_ledger_enabled()) return;
	          auto highs_status_char = [](HighsBasisStatus status) -> char {
	            switch (status) {
	              case HighsBasisStatus::kBasic:
	                return 'B';
	              case HighsBasisStatus::kUpper:
	                return 'U';
	              case HighsBasisStatus::kLower:
	                return 'L';
	              case HighsBasisStatus::kZero:
	                return 'Z';
	              case HighsBasisStatus::kNonbasic:
	                return 'N';
	            }
	            return '?';
	          };
	          std::shared_ptr<Highs> ledger_highs;
	          if (basis != nullptr && basis->cached_sparse_basis) {
	            ledger_highs = basis->cached_sparse_basis->highs_handle();
	          }
	          const bool use_highs_ledger =
	              ledger_highs && static_cast<int>(
	                                  ledger_highs->getLp().num_col_) >= n &&
	              static_cast<int>(ledger_highs->getSolution().col_value.size()) >= n;
	          const std::vector<char> basic =
	              use_highs_ledger ? std::vector<char>{}
	                               : bc_original_basic_mask(n, basis);
	          int basic_cols = 0;
	          int lower_cols = 0;
	          int upper_cols = 0;
	          int zero_cols = 0;
	          int other_cols = 0;
	          int fractional = 0;
	          std::uint64_t frontier_hash = std::uint64_t{0x4849474853465241};
	          std::uint64_t status_hash = std::uint64_t{0x4849474853535441};
	          std::uint64_t basis_hash = std::uint64_t{0x4849474858424153};
	          fmt::memory_buffer sample;
	          int emitted = 0;
		          const HighsSolution* highs_sol =
		              use_highs_ledger ? &ledger_highs->getSolution() : nullptr;
		          const HighsBasis* highs_basis =
		              use_highs_ledger ? &ledger_highs->getBasis() : nullptr;
		          const char* ledger_src = use_highs_ledger ? "highs" : "native";
		          const int ledger_highs_rows =
		              use_highs_ledger
		                  ? static_cast<int>(ledger_highs->getLp().num_row_)
		                  : -1;
		          const int ledger_highs_cols =
		              use_highs_ledger
		                  ? static_cast<int>(ledger_highs->getLp().num_col_)
		                  : -1;
		          const int ledger_highs_sol_cols =
		              use_highs_ledger
		                  ? static_cast<int>(highs_sol->col_value.size())
		                  : -1;
		          for (int j = 0; j < n; ++j) {
	            char status = '?';
	            double value =
	                (j < x.size()) ? x[j] : std::numeric_limits<double>::quiet_NaN();
	            if (use_highs_ledger) {
	              value = highs_sol->col_value[static_cast<std::size_t>(j)];
	              const HighsBasisStatus raw_status =
	                  highs_basis->valid &&
	                          j < static_cast<int>(highs_basis->col_status.size())
	                      ? highs_basis->col_status[static_cast<std::size_t>(j)]
	                      : HighsBasisStatus::kNonbasic;
	              status = highs_status_char(raw_status);
	              switch (raw_status) {
	                case HighsBasisStatus::kBasic:
	                  ++basic_cols;
	                  break;
	                case HighsBasisStatus::kLower:
	                  ++lower_cols;
	                  break;
	                case HighsBasisStatus::kUpper:
	                  ++upper_cols;
	                  break;
	                case HighsBasisStatus::kZero:
	                  ++zero_cols;
	                  break;
	                case HighsBasisStatus::kNonbasic:
	                  ++other_cols;
	                  break;
	              }
	            } else {
	              const bool fixed =
	                  std::isfinite(
	                      ledger_lp.vars[static_cast<std::size_t>(j)].lb) &&
	                  std::isfinite(
	                      ledger_lp.vars[static_cast<std::size_t>(j)].ub) &&
	                  ledger_lp.vars[static_cast<std::size_t>(j)].ub -
	                          ledger_lp.vars[static_cast<std::size_t>(j)].lb <=
	                      std::max(1e-9, highs_mip_feastol);
	              if (j < static_cast<int>(basic.size()) &&
	                  basic[static_cast<std::size_t>(j)] != 0) {
	                status = 'B';
	                ++basic_cols;
	              } else if (fixed) {
	                status = 'Z';
	                ++zero_cols;
	              } else if (basis != nullptr &&
	                         j < static_cast<int>(basis->at_upper.size()) &&
	                         basis->at_upper[static_cast<std::size_t>(j)] != 0) {
	                status = 'U';
	                ++upper_cols;
	              } else {
	                status = 'L';
	                ++lower_cols;
	              }
	              if (status == '?') ++other_cols;
	            }
	            status_hash = xpool_ledger_mix(
	                status_hash, static_cast<std::uint64_t>(j));
	            status_hash = xpool_ledger_mix(
	                status_hash, static_cast<std::uint64_t>(status));
	            status_hash =
	                xpool_ledger_mix(status_hash, xpool_ledger_double(value));
	            basis_hash = xpool_ledger_mix(
	                basis_hash, static_cast<std::uint64_t>(j));
	            basis_hash = xpool_ledger_mix(
	                basis_hash, static_cast<std::uint64_t>(status));
		            const bool integer_like =
	                ledger_lp.vars[static_cast<std::size_t>(j)].type ==
	                    VarType::Binary ||
	                ledger_lp.vars[static_cast<std::size_t>(j)].type ==
	                    VarType::Integer ||
	                (j < static_cast<int>(root_implied_integer_cols.size()) &&
	                 root_implied_integer_cols[static_cast<std::size_t>(j)] != 0);
	            if (!integer_like || !std::isfinite(value)) continue;
	            const double rounded =
	                std::min(ledger_lp.vars[static_cast<std::size_t>(j)].ub,
	                         std::max(ledger_lp.vars[static_cast<std::size_t>(j)].lb,
	                                  std::round(value)));
	            const double frac = std::abs(value - rounded);
	            if (!(frac > highs_mip_feastol)) continue;
	            ++fractional;
	            frontier_hash = xpool_ledger_mix(
	                frontier_hash, static_cast<std::uint64_t>(j));
	            frontier_hash =
	                xpool_ledger_mix(frontier_hash, xpool_ledger_double(value));
	            frontier_hash = xpool_ledger_mix(
	                frontier_hash, static_cast<std::uint64_t>(status));
	            if (emitted < 16) {
	              if (emitted++ > 0)
	                fmt::format_to(std::back_inserter(sample), ";");
	              fmt::format_to(std::back_inserter(sample),
	                             "{}:{:.12g}:r{:.12g}:f{:.6g}:{}",
	                             j, value, rounded, frac, status);
	            }
	          }
			          fmt::print(stderr,
			                     "[B&C-XPOOL-RESOLVE] path={} ledgerSrc={} "
		                     "hRows={} hCols={} hSolCols={} status={} obj={:.17g} "
		                     "rows={} cols={} nnz={} iterDelta={} frac={} "
		                     "frontierHash={:016x} statusHash={:016x} "
	                     "basisHash={:016x} "
	                     "basis=basic{}:lower{}:upper{}:zero{}:other{} "
		                     "sample=[{}] resolveMs={:.6f}\n",
		                     path != nullptr ? path : "unknown",
		                     ledger_src, ledger_highs_rows, ledger_highs_cols,
		                     ledger_highs_sol_cols,
		                     relax.primal.stats.status,
	                     relax.primal.stats.objective,
	                     static_cast<int>(ledger_lp.A.rows()),
	                     static_cast<int>(ledger_lp.vars.size()),
	                     static_cast<int>(ledger_lp.A.nonZeros()), iter_delta,
			                     fractional, frontier_hash, status_hash, basis_hash,
		                     basic_cols, lower_cols, upper_cols, zero_cols, other_cols,
		                     fmt::to_string(sample), resolve_ms);
		          if (xpool_state_trace_enabled()) {
		            const int block_size = 64;
		            for (int start = 0; start < n; start += block_size) {
		              const int end = std::min(n, start + block_size);
		              std::uint64_t block_hash =
		                  std::uint64_t{0x4849474853544154};
		              for (int j = start; j < end; ++j) {
		                char status = '?';
		                double value = (j < x.size())
		                                   ? x[j]
		                                   : std::numeric_limits<double>::quiet_NaN();
		                if (use_highs_ledger) {
		                  value = highs_sol->col_value[static_cast<std::size_t>(j)];
		                  const HighsBasisStatus raw_status =
		                      highs_basis->valid &&
		                              j < static_cast<int>(
		                                      highs_basis->col_status.size())
		                          ? highs_basis->col_status[static_cast<std::size_t>(j)]
		                          : HighsBasisStatus::kNonbasic;
		                  status = highs_status_char(raw_status);
		                } else if (j < static_cast<int>(basic.size()) &&
		                           basic[static_cast<std::size_t>(j)] != 0) {
		                  status = 'B';
		                } else if (basis != nullptr &&
		                           j < static_cast<int>(basis->at_upper.size()) &&
		                           basis->at_upper[static_cast<std::size_t>(j)] != 0) {
		                  status = 'U';
		                } else {
		                  status = 'L';
		                }
		                block_hash = xpool_ledger_mix(
		                    block_hash, static_cast<std::uint64_t>(j));
		                block_hash = xpool_ledger_mix(
		                    block_hash, static_cast<std::uint64_t>(status));
		                block_hash =
		                    xpool_ledger_mix(block_hash, xpool_ledger_double(value));
		              }
		              fmt::print(stderr,
		                         "[B&C-XPOOL-STATE-BLOCK] rows={} start={} end={} "
		                         "hash={:016x}\n",
		                         static_cast<int>(ledger_lp.A.rows()), start, end,
		                         block_hash);
		            }
		            const int limit = xpool_state_trace_terms();
		            int emitted_terms = 0;
		            for (int j = 0; j < n && emitted_terms < limit; ++j) {
		              char status = '?';
		              double value =
		                  (j < x.size()) ? x[j] : std::numeric_limits<double>::quiet_NaN();
		              if (use_highs_ledger) {
		                value = highs_sol->col_value[static_cast<std::size_t>(j)];
		                const HighsBasisStatus raw_status =
		                    highs_basis->valid &&
		                            j < static_cast<int>(highs_basis->col_status.size())
		                        ? highs_basis->col_status[static_cast<std::size_t>(j)]
		                        : HighsBasisStatus::kNonbasic;
		                status = highs_status_char(raw_status);
		              } else if (j < static_cast<int>(basic.size()) &&
		                         basic[static_cast<std::size_t>(j)] != 0) {
		                status = 'B';
		              } else if (basis != nullptr &&
		                         j < static_cast<int>(basis->at_upper.size()) &&
		                         basis->at_upper[static_cast<std::size_t>(j)] != 0) {
		                status = 'U';
		              } else {
		                status = 'L';
		              }
		              if (status != 'B' && value == 0.0) continue;
		              fmt::print(stderr,
		                         "[B&C-XPOOL-STATE-COL] rows={} col={} status={} "
		                         "value={:.17g} bits={:016x} dh={:08x}\n",
		                         static_cast<int>(ledger_lp.A.rows()), j, status,
		                         value, xpool_state_bits(value),
		                         HighsHashHelpers::double_hash_code(value));
		              ++emitted_terms;
		            }
		          }
		        };
			    if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
			      fmt::print(stderr,
			                 "[B&C ROOT-XPOOL] pool={} avail={} violated={} active={} "
		                 "efficacious={} selected={} parallel={} agedDeleted={} "
                         "bestScore={:.12g} minScoreFactor={:.12g} maxViol={:.12g} "
                         "agelim={} numLpCuts={} selNnz={} selIdxHash={:016x} "
                         "selContentHash={:016x}\n",
		                 static_cast<int>(root_transformed_cutpool.size()),
		                 pool_stats.available, pool_stats.violated, pool_stats.active,
                         pool_stats.efficacious, pool_stats.selected,
                         pool_stats.parallel, pool_stats.aged_deleted,
		                 pool_stats.best_score, pool_stats.min_score_factor,
                         pool_stats.max_violation, pool_stats.age_limit,
                         pool_stats.num_lp_cuts, pool_stats.selected_nnz,
                         pool_stats.selected_index_hash,
                         pool_stats.selected_content_hash);
		    }
		    if (selected.empty()) {
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
	        }

        // HiGHS cutpool.separate() marks selected rows as LP-owned
        // immediately (age=-1) before lp.addCuts()/resolveLp().  Keep the same
        // state machine here: selected rows leave the available cutpool before
        // the trial LP is built; only the LP row number is filled after a
        // successful add/resolve.
        std::vector<int> selected_cut_indices;
        selected_cut_indices.reserve(selected.size());
        for (const auto& picked : selected) {
          if (picked.index < 0 ||
              picked.index >= static_cast<int>(root_transformed_cutpool.size())) {
            continue;
          }
          selected_cut_indices.push_back(picked.index);
          if (picked.index <
              static_cast<int>(root_transformed_cut_in_lp.size())) {
            root_transformed_cut_in_lp[static_cast<std::size_t>(
                picked.index)] = 1;
          }
          if (picked.index <
              static_cast<int>(root_transformed_cut_age.size())) {
            root_transformed_cut_age[static_cast<std::size_t>(
                picked.index)] = -1;
          }
        }
        auto rollback_selected_cutpool_lp_pending = [&]() {
          for (int cut_index : selected_cut_indices) {
            if (cut_index >= 0 &&
                cut_index <
                    static_cast<int>(root_transformed_cut_in_lp.size())) {
              root_transformed_cut_in_lp[static_cast<std::size_t>(
                  cut_index)] = 0;
            }
            if (cut_index >= 0 &&
                cut_index < static_cast<int>(root_transformed_cut_age.size())) {
              root_transformed_cut_age[static_cast<std::size_t>(cut_index)] =
                  0;
            }
          }
        };

        const auto t_build0 = std::chrono::steady_clock::now();
	    LPModel trial_lp = root_lp;
	    const int prev_rows = static_cast<int>(trial_lp.A.rows());
	    std::vector<Eigen::SparseVector<double>> rows;
	    std::vector<double> rhs;
	    rows.reserve(selected.size());
	    rhs.reserve(selected.size());
		    for (const auto& picked : selected) {
		      const PoolCut& cut =
		          root_transformed_cutpool[static_cast<std::size_t>(picked.index)];
		      rows.push_back(cut.coeff);
		      rhs.push_back(cut.rhs);
		    }
	        int selected_payload_nnz = 0;
	        const std::uint64_t selected_payload_hash =
	            xpool_rows_payload_hash(rows, rhs, &selected_payload_nnz);
	        const int pre_append_nnz = static_cast<int>(trial_lp.A.nonZeros());
	        const auto t_append0 = std::chrono::steady_clock::now();
		    add_sparse_rows_to_lp(trial_lp, rows, rhs);
	        const double xpool_append_ms =
	            std::chrono::duration<double, std::milli>(
	                std::chrono::steady_clock::now() - t_append0).count();
	        int appended_payload_nnz = 0;
	        const std::uint64_t appended_payload_hash =
	            xpool_lp_appended_payload_hash(
	                trial_lp, prev_rows,
	                static_cast<int>(trial_lp.A.rows()) - prev_rows,
	                &appended_payload_nnz);
	        if (xpool_ledger_enabled()) {
	          fmt::print(stderr,
	                     "[B&C-XPOOL-APPEND] selected={} preRows={} postRows={} "
	                     "preNnz={} postNnz={} selNnz={} appNnz={} "
	                     "selPayloadHash={:016x} appPayloadHash={:016x} "
	                     "success=1 appendMs={:.6f}\n",
	                     static_cast<int>(selected.size()), prev_rows,
	                     static_cast<int>(trial_lp.A.rows()), pre_append_nnz,
	                     static_cast<int>(trial_lp.A.nonZeros()),
	                     selected_payload_nnz, appended_payload_nnz,
	                     selected_payload_hash, appended_payload_hash,
	                     xpool_append_ms);
	        }
	        const double old_bound = root.bound;
		        if (strict_highs_root_fixed_point) {
		          root_source_trace.xpool_build_ms += root_source_elapsed_ms(t_build0);
		          fmt::print(stderr,
		                     "[B&C ROOT-XPOOL-SKIP] "
		                     "reason=strict_rejects_native_append_resolve\n");
		          rollback_selected_cutpool_lp_pending();
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
	        }
		    apply_node_bounds(trial_lp.vars, root.lb, root.ub);
        root_source_trace.xpool_build_ms += root_source_elapsed_ms(t_build0);

        ++root_source_trace.xpool_resolve_calls;
	        const auto t_solve0 = std::chrono::steady_clock::now();
		    LPRelaxationResult pool_relax;
	        const char* xpool_resolve_path = "unset";
	        int xpool_resolve_iter_delta = -1;
	        StandardFormLP accepted_unscaled_sf;
	        bool accepted_unscaled_sf_valid = false;
        // When a basis hint is available, build SF directly and use the sparse
        // warm-start path with built-in basis extension for a row-dimension
        // mismatch after adding cuts.
        // The basis extension in solve_lp_from_sf handles the row shift when
        // cut rows are inserted before equality rows in the SF.
        const bool use_direct_sf_warm =
            !strict_highs_root_fixed_point &&
            !highs_root_cutpool_original_lp &&
            root.basis_hint &&
            root.basis_hint->rows > 0 &&
            static_cast<int>(root.basis_hint->index_count()) == root.basis_hint->rows;
        if (use_direct_sf_warm) {
          const auto t_sf0 = std::chrono::steady_clock::now();
          static const bool xpool_break_diag =
              bc_env_options().value("MIPSOLVERS_XPOOL_BREAK") != nullptr;
          static const bool xpool_conformance =
              bc_env_options().value("MIPSOLVERS_XPOOL_CONFORM") != nullptr;
          static const bool xpool_incremental_safe =
              xpool_conformance &&
              bc_env_options().value("MIPSOLVERS_XPOOL_INCREMENTAL_SAFE") != nullptr;
	          static const bool xpool_incremental_diag =
	              xpool_conformance &&
	              (bc_env_options().value("MIPSOLVERS_XPOOL_INCREMENTAL") != nullptr ||
	               xpool_incremental_safe);
          static const bool xpool_append_parent_factor =
              bc_env_options().value("MIPSOLVERS_XPOOL_APPEND_PARENT_FACTOR") != nullptr;
          static const bool xpool_append_parent_factor_consume =
              bc_env_options().value("MIPSOLVERS_XPOOL_APPEND_PARENT_FACTOR_CONSUME") != nullptr;
          static const bool xpool_parent_state_diff =
              bc_env_options().value("MIPSOLVERS_XPOOL_PARENT_STATE_DIFF") != nullptr;
          static const bool xpool_exact_dse =
              bc_env_options().value("MIPSOLVERS_XPOOL_EXACT_DSE") != nullptr ||
              bc_env_options().value("MIPSOLVERS_XPOOL_PARENT_EXACT_DSE") != nullptr;
          static const bool xpool_sf_diff_diag =
              bc_env_options().value("MIPSOLVERS_XPOOL_SF_DIFF") != nullptr;
          SimplexOptions sf_opt;
          sf_opt.max_iter = std::max(root_solve_opt.max_lp_iter * 10, 2000);
          sf_opt.feasibility_tol = std::max(1e-10, root_solve_opt.lp_tol * 0.1);
          sf_opt.optimality_tol = std::max(1e-10, root_solve_opt.lp_tol * 0.1);
          sf_opt.verbose = false;
          sf_opt.allow_cold_start = true;
          sf_opt.exact_dse_initialization = xpool_exact_dse;
          sf_opt.use_partial_pricing = false;
          sf_opt.perturb_degenerate_primal = false;
          sf_opt.lp_kernel_backend = root_solve_opt.lp_kernel_backend;
          sf_opt.factor_backend =
              simplex_factor_backend_from_id(root_solve_opt.simplex_factor_backend);
          StandardFormLP trial_sf;
          StandardFormLP trial_unscaled_sf;
          bool incremental_sf = false;
          bool canonical_incremental_sf = false;
          if (xpool_incremental_diag &&
              root_relax.simplex &&
              root_relax.simplex->form.A.rows() == root.basis_hint->rows &&
              root_relax.simplex->form.A.cols() == root.basis_hint->cols &&
              root_first_class_sf_valid &&
              root_first_class_unscaled_sf.A.rows() == root.basis_hint->rows) {
            canonical_incremental_sf = append_leq_rows_to_canonical_standard_form(
                root_first_class_unscaled_sf, rows, rhs, trial_unscaled_sf,
                0, sf_opt.feasibility_tol);
            if (canonical_incremental_sf) {
              trial_sf = trial_unscaled_sf;
              ruiz_scale_standard_form(trial_sf);
              accepted_unscaled_sf = trial_unscaled_sf;
              accepted_unscaled_sf_valid = true;
            }
            incremental_sf = canonical_incremental_sf;
          }
          const auto t_sf_built = std::chrono::steady_clock::now();
          if (!incremental_sf) {
            trial_sf = build_standard_form_lp(trial_lp);
            trial_unscaled_sf = trial_sf;
            const auto t_full_built = std::chrono::steady_clock::now();
            ruiz_scale_standard_form(trial_sf);
            [[maybe_unused]] const double sf_only_build_ms =
                std::chrono::duration<double, std::milli>(
                    t_full_built - t_sf0).count();
            [[maybe_unused]] const double sf_scale_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_full_built).count();
            if (xpool_break_diag) {
              fmt::print(stderr,
                         "[B&C ROOT-XPOOL-DIAG] sf_mode=full sf_build={:.1f}ms "
                         "sf_scale={:.1f}ms ",
                         sf_only_build_ms, sf_scale_ms);
            }
          } else if (xpool_break_diag) {
            const double sf_append_ms =
                std::chrono::duration<double, std::milli>(
                    t_sf_built - t_sf0).count();
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-DIAG] sf_mode={} sf_build={:.1f}ms "
                       "sf_scale=0.0ms ",
                       canonical_incremental_sf ? "canonical_append" : "append",
                       sf_append_ms);
          }
          const bool xpool_parent_factor_ready =
              xpool_append_parent_factor &&
              xpool_append_parent_factor_consume &&
              root_relax.simplex &&
              root_relax.simplex->form.A.rows() == root.basis_hint->rows &&
              root_relax.simplex->form.A.cols() == root.basis_hint->cols &&
              root.basis_hint->cached_sparse_basis &&
              sparse_basis_bound_to_matrix(root.basis_hint->cached_sparse_basis,
                                           root_relax.simplex->form.A);
          sf_opt.allow_incremental_row_append_factor =
              incremental_sf && xpool_parent_factor_ready;
          // Xpool conformance compares the native append/refactor/full pivot
          // state itself.  Keep exact DSE enabled for HiGHS-style edge-weight
          // initialization, but do not post-remap the degenerate frontier on
          // these repeated trial solves; the consume gate below must see the
          // actual pivot frontier.
          sf_opt.enable_degenerate_frontier_remap = false;
          sf_opt.suppress_degenerate_frontier_remap = true;
	          const auto t_solve_only0 = std::chrono::steady_clock::now();
	          auto sf_res = std::make_shared<SimplexResult>(
	              solve_lp_from_sf(trial_sf, sf_opt, root.basis_hint.get()));
	          [[maybe_unused]] const double sf_solve_only_ms =
	              std::chrono::duration<double, std::milli>(
	                  std::chrono::steady_clock::now() - t_solve_only0).count();
	          if (xpool_conformance && incremental_sf &&
	              sf_res->result.stats.success &&
	              static_cast<int>(sf_res->result.x.size()) == n) {
	            SimplexOptions refactor_opt = sf_opt;
	            refactor_opt.allow_incremental_row_append_factor = false;
	            auto append_refactor_res = solve_lp_from_sf(
	                trial_sf, refactor_opt, root.basis_hint.get());
	            StandardFormLP full_sf = build_standard_form_lp(trial_lp);
	            ruiz_scale_standard_form(full_sf);
              auto trace_sf_delta = [&](const StandardFormLP& lhs,
                                        const StandardFormLP& rhs) {
                int row_sign_diff = 0;
                int slack_diff = 0;
                int surplus_diff = 0;
                int artificial_diff = 0;
                int ub_row_diff = 0;
                double b_inf = 0.0;
                double c_inf = 0.0;
                double ub_inf = 0.0;
                double row_scale_inf = 0.0;
                double col_scale_inf = 0.0;
                double a_inf = 0.0;
                const bool same_dims =
                    lhs.A.rows() == rhs.A.rows() &&
                    lhs.A.cols() == rhs.A.cols() &&
                    lhs.b.size() == rhs.b.size() &&
                    lhs.c_max.size() == rhs.c_max.size() &&
                    lhs.var_ub.size() == rhs.var_ub.size();
                if (same_dims) {
                  for (int i = 0; i < lhs.b.size(); ++i) {
                    b_inf = std::max(b_inf, std::abs(lhs.b[i] - rhs.b[i]));
                  }
                  for (int j = 0; j < lhs.c_max.size(); ++j) {
                    c_inf = std::max(c_inf,
                                     std::abs(lhs.c_max[j] - rhs.c_max[j]));
                    const bool lf = std::isfinite(lhs.var_ub[j]);
                    const bool rf = std::isfinite(rhs.var_ub[j]);
                    if (lf != rf) {
                      ub_inf = std::numeric_limits<double>::infinity();
                    } else if (lf) {
                      ub_inf = std::max(
                          ub_inf, std::abs(lhs.var_ub[j] - rhs.var_ub[j]));
                    }
                  }
                  Eigen::SparseMatrix<double> ad =
                      lhs.A.to_eigen() - rhs.A.to_eigen();
                  for (int j = 0; j < ad.outerSize(); ++j) {
                    for (Eigen::SparseMatrix<double>::InnerIterator it(ad, j);
                         it; ++it) {
                      a_inf = std::max(a_inf, std::abs(it.value()));
                    }
                  }
                }
                const int row_n = std::min<int>(
                    static_cast<int>(lhs.row_sign.size()),
                    static_cast<int>(rhs.row_sign.size()));
                for (int i = 0; i < row_n; ++i) {
                  if (lhs.row_sign[static_cast<std::size_t>(i)] !=
                      rhs.row_sign[static_cast<std::size_t>(i)]) {
                    ++row_sign_diff;
                  }
                  if (lhs.row_to_slack_col[static_cast<std::size_t>(i)] !=
                      rhs.row_to_slack_col[static_cast<std::size_t>(i)]) {
                    ++slack_diff;
                  }
                  if (lhs.row_to_surplus_col[static_cast<std::size_t>(i)] !=
                      rhs.row_to_surplus_col[static_cast<std::size_t>(i)]) {
                    ++surplus_diff;
                  }
                  if (lhs.row_to_artificial_col[static_cast<std::size_t>(i)] !=
                      rhs.row_to_artificial_col[static_cast<std::size_t>(i)]) {
                    ++artificial_diff;
                  }
                  if (i < static_cast<int>(lhs.ub_row_var.size()) &&
                      i < static_cast<int>(rhs.ub_row_var.size()) &&
                      lhs.ub_row_var[static_cast<std::size_t>(i)] !=
                          rhs.ub_row_var[static_cast<std::size_t>(i)]) {
                    ++ub_row_diff;
                  }
                }
                if (lhs.row_scale.size() == rhs.row_scale.size()) {
                  for (int i = 0; i < lhs.row_scale.size(); ++i) {
                    row_scale_inf = std::max(
                        row_scale_inf,
                        std::abs(lhs.row_scale[i] - rhs.row_scale[i]));
                  }
                } else {
                  row_scale_inf = std::numeric_limits<double>::infinity();
                }
                if (lhs.col_scale.size() == rhs.col_scale.size()) {
                  for (int j = 0; j < lhs.col_scale.size(); ++j) {
                    col_scale_inf = std::max(
                        col_scale_inf,
                        std::abs(lhs.col_scale[j] - rhs.col_scale[j]));
                  }
                } else {
                  col_scale_inf = std::numeric_limits<double>::infinity();
                }
                fmt::print(stderr,
                           "[B&C ROOT-XPOOL-SF-DIFF] sameDims={} "
                           "lhs={}x{} rhs={}x{} nnz={}/{} "
                           "slack={}/{} surplus={}/{} artificial={}/{} "
                           "rowSignDiff={} slackMapDiff={} surplusMapDiff={} "
                           "artMapDiff={} ubRowDiff={} "
                           "Ainf={:.3e} binf={:.3e} cinf={:.3e} "
                           "ubinf={:.3e} rscale={:.3e} cscale={:.3e}\n",
                           same_dims ? 1 : 0,
                           static_cast<int>(lhs.A.rows()),
                           static_cast<int>(lhs.A.cols()),
                           static_cast<int>(rhs.A.rows()),
                           static_cast<int>(rhs.A.cols()),
                           static_cast<int>(lhs.A.nonZeros()),
                           static_cast<int>(rhs.A.nonZeros()),
                           lhs.n_slack, rhs.n_slack,
                           lhs.n_surplus, rhs.n_surplus,
                           lhs.n_artificial, rhs.n_artificial,
                           row_sign_diff, slack_diff, surplus_diff,
                           artificial_diff, ub_row_diff, a_inf, b_inf, c_inf,
                           ub_inf, row_scale_inf, col_scale_inf);
              };
              if (xpool_sf_diff_diag) {
                trace_sf_delta(trial_sf, full_sf);
              }
	            auto full_res = solve_lp_from_sf(
	                full_sf, refactor_opt, root.basis_hint.get());
	            if (full_res.result.stats.success &&
	                static_cast<int>(full_res.result.x.size()) == n) {
              auto count_frac_like = [&](const Eigen::VectorXd& x) {
                int cnt = 0;
                const double tol = std::max(1e-6, opt.int_tol);
                for (int j = 0; j < n; ++j) {
                  const bool integer_like =
                      root_lp.vars[j].type == VarType::Binary ||
                      root_lp.vars[j].type == VarType::Integer ||
                      (j < static_cast<int>(root_implied_integer_cols.size()) &&
                       root_implied_integer_cols[static_cast<std::size_t>(j)] != 0);
                  if (!integer_like) continue;
                  const double v = x[j];
                  if (std::abs(v - std::round(v)) > tol) ++cnt;
                }
                return cnt;
              };
              auto fractional_like_mask = [&](const Eigen::VectorXd& x) {
                std::vector<char> mask(static_cast<std::size_t>(n), 0);
                const double tol = std::max(1e-6, opt.int_tol);
                for (int j = 0; j < n; ++j) {
                  const bool integer_like =
                      root_lp.vars[j].type == VarType::Binary ||
                      root_lp.vars[j].type == VarType::Integer ||
                      (j < static_cast<int>(root_implied_integer_cols.size()) &&
                       root_implied_integer_cols[static_cast<std::size_t>(j)] != 0);
                  if (!integer_like) continue;
                  const double v = x[j];
                  if (std::abs(v - std::round(v)) > tol) {
                    mask[static_cast<std::size_t>(j)] = 1;
                  }
                }
                return mask;
              };
              auto original_col_status = [&](const SimplexResult& res) {
                std::vector<char> status(static_cast<std::size_t>(n), 'L');
                std::vector<char> basic(static_cast<std::size_t>(n), 0);
                for (int col : res.basis.basis_indices()) {
                  if (col >= 0 && col < n) {
                    basic[static_cast<std::size_t>(col)] = 1;
                  }
                }
                for (int j = 0; j < n; ++j) {
                  const bool fixed =
                      std::isfinite(root_lp.vars[j].lb) &&
                      std::isfinite(root_lp.vars[j].ub) &&
                      root_lp.vars[j].ub - root_lp.vars[j].lb <=
                          std::max(1e-9, opt.int_tol);
                  if (basic[static_cast<std::size_t>(j)]) {
                    status[static_cast<std::size_t>(j)] = 'B';
                  } else if (fixed) {
                    status[static_cast<std::size_t>(j)] = 'Z';
                  } else if (j < static_cast<int>(res.basis.at_upper.size()) &&
                             res.basis.at_upper[static_cast<std::size_t>(j)] != 0) {
                    status[static_cast<std::size_t>(j)] = 'U';
                  } else {
                    status[static_cast<std::size_t>(j)] = 'L';
                  }
                }
                return status;
              };
              struct XpoolConformanceDelta {
                double obj_delta{0.0};
                double x_inf{0.0};
                int frac_lhs{0};
                int frac_rhs{0};
                int frontier_diff{0};
                int status_diff{0};
                int basic_diff{0};
                int fixed_status_diff{0};
                int side_flip{0};
                int degenerate_side_flip{0};
                int frontier_side_flip{0};
                int nondegenerate_status_diff{0};
                int status_after_degenerate_remap{0};
                bool raw_mismatch{true};
                bool degenerate_side_remap_conform{false};
                bool mismatch{true};
              };
              auto compare_xpool_states =
                  [&](const SimplexResult& lhs,
                      const SimplexResult& rhs,
                      double obj_tol) -> XpoolConformanceDelta {
                XpoolConformanceDelta delta;
                const double x_tol = std::max(1e-6, 10.0 * opt.int_tol);
                delta.obj_delta =
                    lhs.result.stats.objective - rhs.result.stats.objective;
                for (int j = 0; j < n; ++j) {
                  delta.x_inf = std::max(
                      delta.x_inf,
                      std::abs(lhs.result.x[j] - rhs.result.x[j]));
                }
                delta.frac_lhs = count_frac_like(lhs.result.x);
                delta.frac_rhs = count_frac_like(rhs.result.x);
                const auto frontier_lhs = fractional_like_mask(lhs.result.x);
                const auto frontier_rhs = fractional_like_mask(rhs.result.x);
                for (int j = 0; j < n; ++j) {
                  if (frontier_lhs[static_cast<std::size_t>(j)] !=
                      frontier_rhs[static_cast<std::size_t>(j)]) {
                    ++delta.frontier_diff;
                  }
                }
                const auto status_lhs = original_col_status(lhs);
                const auto status_rhs = original_col_status(rhs);
                for (int j = 0; j < n; ++j) {
                  const char ls = status_lhs[static_cast<std::size_t>(j)];
                  const char rs = status_rhs[static_cast<std::size_t>(j)];
                  if (ls == rs) continue;
                  ++delta.status_diff;
                  if ((ls == 'B') != (rs == 'B')) {
                    ++delta.basic_diff;
                    continue;
                  }
                  if (ls == 'Z' || rs == 'Z') {
                    ++delta.fixed_status_diff;
                    continue;
                  }
                  const bool side_flip =
                      (ls == 'L' && rs == 'U') || (ls == 'U' && rs == 'L');
                  if (!side_flip) {
                    ++delta.nondegenerate_status_diff;
                    continue;
                  }
                  ++delta.side_flip;
                  const double dx = std::abs(lhs.result.x[j] - rhs.result.x[j]);
                  const bool lhs_rc_zero =
                      j < lhs.reduced_costs.size() &&
                      std::abs(lhs.reduced_costs[j]) <= sf_opt.optimality_tol;
                  const bool rhs_rc_zero =
                      j < rhs.reduced_costs.size() &&
                      std::abs(rhs.reduced_costs[j]) <= sf_opt.optimality_tol;
                  const bool on_same_degenerate_face =
                      dx <= x_tol && lhs_rc_zero && rhs_rc_zero;
                  if (frontier_lhs[static_cast<std::size_t>(j)] != 0 ||
                      frontier_rhs[static_cast<std::size_t>(j)] != 0) {
                    ++delta.frontier_side_flip;
                  }
                  if (on_same_degenerate_face) {
                    ++delta.degenerate_side_flip;
                  } else {
                    ++delta.nondegenerate_status_diff;
                  }
                }
                delta.status_after_degenerate_remap =
                    delta.status_diff - delta.degenerate_side_flip;
                delta.raw_mismatch =
                    std::abs(delta.obj_delta) > obj_tol ||
                    delta.x_inf > x_tol ||
                    delta.frac_lhs != delta.frac_rhs ||
                    delta.frontier_diff != 0 ||
                    delta.status_diff != 0;
                delta.mismatch =
                    std::abs(delta.obj_delta) > obj_tol ||
                    delta.x_inf > x_tol ||
                    delta.frac_lhs != delta.frac_rhs ||
                    delta.frontier_diff != 0 ||
                    delta.status_after_degenerate_remap != 0;
                delta.degenerate_side_remap_conform =
                    delta.raw_mismatch && !delta.mismatch &&
                    delta.status_diff > 0 &&
                    delta.status_diff == delta.degenerate_side_flip;
                return delta;
              };
              auto remap_degenerate_nonbasic_side =
                  [&](SimplexResult& target,
                      const SimplexResult& oracle,
                      double obj_tol) -> int {
                const XpoolConformanceDelta delta =
                    compare_xpool_states(target, oracle, obj_tol);
                if (!delta.degenerate_side_remap_conform) return 0;
                const auto status_target = original_col_status(target);
                const auto status_oracle = original_col_status(oracle);
                int remapped = 0;
                if (target.basis.at_upper.size() < static_cast<std::size_t>(n)) {
                  target.basis.at_upper.resize(static_cast<std::size_t>(n), 0);
                }
                for (int j = 0; j < n; ++j) {
                  const char ts = status_target[static_cast<std::size_t>(j)];
                  const char os = status_oracle[static_cast<std::size_t>(j)];
                  const bool side_flip =
                      (ts == 'L' && os == 'U') || (ts == 'U' && os == 'L');
                  if (!side_flip) continue;
                  const double dx =
                      std::abs(target.result.x[j] - oracle.result.x[j]);
                  const bool target_rc_zero =
                      j < target.reduced_costs.size() &&
                      std::abs(target.reduced_costs[j]) <= sf_opt.optimality_tol;
                  const bool oracle_rc_zero =
                      j < oracle.reduced_costs.size() &&
                      std::abs(oracle.reduced_costs[j]) <= sf_opt.optimality_tol;
                  if (dx > std::max(1e-6, 10.0 * opt.int_tol) ||
                      !target_rc_zero || !oracle_rc_zero) {
                    continue;
                  }
                  target.basis.at_upper[static_cast<std::size_t>(j)] =
                      (os == 'U') ? 1 : 0;
                  ++remapped;
                }
                return remapped;
              };
	              const double obj_tol =
	                  std::max(1e-6, 1e-10 * std::max(
	                                      1.0, std::abs(full_res.result.stats.objective)));
              const XpoolConformanceDelta fast_full =
                  compare_xpool_states(*sf_res, full_res, obj_tol);
              bool append_refactor_ok =
                  append_refactor_res.result.stats.success &&
                  static_cast<int>(append_refactor_res.result.x.size()) == n;
              XpoolConformanceDelta fast_refactor;
              XpoolConformanceDelta refactor_full;
              if (append_refactor_ok) {
                fast_refactor =
                    compare_xpool_states(*sf_res, append_refactor_res, obj_tol);
                refactor_full =
                    compare_xpool_states(append_refactor_res, full_res, obj_tol);
                if (xpool_parent_state_diff &&
                    sf_opt.allow_incremental_row_append_factor) {
                  const auto& fast_basis = sf_res->basis.basis_indices();
                  const auto& ref_basis =
                      append_refactor_res.basis.basis_indices();
                  int basis_diff = 0;
                  const int basis_n = std::min<int>(
                      static_cast<int>(fast_basis.size()),
                      static_cast<int>(ref_basis.size()));
                  for (int i = 0; i < basis_n; ++i) {
                    if (fast_basis[static_cast<std::size_t>(i)] !=
                        ref_basis[static_cast<std::size_t>(i)]) {
                      ++basis_diff;
                    }
                  }
                  basis_diff += std::abs(
                      static_cast<int>(fast_basis.size()) -
                      static_cast<int>(ref_basis.size()));
                  int at_upper_diff = 0;
                  int at_upper_nb_diff = 0;
                  std::vector<char> fast_basic(
                      static_cast<std::size_t>(trial_sf.A.cols()), 0);
                  std::vector<char> ref_basic(
                      static_cast<std::size_t>(trial_sf.A.cols()), 0);
                  for (int col : fast_basis) {
                    if (col >= 0 && col < trial_sf.A.cols()) {
                      fast_basic[static_cast<std::size_t>(col)] = 1;
                    }
                  }
                  for (int col : ref_basis) {
                    if (col >= 0 && col < trial_sf.A.cols()) {
                      ref_basic[static_cast<std::size_t>(col)] = 1;
                    }
                  }
                  const int side_n = std::min<int>(
                      static_cast<int>(sf_res->basis.at_upper.size()),
                      static_cast<int>(append_refactor_res.basis.at_upper.size()));
                  for (int j = 0; j < side_n; ++j) {
                    if (sf_res->basis.at_upper[static_cast<std::size_t>(j)] !=
                        append_refactor_res.basis.at_upper[
                            static_cast<std::size_t>(j)]) {
                      ++at_upper_diff;
                      if (!fast_basic[static_cast<std::size_t>(j)] &&
                          !ref_basic[static_cast<std::size_t>(j)]) {
                        ++at_upper_nb_diff;
                      }
                    }
                  }
                  double rc_inf = 0.0;
                  int rc_sign_diff = 0;
                  const int rc_n = std::min<int>(
                      static_cast<int>(sf_res->reduced_costs.size()),
                      static_cast<int>(append_refactor_res.reduced_costs.size()));
                  for (int j = 0; j < rc_n; ++j) {
                    const double lhs = sf_res->reduced_costs[j];
                    const double rhs = append_refactor_res.reduced_costs[j];
                    rc_inf = std::max(rc_inf, std::abs(lhs - rhs));
                    const int lhs_sign =
                        (lhs > sf_opt.optimality_tol)
                            ? 1
                            : (lhs < -sf_opt.optimality_tol) ? -1 : 0;
                    const int rhs_sign =
                        (rhs > sf_opt.optimality_tol)
                            ? 1
                            : (rhs < -sf_opt.optimality_tol) ? -1 : 0;
                    if (lhs_sign != rhs_sign) ++rc_sign_diff;
                  }
                  double xb_inf = 0.0;
                  const int xb_n = std::min<int>(
                      static_cast<int>(sf_res->x_basic.size()),
                      static_cast<int>(append_refactor_res.x_basic.size()));
                  for (int i = 0; i < xb_n; ++i) {
                    xb_inf = std::max(
                        xb_inf,
                        std::abs(sf_res->x_basic[i] -
                                 append_refactor_res.x_basic[i]));
                  }
                  fmt::print(stderr,
                             "[B&C ROOT-XPOOL-PARENT-STATE-DIFF] rows={} "
                             "selected={} basis={} atUpper={} atUpperNB={} "
                             "rcInf={:.3e} rcSign={} xbInf={:.3e} "
                             "iterFast={} iterRef={} objFast={:.12g} "
                             "objRef={:.12g}\n",
                             static_cast<int>(trial_lp.A.rows()),
                             static_cast<int>(selected.size()),
                             basis_diff, at_upper_diff, at_upper_nb_diff,
                             rc_inf, rc_sign_diff, xb_inf,
                             sf_res->result.stats.iterations,
                             append_refactor_res.result.stats.iterations,
                             sf_res->result.stats.objective,
                             append_refactor_res.result.stats.objective);
                }
                const char* split =
                    fast_refactor.mismatch
                        ? "factor_or_live_state"
                        : (refactor_full.mismatch
                               ? "stable_scaling_or_degenerate_vertex"
                               : "append_refactor_matches_full");
                fmt::print(stderr,
                           "[B&C ROOT-XPOOL-CONFORM-SPLIT] rows={} selected={} "
                           "fast_ref:dobj={:.3e}:frontier{}:status{}:"
                           "side{}:degSide{}:post{}:xinf{:.3e} "
                           "ref_full:dobj={:.3e}:frontier{}:status{}:"
                           "side{}:degSide{}:post{}:xinf{:.3e} "
                           "iterFast={} iterRef={} hintFast={}/{} hintRef={}/{} "
                           "diagnosis={}\n",
                           static_cast<int>(trial_lp.A.rows()),
                           static_cast<int>(selected.size()),
                           fast_refactor.obj_delta,
                           fast_refactor.frontier_diff,
                           fast_refactor.status_diff,
                           fast_refactor.side_flip,
                           fast_refactor.degenerate_side_flip,
                           fast_refactor.status_after_degenerate_remap,
                           fast_refactor.x_inf,
                           refactor_full.obj_delta,
                           refactor_full.frontier_diff,
                           refactor_full.status_diff,
                           refactor_full.side_flip,
                           refactor_full.degenerate_side_flip,
                           refactor_full.status_after_degenerate_remap,
                           refactor_full.x_inf,
                           sf_res->result.stats.iterations,
                           append_refactor_res.result.stats.iterations,
                           sf_res->solved_from_hint ? 1 : 0,
                           sf_res->dual_reoptimized ? 1 : 0,
                           append_refactor_res.solved_from_hint ? 1 : 0,
                           append_refactor_res.dual_reoptimized ? 1 : 0,
                           split);
              } else {
                fmt::print(stderr,
                           "[B&C ROOT-XPOOL-CONFORM-SPLIT] rows={} selected={} "
                           "append_refactor_failed status={}\n",
                           static_cast<int>(trial_lp.A.rows()),
                           static_cast<int>(selected.size()),
                           append_refactor_res.result.stats.status);
              }
              int remapped_fast_full_side = 0;
              if (fast_full.degenerate_side_remap_conform) {
                remapped_fast_full_side =
                    remap_degenerate_nonbasic_side(*sf_res, full_res, obj_tol);
              }
              const XpoolConformanceDelta fast_full_after_remap =
                  remapped_fast_full_side > 0
                      ? compare_xpool_states(*sf_res, full_res, obj_tol)
                      : fast_full;
	              const bool frontier_mismatch = fast_full_after_remap.mismatch;
	              const bool consume_append =
	                  xpool_incremental_safe && !frontier_mismatch;
	              fmt::print(stderr,
	                         "[B&C ROOT-XPOOL-CONFORM] rows={} selected={} "
	                         "obj_append={:.12g} obj_full={:.12g} dobj={:.3e} "
	                         "frac_append={} frac_full={} frontier_diff={} "
	                         "status_diff={} side_flip={} deg_side={} "
                             "frontier_side={} basic_diff={} fixed_diff={} "
                             "nondeg_status={} post_remap_status={} "
                             "remapped={} xinf={:.3e} "
	                         "action={}\n",
	                         static_cast<int>(trial_lp.A.rows()),
	                         static_cast<int>(selected.size()),
		                         sf_res->result.stats.objective,
		                         full_res.result.stats.objective,
                             fast_full_after_remap.obj_delta,
		                         fast_full_after_remap.frac_lhs,
                             fast_full_after_remap.frac_rhs,
                             fast_full_after_remap.frontier_diff,
		                         fast_full.status_diff, fast_full.side_flip,
                             fast_full.degenerate_side_flip,
                             fast_full.frontier_side_flip,
                             fast_full.basic_diff,
                             fast_full.fixed_status_diff,
                             fast_full.nondegenerate_status_diff,
                             fast_full_after_remap.status_after_degenerate_remap,
                             remapped_fast_full_side,
                             fast_full_after_remap.x_inf,
		                         consume_append
                                     ? "consume_append_exact"
                                     : (frontier_mismatch
                                            ? "consume_full_mismatch"
                                            : "consume_full_safe_disabled"));
              if (!consume_append) {
                auto full_shared =
                    std::make_shared<SimplexResult>(std::move(full_res));
                full_shared->form = std::move(full_sf);
                sf_res = std::move(full_shared);
                trial_sf = sf_res->form;
                incremental_sf = false;
                accepted_unscaled_sf_valid = false;
              }
	            } else {
	              fmt::print(stderr,
	                         "[B&C ROOT-XPOOL-CONFORM] full_rebuild_failed "
	                         "status={}\n",
	                         full_res.result.stats.status);
              incremental_sf = false;
              accepted_unscaled_sf_valid = false;
              trial_sf = build_standard_form_lp(trial_lp);
              ruiz_scale_standard_form(trial_sf);
              sf_opt.allow_incremental_row_append_factor = false;
              sf_res = std::make_shared<SimplexResult>(
                  solve_lp_from_sf(trial_sf, sf_opt, root.basis_hint.get()));
            }
          }
          if (xpool_break_diag) {
            fmt::print(stderr,
                       "solve={:.1f}ms iters={} rows={} cols={} basis_rows={}\n",
                       sf_solve_only_ms,
                       sf_res->result.stats.iterations,
                       static_cast<int>(trial_lp.A.rows()),
                       static_cast<int>(trial_lp.vars.size()),
                       root.basis_hint ? root.basis_hint->rows : -1);
          }
	          sf_res->form = trial_sf;  // keep SF for Gomory cut generation
	          xpool_resolve_path =
	              incremental_sf
	                  ? (sf_opt.allow_incremental_row_append_factor
	                         ? "direct_sf_incremental_factor"
	                         : "direct_sf_incremental_refactor")
	                  : "direct_sf_full";
	          xpool_resolve_iter_delta = sf_res->result.stats.iterations;
	          if (sf_res->basis.cached_sparse_basis) {
            rebind_sparse_basis_matrix(sf_res->basis.cached_sparse_basis,
                                       sf_res->form.A);
          }
          if (sf_res->result.stats.success &&
              static_cast<int>(sf_res->result.x.size()) == n) {
            pool_relax.primal = sf_res->result;
            pool_relax.dual_bound = pool_relax.primal.stats.objective;
            pool_relax.simplex = sf_res;
            if (sf_res->result.constraint_duals.size() > 0) {
              pool_relax.row_duals = sf_res->result.constraint_duals;
            }
            auto bh = std::make_shared<SimplexBasis>(sf_res->basis);
            bh->sf_n_slack = trial_sf.n_slack;
            bh->sf_n_surplus = trial_sf.n_surplus;
            bh->sf_n_artificial = trial_sf.n_artificial;
            if (sf_res->reduced_costs.size() > 0) {
              bh->cached_reduced_costs =
                  std::make_shared<const Eigen::VectorXd>(sf_res->reduced_costs);
            }
            if (sf_res->basis.cached_sparse_basis) {
              bh->cached_sparse_basis = sf_res->basis.cached_sparse_basis;
            }
            pool_relax.basis_hint = bh;
	          } else {
	            // Warm-start failed — fall back to full solve with IPM
	            pool_relax = solve_lp_relaxation(
	                trial_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
	            xpool_resolve_path = "fallback_solve_lp_relaxation";
	            xpool_resolve_iter_delta = pool_relax.primal.stats.iterations;
	          }
	        } else {
		      pool_relax = solve_lp_relaxation(
		          trial_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
	          xpool_resolve_path =
	              pool_relax.primal.stats.solver_name ==
	                      "VendoredHighsLiveAppendLpKernel"
	                  ? "vendored_highs_live_append"
	                  : "solve_lp_relaxation";
	          xpool_resolve_iter_delta = pool_relax.primal.stats.iterations;
	        }
	        const double xpool_resolve_ms = root_source_elapsed_ms(t_solve0);
	        root_source_trace.xpool_solve_ms += xpool_resolve_ms;
	        xpool_log_resolve_ledger(
	            xpool_resolve_path, trial_lp, pool_relax, pool_relax.primal.x,
	            pool_relax.basis_hint.get(), xpool_resolve_iter_delta,
	            xpool_resolve_ms);
	        if (strict_highs_root_fixed_point &&
	            std::strcmp(xpool_resolve_path,
	                        "vendored_highs_live_append") != 0) {
	          if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL] rejected selected={} rows={} "
	                       "reason=strict_non_live_append_resolve path={}\n",
	                       static_cast<int>(selected.size()),
	                       static_cast<int>(trial_lp.A.rows()) - prev_rows,
	                       xpool_resolve_path);
	          }
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          rollback_selected_cutpool_lp_pending();
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
	        }
		    ++out.bc_stats.lp_solves;
	    if (!pool_relax.primal.stats.success ||
	        pool_relax.primal.x.size() != n ||
	        !std::isfinite(pool_relax.dual_bound) ||
	        pool_relax.dual_bound < old_bound -
	            std::max(1e-6, 1e-9 * std::max(1.0, std::abs(old_bound)))) {
	      if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	        fmt::print(stderr,
	                   "[B&C ROOT-XPOOL] rejected selected={} rows={} "
	                   "old_bound={:.2f} status={}\n",
	                   static_cast<int>(selected.size()),
	                   static_cast<int>(trial_lp.A.rows()) - prev_rows,
	                   old_bound, pool_relax.primal.stats.status);
	          }
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          rollback_selected_cutpool_lp_pending();
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
		      return result;
		    }
	
	        ++root_source_trace.xpool_resolve_ok;
	        const double lift = pool_relax.dual_bound - old_bound;
	        result.lp_resolved = true;
	        result.lift = lift;
	        const double min_lift =
	            std::max(opt.root_cut_min_bound_lift,
	                     1e-10 * std::max(1.0, std::abs(old_bound)));
        const bool highs_cutpool_round =
            opt.auto_highs_root_pipeline && opt.cuts != CutType::None;
        if (!highs_cutpool_round &&
            opt.root_cut_reject_nonmoving_rows && lift <= min_lift) {
          out.bc_stats.root_cut_rows_rejected_nonmoving +=
              static_cast<std::uint64_t>(trial_lp.A.rows() - prev_rows);
          ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
          if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL] rejected nonmoving selected={} rows={} "
                       "bound={:.2f}->{:.2f} lift={:.3e} pool={} avail={} "
                       "active={}\n",
                       static_cast<int>(selected.size()),
                       static_cast<int>(trial_lp.A.rows()) - prev_rows,
                       old_bound, pool_relax.dual_bound, lift,
                       static_cast<int>(root_transformed_cutpool.size()),
                       pool_stats.available, pool_stats.active);
	          }
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          rollback_selected_cutpool_lp_pending();
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
	        }

	        const auto t_apply0 = std::chrono::steady_clock::now();
	        const int admitted_before_aging =
	            static_cast<int>(trial_lp.A.rows()) - prev_rows;
	        int selected_row_offset = 0;
				    for (const auto& picked : selected) {
				      if (picked.index >= 0 &&
				          picked.index < static_cast<int>(root_transformed_cut_in_lp.size())) {
				        if (selected_row_offset < admitted_before_aging) {
				          root_transformed_lp_cut_rows.push_back(
				              RootTransformedLpCutRow{
				                  prev_rows + selected_row_offset,
				                  picked.index, 0, true});
				        }
				      }
				      ++selected_row_offset;
				    }
		    root_lp = std::move(trial_lp);
		    capture_variable_bound_cutpool_rows(root_lp, prev_rows,
		                                        static_cast<int>(root_lp.A.rows()),
		                                        "after_root_xpool");
		    root_relax = std::move(pool_relax);
		    root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
		    root.x_seed = root.x_relax;
		    refresh_root_lp_source_solution();
		    root.basis_hint = root_relax.basis_hint;
	    if (root.basis_hint) root.basis_hint->compact_indices_storage();
        if (maintain_native_xpool_sf_cache && accepted_unscaled_sf_valid) {
          root_first_class_unscaled_sf = std::move(accepted_unscaled_sf);
          root_first_class_sf_valid = true;
        } else if (maintain_native_xpool_sf_cache && root_relax.simplex &&
                   root_relax.simplex->form.A.rows() > 0) {
          root_first_class_unscaled_sf = build_standard_form_lp(root_lp);
          root_first_class_sf_valid = true;
        } else {
          root_first_class_sf_valid = false;
        }
			    root.bound = root_relax.dual_bound;
			    const int aged_lp_cut_rows =
			        perform_root_transformed_lp_cut_aging(
			            "after_root_xpool_resolve");
			    refresh_root_redcost_lurking_bounds("root_xpool_resolve");
			    const StandardFormLP* xpool_trace_sf =
		        root_relax.simplex ? &root_relax.simplex->form : nullptr;
		    trace_native_frontier_conformance(
			        "after_root_xpool", root_lp, xpool_trace_sf,
			        root_implied_integer_cols, root.x_relax, root.lb, root.ub,
			        root.basis_hint.get(), root.bound, highs_mip_feastol);
        ++root_source_epoch;
        root_xpool_noop_epoch = 0;
        root_xpool_noop_pool_size = std::numeric_limits<std::size_t>::max();
	    if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
	      root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
	    } else {
	      root_sbasis.reset();
	    }
			    const int admitted = admitted_before_aging;
	        result.admitted_rows = admitted;
	        root_source_trace.xpool_rows += admitted;
		    out.bc_stats.cuts_added += admitted;
	    out.bc_stats.root_cuts_added += admitted;
		    out.bc_stats.root_gomory_cuts += admitted;
	    const double final_lift = root.bound - old_bound;
	    result.lift = final_lift;
	    if (final_lift > opt.root_cut_min_bound_lift) {
	      out.bc_stats.root_cut_bound_lift += std::max(0.0, final_lift);
	    }
	    fmt::print(stderr,
	               "[B&C ROOT-XPOOL] cuts={} rows={} agedDeleted={} "
	               "bound={:.2f}->{:.2f} lift={:.3e} pool={} avail={} "
	               "active={}\n",
	               static_cast<int>(selected.size()), admitted,
	               aged_lp_cut_rows, old_bound, root.bound, final_lift,
	               static_cast<int>(root_transformed_cutpool.size()),
	               pool_stats.available, pool_stats.active);
	        root_source_trace.xpool_apply_ms += root_source_elapsed_ms(t_apply0);
	        root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	        return result;
			  };

  const bool trace_root_source_loop =
      opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
      bc_env_options().value("MIPSOLVERS_BC_TIMELINE") != nullptr;

	  Eigen::VectorXd pending_highs_root_primal;
	  Eigen::VectorXd pending_highs_root_primal_original;
	  bool strict_highs_evaluate_root_imported = false;
	  const bool strict_highs_evaluate_root_enabled =
	      strict_highs_root_fixed_point &&
	      !bc_env_options().strict_root_disable_highs_evaluate_root;
	  if (strict_highs_evaluate_root_enabled) {
	    const auto t_eval_root0 = std::chrono::steady_clock::now();
	    auto highs_root_owner = std::make_shared<Highs>();
	    root_xpool_authority_highs_owner = highs_root_owner;
	    root_xpool_authority_lpsolver = nullptr;
	    highs_root_owner->setOptionValue("output_flag", false);
	    highs_root_owner->setOptionValue("log_to_console", false);
	    highs_root_owner->setOptionValue("mip_rel_gap", opt.gap_tol);
	    // Compositional root ownership requires the owner's final root
	    // coordinates to stay isomorphic to native's fixed working columns.
	    // Every root re-presolve is gated on mip_allow_restart
	    // (HighsMipSolverData::evaluateRootNode restart sites), so disabling
	    // it pins the preflight coordinate system by construction; the
	    // incompatible-columns branch below is then a genuine invariant
	    // violation. Scope cut (represolve-based restart strength) and sweep
	    // evidence: docs/native_milp_root_quality_restart_prerequisites_
	    // 2026-08-13.md, "Strict-Root Coordinate Pinning".
	    highs_root_owner->setOptionValue("mip_allow_restart", false);
	    bool strict_highs_eval_ok =
	        bc_pass_mip_model_to_highs(*highs_root_owner,
	                                   strict_highs_original_entry_lp, nullptr,
	                                   nullptr);
	    std::string strict_highs_eval_error;
	    if (!strict_highs_eval_ok) {
	      strict_highs_eval_error = "pass_model_failed";
	    } else {
	      root_xpool_live_oracle = RootHighsLiveXpoolOracle{};
	      root_xpool_authority_highs_owner = highs_root_owner;
	      std::shared_ptr<Highs> strict_live_owner = highs_root_owner;
	      auto strict_live_callback =
	          [&](const char* event, const HighsLpRelaxation* lp,
	              const HighsCutSet* cutset, void*) {
	        const std::string ev = event != nullptr ? event : "";
	        auto& oracle = root_xpool_live_oracle;
	        if (ev == "select" || ev == "select_restart") {
	          RootHighsLiveXpoolRound round;
	          round.select_event = ev;
	          round.highs_owner = strict_live_owner;
	          if (cutset != nullptr) {
	            round.selected = root_xpool_cutset_payload_signature(*cutset);
	          }
	          round.have_select = true;
	          oracle.rounds.push_back(std::move(round));
	          const auto& r = oracle.rounds.back();
	          fmt::print(stderr,
	                     "[B&C-XPOOL-LIVE-SELECT] event={} round={} "
	                     "selected={} nnz={} payloadHash={:016x}\n",
	                     r.select_event,
	                     static_cast<int>(oracle.rounds.size()) - 1,
	                     r.selected.rows, r.selected.nnz, r.selected.hash);
	          return;
	        }
	        if (ev == "append") {
	          if (oracle.rounds.empty() || oracle.rounds.back().have_append) {
	            RootHighsLiveXpoolRound round;
	            round.select_event = "missing_select";
	            round.highs_owner = strict_live_owner;
	            oracle.rounds.push_back(std::move(round));
	          }
	          RootHighsLiveXpoolRound& round = oracle.rounds.back();
	          round.highs_owner = strict_live_owner;
	          if (lp != nullptr) {
	            const Highs& h = lp->getLpSolver();
	            const HighsLp& h_lp = h.getLp();
	            const RootXpoolPayloadSignature selected_sig =
	                cutset != nullptr ? root_xpool_cutset_payload_signature(*cutset)
	                                  : RootXpoolPayloadSignature{};
	            const int selected_rows =
	                cutset != nullptr ? static_cast<int>(cutset->numCuts()) : 0;
	            const int post_rows = static_cast<int>(h_lp.num_row_);
	            const int pre_rows = post_rows - selected_rows;
	            const RootXpoolPayloadSignature appended_sig =
	                root_xpool_lp_row_payload_signature(h_lp, pre_rows,
	                                                    selected_rows);
	            round.append = RootHighsXpoolAppendReplay{
	                selected_rows,
	                pre_rows,
	                post_rows,
	                static_cast<int>(h_lp.a_matrix_.numNz()) - appended_sig.nnz,
	                static_cast<int>(h_lp.a_matrix_.numNz()),
	                selected_sig.nnz,
	                appended_sig.nnz,
	                selected_sig.hash,
	                appended_sig.hash,
	                1,
	                0.0};
	            round.append_iter_before =
	                static_cast<int>(std::min<int64_t>(
	                    lp->getNumLpIterations(),
	                    std::numeric_limits<int>::max()));
	            round.lpsolver = const_cast<Highs*>(&h);
	          }
	          round.have_append = true;
	          round.append_verified =
	              round.have_select &&
	              round.selected.rows == round.append.selected &&
	              round.selected.nnz == round.append.selected_nnz &&
	              round.selected.hash == round.append.selected_payload_hash &&
	              round.append.selected ==
	                  round.append.post_rows - round.append.pre_rows &&
	              round.append.appended_payload_hash ==
	                  round.append.selected_payload_hash;
	          if (!round.append_verified) {
	            round.verify_error = "callback_selected_append_mismatch";
	            oracle.ok = false;
	            oracle.error = round.verify_error;
	          }
	          fmt::print(stderr,
	                     "[B&C-XPOOL-LIVE-APPEND] round={} selected={} "
	                     "preRows={} postRows={} preNnz={} postNnz={} "
	                     "selNnz={} appNnz={} selPayloadHash={:016x} "
	                     "appPayloadHash={:016x} verified={}\n",
	                     static_cast<int>(oracle.rounds.size()) - 1,
	                     round.append.selected, round.append.pre_rows,
	                     round.append.post_rows, round.append.pre_nnz,
	                     round.append.post_nnz, round.append.selected_nnz,
	                     round.append.appended_nnz,
	                     round.append.selected_payload_hash,
	                     round.append.appended_payload_hash,
	                     round.append_verified ? 1 : 0);
	          return;
	        }
	        if (ev == "resolve") {
	          if (oracle.rounds.empty()) {
	            RootHighsLiveXpoolRound round;
	            round.select_event = "missing_select";
	            round.highs_owner = strict_live_owner;
	            oracle.rounds.push_back(std::move(round));
	          }
	          RootHighsLiveXpoolRound& round = oracle.rounds.back();
	          round.highs_owner = strict_live_owner;
	          if (lp != nullptr) {
	            const int iter_before =
	                round.have_resolve ? round.resolve.iter_total
	                                   : round.append_iter_before;
	            round.resolve = root_highs_live_state_from_lp(*lp);
	            round.resolve.iter_delta =
	                std::max(0, round.resolve.iter_total - iter_before);
	            round.lpsolver = const_cast<Highs*>(&lp->getLpSolver());
	            root_highs_collect_live_basis(*lp, round);
	            const RootXpoolStateSignature live_sig =
	                root_xpool_state_signature_from_highs(
	                    root_lp, lp->getLpSolver().getSolution(),
	                    lp->getLpSolver().getBasis(), n,
	                    root_implied_integer_cols, highs_mip_feastol);
	            round.resolve_verified =
	                round.append_verified &&
	                live_sig.fractional == round.resolve.frac &&
	                live_sig.frontier_hash == round.resolve.frontier_hash &&
	                live_sig.status_hash == round.resolve.status_hash &&
	                live_sig.basis_hash == round.resolve.basis_hash &&
	                live_sig.basic_cols == round.resolve.basic_cols &&
	                live_sig.lower_cols == round.resolve.lower_cols &&
	                live_sig.upper_cols == round.resolve.upper_cols &&
	                live_sig.zero_cols == round.resolve.zero_cols &&
	                live_sig.other_cols == round.resolve.other_cols;
	            if (!round.resolve_verified) {
	              round.verify_error = round.append_verified
	                                       ? "callback_live_hash_mismatch"
	                                       : "callback_append_not_verified";
	              oracle.ok = false;
	              oracle.error = round.verify_error;
	            }
	          }
	          round.have_resolve = true;
	          ++round.resolve_events;
	          fmt::print(stderr,
	                     "[B&C-XPOOL-LIVE-RESOLVE] round={} event={} "
	                     "status={} obj={:.17g} rows={} cols={} nnz={} "
	                     "iterDelta={} iterTotal={} frac={} "
	                     "frontierHash={:016x} statusHash={:016x} "
	                     "basisHash={:016x} verified={}\n",
	                     static_cast<int>(oracle.rounds.size()) - 1,
	                     round.resolve_events, round.resolve.status,
	                     round.resolve.obj, round.resolve.rows,
	                     round.resolve.cols, round.resolve.nnz,
	                     round.resolve.iter_delta, round.resolve.iter_total,
	                     round.resolve.frac, round.resolve.frontier_hash,
	                     round.resolve.status_hash, round.resolve.basis_hash,
	                     round.resolve_verified ? 1 : 0);
	        }
	      };
	      highs_root_owner->hacdcpfSetRootLpRelaxationCallback(
	          strict_live_callback, nullptr);
	      fmt::print(stderr,
	                 "[B&C-HIGHS-EVALUATE-ROOT-IN] src=original rows={} "
	                 "cols={} nnz={} rootRows={} rootCols={} rootNnz={}\n",
	                 static_cast<int>(strict_highs_original_entry_lp.A.rows()) +
	                     static_cast<int>(
	                         strict_highs_original_entry_lp.Aeq.rows()),
	                 static_cast<int>(
	                     strict_highs_original_entry_lp.vars.size()),
	                 static_cast<int>(
	                     strict_highs_original_entry_lp.A.nonZeros()) +
	                     static_cast<int>(
	                         strict_highs_original_entry_lp.Aeq.nonZeros()),
	                 static_cast<int>(root_lp.A.rows()) +
	                     static_cast<int>(root_lp.Aeq.rows()),
	                 static_cast<int>(root_lp.vars.size()),
	                 static_cast<int>(root_lp.A.nonZeros()) +
	                     static_cast<int>(root_lp.Aeq.nonZeros()));
	      (void)highs_root_owner->setOptionValue(
	          "hacdcpf_retain_root_lp_relaxation", true);
	      (void)highs_root_owner->setOptionValue(
	          "hacdcpf_root_oracle_stop_after_evaluate_root", true);
	      const bool stop_after_root_round =
	          bc_env_options().strict_root_stop_after_round;
	      (void)highs_root_owner->setOptionValue(
	          "hacdcpf_root_oracle_stop_after_root_round",
	          stop_after_root_round);
	      if (opt.time_limit_sec > 0.0 &&
	          std::isfinite(opt.time_limit_sec)) {
	        // Every nested root owner inherits the global deadline less the
	        // existing finalization reserve. Derivation and bounded-overrun
	        // contract: docs/archive/native_milp_cplex_gap_improvement_
	        // 2026-09-12.md, "Pre-registered rationale".
	        const double root_owner_budget_sec = std::max(
	            0.001,
	            bc_remaining_sec() - bc_post_root_finalize_reserve_sec);
	        (void)highs_root_owner->setOptionValue("time_limit",
	                                                root_owner_budget_sec);
	      }
	      const HighsStatus run_status = highs_root_owner->run();
	      root_xpool_authority_lpsolver =
	          highs_root_owner->hacdcpfRootLpSolver();
	      for (const RootHighsLiveXpoolRound& round :
	           root_xpool_live_oracle.rounds) {
	        if (!round.have_select || !round.have_append ||
	            !round.have_resolve || !round.append_verified ||
	            !round.resolve_verified) {
	          root_xpool_live_oracle.ok = false;
	          root_xpool_live_oracle.error =
	              !round.verify_error.empty() ? round.verify_error
	                                          : "incomplete_live_round";
	          break;
	        }
	      }
	      fmt::print(stderr,
	                 "[B&C-XPOOL-LIVE-ORACLE] ok={} rounds={} "
	                 "rootLpSolver={} error={}\n",
	                 root_xpool_live_oracle.ok ? 1 : 0,
	                 static_cast<int>(root_xpool_live_oracle.rounds.size()),
	                 root_xpool_authority_lpsolver != nullptr ? 1 : 0,
	                 root_xpool_live_oracle.error.empty()
	                     ? "none"
	                     : root_xpool_live_oracle.error);
	      if (run_status != HighsStatus::kOk &&
	          root_xpool_authority_lpsolver == nullptr) {
	        strict_highs_eval_ok = false;
	        strict_highs_eval_error =
	            fmt::format("run_status_{}", static_cast<int>(run_status));
	      }
	    }
	    if (strict_highs_eval_ok &&
	        root_xpool_authority_lpsolver != nullptr) {
	      const HighsLpRelaxation* final_lprelax =
	          highs_root_owner->hacdcpfRootLpRelaxation();
	      const HighsLp& final_highs_lp =
	          root_xpool_authority_lpsolver->getLp();
	      LPModel final_root_lp =
	          root_native_lp_from_highs_lp(final_highs_lp, root_lp.sense);
	      if (bc_env_options().root_oracle_stop_after_root_round ||
	          strict_highs_evaluate_root_enabled) {
	        const int keep_cols = std::min(
	            static_cast<int>(final_root_lp.vars.size()),
	            static_cast<int>(root_lp.vars.size()));
	        for (int j = 0; j < keep_cols; ++j) {
	          final_root_lp.vars[static_cast<std::size_t>(j)].lb =
	              root_lp.vars[static_cast<std::size_t>(j)].lb;
	          final_root_lp.vars[static_cast<std::size_t>(j)].ub =
	              root_lp.vars[static_cast<std::size_t>(j)].ub;
	          final_root_lp.vars[static_cast<std::size_t>(j)].type =
	              root_lp.vars[static_cast<std::size_t>(j)].type;
	          final_root_lp.vars[static_cast<std::size_t>(j)].name =
	              root_lp.vars[static_cast<std::size_t>(j)].name;
	          if (j < root_lp.c.size() && j < final_root_lp.c.size()) {
	            final_root_lp.c[j] = root_lp.c[j];
	          }
	        }
	      }
	      fmt::print(stderr,
	                 "[B&C-HIGHS-EVALUATE-ROOT-OUT] rows={} cols={} nnz={} "
	                 "offset={:.17g}\n",
	                 static_cast<int>(final_highs_lp.num_row_),
	                 static_cast<int>(final_highs_lp.num_col_),
	                 static_cast<int>(final_highs_lp.a_matrix_.numNz()),
	                 final_highs_lp.offset_);
	      const bool compatible_working_columns =
	          static_cast<int>(final_highs_lp.num_col_) == n;
	      if (!compatible_working_columns) {
	        // HiGHS root restarts may represolve into a different coordinate
	        // system. Native domain/tree state is fixed at n columns, so import
	        // is transactional and rolls back before any oracle state commits.
	        // Achterberg (2007), Sec. 10.1; derivation and p200 mismatch in
	        // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
	        strict_highs_eval_error = fmt::format(
	            "incompatible_root_columns_{}_native_{}",
	            static_cast<int>(final_highs_lp.num_col_), n);
	        out.stats.status = bc_status::kHighsPresolvedWorkingLpMismatch;
	        return return_with_profile();
	      } else if (auto final_round = root_highs_final_state_round(
	                     highs_root_owner, final_lprelax, final_root_lp)) {
	        root_xpool_authority_highs_owner = highs_root_owner;
	        const bool imported = import_root_highs_oracle_state(
	            *final_round, final_root_lp, root.lb, root.ub);
	        strict_highs_evaluate_root_imported = imported;
	        if (!imported) {
	          strict_highs_eval_error = "state_import_failed";
	        } else {
	          const HighsSolution& root_solution = highs_root_owner->getSolution();
	          if (root_solution.value_valid &&
	              static_cast<int>(root_solution.col_value.size()) == orig_n) {
	            Eigen::Map<const Eigen::VectorXd> original_root_primal(
	                root_solution.col_value.data(), orig_n);
	            // HiGHS::run() exposes the incumbent after postsolve. The exact
	            // inverse affine map and native audit are specified in
	            // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
	            // "Coupled HiGHS Root-Primal Ownership".
	            pending_highs_root_primal = highs_presolve_forward_map(
	                highs_presolve_side_state, orig_n, original_root_primal);
	            if (pending_highs_root_primal.size() != n) {
	              pending_highs_root_primal.resize(0);
	            } else {
	              pending_highs_root_primal_original = original_root_primal;
	            }
	          }
	        }
	      } else {
	        strict_highs_eval_error = "final_state_incomplete";
	      }
	    }
	    const auto t_eval_root1 = std::chrono::steady_clock::now();
	    const double eval_root_ms =
	        std::chrono::duration<double, std::milli>(
	            t_eval_root1 - t_eval_root0)
	            .count();
	    fmt::print(stderr,
	               "[B&C-HIGHS-EVALUATE-ROOT] ok={} rows={} cols={} "
	               "nnz={} bound={:.17g} frac={} frontierHash={:016x} "
	               "basisHash={:016x} ms={:.3f} error={}\n",
	               strict_highs_evaluate_root_imported ? 1 : 0,
	               static_cast<int>(root_lp.A.rows()),
	               static_cast<int>(root_lp.vars.size()),
	               static_cast<int>(root_lp.A.nonZeros()), root.bound,
	               strict_highs_evaluate_root_imported
	                   ? count_fractional_source_integers()
	                   : -1,
	               strict_highs_evaluate_root_imported
	                   ? root_xpool_state_signature_from_highs(
	                         root_lp,
	                         root_xpool_authority_lpsolver->getSolution(),
	                         root_xpool_authority_lpsolver->getBasis(), n,
	                         root_implied_integer_cols, highs_mip_feastol)
	                         .frontier_hash
	                   : 0,
	               strict_highs_evaluate_root_imported
	                   ? root_xpool_state_signature_from_highs(
	                         root_lp,
	                         root_xpool_authority_lpsolver->getSolution(),
	                         root_xpool_authority_lpsolver->getBasis(), n,
	                         root_implied_integer_cols, highs_mip_feastol)
	                         .basis_hash
	                   : 0,
	               eval_root_ms,
	               strict_highs_eval_error.empty()
	                   ? "none"
	                   : strict_highs_eval_error);
	  }

	  const bool run_highs_style_root_source_loop =
	      !strict_highs_root_fixed_point &&
	      opt.cuts != CutType::None &&
          root_highs_presolved_side_state_first_class &&
	      // Achterberg (2007), Secs. 4.1-4.2; derivation in
	      // docs/native_milp_root_source_eligibility_2026-08-13.md.
	      highs_style_root_source_eligible(
	          static_cast<int>(std::count(declared_branchable_cols.begin(),
	                                      declared_branchable_cols.end(), 1)),
	                                      root_implied_integer_count,
	                                      root.bound);
      if (!run_highs_style_root_source_loop && opt.auto_highs_root_pipeline &&
          opt.cuts != CutType::None && strict_highs_root_fixed_point &&
          root_highs_presolved_side_state_first_class &&
          !strict_highs_evaluate_root_imported &&
          (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
           bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr)) {
        fmt::print(stderr,
                   "[B&C ROOT-SOURCE-SKIP] "
                   "reason=strict_requires_direct_evaluateRootNode "
                   "evaluateRootImported={} root_bound={:.17g}\n",
                   strict_highs_evaluate_root_imported ? 1 : 0, root.bound);
      }
      if (!run_highs_style_root_source_loop && opt.auto_highs_root_pipeline &&
          opt.cuts != CutType::None && strict_highs_root_fixed_point &&
          !root_highs_presolved_side_state_first_class &&
          (opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
           bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr)) {
        fmt::print(stderr,
                   "[B&C ROOT-SOURCE-SKIP] reason=highs_space_mismatch "
                   "native_cols={} native_rows={} highs_cols={} highs_rows={} "
                   "root_bound={:.17g}\n",
                   n, root_highs_native_rows, highs_presolve_side_state.cols,
                   highs_presolve_side_state.rows, root.bound);
      }
				  if (run_highs_style_root_source_loop) {
					    const double source_first_bound = root.bound;
	    const Eigen::VectorXd source_first_solution =
	        root_source_solution().size() == n ? root_source_solution() : root.x_relax;
	    Eigen::VectorXd highs_avg_direction = Eigen::VectorXd::Zero(n);
	    double highs_smooth_progress = 0.0;
	    int highs_source_stall = 0;
	        auto root_separation_optimality_limit = [&]() -> double {
	          if (!has_root_cutoff_for_propagation ||
	              !std::isfinite(root_cutoff_raw_for_propagation)) {
	            return kInf;
          }
	          const double root_cutoff_obj = root_cutoff_raw_for_propagation;
	          const double integral_scale =
	              highs_style_objective_integral_scale(
	                  base_lp, std::max(1e-12, highs_mip_feastol));
	          if (integral_scale > 0.0) {
	            double limit = (std::ceil(integral_scale * root_cutoff_obj -
	                                      highs_mip_feastol) -
	                            1.0) /
	                           integral_scale;
	            if (std::isfinite(opt.gap_tol) && opt.gap_tol > 0.0) {
	              limit = std::min(
	                  limit,
	                  root_cutoff_obj -
	                      std::ceil(opt.gap_tol * std::abs(root_cutoff_obj) *
	                                    integral_scale -
	                                highs_mip_feastol) /
	                          integral_scale);
	            }
	            return limit + highs_mip_feastol;
	          }
          double limit = std::min(root_cutoff_obj - highs_mip_feastol,
                                  std::nextafter(root_cutoff_obj, -kInf));
          if (std::isfinite(opt.gap_tol) && opt.gap_tol > 0.0) {
            limit = std::min(
                limit, root_cutoff_obj -
                           opt.gap_tol * std::abs(root_cutoff_obj));
          }
          return limit;
        };
	        auto close_root_source_stage = [&](const char* phase) -> int {
          ++root_source_trace.flush_calls;
          const auto t_flush0 = std::chrono::steady_clock::now();
          (void)flush_root_transformed_cutpool_sources(phase);
          root_source_trace.flush_ms += root_source_elapsed_ms(t_flush0);

          ++root_source_trace.prop_calls;
          const auto t_prop0 = std::chrono::steady_clock::now();
          const int changed_cols = propagate_root_transformed_cutpool_resolve();
          root_source_trace.prop_ms += root_source_elapsed_ms(t_prop0);
          return changed_cols;
        };

		        for (int source_round = 0;;) {
          // Anytime root separation stops between valid LP fixed-point rounds;
          // budget derivation in
          // docs/native_milp_root_source_eligibility_2026-08-13.md.
          if (bc_optional_root_budget_expired()) break;
          if (!(root.bound < root_separation_optimality_limit())) {
            break;
          }
	      const int frac_before = count_fractional_source_integers();
	      if (frac_before <= 0) break;

      const double round_bound_before = root.bound;
      const std::size_t varbounds_before = variable_bound_table.size();
	      const std::size_t cutpool_before = variable_bound_cutpool_rows.size();
		      const std::uint64_t impl_before = implication_graph.size();
		      const int rows_before = static_cast<int>(root_lp.A.rows());
		      const RootSourceLoopTrace round_trace_before = root_source_trace;

	      ++root_source_trace.ib_calls;
	      const auto t_ib0 = std::chrono::steady_clock::now();
          const int cached_ib_rows = 0;
		      const bool ib_moved = run_root_probed_implied_bound_resolve();
	      if (ib_moved) ++root_source_epoch;
	      root_source_trace.ib_ms += root_source_elapsed_ms(t_ib0);
          (void)cached_ib_rows;
				      int implied_bound_pool_added = 0;
	      if (!root_deferred_implied_bound_cutpool_rows.empty()) {
	        implied_bound_pool_added += add_root_transformed_cutpool_rows(
	            root_deferred_implied_bound_cutpool_rows, "implied_bound");
	        root_deferred_implied_bound_cutpool_rows.clear();
	      }
          const int implied_bound_stage_cols =
              close_root_source_stage("after_implied_bounds_propagate");
		      const int tableau_pool_added =
              generate_root_tableau_cutpool();
		      const int path_pool_added = generate_root_path_cutpool();
          const int modk_pool_added = generate_root_modk_cutpool();
	          const int separator_stage_cols =
	              close_root_source_stage("after_separator_propagate");
			      const RootCutpoolResolveResult transformed_pool =
			          separate_root_transformed_cutpool_resolve();
		          // HiGHS ends a separation round after cutpool add/resolve.
		          // New domain propagation caused by the accepted cuts is handled
		          // by the next round's implied-bounds/propagate fixed point, not
		          // by an immediate full closure replay on the post-cutpool LP.
		          const int post_cutpool_stage_cols = 0;
	          const bool cutpool_prop_moved =
	              implied_bound_stage_cols > 0 || separator_stage_cols > 0 ||
	              post_cutpool_stage_cols > 0;

      if (!std::isfinite(root.bound)) {
        if (trace_root_source_loop) {
          fmt::print(stderr,
                     "[B&C ROOT-SOURCE] round={} pruned bound={:.12g}\n",
                     source_round, root.bound);
        }
        break;
      }

	      const double round_lift = root.bound - round_bound_before;
	      const double total_lift = root.bound - source_first_bound;
				      const int highs_round_ncuts =
				          implied_bound_stage_cols +
				          separator_stage_cols +
				          post_cutpool_stage_cols +
				          std::max(0, transformed_pool.admitted_rows);
				      const bool moved = highs_round_ncuts > 0;
	          const int frac_after = count_fractional_source_integers();

		      if (trace_root_source_loop) {
		        fmt::print(stderr,
		                   "[B&C ROOT-SOURCE] round={} ib={} xpoolProp={} xpool={} "
		                   "ibStage={} tabStage={} pathStage={} postXpool={} "
		                   "ibPool={} xtabPool={} xpathPool={} xmodkPool={} "
		                   "rows={} ncuts={} varbounds={} cutpool={} impl={} "
		                   "bound={:.2f}->{:.2f} lift={:.3e} total_lift={:.3e} "
		                   "frac_int={}\n",
		                   source_round, ib_moved ? 1 : 0,
		                   cutpool_prop_moved ? 1 : 0,
		                   transformed_pool.admitted_rows > 0 ? 1 : 0,
		                   implied_bound_stage_cols,
		                   0,
		                   separator_stage_cols,
	                   post_cutpool_stage_cols,
	                   implied_bound_pool_added, tableau_pool_added,
                       path_pool_added, modk_pool_added,
		                   static_cast<int>(root_lp.A.rows()) - rows_before,
                       highs_round_ncuts,
		                   static_cast<long long>(variable_bound_table.size()) -
		                       static_cast<long long>(varbounds_before),
                   static_cast<long long>(variable_bound_cutpool_rows.size()) -
                       static_cast<long long>(cutpool_before),
	                   static_cast<long long>(implication_graph.size()) -
	                       static_cast<long long>(impl_before),
	                   round_bound_before, root.bound, round_lift, total_lift,
		                   frac_after);
        fmt::print(stderr,
                   "[B&C ROOT-SOURCE-TIME] round={} "
                   "ib={:.1f} tab={:.1f}(gen={:.1f},add={:.1f}) "
                   "path={:.1f}(gen={:.1f},add={:.1f}) "
                   "modk={:.1f}(gen={:.1f},add={:.1f}) "
                   "flush={:.1f} prop={:.1f} xpool={:.1f} "
	                   "(select={:.1f},build={:.1f},sf={:.1f},solve={:.1f},apply={:.1f}) "
	                   "xsel={} xrows={} xres={}/{}\n",
                   source_round,
                   root_source_trace.ib_ms - round_trace_before.ib_ms,
                   root_source_trace.tableau_ms -
                       round_trace_before.tableau_ms,
                   root_source_trace.tableau_generate_ms -
                       round_trace_before.tableau_generate_ms,
                   root_source_trace.tableau_pool_add_ms -
                       round_trace_before.tableau_pool_add_ms,
                   root_source_trace.path_ms - round_trace_before.path_ms,
                   root_source_trace.path_generate_ms -
                       round_trace_before.path_generate_ms,
                   root_source_trace.path_pool_add_ms -
                       round_trace_before.path_pool_add_ms,
                   root_source_trace.modk_ms - round_trace_before.modk_ms,
                   root_source_trace.modk_generate_ms -
                       round_trace_before.modk_generate_ms,
                   root_source_trace.modk_pool_add_ms -
                       round_trace_before.modk_pool_add_ms,
                   root_source_trace.flush_ms - round_trace_before.flush_ms,
                   root_source_trace.prop_ms - round_trace_before.prop_ms,
                   root_source_trace.xpool_ms - round_trace_before.xpool_ms,
                   root_source_trace.xpool_select_ms -
                       round_trace_before.xpool_select_ms,
                   root_source_trace.xpool_build_ms -
                       round_trace_before.xpool_build_ms,
                   root_source_trace.xpool_sf_ms -
                       round_trace_before.xpool_sf_ms,
                   root_source_trace.xpool_solve_ms -
                       round_trace_before.xpool_solve_ms,
                   root_source_trace.xpool_apply_ms -
                       round_trace_before.xpool_apply_ms,
                   root_source_trace.xpool_selected -
                       round_trace_before.xpool_selected,
                   root_source_trace.xpool_rows - round_trace_before.xpool_rows,
                   root_source_trace.xpool_resolve_ok -
                       round_trace_before.xpool_resolve_ok,
	                   root_source_trace.xpool_resolve_calls -
	                       round_trace_before.xpool_resolve_calls);
	      }

			      ++source_round;
			      root_source_trace.rounds = source_round;
		      const int source_rows_added =
		          static_cast<int>(root_lp.A.rows()) - rows_before;
		      (void)source_rows_added;
          const bool lp_scaled_optimal =
              root_relax.primal.stats.success && root_relax.primal.x.size() == n;
	          double highs_progress = 0.0;
	          double highs_next_progress = highs_smooth_progress;
	          bool highs_stall_hit = false;
	          if (source_first_solution.size() == n &&
	              root_source_solution().size() == n) {
	            const Eigen::VectorXd cur_direction =
	                source_first_solution - root_source_solution();
	            const double cur_norm2 = cur_direction.squaredNorm();
	            if (cur_norm2 > 0.0 && std::isfinite(cur_norm2)) {
	              const double scale = 1.0 / std::sqrt(cur_norm2);
	              highs_avg_direction =
	                  (scale * cur_direction - highs_avg_direction) /
	                  static_cast<double>(source_round);
	              const double avg_norm2 = highs_avg_direction.squaredNorm();
	              const double dot_product =
	                  highs_avg_direction.dot(cur_direction);
	              if (avg_norm2 > 0.0 && std::isfinite(avg_norm2) &&
	                  std::isfinite(dot_product)) {
	                highs_progress = dot_product / std::sqrt(avg_norm2);
	              }
	            }
	          }
	          if (source_round == 1) {
	            highs_smooth_progress = highs_progress;
	            highs_next_progress = highs_smooth_progress;
	          } else {
	            constexpr double alpha = 1.0 / 3.0;
	            highs_next_progress =
	                (1.0 - alpha) * highs_smooth_progress +
	                alpha * highs_progress;
	            const bool root_obj_not_enough =
	                (root.bound - source_first_bound) <=
	                (round_bound_before - source_first_bound) * 1.001;
	            if (highs_next_progress < highs_smooth_progress * 1.01 &&
	                root_obj_not_enough) {
	              ++highs_source_stall;
	              highs_stall_hit = true;
	            } else {
	              highs_source_stall = 0;
	            }
	            highs_smooth_progress = highs_next_progress;
	          }
				          if (trace_root_source_loop) {
				            const char* stop_reason = "continue";
				            if (frac_after <= 0) {
				              stop_reason = "fractional_empty";
				            } else if (!moved) {
				              stop_reason = "ncuts_zero";
	                  } else if (!lp_scaled_optimal) {
	                    stop_reason = "lp_not_optimal";
	                  } else if (highs_source_stall >= 3) {
	                    stop_reason = "stall";
				            }
			            fmt::print(stderr,
			                       "[B&C ROOT-SOURCE-ROUND] round={} "
				                       "frac={} ncuts={} pool_avail={} pool_viol={} "
				                       "pool_active={} pool_sel={} pool_admit={} "
	                         "obj={:.17g} first={:.17g} last={:.17g} "
	                         "progress={:.17g} smooth={:.17g} next={:.17g} "
	                         "stall_hit={} stall={} "
	                         "optLimit={:.17g} lpOpt={} stop={}\n",
				                       source_round, frac_after, highs_round_ncuts,
				                       transformed_pool.pool_stats.available,
		                       transformed_pool.pool_stats.violated,
		                       transformed_pool.pool_stats.active,
			                       transformed_pool.pool_stats.selected,
			                       transformed_pool.admitted_rows,
				                       root.bound, source_first_bound,
	                         round_bound_before, highs_progress,
	                         highs_smooth_progress, highs_next_progress,
	                         highs_stall_hit ? 1 : 0, highs_source_stall,
	                         root_separation_optimality_limit(),
	                         lp_scaled_optimal ? 1 : 0,
				                       stop_reason);
				          }
				      if (!moved) break;
			          if (frac_after <= 0) {
			            break;
			          }
	              if (!lp_scaled_optimal) break;
	              if (highs_source_stall >= 3) break;
					    }
    if (trace_root_source_loop) {
      fmt::print(stderr,
                 "[B&C ROOT-SOURCE-TIME-SUM] rounds={} "
                 "ib_calls={} tab_calls={} path_calls={} modk_calls={} "
                 "flush_calls={} "
                 "prop_calls={} xpool_calls={} xres={}/{} "
                 "ib={:.1f} tab={:.1f}(gen={:.1f},add={:.1f}) "
                 "path={:.1f}(gen={:.1f},add={:.1f}) "
                 "modk={:.1f}(gen={:.1f},add={:.1f}) "
                 "flush={:.1f} prop={:.1f} xpool={:.1f} "
                 "(select={:.1f},build={:.1f},sf={:.1f},solve={:.1f},apply={:.1f}) "
                 "xsel={} xrows={} pool={} bound={:.6g}\n",
                 root_source_trace.rounds, root_source_trace.ib_calls,
                 root_source_trace.tableau_calls,
                 root_source_trace.path_calls,
                 root_source_trace.modk_calls,
                 root_source_trace.flush_calls, root_source_trace.prop_calls,
                 root_source_trace.xpool_calls,
                 root_source_trace.xpool_resolve_ok,
                 root_source_trace.xpool_resolve_calls,
                 root_source_trace.ib_ms, root_source_trace.tableau_ms,
                 root_source_trace.tableau_generate_ms,
                 root_source_trace.tableau_pool_add_ms,
                 root_source_trace.path_ms, root_source_trace.path_generate_ms,
                 root_source_trace.path_pool_add_ms,
                 root_source_trace.modk_ms,
                 root_source_trace.modk_generate_ms,
                 root_source_trace.modk_pool_add_ms,
                 root_source_trace.flush_ms, root_source_trace.prop_ms,
                 root_source_trace.xpool_ms,
                 root_source_trace.xpool_select_ms,
                 root_source_trace.xpool_build_ms,
                 root_source_trace.xpool_sf_ms,
                 root_source_trace.xpool_solve_ms,
                 root_source_trace.xpool_apply_ms,
                 root_source_trace.xpool_selected,
                 root_source_trace.xpool_rows,
                 static_cast<int>(root_transformed_cutpool.size()),
                 root.bound);
    }
  }

	  if (strict_highs_root_fixed_point &&
	      bc_env_flag_enabled("MIPSOLVERS_ROOT_SEPARATION_ONLY")) {
	    join_root_background_tasks();
	    const auto t_root_stop = std::chrono::steady_clock::now();
	    const double runtime =
	        std::chrono::duration<double>(t_root_stop - t0).count();
	    out.x = root.x_relax;
	    out.stats.objective = std::numeric_limits<double>::quiet_NaN();
	    out.stats.primal_feas = 0.0;
	    out.stats.residual_inf = 0.0;
	    out.stats.mip_gap = kInf;
	    out.stats.runtime_sec = runtime;
	    out.stats.iterations = out.bc_stats.lp_solves;
	    out.stats.success = true;
	    out.stats.status = bc_status::kRootSeparationOnly;
	    out.stats.solver_name = "NativeBranchAndCut";
	    out.bc_stats.runtime_sec = runtime;
	    out.bc_stats.best_bound = root.bound;
	    out.bc_stats.gap = kInf;
	    out.bc_stats.status = out.stats.status;
	    out.bc_stats.root_only_result = true;
	    out.bc_stats.root_only_bound = root.bound;
	    if (!std::isfinite(out.bc_stats.root_only_solver_bound)) {
	      out.bc_stats.root_only_solver_bound =
	          root.bound - (strict_highs_presolved_working_space
	                            ? strict_highs_presolved_objective_offset
	                            : 0.0);
	    }
	    if (out.bc_stats.root_only_objective_offset == 0.0 &&
	        strict_highs_presolved_working_space) {
	      out.bc_stats.root_only_objective_offset =
	          strict_highs_presolved_objective_offset;
	    }
	    if (trace_root_source_loop) {
	      fmt::print(stderr,
	                 "[B&C ROOT-ONLY] status={} bound={:.17g} solverBound={:.17g} "
	                 "offset={:.17g} rows={} "
	                 "cuts={} pool={} rounds={} runtime={:.3f}s\n",
	                 out.stats.status, root.bound,
	                 out.bc_stats.root_only_solver_bound,
	                 out.bc_stats.root_only_objective_offset,
	                 static_cast<int>(root_lp.A.rows()),
	                 out.bc_stats.root_cuts_added,
                 static_cast<int>(root_transformed_cutpool.size()),
                 root_source_trace.rounds, runtime);
    }
    return return_with_profile();
  }

  out.bc_stats.root_implication_arcs =
      static_cast<std::uint64_t>(implication_graph.size());

  root.estimate = compute_node_estimate(declared_branchable_indices, root.x_relax, pc,
                                        opt.int_tol, root.bound,
                                        opt.node_estimate_aggregation);

  const LPModel pre_cut_root_lp = root_lp;
  const auto pre_cut_basis_hint = root.basis_hint;
  const auto pre_cut_x_relax = root.x_relax;
  const double pre_cut_bound = root.bound;
  Eigen::VectorXd root_cut_audit_incumbent;
  const Eigen::VectorXd* root_cut_audit_incumbent_ptr = nullptr;
  if (opt.root_cut_audit_validate_rows &&
      reduced_initial_solution.size() == n) {
    root_cut_audit_incumbent =
        clamp_to_bounds(reduced_initial_solution, root.lb, root.ub);
    if (satisfies_lp(root_lp, root_cut_audit_incumbent,
                     std::max(1e-7, 10.0 * opt.heuristic_feasibility_tol))) {
      root_cut_audit_incumbent_ptr = &root_cut_audit_incumbent;
    } else {
      root_cut_audit_incumbent.resize(0);
    }
  }

  // A verified original-space MIP start can terminate the solve immediately
  // once the root LP bound proves it is already within the requested MIP gap.
  // This is a general-purpose incumbent/dual-bound check and avoids building
  // root cut and heuristic infrastructure that cannot change the outcome.
	  if (!strict_highs_lp_contract &&
      heuristic_x_orig.size() > 0 && std::isfinite(heuristic_obj) &&
	      std::isfinite(root.bound)) {
    const double presolve_objective_offset =
        highs_mip_model_reduced
            ? highs_mip_ps.objective_offset
            : (papilo_model_reduced ? papilo_ps.objective_offset : 0.0);
    const double heuristic_obj_in_root_space = presolved_objective_value(
        heuristic_obj, presolve_objective_offset);
    const double heuristic_gap =
        std::max(0.0, (heuristic_obj_in_root_space - root.bound) /
                           std::max(1.0, std::abs(heuristic_obj)));
    if (!opt.require_tree_exhaustion_certificate &&
        heuristic_gap <= opt.gap_tol) {
      const auto t1 = std::chrono::steady_clock::now();
      const double runtime = std::chrono::duration<double>(t1 - t0).count();

      out.x = heuristic_x_orig;
      // root.bound and heuristic_obj use the internal minimization convention;
      // the public result must retain the caller's objective direction.
      out.stats.objective = prob.linear_part.c.dot(out.x);
      out.stats.primal_feas = 0.0;
      out.stats.residual_inf = 0.0;
      out.stats.mip_gap = heuristic_gap;
      out.stats.runtime_sec = runtime;
      out.stats.iterations = 0;
      out.stats.success = true;
      out.stats.status = bc_status::kOptimalRootGapClosed;

      out.bc_stats.runtime_sec = runtime;
      const double root_bound_in_original_space =
          root.bound + presolve_objective_offset;
      out.bc_stats.best_bound =
          base_lp.sense == Sense::Minimize
              ? root_bound_in_original_space
              : -root_bound_in_original_space;
      out.bc_stats.best_obj = out.stats.objective;
      out.bc_stats.gap = heuristic_gap;
      out.bc_stats.status = out.stats.status;

      return return_with_profile();
    }
  }

  // Root cut rounds.
  // Per-family efficacy tracker used to disable weak cut families adaptively.
  CutFamilyTracker cut_family_tracker;

  // ── Size-adaptive cut parameters for large problems ──
  // On large LPs (>5000 presolved rows), cut generation becomes expensive:
  //   - Simplex re-solve: O(m²·k) per round for m rows, k cuts
  //   - StandardFormLP rebuild: O(nnz) per round
  // Scale down cut rounds and use IPM re-solve to maintain performance.
  int effective_cut_rounds = opt.root_cut_rounds;
  int effective_cuts_per_round = opt.cuts_per_round;
  bool is_scuc_like = false;  // set below in root-fractional diagnostic block
  std::string root_cut_skip_reason;
  // Allocate a bounded share of the remaining root budget to cuts. The cap
  // prevents cut separation from starving incumbent heuristics and tree
  // proof on short solves (Achterberg, 2007, Sections 4.1-4.2, 8.1).
  const double cut_usable_budget_sec =
      std::isfinite(bc_remaining_sec())
          ? std::max(0.0, bc_remaining_sec() -
                              bc_post_root_finalize_reserve_sec)
          : std::numeric_limits<double>::infinity();
  const double cut_budget_sec =
      std::isfinite(cut_usable_budget_sec)
          ? 0.35 * cut_usable_budget_sec
          : std::numeric_limits<double>::infinity();
  const auto cut_stage_start = std::chrono::steady_clock::now();
  auto cut_stage_remaining_sec = [&]() -> double {
    if (!std::isfinite(cut_budget_sec)) {
      return std::numeric_limits<double>::infinity();
    }
    return cut_budget_sec - std::chrono::duration<double>(
                                std::chrono::steady_clock::now() -
                                cut_stage_start)
                                .count();
  };
  if (std::isfinite(cut_budget_sec)) {
    if (cut_budget_sec < 2.0) {
      effective_cut_rounds = std::min(effective_cut_rounds, 3);
    } else {
      // A conservative 50 ms/round lower bound avoids allocating more rounds
      // than can complete even when the first separation is unusually cheap.
      effective_cut_rounds = std::min(
          effective_cut_rounds,
          std::max(0, static_cast<int>(cut_budget_sec / 0.05)));
    }
    if (cut_budget_sec <= 0.05) effective_cut_rounds = 0;
  }
  const bool highs_root_pipeline_active =
      opt.auto_highs_root_pipeline && opt.cuts != CutType::None;
  if (!highs_root_pipeline_active &&
      presolved_m > opt.cut_budget_xlarge_row_threshold) {
    // Extremely large: skip cuts entirely.  Even IPM cut
    // re-solves are prohibitively expensive at this scale.
    effective_cut_rounds = 0;
  } else if (!highs_root_pipeline_active &&
             presolved_m > opt.cut_budget_very_large_row_threshold) {
    // Very large roots: allow only a few IPM-based cut rounds.
    effective_cut_rounds = std::min(opt.root_cut_rounds, 3);
    effective_cuts_per_round = std::min(opt.cuts_per_round, opt.cuts_per_round_very_large);
    opt.gmi_max_density = std::min(opt.gmi_max_density, 0.10);
  } else if (!highs_root_pipeline_active &&
             presolved_m > opt.cut_budget_large_row_threshold) {
    // Large: reduce cut aggression substantially.
    effective_cut_rounds = std::min(opt.root_cut_rounds, 3);
    effective_cuts_per_round = std::min(opt.cuts_per_round, opt.cuts_per_round_large);
    opt.gmi_max_density = std::min(opt.gmi_max_density, 0.15);
  } else if (!highs_root_pipeline_active &&
             presolved_m > opt.cut_budget_medium_row_threshold) {
    // Medium-large: moderate reduction.
    effective_cut_rounds = std::min(opt.root_cut_rounds, 6);
    effective_cuts_per_round = std::min(opt.cuts_per_round, opt.cuts_per_round_medium);
  }

  // Skip cuts when root LP integrality gap is very small: the LP relaxation
  // is already tight and cuts can't meaningfully improve the bound.
  // Only limit cut rounds when the fractional count is truly negligible
  // (< 10 fractional binaries).  For SCUC with 63/2661 fractional the
  // old n_total/20 threshold incorrectly capped cuts to 1 round.
  int root_frac_bin_count = 0;
  int root_total_bin_count = 0;
  {
    int n_frac_bin = 0, n_total_bin = 0;
    for (int i = 0; i < n; ++i) {
      if (base_lp.vars[i].type == VarType::Binary) {
        ++n_total_bin;
        double v = root.x_relax[i];
        if (v > opt.int_tol && v < 1.0 - opt.int_tol) ++n_frac_bin;
      }
    }
    root_frac_bin_count = n_frac_bin;
    root_total_bin_count = n_total_bin;
    // SCUC-like detection (published later for prog-round skip + LNS budget).
    is_scuc_like = domain_heuristics &&
                   (n_total_bin >= opt.scuc_min_binaries && 2 * n_total_bin > n &&
                    n_frac_bin >= opt.scuc_min_frac_binaries &&
                    presolved_m >= opt.scuc_min_rows &&
                    presolved_m <= opt.scuc_max_rows);
    if (!highs_root_pipeline_active &&
        n_total_bin > 100 &&
        n_frac_bin <= std::min(10, std::max(1, n_total_bin / 100))) {
      // Truly negligible fractional binaries — root LP is already tight.
      effective_cut_rounds = std::min(effective_cut_rounds, 1);
    }
    // SCUC-like auto-tune: large binary fraction (>50%) + moderately sized root
    // LP + non-trivial fractional count.  The default filters (efficacy=1e-3,
    // density=0.35) reject many Gomory candidates on this model shape. Relax
    // the filters and grant more rounds; the result remains subject to the
    // ordinary validity, efficacy, density, stall, and deadline gates.
    if (is_scuc_like) {
      if (opt.gmi_min_efficacy > 1e-4)     opt.gmi_min_efficacy = 1e-4;
      if (opt.gmi_max_density  < 0.50)     opt.gmi_max_density  = 0.50;
      if (opt.gmi_min_activity > 0.0)      opt.gmi_min_activity = 0.0;
      if (opt.gmi_min_binary_support > 0.0) opt.gmi_min_binary_support = 0.0;
      effective_cut_rounds   = std::max(effective_cut_rounds,   opt.scuc_min_cut_rounds);
      effective_cuts_per_round = std::max(effective_cuts_per_round, opt.scuc_min_cuts_per_round);
      // Tight stall cap: cut generation costs ~210ms/round even when empty.
      // 2 consecutive stalls is enough to detect exhaustion on SCUC.
      if (opt.root_cut_max_stalls == 0 || opt.root_cut_max_stalls > 2) {
        opt.root_cut_max_stalls = 2;
      }
      if (opt.verbose) {
        fprintf(stderr, "[B&C] SCUC-like auto-tune: cut_rounds=%d cuts/round=%d "
                "gmi_eff=%.0e gmi_dens=%.2f\n",
                effective_cut_rounds, effective_cuts_per_round,
                opt.gmi_min_efficacy, opt.gmi_max_density);
      }
    }
  }
  const bool large_near_integral_root =
      n >= opt.near_integral_n_threshold &&
      root_frac_bin_count <= opt.near_integral_max_frac_bins;

  // Large UC-like detection gates root work that has a bounded optional
  // budget. Hoist the classification before separation so all optional
  // components apply the same shape and deadline policy.
  int bc_skip_n_bin = 0;
  {
    for (int j = 0; j < n; ++j) {
      if (is_integer_type(base_lp.vars[j]) &&
          base_lp.vars[j].lb >= -0.5 && base_lp.vars[j].ub <= 1.5) {
        ++bc_skip_n_bin;
      }
    }
  }
  const bool large_uc_like_root = domain_heuristics &&
      (presolved_m > opt.large_uc_row_threshold) &&
      (bc_skip_n_bin >= opt.large_uc_binary_threshold);
  const bool large_generic_root =
      large_generic_presolved_root;
  const int large_generic_low_frac_cap =
      std::min(opt.large_generic_low_frac_abs,
               std::max(1, static_cast<int>(
                              opt.large_generic_low_frac_ratio *
                              static_cast<double>(std::max(1, root_total_bin_count)))));
  const bool large_generic_low_frac_root =
      large_generic_root && root_total_bin_count > 0 &&
      root_frac_bin_count <= large_generic_low_frac_cap;
  const bool is_xlarge_root =
      (presolved_m > opt.cut_budget_xlarge_row_threshold);
  const int resolved_threads = resolve_num_threads(opt);
  const bool par_mode = (resolved_threads > 1);
  bool parallel_tree_allowed = par_mode;
  std::string parallel_schedule_reason = par_mode ? "enabled" : "single_thread";
  out.bc_stats.parallel_requested_threads = resolved_threads;
  out.bc_stats.parallel_effective_threads = 1;
  out.bc_stats.parallel_explorer_threads = 0;
  out.bc_stats.parallel_tree_launched = false;
  out.bc_stats.parallel_schedule_reason =
      par_mode ? "pending" : "single_thread";
  const bool force_root_cut_audit = opt.root_cut_audit_force_separation;
  if (!highs_root_pipeline_active &&
      !force_root_cut_audit && root_propagation_local_cutoff_tightened &&
      effective_cut_rounds > 0) {
    effective_cut_rounds = 0;
    root_cut_skip_reason = "root_objective_cutoff_domain";
  }

  // Precompute LP column lock counts.  For l <= a*x <= u, a finite upper
  // side contributes an up-lock for a_j>0 and a down-lock for a_j<0; a finite
  // lower side contributes the opposite locks.  Thus every nonzero equality
  // coefficient contributes in both directions.
  // Achterberg (2007), Sec. 9.2; HiGHS HighsMipSolverData::setup; derivation
  // in docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
  std::vector<int> uplocks(n, 0), downlocks(n, 0);
  if (opt.use_lock_count_rounding || opt.use_linesearch_rounding) {
    const LPModel& lock_lp = root_lp;
    for (int k = 0; k < lock_lp.A.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lock_lp.A, k); it; ++it) {
        const int j = static_cast<int>(it.col());
        const int r = static_cast<int>(it.row());
        if (j < 0 || j >= n || r < 0 || r >= lock_lp.A.rows()) continue;
        const double a = it.value();
        if (std::isfinite(lock_lp.b[r])) {
          if (a > 0.0) ++uplocks[j];
          else if (a < 0.0) ++downlocks[j];
        }
        const double lhs = lp_row_lhs_or_neg_inf(lock_lp, r);
        if (std::isfinite(lhs)) {
          if (a > 0.0) ++downlocks[j];
          else if (a < 0.0) ++uplocks[j];
        }
      }
    }
    for (int k = 0; k < lock_lp.Aeq.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lock_lp.Aeq, k); it; ++it) {
        const int j = static_cast<int>(it.col());
        if (j < 0 || j >= n || it.value() == 0.0) continue;
        ++uplocks[j];
        ++downlocks[j];
      }
    }
    if (bc_env_flag_enabled("MIPSOLVERS_ROOT_LOCK_TRACE")) {
      for (int j = 0; j < n; ++j) {
        if (!is_integer_type(lock_lp.vars[static_cast<std::size_t>(j)])) {
          continue;
        }
        fmt::print(stderr,
                   "[NATIVE-ROOT-LOCK] col={} up={} down={}\n", j,
                   uplocks[j], downlocks[j]);
      }
    }
  }

  // Fix high-impact integers first, then use a stable hash/index tie break.
  // This is the HiGHS setupIntCols order used by tryRoundedPoint's sequential
  // propagation projection. Achterberg (2007), Secs. 3.1 and 9.2; derivation
  // in docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md.
  std::vector<int> repair_integer_order;
  repair_integer_order.reserve(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    if (is_integer_type(root_lp.vars[static_cast<std::size_t>(j)])) {
      repair_integer_order.push_back(j);
    }
  }
  constexpr double kRepairOrderFeasTol = 1e-6;
  std::sort(repair_integer_order.begin(), repair_integer_order.end(),
            [&](const int lhs, const int rhs) {
    const double lhs_lock_score =
        (kRepairOrderFeasTol + static_cast<double>(uplocks[lhs])) *
        (kRepairOrderFeasTol + static_cast<double>(downlocks[lhs]));
    const double rhs_lock_score =
        (kRepairOrderFeasTol + static_cast<double>(uplocks[rhs])) *
        (kRepairOrderFeasTol + static_cast<double>(downlocks[rhs]));
    if (lhs_lock_score != rhs_lock_score) {
      return lhs_lock_score > rhs_lock_score;
    }

    const auto literal_degree = [&](const int col, const bool value_one) {
      const auto range = clique_table.literal_neighbours(col, value_one);
      return range.first == nullptr
                 ? 0
                 : static_cast<int>(range.second - range.first);
    };
    const double lhs_clique_score =
        (kRepairOrderFeasTol + literal_degree(lhs, true)) *
        (kRepairOrderFeasTol + literal_degree(lhs, false));
    const double rhs_clique_score =
        (kRepairOrderFeasTol + literal_degree(rhs, true)) *
        (kRepairOrderFeasTol + literal_degree(rhs, false));
    return std::make_tuple(lhs_clique_score,
                           HighsHashHelpers::hash(uint64_t(lhs)), lhs) >
           std::make_tuple(rhs_clique_score,
                           HighsHashHelpers::hash(uint64_t(rhs)), rhs);
  });

  // On very large, already-near-integral roots, full cut separation is often
  // dominated by scanning overhead and rarely changes the solve path.
  if (force_root_cut_audit && effective_cut_rounds == 0 && !is_xlarge_root &&
      opt.cuts != CutType::None && opt.root_cut_rounds > 0) {
    effective_cut_rounds = std::min(opt.root_cut_rounds, 3);
    effective_cuts_per_round = std::max(effective_cuts_per_round,
                                        std::min(opt.cuts_per_round, 40));
    root_cut_skip_reason = "audit_forced";
    if (opt.verbose) {
      fmt::print(stderr,
                 "[CUT-AUDIT] forcing root separation: rounds={} cuts/round={} "
                 "(m={} frac_bin={}/{})\n",
                 effective_cut_rounds, effective_cuts_per_round, presolved_m,
                 root_frac_bin_count, root_total_bin_count);
    }
  }

  if (!highs_root_pipeline_active &&
      !force_root_cut_audit &&
      large_near_integral_root && root_implied_integer_cut_support_count == 0) {
    effective_cut_rounds = 0;
    root_cut_skip_reason = "near_integral_root";
  }

  // If an already-verified heuristic incumbent is within the configured MIP
  // gap of the root LP bound, root cuts cannot change the solve outcome.
  // Skip them entirely rather than spending time scanning cut families that
  // will not affect the termination decision.
  if (!highs_root_pipeline_active &&
      !force_root_cut_audit &&
      (has_verified_root_cutoff || std::isfinite(heuristic_obj)) &&
      std::isfinite(root.bound)) {
    const double root_cutoff_obj = has_verified_root_cutoff
        ? verified_root_cutoff_obj
        : heuristic_obj;
    const double heuristic_gap = (root_cutoff_obj - root.bound) /
        std::max(1.0, std::abs(root_cutoff_obj));
    if (heuristic_gap <= opt.gap_tol) {
      effective_cut_rounds = 0;
      root_cut_skip_reason = "heuristic_gap_closed";
    }
  }
  
  // Skip cuts entirely on very large IPM-only problems: no simplex basis
  // means Gomory can't fire, and MIR/cover/clique typically find 0 cuts
  // on clean UC problems. Saves ~1200ms of fruitless scanning.
  // Also skip on large UC-like MILPs where the clique graph is empty and
  // all cut families are rejected by efficacy/density filters.
  if (!highs_root_pipeline_active &&
      !force_root_cut_audit &&
      large_uc_like_root && root_implied_integer_cut_support_count == 0 &&
      !is_xlarge_root && effective_cut_rounds > 0) {
    if (opt.verbose) {
      fprintf(stderr, "[B&C] skip root cuts on large UC-like (m=%d n_bin=%d) — "
                      "clique graph empty, no amenable rows; saves ~1.3s\n",
              presolved_m, bc_skip_n_bin);
    }
    effective_cut_rounds = 0;
    root_cut_skip_reason = "large_uc_like_opt_in";
  }
  if (!highs_root_pipeline_active &&
      !force_root_cut_audit &&
      large_generic_low_frac_root && root_implied_integer_cut_support_count == 0 &&
      effective_cut_rounds > 0) {
    if (opt.verbose) {
      fprintf(stderr,
              "[B&C] skip root cuts on large generic low-fractionality root "
              "(m=%d frac_bin=%d/%d cap=%d): yield budget to tree search\n",
              presolved_m, root_frac_bin_count, root_total_bin_count,
              large_generic_low_frac_cap);
    }
    effective_cut_rounds = 0;
    root_cut_skip_reason = "large_generic_low_fractionality";
  }
  if (highs_root_pipeline_active && effective_cut_rounds > 0) {
    // HiGHS' root separators publish candidate rows into HighsCutPool; only
    // cutpool.separate() admits rows into the LP after propagation closure.
    // The native equivalent is the root source/xpool loop above.  Running the
    // legacy add_cuts() loop here would bypass that lifecycle by scanning the
    // tableau again and inserting rows directly into root_lp.
    effective_cut_rounds = 0;
    root_cut_skip_reason = "highs_root_cutpool_fixed_point";
  }
  if (parallel_tree_allowed &&
      opt.parallel_serial_on_low_fractionality_root &&
      large_generic_low_frac_root) {
    parallel_tree_allowed = false;
    parallel_schedule_reason = "serial_low_fractionality_root";
  }
  if (opt.cuts != CutType::None &&
      !(presolved_m > opt.xlarge_ipm_only_threshold && !root_relax.simplex)) {

    // Root cut re-solves: when IPM-root is configured to hand off to simplex,
    // keep every post-crossover root LP in simplex.  Otherwise the IPM cut
    // re-solve clears the basis/nonbasic side and the tree no longer sees the
    // same first-class LP state as a simplex root.
    //
    // Pure IPM-node modes may still use IPM cut re-solves on very large roots.
    std::unique_ptr<NativeIPMLPAdapter> root_cut_ipm;
    const bool require_root_simplex_handoff =
        opt.use_ipm_root && opt.use_simplex_lp_nodes;
    // The native IPM checks its limit between iterations, but a single sparse
    // factorization is not interruptible. On a short global budget that can
    // turn one root cut re-solve into a minute-scale overrun. Keep such solves
    // on the selected simplex kernel, whose HiGHS backend owns an internal
    // deadline.
    if (!short_budget_root && !require_root_simplex_handoff &&
        presolved_m > opt.ipm_cut_resolve_row_threshold) {
      IPMLPOptions ipm_opt;
      ipm_opt.max_iter = (presolved_m > opt.ipm_cut_resolve_xlarge_threshold)
                             ? opt.ipm_node_max_iter
                             : std::max(50, opt.ipm_node_max_iter / 2);
      ipm_opt.tol_primal = opt.lp_tol;
      ipm_opt.tol_dual = opt.lp_tol;
      ipm_opt.tol_gap = opt.lp_tol;
      ipm_opt.verbose = false;
      root_cut_ipm = std::make_unique<NativeIPMLPAdapter>(ipm_opt);
    }

    double prev_bound = root.bound;
    int stall_count = 0;
    double prof_gen_ms [[maybe_unused]] = 0, prof_sf_ms [[maybe_unused]] = 0, prof_solve_ms [[maybe_unused]] = 0;
    [[maybe_unused]] int prof_rounds = 0;
    for (int k = 0; k < effective_cut_rounds; ++k) {
      if (bc_optional_root_budget_expired() ||
          cut_stage_remaining_sec() <= 0.0) break;
      const int prev_ineq_rows = static_cast<int>(root_lp.A.rows());
      const auto round_basis_hint = root.basis_hint;
      const auto round_x_relax = root.x_relax;
      const double round_bound = root.bound;
      const LPRelaxationResult round_relax = root_relax;
      const Eigen::VectorXd round_lb = root.lb;
      const Eigen::VectorXd round_ub = root.ub;
      const std::shared_ptr<BasisOps> round_sbasis = root_sbasis;
      std::shared_ptr<BasisOps> cut_sbasis = root_sbasis;
      if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
        cut_sbasis = root_relax.simplex->basis.cached_sparse_basis;
      }
      auto t_cut_start = std::chrono::steady_clock::now();
      const SimplexResult* cut_simplex = root_relax.simplex.get();
      std::unique_ptr<SimplexResult> implied_integer_simplex;
      if (root_implied_integer_cut_support_count > 0 && root_relax.simplex) {
        implied_integer_simplex = std::make_unique<SimplexResult>(*root_relax.simplex);
        const int n_form = static_cast<int>(implied_integer_simplex->form.original_types.size());
        for (int j = 0; j < std::min(n, n_form); ++j) {
          if (root_implied_integer_cols[static_cast<std::size_t>(j)] &&
              implied_integer_simplex->form.original_types[static_cast<std::size_t>(j)] ==
                  VarType::Continuous) {
            implied_integer_simplex->form.original_types[static_cast<std::size_t>(j)] =
                VarType::Integer;
          }
        }
        cut_simplex = implied_integer_simplex.get();
      }
      const int added = add_cuts(
          root_lp, root.x_relax, cut_simplex, opt, effective_cuts_per_round,
          cut_sbasis, &cut_family_tracker, &clique_table,
          &separator_storage_stats);
      // Experimental root CGLP pass. It may add audited rows when explicitly
      // enabled; the production default stays off pending cohort evidence.
      detail::CGLPPassStats cglp_pass;
      const int cglp_added = detail::add_cglp_cuts_root(root_lp, root.x_relax, opt, &cglp_pass);
      out.bc_stats.cglp_candidates   += cglp_pass.n_candidates;
      out.bc_stats.cglp_lp_solved    += cglp_pass.n_cglp_lp_solved;
      out.bc_stats.cglp_fallback_used+= cglp_pass.n_fallback_used;
      out.bc_stats.cglp_accepted     += cglp_pass.n_accepted;
      out.bc_stats.cglp_dedup_rejected+= cglp_pass.n_dedup_rejected;
      auto t_cut_gen = std::chrono::steady_clock::now();
      const int actual_added_rows =
          static_cast<int>(root_lp.A.rows()) - prev_ineq_rows;
      int round_domain_tightened = 0;
      if (opt.root_cut_audit_validate_rows && actual_added_rows > 0) {
        const auto audit = audit_root_cut_rows(
            root_lp, prev_ineq_rows, root.x_relax,
            root_cut_audit_incumbent_ptr, std::max(1e-9, opt.lp_tol));
        const auto sf_audit = audit_root_cut_standard_form_rows(
            root_lp, prev_ineq_rows, std::max(1e-9, 100.0 * opt.lp_tol));
        out.bc_stats.root_cut_standard_form_mismatches +=
            static_cast<std::uint64_t>(std::max(0, sf_audit.mismatches));
        out.bc_stats.root_cut_standard_form_flipped_rows +=
            static_cast<std::uint64_t>(std::max(0, sf_audit.flipped_rows));
        if (opt.verbose || force_root_cut_audit) {
          fmt::print(stderr,
                     "[CUT-AUDIT] round {}: requested={} cglp={} rows={} "
                     "invalid={} lp_nonviolated={} max_lp_viol={:.3e} "
                     "inc_viol={} max_inc_viol={:.3e} sf_mismatch={} "
                     "sf_flip={} sf_coeff_err={:.2e} sf_rhs_err={:.2e}\n",
                     k, added, cglp_added, audit.rows, audit.invalid_rows,
                     audit.nonviolated_at_lp, audit.max_lp_violation,
                     audit.incumbent_violations,
                     audit.max_incumbent_violation, sf_audit.mismatches,
                     sf_audit.flipped_rows, sf_audit.max_coeff_error,
                     sf_audit.max_rhs_error);
        }
        if (audit.invalid_rows > 0 || audit.incumbent_violations > 0 ||
            sf_audit.mismatches > 0) {
          if (opt.verbose || force_root_cut_audit) {
            fmt::print(stderr,
                       "[CUT-AUDIT] rejecting root cut round {} and reverting "
                       "{} rows due to invalid/cutting incumbent/standard-form "
                       "rows\n",
                       k, actual_added_rows);
          }
          truncate_ineq_rows(root_lp, prev_ineq_rows);
          root.lb = round_lb;
          root.ub = round_ub;
          apply_node_bounds(root_lp.vars, root.lb, root.ub);
          root.basis_hint = round_basis_hint;
          root.x_relax = round_x_relax;
          root.bound = round_bound;
          root_relax = round_relax;
          root_sbasis = round_sbasis;
          break;
        }
      } else if ((opt.verbose || force_root_cut_audit) &&
                 (added + cglp_added) > 0 && actual_added_rows <= 0) {
        fmt::print(stderr,
                   "[CUT-AUDIT] round {}: generators reported {} rows but no "
                   "valid rows were inserted\n",
                   k, added + cglp_added);
      }
      if (opt.verbose) {
        fmt::print(stderr, "[B&C CUT-GEN] round {}: added={} cglp={} gen={:.1f}ms\n",
                k, added, cglp_added,
                std::chrono::duration<double, std::milli>(t_cut_gen - t_cut_start).count());
      }
      if (actual_added_rows <= 0) {
        break;
      }
      if (opt.root_cut_reject_nonmoving_rows) {
        const Eigen::SparseMatrix<double, Eigen::RowMajor> root_cut_A_row =
            root_lp.A;
        const Eigen::SparseMatrix<double, Eigen::RowMajor> root_cut_Aeq_row =
            root_lp.Aeq;
        round_domain_tightened = node_bound_propagation(
            root_lp, root_cut_A_row, root_cut_Aeq_row, root.lb, root.ub,
            opt.bound_propagation_rounds);
        if (!bounds_consistent(root.lb, root.ub)) {
          if (opt.verbose || force_root_cut_audit) {
            fmt::print(stderr,
                       "[CUT-AUDIT] rejecting root cut round {}: row-induced "
                       "domain inconsistency before re-solve\n",
                       k);
          }
          truncate_ineq_rows(root_lp, prev_ineq_rows);
          root.lb = round_lb;
          root.ub = round_ub;
          apply_node_bounds(root_lp.vars, root.lb, root.ub);
          root.basis_hint = round_basis_hint;
          root.x_relax = round_x_relax;
          root.bound = round_bound;
          root_relax = round_relax;
          root_sbasis = round_sbasis;
          out.bc_stats.root_cut_rows_rejected_nonmoving +=
              static_cast<std::uint64_t>(actual_added_rows);
          ++out.bc_stats.root_cut_rounds_rejected_nonmoving;
          break;
        }
        if (round_domain_tightened > 0) {
          apply_node_bounds(root_lp.vars, root.lb, root.ub);
        }
      }
      out.bc_stats.cuts_added += actual_added_rows;
      if (opt.verbose) {
        MIPSOLVERS_LOG_DEBUG("[B&C] cut round {}: added {} cuts, re-solving (m_lp={}+{})...",
                k, added, static_cast<int>(root_lp.A.rows()), static_cast<int>(root_lp.Aeq.rows()));
      }

      // The direct HiGHS root object is the authoritative mutable LP state.
      // Keep the round snapshot open until cut admission is decided, since a
      // numerically valid solve can still be rejected as nonmoving.
      std::shared_ptr<BasisOps> root_cut_transaction;
      bool persistent_cut_resolved = false;
      if (!root_cut_ipm && round_basis_hint &&
          round_basis_hint->cached_sparse_basis &&
          round_basis_hint->cached_sparse_basis->supports_incremental_rows() &&
          round_basis_hint->cached_sparse_basis->begin_lp_transaction()) {
        root_cut_transaction = round_basis_hint->cached_sparse_basis;
        BCOptions cut_solve_opt = root_solve_opt;
        cut_solve_opt.use_ipm_root = false;
        cut_solve_opt.use_simplex_lp_nodes = true;
        cut_solve_opt.time_limit_sec = std::max(
            0.001,
            std::min(bc_remaining_sec() - bc_post_root_finalize_reserve_sec,
                     cut_stage_remaining_sec()));
        LPRelaxationResult incremental_relax = solve_lp_relaxation(
            root_lp, &round_x_relax, round_basis_hint.get(), cut_solve_opt);
        if (incremental_relax.primal.stats.success &&
            incremental_relax.primal.x.size() == n &&
            std::isfinite(incremental_relax.primal.stats.objective)) {
          ++out.bc_stats.lp_solves;

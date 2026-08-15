// @file bc_run/04_root_relaxation_c.inc
// BCSolveState::run() segment 4/14: root LP relaxation / cut loop — part 3.
// Textual include preserving run()'s scope; part of the branch_and_cut.cpp split.
// Not compiled standalone.
	      const double cost =
	          j < static_cast<int>(highs_lp.col_cost_.size())
	              ? highs_lp.col_cost_[static_cast<std::size_t>(j)]
	              : 0.0;
	      out_lp.c[j] = native_sense == Sense::Maximize ? -cost : cost;
	      auto& v = out_lp.vars[static_cast<std::size_t>(j)];
	      v.lb = lb <= -kHighsInf / 2 ? -std::numeric_limits<double>::infinity()
	                                  : lb;
	      v.ub = ub >= kHighsInf / 2 ? std::numeric_limits<double>::infinity()
	                                 : ub;
	      HighsVarType type = HighsVarType::kContinuous;
	      if (j < static_cast<int>(highs_lp.integrality_.size())) {
	        type = highs_lp.integrality_[static_cast<std::size_t>(j)];
	      }
	      switch (type) {
	        case HighsVarType::kInteger:
	        case HighsVarType::kSemiInteger:
	          v.type = std::abs(v.lb) <= highs_mip_feastol &&
	                           std::abs(v.ub - 1.0) <= highs_mip_feastol
	                       ? VarType::Binary
	                       : VarType::Integer;
	          break;
	        case HighsVarType::kImplicitInteger:
	        case HighsVarType::kContinuous:
	        case HighsVarType::kSemiContinuous:
	          v.type = VarType::Continuous;
	          break;
	      }
	      v.name = j < static_cast<int>(highs_lp.col_names_.size())
	                   ? highs_lp.col_names_[static_cast<std::size_t>(j)]
	                   : std::string();
	    }

	    std::vector<Eigen::Triplet<double>> triplets;
	    triplets.reserve(static_cast<std::size_t>(highs_lp.a_matrix_.numNz()));
	    HighsSparseMatrix mat = highs_lp.a_matrix_;
	    mat.ensureColwise();
	    if (static_cast<int>(mat.start_.size()) >= cols + 1) {
	      for (int col = 0; col < cols; ++col) {
	        const HighsInt start = mat.start_[static_cast<std::size_t>(col)];
	        const HighsInt end = mat.start_[static_cast<std::size_t>(col + 1)];
	        for (HighsInt p = start; p < end; ++p) {
	          if (p < 0 || p >= static_cast<HighsInt>(mat.index_.size()) ||
	              p >= static_cast<HighsInt>(mat.value_.size())) {
	            continue;
	          }
	          const int row = static_cast<int>(mat.index_[static_cast<std::size_t>(p)]);
	          const double value = mat.value_[static_cast<std::size_t>(p)];
	          if (row >= 0 && row < rows && value != 0.0) {
	            triplets.emplace_back(row, col, value);
	          }
	        }
	      }
	    }
	    out_lp.A.resize(std::max(0, rows), std::max(0, cols));
	    out_lp.A.setFromTriplets(triplets.begin(), triplets.end());
	    out_lp.A.makeCompressed();
	    out_lp.b = Eigen::VectorXd::Constant(
	        std::max(0, rows), std::numeric_limits<double>::infinity());
	    out_lp.row_lhs = Eigen::VectorXd::Constant(
	        std::max(0, rows), -std::numeric_limits<double>::infinity());
	    for (int r = 0; r < rows; ++r) {
	      const double lhs =
	          r < static_cast<int>(highs_lp.row_lower_.size())
	              ? highs_lp.row_lower_[static_cast<std::size_t>(r)]
	              : -std::numeric_limits<double>::infinity();
	      const double rhs =
	          r < static_cast<int>(highs_lp.row_upper_.size())
	              ? highs_lp.row_upper_[static_cast<std::size_t>(r)]
	              : std::numeric_limits<double>::infinity();
	      out_lp.row_lhs[r] =
	          lhs <= -kHighsInf / 2 ? -std::numeric_limits<double>::infinity()
	                                : lhs;
	      out_lp.b[r] =
	          rhs >= kHighsInf / 2 ? std::numeric_limits<double>::infinity()
	                               : rhs;
	    }
	    out_lp.Aeq.resize(0, std::max(0, cols));
	    out_lp.beq.resize(0);
	    return out_lp;
	  };

	  auto root_highs_final_state_round =
	      [&](const std::shared_ptr<Highs>& owner,
	          const HighsLpRelaxation* lprelax,
	          const LPModel& authority_lp)
	          -> std::optional<RootHighsXpoolResolveReplayRound> {
	    if (!owner || lprelax == nullptr) return std::nullopt;
	    const Highs* lpsolver = &lprelax->getLpSolver();
	    const HighsLp& h_lp = lpsolver->getLp();
	    const HighsSolution& sol = lpsolver->getSolution();
	    const HighsBasis& basis = lpsolver->getBasis();
	    const int cols = static_cast<int>(h_lp.num_col_);
	    const int rows = static_cast<int>(h_lp.num_row_);
	    if (cols <= 0 || rows < 0 ||
	        static_cast<int>(sol.col_value.size()) < cols ||
	        !basis.valid ||
	        static_cast<int>(basis.col_status.size()) < cols ||
	        static_cast<int>(basis.row_status.size()) < rows) {
	      return std::nullopt;
	    }
	    RootHighsXpoolResolveReplayRound round;
	    round.resolve.status = root_highs_live_resolve_status_name(
	        lprelax->getStatus());
	    round.resolve.obj = lprelax->getObjective();
	    round.resolve.rows = rows;
	    round.resolve.cols = cols;
	    round.resolve.nnz = static_cast<int>(h_lp.a_matrix_.numNz());
	    round.resolve.iter_total = static_cast<int>(std::min<int64_t>(
	        lprelax->getNumLpIterations(), std::numeric_limits<int>::max()));
	    round.resolve.iter_delta = round.resolve.iter_total;
	    const RootXpoolStateSignature sig = root_xpool_state_signature_from_highs(
	        authority_lp, sol, basis, cols, root_implied_integer_cols,
	        highs_mip_feastol);
	    round.resolve.frac = sig.fractional;
	    round.resolve.frontier_hash = sig.frontier_hash;
	    round.resolve.status_hash = sig.status_hash;
	    round.resolve.basis_hash = sig.basis_hash;
	    round.resolve.basic_cols = sig.basic_cols;
	    round.resolve.lower_cols = sig.lower_cols;
	    round.resolve.upper_cols = sig.upper_cols;
	    round.resolve.zero_cols = sig.zero_cols;
	    round.resolve.other_cols = sig.other_cols;
	    round.col_status.assign(static_cast<std::size_t>(cols), '?');
	    round.col_value.assign(static_cast<std::size_t>(cols),
	                           std::numeric_limits<double>::quiet_NaN());
	    for (int j = 0; j < cols; ++j) {
	      round.col_status[static_cast<std::size_t>(j)] =
	          root_highs_status_char(basis.col_status[static_cast<std::size_t>(j)]);
	      round.col_value[static_cast<std::size_t>(j)] =
	          sol.col_value[static_cast<std::size_t>(j)];
	    }
	    round.row_status.assign(static_cast<std::size_t>(rows), '?');
	    round.row_value.assign(static_cast<std::size_t>(rows),
	                           std::numeric_limits<double>::quiet_NaN());
	    for (int r = 0; r < rows; ++r) {
	      round.row_status[static_cast<std::size_t>(r)] =
	          root_highs_status_char(basis.row_status[static_cast<std::size_t>(r)]);
	      if (r < static_cast<int>(sol.row_value.size())) {
	        round.row_value[static_cast<std::size_t>(r)] =
	            sol.row_value[static_cast<std::size_t>(r)];
	      }
	    }
	    return round;
	  };

      struct RootSourceLoopTrace {
        int rounds{0};
        int ib_calls{0};
        int tableau_calls{0};
        int path_calls{0};
        int modk_calls{0};
        int flush_calls{0};
        int prop_calls{0};
        int xpool_calls{0};
        int xpool_resolve_calls{0};
        int xpool_resolve_ok{0};
        int xpool_selected{0};
        int xpool_rows{0};
        double ib_ms{0.0};
        double tableau_ms{0.0};
        double tableau_generate_ms{0.0};
        double tableau_pool_add_ms{0.0};
        double path_ms{0.0};
        double path_generate_ms{0.0};
        double path_pool_add_ms{0.0};
        double modk_ms{0.0};
        double modk_generate_ms{0.0};
        double modk_pool_add_ms{0.0};
        double flush_ms{0.0};
        double prop_ms{0.0};
        double xpool_ms{0.0};
        double xpool_select_ms{0.0};
        double xpool_build_ms{0.0};
        double xpool_sf_ms{0.0};
        double xpool_solve_ms{0.0};
        double xpool_apply_ms{0.0};
      };

      struct RootCutpoolResolveResult {
        bool lp_resolved{false};
        int admitted_rows{0};
        double lift{0.0};
        RootTransformedPoolStats pool_stats;
      };

	      RootSourceLoopTrace root_source_trace;
	      std::uint64_t root_source_epoch = 1;
	      std::uint64_t root_xpool_noop_epoch = 0;
	      std::size_t root_xpool_noop_pool_size = std::numeric_limits<std::size_t>::max();
	      struct RootSourceSignature {
	        bool valid{false};
	        std::uint64_t lp_state{0};
	        std::uint64_t source_state{0};
	        int fractional_integer_like{0};
	        int frontier_cols{0};
	        int source_frontier_cols{0};
	        std::size_t varbounds{0};
	        std::size_t implications{0};
	        std::size_t total_varbounds{0};
	        std::size_t total_implications{0};

	        bool same_as(const RootSourceSignature& other) const {
	          return valid && other.valid && lp_state == other.lp_state &&
	                 source_state == other.source_state &&
	                 fractional_integer_like == other.fractional_integer_like &&
	                 frontier_cols == other.frontier_cols &&
	                 source_frontier_cols == other.source_frontier_cols &&
	                 varbounds == other.varbounds &&
	                 implications == other.implications;
	        }
	      };
	      RootSourceSignature root_tableau_generated_signature;
	      RootSourceSignature root_path_generated_signature;

	      auto root_source_elapsed_ms =
	          [](std::chrono::steady_clock::time_point start) -> double {
	        return std::chrono::duration<double, std::milli>(
	                   std::chrono::steady_clock::now() - start)
	            .count();
	      };

	      auto root_source_hash_double = [&](double value) -> std::uint64_t {
	        if (std::isinf(value)) {
	          return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
	        }
	        if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
	        const double grid = std::max(1e-8, opt.lp_tol);
	        return static_cast<std::uint64_t>(
	            static_cast<std::int64_t>(std::llround(value / grid)));
	      };

	      auto compute_root_source_signature = [&]() -> RootSourceSignature {
	        RootSourceSignature sig;
		        const Eigen::VectorXd& source_x = root_source_solution();
		        if (source_x.size() != n || root.lb.size() != n ||
		            root.ub.size() != n) {
		          return sig;
		        }
	        sig.valid = true;
	        sig.total_varbounds = variable_bound_table.size();
	        sig.total_implications = implication_graph.size();

		        std::vector<unsigned char> frontier(static_cast<std::size_t>(n), 0);
		        std::vector<unsigned char> source_frontier(static_cast<std::size_t>(n), 0);
            const RootHighsFractionalFrontier highs_frontier =
                compute_root_highs_fractional_frontier();
            std::vector<int> frontier_cols = highs_frontier.cols;
	        auto mark_source_frontier = [&](int j) {
	          if (j < 0 || j >= n) return;
	          unsigned char& flag = source_frontier[static_cast<std::size_t>(j)];
	          if (flag == 0) {
	            flag = 1;
	            ++sig.source_frontier_cols;
	          }
	        };
		        for (int j : frontier_cols) {
              if (j < 0 || j >= n) continue;
		          frontier[static_cast<std::size_t>(j)] = 1;
		          ++sig.fractional_integer_like;
		          mark_source_frontier(j);
		        }
	        sig.frontier_cols = static_cast<int>(frontier_cols.size());

	        for (int j : frontier_cols) {
	          if (j < variable_bound_table.num_vars()) {
	            for (const auto& entry : variable_bound_table.vubs(j)) {
	              mark_source_frontier(entry.trigger_col);
	            }
	            for (const auto& entry : variable_bound_table.vlbs(j)) {
	              mark_source_frontier(entry.trigger_col);
	            }
	          }
	          for (int side = 0; side < 2; ++side) {
	            const bool trigger_one = side != 0;
	            auto range = implication_graph.implications(j, trigger_one);
	            for (const BinaryImplicationGraph::Implication* p = range.first;
	                 p != range.second; ++p) {
	              if (p == nullptr) continue;
	              mark_source_frontier(p->var_idx);
	            }
	          }
	        }
	        const int table_cols =
	            std::min(n, variable_bound_table.num_vars());
	        for (int target = 0; target < table_cols; ++target) {
	          for (const auto& entry : variable_bound_table.vubs(target)) {
	            if ((entry.trigger_col >= 0 && entry.trigger_col < n &&
	                 frontier[static_cast<std::size_t>(entry.trigger_col)] != 0) ||
	                frontier[static_cast<std::size_t>(target)] != 0) {
	              mark_source_frontier(target);
	              mark_source_frontier(entry.trigger_col);
	            }
	          }
	          for (const auto& entry : variable_bound_table.vlbs(target)) {
	            if ((entry.trigger_col >= 0 && entry.trigger_col < n &&
	                 frontier[static_cast<std::size_t>(entry.trigger_col)] != 0) ||
	                frontier[static_cast<std::size_t>(target)] != 0) {
	              mark_source_frontier(target);
	              mark_source_frontier(entry.trigger_col);
	            }
	          }
	        }
	        for (int trigger = 0; trigger < n; ++trigger) {
	          for (int side = 0; side < 2; ++side) {
	            const bool trigger_one = side != 0;
	            auto range = implication_graph.implications(trigger, trigger_one);
	            for (const BinaryImplicationGraph::Implication* p = range.first;
	                 p != range.second; ++p) {
	              if (p == nullptr || p->var_idx < 0 || p->var_idx >= n) continue;
	              if (frontier[static_cast<std::size_t>(trigger)] != 0 ||
	                  frontier[static_cast<std::size_t>(p->var_idx)] != 0) {
	                mark_source_frontier(trigger);
	                mark_source_frontier(p->var_idx);
	              }
	            }
	          }
	        }

	        std::uint64_t lp_hash = 0x4841434452534c50ULL;
	        lp_hash = bc_trace_hash_mix(lp_hash,
	                                    root_source_hash_double(root.bound));
	        const std::vector<char> basic =
	            bc_original_basic_mask(n, root.basis_hint.get());
	        for (int j = 0; j < n; ++j) {
	          if (source_frontier[static_cast<std::size_t>(j)] == 0) continue;
	          const char status = bc_native_basis_status(
	              j, basic, root.basis_hint.get());
	          lp_hash = bc_trace_hash_mix(lp_hash,
	                                      static_cast<std::uint64_t>(j));
	          lp_hash = bc_trace_hash_mix(lp_hash,
	                                      static_cast<std::uint64_t>(status));
		          lp_hash = bc_trace_hash_mix(lp_hash,
		                                      root_source_hash_double(source_x[j]));
	          lp_hash = bc_trace_hash_mix(lp_hash,
	                                      root_source_hash_double(root.lb[j]));
	          lp_hash = bc_trace_hash_mix(lp_hash,
	                                      root_source_hash_double(root.ub[j]));
	          const double rc = bc_native_reduced_cost(j, root.basis_hint.get());
	          lp_hash = bc_trace_hash_mix(lp_hash, root_source_hash_double(rc));
	        }
	        sig.lp_state = lp_hash;

	        std::uint64_t source_hash = 0x4841434452535352ULL;
	        for (int col = 0; col < table_cols; ++col) {
	          for (const auto& entry : variable_bound_table.vubs(col)) {
	            const bool relevant =
	                source_frontier[static_cast<std::size_t>(col)] != 0 ||
	                (entry.trigger_col >= 0 && entry.trigger_col < n &&
	                 source_frontier[static_cast<std::size_t>(
	                     entry.trigger_col)] != 0);
	            if (!relevant) continue;
	            ++sig.varbounds;
	            source_hash = bc_trace_hash_mix(
	                source_hash, static_cast<std::uint64_t>(2 * col));
	            source_hash = bc_trace_hash_mix(
	                source_hash, static_cast<std::uint64_t>(entry.trigger_col));
	            source_hash = bc_trace_hash_mix(
	                source_hash, root_source_hash_double(entry.bound.coef));
	            source_hash = bc_trace_hash_mix(
	                source_hash, root_source_hash_double(entry.bound.constant));
	          }
	          for (const auto& entry : variable_bound_table.vlbs(col)) {
	            const bool relevant =
	                source_frontier[static_cast<std::size_t>(col)] != 0 ||
	                (entry.trigger_col >= 0 && entry.trigger_col < n &&
	                 source_frontier[static_cast<std::size_t>(
	                     entry.trigger_col)] != 0);
	            if (!relevant) continue;
	            ++sig.varbounds;
	            source_hash = bc_trace_hash_mix(
	                source_hash, static_cast<std::uint64_t>(2 * col + 1));
	            source_hash = bc_trace_hash_mix(
	                source_hash, static_cast<std::uint64_t>(entry.trigger_col));
	            source_hash = bc_trace_hash_mix(
	                source_hash, root_source_hash_double(entry.bound.coef));
	            source_hash = bc_trace_hash_mix(
	                source_hash, root_source_hash_double(entry.bound.constant));
	          }
	        }
	        for (int trigger = 0; trigger < n; ++trigger) {
	          for (int side = 0; side < 2; ++side) {
	            const bool trigger_one = side != 0;
	            auto range =
	                implication_graph.implications(trigger, trigger_one);
	            for (const BinaryImplicationGraph::Implication* p = range.first;
	                 p != range.second; ++p) {
	              if (p == nullptr) continue;
	              const bool relevant =
	                  source_frontier[static_cast<std::size_t>(trigger)] != 0 ||
	                  (p->var_idx >= 0 && p->var_idx < n &&
	                   source_frontier[static_cast<std::size_t>(
	                       p->var_idx)] != 0);
	              if (!relevant) continue;
	              ++sig.implications;
	              source_hash = bc_trace_hash_mix(
	                  source_hash,
	                  static_cast<std::uint64_t>(2 * trigger + side));
	              source_hash = bc_trace_hash_mix(
	                  source_hash, static_cast<std::uint64_t>(p->var_idx));
	              source_hash = bc_trace_hash_mix(
	                  source_hash, static_cast<std::uint64_t>(p->is_lb));
	              source_hash = bc_trace_hash_mix(
	                  source_hash, root_source_hash_double(p->value));
	            }
	          }
	        }
	        sig.source_state = source_hash;
	        return sig;
	      };

	      auto root_transformed_cutpool_mark_lp_removed = [&](int cut_index) {
	        if (cut_index < 0 ||
	            cut_index >= static_cast<int>(root_transformed_cut_in_lp.size())) {
	          return;
	        }
	        // HiGHS CutPool::lpCutRemoved(): a removed LP cut re-enters the same
	        // cutpool as an available propagation row with age 1.  It does not
	        // call CutpoolPropagation::cutAdded() or markPropagateCut(); pending
	        // propagation remains purely changed-column/event driven.
	        root_transformed_cut_in_lp[static_cast<std::size_t>(cut_index)] = 0;
	        if (cut_index < static_cast<int>(root_transformed_cut_age.size())) {
	          root_transformed_cut_age[static_cast<std::size_t>(cut_index)] = 1;
	        }
	      };

      auto erase_root_transformed_cutpool_hash_entry = [&](int cut_index) {
        if (cut_index < 0 ||
            cut_index >= static_cast<int>(root_transformed_cutpool.size())) {
          return;
        }
        const auto hash = static_cast<std::uint64_t>(
            root_transformed_cutpool[static_cast<std::size_t>(cut_index)].hash);
        const auto range = root_transformed_cutpool_hash_to_cut.equal_range(hash);
        for (auto it = range.first; it != range.second; ++it) {
          if (it->second == cut_index) {
            root_transformed_cutpool_hash_to_cut.erase(it);
            return;
          }
        }
      };

		  auto add_root_transformed_cutpool_rows =
	      [&](std::vector<PoolCut>& rows, const char* family) -> int {
	    int added = 0;
	    int duplicate = 0;
        int source_rows = 0;
        int live_prop_rows = 0;
        int pending_prop_rows = 0;
        int live_pool_added = 0;
        int extracted_vb_rows = 0;
	    for (auto& row : rows) {
	      if (row.coeff.size() != n || row.coeff.nonZeros() <= 0 ||
	          !std::isfinite(row.rhs)) {
	        continue;
	      }
	      if (!(row.norm > 0.0) || !std::isfinite(row.norm)) {
	        row.norm = std::sqrt(std::max(0.0, row.coeff.squaredNorm()));
	      }
	      if (!(row.norm > 1e-12) || !std::isfinite(row.norm)) continue;
	      double maxabscoef = 0.0;
	      for (Eigen::SparseVector<double>::InnerIterator it(row.coeff); it; ++it) {
	        maxabscoef = std::max(maxabscoef, std::abs(it.value()));
	      }
	      if (!(maxabscoef > 0.0) || !std::isfinite(maxabscoef)) continue;
	      row.hash = static_cast<std::size_t>(
	          highs_style_cut_hash(row.coeff, maxabscoef));
	      bool is_duplicate = false;
	      const auto dup_range =
	          root_transformed_cutpool_hash_to_cut.equal_range(
	              static_cast<std::uint64_t>(row.hash));
	      for (auto dup_it = dup_range.first; dup_it != dup_range.second;
	           ++dup_it) {
	        const int keep_idx = dup_it->second;
	        if (keep_idx < 0 ||
	            keep_idx >= static_cast<int>(root_transformed_cutpool.size())) {
	          continue;
	        }
	        const PoolCut& keep =
	            root_transformed_cutpool[static_cast<std::size_t>(keep_idx)];
		        if (!sparse_same_support(keep.coeff, row.coeff) ||
		            !(keep.norm > 0.0)) {
		          continue;
		        }
		        const double par =
		            sparse_sparse_dot(keep.coeff, row.coeff) / (keep.norm * row.norm);
		        if (std::isfinite(par) && par >= 1.0 - 1e-6) {
		          is_duplicate = true;
		          break;
		        }
	      }
	      if (is_duplicate) {
	        ++duplicate;
	        continue;
	      }
          const bool request_cutpool_propagation =
              root_transformed_cutpool_propagate_requested(row, family);
          const bool appended_cutpool_prop_row =
              request_cutpool_propagation
                  ? append_root_transformed_cutpool_audit_row(row.coeff, row.rhs)
                  : false;
          if (appended_cutpool_prop_row) {
            ++source_rows;
          }
          if (!strict_highs_root_fixed_point &&
              appended_cutpool_prop_row) {
            ++root_transformed_cutpool_pending_source_rows;
            if (root_transformed_cutpool_live_vb_source) {
              append_variable_bound_sparse_cutpool_row(row.coeff, row.rhs);
            }
          }
          if (strict_highs_root_fixed_point && row.coeff.nonZeros() <= 100) {
            if (extract_variable_bounds_from_cutpool_row(row.coeff, row.rhs)) {
              ++extracted_vb_rows;
            }
          }
		          // HiGHS HighsCutPool::separate and HighsLpRelaxation::addCuts;
		          // ownership derivation in
		          // docs/native_milp_cutpool_proof_ownership_2026-08-13.md.
		          // The automatic HiGHS-style pipeline also uses this private pool even
		          // when the full strict contract is disabled.  Tree-global ownership
		          // starts only after successful root-LP admission.
		          if (!highs_style_cutpool_candidate_stays_root_owned(
		                  strict_highs_root_fixed_point,
		                  opt.auto_highs_root_pipeline)) {
		            bool new_domain_source = false;
		            if (cut_pool.add(row.coeff, row.rhs, /*domain_only=*/true,
		                             &new_domain_source)) {
	              ++live_pool_added;
	              if (new_domain_source) {
	                note_global_domain_learning();
	                if (!opt.auto_highs_root_pipeline) {
	                  note_objective_artifact_learning();
	                }
	              }
	            }
	          }
	      root_transformed_cutpool.push_back(std::move(row));
          const int new_cut_index =
              static_cast<int>(root_transformed_cutpool.size()) - 1;
        if (const char* add_trace_env =
                bc_env_options().value("MIPSOLVERS_XPOOL_ADD_TRACE")) {
          const std::string add_trace_value(add_trace_env);
          if (add_trace_value == "all" || add_trace_value == "rows") {
            auto trace_mix = [](std::uint64_t h,
                                std::uint64_t v) -> std::uint64_t {
              v ^= v >> 33;
              v *= std::uint64_t{0xff51afd7ed558ccd};
              v ^= v >> 33;
              v *= std::uint64_t{0xc4ceb9fe1a85ec53};
              v ^= v >> 33;
              return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} +
                          (h << 6) + (h >> 2));
            };
            const PoolCut& stored_row =
                root_transformed_cutpool.back();
            const int len = static_cast<int>(stored_row.coeff.nonZeros());
            std::uint64_t row_sig = trace_mix(
                stored_row.hash,
                static_cast<std::uint64_t>(
                    HighsHashHelpers::double_hash_code(stored_row.rhs)));
            row_sig = trace_mix(row_sig, static_cast<std::uint64_t>(len));
            int trace_terms = 0;
            if (const char* terms_env =
                    bc_env_options().value("MIPSOLVERS_XPOOL_ADD_TRACE_TERMS")) {
              char* end = nullptr;
              const long parsed = std::strtol(terms_env, &end, 10);
              if (end != terms_env && parsed > 0) {
                trace_terms =
                    static_cast<int>(std::min<long>(parsed, 200));
              }
            }
            if (trace_terms > 0) {
              std::string terms;
              int emitted_terms = 0;
              for (Eigen::SparseVector<double>::InnerIterator it(
                       stored_row.coeff);
                   it && emitted_terms < trace_terms; ++it, ++emitted_terms) {
                if (!terms.empty()) terms += ",";
                terms += fmt::format("{}:{:.17g}",
                                     static_cast<int>(it.index()), it.value());
              }
              fmt::print(stderr,
                         "[B&C ROOT-XPOOL-ADD-ROW] family={} idx={} len={} "
                         "rhs={:.17g} sourceId={} rowHash={:016x} rowSig={:016x} "
                         "terms=[{}]\n",
                         family != nullptr ? family : "unknown", new_cut_index,
                         len, stored_row.rhs, stored_row.source_trace_id,
                         stored_row.hash, row_sig, terms);
            } else {
              fmt::print(stderr,
                         "[B&C ROOT-XPOOL-ADD-ROW] family={} idx={} len={} "
                         "rhs={:.17g} sourceId={} rowHash={:016x} rowSig={:016x}\n",
                         family != nullptr ? family : "unknown", new_cut_index,
                         len, stored_row.rhs, stored_row.source_trace_id,
                         stored_row.hash, row_sig);
            }
          }
        }
	      root_transformed_cutpool_hash_to_cut.emplace(
	          static_cast<std::uint64_t>(
	              root_transformed_cutpool.back().hash),
              new_cut_index);
	      root_transformed_cut_in_lp.push_back(0);
	      root_transformed_cut_alive.push_back(1);
	      root_transformed_cut_propagate.push_back(
	          appended_cutpool_prop_row ? 1 : 0);
          root_transformed_cut_prop_pending.push_back(0);
	      root_transformed_cut_age.push_back(root_transformed_cutpool_initial_age);
          if (appended_cutpool_prop_row) {
            const PoolCut& stored_cut =
                root_transformed_cutpool[static_cast<std::size_t>(
                    new_cut_index)];
            for (Eigen::SparseVector<double>::InnerIterator it(
                     stored_cut.coeff);
                 it; ++it) {
              const int col = static_cast<int>(it.index());
              if (col >= 0 && col < n && std::abs(it.value()) > 1e-12) {
                root_transformed_cut_col_to_cuts[static_cast<std::size_t>(col)]
                    .push_back(new_cut_index);
              }
            }
	            if (strict_highs_root_fixed_point && root.lb.size() == n &&
	                root.ub.size() == n) {
                  const int pending_before =
                      static_cast<int>(root_transformed_cut_prop_queue.size());
	              mark_root_transformed_cut_prop_pending(new_cut_index, root.lb,
	                                                     root.ub);
                  pending_prop_rows +=
                      static_cast<int>(root_transformed_cut_prop_queue.size()) -
                      pending_before;
	            }
          }
	      if (appended_cutpool_prop_row) ++live_prop_rows;
	      ++added;
	    }
	    if (bc_env_options().value("MIPSOLVERS_XPOOL_ADD_TRACE") != nullptr) {
		      fmt::print(stderr,
		                 "[B&C ROOT-XPOOL-ADD] family={} generated={} added={} "
		                 "duplicate={} srcRows={} liveProp={} pendingProp={} "
                         "pendingSrc={} vbExtract={} pool={}\n",
		                 family != nullptr ? family : "unknown",
		                 static_cast<int>(rows.size()), added, duplicate,
	                     source_rows, live_prop_rows, pending_prop_rows,
	                     root_transformed_cutpool_pending_source_rows,
                       extracted_vb_rows,
		                 static_cast<int>(root_transformed_cutpool.size()));
	    }
	        (void)live_pool_added;
		    return added;
		  };

	      auto root_inequality_row_slack_basic =
	          [&](const LPRelaxationResult& relax, int lp_row) -> bool {
	        if (lp_row < 0 || lp_row >= static_cast<int>(root_lp.A.rows()) ||
	            !relax.simplex) {
	          return false;
	        }
	        const StandardFormLP& sf = relax.simplex->form;
	        if (lp_row >= static_cast<int>(sf.row_to_slack_col.size())) {
	          return false;
	        }
	        const int slack = sf.row_to_slack_col[static_cast<std::size_t>(lp_row)];
	        if (slack < 0) return false;
	        const auto& basis_indices = relax.simplex->basis.basis_indices();
	        if (lp_row >= static_cast<int>(basis_indices.size())) return false;
	        return basis_indices[static_cast<std::size_t>(lp_row)] == slack;
	      };

	      auto build_root_lp_without_ineq_rows =
	          [&](const std::vector<char>& delete_row,
	              std::vector<int>& row_map,
	              LPModel& compact_lp) -> bool {
	        const int old_m = static_cast<int>(root_lp.A.rows());
	        const int ncols = static_cast<int>(root_lp.A.cols());
	        if (old_m <= 0 || static_cast<int>(delete_row.size()) != old_m) {
	          return false;
	        }
	        row_map.assign(static_cast<std::size_t>(old_m), -1);
	        int new_m = 0;
	        for (int r = 0; r < old_m; ++r) {
	          if (!delete_row[static_cast<std::size_t>(r)]) {
	            row_map[static_cast<std::size_t>(r)] = new_m++;
	          }
	        }
	        if (new_m == old_m) return false;
	        compact_lp = root_lp;
	        Eigen::SparseMatrix<double> compact_A(new_m, ncols);
	        std::vector<Eigen::Triplet<double>> tri;
	        tri.reserve(static_cast<std::size_t>(root_lp.A.nonZeros()));
	        for (int col = 0; col < root_lp.A.outerSize(); ++col) {
	          for (Eigen::SparseMatrix<double>::InnerIterator it(root_lp.A, col);
	               it; ++it) {
	            const int mapped =
	                row_map[static_cast<std::size_t>(static_cast<int>(it.row()))];
	            if (mapped >= 0) tri.emplace_back(mapped, col, it.value());
	          }
	        }
	        compact_A.setFromTriplets(tri.begin(), tri.end());
	        compact_lp.A = std::move(compact_A);
	        compact_lp.b.resize(new_m);
	        const bool had_lhs = lp_has_row_lhs(root_lp);
	        Eigen::VectorXd compact_lhs;
	        if (had_lhs) compact_lhs.resize(new_m);
	        for (int r = 0; r < old_m; ++r) {
	          const int mapped = row_map[static_cast<std::size_t>(r)];
	          if (mapped < 0) continue;
	          compact_lp.b[mapped] = root_lp.b[r];
	          if (had_lhs) compact_lhs[mapped] = root_lp.row_lhs[r];
	        }
	        if (had_lhs) {
	          compact_lp.row_lhs = std::move(compact_lhs);
	        } else {
	          compact_lp.row_lhs.resize(0);
	        }
	        return true;
	      };

	      auto make_root_compacted_basis_hint =
	          [&](const StandardFormLP& old_sf,
	              const StandardFormLP& compact_sf,
	              const std::vector<int>& row_map)
	              -> std::shared_ptr<SimplexBasis> {
	        const SimplexBasis* old_basis = root.basis_hint.get();
	        if (old_basis == nullptr ||
	            static_cast<int>(old_basis->index_count()) !=
	                static_cast<int>(old_sf.A.rows()) ||
	            old_basis->rows != static_cast<int>(old_sf.A.rows()) ||
	            old_basis->cols != static_cast<int>(old_sf.A.cols()) ||
	            static_cast<int>(row_map.size()) !=
	                static_cast<int>(old_sf.A.rows())) {
	          return nullptr;
	        }
	        const int old_rows = static_cast<int>(old_sf.A.rows());
	        const int old_cols = static_cast<int>(old_sf.A.cols());
	        const int new_rows = static_cast<int>(compact_sf.A.rows());
	        const int new_cols = static_cast<int>(compact_sf.A.cols());
	        if (new_rows <= 0 || new_cols <= 0 ||
	            compact_sf.n_original != old_sf.n_original ||
	            compact_sf.n_surplus != old_sf.n_surplus ||
	            compact_sf.n_artificial != old_sf.n_artificial ||
	            old_sf.n_slack - compact_sf.n_slack != old_rows - new_rows) {
	          return nullptr;
	        }
	        const int n_orig = compact_sf.n_original;
	        std::vector<int> old_col_to_new(static_cast<std::size_t>(old_cols),
	                                        -1);
	        for (int j = 0; j < std::min(old_cols, n_orig); ++j) {
	          old_col_to_new[static_cast<std::size_t>(j)] = j;
	        }
	        auto map_logical = [&](const std::vector<int>& old_map,
	                               const std::vector<int>& new_map) {
	          for (int r = 0; r < old_rows; ++r) {
	            const int mapped_row = row_map[static_cast<std::size_t>(r)];
	            const int old_col =
	                r < static_cast<int>(old_map.size())
	                    ? old_map[static_cast<std::size_t>(r)]
	                    : -1;
	            if (old_col < 0 || old_col >= old_cols || mapped_row < 0 ||
	                mapped_row >= static_cast<int>(new_map.size())) {
	              continue;
	            }
	            const int new_col =
	                new_map[static_cast<std::size_t>(mapped_row)];
	            if (new_col >= 0 && new_col < new_cols) {
	              old_col_to_new[static_cast<std::size_t>(old_col)] = new_col;
	            }
	          }
	        };
	        map_logical(old_sf.row_to_slack_col, compact_sf.row_to_slack_col);
	        map_logical(old_sf.row_to_surplus_col, compact_sf.row_to_surplus_col);
	        map_logical(old_sf.row_to_artificial_col,
	                    compact_sf.row_to_artificial_col);

	        const auto& old_indices = old_basis->basis_indices();
	        auto hint = std::make_shared<SimplexBasis>();
	        hint->rows = new_rows;
	        hint->cols = new_cols;
	        hint->indices.assign(static_cast<std::size_t>(new_rows), -1);
	        hint->at_upper.assign(static_cast<std::size_t>(new_cols), 0);
	        hint->sf_n_slack = compact_sf.n_slack;
	        hint->sf_n_surplus = compact_sf.n_surplus;
	        hint->sf_n_artificial = compact_sf.n_artificial;
	        std::vector<char> basic_seen(static_cast<std::size_t>(new_cols), 0);
	        for (int r = 0; r < old_rows; ++r) {
	          const int mapped_row = row_map[static_cast<std::size_t>(r)];
	          if (mapped_row < 0) continue;
	          const int old_col = old_indices[static_cast<std::size_t>(r)];
	          const int new_col =
	              old_col >= 0 && old_col < old_cols
	                  ? old_col_to_new[static_cast<std::size_t>(old_col)]
	                  : -1;
	          int basis_col = new_col;
	          if (basis_col < 0 || basis_col >= new_cols ||
	              basic_seen[static_cast<std::size_t>(basis_col)] != 0) {
	            basis_col =
	                compact_sf.row_to_slack_col[static_cast<std::size_t>(
	                    mapped_row)];
	            if (basis_col < 0) {
	              basis_col =
	                  compact_sf.row_to_artificial_col[static_cast<std::size_t>(
	                      mapped_row)];
	            }
	          }
	          if (basis_col < 0 || basis_col >= new_cols ||
	              basic_seen[static_cast<std::size_t>(basis_col)] != 0) {
	            return nullptr;
	          }
	          hint->indices[static_cast<std::size_t>(mapped_row)] = basis_col;
	          basic_seen[static_cast<std::size_t>(basis_col)] = 1;
	        }
	        for (int r = 0; r < new_rows; ++r) {
	          if (hint->indices[static_cast<std::size_t>(r)] >= 0) continue;
	          int basis_col =
	              compact_sf.row_to_slack_col[static_cast<std::size_t>(r)];
	          if (basis_col < 0) {
	            basis_col =
	                compact_sf.row_to_artificial_col[static_cast<std::size_t>(
	                    r)];
	          }
	          if (basis_col < 0 || basis_col >= new_cols ||
	              basic_seen[static_cast<std::size_t>(basis_col)] != 0) {
	            return nullptr;
	          }
	          hint->indices[static_cast<std::size_t>(r)] = basis_col;
	          basic_seen[static_cast<std::size_t>(basis_col)] = 1;
	        }
	        const int copy_upper =
	            std::min(old_cols, static_cast<int>(old_basis->at_upper.size()));
	        for (int old_col = 0; old_col < copy_upper; ++old_col) {
	          const int new_col = old_col_to_new[static_cast<std::size_t>(
	              old_col)];
	          if (new_col >= 0 && new_col < new_cols) {
	            hint->at_upper[static_cast<std::size_t>(new_col)] =
	                old_basis->at_upper[static_cast<std::size_t>(old_col)];
	          }
	        }
	        return hint;
	      };

	      auto build_compacted_scaled_sf_from_root_state =
	          [&](const StandardFormLP& old_sf,
	              const std::vector<int>& sf_row_map,
	              std::vector<int>& sf_col_map,
	              std::vector<int>& deleted_sf_rows,
	              std::vector<int>& deleted_sf_cols,
	              StandardFormLP& compact_sf) -> bool {
	        const int old_rows = static_cast<int>(old_sf.A.rows());
	        const int old_cols = static_cast<int>(old_sf.A.cols());
	        if (old_rows <= 0 || old_cols <= 0 ||
	            static_cast<int>(sf_row_map.size()) != old_rows) {
	          return false;
	        }
	        const int new_rows =
	            1 + *std::max_element(sf_row_map.begin(), sf_row_map.end());
	        if (new_rows <= 0) return false;
	        sf_col_map.assign(static_cast<std::size_t>(old_cols), -1);
	        for (int j = 0; j < std::min(old_cols, old_sf.n_original); ++j) {
	          sf_col_map[static_cast<std::size_t>(j)] = j;
	        }
	        auto map_logical = [&](const std::vector<int>& old_map) {
	          for (int old_r = 0; old_r < old_rows; ++old_r) {
	            const int old_col =
	                old_r < static_cast<int>(old_map.size())
	                    ? old_map[static_cast<std::size_t>(old_r)]
	                    : -1;
	            const int new_r = sf_row_map[static_cast<std::size_t>(old_r)];
	            if (old_col < 0 || old_col >= old_cols) continue;
	            if (new_r >= 0) {
	              sf_col_map[static_cast<std::size_t>(old_col)] = -2 - new_r;
	            }
	          }
	        };
	        map_logical(old_sf.row_to_slack_col);
	        map_logical(old_sf.row_to_surplus_col);
	        map_logical(old_sf.row_to_artificial_col);
	        int next_col = old_sf.n_original;
	        for (int old_col = old_sf.n_original; old_col < old_cols; ++old_col) {
	          int& slot = sf_col_map[static_cast<std::size_t>(old_col)];
	          if (slot < -1) slot = next_col++;
	        }
	        const int new_cols = next_col;
	        deleted_sf_rows.clear();
	        for (int old_r = 0; old_r < old_rows; ++old_r) {
	          if (sf_row_map[static_cast<std::size_t>(old_r)] < 0) {
	            deleted_sf_rows.push_back(old_r);
	          }
	        }
	        deleted_sf_cols.clear();
	        for (int old_col = 0; old_col < old_cols; ++old_col) {
	          if (sf_col_map[static_cast<std::size_t>(old_col)] < 0) {
	            deleted_sf_cols.push_back(old_col);
	          }
	        }
	        for (int old_col = 0; old_col < old_cols; ++old_col) {
	          int& slot = sf_col_map[static_cast<std::size_t>(old_col)];
	          if (slot < -1) slot = -slot - 2 + old_sf.n_original;
	        }

	        compact_sf = old_sf;
	        compact_sf.A.resize(new_rows, new_cols);
	        std::vector<Eigen::Triplet<double>> tri;
	        tri.reserve(static_cast<std::size_t>(old_sf.A.nonZeros()));
	        for (int old_col = 0; old_col < old_sf.A.outerSize(); ++old_col) {
	          const int new_col = sf_col_map[static_cast<std::size_t>(
	              old_col)];
	          if (new_col < 0) continue;
	          for (StandardColumnMatrix::InnerIterator it(old_sf.A,
	                                                             old_col);
	               it; ++it) {
	            const int new_row =
	                sf_row_map[static_cast<std::size_t>(
	                    static_cast<int>(it.row()))];
	            if (new_row >= 0) tri.emplace_back(new_row, new_col, it.value());
	          }
	        }
	        compact_sf.A.setFromTriplets(tri.begin(), tri.end());
	        compact_sf.A.makeCompressed();
	        compact_sf.A_row = compact_sf.A;

	        auto remap_vec = [&](const Eigen::VectorXd& old_v) {
	          Eigen::VectorXd v(new_rows);
	          for (int old_r = 0; old_r < old_rows; ++old_r) {
	            const int new_r = sf_row_map[static_cast<std::size_t>(old_r)];
	            if (new_r >= 0) v[new_r] = old_v[old_r];
	          }
	          return v;
	        };
	        auto remap_col_vec = [&](const Eigen::VectorXd& old_v) {
	          Eigen::VectorXd v(new_cols);
	          for (int old_c = 0; old_c < old_cols; ++old_c) {
	            const int new_c = sf_col_map[static_cast<std::size_t>(old_c)];
	            if (new_c >= 0) v[new_c] = old_v[old_c];
	          }
	          return v;
	        };
	        compact_sf.b = remap_vec(old_sf.b);
	        compact_sf.c_max = remap_col_vec(old_sf.c_max);
	        compact_sf.var_ub = remap_col_vec(old_sf.var_ub);
	        compact_sf.row_rhs_value =
	            old_sf.row_rhs_value.size() == old_rows
	                ? remap_vec(old_sf.row_rhs_value)
	                : Eigen::VectorXd();
	        compact_sf.row_scale =
	            old_sf.row_scale.size() == old_rows
	                ? remap_vec(old_sf.row_scale)
	                : Eigen::VectorXd();
	        compact_sf.col_scale =
	            old_sf.col_scale.size() == old_cols
	                ? remap_col_vec(old_sf.col_scale)
	                : Eigen::VectorXd();
	        compact_sf.n_slack = static_cast<int>(root_lp.A.rows()) -
	                             static_cast<int>(deleted_sf_rows.size());
	        compact_sf.n_surplus = old_sf.n_surplus;
	        compact_sf.n_artificial = old_sf.n_artificial;
	        compact_sf.row_to_slack_col.assign(static_cast<std::size_t>(new_rows),
	                                           -1);
	        compact_sf.row_to_surplus_col.assign(
	            static_cast<std::size_t>(new_rows), -1);
	        compact_sf.row_to_artificial_col.assign(
	            static_cast<std::size_t>(new_rows), -1);
	        compact_sf.row_sign.assign(static_cast<std::size_t>(new_rows), 1);
	        compact_sf.ub_row_var.assign(static_cast<std::size_t>(new_rows), -1);
	        for (int old_r = 0; old_r < old_rows; ++old_r) {
	          const int new_r = sf_row_map[static_cast<std::size_t>(old_r)];
	          if (new_r < 0) continue;
	          auto remap_row_col = [&](const std::vector<int>& old_map) {
	            const int old_c =
	                old_r < static_cast<int>(old_map.size())
	                    ? old_map[static_cast<std::size_t>(old_r)]
	                    : -1;
	            return old_c >= 0 && old_c < old_cols
	                       ? sf_col_map[static_cast<std::size_t>(old_c)]
	                       : -1;
	          };
	          compact_sf.row_to_slack_col[static_cast<std::size_t>(new_r)] =
	              remap_row_col(old_sf.row_to_slack_col);
	          compact_sf.row_to_surplus_col[static_cast<std::size_t>(new_r)] =
	              remap_row_col(old_sf.row_to_surplus_col);
	          compact_sf.row_to_artificial_col[static_cast<std::size_t>(new_r)] =
	              remap_row_col(old_sf.row_to_artificial_col);
	          if (old_r < static_cast<int>(old_sf.row_sign.size())) {
	            compact_sf.row_sign[static_cast<std::size_t>(new_r)] =
	                old_sf.row_sign[static_cast<std::size_t>(old_r)];
	          }
	          if (old_r < static_cast<int>(old_sf.ub_row_var.size())) {
	            compact_sf.ub_row_var[static_cast<std::size_t>(new_r)] =
	                old_sf.ub_row_var[static_cast<std::size_t>(old_r)];
	          }
	        }
	        return true;
	      };

	      auto perform_root_transformed_lp_cut_aging =
	          [&](const char* phase) -> int {
	        if (!opt.auto_highs_root_pipeline ||
	            root_transformed_lp_cut_rows.empty() ||
	            !root_relax.simplex) {
	          return 0;
	        }
		        // HiGHS OptionRecordInt default for mip_lp_age_limit is 10.  With
		        // deleteRows=true, performAging only runs the deletion-age check
		        // every max(age_limit >> 1, 2) epochs; nonbasic cuts with nonzero
		        // row duals have their LP age reset.
		        ++root_transformed_lp_cut_age_epochs;
	        int effective_age_limit = kHighsDefaultMipLpAgeLimit;
	        const int period =
	            std::max(effective_age_limit >> 1, 2);
	        if (root_transformed_lp_cut_age_epochs %
	                static_cast<std::uint64_t>(period) !=
	            0) {
	          effective_age_limit = std::numeric_limits<int>::max();
	        } else if (static_cast<int>(root_transformed_lp_cut_age_epochs) <
	                   effective_age_limit) {
	          effective_age_limit =
	              static_cast<int>(root_transformed_lp_cut_age_epochs);
	        }

	        const int old_m = static_cast<int>(root_lp.A.rows());
	        std::vector<char> delete_row(static_cast<std::size_t>(old_m), 0);
	        int basic_rows = 0;
	        int dual_reset_rows = 0;
	        int delete_count = 0;
	        const double dual_tol = strict_highs_root_fixed_point
	            ? highs_mip_feastol * 0.1
	            : std::max(1e-7, opt.lp_tol);
	        for (auto& row : root_transformed_lp_cut_rows) {
	          if (!row.alive || row.lp_row < 0 || row.lp_row >= old_m) continue;
	          const bool row_basic =
	              root_inequality_row_slack_basic(root_relax, row.lp_row);
	          if (row_basic) {
	            ++basic_rows;
	            ++row.age;
	            if (row.age > effective_age_limit) {
	              delete_row[static_cast<std::size_t>(row.lp_row)] = 1;
	            }
	            continue;
	          }
	          const bool have_dual = root_relax.row_duals.size() > row.lp_row;
	          const double row_dual =
	              have_dual ? root_relax.row_duals[row.lp_row] : 0.0;
	          if (std::abs(row_dual) > dual_tol) {
	            row.age = 0;
	            ++dual_reset_rows;
	          }
	        }
	        for (char v : delete_row) {
	          if (v) ++delete_count;
	        }
	        if (delete_count <= 0) {
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} epoch={} "
	                       "basic={} dualReset={} delete=0 rows={}\n",
	                       phase != nullptr ? phase : "xpool",
	                       root_transformed_lp_cut_age_epochs, basic_rows,
	                       dual_reset_rows,
	                       static_cast<int>(root_transformed_lp_cut_rows.size()));
	          }
	          return 0;
	        }

	        std::vector<int> row_map;
	        LPModel compact_lp;
	        if (!build_root_lp_without_ineq_rows(delete_row, row_map, compact_lp)) {
	          return 0;
	        }
	        apply_node_bounds(compact_lp.vars, root.lb, root.ub);
	        std::vector<int> sf_row_map;
	        StandardFormLP compact_sf;
	        std::vector<int> sf_col_map;
	        std::vector<int> deleted_sf_rows;
	        std::vector<int> deleted_sf_cols;
	        if (!root_relax.simplex ||
	            !root_relax.simplex->basis.cached_sparse_basis ||
	            root_relax.simplex->basis.cached_sparse_basis->kind() !=
	                BasisOpsKind::VendoredHighs) {
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} delete={} "
	                       "rejected_no_vendored_live_basis\n",
	                       phase != nullptr ? phase : "xpool", delete_count);
	          }
	          return 0;
	        }
	        {
	          const int old_ineq_rows = old_m;
	          const int new_ineq_rows =
	              static_cast<int>(compact_lp.A.rows());
	          const int old_sf_rows =
	              static_cast<int>(root_relax.simplex->form.A.rows());
	          const int old_eq_rows = old_sf_rows - old_ineq_rows;
	          if (old_eq_rows < 0) {
	            return 0;
	          }
	          sf_row_map.assign(static_cast<std::size_t>(old_sf_rows), -1);
	          for (int r = 0; r < old_ineq_rows; ++r) {
	            sf_row_map[static_cast<std::size_t>(r)] =
	                row_map[static_cast<std::size_t>(r)];
	          }
	          for (int e = 0; e < old_eq_rows; ++e) {
	            sf_row_map[static_cast<std::size_t>(old_ineq_rows + e)] =
	                new_ineq_rows + e;
	          }
	        }
	        if (!build_compacted_scaled_sf_from_root_state(
	                root_relax.simplex->form, sf_row_map, sf_col_map,
	                deleted_sf_rows, deleted_sf_cols, compact_sf)) {
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} delete={} "
	                       "rejected_compact_scaled_sf\n",
	                       phase != nullptr ? phase : "xpool", delete_count);
	          }
	          return 0;
	        }
	        std::shared_ptr<SimplexBasis> compact_basis =
	            make_root_compacted_basis_hint(root_relax.simplex->form,
	                                           compact_sf, sf_row_map);
	        SimplexOptions compact_sf_opt;
	        compact_sf_opt.max_iter =
	            std::max(root_solve_opt.max_lp_iter * 10, 2000);
	        compact_sf_opt.feasibility_tol =
	            std::max(1e-10, root_solve_opt.lp_tol * 0.1);
	        compact_sf_opt.optimality_tol =
	            std::max(1e-10, root_solve_opt.lp_tol * 0.1);
	        compact_sf_opt.verbose = false;
	        compact_sf_opt.allow_cold_start = false;
	        compact_sf_opt.use_partial_pricing = false;
	        compact_sf_opt.perturb_degenerate_primal = false;
	        compact_sf_opt.lp_kernel_backend = root_solve_opt.lp_kernel_backend;
	        compact_sf_opt.factor_backend =
	            simplex_factor_backend_from_id(
	                root_solve_opt.simplex_factor_backend);
	        LPRelaxationResult compact_relax;
	        if (compact_basis) {
	          auto sf_res = std::make_shared<SimplexResult>();
	          const bool deleted_resolved =
	              root_relax.simplex->basis.cached_sparse_basis
	                  ->delete_rows_cols_and_resolve(
	                      compact_sf, compact_basis.get(), deleted_sf_rows,
	                      deleted_sf_cols, compact_sf_opt, *sf_res);
	          if (sf_res->result.stats.success &&
	              deleted_resolved &&
	              static_cast<int>(sf_res->result.x.size()) == n) {
	            compact_relax.primal = sf_res->result;
	            compact_relax.dual_bound = compact_relax.primal.stats.objective;
	            compact_relax.simplex = sf_res;
	            compact_relax.row_duals = sf_res->result.constraint_duals;
	            compact_relax.basis_hint =
	                std::make_shared<SimplexBasis>(sf_res->basis);
	            compact_relax.basis_hint->sf_n_slack = sf_res->form.n_slack;
	            compact_relax.basis_hint->sf_n_surplus = sf_res->form.n_surplus;
	            compact_relax.basis_hint->sf_n_artificial =
	                sf_res->form.n_artificial;
	            if (sf_res->reduced_costs.size() > 0) {
	              compact_relax.basis_hint->cached_reduced_costs =
	                  std::make_shared<const Eigen::VectorXd>(
	                      sf_res->reduced_costs);
	            }
	            if (sf_res->basis.cached_sparse_basis) {
	              compact_relax.basis_hint->cached_sparse_basis =
	                  sf_res->basis.cached_sparse_basis;
	            }
	          } else if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr ||
	                     opt.verbose) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} delete={} "
	                       "warm_rejected status={} hint=1 rows={} cols={}\n",
	                       phase != nullptr ? phase : "xpool", delete_count,
	                       sf_res->result.stats.status,
	                       static_cast<int>(deleted_sf_rows.size()),
	                       static_cast<int>(deleted_sf_cols.size()));
	          }
	        }
	        if (!compact_relax.primal.stats.success) {
	          // HiGHS HighsLpRelaxation::performAging(true) deletes aged LP
	          // cut rows and then re-optimizes.  A failed native incremental
	          // delete/warm-start path must therefore fall back to the ordinary
	          // LP solve, not skip the deletion lifecycle.
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} delete={} "
	                       "fallback_full_resolve hint={}\n",
	                       phase != nullptr ? phase : "xpool", delete_count,
	                       compact_basis ? 1 : 0);
	          }
	          compact_relax = solve_lp_relaxation(
	              compact_lp, &root.x_relax, root.basis_hint.get(),
	              root_solve_opt);
	        }
	        ++out.bc_stats.lp_solves;
	        const double old_bound = root.bound;
	        if (!compact_relax.primal.stats.success ||
	            compact_relax.primal.x.size() != n ||
	            !std::isfinite(compact_relax.dual_bound) ||
	            compact_relax.dual_bound <
	                old_bound -
	                    std::max(1e-6,
	                             1e-9 * std::max(1.0, std::abs(old_bound)))) {
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPOOL-AGING] phase={} delete={} "
	                       "rejected oldBound={:.12g} newBound={:.12g} status={}\n",
	                       phase != nullptr ? phase : "xpool", delete_count,
	                       old_bound, compact_relax.dual_bound,
	                       compact_relax.primal.stats.status);
	          }
	          return 0;
	        }

	        for (auto& row : root_transformed_lp_cut_rows) {
	          if (!row.alive) continue;
		          if (row.lp_row >= 0 && row.lp_row < old_m &&
		              delete_row[static_cast<std::size_t>(row.lp_row)]) {
		            row.alive = false;
		            root_transformed_cutpool_mark_lp_removed(row.cut_index);
		          } else if (row.lp_row >= 0 && row.lp_row < old_m) {
	            row.lp_row = row_map[static_cast<std::size_t>(row.lp_row)];
	          }
	        }

		        root_lp = std::move(compact_lp);
		        root_relax = std::move(compact_relax);
		        root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
		        root.x_seed = root.x_relax;
		        refresh_root_lp_source_solution();
		        root.basis_hint = root_relax.basis_hint;
	        if (root.basis_hint) root.basis_hint->compact_indices_storage();
	        root.bound = root_relax.dual_bound;
	        if (maintain_native_xpool_sf_cache && root_relax.simplex &&
	            root_relax.simplex->form.A.rows() > 0) {
	          root_first_class_unscaled_sf = build_standard_form_lp(root_lp);
	          root_first_class_sf_valid = true;
	        } else {
	          root_first_class_sf_valid = false;
	        }
	        if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
	          root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
	        } else {
	          root_sbasis.reset();
	        }
	        ++root_source_epoch;
	        root_xpool_noop_epoch = 0;
	        root_xpool_noop_pool_size =
	            std::numeric_limits<std::size_t>::max();
	        if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
	          fmt::print(stderr,
	                     "[B&C ROOT-XPOOL-AGING] phase={} epoch={} "
	                     "basic={} dualReset={} deleted={} bound={:.12g}->{:.12g} "
	                     "lpRows={}\n",
	                     phase != nullptr ? phase : "xpool",
	                     root_transformed_lp_cut_age_epochs, basic_rows,
	                     dual_reset_rows, delete_count, old_bound, root.bound,
	                     static_cast<int>(root_lp.A.rows()));
	        }
	        return delete_count;
	      };

	      auto flush_root_transformed_cutpool_sources =
	          [&](const char* phase) -> bool {
        if (root_transformed_cutpool_pending_source_rows <= 0) return false;
        const int pending = root_transformed_cutpool_pending_source_rows;
        if (strict_highs_root_fixed_point) {
          root_transformed_cutpool_pending_source_rows = 0;
          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-VB] phase={} mode=sealed_highs "
                       "pendingSrc={} cutpoolRows=0 cutMixed=0 cutVUB=0 "
                       "cutVLB=0 auditVarbounds=0 auditImpl=0 "
                       "liveVarbounds=0 liveImpl=0\n",
                       phase != nullptr ? phase : "after_root_xpool_accept",
                       pending);
          }
          return true;
        }
        const std::size_t old_varbounds = variable_bound_table.size();
        const std::uint64_t old_implications =
            static_cast<std::uint64_t>(implication_graph.size());
        NativeVariableBoundSourceStats audit_stats;
        std::size_t audit_varbounds = 0;
        std::uint64_t audit_implications = 0;
        if (!root_transformed_cutpool_audit_rows.empty()) {
          VariableBoundTable scratch_table = variable_bound_table;
          BinaryImplicationGraph scratch_graph = implication_graph;
          audit_stats = augment_variable_bound_table_from_sparse_cut_rows(
              root_lp, root_transformed_cutpool_audit_rows,
              root_transformed_cutpool_audit_rhs, root_implied_integer_cols,
              scratch_table, &scratch_graph, 100,
              std::max(1e-9, opt.lp_tol));
          audit_varbounds =
              scratch_table.size() > old_varbounds
                  ? scratch_table.size() - old_varbounds
                  : 0;
          audit_implications =
              scratch_graph.size() > old_implications
                  ? static_cast<std::uint64_t>(scratch_graph.size()) -
                        old_implications
                  : 0;
        }
        if (root_transformed_cutpool_live_vb_source) {
          const std::size_t before_varbounds = variable_bound_table.size();
          const std::uint64_t before_implications =
              static_cast<std::uint64_t>(implication_graph.size());
          refresh_variable_bound_table(root_lp, phase);
          const std::size_t after_varbounds = variable_bound_table.size();
          const std::uint64_t after_implications =
              static_cast<std::uint64_t>(implication_graph.size());
          if ((after_varbounds > before_varbounds ||
               after_implications > before_implications) &&
              use_objective_propagation) {
            objective_propagation.setup(root_lp, &root_implied_integer_cols,
                                        &clique_table, &implication_graph,
                                        objective_propagation_build_policy());
          }
        }
        root_transformed_cutpool_pending_source_rows = 0;
        if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
          fmt::print(stderr,
                     "[B&C ROOT-XPOOL-VB] phase={} mode={} pendingSrc={} "
                     "cutpoolRows={} cutMixed={} cutVUB={} cutVLB={} "
                     "auditVarbounds={} auditImpl={} liveVarbounds={} "
                     "liveImpl={}\n",
                     phase != nullptr ? phase : "after_root_xpool_accept",
                     root_transformed_cutpool_live_vb_source ? "live"
                                                             : "audit",
                     pending,
                     audit_stats.cut_rows_scanned,
                     audit_stats.cut_mixed_rows,
                     audit_stats.cut_vub_candidates,
                     audit_stats.cut_vlb_candidates,
                     audit_varbounds,
                     audit_implications,
                     static_cast<long long>(variable_bound_table.size()) -
                         static_cast<long long>(old_varbounds),
                     static_cast<long long>(implication_graph.size()) -
                         static_cast<long long>(old_implications));
        }
        return true;
      };

      std::size_t last_cutpool_clique_scan_size = 0;
      auto synchronize_root_source_cutpool_domain_sources =
          [&](const char* source) -> std::uint64_t {
        if (strict_highs_root_fixed_point) {
          last_cutpool_clique_scan_size = cut_pool.size();
          (void)source;
          return 0;
        }
        if (last_cutpool_clique_scan_size >
            static_cast<std::size_t>(cut_pool.size())) {
          last_cutpool_clique_scan_size = 0;
        }
        std::uint64_t learned = 0;
        std::uint64_t rows_scanned = 0;
        std::uint64_t rows_with_literals = 0;
        std::uint64_t generated_edges = 0;
        const auto& cuts = cut_pool.cuts();
        for (std::size_t i = last_cutpool_clique_scan_size;
             i < cuts.size(); ++i) {
          const PoolCut& cut = cuts[i];
          if (cut.coeff.size() != n || cut.coeff.nonZeros() <= 0 ||
              !std::isfinite(cut.rhs)) {
            continue;
          }
          CliqueTable::CutCliqueExtractionStats stats;
          const std::size_t added = clique_table.add_literal_edges_from_cut(
              base_lp, cut.coeff, cut.rhs, 512, &root_implied_integer_cols,
              &implication_graph, &stats, nullptr);
          learned += static_cast<std::uint64_t>(added);
          rows_scanned += stats.rows_scanned;
          rows_with_literals += stats.rows_with_binary_literals;
          generated_edges += stats.literal_edges_generated;
        }
        last_cutpool_clique_scan_size = cuts.size();
        if (learned > 0) {
          note_global_domain_learning(learned);
          note_objective_artifact_learning(learned);
          if (use_objective_propagation) {
            objective_propagation.setup(base_lp, &root_implied_integer_cols,
                                        &clique_table, &implication_graph,
                                        objective_propagation_build_policy());
            const auto published = publish_objective_implied_event_source(
                base_lp, objective_propagation, implication_graph,
                clique_table, opt.int_tol);
            out.bc_stats.objective_implied_event_published_implications +=
                published.implications_added;
            out.bc_stats.objective_implied_event_published_clique_edges +=
                published.clique_edges_added;
            const std::uint64_t published_total =
                published.implications_added + published.clique_edges_added;
            note_global_domain_learning(published_total);
            note_objective_artifact_learning(published_total);
            learned += published_total;
            if (published_total > 0) {
              objective_propagation.setup(base_lp, &root_implied_integer_cols,
                                          &clique_table, &implication_graph,
                                          objective_propagation_build_policy());
            }
          }
        }
        if ((opt.verbose || bc_env_options().value("MIPSOLVERS_BC_CONF") != nullptr ||
             bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) &&
            rows_scanned > 0) {
          fmt::print(stderr,
                     "[B&C ROOT-CUTPOOL-SRC] source={} scan={} litRows={} "
                     "genEdges={} learned={} cutpool={} clqEdges={} arcs={}\n",
                     source != nullptr ? source : "root_cutpool",
                     rows_scanned, rows_with_literals, generated_edges,
                     learned, cut_pool.size(), clique_table.n_literal_edges(),
                     implication_graph.size());
        }
        return learned;
      };

	      auto propagate_root_transformed_cutpool_resolve = [&]() -> int {
        if (!root_cutoff_domain_commit_allowed) {
          return 0;
        }
        if (root.x_relax.size() != n || root.lb.size() != n ||
            root.ub.size() != n) {
	          return 0;
        }
        const Eigen::VectorXd old_lb = root.lb;
        const Eigen::VectorXd old_ub = root.ub;
        const double old_bound = root.bound;
        const LPModel old_root_lp = root_lp;
        const LPRelaxationResult old_root_relax = root_relax;
        const auto old_basis = root.basis_hint;
	        const Eigen::VectorXd old_x = root.x_relax;
	        const Eigen::VectorXd old_x_seed = root.x_seed;
	        const Eigen::VectorXd old_source_x = root_lp_source_x;
	        const std::size_t old_reason_count = root.domain_reason_bounds.size();

        const std::uint64_t source_epoch_before =
            current_domain_learning_epoch();
        const std::uint64_t objective_epoch_before =
            current_objective_artifact_epoch();
        std::uint64_t source_learned = 0;
        int objective_tightened = 0;
        int objective_pruned = 0;
        int redcost_tightened = 0;
        int redcost_pruned = 0;
        int redcost_active = 0;
        int conflict_tightened = 0;
        int implication_tightened = 0;
        int clique_tightened = 0;
        int row_tightened = 0;
	        int cutpool_tightened = 0;
	        int reasons_added = 0;
	        int closure_passes = 0;
        int strict_model_pending_scans = 0;
        int strict_cut_pending_rows = 0;
        int strict_cut_scanned_rows = 0;
        int strict_cut_active_rows = 0;
	        bool pruned = false;

        auto duplicate_root_reason_bound =
            [&](const DomainReasonBound& rb) -> bool {
          const std::size_t reason_hash = conflict_clause_hash(rb.reason);
          const double tol = std::max(1e-9, opt.int_tol);
          for (const auto& existing : root.domain_reason_bounds) {
            if (conflict_clause_hash(existing.reason) == reason_hash &&
                domain_reason_bounds_equal(existing, rb, tol)) {
              return true;
            }
          }
          return false;
        };
        auto append_root_reason_bounds =
            [&](std::vector<DomainReasonBound>& reason_bounds) {
          for (auto& rb : reason_bounds) {
            if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
                !std::isfinite(rb.bound.value) ||
                duplicate_root_reason_bound(rb)) {
              continue;
            }
            append_domain_reason_bound(root, std::move(rb), n, &old_lb,
                                       &old_ub);
            ++reasons_added;
          }
          reason_bounds.clear();
        };

	        bool redcost_lurking_propagated = false;
	        const std::size_t max_root_closure_passes =
	            std::max<std::size_t>(8, 2 * static_cast<std::size_t>(n) + 8);
        for (std::size_t pass = 0; pass < max_root_closure_passes; ++pass) {
          ++closure_passes;
          const Eigen::VectorXd pass_lb = root.lb;
          const Eigen::VectorXd pass_ub = root.ub;
          const std::size_t pass_reasons = root.domain_reason_bounds.size();
          const int pass_conflicts = conflict_pool.size();
          const int pass_cutpool = cut_pool.size();
          const std::uint64_t pass_impl =
              static_cast<std::uint64_t>(implication_graph.size());
          const std::uint64_t pass_cliques =
              static_cast<std::uint64_t>(clique_table.n_literal_edges());
          const std::uint64_t pass_domain_epoch =
              current_domain_learning_epoch();
          const std::uint64_t pass_objective_epoch =
              current_objective_artifact_epoch();
          const std::uint64_t pass_closure_epoch =
              current_domain_closure_epoch();

          source_learned += synchronize_root_source_cutpool_domain_sources(
              "root_domain_closure");

          if (has_root_cutoff_for_propagation && use_objective_propagation &&
              std::isfinite(root_cutoff_for_propagation)) {
            std::vector<DomainReasonBound> objective_reasons;
            int pass_obj_tight = 0;
            int pass_obj_pruned = 0;
                const Eigen::VectorXd stage_before_lb = root.lb;
                const Eigen::VectorXd stage_before_ub = root.ub;
	            const bool obj_ok = objective_propagation.propagate(
	                base_lp, root.lb, root.ub, root_cutoff_for_propagation,
	                opt.int_tol, /*depth=*/0,
	                static_cast<int>(root.domain_reason_bounds.size()),
	                &objective_reasons, pass_obj_tight, pass_obj_pruned);
	            objective_tightened += std::max(0, pass_obj_tight);
	            objective_pruned += std::max(0, pass_obj_pruned);
	            append_root_reason_bounds(objective_reasons);
                const ChangedColSides obj_changed = collect_changed_col_sides(
                    stage_before_lb, stage_before_ub, root.lb, root.ub);
                mark_root_highs_domain_rows_for_changed_cols(obj_changed.any,
                                                             root.lb, root.ub);
                mark_root_transformed_cut_prop_for_changed_sides(
                    obj_changed, root.lb, root.ub);
            if (!obj_ok || pass_obj_pruned > 0 ||
                !bounds_consistent(root.lb, root.ub)) {
              pruned = true;
              break;
            }

	            if (!redcost_lurking_propagated &&
	                root_redcost_lurking_bounds.size() > 0) {
	              redcost_lurking_propagated = true;
	              std::vector<DomainReasonBound> redcost_reasons;
                  const Eigen::VectorXd stage_before_lb = root.lb;
                  const Eigen::VectorXd stage_before_ub = root.ub;
	              const auto rc_stats = root_redcost_lurking_bounds.propagate(
                  root_cutoff_for_propagation, root.bound, root.lb, root.ub,
                  &redcost_reasons, /*reason_depth=*/0,
                  static_cast<int>(root.domain_reason_bounds.size()),
                  std::max(1e-9, opt.int_tol));
              redcost_tightened += std::max(0, rc_stats.tightened);
              redcost_pruned += std::max(0, rc_stats.pruned);
              redcost_active += std::max(0, rc_stats.records_active);
              append_root_reason_bounds(redcost_reasons);
                  const ChangedColSides rc_changed =
                      collect_changed_col_sides(stage_before_lb,
                                                stage_before_ub, root.lb,
                                                root.ub);
                  mark_root_highs_domain_rows_for_changed_cols(rc_changed.any,
                                                               root.lb,
                                                               root.ub);
                  mark_root_transformed_cut_prop_for_changed_sides(
                      rc_changed, root.lb, root.ub);
              if (rc_stats.pruned > 0 ||
                  !bounds_consistent(root.lb, root.ub)) {
                pruned = true;
                break;
              }
            }
          }

          if (!conflict_pool.empty()) {
            int pass_conflict_tight = 0;
            std::vector<BoundChangeInfo> conflict_changes;
            std::vector<DomainReasonBound> conflict_reasons;
                const Eigen::VectorXd stage_before_lb = root.lb;
                const Eigen::VectorXd stage_before_ub = root.ub;
	            const bool conflict_ok = conflict_pool.propagate_with_reasons(
	                base_lp.vars, root.lb, root.ub, &pass_conflict_tight,
	                &conflict_changes, &conflict_reasons, opt.int_tol);
	            (void)conflict_changes;
	            conflict_tightened += std::max(0, pass_conflict_tight);
	            append_root_reason_bounds(conflict_reasons);
                const ChangedColSides conflict_changed =
                    collect_changed_col_sides(stage_before_lb,
                                              stage_before_ub, root.lb,
                                              root.ub);
                mark_root_highs_domain_rows_for_changed_cols(
                    conflict_changed.any, root.lb, root.ub);
                mark_root_transformed_cut_prop_for_changed_sides(
                    conflict_changed, root.lb, root.ub);
            if (!conflict_ok || !bounds_consistent(root.lb, root.ub)) {
              pruned = true;
              break;
            }
          }

          if (!implication_graph.empty()) {
                const Eigen::VectorXd stage_before_lb = root.lb;
                const Eigen::VectorXd stage_before_ub = root.ub;
		            implication_tightened += implication_graph.propagate(
		                base_lp.vars, root.lb, root.ub, nullptr,
		                std::max(1e-9, opt.int_tol));
                const ChangedColSides impl_changed =
                    collect_changed_col_sides(stage_before_lb,
                                              stage_before_ub, root.lb,
                                              root.ub);
                mark_root_highs_domain_rows_for_changed_cols(impl_changed.any,
                                                             root.lb, root.ub);
                mark_root_transformed_cut_prop_for_changed_sides(
                    impl_changed, root.lb, root.ub);
            if (!bounds_consistent(root.lb, root.ub)) {
              pruned = true;
              break;
            }
          }
          if (!clique_table.empty()) {
                const Eigen::VectorXd stage_before_lb = root.lb;
                const Eigen::VectorXd stage_before_ub = root.ub;
	            clique_tightened += clique_table.propagate(root.lb, root.ub);
                const ChangedColSides clique_changed =
                    collect_changed_col_sides(stage_before_lb,
                                              stage_before_ub, root.lb,
                                              root.ub);
                mark_root_highs_domain_rows_for_changed_cols(
                    clique_changed.any, root.lb, root.ub);
                mark_root_transformed_cut_prop_for_changed_sides(
                    clique_changed, root.lb, root.ub);
            if (!bounds_consistent(root.lb, root.ub)) {
              pruned = true;
              break;
            }
          }

	          const int pass_row_tight =
	                  strict_highs_root_fixed_point
	                      ? propagate_root_highs_presolved_domain_pending(root.lb,
	                                                                      root.ub)
                      : propagate_root_highs_presolved_domain(
	                            root.lb, root.ub, opt.bound_propagation_rounds);
	          row_tightened += std::max(0, pass_row_tight);
              if (strict_highs_root_fixed_point) {
                strict_model_pending_scans += pass_row_tight;
              }
	          if (!bounds_consistent(root.lb, root.ub)) {
            pruned = true;
            break;
          }

          auto propagate_root_sparse_family =
              [&](const std::vector<Eigen::SparseVector<double>>& rows,
                  const std::vector<double>& rhs) -> bool {
            if (rows.empty()) return true;
            SparseCutpoolPropagationStats stats;
            std::vector<DomainReasonBound> cut_reasons;
            const bool ok = propagate_sparse_cutpool_rows(
                rows, rhs, root.lb, root.ub, opt.bound_propagation_rounds,
                &stats, &cut_reasons, /*reason_depth=*/0,
                static_cast<int>(root.domain_reason_bounds.size()));
            cutpool_tightened += std::max(0, stats.tightened);
            append_root_reason_bounds(cut_reasons);
            return ok && bounds_consistent(root.lb, root.ub);
          };
          if (!propagate_root_sparse_family(variable_bound_cutpool_rows,
                                           variable_bound_cutpool_rhs)) {
            pruned = true;
            break;
          }
	          if (strict_highs_root_fixed_point) {
                SparseCutpoolPropagationStats stats;
                std::vector<DomainReasonBound> cut_reasons;
	            if (!propagate_root_transformed_cutpool_pending(
	                        root.lb, root.ub, &stats, &cut_reasons,
                        /*reason_depth=*/0,
                        static_cast<int>(root.domain_reason_bounds.size()))) {
	              pruned = true;
	              break;
		            }
	                cutpool_tightened += std::max(0, stats.tightened);
                    strict_cut_pending_rows += stats.pending_rows;
                    strict_cut_scanned_rows += stats.rows_scanned;
                    strict_cut_active_rows += stats.active_rows;
	                append_root_reason_bounds(cut_reasons);
	          }
	          // In strict HiGHS root fixed-point mode, transformed cuts are consumed
	          // above through the private cutpool lifecycle: only alive,
	          // non-LP-owned, propagation-linked rows are visible to domain
	          // closure.  The append-only audit vector is not replayed here,
	          // because it would keep LP-owned or aged-out rows alive as a second
	          // propagation channel that HiGHS does not have.

          if (!strict_highs_root_fixed_point && cut_pool.size() > 0) {
            int pass_cutpool_tight = 0;
            std::vector<BoundChangeInfo> cutpool_changes;
            std::vector<DomainReasonBound> cutpool_reasons;
            CutPoolPropagationStats cutpool_stats;
            const bool cutpool_ok = cut_pool.propagate_with_reasons(
                base_lp.vars, root.lb, root.ub, opt.bound_propagation_rounds,
                &pass_cutpool_tight, &cutpool_changes, &cutpool_reasons,
                &cutpool_stats, nullptr, &root_implied_integer_cols,
                /*reason_depth=*/0,
                static_cast<int>(root.domain_reason_bounds.size()),
                std::max(1e-9, opt.lp_tol));
            (void)cutpool_changes;
            cutpool_tightened += std::max(0, pass_cutpool_tight);
            append_root_reason_bounds(cutpool_reasons);
            if (!cutpool_ok || !bounds_consistent(root.lb, root.ub)) {
              pruned = true;
              break;
            }
          }

          apply_domain_objective_lower_bound(root, "root_domain_closure");

          const auto pass_delta =
              count_bound_changes(pass_lb, pass_ub, root.lb, root.ub);
          const bool pass_changed =
              pass_delta.first > 0 ||
              root.domain_reason_bounds.size() != pass_reasons ||
              conflict_pool.size() != pass_conflicts ||
              cut_pool.size() != pass_cutpool ||
              static_cast<std::uint64_t>(implication_graph.size()) !=
                  pass_impl ||
              static_cast<std::uint64_t>(clique_table.n_literal_edges()) !=
                  pass_cliques ||
              current_domain_learning_epoch() != pass_domain_epoch ||
              current_objective_artifact_epoch() != pass_objective_epoch ||
              current_domain_closure_epoch() != pass_closure_epoch;
	          if (!pass_changed) break;
	        }

        if (strict_highs_root_fixed_point &&
            (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose ||
             strict_model_pending_scans > 0 || strict_cut_pending_rows > 0 ||
             cutpool_tightened > 0 || row_tightened > 0)) {
          fmt::print(stderr,
                     "[B&C ROOT-PROP-IO] passes={} modelTight={} "
                     "modelPendingTight={} cutPending={} cutScanned={} "
                     "cutActive={} cutTight={} reasons={} pruned={}\n",
                     closure_passes, row_tightened, strict_model_pending_scans,
                     strict_cut_pending_rows, strict_cut_scanned_rows,
                     strict_cut_active_rows, cutpool_tightened, reasons_added,
                     pruned ? 1 : 0);
        }

        const auto delta =
            count_bound_changes(old_lb, old_ub, root.lb, root.ub);
        const auto changed_cols_delta =
            count_changed_cols(old_lb, old_ub, root.lb, root.ub);
        const int changed_cols =
            static_cast<int>(std::min<std::uint64_t>(
                changed_cols_delta.first,
                static_cast<std::uint64_t>(std::numeric_limits<int>::max())));
        if (pruned || !bounds_consistent(root.lb, root.ub)) {
          root.bound = has_root_cutoff_for_propagation
              ? root_cutoff_for_propagation
              : kInf;
          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr ||
              opt.verbose) {
            fmt::print(stderr,
                       "[B&C ROOT-DOM-CLOSURE] pruned pass={} obj={} "
                       "conflict={} impl={} clique={} row={} cutpool={} "
                       "rc={} rcActive={} reasons={} learned={}\n",
                       closure_passes, objective_tightened,
                       conflict_tightened, implication_tightened,
                       clique_tightened, row_tightened, cutpool_tightened,
                       redcost_tightened, redcost_active, reasons_added,
                       source_learned);
          }
          record_conformance_propagation(
              "root_xpool_cut_propagation", conformance_source_total(),
              delta.first, delta.second, 1, /*resolved=*/false,
              /*resolve_ok=*/true, old_bound, root.bound,
              fmt::format("pass={} obj={} conflict={} impl={} clique={} "
                          "row={} cutpool={} rc={} rc_active={} reasons={} "
                          "learned={} obj_pruned={} rc_pruned={}",
                          closure_passes, objective_tightened,
                          conflict_tightened, implication_tightened,
                          clique_tightened, row_tightened,
                          cutpool_tightened, redcost_tightened,
                          redcost_active, reasons_added, source_learned,
                          objective_pruned, redcost_pruned));
          return std::max(1, changed_cols);
        }
        const bool source_changed =
            current_domain_learning_epoch() != source_epoch_before ||
            current_objective_artifact_epoch() != objective_epoch_before;
        if (delta.first == 0) {
          if (source_changed) {
            ++root_source_epoch;
          }
          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr ||
              opt.verbose || source_changed) {
            fmt::print(stderr,
                       "[B&C ROOT-DOM-CLOSURE] pass={} tight=0 "
                       "obj={} conflict={} impl={} clique={} row={} "
                       "cutpool={} rc={} reasons={} learned={} source={}\n",
                       closure_passes, objective_tightened,
                       conflict_tightened, implication_tightened,
                       clique_tightened, row_tightened, cutpool_tightened,
                       redcost_tightened, reasons_added, source_learned,
                       source_changed ? 1 : 0);
          }
          return 0;
        }

        LPModel trial_lp = root_lp;
        apply_node_bounds(trial_lp.vars, root.lb, root.ub);
        LPRelaxationResult prop_relax = solve_lp_relaxation(
            trial_lp, &root.x_relax, root.basis_hint.get(), root_solve_opt);
        ++out.bc_stats.lp_solves;
        const bool resolve_ok =
            prop_relax.primal.stats.success && prop_relax.primal.x.size() == n &&
            std::isfinite(prop_relax.dual_bound) &&
            prop_relax.dual_bound >= old_bound -
                std::max(1e-6, 1e-9 * std::max(1.0, std::abs(old_bound)));
        if (!resolve_ok) {
          root.lb = old_lb;
          root.ub = old_ub;
          root_lp = old_root_lp;
          root_relax = old_root_relax;
	          root.basis_hint = old_basis;
	          root.x_relax = old_x;
	          root.x_seed = old_x_seed;
	          root_lp_source_x = old_source_x;
	          if (root.domain_reason_bounds.size() > old_reason_count) {
            root.domain_reason_bounds.resize(old_reason_count);
          }
          record_conformance_propagation(
              "root_xpool_cut_propagation", conformance_source_total(),
              delta.first, delta.second, 0, /*resolved=*/true,
              /*resolve_ok=*/false, old_bound, old_bound,
              fmt::format("pass={} obj={} conflict={} impl={} clique={} "
                          "row={} cutpool={} rc={} rc_active={} reasons={} "
                          "learned={} resolve_failed=1",
                          closure_passes, objective_tightened,
                          conflict_tightened, implication_tightened,
                          clique_tightened, row_tightened,
                          cutpool_tightened, redcost_tightened,
                          redcost_active, reasons_added, source_learned));
          return 0;
        }
	        root_lp = std::move(trial_lp);
	        root_relax = std::move(prop_relax);
	        root.x_relax = clamp_to_bounds(root_relax.primal.x, root.lb, root.ub);
	        root.x_seed = root.x_relax;
	        refresh_root_lp_source_solution();
		        root.basis_hint = root_relax.basis_hint;
	        if (root.basis_hint) root.basis_hint->compact_indices_storage();
	        root.bound = root_relax.dual_bound;
	        refresh_root_redcost_lurking_bounds("root_xpool_cut_propagation_resolve");
	        if (maintain_native_xpool_sf_cache && root_relax.simplex &&
	            root_relax.simplex->form.A.rows() > 0) {
	          root_first_class_unscaled_sf = build_standard_form_lp(root_lp);
	          root_first_class_sf_valid = true;
	        } else {
          root_first_class_sf_valid = false;
        }
        ++root_source_epoch;
        if (root_relax.simplex && root_relax.simplex->basis.cached_sparse_basis) {
          root_sbasis = root_relax.simplex->basis.cached_sparse_basis;
        } else {
          root_sbasis.reset();
        }
        out.bc_stats.root_cut_domain_tightenings += delta.first;
        record_conformance_propagation(
            "root_xpool_cut_propagation", conformance_source_total(),
            delta.first, delta.second, 0, /*resolved=*/true,
            /*resolve_ok=*/true, old_bound, root.bound,
            fmt::format("pass={} obj={} conflict={} impl={} clique={} "
                        "row={} cutpool={} rc={} rc_active={} reasons={} "
                        "learned={} obj_pruned={} rc_pruned={}",
                        closure_passes, objective_tightened,
                        conflict_tightened, implication_tightened,
                        clique_tightened, row_tightened, cutpool_tightened,
                        redcost_tightened, redcost_active, reasons_added,
                        source_learned, objective_pruned, redcost_pruned));
        if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr || opt.verbose) {
          fmt::print(stderr,
                     "[B&C ROOT-DOM-CLOSURE] pass={} tight={} obj={} "
                     "conflict={} impl={} clique={} row={} cutpool={} "
                     "rc={} rcActive={} reasons={} learned={} cols={} "
                     "bound={:.2f}->{:.2f}\n",
                     closure_passes, delta.first, objective_tightened,
                     conflict_tightened, implication_tightened,
                     clique_tightened, row_tightened, cutpool_tightened,
                     redcost_tightened, redcost_active, reasons_added,
                     source_learned, changed_cols, old_bound, root.bound);
        }
        return changed_cols;
      };

		  auto select_root_transformed_cutpool =
		      [&](RootTransformedPoolStats& stats) {
		    struct ScoredCut {
		      int index{-1};
		      double score{0.0};
		      double violation{0.0};
		      int trace_index{-1};
		    };
		    std::vector<ScoredCut> scored;
		    if (strict_highs_root_fixed_point) {
		      stats.num_lp_cuts = 0;
		      for (int i = 0;
		           i < static_cast<int>(root_transformed_cutpool.size()); ++i) {
		        const bool alive =
		            i >= static_cast<int>(root_transformed_cut_alive.size()) ||
		            root_transformed_cut_alive[static_cast<std::size_t>(i)] != 0;
		        const bool in_lp =
		            i < static_cast<int>(root_transformed_cut_in_lp.size()) &&
		            root_transformed_cut_in_lp[static_cast<std::size_t>(i)] != 0;
		        if (alive && in_lp) ++stats.num_lp_cuts;
		        if (alive && !in_lp) ++stats.available;
		      }
		        fmt::print(stderr,
		                 "[B&C ROOT-XPOOL-AUDIT] replay=disabled; strict "
		                 "selected state is live HiGHS callback\n");
		    }
			    const Eigen::VectorXd& source_x = root_source_solution();
			    if (source_x.size() != n) return scored;
			    const double feastol = highs_mip_feastol;
			    stats.available = 0;
			    int num_available = 0;
                int num_lp_cuts = 0;
		    std::vector<int> age_distribution(
		        static_cast<std::size_t>(
		            std::max(1, root_transformed_cutpool_age_limit) + 1),
	        0);
	    for (int i = 0;
	         i < static_cast<int>(root_transformed_cutpool.size()); ++i) {
	      const bool alive =
	          i >= static_cast<int>(root_transformed_cut_alive.size()) ||
	          root_transformed_cut_alive[static_cast<std::size_t>(i)] != 0;
		      const bool in_lp =
		          i < static_cast<int>(root_transformed_cut_in_lp.size()) &&
		          root_transformed_cut_in_lp[static_cast<std::size_t>(i)] != 0;
              if (alive && in_lp) {
                ++num_lp_cuts;
              }
		      if (alive && !in_lp) {
		        ++num_available;
	        const int age =
	            i < static_cast<int>(root_transformed_cut_age.size())
	                ? root_transformed_cut_age[static_cast<std::size_t>(i)]
	                : 0;
	        const int bucket =
	            std::clamp(age, 0, root_transformed_cutpool_age_limit);
	        ++age_distribution[static_cast<std::size_t>(bucket)];
	      }
	    }
			    int effective_age_limit = root_transformed_cutpool_age_limit;
			    int num_cuts_for_age = num_available;
		    while (effective_age_limit > 1 &&
		           num_cuts_for_age > root_transformed_cutpool_soft_limit) {
		      num_cuts_for_age -= age_distribution[
		          static_cast<std::size_t>(effective_age_limit)];
			      --effective_age_limit;
			    }
                stats.age_limit = effective_age_limit;
                stats.num_lp_cuts = num_lp_cuts;
		    for (int i = 0; i < static_cast<int>(root_transformed_cutpool.size()); ++i) {
	      if (i < static_cast<int>(root_transformed_cut_alive.size()) &&
	          root_transformed_cut_alive[static_cast<std::size_t>(i)] == 0) {
	        continue;
	      }
	      if (i < static_cast<int>(root_transformed_cut_in_lp.size()) &&
	          root_transformed_cut_in_lp[static_cast<std::size_t>(i)] != 0) {
	        continue;
	      }
	      const int current_age =
	          i < static_cast<int>(root_transformed_cut_age.size())
	              ? root_transformed_cut_age[static_cast<std::size_t>(i)]
	              : 0;
	      if (current_age < 0) continue;
        const PoolCut& cut = root_transformed_cutpool[static_cast<std::size_t>(i)];
        if (cut.coeff.size() != n || cut.coeff.nonZeros() <= 0) continue;
        ++stats.available;
          if (!std::isfinite(cut.rhs)) continue;
          double violation = -cut.rhs;
          for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it; ++it) {
            const int j = static_cast<int>(it.index());
            if (j < 0 || j >= n) continue;
            violation += it.value() * source_x[j];
          }
        if (!std::isfinite(violation)) continue;
	      stats.max_violation = std::max(stats.max_violation, violation);
	      if (!(violation > feastol)) {
	        if (i < static_cast<int>(root_transformed_cut_age.size())) {
	          ++root_transformed_cut_age[static_cast<std::size_t>(i)];
		          if (root_transformed_cut_age[static_cast<std::size_t>(i)] >=
		              effective_age_limit) {
		            root_transformed_cut_alive[static_cast<std::size_t>(i)] = 0;
                    ++stats.aged_deleted;
	                if (i < static_cast<int>(
	                            root_transformed_cut_prop_pending.size())) {
                  root_transformed_cut_prop_pending[static_cast<std::size_t>(
                      i)] = 0;
                }
	            erase_root_transformed_cutpool_hash_entry(i);
	          }
	        }
	        continue;
	      }
	      ++stats.violated;
	      if (i < static_cast<int>(root_transformed_cut_age.size())) {
	        root_transformed_cut_age[static_cast<std::size_t>(i)] = 0;
	      }

        HighsCDouble active_norm = 0.0;
	      int active_nnz = 0;
	      for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff); it; ++it) {
	        const int j = static_cast<int>(it.index());
	        if (j < 0 || j >= n) continue;
	        const double a = it.value();
	        if (std::abs(a) <= 1e-12) continue;
	        if (a > 0.0) {
		          if (source_x[j] > root.lb[j] + feastol) {
              active_norm += a * a;
	            ++active_nnz;
	          }
	        } else {
		          if (source_x[j] < root.ub[j] - feastol) {
              active_norm += a * a;
	            ++active_nnz;
	          }
	        }
	      }
			      if (active_nnz > 0) ++stats.active;
			      const double denom =
              static_cast<double>(active_nnz) * std::sqrt(double(active_norm));
			      const double score =
			          (denom > 0.0 && std::isfinite(denom))
			              ? violation / denom
			              : std::numeric_limits<double>::infinity();
	      stats.best_score = std::max(stats.best_score, score);
	      scored.push_back(ScoredCut{i, score, violation});
	    }
		    const std::uint64_t scored_tie_size =
		        static_cast<std::uint64_t>(scored.size());
        pdqsort(scored.begin(), scored.end(),
                [scored_tie_size](const ScoredCut& a,
                                  const ScoredCut& b) {
		      if (a.score != b.score) return a.score > b.score;
		      return std::make_pair(
		                 highs_style_hash_u64(
		                     (static_cast<std::uint64_t>(a.index) << 32) +
		                     scored_tie_size),
		                 a.index) >
		             std::make_pair(
		                 highs_style_hash_u64(
		                     (static_cast<std::uint64_t>(b.index) << 32) +
		                     scored_tie_size),
		                 b.index);
        });
		    if (!scored.empty()) {
		      root_transformed_cutpool_best_observed_score =
	          std::max(root_transformed_cutpool_best_observed_score,
	                   scored.front().score);
	      const double min_score =
	          root_transformed_cutpool_min_score_factor *
	          root_transformed_cutpool_best_observed_score;
	      auto eff_end = std::upper_bound(
	          scored.begin(), scored.end(), min_score,
	          [](double threshold, const ScoredCut& cut) {
	            return threshold > cut.score;
	          });
	      std::size_t num_efficacious =
	          static_cast<std::size_t>(eff_end - scored.begin());
	      const std::size_t lower_threshold = scored.size() / 20;
	      const std::size_t upper_threshold = scored.size() - 1;
	      if (num_efficacious <= lower_threshold) {
	        num_efficacious = std::max(scored.size() / 2, std::size_t{1});
	        root_transformed_cutpool_min_score_factor =
	            scored[num_efficacious - 1].score /
	            root_transformed_cutpool_best_observed_score;
	      } else if (num_efficacious > upper_threshold) {
	        root_transformed_cutpool_min_score_factor =
	            scored[upper_threshold].score /
	            root_transformed_cutpool_best_observed_score;
	      }
		      if (num_efficacious < scored.size()) {
		        scored.resize(num_efficacious);
		      }
              stats.efficacious = static_cast<int>(scored.size());
              stats.min_score_factor = root_transformed_cutpool_min_score_factor;
		    }

            if (const char* score_trace_env =
                    bc_env_options().value("MIPSOLVERS_XPOOL_SCORE_TRACE")) {
              int trace_limit = 32;
              int trace_terms = 0;
              if (const char* terms_env =
                      bc_env_options().value("MIPSOLVERS_XPOOL_SCORE_TRACE_TERMS")) {
                char* end = nullptr;
                const long parsed = std::strtol(terms_env, &end, 10);
                if (end != terms_env && parsed > 0) {
                  trace_terms = static_cast<int>(std::min<long>(parsed, 200));
                }
              }
              if (std::string(score_trace_env) == "all") {
                trace_limit = std::numeric_limits<int>::max();
              } else if (score_trace_env[0] != '\0') {
                char* end = nullptr;
                const long parsed = std::strtol(score_trace_env, &end, 10);
                if (end != score_trace_env) {
                  trace_limit = parsed > 0
                                    ? static_cast<int>(std::min<long>(parsed, 1000000))
                                    : 0;
                }
              }
              for (int ord = 0;
                   ord < static_cast<int>(scored.size()) && ord < trace_limit;
                   ++ord) {
                const ScoredCut& cand = scored[static_cast<std::size_t>(ord)];
                const PoolCut& cut =
                    root_transformed_cutpool[static_cast<std::size_t>(cand.index)];
                HighsCDouble trace_norm = 0.0;
                int trace_active = 0;
                fmt::memory_buffer terms;
                int emitted_terms = 0;
                for (Eigen::SparseVector<double>::InnerIterator it(cut.coeff);
                     it; ++it) {
                  const int j = static_cast<int>(it.index());
                  const double a = it.value();
                  if (j < 0 || j >= n) continue;
                  if (a > 0.0) {
                    if (source_x[j] > root.lb[j] + feastol) {
                      trace_norm += a * a;
                      ++trace_active;
                    }
                  } else if (source_x[j] < root.ub[j] - feastol) {
                    trace_norm += a * a;
                    ++trace_active;
                  }
                  if (emitted_terms < trace_terms) {
                    if (emitted_terms > 0) {
                      fmt::format_to(std::back_inserter(terms), ",");
                    }
                    fmt::format_to(std::back_inserter(terms), "{}:{:.17g}:{:.17g}",
                                   j, a, source_x[j]);
                    ++emitted_terms;
                  }
                }
                fmt::print(stderr,
                           "[B&C ROOT-XPOOL-SCORED] ord={} idx={} len={} "
                           "rhs={:.17g} score={:.17g} viol={:.17g} "
                           "active={} norm={:.17g} rowHash={:016x} terms=[{}]\n",
                           ord, cand.index, static_cast<int>(cut.coeff.nonZeros()),
                           cut.rhs, cand.score, cand.violation, trace_active,
                           double(trace_norm), cut.hash, fmt::to_string(terms));
              }
            }

		    std::vector<ScoredCut> selected;
		    selected.reserve(scored.size());
		    constexpr double kHighsMaxParallelism = 0.1;
		    for (const ScoredCut& cand : scored) {
		      const PoolCut& cand_cut =
		          root_transformed_cutpool[static_cast<std::size_t>(cand.index)];
		      bool discard = false;
	      for (const ScoredCut& keep : selected) {
	        const PoolCut& keep_cut =
	            root_transformed_cutpool[static_cast<std::size_t>(keep.index)];
	        const double par = sparse_sparse_dot(cand_cut.coeff, keep_cut.coeff) /
	                           (cand_cut.norm * keep_cut.norm);
	        if (par > kHighsMaxParallelism) {
	          discard = true;
	          break;
	        }
	      }
	      if (discard) {
	        ++stats.parallel;
	        continue;
		      }
		      selected.push_back(cand);
		    }
		    stats.selected = static_cast<int>(selected.size());
        stats.num_lp_cuts += stats.selected;
        if (const char* trace_env = bc_env_options().value("MIPSOLVERS_XPOOL_SELECT_TRACE")) {
          int trace_limit = 16;
          if (std::string(trace_env) == "all") {
            trace_limit = std::numeric_limits<int>::max();
          } else if (trace_env[0] != '\0') {
            char* end = nullptr;
            const long parsed = std::strtol(trace_env, &end, 10);
            if (end != trace_env) {
              trace_limit = parsed > 0
                                ? static_cast<int>(std::min<long>(parsed, 1000000))
                                : 0;
            }
          }
          if (trace_limit > 0) {
            auto trace_mix = [](std::uint64_t h,
                                std::uint64_t v) -> std::uint64_t {
              v ^= v >> 33;
              v *= std::uint64_t{0xff51afd7ed558ccd};
              v ^= v >> 33;
              v *= std::uint64_t{0xc4ceb9fe1a85ec53};
              v ^= v >> 33;
              return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) +
                          (h >> 2));
            };
            std::uint64_t index_hash = std::uint64_t{0x4849474853435349};
            std::uint64_t content_hash = std::uint64_t{0x4849474853435343};
            int selected_nnz = 0;
            for (int ord = 0; ord < static_cast<int>(selected.size()); ++ord) {
              const ScoredCut& keep = selected[static_cast<std::size_t>(ord)];
              const PoolCut& cut =
                  root_transformed_cutpool[static_cast<std::size_t>(keep.index)];
              const int len = static_cast<int>(cut.coeff.nonZeros());
              selected_nnz += len;
              std::uint64_t row_sig = trace_mix(
                  cut.hash, static_cast<std::uint64_t>(
                                HighsHashHelpers::double_hash_code(cut.rhs)));
              row_sig = trace_mix(row_sig, static_cast<std::uint64_t>(len));
              index_hash = trace_mix(index_hash,
                                     static_cast<std::uint64_t>(keep.index));
              content_hash = trace_mix(content_hash, row_sig);
              if (ord < trace_limit) {
                fmt::print(stderr,
                           "[B&C ROOT-XPOOL-SELECT-ROW] ord={} idx={} len={} "
                           "rhs={:.17g} score={:.17g} rowHash={:016x} "
                           "rowSig={:016x}\n",
                           ord, keep.index, len, cut.rhs, keep.score, cut.hash,
                           row_sig);
              }
            }
            fmt::print(stderr,
                       "[B&C ROOT-XPOOL-SELECT] selected={} indexHash={:016x} "
                       "contentHash={:016x} nnz={} traceRows={}\n",
                       static_cast<int>(selected.size()), index_hash, content_hash,
                       selected_nnz,
                       std::min(trace_limit, static_cast<int>(selected.size())));
          }
        }
        {
          auto trace_mix = [](std::uint64_t h,
                              std::uint64_t v) -> std::uint64_t {
            v ^= v >> 33;
            v *= std::uint64_t{0xff51afd7ed558ccd};
            v ^= v >> 33;
            v *= std::uint64_t{0xc4ceb9fe1a85ec53};
            v ^= v >> 33;
            return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) +
                        (h >> 2));
          };
          stats.selected_index_hash = std::uint64_t{0x4849474853435349};
          stats.selected_content_hash = std::uint64_t{0x4849474853435343};
          stats.selected_nnz = 0;
          for (const ScoredCut& keep : selected) {
            const PoolCut& cut =
                root_transformed_cutpool[static_cast<std::size_t>(keep.index)];
            const int len = static_cast<int>(cut.coeff.nonZeros());
            stats.selected_nnz += len;
            std::uint64_t row_sig = trace_mix(
                cut.hash, static_cast<std::uint64_t>(
                              HighsHashHelpers::double_hash_code(cut.rhs)));
            row_sig = trace_mix(row_sig, static_cast<std::uint64_t>(len));
            stats.selected_index_hash = trace_mix(
                stats.selected_index_hash, static_cast<std::uint64_t>(keep.index));
            stats.selected_content_hash =
                trace_mix(stats.selected_content_hash, row_sig);
          }
        }
		    return selected;
		  };

	  auto mark_implied_integer_simplex = [&](SimplexResult& simplex) -> int {
	    const int n_form =
	        static_cast<int>(simplex.form.original_types.size());
	    int implied_marked = 0;
	    for (int j = 0; j < std::min(n, n_form); ++j) {
	      if (root_implied_integer_cols[static_cast<std::size_t>(j)] &&
	          simplex.form.original_types[static_cast<std::size_t>(j)] ==
	              VarType::Continuous) {
	        simplex.form.original_types[static_cast<std::size_t>(j)] =
	            VarType::Integer;
	        ++implied_marked;
	      }
	    }
	    return implied_marked;
	  };

	  auto generate_root_tableau_cutpool = [&]() -> int {
        ++root_source_trace.tableau_calls;
        const auto t_tableau0 = std::chrono::steady_clock::now();
	        const RootSourceSignature source_sig = compute_root_source_signature();
        if (root_tableau_generated_signature.same_as(source_sig)) {
          root_source_trace.tableau_ms +=
              root_source_elapsed_ms(t_tableau0);
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XTAB-SRC] reuse sig={:016x}/{:016x} "
	                       "frac={} frontier={} srcFrontier={} "
	                       "varbounds={}/{} impl={}/{}\n",
	                       source_sig.lp_state, source_sig.source_state,
	                       source_sig.fractional_integer_like,
	                       source_sig.frontier_cols,
	                       source_sig.source_frontier_cols,
	                       static_cast<int>(source_sig.varbounds),
	                       static_cast<int>(source_sig.total_varbounds),
	                       static_cast<int>(source_sig.implications),
	                       static_cast<int>(source_sig.total_implications));
	          }
          return 0;
        }
		    const Eigen::VectorXd& source_x = root_source_solution();
		    if (!root_relax.simplex || source_x.size() != n) {
          root_source_trace.tableau_ms +=
              root_source_elapsed_ms(t_tableau0);
	      return 0;
	    }
	    std::shared_ptr<BasisOps> tableau_sbasis = root_sbasis;
	    if (root_relax.simplex->basis.cached_sparse_basis) {
	      tableau_sbasis = root_relax.simplex->basis.cached_sparse_basis;
	    }
	    SimplexResult tableau_simplex = *root_relax.simplex;
	    const int implied_marked = mark_implied_integer_simplex(tableau_simplex);
	    const bool large_transformed_source_root =
	        presolved_m > opt.cut_budget_large_row_threshold;
	    const int max_tableau_cuts = strict_highs_root_fixed_point
	        ? 1000
	        : (large_transformed_source_root
	               ? std::max(8, std::min(512, std::max(opt.cuts_per_round,
	                                                    20 * opt.cuts_per_round)))
	               : std::max(8, std::min(64, std::max(opt.cuts_per_round,
	                                                   2 * opt.cuts_per_round))));
		    std::vector<PoolCut> generated_rows;
	        std::function<int(PoolCut&&)> tableau_cutpool_acceptor;
	        const std::function<int(PoolCut&&)>* tableau_acceptor_ptr = nullptr;
	        const std::optional<std::uint64_t> tableau_cutgen_seed =
	            strict_highs_root_fixed_point && root_relax.simplex
	                ? std::optional<std::uint64_t>{
	                      static_cast<std::uint64_t>(std::max(
	                          0, root_relax.simplex->result.stats.iterations)) +
	                      static_cast<std::uint64_t>(
	                          root_transformed_cutpool.size())}
	                : std::nullopt;
	        if (strict_highs_root_fixed_point) {
	          tableau_cutpool_acceptor = [&](PoolCut&& row) -> int {
	            std::vector<PoolCut> one_row;
	            one_row.push_back(std::move(row));
	            return add_root_transformed_cutpool_rows(one_row, "tableau");
          };
          tableau_acceptor_ptr = &tableau_cutpool_acceptor;
        }
        const auto t_gen0 = std::chrono::steady_clock::now();
			    const int generated = add_transformed_tableau_cuts(
			        root_lp, source_x, tableau_simplex, opt, max_tableau_cuts,
			        tableau_sbasis, &root_implied_integer_cols, &implication_graph,
			        &variable_bound_table,
	            strict_highs_root_fixed_point ? nullptr : &generated_rows,
			        strict_highs_root_fixed_point ? highs_mip_feastol : 1e-8,
	            tableau_acceptor_ptr, tableau_cutgen_seed,
                &separator_storage_stats);
        root_source_trace.tableau_generate_ms +=
            root_source_elapsed_ms(t_gen0);
        const auto t_add0 = std::chrono::steady_clock::now();
	    const int added =
            strict_highs_root_fixed_point
                ? generated
                : add_root_transformed_cutpool_rows(generated_rows, "tableau");
        root_tableau_generated_signature = source_sig;
        root_source_trace.tableau_pool_add_ms +=
            root_source_elapsed_ms(t_add0);
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C ROOT-XTAB-SRC] implied_int={} generated={} "
	                 "pool_added={} pool={} sig={:016x}/{:016x} "
	                 "frac={} frontier={} srcFrontier={} "
	                 "varbounds={}/{} impl={}/{}\n",
	                 implied_marked, generated, added,
	                 static_cast<int>(root_transformed_cutpool.size()),
	                 source_sig.lp_state, source_sig.source_state,
	                 source_sig.fractional_integer_like,
	                 source_sig.frontier_cols,
	                 source_sig.source_frontier_cols,
	                 static_cast<int>(source_sig.varbounds),
	                 static_cast<int>(source_sig.total_varbounds),
	                 static_cast<int>(source_sig.implications),
	                 static_cast<int>(source_sig.total_implications));
	    }
        root_source_trace.tableau_ms += root_source_elapsed_ms(t_tableau0);
	    return added;
	  };

	  HighsRandom root_path_separator_randgen(0);
	  std::function<int(int)> root_path_separator_randint =
	      [&](int sup) -> int {
	        if (sup <= 1) return 0;
	        return static_cast<int>(
	            root_path_separator_randgen.integer(static_cast<HighsInt>(sup)));
	      };

	  auto generate_root_path_cutpool = [&]() -> int {
        ++root_source_trace.path_calls;
        const auto t_path0 = std::chrono::steady_clock::now();
	        const RootSourceSignature source_sig = compute_root_source_signature();
        if (root_path_generated_signature.same_as(source_sig)) {
          root_source_trace.path_ms += root_source_elapsed_ms(t_path0);
	          if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	            fmt::print(stderr,
	                       "[B&C ROOT-XPATH-SRC] reuse sig={:016x}/{:016x} "
	                       "frac={} frontier={} srcFrontier={} "
	                       "varbounds={}/{} impl={}/{}\n",
	                       source_sig.lp_state, source_sig.source_state,
	                       source_sig.fractional_integer_like,
	                       source_sig.frontier_cols,
	                       source_sig.source_frontier_cols,
	                       static_cast<int>(source_sig.varbounds),
	                       static_cast<int>(source_sig.total_varbounds),
	                       static_cast<int>(source_sig.implications),
	                       static_cast<int>(source_sig.total_implications));
	          }
          return 0;
        }
		    const Eigen::VectorXd& source_x = root_source_solution();
		    if (!root_relax.simplex || source_x.size() != n) {
          root_source_trace.path_ms += root_source_elapsed_ms(t_path0);
	      return 0;
	    }
	    SimplexResult path_simplex = *root_relax.simplex;
	    const int implied_marked = mark_implied_integer_simplex(path_simplex);
		    std::vector<PoolCut> generated_rows;
	        std::function<int(PoolCut&&)> path_cutpool_acceptor;
	        const std::function<int(PoolCut&&)>* path_acceptor_ptr = nullptr;
	        const std::optional<std::uint64_t> path_cutgen_seed =
	            strict_highs_root_fixed_point && root_relax.simplex
	                ? std::optional<std::uint64_t>{
	                      static_cast<std::uint64_t>(std::max(
	                          0, root_relax.simplex->result.stats.iterations)) +
	                      static_cast<std::uint64_t>(
	                          root_transformed_cutpool.size())}
	                : std::nullopt;
	        if (strict_highs_root_fixed_point) {
	          path_cutpool_acceptor = [&](PoolCut&& row) -> int {
	            std::vector<PoolCut> one_row;
	            one_row.push_back(std::move(row));
	            return add_root_transformed_cutpool_rows(one_row, "path");
          };
          path_acceptor_ptr = &path_cutpool_acceptor;
        }
        const auto t_gen0 = std::chrono::steady_clock::now();
			    const int generated = add_transformed_path_cuts(
			        root_lp, source_x, path_simplex, opt,
	            strict_highs_root_fixed_point
	                ? 0
	                : std::max(8, std::min(512, std::max(opt.cuts_per_round,
	                                                     20 * opt.cuts_per_round))),
		        &root_implied_integer_cols, &implication_graph, &variable_bound_table,
			        strict_highs_root_fixed_point ? nullptr : &generated_rows,
			        strict_highs_root_fixed_point ? highs_mip_feastol : 1e-8,
	            path_acceptor_ptr, path_cutgen_seed,
	            strict_highs_root_fixed_point ? &root_path_separator_randint
	                                          : nullptr,
                &separator_storage_stats);
        root_source_trace.path_generate_ms += root_source_elapsed_ms(t_gen0);
        const auto t_add0 = std::chrono::steady_clock::now();
	    const int added =
            strict_highs_root_fixed_point
                ? generated
                : add_root_transformed_cutpool_rows(generated_rows, "path");
        root_path_generated_signature = source_sig;
        root_source_trace.path_pool_add_ms += root_source_elapsed_ms(t_add0);
	    if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
	      fmt::print(stderr,
	                 "[B&C ROOT-XPATH-SRC] implied_int={} generated={} "
	                 "pool_added={} pool={} sig={:016x}/{:016x} "
	                 "frac={} frontier={} srcFrontier={} "
	                 "varbounds={}/{} impl={}/{}\n",
	                 implied_marked, generated, added,
	                 static_cast<int>(root_transformed_cutpool.size()),
	                 source_sig.lp_state, source_sig.source_state,
	                 source_sig.fractional_integer_like,
	                 source_sig.frontier_cols,
	                 source_sig.source_frontier_cols,
	                 static_cast<int>(source_sig.varbounds),
	                 static_cast<int>(source_sig.total_varbounds),
	                 static_cast<int>(source_sig.implications),
	                 static_cast<int>(source_sig.total_implications));
	    }
        root_source_trace.path_ms += root_source_elapsed_ms(t_path0);
	    return added;
	  };

	  auto generate_root_modk_cutpool = [&]() -> int {
        ++root_source_trace.modk_calls;
        const auto t_modk0 = std::chrono::steady_clock::now();
	        const RootSourceSignature source_sig = compute_root_source_signature();
        const Eigen::VectorXd& source_x = root_source_solution();
        if (!root_relax.simplex || source_x.size() != n) {
          root_source_trace.modk_ms += root_source_elapsed_ms(t_modk0);
          return 0;
        }
        SimplexResult modk_simplex = *root_relax.simplex;
        const int implied_marked = mark_implied_integer_simplex(modk_simplex);
        std::vector<PoolCut> generated_rows;
        std::function<int(PoolCut&&)> modk_cutpool_acceptor;
        const std::function<int(PoolCut&&)>* modk_acceptor_ptr = nullptr;
        const std::optional<std::uint64_t> modk_cutgen_seed =
            strict_highs_root_fixed_point && root_relax.simplex
                ? std::optional<std::uint64_t>{
                      static_cast<std::uint64_t>(std::max(
                          0, root_relax.simplex->result.stats.iterations)) +
                      static_cast<std::uint64_t>(
                          root_transformed_cutpool.size())}
                : std::nullopt;
        if (strict_highs_root_fixed_point) {
          modk_cutpool_acceptor = [&](PoolCut&& row) -> int {
            std::vector<PoolCut> one_row;
            one_row.push_back(std::move(row));
            return add_root_transformed_cutpool_rows(one_row, "modk");
          };
          modk_acceptor_ptr = &modk_cutpool_acceptor;
        }
        const auto t_gen0 = std::chrono::steady_clock::now();
        const int generated = add_transformed_modk_cuts(
            root_lp, source_x, modk_simplex, opt,
            &root_implied_integer_cols, &implication_graph,
            &variable_bound_table,
            strict_highs_root_fixed_point ? nullptr : &generated_rows,
            strict_highs_root_fixed_point ? highs_mip_feastol : 1e-8,
            modk_acceptor_ptr, modk_cutgen_seed, &separator_storage_stats);
        root_source_trace.modk_generate_ms += root_source_elapsed_ms(t_gen0);
        const auto t_add0 = std::chrono::steady_clock::now();
        const int added =
            strict_highs_root_fixed_point
                ? generated
                : add_root_transformed_cutpool_rows(generated_rows, "modk");
        root_source_trace.modk_pool_add_ms += root_source_elapsed_ms(t_add0);
        if (opt.verbose || bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
          fmt::print(stderr,
                     "[B&C ROOT-XMODK-SRC] implied_int={} generated={} "
                     "pool_added={} pool={} sig={:016x}/{:016x} "
                     "frac={} frontier={} srcFrontier={} "
                     "varbounds={}/{} impl={}/{}\n",
                     implied_marked, generated, added,
                     static_cast<int>(root_transformed_cutpool.size()),
                     source_sig.lp_state, source_sig.source_state,
                     source_sig.fractional_integer_like,
                     source_sig.frontier_cols,
                     source_sig.source_frontier_cols,
                     static_cast<int>(source_sig.varbounds),
                     static_cast<int>(source_sig.total_varbounds),
                     static_cast<int>(source_sig.implications),
                     static_cast<int>(source_sig.total_implications));
        }
        root_source_trace.modk_ms += root_source_elapsed_ms(t_modk0);
        return added;
      };

		  auto separate_root_transformed_cutpool_resolve =
		      [&]() -> RootCutpoolResolveResult {
	        RootCutpoolResolveResult result;
			        ++root_source_trace.xpool_calls;
			        const auto t_xpool0 = std::chrono::steady_clock::now();
	        const std::size_t current_pool_size = root_transformed_cutpool.size();
	        if (root_xpool_noop_epoch == root_source_epoch &&
	            root_xpool_noop_pool_size == current_pool_size &&
	            root_transformed_cutpool_pending_source_rows == 0) {
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
		        }
	        if (strict_highs_root_fixed_point) {
	          fmt::print(stderr,
	                     "[B&C ROOT-XPOOL-SKIP] "
	                     "reason=strict_requires_direct_evaluateRootNode\n");
	          root_xpool_noop_epoch = root_source_epoch;
	          root_xpool_noop_pool_size = current_pool_size;
	          root_source_trace.xpool_ms += root_source_elapsed_ms(t_xpool0);
	          return result;
	        }
		    RootTransformedPoolStats pool_stats;
	        const auto t_select0 = std::chrono::steady_clock::now();
		    auto selected = select_root_transformed_cutpool(pool_stats);
        result.pool_stats = pool_stats;
        root_source_trace.xpool_select_ms +=
            root_source_elapsed_ms(t_select0);
	        root_source_trace.xpool_selected += static_cast<int>(selected.size());
	        auto xpool_ledger_enabled = []() {
	          const char* env = bc_env_options().value("MIPSOLVERS_XPOOL_LEDGER");
	          if (env != nullptr && env[0] != '\0' && env[0] != '0') return true;
	          env = bc_env_options().value("MIPSOLVERS_XTAB_DIAG");
	          return env != nullptr && env[0] != '\0' && env[0] != '0';
	        };
	        auto xpool_ledger_mix = [](std::uint64_t h,
	                                   std::uint64_t v) -> std::uint64_t {
	          v ^= v >> 33;
	          v *= std::uint64_t{0xff51afd7ed558ccd};
	          v ^= v >> 33;
	          v *= std::uint64_t{0xc4ceb9fe1a85ec53};
	          v ^= v >> 33;
	          return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) +
	                      (h >> 2));
	        };
		        auto xpool_ledger_double = [](double value) -> std::uint64_t {
		          return static_cast<std::uint64_t>(
		              HighsHashHelpers::double_hash_code(value));
		        };
		        auto xpool_state_trace_enabled = []() {
		          const char* env = bc_env_options().value("MIPSOLVERS_XPOOL_STATE_TRACE");

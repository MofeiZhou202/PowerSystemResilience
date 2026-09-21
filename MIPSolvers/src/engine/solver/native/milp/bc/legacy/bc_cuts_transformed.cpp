/// @file bc_cuts_transformed.cpp
/// @brief Transformed-tableau cut separators split from bc_cuts.cpp:
/// add_transformed_modk_cuts, add_transformed_tableau_cuts, add_transformed_path_cuts.

#include "bc_cuts_xtab_b.hpp"

namespace mipsolvers::engine::detail {

int add_transformed_modk_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    const std::vector<char>* implied_integer_cols,
    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table,
    std::vector<PoolCut>* generated_cutpool_rows,
    double cut_generation_feastol,
    const std::function<int(PoolCut&&)>* cutpool_acceptor,
    std::optional<std::uint64_t> highs_cutgen_seed,
    SeparatorStorageStats* storage_stats) {
  return add_transformed_modk_cuts_impl(
      lp, x, simplex, opt, implied_integer_cols, implication_graph,
      variable_bound_table, generated_cutpool_rows, cut_generation_feastol,
      cutpool_acceptor, highs_cutgen_seed, storage_stats);
}


int add_transformed_tableau_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::shared_ptr<BasisOps>& sbasis,
    const std::vector<char>* implied_integer_cols,
	    const BinaryImplicationGraph* implication_graph,
	    const VariableBoundTable* variable_bound_table,
	    std::vector<PoolCut>* generated_cutpool_rows,
	    double cut_generation_feastol,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor,
	    std::optional<std::uint64_t> highs_cutgen_seed,
        SeparatorStorageStats* storage_stats) {
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  const double tableau_feastol = cut_generation_feastol;
  const double tableau_fractionality_tol = 1000.0 * tableau_feastol;
  if (max_cuts <= 0 || !simplex.exact_optimal ||
      x.size() != simplex.form.n_original) {
    return 0;
  }
  if (!basis_tableau_cuts_admissible(simplex, opt, sbasis)) {
    return 0;
  }
  XTabSourceDiag diag;
  const int m = static_cast<int>(simplex.basis.indices.size());
  const int n_std = static_cast<int>(simplex.form.A.cols());
  const int n_orig = simplex.form.n_original;
  if (m <= 0 || n_std <= 0 || simplex.form.A_row.rows() != m ||
      simplex.form.b.size() != m || simplex.x_basic.size() != m ||
      simplex.x_std.size() != n_std || simplex.form.var_ub.size() != n_std) {
    return 0;
  }
  if (!sbasis && simplex.basis_inverse.rows() != m) {
    return 0;
  }
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
  if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    fmt::print(stderr,
               "[B&C-XTAB-CTX] family=tableau srcDim={} srcRows={} vlb={} "
               "vub={} intVlb={} intVub={}\n",
               source_context.dim, source_context.n_rows,
               transform_context.num_vlb, transform_context.num_vub,
               transform_context.num_integral_vlb,
               transform_context.num_integral_vub);
  }
  struct FractionalBasisRow {
    int row{-1};
    int basic_col{-1};
    int source_col{-1};
    double frac{0.0};
    double btran_scale{1.0};
    double score{0.0};
    std::vector<std::pair<int, double>> row_ep;
  };
  auto trace_fractional_rows =
      [&](const char* stage, const std::vector<FractionalBasisRow>& rows,
          const char* extra) {
        if (!xtab_row_trace_basis_enabled()) return;
        fmt::memory_buffer buffer;
        const int max_terms = xtab_row_trace_terms();
        int emitted = 0;
        for (const auto& fr : rows) {
          if (emitted >= max_terms) break;
          if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
          fmt::format_to(std::back_inserter(buffer),
                         "{}:{}:{}:{:.17g}:{:.17g}:{}",
                         fr.row, fr.basic_col, fr.source_col, fr.frac,
                         fr.score, fr.row_ep.size());
          ++emitted;
        }
        if (static_cast<int>(rows.size()) > emitted) {
          fmt::format_to(std::back_inserter(buffer), ";...");
        }
        fmt::print(stderr,
                   "[B&C-XROW] id=0 family=tableau stage={} {} "
                   "count={} sample=[{}]\n",
                   stage, extra == nullptr ? "" : extra, rows.size(),
                   fmt::to_string(buffer));
      };
  auto trace_row_ep_filter =
      [&](const FractionalBasisRow& fr, int raw_count, int kept_count,
          double min_weight, double max_weight, double norm2,
          const char* reject) {
        if (!xtab_row_trace_basis_enabled()) return;
        fmt::print(stderr,
                   "[B&C-XROW] id=0 family=tableau stage=rowep "
                   "basisRow={} basicVar={} sourceCol={} raw={} kept={} "
                   "min={:.12g} max={:.12g} ratio={:.12g} norm2={:.12g} "
                   "frac={:.12g} score={:.12g} reject={}\n",
                   fr.row, fr.basic_col, fr.source_col, raw_count, kept_count,
                   min_weight, max_weight,
                   min_weight > 0.0 ? max_weight / min_weight
                                    : std::numeric_limits<double>::infinity(),
                   norm2, fr.frac, fr.score, reject);
      };
  std::vector<FractionalBasisRow> fractional_rows;
  fractional_rows.reserve(static_cast<std::size_t>(m));
  for (int row = 0; row < m; ++row) {
    ++diag.basis_rows;
    const int basic_col = simplex.basis.indices[static_cast<std::size_t>(row)];
    int source_col = -1;
    double btran_scale = 1.0;
    if (!xtab_basic_source_info(simplex, source_context, basic_col, source_col,
                                btran_scale) ||
        source_col < 0 || source_col >= source_context.dim ||
        source_context.integral[static_cast<std::size_t>(source_col)] == 0) {
      continue;
    }
    if (source_col < n_orig) {
      ++diag.integer_basic_original;
    } else {
      ++diag.integer_basic_aux;
    }
    const double value =
        xtab_unscaled_source_value(simplex, source_context, source_col);
    const double frac = xtab_fractionality_distance(value);
    if (frac < tableau_fractionality_tol) {
      continue;
    }
    if (source_col < n_orig) {
      ++diag.fractional_basic_original;
    } else {
      ++diag.fractional_basic_aux;
    }
    fractional_rows.push_back(
        FractionalBasisRow{row, basic_col, source_col, frac, btran_scale, 0.0, {}});
  }
  trace_fractional_rows("frac_scan", fractional_rows,
                        "key=basis_order");
  if (fractional_rows.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  const std::int64_t num_tries_before = 0;
  std::int64_t max_tries = 5000;
  std::int64_t integral_cols = 0;
  for (int col = 0; col < n_orig; ++col) {
    if (source_context.integral[static_cast<std::size_t>(col)] != 0) {
      ++integral_cols;
    }
  }
  max_tries = std::min<std::int64_t>(
      max_tries,
      200 + static_cast<std::int64_t>(
                0.1 * static_cast<double>(std::min<std::int64_t>(
                          m, std::max<std::int64_t>(1, integral_cols)))));
  if (max_tries <= 0) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  const bool truncated =
      static_cast<std::int64_t>(fractional_rows.size()) > max_tries;
  if (truncated) {
    std::sort(fractional_rows.begin(), fractional_rows.end(),
              [&](const FractionalBasisRow& a,
                  const FractionalBasisRow& b) {
                return std::make_pair(
                           a.frac,
                           xtab_highs_hash_i64(num_tries_before + a.row)) >
                       std::make_pair(
                           b.frac,
                           xtab_highs_hash_i64(num_tries_before + b.row));
              });
    fractional_rows.resize(static_cast<std::size_t>(max_tries));
  }
  {
    const std::string extra =
        fmt::format("maxTries={} triesBefore={} truncated={} key={}",
                    max_tries, num_tries_before, truncated ? 1 : 0,
                    truncated ? "fractionality_hash" : "basis_order");
    trace_fractional_rows("pre_rowep_order", fractional_rows, extra.c_str());
  }

  for (auto& fr : fractional_rows) {
    Eigen::VectorXd y;
    std::vector<std::pair<int, double>> sparse_y;
    const bool have_sparse_y =
        sbasis && sbasis->kind() == BasisOpsKind::VendoredHighs &&
        sparse_basis_inverse_row_sparse_entries(sbasis, fr.row, sparse_y);
    if (sbasis) {
      if (!have_sparse_y && !sparse_basis_inverse_row(sbasis, fr.row, y)) {
        continue;
      }
    } else {
      if (simplex.basis_inverse.rows() != m ||
          simplex.basis_inverse.cols() != m) {
        continue;
      }
      y = simplex.basis_inverse.row(fr.row).transpose();
    }
    if (!have_sparse_y && (y.size() != m || !y.allFinite())) {
      continue;
    }
    ++diag.btran_rows;

    double norm2 = 0.0;
    double min_weight = std::numeric_limits<double>::infinity();
    double max_weight = 0.0;
    int raw_count = 0;
    fr.row_ep.reserve(have_sparse_y ? sparse_y.size()
                                    : static_cast<std::size_t>(m));
    auto consume_row_ep_entry = [&](int r, double raw_weight) {
      if (r < 0 || r >= m || !std::isfinite(raw_weight)) return;
      const double w = raw_weight * fr.btran_scale *
                       xtab_row_scale_or_one(simplex.form, r);
      ++raw_count;
      const double row_max = standard_row_max_abs(simplex, r);
      const double scaled_weight = row_max * std::abs(w);
      if (scaled_weight <= tableau_feastol) {
        return;
      }
      min_weight = std::min(min_weight, scaled_weight);
      max_weight = std::max(max_weight, scaled_weight);
      norm2 += scaled_weight * scaled_weight;
      fr.row_ep.emplace_back(r, w);
    };
    if (have_sparse_y) {
      for (const auto& entry : sparse_y) {
        consume_row_ep_entry(entry.first, entry.second);
      }
    } else {
      for (int r = 0; r < m; ++r) {
        const double raw_weight = y[r];
        if (std::abs(raw_weight) <= 1e-12) {
          continue;
        }
        consume_row_ep_entry(r, raw_weight);
      }
    }
    const int kept_count = static_cast<int>(fr.row_ep.size());
    if (fr.row_ep.size() <= 1 || !(norm2 > 0.0) ||
        !(min_weight > 0.0) || max_weight / min_weight > 1e4) {
      const char* reject =
          fr.row_ep.size() <= 1
              ? "count"
              : (!(norm2 > 0.0) || !(min_weight > 0.0) ? "weight"
                                                        : "ratio");
      trace_row_ep_filter(fr, raw_count, kept_count, min_weight, max_weight,
                          norm2, reject);
      fr.row_ep.clear();
      continue;
    }
    ++diag.row_ep_rows;
    fr.score = fr.frac * (1.0 - fr.frac) / norm2;
    trace_row_ep_filter(fr, raw_count, kept_count, min_weight, max_weight,
                        norm2, "none");
  }
  fractional_rows.erase(
      std::remove_if(fractional_rows.begin(), fractional_rows.end(),
                     [&](const FractionalBasisRow& row) {
                       return row.row_ep.empty() ||
                              row.score <= tableau_feastol;
                     }),
      fractional_rows.end());
  if (fractional_rows.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
  pdqsort_branchless(
      fractional_rows.begin(), fractional_rows.end(),
      [](const FractionalBasisRow& a, const FractionalBasisRow& b) {
        return a.score > b.score;
      });
  trace_fractional_rows("post_rowep_order", fractional_rows, "key=score");

  std::vector<XTabCandidateCut> candidates;
  candidates.reserve(static_cast<std::size_t>(max_cuts * 2));
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);

  auto tableau_basis_trace_info = [&](const FractionalBasisRow& fr,
                                      int sign) -> std::string {
    if (!xtab_row_trace_basis_enabled()) return {};
    fmt::memory_buffer buffer;
    const char* basic_kind = "unknown";
    int basic_logical_row = -1;
    if (fr.basic_col >= 0 && fr.basic_col < n_orig) {
      basic_kind = "col";
    } else {
      basic_logical_row = standard_form_aux_col_row(simplex.form, fr.basic_col);
      basic_kind = basic_logical_row >= 0 ? "row" : "aux";
    }
    fmt::format_to(std::back_inserter(buffer),
                   "basisRow={} basicVar={} basicKind={} basicLogicalRow={} "
                   "sourceCol={} sign={} frac={:.12g} score={:.12g} "
                   "btranScale={:.12g} rowEp=[",
                   fr.row, fr.basic_col, basic_kind, basic_logical_row,
                   fr.source_col, sign, fr.frac, fr.score, fr.btran_scale);

    int emitted = 0;
    const int max_terms = xtab_row_trace_terms();
    for (const auto& row_weight : fr.row_ep) {
      if (emitted >= max_terms) break;
      const int row = row_weight.first;
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
      double lower = 0.0;
      double upper = 0.0;
      const bool have_bounds =
          xtab_logical_row_bounds(simplex, row, lower, upper);
      fmt::format_to(std::back_inserter(buffer),
                     "row{}:w{:.12g}:lb{:.12g}:ub{:.12g}:sig[{}]",
                     row, row_weight.second,
                     have_bounds ? lower
                                 : -std::numeric_limits<double>::infinity(),
                     have_bounds ? upper
                                 : std::numeric_limits<double>::infinity(),
                     xtab_source_row_signature(simplex, row, max_terms));
      ++emitted;
    }
    if (static_cast<int>(fr.row_ep.size()) > emitted) {
      fmt::format_to(std::back_inserter(buffer), ";...");
    }
    fmt::format_to(std::back_inserter(buffer), "]");
    return fmt::to_string(buffer);
  };

  const double density_cap =
      std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz =
      std::max(2, static_cast<int>(std::ceil(density_cap *
                                             static_cast<double>(n_orig))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);
  const double min_activity =
      std::min(1.0, std::max(0.0, opt.gmi_min_activity));
  const double min_binary_support =
      std::min(1.0, std::max(0.0, opt.gmi_min_binary_support));
  const double activity_weight = std::max(0.0, opt.gmi_activity_weight);

  double best_score = -1.0;
  int accepted_cutpool_rows = 0;
  for (const auto& fr : fractional_rows) {
    const int accepted_or_generated =
        cutpool_acceptor != nullptr
            ? accepted_cutpool_rows
            : static_cast<int>(candidates.size());
    if (cutpool_acceptor != nullptr) {
      if (accepted_cutpool_rows >= 1000) {
        break;
      }
    } else if (accepted_or_generated >= 4 * max_cuts) {
      break;
    }
    const double best_score_factor =
        (cutpool_acceptor != nullptr && accepted_cutpool_rows >= 50)
            ? 0.01
            : 0.0025;
    if (best_score > 0.0 && fr.score < best_score_factor * best_score) {
      break;
    }

    {
      const auto& row_ep_variant = fr.row_ep;
      Eigen::VectorXd base_coeff;
      double base_rhs = 0.0;
      std::vector<int> base_active_cols;
      if (!aggregate_standard_rows(simplex, source_context, row_ep_variant,
                                   base_coeff, base_rhs,
                                   &base_active_cols, storage_stats)) {
        continue;
      }
      ++diag.aggregate_ok;
      const int base_nnz = count_nonzeros(base_coeff);
      if (base_nnz > static_cast<int>(row_ep_variant.size()) &&
          10 * (base_nnz - static_cast<int>(row_ep_variant.size())) >
              10000 + n_orig) {
        continue;
      }

      double max_abs = 0.0;
      double min_abs = std::numeric_limits<double>::infinity();
      for (int j = 0; j < n_orig; ++j) {
        const double a = std::abs(base_coeff[j]);
        if (a <= 1e-12) {
          continue;
        }
        max_abs = std::max(max_abs, a);
        min_abs = std::min(min_abs, a);
      }
      if (!(min_abs > 0.0) || max_abs / min_abs > 1e6) {
        continue;
      }

      for (int sign : {1, -1}) {
        Eigen::VectorXd cut;
        double rhs = 0.0;
        double efficacy = 0.0;
        const Eigen::VectorXd signed_coeff =
            (sign == 1) ? base_coeff : (-base_coeff);
        record_dense_workspace(storage_stats,
                               static_cast<std::size_t>(signed_coeff.size()));
        const double signed_rhs = (sign == 1) ? base_rhs : (-base_rhs);
        const std::string basis_info = tableau_basis_trace_info(fr, sign);
        XTabRejectReason reject_reason = XTabRejectReason::None;
        if (!xtab_generate_cut_from_base_row(simplex, source_context,
                                             signed_coeff, signed_rhs,
                                             &base_active_cols, x,
                                             implied_integer_cols,
                                             &transform_context,
	                                             &diag.vb_substitutions,
	                                             &diag.vb_trigger_terms, cut, rhs,
	                                             efficacy, cut_generation_feastol,
	                                             &reject_reason, &diag,
		                                             /*require_violation=*/true,
                                             "tableau",
                                             basis_info.empty()
                                                 ? nullptr
                                                 : basis_info.c_str(),
                                             &cutgen_random)) {
          xtab_record_reject(&diag, reject_reason);
          continue;
        }
	        ++diag.generated;
	        record_dense_workspace(storage_stats,
	                               static_cast<std::size_t>(cut.size()));
	        const int nnz = count_nonzeros(cut);
	        const double norm = std::max(1e-12, cut.norm());
	        const double activity = gmi_binary_activity_score(simplex, x, cut);
	        const double score = efficacy * (1.0 + activity_weight * activity);
        if (cutpool_acceptor != nullptr) {
          XTabCandidateCut candidate{std::move(cut), rhs, score, efficacy,
                                    norm, nnz};
          storage_tracker.record_ephemeral(candidate.coeff);
          PoolCut row = xtab_candidate_to_pool_cut(candidate);
          const int added = (*cutpool_acceptor)(std::move(row));
          if (added > 0) {
            accepted_cutpool_rows += added;
          }
          continue;
        }
	        if (generated_cutpool_rows != nullptr) {
	          candidates.push_back(
	              XTabCandidateCut{std::move(cut), rhs, score, efficacy, norm, nnz});
          storage_tracker.record(candidates.back().coeff, candidates.size());
          continue;
        }
        if (nnz > max_nnz) {
          ++diag.filtered_density;
          continue;
        }
	        if (efficacy < min_efficacy) {
	          ++diag.filtered_efficacy;
	          continue;
	        }
	        if (activity < min_activity) {
	          ++diag.filtered_activity;
	          continue;
	        }
        const double binary_support = gmi_binary_support_ratio(simplex, cut);
        if (binary_support < min_binary_support) {
          ++diag.filtered_binary_support;
          continue;
        }
	        candidates.push_back(
	            XTabCandidateCut{std::move(cut), rhs, score, efficacy, norm, nnz});
        storage_tracker.record(candidates.back().coeff, candidates.size());
      }
    }
    if (best_score < 0.0 &&
        (cutpool_acceptor != nullptr ? accepted_cutpool_rows > 0
                                     : !candidates.empty())) {
      best_score = fr.score;
    }
  }

  if (cutpool_acceptor != nullptr) {
    maybe_print_xtab_diag("tableau", diag);
    return accepted_cutpool_rows;
  }

  if (candidates.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
	  if (generated_cutpool_rows != nullptr) {
	    generated_cutpool_rows->reserve(generated_cutpool_rows->size() +
	                                    candidates.size());
	    for (const auto& cand : candidates) {
	      generated_cutpool_rows->push_back(xtab_candidate_to_pool_cut(cand));
	    }
	    maybe_print_xtab_diag("tableau", diag);
	    return static_cast<int>(candidates.size());
	  }
  std::sort(candidates.begin(), candidates.end(),
            [](const XTabCandidateCut& a, const XTabCandidateCut& b) {
              if (std::abs(a.score - b.score) > 1e-12) {
                return a.score > b.score;
              }
              return a.rhs < b.rhs;
            });

  std::vector<XTabCandidateCut> selected;
  selected.reserve(static_cast<std::size_t>(max_cuts));
  for (const auto& cand : candidates) {
    if (static_cast<int>(selected.size()) >= max_cuts) {
      break;
    }
    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm,
                                keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (near_parallel) {
      ++diag.filtered_parallel;
      continue;
    }
    selected.push_back(cand);
    ++diag.selected;
  }
  if (selected.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  std::vector<Eigen::SparseVector<double>> rows;
  std::vector<double> rhs;
  rows.reserve(selected.size());
  rhs.reserve(selected.size());
  for (const auto& s : selected) {
    rows.push_back(s.coeff);
    rhs.push_back(s.rhs);
  }
  add_sparse_rows_to_lp(lp, rows, rhs, storage_stats);
  maybe_print_xtab_diag("tableau", diag);
  return static_cast<int>(selected.size());
}

int add_transformed_path_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::vector<char>* implied_integer_cols,
	    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table,
    std::vector<PoolCut>* generated_cutpool_rows,
    double cut_generation_feastol,
    const std::function<int(PoolCut&&)>* cutpool_acceptor,
    std::optional<std::uint64_t> highs_cutgen_seed,
    const std::function<int(int)>* highs_path_randint,
    SeparatorStorageStats* storage_stats) {
  const bool direct_cutpool_mode = cutpool_acceptor != nullptr;
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  if ((!direct_cutpool_mode && max_cuts <= 0) || !simplex.exact_optimal ||
      x.size() != simplex.form.n_original) {
    return 0;
  }
  XTabSourceDiag diag;
  int ledger_source_dim = 0;
  struct XPathLedger {
    int row_eq{0};
    int row_leq{0};
    int row_geq{0};
    int row_unusable{0};
    int integer_like_cols{0};
    int continuous_cols{0};
    int continuous_bd_cols{0};
    int rows_with_continuous{0};
    int substitution_rows{0};
    int in_arcs{0};
    int out_arcs{0};
    int usable_start_rows{0};
    std::uint64_t start_attempts{0};
    std::uint64_t path_iterations{0};
    std::uint64_t aggregation_calls{0};
    std::uint64_t substitution_events{0};
    std::uint64_t path_extensions{0};
    std::uint64_t try_negated_scale{0};
    std::uint64_t cutgen_calls{0};
    std::uint64_t cutgen_success{0};
    std::uint64_t cutpool_calls{0};
    std::uint64_t cutpool_accepted{0};
    std::uint64_t candidate_rows{0};
    std::uint64_t mix_attempts{0};
    std::uint64_t mix_success{0};
    std::uint64_t mix_accepted{0};
  } path_ledger;
  auto print_path_ledger = [&](const char* stage,
                               int accepted_cutpool_rows,
                               std::size_t candidates_size) {
    if (bc_env_options().value("MIPSOLVERS_XPATH_LEDGER") == nullptr &&
        bc_env_options().value("MIPSOLVERS_XTAB_DIAG") == nullptr) {
      return;
    }
    path_ledger.candidate_rows = static_cast<std::uint64_t>(candidates_size);
    fmt::print(
        stderr,
        "[B&C-XPATH-LEDGER] stage={} mode={} srcRows={} srcDim={} "
        "rowtype=eq{}:le{}:ge{}:bad{} cols=int{}:cont{}:bd{} "
        "rowsCont={} subst={} arcs=in{}:out{} usableStart={} "
        "starts={} iters={} aggs={} substEvents={} extensions={} "
        "tryNeg={} cutgen={}/{} gen={} mix={}/{}/{} "
        "poolCalls={} poolAccepted={} acceptedRows={} candidates={}\n",
        stage, direct_cutpool_mode ? "cutpool" : "bounded",
        simplex.form.A_row.rows(), ledger_source_dim,
        path_ledger.row_eq, path_ledger.row_leq, path_ledger.row_geq,
        path_ledger.row_unusable, path_ledger.integer_like_cols,
        path_ledger.continuous_cols, path_ledger.continuous_bd_cols,
        path_ledger.rows_with_continuous, path_ledger.substitution_rows,
        path_ledger.in_arcs, path_ledger.out_arcs,
        path_ledger.usable_start_rows, path_ledger.start_attempts,
        path_ledger.path_iterations, path_ledger.aggregation_calls,
        path_ledger.substitution_events, path_ledger.path_extensions,
        path_ledger.try_negated_scale, path_ledger.cutgen_success,
        path_ledger.cutgen_calls, diag.generated, path_ledger.mix_success,
        path_ledger.mix_attempts, path_ledger.mix_accepted,
        path_ledger.cutpool_calls, path_ledger.cutpool_accepted,
        accepted_cutpool_rows, path_ledger.candidate_rows);
  };
  const int n_orig = simplex.form.n_original;
  const int m = static_cast<int>(simplex.form.A_row.rows());
  if (m <= 0 || simplex.form.b.size() != m ||
      simplex.x_std.size() != simplex.form.A.cols()) {
    return 0;
  }
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  ledger_source_dim = source_context.valid ? source_context.dim : 0;
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("invalid_context", 0, 0);
    return 0;
  }
  if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    fmt::print(stderr,
               "[B&C-XTAB-CTX] family=path srcDim={} srcRows={} vlb={} "
               "vub={} intVlb={} intVub={}\n",
               source_context.dim, source_context.n_rows,
               transform_context.num_vlb, transform_context.num_vub,
               transform_context.num_integral_vlb,
               transform_context.num_integral_vub);
  }
  std::vector<unsigned char> integer_like_orig(
      static_cast<std::size_t>(n_orig), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (!transformed_col_is_integer_like(simplex, col,
                                         implied_integer_cols)) {
      continue;
    }
    integer_like_orig[static_cast<std::size_t>(col)] = 1;
  }
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      ++path_ledger.integer_like_cols;
    } else {
      ++path_ledger.continuous_cols;
    }
  }
  enum class PathRowType : signed char {
    Unusable = -2,
    Geq = -1,
    Eq = 0,
    Leq = 1,
  };
  std::vector<PathRowType> row_type(static_cast<std::size_t>(m),
                                    PathRowType::Unusable);
	  const double feastol = cut_generation_feastol;
  for (int r = 0; r < m; ++r) {
    const int scol = source_context.n_original + r;
    const double lower = source_context.lower[scol];
    const double upper = source_context.upper[scol];
    const double act = source_context.solution[scol];
    if (std::isfinite(lower) && std::isfinite(upper) &&
        std::abs(upper - lower) <= feastol) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Eq;
      continue;
    }
    double lower_slack = std::numeric_limits<double>::infinity();
    double upper_slack = std::numeric_limits<double>::infinity();
    if (std::isfinite(lower)) {
      lower_slack = act - lower;
    }
    if (std::isfinite(upper)) {
      upper_slack = upper - act;
    }
    if (lower_slack > feastol && upper_slack > feastol) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Unusable;
    } else if (lower_slack < upper_slack) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Geq;
    } else {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Leq;
    }
  }
  for (PathRowType type : row_type) {
    switch (type) {
      case PathRowType::Eq:
        ++path_ledger.row_eq;
        break;
      case PathRowType::Leq:
        ++path_ledger.row_leq;
        break;
      case PathRowType::Geq:
        ++path_ledger.row_geq;
        break;
      case PathRowType::Unusable:
        ++path_ledger.row_unusable;
        break;
    }
  }

  auto bound_distance = [&](int col) -> double {
    if (col < 0 || col >= n_orig) {
      return 0.0;
    }
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      return 0.0;
    }
    if (col >= source_context.solution.size() ||
        col >= source_context.lower.size() ||
        col >= source_context.upper.size()) {
      return 0.0;
    }
    const double val = source_context.solution[col];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    if (!std::isfinite(val)) {
      return 0.0;
    }
    double lb_dist = std::isfinite(lb) ? std::max(0.0, val - lb)
                                       : std::numeric_limits<double>::infinity();
    double ub_dist = std::numeric_limits<double>::infinity();
    if (std::isfinite(ub)) {
      ub_dist = std::max(0.0, ub - val);
    }
    if (transform_context.best_vlb.size() > static_cast<std::size_t>(col)) {
      const XTabVarBoundExpr& vlb =
          transform_context.best_vlb[static_cast<std::size_t>(col)];
      if (vlb.valid && vlb.trigger_col >= 0 &&
          vlb.trigger_col < source_context.solution.size()) {
        const double trig = source_context.solution[vlb.trigger_col];
        const double expr = vlb.constant + vlb.coef * trig;
        if (std::isfinite(expr)) {
          lb_dist = std::min(lb_dist, std::max(0.0, val - expr));
        }
      }
    }
    if (transform_context.best_vub.size() > static_cast<std::size_t>(col)) {
      const XTabVarBoundExpr& vub =
          transform_context.best_vub[static_cast<std::size_t>(col)];
      if (vub.valid && vub.trigger_col >= 0 &&
          vub.trigger_col < source_context.solution.size()) {
        const double trig = source_context.solution[vub.trigger_col];
        const double expr = vub.constant + vub.coef * trig;
        if (std::isfinite(expr)) {
          ub_dist = std::min(ub_dist, std::max(0.0, expr - val));
        }
      }
    }
    const double dist = std::min(lb_dist, ub_dist);
    return std::isfinite(dist) && dist > feastol ? dist : 0.0;
  };
  std::vector<double> col_bound_distance(static_cast<std::size_t>(n_orig), 0.0);
  for (int col = 0; col < n_orig; ++col) {
    col_bound_distance[static_cast<std::size_t>(col)] = bound_distance(col);
    if (integer_like_orig[static_cast<std::size_t>(col)] == 0 &&
        col_bound_distance[static_cast<std::size_t>(col)] != 0.0) {
      ++path_ledger.continuous_bd_cols;
    }
  }

  struct PathArc {
    int row{-1};
    double coeff{0.0};
  };

  std::vector<int> num_continuous(static_cast<std::size_t>(m), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0 ||
        col_bound_distance[static_cast<std::size_t>(col)] == 0.0) {
      continue;
    }
    for (StandardColumnMatrix::InnerIterator it(simplex.form.A, col);
         it; ++it) {
      const int row = static_cast<int>(it.row());
      if (row >= 0 && row < m) {
        ++num_continuous[static_cast<std::size_t>(row)];
      }
    }
  }
  for (int r = 0; r < m; ++r) {
    if (num_continuous[static_cast<std::size_t>(r)] > 0) {
      ++path_ledger.rows_with_continuous;
    }
  }

  std::vector<PathArc> col_substitution(static_cast<std::size_t>(n_orig),
                                        PathArc{-1, 0.0});
  for (int r = 0; r < m; ++r) {
    if (row_type[static_cast<std::size_t>(r)] != PathRowType::Eq ||
        num_continuous[static_cast<std::size_t>(r)] != 1) {
      continue;
    }
    int subst_col = -1;
    double subst_val = 0.0;
	    for (StandardRowMatrix::InnerIterator it(
	             simplex.form.A_row, r);
	         it; ++it) {
	      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= n_orig ||
          integer_like_orig[static_cast<std::size_t>(col)] != 0 ||
          col_bound_distance[static_cast<std::size_t>(col)] == 0.0) {
        continue;
      }
	      subst_col = col;
	      subst_val = it.value() / (xtab_row_scale_or_one(simplex.form, r) *
	                                 xtab_col_scale_or_one(simplex.form, col));
	      break;
    }
    if (subst_col >= 0 &&
        col_substitution[static_cast<std::size_t>(subst_col)].row < 0) {
      col_substitution[static_cast<std::size_t>(subst_col)] =
          PathArc{r, subst_val};
      row_type[static_cast<std::size_t>(r)] = PathRowType::Unusable;
      ++path_ledger.substitution_rows;
    }
  }

  std::vector<double> col_source_drive(static_cast<std::size_t>(n_orig), 0.0);
  std::vector<std::vector<PathArc>> col_in_arcs(
      static_cast<std::size_t>(n_orig));
  std::vector<std::vector<PathArc>> col_out_arcs(
      static_cast<std::size_t>(n_orig));
  for (int col = 0; col < n_orig; ++col) {
    if (col_substitution[static_cast<std::size_t>(col)].row >= 0 ||
        integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      continue;
    }
    const double bd = col_bound_distance[static_cast<std::size_t>(col)];
	    if (bd <= feastol) {
	      continue;
	    }
	    col_source_drive[static_cast<std::size_t>(col)] = bd;
  }

  for (int r = 0; r < m; ++r) {
    const PathRowType type = row_type[static_cast<std::size_t>(r)];
    if (type == PathRowType::Unusable) {
      continue;
    }
    for (StandardRowMatrix::InnerIterator it(
             simplex.form.A_row, r);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= n_orig) {
        continue;
      }
      if (col_source_drive[static_cast<std::size_t>(col)] <= feastol) {
        continue;
      }
      const double a =
          it.value() / (xtab_row_scale_or_one(simplex.form, r) *
                        xtab_col_scale_or_one(simplex.form, col));
      if (std::abs(a) <= 1e-12) {
        continue;
      }
      auto& in_arcs = col_in_arcs[static_cast<std::size_t>(col)];
      auto& out_arcs = col_out_arcs[static_cast<std::size_t>(col)];
      switch (type) {
        case PathRowType::Leq:
          if (a < 0.0) {
            in_arcs.push_back(PathArc{r, a});
          } else {
            out_arcs.push_back(PathArc{r, a});
          }
          break;
        case PathRowType::Geq:
          if (a > 0.0) {
            in_arcs.push_back(PathArc{r, a});
          } else {
            out_arcs.push_back(PathArc{r, a});
          }
          break;
        case PathRowType::Eq:
          in_arcs.push_back(PathArc{r, a});
          out_arcs.push_back(PathArc{r, a});
          break;
        case PathRowType::Unusable:
          break;
      }
    }
  }
  for (const auto& arcs : col_in_arcs) {
    path_ledger.in_arcs += static_cast<int>(arcs.size());
  }
  for (const auto& arcs : col_out_arcs) {
    path_ledger.out_arcs += static_cast<int>(arcs.size());
  }

  int usable_start_rows = 0;
  for (int r = 0; r < m; ++r) {
    if (row_type[static_cast<std::size_t>(r)] != PathRowType::Unusable) {
      ++usable_start_rows;
    }
  }
  path_ledger.usable_start_rows = usable_start_rows;
  if (usable_start_rows == 0) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("no_start_rows", 0, 0);
    return 0;
  }
  if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    if (direct_cutpool_mode) {
      fmt::print(stderr,
                 "[B&C-XTAB-PATH-SCHED] mode=highs_row_order rows={} "
                 "contentCap=none\n",
                 usable_start_rows);
    } else {
      fmt::print(stderr,
                 "[B&C-XTAB-PATH-SCHED] mode=highs_row_order rows={} "
                 "maxCuts={}\n",
                 usable_start_rows, max_cuts);
    }
  }

  std::vector<XTabCandidateCut> candidates;
  const int reserve_hint =
      direct_cutpool_mode ? 8 : std::max(8, max_cuts * 2);
  candidates.reserve(static_cast<std::size_t>(reserve_hint));
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);
  int accepted_cutpool_rows = 0;

  const double density_cap =
      std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz =
      std::max(2, static_cast<int>(std::ceil(density_cap *
                                             static_cast<double>(n_orig))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);

  auto row_in_path = [](const std::vector<std::pair<int, double>>& path,
                        int row) {
    return std::any_of(path.begin(), path.end(),
                       [&](const auto& p) { return p.first == row; });
  };

  auto add_path_cuts_from_aggregation = [&](const Eigen::SparseVector<double>& coeff,
                                            double rhs,
                                            const std::vector<int>& active_cols) {
    ++diag.aggregate_ok;
    bool accepted_any = false;
    for (int sign : {1, -1}) {
      ++path_ledger.cutgen_calls;
      Eigen::VectorXd cut;
      double cut_rhs = 0.0;
      double efficacy = 0.0;
      Eigen::VectorXd signed_coeff = Eigen::VectorXd(coeff);
      if (sign == -1) signed_coeff *= -1.0;
      record_dense_workspace(storage_stats,
                             static_cast<std::size_t>(signed_coeff.size()));
          const double signed_rhs = sign == 1 ? rhs : (-rhs);
      XTabRejectReason reject_reason = XTabRejectReason::None;
        std::uint64_t source_trace_id = 0;
      if (!xtab_generate_cut_from_base_row(simplex, source_context,
                                           signed_coeff, signed_rhs,
                                           &active_cols, x,
                                           implied_integer_cols,
                                           &transform_context,
	                                           &diag.vb_substitutions,
	                                           &diag.vb_trigger_terms, cut, cut_rhs,
	                                           efficacy,
	                                           cut_generation_feastol,
		                                           &reject_reason, &diag,
		                                           /*require_violation=*/true,
                                           "path", nullptr, &cutgen_random,
                                           /*only_initial_cmir_scale=*/false,
                                           &source_trace_id)) {
        xtab_record_reject(&diag, reject_reason);
        continue;
      }
      const int nnz = count_nonzeros(cut);
      record_dense_workspace(storage_stats,
                             static_cast<std::size_t>(cut.size()));
      const double norm = std::max(1e-12, cut.norm());
      const double score = efficacy;
      if (cutpool_acceptor != nullptr) {
        XTabCandidateCut candidate{std::move(cut), cut_rhs, score, efficacy,
                                  norm, nnz, source_trace_id};
        storage_tracker.record_ephemeral(candidate.coeff);
        PoolCut row = xtab_candidate_to_pool_cut(candidate);
        const int added = (*cutpool_acceptor)(std::move(row));
        ++path_ledger.cutpool_calls;
        if (added > 0) {
          accepted_cutpool_rows += added;
          path_ledger.cutpool_accepted += static_cast<std::uint64_t>(added);
          ++path_ledger.cutgen_success;
          ++diag.generated;
          accepted_any = true;
        }
        continue;
      }
      if (generated_cutpool_rows != nullptr) {
        candidates.push_back(XTabCandidateCut{std::move(cut), cut_rhs, score,
                                              efficacy, norm, nnz,
                                              source_trace_id});
        storage_tracker.record(candidates.back().coeff, candidates.size());
        ++path_ledger.cutgen_success;
        ++diag.generated;
        accepted_any = true;
        continue;
      }
      if (nnz > max_nnz || efficacy < min_efficacy) {
        if (nnz > max_nnz) {
          ++diag.filtered_density;
        } else {
          ++diag.filtered_efficacy;
        }
        continue;
      }
      candidates.push_back(XTabCandidateCut{std::move(cut), cut_rhs, score,
                                            efficacy, norm, nnz,
                                            source_trace_id});
      storage_tracker.record(candidates.back().coeff, candidates.size());
      ++path_ledger.cutgen_success;
      ++diag.generated;
      accepted_any = true;
    }
    return accepted_any;
  };

  auto try_path_mixing =
      [&](const std::vector<XTabAggregatedSourceRow>& path_aggs) {
    if (path_aggs.size() < 2) {
      return;
    }
    ++path_ledger.mix_attempts;
    Eigen::VectorXd cut;
    double cut_rhs = 0.0;
    double efficacy = 0.0;
    XTabRejectReason reject_reason = XTabRejectReason::None;
    if (!xtab_generate_path_mixing_cut(simplex, source_context, path_aggs, x,
                                       implied_integer_cols, &transform_context,
                                       &diag.vb_substitutions,
                                       &diag.vb_trigger_terms, cut, cut_rhs,
	                                       efficacy, cut_generation_feastol,
	                                       &reject_reason, "pathmix",
                                       storage_stats)) {
      xtab_record_reject(&diag, reject_reason);
      return;
    }
    const int nnz = count_nonzeros(cut);
    record_dense_workspace(storage_stats,
                           static_cast<std::size_t>(cut.size()));
    const double norm = std::max(1e-12, cut.norm());
    const double score = efficacy;
    if (cutpool_acceptor != nullptr) {
      XTabCandidateCut candidate{std::move(cut), cut_rhs, score, efficacy,
                                norm, nnz};
      storage_tracker.record_ephemeral(candidate.coeff);
      PoolCut row = xtab_candidate_to_pool_cut(candidate);
      const int added = (*cutpool_acceptor)(std::move(row));
      ++path_ledger.cutpool_calls;
      if (added > 0) {
        accepted_cutpool_rows += added;
        path_ledger.cutpool_accepted += static_cast<std::uint64_t>(added);
        path_ledger.mix_accepted += static_cast<std::uint64_t>(added);
        ++path_ledger.mix_success;
        ++diag.generated;
      }
      return;
    }
    if (generated_cutpool_rows != nullptr) {
      candidates.push_back(
          XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm, nnz});
      storage_tracker.record(candidates.back().coeff, candidates.size());
      ++path_ledger.mix_success;
      ++diag.generated;
      return;
    }
    if (nnz > max_nnz || efficacy < min_efficacy) {
      if (nnz > max_nnz) {
        ++diag.filtered_density;
      } else {
        ++diag.filtered_efficacy;
      }
      return;
    }
    candidates.push_back(
        XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm, nnz});
    storage_tracker.record(candidates.back().coeff, candidates.size());
    ++path_ledger.mix_success;
    ++diag.generated;
  };

  constexpr int kMaxPathLen = 6;
  const double max_weight = 1.0 / feastol;
  const double min_weight = feastol;
  auto weight_ok = [&](double w) {
    w = std::abs(w);
    return std::isfinite(w) && w >= min_weight && w <= max_weight;
  };
  HighsRandom fallback_path_randgen(0);

  auto skip_col = [&](int col,
                      const std::vector<std::vector<PathArc>>& same_arcs,
                      const std::vector<std::vector<PathArc>>& other_arcs,
                      const std::vector<std::pair<int, double>>& path,
                      int start_row,
                      bool& try_negated_scale) {
    const auto& same = same_arcs[static_cast<std::size_t>(col)];
    const auto& other = other_arcs[static_cast<std::size_t>(col)];
    if (path.size() == 1 && !try_negated_scale) {
      if (same.size() <= path.size()) {
        for (const PathArc& arc : same) {
          if (arc.row != start_row) {
            try_negated_scale = true;
            break;
          }
        }
      } else {
        try_negated_scale = true;
      }
    }
    if (other.empty()) {
      return true;
    }
    if (other.size() <= path.size()) {
      for (const PathArc& arc : other) {
        if (!row_in_path(path, arc.row)) return false;
      }
      return true;
    }
    return false;
  };

  auto find_row = [&](int col,
                      double val,
                      const std::vector<std::vector<PathArc>>& arcs_by_col,
                      const std::vector<std::pair<int, double>>& path,
                      int& row,
                      double& weight) {
    const auto& arcs = arcs_by_col[static_cast<std::size_t>(col)];
    if (arcs.empty()) {
      return false;
    }
    int start_pos = 0;
    if (cutpool_acceptor != nullptr) {
      if (highs_path_randint != nullptr) {
        start_pos = (*highs_path_randint)(static_cast<int>(arcs.size()));
      } else {
        start_pos = static_cast<int>(
            fallback_path_randgen.integer(static_cast<HighsInt>(arcs.size())));
      }
      if (start_pos < 0 || start_pos >= static_cast<int>(arcs.size())) {
        start_pos = 0;
      }
    }
    for (int step = 0; step < static_cast<int>(arcs.size()); ++step) {
      const PathArc& arc =
          arcs[static_cast<std::size_t>((start_pos + step) %
                                        static_cast<int>(arcs.size()))];
      if (row_in_path(path, arc.row) || std::abs(arc.coeff) <= 1e-12) {
        continue;
      }
      const double w = -val / arc.coeff;
      if (!weight_ok(w)) {
        continue;
      }
      row = arc.row;
      weight = w;
      return true;
    }
    return false;
  };

  XTabLpAggregator path_aggregator(simplex, source_context, storage_stats);
  for (int start_row = 0; start_row < m; ++start_row) {
      if (row_type[static_cast<std::size_t>(start_row)] ==
          PathRowType::Unusable) {
        continue;
      }
      std::vector<double> start_scales;
      const PathRowType type = row_type[static_cast<std::size_t>(start_row)];
      if (type == PathRowType::Leq) {
        start_scales = {1.0, -1.0};
      } else if (type == PathRowType::Geq) {
        start_scales = {-1.0, 1.0};
      } else {
        double row_dual = 0.0;
        if (simplex.result.constraint_duals.size() == m) {
          row_dual = simplex.result.constraint_duals[start_row];
        }
        start_scales = row_dual <= 1e-12 ? std::vector<double>{1.0, -1.0}
                                         : std::vector<double>{-1.0, 1.0};
      }
      for (double start_scale : start_scales) {
        ++path_ledger.start_attempts;
        path_aggregator.clear();
        std::vector<std::pair<int, double>> path;
        std::vector<XTabAggregatedSourceRow> path_aggs;
        bool try_negated_scale = false;
        if (!path_aggregator.add_row(start_row, start_scale)) {
          break;
        }
        path.emplace_back(start_row, start_scale);
	        while (static_cast<int>(path.size()) < kMaxPathLen) {
          ++path_ledger.path_iterations;
          Eigen::SparseVector<double> coeff;
          double rhs = 0.0;
          std::vector<int> active_cols;
          if (!path_aggregator.get_current_aggregation(
                  coeff, rhs, false, &active_cols)) {
            break;
          }
          ++path_ledger.aggregation_calls;

          int best_out_col = -1;
          double best_out_val = 0.0;
          double best_out_dist = 0.0;
          int best_in_col = -1;
          double best_in_val = 0.0;
          double best_in_dist = 0.0;
          bool added_substitution_rows = false;
          bool abort_path = false;

          for (int col : active_cols) {
            if (col < 0 || col >= n_orig) {
              continue;
            }
            const double val = coeff.coeff(col);
            if (std::abs(val) <= 1e-10 ||
                col_bound_distance[static_cast<std::size_t>(col)] <= feastol ||
                integer_like_orig[static_cast<std::size_t>(col)] != 0) {
              continue;
            }
            const PathArc subst =
                col_substitution[static_cast<std::size_t>(col)];
	            if (subst.row >= 0 && std::abs(subst.coeff) > 1e-12) {
	              const double w = -val / subst.coeff;
	              if (std::isfinite(w)) {
	                if (!path_aggregator.add_row(subst.row, w)) {
	                  abort_path = true;
	                  break;
	                }
	                ++path_ledger.substitution_events;
	                added_substitution_rows = true;
	                continue;
	              }
            }
            if (added_substitution_rows) {
              continue;
            }

            if (val < 0.0) {
              if (skip_col(col, col_out_arcs, col_in_arcs, path, start_row,
                           try_negated_scale)) {
                continue;
              }
              const double dist =
                  col_bound_distance[static_cast<std::size_t>(col)];
              if (best_out_col < 0 || dist > best_out_dist) {
                best_out_col = col;
                best_out_val = val;
                best_out_dist = dist;
              }
            } else {
              if (skip_col(col, col_in_arcs, col_out_arcs, path, start_row,
                           try_negated_scale)) {
                continue;
              }
              const double dist =
                  col_bound_distance[static_cast<std::size_t>(col)];
              if (best_in_col < 0 || dist > best_in_dist) {
                best_in_col = col;
                best_in_val = val;
                best_in_dist = dist;
              }
            }
          }
          if (abort_path) {
            break;
          }
          if (added_substitution_rows) {
            continue;
          }

          const std::size_t before_cuts = candidates.size();
          const int before_accepted = accepted_cutpool_rows;
          const bool success =
              add_path_cuts_from_aggregation(coeff, rhs, active_cols);
          if (!path_aggs.empty() || best_out_col >= 0 || best_in_col >= 0) {
            XTabAggregatedSourceRow agg;
            agg.coeff = coeff;
            agg.coeff *= -1.0;
            agg.rhs = -rhs;
            agg.active_cols = active_cols;
            path_aggs.push_back(std::move(agg));
          }
          const bool cut_accepted_or_generated =
              cutpool_acceptor != nullptr
                  ? accepted_cutpool_rows > before_accepted
                  : (success && candidates.size() > before_cuts);
          if (cut_accepted_or_generated ||
              (best_out_col < 0 && best_in_col < 0)) {
            break;
          }

          int next_row = -1;
          double next_weight = 0.0;
          if (best_in_col < 0 ||
              (best_out_col >= 0 &&
               best_out_dist >= best_in_dist - feastol)) {
            if (!find_row(best_out_col, best_out_val, col_in_arcs, path,
                          next_row, next_weight)) {
              if (best_in_col < 0 ||
                  !find_row(best_in_col, best_in_val, col_out_arcs, path,
                            next_row, next_weight)) {
                break;
              }
            }
          } else {
            if (!find_row(best_in_col, best_in_val, col_out_arcs, path,
                          next_row, next_weight)) {
              break;
            }
          }
          if (!path_aggregator.add_row(next_row, next_weight)) {
            break;
          }
          ++path_ledger.path_extensions;
          path.emplace_back(next_row, next_weight);
        }
        try_path_mixing(path_aggs);
        if (try_negated_scale) ++path_ledger.try_negated_scale;
        if (!try_negated_scale) {
          break;
        }
      }
      path_aggregator.clear();
  }

  if (cutpool_acceptor != nullptr) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("done", accepted_cutpool_rows, candidates.size());
    return accepted_cutpool_rows;
  }

  if (candidates.empty()) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("empty_candidates", accepted_cutpool_rows,
                      candidates.size());
    return 0;
  }
	  if (generated_cutpool_rows != nullptr) {
	    generated_cutpool_rows->reserve(generated_cutpool_rows->size() +
	                                    candidates.size());
	    for (const auto& cand : candidates) {
	      generated_cutpool_rows->push_back(xtab_candidate_to_pool_cut(cand));
	    }
	    maybe_print_xtab_diag("path", diag);
	    print_path_ledger("generated_rows", accepted_cutpool_rows,
	                      candidates.size());
	    return static_cast<int>(candidates.size());
	  }
  std::sort(candidates.begin(), candidates.end(),
            [](const XTabCandidateCut& a, const XTabCandidateCut& b) {
              return a.score > b.score;
            });
  std::vector<XTabCandidateCut> selected;
  selected.reserve(static_cast<std::size_t>(max_cuts));
  for (const auto& cand : candidates) {
    if (static_cast<int>(selected.size()) >= max_cuts) {
      break;
    }
    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm,
                                keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (!near_parallel) {
      selected.push_back(cand);
      ++diag.selected;
    } else {
      ++diag.filtered_parallel;
    }
  }
  if (selected.empty()) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("empty_selected", accepted_cutpool_rows,
                      candidates.size());
    return 0;
  }
  std::vector<Eigen::SparseVector<double>> rows;
  std::vector<double> rhs;
  rows.reserve(selected.size());
  rhs.reserve(selected.size());
  for (const auto& s : selected) {
    rows.push_back(s.coeff);
    rhs.push_back(s.rhs);
  }
  add_sparse_rows_to_lp(lp, rows, rhs, storage_stats);
  maybe_print_xtab_diag("path", diag);
  print_path_ledger("selected", accepted_cutpool_rows, candidates.size());
  return static_cast<int>(selected.size());
}

// ============================================================================
// Clique cuts from binary conflict graph
// ============================================================================
// For constraints sum_j a_j x_j <= b where all a_j > 0 and vars are binary,
// detect pairs (x_i, x_j) where a_i + a_j > b (conflict: both can't be 1).
// Extend conflicts greedily into maximal cliques. Cut: sum_{i in clique} x_i <= 1.


}  // namespace mipsolvers::engine::detail

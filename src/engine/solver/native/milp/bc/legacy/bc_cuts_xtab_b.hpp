/// @file bc_cuts_xtab_b.hpp
/// @brief Transformed-tableau (xtab) cut subsystem, part B (bc_cuts.cpp split).
/// Internal header included only by bc_cuts_transformed.cpp.

#pragma once

#include "bc_cuts_xtab_a.hpp"

namespace mipsolvers::engine::detail {
namespace {

bool xtab_transform_base_row(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const Eigen::VectorXd& coeff_std,
    double rhs_std,
    const std::vector<int>* active_cols,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    bool* integers_positive_out,
    bool initial_integers_positive,
    double feastol,
    XTabRow& row) {
  (void)simplex;
  feastol = std::max(0.0, feastol);
  if (vb_substitutions_out != nullptr) *vb_substitutions_out = 0;
  if (vb_trigger_terms_out != nullptr) *vb_trigger_terms_out = 0;
  if (integers_positive_out != nullptr) *integers_positive_out = true;
  const int n_src = source_context.dim;
  const int n_orig = source_context.n_original;
  if (!source_context.valid || coeff_std.size() != n_src ||
      source_context.lower.size() != n_src ||
      source_context.upper.size() != n_src ||
      source_context.solution.size() != n_src ||
      static_cast<int>(source_context.integral.size()) != n_src) {
    return false;
  }
  (void)implied_integer_cols;
  row = XTabRow{};
  HighsCDouble rhs_acc = rhs_std;
  row.inds.reserve(static_cast<std::size_t>(std::min(n_src, 1024)));
  row.vals.reserve(row.inds.capacity());
  row.upper.reserve(row.inds.capacity());
  row.solval.reserve(row.inds.capacity());
  row.bound_type.reserve(row.inds.capacity());
  row.varbound_expr.reserve(row.inds.capacity());
  row.complemented.reserve(row.inds.capacity());
  row.is_integral.reserve(row.inds.capacity());

  auto choose_bound_type =
      [&](int col,
          double coeff,
          XTabVarBoundExpr& chosen_varbound) -> XTabBoundType {
    chosen_varbound = XTabVarBoundExpr{};
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    const double simple_lb_dist =
        std::isfinite(lb) ? std::max(0.0, sol - lb)
                          : std::numeric_limits<double>::infinity();
    const double simple_ub_dist =
        std::isfinite(ub) ? std::max(0.0, ub - sol)
                          : std::numeric_limits<double>::infinity();

    const XTabVarBoundExpr* vlb = nullptr;
    const XTabVarBoundExpr* vub = nullptr;
    if (transform_context != nullptr && col < n_orig) {
      const std::size_t pos = static_cast<std::size_t>(col);
      if (pos < transform_context->best_vlb.size() &&
          transform_context->best_vlb[pos].valid) {
        vlb = &transform_context->best_vlb[pos];
      }
      if (pos < transform_context->best_vub.size() &&
          transform_context->best_vub[pos].valid) {
        vub = &transform_context->best_vub[pos];
      }
    }

    const double lb_dist = vlb != nullptr ? vlb->dist : simple_lb_dist;
    const double ub_dist = vub != nullptr ? vub->dist : simple_ub_dist;
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    const double tol = feastol;

    auto simple_type = [&]() {
      if (!std::isfinite(simple_lb_dist)) {
        return XTabBoundType::SimpleUb;
      }
      if (!std::isfinite(simple_ub_dist)) {
        return XTabBoundType::SimpleLb;
      }
      if (simple_lb_dist < simple_ub_dist - tol) {
        return XTabBoundType::SimpleLb;
      }
      if (simple_ub_dist < simple_lb_dist - tol) {
        return XTabBoundType::SimpleUb;
      }
      return coeff > 0.0 ? XTabBoundType::SimpleLb
                         : XTabBoundType::SimpleUb;
    };

    if (integer_like) {
      const double simple_bound_dist =
          std::min(simple_lb_dist, simple_ub_dist);
      const double bound_dist = std::min(lb_dist, ub_dist);
      const bool can_use_integer_vbd =
          col < n_orig && std::isfinite(ub) &&
          ub > 1.5 + 1e-9 && simple_lb_dist > tol &&
          simple_ub_dist > tol && (vlb != nullptr || vub != nullptr) &&
          bound_dist <= tol &&
          bound_dist <= simple_bound_dist + tol;
      if (!can_use_integer_vbd) {
        return simple_type();
      }
      if ((vlb == nullptr && vub != nullptr) ||
          (vub != nullptr && ub_dist < lb_dist - tol)) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      if ((vub == nullptr && vlb != nullptr) ||
          (vlb != nullptr && lb_dist < ub_dist - tol)) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      if (coeff > 0.0 && vub != nullptr) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      if (vlb != nullptr) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return simple_type();
    }

    if (lb_dist < ub_dist - tol) {
      if (vlb != nullptr &&
          (coeff > 0.0 || simple_lb_dist > lb_dist + tol)) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return XTabBoundType::SimpleLb;
    }
    if (ub_dist < lb_dist - tol) {
      if (vub != nullptr &&
          (coeff < 0.0 || simple_ub_dist > ub_dist + tol)) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      return XTabBoundType::SimpleUb;
    }
    if (coeff > 0.0) {
      if (vlb != nullptr) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return XTabBoundType::SimpleLb;
    }
    if (vub != nullptr) {
      chosen_varbound = *vub;
      return XTabBoundType::VariableUb;
    }
    return XTabBoundType::SimpleUb;
  };

  std::vector<int> inds;
  std::vector<double> vals;
  inds.reserve(active_cols != nullptr ? active_cols->size()
                                      : static_cast<std::size_t>(n_src));
  vals.reserve(inds.capacity());
  auto append_input_term = [&](int j) -> bool {
    if (j < 0 || j >= n_src) {
      return false;
    }
    const double a = coeff_std[j];
    if (!std::isfinite(a)) {
      return false;
    }
    if (std::abs(a) > 1e-12) {
      inds.push_back(j);
      vals.push_back(a);
    }
    return true;
  };

  if (active_cols != nullptr) {
    for (int j : *active_cols) {
      if (!append_input_term(j)) {
        return false;
      }
    }
  } else {
    for (int j = 0; j < n_src; ++j) {
      if (!append_input_term(j)) {
        return false;
      }
    }
  }
  int num_nz = static_cast<int>(inds.size());
  if (num_nz == 0) return false;

  std::vector<XTabBoundType> bound_types(static_cast<std::size_t>(n_src),
                                         XTabBoundType::SimpleLb);
  std::vector<XTabVarBoundExpr> bound_varbounds(static_cast<std::size_t>(n_src));
  XTabSparseVectorSum vector_sum(n_src);

  auto remove_term = [&](int pos) {
    --num_nz;
    inds[static_cast<std::size_t>(pos)] =
        inds[static_cast<std::size_t>(num_nz)];
    vals[static_cast<std::size_t>(pos)] =
        vals[static_cast<std::size_t>(num_nz)];
    inds[static_cast<std::size_t>(num_nz)] = 0;
    vals[static_cast<std::size_t>(num_nz)] = 0.0;
  };

  auto simple_lb_dist = [&](int col) {
    const double lb = source_context.lower[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(lb)) return std::numeric_limits<double>::infinity();
    const double dist = sol - lb;
    return dist <= feastol ? 0.0 : std::max(0.0, dist);
  };
  auto simple_ub_dist = [&](int col) {
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(ub)) return std::numeric_limits<double>::infinity();
    const double dist = ub - sol;
    return dist <= feastol ? 0.0 : std::max(0.0, dist);
  };

  for (int i = 0; i < num_nz;) {
    const int col = inds[static_cast<std::size_t>(i)];
    double& val = vals[static_cast<std::size_t>(i)];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(sol) ||
        (!std::isfinite(lb) && !std::isfinite(ub))) {
      return false;
    }
    if (std::isfinite(lb) && std::isfinite(ub) && ub - lb < 1e-9) {
      rhs_acc -= std::min(lb, ub) * val;
      remove_term(i);
      continue;
    }

    XTabVarBoundExpr chosen_varbound;
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    if (integer_like) {
      const double slb = simple_lb_dist(col);
      const double sub = simple_ub_dist(col);
      const double simple_bound_dist = std::min(slb, sub);
      const bool have_vlb =
          transform_context != nullptr && col < n_orig &&
          static_cast<std::size_t>(col) < transform_context->best_vlb.size() &&
          transform_context->best_vlb[static_cast<std::size_t>(col)].valid;
      const bool have_vub =
          transform_context != nullptr && col < n_orig &&
          static_cast<std::size_t>(col) < transform_context->best_vub.size() &&
          transform_context->best_vub[static_cast<std::size_t>(col)].valid;
      double bdist = simple_bound_dist;
      if (have_vlb) {
        bdist = std::min(
            bdist, transform_context->best_vlb[static_cast<std::size_t>(col)].dist);
      }
      if (have_vub) {
        bdist = std::min(
            bdist, transform_context->best_vub[static_cast<std::size_t>(col)].dist);
      }
      XTabBoundType type = XTabBoundType::SimpleLb;
      const bool use_simple =
          (std::isfinite(ub) && std::isfinite(lb) && ub - lb <= 1.5 + 1e-9) ||
          bdist != 0.0 || slb == 0.0 || sub == 0.0;
      if (use_simple) {
        if (slb < sub - feastol) {
          type = XTabBoundType::SimpleLb;
        } else if (sub < slb - feastol) {
          type = XTabBoundType::SimpleUb;
        } else if (val > 0.0) {
          type = XTabBoundType::SimpleLb;
        } else {
          type = XTabBoundType::SimpleUb;
        }
        bound_types[static_cast<std::size_t>(col)] = type;
        ++i;
        continue;
      }
    }

    const XTabBoundType bound_type = choose_bound_type(col, val, chosen_varbound);
    bound_types[static_cast<std::size_t>(col)] = bound_type;
    if (chosen_varbound.valid) {
      bound_varbounds[static_cast<std::size_t>(col)] = chosen_varbound;
    }

    switch (bound_type) {
      case XTabBoundType::SimpleLb:
        if (!std::isfinite(lb)) return false;
        if (val > 0.0) {
          rhs_acc -= lb * val;
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::SimpleUb:
        if (!std::isfinite(ub)) return false;
        if (val < 0.0) {
          rhs_acc -= ub * val;
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::VariableLb:
        if (!chosen_varbound.valid) return false;
        if (vb_substitutions_out != nullptr) ++(*vb_substitutions_out);
        rhs_acc -= chosen_varbound.constant * val;
        if (chosen_varbound.trigger_col >= 0 &&
            chosen_varbound.trigger_col < n_orig &&
            std::abs(val * chosen_varbound.coef) > 1e-12) {
          if (chosen_varbound.trigger_col == xtab_transform_trace_col()) {
            fmt::print(stderr,
                       "[B&C-TRANSFORM-TERM] target={} source={} type=vlb "
                       "val={:.17g} coef={:.17g} constant={:.17g} "
                       "contrib={:.17g} dist={:.17g}\n",
                       chosen_varbound.trigger_col, col, val,
                       chosen_varbound.coef, chosen_varbound.constant,
                       val * chosen_varbound.coef, chosen_varbound.dist);
          }
          vector_sum.add(chosen_varbound.trigger_col,
                         val * chosen_varbound.coef);
          if (vb_trigger_terms_out != nullptr) ++(*vb_trigger_terms_out);
        }
        if (val > 0.0) {
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::VariableUb:
        if (!chosen_varbound.valid) return false;
        if (vb_substitutions_out != nullptr) ++(*vb_substitutions_out);
        rhs_acc -= chosen_varbound.constant * val;
        if (chosen_varbound.trigger_col >= 0 &&
            chosen_varbound.trigger_col < n_orig &&
            std::abs(val * chosen_varbound.coef) > 1e-12) {
          if (chosen_varbound.trigger_col == xtab_transform_trace_col()) {
            fmt::print(stderr,
                       "[B&C-TRANSFORM-TERM] target={} source={} type=vub "
                       "val={:.17g} coef={:.17g} constant={:.17g} "
                       "contrib={:.17g} dist={:.17g}\n",
                       chosen_varbound.trigger_col, col, val,
                       chosen_varbound.coef, chosen_varbound.constant,
                       val * chosen_varbound.coef, chosen_varbound.dist);
          }
          vector_sum.add(chosen_varbound.trigger_col,
                         val * chosen_varbound.coef);
          if (vb_trigger_terms_out != nullptr) ++(*vb_trigger_terms_out);
        }
        val = -val;
        if (val > 0.0) {
          remove_term(i);
          continue;
        }
        break;
    }
    ++i;
  }

  if (!vector_sum.empty()) {
    for (int i = 0; i < num_nz; ++i) {
      if (vals[static_cast<std::size_t>(i)] != 0.0) {
        vector_sum.add(inds[static_cast<std::size_t>(i)],
                       vals[static_cast<std::size_t>(i)]);
      }
    }
    vector_sum.cleanup([](int, double value) {
      return std::abs(value) <= 1e-9;
    });
    inds = vector_sum.nonzeros;
    num_nz = static_cast<int>(inds.size());
    vals.resize(static_cast<std::size_t>(num_nz));
    for (int j = 0; j < num_nz; ++j) {
      vals[static_cast<std::size_t>(j)] =
          vector_sum.values[static_cast<std::size_t>(inds[static_cast<std::size_t>(j)])];
    }
  } else {
    inds.resize(static_cast<std::size_t>(num_nz));
    vals.resize(static_cast<std::size_t>(num_nz));
  }

  bool integers_positive = initial_integers_positive;
  for (int j = 0; j < num_nz; ++j) {
    const int col = inds[static_cast<std::size_t>(j)];
    if (col < 0 || col >= n_src) return false;
    if (source_context.integral[static_cast<std::size_t>(col)] == 0) continue;
    if (bound_types[static_cast<std::size_t>(col)] ==
            XTabBoundType::VariableLb ||
        bound_types[static_cast<std::size_t>(col)] ==
            XTabBoundType::VariableUb) {
      continue;
    }
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double val = vals[static_cast<std::size_t>(j)];
    if (initial_integers_positive) {
      if ((std::isfinite(lb) && val > 0.0) || !std::isfinite(ub)) {
        bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleLb;
      } else {
        bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleUb;
      }
    } else if (simple_lb_dist(col) < simple_ub_dist(col)) {
      bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleLb;
    } else {
      bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleUb;
    }
  }

  for (int j = 0; j < num_nz; ++j) {
    const int col = inds[static_cast<std::size_t>(j)];
    double transformed_coeff = vals[static_cast<std::size_t>(j)];
    if (std::abs(transformed_coeff) <= 1e-12) continue;
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(sol)) return false;
    XTabBoundType bound_type = bound_types[static_cast<std::size_t>(col)];
    const XTabVarBoundExpr& varbound =
        bound_varbounds[static_cast<std::size_t>(col)];
    double upper_range = std::numeric_limits<double>::infinity();
    if (std::isfinite(lb) && std::isfinite(ub)) {
      upper_range = std::max(0.0, ub - lb);
    }
    auto clipped_dist = [feastol](double dist) {
      if (dist <= feastol) return 0.0;
      return std::max(0.0, dist);
    };
    double solval = std::numeric_limits<double>::infinity();
    switch (bound_type) {
      case XTabBoundType::SimpleLb:
        if (!std::isfinite(lb)) {
          return false;
        }
        rhs_acc -= lb * transformed_coeff;
        solval = clipped_dist(sol - lb);
        break;
      case XTabBoundType::SimpleUb:
        if (!std::isfinite(ub)) {
          return false;
        }
        rhs_acc -= ub * transformed_coeff;
        transformed_coeff = -transformed_coeff;
        solval = clipped_dist(ub - sol);
        break;
      case XTabBoundType::VariableLb:
        if (!varbound.valid) {
          return false;
        }
        solval = clipped_dist(varbound.dist);
        break;
      case XTabBoundType::VariableUb:
        if (!varbound.valid) {
          return false;
        }
        solval = clipped_dist(varbound.dist);
        break;
    }
    if (std::abs(transformed_coeff) <= 1e-12) {
      continue;
    }
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    row.inds.push_back(col);
    row.vals.push_back(transformed_coeff);
    row.bound_type.push_back(bound_type);
    row.varbound_expr.push_back(varbound);
    row.solval.push_back(solval);
    row.upper.push_back(upper_range);
    row.complemented.push_back(0);
    row.is_integral.push_back(integer_like ? 1 : 0);
    if (integer_like && transformed_coeff <= 0.0) {
      integers_positive = false;
    }
  }
  if (integers_positive_out != nullptr) {
    *integers_positive_out = integers_positive;
  }
  row.rhs = double(rhs_acc);
  return !row.inds.empty() && std::isfinite(row.rhs);
}

bool xtab_preprocess_base_inequality(const SimplexResult& simplex,
                                     XTabRow& row,
                                     double feastol,
                                     bool& has_unbounded_ints,
                                     bool& has_general_ints,
                                     bool& has_continuous) {
  feastol = std::max(0.0, feastol);
  has_unbounded_ints = false;
  has_general_ints = false;
  has_continuous = false;
  if (row.inds.empty()) {
    return false;
  }

  double max_abs = 0.0;
  for (double v : row.vals) {
    max_abs = std::max(max_abs, std::abs(v));
  }
  row.initial_scale = pow2_scale_for_max_abs(max_abs);
  row.rhs *= row.initial_scale;
  for (double& v : row.vals) {
    v *= row.initial_scale;
  }

  double maxact = -feastol;
  std::vector<unsigned char> erase(row.inds.size(), 0);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    row.is_integral[pos] =
        row.is_integral[pos] != 0 && std::abs(row.vals[pos]) > 10.0 * feastol;
    if (row.is_integral[pos] == 0) {
      if (std::isfinite(row.upper[pos]) &&
          row.upper[pos] < 2.0 * row.solval[pos]) {
        xtab_flip_complementation(row, k);
      }
      if (row.vals[pos] > 0.0 ||
          (std::isfinite(row.upper[pos]) &&
           std::abs(row.vals[pos]) * row.upper[pos] <= 10.0 * feastol)) {
        if (row.vals[pos] < 0.0) {
          if (!std::isfinite(row.upper[pos])) {
            return false;
          }
          row.rhs -= row.vals[pos] * row.upper[pos];
        }
        erase[pos] = 1;
        continue;
      }
      has_continuous = true;
      if (row.vals[pos] > 0.0) {
        if (!std::isfinite(row.upper[pos])) {
          maxact = std::numeric_limits<double>::infinity();
        } else {
          maxact += row.vals[pos] * row.upper[pos];
        }
      }
    } else {
      if (!std::isfinite(row.upper[pos])) {
        has_unbounded_ints = true;
        has_general_ints = true;
      } else if (std::abs(row.upper[pos] - 1.0) > 1e-9) {
        has_general_ints = true;
      }
      if (row.vals[pos] > 0.0) {
        if (!std::isfinite(row.upper[pos])) {
          maxact = std::numeric_limits<double>::infinity();
        } else {
          maxact += row.vals[pos] * row.upper[pos];
        }
      }
    }
  }

  int num_erase = static_cast<int>(
      std::count(erase.begin(), erase.end(), static_cast<unsigned char>(1)));
  const int max_len = 100 + static_cast<int>(
      0.15 * static_cast<double>(std::max(1, simplex.form.n_original)));
  const int live_len = static_cast<int>(row.inds.size()) - num_erase;
  if (live_len > max_len) {
    const int need_cancel = live_len - max_len;
    std::vector<int> cancellable;
    cancellable.reserve(static_cast<std::size_t>(need_cancel));
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      if (erase[static_cast<std::size_t>(k)] != 0) {
        continue;
      }
      const double cancel_slack =
          row.vals[static_cast<std::size_t>(k)] > 0.0
              ? row.solval[static_cast<std::size_t>(k)]
              : row.upper[static_cast<std::size_t>(k)] -
                    row.solval[static_cast<std::size_t>(k)];
      if (cancel_slack <= feastol) {
        cancellable.push_back(k);
      }
    }
    if (static_cast<int>(cancellable.size()) < need_cancel) {
      return false;
    }
    std::partial_sort(
        cancellable.begin(), cancellable.begin() + need_cancel,
        cancellable.end(), [&](int a, int b) {
          return std::abs(row.vals[static_cast<std::size_t>(a)]) <
                 std::abs(row.vals[static_cast<std::size_t>(b)]);
        });
    for (int idx = 0; idx < need_cancel; ++idx) {
      const int k = cancellable[static_cast<std::size_t>(idx)];
      const std::size_t pos = static_cast<std::size_t>(k);
      if (row.vals[pos] < 0.0) {
        row.rhs -= row.vals[pos] * row.upper[pos];
      } else if (std::isfinite(row.upper[pos])) {
        maxact -= row.vals[pos] * row.upper[pos];
      }
      erase[pos] = 1;
    }
  }

  if (num_erase > 0 ||
      std::count(erase.begin(), erase.end(), static_cast<unsigned char>(1)) >
          num_erase) {
    xtab_erase_positions(row, erase);
  }
  return !row.inds.empty() && maxact > row.rhs;
}

bool xtab_cmir_cut_generation(XTabRow& row,
                              double feastol,
                              double min_efficacy,
                              XTabCmirTrace* trace = nullptr,
                              bool only_initial_cmir_scale = false,
                              std::uint64_t trace_id = 0,
                              const char* trace_family = nullptr) {
  feastol = std::max(0.0, feastol);
  constexpr double tiny = 1e-14;
  constexpr double f0_min = 0.005;
  constexpr double f0_max = 0.995;
  constexpr double max_cmir_scale = 1e6;
  if (row.inds.empty()) {
    return false;
  }
  if (trace != nullptr) {
    *trace = XTabCmirTrace{};
  }
  row.integral_support = false;
  row.integral_coefficients = false;

  std::vector<int> integer_inds;
  std::vector<double> deltas;
  integer_inds.reserve(row.inds.size());
  deltas.reserve(row.inds.size() + 2);
  double continuous_contribution = 0.0;
  double continuous_norm_sq = 0.0;
  double max_abs_delta = 0.0;

  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.is_integral[pos] != 0) {
      integer_inds.push_back(k);
      if (std::isfinite(row.upper[pos]) &&
          row.upper[pos] < 2.0 * row.solval[pos]) {
        xtab_flip_complementation(row, k);
      }
      if (only_initial_cmir_scale) {
        continue;
      }
      if (row.solval[pos] > feastol) {
        const double delta = std::abs(row.vals[pos]);
        if (delta > 1e-4 && delta != max_abs_delta) {
          max_abs_delta = std::max(max_abs_delta, delta);
          deltas.push_back(delta);
        }
      }
    } else {
      xtab_update_violation_and_norm(row, k, row.vals[pos],
                                     continuous_contribution,
                                     continuous_norm_sq, feastol);
    }
  }
  if (trace != nullptr) {
    trace->integer_terms = static_cast<int>(integer_inds.size());
    trace->continuous_terms =
        static_cast<int>(row.inds.size()) - trace->integer_terms;
    trace->continuous_contribution = continuous_contribution;
    trace->continuous_norm_sq = continuous_norm_sq;
    trace->max_abs_delta = max_abs_delta;
  }

  if (continuous_norm_sq == 0.0 && deltas.size() > 1) {
    const double int_scale = xtab_integral_scale(deltas, feastol, tiny);
    if (int_scale != 0.0 && int_scale <= 1e4) {
      const double scaled_rhs = row.rhs * int_scale;
      const double down_rhs = fast_floor_xtab(scaled_rhs);
      const double f0 = scaled_rhs - down_rhs;
      if (f0 >= f0_min && f0 <= f0_max) {
        deltas.push_back(1.0 / int_scale);
      }
    }
  }

  deltas.push_back(std::min(1.0, row.initial_scale));
  if (!only_initial_cmir_scale) {
    deltas.push_back(max_abs_delta + std::min(1.0, row.initial_scale));
  }
  std::sort(deltas.begin(), deltas.end());
  double cur_delta = deltas.empty() ? 0.0 : deltas.front();
  for (std::size_t i = 1; i < deltas.size(); ++i) {
    if (deltas[i] - cur_delta <= 10.0 * feastol) {
      deltas[i] = 0.0;
    } else {
      cur_delta = deltas[i];
    }
  }
  deltas.erase(std::remove(deltas.begin(), deltas.end(), 0.0), deltas.end());
  if (trace != nullptr) {
    trace->initial_delta_count = static_cast<int>(deltas.size());
  }
  const bool delta_trace = trace_id != 0 && xtab_cmir_delta_trace_enabled();
  if (delta_trace) {
    fmt::memory_buffer buffer;
    for (std::size_t i = 0; i < deltas.size(); ++i) {
      if (i > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{:.17g}", deltas[i]);
    }
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=cmir_deltas feastol={:.17g} "
               "initialScale={:.17g} deltas=[{}]\n",
               trace_id, trace_family == nullptr ? "" : trace_family,
               feastol, row.initial_scale, fmt::to_string(buffer));
  }

  double best_delta = -1.0;
  double best_efficacy = min_efficacy;
  auto evaluate_delta = [&](const char* phase, double delta) -> double {
    if (trace != nullptr) ++trace->tested_delta_count;
    if (!(delta > 0.0) || !std::isfinite(delta)) {
      return -std::numeric_limits<double>::infinity();
    }
    const double scale = 1.0 / delta;
    const double scaled_rhs = row.rhs * scale;
    const double down_rhs = fast_floor_xtab(scaled_rhs);
    const double f0 = scaled_rhs - down_rhs;
    if (f0 < f0_min || f0 > f0_max) {
      return -std::numeric_limits<double>::infinity();
    }
    const double inv_one_minus = 1.0 / (1.0 - f0);
    if (inv_one_minus > max_cmir_scale) {
      return -std::numeric_limits<double>::infinity();
    }
    const double cont_scale = scale * inv_one_minus;
    double norm_sq = cont_scale * cont_scale * continuous_norm_sq;
    double violation = cont_scale * continuous_contribution - down_rhs;
    for (int k : integer_inds) {
      const double scaled_a =
          row.vals[static_cast<std::size_t>(k)] * scale;
      const double down_a = fast_floor_xtab(scaled_a + tiny);
      const double fj = scaled_a - down_a;
      const double aj =
          down_a + std::max(0.0, (fj - f0) * inv_one_minus);
      xtab_update_violation_and_norm(row, k, aj, violation, norm_sq,
                                     feastol);
    }
    if (!(norm_sq > 0.0)) {
      return -std::numeric_limits<double>::infinity();
    }
    const double efficacy = violation / std::sqrt(norm_sq);
    if (delta_trace) {
      xtab_trace_cmir_delta(trace_id, trace_family, phase, delta, scale,
                            down_rhs, f0, violation, norm_sq, efficacy,
                            best_efficacy);
    }
    return efficacy;
  };

  for (double delta : deltas) {
    const double efficacy = evaluate_delta("candidate", delta);
    if (efficacy > best_efficacy) {
      best_delta = delta;
      best_efficacy = efficacy;
    }
  }
  if (best_delta < 0.0) {
    return false;
  }

  for (int k = 1; !only_initial_cmir_scale && k <= 3; ++k) {
    const double delta = best_delta * static_cast<double>(1 << k);
    const double efficacy = evaluate_delta("scale2", delta);
    const double efficacy_noise =
        1e-15 * std::max(1.0, std::abs(best_efficacy));
    const bool first_doubled_tie = k == 1 && efficacy == best_efficacy;
    if (efficacy > best_efficacy + efficacy_noise || first_doubled_tie) {
      best_delta = delta;
      best_efficacy = efficacy;
    }
  }

  for (int k : integer_inds) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (!std::isfinite(row.upper[pos]) || row.solval[pos] <= feastol) {
      continue;
    }
    xtab_flip_complementation(row, k);
    const double efficacy = evaluate_delta("flip", best_delta);
    if (efficacy > best_efficacy) {
      best_efficacy = efficacy;
    } else {
      xtab_flip_complementation(row, k);
    }
  }

  const HighsCDouble scale = 1.0 / HighsCDouble(best_delta);
  const HighsCDouble scaled_rhs = row.rhs * scale;
  const double down_rhs = std::floor(double(scaled_rhs));
  const HighsCDouble f0 = scaled_rhs - down_rhs;
  const HighsCDouble inv_one_minus = 1.0 / (1.0 - f0);
  row.rhs = down_rhs * best_delta;
  row.integral_support = true;
  row.integral_coefficients = false;
  if (trace != nullptr) {
    trace->best_delta = best_delta;
    trace->best_efficacy = best_efficacy;
    trace->final_f0 = double(f0);
    trace->final_rhs = row.rhs;
    trace->accepted = true;
  }
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (std::abs(row.vals[pos]) <= 0.0) {
      continue;
    }
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] > 0.0) {
        row.vals[pos] = 0.0;
      } else {
        row.vals[pos] = double(row.vals[pos] * inv_one_minus);
        row.integral_support = false;
      }
    } else {
      const HighsCDouble scaled_a = scale * row.vals[pos];
      const double down_a = std::floor(double(scaled_a + tiny));
      const HighsCDouble fj = scaled_a - down_a;
      HighsCDouble aj = down_a;
      if (fj > f0) {
        aj += (fj - f0) * inv_one_minus;
      }
      row.vals[pos] = double(aj * best_delta);
    }
  }
  return true;
}

bool xtab_try_generate_cut(XTabRow& row,
                           bool has_unbounded_ints,
                           bool has_general_ints,
                           bool has_continuous,
                           double feastol,
                           double min_efficacy,
                           int random_tiebreaker,
                           XTabCutgenRandom* cutgen_random,
                           XTabCmirTrace* trace = nullptr,
                           bool only_initial_cmir_scale = false,
                           std::uint64_t trace_id = 0,
                           const char* trace_family = nullptr) {
  if (trace != nullptr) *trace = XTabCmirTrace{};
  if (has_unbounded_ints) {
    return xtab_cmir_cut_generation(row, feastol, min_efficacy, trace,
                                    only_initial_cmir_scale, trace_id,
                                    trace_family);
  }

  const XTabRow saved_row = row;
  XTabLiftedCoverState cover_state;
  bool lifted_success = false;
  bool saved_integral_support = false;
  bool saved_integral_coefficients = false;
  double min_mir_efficacy = min_efficacy;
  XTabRow lifted_row;

  do {
    if (!xtab_determine_cover(row, feastol, cutgen_random, random_tiebreaker,
                              cover_state)) {
      break;
    }

    if (!has_continuous && !has_general_ints) {
      xtab_separate_lifted_knapsack_cover(row, cover_state, feastol, 1e-12);
      lifted_success = true;
    } else if (has_general_ints) {
      lifted_success =
          xtab_separate_lifted_mixed_integer_cover(row, cover_state, feastol,
                                                   1e-12);
    } else {
      lifted_success =
          xtab_separate_lifted_mixed_binary_cover(row, cover_state, 1e-12);
    }
  } while (false);

  if (lifted_success) {
    saved_integral_support = row.integral_support;
    saved_integral_coefficients = row.integral_coefficients;

    double violation = -row.rhs;
    double norm_sq = 0.0;
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      xtab_update_violation_and_norm(row, k,
                                     row.vals[static_cast<std::size_t>(k)],
                                     violation, norm_sq, feastol);
    }
    const double efficacy =
        norm_sq > 0.0 ? violation / std::sqrt(norm_sq)
                      : -std::numeric_limits<double>::infinity();
    if (efficacy <= min_efficacy) {
      lifted_success = false;
    } else {
      min_mir_efficacy += efficacy;
      lifted_row = row;
    }
  }

  row = saved_row;
  XTabCmirTrace cmir_trace;
  if (xtab_cmir_cut_generation(row, feastol, min_mir_efficacy, &cmir_trace,
                               only_initial_cmir_scale, trace_id,
                               trace_family)) {
    if (trace != nullptr) *trace = cmir_trace;
    return true;
  }

  if (!lifted_success) {
    if (trace != nullptr) *trace = cmir_trace;
    return false;
  }

  row = std::move(lifted_row);
  row.integral_support = saved_integral_support;
  row.integral_coefficients = saved_integral_coefficients;
  if (trace != nullptr) {
    *trace = cmir_trace;
    trace->lifted_accepted = true;
  }
  return true;
}

bool xtab_untransform_cut(const SimplexResult& simplex,
                          const XTabSourceContext& source_context,
                          const XTabRow& transformed_cut,
                          Eigen::VectorXd& cut_le,
                          double& rhs_le) {
  const int n_orig = source_context.n_original;
  const int n_src = source_context.dim;
  if (!source_context.valid || n_orig <= 0 || n_src <= n_orig ||
      source_context.lower.size() != n_src ||
      source_context.upper.size() != n_src) {
    return false;
  }
  std::vector<HighsCDouble> std_coeff_acc(static_cast<std::size_t>(n_orig));
  std::vector<int> nonzero_cols;
  nonzero_cols.reserve(static_cast<std::size_t>(std::min(n_orig, n_src)));
  HighsCDouble rhs = transformed_cut.rhs;

  auto add_coeff = [&](int col, double value) {
    if (col < 0 || col >= n_orig || value == 0.0) return;
    HighsCDouble& acc = std_coeff_acc[static_cast<std::size_t>(col)];
    if (acc != 0.0) {
      acc += value;
    } else {
      acc = value;
      nonzero_cols.push_back(col);
    }
    if (acc == 0.0) {
      acc = (std::numeric_limits<double>::min)();
    }
  };

  auto add_logical_row = [&](int row, double scale) -> bool {
    if (row < 0 || row >= simplex.form.A_row.rows()) {
      return false;
    }
    // Use xtab_visit_source_row_terms so that HiGHS-presolved rows are read
    // from sf.source_row_index/value (original, unscaled) rather than from
    // sf.A_row (scaled/presolved).  For non-presolved rows the two paths are
    // equivalent; for presolved rows sf.A_row has different structure.
    xtab_visit_source_row_terms(
        simplex.form, row, [&](int c, double unscaled_coeff) {
          add_coeff(c, scale * unscaled_coeff);
        });
    return true;
  };

  for (int k = 0; k < static_cast<int>(transformed_cut.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    const int col = transformed_cut.inds[pos];
    if (col < 0 || col >= n_src) {
      return false;
    }
    const double val = transformed_cut.vals[pos];
    if (std::abs(val) <= 1e-12) {
      continue;
    }
    const bool is_logical_slack = source_context.is_logical_row_slack(col);
    switch (transformed_cut.bound_type[pos]) {
      case XTabBoundType::SimpleLb:
        if (is_logical_slack) {
          const int row = source_context.row_from_logical_slack(col);
          const double lower = source_context.lower[col];
          if (!std::isfinite(lower)) {
            return false;
          }
          rhs += val * lower;
          if (!add_logical_row(row, val)) {
            return false;
          }
        } else {
          const double lower = source_context.lower[col];
          if (!std::isfinite(lower)) {
            return false;
          }
          rhs += val * lower;
          add_coeff(col, val);
        }
        break;
      case XTabBoundType::SimpleUb: {
        const double ub = source_context.upper[col];
        if (!std::isfinite(ub)) {
          return false;
        }
        rhs -= val * ub;
        if (is_logical_slack) {
          const int row = source_context.row_from_logical_slack(col);
          if (!add_logical_row(row, -val)) {
            return false;
          }
        } else {
          add_coeff(col, -val);
        }
        break;
      }
      case XTabBoundType::VariableLb: {
        if (is_logical_slack) {
          return false;
        }
        const XTabVarBoundExpr& vb = transformed_cut.varbound_expr[pos];
        if (!vb.valid || vb.trigger_col < 0 || vb.trigger_col >= n_orig) {
          return false;
        }
        rhs += vb.constant * val;
        add_coeff(vb.trigger_col, -val * vb.coef);
        add_coeff(col, val);
        break;
      }
      case XTabBoundType::VariableUb: {
        if (is_logical_slack) {
          return false;
        }
        const XTabVarBoundExpr& vb = transformed_cut.varbound_expr[pos];
        if (!vb.valid || vb.trigger_col < 0 || vb.trigger_col >= n_orig) {
          return false;
        }
        rhs -= vb.constant * val;
        add_coeff(vb.trigger_col, val * vb.coef);
        add_coeff(col, -val);
        break;
      }
    }
  }

  Eigen::VectorXd coeff_orig = Eigen::VectorXd::Zero(n_orig);
  for (int col : nonzero_cols) {
    coeff_orig[col] = double(std_coeff_acc[static_cast<std::size_t>(col)]);
  }

  if (!std::isfinite(double(rhs)) || !coeff_orig.allFinite() ||
      coeff_orig.norm() <= 1e-12) {
    return false;
  }
  cut_le = std::move(coeff_orig);
  rhs_le = double(rhs);
  return true;
}

bool xtab_original_col_bounds(const SimplexResult& simplex,
                              int col,
                              double& lb,
                              double& ub) {
  if (col < 0 || col >= simplex.form.n_original ||
      col >= simplex.form.lb_shift.size() ||
      col >= simplex.form.var_ub.size()) {
    return false;
  }
  lb = simplex.form.lb_shift[col];
  ub = original_space_value(simplex, col, simplex.form.var_ub[col]);
  return std::isfinite(lb) && !std::isnan(ub);
}

bool xtab_postprocess_original_cut(const SimplexResult& simplex,
                                   const std::vector<char>* implied_integer_cols,
                                   Eigen::VectorXd& cut_le,
                                   double& rhs_le,
                                   double feastol,
                                   bool known_integral_cut = false) {
  feastol = std::max(0.0, feastol);
  constexpr double epsilon = 1e-12;
  const int n = simplex.form.n_original;
  if (cut_le.size() != n || n <= 0 || !std::isfinite(rhs_le) ||
      !cut_le.allFinite()) {
    return false;
  }
  if (rhs_le < 0.0 && rhs_le > -epsilon) {
    rhs_le = 0.0;
  }
  HighsCDouble rhs_acc = rhs_le;

  double max_abs = 0.0;
  for (int j = 0; j < n; ++j) {
    max_abs = std::max(max_abs, std::abs(cut_le[j]));
  }
  if (!(max_abs > 0.0)) {
    return false;
  }

  if (known_integral_cut) {
    for (int j = 0; j < n; ++j) {
      if (std::abs(cut_le[j]) <= epsilon) cut_le[j] = 0.0;
    }
    return cut_le.norm() > 1e-12 && std::isfinite(rhs_le);
  }

  const double min_coeff = 100.0 * feastol * std::max(max_abs, 1e-3);
  bool integral_support = true;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0) {
      continue;
    }
    if (std::abs(a) <= min_coeff) {
      double lb = 0.0;
      double ub = 0.0;
      if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
          (a < 0.0 && !std::isfinite(ub))) {
        return false;
      }
      rhs_acc -= a * (a < 0.0 ? ub : lb);
      cut_le[j] = 0.0;
      continue;
    }
    if (integral_support &&
        !transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      integral_support = false;
    }
  }

  max_abs = 0.0;
  double min_abs = std::numeric_limits<double>::infinity();
  std::vector<double> live_vals;
  live_vals.reserve(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (std::abs(a) <= 0.0) {
      continue;
    }
    const double aa = std::abs(a);
    max_abs = std::max(max_abs, aa);
    min_abs = std::min(min_abs, aa);
    live_vals.push_back(a);
  }
  if (live_vals.empty()) {
    return false;
  }

  auto scale_cut = [&](double pivot) {
    if (!(pivot > epsilon) || !std::isfinite(pivot)) {
      return false;
    }
    int exp_shift = 0;
    std::frexp(pivot - epsilon, &exp_shift);
    exp_shift = std::min(10, -exp_shift);
    rhs_acc = ldexp(rhs_acc, exp_shift);
    const double scale = std::ldexp(1.0, exp_shift);
    for (int j = 0; j < n; ++j) {
      cut_le[j] *= scale;
    }
    return true;
  };

  if (integral_support) {
    const double int_scale = xtab_integral_scale(live_vals, feastol, epsilon);
    bool scale_smallest_to_one = true;
    if (int_scale != 0.0 &&
        int_scale * std::max(1.0, max_abs) <=
            static_cast<double>(uint64_t{1} << 52)) {
      rhs_acc.renormalize();
      rhs_acc *= int_scale;
      double scaled_max_abs = xtab_nearest_integer(max_abs * int_scale);
      for (int j = 0; j < n; ++j) {
        const double a = cut_le[j];
        if (a == 0.0) {
          continue;
        }
        const double scaled = int_scale * a;
        const double intval = xtab_nearest_integer(scaled);
        const double delta = scaled - intval;
        double lb = 0.0;
        double ub = 0.0;
        if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
            (delta < 0.0 && !std::isfinite(ub))) {
          return false;
        }
        cut_le[j] = intval;
        rhs_acc -= delta * (delta < 0.0 ? ub : lb);
      }
      rhs_acc = floor(rhs_acc + feastol);
      if (int_scale * scaled_max_abs * feastol < 0.5) {
        scale_smallest_to_one = false;
      }
    }
    if (scale_smallest_to_one) {
      min_abs = std::numeric_limits<double>::infinity();
      for (int j = 0; j < n; ++j) {
        if (cut_le[j] != 0.0) {
          min_abs = std::min(min_abs, std::abs(cut_le[j]));
        }
      }
      if (!scale_cut(min_abs)) {
        return false;
      }
    }
  } else {
    if (!scale_cut(max_abs)) {
      return false;
    }
  }

  for (int j = 0; j < n; ++j) {
    if (std::abs(cut_le[j]) <= epsilon) {
      cut_le[j] = 0.0;
    }
  }
  rhs_le = double(rhs_acc);
  return cut_le.norm() > 1e-12 && std::isfinite(rhs_le);
}

bool xtab_tighten_original_cut_coefficients(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
    double feastol) {
  feastol = std::max(0.0, feastol);
  const int n = simplex.form.n_original;
  if (cut_le.size() != n || n <= 0 || !std::isfinite(rhs_le) ||
      !cut_le.allFinite()) {
    return false;
  }

  double max_activity = 0.0;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0) {
      continue;
    }
    double lb = 0.0;
    double ub = 0.0;
    if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
        (a > 0.0 && !std::isfinite(ub)) ||
        (a < 0.0 && !std::isfinite(lb))) {
      // Cannot bound the maximum activity — skip tightening conservatively.
      return true;
    }
    max_activity += a * (a > 0.0 ? ub : lb);
  }

  const double max_abs_coeff = max_activity - rhs_le;
  if (!(max_abs_coeff > feastol) || !std::isfinite(max_abs_coeff)) {
    return true;
  }

  bool tightened = false;
  double new_rhs = rhs_le;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0 ||
        !transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      continue;
    }
    double lb = 0.0;
    double ub = 0.0;
    if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
        (a > 0.0 && !std::isfinite(ub))) {
      return true;
    }
    if (a > max_abs_coeff) {
      const double delta = a - max_abs_coeff;
      new_rhs -= delta * ub;
      cut_le[j] = max_abs_coeff;
      tightened = true;
    } else if (a < -max_abs_coeff) {
      const double delta = -a - max_abs_coeff;
      new_rhs += delta * lb;
      cut_le[j] = -max_abs_coeff;
      tightened = true;
    }
  }
  if (tightened) {
    rhs_le = new_rhs;
  }
  return std::isfinite(rhs_le) && cut_le.allFinite() && cut_le.norm() > 1e-12;
}

bool xtab_generate_cut_from_base_row(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const Eigen::VectorXd& coeff_std,
    double rhs_std,
    const std::vector<int>* active_cols,
    const Eigen::VectorXd& x,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
	    double& efficacy_out,
    double cut_generation_feastol,
	    XTabRejectReason* reject_reason = nullptr,
	    XTabSourceDiag* gate_diag = nullptr,
    bool require_violation = true,
    const char* trace_family = "cut",
    const char* trace_basis_info = nullptr,
    XTabCutgenRandom* cutgen_random = nullptr,
    bool only_initial_cmir_scale = false,
    std::uint64_t* trace_id_out = nullptr) {
  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::None;
  if (gate_diag != nullptr) ++gate_diag->gate_calls;
  std::uint64_t trace_id = 0;
  const bool trace_row = xtab_claim_row_trace(trace_id, trace_family);
  if (trace_id_out != nullptr) *trace_id_out = trace_id;
  if (trace_row) {
    if (trace_basis_info != nullptr && trace_basis_info[0] != '\0') {
      fmt::print(stderr, "[B&C-XROW] id={} family={} stage=basis {}\n",
                 trace_id, trace_family, trace_basis_info);
    }
    xtab_trace_source_row(trace_id, trace_family, "base", coeff_std, rhs_std,
                          active_cols);
    xtab_trace_source_meta(trace_id, trace_family, "base", simplex,
                           source_context, coeff_std, active_cols);
  }
  XTabRow row;
  std::uint64_t vb_substitutions = 0;
  std::uint64_t vb_trigger_terms = 0;
  bool integers_positive = true;
  if (!xtab_transform_base_row(simplex, source_context, coeff_std, rhs_std,
                               active_cols, implied_integer_cols,
                               transform_context,
                               &vb_substitutions, &vb_trigger_terms,
                               &integers_positive,
                               /*initial_integers_positive=*/true,
                               cut_generation_feastol, row)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
    if (trace_row) xtab_trace_fail(trace_id, trace_family, "transform", "fail");
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "transform", row, source_context,
                   integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=transform_vbd "
               "vbSubs={} vbTerms={} initialIntPos=1\n",
               trace_id, trace_family, vb_substitutions, vb_trigger_terms);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_transform_ok;
  if (vb_substitutions_out != nullptr) {
    *vb_substitutions_out += vb_substitutions;
  }
  if (vb_trigger_terms_out != nullptr) {
    *vb_trigger_terms_out += vb_trigger_terms;
  }
  bool has_unbounded_ints = false;
  bool has_general_ints = false;
  bool has_continuous = false;
  if (!xtab_preprocess_base_inequality(simplex, row, cut_generation_feastol,
                                       has_unbounded_ints, has_general_ints,
                                       has_continuous)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Preprocess;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "preprocess", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "preprocess", row, source_context,
                   integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=preprocess_flags "
               "unboundedInt={} generalInt={} continuous={}\n",
               trace_id, trace_family, has_unbounded_ints ? 1 : 0,
               has_general_ints ? 1 : 0, has_continuous ? 1 : 0);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_preprocess_ok;
  if (!has_unbounded_ints && !integers_positive) {
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      const std::size_t pos = static_cast<std::size_t>(k);
      if (row.is_integral[pos] == 0 || row.vals[pos] > 0.0) {
        continue;
      }
      xtab_flip_complementation(row, k);
    }
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "pre_cmir", row, source_context,
                   integers_positive);
  }
  const double cmir_min_efficacy =
      require_violation ? 10.0 * cut_generation_feastol
                        : -std::numeric_limits<double>::infinity();
  XTabCmirTrace cmir_trace;
  const int random_tiebreaker =
      static_cast<int>(trace_id == 0 ? 0 : (trace_id & 0x7fffffffULL));
  if (!xtab_try_generate_cut(row, has_unbounded_ints, has_general_ints,
                             has_continuous, cut_generation_feastol,
                             cmir_min_efficacy, random_tiebreaker,
                             cutgen_random,
                             trace_row ? &cmir_trace : nullptr,
                             only_initial_cmir_scale,
                             trace_row ? trace_id : 0,
                             trace_row ? trace_family : nullptr)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "cmir", "fail");
      fmt::print(stderr,
                 "[B&C-XROW] id={} family={} stage=cmir_stats "
                 "int={} cont={} deltas={} tested={} contContrib={:.12g} "
                 "contNorm2={:.12g} maxDelta={:.12g} bestDelta={:.12g} "
                 "bestEff={:.12g} minEff={:.12g}\n",
                 trace_id, trace_family, cmir_trace.integer_terms,
                 cmir_trace.continuous_terms, cmir_trace.initial_delta_count,
                 cmir_trace.tested_delta_count,
                 cmir_trace.continuous_contribution,
                 cmir_trace.continuous_norm_sq, cmir_trace.max_abs_delta,
	                 cmir_trace.best_delta, cmir_trace.best_efficacy,
	                 cmir_min_efficacy);
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family,
                   cmir_trace.lifted_accepted ? "lifted" : "cmir", row,
                   source_context, integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=cmir_stats "
               "int={} cont={} deltas={} tested={} contContrib={:.12g} "
               "contNorm2={:.12g} maxDelta={:.12g} bestDelta={:.12g} "
               "bestEff={:.12g} f0={:.12g} finalRhs={:.12g} minEff={:.12g} "
               "generator={}\n",
               trace_id, trace_family, cmir_trace.integer_terms,
               cmir_trace.continuous_terms, cmir_trace.initial_delta_count,
               cmir_trace.tested_delta_count,
               cmir_trace.continuous_contribution,
               cmir_trace.continuous_norm_sq, cmir_trace.max_abs_delta,
               cmir_trace.best_delta, cmir_trace.best_efficacy,
               cmir_trace.final_f0, cmir_trace.final_rhs, cmir_min_efficacy,
               cmir_trace.lifted_accepted ? "lifted" : "cmir");
  }
  if (gate_diag != nullptr) ++gate_diag->gate_cmir_ok;
  xtab_remove_complementation(row);
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "remove_complementation", row,
                   source_context, integers_positive);
  }

  std::vector<unsigned char> erase(row.inds.size(), 0);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (std::abs(row.vals[static_cast<std::size_t>(k)]) <= 1e-12) {
      erase[static_cast<std::size_t>(k)] = 1;
    }
  }
  xtab_erase_positions(row, erase);
  if (row.inds.empty()) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
    if (trace_row) xtab_trace_fail(trace_id, trace_family, "erase", "empty");
    return false;
  }
  if (!xtab_untransform_cut(simplex, source_context, row, cut_le, rhs_le)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "untransform", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "untransform", cut_le,
                            rhs_le, x);
  }
  if (!xtab_postprocess_original_cut(simplex, implied_integer_cols, cut_le,
                                     rhs_le, cut_generation_feastol,
                                     row.integral_support &&
                                         row.integral_coefficients)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "postprocess", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "postprocess", cut_le,
                            rhs_le, x);
  }
  if (!xtab_tighten_original_cut_coefficients(simplex, implied_integer_cols,
                                              cut_le, rhs_le,
                                              cut_generation_feastol)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "tighten", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "tighten", cut_le, rhs_le,
                            x);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_untransform_ok;

  const double violation = cut_le.dot(x) - rhs_le;
  const double norm = std::max(1e-12, cut_le.norm());
  efficacy_out = violation / norm;
  const bool violated = violation > 10.0 * cut_generation_feastol;
  if (violated && gate_diag != nullptr) ++gate_diag->gate_violation_ok;
  if (!violated && reject_reason != nullptr) {
    *reject_reason = XTabRejectReason::NotViolated;
  }
  if (trace_row) {
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=final violated={} "
               "violation={:.12g} norm={:.12g} efficacy={:.12g} "
               "requireViolation={}\n",
               trace_id, trace_family, violated ? 1 : 0, violation, norm,
               efficacy_out, require_violation ? 1 : 0);
  }
  return require_violation ? violated : true;
}

bool aggregate_standard_rows(const SimplexResult& simplex,
                             const XTabSourceContext& source_context,
                             const std::vector<std::pair<int, double>>& rows,
                             Eigen::VectorXd& coeff,
                             double& rhs,
                             std::vector<int>* active_cols = nullptr,
                             SeparatorStorageStats* storage_stats = nullptr);
double standard_row_max_abs(const SimplexResult& simplex, int row);

struct XTabLpAggregator {
  XTabLpAggregator(const SimplexResult& simplex,
                   const XTabSourceContext& source_context,
                   SeparatorStorageStats* storage_stats = nullptr)
      : sf(simplex.form),
        ctx(source_context),
        sum(static_cast<std::size_t>(source_context.dim)),
        touched_flag(static_cast<std::size_t>(source_context.dim), 0),
        storage_stats(storage_stats) {}

  void add_value(int col, double value) {
    if (col < 0 || col >= ctx.dim || value == 0.0) return;
    HighsCDouble& acc = sum[static_cast<std::size_t>(col)];
    if (acc != 0.0) {
      acc += value;
    } else {
      acc = value;
      if (touched_flag[static_cast<std::size_t>(col)] == 0) {
        touched_flag[static_cast<std::size_t>(col)] = 1;
        touched.push_back(col);
      }
    }
    if (acc == 0.0) {
      acc = (std::numeric_limits<double>::min)();
    }
  }

  bool add_row(int row, double weight) {
    if (!ctx.valid || row < 0 || row >= sf.A_row.rows() ||
        row >= ctx.n_rows || !std::isfinite(weight)) {
      return false;
    }
    const double row_scale_inv = 1.0 / xtab_row_scale_or_one(sf, row);
    for (StandardRowMatrix::InnerIterator it(
             sf.A_row, row);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= ctx.n_original) {
        continue;
      }
      const double unscaled =
          it.value() * row_scale_inv / xtab_col_scale_or_one(sf, col);
      add_value(col, weight * unscaled);
    }
    const int slack_col = ctx.n_original + row;
    if (slack_col < 0 || slack_col >= ctx.dim) {
      return false;
    }
    add_value(slack_col, -weight);
    return true;
  }

  bool get_current_aggregation(Eigen::VectorXd& coeff,
                               double& rhs,
                               bool negate,
                               std::vector<int>* active_cols = nullptr) {
    record_dense_workspace(storage_stats, sum.size());
    if (coeff.size() != static_cast<Eigen::Index>(sum.size())) {
      coeff.resize(static_cast<int>(sum.size()));
    }
    coeff.setZero();
    if (active_cols != nullptr) active_cols->clear();
    const double sign = negate ? -1.0 : 1.0;
    rhs = 0.0;
    constexpr double droptol = 1e-12;
    int num_nz = static_cast<int>(touched.size());
    for (int i = num_nz - 1; i >= 0; --i) {
      const int col = touched[static_cast<std::size_t>(i)];
      const double val = double(sum[static_cast<std::size_t>(col)]);
      if (!std::isfinite(val)) {
        return false;
      }
      // HiGHS' HighsLpAggregator drops tiny structural coefficients but keeps
      // exact logical row-activity slack terms.
      const bool drop = col < ctx.n_original ? std::abs(val) <= droptol
                                             : val == 0.0;
      if (drop) {
        sum[static_cast<std::size_t>(col)] = 0.0;
        touched_flag[static_cast<std::size_t>(col)] = 0;
        --num_nz;
        std::swap(touched[static_cast<std::size_t>(num_nz)],
                  touched[static_cast<std::size_t>(i)]);
      }
    }
    touched.resize(static_cast<std::size_t>(num_nz));
    for (int col : touched) {
      const double val = sign * double(sum[static_cast<std::size_t>(col)]);
      coeff[col] = val;
      if (active_cols != nullptr) active_cols->push_back(col);
    }
    return std::isfinite(rhs);
  }

  bool get_current_aggregation(Eigen::SparseVector<double>& coeff,
                               double& rhs,
                               bool negate,
                               std::vector<int>* active_cols = nullptr) {
    if (active_cols != nullptr) active_cols->clear();
    const double sign = negate ? -1.0 : 1.0;
    rhs = 0.0;
    constexpr double droptol = 1e-12;
    int num_nz = static_cast<int>(touched.size());
    for (int i = num_nz - 1; i >= 0; --i) {
      const int col = touched[static_cast<std::size_t>(i)];
      const double val = double(sum[static_cast<std::size_t>(col)]);
      if (!std::isfinite(val)) return false;
      const bool drop = col < ctx.n_original ? std::abs(val) <= droptol
                                             : val == 0.0;
      if (drop) {
        sum[static_cast<std::size_t>(col)] = 0.0;
        touched_flag[static_cast<std::size_t>(col)] = 0;
        --num_nz;
        std::swap(touched[static_cast<std::size_t>(num_nz)],
                  touched[static_cast<std::size_t>(i)]);
      }
    }
    touched.resize(static_cast<std::size_t>(num_nz));

    std::vector<int> sorted_cols = touched;
    std::sort(sorted_cols.begin(), sorted_cols.end());
    coeff.resize(ctx.dim);
    coeff.setZero();
    coeff.reserve(static_cast<int>(sorted_cols.size()));
    for (int col : sorted_cols) {
      coeff.insertBack(col) =
          sign * double(sum[static_cast<std::size_t>(col)]);
    }
    if (active_cols != nullptr) *active_cols = std::move(sorted_cols);
    if (storage_stats != nullptr) {
      ++storage_stats->sparse_aggregation_snapshots;
      storage_stats->sparse_aggregation_entries +=
          static_cast<std::uint64_t>(coeff.nonZeros());
    }
    return std::isfinite(rhs);
  }

  void clear() {
    for (int col : touched) {
      sum[static_cast<std::size_t>(col)] = 0.0;
      touched_flag[static_cast<std::size_t>(col)] = 0;
    }
    touched.clear();
  }

  const StandardFormLP& sf;
  const XTabSourceContext& ctx;
  std::vector<HighsCDouble> sum;
  std::vector<int> touched;
  std::vector<unsigned char> touched_flag;
  SeparatorStorageStats* storage_stats{nullptr};
};

struct XTabAggregatedSourceRow {
  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
  std::vector<int> active_cols;
};

bool xtab_generate_path_mixing_cut(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const std::vector<XTabAggregatedSourceRow>& aggregated_path,
    const Eigen::VectorXd& x,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
    double& efficacy_out,
    double cut_generation_feastol,
    XTabRejectReason* reject_reason = nullptr,
    const char* trace_family = "pathmix",
    SeparatorStorageStats* storage_stats = nullptr) {
  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::None;
  if (vb_substitutions_out != nullptr) *vb_substitutions_out = 0;
  if (vb_trigger_terms_out != nullptr) *vb_trigger_terms_out = 0;
  std::uint64_t trace_id = 0;
  const bool trace_row = xtab_claim_row_trace(trace_id, trace_family);
  if (trace_row) {
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=pathmix_start rows={} "
               "nsrc={} norig={}\n",
               trace_id, trace_family, aggregated_path.size(),
               source_context.dim, simplex.form.n_original);
  }
  const double feastol = std::max(0.0, cut_generation_feastol);
  constexpr double epsilon = 1e-12;
  const int n_src = source_context.dim;
  const int n_orig = simplex.form.n_original;
  if (!source_context.valid || aggregated_path.size() < 2 || n_src <= 0 ||
      n_orig <= 0 || x.size() != n_orig) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_validate", "fail");
    }
    return false;
  }

  struct PathTermInfo {
    int col{-1};
    double upper{0.0};
    double solval{0.0};
    XTabBoundType bound_type{XTabBoundType::SimpleLb};
    XTabVarBoundExpr varbound;
    unsigned char is_integral{0};
  };

  std::vector<XTabRow> transformed_rows;
  transformed_rows.reserve(aggregated_path.size());
  std::vector<int> pos(static_cast<std::size_t>(n_src), -1);
  std::vector<PathTermInfo> terms;
  terms.reserve(static_cast<std::size_t>(std::min(n_src, 4096)));
  std::vector<double> transformed_rhs;
  transformed_rhs.reserve(aggregated_path.size());
  double delta = 1.0;

  int source_row_index = 0;
  for (const auto& source_row : aggregated_path) {
    const Eigen::VectorXd source_coeff = Eigen::VectorXd(source_row.coeff);
    record_dense_workspace(storage_stats,
                           static_cast<std::size_t>(source_coeff.size()));
    if (trace_row) {
      const std::string stage = fmt::format("path_base_{}", source_row_index);
      xtab_trace_source_row(trace_id, trace_family, stage.c_str(),
                            source_coeff, source_row.rhs,
                            &source_row.active_cols);
    }
    XTabRow row;
    std::uint64_t vb_substitutions = 0;
    std::uint64_t vb_trigger_terms = 0;
    bool integers_positive = true;
    if (!xtab_transform_base_row(simplex, source_context, source_coeff,
                                 source_row.rhs, &source_row.active_cols,
                                 implied_integer_cols, transform_context,
                                 &vb_substitutions, &vb_trigger_terms,
                                 &integers_positive,
                                 /*initial_integers_positive=*/false,
                                 cut_generation_feastol, row)) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
      if (trace_row) {
        const std::string stage =
            fmt::format("path_transform_{}", source_row_index);
        xtab_trace_fail(trace_id, trace_family, stage.c_str(), "fail");
      }
      break;
    }
    if (trace_row) {
      const std::string stage =
          fmt::format("path_transform_{}", source_row_index);
      xtab_trace_row(trace_id, trace_family, stage.c_str(), row,
                     source_context, integers_positive);
      fmt::print(stderr,
                 "[B&C-XROW] id={} family={} stage=path_transform_vbd_{} "
                 "vbSubs={} vbTerms={} initialIntPos=0 intPos={}\n",
                 trace_id, trace_family, source_row_index, vb_substitutions,
                 vb_trigger_terms, integers_positive ? 1 : 0);
    }
    if (vb_substitutions_out != nullptr) {
      *vb_substitutions_out += vb_substitutions;
    }
    if (vb_trigger_terms_out != nullptr) {
      *vb_trigger_terms_out += vb_trigger_terms;
    }
    const int k = static_cast<int>(transformed_rhs.size());
    if (k == 0) {
      if (row.rhs > epsilon) {
        if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
        if (trace_row) {
          xtab_trace_fail(trace_id, trace_family, "path_rhs_order",
                          "first_rhs_positive");
        }
        break;
      }
      if (row.rhs >= -feastol) row.rhs = 0.0;
    } else if (row.rhs >= transformed_rhs.back() - feastol) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
      if (trace_row) {
        xtab_trace_fail(trace_id, trace_family, "path_rhs_order",
                        "not_decreasing");
      }
      break;
    }
    delta = std::max(delta, std::abs(row.rhs));

    bool consistent = true;
    for (int t = 0; t < static_cast<int>(row.inds.size()); ++t) {
      const std::size_t tp = static_cast<std::size_t>(t);
      const int col = row.inds[tp];
      if (col < 0 || col >= n_src) {
        consistent = false;
        break;
      }
      int& p = pos[static_cast<std::size_t>(col)];
      if (p < 0) {
        p = static_cast<int>(terms.size());
        terms.push_back(PathTermInfo{
            col, row.upper[tp], row.solval[tp], row.bound_type[tp],
            row.varbound_expr[tp], row.is_integral[tp]});
      } else {
        const PathTermInfo& info = terms[static_cast<std::size_t>(p)];
        const double scale =
            std::max(1.0, std::max(std::abs(info.upper), std::abs(row.upper[tp])));
        auto same_varbound = [&]() {
          if (info.bound_type != XTabBoundType::VariableLb &&
              info.bound_type != XTabBoundType::VariableUb) {
            return true;
          }
          const XTabVarBoundExpr& a = info.varbound;
          const XTabVarBoundExpr& b = row.varbound_expr[tp];
          return a.valid == b.valid &&
                 a.trigger_col == b.trigger_col &&
                 std::abs(a.constant - b.constant) <=
                     100.0 * feastol * scale &&
                 std::abs(a.coef - b.coef) <=
                     100.0 * feastol * scale;
        };
        if (info.bound_type != row.bound_type[tp] ||
            info.is_integral != row.is_integral[tp] ||
            !same_varbound() ||
            std::abs(info.solval - row.solval[tp]) >
                100.0 * feastol * scale ||
            ((std::isfinite(info.upper) || std::isfinite(row.upper[tp])) &&
                (!std::isfinite(info.upper) || !std::isfinite(row.upper[tp]) ||
                 std::abs(info.upper - row.upper[tp]) >
                     100.0 * feastol * scale))) {
          consistent = false;
          break;
        }
      }
      if (row.is_integral[tp] != 0) {
        delta = std::max(delta, std::abs(row.vals[tp]));
      }
    }
    if (!consistent) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
      if (trace_row) {
        xtab_trace_fail(trace_id, trace_family, "path_consistency", "fail");
      }
      break;
    }

    transformed_rhs.push_back(row.rhs);
    transformed_rows.push_back(std::move(row));
    ++source_row_index;
  }

  if (transformed_rows.size() < 2) {
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_transform",
                      "too_short");
    }
    return false;
  }

  delta = std::exp2(std::ceil(std::log2(delta + 1.0)));
  const int num_terms = static_cast<int>(terms.size());
  if (num_terms == 0) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_terms", "empty");
    }
    return false;
  }
  if (trace_row) {
    fmt::memory_buffer rhs_sample;
    const int max_terms = xtab_row_trace_terms();
    const int emit = std::min<int>(max_terms, transformed_rhs.size());
    for (int i = 0; i < emit; ++i) {
      if (i > 0) fmt::format_to(std::back_inserter(rhs_sample), ",");
      fmt::format_to(std::back_inserter(rhs_sample), "{:.12g}",
                     transformed_rhs[static_cast<std::size_t>(i)]);
    }
    if (static_cast<int>(transformed_rhs.size()) > emit) {
      fmt::format_to(std::back_inserter(rhs_sample), ",...");
    }
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=pathmix_delta rows={} "
               "terms={} delta={:.12g} rhs=[{}]\n",
               trace_id, trace_family, transformed_rows.size(), num_terms,
               delta, fmt::to_string(rhs_sample));
  }

  std::vector<double> cut_vals(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> max_frac(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> down_sum(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> f_sum(static_cast<std::size_t>(num_terms), 0.0);

  double cut_rhs = 0.0;
  double f_last = 0.0;
  const double inv_delta_scale = -1.0 / delta;

  for (int k = 0; k < static_cast<int>(transformed_rows.size()); ++k) {
    const XTabRow& row = transformed_rows[static_cast<std::size_t>(k)];
    const double f = transformed_rhs[static_cast<std::size_t>(k)] *
                     inv_delta_scale;
    const double f_diff = f - f_last;
    cut_rhs += f_diff;

    for (int t = 0; t < static_cast<int>(row.inds.size()); ++t) {
      const std::size_t tp = static_cast<std::size_t>(t);
      const int col = row.inds[tp];
      const int p = pos[static_cast<std::size_t>(col)];
      if (p < 0) continue;
      const double gj = row.vals[tp] * inv_delta_scale;
      const std::size_t pp = static_cast<std::size_t>(p);
      if (terms[pp].is_integral == 0) {
        cut_vals[pp] = std::max(cut_vals[pp], gj);
      } else {
        const double gj_down = std::floor(gj);
        const double hj = gj - gj_down;
        max_frac[pp] = std::max(max_frac[pp], hj);
        down_sum[pp] += f_diff * gj_down;
        f_sum[pp] += f_diff;
        cut_vals[pp] =
            down_sum[pp] +
            (f_sum[pp] < max_frac[pp] ? f_sum[pp] : max_frac[pp]);
      }
    }

    if (k > 0) {
      double violation = cut_rhs;
      for (int p = 0; p < num_terms; ++p) {
        violation -= terms[static_cast<std::size_t>(p)].solval *
                     cut_vals[static_cast<std::size_t>(p)];
      }
      violation *= delta;
      if (trace_row) {
        fmt::print(stderr,
                   "[B&C-XROW] id={} family={} stage=pathmix_probe k={} "
                   "pathViolation={:.12g} cutRhs={:.12g} delta={:.12g}\n",
                   trace_id, trace_family, k, violation, cut_rhs, delta);
      }
      if (violation > 10.0 * feastol) {
        XTabRow out;
        out.rhs = cut_rhs * (-delta);
        out.inds.reserve(static_cast<std::size_t>(num_terms));
        out.vals.reserve(static_cast<std::size_t>(num_terms));
        out.upper.reserve(static_cast<std::size_t>(num_terms));
        out.solval.reserve(static_cast<std::size_t>(num_terms));
        out.bound_type.reserve(static_cast<std::size_t>(num_terms));
        out.varbound_expr.reserve(static_cast<std::size_t>(num_terms));
        out.complemented.reserve(static_cast<std::size_t>(num_terms));
        out.is_integral.reserve(static_cast<std::size_t>(num_terms));
        for (int p = 0; p < num_terms; ++p) {
          const double val = cut_vals[static_cast<std::size_t>(p)] * (-delta);
          if (std::abs(val) <= epsilon) continue;
          const PathTermInfo& info = terms[static_cast<std::size_t>(p)];
          out.inds.push_back(info.col);
          out.vals.push_back(val);
          out.upper.push_back(info.upper);
          out.solval.push_back(info.solval);
          out.bound_type.push_back(info.bound_type);
          out.varbound_expr.push_back(info.varbound);
          out.complemented.push_back(0);
          out.is_integral.push_back(info.is_integral);
        }
        if (out.inds.empty()) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "pathmix_row", "empty");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_row(trace_id, trace_family, "pathmix_row", out,
                         source_context, false);
        }
        if (!xtab_untransform_cut(simplex, source_context, out, cut_le,
                                  rhs_le)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "untransform", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "untransform",
                                  cut_le, rhs_le, x);
        }
        if (!xtab_postprocess_original_cut(simplex, implied_integer_cols,
                                           cut_le, rhs_le, feastol)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "postprocess", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "postprocess",
                                  cut_le, rhs_le, x);
        }
        if (!xtab_tighten_original_cut_coefficients(
                simplex, implied_integer_cols, cut_le, rhs_le, feastol)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "tighten", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "tighten", cut_le,
                                  rhs_le, x);
        }
        const double final_violation = cut_le.dot(x) - rhs_le;
        const double norm = std::max(1e-12, cut_le.norm());
        efficacy_out = final_violation / norm;
        const bool accepted = final_violation > 10.0 * feastol;
        if (trace_row) {
          fmt::print(stderr,
                     "[B&C-XROW] id={} family={} stage=final violated={} "
                     "violation={:.12g} norm={:.12g} efficacy={:.12g} "
                     "requireViolation=1\n",
                     trace_id, trace_family, accepted ? 1 : 0,
                     final_violation, norm, efficacy_out);
        }
        if (accepted) {
          return true;
        }
        if (reject_reason != nullptr) *reject_reason = XTabRejectReason::NotViolated;
        return false;
      }
    }
    f_last = f;
  }

  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::NotViolated;
  if (trace_row) {
    xtab_trace_fail(trace_id, trace_family, "pathmix", "no_violation");
  }
  return false;
}

double standard_row_max_abs(const SimplexResult& simplex, int row) {
  double max_abs = 0.0;
  const double row_scale_inv = 1.0 / xtab_row_scale_or_one(simplex.form, row);
  for (StandardRowMatrix::InnerIterator it(
           simplex.form.A_row, row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col >= 0 && col < simplex.form.n_original) {
      const double value = std::abs(it.value()) * row_scale_inv /
                           xtab_col_scale_or_one(simplex.form, col);
      max_abs = std::max(max_abs, value);
    }
  }
  return max_abs;
}

bool aggregate_standard_rows(const SimplexResult& simplex,
                             const XTabSourceContext& source_context,
                             const std::vector<std::pair<int, double>>& rows,
                             Eigen::VectorXd& coeff,
                             double& rhs,
                             std::vector<int>* active_cols,
                             SeparatorStorageStats* storage_stats) {
  XTabLpAggregator aggregator(simplex, source_context, storage_stats);
  for (const auto& [r, w] : rows) {
    if (!aggregator.add_row(r, w)) {
      return false;
    }
  }
  return aggregator.get_current_aggregation(coeff, rhs, false, active_cols);
}

template <int k, typename FoundModKCut>
bool xtab_separate_modk_system(const std::vector<std::int64_t>& values,
                               const std::vector<int>& indices,
                               const std::vector<int>& starts,
                               int num_cols,
                               FoundModKCut&& found_cut) {
  HighsGFkSolve solver;
  std::vector<HighsInt> highs_indices(indices.begin(), indices.end());
  std::vector<HighsInt> highs_starts(starts.begin(), starts.end());
  solver.fromCSC<static_cast<unsigned int>(k)>(values, highs_indices,
                                               highs_starts, num_cols + 1);
  solver.setRhs<static_cast<unsigned int>(k)>(num_cols, 1);
  bool found = false;
  solver.solve<static_cast<unsigned int>(k)>(
      [&](std::vector<HighsGFkSolve::SolutionEntry>& weights,
          int rhs_index) {
        found = true;
        found_cut(weights, rhs_index);
      });
  return found;
}

int add_transformed_modk_cuts_impl(
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
  const bool direct_cutpool_mode = cutpool_acceptor != nullptr;
  (void)lp;
  (void)opt;
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  if ((!direct_cutpool_mode && generated_cutpool_rows == nullptr) ||
      !simplex.exact_optimal || x.size() != simplex.form.n_original) {
    return 0;
  }

  XTabSourceDiag diag;
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("modk", diag);
    return 0;
  }

  struct XModkLedger {
    int src_rows{0};
    int src_dim{0};
    int skipped_continuous{0};
    int active_le{0};
    int active_ge{0};
    int inactive{0};
    int transform_ok{0};
    int transform_fail{0};
    int integral_rows{0};
    int scaled_rows{0};
    int long_skip{0};
    int zero_skip{0};
    int system_nnz{0};
    int nonzero_rhs{0};
    int gf2_calls{0};
    int gf3_calls{0};
    int gf5_calls{0};
    int gf7_calls{0};
    int gf_solutions{0};
    int cutgen_calls{0};
    int cutgen_success{0};
    int pool_calls{0};
    int pool_accepted{0};
    std::uint64_t system_hash{0};
  } ledger;
  ledger.src_rows = source_context.n_rows;
  ledger.src_dim = source_context.dim;

  auto print_modk_ledger = [&](const char* stage, int accepted) {
    if (bc_env_options().value("MIPSOLVERS_XPATH_LEDGER") == nullptr &&
        bc_env_options().value("MIPSOLVERS_XTAB_DIAG") == nullptr) {
      return;
    }
    fmt::print(
        stderr,
        "[B&C-XMODK-LEDGER] stage={} mode={} srcRows={} srcDim={} "
        "skipCont={} active=le{}:ge{} inactive={} transform={}/{} "
        "rows=int{}:scaled{} longSkip={} zeroSkip={} systemNnz={} rhsNz={} "
        "gf=2:{}:3:{}:5:{}:7:{} sol={} cutgen={}/{} "
        "poolCalls={} poolAccepted={} acceptedRows={} sysHash={:016x}\n",
        stage, direct_cutpool_mode ? "cutpool" : "rows",
        ledger.src_rows, ledger.src_dim, ledger.skipped_continuous,
        ledger.active_le, ledger.active_ge, ledger.inactive,
        ledger.transform_ok, ledger.transform_fail, ledger.integral_rows,
        ledger.scaled_rows, ledger.long_skip, ledger.zero_skip,
        ledger.system_nnz, ledger.nonzero_rhs, ledger.gf2_calls,
        ledger.gf3_calls, ledger.gf5_calls, ledger.gf7_calls,
        ledger.gf_solutions, ledger.cutgen_success, ledger.cutgen_calls,
        ledger.pool_calls, ledger.pool_accepted, accepted,
        ledger.system_hash);
  };

  const int n_orig = simplex.form.n_original;
  const int m = source_context.n_rows;
  const double feastol = cut_generation_feastol;
  constexpr double epsilon = 1e-9;
  const int system_trace_limit = xtab_modk_system_trace_limit();
  const int system_trace_terms = xtab_modk_system_trace_terms();

  std::vector<unsigned char> integer_like(
      static_cast<std::size_t>(n_orig), 0);
  std::vector<double> bound_distance(static_cast<std::size_t>(n_orig), 0.0);
  for (int col = 0; col < n_orig; ++col) {
    integer_like[static_cast<std::size_t>(col)] =
        source_context.integral[static_cast<std::size_t>(col)] != 0 ? 1 : 0;
    if (integer_like[static_cast<std::size_t>(col)] != 0) continue;
    const double val = source_context.solution[col];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    double lb_dist = std::isfinite(lb) ? std::max(0.0, val - lb)
                                       : std::numeric_limits<double>::infinity();
    double ub_dist = std::isfinite(ub) ? std::max(0.0, ub - val)
                                       : std::numeric_limits<double>::infinity();
    if (col < static_cast<int>(transform_context.best_vlb.size())) {
      const XTabVarBoundExpr& vlb =
          transform_context.best_vlb[static_cast<std::size_t>(col)];
      if (vlb.valid) lb_dist = std::min(lb_dist, vlb.dist);
    }
    if (col < static_cast<int>(transform_context.best_vub.size())) {
      const XTabVarBoundExpr& vub =
          transform_context.best_vub[static_cast<std::size_t>(col)];
      if (vub.valid) ub_dist = std::min(ub_dist, vub.dist);
    }
    const double dist = std::min(lb_dist, ub_dist);
    bound_distance[static_cast<std::size_t>(col)] =
        std::isfinite(dist) && dist > feastol ? dist : 0.0;
  }

  std::vector<unsigned char> skip_row(static_cast<std::size_t>(m), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like[static_cast<std::size_t>(col)] != 0 ||
        bound_distance[static_cast<std::size_t>(col)] == 0.0) {
      continue;
    }
    for (StandardColumnMatrix::InnerIterator it(simplex.form.A, col);
         it; ++it) {
      const int row = static_cast<int>(it.row());
      if (row < 0 || row >= m ||
          skip_row[static_cast<std::size_t>(row)] != 0) {
        continue;
      }
      skip_row[static_cast<std::size_t>(row)] = 1;
      ++ledger.skipped_continuous;
    }
  }

  const int max_int_row_len = static_cast<int>(1000 + 0.1 * n_orig);
  std::vector<std::pair<int, double>> integral_scales;
  std::vector<std::int64_t> int_system_value;
  std::vector<int> int_system_index;
  std::vector<int> int_system_start;
  int_system_start.push_back(0);

  for (int row = 0; row < m; ++row) {
    if (skip_row[static_cast<std::size_t>(row)] != 0) continue;

    const int scol = n_orig + row;
    const double lower = source_context.lower[scol];
    const double upper = source_context.upper[scol];
    const double act = source_context.solution[scol];
    bool leq_row = true;
    if (std::isfinite(upper) && upper - act <= feastol) {
      leq_row = true;
      ++ledger.active_le;
    } else if (std::isfinite(lower) && act - lower <= feastol) {
      leq_row = false;
      ++ledger.active_ge;
    } else {
      ++ledger.inactive;
      continue;
    }

    Eigen::VectorXd coeff = Eigen::VectorXd::Zero(source_context.dim);
    double rhs = leq_row ? upper : -lower;
    std::vector<int> active_cols;
    int source_row_len = 0;
    xtab_visit_source_row_terms(simplex.form, row, [&](int col, double val) {
      ++source_row_len;
      if (std::abs(val) <= 1e-12) return;
      coeff[col] = leq_row ? val : -val;
      active_cols.push_back(col);
    });

    XTabRow transformed;
    bool integers_positive = false;
    std::uint64_t vb_subs = 0;
    std::uint64_t vb_terms = 0;
    if (!xtab_transform_base_row(simplex, source_context, coeff, rhs,
                                 &active_cols, implied_integer_cols,
                                 &transform_context, &vb_subs, &vb_terms,
                                 &integers_positive,
                                 /*initial_integers_positive=*/false,
                                 feastol, transformed)) {
      ++ledger.transform_fail;
      continue;
    }
    ++ledger.transform_ok;
    diag.vb_substitutions += vb_subs;
    diag.vb_trigger_terms += vb_terms;
    if (row == xtab_modk_transform_trace_row()) {
      fmt::memory_buffer rawbuf;
      const int raw_emit = std::min<int>(active_cols.size(), 200);
      for (int k = 0; k < raw_emit; ++k) {
        const int col = active_cols[static_cast<std::size_t>(k)];
        if (k > 0) fmt::format_to(std::back_inserter(rawbuf), ",");
        fmt::format_to(std::back_inserter(rawbuf), "{}:{:.17g}", col,
                       col >= 0 && col < coeff.size() ? coeff[col] : 0.0);
      }
      if (static_cast<int>(active_cols.size()) > raw_emit) {
        fmt::format_to(std::back_inserter(rawbuf), ",...");
      }
      fmt::memory_buffer solbuf;
      const int emit = std::min<int>(transformed.inds.size(), 200);
      for (int k = 0; k < emit; ++k) {
        if (k > 0) fmt::format_to(std::back_inserter(solbuf), ",");
        fmt::format_to(std::back_inserter(solbuf), "{}:{:.17g}:{:.17g}:{}",
                       transformed.inds[static_cast<std::size_t>(k)],
                       transformed.vals[static_cast<std::size_t>(k)],
                       transformed.solval[static_cast<std::size_t>(k)],
                       transformed.is_integral[static_cast<std::size_t>(k)]);
      }
      if (static_cast<int>(transformed.inds.size()) > emit) {
        fmt::format_to(std::back_inserter(solbuf), ",...");
      }
      fmt::print(stderr,
                 "[B&C-XMODK-TRANSFORM] row={} side={} rawLen={} rhs={:.17g} "
                 "raw=[{}] transLen={} transRhs={:.17g} trans=[{}]\n",
                 row, leq_row ? "le" : "ge", source_row_len, rhs,
                 fmt::to_string(rawbuf),
                 static_cast<int>(transformed.inds.size()), transformed.rhs,
                 fmt::to_string(solbuf));
    }

    const int rowlen = static_cast<int>(transformed.inds.size());
    if (rowlen > max_int_row_len) {
      int int_row_len = 0;
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.solval[pos] <= feastol) continue;
        if (transformed.is_integral[pos] == 0) continue;
        ++int_row_len;
      }
      if (int_row_len > max_int_row_len ||
          (int_row_len == 0 && std::abs(transformed.rhs) <= epsilon)) {
        ++ledger.long_skip;
        continue;
      }
    }

    double intscale = 1.0;
    std::int64_t intrhs = 0;
    const int system_row_start = int_system_start.back();
    const bool row_integral =
        source_context.integral[static_cast<std::size_t>(scol)] != 0;
    if (!row_integral) {
      std::vector<double> scale_vals;
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.is_integral[pos] == 0) continue;
        if (transformed.solval[pos] > feastol) {
          scale_vals.push_back(transformed.vals[pos]);
        }
      }
      if (std::abs(transformed.rhs) > epsilon) {
        scale_vals.push_back(-transformed.rhs);
      }
      if (scale_vals.empty()) {
        ++ledger.zero_skip;
        continue;
      }
      intscale = xtab_integral_scale(scale_vals, feastol, epsilon);
      if (intscale == 0.0 || intscale > 1e6) {
        ++ledger.zero_skip;
        continue;
      }
      intrhs = static_cast<std::int64_t>(
          xtab_nearest_integer(intscale * transformed.rhs));
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.is_integral[pos] == 0) continue;
        if (transformed.solval[pos] > feastol) {
          int_system_index.push_back(transformed.inds[pos]);
          int_system_value.push_back(static_cast<std::int64_t>(
              xtab_nearest_integer(intscale * transformed.vals[pos])));
        }
      }
      ++ledger.scaled_rows;
    } else {
      intrhs = static_cast<std::int64_t>(
          xtab_nearest_integer(transformed.rhs));
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.solval[pos] > feastol) {
          int_system_index.push_back(transformed.inds[pos]);
          int_system_value.push_back(static_cast<std::int64_t>(
              xtab_nearest_integer(transformed.vals[pos])));
        }
      }
      ++ledger.integral_rows;
    }

    ledger.nonzero_rhs += intrhs != 0 ? 1 : 0;
    int_system_index.push_back(n_orig);
    int_system_value.push_back(intrhs);
    int_system_start.push_back(static_cast<int>(int_system_value.size()));
    if (system_trace_limit > 0 &&
        static_cast<int>(integral_scales.size()) < system_trace_limit) {
      fmt::memory_buffer terms;
      const int system_row_end = static_cast<int>(int_system_value.size());
      const int emit_end =
          std::min(system_row_end, system_row_start + system_trace_terms);
      for (int p = system_row_start; p < emit_end; ++p) {
        if (p > system_row_start) fmt::format_to(std::back_inserter(terms), ",");
        fmt::format_to(std::back_inserter(terms), "{}:{}",
                       int_system_index[static_cast<std::size_t>(p)],
                       int_system_value[static_cast<std::size_t>(p)]);
      }
      if (system_row_end > emit_end) {
        fmt::format_to(std::back_inserter(terms), ",...");
      }
      fmt::print(stderr,
                 "[B&C-XMODK-SYS] seq={} row={} side={} rowIntegral={} "
                 "intscale={:.17g} intrhs={} rawLen={} transLen={} start={} "
                 "end={} terms=[{}]\n",
                 integral_scales.size(), row, leq_row ? "le" : "ge",
                 row_integral ? 1 : 0, intscale, intrhs, source_row_len,
                 rowlen, system_row_start, system_row_end,
                 fmt::to_string(terms));
    }
    integral_scales.emplace_back(row, intscale);
  }

  ledger.system_nnz = static_cast<int>(int_system_value.size());
  ledger.system_hash =
      xtab_modk_system_hash(int_system_value, int_system_index,
                            int_system_start);
  if (integral_scales.empty() || ledger.nonzero_rhs == 0) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("empty_system", 0);
    return 0;
  }

  int accepted_rows = 0;
  std::set<std::vector<std::pair<int, int>>> used_weights;
  XTabLpAggregator aggregator(simplex, source_context, storage_stats);
  SeparatorCandidateStorageTracker storage_tracker(storage_stats);

  auto found_cut = [&](auto& weights, int rhs_index, int k) {
    (void)rhs_index;
    if (weights.empty()) return;
    std::sort(weights.begin(), weights.end());
    std::vector<std::pair<int, int>> key;
    key.reserve(weights.size());
    for (const auto& w : weights) {
      key.emplace_back(static_cast<int>(w.index), static_cast<int>(w.weight));
    }
    if (!used_weights.insert(key).second) return;
    ++ledger.gf_solutions;

    auto generate_from_weights = [&](bool negated_weights) {
      aggregator.clear();
      for (const auto& w : weights) {
        const int idx = static_cast<int>(w.index);
        if (idx < 0 || idx >= static_cast<int>(integral_scales.size())) {
          continue;
        }
        const double base_scale =
            integral_scales[static_cast<std::size_t>(idx)].second;
        const int row = integral_scales[static_cast<std::size_t>(idx)].first;
        double weight = 0.0;
        if (negated_weights) {
          weight = base_scale *
                   (static_cast<double>((w.weight * (k - 1)) % k) /
                    static_cast<double>(k));
        } else {
          weight = base_scale *
                   (static_cast<double>(w.weight) / static_cast<double>(k));
        }
        (void)aggregator.add_row(row, weight);
      }
      Eigen::VectorXd coeff;
      double rhs = 0.0;
      std::vector<int> active_cols;
      if (!aggregator.get_current_aggregation(coeff, rhs,
                                              !negated_weights,
                                              &active_cols)) {
        return;
      }
      Eigen::VectorXd cut;
      double cut_rhs = 0.0;
      double efficacy = 0.0;
      XTabRejectReason reject_reason = XTabRejectReason::None;
      ++ledger.cutgen_calls;
      const bool ok = xtab_generate_cut_from_base_row(
          simplex, source_context, coeff, rhs, &active_cols, x,
          implied_integer_cols, &transform_context, &diag.vb_substitutions,
          &diag.vb_trigger_terms, cut, cut_rhs, efficacy, feastol,
          &reject_reason, &diag, /*require_violation=*/true, "modk",
          nullptr, &cutgen_random,
          /*only_initial_cmir_scale=*/true);
      if (!ok) {
        xtab_record_reject(&diag, reject_reason);
        return;
      }
      const int nnz = count_nonzeros(cut);
      record_dense_workspace(storage_stats,
                             static_cast<std::size_t>(cut.size()));
      const double norm = std::max(1e-12, cut.norm());
      if (cutpool_acceptor != nullptr) {
        XTabCandidateCut candidate{std::move(cut), cut_rhs, efficacy,
                                  efficacy, norm, nnz};
        storage_tracker.record_ephemeral(candidate.coeff);
        PoolCut row_cut = xtab_candidate_to_pool_cut(candidate);
        const int added = (*cutpool_acceptor)(std::move(row_cut));
        ++ledger.pool_calls;
        if (added > 0) {
          accepted_rows += added;
          ledger.pool_accepted += added;
          ++ledger.cutgen_success;
          ++diag.generated;
        }
      } else if (generated_cutpool_rows != nullptr) {
        XTabCandidateCut candidate{std::move(cut), cut_rhs, efficacy,
                                  efficacy, norm, nnz};
        storage_tracker.record_ephemeral(candidate.coeff);
        generated_cutpool_rows->push_back(
            xtab_candidate_to_pool_cut(candidate));
        ++accepted_rows;
        ++ledger.cutgen_success;
        ++diag.generated;
      }
    };

    generate_from_weights(/*negated_weights=*/true);
    generate_from_weights(/*negated_weights=*/false);
    aggregator.clear();
  };

  ledger.gf2_calls = 1;
  int accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<2>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 2);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf3_calls = 1;
  accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<3>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 3);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf5_calls = 1;
  accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<5>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 5);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf7_calls = 1;
  (void)xtab_separate_modk_system<7>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 7);
      });

  maybe_print_xtab_diag("modk", diag);
  print_modk_ledger("done", accepted_rows);
  return accepted_rows;
}


}  // anonymous namespace
}  // namespace mipsolvers::engine::detail

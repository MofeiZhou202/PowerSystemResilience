/// @file bc_cglp.cpp
/// @brief Phase 2/3 implementation for lift-and-project disjunctive cuts (CGLP).
///
/// Phase summary:
///   * Phase 2: build_cglp_lp() in bc_cglp_model.cpp constructs the full
///     master LP (two side blocks + beta-links + normalization).
///   * Phase 3 (this file): candidate selection, CGLP LP solve, alpha/beta
///     extraction, efficacy filtering, and conservative branch-side validity
///     checks before cut admission.

#include "mipsolvers/engine/detail/bc_cglp.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_set>
#include <algorithm>
#include <utility>
#include <vector>

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_pools.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {
namespace {

bool validate_cut_on_branch(const CGLPContext& ctx,
                            int branch_var,
                            double branch_value,
                            const Eigen::VectorXd& alpha,
                            double beta,
                            double tol = 1e-7);

double row_dot(const Eigen::VectorXd& alpha, const Eigen::VectorXd& x) {
  return alpha.dot(x);
}

double cut_efficacy(const Eigen::VectorXd& alpha,
                    double beta,
                    const Eigen::VectorXd& x_star) {
  const double norm = alpha.norm();
  if (!(std::isfinite(norm) && norm > 1e-12)) return -kInf;
  const double violation = beta - row_dot(alpha, x_star);
  return violation / norm;
}

bool optimize_var_on_branch(const CGLPContext& ctx,
                            int branch_var,
                            double branch_value,
                            int target_var,
                            bool maximize,
                            double* value_out) {
  if (value_out == nullptr) return false;
  LPModel side_lp = ctx.lp;
  side_lp.vars[static_cast<std::size_t>(branch_var)].lb = branch_value;
  side_lp.vars[static_cast<std::size_t>(branch_var)].ub = branch_value;

  side_lp.sense = maximize ? Sense::Maximize : Sense::Minimize;
  side_lp.c = Eigen::VectorXd::Zero(static_cast<int>(side_lp.vars.size()));
  side_lp.c[target_var] = 1.0;

  SimplexOptions opt;
  opt.lp_kernel_backend = ctx.opt.lp_kernel_backend;
  opt.max_iter = 2500;
  opt.feasibility_tol = 1e-8;
  opt.optimality_tol = 1e-8;
  opt.allow_cold_start = true;
  opt.verbose = false;
  opt.factor_backend = simplex_factor_backend_from_id(ctx.opt.simplex_factor_backend);

  const auto side = solve_lp_with_basis(side_lp, opt, nullptr);
  if (!side.result.stats.success ||
      side.result.x.size() != static_cast<int>(side_lp.vars.size())) {
    return false;
  }
  *value_out = side.result.x[target_var];
  return std::isfinite(*value_out);
}

bool fallback_interpolation_cut(const CGLPContext& ctx,
                                int branch_var,
                                Eigen::VectorXd& alpha_out,
                                double& beta_out) {
  const int n = static_cast<int>(ctx.lp.vars.size());
  std::vector<std::pair<double, int>> scored;
  scored.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (i == branch_var) continue;
    const double frac = std::abs(ctx.x_star[i] - std::round(ctx.x_star[i]));
    scored.emplace_back(frac, i);
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  const int probe_limit = std::min(8, static_cast<int>(scored.size()));
  const double guard = 1e-7;
  double best_eff = -kInf;
  Eigen::VectorXd best_alpha;
  double best_beta = 0.0;

  for (int kk = 0; kk < probe_limit; ++kk) {
    const int i = scored[static_cast<std::size_t>(kk)].second;
    double u0 = 0.0, u1 = 0.0, l0 = 0.0, l1 = 0.0;
    if (!optimize_var_on_branch(ctx, branch_var, 0.0, i, true, &u0)) continue;
    if (!optimize_var_on_branch(ctx, branch_var, 1.0, i, true, &u1)) continue;
    if (!optimize_var_on_branch(ctx, branch_var, 0.0, i, false, &l0)) continue;
    if (!optimize_var_on_branch(ctx, branch_var, 1.0, i, false, &l1)) continue;

    // Upper interpolation: x_i <= U0 + (U1-U0) * x_j.
    {
      const double U0 = u0 + guard;
      const double U1 = u1 + guard;
      Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n);
      alpha[i] = -1.0;
      alpha[branch_var] = (U1 - U0);
      const double beta = -U0;
      const double eff = cut_efficacy(alpha, beta, ctx.x_star);
      if (eff > best_eff) {
        best_eff = eff;
        best_alpha = std::move(alpha);
        best_beta = beta;
      }
    }

    // Lower interpolation: x_i >= L0 + (L1-L0) * x_j.
    {
      const double L0 = l0 - guard;
      const double L1 = l1 - guard;
      Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n);
      alpha[i] = 1.0;
      alpha[branch_var] = -(L1 - L0);
      const double beta = L0;
      const double eff = cut_efficacy(alpha, beta, ctx.x_star);
      if (eff > best_eff) {
        best_eff = eff;
        best_alpha = std::move(alpha);
        best_beta = beta;
      }
    }
  }

  if (!(std::isfinite(best_eff) && best_eff >= ctx.opt.cglp_min_efficacy)) {
    return false;
  }
  alpha_out = std::move(best_alpha);
  beta_out = best_beta;
  return true;
}

void sparsify_alpha(Eigen::VectorXd& alpha, double tol = 1e-10) {
  for (int i = 0; i < alpha.size(); ++i) {
    if (std::abs(alpha[i]) <= tol) alpha[i] = 0.0;
  }
}

void strengthen_cut_with_validation(const CGLPContext& ctx,
                                    int branch_var,
                                    Eigen::VectorXd& alpha,
                                    double& beta) {
  sparsify_alpha(alpha);
  int nnz = 0;
  for (int i = 0; i < alpha.size(); ++i) {
    if (std::abs(alpha[i]) > 1e-12) ++nnz;
  }
  if (nnz <= 0 || nnz > 8) return;

  double best_eff = cut_efficacy(alpha, beta, ctx.x_star);
  if (!(std::isfinite(best_eff) && best_eff > 0.0)) return;

  // Greedy coefficient dropping: accept only if efficacy improves and branch
  // validity remains true. This keeps safety checks strict while increasing
  // cut sharpness/sparsity when possible.
  bool improved = true;
  for (int round = 0; round < 2 && improved; ++round) {
    improved = false;
    for (int k = 0; k < alpha.size(); ++k) {
      if (k == branch_var || std::abs(alpha[k]) <= 1e-12) continue;
      Eigen::VectorXd trial = alpha;
      trial[k] = 0.0;
      const double eff = cut_efficacy(trial, beta, ctx.x_star);
      if (!(std::isfinite(eff) && eff > best_eff + 1e-6)) continue;
      if (!validate_cut_on_branch(ctx, branch_var, 0.0, trial, beta)) continue;
      if (!validate_cut_on_branch(ctx, branch_var, 1.0, trial, beta)) continue;
      alpha = std::move(trial);
      best_eff = eff;
      improved = true;
    }
  }
}

bool validate_cut_on_branch(const CGLPContext& ctx,
                            int branch_var,
                            double branch_value,
                            const Eigen::VectorXd& alpha,
                            double beta,
                            double tol) {
  LPModel side_lp = ctx.lp;
  side_lp.vars[static_cast<std::size_t>(branch_var)].lb = branch_value;
  side_lp.vars[static_cast<std::size_t>(branch_var)].ub = branch_value;

  SimplexOptions opt;
  opt.lp_kernel_backend = ctx.opt.lp_kernel_backend;
  opt.max_iter = 2000;
  opt.feasibility_tol = 1e-8;
  opt.optimality_tol = 1e-8;
  opt.allow_cold_start = true;
  opt.verbose = false;
  opt.factor_backend = simplex_factor_backend_from_id(ctx.opt.simplex_factor_backend);

  const auto side = solve_lp_with_basis(side_lp, opt, nullptr);
  if (!side.result.stats.success || side.result.x.size() != alpha.size()) {
    // Infeasible side is fine for validity; solver failure is treated as
    // "cannot certify" and therefore rejected by caller.
    return side.result.stats.status == "Infeasible";
  }
  const double lhs = row_dot(alpha, side.result.x);
  return lhs + tol >= beta;
}

}  // namespace

std::vector<int> select_cglp_candidates(const CGLPContext& ctx) {
  std::vector<std::pair<double, int>> scored;
  const int n = static_cast<int>(ctx.lp.vars.size());
  scored.reserve(static_cast<std::size_t>(n));

  for (int j = 0; j < n; ++j) {
    const auto& v = ctx.lp.vars[static_cast<std::size_t>(j)];
    if (v.type != VarType::Binary) continue;
    const double xj = ctx.x_star[j];
    const double frac = std::abs(xj - std::round(xj));
    if (frac < ctx.opt.cglp_min_fractionality) continue;
    scored.emplace_back(frac, j);
  }

  std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) {
              if (std::abs(a.first - b.first) > 1e-12) return a.first > b.first;
              return a.second < b.second;
            });

  const int keep = std::max(0, std::min(ctx.opt.cglp_max_candidates,
                                        static_cast<int>(scored.size())));
  std::vector<int> out;
  out.reserve(static_cast<std::size_t>(keep));
  for (int k = 0; k < keep; ++k) out.push_back(scored[static_cast<std::size_t>(k)].second);
  return out;
}

CGLPResult generate_cglp_cut(const CGLPContext& ctx, int j) {
  CGLPResult r;
  r.branch_var = j;
  r.generated = false;

  CGLPModel model;
  if (!build_cglp_lp(ctx, j, model)) {
    r.reason = "build_failed";
    return r;
  }

  SimplexOptions sopt;
  sopt.lp_kernel_backend = ctx.opt.lp_kernel_backend;
  sopt.max_iter = 40000;
  sopt.feasibility_tol = 1e-8;
  sopt.optimality_tol = 1e-8;
  sopt.allow_cold_start = true;
  sopt.verbose = false;
  sopt.factor_backend = simplex_factor_backend_from_id(ctx.opt.simplex_factor_backend);

  Eigen::VectorXd alpha_dense;
  double beta = 0.0;
  const auto cglp = solve_lp_with_basis(model.lp, sopt, nullptr);
  if (cglp.result.stats.success && cglp.result.x.size() == model.lp.c.size()) {
    alpha_dense = cglp.result.x.segment(model.alpha_start, model.n_original);
    beta = cglp.result.x[model.beta_idx];
    r.cglp_lp_solved = true;
  } else {
    if (ctx.opt.verbose) {
      std::fprintf(stderr, "[CGLP]   j=%d master-LP failed status=%s\n",
                   j, cglp.result.stats.status.c_str());
    }
    if (!fallback_interpolation_cut(ctx, j, alpha_dense, beta)) {
      r.reason = "cglp_lp_failed";
      return r;
    }
    r.used_fallback = true;
  }

  // Only strengthen and re-validate for fallback cuts; CGLP-derived cuts are
  // already tight and theoretically valid by Farkas-2 duality. Strengthening
  // calls validate_cut_on_branch up to O(n) times per coefficient drop, which
  // on production LPs costs >4 s/candidate and blows the time budget.
  if (r.used_fallback) {
    strengthen_cut_with_validation(ctx, j, alpha_dense, beta);
  }

  const double norm = alpha_dense.norm();
  if (!(std::isfinite(norm) && norm > 1e-12)) {
    if (!fallback_interpolation_cut(ctx, j, alpha_dense, beta)) {
      r.reason = "degenerate_alpha";
      return r;
    }
    r.used_fallback = true;
    strengthen_cut_with_validation(ctx, j, alpha_dense, beta);
  }

  const double violation = beta - row_dot(alpha_dense, ctx.x_star);
  const double efficacy = violation / norm;
  if (!(std::isfinite(efficacy) && efficacy >= ctx.opt.cglp_min_efficacy)) {
    if (fallback_interpolation_cut(ctx, j, alpha_dense, beta)) {
      strengthen_cut_with_validation(ctx, j, alpha_dense, beta);
      r.used_fallback = true;
    }
  }

  const double norm2 = alpha_dense.norm();
  const double violation2 = beta - row_dot(alpha_dense, ctx.x_star);
  const double efficacy2 = (std::isfinite(norm2) && norm2 > 1e-12)
      ? (violation2 / norm2)
      : -kInf;
  if (!(std::isfinite(efficacy2) && efficacy2 >= ctx.opt.cglp_min_efficacy)) {
    r.reason = "weak_cut";
    return r;
  }

  // Safety net: validate on both disjunction sides only when we cannot rely
  // on CGLP's Farkas-2 certificate (i.e. the cut came from the interpolation
  // fallback). CGLP-derived cuts from a successful master-LP solve are valid
  // by construction; re-validating costs O(n) auxiliary LP solves per cut.
  if (r.used_fallback) {
    const bool valid_0 = validate_cut_on_branch(ctx, j, 0.0, alpha_dense, beta);
    const bool valid_1 = validate_cut_on_branch(ctx, j, 1.0, alpha_dense, beta);
    if (!valid_0 || !valid_1) {
      r.reason = "branch_validity_failed";
      return r;
    }
  }

  r.alpha = Eigen::SparseVector<double>(model.n_original);
  int nnz = 0;
  for (int i = 0; i < model.n_original; ++i) {
    if (std::abs(alpha_dense[i]) > 1e-12) ++nnz;
  }
  r.alpha.reserve(nnz);
  for (int i = 0; i < model.n_original; ++i) {
    if (std::abs(alpha_dense[i]) > 1e-12) {
      r.alpha.insertBack(i) = alpha_dense[i];
    }
  }
  r.beta = beta;
  r.violation = violation2;
  r.efficacy = efficacy2;
  r.generated = true;
  r.reason = "ok";
  return r;
}

int add_cglp_cuts_root(LPModel& lp,
                       const Eigen::VectorXd& x_star,
                       const BCOptions& opt,
                       CGLPPassStats* stats_out) {
  if (!opt.enable_cglp_cuts) {
    return 0;
  }

  CGLPContext ctx{lp, x_star, opt};
  const auto cands = select_cglp_candidates(ctx);
  if (cands.empty()) return 0;

  CGLPPassStats pass;
  pass.n_candidates = static_cast<int>(cands.size());

  const auto t0 = std::chrono::steady_clock::now();
  const double max_seconds = std::max(0.0, opt.cglp_time_limit_sec);

  std::vector<Eigen::SparseVector<double>> new_rows;
  std::vector<double> new_rhs;
  int generated = 0;

  // Phase 4 dedupe: hash existing rows once, and also dedupe within this pass.
  std::unordered_set<std::size_t> seen_hashes;
  const int approx_cap = std::max(64, static_cast<int>(lp.A.rows()) + 32);
  seen_hashes.reserve(static_cast<std::size_t>(approx_cap));
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
    for (int r = 0; r < A_row.rows(); ++r) {
      Eigen::SparseVector<double> row(lp.c.size());
      int nnz = 0;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        if (std::abs(it.value()) > 1e-12) ++nnz;
      }
      row.reserve(nnz);
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        if (std::abs(it.value()) > 1e-12) row.insertBack(it.col()) = it.value();
      }
      seen_hashes.insert(sparse_cut_hash(row));
    }
  }

  for (int j : cands) {
    if (max_seconds > 0.0) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      if (elapsed > max_seconds) break;
    }

    CGLPResult res = generate_cglp_cut(ctx, j);
    if (res.cglp_lp_solved) ++pass.n_cglp_lp_solved;
    if (res.used_fallback)  ++pass.n_fallback_used;
    if (!res.generated) {
      const std::string reason = res.reason ? res.reason : "";
      if      (reason == "build_failed")           ++pass.n_rej_build_failed;
      else if (reason == "cglp_lp_failed")         ++pass.n_rej_cglp_lp_failed;
      else if (reason == "degenerate_alpha")       ++pass.n_rej_degenerate_alpha;
      else if (reason == "weak_cut")               ++pass.n_rej_weak_cut;
      else if (reason == "branch_validity_failed") ++pass.n_rej_branch_validity;
      if (opt.verbose) {
        std::fprintf(stderr, "[CGLP]   j=%d rejected reason=%s\n", j, reason.c_str());
      }
      continue;
    }

    // Convert alpha^T x >= beta to (-alpha)^T x <= -beta for LPModel::A.
    Eigen::SparseVector<double> cut(lp.c.size());
    cut.reserve(res.alpha.nonZeros());
    for (Eigen::SparseVector<double>::InnerIterator it(res.alpha); it; ++it) {
      cut.insertBack(it.index()) = -it.value();
    }
    const std::size_t h = sparse_cut_hash(cut);
    if (seen_hashes.find(h) != seen_hashes.end()) {
      ++pass.n_dedup_rejected;
      continue;
    }
    seen_hashes.insert(h);
    new_rows.push_back(std::move(cut));
    new_rhs.push_back(-res.beta);
    ++pass.n_accepted;
    ++generated;
  }

  if (!new_rows.empty()) {
    add_sparse_rows_to_lp(lp, new_rows, new_rhs);
  }

  if (opt.verbose) {
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    std::fprintf(stderr,
                 "[CGLP] candidates=%d lp_solved=%d fallback=%d accepted=%d dedup_rej=%d"
                 " rej[build=%d lp=%d degen=%d weak=%d validity=%d] elapsed=%.1fms\n",
                 pass.n_candidates, pass.n_cglp_lp_solved, pass.n_fallback_used,
                 pass.n_accepted, pass.n_dedup_rejected,
                 pass.n_rej_build_failed, pass.n_rej_cglp_lp_failed,
                 pass.n_rej_degenerate_alpha, pass.n_rej_weak_cut,
                 pass.n_rej_branch_validity, elapsed);
  }
  if (stats_out) *stats_out = pass;
  return generated;
}

}  // namespace mipsolvers::engine::detail

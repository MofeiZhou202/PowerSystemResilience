/// @file bc_cglp_model.cpp
/// @brief Phase 2 CGLP model builder.

#include "mipsolvers/engine/detail/bc_cglp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Sparse>

namespace mipsolvers::engine::detail {
namespace {

constexpr double kFiniteBound = 1e19;

bool is_finite_bound(double v) {
  return std::isfinite(v) && std::abs(v) < kFiniteBound;
}

void push_matrix_row(std::vector<CGLPCanonicalRow>& rows,
                     const Eigen::SparseMatrix<double>& A,
                     int r,
                     double rhs,
                     double scale = 1.0) {
  const int n = static_cast<int>(A.cols());
  CGLPCanonicalRow row;
  row.coeff = Eigen::SparseVector<double>(n);

  int nnz = 0;
  for (Eigen::SparseMatrix<double>::InnerIterator it(A, r); it; ++it) {
    if (std::abs(scale * it.value()) > 1e-15) ++nnz;
  }
  row.coeff.reserve(nnz);
  for (Eigen::SparseMatrix<double>::InnerIterator it(A, r); it; ++it) {
    const double v = scale * it.value();
    if (std::abs(v) > 1e-15) {
      row.coeff.insertBack(it.col()) = v;
    }
  }
  row.rhs = scale * rhs;
  rows.push_back(std::move(row));
}

void append_canonical_rows(const LPModel& lp,
                           std::vector<CGLPCanonicalRow>& rows) {
  const int n = static_cast<int>(lp.vars.size());

  // Inequalities A x <= b.
  Eigen::SparseMatrix<double> A_col = lp.A;
  A_col.makeCompressed();
  for (int r = 0; r < A_col.rows(); ++r) {
    push_matrix_row(rows, A_col, r, lp.b[r], 1.0);
  }

  // Equalities Aeq x = beq  ->  Aeq x <= beq and -Aeq x <= -beq.
  Eigen::SparseMatrix<double> Aeq_col = lp.Aeq;
  Aeq_col.makeCompressed();
  for (int r = 0; r < Aeq_col.rows(); ++r) {
    push_matrix_row(rows, Aeq_col, r, lp.beq[r], 1.0);
    push_matrix_row(rows, Aeq_col, r, lp.beq[r], -1.0);
  }

  // Finite variable bounds as inequalities.
  for (int j = 0; j < n; ++j) {
    if (is_finite_bound(lp.vars[static_cast<std::size_t>(j)].ub)) {
      CGLPCanonicalRow up;
      up.coeff = Eigen::SparseVector<double>(n);
      up.coeff.reserve(1);
      up.coeff.insertBack(j) = 1.0;
      up.rhs = lp.vars[static_cast<std::size_t>(j)].ub;
      rows.push_back(std::move(up));
    }
    if (is_finite_bound(lp.vars[static_cast<std::size_t>(j)].lb)) {
      CGLPCanonicalRow lo;
      lo.coeff = Eigen::SparseVector<double>(n);
      lo.coeff.reserve(1);
      lo.coeff.insertBack(j) = -1.0;
      lo.rhs = -lp.vars[static_cast<std::size_t>(j)].lb;
      rows.push_back(std::move(lo));
    }
  }
}

}  // namespace

bool build_cglp_lp(const CGLPContext& ctx, int j, CGLPModel& out_model) {
  out_model = CGLPModel{};

  const int n = static_cast<int>(ctx.lp.vars.size());
  if (j < 0 || j >= n) return false;
  if (ctx.lp.vars[static_cast<std::size_t>(j)].type != VarType::Binary) return false;
  if (ctx.x_star.size() != n) return false;

  out_model.n_original = n;
  out_model.branch_var = j;

  append_canonical_rows(ctx.lp, out_model.canonical_rows);
  const int R = static_cast<int>(out_model.canonical_rows.size());
  if (R <= 0) return false;
  out_model.canonical_row_count = R;

  // Variable indexing.
  out_model.alpha_start = 0;
  out_model.beta_idx = n;
  out_model.lambda0_start = n + 1;
  out_model.gamma0_idx = out_model.lambda0_start + R;
  out_model.lambda1_start = out_model.gamma0_idx + 1;
  out_model.gamma1_idx = out_model.lambda1_start + R;
  const int nvars = out_model.gamma1_idx + 1;

  LPModel cglp;
  cglp.sense = Sense::Minimize;
  cglp.c = Eigen::VectorXd::Zero(nvars);
  cglp.vars.reserve(static_cast<std::size_t>(nvars));

  // Objective: min alpha^T x* - beta.
  for (int col = 0; col < n; ++col) {
    cglp.c[out_model.alpha_start + col] = ctx.x_star[col];
  }
  cglp.c[out_model.beta_idx] = -1.0;

  for (int col = 0; col < n; ++col) {
    cglp.vars.push_back(VariableMeta{VarType::Continuous, -kInf, kInf, "alpha_" + std::to_string(col)});
  }
  cglp.vars.push_back(VariableMeta{VarType::Continuous, -kInf, kInf, "beta"});
  for (int r = 0; r < R; ++r) {
    cglp.vars.push_back(VariableMeta{VarType::Continuous, 0.0, kInf, "lam0_" + std::to_string(r)});
  }
  cglp.vars.push_back(VariableMeta{VarType::Continuous, 0.0, kInf, "gam0"});
  for (int r = 0; r < R; ++r) {
    cglp.vars.push_back(VariableMeta{VarType::Continuous, 0.0, kInf, "lam1_" + std::to_string(r)});
  }
  cglp.vars.push_back(VariableMeta{VarType::Continuous, 0.0, kInf, "gam1"});

  // Equality rows (Farkas-2 certificate for Mx <= d with disjunction x_j<=0 | x_j>=1).
  // 1) alpha + M^T*lambda0 + e_j*gamma0 = 0 (n rows)
  // 2) alpha + M^T*lambda1 - e_j*gamma1 = 0 (n rows)
  // 3) beta + d^T*lambda0 <= 0
  // 4) beta + d^T*lambda1 - gamma1 <= 0
  // 5) normalization: sum(lambda0)+gamma0+sum(lambda1)+gamma1 = 1
  const int n_eq_rows = 2 * n + 1;
  const int n_ineq_rows = 2;

  std::vector<Eigen::Triplet<double>> eq_triplets;
  eq_triplets.reserve(static_cast<std::size_t>(2 * n + 2 * R * n + 2 * R + 2));
  cglp.beq = Eigen::VectorXd::Zero(n_eq_rows);

  // alpha block
  for (int i = 0; i < n; ++i) {
    eq_triplets.emplace_back(i, out_model.alpha_start + i, 1.0);
    eq_triplets.emplace_back(n + i, out_model.alpha_start + i, 1.0);
  }

  // M^T * lambdas in both sides (Farkas-2 sign: +M[r,i] so that alpha = -M^T lambda).
  for (int r = 0; r < R; ++r) {
    for (Eigen::SparseVector<double>::InnerIterator it(out_model.canonical_rows[static_cast<std::size_t>(r)].coeff); it; ++it) {
      const int i = static_cast<int>(it.index());
      const double a = it.value();
      eq_triplets.emplace_back(i, out_model.lambda0_start + r, a);
      eq_triplets.emplace_back(n + i, out_model.lambda1_start + r, a);
    }
  }

  // Disjunction multipliers on branch var.
  eq_triplets.emplace_back(j, out_model.gamma0_idx, 1.0);
  eq_triplets.emplace_back(n + j, out_model.gamma1_idx, -1.0);

  // Normalization row.
  const int norm_row = 2 * n;
  for (int r = 0; r < R; ++r) {
    eq_triplets.emplace_back(norm_row, out_model.lambda0_start + r, 1.0);
    eq_triplets.emplace_back(norm_row, out_model.lambda1_start + r, 1.0);
  }
  eq_triplets.emplace_back(norm_row, out_model.gamma0_idx, 1.0);
  eq_triplets.emplace_back(norm_row, out_model.gamma1_idx, 1.0);
  cglp.beq[norm_row] = 1.0;

  cglp.Aeq.resize(n_eq_rows, nvars);
  cglp.Aeq.setFromTriplets(eq_triplets.begin(), eq_triplets.end());
  cglp.Aeq.makeCompressed();

  std::vector<Eigen::Triplet<double>> ineq_triplets;
  ineq_triplets.reserve(static_cast<std::size_t>(2 + 2 * R + 1));
  cglp.b = Eigen::VectorXd::Zero(n_ineq_rows);

  // beta + d^T lambda0 <= 0  (so beta <= -d^T lambda0)
  ineq_triplets.emplace_back(0, out_model.beta_idx, 1.0);
  for (int r = 0; r < R; ++r) {
    ineq_triplets.emplace_back(0, out_model.lambda0_start + r,
                               out_model.canonical_rows[static_cast<std::size_t>(r)].rhs);
  }
  cglp.b[0] = 0.0;

  // beta + d^T lambda1 - gamma1 <= 0  (so beta <= -d^T lambda1 + gamma1)
  ineq_triplets.emplace_back(1, out_model.beta_idx, 1.0);
  for (int r = 0; r < R; ++r) {
    ineq_triplets.emplace_back(1, out_model.lambda1_start + r,
                               out_model.canonical_rows[static_cast<std::size_t>(r)].rhs);
  }
  ineq_triplets.emplace_back(1, out_model.gamma1_idx, -1.0);
  cglp.b[1] = 0.0;

  cglp.A.resize(n_ineq_rows, nvars);
  cglp.A.setFromTriplets(ineq_triplets.begin(), ineq_triplets.end());
  cglp.A.makeCompressed();

  out_model.lp = std::move(cglp);
  return true;
}

}  // namespace mipsolvers::engine::detail

/// @file bc_root_highs_oracle.hpp
/// @brief Root HiGHS oracle basis-ops, xpool signatures, and strict warm-start
/// injection \u2014 extracted from branch_and_cut.cpp. HiGHS-only (guarded).
/// Co-located internal header; included only by branch_and_cut.cpp.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#include "util/HVector.h"
#include "util/HighsHash.h"

namespace mipsolvers::engine::detail {

class RootHighsOracleBasisOps : public BasisOps {
 public:
  RootHighsOracleBasisOps(std::shared_ptr<Highs> highs, int rows, int cols,
                          const StandardColumnMatrix* A)
      : highs_(std::move(highs)),
        m_(rows),
        n_(cols),
        A_owned_(A != nullptr ? *A : StandardColumnMatrix()) {}

  BasisOpsKind kind() const override { return BasisOpsKind::VendoredHighs; }

  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(
          m_, std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisTransposeSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(
          m_, std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override {
    out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || row < 0 || row >= m_) return false;
    std::vector<double> row_vec(static_cast<std::size_t>(m_), 0.0);
    HighsInt row_num_nz = 0;
    std::vector<HighsInt> row_indices(static_cast<std::size_t>(m_), 0);
    const HighsStatus st = highs_->getBasisInverseRow(
        static_cast<HighsInt>(row), row_vec.data(), &row_num_nz,
        row_indices.data());
    if (st != HighsStatus::kOk || row_num_nz < 0 || row_num_nz > m_) {
      return false;
    }
    for (HighsInt k = 0; k < row_num_nz; ++k) {
      const int r = static_cast<int>(row_indices[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      out[r] = row_vec[static_cast<std::size_t>(r)];
    }
    return out.allFinite();
  }

  bool basis_inverse_row_sparse_entries(
      int row, std::vector<std::pair<int, double>>& out) const override {
    out.clear();
    if (!highs_ || row < 0 || row >= m_) return false;
    HVector row_ep;
    row_ep.setup(static_cast<HighsInt>(m_));
    if (highs_->getBasisInverseRowSparse(static_cast<HighsInt>(row), row_ep) !=
        HighsStatus::kOk) {
      return false;
    }
    out.reserve(static_cast<std::size_t>(std::max<HighsInt>(0, row_ep.count)));
    for (HighsInt k = 0; k < row_ep.count; ++k) {
      const int r = static_cast<int>(row_ep.index[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      const double value = row_ep.array[static_cast<std::size_t>(r)];
      if (!std::isfinite(value)) return false;
      out.emplace_back(r, value);
    }
    return true;
  }

  bool tableau_row(int row, Eigen::RowVectorXd& out) const override {
    if (!highs_ || row < 0 || row >= m_ || n_ <= 0) return false;
    Eigen::VectorXd y;
    if (!basis_inverse_row(row, y)) return false;
    if (A_owned_.rows() != m_ || A_owned_.cols() != n_) return false;
    out = A_owned_.transpose_multiply(y).transpose();
    return out.allFinite();
  }

  SparseFactorTelemetry factor_telemetry() const override {
    SparseFactorTelemetry t;
    t.ft_valid = true;
    return t;
  }

  void rebind_A(const StandardColumnMatrix& A) override { A_owned_ = A; }
  bool bound_to_A(const StandardColumnMatrix& A) const override {
    return A.rows() == m_ && A.cols() == n_;
  }
  std::shared_ptr<Highs> highs_handle() const override { return highs_; }

 private:
  std::shared_ptr<Highs> highs_;
  int m_{0};
  int n_{0};
  StandardColumnMatrix A_owned_;
};

struct RootXpoolStateSignature {
  int fractional{0};
  std::uint64_t frontier_hash{0};
  std::uint64_t status_hash{0};
  std::uint64_t basis_hash{0};
  int basic_cols{0};
  int lower_cols{0};
  int upper_cols{0};
  int zero_cols{0};
  int other_cols{0};
};

inline char root_highs_status_char(HighsBasisStatus status) {
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
}

inline std::uint64_t root_xpool_ledger_mix(std::uint64_t h, std::uint64_t v) {
  v ^= v >> 33;
  v *= std::uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= std::uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

inline std::uint64_t root_xpool_ledger_double(double value) {
  return static_cast<std::uint64_t>(HighsHashHelpers::double_hash_code(value));
}

struct RootXpoolPayloadSignature {
  int rows{0};
  int nnz{0};
  std::uint64_t hash{0};
};

inline RootXpoolPayloadSignature root_xpool_cutset_payload_signature(
    const HighsCutSet& cutset) {
  RootXpoolPayloadSignature sig;
  sig.rows = static_cast<int>(cutset.numCuts());
  sig.hash = std::uint64_t{0x4849474858504150};
  for (HighsInt i = 0; i != cutset.numCuts(); ++i) {
    const HighsInt start = cutset.ARstart_[static_cast<std::size_t>(i)];
    const HighsInt end = cutset.ARstart_[static_cast<std::size_t>(i + 1)];
    const HighsInt len = end - start;
    sig.nnz += static_cast<int>(len);
    std::uint64_t row_hash = std::uint64_t{0x4849474858505257};
    row_hash = root_xpool_ledger_mix(row_hash, static_cast<std::uint64_t>(i));
    row_hash =
        root_xpool_ledger_mix(row_hash, static_cast<std::uint64_t>(len));
    row_hash = root_xpool_ledger_mix(
        row_hash, root_xpool_ledger_double(cutset.lower_[static_cast<std::size_t>(i)]));
    row_hash = root_xpool_ledger_mix(
        row_hash, root_xpool_ledger_double(cutset.upper_[static_cast<std::size_t>(i)]));
    for (HighsInt k = start; k != end; ++k) {
      row_hash = root_xpool_ledger_mix(
          row_hash,
          static_cast<std::uint64_t>(cutset.ARindex_[static_cast<std::size_t>(k)]));
      row_hash = root_xpool_ledger_mix(
          row_hash,
          root_xpool_ledger_double(cutset.ARvalue_[static_cast<std::size_t>(k)]));
    }
    sig.hash = root_xpool_ledger_mix(sig.hash, row_hash);
  }
  return sig;
}

inline RootXpoolPayloadSignature root_xpool_lp_row_payload_signature(
    const HighsLp& lp,
    HighsInt first_row,
    HighsInt num_rows) {
  RootXpoolPayloadSignature sig;
  sig.rows = static_cast<int>(std::max<HighsInt>(0, num_rows));
  sig.hash = std::uint64_t{0x4849474858504150};
  if (num_rows <= 0 || first_row < 0 || first_row + num_rows > lp.num_row_) {
    sig.rows = 0;
    return sig;
  }
  std::vector<HighsInt> index(
      static_cast<std::size_t>(std::max<HighsInt>(0, lp.num_col_)));
  std::vector<double> value(
      static_cast<std::size_t>(std::max<HighsInt>(0, lp.num_col_)));
  for (HighsInt ord = 0; ord != num_rows; ++ord) {
    HighsInt len = 0;
    lp.a_matrix_.getRow(first_row + ord, len, index.data(), value.data());
    sig.nnz += static_cast<int>(len);
    std::uint64_t row_hash = std::uint64_t{0x4849474858505257};
    row_hash =
        root_xpool_ledger_mix(row_hash, static_cast<std::uint64_t>(ord));
    row_hash =
        root_xpool_ledger_mix(row_hash, static_cast<std::uint64_t>(len));
    row_hash = root_xpool_ledger_mix(
        row_hash,
        root_xpool_ledger_double(lp.row_lower_[static_cast<std::size_t>(first_row + ord)]));
    row_hash = root_xpool_ledger_mix(
        row_hash,
        root_xpool_ledger_double(lp.row_upper_[static_cast<std::size_t>(first_row + ord)]));
    for (HighsInt k = 0; k != len; ++k) {
      row_hash =
          root_xpool_ledger_mix(row_hash, static_cast<std::uint64_t>(index[k]));
      row_hash =
          root_xpool_ledger_mix(row_hash, root_xpool_ledger_double(value[k]));
    }
    sig.hash = root_xpool_ledger_mix(sig.hash, row_hash);
  }
  return sig;
}

inline RootXpoolStateSignature root_xpool_state_signature_from_highs(
    const LPModel& lp,
    const HighsSolution& sol,
    const HighsBasis& basis,
    int n,
    const std::vector<char>& implied_integer_cols,
    double feastol) {
  RootXpoolStateSignature sig;
  sig.frontier_hash = std::uint64_t{0x4849474853465241};
  sig.status_hash = std::uint64_t{0x4849474853535441};
  sig.basis_hash = std::uint64_t{0x4849474858424153};
  if (static_cast<int>(lp.vars.size()) < n ||
      static_cast<int>(sol.col_value.size()) < n) {
    sig.other_cols = n;
    return sig;
  }
  for (int j = 0; j < n; ++j) {
    const double value = sol.col_value[static_cast<std::size_t>(j)];
    const HighsBasisStatus raw_status =
        basis.valid && j < static_cast<int>(basis.col_status.size())
            ? basis.col_status[static_cast<std::size_t>(j)]
            : HighsBasisStatus::kNonbasic;
    const char status = root_highs_status_char(raw_status);
    switch (raw_status) {
      case HighsBasisStatus::kBasic:
        ++sig.basic_cols;
        break;
      case HighsBasisStatus::kLower:
        ++sig.lower_cols;
        break;
      case HighsBasisStatus::kUpper:
        ++sig.upper_cols;
        break;
      case HighsBasisStatus::kZero:
        ++sig.zero_cols;
        break;
      case HighsBasisStatus::kNonbasic:
        ++sig.other_cols;
        break;
    }
    sig.status_hash =
        root_xpool_ledger_mix(sig.status_hash, static_cast<std::uint64_t>(j));
    sig.status_hash = root_xpool_ledger_mix(
        sig.status_hash, static_cast<std::uint64_t>(status));
    sig.status_hash =
        root_xpool_ledger_mix(sig.status_hash, root_xpool_ledger_double(value));
    sig.basis_hash =
        root_xpool_ledger_mix(sig.basis_hash, static_cast<std::uint64_t>(j));
    sig.basis_hash =
        root_xpool_ledger_mix(sig.basis_hash, static_cast<std::uint64_t>(status));

    const bool integer_like =
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Binary ||
        lp.vars[static_cast<std::size_t>(j)].type == VarType::Integer ||
        (j < static_cast<int>(implied_integer_cols.size()) &&
         implied_integer_cols[static_cast<std::size_t>(j)] != 0);
    if (!integer_like || !std::isfinite(value)) continue;
    const double rounded =
        std::min(lp.vars[static_cast<std::size_t>(j)].ub,
                 std::max(lp.vars[static_cast<std::size_t>(j)].lb,
                          std::round(value)));
    const double frac = std::abs(value - rounded);
    if (!(frac > feastol)) continue;
    ++sig.fractional;
    sig.frontier_hash =
        root_xpool_ledger_mix(sig.frontier_hash, static_cast<std::uint64_t>(j));
    sig.frontier_hash =
        root_xpool_ledger_mix(sig.frontier_hash, root_xpool_ledger_double(value));
    sig.frontier_hash = root_xpool_ledger_mix(
        sig.frontier_hash, static_cast<std::uint64_t>(status));
  }
  return sig;
}

// ── StrictHiGHS warm-start injection ─────────────────────────────────────────
// Injects a caller-provided initial_solution into a HiGHS MIP instance before
// highs.run().  Strategy:
//   1. Direct injection: clamp+project, check satisfies_with_bounds(tol=1e-4).
//      Works whenever the solution was produced by a prior MIP solve.
//   2. Dispatch LP fallback: if the direct check fails AND ws_nfree ≤ 8000,
//      fix integer variables and re-solve continuous dispatch via IPM.
//   3. Skip: direct check failed and ws_nfree > 8000 (dispatch LP too large).
// Returns the number of LP solves consumed (0 or 1).
inline int bc_strict_highs_inject_warm_start(Highs& highs,
                                      const LPModel& base_lp,
                                      const Eigen::VectorXd& initial_solution,
                                      const BCOptions& opt) {
  const int orig_n = static_cast<int>(base_lp.vars.size());
  if (static_cast<int>(initial_solution.size()) != orig_n) return 0;

  Eigen::VectorXd olb(orig_n), oub(orig_n);
  for (int i = 0; i < orig_n; ++i) {
    olb[i] = base_lp.vars[i].lb;
    oub[i] = base_lp.vars[i].ub;
  }
  Eigen::VectorXd x0 =
      project_integer_solution(base_lp.vars, initial_solution, olb, oub);

  std::vector<int> ws_free, ws_fixed;
  for (int i = 0; i < orig_n; ++i) {
    if (is_integer_type(base_lp.vars[i]) || olb[i] >= oub[i] - 1e-12)
      ws_fixed.push_back(i);
    else
      ws_free.push_back(i);
  }
  const int ws_nfree = static_cast<int>(ws_free.size());
  if (ws_nfree == 0) return 0;

  static constexpr int kMaxDispatchFreeVars = 8000;
  const Eigen::VectorXd x0c = clamp_to_bounds(x0, olb, oub);

  if (satisfies_with_bounds(base_lp, x0c, olb, oub, 1e-4)) {
    // ── Direct injection ──────────────────────────────────────────────────
    HighsSolution mip_start;
    mip_start.value_valid = true;
    mip_start.dual_valid = false;
    mip_start.col_value.assign(x0c.data(), x0c.data() + orig_n);
    highs.setSolution(mip_start);
    if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C-STRICT] warm-start direct inject: obj={:.6g}\n",
                 objective_value(base_lp.c, x0c, base_lp.sense));
    }
    return 0;
  }

  if (ws_nfree > kMaxDispatchFreeVars) {
    if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C-STRICT] warm-start skipped: solution not feasible and "
                 "nfree={} > {} (dispatch LP too large)\n",
                 ws_nfree, kMaxDispatchFreeVars);
    }
    return 0;
  }

  // ── Dispatch LP (small problems only) ────────────────────────────────────
  // Fix integer variables, re-solve continuous dispatch via IPM.
  Eigen::VectorXd b_adj = base_lp.b;
  Eigen::VectorXd beq_adj = base_lp.beq;
  for (int j : ws_fixed) {
    if (std::abs(x0[j]) < 1e-15) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, j); it; ++it)
      b_adj[it.row()] -= it.value() * x0[j];
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, j); it; ++it)
      beq_adj[it.row()] -= it.value() * x0[j];
  }
  std::vector<Eigen::Triplet<double>> a_trips, aeq_trips;
  for (int k = 0; k < ws_nfree; ++k) {
    const int j = ws_free[k];
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.A, j); it; ++it)
      a_trips.emplace_back(it.row(), k, it.value());
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_lp.Aeq, j); it; ++it)
      aeq_trips.emplace_back(it.row(), k, it.value());
  }
  LPModel ws_lp;
  ws_lp.sense = base_lp.sense;
  ws_lp.vars.resize(ws_nfree);
  ws_lp.c = Eigen::VectorXd::Zero(ws_nfree);
  for (int k = 0; k < ws_nfree; ++k) {
    const int j = ws_free[k];
    ws_lp.vars[k] = base_lp.vars[j];
    ws_lp.vars[k].type = VarType::Continuous;
    ws_lp.c[k] = base_lp.c[j];
  }
  Eigen::SparseMatrix<double> Ar(
      static_cast<int>(base_lp.A.rows()), ws_nfree);
  Ar.setFromTriplets(a_trips.begin(), a_trips.end());
  ws_lp.A = std::move(Ar);
  ws_lp.b = b_adj;
  Eigen::SparseMatrix<double> Aeqr(
      static_cast<int>(base_lp.Aeq.rows()), ws_nfree);
  Aeqr.setFromTriplets(aeq_trips.begin(), aeq_trips.end());
  ws_lp.Aeq = std::move(Aeqr);
  ws_lp.beq = beq_adj;
  Eigen::VectorXd x0f(ws_nfree);
  for (int k = 0; k < ws_nfree; ++k) x0f[k] = x0[ws_free[k]];
  IPMLPOptions ws_ipm_opt;
  ws_ipm_opt.max_iter = 50;
  ws_ipm_opt.tol_primal = 1e-6;
  ws_ipm_opt.tol_dual = 1e-6;
  ws_ipm_opt.tol_gap = 1e-6;
  ws_ipm_opt.verbose = false;
  NativeIPMLPAdapter ws_ipm(ws_ipm_opt);
  const auto ws_t0 = std::chrono::steady_clock::now();
  const auto ws_res = ws_ipm.solve_lp(ws_lp, x0f);
  if (ws_res.stats.success &&
      static_cast<int>(ws_res.x.size()) >= ws_nfree) {
    Eigen::VectorXd xr = x0;
    for (int k = 0; k < ws_nfree; ++k) xr[ws_free[k]] = ws_res.x[k];
    xr = clamp_to_bounds(xr, olb, oub);
    for (int j : ws_fixed) xr[j] = x0[j];
    if (satisfies_with_bounds(base_lp, xr, olb, oub, 1e-6)) {
      HighsSolution mip_start;
      mip_start.value_valid = true;
      mip_start.dual_valid = false;
      mip_start.col_value.assign(xr.data(), xr.data() + orig_n);
      highs.setSolution(mip_start);
      if (opt.verbose) {
        const double ws_obj = objective_value(base_lp.c, xr, base_lp.sense);
        const double ws_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - ws_t0)
                .count();
        fmt::print(stderr,
                   "[B&C-STRICT] warm-start injected: obj={:.6g} ({:.1f}ms)\n",
                   ws_obj, ws_ms);
      }
    } else if (opt.verbose) {
      fmt::print(stderr,
                 "[B&C-STRICT] warm-start dispatch feasible but "
                 "fails bounds check; not injected\n");
    }
  } else if (opt.verbose) {
    fmt::print(stderr, "[B&C-STRICT] warm-start dispatch LP {}\n",
               ws_res.stats.success ? "clamped infeasible" : "infeasible");
  }
  return 1;  // consumed one LP solve
}

}  // namespace mipsolvers::engine::detail
#endif  // MIPSOLVERS_HAVE_HIGHS_LIB

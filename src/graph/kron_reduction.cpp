/// src/graph/kron_reduction.cpp
/// =============================
/// Kron (Schur complement) reduction on Y-bus matrices.

#include "hacdcpf/graph/kron_reduction.hpp"

#include <algorithm>
#include <complex>
#include <stdexcept>

#include <Eigen/Dense>

namespace hacdcpf::graph {

namespace {

// ── Partition Y_full into blocks using retained/eliminated index lists ───

struct YBusBlocks {
  Eigen::MatrixXcd Y_aa; // retained × retained
  Eigen::MatrixXcd Y_ab; // retained × eliminated
  Eigen::MatrixXcd Y_ba; // eliminated × retained
  Eigen::MatrixXcd Y_bb; // eliminated × eliminated
};

static YBusBlocks partition_ybus(
    const Eigen::SparseMatrix<std::complex<double>>& Y,
    const std::vector<int>& alpha, // retained
    const std::vector<int>& beta)  // eliminated
{
  const int na = static_cast<int>(alpha.size());
  const int nb = static_cast<int>(beta.size());

  YBusBlocks blk;
  blk.Y_aa.resize(na, na); blk.Y_aa.setZero();
  blk.Y_ab.resize(na, nb); blk.Y_ab.setZero();
  blk.Y_ba.resize(nb, na); blk.Y_ba.setZero();
  blk.Y_bb.resize(nb, nb); blk.Y_bb.setZero();

  // Build fast lookup: global_idx → alpha-pos / beta-pos
  const int n = static_cast<int>(Y.rows());
  std::vector<int> pos(n, -1);
  std::vector<bool> in_alpha(n, false);
  for (int i = 0; i < na; ++i) { pos[alpha[i]] = i; in_alpha[alpha[i]] = true; }
  std::vector<int> beta_pos(n, -1);
  for (int i = 0; i < nb; ++i) beta_pos[beta[i]] = i;

  for (int k = 0; k < Y.outerSize(); ++k) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(Y, k); it; ++it) {
      int r = static_cast<int>(it.row());
      int c = static_cast<int>(it.col());
      auto v = it.value();

      bool r_alpha = in_alpha[r];
      bool c_alpha = in_alpha[c];

      if (r_alpha && c_alpha) {
        blk.Y_aa(pos[r], pos[c]) += v;
      } else if (r_alpha && !c_alpha) {
        int ci = beta_pos[c];
        if (ci >= 0) blk.Y_ab(pos[r], ci) += v;
      } else if (!r_alpha && c_alpha) {
        int ri = beta_pos[r];
        if (ri >= 0) blk.Y_ba(ri, pos[c]) += v;
      } else {
        int ri = beta_pos[r];
        int ci = beta_pos[c];
        if (ri >= 0 && ci >= 0) blk.Y_bb(ri, ci) += v;
      }
    }
  }
  return blk;
}

static int count_nnz(const Eigen::MatrixXcd& M,
                     double tol = 1e-15) {
  int cnt = 0;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i,j)) > tol) ++cnt;
  return cnt;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────
// apply_kron_reduction
// ─────────────────────────────────────────────────────────────────────

KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<int>&                          retained,
    const std::vector<int>&                          eliminated,
    double max_fill_ratio)
{
  KronReductionResult res;
  res.kron_data.retained_bus_indices  = retained;
  res.kron_data.eliminated_bus_indices = eliminated;

  if (eliminated.empty()) {
    // Nothing to do: Y_red = Y_aa (retained submatrix)
    res.kron_data.valid = true;
    // Build Y_reduced from the retained rows/cols
    const int na = static_cast<int>(retained.size());
    res.Y_reduced.resize(na, na);
    std::vector<int> pos(Y_full.rows(), -1);
    for (int i = 0; i < na; ++i) pos[retained[i]] = i;
    std::vector<Eigen::Triplet<std::complex<double>>> trips;
    for (int k = 0; k < Y_full.outerSize(); ++k) {
      for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(Y_full, k); it; ++it) {
        int r = pos[it.row()];
        int c = pos[it.col()];
        if (r >= 0 && c >= 0) trips.emplace_back(r, c, it.value());
      }
    }
    res.Y_reduced.setFromTriplets(trips.begin(), trips.end());
    return res;
  }

  // Partition
  auto blk = partition_ybus(Y_full, retained, eliminated);

  // Check Y_bb is non-singular via LU
  Eigen::FullPivLU<Eigen::MatrixXcd> lu(blk.Y_bb);
  if (!lu.isInvertible()) {
    Diagnostic d;
    d.code    = DiagCode::KronSingularYbb;
    d.message = "Y_ββ is singular; Kron reduction aborted.";
    res.diagnostics.push_back(d);
    return res;
  }

  // Schur complement: Y_red = Y_aa - Y_ab * Y_bb^{-1} * Y_ba
  // Store Y_bb^{-1} * Y_ba for voltage recovery
  Eigen::MatrixXcd Ybb_inv_Yba = lu.solve(blk.Y_ba); // nb × na
  Eigen::MatrixXcd Yab_Ybb_inv = blk.Y_ab * lu.inverse(); // na × nb

  Eigen::MatrixXcd Y_red_dense = blk.Y_aa - blk.Y_ab * Ybb_inv_Yba;

  // Fill-in check
  const int nnz_orig = static_cast<int>(Y_full.nonZeros());
  const int nnz_red  = count_nnz(Y_red_dense);
  res.fill_ratio = (nnz_orig > 0) ?
      static_cast<double>(nnz_red) / static_cast<double>(nnz_orig) : 1.0;

  if (res.fill_ratio > max_fill_ratio) {
    Diagnostic d;
    d.code    = DiagCode::KronFillInTooLarge;
    d.message = "Kron fill-in ratio " + std::to_string(res.fill_ratio) +
                " exceeds limit " + std::to_string(max_fill_ratio) +
                "; reduction aborted.";
    res.diagnostics.push_back(d);
    return res;
  }

  // Convert dense Y_red to sparse
  const int na = static_cast<int>(retained.size());
  std::vector<Eigen::Triplet<std::complex<double>>> trips;
  trips.reserve(nnz_red);
  for (int i = 0; i < na; ++i)
    for (int j = 0; j < na; ++j)
      if (std::abs(Y_red_dense(i,j)) > 1e-15)
        trips.emplace_back(i, j, Y_red_dense(i,j));
  res.Y_reduced.resize(na, na);
  res.Y_reduced.setFromTriplets(trips.begin(), trips.end());

  // Store KronData
  res.kron_data.Ybb_inv_Yba = Ybb_inv_Yba;
  res.kron_data.Yab_Ybb_inv = Yab_Ybb_inv;
  res.kron_data.valid = true;

  return res;
}

// ─────────────────────────────────────────────────────────────────────
// apply_kron_reduction_with_injection
// ─────────────────────────────────────────────────────────────────────

KronReductionResult apply_kron_reduction_with_injection(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const Eigen::VectorXcd&                          I_full,
    const std::vector<int>&                          retained,
    const std::vector<int>&                          eliminated,
    double max_fill_ratio)
{
  auto res = apply_kron_reduction(Y_full, retained, eliminated, max_fill_ratio);
  if (!res.kron_data.valid) return res;

  // I_red = I_alpha - Y_ab * Y_bb^{-1} * I_beta
  const int na = static_cast<int>(retained.size());
  const int nb = static_cast<int>(eliminated.size());

  Eigen::VectorXcd I_alpha(na), I_beta(nb);
  for (int i = 0; i < na; ++i) I_alpha(i) = I_full(retained[i]);
  for (int i = 0; i < nb; ++i) I_beta(i)  = I_full(eliminated[i]);

  // Y_ab * Y_bb_inv already stored as Yab_Ybb_inv (na × nb)
  res.I_reduced = I_alpha - res.kron_data.Yab_Ybb_inv * I_beta;

  return res;
}

// ─────────────────────────────────────────────────────────────────────
// recover_eliminated_voltages
// ─────────────────────────────────────────────────────────────────────

Eigen::VectorXcd recover_eliminated_voltages(
    const KronData&         kron_data,
    const Eigen::VectorXcd& V_alpha)
{
  if (!kron_data.valid || kron_data.Ybb_inv_Yba.rows() == 0)
    return Eigen::VectorXcd();

  // V_beta = -Y_bb^{-1} * Y_ba * V_alpha
  return -(kron_data.Ybb_inv_Yba * V_alpha);
}

}  // namespace hacdcpf::graph

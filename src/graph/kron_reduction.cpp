/// src/graph/kron_reduction.cpp
/// =============================
/// Kron (Schur complement) reduction on Y-bus matrices.

#include "hacdcpf/graph/kron_reduction.hpp"

#include <algorithm>
#include <complex>
#include <stdexcept>
#include <unordered_map>

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

static KronReductionResult apply_kron_reduction_impl(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<int>&                          retained,
    const std::vector<int>&                          eliminated,
    double max_fill_ratio,
    const Eigen::VectorXcd*                          I_full)
{
  KronReductionResult res;
  res.kron_data.retained_bus_indices  = retained;
  res.kron_data.eliminated_bus_indices = eliminated;

  const int n = static_cast<int>(Y_full.rows());
  std::vector<int> membership(static_cast<std::size_t>(std::max(0, n)), 0);
  const auto valid_partition = [&]() {
    if (Y_full.rows() != Y_full.cols() || max_fill_ratio < 0.0) return false;
    if (I_full != nullptr && I_full->size() != n) return false;
    for (int index : retained) {
      if (index < 0 || index >= n || membership[index] != 0) return false;
      membership[index] = 1;
    }
    for (int index : eliminated) {
      if (index < 0 || index >= n || membership[index] != 0) return false;
      membership[index] = 2;
    }
    return retained.size() + eliminated.size() == static_cast<std::size_t>(n);
  };
  if (!valid_partition()) {
    res.diagnostics.push_back(
        {DiagCode::Error,
         "Kron reduction requires a square matrix, a complete disjoint "
         "in-range partition, a matching injection vector, and a "
         "non-negative fill limit."});
    return res;
  }

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
    if (I_full != nullptr) {
      res.I_reduced.resize(na);
      for (int i = 0; i < na; ++i) res.I_reduced[i] = (*I_full)[retained[i]];
      res.kron_data.has_current_injection = true;
    }
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

  // Kron (1939), partitioned-network elimination; derivation in
  // docs/modules/graph/chapters/source_equivalent_algorithms.tex.
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
  res.kron_data.Ybb_inv_Ibeta = Eigen::VectorXcd::Zero(
      static_cast<int>(eliminated.size()));
  if (I_full != nullptr) {
    const int na = static_cast<int>(retained.size());
    const int nb = static_cast<int>(eliminated.size());
    Eigen::VectorXcd I_alpha(na), I_beta(nb);
    for (int i = 0; i < na; ++i) I_alpha[i] = (*I_full)[retained[i]];
    for (int i = 0; i < nb; ++i) I_beta[i] = (*I_full)[eliminated[i]];
    res.kron_data.Ybb_inv_Ibeta = lu.solve(I_beta);
    res.kron_data.has_current_injection = true;
    res.I_reduced = I_alpha - blk.Y_ab * res.kron_data.Ybb_inv_Ibeta;
  }
  res.kron_data.valid = true;

  return res;
}

KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<int>&                          retained,
    const std::vector<int>&                          eliminated,
    double max_fill_ratio) {
  return apply_kron_reduction_impl(Y_full, retained, eliminated,
                                   max_fill_ratio, nullptr);
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
  return apply_kron_reduction_impl(Y_full, retained, eliminated,
                                   max_fill_ratio, &I_full);
}

namespace {

void attach_kron_mapping(KronReductionResult& result,
                         const std::vector<BusRef>& bus_order,
                         const std::vector<int>& retained,
                         const std::vector<int>& eliminated) {
  if (!result.kron_data.valid) return;
  for (const auto& bus : bus_order) {
    result.mapping.original_to_reduced_buses[bus] = bus;
  }
  KronReductionRecord record;
  for (int position : retained) {
    record.retained_buses.push_back(bus_order[position]);
  }
  for (int position : eliminated) {
    const BusRef bus = bus_order[position];
    record.eliminated_buses.push_back(bus);
    result.mapping.original_to_reduced_buses[bus] = {bus.domain, -1};
  }
  result.mapping.kron_records.push_back(std::move(record));
  rebuild_reduction_reverse_maps(result.mapping);
}

bool valid_bus_order(const std::vector<BusRef>& bus_order,
                     int matrix_size) {
  if (bus_order.size() != static_cast<std::size_t>(matrix_size)) return false;
  std::unordered_map<BusRef, int, BusRefHash> seen;
  for (int position = 0; position < matrix_size; ++position) {
    if (!bus_order[position].valid() ||
        !seen.emplace(bus_order[position], position).second) {
      return false;
    }
  }
  return true;
}

}  // namespace

KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<BusRef>&                        bus_order,
    const std::vector<int>&                           retained,
    const std::vector<int>&                           eliminated,
    double max_fill_ratio) {
  if (!valid_bus_order(bus_order, static_cast<int>(Y_full.rows()))) {
    KronReductionResult result;
    result.diagnostics.push_back(
        {DiagCode::Error,
         "Kron bus ordering must contain one unique valid BusRef per row."});
    return result;
  }
  auto result = apply_kron_reduction_impl(
      Y_full, retained, eliminated, max_fill_ratio, nullptr);
  attach_kron_mapping(result, bus_order, retained, eliminated);
  return result;
}

KronReductionResult apply_kron_reduction_with_injection(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const Eigen::VectorXcd&                          I_full,
    const std::vector<BusRef>&                        bus_order,
    const std::vector<int>&                           retained,
    const std::vector<int>&                           eliminated,
    double max_fill_ratio) {
  if (!valid_bus_order(bus_order, static_cast<int>(Y_full.rows()))) {
    KronReductionResult result;
    result.diagnostics.push_back(
        {DiagCode::Error,
         "Kron bus ordering must contain one unique valid BusRef per row."});
    return result;
  }
  auto result = apply_kron_reduction_impl(
      Y_full, retained, eliminated, max_fill_ratio, &I_full);
  attach_kron_mapping(result, bus_order, retained, eliminated);
  return result;
}

KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<BusRef>&                        bus_order,
    const ReductionAction&                            action) {
  KronReductionResult error;
  if (action.type != ReductionActionType::KronEliminate) {
    error.diagnostics.push_back(
        {DiagCode::Error, "Kron executor requires a KronEliminate action."});
    return error;
  }
  std::unordered_map<BusRef, int, BusRefHash> position;
  for (int index = 0; index < static_cast<int>(bus_order.size()); ++index) {
    position.emplace(bus_order[index], index);
  }
  std::vector<int> retained;
  std::vector<int> eliminated;
  for (const auto& bus : action.retained_buses) {
    const auto it = position.find(bus);
    if (it == position.end()) {
      error.diagnostics.push_back(
          {DiagCode::Error, "Kron retained BusRef is absent from bus_order."});
      return error;
    }
    retained.push_back(it->second);
  }
  for (const auto& bus : action.eliminated_buses) {
    const auto it = position.find(bus);
    if (it == position.end()) {
      error.diagnostics.push_back(
          {DiagCode::Error, "Kron eliminated BusRef is absent from bus_order."});
      return error;
    }
    eliminated.push_back(it->second);
  }
  return apply_kron_reduction(Y_full, bus_order, retained, eliminated,
                              action.max_fill_ratio);
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

  if (V_alpha.size() != kron_data.Ybb_inv_Yba.cols()) {
    throw std::invalid_argument(
        "Kron retained-voltage dimension does not match recovery operator");
  }

  // Kron (1939), back-substitution of the eliminated block; derivation in
  // docs/modules/graph/chapters/source_equivalent_algorithms.tex.
  Eigen::VectorXcd recovered = -(kron_data.Ybb_inv_Yba * V_alpha);
  if (kron_data.has_current_injection) {
    if (kron_data.Ybb_inv_Ibeta.size() != recovered.size()) {
      throw std::invalid_argument("Kron injection-correction dimension mismatch");
    }
    recovered += kron_data.Ybb_inv_Ibeta;
  }
  return recovered;
}

}  // namespace hacdcpf::graph

#pragma once

/// graph/kron_reduction.hpp
/// =========================
/// Kron reduction (Schur complement) for passive interior nodes.
///
/// Theory:
///   Partition Y-bus into retained (α) and eliminated (β) sets.
///   If I_β = 0 (no injection at eliminated nodes):
///
///     Y_red = Y_αα - Y_αβ · Y_ββ⁻¹ · Y_βα
///
///   Voltage recovery:
///     V_β = -Y_ββ⁻¹ · Y_βα · V_α
///
///   For nodes with constant-current injection I_β ≠ 0:
///     Y_red  = Y_αα - Y_αβ · Y_ββ⁻¹ · Y_βα          (admittance unchanged)
///     I_red  = I_α  - Y_αβ · Y_ββ⁻¹ · I_β            (modified injection)
///
/// ⚠ Warning: Kron reduction is exact only for passive nodes and linear
///   networks.  For nonlinear AC power flow with constant-power loads,
///   it is an approximation that depends on the operating point.
///
/// ⚠ fill-in guard: if nnz(Y_red) / nnz(Y_orig) > max_fill_ratio,
///   the reduction is aborted and an error diagnostic is returned.

#include <complex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/graph/reduction_mapping.hpp"
#include "hacdcpf/graph/reduction_plan.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// KronData — stored for voltage recovery after solve
// ═══════════════════════════════════════════════════════════════════════

struct KronData {
  /// Y_ββ⁻¹ · Y_βα  (dense, shape: |β| × |α|)
  Eigen::MatrixXcd  Ybb_inv_Yba;

  /// For constant-current injection case:
  /// Y_αβ · Y_ββ⁻¹  (dense, shape: |α| × |β|)
  Eigen::MatrixXcd  Yab_Ybb_inv;

  /// Y_ββ⁻¹ I_β. Zero for the passive variant; populated by the
  /// constant-current-injection variant.
  Eigen::VectorXcd  Ybb_inv_Ibeta;

  std::vector<int> retained_bus_indices;   ///< Indices in original Y-bus
  std::vector<int> eliminated_bus_indices; ///< Indices in original Y-bus

  bool valid{false};
  bool has_current_injection{false};
};

// ═══════════════════════════════════════════════════════════════════════
// Kron reduction result
// ═══════════════════════════════════════════════════════════════════════

struct KronReductionResult {
  /// Reduced Y-bus (|α| × |α| sparse complex)
  Eigen::SparseMatrix<std::complex<double>> Y_reduced;

  /// Modified injection vector (only populated for I_β ≠ 0 variant)
  Eigen::VectorXcd I_reduced;

  KronData kron_data;
  ReductionMapping mapping;

  double fill_ratio{1.0};
  std::vector<Diagnostic> diagnostics;
};

// ═══════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════

/// Kron reduce \p Y_full by eliminating \p eliminated_indices.
/// \p retained_indices    Indices of rows/cols to keep (α set).
/// \p eliminated_indices  Indices of rows/cols to eliminate (β set).
///                        Must satisfy: I_β = 0 for lossless reduction.
/// \p max_fill_ratio      Abort if fill exceeds this ratio.
KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<int>&                          retained_indices,
    const std::vector<int>&                          eliminated_indices,
    double max_fill_ratio = 2.0);

/// Extended Kron: nodes in β have known constant-current injection I_beta.
/// Returns Y_red (same Schur complement) and modified injection I_red.
KronReductionResult apply_kron_reduction_with_injection(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const Eigen::VectorXcd&                          I_full,
    const std::vector<int>&                          retained_indices,
    const std::vector<int>&                          eliminated_indices,
    double max_fill_ratio = 2.0);

/// Domain-qualified overloads. bus_order maps original matrix positions to
/// stable identities and therefore allow the result to carry a complete Kron
/// reduction certificate.
KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<BusRef>&                        bus_order,
    const std::vector<int>&                           retained_indices,
    const std::vector<int>&                           eliminated_indices,
    double max_fill_ratio = 2.0);

KronReductionResult apply_kron_reduction_with_injection(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const Eigen::VectorXcd&                          I_full,
    const std::vector<BusRef>&                        bus_order,
    const std::vector<int>&                           retained_indices,
    const std::vector<int>&                           eliminated_indices,
    double max_fill_ratio = 2.0);

/// Execute a domain-qualified Kron plan action against a matrix whose row
/// ordering is declared by bus_order. The action's max_fill_ratio is used.
KronReductionResult apply_kron_reduction(
    const Eigen::SparseMatrix<std::complex<double>>& Y_full,
    const std::vector<BusRef>&                        bus_order,
    const ReductionAction&                            action);

/// Recover eliminated node voltages from retained node voltages.
/// Uses: V_β = -Y_ββ⁻¹ · Y_βα · V_α  (or with I_β correction).
Eigen::VectorXcd recover_eliminated_voltages(
    const KronData&         kron_data,
    const Eigen::VectorXcd& V_alpha);

}  // namespace hacdcpf::graph

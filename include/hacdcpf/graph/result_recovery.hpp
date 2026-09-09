#pragma once

/// graph/result_recovery.hpp
/// ==========================
/// Reverse-recovery of eliminated node voltages and branch flows
/// after solving on a reduced network.
///
/// Recovery rules by reduction type:
///
/// Switch contraction (i,j → K):
///   V_i = V_K,  θ_i = θ_K  (exact)
///
/// Series reduction (i--j--k, j eliminated):
///   I_ik = (V_i - V_k) / Z_ik_eq
///   V_j  = V_i - Z_ij · I_ik   (exact for passive j)
///
/// Pendant reduction (i--j, j eliminated with load):
///   Iterative: V_j^(t+1) = V_i - Z_ij · (S_j / V_j^(t))^*
///   Init: V_j^(0) = V_i
///
/// Kron reduction:
///   V_β = Y_ββ⁻¹ I_β - Y_ββ⁻¹ · Y_βα · V_α   (via stored KronData)

#include <complex>
#include <unordered_map>
#include <vector>

#include "hacdcpf/graph/kron_reduction.hpp"
#include "hacdcpf/graph/reduction_mapping.hpp"
#include "hacdcpf/graph/switch_contraction.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Per-bus result (complex voltage)
// ═══════════════════════════════════════════════════════════════════════

struct FullNetworkVoltages {
  /// bus_id → complex voltage in pu  (V = Vm · e^{jθ})
  /// @note Legacy map: written for all buses; when AC and DC share a bus ID
  ///   the AC voltage wins (last-write semantics preserved from earlier
  ///   releases).  For hybrid systems with same-numeric-ID AC/DC buses prefer
  ///   the domain-qualified maps below.
  std::unordered_map<int, std::complex<double>> bus_voltage;

  /// Domain-safe voltages: bus_id → voltage pu for AC buses only.
  std::unordered_map<int, std::complex<double>> ac_bus_voltage;
  /// Domain-safe voltages: bus_id → voltage pu for DC buses only.
  std::unordered_map<int, std::complex<double>> dc_bus_voltage;

  /// Convenience accessors (use the legacy map; suitable for AC-only or
  /// numerically-disjoint systems)
  double vm_pu(int bus_id, double default_val = 1.0) const;
  double va_deg(int bus_id, double default_val = 0.0) const;
};

// ═══════════════════════════════════════════════════════════════════════
// Recovery options
// ═══════════════════════════════════════════════════════════════════════

struct RecoveryOptions {
  int    max_pendant_iterations{5};
  double pendant_convergence_tol{1e-6};  ///< |ΔV| < tol → converged
};

// ═══════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════

/// Expand switch-contracted super-node voltages back to all original buses.
/// V_i = V_K for every bus i that was merged into super-node K.
/// @deprecated  Prefer the ContractionResult overload below; this legacy
///   variant is only correct when AC and DC bus IDs are numerically disjoint.
void recover_switch_contracted_buses(
    FullNetworkVoltages&    voltages,
    const std::unordered_map<int, int>&                bus_to_super,
    const std::unordered_map<int, std::vector<int>>&   super_to_buses);

/// Domain-safe variant: uses the domain-qualified maps in ContractionResult
/// so that an AC bus and a DC bus that share the same numeric ID are never
/// confused.  This is the preferred overload for hybrid AC/DC systems.
/// Missing qualified values remain unavailable when legacy ownership is
/// ambiguous; an AC voltage is never substituted for a missing DC voltage.
void recover_switch_contracted_buses(
    FullNetworkVoltages&     voltages,
    const ContractionResult& contraction);

/// Recover voltage at series-eliminated degree-2 nodes.
/// For each record (i--j--k, j eliminated):
///   V_j = V_i - Z_ij · (V_i - V_k) / Z_ik_eq
void recover_series_reduced_buses(
    FullNetworkVoltages&          voltages,
    const ReductionMapping&       mapping,
    const HybridPowerSystem&      original_system);

/// Recover voltage at Kron-eliminated passive nodes.
/// V_β = Y_ββ⁻¹ I_β - Y_ββ⁻¹ · Y_βα · V_α (uses pre-computed KronData)
/// Legacy single-domain overload. Bus IDs are resolved through bus_voltage.
void recover_kron_eliminated_buses(
    FullNetworkVoltages&     voltages,
    const KronData&          kron_data,
    const std::vector<int>&  bus_ids_alpha,
    const std::vector<int>&  bus_ids_beta);

/// Domain-qualified overload for hybrid AC/DC systems.
void recover_kron_eliminated_buses(
    FullNetworkVoltages&     voltages,
    const KronData&          kron_data,
    const std::vector<BusRef>& bus_refs_alpha,
    const std::vector<BusRef>& bus_refs_beta);

/// Recover voltage at pendant (leaf) eliminated nodes iteratively.
/// V_j^(t+1) = V_i - Z_ij · (S_j^* / V_j^(t)^*)
void recover_pendant_buses(
    FullNetworkVoltages&      voltages,
    const ReductionMapping&   mapping,
    const HybridPowerSystem&  original_system,
    const RecoveryOptions&    opts = {});

}  // namespace hacdcpf::graph

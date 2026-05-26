/// src/graph/result_recovery.cpp
/// ==============================
/// Reverse recovery of eliminated node voltages after solving on a
/// reduced network.

#include "hacdcpf/graph/result_recovery.hpp"

#include <cmath>
#include <complex>

#include "hacdcpf/graph/switch_contraction.hpp"

namespace hacdcpf::graph {

// ─────────────────────────────────────────────────────────────────────
// FullNetworkVoltages helpers
// ─────────────────────────────────────────────────────────────────────

double FullNetworkVoltages::vm_pu(int bus_id, double default_val) const {
  auto it = bus_voltage.find(bus_id);
  if (it == bus_voltage.end()) return default_val;
  return std::abs(it->second);
}

double FullNetworkVoltages::va_deg(int bus_id, double default_val) const {
  auto it = bus_voltage.find(bus_id);
  if (it == bus_voltage.end()) return default_val;
  return std::arg(it->second) * (180.0 / M_PI);
}

// ─────────────────────────────────────────────────────────────────────
// recover_switch_contracted_buses
// ─────────────────────────────────────────────────────────────────────

void recover_switch_contracted_buses(
    FullNetworkVoltages&    voltages,
    const std::unordered_map<int, int>&              bus_to_super,
    const std::unordered_map<int, std::vector<int>>& super_to_buses)
{
  // For every super-node that has a known voltage, propagate to all members
  for (const auto& [sup, members] : super_to_buses) {
    auto it = voltages.bus_voltage.find(sup);
    if (it == voltages.bus_voltage.end()) continue;
    auto V_sup = it->second;
    for (int bid : members) {
      voltages.bus_voltage[bid] = V_sup;
    }
  }
  // Also handle original buses not in any super-node
  for (const auto& [orig, sup] : bus_to_super) {
    if (voltages.bus_voltage.count(orig) == 0) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end())
        voltages.bus_voltage[orig] = it->second;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// recover_switch_contracted_buses (ContractionResult overload)
// ─────────────────────────────────────────────────────────────────────

void recover_switch_contracted_buses(
    FullNetworkVoltages&     voltages,
    const ContractionResult& contraction)
{
  // AC recovery: propagate AC super-node voltages to all AC member buses
  for (const auto& [sup, members] : contraction.ac_super_to_buses) {
    auto it = voltages.bus_voltage.find(sup);
    if (it == voltages.bus_voltage.end()) continue;
    auto V_sup = it->second;
    for (int bid : members)
      voltages.bus_voltage[bid] = V_sup;
  }
  for (const auto& [orig, sup] : contraction.ac_bus_to_super) {
    if (voltages.bus_voltage.count(orig) == 0) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end())
        voltages.bus_voltage[orig] = it->second;
    }
  }

  // DC recovery: propagate DC super-node voltages to all DC member buses
  for (const auto& [sup, members] : contraction.dc_super_to_buses) {
    auto it = voltages.bus_voltage.find(sup);
    if (it == voltages.bus_voltage.end()) continue;
    auto V_sup = it->second;
    for (int bid : members)
      voltages.bus_voltage[bid] = V_sup;
  }
  for (const auto& [orig, sup] : contraction.dc_bus_to_super) {
    if (voltages.bus_voltage.count(orig) == 0) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end())
        voltages.bus_voltage[orig] = it->second;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// recover_series_reduced_buses
// ─────────────────────────────────────────────────────────────────────

void recover_series_reduced_buses(
    FullNetworkVoltages&     voltages,
    const ReductionMapping&  mapping,
    const HybridPowerSystem& original_system)
{
  for (const auto& rec : mapping.series_records) {
    auto it_i = voltages.bus_voltage.find(rec.from_bus_id);
    auto it_k = voltages.bus_voltage.find(rec.to_bus_id);
    if (it_i == voltages.bus_voltage.end() ||
        it_k == voltages.bus_voltage.end()) continue;

    std::complex<double> V_i = it_i->second;
    std::complex<double> V_k = it_k->second;

    // Z_ik_eq = Z_ij + Z_jk  (both stored as r_eq, x_eq in record)
    // Use the equivalent Z to find current: I_ik = (V_i - V_k) / Z_ik_eq
    double base_mva = original_system.base_mva > 0 ? original_system.base_mva : 100.0;
    (void)base_mva; // voltages are in pu, impedance already in pu

    // Z_ij: from the first original branch
    // We stored r_eq = R_ij + R_jk, x_eq = X_ij + X_jk.
    // But to recover V_j we need Z_ij specifically.
    // Look up Z_ij from original branches using the first ID in original_branch_ids
    double r_ij = 0.0, x_ij = 0.0;
    double r_eq = rec.r_eq, x_eq = rec.x_eq;

    if (!rec.original_branch_ids.empty()) {
      int eid_ij = rec.original_branch_ids[0];
      for (const auto& br : original_system.ac.branches) {
        if (br.index == eid_ij) {
          r_ij = br.r_pu;
          x_ij = br.x_pu;
          break;
        }
      }
    }

    std::complex<double> Z_ik{r_eq, x_eq};
    std::complex<double> Z_ij{r_ij, x_ij};

    std::complex<double> I_ik = (std::abs(Z_ik) > 1e-15) ?
        (V_i - V_k) / Z_ik : std::complex<double>{0.0, 0.0};

    // V_j = V_i - Z_ij * I_ik
    std::complex<double> V_j = V_i - Z_ij * I_ik;
    voltages.bus_voltage[rec.eliminated_bus_id] = V_j;
  }
}

// ─────────────────────────────────────────────────────────────────────
// recover_kron_eliminated_buses
// ─────────────────────────────────────────────────────────────────────

void recover_kron_eliminated_buses(
    FullNetworkVoltages&     voltages,
    const KronData&          kron_data,
    const std::vector<int>&  bus_ids_alpha,
    const std::vector<int>&  bus_ids_beta)
{
  if (!kron_data.valid) return;
  const int na = static_cast<int>(bus_ids_alpha.size());
  const int nb = static_cast<int>(bus_ids_beta.size());

  // Build V_alpha vector
  Eigen::VectorXcd V_alpha(na);
  for (int i = 0; i < na; ++i) {
    auto it = voltages.bus_voltage.find(bus_ids_alpha[i]);
    V_alpha(i) = (it != voltages.bus_voltage.end()) ?
        it->second : std::complex<double>{1.0, 0.0};
  }

  // Recover: V_beta = -Y_bb^{-1} * Y_ba * V_alpha
  Eigen::VectorXcd V_beta = recover_eliminated_voltages(kron_data, V_alpha);

  for (int i = 0; i < nb && i < static_cast<int>(V_beta.size()); ++i) {
    voltages.bus_voltage[bus_ids_beta[i]] = V_beta(i);
  }
}

// ─────────────────────────────────────────────────────────────────────
// recover_pendant_buses
// ─────────────────────────────────────────────────────────────────────

void recover_pendant_buses(
    FullNetworkVoltages&      voltages,
    const ReductionMapping&   mapping,
    const HybridPowerSystem&  original_system,
    const RecoveryOptions&    opts)
{
  double base_mva = original_system.base_mva > 0 ? original_system.base_mva : 100.0;

  for (const auto& rec : mapping.pendant_records) {
    auto it_parent = voltages.bus_voltage.find(rec.parent_bus_id);
    if (it_parent == voltages.bus_voltage.end()) continue;

    std::complex<double> V_i = it_parent->second;

    // Get Z_ij from original branches
    double r_ij = 0.0, x_ij = 0.0;
    for (const auto& br : original_system.ac.branches) {
      if (br.index == rec.branch_id) {
        r_ij = br.r_pu;
        x_ij = br.x_pu;
        break;
      }
    }
    std::complex<double> Z_ij{r_ij, x_ij};

    // Collect load S_j (in pu)
    double p_j_mw = 0.0, q_j_mvar = 0.0;
    for (const auto& bus : original_system.ac.buses) {
      if (bus.index == rec.eliminated_bus_id) {
        p_j_mw   += bus.pd_mw;
        q_j_mvar += bus.qd_mvar;
        break;
      }
    }
    for (const auto& ld : original_system.ac.loads) {
      if (ld.bus == rec.eliminated_bus_id && ld.in_service) {
        p_j_mw   += ld.p_mw;
        q_j_mvar += ld.q_mvar;
      }
    }
    std::complex<double> S_j{p_j_mw / base_mva, q_j_mvar / base_mva};

    // Iterative recovery: V_j^(t+1) = V_i - Z_ij * (S_j^* / V_j^(t)^*)
    std::complex<double> V_j = V_i; // initial guess: flat start = V_i
    for (int iter = 0; iter < opts.max_pendant_iterations; ++iter) {
      if (std::abs(V_j) < 1e-12) break; // degenerate
      std::complex<double> I_ij = std::conj(S_j) / std::conj(V_j);
      std::complex<double> V_j_new = V_i - Z_ij * I_ij;
      if (std::abs(V_j_new - V_j) < opts.pendant_convergence_tol) {
        V_j = V_j_new;
        break;
      }
      V_j = V_j_new;
    }
    voltages.bus_voltage[rec.eliminated_bus_id] = V_j;
  }
}

}  // namespace hacdcpf::graph

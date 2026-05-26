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
  // AC recovery: propagate AC super-node voltages to all AC member buses.
  // Prefer the domain-qualified map (written by AC solvers) before falling
  // back to the legacy map, so that callers using only the new API are handled
  // symmetrically with callers that only write legacy bus_voltage.
  for (const auto& [sup, members] : contraction.ac_super_to_buses) {
    std::complex<double> V_sup;
    bool found = false;
    {
      auto it = voltages.ac_bus_voltage.find(sup);
      if (it != voltages.ac_bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) continue;
    for (int bid : members) {
      voltages.ac_bus_voltage[bid] = V_sup;
      voltages.bus_voltage[bid]    = V_sup;  // legacy map (AC wins on collision)
    }
  }
  for (const auto& [orig, sup] : contraction.ac_bus_to_super) {
    if (voltages.ac_bus_voltage.count(orig) > 0) continue;
    std::complex<double> V_sup;
    bool found = false;
    {
      auto it = voltages.ac_bus_voltage.find(sup);
      if (it != voltages.ac_bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) continue;
    voltages.ac_bus_voltage[orig] = V_sup;
    voltages.bus_voltage[orig]    = V_sup;
  }

  // DC recovery: propagate DC super-node voltages to all DC member buses.
  // Lookup priority: dc_bus_voltage (set by DC solver) → legacy bus_voltage
  // (set by any prior recovery).  Writes to dc_bus_voltage AND to the legacy
  // bus_voltage so that DC-only / disjoint-ID callers still see results there.
  // When an AC bus shares the same integer ID, the AC write above already
  // populated bus_voltage; we skip the DC overwrite in that case to let AC win.
  for (const auto& [sup, members] : contraction.dc_super_to_buses) {
    // Resolve the super-node voltage: prefer the domain-qualified map, fall
    // back to the legacy map.  Use separate iterator variables to avoid
    // comparing iterators from different containers (UB).
    std::complex<double> V_sup;
    bool found = false;
    {
      auto it = voltages.dc_bus_voltage.find(sup);
      if (it != voltages.dc_bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) continue;
    for (int bid : members) {
      voltages.dc_bus_voltage[bid] = V_sup;
      // Write to legacy map only if no AC bus has already claimed this key.
      if (voltages.ac_bus_voltage.count(bid) == 0)
        voltages.bus_voltage[bid] = V_sup;
    }
  }
  for (const auto& [orig, sup] : contraction.dc_bus_to_super) {
    if (voltages.dc_bus_voltage.count(orig) > 0) continue;
    std::complex<double> V_sup;
    bool found = false;
    {
      auto it = voltages.dc_bus_voltage.find(sup);
      if (it != voltages.dc_bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) {
      auto it = voltages.bus_voltage.find(sup);
      if (it != voltages.bus_voltage.end()) { V_sup = it->second; found = true; }
    }
    if (!found) continue;
    voltages.dc_bus_voltage[orig] = V_sup;
    if (voltages.ac_bus_voltage.count(orig) == 0)
      voltages.bus_voltage[orig] = V_sup;
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
    // Use the domain recorded at reduction time — this is the only correct
    // way when AC bus N and DC bus N share the same numeric ID.
    const bool is_dc = (rec.domain == NodeDomain::DC);

    // ── Resolve endpoint voltages from the domain-appropriate map ───────
    // DC lookup: prefer dc_bus_voltage; fall back to legacy bus_voltage only
    // when no AC bus has claimed that key (prevents reading AC voltage for DC
    // buses that happen to share the same integer ID as an AC bus).
    auto lookup_dc_v = [&](int bid, std::complex<double>& out) -> bool {
      auto it = voltages.dc_bus_voltage.find(bid);
      if (it != voltages.dc_bus_voltage.end()) { out = it->second; return true; }
      if (voltages.ac_bus_voltage.count(bid) == 0) {
        auto jt = voltages.bus_voltage.find(bid);
        if (jt != voltages.bus_voltage.end()) { out = jt->second; return true; }
      }
      return false;
    };
    std::complex<double> V_i, V_k;
    if (is_dc) {
      if (!lookup_dc_v(rec.from_bus_id, V_i) || !lookup_dc_v(rec.to_bus_id, V_k)) continue;
    } else {
      // AC: prefer domain-qualified map, fall back to legacy.
      auto try_ac = [&](int bid, std::complex<double>& out) -> bool {
        auto it = voltages.ac_bus_voltage.find(bid);
        if (it != voltages.ac_bus_voltage.end()) { out = it->second; return true; }
        auto jt = voltages.bus_voltage.find(bid);
        if (jt != voltages.bus_voltage.end()) { out = jt->second; return true; }
        return false;
      };
      if (!try_ac(rec.from_bus_id, V_i) || !try_ac(rec.to_bus_id, V_k)) continue;
    }

    // ── Look up segment impedance from domain-correct branch table ───────
    double r_eq = rec.r_eq, x_eq = rec.x_eq;
    double r_ij = 0.0, x_ij = 0.0;
    if (!rec.original_branch_ids.empty()) {
      int eid_ij = rec.original_branch_ids[0];
      if (is_dc) {
        // DC branches have only r_pu (no reactance).
        for (const auto& br : original_system.dc.branches) {
          if (br.index == eid_ij) { r_ij = br.r_pu; break; }
        }
        x_ij = 0.0;
        x_eq = 0.0;  // DC: no reactive component in the equivalent either
      } else {
        for (const auto& br : original_system.ac.branches) {
          if (br.index == eid_ij) { r_ij = br.r_pu; x_ij = br.x_pu; break; }
        }
      }
    }

    // V_j = V_i - Z_ij * I_ik,  I_ik = (V_i - V_k) / Z_ik_eq
    std::complex<double> Z_ik{r_eq, x_eq};
    std::complex<double> Z_ij{r_ij, x_ij};
    std::complex<double> I_ik = (std::abs(Z_ik) > 1e-15) ?
        (V_i - V_k) / Z_ik : std::complex<double>{0.0, 0.0};
    std::complex<double> V_j = V_i - Z_ij * I_ik;

    // ── Store into the correct domain map ───────────────────────────────
    if (is_dc) {
      voltages.dc_bus_voltage[rec.eliminated_bus_id] = V_j;
    } else {
      voltages.ac_bus_voltage[rec.eliminated_bus_id] = V_j;
      voltages.bus_voltage[rec.eliminated_bus_id]    = V_j;  // legacy map
    }
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
    const bool is_dc = (rec.domain == NodeDomain::DC);

    // ── Resolve parent voltage from the domain-correct map ───────────────
    // DC lookup: prefer dc_bus_voltage; fall back to legacy bus_voltage only
    // when no AC bus has claimed that key.
    auto lookup_dc_parent = [&](int bid, std::complex<double>& out) -> bool {
      auto it = voltages.dc_bus_voltage.find(bid);
      if (it != voltages.dc_bus_voltage.end()) { out = it->second; return true; }
      if (voltages.ac_bus_voltage.count(bid) == 0) {
        auto jt = voltages.bus_voltage.find(bid);
        if (jt != voltages.bus_voltage.end()) { out = jt->second; return true; }
      }
      return false;
    };
    std::complex<double> V_i;
    if (is_dc) {
      if (!lookup_dc_parent(rec.parent_bus_id, V_i)) continue;
    } else {
      // AC: prefer domain-qualified map, fall back to legacy
      auto it = voltages.ac_bus_voltage.find(rec.parent_bus_id);
      if (it != voltages.ac_bus_voltage.end()) {
        V_i = it->second;
      } else {
        auto jt = voltages.bus_voltage.find(rec.parent_bus_id);
        if (jt == voltages.bus_voltage.end()) continue;
        V_i = jt->second;
      }
    }

    // ── Get connecting branch impedance from domain-correct table ─────────
    double r_ij = 0.0, x_ij = 0.0;
    if (is_dc) {
      for (const auto& br : original_system.dc.branches) {
        if (br.index == rec.branch_id) { r_ij = br.r_pu; break; }
      }
      x_ij = 0.0;  // DC branches have no reactance
    } else {
      for (const auto& br : original_system.ac.branches) {
        if (br.index == rec.branch_id) { r_ij = br.r_pu; x_ij = br.x_pu; break; }
      }
    }
    std::complex<double> Z_ij{r_ij, x_ij};

    // ── Collect load at the eliminated bus from domain-correct tables ─────
    double p_j_mw = 0.0, q_j_mvar = 0.0;
    if (is_dc) {
      for (const auto& bus : original_system.dc.buses) {
        if (bus.index == rec.eliminated_bus_id) { p_j_mw += bus.pd_mw; break; }
      }
      for (const auto& ld : original_system.dc.loads) {
        if (ld.bus == rec.eliminated_bus_id && ld.in_service) p_j_mw += ld.p_mw;
      }
      // DC: no reactive power
    } else {
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
    }
    std::complex<double> S_j{p_j_mw / base_mva, q_j_mvar / base_mva};

    // ── Iterative recovery: V_j^(t+1) = V_i - Z_ij * (S_j^* / V_j^(t)*) ─
    std::complex<double> V_j = V_i; // flat-start: V_j ≈ V_i
    for (int iter = 0; iter < opts.max_pendant_iterations; ++iter) {
      if (std::abs(V_j) < 1e-12) break;
      std::complex<double> I_ij = std::conj(S_j) / std::conj(V_j);
      std::complex<double> V_j_new = V_i - Z_ij * I_ij;
      if (std::abs(V_j_new - V_j) < opts.pendant_convergence_tol) {
        V_j = V_j_new;
        break;
      }
      V_j = V_j_new;
    }

    // ── Store in the domain-correct voltage map ───────────────────────────
    if (is_dc) {
      voltages.dc_bus_voltage[rec.eliminated_bus_id] = V_j;
      // Write to legacy map only if no AC bus has claimed this key
      if (voltages.ac_bus_voltage.count(rec.eliminated_bus_id) == 0)
        voltages.bus_voltage[rec.eliminated_bus_id] = V_j;
    } else {
      voltages.ac_bus_voltage[rec.eliminated_bus_id] = V_j;
      voltages.bus_voltage[rec.eliminated_bus_id]    = V_j;  // legacy map
    }
  }
}

}  // namespace hacdcpf::graph

#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Count queries
// ═══════════════════════════════════════════════════════════════════════
int n_ac_buses(const HybridPowerSystem& sys);
int n_dc_buses(const HybridPowerSystem& sys);
int n_buses(const HybridPowerSystem& sys);

int n_ac_branches(const HybridPowerSystem& sys);
int n_dc_branches(const HybridPowerSystem& sys);
int n_branches(const HybridPowerSystem& sys);

int n_generators(const HybridPowerSystem& sys);
int n_loads(const HybridPowerSystem& sys);
int n_shunts(const HybridPowerSystem& sys);
int n_storage(const HybridPowerSystem& sys);
int n_renewable_gens(const HybridPowerSystem& sys);
int n_converters(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Aggregation queries
// ═══════════════════════════════════════════════════════════════════════
double total_gen_capacity_mw(const HybridPowerSystem& sys);
double total_gen_pmin_mw(const HybridPowerSystem& sys);
double total_load_p_mw(const HybridPowerSystem& sys);
double total_load_q_mvar(const HybridPowerSystem& sys);
double total_renewable_capacity_mw(const HybridPowerSystem& sys);
double total_storage_capacity_mwh(const HybridPowerSystem& sys);
double total_storage_power_mw(const HybridPowerSystem& sys);

// System inertia: sum of H*Sbase for all online generators (MW·s)
double total_system_inertia_mws(const HybridPowerSystem& sys);

// Total CO2 emission at current dispatch (tCO2/h)
double total_emission_tco2_h(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════
struct ValidationResult {
  bool valid{true};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

// Check structural integrity: bus references exist, no duplicate indices,
// impedances non-negative, limits consistent, etc.
ValidationResult validate(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Summary printing
// ═══════════════════════════════════════════════════════════════════════
void print_summary(const HybridPowerSystem& sys, std::ostream& os);
std::string summary_string(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Topology helpers
// ═══════════════════════════════════════════════════════════════════════

// Build adjacency list from AC branches (using bus index -> position mapping).
// Returns vector of vectors: adj[i] = list of neighbor positions for bus position i.
std::vector<std::vector<int>> ac_adjacency_list(const HybridPowerSystem& sys);

// Count in-service generators at each AC bus.
// Returns vector indexed by bus position, value = number of generators.
std::vector<int> generators_per_bus(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Component projection (canonical model expansion)
// ═══════════════════════════════════════════════════════════════════════
//
// Expands rich component tables (e.g., Transformer2W/3W, Switch,
// FlexibleLoad, AsymmetricLoad, Charger, optional three-phase subsystem)
// into the canonical component sets consumed by steady-state PF/OPF/SC
// engines: ACBranch, Load, ChargingStation, etc.
//
// The returned system is a copy of `sys` with additional equivalent
// components appended. Existing canonical components are preserved.
// When zero-impedance branches exist, connected buses are merged and
// the bus_merge_map field is populated in the returned system.
HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys);

/// Rvalue-ref overload — avoids the internal deep copy when the caller
/// already owns a temporary or has std::move()'d the system.
HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys);

// ═══════════════════════════════════════════════════════════════════════
// Zero-impedance bus merging
// ═══════════════════════════════════════════════════════════════════════
//
// Threshold (in per-unit) below which a branch impedance is considered
// "zero" for the purpose of bus merging.
constexpr double kBusMergeZThreshold = 1e-4;

// Identify equivalence classes of AC buses connected by zero-impedance
// branches (both |R| and |X| < kBusMergeZThreshold pu), contract them to a single
// representative bus, reindex all components, and remove self-loops.
// Populates sys.bus_merge_map.  No-op if no zero-Z branches exist.
void merge_zero_impedance_buses(HybridPowerSystem& sys);

// Expand a merged result vector (indexed by internal position, 0-based)
// back to the original external bus count using the merge map.
// Buses that were merged receive the representative bus's value.
std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map);

}  // namespace hacdcpf

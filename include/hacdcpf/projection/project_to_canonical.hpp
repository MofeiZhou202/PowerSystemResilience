#pragma once

/// projection/project_to_canonical.hpp
/// =====================================
/// Functions for projecting a HybridPowerSystem to canonical form
/// (expanding rich components into solver-ready equivalents, merging
/// zero-impedance buses, stripping dead islands).
/// Replaces: model/network_utils.hpp.

#include <iosfwd>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

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

double total_system_inertia_mws(const HybridPowerSystem& sys);
double total_emission_tco2_h(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════
struct ValidationResult {
  bool valid{true};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};

ValidationResult validate(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Summary printing
// ═══════════════════════════════════════════════════════════════════════
void print_summary(const HybridPowerSystem& sys, std::ostream& os);
std::string summary_string(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Topology helpers
// ═══════════════════════════════════════════════════════════════════════
std::vector<std::vector<int>> ac_adjacency_list(const HybridPowerSystem& sys);
std::vector<int> generators_per_bus(const HybridPowerSystem& sys);

// ═══════════════════════════════════════════════════════════════════════
// Component projection (canonical model expansion)
// ═══════════════════════════════════════════════════════════════════════
HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys);
HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys);

// ═══════════════════════════════════════════════════════════════════════
// Zero-impedance bus merging
// ═══════════════════════════════════════════════════════════════════════
constexpr double kBusMergeZThreshold = 1e-4;

void merge_zero_impedance_buses(HybridPowerSystem& sys, bool allow_merge = true);
void strip_dead_islands(HybridPowerSystem& sys);

std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map);

}  // namespace hacdcpf

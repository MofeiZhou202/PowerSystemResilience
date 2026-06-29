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
#include "hacdcpf/power_flow/power_flow_result.hpp"  // BranchFlow, DeviceTerminalFlow

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

// Reconfiguration-friendly projection: expands rich devices and merges
// zero-impedance buses but optionally KEEPS dead islands so open tie switches
// into currently-unenergised sections remain valid reconnection candidates.
HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys,
                                              bool strip_dead);

// ═══════════════════════════════════════════════════════════════════════
// Zero-impedance bus merging
// ═══════════════════════════════════════════════════════════════════════
constexpr double kBusMergeZThreshold = 1e-4;

void merge_zero_impedance_buses(HybridPowerSystem& sys, bool allow_merge = true);
void strip_dead_islands(HybridPowerSystem& sys);

std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map);

// ═══════════════════════════════════════════════════════════════════════
// Device terminal flows (switches / circuit breakers)
// ═══════════════════════════════════════════════════════════════════════
// Closed switches/CBs collapse to zero-impedance branches and are merged out of
// the canonical solve, so they carry no ACBranch flow. This recovers their flow
// as the net real/reactive injection across the two-terminal cut, using the
// original endpoint buses and per-bus net injection (gen − load). Open devices
// report zero. base_mva scales the result to MW/MVAr.
// ═══════════════════════════════════════════════════════════════════════
// Device terminal flows (switches / circuit breakers)
// ═══════════════════════════════════════════════════════════════════════
// Closed switches/CBs collapse to zero-impedance branches and are merged out of
// the canonical solve, so they carry no ACBranch flow. This recovers their flow
// as the net real/reactive injection across the two-terminal cut. The base
// overload uses rich component net injection (exact for radial feeders). The
// PF-aware overload subtracts solved AC line flows at each bus so meshed/looped
// networks are handled from the actual converged state. Open devices report 0.
struct DeviceTerminalFlows {
  std::vector<DeviceTerminalFlow> ac_switches;
  std::vector<DeviceTerminalFlow> ac_circuit_breakers;
};
DeviceTerminalFlows compute_device_terminal_flows(const HybridPowerSystem& sys,
                                                  const std::vector<double>& vm,
                                                  const std::vector<double>& va);
// PF-aware: orig-space AC branch flows (same indexing as sys.ac.branches) make
// the cut account for already-solved line flows.
DeviceTerminalFlows compute_device_terminal_flows(const HybridPowerSystem& sys,
                                                  const std::vector<BranchFlow>& ac_branch_flows);

}  // namespace hacdcpf

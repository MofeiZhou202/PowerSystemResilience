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

struct ProjectionOptions {
  ProjectionMode mode{ProjectionMode::ThresholdApproximate};
  bool strip_dead_islands{true};
  double impedance_threshold{1e-4};
};

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

HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys,
                                              const ProjectionOptions& options);
HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys,
                                              const ProjectionOptions& options);

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

/// Report attribution coverage for the requested result field. Coverage is
/// computed from projection metadata rather than inferred from vector length.
ObservableAttribution evaluate_attribution(
    const HybridPowerSystem& original,
    const HybridPowerSystem& projected,
    ObservableKind observable);

enum class BusVectorSemantics {
  Intensive,
  Extensive,
};

std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map,
    BusVectorSemantics semantics);

// ═══════════════════════════════════════════════════════════════════════
// Device terminal flows (switches / circuit breakers)
// ═══════════════════════════════════════════════════════════════════════
// Closed switches/CBs collapse to zero-impedance branches and are merged out of
// the canonical solve. Recovery subtracts solved authored AC-line flows from
// rich component injections; the residual KCL is the collapsed-device terminal
// injection, including on meshed networks. Open devices report zero.
struct DeviceTerminalFlows {
  std::vector<DeviceTerminalFlow> ac_switches;
  std::vector<DeviceTerminalFlow> ac_circuit_breakers;
};
// ac_branch_flows must use the authored sys.ac.branches index space.
DeviceTerminalFlows compute_device_terminal_flows(const HybridPowerSystem& sys,
                                                  const std::vector<BranchFlow>& ac_branch_flows);

}  // namespace hacdcpf

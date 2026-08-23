#pragma once

/// graph/switch_contraction.hpp
/// =============================
/// Zero-impedance supernode merging (closed switches, closed breakers,
/// zero-impedance lines).
///
/// Mathematical basis:
///   If Z_ij → 0, then V_i = V_j.  Merging prevents Y-bus ill-conditioning.
///
/// Aggregation rules:
///   - Loads / generators / shunts: sum P, Q, limits
///   - Bus type: max priority wins (SLACK > PV > PQ > ISOLATED)
///   - base_kv: must match within tolerance (error otherwise)
///   - External branch endpoints: redirected to the super-node

#include <vector>

#include "hacdcpf/graph/power_system_graph.hpp"
#include "hacdcpf/graph/reduction_mapping.hpp"
#include "hacdcpf/graph/topology_analysis.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Options
// ═══════════════════════════════════════════════════════════════════════

struct ContractionOptions {
  bool   contract_closed_switches{true};
  bool   contract_closed_breakers{true};
  bool   contract_zero_impedance_lines{true};
  double zero_impedance_threshold{1e-8};    ///< |Z| < threshold → zero-Z
  double voltage_base_tolerance{1e-6};      ///< kV tolerance for base check
  bool   allow_multi_slack_merge{false};    ///< false → error on multi-slack
};

// ═══════════════════════════════════════════════════════════════════════
// Result
// ═══════════════════════════════════════════════════════════════════════

struct ContractionResult {
  /// Graph after contraction (super-nodes replace merged node groups)
  PowerSystemGraph contracted_graph;
  /// System model after contraction (loads/gens/shunts aggregated)
  HybridPowerSystem contracted_system;
  /// Domain-qualified, composable bus certificate for this stage.
  ReductionMapping mapping;

  // ── Domain-qualified maps (correct in all hybrid systems) ──────────
  /// AC bus_id → AC super-bus_id.  Use this for AC component remapping.
  std::unordered_map<int, int>              ac_bus_to_super;
  /// DC bus_id → DC super-bus_id.  Use this for DC component remapping.
  std::unordered_map<int, int>              dc_bus_to_super;
  /// AC super-bus_id → list of original AC bus_ids in that super-node.
  std::unordered_map<int, std::vector<int>> ac_super_to_buses;
  /// DC super-bus_id → list of original DC bus_ids in that super-node.
  std::unordered_map<int, std::vector<int>> dc_super_to_buses;

  // ── Legacy combined maps (backward compat) ─────────────────────────
  /// @deprecated  Only safe when AC and DC bus IDs are numerically disjoint.
  ///   If AC bus N and DC bus N both exist, the DC entry overwrites the AC
  ///   entry in this map.  Prefer ac_bus_to_super / dc_bus_to_super.
  std::unordered_map<int, int>              bus_to_super;
  /// @deprecated  Same caveat as bus_to_super.
  std::unordered_map<int, std::vector<int>> super_to_buses;

  std::vector<SwitchContractionRecord> switch_records;
  std::vector<Diagnostic>              diagnostics;
};

// ═══════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════

/// Merge all zero-impedance / closed-switch edges in \p graph.
/// The returned ContractionResult contains the contracted graph AND a
/// modified HybridPowerSystem with aggregated component data.
ContractionResult contract_zero_impedance_edges(
    const PowerSystemGraph&  graph,
    const HybridPowerSystem& system,
    const ContractionOptions& options = {});

}  // namespace hacdcpf::graph

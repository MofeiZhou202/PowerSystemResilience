#pragma once

/// graph/topology_analysis.hpp
/// ============================
/// Topological analysis of a PowerSystemGraph:
///   - Connected component / island detection
///   - Radiality check
///   - Cycle detection and fundamental cycle basis
///   - Bridge (cut-edge) and cut-vertex (articulation point) detection
///
/// Results feed into:
///   - Power flow island solver
///   - OPF feasibility checks
///   - ONR radiality constraints
///   - Reliability and resilience analysis

#include <string>
#include <vector>

#include "hacdcpf/graph/power_system_graph.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Diagnostics
// ═══════════════════════════════════════════════════════════════════════

enum class DiagCode {
  OK = 0,
  // Connectivity
  GraphNoSlackInIsland,
  GraphNoDCVoltageRef,
  GraphIsolatedLoad,
  GraphDisconnectedBus,
  // Contraction
  GraphVoltageBaseMismatch,
  GraphMultipleSlack,
  // Reduction
  NodeHasInjection,
  NodeIsRetained,
  NodeDegreeNotTwo,
  // Kron
  KronFillInTooLarge,
  KronSingularYbb,
  // General
  Warning,
  Error,
};

struct Diagnostic {
  DiagCode       code{DiagCode::OK};
  std::string    message;
  std::vector<int> related_buses;
  std::vector<int> related_branches;
};

// ═══════════════════════════════════════════════════════════════════════
// Island info
// ═══════════════════════════════════════════════════════════════════════

enum class IslandStatus {
  Valid,             ///< Has voltage reference, in-service
  NoSlack,           ///< AC island with no slack / external grid
  NoDCVoltageRef,    ///< DC island with no V-reference bus
  IsolatedLoad,      ///< Load node with no path to any source
  Empty,             ///< No in-service buses
};

struct IslandInfo {
  int            island_id{0};
  std::vector<int> bus_ids;   ///< All buses in this island (original IDs)
  NodeDomain     domain{NodeDomain::AC};
  bool           has_ac_slack{false};
  bool           has_dc_voltage_ref{false};
  IslandStatus   status{IslandStatus::Valid};
};

// ═══════════════════════════════════════════════════════════════════════
// Topology report
// ═══════════════════════════════════════════════════════════════════════

struct TopologyReport {
  // Islands / components
  std::vector<IslandInfo> islands;
  int   n_ac_islands{0};
  int   n_dc_islands{0};
  bool  all_islands_valid{true};

  // Radiality
  bool  is_connected{false};    ///< True iff single connected component
  bool  is_radial{false};       ///< True iff connected and tree
  int   cycle_count{0};         ///< #in-service-edges - #nodes + #components

  // Structural analysis
  std::vector<int> bridge_edge_ids;       ///< Edge IDs that are cut-edges
  std::vector<int> cut_vertex_bus_ids;    ///< Bus IDs of articulation points
  std::vector<std::vector<int>> fundamental_cycles; ///< Each cycle is a list of node indices

  // Diagnostics
  std::vector<Diagnostic> diagnostics;
};

// ═══════════════════════════════════════════════════════════════════════
// API
// ═══════════════════════════════════════════════════════════════════════

/// Full topology analysis on a PowerSystemGraph.
TopologyReport analyze_topology(const PowerSystemGraph& graph);

/// Quick connectivity check (no full analysis overhead).
bool is_connected(const PowerSystemGraph& graph);

/// Quick radiality check.
bool is_radial(const PowerSystemGraph& graph);

/// Count connected islands (considering only in-service nodes/edges).
int count_islands(const PowerSystemGraph& graph);

/// Find all bridge edges (cut-edges) via Tarjan's DFS.
/// Returns list of edge indices in graph.edges[].
std::vector<int> find_bridges(const PowerSystemGraph& graph);

/// Find all articulation points (cut vertices) via Tarjan's DFS.
/// Returns list of node indices in graph.nodes[].
std::vector<int> find_articulation_points(const PowerSystemGraph& graph);

/// Build a fundamental cycle basis for the graph.
/// Each cycle is a list of edge indices forming a simple cycle.
std::vector<std::vector<int>> find_fundamental_cycles(const PowerSystemGraph& graph);

}  // namespace hacdcpf::graph

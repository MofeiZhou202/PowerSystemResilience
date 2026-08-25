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
  IsolatedLoad,      ///< Load bus with no incident edge at all (orphan); a load
                     ///  islanded only by out-of-service branches is NoSlack
  Empty,             ///< No in-service buses
};

struct IslandInfo {
  int            island_id{0};
  std::vector<int> bus_ids;       ///< All buses in this island (original IDs) — legacy; may contain
                                  ///  duplicate integers when AC and DC bus IDs overlap.  Prefer
                                  ///  ac_bus_ids / dc_bus_ids for domain-correct operations.
  std::vector<int> ac_bus_ids;    ///< AC-domain bus IDs in this island
  std::vector<int> dc_bus_ids;    ///< DC-domain bus IDs in this island
  NodeDomain     domain{NodeDomain::AC};  ///< Primary domain (AC if island has any AC nodes)
  bool           has_ac_slack{false};
  bool           has_dc_voltage_ref{false};
  IslandStatus   status{IslandStatus::Valid};
};

struct TopologyBusRef {
  NodeDomain domain{NodeDomain::AC};
  int bus_id{0};
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
  std::vector<int> bridge_edge_ids;       ///< Cut-edge positions in graph.edges[]
  std::vector<TopologyBusRef> cut_vertices; ///< Domain-qualified articulation points
  std::vector<int> cut_vertex_bus_ids;      ///< Deprecated flat compatibility view
  std::vector<std::vector<int>> fundamental_cycles; ///< Each cycle is graph.edges[] indices

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

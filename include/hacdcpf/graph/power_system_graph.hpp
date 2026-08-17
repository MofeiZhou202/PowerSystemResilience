#pragma once

/// graph/power_system_graph.hpp
/// ============================
/// Core graph representation of a HybridPowerSystem.
/// Nodes correspond to AC/DC buses; edges correspond to branches, switches,
/// breakers, transformers, VSC couplings, and DCDC couplings.
///
/// This is the entry point for the graph analysis & reduction pipeline:
///   HybridPowerSystem → PowerSystemGraph → topology/reduction modules.

#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/model/typed_ids.hpp"

namespace hacdcpf::graph {

// ═══════════════════════════════════════════════════════════════════════
// Node domain
// ═══════════════════════════════════════════════════════════════════════

enum class NodeDomain { AC, DC };

// ═══════════════════════════════════════════════════════════════════════
// Edge category
// ═══════════════════════════════════════════════════════════════════════

enum class EdgeCategory {
  AC_Line,         ///< Regular AC transmission / distribution branch
  AC_Transformer,  ///< Two/three-winding transformer
  Switch,          ///< AC switch (may be open/closed)
  Breaker,         ///< AC circuit breaker
  DC_Line,         ///< DC branch
  DC_Switch,       ///< DC circuit breaker
  VSC_Coupling,    ///< VSC converter AC↔DC link (virtual edge)
  DCDC_Coupling,   ///< DCDC converter DC↔DC link (virtual edge)
};

// ═══════════════════════════════════════════════════════════════════════
// GraphNode — represents one AC or DC bus
// ═══════════════════════════════════════════════════════════════════════

struct GraphNode {
  int bus_id{0};          ///< Original bus index in the model
  NodeDomain domain{NodeDomain::AC};

  /// For AC nodes: BusType (PQ / PV / SLACK / ISOLATED)
  BusType  ac_bus_type{BusType::PQ};
  /// For DC nodes: DCBusType (DC_P / DC_V)
  DCBusType dc_bus_type{DCBusType::DC_P};

  double base_kv{0.0};

  /// Injection / device flags (set during graph construction)
  bool has_generator{false};   ///< Generator or external grid connected
  bool has_load{false};        ///< Load (AC or DC) connected
  bool has_shunt{false};       ///< Shunt capacitor/reactor/conductance
  bool has_storage{false};     ///< Storage device connected
  bool has_vsc_ac{false};      ///< AC side of a VSC
  bool has_vsc_dc{false};      ///< DC side of a VSC
  bool has_dcdc{false};        ///< Terminal of a DCDC converter
  bool has_controllable{false};///< Any tap/voltage-control device

  /// Convenience: true if slack / external grid
  bool is_slack{false};
  /// True if the user has marked this bus as a monitored output node
  bool is_monitored{false};
  /// True if there is an active generator (PV or slack) that controls voltage
  bool is_voltage_controlled{false};

  bool in_service{true};
};

// ═══════════════════════════════════════════════════════════════════════
// GraphEdge — represents one branch / coupling
// ═══════════════════════════════════════════════════════════════════════

struct GraphEdge {
  int edge_id{0};         ///< Unique positional ID within the graph (0 … E-1).
                          ///  Used for graph-internal lookups and as the branch
                          ///  key in ReductionMapping.  NOT the source model's
                          ///  component index — use comp_index for that.
  int comp_index{0};      ///< Original component .index in the source model
                          ///  (ACBranch/DCBranch/Switch/CircuitBreaker/…).
                          ///  Reducers disable the right model component by
                          ///  matching this, never edge_id.
  int from_node{0};       ///< Index into PowerSystemGraph::nodes
  int to_node{0};         ///< Index into PowerSystemGraph::nodes
  int from_bus_id{0};     ///< Original bus ID
  int to_bus_id{0};       ///< Original bus ID

  EdgeCategory category{EdgeCategory::AC_Line};
  bool in_service{true};

  double r_pu{0.0};
  double x_pu{0.0};
  double b_pu{0.0};       ///< Total line-charging susceptance (π model)
  double tap{1.0};
  double shift_deg{0.0};
  double rate_a_mva{0.0};

  /// True when |Z| < zero_impedance_threshold (closed switch / zero-Z line)
  bool is_zero_impedance{false};
  /// True when this edge is a closed switch or breaker
  bool is_closed_switch{false};
};

// ═══════════════════════════════════════════════════════════════════════
// PowerSystemGraph — the graph itself
// ═══════════════════════════════════════════════════════════════════════

struct PowerSystemGraph {
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;

  /// bus_id → index in nodes[] for AC buses only.
  /// Use ac_node_idx() / dc_node_idx() for domain-specific lookups in hybrid
  /// systems.  The legacy bus_id_to_node_idx is kept for backward-compatibility
  /// with AC-only consumers (graph reduction, contraction, etc.) but MUST NOT
  /// be used when both domains share the same bus-index values.
  std::unordered_map<int, int> bus_id_to_node_idx;

  /// Domain-qualified lookup maps.  These are populated by
  /// build_power_system_graph() and are always authoritative.
  std::unordered_map<int, int> ac_bus_id_to_node_idx;
  std::unordered_map<int, int> dc_bus_id_to_node_idx;

  /// Adjacency list: node_idx → list of (edge_idx, neighbor_node_idx)
  std::vector<std::vector<std::pair<int, int>>> adj;

  int node_count() const { return static_cast<int>(nodes.size()); }
  int edge_count() const { return static_cast<int>(edges.size()); }

  /// Legacy lookup — searches the shared map.  Works correctly for AC-only
  /// graphs; for hybrid graphs prefer ac_node_idx() / dc_node_idx().
  int node_idx(int bus_id) const {
    auto it = bus_id_to_node_idx.find(bus_id);
    return (it == bus_id_to_node_idx.end()) ? -1 : it->second;
  }

  /// AC-domain lookup: returns -1 if the AC bus does not exist.
  int ac_node_idx(int bus_id) const {
    auto it = ac_bus_id_to_node_idx.find(bus_id);
    return (it == ac_bus_id_to_node_idx.end()) ? -1 : it->second;
  }

  /// DC-domain lookup: returns -1 if the DC bus does not exist.
  int dc_node_idx(int bus_id) const {
    auto it = dc_bus_id_to_node_idx.find(bus_id);
    return (it == dc_bus_id_to_node_idx.end()) ? -1 : it->second;
  }

  /// Strong-typed overloads (model/typed_ids.hpp): adopt StableBusId -> NodeIdx
  /// at this public graph boundary so a bus id cannot be silently used as a node
  /// index. An absent bus yields an invalid NodeIdx (value() == -1).
  NodeIdx ac_node_idx(StableBusId bus_id) const {
    return NodeIdx{ac_node_idx(bus_id.value())};
  }
  NodeIdx dc_node_idx(StableBusId bus_id) const {
    return NodeIdx{dc_node_idx(bus_id.value())};
  }
};

// ═══════════════════════════════════════════════════════════════════════
// Factory
// ═══════════════════════════════════════════════════════════════════════

/// Build a PowerSystemGraph from a HybridPowerSystem.
/// \param system            The hybrid power system to graph.
/// \param zero_impedance_threshold  Branches with |Z| < threshold are
///                          flagged is_zero_impedance (default 1e-8 pu).
PowerSystemGraph build_power_system_graph(
    const HybridPowerSystem& system,
    double zero_impedance_threshold = 1e-8);

}  // namespace hacdcpf::graph

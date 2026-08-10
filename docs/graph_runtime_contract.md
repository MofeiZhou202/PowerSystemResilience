# Graph and Reduction Runtime Contract

This document defines the current contract of `include/hacdcpf/graph/` and
`src/graph/`. It covers graph construction, topology reports, network
reduction, mappings, and result recovery. Projection to solver models is a
separate contract described in [projection and result attribution](projection_and_results.md).

## Public entry points

The umbrella header is `hacdcpf/graph/graph.hpp`. The main stages are:

1. `build_power_system_graph()` creates `PowerSystemGraph` from a
   `HybridPowerSystem`.
2. `analyze_topology()` reports islands, radiality, cycles, bridges, cut
   vertices, and diagnostics.
3. `contract_zero_impedance_edges()`, the reduction-plan APIs, series/pendant
   reduction, and Kron APIs create smaller models and recovery records.
4. `recover_*()` expands retained complex voltages back to eliminated buses.

## Identity and index spaces

| Value | Meaning |
|---|---|
| `GraphNode::bus_id` | Stable source-model bus `.index`; qualify it with `NodeDomain`. |
| `GraphEdge::comp_index` | Stable source component `.index`. |
| `GraphEdge::edge_id` | Graph-internal edge position, normally `0..edges.size()-1`. |
| `from_node`, `to_node` | Positions in `PowerSystemGraph::nodes`. |
| adjacency `edge_idx` | Position in `PowerSystemGraph::edges`. |
| `TopologyReport::bridge_edge_ids` | Positions in `graph.edges[]`. |
| `fundamental_cycles` | Each cycle is a list of positions in `graph.edges[]`. |
| `find_articulation_points()` | Positions in `graph.nodes[]`. |
| `TopologyReport::cut_vertices` | Stable, domain-qualified `{domain,bus_id}` references. |

`ac_bus_id_to_node_idx` and `dc_bus_id_to_node_idx` are authoritative. The
flat `bus_id_to_node_idx`, `IslandInfo::bus_ids`,
`TopologyReport::cut_vertex_bus_ids`, and flat maps in `ReductionMapping` are
compatibility views. They are ambiguous when AC and DC buses share an integer
ID and must not drive hybrid calculations.

## Graph semantics

Nodes represent in-service AC and DC buses. Edges represent AC/DC branches,
transformers, switches, breakers, VSC virtual couplings, and DC/DC virtual
couplings. Electrical quantities stored on graph edges use per-unit impedance,
degrees for phase shift, kV for voltage bases, and MVA for ratings.

Open switches and breakers may remain as out-of-service edges so latent
topology is inspectable. Connectivity, cycles, bridges, and radiality use only
in-service nodes and edges. An island is valid only when its active domains
have the required voltage reference. Diagnostics expose missing references,
isolated loads, invalid merges, and reduction failures.

## Reduction and recovery boundaries

| Operation | Preserved behavior | Boundary or approximation |
|---|---|---|
| Closed-switch/zero-impedance contraction | Members receive the super-node voltage exactly. | Voltage bases must match; multiple slack merging is rejected unless enabled. |
| Passive degree-2 series reduction | Series impedance and passive-node voltage recovery. | Buses with injections or retained roles are not eligible. |
| Pendant reduction | Leaf demand is folded to its parent. | Constant-power voltage recovery is iterative and reports no separate convergence flag. |
| Dense Kron reduction | Linear Schur complement; optional known current injection. | The zero-injection form requires `I_beta=0`; fill and singular-pivot guards can reject it. Constant-power interiors are an approximation. |
| Sparse Kron reduction | Reduced sparse matrix plus reverse recovery tape. | Candidates violating pivot, front, or fill caps remain retained. `valid()` and `error` carry status. |

`ReductionMapping` records authored-to-reduced buses and branches and the
operation-specific recovery tape. Hybrid callers must use the AC/DC qualified
bus maps. `FullNetworkVoltages::ac_bus_voltage` and `dc_bus_voltage` are the
authoritative outputs; its flat `bus_voltage` map is AC-preferred compatibility
output when IDs collide. Complex voltage is per unit; angle accessors return
degrees.

No graph reduction by itself proves that a nonlinear solver result is
equivalent. Use the registered round-trip tests and projection certificate for
that claim, and retain approximation/fill diagnostics in user-facing results.

## Registered verification

- `test_graph`: construction, topology, AC/DC same-ID cut vertices,
  contraction, series/pendant reduction, and recovery.
- `test_graph_kron`: dense/sparse Kron identities, fill guards, recovery, and
  the constant-power approximation boundary.
- `test_graph_roundtrip`: PF/OPF round trips, stable component IDs, switches,
  breakers, AC/DC collisions, and real feeder cases.
- `test_distribution_pipeline`: graph information through projection and
  downstream analyses.


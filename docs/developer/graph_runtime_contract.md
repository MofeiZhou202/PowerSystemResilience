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
| `GraphEdge::edge_id` | Graph-internal edge position, always `0..edges.size()-1`. |
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
two/three-winding transformer connectivity, switches, breakers, and VSC,
LCC, DC/DC, or EnergyRouter virtual couplings. Electrical quantities stored on
ordinary line edges use per-unit impedance, degrees for phase shift, kV for
voltage bases, and MVA for ratings. Transformer3W uses an HV--MV/HV--LV
connectivity tree; LCC and EnergyRouter edges are connectivity-only and do not
invent electrical equivalents. `build_three_phase_power_system_graph()` owns a
separate phase-resolved graph so it is not overlaid on the balanced graph.

Rich load/generation/storage/converter terminals set retention flags.
Transformer2W edges still omit electrical parameters, and newly introduced
component families must add flags/edges or fail closed before reduction.
Consumers must not infer full electrical-model coverage from graph connectivity.

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

`ReductionMapping` records authoritative `BusRef`/`BranchRef` forward and
reverse maps plus operation-specific recovery records.
`compose_reduction_mappings()` composes multi-stage identity certificates;
hybrid callers must use these domain-qualified maps. Legacy integer maps are
AC-preferred compatibility views. `FullNetworkVoltages::ac_bus_voltage` and `dc_bus_voltage` are the
authoritative outputs; its flat `bus_voltage` map is AC-preferred compatibility
output when IDs collide. Complex voltage is per unit; angle accessors return
degrees.

No graph reduction by itself proves that a nonlinear solver result is
equivalent. Use the registered round-trip tests and projection certificate for
that claim, and retain approximation/fill diagnostics in user-facing results.

`make_reduction_plan()` uses `BusRef` for candidates, conflicts, and Kron
batches, so same-number AC/DC buses remain independent. Action edge references
are declared `graph.edges[]` positions. Series-created edges preserve
`GraphEdge::edge_id == edges[] position`; stable source identity is always
`comp_index`/`BranchRef`. There is still no unified public C++ API that executes
all reductions, solves, and recovers in one call.

## Production HTTP composition

- `POST /api/session/topology` analyzes the resident graph. Domain-qualified
  `cut_vertices` and per-endpoint bridge domains are authoritative; flat cut
  IDs and diagnostic bus IDs are compatibility output. Its bus/branch counts
  are graph storage counts, including out-of-service objects.
- `POST /api/session/network_reduction` applies contraction, one series plan,
  and optional one pendant plan. HTTP defaults are switch=true, series=true,
  pendant=false, Kron=false, and zero-impedance-line contraction=false. The
  last default differs from C++ `ContractionOptions` (true).
- The endpoint only identifies Kron candidates; it does not apply Kron to the
  exported system. Sparse Kron is separately integrated into graph-reduced
  three-phase hybrid OPF.
- `reduced_system` removes inactive audit ghosts, but rich endpoints are first
  remapped and then validated. Dangling LCC/EnergyRouter/MobileStorage data is
  rejected rather than silently deleted. Transformer3W, LCC, EnergyRouter, and
  three-phase preservation is covered by export/reload E2E; their virtual graph
  edges remain connectivity-only.

The complete source-equivalent contract, HTTP fields, formulas, and audit are
in the [graph module manual](../modules/graph/graph_manual.tex).

## Registered verification

- `test_graph`: construction, topology, AC/DC same-ID cut vertices,
  contraction, series/pendant reduction, and recovery.
- `test_graph_kron`: dense/sparse Kron identities, fill guards, recovery, and
  the constant-power approximation boundary.
- `test_graph_roundtrip`: PF/OPF round trips, stable component IDs, switches,
  breakers, AC/DC collisions, and real feeder cases.
- `test_topology_crossval`: bridge, cycle, and cut-vertex checks against
  independent graph oracles.
- `test_validation`: isolated-load topology status reaches validation.
- `gui_api_e2e`: rich reduction, compact export, and reload preservation.

The rebuilt `macos-release` graph binaries pass 76 cases / 629 assertions;
`test_validation` adds 38/180 and GUI E2E passes 80 checks. A selected rebuilt
ASan/UBSan run passes 42 tests with one conditional skip and no report. This is
a focused validation set, not a full CTest run or a full-network scale study.

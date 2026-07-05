> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# CIM / CGMES 3.0 ↔ HACDCPF Crosswalk

This document is the normative field-level mapping for the bounded CGMES 3.0
adapter (`include/hacdcpf/io/cim_io.hpp`, `src/io/cim_io.cpp`).  Per
`digital_twin_data_io_architecture.md` §5, a standards adapter is not "complete"
without a committed crosswalk; this is that crosswalk plus the explicit,
documented ceiling.

## Scope

- **Profiles:** CGMES 3.0 **EQ** (equipment) + **SSH** (steady-state hypothesis).
- **Binding level:** Rich (§3.2). Import reconstructs rich AC components.
- **Namespaces:** `cim` = `http://iec.ch/TC57/CIM100#`, `rdf` = the RDF syntax ns.
- **Units:** asserted (SI / per-unit as typed by CGMES); no heuristic inference.

## Class / attribute mapping

| CGMES class | CGMES attribute | HACDCPF target | Notes |
| --- | --- | --- | --- |
| `BaseVoltage` | `nominalVoltage` | `ACBus.base_kv` | Referenced by `TopologicalNode`. |
| `TopologicalNode` (or `ConnectivityNode`) | `IdentifiedObject.name` | `ACBus.name` | Bus index from `rdf:ID` suffix `_busN`, else sequential. |
| `TopologicalNode` | `.BaseVoltage` → `BaseVoltage` | `ACBus.base_kv` | Resolved via `rdf:resource`. |
| `ACLineSegment` | `.r` / `.x` / `.bch` | `ACBranch.r_pu` / `.x_pu` / `.b_pu` | Two `Terminal`s give `from_bus`/`to_bus`. |
| `SynchronousMachine` | `.minP` / `.maxP` / `.minQ` / `.maxQ` | `Generator.pmin_mw` / `.pmax_mw` / `.qmin_mvar` / `.qmax_mvar` | Bus via its `Terminal`. |
| `RotatingMachine` | `.ratedS` / `.p` (SSH) | `Generator.mbase_mva` / `.pg_mw` | `.p` is SSH-only. |
| `EnergyConsumer` | `.p` / `.q` (SSH) | `Load.p_mw` / `.q_mvar` | Bus via its `Terminal`. |
| `Terminal` | `.ConductingEquipment`, `.TopologicalNode` | (connectivity) | Wires equipment to buses. |

## Out of scope (reported, not silently dropped)

The importer emits an `ImportRecord{Skipped, StructuralLoss, Warning}` for every
CGMES class outside the table above, e.g.:

- `PowerTransformer` / `PowerTransformerEnd` (transformers)
- DC equipment, converters, `EnergyRouter`-equivalents
- `TP` (topology) and `SV` (state-variable) profiles
- Dynamics / protection / measurement profiles

## Round-trip contract

`import(export(M)) ≈ M` holds for the bounded subset (buses, lines, synchronous
machines, loads, connectivity, and per-bus base voltage).  Verified by
`tests/test_io_cim.cpp`.  Bus/branch/generator/load indices are preserved via the
`rdf:ID` suffix convention (`_busN`, `_lineN`, `_genN`, `_loadN`); arbitrary
hand-authored ids fall back to sequential assignment.

## Security

The RDF/XML reader is self-contained and **XXE-hardened**: `<!DOCTYPE>` and
`<!ENTITY>` declarations are rejected rather than expanded, closing the external-
entity attack surface (§12).

## Roadmap beyond the ceiling

- Transformers (`PowerTransformer` + ends → `Transformer2W`/`3W`).
- `TP`/`SV` profiles for solved-state exchange.
- Operational limits (`OperationalLimitSet` → ratings / voltage limits).
- Full ENTSO-E boundary-set handling and multi-file EQ/SSH/TP/SV packages.

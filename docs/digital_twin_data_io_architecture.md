> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Data IO to Digital Twin Architecture

> **Document status.** Specification-grade. This document is both the theory
> roadmap *and* the normative contract for the `数据IO` module. Sections marked
> **[CONTRACT]** state requirements that importers, exporters, and readiness
> scoring must satisfy; sections marked **[STATUS]** describe what is built
> today; sections marked **[ROADMAP]** describe committed future work with a
> defined scope. Where the current implementation diverges from a **[CONTRACT]**
> clause, the divergence is called out explicitly rather than hidden.

## 1. Motivation

The `数据IO` module is not a collection of file import/export buttons. For a
distribution-system digital twin, IO is the **model contract** that decides
whether a network model can be exchanged, solved, validated, synchronized with
measurements, calibrated, and audited over time.

A contract has testable clauses. This document therefore replaces prose
aspirations with:

- an explicit **hub-and-spoke** conversion architecture (§3),
- a **maturity model that gates rather than averages** (§4),
- a **round-trip / IO conformance contract** (§7),
- a **cross-field parameter-validity layer**, not only per-field ranges (§6),
- a **unit-discipline rule** that forbids heuristic inference on the twin path
  (§6.3),
- a **temporal and telemetry data model** for operational synchronization (§10),
- a **uniform import-diagnostic contract** across all adapters (§8),
- a **security/trust-boundary posture** for untrusted inputs and external
  solvers (§12).

## 2. Current Implementation Status [STATUS]

The architecture must be read against what exists. The IO layer today is
richer, and more asymmetric, than earlier drafts of this document acknowledged.

| Format | Import | Export | Round-trip today | Notes |
| --- | --- | --- | --- | --- |
| Native JSON / JPC | Full | Full | Lossless (design goal) | Schema `v1.0`, package `0.5.0`; `ImportMode {Strict, Permissive}`. |
| MATPOWER `.m` | Full (AC only) | **None** | N/A | Bus/gen/branch/gencost; transformer detected from tap; **no exporter**. |
| GridLAB-D `.glm` | Balanced-AC subset | Full canonical | **Asymmetric** | Export projects the full canonical model; import handles balanced AC only. |
| OpenDSS `.dss` | Balanced-AC subset | Full canonical | **Asymmetric** | Text import + C-API snapshot solve; export projects canonical model. |
| ETAP `.xlsx` / native XML | Full | Full | Fidelity-checked | Conditional on `-DHACDCPF_ENABLE_ETAP=ON`; otherwise stubbed. |
| CIM (IEC 61970) | None | None | N/A | Standards profile referenced in registry only; no parser. |
| CSV / telemetry / event catalog | None | None | N/A | Roadmap. |

Two facts drive the rest of this document:

1. **Import and export are not symmetric.** External-format *export* runs
   through the full canonical projection; external-format *import* is limited
   to balanced-AC subsets (GridLAB-D/OpenDSS) or absent (MATPOWER export, CIM).
   Any "bidirectional" claim must state the fidelity ceiling per direction.
2. **ETAP is a first-class adapter** (Excel + native XML, with a round-trip
   fidelity check) and belongs in the architecture, not outside it.

## 3. Architectural Principles [CONTRACT]

### 3.1 Canonical model is the mandatory pivot (hub-and-spoke)

Every external format converts **through** the canonical model, never directly
to another external format. With `N` external formats this requires `2N`
adapters (import-to-canonical, canonical-to-export), not `N²` pairwise
converters. Format-to-format shortcuts are prohibited and are a review-blocking
finding.

- Rich model → canonical: `project_to_canonical_models(...)`
  (`include/hacdcpf/projection/project_to_canonical.hpp`).
- Result back-projection: `BranchExpandMap`, `BusMergeMap`
  (`include/hacdcpf/projection/canonical_network.hpp`).

### 3.2 Rich vs. canonical import target must be declared per format

Projection is **one-way**: rich → canonical, with provenance maps that
back-project *results*, not *models*. It cannot be inverted to reconstruct rich
structure (Transformer3W, switches, DER controllers) from a canonical-level
external file. Therefore each importer must declare its **binding level**:

- **Rich-binding importer** — reconstructs rich components (e.g., native JSON,
  ETAP). Round-trip target: rich-model preservation.
- **Canonical-binding importer** — lands at canonical buses/branches and does
  **not** attempt to recover rich structure (e.g., GridLAB-D/OpenDSS today).
  Round-trip target: canonical-model preservation only. The structural loss is
  explicit, recorded in the import report (§8), and downgrades the
  `StandardsInteroperability`/`ProvenanceGovernance` readiness dimensions
  accordingly.

No importer may silently pretend canonical-level input reconstructs rich
structure.

### 3.3 Schema is versioned; field additions are migration events

The native contract carries a schema version (`kSchemaVersion`,
`include/hacdcpf/io/schema_version.hpp`). Adding governed fields (telemetry
bindings, uncertainty, calibration history) to `DynamicModelProfile` or rich
models is a **schema-version event**, not an ad-hoc field addition. Each such
change must:

- bump the schema minor/major version per semantic-versioning rules,
- remain readable under `ImportMode::Permissive` from the prior version,
- document the migration (defaulting/coercion) for older files.

## 4. Maturity Model [CONTRACT]

### 4.1 Two orthogonal axes

Digital-twin maturity is **two independent axes**, not one ladder:

- **Fidelity axis (F0–F3):** how trustworthy the offline model is.
  F0 file-parses → F1 static network → F2 executable (checked params, canonical
  projection defined) → F3 validated (numerical cross-checks exist).
- **Integration axis (I0–I2):** how connected to operations the model is.
  I0 offline → I1 synchronized (telemetry anchors, timestamped state seeds,
  event surfaces, provenance) → I2 closed-loop (forecasting, calibration,
  control/replay, uncertainty governance).

A rigorously validated offline model is `F3/I0` and must not be penalized for
lacking live binding, nor may a lightly-validated but well-instrumented model
claim high fidelity. The legacy single label `L0–L5` is retained as a
**coarse convenience projection** of `(F,I)` for display, but `(F,I)` is the
normative representation.

| Legacy | Approx. (F, I) | Meaning |
| --- | --- | --- |
| L0 | F0, I0 | Files parse; semantics not guaranteed. |
| L1 | F1, I0 | Stable asset identity + topology. |
| L2 | F2, I0 | Native solvers run on checked parameters; canonical projection defined. |
| L3 | F3, I0 | Numerical verification exists vs. native and external references. |
| L4 | F3, I1 | Telemetry anchors, timestamped state seeds, event surfaces, provenance present. |
| L5 | F3, I2 | Uncertainty, parameter estimation, validation history, control/replay governed. |

### 4.2 Maturity gates; it does not average

**This is a correction to the current implementation.** Today
`maturity_level` is derived by bucketing a weighted-average `readiness_ratio`
(`src/io/component_io_mapping.cpp`: `readiness_ratio = score / max_score`, then
thresholds `0.15 / 0.35 / 0.55 / 0.72 / 0.88`). Averaging is **compensatory**:
strong dimensions mask absent ones, so a model can be labeled L4 with *zero*
provenance or telemetry as long as other dimensions score high enough. That
contradicts the acceptance criteria in this table.

**[CONTRACT]** Maturity is a **weakest-link** property. A model is at the
highest level `L` for which **every** gate criterion at levels `≤ L` passes:

```
maturity_level = max { L : all gate_criteria(level ≤ L) pass }
```

The gate criteria per level:

| Level | Gate criteria (all required) |
| --- | --- |
| F1 | Stable asset identity table; topology table solvable. |
| F2 | Static params pass hard gates; canonical projection defined for all populated collections. |
| F3 | ≥1 stored numerical verification (native and external) within tolerance (§9). |
| I1 | Telemetry anchors bound; ≥1 timestamped state seed; event surface present; provenance populated. |
| I2 | Uncertainty fields present; calibration/estimation history stored; control/replay workflow defined. |

The existing weighted `readiness_ratio` is retained as a **secondary
within-level completeness indicator** (how fully a level is satisfied), never as
the level selector. Implementations MUST report both the gated level and the
per-dimension ratio.

## 5. Standards Direction [CONTRACT / ROADMAP]

Standards are named **with a bounded profile and version**, not as open-ended
families. "Support IEC 61970 CIM" is unbounded and therefore not a requirement;
"support CGMES 3.0 EQ+SSH for topology, assets, base voltage, and operational
limits" is. `ComponentStandardProfile.profile` / `.model_name` are the hooks
that MUST be filled from the controlled vocabulary below.

| Standard | Bounded target profile | Scope | Status |
| --- | --- | --- | --- |
| IEC 61970/61968 CIM | **CGMES 3.0**, EQ + SSH (TP/SV later) | Topology, assets, terminals, base voltage, operational limits, naming. | [ROADMAP] — crosswalk required before parser (§13). |
| IEC 61850 | Logical-node subset for MMXU/MMTR + telemetry binding | Measurement binding, protection/control semantics. | [ROADMAP] — see §10. |
| IEC 60909 | Short-circuit source model | SC sources, motors, converter current contribution, equivalent Z. | Partial (transient params governed). |
| IEEE 1547 | DER interconnection profile | Ride-through, V/f support, trip curves, inverter functions. | [ROADMAP] validator. |
| IEEE 421.5 | Exciter/governor/machine families | Dynamics + parameter naming. | [ROADMAP] validator. |
| NERC generic IBR | REGC_A, REEC_A, REPC_A, DER_A | Plant controls + validation traces. | [ROADMAP] validator. |
| OpenDSS / GridLAB-D | AC distribution snapshot | Unbalanced PF, ZIP loads, regulators, transformers, DER. | Export full; import balanced-AC subset (§2). |
| ETAP | Excel + native XML PDE | AC/DC network + machines + protection. | Built (conditional). |
| Native HACDCPF | JSON / canonical | Lossless internal contract + solver projection. | Built. |

Each supported standard MUST ship a **field-level crosswalk table**
(HACDCPF rich field ↔ standard class/attribute) in its own doc before its
adapter is considered complete.

## 6. Parameter Governance [CONTRACT]

Each component exposes five parameter domains: **static, dynamic, transient,
failure, reliability** (as today). Governance operates at three nested layers;
the current code implements Layer 1 only, and Layers 2–3 are required additions.

### 6.1 Layer 1 — Field-level rules (built)

`ComponentParameterRule` (`include/hacdcpf/io/component_io_mapping.hpp`) checks
one `parameter_path`: type, presence (`required`), numeric range
(`min_value`/`max_value` with inclusivity), standard family/profile, units, and
`missing`/`range` severities. Retain as-is.

### 6.2 Layer 2 — Cross-field structural invariants (required)

Field-level ranges cannot catch electrically-invalid models where every field
is individually in range. A new rule type MUST express invariants over multiple
fields of a component (or across a terminal):

- Generator / DER: `Pmin ≤ Pmax ≤ Prated`; capability `P² + Q² ≤ S²`.
- Branch: `X/R` plausibility; impedance-vs-rating coherence; non-negative R.
- Transformer: tap range vs. winding configuration; off-nominal ratio bounds.
- Terminal: base-voltage consistency at both ends; per-unit vs. actual-value
  coherence.

These invariants are the **component-validation** layer named in the validation
pyramid (§9). Findings reuse `ComponentParameterFinding` with a
`cross_field` category tag and reference the set of `parameter_path`s involved.

### 6.3 Layer 3 — Unit discipline (required)

Units MUST be **declared, not inferred**, on the twin path. The current
MATPOWER path infers units heuristically ("load kW conversion heuristic",
"branch ohm conversion heuristic"); such heuristics are a correctness hazard for
a contract.

**[CONTRACT]**

- Every file/collection asserts its unit system explicitly: `Sbase`, per-bus
  `Vbase`, and `pu | SI | mixed`.
- On the twin path, heuristic unit inference is **forbidden**. Files that do not
  assert units are rejected in `Strict` mode.
- Heuristic inference is permitted **only** under an explicit
  `best_effort_import` flag, which stamps the affected values with reduced
  provenance and downgrades `ElectricalParameters` / `ProvenanceGovernance`
  readiness.

### 6.4 Provenance and uncertainty fields

Every governed parameter carries: standard family/profile; units and admissible
range; required/optional; missing/range severity; provenance (source, parameter
set, notes, calibration history); and future uncertainty fields (confidence
interval, covariance, estimation method). Uncertainty/calibration additions are
schema-version events (§3.3).

## 7. IO Conformance Contract [CONTRACT]

This is the core clause that makes the module an IO *contract* rather than a set
of converters.

### 7.1 Round-trip requirements

For every adapter, the following MUST hold within a declared tolerance `ε`
appropriate to the binding level (§3.2):

- **Rich round-trip:** `import(export(M)) ≈ M` at the rich level for
  rich-binding formats (JSON, ETAP).
- **Canonical round-trip:** `import(export(M)) ≈ canonical(M)` for
  canonical-binding formats (GridLAB-D, OpenDSS), where structural loss is
  expected and bounded.
- **Foreign round-trip (where import exists):** `export(import(F)) ≈ F` for the
  supported subset of `F`.

Each adapter publishes its tolerance `ε` and its fidelity ceiling. Existing
machinery to build on: `etap_fidelity_check`,
`compare_gridlabd_snapshot`, and the projection provenance maps.

### 7.2 Binding to readiness

The `NumericalValidation` readiness dimension MUST be backed by a **stored**
conformance result (pass/fail + residual + tolerance + timestamp), not a
self-assertion. An adapter with no stored round-trip result scores zero on that
dimension regardless of other evidence.

## 8. Import Diagnostic Contract [CONTRACT]

Today diagnostics are inconsistent per format (`ExternalGridImportReport`,
`EtapIoReport`, GUI `_io_warnings` / `_io_skipped`). All importers MUST emit a
**uniform, machine-readable report**:

```
ImportReport {
  records: [ {
    source_locator,        // file line / sheet+row / XML xpath
    target_ref,            // component ref (may be null if rejected)
    disposition,           // accepted | coerced | rejected | skipped
    reason_code,           // enumerated, stable
    severity,              // info | warning | error
    message
  } ],
  summary: { accepted, coerced, rejected, skipped },
  binding_level,           // rich | canonical (§3.2)
  unit_assertion,          // asserted | inferred | best_effort (§6.3)
}
```

`ImportMode {Strict, Permissive}` is retained: `Strict` fails on any `error` or
any `coerced`/`inferred` record; `Permissive` accepts and records them.

## 9. Validation Pyramid [CONTRACT]

Layers, now with **pass criteria** and **reference management**:

1. **Field-level** — type, units, range, required (§6.1). Pass: zero `error`.
2. **Component-level** — cross-field invariants (§6.2) for generator, ZIP load,
   transformer, regulator, line, inverter, storage, protection. Pass: zero
   invariant violations.
3. **Projection** — rich↔canonical preservation. Pass: back-projection via
   `BranchExpandMap`/`BusMergeMap` reproduces rich quantities within `ε`.
4. **Small-system** — two-bus, feeder section, DER+load, converter-coupled
   AC/DC, three-phase unbalanced. Pass: native solve matches stored reference.
5. **Medium/large** — IEEE feeders, MATPOWER AC cases, hybrid AC/DC, many DER
   controllers. Pass: within tolerance of versioned reference solutions.
6. **External cross-check** — OpenDSS / GridLAB-D snapshots (built via
   `compare_gridlabd_snapshot`, `solve_opendss_snapshot`);
   PowerSimulationDynamics.jl for dynamics. Pass: bus V/θ and branch flow
   residuals within `ε`.
7. **Replay** — event schedules, measured disturbance traces, no-event
   equilibrium residual gate. Pass: residual below gate at every step.

**Reference management [CONTRACT]:** golden reference solutions are versioned
artifacts with a recorded tolerance. Layers 4–6 are prose until references +
tolerances are checked in and wired into CI.

## 10. Temporal and Telemetry Data Model [ROADMAP → CONTRACT]

A twin is time-indexed end to end; this section is required before Integration
axis `I1` (§4).

### 10.1 Time base

- Single UTC epoch; all timestamps carry timezone; resolution declared per
  stream.
- Heterogeneous streams (SCADA seconds, PMU sub-cycle, AMI 15-min) are aligned
  by an explicit resampling/interpolation policy; a "state seed" is timestamped
  and reconciled against a snapshot solve at a stated instant.

### 10.2 Telemetry binding schema

Expanded beyond the earlier draft to be usable for state synchronization:

```
TelemetryBinding {
  component_ref, measurement_type, phase,
  unit, sign_convention,            // load vs generator
  reference_frame,                  // PMU angle reference; sequence vs phase
  source_system, tag,
  sampling_interval, deadband, scale,
  cardinality,                      // component↔tags; tag↔derived quantity
  constrains_state,                 // which state variable(s) this measures
  quality, timestamp
}
```

### 10.3 State reconciliation

`I1` requires more than raw tag binding: a state-estimation interface that
computes measured-vs-solved residuals, flags bad data, and records which state
variables each measurement constrains. Replay validation (§9.7) consumes this.

## 11. Digital-Twin Readiness Dimensions [CONTRACT]

The 11 dimensions remain: asset identity, topology connectivity, electrical
parameters, dynamic behavior, telemetry observability, state synchronization,
scenario events, reliability lifecycle, standards interoperability, numerical
validation, provenance governance.

**Tiered scoring [CONTRACT].** Dimensions that today credit mere *presence of a
name* MUST distinguish three tiers, because "declares a profile" ≠ "has a
runnable, validated model":

- **Declared** — a named profile/anchor exists (lowest credit).
- **Parameterized** — the model/binding has a complete, range-checked parameter
  set.
- **Validated** — backed by a stored numerical/replay result (§7.2, §9).

This applies especially to `DynamicBehavior` (IEEE 1547 / NERC REGC/REEC/REPC
are parameterized control blocks, not strings) and `TelemetryObservability`
(component *names* are not telemetry channels).

## 12. Security and Trust Boundary [CONTRACT]

IO is the primary attack surface and must be hardened:

- **Parser hardening:** disable XML external entities (XXE) in ETAP/CIM XML;
  guard against zip/formula bombs in `.xlsx`; enforce file-size and
  element-count caps; reject path traversal in embedded references.
- **External solvers:** invocations of `gridlabd` / `opendss` run in a
  constrained working directory with time and resource limits; their outputs
  are treated as untrusted until parsed and validated.
- **Live binding:** telemetry sources cross a trust boundary; measurements are
  quality-flagged and bad-data-checked (§10.3) before influencing state.

## 13. Roadmap — Prioritized [ROADMAP]

Ordered by value-to-effort for import/export specifically:

1. **Round-trip conformance contract (§7)** wired into `NumericalValidation`,
   reusing `etap_fidelity_check` / `compare_gridlabd_snapshot`. Highest payoff.
2. **Cross-field invariant layer (§6.2)** — where imports actually break.
3. **Unit discipline (§6.3)** — declared, not inferred, on the twin path.
4. **Gate-based maturity (§4.2)** — small change; makes the maturity table
   honest.
5. **Uniform import-diagnostic report (§8)** across all adapters.
6. **Close export gaps:** MATPOWER export; lift GridLAB-D/OpenDSS *import*
   toward export fidelity, or document the subset ceiling per §3.2.
7. **Scope CIM concretely (§5)** — CGMES 3.0 EQ+SSH crosswalk before any parser.
8. **Temporal + telemetry model (§10)** before building `I1`.

Additional committed items:

- Add parameter uncertainty and calibration metadata to `DynamicModelProfile`
  and static rich models (schema-version event, §3.3).
- Persist validation results and external-solver traces so the GUI shows
  evidence history, not only the latest score.
- Profile-specific validators for IEEE 1547, IEEE 421.5, NERC
  REGC/REEC/REPC, GENROU/GENSAL, TGOV/HYGOV, and ZIP loads.

## 14. GUI Direction [CONTRACT]

`数据IO` stays the single user-facing place for: model compatibility; input and
output format conversion; standards profile registry; parameter rule browsing;
parameter health diagnostics; digital-twin readiness and maturity visualization;
and future telemetry-binding and calibration dashboards.

Checks launched from transient-simulation or reliability modules MUST route
results **back** to the unified IO dashboard rather than scattering
import/export and standards checks across panels. Centralization is enforced by
the API contract: GUI code consumes `/api/io/model_compatibility` and MUST NOT
duplicate C++ scoring logic in JavaScript. As telemetry/calibration/validation
history are added, the endpoint payload is segmented/paginated so it does not
become a monolith.

# IO Governance and Digital-Twin Readiness Review {#io_digital_twin_review}

This page is the code-review guide for the `数据IO` governance layer.  It is
generated into the Doxygen notebook together with the public C++ API.

## Review Targets

- Public contract:
  `include/hacdcpf/io/component_io_mapping.hpp`
- Registry, audit, and gated scoring implementation:
  `src/io/component_io_mapping.cpp`
- Uniform import-diagnostic contract (§8):
  `include/hacdcpf/io/import_report.hpp`, `src/io/import_report.cpp`
- Round-trip / IO conformance (§7):
  `include/hacdcpf/io/roundtrip.hpp`, `src/io/roundtrip.cpp`
- Telemetry + temporal model (§10):
  `include/hacdcpf/model/telemetry.hpp`, serialization in `src/io/json_io.cpp`
- CIM / CGMES 3.0 bounded adapter (§5):
  `include/hacdcpf/io/cim_io.hpp`, `src/io/cim_io.cpp`,
  crosswalk `docs/cim_cgmes3_crosswalk.md`
- Unit discipline + MATPOWER export (§6.3, §13):
  `src/io/matpower_parser.cpp`
- Schema versioning (§3.3):
  `include/hacdcpf/io/json_io.hpp` (`check_schema_version`, `try_from_json` mode)
- GUI/API endpoint:
  `tests/run_gui_server.cpp`, route `/api/io/model_compatibility`
- Front-end dashboard:
  `web/js/app.js` and `web/css/style.css`
- Focused regression tests:
  `tests/test_component_io_mapping.cpp`, `tests/test_io_matpower.cpp`,
  `tests/test_io_roundtrip.cpp`, `tests/test_io_cim.cpp`
- Theory and roadmap:
  `docs/digital_twin_data_io_architecture.md`

## Conceptual Contract

The IO layer has three nested responsibilities:

1. Component compatibility:
   each rich component collection declares its preservation policy for native
   JSON, canonical projection, GridLAB-D, and OpenDSS.
2. Parameter governance:
   static, dynamic, transient, failure, and reliability parameters are checked
   against a standards-aware rule catalog.
3. Digital-twin readiness:
   the loaded model is scored over asset identity, topology, electrical
   parameter health, dynamic profiles, telemetry anchors, state seeds, event
   surfaces, reliability metadata, standards interoperability, numerical
   validation, and provenance.

## API Objects

- `ComponentIOMapping` describes one component collection and target-format
  policy tuple.
- `ComponentStandardProfile` links a component to standards such as CIM,
  IEC 61850, IEC 60909, IEEE 1547, IEEE 421.5, NERC, GridLAB-D, and OpenDSS.
- `ComponentParameterRule` is a GUI-visible rule for one parameter path.
- `ComponentParameterAuditReport` is the result of applying those rules to a
  `HybridPowerSystem`.
- `DigitalTwinReadinessCriterion` is a weighted maturity criterion.
- `DigitalTwinReadinessReport` carries the **two gated axes** — `fidelity_level`
  (F0--F3) and `integration_level` (I0--I2) — plus the weakest-link `gates`
  vector, the legacy L0--L5 projection (`maturity_level`), the secondary
  `readiness_ratio`, and stored `round_trip_evidence` (§4, §7.2).
- `DigitalTwinMaturityGate` is one weakest-link gate result (F1..F3 / I1..I2).
- `ImportReport` / `ImportRecord` are the uniform import-diagnostic contract with
  disposition, reason code, severity, binding level, and unit assertion (§8);
  `passes_mode` enforces Strict/Permissive.
- `RoundTripEvidence` is stored conformance evidence from `json_roundtrip` /
  `diff_systems` (§7).
- `TelemetrySection` (`TelemetryStream`, `TelemetryBinding`, `StateSeed`,
  `TimeBase`) is the optional integration-axis state on `HybridPowerSystem` (§10).
- `CimImportResult` is the bounded CGMES 3.0 import result (§5).

## Code-Review Checklist

- Confirm every public component collection has an entry in
  `component_io_mappings()`.
- Confirm external-format losses are explicit through
  `ComponentIOPolicy`, not hidden in import/export code.
- Confirm new rich-model fields either have JSON round-trip coverage or are
  explicitly documented as gaps.
- Confirm dynamic-capable components have `DynamicModelProfile` preservation
  through rich-to-canonical projection.
- Confirm new parameter families add `ComponentParameterRule` entries with
  units, bounds, standard family/profile, and severities.
- Confirm no-event equilibrium, external solver cross-checks, and component
  comparison harnesses are represented by `NumericalVerificationScope` before
  equivalence claims are made.
- Confirm GUI additions consume `/api/io/model_compatibility` rather than
  duplicating C++ scoring logic in JavaScript.

## Expected API Payload

`/api/io/model_compatibility` returns:

- `mappings`
- `coverage`
- `summary`
- `diagnostics`
- `parameter_rules`
- `parameter_audit`
- `digital_twin_criteria`
- `digital_twin_readiness`

For a loaded built-in system, the readiness object should include a summary,
eleven dimension scores, and one finding per readiness criterion.

## Validation Commands

From the repository root:

```bash
cmake --build build/macos-release --target run_gui_server \
  test_component_io_mapping test_io_matpower test_io_roundtrip test_io_cim -j8
./build/macos-release/tests/test_component_io_mapping
./build/macos-release/tests/test_io_matpower
./build/macos-release/tests/test_io_roundtrip
./build/macos-release/tests/test_io_cim
node --check web/js/app.js
node --check web/js/components.js
```

Optional GUI/API smoke:

```bash
./build/macos-release/run_gui_server --port 8097
curl -X POST http://127.0.0.1:8097/api/session/load_builtin \
  -H 'Content-Type: application/json' \
  -d '{"case":"ieee24_3area_acdc_expanded"}'
curl http://127.0.0.1:8097/api/io/model_compatibility
```

## Implemented (previously roadmap)

- **Gated maturity** over two axes (fidelity / integration), replacing the
  compensatory weighted average; the legacy L0--L5 label is now a projection.
- **Cross-field electrical invariants** (capability, X/R, base-voltage coherence).
- **Unit discipline** on the MATPOWER path (no heuristic inference on the twin
  path unless `best_effort_import`).
- **Round-trip conformance** (JSON/generic field diff) gating `NumericalValidation`.
- **Schema versioning**: `check_schema_version` implemented, `try_from_json`
  honors `ImportMode`, schema stamped and bumped to 1.1.
- **Telemetry + temporal model** (§10) attached to `HybridPowerSystem`, JSON
  round-tripped, driving the I1 gate.
- **CIM / CGMES 3.0 (EQ+SSH)** bounded import/export, XXE-hardened (§5).
- **MATPOWER export** (`to_matpower`), closing import/export symmetry.
- **Uniform ImportReport** contract with a bridge for legacy per-format reports.

## Future Review Areas

- State reconciliation / WLS bad-data engine (§10.3) — planned as a separate
  standalone function.
- CIM transformers, TP/SV profiles, and full ENTSO-E boundary sets (beyond the
  bounded EQ+SSH ceiling documented in `docs/cim_cgmes3_crosswalk.md`).
- IEC 61850 telemetry binding to live sources; measurement quality in the loop.
- Profile-specific validators for IEEE 1547, IEEE 421.5, NERC REGC/REEC/REPC,
  GENROU/GENSAL, and ZIP load profiles.
- GUI surfacing of the F/I gates, round-trip evidence, telemetry, and import
  reports in `/api/io/model_compatibility` and the front-end dashboard.
- Parameter uncertainty, estimator provenance, and calibration history.

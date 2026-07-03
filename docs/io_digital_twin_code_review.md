# IO Governance and Digital-Twin Readiness Review {#io_digital_twin_review}

This page is the code-review guide for the `数据IO` governance layer.  It is
generated into the Doxygen notebook together with the public C++ API.

## Review Targets

- Public contract:
  `include/hacdcpf/io/component_io_mapping.hpp`
- Registry, audit, and scoring implementation:
  `src/io/component_io_mapping.cpp`
- GUI/API endpoint:
  `tests/run_gui_server.cpp`, route `/api/io/model_compatibility`
- Front-end dashboard:
  `web/js/app.js` and `web/css/style.css`
- Focused regression tests:
  `tests/test_component_io_mapping.cpp`
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
- `DigitalTwinReadinessReport` is the scored L0--L5 readiness result.

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
cmake --build build/macos-release --target run_gui_server test_component_io_mapping -j8
./build/macos-release/tests/test_component_io_mapping
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

## Future Review Areas

- CIM package import/export for topology and asset identity.
- IEC 61850 telemetry binding schema and measurement quality flags.
- Profile-specific validators for IEEE 1547, IEEE 421.5, NERC REGC/REEC/REPC,
  GENROU/GENSAL, and ZIP load profiles.
- Persisted validation evidence and external solver trace history.
- Parameter uncertainty, estimator provenance, and calibration history.

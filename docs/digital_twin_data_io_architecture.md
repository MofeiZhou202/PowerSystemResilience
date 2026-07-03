# Data IO to Digital Twin Architecture

## Motivation

The `数据IO` module should not remain a collection of file import/export buttons. For a distribution-system digital twin, IO is the model contract that decides whether a network model can be exchanged, solved, validated, synchronized with measurements, calibrated, and audited over time.

The current enhancement adds a standards-aware readiness layer on top of the rich model registry:

- Component IO mapping: native JSON, canonical projection, GridLAB-D, and OpenDSS representation policy.
- Parameter governance: static, dynamic, transient, failure, and reliability rule ranges with severity.
- Digital-twin readiness: a maturity score over identity, topology, parameters, dynamics, telemetry, state, events, reliability, standards, validation, and provenance.

## Maturity Model

The recommended maturity levels are:

| Level | Meaning | Acceptance criterion |
| --- | --- | --- |
| L0 file exchange | Files parse, but model semantics are not guaranteed. | JSON import/export does not crash. |
| L1 static network model | Assets and topology can be displayed and saved. | Stable asset identity and topology tables exist. |
| L2 executable simulation model | Native solvers can run using checked parameters. | Static parameters pass hard gates and canonical projection is defined. |
| L3 validated model | Native results are cross-checkable. | Component/system numerical verification exists against native and external references. |
| L4 synchronized operational twin | Model can bind live or historical measurements. | Telemetry anchors, state seeds, timestamped event surfaces, and provenance are present. |
| L5 predictive closed-loop twin | The twin supports forecasting, calibration, control, and lifecycle governance. | Uncertainty, parameter estimation, validation history, and control/replay workflows are governed. |

## Standards Direction

The IO layer should use standards profiles instead of one-off field names:

- IEC 61970/61968 CIM: topology, assets, terminals, base voltage, operational limits, and naming.
- IEC 61850: telemetry binding, logical nodes, protection/control semantics, and substation automation metadata.
- IEC 60909: short-circuit sources, motors, converter current contribution, and equivalent impedances.
- IEEE 1547: DER ride-through, voltage/frequency support, trip curves, and inverter functions.
- IEEE 421.5 and IEEE synchronous-machine families: exciters, governors, machine dynamics, and parameter naming.
- NERC generic IBR models: REGC_A, REEC_A, REPC_A, DER_Aggregate, plant controls, and validation traces.
- OpenDSS and GridLAB-D: distribution AC network cross-check targets, especially unbalanced power flow, ZIP loads, regulators, transformers, and DER snapshots.
- Native HACDCPF JSON/canonical models: the lossless internal contract and solver-facing projection.

## Parameter Governance

Each component should expose five parameter domains:

- Static: nameplate, topology, impedance, limits, operating point.
- Dynamic: machine, governor, exciter, PLL, converter, load, and storage state models.
- Transient: current limits, duty/modulation feasibility, protection timing, fault parameters.
- Failure: failure rates, forced-outage rates, breaker/switch failure probability.
- Reliability: MTTR, MTBF, scheduled outage time, lifecycle and degradation metadata.

Every governed parameter should carry:

- Standard family/profile.
- Units and admissible range.
- Required/optional status.
- Severity for missing and out-of-range values.
- Provenance fields for source, parameter set, notes, and calibration history.
- Future uncertainty fields such as confidence interval, covariance, and estimation method.

## Digital-Twin Readiness Dimensions

The implemented readiness registry uses these dimensions:

- Asset identity: stable indices and names for every object.
- Topology connectivity: enough bus/edge structure to solve networks.
- Electrical parameters: range-gated checked parameters.
- Dynamic behavior: named dynamic model profiles for transient-capable devices.
- Telemetry observability: anchors for SCADA, PMU, AMI, DERMS, and historian streams.
- State synchronization: voltage, SOC, dispatch, and status seeds.
- Scenario events: exposed disturbances, faults, trips, references, and restoration actions.
- Reliability lifecycle: failure/repair/lifecycle metadata.
- Standards interoperability: declared JSON/canonical/GridLAB-D/OpenDSS policies.
- Numerical validation: native and external solver verification scopes.
- Provenance governance: source identifiers, standards, parameter sets, and diagnostics.

## Validation Pyramid

The validation workflow should be layered:

1. Field-level validation: type checks, units, range gates, required fields.
2. Component validation: generator, ZIP load, transformer, regulator, line, inverter, storage, and protection model behavior.
3. Projection validation: rich-to-canonical and canonical-to-rich preservation checks.
4. Small-system validation: two-bus, feeder section, DER+load, converter-coupled AC/DC, and three-phase unbalanced circuits.
5. Medium/large system validation: IEEE feeders, MATPOWER-style AC cases, hybrid AC/DC cases, and many DER controllers.
6. External solver cross-check: OpenDSS and GridLAB-D for AC distribution snapshots; PowerSimulationDynamics.jl for dynamic controls where applicable.
7. Replay validation: event schedules, measured disturbance traces, and no-event equilibrium residual gates.

## GUI Direction

The GUI should keep `数据IO` as the single user-facing place for:

- Model compatibility.
- Input format conversion.
- Output format conversion.
- Standards profile registry.
- Parameter rule browsing.
- Parameter health diagnostics.
- Digital-twin readiness and maturity visualization.
- Future telemetry binding and calibration dashboards.

This avoids scattering import/export, transient compatibility, and standards checks across unrelated panels. The GUI can still launch these checks from transient simulation or reliability modules, but the result should route back to the unified IO dashboard.

## Future Work

Recommended next steps:

- Add a telemetry binding schema: `{component_ref, measurement_type, phase, unit, source_system, tag, quality, timestamp}`.
- Add parameter uncertainty and calibration metadata to `DynamicModelProfile` and static rich models.
- Add bidirectional CIM package import/export for topology and asset identity.
- Add an explicit event catalog schema shared by GUI, JSON, transient simulation, GridLAB-D, OpenDSS, and PSD comparison harnesses.
- Add profile-specific validators for IEEE 1547, IEEE 421.5, NERC REGC/REEC/REPC, GENROU/GENSAL, TGOV/HYGOV, and ZIP load standards.
- Persist validation results and external solver traces so the GUI can show evidence history, not only the latest score.

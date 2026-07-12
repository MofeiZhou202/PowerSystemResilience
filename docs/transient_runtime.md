# Transient Runtime Contract

Updated: 2026-07-12

The transient module is a phasor-domain electromechanical simulation. Its
implementation is under `include/hacdcpf/dynamics` and `src/dynamics`.
Mathematical background is retained in
`dynamics_electromechanical_transient_design.md`; this document defines current
runtime behavior.

## Solve path

`POST /api/session/run_transient` builds a `DynamicSystem`, optionally performs
PF initialization and dynamic trimming, applies scheduled events, integrates
the model, and records selected snapshots. The response includes initialization
diagnostics, solver statistics, bus voltage/frequency matrices, device series,
applied event records, warnings, and optional modal/CSV data.

The server caches the serialized rich-space result. The GUI retrieves one
sample with:

```http
GET /api/session/transient/frame?index=N
```

## What a frame may display

- authored AC and DC bus voltage;
- authored AC bus frequency;
- dynamic device P/Q, current, controller/state, and in-service metrics that
  were actually recorded;
- SOC only when an SOC/state-of-charge series exists;
- applied event location and active/applied state.

## Prohibited inference

A transient frame must not compute branch P/Q from the static PF formula.
Dynamic branch arrows are permitted only after the dynamic solver records
terminal current or terminal P/Q for that sample. Until then the frame returns:

```json
{
  "capabilities": { "branch_power": false },
  "geo_ac_branches": [],
  "geo_dc_branches": [],
  "ac_circuit_breaker_flows": []
}
```

## Validation boundaries

GridLAB-D/OpenDSS comparisons validate only their explicitly shared model
scope. A balanced snapshot comparison does not certify dynamic inverter,
machine, DC-link, storage, or event equations. Voltage-health failures and
non-converged initialization remain visible errors; playback must not relabel a
failed run as converged.

`power_system.json` can expose a very large initial DC voltage when the dynamic
health guard is enabled. That is a dynamic-model/initialization defect, not a
Canvas mapping issue, and must be resolved in the solver rather than hidden in
the renderer.


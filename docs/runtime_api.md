# Runtime API Contract

Updated: 2026-07-12

The GUI server is implemented in `tests/run_gui_server.cpp`. Session endpoints
operate on one loaded `HybridPowerSystem`; a model-changing request clears
cached analysis results.

## Session lifecycle

| Method and route | Purpose |
|---|---|
| `POST /api/session/new_empty` | Create an empty system. |
| `POST /api/session/load_builtin` | Load a built-in case. |
| `POST /api/session/load_json_string` | Load authored rich-model JSON. |
| `POST /api/session/load_matpower` | Load a MATPOWER case. |
| `POST /api/session/update_components` | Apply component changes. |
| `GET /api/session/status` | Read current session state. |
| `POST /api/session/cancel` | Request cancellation of a long analysis. |

The GUI calls `syncToBackend()` before analyses when the authored Canvas model
is dirty. Result playback never synchronizes or changes the model.

## Primary analyses

| Route | Result family |
|---|---|
| `POST /api/session/pf` | Authored-space PF result with rich component attribution and P/Q diagnostics. |
| `POST /api/session/opf` | OPF plus authored-space post-PF Canvas payload. |
| `POST /api/session/run_ts_pf` | UC/OPF/PF time-series summary and cached per-step results. |
| `POST /api/session/run_annual_sim` | Annual production simulation and aggregate statistics. |
| `POST /api/session/run_carbon` | Static carbon flow. |
| `POST /api/session/run_dynamic_carbon` | Dynamic carbon flow using cached or newly solved TSPF states. |
| `POST /api/session/run_transient` | Phasor-domain transient result and cached telemetry. |
| `POST /api/session/small_signal` | Modal analysis at the initialized dynamic operating point. |
| `POST /api/session/sc`, `dc_sc`, `sc_detailed` | AC/DC short-circuit analyses. |
| `POST /api/session/harmonics*` | Harmonic PF, three-phase, frequency scan, metrics, and Newton variants. |
| `POST /api/session/run_reliability*` | Non-sequential, sequential, FMEA, feeder, and three-stage reliability. |
| `POST /api/session/run_reconfig` | Topology reconfiguration. |

Specialized production routes for resilience, hosting capacity, campus IES,
EV traffic, lifecycle, scenario generation, and SPPT remain discoverable in
the server source. Legacy embedded-UI routes are not part of this contract.

## Lazy Canvas frames

Large analyses cache solver results server-side and expose one frame at a time:

```http
GET /api/session/tspf/frame?step=12
GET /api/session/transient/frame?index=120
```

Both return `schema: "canvas_frame_v1"`, `analysis`, `index`, `count`, `time`,
`time_unit`, `converged`, `capabilities`, and authored-space component data.
Missing caches return HTTP 400. Out-of-range indices return HTTP 416.

TSPF frames may contain bus voltages, AC/DC branch flows, switch and circuit
breaker terminal P/Q, Grid/generator/storage dispatch, VSC/DC-DC transfers,
SOC, transformer terminal flows, and bus balance diagnostics.

Transient frames contain only recorded dynamic quantities: bus voltage,
frequency, device metrics, and applied event state. Their
`capabilities.branch_power` is false and branch/CB flow arrays are empty unless
the dynamic solver is later extended to record real terminal current or P/Q.

## Error and cache rules

- Heavy analyses return HTTP 409 when another analysis owns the session.
- Invalid input and missing prerequisites return HTTP 400 with `error`.
- Frame responses use `Cache-Control: no-store`; the server-side session cache
  is the source of truth.
- Loading or changing a model invalidates PF, TSPF, and transient caches.
- A consumer must check `converged`/`success` and capability flags before
  interpreting optional result arrays.

## Verification

```bash
python3 tools/gui_api_e2e.py \
  --server build/tests/run_gui_server \
  --data-dir data --skip-etap
```

The E2E suite covers PF/OPF Canvas reprojection, Grid and CB P/Q, TSPF frame
retrieval, P/Q diagnostics, and the transient no-fabricated-flow contract.


# Runtime API Contract

Updated: 2026-07-19

The GUI server is implemented in `tests/run_gui_server.cpp`. Session endpoints
operate on one loaded `HybridPowerSystem`; a model-changing request clears
cached analysis results.

## Version 1 multi-session API

New automation and AI clients should use `/api/v1`. These resources are
independent of the GUI's process-global `/api/session/*` state.

| Method and route | Contract |
|---|---|
| `GET /api/v1` | Capability and bounded-resource declaration. |
| `POST /api/v1/sessions` | Create an empty session or load `case`, `model`, or `json_string`; returns `201`, `Location`, and `ETag`. |
| `GET /api/v1/sessions` | List in-memory sessions. |
| `GET /api/v1/sessions/{session_id}` | Read model summary and current revision. Supports `If-None-Match`. |
| `GET /api/v1/sessions/{session_id}/model` | Read the authored rich model and its ETag. |
| `GET /api/v1/sessions/{session_id}/topology` | Read a paged LOD0/1/2 topology chunk; accepts `offset`, `limit`, and a complete `xmin,ymin,xmax,ymax` viewport. |
| `GET /api/v1/sessions/{session_id}/subgraph` | Read a bounded k-hop subgraph around the required domain-qualified `domain,index` bus reference. |
| `PUT /api/v1/sessions/{session_id}/model` | Atomically replace the model and increment revision. Requires the current `If-Match`. |
| `DELETE /api/v1/sessions/{session_id}` | Delete a session with no active jobs. Requires `If-Match`. |
| `POST /api/v1/sessions/{session_id}/jobs` | Snapshot the matched model revision and enqueue PF or OPF; returns `202` and `Location`. |
| `GET /api/v1/sessions/{session_id}/jobs` | List that session's retained jobs without result arrays. |
| `GET /api/v1/jobs/{job_id}` | Poll state and retrieve the terminal result. |
| `GET /api/v1/jobs/{job_id}/frames/{step}` | Read one time step with domain, stable-index, viewport, offset, and limit filters. Static PF/OPF has step 0. |
| `GET /api/v1/jobs/{job_id}/violations` | Read the worst voltage/loading violations for one step without downloading the full frame. |
| `POST /api/v1/jobs/{job_id}/cancel` | Cancel queued work immediately or mark running work as cancelling. |
| `DELETE /api/v1/jobs/{job_id}` | Remove a retained terminal job. |

The model ETag is a strong opaque value such as
`"hysim-ses-...-r3"`. Missing `If-Match` returns HTTP 428; a stale value
returns HTTP 412 with `current_etag` and `current_model_revision`. A submitted
job owns an immutable model snapshot, so a later model replacement cannot alter
the solve. Polling reports `stale_against_current_model`; the nested
`hysim_result_v1` contract repeats the job and current revisions.

The bounded in-memory worker pool defaults to two threads and is configurable
with `--api-job-workers 1..32`. The process retains at most 128 sessions and
4096 jobs. Terminal jobs should be deleted by the client. Storage is not durable
across server restarts.

Version 1 currently accepts `power_flow` with `method=ac_newton` and
`optimal_power_flow` with `network_model=balanced_aggregate` using parity,
Ipopt, auto, economic dispatch, or DC. Other production analyses remain on the
legacy routes until their result contracts are migrated.

Queued cancellation is immediate. A running PF/OPF solver currently has no
cooperative cancellation token; its state becomes `cancelling`, its eventual
result is discarded, and its terminal state becomes `cancelled`.

Topology LOD2 nodes expose stable `ref: {domain,index}` values. Edge references
also carry `resource` so a branch, transformer, and converter cannot collide.
LOD1 aggregates by domain/area/zone; LOD0 aggregates by electrical domain.
Spatial coordinates are an API layout space, not graph indices or engineering
identity. Pagination is applied to nodes and only edges whose endpoints are in
the returned node chunk are included.

Frame chunks return authored-space result rows. `indices=1,7,20` always means
stable component indices in the selected domain. A viewport filter is available
while the matching session revision still exists. The violation endpoint keeps
the largest engineering excursions first so overview clients can inspect the
worst state before requesting dense detail.

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
| `POST /api/session/pf` | Authored-space PF result with rich component attribution and P/Q diagnostics. `method=three_phase_hybrid` selects the monolithic unbalanced abc/DC/VSC Newton path. |
| `POST /api/session/opf` | OPF plus authored-space post-PF Canvas payload. `network_model=three_phase_hybrid` selects Full/GraphReduced phase-domain hybrid OPF and same-model PF replay. |
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

The v1 isolation and concurrency contract is covered independently:

```bash
python3 tools/runtime_api_v1_e2e.py \
  --server build/macos-release/tests/run_gui_server --data-dir data
```

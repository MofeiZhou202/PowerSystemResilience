# Runtime API Contract

Updated: 2026-08-10

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

## Reliability and protection configuration

The process-global GUI session exposes a model-bound reliability configuration:

| Method and route | Contract |
|---|---|
| `GET /api/session/reliability/configuration` | Return schema, sparse saved overrides, effective failure modes, stable component inventory, coverage, support diagnostics, and validation. |
| `POST /api/session/reliability/configuration/validate` | Parse and validate a candidate without changing session state. |
| `POST /api/session/reliability/configuration` | Validate, resolve protection-zone references, save atomically, and clear cached analyses. |

Failure-mode overrides are keyed by `mode_id`; protection references use
`component_kind + component .index` stable IDs such as `ac_transformer_2w:7`.
Vector position is returned only as `component_position` for diagnostics and is
not an external identity. Duplicate `.index` values within one component kind
are rejected because they would make the stable mapping ambiguous.

The mode schema exposes optional `enabled`, hazard (`failure_rate_per_year`,
`mtbf_hours`, `forced_outage_rate`), repair/duration, conditional/demand
probability, cyber recovery, and residual-capacity fields. At most one hazard
form may be supplied. Effective precedence is user mode override, user
protection configuration, authored case data, built-in component template,
then the selected missing-data policy.

Every `effective_modes[]` row also returns `resolved_parameters` with the same
12 numeric field names used by the writable schema. These values are the
canonical/equivalent result after applying that precedence: resolved annual
frequency, equivalent MTTF/MTBF, repair duration, steady unavailability,
conditional and demand probabilities, stage durations, cyber recovery, and
residual capacity. They are display values, not saved overrides. The GUI marks
them as inherited and posts only fields the user actually changes; choosing a
different hazard representation replaces the prior hazard override instead of
submitting mutually exclusive forms together.

Protection rows map one protective device to a protected component, optional
backup device, and explicit protection-zone component IDs. Fail-to-trip and
fail-to-open probabilities, nuisance-trip frequency, clearing times, automatic
reclose, and reclose-success probability round trip through the same schema.
Failure-mode FMEA consumes device failure probabilities, nuisance trips, and
zone mutations. Three-stage restoration consumes protection rows, configured
clearing times, automatic-reclose probability, primary/backup dependability,
and backup-zone expansion; it does not consume `mode_overrides`.

For a physical initiating frequency `lambda`, reclose-success probability
`r`, and primary/backup failure probabilities
`q=1-(1-p_fail_to_trip)*(1-p_fail_to_open)`, the three-stage route generates
mutually exclusive sustained scenarios:

```text
lambda_transient  = lambda*r
lambda_primary    = lambda*(1-r)*(1-q1)
lambda_backup     = lambda*(1-r)*q1*(1-q2)
lambda_unresolved = lambda*(1-r)*q1*q2
```

Successful reclose is reported as a momentary frequency and excluded from the
sustained IEEE 1366 SAIFI/SAIDI/EENS aggregation. Primary and backup scenarios
use their configured clearing times. Backup clearance removes every supported
stable component in `zone_component_ids`; unresolved primary-plus-backup
failure blocks restoration until clearance. A matching protection row for the
backup device supplies `q2`; otherwise the response declares the assumed
upstream-backup boundary. Initiating frequency is conserved between transient
and sustained scenarios.

This is a probability-conditioned protection event abstraction, not a relay
time-current simulation. It does not calculate pickup from short-circuit
current, direction/distance/differential selectivity, setting coordination,
breaker mechanics, DER ride-through, or GFM/GFL dynamic feedback. Configured
nuisance-trip frequency remains a failure-mode-FMEA input and is not yet a
three-stage recovery scenario.

Saved overrides are sparse. The GUI does not turn every effective built-in row
into an explicit user value. Loading a different built-in or imported model, or
replacing component arrays through `/api/session/update_components`, clears the
configuration. `/api/session/load_json_string` also clears it unless
the caller sets `preserve_reliability_configuration: true`; the GUI uses that
flag only for a same-canvas synchronization, after which stable references are
validated again before reliability execution. Saving or importing a non-empty
custom configuration makes the GUI select `failure_mode_fmea`, because it is
the only method that consumes failure-mode IDs; users may then explicitly
select three-stage restoration to apply the protection rows only.

Every `POST /api/session/run_reliability` response includes a
`reliability_configuration` audit object. `configured` reports whether the
session has sparse mode/protection rows. Failure-mode FMEA consumes mode and
protection overrides. Three-stage restoration reports `applied: true` and
`applied_scope: ["protection"]` only when at least one enabled protection row
matches an enumerated contingency; otherwise its scope is empty and its
limitation explains unmatched rows or ignored mode overrides. Other methods
return `applied: false` plus a non-empty `limitation` when saved configuration
exists; they never silently imply that custom values affected the run.

`coverage.modes_unsupported` is derived from the same consequence-patch
decision returned as `effective_modes[].supported`; it is not a catalog-only
estimate. For `dist33_microgrid_der` the current catalog has 228 modes, of
which 54 have no consequence representation: 33 AC-bus, 2 DC-bus, 14
AC-switch, 3 flexible-load, and 2 VSC measurement/control modes. Their editable
reliability parameters remain available, but the response carries a specific
`unsupported_reason` rather than treating their impact as zero.

The GUI workflow distinguishes a completed approximation from a failed solve.
A non-empty `model_limitations` makes consequence mapping visibly limited even
when metrics were calculated. False validity fields whose names express
validity, convergence, enforcement, or solver admission make solving/recovery
limited. A transport error or JSON `error` is a failed stage instead.

For `dist33_microgrid_der`, ordinary FMEA currently completes with
`physical_model=hybrid_network_lp` and `model_scope=hybrid-acdc-network-lp`.
It includes active-power AC/DC restoration, but does not certify nonlinear AC
voltage/reactive feasibility and does not search DC-side repair switching.
Three-stage evaluation uses
`model_scope=coupled-acdc-lindistflow-restoration-milp` and returns `ok=true`
with restoration-MILP, branch-flow, voltage, radial-topology, DC-power-flow,
and VSC/SOP-dispatch validity flags true. Fault rows return signed VSC dispatch
for every stage. The pure-AC `dist33_tie_demo` remains the corresponding
AC-only certified case. Both three-stage HTTP routes execute the solver on a
dedicated large-stack worker so a solver stack requirement cannot terminate an
HTTP worker process.

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

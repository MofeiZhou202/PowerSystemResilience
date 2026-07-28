# Canvas Runtime and Result Playback

For large models the main viewport is no longer an empty headless surface.
`web/js/core/network_overview.js` renders the full bus/primary-edge graph with
WebGL2 and LOD0/1/2 aggregation while creating no per-bus SVG DOM. The existing
SVG canvas remains authoritative for normal-size authored diagrams. Large-model
selection uses one `{domain,index}` bus reference across WebGL, the virtualized
topology tables, result navigation, and the bounded local SVG subgraph.

The intended division of work is:

- WebGL: pan, zoom, hit-test, and inspect the full network.
- Local SVG: inspect/select a bounded k-hop neighborhood.
- Virtual tables: edit the complete authored model without materializing rows.
- Charts: browse downsampled time windows while retaining complete export data.
- Backend chunks: serve topology LOD, spatial windows, time frames, and worst
  violations through `/api/v1`.

Updated: 2026-07-28

The authored model lives in `Canvas.state`, while analysis results live in SVG
result overlays. These states must remain separate.

## Static results

`Canvas.showPowerFlowResults(result)` is the static PF/OPF presentation path.
It can refresh solved component displays and the visualization overlay.
PF and OPF post-PF payloads share authored-space component keys and terminal
flow fields.

## Time playback

`CanvasPlaybackController` in `web/js/app.js` provides:

- time slider;
- play/pause and previous/next frame;
- 1x, 2x, and 5x speed;
- current frame time and convergence state;
- voltage, flow, frequency, P/Q, and SOC modes;
- lazy frame fetch with stale-request cancellation and a bounded frame cache.

`Canvas.renderFrame(frame)` in `web/js/canvas.js` accepts
`canvas_frame_v1`. It removes and updates only `.frame-overlay` and
`.viz-overlay` elements. It must not:

- write `component.params`;
- rebuild components or connections;
- synchronize the model to the backend;
- reuse static PF equations for transient branch arrows.

TSPF flow/PQ modes may reuse solved static branch geometry because the frame
contains actual per-step PF terminal powers. Transient rendering is limited to
recorded voltage, frequency, device telemetry, SOC/state, and active event
markers. Flow mode is disabled when `capabilities.branch_power` is false.

## Identity and units

Frame component lookup uses `canvas_type`, `canvas_index`, and `position`, with
explicit aliases such as `switch -> switch_comp` and
`renewable_generator -> renewable_gen`. Power values are stored in MW/MVar;
the Canvas display converts both P and Q consistently when MW/kW/W is changed.

## Performance

The backend does not send thousands of complete Canvas frames with the main
analysis response. Playback requests one frame by index and caches a small
window in the browser. This keeps long runs responsive and avoids contaminating
the next authored-model synchronization.

Large headless systems with at least 5,000 buses request
`POST /api/session/pf` with `response_detail=compact`. This response keeps the
complete solved vectors:

- `vm` and `va`: AC bus vector position, pu and radians;
- `vdc`: DC bus vector position, pu;
- `branch_abs`: authored AC branch-flow position, absolute MW.

It omits `geo_*`, `component_results`, and rich-to-canonical attribution rows
whose construction scales with every authored component. The response declares
this boundary through `model_scope`, `model_limitations`, and
`presentation_omitted`; the GUI renders vector statistics instead of
materializing one HTML row per bus or branch. Normal-size systems continue to
use the default `response_detail=full` contract.

PF responses expose snapshot, setup, solve, presentation, finalization, and
serialization durations through the JSON `timing` object and the HTTP
`Server-Timing` header. Large MATPOWER loads use one structured `_system_json`
model payload, avoiding a second escaped copy and an additional browser
`JSON.parse`; smaller loads retain the legacy response for API compatibility.

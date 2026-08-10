# Canvas Runtime and Result Playback

For large models the main viewport is no longer an empty headless surface.
`web/js/core/network_overview.js` renders the full bus/primary-edge graph with
WebGL2 and LOD0/1/2 aggregation while creating no per-bus SVG DOM. The existing
SVG canvas remains authoritative for normal-size authored diagrams. Large-model
selection uses one `{domain,index}` bus reference across WebGL, the virtualized
topology tables, result navigation, and the bounded local SVG subgraph.
For normal SVG models, topology-table component rows use stable component
`.index` mappings. A two-winding transformer extracted from an AC branch is
registered under both its branch index and transformer index, so either table
selects the same transformer glyph rather than losing the rich-device link.

The component property editor groups authored fields by semantics: identity
and connection, electrical/rating, operation/control, protection/safety,
cost/carbon, reliability, analysis-specific, data/source, and dynamics. The
reliability group is present for every selected Canvas component and resolves
its failure modes from the backend `component_kind + component .index`
inventory. Energy-router port modes are shown on their parent router, and a
branch-backed transformer retains both branch and transformer aliases. All 12
backend numeric mode fields are displayed; inherited effective values use a
dashed style, while Apply posts only fields actually edited by the user.

The Model Parameters module follows the same identity rule for its read-only
current-value snapshot: `domain:component_kind:component .index`. Its profile
table edits registered defaults and validation policy; authored instance values
remain owned by the Canvas property editor. While this module is active,
selecting a mapped Canvas glyph updates both the component-model filter and the
exact current instance. AC/DC buses remain domain-qualified, and a
branch-backed transformer prefers its dedicated transformer identity when that
alias exists, otherwise its AC-branch identity. The selection refresh preserves
unsaved profile-table edits. A displayed typical range or profile default
therefore cannot silently become an instance override.

The component-model selector is populated from the backend `model_catalog`,
currently 44 serializable physical/system families, rather than from the
smaller standard-rule set. Current-instance rows are the flattened authoritative
system JSON fields and are grouped by identity/connection, electrical/physical,
ratings/limits, operation/control, cost/carbon, reliability, and dynamics.
Resolved failure modes are integrated into the selected physical instance;
legacy `Reliability - ...` registry groups are not shown as standalone models.
Unknown typical or validation ranges are explicitly marked unpublished or
unregistered.

## Workspace layout

The GUI uses an engineering-workstation layout rather than keeping every tool
surface permanently open. First-time desktop sessions use compact density and
a collapsed console; first-time viewports up to 720 px also collapse the
component library and contextual ribbon. Users can independently show or hide
the component library, contextual ribbon, right inspector, and console. The
standard density always retains every original toolbar command, while compact
density exposes the primary canvas commands and keeps secondary commands
available by returning to standard density.

Focus mode hides workflow guidance, dependency status, the contextual ribbon,
the component library, and the console while preserving the current module,
inspector tab, results, authored model, and console contents. Exiting focus mode
restores the prior dock preferences. Density, dock visibility, and focus state
round trip through `hysimWorkspaceLayoutV1`; the shortcut is
`Ctrl/Cmd+Shift+F`. Layout changes trigger chart and viewport resize handling
without rebuilding the Canvas model or changing result identity.

The intended division of work is:

- WebGL: pan, zoom, hit-test, and inspect the full network.
- Local SVG: inspect/select a bounded k-hop neighborhood.
- Virtual tables: edit the complete authored model without materializing rows.
- Charts: browse downsampled time windows while retaining complete export data.
- Backend chunks: serve topology LOD, spatial windows, time frames, and worst
  violations through `/api/v1`.

Updated: 2026-08-10

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

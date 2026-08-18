# Canvas Runtime and Result Playback

For large models the main viewport is no longer an empty headless surface.
`web/js/core/network_overview.js` renders the full bus/primary-edge graph with
WebGL2 and LOD0/1/2 aggregation while creating no per-bus SVG DOM. Render state
lives in preallocated typed-array vertex stores: selection changes patch node
colors through a merged `bufferSubData` dirty range instead of rebuilding the
buffers, and LOD2 picking uses a uniform-grid spatial index over node
positions rather than a linear scan.

When enough buses carry real coordinates (≥80% non-default lat/lon) the
overview switches to viewport-driven fetching: it requests
`POST /api/session/topology_window` for the current viewport grown by a 40%
prefetch margin on each side, debounced 250 ms after pan/zoom, and skips the
request entirely while the viewport stays inside the already-fetched window at
the same LOD. LOD selection is automatic from the estimated visible-node count
(≤2,500 → LOD2, ≤30,000 → LOD1, else LOD0) unless the user pins a level in the
LOD select (the `自动` entry re-enables auto). Buses without coordinates are
excluded by the backend; its `coordinate_coverage` and `model_limitations` are
surfaced verbatim in the overview status line and in `stats()`. Systems with
insufficient coordinate coverage — or a failed first window fetch — fall back
to the historical local full-system rendering with the manual 12,000-bus LOD
heuristic.

Windowed mode also overlays the cached last power-flow result: alongside each
topology window it requests `POST /api/session/result_window` for the same
bbox (no LOD field; the response is always per-bus/per-branch), and
`Canvas.showPowerFlowResults` triggers a result-only refetch when a new PF
completes. LOD2 nodes are colored by `vm_pu` with the same bands as the SVG
result overlay (<0.95 pu red, >1.05 pu orange, in-band green/cyan), and
branches by `loading_pct` with the same green→yellow→red ramp as the heatmap
(clamped at 150%). Result colors are written into the SoA color slots and
uploaded through merged `bufferSubData` dirty ranges; a selected bus keeps its
selection color and falls back to its result color on deselection. A `409
no_cached_power_flow` response silently clears the coloring back to structural
domain colors. `result_meta.converged = false` or
`result_matches_current_system = false` is declared in the status line and in
`stats()`; aggregated LOD0/1 views keep structural colors because the result
window carries no group-level aggregates. The local fallback path never
fetches or applies result colors.

The existing
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

## User feedback and data protection

Transient notifications use a stacked toast container (`#toastContainer`,
`aria-live="polite"`, error toasts carry `role="alert"`) exposed as
`App.toast(msg, level, {sticky, actions})`. `log()` mirrors warn/error console
entries to toasts (info stays console-only); `setStatus(text, 'error')` reports
specific failure texts and skips generic one-word statuses whose detail already
went through `log('error')`. Both channels dedupe identical messages inside a
2 s window.

A persistent Problems tab (`#problemList`) shares the bottom console viewport
with the log: every error-level console entry is collected automatically, and
`App.reportProblem({level, message, compId?, bus?})` additionally attaches an
optional Canvas locate target rendered as a keyboard-reachable 定位 button that
reuses `panToComponent`/`panToBusId`. Entries carry time and level, dedupe
consecutive identical messages within 3 s, and are capped at 100. Property-panel
validation failures (JSON field parse, reliability configuration save) report
with the edited component id; single-bus short-circuit failures report with the
fault bus. Locate is disabled in headless WebGL overview mode.

Property editing validates immediately, not only at Apply: numeric inputs mark
a red border + `title` hint on `input` and report to the Problems panel on
`change` (with the component id); ranges come from the element's own `min`/`max`
attributes — populated by the backend reliability schema or the dynamic-model
catalog — and generic fields without a declared range get legality-only checks
(nothing is hardcoded). JSON textareas (`dynamic_model`, `*_profile_values`)
pre-validate syntax and shape on `blur` under the same rules Apply enforces.
Recognized units in `COMP.fieldLabels` trailing parentheses (whitelist in
`splitPropertyLabelUnit`) move from the label into an input suffix span;
unrecognized parenthesized hints (`0=HV,1=LV`, `JSON数组`, …) stay in the label
— no units are invented.

A `beforeunload` guard prompts whenever `_canvasDirty` is true; the flag clears
on successful backend sync or fresh model load, so the prompt appears only with
real unsaved work. While the canvas is dirty and the page is visible, an
auto-draft timer (30 s) stores `Canvas.buildSystemJson()` in localStorage under
`hysim.canvasDraft.v1` as `{saved_at, json}`. Drafts are skipped in headless
large-system mode and when the serialized system exceeds 4 MB (notified once
per session); on startup a sticky toast offers explicit 恢复草稿/丢弃 actions —
the canvas is never overwritten silently. Restore posts the stored string to
`/api/session/load_json_string` and then runs `Canvas.loadFromSystemJson` on
the local copy, which still carries the `_canvas` layout block the backend
strips. Loading any model clears a stored draft as stale.

The intended division of work is:

- WebGL: pan, zoom, hit-test, and inspect the full network; with coordinate
  coverage it fetches only the visible topology window from the backend.
- Local SVG: inspect/select a bounded k-hop neighborhood.
- Virtual tables: edit the complete authored model without materializing rows.
- Charts: browse downsampled time windows while retaining complete export data.
- Backend chunks: serve topology spatial windows (`/api/session/topology_window`,
  consumed by the overview) plus playback time frames and analysis-scoped
  result subsets through `/api/v1`.

Updated: 2026-08-17 (feedback/toast/problems/draft section; property-panel
immediate validation + unit suffixes; result-table sort/filter/CSV layer)

The authored model lives in `Canvas.state`, while analysis results live in SVG
result overlays. These states must remain separate.

## Editing safety net (SVG canvas)

The SVG editor keeps a session-local undo/redo stack (capacity 200 commands)
hooked at the structural mutation choke points in `web/js/canvas.js`
(`addComponent` / `removeComponent` / `addConnection` / `removeConnection`).
One command covers: component add; component delete (its attached wires are
bundled into the same composite command); wire add/delete; a completed drag
(recorded once at mouse-up, never per mousemove); rotation; paste; and
multi-select move/delete. Undo restores deleted components and wires under
their original ids, including user-edited wire waypoints. `clearAll()` and
`loadFromSystemJson()` empty the stack — a freshly loaded model cannot be
undone back into the previous one. Shortcuts: `Ctrl/Cmd+Z` undo,
`Ctrl/Cmd+Shift+Z` or `Ctrl+Y` redo. `Canvas.undo()` / `Canvas.redo()` /
`Canvas.canUndo()` / `Canvas.canRedo()` back the toolbar 撤销/重做 buttons
(`#btnUndo` / `#btnRedo`, wired by `web/js/core/help_panel.js`; disabled
state refreshes on every keyup/mouseup plus a 300 ms poll).
Property-panel parameter edits (owned by `app.js`) are deliberately NOT
undoable yet: they mutate `comp.params` in place outside the choke points.

`Ctrl+C` copies the selected components plus every wire whose both endpoints
are selected into an internal, session-local clipboard (not the OS clipboard)
with deep-copied parameters. `Ctrl+V` pastes with fresh ids at a cumulative
+20 px grid-aligned offset per consecutive paste; `Ctrl+D` duplicates (copy +
immediate paste). Paste lands as a single composite undo command and selects
the new components. Dragging any member of a multi-selection moves the whole
selection by the same delta; grid and alignment snapping apply to the pressed
component only, the rest follow the delta. All editing shortcuts are ignored
while an input/textarea/select or contentEditable element has focus. In
headless large-system mode (>400 buses, no SVG diagram) copy/paste/duplicate
no-op and the undo stack stays empty.

`web/js/core/help_panel.js` owns the keyboard-shortcut cheat sheet
(`#helpModal`): `?` (Shift+/) toggles it, Esc or a backdrop click closes it,
and the same form-field guard as the canvas handler applies (INPUT/SELECT/
TEXTAREA/contentEditable focus suppresses `?`). The content is generated from
a static data array (groups 选择 / 编辑 / 视图 / 分析) via DOM building with
`textContent`, and restates the headless large-system limitation (editing
shortcuts unavailable, undo stack stays empty). The modal follows the
`.modal` + `h3` convention, so `core/accessibility.js` automatically attaches
`role="dialog"` / `aria-modal` / `aria-labelledby`. It is exposed as
`HySimCore.HelpPanel` (`open` / `close` / `toggle` / `refreshUndoRedo` /
`shortcutGroups`, schema `hysim_help_panel_v1`).


## Static results

`Canvas.showPowerFlowResults(result)` is the static PF/OPF presentation path.
It can refresh solved component displays and the visualization overlay.
PF and OPF post-PF payloads share authored-space component keys and terminal
flow fields.

The balanced-PF advanced panel exposes smooth-NCP continuation and local-VSC
Schur admission as expert controls. Its tri-state switches and blank numeric
fields preserve sparse intent: blank means that the browser omits the field
and the backend resolves its authoritative default. After a solve, the panel
shows `options_effective.robust_nonlinear`; it never derives an effective
tolerance from a JavaScript default.

The PF result group renders `linear_structure` as a solver certificate. It
distinguishes disabled/not-admitted, accepted, and rejected-with-full-LU-
fallback states and shows dimension/structural-nonzero reduction, local-block
regularity, backward error, smooth continuation updates, and local-rate
samples. Device rows continue to own physical current-limit activity, margin,
priority, and exact NCP residual. The advanced numerical controls apply only
to balanced positive-sequence steady/quasi-steady PF: they do not claim that
the GFM priority NCP is inside the OPF KKT system and do not overwrite the
shared GFM parameters consumed by transient initialization.

Changing any NCP/Schur expert control invalidates the displayed effective
policy and solver certificate immediately. The GUI labels the old certificate
as stale and requires a new PF run; it never presents a previous generalized-
Jacobian certificate as evidence for newly edited numerical controls.

Topology and result-table navigation has two explicit identity paths. Normal
SVG systems use the internal Canvas component identifier only for immediate
glyph selection. Headless/WebGL systems recover the authored component from
`canvas_type` plus stable `.index` (or explicitly named component position as
a compatibility fallback), resolve its connected bus, and call the
domain-qualified `{domain, index}` overview selector. AC and DC buses with the
same integer index therefore remain distinct. Topology rows also retain their
search-registry source so a WebGL selection can visibly highlight the matching
virtualized row.

Result tables in the 结果 tab (power flow, OPF, short circuit, reliability, and
the day-ahead / real-time / repeated-game market views) are progressively
enhanced by `enhanceResultTablesIn(root, {module})` in `web/js/app.js` after
each module renders: header-click three-state sorting (numeric-aware,
`aria-sort` on the active `<th>`, third click restores the original row order),
a per-column contains-match filter row directly under the header (200 ms
debounce, case-insensitive), and an 导出 CSV toolbar button exporting only the
currently visible rows (RFC-4180 escaping, UTF-8 BOM so Excel reads Chinese
headers, filename `<module>_results_<timestamp>.csv`). Tables that do not fit
the expected shape — missing thead/tbody, multi-row headers, colspan/irregular
rows, embedded form controls (cost editor, market participant editor), or
virtualized tables — are skipped with a `console.debug` note and rendering is
never broken. Row-level `data-comp-id` canvas locate keeps working under
sorting because the click delegation uses `closest('[data-comp-id]')` and
sorting only reorders DOM rows. Covered by `tmp/result_table_smoke.mjs`.

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

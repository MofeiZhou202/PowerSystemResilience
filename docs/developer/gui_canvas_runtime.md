# Canvas Runtime and Result Playback

## Time-series solver threads

After this update, restart the server executable and reload the page: rebuilding
does not replace an already-running process. The `app.js`/`style.css` asset URLs
use version `20260910-uc-threads` to invalidate the previous frontend cache.

The shared production toolbar distinguishes **求解器线程数** (`tspfSolverThreads`)
from **按日并行线程数** (`tspfThreads` / `annThreads`). The solver input is
editable for Auto, Native and Gurobi regardless of the annual daily mode; switching
to HiGHS/SCIP disables it with an explicit unsupported-adapter hint, retains the
entered value and submits 0 (backend default). Switching back restores the value.
SCUC/SCED continue to disable independent annual days, not the solver input.

| Value | API / C++ source | Contract |
|---|---|---|
| Solver cap | `uc_solver_threads` → `TimeSeriesPFOptions::uc_solver_threads`; annual `ts_pf_options` | Integer 0–256; 0 retains defaults; 1–256 applied to Native B&C or Gurobi |
| Day workers | `parallel_threads` → existing day-pool options | Independent of the solver cap; annual decomposition requires DynamicOPF and its existing state-admissibility checks |
| Configured cap | `uc_solver_threads_configured` | Positive configured limit, or null for backend default/unknown/no UC; never measured active threads |
| Requested cap | `uc_solver_threads_requested` | Request echo, independent of actual backend/fallback |
| Actual UC backend | `uc_solver_name` | Actual coupled-UC backend; distinct from annual-planning solver |

Both `/api/session/run_ts_pf` and `/api/session/run_annual_sim` validate the cap
before solving: fractional, negative, null, string and >256 values return HTTP
400. Nonzero overrides for current HiGHS/SCIP adapters also return 400. Annual
day drill-down forwards the shared solver cap too. The result panel distinguishes
day-task execution from the UC solver's configured limit. DynamicOPF with no UC
reports that UC was not run; its OPF threads are not controlled by this field.

With a nonzero cap, Auto/Gurobi try Gurobi then Native, skipping the current
HiGHS adapter because it cannot honor that override. With 0, the existing
Gurobi→HiGHS→Native fallback order remains. Time-series independent-day Auto
uses Native for an explicit cap; each day retains the requested solver cap.
Default independent-day Native keeps its existing one-thread policy. Thus day
workers × solver workers may exceed the hardware count; these are separate
user-controlled concurrency limits, not an automatic speedup guarantee.

The Gurobi thread-only setter lives in the sibling MIPSolvers adapter and preserves
its existing time limits and tolerances. Regression targets: `test_uc_solver_threads`
and `annual_ui_reconcile_e2e`. Numerical evidence and dependency revision are
recorded in `docs/overview/development_status.md`.

## Bounded local busbar sheets

Large systems continue to enter the WebGL overview under the existing size
policy; they are never expanded into thousands of busbar SVG elements by local
navigation. Select an overview bus and choose **局部母线图**, or enter a bus
in the global search and choose **邻域图**. The local sheet defaults to 20 buses,
offers 40/80-bus limits and 1–4 hops, and preserves readable 13px identity labels
inside its own scroll region. Mobile stacks the connection inspector below the
sheet rather than scaling the whole drawing down.

`web/js/core/local_bus_diagram.js` builds a read-only structural graph and
extracts a bounded neighborhood. Each bus occupies a separate 160px row;
rendered connections receive distinct taps and orthogonal lanes outside the bus
column. At most 160 links and 12 taps per bus are drawn. Both off-sheet and
undrawn internal links remain in the selected bus's connection inspector,
paged at 20 rows. Counts explicitly disclose the rendering limits. Selecting a
bus highlights its incident paths and synchronizes its stable selection with
the main Canvas/WebGL/topology table. Clicking a connection or double-clicking
a bus recenters the sheet; **返回上一母线** restores the previous center.
Closing/Escape returns to the network; editor keyboard commands do not pass
through the modal. A full model replacement clears the sheet and its history.

| Contract | Authoritative source | GUI behavior |
|---|---|---|
| Bus identity | `ac.buses[].index` / `dc.buses[].index` | `{domain,index}`; duplicate IDs in one domain rejected; AC/DC equal IDs remain distinct |
| Nominal voltage | `buses[].base_kv`, kV | Missing/non-positive values show “电压未提供”; no inferred 110/320 kV |
| Ordinary branches | `ac/dc.branches[]`, `from_bus`, `to_bus`, stable `index` | Parallel records retain separate identities and connection rows |
| Switching equipment | `ac.switches`, `ac.circuit_breakers`, `dc.dc_circuit_breakers`; `bus_from`, `bus_to`, `closed`, `in_service` | Open/offline links remain visible, dashed and labeled; not an energization result |
| Transformers | `ac.transformers_2w`, `source_branch_idx`, `hv_bus/lv_bus` | Branch-backed metadata becomes an alias of the branch, not a duplicate link |
| Converter links | VSC/LCC `bus_ac/bus_dc`; DC/DC `bus_in/bus_out` | Domain-qualified cross-domain endpoints; current nested and legacy top-level DC/DC collections recognized |
| Multi-terminal equipment | 3W `hv_bus/mv_bus/lv_bus`; router `ports[].port_type/bus` | Port association spokes explicitly declared; no internal-conduction or electrical-equivalent claim |
| Attached devices | Nested AC/DC component records with `bus` | Counts in inspector; no individual local device glyphs or top-level aggregate-device coverage claim |
| Scope/boundaries | Full authored graph and extracted membership | No electrical merging, deletion, backend sync or result fabrication |

Local selection/navigation leaves the authored JSON, including `_canvas.viewBox`,
unchanged; `selectStableRef` forwards `preserveViewport: true` for local-sheet
selection, synchronizing the selected component without panning the main Canvas
or switching the underlying workspace tab. Ordinary search still pans to its
target. The local sheet has no separate save format. Unknown link IDs/references are disclosed as
limitations, duplicate stable links are rejected, and a nonexistent focus bus
is an error. Crossings are possible: only bus tap dots indicate connections.
This first delivery is a structural inspection sheet, not an editable
substation schematic, automatic zoom morphing, or a full-network detailed sheet.

The main editor now reserves the configured **maximum** busbar footprint before
ELK layout and during legacy placement/collision checks. Its tap projection is
rotation-aware, and live re-span keeps bar endpoints, labels and connect handles
aligned. This reservation applies at automatic-layout time: user-authored
overlaps/locked positions are preserved, and increasing the global busbar cap
after layout may require another automatic layout. It is not a universal
wire-crossing or arbitrary manual-layout guarantee.

The mathematical bounds and predeclared performance ceiling are in
[the scale-first rationale](../planning/gui_one_line_redesign.md#scale-first-implementation-rationale).
Registered tests: `local_bus_diagram_test`, `local_bus_diagram_e2e`; existing
`network_overview_e2e`, `gui_scale_features_e2e` and the busbar-length script
cover adjacent behavior. The browser regression includes MATPOWER `case118.m`
(118 AC buses) and built-in `ieee118_acdc` (118 AC + 6 DC buses), both main layout
engines, 20/40/80 local limits, stable selection, full-JSON preservation, Back,
and 390/320px mobile widths. Full-network fit can make main-editor labels too
small to read; readable detail is provided by the local sheet. Current numerical
evidence is in the development status.

Southern market modules use a separate market topology inside the main viewport,
implemented by `web/js/core/market_canvas.js`. They preserve the engineering
Canvas model and use typed Southern stable IDs, synchronized scenario/day/slot
results and bounded large-case neighborhoods. See the
[market Canvas contract](../modules/market/southern_execution_contract.md#市场-canvas-身份与时段契约)
for validity, clicking, playback and regression evidence. The engineering
WebGL overview described below is a different view and identity space.

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
bbox at the same LOD, and `Canvas.showPowerFlowResults` triggers a
result-only refetch when a new PF completes. LOD2 nodes are colored by
`vm_pu` with the same bands as the SVG result overlay (<0.95 pu red, >1.05 pu
orange, in-band green/cyan), and branches by `loading_pct` with the same
green→yellow→red ramp as the heatmap (clamped at 150%). At LOD0/1 the backend
returns group-level aggregates: a group node is colored by the worst band
deviation among its members (`vm_min`/`vm_max` outside [0.95, 1.05]) and
otherwise by `vm_avg`, while an aggregate edge takes the maximum `loading_pct`
of its collapsed branches — both semantics are declared by the backend in
`units`/`model_limitations` and surface in the status line and `stats()`.
Result colors are written into the SoA color slots and uploaded through
merged `bufferSubData` dirty ranges; a selected bus keeps its selection color
and falls back to its result color on deselection. A `409
no_cached_power_flow` response silently clears the coloring back to structural
domain colors. `result_meta.converged = false` or
`result_matches_current_system = false` is declared in the status line and in
`stats()`. While result coloring is active the overview shows a small legend
overlay (created from JS with inline styles, `aria-label="结果着色图例"`):
the three voltage bands plus the loading ramp with numeric annotations; it
hides when no result is active. The voltage bands and loading ramp are
defined once as shared constants in `network_overview.js` and consumed by
both the WebGL coloring and the legend. The local fallback path never fetches
or applies result colors.

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

Updated: 2026-08-18 (result_window LOD0/1 聚合结果色 + 结果着色图例；色阶抽为共享常量)
此前：2026-08-17 (feedback/toast/problems/draft section; property-panel
immediate validation + unit suffixes; result-table sort/filter/CSV layer)

## Onboarding tour, help menu, documentation center, and example templates

First-time sessions (no `hysim.tourDone.v1` in localStorage) auto-start a
seven-step onboarding tour after a short settle delay: 元件库 → 画布拖放与连线
→ 属性面板 → 运行潮流 → 结果浏览与定位 → 全局定位 → 帮助入口. The tour is pure
DOM (spotlight ring with an oversized dimming `box-shadow` plus a clamped
tooltip bubble, styles in `style.css` under "Onboarding Tour"); captions are
assigned via `textContent` only. Steps whose target selector resolves to a
missing or layout-less (collapsed/hidden) element are filtered out before
starting and re-checked at each step, so an inactive module's run button or a
collapsed panel simply drops its step. Esc exits at any time, ArrowRight/
ArrowLeft step, and 上一步/下一步/跳过/完成 buttons are keyboard reachable
(primary action receives focus per step); completing, skipping, or Esc all
write `hysim.tourDone.v1=1` so the tour runs once per browser. Controls are
exposed as `App.tour` for smoke tests (`tmp/tour_smoke.mjs`).

A 帮助 button in the top toolbar aux cluster opens a dropdown menu: 文档中心
(calls `HySimCore.HelpCenter.open()` behind a typeof guard; also bound to F1,
which suppresses the browser default and is ignored while an editable control
has focus), 新手引导 (replays the tour regardless of the done flag), 快捷键面板
(calls `HySimCore.HelpPanel.open()` behind a typeof guard), and 示例模板
(opens the case-load modal). The menu closes on outside click or Esc and
returns focus to the button.

The documentation center (`web/js/core/help_center.js`, schema
`hysim_help_center_v1`) is a `.modal` with a sectioned navigation tree, a
debounced title/tag search box, and a markdown content pane. Entries come
from the manifest `web/help_docs.json` (schema `hysim_help_docs_v1`), grouped
into 用户指南 / 示例教程 / 模块手册 / API 契约 / 理论模型 / 测试验证 sections;
each entry may carry a `modules` list of GUI module ids (`.module-btn`
`data-module` values), and matching entries are pinned to a 当前模块相关 block
when that module tab is active. Documents are fetched read-only from the
server mount `/xjtu/docs/<path>` (see [运行时 API](../reference/runtime_api.md)),
rendered with the vendored marked (`web/vendor/marked.min.js`; raw HTML tokens
escaped, with a minimal built-in renderer as fallback), and relative `.md`
links are rewritten to in-center navigation while external/non-markdown links
open in a new tab and relative image sources are repointed under
`/xjtu/docs/`. Below 720px the navigation collapses into a drawer toggled by
the 目录 button. Esc closes; ArrowUp/ArrowDown move within the visible
navigation; the shared `.modal` focus trap applies. Manifest or document
fetch failures render an explicit error panel (never a blank pane), and failed
document fetches are not cached so reopening retries.

Updated: 2026-08-18 (documentation help center; onboarding tour, help menu,
example templates)

The case-load modal additionally offers shipped JSON example templates from
`web/examples/` (`EXAMPLE_TEMPLATES` manifest in `app.js`; the C++ server
mounts `web/` at `/xjtu/`, so templates load via relative `fetch` and then
follow the exact `importJson` path through `/api/session/load_json_string`,
factoring the shared logic into `importSystemJson`). Current templates:
`ac_radial_feeder_example.json` (5-bus 10 kV radial AC feeder, field sets
copied from `data/simple_case.json` + `data/dsp/cigre.json`) and
`hybrid_acdc_microgrid_example.json` (two-bus DC side with a DC branch and two
PV arrays behind a VSC, derived from `data/simple_case.json`).
`ev_traffic_scenario_template.json` is a separate EV-traffic scenario schema,
not a network model, and is not listed in the case-load modal.

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

With the SVG canvas itself focused (`tabindex="0"`, set by both `canvas.js`
init and `core/accessibility.js`; focus ring from the global
`[tabindex]:focus-visible` rule), the arrow keys nudge every selected
component by one grid step (20 px; 1 px with Shift for fine positioning). The
branch only engages when the keydown target IS the canvas element, so arrow
navigation in panels, tablists, and dialogs is never hijacked, and it is a
no-op outside select mode and in headless mode. A burst of presses — key
auto-repeat or quick taps — collapses into ONE composite `move` undo command,
mirroring the drag mouse-up recording: start positions are captured on the
first keydown, the burst extends while keydowns arrive within 500 ms, and it
commits on the first arrow keyup, on the 500 ms pause timer, or eagerly when
any other edit begins (mouse down, any non-arrow key such as Ctrl+Z or Delete,
`clearUndoStacks`). Live nudges re-route wires with the cheap orthogonal
router and skip alignment guides; the commit runs the full avoid-aware
re-route and visualization refresh once.

A zoom-percentage indicator (`#zoomIndicator`, e.g. "125%") sits in the
canvas top-right corner. It is created lazily by `canvas.js` with inline
styles (no stylesheet dependency), `aria-live="off"`, and refreshed from
`updateViewBox()` — the single pan/zoom choke point — so wheel, toolbar
buttons, fit-all, and minimap pans stay in sync. It is never created in
headless large-system mode and is removed if a session switches into it.

`web/js/core/help_panel.js` owns the keyboard-shortcut cheat sheet
(`#helpModal`): `?` (Shift+/) toggles it, Esc or a backdrop click closes it,
and the same form-field guard as the canvas handler applies (INPUT/SELECT/
TEXTAREA/contentEditable focus suppresses `?`). The content is generated from
a static data array (groups 选择 / 编辑 / 视图 / 分析) via DOM building with
`textContent`, and restates the headless large-system limitation (editing
shortcuts unavailable, undo stack stays empty). The modal follows the
`.modal` + `h3` convention, so `core/accessibility.js` automatically attaches
`role="dialog"` / `aria-modal` / `aria-labelledby`. The same module also runs
a focus trap for every visible `.modal`: while one is open, Tab / Shift+Tab
cycle through its focusable elements and cannot leave the dialog (Esc stays
with each modal's own logic); on close, focus returns to the pre-open element
only when it is still stranded inside the closing modal or on `<body>`, so
modals like the help panel that restore focus themselves are not overridden.
Attach/detach is driven by the existing MutationObserver → scheduled sync
pipeline (per-modal attribute observers plus a `body` childList observer for
late-added modals), and the pre-open focus target is kept in a WeakMap. The
trap is exposed as `HySimCore.Accessibility.syncFocusTraps`. The canvas
`aria-label` (set in `core/accessibility.js` init) announces the arrow-key
nudge. The help panel itself is exposed as
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

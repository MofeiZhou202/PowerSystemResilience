# One-Line Diagram Editor Redesign (Busbar-Centric Authoring Canvas)

> 规划文档 / Planning proposal. This describes a **controlled improvement**, not
> current behavior. Status, acceptance criteria and risks are stated below.
> Owning contract: [GUI ↔ backend contract](../../.github/skills/develop-gui-backend-contract/SKILL.md)
> and [Canvas Runtime](../developer/gui_canvas_runtime.md).

**摘要（中文）.** 当前 `web/` 授权画布把交流/直流母线画成固定 80px 的短线段、每条母线只有
4 个端口，连线用逐条 A\* 在 20px 栅格上避障，规模一大就静默退回正交直连、母线端口重叠、
连线交叉，可读性远低于 ETAP 等工业软件。本提案在**保留 C++ 求解引擎、REST 契约与 WebGL 大网
总览**的前提下，把授权画布重构为**以母线排（busbar）为中心**的编辑器：可伸缩母线 + 沿母线分布
的抽头（tap）+ 基于 ELK 的正交母线通道布线 + IEC 60617 符号库 + 内嵌 SQLite 模型库。分阶段交付，
每阶段有验收门槛。

- Status: **Proposed** (2026-09-08). Prototype delivered at `web/prototype/one_line_redesign.html`.
- Scope: front-end authoring canvas + model store only. **No solver/engine change.**
- Non-negotiable invariants preserved: domain-qualified `{ac_bus, dc_bus}` identity,
  stable component `.index` reporting, honesty flags (`model_scope` / `ValidityFlags`).

### Decisions (2026-09-08, confirmed)

- **Stack**: modernize the existing web app (Konva/WebGL rendering + elkjs),
  not a JavaFX rewrite. Reuse the C++ engine and 86 REST endpoints.
- **Store**: embedded SQLite (sql.js in-browser; server file authoritative).
- **Symbols**: IEC 60617.
- **Integration**: **refactor `web/js/canvas.js` in place** behind a runtime
  flag (not a separate parallel module). See §8 R3 for the safety approach.
- **P0 shipped**: model store + lossless adapter
  (`web/js/core/one_line_store.js`, doc `../developer/gui_one_line_model_store.md`,
  test `tests/e2e/one_line_store_roundtrip.mjs`).

---

## 1. Problem statement (grounded in current code)

The authoring surface is `web/js/canvas.js` (SVG/DOM, ~8.6k lines) driven by
`web/js/app.js` (~25k lines) with the symbol/port library in
`web/js/components.js`. The WebGL2 large-network overview
(`web/js/core/network_overview.js`) and the ELK auto-arrange
(`web/js/core/layout_engine.js`, `layout_graph.js`) are already solid; the
**SVG authoring canvas is the weak link**. Concrete defects:

1. **Buses are fixed 80-px line symbols, not busbars.**
   `components.js` draws `ac_bus` as a solid 80-px line and `dc_bus` as a dashed
   80-px line. The port table gives each bus exactly four ports
   (`left(-40,0)`, `right(40,0)`, `top(0,-6)`, `bottom(0,6)`); top/bottom are the
   same point. A bus can therefore host ~4 devices cleanly; real feeders host
   10–40. Excess devices collapse onto identical coordinates → the AC/DC
   connection tangle users report.

2. **No busbar / node-breaker model.** There is no resizable bar and no
   bus-section / coupler / breaker structure. ETAP/PowerFactory/CYME treat a
   busbar as a first-class element with many taps and an explicit node-breaker
   topology; that is what makes their one-lines legible.

3. **Routing is per-connection and does not scale.** `routeAvoid`
   (`canvas.js` ~L1500) is a per-wire 4-connected A\* on a 20-px grid. It is
   hard-disabled above **800 connections / 250k cells / 8000 expansions** and
   silently falls back to greedy orthogonal (crossings). There is **no global
   bus-lane routing**; multi-edges get only a ±9-px nudge.

4. **Labels collide.** Labels are absolute `<text>` at fixed offsets; the 3-pass
   `deOverlapLabels()` nudge is a symptom of missing layout.

5. **The editor gives up on large models.** Above 400 buses / 2500 elements it
   switches to a static summary overlay instead of drawing — opposite to ETAP's
   fluid hierarchical one-lines.

## 2. Goals / non-goals

**Goals**
- G1. A busbar primitive that stretches and hosts N devices along its length.
- G2. A node-breaker-aware model (bus sections, couplers, breakers,
  disconnectors) that still projects to the existing canonical model.
- G3. Global orthogonal routing with bus lanes, junction dots, and automatic
  label placement — legible at hundreds of authored elements.
- G4. IEC 60617 symbol library with engineering conventions (color by kV /
  domain / energization; de-energized greying; grid, snap, rulers).
- G5. An embedded SQLite model store as the single source of truth for authored
  models, symbol library, and cached results.
- G6. Hierarchical / nested one-lines (drill into composite networks).

**Non-goals**
- Replacing the C++ engine, the 86 REST endpoints, or the WebGL overview.
- Changing solver semantics, validation tiers, or result attribution.
- A desktop rewrite (JavaFX etc.) — decision is to **modernize the web app**.

## 3. Target architecture

```text
                 ┌─────────────────────────── Browser (SPA, /xjtu/) ──────────────────────────┐
                 │  Authoring canvas (NEW)          Large-net overview (KEEP)   Inspector/panels │
                 │  Konva scene graph  ───────────  network_overview.js (WebGL2)  properties     │
                 │      │  busbar+tap model                                        topology tables│
                 │      │  ELK bus-lane routing (elk.bundled.js, KEEP)                            │
                 │      ▼                                                                          │
                 │  Model store: SQLite (sql.js WASM in-browser)  ◄──sync──►  server SQLite       │
                 └───────────────────────────────┬────────────────────────────────────────────────┘
                                                 │  existing /api/session/* REST (KEEP, 86 routes)
                                                 ▼
                            run_gui_server (C++)  ──►  hacdcpf engine (PF/OPF/…)
```

Key moves:
- **Rendering**: replace the per-element SVG DOM authoring layer with a **Konva**
  (Canvas/WebGL) scene graph. Konva gives layered rendering, hit-testing, and
  transforms without one DOM node per glyph, which is what caps the current SVG
  editor. The WebGL overview and SVG result overlays stay.
- **Layout/routing**: reuse the already-vendored **elkjs**
  (`web/vendor/elk.bundled.js`) with `elk.algorithm=layered`,
  `elk.edgeRouting=ORTHOGONAL`, and **port constraints on busbar taps** so wires
  enter buses on lanes. Device stubs (device→own bus) are deterministic
  (vertical drop to nearest free tap); inter-bus links (transformer, converter,
  feeder) are ELK-routed. This is the prototype's model.
- **Model store**: an embedded **SQLite** database. In-browser via **sql.js**
  (WASM) for zero-setup single-user editing; the C++ `run_gui_server` owns the
  authoritative on-disk SQLite and exposes load/save so the same file round-trips
  headless and in E2E. (H2 was considered but is Java-only; SQLite fits the
  C++/JS split.)

## 4. Data model (busbar + node-breaker)

The authored model keeps the repo's three iron rules (domain-qualified AC/DC
maps, stable `.index`, honest scope). New authoring tables (SQLite):

| table | key columns | notes |
|---|---|---|
| `bus` | `id, domain('ac'/'dc'), index, name, base_kv, x, y, length, orient` | busbar geometry; `index` is the stable exported id |
| `bus_section` | `id, bus_id, from_frac, to_frac` | node-breaker sectionalizing along the bar |
| `tap` | `id, bus_id, frac(0..1), side('+'/'-')` | attachment point along the bar; auto-reflows |
| `device` | `id, kind, index, name, params_json` | 1-port devices (gen/load/PV/BESS/…) |
| `device_terminal` | `device_id, tap_id` | docks a device to a tap |
| `link` | `id, kind('transformer'/'converter'/'line'/'coupler'), from_tap, to_tap, params_json` | inter-bus 2-port elements |
| `sheet` | `id, parent_id, name` | hierarchical one-line pages |

Projection: a thin adapter turns these rows back into the current authored
`HybridPowerSystem` JSON (ac.buses/dc.buses/branches/converters/…) so **no
backend change is required**. Busbars with multiple sections expand to the
existing bus-merge/zero-impedance handling already in `graph/`.

## 5. Symbol library (IEC 60617)

- One vector symbol per family (generator, load, 2W/3W transformer, VSC/LCC
  converter, grid infeed, PV, battery, shunt, breaker, disconnector, motor…),
  authored to **IEC 60617** with a shared 80-px module grid and named terminals.
- Symbols are data (SVG path strings or Konva shape factories) so the same
  library feeds the editor, the minimap, and print/report export.
- Conventions: solid bar = AC busbar; double/annotated bar = DC busbar; color
  ramp by voltage level; de-energized elements greyed; breaker open/closed state
  drawn explicitly (node-breaker).

## 6. Interop / migration

- Import the current authored model JSON → new tables (loss-free; the four-port
  bus becomes a short busbar with taps derived from existing connections).
- Preserve existing **ETAP** import/export (`load_etap_xlsx/xml`,
  `export_etap*`) and all other IO; the busbar model maps naturally to ETAP's
  busbar concept.
- The WebGL overview, topology tables, and result overlays keep the single
  `{domain, index}` selection reference.

## 7. Phased roadmap & acceptance criteria

Each phase must land with tests (Node/Playwright E2E for GUI, round-trip for the
model adapter) and a docs update, per repo rules.

- **P0 — Model & store.** SQLite schema + sql.js load/save + JSON↔tables adapter.
  *Accept:* import every built-in case and round-trip to identical authored JSON
  (byte-stable after canonical sort); server SQLite load/save E2E green.
- **P1 — Busbar editor core.** Konva scene, resizable busbar, tap docking, IEC
  symbols, grid/snap, property panel.
  *Accept:* a 10 kV feeder with 20 devices renders with zero overlapping stubs;
  dragging the bar re-flows taps; select/edit round-trips to the store.

  **P1 status (2026-09-08).** Slices 1–2 have **landed in the existing SVG
  `web/js/canvas.js`** behind the runtime flag `state.busbarMode` (default
  **off**; persisted as `localStorage.busbarMode`; toggled from the toolbar
  **母线模式** button (`#btnBusbarMode`, wired in `app.js`) or
  `Canvas.setBusbarMode(true)`).
  - *Slice 1 — busbar + tap model:* AC/DC buses render as a resizable bar that
    **auto-spans their connected devices**, and each wire's bus endpoint
    **slides to the device's x** (`projectOntoBar`) so feeders drop vertically
    with no 4-port pileup.
  - *Slice 2 — IEC 60617 symbols:* `web/js/core/iec_symbols.js` provides
    IEC-styled device glyphs (generator, load, transformer, converter, battery,
    PV, motor, shunt, breaker, …) with terminals matched to `COMP.ports`; used
    only in busbar mode.
  - *Slice 3 — bus-lane routing:* enabling busbar mode defaults the auto-layout
    to the existing `BUSBAR` arrangement (horizontal bus lanes + vertical
    feeders) and switches connections to `orthogonal`, so the 自动布局 button
    yields a clean substation one-line (validated: 2-bus case, **0 crossings /
    0 overlaps**).
  - *Polish:* the 4 legacy port dots are hidden on busbars (kept at opacity 0
    for connect-start); bus name/kV render as an ETAP-style tag anchored at the
    left end of the bar. Because the dots are hidden and slide as taps, a wire is
    drawn to/from a bus by clicking **anywhere along the bar** in connect mode
    (the click is projected onto the bar); in **select mode** a bus also exposes
    hover-revealed **connect handles** at the bar ends + centre so a wire can be
    started without switching tools (the bar body still drags). The handles are
    counter-scaled to a constant on-screen size (world-space radius goes
    sub-pixel when zoomed out over wide bars) and are found via an
    `elementsFromPoint` hit-stack scan (they can sit under a wider transparent
    `comp-outline`). Fixed a bug where buses were unconnectable — covered by
    `tests/e2e/busbar_connect_e2e.mjs` (**7/7**, both connect- and select-mode).
  - *Slice 4 — IEC palette + resizable library (2026-09-10):* the component
    library (元件库) now renders each entry with its IEC 60617 glyph
    (`window.IEC_SYMBOLS[type] || COMP.symbols[type]`) so the palette matches the
    canvas; ~18 device types have IEC symbols, the rest fall back to legacy. The
    library panel is now width-adjustable via a drag resizer (`#libraryResizer`,
    mirroring the right panel; drives `--lib-w` **and** an inline width to beat
    the compact-density `#componentLib { width:156px }` override), persisted as
    `localStorage.hysim.libW`.

  Default-off preserves the legacy 80 px, 4-port bus and glyphs exactly
  (validated: bar half-length 40 legacy vs 120 spanning a device 100 px away;
  no console errors; frontend status normal), so the existing GUI E2E is
  unaffected. Shipping a `canvas.js`/`app.js` change requires bumping their
  `?v=` cache-busters in `index.html` (each file has its own; done: `busbar-p1c`).
  Per the in-place decision this improves the SVG renderer first; the
  Konva/WebGL migration is deferred. **P1 is functionally complete behind the
  flag, and validated end-to-end against a running `run_gui_server`:**
  `tests/e2e/busbar_mode_e2e.mjs` (Playwright) loads `ieee14_acdc`, enables
  busbar mode, runs the BUSBAR auto-layout (0 crossings / 0 overlaps) and a real
  power flow (converged), and asserts the busbar rendering, energization
  greying, the exact legacy revert when off, and no JS errors — **12/12 checks
  pass**; the canonical `tools/gui_api_e2e.py` backend smoke is **82/82**.
  Busbar mode is now the **default** one-line look (`localStorage.busbarMode='0'`
  forces legacy). The metric/topology E2E baselines are unaffected because busbar
  mode changes rendering only — verified: `layout_baseline_e2e.mjs` (ieee33:
  0 overlaps) and `canvas_3w_e2e.mjs` both pass with default-on — so only the
  human-review screenshot artifacts differ. The in-browser SQLite **store**
  persistence is now wired into the app (Save/Open `.sqlite`, see the store
  status below); the remaining P3 item is the *engine* overlay parity on the
  new busbar canvas.
- **P2 — Global routing/layout.** ELK bus-lane routing + junction dots +
  label placement; deterministic device stubs.
  *Accept:* on a 150-bus authored model, 0 wire-wire overlaps outside junctions,
  ≤2 bends median per inter-bus link, layout ≤2 s.
- **P3 — Engine integration.** Wire the new canvas to the 86 REST endpoints;
  flow/loading/voltage overlays on busbars and links.
  *Accept:* PF/OPF result overlay parity with the current SVG overlay on 5 cases.

  **Store persistence status (2026-09-10).** The P0 one-line store is now wired
  into the live app: the toolbar **保存 DB** / **打开 DB** buttons save and open
  the authored model as a portable single-file **`.sqlite`** database. Save does
  `Canvas.buildSystemJson()` → `OneLineStore.importSystemJson` →
  `saveToDb` (vendored **sql.js** WASM SQLite) → `db.export()` → download; Open
  reads the file with `new SQL.Database(bytes)` → `readFromDb` →
  `exportSystemJson` → the shared `importSystemJson` load path (backend +
  canvas). sql.js is vendored (`web/vendor/sql-wasm.{js,wasm}`); the `.wasm`
  loads lazily on first Save/Open via `locateFile`. Covered by
  `tests/e2e/one_line_store_app_e2e.mjs` — **11/11**: real `SQLite format 3`
  bytes, byte-stable round-trip through a real `.sqlite`, the Save button
  downloads `*.sqlite`, and the Open button reloads the model losslessly
  (55/55 components on `ieee14_acdc`) with zero page errors.

  **Overlay parity status (2026-09-10).** PF/OPF result overlays now render on
  the busbar canvas (ETAP convention): a converged power flow **recolors each
  busbar by voltage** (red `<0.95`, amber `>1.05`, else green; DC in cyan) via an
  overlay line held in the auto-cleared results layer, with a centered kV/pu
  readout above the bar; `showPowerFlowResults` draws it and it is included in the
  exported SVG. The flow-arrow + loading heatmap visualization
  (`setVisualizationMode('flow'|'heatmap'|'both')`) draws on the busbar-connected
  links, and additionally **tints each branch/line wire by loading** (green→amber
  →red; overloaded lines thicken) so an overloaded cable reads red directly on
  the one-line — not just via the glow/arrow — for AC and DC branches alike; the
  tint resets on the next apply / when visualization is off. The voltage bars are
  preserved across visualization toggles (the viz overlay only clears
  `.viz-overlay` elements). Covered by `tests/e2e/busbar_mode_e2e.mjs` —
  **17/17**: on `ieee14_acdc` all 16 busbars recolor by voltage and span their
  width, 63 flow labels + 42 loading-tinted wires render on the links, and the
  bars survive a `both` → `off` toggle.
- **P4 — ETAP-grade polish.** Hierarchical sheets, energization coloring,
  large-model performance, print/report.
  *Accept:* author + navigate a 1000-bus hierarchical model at ≥30 fps pan/zoom.

  **P4 status (2026-09-09).** *Energization coloring* has landed (busbar mode):
  `computeDeenergized()` runs a topology BFS from in-service sources through
  conducting, in-service elements; out-of-service or islanded buses/branches/
  devices get a `deenergized` class and render greyed (ETAP convention), and
  wires to de-energized buses grey too. Controlled by `state.showEnergization`
  (default on in busbar mode) / `Canvas.setEnergizationColoring(bool)`; cached and
  invalidated with the comp-bus map. Validated in `busbar_mode_e2e.mjs`
  (out-of-service bus greys, restoring un-greys) and live on `case33bw_acdc`
  (opening a branch greys its islanded section).

  *Hierarchical sheets (drill-down):* double-clicking a composite node
  (`microgrid` / `vpp` / `energy_router`) opens its sub-network on a sheet
  (`Canvas.enterSheet`), with a breadcrumb bar (`根 Root › …`) for navigating back
  (`Canvas.exitToRoot` / clicking a crumb). The sub-sheet is a visual authoring
  layer held on the host (`host._sheet`); **analysis always runs on the ROOT
  network** — `buildSystemJson()` swaps in the root context so a solve from inside
  a sub-sheet still sends the root model. Covered by
  `tests/e2e/hierarchical_sheets_e2e.mjs` — **9/9**, including that root-safety
  check and a save/load round-trip. Sub-sheets **persist across save/load**
  (serialized recursively in the `_canvas` layout under each composite). Known
  v1 limits: sub-sheets are not flattened into the solver model, and result
  overlays apply to the root view.

  *Print / export & large-model:* `Canvas.exportOneLineSvg()` /
  `downloadOneLineSvg()` (toolbar **导出 SVG**) serialize the drawn one-line
  (busbar rendering + IEC symbols + result overlay) to a self-contained,
  printable vector SVG. Validated in `busbar_mode_e2e.mjs` (**13/13** — the
  export produces valid SVG). Large-model check: `ieee118_acdc` (118 buses / 488
  components) renders in busbar mode without the headless fallback — load ≈0.83 s,
  BUSBAR layout ≈0.61 s, SVG export ≈6 ms, no console errors.

  *Large-scale performance hardening (2026-09-10).* Four incremental
  optimizations on top of the existing 3-tier strategy (full SVG < 350 comps →
  SVG + viewport culling 350–2500 → headless WebGL overview > 2500 / 400 buses):
  - **Overlay culling** — the PF/OPF voltage-bar overlay (tagged `data-comp-id`)
    is now culled with the same visibility as its bus in `updateViewportCulling`,
    so overlay **paint** stays O(visible), not O(model). (Loading-tinted wires
    ride on the connection elements, already culled.)
  - **Windowed overlay creation** — on a culled diagram the voltage overlay is
    only *built* for buses within the viewport + one screen of margin
    (`_overlayRegion`); panning past that region triggers a debounced rebuild, so
    **creation** is O(visible) too. Small/uncalled diagrams are unchanged.
  - **Adaptive thresholds** — the cull/headless thresholds scale with device
    capability (`hardwareConcurrency`, DPR) plus a downward-only frame-time guard
    that tightens the cull threshold if sustained jank is measured. The
    headless/WebGL switch scales **down-only** (`min(1, factor)`) so a strong
    device never *delays* the GPU overview past the tuned default.
  - **Off-thread layout** — `HySimCore.LayoutEngine` runs ELK in a **Web Worker**
    (vendored `web/vendor/elk-worker.min.js`, elkjs 0.9.3) so large-graph layout
    never blocks the UI thread, with a latched main-thread fallback if the worker
    fails or times out (`layout()` reports which path ran via `result.worker`).

  Validated: `busbar_mode_e2e.mjs` **17/17**, `gui_scale_features_e2e.mjs`
  **ALL CHECKS PASSED** (incl. the 919-element `test.xml` → WebGL-overview
  switch), full GUI E2E suite green.

  *Backend robustness (found via the busbar E2E work):* the pre-existing
  `gui_scale_features` failure was a `run_gui_server` **worker-thread stack
  overflow**, not a frontend issue. httplib's default `ThreadPool` uses
  `std::thread` (~512 KiB stack on macOS), which several deep/large-stack
  handlers overflow (first the dynamics `build_catalog()`, then a
  reliability/topology builder). Fixed generally with a drop-in
  `BigStackThreadPool` (`tests/run_gui_server.cpp`) that gives every worker a
  16 MiB stack, plus a main-thread warm-up of `dynamic_model_catalog()`.
  Verified after rebuilding both the ASAN and release servers: `gui_scale_features`
  no longer crashes (no "Failed to fetch"; server survives multiple full runs;
  0 sanitizer errors). A second, unrelated failure surfaced once the crash was
  gone: the test's own offline **Plotly CDN stub was incomplete** (defined
  `newPlot`/`react`/`Plots.resize` but omitted `purge`/`downloadImage`), so an
  empty-data chart branch threw `Plotly.purge is not a function` as an uncaught
  `pageerror`. Completing the stub makes the whole suite green (`ALL CHECKS
  PASSED`, 3/3 runs).

## 8. Risks & mitigations

- **R1 elkjs worker under file://.** ELK’s bundled worker may not start from
  `file://`. *Mitigation:* serve `/xjtu/` over HTTP (already the deployment
  mode); prototype degrades gracefully to a deterministic router when ELK is
  absent.
- **R2 Dual store drift (browser sql.js vs server SQLite).** *Mitigation:*
  server file is authoritative; browser store is a working copy flushed via the
  existing session save endpoint; single `{domain,index}` identity.
- **R3 In-place refactor of the 8.6k-line canvas.js.** Per the integration
  decision the new editor replaces the SVG authoring path inside `canvas.js`
  rather than living in a separate module. *Mitigation:* gate the new renderer
  behind a runtime flag (`state.renderer = 'svg' | 'busbar'`), replace functions
  incrementally (symbols → busbar model → routing), keep the old SVG path callable
  until P3 reaches result-overlay parity, and cover each step with the Node/
  Playwright GUI E2E. The P0 adapter (`one_line_store.js`) is already a
  standalone module feeding both paths.
- **R4 Honesty regressions.** *Mitigation:* the JSON adapter reuses existing
  projection; result overlays keep `model_scope`/`ValidityFlags` verbatim.

## 9. Prototype

`web/prototype/one_line_redesign.html` — self-contained (Canvas 2D + vendored
elkjs + vendored sql.js); serve over HTTP for ELK and real-case loading. Proof
of concept — result overlays are illustrative, not a power-flow solution. It
demonstrates: resizable AC/DC busbars with taps, IEC 60617 symbols, a
deterministic orthogonal bus-lane router, a hybrid AC/DC demo (110 kV AC →
110/10 kV transformer → 10 kV AC → VSC → ±20 kV DC); **node-breaker** (breakers/
disconnectors on feeders + a bus-section tie); an **IEC 60617 palette** with
click-to-place; **result overlays** (voltage bands, loading %, flow arrows,
honest illustrative banner); **loading a real built-in case** from a running
`run_gui_server` via the P0 adapter (`/api/cases`, `/api/session/load_builtin`);
and **in-browser SQLite** Save/Open `.sqlite` via sql.js.

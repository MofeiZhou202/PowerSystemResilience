# Resilience Edition Contract

Status: living implementation contract for the shared source tree.

The Resilience edition profile additionally provides `scenario_hazards`, the
backend-owned parameter schema for typhoon, rainstorm and lightning. See
[weather scenarios](../modules/resilience/weather_scenarios.md) for the model,
JSON contract and authored-outage-window restoration mode. The Full/Trial
profile key sets are unchanged.

Resilience Edition is a build, runtime-capability, GUI, Python, and Windows
packaging profile. It is distinct from the `resilience/` analysis module and
does not fork the Full implementation. `HACDCPF_RESILIENCE_EDITION` and
`HACDCPF_TRIAL_EDITION` are mutually exclusive. Restricted editions retain the
common library sources but admit only their declared public capabilities.

## Discovery domains

Two discovery documents serve different state and must not be merged:

- `GET /api/edition` describes the process-global product profile used by the
  GUI and legacy `/api/session/*` clients. Its
  `schema` is `hacdcpf.edition-profile.v1`; it includes the edition, workflow,
  indicators, frontend/I/O/module lists, limitations, route policy, and the
  legacy-session `analysis_catalog`.
- `GET /api/v1` describes the isolated, multi-session asynchronous runtime. Its
  `analyses` list contains only job IDs executable through `/api/v1` (currently
  `power_flow` and `optimal_power_flow`), not every retained legacy route.
  `known_disabled_analyses` names SDK-known analyses that are not v1 job types.

The top-level `/api/edition` `analyses` array remains the small v1-compatible
PF/OPF list for compatibility; it is not the legacy-session catalog. The three
surfaces `/api/edition.analysis_catalog`, `/api/edition.analyses`, and
`/api/v1` discovery therefore have separate meanings.

`analysis_catalog` has the exact object shape:

```json
{
  "schema": "hacdcpf.edition-analysis-catalog.v1",
  "entries": [
    {
      "name": "distribution_resilience",
      "route": "/api/session/run_distribution_resilience",
      "method": "POST",
      "enabled": true
    }
  ]
}
```

The catalog object contains exactly `schema` and `entries`; each entry contains
exactly `name`, `route`, `method`, and boolean `enabled`. It covers all 69
canonical legacy/session analysis names known to the Python
`AnalysisSpec.name` catalog. A server name is the stable canonical identifier.
Client conveniences such as `resilience` -> `distribution_resilience` and
`integrated_energy` -> `campus_ies` are aliases resolved locally and are never
additional server IDs.

`POST /api/edition/analysis_plan` maps selected indicator IDs to a
backend-ordered, user-confirmable workflow. It does not execute solvers or
manufacture a result. Resilience Edition omits carbon indicators.

For Resilience Edition the returned workflow is exactly:

```json
[
  {"id": "metric_selection", "label": "指标选择"},
  {"id": "scenario_selection", "label": "场景生成与选择"},
  {"id": "proactive_defense", "label": "主动防御"},
  {"id": "rapid_recovery", "label": "快速恢复"},
  {"id": "metric_output", "label": "指标输出"}
]
```

The proactive-defense step is currently a non-blocking `skipped_unavailable`
product step; it does not create a solver request or canonical analysis. The
existing `distribution_resilience` endpoint is the sole recovery execution
entry and is presented as rapid recovery with its legacy execution mode
explicitly disclosed. Metric definitions and values are served by the backend
catalog/evaluator, not derived in the browser.

The Resilience GUI uses a separate product surface, `#resiliencePortalRoot`, rather than renaming the legacy workflow/module bar. It contains a fixed product header, a five-step vertical navigator, a central task panel, and a conclusion/evidence panel. Its selectors are scoped under `.resilience-portal`; the hidden legacy `#appShell` remains an inert compatibility host for Canvas, synchronization, existing scenario generation, and parameter collection. Full and Trial continue to mount and operate the legacy shell.

The portal state keeps `selectionRevision`, `scenarioRevision`, `modelRevision`, `runRevision`, and a local request revision distinct. Recovery input is layered as `baseRecoveryConfig` + atomic `scenarioDerived` + current-scenario `scenarioOverrides` + persistent non-scenario `userOverrides`; own-property semantics preserve explicit `0`, `false`, and empty strings. Selecting another representative replaces faults/profiles/identity together, clears only scenario fault overrides, preserves solver/MIP/switch/MESS/horizon overrides, and invalidates prior recovery/metric artifacts. It accepts only catalog data and backend result contracts through the narrow adapter in `web/js/app.js`; it does not fetch directly, duplicate recovery payload construction, or calculate scientific values in the browser. A scenario must be explicitly selected before recovery. Scenario identity is surfaced as `scenario_ref`, `scenario_revision`, and `scenario_digest`; these are provenance metadata and do not turn the existing recovery solver into a scenario identity validator.

Only clusters carrying both a backend `representative_id` and representative object are selectable. There is no synthetic family-level `generated_resilience` fallback. The scenario projector preserves domain-qualified AC/DC fault identities and validates profile metadata/samples without compressing invalid time positions. Invalid profile length, step, or non-finite samples block recovery explicitly. The visible recovery form shows profile source/bindings and both requested and repair-aware effective horizon provenance; the final payload is still produced by the single `collectResilienceParams()` collector in `web/js/app.js`.

The scenario step also owns the model entry point needed by a fresh session: it lists and loads the resilience default built-in case (`dist33_microgrid_der`), lists and loads MATPOWER files, and imports a system JSON file through the adapter. It displays the original resilience controls rather than hiding them in the compatibility shell: per-typhoon-level cluster count, reduction method, baseline comparison, intensity-level selection, derived candidate counts, and the fixed 48-hour time-domain boundary. The recovery step exposes the semantic recovery configuration (model/solver, AC/DC fault IDs and timing, horizon/load scale, switch/reconfiguration, MESS, and MIP settings) and maps it to the one canonical recovery request. Structured model, scenario, and recovery errors remain visible in the portal; a fresh session must not attempt scenario generation before a model has been loaded.

The proactive-defense page is deliberately informational: it shows `skipped_unavailable`, `blocking=false`, and `execution_created=false`, with no run button, task, HTTP request, or recovery-derived result. Rapid recovery calls only the existing `POST /api/session/run_distribution_resilience` path. Metric output calls only `POST /api/session/resilience/metrics`, carries the selected IDs, selection revision, current model revision, `t_sp` target ratio, and explicit APDA/RES approximation consent, and renders nullable backend results with status, provenance, limitations, missing dependencies, approximation, and censoring. It never substitutes zero for an unavailable value or presents one deterministic run as a probability or expectation.

The portal owns scoped scenario-generation and recovery evidence renderers. They accept an actual portal `HTMLElement`, purge prior local Plotly instances before replacement, pass elements rather than document-global string IDs to Plotly, and never write legacy result panels or trigger legacy navigation. Scenario evidence includes backend summary/warnings/audit, representative-cluster and fault tables, maximum-wind/fault-count coverage, and representative profiles. Recovery evidence normalizes flattened arrays and bounded `time_axis` + `steps/hourly` artifacts while preserving backend nonuniform times; it shows demand/served/shed, restoration, fault/repair/switch counts, fault lifecycle, MESS power/energy/position, backend KPIs and limitations with readable tables. Missing values remain unavailable. These plots are presentation evidence only and never calculate authoritative Chapter 3 metrics or imply that active defense ran.

The Resilience portal also provides top-level **工作流 / 配电系统架构** tabs. It reparents the complete existing `#canvasContainer` once into `#resiliencePortalArchitectureHost`; no Canvas, SVG, WebGL overview, minimap, legend, or ID is cloned. Switching only changes tab/surface ARIA, `hidden`/`inert`, and interaction state, then calls the presentation-only `Canvas.refreshHostViewport()` / `NetworkOverview.refreshViewport()` resize path. It does not reload/synchronize the model, rebuild the overview graph, fit the camera, or change model/scenario/run identity, selection, zoom, pan, authored layout, or AC/DC stable references. Full and Trial retain the original Canvas owner and lifecycle.

The registered GUI contract verifies the isolated portal and Full/Trial compatibility with mocked desktop/mobile Chromium. Its executable workflow fixture also verifies fresh-state prerequisite locking, default built-in/MATPOWER/JSON model entry, the Resilience-only scenario payload and user-edited generation controls, duplicate-submit suppression, stable scenario identity, proactive-defense zero-request behavior, selected faults plus 48-hour profiles entering exactly one recovery request, semantic recovery settings, scenario A/B atomic reselection and override layering, exactly one backend metric request without rerunning recovery, scoped element-target Plotly rendering, nonuniform recovery time coordinates, plot purging on step revisit, backend MESS evidence, backend artifact revision use, explicit nullable unavailable output, and immediate structured generation errors. A separate live headless-Chromium smoke against the already-running Resilience server completed model load, reduced scenario generation, explicit selection, proactive-defense no-request, one recovery request, and one metric-evaluator request. That running executable predates the current scenario-identity response echo, so the portal accepts only its distinguishable legacy `0`/null omission and records `portal_provenance_compatibility`; any nonzero mismatched identity still fails closed. The current source server echoes the request identity directly. The GUI server source compiled and linked after the earlier portal contract changes, but copying that executable over the currently running server is intentionally not performed while the process holds the target file. Current scoped-renderer and architecture-switch changes have mocked-browser/static validation only unless separately noted in the development status.

`GET /api/session/resilience/metric_catalog` is available only in Resilience
Edition and returns 42 entries: 18 Chapter 3 metrics (8 pre-disaster,
3 during-disaster, 7 post-disaster) and 24 operational metrics, with definition version
`book_ch3_2026.2`. Formulas, event-time defaults, nullable behavior and evidence are
maintained in [Web resilience metrics](../modules/resilience/metrics.md). `POST /api/session/resilience/metrics` accepts a
`resilience_metric_request_v1` that references an existing in-process run
artifact by `run_id`; it does not rerun recovery. Unknown or evicted artifacts
return `UNKNOWN_RESILIENCE_RUN`, and a model revision mismatch returns
`STALE_RESILIENCE_RUN` or `MODEL_REVISION_CONFLICT`. The first implementation
keeps at most 16 artifacts in memory and does not promise recovery across a
server restart. Results preserve nullable values and explicit metric status,
assumptions, limitations, approximation, censoring, and missing dependencies.

## Fail-closed capability boundary

The route manifest classifies each method/path pair as retained, known disabled,
or unclassified. In Resilience Edition:

- retained routes proceed to their normal handlers;
- known disabled legacy routes return HTTP 403 with nested
  `EDITION_FEATURE_DISABLED` data;
- an unclassified API route also returns HTTP 403 and reports
  `feature=unclassified_api` rather than becoming reachable by default;
- `/api/v1` accepts an enabled job ID, returns flat HTTP 403
  `edition_feature_disabled` for a known-but-disabled canonical analysis, and
  flat HTTP 400 `unsupported_analysis` for a name outside the canonical
  catalog.

A representative legacy-route denial is:

```json
{
  "error": {
    "code": "EDITION_FEATURE_DISABLED",
    "edition": "resilience",
    "feature": "market",
    "message": "This capability is not included in the Resilience edition."
  }
}
```

Full Edition uses the exact singleton wildcard array `["*"]` for
`enabled_modules`, `frontend_modules`, and `enabled_io_formats`. Trial and
Resilience must enumerate explicit entries and must never contain `"*"`.

The GUI fetches `/api/edition` before making the application shell available.
It validates the exact profile envelope, product, module/I/O/workflow lists, and
catalog schema; within the catalog it rejects malformed/extraneous entry fields
and duplicate composite entries, while runtime UI admission remains governed by
the exact edition capability lists. Failure to fetch or validate the profile
leaves the shell unavailable; it must not fall back to Full capabilities.

The Resilience GUI intentionally exposes a smaller surface than the retained
backend research library. Its frontend modules are exactly model I/O, parameter
library, topology analysis, scenario generation, power flow, OPF, and resilience
analysis. Consequently, the Security and Dynamics workflow is absent; Planning
and Operation contains only scenario generation; and Low Carbon and Resilience
contains only resilience analysis. Scenario generation requests are forced to
`regular.enabled=false`, `reliability.enabled=false`, and
`resilience.enabled=true` by both the GUI and the Resilience HTTP handler, so
hidden families are not computed. Resilience case import fail-closes on a
non-resilience family, sends only the pure AC/DC system object to model import,
and retains scenario metadata, faults, and 48-hour multiplier profiles in the
GUI for the resilience request. It does not call the separately disabled time
series configuration route.

The Python process-global client independently parses `/api/edition`, requires
an exact catalog, reconciles all names/routes/methods against its local known
catalog, and filters the connected catalog by `enabled`. It raises
`UnknownAnalysisError` before transport for a name outside that universe and
`AnalysisDisabledError` before transport for a known disabled canonical name.
Supported aliases are canonicalized before this decision. The v1 client applies
the corresponding tri-state rule to its independent `/api/v1` discovery.
Malformed or incomplete discovery fails closed as `TransportError`.

## AI safety and result honesty

Edition capability admission and `ToolPolicy` answer orthogonal questions:

1. the connected runtime profile decides whether an analysis exists and is
   enabled in that edition;
2. `ToolPolicy` decides whether the caller may perform the analysis' effect
   class (`read`, `analyze`, `modify_model`, or `control`) and whether explicit
   approval is required.

Allowing an effect cannot enable a disabled edition capability. Conversely, an
enabled capability still cannot bypass effect or approval policy. Family tools
ultimately call the same capability-gated Python client seam; the v1 tool path
uses v1 discovery for job submission.

A successful transport is not, by itself, a scientifically usable result.
Fallbacks, approximations, time limits, unsupported model coverage, skipped
physical validation, stale results, and convergence/feasibility state remain
explicit in the returned result and its limitations.

## Restoration certification boundary

Resilience Edition exposes AC/DC hybrid assessment and restoration workflows,
but ordinary restoration feasibility is not a certified dynamic-safety result.
The profile states
`ordinary_feasibility_is_certified_safe=false` and marks dynamic certification
as not exposed in the first release. Standalone transient and small-signal
research endpoints are disabled. A result may claim dynamic safety only when a
separate explicit certification contract and evidence say so.

The Resilience GUI now presents the product workflow as five steps:

- **指标选择**: catalog-driven selection from the Chapter 3 metric definitions;
- **场景生成与选择**: resilience/typhoon scenario creation and explicit selection;
- **主动防御**: `skipped_unavailable`, non-blocking, with no solver request;
- **快速恢复**: the existing full distribution-resilience execution kernel;
- **指标输出**: backend evaluator results for the selected metrics only.

The legacy navigation still contains the retained frontend module IDs for
compatibility, but those IDs are presentation capabilities rather than new
canonical analysis routes.

`applyResilienceNavigation()` in `web/js/app.js` (gated on
`edition === 'resilience'`; Full and Trial markup and labels are untouched):

- **规划与运行 → 场景生成**: the scenario-generation module is renamed
  台风致灾场景生成, followed by a disabled placeholder
  更多极端灾害场景模拟（制作中…）.
- **低碳与弹性 → 弹性分析**: the resilience module is renamed 完整弹性分析,
  followed by three view modules, each with dedicated visualizations (no
  charts are shared with 完整弹性分析): 主动防御 (`proactiveDefense`,
  during-disaster withstand only — supply-gap curve with disaster-stage
  background bands, fault occurrence–repair lifecycle Gantt, critical/high
  priority unserved-load timeline, priority load-shedding donut; topology
  restoration/reconfiguration content such as switch actions is deliberately
  excluded), 快速恢复 (`rapidRecovery`, post-disaster restoration — supply-ratio
  recovery trajectory annotated with repair events, cumulative repair
  progress steps, cumulative unserved energy, MESS cumulative delivered
  energy and bus-position trajectories when MESS data exists), and 弹性指标
  (`resilienceMetrics`, metrics dashboard — resilience radar, energy-balance
  waterfall, recent-run comparison chart, full KPI table) — and a disabled
  placeholder 薄弱环节（制作中…）.

The three view modules share one backend run
(`POST /api/session/run_distribution_resilience`, result cached in
`_lastResilienceData`) with 完整弹性分析; they are presentation views, not
separate analyses, and their run buttons trigger the same solver call.
Fault-event, time-series, and solver parameters are edited only in the
完整弹性分析 sub-toolbar. Placeholder buttons carry `disabled` /
`aria-disabled` and are intentionally absent from `KNOWN_FRONTEND_MODULES`, so
they cannot be activated even programmatically.

`frontend_modules` for the resilience profile therefore lists
`proactiveDefense`, `rapidRecovery`, and `resilienceMetrics` alongside the
pre-existing modules; the C++ profile (`src/server/edition_profile.cpp`), the
frontend mirror (`EDITION_FRONTEND_MODULES` in `web/js/app.js`), and the e2e
fixture (`tests/e2e/resilience_edition_gui_e2e.mjs`) must stay in exact
order-equal sync because profile validation asserts exact array equality.
The resilience result carries no per-hour island membership, so the
主动防御 view reports isolation/reconfiguration evidence only (stage
timeline, switch actions, open/closed branch counts) and must not claim an
island visualization.

Resilience Edition presentation defaults: the built-in case selector defaults
to the resilience flagship case `dist33_microgrid_der` (frontend override of
`/api/cases.default_case`), the resilience MIP solver defaults to HiGHS, and
all four resilience result groups auto-widen the right results panel via the
`setActiveResultGroup` wide-group list. The three view-module result
containers are exempt from the generic `.topo-table-wrap` 200px max-height
cap (like `#resilienceResults` itself), and the resilience result groups
stretch the full results-area height through a flex chain down to the chart
grid.

## Calculation summaries and home navigation

The scenario-generation, rapid-recovery, and metric-output pages place their
calculation summaries at the end of the result content in collapsed native
`details` elements. Scenario generation keeps its generation timing, audit and
summary warnings there; recovery keeps run metadata and backend execution
evidence there; metric output keeps raw results and calculation assumptions
there. Actionable request errors remain visible beside the workflow controls.
Result charts remain outside the collapsed summaries. This changes presentation
only, with no solver, metric definition, or API change.

The CoPlanning `mac` branch was inspected at commit
`966947339444d585a3db0aef26d4211bd3d3852a`. Its
`planning/web/mv_network/app.js` defaults to the projects view; `index.html` and
`projects-view.js` implement the welcome page, demonstration walkthrough, project
journey and state-dependent next action. The calculation workbench, comparison
view, help center and project backend also provide source evidence for task
records, result comparison, contextual help and persistent project versions.
This is source inspection, not a runtime validation of CoPlanning.

Implemented navigation now defaults to home, with workflow, architecture, local
projects, task history, comparison and help tabs. Safe `#resilience/` routes
survive refresh and browser history. The five-step calculation contract remains
inside the workflow. New analysis and the demonstration select backend-catalog
`available` metrics; the demonstration loads the existing 33-node case and
prepares one TD cluster without automatically generating or solving.

`web/js/core/resilience_workspace.js` owns an IndexedDB store for immutable saved
versions and task records. Each version includes Canvas rich-model JSON, explicit
configuration fields, scenario evidence, catalog and returned results. Metric
success saves a version automatically; explicit saves create additional versions.
Saving does not replace the backend model. Opening imports the saved model and
restores configuration but clears active recovery and metric handles. Historical
results remain read-only. Transfer is schema-validated JSON, bounded to 32 MiB.
Storage is browser/origin local, survives backend restart, and is not a shared
server project database. Export is necessary for cross-device transfer or backup.

Tasks show request state, elapsed wall time and errors, and connect recovery
cancellation to the existing task manager. A cancel request is not a confirmed
solver stop. Reloaded unfinished records are marked interrupted. The synchronous
backend does not supply internal solver progress or percentages; this change adds
no progress stream and no HTTP routes.

Basic mode collapses advanced controls, fault details and long profile provenance;
expert mode expands them. Next-action guidance, help, result highlights and
domain-qualified fault-to-Canvas links use existing capabilities. Editing recovery
parameters marks old results stale. Comparison preserves backend values and
curves for up to four saved versions. Model/scenario/time-axis/definition/parameter
differences, imported provenance, stale results and missing evidence prevent
delta calculation; only finite, non-censored `computed` values with equal units
receive a baseline difference. No synthetic ranking is produced.

The [user guide](../guides/resilience_workspace.zh.md) describes storage and
workflow boundaries. Registered GUI coverage includes version round trips,
recalculation, comparison, domain-qualified navigation and desktop/mobile layouts;
the living status records actual backend/browser evidence. Enterprise project
governance and server-wide shared storage remain outside this implementation.

## Build, dependency, and Windows package

The Windows build profile is:

```powershell
cmake --preset windows-resilience-release
cmake --build --preset windows-resilience-release --target run_gui_server
ctest --preset windows-resilience-release -L edition --output-on-failure
```

It inherits `windows-source-release`, enables Resilience, disables Trial, and
uses the locked in-repository `MIPSolvers/` import. The dependency lock records
upstream commit `c6f77f297350b357ff30cc96d9234b2031fd316c` and imported tree
`782f8745b7e3d39754d02999eeccfe11c04528dc`. An explicit
`MIPSOLVERS_SOURCE_DIR` override is a developer path only; formal packaging
requires the clean committed in-repository tree. There is no implicit
`../MIPSolvers` fallback.

Windows zlib and sequential oneMKL/PardisoMKL outputs are staged below
`build/windows-dependencies/`. Generated `.lib`, `.dll`, and installation
content must not be written into the locked source import.

`tools/package_resilience_windows.ps1` implements an explicit Resilience
resource and runtime-DLL allowlist; it must not be replaced with recursive
copies of `data/`, `docs/`, or `external_data/`. It requires source-mode locked
MIPSolvers, disables Gurobi/CPLEX/IPO/OpenDSS while retaining Ipopt with
sequential PardisoMKL, audits recursive PE dependencies,
records a per-file manifest and ZIP SHA-256, and invokes
`tools/verify_resilience_windows_release.ps1`. The verifier extracts to a clean
path containing spaces, restricts `PATH` to the package and Windows system
directories, rejects forbidden resources/DLLs, checks the Resilience profile,
fail-closed legacy and v1 behavior, and runs a v1 power-flow job.

These scripts define the package contract; they do not prove that a package has
been built successfully. Executed build, API, GUI, Python runtime, archive, and
clean-extracted-package evidence belongs in the living development status.

## Acceptance boundary

Verified in the current Windows worktree on 2026-09-19:

- Full, Trial, and Resilience MSVC Release edition-profile binaries built; their
  focused unit contracts passed 6/6 in each edition.
- Trial HTTP acceptance passed 1/1. The focused Resilience set passed 8/8:
  six edition unit tests, one HTTP acceptance test covering the complete catalog,
  fail-closed routes, independent v1 tri-state discovery, revision/ETag/stale-result
  semantics, PF/OPF, bounded graph/result resources, cancellation, cleanup, and
  complete Windows rejection bodies, plus one registered mocked-Chromium contract.
- The registered mocked-profile Chromium contract covered desktop/mobile
  Resilience, Trial, Full, delayed, hung, HTTP, JSON, schema, edition, module,
  workflow, wildcard, hash, ownership, dynamics no-fetch, overflow, and
  dynamic-model round-trip cases.
- The static Windows packaging contract and Windows PowerShell 5.1 parsing of all
  five release/dependency scripts passed; JavaScript/Python E2E syntax, all
  CMake preset parsing, and `git diff --check` also passed.

The complete Python SDK/AI runtime suite is still being finalized. Resilience
package creation and clean-extracted execution have not run: generated Windows
dependencies/build output are absent and formal provenance correctly requires
the staged `MIPSolvers/` import to exist in `HEAD`. Test registration or script
presence alone is not package evidence.

See [runtime API](../reference/runtime_api.md),
[Python API](../reference/python_api.md),
[comprehensive Python API design](../reference/python_comprehensive_api_design.md),
[cross-platform build](cross_platform_build.md), and the
[living development status](../overview/development_status.md).

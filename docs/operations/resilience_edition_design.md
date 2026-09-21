# Resilience Edition Contract

Status: living implementation contract for the shared source tree.

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

## GUI navigation framework

Resilience Edition reshapes the shared GUI navigation at runtime through
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

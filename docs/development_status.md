# Development Status

Updated: 2026-08-14

This is the living handoff for verified build state and active engineering work.
Update it in place; do not create dated copies. Source, registered tests, and
the current Git worktrees remain authoritative.

## Verified baseline

| Scope | Result |
|---|---|
| Required MIPSolvers source | Current pin `adfc98f3bd4721682f72f65ba030a18cfad1749d` on MIPSolvers `main`; the full regressions below were established at the older `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0` baseline |
| `full-dev` regression | 1430/1430 registered tests completed without failure; 3 condition-dependent tests skipped |
| `macos-release` regression | 1425/1425 registered tests completed without failure; 3 condition-dependent tests skipped |
| Graph ASan/UBSan subset | 28 cases, 113 assertions passed after the iterative Tarjan fix |
| Reliability ASan/UBSan | Complete three-stage suite 25 cases/1339 assertions; `case33mg_acdc` 477 assertions and five consecutive parallel repeats passed; `test_1_no_sop` 79 assertions |
| Market ASan/UBSan | Complete suite 22 cases/845 assertions; focused initial root-cut case 1/42 |
| Other sanitizer subsets | Thread pool 4 cases/6 assertions; `test_hacdcpf` 26/89; `test_acopf_dcopf_crossval` 10/84; `test_power_flow_math_audit` 40/204 |

## Trial edition integration

Current `main` reimplements the two `trial_design` commits on top of the newer
reliability and GUI work. The backend owns an embedded edition profile,
indicator-to-analysis plan, and fail-closed method/path policy. Known disabled
routes return `TRIAL_FEATURE_DISABLED`; an unclassified future API route is
also denied. The GUI consumes the profile and plan, removes disabled controls,
avoids the eager dynamics-schema request, and keeps each solver execution
explicit. Windows packaging requires a clean pinned MIPSolvers checkout, Trial
CTest success, required runtime/data presence, and an extracted-package startup
smoke test before hashing the manifest and ZIP.

The Trial verification below used the earlier MIPSolvers `7b4cba8` pin. That
commit guards the optional CHOLMOD augmented-factor calls and restores the
no-SuiteSparse build. Its clean Release build with SuiteSparse, SuperLU, MKL,
Ipopt, Gurobi, and PaPILO disabled passed `test_ipm_solver`: 30 cases and 188
assertions. The current dependency upgrade and OPF verification are recorded
separately below. No claim is made for a Windows package yet.

A clean Debug Trial build against that checkout rebuilt `test_edition_profile`
and `run_gui_server`. All five registered Trial tests passed in 8.19 seconds:
three C++ profile/policy/plan cases, the fail-closed HTTP E2E, and the
Playwright GUI E2E. Manual browser inspection at 1440x1000 and 390x844 found no
page-level horizontal overflow, toolbar overlap, or clipped controls; the
mobile indicator-plan band was expanded so all nine backend-ordered links end
above the workspace. A separate clean full-edition Debug build passed the full
profile case, `runtime_api_v1_e2e`, `reliability_workflow_e2e`, and
`reliability_dimension_validation_e2e` in 57.93 seconds. JavaScript syntax,
Python AST parsing, preset JSON parsing, required package-data presence, and
`git diff --check` passed. PowerShell is unavailable on this macOS host, so the
Windows preset, staged DLL startup, manifest, and ZIP creation remain
unverified.

The two full CTest results establish the normal build baseline. They do not
claim that every sanitizer entry point is green.

## Three-phase hybrid OPF Phase I/II integration

The dependency pin now advances to MIPSolvers `adfc98f`. For the monolithic
three-phase hybrid OPF NativeIPM path, Phase I restores a primal point in the
assembled OPF coordinates and generates equality multipliers, positive
inequality multipliers, and positive slacks. Restoration uses a declared sparse
state/basic Newton system when available and SparseQR otherwise; it no longer
materializes a dense Jacobian or forms `J J^T`. Iteration, total primal-plus-
dual factorization, and per-iteration backtrack caps are hard work bounds. The
wall-clock deadline is cooperative because an in-flight sparse factorization
cannot be interrupted. Finite-budget Phase I never invokes Ipopt. The
`NLPModel` exposes an
independent original-per-unit maximum violation callback. Phase II requests
MIPSolvers' `primal_feasible_start` and `preserve_initial_point` policies; the
adapter preserves the point only when its independent constraint and variable-
bound audit is finite and no greater than `tol_primal`. Otherwise its ordinary
infeasible-start interiorization remains active. Public results report the
Phase I certificate, budget consumption and termination, whether dual
initialization completed, whether Phase II requested and accepted the start,
and the selected linear backend. Dual quality is the state/basic stationarity
fit (or SparseQR normal residual), not full-space stationarity: Phase I leaves
the reduced gradient for Phase II instead of duplicating optimization. A Phase II
zero-iteration termination honestly reports `unselected` because no KKT
factorization occurred.

On AppleClang 21, arm64 macOS, Release, the rebuilt MIPSolvers
`test_ipm_solver --rng-seed 1` passed 30 cases and 203 assertions. The rebuilt
HySim `test_three_phase_hybrid_opf --rng-seed 1 --reporter compact` passed 11
cases and 139 assertions. The focused graph-reduced-to-Full primal-dual transport passed 16
assertions: the Full Phase I violation was approximately `8.08e-15`, Phase II
requested and accepted the preserved point, and final primal/dual/
complementarity residuals were approximately `8.08e-15`, `2.56e-7`, and
`1.00e-7`. This small case terminated before a Phase II KKT factorization, so
the backend correctly reported `unselected`. The added O(rows + variables)
audit was not timed against a disabled-audit baseline, so the predeclared
under-1% performance prediction remains unverified.

The added sparse large-case regression has 360 three-phase buses, 1080 phase
nodes, and 2175 OPF variables. With two Newton iterations, five total
factorizations, four backtracks per iteration, and a 5000 ms cooperative time
budget, Phase I used SparseQR and three total factorizations. It reduced the
original-coordinate violation from `1.0e-3` to `2.66788e-9` in approximately
1539 ms and produced a `7.81839e-15` dual-fit residual. The full-space
stationarity was `2.2572`, as expected for a feasible but non-optimal point.
The zero-factorization and zero-backtracking tests also passed. The
predeclared `>=2x` restoration-speed prediction has not been measured against
the removed old path and remains unverified. The rebuilt production
`run_gui_server` target compiles and exports the Phase I diagnostics, but no
monolithic phase-hybrid HTTP E2E currently exercises that route. A manual
probe loaded the three-bus `td_coordination_all_components` fixture, but its
monolithic OPF request returned the server's generic `Unknown server error`,
so the runtime JSON contract is not claimed as verified. No full CTest,
sanitizer run, or Windows package validation was performed for this increment.

## DER control and reliability-method comparison

The unified reliability request now applies an optional calculation-copy DER
control scenario: authored roles, forced grid-following, or promotion of
eligible controllable resources to grid-forming. Every result exposes a stable-
ID device audit with authored/effective role, reference capability,
anti-islanding and black-start credit. This is a steady-state reliability
sensitivity model; dynamic synchronization, current limiting and protection-
FRT trajectories remain explicit limitations. Disabling black-start removes
the explicit FMEA storage credit, while methods that still use a composite GFM/
black-start flag disclose that boundary.

The GUI adds deterministic, Monte Carlo and full six-method comparison sets.
Each production result carries a system fingerprint and normalized calculation
basis. The comparison backend rejects mismatched bases, omits unavailable
metrics, reports min/median/max, coefficient of variation and normalized range,
and compares weak-component rankings by stable-ID Spearman correlation, top-5
Jaccard overlap and consensus reciprocal rank. The default reporting horizon is
8760 hours across the compared methods.

Focused `macos-release` verification rebuilt `run_gui_server`. The three
registered GFL/GFM physical cases passed, `reliability_control_state_test`
passed, and the extended `reliability_workflow_e2e` passed in 8.90 seconds. The
latter used the built-in cyber-physical case to verify the per-device control
audit and same-basis component-FMEA versus failure-mode-FMEA comparison,
including statistic bounds and stable consensus identities. JavaScript syntax
and `git diff --check` passed. No complete CTest or sanitizer run was performed
for this increment.

The reliability calculation-guide refresh reused the existing
`macos-release/tests/run_gui_server` binary without rebuilding it. A direct
`reliability_workflow_e2e.mjs` run completed successfully and reproduced the
documented cyber-physical EENS sequences `1.0000005 -> 4.30000075`,
`0.10000005 -> 4.400001`, and `10.000005 -> 20.000005 MWh/year`. Direct
production-API capture for `dist33_microgrid_der` returned the coupled-model
baseline EENS/LOLE/SAIDI of `4.276339 MWh/year`, `1.261167 h/year`, and
`72.861707 min/customer-year`; promoting 11 eligible resources to steady-state
GFM reduced those values to `0.010627`, `0.027833`, and `0.172683`. These
Dist33 values are worked-case evidence, not registered cross-version numeric
thresholds. The validity flags and directional checks remain the enforced
contracts. No full CTest or sanitizer run was performed for this documentation
refresh.

## Reliability configuration enhancement

The dirty worktree now registers `reliability_dimension_validation_e2e`, which
uses the production unified reliability route for seven physical,
information, and intelligent scenarios on both `dist33_microgrid_der` and
`comprehensive_hybrid_acdc`. The registered `macos-release` CTest passed 1/1
in 0.79 seconds. Load scale 1.3 changed EENS from `0.149400` to `0.194220`
and from `15.162140` to `35.964740 MWh/year`; disabling six restoration
resource classes changed it to `1.656000` and `21.312060`. With automatic and
manual endpoints of `0.014940/0.334800` and `15.449294/267.580000`, the
factorized intelligent probability was exactly `0.6500736`, effective
automation probability was `0.58506624`, and measured joint EENS was
`0.1476607125` and `120.0668358520`, matching the affine total-expectation
identity within the predeclared `1e-8` tolerance.

The comprehensive case retained a signed duration attribution of
`-0.251539 MWh/year` and a control attribution of `+104.869081 MWh/year`.
This is not clamped: the derivation explicitly permits non-monotone
counterfactual attributions when a longer switching window shortens the
fixed-MTTR repair window or changes topology. The response localized negative
duration terms to `HV-Line-110kV` and `F3-Line-15-16`; total endpoint ordering
and the exact decomposition still passed. Both cases declare the Level-1
independence boundary, no joint class probabilities, and no protection/FRT
coupling. The test reused the existing `macos-release/tests/run_gui_server`
binary; CMake reconfiguration disclosed that local MIPSolvers HEAD
`82f4583d4f9070549c75a1ffa5d8d3bac8d9dc6f` differs from the recorded pin
`60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0`, so no reproducible rebuild or
full-suite claim is made for this increment.

The current dirty HySim worktree adds a model-bound reliability/protection
configuration across `failure_mode`, the GUI server, and `/xjtu/`. The catalog
now includes previously absent AC/DC/three-phase buses, transformers, bus-load,
regulator, dedicated DC storage, LCC, and three-phase component families. User
overrides are sparse and use stable component `.index` identities; explicit
protection zones and all backend schema fields round trip through the GUI.
Unsupported steady-state consequences and protection-zone targets remain
diagnostics rather than fabricated zero-impact support.

Focused verification used the existing Debug build directory with the local
dependency dirty-check override; no Release result is claimed:

```bash
cmake -S . -B /private/tmp/hysim_reliability_config_debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DHACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK=ON \
  -DHACDCPF_ENABLE_ETAP=OFF \
  -DHACDCPF_ENABLE_IPOPT=OFF \
  -DHACDCPF_ENABLE_OPENDSS=OFF
cmake --build /private/tmp/hysim_reliability_config_debug \
  --target test_reliability_resolver run_gui_server -j4
```

`test_reliability_resolver` passed 53 cases and 347 assertions. Registered
CTest `reliability_configuration_e2e` passed in 40.09 seconds and verified
transformer and AC-branch overrides, resolved-value display for every returned
mode and all 12 numeric fields, sparse GUI save, all mode/protection fields,
invalid-input rejection, duplicate-name identity, same-model preservation,
component-array and replacement-model reset, explicit non-application by other
methods, custom protection probabilities, semantic property grouping across 27
mapped Canvas component kinds, a non-generator property-panel edit, and a
390 px layout after overriding a persisted 1180 px panel width. The focused
`topology_transformer_link_e2e` passed in 3.71 seconds and verified that both
standalone and branch-backed transformer rows select the correct Canvas glyph.
The automated comprehensive case covers the core AC/DC equipment families;
manual GUI inspection of that case showed 121 components, 266 modes, and all
3192 numeric cells populated with finite resolved values.
`node --check` passed for the changed JavaScript files. Browser inspection at
1440x1000 and 390x844 found no
page-level horizontal overflow; the large mode table uses bounded independent
scrolling; the calculation principles, all five execution stages, and the
protection editor remain reachable. Only the focused `macos-release` targets
described below were rebuilt; no complete Release/full-dev build, full CTest,
or sanitizer run was performed for this enhancement.

Three-stage restoration now consumes the same session protection rows without
claiming that failure-mode overrides apply. For each initiating fault it splits
frequency into successful-reclose, primary-cleared, backup-cleared, and
unresolved scenarios; configured clearing times enter Stage 1, the backup zone
expands the outage set, and primary-plus-backup failure blocks restoration.
Successful reclose is excluded from sustained IEEE 1366 indices, while the
result audits initiating, transient, and sustained frequencies and their exact
conservation. The GUI renders this protection/recovery audit and the governing
formula from the response.

Focused `macos-release` verification now passes 35 regular
`test_three_stage_reliability` cases with 1454 assertions. In addition to exact
`r=0.8` scaling of sustained EENS to 20% of baseline, primary/backup frequency
conservation, backup-zone expansion, and configured clearing times, the suite
now verifies coupled AC/DC nodal balance and voltage/radial constraints,
bidirectional VSC and DC-DC efficiency, DC line limits, four DC DER fault
families, AC/DC/mobile-storage stage chronology, mobile-storage transit state,
VPP PCC-boundary outages, and equal-number AC/DC bus-ID isolation. A direct
`build_dist33_microgrid_der()` regression returns `ok=true` with the coupled
scope and all branch-flow/voltage/radial/DC/VSC validity flags true. The
registered `reliability_configuration_e2e` and
`reliability_default_policy_e2e` passed in 39.39 and 7.16 seconds after
checking both unified and legacy API protection fields, strict-policy rejection,
unified fallback execution, and the empty-configuration JSON contract. A
direct-C++ timing probe on the
same arm64 macOS 26.5.2 host, Apple clang 21, `-O3 -DNDEBUG`, commit `493f6351`
used two warmups plus five measured repetitions per variant. Across three
independent runs, median no-configuration/primary/primary-plus-backup times were
`0.473--0.488 / 0.477--0.480 / 0.630--0.648 ms`; the two-scenario/single-
scenario ratio was `1.320--1.353x`, within the predeclared approximately `2x`
upper-order prediction because parsing, assembly, and healthy-state solves are
shared fixed costs. The hidden diagnostic is reproducible with
`./build/macos-release/tests/test_three_stage_reliability
'Protection scenario runtime probe'`; it is excluded from regular CTest.
Manual inspection of the rebuilt GUI loaded `dist33_tie_demo`, selected the
three-stage consequence model, and displayed its equation and five-stage
workflow. The follow-up browser regression closes a discovered policy mismatch:
`missing_only` now runs with the backend's declared unified fallback values,
while `case_data_only` alone blocks zero-coverage input and offers the explicit
`Apply parameter library and run` path. Selecting three-stage restoration now
defaults the GUI fault loop to serial execution, avoiding queue amplification
against the process-serialized solver while retaining an explicit parallel
opt-in. Earlier Dist33 GUI evidence in this paragraph predated the coupled DC
model and is superseded by the current source-level and workflow contracts
below. The obsolete empty-result text claiming that
the reliability and resilience backend interfaces were pending was removed.

The same dirty worktree now adds a reliability calculation workflow to the GUI:
method-specific principles and equations, five persistent execution stages,
catalog/protection coverage, and response-backed limitation states. Because the
unified endpoint has no stage-progress stream, the GUI labels enumeration,
consequence mapping, and solving as one backend interval instead of inventing
percent completion. The registered `reliability_workflow_e2e` uses the built-in
`cyber_physical_reliability_demo` case and passed with these directional EENS
checks (MWh/year): physical load scale `1.0 -> 1.5`,
`1.0000005 -> 4.30000075`; information availability `1 -> 0`,
`0.10000005 -> 4.400001`; dominant passive-mode failure frequency doubled,
`10.000005 -> 20.000005`. Its completed GUI stage states were
`complete/complete/limited/complete/complete`; the limited consequence state
was backed by runtime model declarations, and 390 px viewport overflow was
zero. After rebuilding the focused `macos-release` targets,
the registered `reliability_workflow_e2e` passed in 11.14 seconds and exercises
real three-stage requests for both
Dist33 variants: `dist33_microgrid_der` uses
`coupled-acdc-lindistflow-restoration-milp`, returns `ok=true`, signed VSC
dispatch, and true branch-flow/voltage/radial/DC/restoration validity flags;
`dist33_tie_demo` remains an exact AC case. The rebuilt registered
`reliability_configuration_e2e` and `reliability_default_policy_e2e` also
passed in 50.17 and 9.78 seconds. Both unified and legacy
three-stage routes execute the core solver on a dedicated 4 MiB-stack worker;
the workflow wall time is `1.21x` the previously recorded 9.20-second run,
inside the predeclared `3x` AC/DC-model expansion budget. This is an end-to-end
runtime comparison, not a solver-only microbenchmark. An additional
response-construction failure was traced under LLDB to a nullable
`const char*` passed to nlohmann JSON after a successful solve. Both routes now
construct an explicit JSON string or JSON null, closing the observed
`strlen(nullptr)` process termination for empty custom configurations. The
rebuilt `test_reliability_resolver` remained at 53 cases and 347 assertions. The broad
`gui_scale_features_e2e` transformer-link assertion also passed, but that run
was not green overall because two later, unrelated comprehensive-OPF checks did
not converge; no green result is claimed for that suite.

## Model parameter explorer

The dirty worktree extends `/api/session/parameter_library` and the Model
Parameters GUI with backend-owned presentation metadata. All 52 underlying
rules return symbols, quantities, model roles, equations, typical-range
provenance, and an equivalent-circuit family. Typical screening ranges are
explicitly separate from editable hard validation bounds. The backend now owns
a 44-entry catalog covering every physical/system family serialized by
`HybridPowerSystem`; the selector no longer depends on which families happen
to have standard-completion rules.

The current-system snapshot flattens all fields from `hacdcpf::io::to_json`,
uses domain-qualified stable identities, and groups them by engineering
semantics. Resolved failure modes are attached to their physical component by
`reliability_kind + component .index`; the legacy reliability rule groups stay
in the 52-rule JSON contract but are hidden as standalone models. The GUI
therefore shows 24 physical standard rules in the overview, complete read-only
instance fields, and reliability parameters within each matching instance.
Unknown typical/hard ranges are reported as unpublished/unregistered rather
than synthesized. Existing save/import/export/validate/apply behavior remains
intact; authored instance values remain editable through Canvas properties.

Focused Debug verification rebuilt `run_gui_server`. The registered
`parameter_contract_e2e` passed in 8.99 seconds with 44 catalog models, no
standalone reliability pseudo-models, complete field-contract checks, stable
Canvas selection, profile save round trips, and mobile overflow checks. The
registered `reliability_configuration_e2e` passed in 38.63 seconds. Real
browser inspection of `comprehensive_hybrid_acdc` found 108 stable instances;
the selected generator exposed 76 grouped rows, including 41 resolved
reliability rows, with no visible overlap. `node --check` passed for
`web/js/app.js` and the E2E script. No Release/full-dev build, full CTest, or
sanitizer run was performed for this GUI enhancement.

The same regression exercises every mappable instance family present in the
distribution case, same-number AC/DC buses, consecutive AC branches,
VSC/storage, transformer aliases, current-value refresh, unsaved profile-edit
preservation, and module/tab stability. Selecting a Canvas glyph while Model
Parameters is active follows the exact domain-qualified instance without
rebuilding or losing unsaved profile edits.

## GUI workspace density

The dirty worktree now provides a workspace-first GUI layout modeled on common
engineering simulation tools: standard and compact densities, independent
component-library/context-ribbon/inspector/console docks, a focus mode that
preserves the active module and inspector tab, and persisted layout state.
Desktop first load uses compact density with a collapsed console. At 720 px and
below the component library, contextual ribbon, console, and desktop-only
dependency chips start collapsed while global element location and the active
right-side parameter/result view remain reachable.

The registered Debug CTest selection passed 5/5:
`top_toolbar_semantics_test`, `workspace_layout_e2e`,
`topology_transformer_link_e2e`, `parameter_contract_e2e`, and
`reliability_configuration_e2e` (54.21 seconds total). At 1440x1000, focus mode
increased the measured Canvas area from 959x649 to 1115x919 while preserving the
short-circuit module, topology tab, and console contents. The layout round trip
survived reload; a real 390x844 Chromium check reported zero page-level
horizontal overflow and showed no overlapping controls. JavaScript syntax and
`git diff --check` passed. No Release/full-dev build, full CTest, or sanitizer
run was performed for this frontend-only change.

## Active module audit

The living [module code audit](module_code_audit.md) records the current
source-backed findings and audit depth. AUD-001 through AUD-011 are closed.
Focused runtime contracts now cover `graph/`, `scenario_generation/`,
`carbon_analysis/`, `integrated_energy/`, and `sppt/`; the canonical links are
in [docs/README.md](README.md).

Current-source Debug verification rebuilt all affected targets. The focused
and contract suites passed: typhoon traffic/catalog 5 cases/35 assertions,
campus IES 6/106, harmonics 52/359, EV Formulation D 15/196,
graph/Kron/round-trip 55/461, scenario generation/schema 12/105,
carbon snapshot/annual/GEC 41/574, SPPT 33/213, and the shared
resilience/reliability executable 39/360. `run_gui_server` also rebuilt and
linked successfully.

AUD-011 is closed by running StrictHiGHS restoration B&C calls on a fresh
joined thread while keeping Native on the caller thread, whose larger stack is
required by recursive sub-MIPs. The same-process regression completed five
Native-to-StrictHiGHS cycles. A 1000-cycle packaged-task/jthread benchmark
measured 0.0145--0.0187 ms fixed overhead per cycle, below the 10 ms threshold.
No full CTest, complete sanitizer suite, or external-engine cross-validation
was run specifically for this closure pass.

## Closed investigation

The HySim worktree currently contains uncommitted dependency, documentation,
graph, reliability, and graph-test changes. Re-run `git status` before relying
on this list.

- `src/graph/topology_analysis.cpp` fixes an invalidated stack-frame reference
  in iterative Tarjan traversal and preserves the exact parent edge for
  parallel-edge correctness. Focused ASan/UBSan tests pass.
- `src/reliability/three_stage_reliability.cpp` serializes process-global HiGHS
  and native B&C solve state across contingency workers; the native fallback
  uses one internal thread. Solver-bearing reliability workers request a 4 MiB
  POSIX stack through the optional `ThreadPool` stack-size parameter.
- The sibling MIPSolvers worktree has two uncommitted HiGHS changes: hash
  non-finite cut bounds from their IEEE-754 representation, and allow an
  initial root user-cut pool before the first restart. Both changes were rebuilt;
  focused and complete market sanitizer regressions pass.

The apparent `HPresolve::changeImplColUpper` container corruption in
`case33mg_acdc` was a downstream symptom of worker stack overflow, not a
presolve data-structure defect. On Darwin, the default pthread stack is about
512 KiB, while sanitizer instrumentation expanded the largest observed solver
frame to about 340 KiB. The same case completed on the main thread, and the
parallel case became stable after provisioning a 4 MiB worker stack. Five
consecutive sanitizer repeats completed in 414.63 seconds. The resulting
case33mg metrics were SAIFI 0.7833, SAIDI 10.87, and EENS 588.5; the case33bw
cross-check produced SAIFI 0.7787, SAIDI 10.60, and EENS 570.3.

## Performance closure

The dirty MIPSolvers Release binary was compared with a clean export of the
pinned commit on the same arm64 host using an A/B/C sandwich: 24 NETLIB cases,
Native and HiGHS, 3 repeats, single-threaded HiGHS, and a 30 second solve limit.
Every run was accurate (72/72 for each solver in each leg).

| Leg | Native geometric mean |
|---|---:|
| Clean A | 3.761956 ms |
| Dirty B | 3.761257 ms |
| Clean C | 3.763764 ms |
| `sqrt(A * C)` control | 3.762860 ms |

Dirty versus sandwich control was -0.0426%, within the predeclared absolute
1% acceptance threshold. After timing fields were removed, the JSON run
records matched exactly across A/B/C, including status, objective, feasibility,
iterations, and dual-pivot counts. The raw reports are
`/private/tmp/hysim_native_dual_{control_a,experiment_b,control_c}_repeat3.{csv,json}`.

## Fast orientation

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
cmake --build --preset full-dev
ctest --preset full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

Use [README.md](../README.md) for the user-facing capability baseline,
[AGENTS.md](../AGENTS.md) for architecture and invariants, and
[docs/README.md](README.md) for topic documentation.

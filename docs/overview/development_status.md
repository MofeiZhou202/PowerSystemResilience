# Development Status

Updated: 2026-09-12

This is the living handoff for verified build state and active engineering work.
Update it in place; do not create dated copies. Source, registered tests, and
the current Git worktrees remain authoritative.

## Market module documentation audit and manual upgrade

Systematic doc-vs-code audit of `src/market/` against `docs/modules/market/`
(six parallel review passes over the md contracts, the LaTeX manual chapters,
the Yunnan ancillary workflow, and the performance/validation evidence).
Overall match was high; no "implemented-worse-than-documented" or
fallback-presented-as-exact cases were found. Fixed the confirmed mismatches:

- `southern_execution_contract.md`: benchmark binary name
  (`run_southern_market_benchmark`), the stale "current auto/dual_simplex
  divergent duals" claim rewritten as pre-fixed-strategy historical evidence,
  forecast RATIONALE updated to 8-day joint sampling / 7-day clearing
  (`market_forecast.cpp:129-137`), statistics table gained the scenario-count
  and weekly-energy summary fields, forecast week-only horizon rejection
  documented, `horizon="day"` and the eight daily multiplier fields registered,
  `explain_trigger`/`recovery_pricing` defaults noted, dated snapshot wording.
- `performance.md`: CTest gating of `southern_market_2000_benchmark`
  (`HACDCPF_ENABLE_MARKET_SCALE_TESTS`, UNIX, licensed Gurobi) declared;
  outer-vs-inner wall-clock provenance note (outer values are retained in
  `output/market-operation/local-2000-reduced/comparison.json`); recovery
  parallelism requires more than one experiment.
- `README.md`: port 8095→8088, round-count wording, new controlled-sources and
  chapter-map section mirroring the OPF manual README conventions.
- `southern_real_time.md`: tap factor restored in the network equation;
  snapshot wording tied to HEAD.
- `yunnan_ancillary_markets.md`: implementation location corrected (rule logic
  lives in the two private headers), mandatory AGC roster for every boundary
  hydro unit documented, the explicit-zero historical-price gap recorded as a
  known limitation (validation lower bound unchanged), intraday replaceable
  field list widened to match `southern_market.cpp:2302-2305`, default
  reservoir/unit AGC grouping documented.
- LaTeX manual aligned with the PF/OPF manual conventions: typo (铜板银),
  test target renamed to `hacdcpf_test_market_simulation`, real-time penalty
  deviation formula gained the reserve-instruction gate
  (`market_simulation.cpp:5611-5619`), HHI zero-clipping noted, AC-validation
  failure consequences stated precisely (`feasible=false`,
  `ac_validation_failed`), per-chapter verification subsections added, and the
  preamble gained solver-family/dispatch and verification-channel overview
  sections (all line references re-verified).
- `tools/market_validation/run_cross_validation.py` gained `--negative-control`
  (corrupts one LMP, expects the oracle to FAIL; exit 0 only when detected),
  making the negative-control claim in `numerical_cross_validation.tex`
  reproducible. Verified: forward oracle passes (max_error=0.0),
  negative control detected (max_error=1.0), registered ctest
  `market_sced_cross_validation` passes; `hacdcpf_test_market_simulation`
  22/845, `test_southern_market` 74/29807, `test_market_forecast` 8/12503 all
  green; `market_manual.tex` compiles with xelatex twice, no errors, no
  overfull boxes.

Follow-up: new manual chapter `chapters/module_io.tex` (各模块输入输出),
organized per public entry point — generic hybrid engine (day-ahead,
real-time, repeated game, offer submission), southern pipeline (day-ahead,
real-time rolling, forecast, operation/recovery), and Yunnan ancillary
(clearing/intraday, settlement/monthly). Each entry documents its input
parameter table, output field table, units, and index spaces with verified
`file:line` citations (including the forecast 7-marginal vs operation
8-multiplier distinction and the 96+2 / 72×5min / 24×4 time grids). Manual
now compiles to 80 pages; `docs/modules/market/README.md` chapter map and
page count updated.

Follow-up: new manual chapter `chapters/yunnan_regulation.tex` (云南调频市场模型)
adds the mathematical-model chapter for the Yunnan frequency-regulation market:
hourly prearrangement clearing (demand, capability bounds, ranking price,
whole-block award), performance indices, the MILP coupling constraints
(secondary reserves, hydro safe-band disjunction, independent storage/load
exclusion), metering/allocation/journal/month-close, workflow timing, a
parameter table, and verification anchors — every formula carries a verified
`file:line` citation to `src/market/yunnan_ancillary.hpp`,
`yunnan_rules_workflow.hpp` and `southern_market.cpp`.

Follow-up: two more manual chapters — `chapters/market_boundary.tex`
(市场边界与边界模拟: the full 15-table southern boundary schema with units and
validation rules, 98-point semantics, forecast copula/AR(1) sampling, and
operation multipliers/peak-valley/carry/recovery) and
`chapters/algorithm_selection.tex` (算法选择与求解策略: SCUC backend dispatch
and fallback chain, pricing-LP routing, deterministic southern pricing with
bit-exact dual comparison, assembly/derive/reuse admission, certified integer
repair, and recovery parallelism). The preamble solver-family subsection is
now a summary cross-referencing the new chapter.

Follow-up: new manual chapter `chapters/rule_implementation_gaps.tex`
(规则—实现差异清单) systematically classifies every rule clause (2.6.3.1–21,
2.6.4–2.6.6) against the production code into consistent / interpretive
(A1–A7) / deviating / not-implemented, with per-clause `file:line` evidence
re-verified against `southern_market.cpp`/`southern_boundary.cpp`; the
abstract now cross-references it. The chapter now also includes a compact
theory-to-code-to-test traceability table covering balance/reserves/start-stop,
storage recursion, SCED→LMP projection, and AC security feedback, and states
explicitly that synthetic test passes are implementation evidence rather than
formal rule certification. `market_manual.tex` was rebuilt twice with XeLaTeX
after this addition (100 pages, no errors).

## Branch-aware dynamic initialization for IBR systems

Fixed a transient-initialization defect where systems with grid-following
inverters could fail the `t=0` AC voltage health check (e.g. `ieee118_acdc`:
`max |V| = 2.51 pu > 2.5 pu`). Root cause: the reduced consistent-initialization
Newton eliminates the network voltage through a free `solveNetwork`, whose map
`y(x)` becomes multi-valued once grid-following inverters inject near-constant
power; the free solve locked onto a spurious high-voltage load-flow branch
(`≈ 1.9 pu` for `ieee118_acdc`). A pre-existing benchmark had masked this by
running the case with `enforce_voltage_health_check = false`.

The fix (only `src/dynamics/DynamicSystem.cpp`) makes `initializeStatesFromPowerFlow`
branch-aware: machine-dominated systems keep the exact-`g` reduced Newton from the
trimmed state unchanged, while systems containing grid-following inverters restart
from the power-flow seed and carry the network voltage as an explicit unknown in a
coupled `(x, y)` Levenberg–Marquardt Newton that solves `[mask f; g] = 0`,
selecting the physical branch, then tighten with the reduced Newton. `ieee118_acdc`
now initializes at a consistent `max|V| ≈ 1.165 pu` and passes the health check;
AC-only and machine/controller cases are numerically unchanged (e.g. the 2-bus
SEXS catalog case retains its `1.29 pu` equilibrium). Verified on macOS Release:
`test_transient_dynamics` 119/120 (the one failure is a pre-existing cross-ABI
exception-wrapping issue in the `GFL DC-link fault control` validation test,
confirmed identical on the baseline via `git stash`), `test_dynamic_model_catalog`
8/8, `test_converter_coordination` 47/47, `test_intelligent_cyber_physical_reliability`
7/7, `test_resilience_assessment` 39/39. See the transient runtime contract for the
initialization semantics.

## Live PowerSimulationsDynamics.jl cross-validation (34/34) and two stiff-machine fixes

The env-gated executable PSD manifest (`tools/psd_validation/`, run with
`HACDCPF_RUN_PSD_COMPARE=1` on tag `[dynamics][benchmark][psd][manifest][external]`)
now clears **all 34 enabled cases against live PowerSimulationsDynamics.jl
`ResidualModel`/`IDA` traces** — 1,069,529 pointwise assertions, every signal gate
`passed=true`. Coverage spans classical/OneDOneQ/Marconato/Anderson–Fouad/GENROU/
GENROE/GENSAL/GENSAE/CSVGN1 machines, AVR (AVRtype1/ESAC1A/SCRX/SEXS), governors
(GAST/TGOV1/HYGOV), PSS (IEEEST/PSS2A/B/C), multi-machine, and the power-electronic
families (GFM VSM/droop/VOC, GFL reduced/Kaura-PLL). The grid-following gate
(`[dynamics][benchmark][psd][gridfollowing]`, test24 ReducedOrderPLL + test51
KauraPLL) also passes live, with pre-step `p_oc` matching PSD to `~1e-6` — a live
confirmation of the branch-aware IBR initialization above.

Closing the last case (`psd-test41-stab1`, a GENROU + SEXS + STAB1 OMIB) required
two further fixes, both pre-existing and independent of the IBR work:

1. **Reachable init algebraic tolerance** (`src/dynamics/DynamicSystem.cpp`,
   `solve_reduced_dynamic_initial_state`). The init algebraic tolerance was
   tightened only to `0.1·dynamic_trim_tol`. A forward-difference reduced Jacobian
   evaluated through an algebraic solve of accuracy `η` has `O(√η)` error, so that
   decade left a `~1e-4` noise floor that stalled the stiff STAB1 hand-off (q-axis
   emf and field residuals locked at `≈ 3.8e-4 > 1e-7`). The tolerance now targets
   `dynamic_trim_tol²`, bounded a few decades under the caller's reachable network
   tolerance so it stays achievable on stiff AC/DC networks (a DCDC-coupled DC bus
   floors near `2e-11`; demanding `1e-14` there previously failed the network
   Newton). stab1 now initializes to `‖dx/dt‖ ≈ 7.7e-8`.
2. **Exciter voltage-reference step** (`src/dynamics/devices/BasicDynamicDevices.cpp`,
   `Exciter::handleEvent`). A `Custom` event carrying `v_ref_pu` was matched against
   the host generator component type, so the exciter never saw its own reference
   step and `computeDerivatives` kept using the captured equilibrium reference —
   field voltage stayed flat. The handler now matches the `Exciter` component type
   and switches to `params_.v_ref_pu` (with `captured_ = false`), so the step is
   applied at `t_event`. stab1's field voltage now tracks PSD (`rms 0.141`,
   `max 0.300`, tol `0.3/0.8`), reproducing the committed baseline exactly; the
   PSS2A/2B/2C v-ref cases (which share this event) remain green.

A third, unrelated pre-existing failure was also closed. The
`GFL DC-link fault control` test's `CHECK_THROWS_WITH` gates on VSC config
validation lost their messages because a `std::invalid_argument` thrown in
`apply_gfl_params` / the grid-following inverter validator was not matched by the
builder's `catch (const std::exception&)` and fell through to the `catch (...)`
"cross-ABI" wrapper. Diagnosis (via `abi::__cxa_current_exception_type`) confirmed
the in-flight type is exactly `std::invalid_argument`, but the `std::logic_error`
family RTTI typeinfo is not matched under this build's Apple `libc++abi`
(pointer-based comparison), while `std::runtime_error` matches. It is not a
static-library duplicate typeinfo and not dependency interposition — a subtle
runtime RTTI-ABI split. As a bounded workaround the two VSC DC-fault-control
config validations (`src/dynamics/DynamicModelBuilder.cpp` and
`src/dynamics/devices/BasicDynamicDevices.cpp`) now throw `std::runtime_error`,
which propagates its message intact. This is a workaround, not a root fix: any
code that catches `std::logic_error`/`std::invalid_argument` specifically remains
affected by the underlying RTTI split.

Verified on macOS Release: full live manifest 34/34 (1,069,529 assertions),
`test_transient_dynamics` 120/120, `test_dynamic_model_catalog` 8/8.

## SCUC solver threads and daily workers

Live-preview follow-up: the only running preview was still PID 9258 on port
61990, started at 20:02 before the rebuilt binary (22:42). Its static directory
already contained the new controls, but unchanged asset URLs and an open page
could retain old JavaScript. Bumped `app.js`/`style.css` URLs to
`20260910-uc-threads` and started the rebuilt server separately on port 8080,
leaving the earlier session intact. `/xjtu/?v=uc-threads` returns the new input
and versioned assets; invalid `uc_solver_threads` on the annual route returns
HTTP 400, confirming the new backend is live. The solver-thread input is in
shared public modeling, not the annual daily-worker input.

Fixed the production GUI's ambiguous thread control. A new shared solver-thread
input (`tspfSolverThreads`) is editable for Auto/Native/Gurobi even with annual
SCUC/SCED; the existing two controls are labeled daily-worker counts. Unsupported
HiGHS/SCIP adapters disable custom solver counts explicitly and preserve the UI
value for switching back. SCUC's annual inter-day coupling stays intact.

Both production routes consume `uc_solver_threads` (integer 0–256) and reject
invalid/unsupported overrides with HTTP 400. Native uses `BCOptions.num_threads`;
Gurobi sets only `Threads`, leaving previous tolerances/time limits intact.
Auto/Gurobi with a nonzero override skip HiGHS fallback and retain that limit
when falling back to Native. Defaults retain the existing backend order.
The result distinguishes requested count, configured cap (nullable), actual UC
backend and day-worker execution; it does not claim measured active CPU workers.
See the GUI runtime contract (including the field ledger) and the time-series
manual's GUI-thread subsection for scope, rationale and numerical oracle.

MIPSolvers required a narrow thread-only adapter setter. It is locally committed
on `codex/uc-solver-threads` at `80ddb8b40540ef06c0cb5ce282dbc2f79e6114c5`
(parent `5eac6be`); no remote push. `cmake/Dependencies.cmake` now pins this local
revision because the previous pin lacks that interface. Its worktree is clean.
Release's dirty-dependency guard remains enabled (`HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK=OFF`);
initial attempts with dirty sources failed that guard and were resolved by the
local dependency commit, not by disabling Release reproducibility checks.

Verified on macOS arm64 with `macos-release`:

- Rebuilt `run_gui_server` and `test_uc_solver_threads` successfully.
- `test_uc_solver_threads '[uc_threads]' -s`: 3 cases / 29 assertions pass.
  Native 1/2-thread and real Gurobi 2-thread solves match the predeclared objective
  2200 within 1e-5; period MW balances within 1e-5. Annual coupled UC reports cap
  2 independently of a requested daily-worker count of 7. No speedup claim.
- Running the Auto/Gurobi case with an intentionally missing `GRB_LICENSE_FILE`
  forces actual Native fallback: 8 assertions pass, retaining cap 2 and the same
  objective. Logs: `/tmp/hysim-uc-threads-test.log` and
  `/tmp/hysim-uc-threads-fallback.log`.
- Registered `annual_ui_reconcile_e2e`: 33 checks pass, including SCUC/SCED input
  editing, restored values, 390px mobile editing, independent day/solver counts,
  invalid requests on both routes, and real IEEE14 AC/DC two-step UC via the GUI
  with Native cap 2 and feasible scheduling. Result columns do not overlap.
  Annual request serialization is intercepted; annual cap propagation is tested
  with the short C++ fixture, not a fresh full-year performance run.
- Focused CTest run covering these 3 C++ cases plus annual UI, scale GUI and local
  busbar E2E: 6/6 pass, 32.93 s. After the final result-column CSS fix, annual UI
  passes again in 2.16 s (33 checks). JS syntax and both Git whitespace checks pass.
- Browser screenshots reviewed: `output/gui-uc-threads/annual-controls.png` and
  `solver-result.png`. The result screenshot retains deliberate intercepted-run
  error notifications from the request-contract tests; the subsequent real UC
  run succeeds. CUA manual browser control remains unavailable (auth token).

No full C++/MIPSolvers suite or yearly parallel-performance benchmark was run.
HiGHS/SCIP custom solver-thread interfaces and per-step OPF threading remain
outside this change. The GUI setting controls a limit, not guaranteed occupancy
or linear speedup; parallel daily workers and solver workers can multiply.

## Scale-first busbar GUI

Implemented the first scale-first delivery in the working tree based on
`b7ac722b`: existing WebGL overview → bounded read-only local busbar sheet.
Default/max 20/80 buses, 160 rendered links, 12 taps per bus, 20 connection rows
per page; explicit off-sheet/undrawn counts, domain-qualified navigation and
history, keyboard isolation, fixed readable labels and mobile scrolling.
Separate bus rows replace the concentric local node layout. Main SVG legacy/ELK
layout reserves the configured adaptive span; rotated tap projection and live
label/handle re-span are corrected. Contract and identity/field ledger:
`docs/developer/gui_canvas_runtime.md` and its Chinese help translation.
The planning document records the pre-implementation model and bounds.

Verification on macOS arm64, Node v26.7.0, existing `macos-release` server:

- `cmake --preset macos-release` completed and registered both new tests.
  Latest `ctest --test-dir build/macos-release --output-on-failure -R
  '^(local_bus_diagram_(test|e2e)|network_overview_e2e|gui_scale_features_e2e)$'`:
  4/4 passed, 28.71 s, including the expanded IEEE118 browser regression.
- Pure 5,000-bus hub: 80 local buses / 12 paths / **0 bus overlaps**,
  graph + extraction + layout 22.355 ms; 5,000-bus mesh: 39 local buses /
  59 paths / **0 bus overlaps**, 43.636 ms. Predeclared ceilings B≤80,
  L≤160, no overlaps, <500 ms all pass; these are local fixtures, not an SLA.
  An 80-bus complete graph reaches the 160-edge ceiling without overlap.
- Browser synthetic 5,001 AC/DC buses: 20/80 limits, paged off-sheet navigation,
  back, AC/DC ID 1 separation and JSON preservation pass. Desktop 1440×1000
  and mobile 390×844 stay within the viewport with 13px identity labels.
  Actual case2869pegase local view also passes geometry/model preservation.
- IEEE14 AC/DC main editor: both legacy and ELK layouts have zero tested bus
  overlaps; resize labels/handles align; all three links on the rotated test
  bus remain on its axis. Delete inside the local sheet cannot edit the model,
  Escape closes, and replacing the system clears the previous local view.
- IEEE118 extension: MATPOWER `case118.m` (118 AC) and built-in `ieee118_acdc`
  (118 AC + 6 DC) both have **0 tested bus overlaps** under legacy and ELK.
  Local neighborhoods centered on highest-degree AC 49 pass 20/40/80 limits:
  actual bus counts 20/40/59 and 20/40/68 at four hops, respectively, with zero
  overlaps. Desktop 1440×1000 and mobile 390/320×844 preserve 13px labels and fit
  the viewport. Selection, connection navigation, Back and full serialized JSON
  preservation pass. This caught and fixed local selection panning the main SVG
  viewport (`_canvas.viewBox`); ordinary global navigation retains its pan behavior.
  Hybrid AC 49 has 13 connections: the 12-tap cap omits one drawn link while
  retaining its inspector record. Main fit-to-network labels remain small;
  these checks establish bus separation, not readability of the full-network fit.
  Screenshot inspection confirms local readability and intentional independent
  scrolling. Reproduce with `node tests/e2e/local_bus_diagram_e2e.mjs --server
  build/macos-release/run_gui_server --data-dir data --output-dir
  output/gui-busbar-ieee118`; screenshots and `verification.json` are in that
  directory. Current in-app manual reinspection was unavailable because the
  browser-control service returned an unavailable auth token; Playwright browser
  execution and screenshot inspection completed.
- Existing `network_overview_e2e.mjs`, `gui_scale_features_e2e.mjs` and
  `busbar_length_e2e.mjs` pass (the latter 7/7 checks). The scale suite covers
  large-network PF/OPF, table editing, imports and normal-editor return.
  Updated its geometry selectors for busbars; repaired the overview test's
  pre-existing incomplete Plotly mock by adding `purge`.
- Manual in-app browser inspection loaded case2869pegase and navigated
  AC 100 → AC 1203: separated bars, selection, five off-sheet connections and
  three independent transformer records to AC 556 are visible. Screenshots:
  `output/gui-busbar/` and `build/macos-release/local-bus-diagram-e2e/`.

No C++ source or API schema changed for the busbar GUI. The GUI regressions above
reused the existing server. Subsequently rebuilt `run_gui_server` successfully
with `cmake --build build/macos-release --target run_gui_server -j4` at the user's
request. The rebuilt executable passed a temporary localhost startup smoke:
`/api/cases`, `/xjtu/` and `/xjtu/js/core/local_bus_diagram.js` returned HTTP 200;
the temporary process was stopped afterward. Build emitted compiler warnings
(including OpenXLSX deleted default operations and DynamicSystem class/struct
declarations), but no build errors. No complete C++ suite was run.
Configure warns that local MIPSolvers HEAD
`5eac6be` differs from pin `a39812a`, and that its prebuilt dependency manifest
does not match the current toolchain; configuration used vendored sources.
No dependency pin was changed. Remaining scope: local device glyphs/editing,
continuous zoom morphing, substation grouping, and full detailed sheets are
not implemented by this increment. Wire crossings remain possible; main-editor
manual/locked overlaps or cap increases after layout can require re-layout.

## Strict Hydro Label Convergence and Prospective Holdout (running)

Theory §14 / intelligent simulation §12: four old training-only representative
pairs at gaps 1e−5 and 1e−6 completed: 16 evaluations / 112 LMP days audited,
maximum paired-gain difference 5.128e−11 MWh, constituent curtailment difference
0.263208 MWh. The ≤5 MWh label stability gate passes; one pair still has 0.186468
MWh diagnostic flow violations. Wall 820.943 s vs predicted ≤1200 s.
A new frozen design has 4 training and 2 new test roots, 4 shape/condition variants,
2 actions and 2 gap levels (96 evaluations). Narrower line factors [0.8,1];
input-designed 150 MWh benefits plus hinge-switch/reverse cases. Training completed: 64 evaluations / 448 LMP days, 16 stable pairs; 14 pairs pass
the diagnostic flow/balance screen, 2 retain 0.032925 MWh overflow per candidate.
Eight models were frozen before prospective testing. Held-out primary MAE/max
0.069642/0.246703 MWh, 0/8 high-benefit misses, maximum local inference 51.429 ms;
all 8 test pairs are stable and pass the diagnostic screen. Original gates pass,
but every actual nonzero gain is approximately 150 MWh: this is a weak constant-
response test, not production certification. A new frozen-model challenge with
3 new roots and expected hinge gains 110/90/−150 MWh is running (48 Oracles),
without refitting or changing the original results. Theory §15 / module §13.

Forty-two hydro Python tests pass. New strict runner, model freeze/inference
and independent validation preserve original v3 outputs and reject held-out
execution before training freeze. Existing Release binary reused, no C++ rebuild.
Four isolated servers × 2 threads on 16 logical cores; predicted convergence
wall ≤1200 s, later batch ≤6500 s. Artifacts: hydro-strict-v4.

## Equal-energy Intraday / Thermal-condition Supplement

Completed hydro-temporal-v3: 72 full-price weekly evaluations / 504 main LMP
days, plus 4 precision reruns / 28 LMP days. Six independent exogenous roots ×
2 load shapes × 2 thermal operating packages × 3 actions; 4 training roots and
2 fixed test roots, with all siblings kept together. Fixed station weekly energy,
node daily load energy and terminal water contract. This is a sequential rolling
linear diagnostic Oracle, not a jointly optimal week or AC/N-1 certification.

Research accuracy gates fail: on 16 nonzero held-out actions, primary temporal_gp
MAE/max = 64.496/214.473 MWh versus daily_gp 117.537/250.570. Pre-test secondary
network_gp gives 57.975/205.201; complete 8-day context gives 66.916/234.905.
All models were frozen before test-label reads; no post-test refitting. No true
positive benefit reaches 100 MWh (maximum 87.226), so no-miss performance is
unverified. Full local validated request P95 is 19.51/22.39/24.85 ms for temporal,
network and complete models; the 1 s local speed gate passes, HTTP SLA untested.

Main new finding: tightening gap on one training baseline/action pair changes
gain from −481.875 (1e−2) to −182.451 (1e−4) to −180.164 MWh (1e−5).
Adjacent changes are 299.424 and 2.287 MWh; only this pair is locally stable at
the last two levels. All original labels/models remain frozen for diagnostics.
Do not deploy 1%-gap models or interpret loose-solution violations as proof of
input infeasibility. Thirty-nine main solutions, including 12/24 test candidates,
have diagnostic flow violations; all 3 positive test actions fail that screen.
Four constrained test families have no passing candidate, including baseline.

Independent audit verifies 72 raw weeks, 504 LMP days and 33264 generator state
links, plus both precision pairs. Main maximum energy/water residuals are
3.121e−8 MWh / 5.344e−5 m³, with no stage limit reached. Raw storage charging is
negative injection; independent accounting was corrected to −(charge + discharge).
Thirty hydro Python tests and 18 label/stress/identification tests pass; existing
Release binary reused, no C++ rebuild or full C++ regression. Figures pass geometry
and visual review. Main call wall time 5271.19 s versus predicted 6000 s.

Next design must establish label convergence across representative pairs and add
feasible high-benefit cases on new held-out roots. The current ±180 MWh surplus
region with ±300 MWh actions bounds positive daily-hinge benefit at 60 MWh;
it was unsuitable for testing ≥100 MWh recall. More input factors alone did not
improve this small-cohort GP; theory §13 records the mismatch and limits.
No production admission or adequate-total-sample claim. Theory §12–13 / module §11;
artifacts under output/market-intelligence/hydro-temporal-v3 (provenance.json),
hydro-temporal-analysis-v3 (summary.md), hydro-complete-context-analysis-v1,
and hydro-label-precision-v1/v2 (v2/convergence.json compares adjacent gaps).

## Hydro Transition Surrogate Iteration

Completed hydro-transition-v2: 36/36 full-price weekly Oracles / 252 main LMP
days, 8 training and 4 fixed independent test families. Four/eight-family models
were saved and hashed before the analysis read test labels; no post-test tuning.
Primary physics+GP test MAE 6.568845 MWh vs same-cohort context trees 216.143499
(96.9609% lower); max error 26.763566, meaningful misses 0/6, candidate regret 0
on four test weeks. Research gate passed; no production admission. Pure physical
formula is near roundoff on this tiny test, but one training action has a
149.155 MWh residual and opposite sign due to omitted realized thermal energy.
Independent full daily accounting verifies this. The v3 input audit subsequently
identified the exact singleton import constraint at bus116 / branch183 / unit54
(theory §12.4). Keep this counterexample; do not infer full-domain exactness.

Frozen model artifacts and local predict_hydro.py enforce original base, operation
contract, quotas, bids, input ranges and station1 day2/day5 action. Nine of twelve
test candidates lie outside the finite training-coordinate box; flags and Oracle
confirmation remain mandatory. GP standard deviation is uncalibrated. No new
HTTP/GUI, automatic background learning or multi-station optimizer deployment.

Oracle wall 1957.343 s vs predicted 2200 s, mean 108.511 worker-s; 8-family fit
0.035832 s. Cached prediction median 0.021958 ms, full features + prediction
2.545000 ms: local microbenchmarks, not service SLA or solver acceleration.
19 hydro + 18 existing Python tests pass. Independent validation checks 36 raw
weeks / 96 predictions, water residual <=9.239e-7 m³, renewable residual
<=2.058e-11 MWh, metric difference <=2.842e-14; saved-model inference matches
12 held-out candidates. Figures pass geometry/perceptual checks. Original Release
binary reused without C++ changes/rebuild. Theory §10–11 / module record §10;
artifacts hydro-transition-v2 and hydro-transition-analysis-v2.

## Hydro Allocation Research Scope

User confirmed the primary task: hold each hydro station's weekly energy fixed
and optimize allocation across seven days to improve wind/solar utilization.
Online analysis is required; offline learning/labeling may take several hours.
Design is in `docs/theory/hydro_renewable_allocation.md`. Existing code admits
reservoir daily min/max MWh overrides and carries hydrological states across
days. Station-to-generator-to-reservoir identity must be explicitly mapped;
shared-reservoir groups are not automatically actual station identities.

The hydro-specific pipeline completed 30 full-price weekly evaluations / 210 main
LMP days across 6 new external families and 5 station1 day2/day5 actions each.
All pass generic pricing and independent hydro audits. Explicit synthetic station
mapping, 7-day exact energy overrides, common terminal levels/release tails and
full input/reference/action identity are implemented via existing market_operation
API. No C++ solver changes or online GUI/API deployment.

Only family2 has significant gain: shifting 300 MWh from day5 to day2 reduces
solar curtailment by 300 MWh (+0.301496 percentage points), with fixed realized
weekly hydro/terminal water and zero diagnostic deficit/overload. Five families
have zero gain on all actions. 3-fold 2/3/4-family development training yields
Ridge MAE 37.500/37.500/69.453 MWh and trees 37.500/37.500/61.011, versus no-gain
37.500 throughout; all maximum errors are 300 MWh. No generalization admission,
formal sample-size extrapolation, adaptive new cohort or production model.
Next sampling must address response/curtailment-transition coverage; mean-error
alone misses the sole useful family. See theory §9 and module record §9.

Oracle call wall totals 1799.209 s vs predicted 1800 s; worker mean 118.490 s,
P90 168.238 s, fitting 0.295 s. 3 h capacity examples are not approved exact
budgets or enough-sample guarantees. 10 hydro + 18 existing Python tests pass;
existing Release SI/D-day hydro test passes 13 assertions. No C++ rebuild.
Independent audit: 30 raw weeks, 216 predictions, water residual <=1.289e-6 m³,
renewable residual <=1.455e-11 MWh, metric reconstruction difference <=1.421e-14.
Plots pass geometric/perceptual review. Artifacts: hydro-pilot-v1 and
hydro-analysis-v1 under output/market-intelligence; complete provenance recorded.

## Market Identification Preflight

The frozen `preflight-v3` experiment is complete. User selected error-versus-cost
curves before setting engineering tolerances: 12 Latin-hypercube input families
crossed with two daily orders and two line limits (10% tightening), plus two
identical-input replays. All 50 weekly evaluations / 350 main LMP stages pass;
48 unique weeks are grouped in 12 families, not 50 independent observations.
All paired exogenous/reference trajectories and initial states match, and
applied directional-limit reconstruction error is zero. Theory and full results:
`docs/theory/market_surrogate_sampling_design.md` section 9 and
`docs/modules/market/intelligent_simulation.md` section 8.

Input low5 plus intercept has rank 6/6, condition number 2.022. Reversing the
first seven days preserves low5 and orderless42 exactly but changes output:
empirical same-coordinate average MAE floors are 2621.765 MWh for deficit,
7914.845 MWh for active-flow overload, and 615.679 currency/MWh for mean price.
Two identical-input replays have zero difference on all 10 targets; this is
not a global numerical-stability certificate. Temporal17 preserves some order
and helps some metrics, but is not established as sufficient or universally best.

Nested 4/6/8-family development training uses 16/24/32 weekly evaluations, costing
27.988/44.441/56.758 average Oracle worker-minutes per fold. Full56 trees' spike
fraction MAE falls 22.657/16.097/12.264 percentage points, but deficit MAE is
94985.4/118133.7/83589.2 MWh. Action-delta deficit MAE at 8 families is 4407.793
MWh versus 133.945 MWh for predicting no change. 47/48 nonrepeat evaluations
have spike duration 168 h, so its low error reflects saturation. No deployment,
engineering gate, final blind test, adaptive sampling or boundary optimization
is claimed. Do not compare v2/v3 MAEs directly: their evaluation distributions
differ. Learning curves have only three development sizes; no sample-size
extrapolation or convergence law is fitted.

Oracle wall is 2712.082 s (45m12s) versus predicted 1500 s (+80.8%); curve fitting
17.818 s versus predicted <120 s. Cost audit finds average nonrepeat worker
106.422 s/week vs v2's 66.866; SCUC/SCED/LMP mean solve times are
44.977/40.264/14.247 s vs 26.944/21.884/11.119. Worker minus operation runtime
is about 4 s in both cohorts. Existing input/label checks and independent metric
reconstruction pass; the old cost-distribution assumption did not transfer to
new inputs. This is descriptive cohort evidence, not a paired solver speed test.

Tools in `tools/market_intelligence/`: `run_identification_oracle.py`,
`analyze_identification.py`, `validate_identification_report.py`,
`audit_identification_cost.py` and `plot_identification.py`. Isolated artifacts:
`output/market-intelligence/identification-v3/` and `identification-analysis-v3/`.
Independent audit reconstructed 720 metric groups from 11520 predictions to
5.821e-11 (tolerance 1e-8), including split and baseline checks and 120 compression
bounds. Seven identification + seven label + four stress tests pass. Both
PNG/PDF figures (10 panels) pass geometric and visual checks; doc anchors have
zero failures. No C++ production changes, rebuild or full C++ suite run.

## Market Stress Oracle and Second Surrogate

Follow-up sampling theory is documented in
`docs/theory/market_surrogate_sampling_design.md`: conditional simulation versus
boundary-action identification, parametric-programming regions, paired common
scenarios, learning-curve/cost allocation, transition learning and independent
family-grouped holdouts. It explicitly distinguishes target and adaptive
sampling distributions and derives zero-miss binomial sample requirements.
The preflight subset is now completed as described above; adaptive acquisition,
GP-based allocation and final blind-test admission remain theoretical proposals.
There is no change to the frozen v2 evaluation. Bibliographic metadata for the
three linked papers was checked against Crossref; no full-text systematic
literature review is claimed.

The second offline experiment is complete: 32 new synthetic IEEE118 weeks
(seed 20261000–20261031), 224 valid and dual-consistent main LMP stages, zero
ineligible weeks. Ordinary/scarcity/congestion/surplus inputs now yield
nonzero deficit, active-flow overload and renewable curtailment labels.
Training uses 18 old plus 24 new weeks; 8 new holdouts, two per stratum, remain
frozen. Old test weeks were not used for selection. Contract, design and
negative findings: `docs/modules/market/intelligent_simulation.md` sections 6–7.

All 10 regressors pass the original global-mean 10% MAE gate, but a posthoc
training-stratum-mean baseline beats the model on 8/10 targets. The other two
improve only 4.9%/8.1%. Thus this is not admitted for Bayesian/CMA-ES boundary
optimization. Overall spike-fraction MAE is 0.030845 (3.084 percentage points),
duration MAE 4.450 h. On the two new ordinary weeks, average-price MAE worsens
from frozen v1's 10.081 to 25.347 currency/MWh, and curtailment MAE from 0.212
to 159.898 MWh. Three event classifiers make no errors on these eight weeks,
but have only 2/4/4 positives and no reliable or calibrated probability claim.
No test-driven retraining or replacement of the original gate was performed.

Oracle wall 1088.214 s versus predicted 811 s; scarcity/congestion weeks are
slower than ordinary weeks. Training 16.880 s, warm batched regression
prediction 0.009370 s/week versus predicted <0.1 s (no I/O, classifier or
actual clearing). Artifacts: `output/market-intelligence/stress-full-v2/` and
`output/market-intelligence/price-surrogate-v2/`. Existing Release binary hash
is unchanged from v1 below; no rebuild. Per-run HEAD changed from 5041bcdb
to ef54df20 during execution; final source and artifact hashes are recorded
in the stress Oracle directory's `provenance.json` without asserting a clean
source-to-binary build.

7 label tests and 4 stress-feature/split/support tests pass. Inference reload
matches frozen predictions; wrong case, missing feature and inputs outside
the union of sampled strata are rejected. Posthoc baseline audit reproduces
the original diagnostic to 1e-8 with unchanged frozen-artifact hashes.
Ridge candidates emitted NumPy/scikit-learn matmul numerical warnings:
36 training-fold fits agree with independent augmented least squares to
2.765e-10, CV score differences <=1.001e-11 (tolerance 1e-5). All selected
regressors are Extra-Trees. Warning root cause remains unproven, not fixed.
No C++ or production GUI changes or full C++ suite run in this experiment.

## Market Price Oracle and First Surrogate

Offline tools in `tools/market_intelligence/` now generate isolated weekly
Oracle jobs, audit labels and train a fixed-boundary price surrogate. Protocol:
`docs/modules/market/intelligent_simulation.md`. No production solver or GUI
changes were made for this experiment. Existing local Release binary
`build/macos-release/tests/run_gui_server` SHA256
`795bcc72bbed8697d9b0cbd1f5920a2f99eab6462f3622dd76610f607d460c30`
was used, not rebuilt; workspace/source/dependency identity is recorded in
`output/market-intelligence/provenance.json`.

24 independently seeded IEEE118 weeks completed in 608.433 s with two isolated
workers, Gurobi 2 threads per SCUC, gap 0.01 and 120 s per stage. All 168 main
LMP stages passed price validity and repeated-dual consistency, yielding
1,903,104 executed node-time prices. The separate original-input calibration
week with `explain=true/recovery_pricing=full` completed in 178.430 s: 7 main
and all 42 counterfactual price stages passed. The fixed spike threshold is
59.61200124564096 currency/MWh (calibration node-time P95).

Schema-v3 labels independently check energy, directional overload, renewable
accounting, component IDs, 96-point execution and cross-day carry. Earlier
v1/v2 labels are superseded: nodes were misread as an object, repeated weeks
counted by path, and lookahead-inclusive objectives mislabeled weekly cost.
The new cost field is explicitly `scuc_98point_objective_sum` and not trained.
7 Python analytic/mutation/identity tests pass via
`python3 -m unittest discover -s tests -p test_market_intelligence.py -v`.

Model uses 18 training weeks and 6 untouched holdout weeks, per-target 3-fold
training CV over mean/Ridge/Extra-Trees. Price mean/P95/P99/CVaR99 test MAEs
are 11.078/50.912/55.774/48.190 currency/MWh; relative MAE reductions vs the
training-mean baseline are 26.2/37.6/32.9/26.2%. These pass the predeclared
10% improvement gate, but P99 R2 is only 0.006 and CVaR99 R2 is -0.262.
Spike fraction worsens by 14.4%; duration improves only 8.0%, failing the gate.
Load deficit and line overload have no training variation; renewable targets
select the mean baseline and fail the improvement gate. No safety classifier
or calibrated uncertainty claim is supported. No post-test tuning was done.

Training took 3.987 s; warm batched prediction of all fitted targets took
0.001646 s/week excluding I/O. Models, labels and heldout predictions are in
`output/market-intelligence/price-surrogate-v1/`; `report.md/report.json`
contain complete results. Reloaded predictions exactly match the heldout
report; inference rejects a changed case hash and out-of-support features.
Evidence is `inference-validation.json` in the model directory.
Scope is fixed synthetic boundary/initial state,
not a learned boundary policy, realized-market forecast or AC/N-1 certificate.
No new C++ suite was run for these Python-only changes.

## Market Module Documentation Resync

A full source-to-document audit of `docs/modules/market/` against
`src/market/` (HEAD `5041bcdb`, source unchanged since `ccfe0bce`) is complete.
All ten module documents plus the four LaTeX chapters were checked claim by
claim; corrections cover stale line anchors (e.g. the controllable-load
compensation model is at `southern_market.cpp:815-835`, not 505), the LMP
weight-division formula in `southern_execution.tex`, wrong pricing solver
anchors in `market_manual.tex`, stale real-time timings and exited-server
claims, the Gurobi Seed/Method pricing strategy description, SCED reuse and
certified-repair admission gates, HiGHS LMP time-limit scope, the
`recovery_execution` dual-worker gate, and the AEMO counterexample field names
(RAISEREGACTUALAVAILABILITY vs RAISEREGAVAILABILITY). Two unsourced assertion
counts in the 2000-bus compact-formulation section are now marked as having no
independent source. The module README gains a generic AC/DC hybrid engine
section (pipeline, `ac-dc-linear-v1:dc-voltage+bidirectional-converters` scope,
explicit unsupported-asset rejection, 22 cases/845 assertions baseline), and
`docs/README.md` indexes `reference/market_simulation_runtime.md`. The stale
`run_day_ahead_market` header comment ("hybrid co-optimisation is a later
extension") was corrected in `include/hacdcpf/market/market_simulation.hpp`.
`market_manual.pdf` rebuilt with xelatex: 62 pages, zero errors (page count
unchanged). Rebuilt `test_southern_market` passes 74 cases/29807 assertions and
`test_market_simulation` 22/845 on current source.

Full `ctest -R 'market|southern'` on the pre-existing macos-release build:
26/29 passed. After rebuilding `run_gui_server`, `market_ieee118_e2e`,
`market_ieee118_row_presolve_e2e` and `market_ieee118_native_e2e` still failed
identically at `tests/e2e/market_ieee118_e2e.mjs:170`: the assertion expected
2 wind units in the legacy IEEE118 picker, but `make_southern_market_ieee118`
has added exactly 1 wind + 1 solar + 4 hydro since `308ccc57` (contract:
southern_execution_contract.md, intermediate-scale fixture, "adds 1 wind").
The test expectation `mixed?6:2` introduced in `ccfe0bce` was an authoring
error (mixed variant with 6 wind always passed); corrected to `mixed?6:1` in
both the option-count and CSV-row assertions. Rerun after the one-line test
fix: all three pass (48.7/48.7/82.8 s). No solver, boundary or GUI production
code was involved.


## PF Presentation Profiling

The next measurement pass is complete without production-code changes.
`tools/pf_presentation_profile.py` generates an instrumented server copy and
uses the previous incremental Release overlays. Final paired alternating runs:
four MATPOWER cases x five fresh servers x baseline/coarse/scan modes = 60
converged responses with exact full non-timing body hashes. Large-case median
wall overhead stays below 0.6% (predeclared limit 5%); stage gap <=0.024751 ms.

case9241 coarse medians: HTTP 922.896 ms; component display JSON 122.093 ms,
attribution JSON 158.251 ms, diagnostic scan/rows 131.434 ms, final JSON dump
137.120 ms. Two diagnostic lookups consume 45.055 + 46.241 ms; repeat projection
only 2.172 ms. 35,396 merged rows leave 119,161 prebuilt metric items unused.
Full output is 49.71 MB, with component results 33.53 MB. Priority is diagnostic
indexing and selective attribution JSON construction; no savings claim yet.

Timing interpretation corrected: `add_geo_data` destruction costs 62.800 ms
outside the existing presentation timer, so existing `solve_ms` includes it.
Handler cleanup costs another 69.536 ms; outside-handler remainder is 12.135 ms,
not all network time. Fresh-server first requests are not comparable as a
code regression to prior same-session 836 ms. OPF/TSPF and rich 3W/converter
performance remain unmeasured. No numerical model, tolerance, API or GUI changed.

Detailed stage/scan/byte tables, boundaries and follow-up priorities:
`docs/testing/module_code_audit.md`; tool contract:
`docs/developer/projection_and_results.md`; two-pass evidence:
`output/pf-presentation-profile/` (final results under `final/`). Python syntax,
exact response comparisons and source-anchor checks pass. No new full CTest,
sanitizer, clean build or browser performance run. Existing previews were
preserved and temporary measurement servers stopped. Final dependency check
observes externally updated clean HEAD `5eac6be` (pin still `a39812aa`), replacing
the prior dirty `e6c932e5` state; this pass made no dependency edits or commits.
The measured server retains old static archives and is not validation of a
fresh `5eac6be` dependency build.

## Attribution Performance Follow-Up

AUD-097 replaces repeated linear result-attribution lookups with call-local,
domain-aware voltage/position indexes and lazy component-family row indexes.
Output order, canonical OPF positions, fallback and recovery semantics remain.
No cross-call cache or solver tolerance change. Source reports still lack
domain fields and retain existing all-match provenance behavior.

Three fresh-process samples per mode/fixture, with 100-call batches for small
cases: synthetic 10000 attribution median 0.524050 -> 0.007305 s (71.74x),
case9241pegase 0.184947 -> 0.005857 s (31.58x). All full-field output hashes
match. Initial eager indexing added about 10 us on case14; lazy indexing removes
the regression (14.065 -> 13.748 us). No memory-reduction claim. These timings
exclude projection, PF and serialization. Full HTTP case9241 improves
0.942449 -> 0.836189 s (11.27%), with exact voltage/component-result hashes.
ACTIVSg2000 has no material HTTP gain. Remaining measured presentation time
is 511 ms versus the then-reported 128 ms analysis remainder, and the full
payload is 49.7 MB. The subsequent profiling section above resolves diagnostic,
JSON and destruction costs; the 128 ms is not pure numerical solve time.

Incremental Release: 99 cases / 669 assertions across attribution, component math,
carbon storage, SPPT and graph; attribution-unit ASan+UBSan: 4/96
(`detect_leaks=0`, other archives Release); current relinked server GUI API: 82/82.
Dependency mismatch/dirty tree and guards remain unchanged. This is not a clean
build, full CTest or whole-library sanitizer run. Reproduction tool:
`tools/attribution_performance_benchmark.py`; provenance and measurements:
`output/attribution-performance/`. Contract and full ledger:
`docs/developer/projection_and_results.md`, `docs/testing/module_code_audit.md`.

Current preview: http://127.0.0.1:53731/xjtu/ (PID 77754); previous services
retained. Binary SHA256
`79acc4a603abacc24a6028378f22843bc04d3264eb6ec384c46570be8696adff`;
launch provenance: `output/attribution-performance/preview.json`.
Chrome MATPOWER case14 import/run smoke converges in 3 iterations, residual
1.3159e-10, with topology and voltage labels rendered. This is a basic desktop
smoke, not exhaustive GUI acceptance: the imported case's load rows show
"missing post-PF component row", and Canvas uses 110 kV labels while the result
table reports zero/unknown base kV. These presentation observations remain
untriaged and are not covered by the exact backend-output comparison.
Documentation anchor check passes (1847 symbols, 719 paths, zero failures).

## Repository Review: Correctness Fixes and Measured Optimizations

AUD-089--096 from the risk-directed review of `48e0cf62` are fixed within the
focused validation scope. All 45 analysis handlers own their busy lease;
13 PF cache sites reject publication from replaced immutable snapshots.
User/node GEC rejects incomplete annual evidence and nonfinite metrics;
distributed-slack participants use stable in-service AC IDs; voltage recovery
keeps missing same-ID AC/DC values unavailable. Hourly-carbon tables now pad
once, and distributed-slack mismatch traverses sparse Ybus.

Incremental Release: 148 cases / 1113 assertions pass across six rebuilt test
executables. The three modified numerical translation units plus new regression
test pass ASan+UBSan, 6 cases / 228 assertions (`detect_leaks=0`, container checks
enabled). The rebuilt server passes GUI API 82/82, both session-integrity
concurrency scenarios, and market-operation desktop/mobile GUI/API E2E.
Other archives remain prebuilt; this is not full CTest, a clean build, or a
whole-library sanitizer run. Server compilation retains 19 existing warnings.

Three sequential runs per fixture/mode, 84 samples including controls:
frozen verified carbon materialization (200 buses/200 loads/4000 steps) median
2.899230 -> 0.036692 s, 79.0x; AC mismatch (10000 buses) 0.525210 -> 0.0001293 s,
process peak 1630.06 -> 30.33 MB. Predeclared padding/time/RSS thresholds pass.
Outputs match exactly; independent injection oracle error <=4.10e-15;
full small-system redispatch and binding-limit outcomes match. These are
isolated kernel/materialization results, not full annual/PF speedups.
Reproducible tool: `tools/review_performance_benchmark.py`; evidence:
`output/code-optimization/` and `docs/testing/module_code_audit.md`.

Preview: http://127.0.0.1:52180/xjtu/ (PID 47682; separate new server).
Chrome interaction loads case14 and runs distributed slack: converged in
15 iterations, residual 1.3157e-10, frontend/backend ready. Existing services
were not replaced. Binary SHA256
`3764249b13e88a7179d7d46d4501cd7682d4ae8816edf0c09b26300013cc562e`;
launch details are in `output/code-optimization/preview.json`.

Main worktree was initially clean; MIPSolvers has five modified files and HEAD
`e6c932e5` differs from recorded pin `a39812aa`. No dependency files or guards
were changed. An initial diagnostic executable could not unwind the optional
sparse-ID exception; that harness issue remains unassigned, and the successful
non-throwing permutation probe is the identity finding's executable evidence.

## Southern Market Execution and Boundary GUI

Local/online documentation synchronization: the user tutorial now explains chronological
SCED end-of-day carry, daily authored SOC targets, failed-day behavior and the distinction
between rolling daily optimization and a joint weekly optimum. The execution contract
maps state fields/units/indices to apply/carry_from/step_market_operation. The tutorial's
outdated single-thread-only pricing statement now reflects the large Gurobi8-thread policy.
The existing market LaTeX execution chapter and62-page PDF include carry equations,
SCED-to-LMP reuse, full-dual gates and the previously measured five-run acceptance.
No numerical code or benchmark result was changed in this documentation pass.

Online help manifest points to docs/README.md as the canonical root and directly indexes
the execution contract and performance record, with carry/SOC/performance search tags.
43 manifest paths and84 relative Markdown links in the five market/navigation documents
pass; two stale LaTeX-guide references now target the existing manual documentation.
Port8107 serves those Markdown sources byte-for-byte from docs/. Playwright verifies
search/internal navigation and1440x1000/390x844 views without browser errors; PDF new
pages rendered and visually checked. Evidence: output/market-docs-sync/. Port8097 was
not listening during this check; no old process or market task was replaced. Existing
open help tabs may cache prior content until the page is refreshed.

SCED-to-LMP ordered-matrix reuse is now validated on the unchanged2000/1320-generator
fixture. Serial browser protocol: five fresh-server first runs plus five same-session
repeats per version. Baseline maxima61.292012/61.407915s; candidate maxima
58.515226/58.570428s (means58.251299/58.296980s). All ten candidate samples are
below60s; no hard guarantee for arbitrary boundaries or background load. LMP assembly
means3.365804/3.336026s become0.590407/0.594450s. Shared coefficients/order are
preserved by row/column selection, original costs/bounds recomputed, independent
verify oracle retained. No large cross-run cache or solver ordering changes this turn.

All20 complete outputs match scale-final5 except timing/assembly diagnostics; full-size
verify matches6 matrices and all metadata. Cross-process original/derived LPs match
all3247054 row duals bitwise (SHA2568ce718243b40c07274a276dc6e7dce446dcb7f044d33ee31fff667803c0cc31a).
Small-model seven-day reference/verify rolling resources, nodes/prices, water/SOC,
state/carry, objectives and residuals match. Full2000-node seven-day runtime remains
unmeasured. Release74 cases/29807 assertions; focused Gurobi ASan+UBSan7 cases/377
assertions pass with detect_leaks=0 and container checks enabled, remaining Release
archives mean no whole-library sanitizer claim. Southern and operation GUI/API pass.
Supplemental trade/noncompact-commitment/nonzero-storage-minima matrix tests pass.

Standalone original-matrix structural diagnosis plus production Gurobi logs identifies
SCUC relaxation Factor NZ1.014e8/Factor Ops1.284e11, repair4.901e7/2.414e10,
LMP1.369e7/9.464e8. Wide intertemporal hydro energy, storage cycles, start/stop totals
and spatial reserve rows are candidates for future structure work; structural degree
proxy is NOT measured factor fill, device timing attribution or proof of root cause.
Evidence: output/market-performance/lmp-reuse-dev/{build,verification,test-rebuild}.json,
lmp-reuse-repetition/{summary,verified-summary}.json, lmp-reuse-checks/ and
lmp-reuse-asan/{build,test-rebuild}.json. Same preserved dirty archives/dependency guard.
Server SHA256948626bcdc7602d27352f3baf333d4a6599a23bcc81e6d2707f18f462c20317a.
Preview http://127.0.0.1:8107/xjtu/#southern-market; saved2000 boundary, idle at startup,
older ports retained. The following scale-final5 record is the preceding baseline.

2000-node optimization has completed the following scoped validation. The GUI-default fixture has
2000 buses,3206 branches,120 thermal+720 hydro+480 renewables,180 reservoirs,
80 storage and120 controllable loads at98 points. Original price-final binary
failed SCUC after187.889s. Final scale-final5 browser click-to-render waits are
60.345730792/59.45793225s (mean59.901831521s), with valid schedules/prices.
Strict <=60s on every run is NOT established; do not cherry-pick the faster run.
Final candidate precision remains strict; all failed tuning experiments are in
performance.md. The544-thermal variant and a2000-node full week were not tested.

Implemented: independent outward-rounded Lagrangian box lower bound, fixed-integer
candidate repair with original-unit audit, same-call SCED bound/candidate reuse
after exact LP/bound checks, conditional SCED derivation with independent verify,
empty commitment-window skip, family index hints, scalar result aggregation,
indexed exports and ownership transfer of completed JSON/SCED solution maps. Original constraints,
water/SOC equations and pricing gates remain. Candidate trajectories may differ
from a previous gap-feasible solution; no claim of exact optimality at positive gap.
Large Gurobi pricing policy is ordered-lp-barrier-8-v2 with concurrent independent
fresh solves and exact full-dual gate. Small pricing remains v1. Failed network
bounds/BarOrder/PreDual and bulk-index-sort experiments were removed. Final Release
71 cases/29589 assertions pass; Southern and operation GUI/API tests pass.
Gurobi ASan/UBSan scope passes5 cases/327 assertions with container checks enabled.
The coherent native sanitizer build at scale-final2 passes6 cases/177 assertions,
with1 Gurobi-only skip. Mixed uninstrumented HiGHS/container-annotation failures
are retained, not hidden by disabling checks; no whole-library sanitizer claim.

Full final responses, including objective/residual/resource/water/SOC/price values,
match scale-final2 exactly after excluding timing/assembly diagnostics. Full-size
verify independently matches6 matrices. Cross-process identical pricing LPs match
all3247054 row duals bitwise; injected failed checks export null prices. SCUC gap
0.0652742323%, SCUC/SCED residual2.1792512e-10, LMP residual2.3235316e-9.
Final local overlay: output/market-performance/scale-final5/build.json (parent
provenance chain, no dependency guard changes), verification.json and test logs.
Server SHA25655ca6063e2c633e172f0841666f2a22fecfa60d40dbb836836db7bdd985b0285.
Preview http://127.0.0.1:8106/xjtu/#southern-market, PID99763, saved2000 boundary,
Gurobi8 threads/auto strategy/cached assembly, idle at startup; older ports preserved.

Earlier small-model deterministic pricing follow-up: `ordered-lp-dual-simplex-v1` fixes only LMP to
fresh single-thread dual simplex, independently of requested dispatch method or
threads. Every LMP is solved again within the remaining stage budget; finite
original-unit full row duals must match exactly, both statuses must prove optimality,
and objective/primal residual audits must pass. Failed checks export null prices
and block energy settlement. The GUI displays the check and separate repeat wall.
Scope is identical ordered LP/backend/version/platform, not mathematical uniqueness
or cross-backend dual identity. Generic dispatch algorithms remain configurable.

Final local incremental build: `output/market-performance/price-final/`, server
SHA256 `87becb70f7da09e3a5a6170d8ad3412fe3febf6e51673a08c4462134a8643442`.
Release 68 tests/29305 assertions pass, including existing RT/ancillary cases;
ASan/UBSan price_consistency 2 tests/26 assertions pass (no Gurobi, detect_leaks=0).
Final-binary operation and forecast GUI/API tests pass. Single-assembly IEEE118
counterexample: 199246 row duals match exactly across two fresh processes and
requested auto/dual_simplex/barrier; objective 14034457.13961928, residual
1.1596057447604835e-10. Injected failed check exports only null node prices.
The initial new test used invalid nonzero HiGHS threads; corrected to the existing
contract, then full tests rerun. No validation gate was relaxed.

Four sequential final-binary seven-day browser runs: always/full 170.628 s,
always/dispatch_only 140.670 s, anomaly/dispatch_only 41.771 s, verify/anomaly
47.473 s. Strict comparator passes: all-mode main price difference 0, all 147
compared objective/residual differences 0, exact physical/water/SOC/recovery
metrics, and 21 main stages/42 verify matrices exact. Default week adds3.209 s
versus the old38.562 s, close to the predicted3.5 s and below50 s acceptance.
Old automatic policy to fixed policy can change historical prices (max10.6907241
CNY/MWh); physical/carry values remain exact and objectives/residual differences
stay below1e-6. Do not claim bitwise price identity with the old policy.

New service http://127.0.0.1:8105/xjtu/#market-operation, PID66948, saved IEEE118
boundary/seed with anomaly/dispatch_only, ready 0/7 at launch. Previous services
8097/8101/8102/8103/8104 preserved. Performance and reproducibility ledger:
`docs/modules/market/performance.md`, `output/market-performance/price-*`.

Recovery policy follow-up is implemented: new GUI/forecast defaults use anomaly
triggering (realized-day deficit/surplus/line-overload sum >1e-6 MW); legacy configs
without explain_trigger retain always. Recovery pricing defaults to dispatch_only;
main LMP remains enabled. Historical-day manual explain uses stored state_start and
original lookahead without advancing days, changing carry/statistics, or rewriting
original execution timings. Gurobi thread-local timing splits environment, model
import, optimize and extraction; presolve/search remain explicitly unmeasured.

Final incremental build `output/market-performance/recovery-final/` replaces four
market units, server and external adapter, retaining other archives/libraries.
Server SHA256 `59c415d2c21c28b3a13ecea3a8f066b1005b60a0d5e03699077ea7063337ad5e`.
Release 66 tests/29262 assertions pass; ASan/UBSan recovery_policy 2 tests/53
assertions pass (Gurobi unavailable in that build, leak detection disabled).
Operation/forecast GUI/API pass including manual scopes, unchanged historical
state/statistics, invalid/stale requests, responsive views and legacy API behavior.

Same-binary seven-day browser wall: always/full 157.075 s; always/dispatch_only
136.086 s (-13.36%); anomaly/dispatch_only 38.562 s (-75.45%, zero anomalies).
Performance predictions met; physical trajectories and water/SOC carry are exact.
42 recovery metric sets match; 147 compared stages differ at most 4.47e-8 in
objective and 1.12e-10 in residual. Main verify: 21 stages/42 exact matrices.
Historical day 3 manual recovery: 17.986 s, all original main results unchanged.

Historical recovery-policy build: the predeclared price-identity gate FAILED in the dispatch-only full week:
max main LMP difference 6.64876 CNY/MWh, while auto/verify prices match full exactly.
One assembled LP (Gurobi fingerprint 0x0819db94) independently reproduces both
price vectors with auto and dual simplex, equal optimal objective within 1e-6 and
valid residuals. This is nonunique optimal dual selection under concurrent LP,
not rounding or changed SCED/water/SOC. Do not claim all-mode price identity or
silently relax its tolerance. `compare_market_recovery.mjs` records false and
returns 1 for those archived runs. The deterministic pricing follow-up above
replaces concurrent selection; archived failed evidence remains unchanged.
See performance.md and recovery-lmp-repeat diagnostic artifacts.

New service: http://127.0.0.1:8104/xjtu/#market-operation, PID58652, same saved
boundary/seed with explicit anomaly/dispatch_only, ready 0/7 at launch. Desktop
and mobile checked; prior 8097/8101/8102/8103 services preserved. Full evidence,
commands, dependency limitations and solver warnings are in performance.md.

Assembly follow-up now implements `execution.assembly_mode=cached|reference|verify`
(default cached), independently of compact/reference formulation. Sorted vector
terms preserve original ordered accumulation; indexed column lookup removes
repeated string-key construction. Exclusive per-stage bounded template leases
retain storage and CSC positions, recomputing all numeric data and rebuilding
on any layout/nonzero pattern mismatch. No solver model/solution cache or skipped
SCUC/SCED/LMP stages. See the assembly section of the performance ledger.
First three-run diagnostic: assembly median 0.6535 -> 0.4779 s (26.9% less),
request 4.5116 -> 4.3054 s (4.57% less). This misses the predeclared assembly
<=0.32 s / roughly 7% request target; the cost model overstated removable work,
since equation evaluation, row names and certification/report scans remain.
All nine verify stages match original matrices, objectives and residuals exactly.

Final incremental build `output/market-performance/assembly-final/` recompiles
southern_market, market_operation and southern_boundary with recorded existing
dependencies. Server SHA256
`92b13f4329e79ee3a690c1f6ff3bc9194a3eecb58fec54137b8d9bd675a0ec6c`.
Release full market: 64 tests/29176 assertions pass. ASan/UBSan assembly cache:
2 tests/88 assertions pass (exact matrix identity, topology/zero coefficient/
resource changes, concurrent leases, two-day water/SOC carry). Direct operation
and forecast GUI/API regressions pass. Independent IEEE118 resources regression
passes cascade/SOC recurrences, peak/outage cases, 14 sampled days and 28-day
carry; AC security remains explicitly failed.
Complete-week reference/cached browser wall is 165.664 -> 158.446 s (4.36%
less), all 7 main chains/42 interventions retained. Stage assembly sum falls
32.911 -> 24.143 s (26.64%); 282 of 294 matrices reuse CSC storage. The extra
verify replay passes exact matrix/model checks for all 147 stages/294 matrices.
Reference/cached objectives, residuals, trajectories, prices and carry match
exactly. Verify's scalar reports differ by at most 1.87e-9 objective and
3.40e-11 residual, while trajectories/carry remain exact; scalar bitwise equality
is explicitly false, not concealed by the 1e-6 numerical acceptance. Evidence:
`assembly-week-{reference,cached,verify}/` and cached `assembly-comparison.json`.
The two performance replays were sequential without other agent compute;
verify and independent resources were concurrent correctness tasks, not timings.

New cached-assembly service is `http://127.0.0.1:8103/xjtu/#market-operation`,
PID 47315, with the same saved boundary/seed/week configuration ready (0/7 at
launch). Launch log/record in `output/market-performance/assembly-service/`.
Existing 8097/8101/8102 services remain untouched and do not pick up this binary.

The current compute changes go beyond the earlier frontend-only correction below.
`southern_market.cpp` projects eligible zero-minimum storage onto hourly direction
integers (196 -> 26 per asset) and fixes a dominant available-state representative
for costless, unrestricted hydro/renewable commitment. Positive minima, costs,
slow ramps or event limits retain the original commitment; thermal, realtime and
ancillary paths are excluded. The original constraint/residual audits remain.
`market_operation.cpp` runs at most two independent same-day Gurobi interventions;
chronological carry stays sequential. Automatic threads resolve to two per recovery
worker. Day timing separates main solve, recovery wall and total wall time.
See `docs/modules/market/performance.md` for proofs, admission and benchmarks.

Follow-up verification repeats the saved complete week at 164.279 s (prior
optimized 164.951 s, old 304.315 s); all 147 stage comparisons pass. A new
`profile_market_parameters.mjs` tests the saved first-day main chain with the
actual next-day forecast points, keeping 120 s and 1% GAP. Three-run medians:
auto 4.498 s, explicit start 6.229 s, dual simplex 5.256 s, barrier 4.948 s,
certified row removal 4.456 s, two threads 4.456 s. None meets the declared
10% reduction, so defaults are unchanged. All 54 stages pass feasibility and
pricing; max residual 3.63e-8, relative objective difference 0.006363.

Diagnostic-only timers now expose boundary validation, full solve wall,
dispatch-map export, daily boundary generation, summary and analysis, including
recovery runtime/validation/boundary fields. Timers do not alter solver options
or model equations. Isolated incremental `timing-overlay/run_gui_server` has
SHA256 `5a19619a4a699ac4b3e144db4f30b8cd859849fefbf4dab03e89356aa2118230`.
Three-run median 4.511 s is +0.30%, within the predeclared 2% timing overhead
gate. Representative request: boundary generation/validation 0.0419 s (0.93%),
assembly 0.6365 s, solve wall 3.3838 s (75.0%), audit 0.2630 s, dispatch export
0.0629 s. Parameter parsing is not the principal bottleneck in this input.
Exact first-day resource/period/lookahead/carry and stage objective/residual/size
parity passed. Updated operation GUI/API timing-accounting regression and
forecast GUI/API generation/statistics/export/reload/mobile regression passed.
Evidence: `verification-repeat/`, `parameters/`, `timing/` under the performance
output directory. No existing 8097/8101/8102 process was replaced; diagnostic
fields are currently in the separate test binary, not the existing service.

The user's saved one-scenario/week workload includes seven main chains and
42 paired interventions. A real browser replay of the old backend took 304.315 s;
main chains 44.163 s, recovery solver stages 214.178 s, recovery assembly/audit
36.806 s. Recorded browser long tasks were only 109/152 ms. This is not the
single-day 23.654 s median from the earlier storage/parallel milestone (old
39.108 s, reduction 39.5%, 63 matched stage audits, max residual 1.265e-10).
The saved input and exported resolved base are in `output/market-performance/live-input/`.
The matched optimized browser replay completed all seven days/42 interventions
in 164.951 s (45.8% less); main 38.879 s, recovery wall 124.552 s. All 147
matched stages pass, max residual 3.40e-9, max relative objective difference
0.0095151 within the existing 0.01 gap. This is one full-week replay per version,
not a universal latency guarantee. Evidence: `live-before/`, `live-after/` and
`live-after/comparison.json` under `output/market-performance/`.
Three-run isolated single-day medians with storage projection held constant:
Gurobi 4.913 -> 4.469 s (9.0%, below predeclared 25%); HiGHS 17.072 -> 4.889 s
(71.4%, passes). SCUC binary declarations drop by the predicted 4704. The
Gurobi miss is reconciled by its measured SCUC fraction (39.7%) and SCUC speedup
(20.0%); full-chain cost prediction was overoptimistic, not a failed identity.
Evidence: `commitment-before/` and `commitment-after/` under the same directory.

Optimized local service is 8102, PID recorded in `output/market-performance/service/server.json`,
binary `commitment-overlay/run_gui_server` (SHA256
`5f2f011d87101c71230a2ab3452092224a7d0896bb43b1d943a588d135887c75`).
At launch it received the saved boundary and identical sampled-week configuration in ready state.
Existing 8097/8101 sessions/results remain untouched; their old processes do not
pick up rebuilt C++ automatically.

The optimized release overlay passed Southern 62 cases/29082 assertions before
the oracle-only test revision; revised commitment oracle passes 172 assertions.
ASan/UBSan projection checks pass 1086 assertions; Gurobi parallel recovery is
skipped in this sanitizer archive because Gurobi is unavailable there. Release
does cover parallel recovery and GUI auto-thread/budget fallback. The old full
reference formulation triggers a HiGHS debug assertion at `HighsDomain.cpp:1705`
on the partial-outage oracle; this remains unresolved and is not a sanitizer
pass. The production compact oracle retains the original free commitments and
has identical energy/reserve equations in the reservoir-free test fixture.
Direct optimized Node operation, forecast, ancillary, realtime and IEEE118
resources E2Es pass. Resources independently checks water/SOC/pressure cases,
14 stochastic days and 28-day carry. AC security still fails as expected.

Builds use `tools/market_validation/build_market_overlay.py`: two changed market
translation units and the test source are rebuilt with existing generated flags,
then linked ahead of the recorded existing archive. Dirty MIPSolvers prevents a
clean CMake regeneration; no dependency guard or pin was bypassed/changed. These
are incremental local builds, not a clean release certification. Source, binary
and archive hashes plus commands are in each overlay's `build.json`.

IEEE118 frontend performance correction adds `market_activity.js`: elapsed
request time, actual bytes received, scenario/day labels, and lightweight
`/api/session/status` polling while a request is pending. Session `busy` is not
a solver-stage heartbeat; no invented percentage/ETA. POSTs are never retried.
Overlapping GETs are merged within a mutation generation, redundant operation
loads removed, and hidden weekly/forecast plots deferred until shown. No
solver/options/precision/backend changes or C++ rebuild in this correction.
Read-only measurement of the same 135,439,395-byte forecast on service 8101:
four downloads -> one (541,757,580 -> 135,439,395 bytes, 75% reduction);
hidden weekly overview SVGs on step 3: 18 -> 0. Current repeatable profile:
`node tools/market_validation/profile_market_gui.mjs --base http://127.0.0.1:8101`,
evidence `output/market-activity/profile.json`. This is a frontend improvement;
large whole-job responses and solver compute time remain. New activity E2E
passes delayed response/elapsed clock, honest global busy, pending pause,
heartbeat failure, real day solve, deferred plot rendering, network failure and
desktop/mobile visibility. Its CTest registration has not run via CTest.
This correction directly passed eight Node scripts with the existing
`build/macos-release/tests/run_gui_server`: `market_activity`, `market_workflow`,
`market_operation`, `market_forecast`, `southern_market`, `market_study`,
`yunnan_ancillary`, `southern_realtime` (all filenames end `_e2e.mjs`). Study's
18 IEEE118 scenarios took 100.477 s before its additional carry checks; all AC
audits still fail as expected. Nine syntax checks and `git diff --check` pass.
Temporary E2E servers exited; the existing 8097/8101 sessions were preserved.

Market workflow now has two model families and ten distinct pages: five Southern
workspaces (operation, boundary/day-ahead, study, Yunnan ancillary, realtime) and
five generic AC/DC pages (participants, inputs, clearing, security, settlement).
The duplicated five-step strip is removed; study and Southern boundary editors
each have one scope. `market_canvas.js` caches updates by workspace owner so a
late response cannot overwrite another page. Ten hash links survive asynchronous
startup/reload; market entry skips the unrelated automatic engineering tour.
Six task-specific tutorials and the searchable Chinese operation manual are
available from every market page. Narrow screens show operations before the
optional topology. See `docs/guides/market_simulation_workflow.zh.md` and the GUI
audit in `docs/modules/market/southern_execution_contract.md`.

Eight direct Node E2E scripts passed against the existing macos-release server:
`market_workflow`, `market_gui`, `southern_market`, `market_operation`,
`market_forecast`, `market_study`, `yunnan_ancillary`, `southern_realtime`.
Command pattern: `node tests/e2e/<name>_e2e.mjs --server
build/macos-release/tests/run_gui_server`. New workflow evidence is in
`output/market-workflow/verification.json` and 30 screenshots (ten pages at
1440/768/390 px). It covers a real two-node solve, help opening/closing,
initial mobile topology, first-visit deep links, and delayed-response/404 handling.
Study passed 18 IEEE118 scenarios plus seven-day joint-fault carry checks;
all 18 AC audits still fail and their ledgers remain conditional. Generic
`n1_security_failed` and game `maximum_rounds_reached` are expected test outcomes,
not safety/equilibrium certificates. No C++ solver or mathematical changes in
this UI correction; no rebuild or CMake regeneration under the existing dirty
dependency guard. The new workflow CTest registration has not run via CTest.

Existing service 8097 still has an older backend: ancillary/realtime GETs return
404. The UI now explains this in Chinese. Existing service 8101 supplies these
APIs and serves the updated frontend; both user sessions were preserved. The
broader design in `docs/modules/market/system_design.md` remains a proposal for
unified persistent tasks and immutable stage references. Automatic
day-ahead/AGC/realtime handoff has not been implemented.

The AEMO adversarial campaign is documented in `docs/modules/market/aemo_validation.md`.
Seven official DispatchIS archives plus next-day unit dispatch and the existing bid
archives are pinned in `external_data/market_validation/source_manifest.json`.
Regional balance max0.010000000002MW, 5min/15min energy relative error3.147e-15;
25 FCAS award/actual-availability differences remain unresolved field semantics,
with complete source rows retained. These differences are excluded from the
regional `identities_pass` flag. No NEM replay or Southern empirical certification.

`tools/market_validation/` runs108 finite checks including256 complete two-unit
commitment combinations (17 feasible; objective132.58706884519836 matches
HiGHS/native/Gurobi), two-bus LMP and separate positive deficit/line overload,
mixed-resource independent physics,12 mutations,7x96+2 day rolling,4 realtime
windows, AGC shortage/success counterfactuals,14 stochastic scenario-days and
4 fault/inflow AC studies. Independent maxima: nodal1.706e-12MW,
SOC1.635e-12MWh, water level1.422e-14m; hourly price reconstruction error0.
All108 checks pass, INCLUDING expected rejection gates: all4 AC studies fail
security; energy-only UC lacks AGC capacity in hours10-15; conditioned research
ledgers are not formally eligible. Post-outage AC nonconvergence still lacks
detailed cause metadata and a zero default residual is not a feasibility proof.

Direct Python18 tests pass (9 independent audit,9 existing bid tests); new
`market_independent_audit_unit` is registered but not executed through CTest.
Existing macos-release binaries pass Southern59/27962, generic22/845 and
forecast8/12503 (89 cases/41310 assertions). No C++ rebuild this campaign:
the existing dependency pin/worktree mismatch remains. Four direct Node E2Es
pass (realtime, Yunnan ancillary, forecast, generic GUI); the generic navigation
expectation was corrected from6 to the implemented10 pages (5 southern + 5
generic; this record previously said 8, which was never an implemented state).
Its expected `n1_security_failed` and `maximum_rounds_reached` outcomes are not
security or
equilibrium certificates. Evidence, exact commands, binary/source hashes and
coverage limits are in `output/market-validation/` and the module report.
Production solver/GUI behavior was not changed by this validation campaign;
temporary experiment servers exited and existing user GUI sessions were preserved.

Bidding behavior literature/source audit is recorded in
`docs/modules/market/bidding_behavior_review.md`: Southern scenarios use
synthetic common bid multipliers; the generic local-best-response game is
separate. No empirical bid calibration or water-opportunity-value bidding
has been established. The review verifies AMES mechanisms and the2024 Aneo
hydropower benchmarking paper, and distinguishes implementation tests from
real-world behavioral validation. A subsequent empirical experiment is now
recorded in `docs/modules/market/empirical_bidding_analysis.md`:
seven official AEMO bid archives (385 positive-capacity GEN DUIDs,58 LOAD DUIDs),
chronological descriptive comparisons,27 paired98-point Southern API solves
with HiGHS, and9 directly executed Python tests all pass. Independent merit
order audit: max power residual0 MW, cost relative error1.88e-13, LMP interval
distance0. Runtime0.389-0.585s for fixed-on19/19/17-unit single-bus cases.
Outputs and three PNG/PDF figures are in `output/market-bids/`; source ZIPs
and official dictionary in `external_data/market_bids/`. No production solver
or GUI changes; temporary experiment server exited, existing GUI preserved.
This is foreign bid-shape transfer, NOT Southern behavioral calibration,
NEM dispatch replication, a real-time-available forecast backtest, or UC/network
performance validation. Nine tests are registered as `market_bid_empirical_unit`
but CMake regeneration/ctest was not rerun under the existing dependency guard.

Southern chapter3 realtime is now a separate C++ job and GUI page
`/xjtu/#market-realtime`, API `/api/session/southern_realtime`.
It uses24x5min SCUC/SCED, independent8x15min LMP, two-hour reference outlook,
and15min executed-state carry. Sourced6h forecasts, sealed energy offers,
province accident reserve, per-period hydro energy and priority transfers,
storage hour history, topology outages and nullable historical hourly prices
are documented in `docs/modules/market/southern_real_time.md`.
Final Release Southern59 tests/27962 assertions and ASan/UBSan
realtime6 tests/81 assertions passed (leak detection disabled).
Reports: `output/market-realtime/{release-tests,sanitizer-tests}.txt`.
IEEE118 two-roll E2E passed with Gurobi0.801/0.776s dispatch windows,
water level residual6.74e-15m, storage/carry residual0 and no browser errors.
Desktop1440 and mobile390/768 screenshots and independent conservation
audit are in `output/market-realtime/`; Southern editor and Yunnan ancillary
E2E also passed after the shared builder change. Invalid realtime declarations
are explicitly caught as typed400 errors; stale revision/run ID returns409.
GUI shows actual solver availability, independently selected reservoir curves,
and disables execution when a raw JSON draft is pending.
Live updated server:8101 PID81465, `/xjtu/#market-realtime`, with four
IEEE118 mixed rounds preloaded. Gurobi dispatch plus pricing0.805-0.842s per
round; bus1 actual four-quarter hour0 price48 CNY/MWh. This timing excludes
the separate2-4h reference solve; it is not a2000-bus benchmark.
Cached object/archive/link rules were used because the existing sibling
dependency pin/worktree mismatch still prevents normal CMake regeneration;
no dependency pin or unrelated worktree change was reset. The new E2E is
registered in tests/CMakeLists.txt and was run directly with Node.
This is an explicit research execution interpretation, not certification
of AGC/deep-peak clearing, official price repair or dynamic security.

Yunnan auxiliary page now implements hourly AGC prearrangement after SCUC,
with fixed commitment/stable state/primary reserve, plant AGC aggregation,
five-minute upward/downward reserve, and hydro connected safe operating bands
in coupled SCED. LMP freezes bands and secondary allocation. Official 2025
regulation and black-start PDFs (joint notice 114) are downloaded, hashed and
mapped to equations in `docs/modules/market/yunnan_ancillary_markets.md`.
Black start is rule extraction only. The new independent session endpoint is
`/api/session/yunnan_ancillary`; main boundary revisions invalidate its saved
declarations/results. Raw JSON drafts disable running until validated.
The workflow extension adds sealed bid logs, sourced article41 safety
removal/backfill/uplift, fixed-day-ahead intraday SCED, suspension/same-hour
pricing and conservative real-time capacity envelopes. Independent storage
and controllable-load AGC now enforce sustained headroom and energy-market
exclusion, including the conditional energy ledger. Explicit AUTOR command
records feed compensation; complete daily statements can enter an append-only
simulation journal with corrections and whole-month energy-weight allocation.
GUI exposes these actions, plots, independent-resource editing and journal
export. The journal is session-local, not a persistent legal accounting system.

Verified final Release Southern53 tests/27881 assertions and focused
ASan/UBSan ancillary8/2077, with leaks disabled. Reports are
`output/market-ancillary/{release-tests,sanitizer-tests}.txt`.
Hand oracles cover merit ties/fallback/cap,
hourly availability minima, nonzero frozen primary, reserve sums, safe bands,
capacity shortage and a physically incompatible-band failure. Forecast formula
coefficients are authored, not falsely presented as fixed official defaults.
IEEE118 has36 hydro grouped in12 synthetic plant AGCs plus18 thermal AGCs,
with wind/solar/storage/controllable demand retained in the energy model.
Gurobi20 MW test4.790s and HiGHS approximately22.3s both solve; independent
all-member/98-point reserve and band checks pass. Registered ancillary E2E
passes save/reload,400/409, plots, Canvas slots/clicks, stale/reset semantics,
and390px viewport geometry. Southern and operation GUI/API regressions pass.
New plotting legend geometry is checked separately from page overflow.
Expanded ancillary E2E passes IEEE118 day-ahead + intraday safety replacement,
sealed-bid rejection, stale clearance IDs, duplicate/incomplete metering,
actual GUI compensation and daily posting, duplicate-accounting rejection,
missing-month-day gating and journal download. Hourly mileage bars replaced
crowded unit/hour category ticks;390px axis-title separation is asserted.
Southern editor and operation regression suites pass on the updated server.

Top-level Release server updated atomically. New live8099 (PID52096) serves
`http://127.0.0.1:8099/xjtu/#market-ancillary`, with mixed118 day-ahead and
intraday results: Gurobi4.563s/2.302s,20 MW research demand, independent storage
AGC3 MW and load AGC1 MW, plus conventional AGCs. Removing one conventional
AGC triggers safe backfill. Synthetic metered compensation16 CNY and cash
residual0; all98-point frozen states, plant reserve, hydro-band, and storage
energy-exclusion residuals0 in `output/market-ancillary/live-rules-audit.json`.
The first load5+3 MW experiment correctly failed against the unchanged80 MWh
daily cap (192 MWh required); its failure JSON is retained. The accepted1+1 MW
experiment requires48 MWh. No boundary capacity was relaxed to obtain success.
HiGHS workflow E2E also passes; latest captured day-ahead21.440s, intraday
11.436s in `ieee118-intraday-highs.json`. These are individual observations.
Older live8098 is preserved and serves
`http://127.0.0.1:8098/xjtu/#market-ancillary` with saved mixed118 results:
10 MW primary, Cmin50/R1=.005/R2=.003, hourly secondary demand68.789–76.747 MW,
awards70–80 MW, Gurobi5.106s; independent fixed-state/capacity/band residuals0.
Artifacts are `output/market-ancillary/`. Existing8097 study and8095 weekly
services are preserved. Cached direct object/archive/link builds were used;
dependency pin and dirty-check guard are unchanged. Existing DynamicSystem
class/struct forward-declaration warning remains outside this change.

Limits: one-day98-point auxiliary task, no automatic weekly AGC-history carry;
actual timestamp validity of last-eight performance/qualification records,
official raw-signal statistics, automatic unique safety-failure attribution,
head-dependent vibration/transition, AGC activation dynamics/security,
market supervision/disclosure/approvals and legal remittance remain external.
Monthly outputs restate latest metered daily versions; next legal settlement
payment and externally approved correction exceptions are not automated.
Safe bands and qualifications are authored research declarations. Successful
linear schedules are not AC/dynamic safety or regulator certification.
The prior study baseline and its independent limitations follow below.

Fault/inflow study now uses the Southern mixed-resource rolling model through
an independent `/api/session/market_study` task. Day/week/month calendars keep
96 realized + 2 D+1 points. Up to64 deterministic fault/inflow/bid combinations
have independent UC/SOC/reservoir carry and common baseline references. Five
market pages share scenario selection, comparison curves, actual stage status,
optional98-point post-dispatch AC audit and a conditional closed-AC energy
ledger. Canvas scenario/day/slot controls and curve clicks are bidirectional;
the existing full-device plan can display a study scenario. Single-day prices
show all-node range/median plus a selected node instead of80 overlapping traces.

Current Release Southern45/25804 and focused ASan/UBSan study2/1874 pass, leaks
disabled. Registered study E2E passed18 IEEE118 scenarios (3 fault profiles x3
inflows x2 bids), then a7-day joint-fault run with exact carry and paired
restoration. Actual Plotly, all five pages, Canvas/time clicks, independent
cashflow recomputation and desktop/mobile geometry pass. Generic mixed-market
six-tab workflow, Southern editor, forecast, operation and Canvas-layout suites
pass. The operation browser harness initially fulfilled a request cancelled by
navigation; draining background requests before changing route mocks fixes the
reproducible harness failure. Additional live checks opened the study full-device
heatmap and selected price node118; a390px actual chart fits within374px.

18 scenarios have valid linear clearing/LMP and no deficit or active-flow
overload, but all fail AC security. PF converges; violated P/Q capability,
voltage and apparent-power limits remain explicit. No limits were weakened.
Cashflow maximum residual5.93e-10 CNY is below the fixed1e-4 independent test
threshold. First18-day study elapsed102.803s; another full run103.521s. This is
not a2000-bus performance claim. Formal settlement eligibility is alwaysfalse:
Southern real-time/contract/auxiliary-service settlement remains outside this
conditional research ledger; external schedules/DC links return unsupported.

Export replay exposed an omitted explicit native_root_cuts default in the raw
baseline. Persisting resolved solver defaults fixes exact replay, with no
numerical model change. Current test counts above include the new replay and
invalid-mode/reference tests. Build used cached object/link rules because the
dependency guard blocks normal regeneration; dependency sources/pins were not
edited. Top-level macOS server executable is replaced atomically. Active study
service8097 and `output/market-operation/fault-inflow-study/` hold the retained
experiment; prior8095 weekly session is preserved. Detailed units, equations,
API mapping and limitations: execution contract, Fault/Inflow Study.

Market Canvas now uses the existing ELK layered router, distinct per-branch
ports, a24-bus/two-hop default neighborhood and explicit80-bus expansion.
Bus values are opt-in; branch values stay in tooltips/inspector. Same-revision
reloads and time changes retain positions/zoom; obsolete layout promises are
cancelled when selection changes, including return to an already-visible bus.
Independent workspace tracks prevent toolbar/topology/inspector overlap.
Registered `market_canvas_layout_e2e` passes both fresh IEEE118 and live8095
sessions: layout boxes/labels, distinct parallel paths, selection races,
shared-reservoir highlighting and desktop/mobile geometry. Existing operation
and Southern GUI/API suites pass. Artifacts: `canvas-layout/`; live8095's
seven-day result is unchanged. This is a static GUI change, with no new C++ or
numerical validation claim. See execution contract Market Canvas Layout.

Weekly/monthly results now default to six graphical overviews and an all-device
heatmap/curve explorer; numeric tables are collapsed. `market_weekly_plan.js`
supports16 metrics,40-device pagination without data truncation, full-calendar
CSV, qualified Canvas selection, and null gaps for unavailable/stale results.
SCED primary reserve is exported; per-generator signed up/down constraint
contributions, area demand/margins, renewable availability/consumption and
directional active line loading are projected in the daily report. No reserve
award or AC apparent-power certification is inferred. The inherited300px
chart container initially clipped heatmap rows; explicit calculated height
fixes it. New sessions without saved mode preferences can open existing manual
results, preserving explicit user choices.

Verified current Release Southern43/23930; focused ASan/UBSan weekly projection
1/963, leak detection disabled. Mixed seven-day plus forecast-resolution E2E
passes numeric/source/plot/CSV/filter/stale checks; operation and forecast
GUI/API regressions pass. All7 objectives, variable and nonzero counts match
the prior mixed-fixture log exactly. Latest live8095 stores a full seven-day
mixed stress plan; top-level server executable is updated atomically. The
final desktop/mobile and actual heatmap click checks use the retained live
report. See execution contract Weekly Plan Results for formulas/field units,
cost model and verification limits;2000-bus monthly rendering remains untested.

New built-in `market_ieee118` and Southern action `ieee118_mixed` implement
`IEEE118-mixed-v1`: original54 units become36 hydro/18 thermal;6 wind,6 solar,
12 shared reservoirs in4 three-reservoir chains,6 storage and6 compensated
loads. Source network/capabilities/IDs and daily demand energy are retained;
fuel assignments, offers, temporal shapes and water data are explicit synthetic
assumptions. The global load also installs the companion Southern boundary;
old `ieee118` is retained as the previous performance fixture.

Verified: Release Southern42/22870; current-source generic market22/845;
focused mixed-fixture ASan/UBSan1/273 (leaks disabled). Mixed browser7-day stress
plus forecast-resolution tests, resources independent conservation/ablation,
2 joint scenarios/14 solved days,28-day February month with exact carry,
generic market six-tab workflow, Southern editor and operation regressions pass.
Generic workflow explicitly preserves N-1/AC rejection; only the conditional
linear financial experiment converges (cashflow residual2.0e-11). Source CTest
entries register mixed/resources/generic suites; the guarded cache uses direct
invocation. The generic test object was rebuilt from current source; its stale
link file references GCC15 and duplicates HFactor archives, so a temporary
invocation reused the current Southern test link libraries with the generic
test object. No generated link files or dependency pins were modified.

GUI validation found and fixed generator Canvas round-trip loss of fuel type,
minimum up/down times and maximum starts/stops. Fuel editor now includes every
backend enum. Generic market result limitations explicitly identify exogenous
AC storage/flexible loads and absent reservoir coupling. `case_profiles` gates
the new static selector against old binaries. Rolling day reports now retain
96 realized points in `resources` for generators/storage/reservoirs/controllable
loads, using authored IDs and MW/MWh/m/m3/s units. Canvas selections display
actual dispatch, commitment, charging/energy, water level/release and response;
the final mixed seven-day and operation E2E plus Southern42/22870 pass with
this result-only projection.2000-bus monthly payload memory remains untested.
The initial mixed fixture used service8094; weekly graphics now use8095 as
described above. Earlier services are preserved.

Important open boundaries: Southern AC-required strict run fails certification
and returns invalid prices. Generic N-1 detects actual load shedding, including
branch113 (71->73) and183 (68->116) outages; these are detected insecure
contingencies, not a passing security certificate. Gurobi and HiGHS solve the
new98-point fixture with objective4212673.15329467CNY; Native fails its root LP
at25000 simplex iterations in both initial and fallback attempts. The previous
Native6.4s success below applies only to the old IEEE118 fixture. No Native
repair for the new fixture is claimed. Southern real-time settlement and
forecast price/renewable-utilization aggregates are still missing from a unified
multi-resource workflow. Detailed matrix, equations and evidence are in
execution contract Mixed IEEE118; artifacts `ieee118-mixed*` and
`market_ieee118-generic/`. The month and probabilistic runs are regression
coverage, not real-data calibration or a full statistical validation study.

Native IEEE118 fixed-integer repair now honors its selected HiGHS LP kernel
and remaining optional-root budget in sibling `06_root_heuristics_a.inc`.
The disabled automatic root pipeline no longer blocks GAP exit; an audited
incumbent/root-bound certificate skips optional fixed-point polishing at the
requested gap unless tree exhaustion is required. Feasibility-jump attempts
and flips check the same deadline (`07_root_heuristics_b.inc`). No market rows
or96+2 points are removed. The initial repair-only experiment reached an
incumbent in2.435s but still took45.103s; the GAP control-flow correction then
gave6.401s/full chain and3.532s/SCUC versus roughly37s/no incumbent previously.
Prediction and mismatch analysis: execution contract, Native Fixed-Integer
Repair Debug; current-source sibling B&C regression passes35 cases/490
assertions. Release Southern41/22597 and operation/forecast browser suites pass.
Native IEEE118 E2E passes seven stress days and15/30/60-minute forecast inputs
with unchanged15-minute calculation and60s per-solve budget. Evidence:
`output/market-operation/ieee118-native/`; this is not a2000-bus performance
claim. The new `market_ieee118_native_e2e` CTest entry was invoked directly,
since the dirty-dependency configure guard still blocks cache regeneration.

The ASan/UBSan cache is unoptimized.30s IEEE118 attempts time out (also in an
isolated rerun) without a sanitizer report; the test-only
`HACDCPF_TEST_NATIVE_REPAIR_SECONDS` override permits explicit instrumentation
budgets without changing the default30s Release criterion. At180s per solve,
ASan/UBSan passes2 cases/62 assertions (both IEEE118 profiles plus analytic
root cuts), with leak detection disabled and no sanitizer report. The failed
30s and successful180s logs are retained in `native-repair-fixed/`.
Final isolated alternating default/enhanced/enhanced/default process times
are6.404/6.495/6.394/6.387s; full chain6.276..6.387s, SCUC3.485..3.585s.
All have valid schedules/prices, gap1.7527268e-6, original/reconstructed
residual<=1e-6,1 incumbent,3 LP solves and0 cuts/nodes. This closes the below15s
prediction through repair/GAP exit, not cut strengthening. Top-level server
is updated; live8092 has a verified enhanced Native stress day1/7, ready to
resume. GUI/Canvas and desktop/mobile results were inspected, with no390px
horizontal overflow. Earlier services are preserved. Exact evidence and
build-cache limitations are in the execution contract.

Native root-cut experiments add `execution.native_root_cuts=default|enhanced`,
also carried by rolling/forecast solver_options. Enhanced requests20 rounds /
100 cuts, enables existing audited separation and raises the root row admission
limit35000->100000 (very-large3x30 adaptive caps retained). Default unchanged.
That earlier profile change added no sibling source edits or inequality formulas.
The market preserves public
BCStats via the same solve_milp_bc entry point; stage/rolling GUI reports actual
cuts, nodes, LP solves, incumbent updates and available bounds, with null for
unavailable bounds/LP-only diagnostics. Requested versus measured values are
distinct. See Native Root Cut Experiments in the execution contract.
Capability-gated controls avoid sending new fields to old running binaries.
Release Southern40 cases/22567 assertions, focused ASan/UBSan1/29 (Gurobi OFF),
Southern editor and operation/forecast browser suites pass. An E2E teardown
race from outstanding legacy-capability route.fetch was fixed with awaited
unrouteAll; final operation rerun exits0. Sanitizer covers the numeric/profile
change; the final additive capability metadata was checked in Release/browser.
The previous service8091 has the pre-repair IEEE118 boundary loaded;
earlier tasks preserved. Model performance evidence follows the execution
contract protocol; initial gated zero-cut runs are retained separately.
Pre-repair alternating IEEE118 default/enhanced/enhanced/default process times:
38.592/36.978/37.357/37.822s for30s request. All have0 cuts,0 nodes,0 incumbents,
5 LP solves and best bound17413231.01694171CNY, matching the prior Gurobi optimum.
Measured bound lift0 matches prediction<1CNY; none produced a valid schedule or
prices. The subsequent fixed-integer repair above closes this IEEE118 failure;
2000-node Native solve performance remains open.
Evidence `output/market-operation/native-root-cuts/comparison.json`, complete
profile/admission caveats in the execution contract. Port8091 remains historical.
Live8091 holds failed IEEE118 rolling day0 with enhanced diagnostics:80239 root
rows,2 nonmoving rejections,0 admitted cuts/incumbents,36.979s. Its resolved
D+1 boundary differs from the benchmark, so bounds are not directly comparable.
Live JSON and desktop/mobile screenshots are in `native-root-cuts/`.
Live inspection also fixed empty-result totals/plots: no valid periods now
shows unavailable statistics, hiding empty plots instead of zeros/default date
axes; partial totals explicitly cover valid periods. Updated operation E2E
checks failure-to-valid chart restoration, and live desktop/mobile checks pass.

Certified row omission is now implemented behind `execution.row_presolve`
(`none` default, `enabled` only with compact). SCUC/SCED omit inequalities proved
by outward-rounded variable-box support; all authored rows remain in final audit.
LMP retains all rows. Submitted duals map back to original row indices, and GUI
reports original versus submitted row/nonzero counts. The boundary execution
schema editor exposes and persists the option; rolling/forecast inherit it.
Fixed-variable elimination and variable calculation steps are not implemented.
Reduced2000 assembly removes exactly213331 rows:3333454 to3120123, nonzeros
10515386 to9950617. This is not a completed2000 solve or throughput claim.
IEEE118 removes7022 SCUC rows and passes seven enabled stress days plus forecast
resolution experiments; saved daily deficit/surplus/line-excess integrals match
the preceding default run within1e-6. Rebuilt Release Southern39 cases/22538
assertions; focused ASan/UBSan1/61 (Gurobi OFF), Southern editor, operation,
forecast and enabled IEEE118 browser suites pass. Full pricing objectives may
differ for nonunique predecessor schedules; only the unique analytic price is
asserted identical. Performance and mapping protocol: Certified Inequality
Omission in the execution contract. Artifacts `row-presolve/` and
`ieee118-row-presolve/`; new comparison driver is
`tests/run_southern_row_presolve_comparison.mjs`.
Final isolated three-pair IEEE118 medians none/enabled: Gurobi3.345/3.330s,
HiGHS7.358/7.163s; max original residual6.06e-9. Small local gains do not justify
a default change. Final evidence `row-presolve-isolated/comparison.json`.
Updated top-level executable and new service8090; earlier services preserved.

IEEE118 multi-resource system fixture is available in both market case selectors
and benchmark case118: original118 nodes/186 branches/54 generator records plus
1 wind/1 solar/4 hydro,2 shared cascade reservoirs,2 storage and2 interruptible
loads. Offers, resource additions and200MW line limits are explicitly synthetic;
source synchronous condensers use declared synthetic dispatchable assumptions.
Fixed MATPOWER transformer metadata is checked against its source branch before
alias collapse; original branch parameters and IDs remain intact. Unknown or
modified/controllable transformer aliases are rejected. The generic importer
still rejects unsupported engineering components.

New registered `market_ieee118_e2e` passes seven coupled rolling days with load,
line-limit, islanding, resource and bid stresses, plus15/30/60-minute forecast
resolution at unchanged15-minute calculation. The latter preserves daily load
energy104989.5MWh and132300 columns/481939 nonzeros, but peak deficit6051.213MW
at15-minute forecast disappears at30/60; line excess integrals261.771/930.197/
22.000MW*h are not monotone. Actual30/60-minute calculation is NOT implemented;
the validator rejects0.5h operating intervals. Evidence and rationale are in
the execution contract and `output/market-operation/ieee118/`.

Local30s/GAP.01 algorithm comparison: Gurobi full chain4.253s, HiGHS9.168s,
both valid with SCUC objective relative difference8.90e-6; native44.157s,
time-limit without verified solution. Diagnostic comparison overlapped the
system tests and is not a controlled speed ranking. Rebuilt cached Release
Southern38 cases/22477 assertions, ASan/UBSan IEEE1181/1507 (Gurobi OFF), and
IEEE118/manual/forecast browser suites pass. New CTest entry is in source;
dirty dependency guard still prevents normal reconfigure of the current cache,
so the new browser script was invoked directly. Updated top-level server and
port8089 expose IEEE118; older services/tasks remain intact.

Model-size follow-up adds read-only SCUC inspection and per-stage assembly
census, propagated through rolling/forecast results to an expandable GUI table.
The benchmark accepts final `inspect` mode without invoking optimization.
Source counts confirm reducing synthetic thermal544 to120 removes only10.17%
of columns and15.77% of rows: reduced model2569364 columns/145040 declared
binaries/3333454 rows. Hydro70560 and wind/solar47040 commitment binaries remain;
network plus diagnostic columns total1530564. Conservative box-bound audit finds
213331 redundant inequality candidates; `constraints_removed=0` explicitly
records that this follow-up does not prune or change the solver matrix.
Further custom constraint removal is not implemented or benchmarked. Rules,
certificate assumptions and exact family counts are in the execution contract
under Model Size and Bound Redundancy Audit and `output/market-operation/model-size-*.json`.
Inspection process wall6.892s for120 thermal /7.985s for544 thermal; no solver
speedup is claimed. Rebuilt cached Release Southern37 cases/20970 assertions,
focused ASan/UBSan1/17 (Gurobi OFF), and operation/forecast browser E2E pass.
Port8088 runs the updated binary with a multi-resource demo day1/7 for inspection;
8087 and older user tasks remain intact. Top-level run_gui_server is updated.
Live desktop/mobile screenshots `model-size-live-{desktop,mobile}.png` show the
expanded tables; old saved results lacking the field remain readable.

Solver selection follow-up fixes a forecast-editor capability-loading race:
the editor mounted before the asynchronous solver list and stayed empty because
preserving the forecast draft skipped reconstruction. Mounted operation/forecast
selects now hydrate together, disable while loading, retain authored choices and
reject unavailable selections before execution. Both browser suites pass,
including deliberately delayed capability GETs, direct selection without editing
distributions, actual Gurobi execution, reload persistence and390px layout.
Failure detail now distinguishes requested/actual solver and requested/elapsed
time from solution quality; root failure does not imply proven infeasibility.
Only frontend and tests changed in this follow-up; the previous C++ baseline
below remains applicable. Refresh port8087 to load the new frontend assets.

Reduced2000 diagnostic logs identify Gurobi120s exhaustion in root barrier
with zero incumbents, and native root HiGHS LP rejection at25000 simplex
iterations (confirmed with `MIPSOLVERS_HIGHS_LP_KERNEL_TRACE=1`). A short HiGHS
stack sample beyond120s finds root ziRound repeatedly computing row activities,
with analytic-center IPM on a worker; its inner loop lacks deadline checks.
These findings explain failure mechanisms, not proof of model infeasibility.
Detailed reproduction, evidence and limitations are recorded in the execution
contract under Solver Selection Readiness and Failure Diagnosis. Artifacts:
`output/market-operation/failure-diagnosis/` and `failure-native-kernel/`.
Hard HTTP process deadlines and numerical strategy changes remain outstanding.
The HiGHS diagnostic rerun finished naturally in418.318s process wall
(409.506s SCUC adapter), TimeLimit with original-model violation285.41 rejected.
All three diagnostic runs have no valid prices; comparison.json is complete.
The separate native trace overlapped HiGHS, so timings are not a controlled
performance ranking. JavaScript syntax and whitespace checks pass.

Current follow-up adds a four-step operation GUI, configurable synthetic thermal
fleet (new action `activsg2000_hydro`, default120, original544 case preserved),
defaultgap0.01, native B&C with HiGHS LP, and selected PTDF rows for authored daily
topologies including phase shifts and independent island references. New public
function and HTTP `/api/session/market_ptdf` use stable AC IDs; no dense PTDF is
inserted into the optimization model and no cross-request cache is claimed.
Switching cases resets stale daily drafts. Active backend is now available on
port8087; port8086's existing task was not replaced. The top-level executable is
updated via the cached optimized build; dirty dependency configure guard remains.

Rebuilt optimized Southern suite passes36 cases/20953 assertions. ASan/UBSan
`[ptdf],[reduced_fleet],[local_solvers]` passes3 cases/771 assertions (Gurobi disabled).
Analytic native and HiGHS objective490000/LMP200 agree; phase-shift ring/island
PTDF identity is within1e-6. Reduced1320-unit fleet checks all720 hydro references.
Local multi-resource30s comparison: HiGHS2.249s wall, SCUCgap0.000815861;
Gurobi0.170s wall, gap0; native29.153s, no verified incumbent at time limit.
Both successful full chains pass original-unit audit. These are one-run
observations, not statistical speed rankings or real-market-price validation.
Full numerical and UI workflow validation ledger continues in the execution contract.
Reduced2000 comparison now complete at120s requested optimizer budget:
HiGHS380.485s process wall/15.526GiB, time-limit vector rejected (residual285.41);
native56.178s/13.410GiB, root relaxation failed; Gurobi129.489s/13.268GiB,
time-limit without incumbent. All lack valid prices. HiGHS optimizer actually
took371.953s, so productionHTTP still has non-hard deadline risk; independent
benchmark process watchdog is implemented. No new2000 throughput success claim.
Binary reduction41552 is verified (186592 to145040). PTDF single-row2000 API
0.609s/residual6.90e-12;98 identical topology periods. Final manual/forecast
four-step browser suites pass, including390px. New service8087 has the120-thermal
case loaded; old8086 task remains untouched. Refreshing restores the current
workflow step; mode tabs synchronize `.active` with the accessibility module.

2000-bus performance work now has a completed 98-point SCUC/SCED/LMP benchmark:
1744 generators (720 hydro),180 shared/cascade reservoirs,80 storage,120 flexible
loads,3206 branches. Exact startup-class projection and transition hull remove
683648 columns /854560 binary declarations. Matched120s runs reduce RSS29.4%
but still time out without incumbents. Dedicated Gurobi barrier plus invertible
water-energy coordinate completes the chain in597.122s, peak16.613GiB, SCUC
gap2.21e-7 and stage original residuals <=1.65e-7. Evidence and the unsuccessful
default/concurrent runs are in `output/market-operation/performance-*.json`;
derivations and fixed predictions are in the Southern execution contract.
This is synthetic linear-network schedule/conditional-price evidence, not AC
security or week/month throughput. Final code also completed one actual rolling
day via the port8086 API in680.365s (1/7, resumable), true D+1 slots0/95.
SCUC/SCED/LMP times493.130/145.502/13.364s; max original residual8.12e-7,
SCUC gap8.83e-7, conditional prices valid, all2000 nodes/3206 lines returned.
Partial integer starts were actually loaded and LMP skipped crossover. Export:
`output/market-operation/performance-2000-gui-day.json`. The Node driving request
hit300s headers timeout; GET recovered the persisted result without duplicate
submission. This is not a full-week benchmark or asynchronous progress service.

Build note: user commits during this session moved HySim to8b93145b and sibling
MIPSolvers toe6c932e5; current changes build by cached optimized local incremental
commands. Standard Release configure is refused by the unchanged dirty-dependency
guard. The top-level GUI binary must be copied after linking the tests/ target;
port8086 is the current local performance service. Earlier servers remain running.

Final optimized local regressions: Southern33 cases /20182 assertions, forecast
8 /12503; four registered market browser suites pass34.96s. Rebuilt ASan/UBSan
(Gurobi disabled) passes the reservoir-coordinate test1 /597. The aggregate
`[compact]` sanitizer invocation aborts in embedded HiGHS `HighsDomain.cpp:1705`
`updateActivityUbChange` during the original-form startup fixture, before any
reference/compact comparison completes. No ASan memory report was emitted;
this is an unresolved debug solver assertion, not a sanitizer clean-bill claim.
After the final meter-bound audit addition, rebuilt `[compact],[gurobi]` passes
5 /8958; operation+forecast browser suites pass23.15s. Top-level GUI executable
is updated, while port8086 retains the prior solved matrix's in-memory1/7 result.
Live2000-bus Canvas selection/slot update and390px/1440px screenshots pass;
small nonzero gaps now render scientifically instead of0.

Gurobi integration is verified: explicit Southern execution and
operation/forecast solver options, per-call time/gap/threads, sparse LP dispatch
through the existing MIPSolvers adapter, ranged-row dual remapping and honest
limit status handling. Local Gurobi 13.0 license initialization and two adapter/
parity tests (41 assertions) passed, including actual TimeLimit LP without Pi.
Final Release Southern passes 30 cases / 11263 assertions, forecast 8 / 12503;
four browser suites pass (34.89 s) with real Gurobi manual/forecast dispatch.
Rebuilt ASan/UBSan with Gurobi disabled passes `[solver_options]` (1 / 9), including
explicit unavailable failure and unchanged carry. This does not sanitize Gurobi.
The new port 8085 server completed a multi-resource manual week and a fixed-input
forecast week with Gurobi, 2 threads, 120 s/call, 1% gap target. Day-1 audited
SCUC/SCED objective is 481998.849107 CNY, stage residuals below 9e-14; SCUC took
0.169 s in this small run, not a scale claim. Exports and screenshots are
`output/market-operation/gurobi-{week,forecast}-evidence.json` and `gurobi-*.png`.
Both this repository and sibling MIPSolvers contain the required source changes.

Rolling correction: windows now assemble D+1 peak/valley time-series inputs from
the following sampled/overridden 96-point forecast, with stable-ID/source-slot
provenance. Calendar editors include a forecast-only terminal day; sampling uses
8 days but executes/statistically integrates 7. Solver quality is audited before
optimality labeling; limited/unproven complete scenarios are separately counted.
New hand oracle: next-day 180 MW minimum at 00:00 and 2 MW/min ramp causes D-end
150 MW carry and 17.5 MWh surplus. Demand compensation now changes correctly at
the two representatives: 397220 CNY, versus the old template-tail 396410 CNY
(difference 2*0.25*20*(90-9)=810). Final macOS Release rebuild passes Southern
27 cases / 11214 assertions and forecast 8 / 12503. Rebuilt ASan/UBSan Southern
`[lookahead]` passes 3 / 438 (leak detection disabled), no sanitizer report.
All four registered market browser suites pass (33.22 s); JavaScript syntax and
`git diff --check` pass. Multi-resource seven-day replay on port 8084 completed,
with day 7 using day 8 load 550 MW. Evidence:
`output/market-operation/lookahead-demo-evidence.json`; live desktop/mobile
screenshots `lookahead-live-*.png`. Older evidence below predates this correction.
Remaining solver work: process isolation and whole-day budgets; no 2000-node throughput claim.

Rule boundary organization now follows sections 2.3/2.4, using 15 backend-owned
categories with definitions, schema fields, entity counts and explicit coverage
limits. Daily typed `boundary_overrides` enter the real rolling solver and forecast
template; metadata/probabilistic offers are distinguished from physical inputs.
Raw bus forecasts are preserved through job creation, with proportional
reconciliation after daily edits. A read-only `preview` operation exposes authored
and effective boundaries; day node results now carry effective `load_mw` for Canvas.
Three new tests (336 assertions) cover admitted-field round trips, invalid overlays,
200 MW allocation 50/150 -> 100/100, and 40 MW nonmarket injection reducing generator
dispatch 100 -> 60 MW, including forecast templates. Release Southern passes
24 cases / 2035 assertions; rebuilt forecast passes 7 / 12486. Rebuilt ASan/UBSan
passes the new boundary tests (3 / 336; leak detection disabled as in the baseline).
All four registered market E2E suites pass, including real rule-field editing,
sampled-day preview, clearing, reload, atomic failure and mobile.
Remaining original-rule gaps include nonlinear reservoir/head curves, vibration
zones, fixed spill plans, level-rate bounds, full D+1 forecast ingestion and business
approval/forecast-material conversion. This is not complete rule certification.
The review server's multi-resource week also completed 7 days with day 2 authored
dispatch load 700 MW and reservoir inflow 150 m3/s. Both day-2 deficit and line
excess integrals were zero; the joint boundary restoration was valid. This is a
synthetic feasible boundary case, not evidence that every boundary edit creates
violations. Export: `output/market-operation/rule-boundary-demo-evidence.json`;
desktop/mobile hydro catalog screenshots: `rule-hydro-*.png` in that directory.
The following Canvas-only evidence predates this backend boundary extension.

Market Canvas synchronization now uses an isolated Southern topology in the
main viewport (`web/js/core/market_canvas.js`). Stable typed market IDs never
overwrite engineering Canvas state. Scenario/day/slot controls, daily following,
playback, node/line clicks, result table links, resource lookup and boundary
editor navigation are connected. Failed/pending/stale data are gray/unavailable;
restoration differences remain system conditional sensitivities. Large cases
show an explicitly bounded 80-bus neighborhood, with all devices selectable.
The four registered market browser suites pass with real Canvas interaction,
100 MW deficit color changes, 50/75 MW line excess, forecast/API value parity,
engineering same-ID isolation, mobile screenshots and 2000-bus last-ID picking.
Artifacts: `output/market-operation/overload.png`, `canvas-mobile.png` and the
existing Southern/forecast evidence directories. This frontend change does not
alter numerical models or backend schemas; no new C++/sanitizer or large-case
clearing claim is made.
The existing synthetic multi-resource forecast was replayed for all 3 paths /
21 daily windows. Live Canvas desktop/mobile checks passed; screenshots are
`output/market-operation/demo-canvas-{desktop,mobile}.png`. Scene 1, September 8,
00:00 shows line 1 at 1035.951 MW against +/-82.179 MW, with 953.772 MW excess.
These deliberately stressed synthetic results are diagnostic, not operating limits
that passed an AC security assessment.

Forecast extension: the operation page now defaults to probabilistic / interval
seven-day scenario generation. Seven factors include separate generation/storage
and controllable-load compensation offers. Gaussian copula/AR1 correlations,
interval stratification, independent weekly carry, per-slot/entity Delta P and
Delta Pij statistics, valid/unknown denominators, Wilson intervals and paired
restoration to daily prediction centers are connected through
`/api/session/market_forecast`. Whole-report export includes the base boundary.
Forecast centers multiply the baseline daily curve (and any authored daily
overrides); no claim of fitted real-world forecasts, node-specific errors,
15-minute random innovations or large-case ensemble throughput is made.

Release and rebuilt ASan/UBSan `test_market_forecast` both pass 7 cases / 12486
assertions. Uniform mean/variance are .981108/.342838 versus 1/(1/3), within
the preregistered .02 tolerance; AR1 correlation .579475 versus .6, within .1.
Two-path hand oracle: event fraction .5, mean weekly deficit 8400 MWh; an unknown
third path expands probability bounds to [1/3,2/3] without changing valid-only
means. Original Southern Release passes 21 / 1699. All four registered market
browser suites pass, including forecast statistics, export/reload, interval
semantics, atomic rejection, cancellation, cause restoration and mobile charts.
The rebuilt sanitizer rolling-operation regression passes 5 cases / 1250
assertions; no sanitizer report. Leak detection remains disabled as in the
existing macOS baseline.

The multi-resource forecast demo completes 3 weekly paths / 21 daily windows
(seed 42, seven triangular factors, load/wind latent correlation -.3,
AR1 .5, authored load/maintenance stress). Mean weekly deficit is
10962.6607637 MWh; mean node/line weekly peak excess is
495.0653368 / 952.3247118 MW. All three paths have events; Wilson95 is
[.438503,1], explicitly unsuitable as a calibrated real-world risk estimate.
Export and desktop/mobile screenshots: `output/market-forecast/demo-*`.

The market workflow now contains a Canvas/results split "运行模拟" page at
`/xjtu/#market-operation`. Its browser-driven task API runs sequential Southern
98-slot daily windows, carries realized slot 95 state, and reports the first
96 slots per day for a week or actual calendar month. Daily boundary factors,
interval outages, diagnostic deficit/surplus, signed line-limit violations,
conditional prices and one-factor restoration comparisons are connected end
to end. Pause/resume/cancel and stale revision rejection are explicit.
Each generated path uses rolling diagnosis with penalty slacks, not a full-horizon
joint optimum or AC certification. Startup/shutdown
power trajectories are rejected at rolling admission; daily storage targets
remain authored. Large-case week/month clearing has not been executed.

Current Release tests pass 21 cases / 1699 assertions. Predictions match:
100 MW deficit / 2400 MWh per day, 50 MW surplus / 1200 MWh, line tightening
adds 25 MW for one hour (25 MW h), 7 days = 672 slots, leap February = 2784
slots, delayed downstream reservoir ends at 195.5 m then 291.5 m.
Registered operation browser E2E covers week/month, restored-factor evidence,
nonzero line overload, reload/resume/cancel/stale revisions and desktop/mobile.
Generic market E2E expectation was updated for the new menu entry; it and the
Southern GUI suite pass. Rebuilt `macos-asan-ubsan` also passes all 21 cases /
1699 assertions with `ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1`.
The multi-resource demo completes seven days with all changed-factor restoration
runs valid; source boundary, results and screenshots are under
`output/market-operation/demo-*`. The deliberately extreme sixfold-load day
has 23227.5948004 MWh deficit and 22416 MW h line excess. This is a synthetic
stress experiment, not a forecast or a probability estimate.
Full input/output/limitation ledger: `docs/modules/market/southern_execution_contract.md`.

Earlier GUI entry baseline: `/xjtu/#southern-market` opens the workspace directly;
entering the five day-ahead stages also reveals it. The named case selector exposes the
analytic 100 MW case, a runnable 2-bus wind/solar/hydro/thermal/storage/DR demo,
and an ACTIVSg2000 research boundary. The latter validates at 2000 buses,
3206 branches, 1744 units including 720 hydro, 180 four-unit reservoirs in
three-reservoir chains, 80 storage and 120 interruptible loads. This repairs
the previous unconditional importer augmentation and invalid hydro renewable
fields; old claims that this large import already worked were incorrect.
The ordinary engineering importer no longer adds synthetic assets.

Before the operation extension, Release direct tests passed 16 cases / 445 assertions. Browser E2E exercises
the direct entry, named demo, shared-reservoir editor, storage bid save,
dispatch, node/branch results, mobile layout and the large boundary (including
the last reservoir). Screenshots: `output/southern-market/demo-boundary.png`,
`demo-results.png`, `demo-mobile.png`, `large-boundary.png`.
No full large-case clearing was executed. The forecast page now supplies
probabilistic/interval statistics and conditional restoration evidence.
`node_imbalance_mw` remains a numerical equality residual; diagnostic mode now
reports physical deficit/surplus penalty variables separately.
Line overload is now recomputed from signed flow limits. A new demo exposed
tiny SCED storage bound roundoff causing an inverted LMP intersection; projecting
the neighborhood center onto physical bounds fixes this without loosening the
1e-6 stage residual gate.

The rebuilt `macos-asan-ubsan` target also passes all 16 cases / 445 assertions
with `ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1`; unlike the old
11-case binary, this includes the new demo, importer and large-boundary checks.
Generic market GUI regression passes its expected DA `n1_security_failed`,
RT/hybrid `converged` and game `maximum_rounds_reached` fixtures.
The locally served default demo produces `schedule_only`, valid conditional
prices, and SCUC/SCED/LMP maximum residuals 8.53e-14 / 3.11e-7 / 2.84e-14.
Its SCUC/SCED objective is 460211.0600616677 CNY; LMP objective is
457911.73434347636 CNY. The day-only commercial schedule is not AC certification.

The worktree now adds the separate `market::run_southern_day_ahead_market`
entry point (`southern_market.hpp`, `southern_boundary.cpp`,
`southern_market.cpp`) and schema-driven GUI work area. The existing generic
market endpoints retain their own model. This is a research execution model
with declared A1--A7 interpretations, not completed unconditional reproduction
of the official market. The full input/solver/API/GUI ledger and open fidelity
limits are in `docs/modules/market/southern_execution_contract.md`.

Implemented boundaries include dispatch and bus forecasts with proportional
reconciliation, sourced nonmarket/external schedules, time-dependent generator
and transmission maintenance, unit/group limits and energy, reserve eligibility,
primary capacity, storage, HVDC hubs and commercial gateways, adjusted D-2
priority energy, renewable categories, and cascade reservoir control. Results
retain raw/effective boundary snapshots, solver stage status, objective terms,
original-unit row residuals, stable entity IDs, and baseline/scenario differences.
Session saves are atomic with revision conflicts rejected; engineering-model
replacement clears independent market snapshots. GUI supports scalar/series
and interval editing, import/export, save/reload, baseline/restore, all resource
result categories and AC feedback diagnostics at desktop/mobile widths.

SCUC, fixed-combination SCED and separate LMP are distinct builds. Sourced
external regulation preclearing awards alter SCED bounds; this is not an
implementation of the separate regulation-market bidding/optimization rules.
AC thermal/security violations add local sensitivity cuts and rerun the chain.
Security failure never publishes prices; `schedule_only` returns a conditional
linear schedule with `feasible=false`. Original-rule ambiguities, cross-midnight
startup trajectories, province-specific submission parameters, real market replay,
and scalable online validation remain open. Inputs starting inside a startup
trajectory are explicitly rejected. No regulator-certified equivalence is claimed.

Verified against workspace base `92c9c3b49c0dede3c090ee9d1750c745ebcc5086`
with these uncommitted changes, Apple arm64 / AppleClang, `macos-release`
(`-O3 -DNDEBUG`), and MIPSolvers `e003dbb11ae29b99e04cbcb1d70c2da02118d4dd`:

- `cmake --build build/macos-release --target test_southern_market run_gui_server -j4`
  succeeds. New market translation units enable `-Wall -Wextra -Wpedantic`.
  A pre-existing `DynamicSystem` class/struct forward-declaration warning is
  exposed through the public API header; the new source itself has no warning.
- `ctest --test-dir build/macos-release --output-on-failure -R 'Southern|southern_market'`
  passes 12/12 tests in 6.85 s: 11 numerical suites and registered real-Plotly
  browser/API E2E. Numerical suites contain 323 assertions, covering analytic
  prices/fees, forecast changes, validation, congestion and outages, startup
  classes/history/curves, reserve/group policy, regulation bounds, storage,
  reservoir lag/history, HVDC losses/hubs/directions, renewable/priority and AC
  feedback. Following the last CSS/evidence-export change, the browser E2E was
  rerun successfully. Its screenshots and complete numerical snapshots are in
  `output/southern-market/`.
- Predicted vs measured: 100 MW at 200 CNY/MWh gives D-day energy-bid cost
  480000 vs 480000 CNY, total with explicit representative weights 490000 CNY,
  and LMP 200 vs 200 CNY/MWh. +1 MW across the D-day gives +4800 vs +4800 CNY;
  the GUI one-slot +1 MW case gives +50 CNY and +0.25 MWh. Congested-node
  finite-difference price meets the fixed 1e-4 CNY/MWh threshold. Storage charging
  10 MWh at 0.9 efficiency adds 9 MWh. Reservoir levels match 99.9/90.4 m and
  delayed upstream inflow matches 100.5/101.5 m within 1e-6.
- `cmake --build build/macos-asan-ubsan --target test_southern_market -j4` and
  `ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 build/macos-asan-ubsan/tests/test_southern_market`
  pass all 323 assertions with no sanitizer report. `detect_leaks=1` was rejected
  by Apple ASan as unsupported, so this is not LeakSanitizer evidence.
- `node tests/e2e/market_gui_e2e.mjs` passes the existing workflow: its DA fixture
  intentionally reports `n1_security_failed`, RT/hybrid report `converged`, and
  the two-round game reports `maximum_rounds_reached`. This is a passing failure-
  handling contract, not a claim that this DA fixture was secure.
- JavaScript syntax and `git diff --check` pass. Real browser inspection of the
  served application confirms editable categories, converged analytic results,
  490000 CNY total objective and a nonblank 200 CNY/MWh price chart.
- The manual compiles to 61 pages. All 36 appended official pages remain text-
  identical after whitespace normalization; no overfull, missing-character or
  undefined-reference diagnostics. New authored pages were rendered and inspected.

The MIPSolvers checkout is clean but still differs from the recorded pin
`a39812aa5941691b44e8379a8e0b7d42ccdde955`; no dependency revision was changed.
No full project-wide C++ suite, external real-market oracle or province-scale
performance benchmark was run. The local review server is bound to
`http://127.0.0.1:8080/xjtu/` and seeded only with the synthetic analytic case.

## Southern regional day-ahead rules documentation (2026-09-05)

The market manual now uses section 2.6 of the official Southern Regional Spot
Energy Trading Implementation Rules (2025 V1.0) as its regulatory model baseline.
The complete 171-page attachment from Guangzhou Power Exchange is retained at
`docs/modules/market/references/southern_region_spot_energy_rules_2025_v1_0.pdf`.
Its SHA-256 is `dba60a0baaa3c296a3fd90047bfdb926fda1426893a397c7c4f69c6ea15b1b73`.
Printed pages 28--63 correspond to PDF pages 31--66; the manual includes these
36 original pages and an editable transcription of the SCUC objective and all
21 constraint clauses, plus the 18-clause SCED / 16-clause LMP correspondence.

`docs/modules/market/southern_rules_comparison.md` records the source-backed gaps:
98-point horizon, provincial positive/negative/primary-frequency reserves,
startup trajectories and three-state fees, unit groups, reservoirs, interprovincial
transactions, soft line/section limits, signed storage charging and hourly
direction restrictions, and a separate LMP solve with pricing eligibility and
dispatch neighborhoods. The official text has unresolved index, time-weight,
startup-state, priority-slack, reservoir-unit, and storage-pricing ambiguities;
the transcription retains these rather than silently selecting an execution model.

This is a documentation and reference-source change, not a production solver
upgrade. Existing market regression evidence is explicitly historical and does
not certify equivalence to these rules. C++ code, interfaces and numerical tests
are unchanged; no new numerical market run is claimed.

Verification: XeLaTeX/latexmk (TeX Live 2026) builds a 59-page manual at
`output/pdf/market_rules/market_manual.pdf`. From `docs/modules/market`, reproduce
with `latexmk -norc -xelatex -interaction=nonstopmode -halt-on-error
-outdir=../../../output/pdf/market_rules market_manual.tex`. Final checks confirm
all 21 SCUC, 18 SCED and 16 LMP constraint clause identifiers and the 2.6.6 price
formula, the source hash/page count, local module links, and whitespace-normalized
text equality for all 36 included official pages. No authored text extends beyond
the output pages, and the final log has no overfull boxes, missing characters or
undefined references. Authored pages and the referenced original pages were
rendered and inspected; original formula clipping/notation issues remain explicitly
documented. The shared-style package-name warning, underfull text boxes and ignored
glue-shrinkage diagnostics remain, without observed overlap. `git diff --check`
passes. These are document checks, not numerical rule-equivalence tests.

## Cyber-dynamic safe-restoration source audit bundle (2026-09-05)

The paper workspace now contains a source-backed audit bundle under
`docs/latex/paper/cyber_dynamic_safe_restoration/code/`. Its declared scope
expands to 147 files (3.5 MiB) while preserving repository-relative paths. The
bundle includes both paper drivers, complete dynamics and resilience modules,
hybrid component contracts, authored case builders, network reconfiguration,
certificate/resilience regressions, PSD and GridLAB-D validation code, build
registration, and the canonical runtime contract. `FILESET.tsv` declares the
scope, `MANIFEST.tsv` records SHA-256 hashes, `TRACEABILITY.md` maps manuscript
claims to symbols, and `PROVENANCE.txt` records both repository states.

`python3 .../code/audit_bundle.py verify` passes for all 147 source-backed
snapshot files. The `macos-release` targets
`cyber_dynamic_safe_restoration_small` and
`cyber_dynamic_safe_restoration_scale` rebuild successfully. The focused
`test_transient_dynamics '[resilience][certificate]'` regression passes 11
cases and 110 assertions. The numerical paper drivers were not executed during
this extraction because they overwrite the retained result directories; the
existing small, scale, PSD, and GridLAB-D artifacts were not regenerated or
reclassified.

This is an audit snapshot, not a standalone fork: root-project build support
and the sibling MIPSolvers repository remain external. The build above used
MIPSolvers `e003dbb11ae29b99e04cbcb1d70c2da02118d4dd`, while
`cmake/Dependencies.cmake` still records
`a39812aa5941691b44e8379a8e0b7d42ccdde955`; this mismatch remains an explicit
reproducibility limitation rather than a clean-pin publication baseline.

## Three-phase graph-reduced Native IPM and Ipopt warm starts (2026-09-04)

The Native phase-hybrid path now converges for H13, H34, H123, and H8500 under the
current line-limit setup. MIPSolvers enables MUMPS symmetric automatic scaling,
uses the congruence-scaled augmented inequality block `-I`, and in `Auto` mode
can fall back from exhausted condensed inertia correction to the algebraically
equivalent augmented Newton system. Forced `Condensed` behavior and all final
KKT tolerances are unchanged. The graph-reduced 20/100 early-restoration policy
also resumes the same primal-dual state with the remaining user budget when its
first cap is reached at an already primal-feasible point; restoration remains
reserved for a nonzero normal residual.

`ParametricWarmStartMode` defines the two initial-point methods used in the
comparison. `PrimalDual` uses the primal variables, equality multipliers, and
full-row inequality multipliers from a converged base OPF. Native IPM also uses
the saved slack variables, reconstructs them at the perturbed point, and projects
only multipliers outside 90% of the configured central neighborhood before the
independent Phase-II check. Ipopt instead receives variable lower- and upper-bound
multipliers through its TNLP callback and reconstructs its internal slacks.
`PrimalOnly` supplies the identical primal point without these multiplier data.
The MIPSolvers adapter converts the public `[inequalities | equalities]` order to
Ipopt's `[equalities | inequalities]` order, enables `warm_start_init_point`, and
rejects incomplete or invalid complete starts instead of silently reverting.

The fixed Release verification command is
`./build/macos-release/phase_hybrid_opf_benchmark all parambench native quiet`.
For each case it excludes one converged base OPF, applies a `+0.1%`
load perturbation, and alternates five primal-dual-slack/primal-only pairs on the
same graph-reduced model and enforced row set. Both sides use a 500-iteration limit
with the early restoration cap disabled. Every repeat must converge with
objective relative error, recovered-voltage error, and primal/dual/
complementarity residuals at most `1e-6`; each case also requires at least
`1.05x` median speedup, while the four-case result requires at least `1.20x`
geometric-mean speedup. Factorization count is retained as a diagnostic, not as
an acceptance criterion, because constraint generation, refinement, and terminal
KKT certification can add factorizations while reducing total runtime.

All four cases pass. Median primal-dual-slack versus primal-only results are H13
`1.778/10.067 ms` and `4/47` factorizations (`5.661x`), H34
`7.081/20.163 ms` and `7/27` (`2.848x`), H123 `25.570/35.359 ms` and
`20/30` (`1.383x`), and H8500 `970.395/1443.491 ms` and `131/100`
(`1.488x`). The geometric mean of the four speedups is `2.400x`. Across all 40
solve rows, maxima are `5.73e-9` objective relative error, `1.52e-8` p.u.
recovered-voltage error, and `1.35e-10/8.40e-7/1.85e-7`
primal/dual/complementarity residuals. Evidence
is `output/benchmarks/paper_native_ipm_parametric_warmbench.csv`, produced on
Apple M4 Max, macOS 26.5.2, AppleClang 21, arm64, dirty HySim `ef5f1f6b` and
MIPSolvers `a5d614b7`. This host differs from the paper's AMD case-study host,
and the result is not a clean-pin release baseline.

The Ipopt comparison uses
`./build/macos-release/phase_hybrid_opf_benchmark all parambench quiet` with the
same load perturbation, generated row sets, convergence checks, and five
alternating pairs. Median primal-dual versus primal-only results are H13
`2.064/3.522 ms` and `3/7` iterations (`1.707x`), H34 `4.400/7.178 ms`
and `2/5` (`1.632x`), H123 `6.451/12.664 ms` and `2/7` (`1.963x`), and
H8500 `274.799/743.846 ms` and `6/23` (`2.707x`). All four pass the fixed
`1.02x` per-case and geometric-mean gates; the geometric mean is `1.961x`.
Across all 40 rows, maxima are `4.44e-9` objective relative error, `1.31e-8`
p.u. recovered-voltage error, and `1.00e-8/9.98e-7/2.80e-9`
primal/dual/complementarity residuals. Evidence is
`output/benchmarks/paper_ipopt_multiplier_warmbench.csv` on the same dirty-worktree
Apple M4 Max environment.

The independent solver check is
`./build/macos-release/phase_hybrid_opf_benchmark all crosssolver quiet`. Native
IPM and Ipopt first screen rows independently; their row-set union then defines
the identical restricted NLP used by both final solves, and every omitted
physical inequality is evaluated. Native is forbidden from calling the optional
Ipopt initializer. Both solvers converge and satisfy KKT/physical feasibility on
all four cases. H13/H34/H123 meet the fixed `1e-6` objective and pointwise-voltage
agreement criterion. H8500 does not: its objective relative difference is
`1.915e-6` and voltage maximum difference is `5.994e-5` p.u.; the two nonconvex
solves are therefore recorded as nearby KKT points, not pointwise-identical
solutions. Evidence is
`output/benchmarks/paper_native_ipm_ipopt_crosscheck.csv`.

The English manuscript reports Ipopt 3.14.20 as its only nonlinear OPF solver.
Its title now uses `Constraint Generation` consistently with the method described
in the body. The abstract follows the problem--model--solution--verification order
of the reference draft and remains below 200 words. A four-part Nomenclature based
on the reference draft's IEEEdescription structure now precedes Section I and
defines the acronyms, sets and indices, parameters, variables, mappings, and KKT
multipliers used in the paper. The former Discussion section has been removed.
The full-width comparison table has been
restored between the Motivation and Literature Review to distinguish phase-domain
modeling, AC/DC coupling, converter models, reduction, and solution methods.
Repeated symbol descriptions in the body have been consolidated into the
Nomenclature. Section I.B now progresses from
unbalanced phase-domain OPF to hybrid AC/DC modeling, converter controls, Kron
reduction, and KKT computation; each cited work is assigned a specific method or
scope rather than being grouped under one generic claim. Section I retains
Motivation, Literature Review, and three concise Contributions, uses passive
constructions throughout its prose, and occupies less than two text pages before
Section II begins on page 3. Normalized display/float spacing and the naturally
balanced IEEEtran bibliography keep the manuscript at ten fully used pages.
Section IV presents one constraint-generation iteration: after restricted OPF
iteration `k`, the exact recovery relation from Section III identifies the rows
added at iteration `k+1`; the primal variables and multipliers from iteration
`k` initialize Ipopt for that next restricted OPF, with zero multipliers for new
rows. Restricted OPF calculation, constraint update, and Ipopt multiplier
initialization are combined in one Algorithm 1. The three case studies correspond
directly to the converter model, exact passive-phase reduction and recovery,
and computational performance. Case 1 now uses the physical H123 topology and
the converged all-GFM and mixed GFL/GFM Ipopt results. The mixed configuration
places GFM VSCs at buses 53 and 21 and GFL VSCs at buses 135 and 150; the figure
reports the resulting line active-power distribution, VSC phase active powers,
and converter-current VUF. The `controls` benchmark exports the topology, node,
and branch evidence to `paper_converter_topology_H123.csv`,
`paper_converter_node_results_H123.csv`, and
`paper_converter_branch_results_H123.csv`. Nonconverged equal-phase and all-GFL
rows remain in the audit CSV but are not plotted or cited as numerical evidence.
The compact performance figure is Ipopt-only and retains three panels:
inequality-Jacobian density, all five calculation times with medians, and the
four-case speedups with iteration reductions. Figure 1 uses the restored
three-panel TikZ/LaTeX version. Figures 2 and 3 are generated with MATLAB R2025b
at their final double-column sizes. Figure 2 uses a 9/8/7-pt Times New Roman
hierarchy for panel letters, titles/main labels, and numeric/detail labels;
Figure 3 retains 10-pt Times New Roman text. Their editable PDF/SVG, 600-dpi
PNG/TIFF, MATLAB source, and source-data CSVs are retained beside the manuscript.
Figures 1--3 place each combined panel label and title below its corresponding
panel, centered in black Times New Roman. Figure 3(c) vertically centers each
iteration-count transition within its bar so both the initial and final IPOPT
iteration counts remain visible for all four cases.
The H123 bus columns are forced to string during MATLAB import so auxiliary bus
names such as `25r` retain their transformer and feeder connections; only the
authored open-switch links at `300_open` and `94_open` remain open.

The Abstract, Section I.B, and Contribution 3 describe multiplier transfer in
words: the current restricted OPF solution initializes the subsequent IPOPT
calculation after a constraint update. The explicit `k`-to-`k+1` notation is
reserved for the mathematical development in Section IV.

Float declarations have been reordered to keep every figure and table near its
first formal reference. Table I is now cited explicitly at the end of the
Motivation. Figure 2 is declared before Case 1 so that the double-column float can
be placed on the immediately following page, Table II remains inside Case 2, and
Figure 3 is declared immediately before Case 3.

The manuscript no longer exposes internal solver options, callback/interface
names, or local program identifiers. The Case Description reports only IPOPT
3.14.20 and the public calculation environment. A fresh four-feeder OPF/PF check
was run with the Release benchmark; all OPF and PF calculations converge, with
maximum AC-voltage and converter-total-power differences of `7.88842665086e-7`
p.u. and `2.05967849381e-10` p.u. Evidence is
`output/benchmarks/paper_opf_pf_crosscheck.csv`, and the rounded manuscript values
are `7.89e-7` and `2.06e-10` p.u.

The GFM internal voltage is denoted by `v^{int}` and its phase value by
`v_phi^{int}` throughout the Nomenclature and converter equations. The symbol
`mathcal E` is reserved exclusively for the AC-line, DC-branch, and
converter-coupling edge sets; the unused `cE` macro has been removed.

Table II, Case 2, and the Conclusion now use the values in
`phase_hybrid_opf_case_audit_cold_oracle.csv` rather than an earlier manually
summarized table: the four-case maxima are `5.27725230529e-9` in relative
objective difference and `6.07869213013e-9` p.u. in recovered voltage, reported
as `5.3e-9` and `6.1e-9` in the manuscript.

The public release folder is
`docs/latex/paper/Graph Reduction for Parametric Unbalanced Hybrid AC-DC Optimal Power Flow/public_case_study_data/`.
It contains the four OpenDSS feeder inputs, explicit case and common parameter
tables, the benchmark-driver snapshot, Case 1--3 CSV outputs, the derived LaTeX
table, self-contained MATLAB R2025b programs and source data for Figs. 2--3,
vector/raster figure exports, provenance limitations, and per-file SHA-256
checksums. Nonconverged converter-control rows and diagnostic limit-omission rows
are retained but explicitly excluded from validation claims. Native-IPM outputs
are not included as manuscript evidence. Both copied MATLAB programs execute
successfully using only paths within the public folder.

Focused verification passes MIPSolvers augmented equivalence at 1 case/17
assertions, restoration budgeting at 1 case/4 assertions, the Native parametric
branch test at 1 case/11 assertions, and the complete HySim phase-hybrid target
at 15 cases/191 assertions. The MIPSolvers `test_engine_api` target passes
19 cases/92 assertions, including the Ipopt row-order, bound-multiplier, and
complete-start checks; `test_ipopt_parameter_stability` passes 7 cases/515
assertions. Full MIPSolvers `test_numerical_stability` passes
27 cases/495 assertions; `test_ipm_solver`, including the terminal active-set
crossover regression, passes 37 cases/263 assertions.
The English manuscript rebuilds to 10 fully used pages with no overfull boxes or
undefined references/citations. The OPF manual rebuilds to 180 pages; its
pre-existing long-source-identifier overfull boxes and font-substitution warnings
remain.

## N-k DAE restoration-entry compression pilot (2026-08-30)

The manuscript directory now contains a source-backed code audit bundle under
`docs/latex/paper/Progressive Class-Conditioned Frequency–Duration/code/`. Its
declared extraction scope expands to 137 files (5.1 MiB) while preserving
repository-relative paths. The manifest records a SHA-256 hash and audit role
for every implementation, test, input, and formal-result file; provenance
records both repository revisions and the dirty HySim worktree. The read-only
source-to-snapshot verification passes for all 137 files. The copied independent
oracle also passes the copied case33 report (552 conditions, 92 classes,
`2.220446049250313e-16` MWh/yr aggregation error, 100% covered leave-one-out
accuracy) and case123 report (1050 conditions, 173 classes,
`4.440892098500626e-16` MWh/yr aggregation error, 100% covered leave-one-out
accuracy). This is an audit snapshot, not an independently buildable fork or a
replacement for the production sources; common HySim infrastructure and
MIPSolvers remain external to the extraction boundary.

The manuscript theory now separates high-order event construction from
trajectory partitioning. Authored joint events form a canonical ordered subset tree;
for each fixed event, the nonempty inverse images of the complete DAE-to-MILP
interface form the unique coarsest lossless partition. The complete interface
retains initiating outages, DAE-derived VSC availability, and repair-stage
data, so equal VSC labels from different events are not merged. The manuscript
now states the exact event/class counts, the combinatorial boundary, the lack
of monotone parent-to-child class inheritance, and an EENS remainder bound for
future certified order truncation. The current code already uses the matching
`(event_index, class_label)` key and canonical combination recursion, but only
complete authored N-1/N-2/N-3 catalogs are numerically validated; the
higher-order stopping certificate is theory-only. Fig. 2 now has a canonical
MATLAB R2025b source showing these three distinct operations.
The registered `nk_acdc_reliability_study_smoke` and
`nk_acdc_reliability_oracle` tests pass 2/2 in 2.29 s. The oracle now also
checks the complete per-order combination counts, uniqueness of event subsets,
and preservation of event identity when two events share an unavailable-VSC
label. These tests cover the current authored N-1/N-2/N-3 path, not automatic
termination by the theoretical remainder bound.

The research driver `nk_acdc_reliability_study` now defines a trajectory class
only by the DAE-derived unavailable-VSC set actually supplied to the existing
hybrid AC/DC restoration MILP for a fixed initiating event. AC/DC load scales
remain condition-specific MILP inputs. The MILP's native `sP/sQ` active and
reactive load-shedding variables are retained and every projected future state
still incurs a restoration solve. The corrected cost model is
`N*(DAE+restoration)` versus `DAE library + N*(lookup+restoration)`.

A production defect, not a missing load-shedding feature, caused 150 false
case33 restoration infeasibilities. With `split_domain_trees=true`, tie
preprocessing used unified AC+DC+VSC connectivity and could prune a required AC
tie because a VSC-DC-VSC path joined its endpoints, although that path is absent
from the AC fictitious-flow and forest equations. Unified component pruning is
now skipped in split mode; per-domain forest cardinality removes cycles. The
focused regression passes 1 case / 2 assertions and the full
`test_reconfig_options` target passes 14 cases / 56 assertions.

The corrected formal case33 N-1/N-2/N-3 run has 92 explicit joint-entry events,
6 operating states and 552/552 resolved DAE-plus-restoration conditions, with
zero unresolved frequency. Static and DAE-entry EENS are
`0.0905561618570` and `0.260830925306 MWh/year`, a 65.2817% relative
difference. The 92 restoration-entry classes give a class/state ratio of
`0.166667`. Leave-one-operating-state-out nearest-state prediction has 100%
entry-class accuracy; re-solving the held-out state's load-shedding MILP gives
zero aggregate and weighted-absolute EENS error. At 100,000 projected samples,
two formal reruns give 5.76x and 5.99x end-to-end speedup and a 99.448% DAE
solve reduction after library construction. This demonstrates net saving, but
the pre-registered 20x gate fails; the earlier approximately 150x figure
omitted state-specific restoration and is invalid.

Registered smoke and Python-oracle CTests pass 2/2 in 2.64 s. `--audit-only`
changes only the process exit code so structural CTest execution can pass while
`validation_passed=false` remains in JSON. The Python oracle independently
recomputes frequencies, EENS, class grouping, leave-one-out outcomes and all
DAE/restoration timing identities from conditional records.

The formal case123 run completed in about 11 minutes. Its 175 N-1/N-2/N-3
events and 6 states form 1050 conditions; 892 reached and completed the existing
load-shedding restoration MILP, while 158 failed in the DAE or event-time
algebraic network solve. Unresolved frequency is `0.0319160406627/year`.
Static/DAE-entry EENS over only the resolved submeasure is
`0.0578626786707/0.235781116183 MWh/year`. Entry-class prediction is 100% where
a resolved neighbor exists, but `0.000761065845/year` has no leave-one-out
coverage, so class accuracy is not admitted. Two formal reruns give 8.81x and
8.91x timing projections, which are
diagnostic only because failed-library tails and resolved-only direct means are
not a common measure; the report now rejects timing/net-saving admission when
unresolved frequency is nonzero. This is a DAE closure failure, not a missing
restoration load-shedding capability. The formal case33 and case123 JSON files
both pass independent raw-record oracle recomputation.

The manuscript abstract, Introduction, Sections II--V, and conclusion now use
the same N-1/N-2/N-3 trajectory-class/restoration-entry-state story and follow
the structure and
writing pattern of the author's `TSG-01621-2022` paper. The Case Studies examine
transient necessity, restoration-entry-state identification, computational
performance, and large-feeder numerical closure. A four-panel mechanism figure
uses the actual case33bw AC-fault DAE trace to show the 0.36558-s VSC2 trip and
the resulting change from an initial-fault-only restoration input to the
initial fault plus VSC2; this representative trace is explicitly excluded from
the EENS sum. The case33 result is the admitted positive validation; case123 is
reported only as a DAE-closure diagnostic. Sections III--IV now formally define
the complete event-reset DAE trajectory, terminal unavailable-VSC map,
device-wise progressive partition, finite mutually exclusive classes, class
frequency conservation, state-specific EENS conservation, covered-range class
identification error, and break-even cost. The number of partition levels is
the number of retained VSC statuses rather than a fixed three. XeLaTeX produces
a 12-page PDF with five Python-generated figures, no overfull boxes, and no undefined
citations/references. All pages have been rendered for visual inspection; the
figures are legible and unclipped. The two propagation CTests and both formal
N-k report oracles pass. Auxiliary highlights and response-letter files still
contain historical TCERM wording but are not included by `main.tex`.

The manuscript formulation now expands the generic trajectory equation into
controller-state, AC-current-balance, and DC-current-balance blocks. It locates
configured GFL/GFM control states, FRT/current limitation, protection timers,
and the optional DC-link state explicitly; defines the VSC AC/DC power and
DC-link energy coupling; and states that DC-to-AC propagation requires an
authored DC-voltage-feedback, derating, or blocking path. Section IV records the
actual Backward-Euler, simultaneous damped-Newton, event-localization, reset,
and algebraic-reinitialization procedure. The restoration MILP remains outside
the DAE and is executed once after the pre-restoration interval. This
manuscript-only clarification does not change the formal numerical records.

The three Section V figures now have a single canonical MATLAB R2025b renderer,
`figures/generate_case_study_figures.m`. The script reads the audited figure
source CSVs and the formal case33 event record and exports vector PDF/SVG and
600-dpi PNG/TIFF files. The transient figure uses waveform evidence before the
restoration-entry comparison; the case33 figure uses paired point comparisons
instead of grouped bars; and the case123 figure separates condition closure,
unresolved frequency mass, and DAE-to-MILP attrition. The former Python plotting
scripts remain historical and are not the Section V canonical renderer.
The strengthened partition theory increases the final XeLaTeX build to 13
pages. The new Fig. 2 and its surrounding theory were rechecked on rendered
pages 4--7 at 180 dpi; Figs. 3--5 retain their previously verified MATLAB
rendering. Titles, legends, annotations, formulas, and captions are legible and
unclipped. The build has no overfull boxes or
undefined citations/references. The remaining warnings are IEEE-template font
substitutions and underfull spacing diagnostics.

The current clean sibling
MIPSolvers checkout is `a5d614b725b44e43d07c5fce0050b93a09f6d4d1`, not the
recorded `a39812aa5941691b44e8379a8e0b7d42ccdde955` pin, so these are dirty-main
research results rather than a pinned release baseline.

## Modified case33bw AC/DC propagation capability audit (2026-08-30)

The registered `case33bw_acdc_propagation_study` derives a dynamic study case
from `hacdcpf::io::build_case33bw_acdc()` without replacing its 33-AC-bus,
2-DC-bus, 37-AC-branch, 1-DC-branch and 2-VSC topology. It adds a 1 MW DC load,
explicit VSC schedules/ratings/current limits, and IEEE 1547 Category II
AC-terminal protection on the remote VSC. The driver compares a no-fault
trajectory, an AC-bus-18 shunt fault and a DC-bus-2 shunt fault at identical
initial conditions. Cross-domain propagation is accepted only from the maximum
deviation against the time-aligned no-fault trajectory, with fixed gates of
`1e-4 pu` voltage or `1e-3 MW` converter AC power.

The original GFL path was retained as a negative control: the DC fault changed
AC-bus-18 voltage by only `2.1082e-10 pu` and VSC2 AC power by
`1.2604e-10 MW`, below the predeclared gates. An opt-in average-value DC-link
fault control now maps measured DC voltage into a piecewise active-power
derating envelope and a definite-time latched undervoltage block. With authored
study settings `v_derate=0.95 pu`, `v_block=0.75 pu` and `delay=20 ms`, the same
DC fault changes AC voltage by `0.0880741146 pu` and VSC2 AC power by
`1.04999999994 MW`. The block times at 1/2/5 ms are `0.238/0.238/0.240 s`;
all three steps give the same propagation classification, the block-time spread
is `2 ms`, and the 1-vs-2-ms peak-power difference is zero. The AC fault still
changes DC voltage by `7.790176e-4 pu` and the binary-versus-DAE restoration
entry mismatch remains demonstrated. Maximum post-event algebraic residual is
`4.62e-14`.

The focused C++ test passes 49 assertions, including profile mapping/rejection, the analytical
first-order current derivative and 1/0.5-ms protection timing. An independent
Julia/Base oracle reruns the study and reconstructs the envelope, timer,
legacy identity and propagation gates from CSV without calling the C++ control
law; it passes with zero scale error and a `2 ms` block-time spread. Registered
CTest names are `case33bw_acdc_propagation_study_smoke` and
`case33bw_acdc_dc_fault_julia_oracle`. Full artifacts are
`build/macos-release/case33bw_acdc_propagation_report.json` and
`build/macos-release/case33bw_acdc_propagation_trajectories.csv`.

The admitted capability is bidirectional propagation only at the balanced-
phasor reliability-entry level: DC fault -> DC-link voltage collapse -> active-
power/current change -> optional latched block. IEEE Access 2022, IEEE TIA
2024, IEEE TPWRD 2024/2025 and IEEE TII 2025 evidence is compared in the
dynamics/reliability manuals. Those papers validate the physical mechanisms,
not this case's numerical values. In particular, the RTDS/CHIL diode-fed
uncontrolled-rectifier path reported by Pandey et al. is not represented.
MMC arm/submodule dynamics, converter saturation, switching ripple, DC
overcurrent protection, DCCB arcs and vendor deblocking/precharge remain
explicitly unsupported. These focused tests do not imply a full regression.
The updated dynamics and reliability manuals also compile successfully with
TeX Live 2026/XeLaTeX (88 and 115 pages, respectively); pre-existing manual-wide
box/font warnings remain and are not claimed as resolved by this focused work.

## Superseded transition-aware manuscript baseline (2026-08-30)

The Section V results below are retained as historical evidence only. They were
replaced in the current Case Studies by the formal N-1/N-2/N-3
restoration-entry studies described above and must not be cited as the current
manuscript validation.

Sections II--IV now use one explicit controller-to-trajectory contract. Local
converter outputs enter the continuous DAE flow; protection and IBR logic
create guards and physical resets; the distribution-level controller consumes
only delivered observations and applies its accepted switching/DER action
through a supervisory reset before the next DAE flow. Section IV was reduced
from roughly four pages to pages 7--8 and no longer duplicates a device-model
manual. Local converter control advances continuously, whereas system-level
restoration is solved only at finite decision epochs triggered by delivered
relay/IBR status, breaker feedback, communication recovery, or a restoration
timer. It may execute zero, one, or several times per contingency, never at
every DAE integration step. The generic mixed-detail DAE/restoration sequence
remains a method specification, not a claimed production implementation.

Section V was reorganized by validation question. Q1 uses the admitted
60-condition production DAE grid and shows that static restoration
underestimates mean 3-s event energy by 44.680% at a 0.25-s restoration delay.
Q2 performs 2000 seeded independent replay replications at
$N=60,600,6000$; observed-to-predicted MC RMSE ratios are 1.018, 0.988, and
1.006, while exhaustive TCERM conditional means reconstruct the direct
trajectory average to displayed zero. Replay is statistically equivalent to
rerunning identical deterministic inputs, but only DAE solve counts are
compared; no replay wall-clock speedup is claimed. Q3 now uses a modified IEEE
33-bus hybrid AC/DC feeder with 33 AC buses, two DC buses, 32 closed AC lines,
five normally open ties, one DC line, and two VSCs. Its 32 N-1 line outages and
18 operating states give 576 direct calculations and 84 equivalent states.
Static/direct/equivalent EENS values are `1.4382093024`, `1.4953717429`, and
`1.4953717429 MWh/year`; the static calculation underestimates the
transition-aware result by `3.822624%`. This is a disclosed reduced consequence
study, not feeder-wide DAE validation or a utility forecast.

`generate_case_study_validation.py` is the canonical generator for the CSV and
table evidence and the three-panel Python Fig. 3. Panel (a) tests transient
necessity, panel (b) compares MC sampling error and required DAE solve count,
and panel (c) reports the IEEE 33-bus static/transition-aware EENS comparison
and equivalent-state closure. The standalone C++ project independently reads
the Baran--Wu MATPOWER AC network and reproduces the same 576/84 dimensions and
EENS values; its direct/equivalent difference is `4.4408920985e-16 MWh/year`.
The predeclared numerical acceptance gates passed. The standalone Release
project rebuilt and `pccfd_tests` passed 1/1 in 1.67 s. The final TeX Live 2026
build is 12 pages with no overfull boxes,
undefined citations/references, duplicate labels, package warnings, or fatal
errors. All 12 rendered pages were visually checked: Section IV occupies pages
7--9, Section V starts on page 9, and Figs. 1--3, Algorithm 1, and Tables I--II
are legible and unclipped. The final PDF is stored beside the manuscript sources;
the full HySim regression was not run and is not implied.

The theory-first prediction was 32 contingencies, 576 direct calculations, 84
equivalent states, and less than 2 s for the IEEE 33 calculation. The measured
values are exactly 32/576/84 with a `5.2e-05 s` core aggregation time in the
formal 500,000-year reproduction run; no re-derivation trigger occurred. A
fresh AppleClang Debug build with AddressSanitizer and UndefinedBehaviorSanitizer
also passes `pccfd_tests` 1/1 in 2.43 s.

## Converter and protection deterministic failure models (2026-08-28)

The phasor-domain dynamic devices now separate failure occurrence from failure
consequence. GFL/GFM converters and their VSC wrapper accept the deterministic
`None`/`ForcedBlock` control state; a block freezes control states, removes AC
and DC stamps, suppresses protection reconnect, and reports the failure mode,
control availability, block state and actual terminal-current injection. The
rich VSC dynamic profile admits the explicit numeric parameter
`converter_forced_block` (exactly 0 or 1). Its healthy default follows the
pre-existing control path without an added DAE state.

`ProtectionRelay` now accepts `None`, `FailToTrip` and `SpuriousTrip`. A
fail-to-trip retains local CT/PT measurement, pickup margin and timer but
records that its output command was suppressed; a spurious trip bypasses the
guard and emits exactly one event carrying the failure mode. Static nonnegative
`pt_ratio_gain` and `ct_ratio_gain` act before the existing exact ZOH phasor
filters. These inputs represent deterministic IEC 61869 ratio/channel
consequences, not calibrated failure rates, CT saturation, EMT waveforms,
breaker mechanics, communication delay or backup-relay coordination.

Verified on the existing `macos-release` build: `test_transient_dynamics`
rebuilt successfully. The three new focused cases pass 3 cases / 77 assertions:
healthy converter identity is exact, forced GFL/GFM AC/DC injections and
derivatives satisfy the `1e-12` gate, fail-to-trip emits zero actions, and
spurious-trip emits exactly one. The PT-gain DAE case has no healthy action,
while `g_PT=0.8` acts at 0.025 s for both 10 ms and 1 ms nominal steps with a
localized bracket no larger than `1e-6` s and post-event algebraic residual no
larger than `1e-8`. Four affected pre-existing cases also pass 4 cases / 183
assertions (COSMIC relay equations, relay DAE localization, IEEE 1547
trip/reconnect, and VSC limiter mapping). A full CTest regression and external
EMT/field validation were not run and are not implied.

## Hybrid AC/DC terminal-state reliability pilot (2026-08-27)

The production mass-matrix DAE now has a registered research driver,
`terminal_state_boundary_study`, for the networked-microgrid AC-bus-6 fault.
The driver reports IEEE-1547 trip, terminal blocked state, reconnect and
return-to-full-power times separately; unresolved reliability consequences are
serialized as null rather than zero. It also records that design-grid fractions
are not calibrated field probabilities and that the present `VSCTrip` model
does not cover MMC valve blocking, DC overcurrent blocking, precharge, or
vendor unlock logic. `terminal_state_boundary_study_smoke` is registered in
CTest.

The pilot exposed and corrected a production mapping omission: GFL VSC dynamic
assembly now preserves `VSCConverter::current_limit_priority` as magnitude,
active-power-priority, or reactive-power-priority limiter geometry, matching
the existing GFM branch. The focused regression `GFL VSC builder preserves
authored current-limit priority` passes 1 case / 9 assertions.

The admitted numerical result is limited to the 3 s near-fault window. The
0.625 and 0.3125 ms full 4x5x3 grids both complete 60/60 cases, agree on every
terminal class, and differ in trip time by at most 2.976 ms (mean 0.600 ms over
21 trip cases). The uniform-grid protective-trip and terminal-blocked fractions
are both 0.35; mutually exclusive counts are 24 ride-through without cessation,
15 ride-through with momentary-cessation diagnostic, 12 undervoltage trips and
9 overfrequency trips. Each limiter priority produces 7/20 trips, so this case
does not show a reliability-class effect from priority choice.

The admitted 3 s window now includes an explicit nested static/transient ENS
comparison for network-restoration delays 0.25, 0.5, 1.0, 1.5 and 2.0 s. At
0.3125 ms the uniform-grid mean static underestimates are 44.680%, 26.677%,
11.539%, 4.459% and 0%; the maximum difference from 0.625 ms is 0.0093
percentage points. Every one of the 39 ride-through cases has zero incremental
ENS, every scenario satisfies transient ENS greater than or equal to static
ENS, and the static model meets the conditional 5% gate only at 1.5 and 2.0 s.
These are per-event consequences inside the finite observation window, not
annual EENS estimates.

The driver also reports the executable analytical reduction
`mean_static_ens + p_trip * L_dc * conditional_trip_unavailable_time / 3600`.
It reconstructs direct trajectory averaging to the fixed `1e-12 MWh` gate.
This establishes the training-grid identity and the static model's nested
limit; it does not validate interpolation to unseen fault/operating points or
replace the need for calibrated scenario probabilities.

Long-horizon reconnect evidence is explicitly rejected. For the representative
0.01 pu, 0.30 s fault, 5 ms backward Euler reports reconnect/full power at
18.56568/20.56568 s, 2.5 and 1.25 ms do not reconnect by 25 s, and 0.625 ms
fails the AC-voltage health gate at 5.488125 s. The corresponding 5 ms static
EENS boundary candidate is retained only as a rejected diagnostic; no actual
reconnect time or static-reliability applicability boundary is claimed. The
MATLAB source `make_terminal_state_boundary_figure.m` renders the admitted and
rejected evidence separately.

This work used HySim base `8712048e58d2a0dd3edd139a73be9435038a6ea9` plus the
documented dirty-worktree pilot changes and clean MIPSolvers
`a5d614b725b44e43d07c5fce0050b93a09f6d4d1`. Configure succeeds with the
existing warning that the recorded dependency pin is still `a39812a`.

Verified on `macos-release`: `terminal_state_boundary_study` rebuilds, its
registered smoke passes in 1.48 s, the GFL priority mapping regression passes
1 case / 9 assertions, the existing networked-microgrid outage transient passes
1 case / 3207 assertions, and the online-protection DAE regression passes 1
case / 31 assertions. The dynamics manual compiles with XeLaTeX to 86 pages;
the initial generic TeX Live attempt selected pdfLaTeX and failed on the CTeX
font engine, after which the explicit XeLaTeX run succeeded. MATLAB R2025b
regenerated the three-panel diagnostic and the admitted two-panel finite-window
comparison PDF/PNG; both were visually inspected for legibility and clipping.
`git diff --check`, the untracked-source whitespace
check, and the forbidden-marker scan are clean. `clang-format` is not
installed on this host, so no formatter command was run.

## Transition-aware reliability manuscript (2026-08-28)

The IEEEtran manuscript under
`docs/latex/paper/Progressive Class-Conditioned Frequency–Duration/` is now
titled *Transition Aware Reliability Assessment of Hybrid AC/DC Distribution
Systems with Inverter Based Resources*. The paper now follows the complete
detailed-generator--equivalent--analytical-embedding logic of Omri et al.,
*IEEE Transactions on Power Systems* 39(5), 6319--6331 (2024), DOI
`10.1109/TPWRS.2024.3354299`, after reading its full IEEE Xplore interactive
HTML. Omri et al. use sequential Monte Carlo, day-ahead scheduling, an islanded
resource-management MILP, apportioning, and fictitious multi-state DGs to
compress microgrid export capacity. The present paper instead defines a
transition-conditioned equivalent reliability model (TCERM) for the earlier
joint protection--IBR--restoration path. Singh and Billinton,
*IEEE Transactions on Reliability* R-24(1), 31--36 (1975), is retained only as
the classical frequency--duration foundation.

The TCERM is formulated as a semi-Markov renewal--reward kernel retaining entry
frequency, holding time, loss-set boundaries, duration-weighted load shed, and
customer interruption rewards. The initiating-contingency frequency is allocated
among mutually exclusive paths and is not reused as an IBR failure rate. Exact
conditional means preserve linear frequency--duration rewards. A
frequency-weighted within-class energy range bounds terminated refinement, and
an adaptive allocation rule prioritizes classes by removable bound and event-
solve cost. The manuscript additionally derives a total fast-assessment error
decomposition, decision-separation certificate, nested static limit, signed
extension, and a conditional stress-hazard interface. The hazard interface is
disabled in all numerical cases because its baseline hazards and stress
coefficients are not calibrated.

Section III.B is now titled ``Trajectory-to-Class Mapping and Progressive
Partitioning'' and is organized into three subsubsections. The first generates
a certified hybrid fault-to-service trajectory with the coupled mass-matrix DAE
and measurable guard/reset maps. The second maps only declared finite records
to disjoint exhaustive preimage classes. The third defines nested parent--child
refinement, class probability/frequency allocation, the semi-Markov class
record, and conditional-expectation reward conservation. The previous detailed
GFL controller expansion, timer equations, information filtrations, auxiliary
well-posedness/quotient propositions, state-complement discussion, and repeated
reward-sufficiency text were removed from III.B.

Section III.B now uses one integrated Python-generated 183-by-102-mm Fig. 2
instead of separate trajectory-mapping and progressive-partition figures. Its
four panels follow the subsection exactly: coupled DAE/guard/reset trajectory
generation, finite record-to-exact-preimage mapping, device-indexed nested
refinement, and initiating-frequency/linear-reward conservation. For one fixed
initiating event it shows distinct trajectories with equal retained records,
an explicit unresolved class, unique parent--child relations, and allocation of
one initiating exposure. Observation, classification, and parent-map arrows are
explicitly not physical state transitions. The canonical source is
`figures/generate_trajectory_class_mapping.py`; editable PDF/SVG and 600-dpi
PNG/TIFF exports are generated exclusively with Python and pass text-collision
and figure-boundary audits.

The rewritten III.B keeps only the distinctions required by the method:
$(\mu,x,y)$ is one hybrid DAE state on a flow segment, $\rho_{k,\omega}$ is the
ordered flow/reset trajectory, $\mathfrak o_{k,\omega}$ is its finite observed
record, and $C_L(k,\omega)$ is the resulting class label. Failed
initialization, singular/nonunique DAE continuation, unresolved simultaneous
events, and failed post-event reinitialization map to $\rho_\bot$. Finiteness
comes from finite record alphabets and a bounded record schema; a finite time
horizon alone is not asserted to bound event count. The discrete hybrid mode
uses $\mu\in\mathcal M$; $q\in\mathcal Q$ remains reserved for an
information/automation function.
The implemented Levels 0--2 are causal fidelity tiers, not controller counts.
Protection, information-function, IBR, event-order, and service-stage outcomes
are now device-indexed finite power-system state groups. Adding devices
normally enlarges a group and the reachable equivalent-state set. Level 2 is sufficient
only for the authored model scope and only when the Section IV residual-
consequence and decision-separation gates pass; otherwise the equivalent-state variables must be
enlarged or additional nested levels introduced.
The paper now defines the initiating-event catalog at its first use in Section
II.B through an exact physical state entry
$k=(\Phi_k^-,\Delta\Phi_k)$, with post-entry outage set
$\Phi_k^+=\Phi_k^-\cup\Delta\Phi_k$. Section III.A is now titled
``Initial Failure'' and now contains exactly two paragraphs. The first states
the evaluated $\mathcal K^{N-1}=\{(\varnothing,\{a\}):a\in\mathcal U\}$ scope;
the second gives the cited stationary two-state conversion from an up-state
failure intensity to a calendar entry frequency and the hybrid fault jump.
Directly reported annual outage frequency is not converted a second time.
Sequential-overlap and simultaneous/common-cause outage
entries require chronological/multi-state or joint-event models; marginal
annual rates are not multiplied. The reported numerical campaign remains
explicitly limited to $\Phi_k^-=\varnothing,|\Delta\Phi_k|=1$. Protection non-operation, breaker
failure to open, IBR trip, failed islanding and restoration remain downstream
conditional outcomes and never receive a second initiating frequency.
Section III.C now states that all annual indices are conditional on the authored
catalog. Excluded higher-order outages and unmodeled joint-entry mechanisms are reported
as catalog truncation and are not absorbed into the within-event unresolved
class.
The standalone manuscript project removes both its unused failure-on-demand
helper and the former independent overlapping-outage utility so that the executable
scope matches the manuscript. `calendar_frequency()` retains the fixed
two-state N-1 oracle at the declared $10^{-15}$ absolute tolerance. No
sequential-overlap or simultaneous/common-cause occurrence model is
implemented or numerically validated. After this scope correction, the
standalone Release project rebuilt and its registered `pccfd_tests` test passed.
The current final build and test evidence is recorded at the start of this
status entry.

Section II and Fig. 1 now state the controller boundary before Section III
begins. Subsection II.A is titled ``Hybrid AC/DC Distribution Systems with
Hierarchy Controllers'' and contains exactly four paragraphs: physical network
boundary, asset-local converter control, protection incidence and authority,
and information-limited supervisory control. Subsection II.B is titled
``Reliability Assessment Scheme and Assumptions'' and contains exactly two
paragraphs: the event-to-index assessment chain and its assumptions/evidence
boundary. This moves the initiating-event catalog, scenario, execution record,
TCERM allocation, and reliability outputs out of the controller architecture
and into the assessment scheme. The first paragraph now follows the visible
Fig. 1(b)--(c) sequence in short steps: initiating contingency, annual
frequency, operating scenario, post-contingency simulation, finite
event-and-service record, consequence class, and reliability indices. The
second paragraph groups the scope into four explicit assumptions instead of
interleaving definitions, exclusions, and validation claims.

The manuscript distinguishes the component-level compact GFL converter
controller, component-level network/interconnection protection, and the
system-level supervisory isolation/restoration optimization. The first two
layers share the fault-on DAE voltage/current trajectory; the supervisory layer
subsequently consumes only the telemetry, status contacts, protection events,
and source certificates delivered by the authored information state. The new
formulation defines asset-local converter ownership and protection incidence
through protected-zone, measurement, and commanded-interrupting-device sets;
protection is not incorrectly forced into a one-to-one component mapping.
The supervisory observation map is non-injective, so different physical states
may be observationally indistinguishable. Missing information removes remote
action/source credit or enters unresolved mass rather than exposing simulator
truth. No general state estimator or POMDP is claimed for the paper results.
The supervisory layer remains explicitly distinct from a differential
controller or transient optimal-control problem solved simultaneously with the
DAE. Section IV now interprets switch executability as requiring both the
necessary status observations and command path, and source qualification as
reported evidence rather than a hidden-state lookup. The figure is anchored so
that the IEEE two-column layout places it before Section III, and the Section II
evidence boundary states that the 60-condition campaign validates compact
GFL/IEEE 1547 consequences conditional on scheduled clearing rather than
endogenous relay clearing or a detailed GFM transient model.
The Python-generated 183-by-88-mm Fig. 1 now mirrors Section II directly.
Panel (a) contains the $G_0$ physical boundary and the three parallel
measurement--control--equipment relations for local converter control,
protection, and distribution-level restoration. Panel (b) now contains the
contingency occurrence model: exact pre-contingency/new-outage sets, annual
occurrence frequency, operating scenario, the reported N-1 evidence boundary,
and the distinction between post-contingency outcomes and independent
contingencies. Panel (c) continues with the post-contingency trajectory,
restoration-command requirements, reported protection/IBR/service outcomes, one
TCERM equivalent reliability state, frequency conservation, and six reliability indices. The
labels use distribution-system, relay, SCADA/IED, switching, and DER-dispatch
terminology rather than abstract authority/admission language. Its collision
audit compares text boxes across
panel boundaries as well as within a panel. The Section III.A cross-reference
to the initiating-event catalog now points to II.B rather than the obsolete
II.A location. The rebuilt `research_object.pdf` and the 12-page IEEEtran manuscript compile
with TeX Live 2026 without overfull boxes, undefined citations/references,
duplicate labels, or fatal errors. Rendered pages 3--6 were inspected at
publication scale: Section II.A introduces the hierarchy on page 2, Fig. 1 is
at the top of page 3, Section II.B follows beneath it, and the compact N-1
frequency and fault-reset equations remain inside their columns on pages 3--4.
The loss-boundary equation was split into three aligned rows, removing
the previous literal `qquad` rendering defect. The merged Fig. 2 is on page 6;
there is no longer a separate progressive-partition Fig. 3 or double-column
figure stacking. Only nonfatal underfull diagnostics from the IEEE layout
remain. The standalone manuscript Release test passes 1/1; no full HySim
regression is implied by this paper-focused change.

The 60-condition detailed campaign is now tied to the controller actually
authored by `terminal_state_boundary_study`: compact phasor-domain GFL PLL,
algebraic P/Q-to-dq reference conversion, magnitude/active/reactive-priority
current projection, first-order current states, algebraic AC/DC network, and
IEEE 1547 event logic. Differential LCL, inner-PI, switching-device and vendor
firmware states are not activated by that driver and are not validated by its
terminal-class or finite-window energy results. The driver applies an authored
scheduled clear event at each selected duration; it does not derive clearance
from an endogenous relay guard. Its evidence is therefore conditional on
clearing time and does not validate the closed-loop relay-clearing model.

The evidence layers remain separated. The 60-condition mass-matrix DAE campaign
supports terminal-class and 3-s finite-window event-energy claims only. Its
seeded MC replay validates the estimator's sampling error and required solve
count, not transient wall-clock acceleration. The modified IEEE 33-bus hybrid
AC/DC case uses the same disclosed reduced protection/FRT/restoration equations
for direct and equivalent-state aggregation and supports the reported EENS
comparison, probability closure, and branch ranking. The deterministic detailed grid is not a field probability
model, `21/60` is not an annual trip probability, and the nonconverged
long-horizon reconnect tail remains rejected.

The manuscript README, Highlights, cover letter, standalone C++ audit, and
equation traceability use the same title, TCERM framing, and evidence boundary.
The former large-feeder generator and audit were replaced by the modified IEEE
33-bus application. Its Python and standalone C++ calculations reproduce the
576 direct calculations, 84 equivalent states, and transition-aware EENS of
`1.4953717429 MWh/year` within the declared `1e-12 MWh/year` aggregation gate.
Generator root discovery and documented reproduction paths are verified;
the full production regression is not implied by this manuscript-only rebuild.

## Windows Release and distribution baseline (2026-08-25)

The annual and plain time-series daily-parallel admission contract now permits
in-service stationary AC, legacy DC, and rich DC storage. Every independent day
forces terminal SOC back to the authored initial SOC, and the annual result
certifies a maximum boundary residual of `1e-8`; missing trajectories or larger
residuals are infeasible. The result scope explicitly excludes inter-day and
seasonal storage energy transfer. Annual SCUC/DynamicSCED and enabled mobile
storage remain on the coupled sequential path because commitment/ramp and
travel/SOC boundary states are not yet exchanged between days.

Verified on `windows-msvc-release`: the affected `test_nighttime_opf`,
`test_multiscale_comprehensive`, and `run_gui_server` targets rebuilt
successfully. `test_multiscale_comprehensive "[integration][time_series]"`
passed 6 test cases / 64 assertions, including the new annual two-day cyclic
SOC admission/rejection test. `test_nighttime_opf "[time_series]"` passed 1
test case / 28 assertions, including a 48-hour rich-DC-storage parallel run
whose terminal SOC matches the authored value at both day boundaries. The
updated time-series manual also compiles successfully with XeLaTeX (52 pages).

The `windows` HySim branch pins the clean sibling `../MIPSolvers`
`windows-hysim` branch at `a39812aa5941691b44e8379a8e0b7d42ccdde955`.
`cmake --preset windows-msvc-release` followed by
`cmake --build --preset windows-msvc-release --clean-first` completed with
MSVC 19.44. The full-edition `run_gui_server.exe` is built with embedded Ipopt,
static sequential oneMKL/PardisoMKL, CHOLMOD/UMFPACK/KLU and packaged
HiGHS/SCIP; Gurobi is disabled for the distributable preset. `dumpbin` reports
only Windows system DLLs and redistributable VC++/OpenMP runtime dependencies.

After replacing unsafe Node `import.meta.url.pathname` conversions, the seven
affected registered browser E2E tests pass 7/7. The full
`windows-msvc-release` CTest sweep registered 1694 tests and completed in
317.52 s: **1656 passed, 5 skipped, 33 failed**. The failures remain explicit:
four tests expose the same strict PardisoMKL/Ipopt hybrid-OPF convergence
mismatch; twenty require ignored/untracked external fixtures absent from this
checkout; seven require working GridLAB-D external comparisons; `case16am` PF
does not converge; and graph-reduced Native Full OPF stops at its unchanged
strict accepted-step gate (`p=8.008e-07`, `d=0.289586`, `c=0.1`). No backend
was disabled and no OPF assertion or residual gate was relaxed to improve the
count. The Windows package gate therefore verifies solver capabilities plus a
standalone full-edition API/frontend startup smoke; it does not claim that the
full CTest sweep is green.

## Authoritative Release baseline confirmed (2026-08-24)

The sibling `../MIPSolvers` benchmark worktree that had blocked the strict
reproducibility guard is clean again (HEAD `a5d614b`), so `build/macos-release`
was reconfigured and rebuilt, and the full Release CTest sweep passed
**1698/1698 in 54.3 s**. This supersedes the "Release run pending" notes in the
oracle sections below — all five session additions now pass in Release:
`doc_anchor_check` (#1666), `carbon_analysis_cross_validation` (#1670),
`reliability_resolver_cross_validation` (#1671), `market_sced_cross_validation`
(#1672) and `hosting_capacity_cross_validation` (#1673). The registered count
rose from 1690 → 1698 across the session (+3 time_series fixtures via
`catch_discover_tests`, +1 `doc_anchor_check`, +4 module oracle tests).

## analysis hosting-capacity independent oracle (2026-08-24)

`analysis` (hosting capacity) gained the same independent-oracle cross-check
(logged as AUD-088), completing the internal-only quantitative modules. The
DL/T 2041-2025 equipment-level hosting capacity is a clean algebraic closed form:

- `tools/analysis_validation/validate_hosting_xref.cpp` — runs the production
  `assess_hosting_capacity` on four deterministic single-transformer supply areas
  and dumps the supply-area aggregates, transformer parameters and the
  hosting/accessible-capacity figures.
- `tools/analysis_validation/run_cross_validation.py` — a numpy-free oracle that
  does **not** link hacdcpf; it re-derives `S_d = max(0, (P - P_G + beta*n*S*cos
  + P_ESS + dP_ESS)/tau)` and the accessible-capacity subtractions
  (`C_grid = S_d - existing_DR`, `C_reg = C_grid - registered_DR`), with case 1
  the hand-verified analytic anchor from `test_hosting_capacity` (S_d in
  [10.5, 12.5], accessible 9.5 / 7.5).

Registered as the CTest test `hosting_capacity_cross_validation`. It passes with
a worst error of `0` across all figures on four cases (auto-beta single 18.2,
parallel auto-beta 10.4, storage interval clamped to [0, 3]) at a `1e-9` gate; a
negative control (rewriting one S_d,max) fails as expected; the analysis manual
recompiles with XeLaTeX. Verified in the Debug ASan build, with the Release run
pending a clean `../MIPSolvers` worktree.

This completes independent equation oracles for all four internal-only
quantitative modules this session — `carbon_analysis` (AUD-085),
`reliability` (AUD-086), `market` (AUD-087) and `analysis` (AUD-088) — each
following the emitter + numpy-free oracle + CTest pattern of
`integrated_energy`/`model`/`power_models`.

## market SCED/LMP independent oracle (2026-08-24)

`market` gained the same independent-oracle cross-check (logged as AUD-087). The
deterministic slice is the fixed-commitment SCED price and settlement on a
single-bus copper plate, where the locational marginal price and revenue
identities have exact closed forms:

- `tools/market_validation/validate_market_xref.cpp` — runs the production
  `run_day_ahead_market` on two copper-plate cases (linear-cost units, marginal
  20 and 40 currency/MWh, 100 MW each, at demand 60 and 140 MW) and dumps the
  offers, demand, LMP, dispatch and settlement ledger.
- `tools/market_validation/run_cross_validation.py` — a numpy-free oracle that
  does **not** link hacdcpf; from the offers and demand alone it re-derives the
  merit-order dispatch and the marginal (price-setting) offer — the uniform LMP
  on a lossless bus — and independently checks the revenue-adequacy identities
  (resource energy revenue = sum_g LMP_bus(g) * dispatch_g, customer energy
  payment = LMP * demand, zero single-bus congestion rent, zero cash-flow
  residual).

Registered as the CTest test `market_sced_cross_validation`. The production
clearing matched the analytic merit order exactly (demand 60 -> LMP 20, dispatch
(60,0), payment 1200; demand 140 -> LMP 40, dispatch (100,40), payment 5600);
the oracle passes with a worst error of `0` across nine checks at a `1e-6` gate,
a negative control (rewriting one LMP) fails as expected, and the market manual
recompiles with XeLaTeX. Verified in the Debug ASan build, with the Release run
pending a clean `../MIPSolvers` worktree.

## reliability parameter-resolver independent oracle (2026-08-24)

`reliability` gained the same independent-oracle cross-check as `carbon_analysis`
(logged as AUD-086), targeting the deterministic core that every method (NSQ/SEQ
Monte Carlo, FMEA, three-stage) shares — `resolve_reliability_params`:

- `tools/reliability_validation/validate_reliability_xref.cpp` — a C++ emitter
  that runs the production resolver on eight deterministic failure-mode inputs
  covering every conversion branch (lambda+MTTR on an operating or calendar
  basis, legacy MTBF with either convention, explicit MTTF, forced-outage-rate
  with and without repair time, and active-on-demand) and dumps the raw inputs,
  data policy and resolved canonical parameters.
- `tools/reliability_validation/run_cross_validation.py` — a numpy-free oracle
  that does **not** link hacdcpf; it re-derives the Billinton & Allan
  alternating-renewal closed forms (`U = lambda/(lambda+mu)`, `mu = H/r`;
  `lambda = f/((1-f) r) H`; the calendar-basis correction; `lambda_active = nu
  p_d`) and checks lambda, repair time, unavailability, MTTF and the
  calendar/active-equivalent frequencies.

Registered as the CTest test `reliability_resolver_cross_validation`. It passes
with a worst error of `0` across all six parameters on the eight cases (the
independent re-derivation is bit-identical to the production resolver in IEEE 754
double) at a `1e-9` gate; a negative control (perturbing the FOR-branch lambda by
1%) fails as expected; the reliability manual recompiles with XeLaTeX. Like the
carbon oracle it was verified in the Debug ASan build, with the Release run
pending a clean `../MIPSolvers` worktree.

## carbon_analysis independent equation oracle (2026-08-24)

`carbon_analysis` was internal-regression-only (its `test_carbonflow_*` suites
are self-consistency checks, and the module chapter admitted "no independent
linear-algebra oracle"). It now has the same independent-oracle cross-check that
raised `integrated_energy`/`model`/`power_models` to a higher evidence tier
(logged as AUD-085):

- `tools/carbon_analysis_validation/validate_carbon_xref.cpp` — a C++ evidence
  emitter that runs the production `compute_carbon_analysis` on three
  deterministic AC cases with analytically known nodal carbon intensities and
  dumps the inputs (generator emission factors + dispatch, load demand, directed
  branch flows) plus the solved intensity vector.
- `tools/carbon_analysis_validation/run_cross_validation.py` — a numpy-free
  oracle that does **not** link hacdcpf; it re-derives and re-solves the Kang
  carbon-emission-flow system `A w = b` (mirroring `solve_carbon_matrix`) by
  independent Gaussian elimination and checks analytic closed forms, the
  independent re-solve, nodal conservation `||A w - b||`, the branch
  loss-allocation rule, and the system emission balance.

Registered as the CTest test `carbon_analysis_cross_validation`. It passes with a
worst error of `3.553e-15` across all seven checks (single-source propagation
`w=0.5`; lossless dispatch-weighted mixing `w=0.3`; lossy mixing `w=57.5/95` with
the emission balance closing to 60 tCO2) at a `1e-7` gate, a negative control
(perturbing one emitted intensity) fails as expected, and the carbon manual
recompiles with XeLaTeX. Verified via the pre-existing Debug ASan build
(`build/macos-asan-ubsan`, which permits the local dirty-dependency skip); the
strict-reproducibility `build/macos-release` configure is currently blocked by
unrelated uncommitted changes in the sibling `../MIPSolvers` benchmark tree, so
the Release CTest run of this new test is pending a clean dependency worktree.

## Documentation source-anchor CI validator + drift closure (2026-08-24)

The manuals reference the code through ~1800 `file:symbol` anchors and ~700
structured path anchors, but nothing verified they still resolved. A new
checker, [doc_anchor_check.py](../../tools/doc_anchor_check.py), now validates
every anchor against the `src/include/tests/tools` tree and is registered as the
tracked CTest test `doc_anchor_check` (pure Python, no build or private
dependency, so it runs on every `ctest` sweep and in the build-test CI job). The
same one-line command is also added to the always-on `lint` job in
`.github/workflows/ci.yml`; that workflow file is git-ignored in this checkout
(`.gitignore` ignores `.github/*` except two skills), so the canonical CI must
mirror the step. The checker resolves
`file:symbol` anchors from both the structured macros (`\srcpath`, `\implfull`,
`\compmeta`'s path argument) and Markdown prose by whole-word symbol presence,
validates structured path anchors rooted at tracked source trees, understands
the `symbol_*` wildcard-family and `foo.cpp/.hpp` dual-extension conventions,
and skips the external `../MIPSolvers` sibling repo.

Running it surfaced eight drifted anchors, now corrected against the verified
symbols (logged as AUD-084). Each dead reference is written below as
`symbol` in `file` form rather than the live `file:symbol` anchor syntax, so the
checker does not re-flag this changelog:

- `ResultAttributionLayer::apply` (×2, cited against `result_attribution.cpp`) →
  the real device-attribution entry `CanonicalToRichOperator::apply`; the named
  class exists in no source file.
- `stage_topology` (cited against `three_stage_reliability.cpp`) →
  `solve_stage_milp`, the F7 "faulted branch is out in every stage" logic.
- `CyberPhysicalFMEAOptions` (×2, cited against `failure_mode.hpp`) → the header
  `reliability_assessment.hpp`, where the struct is actually defined.
- `apply_typhoon_impact` (cited against `scenario_generation.cpp`) →
  `wind_generation_from_track`, and `traffic_node_locations` (cited against
  `typhoon_traffic_impact.cpp`) → `georeference_nodes`; both named phantoms.
- the test path with a non-existent `model/` subdirectory →
  `tests/test_component_models_math_audit.cpp`.

The checker reports `file:symbol ok=1828 path ok=699 bare=1481 failures=0`
(exit 0) on the corrected tree, detects an injected fake anchor as a negative
control, and the model/reliability/scenario_generation manuals recompile with
XeLaTeX.

## time_series regression closure AUD-066/069/070 (2026-08-24)

The three "closed in code, regression pending" annual/lifecycle findings now
have dedicated fixtures in `test_multiscale_comprehensive`, so they are
regression-closed rather than code-only:

- AUD-066 — "Annual replay failure is reported rather than masked as feasible":
  a step whose OPF cannot serve `500 MW` on a single `10 MW` unit leaves
  `physical_replay_complete=false` and `feasible=false`, with the replay
  `model_scope` string, instead of copying the optimistic L0 default (4
  assertions).
- AUD-069 — "Daily replay preserves the frozen UC schedule field by field": on
  `io::build_ieee14_acdc()` the sliced weekly schedule keeps every dispatch and
  VSC/DC-DC/market-storage/DR 2-D field at the window width, and the PF-only
  frozen-schedule replay completes (20 assertions).
- AUD-070 — "Lifecycle storage: duplicate names and repeated replacement stay
  index-keyed": two same-name storages with distinct stable indices are each
  replaced at least twice over a 20-year horizon with per-index cost attribution
  rather than name-collision aliasing (9 assertions). The lifecycle manual
  chapter was corrected from its stale name-match/cycle-truncation description to
  the shipped index-keyed fractional-FDE accumulator with age-since-replacement.

The full `test_multiscale_comprehensive` binary is green (113 assertions across 7
test cases, up from 4), and the time_series manual recompiled with XeLaTeX.

## network_reconfiguration audit closure (2026-08-24)

The six open network_reconfiguration code-audit findings (AUD-018–023, manual
NR-01–NR-06) are fixed and regression-locked. `estimated_loss_mw` on
`POST /api/session/run_reconfig` now returns the core's defined nominal-current
loss proxy (`reconf_loss_mw`) instead of the dimensionally invalid
`milp_objective × base_mva`; `radial_topology_enforced` is
`!(split_domain_trees ∨ allow_dc_mesh)` and the empty-system return clears all
`ValidityFlags`; the `solver` comments now match the runtime HiGHS→SCIP
dispatch, with native branch-and-cut documented as opt-in (weaker than HiGHS on
this LinDistFlow MILP, not an automatic fallback); the core assigns
`base_loss_mw`/`loss_reduction_mw`/`loss_reduction_pct` as defined proxies
instead of leaving them zero; `ONRResult::fallback_used` marks the legacy
base-topology connectivity fallback; and the 629-line unreachable historical AC
branch-and-cut body was removed so `solve_optimal_reconfiguration(ACSystem)` has
one reachable model. New regressions `test_reconfig_options` NR-02/03/04 and
`test_topology_crossval` NR-05 lock the behavior, and the network_reconfiguration
manual recompiled with XeLaTeX. A full Release CTest sweep passed 1690/1690 in
48.3 s (the four added test cases raised the registered count from 1686).

## Repository-baseline test-failure closure (2026-08-24)

The six repository-baseline test failures previously carried in this file were
confirmed reproducible at HEAD on freshly rebuilt Release binaries and fixed;
they are logged as AUD-082 in the module code audit. The earlier
"1675 passed, zero failed" note did not hold for the current tree — the six
tests genuinely failed until this pass. The numbering had also drifted (+11
above ~#1514), so the failures were matched by name, not by number. The fixes:

- `analyze_topology` classified any load-bearing singleton with no *in-service*
  neighbor as `IsolatedLoad`. A single bus carrying a load plus a slack
  generator/external grid was therefore rejected by the AC OPF island pre-check
  ("no island with slack bus"), and a load bus islanded only by an
  out-of-service branch was reported as an orphan. `IsolatedLoad` now requires a
  topological orphan (`has_load && !has_generator && g.adj.empty()`); a
  source-hosting or out-of-service-branch-islanded load falls through to
  `NoSlack`/`Valid`. Fixes `#812`, `#925`, `#1528`, `#1531`.
- `NewtonSolver::solve` flattened every exception to `std::runtime_error`;
  input-contract `std::invalid_argument` (unanchored island, converter
  reference-row requirements) is now re-thrown with its type preserved. Part of
  `#812`.
- `run_three_stage_reliability[_from_string]` swallowed the
  `apparent_power_polygon_sides` `std::invalid_argument` into a soft
  `result.error`; the option is now validated before the soft catch. Fixes
  `#1525`.
- `harmonics_ieee13_opendss` was registered without the `HACDCPF_HAVE_OPENDSS`
  gate; with OpenDSS off the C++ importer returns zero buses and the test failed
  instead of skipping. It is now gated like the sibling OpenDSS tests.

After rebuilding the affected targets, the twelve focused regression suites
`test_graph`, `test_graph_kron`, `test_validation`, `test_reliability_resolver`,
`test_three_stage_reliability`, `test_vsc_limit_ncp`, `test_nighttime_opf`,
`test_advanced_pf`, `test_opf_solver_backends`, `test_distribution_pipeline`,
`test_component_models_math_audit` and `test_topology_crossval` all passed, and
`test_graph`'s AUD-029 orphan-isolated-load case remains green. A first complete
parallel Release CTest sweep processed 1686 registered tests in 52.4 s with one
pre-existing dynamics failure, `transient_native_disturbance_matrix` (#1658),
which was then fixed as described below. The final sweep is 1686/1686 in 59.1 s.

`#1658` was a real, production-relevant dynamics defect surfaced (not caused) by
this pass. Commit `99dfae18` correctly added strict transient event-target
validation; before it, an event whose target could not be found silently did
nothing. That exposed that DC-domain disturbance events authored in the caller's
bus-id space never resolved: canonical projection renumbers non-contiguous DC
bus ids (`canonicalize_dc_bus_indices` maps {10,11,12}→{1,2,3}), while the
driver authored events against DC buses 11/12 and never set `canonical_bus`.
AC worked only because its ids were already contiguous. The GUI
`/api/session/run_transient` route validated DC load events by
`component_index` (passing) while the solver applied them by bus (failing), so a
real user with non-contiguous DC buses would hit the same wall. The fix stores
authored→canonical AC/DC bus maps on the built `DynamicNetwork`
(`DynamicModelBuilder::build`, mirroring resilience `canonical_bus_ids`) and
auto-populates each author-space event's `canonical_bus` in `apply_events`;
`FaultShunt`/`ClearFault` and the device-target helpers now resolve through it.
An explicit `canonical_bus` (as the resilience DAE path sets) still wins.
The driver disturbance matrix is now 18/18. `test_transient_dynamics`
(116774 assertions / 115 cases), `test_dynamic_model_catalog` (4034 / 8),
`test_intelligent_cyber_physical_reliability` (60 / 7) and
`test_resilience_assessment` (364 / 39) all pass unchanged, and the full
registered suite is 1686/1686.

## Electromechanical phasor DAE closure (2026-08-24)

The declared electromechanical transient envelope is now closed for the
phasor-domain model actually implemented: balanced positive-sequence and
explicit three-phase phasor networks, average-value differential devices,
coupled AC/DC algebraic networks, power-flow-consistent initialization,
`MassMatrixDae`, and the numerical event localization/consistent-restart scope
documented below. This is not a claim that EMT, travelling waves, converter
switching waveforms, or formal chronology certification are implemented.

The mixed AC/DC initialization now uses the same DC/DC port-power equation as
steady-state power flow for Power, Voltage and Droop controls. The classical
GUI mixed case has one authored DC voltage source, reproduces PF DC voltages
`[0.99996, 1, 1.00003, 1.06]` as dynamic initial voltages
`[0.999961, 1, 1.00003, 1.06]`, reports DC/DC input/output powers
`-0.293981/-0.299989 MW`, and has initial fast-state residual
`2.1019922e-10 < 1e-7`. During consistent-initialization Newton evaluations,
the algebraic network tolerance is temporarily bounded by
`0.1 * dynamic_trim_tol` and is restored before time stepping; the public
default network tolerance remains `1e-6`. The Anderson--Picard budget is ten
iterations with early exit and the existing Newton fallback.

`DERAADynamic` now follows the PowerSimulationsDynamics/WECC 7-state
(`Freq_Flag=0`) and 10-state (`Freq_Flag=1`) state order and equations,
including frequency deadband/droop, P/Q current priority, generator/load sign,
IEEE 421.5-style directional non-windup, power-order limits, and current ramp
limits. Reduced-ODE, implicit-DAE and initialization residual evaluations feed
DER_A the system COI frequency recomputed from the current trial state. This is
separate from directly attached protection relays, which retain local phasor
PT-angle frequency and local CT current measurements.

The focused DER_A oracle passed 27 assertions with `1e-12` equation gates; the
shared DC/DC equation test passed 15 assertions and the real GUI mixed-case
initialization test passed six. After rebuilding the affected macOS Release
targets, the complete `test_transient_dynamics` executable passed 115 cases /
116774 assertions, `test_dynamic_model_catalog` passed 8 / 4034,
`test_intelligent_cyber_physical_reliability` passed 7 / 60, and the rebuilt
production server passed `gui_api_e2e` in 3.31 s. The HTTP transient response
now echoes the effective `algebraic_network_max_iters` and
`algebraic_network_tol` under its actual `options` object. This pass did not
rerun every registered non-dynamics CTest target. The dynamics manual rebuilt
successfully with XeLaTeX/latexmk to 86 pages with no unresolved references;
only the existing long-identifier box and font-substitution warnings remain.

PSD Test 42 trajectory parity remains open. A direct clean-worktree run of
`julia --project=test test/runtests.jl test_case42_dera` failed before executing
the case because the checked-out SciML environment cannot precompile:
`LinearVerbosity` is undefined and the DiffEq/SciML ChainRules extensions
overwrite a method during precompilation. The external PSD repository was not
modified or instantiated. Consequently, the current evidence is exact
source-equation validation plus internal DAE regression, not a PSD Test 42
trajectory certificate.

## Event-consistent IEEE 1547 and definite-time relay DAE baseline (2026-08-23)

The `MassMatrixDae` path now provides numerical localization for endogenous
IEEE 1547 device-protection actions and directly attached definite-time
`ProtectionRelay` devices. Protection preview is side-effect free; the
accepted candidate step is rolled back and bisected to a bounded time bracket,
actions inside a separately configured anchored forward window are reported as
one event cluster, and the
post-reset algebraic network is solved and audited while the post-reset
differential state is held fixed. First-order IEEE 1547 measurement filters
use their exact interval solution. The external relay implements voltage,
local-frequency, and COSMIC-type balanced positive-sequence Zone-1
apparent-admittance guards with local PT/CT/filter states,
linearly interpolates a guard crossing between accepted endpoints, and emits
`ACLoadScale` or `ACBranchTrip` through the same reset path as authored events.
The production HTTP route parses and echoes localization controls and reports
trial integrations, cluster count, maximum final bracket/cluster span and maximum
post-event algebraic residual; it does not yet construct external relays from
the rich model or request JSON.

The verification is a theory--implementation--test--numerical-evidence
closure, not a formal chronology certificate. The implementation follows the
consistent-state hybrid-DAE event/reinitialization equations documented from
Hiskens--Pai, Zhao--Hu, Henningsson et al. and Song et al. The authors' COSMIC
repository was run with MATLAB R2025b at clean commit
`6acc77e4d3f17925f1f4b79a93652eef0d1314cc`. Its public `sim_case9.m` scenario
is reproduced: branch 6 trips at `10.0 s`, bus 5 UVLS acts at `10.5 s`, and
`31.25 MW` (25%) is shed.

The paper Fig. 2 chronology is not reproduced by that public source. With its
stated branch-7 outage, `0.92 pu` UVLS threshold and `0.5 s` distance/UVLS
delays, the authors' code records only the initial `10.0 s` branch-7 trip.
Bus 5 reaches `0.896065 pu`, but branch 6's maximum distance pickup ratio is
`0.3098124214893234 < 1`; neither the claimed `10.5 s` distance trip nor the
`10.7 s` UVLS action occurs. The fixed public commit therefore does not fully
contain the paper configuration or code version. This is retained as an
explicit failed external cross-validation rather than adjusted away.

Focused Release evidence comprises an analytic filtered-threshold action with
error below `2 us`, a pair of actions separated by `50 us` under a `10 ms`
base step, a simultaneous-action cluster, direct voltage/Zone-1 relay equation
checks, a linear threshold-exit/equal-rate timer-recovery check, and four-step
studies at `0.010`, `0.005`, `0.001` and `0.0002 s`.
The IEEE 1547 action-time range was `1.51996255093 us`, the largest terminal
bisection bracket was `0.9765625 us`, and the largest post-event algebraic
residual was `3.8448755e-10`; the configured gates were `5 us`, `1 us` and
`1e-8`, with at most 14 trial integrations per event. The new relay selection
passed 85 assertions in two cases, and the complete dynamics target passed
116637 assertions in 106 cases. The previously verified online-protection
reliability target remains 60 assertions in seven cases, and `gui_api_e2e`
passed. The updated dynamics manual compiled with XeLaTeX to 85 pages with no
unresolved references; the new formula/evidence pages were rendered and
visually checked, and only pre-existing long-identifier and font-substitution
warnings remain.

After a complete macOS Release rebuild, CTest processed 1676 registered tests
in 187.28 s: 1666 passed, four were conditionally skipped, and six failed.
All six failures reproduced in an isolated six-test replay and are outside the
modified dynamics/protection/HTTP paths: two VSC/AC-OPF exception/reference
contracts (`#812`, `#925`), one invalid reliability-polygon contract (`#1514`),
two graph-island/reference contracts (`#1517`, `#1520`), and the IEEE13
OpenDSS harmonic fixture whose source bus `675` was not found (`#1675`). These
remain open repository-baseline failures; this work neither repairs nor masks
them.

The implemented guarantee is deliberately narrow: it applies only to
`MassMatrixDae` devices implementing `DynamicDevice::previewProtection`.
External relay construction is direct-API only; rich-model builder,
JSON/HTTP/GUI construction, inverse-time integration, multi-zone distance
protection, grazing/Zeno handling, EMT switching, uncertainty/reachability and
formal interval proof remain unsupported. EMT relay measurement is explicitly
rejected because this network supplies phasors. The Zone-1 guard assumes a balanced positive-sequence,
no-tap line model. Partitioned integrators retain accepted-step-end semantics.
Linear endpoint interpolation can miss a within-step pulse whose two endpoints
are both safe; the current mitigation is a sufficiently small base step,
pending dense-output guard root finding.

## `liuyanhui` branch integration and BPA/DSP closure (2026-08-23)

The two commits ending at `origin/liuyanhui`
`2c122751e503a5a7a0c9b568eda923a6d95b2fda` were integrated into local `main`
from pre-merge HEAD `6c6103804d8a880c2b7567531ea0ce0d8b2016d3` after resolving the JSON,
reliability-basis and Newton conflicts against the newer mainline. The
dependency declaration was deliberately kept at MIPSolvers pin
`4a0b16a00daefcbe18d2c5648fe30f840ed77052`; the sibling dependency worktree
was clean at exactly that commit. The incoming legacy dependency policy and
two conditional CMake targets whose source files were absent were not kept.

The accepted BPA/DSP scope adds explicit `DspCompatible` versus
`PreserveSource` L/T small-reactance handling, BQ source provenance, JSON
round trips for the new fields, native engineering provenance on LCC/DC
components, and bounded BA/BA1/BA2/BB/BM/LM/LY parsing. BA layered conversion
is a declared quasi-steady projection: the present one-terminal LCC equations
do not certify physical high/low valve-group series sharing. BM/LM uses the
general DC nodal network, but its external three-terminal DSP fixture
`data/dsp/mtdc_bm.dat` is absent and the corresponding test is explicitly
skipped; this path is not externally numerically certified.

Newton retained mainline globalization, VSC/GFM/NCP behavior, LCC current-limit
semantics and generic batch PV/PQ active sets. Only imported BPA BQ controls use
the format-specific sequential exception: at a converged fixed active set, one
nonzero-range BQ controller with maximum normalized Q violation enters its
bound before re-solving, while violated zero-width BQ controls become fixed-Q
PQ buses as a batch. A proposed automatic four-pass Jacobian max-norm
equilibration was rejected from production after it changed floating-point
factorization/globalization trajectories and regressed VSC/GFM/NCP/MATPOWER
cases. Its standalone API remains experimental, with a `1e-9` physical-step
equivalence unit gate. The proposed generic low-voltage trial guard was removed
entirely rather than exposed as an ineffective option.

After a complete `cmake --build --preset macos-release -j4`, the fully relinked
suite completed with
`ctest --preset macos-release --output-on-failure`: 1679 registered tests in
202.78 s, 1675 passed, four conditionally skipped and zero failed. The skips
were the unavailable formal-SOC JSON fixture, the unavailable BM/LM DSP
fixture, and two declared GridLAB-D conditional comparisons. OpenDSS covered
5 registered tests, GridLAB-D 3, and GUI/E2E 17; all tests that actually ran
passed. Focused evidence also includes 24 assertions in four BPA BQ cases,
five assertions for generic strongly coupled batch switching, 1680 executed
BPA I/O assertions (31 pass/1 skip), 18 scaling assertions, 313 power-flow math
audit assertions, and the ACTIVSg2000 GFM benchmark. The I/O and power-flow
manuals compiled successfully with XeLaTeX to 12 and 152 pages; pre-existing
long-path box/font warnings remain, with no compile failure.

## Complete Release, sanitizer and external-engine verification (2026-08-23)

The full macOS Release baseline for the current solver source contents was
rebuilt with
`cmake --build --preset macos-release -j4`, then the complete registered suite
was executed with `ctest --preset macos-release --output-on-failure`. After the
four sanitizer repairs, all 1666 tests completed in 206.45 s with no failures:
1663 passed and three were conditionally skipped (one unavailable formal-SOC
JSON input and two declared GridLAB-D conditional comparisons). This is the
normal-build baseline for repository HEAD
`c3cd6315bcbf66ebe13d5719c55a4f30991dd204`. The dependency source files used
by that run are now exactly represented by the clean MIPSolvers commit and pin
`4a0b16a00daefcbe18d2c5648fe30f840ed77052`.

The dependency declaration in `cmake/Dependencies.cmake` was advanced from
`3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c` to that committed MIPSolvers HEAD.
During the transition, Release configuration correctly refused the then-dirty
four-file sanitizer overlay; after those files became commit `4a0b16a`, the
clean sibling worktree passed normal `cmake --preset macos-release`
configuration without a pin or dirty-tree warning. A subsequent
`cmake --build --preset macos-release --target run_gui_server --clean-first -j4`
rebuilt MIPSolvers, `hacdcpf` and the server from source. The resulting 41 MiB
arm64 binary is `build/macos-release/run_gui_server`, with SHA-256
`68f99ad3169eb91edbdccadc88c612893b2e03f9ce515e969627dbfac951001a`.
The Release `gui_api_e2e` and `runtime_api_v1_e2e` tests passed 2/2 in 3.41 s.
A live smoke check returned HTTP 200 for `/xjtu/`, reported full edition at
`/api/edition`, and returned `hysim_api_v1` version 1.0 at `/api/v1`. The
complete 1666-test suite was not rerun after the pin metadata update; the
earlier full result above used byte-equivalent dependency source changes.

The OpenDSS/GridLAB-D label selections covered five distinct registered tests
and all five passed in 5.51 s (OpenDSS 5/5 and GridLAB-D 3/3, with overlapping
cross-engine tests counted once). The machine checks reported:

- 50 OpenDSS short-circuit cases with maximum `Ikss` relative error
  `1.805581638e-7`, plus 35 GridLAB-D balanced shunt short-circuit cases with
  maximum relative error `2.329225394e-8`;
- IEEE 13/34/123 Thevenin comparisons over 111 buses and 2220 quantities with
  maximum relative error `8.558706274e-16`;
- six time-series PF steps with maximum voltage error `6.587109747e-10 pu`
  against OpenDSS and `5.847480753e-7 pu` against GridLAB-D;
- 164 IEEE13/OpenDSS harmonic points with maximum complex-voltage error
  `0.001651738435 pu`, below the declared `0.002 pu` gate;
- 14/14 harmonic-matrix cases, including nine numerical OpenDSS cases and six
  GridLAB-D cases/24 slices, with maximum error approximately `5.35e-10 pu`.

These external comparisons certify only their common implemented electrical
subsets; they do not certify UC, lifecycle, control, switching transient or
unmodelled AC/DC physics. The ETAP adapter binary passed 21 cases/733 assertions,
covering OpenXLSX/XML adaptation, round trips and PF/short-circuit fields. The
real `data/etap_test.xml` PDE fixture is absent, so its conditional fixture path
only emitted a warning and returned. No ETAP commercial executable was invoked,
and this result must not be described as commercial-ETAP numerical parity.
`gui_api_e2e` also passed 1/1 in 2.12 s.

A separate Debug tree at `build/macos-asan-ubsan` was configured with AppleClang
21, `-fsanitize=address,undefined -fno-omit-frame-pointer`, and ETAP, OpenDSS,
Ipopt and SuiteSparse disabled. The full command was:

```bash
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir build/macos-asan-ubsan --output-on-failure
```

After repairing the four sanitizer defect classes described below, the entire
tree was rebuilt and the same strict command completed all 1628 tests in
4573.27 s: 1597 passed, 10 were skipped and 21 failed. There was no ASan or
UBSan report from any repaired path. LeakSanitizer was not enabled on this
macOS runtime. The ten skips are one Ipopt-only case, one unavailable
formal-SOC fixture, two GridLAB-D conditional cases and six phase-hybrid
OPF/relaxation cases excluded by the configured optional-feature set.

The four sanitizer defects are closed as follows:

- MIPSolvers' two mirrored `HighsHash.h` implementations now classify NaN and
  signed infinities into distinct stable IEEE-754 tags before the finite-value
  mantissa/exponent hash performs any integer conversion. Finite hashes and
  equality-based collision resolution are unchanged; the storage-market test
  therefore keeps its original optimization semantics without `-Inf` to
  `short` undefined behaviour.
- `tests/test_nighttime_opf.cpp` now supplies all 24 entries of `solar24` and
  asserts the 24-sample contract before constructing a repeating profile. The
  modulo-24 access is consequently total over the declared profile period.
- Dense parity-IPM explicitly recognizes the unique empty solution of a
  `0 x 0` KKT operator before factorization and returns empty primal/dual
  increments only when the right-hand side matches that zero dimension; all
  dense solves now reject a mismatched KKT/RHS dimension. In addition, vendored
  Eigen `PartialPivLU` skips the mathematically empty trailing Schur update when
  its recursive rectangular panel has rows but zero columns. This preserves
  the original partial-pivot sequence and avoids constructing a writable
  zero-width `Ref`.
- The same Eigen guard closes the small-signal path in which the complex
  eigenvector matrix is empty. The modal formulation and
  `src/dynamics/SmallSignal.cpp` remain unchanged: inversion on the zero-space
  now terminates through Eigen's valid empty-operation path instead of UBSan.

Strict focused replay of the original five failing tests passed 5/5 in
104.26 s. An expanded selection covering dense KKT and every registered
small-signal case passed 6/6 in 103.86 s. The corresponding Release regression
selection passed 10/10 in 0.82 s. After adding the explicit dense-KKT/RHS
dimension guard, the combined ten-test ASan/UBSan selection passed 10/10 under
strict halt/abort settings in 111.22 s. A trial replacement of `PartialPivLU` by
rank-revealing `FullPivLU` was rejected: although it removed the immediate UB,
it changed the Newton trajectory and reduced two existing convergence counts
from 24/24 to 12/24 and from 4/4 to 3/4. The accepted guard instead retains the
existing numerical method and results.

Ten further failures are reproducible Debug/configuration contract differences,
not sanitizer-clean evidence: the multiscale and GUI hybrid OPF cases hit the
no-Ipopt parity-IPM iteration limit (constraint residuals about `4.05665e-4`
and `7.12345e-2`); the unanchored-island case fails closed with a different
exception type; the zero-sequence short-circuit values pass but report status
`solved` instead of `solved_zero_sequence_open`; four three-stage reliability
cases hit the HiGHS Debug assertion `col < origColIndex.size()` at
`HighsPostsolveStack.h:287`; the invalid reliability polygon is captured into
`result.error` instead of throwing; and `harmonics_ieee13_opendss` cannot resolve
source bus `675` because the OpenDSS bridge is disabled in this tree.

The remaining 11 failures are browser/HTTP E2E timeouts, connection refusals or
insufficient LCC result state in the instrumented tree. The exact same 11-test
selection passed 11/11 in the Release tree in 100.41 s, including NCP/Schur,
layout/scale, LCC BPA round trip, overview, parameter contract, three reliability
workflows, market and RPO. They are therefore retained as sanitizer-tree
integration failures rather than promoted to Release regressions. Thus the four
memory/undefined-behaviour defects are closed, while the 21 separate
Debug/configuration and integration failures keep the full sanitizer tree from
being a green all-contract baseline.

## Integrated-energy, model and SPPT monographs (2026-08-22)

The compact contract manuals were replaced by theory--implementation--numerical
evidence chains. `integrated_energy` is now 10 chapters/23 A4 pages with three theory
chapters, a source-equivalent LP/MILP, full I/O contracts, two analytic optimal
solutions and a 24-hour independent equation/cost oracle. The analytic cases match
within floating-point error; the 24-hour maximum electric balance and stock
recurrence residuals are `1.776e-15 MW` and `1.066e-14 MWh`, and independently
recomputed cost differs by `1.819e-12`. The registered Release
`integrated_energy_cross_validation` passes at a `1e-7` gate. This remains an
aggregate-PCC campus model with no AC voltage/branch certificate and no independent
external full-MILP oracle.

`model` is now 11 chapters/22 pages with typed-identity, dimensional/per-unit and
quotient/recovery theory. The registered `model_projection_cross_validation` passes
15 declared set/algebra/unit checks at `1e-12`, including exact load/generation
conservation, intensive/extensive recovery, same-number AC/DC separation and unit
conversion idempotence. It also found an open contract defect: for authored DC IDs
`(1,2,5)`, position recovery is correctly `(1.0,0.99,0.0)` pu but
`dc_dead_bus_indices` reports position token `3`, not stable ID `5`; the machine
report therefore keeps `stable_dead_id_contract=false`.

`sppt` is now 11 chapters/32 pages. The certificate corpus has MR3 residual zero on
10/10 cases, but the independent authored-equation evaluator supports only 6/10;
its largest observed residual is `8.26e-9` under a `1e-6` gate. Eight scale cases
through 2869 buses converged; the switch ablation enlarged max Ybus diagonal by
`3.61e4`. A fixed-seed, five-repetition public-feeder campaign completed 315 base
samples and reports Wilson intervals. Increasing repetitions to 10 or 20 reproducibly
terminates on uncaught `std::invalid_argument`, so fault statistics remain
exploratory rather than release-level evidence. MR8 still has no public relation API.

Existing focused direct binaries also pass: integrated energy 6 cases/106 assertions,
model 30 cases/1696 assertions, and eight SPPT targets 37 cases/246 assertions. The
three manuals were compiled with XeLaTeX, all 77 pages rendered at 120 dpi, and full
contact sheets plus dense numerical/table pages were inspected. Logs have no
overfull boxes, unresolved references/citations, missing glyphs or fatal errors; no
blank, clipped, overlapping or broken-table page was observed. No full CTest,
sanitizer, external PF/MILP engine or pinned-dependency release validation was run.
Local MIPSolvers HEAD `7721936245d5756193381b415457e6cf2214341e` differs from pin
`3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`.

## EV--traffic, I/O and AML power-model documentation closure (2026-08-22)

The final three compact manuals identified by the 23-module audit now have explicit
theory--source--result chains. `ev_power_traffic` adds CTM/LTM conservation and CFL,
Wardrop VI, charging-stock and certificate theory ahead of the existing A--H source
model. Four current Release binaries pass 38 cases/1326 assertions. The 20-vehicle
pulse closes exactly in both CTM and LTM; the deliberately CFL-invalid case separates
150, 1967 and 1998 veh/h LTM/coarse-CTM/fine-CTM outcomes; the DUE case serves 40/40
kWh at zero reported gap. These are analytic and cross-discretization checks, not a
SUMO/MATSim or field-calibrated traffic oracle.

`io` now defines round-trip equivalence, domain-qualified identity, unit/source
evidence, structural loss, XML safety and the exact `ImportReport::passes_mode`
predicate before mapping those contracts to JSON, MATPOWER, CIM and BPA. Six focused
Release binaries pass 93 cases/1758 assertions; `test_io_json` has one explicitly
skipped case because its optional data file is absent. BPA--DSP comparison passes
5 cases/251 assertions across the shared LCC/VSC electrical subset. GridLAB-D,
OpenDSS and ETAP were not rerun in this pass, and no complete CGMES 3.0 or commercial
ETAP bidirectional oracle is claimed.

`power_models` now separates general ACOPF/ACDCOPF/DCOPF/LinDistFlow/SCUC theory from
the source-equivalent AML builders. The new registered
`power_models_cross_validation` executes five repetitions, independently recomputes
all public equations in Python and aggregates the worst residual at a fixed `1e-7`
gate. DCOPF is fixed to `NativeDualSimplex`, LinDistFlow alternates explicit
`NativeDualSimplex`/`NativeIPMLP`, and SCUC is fixed to `StrictHiGHS`, all with
fallback disabled. Backend-contract error is zero; the worst numerical error is
`3.73311195e-8`, with LinDistFlow's worst analytic voltage error `8.737e-12`. The
test completed in no more than 1.71 s during this pass. An adversarial
`NativeIPMLP` DCOPF run had only `7.050e-9 MW`
dispatch error but `1.7402e-7` absolute objective error and is therefore not admitted
at the unchanged gate. This is an internal cross-backend plus independent-equation
oracle, not an external solver certification; nonlinear ACOPF/ACDCOPF remain outside
this specialized oracle and SCUC prices remain invalid.

The three manuals compile with XeLaTeX to 14, 12 and 15 A4 pages. All 41 pages were
rendered at 110 dpi and inspected; the added theory, source and numerical pages show
no clipping, overlap, broken tables, unintended blanks or literal TeX control words.
This was a focused Release/documentation pass, not full CTest, sanitizer or pinned-
dependency validation. Project HEAD is `e9799bb7aac4d239ff2c5811a202cbfa0fd1c086`;
the local MIPSolvers HEAD remains different from the repository pin as recorded above.

## Market, carbon, analysis, API, server and validation documentation closure (2026-08-22)

The six previously compact module manuals now include separate general-theory chapters,
source-symbol mappings, numerical evidence and explicit validation gaps. `market` documents
SCUC/SCED/LMP, LODF security and settlement; its Release regression is 22 cases/845 assertions
with 24/24 pricing and AC-security periods, while no independent LP/MILP or market-software
oracle was run. `carbon_analysis` documents proportional tracing, sparse carbon-potential
equations and sequential storage-carbon inventory; its three Release targets pass 21/246,
15/126 and 5/202 assertions, without an external carbon-flow oracle.

`analysis` documents hosting-capacity, multidimensional weak-link and counterfactual decision
theory; the three focused targets pass 3/27, 4/33 and 2/52 assertions, without an independent
DL/T 2041 calculator or 8760-hour planning oracle. `api` documents capability Boolean gates,
safe-result semantics, signature reuse and OPF post-audit; `test_solver_capabilities` passes
17/61 assertions, but no long-lived handle benchmark or cross-language ABI test was run.
`server` documents ETag/revision preconditions, cache invalidation, stale asynchronous jobs and
edition routing; `test_edition_profile` passes 3/33 assertions, while production RuntimeApiV1
HTTP E2E remains unquantified. `validation` documents exact predicates and Basic/Electrical/
SolverReady/Strict filtering; `test_validation` passes 37/178 assertions. These results are
regression and closed-form evidence, not field accuracy, false-positive-rate or large-scale
performance certification.

All six manuals were rebuilt from the current worktree with XeLaTeX: `analysis` 16 pages,
`api` 10, `carbon_analysis` 11, `market` 12, `server` 6 and `validation` 6. All 61 A4
pages were rendered at 100 dpi and reviewed as contact sheets; no unintended blank page,
clipping, overlap or broken theory/numerical table was observed.

## Resilience dynamic-feedback and cross-validation closure (current worktree)

The resilience monograph now follows the theory/implementation/interface/
numerical/admission chain for heuristic, strict hybrid AC/DC, RA-stage, MESS and
MIP-to-DAE restoration. The implementation replays domain-qualified bus service
and generator/renewable/storage dispatch, preserves grid-forming microgrid
semantics through canonical projection, and applies topology-aware DC voltage
health checks. DAE initialization/build/integration failures are fail-closed.

`run_certified_distribution_resilience_mip` closes the finite master loop: only
proof-valid Unsafe L3 results produce an exact topology no-good and any
applicable bus-service upper bound; Failed/Unresolved never produce an
operational cut. The review case required two restoration MIP solves and one
applied cut. Its final `4->5` and `23->23` transitions are Safe with
`proof_valid=true`; minimum frequency was 50.686552/50.012250 Hz and minimum AC
voltage was 0.996237/0.998483 pu. Same-process catalog-master and restoration
MIP calls are isolated on fresh joined threads; the full 11-case certificate
tag passes 110 assertions.

The fixed-seed Release review completed 2048/2048 feasible paired risk samples
and stopped only after two consecutive declared checkpoints passed the mean
(5%) and CVaR95 (10%) relative-change limits. Baseline/intervention mean ENS was
1.146187/0.049942 MWh; VaR95 was 1.75/0.23 MWh; CVaR95 was
2.482422/0.962422 MWh. Strict restoration served 67.15/68.40 MWh with 1.25 MWh
ENS and zero reported MIP gap.

Three frozen AC snapshots (`pre_event`, `post_fault`, `restored_final`) were
actually solved by HySim, DSS C-API 0.14.5 and the local GridLAB-D 5.3.0 binary.
All passed the declared 0.005 pu/0.5 degree gates. Maximum OpenDSS errors were
0.0006072 pu and 0.071191 degrees; GridLAB-D maxima were 7.904e-7 pu and
3.819e-6 degrees. This certifies only the common balanced positive-sequence AC
steady-state scope, not DC, protection, controls, switching transients or DAE.
Machine evidence is under `external_data/resilience_validation/`.

## Time-series closure repair (current worktree)

The annual/lifecycle repair pass is source-backed and intentionally narrower
than a claim of full lifecycle closure. `AnnualProductionSimResult::feasible`
now aggregates weekly UC and required OPF/PF stages; `schedule_only`,
`physical_replay_complete`, `ens_complete`, and `model_scope` expose whether a
physical certificate exists. Curtailment is computed from explicit
available-minus-dispatch power and ENS carries a known/proxy flag. Annual
cyclic SOC and penalty settings are forwarded to the lower-level model;
`iterative_feedback=true` now runs a bounded bottom-up fixed-point loop over
physical replay and updates ENS/curtailment penalties; hard budget violations
remain infeasible. The annual UC is one coupled horizon, so SOC/ramp/commitment
states cross week boundaries. Daily replay preserves every current `UCSchedule` trajectory row,
direction row, market-storage row, and solver certificate.

Lifecycle state now validates domains and cadence, uses stable storage indices,
retains fractional equivalent-full cycles, and resets calendar age on
replacement. The deterministic carbon estimator is asset-resolved for
authored AC generators/static generators and declares its fallback/source.
Sampling and its bound share one Neyman allocation and report `bound_passed`.
Lifecycle defaults to physical annual replay and executes independent sampled
OPF/PF corrections. External-grid profile/static factors, AC/DC storage
inventory intensity and DC static-generator carbon are included. Embodied
manufacturing carbon, converter/network material inventories and fuel heat-rate
curves remain explicit open boundaries.

Verified after the repair: `test_multiscale_comprehensive` 4 cases/75
assertions, `test_uc_storage_efficiency` 2 cases/13 assertions, the registered
`Multiscale comprehensive AC/DC case runs from milliseconds to a year`,
`gui_api_e2e`, and `time_series_cross_engine_matrix` all pass in the current
macOS Release tree. The latter retains the existing OpenDSS/GridLAB-D/HiGHS/
SciPy evidence and is a PF/UC cross-engine check, not a lifecycle oracle.
The time-series manual was rebuilt to
`output/pdf/time_series_manual.pdf` (50 A4 pages); the XeLaTeX log has no
overfull boxes, unresolved references, missing glyphs, or fatal errors, and all
50 pages rendered successfully. No full CTest, sanitizer, or new external
lifecycle oracle was run in this repair pass.

## Time-series monograph and verification baseline (2026-08-22)

The `time_series` manual is now a three-volume, 50-page source-equivalent
monograph at PF/OPF granularity. In addition to the theory and
`time_series_pf.cpp` coverage, two dedicated chapters (506 source lines) now
audit every public option/result family and reachable branch in
`annual_production_sim.cpp` and `lifecycle_simulation.cpp`: time/block slicing,
L0 defaults, weekly look-ahead, daily replay, three parallel-day modes,
energy/cost accounting, component statistics, yearly state mutation, SOH and
replacement, deterministic strata, the separate Neyman bound allocation,
carbon proxies, NPV, capacity scaling and comparison. The manual distinguishes
declared fields from consumed behavior and does not promote roadmap behavior to
current capability.

The Release evidence recorded in the manual includes 24/24 and 168/168
time-series PF convergence on the 21-AC/4-DC multiscale case (maximum residuals
`4.10506e-10` and `4.11283e-10`), the storage recursion checks `SOC=0.35` and
`0.53`, a three-year lifecycle HTTP run (`NPV=8718508.6873`, total carbon
`149582.0631 tCO2`), and the 0.5×/1.0×/1.5× BESS capacity scan. The focused
time-series CTest selection passed 29/29 tests. The current source rerun also
passes 11/11 focused annual/SOC/replay tests, `gui_api_e2e`, and the registered
`time_series_cross_engine_matrix`. The annual 365×24 h case is
historically used `skip_replay=true`; new runs can enable coupled full-year
replay. The HTTP lifecycle route constructs 6 h input by default and now
defaults to physical replay plus sampled correction. OpenDSS
and GridLAB-D remain per-step PF references, not UC/lifecycle oracles.

The deep source audit is now split between closed runtime repairs and explicit
remaining boundaries. Annual feasibility includes one coupled UC, hard budget
certificates, cyclic SOC residuals, feedback status, required OPF/PF status and
explicit curtailment/ENS provenance. Lifecycle replacement age, fractional
cycles, AC/DC stable-index attribution, physical replay, sampled PF correction
and grid/storage/DC carbon components are source-backed. Remaining boundaries
are embodied manufacturing carbon, network/converter material inventories,
fuel heat-rate curves, randomized coverage guarantees and the intentionally
silent `verbose` flag. Schedule-only annual results remain explicitly non-AC
certification.

The manual was rebuilt with the repository LaTeX skill using XeLaTeX and
rendered with Poppler. The final artifact is
`output/pdf/time_series_manual.pdf` (50 A4 pages). All 50 pages were rendered
at 120 dpi; the full contact sheets and the dense annual/lifecycle option,
formula and result pages show no clipping, overlap, missing glyphs, broken
tables or blank pages. The final XeLaTeX log has no overfull boxes, unresolved
references/citations, missing glyphs or fatal errors. The local MIPSolvers HEAD remains
`7721936245d5756193381b415457e6cf2214341e`, different from the recorded pin
`3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`; this is a focused Release result,
not a pinned-dependency or sanitizer baseline.

## Harmonic state-space and converter-model upgrade (2026-08-22)

The harmonic model layer now owns first-class `DCCapacitor`, `DCReactor`, and
domain-qualified AC/DC `HarmonicFilter` collections. Their SI R/L/C, ESR/ESL,
leakage and ratings survive JSON round trip, canonical bus remapping and DC
dead-island stripping; stable component rows are available to result attribution.
The per-order network assembly and the new HSS path both stamp these devices.

`solve_harmonic_state_space` assembles one sparse AC/DC matrix over a requested
positive-order Fourier grid. Explicit complex current injections and arbitrary
frequency-coupled admittance entries share the same public contract. Results are
broadcast to authored bus IDs and report per-device terminal complex-current
spectra, matrix dimensions/nonzeros, factorization status, model scope and a
default `1e-10` normalized backward-error admission gate. The production route is
`POST /api/session/harmonics_hss`.

Opt-in converter models now cover two-level sinusoidal PWM (fundamental plus two
carrier groups), averaged MMC insertion with arm/submodule energy and circulating
control, six-pulse LCC coefficients with commutation-overlap attenuation, and
Buck/Boost/BuckBoost/Isolated DC/DC rectangular PWM. AC filters, DC-link/input/
output capacitors, smoothing/arm/output inductors and delayed PI controllers are
stamped explicitly. Missing physical parameters and unsupported Generic DC/DC
models fail rather than fall back to an empirical spectrum.

The rebuilt Release harmonic target passes 63 cases/412 assertions. New numerical
gates include zero-coupling HSS versus per-order HPF and two independent closed
forms (DC capacitor and an off-diagonal two-frequency block), all at `1e-10 pu`,
plus nonzero cross-frequency response for all four converter families. The frozen
1000-AC-node, 20-frequency, four-VSC case has matrix dimension 20,080 and passes
the predeclared sparse-storage/dense-storage ratio below 20%. The production
server target compiles. The mandatory 14/14 cross-engine matrix reran with
OpenDSS and local GridLAB-D 5.3.0: maxima remain `5.333277457e-10 pu` and
`5.349716508e-10 pu`; IEEE13/OpenDSS remains `1.651738435e-3 pu` over 164 points.
Those engines validate network/RLC frequency response, not internal HSS switching
matrices; converter validation is analytic/degeneration/backward-error evidence.

The local dependency remains at `7721936245d5756193381b415457e6cf2214341e`,
different from the recorded pin `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`.
The ten-chapter monograph was rebuilt with XeLaTeX to
`output/pdf/harmonics_power_flow_manual.pdf` (19 A4 pages). All pages were
rendered at 120 dpi; the new HSS pages and dense API/audit pages were visually
checked with no clipping, overlap or broken tables. The final log has no
overfull boxes, unresolved references, missing glyphs or formula warnings.

## Short-circuit theory-to-code closure and monograph (2026-08-21)

`docs/modules/short_circuit/` is now a fifteen-domain-chapter, source-equivalent
manual at PF/OPF granularity, followed by the shared industrial evaluation baseline.
The existing symmetrical-components, IEC 60909,
converter/DC-transient theory is retained and separated from reachable code.
New chapters cover rich-to-canonical projection and authored identity,
selective sparse inverse and batch reuse, complete C++ option/result fields,
the three production HTTP/GUI paths, evidence-tiered numerical cross-validation,
failure truth tables, a deep source audit, and industrial admission gates.

The audit findings AUD-032--AUD-053 are closed in runtime code and validation tooling, with zero open
findings in the detailed IEC 60909 scope. Detailed AC now
checks every required sparse factor and selected solve against a `1e-9`
backward-error gate, returns exact fault-point phase/ground currents, separates
requested and effective voltage factors, classifies failures, and maps branch
results to authored domain/kind/index identity. Contracted ideal AC switchgear
retains an authored row with `electrical_value_available=false` instead of a
fabricated zero current.

The IEC closure now includes line end-temperature resistance, generator
nameplate voltage and `pg_percent`, complete `K_G/K_T/K_S` handling (including
internal power-station faults), two-winding vector-group zero sequence, and
three-winding positive/zero-sequence four-node stamps with tap-aware Schur
elimination. Method C owns an independent `fc/f=0.4` sparse factor with IEC
generator peak resistance. Methods A/B use the standard single fault-point peak
factor; Method B defaults to independently tested automatic topology detection
with LV/MV-HV caps. Breaking current implements formula (77), full `mu` curve
interpolation and unbalanced formulas (78)-(80). Steady current requires authored
lambda data, uses `lambda_min` for terminal-fed static excitation, and uses
motor-free `Ibmo` for multiple-fed near faults. Annex A uses the effective factor
represented by the reported peak. Zero sequence is factored by connected component:
grounded components solve normally, while an ungrounded fault component returns
`solved_zero_sequence_open` and physical 0 A with the capacitive-current boundary
declared. All added IEC fields survive JSON round trip.

The DC kernel now contracts ideal conductors, excludes source-free islands from
the active Dirichlet block, reuses one sparse factorization and selected columns
across a batch, observes cancellation, and rejects invalid/ambiguous inputs.
Post-fault voltages are recovered by the compensation theorem; DCCB duty is the
actual resistive-edge current, including 10/5 kA parallel division and explicit
series chains. The production AC/DC routes expose per-item status, scope,
limitations, residual quality and aggregate completion counts; the detailed AC
HTTP boundary rejects a negative fault impedance with an explanatory 400.

The current Release targets were rebuilt and executed:
`test_short_circuit_crossval` passed 39 cases/626 assertions,
`test_dc_short_circuit` 16/60, and `test_short_circuit_iec60909_4` 2/231, for
57 direct short-circuit cases/917 assertions. Focused JSON short-circuit tests
passed 3/48. The IEC TR 60909-4 section 6.2 reference keeps all six reported
errors below 0.5%. The comprehensive 13-bus test checks 3PH max/min, 2PH, SLG,
and method-C peak arrays; all grounded numerical gates pass, with method-C peak
maximum relative error `1.15e-12` and grounded SLG maximum `1.17e-6`.

Both registered OpenDSS validations were rerun. The 50-case complete-network
matrix has maximum `Ik''` relative error `1.805581638e-7` and peak error
`1.485279759e-7`. IEEE 13/34/123 external-Thevenin validation covers 111 buses,
444 faults and 2220 quantities with maximum relative error
`8.558706274e-16`; this is fault-kernel parity, not complete phase-domain
network parity. The registered matrix automatically found the sibling GridLAB-D
5.3.0 build and its mandatory gate passed 35/35 balanced-probe cases; maximum
relative error was `2.329225394e-8`. Missing GridLAB-D, fewer than 35 numerical
cases, or error above `1e-6` now fails the test. `gui_api_e2e` passed 77/77 checks.
On the frozen 1000-bus/64-fault protocol, batch median was 1.02900 ms versus
16.5459 ms for repeated single calls (ratio 0.0621906, maximum current difference 0 pu), below
the predeclared 0.10 threshold.

The monograph was compiled with XeLaTeX to
`output/pdf/short_circuit_manual.pdf` (39 A4 pages). All 39 pages were rendered
with Poppler at 120 dpi and visually reviewed, including the dense option,
HTTP, numerical-validation and audit tables. The final log has no overfull
boxes, unresolved references/citations, missing glyphs or fatal errors; the
rendered pages have no clipping, overlap, broken tables or unintended blanks.

Required IEC input that is absent is rejected rather than inferred. EMT/DC
capacitor discharge, controller, protection and IEC 61660 studies are owned by
their dedicated model families and are not tracked as unfinished IEC 60909 work.
The evidence uses project HEAD
`7970d9b21c765b06d21f72f0b0e93a56564cf85e` and MIPSolvers HEAD
`7721936245d5756193381b415457e6cf2214341e`; the latter differs from the
repository pin `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`. It is a focused
Release result, not a full CTest, sanitizer, or pinned-dependency baseline.

## Graph audit closure (2026-08-23)

`docs/modules/graph/` now has an eleven-chapter source-equivalent manual at the
granularity of the PF/OPF manuals. The existing 664-line graph-theory chapter
is retained intact; nine implementation chapters now separate public identity
and units, rich-model graph coverage, topology algorithms, switch/zero-Z
contraction, reduction planning, series/pendant behavior, dense/sparse Kron,
mapping/recovery, production HTTP/GUI composition, and verification/audit.
The manual explicitly separates general graph/Ward theory from reachable code
and states that no unified public C++ reduction pipeline exists.

The deep source audit now covers all ten public graph headers, nine
implementations, CMake registration, direct tests, validation, major
cross-module consumers, and production topology/network-reduction routes.
AUD-024--AUD-031 are closed for the declared topology/reduction scope:
planning is domain-qualified; every public option/action has reachable behavior;
mapping is bidirectional and composable; pendant recovery uses stable component
IDs and fails on missing lookup; Kron stores the interior injection correction;
`IsolatedLoad` reaches validation; `edge_id` remains graph storage position; and
Transformer3W/LCC/EnergyRouter/three-phase connectivity plus rich-terminal
contraction/export behavior is covered. Unsupported phase/transformer/DCDC
collapses fail closed. Rich virtual edges remain connectivity-only, HTTP Kron
remains identify-only, and pendant folding remains approximate.

The affected Release targets rebuilt successfully. `test_graph` passed 37
cases/193 assertions, `test_graph_kron` 10/56, `test_graph_roundtrip` 17/294,
`test_topology_crossval` 12/86, and `test_validation` 38/180, for 114/809. The
four graph binaries ran serially in about 0.51 s. `gui_api_e2e` passed all 80
checks, including rich reduction/export/reload preservation. A rebuilt
ASan/UBSan selection passed 42 tests with one conditional skip and no sanitizer
report. The constant-current Kron partition residual is at most `1e-12`.
MIPSolvers is clean at the repository pin
`4a0b16a00daefcbe18d2c5648fe30f840ed77052`. Full CTest and a full-network graph
scale benchmark were not run.

## Network reconfiguration documentation and deep audit (2026-08-21)

`docs/modules/network_reconfiguration/` has been expanded from a 136-line main
file plus one 120-line implementation chapter into an eight-chapter manual. It
now separates engineering theory from reachable implementation and covers the
public option/result contract, canonical projection and domain-qualified IDs,
source/load aggregation, unified and split-domain radiality, optional
LinDistFlow, device capabilities and protection sequencing, heuristic/external/
native solver paths, posterior certificates, the legacy ONR wrapper, production
HTTP post-validation, registered tests, and known limits. The previously
referenced but missing `chapters/theory_reconfiguration.tex` is now present.

The module audit depth in `docs/testing/module_code_audit.md` is now **Deep**.
Six open findings are recorded as AUD-018--AUD-023. The highest-severity issue
is the production route's dimensionally invalid
`estimated_loss_mw = milp_objective * base_mva`; clients must use post-PF
`reconfig_loss_mw` only when `reconfig_pf_converged=true`. Other findings cover
validity flags set before feasibility, solver-comment/fallback drift, public
result fields with no core assignment path, a legacy ONR fallback that hides a
failed MILP behind unchanged-topology PF feasibility, and an unreachable old
AC B&C implementation. This pass documents the defects but does not change
runtime code.

The same focused Release rebuild also passed `test_topology_crossval` (12 cases/86
assertions), `test_reconfig_options` (10/37), `test_device_flows` (5/18),
`test_distribution_pipeline` (1/178), and `test_crossmodule_integration` (2/82),
totalling 30 cases/401 assertions. Numerical evidence includes 6-bus MILP versus
exhaustive relative difference 0% with `optimal=false`, independent IEEE 33-bus
BFS loss `0.118446 -> 0.0995629 MW`, and cross-module Newton PF loss
`0.202677 -> 0.16527 MW` with residuals below `1e-8`. The BFS and Newton values are
different model outputs and are intentionally not merged. No full CTest, HTTP E2E,
sanitizer, or pinned-dependency rerun is claimed.

## 可靠性理论与执行闭环（2026-08-20）

### 物理后果独立章节与三阶段可行性审计（2026-08-20）

新增中文手册章节 `chapters/physical_consequence_models.tex`，把物理后果单独定义为状态投影后的
最小切负荷算子，系统审计主网 HL-II DC-OPF、混合 AC/DC 网络 LP、配网 LinDistFlow、Stage 1
保护隔离、Stage 2 运行拓扑重构和 Stage 3 修复窗保持拓扑。章节给出逐节点平衡、角度参考、固定注入
削减、VSC 双向效率、辐射森林、开关预算、储能跨阶段递推和任意故障的构造性可行点，并明确区分
物理削减、保护不准入与构模/数值失败。可靠性手册 XeLaTeX 编译由 107 页增至 112 页，新增章节
第 24--27 页及受影响的三阶段第 48--49 页经 Poppler 渲染复核，无致命错误、溢出版心、裁切或重叠。

三阶段实现删除了求解失败、等式后验违反和不等式后验违反时的“全切负荷”伪造路径。闭环健康拓扑
固定全闭与辐射森林冲突时，固定拓扑仅作为可重试预解，随后调用允许开边/重构的通用 MILP；通用
MILP 或其后验认证失败立即返回带阶段号、后端状态、间隙或残差的错误，不写入 EENS/SAIDI/SAIFI。
保护联锁未通过时，Stage 2/3 仍在 Stage-1 安全拓扑上求解并认证实际削减，状态为
`success (protection interlock)`、恢复准入标志为 false；该物理后果按持续时间计入指标，但不再被误报为
求解失败。章节中的辐射森林已按实现修订为商品流、健康常闭边保持和边数不等式，并补全电压区间、
Big-M 与含根健康分量生成树等构造性可行条件。
可行性定理的配网适用域已收紧为根森林兼容输入：有效径向配网，或为健康闭环提供可操作分段设备。
缺少分段设备元数据的闭环不会由优化器凭空开线，而会把径向化失电显式计入 N-0 原始削减。新增唯一
发电机故障回归覆盖“无源、无可用 slack、全切负荷仍成功求解”的三阶段构造性可行点。

进一步按构造性证明逐式审计发现：旧模型仅以
$q_i^{sh}=(Q_i^d/P_i^d)p_i^{sh}$ 隐式表示无功削减，且母线负荷导入会跳过
$P_i^d=0,Q_i^d\ne0$ 的纯无功负荷；唯一源故障后因此缺少 $q_i^{sh}=Q_i^d$ 的可行点。现实现保留
$P_i^d>0$ 时的代数消元，只对纯无功母线增加
$\min(0,Q_i^d)\le q_i^{sh}\le\max(0,Q_i^d)$，并在母线失电时强制完全削减。这样既补齐任意有效
故障的无功平衡构造，又保持既有 MILP 矩阵及求解器原始间隙不变。新增纯无功唯一源故障反例后，
`test_three_stage_reliability` 为 38 个用例、1904 个断言全部通过；完整 `macos-release` 回归完成
1611 项注册测试调度，其中 1608 通过、3 条件跳过、0 失败，总耗时 228.47 秒。

### RTS-24 拓扑—求解一致性修复

已定位并修复一个会系统性抬高可靠性指标的表示错误。MATPOWER 对带非单位变比的
支路保留精确 `ACBranch`，同时生成带 `source_branch_idx` 的 `Transformer2W` 元数据。
旧实现把两者都放入拓扑图，但 DC-OPF 只使用 `ACBranch`；当原始支路停运、元数据边
仍在线时，拓扑预筛认为负荷仍处于有源连通岛，DC-OPF 却面对断开的实际 B 矩阵并失败，
随后回退逻辑把剩余负荷全部计为切负荷。反向状态还会把元数据误当成独立随机故障。

现行不变量为：`source_branch_idx > 0` 的变压器行只用于映射和结果归因，不参与图边、
孤岛子系统边或独立蒙特卡洛元件集合；其物理状态完全由对应 `ACBranch` 决定。图构建、
孤岛检测、可靠性状态向量和子系统复制均已采用同一规则。HL-II DC-OPF 不再检查输入 slack、
`NoSlack`、在线源或全局机组数；每个连通分量任取角度参考，在线机组采用 `0<=Pg<=Pmax`，
并执行“先最小切负荷、再最小 PWL 发电成本”的两阶段字典序 LP；对静态电源、可再生、PV、
储能和负净负荷额外引入有界固定电源削减 `pgc`，因此固定注入过剩也有显式可行点。任何未收敛
或规范坐标后验证书失败的状态立即抛错，不写入 EENS；混合 AC/DC FMEA LP 同样执行两阶段证书，
删除数值失败全切负荷回退。

RTS-24 可靠性数据已从按母线复制改为 MATPOWER 33 行机组、38 行支路逐行映射，并校验
母线/Pmax/端点签名；并联支路不再共用首个端点匹配值。确定性 `relscan` 枚举 71 个导入记录
（33 行机组，含 1 行零有功同步调相机；38 条物理支路）的 N-0/N-1/N-2 共 2557 个状态，
实测 2557/2557 收敛，非有限、不可行、负削减、超总负荷削减均为 0，最大功率平衡/约束违反
为 0 MW，最大状态削减 245 MW。当前 `test_reliability_resolver` 为 88 个用例、704 个断言，
全部通过。逐台数据 MC 诊断得到 NSQ EENS=123156.8 MWh/年、LOLE=738.5 h/年、CoV=0.0314；
SEQ EENS=1222.6 MWh/年、LOLE=10.54 h/年、CoV=0.1152；两者 `OPFFailedP/EENS=0`，但均未
达到各自预设停止条件，故不作为最终收敛基准发布。

顺序 MC 现行字段口径已固定：`eens_mwh_yr`/`annual_eens` 为逐小时总切负荷（含 N-0 基线），
`baseline_eens_mwh_yr` 为同负荷曲线健康状态缺额，`incremental_eens_mwh_yr` 为相对 N-0 的故障增量；
LOLE/LOLF 按总切负荷标志统计。N-0 缺额反例与逐负荷空间因子反例均已注册回归。

可靠性模块的现行理论均已落到可调用代码、结果字段和注册测试，不再以路线图充当
功能。闭环范围包括：不分箱 COPT 精确状态概率与 F&D 有效性诊断、独立两状态
串并联约化、IEC 61508 低需求 PFD、含 VaR 原子分数权重的经验期望短缺、带完整性
诊断的 N-2 二阶交互、物理与信息最小割集、共享依赖信息路径与 QoS、联合功能
可用率、互斥信息功能类、有限 POMDP 精确信念树，以及静态 FMEA、仅保护、
信息物理联合三种方法的同口径 EENS/LOLE/LOLF 对照。

保护与动态链实际执行 CT/PT 一阶动态和饱和、定时限/反时限、方向、mho 与四边形
距离、差动、主后备配合、断路器失灵、重合器—熔断器—分段器序列、自动保护
拓扑、径向失电岛负荷退出、IEEE 1547 DER-FRT、瞬时闭锁、微网同步窗、信息
共因、物理供电和备用电池。三级对照既可消费给定轨迹，也可执行在线 Mass-Matrix
DAE；在线模式对主、后备各做发现和动作反馈两遍动态计算，并把实际轨迹、清除
时刻和 DER 终态送入年度事件树。HTTP 与 GUI 返回 DAE/事件树时间一致性、轨迹
点数、动作反馈、FRT 消费及模型限制，不以 HTTP 200 代替物理有效性证明。

三阶段 MILP 的零目标相对间隙修订遵循“明确最优终止证书强于派生相对量”。修订前
预测：`case33mg_acdc` 支路 2 的阶段 2、3 应由近似改判为已认证最优，认证间隙从
1 归一为 0，后端原始间隙仍为 1，SAIFI/SAIDI/EENS 数值不变，36/36 用例通过。
实测与预测一致：36/36 用例、1887/1887 断言通过，SAIFI=0.7580 次/(户·年)、
SAIDI=7.20 分钟/(户·年)、EENS=440.0 kWh/年；后端状态为 StrictHiGHS Optimal，
认证间隙为 0，原始间隙为 1。未触发理论重新推导阈值。

当前 `macos-release` 专项回归重新实测：`test_reliability_resolver` 为 79 个用例、
613 个断言；`test_intelligent_cyber_physical_reliability` 为 7 个用例、60 个断言；
`test_three_stage_reliability` 为 36 个用例、1887 个断言；
`test_resilience_assessment` 为 39 个用例、360 个断言。合计 161 个用例、2920 个
断言全部通过。已注册浏览器测试 `reliability_workflow_e2e` 与
`reliability_configuration_e2e` 全部通过，实测分别为 18.98 秒和 52.70 秒；覆盖
在线 DAE HTTP/GUI、精确灵敏度、重要抽样、三级对照、参数稀疏保存/回读和
390×844 移动视口无横向溢出。

闭式三级基准的静态、仅保护、联合 EENS 分别为 200.0、0.4007777778、
21.5217333333 MWh/年，对应 LOLE 为 20.0、0.2000777778、4.1600622222 h/年。
保护误动基准为 0.4 次/年，EENS、LOLE、LOLF 增量分别为 0.2 MWh/年、0.1 h/年、
0.4 次/年，分解残差为 0。有效故障率同时报告运行时间条件强度和日历年事件频率，
并由 `(1-U)*lambda_up=f_calendar` 回归验证，避免把两种口径直接混用。

中文可靠性手册当前为 112 页 A4；XeLaTeX 干净编译无致命错误或溢出版心警告，原 107 页基线曾以
Poppler 全页渲染验证，本次新增/受影响的第 24--27、48--49 页以 120 dpi 重新渲染目视检查。
联系表和封面、求解证书、保护、信息系统、在线 DAE、三级对照、参数表、验证边界、
末页均已目视检查，无裁切、重叠或英文生成附录。可靠性手册、总理论文档及可靠性
源码的悬空状态词扫描为零，新增代码的任务标记、占位和固定伪值扫描为零。

明确拒绝边界保留：在线保护是正序网络和单相故障输入，不认证三相 EMT、行波保护
或 CT 磁滞；年度后果使用三窗口模型；三阶段恢复入口拒绝 LCC 和多端口能量路由器，
不伪造零影响结果。活动兄弟仓库 MIPSolvers 为 `7721936245d...`，仓库记录 pin 为
`3bf1e66749e...`，因此上述结果只证明当前本地 Release 工作区，不能声称依赖锁定
可复现。

## Module manual theory layer (in progress)

The 23 module manuals are being upgraded from code-transcription-only to a
two-layer structure (general mathematical theory + implementation
correspondence), as the foundation for a planned monograph. Infrastructure
landed 2026-08-19: `docs/_manual_common/theory_environments.tex` (amsthm
theorem family, `theorynote`/`gapnote` boxes, `\implfull/\implpart/\implnone`
status tags), auto-included by `hysim_manual.sty`; `docs/modules/README.md`
math-model rule 3 revised to permit general theory in `chapters/theory_*.tex`
under strict marking rules; writing spec in
`docs/modules/THEORY_WRITING_GUIDE.md`. Depth tiers: monograph-grade for
power_flow, optimal_power_flow, dynamics, sppt, graph; engineering-reference
grade (≤1200 lines each) for the rest. Rollout: short_circuit pilot first,
then waves W1–W5; existing implementation chapters must not be modified.

Pilot (short_circuit) accepted and W1 complete (2026-08-19): short_circuit
gained 3 theory chapters (665 lines: symmetrical components, IEC 60909
method, converter/DC fault); power_flow gained 4 monograph-grade chapters
(~2900 lines: fundamentals+Newton+sparsity, classic methods, globalization+
HELM/homotopy/NCP, voltage stability+converter theory); graph gained 1
monograph-grade chapter (664 lines). All compile clean under XeLaTeX with
zero line-number anchors; each chapter carries an implementation
correspondence table (verified anchors) and gapnote markers for
unimplemented theory.

## SPPT Applied Energy manuscript and fault campaign

`docs/latex/sppt_theory.tex` is now the sole Applied Energy-oriented SPPT
manuscript source. Its research question is the verifiable safety boundary of
automated hybrid AC/DC digital-twin construction, rather than a generic
AI-assisted software framework. The rewritten abstract, introduction-based literature positioning,
contributions, numerical study, discussion, and conclusion use a fixed-seed
statistical campaign instead of the previous six isolated modifications as primary
evidence. `docs/latex/sppt_campaign_study.tex` records the protocol and results.

The theory now defines SPPT as an analysis-indexed family of projections rather
than a catalogue of fault checks. Formal statements and proofs cover the exact
projection fixed-point property, ideal-connection contraction,
intensive/extensive reconstruction, compositional preservation,
internal/independent residual separation, and AC reference singularity. The
reverse ideal-connection result explicitly requires class balance for retained
member injections and distinguishes unique tree currents from cycle-flow
non-uniqueness. Representation invariance, transformation composition,
observable-dependent reconstruction, conservation/topology preservation, and
converter-role closure provide extension conditions for new data
representations, devices, controls, reductions, and analyses. The six injected
structural classes are selected tests of these conditions, not a completeness
definition. The introduction now supports its model-heterogeneity, calibration,
network-equivalence, AI, and anomaly-detection claims with 37 cited sources;
the bibliography is ordered by first appearance. Repeated defensive caveats
were consolidated into a positive statement of demonstrated domain and transfer
path in the discussion.

The new reusable `sppt::run_fault_campaign` implementation and
`sppt_scale_fault_campaign` target import seven public GridLAB-D r5643
distribution feeders and attach the same fully disclosed two-terminal DC/VSC
overlay. The resulting 32--6986-AC-bus variants evaluate nine edit classes,
five detector chains, localization, 95% Wilson
intervals, independent residuals, physical AC/DC consequences, and stage timing.
The completed seed-20260818 run contains 1,575 samples: 175 randomized valid
controls, 1,050 structural faults, and 350 plausible intention errors. Full SPPT
detected/localized 1,050/1,050 structural faults (pooled 95% Wilson lower bound
0.9964) and rejected 0/175 controls. Convergence-only screening missed all tested dangling VSC
terminal, missing-DC-support, and invalid-efficiency faults and detected 4.0% of
duplicate identities in its prespecified 32/78/330-bus scope. The 200
intention-error samples solved through the 823-bus variant produced maxima of
0.00424 p.u. AC-voltage deviation, 0.526 MW branch-flow deviation, and 0.0322 MW
VSC-transfer deviation. The VSC fault generator selects
only in-service PQ controls and applies a nonzero signed perturbation, so no
no-op setpoint samples inflate the count. These results explicitly bound
SPPT: structurally consistent but incorrect parameters require telemetry,
parameter records, state estimation, or human confirmation.

The final seed-20260818 campaign was run twice: all 1,575 non-timing rows matched
field-for-field, while the four wall-clock columns were excluded from the
determinism comparison. Publication figures are regenerated directly from the
CSV and manifest by `tools/sppt_campaign_plots.py`, whose data assertions cover
case count, bus ladder, solver scope, structural rates, and impact statistics.

The initial campaign exposed degenerate `vmin_pu == vmax_pu == 1.0` slack-bus
limits in the case33bw/case69 hybrid builders. `parse_case_with_overlay` now
expands only degenerate imported intervals around the authored setpoint; the
dedicated validation target passes 178 assertions. The campaign regression
passes 19 assertions. Full campaign correctness was exercised in an independent
Debug build with ETAP, Ipopt, and SuiteSparse disabled because the local sibling
MIPSolvers worktree is dirty; therefore the recorded Debug timings are diagnostic
only and no reproducible Release-performance claim is made. The manuscript
and six-page Elsevier-style supplement compile under TeX Live 2026 without
undefined references. Rendered inspection covers the first page, all three
main-text numerical figures, the public-case manifest, the paginated
theory-to-code crosswalk, and the complete landscape fault table; no figure or
table is clipped. Existing overfull diagnostics in the long theory sections do
not originate from the new numerical study and remain a later manuscript-
compression task.

The main manuscript now uses four generated mechanism figures rather than
software-flow TikZ diagrams: physical rich-model semantics and canonical
projection, provenance reconstruction, converter-role closure, and typed-agent
candidate-state isolation. The converter-role mechanism was checked against the
DC nodal equations and is described as a structural role-closure/rank condition,
not as a universal voltage-shift invariance. The strengthened 19-page,
two-column PDF was compiled to
`build/latex-sppt-theory-strengthened-v2/sppt_theory.pdf`; all pages were
rendered at publication scale and show no clipping, overlap, or overfull
content. The log contains no undefined references, duplicate labels, or fatal
errors.

The internal geometry of manuscript Figs. 1, 2, 4, and 5 was subsequently
reworked at the source-script level. Status labels and reconstruction paths in
Fig. 1 now have separate lanes; Fig. 2 uses separated terminal labels, device-ID
tracks, and one-to-one reverse-attribution rows; Fig. 4 separates evidence and
acceptance-gate labels from the panel headings and decision paths; and Fig. 5
routes the DC connection below the node labels. The regenerated vector figures
were checked both individually and after full-width placement in the 19-page
two-column manuscript. The current PDF is
`build/latex-sppt-theory-layout-v3/sppt_theory.pdf`.

A final publication-scale pass adds explicit safety margins at the remaining
near-contact points: the Fig. 1 reconstruction lane is horizontal and detached
from its label, Fig. 2 attribution arrows terminate before the asset boxes, Fig.
4 evidence text, arrows, gate title, and gate line occupy separate regions, and
Fig. 5 separates its DC-line path from both node and parameter labels. The
corresponding manuscript build is
`build/latex-sppt-theory-layout-v4/sppt_theory.pdf`.

## 技术委托要求文档

已根据当前源码、公共接口、GUI、数据交换能力和已验证测试基线完成
`技术委托要求V2_已填充.docx`。文档明确覆盖交直流及三相潮流、OPF、
RPO、CPF、短路、谐波、动态、小信号、可靠性、弹性、重构、时序、市场、
碳流、HTTP/Python 接口和 GUI，并对可选 ETAP/OpenDSS 依赖、近似模型与
能力边界作出限定；未将 DLL 热插拔、PSCAD/PSASP 原生导入或通用自动参数
辨识表述为现有功能。原始模板保持不变，填充副本经 OOXML 结构检查、
占位符检查和 Microsoft Pages 全 16 页 A4 渲染复核，未发现裁切、重叠或
表格破损。

## Verified baseline

| Scope | Result |
|---|---|
| Required MIPSolvers source | Current pin `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`; the clean sibling worktree matches it and includes the PF KLU numeric-refactor interface/adapter. The PF regressions below used the same three-file dependency content before it was committed. The earlier general full-regression baseline was established at `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0`. |
| Current clean `macos-release` build | At repository `1773aa0e75d7` with MIPSolvers `3bf1e66749e3`, `cmake --preset macos-release` followed by `cmake --build --preset macos-release --clean-first` completed successfully on macOS 26.5.2 / Apple M4 Max. The precompiled MIPSolvers dependency manifest did not match the active ABI, so configuration correctly used the repository-vendored HiGHS, SCIP, Ipopt, and SuiteSparse instead. |
| Current complete `macos-release` regression | After the reliability raw/incremental EENS and DCOPF status fixes, `cmake --build --preset macos-release -j8` completed and `ctest --preset macos-release --output-on-failure` ran all 1609 registered tests in 225.45 s with 0 failures: 1606 passed and 3 were conditionally skipped. The skips are one unavailable formal-SOC JSON input and two unavailable external GridLAB-D comparisons. |
| Current physical-consequence audit | The registered suite now contains 1611 tests after adding the only-source-outage and pure-reactive-load constructive-feasibility cases. `test_three_stage_reliability` passed 38 cases / 1904 assertions. The final complete `ctest --preset macos-release --output-on-failure` rerun completed all 1611 registered tests in 228.47 s with 0 failures: 1608 passed and 3 were conditionally skipped. The skips are one unavailable formal-SOC JSON input and two unavailable external GridLAB-D comparisons. The earlier focused main-grid HL-II, hybrid consequence, and three-stage selection passed 39/39 in 13.42 s before the pure-reactive case was registered. |
| `full-dev` regression | 1430/1430 registered tests completed without failure; 3 condition-dependent tests skipped |
| Earlier green `macos-release` regression | 1425/1425 registered tests completed without failure; 3 condition-dependent tests skipped |
| Graph ASan/UBSan subset | 28 cases, 113 assertions passed after the iterative Tarjan fix |
| Reliability ASan/UBSan | Complete three-stage suite 25 cases/1339 assertions; `case33mg_acdc` 477 assertions and five consecutive parallel repeats passed; `test_1_no_sop` 79 assertions |
| Market ASan/UBSan | Complete suite 22 cases/845 assertions; focused initial root-cut case 1/42 |
| Other sanitizer subsets | Thread pool 4 cases/6 assertions; `test_hacdcpf` 26/89; `test_acopf_dcopf_crossval` 13/119; `test_power_flow_math_audit` 42/231 |

## GUI scale-out: persistent indexes, SoA WebGL overview, session sharing, topology window

Four-part scalability enhancement landed in the working tree (uncommitted):

1. `web/js/canvas.js` gained persistent indexes: `state.componentById` (Map,
   maintained in add/remove/clear/load paths) and `state.connectionsByEndpoint`
   (per-`compId`/`compId:portId` buckets). `getComponent` is O(1); connection
   rescans in add-connection dedup, `syncConnectivity`, `buildSystemJson`,
   drag re-routing, and the visualization layer now use bucket lookups (drag
   re-route is O(degree) per frame instead of O(E)). `getCompBusMap()` is
   cached with lazy invalidation on topology edits; `resultRowsByIndexOrOrder`
   pre-builds key→row-queue maps (O(C+R)).
2. `web/js/core/network_overview.js` is SoA: preallocated `Float32Array`
   node/line/transformer buffers with 2× geometric growth; selection updates
   merge dirty slots into a single `gl.bufferSubData` (no full
   `rebuildBuffers` on click); hit-testing uses a uniform grid (~4× mean
   spacing, ≤256 cells/axis) keeping the LOD2-only pick semantics and the
   `{domain, index}` identity contract.
3. `tests/run_gui_server.cpp` Session now holds
   `std::shared_ptr<const HybridPowerSystem>`; all 14 whole-system assignment
   sites funnel through `session_replace_system()`, which atomically swaps the
   pointer, rebuilds the resident `PowerSystemGraph` and bus uniform-grid
   spatial index, and invalidates the compact-JSON serialization cache.
   `/api/session/pf` keeps a request-private copy (it must mutate) but copies
   outside the lock; `parameter_library/apply`, `design_handbook/apply+preview`,
   `update_carbon_factors`, and `set_ts_config` materialize-then-replace. All
   16 `_raw_json` embeds were verified to be on frontend-consumed paths and
   kept, now served from the cache; `io::to_json_dom()` eliminates the
   `to_json`→`json::parse` double conversion in `system_summary` and friends.
4. New `POST /api/session/topology_window` (schema `topology_window_v1`):
   bbox query over WGS84 coordinates with optional `lod` 0/1/2 aligned
   line-by-line with the frontend `aggregate()` semantics; responses carry
   `units`, `coordinate_coverage`, and `model_limitations` (missing-coordinate
   buses are declared, never silently dropped). `/api/session/status`
   capabilities report `topology_window_v1: true`.

Verification: `node --check` on all touched JS; vm-based canvas index smoke
plus 500 randomized `resultRowsByIndexOrOrder` equivalence cases;
stub-WebGL2 overview smoke (selection = 0 `bufferData` + 1 `bufferSubData`);
`cmake --build --preset macos-release --target run_gui_server` clean;
`ctest -R 'gui_api|topology_window|runtime_api_v1'` 3/3, `ctest -R gui` 9/9,
JSON round-trip subset 74/74. Browser-interaction E2E (chromium) not yet run
for the new index/overview paths.

Follow-up (same session): the remaining ~40 analysis endpoints were audited
one by one — 23 verified read-only (const-ref solver signatures, compile-
verified zero mutation) now bind the shared snapshot with no copy;
18 endpoints that genuinely mutate a working copy (`solve_power_flow`'s
in-place canonical projection, grid-forming converter translation,
`assign_available_default_profiles`, template reliability data application,
operating-point write-back in opf_ac/parity/dc, etc.) copy from the snapshot
after releasing the lock; `run_carbon` now computes off-lock and
`last_pf_system` is itself a shared snapshot; `export_json` serializes
off-lock with the response byte-identical (indent=2). `DCBus.area/zone` now
round-trip through `io::to_json`/`from_json` (defaults 0), so
`topology_window` lod=1 DC groups key by real values; the E2E assertion was
updated accordingly. The v1 multi-session API (`src/server/runtime_api_v1.cpp`)
cannot share the GUI session's resident indexes — its `ApiSession` store is a
separate, revision-tracked model space — so it instead caches the LOD2
topology DOM per session, invalidated by `revision`; `/topology` and
`/subgraph` no longer re-serialize the model per request. Still open: the v1
job-frame viewport filter (`frame_chunk`) rebuilds LOD2 positions per request
when a viewport is supplied (job-level cache not added); browser E2E
(chromium) still not run for the new paths.

Follow-up (result window): new `POST /api/session/result_window` (schema
`result_window_v1`) serves per-element PF results inside a WGS84 bbox from the
cached `last_pf_result` + `last_pf_system` — both now shared snapshots, so the
route copies nothing.  Session gains `system_revision`/`last_pf_revision`;
the route reports `result_matches_current_system` and declares a lag in
`model_limitations` when the model changed after the solve (the window then
falls back to a linear scan of the solved system's own coordinates instead of
the resident spatial index).  409 `no_cached_power_flow` when no PF exists.
Units/limits are declared in `units`/`model_limitations` (vm/vdc pu, va rad,
MW/MVAr, AC branches only).  Measured on ACTIVSg25k with synthesized
coordinates: world window (25,000 nodes + 32,230 branches) 78.6 ms/10.7 MB —
vs 3.4 s/106 MB for the full PF response — and a ~2% viewport 1.8 ms/65 KB.
E2E: `tools/gui_result_window_e2e.py` (23 assertions: 409, bbox filter,
coverage declaration, sampled value identity with the full PF response, stale
flagging, hybrid DC nodes).

Follow-up (result_window aggregation + legend): `result_window` accepts an
optional `lod` (default 2); lod 0/1 reuse the exact `topology_window` grouping
keys/centroids and return aggregate nodes `{group, count, vm_avg, vm_min,
vm_max}` and aggregate edges with `loading_pct` = max of collapsed members
(conservative; declared in `units`/`model_limitations`). The overview now
colors aggregate LOD0/1 views (worst-deviation vm per group) and shows a
result legend; color scales were hoisted to shared single-source constants.
`gui_result_window_e2e` grew to 42 assertions.

## GUI usability rounds (P0/P1/P2)

P0 (editing safety net): `canvas.js` gained an undo/redo command stack (6
primitives + composite commands, capacity 200, structural edits only —
property-panel edits are not undoable yet), component copy/paste/duplicate
(Ctrl+C/V/D, internal clipboard, +20px grid-aligned paste offset), and
multi-select group drag (one composite undo per gesture). `app.js` gained a
toast system (`App.toast`), a problems panel (`App.reportProblem` with
click-to-locate via the existing `panToComponent` infra), beforeunload
protection on `_canvasDirty`, and a 30 s localStorage canvas draft
(`hysim.canvasDraft.v1`, skipped in headless mode and above ~4 MB).

P1 (data browsing): `enhanceResultTable` progressively upgrades result tables
(power flow, OPF, short circuit, market, reliability) with three-state
numeric-aware header sort, per-column filters, and CSV export of the filtered
rows (RFC-4180 + BOM); editor tables and virtualized tables are skipped by
rule. The property panel gained on-input numeric validation (ranges only from
backend schema/catalog, never hard-coded), unit suffix spans from a whitelist
derived from existing labels, and blur-time JSON pre-validation. A new
`core/help_panel.js` provides a `?`-opened shortcut cheat sheet (cross-checked
against the actual canvas keydown handler) and wires the toolbar undo/redo
buttons.

P2 (onboarding/a11y/result viz): first-visit 7-step onboarding tour
(`hysim.tourDone.v1`, spotlight overlay, skip/Esc, missing-target step
skipping), a top-bar help menu (replay tour, shortcut panel, example
templates), and two example templates (`web/examples/ac_radial_feeder_example.json`,
`hybrid_acdc_microgrid_example.json`, field sets copied verbatim from
`data/simple_case.json` / `data/dsp/cigre.json` and verified loadable through
`/api/session/load_json_string`). Arrow-key nudge on the focused canvas
(20 px grid, Shift = 1 px, burst coalesced into one undo command), modal focus
trapping in `core/accessibility.js`, and a live zoom-percentage indicator.

Regression lesson: the tour's first-visit auto-start intercepted ArrowRight
and pointer events in fresh Playwright contexts and broke 4 browser E2E
(`pf_ncp_schur_gui_e2e`, `gui_scale_features_e2e`, `market_gui_e2e`,
`rpo_gui_e2e`). All 15 browser E2E files now pin
`localStorage['hysim.tourDone.v1']='1'` via `addInitScript` right after
`newPage`; any future first-visit UX must be disabled the same way.
`ctest -R e2e`: 17/17 after the fix.

Verification (all three rounds): `node --check` on every touched JS file;
8 node vm+DOM smoke harnesses under `tmp/` (undo, indexes, feedback, help
panel, result table, tour, focus trap, WebGL SoA) all green; `ctest -R e2e`
17/17; `ctest -R 'gui|topology_window|result_window'` 10/10.

## Balanced VSC current-limit NCP

The ordinary `case2000_acdc` built-in now sizes its four Vdc-Q stations from
the aggregate four-by-0.5 pu fixed-P transfer and the centralized 5% Vdc droop
band instead of the infeasible authored `k_vdc=0.1`. With automatic fallback
explicitly disabled, the registered production hybrid Newton regression
converges in 37 iterations with residual `7.263e-9` and DC voltages
`0.9491--0.9499 pu`; pure-AC convergence is not accepted. The rebuilt browser
test also loads the ordinary built-in and clicks the normal PF button. Its
sparse `后端默认` request omits the fallback override, the backend resolves it
to enabled, and the observed solve converged in 5 iterations with residual
`7.633e-10` and 100% voltage qualification. The same test continues to cover
the GFM NCP/Schur expert controls. The focused Release target now passes 47
cases and 563 assertions, and the registered browser E2E passes with no page
overflow.

The production unified Newton path now has an opt-in fixed six-state local
block per supported `PQ_MODE`, `VDC_Q`, or grid-connected `AC_GRID_FORMING`
converter. The shared `(Pac,Qac,Pdc,Er,Ei,lambda)` layout uses identity internal-
voltage slots for power-port modes and an explicit internal voltage behind
virtual impedance for GFM. It enforces the AC current disk,
Magnitude/P-first/Q-first policy, energy balance, and saturated Vdc-droop
command. Network and local generalized Jacobians, including terminal-angle
derivatives, are analytic and retain one fixed sparse layout across activation.

The complete Release `test_vsc_limit_ncp` target passes 46 cases and 550
assertions. It covers three priorities, the full local/terminal Jacobian
finite-difference check, two simultaneous GFM limits, zero-radius priority
ties, weak-grid and infeasible points, three-step quasi-steady replay, balanced
OPF dispatch replay, two-level OpenDSS validation, legacy power-port oracles,
nonbinding equivalence, Vdc saturation, structural admission, standard/JPC
round trips, stable identity, shared parameter precedence, the production
demo, sole/multiple slackless GFM islands, adaptive island extraction, and the
case300/ACTIVSg2000 families. Exact FB now satisfies the biactive origin rather
than adding a hidden epsilon. Optional smooth FB/CHKS continuation covers all
three power-port priorities and all three current-limited GFM priorities; the
measured solves took 16--25 iterations with 11 continuation updates, retained
one Jacobian pattern build, and returned exact residual certificates between
`1.0e-12` and `3.6e-10`. The three-phase hybrid OPF target
passes 12 cases/152 assertions, including rich-to-phase GFM parameter mapping.

The focused transient initialization contract passes 4 cases and 51
assertions. Its primary 15-assertion case certifies the stable-ID PF-to-dynamic
Norton seed before network trimming: internal voltage, current, and power
errors are each at most `1e-8`. A companion executable guard proves that the
legacy authored fallback resolves identically across PF and dynamics, while an
explicit transient profile override invalidates that continuity certificate
and emits a warning. This seed certificate is distinct from the later
full-dynamic equilibrium, which may move when dynamic device equivalents do
not reproduce the PF slack dispatch. The added case starts without a terminal
SLACK and certifies the stable GFM Norton seed through dynamic construction.

The full `tools/gui_api_e2e.py` run passed all production GFM/NCP HTTP
checks, including the Schur-policy request/effective-options round trip. The
`gfm_norton_limit_demo` built-in case loads with 2 AC buses, 2 DC
buses, and 2 VSCs; it preserves all four authored Norton fields through GUI
editing and returns stable Canvas references, local certificates, and the GFM
validity flags. The current registered GUI API E2E test passes, including
hybrid Auto OPF recovery, the GFM demo, and case300 NCP solves. Four
simultaneous converter limits are active and the maximum reported NCP residual
is `1.5543e-15`.

The balanced-PF GUI now exposes smooth-NCP continuation and local-VSC Schur
admission in an expert advanced group. Blank controls preserve backend-owned
defaults; completed solves render normalized effective values and a separate
`linear_structure` certificate with admission/fallback status, dimension and
structural-nonzero reduction, `rcond`, backward errors, and local rate samples.
The versioned `/api/v1` PF path accepts and echoes the same policy. The focused
Release `test_vsc_limit_ncp` rebuild passes 46 cases and 550 assertions after
adding legal/invalid smooth-policy normalization. The registered Playwright
`pf_ncp_schur_gui_e2e` passes against the rebuilt Release server: it drives the
real GFM case through the GUI, verifies sparse blank-field request semantics,
effective-policy echo, the runtime certificate and VSC NCP table, and observes
zero page-level overflow at both 1440x1000 and 390x844. It now also edits the
expert policy after a completed solve and verifies that the previous
certificate becomes explicitly stale, then reruns Schur-on and Schur-off to
observe fresh `7 / 7 / 0` and `0 / 0 / 0` certificates. On the ordinary
`case2000_acdc` WebGL path it clicks an AC topology row plus AC-bus, DC-bus,
and VSC result rows and verifies domain-qualified overview selections. The
focused rebuilt Release browser set
`pf_ncp_schur_gui_e2e|topology_transformer_link_e2e` passes 2/2. The rebuilt versioned
Runtime API v1 E2E also passes its complete session/job/topology workflow with
the same effective policy round trip.

Two versioned MTDC/PQ-Vdc-Q benchmark builders prevent pure-AC evidence from
being misreported as converter-limit scalability. The registered case300 solve
has 300 AC buses, 6 DC buses, and 6 explicit VSC blocks; ACTIVSg2000 has 2000
AC buses, 8 DC buses, and 8 explicit blocks. Each has four simultaneous
power-port limits. Separate GFM builders solve that real MTDC benchmark first,
retain its voltage state as an explicit continuation seed, and add one
nonbinding Norton GFM block; their focused tests each pass 13 assertions. They
prove fixed sparse structure on real hybrid networks, not GFM flat-start
robustness, large-scale simultaneous GFM binding, or GPU acceleration.

The fixed two-warmup/five-repeat Release/KLU Schur benchmark was rerun after
the final all-suite fixes. Forced case300 Schur reduced dimension `640 -> 604`
and structural nonzeros `4820 -> 4502`, but increased median linear time from
`0.524958` to `0.689582 ms` (`31.36%`) and wall time from `2.522875` to
`3.353792 ms` (`32.94%`); the production default therefore correctly keeps
this case on full LU. Relative to the preceding run, the linear penalty is
1.64 percentage points smaller and the wall penalty is 15.48 points larger.
ACTIVSg2000 reduced `4054 -> 4006` and `29806 -> 29382`; median linear time
fell from `31.979500` to `27.581291 ms` (`13.75%`) and wall time from
`47.372583` to `42.373500 ms` (`10.55%`). Relative to the preceding run, these
speedups are 0.10 and 1.44 percentage points lower. The large-case benefit
keeps the predicted sign, and no measurement deviates by the predefined 50%
re-derivation threshold; the small-case wall overhead remains an explicitly
excluded ablation rather than a production regression. The Schur enable flag,
network-dimension admission, local `rcond`,
and backward-error tolerances now round-trip through the production PF HTTP
configuration. Invalid dimensionless tolerances restore named defaults; the
global machine epsilon remains read-only.

The admitted GFM scope is balanced positive-sequence steady/quasi-steady
operation. A slackless island retains all terminal `Vm/Va` variables and is
anchored by one or more authored fixed internal GFM Norton phasors; no terminal
SLACK is manufactured. Strict coordination still requires physical DC-side
voltage/power support, and the local Newton method does not claim uniqueness
across high- and low-voltage basins. Balanced OPF does not put the
GFM internal-voltage/priority NCP inside its KKT system: the production PF
`post_pf` replay preserves GFM mode and provides the certificate. OpenDSS
independently reproduces the nonbinding Thevenin root; at a binding point it
freezes the HySim internal voltage and re-solves the circuit with native
terminal-power KCL residual at most `1e-6 pu`, which is not presented as an
independent mode-selection oracle. GPU assembly/factorization remains
unimplemented. The concrete authored/canonical/solver/result/replay ownership
rules are in `docs/developer/model_data_semantics_contract.md`; the equations and
numerical boundaries are in `docs/theory/vsc_limit_ncp_power_flow_contract.md`.

The repository-wide semantics guard `test_model_semantics_contract` passes 5
cases and 1531 assertions. It pairs all 43 component I/O collections with
runtime identity/unit/sign/model-fidelity contracts, covers all 22 top-level
production source modules, and locks shared `DCStorage`/`StaticGeneratorDC`
execution views plus additive bus/component demand. This audit found and fixed
loss of `StaticGeneratorDC.cost_c1` in both SolverData construction paths. The
existing component I/O registry also passes 14 cases and 373 assertions.

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

## OPF Phase I/II integration and performance

The checked-in dependency pin is MIPSolvers `3bf1e66`; its clean `main`
contains the audited central warm-start and NativeLCQP cooperative-deadline
contracts used by this HySim change, plus the guarded KLU numeric-refactor
adapter used by PF. Bounded Phase I fronts balanced Parity Native IPM (pure AC
and hybrid AC/DC) and monolithic three-phase hybrid NativeIPM. Pure AC keeps
the state/basic PF-Newton basis. Hybrid models first eliminate converter
`Pdc` directions and solve the DC-conductance/converter Schur block, then use
the same state/basic fallback when the global admission test passes. It applies sparse
constraint Newton steps in assembled OPF coordinates, then performs one
equality-dual and positive inequality-dual/slack fit. A declared state/basic
square Jacobian uses SparseLU; unusual component mixes or active nonlinear
inequalities fall back to direct SparseQR. No dense Jacobian, `J J^T`, barrier
schedule, filter, Hessian solve, second-order correction, elastic variable, or
Ipopt call is part of finite-budget Phase I.

The coordinated defaults are `mu0 = 0.1`, admission ratio `eta_a = 1.0`,
primal handoff ratio `eta_p = 0.1`, and centrality tolerance `eta_c = 0.5`.
Thus Phase I is attempted only when the original-coordinate violation is at
most `eta_a * mu0 = 1e-1`; its finite-accuracy handoff target is
`max(final_primal_tolerance, eta_p * mu0) = 1e-2`. Phase II's final primal,
dual, and complementarity tolerances are unchanged. A handoff may therefore be
inside the Phase I corridor without satisfying the final solution tolerance;
`phase_one_in_handoff_corridor` and `phase_one_primal_feasible` report these
facts separately.

MIPSolvers accepts the handoff only under filter globalization with complete,
finite primal/equality-dual/inequality-dual/slack vectors, positive inequality
duals and slacks, original and perturbed primal residuals within the corridor,
`mu0 <= 0.1`, centrality
`max_i |s_i z_i / mu0 - 1| <= eta_c`, and maximum inequality dual at most
`1e4`. Full dual residual is diagnostic rather than an admission gate because
Phase I deliberately leaves the reduced gradient to Phase II. Rejection is
all-or-nothing: every warm vector is cleared and ordinary interiorization is
used. Diagnostics include the audit residuals and reason plus first accepted
step lengths and barrier-objective change. Fixed-variable reduction preserves
the nonlinear, nonfixed-lower-bound, then nonfixed-upper-bound row order.

Iteration, total primal-plus-dual factorization, and per-iteration backtrack
caps are hard work bounds. The wall-clock deadline is cooperative because an
in-flight sparse factorization cannot be interrupted. The corridor-aware slack
floor `max(2e-10, (epsilon_p - violation) / 2)` keeps `s_i z_i = mu0` without
creating the extreme multipliers observed with a uniform `2e-10` floor. That
original construction increased Phase II from 17 to 35 iterations on case30
and from 13 to 35 on the hybrid microgrid; the revised scaling removed the
regression.

The reproducible `phase_one_restoration_benchmark` protocol uses two warmups
and five timed repetitions. On AppleClang 21, arm64 macOS, Release, case30 AC
improved from `4.458 ms` off to `4.225 ms` on (`1.055x`), while Phase II fell
from 17 to 16 iterations and from 16 to 15 factorizations; Phase I reduced
violation `3.106e-2 -> 4.030e-4` with centrality `2.22e-16`. The 360-bus
three-phase case has 1080 phase nodes and 2175 variables: its `1e-3` point was
already inside the corridor, so Phase I used no primal-restoration iteration
and one dual-fit factorization, reached dual-fit residual `8.33e-15`, and had a
`5.438 ms` median.

Admission prevents wasted full-model sparse work outside the local basin. The
Hybrid Schur step is audited by its own DC-balance/converter/reference norm but
may not bypass the global admission threshold. At the production default, the
microgrid (`0.144 > 0.1`) and case300 ACDC (`1.129 > 0.1`) therefore retain
their exact cold Phase-II starts after the bounded structural probe. With the
research setting `phase_one_admission_mu_factor = 2`, the microgrid Schur block
is accepted and state/basic Newton reaches `1.689e-5`; the central handoff is
certified. The same bypass was deliberately rejected for case300: it incurred
about 160 ms of full-model QR while only reaching `1.100`, so the global gate
remains authoritative.

DCOPF now has a separate affine structural start for NativeLCQP: each energized
component is power-balanced within generator/load-shed bounds, one angle
reference is selected, and a reduced Laplacian projects angles/flows before the
QP IPM. MIPSolvers `QPModel::x0` audits size/finiteness and reports actual use
and initial primal residual; rejection is exactly the deterministic cold
initialization. In the same two-warmup/five-repeat Release protocol, case57
improved from 29 to 18 iterations and `1.934` to `1.284` ms (`1.506x`), case118
from 43 to 18 iterations and `5.841` to `2.766` ms (`2.112x`), and case300
from 68 to 19 iterations and `33.360` to `10.636` ms (`3.137x`). The
original equality residuals were respectively `6.11e-15`, `1.53e-14`, and
`9.13e-14`; solver initial residual was `0.01` after positive bound-slack
initialization, versus cold `2.60`, `4.03`, and `12.00`. The residual,
iteration, and end-to-end predictions therefore held on all three cases.

Large pure-AC Parity solves can additionally compose those structures as an
opt-in dispatch Phase I: ordinary AC PF is evaluated first, and only a poor
full-model initial dual residual triggers a five-iteration compact NativeLCQP
DCOPF. The DC dispatch and reduced-Laplacian angles are replayed through AC PF;
the candidate is admitted only after evaluating the actual Parity initial
primal/dual metrics, and every failure retains the original formulation and PF
point. This is a dispatch warm start rather than a central-state certificate:
on `case9241pegase`, candidate dual residual improved `838.166 -> 236.753`
while the maximum full-model primal metric increased `1.601e-3 -> 1.506e-1`.
The final Native IPM therefore remains responsible for the unchanged KKT
certificate.

The fixed Release/KLU protocol used one warmup and three timed repetitions on
`case9241pegase`. Ordinary AC-PF start measured `12.657 s` median, 77
iterations, and 76 Phase-II factorizations; the structured dispatch start
measured `11.088 s`, 56 iterations, and 55 factorizations. End-to-end median
fell `12.4%` and Phase-II factorizations fell `27.6%`, exceeding the fixed
10%/15% acceptance thresholds. Objectives were `315911.571480` and
`315911.571406`; final primal/dual residuals were respectively
`6.49e-8/4.66e-7` and `5.78e-8/3.05e-7`. On `case13659pegase`, baseline dual
residual `1.970` passed the prefilter, so no DC work ran; the guard retained
138 iterations, 137 factorizations, objective `386116.837520`, primal
`2.05e-7`, and dual `5.89e-9` in `21.506 s`.

The DC Phase-I default wall budget is `2000 ms` end to end: canonical
projection, compact formulation, connected-component/reduced-Laplacian seed,
symbolic analysis, and numeric iterations are all charged. Sparse
factorizations are cooperative indivisible units, so diagnostics report any
overshoot. On case9241 the DC work used `1638 ms`, did not exhaust the budget,
and performed one NativeLCQP symbolic analysis. Baseline and candidate share
one Parity formulation (`parity_formulation_builds=1`); only mutable authored
operating-point seeds are refreshed before candidate slack construction. On
case13659 the prefilter performed zero DC symbolic analyses.

The 20k-class validation uses the 25,000-bus ACTIVSg25k case (4,834
generators and 32,230 AC branches) with Release/KLU on the same AppleClang 21
arm64 host. The fixed protocol parses once, then runs one warmup and three
timed solves. The ordinary AC-PF start samples were `183.967`, `182.383`, and
`183.808 s` (median `183.808 s`); every solve converged in 66 iterations and
65 Phase-II factorizations to objective `6252070.20694`, primal
`3.239e-7`, and dual `1.004e-7`. The process peak RSS was `5.13 GiB`.

With only the default 2-second DC Phase-I budget changed, samples were
`186.588`, `184.993`, and `187.476 s` (median `186.588 s`, 1.51% slower).
The DC path performed one symbolic analysis and two iterations, exhausted the
budget in `2064 ms` with `64 ms` cooperative overshoot, produced no admissible
iterate, and retained the exact baseline start. Phase II therefore remained
66 iterations/65 factorizations with bit-identical objective and KKT metrics;
`parity_formulation_builds` remained one. Full-solve peak RSS was `4.48 GiB`;
the difference from the baseline peak is allocator/run variability and is not
claimed as a memory reduction.

A predeclared budget sweep explains why increasing the DC-IPM allowance is not
the remedy for this case. A 5-second embedded run completed five DC iterations
but still produced no iterate within the `1e-2` handoff tolerance. DC-only
residuals were `3.639e-2` after
`10.55 s/10` iterations, `1.146e-2` after `20.27 s/17`, and `8.767e-4`
after `31.05 s/25`; peak RSS rose from `2.12` to `3.84` and `5.49 GiB`.
The 30-second embedded run reached that last DC residual but its AC replay
changed the actual Parity primal metric from `3.929e-4` to `9.786e-2` and
improved dual only `468.107 -> 448.664` (4.15%, below the fixed 20% gate).
It was correctly rejected and finished in `215.012 s`, 16.98% slower than the
baseline, with unchanged Phase-II work.

The original `O(n^1.5)` nested-dissection estimate predicted 45--90 seconds
from case9241 and was violated. Re-derivation found a machine/cost-model and
input-assumption error rather than implementation infidelity: bus count alone
does not determine KLU fill, elimination-tree separators, generator/bound-row
density, or memory traffic across PEGASE and ACTIVS families. The benchmark
now reports buses, generators, branches, controllable DC budget/iterations,
all handoff diagnostics, and POSIX peak RSS. The verified conclusion is that
the current solver can complete this 25k ACOPF with a final KKT certificate,
but the current compact DC Phase I does not accelerate ACTIVSg25k. Future 20k
work should target a cheaper dispatch/dual predictor or symbolic reuse across
repeated operating points, not a more accurate auxiliary DC IPM.

The first zero-factorization dual-predictor probe is now implemented as an
experimental, default-off option. It forms one active/reactive balance price
per conductive AC component from the median stationarity of interior
generators. The candidate reuses the Jacobians that Phase II already assembles,
performs no symbolic or numeric factorization, and is accepted only if both the
raw and multiplier-normalized full stationarity residual improve by at least
`phase_one_dispatch_dual_min_improvement` (default 5%). Rejection leaves the
ordinary zero equality-dual start bit-for-bit intact. The JSON and `/api/v1`
diagnostics report attempted/accepted, runtime, both residual pairs, and status.

On ACTIVSg25k, the single Release/KLU research probe cost `1.978 ms` but changed
raw stationarity only `47278.840 -> 47276.850` and normalized stationarity only
`468.10733 -> 468.08762`, a `0.0042%` improvement. The candidate was therefore
rejected; the solve completed in `184.344 s` with the unchanged 66 iterations,
65 Phase-II factorizations, objective, and final KKT residuals. Case30 also
worsened and was rejected; case118 improved about 2% but remained below the
fixed 5% production gate. This falsifies the prediction that an island-common
partial dual would remove one KKT factorization on the 25k case. The dominant
stationarity entries lie outside the free-generator price subspace, especially
in bound/inequality and uncovered variable families.

Current research evidence supports the following priority order:

| Direction | Extra factorization | Memory lifetime | Valid after data/topology change | Measured/expected value |
|---|---:|---|---|---|
| Component dispatch/dual predictor | 0 | `O(nb + ng)` for one initialization | Recompute after either change | Dual-only probe is negligible on ACTIVSg25k; a primal dispatch reshape still needs AC-PF replay and end-to-end validation |
| AC-PF adjoint dual seed | One transpose solve only if the final PF factorization is retained; otherwise one new factorization | PF reduced Jacobian and factors must survive into OPF | Numeric reuse only for unchanged values; symbolic reuse requires unchanged PV/PQ and topology pattern | Deliberately excluded: the reduced PV/PQ PF Jacobian does not directly cover the full OPF equality/bound multiplier layout |
| Repeated-solve continuation plus prepared symbolic cache | 0 extra symbolic analyses after a compatible first solve; numeric KKT factorization remains per IPM iteration | Prepared formulation, maps, sparse pattern/order, and last complete `(x,lambda,z,mu)` state persist across solves | Full continuation requires identical layout; symbolic ordering survives parameter changes with fixed pattern; topology/layout changes invalidate it | Highest priority for time series/contingency batches because HySim already validates and reuses a complete continuation state within one compatible formulation |

That highest-priority path is now implemented as the non-copyable, movable
`opf::PreparedACOPFSession`. Its exact compatibility key covers variable and
constraint blocks, bound-row finiteness/order, component maps, reference rows,
limited-device row membership, energy-router port order, and Ybus/Gdc sparse
patterns. LCC stable identity, AC/DC terminals, resolved commutation bus, and
energy-router port stable identity are included because they select KKT row,
column, or public mapping semantics. Identical inputs reuse the owning
`parity::Problem`; compatible load,
cost, setpoint, and limit changes refresh canonical numeric `SolverData` and
derived coefficients without rebuilding OPF maps. A failed structural audit
rebuilds the formulation, clears the KKT symbolic cache, and prevents opaque
state reuse on that solve. The four continuation blocks are all-or-nothing and
still pass the native positivity, centrality, and residual admission checks.
Each sparse backend additionally keys its symbolic analysis by an FNV-1a scan
of every compressed KKT coordinate, not only dimension and nonzero count; an
equal-size, equal-nnz graph with different coordinates is therefore reanalyzed.

On the fixed case118 Release/KLU protocol, three standalone Phase-I solves
measured `22.199/16.407/14.786 ms` (median `16.407 ms`), 27 iterations, 26
factorizations, and 2 symbolic analyses. One prepared session measured
`18.524/7.163/6.760 ms` (median `7.163 ms`), with the repeated solve at 2
iterations, 1 factorization, and 0 analyses. The prepared
`0,+0.5,-0.5,+2,-2%` load sequence measured
`24.907/8.833/7.859/8.374/8.434 ms` (median `8.434 ms`); the final -2% point
used 7 iterations, 6 factorizations, and 0 analyses. Thus the fixed prediction
of at least 25% Phase-II iteration reduction is met on this oracle. Final
objectives and KKT residuals remain within the existing solver tolerances.
After the exact KKT-coordinate cache audit, a two-warmup/ten-repeat case118
prepared-session check measured `6.121 ms` median, 2 iterations, 1 fresh
numeric factorization, and 0 symbolic analyses. This corrected the initial
three-sample timing concern (`8.501 ms` median): the longer sequence showed no
stable regression relative to the earlier `7.163 ms` result.

The 20k-class acceptance run used ACTIVSg25k (25,000 buses, 4,834 generators,
32,230 branches) with `prepared-power-flow`, no warmup, and two solves in one
Release/KLU session. The first solve took `184645.248 ms`, 66 iterations, and
65 factorizations. The compatible repeated solve took `8666.249 ms`, 4
iterations, 3 fresh numeric factorizations, and zero symbolic analyses: a
`95.31%` wall-clock reduction while improving final primal/dual residuals to
`3.389e-8/2.092e-8`. Its objective `6252069.93711` is within the established
large-case tolerance of the first-solve/reference objective. Peak process RSS
was `4709.6 MiB`; therefore the session closes the repeated-solve time target
at 25k but does not close the memory-efficiency target for larger batches.

Numeric-factor reuse remains deliberately unsupported; ordinary
`factorize()` is not labelled as reuse. The current Eigen SuiteSparse adapters and MIPSolvers
MUMPS wrapper expose persistent symbolic analysis followed by fresh numeric
factorization, but no distinct cross-solve factor object with pre-use matrix
drift and post-use backward-residual/refinement audit. Enabling
`prepared_numeric_refactor` therefore returns an explicit `unsupported` status
and performs ordinary numeric factorization. AC-PF adjoint dual seeding also
remains excluded: its reduced PV/PQ Jacobian does not match the full OPF
equality/bound multiplier layout, and without a retained PF factor it adds a
factorization rather than removing one.

The literature check is consistent with these measurements. Baker,
*Learning Warm-Start Points for AC Optimal Power Flow* (2019,
doi:10.1109/MLSP.2019.8918690), and Cao et al., *Fast and explainable
warm-start point learning for AC Optimal Power Flow using decision tree*
(2023, doi:10.1016/j.ijepes.2023.109369), predict primal operating points but
require offline data and distribution-shift controls. Park et al., *Compact
Optimization Learning for AC Optimal Power Flow* (arXiv:2301.08840), reports
learned warm starts up to 30,000 buses, again with an offline model. More
directly relevant, Taheri and Molzahn, *Not All Warm Starts Help: Benchmarking
Primal-Dual Initializations for ACOPF Algorithms* (arXiv:2606.08984), tests
systems up to 30,000 buses and finds most partial primal-plus-dual restarts
slower or less reliable; complete coverage is the robust case, and DC seeding
loses statistical significance after presolve cost. WARP
(arXiv:2605.05728) likewise identifies the complete `(x,lambda,z,mu)` state as
the useful IPM target. Gondosiswanto and Pulsipher
(arXiv:2606.04725) demonstrate the analogous parametric-NLP benefit of keeping
structure-dependent symbolic work across repeated solves. These results do not
justify a learned model in the production path today; they justify a prepared
OPF session with exact compatibility keys, complete-state audit, and symbolic
cache invalidation before attempting numeric refactor reuse.

The formal runs are reproducible with:

```bash
cmake --build build/macos-release --target opf_numerical_benchmark -j4
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow \
  --warmups 1 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode structured-power-flow \
  --warmups 1 --repeats 3 --max-iterations 300 \
  --dc-max-iterations 5 --dc-time-limit-ms 2000
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow-phase-one \
  --warmups 0 --repeats 1 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-session \
  --warmups 0 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-sweep \
  --warmups 0 --repeats 5 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode prepared-power-flow \
  --warmups 0 --repeats 2 --max-iterations 300
```

The enhanced diagnostic target rebuilt in both `macos-release` and the local
ASan/UBSan configuration. Focused Release and sanitizer runs each passed the
four prepared-session cases with 46 assertions and the eight combined
warm-start/Phase-I cases with 120 assertions. These include Hybrid AC/DC
numeric refresh and LCC numeric-versus-structural mapping coverage. The 25k
timing protocol itself was Release-only and preceded the final KKT-coordinate
fingerprint hardening; it was not rerun after that `O(nnz)` safety scan. No 25k
sanitizer or full CTest claim is made.

Library JSON, the GUI response, and `/api/v1` now preserve the Phase-I
baseline/candidate residuals, budgets, admission status, and warm-start-only
DC contract. The AC request surface accepts bounded central Phase-I and large
pure-AC DC-dispatch controls; callers must still opt into `ac_pf_warm_start`,
and the DC path defaults to pure AC networks with at least 5000 buses.

A barrier sweep put case30 outside admission at `mu0 = 0.03` and `0.01`,
supporting the retained `mu0 = 0.1` default. Current verification passed
MIPSolvers Native IPM 34 cases/246 assertions, HySim
three-phase hybrid OPF 11/145, and ACOPF/DCOPF cross-validation 13/119. The
complete OPF backend binary passed 22/24 cases and 639/644 assertions; both
failed cases are Ipopt or Auto-to-Ipopt iteration-limit stalls that bypass
Native Phase I. The focused `/api/v1` runtime E2E passed. The earlier rebuilt
GUI/API E2E passed 68/69 checks; its sole failure is the same Auto-to-Ipopt
showcase stall. DCOPF remains affine and pays only
the connected-component/reduced-Laplacian projection when NativeLCQP is
selected. Focused ASan/UBSan verification
passed balanced Phase I 3 cases/45 assertions, three-phase Native coverage 5
cases/54 assertions (6 Ipopt-dependent cases skipped by that build), and
ACOPF/DCOPF cross-validation 13 cases/119 assertions. No full CTest, Windows,
or browser-layout E2E claim is made for this increment.

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

The living [module code audit](../testing/module_code_audit.md) records the current
source-backed findings and audit depth. AUD-001 through AUD-011 are closed.
Focused runtime contracts now cover `graph/`, `scenario_generation/`,
`carbon_analysis/`, `integrated_energy/`, and `sppt/`; the canonical links are
in [docs/README.md](README.md).

`scenario_generation` 手册已按《潮流计算》《最优潮流计算》的结构重写为 16 个专章、56 页，
覆盖条件概率、二元 AR(1)、缩减理论，常规/可靠性/弹性三族，台风风雨--易损--故障--修复--
交通链，全部公开选项/结果和 JSON/HTTP/workbook 契约。固定 review case 使用 64 个候选、6 个
代表；4096 步统计实验得到 load lag-1=0.743753（目标 0.75）、全天 load--wind 同期相关
-0.290252（目标 -0.25）。PV 白天子样本 -0.341279 超过原 0.08 门限，未放宽阈值，而改用全天
非零风电验证完整 AR(1) 样本并保留 PV 偏差。128→12 独立聚类复算概率/距离误差为 0；hybrid
全局尾覆盖由 0 提高到 1，但 transport mean 从 1.188380 恶化到 2.148101、regime mass L1
从 0.046875 恶化到 1.640625，文档按真实取舍报告。Holland 风速误差 0、雨量误差
7.82e-14、易损概率误差 6.66e-16、最大积水误差 6.39e-14；证据位于
`external_data/scenario_generation_validation/`。该记录不宣称气象预报、二维水动力、结构可靠度标定或
OpenDSS/GridLAB-D 全模型认证；MIPSolvers 当前 HEAD 与记录 pin 不一致，不能称作 pinned
release baseline。

Current-source Debug verification rebuilt all affected targets. The focused
and contract suites passed: typhoon traffic/catalog 5 cases/35 assertions,
campus IES 6/106, harmonics 52/359, EV Formulation D 15/196,
graph/Kron/round-trip 55/461, scenario generation 12/4214 plus schema 3/9,
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
- At that closure point, the sibling MIPSolvers worktree was clean at
  `5a5fad0`. Earlier local HiGHS experiments are not pending worktree state.

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

An earlier pre-cleanup MIPSolvers Release experiment compared the modified
binary with a clean export of the pinned commit on the same arm64 host using an
A/B/C sandwich: 24 NETLIB cases, Native and HiGHS, 3 repeats, single-threaded
HiGHS, and a 30 second solve limit. Every run was accurate (72/72 for each
solver in each leg). This is retained as historical performance evidence, not
as a statement that the current sibling worktree is dirty.

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

## PV/PQ reactive-limit switching

Balanced Newton PF now performs generator Q-limit conversion only around
fixed-active-set converged points. All violated PV buses enter their Q bounds
in one batch. PQ-to-PV restoration is audited at most once and requires the
configured hold count plus the existing voltage-direction margin. Converged
active-set signatures, outer-budget exhaustion, final Q violation, and an
explicit certificate are returned. The GUI defaults to fast screening with
conversion disabled and labels Q-limit certification separately.

Smooth NCP uses CHKS median smoothing for PV/PQ and priority projections plus
Kanzow smooth-FB for magnitude-priority VSC blocks. Continuation owns a local
`SolverData` copy, initializes `mu` before the first evaluation, reassembles
residual and Jacobian after every reduction, and cannot terminate until
`ncp_mu_min` is reached. VSC continuation can be enabled without the generator
PV/PQ NCP option. Public converter certificates are re-evaluated against the
exact `mu=0` equations.

Release verification rebuilt `test_power_flow_math_audit`,
`matpower_pf_compare`, and `run_gui_server`. The full math audit passed 50 cases
and 305 assertions. The ASan/UBSan build passed its configured 42 cases and 260
assertions; LeakSanitizer is unavailable in the macOS runtime. JavaScript syntax
and `git diff --check` passed.

On `case6515rte`, Q-limit enforcement converged in 9 Newton iterations and 8
factorizations, with 74 PV-to-PQ switches, no restoration, no repeated active
set, and a certified maximum Q violation of `4.22e-11 pu`. The ordinary-layout
five-repeat Release run measured `56.4--61.2 ms` with a `58.7 ms` median; the
fixed-layout comparison is recorded in the flat-start follow-up below.
Fast screening previously used 4 iterations and 3 factorizations with a median
of approximately 21.2 ms.

The original `data/云南案例.json` exposed a distinct input/projection defect,
not active-set oscillation. It contains 587 legacy BPA zero-data L-card ties
encoded as physical `x=1e-4 pu` branches. Conflicting PV setpoints across those
rows produced `O(Delta V/x)` reactive circulation; the old solve reached about
`410 pu` residual and `8736 pu` maximum Q violation without repeating an active
set. BPA import now marks exact-zero L cards as ideal connectivity. JSON round
trip preserves that provenance, and a strict BPA-schema migration recognizes
the legacy encoding while excluding ordinary short physical lines. Canonical
projection performs 585 effective merges, reducing 4512 authored buses to 3927
canonical buses before Ybus and PV/PQ construction.

With the unmodified Yunnan JSON loaded through the rebuilt Release GUI server,
Q-limit enforcement converged and certified in 19 total Newton iterations and
5 outer solves: 218 PV-to-PQ entries, 3 restorations, 215 final limited buses,
no repeated active set or cycle, `7.28e-12` electrical residual, and
`2.27e-13 pu` maximum Q violation. Cold solver/presentation/total-before-
serialization timings were `71.9/224.2/296.8 ms`; five warm compact Q-limit
solves had a `47.3 ms` median. Q-disabled screening measured `18.3 ms`.

Release verification additionally passed `test_io_json` (41 cases, 398
assertions, one data-dependent skip), `test_bpa_io` (25/885),
`test_component_models_math_audit` (15/111), `test_power_flow_math_audit`
(49/304), and `test_advanced_pf` (30/218). The rebuilt `run_gui_server` is the
binary used for the full Yunnan request.

The production-HTTP Yunnan capability audit is reproducible with
`tools/validate_yunnan_capabilities.py`; its merged machine-readable and human
reports are `output/yunnan_capability_audit.json` and `.md`. It covers 46
probes without inventing missing case data: 14 passed, 6 ran with an explicit
input limitation, 20 were correctly classified not applicable, 3 returned a
numerical non-convergence, and 3 exhausted a declared wall budget.

The PF comparison confirms that the fixed active-set Newton path is the right
production default for this case. Screening used `16.0 ms` solver time and 7
iterations. Q-certified Newton used `42.6 ms`, 19 total iterations, 218
PV-to-PQ entries, 3 restorations, and no cycle. Distributed slack (`1081 ms`),
HELM (`443 ms`), and Newton-Krylov (`378 ms`) converged but were materially
slower. FDPF stopped after 1000 iterations without convergence; explicit
homotopy stopped after 110 iterations at residual `1.35` and an uncertified Q
violation. DC and hybrid-linearized PF remain useful screening models, not
substitutes for the nonlinear Q certificate.

Parity OPF with AC-PF warm start converged in 37 iterations and 36
factorizations both with and without central Phase I. Phase I measured
`0.66 ms`, observed initial constraint violation `0.2199 > 0.1`, terminated
`outside-phase-one-admission`, and correctly left the ordinary Phase-II start
unchanged. Core solve time was `676.7 ms` with Phase I and `675.3 ms` without;
the difference is noise. DCOPF completed in `217 ms` core time. Because the
case has no authored generator cost curves, these runs certify feasibility and
solver work, not economic fidelity.

Topology, projection guard, network reduction, harmonics zero-injection,
hosting capacity, carbon-flow plumbing, and scenario generation all returned
normally. The system is one connected AC island with 673 cycles. General graph
reduction eliminated 143 buses/branches in `159 ms`; SPPT admission passed.
Hosting capacity evaluated 2428 BPA transformer branches in `27.7 ms`.
Carbon tracing solved a rank-3927 canonical matrix with zero reported relative
residual in `11.35 s`, but the result uses fallback factors because no authored
carbon factors exist. Scenario generation took `2.36 s` and produced 6344
default reliability contingencies; those defaults are workflow evidence, not
Yunnan stochastic-data evidence.

The Yunnan short-circuit and carbon-flow scale failures are closed in the dirty
worktree. Detailed sequence matrices are sparse, KLU factors are shared across
batch fault locations, selected inverse columns replace full `Zbus` matrices,
and inverse diagonals use bounded RHS blocks with cooperative cancellation.
The GUI all-bus route now uses its documented positive-sequence overview model;
the selected-fault route computes complete fault-point duties and the full
remaining-voltage profile without unused non-fault current diagonals. Release
HTTP wall times were `74.2 ms` for 4512 overview rows and `29.7 ms` for one
detailed fault, versus both former paths exceeding 90 seconds. Carbon tracing
and nodal intensity now remain sparse throughout; the same case fell from
`11.35 s` to `1.469 s`, with rank 3927, relative residual zero, finite condition
estimate `5.173`, and all validity flags true. Focused Release tests passed 523
short-circuit assertions, 8 IEC assertions, and 594 carbon assertions.

Two scale failures remain open. Default counterfactual planning
exceeded 180 seconds and ignored cancellation because it nests baseline, five
measure, and up to ten pair evaluations across economic, carbon, reliability,
and resilience models. It needs a shared prepared assessment session, bounded
dimension selection, and cancellation at every clone/measure/pair boundary.
Finally, the four-step default time-series profile returned voltage arrays but
zero converged steps; all four minima hit `0.05 pu`. The case has no authored
load or generator profiles, so the default `0.4--0.5` load multipliers are not
an admissible Yunnan operating trajectory and must not be presented as a
successful production simulation.

The GUI/API E2E passed all PF Q-certificate and fast-screening assertions, but
the complete run was 70/71 because the separate Hybrid Auto OPF check reached
the Ipopt iteration limit. Browser E2E passed the new default/control assertion
and all large-system PF checks; it remained non-green because two existing
multiscale comprehensive-OPF assertions failed. No all-suite green claim is
made for those OPF paths.

The 82,000-bus `case_SyntheticUSA.m` exposed a separate GUI orchestration
regression. The `ac_newton` route detected three solvable AC islands, ran the
adaptive island solver, and then ran a second full-network Newton solely to
populate display/cache data. The second solve has been removed. Adaptive
results now aggregate per-island Q-limit certificates and profiling and are
used directly for branch-flow reconstruction, compact presentation, and PF
cache state. The exact Release GUI request with PV/PQ conversion enabled
converged in 2.58 seconds with 4,820 PV-to-PQ switches, 76 restorations, no
repeated active set, and maximum Q violation `1.34e-11 pu`. The former double
solve took approximately 5.80 seconds on the same server. Focused Release
adaptive tests passed 30 cases/218 assertions; the ASan/UBSan integration and
adaptive selection passed 4 cases/23 assertions.

## Power-flow flat-start robustness follow-up

The committed PF follow-up closes two correctness defects from the large-case
PF evaluation. Reactive-limit certification now uses
`max(1e-10, PowerFlowOptions::tol)`: the limited-bus Q violation is the
remaining Q-row mismatch and cannot be certified more tightly than the root
that produced it. The focused regression deliberately converges with a
violation above `1e-10` and below `1e-3`, and remains certified. The optional
Levenberg-Marquardt recovery no longer factorizes `J^T J + lambda I` through
the symbolic analysis for `J`; it owns a separate sparse solver, analyzes its
normal-equation pattern once, reuses that pattern across damping attempts, and
reports its analyses, factorizations, solves, and elapsed linear time.

Default line-search Newton now detects only high-residual stagnation: the best
residual must fail to improve by 10% over 15 iterations while remaining above
`1.0`. Near-root damped progress retains the caller's full iteration budget.
For a structurally closed numerical failure, the public default escalation is
now deterministic: direct fixed-layout Newton, semi-smooth NCP, linearized-DC
angle seed plus fixed-layout active set, DC-seeded NCP, then homotopy as the
final fallback. Internal retries own their `SolverData` and `NewtonSolver`, do
not recurse through the facade, preserve the caller's Q-enforcement semantics,
and aggregate their work/provenance in `SolverProfiling`. LCC transformer tap
control retains its dedicated outer loop and is not bypassed by direct retries.

`enable_fixed_pv_pq_layout` is now the C++ default. It embeds every non-slack
AC bus's `Vm/Q` coordinates in one stable canonical bus-position order. PV rows
become `Vm` identity rows while Q-limited rows retain physical Q balance; the
outer active-set timing and certificate semantics are unchanged. Semi-smooth
NCP remains mutually exclusive and wins with an explicit warning when both
options are requested. Row nonzero indices are precomputed once, making
active-row replacement `O(nnz(row))` rather than a full compressed-matrix scan.

On AppleClang 21 arm64 macOS, Release/KLU, working tree `3cc92658373a`,
`case6515rte` reduced Jacobian pattern rebuilds/analyses from `3/3` to `1/1`
with unchanged 9 Newton iterations, 8 factorizations, 74 PV-to-PQ switches,
and Q certificate (`4.22e-11 pu`); five-repeat median solve time was
`58.7 -> 40.9 ms` (`30%`). `case_ACTIVSg10k` reduced `10/10 -> 1/1` with
unchanged 26 iterations, 25 factorizations, 1151 entries, 40 restorations,
and certificate; its median was `261.1 -> 152.0 ms` (`42%`). The default fixed
layout does not claim to reduce the number of outer active-set solves.

The hard flat-start acceptance set is closed without homotopy tuning. Release/
KLU default solves converged and Q-certified as follows: `case1888rte` used the
DC-angle/fixed-active-set stage in 21 iterations, `case3375wp` used semi-smooth
NCP in 9, `case6468rte` used DC-angle/fixed in 21, `case6515rte` used
DC-angle/fixed in 30, and `case_ACTIVSg10k` used DC-angle/fixed in 29. None
reached homotopy. Flat-start `case118` stayed on direct Newton and converged in
18 iterations both with and without the outer ladder; warm `case3375wp` stayed
at 7 iterations and warm `ACTIVSg10k` at 26, with zero escalation attempts.

`PreparedPowerFlowSession` now caches canonical projection, `SolverData`, the
fixed Jacobian pattern, and symbolic analysis across compatible solves. A
same-layout authored load/setpoint change refreshes numeric values without
projection or assembly; topology/network-parameter changes rebuild
conservatively. On `case6515rte`, five Release repeats had medians of
`28.962 ms` through the ordinary facade and `16.463 ms` through one prepared
session (`43.2%` lower); prepared repeats reported zero projection, assembly,
and symbolic-analysis work.

The PF linear path now requests KLU numeric refactorization only after an
existing factorization with an unchanged pattern. Every candidate solve is
guarded by a normwise backward-error audit and falls back to fresh pivoting on
failure or excess error. On warm `case_ACTIVSg10k`, a five-repeat Release
protocol reduced median total time from `154.032` to `104.024 ms` (`32.5%`)
and final-solve linear time from `98.549` to `54.699 ms` (`44.5%`); all 24
refactors were accepted and maximum backward error was `2.29e-17`. A zero
tolerance regression forces every candidate through the full-factorization
fallback and still converges.

Profiling now covers residual evaluation, Jacobian scaling, active-set scans,
projection, assembly, result derivation, solver-core/facade totals, unclassified
remainders, prepared-session reuse, and escalation provenance. The W4 sweep is
also closed: FDPF voltage division uses the configured floor, GFM derivatives
use `min_vm_pu`, the Newton-Krylov Schur preconditioner reuses symbolic analysis,
and the standalone DC solver uses an analytic sparse Jacobian/SparseLU instead
of dense finite differences and `FullPivLU`.

Release/KLU focused verification passed `test_power_flow_math_audit` 50 cases/
305 assertions, `test_advanced_pf` 30/218, `test_homotopy_continuation` 5/36,
`test_newton_krylov` 5/30, `test_hacdcpf` 27/117, and the flat-start/KLU
MATPOWER filter 3/45. ASan/UBSan with SuiteSparse disabled and leak detection
disabled passed 43/261, 30/218, 5/36, 5/30, the prepared-session filter 1/28,
and the MATPOWER filter 3/41. The eight subsequently observed Release failures
are now closed. Hybrid OPF tests 34, 62, 73 and their GUI/RPO consequences
1531, 1538, and 1548 use Ipopt limited-memory Hessian approximation whenever
DC/VSC variables are present; pure-AC problems retain the audited exact
Lagrangian Hessian. This avoids consuming an incomplete hybrid second-order
block without claiming that block is certified. Test 904 now checks its actual
authored-order contract while accounting for the positive loss of the
`r=0.01 pu` branch. Dynamic initialization test 1127 now routes rich systems
through the production PF projection facade instead of interpreting stable bus
IDs as canonical positions. The first complete rerun exposed the related
canonical-input test 1148; an explicit `projection_certificate` split now
prevents re-projecting already-canonical systems, and focused tests 1127/1148
both pass. The final `macos-release` CTest ran all 1609 registered tests in
225.45 s with 0 failures: 1606 passed and 3 condition-dependent tests were
skipped. The skips comprise one unavailable formal SOC JSON input and two
unavailable external GridLAB-D comparisons. The focused NCP/Schur GUI test,
Canvas/WebGL linking, hybrid Auto OPF, and RPO cross-validation all pass. The
main repository is at `1773aa0e75d7`; the clean MIPSolvers dependency is at the
pinned `3bf1e66749e3`.

## Harmonic power-flow closure

The periodic steady-state harmonic scope is now closed across theory, implementation,
failure semantics, and external evidence. The code fixes canonical transformer
double-stamping; derives every linear/Newton `ok` flag from actual order status or
convergence; reuses the pi/tap/phase-shift stamp for from/to terminal current;
separates series current for copper loss; rejects invalid/duplicate orders and
numeric options; and implements Y/YN, Z/ZN, Delta, and authored transformer zero-sequence
impedance in the abc network. The HTTP branch spectrum exposes canonical branch ID,
from current, to current, and series current.

Focused Release verification passes `test_harmonics_power_flow` at 63 cases / 412
assertions. The 14-case AC/three-phase/DC/hybrid matrix passes 14/14 with maxima
`5.349715355e-10 pu` against NumPy, `5.333277457e-10 pu` against OpenDSS, and
`5.349716508e-10 pu` across 24 mandatory GridLAB-D 5.3.0-20236
(`e1841e1e:master:Modified`) frequency slices. IEEE13
passes its pre-fixed `2e-3 pu` gate at `1.652e-3 pu` over 164 bus/phase/order points;
the VSC equivalent-network OpenDSS comparison is `5.075e-10 pu`. GridLAB-D is now a
hard CTest dependency for the harmonic cross-engine test and cannot silently skip.
Machine-readable evidence lives in `external_data/harmonics_validation/` and the
ten-chapter monograph is `docs/modules/harmonics_power_flow/harmonics_power_flow_manual.tex`.
The changed production server target compiles, and `gui_api_e2e` passes against the
rebuilt executable. The XeLaTeX monograph is 17 A4 pages at
`output/pdf/harmonics_power_flow_manual.pdf`; all 17 pages were rendered at 144 dpi
and visually reviewed. The final log has no overfull boxes, unresolved references,
missing glyphs, or PDF-string warnings, and the page images have no clipping,
overlap, broken tables, unintended blanks, or content touching the page boundary.

The local dependency checkout remains at `7721936245d5756193381b415457e6cf2214341e`,
which differs from the recorded MIPSolvers pin `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`;
focused results above therefore record the actual dependency HEAD and do not imply a
pinned clean-clone build.

## Fast orientation

## Southern market large-system extension (2026-09-05)

`southern_market_from_system(system, true)` preserves imported AC topology and augments the
ACTIVSg2000-scale engineering system with 240 wind, 240 solar and 720 hydro
synthetic market units (1200 added, 1744 total), 80 storage units, and 120 compensated
interruptible-load records. The schema accepts `wind`/`solar` kinds, exposes
bus shunt fields, and returns demand-response reductions and binding-count
truncation metadata. GUI result tables use 100-row pagination and price plots
at most 80 traces; all nodes remain accessible in result tables and exports.

The augmentation is explicitly synthetic and is not Southern grid data or a
claim of official rule equivalence. Focused regression remains 11 cases / 323
assertions (`test_southern_market`); a 2000-bus end-to-end MILP benchmark and
AC-security certification are still pending and must report actual runtime,
memory, gap and residual before being described as scalable.

## Southern model internal verification (2026-09-05)

Release CTest `-R 'Southern|southern_market'` passes 14/14, including GUI E2E;
the direct C++ filter passes 13 cases / 335 assertions. Coverage includes
analytic dispatch, compensated interruption, two-generator one-reservoir
conservation, congestion/LMP sensitivity, startup states, reserves, storage,
cascade hydrology, HVDC loss, renewable limits, AC feedback, and priority trades.

The existing ASan/UBSan executable passes its 11-case / 323-assertion Southern
baseline; it predates the two newest hand-oracle cases, so those additions are
not included in that sanitizer result. No external market oracle or user bid
data was used. This establishes internal equation, constraint, conservation,
and synthetic-bid behavior only, not real-market prices or 2000-bus runtime.

### Protection chronology and local measurement closure (2026-08-24)

The follow-up audit closed three previously open correctness gaps: protection
events now require semantic target qualification (preventing same-index PV/VSC/
generator cross-trips), missing topology targets fail explicitly without a false
`applied_event_records` entry, and MassMatrixDae performs bounded same-time
protection closure after topology reinitialization. A separate
`protection_event_cluster_window_s` opens at the earliest physical action,
forces an accepted DAE endpoint at its fixed right boundary, and groups later
actions without rolling the boundary. Direct relays derive frequency from their
local positive-sequence PT angle, apply exact first-order PT/CT/frequency-filter
updates, and block invalid low-voltage frequency measurements until one complete
recovery interval has valid PT voltage at both endpoints; system COI is no
longer a relay input. The focused Release protection subset passes 16 cases /
380 assertions, including boundary-inclusive and three-event
non-rolling-window oracles,
local frequency error below `1e-10 Hz`, and the CT response `1-exp(-1)` within
`1e-10`. EMT measurement remains explicitly unsupported and rejected. Same-time
closure is bounded at 32 iterations and is not a formal Zeno proof. The complete
transient suite passes 111 cases / 116701 assertions, the online-protection
reliability target passes 7 cases / 60 assertions, and `gui_api_e2e` passes in
3.29 s. The XeLaTeX dynamics manual rebuild succeeds at 85 pages; pre-existing
long-identifier box and font-substitution warnings remain.

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
cmake --build --preset full-dev
ctest --preset full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

Use [README.md](../README.md) for the user-facing capability baseline,
[AGENTS.md](../../AGENTS.md) for architecture and invariants, and
[docs/README.md](README.md) for topic documentation.

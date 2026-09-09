# Module Code Audit

Updated: 2026-09-08

This is the living code-audit ledger for repository modules. It records
source-backed defects and audit coverage; it is not a dated snapshot and does
not replace issue tracking. Runtime behavior remains authoritative in
`include/`, `src/`, `tests/run_gui_server.cpp`, `web/`, and registered tests.
Documentation coverage is tracked separately in the
[模块文档地图](../overview/module_documentation_map.md)。

## Audit interpretation

| Depth | Meaning |
|---|---|
| **Deep** | Public headers, core implementation paths, failure/result semantics, and registered tests were inspected. |
| **Sampled** | Shared invariants and high-risk paths were inspected, but the whole module was not reviewed function by function. |
| **Baseline** | Source/header/test ownership was confirmed; existing focused documents and regression evidence remain the main evidence. |

This pass prioritised modules with no focused documentation, followed by
modules whose active documentation is distributed. The bundled
regex-based quality checker was attempted but did not complete in practical
time; the findings below come from manual source review, targeted searches,
and existing executable tests.

## Findings and Closure Evidence

### PF Presentation Cost Breakdown

Follow-up measurement of AUD-097 uses the unchanged production server/source
and a generated instrumented copy from `tools/pf_presentation_profile.py`.
This is profiling, not an additional production optimization. The model is
an additive wall-clock partition: request setup + solver/other analysis +
presentation stages + JSON dump + cleanup + framework/client remainder.
Per-bus linear searches can contribute O(B^2 + B*G); JSON construction/copy/
serialization scales with emitted nodes/bytes, while destruction adds allocator
work. References are `tests/run_gui_server.cpp` (`add_geo_data`,
`rich_attribution_to_component_results`, PF response finalization) and the
[measurement contract](../developer/projection_and_results.md).

Predeclared protocol: four MATPOWER cases, five fresh-process samples in each
baseline/coarse/scan mode, alternating order, `ac_newton`, full responses.
Require exact non-timing response equality, <=5% large-case median wall
overhead, and stage totals within max(1 ms, 1%) of presentation. No speedup
prediction applies because production behavior is not changed. The first
60-sample pass resolved coarse stages and scans; the second added scope
lifetime timers and counts of metrics constructed but unused by the merge.
Both passes are retained, without selecting fast samples.

Final macOS arm64 Release experiment: **60/60 converged responses**, with exact
normalized full-body hashes after excluding `timing`, `profiling` and
`execution_time_sec`. Maximum stage accounting gap is 0.024751 ms. Baseline to
coarse median wall overhead is -0.041% (2000) / +0.548% (9241); scan mode is
+0.352% / +0.514%. All large-case overhead thresholds pass. Small cases are
controls, not evidence of microsecond precision or production regressions.
Fresh-server first requests differ from the preceding same-session protocol;
the absolute 923 ms here must not be compared to 836 ms as a code regression.

| Cost, median ms | ACTIVSg2000 | case9241pegase |
|---|---:|---:|
| Full HTTP baseline | 158.361 | 917.865 |
| Full HTTP coarse instrumented | 158.295 | 922.896 |
| Existing presentation timer | 84.826 | 560.497 |
| Geographic JSON construction | 11.440 | 57.849 |
| Geographic JSON copies into response | 2.223 | 9.970 |
| Component display JSON construction | 20.153 | 122.093 |
| Attribution JSON construction | 28.663 | 158.251 |
| Component JSON merge | 5.701 | 36.811 |
| Diagnostic scan and row construction | 11.177 | 131.434 |
| Diagnostic JSON copy into response | 1.412 | 5.685 |
| Device injection reassembly | 0.177 | 1.383 |
| Generator balance / 2W recovery | 0.339 | 1.837 |
| Terminal-flow recovery | 0.759 | 3.831 |
| Source balance refresh | 0.451 | 17.960 |
| Repeated attribution projection | 0.430 | 2.172 |
| Attribution apply | 0.875 | 5.612 |
| JSON string serialization (`out.dump`) | 25.981 | 137.120 |
| `add_geo_data` local cleanup, outside presentation | 10.856 | 62.800 |
| Handler cleanup after response content is set | 13.302 | 69.536 |
| Outside measured handler lifetime | 2.910 | 12.135 |

Nested rows are not additive with their parents, and medians of individual
stages need not sum to the median total. The raw records preserve every stage
and sample. `geo_cleanup_ms` is the scope-lifetime minus existing presentation
timer: it includes local destruction and negligible timer-boundary work.
The production `solve_ms` currently includes this cleanup (case9241 reports
142.738 ms); subtracting the 62.800 ms still leaves cache publication and other
analysis overhead, so neither number is an isolated numerical-solver timing.
Handler cleanup includes destruction of `out`, the request-local model and
other locals. The remaining 12.135 ms includes HTTP framework, scheduling,
socket and client work; client response-body read alone is 11.431 ms and can
overlap sending. Neither is a pure network-bandwidth measurement.

Optional scan instrumentation isolates three lambda calls (clock overhead
included, counts deterministic across repetitions):

| Lookup | 2000 calls / ms | 9241 calls / ms |
|---|---:|---:|
| `ac_bus_by_id` | 3999 / 1.199 | 18481 / 45.055 |
| `ac_is_slack_bus` | 2000 / 1.288 | 9241 / 46.241 |
| `ac_bus_is_slack_source` | 392 / 0.400 | 1445 / 8.193 |

The first two belong to diagnostic row construction, the third to source
refresh. The separate `attributed_power_balance_diagnostics` helper is not
called by this PF branch; its OPF/TSPF performance remains unmeasured. The
three-winding recovery branch is not exercised by these MATPOWER fixtures.
There is no evidence here that repeated Ybus/SolverData assembly dominates
presentation: the observed repeat is result-side projection and aggregation.

Actual case9241 response size is approximately 49,706,646 bytes (timing text
length can vary). Exact UTF-8 value sizes, excluding surrounding top-level
keys and separators: `component_results` 33,533,469 bytes (67.46% of response),
`geo_ac_branches` 6,757,215 (13.59%), `power_balance_diagnostics` 6,425,883
(12.93%), `geo_buses` 2,047,630 (4.12%). Values are sliced from actual compact
wire JSON, not estimated from Python's serialization. JSON dump costs
137.120 ms; allocation/formatting/destruction are separately material costs.

The generated merge counter observes 35,396 matched attribution rows and
119,161 preconstructed `metrics` items that are not copied into existing
display rows (2000: 5,899 rows / 22,504 items). The merge copies seven metadata/
source/terminal fields but keeps the existing display metrics. This establishes
avoidable construction, not its isolated time: 158.251 ms includes retained
fields and unmatched rows as well. It must not all be claimed as removable.

Evidence-directed next implementation order:

1. Index diagnostic bus/slack membership while preserving all validation,
   domain, stable-ID and source semantics; about 91 ms of measured searches
   is the relevant upper bound before new indexing overhead, not a promise.
2. Construct only required attribution fields for already-present rows, with
   the complete existing path retained for missing rows; measure construction,
   merge and destruction together and compare every field.
3. Remove avoidable ownership copies after proving source lifetimes. Response
   size options/compression require separate API and browser measurements;
   do not drop diagnostics or change the full-response contract silently.
4. Deprioritize the 2.2 ms repeat projection on these AC cases. Profile OPF,
   rich 3W and converter-heavy fixtures before generalizing that conclusion.

Reproduce with retained prior build artifacts:

```sh
python3 tools/pf_presentation_profile.py --build --run --output output/pf-presentation-profile/final
```

`output/pf-presentation-profile/final/{build,results}.json` records compile/link
commands, source/generated/binary/output hashes, all 60 samples, stage/scan
measurements, wire field sizes and acceptance results. First-pass evidence
remains in the parent directory. Python syntax and documentation anchors pass;
unique injection anchors and byte-size extraction were exercised by the actual
run. No production changes this pass, so prior unit/API/sanitizer counts are
not presented as new test runs. No clean build, full CTest, sanitizer or browser
performance campaign was run. Dependency guards and existing preview services
remain untouched; all measurement servers were terminated. Final dependency
inspection reports clean HEAD `5eac6be`, changed externally from the previous
dirty `e6c932e5` record. This pass issued no dependency edits/commits. The
recorded pin is still `a39812aa`; measured binaries retain existing static
archives, so the newly observed dependency HEAD is not a rebuilt baseline.

### Attribution Performance Follow-Up: AUD-097

**P2, closed within the focused scope below.** In
`src/model/result_attribution.cpp`, repeated voltage, result-row, canonical
generator/transformer position and projection-source lookups scale quadratically
on growing systems. The previous lookup terms include O(C*B + M*C + G^2),
for C component rows, B buses, M mappings and G generators. Call-local indexes
now make these terms expected O(B + F*C + G + M + Q), with fixed queried-family
count F and query/emitted-match count Q. Terminal-flow computation, converter
transfer scans, same-bus grid scans and the OPF canonical-system copy remain
outside this bound. No solver equation, tolerance or JSON schema changed.

Before implementation, acceptance was >=70% attribution-time reduction at
10000 nodes, no material small-case regression and exact full-field equality.
The first eager row index added about 10 us on the 14-node fixture: asymptotic
savings did not cover allocator overhead. Re-deriving the small-case cost led
to lazy per-family indexes. Final batched small-case results meet the target;
the initial measurements are retained in `output/attribution-performance/`.
The indexes preserve first-position lookup, AC/DC voltage separation, authored
fallback, OPF canonical position translation and output/source ordering.
Source reports lack domain fields, so their existing all-match provenance
semantics remain; electrical lookup still requires a domain.

macOS arm64, existing Release compile/link configuration, three fresh processes
per fixture/mode. Fixtures with <=118 buses average 100 calls per process;
others time one call. Synthetic fixtures use sparse stable IDs, reversed AC
bus/generator ordering, same-ID DC buses and one transformer per bus pair.
Real projection and converged PF run before timing. All attribution fields,
including identities, values, terminals, sources, recovery, diagnostics and
coverage, serialize to exactly matching baseline/candidate hashes in all runs.

| Fixture | Baseline median s | Candidate median s | Attribution speedup |
|---|---:|---:|---:|
| Synthetic 14 | 0.000014065 | 0.000013748 | 1.02x |
| Synthetic 1000 | 0.004980125 | 0.000680500 | 7.32x |
| Synthetic 2000 | 0.020345958 | 0.001430250 | 14.23x |
| Synthetic 10000 | 0.524050042 | 0.007304500 | 71.74x |
| case118 | 0.000066817 | 0.000054300 | 1.23x |
| case_ACTIVSg2000 | 0.003230667 | 0.000824541 | 3.92x |
| case9241pegase | 0.184946667 | 0.005857250 | 31.58x |

Process peak RSS includes setup/PF: synthetic 10000 is 137.036 -> 136.004 MB;
case9241 is 124.813 -> 127.009 MB. This is not an isolated allocation measure
or a memory-reduction claim. The timings exclude projection, PF and export.

A separate full HTTP comparison uses prior-round/current servers, three
sequential `ac_newton` PF requests per case with `response_detail=full`.
Voltages, angles and complete `component_results` hashes match exactly.
case9241 median request-through-response time is 0.942449 -> 0.836189 s
(11.27% lower); presentation is 621.313 -> 511.089 ms, solve is
126.853 -> 128.122 ms. Its full response is approximately 49.7 MB. ACTIVSg2000
wall time is 0.145961 -> 0.145678 s, with no material end-to-end improvement.
This excludes browser rendering and does not imply a 32x HTTP speedup.
Remaining priorities are profiling presentation diagnostics/repeated assembly
and JSON serialization/transfer. The 511 ms presentation timer covers more
than attribution; source-visible scans are candidates, not measured individual
root causes. Converter-heavy and OPF-copy performance remain unmeasured.

Incremental Release passes **99 cases / 669 assertions** across
`test_result_attribution` (4/96), `test_component_models_math_audit` (25/165),
`test_carbonflow_dynamic_storage` (15/126), `test_sppt_metamorphic` (18/89)
and `test_graph` (37/193). The attribution and test units pass ASan+UBSan 4/96
with `detect_leaks=0`; remaining archives are Release. The relinked current
server passes GUI API **82/82**. New cases in the already registered attribution
target cover sparse/reordered and same-domain/cross-domain IDs, short/null/
changing PF input, OPF position translation, storage preference and source order.
MIPSolvers remains dirty at `e6c932e5`, differing from pin `a39812aa`.
No guard bypass, clean build, full CTest or whole-library sanitizer claim.

Reproduction and local provenance:

```sh
python3 tools/attribution_performance_benchmark.py --mode baseline --output output/attribution-performance/final
python3 tools/attribution_performance_benchmark.py --mode candidate --output output/attribution-performance/final
python3 tools/gui_api_e2e.py --server output/attribution-performance/run_gui_server --data-dir data
```

`output/attribution-performance/final/{baseline,candidate}.json` records source
hashes, compile/link commands, output hashes, timing and RSS. Incremental build
commands/retained archives are in `{tests-build,server-build,sanitize-build}.json`;
test logs and `tests-results.json` are in the same parent directory. The local
`http_compare.py` and `http-results.json` retain the full HTTP protocol, payload
sizes, output/binary hashes and all samples. Temporary test servers were stopped.

### Current Repository Review

Review base: `48e0cf62` (initial main-repository worktree clean). This is a
risk-directed review, not a claim that all 635 C++/header/JS/Python files under
`src/`, `include/hacdcpf/`, `tests/`, and `web/` were read or tested exhaustively.
AUD-089--096 are now **closed within the focused verification scope below**.
The counterexamples and source locations in the finding descriptions refer to
the pre-fix review base, not to the updated implementations. Release-wide
acceptance remains subject to the preserved dependency/build limitations.

#### Implemented Corrections and Validation

- AUD-089: `AtomicFlagLease` owns acquisition/release across all 45 analysis
  handlers; the global exception handler no longer clears `busy`. The two
  market check-then-store sites now acquire via CAS as well.
- AUD-090: all 13 PF cache publication sites pass their captured immutable
  source; publication under the session mutex rejects a replaced source.
  An old response can finish for its original caller without publishing to the
  new model's cache. No JSON schema change.
- AUD-091/094: both GEC APIs reject incomplete/nonfinite/negative hourly
  energy/emissions pairs and unverified supplied step diagnostics; both result
  validators check all numerical summary/hourly metrics for finiteness.
  External complete matrices without step diagnostics remain supported without
  an independent verification claim. Inputs now fail with `invalid_argument`
  rather than manufacturing full coverage from partial evidence.
- AUD-092: generation and participants use in-service AC stable bus IDs, not
  vector positions; capacity/equal/droop and reordered/noncontiguous IDs pass.
- AUD-093: legacy recovery is permitted only when the opposite domain has no
  matching member/super/voltage ID; unknown same-ID AC/DC values stay absent.
- AUD-095/096: hourly tables normalize once after local writes; mismatch
  traverses sparse Ybus with the original polar formula and per-row order.

Incrementally rebuilt Release tests pass **148 cases / 1113 assertions**:
`test_review_regressions` 6/228, `test_advanced_pf` 35/247, `test_graph` 37/193,
`test_carbonflow_dynamic_storage` 15/126, `test_power_flow_math_audit` 51/313,
and `test_thread_pool` 4/6. The three modified numerical translation units and
new regression test also pass ASan+UBSan, 6/228 with `detect_leaks=0` and
container checks enabled; other linked Release archives are not instrumented.
The rebuilt server passes `gui_api_e2e.py` **82/82**, registered
`session_integrity_e2e.py` (ownership/cancellation and stale-publication flows),
and `market_operation_e2e.mjs` (week/month, resume/cancel/stale, desktop/mobile).
Desktop/mobile screenshots were inspected. No full route-by-route or
whole-library sanitizer certification is implied. The server build retains
19 existing compiler warnings outside this fix's scope.

Repeatable commands and provenance:

```sh
python3 tools/review_performance_benchmark.py --baseline 48e0cf62
output/code-optimization/test_review_regressions --reporter compact
ASAN_OPTIONS=detect_leaks=0 output/code-optimization/test_review_regressions-san --reporter compact
python3 tools/session_integrity_e2e.py --server output/code-optimization/run_gui_server
python3 tools/gui_api_e2e.py --server output/code-optimization/run_gui_server --data-dir data
node tests/e2e/market_operation_e2e.mjs --server output/code-optimization/run_gui_server
```

`output/code-optimization/{test-build,server-build,san-build,test-results}.json`
records commands, source hashes and retained archives; logs are in that directory.
The new C++ and session E2E tests are registered in `tests/CMakeLists.txt`;
the current un-reconfigured build was exercised by direct invocation.
The benchmark driver is repository source under `tools/`; generated copies,
executables and measurements remain ignored artifacts under
`output/code-optimization/performance/`.

#### Measured Performance Against the Predeclared Protocol

macOS 26.6.2 arm64, C++20 `-O3 -DNDEBUG`, same existing Release archives;
three sequential fresh-process samples per mode and fixture, 84 total samples
including full-redispatch controls. Build/other test workloads finished before
timing. `performance/results.json` preserves all timings, RSS, commands,
source SHA256 and exact output SHA256 comparisons. RSS is process peak before
export serialization, not an isolated allocator measurement.

| Kernel / fixture | Baseline median seconds | Candidate median seconds | Baseline / candidate peak MB |
|---|---:|---:|---:|
| Frozen carbon B/L=1/128, T=1000 | 0.086408 | 0.005152 | 20.41 / 18.46 |
| Frozen carbon B/L=1/128, T=2000 | 0.313586 | 0.010160 | 26.02 / 22.20 |
| Frozen carbon B/L=1/128, T=4000 | 1.256438 | 0.020222 | 37.50 / 29.64 |
| Frozen carbon B/L=200/200, T=1000 | 0.196518 | 0.009166 | 32.69 / 26.13 |
| Frozen carbon B/L=200/200, T=2000 | 0.723020 | 0.018107 | 51.36 / 37.83 |
| Frozen carbon B/L=200/200, T=4000 | 2.899230 | 0.036692 | 85.93 / 58.85 |
| AC mismatch N=1000 | 0.005474 | 0.0000212 | 29.46 / 13.53 |
| AC mismatch N=2000 | 0.017811 | 0.0000287 | 79.04 / 15.25 |
| AC mismatch N=10000 | 0.525210 | 0.0001293 | 1630.06 / 30.33 |
| Hybrid ZIP mismatch N=1000 | 0.004175 | 0.0000152 | 29.57 / 13.57 |
| Hybrid ZIP mismatch N=2000 | 0.018025 | 0.0000310 | 79.22 / 15.27 |
| Hybrid ZIP mismatch N=10000 | 0.524812 | 0.0001303 | 1630.24 / 30.15 |

AUD-095 predicted >=80% isolated padding reduction at T=4000. Measured
padding is 1.209245 s -> 4.417 us (1/128) and 2.802789 s -> 6.250 us (200/200),
both >99.99% reduction. Aggregation grows approximately 2x when T doubles;
the 200/200 T=4000 aggregation speedup is 79.0x. The timer adds clock reads
per padding call, so these are instrumented materialization measurements.
Each run first computes an actual verified carbon snapshot and then freezes
it for annual materialization; this excludes repeated carbon solves and
does not predict whole-year runtime. Stable IDs, hourly values/NaN masks,
raw hourly energy, summaries, totals and verification counts match exactly;
late columns/failed rows are also covered by the registered regression.

AUD-096 predicted >=90% mismatch time and RSS reduction at N=10000.
Both AC and hybrid ZIP fixtures exceed 99.97% time and 98.1% process peak-RSS
reduction; removing the dense allocation accounts for about 1.60 GB. The
10000-node chain has 29998 Ybus nonzeros and includes taps, phase shifts and
shunts; hybrid fixtures add a PQ VSC and DC voltage. Per-bus calculated
injections and total mismatch match baseline exactly, with independent
`Re(V .* conjugate(YV))` normalized error <=4.10e-15 (threshold 1e-10).
Three-bus full redispatch, with and without a binding participation limit,
matches convergence, iteration count, residual, voltages, allocation and
limit hits exactly. No solver tolerances changed. These are kernel results,
not a 4000x whole-PF speedup promise. Both acceptance targets are met; no
cost-model mismatch requiring re-derivation was observed.

#### Original Counterexamples

| ID | Priority | Finding | Evidence |
|---|---|---|---|
| AUD-089 | P1 | A request that never acquired the session computation flag can clear another request's `busy`. | Reproduced against production HTTP routes; malformed request returns 400, busy changes true to false, a third analysis returns 200 while the first remains running. |
| AUD-090 | P1 | An old PF snapshot is stamped with the current model revision when it finishes. | Reproduced: replace 9241-bus model with case14 during PF; old 9241-voltage result later reports `result_matches_current_system=true`. |
| AUD-091 | P1 | Annual GEC accounting turns missing carbon/energy samples into zero and loses completeness evidence. | Current-source probe: only 1/2 steps verified; failed hour becomes zero energy/emissions, annual coverage is 1.0 and result validator passes. |
| AUD-092 | P2 | Distributed-slack participant selection mixes stable bus IDs and vector positions. | Current-source probe: identical three-bus model reordered from [1,2,3] to [3,2,1] changes participants [1,2] to [2]. |
| AUD-093 | P2 | Domain-qualified switch recovery falls back to an AC-owned legacy voltage for a missing same-ID DC result. | Current-source probe: only AC1=1+0.2j is supplied; DC1 and DC2 are fabricated as 1+0.2j pu. |
| AUD-094 | P2 | Summary-only GEC validators accept NaN numerical fields. | Current-source user-result probe: `energy_mwh=NaN` returns true; node validator has the same comparison/early-return structure. |
| AUD-095 | P2 | Hourly annual-carbon output repeatedly scans all T time rows inside every bus/load iteration. | Source-proven O(T^2(B+L)) metadata scanning; no before/after timing claim. |
| AUD-096 | P2 | Distributed-slack mismatch calculation densifies sparse Ybus and visits all N^2 entries. | Source-proven 16N^2-byte matrix plus N^2 trigonometric evaluations per mismatch; no measured speedup claim. |

#### AUD-089: computation flag ownership

Sources: `tests/run_gui_server.cpp:21203` parses the request before the
`busy.exchange(true)` at line 21241; the exception handlers at lines 21982 and
21986 unconditionally clear the flag. Other legacy analysis handlers use the
same unconditional cleanup pattern, although their pre-acquisition triggers
differ. The production reproduction starts case14 PF-only time series for
1000 periods, waits for busy=true, and submits the body `{` to `run_ts_pf`.
The malformed request clears busy while the first call is still running; a
third `/api/session/pf` succeeds concurrently. The original time series also
completes all 1000 periods. This defeats the guard intended to serialize shared
solver/session resources and can make cancellation target the wrong work.

Required closure: use an ownership-scoped guard or the existing `owns_busy`
pattern consistently; release only a successfully acquired lease. Add a
barrier-controlled overlapping-request regression covering malformed JSON,
pre-acquisition errors, conflict returns, and cancellation. A rejected or invalid
request must leave another task's flag and cancellation state intact.

#### AUD-090: snapshot revision at result publication

Sources: `cache_last_power_flow` in `tests/run_gui_server.cpp:3254` assigns
`last_pf_revision = s.system_revision` at publication, although `/api/session/pf`
captured its system earlier (line 13013). Import/edit routes can replace the
model meanwhile (`load_matpower`, line 11579; `load_json_string`, line 11674).
`result_window` trusts the revision equality at line 17746 to select the current
topology/spatial index. A mutex around publication does not bind the old
snapshot to the new revision.

Production reproduction: launch case9241pegase PF, replace the session with
case14 while that PF is still pending, await convergence (9241 voltages, residual
1.0043876841336896e-10), then request `result_window`. It returns 200 with
`result_matches_current_system=true` and no stale-model limitation. Consequently
the cached result's index can describe a different model from its values.

Required closure: capture system handle and revision together under the lock;
carry that revision through every PF/OPF publication. Either decline to publish
over a newer revision or keep the original revision and use only the solved
snapshot's indices. Test replacement *during* a solve as well as after it.

#### AUD-091: incomplete annual GEC inputs

Sources: `src/carbon_analysis/annual_carbon_analysis.cpp:1154` ignores non-finite
energy/emissions and explicitly substitutes zero when no finite energy exists.
The node path's `aggregate_load_by_bus` (line 1723) likewise skips unknown data.
Upstream annual-carbon aggregation correctly leaves failed samples as NaN and
records verification counts, but the GEC result types carry no completeness
field. A user cannot distinguish an unknown hour from a verified zero-load hour.

The probe computes annual carbon from a one-bus 10 MW, 0.8 tCO2/MWh source/load
case with one converged PF and one failed PF. `num_carbon_verified=1`,
`num_steps=2`; the second input energy is NaN. GEC accounting with 10 MWh of
certificates exports zero for that hour, a 100% coverage ratio, and passes
`validate_annual_user_gec_result`. These are only known-subset figures, not an
annual completeness certificate. This also permits a partially missing user's
load set to be summed without a per-user warning.

Required closure: reject incomplete accounting inputs, or carry explicit
per-hour/per-user/per-node completeness and distinguish known-subset totals from
annual totals. Preserve missing energy/emissions pairs through JSON/CSV export;
do not issue complete annual coverage/net-emission claims from the subset.
Test failed PF, failed carbon verification, one missing load among several,
and entirely unknown horizons for both user and node APIs.

#### AUD-092: stable IDs in participation factors

Sources: `src/power_flow/distributed_slack_solver.cpp:33` creates an N+1 array
and discards generator buses outside [1,N]; lines 358-362 use vector position
to inspect bus type and then emit that position as a stable ID. Explicit
participants are also filtered by `bus <= nac` (line 369).

The reordered three-bus probe preserves every component ID, generator terminal,
and branch terminal, yet drops the slack generator at bus 1 from the automatic
participant set. Sparse IDs such as 10,20,30 are also rejected by source logic.
Required closure: key aggregation and existence checks by authored AC bus ID,
filter in-service buses, and return stable IDs. Regress both permutation and
non-contiguous numbering for automatic/explicit/equal/capacity/droop paths.

#### AUD-093: recovery must not invent a missing domain

Source: `src/graph/result_recovery.cpp:123` falls back to `bus_voltage` when
`dc_bus_voltage` lacks the supernode. The preceding AC recovery loop populates
that same legacy map, so even callers using only the recommended qualified
input API can receive an AC voltage as a DC result. The second DC recovery loop
contains the same fallback.

Required closure: admit legacy fallback only when its domain is unambiguous;
otherwise retain an unavailable value or report an error. Add AC-only-known and
DC-only-known partial-result cases with colliding supernode/member IDs, in
addition to the existing tests where both domains are fully supplied.

#### AUD-094: NaN bypasses summary validation

Sources: `src/carbon_analysis/annual_carbon_analysis.cpp:1364` and line 1961
only use ordered comparisons on summary values, then return true when hourly
rows are absent. IEEE 754 comparisons with NaN are false, so all those rejection
conditions can be bypassed. Some derived summary fields are not checked at all.
This is distinct from AUD-091: a caller-supplied malformed result is certified
even without any missing annual input data.

Required closure: validate finiteness of all required numerical fields before
range/identity checks; apply the same rules with and without hourly output.
Test NaN and positive/negative infinity in primary and derived metrics for both
user and node result validators.

#### Performance Rationale and Acceptance

The following rationale and thresholds were recorded before implementation;
the completed experiment and measured acceptance appear above.

- **AUD-095 model/claim:** dense hourly tables are a materialization of indexed
  per-step rows; padding a row once after the final column registry is known
  preserves row/column identity, values and the NaN mask. The existing
  `add_bus_result`/`add_load_result` already resize the current row locally.
  `extend_hourly_width` (line 423) scans every time row, even when no row needs
  growth; it is called inside each bus/load loop at lines 923/952/955. With all
  three tables enabled, the leading metadata visits are T^2(B+2L), versus
  O(T(B+L)) output work for local growth plus one final normalization. At
  T=8760, B=L=2000, this is about 4.60e11 redundant row-size visits. Prediction:
  remove the T multiplier from metadata scanning; this is **not** a wall-time
  speedup estimate. Reference: current helper/write/final-padding paths in
  `annual_carbon_analysis.cpp`. Benchmark fixed verified inputs at
  T=1000/2000/4000, B/L=1/128 and 200/200, three sequential runs per mode;
  report aggregation wall and peak RSS. Require exact IDs, NaN masks, hourly
  finite values, verification counts and totals; measure near-linear output
  growth separately from per-step carbon solves. Target at least 80% reduction
  in isolated padding time at T=4000; if it misses, inspect allocations and
  the actually measured fraction before revising the prediction.
- **AUD-096 model/claim:** the same nodal active injection is
  P=Re(V .* conjugate(YV)), or the existing nonzero AC kernel's polar sum.
  `distributed_slack_solver.cpp:193` instead allocates complex-double N-by-N
  storage and evaluates sin/cos even for structural zeros. Memory is 64 MB at
  N=2000 and 1.6 GB at N=10000, excluding the original matrix and temporaries;
  repeated full redispatch can invoke it up to 20 times. Prediction: extra
  storage drops to O(N+nnz(Y)) and arithmetic visits from N^2 to nnz(Y);
  on a 10000-node chain the visit ratio is about 3333, not a promised time ratio.
  Reuse `src/power_flow/ac_kernel.cpp:34` or sparse complex multiplication;
  keep ZIP/converter injection assembly unchanged. Benchmark frozen states at
  N=1000/2000/10000, with taps/shunts and hybrid ZIP cases. Require per-bus
  normalized difference <=1e-10 and unchanged convergence/limit outcomes;
  compare end-to-end redispatch within its existing tolerance. Target >=90%
  reduction in isolated mismatch time/RSS at N=10000; do not loosen solver
  tolerances or claim whole-PF improvement from the kernel alone.

The Southern market already has matrix-reuse guards and a separate detailed
[performance ledger](../modules/market/performance.md). Its historical Gurobi
factor-fill/repair-iteration evidence supports investigating ordering and
intertemporal structure next; this review did not rerun those timings and does
not identify another proven market bottleneck. Likewise, linear `find_row`
lookups in rich-result attribution are profiling candidates, not measured
top-priority bottlenecks here.

#### Initial Review Coverage and Verification

| Area | This review's scope | Limit |
|---|---|---|
| PF, graph recovery, annual carbon/GEC | Read implicated implementations, public result contracts and adjacent tests; rebuild three production translation units for focused probes. | Other linked objects remain prebuilt; no whole-library sanitizer claim. |
| HTTP/session and frontend task flow | Inspect snapshot/busy/cache ownership, import routes, API client/task manager; exercise two production concurrency scenarios. | No exhaustive route audit or visual browser E2E in this pass. |
| Annual/lifecycle and generic/Southern market | Sample daily admission, solver concurrency, state carry, failure paths, scope/derivation guards and prior numerical evidence. | No new full-year/week or cross-solver campaign. |
| Reliability/resilience, projection/assembly and I/O | Targeted map/cache/ownership searches and selected implementation/contract reads. | Not a new deep audit of all numerical branches. |
| Remaining numerical modules | Repository/test/document inventory and prior ledger only. | Prior deep-audit labels below must not be read as new verification. |

Executed: 63 selected existing Release CTest cases passed, zero failures/skips,
1.91 s. All 60 JS files under `web/` plus browser E2E scripts pass `node --check`.
The documentation anchor check passes (1846 file/symbol anchors, 711 paths,
zero failures before this ledger update); `git diff --check` passes.
Exact focused-test invocation:

```sh
ctest --test-dir build/macos-release --output-on-failure --timeout 60 -j 2 \
  -R '^(Graph|Round-trip|Distributed slack|Audit B17|Carbon|carbon|Matrix carbon|Annual replay|Lifecycle storage|Parallel annual)' \
  --output-log output/code-review/ctest.log
python3 output/code-review/run_probes.py
python3 output/code-review/http_probe.py
```

Probe artifacts: `output/code-review/probe-results.json`, `probe-build.json`,
`http-results.json`, scripts and server log. These are local diagnostic artifacts
under ignored `output/`, not registered CI regressions. The HTTP executable is
`output/market-performance/lmp-reuse-dev/run_gui_server`, SHA256
`948626bcdc7602d27352f3baf333d4a6599a23bcc81e6d2707f18f462c20317a`;
its recorded server-source SHA256 matches the pre-fix `tests/run_gui_server.cpp`
(`114384b6745f45ab8516bd503e424c64847622b6f9fbb45a687ee86eaeb48d5b`).
Both tests used a fresh private server, which was stopped afterwards; existing
user services were not changed.

Build boundary: MIPSolvers HEAD `e6c932e5f8a409cc87bf2668b86eb895f3eccea5`
differs from CMake's recorded `a39812aa5941691b44e8379a8e0b7d42ccdde955`
and has five modified files. The Release dirty-dependency guard was not bypassed;
no clean configure/rebuild or full CTest was performed. Existing regression
passes cannot certify all current source. Initial standalone-probe attempts
needed `DYLD_LIBRARY_PATH` for OpenDSS; the optional sparse-ID exception probe
also hit an unresolved exception-unwinding abort (LLDB reached the expected
`create_participation_factors` throw). The successful identity evidence above
uses the non-throwing reordered-ID counterexample; this review does not attribute
that diagnostic executable's unwind failure to a new production defect.

The eight findings above have completed their focused closure. Establish a
reproducible dependency/build baseline before a
release-wide regression or performance acceptance; the default CI still gates
C++ execution behind `ENABLE_FULL_CI` and lacks a concrete dependency checkout.

### Prior Audit Closures

All data-structure review findings R-01–R-08
（[数据结构设计评审](../developer/data_structure_design_review.md) §2）已关闭
or reclassified this iteration; see the table below. AUD-001 through AUD-011
remain closed.

Addressed this iteration:

| ID | Finding | Closure | Regression |
|---|---|---|---|
| AUD-088 | `analysis` (hosting capacity) was internal-regression-only: `test_hosting_capacity` checks the capacity formula, grade boundaries and JSON against the same implementation, and the module chapter stated there was "no independent DL/T 2041 calculator, commercial planning software, or 8760-hour external planning oracle." Nothing independently re-derived the equipment-level hosting-capacity closed form. | Added the independent equation-oracle pair used by the other quantitative modules: a C++ emitter (`validate_hosting_xref.cpp`) runs the production `assess_hosting_capacity` on four deterministic single-transformer supply areas and dumps the supply-area aggregates, transformer parameters and hosting/accessible figures; a numpy-free Python oracle (`run_cross_validation.py`, does not link hacdcpf) re-derives the DL/T 2041-2025 closed form `S_d = max(0, (P - P_G + beta*n*S*cos + P_ESS + dP_ESS)/tau)` and the accessible-capacity subtractions and checks them, with case 1 the hand-verified analytic anchor from `test_hosting_capacity`. Registered as the CTest test `hosting_capacity_cross_validation`; the analysis cross-validation chapter's boundary statement was upgraded to cite it. | `hosting_capacity_cross_validation` passes with worst error `0` across all figures on four cases (anchor S_d in [10.5, 12.5], accessible 9.5/7.5; auto-beta single 18.2; parallel auto-beta 10.4; storage interval clamped to [0, 3]) at a `1e-9` gate; a negative control (rewriting one S_d,max) fails as expected; the analysis manual recompiles with XeLaTeX |
| AUD-087 | `market` was internal-regression-only: `test_market_simulation` checks vector lengths, reserve balance, LMP finiteness and settlement records against the same implementation, and the module's own cross-validation chapter stated there was "no independent LP/MILP reconstruction, MATPOWER market oracle, or cross-engine LMP comparison." Nothing independently re-derived the SCED locational marginal price or the settlement identities. | Added the independent equation-oracle pair used by the other quantitative modules: a C++ emitter (`validate_market_xref.cpp`) runs the production `run_day_ahead_market` on two deterministic single-bus copper-plate cases with linear generator costs and dumps the offers, demand, LMP, dispatch and settlement ledger; a numpy-free Python oracle (`run_cross_validation.py`, does not link hacdcpf) re-derives the merit-order dispatch stack and marginal (price-setting) offer — the uniform LMP on a lossless bus — and independently checks the revenue-adequacy identities (resource energy revenue = sum_g LMP_bus(g) * dispatch_g, customer energy payment = LMP * demand, zero single-bus congestion rent, zero cash-flow residual). Registered as the CTest test `market_sced_cross_validation`; the market cross-validation chapter's honesty statement was upgraded to cite it. | `market_sced_cross_validation` passes with worst error `0` across nine checks on the two cases (demand 60 -> LMP 20, dispatch (60,0), payment 1200; demand 140 -> LMP 40, dispatch (100,40), payment 5600; both revenue adequate) at a `1e-6` gate; a negative control (rewriting one LMP) fails as expected; the market manual recompiles with XeLaTeX |
| AUD-086 | `reliability` had no independent-implementation oracle for its deterministic core. Every method (NSQ/SEQ Monte Carlo, FMEA, three-stage) consumes `resolve_reliability_params`, which converts heterogeneous failure-mode fields into canonical `lambda`/repair/unavailability via documented Billinton & Allan closed forms, but that conversion was only checked by same-repo Catch2 assertions; the system-level indices are Monte-Carlo (sampling noise, not machine-precision comparable). | Added the independent equation-oracle pair used by the other quantitative modules: a C++ emitter (`validate_reliability_xref.cpp`) runs the production `resolve_reliability_params` on eight deterministic inputs covering every conversion branch (lambda+MTTR operating/calendar basis, legacy MTBF with either convention, explicit MTTF, forced-outage-rate with and without repair time, and active-on-demand) and dumps the raw inputs, policy and resolved parameters; a numpy-free Python oracle (`run_cross_validation.py`, does not link hacdcpf) re-derives the alternating-renewal closed forms — `U = lambda/(lambda+mu)` with `mu = H/r`, `lambda = f/((1-f) r) H`, the calendar-basis correction `lambda = f_cal/(1 - f_cal r/H)`, and `lambda_active = nu p_d` — and checks lambda, repair time, unavailability, MTTF, and the calendar/active-equivalent frequencies. Registered as the CTest test `reliability_resolver_cross_validation`; a new subsection in the reliability verification chapter documents it. | `reliability_resolver_cross_validation` passes with worst error `0` across all six parameters on the eight cases (the independent re-derivation is bit-identical to the production resolver in IEEE 754 double) at a `1e-9` gate; a negative control (perturbing the FOR-branch lambda by 1%) fails as expected; the reliability manual recompiles with XeLaTeX |
| AUD-085 | `carbon_analysis` was internal-regression-only: its `test_carbonflow_*` Catch2 suites are closed-form/constructed-network checks, and the module's own cross-validation chapter stated there was "no independent linear-algebra oracle … only implementation-equation consistency, not external accuracy." No independent re-derivation of the Kang carbon-emission-flow nodal-intensity system existed, unlike the `integrated_energy`/`model`/`power_models` modules. | Added the independent equation-oracle pair used by the other quantitative modules: a C++ evidence emitter (`validate_carbon_xref.cpp`) runs the production `compute_carbon_analysis` on three deterministic AC cases with analytically known nodal intensities and dumps the inputs plus the solved intensity vector; a numpy-free Python oracle (`run_cross_validation.py`, does not link hacdcpf) re-derives and re-solves the same `A w = b` system (mirroring `solve_carbon_matrix`) by independent Gaussian elimination and checks analytic closed forms, the independent re-solve, nodal conservation, the branch loss-allocation rule, and the system emission balance. Registered as the CTest test `carbon_analysis_cross_validation` (`carbon_analysis;cross_validation;python_oracle`); the chapter's honesty statement was upgraded to cite it. | `carbon_analysis_cross_validation` passes with worst error `3.553e-15` across all seven checks on the three cases (single-source propagation `w=0.5`; lossless mixing `w=0.3`; lossy mixing `w=57.5/95` with the balance closing to 60 tCO2) at a `1e-7` gate; a negative control (perturbing one emitted intensity) fails as expected; the carbon manual recompiles with XeLaTeX |
| AUD-084 | The LaTeX manuals and Markdown design docs carry ~1800 `file:symbol` source anchors (inside `\srcpath{}`/`\implfull{}` and the second argument of `\compmeta{}`) plus ~700 structured path anchors, but nothing verified they still resolved. Eight had drifted undetected (written here as `symbol` in `file` form, not the live anchor syntax, so the checker does not re-flag this ledger): `ResultAttributionLayer::apply` (×2) cited against `result_attribution.cpp` named a class that exists in no source file (the real entry is `CanonicalToRichOperator::apply`); `stage_topology` cited against `three_stage_reliability.cpp` named a non-existent symbol (the F7 "faulted branch is out in every stage" logic lives in `solve_stage_milp`); `CyberPhysicalFMEAOptions` (×2) cited against `failure_mode.hpp` named the wrong header (the struct is in `reliability_assessment.hpp`); `apply_typhoon_impact` cited against `scenario_generation.cpp` and `traffic_node_locations` cited against `typhoon_traffic_impact.cpp` named phantom functions (the real ones are `wind_generation_from_track` and `georeference_nodes`); and the component-math-audit test path used a non-existent `model/` subdirectory. | Added [doc_anchor_check.py](../../tools/doc_anchor_check.py): it resolves every `file:symbol` anchor (structured macros + Markdown prose) against the `src/include/tests/tools` tree by whole-word symbol presence, validates structured path anchors rooted at tracked source trees, supports the `symbol_*` wildcard-family and `foo.cpp/.hpp` dual-extension conventions, and skips the external `../MIPSolvers` sibling. The six drifted anchors were corrected against the verified symbols. The check is registered as the tracked CTest test `doc_anchor_check` (runs on every `ctest` sweep and in the build-test CI job); the same command is also added as a step to the always-on `lint` job in `.github/workflows/ci.yml`, which is git-ignored in this checkout (`.gitignore` ignores `.github/*` except two skills), so the canonical CI must mirror that one-line step. | `python3 tools/doc_anchor_check.py` reports `file:symbol ok=1828 path ok=699 bare=1481 failures=0` (exit 0) and the `doc_anchor_check` CTest test passes; an injected fake anchor is detected (`failures=1`, exit 1); model/reliability/scenario_generation manuals recompile with XeLaTeX |
| AUD-083 | Transient DC-domain disturbance events could not resolve their targets after canonical projection: `canonicalize_dc_bus_indices` renumbers non-contiguous DC bus ids ({10,11,12}→{1,2,3}) but events authored in the caller's bus-id space kept authored ids and never set `canonical_bus`. AC was unaffected only because its ids were already contiguous. The strict target validation added in `99dfae18` turned this from a silent no-op into a hard `Dynamic event target not found`, and the production `/api/session/run_transient` route validated DC load events by `component_index` while the solver applied them by bus — an inconsistency a real user with non-contiguous DC buses would hit. | `DynamicModelBuilder::build` now records authored→canonical AC/DC bus maps on the `DynamicNetwork` (mirroring resilience `canonical_bus_ids`: AC via `bus_merge_map`, both domains positional fallback). `apply_events` auto-populates each author-space event's `canonical_bus` (an explicit value, e.g. from the resilience DAE path, still wins), and `FaultShunt`/`ClearFault` plus `load_event_has_dynamic_device_target`/`device_event_has_target` resolve through it. Devices already consumed `canonical_bus`. | `transient_native_disturbance_matrix` (#1658) 18/18; `test_transient_dynamics` 115/116774, `test_dynamic_model_catalog` 8/4034, `test_intelligent_cyber_physical_reliability` 7/60, `test_resilience_assessment` 39/364 unchanged; full registered suite 1686/1686 |
| AUD-082 | Six repository-baseline test failures (previously tracked only in `development_status.md`, and inconsistently reported as both six-failing and zero-failing) were confirmed reproducible at HEAD on freshly rebuilt binaries and closed. Root causes spanned graph island classification, Newton exception typing, three-stage option validation, and OpenDSS test gating. | (1) `analyze_topology` marked any load-bearing singleton with no *in-service* neighbor as `IsolatedLoad`, so a single load+slack bus and load buses islanded only by an out-of-service branch were misclassified; `IsolatedLoad` now requires a topological orphan (`has_load && !has_generator && adj.empty()`), and a source-hosting or out-of-service-branch-islanded load falls through to `NoSlack`/`Valid` ([topology_analysis.cpp](../src/graph/topology_analysis.cpp)). (2) `NewtonSolver::solve` flattened every exception to `std::runtime_error`; input-contract `std::invalid_argument` is now re-thrown with its type preserved ([newton_solver.cpp](../src/power_flow/newton_solver.cpp)). (3) `run_three_stage_reliability[_from_string]` swallowed the polygon-sides `std::invalid_argument` into a soft `result.error`; option validation now runs before the soft catch ([three_stage_reliability.cpp](../src/reliability/three_stage_reliability.cpp)). (4) `harmonics_ieee13_opendss` was registered without the `HACDCPF_HAVE_OPENDSS` gate, so with OpenDSS off it failed (importer returns 0 buses) instead of skipping. | `test_vsc_limit_ncp` #812, `test_nighttime_opf` #925, `test_three_stage_reliability` #1525, `test_reliability_resolver` #1528/#1531 pass; `harmonics_ieee13_opendss` skips when OpenDSS is off; focused regression green across `test_graph`, `test_graph_kron`, `test_validation`, `test_reliability_resolver`, `test_three_stage_reliability`, `test_vsc_limit_ncp`, `test_nighttime_opf`, `test_advanced_pf`, `test_opf_solver_backends`, `test_distribution_pipeline`, `test_component_models_math_audit`, `test_topology_crossval` |
| AUD-012 | DC-side dead islands were never stripped (review R-01/R-02): AC-only strip could leave an unsourced or orphaned DC bus, risking a singular `Gdc`. | `strip_dead_dc_islands()` removes unsourced DC islands and records `dc_prestrip_to_survivor` on the certificate; `unproject_dc_bus_vector()` recovers DC voltages to authored space (0 pu at a stripped bus). Recovery is wired into the PF facade, `solve_handle`, `solve_dc_power_flow`, and AC OPF. The strip criterion keeps DC buses reached by an out-of-service converter or a closed DC breaker. | `test_component_models_math_audit` "R-01/R-02: DC dead islands are stripped..." and "...unproject_dc_bus_vector recovers..."; cross-suite guards `test_opf_solver_backends`, `test_distribution_pipeline` |
| AUD-013 | Standalone `merge_zero_impedance_buses` ignored `ACBranch::ideal_connectivity` (review R-04), diverging from the projection whitelist path. | The nullptr path now contracts `ideal_connectivity` branches regardless of magnitude, additively, without changing its tested numeric-threshold behavior. | `test_component_models_math_audit` "R-04: standalone merge honors ideal_connectivity..." |
| AUD-014 | Merge dropped `ACBus::importance` (R-06); `unproject_bus_vector` silently returned empty for an unbuilt map (R-07); merge participation basis ignored detailed `Charger` rows (R-08). | Merge aggregates `max(importance)`; `unproject_bus_vector` throws on a non-empty input against an `n_merged==0` map; `charging_station_effective_kw` prefers detailed charger demand. R-03 (load-basis participation) and R-05 (`MobileStorage` is AC-domain) are confirmed by-design with locking regressions. | `test_component_models_math_audit` R-03/R-05/R-06/R-07/R-08 cases |
| AUD-015 | Reliability review (RL-01/RL-02): `compute_tail_risk` could index past the LOLE sample vector when the LOLE series is shorter than the EENS series; `compute_distribution_indices` could emit `asai` outside `[0,1]` when `saidi > hours_per_year` (e.g. multi-interruption microgrids). | `compute_tail_risk` resizes `sorted_lole` to the EENS length (0-fill) before percentile indexing ([reliability_assessment.cpp#L2041](../src/reliability/reliability_assessment.cpp#L2041)); `asai = std::clamp(1 - saidi/H, 0, 1)` ([#L2351](../src/reliability/reliability_assessment.cpp#L2351)). | `test_reliability_resolver` RL-01 (shorter-LOLE tail risk) and RL-02 (ASAI clamp) cases |
| AUD-016 | Dynamics review (DY-01): `small_signal_analysis` computed participation factors from `R.inverse()` without guarding a defective/near-singular reduced Jacobian, so a non-finite left-eigenvector inverse could poison every participation factor. | When `Linv = R.inverse()` is not all-finite the participation loop falls back to right-eigenvector magnitude (`rk*rk`) and `result.message` declares the defective-Jacobian fallback ([SmallSignal.cpp#L423](../src/dynamics/SmallSignal.cpp#L423)). | `test_transient_dynamics` small-signal suite (the defective-matrix branch is documented as impractical to force through the public API) |
| AUD-017 | Market pricing index-space deep review (position/index dual-key, LMP dual extraction, N-1 cut loop). | **No defect.** LMP/shed/curtail use a direct authored-bus index that is correct only because AC/DC buses are never filtered (`build.B == buses.size()`, `b ≡ authored position` — [market_simulation.cpp#L958](../src/market/market_simulation.cpp#L958)); balance/reserve/dc_balance duals all live in the equality block and are read as `constraint_duals[inequality_rows + row]` ([#L2758](../src/market/market_simulation.cpp#L2758)); settlement keeps both `generator_position` (authored) and `generator_index` (stable); the N-1 cut loop dedups via `cut_keys`, breaks on `added==0`, and is bounded by `max_iterations`. The **buses-never-filtered ⇒ LMP-authored-position** invariant is recorded as a design guarantee to preserve. | Existing `test_market_simulation` settlement (per-authored-bus `lmp_per_mwh[b]`) and hybrid-DC (`dc_*[0]`/`[1]` at authored positions) cases pin the direct-index attribution |
| AUD-079 | Dynamics protection chronology review: IEEE 1547 timers and filters advanced only at accepted-step endpoints; MassMatrixDae did not locate endogenous actions, cluster simultaneous actions, or audit algebraic consistency after the topology reset. | IEEE 1547 interval advancement now integrates first-order measurement filters analytically and returns sub-step action time. `MassMatrixDae` previews protection without mutation, rolls back and bisects the first event, commits same-time actions as one cluster, then solves and audits the post-reset algebraic constraint with frozen post-reset differential state. Direct `DynamicSystem` API coverage now also includes definite-time voltage and COSMIC-type balanced positive-sequence Zone-1 apparent-admittance relays emitting `ACLoadScale`/`ACBranchTrip`. Rich-model/HTTP automatic relay assembly, inverse-time, multi-zone, within-step pulses, grazing/Zeno and EMT remain explicit limitations. | `test_transient_dynamics` relay subset 2 cases/85 assertions (including linear threshold-exit/equal-rate timer recovery) and full target 106 cases/116637 assertions; `test_intelligent_cyber_physical_reliability` 7/60; `gui_api_e2e` production-route contract; COSMIC clean commit `6acc77e...` public example matches, paper Fig. 2 reconstruction does not match (`V5_min=0.896065 pu`, branch-6 pickup ratio `0.3098124214893234`) |
| AUD-080 | DER_A `Freq_Flag=1` used a simplified PI/ramp chain that was not equivalent to the PSD/WECC state equations, and its frequency filter always targeted nominal frequency. | Added the missing droop/deadband/error/power/ramp parameters, P/Q current-priority and generator/storage flags; initialization and both frequency branches now follow the PSD state order and non-windup equations. `NetworkState::system_frequency_pu` is derived from the trial state's system COI frequency for every reduced/DAE residual evaluation. PSD Test 42 trace parity remains unclaimed because the local external test environment fails before execution with a SciML precompile conflict. | DER_A equation oracle: 1 case/27 assertions at `1e-12`; PSD standalone IBR path 1/22; full dynamics 115/116774 |
| AUD-081 | Mixed AC/DC dynamic initialization did not honor Voltage/Droop DC/DC control, so a valid PF seed could become an unsourced DC island; consistent initialization also evaluated a `1e-7` dynamic gate through a looser `1e-6` algebraic solve. | Dynamic DC/DC now reuses `dcdc_power_transfer` for all modes; Voltage/Droop are explicit fast-inner-loop algebraic reductions and Power retains one actuator state. Consistent initialization temporarily tightens the network tolerance below the dynamic gate, and Picard/Newton fallback restores the pre-Picard seed. | Shared-equation DC/DC oracle 3 modes/15 assertions; classical GUI mixed case 1/6; model catalog 8/4034; GUI API E2E passes |
| AUD-041 | The English and Chinese rich AC/DC short-circuit derivations stated that DCCB state/resistance was unused and later retained a stale full-bus-current duty approximation. | Both living theory notes now describe ideal-edge contraction, unambiguous matching, open/closed topology, actual resistive-edge current recovery, series chains, parallel division, and the remaining EMT boundary. | Documentation drift scan plus DCCB analytic regressions |

Findings closed by the deep `network_reconfiguration/` implementation pass:

| ID | Closure | Focused evidence |
|---|---|---|
| AUD-018 | The HTTP `estimated_loss_mw` no longer computes `milp_objective × base_mva`; it now reports the core's explicitly-defined nominal-current loss proxy `TopoReconfResult::reconf_loss_mw` (MW). Physical loss remains the separate PF-based `reconfig_loss_mw`, valid only when `reconfig_pf_converged`. | `run_gui_server.cpp` route change; existing `gui_api_e2e`/`reliability_workflow_e2e` reconfig consumers; full suite 1690/1690 |
| AUD-019 | `radial_topology_enforced` is now `!(split_domain_trees ∨ allow_dc_mesh)`, and the empty-system early return clears all `ValidityFlags`. The header documents that `*_enforced`/`*_modelled` are per-solve formulation properties while `*_validated`/`executable` are post-solve checks that stay false for a direct core solve (never a masked failure). | `test_reconfig_options` "NR-02: validity flags reflect DC mesh and empty model" |
| AUD-020 | The `TopoReconfOptions::solver` and `topology_analysis.hpp` comments now match runtime: `auto`/`highs` both run HiGHS→SCIP, `native`/`scip` are single-backend, and native branch-and-cut is documented as opt-in (weaker than HiGHS on this LinDistFlow MILP, not an automatic fallback). Dispatch behavior is unchanged. | `test_reconfig_options` "NR-03: solver strings dispatch to the declared backend" |
| AUD-021 | `base_loss_mw`, `loss_reduction_mw`, `loss_reduction_pct` are now assigned in the core as the same nominal-current proxy as `reconf_loss_mw` (base sums the initial in-service branches), so a direct library caller sees defined values, not silent zeros; the header documents them as proxies distinct from the HTTP PF-based fields. The post-validation flags stay false for a core solve, documented as "not validated by the core", never a masked failure. | `test_reconfig_options` "NR-04: core reports a defined loss-proxy base and reduction" |
| AUD-022 | `ONRResult::fallback_used` was added and is set true when the legacy wrapper reports the unchanged base topology after a failed core MILP (`feasible` true, `optimal` false, objective 0). The header documents it as a connectivity fallback, not an ONR incumbent. | `test_topology_crossval` "NR-05: legacy ONR fallback to base topology is marked" (mesh whose three fixed branches exceed the spanning-tree count → core infeasible, connected mesh power flow converges) |
| AUD-023 | The 629-line unreachable historical AC LinDistFlow branch-and-cut body after the compatibility wrapper's return was removed; `solve_optimal_reconfiguration(ACSystem)` now has one reachable model (the hybrid wrapper), and the header comment no longer claims a direct native B&C. | `test_topology_crossval`/`test_reconfig_options` full reconfig suites green (91 + 54 assertions); library rebuilds with no unused-code errors |

Findings closed by the `graph/` implementation pass:

| ID | Status | Closure | Focused evidence |
|---|---|---|---|
| AUD-024 | Closed | Planning and actions use `BusRef {domain,bus_id}` throughout candidate lookup, conflict tracking and Kron batching; AC/DC same-number buses cannot suppress each other. | `test_graph` same-number AC/DC independent Kron planning |
| AUD-025 | Closed | `mode`, voltage-constrained preservation, switch/retain actions, `ReductionMethod`, and per-action `max_fill_ratio` now have reachable behavior; the unused `RadialFeederSegment` surface was removed. | `test_graph` mode/option/action effect cases; `test_graph_kron` plan fill guard |
| AUD-026 | Closed | `ReductionMapping` has authoritative domain-qualified bus/branch forward and reverse maps, explicit invalid branch targets, operation records, AC-preferred legacy views, and `compose_reduction_mappings()`. | `test_graph` reverse-map and two-stage composition cases |
| AUD-027 | Closed | Pendant records store stable AC/DC branch component `.index` in `source_branch_index`; recovery rejects missing lookup instead of substituting `Z=0`. | `test_graph` non-sequential pendant ID, numerical voltage recovery, and missing-source exception |
| AUD-028 | Closed | Injection Kron stores `Ybb^-1 I_beta` in `KronData` and back-substitution adds it to `-Ybb^-1 Yba V_alpha`. | `test_graph_kron` full partition equation and interior residual `<=1e-12` |
| AUD-029 | Closed | A load bus with **no incident edge at all** (a topological orphan) is assigned `IslandStatus::IsolatedLoad`, emits the matching diagnostic, and reaches validation; a load bus islanded only by an out-of-service branch retains network structure and is `NoSlack` (refined by AUD-082). | `test_graph` isolated-load status; `test_validation` topology propagation |
| AUD-030 | Closed | `GraphEdge::edge_id` is graph storage position for initial, contracted, and series graphs; stable component identity remains `comp_index`/`BranchRef`. | `test_graph` post-series identity, bridge, and fundamental-cycle regressions |
| AUD-031 | Closed for declared topology/reduction scope | Graph construction covers Transformer3W, LCC, EnergyRouter connectivity and a separate three-phase graph; contraction remaps rich terminals and fails closed for unsupported phase/transformer/DCDC collapses; HTTP rejects dangling rich terminals rather than silently deleting assets. Virtual edges remain connectivity-only. | `test_graph` construction/remap/fail-closed cases; `gui_api_e2e` reduction-export-reload preservation for Transformer3W, LCC, EnergyRouter and three-phase data |

New open findings from the deep `time_series/` annual/lifecycle audit:

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| AUD-066 | Closed | Sequential annual `feasible` no longer copies only the L0 default: it aggregates weekly UC and, when replay is requested, every required OPF/PF step. `model_scope`, `schedule_only`, `physical_replay_complete` and `ens_complete` expose the certificate boundary. | `test_multiscale_comprehensive` "Annual replay failure is reported rather than masked as feasible": a step whose OPF cannot serve `500 MW` on a `10 MW` unit leaves `physical_replay_complete=false` and `feasible=false` with the replay `model_scope`. |
| AUD-067 | Closed for implemented scope | L0 generator/fuel budgets are MILP inequalities, the annual UC is one coupled horizon (SOC/ramp/commitment cross week boundaries), and `iterative_feedback` reruns physical replay with explicit penalty updates and convergence evidence. Fuel is an MWh-equivalent generation proxy because no heat-rate curve exists. | Add heat-rate/fuel-type curves if fuel rather than output budgets are required. |
| AUD-068 | Partially closed | Step curtailment is now `max(0, available-dispatch)` with source availability tracked; schedule-only ENS is marked as a schedule proxy and unknown physical ENS is not silently zero. | Add independent balance/ENS identity cases for OPF failure and export. |
| AUD-069 | Closed | Daily replay slices all current `UCSchedule` 2-D arrays, including DR, VSC/DC-DC directions, market DC storage, and solver certificates. VPP/microgrid/router state is represented only where the schedule contract contains rows. | `test_multiscale_comprehensive` "Daily replay preserves the frozen UC schedule field by field": on `ieee14_acdc` the sliced weekly schedule keeps sixteen dispatch/direction 2-D fields at the window width and the PF-only frozen-schedule replay completes. |
| AUD-070 | Closed | Replacement age is local to each stable storage index; fractional FDE cycles are retained in an internal double accumulator and public integer compatibility field; name matching is removed. | `test_multiscale_comprehensive` "Lifecycle storage: duplicate names and repeated replacement stay index-keyed": two same-name storages with distinct indices are each replaced ≥2× over 20 years with per-index cost attribution (`8.0M` vs `5.0M` USD). The lifecycle manual was corrected from the stale name-match/truncation description. |
| AUD-071 | Closed for implemented scope | Each selected stratum hour now runs an independent one-step UC-schedule OPF/PF replay; the measured correction and convergence count are reported. The estimator remains deterministic stratified sampling, so the z-bound is not a random-design confidence guarantee. | Add randomized sampling and repeated PF campaigns for a statistical coverage claim. |
| AUD-072 | Partially closed | Years, rates, confidence, loss proxy, cadence and positive sample count are validated; a nonzero `step_duration_hr` must match `TimeSeriesData`. `verbose` remains non-operative and is not advertised as a control. | Remove/deprecate `verbose` or implement observable logging. |
| AUD-073 | Closed for implemented scope | External-grid static/profile factors, AC/DC storage inventory intensity, and DC static-generator factors are included in lifecycle carbon components and JSON/API results. Embodied manufacturing carbon, network-loss carbon and converter material inventories remain outside the model. | Add authored embodied-carbon factors and a component bill-of-materials model for full asset LCA. |

Findings closed by the short-circuit theory-to-code implementation pass:

| ID | Status | Closure | Focused evidence |
|---|---|---|---|
| AUD-032 | Closed | Detailed HTTP serializes every available branch row, including authored identity and canonical diagnostic ID. | `gui_api_e2e` detailed branch assertion |
| AUD-033 | Closed | Request `c_factor` is separated from per-fault `effective_c_factor`; the batch also returns the effective-factor vector. | `gui_api_e2e` automatic-factor assertion |
| AUD-034 | Closed | AC overview/detailed and DC public options reject non-finite/out-of-domain values; HTTP propagates 400 and negative DC resistance is never clamped. | AC/DC option tests plus HTTP negative-impedance assertion |
| AUD-035 | Closed | Compensation-theorem post-fault voltages recover each DCCB edge current; radial spur and 10/5 kA parallel division are analytic regressions. | `test_dc_short_circuit` current-recovery/parallel cases |
| AUD-036 | Closed | DC uses ideal-edge contraction, sparse factorization, selected columns, batch reuse and cooperative cancellation. | 1000-bus/64-fault benchmark: batch/repeated ratio 0.0621906, identical currents |
| AUD-037 | Closed | Detailed AC/DC results carry status, message, scope, limitations and residual quality; HTTP returns complete/partial/failed counts. | direct result assertions and `gui_api_e2e` |
| AUD-038 | Closed | AC rows return `domain + component_kind + component_index + pair_number`; contracted ideal switchgear is returned with `electrical_value_available=false`, never fake zero current. | Transformer and contracted-switch identity regressions |
| AUD-039 | Closed | Zero-resistance DC edges are contracted before sparse assembly and retain exact equal-potential connectivity. | ideal-edge analytic regression |
| AUD-040 | Closed | Multiple explicit DCCBs on one branch form a series chain; ambiguous terminal-only matches across parallel branches are rejected. | series-chain and ambiguity regressions |
| AUD-042 | Closed | Every required factor and selected solve is checked; backward residual must be at most `1e-9`; failure returns `numerical_failure`. | singular-factor and finite-quality regressions |
| AUD-043 | Closed | IEC method C now uses an independent `fc/f=0.4` sparse network, generator `R_Gf`, and a single network kappa instead of sharing method A's equivalent R/X. | Comprehensive 13-bus method-C peak maximum relative error `1.15e-12` |
| AUD-044 | Closed | Three-winding zero sequence now stamps a tap-aware four-node winding/star network and Schur-eliminates the star point; impedances on different voltage bases are no longer added directly. | Seven grounded comprehensive SLG buses pass the fixed 0.5% gate |
| AUD-045 | Closed | Zero sequence is factored by connected component, so an ungrounded island no longer invalidates grounded islands; an ungrounded fault returns `solved_zero_sequence_open` and physical 0 A. | Comprehensive isolated-winding SLG regressions |
| AUD-046 | Closed | All added IEC line, transformer, and generator parameters serialize in both JSON directions. | `test_io_json [short_circuit]` passes 3 cases / 48 assertions |
| AUD-047 | Closed | GridLAB-D discovery previously missed the sibling build and allowed the cross-engine test to pass with zero GridLAB-D cases. The runner now discovers supported sibling paths and CTest requires the engine, at least 35 numerical cases, and the fixed `1e-6` error gate. | GridLAB-D 5.3.0: 35/35 cases, maximum relative error `2.329225394e-8` |
| AUD-048 | Closed | Method A previously used an equivalent R/X instead of the minimum R/X of every participating branch and feeding source path. | Independent unequal-R/X path regression; IEC comprehensive and OpenDSS matrices pass |
| AUD-049 | Closed | Steady current previously inferred lambda-like behavior and treated terminal-fed static excitation as zero. It now requires authored lambda data, uses `lambda_min` for a terminal static-excitation fault, and uses motor-free `Ibmo` for multiple-fed near faults. | Near/far, missing-data, max/min static-excitation, and motor-inclusive multiple-fed regressions |
| AUD-050 | Closed | Annex A used a fault-network kappa that could differ from the reported peak, and the OpenDSS peak oracle used the old per-source convention. Thermal `m` now uses `ip/(sqrt(2) Ik'')`; the oracle uses the same IEC method-A network rule. | Method-C thermal identity; OpenDSS 50-case peak maximum relative error `1.485279759e-7` |
| AUD-051 | Closed | Breaking current omitted the exact formula (77) voltage-depression weighting, complete mu curve interpolation, and the unbalanced `Ib=Ik''` rule. | Ten comprehensive `Ib` values (max error `1.96e-3`), curve interpolation, and unbalanced regressions |
| AUD-052 | Closed | Method B used an LV cap at all voltage levels and depended on a manual topology default. It now uses 1.8 below 1 kV, 2.0 otherwise, and defaults to independently tested `Auto` path classification. | LV/MV caps plus Auto radial/meshed and low-R/X exception regressions |
| AUD-053 | Closed | Zigzag windings were rejected and line zero-sequence shunts lacked a physical regression. Z/ZN neutral semantics now use authored zero-sequence test impedances; `b0_pu` is verified to close a capacitive earth-current path. | 2W/3W zigzag equivalence and zero-sequence capacitance open/closed-path regressions |

## Closed findings

| ID | Closure | Focused regression |
|---|---|---|
| AUD-001 | Typhoon catalogs and selected samples are immutable caller-owned `shared_ptr` snapshots, including mixed-option concurrent refresh. | `test_typhoon_traffic_impact` catalog concurrency case |
| AUD-002 | Traffic-impact inputs validate finite hydrology, wind, speed, capacity, coordinate, and ordered clamp ranges before evaluation. | `test_typhoon_traffic_impact` invalid-bound cases |
| AUD-003 | Campus results declare `isolated-campus-multi-carrier-milp`, capability flags, limitations, and aggregate PCC attribution; non-unity PF is rejected. | `test_integrated_energy_campus` scope case |
| AUD-004 | `total_transport_km` is computed from solved EV/HV/ICV distance and is zero when transport is disabled. | `test_integrated_energy_campus` transport case |
| AUD-005 | Passive conversion, storage, retention, and transfer efficiencies reject values outside `(0,1]`; heat-pump COP remains a distinct gain. | `test_integrated_energy_campus` invalid-efficiency case |
| AUD-006 | SPPT exposes `Observed`/`NotObserved`; missing MR3d LMPs are non-passing, non-observed evidence. | `test_sppt_metamorphic` MR3d case |
| AUD-007 | Formulation D leaves unavailable LMP vectors empty and reports `lmp_available=false`, a reason, and a warning. | `test_ev_power_traffic_joint_opt_d` LMP availability checks |
| AUD-008 | Every public HPF result family reports requested/converged base PF, stored-operating-point use, and model limitations. | `test_harmonics_power_flow` fallback cases |
| AUD-009 | Topology cycles explicitly contain graph-edge positions; cut vertices and propagated resilience results expose AC/DC-qualified IDs. | `test_graph` same-ID topology case |
| AUD-010 | CSV uses full-field integer parsing and quoted-field state-machine parsing; trailing text and malformed quotes are rejected. | `test_carbonflow_dynamic_storage` CSV cases |
| AUD-074 | DAE initialization/build/integration/replay exceptions become fail-closed Failed certificates and cannot abort the surrounding resilience study. | `test_transient_dynamics` `[resilience][certificate][fail_closed]` case plus completed review executable |
| AUD-075 | Proof-valid Unsafe DAE candidates generate exact topology/service cuts and trigger restoration-MIP re-optimization; Failed/Unresolved remain non-cutting. | `[resilience][certificate][mip_dae_feedback]`; review case 2 MIP solves/1 cut |
| AUD-076 | Grid-forming microgrid/source semantics, zero-dispatch storage P/E eligibility, and bus-level dispatch replay survive projection into PF/DAE. | projection B9 and `[dispatch_attribution]` regressions |
| AUD-077 | Resilience external comparison executes three frozen AC snapshots in both DSS C-API and GridLAB-D with nonzero rows and fixed voltage gates. | `external_snapshot_comparison.json/csv`, 3/3 + 3/3 pass |
| AUD-078 | Risk sampling uses geometric checkpoints and stops only after two consecutive mean/CVaR criteria pass. | 2048/2048 pairs; final max changes 0.7329%/3.1397% |
| AUD-011 | StrictHiGHS B&C calls execute on fresh joined threads, isolating them from Native adapter TLS while Native retains the caller thread's required stack capacity. | `test_resilience_assessment` five-cycle ordered backend case |
| AUD-054 | HPF linear and Newton result families derive `ok` from per-order solution or final convergence; a failed factorization/iteration cannot report success. | `test_harmonics_power_flow` invalid-option and forced non-convergence cases |
| AUD-055 | AC harmonic terminal currents reuse the exact pi/tap/phase-shift Ybus stamp; copper loss uses series current and `R(h)`. | 1.1-tap/17-degree/charging analytic regression |
| AUD-056 | Canonical transformer equivalents are stamped once; the rich `Transformer2W` collection is not counted a second time. | 10% transformer exact `V5=j1.5 pu` regression |
| AUD-057 | Three-phase transformer parsing distinguishes Y/YN, Z/ZN, and Delta; zero-sequence paths use projectors and authored `vk0/vkr0`. | Yy0/YNyn0/ZNyn0 and 10%/20% zero-sequence regressions |
| AUD-058 | HPF options reject invalid, non-finite, non-positive, and duplicate order/numerical settings before assembly. | duplicate-order regression and result-message check |
| AUD-059 | GridLAB-D is a mandatory numerical gate with 24 executed frequency slices, not a capability-only row. | `harmonics_cross_engine_matrix`, max error `5.349716508e-10 pu` |
| AUD-062 | First-class DC capacitor/reactor and AC/DC harmonic-filter identity survives JSON, projection and terminal-current attribution. | `test_harmonics_power_flow` JSON, analytic capacitor and attribution cases |
| AUD-063 | HSS off-diagonal frequency indexing degenerates to per-order HPF at zero coupling and matches an independent two-frequency closed form. | `test_harmonics_power_flow`, `1e-10 pu` gates |
| AUD-064 | Enabled two-level VSC, MMC, LCC and supported DC/DC models stamp nonzero cross-frequency blocks; incomplete or unsupported models reject explicitly. | converter-family HSS sections and validation failures |
| AUD-065 | Every successful HSS solve passes a normalized backward-error gate and reports sparse dimensions/nonzeros; non-finite assembly is rejected before factorization. | HSS analytic, converter and 1000-node tests |
| AUD-060 | IEEE13 OpenDSS validation has a fixed `2e-3 pu` exit gate and is registered in CTest. | 164 points, max error `1.652e-3 pu` |
| AUD-061 | Frequency scans reject invalid ranges/unknown buses, expose one solve flag per frequency, use NaN rather than fabricated zero on failure, and aggregate `ok`. | scan range and unknown-bus regressions |

The original evidence and required closure statements are retained below as
the review record. The table above is the current status.

### High

#### AUD-001: typhoon catalog cache returns references that can be invalidated concurrently

- Module: `scenario_generation/` (focused contract added)
- Evidence: `src/scenario_generation/typhoon_resilience.cpp:949-979`,
  `tests/run_gui_server.cpp:2093-2124`,
  `src/scenario_generation/scenario_generation.cpp:2436-2453`
- `get_or_build_typhoon_catalog()` serialises mutation of one process-global
  cache, then returns `const TyphoonCatalog&` after releasing the mutex. A
  concurrent request with different options can replace that object while the
  first caller reads it. `sample_typhoon_catalog()` additionally returns a
  pointer into the same storage.
- Impact: concurrent HTTP or scenario-generation requests can race, observe a
  catalog built for different options, or dereference invalidated sample
  storage.
- Required closure: return owned immutable storage (for example a
  `shared_ptr<const TyphoonCatalog>`) or retain per-key immutable cache entries;
  add a concurrent mixed-option regression.

#### AUD-002: typhoon traffic validation allows invalid `std::clamp` bounds

- Module: `scenario_generation/` (focused contract added)
- Evidence: `src/scenario_generation/typhoon_traffic_impact.cpp:176-185` and
  `196-209`, with later clamps at lines 269 and 278.
- Validation does not reject non-finite values, negative
  `maximum_surface_water_mm`, `minimum_open_capacity_factor > 1`, or
  `minimum_open_speed_factor` outside `[0, 1]`. Those values can reverse the
  lower/upper arguments to `std::clamp`, whose contract requires an ordered
  range, or produce nonphysical travel times.
- Impact: malformed public options can trigger undefined behavior or invalid
  road profiles instead of a typed input error.
- Required closure: validate every finite range before computation and add
  boundary/non-finite tests.

#### AUD-003: campus IES exposes electrical PCC fields without network coupling

- Module: `integrated_energy/` (focused contract added)
- Evidence: public fields at
  `include/hacdcpf/integrated_energy/integrated_energy_system.hpp:12-15`, solver
  entry at `src/integrated_energy/integrated_energy_optimizer.cpp:369-389`, and
  result construction at lines 815-819.
- `pcc_ac_bus` and `fixed_power_factor` are not consumed by
  `solve_campus_ies()`. The solver accepts no `HybridPowerSystem`, voltage,
  reactive-power, or branch-limit data; `p_pcc_mw` is only export minus import.
  `CampusIESResult` also has no `model_scope`, limitation, fallback, or gap
  availability fields.
- Impact: callers can interpret the result as grid-coupled integrated-energy
  optimisation even though it is an isolated multi-carrier balance model.
- Required closure: either implement explicit network coupling or remove/name
  the unused fields accordingly, and add honest scope/validity metadata.

#### AUD-011: resilience solver result depends on an earlier solver call in the same process

- Module: `resilience/` (distributed documentation)
- Evidence: `tests/test_resilience_assessment.cpp:817-859` followed by lines
  1033-1087; backend dispatch at
  `src/resilience/resilience_restoration_mip.cpp:1586-1603`.
- On the rebuilt Debug sanitizer target, running `Resilience strict MIP honors
  external MESS availability` (Native) before `Resilience: strict MIP models
  hybrid transfer components and DC faults` (StrictHiGHS) makes the second
  solve return `StrictHiGHS Other run=-1` with no incumbent. The second case
  passes alone with 26 assertions. The same order dependence occurs in the
  existing macOS Release binary.
- Impact: a long-lived process can report an infeasible hybrid restoration
  model based on prior solver use. Per-test CTest process isolation hides this
  failure mode.
- Required closure: identify and reset or isolate mutable solver state across
  Native/StrictHiGHS calls; register a same-process ordered regression and run
  it under sanitizers and the production server execution model.

#### AUD-074: DAE certificate initialization exception escaped the resilience study

- Module: `resilience/`.
- Evidence: the end-to-end review case terminated during
  `MultiFidelityCertificateEngine::evaluate` when dynamic power-flow
  initialization rejected a represented restoration state. The public
  certificate interface had no exception-to-result boundary.
- Impact: a single uninitializable action aborted the remaining weak-link and
  risk study, leaving partial artifacts and no structured failure
  certificate.
- Closure: `src/resilience/certified_restoration.cpp` now maps standard and
  cross-ABI exceptions to `CertificateLabel::Failed`, with
  `proof_valid=false` and `simulation_success=false`. The focused
  `[resilience][certificate][fail_closed]` test passes 6 assertions. The full
  review then completed 2048 paired samples, while the dynamic feedback loop
  separately cut a proof-valid Unsafe incumbent and certified the replacement.
  Fail-closed itself establishes process safety only; dynamic feasibility comes
  from the later proof-valid Safe certificates.

### Medium

#### AUD-004: disabling transport still reports authored transport demand as solved activity

- Module: `integrated_energy/` (focused contract added)
- Evidence: `src/integrated_energy/integrated_energy_optimizer.cpp:615` sets
  modelled transport to zero when disabled, while line 832 always sums
  `data.transport_demand_km` into `result.total_transport_km`.
- Impact: result totals contradict the solved model when
  `enable_transport=false`.
- Required closure: derive the total from solved `d_ev_km`, `d_hv_km`, and
  `d_icv_km`, or report authored demand in a separately named field.

#### AUD-005: integrated-energy normalisation accepts efficiencies above unity

- Module: `integrated_energy/` (focused contract added)
- Evidence: `src/integrated_energy/integrated_energy_optimizer.cpp:42-51` and
  applications at lines 153-168.
- `bounded_efficiency()` accepts values through `1.5` for conversion, storage,
  retention, and transfer efficiencies. Values above one permit energy
  creation in the carrier balances. Other invalid public values are silently
  replaced or clamped without a result diagnostic.
- Impact: feasible and apparently optimal results can represent an unintended
  physical model while hiding that inputs were changed.
- Required closure: reject efficiencies outside their documented physical
  range, or introduce explicitly named gain/COP parameters; report every
  sanitisation in result limitations.

#### AUD-006: the public SPPT MR3d relation passes when no nodal prices exist

- Module: `sppt/` (focused contract added)
- Evidence: `src/sppt/metamorphic.cpp:216-269` and suite registration at lines
  499-508; `src/sppt/certificate.cpp:135-140` uses a different interpretation.
- `mr3_semantic_preservation_opf_dual()` returns `passed=true` when either LMP
  vector is empty, so `run_core_metamorphic_suite()` reports the OPF-price
  relation as passing without observing prices. The certificate layer instead
  marks the same result as not converged/available.
- Impact: direct consumers of the relation or core suite can publish a false
  semantic-preservation pass. The current test at
  `tests/test_sppt_metamorphic.cpp:325-331` checks only `passed`.
- Required closure: model `pass`, `fail`, and `not_applicable/not_observed`
  separately and require non-empty, correctly sized LMP vectors for a pass.

#### AUD-007: Formulation D advertises LMP output but returns empty maps

- Module: `ev_power_traffic/` (distributed documentation)
- Evidence: result contract at
  `include/hacdcpf/ev_power_traffic/simulation.hpp:310-315`; decoding at
  `src/ev_power_traffic/joint_optimizer.cpp:2613-2630`. The certified dynamic
  and full-NLP paths similarly resize empty maps at lines 1624-1638 and
  `src/ev_power_traffic/joint_optimizer_full_nlp.cpp:1676-1689`.
- With `include_dcopf=true`, generator dispatch is decoded but dual variables
  are not extracted. `lmp_by_step` is resized to the horizon with every map
  empty, without an availability flag or warning.
- Impact: a non-empty outer vector can be mistaken for populated nodal prices;
  price-feedback consumers receive no usable values despite the public field
  contract.
- Required closure: extract duals where the backend supports them, otherwise
  expose `lmp_available=false` and leave an explicit limitation. Extend the D2
  test beyond generator dispatch.

#### AUD-008: hybrid and Newton harmonic fallbacks are not disclosed consistently (closed)

- Module: `harmonics_power_flow/` (distributed documentation)
- Evidence: swallowed base-PF failure at
  `src/harmonics_power_flow/harmonics_power_flow.cpp:1535-1542` and successful
  return at lines 1768-1769; the single-phase Newton path obtains the same
  hidden fallback at lines 1796 and 1880-1882. In contrast, `HPFResult` and
  `HPF3phResult` report `base_pf_converged` and a fallback message.
- Impact: callers requesting a base power flow can receive `ok=true` harmonic
  results based on stored/nominal voltages without knowing the operating-point
  solve failed.
- Required closure: add base-PF validity and model-limitation fields to every
  harmonic result family and preserve exception/non-convergence diagnostics.

#### AUD-009: graph topology exposes conflicting and domain-ambiguous index contracts

- Module: `graph/` (focused contract added)
- Evidence: `include/hacdcpf/graph/topology_analysis.hpp:99-101` describes
  `fundamental_cycles` as node indices, while the API contract at lines 131-133
  and `src/graph/topology_analysis.cpp:256-286` return graph edge indices.
  `cut_vertex_bus_ids` is a flat integer list even when AC and DC IDs overlap;
  the HTTP layer explicitly works around this at
  `tests/run_gui_server.cpp:14831-14857`.
- Impact: direct library consumers can index the wrong collection or conflate
  AC and DC articulation points. The ambiguity can propagate into resilience
  results through `src/resilience/resilience_assessment.cpp:1289-1305`.
- Required closure: correct the cycle field contract and add domain-qualified
  cut-vertex/diagnostic references while retaining legacy fields only as
  explicitly deprecated compatibility output.

### Low

#### AUD-010: carbon CSV integer fields accept trailing text

- Module: `carbon_analysis/` (focused contract added)
- Evidence: `src/carbon_analysis/annual_carbon_analysis.cpp:2093-2100` and
  2226-2234 call `std::stoi` without checking the consumed length, unlike the
  strict double parser at lines 2038-2051.
- Impact: values such as `12abc` are silently accepted as ID `12`, weakening
  import validation and making malformed data hard to diagnose.
- Required closure: use the `pos` overload and require full-field consumption;
  add parser tests for trailing text and quoted/escaped CSV behavior.

## Module audit matrix

The matrix below is retained from prior module audits. Its depth labels and
test counts are historical; the current review's narrower coverage and new open
findings are listed above.

| Module/domain | Documentation | Audit depth | Current result |
|---|---|---|---|
| `model/` | Focused | Deep | Independent quotient/recovery/unit oracle passes its declared algebraic checks, but found an open contract defect: sparse authored DC dead-bus ID 5 is reported as prestrip position token 3 in `dc_dead_bus_indices`; position recovery remains correct. |
| `validation/` | Focused | Baseline | Active validation manual and regression baseline retained. |
| `projection/`, `assembly/` | Focused | Sampled | Core AC/DC map and attribution invariants checked; no new finding recorded. |
| `power_flow/` | Focused | Baseline | Active manual and regression baseline retained; the point-in-time math audit is archived. |
| `optimal_power_flow/` | Focused | Baseline | Active OPF manual retained; point-in-time diagnostics and validation are archived. |
| `power_models/` | Focused | Baseline | Ownership and AML builder documentation confirmed. |
| `graph/` | Focused | Deep | AUD-009 and AUD-024--AUD-031 closed for the declared topology/reduction scope. Focused Release: four graph targets pass 76 cases / 629 assertions; validation adds 38/180, GUI E2E passes 80 checks, and the selected ASan/UBSan run passes 42 with one conditional skip. Connectivity-only rich virtual edges, HTTP Kron identify-only, and pendant approximation remain explicit model boundaries. |
| `network_reconfiguration/` | Focused | Deep | AUD-018--AUD-023 open: HTTP loss units, validity timing, solver-comment drift, unpopulated result fields, hidden ONR fallback, and unreachable legacy model. Focused Release rebuild: five focused targets pass 30 cases / 401 assertions; numerical chapter records exhaustive, BFS, PF and pipeline cross-validation with proxy limitations. |
| `reliability/` | Focused | Deep | AUD-015 closed: tail-risk LOLE-vs-EENS length guard and ASAI `[0,1]` clamp; result-scope invariants checked. |
| `resilience/` | Distributed | Deep | AUD-011/AUD-074--078 closed; DAE feedback, 2048-pair convergence and three-snapshot external AC comparison are admitted only within their declared scopes. |
| `analysis/` | Distributed | Sampled | Focused submodule documents exist; no umbrella result contract. |
| `scenario_generation/` | Focused | Deep | AUD-001 and AUD-002 closed. |
| `short_circuit/` | Focused | Deep | AUD-032--AUD-053 are closed with zero open findings in the detailed IEC 60909 calculation scope. Release direct short-circuit targets pass 57 cases / 917 assertions; focused JSON passes 3/48. IEC §6.2 plus a 13-bus comprehensive network, 50 OpenDSS complete-network cases, mandatory 35-case GridLAB-D 5.3.0 gate, IEEE 13/34/123 external-Thevenin fault kernels, 77-check production API E2E, authored identity, analytic DCCB division and a 1000-bus sparse batch benchmark are recorded in the manual. EMT, controller, protection and IEC 61660 studies belong to their dedicated model families and are not represented as unfinished IEC 60909 work. |
| `harmonics_power_flow/` | Focused | Deep | AUD-008 and AUD-054--AUD-065 closed. Ten implementation/theory/validation chapters cover per-order and HSS periodic steady state. Focused test count and external evidence are recorded below. |
| `dynamics/` | Focused | Deep | AUD-016 and AUD-079--AUD-081 are closed for the declared scope: defective small-signal participation has an explicit fallback; mixed AC/DC DC/DC initialization and DER_A state equations are source-aligned; MassMatrixDae numerically localizes IEEE 1547 plus direct-API definite-time UVLS/local-frequency/positive-sequence Zone-1 events, uses an anchored forward clustering window, and audits post-event algebraic consistency. Direct relays use local positive-sequence PT, CT and angle-frequency filters. Rich-model/HTTP relay assembly, partitioned integrators, grazing/Zeno, inverse-time, multi-zone distance and EMT remain explicit boundaries; EMT measurement is rejected. COSMIC's public example matches its fixed source, while its paper Fig. 2 and PSD Test 42 trajectory closure remain explicit external gaps. |
| `time_series/` | Focused | Deep | AUD-066/069/070 are now regression-closed with dedicated `test_multiscale_comprehensive` fixtures (forced replay failure, field-by-field frozen daily replay, duplicate-name multi-replacement); AUD-067/068/071/072/073 remain partial with explicit scope. Existing focused rerun passes and the full Release suite is green. |
| `carbon_analysis/` | Focused | Deep | AUD-010 closed. |
| `ev_power_traffic/` | Distributed | Deep | AUD-007 closed. |
| `integrated_energy/` | Focused | Deep | Ten-chapter source-equivalent monograph plus registered analytic/Python cross-validation; AUD-003, AUD-004, and AUD-005 remain closed. The isolated-campus electrical boundary and missing external full-MILP oracle are explicit. |
| `market/` | Focused | Deep | AUD-017: index-space deep review (position/index dual-key, LMP dual extraction, N-1 cut loop) found no defect; buses-never-filtered ⇒ LMP-authored-position invariant recorded. |
| `sppt/` | Focused | Deep | Eleven-chapter theory-to-executable monograph. Independent residual supports 6/10 certificate cases; repetitions=10/20 scale campaigns terminate on uncaught `std::invalid_argument`, so statistical closure remains open. MR8 has no public relation API. |
| `io/` | Focused | Baseline | Format ownership and clean-clone documentation checked. |
| `api/`, `src/server/`, `web/` | Focused | Sampled | Typhoon snapshot ownership and domain-qualified graph output are verified at the HTTP boundary. |

## Verification evidence

The following targets were rebuilt from current source in
`/private/tmp/hysim_reliability_config_debug` with ETAP, Ipopt, and OpenDSS
disabled and the local dependency dirty-check override:

| Scope | Result |
|---|---|
| Typhoon traffic/catalog | `test_typhoon_traffic_impact`: 5 cases, 35 assertions |
| Campus integrated energy | `test_integrated_energy_campus`: 6 cases, 106 assertions |
| SPPT executable layer | 7 targets: 33 cases, 213 assertions |
| Harmonics | `test_harmonics_power_flow`: 63 cases, 412 assertions; cross-engine 14/14, GridLAB-D 24 slices, IEEE13/OpenDSS 164 points |
| EV Formulation D | `test_ev_power_traffic_joint_opt_d`: 15 cases, 196 assertions |
| Graph/reduction | Focused Release rebuild of four graph targets: 76 cases, 629 assertions; serial runtime about 0.51 s |
| Graph validation/runtime | `test_validation`: 38 cases, 180 assertions; GUI E2E: 80 checks; selected ASan/UBSan: 42 passed, 1 conditional skip |
| Scenario generation/schema | 2 targets: 12 cases, 105 assertions |
| Carbon snapshot/annual/GEC | 3 targets: 41 cases, 574 assertions |
| Resilience/reliability shared suite | `test_resilience_assessment`: 39 cases, 360 assertions, including five Native-to-StrictHiGHS cycles |
| Runtime server | `run_gui_server` compiled and linked against the changed contracts |

The monograph pass rebuilt and passed the registered Release
`integrated_energy_cross_validation` and `model_projection_cross_validation` tests.
The first includes two analytic optimal solutions and a 24-hour independent Python
equation/cost oracle; the second includes independent set, conservation, recovery,
domain-qualified identity, dead-island and per-unit checks while retaining the failed
stable dead-bus ID contract. Existing direct binaries also passed 6 cases/106
assertions for campus integrated energy, 30 cases/1696 assertions for model, and 41
cases/246 assertions across eight SPPT targets. SPPT tools additionally ran 10-case
certification, 8-case scaling, ablation and a 315-sample fixed-seed campaign. No full
CTest, sanitizer, external PF/MILP engine, or pinned-dependency baseline was run.

An exact packaged-task/jthread microbenchmark measured 0.0145--0.0187 ms per
create/run/join cycle across five 1000-cycle runs, below the 10 ms fixed-overhead
threshold. The full resilience test executable completed after isolation; no
material runtime increase was observable at its reported precision.

No full CTest, complete sanitizer suite, thread race detector, or
external-engine cross-validation was performed for this closure pass.

## Closure rules

The protection closure pass also covers semantic event-target matching,
invalid topology-target rejection, same-timestamp post-topology closure, an
anchored non-rolling future event window, and local phasor CT/PT frequency and
distance measurements. Focused regressions are in
`tests/test_transient_dynamics.cpp` under `[event_target]`, `[frequency]`,
`[ct_pt]`, and `[failure]`; the protection subset passed 16 cases / 380
assertions after the change, including the rule that low-voltage recovery needs
one complete interval with valid PT voltage at both endpoints before frequency
protection is unblocked.

1. Keep a finding open until the implementation, public result contract, and a
   focused regression agree.
2. Add the focused module contract before changing an `Uncovered` label.
3. Record units, authored/canonical index space, fallback, approximation,
   time-limit, and unavailable outputs in public results.
4. Move volatile test evidence to `development_status.md` when this audit is
   superseded; do not create dated copies of this file.

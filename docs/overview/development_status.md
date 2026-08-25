# Development Status

Updated: 2026-08-24

This is the living handoff for verified build state and active engineering work.
Update it in place; do not create dated copies. Source, registered tests, and
the current Git worktrees remain authoritative.

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

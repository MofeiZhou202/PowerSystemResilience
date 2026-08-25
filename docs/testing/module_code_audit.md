# Module Code Audit

Updated: 2026-08-24

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

## Open findings

All data-structure review findings R-01–R-08
（[数据结构设计评审](../developer/data_structure_design_review.md) §2）已关闭
or reclassified this iteration; see the table below. AUD-001 through AUD-011
remain closed.

Addressed this iteration:

| ID | Finding | Closure | Regression |
|---|---|---|---|
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

# Module Code Audit

Updated: 2026-08-22

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
| AUD-012 | DC-side dead islands were never stripped (review R-01/R-02): AC-only strip could leave an unsourced or orphaned DC bus, risking a singular `Gdc`. | `strip_dead_dc_islands()` removes unsourced DC islands and records `dc_prestrip_to_survivor` on the certificate; `unproject_dc_bus_vector()` recovers DC voltages to authored space (0 pu at a stripped bus). Recovery is wired into the PF facade, `solve_handle`, `solve_dc_power_flow`, and AC OPF. The strip criterion keeps DC buses reached by an out-of-service converter or a closed DC breaker. | `test_component_models_math_audit` "R-01/R-02: DC dead islands are stripped..." and "...unproject_dc_bus_vector recovers..."; cross-suite guards `test_opf_solver_backends`, `test_distribution_pipeline` |
| AUD-013 | Standalone `merge_zero_impedance_buses` ignored `ACBranch::ideal_connectivity` (review R-04), diverging from the projection whitelist path. | The nullptr path now contracts `ideal_connectivity` branches regardless of magnitude, additively, without changing its tested numeric-threshold behavior. | `test_component_models_math_audit` "R-04: standalone merge honors ideal_connectivity..." |
| AUD-014 | Merge dropped `ACBus::importance` (R-06); `unproject_bus_vector` silently returned empty for an unbuilt map (R-07); merge participation basis ignored detailed `Charger` rows (R-08). | Merge aggregates `max(importance)`; `unproject_bus_vector` throws on a non-empty input against an `n_merged==0` map; `charging_station_effective_kw` prefers detailed charger demand. R-03 (load-basis participation) and R-05 (`MobileStorage` is AC-domain) are confirmed by-design with locking regressions. | `test_component_models_math_audit` R-03/R-05/R-06/R-07/R-08 cases |
| AUD-015 | Reliability review (RL-01/RL-02): `compute_tail_risk` could index past the LOLE sample vector when the LOLE series is shorter than the EENS series; `compute_distribution_indices` could emit `asai` outside `[0,1]` when `saidi > hours_per_year` (e.g. multi-interruption microgrids). | `compute_tail_risk` resizes `sorted_lole` to the EENS length (0-fill) before percentile indexing ([reliability_assessment.cpp#L2041](../src/reliability/reliability_assessment.cpp#L2041)); `asai = std::clamp(1 - saidi/H, 0, 1)` ([#L2351](../src/reliability/reliability_assessment.cpp#L2351)). | `test_reliability_resolver` RL-01 (shorter-LOLE tail risk) and RL-02 (ASAI clamp) cases |
| AUD-016 | Dynamics review (DY-01): `small_signal_analysis` computed participation factors from `R.inverse()` without guarding a defective/near-singular reduced Jacobian, so a non-finite left-eigenvector inverse could poison every participation factor. | When `Linv = R.inverse()` is not all-finite the participation loop falls back to right-eigenvector magnitude (`rk*rk`) and `result.message` declares the defective-Jacobian fallback ([SmallSignal.cpp#L423](../src/dynamics/SmallSignal.cpp#L423)). | `test_transient_dynamics` small-signal suite (the defective-matrix branch is documented as impractical to force through the public API) |
| AUD-017 | Market pricing index-space deep review (position/index dual-key, LMP dual extraction, N-1 cut loop). | **No defect.** LMP/shed/curtail use a direct authored-bus index that is correct only because AC/DC buses are never filtered (`build.B == buses.size()`, `b ≡ authored position` — [market_simulation.cpp#L958](../src/market/market_simulation.cpp#L958)); balance/reserve/dc_balance duals all live in the equality block and are read as `constraint_duals[inequality_rows + row]` ([#L2758](../src/market/market_simulation.cpp#L2758)); settlement keeps both `generator_position` (authored) and `generator_index` (stable); the N-1 cut loop dedups via `cut_keys`, breaks on `added==0`, and is bounded by `max_iterations`. The **buses-never-filtered ⇒ LMP-authored-position** invariant is recorded as a design guarantee to preserve. | Existing `test_market_simulation` settlement (per-authored-bus `lmp_per_mwh[b]`) and hybrid-DC (`dc_*[0]`/`[1]` at authored positions) cases pin the direct-index attribution |
| AUD-041 | The English and Chinese rich AC/DC short-circuit derivations stated that DCCB state/resistance was unused and later retained a stale full-bus-current duty approximation. | Both living theory notes now describe ideal-edge contraction, unambiguous matching, open/closed topology, actual resistive-edge current recovery, series chains, parallel division, and the remaining EMT boundary. | Documentation drift scan plus DCCB analytic regressions |

New open findings from the deep `network_reconfiguration/` audit:

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| AUD-018 | High | `POST /api/session/run_reconfig` reports `estimated_loss_mw = milp_objective * base_mva`. The objective mixes weighted switching, shedding, island, and loss terms and omits switching constants, so the conversion is dimensionally invalid. | Remove/rename the field or map it to an explicitly defined proxy; use post-PF `reconfig_loss_mw` for physical loss and add an HTTP regression with nonzero switching/shedding terms. |
| AUD-019 | Medium | `TopoReconfResult::ValidityFlags` sets radial/device/protection flags before feasibility is known; `radial_topology_enforced` remains true when `allow_dc_mesh=true`. Failed solves and meshed DC results can therefore look fully certified. | Separate requested/modelled/enforced/validated states and report AC/DC radiality independently; add infeasible and DC-mesh result-contract tests. |
| AUD-020 | Medium | Public solver comments claim `auto` is HiGHS→Native and the historical ONR directly uses native B&C/MIR. Reachable code dispatches `auto` and `highs` as HiGHS→SCIP; the old AC B&C body is after an unconditional return. | Align public comments with runtime behavior, decide whether Native fallback is intended, and register dispatch/fallback tests for all solver strings. |
| AUD-021 | Medium | Core result fields `base_loss_mw`, `loss_reduction_mw`, `loss_reduction_pct`, and four post-action/PF/OPF/executable flags have no assignment path. HTTP mutates a local result copy or computes separate locals, while library callers cannot distinguish not-evaluated from failed. | Remove the fields, make availability explicit, or expose a documented post-validation API that populates them; add direct-library field tests. |
| AUD-022 | Medium | The legacy `solve_optimal_reconfiguration` wrapper converts a failed core solve into `feasible=true` when the unchanged connected AC topology passes PF, with objective 0 and no fallback/status/scope marker. | Preserve the core failure and expose `fallback_used`/scope, or keep optimization feasibility false; add a regression that forces this branch. |
| AUD-023 | Low | Roughly 600 lines of the historical AC LinDistFlow B&C implementation remain unreachable after the compatibility wrapper's unconditional return, while comments and test prose still describe it as active. | Delete/move the historical implementation or restore it as a separate explicit, tested entry; keep one reachable model per function. |

New open findings from the deep `graph/` audit:

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| AUD-024 | High | Reduction candidates carry `NodeDomain`, but `make_reduction_plan()` keys candidate lookup, eliminated sets, neighbour conflicts, and the batch Kron list by bare integer bus IDs. Same-number AC/DC candidates can suppress each other, and one Kron action has only its default AC `bus_domain`. | Use domain-qualified keys throughout planning/actions and add same-ID AC/DC multi-action regressions. |
| AUD-025 | Low | The public reduction surface drifts from reachable behavior: `mode`, `preserve_all_voltage_constrained_buses`, and `ReductionMethod` are unused; switch contraction and `max_fill_ratio` are not wired through the plan; switch/retain actions and `RadialFeederSegment` have no generation path. | Remove unsupported surface or implement every option/action with effect tests and explicit diagnostics. |
| AUD-026 | Medium | `ReductionMapping` is not a complete bidirectional or multi-stage certificate: reverse bus maps remain identity after series/pendant, pendant branch maps are empty, mapping-level switch/Kron records are never populated, and no composition API exists. | Populate domain-qualified forward/reverse maps for every operation and add composition plus round-trip tests. |
| AUD-027 | High | Pendant reduction records the graph `edge_id` as `PendantReductionRecord::branch_id`, while voltage recovery looks it up as a source branch component `.index`. With non-sequential component IDs, impedance lookup can fail and silently recover with `Z=0`. | Use one declared ID space, reject lookup failure, and add a non-sequential-ID numerical recovery regression. |
| AUD-028 | Medium | Dense Kron with injection computes `I_reduced`, but recovery only evaluates `V_beta=-K V_alpha`; it cannot add `Ybb^-1 I_beta` because the recovery API accepts no interior injection. | Store/accept the injection correction and verify the full partitioned equation. |
| AUD-029 | Medium | `IslandStatus::IsolatedLoad` is never assigned: topology emits only `GraphIsolatedLoad`, while `validate_system.cpp` checks the unreachable status branch. | Unify status/diagnostic semantics and register topology-to-validation isolated-load tests. |
| AUD-030 | Medium | A series-created edge receives an `edge_id` beyond existing graph/component IDs while adjacency, bridge, and cycle APIs continue to use vector positions. The initial convenience relation `edge_id == edges[] position` no longer holds after reduction. | Separate these types or renumber graph-internal IDs consistently, then add post-series topology/index regressions. |
| AUD-031 | High | Graph construction omits Transformer3W, LCC, EnergyRouter, and three-phase topology and under-flags several rich injections. Contraction does not remap multiple rich asset terminals; HTTP compact export then filters dangling objects, potentially deleting assets instead of preserving them. | Complete rich-component graph/remap coverage or reject unsupported reductions, with per-component HTTP reload and semantic round-trip tests. |

New open findings from the deep `time_series/` annual/lifecycle audit:

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| AUD-066 | Closed in code; regression pending | Sequential annual `feasible` no longer copies only the L0 default: it aggregates weekly UC and, when replay is requested, every required OPF/PF step. `model_scope`, `schedule_only`, `physical_replay_complete` and `ens_complete` expose the certificate boundary. | Add a forced failed-week/replay fixture before upgrading this to independently regression-closed. |
| AUD-067 | Closed for implemented scope | L0 generator/fuel budgets are MILP inequalities, the annual UC is one coupled horizon (SOC/ramp/commitment cross week boundaries), and `iterative_feedback` reruns physical replay with explicit penalty updates and convergence evidence. Fuel is an MWh-equivalent generation proxy because no heat-rate curve exists. | Add heat-rate/fuel-type curves if fuel rather than output budgets are required. |
| AUD-068 | Partially closed | Step curtailment is now `max(0, available-dispatch)` with source availability tracked; schedule-only ENS is marked as a schedule proxy and unknown physical ENS is not silently zero. | Add independent balance/ENS identity cases for OPF failure and export. |
| AUD-069 | Closed in code; regression pending | Daily replay slices all current `UCSchedule` 2-D arrays, including DR, VSC/DC-DC directions, market DC storage, and solver certificates. VPP/microgrid/router state is represented only where the schedule contract contains rows. | Add a field-by-field frozen schedule replay test. |
| AUD-070 | Closed in code; regression pending | Replacement age is local to each stable storage index; fractional FDE cycles are retained in an internal double accumulator and public integer compatibility field; name matching is removed. | Add duplicate-name and multi-replacement numerical fixtures. |
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
| `model/`, `validation/` | Focused | Baseline | No new finding in this pass. |
| `projection/`, `assembly/` | Focused | Sampled | Core AC/DC map and attribution invariants checked; no new finding recorded. |
| `power_flow/` | Focused | Baseline | Active manual and regression baseline retained; the point-in-time math audit is archived. |
| `optimal_power_flow/` | Focused | Baseline | Active OPF manual retained; point-in-time diagnostics and validation are archived. |
| `power_models/` | Focused | Baseline | Ownership and AML builder documentation confirmed. |
| `graph/` | Focused | Deep | AUD-009 closed; AUD-024--AUD-031 open across domain-safe planning, option drift, mapping/recovery, edge identity, island status, and rich-component coverage. Focused Release rebuild: three direct targets pass 56 cases / 467 assertions; numerical chapter records Kron/current/recovery and PF/OPF round-trip errors. |
| `network_reconfiguration/` | Focused | Deep | AUD-018--AUD-023 open: HTTP loss units, validity timing, solver-comment drift, unpopulated result fields, hidden ONR fallback, and unreachable legacy model. Focused Release rebuild: five focused targets pass 30 cases / 401 assertions; numerical chapter records exhaustive, BFS, PF and pipeline cross-validation with proxy limitations. |
| `reliability/` | Focused | Deep | AUD-015 closed: tail-risk LOLE-vs-EENS length guard and ASAI `[0,1]` clamp; result-scope invariants checked. |
| `resilience/` | Distributed | Deep | AUD-011 closed; AUD-009 propagation now exposes AC/DC cut-vertex lists. |
| `analysis/` | Distributed | Sampled | Focused submodule documents exist; no umbrella result contract. |
| `scenario_generation/` | Focused | Deep | AUD-001 and AUD-002 closed. |
| `short_circuit/` | Focused | Deep | AUD-032--AUD-053 are closed with zero open findings in the detailed IEC 60909 calculation scope. Release direct short-circuit targets pass 57 cases / 917 assertions; focused JSON passes 3/48. IEC §6.2 plus a 13-bus comprehensive network, 50 OpenDSS complete-network cases, mandatory 35-case GridLAB-D 5.3.0 gate, IEEE 13/34/123 external-Thevenin fault kernels, 77-check production API E2E, authored identity, analytic DCCB division and a 1000-bus sparse batch benchmark are recorded in the manual. EMT, controller, protection and IEC 61660 studies belong to their dedicated model families and are not represented as unfinished IEC 60909 work. |
| `harmonics_power_flow/` | Focused | Deep | AUD-008 and AUD-054--AUD-065 closed. Ten implementation/theory/validation chapters cover per-order and HSS periodic steady state. Focused test count and external evidence are recorded below. |
| `dynamics/` | Focused | Deep | AUD-016 closed: small-signal participation falls back to right-eigenvector magnitude on a defective reduced Jacobian; active runtime contract confirmed. |
| `time_series/` | Focused | Deep | AUD-066/069/070 are closed in runtime code pending dedicated regressions; AUD-067/068/071/072/073 remain partial with explicit scope. Existing focused rerun passes 11/11 direct tests; new Release rebuild and numerical evidence must be recorded before claiming full closure. |
| `carbon_analysis/` | Focused | Deep | AUD-010 closed. |
| `ev_power_traffic/` | Distributed | Deep | AUD-007 closed. |
| `integrated_energy/` | Focused | Deep | AUD-003, AUD-004, and AUD-005 closed. |
| `market/` | Focused | Deep | AUD-017: index-space deep review (position/index dual-key, LMP dual extraction, N-1 cut loop) found no defect; buses-never-filtered ⇒ LMP-authored-position invariant recorded. |
| `sppt/` | Focused | Deep | AUD-006 closed. |
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
| Graph/reduction | Focused Release rebuild `test_graph`, `test_graph_kron`, `test_graph_roundtrip`: 56 cases, 467 assertions; numeric logs in `/private/tmp/hysim_graph_nr_evidence_20260821/` |
| Graph cross-module | Existing `macos-release` `test_distribution_pipeline`, `test_three_phase_hybrid_opf`: 13 cases, 330 assertions |
| Scenario generation/schema | 2 targets: 12 cases, 105 assertions |
| Carbon snapshot/annual/GEC | 3 targets: 41 cases, 574 assertions |
| Resilience/reliability shared suite | `test_resilience_assessment`: 39 cases, 360 assertions, including five Native-to-StrictHiGHS cycles |
| Runtime server | `run_gui_server` compiled and linked against the changed contracts |

An exact packaged-task/jthread microbenchmark measured 0.0145--0.0187 ms per
create/run/join cycle across five 1000-cycle runs, below the 10 ms fixed-overhead
threshold. The full resilience test executable completed after isolation; no
material runtime increase was observable at its reported precision.

No full CTest, complete sanitizer suite, thread race detector, or
external-engine cross-validation was performed for this closure pass.

## Closure rules

1. Keep a finding open until the implementation, public result contract, and a
   focused regression agree.
2. Add the focused module contract before changing an `Uncovered` label.
3. Record units, authored/canonical index space, fallback, approximation,
   time-limit, and unavailable outputs in public results.
4. Move volatile test evidence to `development_status.md` when this audit is
   superseded; do not create dated copies of this file.

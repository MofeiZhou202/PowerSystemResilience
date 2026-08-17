# Module Code Audit

Updated: 2026-08-09

This is the living code-audit ledger for repository modules. It records
source-backed defects and audit coverage; it is not a dated snapshot and does
not replace issue tracking. Runtime behavior remains authoritative in
`include/`, `src/`, `tests/run_gui_server.cpp`, `web/`, and registered tests.
Documentation coverage is tracked separately in the
[module documentation map](module_documentation_map.md).

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

Data-structure review findings R-03, R-05–R-08 (Low, latent foot-guns) are
tracked in [data structure design review](data_structure_design_review.md) §2
and remain open pending the prioritized regression list there. AUD-001 through
AUD-011 remain closed.

Addressed this iteration:

| ID | Finding | Closure | Regression |
|---|---|---|---|
| AUD-012 | DC-side dead islands were never detected or declared (review R-01/R-02): AC-only strip could leave an unsourced or orphaned DC bus, risking a singular `Gdc`. | `detect_dc_dead_buses()` reports unsourced DC buses into `ProjectionCertificate.diagnostics` after strip (honest boundary; full DC removal deferred to a focused change since DC renumbering is owned by `canonicalize_dc_bus_indices`). | `test_component_models_math_audit` "R-01/R-02: DC dead islands are detected..." |
| AUD-013 | Standalone `merge_zero_impedance_buses` ignored `ACBranch::ideal_connectivity` (review R-04), diverging from the projection whitelist path. | The nullptr path now contracts `ideal_connectivity` branches regardless of magnitude, additively, without changing its tested numeric-threshold behavior. | `test_component_models_math_audit` "R-04: standalone merge honors ideal_connectivity..." |

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

#### AUD-008: hybrid and Newton harmonic fallbacks are not disclosed consistently

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
| `graph/` | Focused | Deep | AUD-009 closed; domain-qualified topology and cycle index contracts are tested. |
| `network_reconfiguration/` | Focused | Baseline | No new finding in this pass. |
| `reliability/` | Focused | Sampled | Result-scope and existing sanitizer evidence checked; no new finding recorded. |
| `resilience/` | Distributed | Deep | AUD-011 closed; AUD-009 propagation now exposes AC/DC cut-vertex lists. |
| `analysis/` | Distributed | Sampled | Focused submodule documents exist; no umbrella result contract. |
| `scenario_generation/` | Focused | Deep | AUD-001 and AUD-002 closed. |
| `short_circuit/` | Focused | Baseline | Existing derivation/audit retained. |
| `harmonics_power_flow/` | Distributed | Deep | AUD-008 closed. |
| `dynamics/` | Focused | Baseline | Active runtime contract confirmed; broader design material is archived. |
| `time_series/` | Focused | Baseline | Existing pipeline and annual/lifecycle contracts retained. |
| `carbon_analysis/` | Focused | Deep | AUD-010 closed. |
| `ev_power_traffic/` | Distributed | Deep | AUD-007 closed. |
| `integrated_energy/` | Focused | Deep | AUD-003, AUD-004, and AUD-005 closed. |
| `market/` | Focused | Baseline | AC-only/fallback contract confirmed. |
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
| Harmonics | `test_harmonics_power_flow`: 52 cases, 359 assertions |
| EV Formulation D | `test_ev_power_traffic_joint_opt_d`: 15 cases, 196 assertions |
| Graph/reduction | `test_graph`, `test_graph_kron`, `test_graph_roundtrip`: 55 cases, 461 assertions |
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

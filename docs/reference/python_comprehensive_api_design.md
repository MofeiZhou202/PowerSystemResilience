# Comprehensive Python API Design

Last verified: 2026-09-16

Status: design document. It defines the target surface for one comprehensive
Python API over the whole `hacdcpf` C++ platform. Sections marked **Implemented**
already exist under `python/src/hysim/`; sections marked **Proposed** are the
contract this document commits the implementation to. Runtime behavior is
governed by `include/`, `src/`, `tests/run_gui_server.cpp`, `web/`, and the
registered tests; this document explains the boundary, it does not replace it.

## 1. Decision and scope

Python is the orchestration and intelligence boundary of HySim-XJTU-HRPES. It is
not a second simulation kernel. The authoritative electrical semantics — the
rich `HybridPowerSystem` model, the four-level validation ladder, canonical
projection, every solver family, and authored-space result attribution — remain
in C++ and are reached through the production HTTP routes exposed by
`run_gui_server`.

This document extends the existing dependency-free SDK (`python/src/hysim/`) into
**one comprehensive Python API** that gives typed, safe, honest access to every
production analysis family, not only balanced PF/OPF. The comprehensive surface
is additive: it keeps the isolated-session `/api/v1` runtime and the legacy
process-global routes, and layers a unified, namespaced facade and a single
analysis catalog on top of them.

```text
AI / optimization / data-science / notebook code
        |
        +-- HySim facade (namespaced families: pf, opf, dynamics, market, ...)
        |
        +-- AnalysisCatalog (one registry: name -> route, method, effect, category)
        |
        +-- Session + Job model (isolated /api/v1) | process-global legacy client
        |
        +-- Transport protocol (HTTP now; in-process binding later)
        |
run_gui_server -> hacdcpf C++ facade -> rich/canonical projection -> solvers
```

### Non-goals

1. No electrical, projection, or solver logic is reimplemented in Python. The
   API only builds requests, transports them, and wraps honest results.
2. No silent numeric repair. The API never fills missing result fields with
   zeros, never averages away a non-converged solve, and never downgrades a
   `model_scope` or `model_limitations` string.
3. No automatic retry on non-idempotent calls. Model mutation and analyses are
   not idempotent under the process-global session, so retries are explicit.
4. No provider-specific AI dependencies in the core package. Adapters consume
   the neutral tool schemas.

## 2. Source of truth

| Contract | Anchor |
|---|---|
| Analysis routes | `run_gui_server.cpp` route registrations, e.g. `"/api/session/run_uc"`, `"/api/session/run_market_clearing"`, `"/api/session/run_reliability_nsq"` |
| Isolated runtime | `src/server/` `RuntimeApiV1`; `"/api/v1/sessions"`, `"/api/v1/sessions/{id}/jobs"` |
| C++ facade | `include/hacdcpf/api/hacdcpf.hpp`, `include/hacdcpf/api/solver_capabilities.hpp` |
| Solver entry points | e.g. `solve_unit_commitment`, `run_nonsequential_mc`, `market::clear_day_ahead`, `dynamics::simulate` |
| HTTP/session protocol | [runtime_api.md](runtime_api.md) |
| Existing SDK architecture | [python_api.md](python_api.md) |

Field names, defaults, and result keys in this document are transcribed from the
current handlers. When a handler changes a field name, default, or result key,
this document and the typed request model must be updated in the same change.
Route handlers are referenced by their registered path string, never by line
number, because line numbers drift with the file.

## 3. Layered package contract

| Layer | Public types | Responsibility | State |
|---|---|---|---|
| Runtime | `LocalHySimServer` | One isolated C++ process per experiment or AI worker. | Implemented |
| Transport | `Transport`, `UrllibTransport` | Replaceable request execution; no unsafe automatic retry. | Implemented |
| SDK v1 | `HySimV1Client`, `HySimV1Session`, `HySimJob` | Isolated model lifecycle, optimistic concurrency, asynchronous PF/OPF, typed topology and result chunks. | Implemented |
| SDK legacy | `HySimClient` | Process-global lifecycle; typed PF/OPF; generic `run_analysis`. | Implemented |
| **Catalog** | `AnalysisSpec`, `AnalysisCatalog`, `AnalysisEffect`, `AnalysisCategory` | One authoritative registry of every production analysis: route, HTTP method, effect class, model-mutation flag, category, brief. | **Implemented** |
| **Families** | `PowerFlowApi`, `OptimalPowerFlowApi`, `ReactivePowerApi`, `ShortCircuitApi`, `HarmonicsApi`, `DynamicsApi`, `TimeSeriesApi`, `CarbonApi`, `MarketApi`, `ReliabilityApi`, `ResilienceApi`, `ReconfigurationApi`, `HostingCapacityApi`, `IntegratedEnergyApi`, `EvTrafficApi`, `PlanningApi`, `ScenarioApi`, `ModelIoApi` | Namespaced, typed methods per analysis family, each returning an honest `AnalysisResult`. | **Implemented** |
| **Requests** | `UnitCommitmentRequest`, `MarketClearingRequest`, `ReliabilityRequest`, `TransientRequest`, `RpoRequest`, `ShortCircuitRequest`, `ResilienceRequest`, `CampusIesRequest`, `ScenarioGenerationRequest`, `CimLoadRequest`, `SvgImportRequest`, `HarmonicsStateSpaceRequest`, ... (44 models) | Typed request builders that transcribe the current handler fields; long-tail fields flow through `extra`. | **Implemented** |
| Resources | `TopologyChunk`, `SubgraphView`, `ResultFrameChunk`, `ViolationChunk` | Validate schemas/counts, expose stable bus references, retain raw payloads. | Implemented |
| Models | `BusRef`, `ComponentRef`, PF/OPF requests | Domain-qualified bus IDs and stable authored component IDs. | Implemented |
| Results | `AnalysisResult` | Request/model provenance, scientific status, limitations, raw payload. | Implemented |
| AI | `HySimV1ToolRegistry`, `ToolPolicy` | Job-oriented tool schemas, effect allowlist, explicit mutation approval. | Implemented |

The comprehensive API deliberately preserves unknown response fields. C++
modules evolve quickly, so Python must not discard new diagnostics while waiting
for a package release. Common inputs are typed; every analysis also accepts its
native JSON payload so a newly added server field is reachable before the typed
model is updated.

## 4. Analysis catalog

The catalog is the single source of truth in Python for which analyses exist,
where they live, and how dangerous they are. Every family method resolves its
route through the catalog rather than hard-coding a path string, so the route
table is defined once.

### 4.1 `AnalysisSpec`

```python
@dataclass(frozen=True)
class AnalysisSpec:
    name: str                 # stable Python identifier, e.g. "unit_commitment"
    route: str                # production route, e.g. "/api/session/run_uc"
    method: str               # "POST" or "GET"
    effect: AnalysisEffect    # READ | ANALYZE | MODIFY
    category: AnalysisCategory
    summary: str              # one-line description
```

`AnalysisEffect` classifies risk so the AI tool policy and audit hook can gate
calls without inspecting payloads:

- `READ` — pure query, no solve, no model change (`topology`, `status`).
- `ANALYZE` — runs a solver, does not mutate the authored model.
- `MODIFY` — replaces or edits session state (`load_builtin`,
  `update_components`, `set_ts_config`).

`AnalysisSpec` also carries a narrower `mutates_model` flag: whether the call
replaces or edits the authored system model and therefore advances the model
revision. Every model load has `mutates_model = True`; a session-config write
such as `set_ts_config` has `effect == MODIFY` but `mutates_model == False`, so
it does not advance the model revision.

### 4.2 Catalog coverage by category

Every route below is a production route in `run_gui_server.cpp`. The catalog
registers 69 analyses across 18 families; the "Typed" column marks families with
a dedicated request dataclass. Families marked "pass-through" are callable now
through the generic mapping argument (and reach every server field), and are
candidates for a dedicated dataclass in phase 3.

| Category | Python name → route | Effect | Typed |
|---|---|---|---|
| Power flow | `power_flow` → `/api/session/pf` | ANALYZE | yes |
| Optimal power flow | `optimal_power_flow` → `/api/session/opf` | ANALYZE | yes |
| Optimal power flow | `opf_ac` → `/api/session/opf_ac` | ANALYZE | yes |
| Optimal power flow | `opf_parity`/`opf_dc` → `/api/session/opf_parity`,`opf_dc` | ANALYZE | pass-through (no handler fields) |
| Reactive power | `reactive_power_optimization` → `/api/session/run_rpo` | ANALYZE | yes |
| Reactive power | `rpo_inputs` → `/api/session/rpo_inputs` | READ | pass-through |
| Short circuit | `short_circuit`/`detailed_short_circuit` → `/api/session/sc`,`sc_detailed` | ANALYZE | yes |
| Short circuit | `dc_short_circuit` → `/api/session/dc_sc` | ANALYZE | yes |
| Harmonics | `harmonics`/`three_phase_harmonics`/`harmonics_metrics` | ANALYZE | yes |
| Harmonics | `harmonics_frequency_scan`/`harmonics_newton` | ANALYZE | yes |
| Harmonics | `harmonics_hss` → `/api/session/harmonics_hss` | ANALYZE | yes |
| Dynamics | `transient` → `/api/session/run_transient` | ANALYZE | yes |
| Dynamics | `small_signal` → `/api/session/small_signal` | ANALYZE | yes |
| Time series | `set_ts_config` → `/api/session/set_ts_config` | MODIFY (session) | yes |
| Time series | `unit_commitment` → `/api/session/run_uc` | ANALYZE | yes |
| Time series | `time_series_power_flow`/`annual_production` | ANALYZE | yes |
| Time series | `lifecycle_simulation`/`lifecycle_compare` | ANALYZE | yes |
| Carbon | `carbon_flow` → `/api/session/run_carbon` | ANALYZE | pass-through (no fields) |
| Carbon | `dynamic_carbon_flow` → `/api/session/run_dynamic_carbon` | ANALYZE | yes |
| Reliability | `reliability_nonsequential`/`reliability_sequential` | ANALYZE | yes |
| Reliability | `reliability_fmea`/`reliability_fd`/`reliability_three_stage` | ANALYZE | yes |
| Reliability | `reliability` → `/api/session/run_reliability` | ANALYZE | pass-through |
| Resilience | `distribution_resilience` → `/api/session/run_distribution_resilience` | ANALYZE | yes |
| Market | `market_clearing` → `/api/session/run_market_clearing` | ANALYZE | yes |
| Market | `real_time_market`/`repeated_market_game` | ANALYZE | yes |
| Market | `southern_market` → `/api/session/run_southern_market` | ANALYZE | yes |
| Market | `market_ptdf` → `/api/session/market_ptdf` | READ | yes |
| Integrated energy | `campus_ies` → `/api/session/run_campus_ies` | ANALYZE | yes |
| EV–traffic | `ev_power_traffic` → `/api/session/run_ev_traffic` | ANALYZE | yes |
| Reconfiguration | `reconfiguration` → `/api/session/run_reconfig` | ANALYZE | yes |
| Hosting capacity | `hosting_capacity` → `/api/session/run_hosting_capacity` | ANALYZE | yes |
| Planning | `counterfactual_planning`/`multidimensional_weak_links` | ANALYZE | yes |
| Scenario | `scenario_generation`/`typhoon_faults` | ANALYZE | yes |
| SPPT | `sppt_guard` → `/api/session/sppt_guard` | ANALYZE | pass-through |
| Topology | `topology` → `/api/session/topology` | READ | pass-through |
| Topology | `network_reduction` → `/api/session/network_reduction` | ANALYZE | pass-through |
| Model I/O | `load_*` (builtin, matpower, json, bpa_dat, cim_dist, gridlabd, opendss, etap_xml, svg) | MODIFY | `ModelIoApi.load` + typed requests |
| Model I/O | `export_*` (json, matpower, bpa_dat, etap, etap_xml, cim_dist, gridlabd, opendss, svg) | READ | `ModelIoApi.export` (+ `SvgExportRequest`) |
| Model I/O | `update_components`, `new_empty` | MODIFY | pass-through |

The ETAP XLSX import (`load_etap_xlsx`) and one BPA DAT path take a raw binary
or raw-text body rather than a JSON object, so they are reached through the raw
`HySimClient` methods rather than a typed model; see
[runtime_api.md](runtime_api.md) and
[digital_twin_data_io_architecture.md](digital_twin_data_io_architecture.md).

## 5. Family facade design

The comprehensive facade exposes families as namespaced attributes on the
client, so discovery is local to a domain and stays readable at call sites.
Each family method transcribes the handler's fields into a typed request and
returns an honest `AnalysisResult`.

```python
sim = HySim("http://127.0.0.1:8088")
sim.load_builtin("ieee14_acdc")

pf = sim.pf.run(PowerFlowRequest())
uc = sim.time_series.unit_commitment(UnitCommitmentRequest(num_steps=24))
mkt = sim.market.clearing(MarketClearingRequest(num_steps=24, reserve_fraction=0.06))
rel = sim.reliability.fmea(ReliabilityFmeaRequest(load_scale_factor=1.1))
sc = sim.short_circuit.detailed(ShortCircuitRequest(fault_bus_ids=[3]))
res = sim.resilience.distribution(ResilienceRequest(horizon_hours=8))
ies = sim.integrated_energy.campus(CampusIesRequest(num_steps=24))
sim.model_io.export("matpower")

mkt.require_usable()        # honest gate; raises on failed/stale/invalid
print(mkt.summary())        # compact status; large vectors reported by size
```

`HySim` is the comprehensive process-global facade: it composes `HySimClient`
for transport, model lifecycle, and audit, and attaches one family object per
category (`pf`, `opf`, `reactive_power`, `short_circuit`, `harmonics`,
`dynamics`, `time_series`, `carbon`, `market`, `reliability`, `resilience`,
`reconfiguration`, `hosting_capacity`, `integrated_energy`, `ev_traffic`,
`planning`, `scenario`, `model_io`). The isolated-session path stays available
through `HySimV1Client` for concurrent experiments; a matching `sim.isolated()`
helper returns a v1 client bound to the same transport.

### 5.1 Typed request pattern

Every request dataclass follows the same pattern as the phase-1 models in
`python/src/hysim/models.py`: high-signal handler fields are named parameters,
`None` fields are omitted so the C++ default stays authoritative, and an `extra`
mapping carries the long tail (e.g. the SCUC network-cut knobs, the reliability
data policy, the campus multi-energy time series). Requests live in
`python/src/hysim/requests.py`; a few representative examples:

- `UnitCommitmentRequest` — `num_steps`. Handler `"/api/session/run_uc"`, C++ `solve_unit_commitment`.
- `ShortCircuitRequest` — `fault_type`, `calc_type`, `kappa_method`, `topology`, `c_factor`, `fault_impedance_pu`, `compute_branch_flows`, `compute_voltage_drops`, `compute_ith`, `fault_bus_ids` (detailed route). Handlers `"/api/session/sc"`, `"/api/session/sc_detailed"`.
- `ResilienceRequest` — `apply_demo_data`, `horizon_hours`, `time_step_hr`, `load_scale_factor`, `allow_reconfiguration`, `allow_mess_dispatch`, `model`, `mip_solver`, `mip_gap`, `mip_time_limit_s`; profiles and manual faults via `extra`. Handler `"/api/session/run_distribution_resilience"`.
- `CampusIesRequest` — `solver`, `objective`, `num_steps`, `time_limit_sec`, `mip_gap_tol`, `enable_transport`, `enable_hydrogen_storage_layers`; loads/prices/device sizing via `extra`. Handler `"/api/session/run_campus_ies"`.
- `MarketClearingRequest` — `num_steps`, `offer_segments`, `reserve_fraction`, `voll_per_mwh`, `scuc_mip_relative_gap`, `scuc_time_limit_sec`, `network_constraints`, `run_ac_validation`, ...; remaining SCUC network-cut knobs via `extra`. Handler `"/api/session/run_market_clearing"`.

## 6. Correctness rules

These rules are inherited from [python_api.md](python_api.md) and apply
unchanged to every family in the comprehensive surface:

1. AI-facing bus references are `{domain, index}`. A bare integer is never a
   cross-domain bus identity.
2. `index` means the stable authored component ID, never a C++ vector position
   or graph node index.
3. A successful HTTP response is not a valid engineering result. Consumers
   inspect `scientific_status`, call `require_usable()`, and retain
   `model_scope`, `validity_flags`, fallback state, warnings, and limitations.
4. Each result carries request ID, model revision at submit time, route, and
   analysis name. The audit hook stores a request hash, not the full model.
5. Requests are not retried automatically; model changes and analyses are not
   idempotent under the process-global session.

Result honesty is enforced by the existing `AnalysisResult`: `scientific_status`
returns `failed`/`stale`/`invalid`/`unqualified`/`qualified` from the result's
own `error`, `_result_contract.stale`, and the boolean qualification keys
(`accepted`, `converged`, `success`, `feasible`); `require_usable()` refuses to
pass a result that is failed, stale, invalid, or unqualified.

## 7. AI tool safety

The default policy permits `READ` and `ANALYZE` effects. `MODIFY` (model
replacement or component edits) is disabled unless a deployment opts in, and
each mutating call still requires explicit approval. The catalog's per-analysis
`AnalysisEffect` lets the policy gate a call without parsing the payload, so an
agent cannot reach a `MODIFY` route through a family method while only `ANALYZE`
is allowed. Tool results stay compact: scalar status and diagnostics are kept,
large vectors are represented by their sizes; direct SDK calls retain the full
raw result for NumPy/pandas/PyTorch post-processing.

`HySimToolRegistry.register_family_tools()` opts a deployment into one bounded
tool per catalog analysis (`hysim_<name>`). Each tool inherits the analysis'
effect class, so `MODIFY` family tools are gated by the default policy, and it
returns the same compact honest record as a direct call. Names already curated
(the power-flow, OPF, and SPPT tools) are left untouched.

## 8. Coverage and status matrix

| Capability | Status | Notes |
|---|---|---|
| Isolated sessions, ETags, async PF/OPF jobs | Implemented | `HySimV1Client` |
| Process-global lifecycle + typed PF/OPF | Implemented | `HySimClient` |
| Honest `AnalysisResult`, effect-aware audit | Implemented | `models.py`, `client.py` |
| Scalable topology / frame / violation chunks | Implemented | `resources.py` |
| Analysis catalog (69 production routes) | Implemented | `analyses.py` |
| Namespaced family facade (18 families) | Implemented | `analyses.py`, `HySim` |
| Typed requests: UC, market clearing, reliability, transient, RPO | Implemented | `analyses.py` |
| Typed requests: SC, harmonics, OPF-AC, TS/lifecycle, carbon, reliability FMEA/FD/3-stage, resilience, market RT/repeated/southern/PTDF, IES, EV, reconfig, hosting, planning, scenario | Implemented | `requests.py` |
| Model-I/O catalog + `ModelIoApi` (load/export by format) | Implemented | `analyses.py` |
| Effect-gated per-family AI tools | Implemented | `ai.py` `register_family_tools()` |
| Typed I/O load/export requests + HSS harmonics | Implemented | `requests.py`, `ModelIoApi`, `HarmonicsApi.state_space` |
| ETAP XLSX binary upload (raw body) | Proposed | phase 3 remainder |
| LaTeX module manual | Implemented | `python_comprehensive_api/python_comprehensive_api_manual.tex` |
| In-process pybind11 transport | Proposed | phase 4, only for high-throughput kernels |

Only rows marked Implemented describe current behavior. Proposed rows are the
committed contract; they must not be described as working until their tests are
registered and green.

## 9. Limitations and boundaries

1. `/api/v1` isolated sessions currently accept only balanced aggregate PF/OPF
   jobs. All other families run on the process-global routes; concurrency
   across families therefore requires independent server processes, not
   independent v1 sessions.
2. Results are only as honest as the handler. When a handler returns a
   fallback, time-limit, or reduced `model_scope`, the API surfaces it verbatim;
   it never upgrades scope or hides a limitation.
3. Sessions and job results are not durable across a server restart.
4. Exposure beyond a trusted workstation still requires authentication,
   authorization, request-size limits, and TLS at a gateway; the SDK does not
   add these.
5. The typed request models cover the fields named in the current handlers. A
   newly added server field is reachable through the `extra` pass-through until
   it is promoted to a named parameter.

## 10. Implementation phases

| Phase | Deliverable | Test gate | State |
|---|---|---|---|
| 1 | Catalog, effect/category enums, family facade, typed UC/market/reliability/transient/RPO requests, `HySim` composition | `python/tests/test_analyses.py` with `FakeTransport` | Done |
| 2 | Typed requests for the remaining ANALYZE families, model-I/O catalog + `ModelIoApi`, effect-gated per-family AI tools | `test_analyses.py` route/effect/mutation assertions | Done |
| 3 | Deep typing of I/O payloads (CIM, OpenDSS, SVG, BPA, MATPOWER, JSON), HSS harmonics, LaTeX manual | I/O route/serialization tests in `test_analyses.py` | Done, except ETAP XLSX raw-binary upload |
| 4 | ETAP XLSX binary upload wrapper; optional in-process transport | throughput benchmark, parity vs HTTP | Planned |

Phases 1–2 are dependency-free and testable without a running solver, exactly
like the existing `test_client.py` and `test_v1.py`. The family facade and
catalog are validated with a `FakeTransport` that asserts the resolved route,
HTTP method, model-mutation effect, and serialized payload for each typed
request. The LaTeX module manual is at
[python_comprehensive_api/python_comprehensive_api_manual.tex](python_comprehensive_api/python_comprehensive_api_manual.tex),
compiled like the module manuals with the shared `_manual_common` style.

## 11. Cross-references

- Existing SDK architecture and AI policy: [python_api.md](python_api.md)
- HTTP and session protocol: [runtime_api.md](runtime_api.md)
- C++ public facade manual: [../modules/api/api_manual.tex](../modules/api/api_manual.tex)
- Server runtime contract: [../modules/server/](../modules/server/)
- Data I/O architecture: [digital_twin_data_io_architecture.md](digital_twin_data_io_architecture.md)

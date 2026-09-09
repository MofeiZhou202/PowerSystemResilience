# Runtime API Contract

Updated: 2026-08-21

The GUI server is implemented in `tests/run_gui_server.cpp`. Session endpoints
operate on one loaded `HybridPowerSystem`; a model-changing request clears
cached analysis results.

## Static documentation mount

`GET /xjtu/docs/<relative path>` serves markdown files from the repository
`docs/` tree read-only (for example `/xjtu/docs/README.md`,
`/xjtu/docs/overview/case_catalog.md`). Path traversal outside `docs/` is
rejected with 404. The GUI documentation help center
(`web/js/core/help_center.js`, manifest `web/help_docs.json` with schema
`hysim_help_docs_v1`) renders these documents in-app; see
[GUI Canvas runtime](../developer/gui_canvas_runtime.md) for the front-end
contract.

## Version 1 multi-session API

New automation and AI clients should use `/api/v1`. These resources are
independent of the GUI's process-global `/api/session/*` state.

| Method and route | Contract |
|---|---|
| `GET /api/v1` | Capability and bounded-resource declaration. |
| `POST /api/v1/sessions` | Create an empty session or load `case`, `model`, or `json_string`; returns `201`, `Location`, and `ETag`. |
| `GET /api/v1/sessions` | List in-memory sessions. |
| `GET /api/v1/sessions/{session_id}` | Read model summary and current revision. Supports `If-None-Match`. |
| `GET /api/v1/sessions/{session_id}/model` | Read the authored rich model and its ETag. |
| `GET /api/v1/sessions/{session_id}/topology` | Read a paged LOD0/1/2 topology chunk; accepts `offset`, `limit`, and a complete `xmin,ymin,xmax,ymax` viewport. |
| `GET /api/v1/sessions/{session_id}/subgraph` | Read a bounded k-hop subgraph around the required domain-qualified `domain,index` bus reference. |
| `PUT /api/v1/sessions/{session_id}/model` | Atomically replace the model and increment revision. Requires the current `If-Match`. |
| `DELETE /api/v1/sessions/{session_id}` | Delete a session with no active jobs. Requires `If-Match`. |
| `POST /api/v1/sessions/{session_id}/jobs` | Snapshot the matched model revision and enqueue PF or OPF; returns `202` and `Location`. |
| `GET /api/v1/sessions/{session_id}/jobs` | List that session's retained jobs without result arrays. |
| `GET /api/v1/jobs/{job_id}` | Poll state and retrieve the terminal result. |
| `GET /api/v1/jobs/{job_id}/frames/{step}` | Read one time step with domain, stable-index, viewport, offset, and limit filters. Static PF/OPF has step 0. |
| `GET /api/v1/jobs/{job_id}/violations` | Read the worst voltage/loading violations for one step without downloading the full frame. |
| `POST /api/v1/jobs/{job_id}/cancel` | Cancel queued work immediately or mark running work as cancelling. |
| `DELETE /api/v1/jobs/{job_id}` | Remove a retained terminal job. |

The model ETag is a strong opaque value such as
`"hysim-ses-...-r3"`. Missing `If-Match` returns HTTP 428; a stale value
returns HTTP 412 with `current_etag` and `current_model_revision`. A submitted
job owns an immutable model snapshot, so a later model replacement cannot alter
the solve. Polling reports `stale_against_current_model`; the nested
`hysim_result_v1` contract repeats the job and current revisions.

The bounded in-memory worker pool defaults to two threads and is configurable
with `--api-job-workers 1..32`. The process retains at most 128 sessions and
4096 jobs. Terminal jobs should be deleted by the client. Storage is not durable
across server restarts.

Version 1 currently accepts `power_flow` with `method=ac_newton` and
`optimal_power_flow` with `network_model=balanced_aggregate` using parity,
Ipopt, auto, economic dispatch, or DC. Other production analyses remain on the
legacy routes until their result contracts are migrated.

Parity/auto OPF accepts bounded Phase-I controls under `request.options`:
`enable_phase_one`, time/iteration/factorization/backtrack budgets, barrier,
admission, primal-corridor, and centrality parameters. Large pure-AC dispatch
starts additionally accept `ac_pf_warm_start`, `ac_pf_dc_phase_one`, minimum
bus count, an end-to-end `ac_pf_dc_phase_one_time_limit_ms` wall budget, DC
iteration/residual limits, required dual improvement, and the baseline-dual
trigger. The budget includes projection, formulation, structural initialization,
symbolic analysis, and numeric iterations; one in-flight sparse factorization
may overshoot it. Values `<= 0` disable the wall limit. The response returns
`ipm_profiling` with the effective
baseline/candidate primal-dual residuals, budget use, admission status, and
Phase-II acceptance. `parity_formulation_builds=1` certifies that baseline and
candidate evaluation reused one immutable formulation, while
`dc_phase_one_symbolic_analyze_calls` reports the separate DC KKT analysis.
The default-off research controls `phase_one_dispatch_dual_predictor` and
`phase_one_dispatch_dual_min_improvement` enable a zero-factorization
component-price candidate. Its attempted/accepted/runtime/status and raw plus
normalized residual pairs are returned as `dispatch_dual_predictor_*`; a
rejected candidate does not alter the Phase-II start.
The same profiling object also carries the prepared-session contract:
`prepared_session_used`, `formulation_reused`, `mapping_reused`,
`symbolic_reused`, `continuation_state_reused`, `numeric_refactor_*`, and
`prepared_session_invalidation_reason`. The HTTP endpoint currently performs
standalone solves, so `prepared_session_used=false`; the owning repeated-solve
API is `opf::PreparedACOPFSession` in C++. These fields are still returned so a
future session-aware HTTP scheduler can adopt the contract without changing
the response schema. Numeric-factor reuse fails closed: the current backends
recompute numeric factors after a reusable symbolic ordering and report an
explicit `unsupported` status when the experimental request is enabled.
Non-finite inactive residuals are JSON `null`. A DC
iteration-limit point is returned only as `phase_one_warm_start_only=true`; it
does not set `converged=true` or claim DCOPF optimality.

Queued cancellation is immediate. A running PF/OPF solver currently has no
cooperative cancellation token; its state becomes `cancelling`, its eventual
result is discarded, and its terminal state becomes `cancelled`.

Topology LOD2 nodes expose stable `ref: {domain,index}` values. Edge references
also carry `resource` so a branch, transformer, and converter cannot collide.
LOD1 aggregates by domain/area/zone; LOD0 aggregates by electrical domain.
Spatial coordinates are an API layout space, not graph indices or engineering
identity. Pagination is applied to nodes and only edges whose endpoints are in
the returned node chunk are included.

Frame chunks return authored-space result rows. `indices=1,7,20` always means
stable component indices in the selected domain. A viewport filter is available
while the matching session revision still exists. The violation endpoint keeps
the largest engineering excursions first so overview clients can inspect the
worst state before requesting dense detail.

## Session lifecycle

| Method and route | Purpose |
|---|---|
| `POST /api/session/new_empty` | Create an empty system. |
| `POST /api/session/load_builtin` | Load a built-in case. |
| `POST /api/session/load_json_string` | Load authored rich-model JSON. |
| `POST /api/session/load_matpower` | Load a MATPOWER case. |
| `POST /api/session/update_components` | Apply component changes. |
| `GET /api/session/status` | Read current session state. |
| `POST /api/session/cancel` | Request cancellation of a long analysis. |

The session holds the current system as a read-shared immutable snapshot
(`std::shared_ptr<const HybridPowerSystem>`); model-changing routes replace it
atomically (copy-on-write) via `session_replace_system`, which rebuilds the
resident `PowerSystemGraph`, the bus spatial index, and the compact `_raw_json`
serialization cache once per write and increments `system_revision`.
`cache_last_power_flow` (called under the session lock) publishes only if the
request's captured immutable system is still `current_system`, then records
`last_pf_revision`. A PF finishing after model replacement may return its old
snapshot to its original caller, but cannot repopulate the session cache. The
window route returns `409/no_cached_power_flow` until a current PF publishes.
`/api/session/result_window` compares the two revisions to
declare `result_matches_current_system`. `last_pf_result`/`last_pf_system` are
shared snapshots, so result-serving routes read them with no copy.
Load-path responses (`load_*`, `update_components`, parameter-library and
design-handbook apply) embed `_raw_json` as the cached compact (un-indented)
serialization; the frontend parses it with `JSON.parse`.

All 45 legacy analysis handlers that acquire `Session::busy` use an
`AtomicFlagLease`: acquisition is compare-and-exchange, and cleanup releases
only the acquiring request's lease. Malformed JSON, pre-acquisition exceptions,
and `409` conflicts cannot clear another task's `busy` or reset its `cancel`.
The global exception handler only formats the error; stack unwinding releases
an owned lease. `/api/session/status.busy` and `.cancel` remain booleans with
the existing frontend polling/task-manager contract; no response keys change.
The registered `session_integrity_e2e` exercises rejected requests, cancellation,
model replacement during PF, and successful publication of the next current PF.

## Topology window

| Method and route | Contract |
|---|---|
| `POST /api/session/topology_window` | Bounding-box query over the resident uniform-grid spatial index. Body: required numeric `min_x,min_y,max_x,max_y` (WGS84 degrees; x=longitude, y=latitude; min<=max enforced) and optional `lod` (0=aggregate by electrical domain, 1=aggregate by domain/area/zone, 2=full per-bus detail, default 2; semantics mirror `web/js/core/network_overview.js` `aggregate()`). Response `topology_window_v1`: `nodes` carry domain-qualified stable `.index` (lod 2) or group `key`/centroid/`count` (lod 0/1); `edges` are included only when both endpoints are inside the window and carry `category` plus stable `index`. `coordinate_coverage` declares how many AC/DC buses lack coordinates (model-default lat/lon 0,0); such buses are excluded from window results and the exclusion is repeated in `model_limitations` rather than silently returning an empty view. The response never embeds the full component parameter set. |
| `POST /api/session/result_window` | Viewport-scoped per-element power-flow results from the cached last PF (`last_pf_result` + `last_pf_system`). Same bbox body as `topology_window`, plus the same optional `lod` (default 2; 0/1 rejected values return 400). Returns `409` with `error=no_cached_power_flow` when no PF result is cached — never a silent empty view. Response `result_window_v1` (extended with `lod` and the aggregated node/branch shapes below): at lod 2, `nodes` = in-window geo-referenced buses `{domain,index,x,y,vm_pu,va_rad?}` (`vm_pu` in pu; `va_rad` in radians, AC only; DC buses carry `vm_pu` = vdc in pu); `branches` = AC branches with both endpoints in-window `{domain,index,from,to,in_service,pf_mw,qf_mvar,pt_mw,qt_mvar,loss_mw,loading_pct,rate_mva}` (MW/MVAr; `loading_pct = 100 * max(|S_from|,|S_to|)/rate_a_mva`, same as the full PF `geo_ac_branches`; DC branch flows and converter transfers are excluded and declared). At lod 0/1 the grouping keys, centroid coordinates, and intra-group edge collapse are identical to `topology_window`: `nodes` = `{group,domain,x,y,count,vm_avg,vm_min,vm_max}` where the vm fields are statistics over member bus voltages (not a solved group voltage), and `branches` = group-pair aggregate edges `{key,source,target,domain:"AC",kind:"aggregate",count,loading_pct,in_service}` whose `loading_pct` is the **maximum** among the collapsed members (conservative); both semantics are declared in `units` (`vm_avg`/`vm_min`/`vm_max`/`aggregated_loading_pct`) and `model_limitations`. `result_meta` reports `source`, `method`, `converged`, `iterations`, `residual`, and `result_matches_current_system` — when the model was replaced or edited after the solve, the flag is `false` and `model_limitations` declares the lag. `units`, `bbox`, and `coordinate_coverage` mirror `topology_window_v1`. |

The GUI calls `syncToBackend()` before analyses when the authored Canvas model
is dirty. Result playback never synchronizes or changes the model.

## Graph analysis and network reduction

| Method and route | Contract |
|---|---|
| `POST /api/session/topology` | Analyzes the resident `PowerSystemGraph`; the body is ignored. Returns connectivity/radiality/cycle counts, AC-primary islands, authored-position island/cut masks, domain-qualified `cut_vertices`, per-endpoint-domain bridges, and diagnostics. `n_buses/n_branches` are graph storage counts and may include out-of-service objects. Flat `cut_vertex_bus_ids` and diagnostic bus IDs are compatibility output and are ambiguous for same-number AC/DC buses. |
| `POST /api/session/network_reduction` | Request booleans default to switch contraction=true, one-pass series=true, pendant=false, Kron=false, and zero-impedance-line contraction=false. Executes contraction -> one series plan -> optional one pendant plan. Kron candidates are counted only; no Kron operation changes `after` or `reduced_system`. Returns live before/after counts, domain-qualified bus representative rows, operation records/fidelity strings, diagnostics, and a compact reloadable `hacdcpf_system_json_v1`. Compacting filters dangling rich assets and clears three-phase/projection/telemetry state, so reloadability is not a semantic-equivalence certificate. |

Both routes return 409 when another session analysis owns the busy flag and
400 for no loaded system, malformed JSON, or an exception. They do not run
PF/OPF or recover eliminated voltages/branch flows. The HTTP default for
zero-impedance line contraction is deliberately false, unlike the public C++
`ContractionOptions` default of true. The full contract and open audit issues
are in the [graph manual](../modules/graph/graph_manual.tex).

数值交叉验证索引：稠密/稀疏 Kron 的 boundary-current 与 recovery 误差、闭合开关和
series PF round-trip、case33bw split-series 以及 DC-OPF objective round-trip 见
[graph 数值章节](../modules/graph/chapters/numerical_cross_validation.tex)。这些是
库/测试层证据；两个 HTTP 路由本身仍不执行 PF/OPF，也不生成被消去电压恢复证书。

## Primary analyses

| Route | Result family |
|---|---|
| `POST /api/session/pf` | Authored-space PF result with rich component attribution and P/Q diagnostics. `method=three_phase_hybrid` selects the monolithic unbalanced abc/DC/VSC Newton path. |
| `POST /api/session/opf` | OPF plus authored-space post-PF Canvas payload. `network_model=three_phase_hybrid` selects Full/GraphReduced phase-domain hybrid OPF and same-model PF replay. |
| `POST /api/session/run_ts_pf` | UC/OPF/PF time-series summary and cached per-step results. UC certificates are exposed as `uc_solver_status`, `uc_mip_gap`, `uc_mip_gap_target_met`, and `uc_optimality_proven`; feasibility alone is not an optimality claim. |
| `POST /api/session/run_annual_sim` | Annual production simulation and aggregate statistics. Response includes `model_scope`, `schedule_only`, `physical_replay_complete`, `ens_complete`, budget residuals, SOC boundary residual and bottom-up feedback status; schedule-only is not a full-year AC physical certificate. |
| `POST /api/session/run_carbon` | Static carbon flow. |
| `POST /api/session/run_dynamic_carbon` | Dynamic carbon flow using cached or newly solved TSPF states. |
| `POST /api/session/run_transient` | Phasor-domain transient result and cached telemetry. |
| `POST /api/session/small_signal` | Modal analysis at the initialized dynamic operating point. |
| `POST /api/session/sc`, `dc_sc`, `sc_detailed` | AC/DC short-circuit analyses. |
| `POST /api/session/harmonics*` | Harmonic PF, three-phase, frequency scan, metrics, and Newton variants. |
| `POST /api/session/harmonics_hss` | Frequency-coupled AC/DC HSS with explicit complex injections/couplings and first-class converter/passive-device attribution. |

`POST /api/session/harmonics` 的 AC 支路谱以 canonical `branch_index` 标识，并同时返回
`i_pu`（from 端）、`i_to_pu`（to 端）和 `i_series_pu`（铜耗串联元件）以及 `thd_i_pct`。
带变压器/开关来源的支路身份通过投影映射附加到响应；不能把数组位置当作 authored 组件 ID。
`POST /api/session/harmonics_freqscan` 返回与 `freqs` 等长的 `frequency_solved`；任一频点失败时
顶层 `ok=false`，失败点阻抗为 JSON `null`（C++ 中为 NaN），不得解释为零阻抗反共振。
| `POST /api/session/run_reliability*` | Non-sequential, sequential, FMEA, feeder, and three-stage reliability. |
| `POST /api/session/run_reconfig` | Topology reconfiguration. |

`POST /api/session/run_transient` accepts the MassMatrixDae IEEE 1547 event
controls `enable_der_protection`, `localize_der_protection_events`,
`protection_event_time_tol_s`, `protection_event_max_localization_iters`, and
`protection_event_cluster_window_s`, and `post_event_algebraic_residual_tol`.
The cluster window is anchored at the earliest physical action and does not
roll forward. Its result reports localization use/trials, event-cluster count,
maximum bracket width and cluster span in seconds, and maximum post-event
algebraic current-balance infinity norm in pu. Protection event records include
zero-based `protection_cluster_id`; non-protection events use `-1`. The exact numerical scope and
unsupported relay/within-step pulse cases are defined in the
[transient runtime contract](../theory/transient_runtime.md).
The route also accepts and echoes `algebraic_network_max_iters` (default 10)
and `algebraic_network_tol` (default `1e-6`) in `options`. During
consistent initialization only, the implementation may temporarily use a
tighter internal algebraic tolerance as defined by the transient contract; the
echo remains the requested time-stepping value.

The three short-circuit routes have deliberately different result scopes:

| Route | Effective contract |
|---|---|
| `POST /api/session/sc` | All-authored-AC-bus overview. Returns `bus_id`, `ikpp_ka`, `sk_mva`, and complex Thevenin impedance parts with `model_scope=overview-positive-sequence`. For unbalanced faults this is the simplified `Z1=Z2=Z0` overview, not the detailed sequence solve. |
| `POST /api/session/sc_detailed` | Requires non-empty authored AC `fault_bus_ids`; defaults Method-B topology to independently evaluated `Auto`, and forces `compute_nonfault_currents=false` while preserving complete selected-fault duties, phase currents, source contributions, voltage profile, and authored branch rows. Each result carries status/message/scope/limitations, effective `c_factor`, and numerical quality; the top level separates requested options and aggregate complete/partial/failed counts. Contracted ideal switchgear has an identity row with `electrical_value_available=false`. |
| `POST /api/session/dc_sc` | Requires non-empty authored DC `fault_bus_ids`; one sparse batch reports per-item status/scope/limitations, `v_prefault_pu`, `r_thevenin_pu`, pu/kA fault current, residual, graph counts, and actual resistive-edge DCCB duties. HTTP 200 may contain per-item `solved=false`; the top-level batch status must also be checked. |

All three routes return 409 for session contention and 400 for parsing/raised
errors. Public solver validation rejects non-finite and out-of-domain numeric
options before assembly. Required sparse factors and selected solves must pass a
`1e-9` backward-residual gate before detailed AC can report success. The complete
option/result, identity, numerical validation, and closed-audit contract is in the
[short-circuit manual](../modules/short_circuit/short_circuit_manual.tex).

`run_reconfig` solves one snapshot even though the response exposes a
one-element `steps_topology` view. `milp_feasible` certifies only the core
topology/LinDistFlow model; `executable` additionally requires rich-device
action mapping, switching-sequence validation, reprojection, PF convergence,
and AC-OPF convergence. Clients must not collapse these states into one
success flag. Hybrid branches are domain-qualified in `dc_branch_details`,
`vsc_details`, and `switch_operations`; the bare `open_branch_ids` and
`closed_branch_ids` arrays are AC-only compatibility output.

The `estimated_loss_mw` response is the core's explicitly-defined
nominal-current loss proxy (`reconf_loss_mw`, MW), not the old dimensionally
invalid `milp_objective * base_mva` (AUD-018, fixed). It is a topology
comparison proxy, not a measured loss: for physical loss use
`reconfig_loss_mw` only when `reconfig_pf_converged=true`. The complete
request/result and validity contract is in the
[network-reconfiguration manual](../modules/network_reconfiguration/network_reconfiguration_manual.tex).

6-bus 穷举、IEEE 33-bus 独立 BFS、Newton PF/DC-OPF/碳流流水线的原始数值和验收门槛见
[网络重构数值章节](../modules/network_reconfiguration/chapters/numerical_cross_validation.tex)。
其中 BFS 与 Newton PF 是两套不同模型输出，MILP objective 和 HTTP
`estimated_loss_mw`（名义电流代理）均不得作为物理损耗交叉替代。

Power-flow numerical overrides are sparse intent. In particular, omitting
`options.robust_nonlinear.enable_auto_fallback_scheduling` preserves the C++
backend default; the GUI's `后端默认` selection deliberately omits that key.
An explicit `true` or `false` overrides it for the request. Every PF response
returns the resolved value under
`options_effective.robust_nonlinear.enable_auto_fallback_scheduling`; clients
must display that effective value instead of inferring it from an unchecked or
missing control.

Balanced Newton PF accepts `enable_pv_pq_conversion` and the bounded
`pv_pq_max_outer_iterations` control. The response `reactive_limits` object
reports whether generator Q-limit enforcement was requested and certified,
whether an active-set signature repeated, whether the outer budget was
exhausted, switch counts, active limited buses, and maximum violation in pu.
The corresponding validity flags are
`generator_reactive_limits_enforced` and
`generator_reactive_limits_certified`. The GUI default is fast screening with
conversion disabled; a converged screening result must not be interpreted as a
Q-limit-certified engineering result. The switching and smooth-NCP theory is
specified in [the PV/PQ contract](../theory/pv_pq_switching_contract.md).

Balanced Newton PF also accepts the following research-grade numerical policy
under `options.robust_nonlinear` on both `/api/session/pf` and `/api/v1`:

| Field | Type and effective domain | Meaning |
|---|---|---|
| `enable_smooth_ncp` | boolean | Use smooth FB/CHKS continuation without changing the fixed equation layout. |
| `ncp_mu0`, `ncp_mu_min` | finite, nonnegative; `mu0 >= mu_min` | Initial and terminal smoothing levels. |
| `ncp_mu_factor`, `ncp_mu_factor_coarse` | finite values strictly between 0 and 1 | Fine- and coarse-phase multiplicative reductions. |
| `ncp_mu_phase_transition` | finite value in `(0,1]` | Relative residual threshold between the coarse and fine schedules. |
| `enable_vsc_local_schur` | boolean | Admit exact elimination of supported fixed six-state VSC local blocks. |
| `vsc_schur_min_network_dimension` | integer, normalized to at least zero | Production scale crossover below which complete sparse LU is retained. |
| `vsc_schur_local_rcond_tolerance` | finite value in `[0,1]` | Minimum accepted reciprocal condition estimate for every local block. |
| `vsc_schur_backward_error_tolerance` | finite value in `[0,1]` | Maximum reduced and reconstructed full-system normwise backward error. |

Invalid dimensionless policy values restore the named C++ defaults. Machine
epsilon is a read-only representation constant and is not an API parameter.
The response returns the normalized policy under
`options_effective.robust_nonlinear`. `linear_structure` separately reports
whether Schur was actually attempted and accepted, its dimensions and
structural nonzeros, rejection/fallback counts, local `rcond`, backward errors,
and sampled semismooth rates. An enabled option is not evidence that the path
was admitted. These runtime values certify the selected generalized-Jacobian
element assembled during the solve, not every element of the B-subdifferential.

Specialized production routes for resilience, hosting capacity, campus IES,
EV traffic, lifecycle, scenario generation, and SPPT remain discoverable in
the server source. Legacy embedded-UI routes are not part of this contract.

## Reliability and protection configuration

The process-global GUI session exposes a model-bound reliability configuration:

| Method and route | Contract |
|---|---|
| `GET /api/session/reliability/configuration` | Return schema, sparse saved overrides, effective failure modes, stable component inventory, coverage, support diagnostics, and validation. |
| `POST /api/session/reliability/configuration/validate` | Parse and validate a candidate without changing session state. |
| `POST /api/session/reliability/configuration` | Validate, resolve protection-zone references, save atomically, and clear cached analyses. |

`POST /api/session/run_reliability` accepts a top-level `failure_rate_basis`:
`operating_time` is the conditional up-time intensity and `calendar_time` is
the observed calendar frequency. The latter is inverted exactly using
`U=f_cal*r/H` and `lambda_up=f_cal/(1-U)`; invalid basis values and `U>=1` are
rejected. The selected basis is echoed in every response.

`method=physical_cut_set` accepts `physical_cut_set.components[]` rows with a
unique `stable_id` and availability in `[0,1]`, plus `success_paths[][]` using
zero-based indices into that component array. It performs exact inclusion-
exclusion on reduced success paths and returns availability, loss probability,
reduced path count, minimal cuts in both stable-ID and index spaces, and
`exact_independent_path_model=true`. Invalid/oversized inputs fail explicitly;
the endpoint never substitutes a path-independence approximation.

Three-stage requests accept an even
`restoration.apparent_power_polygon_sides >= 4` (default 16). Responses echo
the side count, maximum AC branch apparent-power/rating ratio, and
`validity.apparent_power_polygon_enforced`.
Each fault also returns the backend `stage1/2/3_solver_status`, certified
`stage1/2/3_mip_gap`, and raw `stage1/2/3_solver_reported_mip_gap`. An explicit
optimal termination certificate normalizes the certified gap to zero when a
zero or near-zero objective makes the backend relative gap ill-scaled; the raw
diagnostic is retained and all post-solve feasibility audits remain mandatory.

Failure-mode overrides are keyed by `mode_id`; protection references use
`component_kind + component .index` stable IDs such as `ac_transformer_2w:7`.
Vector position is returned only as `component_position` for diagnostics and is
not an external identity. Duplicate `.index` values within one component kind
are rejected because they would make the stable mapping ambiguous.

Every `effective_modes[]` row distinguishes
`effective_failure_rate_per_year` (conditional up-time intensity) from
`effective_calendar_frequency_per_year` (calendar event frequency used for
annual consequence weighting). Calendar-input protection events expose both.

The mode schema exposes optional `enabled`, hazard (`failure_rate_per_year`,
`mtbf_hours`, `forced_outage_rate`), repair/duration, conditional/demand
probability, cyber recovery, and residual-capacity fields. At most one hazard
form may be supplied. Effective precedence is user mode override, user
protection configuration, authored case data, built-in component template,
then the selected missing-data policy.

Every `effective_modes[]` row also returns `resolved_parameters` with the same
12 numeric field names used by the writable schema. These values are the
canonical/equivalent result after applying that precedence: resolved annual
frequency, equivalent MTTF/MTBF, repair duration, steady unavailability,
conditional and demand probabilities, stage durations, cyber recovery, and
residual capacity. They are display values, not saved overrides. The GUI marks
them as inherited and posts only fields the user actually changes; choosing a
different hazard representation replaces the prior hazard override instead of
submitting mutually exclusive forms together.

Protection rows map one protective device to a protected component, optional
backup device, and explicit protection-zone component IDs. Fail-to-trip and
fail-to-open probabilities, nuisance-trip frequency, clearing times, automatic
reclose, and reclose-success probability round trip through the same schema.
Failure-mode FMEA consumes device failure probabilities, nuisance trips, and
zone mutations. Three-stage restoration consumes protection rows, configured
clearing times, automatic-reclose probability, primary/backup dependability,
and backup-zone expansion; it does not consume `mode_overrides`.

For a physical initiating frequency `lambda`, reclose-success probability
`r`, and primary/backup failure probabilities
`q=1-(1-p_fail_to_trip)*(1-p_fail_to_open)`, the three-stage route generates
mutually exclusive sustained scenarios:

```text
lambda_transient  = lambda*r
lambda_primary    = lambda*(1-r)*(1-q1)
lambda_backup     = lambda*(1-r)*q1*(1-q2)
lambda_unresolved = lambda*(1-r)*q1*q2
```

Successful reclose is reported as a momentary frequency and excluded from the
sustained IEEE 1366 SAIFI/SAIDI/EENS aggregation. Primary and backup scenarios
use their configured clearing times. Backup clearance removes every supported
stable component in `zone_component_ids`; unresolved primary-plus-backup
failure blocks restoration until clearance. A matching protection row for the
backup device supplies `q2`; otherwise the response declares the assumed
upstream-backup boundary. Initiating frequency is conserved between transient
and sustained scenarios.

三阶段 HTTP 路由本身是概率条件化的恢复模型；可靠性主路由另提供
`method=protection_cyber_compare`。当
`protection_cyber.online_dae=false` 时，它消费请求给出的主/后备轨迹；为 true
时，每个场景执行主、后备各自的故障发现与动作反馈 Mass-Matrix DAE，并把
CT/PT 后的轨迹、支路跳闸、失电岛负荷退出和 DER-FRT 终态送入同一年度事件树。
两种模式都返回静态 FMEA、仅保护、信息物理联合 EENS/LOLE/LOLF。在线模式
额外返回 `dae_trajectories_consumed_by_event_tree` 和逐场景
`online_diagnostics`，用于核对 DAE 与事件树清除时刻、轨迹点数、动作反馈、
失电母线稳定 ID 和 FRT 消费状态。当前在线结果口径为正序网络、单相故障输入
和三窗口年度后果；LCC 与显式三相保护动态不在该口径内。

Saved overrides are sparse. The GUI does not turn every effective built-in row
into an explicit user value. Loading a different built-in or imported model, or
replacing component arrays through `/api/session/update_components`, clears the
configuration. `/api/session/load_json_string` also clears it unless
the caller sets `preserve_reliability_configuration: true`; the GUI uses that
flag only for a same-canvas synchronization, after which stable references are
validated again before reliability execution. Saving or importing a non-empty
custom configuration makes the GUI select `failure_mode_fmea`, because it is
the only method that consumes failure-mode IDs; users may then explicitly
select three-stage restoration to apply the protection rows only.

Every `POST /api/session/run_reliability` response includes a
`reliability_configuration` audit object. `configured` reports whether the
session has sparse mode/protection rows. Failure-mode FMEA consumes mode and
protection overrides. Three-stage restoration reports `applied: true` and
`applied_scope: ["protection"]` only when at least one enabled protection row
matches an enumerated contingency; otherwise its scope is empty and its
limitation explains unmatched rows or ignored mode overrides. Other methods
return `applied: false` plus a non-empty `limitation` when saved configuration
exists; they never silently imply that custom values affected the run.

The <code>fd</code> response returns exact, unbinned capacity-outage levels
together with state probabilities, cumulative probabilities/frequencies,
probability/frequency validity, exact-capacity-state status, and warnings. FOR
without MTTR remains usable for probability/LOLE but does not fabricate LOLF
or LOLD. Failure-mode N-2 responses distinguish each pair's raw ranking
contribution from its EENS/LOLE interaction corrections; aggregate fields
report first-order EENS, second-order interaction EENS, skipped-pair counts,
and second-order expansion completeness.

`coverage.modes_unsupported` is derived from the same consequence-patch
decision returned as `effective_modes[].supported`; it is not a catalog-only
estimate. For `dist33_microgrid_der` the current catalog has 228 modes, of
which 54 have no consequence representation: 33 AC-bus, 2 DC-bus, 14
AC-switch, 3 flexible-load, and 2 VSC measurement/control modes. Their editable
reliability parameters remain available, but the response carries a specific
`unsupported_reason` rather than treating their impact as zero.

The GUI workflow distinguishes a completed approximation from a failed solve.
A non-empty `model_limitations` makes consequence mapping visibly limited even
when metrics were calculated. False validity fields whose names express
validity, convergence, enforcement, or solver admission make solving/recovery
limited. A transport error or JSON `error` is a failed stage instead.

For `dist33_microgrid_der`, ordinary FMEA currently completes with
`physical_model=hybrid_network_lp` and `model_scope=hybrid-acdc-network-lp`.
It includes active-power AC/DC restoration, but does not certify nonlinear AC
voltage/reactive feasibility and does not search DC-side repair switching.
Three-stage evaluation uses
`model_scope=coupled-acdc-lindistflow-restoration-milp` and returns `ok=true`
with restoration-MILP, branch-flow, voltage, radial-topology, DC-power-flow,
and VSC/SOP-dispatch validity flags true. Fault rows return signed VSC dispatch
for every stage. The pure-AC `dist33_tie_demo` remains the corresponding
AC-only certified case. Both three-stage HTTP routes execute the solver on a
dedicated large-stack worker so a solver stack requirement cannot terminate an
HTTP worker process.

## Lazy Canvas frames

Large analyses cache solver results server-side and expose one frame at a time:

```http
GET /api/session/tspf/frame?step=12
GET /api/session/transient/frame?index=120
```

Both return `schema: "canvas_frame_v1"`, `analysis`, `index`, `count`, `time`,
`time_unit`, `converged`, `capabilities`, and authored-space component data.
Missing caches return HTTP 400. Out-of-range indices return HTTP 416.

TSPF frames may contain bus voltages, AC/DC branch flows, switch and circuit
breaker terminal P/Q, Grid/generator/storage dispatch, VSC/DC-DC transfers,
SOC, transformer terminal flows, and bus balance diagnostics.

Transient frames contain only recorded dynamic quantities: bus voltage,
frequency, device metrics, and applied event state. Their
`capabilities.branch_power` is false and branch/CB flow arrays are empty unless
the dynamic solver is later extended to record real terminal current or P/Q.

## Error and cache rules

- Heavy analyses return HTTP 409 when another analysis owns the session.
- Invalid input and missing prerequisites return HTTP 400 with `error`.
- Frame responses use `Cache-Control: no-store`; the server-side session cache
  is the source of truth.
- Loading or changing a model invalidates PF, TSPF, and transient caches.
- A consumer must check `converged`/`success` and capability flags before
  interpreting optional result arrays.

## Verification

```bash
python3 tools/gui_api_e2e.py \
  --server build/tests/run_gui_server \
  --data-dir data --skip-etap
```

The E2E suite covers PF/OPF Canvas reprojection, Grid and CB P/Q, TSPF frame
retrieval, P/Q diagnostics, and the transient no-fabricated-flow contract.

The v1 isolation and concurrency contract is covered independently:

```bash
python3 tools/runtime_api_v1_e2e.py \
  --server build/macos-release/tests/run_gui_server --data-dir data
```

# Transient Runtime Contract

Updated: 2026-08-24

The transient module is a phasor-domain electromechanical simulation. Its
implementation is under `include/hacdcpf/dynamics` and `src/dynamics`.
Historical mathematical background is retained in
`archive/theory/dynamics_electromechanical_transient_design.md`; that archived
design does not define current behavior. This document owns the runtime
contract.

## Solve path

`POST /api/session/run_transient` builds a `DynamicSystem`, optionally performs
PF initialization and dynamic trimming, applies scheduled events, integrates
the model, and records selected snapshots. The response includes initialization
diagnostics, solver statistics, bus voltage/frequency matrices, device series,
applied event records, warnings, and optional modal/CSV data.

For `MassMatrixDae`, a run is rejected before time stepping unless requested
power-flow initialization converged and the finite fast-state residual satisfies
`dynamic_trim_tol`. During the global consistent-initialization Newton solve,
the algebraic network tolerance is temporarily tightened to the smaller of the
user value and one tenth of `dynamic_trim_tol`, then restored for time stepping.
This prevents a nominally looser algebraic solve from setting the floor of a
stricter differential-state certificate. The default Anderson-Picard network
budget is ten iterations with early exit; the Newton fallback restarts from the
pre-Picard voltage seed.

The mixed AC/DC initialization uses the same DC/DC port-power equation as
steady-state power flow. Power-controlled DC/DC devices retain a first-order
power state; Voltage and Droop controls are algebraic fast-inner-loop reductions
on the electromechanical time scale. `DERAADynamic` implements the PSD/WECC
7-state (`Freq_Flag=0`) and 10-state (`Freq_Flag=1`) chains, including P/Q
current priority, frequency deadband/droop, IEEE 421.5-style non-windup,
power-order and current ramp limits. Its frequency measurement input is the
system COI frequency derived from the current dynamic state; this does not
change the local CT/PT frequency used by direct protection relays.

For `MassMatrixDae`, setting `enable_der_protection=true` and
`localize_der_protection_events=true` enables rollback/bisection localization
for protection implemented through `DynamicDevice::previewProtection`. Current
executable coverage comprises IEEE 1547 device protection and definite-time
voltage or balanced positive-sequence Zone-1 apparent-admittance relays attached
directly to a `DynamicSystem`. Relay actions currently cover `ACLoadScale` and
`ACBranchTrip`; emitted actions use the authored-event network reset path.
The request controls `protection_event_time_tol_s`,
`protection_event_cluster_window_s`,
`protection_event_max_localization_iters`, and
`post_event_algebraic_residual_tol`. The response reports whether localization
ran, trial and event-cluster counts, the largest final time bracket and cluster
span in seconds,
and the largest audited post-event algebraic residual. Exceeding either the
iteration or residual bound fails the simulation instead of returning a
plausible trajectory.

The localization tolerance and cluster window are independent. If the earliest
physical protection time is `t_e`, a non-rolling window
`[t_e, t_e + protection_event_cluster_window_s]` remains open until the DAE has
accepted its right boundary. Every protection action in that interval retains
its physical timestamp but belongs to the same cluster. An action after the
anchored boundary starts a new cluster; intermediate actions never extend it.
Each protection record carries a zero-based `protection_cluster_id`; authored
or otherwise non-protection events carry `-1`.

Direct relays use a local phasor-domain CT/PT measurement chain. With a
zero-order-held local positive-sequence input `z`, each complex transducer state
is advanced exactly as

```text
z_m(t+dt) = z + (z_m(t)-z) exp(-dt/T).
```

Local frequency is `f_n + unwrap(angle(V_PT,k)-angle(V_PT,k-1))/(2*pi*dt)` and
may pass through its own exact first-order filter. Frequency is invalid and the
frequency element is blocked below `frequency_min_voltage_pu`. After voltage
recovery it remains blocked until one complete interval has valid PT voltage at
both endpoints, so a stale pre-collapse frequency is never re-enabled. The distance
element uses `|I_CT|/|V_PT|`. These are device-local measurements; system COI
frequency is not a relay input.
The phase estimator assumes `|Delta angle| < pi` per accepted interval. The
linear CT/PT equivalent does not model saturation, hysteresis, ratio error,
anti-aliasing, or sampled-data relay firmware.

The server caches the serialized rich-space result. The GUI retrieves one
sample with:

```http
GET /api/session/transient/frame?index=N
```

## What a frame may display

- authored AC and DC bus voltage;
- authored AC bus frequency;
- dynamic device P/Q, current, controller/state, and in-service metrics that
  were actually recorded;
- SOC only when an SOC/state-of-charge series exists;
- applied event location and active/applied state.

## Prohibited inference

A transient frame must not compute branch P/Q from the static PF formula.
Dynamic branch arrows are permitted only after the dynamic solver records
terminal current or terminal P/Q for that sample. Until then the frame returns:

```json
{
  "capabilities": { "branch_power": false },
  "geo_ac_branches": [],
  "geo_dc_branches": [],
  "ac_circuit_breaker_flows": []
}
```

## Validation boundaries

GridLAB-D/OpenDSS comparisons validate only their explicitly shared model
scope. A balanced snapshot comparison does not certify dynamic inverter,
machine, DC-link, storage, or event equations. Voltage-health failures and
non-converged initialization remain visible errors; playback must not relabel a
failed run as converged.

The localized scope is numerical, not formal certification. Partitioned
integrators retain accepted-step endpoint protection semantics and emit a
warning. Rich-model builder, JSON/HTTP/GUI relay construction, inverse-time
integration, multi-zone distance logic, grazing/Zeno events, EMT switching, and
uncertainty reachability are not covered. `EmtInstantaneous` relay measurement
is rejected at construction because the network exposes phasors, not
instantaneous waveforms. The Zone-1 implementation is the COSMIC balanced
positive-sequence, no-tap line subset; it is not a general distance-protection
package. Relay guard interpolation is linear between accepted endpoints. A
within-step violation that recovers before both endpoints can therefore be
missed; reduce the base step for that regime.

External validation runs the authors' COSMIC repository at fixed clean commit
`6acc77e4d3f17925f1f4b79a93652eef0d1314cc` with MATLAB R2025b. The repository's
own `sim_case9.m` chronology is reproduced: branch 6 trips at `10.0 s`, bus 5
UVLS acts at `10.5 s`, and `31.25 MW` (25%) is shed. Reconstructing paper Fig. 2
from its stated branch-7 outage, `0.92 pu` UVLS threshold, and `0.5 s` delays does
not reproduce the published distance/UVLS sequence: bus 5 reaches `0.896065 pu`,
but branch 6's maximum distance pickup ratio is only
`0.3098124214893234 < 1`, so neither the `10.5 s` distance trip nor the `10.7 s`
UVLS event occurs. The paper configuration or code version is therefore not
fully represented by that public commit; this is an explicit failed
cross-validation, not a HySim match. Reproduction commands are documented in
`tools/transient_validation/README.md`.

The classical GUI `power_system.json` mixed case is a registered
`MassMatrixDae` initialization regression. Its Droop DC/DC device provides the
DC voltage reference through the shared steady-state/dynamic equation; missing
that control is not replaced by a fabricated DC source or a renderer fallback.

DER_A equation-level validation is exact against the local
PowerSimulationsDynamics implementation for initialization, both frequency-flag
branches, non-windup, power-order and current-ramp limits (`1e-12` gates). PSD
Test 42 trajectory parity is still not claimed: the checked-out PSD test
environment currently fails before the case runs because its SciML package set
cannot precompile (`LinearVerbosity`/extension method-overwrite conflict).

Protection actions are target-qualified by semantic `component_type` plus index
and (when supplied) bus. Thus a PV, VSC, and synchronous generator may reuse an
index without cross-tripping. Missing branch, bus-load, or fault targets fail
the run and are not appended to `applied_event_records`. Scheduled events are
authored in the caller's bus-id space; because canonical projection renumbers
non-contiguous DC bus ids and may merge AC buses, `apply_events` translates each
author-space event's `bus` into the canonical id (via the authored→canonical
maps the builder records on the network) before matching, so a DC event on a
non-contiguous authored bus resolves. An explicit `canonical_bus` param — as the
resilience DAE path supplies — takes precedence, and canonical protection-emitted
events are left unmapped. After a topology action,
MassMatrixDae resolves same-timestamp protection closure (maximum 32 iterations)
and re-solves the algebraic constraints after each action; exceeding the cap is
an explicit chronology failure. Protection clusters use a separately configured
anchored forward window and report their largest physical event-time span.
Direct voltage, frequency, and distance relays consume their local PT/CT states;
EMT instantaneous-waveform measurement remains explicitly unsupported.

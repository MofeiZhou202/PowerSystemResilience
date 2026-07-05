> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: analysis or planning note; verify decisions against current source before execution.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Reliability Assessment Code Review and Unification Design

Date: 2026-06-30

Repository: `HybridACDCDistributionSystemsSimulation`

Scope: reliability assessment code, reliability data contracts, model-scope consistency, metrics, GUI/API exposure, and a proposed design path for unifying the four currently available reliability methods.

Current methods reviewed:

1. Non-sequential Monte Carlo reliability assessment.
2. Sequential Monte Carlo reliability assessment.
3. Distribution FMEA / N-1 failure-mode enumeration.
4. Frequency and duration analytical assessment.

Related reliability/restoration method:

5. Three-stage fault-recovery reliability evaluator. This is not currently exposed in the same web reliability selector, but it should be part of the future reliability model choices because it is a distinct restoration/reconfiguration model.

## Executive Summary

The reliability assessment code has grown into several parallel implementations with different assumptions about component reliability parameters, component scope, physics scope, and metrics. The most important issue is not a single solver bug. The main issue is that the four methods do not share one reliability data contract.

At present:

- `failure_rate` usually means failures/year for AC branches.
- `forced_outage_rate` usually means steady-state unavailability or FOR.
- Several rich components use `mtbf_hours` and `mttr_hours`.
- Some methods convert these fields to unavailability, while others convert them to failure frequency.
- Missing data is often replaced by hard-coded defaults.
- GUI endpoints can automatically apply templates that overwrite case-specific reliability data.
- Model limitations already exist in core result structs, but the GUI API strips many of them.

This can produce reliability results that look authoritative while mixing different meanings of "rate", "outage probability", "repair time", "curtailment", and "customer interruption".

Recommended first milestone:

1. Define one reliability parameter contract.
2. Preserve all reliability fields through JSON/API/GUI.
3. Surface model scope and validity in every result.
4. Fix hybrid AC/DC customer metrics.
5. Make reliability template data opt-in.
6. Then unify method-specific engines behind a common assessment request/result schema.

Updated design requirement:

The reliability system should not stop at branches, generators, and a small subset of rich devices. Every rich model in the `HybridPowerSystem` should be representable in the reliability assessment. The correct abstraction is not "one reliability tuple per component"; it is "one or more failure modes per component". A VSC converter, DCDC converter, circuit breaker, or switch may have passive physical failures, active physical failures on demand, passive cyber/control failures, and active cyber/control failures on command. These modes have different probability models and different consequences, so they must be modelled separately before any OPF, reconfiguration, or restoration algorithm is called.

The intended development order is therefore:

1. Mathematical component reliability model.
2. Failure-mode taxonomy and parameterization.
3. Failure-mode-to-network consequence mapping.
4. Consequence analysis engine: AC/DC OPF, network reconfiguration, three-stage restoration, or adequacy model.
5. Reliability metrics and GUI/API presentation.

The code should follow this order. GUI controls should expose the model choices, but GUI design should not define the mathematics.

## Pre-Solver Upgrade Gates

The following gates should be completed before major OPF, network-reconfiguration, or three-stage solver upgrades.

### Priority 1: One Parameter Resolver

There must be exactly one reliability parameter resolver used by every method. No method may implement its own conversion from raw reliability fields.

The resolver must accept:

- `lambda_per_year` plus repair time.
- `MTBF` plus `MTTR`.
- `MTTF` plus `MTTR`.
- `FOR` plus `MTTR`.
- active failure probability per demand.
- demand frequency.

The resolver must output a canonical failure-mode parameter record containing at least:

- annual frequency `lambda_per_year`.
- repair or recovery duration `repair_hr`.
- steady-state unavailability `u`.
- `MTTF`.
- active probability per demand, if applicable.
- demand frequency, if applicable.
- data provenance.
- missing/default/template status.
- warnings.

### Priority 2: Explicit Missing-Data Semantics

Missing reliability data must never silently become a hidden default. Each missing value or incomplete failure mode must resolve to exactly one policy outcome:

- blocked run.
- skipped mode.
- template-filled mode.
- warning with explicit degraded status.

Every result must report how many modes were case-sourced, template-filled, skipped, unsupported, or blocked.

## Frozen Mathematical Specification

This section is the short mathematical contract that should be frozen before coding further reliability changes.

### Definitions

Let `H` be the reporting hours per year, normally 8760 unless the user explicitly selects another convention such as 8736.

For a failure mode `m`:

- `lambda_m` is the expected failure frequency in occurrences/year.
- `u_m` is steady-state unavailability, dimensionless.
- `r_m` is mean repair or recovery duration in hours.
- `mu_m = H / r_m` is repair rate in repairs/year when `r_m > 0`.
- `MTTF_m` is mean operating time to failure in hours.
- `MTTR_m` is mean time to repair in hours.
- `FOR_m` is forced outage rate, interpreted as steady-state unavailability.

`MTBF` is ambiguous in industry and must not be used without a declared convention. The future data model should prefer explicit `MTTF`. For legacy fields named `mtbf_hours`, the resolver must record which convention was applied:

- `MTBF_as_MTTF`: `lambda = H / MTBF`.
- `MTBF_as_cycle_time`: `MTTF = max(MTBF - MTTR, 0)`, then `lambda = H / MTTF`.

If the convention is not declared, strict mode blocks the mode. Compatibility mode may assume `MTBF_as_MTTF`, but must emit a provenance warning.

### Passive Failure Conversion

For `lambda` plus repair duration:

```text
mu_m = H / r_m
u_m = lambda_m / (lambda_m + mu_m)
MTTF_m = H / lambda_m
```

Equivalent form:

```text
u_m = lambda_m * r_m / (H + lambda_m * r_m)
```

For `MTTF` plus `MTTR`:

```text
lambda_m = H / MTTF_m
r_m = MTTR_m
u_m = MTTR_m / (MTTF_m + MTTR_m)
```

For cycle-time `MTBF` plus `MTTR`:

```text
MTTF_m = MTBF_m - MTTR_m
lambda_m = H / MTTF_m
r_m = MTTR_m
u_m = MTTR_m / MTBF_m
```

For legacy `MTBF_as_MTTF` plus `MTTR`:

```text
MTTF_m = MTBF_m
lambda_m = H / MTBF_m
r_m = MTTR_m
u_m = MTTR_m / (MTBF_m + MTTR_m)
```

For `FOR` plus `MTTR`:

```text
u_m = FOR_m
r_m = MTTR_m
lambda_m = FOR_m / ((1 - FOR_m) * MTTR_m) * H
MTTF_m = MTTR_m * (1 - FOR_m) / FOR_m
```

All conversions require domain checks. Negative values, non-finite values, `FOR >= 1`, repair time <= 0 for repairable modes, or zero/negative `MTTF` are invalid.

### Active-On-Demand Failure Equations

For an active failure mode:

- `p_d_m` is probability of failure per demand.
- `nu_d_m` is demand frequency in demands/year.
- `lambda_active_m = nu_d_m * p_d_m` is equivalent annual frequency.

If active failures are annualized for deterministic FMEA or non-sequential state sampling:

```text
lambda_m = lambda_active_m
u_m = lambda_m * r_m / (H + lambda_m * r_m)
```

Sequential studies should not automatically convert active failures into steady unavailable states. They should sample active failure at actual demand events, such as trip, close, open, mode-change, islanding, or dispatch-command events. If an active failure creates a persistent failed state after the demand event, the repair/recovery process starts from that event time.

### Failure-Mode Independence Assumption

Default deterministic FMEA assumes first-order rare-event independence:

```text
System risk approximately equals sum of individual mode contributions.
```

Default non-sequential MC samples independent failure-mode states unless dependency groups are explicitly provided.

The result must declare:

- `dependencies_modelled = false` when no dependency model is used.
- common-cause groups, conditional probabilities, or correlation model if dependencies are represented.

Failure-mode dependencies must not be silently ignored. If a case declares dependencies and the selected method cannot represent them, the run is blocked or the affected modes are skipped according to policy.

### Load-Point Customer Metric Definitions

Let load point `i` have demand `P_i`, customer count `N_i`, and stage shed `p_shed_{i,m,s}` under mode `m` and stage `s`.

Define the partial-shedding customer fraction:

```text
alpha_{i,m,s} = clamp(p_shed_{i,m,s} / P_i, 0, 1)
```

If `P_i <= epsilon`, then `alpha = 0`.

The default customer-interruption rule is proportional:

```text
interrupted_customers_{i,m,s} = alpha_{i,m,s} * N_i
```

For all-or-nothing load points, a binary rule may be selected:

```text
alpha_{i,m,s} = 1 if p_shed_{i,m,s} > epsilon, else 0
```

The rule used must be reported in result metadata. The default proportional rule avoids overcounting customers when an aggregate load point is partially curtailed.

For a whole event:

```text
alpha_event_{i,m} = max_s alpha_{i,m,s}
```

Then:

```text
CIF_i = sum_m f_m * alpha_event_{i,m}
CID_i = sum_m f_m * sum_s duration_{m,s} * alpha_{i,m,s}
EENS_i = sum_m f_m * sum_s duration_{m,s} * p_shed_{i,m,s}
```

where `f_m` is the annual frequency used by the selected method.

Customer indices:

```text
SAIFI = sum_i CIF_i * N_i / sum_i N_i
SAIDI = sum_i CID_i * N_i / sum_i N_i
CAIDI = SAIDI / SAIFI
ASAI = 1 - SAIDI / H
```

### Deterministic FMEA Metric Equations

For deterministic failure-mode enumeration:

```text
EENS = sum_m f_m * sum_s duration_{m,s} * sum_i p_shed_{i,m,s}
EDNS = EENS / H
LOLE = sum_m f_m * sum_s duration_{m,s} * I(total_shed_{m,s} > epsilon)
LOLF = sum_m f_m * I(any_s total_shed_{m,s} > epsilon)
PLC  = sum_m u_m * I(any_s total_shed_{m,s} > epsilon)
```

For passive modes, `f_m = lambda_m`. For active modes in deterministic FMEA, `f_m = nu_d_m * p_d_m` unless the method explicitly enumerates individual demand events.

### Monte Carlo Estimators and Uncertainty

For annual sample `y`:

```text
EENS_y = sum_t Delta_t * sum_i p_shed_i(t)
LOLE_y = sum_t Delta_t * I(total_shed(t) > epsilon)
LOLF_y = number of transitions into loss state during year y
PLC_y  = sum_t Delta_t * I(total_shed(t) > epsilon) / H
```

Point estimates:

```text
EENS_hat = mean_y(EENS_y)
LOLE_hat = mean_y(LOLE_y)
LOLF_hat = mean_y(LOLF_y)
PLC_hat  = mean_y(PLC_y)
```

Uncertainty:

```text
SE(EENS_hat) = sample_std(EENS_y) / sqrt(n)
CoV(EENS_hat) = SE(EENS_hat) / EENS_hat
```

The same pattern applies to LOLE, LOLF, and PLC when those metrics are available. Tail metrics such as VaR/CVaR must report confidence level and sample size.

### Three-Stage Duration Convention

For each failure mode `m`, use total restoration duration `r_m` from failure initiation to removal of the failed-mode constraint.

Define:

```text
d1_m = tau_iso_m
d2_m = tau_sw_m
d3_m = max(0, r_m - tau_iso_m - tau_sw_m)
```

Stage intervals:

```text
Stage 1: [0, d1_m]
Stage 2: [d1_m, d1_m + d2_m]
Stage 3: [d1_m + d2_m, r_m]
```

If a data source provides repair time after isolation and switching, convert it to total duration before metric calculation:

```text
r_m = tau_iso_m + tau_sw_m + repair_after_switch_m
```

Cyber/control failures may use `cyber_recovery_hr` instead of physical repair time when no physical repair is required. The duration source must be reported per mode.

### Metric Availability and Nullability

Unavailable or inapplicable metrics must not be reported as zero. Each metric should carry:

```json
{
  "value": null,
  "available": false,
  "reason": "not modelled by selected method",
  "model_scope": "...",
  "validity": {}
}
```

Examples:

- F&D generation adequacy does not provide SAIFI/SAIDI unless extended with load-point states.
- AC-only DC-OPF reliability does not provide full hybrid AC/DC EENS if DC load curtailment is excluded.
- A metric computed without voltage/reactive feasibility should report that limitation.

AC/DC solver limitations must be attached to every metric, not only to the top-level result.

### Data Provenance and Missing-Data Semantics

Every resolved failure mode must report parameter provenance:

- `case`.
- `user_override`.
- `template`.
- `derived`.
- `legacy_assumed`.
- `missing`.

Missing-data policy outcomes:

- `blocked`: the run does not proceed.
- `skipped`: the mode is omitted and counted as skipped.
- `template_filled`: a named template fills the value and the mode is marked defaulted.
- `warning`: the mode proceeds with explicitly degraded status.

Hidden defaults are forbidden. A numeric value used in an assessment must be traceable to case data, user override, named template, or documented derivation.

### Consequence Patch Conflict Resolution

Failure modes produce consequence patches. When multiple patches are composed, conflicts must be resolved formally.

Required precedence:

1. Baseline in-service/out-of-service state.
2. Scheduled outages.
3. Hard failure topology constraints, such as forced open or forced outage.
4. Protection and operation restrictions, such as fail-to-trip or fail-to-close.
5. Capacity deratings.
6. Control restrictions, such as setpoint frozen or grid-forming unavailable.
7. Observation/communication restrictions.

Rules:

- Hard outage dominates derating and setpoint constraints.
- Forced-open and forced-closed on the same element is a hard conflict.
- Stuck-open and command-close on the same device resolves to stuck-open plus failed command.
- Stuck-closed and command-open resolves to stuck-closed plus failed isolation.
- Multiple deratings compose by the most restrictive capacity.
- Multiple recovery durations compose by the maximum active duration unless the mode explicitly declares sequential recovery.
- Conflicting hard constraints make the patch infeasible; the mode is blocked or skipped according to policy.

Patch composition must emit diagnostics:

- affected rich components.
- affected canonical elements.
- dropped mutations.
- conflicts.
- final applied mutations.

## Mathematical Reliability Modelling Foundation

### Sets and Symbols

Let:

- `C` be the set of rich components in the submitted `HybridPowerSystem`.
- `M_c` be the set of failure modes attached to component `c`.
- `m = (c, k)` be one failure mode of component `c`.
- `D_m` be the set of demand events that can trigger an active failure mode.
- `S_m` be the set of consequence stages for mode `m`, for example switching, repair, or three-stage restoration intervals.
- `N` be the set of load points, including AC and DC load points.

For each failure mode `m`, define:

- `lambda_m`: passive failure frequency in occurrences/year.
- `p_demand_m`: active failure probability per demand event.
- `nu_demand_m`: demand-event frequency in demands/year.
- `lambda_active_m = nu_demand_m * p_demand_m`: equivalent active failure frequency in occurrences/year.
- `r_m`: mean repair time in hours.
- `u_m`: steady-state unavailability.
- `tau_iso_m`: detection/isolation time in hours.
- `tau_sw_m`: switching or cyber-restoration time in hours.
- `tau_rep_m`: physical repair time in hours.
- `q_m`: probability that the mode produces the mapped consequence once initiated.

For exponential two-state passive modes:

```text
u_m = lambda_m / (lambda_m + mu_m)
mu_m = H / r_m
```

For legacy MTBF-as-MTTF plus MTTR input:

```text
lambda_m = H / MTBF_m
u_m = MTTR_m / (MTBF_m + MTTR_m)
r_m = MTTR_m
```

For FOR/MTTR input:

```text
u_m = FOR_m
lambda_m = FOR_m / ((1 - FOR_m) * MTTR_m) * H
MTTF_m = MTTR_m * (1 - FOR_m) / FOR_m
```

For active failures:

```text
lambda_m = nu_demand_m * p_demand_m
u_m = lambda_m / (lambda_m + H / r_m)
```

This conversion is only valid if the active demand process can reasonably be represented by an annual demand frequency. In sequential simulation, active failures should instead be sampled at the actual operation or command event.

### Component Versus Failure-Mode Reliability

A component should not have a single ambiguous reliability value when it has multiple physical and cyber mechanisms. A component-level availability may be derived for summary reporting, but consequence analysis should keep modes separate.

If modes are approximately independent:

```text
U_c = 1 - product_{m in M_c}(1 - u_m)
```

For small unavailabilities:

```text
U_c approximately equals sum_{m in M_c} u_m
```

However, the approximation must not be used to collapse modes before consequence evaluation. For example, "VSC power-stage fault" and "VSC communication loss" may have similar unavailability but different network consequences.

### Passive and Active Failures

Passive failure:

- Occurs while the component is simply in service.
- Usually caused by hardware ageing, insulation breakdown, thermal stress, semiconductor failure, mechanical wear, or continuous cyber/communication outage.
- Modelled by a time-based hazard rate, normally `lambda_m`.

Active failure:

- Occurs when the component is required to act.
- Examples: switch fails to open, breaker fails to trip, breaker fails to close, VSC fails to change control mode, DCDC fails to execute a setpoint command, cyber command is delayed or rejected.
- Modelled by probability per demand event, `p_demand_m`, and optionally an annual demand frequency.

The assessment must store both forms. It is wrong to force all active failures into MTBF/MTTR without recording the demand process.

### Physical and Cyber Failure Classes

Every mode should have two independent classifications:

1. Activation class:
   - `Passive`
   - `ActiveOnDemand`

2. Cause class:
   - `Physical`
   - `CyberControl`
   - `ProtectionLogic`
   - `Communication`
   - `Measurement`
   - `HumanOperation`

Examples:

- Passive physical: cable permanent fault, transformer internal fault, converter power-stage failure.
- Active physical: breaker trip coil fails on demand, switch mechanism stuck during an open/close command.
- Passive cyber/control: persistent communication outage disables remote control.
- Active cyber/control: remote open command is lost, delayed, blocked, or malformed during restoration.
- Protection logic: false trip, failure to trip, wrong-zone trip.
- Measurement: voltage/current measurement bias causes incorrect converter or protection action.

### Failure Mode Consequence Mapping

Each failure mode must map to a network and control consequence before an analysis engine is called.

The consequence map should support:

- Topology changes:
  - forced open.
  - stuck closed.
  - stuck open.
  - cannot close.
  - cannot open.
  - wrong element trips.
  - protection zone trips.

- Capacity changes:
  - branch capacity derating.
  - converter active-power derating.
  - reactive-power derating.
  - storage energy/power derating.
  - source unavailable.

- Control changes:
  - setpoint frozen.
  - control mode lost.
  - grid-forming capability lost.
  - droop disabled.
  - VSC/DCDC transfer disabled.
  - remote control disabled but local manual operation still possible.

- Observability/cyber changes:
  - measurement unavailable.
  - measurement biased.
  - communication unavailable.
  - command delayed.
  - command blocked.

- Repair/restoration changes:
  - isolation time modified.
  - switching action forbidden.
  - manual-only operation time used.
  - cyber recovery time used instead of physical repair time.

Mathematically, define a consequence operator:

```text
G_m = Phi_m(G_0, x_m)
```

where:

- `G_0` is the original rich/canonical hybrid network.
- `x_m` is the failure-mode state.
- `G_m` is the modified network/control problem passed to the consequence engine.

The operator `Phi_m` must preserve provenance:

```text
source rich component -> failure mode -> affected canonical elements -> analysis constraints
```

This is the bridge between rich models and solver-ready canonical models.

### Consequence Analysis Models

After `Phi_m` creates the failed network/control state, one of several consequence engines can be selected.

#### AC/DC OPF Consequence Model

Purpose:

- Compute minimum load shedding under steady-state AC/DC operating constraints.

Generic objective:

```text
minimize   generation_cost + converter_cost + VOLL * sum_i p_shed_i
```

Subject to:

- AC power-balance equations or selected linear approximation.
- DC power-balance equations.
- AC branch flow limits.
- DC branch flow limits.
- Generator bounds.
- Load-shedding bounds.
- VSC active/reactive/DC coupling constraints.
- DCDC transfer constraints.
- Storage power/energy limits if the study period is finite.
- Failed-mode constraints generated by `Phi_m`.

This is the preferred consequence model when the goal is steady-state hybrid AC/DC adequacy under a failed state.

#### Network Reconfiguration Consequence Model

Purpose:

- Determine switching actions and topology that restore maximum load after a contingency.

Generic objective:

```text
minimize   VOLL * sum_i p_shed_i + switch_cost * sum_j |a_j|
```

Subject to:

- Topology/radiality constraints when required.
- Switch action limits.
- Branch and voltage constraints for the selected approximation.
- DER/source limits.
- Failed-mode constraints:
  - failed switch cannot operate.
  - cyber-disabled switch cannot be remotely operated.
  - stuck breaker cannot change state.
  - failed converter cannot support islanding.

This model is appropriate for post-contingency restoration and service recovery.

#### Three-Stage Restoration Consequence Model

Purpose:

- Evaluate time-staged customer interruption and energy not supplied during isolation, switching, and repair.

For each mode `m`, define stages:

```text
Stage 1: [0, tau_iso_m]         fault detection and isolation
Stage 2: [tau_iso_m, tau_sw_m]  post-fault switching/restoration
Stage 3: [tau_sw_m, tau_rep_m]  repair or long-duration degraded operation
```

The stage durations should come from the failure mode:

- physical passive fault: repair time from physical asset MTTR.
- cyber failure: recovery time from communication/control restoration.
- active switching failure: manual operation or field crew time.
- protection misoperation: diagnosis and reset time, unless equipment damage also occurs.

For each stage `s`, solve:

```text
P_shed_{m,s} = A_s(G_m, options)
```

Then deterministic FMEA-style contribution is:

```text
EENS_m = lambda_m * sum_s tau_{m,s} * P_shed_{m,s}
LOLE_m = lambda_m * sum_s tau_{m,s} * I(P_shed_{m,s} > epsilon)
```

This is the preferred model when repair and restoration timeline matters.

#### Frequency-Duration Adequacy Model

Purpose:

- Analytical generation adequacy.

This model should only consume generation-like failure modes unless explicitly extended to network states. It should not be presented as a full rich-component network reliability method.

### Reliability Metrics

For deterministic failure-mode enumeration:

```text
EENS = sum_m lambda_m * sum_s tau_{m,s} * P_shed_{m,s}
EDNS = EENS / T
LOLE = sum_m lambda_m * sum_s tau_{m,s} * I(P_shed_{m,s} > epsilon)
LOLF = sum_m lambda_m * I(any_s P_shed_{m,s} > epsilon)
```

For load point `i`:

```text
CIF_i = sum_m lambda_m * I(load point i interrupted by mode m)
CID_i = sum_m lambda_m * sum_s tau_{m,s} * I(p_shed_{i,m,s} > epsilon)
EENS_i = sum_m lambda_m * sum_s tau_{m,s} * p_shed_{i,m,s}
```

Customer indices:

```text
SAIFI = sum_i CIF_i * N_i / sum_i N_i
SAIDI = sum_i CID_i * N_i / sum_i N_i
CAIDI = SAIDI / SAIFI
ASAI = 1 - SAIDI / T
```

where `N_i` is the number of customers at AC or DC load point `i`, and `T` is the reporting hours per year.

For Monte Carlo methods:

```text
EENS = E[sum_t p_shed(t) * Delta_t]
LOLE = E[sum_t I(p_shed(t) > epsilon) * Delta_t]
LOLF = E[number of transitions into loss-of-load state per year]
```

Sequential MC should model active failures at operation events rather than converting every active failure into a constant unavailability.

## Rich Component Reliability Coverage Target

All rich models should be registered in a component catalog, even if the first implementation marks some failure modes as unsupported by a chosen consequence engine. The GUI should show unsupported modes as "not modelled by selected analysis", not silently ignore them.

### AC Components

| Component | Required failure modes |
| --- | --- |
| `ACBus` | busbar/station outage, bus-level load interruption, measurement/telemetry failure if bus is controlled/observed |
| `ACBranch` | passive line/cable permanent fault, temporary fault if supported, thermal derating, scheduled outage |
| `Transformer2W` / `Transformer3W` | internal fault, tap changer failure, cooling derating, scheduled maintenance |
| `ExternalGrid` | upstream supply unavailable, voltage support unavailable, short-circuit strength degradation |
| `Generator` | forced outage, derating, start failure if used as backup, control/AGC unavailable |
| `StaticGenerator` | unit outage, derating, dispatch/control unavailable |
| `RenewableGen` | unit outage, resource/availability derating, control unavailable |
| `PVSystem` | whole plant outage, inverter failure, panel/string failure, curtailment/control failure |
| `Load` | load point interruption, controllable-load control failure, priority/customer metadata |
| `FlexibleLoad` | demand response unavailable, command failure, rebound/deferred energy if modelled |
| `AsymmetricLoad` | phase-specific load interruption/degradation |
| `AsynchronousMotor` | motor outage, start failure, protection trip |
| `Storage` | whole unit outage, battery subsystem failure, PCS failure, BMS/control failure, SOC unavailable |
| `MobileStorage` | vehicle unavailable, battery/PCS/BMS failure, communication/control failure |
| `Charger` | charger outage, connector failure, communication/payment/control failure |
| `ChargingStation` | station outage, feeder/transformer outage, charger aggregation derating |
| `Switch` | passive physical open/short/outage, active fail-to-open, active fail-to-close, stuck open, stuck closed, cyber command failure, status telemetry failure |
| `CircuitBreaker` | passive hardware outage, active fail-to-trip, active fail-to-close, nuisance trip, stuck closed, stuck open, protection logic failure, cyber command/status failure |
| `Shunt` | shunt unavailable, stuck connected, stuck disconnected, control failure |
| Three-phase AC line/transformer/load/generator | phase-specific versions of the corresponding AC modes |

### DC Components

| Component | Required failure modes |
| --- | --- |
| `DCBus` | busbar outage, DC load-point interruption, measurement/telemetry failure |
| `DCBranch` | passive pole/cable fault, derating, scheduled outage |
| `StaticGeneratorDC` | source outage, derating, dispatch/control unavailable |
| `PVArrayDC` | array/string outage, DC/DC interface unavailable if represented separately, resource derating |
| `DCLoad` | load point interruption, controllable-load command failure, customer/priority metadata |
| `DCStorage` | whole unit outage, battery failure, DC/DC or PCS failure, BMS/control failure |
| `DCCircuitBreaker` | passive hardware outage, active fail-to-trip, active fail-to-close, nuisance trip, stuck open, stuck closed, protection/cyber/control failure |

### Converter and Hybrid Components

| Component | Required failure modes |
| --- | --- |
| `VSCConverter` / AC-DC converter | passive physical power-stage outage, derating, loss increase, AC-side control failure, DC-side control failure, grid-forming loss, active command failure, communication failure, measurement bias, protection block/trip |
| `DCDCConverter` | passive physical power-stage outage, derating, duty-ratio/control failure, active command failure, communication failure, measurement bias, stuck transfer/setpoint |
| `EnergyRouter` | whole router outage, port outage, internal DC link failure, routing/control failure, per-port active/passive failures |
| `EnergyRouterPort` | AC/DC port outage, port control failure, measurement/communication failure |
| `Microgrid` | islanding unavailable, black-start failure, internal source outage, controller/EMS communication failure |
| `VirtualPowerPlant` | aggregation unavailable, dispatch command failure, member DER derating |

## Failure Mode Data Model

The current `ReliabilityParams` resolver is a good component-level start. The next version should lift reliability data to failure-mode level.

Proposed core structs:

```cpp
enum class FailureActivation {
  Passive,
  ActiveOnDemand
};

enum class FailureCause {
  Physical,
  CyberControl,
  Communication,
  Measurement,
  ProtectionLogic,
  HumanOperation,
  Scheduled
};

enum class FailureConsequenceKind {
  ForcedOutage,
  Derating,
  StuckOpen,
  StuckClosed,
  FailToOpen,
  FailToClose,
  FailToTrip,
  NuisanceTrip,
  ControlUnavailable,
  SetpointFrozen,
  MeasurementBias,
  CommunicationLoss,
  GridFormingUnavailable,
  ProtectionZoneTrip
};

struct FailureModeRef {
  ComponentRef component;
  std::string mode_id;
  std::string display_name;
  FailureActivation activation;
  FailureCause cause;
  FailureConsequenceKind consequence;
};

struct FailureModeReliability {
  FailureModeRef ref;
  ReliabilityParams params;
  double probability_given_initiated{1.0};
  double demand_frequency_per_year{0.0};
  double probability_per_demand{0.0};
  double isolation_hr{0.0};
  double switching_hr{0.0};
  double repair_hr{0.0};
  double cyber_recovery_hr{0.0};
};
```

The old `failure_rate`, `forced_outage_rate`, `mtbf_hours`, and `mttr_hours` fields can remain as legacy component-level shortcuts, but the assessment engine should internally expand them into one or more `FailureModeReliability` records.

## Active and Passive Failure Models for Key Devices

### Switches

Passive physical modes:

- Mechanism unavailable while in service.
- Contact failure or abnormal resistance.
- Stuck open.
- Stuck closed.

Active physical modes:

- Fail to open on command.
- Fail to close on command.
- Slow operation beyond allowed restoration time.

Cyber/control modes:

- Remote command unavailable.
- Wrong command issued.
- Status telemetry unavailable or wrong.
- Local manual operation still possible with longer operation time.

Consequence examples:

- Fail-to-open during isolation can expand the outage zone.
- Fail-to-close during restoration blocks a tie-switch action.
- Communication loss disables remote reconfiguration but may allow manual switching after `tau_manual`.

### Circuit Breakers and DC Circuit Breakers

Passive physical modes:

- Breaker hardware unavailable.
- Contact failure.
- Stuck open.
- Stuck closed.

Active physical/protection modes:

- Fail to trip for a fault.
- Fail to close during restoration.
- Nuisance trip.
- Delayed trip.
- Protection zone miscoordination.

Cyber/control modes:

- Trip/close command blocked.
- Status telemetry wrong.
- Protection setting corrupted or unavailable.

Consequence examples:

- Fail-to-trip may require upstream breaker operation and larger load loss.
- Nuisance trip creates an outage without a downstream physical fault.
- Stuck closed can make a fault non-isolatable by the intended device.

### AC/DC VSC Converters

Passive physical modes:

- Converter outage.
- Power-stage derating.
- Increased losses.
- AC-side filter/transformer interface unavailable.
- DC-side interface unavailable.

Active/control modes:

- Fails to switch between PQ, voltage control, droop, or grid-forming mode.
- Fails to follow active/reactive power setpoint.
- Fails to participate in DC voltage coordination.

Cyber/control modes:

- Communication loss.
- Setpoint frozen.
- Measurement bias on AC voltage, DC voltage, current, or power.
- Grid-forming command unavailable.

Consequence examples:

- Passive outage removes AC/DC transfer.
- Derating reduces `pmax_mw`, `qmax_mvar`, or current limits.
- Grid-forming loss removes island source capability but may leave PQ transfer available.
- Communication loss may freeze the previous setpoint rather than trip the converter.

### DCDC Converters

Passive physical modes:

- Converter outage.
- Power-stage derating.
- Increased losses.
- Duty-ratio limit violation or unavailable topology path.

Active/control modes:

- Fails to execute voltage/power setpoint.
- Fails to change direction or transfer level.
- Control mode stuck.

Cyber/control modes:

- Communication loss.
- Setpoint frozen.
- Measurement bias on input/output voltage or current.

Consequence examples:

- Outage opens the DC transfer path.
- Derating reduces `pmax_mw`.
- Setpoint frozen fixes transfer variable instead of optimizing it.
- Measurement bias may impose incorrect voltage or duty-ratio constraints.

## Current Code Map

### Core Reliability Engine

Main files:

- `include/hacdcpf/reliability/reliability_assessment.hpp`
- `src/reliability/reliability_assessment.cpp`
- `src/reliability/reliability_data.cpp`

Implemented methods:

- `run_nonsequential_mc`
- `run_sequential_mc`
- `run_distribution_fmea`
- `run_frequency_duration_analysis`
- `compute_distribution_indices`
- `apply_ieee24_reliability_data`
- `apply_comprehensive_reliability_data`

Important current model-scope declarations:

- `ReliabilityResult::model_scope`
- `ReliabilityResult::model_limitations`
- `ReliabilityResult::validity`
- `FMEAResult::model_scope`
- `FMEAResult::model_limitations`
- `FMEAResult::validity`

These declarations are good and should become mandatory API output fields.

### Three-Stage Reliability Engine

Main files:

- `include/hacdcpf/analysis/three_stage_reliability.hpp`
- `src/reliability/three_stage_reliability.cpp`

Implemented method:

- `run_three_stage_reliability`
- `run_three_stage_reliability_from_string`

This evaluator solves a staged restoration model:

- Stage 1: fault isolation.
- Stage 2: post-fault switching restoration.
- Stage 3: post-repair topology restoration.

The current implementation uses an AC LinDistFlow MILP plus a DC connectivity/capacity fallback for hybrid networks.

### GUI and API

Main files:

- `tests/run_gui_server.cpp`
- `web/index.html`
- `web/js/app.js`

Current web selector exposes:

- `nsq`
- `seq`
- `fmea`
- `fd`

Three-stage reliability is not part of the same GUI selector yet.

### JSON I/O

Main file:

- `src/io/json_io.cpp`

This layer is critical for reliability because GUI editing, scenario import/export, and round-trip case persistence all depend on it.

### Canonical and Rich Model Boundary

Main files:

- `include/hacdcpf/model/effective_capacity.hpp`
- `include/hacdcpf/projection/project_to_canonical.hpp`
- `include/hacdcpf/projection/canonical_network.hpp`
- `src/model/network_utils.cpp`

Reliability code currently mixes direct rich-component handling with solver paths that often use canonicalized network models. This needs an explicit policy.

## Current Method Behavior

### 1. Non-Sequential Monte Carlo

Entry point:

- `run_nonsequential_mc`

Current physical evaluator:

- Calls `evaluate_state`.
- Uses AC-only DC-OPF.
- Samples a broad state vector containing AC generators, AC branches, static generators, renewable generators, AC storage, VSC converters, DC branches, transformers, DC/DC converters, DCCBs, DC storage, DC PV, AC switches, AC circuit breakers, AC PV systems, and DC static generators.

Current limitation:

- DC/VSC/rich components may be sampled, but DC load curtailment and DC power-flow balance are not included in EENS/LOLE.
- Failures of DC components can affect flags and in-service status, but the physical consequence is not a full hybrid AC/DC OPF.

Current result object already says this:

- `model_scope = "ac-only-dcopf"`
- `validity.dc_load_curtailment_included = false`
- `validity.vsc_dc_power_flow_modelled = false`

Issue:

- GUI endpoint does not return these fields.

### 2. Sequential Monte Carlo

Entry point:

- `run_sequential_mc`

Current physical evaluator:

- Also calls `evaluate_state`.
- Uses AC-only DC-OPF.
- Uses chronological up/down sampling with MTTF/MTTR derived per component.

Current limitation:

- Same physical limitation as non-sequential MC.
- DC load curtailment is excluded from system EENS/LOLE.
- Method-specific defaults for MTTF/MTTR differ from non-sequential MC default unavailabilities.

Current result object already says this:

- `model_scope = "ac-only-dcopf"`
- validity flags identify missing DC/VSC coverage.

Issue:

- GUI endpoint does not return these fields.

### 3. Distribution FMEA / N-1 Enumeration

Entry point:

- `run_distribution_fmea`

Current physical evaluator:

- Builds a component catalog.
- Evaluates switching stage and repair stage for each N-1 component outage.
- For hybrid systems, uses `evaluate_hybrid_fmea_network_lp`.
- For AC-only systems, falls back to AC-only DC-OPF.

Hybrid LP scope:

- AC branch transfer limits.
- DC branch transfer limits.
- AC/DC load shedding.
- DC sources.
- DC/DC active transfer.
- VSC active-power transfer.
- Storage emergency support when enabled.

Current limitations:

- No nonlinear AC power-flow validation.
- No reactive feasibility certification.
- No AC voltage feasibility in the hybrid LP.
- Repair-stage explicit reconfiguration enumerates AC switch/branch actions only.
- DC breaker, DC branch, VSC, and DCDC topology actions are not considered as repair reconfiguration candidates.
- Direct rich-component treatment can diverge from canonical solver projection.

Result object already exposes:

- `model_scope`
- `model_limitations`
- `validity`
- per-contingency stage fields.

Issues:

- GUI strips many important fields.
- GUI calculates `n_with_loss` using only repair-stage shed, ignoring switching-stage-only loss.
- Hybrid SAIFI/SAIDI do not currently include DC customers correctly.

### 4. Frequency and Duration Analytical Method

Entry point:

- `run_frequency_duration_analysis`

Current scope:

- Generation adequacy only.
- AC generators only.
- Constant peak load.
- No network contingencies.
- No DC network.
- No chronological load profile.
- No load-point/customer interruption metrics.

This method is still useful, but it should be labeled as "generation adequacy F&D", not "full system reliability".

### 5. Three-Stage Fault-Recovery Reliability

Entry point:

- `run_three_stage_reliability`

Current scope:

- Enumerates in-service ACBranch and DCBranch outages.
- AC network is evaluated with a LinDistFlow restoration MILP.
- DC network uses a connectivity/capacity fallback.
- VSC/DCDC are graph edges in the fallback, not fully dispatched physical devices.

Current limitations:

- Branch outages only.
- Generator, load, transformer, switch, breaker, VSC, DCDC, storage, and PV outages are not enumerated.
- Stage durations are hard-coded constants in the implementation.
- Per-component MTTR is not currently used as repair-stage duration in the reported reliability indices.
- Hybrid cases return metrics with `ok == false` because the DC/VSC/SOP portion is fallback-only.

This method should be exposed in the GUI as a separate model choice, but with strong model-scope badges.

## Critical Findings

### Finding 1: Reliability Parameter Semantics Are Not Unified

Severity: Critical

Current fields have different meanings by component and method:

- ACBranch: `failure_rate` is failures/year.
- Generator: `forced_outage_rate` is FOR/unavailability.
- StaticGenerator, RenewableGen, DCBranch, DCDC, transformers: `mtbf_hours` and `mttr_hours`.
- Storage and VSC often use `forced_outage_rate` plus repair time.
- Switches have `p_sw_fail`, `mtbf_hours`, and `mttr_hours`.
- Some breakers have no reliability fields.

Current conversions:

- Non-sequential MC needs unavailability.
- Sequential MC needs MTTF/MTTR.
- FMEA needs failure frequency and repair duration.
- F&D needs generator availability and transition frequency.

Problem:

The conversions are implemented independently, with different default values. That means the same case can produce inconsistent component risk between methods.

Required design:

Create a single resolver:

```cpp
struct ReliabilityParams {
  bool has_data{false};
  bool used_default{false};
  std::string data_source;
  double lambda_per_year{0.0};
  double repair_hr{0.0};
  double unavailability{0.0};
  double mttf_hr{0.0};
  std::vector<std::string> warnings;
};
```

Every method should call one resolver:

```cpp
ReliabilityParams resolve_reliability_params(
    const HybridPowerSystem& sys,
    ComponentRef component,
    const ReliabilityDataPolicy& policy);
```

Rules:

- If lambda/year and repair hours are provided:
  - `unavailability = lambda / (lambda + H / repair_hr)`
  - `mttf_hr = H / lambda`
- If legacy MTBF-as-MTTF and MTTR are provided:
  - `lambda_per_year = H / mtbf_hours`
  - `repair_hr = mttr_hours`
  - `unavailability = mttr / (mtbf + mttr)`
- If FOR and MTTR are provided:
  - `unavailability = FOR`
  - `lambda_per_year = FOR / ((1 - FOR) * MTTR) * H`
  - `mttf_hr = MTTR * (1 - FOR) / FOR`
- If data is missing:
  - Do not invent values unless the user selects a named default library.

### Finding 2: Missing Data Is Silently Replaced With Defaults

Severity: Critical

Examples:

- Non-sequential MC uses default unavailability values when fields are missing.
- Sequential MC uses default MTTF/MTTR values when fields are missing.
- FMEA uses default lambda and repair time values when fields are missing.
- GUI endpoints default to applying IEEE-24 or comprehensive reliability data.

Problem:

The user cannot tell whether results came from the case data or from hidden defaults.

Required design:

Introduce `ReliabilityDataPolicy`:

```cpp
enum class ReliabilityDefaultPolicy {
  StrictCaseDataOnly,
  UseNamedTemplateForMissingOnly,
  OverwriteWithNamedTemplate
};

struct ReliabilityDataPolicy {
  ReliabilityDefaultPolicy default_policy{ReliabilityDefaultPolicy::StrictCaseDataOnly};
  std::string template_name;
  bool fail_on_missing_required_data{true};
  bool report_defaulted_components{true};
};
```

GUI default should be `StrictCaseDataOnly`.

Named templates should be explicit:

- IEEE RTS-24.
- Comprehensive hybrid distribution demo.
- Utility typical distribution.
- User uploaded reliability library.

### Finding 3: Hybrid Customer Metrics Are Incomplete

Severity: Critical

FMEA hybrid evaluation can shed DC load and include it in EENS/LOLE. But `compute_distribution_indices()` only builds customer weights from AC buses and AC loads. DC load interruption frequencies and durations can be present in the nodal vectors but excluded from SAIFI/SAIDI weighting.

Required design:

Add a hybrid load-point metric builder:

```cpp
struct ReliabilityLoadPoint {
  enum class Domain { AC, DC };
  Domain domain;
  int bus_id;
  size_t nodal_index;
  double demand_mw;
  double customers;
  double priority_weight;
};
```

Then compute:

- SAIFI = sum(lambda_i * customers_i) / total_customers
- SAIDI = sum(lambda_i * duration_i * customers_i) / total_customers
- CAIDI = SAIDI / SAIFI
- ASAI = 1 - SAIDI / hours_per_year

This should include:

- AC loads.
- AC bus-level loads.
- DC loads.
- DC bus-level loads.

### Finding 4: GUI/API Hides Model Scope and Validity

Severity: High

Core result structs contain model-scope and validity flags, but GUI endpoints omit them.

Required API output for every reliability method:

```json
{
  "method": "fmea",
  "model_scope": "hybrid-acdc-network-lp",
  "model_limitations": "...",
  "validity": {
    "dc_load_curtailment_included": true,
    "vsc_dc_power_flow_modelled": true,
    "ac_opf_curtailment": true,
    "ac_voltage_reactive_feasibility_certified": false
  },
  "data_quality": {
    "components_total": 0,
    "components_with_reliability_data": 0,
    "components_defaulted": 0,
    "missing_required_data": []
  },
  "metrics": {}
}
```

The GUI should always show:

- Method.
- Physical model.
- Reliability data policy.
- Scope badge: exact, approximate, fallback, or adequacy-only.
- Missing/defaulted data warnings.
- Whether DC load curtailment is included.
- Whether AC voltage/reactive feasibility is certified.

### Finding 5: FMEA Rich Model May Diverge From Canonical Solver Model

Severity: High

Canonical projection expands rich components:

- Transformers to equivalent branches.
- Switches to equivalent branches and possible bus merges.
- Circuit breakers to equivalent branches.
- Energy routers to VSC/DCDC structures.
- VPP/microgrid/mobile storage to canonical sources/storage.

FMEA directly models rich components. This can create divergence between FMEA component IDs, branch availability, and actual PF/OPF solver behavior.

Required design decision:

Choose one of these policies:

Policy A: Canonical-first reliability.

- Project the system to canonical model first.
- Evaluate reliability on canonical components.
- Use provenance maps to report original rich components.
- Best for consistency with PF/OPF.

Policy B: Rich-component reliability.

- Reliability owns rich component semantics.
- PF/OPF is used only as a lower-level physical evaluator.
- Requires a rich-to-physical consequence map for every component.
- Best for asset-management interpretation.

Recommendation:

Use a hybrid of both:

- Build the contingency catalog in rich component space.
- Resolve each contingency into a canonical physical model mutation.
- Store provenance both ways:
  - `source_component_ref`
  - `canonical_affected_elements`
  - `modelled_consequence`

### Finding 6: Three-Stage Reliability Uses Fixed Durations for Reliability Indices

Severity: High

The three-stage evaluator uses constants:

- `kTauSwitchHr`
- `kTauTrippingHr`
- `kTauRepairHr`

Problem:

This is suitable for a staged restoration demonstration, but not for component-specific reliability indices if repair times vary by asset.

Required design:

For each faulted component:

- `tau_isolation_hr`
- `tau_switching_hr`
- `tau_repair_hr`

Then:

- Stage 1 duration = isolation time.
- Stage 2 duration = switching/restoration time window.
- Stage 3 duration = repair duration minus previous stages, or a separate post-repair operating interval if that is the intended theory.

Use the resolved component `repair_hr` as the default repair duration.

### Finding 7: F&D Method Is Adequacy-Only

Severity: Medium

The F&D method only builds a generator capacity outage probability table. It does not model network outages, DC equipment, branch constraints, load-point interruption, or restoration.

Required design:

In GUI and API, label this as:

`Generation adequacy frequency-duration`

Not:

`Full reliability assessment`

## Proposed Unified Reliability Architecture

The unified architecture should be failure-mode centric. A "method" is not the first design object. A method is only a way to aggregate or sample failure modes and call a consequence engine.

Recommended dependency direction:

```text
Rich component data
  -> component catalog
  -> failure mode catalog
  -> reliability parameter resolver
  -> consequence operator Phi_m
  -> consequence engine
  -> reliability metrics
  -> GUI/API/reporting
```

No layer should depend upward. For example, the GUI should not decide what a breaker fail-to-trip means; it should only allow the user to enable or parameterize a failure mode that is already mathematically defined.

### Layer 1: Component Identity

Introduce a stable component reference:

```cpp
enum class ReliabilityComponentKind {
  ACGenerator,
  ACBranch,
  ACLoad,
  ACBusLoad,
  ACStaticGenerator,
  ACRenewableGenerator,
  ACStorage,
  ACPVSystem,
  ACTransformer2W,
  ACTransformer3W,
  ACSwitch,
  ACCircuitBreaker,
  DCBusLoad,
  DCBranch,
  DCLoad,
  DCStaticGenerator,
  DCDCConverter,
  DCCircuitBreaker,
  DCStorage,
  DCPVArray,
  VSCConverter,
  EnergyRouter,
  Microgrid,
  MobileStorage,
  VirtualPowerPlant
};

struct ComponentRef {
  ReliabilityComponentKind kind;
  int element_index;
  std::string element_name;
  std::string domain;      // "AC", "DC", "Hybrid"
  std::string stable_id;   // for GUI and exported results
};
```

The GUI should use `stable_id`, not only vector position.

### Layer 2: Failure Mode Catalog

Every rich component should expand into zero or more failure modes. The expansion should depend on component type, device role, control role, and user-selected study scope.

Examples:

- AC branch expands to passive physical branch outage and optional scheduled outage.
- Switch expands to passive hardware outage, fail-to-open, fail-to-close, stuck-open, stuck-closed, communication loss, and status telemetry failure.
- AC circuit breaker expands to fail-to-trip, fail-to-close, nuisance trip, stuck-open, stuck-closed, protection logic failure, and communication/status failure.
- VSC expands to passive converter outage, derating, grid-forming unavailable, control-mode failure, setpoint freeze, communication loss, and measurement bias.
- DCDC expands to passive converter outage, derating, control-mode failure, duty-ratio/control failure, setpoint freeze, and communication loss.

Proposed catalog interface:

```cpp
std::vector<FailureModeReliability> build_failure_mode_catalog(
    const HybridPowerSystem& sys,
    const FailureModeCatalogOptions& options,
    const ReliabilityDataPolicy& data_policy);
```

The catalog builder should return disabled modes too when useful for diagnostics:

```cpp
struct FailureModeCatalogEntry {
  FailureModeReliability mode;
  bool enabled{true};
  bool supported_by_selected_consequence_model{true};
  std::string disabled_reason;
  std::string unsupported_reason;
};
```

### Layer 3: Reliability Parameter Resolver

All methods call the same resolver. It returns:

- lambda/year.
- repair hours.
- unavailability.
- data source.
- default status.
- validation warnings.

This resolver should be unit-tested independently.

For active failures, the resolver must also preserve:

- probability per demand.
- demand event type.
- demand frequency per year.
- equivalent annual frequency if used.

### Layer 4: Consequence Operator

Each failure mode should produce a consequence operator. This is the core model-unification layer.

```cpp
struct ConsequencePatch {
  FailureModeRef mode;
  std::vector<TopologyMutation> topology_mutations;
  std::vector<CapacityMutation> capacity_mutations;
  std::vector<ControlMutation> control_mutations;
  std::vector<ProtectionMutation> protection_mutations;
  std::vector<ObservationMutation> observation_mutations;
  std::vector<RestorationConstraintMutation> restoration_mutations;
  std::vector<std::string> affected_canonical_ids;
  std::vector<std::string> warnings;
};
```

The operator:

```cpp
ConsequencePatch build_consequence_patch(
    const HybridPowerSystem& sys,
    const FailureModeReliability& mode,
    const ConsequenceModelCapabilities& target_model);
```

Examples:

- ACBranch passive outage:
  - force branch out of service.
- Switch fail-to-close:
  - forbid selected close action in restoration model.
- Switch communication loss:
  - forbid remote operation; optionally allow manual operation after longer time.
- Circuit breaker fail-to-trip:
  - intended breaker cannot isolate fault; require upstream protective device or expanded outage zone.
- VSC passive outage:
  - remove AC/DC transfer and grid-forming support.
- VSC grid-forming control failure:
  - keep converter hardware available but remove island voltage-source capability.
- DCDC setpoint frozen:
  - fix DCDC transfer at pre-fault setpoint instead of allowing optimization.

### Layer 5: Physical Evaluation Model

Separate reliability sampling/enumeration from physical consequence evaluation.

Proposed evaluator interface:

```cpp
enum class ReliabilityPhysicalModel {
  ACOnlyDCOPF,
  HybridNetworkLP,
  ACLinDistFlowRestorationMILP,
  GenerationAdequacyCOPT,
  FutureFullHybridACDCOPF
};

struct ReliabilityPhysicalModelResult {
  double total_shed_mw;
  std::vector<double> nodal_shed_mw;
  std::string model_scope;
  std::string model_limitations;
  nlohmann::json validity;
};
```

Then methods become:

- Non-sequential MC = stochastic state sampling + selected physical evaluator.
- Sequential MC = chronological state sampling + selected physical evaluator.
- FMEA = deterministic N-1 catalog enumeration + selected physical evaluator.
- F&D = analytical generation outage table + adequacy model.
- Three-stage = branch outage catalog + restoration MILP evaluator.

### Layer 6: Unified Result Contract

Every method should return:

```cpp
struct UnifiedReliabilityResult {
  std::string method;
  std::string physical_model;
  std::string data_policy;
  std::string model_scope;
  std::string model_limitations;
  nlohmann::json validity;
  ReliabilityDataQuality data_quality;
  ReliabilityMetrics metrics;
  std::vector<NodalReliabilityMetric> nodal_metrics;
  std::vector<ContingencyReliabilityResult> contingencies;
  std::vector<ComponentImportanceResult> critical_components;
  std::vector<std::string> warnings;
};
```

Minimum common metrics:

- EENS.
- EDNS.
- LOLE.
- LOLF.
- PLC.
- SAIFI.
- SAIDI.
- CAIDI.
- ASAI.

For methods where a metric is not meaningful, return:

```json
{
  "value": null,
  "available": false,
  "reason": "F&D generation adequacy method does not compute customer interruption indices."
}
```

This is better than showing zero.

## Algorithm Design After Mathematical Model

Once the component/failure/consequence model is defined, algorithms can be selected cleanly.

### Deterministic Failure-Mode Enumeration

Use for:

- FMEA.
- N-1/N-k deterministic studies.
- Debugging and validation.
- Producing ranked critical failure modes.

Algorithm:

1. Build failure mode catalog.
2. Filter enabled and supported modes.
3. For each mode `m`:
   - resolve reliability parameters.
   - build consequence patch `Phi_m`.
   - solve selected consequence model.
   - compute frequency-weighted metrics.
4. Aggregate system, nodal, and customer metrics.

Pseudo-code:

```cpp
for (const auto& mode : modes) {
  ReliabilityParams rp = mode.params;
  ConsequencePatch patch = build_consequence_patch(sys, mode, engine.capabilities());
  ConsequenceResult cr = engine.evaluate(sys, patch, options);
  accumulate_metrics(mode, rp, cr);
}
```

### Non-Sequential Monte Carlo

Use for:

- State probability sampling.
- Multi-component outage combinations.
- Approximate risk over large catalogs.

Important rule:

Passive modes can be sampled from unavailability. Active modes should not be sampled as steady unavailable components unless the user explicitly selects equivalent annualization.

Algorithm:

1. Build failure mode catalog.
2. Convert passive modes to Bernoulli unavailability.
3. Convert active modes only if annualized demand model is enabled.
4. Sample failure-mode state vector.
5. Compose consequence patches.
6. Solve selected consequence model.
7. Estimate EENS, LOLE, PLC, and uncertainty.

### Sequential Monte Carlo

Use for:

- Chronological load profiles.
- Repair process.
- Active failure at operation events.
- Weather/cyber/time-correlated extensions.

Algorithm:

1. Build failure mode catalog.
2. For passive modes, sample time-to-failure and time-to-repair.
3. For active modes, sample failure on actual command/protection demand events.
4. Maintain component/failure-mode state timeline.
5. At each event or time step, compose active consequence patches.
6. Solve selected consequence model or reuse cached result.
7. Aggregate annual distributions.

### Three-Stage Restoration

Use for:

- Time-dependent restoration and switching.
- Active protection/switching failures.
- Cyber/manual restoration delays.

Algorithm:

1. Build failure mode catalog.
2. Select modes that require staged restoration.
3. For each mode, build stage-specific patches:
   - isolation patch.
   - switching patch.
   - repair patch.
4. Solve restoration model for each stage.
5. Weight stage load shedding by mode frequency and duration.

### Frequency-Duration

Use for:

- Generation adequacy.
- Analytical capacity outage tables.

This should remain a specialized algorithm consuming generation-like passive outage modes unless a new network-state F&D theory is implemented.

## GUI Redesign Proposal

### Design Goal

The reliability GUI should make the user choose:

1. Reliability method.
2. Physical consequence model.
3. Reliability data source/policy.
4. Failure-mode scope.
5. Component scope.
6. System analysis options.
7. Output metrics and diagnostics.

It should not silently choose templates or hide model limitations.

The GUI must reflect the mathematical hierarchy:

```text
Component -> failure mode -> consequence model -> method/algorithm -> metrics
```

The user should be able to inspect and edit component reliability data, but the GUI should also show the expanded failure modes because switches, breakers, VSCs, and DCDC converters do not have only one failure behavior.

### Proposed Reliability Page Layout

Use one main analysis workspace, not separate hidden forms per method.

Sections:

1. Method and physical model.
2. Reliability data.
3. Failure-mode library.
4. Component scope.
5. Load and time settings.
6. Restoration and switching settings.
7. Advanced solver settings.
8. Run and diagnostics.
9. Results.

### Section 1: Method and Physical Model

Controls:

- Method segmented control:
  - Non-sequential MC.
  - Sequential MC.
  - FMEA N-1.
  - Frequency-duration adequacy.
  - Three-stage restoration.

- Physical model selector:
  - AC-only DC-OPF.
  - Hybrid AC/DC network LP.
  - AC LinDistFlow restoration MILP.
  - Generation adequacy COPT.
  - Future full hybrid AC/DC OPF.

Compatibility behavior:

- F&D only allows generation adequacy COPT.
- Three-stage defaults to AC LinDistFlow restoration MILP.
- MC methods can initially use AC-only DC-OPF, with hybrid LP as a future option.
- FMEA can use AC-only DC-OPF or hybrid AC/DC network LP.

Display badges:

- `AC only`
- `Hybrid included`
- `DC fallback`
- `Voltage not certified`
- `Adequacy only`
- `Restoration MILP`

### Section 2: Reliability Data

Controls:

- Data policy menu:
  - Case data only.
  - Template for missing fields only.
  - Overwrite with template.

- Template menu:
  - None.
  - IEEE RTS-24.
  - Comprehensive hybrid distribution demo.
  - User uploaded library.

- Missing-data behavior:
  - Block run.
  - Warn and skip missing components.
  - Warn and use selected template.

- Component parameter table:
  - Component.
  - Domain.
  - Type.
  - Lambda/year.
  - FOR/unavailability.
  - MTBF hours.
  - MTTR hours.
  - Customers.
  - Source: case/template/default/missing.
  - Validation status.

Table actions:

- Edit selected component.
- Bulk apply template to selected components.
- Export reliability data.
- Import reliability data CSV/JSON.
- Reset edited values.

### Section 3: Failure-Mode Library

Controls:

- Failure activation filters:
  - Passive failures.
  - Active on-demand failures.

- Failure cause filters:
  - Physical.
  - Cyber/control.
  - Communication.
  - Measurement.
  - Protection logic.
  - Human operation.
  - Scheduled.

- Failure consequence filters:
  - Forced outage.
  - Derating.
  - Fail to open.
  - Fail to close.
  - Fail to trip.
  - Nuisance trip.
  - Stuck open.
  - Stuck closed.
  - Control unavailable.
  - Grid-forming unavailable.
  - Setpoint frozen.
  - Measurement bias.
  - Communication loss.

Failure-mode table columns:

- Enabled.
- Component.
- Mode name.
- Activation.
- Cause.
- Consequence.
- Lambda/year.
- Probability per demand.
- Demand frequency/year.
- Repair/recovery time.
- Isolation time.
- Switching/manual operation time.
- Supported by selected consequence model.
- Data source.
- Warnings.

Mode-specific editing examples:

- Switch fail-to-open:
  - probability per operation.
  - manual fallback time.
  - affected switch action: open.

- Circuit breaker fail-to-trip:
  - probability per fault-clearing demand.
  - backup protection clearing time.
  - upstream outage zone mapping.

- VSC grid-forming unavailable:
  - failure frequency or probability per islanding demand.
  - whether PQ operation remains available.
  - cyber recovery time.

- DCDC setpoint frozen:
  - communication failure frequency.
  - frozen setpoint source.
  - recovery time.

The GUI should show unsupported modes as disabled with an explanation. Example:

`VSC measurement bias is defined, but the selected Hybrid Network LP does not model measurement equations.`

### Section 4: Component Scope

Controls:

- Include component kinds:
  - AC branches.
  - DC branches.
  - AC generators.
  - DC generators.
  - VSC converters.
  - DCDC converters.
  - Transformers.
  - Switches.
  - AC circuit breakers.
  - DC circuit breakers.
  - Storage.
  - PV.
  - Loads.

- Fault set:
  - N-1 only.
  - Selected components only.
  - Branch-only.
  - Generation-only.
  - Critical components from previous run.

- Domain:
  - AC.
  - DC.
  - Hybrid.

### Section 5: Load and Time Settings

Controls:

- Load scale factor.
- Load profile:
  - Constant.
  - IEEE RTS-24 hourly profile.
  - Uploaded hourly profile.
  - Existing time-series profile.

- Hours per year:
  - 8760.
  - 8736.
  - Custom.

- Customer source:
  - Load table `n_customers`.
  - Bus table `n_customers`.
  - Estimate from MW.
  - Uploaded customer table.

### Section 6: Restoration and Switching Settings

Shown for FMEA and three-stage.

Controls:

- Switching time.
- Isolation time.
- Use component MTTR for repair duration.
- Manual repair duration override.
- Enable repair reconfiguration.
- Enable switch reconfiguration.
- Maximum switch actions.
- Maximum OPF/MILP evaluations.
- Include AC switches.
- Include DC breaker actions.
- Include normally-open branch candidates.
- Include active switching failures.
- Include breaker fail-to-trip/fail-to-close modes.
- Include cyber/manual fallback times.

Current code supports only part of this, so unsupported options should be disabled with a clear model-scope message.

### Section 7: Advanced Solver Settings

Controls:

- Random seed.
- Max samples / years.
- CoV threshold.
- Tail risk enabled.
- VaR confidence.
- OPF load shedding penalty.
- MILP gap tolerance.
- MILP time limit.
- Solver log verbosity.

### Section 8: Pre-Run Diagnostics

Before run, show:

- Number of components in scope.
- Number of failure modes in scope.
- Number of active failure modes.
- Number of passive failure modes.
- Number of physical failure modes.
- Number of cyber/control failure modes.
- Number with complete reliability data.
- Number missing reliability data.
- Number defaulted/template-filled.
- Number unsupported by selected consequence model.
- Whether DC load curtailment will be included.
- Whether AC voltage/reactive feasibility is certified.
- Whether result will be exact, approximate, or fallback.

Block run only if:

- Required reliability data is missing and policy is strict.
- Selected method/model combination is unsupported.
- Enabled failure modes cannot be mapped to the selected consequence model and the user selected strict model support.
- No valid contingency/sampling components exist.

### Section 9: Results Design

Result tabs:

1. Summary.
2. Model scope.
3. Data quality.
4. Failure-mode coverage.
5. Critical components.
6. Critical failure modes.
7. Contingencies.
8. Nodal/customer metrics.
9. Convergence and uncertainty.
10. Export.

Summary KPIs:

- EENS.
- EDNS.
- LOLE.
- LOLF.
- PLC.
- SAIFI.
- SAIDI.
- CAIDI.
- ASAI.
- Cost of EENS.

Each KPI should show availability:

- Available.
- Not applicable.
- Approximate.
- Excluded by model.

Model scope tab:

- Model name.
- Validity flags.
- Limitations.
- Physical constraints included/excluded.
- DC treatment.
- VSC/DCDC treatment.
- Restoration treatment.

Contingency table:

- Rank.
- Component name.
- Component kind.
- Domain.
- Failure mode.
- Activation: passive or active.
- Cause: physical, cyber/control, communication, measurement, protection.
- Consequence kind.
- Failure rate.
- Probability per demand if active.
- Demand frequency if active.
- Repair time.
- Switching-stage shed.
- Repair-stage shed.
- EENS contribution.
- LOLE contribution.
- Loss flags.
- Selected repair actions.
- Search truncated flag.

Nodal/customer table:

- Domain.
- Bus/load id.
- Demand.
- Customers.
- EENS.
- CIF.
- CID.
- SAIFI contribution.
- SAIDI contribution.

Failure-mode coverage tab:

- Total rich components.
- Total generated modes.
- Enabled modes.
- Disabled modes.
- Unsupported modes by selected consequence engine.
- Modes using case data.
- Modes using template data.
- Modes missing data.
- Active/passive split.
- Physical/cyber split.

## API Redesign Proposal

### New Endpoint

Instead of separate method endpoints only, add:

`POST /api/session/run_reliability`

Request:

```json
{
  "method": "fmea",
  "physical_model": "hybrid_network_lp",
  "data_policy": {
    "mode": "case_data_only",
    "template": null,
    "fail_on_missing_required_data": true
  },
  "component_scope": {
    "domains": ["AC", "DC"],
    "kinds": ["ACBranch", "DCBranch", "VSCConverter", "DCDCConverter"],
    "only_in_service": true
  },
  "failure_mode_scope": {
    "activation": ["passive", "active_on_demand"],
    "causes": ["physical", "cyber_control", "communication", "protection_logic"],
    "consequences": ["forced_outage", "derating", "fail_to_open", "fail_to_trip", "control_unavailable"],
    "include_unsupported_as_warnings": true
  },
  "load": {
    "scale_factor": 1.0,
    "profile": "constant",
    "hours_per_year": 8760
  },
  "restoration": {
    "switching_time_hr": 0.5,
    "use_component_repair_time": true,
    "enable_switch_reconfiguration": true,
    "max_switch_actions": 2,
    "max_physical_evaluations": 200
  },
  "monte_carlo": {
    "max_iterations": 5000,
    "cov_threshold": 0.05,
    "seed": 0,
    "compute_tail_risk": false,
    "var_confidence": 0.95
  }
}
```

Response:

```json
{
  "method": "fmea",
  "physical_model": "hybrid_network_lp",
  "model_scope": "hybrid-acdc-network-lp",
  "model_limitations": "...",
  "validity": {},
  "data_quality": {},
  "failure_mode_coverage": {},
  "metrics": {},
  "contingencies": [],
  "critical_failure_modes": [],
  "nodal_metrics": [],
  "critical_components": [],
  "warnings": []
}
```

Keep old endpoints temporarily as wrappers:

- `/api/session/run_reliability_nsq`
- `/api/session/run_reliability_seq`
- `/api/session/run_reliability_fmea`
- `/api/session/run_reliability_fd`

But make them call the unified request path internally.

## Debug and Implementation Roadmap

### Phase 0: Mathematical Specification Freeze

Goal:

Document and agree on the reliability mathematics before changing algorithms.

Tasks:

1. Finalize the rich component catalog.
2. Finalize the failure activation classes:
   - passive.
   - active on demand.
3. Finalize the cause classes:
   - physical.
   - cyber/control.
   - communication.
   - measurement.
   - protection logic.
   - human operation.
   - scheduled.
4. Finalize the consequence kinds:
   - outage.
   - derating.
   - stuck state.
   - fail-to-operate.
   - nuisance operation.
   - control unavailable.
   - measurement/communication degradation.
5. Define mathematical conversion for lambda, probability per demand, demand frequency, repair time, recovery time, and unavailability.
6. Define metric equations for deterministic enumeration, MC, sequential MC, and three-stage restoration.

Expected result:

All later code implements an agreed model rather than patching method-specific behavior.

### Phase 1: Preserve Current Behavior But Expose Truth

Goal:

No major solver/model changes. Make outputs honest.

Tasks:

1. Return `model_scope`, `model_limitations`, and `validity` from NSQ/SEQ/FMEA/F&D endpoints.
2. Return complete critical component metadata:
   - component type.
   - component name.
   - global state index.
   - domain.
3. Return complete FMEA contingency details:
   - switching-stage shed and ENS.
   - repair-stage shed and ENS.
   - loss flags.
   - repair switch actions.
   - truncation flag.
   - nodal shedding.
4. Fix GUI `n_with_loss` to use core `n_loss_contingencies` or count both switching and repair stages.
5. Make GUI default reliability template application opt-in.
6. Add visible model-scope warnings in the results page.

Expected result:

Existing results may not change numerically, but users can see what the numbers mean.

### Phase 2: Failure-Mode Data Contract

Goal:

All methods use identical reliability parameters for identical failure modes.

Tasks:

1. Implement `ComponentRef`.
2. Implement `FailureModeRef`.
3. Implement `FailureModeReliability`.
4. Extend `ReliabilityParams` for active failure fields:
   - probability per demand.
   - demand frequency/year.
   - equivalent lambda/year.
   - cyber recovery time.
5. Implement `ReliabilityDataPolicy`.
6. Implement one resolver for all failure-mode types.
7. Replace method-local conversion/default logic with resolver calls.
8. Add data-quality and failure-mode-coverage summaries.
9. Add strict missing-data mode.

Expected result:

MC, FMEA, and three-stage use the same lambda, active-failure probability, and repair/recovery times for the same failure mode.

### Phase 3: Failure-Mode Catalog and Consequence Mapping

Goal:

All rich model types can be represented in reliability assessment, even if some modes are initially unsupported by some consequence engines.

Tasks:

1. Build a rich component enumerator.
2. Build default failure-mode expansions for every rich component type.
3. Add active/passive physical/cyber modes for:
   - switches.
   - AC circuit breakers.
   - DC circuit breakers.
   - VSC converters.
   - DCDC converters.
   - energy routers.
4. Implement consequence patches:
   - forced outage.
   - derating.
   - stuck open/closed.
   - fail to open/close/trip.
   - nuisance trip.
   - control unavailable.
   - setpoint frozen.
   - grid-forming unavailable.
   - communication/manual fallback.
5. Add model capability checks so unsupported modes are reported, not ignored.

Expected result:

The reliability engine has full rich-component visibility before solver upgrades.

### Phase 4: JSON and GUI Data Preservation

Goal:

Reliability data survives round trips.

Tasks:

1. Add missing DC bus fields to JSON:
   - `n_customers`
   - `importance`
   - `is_load`
2. Add missing DC branch reliability fields:
   - `mtbf_hours`
   - `mttr_hours`
   - `t_scheduled_hr`
3. Add missing DC load fields:
   - `scaling`
   - `n_customers`
   - `priority`
   - rated/metadata fields if present in the model.
4. Add reliability fields to breaker models if asset-level reliability is required.
5. Add GUI reliability parameter table.
6. Add import/export of reliability parameter library.

Expected result:

User-entered reliability parameters are not erased by save/load/import/export.

### Phase 5: Hybrid Metrics

Goal:

AC and DC customer metrics are correct.

Tasks:

1. Replace or overload `compute_distribution_indices`.
2. Build explicit AC/DC reliability load points.
3. Include DC loads and DC bus-level loads in SAIFI/SAIDI.
4. Add nodal metrics with domain and bus id.
5. Add tests for DC-only and mixed AC/DC systems.

Expected result:

DC interruption contributes to SAIFI/SAIDI/ASAI, not only EENS/LOLE.

### Phase 6: Method Unification

Goal:

One request/result contract drives all methods.

Tasks:

1. Implement unified reliability request.
2. Implement unified result schema.
3. Add method wrappers.
4. Add GUI method/model compatibility logic.
5. Keep existing endpoints as backward-compatible wrappers.

Expected result:

Adding new reliability methods no longer requires custom GUI/API/result logic.

### Phase 7: Physical Model Alignment

Goal:

Model choices are explicit and consistent with canonical/rich component semantics.

Tasks:

1. Decide canonical-first vs rich-first contingency semantics.
2. Add component provenance to reliability results.
3. Optionally allow FMEA to evaluate canonical-projected consequences.
4. Optionally upgrade MC to use hybrid network LP for AC/DC load curtailment.
5. Extend three-stage durations to component-specific repair times.
6. Extend three-stage contingency catalog beyond branch-only using failure modes.
7. Add active failure handling in sequential MC and three-stage restoration.
8. Add cyber/control consequence support where mathematically defined.

Expected result:

Reliability results can be compared across methods because they share the same data and clearly declare physical differences.

## Recommended Test Plan

### Unit Tests

Reliability parameter resolver:

- Lambda/year plus MTTR produces correct unavailability.
- MTBF/MTTR produces correct lambda and unavailability.
- FOR/MTTR produces correct lambda and MTTF.
- Active failure probability per demand plus demand frequency produces equivalent annual lambda.
- Active failure remains distinguishable from passive failure after resolution.
- Missing data in strict mode reports missing, not defaults.
- Template-for-missing mode marks defaulted components.
- Overwrite-template mode marks overwritten components.

Failure-mode catalog:

- Every rich component type emits expected default failure modes.
- Switch emits passive physical, fail-to-open, fail-to-close, stuck-open, stuck-closed, and communication/status modes.
- AC and DC circuit breakers emit fail-to-trip, fail-to-close, nuisance trip, stuck-open, stuck-closed, and protection/cyber modes.
- VSC emits passive outage, derating, grid-forming unavailable, setpoint frozen, communication loss, and measurement modes.
- DCDC emits passive outage, derating, control failure, setpoint frozen, communication loss, and duty/control modes.
- Unsupported modes are reported with reason under each consequence engine.

Consequence mapping:

- Branch outage maps to forced-open topology mutation.
- Switch fail-to-close forbids restoration close action.
- Breaker fail-to-trip expands isolation to backup protection zone.
- VSC grid-forming failure removes island-forming capability but can preserve PQ transfer if configured.
- DCDC setpoint frozen fixes transfer instead of allowing optimization.

JSON round-trip:

- DC bus customer fields survive.
- DC branch MTBF/MTTR survive.
- DC load scaling/customers survive.
- DCDC MTBF/MTTR survive.
- Switch MTBF/MTTR and operation fields survive.

Metrics:

- DC-only load interruption changes SAIFI/SAIDI.
- AC+DC mixed load interruption weights customers correctly.
- ASAI uses configured hours per year.
- FMEA switching-only interruption counts as loss contingency.

### Integration Tests

Non-sequential MC:

- Hybrid system returns `model_scope = ac-only-dcopf`.
- DC load curtailment validity flag is false.
- Missing data strict mode blocks or warns.

Sequential MC:

- Uses same resolved parameters as NSQ.
- Annual outputs retain model-scope metadata.

FMEA:

- Hybrid LP includes DC load shedding in EENS.
- DC customers included in SAIFI/SAIDI.
- Active and passive modes are separately ranked.
- Contingency details include both switching and repair stages.
- Repair search truncation surfaces in API.

F&D:

- Labeled as generation adequacy.
- Does not expose unavailable customer metrics as zero.

Three-stage:

- Per-component repair duration affects EENS once implemented.
- Failure-mode catalog scope is reported in data/model scope.
- Active switch/breaker failure affects restoration stages.
- Cyber communication failure can change remote operation to delayed/manual operation.
- Hybrid fallback sets `ok = false` and exposes validity flags.

### GUI/API Tests

- Default run does not overwrite reliability data.
- User can select template explicitly.
- Result page displays model-scope badges.
- Result export contains validity, data quality, and limitations.
- Parameter table edits are reflected in the backend request.
- Failure-mode table edits are reflected in the backend request.
- Active/passive/cyber/physical filters change the catalog and diagnostics.
- Method/model incompatible combinations are disabled or rejected with clear errors.

## Immediate High-Value Fixes

These are the best first debug tasks because they reduce misleading results quickly:

1. Change GUI defaults so reliability templates are opt-in.
2. Return model scope and validity from all reliability endpoints.
3. Preserve missing DC reliability/customer fields in JSON.
4. Fix hybrid SAIFI/SAIDI to include DC customers.
5. Create a shared reliability parameter resolver and migrate FMEA first.
6. Add tests for the resolver and DC customer metrics.

## Open Design Questions

1. Should MC remain AC adequacy only, or should it support the hybrid FMEA LP?
2. Should FMEA contingencies be rich assets, canonical elements, or both with provenance?
3. Should three-stage reliability become one of the four primary GUI methods or a separate restoration-reliability mode?
4. Should breaker and switch failures be asset outages, operation failures, or both?
5. Should missing reliability data block runs by default?
6. What is the official hours-per-year convention for this project: 8760 or 8736?
7. Should customer counts be stored on buses, loads, or both with a clear precedence rule?

## Proposed GUI Labels

Recommended method names:

- `Monte Carlo - non-sequential`
- `Monte Carlo - sequential`
- `FMEA / N-1 enumeration`
- `Frequency-duration generation adequacy`
- `Three-stage restoration reliability`

Recommended physical model names:

- `AC-only DC-OPF`
- `Hybrid AC/DC network LP`
- `AC LinDistFlow restoration MILP`
- `Generation adequacy COPT`
- `Full hybrid AC/DC OPF (future)`

Recommended scope badges:

- `AC only`
- `DC load excluded`
- `Hybrid load included`
- `DC fallback`
- `Voltage not certified`
- `Reactive feasibility not certified`
- `Generation adequacy only`
- `Branch-only contingencies`
- `Template data used`
- `Case data only`

## Conclusion

The reliability modules should be unified around a common data contract before deeper solver work. Once lambda, repair time, unavailability, component identity, customer weighting, and model-scope reporting are consistent, the four methods can remain different where they should be different: sampling versus enumeration, adequacy versus network reliability, and static OPF versus staged restoration.

The near-term goal should not be to force all methods to produce identical numbers. It should be to make them consume the same component data, report comparable metrics, and clearly explain which physical assumptions each result used.

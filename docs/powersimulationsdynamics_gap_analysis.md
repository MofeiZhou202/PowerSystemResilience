> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: analysis or planning note; verify decisions against current source before execution.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# HACDCPF vs PowerSimulationsDynamics.jl Dynamic Capability Analysis

Date: 2026-07-04

This note compares the current transient modelling and analysis capability in
this repository with the local sibling repository:

```text
/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl
```

The user-facing question is whether HACDCPF can be treated as comparable to
PowerSimulationsDynamics.jl for transient dynamics, especially for power
electronic controls and verification.

Short answer: not yet. HACDCPF now has a useful hybrid AC/DC transient layer,
GUI integration, rich static component coverage, canonical projection, and
GridLAB-D/OpenDSS static-network validation plumbing. PowerSimulationsDynamics.jl
is still much more mature in industry-style dynamic model decomposition,
converter control libraries, equilibrium initialization, small-signal analysis,
and benchmark validation against PSS/E, PSCAD, PSAT, and ANDES.

The right direction is not to depend on Fortran or to replace the C++ module.
The right direction is to keep the HACDCPF hybrid AC/DC, three-phase,
distribution, GUI, and IO architecture, then adopt a PowerSimulationsDynamics.jl
style dynamic-component taxonomy and verification discipline.

## 2026-07-04 Reinvestigation Result

The live PSD ledgers are:

- `tools/psd_validation/psd_model_comparison.md`
- `tools/psd_validation/psd_component_test_matrix.md`

They are the authority for current stop/go status. As of this update, the
model crosswalk has 44 rows: 2 `exact-or-close`, 22 `supported-subset`, 1
`trace-failing`, 2 `profile-only`, and 17 `missing`. The machine/IBR component
gate has 39 rows: 12 `compare-limited`, 1 `compare-failing`, 12
`blocked-missing-model`, 9 `blocked-missing-controller`, 3
`blocked-missing-formulation`, and 2 `metadata-only`.

Current answer to "can we pass the PSD machine/IBR component suite?" is no.
The reason is not one tolerance problem. The suite is blocked by missing model
families, missing controller families, and missing residual/mass-matrix and
small-signal parity.

What can be compared today is a reduced trace subset:

- OneDOneQMachine from PSD Test 02: `delta`, `omega`, `eq_p`, `ed_p`.
- GENROU from PSD Test 15: reduced machine trace gates.
- GENROE from PSD Test 16, including high-saturation variant: `delta`,
  `omega`, `eq_p`, `ed_p`.
- GENSAL from PSD Test 18: `delta`, `omega`, `eq_p`, `psiq_pp`.
- GENSAE from PSD Test 19: `delta`, `omega`, `eq_p`, `psiq_pp`.
- SimpleMarconatoMachine from PSD Test 03 now has a runtime path and exporter,
  but the external trace gate is not accepted because generator-103
  `delta_rad_relative` fails after the BUS 1-BUS 3 trip.
- Grid-following inverter Tests 24 and 51: selected active-power traces.
- ZIP/constant-power load traces from PSD Test 33.

These are useful validation gates, but they are not full PSD parity. Full pass
claims require a residual/mass-matrix formulation, initialization residual
checks at PSD scope, eigenvalue/small-signal comparison, and the remaining
model/controller library.

Julia and PowerSimulationsDynamics.jl remain local validation oracles only.
They must not be linked into or required by the HACDCPF release module.

## Evidence Sources

HACDCPF sources inspected:

- `include/hacdcpf/model/hybrid_power_system.hpp`
- `include/hacdcpf/model/ac_components.hpp`
- `include/hacdcpf/model/converter_components.hpp`
- `include/hacdcpf/io/component_io_mapping.hpp`
- `src/io/component_io_mapping.cpp`
- `include/hacdcpf/dynamics/DynamicSystem.hpp`
- `include/hacdcpf/dynamics/DynamicSolverOptions.hpp`
- `include/hacdcpf/dynamics/DynamicEvent.hpp`
- `include/hacdcpf/dynamics/DynamicResults.hpp`
- `include/hacdcpf/dynamics/devices/DynamicDevice.hpp`
- `include/hacdcpf/dynamics/devices/BasicDynamicDevices.hpp`
- `src/dynamics/DynamicModelBuilder.cpp`
- `src/dynamics/DynamicSystem.cpp`
- `src/dynamics/DynamicSolver.cpp`
- `src/dynamics/devices/BasicDynamicDevices.cpp`
- `src/dynamics/integration/*`
- `src/dynamics/solvers/*`
- `tests/test_transient_dynamics.cpp`
- `docs/transient_simulation.md`
- `docs/transient_analysis.md`
- `docs/Updated_GFL_GFM_Control_Modeling.md`

PowerSimulationsDynamics.jl sources inspected:

- `../PowerSimulationsDynamics.jl/src/PowerSimulationsDynamics.jl`
- `../PowerSimulationsDynamics.jl/docs/src/models.md`
- `../PowerSimulationsDynamics.jl/docs/src/initialization.md`
- `../PowerSimulationsDynamics.jl/docs/src/perturbations.md`
- `../PowerSimulationsDynamics.jl/docs/src/small.md`
- `../PowerSimulationsDynamics.jl/src/models/generator_models/*`
- `../PowerSimulationsDynamics.jl/src/models/inverter_models/*`
- `../PowerSimulationsDynamics.jl/src/initialization/generator_components/*`
- `../PowerSimulationsDynamics.jl/src/initialization/inverter_components/*`
- selected PSD tests including renewable, DERA, REGCA voltage, grid-following,
  droop inverter, VSM, and VOC cases.

## Executive Comparison

| Surface | HACDCPF current state | PowerSimulationsDynamics.jl current state | Gap |
|---|---|---|---|
| Primary scope | Hybrid AC/DC distribution, rich static components, canonical projection, GUI, power flow, OPF, reliability, transient extension | Positive-sequence dynamic simulation framework integrated with PowerSystems.jl | HACDCPF is broader in hybrid network scope; PSD is deeper in dynamics |
| Dynamic device architecture | `DynamicDevice` plus concrete compact devices in `BasicDynamicDevices` | Dynamic injection composition: machine/shaft/AVR/PSS/governor and inverter DC source/frequency estimator/outer/inner/converter/filter/limiter | HACDCPF needs composable dynamic subcomponents |
| Inverter GFL | Generic PLL/current-source subset, current limit, droop options, optional DC link | KauraPLL, ReducedOrderPLL, FixedFrequency, renewable active/reactive controllers, RECurrentControlB, REGCA/REEC/REPCA-style behavior, LVPL, current ramp, current limiters | HACDCPF GFL is not industry-complete |
| Inverter GFM | Norton voltage-source droop model with telemetry and optional DC link | Droop inverter, VSM, VOC and average converter tests with PSCAD-style references | HACDCPF needs multiple standard GFM profiles |
| AC filters | Algebraic Norton/reactance interface | RL and differential LCL filter models with mass-matrix entries | HACDCPF lacks selectable filter dynamics |
| DC side | Native AC/DC network, DC buses, DC branches, VSC, DCDC, battery, PV, optional dynamic DC link | DC source component inside inverter meta-model; not a hybrid AC/DC network framework | HACDCPF is stronger here, but needs standard DC-source submodels |
| Conventional machines | Classical, OneDOneQ, GENROU, GENROE, GENSAL, and GENSAE reduced runtime paths with selected PSD trace gates; simplified SEXS/TGOV1/IEEET1/PSS1A coverage | Many machine, shaft, AVR, PSS, and governor families with initialization, residual/mass-matrix, and eigenvalue tests | HACDCPF still needs Marconato, Anderson-Fouad, Sauer-Pai, five-mass shaft, broader controllers, and full DAE parity |
| Initialization | Starts from power flow, initializes device states, runs local trim hooks, records `dynamic_fast_dxdt_inf_norm` | PF-based device initialization, inverter sequence, full nonlinear equilibrium solve, strict no-perturbation stationary expectation | HACDCPF needs system-wide DAE equilibrium solve |
| Numerical formulation | Device-stamped partitioned phasor network, Euler/Heun/RK4, Backward Euler, trapezoidal, Rosenbrock-like step, numerical state Jacobian | `ResidualModel` and `MassMatrixModel`, simultaneous DAE formulation, SciML solvers, AD Jacobian | HACDCPF lacks full residual/mass-matrix DAE API and sparse Jacobian path |
| Small signal | Not yet a first-class dynamic API | `get_jacobian`, reduced Jacobian, eigenvalues/eigenvectors, participation summaries | Major HACDCPF gap |
| Perturbations | Branch trip/close, load scale, generator/VSC/DCDC trips, storage step, fault shunt, clear fault, structured event records | NetworkSwitch, BranchTrip, BranchImpedanceChange, GeneratorTrip, LoadTrip/Change, ControlReferenceChange, SourceBusVoltageChange, PerturbState | HACDCPF event family is good but should add control reference and impedance-change semantics |
| IO/standards | Strong component IO registry, JSON, GridLAB-D/OpenDSS static text, standard profile metadata placeholders | Uses PowerSystems.jl dynamic model data and PSS/E-style benchmark fixtures | HACDCPF needs exact dynamic parameter-profile IO, not just metadata |
| Verification | C++ synthetic transient tests, GUI route tests, GridLAB-D/OpenDSS static-network checks, and local opt-in PSD trace gates for selected machines/IBRs | Extensive benchmark tests against PSS/E, PSCAD, PSAT, ANDES; residual and mass-matrix variants | HACDCPF needs to promote trace gates into full residual/mass-matrix/small-signal gates before broad parity claims |
| GUI | `暂态仿真` route, event editor, voltage/frequency/converter/generator/observer plots | Not the same GUI focus | HACDCPF GUI can become a differentiator once models are validated |

## HACDCPF Strengths To Preserve

HACDCPF should not try to become a clone of PowerSimulationsDynamics.jl. The
native strengths are important:

1. Rich hybrid component model:
   `HybridPowerSystem` includes AC buses, AC branches, generators, static
   generators, loads, flexible/asymmetric loads, shunts, storage, renewable
   generators, PV systems, external grids, transformers, regulator controls,
   switches, circuit breakers, charging stations, motors, DC buses, DC branches,
   DC loads, DC storage, DC PV arrays, DC/DC converters, VSC converters, energy
   routers, mobile storage, VPPs, microgrids, and optional three-phase AC.

2. Hybrid AC/DC converter semantics:
   `VSCConverter` already distinguishes PQ, AC PV, AC grid-forming, DC
   grid-forming, DC droop, hard vs released active-power constraints, AC angle
   reference, DC coordination groups, current limits, modulation feasibility, and
   dual-side grid-forming gates.

3. Static external verification plumbing:
   GridLAB-D and OpenDSS text IO plus the GridLAB-D comparison report are useful
   foundations for AC snapshot validation. PSD does not replace this.

4. Dynamic runtime shell:
   The dynamics module is no longer empty. It has:
   - dynamic system and network state containers,
   - `DynamicDevice` interface,
   - device stamping,
   - power-flow initialization and dynamic trimming,
   - event scheduling,
   - sampled results and CSV export,
   - GUI endpoint integration,
   - Euler/Heun/RK4/Backward Euler/trapezoidal/Rosenbrock-style integrators,
   - sparse linear solver wrapper,
   - tests for GFL/GFM, hybrid AC/DC, events, initialization residuals, implicit
     solvers, and GUI-facing output labels.

5. Three-phase distribution orientation:
   The updated GFL/GFM document and current dynamic network are phasor-domain,
   device-stamped, and compatible with unbalanced distribution studies.

These strengths should remain the platform. The missing part is dynamic model
depth and validation evidence.

## PowerSimulationsDynamics.jl Strengths To Borrow

### 1. Dynamic Model Composition

PSD generator models are composed from:

- machine,
- shaft,
- automatic voltage regulator,
- power system stabilizer,
- turbine governor.

PSD inverter models are composed from:

- DC source,
- frequency estimator,
- outer-loop control,
- inner-loop control,
- converter,
- filter,
- output current limiter.

This is the most important architectural gap. HACDCPF currently has compact
whole-device GFL/GFM classes. Those are useful, but they cannot scale cleanly to
REGC/REEC/REPCA, DERA, droop, VSM, VOC, storage, PV, wind, and vendor-like
variants without becoming monolithic.

### 2. Power-Electronic Controller Detail

PSD has explicit implementations or test coverage around:

- `KauraPLL`,
- `ReducedOrderPLL`,
- `FixedFrequency`,
- `ActiveRenewableControllerAB`,
- `ReactiveRenewableControllerAB`,
- `RECurrentControlB`,
- `AverageConverter`,
- `RenewableEnergyConverterTypeA`,
- `RenewableEnergyVoltageConverterTypeA`,
- `RLFilter`,
- `LCLFilter`,
- instantaneous, magnitude, saturation, and hybrid output current limiters,
- DERA / aggregate distributed generation,
- REGCA voltage behavior,
- droop inverter,
- virtual synchronous machine,
- virtual oscillator control.

HACDCPF GFL/GFM currently covers the correct high-level direction but not this
subcomponent-level detail.

### 3. Equilibrium Initialization Discipline

PSD documentation explicitly states that if a simulation is properly initialized,
all states should remain fixed until a perturbation is applied. That matches the
correct transient-stability interpretation: a converged power flow is necessary
but not sufficient. The pre-event point must satisfy:

```text
f(x0, y0, u0) = 0
g(x0, y0, u0) = 0
```

HACDCPF already records `dynamic_fast_dxdt_inf_norm`, which is excellent. The
next step is to make this residual a hard verification gate for cases that claim
transient equilibrium.

### 4. Residual And Mass-Matrix Formulations

PSD exposes both:

- `ResidualModel`,
- `MassMatrixModel`.

It also builds Jacobians and supports small-signal analysis through block
reduction:

```text
J_red = f_x - f_y * inv(g_y) * g_x
```

HACDCPF currently integrates dynamic states while solving the network
algebraically in a partitioned manner. That is acceptable for a first dynamic
engine, but it is not enough for PSD-level DAE analysis, stiffness handling, or
small-signal verification.

### 5. Benchmark Verification Culture

PSD has tests and fixtures for:

- PSS/E: GENCLS, GENROU, GENROE, GENSAL, GENSAE, TGOV1, GAST, HYGOV, SEXS,
  EXAC1, EXST1, SCRX, ESST1A, ST6B, ST8C, STAB1, PSS2A/B/C, DERA, RENA,
  REGCA voltage, load models, and others,
- PSCAD: grid-following, droop inverter, VSM, dynamic line cases,
- PSAT: classic generator benchmarks,
- ANDES: eigenvalue comparisons.

The critical point is that PSD does not just implement models. It repeatedly
checks:

- initial condition residuals,
- small-signal eigenvalues,
- time-domain trajectories,
- residual-model and mass-matrix-model consistency.

HACDCPF needs the same style of evidence before making industry-grade claims.

## Detailed Gap Analysis

### A. Rich And Canonical Dynamic Model Gap

Current HACDCPF projection maps many static devices into generic dynamic
surrogates:

- static generators, PV systems, and renewable generators become generic GFL
  current-source devices,
- storage can become GFM plus battery SOC dynamics,
- VSC role resolution maps AC grid-forming to GFM, DC grid-forming to DC voltage
  source, otherwise GFL,
- DC PV/static generation is represented through native simplified dynamics.

This is useful for broad coverage, but it loses standard dynamic model identity.
A PV plant represented by `REGC_A + REEC_B + REPCA_A` should not become only
`REGC_REEC_GFL_Subset`; its profile, parameter set, block flags, limiters, and
initialization order must survive rich-to-canonical projection.

Required change:

- Add dynamic model profile records to rich devices and canonical devices.
- Preserve `standard`, `model_name`, `parameter_set`, units, base MVA, base kV,
  and source file identity.
- Let projection generate canonical dynamic subcomponents with provenance back
  to the rich component.
- Treat generic HACDCPF GFL/GFM models as fallback profiles, not as the only
  dynamic representation.

### B. GFL Inverter Gap

Current HACDCPF GFL is a compact positive-sequence current-source style model
with:

- PLL gains,
- active/reactive power references,
- first-order response,
- power filtering,
- optional current limit,
- optional reactive current priority,
- stabilizing admittance,
- frequency-watt and volt-var droop fields,
- optional dynamic DC link.

PSD GFL models include more detailed control blocks:

- frequency estimator variants,
- active renewable controller flags and deadbands,
- reactive renewable controller modes,
- inner current controller,
- REGCA converter dynamics,
- LVPL,
- current ramp limits,
- high-voltage reactive current behavior,
- output current limiters,
- RL/LCL filter choices.

Required change:

```text
DynamicInverter
  DCSideModel
  FrequencyEstimatorModel
  OuterControlModel
  InnerControlModel
  ConverterModel
  ACFilterModel
  OutputCurrentLimiterModel
```

Then implement GFL profiles in this order:

1. Existing HACDCPF GFL as `HACDCPF_GFL_Subset`.
2. `KauraPLL` and `ReducedOrderPLL`.
3. `REGC_A` converter subset with LVPL and ramp limits.
4. `REEC_A/B` electrical controls.
5. `REPCA_A` plant controller.
6. `DERA1` / aggregate distributed generation profile.
7. IEEE 1547 volt-var, frequency-watt, ride-through profile fields.

### C. GFM Inverter Gap

Current HACDCPF GFM is a Norton voltage-source droop model with:

- internal voltage source,
- virtual impedance,
- P/f and Q/V droop,
- power filter,
- voltage and overload controller fields,
- current limit,
- optional dynamic DC link.

This is a good feeder-scale GFM base model. It is not yet enough for PSD-style
GFM comparisons because PSD has separate droop inverter, VSM, and VOC
benchmarks.

Required GFM profiles:

1. `HACDCPF_GFM_NortonDroop` compatibility profile.
2. Droop grid-forming inverter profile compatible with PSD test case 23.
3. Virtual synchronous machine profile compatible with PSD test case 08.
4. Virtual oscillator control profile compatible with PSD test case 44.
5. Matching-control or dispatchable virtual oscillator profile, if needed
   later.
6. DC-link and energy-buffer coupling rules for hybrid AC/DC grid-forming.

### D. Conventional Generator Gap

Current HACDCPF has:

- `ClassicalMachine`,
- `OneDOneQMachine`,
- `GENROU`,
- `GENROE`,
- `GENSAL`,
- `GENSAE`,
- simplified `SEXS`, `IEEET1`, `TGOV1`, `IEEEG1`, and `PSS1A` profile/runtime
  coverage.

The machine trace gates are compare-limited, not full PSD test passes. They
compare selected state trajectories, but they do not yet reproduce PSD's full
initialization, `ResidualModel`, `MassMatrixModel`, PSAT/PSS/E benchmark, and
small-signal contracts.

PSD has many industry-style machine and controller families, including:

- GENROU, GENROE, GENSAL, GENSAE-style round/salient machine coverage,
- Sauer-Pai, Marconato, Anderson-Fouad, one-d/one-q and simplified models,
- SEXS, SCRX, EXST1, EXAC1, ESST1A, ST6B, ST8C,
- IEEEST, STAB1, PSS2A, PSS2B, PSS2C,
- TGOV1, HYGOV, GAST, DEGOV, DEGOV1, PIDGOV, WPIDHY and related turbine
  governors.

Required change:

- Split `SynchronousMachine`, `Governor`, and `Exciter` into composable
  machine/shaft/exciter/PSS/governor components.
- Add fuel-type templates for thermal, hydro, nuclear, gas, and diesel, but
  store the actual dynamic profile explicitly rather than inferring it only from
  fuel type.
- Build benchmark cases one model family at a time.
- Treat a PSD component row as passing only when the accepted trace signals and
  the agreed formulation gates pass.

Machine-first implementation order:

1. `SimpleMarconatoMachine`, then `MarconatoMachine`.
2. `SimpleAFMachine`, then `AndersonFouadMachine`.
3. `SauerPaiMachine`.
4. `FiveMassShaft`.
5. Controller-by-controller expansion: first `SEXS`, `IEEET1`, and `TGOV1`
   hard trace gates; then the missing AVR, governor, and PSS families selected
   by the benchmark set.

### E. Initialization Gap

HACDCPF currently performs:

1. optional static power flow,
2. network voltage initialization,
3. device initialization,
4. repeated network solve plus per-device `trimToNetworkEquilibrium`,
5. residual reporting.

This is a solid first implementation. It still differs from PSD because PSD
performs per-component initialization and then solves a full nonlinear
system-wide equilibrium.

Required change:

- Define a full dynamic residual vector:

```text
R(z) = [g(x, y, u); f_fast(x, y, u)]
```

- Solve it after per-device initialization.
- Keep slow energy states such as SOC out of the fast equilibrium residual when
  appropriate.
- Fail or warn loudly when no-event drift exceeds tolerance.
- Expose GUI diagnostics:
  - PF residual,
  - dynamic residual,
  - worst device residual,
  - worst state name,
  - whether the case is equilibrium-certified.

### F. Solver And Analysis Gap

HACDCPF solver status:

- Partitioned Euler, Heun, RK4.
- Backward Euler Newton.
- Trapezoidal Newton.
- Rosenbrock-like Euler.
- Algebraic AC/DC sparse network solves.
- Numerical state Jacobian for implicit stepping.
- Sparse linear solver wrapper with Eigen baseline and optional SuiteSparse
  macros.

PSD solver and analysis status:

- residual DAE formulation,
- mass-matrix DAE formulation,
- simultaneous device-network solution,
- automatic differentiation Jacobians,
- reduced Jacobian and small-signal analysis,
- participation-factor reporting,
- broad solver compatibility through SciML.

Required change:

- Add `ResidualDynamicModel` and `MassMatrixDynamicModel` in C++.
- Include both differential and algebraic variables in the residual layout.
- Add sparse numerical Jacobian first; add analytic blocks progressively.
- Add small-signal analysis:
  - build `g_y`, `g_x`, `f_y`, `f_x`,
  - compute `J_red`,
  - report eigenvalues, damping ratios, frequencies, and participation factors.
- Keep the current partitioned solvers for fast distribution runs, but make
  claim-grade studies use the residual/mass-matrix path.

### G. IO And Industry-Standards Gap

HACDCPF already has an important start:

- component IO coverage registry,
- standard profile metadata families,
- internal JSON,
- OpenDSS text conversion,
- GridLAB-D text conversion,
- GridLAB-D AC snapshot verification gate.

The missing part is exact dynamic model parameter exchange. Industry-standard
dynamic validation needs model parameter sets, not only device labels.

Required dynamic IO layers:

1. Internal JSON dynamic profiles:
   - `dynamic_model.standard`
   - `dynamic_model.model_name`
   - `dynamic_model.parameter_set`
   - `dynamic_model.components[]`
   - unit/base metadata
   - original source file and record ID

2. PSS/E DYR import/export subset:
   - GENCLS, GENROU, GENROE, GENSAL, GENSAE,
   - TGOV1, HYGOV, GAST, SEXS, EXAC1, EXST1,
   - REGC_A, REEC_A/B, REPCA_A, DERA1.

3. CIM dynamic profile mapping:
   - keep this as a long-term exchange target, because CIM dynamic profile
     coverage and vendor interpretation can be uneven.

4. OpenDSS/GridLAB-D:
   - keep them primarily for distribution network algebraic checks,
   - use boundary-injection snapshots for DER controls until exact dynamic
     object mapping exists.

### H. Verification Gap

Current HACDCPF tests prove that the new transient framework runs and exposes
the expected surfaces. They do not yet prove industry-equivalent dynamic
behavior.

Required verification ladder:

| Level | Target | Reference | Required evidence |
|---|---|---|---|
| L0 | Static AC network | GridLAB-D/OpenDSS | Bus voltage and branch-flow match within tolerance |
| L1 | Single dynamic component | analytic or PSD fixture | initial residual, step response, limits |
| L2 | Single device plus infinite bus | PSD, PSCAD, PSS/E CSV | initial condition, time trajectory |
| L3 | Small system | PSD/PSS/E/PSCAD | time trajectory plus eigenvalues |
| L4 | Medium system | PSD and native C++ | residual/mass-matrix consistency, runtime, stability |
| L5 | Large hybrid system | native plus snapshot external checks | claim scoped to supported features |

Every model should have a claim ledger:

```text
model: REGC_A
standard_family: NERC
implemented_blocks: [...]
omitted_blocks: [...]
validated_against: PSS/E CSV, PSD local run
initial_residual_tolerance: ...
trajectory_tolerance: ...
small_signal_tolerance: ...
claim: supported / diagnostic / experimental
```

## Recommended Roadmap

### Phase 0: Freeze The Claim Boundary

Do this before adding many new equations.

- Keep the current statement: HACDCPF supports a compact phasor-domain hybrid
  AC/DC transient simulation with generic GFL/GFM and dynamic initialization
  diagnostics.
- Do not claim equivalence to PSD for dynamic controls yet.
- Do not claim full GridLAB-D AC distribution equivalence beyond the gated
  static snapshot scope.
- Add a dynamic model claim ledger file or generated report.

### Phase 1: Dynamic Profile Schema And IO

Add explicit dynamic profiles to rich and canonical components:

```text
include/hacdcpf/dynamics/DynamicModelProfile.hpp
include/hacdcpf/io/dynamic_model_io.hpp
src/io/dynamic_model_io.cpp
```

Key requirements:

- preserve standard/model/parameter-set identity,
- track source file and source record,
- validate units and base conversions,
- expose profiles in GUI results and JSON exports,
- keep current generic models as fallback profiles.

### Phase 2: PSD-Style Inverter Meta-Model

Introduce composable inverter subcomponents:

```text
include/hacdcpf/dynamics/devices/inverter/DCSideModel.hpp
include/hacdcpf/dynamics/devices/inverter/FrequencyEstimatorModel.hpp
include/hacdcpf/dynamics/devices/inverter/OuterControlModel.hpp
include/hacdcpf/dynamics/devices/inverter/InnerControlModel.hpp
include/hacdcpf/dynamics/devices/inverter/ConverterModel.hpp
include/hacdcpf/dynamics/devices/inverter/ACFilterModel.hpp
include/hacdcpf/dynamics/devices/inverter/OutputCurrentLimiterModel.hpp
include/hacdcpf/dynamics/devices/inverter/DynamicInverter.hpp
```

Start by wrapping the existing GFL/GFM behavior as subcomponent-compatible
profiles. Then add PSD-compatible blocks one at a time.

### Phase 3: Industry GFL And DER Models

Implement, initialize, and verify:

- Kaura PLL,
- reduced-order PLL,
- fixed-frequency estimator,
- REGC_A converter subset,
- REEC_A/B electrical controls,
- REPCA_A plant controller,
- DERA1 aggregate DER,
- LVPL,
- active/reactive current ramp limits,
- P-priority/Q-priority limiting,
- IEEE 1547 volt-var and frequency-watt functions.

Verification targets:

- PSD `test_case29_renewablemodels.jl`,
- PSD `test_case42_dera.jl`,
- PSD `test_case43_regca_voltage.jl`,
- PSD `test_case_gridfollowing.jl`,
- PSS/E benchmark CSVs where available.

### Phase 4: GFM Model Family

Implement and verify:

- droop inverter,
- virtual synchronous machine,
- virtual oscillator control,
- current-limited GFM behavior,
- hybrid AC/DC DC-link and energy-buffer coupling,
- islanding and reference-frequency handling.

Verification targets:

- PSD `test_case_droopinverter.jl`,
- PSD `test_case_VirtualSynchMachine.jl`,
- PSD `test_case44_voc.jl`,
- PSCAD benchmark CSVs where available.

### Phase 5: Generator Dynamic Library

Implement industry generator families gradually:

- GENCLS/classical compatibility,
- GENROU,
- GENROE,
- GENSAL,
- GENSAE,
- SEXS, SCRX, EXST1, EXAC1, ESST1A, ST6B, ST8C,
- TGOV1, HYGOV, GAST, DEGOV, DEGOV1, PIDGOV, WPIDHY,
- IEEEST, STAB1, PSS2A, PSS2B, PSS2C.

Verification targets:

- PSD generator test cases,
- PSS/E benchmark CSVs,
- PSAT classic cases,
- ANDES eigenvalue fixtures.

### Phase 6: Residual/Mass-Matrix DAE And Small Signal

Add:

- full residual layout,
- mass matrix layout,
- sparse Jacobian,
- reduced Jacobian,
- eigenvalue and participation-factor API,
- GUI small-signal panel,
- benchmark comparisons against PSD.

The existing partitioned solvers should remain available for quick feeder
studies, but the residual/mass-matrix path should become the claim-grade route.

### Phase 7: GUI And Verification Dashboard

Extend `暂态仿真` so the GUI communicates model credibility, not just waveforms:

- dynamic model profile table,
- initial equilibrium certificate,
- worst residual per device,
- event timeline with exact parameters,
- observer/meter placement on buses and devices,
- overlay imported benchmark traces,
- show whether each plotted device is supported, diagnostic, or experimental,
- provide export of time series plus model claim ledger.

## Proposed First Implementation Slice

The current highest-value first slice is machine-first, because the local PSD
suite exposes clear isolated machine tests and the selected comparison ladder
starts there:

1. Keep Julia/PSD in `tools/psd_validation` and opt-in tests only.
2. Add a PSD trace exporter case for `SimpleMarconatoMachine`.
3. Implement the HACDCPF `SimpleMarconatoMachine` runtime and initialization
   path.
4. Gate it against PSD Test 03 with agreed signals: `delta`, `omega`, `eq_p`,
   and `ed_p`.
5. Current stop point: the Test 03 exporter and local smoke gate exist, but the
   opt-in PSD comparison fails `generator-103-1:delta_rad_relative` after the
   BUS 1-BUS 3 trip (rms about 0.0935 rad, max about 0.2119 rad). Do not proceed
   to `MarconatoMachine` until this drift is resolved or explicitly accepted.

This gives a disciplined proof path:

```text
profile IO -> canonical projection -> initialization -> time simulation
-> PSD/PSS-E comparison -> GUI-visible claim
```

After `SimpleMarconatoMachine`, continue one model at a time:
`MarconatoMachine`, `SimpleAFMachine`, `AndersonFouadMachine`,
`SauerPaiMachine`, and `FiveMassShaft`. Controller work should follow the same
pattern: one PSD test, one controller, one exported trace set, one stop/go
decision.

## Claim Wording

Current defensible claim:

> HACDCPF provides a hybrid AC/DC distribution-system transient simulation
> framework with compact phasor-domain synchronous-machine, GFL, GFM, VSC,
> DC/DC, battery, PV, load, and relay models, power-flow-based initialization,
> dynamic trimming diagnostics, event replay, GUI visualization, and static
> AC-snapshot cross-checks against GridLAB-D/OpenDSS within declared scope.

Current non-defensible claim:

> HACDCPF is equivalent to PowerSimulationsDynamics.jl for inverter-based
> resource dynamics.

Also non-defensible today:

> HACDCPF passes the PowerSimulationsDynamics.jl machine and IBR component test
> suite.

Future claim after the roadmap:

> HACDCPF supports named IEEE/NERC/WECC/IEEE-1547 dynamic model profiles with
> documented omissions and benchmark evidence against PSD/PSS-E/PSCAD/ANDES for
> each claimed profile, while retaining native hybrid AC/DC and three-phase
> distribution capabilities.

That future claim should be made per profile and per verification scope, not as
a blanket package-level statement.

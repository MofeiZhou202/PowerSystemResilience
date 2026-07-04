# HACDCPF vs PowerSimulationsDynamics.jl Model Crosswalk

Generated from `psd_hacdcpf_model_crosswalk.json`. Crosswalk updated: 2026-07-04.

This is a model-by-model and controller-by-controller comparison. It is not an equivalence claim; it tells the validation harness which pairs can be compared today and which pairs should only preserve metadata.

## Summary

- Rows: 43
- exact-or-close: 2
- missing: 19
- profile-only: 3
- supported-subset: 19

## Status Legend

- `exact-or-close`: The named HACDCPF model is intended to be directly comparable for focused traces, subject to parameter and initialization matching.
- `supported-subset`: HACDCPF has a named model or controller that captures the main behavior but omits PSD states, limiters, saturation, or formulation details.
- `profile-only`: HACDCPF can preserve the profile identity in dynamic_model metadata, but the transient runtime does not yet implement equivalent behavior.
- `missing`: No HACDCPF runtime counterpart is currently present.

## Comparison Levels

- `L0`: Catalog/profile identity only.
- `L1`: Single component or single controller trace.
- `L2`: Composed device trace.
- `L3`: Small network/system trace.
- `L4`: Small-signal or residual/mass-matrix parity.

## Crosswalk

| Domain | PSD Slot | PSD Model / Controller | HACDCPF Slot | HACDCPF Model | Status | Level | Trace Case | Catalog | Notes |
|---|---|---|---|---|---|---|---|---|---|
| generator | machine | BaseMachine / GENCLS | machine | ClassicalMachine | supported-subset | L1 | genrou:generator-102-1:omega_pu | yes | Comparable as a classical voltage-source swing subset; PSD still carries explicit machine/shaft composition. |
| generator | machine | RoundRotorQuadratic / GENROU | machine | GENROU | supported-subset | L2 | genrou:generator-102-1:delta_deg | yes | Parameter profile and trace gate exist; HACDCPF remains a reduced runtime compared with PSD's full positive-sequence DAE implementation. |
| generator | machine | RoundRotorExponential / GENROE | machine | GENROU | profile-only | L0 |  | yes | Can preserve identity as source metadata, but exponential saturation and exact GENROE behavior are not implemented as a separate HACDCPF model. |
| generator | machine | SalientPoleQuadratic / GENSAL | machine |  | missing | L0 |  | n/a | Add a salient-pole machine family before claiming model parity. |
| generator | machine | SalientPoleExponential / GENSAE | machine |  | missing | L0 |  | n/a | Requires salient-pole states plus exponential saturation. |
| generator | machine | OneDOneQMachine | machine | OneDOneQMachine | supported-subset | L1 | onedoneq:generator-102-1:eq_p | yes | Named two-state OneDOneQ runtime path is implemented from the local PSD equations and initialization contract. Still lacks PSD ResidualModel/MassMatrixModel, eigenvalue parity, and an exact salient-machine algebraic network stamp. |
| generator | machine | MarconatoMachine / SimpleMarconatoMachine | machine |  | missing | L0 |  | n/a | Needs a dedicated machine implementation and initialization path. |
| generator | machine | SauerPaiMachine | machine |  | missing | L0 |  | n/a | No named HACDCPF Sauer-Pai machine counterpart. |
| generator | machine | AndersonFouadMachine / SimpleAFMachine | machine |  | missing | L0 |  | n/a | No named HACDCPF Anderson-Fouad machine counterpart. |
| generator | shaft | SingleMass | machine | ClassicalMachine | supported-subset | L1 | genrou:generator-102-1:omega_pu | yes | H and D are consumed by HACDCPF machine parameters; PSD models shaft as an explicit subcomponent. |
| generator | shaft | FiveMassShaft | shaft |  | missing | L0 |  | n/a | Requires multi-mass turbine-generator shaft states. |
| generator | avr | SEXS | exciter | SEXS | supported-subset | L1 |  | yes | Catalog keys are wired; add a focused PSD trace gate for field-voltage response. |
| generator | avr | IEEET1 / AVRTypeI | exciter | IEEET1 | supported-subset | L1 |  | yes | Equivalent profile exists, but PSD's full AVR state and limiter details still need trace validation. |
| generator | avr | ESAC1A / EXAC1 / EXST1 / SCRX / ESST1A / ST6B / ST8C / CSVGN1 | exciter |  | missing | L0 |  | n/a | Preserve imported names in metadata until the AVR library is expanded. |
| generator | governor | SteamTurbineGov1 / TGOV1 | governor | TGOV1 | supported-subset | L1 |  | yes | Catalog keys are wired; compare mechanical-power response under a speed or branch-trip perturbation. |
| generator | governor | IEEETurbineGov1 / IEEEG1 | governor | IEEEG1 | supported-subset | L1 |  | yes | HACDCPF has the named profile and simplified reheat parameters. |
| generator | governor | GasTG / GAST | governor |  | missing | L0 |  | n/a | Gas turbine governor runtime model is not present. |
| generator | governor | HydroTurbineGov / HYGOV | governor |  | missing | L0 |  | n/a | Hydro governor runtime model is not present. |
| generator | governor | TGSimple / TGTypeI / TGTypeII / DEGOV / DEGOV1 / PIDGOV / WPIDHY | governor |  | missing | L0 |  | n/a | Use metadata preservation only until additional governor families are implemented. |
| generator | pss | PSSSimple / PSS1A | pss | PSS1A | supported-subset | L1 |  | yes | Basic stabilizer profile exists; requires signal-by-signal validation. |
| generator | pss | STAB1 | pss |  | missing | L0 |  | n/a | PSD test 41 coverage has no HACDCPF runtime counterpart. |
| generator | pss | PSS2A / PSS2B / PSS2C | pss |  | missing | L0 |  | n/a | Multi-input PSS2 families are not implemented. |
| inverter | dc_source | FixedDCSource | dc_source | REGC_REEC_GFL_Subset | supported-subset | L2 | test24:generator-102-1:p_oc | yes | HACDCPF GFL/GFM models can hold a fixed DC-side assumption but do not expose PSD's DC source as a separate block. |
| inverter | dc_source | PeriodicVariableSource | dc_source |  | missing | L0 |  | n/a | Requires a dynamic DC source block or boundary trace injection. |
| inverter | frequency_estimator | ReducedOrderPLL | pll | ReducedOrderPLL | supported-subset | L2 | test24:generator-102-1:p_oc | yes | External PSD comparison hook exists for the reduced-order PLL grid-following case. |
| inverter | frequency_estimator | KauraPLL | pll | KauraPLL | supported-subset | L2 | test51:generator-102-1:p_oc | yes | External PSD comparison hook exists for the Kaura PLL grid-following case. |
| inverter | frequency_estimator | FixedFrequency | pll | FixedFrequency | supported-subset | L1 |  | yes | Supported as a profile/controller choice; add a no-PLL trace gate. |
| inverter | outer_control | ActiveRenewableControllerAB + ReactiveRenewableControllerAB | converter | REGC_REEC_GFL_Subset | supported-subset | L2 | test24/test51:p_oc | yes | HACDCPF has a generic REGC/REEC subset; many PSD flags, deadbands, and plant-control paths remain unrepresented. |
| inverter | inner_control | RECurrentControlB | converter | REGC_REEC_GFL_Subset | supported-subset | L2 | test24/test51:p_oc | yes | Current response and limit behavior are simplified in HACDCPF. |
| inverter | converter | AverageConverter | converter | REGC_REEC_GFL_Subset | supported-subset | L2 | test24/test51:p_oc | yes | PSD converter block is folded into the HACDCPF compact inverter model. |
| inverter | filter | RLFilter | filter |  | profile-only | L0 |  | n/a | HACDCPF uses algebraic Norton/virtual impedance behavior; no differential RL filter block yet. |
| inverter | filter | LCLFilter / LCFilter | filter |  | missing | L0 |  | n/a | Needs differential filter states and mass-matrix participation. |
| inverter | limiter | InstantaneousOutputCurrentLimiter / MagnitudeOutputCurrentLimiter / SaturationOutputCurrentLimiter / PriorityOutputCurrentLimiter / HybridOutputCurrentLimiter | limiter | REGC_REEC_GFL_Subset | supported-subset | L1 |  | yes | HACDCPF exposes a current_limit_pu scalar, not PSD's selectable limiter families. |
| inverter | outer_control | ActivePowerDroop + ReactivePowerDroop | converter | GridFormingNortonDroop | supported-subset | L2 |  | yes | Comparable at droop trend level; PSD has explicit block composition and filter/controller dynamics. |
| inverter | outer_control | VirtualInertia / VSM | converter |  | missing | L0 |  | n/a | VSM is a priority gap for grid-forming comparison. |
| inverter | outer_control | ActiveVirtualOscillator + ReactiveVirtualOscillator / VOC | converter |  | missing | L0 |  | n/a | VOC requires a separate oscillator-based GFM runtime model. |
| load | load | StandardLoad transformed to constant power | load | ConstantPower | exact-or-close | L1 | zip_constant_power:bus102:voltage_mag | yes | Existing external PSD comparison checks constant-power load voltage deviation. |
| load | load | StandardLoad transformed to constant impedance | load | ConstantImpedance | exact-or-close | L1 |  | yes | Static admittance behavior is directly comparable after base and sign conventions are aligned. |
| load | load | ZIPLoad | load | ZIP | supported-subset | L1 | zip_constant_power:bus103:voltage_mag | yes | HACDCPF has ZIP model selection; parameter-level ZIP coefficient import should be added for richer cases. |
| load | load | ExponentialLoad / PowerLoad | load |  | missing | L0 |  | n/a | Needs explicit dynamic load families beyond ZIP/current/impedance/power. |
| network | branch | DynamicBranch / dynamic line | branch |  | missing | L0 |  | n/a | HACDCPF transient branch model is algebraic; no PSD-style dynamic line state model yet. |
| dae | simulation_formulation | ResidualModel | solver |  | profile-only | L4 |  | n/a | HACDCPF has implicit integrators and diagnostics but not a PSD-equivalent full residual DAE API. |
| dae | simulation_formulation | MassMatrixModel | solver |  | missing | L4 |  | n/a | Mass-matrix model and sparse Jacobian reduction remain future work. |

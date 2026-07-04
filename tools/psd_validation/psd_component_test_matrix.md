# PSD Machine/IBR Component Test Matrix

Generated from `psd_component_test_matrix.json`. Matrix updated: 2026-07-04.

This is the first-pass gate for comparing PowerSimulationsDynamics.jl transmission-dynamics component tests against HACDCPF. It answers whether each PSD machine or IBR component test group can be compared now.

## Verdict

- Can pass all listed component test groups now: `false`
- Rows: 39
- Blocked rows: 30
- blocked-missing-controller: 9
- blocked-missing-formulation: 3
- blocked-missing-model: 15
- compare-limited: 9
- metadata-only: 3

## Status Legend

- `compare-limited`: HACDCPF has a current model and an existing or near-existing trace hook, but it does not cover PSD's full residual/mass-matrix/small-signal test contract.
- `blocked-missing-model`: The PSD test's primary model is not implemented in the HACDCPF transient runtime.
- `blocked-missing-controller`: The base machine or IBR may exist, but a required PSD controller/block is not implemented.
- `blocked-missing-formulation`: The dynamic behavior may be partially present, but HACDCPF does not yet expose PSD-equivalent residual/mass-matrix/small-signal parity.
- `metadata-only`: The model identity can be preserved for IO, but behavior should not be compared yet.

## Matrix

| Group | PSD Tests | PSD Components | HACDCPF Candidate | Status | First Blocker | Next Action |
|---|---|---|---|---|---|---|
| machine | Test 01 OMIB | BaseMachine / GENCLS, SingleMass | ClassicalMachine | compare-limited | No PSD-equivalent ResidualModel/MassMatrixModel/small-signal gate in HACDCPF. | Use as the first machine smoke trace after defining accepted signals and tolerances. |
| machine | Test 02 | OneDOneQMachine, SingleMass | OneDOneQMachine | compare-limited | Local PSD Test 02 delta/omega/eq_p/ed_p trace gates pass over 0-2s, but HACDCPF does not yet provide PSD-equivalent ResidualModel, MassMatrixModel, eigenvalue, or PSAT trace parity. | Implement the residual/mass-matrix/small-signal parity gate before claiming a full PSD Test 02 pass. |
| machine-ibr | Test 10, Test 11 | OneDOneQMachine, VSM inverter, static/dynamic branches | OneDOneQMachine | blocked-missing-model | VSM inverter is not implemented; Test 11 also needs dynamic branch behavior. | Do not attempt these tests until VSM and dynamic branch models exist. |
| machine-controller | Test 13 | OneDOneQMachine with AVRTypeII / AVRSimple and TGTypeI variations | OneDOneQMachine | blocked-missing-controller | AVRTypeII, AVRSimple, and TGTypeI are not implemented as PSD-equivalent controllers. | Run only after the OneDOneQ base trace passes and the required controller blocks are mapped. |
| machine | Test 03 | SimpleMarconatoMachine |  | blocked-missing-model | No SimpleMarconato runtime model. | Defer until the basic machine ladder is complete. |
| machine | Test 04, Test 25 | MarconatoMachine, dynamic-line benchmark |  | blocked-missing-model | No Marconato runtime model; Test 25 also needs dynamic branch behavior. | Do not attempt trace parity until Marconato and dynamic branches exist. |
| machine | Test 05 | SimpleAFMachine |  | blocked-missing-model | No Simple Anderson-Fouad runtime model. | Defer behind GENROU/GENSAL coverage. |
| machine | Test 06 | AndersonFouadMachine |  | blocked-missing-model | No Anderson-Fouad runtime model. | Defer behind GENROU/GENSAL coverage. |
| shaft | Test 07 | FiveMassShaft |  | blocked-missing-model | No multi-mass shaft model. | Implement after single-mass machine parity is accepted. |
| machine | Test 12 | Two BaseMachine devices with TGTypeII | ClassicalMachine | blocked-missing-controller | TGTypeII is not implemented; full PSD formulation gates are also missing. | Use only after TGTypeII or an accepted surrogate is defined. |
| machine | Test 15 | GENROU variants: normal, no saturation, high saturation | GENROU | compare-limited | Current comparison is a reduced trace gate, not PSD's full initialization, eigenvalue, ResidualModel, and MassMatrixModel contract. | Promote GENROU to the first full machine comparison if you choose machine-first work. |
| machine | Test 16 | GENROE variants | GENROU metadata surrogate | metadata-only | No distinct GENROE exponential-saturation runtime model. | Implement GENROE or explicitly decide to treat it as out of first release. |
| machine | Test 18 | GENSAL |  | blocked-missing-model | No salient-pole machine runtime model. | Required before claiming broad synchronous-machine PSD parity. |
| machine | Test 19 | GENSAE |  | blocked-missing-model | No salient-pole exponential machine runtime model. | Build after GENSAL if salient-pole machines are prioritized. |
| machine | Test 35 | Multi-generator PSSE case | ClassicalMachine / GENROU subsets | blocked-missing-formulation | System-level multi-machine PSD gates depend on missing model/controller coverage and full DAE parity. | Defer until single-device machine tests pass. |
| machine | Test 36 | GENROU + SEXS + TGOV1 eigenvalues vs ANDES | GENROU + SEXS + TGOV1 | blocked-missing-formulation | No PSD-equivalent small-signal/eigenvalue API in HACDCPF. | Do not claim this until reduced Jacobian/eigenvalue tooling exists. |
| machine | Test 45 | SauerPaiMachine |  | blocked-missing-model | No Sauer-Pai runtime model. | Defer behind GENROU/GENSAL unless explicitly prioritized. |
| machine-controller | Test 17 | GENROU + AVRTypeI | GENROU + IEEET1 | compare-limited | IEEET1 profile exists, but full AVR state/limiter parity and PSD DAE gates are missing. | Candidate after GENROU base trace is accepted. |
| machine-controller | Test 20 | ESAC1A |  | blocked-missing-controller | No ESAC1A runtime model. | Add to AVR library only after selecting AVR priority. |
| machine-controller | Test 21 | GAST |  | blocked-missing-controller | No gas turbine governor runtime model. | Add if gas-governor parity matters for the first benchmark set. |
| machine-controller | Test 22 | TGOV1 | TGOV1 | compare-limited | TGOV1 exists as a simplified controller; full mechanical-power trace/eigenvalue parity is not yet wired. | Candidate for the first controller-by-controller comparison. |
| machine-controller | Test 26 | SEXS | SEXS | compare-limited | SEXS exists, but full field-voltage trace parity is not yet a hard gate. | Candidate for the first AVR comparison. |
| machine-controller | Test 30 | IEEEST / STAB1-like stabilizer |  | blocked-missing-controller | No IEEEST/STAB1 runtime model. | Defer behind PSS1A/SEXS/TGOV1. |
| machine-controller | Test 31 | HYGOV |  | blocked-missing-controller | No hydro governor runtime model. | Add only if hydro governor parity is selected. |
| machine-controller | Test 39, Test 40, Test 47, Test 55, Test 59, Test 61 | EXST1, EXAC1, SCRX, ESST1A, ST6B, ST8C |  | blocked-missing-controller | These AVR/exciter families are not implemented. | This is a model-library expansion decision, not a tolerance issue. |
| machine-controller | Test 41, Test 52, Test 53, Test 54 | STAB1, PSS2A, PSS2B, PSS2C | PSS1A only | blocked-missing-controller | PSD's stabilizer families are broader than HACDCPF's PSS1A subset. | Pick PSS1A first or expand to STAB1/PSS2 families. |
| machine-controller | Test 57, Test 58, Test 60, Test 62, Test 63 | DEGOV, PIDGOV, WPIDHY, TGSimple, DEGOV1 |  | blocked-missing-controller | These governor families are not implemented. | Defer unless governor breadth is the first priority. |
| ibr | Test 08 | VSM inverter |  | blocked-missing-model | No VSM grid-forming inverter runtime model. | Required before claiming PSD-level GFM IBR parity. |
| ibr | Test 09, Test 10, Test 11, Test 27 | VSM plus OneDOneQ machine / source-bus perturbations / dynamic branches |  | blocked-missing-model | VSM is missing; dynamic branch behavior is also missing for Test 11. OneDOneQ now exists as a named HACDCPF machine, but the combined VSM-machine tests are still blocked. | Do not run as a first gate until VSM and dynamic branch models exist and the simpler OneDOneQ base trace passes. |
| ibr | Test 14 | Inverter reference case | GridFormingNortonDroop subset | blocked-missing-formulation | Reference-device semantics and PSD DAE parity are not aligned. | Defer behind single-inverter GFM droop comparison. |
| ibr | Test 23 | Droop grid-forming inverter | GridFormingNortonDroop | compare-limited | HACDCPF droop is a compact Norton model, not PSD's full composed inverter/filter/controller stack. | Candidate first GFM comparison if you accept subset parity. |
| ibr | Test 24 | Grid-following inverter with ReducedOrderPLL | REGC_REEC_GFL_Subset + ReducedOrderPLL | compare-limited | Existing hook compares selected active-power state only; PSD full eigenvalue and DAE gates remain missing. | This is the strongest current IBR comparison candidate. |
| ibr | Test 28 | PeriodicVariableSource |  | blocked-missing-model | No periodic DC/source model. | Defer unless source models are prioritized. |
| ibr | Test 29 | RENA / REPCA + REECB + REGCA | REGC_REEC_GFL_Subset | metadata-only | HACDCPF subset lacks many PSD RENA flags, limiters, and plant-control paths. | Decide whether to implement NERC renewable stack before testing RENA cases. |
| ibr | Test 42 | DERA aggregate distributed generation |  | blocked-missing-model | No AggregateDistributedGenerationA/DERA runtime model. | Separate DERA from generic GFL inverter work; it is its own IBR family. |
| ibr | Test 43 | REGCA voltage converter | REGC_REEC_GFL_Subset metadata | metadata-only | No RenewableEnergyVoltageConverterTypeA equivalent. | Add after basic REGCA/REEC active-power traces pass. |
| ibr | Test 44 | VOC grid-forming inverter |  | blocked-missing-model | No virtual oscillator control runtime model. | Required only if VOC is in the first GFM target set. |
| ibr | Test 51 | Grid-following inverter with KauraPLL | REGC_REEC_GFL_Subset + KauraPLL | compare-limited | Existing hook compares selected behavior only; full PSD eigenvalue and DAE gates remain missing. | Pair with Test 24 for the first GFL comparison set. |
| dynamic-injection | Test 49 | CSVGN1 |  | blocked-missing-model | No CSVGN1 dynamic injector model. | Defer unless this device is explicitly in scope. |

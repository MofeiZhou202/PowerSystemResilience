# Transient Cross-Engine Disturbance Matrix

Generated: `2026-07-12T06:01:00.975273+00:00`

## Results

| Case | Domain | Balance | Events | Native | Min AC pu | Min DC pu | Frequency nadir Hz | GridLAB-D scope |
|---|---|---|---:|---|---:|---:|---:|---|
| `no_event_hybrid` | hybrid_acdc | balanced | 0 | PASS | 0.991368 | 0.999498 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `ac_load_step` | ac | balanced | 1 | PASS | 0.989237 | 0.999498 | 49.998255 | sampled_ac_stage_numeric |
| `ac_branch_trip_close` | ac | balanced | 2 | PASS | 0.986679 | 0.999498 | 50.000000 | sampled_ac_stage_numeric |
| `ac_branch_impedance_step` | ac | balanced | 1 | PASS | 0.990604 | 0.999498 | 50.000000 | native_only_dynamic_model_not_equivalent |
| `ac_three_phase_fault` | ac | balanced | 1 | PASS | 0.207887 | 0.999498 | 49.967201 | prefault_postfault_stage_numeric_fault_on_diagnostic |
| `ac_slg_fault_unbalanced` | ac | unbalanced | 1 | PASS | 0.240669 | 0.999498 | 50.000000 | native_only_unbalanced_transient_unsupported |
| `ac_explicit_fault_clear` | ac | balanced | 2 | PASS | 0.207887 | 0.999498 | 49.967201 | prefault_postfault_stage_numeric_fault_on_diagnostic |
| `generator_trip` | ac | balanced | 1 | PASS | 0.984487 | 0.999498 | 50.000000 | native_only_dynamic_model_not_equivalent |
| `dc_load_step` | dc | n/a | 1 | PASS | 0.991368 | 0.999371 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `dc_branch_trip_close` | dc | n/a | 2 | PASS | 0.991368 | 0.998793 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `dc_pole_fault` | dc | n/a | 1 | PASS | 0.991368 | 0.701291 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `vsc_trip` | hybrid_acdc | balanced | 1 | PASS | 0.990274 | 0.999498 | 49.998246 | native_only_gridlabd_dc_hybrid_unsupported |
| `dcdc_trip` | hybrid_acdc | balanced | 1 | PASS | 0.991368 | 0.999357 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `ac_storage_power_step` | ac | balanced | 1 | PASS | 0.991380 | 0.999498 | 50.000000 | native_only_dynamic_model_not_equivalent |
| `dc_storage_power_step` | dc | n/a | 1 | PASS | 0.991368 | 0.999498 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |
| `sequential_ac_combination` | ac | balanced | 5 | PASS | 0.199229 | 0.999498 | 49.962064 | partial_sequence_stage_numeric |
| `sequential_hybrid_combination` | hybrid_acdc | balanced | 9 | PASS | 0.199228 | 0.998553 | 49.960680 | native_only_gridlabd_dc_hybrid_unsupported |
| `unbalanced_sequential_combination` | hybrid_acdc | unbalanced | 5 | PASS | 0.240494 | 0.999371 | 50.000000 | native_only_gridlabd_dc_hybrid_unsupported |

## Numerical Summary

- Native cases: 18/18 passed
- Native event types: 14
- GridLAB-D exact sampled-stage gates: 18/18 passed
- GridLAB-D unique electrical stages executed: 3
- Maximum native-transient/GridLAB-D voltage difference: `4.696992e-04 pu`
- Maximum native/GridLAB-D equilibrium voltage-magnitude difference: `6.205294e-07 pu`
- Maximum equilibrium voltage-angle difference: `3.266908e-07 deg`
- Maximum equilibrium branch-P difference: `2.063428e-06 MW`
- Maximum equilibrium branch-Q difference: `3.642993e-07 Mvar`
- Worst native AC voltage: `0.199228 pu`
- Worst native DC voltage: `0.701291 pu`
- Worst native frequency nadir: `49.960680 Hz`
- Maximum native ROCOF: `38.304428 Hz/s`
- Maximum native bus-frequency spread: `3.458563 Hz`
- Maximum AC voltage-deviation integral: `3.481941e-02 pu-s`
- Maximum DC voltage-deviation integral: `1.205092e-02 pu-s`
- Maximum frequency-deviation integral: `8.276670e-03 Hz-s`
- Maximum AC time below 0.9 pu: `0.040563 s`
- Maximum DC time below 0.9 pu: `0.040562 s`
- Maximum qualified settling time: `0.257000 s`
- Cases not settled by the 0.30 s horizon: 0
- Worst native measured bus-frequency range: `44.571187` to `54.695292 Hz`
- Native bus-frequency range on the GridLAB-D anchor case: `45.330602` to `54.669398 Hz`

## Fidelity Boundary

HACDCPF rows are actual time-domain dynamic simulations. GridLAB-D rows are balanced AC algebraic snapshots solved at unique event stages and compared against the native trajectory at the same times. They validate voltage envelopes and post-event equilibria, not synchronous-machine, inverter-control, protection, frequency, ROCOF, DC-link, or converter-state trajectory equivalence.

DC, hybrid AC/DC, unbalanced, converter, and storage-control disturbances remain native-only because the current GridLAB-D bridge has no matched dynamic model and observable contract for those states. Unsupported cells are capability results, not inferred numerical comparisons.

## Event Coverage

| Event | Native | GridLAB-D |
|---|---|---|
| `ACBranchClose` | numeric_trajectory | numeric_sampled_ac_stage |
| `ACBranchImpedanceScale` | numeric_trajectory | representable_but_not_executed_in_gridlabd_sequence |
| `ACBranchTrip` | numeric_trajectory | numeric_sampled_ac_stage |
| `ACLoadScale` | numeric_trajectory | numeric_sampled_ac_stage |
| `ClearFault` | numeric_trajectory | covered_as_restored_ac_stage |
| `DCBranchClose` | numeric_trajectory | unsupported_gridlabd_dc_network |
| `DCBranchTrip` | numeric_trajectory | unsupported_gridlabd_dc_network |
| `DCDCTrip` | numeric_trajectory | unsupported_gridlabd_dc_network |
| `DCLoadScale` | numeric_trajectory | unsupported_gridlabd_dc_network |
| `DCStoragePowerStep` | numeric_trajectory | unsupported_gridlabd_dc_network |
| `FaultShunt` | numeric_trajectory | numeric_pre_post_ac_stage_fault_on_native_only |
| `GeneratorTrip` | numeric_trajectory | unsupported_matched_dynamic_device_model |
| `StoragePowerStep` | numeric_trajectory | unsupported_matched_dynamic_device_model |
| `VSCTrip` | numeric_trajectory | unsupported_matched_acdc_converter_dynamics |

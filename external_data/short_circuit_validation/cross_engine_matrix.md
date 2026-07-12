# Short-Circuit Cross-Engine Validation

Cases: 34
OpenDSS: dss-python 0.15.7
GridLAB-D: available

## Aggregate Errors

| Quantity | Mean relative error | Maximum relative error | Worst case |
|---|---:|---:|---|
| ikss_ka | 0.000005% | 0.000015% | transformer_vk_16 |
| ip_ka | 0.000006% | 0.000017% | transformer_vk_16 |
| GridLAB-D Ikss (28 balanced cases) | 0.000001% | 0.000002% | transformer_lv_tap_110 |

## Category Coverage

| Category | Cases | OpenDSS Ikss max error | OpenDSS ip max error | GridLAB-D cases | GridLAB-D Ikss max error |
|---|---:|---:|---:|---:|---:|
| distribution_feeder | 3 | 0.000000% | 0.000000% | 3 | 0.000000% |
| fault_type | 1 | 0.000000% | 0.000000% | 0 | n/a |
| line_sweep | 4 | 0.000000% | 0.000000% | 4 | 0.000001% |
| source_sweep | 4 | 0.000000% | 0.000000% | 4 | 0.000002% |
| topology | 3 | 0.000000% | 0.000000% | 3 | 0.000000% |
| transformer_impedance | 4 | 0.000015% | 0.000017% | 4 | 0.000001% |
| transformer_tap | 10 | 0.000015% | 0.000016% | 10 | 0.000002% |
| zero_sequence | 5 | 0.000011% | 0.000013% | 0 | n/a |

## Per-Case Results

| Case | Category | Fault | HACDCPF Ikss kA | OpenDSS Ikss kA | Error | GridLAB-D Ikss kA | Error | HACDCPF ip kA | OpenDSS+IEC ip kA | Error |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| source_strong_xr20 | source_sweep | three_phase | 28.804599 | 28.804599 | 0.0000% | 28.804599 | 0.0000% | 73.426757 | 73.426757 | 0.0000% |
| source_medium_xr10 | source_sweep | three_phase | 19.213067 | 19.213067 | 0.0000% | 19.213067 | 0.0000% | 47.441280 | 47.441280 | 0.0000% |
| source_weak_xr5 | source_sweep | three_phase | 11.544341 | 11.544341 | 0.0000% | 11.544341 | 0.0000% | 25.985282 | 25.985282 | 0.0000% |
| source_resistive_xr2 | source_sweep | three_phase | 11.658721 | 11.658721 | 0.0000% | 11.658721 | 0.0000% | 21.503613 | 21.503613 | 0.0000% |
| line_short | line_sweep | three_phase | 26.231300 | 26.231300 | 0.0000% | 26.231300 | 0.0000% | 64.770840 | 64.770840 | 0.0000% |
| line_nominal | line_sweep | three_phase | 15.983432 | 15.983432 | 0.0000% | 15.983432 | 0.0000% | 38.927139 | 38.927139 | 0.0000% |
| line_long | line_sweep | three_phase | 8.412295 | 8.412295 | 0.0000% | 8.412295 | 0.0000% | 19.633063 | 19.633063 | 0.0000% |
| line_high_r | line_sweep | three_phase | 13.026575 | 13.026575 | 0.0000% | 13.026575 | 0.0000% | 20.848394 | 20.848394 | 0.0000% |
| radial_3bus_near | topology | three_phase | 17.958403 | 17.958403 | 0.0000% | 17.958403 | 0.0000% | 43.006962 | 43.006962 | 0.0000% |
| radial_3bus_far | topology | three_phase | 11.458102 | 11.458102 | 0.0000% | 11.458102 | 0.0000% | 26.959780 | 26.959780 | 0.0000% |
| meshed_3bus | topology | three_phase | 17.939616 | 17.939616 | 0.0000% | 17.939616 | 0.0000% | 42.582224 | 42.582224 | 0.0000% |
| feeder_10bus_near | distribution_feeder | three_phase | 36.477167 | 36.477167 | 0.0000% | 36.477167 | 0.0000% | 84.757353 | 84.757353 | 0.0000% |
| feeder_10bus_mid | distribution_feeder | three_phase | 26.782588 | 26.782588 | 0.0000% | 26.782588 | 0.0000% | 58.987156 | 58.987156 | 0.0000% |
| feeder_10bus_far | distribution_feeder | three_phase | 18.426738 | 18.426738 | 0.0000% | 18.426738 | 0.0000% | 38.485266 | 38.485266 | 0.0000% |
| transformer_hv_tap_90 | transformer_tap | three_phase | 12.918611 | 12.918613 | 0.0000% | 12.918612 | 0.0000% | 31.889920 | 31.889925 | 0.0000% |
| transformer_hv_tap_95 | transformer_tap | three_phase | 13.694051 | 13.694053 | 0.0000% | 13.694051 | 0.0000% | 33.803541 | 33.803546 | 0.0000% |
| transformer_hv_tap_100 | transformer_tap | three_phase | 14.433757 | 14.433759 | 0.0000% | 14.433757 | 0.0000% | 35.628921 | 35.628926 | 0.0000% |
| transformer_hv_tap_105 | transformer_tap | three_phase | 15.137424 | 15.137426 | 0.0000% | 15.137424 | 0.0000% | 37.365311 | 37.365316 | 0.0000% |
| transformer_hv_tap_110 | transformer_tap | three_phase | 15.805291 | 15.805292 | 0.0000% | 15.805291 | 0.0000% | 39.013310 | 39.013315 | 0.0000% |
| transformer_lv_tap_90 | transformer_tap | three_phase | 17.819453 | 17.819455 | 0.0000% | 17.819453 | 0.0000% | 43.986322 | 43.986327 | 0.0000% |
| transformer_lv_tap_95 | transformer_tap | three_phase | 15.993083 | 15.993085 | 0.0000% | 15.993083 | 0.0000% | 39.478028 | 39.478033 | 0.0000% |
| transformer_lv_tap_100 | transformer_tap | three_phase | 14.433757 | 14.433759 | 0.0000% | 14.433757 | 0.0000% | 35.628921 | 35.628926 | 0.0000% |
| transformer_lv_tap_105 | transformer_tap | three_phase | 13.091843 | 13.091845 | 0.0000% | 13.091843 | 0.0000% | 32.316481 | 32.316486 | 0.0000% |
| transformer_lv_tap_110 | transformer_tap | three_phase | 11.928725 | 11.928727 | 0.0000% | 11.928725 | 0.0000% | 29.445389 | 29.445394 | 0.0000% |
| transformer_vk_4 | transformer_impedance | three_phase | 20.621038 | 20.621040 | 0.0000% | 20.621038 | 0.0000% | 50.452418 | 50.452424 | 0.0000% |
| transformer_vk_6 | transformer_impedance | three_phase | 18.044649 | 18.044651 | 0.0000% | 18.044649 | 0.0000% | 43.851938 | 43.851943 | 0.0000% |
| transformer_vk_10 | transformer_impedance | three_phase | 14.433757 | 14.433759 | 0.0000% | 14.433757 | 0.0000% | 35.628921 | 35.628926 | 0.0000% |
| transformer_vk_16 | transformer_impedance | three_phase | 11.102934 | 11.102935 | 0.0000% | 11.102934 | 0.0000% | 27.539077 | 27.539082 | 0.0000% |
| slg_z0_5 | zero_sequence | single_phase_ground | 19.180119 | 19.180119 | 0.0000% | unsupported_fault_type | n/a | 46.712567 | 46.712567 | 0.0000% |
| slg_z0_10 | zero_sequence | single_phase_ground | 15.983432 | 15.983432 | 0.0000% | unsupported_fault_type | n/a | 38.927139 | 38.927139 | 0.0000% |
| slg_z0_20 | zero_sequence | single_phase_ground | 11.987574 | 11.987574 | 0.0000% | unsupported_fault_type | n/a | 29.195354 | 29.195354 | 0.0000% |
| slg_z0_30 | zero_sequence | single_phase_ground | 9.590059 | 9.590059 | 0.0000% | unsupported_fault_type | n/a | 23.356283 | 23.356283 | 0.0000% |
| two_phase_radial | fault_type | two_phase | 13.842058 | 13.842058 | 0.0000% | unsupported_fault_type | n/a | 33.711891 | 33.711891 | 0.0000% |
| transformer_slg_tap_lv_95 | zero_sequence | single_phase_ground | 15.993083 | 15.993085 | 0.0000% | unsupported_fault_type | n/a | 39.478028 | 39.478033 | 0.0000% |

## GridLAB-D Scope

GridLAB-D has no FaultStudy/Zsc result interface. Three-phase RMS current is extrapolated from an explicit 100 ohm balanced shunt-probe powerflow; unbalanced faults and IEC peak remain unsupported.
GridLAB-D cells are therefore capability results, not inferred short-circuit numbers.

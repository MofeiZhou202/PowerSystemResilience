# Harmonic Power-Flow Cross-Engine Matrix

Generated: `2026-08-21T15:57:27.504784+00:00`

| Case | Domain | Balance | Native vs NumPy max | OpenDSS status | OpenDSS max | GridLAB-D | Result |
|---|---|---|---:|---|---:|---|---|
| `ac_radial_characteristic` | ac | positive_sequence | 4.614e-10 | direct_equivalent_network | 4.579e-10 | 4 slices / 4.614e-10 | PASS |
| `ac_resistive_feeder` | ac | positive_sequence | 3.464e-10 | direct_equivalent_network | 3.436e-10 | 4 slices / 3.464e-10 | PASS |
| `ac_skin_sqrt` | ac | positive_sequence | 3.820e-10 | direct_equivalent_network | 3.790e-10 | 4 slices / 3.820e-10 | PASS |
| `ac_skin_proportional` | ac | positive_sequence | 3.819e-10 | direct_equivalent_network | 3.789e-10 | 4 slices / 3.819e-10 | PASS |
| `ac_3ph_balanced` | ac_3ph | balanced | 4.202e-10 | direct_phase_domain_equivalent | 4.256e-10 | unsupported_or_unavailable | PASS |
| `ac_3ph_unbalanced` | ac_3ph | unbalanced | 4.202e-10 | direct_phase_domain_equivalent | 4.170e-10 | unsupported_or_unavailable | PASS |
| `ac_3ph_unbalanced_skin` | ac_3ph | unbalanced | 4.202e-10 | direct_phase_domain_equivalent | 4.170e-10 | unsupported_or_unavailable | PASS |
| `ac_3ph_zero_sequence` | ac_3ph | balanced | 2.271e-10 | unsupported_without_sequence_model_substitution | n/a | unsupported_or_unavailable | PASS |
| `dc_resistive_ripple` | dc | n/a | 2.220e-13 | unsupported_dc_harmonic_network | n/a | unsupported_or_unavailable | PASS |
| `dc_inductive_ripple` | dc | n/a | 1.528e-12 | unsupported_dc_harmonic_network | n/a | unsupported_or_unavailable | PASS |
| `dc_capacitive_filter` | dc | n/a | 2.218e-13 | unsupported_dc_harmonic_network | n/a | unsupported_or_unavailable | PASS |
| `dc_lc_filter` | dc | n/a | 1.539e-12 | unsupported_dc_harmonic_network | n/a | unsupported_or_unavailable | PASS |
| `hybrid_vsc_characteristic` | hybrid_acdc | positive_sequence | 5.350e-10 | ac_port_equivalent_network_only | 5.333e-10 | 4 slices / 5.350e-10 | PASS |
| `hybrid_vsc_filtered` | hybrid_acdc | positive_sequence | 5.350e-10 | ac_port_equivalent_network_only | 5.333e-10 | 4 slices / 5.350e-10 | PASS |

## Summary

- Cases: 14 (14 passed, 0 failed)
- OpenDSS numeric cases: 9
- Maximum native/independent voltage error: `5.349715e-10 pu`
- Maximum native/OpenDSS voltage error: `5.333277e-10 pu`
- Maximum branch-current magnitude error: `5.366554e-12 pu`
- Maximum voltage IHD error: `7.880336e-10 percentage points`
- Maximum voltage THD error: `1.361585e-09 percentage points`
- Maximum sequence-component error: `1.508189e-12 pu`
- GridLAB-D numerical frequency slices: 24
- Maximum native/GridLAB-D complex-voltage error: `5.349717e-10 pu`

## Interpretation

OpenDSS rows are direct frequency-domain solves of an electrically equivalent Reactor/Isource network. The hybrid rows compare only the AC port in OpenDSS; DC and converter coupling are independently checked with a dense complex nodal solve. The zero-sequence case is not translated to a decoupled OpenDSS circuit because doing so would discard its distinct zero-sequence impedance.

GridLAB-D has no native harmonic-order API. Each reported slice is therefore an independently executed complex steady-state network at one authored harmonic frequency: R(h), hX, source impedance, and current injection are explicit. The measured voltage/current transfer impedance is applied to the same source phasor before comparison; this validates the decoupled per-order network equation without claiming nonlinear waveform/FFT coverage.

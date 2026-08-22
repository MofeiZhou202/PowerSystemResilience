# Harmonic Power-Flow Verification Contract

The full theory, implementation contract, audit, and numerical tables are in
[the harmonic power-flow monograph](harmonics_power_flow_manual.tex).

| Comparison | Frozen gate | Current result |
|---|---:|---:|
| Native / independent NumPy | `2e-8 pu` | `5.349715355e-10 pu` |
| Native / OpenDSS | `2e-6 pu` | `5.333277457e-10 pu` |
| Native / GridLAB-D | `2e-5 pu`, at least 20 slices | `5.349716508e-10 pu`, 24 slices |
| IEEE13 / OpenDSS | `2e-3 pu` | `1.652e-3 pu`, 164 points |

All 14 AC, three-phase, DC, and hybrid AC/DC matrix cases pass. The device-level
VSC equivalent-network comparison differs from OpenDSS by at most `5.075e-10 pu`.
The C++ suite passes 63 cases / 412 assertions. The added HSS gates cover
zero-coupling degeneration, an SI DC-capacitor closed form, an independent
two-frequency off-diagonal closed form, four converter families, JSON identity,
and the 1000-node/20-frequency/four-VSC sparse-storage case.

GridLAB-D 5.3.0 has no native harmonic-order API. Each reported AC slice is an
actual independent process with explicit `R(h)+j h X`, source impedance, and
constant-current injection. Measured complex voltages and branch current identify
the transfer impedance before applying the common harmonic source phasor. This
validates the decoupled frequency-slice equation; it is not an EMT/PWM/FFT claim.
CTest requires GridLAB-D and at least 20 numerical slices.

Machine-readable evidence is stored in `external_data/harmonics_validation/`.

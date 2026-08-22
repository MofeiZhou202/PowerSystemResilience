# Harmonic Cross-Engine Validation Data

`cross_engine_matrix.json` is the machine-readable benchmark result and
`cross_engine_matrix.md` is its compact human-readable summary. Regenerate both
with:

```bash
cmake --build build/macos-release --target validate_harmonics_cross_engine_matrix
```

The evidence set distinguishes four validation levels:

- Direct/equivalent OpenDSS frequency-domain AC solves.
- Independent NumPy complex-nodal solves for AC, DC, and hybrid AC/DC ports.
- Executed GridLAB-D 5.3.0 complex steady-state frequency slices for every
  representable AC order (`--require-gridlabd`, minimum 20 slices).
- IEEE13 real-feeder phasor comparison against OpenDSS.

GridLAB-D slices explicitly author `R(h)+j h X`, source impedance, and a current
probe, then recover complex transfer impedance from recorded voltage/current.
They validate the decoupled AC frequency-domain equation without claiming an
EMT/PWM waveform or FFT comparison. DC and converter cross-order paths remain
covered by the independent NumPy equations because GridLAB-D has no matching
native API.

Additional machine-readable files:

- `ieee13_opendss.json`: 164 common bus/phase/order points, `2e-3 pu` gate.
- `device_opendss_report.json`: converter-current network response and native
  OpenDSS device capability probe.

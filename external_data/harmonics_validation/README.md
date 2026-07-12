# Harmonic Cross-Engine Validation Data

`cross_engine_matrix.json` is the machine-readable benchmark result and
`cross_engine_matrix.md` is its compact human-readable summary. Regenerate both
with:

```bash
cmake --build build/macos-release --target validate_harmonics_cross_engine_matrix
```

The matrix distinguishes three validation levels:

- Direct/equivalent OpenDSS frequency-domain AC solves.
- Independent NumPy complex-nodal solves for AC, DC, and hybrid AC/DC ports.
- Explicit unsupported capability rows for GridLAB-D frequency-domain HPF.

Unsupported rows are not numerical failures. GridLAB-D requires a validated
deltamode waveform model and FFT extraction before an AC harmonic comparison is
scientifically meaningful, and it has no matching native DC harmonic network or
converter-coupled AC/DC harmonic API.

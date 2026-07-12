# Short-Circuit Cross-Engine Validation

This directory contains reproducible HACDCPF, OpenDSS, and GridLAB-D
short-circuit comparison artifacts.

## Coverage

The matrix includes 34 cases across:

- source strength and X/R ratio;
- line length and high-resistance distribution lines;
- radial, meshed, and 10-bus feeder fault locations;
- 4%, 6%, 10%, and 16% transformer impedances;
- HV and LV taps from 0.90 through 1.10;
- three-phase, two-phase, and single-line-to-ground faults;
- zero-sequence impedance multipliers from 0.5 through 3.0.

HACDCPF and OpenDSS are compared for all 34 cases. GridLAB-D is compared for
the 28 balanced three-phase cases. GridLAB-D does not expose an OpenDSS-style
`FaultStudy`, `Zsc1`, `Zsc0`, or IEC peak-current API, so its balanced current
is obtained by solving a 100-ohm three-phase shunt probe and extrapolating the
complex Thevenin impedance. GridLAB-D is not used for unbalanced faults or
peak-current validation.

IEC transformer correction is disabled for cross-engine comparisons because
OpenDSS and GridLAB-D model physical nameplate impedance rather than IEC 60909
`K_T`. The normal HACDCPF default remains to apply `K_T`.

## Reproduction

Build the native matrix generator:

```sh
cmake --build build/macos-release --target short_circuit_validation_matrix -j4
```

Run OpenDSS-only validation:

```sh
tools/harmonics_validation/.venv/bin/python \
  tools/short_circuit_validation/run_cross_engine_matrix.py
```

Run all three engines:

```sh
GRIDLABD_BIN=/path/to/gridlabd \
tools/harmonics_validation/.venv/bin/python \
  tools/short_circuit_validation/run_cross_engine_matrix.py
```

Outputs:

- `cross_engine_matrix.json`: complete machine-readable inputs and results;
- `cross_engine_matrix.md`: aggregate and per-case human-readable report;
- `opendss_transformer_taps.json`: focused transformer-tap reference fixture.

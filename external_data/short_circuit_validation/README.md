# Short-Circuit Cross-Engine Validation

This directory contains reproducible HACDCPF, OpenDSS, and GridLAB-D
short-circuit comparison artifacts.

## Coverage

The matrix includes 50 cases across:

- source strength and X/R ratio;
- line length and high-resistance distribution lines;
- radial, meshed, and 10-bus feeder fault locations;
- 4%, 6%, 10%, and 16% transformer impedances;
- HV and LV taps from 0.90 through 1.10;
- three-phase, two-phase, and single-line-to-ground faults;
- zero-sequence impedance multipliers from 0.5 through 3.0.
- grid-following IBR current limits from 1.05 through 1.50 pu;
- grid-following ratings, local/remote location, and multiple converters;
- grid-forming ratings and virtual/filter reactance from 0.10 through 0.30 pu;
- mixed grid-forming and grid-following converter systems.

HACDCPF and OpenDSS are compared for all 50 cases. GridLAB-D is compared for
35 balanced three-phase passive/GFM cases. GridLAB-D does not expose an OpenDSS-style
`FaultStudy`, `Zsc1`, `Zsc0`, or IEC peak-current API, so its balanced current
is obtained by solving a 100-ohm three-phase shunt probe and extrapolating the
complex Thevenin impedance. Its steady NR inverter does not enforce a declared
GFL short-circuit current limit, so GridLAB-D is not used for GFL, unbalanced,
or peak-current validation. OpenDSS GFL validation uses explicit `Isource`
elements aligned with the utility fault-current phasor and an explicit fault.

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

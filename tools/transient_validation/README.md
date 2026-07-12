# Transient Cross-Engine Validation

Build the native disturbance emitter and GridLAB-D validation runner:

```bash
cmake --build build/macos-release --target transient_validation_matrix gridlabd_validation_matrix
```

Run the full matrix with GridLAB-D:

```bash
GRIDLABD_BIN=/path/to/gridlabd \
tools/harmonics_validation/.venv/bin/python \
  tools/transient_validation/run_cross_engine_matrix.py \
  --gridlabd-bin /path/to/gridlabd
```

The report distinguishes actual HACDCPF dynamic trajectories from GridLAB-D
balanced AC event-stage snapshots. The latter validate voltage envelopes and
post-event equilibrium only. They do not establish equivalence of machine,
inverter, protection, frequency, DC-link, or converter dynamic states.

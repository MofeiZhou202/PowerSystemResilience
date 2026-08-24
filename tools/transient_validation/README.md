# Transient Cross-Engine Validation

## GridLAB-D disturbance matrix

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

## COSMIC transient-protection comparison

This directory runs the authors' public COSMIC IEEE 9-bus examples without
modifying their source. It validates only the common definite-time UVLS and
balanced positive-sequence Zone-1 apparent-admittance subset; it is not an EMT,
general distance-protection, or formal chronology certificate.

### Reproduce

Requirements:

- MATLAB R2025b or a compatible MATLAB executable;
- a clean checkout of COSMIC commit
  `6acc77e4d3f17925f1f4b79a93652eef0d1314cc`.

```bash
git clone https://github.com/ecotillasanchez/cosmic.git /tmp/hysim-cosmic
git -C /tmp/hysim-cosmic checkout 6acc77e4d3f17925f1f4b79a93652eef0d1314cc
python3 tools/transient_validation/run_cosmic_reference.py \
  --cosmic-root /tmp/hysim-cosmic \
  --out /tmp/hysim_cosmic_cross_validation.json
```

The wrapper rejects a different commit, dirty worktree, or source archive
without Git metadata with exit code 4. `--allow-source-mismatch` is only for
explicit exploratory runs. `--require-paper-match` additionally returns exit
code 3 when the paper Fig. 2 chronology is absent.

### Verified result

The public `sim_case9.m` scenario matches its own source: branch 6 trips at
`10.0 s`, bus 5 UVLS acts at `10.5 s`, and `31.25 MW` (25%) is shed.

The paper Fig. 2 reconstruction does not match the same public commit. With
branch 7 initially tripped, a `0.92 pu` UVLS threshold, and `0.5 s` relay
delays, bus 5 reaches `0.896065 pu`, while branch 6's maximum distance pickup
ratio is `0.3098124214893234 < 1`. No `10.5 s` distance action or `10.7 s`
UVLS event is emitted. This indicates that the paper's complete configuration
or code version is not present in the fixed public source; it must not be
reported as a successful full-case reproduction.

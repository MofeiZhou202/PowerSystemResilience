# Time-series validation evidence

This directory contains the reproducible cross-validation evidence for the
`time_series` monograph.

- `cross_engine_matrix.json`: machine-readable inputs, all oracle outputs,
  errors, gates, engine scope and exclusions.
- `cross_engine_matrix.md`: compact human-readable result table.

Reproduce from the repository root:

```bash
cmake --preset macos-release
cmake --build build/macos-release --target validate_time_series_xref -j4
.venv/bin/python tools/time_series_validation/run_cross_engine_matrix.py \
  --cpp-bin build/macos-release/tests/validate_time_series_xref
```

The UC case is independently rebuilt with SciPy/HiGHS and proved over all 256
two-unit/four-period commitment sequences. The AC network case consists of six
balanced positive-sequence snapshots solved by HySim, OpenDSS and GridLAB-D.
OpenDSS/GridLAB-D do not provide an oracle for commitment, annual decomposition,
or lifecycle accounting; no such equivalence is claimed.

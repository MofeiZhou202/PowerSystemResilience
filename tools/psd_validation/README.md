# PowerSimulationsDynamics.jl Validation

This folder contains reusable helpers for cross-checking HACDCPF transient
models against PowerSimulationsDynamics.jl benchmark cases.

These helpers are local validation tools only. They are not part of the HACDCPF
release module, are not linked by the C++ library, and are only invoked by tests
when a developer explicitly sets `HACDCPF_RUN_PSD_COMPARE=1`.

## Validation Ladder

The C++ transient test suite now writes HACDCPF validation traces for three
levels:

- Component level: conventional generator subset derived from PSD Test 15
  `GENROU` three-bus data.
- Component level: constant-power load voltage response derived from PSD Test 33
  ZIP-load data.
- System level: hybrid AC/DC VSC step response with preserved dynamic profiles.

Normal C++ tests do not require Julia. They check that the HACDCPF models
initialize at dynamic equilibrium, react to events, preserve dynamic metadata,
and emit trace artifacts.

Artifacts are written under:

```text
${TMPDIR}/hacdcpf_psd_validation
```

The summary CSV is:

```text
${TMPDIR}/hacdcpf_psd_validation/hacdcpf_psd_validation_summary.csv
```

## Grid-Following Inverter Benchmarks

The C++ test `test_transient_dynamics` always runs local HACDCPF traces for:

- PSD Test 24: grid-following inverter with `ReducedOrderPLL`.
- PSD Test 51: grid-following inverter with `KauraPLL`.

Both use the PSD reference-power step `P_ref: 0.5 -> 0.7` at `t = 1.0 s`.
The normal test run does not require Julia.

To run the external PSD comparison:

```bash
cd /Users/tianyangzhao/Codes/HybridACDCDistributionSystemsSimulation
HACDCPF_RUN_PSD_COMPARE=1 ./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][benchmark][psd][gridfollowing]"
```

Optional environment variables:

- `HACDCPF_PSD_REPO`: path to the PowerSimulationsDynamics.jl checkout.
  Defaults to `/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl`.
- `HACDCPF_JULIA_BIN`: Julia executable. Defaults to `julia`.

If PSD dependencies are missing, instantiate the PSD test environment first:

```bash
cd /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl
julia --project=test -e 'using Pkg; Pkg.instantiate()'
```

The C++ test writes HACDCPF and PSD CSV traces plus Julia logs under:

```text
${TMPDIR}/hacdcpf_psd_gridfollowing
```

The focused grid-following exporter can also be run directly:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_gridfollowing_trace.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  test24 \
  /tmp/psd_test24_p_oc.csv \
  p_oc
```

## Generic PSD Trace Exporter

`export_trace.jl` supports the broader validation ladder. The single-signal form
is:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_trace.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  genrou \
  /tmp/psd_genrou_delta.csv \
  generator-102-1:delta_deg
```

For external test performance, prefer batch mode. It starts Julia once, runs each
PSD case once, and writes all requested signals:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_trace.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  --batch \
  'genroe|generator-102-1:delta_rad=/tmp/psd_genroe_delta.csv' \
  'genroe|generator-102-1:omega_pu=/tmp/psd_genroe_omega.csv'
```

PSD Test 02 / OneDOneQ machine traces can be exported the same way:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_trace.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  onedoneq \
  /tmp/psd_onedoneq_eq_p.csv \
  generator-102-1:eq_p
```

Supported cases today:

- `onedoneq` / `test02`
- `simple_marconato` / `test03`
- `genrou`
- `genroe` / `test16`
- `genroe_high_sat` / `test16_high_sat`
- `gensal` / `test18`
- `gensae` / `test19`
- `zip_constant_power`
- `test24` / `gridfollowing_reduced`
- `test51` / `gridfollowing_kaura`

Supported signals today:

- `bus<number>:voltage_mag`
- `bus<number>:voltage_angle`
- `<device>:delta_rad`
- `<device>:delta_deg`
- `<device>:omega_pu`
- `<device>:eq_p`
- `<device>:ed_p`
- `<device>:eq_pp`
- `<device>:ed_pp`
- `<device>:psi_kd`
- `<device>:psi_kq`
- `<device>:psiq_pp` / `<device>:psi_q_pp`
- `<device>:frequency_pu`
- `<device>:p_pu`
- `<device>:q_pu`
- `<device>:p_oc`

Run the external comparison tests with:

```bash
cd /Users/tianyangzhao/Codes/HybridACDCDistributionSystemsSimulation
HACDCPF_RUN_PSD_COMPARE=1 ./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][benchmark][psd][external]"
```

In addition to the RMS/max summary, the external comparison writes a pointwise
CSV under:

```text
${TMPDIR}/hacdcpf_psd_validation/psd_external_pointwise_comparison.csv
```

Each row is one comparison sample with the PSD test case, signal, time,
HACDCPF raw value, PSD raw value, aligned comparison values, signed error, and
absolute error. Signals marked `relative_to_initial` use the same initial-value
alignment as the pass/fail gate.

The conventional component checks are now numerical PSD trace gates rather than
plumbing-only checks. The ZIP constant-power case compares bus-102 and bus-103
voltage-magnitude deviations against PSD Test 33. The machine gates cover GENROU,
OneDOneQ, SimpleMarconato, GENROE normal/high-saturation variants, GENSAL, and
GENSAE traces. SimpleMarconato now clears the PSD Test 03 trace gate for both
machines, including `generator-103-1:delta_rad_relative` at about `0.00142` rms
and `0.00402` max error. GENROE compares `delta_rad`/`omega_pu`/`eq_p`/`ed_p`;
GENSAL and GENSAE compare `delta_rad`/`omega_pu`/`eq_p`/`psiq_pp`. The PSSE
machine angle gate is intentionally broader than the inverter gates because
HACDCPF still uses its synthesized three-phase network solve instead of PSD's
exact positive-sequence residual DAE.

## Executable Component Manifest

`psd_component_test_matrix.json` is also an executable test manifest. The report
rows remain the human-facing passability matrix, while
`execution_manifest.cases[]` declares opt-in numerical gates that the C++ test
suite can run directly.

The always-on structural check validates that each enabled executable case maps
to exactly one matrix row and has a complete fixture, solver, event, signal, and
tolerance declaration:

```bash
./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][benchmark][psd][manifest]"
```

The first executable gate is PSD Test 03 SimpleMarconato. The external trace
part compares PSD `ResidualModel`/`IDA` traces from `simple_marconato` against
the HACDCPF `MassMatrixDae` trajectory for both machines and all declared
`delta_rad`/`omega_pu`/`eq_p`/`ed_p` state traces:

```bash
HACDCPF_RUN_PSD_COMPARE=1 ./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][benchmark][psd][manifest][external]"
```

This writes artifacts under:

```text
${TMPDIR}/hacdcpf_psd_manifest_validation
```

The key files are `psd_manifest_validation_summary.csv`,
`psd_manifest_pointwise_comparison.csv`, and `psd_manifest_batch.log`.

The internal diagnostics part compares the initialized PSD-coordinate DAE
objects for the same case: residual vector norm, mass diagonal, full finite-
difference Jacobian, Schur-reduced Jacobian, and small-signal eigenvalues:

```bash
HACDCPF_RUN_PSD_COMPARE=1 ./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][benchmark][psd][manifest][internal]"
```

This writes artifacts under:

```text
${TMPDIR}/hacdcpf_psd_internal_validation
```

The key files are `internal_comparison_summary.csv`, `psd/state_table.csv`,
`hacdcpf/state_table.csv`, and the paired `jacobian.csv`,
`reduced_jacobian.csv`, and `eigenvalues.csv` files. The current residual,
mass-diagonal, unreduced full-Jacobian, reduced-Jacobian, and eigenvalue gates
are all tight for the accepted SimpleMarconato Test 03 baseline.

## PSD Input Snapshot Conversion

`export_psd_snapshot.jl` converts a PSD/PowerSystems case into a neutral JSON
manifest:

```text
hacdcpf_psd_snapshot.v1
```

The snapshot preserves:

- static `PowerSystems.System` component identity,
- PSD `DynamicInjection` devices,
- generator slots: machine, shaft, AVR, governor, PSS,
- inverter slots: DC source, frequency estimator, outer control, inner control,
  converter, filter, limiter,
- a best-effort HACDCPF `dynamic_model` candidate for currently supported
  profiles.

Run it with a named PSD validation case:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_psd_snapshot.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  test24 \
  /tmp/hacdcpf_psd_test24_snapshot.json
```

It also accepts a PowerSystems JSON file, or a RAW/DYR pair:

```bash
julia --project=/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl/test \
  tools/psd_validation/export_psd_snapshot.jl \
  /Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl \
  /path/to/case.raw \
  /tmp/hacdcpf_psd_case_snapshot.json \
  /path/to/case.dyr
```

This is an input-conversion manifest for comparison and validation. It is not
yet a full HACDCPF network importer for every PSD/PowerSystems feature.

## Model And Controller Crosswalk

The model-by-model comparison lives in:

```text
tools/psd_validation/psd_hacdcpf_model_crosswalk.json
```

Regenerate the readable reports with:

```bash
python3 tools/psd_validation/compare_model_catalogs.py \
  --out-md tools/psd_validation/psd_model_comparison.md \
  --out-csv tools/psd_validation/psd_model_comparison.csv \
  --fail-on-stale \
  --print-summary
```

The generator cross-checks referenced HACDCPF model names against
`src/dynamics/DynamicModelCatalog.cpp`, so a stale crosswalk row fails visibly.

## Machine And IBR Component Test Gate

For the first PSD-vs-HACDCPF decision gate, focus on transmission-dynamics
machines and IBRs:

```bash
python3 tools/psd_validation/compare_component_tests.py \
  --out-md tools/psd_validation/psd_component_test_matrix.md \
  --out-csv tools/psd_validation/psd_component_test_matrix.csv \
  --fail-on-blocked \
  --print-summary
```

`--fail-on-blocked` intentionally returns non-zero while any PSD machine/IBR
component test group is not comparable with the current HACDCPF runtime. This is
the stop-and-decide point before implementing a large model library.

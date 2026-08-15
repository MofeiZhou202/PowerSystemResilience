# Native IPM and Windows Integration, 2026-08-13

## Scope

This first integration sequence adopts contract-preserving work from
`release/windows-self-contained`: shared variable-bound predicates, NLP
diagnostic metadata, audited restored starts, observational filter diagnostics,
and explicit Windows oneMKL packaging. It does not change the current barrier,
filter, inertia, Newton-formulation, or LCQP defaults on `main`.

The source branch's LCQP predictor-corrector and automatic Newton changes remain
held. In the audited AppleClang Release build, its LP case terminated with
`NativeLCQP: KKT affine solve NaN`, while the automatic Newton fallback case did
not terminate within 90 seconds. The equivalent `main` IPM suite completed in
about two seconds before this integration.

## Rationale

Model/algorithm: Contract-preserving integration around the existing
Mehrotra/filter IPM implementations. Restored points are accepted only after an
independent primal-feasibility audit; diagnostics do not enter any update rule.

Claim: Metadata, diagnostics, restored-point auditing, and Windows runtime
packaging do not change accepted iterates under default options. A requested
restored start is preserved only when its original-coordinate violation and its
variable-bound violation are finite and no greater than `tol_primal`.

Cost model: Metadata requires O(rows + variables) storage. Diagnostic counters
perform O(1) work per line-search event. The restored-start audit is O(rows +
variables) once per requested solve. These operations are control-bound and are
expected to change total solve time by less than 1%. Packaging has no solve-time
cost.

Prediction: Existing macOS solver tests retain their behavior, focused tests
pass, and runtime changes remain within measurement noise (0%, tolerance +/-1%).
Windows `SEQUENTIAL` packages declare no Intel OpenMP DLL; Windows `INTEL`
packages contain every runtime DLL declared by their package config and
manifest.

Assumptions: Diagnostic callbacks are observational, callers return a finite
nonnegative original-coordinate violation, sequential MKL remains the Windows
default, and macOS continues to use Accelerate and MUMPS without MKL.

References: Wachter and Biegler, *Mathematical Programming* 106 (2006),
Sections 2-3; Higham, *Accuracy and Stability of Numerical Algorithms*, second
edition, Section 7.6; [numerical_methods.md](numerical_methods.md); common
ancestor `4989eba` and pre-integration `main` commit `4dbb71f`.

Validation fixed before implementation: AppleClang Release builds of
`test_ipm_solver` and `test_engine_api`; focused metadata, restored-start, and
counter tests; a non-MKL CMake configure; Windows package configuration and
consumer checks when an MSVC runner is available. Correctness requires all
existing assertions to pass. Termination requires no test case to exceed ten
times its pre-integration runtime. The performance prediction is 0% with a
measurement-noise tolerance of +/-1%.

## Bound Contract

`VariableMeta` uses `-1e20` and `1e20` as its public no-bound sentinels.
Directional predicates treat values strictly inside those sentinels as finite.
This avoids adapter-specific nearby thresholds and correctly distinguishes a
finite lower side from a finite upper side.

## Restored Starts

`primal_feasible_start` and `preserve_initial_point` are opt-in filter-driver
policies. The audit first checks the point dimension and finiteness. It then uses
`original_constraint_violation` when supplied, otherwise the solver-coordinate
equality and inequality callbacks, and always checks variable bounds
independently. Invalid callback results, including negative or nonfinite values,
reject preservation and retain ordinary interiorization.

Scaling retains the original-coordinate callback unchanged because scaling does
not transform the primal variables. Fixed-variable reduction wraps the callback
by expanding reduced variables back into original coordinates before auditing.

## Diagnostics

The filter reports the selected sparse factorization backend, accepted coherent
steps, evaluated value-only trial points, rejected evaluated trials, and
rejections that occurred before derivative evaluation. Current line-search
trials are value-only, so full trial derivative evaluations are zero. Counters
aggregate all executed restoration, retry, and continuation solves rather than
only the selected final outcome.

## Windows Packaging

`MIPSOLVERS_MKL_THREADING` is Windows-only. `SEQUENTIAL` is the default and is
the only mode supported by the repository-staged static bundle. `INTEL` is an
explicit local oneAPI profile that selects `mkl_intel_thread`, requires
`libiomp5md.lib` and `libiomp5md.dll`, installs the DLL into the package `bin`
directory, and records its relative payload in the package config and manifest.
Package loading and the consumer smoke test fail if a declared file is absent.

macOS remains outside this MKL policy and continues to use Accelerate/MUMPS.

## Benchmark Matrix

The correctness matrix is `test_ipm_solver` and `test_engine_api`, plus focused
tests tagged `[ipm][bounds][contract]`, `[ipm][scaling][diagnostics]`,
`[ipm][nlp][initialization][regression]`, and `[ipm][nlp][diagnostics]`.
Metrics are success/status, KKT residuals, iterations, factorization counts,
trial counters, and wall time. Future adoption of the held LCQP/Newton work must
compare those metrics against `main`, show no correctness loss or greater than
10x case runtime, and state a separate quantitative speed prediction before any
algorithm code changes.

Measured results are appended only after running this fixed protocol. A wrong
sign or a deviation greater than 50% of the predicted effect triggers the
repository mismatch protocol before further algorithm edits.

## Measured Results

Machine: Apple M4 MacBook Pro, arm64, Darwin 25.5.0. Compiler and flags:
AppleClang 21.0.0, CMake `macos-release` preset, Release, Accelerate, MUMPS, and
OpenMP. Source commit before working-tree changes: `4dbb71f`.

Commands:

```text
cmake --preset macos-release
cmake --build build/macos-release --target test_ipm_solver test_engine_api -j 8
./tests/test_ipm_solver --rng-seed 1
./tests/test_engine_api --rng-seed 1
```

`test_ipm_solver` passed 30 cases and 203 assertions; the timed run before the
final fixed-variable regression case passed 29 cases and 199 assertions in
0.641 seconds. `test_engine_api` passed 25 cases and 143 assertions in
0.640 seconds. Five warm runs of the clean-main snapshot and five warm runs of
the integration's IPM executable were both below the 0.01-second resolution of
`/usr/bin/time`; the measured change is therefore indistinguishable from 0%,
matching the predicted 0% observational effect. The focused integration subset
passed 5 cases and 27 assertions.

The macOS configure selected Accelerate and MUMPS and did not select MKL. A
third-party-only, MKL-disabled configure also completed successfully. Windows
MSVC runtime packaging was not executable on this machine; its configure-time
requirements and package consumer checks are present but still require a
Windows CI or release-host run.

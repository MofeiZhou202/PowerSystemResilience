# Native Windows Experience Integration

## Scope and baseline

This work replays platform-independent findings from
`origin/release/windows-self-contained` onto macOS `main`; it does not merge
the release branch or replace the correctness fixes at `aef3be077ef2`.
CPLEX changes already present in the working tree are retained. The ordered
stages are benchmark telemetry, unified deadline propagation, LP cancellation
and timing evaluation, and finally concurrent-tree evaluation.

## Pre-registered rationale

Model and algorithm: the native MILP solver is an anytime branch-and-cut
method. One absolute `steady_clock` deadline is owned by the outer solve and
inherited by preflight, presolve, root LP, cut separation, primal heuristics,
and tree LPs. Parallel search is governed by one total thread budget rather
than multiplying explorer and nested LP/BLAS thread counts.

Claim: telemetry does not alter solver decisions. Deadline propagation bounds
every cooperative phase by the remaining outer budget and never publishes a
partial presolve transformation or an uncertified primal/dual result.
Concurrent tree search starts only after the shared root LP, cut pool, and
domain state are immutable and only when the remaining budget can amortize
worker startup.

Cost model: telemetry copies a constant number of scalar fields per solve and
serializes them outside the measured solve interval. Deadline polling is one
`steady_clock` read and comparison at reduction boundaries, O(1) per poll and
predicted below 0.5% of solve time. Effective parallelism is bounded by
`min(requested_threads, hardware_threads)`; nested LP kernels must not create
multiplicative oversubscription.

Quantitative predictions fixed before implementation:

- telemetry causes zero changes to objectives, statuses, node counts, or
  existing assertions for identical one-thread arguments;
- `30n20b8`, three-second limit, one thread, seed zero, and 5000 nodes returns
  within 4.5 seconds of measured solve time, reports a time-limit status when
  unfinished, and never reports an unproved result as proven;
- every case in the fixed twelve-case deadline cohort returns within 4.5
  seconds; a violation is a mismatch requiring phase-level investigation;
- the one-thread correctness suites have zero new failures;
- on `mas74`, four requested threads explore at least 20% more nodes than one
  requested thread at the same three-second budget. No wall-time speedup is
  predicted. `sct2` is retained as the root-serial control.

Assumptions: Release build, seed zero, relative gap `1e-4`, unchanged MIPLIB
files, and a sufficiently stable local macOS load. CPLEX is a quality and
termination comparison, not a correctness oracle; original-model row, bound,
integrality, and objective audits remain authoritative.

References: Achterberg, *Constraint Integer Programming* (2007), Sections
4.1-4.2 and 8.1; HiGHS root time-limit handling; and
`docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md` for root
ownership and postsolve contracts.

## Fixed validation protocol

1. Build `miplib2017_benchmark`, `test_branch_and_cut`, and
   `test_milp_solver` in Release mode and run both focused suites.
2. Run `30n20b8` with `--time-limit 3 --max-nodes 5000 --seeds 0
   --repeat 1 --native-threads 1`.
3. Run the twelve-case deadline cohort with the same flags and check the
   4.5-second measured-solve gate per case.
4. Run paired one/four-thread `mas74,sct2` measurements and record requested,
   effective, explorer, launched, and schedule-reason telemetry.
5. Record command, build flags, source identity, measured versus predicted
   values, and original-model audits. A wrong-sign result or deviation greater
   than 50% of the predicted effect triggers the repository mismatch protocol
   before further algorithm changes.

## Measurements

Measurements are appended only after each implementation stage is complete.

### Stage 1: benchmark telemetry

Build: macOS arm64, AppleClang Release, CPLEX ON from
`/Applications/CPLEX_Studio2211`, OpenMP ON, IPO and native-architecture
optimizations OFF; build directory `build/codex-cplex-macos-make`; source
identity `aef3be077ef2+dirty`.

The benchmark now accepts `--native-threads` and records the requested,
effective, explorer, launched, and schedule-reason fields in each native JSON
result. A CPLEX-ON smoke run on `50v-10` with a 0.2 second limit recorded
`requested=3`, `effective=1`, `explorers=0`, `tree_launched=false`, and
`schedule_reason=serial_until_incumbent`; CPLEX was reported separately and
passed its original-model audit. Native, CPLEX API, B&C, and MILP focused
suites passed (15, 175, 497, and 2049 assertions respectively).

This stage did not change native solver decisions. The telemetry prediction is
therefore satisfied for the tested one-thread path; the remaining risk is
covered by the serialization round-trip and later multi-thread measurements.

### Stage 2: unified deadline

The outer clock now starts before strict HiGHS preflight. Remaining budget is
passed to strict HiGHS, PaPILO/native presolve, root-owner HiGHS, and nested
native presolve; timeout paths retain the original model and do not publish a
partial certificate. The new native-presolve timeout regression passed.

The first replay exposed two deadline mismatches. `cbs-cta` spent a second
root-incumbent propagation resolve using the original full root budget; the
resolve is now transactional and receives only the remaining outer budget.
`neos-848589` spent about `12.7s` in non-cooperative HiGHS preflight and root
source separation before the deadline could be observed. The corrected path
uses a documented size gate (at most `100000` rows, `100000` columns, and
`1000000` nonzeros) to skip those optional passes for oversized models and
continues with the native path. This is an implementation-fidelity and
phase-ownership correction, recorded here before any constant tuning.

Measured results after the correction:

- `30n20b8`, three-second limit: native returned in `2.795s` with an explicit
  `Time limit reached` status, no incumbent, and no proof.
- `cbs-cta`: native returned in `3.165s` with an audited incumbent; the
  earlier `6.112s` replay is no longer representative.
- `neos-848589`: native returned in `2.826s`; the former approximately
  `12.7s` preflight overrun is gone.
- The fixed twelve-case cohort had no hard process timeout; its maximum native
  solve time was approximately `3.40s`, below the pre-registered `4.5s` gate.

The deadline prediction is therefore satisfied for the tested cohort. The
remaining non-cooperative interval risk belongs to LP cancellation and timing
instrumentation, which is intentionally deferred to Stage 3.

### CPLEX=ON paired validation

The comparison build is macOS arm64, AppleClang Release, `MIPSOLVERS_USE_CPLEX=ON`,
OpenMP ON, IPO and native-architecture optimizations OFF, using the callable
static library from `/Applications/CPLEX_Studio2211`; build directory:
`build/codex-cplex-macos-make`. Source identity for this dirty worktree is
`aef3be077ef2+dirty`.

Fixed command:

```bash
tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solvers native-highs-lp,cplex-mip \
  --case cbs-cta,neos-2075418-temuka,neos-848589,30n20b8,mas74,sct2 \
  --time-limit 3 --hard-timeout-grace 12 \
  --native-threads 1 --repeat 1 \
  --json /tmp/miplib_cplex_on_stage2.json
```

The run completed all 12 processes without a hard timeout. Native results had
3 audited incumbents and 0 incumbent-audit failures; CPLEX had 5 audited
incumbents, 2 proven optimal results (`cbs-cta` and `30n20b8`), and 0
incumbent-audit failures. Representative solve times were native/CPLEX:
`cbs-cta` `3165/317 ms`, `30n20b8` `2707/2153 ms`,
`neos-848589` `2826/3009 ms`, `mas74` `2703/3000 ms`, and
`sct2` `2702/3001 ms`; `neos-2075418-temuka` reached the three-second limit
without an incumbent in both solvers. CPLEX is used only as a termination and
quality comparison; the original-model row, bound, integrality, and objective
audits remain the native correctness gates.

This paired run also confirms that native telemetry and deadline fields remain
separate from CPLEX summaries. No concurrent-tree behavior was enabled.

## Main re-integration after Windows release update

Source baseline: `d7632c58`, macOS arm64 Release,
`MIPSOLVERS_USE_CPLEX=ON`, CPLEX Studio 22.1.1 from
`/Applications/CPLEX_Studio2211`, IPO and native-architecture flags off.

### Windows paths, telemetry, and hard deadline

The benchmark now supervises Windows workers with a Job Object configured with
`JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE`, creates each worker suspended, assigns it
before resuming, and terminates the full job at the absolute hard deadline.
Filesystem paths remain UTF-16 from `std::filesystem::path`; non-path command
arguments use strict UTF-8-to-UTF-16 conversion instead of the Windows active
code page. The benchmark uses a UTF-16 `wmain` entry point and reconstructs
path arguments from normalized UTF-8, so the worker does not round-trip paths
through the Windows active code page. Windows CI covers this contract with a
Unicode data-directory Job Object smoke test. The cross-platform CI wrapper
returned exit code 124 in `0.135361s`
for the fixed `0.1s` probe, below the pre-registered `0.6s` gate. The native
Windows Job Object path requires the new Windows CI run for direct validation.

The hosted Ubuntu/Windows correctness matrix intentionally disables CPLEX,
MKL, and optional external dependencies because those runners do not carry the
licensed installation. Local macOS validation remains CPLEX-enabled. The CI
does not include the self-hosted performance gate from the release branch.

The release branch's generated module manuals, persistent-session API, vendored
binary dependency tree, and self-hosted performance baselines are not copied as
part of this integration. They describe branch-specific code and deployment
topology that `main` does not yet expose, so importing the prose alone would
make the `main` documentation inaccurate. Likewise,
`docs/miplib2017_benchmark_protocol_2026-08-25.md` remains at its current path
because live `main` code and CPLEX documentation cite it there. Those artifacts
require separate implementation-aware reviews rather than a bulk documentation
merge.

### Unified cooperative deadline and LP cancellation

`SolveContext` owns one absolute `steady_clock` deadline across normalization,
dispatch, and fallback. Native B&C, embedded HiGHS, Ipopt, CPLEX, native dual
simplex, and native LP IPM receive only the remaining budget. A no-limit
CPLEX call retains the previous adapter path and defaults; the CPLEX API focused
gate passed 24 assertions, including both the unchanged default SolverEngine
path and an explicit deadline/thread/seed path.

The LP interface gate uses the identity
`T_return = T_first_result + T_cancel_join + T_bookkeeping`: a latency
portfolio may select the first audited answer, but it cannot return while a
losing worker still borrows the model. Telemetry is O(1) scalar recording and
does not alter pivots, tolerances, or factorization choices. The fixed protocol
is:

```bash
python3 tools/run_with_hard_deadline.py --timeout 120 -- \
  tests/netlib_solver_benchmark \
  --data-dir tests/data \
  --cases afiro,sc205,fit1p \
  --solvers native-auto-throughput,native-auto-latency \
  --threads 4 --time-limit 5 --repeat 3 \
  --json /tmp/mipsolvers_lp_portfolio_gate.json
```

Before running it, the acceptance prediction is 18 of 18 accurate
original-model results; throughput reports one worker with limit four and zero
cancellation wait; latency reports two workers with limit two; every run has
nonnegative timing fields and satisfies
`runtime >= first_result + cancel_wait` up to one nanosecond of representation
slack. No wall-time improvement is predicted or promoted by this interface
gate. A Windows policy recommendation still requires the stable-machine
release protocol.

The fixed run met the prediction: 18 of 18 results passed original-model
objective and feasibility audits. All nine throughput runs reported one worker,
limit four, and zero cancellation wait; all nine latency runs reported two
workers with limit two. There were zero timing-identity violations. Throughput
first-result time ranged from `0.000089s` to `0.113598s`; latency first-result
time ranged from `0.000145s` to `0.015553s`, while cancellation join wait ranged
from `0.000019s` to `0.143080s`. The large tail confirms the stated cooperative
cancellation limitation rather than supporting a hard in-process bound.

The native LP latency portfolio now joins both workers before returning.
Telemetry separates first-result latency from cooperative cancellation wait.
On the fixed one-row LP, the first result arrived in `0.000075s` and the losing
worker joined after another `0.000774s`; all six lifecycle assertions passed.
Throughput mode with a one-thread budget launched exactly one worker. These
measurements satisfy the correctness and ordering prediction, but they do not
establish a general cancellation-latency bound for noninterruptible sparse
factorizations. That requires the Windows NETLIB protocol before any default
thread recommendation.

For an explicit budget `N`, throughput mode applies the oneMKL thread-local
limit `N` to its single worker, while latency mode applies
`max(1, floor(N / 2))` to each of its two workers. This is a resource partition,
not a speedup claim; the existing fixed Windows NETLIB protocol remains the
acceptance gate for any default-policy change. The same per-worker value is
passed to native dual-simplex pricing, preventing its internal kernel from
silently multiplying the portfolio worker budget.

The post-partition focused run measured the four-thread throughput contract as
one worker with limit four and `0.000936s` runtime. The four-thread latency
contract measured two workers with limit two each, first result at
`0.000052s`, cancellation join wait `0.000022s`, and total runtime `0.000081s`.
Measured worker counts and limits match the prediction exactly; the timing is
reported as lifecycle evidence only, not as a general performance conclusion.

### Concurrent-tree evaluation and mismatch

Parallel tree search is now an explicit experiment: `num_threads > 1` alone
does not enable it, and the MIPLIB runner requires
`--native-concurrent-tree`. The disabled four-thread control reported
`requested=4`, `effective=1`, `explorers=0`, `tree_launched=false`, and
`schedule_reason=disabled_by_policy`.

The fixed three-second, 5000-node, seed-zero comparison measured:

| Case | Serial nodes / ms | Concurrent nodes / ms | Audit | Concurrent state |
|---|---:|---:|---|---|
| `mas74` | 831 / 2703.197 | 4000 / 3022.514 | pass / pass | 4 explorers, launched |
| `sct2` | 10 / 2701.381 | 69 / 3048.698 | pass / pass | 4 explorers, launched |

`mas74` increased explored nodes by `381.35%`, exceeding the predicted minimum
of 20%. However, `sct2` did not remain root-serial, so the control assumption
was false. Following the mismatch protocol, the implementation was checked
first (the opt-in reached the existing late-tree scheduler), then the model
assumption was rejected: this sample does not provide a valid serial control
under the current root schedule. No instance-specific gate or tuning was added.
Concurrent tree remains disabled by default pending a broader fixed cohort with
throughput, proof progress, deterministic audit, and oversubscription metrics.

### Final CPLEX-enabled regression

Command:

```bash
cmake --build build/codex-cplex-macos-make --parallel 6
ctest --test-dir build/codex-cplex-macos-make --output-on-failure -j 1
```

The first run was 19 of 20 tests in `2.33s`, with no new failure. The sole
failure was the pre-existing `Native NLP nonlinear multiplier initialization
preserves centrality` assertion: expected `0.02`, actual `0.002`. It was
reproduced before this change with fixed seed 1. History identifies an
implementation-fidelity mismatch in the regression itself: the test was added
when the barrier floor was `0.1 * tol_complementarity`; commit `e003dbb1`
changed the documented rule to one additional barrier decade,
`0.01 * tol_complementarity`, without updating the expected product. The test
must express `max(mu_min, 0.01 * tol_complementarity, mu_init)` directly. This
repairs the stale oracle and does not change the numerical algorithm or relax a
current tolerance.

After correcting that stale oracle, the same CPLEX-enabled command completed
20 of 20 tests in `16.45s`; the post-portability rerun completed 20 of 20 in
`2.50s`, the resource-partition cold rerun completed 20 of 20 in `18.14s`, and
the final current-worktree rerun completed 20 of 20 in `2.49s`.

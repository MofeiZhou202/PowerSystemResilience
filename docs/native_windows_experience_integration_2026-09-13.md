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

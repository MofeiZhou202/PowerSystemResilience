# MIPLIB 2017 MILP Solver Evaluation (2026-08-01)

## Scope

This evaluation establishes a reproducible MIPLIB 2017 benchmark path for:

- HiGHS 1.14.0 as a complete MIP solver;
- SCIP 9.0.0 as a complete MIP solver;
- Native branch-and-cut with the HiGHS node-LP kernel;
- Native branch-and-cut with the experimental Native node-LP kernel.

The machine was Apple ARM64 running Darwin 25.5.0. The project and benchmark
were compiled in Release mode with Apple Clang 21.0.0. Every solve requested
one thread, CLI seed 0, and relative MIP gap `1e-4`.

> **Provenance correction (2026-08-02).** The pre-D10 benchmark passed seed 0
> to HiGHS and SCIP, but did not copy `cfg.seed` into Native `BCOptions`.
> Consequently, both Native configurations below used the Native default
> `0x9E3779B97F4A7C15` for their heuristic RNG, not seed 0. A directly supplied
> zero also had inconsistent semantics in the then-current parallel
> work-stealing code. The historical JSON `"seed": 0` field records the CLI
> request, not the effective Native configuration. The objective/feasibility
> audits remain historical diagnostic evidence, but these reports do not
> establish an effective-zero-seed or fully replayable Native experiment.

The goal of the short runs below is diagnostic coverage, not a claim about the
official MIPLIB ranking. A publication-quality comparison still requires all
240 instances, substantially longer limits, repeated runs, and an external
hard-limit supervisor.

## Data and correctness contract

The official MIPLIB 2017 benchmark v2 archive contains 240 `.mps.gz` files.
The downloaded `benchmark.zip` SHA-256 is:

```text
c756eefd544d83b31809306b45d3549a1a5b9378e6aa78b68738b1a3b6a418fa
```

Reference statuses and objectives come from the official
`miplib2017-v36.solu`. Since the embedded HiGHS build does not read gzip
directly, the benchmark streams each instance to one temporary MPS file and
passes that identical file to all backends. Decompression, model reading, and
solve time are reported separately.

An incumbent is accepted only when all of the following pass at `1e-5`:

- original ranged rows and equality rows;
- original variable bounds, including semi-variable zero handling;
- integer feasibility;
- independently recomputed objective, including the MPS objective offset.

The quick run's accepted incumbents had maximum row, integrality, and bound
violations of `9.70e-7`, `2.82e-12`, and `9.11e-13`, respectively.

## Quick stratified run

Command:

```sh
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --sample 12 --time-limit 3 --gap 1e-4 --max-nodes 50000 \
  --csv reports/miplib2017_quick_2026-08-01.csv \
  --json reports/miplib2017_quick_2026-08-01.json
```

`--sample 12` deterministically selected names across the sorted 240-instance
set. Results use PAR-10 with a one-second shift; an unsolved 3-second run is
therefore charged 30 seconds.

| Solver configuration | Valid incumbent | Proven optimal | Timeouts | PAR-10 shifted mean |
|---|---:|---:|---:|---:|
| HiGHS-MIP | 5/12 | 0/12 | 12 | 30.000 s |
| SCIP-MIP | 6/12 | 1/12 | 11 | 22.288 s |
| Native B&C + HiGHS LP | 0/12 | 0/12 | 6 | 30.000 s |
| Native B&C + Native LP | 0/12 | 0/12 | 7 | 30.000 s |

SCIP proved `enlight_hard` optimal in 1.615 ms at objective 37. HiGHS found the
same incumbent but retained a 27.0% gap at three seconds. This single instance
is a useful reminder that MILP performance is structure-dependent and cannot
be reduced to an LP-kernel speed ratio.

Native failures were not ordinary timeouts:

- six runs rejected an invalid reduced-space/postsolve incumbent;
- five runs ended with `Search queue exhausted` without a valid incumbent or
  proof;
- `Native B&C + HiGHS LP` took 124.685 seconds on
  `neos-5093327-huahum`, 41.56 times its three-second limit.

The last result invalidates fair full-suite timing until Native has a hard
deadline contract. It must not be counted as a normal three-second timeout.

## Ten-second incumbent run

Eight smaller/traditional instances were tested with HiGHS and SCIP at ten
seconds: `50v-10`, `air05`, `binkar10_1`, `glass4`, `markshare2`, `nw04`,
`qap10`, and `uct-subprob`.

| Metric | HiGHS-MIP | SCIP-MIP |
|---|---:|---:|
| Valid incumbent | 7/8 | 8/8 |
| Proven optimal | 0/8 | 0/8 |
| Better final gap | 5/8 | 3/8 |

HiGHS had the better gap on `50v-10`, `air05`, `glass4`, `markshare2`, and
`uct-subprob`. SCIP led on `binkar10_1`, `nw04`, and `qap10`. SCIP was the only
backend to return an incumbent for `nw04`. HiGHS itself took 13.149 seconds on
that instance despite the ten-second internal limit, so final official runs
also need an external wall-clock supervisor for all backends.

Native was run separately on `50v-10` because a subsequent `air05` Native run
failed to honor the ten-second deadline and blocked the serial experiment:

| Configuration | Incumbent | Objective | Gap | Nodes | LP solves | Fallbacks |
|---|---:|---:|---:|---:|---:|---:|
| HiGHS-MIP | yes | 3543.3300 | 8.59% | 8 | n/a | n/a |
| SCIP-MIP | yes | 3876.7800 | 19.85% | 29 | 451 | n/a |
| Native B&C + HiGHS LP | yes | 6195.5900 | 49.97% | 369 | 846 | 0 |
| Native B&C + Native LP | yes | 6258.9700 | 50.60% | 650 | 1498 | 4/4 recovered |

The official optimum is 3311.1799841. Both Native incumbents passed the full
mathematical audit, but their primal quality and dual gaps are substantially
worse than the mature solvers at the same nominal limit.

## Conclusions

The current evidence does not support a claim that Native is competitive with
HiGHS-MIP or SCIP-MIP on general MILP, much less faster. Improving Native Dual
Simplex cold starts is insufficient: the observed blockers are above the LP
kernel and occur in presolve/postsolve, incumbent production, tree termination,
and deadline enforcement.

The required order of work is:

1. Enforce a hard wall-clock contract around every Native phase and subsolve.
2. Fix reduced-space incumbent/postsolve validation on the failing MIPLIB cases.
3. Make `Search queue exhausted` produce a valid proof status or an explicit
   failure, never an ambiguous completion.
4. Only after those gates pass, profile root LP, node reoptimization, cuts,
   branching, and heuristics against HiGHS/SCIP.

The full 240-instance command is ready:

```sh
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --time-limit 300 --gap 1e-4 --repeat 3 \
  --csv reports/miplib2017_full.csv \
  --json reports/miplib2017_full.json
```

Do not run that serial Native matrix unattended until the deadline defect is
fixed or each solve is isolated under an external process-level timeout.

## Deadline repair, stage 1

The first deadline repair was implemented after the diagnostic run. It found
two independent budget leaks:

- HiGHS presolve side-state and direct/live-append LP calls did not always
  receive the remaining MIP budget. A timed-out side-state result could also be
  cached and reused by a later solve with a larger budget.
- Large root cut re-solves selected Native IPM even under a three-second MIP
  limit. Native IPM checks time between iterations, but one Apple Accelerate
  sparse factorization is not interruptible; sampling the 124-second failure
  found the main thread inside that factorization.

The repair propagates remaining time into each HiGHS call, does not cache a
timed-out presolve result, checks the global deadline immediately after
presolve, and keeps short-budget large-root cut re-solves on the selected
deadline-aware simplex kernel. Optional analytic-centre work is also disabled
for limits of ten seconds or less.

Re-running the exact failing contract:

```sh
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --case neos-5093327-huahum --solvers native-highs-lp \
  --time-limit 3 --gap 1e-4 --max-nodes 50000
```

reduced solve time from 124.685 seconds to 3.682 seconds (33.9x), with the
honest status `Time limit reached`. The remaining 0.682-second overshoot is
model transformation/finalization latency outside an interruptible solver
loop. Therefore this is a successful removal of the catastrophic subsolve
budget leak, but not yet a process-level hard deadline. Full official runs
must still use an external supervisor until the residual overrun contract is
bounded across the 240-instance suite.

# CPLEX Callable Library integration and MILP comparison protocol

## Pre-registered rationale

Model and algorithm: an `MIPModel` represents a linear mixed-integer program
with objective `c'x`, boxed variables, two-sided inequality rows
`lhs <= A x <= rhs`, equality rows `Aeq x = beq`, and explicit binary/general
integer index sets. The adapter maps this model without reformulation to IBM
ILOG CPLEX 22.1.1 and invokes `CPXmipopt`.

Claim: using `CPXcopylp` with one column-compressed matrix preserves the
original variable order and mathematical feasible set. A finite upper side is
loaded as `A_i x <= rhs_i`, a finite lower side as `A_i x >= lhs_i`, and an
equality as `Aeq_i x = beq_i`. The original objective sense is passed as
`CPX_MIN` or `CPX_MAX`; therefore neither primal objectives nor dual bounds
requires sign conversion.

Cost model: the adapter allocates `O(n+m+nnz)` storage and scans each matrix
nonzero once, with a second emitted entry only when an inequality has two
finite sides. Import work is `O(n+m+nnz)` rather than dense `O(m*n)`. CPLEX
search dominates medium and large MILPs, so predicted adapter import overhead
is below 5% of import plus optimize time on those instances.

Quantitative prediction: all three small known-optimum MILP tests return their
existing objectives within `1e-4`. On the MIPLIB pilot, every CPLEX incumbent
passes the common `1e-5` original-model audit, the configured three-second
limit is reported as a time-limit status when reached, and adapter import is
below 5% of import plus optimize time for instances whose optimize phase lasts
at least one second. No direction is predicted for CPLEX versus HiGHS or the
native branch-and-cut because that comparison is the measurement objective.

Assumptions: CPLEX Studio 22.1.1 and a valid local license are available; runs
use one thread, seed zero, relative gap `1e-4`, the same time limit, and the
same `MIPModel`; semi-continuous and semi-integer columns are excluded because
the current public `MIPModel` cannot represent their disjunctive domains.

References:

- IBM ILOG CPLEX 22.1.1 Callable Library, `CPXcopylp`, `CPXcopyctype`,
  `CPXmipopt`, solution-status, objective, best-bound, gap and node-count APIs.
- Achterberg, *Constraint Integer Programming*, PhD thesis (2007), Sections
  4.1-4.2, for branch-and-bound incumbent, dual-bound and proof semantics.
- Gleixner et al., "MIPLIB 2017", Mathematical Programming Computation 13
  (2021), Section 3 and Appendix A.
- `docs/archive/miplib2017_benchmark_protocol_2026-08-25.md` for the repository's
  fixed audit, PAR-10 and shifted-geometric-mean definitions.

## Fixed validation protocol

1. Configure a fresh MSVC Release build with `MIPSOLVERS_USE_CPLEX=ON` and the
   detected Studio root, then build `test_engine_api`, `test_milp_solver`, and
   `miplib2017_benchmark`.
2. Run the engine and MILP tests, including the known-optimum binary,
   general-integer, and equality-constrained models.
3. Run `mas74` and `sct2` with CPLEX, HiGHS, and native HiGHS-LP branch-and-cut
   at one thread, seed zero, relative gap `1e-4`, and three seconds.
4. Reject every incumbent that fails the original `HighsLp` row, bound,
   integrality or objective audit. Record command, Release flags, commit,
   measured status, time, gap, node count, and import fraction.
5. If a measured effect has the wrong sign or differs from the import-overhead
   prediction by more than 50%, investigate implementation fidelity first,
   then machine/cost-model error, assumption violation, and theory error; add
   the finding below before changing the implementation.

## Measurements and mismatch record

Source commit before the working-tree integration was
`7fb0f10fd7a9fbaba621ed263ea42df6a87d6fe8`. The validated build was MSVC
19.44, x64, Release, `/MD`, IPO disabled, oneMKL sequential, CPLEX enabled,
Gurobi disabled, and source-built third-party dependencies. CMake detected
`C:/Program Files/IBM/ILOG/CPLEX_Studio2211/cplex/lib/x64_windows_msvc14/stat_mda/cplex2211.lib`.
The runtime used the vendor DLL directory on `PATH`; no IBM binary was copied
into the repository.

Build targets `test_engine_api`, `test_milp_solver`, and
`miplib2017_benchmark` linked successfully. With
`C:/Program Files/IBM/ILOG/CPLEX_Studio2211/cplex/bin/x64_win64` prepended to
`PATH`, the final two-test regression passed (2/2, 27.19 seconds); the focused
CPLEX slice passed 7 assertions in 2 cases, with Catch2 reporting 0.001 seconds
per case. Running without
that directory failed at Windows process loading with `0xc0000135`; this is a
deployment-path requirement, not a solver result.

Pilot command:

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir "$env:TEMP\mipsolvers-miplib-eval\sample" `
  --solu "$env:TEMP\mipsolvers-miplib-eval\miplib2017-v36.solu" `
  --solvers cplex-mip,highs-mip,native-highs-lp `
  --seeds 0 --repeat 1 --time-limit 3 --case mas74,sct2 `
  --json reports\miplib_cplex_pilot.json `
  --csv reports\miplib_cplex_pilot.csv
```

| Instance | Solver | Optimize ms | Nodes | Final gap | Audit | Status |
|---|---|---:|---:|---:|---|---|
| mas74 | CPLEX | 3005.8053 | 14110 | 10.3008% | pass | time limit |
| mas74 | HiGHS | 3004.2553 | 201 | 17.4666% | pass | time limit |
| mas74 | Native/HiGHS LP | 2722.6739 | 195 | 39.6664% | pass | time limit |
| sct2 | CPLEX | 3011.7691 | 605 | 0.02690% | pass | time limit |
| sct2 | HiGHS | 3015.7474 | 0 | 23.4491% | pass | time limit |
| sct2 | Native/HiGHS LP | 8738.6197 | 0 | 3.1466% | pass | time limit |

Measured CPLEX import was 0.1647 ms on `mas74` and 0.7557 ms on `sct2`, or
respectively 0.00548% and 0.02509% of import plus optimize time. Both satisfy
the pre-registered below-5% prediction by a wide margin, so the mismatch
protocol was not triggered. Both CPLEX incumbents passed the `1e-5` audit and
both three-second limits were reported as time-limit statuses, as predicted.

CPLEX produced the best final gap of the three paths on both pilot instances,
but neither instance was proven optimal. PAR-10 is consequently 30 seconds
for every path and the two-instance, one-seed pilot cannot establish parity
with Gurobi or general MIPLIB leadership. In addition, Native exceeded the
configured three-second wall limit on `sct2` (8.739 seconds), so its absolute
time is not a fair fixed-budget comparison. A broader stratified MIPLIB run
with process-level hard deadlines is required for a publishable performance
claim.

## Expanded local 12-instance protocol (fixed before execution)

The local sample directory contains exactly 12 uncompressed MIPLIB instances:
`30n20b8`, `cbs-cta`, `dws008-01`, `graph20-20-1rand`, `mas74`,
`neos-2075418-temuka`, `neos-4387871-tavua`, `neos-848589`, `ns1952667`,
`rail01`, `sct2`, and `supportcase40`. Run all 12 without result-based
selection, comparing `cplex-mip` and `highs-mip` at one thread, seed zero, one
repeat, relative gap `1e-4`, and three seconds. Solver order remains the
benchmark's per-instance block rotation. Native is excluded from this expanded
run because the Windows runner cannot enforce a process-level hard deadline and
the pilot already measured an 8.739-second run under a three-second setting.

Before execution, the only quantitative predictions remain: every returned
incumbent passes the common `1e-5` original-model audit, and CPLEX import is
below 5% of import plus optimize time whenever optimize lasts at least one
second. No prediction is made for solved count, gap, node count, or relative
CPLEX/HiGHS performance. Report feasible and proven counts separately; do not
rank solvers by node count because node definitions and internal work differ.

### Pre-publication mismatch investigation

The first expanded run returned a deterministic HiGHS `read error` on
`neos-2075418-temuka`. A focused verbose rerun reported `Free format reader
reached time_limit while parsing the input file` after about 3.03 seconds.
Implementation inspection found that `run_highs` set the three-second limit
before `readModel`, whereas `run_cplex` measured model import first and applied
the limit only inside `CPXmipopt`. This is implementation infidelity to the
benchmark's separately reported import/optimize cost model, not evidence about
HiGHS search performance.

Before changing code, the correction is fixed as follows: set the HiGHS time
limit immediately after successful `readModel` and before callback setup and
`run`. Prediction: `neos-2075418-temuka` import completes rather than returning
`read error`, its optimize phase runs for approximately three seconds, and all
returned incumbents still pass the unchanged `1e-5` audit. Read-heavy cases may
receive more actual search time, so the entire 12-instance comparison must be
rerun; no direction or magnitude is predicted for changed gaps or feasible
counts.

### Corrected expanded measurements

The focused post-fix rerun imported `neos-2075418-temuka` successfully and
entered HiGHS presolve; it then reported `Time limit reached` after 3.192
seconds of optimize time. This matches the correction prediction. The complete
corrected command was:

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir "$env:TEMP\mipsolvers-miplib-eval\sample" `
  --solu "$env:TEMP\mipsolvers-miplib-eval\miplib2017-v36.solu" `
  --solvers cplex-mip,highs-mip --seeds 0 --repeat 1 `
  --time-limit 3 `
  --json reports\miplib_cplex_12case.json `
  --csv reports\miplib_cplex_12case.csv
```

| Solver | Attempts | Proven | Feasible incumbents | Audit failures | PAR-10 shifted geomean |
|---|---:|---:|---:|---:|---:|
| CPLEX | 12 | 1 | 8 | 0 | 23460.1 ms |
| HiGHS | 12 | 0 | 5 | 0 | 30000.0 ms |

CPLEX proved `cbs-cta` optimal in 805.2 ms. Among the five instances where
both solvers returned an audited incumbent, CPLEX had the smaller final gap on
`cbs-cta`, `mas74`, `sct2`, and `supportcase40`; HiGHS had the smaller gap on
`graph20-20-1rand`. CPLEX additionally returned incumbents on `dws008-01`,
`neos-4387871-tavua`, and `neos-848589`, where HiGHS returned none.

Across CPLEX runs whose optimize phase lasted at least one second, the maximum
import fraction was 4.6122% on `neos-2075418-temuka`; the other fractions were
at most 2.0493%. This passes the pre-registered below-5% prediction without a
mismatch. All 13 incumbents across both solvers passed the original-model
`1e-5` audit; maximum row violation was `6.2371e-8` for CPLEX and `5.1479e-12`
for HiGHS.

Windows still lacks the benchmark's process-level hard deadline. CPLEX's
largest recorded optimize time was 3.712 seconds and HiGHS's was 10.033
seconds despite the three-second setting. The outcome therefore supports the
adapter's correctness and shows a favorable short-budget signal for CPLEX, but
it is not a strict equal-wall-time ranking, a multi-seed stability result, a
full MIPLIB assessment, or evidence of parity with Gurobi. Gurobi was disabled
in this build and was not measured.

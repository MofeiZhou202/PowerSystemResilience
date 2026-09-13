# General solver performance program (2026-09-13)

## Scope and fixed acceptance protocol

Baseline commit: `78939619f31ebe19b9399c8f52d5c35f6d6c06ba`.
Windows baseline: MSVC 19.44 Release, `/O2 /Ob2 /Oi /fp:fast`, Eigen
vectorization enabled, oneMKL INTEL threading, IPO/PGO/native-arch disabled.

This record fixes the model, predictions, assumptions, and validation gates
before implementation. A wrong-sign result or a deviation greater than 50%
of the predicted effect stops that experiment. Investigation order is
implementation fidelity, machine/cost-model error, assumption violation,
then theory error; the finding is appended here before another code change.

## R1: call-wide resource context and deadlines

- Model: a solve is one transaction with absolute steady-clock deadline `D`.
  Every adapter and fallback receives `max(0, D - now)`, never the original
  relative limit. This is the call-wide deadline model already used inside the
  native LP portfolio in `windows_remediation_2026-09-11.md`, R4.
- Cost model: context checks are O(1). They do not change mathematical work,
  pivot selection, tolerances, or objective values. A cooperative deadline can
  overrun by one non-interruptible backend operation; a hard deadline therefore
  belongs to a supervising process, not an in-process C++ thread.
- Prediction: the `dws008-01` three-second probe falls from 7.8739 seconds to
  soft P95 at most 3.30 seconds (at least 58% lower externally visible latency)
  once its active backend polls the deadline. Dispatcher fallback never starts
  after the deadline. Unlimited calls retain their current result and path.
- Assumptions: steady clock is monotonic; migrated adapters interpret zero as
  unlimited only when the context itself has no deadline; cancellation is
  cooperative; hard enforcement terminates an isolated process.
- Backend encoding: APIs that reserve a zero time limit for "unlimited" receive
  the smallest positive finite `double` when the absolute deadline expires
  between the dispatch check and parameter assignment. This preserves the
  mathematical limit without introducing a positive grace interval. SCIP,
  Ipopt, HiGHS, CPLEX, and Gurobi receive the remaining call-wide limit; their
  documented thread and deterministic-seed controls receive the same context.
- References: `windows_remediation_2026-09-11.md`, R4; C++20
  `[time.clock.steady]` and `[stoptoken]`.
- Validation: unit tests for expired contexts, external stop tokens, and a
  slow failing adapter followed by a successful adapter; full serial CTest;
  MIPLIB `dws008-01`, seed 0, one thread, three-second limit, repeated 20 times
  in alternating baseline/candidate process order. The hard-deadline gate is
  process exit no later than limit plus 10% or 0.30 seconds, whichever is
  larger.

## R2: latency and throughput portfolios

- Model: latency mode races independently audited kernels and minimizes
  `min(T_i)` at additional CPU cost. Throughput mode runs one robust kernel and
  minimizes CPU-seconds. A call-wide thread budget is divided among concurrent
  work and nested solvers.
- Prediction: throughput mode uses one LP worker and no more than the requested
  worker budget; latency mode uses two workers only with budget at least two.
  No wall-time speedup is predicted before repeated measurement. Historical
  evidence in manual section 9.7 shows direct IPM improving 4.53% with MKL/4
  while Auto regressed 10.44%, so one setting cannot be generalized.
- Validation: verify worker counts, CPU-seconds, wall time, P95, peak working
  set, solution audit, and deterministic fixed-seed repeats.

## R3: LP tail work

- Model: per-case Amdahl decomposition
  `T = T_assembly + T_symbolic + T_numeric + T_other`; retry time is a subset
  and is not added twice. Definitions follow `windows_remediation_2026-09-11.md`,
  R6.
- Predictions from the current one-shot diagnostic: reducing `dfl001` numeric
  time by 25% lowers total time 16.1%; halving `maros-r7` symbolic time lowers
  total time 12.8%; removing all measured `greenbea` retry time has only a 7.4%
  upper bound. These are hypotheses, not measured improvements.
- Constraints: no formula, threshold, or path change is allowed until 20-block
  baseline measurements identify a stable intervention. Native dual-simplex
  pure-performance changes require bit-identical pivot paths.
- References: `lp_tail_elimination_2026-08-18.md`; `windows_lp_stability_2026-09-11.md`.
- Validation: the manual section 9.7 alternating-process protocol, full NETLIB
  original-model audits, and full CTest. `greenbea` must include repeat-level
  trajectory and retry distributions.

## R4: persistent and incremental models

- Model: immutable matrix structure is compiled once; updates to objective,
  bounds, and right-hand sides are O(n + m) and reuse compatible presolve,
  basis, and symbolic state. Structural mutation invalidates the session.
- Prediction: remove one O(nnz+n+m) public-boundary copy and one model-sized
  peak-memory allocation per repeated solve. No wall-time percentage is
  predicted until workloads are measured.
- Validation: equivalence against cold solves after each update, deliberate
  dimension/structure mismatch failures, allocation/peak-working-set evidence,
  and rolling SCUC/OPF workloads.

## R5: compiler and floating-point experiments

- Model: IPO enables cross-translation-unit optimization; PGO changes layout
  and branch decisions using a training distribution; AVX2 widens eligible
  vector operations. `/fp:fast` can change reassociation and exceptional-value
  semantics, so `/fp:precise` is the correctness control.
- Prediction: no option is promoted unless its held-out median improves by at
  least 5% with a confidence interval excluding zero. No accuracy, certificate,
  deadline, or determinism gate may regress.
- Validation: one option at a time, interleaved A/B repetitions, separate
  training and held-out suites, exact build flags and binary hashes. Pure
  dual-simplex performance changes retain bit-identical pivot paths.

## R2/R3 baseline result and mismatch review

Command:

```powershell
python tools/windows_lp_stability.py --stage stability `
  --output reports/windows_performance_program_20260913_baseline
```

Build identity is the R1 working tree over commit `78939619`; Release flags and
libraries are listed at the top of this record. Both thread settings produced
60/60 accurate original-model results. The three-case aggregate medians were
8669.21 ms at two threads and 8569.57 ms at four threads, a 1.15% reduction.
The pre-existing five-percent promotion gate therefore rejected four threads
as a general default, although its per-case P95 non-regression gate passed.

The result differs materially from the historical 18.50% aggregate reduction
in `windows_lp_stability_2026-09-11.md`, so the mismatch protocol applies:

1. Implementation fidelity: recorded `mkl_max_threads`, binary provenance, and
   120/120 total accurate results establish that the intended configurations
   ran. `dfl001` and `maros-r7` retained fixed 44/21 iteration counts.
2. Machine/cost model: `dfl001` numeric median fell 3204.29 to 2212.54 ms and
   total median fell 4353.33 to 3351.69 ms. Its Amdahl direction is confirmed.
   `maros-r7` fell 1257.36 to 1143.84 ms.
3. Assumption violation: `greenbea` iteration sets were 40--400 at two threads
   and 42--415 at four threads. Its median rose 3056.02 to 4019.19 ms and P95
   rose 5628.84 to 5797.55 ms. Fixed-work decomposition is invalid for this
   case; trajectory variance erased the factorization gains.
4. Revised theory: thread scaling may be selected for fixed-trajectory cases,
   but it is not a general portfolio policy. `greenbea` requires a separate
   path/recovery variance model before any threshold change. Default remains
   two threads. No LP algorithm or numerical threshold is modified in this
   stage.

Machine-readable distributions, process memory, raw logs, and factor records
are under `reports/windows_performance_program_20260913_baseline/`.

## R1/R2/R4 implementation and validation status

Implementation over baseline commit `78939619f31ebe19b9399c8f52d5c35f6d6c06ba`
uses one `SolveContext` created before normalization and validation. The same
absolute steady-clock deadline, stop token, resolved thread budget, seed, and
advisory memory request reaches every dispatcher candidate. Backend APIs receive
only the remaining time; the dispatcher does not start a fallback after expiry.
The in-process API reports `hard_deadline_enforced=false` and
`memory_limit_enforced=false`; hard enforcement is the process-tree supervisor
in `tools/run_with_hard_deadline.py`.

NativeAutoLP now has two explicit modes under the one call-wide budget.
Throughput runs one direct-IPM worker with the complete budget. Latency starts
two workers only when the budget is at least two and limits each worker's MKL
nested parallelism to `floor(budget/2)`. Result telemetry records both the
actual portfolio worker count and the per-worker nested-thread limit, so the
budget claim is testable rather than inferred from configuration.

`LPModelSession` retains a validated public LP and one embedded HiGHS model.
For explicit HiGHS with fallback disabled, objective, variable-bound, ranged-row,
and equality-RHS updates use HiGHS incremental change calls and retain compatible
basis state. Invalid updates restore the public model; a rejected backend update
also rebuilds the resident backend before throwing. Other solver policies reuse
the public model and standard dispatcher without an extra normalization copy.

Validation commands and measured results:

```powershell
cmake --build build\windows-msvc-cplex --config Release `
  --parallel 4 -- /nodeReuse:false
tests\Release\test_engine_api.exe
python tools\run_with_hard_deadline.py --timeout 0.1 -- `
  powershell -NoProfile -Command "Start-Sleep -Seconds 5"
python benchmark\check_lp_release_gate.py `
  --result reports\windows_performance_program_20260913_baseline\summary.json `
  --baseline benchmark\windows_lp_release_baseline.json
ctest --test-dir build\windows-msvc-cplex -C Release `
  -R "native_kernel_comparison|test_milp_solver|test_dual_simplex|test_l2o_trace" `
  --output-on-failure -j 1
python tools\run_with_hard_deadline.py --timeout 1800 -- `
  ctest --test-dir build\windows-msvc-cplex -C Release `
  --output-on-failure -j 1
ctest --test-dir build\windows-msvc-cplex -C Release `
  -R "milp_benchmark(_regression)?$" --output-on-failure -j 1
```

The build used MSVC 19.44 Release with the CPLEX configuration and completed.
The final API suite passed 57/57 cases and 339/339 assertions, including the
portfolio worker-count and per-worker thread-limit telemetry checks. The
hard-deadline probe returned 124 after 0.1 seconds. The LP release baseline gate
passed all case and aggregate median/P95 and fixed-iteration checks. The clean
single-writer targeted reproduction passed 4/4 tests, the process-supervised
serial CTest passed 21/21 tests in 14.34 seconds, and the dedicated MILP
benchmark/regression gate passed 2/2 tests. The requested `dws008-01` 20-repeat
three-second validation remains unavailable because that dataset is not in this
checkout. No wall-time speedup is claimed for R1, R2, or R4 until the specified
workload measurements are run.

R3 remains stopped under the mismatch protocol above. In particular, no
`greenbea` threshold, recovery, or numerical-path change is admitted until a
repeat-level path/recovery variance model explains the wrong-sign four-thread
result. `dfl001` numeric and `maros-r7` symbolic/numeric interventions likewise
require their fixed 20-block baselines and bit-identical pivot-path contracts.
R5 (IPO, PGO, AVX2, and floating-point mode) remains last and unpromoted; it is
not valid to evaluate those options before the algorithmic and API gates are
stable.

## R2 mismatch: automatic budget must not force backend work

The first fully relinked serial CTest run found a wrong-sign correctness result:
`native_kernel_comparison` crashed and `test_milp_solver` reported failed root
relaxations with repeated HiGHS LPI errors, while the API suite remained green.
Per the mismatch protocol, performance work stopped before another algorithmic
change.

1. Implementation fidelity: `SolveContext` resolved `threads=0` to hardware
   concurrency as designed, but the Native B&C adapters unconditionally copied
   that resolved maximum into `BCOptions::num_threads`. This replaced the
   backend's `-1` automatic-policy sentinel and therefore changed scheduling
   and root-LP behavior; a maximum resource budget was incorrectly implemented
   as a demand to consume the entire budget.
2. Machine/cost model: the failure occurs before a performance comparison and
   is accompanied by backend state errors, so it is not explained by timing
   noise or the MKL cost model.
3. Assumption violation: the original implementation assumed that setting an
   adapter thread parameter to the global budget is behaviorally equivalent to
   leaving its automatic policy in control. Native B&C uses the sentinel in
   policy decisions, so that assumption is false.
4. Revised theory: a thread budget is an upper bound. An automatic backend
   configuration remains automatic when the caller supplied `threads=0`; an
   explicit positive call-wide budget caps any configured positive count and
   replaces an unbounded/automatic sentinel. Portfolio workers still divide the
   resolved budget because their concurrency is owned by the dispatcher.

Validation is fixed before the correction: rebuild all Release targets, rerun
the API portfolio assertions, reproduce the previously failing MILP and native
kernel tests, then rerun serial CTest under the process hard deadline. No
numerical formula, tolerance, pivot, or recovery rule may change in this fix.

Follow-up implementation-fidelity audit: the completed CTest run was 17/21,
but the additional failures included `bad allocation` in the direct dual test,
`vector too long` in L2O, and segmentation faults in unrelated executables.
Process inspection showed overlapping MSBuild trees had written the same output
directory while the public `SolveStats` layout was changing. The resulting
cross-module pattern is consistent with mixed/stale ABI artifacts and is not
evidence that the thread-budget hypothesis alone caused every failure. The
budget-as-upper-bound correction remains required by the resource model, but
its causal effect will be evaluated only after a single-writer clean rebuild.
The contaminated 17/21 run is retained as a failed implementation-fidelity
check and must not be used as a performance or algorithm result.

The fixed validation protocol then completed after deleting generated build
artifacts and performing one clean, single-writer Release build with MSBuild
node reuse disabled. The four previously failing executables passed in serial,
and the full hard-deadline-supervised serial CTest passed 21/21. This resolves
the cross-module crashes as build contamination. It does not reverse the
resource-model correction: the explicit portfolio assertions separately prove
the budget-as-upper-bound semantics, while no LP formula, tolerance, pivot, or
recovery rule changed.

A final resource-path audit found the same implementation-fidelity issue in the
persistent HiGHS path: it wrote the resolved hardware capacity into the backend
even when the caller specified `threads=0`. The correction maps automatic mode
to the untouched HiGHS default, rebuilds the resident backend when crossing
between automatic and explicit budgets, and retains it when the mode and budget
are unchanged. The fixed regression sequence automatic -> one thread ->
automatic passed, and the final API/full-suite counts above include it. This
changes no numerical rule and is the R2 upper-bound model applied consistently.

The same audit found the external HiGHS context overloads were conservatively
forcing one thread for automatic calls, even though HiGHS documents
`threads=0` as its automatic setting. A controlled attempt to pass zero was
stopped immediately under the mismatch rule: after rebuilding, the full serial
CTest run changed from 21/21 to 20/21, with `test_milp_solver` reporting 13
failed cases and repeated HiGHS LPI error `-6`; the other 20 tests passed. This
is an error-direction correctness regression, not a performance result.

Implementation-fidelity review found that the persistent automatic session and
Native B&C share HiGHS' process-global scheduler. Leaving the option at zero
changes the scheduler state observed by the subsequent B&C root-LP lifecycle;
the existing one-thread context setting is therefore an intentional isolation
barrier, despite HiGHS' standalone automatic semantics. Machine/cost-model and
assumption review is not entered because correctness already disproves the
zero-thread intervention. The code is restored to one thread for the external
HiGHS context overloads, with no LP arithmetic or tolerance change. Future
automatic-policy work must first introduce an explicit scheduler ownership and
reset contract, then repeat the fixed full-suite protocol.

The CI diff gate also initially contained its own prohibited marker literals,
so the first commit containing the workflow would have rejected itself. The
pattern is now assembled from non-matching shell fragments before scanning the
added lines. YAML 1.2 parsing and a local self-match scan both pass; hosted
GitHub expression and runner validation remain CI-only checks.

The release workflow's first path audit found another pre-execution failure:
the Ninja job emits benchmark executables to `tests/`, while the Windows
measurement protocol historically assumed the Visual Studio multi-config path
`tests/Release/`. The protocol now accepts an explicit `--binary-dir`, validates
all binaries required by the selected stage before measurement, and records the
resolved directory plus every binary hash. The workflow passes `tests`; the
historical multi-config default remains `tests/Release`. A valid-directory
summary probe passed and a missing-directory probe failed before execution as
required.

The generic adapter ABI retains legacy virtual methods by defaulting each
context overload to the old method. This is source-compatible but cannot inject
cooperative limits into an adapter that has not opted into the new overloads.
That boundary is now explicit in the API manual and is a release-review item:
third-party production adapters must implement context-aware methods or run in
an externally supervised worker. No built-in adapter used by the registered
engine paths is exempt from this requirement.

## R3 greenbea path/recovery variance model

The retained 20-block factor records permit a two-component model without a
new algorithm experiment. Every run has two measured variants. The first is a
stable recovery cost with 79 retries at two threads and 65 at four threads; the
second carries the variable convergence trajectory. For thread count `p`, use

`T_greenbea(p) = F_recovery(p) + alpha(p) + beta(p) * I_second + epsilon`,

where `I_second` is the returned second-variant iteration count. This separates
retry time (already included in `F_recovery`) from trajectory work and prevents
double counting.

Measured from
`reports/windows_performance_program_20260913_baseline/block-*.{json,factors.json}`:

| threads | median total ms | median recovery variant ms | median recovery numeric ms | median retry ms | median second iterations | P(iterations > 100) | second beta ms/iter | corr(iterations, second time) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 3056.02 | 2332.82 | 937.16 | 269.59 | 50.0 | 0.40 | 8.49 | 0.9925 |
| 4 | 4019.19 | 2850.11 | 1097.28 | 258.19 | 97.5 | 0.50 | 7.35 | 0.9921 |

The wrong-sign result has two distinct causes. Four threads reduce neither the
fixed recovery variant nor its numeric component: their medians increase 22.2%
and 17.1%, despite retry time falling 4.2%. Separately, the second-variant
iteration distribution worsens; its median rises 95% and the probability of a
run above 100 iterations rises by 10 percentage points. The nearly unit
correlation confirms that second-variant wall time is a trajectory proxy, while
the fixed 65/79 retry counts show that aggregate retry removal alone cannot
explain the tail.

Revised intervention contract: do not promote four threads for this matrix
shape, and do not tune a retry threshold from aggregate wall time. Before an
algorithmic change, add repeat-level telemetry for recovery reason/path id,
regularization escalation and decay counts, audit-rejection reason, and the
iteration at each formulation transition. A candidate must predict its effect
separately on `F_recovery` and the empirical high-iteration probability, retain
full original-model accuracy, and preserve paths for all non-greenbea cases.

## R3 candidate: bounded parallelism for numerical recovery

The first recovery-only intervention is deliberately limited to the generic
normal-equations rejection boundary.  When an automatic normal-equations
trajectory reports `Normal equations stalled` or `Normal equations rejected`,
the outer solver already performs a clean augmented-KKT restart.  The restart
is numerically sensitive: the fixed 20-block records show that four MKL
threads reduce per-iteration factor time but change the augmented trajectory
from a median of 50 to 97.5 iterations, while two threads retain the shorter
trajectory.  This is consistent with parallel reduction roundoff moving the
regularization/step decisions across a branch boundary; it is not a change to
the KKT equations.

**Model and prediction (fixed before implementation).**  Apply a local MKL
thread limit of `min(current_limit, 2)` only around the augmented recovery
solve, and leave the initial solve and all non-recovery calls untouched.  The
cost model is `T = F_recovery(p) + beta(p) I`; the observed `beta` is 7.35
ms/iteration at four threads and 8.49 ms/iteration at two threads.  If the
recovery iteration distribution returns to the two-thread median, the
greenbea four-thread median is predicted to fall from 4019 ms to 3000--3300
ms (17--25%); the aggregate three-case median is predicted to improve by
approximately 8--12%.  No change is predicted for `dfl001` or `maros-r7`,
because neither enters the normal-rejection recovery branch.  Accuracy and
all non-greenbea trajectories are predicted bit-for-bit unchanged; this is a
resource-policy change, not an arithmetic or pivot rule.

**Assumptions and references.**  The recovery state is already a clean
augmented restart (Wächter--Biegler 2006, Sections 3.1 and 3.3); MKL's local
thread limit is scoped to the calling thread (`mkl_set_num_threads_local`,
oneMKL Developer Reference).  The assumption under test is that the wrong-sign
effect is caused by parallel reduction sensitivity in the recovery trajectory,
not by the normal solve or by the factorization cost model.

**Fixed validation protocol.**  Build the same MSVC Release binary, then run
the 20-block stability command with `MKL_NUM_THREADS` 2/4, followed by the
full Release CTest and the dedicated MILP gates.  Accept only if all 60/60
R3 runs remain accurate, `dfl001` remains 44 iterations and `maros-r7` remains
21 iterations in every block, greenbea's four-thread median is within the
3000--3300 ms prediction, and no dual-simplex pivot trace changes.  A wrong
sign or a deviation greater than 50% of the predicted improvement stops
tuning immediately and requires implementation-fidelity, machine-cost,
assumption, then theory review in this document.

The preceding CBWR=COMPATIBLE probe is rejected: it made both thread settings
deterministically take 154 iterations and about 7.3 s, a material regression
from the unconstrained 20-block medians.  It is retained as a cost-model
mismatch, not as an implementation candidate.

### R3 candidate result (2026-09-13)

The candidate was implemented as a scoped `ScopedMklThreadLimit` around the
normal-rejection augmented restart.  The Release build was reconfigured with
the staged oneMKL INTEL threading layer and rebuilt single-writer after an
earlier overlapping-build attempt was discarded from the measurement set.
The fixed command was:

```powershell
python tools\run_with_hard_deadline.py --timeout 1800 -- `
  python tools\windows_lp_stability.py --stage stability --output `
  reports\r3_cap2_intel_20260913
```

Measured 20-block medians (MSVC 19.44 Release, oneMKL INTEL, commit recorded
in each block provenance) were:

| case | 2 threads | 4 threads | iterations (2/4) |
|---|---:|---:|---:|
| `dfl001` | 3369.37 ms | 2266.88 ms | 44 / 44 |
| `greenbea` | 3489.81 ms | 2834.69 ms | variable / variable |
| `maros-r7` | 1091.68 ms | 933.24 ms | 21 / 21 |
| aggregate | 7935.32 ms | 6052.84 ms | 60/60 accurate |

The four-thread greenbea median is 2835 ms, below the 3000--3300 ms predicted
range.  Relative to the two-thread arm it improves 18.8%; relative to the
pre-intervention four-thread median of 4019 ms it improves 29.5%.  Taking the
midpoint prediction (20.8%) gives a 42% relative prediction error, below the
50% stop threshold; the sign is correct.  The greenbea high-iteration tail is
reduced but not eliminated (the 20-block set still contains 130--318-iteration
runs), so no claim of fixed trajectory is made.  `dfl001` and `maros-r7` retain
their fixed 44/21 IPM iteration contracts.  The patch does not touch the
dual-simplex kernel and the full `test_dual_simplex` gate passes, but that is
not a cross-binary pivot trace.  The bit-identical dual-simplex claim is
therefore limited to source-scope preservation until an old/new trace artifact
is available; IPM iteration equality is not used as a substitute for it.

### R3 recovery telemetry extension

Before further tuning, the factor record is extended with the solve-entry
reason, the source iteration at a formulation transition, and the formulation
actually selected.  This follows the two-component model above: separate
records must identify `primary` normal work from a `normal_stalled`,
`normal_rejected`, or factorization-failure augmented recovery.  The change is
observational only and is predicted to add less than 0.1% wall time because it
writes three scalar fields into the already-enabled diagnostic record once per
variant.  With `MIPSOLVERS_LP_FACTOR_TIMING=0`, the path remains unchanged.
Validation is fixed as JSON parsing of every 20-block factor record, exact
closure of the existing exclusive timing partition, and the full Release test
suite.  The reference is the measurement decomposition in this document's R3
model; no numerical formula, tolerance, or iteration decision reads the new
fields.

### R3 release-gate mismatch audit

The schema-v1 stable-hardware release gate deliberately reads the baseline's
fixed `thread_budget=2`; it does not consume the stability report's
`decision.recommended_threads`.  Against baseline commit
`78939619f31ebe19b9399c8f52d5c35f6d6c06ba`, the candidate's two-thread
`greenbea` median is 3489.812 ms.  This exceeds the 3056.023 ms baseline times
the fixed 1.10 ceiling (3361.625 ms) by 128.187 ms, or 3.81%.  All other case
medians/P95 values, both aggregate limits, 60/60 two-thread accuracy, and the
fixed 44/21 iteration sets pass.  The candidate is therefore not release-gate
clean even though the four-thread policy result has the correct sign.

The mandatory mismatch review is:

1. **Implementation fidelity.**  The bounded scope is entered only for a fresh
   augmented factorization and preserves a caller limit below two; the later
   telemetry audit below clarifies that its primal seed can still be retained.
   The full test suite and original-model accuracy pass; no evidence identifies
   an arithmetic or tolerance change in the primary two-thread solve.
2. **Machine/cost model.**  The candidate batch uses oneMKL INTEL threading,
   while absolute `greenbea` time has substantial process-to-process variance
   (two-thread IQR/median 46.6%).  A 3.81% ceiling exceedance is smaller than
   this observed dispersion and cannot by itself identify a deterministic code
   regression, but the gate result remains a failure.
3. **Assumption.**  The performance prediction targeted the former wrong-sign
   four-thread recovery.  It did not predict that the historical two-thread
   control distribution would move.  Treating a recommendation gate as a
   replacement for the control gate would silently weaken the release contract.
4. **Theory/gate definition.**  Retain the schema-v1 two-thread control gate and
   its thresholds.  A separate policy gate may verify that the eligible
   four-thread arm is accurate and improves the paired aggregate distribution;
   it cannot waive a failed control arm.  No threshold or baseline is changed
   from this batch.

The next admissible measurement is one fresh 20-block run from the telemetry-
complete binary under the already fixed protocol.  This is validation of the
observational extension, not parameter tuning.  If its two-thread control still
fails, R3 remains blocked from release and any new intervention requires a new
pre-implementation model and prediction here.

### R3 telemetry-complete revalidation and stop (2026-09-13)

The admissible revalidation used the command above with output directory
`reports/r3_cap2_intel_telemetry_20260913`, commit
`1e61b2434b07f2bb5a71839048e71fea97ccf586`, and benchmark binary SHA256
`35C3C4525D79707F0537D5FB352F7A20740520851623C3A118264C6AE639AD5A`.
The build remained MSVC 19.44 Release with `/O2 /Ob2 /Oi /fp:fast`, oneMKL
INTEL threading, and IPO/PGO/native-arch disabled.  All 120/120 case-runs were
accurate, all 40 `dfl001` runs used 44 iterations, and all 40 `maros-r7` runs
used 21 iterations.  All 168 factor records passed the telemetry schema and
exclusive-time closure checks.

| case | 2-thread median/P95 ms | 4-thread median/P95 ms | fixed iterations |
|---|---:|---:|---:|
| `dfl001` | 3383.68 / 3489.92 | 2305.82 / 2417.44 | 44 / 44 |
| `greenbea` | 2637.62 / 4806.21 | 2766.03 / 4157.77 | variable |
| `maros-r7` | 1089.73 / 1113.10 | 915.47 / 955.58 | 21 / 21 |
| aggregate | 7097.64 / 9406.23 | 5998.60 / 7374.65 | 60/60 each arm |

The unchanged schema-v1 two-thread release gate passes this fresh batch.
Nevertheless, the specific R3 `greenbea` median objective has the wrong sign:
four threads are 4.87% slower, the paired median delta is +147.18 ms, and the
four-thread arm wins only 8/20 pairs.  Its P95 improves 13.49% and the aggregate
median improves 15.48%, so tail and portfolio conclusions must not be conflated
with the per-case median contract.  This contradictory batch invalidates the
candidate's claim of a stable `greenbea` median fix and triggers the stop rule.

The ordered mismatch investigation is now conclusive at the first layer:

1. **Implementation fidelity.**  Telemetry shows every `greenbea` run first
   enters normal equations and then enters `factorization_failed` augmented
   recovery.  The normal phase stops at iteration 169 for every two-thread run
   and 181 for every four-thread run.  The outer code then passes that failed
   phase's finite `res.x` into recovery.  The restart has a fresh augmented
   factorization, but it is not a clean, thread-independent state as assumed by
   the candidate model.  Capping only the recovery is therefore too late to
   make its input trajectory invariant.
2. **Machine/cost model.**  Median normal-phase time is 1910.80 ms at two
   threads and 2124.55 ms at four (+11.19%); median augmented recovery time is
   720.25 and 639.87 ms (-11.16%).  The intended recovery saving is real but is
   outweighed in the median by the longer upstream four-thread trajectory.
3. **Assumptions.**  The earlier model treated recovery work and its iteration
   count as though the local thread cap controlled the full sensitive state.
   The retained primal seed violates that separation.  The new arm has fewer
   runs above 100 returned iterations (7/20 versus 9/20), yet its low-mode base
   cost remains higher; iteration-tail probability alone does not determine the
   median sign.
4. **Theory.**  Wächter--Biegler's restoration principle does not imply that a
   retained primal iterate is independent of the arithmetic path that produced
   it.  A future candidate must model the normal-phase state transition and the
   augmented recovery jointly.  Discarding the seed is not presumed valid: the
   archived H2-prime experiments already show a cold augmented-start barrier.

No further threshold, thread-cap, regularization, or seed tuning is admissible
under R3 until a new derivation predicts both normal-phase and recovery costs
and fixes a validation rule for the median sign.  The telemetry extension and
fixed-iteration evidence remain valid; the bounded-recovery cap remains an
unaccepted candidate rather than a release result.

## R3 joint normal-phase and retained-seed model

The next phase is observational.  It does not alter formulation selection,
regularization, tolerances, or the unaccepted bounded-recovery cap.  For a
normal-to-augmented transition at requested thread count `p`, use

`T(p) = T_N(p, I_N, R_N) + T_A(2, x_seed(p), I_A) + epsilon`,

where `I_N` is the source normal iteration, `R_N` is its factorization-retry
count, `x_seed` is the exact finite primal vector retained by the outer solver,
and `I_A` is the augmented recovery iteration count.  Decompose the measured
terms as

`T_N = a_N(p) + b_N(p) I_N + c_N(p) R_N`,

`T_A = a_A(2, B) + b_A(2, B) I_A + g(q(x_seed))`,

with seed-quality observation

`q(x_seed) = (||x||_inf, ||x||_2, c^T x, r_p, r_d, mu, rho_p, rho_d, rho_g)`.

where `B` is the ordered sequence of adaptive PARDISO backend selections.
Here `(r_p,r_d,mu)` are the scaled termination metrics and
`(rho_p,rho_d,rho_g)` are the original-model relative audit residuals/gap.
The model does not assume that these nine scalars uniquely determine recovery;
an FNV-1a fingerprint over the exact IEEE-754 seed bits identifies equality or
inequality of retained states without entering any numerical decision.  The
fingerprint is diagnostic identity, not a distance metric.

**Claim and cost model.**  A structured transition record emitted after each
recovery will join source reason/iterations/runtime and seed state to recovery
iterations/runtime/success.  When diagnostics are disabled, no seed traversal
or formatting occurs.  When enabled, the added work is one `O(n)` sequential
read (5405 doubles for `greenbea`) and one bounded JSON line per transition,
against hundreds of sparse factorization calls.  The predicted incremental
cost is below 0.5 ms per transition, below 0.02% of the observed 2.6 s solve.
Because solve-level timing/no-timing pairs are dominated by the multi-second
trajectory mixture, the record separately measures seed traversal and JSON
preparation.  Their combined median, excluding the OS-controlled stderr flush,
must be below 0.5 ms; paired solve totals remain a coarse regression check.

The adaptive recovery first probes LDLT/LU and may rebuild the portfolio after
pivot instability.  A pre-instrumentation verbose probe selected LU for the
unregularized probe (144 versus 9 perturbed pivots) and LDLT after stabilization
(zero perturbed pivots).  The backend sequence is therefore added as one byte
per adaptive selection.  `UL` is predicted to dominate the `greenbea` sample.
If every run has the same sequence while recovery iterations remain dispersed,
backend policy is excluded as the within-arm random variable; if the sequence
varies and separates the tail, the wall-clock tie-break remains causal.

**Predictions and assumptions fixed before implementation.**  The observed
source iteration sets remain exactly `{169}` at two threads and `{181}` at four
threads; their retained-seed fingerprints differ across thread arms.  No
within-arm fingerprint determinism is assumed because the threaded reduction
order is itself under investigation.  `dfl001` remains 44 iterations and
`maros-r7` remains 21.  The retained primal vector is assumed to be the only
iterate state crossing the formulation boundary; dual and barrier variables
are rebuilt by the augmented solve.  Existing failure-exit stats and the final
original-model audit are assumed valid observations, not acceptance signals.

**References.**  The restoration/reinitialization model follows
Wächter--Biegler (2006), Sections 3.1 and 3.3, while explicitly not assuming
that a retained primal point is arithmetic-path independent.  FNV-1a is used
only as the repository's established bit-fingerprint convention.  The state
and cost decomposition is the corrected R3 model above.

**Fixed validation protocol.**  Rebuild the same MSVC 19.44 Release profile
(`/O2 /Ob2 /Oi /fp:fast`, oneMKL INTEL, IPO/PGO/native-arch off), then execute
the existing 20-block command into a new evidence directory.  Require 120/120
accuracy, exact 44/21 iteration sets for `dfl001`/`maros-r7`, one recovery
record for every non-primary factor record, valid finite-or-null metrics, and
the predicted 169/181 source sets.  Run the unchanged two-thread release gate,
full Release CTest, and the forbidden-marker/diff checks.  A trajectory change,
wrong fingerprint prediction, or material overhead triggers the ordered
mismatch protocol before a policy candidate is proposed.

### Joint-model measurement result

The telemetry-complete 20-block command used output directory
`reports/r3_joint_backend_model_20260913`, commit
`1e61b2434b07f2bb5a71839048e71fea97ccf586`, and benchmark SHA256
`08D1F4EEE980C902CEB480C94A9712586643AFB20990552AC739DCCFF4360BA3`.
All 120/120 case-runs were accurate.  `dfl001` remained 44 iterations and
`maros-r7` remained 21 in all 40 runs each.  The unchanged two-thread release
gate passed.

| observation | 2 threads | 4 threads |
|---|---:|---:|
| normal source iteration | 169 (20/20) | 181 (20/20) |
| unique retained-seed hashes | 1 (`ba675ba0af6c2f27`) | 1 (`5a66d14e69c8411e`) |
| adaptive backend sequence | `UL` (20/20) | `UL` (20/20) |
| normal source median ms | 1920.93 | 2173.59 |
| recovery iterations median / range | 48.5 / 41--357 | 48.5 / 42--365 |
| recovery median / P95 ms | 626.47 / 2555.24 | 640.62 / 2499.07 |
| telemetry median / maximum ms | 0.121 / 0.244 | 0.128 / 0.187 |
| `greenbea` median / P95 ms | 2551.45 / 4509.72 | 2911.00 / 4657.87 |

The seed and backend predictions hold, and telemetry cost is below the fixed
0.5 ms threshold.  The result identifies two distinct effects.  Requested
thread count deterministically changes the entire normal trajectory and its
retained seed.  Within either arm, however, the exact seed and adaptive backend
sequence are constant while recovery iterations remain highly dispersed.
Consequently neither retained-seed variation nor the adaptive wall-clock
tie-break explains the within-arm tail.  The remaining changing state is inside
the two-thread PARDISO factor/solve execution and its floating reduction or
scheduling order.

An earlier telemetry batch in `reports/r3_joint_model_20260913` ran while the
machine was globally degraded: even fixed `maros-r7` rose to 2518.87 ms at two
threads versus the 1383.09 ms release ceiling.  Its release gate failed six
latency checks while all accuracy and iteration contracts passed.  Repeating
after the machine recovered restored `maros-r7` to 1092.65 ms and passed every
release comparison.  In mismatch order this is a machine/cost-model event, not
evidence for changing an algorithm or threshold; the degraded batch is retained
but excluded from the candidate prediction.

A separate ten-process, global-one-thread diagnostic probe is retained under
`reports/r3_recovery_1t_probe_20260913`.  Its normal source is a third,
thread-specific state (iteration 199, hash `5f05c808237caa08`), so it is not a
direct A/B for the two/four-thread seeds.  It nevertheless bounds serial
recovery cost: all 10 runs used backend sequence `UL`, converged accurately in
exactly 44 recovery iterations, and had recovery median/P95 732.85/801.07 ms.

### Proposed R3-C2 candidate: formulation-scoped deterministic resources

**Model/algorithm and claim.**  Limit an actually selected normal-equations
variant to `min(requested_threads, 2)` for its full lifetime, and limit an
augmented recovery entered from a normal failure to one thread.  An augmented
primary solve retains the caller budget.  This formulation-scoped policy is
generic: it observes the selected mathematical formulation and transition
reason, never a case name or matrix dimension.  It makes the upstream retained
seed independent of a requested budget above two and removes threaded PARDISO
reductions from the sensitive recovery.

**Cost model and quantitative prediction.**  Use

`T_C2(p) = T_N(min(p,2), x_seed(2)) + T_A(1, x_seed(2)) + epsilon`.

The final 20-block two-thread normal median is 1920.93 ms; the one-thread probe
bounds a 44-iteration serial recovery at 732.85 ms median and 801.07 ms P95.
Allowing for the untested seed interaction gives a `greenbea` median prediction
of 2650--2800 ms in both requested arms.  Against the current four-thread
2911.00 ms this is a 4--9% reduction; the two-thread median may regress 4--10%.
The primary objective is distribution stability: predict `greenbea` P95 at or
below 3200 ms, a 29--31% reduction from 4509.72/4657.87 ms.  `dfl001` is
predicted unchanged.  `maros-r7` four-thread median is predicted to regress
about 14% from 957.45 to the two-thread 1092.65 ms because it selects normal
equations.  The four-thread aggregate median is predicted at 6100--6300 ms,
roughly -2% to +1% versus 6231.54 ms, with a materially smaller tail.

**Assumptions and references.**  Serial PARDISO removes parallel reduction
order as demonstrated by the 10/10 fixed recovery count, but the extrapolation
from the one-thread seed to `x_seed(2)` remains the explicit risk.  Actual
formulation is known before iterative factorization begins.  The resource scope
uses oneMKL's thread-local control; restoration state follows Wächter--Biegler
(2006), Sections 3.1 and 3.3.  The separation of seed identity, backend sequence,
and internal factorization state follows the measured joint model above.

**Fixed validation for a future implementation.**  First use a diagnostic-only
override to run at least three independent recoveries for each retained seed
hash and require a fixed recovery iteration count before admitting the policy.
Then run the standard 20-block protocol.  Require 120/120 accuracy, exact 44/21
`dfl001`/`maros-r7` iteration sets, `greenbea` median 2650--2800 ms in both arms,
P95 no greater than 3200 ms, four-thread aggregate median 6100--6300 ms, and the
unchanged schema-v1 release gate.  Run full Release CTest and dedicated MILP
gates.  Wrong sign or greater-than-50% prediction error triggers the mandated
ordered mismatch review.  R3-C2 exists only as a design candidate and lacks
release approval;
the existing bounded-recovery cap likewise remains explicitly unapproved for
release.

## R3 release isolation and schema-v2 RATIONALE (fixed before code)

Model/algorithm: the LP execution state is (formulation, retained primal bits,
rebuilt dual/barrier state, factor internal state, resource scope). A diagnostic
is a read-only projection of that state. Equal FNV fingerprints and equal
backend labels do not prove equality of all factor state or reduction schedules.
The joint evidence rules out observed seed/backend variation within an arm;
PARDISO reduction/scheduling is a remaining hypothesis, not a proved cause.

Implementation-fidelity audit: the unaccepted cap currently changes the default
resource scope, and the parser incorrectly requires every recovery to use at
most two threads. Backend labels silently truncate after sixteen selections.
Timing validation can admit missing/null metrics and non-finite intervals.
These are release/measurement defects, not reasons to tune the numerical method.
Machine/cost audit: T = T_N(p,s_N) + T_A(q,s_seed,s_factor) + diagnostic overhead;
conditional recovery iteration variance prevents a fixed-work Amdahl prediction.
Assumptions: the retained primal is not the complete backend state; serial
recovery from a different seed proves nothing about the retained 2T/4T seeds.
Theory: retain the joint model with an explicit latent backend state; C2's
predicted 14% maros-r7 regression is not excused by aggregate savings.

Claim: an explicit experimental option, false by default, is the only path to
the two-worker cap. Diagnostics off performs zero diagnostic seed reads and
zero numerical decisions based on telemetry. No C2 policy is implemented.
Cost model: disabled diagnostics are O(1), enabled seed metrics/hash are O(n)
streaming reads; trace output is O(number of committed pivots), separate from
latency runs. Gate validation is O(raw records + trace bytes), offline.
Prediction: zero cap activations on the default path; zero diagnostic seed
traversals when disabled; zero changed native dual pivot records across the
instrumented base HEAD and candidate. No speedup is predicted for removing an
unapproved policy. Enabled transition preparation retains the pre-existing
median <0.5 ms budget. Gate fixtures: valid complete evidence passes; each
missing/malformed/failed contract fails. Existing joint evidence cannot pass R3.
References: this record's R3 joint model and original bounded-recovery fixed
protocol; oneMKL Developer Reference, mkl_set_num_threads_local; IEEE 754 binary64
bit serialization; native_dual solver/primal committed basis transactions.

Fixed validation: MSVC 19.44 Release /O2 /Ob2 /Oi /fp:fast, oneMKL INTEL,
IPO/PGO/native-arch off. Full serial Release CTest and separate executions of
test_dual_simplex, test_engine_api, test_milp_solver, native_kernel_comparison;
Python compile, gate rejection tests, doc anchors, whitespace and added-marker
scans. Preserve commit, source diff, build flags, commands and binary SHA256.
Build baseline 1e61b2434b07f2bb5a71839048e71fea97ccf586 in an isolated checkout
with exactly the same observational pivot probe as the candidate; label this
instrumented baseline explicitly. Compare actual per-pivot bytes (phase,
entering/leaving column, row, pivot and step binary64 bits) from independent
processes; empty traces are not evidence. Never infer this from source scope.

The schema-v2 contract retains the historical performance reference
78939619f31ebe19b9399c8f52d5c35f6d6c06ba and its archived 20-block report.
The supplied HEAD is the code baseline for trace equivalence, not a replacement
performance reference. Require exactly 20 alternating 2T/4T process blocks,
120/120 original-model accurate runs, per-run 44/21 iterations, complete
versioned recovery/factor records with one-to-one joins and time closure.
Require greenbea median(4T-2T paired runtime) < 0 AND median(4T) < median(2T),
4T median in the original [3000,3300] ms interval, and 4T P95 <= 1.10 * 2T P95.
Each arm/case median and P95 <= 1.10 * historical reference; aggregate median
<= 1.05 and P95 <= 1.10. Recovery runtime/iteration P95 <= 1.10 * archived
joint-model reference; missing state/schema/trace evidence fails closed.
All thresholds are fixed here and may not be relaxed by a result file.

Any behavior candidate additionally requires paired baseline/candidate broad
corpus runs at both budgets, 20 independent blocks each, with the same per-case
and aggregate ceilings. Fixed corpus: adlittle, afiro, agg, bandm, beaconfd,
blend, sc50a, sc50b, sc105, sc205, dfl001, greenbea, maros-r7. Hash every MPS.
The trace corpus uses these same cases, independent native-dual processes with
no wall-time cutoff, at 2T and 4T; every case must succeed with nonempty actual
pivot output and identical bytes. This finite corpus is evidence with stated
scope, not proof for every LP. C2 remains unimplemented; any future admission
requires at least three independent serial recoveries from each identical
retained seed before the 20-block experiment, plus a new fixed rationale.

Correction of historical interpretation: 2834.69 ms did not meet the explicit
[3000,3300] ms contract. The separate midpoint relative-error calculation cannot
waive that interval. Every earlier schema-v1 pass means only the legacy 2T
regression check, never R3 approval. A wrong sign or >50% prediction mismatch
stops tuning and is recorded here in fidelity/cost/assumption/theory order.

Reproducibility transcription before validation: the ignored historical summary
files are copied byte-for-byte to benchmark/r3_reference, retaining their exact
SHA256 values in the v2 contract. This relocates evidence, not a baseline change.
The trace probe also records committed primal bound flips as row=-1, with zero
pivot/dual-step fields because those operations have no algebraic pivot.
Raw line terminators participate in the cross-binary byte comparison.

### Default-path isolation measurement and ordered stop review

The first post-isolation fixed 20-block run is
reports/r3_release_isolation_20260913/stability, candidate benchmark SHA256
4b520f6840f0689e64d11768c1cbc269a53cfa718a22a25bbd5a35b8d7c94f04, working tree
over 1e61b2434b07f2bb5a71839048e71fea97ccf586. Build profile is unchanged.
120/120 original-model runs are accurate, and all 44/21 iteration checks hold.

| case | measured 2T median/P95 ms | measured 4T median/P95 ms |
|---|---:|---:|
| dfl001 | 3389.96 / 3515.45 | 2305.12 / 2411.01 |
| greenbea | 2586.80 / 4426.60 | 3096.12 / 4545.22 |
| maros-r7 | 1103.98 / 1281.36 | 950.84 / 1077.48 |

The greenbea paired median delta is +556.41 ms and the marginal median is
19.69% slower at 4T. The original absolute interval passes; the independently
fixed direction contract fails. Aggregate recommendation (8.48% improvement)
cannot waive this failure. No speedup was predicted for isolating an unapproved
policy, and no R3 improvement is claimed from these data.

Ordered review before any further implementation: (1) fidelity: recovery
telemetry now reports requested/effective 4/4, confirming the default cap is
absent; the initial state model and fixed iteration contracts hold. (2) machine
model: recovered work varies substantially, with median recovery iterations
47 at 2T and 91 at 4T; fixed-work thread scaling still cannot model greenbea.
(3) assumptions: resource inheritance restores the baseline policy but does not
make recovery deterministic. (4) theory: isolation is a release-safety property,
not a solution to the joint-state performance problem. R3 remains blocked and
C2 remains unimplemented. The already-fixed broad-corpus validation continues
to measure observational isolation; no tuning or threshold change follows.

Actual instrumented baseline/candidate pivot evidence is complete before this
review: all 26 corpus/thread pairs match byte-for-byte, with 83,398 records per
binary and 52/52 accurate solves. Baseline benchmark SHA256 is
73dcc86e8d49b75551d4a6b189526e823ba30a777951ebe5fb52cfdfa4f6c874. The only source
changes to the isolated baseline are the identical committed-transaction probe.
Raw traces and comparisons are under reports/r3_release_isolation_20260913.

### Broad-corpus accuracy stop (before further code)

The predeclared broad run stopped at broad-01-t4-candidate after three accurate
13-case processes. greenbea returned Time limit at 15010.5702 ms / 2000 reported
iterations, with normalized original primal violation 1.2712008694366573e-7,
above the unchanged 1e-7 contract. Objective error was 1.6177526746739278e-12,
dual residual 6.984919309616089e-12, and relative gap 2.9420219506217114e-13.
The comparator's accurate flag is false. No complete 20-block broad validation
exists; 51/52 accurate rows in the attempted four processes cannot be promoted
to an all-corpus pass. The original failed batch remains authoritative.

Ordered investigation: (1) implementation fidelity: raw JSON and process logs
confirm the fixed 13-case command, 4T, diagnostics off, 15-second budget, and
candidate SHA. Default cap isolation has already been directly observed in the
separate enabled-diagnostic run. This is an actual accuracy failure, not merely
a reporting/gate error. Source review finds no new default arithmetic/threshold
change, but source scope is not a proof of numerical execution equivalence.
(2) machine/cost model: the candidate exhausted the process solve budget while
primal violation remained near the tolerance; time and work cannot be separated
by the previously assumed fixed recovery count. (3) assumptions: success on
three target cases does not establish robustness after earlier corpus solves;
allocator/backend/process state may matter in addition to the retained seed.
(4) theory: the latent factor-state model remains incomplete. The candidate
cannot receive general-solver release approval on this evidence.

Fixed diagnostic follow-up before execution: exactly three paired independent
baseline/candidate processes on the same 13-case corpus, requested 4T,
diagnostics off, unchanged 15-second limit/tolerances, alternating A/B order.
Retain every result, including failures. Purpose is implementation-fidelity
classification, not another acceptance sample. Do not resume or replace the
failed broad batch even if every diagnostic replay succeeds. Do not implement
C2, change resources, or tune numerical parameters in response.

The fixed diagnostic replay completed exactly three pairs: baseline 39/39 and
candidate 39/39 accurate. greenbea baseline times/returned iterations were
2981.44/47, 2869.68/43, 2866.04/43; candidate values were 3995.70/142,
5020.67/355, 3525.72/118 (ms/iterations). These six processes are diagnostic,
not a replacement 20-block acceptance sample. They do not reproduce the timeout
but also do not demonstrate equivalent recovery work. The root cause remains
unresolved; neither a pure machine-noise explanation nor a telemetry-induced
regression is established. No release approval follows from 78/78 replay rows.

Further fidelity readback: the existing netlib runner transcribes
opt.max_iter=min(cfg.max_iterations,2000) for Native-IPM. Thus the recorded CLI
budget 100000 has an effective per-variant IPM ceiling of 2000, unchanged in
both binaries. Returned iterations are not a sum over all fallback variants.
The reported 2000 must not be described as 2000 total solve iterations. This
is a retained benchmark interpretation constraint; it is not relaxed here.

### Final validation and release decision

The completed final single-writer validation ran full Release CTest serially:
21/21 registered tests passed in 3.78 s. Independent test_dual_simplex,
test_engine_api, test_milp_solver and native_kernel_comparison exited zero.
Catch suites reported respectively 23141/80, 339/57 and 2049/55
assertions/cases. native_kernel_comparison contains failed extreme-scaling
probe rows despite its zero exit status; that exit is not a claim that every
printed diagnostic problem was accurate. Gate adversarial tests passed 19/19;
Python compilation, document anchors, whitespace and added-marker checks passed.
The engine manual compiled twice with XeLaTeX (108 pages, no unresolved source
references); the existing font-shape substitution warning remains in its log.

Schema-v2 verifies the actual isolated baseline HEAD and permits exactly the
common observer transcription, whose header SHA256 is fixed in the contract.
This is provenance validation only: actual cross-binary pivot bytes are still
mandatory and are verified separately. Final check command:

```powershell
python tools/r3_collect_evidence.py --stage checks --baseline-binary C:/Users/matri/Codes/MIPSolvers-r3-baseline/tests/Release/netlib_solver_benchmark.exe --candidate-binary tests/Release/netlib_solver_benchmark.exe --baseline-build C:/Users/matri/Codes/MIPSolvers-r3-baseline/build/r3 --candidate-build build/windows-msvc-cplex --output reports/r3_release_isolation_20260913
python benchmark/check_lp_release_gate.py --result reports/r3_release_isolation_20260913/evidence.json --baseline benchmark/windows_lp_r3_contract.json
```

Measured versus fixed prediction/contract:

| observation | fixed prediction or contract | measured | disposition |
|---|---|---|---|
| default recovery scope | zero cap activations | requested/effective 2/2 and 4/4 in all 40 transition records | pass |
| enabled telemetry preparation median | <0.5 ms | 0.1287 / 0.14215 ms (2T/4T) | pass |
| native pivot record changes | zero | 0 of 83398 per binary; 26 actual pairs | pass |
| target original-model accuracy | 120/120 | 120/120 | pass |
| dfl001 / maros-r7 iterations | 44 / 21 every run | 44 / 21 every run | pass |
| greenbea 4T median interval | [3000,3300] ms | 3096.12 ms | pass |
| greenbea paired median delta | <0 ms | +556.41 ms | fail |
| full broad-corpus accuracy | all 1040 rows, 20 paired blocks per budget | stopped at 51/52 rows in first block | fail/incomplete |

The v2 gate returns nonzero with r3_release_approved=false, identifying the
wrong median direction and the broad-corpus inaccurate original-model result.
No baseline or threshold was replaced, no failing batch discarded, no C2
implementation introduced, and no release approval issued. Observations and
experimental policy are now separable in code, but this candidate has not met
the general-solver release contract. The unresolved recovery-state/trajectory
failure requires a further derivation before any numerical intervention.

### Commit-preparation scan fidelity correction

The new research-note file exposes a CI scanner fidelity defect: it scans diff
file headers as though they were added source content. Exclude only the
three-plus file-header prefix, retaining all actual added lines. This matches
the existing local collector's content-only scan. Prediction: a header-only
fixture produces zero findings, while an added forbidden marker is still
rejected. Validate both fixtures and the complete staged diff before commit.
This changes no solver code, numerical behavior, or release threshold.

Pre-push reproducibility check found Git would normalize the archived reference
JSON from CRLF to LF, invalidating the fixed byte hashes after checkout. Preserve
those JSON bytes and force LF for the hashed observer header with narrow Git
attributes. Prediction: staged reference objects and observer header have the
same SHA256 as the fixed contract; verify the index objects before committing.

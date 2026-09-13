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

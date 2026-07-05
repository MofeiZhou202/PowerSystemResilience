> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: analysis or planning note; verify decisions against current source before execution.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Parallel Computation Analysis and Implementation Plan

Scope: heavy-computation workflows in the current hybrid AC/DC simulation
package, especially `时序生产模拟` and `可靠性分析` with NMC, MC, FMEA, and
failure-mode FMEA. This document is theory-first: the mathematical independence
conditions are stated before any implementation plan, because parallelism is
correct only when it preserves the study model.

Implementation anchors inspected:

- `include/hacdcpf/util/thread_pool.hpp`
- `include/hacdcpf/time_series/time_series_pf.hpp`
- `src/time_series/time_series_pf.cpp`
- `include/hacdcpf/time_series/annual_production_sim.hpp`
- `src/time_series/annual_production_sim.cpp`
- `include/hacdcpf/reliability/reliability_assessment.hpp`
- `src/reliability/reliability_assessment.cpp`
- `include/hacdcpf/reliability/failure_mode.hpp`
- `src/reliability/failure_mode.cpp`
- `tests/run_gui_server.cpp`
- `web/js/app.js`

The analysis keeps solver internals out of scope. The intended changes should be
localized to business/API/UI orchestration and diagnostics, consistent with the
project constraints in `AGENTS.md`.

---

## 1. Mathematical Structure of the Workloads

### 1.1 A General Parallel Decomposition

Most package workloads have the form

$$
R = \mathcal A\left(\{y_\omega\}_{\omega\in\Omega}\right),\qquad
y_\omega = \mathcal S(x_\omega; \theta),
$$

where:

- $\Omega$ is the set of independent work items: days, hours, Monte Carlo
  samples, outage states, contingencies, or failure modes.
- $x_\omega$ is the data for one work item.
- $\theta$ is read-only shared case data and solver options.
- $\mathcal S$ is the expensive simulation/evaluation kernel.
- $\mathcal A$ is a deterministic aggregation: concatenation, summation,
  sorting, metric reduction, or convergence tracking.

Parallel execution is correct if every worker can compute
$y_\omega$ without mutating shared state used by another worker:

$$
y_\omega \perp y_{\omega'} \mid (x_\omega, x_{\omega'}, \theta),
\qquad \omega \neq \omega'.
$$

In code terms, each task should receive either a read-only reference to the base
case or an intentional copy of the mutable case. Results should be written into
disjoint slots and reduced after the parallel region. The current package mostly
follows this pattern: `evaluate_state` receives the system by value, and the
time-series daily paths write to day-indexed vectors.

### 1.2 Speedup Bound and What "100% CPU" Means

Let $p$ be the fraction of runtime that can be parallelized and $N$ the worker
count. Amdahl's law gives

$$
S_N = \frac{1}{(1-p) + p/N}.
$$

For heavy simulations, CPU utilization will be high only when:

1. $p$ is high, meaning the expensive part is outside single serial solver calls.
2. $|\Omega| \gg N$, meaning there are enough independent tasks.
3. Task cost is reasonably balanced, or the scheduler can dynamically rebalance.
4. The called solver libraries are thread-safe, or each call is isolated.
5. Nested parallelism is controlled so that outer workers do not each spawn many
   inner solver threads and oversubscribe the machine.

Low CPU on Windows therefore does not necessarily mean C++ threads are broken.
It often means the current run fell into a mathematically or solver-safety
guarded serial path, produced only one independent task, or spends most time in
one single-threaded solver call.

### 1.3 Windows-Specific Execution Model

The current package uses a C++ `std::thread`-based `util::ThreadPool`, not Python
`multiprocessing`. Therefore the usual Windows `spawn` import-guard problem is
not the direct cause.

The Windows-relevant risks are instead:

- `std::thread::hardware_concurrency()` may return a conservative value or zero
  on some environments; the package falls back to one worker when it sees zero.
- Third-party solver adapters may rely on process-global scheduler state or
  non-reentrant native libraries; the code already guards some backends.
- GUI/API request fields can fail to request parallelism, request only one
  thread, or run a one-day/one-contingency study.
- Antivirus, debug builds, console logging, and memory allocation can amplify
  serial overhead on Windows.

The package should therefore expose not only "parallel requested", but also
"parallel effective", "worker count", "work item count", "serial guard reason",
and "dominant serial fraction".

---

## 2. Time-Series and Annual Production Simulation Theory

### 2.1 Fully Coupled Multi-Period Model

For a coupled production simulation over
$\mathcal T=\{0,\ldots,T-1\}$, a simplified UC/ED formulation is:

$$
\min \sum_{t\in\mathcal T} C_t(p_t,u_t,r_t)
$$

subject to:

$$
g_t(p_t,u_t,e_t,\theta_t,v_t)=0,
$$

$$
h_t(p_t,u_t,e_t,\theta_t,v_t)\le 0,
$$

and inter-temporal constraints:

$$
e_{s,t+1}=f_s(e_{s,t},p^{ch}_{s,t},p^{dis}_{s,t}),
$$

$$
u_{g,t}-u_{g,t-1}=z^+_{g,t}-z^-_{g,t},
$$

$$
p_{g,t}-p_{g,t-1}\le R_g^\uparrow\Delta t,\qquad
p_{g,t-1}-p_{g,t}\le R_g^\downarrow\Delta t.
$$

If the model includes constraints linking day $d$ to day $d+1$, then solving
days independently is not equivalent to the fully coupled model.

### 2.2 Exact Daily Independence Under Boundary Conditions

Let the time horizon be partitioned into days
$\mathcal T=\cup_{d=1}^D \mathcal T_d$. The model decomposes into independent
daily subproblems if all cross-day constraints are removed or replaced by fixed
day-local boundary conditions:

$$
\min \sum_{d=1}^D \sum_{t\in\mathcal T_d} C_t(\cdot)
= \sum_{d=1}^D \min_{\mathcal X_d}\sum_{t\in\mathcal T_d} C_t(\cdot),
$$

provided

$$
\mathcal X = \mathcal X_1 \times \cdots \times \mathcal X_D.
$$

For storage, the current daily decomposition imposes an energy-neutral boundary:

$$
e_{s,t_d^{end}} = e_{s,t_d^{start}},
$$

so no storage energy is borrowed from adjacent days. This is a valid daily
study model and is the main mathematical basis for parallel day solves.

The consequence is important: parallel daily simulation is exact for the
energy-neutral daily model, but it is not automatically equivalent to a fully
coupled annual UC with inter-day SOC, ramping, or commitment memory. The GUI and
API should label it as a deliberate model choice, not just a faster version of
the hierarchical/coupled run.

### 2.3 Current Time-Series Parallel Path

The plain time-series solver supports a per-day split in
`solve_time_series_pf`:

- It runs only when `opts.parallel_daily == true` and the horizon spans more than
  one day.
- It sets `day_opts.parallel_daily = false` so recursion stops.
- It sets `day_opts.enforce_terminal_soc_cyclic = true`.
- It resolves the UC backend through `resolve_parallel_daily_uc_solver`.
- It forces the AC OPF backend to `ParityIPM`.
- It guards unsafe MILP backends to serial execution.
- It returns `parallel_daily_effective`, `parallel_workers`, and
  `parallel_mode`.

The relevant current code is around
`src/time_series/time_series_pf.cpp:3273` to `3330`.

### 2.4 Current Annual Production Parallel Path

The annual production simulation has an optional `solve_parallel_daily` path:

- The year is partitioned into independent calendar-day windows.
- `DailySimMode::SCUC` runs daily UC plus OPF/PF replay.
- `DailySimMode::DynamicSCED` fixes commitment, optionally from a
  representative peak day.
- `DailySimMode::DynamicOPF` skips UC and performs per-step OPF/PF.
- OPF is forced to `ParityIPM` in parallel mode.
- HiGHS/Gurobi explicit selections are guarded to serial daily execution.
- Native B&C is capped to one worker per daily UC solve.
- Results are written to day-indexed containers and stitched after workers
  complete.

The relevant current code is around
`src/time_series/annual_production_sim.cpp:611` to `823`.

### 2.5 Why Parallel Time-Series May Not Trigger

For `时序生产模拟`, parallelism is not effective when any of these conditions
holds:

| Condition | Effect |
|---|---|
| `parallel_daily=false` | The coupled path runs. |
| `num_steps <= steps_per_day` | Only one independent day exists. |
| `parallel_threads=1` | Worker count resolves to one. |
| `hardware_concurrency()` returns 0 or 1 | Auto worker count becomes one. |
| UC is enabled and backend is explicit HiGHS/Gurobi | Code guards to serial for solver safety. |
| Most runtime is one daily UC solve | Outer day parallelism cannot fill all cores. |
| Too few days relative to cores | CPU cannot reach full utilization. |
| Debug build / heavy logging | Thread overhead and I/O can dominate. |

The present `parallel_mode` strings already reveal some of this:
`parallel-daily/native-bc`, `parallel-daily/scip`, `parallel-daily/no-milp`, or
`parallel-daily/serial-guarded`.

---

## 3. Reliability Analysis Theory

### 3.1 Non-Sequential Monte Carlo (NMC / NSQ MC)

For each component $i$, define unavailability $U_i$. A non-sequential sample is
a random outage state

$$
X_i^{(k)} \sim \mathrm{Bernoulli}(U_i),
\qquad X^{(k)}=(X_1^{(k)},\ldots,X_n^{(k)}).
$$

The consequence evaluator returns load shedding

$$
S^{(k)} = \mathcal S(X^{(k)}, L),
$$

where $L$ is the study load level or load scaling. The expected demand not
served is

$$
\mathrm{EDNS} \approx \frac{1}{K}\sum_{k=1}^K S^{(k)},
$$

and annual expected energy not served is

$$
\mathrm{EENS} \approx H \cdot \mathrm{EDNS}.
$$

The samples are independent after their random states have been generated, so
state consequence evaluations can be parallelized:

$$
\mathcal S(X^{(k)}) \perp \mathcal S(X^{(j)}) \quad (k\neq j).
$$

The current implementation parallelizes only cache-miss states in batches,
preserving ordered sampling and deterministic reduction. This is a good
correctness pattern. However, if most samples are N-0 states or cache hits, the
parallel work set can be small and CPU utilization can remain low even when
`parallel_effective=true`.

Current anchor: `src/reliability/reliability_assessment.cpp:1238` to `1317`.

### 3.2 Sequential Monte Carlo (MC)

Sequential MC samples chronological component up/down trajectories. For a
component with mean time to failure $\mathrm{MTTF}_i$ and repair time
$\mathrm{MTTR}_i$, the continuous durations are sampled as:

$$
T_i^{up} = -\mathrm{MTTF}_i \ln U,\qquad
T_i^{down} = -\mathrm{MTTR}_i \ln V,
$$

with $U,V\sim\mathrm{Uniform}(0,1)$. The implementation maps those durations to
hourly states.

Within one sampled year, the hourly state evaluations are conditionally
independent after the trajectory matrix is created:

$$
y_h = \mathcal S(X_h, L_h),\qquad h=1,\ldots,H.
$$

Therefore hourly consequence evaluations can be parallelized, while the
chronological trajectory generation and annual reduction remain serial. The
current implementation follows this form and dispatches `down_hours`.

Current anchor: `src/reliability/reliability_assessment.cpp:1532` to `1657`.

Important utilization nuance: if the system has low unavailability, most hours
are N-0 and are served from an N-0 cache. The actual parallel set is only
`down_hours`, so a very reliable case can report parallel mode but still use
little CPU.

### 3.3 Deterministic FMEA / N-1 Analysis

For first-order FMEA, each contingency $c$ has annual failure frequency
$\lambda_c$, stage duration $\tau_{c,s}$, and stage shedding $S_{c,s}$:

$$
\mathrm{EENS} =
\sum_{c\in\mathcal C}\lambda_c
\sum_{s\in\{\mathrm{sw},\mathrm{rep}\}}
\tau_{c,s}S_{c,s}.
$$

Each contingency consequence is independent:

$$
y_c = \mathcal S(c),
\qquad c\in\mathcal C.
$$

This is the cleanest package workload for outer parallelism because no
statistical convergence loop is needed. The current `run_distribution_fmea`
parallelizes over the contingency catalog.

Current anchor: `src/reliability/reliability_assessment.cpp:4012` to `4038`.

### 3.4 Failure-Mode FMEA and Pairwise N-2

Failure-mode FMEA generalizes "one component fails" to "one failure mode fails".
For single modes:

$$
\mathrm{EENS}_1 = \sum_{m\in\mathcal M} \lambda_m \tau_m S_m.
$$

For pairwise independent overlaps, the implemented approximation is:

$$
U_{ij}=U_iU_j,
\qquad
\mathrm{EENS}_2 \approx
\sum_{i<j} U_iU_j H S_{ij}.
$$

Both single-mode evaluations and pair evaluations are independent and are
already parallelized by the current code.

Current anchors:

- Single modes: `src/reliability/failure_mode.cpp:1391` to `1410`
- Pairs: `src/reliability/failure_mode.cpp:1533` to `1549`

### 3.5 Why Reliability Parallelism May Not Trigger

For `可靠性分析`, parallelism is not effective when any of these conditions holds:

| Method | Non-trigger / low-utilization cause |
|---|---|
| NMC / NSQ MC | `max_iterations <= 1`; all samples N-0; cache-hit dominated batches; low `parallel_threads`; few cache misses per batch. |
| Sequential MC | `hours_per_year <= 1`; very few `down_hours`; N-0 cache dominates; few simulated years. |
| FMEA | contingency catalog has one item or is empty; repair search inside each contingency dominates and is serial. |
| Failure-mode FMEA | catalog has one mode; pair enumeration disabled or pruned; consequence model too cheap relative to thread overhead. |
| All methods | API did not set `parallel`; GUI field parsed as one; hardware concurrency resolves to one; solver/consequence evaluator is internally serialized. |

---

## 4. Current Parallel Infrastructure Assessment

### 4.1 Thread Pool

`util::ThreadPool` is a fixed-size C++ thread pool. It provides:

- constructor with `num_threads <= 0` mapped to `std::thread::hardware_concurrency()`;
- `submit` returning a `std::future`;
- `parallel_for(count, body, num_tasks)`;
- process-wide singleton `ThreadPool::global()`.

Current anchor: `include/hacdcpf/util/thread_pool.hpp:21` to `105`.

This is appropriate for Windows because it does not depend on fork semantics.
The missing production-grade features are not correctness blockers, but they are
important for diagnosis and heavy workloads:

- no explicit reporting of resolved hardware concurrency;
- no queue depth / completed task counters;
- no dynamic chunking for irregular FMEA or MC state costs;
- no central execution policy to avoid nested parallel oversubscription;
- no structured reason when worker count resolves to one.

### 4.2 Worker Count Resolution

Time-series and reliability use similar worker-count functions:

$$
N = \max(1,\min(N_{requested\ or\ hardware}, |\Omega|)).
$$

This is safe, but it hides why only one worker was chosen. For user-facing
Windows debugging, the result should expose:

- requested thread count;
- hardware concurrency;
- work item count;
- resolved worker count;
- disabled/guarded/insufficient-work reason.

### 4.3 Solver Safety Guards

The time-series path already encodes important solver-safety decisions:

- Native B&C may run in parallel daily mode, but each daily UC is capped to one
  backend thread.
- SCIP is allowed when available and supports MILP.
- HiGHS and Gurobi explicit selections are guarded to serial day execution
  because of process-global scheduling state.
- AC OPF is forced to `ParityIPM` so Ipopt/MUMPS is not invoked concurrently.

This is the right safety posture. The implementation plan should improve
diagnostics rather than remove these guards.

### 4.4 API and GUI Surface

The server already parses:

- `parallel_daily` and `parallel_threads` for time-series and annual production;
- `parallel` / `enable_parallel` and `parallel_threads` for reliability;
- method-specific reliability parallel fields in unified endpoints.

The response already includes basic fields:

- `parallel_daily_effective`, `parallel_workers`, `parallel_mode`;
- `parallel_effective`, `parallel_workers`, `parallel_mode`,
  `parallel_execution`.

The gap is that the response does not yet expose enough causal diagnostics to
explain Windows low CPU. Users can see "not effective" but not always why.

---

## 5. Heavy-Scenario Execution Design

### 5.1 Execution Policy Object

Introduce a shared execution policy in the business layer, not solver internals:

```cpp
struct ParallelExecutionPolicy {
  bool requested{false};
  int requested_threads{0};
  int hardware_threads{1};
  int work_items{0};
  int resolved_workers{1};
  std::string backend;
  std::string mode;
  std::string guard_reason;
  bool effective{false};
};
```

Use it in time-series and reliability result structs or embed it as a
JSON-only diagnostic first. The policy should be computed before a parallel
region and echoed after completion.

### 5.2 Parallelism Levels

The package should explicitly distinguish three levels:

| Level | Example | Safe default |
|---|---|---|
| Study-level | independent days, MC states, FMEA contingencies | Prefer for heavy scenarios. |
| Solver-level | MILP/OPF internal threads | Cap to one when outer parallelism is active unless backend is known safe. |
| Kernel-level | Jacobian/ac kernel parallel loops | Disable or cap during outer parallel regions to avoid oversubscription. |

For heavy GUI workflows, outer study-level parallelism should be the primary
strategy because it is naturally independent and easier to make deterministic.

### 5.3 Dynamic Scheduling for Irregular Tasks

Current `parallel_for` uses static contiguous chunks. This is fine when each
work item has similar cost, but reliability FMEA and MC cache-miss states can be
irregular. For example, one contingency may trigger a large repair search while
another immediately returns no shed.

A dynamic worker loop should be added for irregular workloads:

```cpp
std::atomic<size_t> next{0};
auto worker = [&] {
  while (true) {
    size_t i = next.fetch_add(1);
    if (i >= work_items) break;
    evaluate(i);
  }
};
```

This can be added as `ThreadPool::parallel_for_dynamic` or as a local utility in
the business layer. It should be used for FMEA, failure-mode FMEA pairs, and MC
cache-miss batches. Static chunking can remain for uniform kernels.

### 5.4 Batch Size for Monte Carlo

The current NSQ MC batch size is approximately `workers * 4`. For cheap states
or many N-0/cache-hit samples, this can starve workers. A better policy is:

$$
B = \max(4N,\min(B_{max}, cN)),
$$

where $c$ may be 16 or 32 for low-cost evaluations. The code should track:

- batch size;
- number of N-0 samples;
- number of cache hits;
- number of cache misses;
- number of actual parallel evaluations.

This explains whether low CPU is caused by the algorithm doing very little real
work in each batch.

### 5.5 Windows Diagnostic Contract

Every heavy endpoint should return:

```json
{
  "parallel_execution": {
    "requested": true,
    "effective": true,
    "requested_threads": 0,
    "hardware_threads": 16,
    "workers": 12,
    "work_items": 365,
    "mode": "parallel-daily/scip",
    "guard_reason": "",
    "actual_parallel_evaluations": 365,
    "serial_evaluations": 0
  }
}
```

For guarded serial:

```json
{
  "parallel_execution": {
    "requested": true,
    "effective": false,
    "workers": 1,
    "work_items": 365,
    "mode": "parallel-daily/serial-guarded",
    "guard_reason": "explicit HiGHS/Gurobi UC backend is not treated as safe for concurrent daily solves"
  }
}
```

The GUI should show this in the result summary so Windows users can distinguish
configuration issues from actual performance bugs.

---

## 6. Method-Specific Implementation Plan

### Phase 1: Diagnostics Without Behavioral Change

Add structured parallel diagnostics to all heavy endpoints.

Time-series and annual production:

- Report requested parallel flag.
- Report requested thread count.
- Report `std::thread::hardware_concurrency()`.
- Report number of days/work items.
- Report resolved workers.
- Report solver backend after auto-resolution.
- Report guard reason when serial.

Reliability:

- NMC: report `max_iterations`, batch size, cache misses evaluated in parallel,
  N-0 samples, cache hits.
- MC: report `hours_per_year`, `down_hours` per simulated year summary, actual
  hourly evaluations.
- FMEA: report catalog size and dynamic/static scheduling mode.
- Failure-mode FMEA: report single-mode catalog size, pair count, pruned pairs,
  workers.

No mathematical behavior changes are required in this phase.

### Phase 2: Shared Execution Policy

Create a small shared helper, for example under `include/hacdcpf/util/` or a
business-layer utility header:

- `detect_hardware_threads()`
- `resolve_worker_count(requested, work_items)`
- `make_parallel_policy(requested, requested_threads, work_items, backend)`
- `parallel_guard_reason(...)`

Keep the public API stable if desired by filling existing result fields and
adding optional diagnostics only in JSON.

### Phase 3: Dynamic Scheduling for Irregular Work

Add dynamic scheduling and apply it where task cost varies:

- `run_distribution_fmea`
- `run_failure_mode_fmea`
- failure-mode pair evaluation
- NSQ MC cache-miss evaluation
- SEQ MC `down_hours` evaluation if profiling shows imbalance

Expected benefit: better CPU utilization on cases where a few contingencies or
states dominate runtime.

### Phase 4: Windows Heavy-Scenario Presets

Add GUI/API presets that choose safe defaults:

| Preset | Time-series | Reliability |
|---|---|---|
| Conservative | outer workers = physical cores / 2; solver threads = 1 | same |
| Full CPU | outer workers = hardware threads; solver threads = 1 | same |
| Solver-heavy | outer workers small; allow backend internal threads when safe | only for solver-safe methods |

For `时序生产模拟`, the Windows-safe high-throughput default should be:

- `parallel_daily=true`;
- `parallel_threads=0` or a detected hardware value;
- `daily_mode=DynamicOPF` when UC is not required;
- `daily_mode=DynamicSCED` when commitment can be reused;
- `uc_solver=auto` so the resolver can avoid unsafe backends;
- keep `ParityIPM` for parallel OPF.

### Phase 5: Benchmark and Regression Harness

Add benchmark-oriented tests or tools rather than fragile unit tests that assert
wall time:

- CLI/API smoke case: annual run with 7, 30, and 365 days.
- Reliability NMC with synthetic high-failure data to force cache misses.
- Sequential MC with high-failure data to force many `down_hours`.
- FMEA with a large contingency catalog.
- Failure-mode FMEA with N-2 pairs enabled.

Record:

- wall time;
- worker count;
- work item count;
- actual parallel evaluations;
- speedup against single worker;
- `parallel_mode`;
- guard reason.

Assertions should check diagnostics and numerical consistency between serial and
parallel runs, not exact timing.

### Phase 6: Optional Process Isolation for Non-Reentrant Solvers

If future requirements demand parallel execution with solver backends currently
guarded as unsafe, use process-level isolation, not shared in-process threads.
On Windows this means explicit worker executables or job processes, not fork.

This is a larger architectural step and should remain optional because it
requires serialization of case data, request data, and result data.

---

## 7. Numerical Correctness Requirements

Parallel execution must satisfy:

### 7.1 Deterministic Aggregation

Results should be written to indexed slots:

$$
Y[i] = \mathcal S(X_i),
$$

then reduced in a fixed order:

$$
R = \sum_{i=1}^n w_iY[i].
$$

This avoids non-deterministic floating-point ordering where possible.

### 7.2 No Shared Mutable Case State

Each work item must receive:

- a `const HybridPowerSystem&` if it never mutates, or
- a local `HybridPowerSystem` copy if it applies outages, switching, or load
  scaling.

The current `evaluate_state(HybridPowerSystem sys, ...)` copy is correct for
MC/FMEA state evaluation.

### 7.3 Random Number Generation

For MC:

- Sampling should remain serial and ordered, or
- parallel RNG streams must be seeded by deterministic stream IDs.

The current NSQ path samples serially and parallelizes only evaluation, which is
the simplest way to preserve reproducibility.

### 7.4 Solver Thread Caps

When outer workers are active:

$$
N_{outer}\times N_{inner} \le N_{hardware}
$$

unless a backend is explicitly known to handle oversubscription well. The
current daily UC cap for Native B&C is aligned with this requirement.

---

## 8. Immediate Investigation Checklist for Low CPU on Windows

When a user reports that `时序生产模拟` runs with only one low-utilization thread,
check in this order:

1. Confirm the response fields:
   `parallel_daily_effective`, `parallel_workers`, `parallel_mode`.
2. Confirm the request sent `parallel_daily=true`.
3. Confirm the horizon contains more than one independent day:
   `num_steps > round(24 / step_duration_hr)`.
4. Confirm `parallel_threads` is not set to `1`.
5. Confirm `hardware_concurrency()` is not resolving to `0` or `1`.
6. Confirm the requested UC backend is not an explicit guarded backend.
7. If `parallel_mode=parallel-daily/...` but CPU is low, check whether most time
   is spent in one long day solve or in a serial pre/post stage.
8. If `DynamicSCED` is used, remember the representative SCUC day is solved
   before the parallel region.
9. If the case has only a few days, the maximum outer CPU fill is bounded by the
   number of days.
10. Run an annual 30-day or 365-day test with `DynamicOPF` or `Auto` solver to
    separate scheduler issues from solver-backend guards.

For reliability:

1. Check `parallel_effective`, `parallel_workers`, and `parallel_mode`.
2. For NMC, inspect cache-miss count rather than sample count.
3. For sequential MC, inspect `down_hours`.
4. For FMEA, inspect catalog size and whether a few contingencies dominate.
5. For failure-mode FMEA, inspect pair count and pruning thresholds.

---

## 9. Recommended Final Architecture

The target architecture should look like this:

```text
GUI/API request
  -> ParallelExecutionPolicy
       -> validates requested threads, work count, backend guard
       -> emits diagnostics
  -> Work item builder
       -> days / MC states / hours / contingencies / failure modes
  -> Safe outer parallel executor
       -> dynamic for irregular work
       -> static for uniform work
       -> solver inner threads capped
  -> Indexed result collection
  -> Deterministic reducer
  -> Result + parallel diagnostics
```

This architecture gives the package two things it currently needs for heavy
Windows studies:

1. Higher CPU utilization when there is real independent work to execute.
2. Clear explanations when parallelism is intentionally or mathematically not
   available.

---

## 10. Priority Summary

1. Add causal diagnostics first. This is the fastest way to explain the current
   Windows one-thread reports.
2. Preserve the existing solver-safety guards. Do not "fix" low CPU by running
   non-reentrant solver backends concurrently.
3. Use dynamic scheduling for irregular reliability workloads.
4. Make daily decomposition semantics explicit in the UI: it is an
   energy-neutral daily study model.
5. Add heavy benchmark harnesses that compare serial and parallel numerical
   results and record speedup diagnostics.


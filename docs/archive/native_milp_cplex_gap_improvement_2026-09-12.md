# Native MILP deadline, stability, and parallel evaluation

## Pre-registered rationale

Model and algorithm: Native solves a MILP by importing a HiGHS-owned root
relaxation and cut fixed point, then continuing with the project's
branch-and-cut tree. A valid global wall limit must be inherited by every
nested root LP, presolve, cut loop, heuristic, and tree LP. Otherwise an
unbounded nested phase can consume arbitrarily more time than the caller's
budget before the tree or its parallel explorers can run.

Observed implementation mismatch before modification: on `30n20b8`, a Native
run configured for three seconds entered the HiGHS `evaluateRootNode()` owner,
performed 18 live cut-pool rounds, and reported 45.764239 seconds in that root
phase before manual interruption. Inspection showed that the dedicated
`highs_root_owner` received gap and coordinate-pinning parameters but no
`time_limit`. This is implementation infidelity to the existing global
deadline model, not an algorithm-quality result.

Correction: immediately before `highs_root_owner->run()`, pass the positive
remaining Native budget after the existing post-root finalization reserve.
The root owner's internal presolve, LP and cut loop then share the same finite
budget. The mathematical relaxation, cut validity rules, tolerances, objective
sense, variable domains, and postsolve audit are unchanged.

Cost model: the added work is one steady-clock query and one HiGHS parameter
assignment, `O(1)`. On short-budget cases previously dominated by an unlimited
root cut loop, elapsed time should fall from `T_root_unbounded` to approximately
`T_limit + T_check + T_finalize`, where `T_check` is HiGHS' coarse internal
deadline polling latency. It does not predict better bounds per LP iteration;
it reallocates finite time from root cuts to a bounded root/tree schedule.

Quantitative predictions fixed before implementation:

- `30n20b8`, three-second limit, one thread, seed zero, at most 5000 nodes:
  total Native wall time is at most eight seconds and at least 80% below the
  observed 45.764-second root phase.
- Every returned incumbent continues to pass the unchanged `1e-5`
  original-model row/bound/integrality/objective audit, and no time-limited run
  is relabeled proven optimal.
- A 1-thread versus 4-thread comparison is measurement-only. No speedup is
  predicted because the root pipeline is serial and `parallel_delay_until_incumbent`
  may intentionally keep the tree serial. The benchmark must report this as a
  scheduling limitation rather than interpreting extra threads as effective
  parallel work.

Assumptions: HiGHS honors its time limit at internal polling points; the Native
clock starts before root preparation; the existing 10%/50-ms-to-1-second
finalization reserve remains sufficient; runs use Release, seed zero, gap
`1e-4`, and the same original MIPLIB models.

References:

- Achterberg, *Constraint Integer Programming* (2007), Sections 4.1-4.2 and
  10.1, for global branch-and-cut bounds, node processing, and valid stopping.
- HiGHS 1.15.1 `HighsMipSolverData::evaluateRootNode()` and time-limit status
  handling, as vendored in this repository.
- `docs/archive/native_milp_root_quality_restart_prerequisites_2026-08-13.md`
  for the HiGHS-root/native-tree ownership contract.
- `docs/archive/cplex_callable_library_integration_2026-09-12.md` for the
  paired CPLEX/HiGHS import, audit, and three-second comparison protocol.

## Fixed validation protocol

1. Add the root-owner deadline without changing algorithms or tolerances.
2. Add a benchmark-only `--native-threads N` control, defaulting to one, and
   record it in JSON so 1/4-thread runs are reproducible.
3. Rebuild `miplib2017_benchmark` and `test_milp_solver`; run the existing
   deadline, parallel determinism, queue, and MILP correctness tests.
4. Rerun `30n20b8` with the exact baseline flags and check the eight-second and
   80% gates.
5. Run paired 1-thread and 4-thread Native evaluations on cases that reach the
   tree within the budget. Compare final gap, feasible count, nodes, and wall
   time; do not compare backend node counts to CPLEX/HiGHS node counts.
6. If elapsed time misses either deadline prediction, investigate parameter
   propagation, HiGHS polling granularity, assumptions, then the model before
   changing more code. Preserve every numerical failure and write findings
   below.

## Measurements and mismatch record

### Deadline propagation measurement

Build: MSVC 19.44 Release, `MIPSOLVERS_USE_CPLEX=ON`, base commit
`7fb0f10fd7a9fbaba621ed263ea42df6a87d6fe8` plus the working-tree changes
described here. Command:

```powershell
tests\Release\miplib2017_benchmark.exe `
  --data-dir "$env:TEMP\mipsolvers-miplib-eval\sample" `
  --solu "$env:TEMP\mipsolvers-miplib-eval\miplib2017-v36.solu" `
  --solvers native-highs-lp --case 30n20b8 --seeds 0 --repeat 1 `
  --time-limit 3 --max-nodes 5000 --native-threads 1
```

The first post-change run measured `solve_ms=6263.9466`, 86.31% below the
45.764239-second pre-change root phase and below the fixed eight-second solve
gate. The process wall time was 15.326899 seconds because the Windows runner
loads the MPS before timing `solve_ms`; it is not the Native solve interval
used by the prediction. A diagnostic repeat measured `solve_ms=6361.4`.

The elapsed-time prediction passed, but the result was incorrectly classified
as `Root relaxation failed` with `timed_out=false`. The diagnostic run showed
that HiGHS presolve consumed 2.8739 seconds, leaving 0.083952 seconds for the
root LP. The direct LP returned `HighsModelStatus::kTimeLimit` after 169
iterations. `solve_lp_relaxation_with_vendored_highs` then returned `false`
without copying that status to `LPRelaxationResult`; the strict wrapper
replaced it with `VendoredHiGHS LP rejected`, causing the outer root classifier
to lose the time-limit certificate. This is implementation infidelity in
termination-state propagation, not a machine-cost or numerical-tolerance
mismatch.

Correction fixed before implementation: for a direct HiGHS LP time limit,
publish a failed `SolveResult` whose status begins with `Time limit`, retain no
uncertified partial primal or dual bound, and return the handled result to the
strict wrapper. This is `O(1)` and should change `solve_ms` by less than 1%,
preserve the eight-second gate, report `Time limit reached`, and leave
`optimal=false`, `proven=false`, and `has_solution=false` on this case.

### Full-sample deadline mismatch

The corrected `30n20b8` case met the fixed gate, but the 12-case Native rerun
showed that the same global-budget contract is still violated on larger root
presolve paths. With the identical Release/seed/gap/three-second command,
`solve_ms` was 10.1185 s on `dws008-01`, 11.8722 s on
`neos-2075418-temuka`, 17.6359 s on `neos-848589`, and 11.6020 s on
`supportcase40`. All four feasible incumbents in the full sample passed the
unchanged `1e-5` audit, so this is a performance/deadline mismatch rather than
a numerical correctness failure.

The fixed prediction therefore does not generalize from `30n20b8`: another
phase, most likely HiGHS presolve/model import before the nested root owner is
started, is consuming time without observing `bc_remaining_sec()`. Per the
mismatch protocol, no further performance change is made until that phase is
identified and its budget propagation is re-derived.

### Unified presolve deadline derivation (2026-09-13)

The solve clock now starts before strict HiGHS preflight and the remaining
budget `max(0, T - elapsed)` is passed to every nested presolve invocation.
Native `MILPPresolve` polls this deadline at each reduction boundary and every
256 probing candidates/rows. Each poll is O(1) (`steady_clock::now` plus one
comparison); with at most 500 probing candidates this adds fewer than three
clock reads per solve outside the existing row-level checks. A timeout never
publishes a partially reduced model: the caller retains the original model and
returns `Time limit reached`, preserving postsolve and integrality contracts.

Prediction: on models whose preflight/presolve previously consumed 10–18 s
under a 3 s limit, wall time should be bounded near 3 s plus the existing
finalization reserve (at most 1 s), with no change in feasible-solution audit
results. Validation is a Release MSVC build using the fixed 30n20b8, 12-case,
and 1/4-thread commands above; report measured-vs-predicted elapsed time and
rerun all correctness tests. If elapsed time exceeds the prediction by more
than 50%, first inspect deadline propagation and polling granularity before
changing algorithms.

The post-change 30n20b8 rerun measured `solve_ms=3290.4` (3.29 s), status
`Time limit reached`, zero explored nodes, and no incumbent. This is within
the predicted three-second budget plus the documented finalization/runner
overhead; the eight-second gate remains satisfied.

The budget/launch change was then measured with the fixed Release commands.
On `mas74`, one thread explored 224 nodes in 2707.0 ms with a 17.8% gap;
four threads launched the tree before the remaining root heuristics, explored
1395 nodes in 3076.0 ms, and reached a 17.0% gap. The incumbent audit passed
in both runs. On root-dominated `sct2`, one and four threads both explored
zero nodes (2743.4 ms versus 2788.7 ms, 31.7% gap), confirming that the launch
gate does not create speculative workers when no tree work is available.

The 12-case one-thread rerun measured (ms): `30n20b8 3870.7`, `cbs-cta
4080.6`, `dws008-01 9811.4`, `graph20-20-1rand 2736.9`, `mas74 2707.0`,
`neos-2075418-temuka 3790.1`, `neos-4387871-tavua 2747.6`, `neos-848589
11055.9`, `ns1952667 5527.9`, `rail01 3315.1`, `sct2 2772.0`, and
`supportcase40 8140.2`. The previous measurements were 6384, 5109, 10118,
2918, 2730, 11872, 3500, 17636, 9091, 6627, 3377, and 11602 ms respectively;
the large-model reductions are consistent with eliminating repeated strict
presolve, while the remaining >3 s cases are retained as follow-up mismatch
work rather than hidden by a hard timeout.

### Root budget, presolve ownership, and parallel launch derivation

The root phase uses a finite work allocation rather than fixed iteration
counts. After the first root relaxation, optional cut work may consume at most
35% of the usable remaining budget, where usable time excludes the existing
post-root finalization reserve. The effective number of rounds is
`min(configured_rounds, floor(cut_budget / 0.05))`, with the existing per-round
deadline checks acting as the runtime backstop;
before a first-round observation it is conservatively capped at three when
less than two seconds remain. This is the standard adaptive-work principle of
Achterberg (2007), Sections 4.1-4.2 and 8.1: spend root effort only while its
bound improvement can amortize delayed tree search. Prediction: three-second
runs retain at least 55% of post-root time for heuristics/tree, while long runs
retain the configured cut ceiling.

Strict HiGHS presolve has one owner per solve. The retained preflight result is
copied into the later side-state slot and supplies the working LP, affine map,
variable-bound source, and postsolve instance. No second call is permitted for
the same original model. This removes one O(nnz plus presolve reductions) pass;
on cache misses the strict-root presolve portion should fall by approximately
40-50%, with bit-identical reduced coordinates because the authoritative pass
is unchanged.

Parallel proof search starts after root cuts, once the root LP/cut pool and
domain state are immutable, whenever at least two explorer threads are
available, an audited incumbent exists, the certified root gap remains open,
and at least 25% of the original time limit (minimum 0.25 s) remains. Root
progressive-rounding/diving/LNS then overlap with tree proof. This preserves
the single-writer root contract while advancing the launch point. Prediction:
on `mas74`-like cases the 4-thread run gains at least 20% more explored nodes
within the same limit without worsening audited incumbent validity; 1-thread
pivot paths remain unchanged.

Validation is fixed before measurement: Release MSVC, seed zero, gap `1e-4`,
the existing 12-case three-second cohort, plus paired 1/4-thread `mas74,sct2`.
Record solve time, root phase, cuts, nodes, final gap, schedule reason, and
original-model audit. A wrong-sign or greater-than-50% prediction miss triggers
the repository mismatch protocol before further tuning.

### Parallel measurement

The benchmark-only `--native-threads` control was evaluated on `mas74,sct2`
with the same Release build, seed 0, gap `1e-4`, three-second limit and 5000
node cap. Results:

| Case | Threads | Solve ms | Nodes | Gap | Effective/explorers | Schedule |
|---|---:|---:|---:|---:|---:|---|
| `mas74` | 1 | 2720.6 | 175 | 18.562% | 1 / 0 | `single_thread` |
| `mas74` | 4 | 3047.1 | 783 | 17.952% | 4 / 4 | `late_tree` |
| `sct2` | 1 | 3045.4 | 0 | 31.709% | 1 / 0 | `single_thread` |
| `sct2` | 4 | 3022.2 | 0 | 31.709% | 4 / 0 | `enabled` |

`mas74` demonstrates real parallel tree work: 4.47x as many nodes and a
0.61-point gap improvement at essentially the same short budget, but not a
wall-time speedup. `sct2` remains root-serial, so extra threads have no
effect. All four incumbents passed the original-model `1e-5` audit. The JSON
artifacts are `reports/miplib_native_parallel_1t.json` and
`reports/miplib_native_parallel_4t.json`.

# Development Status

Updated: 2026-08-09

This is the living handoff for verified build state and active engineering work.
Update it in place; do not create dated copies. Source, registered tests, and
the current Git worktrees remain authoritative.

## Verified baseline

| Scope | Result |
|---|---|
| Required MIPSolvers source | Sibling `../MIPSolvers`, branch `dev`, expected commit `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0` |
| `full-dev` regression | 1430/1430 registered tests completed without failure; 3 condition-dependent tests skipped |
| `macos-release` regression | 1425/1425 registered tests completed without failure; 3 condition-dependent tests skipped |
| Graph ASan/UBSan subset | 28 cases, 113 assertions passed after the iterative Tarjan fix |
| Reliability ASan/UBSan | Complete three-stage suite 25 cases/1339 assertions; `case33mg_acdc` 477 assertions and five consecutive parallel repeats passed; `test_1_no_sop` 79 assertions |
| Market ASan/UBSan | Complete suite 22 cases/845 assertions; focused initial root-cut case 1/42 |
| Other sanitizer subsets | Thread pool 4 cases/6 assertions; `test_hacdcpf` 26/89; `test_acopf_dcopf_crossval` 10/84; `test_power_flow_math_audit` 40/204 |

The two full CTest results establish the normal build baseline. They do not
claim that every sanitizer entry point is green.

## Active module audit

The living [module code audit](module_code_audit.md) records the current
source-backed findings and audit depth. The documentation coverage pass marks
`graph/`, `scenario_generation/`, `carbon_analysis/`, `integrated_energy/`, and
`sppt/` as having no active dedicated contract. Focused macOS Release
executables for typhoon traffic,
campus IES, SPPT metamorphic relations, harmonics, graph topology, EV
Formulation D, and carbon case validation passed on 2026-08-09; these tests do
not exercise the open concurrency, invalid-input, or unavailable-result paths.
No full rebuild, full CTest, or complete sanitizer suite was run specifically
for this audit.

A focused current-source sanitizer rebuild exposed an additional ordered-call
failure: the hybrid StrictHiGHS resilience case passes alone, but fails with
`StrictHiGHS Other run=-1` after the Native external-MESS-availability case in
the same process. This is tracked as AUD-011. The experiment used the dirty
sibling MIPSolvers worktree with the non-Release dirty-check override; it does
not replace the reproducible Release baseline above.

## Closed investigation

The HySim worktree currently contains uncommitted dependency, documentation,
graph, reliability, and graph-test changes. Re-run `git status` before relying
on this list.

- `src/graph/topology_analysis.cpp` fixes an invalidated stack-frame reference
  in iterative Tarjan traversal and preserves the exact parent edge for
  parallel-edge correctness. Focused ASan/UBSan tests pass.
- `src/reliability/three_stage_reliability.cpp` serializes process-global HiGHS
  and native B&C solve state across contingency workers; the native fallback
  uses one internal thread. Solver-bearing reliability workers request a 4 MiB
  POSIX stack through the optional `ThreadPool` stack-size parameter.
- The sibling MIPSolvers worktree has two uncommitted HiGHS changes: hash
  non-finite cut bounds from their IEEE-754 representation, and allow an
  initial root user-cut pool before the first restart. Both changes were rebuilt;
  focused and complete market sanitizer regressions pass.

The apparent `HPresolve::changeImplColUpper` container corruption in
`case33mg_acdc` was a downstream symptom of worker stack overflow, not a
presolve data-structure defect. On Darwin, the default pthread stack is about
512 KiB, while sanitizer instrumentation expanded the largest observed solver
frame to about 340 KiB. The same case completed on the main thread, and the
parallel case became stable after provisioning a 4 MiB worker stack. Five
consecutive sanitizer repeats completed in 414.63 seconds. The resulting
case33mg metrics were SAIFI 0.7833, SAIDI 10.87, and EENS 588.5; the case33bw
cross-check produced SAIFI 0.7787, SAIDI 10.60, and EENS 570.3.

## Performance closure

The dirty MIPSolvers Release binary was compared with a clean export of the
pinned commit on the same arm64 host using an A/B/C sandwich: 24 NETLIB cases,
Native and HiGHS, 3 repeats, single-threaded HiGHS, and a 30 second solve limit.
Every run was accurate (72/72 for each solver in each leg).

| Leg | Native geometric mean |
|---|---:|
| Clean A | 3.761956 ms |
| Dirty B | 3.761257 ms |
| Clean C | 3.763764 ms |
| `sqrt(A * C)` control | 3.762860 ms |

Dirty versus sandwich control was -0.0426%, within the predeclared absolute
1% acceptance threshold. After timing fields were removed, the JSON run
records matched exactly across A/B/C, including status, objective, feasibility,
iterations, and dual-pivot counts. The raw reports are
`/private/tmp/hysim_native_dual_{control_a,experiment_b,control_c}_repeat3.{csv,json}`.

## Fast orientation

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
cmake --build --preset full-dev
ctest --preset full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

Use [README.md](../README.md) for the user-facing capability baseline,
[AGENTS.md](../AGENTS.md) for architecture and invariants, and
[docs/README.md](README.md) for topic documentation.

# Native MILP Solver vs HiGHS — Performance & Numerical-Stability Evaluation

**Date:** 2026-07-20 (round 2 update same day, see §5.6)
**Scope:** native numerical stack in `src/` (dual simplex + SparseBasis LU, IPM-LP, native B&C) vs embedded HiGHS, on synthetic SCUC MILPs (6-bus/4T, 39-bus/4T, 39-bus/24T, 118-bus/24T), 5 NETLIB LPs, and adversarially scaled LPs.
**Method:** all numbers come from the dedicated driver `benchmark/native_kernel_comparison.cpp` (binary `tests/native_kernel_comparison`), which isolates the *pure native* kernels — see §1 for why this matters. HiGHS runs single-threaded via the raw `Highs` API; native B&C uses default auto-parallel options.
**Companion runs:** `tests/milp_benchmark_runner --full` (production StrictHiGHS ablation), Catch2 suites `test_numerical_stability`, `test_netlib_regression`, `test_dual_simplex`, etc.

---

## 1. Why a new benchmark driver was needed

`benchmark/solver_comparison.cpp` and `benchmark/milp_benchmark_runner.cpp` both force
`BCOptions::lp_kernel_backend = LpKernelBackend::HiGHS`, i.e. the "native" rows exercise HiGHS LP numerics
inside the native B&C — they benchmark HiGHS against itself. `native_kernel_comparison` instead runs:

- **MILP level:** native B&C with `lp_kernel_backend = ExperimentalNative` (native dual simplex, or native IPM at the root) vs raw-API HiGHS MIP.
- **LP level:** native dual simplex and native IPM-LP vs HiGHS LP on NETLIB (published optima), the SCUC LP relaxations (large, highly degenerate), and diagonally scaled NETLIB problems (dynamic range 10^k, exact objective invariant) as a numerical-stability probe.
- Every run is cross-checked with an independent solution audit (max row / bound / integrality violation of the returned `x`).

Usage: `--full` (adds 118-bus), `--skip-milp`, `--skip-lp`, `--time-limit N`.

---

## 2. Executive summary

- **The native dual-simplex kernel is numerically strong at small/medium scale.** It matches HiGHS on all 5 NETLIB problems (~1e-11 relative error), survives 10^6 diagonal scaling *better than HiGHS* (HiGHS reports "Optimal" with 7–16% objective error at 10^6 scaling), and its MILP incumbents on ≤39-bus cases match HiGHS-proven optima with ~1e-13 constraint violation.
- **The pre-existing gap vs HiGHS is concentrated in two places:** the IPM-LP kernel (fails on degenerate/large/scaled LPs across the board) and Phase-I robustness of the simplex on large degenerate root LPs (the 118-bus cliff).
- **The escalation chain (round 1) fixed the unpresolved 118-bus LP relaxation**: previously unsolvable, now Optimal in ~40 s with ~1e-12 residuals (single-threaded HiGHS did not converge within 300 s on the same LP).
- **Round 2 (§5.6) closed the B&C failure path on 118-bus:** the presolved root LP is still natively unsolvable, but the new unpresolved-root fallback (plus per-level escalation budgets, a root-budget reserve, and a cold-start iteration cap) turns the outcome from "garbage incumbent 2.1e11 + `-1e30` bound + 2× time-limit overrun" into an honest solve trajectory: root solved on the retry, tree started, real dual bound 135.7k, clean "Time limit" status at ~1.5× the limit.
- **The false-optimal audit is now strictly more honest than HiGHS on the adversarial scaling probe:** at 10^6 diagonal scaling HiGHS reports "Optimal" with 7%–7e17 objective error on **all 5** NETLIB problems; natDualSimplex solves 4 of them to ~1e-11 and *honestly rejects* the 5th (stocfor1) via the residual audit — after round-2.5 fixed three audit bypasses the new `--check` gate caught (last-level skip, 1e20 sentinel poisoning, NaN objective).
- **The HiGHS-style Phase I is not a win (A/B verdict): keep the default at 0.**
- **The production hybrid (native orchestration + vendored HiGHS LP kernel + native dynamic cuts) is genuinely competitive**: on 118-bus @120 s it reaches obj 152,522 (gap 5.9%) vs raw HiGHS 173,204 (gap 19.1%). The B&C machinery has real value; the bottleneck is the native numerical kernels underneath it.

---

## 3. LP-kernel level results

### 3.1 NETLIB (published optima)

| Solver | afiro | adlittle | share2b | stocfor1 | kb2 |
|---|---|---|---|---|---|
| natDualSimplex | ✅ 6.1e-12 | ✅ 1.1e-11 | ✅ 3.4e-12 | ✅ 1.1e-11 | ✅ 3.5e-12 |
| natIPM | ✅ 8.8e-12 | ❌ MaxIter | ✅ 4.4e-11 | ✅ 1.1e-11 | ✅ 8.7e-11 |
| HiGHS-LP | ✅ 6.1e-12 | ✅ 1.1e-11 | ✅ 3.4e-12 | ✅ 1.1e-11 | ✅ 3.5e-12 |

(relative objective error vs published optimum)

### 3.2 Adversarial diagonal scaling (10^k dynamic range)

Round 2 extended the probe from {afiro, adlittle} to all 5 NETLIB problems. 10^6 rows:

| Case | natDualSimplex | natIPM | HiGHS-LP |
|---|---|---|---|
| afiro 1e2 / 1e4 / 1e6 | ✅ ~1e-11 all | ❌ MaxIter all | ✅ 1e-11 / 1e-11 / **7.4e-2 err, "Optimal"** |
| adlittle 1e2 / 1e4 / 1e6 | ✅ ~1e-11 all | ❌ NumericalError all | ✅ 1e-11 / 1e-11 / **1.6e-1 err, "Optimal"** |
| share2b 1e2 / 1e4 / 1e6 | ✅ ~3e-12 all | ❌ NumericalError all | ✅ 3e-12 / 3e-12 / **3.3e-1 err, "Optimal"** |
| stocfor1 1e2 / 1e4 | ✅ ~1e-11 | ❌ NumericalError | ✅ ~1e-11 |
| stocfor1 1e6 | 🛈 **honest FAIL ("Residual audit rejected")** | ❌ NumericalError | ❌ **7.0e17 err, "Optimal"** |
| kb2 1e2 / 1e4 / 1e6 | ✅ ~3.5e-12 all | ❌ NumericalError all | ✅ 3.5e-12 / 3.5e-12 / **9.99e-1 err, "Optimal"** |

Native simplex (Ruiz equilibration + UMFPACK with iterative refinement) is *more* robust than HiGHS at extreme scaling: correct on 4/5 at 10^6 where HiGHS is false-optimal on 5/5, and the residual audit (§5.1, hardened in §5.6) converts the one unsolvable case into an honest failure instead of HiGHS' "Optimal but wrong" answer.

### 3.3 SCUC LP relaxations (large, degenerate)

| Case | natDualSimplex | natIPM | HiGHS-LP (1 thread) |
|---|---|---|---|
| 6-bus/4T | ✅ 0.8 ms, viol 1e-14 | ❌ NumericalError | ✅ 2.4 ms |
| 39-bus/24T | ✅ 56 ms, viol 1e-13 | ❌ NumericalError (garbage obj 5.8e102, correctly flagged) | ✅ 14 ms |
| 118-bus/24T | **✅ 40.1 s, viol 3e-12** (was FAIL before this work) | ❌ ProblemTooLarge (graceful) | ⏱ time limit 300 s (678k iterations, unconverged) |

### 3.4 LP-level conclusions

- **natDualSimplex:** accuracy = HiGHS class on all tested sizes; extreme-scaling robustness superior. Speed: parity on small LPs, ~4× slower than HiGHS on 39-bus relaxation.
- **natIPM (IPM-LP):** the weakest kernel — hard failures on degenerate (adlittle), SCUC relaxations, and every scaled problem. Anything routed through it (`use_ipm_root`, `NativeBC[natIPMroot]`) inherits the fragility.

## 4. MILP-level results (pure native B&C vs HiGHS)

| Case | NativeBC[natSimplex] | NativeBC[natIPMroot] | HiGHS[direct] |
|---|---|---|---|
| 6-bus/4T | ✅ same optimum, 21–36 ms | ✅ same optimum, 31 ms | ✅ 2.1 ms |
| 39-bus/4T | ✅ same optimum, 15 ms | ✅ 36 ms | ✅ 2.3 ms |
| 39-bus/24T | ✅ same optimum, 78 ms | ✅ 72 ms | ✅ 14.5 ms |
| 118-bus/24T (round 1) | ❌ no valid solve, ~600 s ("Time limit", garbage incumbent 2.1e11) | ❌ same | ⏱ 300 s: incumbent 163k, gap ~14% |
| 118-bus/24T (round 2) | 🛈 honest trajectory: fallback solves root on retry, best_bound 135.7k, no incumbent yet, "Time limit" at 455 s (~1.5×) | 🛈 same (bound 135.7k) | ⏱ 300 s: incumbent 163k, bound 144.4k |

- Small/medium MILPs: **identical optima as HiGHS**, clean feasibility (~1e-13), but 5–15× slower despite native using multiple threads vs HiGHS pinned to 1.
- Root certificates: 0-node "Optimal (tree exhausted)" on the tight-relaxation cases is legitimate (root domain-closure certificates + LP relaxation already integral). **The stale-bound artifact is fixed in round 2 (§5.6):** `best_bound`/`best_obj` are now reported in original objective space alongside `stats.objective` — the 7.5–21% "gap" next to an Optimal status is gone (best_bound == best_obj, gap 0).
- 118-bus round 1: root LP failure → B&C repair path → garbage incumbent 2.1e11, best_bound at the -1e30 sentinel, 0 nodes in ~600 s. Round 2: the unpresolved-root fallback fires (presolved root capped at 50% of the budget), the retry solves the root and starts the tree (1 node, 10 LPs in the remaining ~150 s — native node-LP throughput is the bottleneck), and reports an honest bound (135.7k vs HiGHS 144.4k) with no fabricated incumbent.

### 4.1 Production hybrid comparison (milp_benchmark_runner --full, 120 s)

| 118-bus @120 s | Objective | Best bound | Gap |
|---|---|---|---|
| NativeBC + StrictHiGHS `Fallback12k[S]` | **152,521.56** | **143,604.73** | **5.85%** |
| NativeBC + StrictHiGHS `Seed+A/B[S]` | 157,536.17 | 141,219.61 | 10.4% |
| HiGHS direct (1 thread) | 173,203.93 | 140,155.36 | 19.1% |

The native orchestration (dynamic cuts, seeding, fallback policies) on top of the HiGHS LP kernel **beats raw HiGHS** on the large case. This confirms the evaluation's central finding: the gap is in the native numerical kernels, not in the B&C machinery.

## 5. Robustness improvements implemented (2026-07-20, this work)

All changes are uncommitted working-tree edits. Files: `src/engine/kernel/lp_kernel/dual_simplex.cpp`, `include/mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp`, `src/engine/solver/native/milp/bc/legacy/branch_and_cut.cpp` (1-line status sync), `tests/test_numerical_stability.cpp` (2 new tests), `benchmark/native_kernel_comparison.cpp` (new), `CMakeLists.txt` (driver target).

### 5.1 Stage 1 — cold-start escalation chain

- New `SimplexOptions`: `escalation_max_level{2}`, `escalation_umfpack_pivot_tolerance{1.0}`, `escalation_ruiz_rounds{25}`, `escalation_residual_tol{1e-6}`, `escalation_start_level{0}`.
- `solve_lp_with_basis` is now a wrapper around `solve_lp_with_basis_impl(lp, opt, hint, esc_level)`:
  - Level 0 — legacy behaviour (Ruiz 10 rounds, UMFPACK pivot tolerance 0.1).
  - Level 1 — fresh rebuild, Ruiz 25 rounds, UMFPACK true partial pivoting (`SparseBasis::set_pivot_tolerance`, previously never overridden from the UMFPACK default).
  - Level 2 — KLU rescue backend (§5.2).
- **False-optimal audit:** cold-start "Optimal" results must pass an original-space residual gate (`max viol ≤ tol·max(1,|b|∞,|beq|∞)`), otherwise treated as failure and escalated — directly counters the HiGHS-style "Optimal but 7–16% wrong" failure mode found in §3.2.
- Escalation only fires for cold-start solves (`allow_cold_start == true`, i.e. root LPs); warm-started tree node LPs are never retried.
- Inter-level wall-clock budget check (the escalation loop no longer doubles the effective time limit at LP level).

### 5.2 Stage 2 — KLU rescue backend

- `SparseBasis::FactorBackendKind::KluRescue` (internal only, not reachable from the public `SimplexFactorBackend` ids): two fresh KLU factorizations (B and B^T — this Eigen has no transpose-solve API), standard eta updates between refactorizations, `Eigen::SparseLU` (pivot threshold 1.0) as inner fallback, `#ifdef MIPSOLVERS_HAVE_KLU` gated (chain degrades to level 1 without KLU).

### 5.3 Stage 3 — HiGHS-style Phase I (flag-gated, A/B)

- Mirrors `HEkk.cpp:2440` / `HEkkDual.cpp:2076` / cleanup: magnitude-aware deterministic perturbations (`highs_style_perturb`), exact-zero cost shifts pinned at the dual boundary, and `dual_cleanup_resolve` restoration back to the unperturbed problem before Phase II.
- **Bug found & fixed during A/B:** the initial implementation also perturbed the RHS (±1e-7); on tightly-balanced SCUC equality systems (power-balance rows where capacity barely meets demand) that made the perturbed LP *genuinely infeasible* and produced false "LP infeasible" certificates. Now only costs and finite variable upper bounds are perturbed (matching HiGHS).
- **A/B verdict: NOT a win — default stays 0.** ~2× slower on 39-bus relaxation; turns a strategy-0-solvable 118-bus relaxation into "Phase II failed"; no better than strategy 0 on the presolved root LP either.

### 5.4 Stage 4 — status & diagnostics hygiene

- `branch_and_cut.cpp`: root-LP failure path now syncs `bc_stats.status` with `stats.status` (empty status strings eliminated).
- `[SIMPLEX-DIAG]` stderr print gated behind `opt.verbose`.
- `SolveStats::iterations` now reports true pivot counts via a thread-local accumulator (was hard-coded 0).
- Stale file-organization comment block in `dual_simplex.cpp` corrected (incl. `ruiz_scale_standard_form` having moved to `dual_simplex_api.cpp`).

### 5.5 Stage 5 — tests & validation

- New Catch2 cases in `tests/test_numerical_stability.cpp`: KLU-forced solve of a 240-row degenerate assignment LP (matches default path to 0 error, row violation < 1e-8) and escalation-chain integration (`escalation_max_level=2` agrees with legacy within 1e-9).
- Full regression: `test_numerical_stability` (370 assertions/11 cases), `test_dual_simplex`, `test_netlib_regression`, `test_lp_solver`, `test_ipm_solver`, `test_presolve`, `test_milp_solver`, `test_engine_api`, `test_problem_validation`, `test_adapter_registry`, `test_l2o_trace` — **all green**.
- Production path: `milp_benchmark_runner` quick suite 16/16, timings unchanged.

## 5.6 Round 2 — B&C failure-path hardening, unpresolved-root fallback, SF escalation, presolve diagnostics (2026-07-20)

Implements roadmap items P0(a,b,c) + P1 (all) + P2-short + P4 from §7, plus three audit bypasses and two budget/bookkeeping bugs found by the new gates while verifying ("round 2.5"). All changes are uncommitted working-tree edits on top of round 1.

### Round-2 proper (planned stages)

- **P1 — failure-path hardening (Stage A).** New `BCOptions::incumbent_quality_reject_factor{1e6}` gates `adopt_root_incumbent` and the time-limit publish path against the root dual bound. Sentinel hygiene: `return_with_profile` scrubs `best_bound ≤ -1e29 → -inf`, `best_obj/gap ≥ 1e29 → inf`; tree-exhausted optimality syncs `best_bound = best_obj`. LP-stage budgeting in `solve_lp_relaxation`: every stage (IPM root / simplex / IPM repair) runs against `lp_remaining_budget()` with explicit exhaustion guards — the 120 s-limit failure row dropped from ~2× to ~1.04× the limit. Tests in `tests/test_milp_solver.cpp`.
- **P0(a) — unpresolved-root fallback (Stage B).** New `BCOptions::root_presolve_fallback{true}`, `root_presolve_fallback_min_remaining_sec{5.0}`, `BCStats::presolve_fallback_attempts`; on root-relaxation failure with PaPILO actually reducing the model, the whole solve is retried once with presolve off (recursion depth bounded at 2; background tasks joined first; test seam `MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL=1`). Tests in `tests/test_branch_and_cut.cpp`.
- **P0(c) — escalation for `solve_lp_from_sf` (Stage C).** The standard-form entry (cut re-solves, IPM-repair crossover) gets the same cold-start escalation chain: per-level backends, extra-Ruiz copy-solve with publish-via-warm-re-solve (no output back-mapping), caller-space residual audit `sf_solution_residual_acceptable`, separate knob `escalation_max_level_sf{2}`. Tests: level agreement, publish-space residuals, warm-hint immunity.
- **P2-short — IPM health probe (Stage D).** `BCOptions::ipm_root_probe{true}`, `ipm_probe_max_iter{20}`, `ipm_probe_time_sec{5.0}`: a few relaxed-tolerance IPM iterations on the exact root LP classify NumericalError/Cholesky-failed/ProblemTooLarge as unhealthy and skip the doomed full solve (and the repair-path IPM); a healthy probe seeds the full solve.
- **P0(b) — presolve-hardening diagnostics (Stage E).** New `MatrixScalingStats` + `compute_matrix_scaling_stats` on the PaPILO result (`[PRESOLVE-SCALING]` lines, env `MIPSOLVERS_PRESOLVE_SCALING_DIAG=1`). **Finding: presolve does NOT harden the 118-bus matrix by any scaling metric** — dynamic range improves 1.2e10 → 2.8e9, worst intra-row range 1.5e7 → 3.5e6, zero rows above 1e8, and the aggressive-rerun swap barely changes the picture. The presolved root is harder for the native Phase I for structural reasons (fewer but denser rows/columns after substitution: max_col_nnz 364 → 157 on a matrix 1/5 the nnz), not coefficient blow-up. Unit test in `tests/test_presolve.cpp`.
- **P4 — evaluation infra (Stage F).** Driver `--check` flag (exit 1 on must-pass failures) registered in CTest as `native_kernel_comparison` (LABELS benchmark); scaling probe extended to all 5 NETLIB (table in §3.2); `lp_solution_residual_acceptable` exposed in `dual_simplex.hpp` with corrupted/true-solution unit tests; `tests/test_branch_and_cut.cpp` created.

### Round-2.5 — bugs found by the new gates while verifying

1. **Last escalation level bypassed the false-optimal audit** (`level < max_level` guard), returning garbage "Optimal" from the final level. Deterministic trigger: stocfor1 at 10^6 scaling (rel-err 7.1e17, caught by `--check`). Now demoted to an honest `"Residual audit rejected"` failure at the last level, on both the LPModel and SF wrappers.
2. **1e20 no-bound sentinels poisoned the audit scale.** `tol · max(1, |b|∞)` used sentinel row sides (1e20, ×10^6 instance scaling → acceptance threshold ~1e20+), so the audit accepted anything. Sides with `|side| ≥ 1e19` are now excluded from both the violation and the scale.
3. **NaN objective returned as "Optimal"** (stocfor1-1e6 again: finite-but-wrong x, `c·x` non-finite). A non-finite objective now fails the audit as well.
4. **Escalation levels each got a fresh full time budget** (the impl measures wall clock from its own start) — the 118-bus root burned ~1.8× the MILP limit. Each level now receives only the remaining budget (both wrappers).
5. **The P0(a) fallback never fired as designed**: the presolved root died by *time limit* (whole budget eaten by escalation), not by a failure status the trigger required. Two changes: `root_presolve_fallback_budget_fraction{0.5}` reserves retry budget by capping the presolved root LP, and the trigger now also accepts a root-stage time limit (a genuinely expired limit is still filtered by the min-remaining guard).
6. **Cold-start root simplex had `max_iter = max_lp_iter·10 = 5000`** — two orders of magnitude below what the unpresolved 118-bus relaxation needs (~1e5 pivots, ~40 s), so the fallback retry's root failed with MaxIter. Cold solves now get `max(..., 100000)`; the wall-clock budget is the real governor.
7. **Fake 7.5–21% "gap" on proven-optimal solves**: `bc_stats.best_obj/best_bound` were reported in reduced-model objective space while `stats.objective` was recomputed in original space (folded constant 59,500 on 39-bus). The bookkeeping is now shifted by the same delta; tree-exhausted rows report `best_bound == best_obj`, gap 0.

### Round-2 results

- `ctest -L unit` 10/10; `test_branch_and_cut` green; `native_kernel_comparison --check` 0 must-pass failures (now gating in CTest).
- LP level: tables unchanged (NETLIB ~1e-11; SCUC relaxations; 118-bus relaxation still ~40 s); scaling probe table in §3.2.
- MILP level: small-case optima unchanged, fake gaps gone; 118-bus row in §4 (honest trajectory, bound 135.7k, ~1.5× limit).
- Production path unaffected (all changes are on native-kernel/failure paths; StrictHiGHS rows route around them).

## 5.7 Round 3 — tree node-LP warm-start diagnosis + fail-fast (2026-07-20)

Motivation: native B&C tree throughput with the native LP kernel is orders of magnitude off HiGHS (retry tree: 10 LPs in ~150 s; HiGHS: ~720k LPs in 300 s), i.e. incremental (warm-start) LP solving was not delivering.  New diagnostics: driver env overrides `MIPSOLVERS_NKC_VERBOSE` / `MIPSOLVERS_NKC_REQUIRE_TREE_CERT`, and a kernel-level `--warm-probe` (shared Ruiz-scaled SF + bound-only updates + root-basis hint, exactly the B&C node pattern; `--warm-probe-39` adds a HiGHS LP cross-check per probe).

**Findings (layered).**

1. **The kernel warm machinery is functional in isolation at 118-bus scale.** Single branch fixings from the root basis re-optimize warm in 35–1379 pivots (78 ms–5.4 s), cut-appended re-solves in 43–255 pivots (0.13–0.8 s); infeasibility certificates match HiGHS exactly on 39-bus (away-fixings there are genuinely infeasible — SCUC commitment logic — and both solvers agree).
2. **In the real B&C context, 8/10 PATH-A warm attempts fail** (LP-STATS: `entered=10 ok=1 fail=8`), and each failure burns ~5 s because the doomed warm reopt was allowed `max(2000, m)` iterations, *plus* an equally large crash-repair retry, before the cold fallback ran.  Failed warm starts alone accounted for ~40 s of ~52 s LP time in the 118-bus failure path (`fail_ms=40.3k`).
3. **Even successful warm re-solves are ~100–1000× slower per LP than HiGHS** (0.1–5 s vs ~0.4 ms): the per-pivot cost at 35.6k rows (~3 ms) and the pivot counts for single bound changes (hundreds–thousands vs HiGHS' tens) are the structural bottleneck — this is the P3 simplex-speed item, not a bookkeeping bug.

**Change (fail-fast warm caps).** Warm-reopt iteration caps reduced to `max(2000, m/2)` (crossover/root-level, was `max(2000, m)`) and `max(500, m/10)` (tree nodes, was `max(500, m/2)`); the crash-repair retry is capped at `max(500, m/10)` as well (was `max(2000, m)`).  A doomed warm start now bails to the cold fallback quickly instead of indulging two long reopts.  Verification: `--warm-probe(-39)` results unchanged (all observed legitimate re-solves need ≤ ~1.4k pivots, far below the new caps); LP-STATS on the 118-bus B&C row (240 s limit, require-tree-cert): `entered=10 ok=1 infeas=0 fail=8 | COLD=24 | warm_ms=30 cold_ms=12.8k fail_ms=34.6k`.

**Round-3 addendum (2026-07-20, second session) — root cause of the 8/10 warm failures found.**

New kernel instrumentation: per-`return false` reason codes in `sparse_dual_simplex_reoptimize` (wall-limit / 4× refactorization-failure sites / NaN-guard / max-iter) plus a `SparseBasis::refactorize` sub-reason detail, both aggregated into `[LP-STATS]`.  A new `--warm-probe` **tree-chain section** (cumulative away-fixings, parent-basis hint — the exact B&C descent pattern) reproduces a depth-2 `LP infeasible` verdict on 118-bus that HiGHS **confirms as genuinely infeasible** (cross-check now built into the probe): the warm machinery issues no false certificates, in isolation or chained.

Instrumented tree run (same row as above):

```
[LP-STATS] PATH_A fail reasons: refact-periodic=6 max-iter=2 | reopt_iters_burned=19277
[LP-STATS] PATH_A fail refact detail: umf-numeric=8
```

i.e. the tree-context warm failures are **not** iteration-cap exhaustion (the round-3 hypothesis) and not basis bookkeeping (no duplicate/out-of-range columns): every failure bottoms out in **UMFPACK's numeric phase rejecting the warm-drifted basis as (numerically) singular** at a periodic refactorization.  Small pivots admitted by the BFRT (admission `max(1e-9, 1e-7·‖w‖∞)`, hard guard only 1e-10) compound `|det B| ← |det B|·|pivot|` over hundreds–thousands of pivots until a fresh LU sees an exactly-singular matrix; the reopt then has no repair path (fail → crash-repair reopt drifts the same way → fail → cold fallback).  This also means the fail-fast caps buy less than hoped (fail_ms 40.3k → 34.6k; the failures die at expensive refactorizations, not at the cap).

**Two attempted cures, both rejected by evidence:**

1. *Relative pivot admission (Harris-style `max(1e-10, 1e-8·‖direction‖∞)` + candidate blacklist + Farkas gating).*  Fixed the tree failures in principle but **broke the 118-bus cold root** (`Simplex Phase II failed`, 137–193 s vs 44 s healthy): the cold dual Phase I (which shares `sparse_dual_simplex_reoptimize`, call site at the shifted-cost Phase-I) *needs* pivots in the rejected band on this model.  Reverted.
2. *KLU rescue on umf-numeric* (demote `SparseBasis` to the existing `KluRescue` backend and retry the refactorization).  Also broke the cold root — at m≈27.7k KLU's BTF ordering degenerates to one giant block (two full factorizations per refactorize), turning each rescue into a multi-second grind; an initial version additionally left stale `slu_/nlu_` state behind (fixed, but the speed verdict stands).  `MIPSOLVERS_FACTOR_BACKEND=hfactor` was probed as an alternative: cold Phase I fails immediately (2.9 s) — the vendored HFactor port is not cold-start-capable at this scale yet.  Both reverted; the stale-`slu_` invalidation in the `KluRescue` branch is kept as a correctness guard.

**Status:** the failure mechanism is pinned (UMFPACK numeric singularity of warm-drifted bases), correctness of all verdicts is HiGHS-verified, and the cost is bounded by the fail-fast caps.  A real fix needs one of: (a) a HiGHS-style *basis repair* on singularity (replace the dependent basic columns with logicals and continue — requires column-level dependence info UMFPACK does not expose, or a rank-revealing factor); (b) maturing the HFactor port for the warm path only (it is HiGHS' own factor and handles rank deficiency natively); (c) pivot-quality work inside the BFRT that does not starve Phase I (item P3).

**Not fixed (explicitly out of scope, P3):** per-pivot cost at 35.6k rows (FTRAN/BTRAN, refactorization frequency, DSE weight updates; candidate lever: the already-built vendored `mipsolvers_hfactor` backend A/B) and the tree-context warm failures themselves (see addendum — mechanism identified, cure deferred to (a)/(b)/(c) above).

## 5.8 Round 4 — warm-failure mechanism hunt: what the singularity is NOT (2026-07-20)

Systematic elimination campaign against the round-3 finding (tree PATH-A warm failures all bottom out in `umf-numeric` refactorize failures).  New instrumentation shipped: per-return-site fail-reason codes + `SparseBasis::refactorize` sub-reason detail aggregated in `[LP-STATS]`; env-gated autopsies `MIPSOLVERS_SING_DIAG` (min column norm + pairwise cosine on failing bases), `MIPSOLVERS_PIVOT_WATCH` (minimum admitted pivot per trajectory), `MIPSOLVERS_MAX_ETAS` (eta-chain cap), and a `--warm-probe` tree-chain section with HiGHS ground-truth cross-check on any failure verdict.

**Ruled out (each with a controlled experiment):**

1. *Small pivots / death-spiral det(B)→0.*  PIVOT-WATCH on healthy cold solves: minimum admitted pivot **3.0e-3** (39-bus) and **1.49e-7** (118-bus, 18.3k pivots).  An absolute 1e-8 pivot floor + candidate blacklist + Farkas gating was implemented and verified zero-perturbation on both probes (bit-identical iteration counts and objectives) — **tree failures unchanged** (`fail=8, umf-numeric=8`).  (The earlier relative floor `1e-8·‖direction‖∞` was rejected: ‖direction‖∞ reaches 1e6 on this model family, turning it into an absolute 1e-2 floor that starves legitimate pivots — cold root `Phase II failed`.)
2. *Eta-chain drift.*  `MIPSOLVERS_MAX_ETAS=8` (vs 40 default): still `umf-numeric` — singular bases arise within ≤8 pivots of a fresh factorization.
3. *Duplicate or zero columns.*  SING_DIAG autopsy on 20+ failing bases: min column 2-norm ≈ 1.0, best pairwise cosine = 0.0.  Eigen SparseLU cross-check confirms the bases are **genuinely** singular (119/123), so the dependence is a many-column numerical combination, not a bookkeeping bug (no duplicate basis indices either — detail≠2).
4. *HFactor as drop-in backend.*  Two sub-findings: (a) the vendored HFactor's native FT update (`updateFT`) is itself **broken** — stores the new U column correctly but leaves an incomplete factorization (missing row-eta content; Sherman-Morrison-verified divergence at the 2nd consecutive update on a standalone 200×200 reproducer; ASan-clean); (b) HFactor build + eta replay + HiGHS-style logical repair works mechanically but **destroys the simplex trajectory** on this model family — borderline bases are chronic, and each logical swap surfaces another deficient row (whack-a-mole: 90+ repair rounds, cold Phase I fails).  A UMFPACK-side variant (HFactor diagnostic build to identify no-pivot rows, then logical swap) was equally churny and is now **default-off** (`MIPSOLVERS_UMF_REPAIR=1` keeps it for experimentation).

**Kept (verified non-regressing):** the 1e-8 absolute pivot floor with blacklist/Farkas gate (defensive; zero-perturbation), the `refactorize_with_repair` machinery (HFactorPort path), the HFactorPort build+eta rewire (fresh HFactor LU + native eta stack — avoids the broken `updateFT`), the stale-`slu_/nlu_` invalidation in the `KluRescue` branch, all LP-STATS instrumentation, and the probe self-test/diagnostics.

**Current understanding.**  Warm-hint bases from the parent solve (with the node's tightened domain) drift to *exact* singularity within a handful of pivots, on trajectories whose admitted pivots are all ≥ 1e-8.  The remaining candidate mechanism is the ratio/pivot *selection logic itself* on highly-degenerate bound-tightened vertices (BFRT long-step admitting columns that are exactly dependent in exact arithmetic — pivot element ~1e-8-ish nonzero but structurally dependent), i.e. the simplex **pricing/pivoting strength** issue, not a numerical-tolerance tuning issue.  Next concrete step: capture one failing (basis, SF) pair and extract the actual dependent row/column set (HFactor no-pivot rows on the failing basis — machinery in place), then either repair surgically without trajectory loss or strengthen dual pricing on degenerate vertices.  Note this does not gate the headline gap: even with all warm failures eliminated, per-pivot cost (~3 ms at 27.7k rows vs HiGHS ~µs) keeps native ~100× behind on this family — the P3 simplex-speed program remains the main lever.

## 6. Open issues (ranked, post-round-2)

1. **Presolved 118-bus root LP (m≈27.7k) still unsolvable natively** (unchanged — but now routed around by the P0(a) fallback, §5.6). Round-2 diagnostics (§5.6, Stage E) rule out coefficient blow-up: presolve *improves* every scaling metric. The hardening is structural — substitution/aggregation produces fewer but denser rows/columns (max_col_nnz 364 → 157 at 1/5 the nnz), which the native dual Phase I handles far worse than the sparse original. Next diagnostic step: compare Phase-I iteration/factorization profiles on the two matrices, not scaling stats.
2. **Fallback retry overruns its own time budget (~1.5× the MILP limit).** The retry solves the root and starts the tree, but tree-side work (root cut rounds, node LPs) still runs against the stale `root_solve_opt.time_limit_sec` snapshot taken once before the root solve. Plumb a live `bc_remaining_sec()`-derived budget into tree-stage LP/cut re-solves.
3. **Native node-LP throughput.** 10 LPs in ~150 s on the retry's tree (HiGHS: ~720k LPs in 300 s). Even with a valid root, the native tree barely moves at 118-bus scale — the FTRAN/BTRAN/refactorization costs from round-1 item 5 dominate everything downstream. Round-3 addendum (§5.7) pins the warm-failure mechanism (UMFPACK numeric rejection of warm-drifted bases; no false certificates — HiGHS-verified) and records two rejected cures; the repair path is now scoped to (a) HiGHS-style basis repair on singularity, (b) HFactor warm-path maturation, or (c) BFRT pivot-quality work that does not starve Phase I.
4. **IPM-LP (natIPM) robustness** (unchanged): NumericalError/MaxIter on degenerate and SCUC-relaxation LPs; mitigated at B&C level by the round-2 health probe (doomed root solves are now skipped cheaply), but the kernel itself is unfixed. `use_ipm_root` / `natIPMroot` configurations remain unsafe at scale.
5. **natDualSimplex speed** — ~4× slower than HiGHS on the 39-bus relaxation (parity on small LPs).

## 7. Future improvement suggestions (prioritized roadmap)

**P0 — Root-LP robustness at scale.** ✅ done in round 2 (§5.6)
- (a) ✅ B&C-level unpresolved-root fallback (incl. budget reserve + widened trigger + cold-start iteration cap from round 2.5).
- (b) ✅ Presolve-hardening diagnostics — scaling blow-up ruled out; structural follow-up is now §6 item 1.
- (c) ✅ Escalation chain ported to `solve_lp_from_sf`.

**P1 — B&C failure-path hardening.** ✅ done in round 2 (§5.6)
- Incumbent-quality gate; sentinel scrub (`-1e30` never leaks — verified on the 118-bus failure row, now `-inf`); LP-stage remaining-budget plumbing (2× → ~1.04× at the LP stage); tree-exhausted `best_bound` sync (+ the original-space offset alignment found in round 2.5).

**P1.5 (new) — Time-limit plumbing beyond the LP stage.** Tree/cut-round stages still use a stale pre-root budget snapshot (§6 item 2); target ≤ ~1.1× worst-case overrun on the 118-bus fallback path.

**P2 — IPM-LP rescue or isolation.**
- ✅ Short term (round 2): health-probe gate skips doomed IPM root solves.
- Medium term: fix the barrier-μ blow-up on degenerate problems (the adlittle failure mode), reusing the escalation ideas (Ruiz rounds retry already exists; add regularization escalation + fallback-to-simplex).

**P3 — Simplex speed (competitiveness, not robustness).** Now the top priority for native MILP at scale (§6 item 3).
- Profile the ~4× gap vs HiGHS on 39-bus-class LPs: refactorization frequency, hyper-sparse FTRAN/BTRAN coverage, DSE weight updates.
- The vendored `mipsolvers_hfactor` backend is already built — a controlled A/B of `HFactorPort` vs `UmfpackNativeA` on the benchmark suite would quantify what HiGHS' HFactor buys inside our simplex.
- KluRescue is correctness-first; if level-2 fires often in production, revisit its cost model.

**P4 — Evaluation infrastructure.** ✅ done in round 2 (§5.6)
- `native_kernel_comparison --check` wired into CTest (LABELS benchmark, not in the CI unit tier).
- Scaling probe covers all 5 NETLIB problems; stocfor1-1e6 is the standing deterministic false-optimal rejection case (native: honest failure; HiGHS: 7e17 rel-err "Optimal").

## 8. Reproduction

```bash
cmake . && cmake --build . --target native_kernel_comparison -j$(nproc)

# LP level (NETLIB + SCUC relaxations + scaling probe over all 5 NETLIB)
./tests/native_kernel_comparison --full --skip-milp --time-limit 300

# MILP level (pure native B&C vs HiGHS)
./tests/native_kernel_comparison --full --skip-lp --time-limit 300

# Must-pass gate (registered in CTest, LABELS benchmark)
./tests/native_kernel_comparison --check --time-limit 60
ctest -R "native_kernel_comparison|milp_benchmark" --output-on-failure

# HiGHS-style Phase I A/B
./tests/native_kernel_comparison --full --skip-milp
./tests/native_kernel_comparison --full --skip-lp

# Presolve scaling diagnostics (round 2, Stage E)
MIPSOLVERS_PRESOLVE_SCALING_DIAG=1 ./tests/native_kernel_comparison --full --skip-lp --time-limit 300 2> presolve_scaling.txt

# Root-fallback test seam
MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL=1 ./tests/test_branch_and_cut

# Production hybrid ablation
./tests/milp_benchmark_runner --full --no-highs-adapter --json out.json
```

Result logs referenced in this document: `/tmp/lp_level_results.txt`, `/tmp/stage1_lp_results.txt`, `/tmp/stage2_milp_results.txt`, `/tmp/stage3_lp_results.txt`, `/tmp/stage3_milp_results.txt`, `/tmp/milp_benchmark_full.txt` (+ `.json`) — round 2: `/tmp/presolve_scaling_118.txt` (Stage E diagnostics), `/tmp/round2_milp_118b.txt` (fallback verification), `/tmp/round2_lp_full.txt` (LP-level re-validation), `/tmp/nkc_check3.txt` (`--check` gate).

# IEEE-118 SCUC / Native LP-Kernel — Status & Bug Analysis

_Snapshot for deeper analysis. Branch `dev`. Date: 2026-07-31._

## 0. TL;DR

- **Shipped win (committed `73269e6`):** root-caused the "3 unsound presolve
  reductions" to a single missing objective-cost transfer in
  `process_singleton_columns`. Fixed it, enabled singleton-row/forcing/probing
  by default, fixed an empty-model edge case. Result: 8-suite 8/8, `--check`
  exact, **39-bus/24h native 386 → 43 ms (9×)**, and 118-bus presolve now
  reduces **35599 → 28210 rows / 26400 → 16301 cols / nnz 517173 → 255687**
  (HiGHS presolve *skips* 118-bus — over its nnz cap).
- **Open problem:** 118-bus still does **not** beat HiGHS. At an 8 s limit the
  native solver returns garbage (obj ≈ 2.1e11 warm-start penalty, `bound=-inf`,
  `nodes=0`). Root cause is now precisely located (§3–§5): the **root LP
  relaxation never converges in budget** because the **native dual simplex is
  ~3–4× slower per pivot than HiGHS** *and* the LP intrinsically needs ~10× more
  pivots than fit the budget.
- **In-progress (uncommitted):** LP-kernel profiling instrumentation + two safe
  micro-opts in `native_dual/solver.cpp`; net effect so far only ~4 % (§6).

## 1. The user-visible bug

`NativeBC[natSimplex]` on `UC_118bus_54G_24T` at a short time limit:

| field | value | meaning |
|---|---|---|
| objective | ~2.1e11 | the warm-start penalty value — **useless** |
| best_bound | −inf | root LP never produced a dual bound |
| nodes | 0 | tree never started |
| status | Time limit reached | |

HiGHS on the same model at 8 s: `obj≈4e9, bound=137272` — also not optimal, but
it produces a real bound and explores 18621 simplex iters. True optimum ≈156199;
true root-LP bound ≈123363.

## 2. Which kernel actually solves it (important correction)

The `NativeBC[natSimplex]` benchmark row sets
`lp_kernel_backend = ExperimentalNative`, so `uses_highs_lp_kernel()==false` and
`solve_lp_relaxation` **never enters the vendored-HiGHS path**. Earlier
HiGHS-side experiments (setSolution / IPM / crossover) were **dead code** for
this row. The 118-bus root LP is solved by the **native dual simplex**
(`src/engine/kernel/lp_kernel/native_dual/solver.cpp`), cold, via the
`if (opt.use_simplex_lp_nodes)` path at the bottom of `solve_lp_relaxation`.

> Note: `ExperimentalNative` is a **development-only** kernel; the production
> Native[B&C] product path uses the HiGHS LP kernel. This lowers the blast
> radius of kernel changes but the benchmark/`--check` must-pass row exercises
> the native kernel, so its speed is what the campaign is measured on.

## 3. Root-LP profile (native dual simplex on the 118-bus root)

Env `MIPSOLVERS_DS_PROFILE=1` → `[DS-PROFILE]` printed at the dual-simplex time
limit. Standard-form root: **m = 28210, n = 45250** (the standard-form build
nearly triples 16301 structural cols with slacks).

Per `solve_impl` (a representative window, normalized to **ms / pivot**):

| phase | ms/pivot | share | what it is |
|---|---|---|---|
| leaving (CHUZR+BTRAN) | 0.16 | 10 % | dual pricing + BTRAN for `row_ep` |
| price (Aᵀ·row_ep) | 0.15 | 9 % | pivot-row PRICE, O(nnz A) |
| entering (ratio/BFRT) | 0.22 | 14 % | bound-flipping ratio test |
| ftran | 0.09 | 6 % | B⁻¹·a_q |
| dse | 0.11 | 7 % | steepest-edge weight update |
| **postcond** (O(n) dual-feas) | 0.02 | 1 % | analytical postcondition check |
| **validBlock** (predFull+resid) | 0.20 | 12 % | full-state reconstruction + `‖Ax−b‖` audit |
| **OTHER** (update/copy) | 0.64 | **40 %** | per-pivot state commit + copies |
| **total minor** | ~1.60 | 100 % | |

`major_rebuild` (LU refactorization) ≈ 0.02 s total — **cheap**.
`edgeInit` / `primalPhaseI` ≈ 0 (cold basis is diagonal → steepest-edge init is
free; dual-feasible cold start avoids primal Phase I).

**Reads:** the real simplex work (leaving+price+entering+ftran+dse) is ≈ 46 %;
the remaining ≈ 54 % is per-pivot **overhead**: full-state reconstruction /
validation (`validBlock`) and — dominantly — the **per-pivot state commit
(`OTHER`)**.

## 4. What `OTHER` (0.64 ms/pivot, 40 %) actually is

Every pivot, `minor_iteration` builds and commits **full-length dense vectors**:

- `candidate_reduced_costs = reduced_costs + step·pivot_row + Δcost` — O(n).
  This is the **inherent** dual reduced-cost update (pivot_row is dense) and
  cannot be made sparse without a different pricing scheme.
- `candidate_move = state.move` (copy, O(n)) then ~3 entries changed.
- `candidate_basis = state.basis` (copy, O(m)) then 1 entry changed.
- `state.cost = candidate_cost` / `state.cost_shift = candidate_cost_shift`
  (O(n) each) — only actually change when `transaction.cost_shifts` is
  non-empty, but are recomputed/copied **every** pivot.
- Misc O(m) temporaries (`flipped_basic`, `bfrt_delta`, `predicted_post_pivot`).

So `OTHER` = 1 inherent O(n) update + ~4 avoidable O(n)/O(m) **full-vector
copies** per pivot on n≈45k dense vectors → memory-bandwidth bound. HiGHS avoids
this with packed/incremental structures.

## 5. The fundamental barrier (why per-pivot tuning is not enough)

- Pivot rate: native ≈ **500–650 pivots/s**; HiGHS ≈ 2000/s on the same LP.
- A dual simplex needs ≈ m…3m pivots to converge → **~28 000–84 000 pivots** for
  m=28210. The root-LP budget fits only ~**2000–4000** pivots.
- ⇒ Even a 2× per-pivot speedup (≈1000–1300 pivots/s) still gets ≤ ~8000 pivots
  in budget — **the LP still will not converge**, so `bound` stays −inf.

**Conclusion:** per-pivot optimization helps every case (esp. 39-bus/24h) and is
worth doing, but it **cannot alone** make 118-bus converge. Crossing HiGHS on
118-bus additionally requires **fewer pivots** (a warm-start *basis* rather than
the integer incumbent, better pricing, or aggressive presolve to shrink m) or a
**different method** (barrier was measured *worse* — dense normal equations from
SCUC coupling).

## 6. Optimization attempts so far (uncommitted, `solver.cpp`)

1. **Reuse per-pivot scratch buffers** (`MinorScratch`, thread_local): eliminates
   ~6 allocations/pivot. Byte-identical results. **Net speedup ≈ 0** — the
   allocations were cheap (cached); the cost is the O(n) *zeroing/ops*, not
   malloc.
2. **Gate the full audit to periodic** (`kPivotAuditStride`): the
   `predicted_full` reconstruction + `‖Ax−b‖` residual + 2nd dual-feas loop now
   run every K pivots (or paranoid) instead of every pivot; the objective is
   refreshed on audit and stays *stale-but-valid* (dual objective is monotone →
   a conservative lower bound) between. Reduced `validBlock` 0.26 → 0.20
   ms/pivot. **Net ≈ 4 %.** `--check` exact; `test_numerical_stability`,
   `test_dual_simplex`, `test_netlib_regression`, `test_lp_solver`,
   `test_milp_solver` all PASS.

Both are correctness-safe but individually small. The dominant `OTHER` (the
full-vector copies) is untouched.

## 7. Open bugs / risks / cleanups

- **Uncommitted profiling scaffolding** in `solver.cpp` (`DSProfile`,
  `g_ds_profile`, per-phase timers). Env-gated, zero-cost when off, but must be
  cleaned or kept deliberately before commit.
- **Periodic-audit change** relaxes the "certified every pivot" contract to
  "certified every K pivots + at optimality". Tests pass, but this is a design
  posture change for `ExperimentalNative` and should be reviewed / made an
  option before shipping.
- `bc_relaxation.cpp`: the `30000` warm-start-crash gate was renamed to a
  documented `kWarmstartCrashMinCols` (behavior-preserving). The crash-basis
  path itself does **not** help 118-bus (integer seed is a poor LP basis) and
  regresses medium models if the gate is lowered — kept high on purpose.
- Pre-existing unrelated working-tree changes: `.github/workflows/ci.yml`
  (−211), `.gitignore` (+2) — **not** from this work.

## 8. Plan (next steps, in priority order)

1. **Kill the per-pivot full-vector copies (OTHER, the 40 %).** Skip
   `candidate_cost`/`candidate_cost_shift`/`transaction_cost_shift` entirely when
   `transaction.cost_shifts` is empty (common case), and update
   `state.move`/`state.basis` **in place** (only the changed entries) instead of
   copy-modify-move. Target: `OTHER` 0.64 → ~0.30 ms/pivot ⇒ overall ~1.6× per
   pivot. Validate the full suite + `--check`.
2. **Reduce pivot count** (the real 118-bus lever): build a warm-start *basis*
   (not the integer solution) for the cold root — e.g. crossover from a cheap
   partial solve **in the native standard-form space** — so the dual simplex
   starts near-optimal. This is the only thing that can make 118-bus converge in
   budget.
3. **Publish a valid root bound on timeout.** Even unconverged, the dual
   objective is a valid lower bound; publish it instead of `−inf` so 118-bus
   reports a real (weak) gap rather than garbage.
4. Decide the disposition of the profiling scaffolding and the periodic-audit
   posture (option-gate vs default) before committing the kernel work.

## 9. Reproduce

```sh
# build
cmake --build build_mipsolvers --target native_kernel_comparison --parallel 8

# correctness gate (0 must-pass failures, exact objectives)
./tests/native_kernel_comparison --check

# per-phase dual-simplex profile of the 118-bus root
env MIPSOLVERS_DS_PROFILE=1 ./tests/native_kernel_comparison --full --time-limit 20 \
  2>&1 | grep DS-PROFILE

# full LP/numerical suites
for t in test_lp_solver test_numerical_stability test_dual_simplex \
         test_netlib_regression test_milp_solver; do ./tests/$t; done
```

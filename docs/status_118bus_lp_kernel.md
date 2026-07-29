# IEEE-118 SCUC / Native LP-Kernel — Status & Bug Analysis

_Snapshot for deeper analysis. Branch `dev`. Started 2026-07-31 (§0–§9);
updated 2026-07-30 with the deep-profiling round (§10–§13)._

## 0. TL;DR

- **Shipped win (committed `73269e6`):** root-caused the "3 unsound presolve
  reductions" to a single missing objective-cost transfer in
  `process_singleton_columns`. Fixed it, enabled singleton-row/forcing/probing
  by default, fixed an empty-model edge case. Result: 8-suite 8/8, `--check`
  exact, **39-bus/24h native 386 → 43 ms (9×)**, and 118-bus presolve now
  reduces **35599 → 28210 rows / 26400 → 16301 cols / nnz 517173 → 255687**
  (HiGHS presolve *skips* 118-bus — over its nnz cap).
- **User-visible bug FIXED (uncommitted, §12):** `NativeBC[natSimplex]` on
  118-bus at 8 s used to return `obj≈2.1e11 / bound=−inf` (garbage). It now
  returns a **kernel-certified dual bound** (`bound≈64.8k–66.1k` at 8 s) and no
  longer publishes the useless warm-start-penalty incumbent. The bound is
  rigorous: certified against the *original* cost by the same dual-feasibility
  audit an Optimal result rests on (§11).
- **Analysis correction (§10):** the earlier §4 attribution of the 40 %
  per-pivot `OTHER` cost to "full-vector copies" was **wrong on this
  hardware** — eliminating ~5.4 MB/pivot of copies moved nothing (Apple-silicon
  bandwidth makes them ~40 µs). Deep profiling found the real hidden costs:
  the **O(m+n) cycle-guard signature recomputed every pivot (11 %)** and the
  **O(nnz) backward-error residual validation inside every checked
  FTRAN/BTRAN** (~19 % top-of-stack in `sample`). The cycle signature is now
  maintained incrementally (Zobrist-style) → **+15 % pivots/s** end-to-end;
  the checked-solve posture is quantified but deliberately untouched.
- **New capability discovered (§13):** the native dual simplex now solves the
  *unpresolved* 118-bus root LP to **true optimality (obj 123363.6, 37054
  iters, ~58 s)** where vendored HiGHS-LP fails a 20 s limit; and **natIPM
  solves the same root in 5.4 s** — the strongest known lever for plan item
  §8.2 (root bound via IPM + crossover).

## 1. The original user-visible bug (fixed — see §12)

`NativeBC[natSimplex]` on `UC_118bus_54G_24T` at a short time limit:

| field | before | after (this round) |
|---|---|---|
| objective | ~2.1e11 (warm-start penalty — useless) | no incumbent published (quality gate) |
| best_bound | −inf | **≈64786–66136, certified** |
| nodes | 0 | 0 (root LP still cannot converge in 8 s) |
| status | Time limit reached | Time limit reached |

HiGHS on the same model at 8 s: `obj≈4e9, bound=137272`. True optimum ≈156199;
true root-LP bound ≈123364.

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
limit. Standard-form root: **m = 28210, n = 45250**.

Per-pivot profile **after this round** (7.08 s window, 4886 pivots; baseline
from the previous snapshot in parentheses):

| phase | ms/pivot | share | what it is |
|---|---|---|---|
| leaving (CHUZR+BTRAN) | 0.21 (0.16) | 15 % | dual pricing + checked BTRAN for `row_ep` |
| price (Aᵀ·row_ep) | 0.15 (0.15) | 10 % | pivot-row PRICE, O(nnz A) |
| entering (ratio/BFRT) | 0.19 (0.22) | 13 % | bound-flipping ratio test |
| ftran | 0.09 (0.09) | 6 % | checked B⁻¹·a_q |
| dse | 0.11 (0.11) | 7 % | steepest-edge weight update |
| postcond (O(n) dual-feas) | 0.02 (0.02) | 1 % | analytical postcondition check |
| validBlock (predFull+resid) | 0.19 (0.20) | 13 % | periodic full-state audit (stride 16) |
| **luUpdate** (new timer) | 0.11 (—) | 8 % | Forrest–Tomlin LU update per pivot |
| **cycleGuard** (new timer) | **0.00 (0.18)** | 0 % | cycle-signature + taboo bookkeeping — **eliminated (§10.2)** |
| OTHER (update/copy) | 0.35 (0.46¹) | 25 % | rc dense update, basic re-zero, new x_basic, allFinite guards, BFRT FTRAN on flip pivots |
| **total minor** | **1.42 (1.63)** | 100 % | **+15 % pivots/s** |

¹ the old table's 0.64 "OTHER" implicitly contained luUpdate and cycleGuard,
which had no timers then.

`major_rebuild` ≈ 0.14 s total — cheap. `primalPhaseI`/`edgeInit` ≈ 0.
Pivot statistics on this LP: bound flips are rare (≈2.4 % of pivots) and cost
shifts did not occur at all — the flip/shift machinery is off the common path.

## 4. What `OTHER` actually was — corrected attribution

The previous snapshot attributed `OTHER` (then 0.64 ms/pivot) to per-pivot
full-vector copies and predicted ~1.6× from eliminating them. **That was
falsified by measurement (§10.1):** all of the listed copies (~5.4 MB/pivot)
were eliminated with byte-identical semantics and the profile did not move.
At this machine's memory bandwidth (~100+ GB/s) the copies cost ~40 µs/pivot,
not 0.6 ms. The real components hiding in `OTHER` were:

- **cycle-guard signature**: `compute_cycle_signature` rehashed all m basis
  tokens + all nonbasic move tokens (~73k elements × 4 avalanche mixes) after
  **every** pivot → 0.18 ms/pivot (11 % of the whole solve). Fixed (§10.2).
- **LU update** (`BasisFactor::update` → `HFactorBackend::update`):
  0.11 ms/pivot — inherent, comparable to HiGHS' own update cost.
- the genuinely inherent state update: dense O(n) reduced-cost update, O(m)
  basic re-zeroing scatter, new O(m) primal vector, two allFinite guards, and
  the BFRT-rhs FTRAN on the rare flip pivots — the remaining ≈0.35 ms/pivot.

A `sample`-profiler run over the whole solve additionally shows
`BasisFactor::residual_norm` + `equation_residual_vector` (the **backward-error
validation inside every checked FTRAN/BTRAN** plus the periodic audits) at
roughly **25–30 % of wall time**, spread across the `leaving`/`ftran`/`valid`
timers. This is the price of the kernel's "checked solve" contract, and it is
the single largest structural gap to HiGHS' per-pivot speed. It is left
untouched: relaxing it is a design-posture decision, not an optimization.

## 5. The fundamental barrier (unchanged in kind, better in degree)

- Pivot rate: native ≈ **690/s** after this round (was ≈600/s); HiGHS ≈ 2000/s.
- The presolved root needs ≈ m…3m pivots (~28k–84k); an 8 s budget now fits
  ~4900 (was ~4200).
- ⇒ Per-pivot tuning still **cannot alone** make the 118-bus root converge in
  budget. Crossing HiGHS on 118-bus requires **fewer pivots**. The two
  measured levers (§13): natIPM solves the root LP in **5.4 s** (74 iters,
  exact bound 123363.6), and the native dual simplex *does* converge on the
  root when given ~60 s. An IPM-root + crossover-in-native-standard-form
  path is now clearly the right §8.2 implementation.

## 6. Optimization attempts — ledger (all uncommitted, this working tree)

1. **Reuse per-pivot scratch buffers** (previous round): byte-identical,
   **≈ 0** net — allocations were never the cost.
2. **Periodic full audit** (`kPivotAuditStride = 16`, previous round): reduced
   `validBlock` 0.26 → 0.20 ms/pivot, **≈ 4 %**. Kept.
3. **Per-pivot copy elimination** (this round, §10.1): skip the
   cost-shift vectors (`transaction_cost_shift`, `candidate_cost`,
   `candidate_cost_shift` + their commit copies) on shift-free pivots, mutate
   `state.move`/`state.basis` in place behind an exact-undo `PivotStateGuard`,
   alias `flipped_basic` to `x_basic` on flip-free pivots, swap (not copy)
   `x_basic`/`reduced_costs` scratch at commit, sparse re-zero of the FTRAN
   column and flip-mark scratches, move-not-copy the FTRAN solution.
   **Measured ≈ 0 on this hardware** (see §4/§10.1) but it removes
   ~5.4 MB/pivot of traffic (matters on bandwidth-poor targets), and it
   removed the allocations the incremental-signature work then built on.
   Values identical up to signed-zero in skipped `+0.0` adds.
4. **Incremental cycle signature** (this round, §10.2): cycleGuard
   0.18 → 0.00 ms/pivot ⇒ **+15 % pivot throughput end-to-end**
   (4260 → 4886 pivots in the same 7 s root window; 39-bus/24h MILP
   43 → ~35 ms).
5. **Certified dual bound at time/iteration limit** (this round, §11): not a
   speed change — turns the 118-bus 8 s result from garbage into a rigorous
   bound.

## 7. Open bugs / risks / cleanups

- **Uncommitted scaffolding** in `solver.cpp`: `DSProfile` (env-gated,
  zero-cost off) now also times `luUpdate`/`cycleGuard`; keep or strip before
  commit. `MIPSOLVERS_DS_VERIFY_SIG=1` cross-checks the incremental cycle
  signature against a full recomputation at every pivot (validated: all five
  suites + `--check` run clean under it).
- **Incremental-signature contract**: `record_cycle_arrival` now consumes
  `State::cycle_signature_live_*`, maintained at pivot commit and resynced in
  `major_rebuild`/`initialize_cycle_guard`. Any new code path that mutates
  `basis`/`move` outside those points must call `resync_cycle_signature`
  (the taboo unit test does exactly this — see `test_dual_simplex.cpp`).
  The `b` accumulator changed from a sequential hash chain to XOR-of-mixed
  tokens (Zobrist); collision structure is equivalent, signature *values*
  differ from the previous build.
- **Periodic-audit posture** (stride 16) unchanged from the previous round;
  still a design decision to ratify before shipping. Raising the stride to 64
  is worth ~0.14 ms/pivot more if the posture allows.
- **Quality-gate interaction**: now that a finite root bound exists on the
  118-bus timeout path, `incumbent_quality_acceptable` (factor 1e6) rejects
  the 2.1e11 warm-start incumbent — the row reports bound-only. That is the
  gate working as designed on honest data; flagged here because the row's
  "objective" column changed meaning (no incumbent vs garbage incumbent).
- `bc_relaxation.cpp`: the `30000` warm-start-crash gate was renamed to a
  documented `kWarmstartCrashMinCols` (behavior-preserving, previous round).
- Pre-existing unrelated working-tree changes: `.github/workflows/ci.yml`
  (−211), `.gitignore` (+2) — **not** from this work.

## 8. Plan (updated after this round)

1. ~~Kill the per-pivot full-vector copies~~ — **done, measured ≈ 0 here**;
   the hypothesis was wrong (§4). Kept for traffic reduction.
2. ~~Publish a valid root bound on timeout~~ — **done and certified** (§11).
3. **Reduce pivot count (the real 118-bus lever), now concretely:** solve the
   root with **natIPM (5.4 s to the exact bound, §13)** and cross over to a
   simplex basis *in the native standard-form space*, so the dual simplex
   starts near-optimal and the B&C keeps its simplex certificates. The
   existing IPM-repair path in `solve_lp_relaxation` already does
   IPM → `recover_primal_activity_basis_impl` → simplex; it is just never
   reached for the ExperimentalNative root because the simplex runs first and
   eats the whole budget. Ordering/budget-split experiment next.
4. **Optional per-pivot follow-ups**, in measured-value order: raise audit
   stride 16 → 64 (~0.14 ms/pivot), then revisit the checked-solve
   residual-validation posture (~25–30 % of wall) **only** as an explicit
   design decision.
5. Decide the disposition of the profiling/verify scaffolding and commit the
   kernel work in reviewable slices (copy-elimination + signature + certified
   bound are separable).

## 9. Reproduce

```sh
# build
cmake --build build_mipsolvers --target native_kernel_comparison --parallel 8

# correctness gate (0 must-pass failures, exact objectives)
./tests/native_kernel_comparison --check

# per-phase dual-simplex profile of the 118-bus root (incl. new timers)
env MIPSOLVERS_DS_PROFILE=1 ./tests/native_kernel_comparison --full --skip-lp \
  --time-limit 8 2>&1 | grep -E "DS-PROFILE|UC_118"

# cross-check the incremental cycle signature on every pivot
env MIPSOLVERS_DS_VERIFY_SIG=1 ./tests/native_kernel_comparison --check

# standalone 118-bus root LP through the native kernel (unpresolved)
env MIPSOLVERS_DS_VERBOSE=1 ./tests/native_kernel_comparison --lp118-native

# full LP/numerical suites
for t in test_lp_solver test_numerical_stability test_dual_simplex \
         test_netlib_regression test_milp_solver; do ./tests/$t; done
```

---

# Progress log — deep-profiling round (2026-07-30)

## 10. Falsifying §4 and finding the real per-pivot costs

### 10.1 Copy elimination: implemented, then measured ≈ 0

All §8.1 items were implemented in `minor_iteration`
(`native_dual/solver.cpp`): shift-free pivots skip the three cost-shift
vectors and both commit copies; `state.move`/`state.basis` are mutated in
place behind `PivotStateGuard` (exact undo on every numerical-trouble return —
the driver rebuilds from `state.basis`, so a partially applied exchange would
corrupt the iterate); `x_basic`/`reduced_costs` swap with thread-local scratch
at commit; the FTRAN column and flip-mark scratches keep an all-zero invariant
via RAII guards and are re-zeroed sparsely. One real bug was caught by the
warm-start suite along the way: the flip-mark scratch can be sized for a
*previous* solve of different n, so the postcondition loop must treat it as
absent unless `flipped.size() == n` (out-of-bounds read otherwise).

Result: `--check` exact, all suites green — and **no measurable speedup**.
The eliminated traffic (~5.4 MB/pivot) is ~40 µs at Apple-silicon bandwidth.
The §4 model ("memory-bandwidth bound") was wrong for this machine.

### 10.2 The real finds: cycle guard and checked-solve validation

New `DSProfile` timers (`luUpdate`, `cycleGuard`) plus a `sample` run gave the
corrected attribution in §3/§4. The actionable one: `record_cycle_arrival`
recomputed a hash over **all** m basis tokens and **all** nonbasic move tokens
after every pivot — 0.18 ms/pivot, 11 % of the solve, for information that
changes by ~3 tokens per pivot.

Fix: the signature's second accumulator was converted from a sequential hash
chain to XOR-of-mixed-tokens (both accumulators are now order-independent
Zobrist set-hashes — same collision structure), and the signature is
maintained incrementally: `cycle_signature_apply_basis_swap` + two or three
`cycle_signature_apply_move_toggle` calls at the pivot commit, with a full
`resync_cycle_signature` in `major_rebuild` (which reclassifies every
nonbasic side) and `initialize_cycle_guard`. An env hook
(`MIPSOLVERS_DS_VERIFY_SIG=1`) verifies live == full-recompute at every
arrival; the five suites and `--check` run clean under it. The one divergence
it ever caught was the taboo *unit test* driving `record_cycle_arrival`
directly after hand-mutating `basis`/`move` — the test now resyncs, matching
the documented contract.

**Measured: cycleGuard 0.75 s → 0.01 s over the 7 s root window; pivots
4260 → 4886 (+15 %); total 1.63 → 1.42 ms/pivot. 39-bus/24h MILP 43 → ~35 ms.
`--check` exact; all 5 suites green (1018/1018 dual-simplex assertions).**

## 11. Certified dual bound on interrupted solves (§8.3 — done)

The naive plan ("publish `state.objective` on timeout") would have been
**unsound**: on a cold primal-infeasible start `initialize_stabilized_cost`
perturbs the working cost, so the running objective is a dual bound for the
*perturbed* problem only (and the infinite-upper-bound perturbation signs make
a finite correction impossible).

Implemented instead — `certify_interrupted_dual_bound` (kernel, runs once at
`TimeLimit`/`IterationLimit`): `restore_original_cost` →
`reconstruct` (checked FTRAN/BTRAN) → `normalize_nonbasic_moves` (re-classify
bound sides against original reduced costs; fails if a lower-only column has
positive reduced cost) → dual-only `audit`. On success, `state.objective` is
the exact original-cost dual objective of an audited dual-feasible point —
by weak duality a rigorous lower bound on the LP minimum, resting on the same
certificate as an Optimal result. Cost: ~one pivot's worth of work, once.

Plumbing: `native_dual::Result::dual_bound_certified` →
`SolveStats::certified_dual_bound` (NaN when absent; the uncertified legacy
`stats.objective` publication is preserved verbatim for the vendored-HiGHS
kernel) → `LPRelaxationResult::dual_bound` (only when certified) →
`branch_and_cut` root-timeout path feeds `best_known_root_bound`, and
`publish_verified_warm_start_time_limit` now publishes the bound
**independently of** incumbent acceptance.

## 12. 118-bus benchmark row — before/after (8 s limit)

```
before  NativeBC[natSimplex]  obj 210410672237.67   bound -inf        nodes 0
after   NativeBC[natSimplex]  obj (none published)  bound 64785.90*   nodes 0
        HiGHS[direct]         obj 3988093847.76     bound 137272.32
```
\* certified; run-to-run 64.8k–66.1k depending on where the 8 s window cuts.
The root uses two dual-simplex attempts (presolved + fallback), and the bound
comes from the certification at the second attempt's timeout. HiGHS' bound is
stronger because its ~19k simplex iterations dwarf our ~7.8k across both
attempts — closing that is §8.3-plan item 3 (IPM root), not more bound
plumbing.

## 13. New measurements that reshape the plan

`--full --skip-milp --time-limit 20` (LP section), 118-bus **unpresolved**
relaxation (m=35599, n=62786):

| kernel | time | iters | objective | status |
|---|---|---|---|---|
| natDualSimplex | 57.5 s | 37054 | **123363.61 (true root bound, exact)** | Optimal |
| natIPM | **5.4 s** | 74 | 123363.61 | Optimal |
| HiGHS-LP (vendored) | 20.1 s | 29974 | 2.4e9 | FAIL: time limit |

The native kernel is no longer wrong or fragile on this LP — it is *slow per
pivot* (checked-solve contract) and *pivot-hungry* (cold start). natIPM
reaching the exact root bound in 5.4 s makes the IPM-root + native-crossover
ordering the highest-value remaining change for the 118-bus campaign.

## 14. Validation state (this working tree)

- `./tests/native_kernel_comparison --check` — **0 must-pass failures**, exact
  objectives (also under `MIPSOLVERS_DS_VERIFY_SIG=1`).
- `test_lp_solver` 41/41 · `test_numerical_stability` 394/394 ·
  `test_dual_simplex` 1018/1018 · `test_netlib_regression` 220/220 ·
  `test_milp_solver` 131/131 — all also clean under the signature-verify env.
- Files touched this round: `native_dual/{solver,state}.cpp`,
  `native_dual/{state,model}.hpp`, `native_dual_core.hpp`,
  `dual_simplex.cpp`, `solver_adapter.hpp`, `bc_relaxation.cpp`,
  `branch_and_cut.cpp`, `tests/test_dual_simplex.cpp`.

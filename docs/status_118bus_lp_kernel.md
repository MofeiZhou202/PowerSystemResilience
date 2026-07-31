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

---

# Progress log — natIPMroot bound + basis-free heuristic + checked-solve tiering (2026-07-30)

## 15. `NativeBC[natIPMroot]` fails at 8 s despite `natIPM` solving the root

### 15.1 Diagnosis — the converged IPM bound was discarded (not an IPM problem)

Verbose trace (`MIPSOLVERS_NKC_VERBOSE=1`, 118-bus, 8 s) showed the IPM is **not**
the bottleneck:

1. native presolve 35599→28210 rows / 26400→16301 cols (37 ms);
2. `[BC_RELAX] IPM root: success=1 status='Optimal' iter=53 obj=123363.6143
   time=1.781s` — the IPM **converges to the exact root LP bound in 1.8 s**
   (faster than the 5.4 s standalone because it runs on the natively-presolved
   model);
3. because `use_simplex_lp_nodes=true` ⇒ `require_simplex_crossover=true`, a
   **mandatory IPM→simplex crossover** runs, starts from a numerically bad
   basis and times out (`unstable_pivot_rejections=3345`, `sxObj` regresses to
   62730, `sxSfPr=inf`);
4. the crossover-fail path falls through to a cold simplex, `lp_budget_exhausted()`
   fires, and the relaxation returns `success=false` **without writing
   `out.dual_bound`** → default `+inf` → `branch_and_cut` publishes
   `bound=-inf` and the 2.1e11 warm-start garbage.

The `ipm_root_dual_bound` variable (bc_relaxation.cpp) was declared for exactly
this retention but **was never assigned** — the documented fix was never wired
up.

### 15.2 Fix (Option A, committed `36b96d7`)

Retain the converged IPM root objective and publish it on the crossover-timeout
fallthrough (and floor the cold-simplex certified bound with it). Result at 8 s:
`NativeBC[natIPMroot]` now reports **bound = 123363.6140** (the exact root LP
bound, was −inf) and the quality gate rejects the 2.1e11 incumbent — matching
`natSimplex`'s honest reporting but with a ~2× stronger bound (`natSimplex`
64785; HiGHS 137272 with root cuts). Validated: 5 LP/MILP suites, `--check`
0 must-pass, small cases exact.

### 15.3 Measured dead-ends for a 118-bus incumbent (all reverted)

- **IPM crash-basis crossover** (force the optimal-partition crash basis): 6/39-bus
  converge in 12–349 pivots, but 118-bus thrashes worse — `unstable_pivot_
  rejections=16955`, objective regresses 123363→77243. Severe SCUC degeneracy
  makes the optimal-partition basis near-singular.
- **Skip crossover → basis-free heuristics**: reaches the heuristics (lp_solves
  1→3) but overruns the budget (12.5 s @8 s) and finds no incumbent. Every
  incumbent-capable heuristic (`feasibility_pump`, `prog_round`, `dive`, `lns`,
  `low_fractionality_rens`) is gated off for large roots; native RENS is only
  enabled for the vendored-HiGHS kernel.
- **Force native low-fractionality RENS @30 s**: the IPM root is only 216/3882
  fractional (ideal for RENS), and RENS runs and fixes 1278 vars — but its
  residual sub-MILP LP is solved by the native dual simplex, which hits its
  5000-iteration limit with `unstable_pivot_rejections=9651`. No incumbent.

**Unified conclusion:** every path to an integer-feasible 118-bus point (root LP,
crossover, RENS sub-solve) needs the native dual simplex to solve a variant of
the degenerate SCUC LP, and it can't (~5000 iters, ~9k unstable rejections
everywhere). The IPM solves the LP but yields no integer-usable basis, and
IPM-at-all-nodes is a documented dead end. This is the **LP-kernel gap**, not a
heuristic/orchestration gap.

## 16. Dual-simplex stability — the anti-degeneracy is already advanced

`unstable_pivot_rejections` are per-*candidate* skips (`stability_blocked_rows=0`),
not stalls: the Harris two-pass BFRT still finds a stable pivot every iteration
(pass 2 picks the largest-α pivot), backed by DSE pricing, taboo anti-cycling,
and cost-shifting. The barrier is pivot **count × per-pivot cost** (§4/§5), not
stability. There is no targeted stability fix to make.

## 17. Checked-solve tiering (opt-in, default off)

The single largest recoverable per-pivot cost is the checked-solve contract:
every FTRAN/BTRAN runs an O(nnz) backward-error residual (`residual_norm`) on
every solve. New `SimplexOptions::tier_checked_solves` (default **false**; env
`MIPSOLVERS_DS_TIER_CHECKED_SOLVES=0/1`) tiers the two hot per-pivot solves
(`choose_leaving` row_ep BTRAN, pivotal-column FTRAN) so the residual audit runs
only on the periodic audit stride (`kNativeDualAuditStride=16`), trusting a
finite raw LU solve in between. Soundness is preserved: the periodic full
primal-residual audit reinverts on drift, `certify_interrupted_dual_bound` and
`reconstruct` stay strict, and optimality is only declared after a fresh rebuild.

**Measured** (118-bus SF root, ~6.9 s window): pivots 4756→5189 (**+9.1%**),
`leaving` 0.214→0.112 ms/pivot (−48%), `ftran` 0.088→0.054 (−39%); the certified
8 s bound is *tighter* (64785→67642). It is +9 %, not ~1.4×, because §4's
"25–30 %" also counted the periodic `validBlock` audit that is deliberately kept
as the drift backstop. It does **not** cross HiGHS on 118-bus (needs *fewer*
pivots, §5). Validated: 5 suites + `--check` 0 must-pass under
`MIPSOLVERS_DS_TIER_CHECKED_SOLVES=1`; default path byte-identical.

Files (uncommitted): `dual_simplex.hpp`, `native_dual/{model,factor}.hpp`,
`native_dual/{factor,pricing,solver}.cpp`.

---

# Theoretical analysis — why the native simplex root is still slow (2026-07-30)

## 18. Decomposing the wall time: pivot count × per-pivot cost

The root-LP wall time factors exactly as

$$T_{\text{root}} \;=\; N_{\text{pivots}} \times c_{\text{pivot}}.$$

Both factors are worse than HiGHS, but for **different** reasons: $N_{\text{pivots}}$
is a property of the *method* (simplex on a degenerate LP), while $c_{\text{pivot}}$
is a property of the *implementation* (hyper-sparsity handling). Measured anchors:

| solver | LP-root time | pivots/iters | per unit |
|---|---|---|---|
| native dual simplex | 57.5 s | 37054 | **1.55 ms/pivot** |
| vendored HiGHS-LP | 20.1 s (unconverged) | 29974 | **0.67 ms/pivot** |
| native IPM | **5.4 s** | **74** | 73 ms/iter |

### 18.1 The pivot count is a method limit, not an implementation bug

Dual simplex needs $\Theta(m)$–$\Theta(3m)$ pivots; with $m=28210$ (presolved
standard form) the $\sim$30k–37k pivots of *both* the native kernel and HiGHS-LP
are squarely in that band. SCUC degeneracy (min-up/down feasibility windows,
symmetric identical-generator commitments) inflates the count *within* the band
but not pathologically — and crucially HiGHS-LP is **also** stuck (29974 iters,
fails a 20 s limit). So no simplex implementation crosses this root cheaply by
pivoting; the count is intrinsic to the method on a 28k-row degenerate LP.

The interior-point method escapes it: **74 Newton steps** (§13). An IPM's
iteration count is essentially independent of $m$ and of degeneracy
($O(\sqrt{n}\log 1/\varepsilon)$ worst case, $\sim$50–100 in practice); it
replaces 37 000 rank-1 pivots with $\sim$74 full re-factorizations. That is the
entire reason `natIPM` solves the root in 5.4 s — at the cost of an interior
point with **no vertex/basis** for the B&C (§15.3).

### 18.2 The per-pivot cost is hyper-sparsity-blind — the fixable half

Measured per-pivot profile (`MIPSOLVERS_DS_PROFILE`, $m=28210$, $n=45250$,
$\mathrm{nnz}(A)=255687$, 4920 pivots):

| phase | ms/pivot | complexity as coded |
|---|---|---|
| leaving (CHUZR + BTRAN) | 0.209 | O(m) infeasibility scan + BTRAN |
| **price (Aᵀ·row_ep)** | **0.144** | **dense O(nnz(A))** |
| **entering (ratio/BFRT)** | **0.185** | **dense O(n) candidate scan** |
| ftran | 0.085 | B⁻¹a_q |
| dse | 0.108 | steepest-edge weight update |
| postcond (dual-feas) | 0.020 | dense O(n) |
| validBlock (periodic audit) | 0.191 | O(nnz(A)) every 16 pivots |
| luUpdate | 0.093 | Forrest–Tomlin update |
| OTHER (incl. **rc update**) | 0.358 | dense O(n) reduced-cost update + copies |
| **total minor** | **1.40** | |

The decisive new measurement (`[DS-DENSITY]`, same run) is that the two hot
simplex vectors are **extraordinarily hyper-sparse**:

$$\mathrm{nnz}(\texttt{row\_ep}) \approx 6.5 \;(0.02\%\text{ of }m), \qquad
  \mathrm{nnz}(\texttt{pivot\_row}) \approx 44 \;(0.10\%\text{ of }n).$$

This is exactly the "hyper-sparse family" SCUC belongs to (the time-block /
staircase coupling keeps each basis-inverse row localized to a handful of
periods). It means the dense phases above are doing $\sim$3 orders of magnitude
more arithmetic than the problem requires:

- **PRICE** (`multiply_AT`, `native_dual/state.cpp`) forms $A^\top\texttt{row\_ep}$
  by iterating **all** $\mathrm{nnz}(A)=255687$ entries, though `row_ep` has 6.5
  nonzeros and the result 44. A hyper-sparse *row-wise* PRICE costs
  $\mathrm{nnz}(\texttt{row\_ep})\times \overline{\mathrm{nnz}}_{\text{row}} \approx
  6.5\times 9.06 \approx 59$ multiply-adds — a **~4300× arithmetic reduction**.
  This is what HiGHS' `HEkkDual` hyper-sparse PRICE does.
- **Ratio test / BFRT** (`native_dual/pricing.cpp`) loops over **all** $n=45250$
  nonbasic columns testing `pivot_row[j] != 0`, though only $\sim$44 are nonzero
  → should be O(44).
- **Reduced-cost update + dual-feasibility postcondition**
  (`native_dual/solver.cpp`) sweep **all** $n$ columns, though only the 44 with
  `pivot_row[j] ≠ 0` actually change → should be O(44).
- **CHUZR** rescans all $m$ primal infeasibilities each pivot; HiGHS maintains
  the infeasibility set incrementally.

The identifiable dense O(nnz(A)) + O(n) sweeps (PRICE + ratio + postcond + the
reduced-cost update) sum to $\approx 0.5$ ms of the 1.40 ms/pivot, and their
hyper-sparse floor is tens of microseconds. This — not stability (§16) and not
per-solve validation alone (§17) — is the bulk of the 2.3× per-pivot gap to
HiGHS. The kernel already gets hyper-sparse *triangular solves* from
`HFactorBackend` (BTRAN/FTRAN are cheap: 0.085 ms), but then **re-densifies** in
PRICE, the ratio test, and the reduced-cost update.

### 18.3 What the numbers say about the ceiling

- **Per-pivot**: a full hyper-sparse rewrite of PRICE / ratio / rc-update would
  plausibly take the kernel from 1.55 → ~0.5–0.7 ms/pivot, i.e. to parity with
  or below HiGHS-LP. But $0.6\text{ ms} \times 37000 \approx 22$ s — it closes
  the gap to HiGHS yet **still does not** put the root under an 8 s budget,
  because the pivot count is unchanged.
- **Pivot count**: only a *method* change removes it — the IPM (done, 5.4 s), or
  presolve/decomposition that shrinks $m$, or warm starts that cut the count.
- **Therefore** the honest decomposition of "still slow" is:
  1. $N_{\text{pivots}}\approx 37\text{k}$ — intrinsic to simplex on this
     degenerate 28k-row LP (HiGHS-LP pays it too); only the IPM avoids it.
  2. $c_{\text{pivot}}\approx 1.55$ ms — a ~1000×-arithmetic-overhead
     *implementation* gap (dense PRICE/ratio/update on 6.5/44-nonzero vectors),
     provably fixable, and the concrete reason our simplex trails HiGHS' simplex.

The strategic consequence is unchanged and now quantified: the 8 s root is an
**IPM job** (§8.3), and the remaining value of the native simplex is (a) a fast
hyper-sparse *crossover* from the IPM point to a basis, and (b) warm-started
node LPs in the tree — both of which are worth a hyper-sparse PRICE/ratio/update
rewrite, but neither of which is a root-LP speed lever on its own.

### 18.4 Reproduce

```sh
# per-pivot profile + hyper-sparsity of row_ep / pivot_row on the 118-bus root
env MIPSOLVERS_DS_PROFILE=1 ./tests/native_kernel_comparison --full --skip-lp \
  --time-limit 8 2>&1 | grep -E "DS-PROFILE|DS-DENSITY"
```

The `[DS-DENSITY]` line is emitted by an env-gated probe in
`native_dual/solver.cpp` `minor_iteration` (zero cost when `MIPSOLVERS_DS_PROFILE`
is unset), alongside the existing `[DS-PROFILE]` phase timers.

## 19. Optimization step 1 — hyper-sparse PRICE (done, default on)

Acting on §18.2, the dense PRICE (`multiply_AT`, dense $A^\top\texttt{row\_ep}$
over all $\mathrm{nnz}(A)$) is replaced by a **hyper-sparse row-wise PRICE**
(`multiply_AT_hypersparse`, `native_dual/state.cpp`): it scatters only the
nonzero entries of `row_ep` through the row-major `A_row` (which
`StandardFormLP` already maintains and `ruiz_scale_standard_form` scales in
sync). It is numerically equal to the dense product up to summation order and
falls back to the dense product if `A_row` is dimensionally inconsistent.

- New option `SimplexOptions::use_hypersparse_price` (**default true**), env
  `MIPSOLVERS_DS_HYPERSPARSE_PRICE=0/1`; `MIPSOLVERS_DS_VERIFY_PRICE=1`
  cross-checks the hyper-sparse pivot row against the dense product every pivot.

**Measured** (118-bus root, ~7 s window): the PRICE phase collapses
**0.71 s → 0.05 s (~14×**, 0.145 → 0.009 ms/pivot), lifting throughput
4903 → 5319 pivots (**+8.5%** end-to-end). The end-to-end gain is "only" +8.5%
because PRICE was ~10% of the per-pivot cost — the theory (§18.2) is confirmed
exactly, and the *next* dominant terms are now the dense O(n) `entering`
(ratio/BFRT) and `OTHER` (reduced-cost update) sweeps.

**Validation:** 5 LP/MILP suites pass **with 0 `[DS-VERIFY-PRICE]` mismatches**
under `MIPSOLVERS_DS_VERIFY_PRICE=1` (proving equivalence on NETLIB, adversarial
scaling, and node LPs with appended cut rows), and `--check` reports 0 must-pass
failures with the feature default-on.

**Step 2 status:** PRICE now emits its structural column pattern; BFRT and the
non-audit reduced-cost postcondition/update consume it instead of sweeping all
$n=45250$ columns. See §20 for the post-PRICE diagnosis and §21 for the
reduced-cost implementation and measurement.

Files (uncommitted): `dual_simplex.hpp`, `native_dual/{model,state}.hpp`,
`native_dual/{state,solver}.cpp`.

## 20. Post-hyper-sparse analysis: why it is still slow

The sparse pivot-pattern continuation described at the end of §19 is now also
present in the working tree: `choose_entering_bfrt` scans the PRICE pattern
instead of `0..n`. A fresh profile of that code gives:

| phase | before hyper-sparse work (ms/pivot) | PRICE + sparse BFRT (ms/pivot) |
|---|---:|---:|
| leaving (CHUZR + BTRAN) | 0.209 | 0.209 |
| PRICE | 0.144 | **0.014** |
| entering (ratio/BFRT) | 0.185 | **0.030** |
| FTRAN | 0.085 | 0.081 |
| DSE | 0.108 | 0.103 |
| postcondition | 0.020 | 0.020 |
| periodic audit | 0.191 | 0.180 |
| LU update | 0.093 | 0.089 |
| OTHER | 0.358 | 0.361 |
| **total** | **1.40** | **1.09** |

The new run completed 6400 pivots in a 7.16 s simplex window, about **894
pivots/s**. PRICE and BFRT are behaving as intended: together they fell from
0.329 to 0.044 ms/pivot. The important result is that they now occupy only
**4%** of an iteration. The remaining slowness is therefore not evidence that
row-wise PRICE failed; it is evidence that the rest of the iteration is not
hyper-sparse yet.

### 20.1 Amdahl's-law explanation

Before the change, PRICE + BFRT were about 23.5% of the 1.40 ms pivot. Even an
infinitely fast implementation of just those two phases has a speedup ceiling

$$S_{\max}=\frac{1}{1-0.235}\approx 1.31.$$

The measured 1.40 -> 1.09 ms improvement is **1.28x**, very close to that
ceiling. Thus the apparently small end-to-end improvement is the expected
result of optimizing the selected region almost completely.

The 8 s outer budget leaves about 7.2 s for simplex after model construction
and presolve. At the historical 37054-pivot trajectory, it would require

$$c_{\text{pivot}} \le \frac{7.2\text{ s}}{37054}
  \approx 0.194\text{ ms/pivot},$$

or more than another **5.6x** reduction from the measured 1.09 ms. Eliminating
the remaining 0.044 ms PRICE/BFRT cost cannot materially approach that target.

### 20.2 The implementation is hyper-sparse only in the middle

The current dataflow is sparse for the triangular-solve kernel and for
PRICE/BFRT, but becomes dense at their interfaces:

$$
e_p \xrightarrow{\text{dense wrapper}} B^{-T}e_p
\xrightarrow{\text{sparse PRICE}} A^TB^{-T}e_p
\xrightarrow{\text{sparse BFRT}} q
\xrightarrow{\text{dense state updates}} (x_B,\bar c,w).
$$

Concretely:

- `HFactorBackend::{ftran,btran}*` accepts and returns dense length-$m$
  pointers. The large-system path scans the full input to construct an
  `HVector`, runs the hyper-sparse HFactor solve, then copies/scatters a full
  dense result. The `*_for_update` variants additionally construct a new
  `HVector` and save dense RHS/solution buffers. Consequently a solve with a
  one-entry RHS and a 6-entry result is still **Omega(m)** at the wrapper
  boundary.
- `Leaving::row_ep`, `direction`, `rho`, and `pivot_row` are dense Eigen
  vectors. PRICE still zero-initializes a length-$n$ result; guards such as
  `allFinite()` and norms rescan the full vector. The sparse pattern accompanies
  the dense vector instead of replacing it.
- BFRT starts by zeroing a length-$m$ RHS and constructs length-$n$
  `flipped_col`/`shifted_col` markers. Its candidate scan is sparse, but its
  transaction setup is not.
- The reduced-cost commit still evaluates a dense length-$n$ Eigen expression,
  then zeroes basic reduced costs with an $m$-row loop. The analytical
  postcondition separately scans all $n$ columns.
- The primal update forms all $m$ entries of `predicted_post_pivot`. DSE copies
  all $m$ weights, performs another FTRAN, and updates all $m$ rows. CHUZR then
  scans all $m$ rows again to find the largest weighted infeasibility.
- Every 16 pivots, the full audit deliberately reconstructs a length-$n$
  primal vector, scans dual feasibility, evaluates the objective, and computes
  an $O(\mathrm{nnz}(A))$ residual. This alone remains about 0.18 ms/pivot when
  amortized.

Therefore the current iteration is still

$$
\Theta(m+n+\mathrm{nnz}(A)/16)
$$

outside the factorization, despite
$\mathrm{nnz}(\texttt{row\_ep})\approx 6$--12 and
$\mathrm{nnz}(\texttt{pivot\_row})\approx 50$--60. A true hyper-sparse
iteration must preserve index sets across the whole pipeline, not merely use
them for the matrix product and ratio test.

### 20.3 The new dominant costs

After sparse BFRT, the factor/edge pipeline dominates:

- CHUZR + checked BTRAN: 0.209 ms/pivot;
- pivotal-column FTRAN: 0.081 ms/pivot;
- DSE solve and weight update: 0.103 ms/pivot;
- Forrest-Tomlin update: 0.089 ms/pivot.

These four terms already total **0.482 ms/pivot**, 2.5 times the complete
per-pivot budget needed for an 8 s simplex root, before primal/reduced-cost
state maintenance or audits. Much of the first three terms is not sparse
factor arithmetic: it is dense marshaling, backward-error residual evaluation,
vector copying, and dense CHUZR/DSE loops.

With `MIPSOLVERS_DS_TIER_CHECKED_SOLVES=1`, a current 4 s profile falls from
1.09 to about **0.915 ms/pivot** (3235 pivots in 3.04 s, about 1064/s): leaving
drops to 0.105 and FTRAN to 0.050 ms/pivot. This is a useful further 16% but
still projects to roughly 34 s for 37k pivots. It confirms that checked-solve
residuals matter, while also showing that they are not the root solution.

### 20.4 Algorithmic work required for a true hyper-sparse iteration

The next implementation boundary should be an indexed vector (`HVector` or the
existing `SparseVec`) carried end-to-end:

1. Return the BTRAN/FTRAN pattern from `HFactorBackend` without full input and
   output scans; retain the packed update data without duplicating dense RHS
   and solution arrays.
2. Update reduced costs, basic values, and DSE weights in place over touched
   indices, with a sparse undo log for numerical-failure rollback. For reduced
   costs, columns outside the pivot-row/shift pattern are unchanged, so the
   existing dual-feasibility invariant is sufficient between full audits.
3. Maintain CHUZR candidates incrementally. A pivot changes primal values and
   DSE weights only on the union of the FTRAN/BFRT/DSE patterns; update those
   rows in a heap or bucket structure instead of rescanning all $m$ rows.
4. Keep periodic dense audits as a correctness backstop, but measure an
   adaptive cadence separately. At stride 16 their 0.18 ms amortized cost alone
   nearly consumes the entire 0.194 ms target.
5. Remove the profiling observer effect: `[DS-DENSITY]` currently counts
   nonzeros by scanning both dense vectors on every profiled pivot. Once vector
   patterns are authoritative, report their stored counts directly.

This work can plausibly close the remaining per-pivot gap to HiGHS and is
valuable for crossover and warm node LPs. It still cannot guarantee the 8 s
root target: the historical 37k pivot count leaves essentially no budget for
factor solves, updates, and audits even at HiGHS-class pivot speed. The primary
root path should remain IPM; full hyper-sparsity is the enabling work for fast
crossover and simplex reoptimization after the IPM root.

### 20.5 Two measurement corrections

- A 4 s no-profile run completed **3000 pivots** after root setup, consistent
  with the profiled order of magnitude. The density probe makes `OTHER`
  slightly pessimistic but does not explain the multi-fold gap.
- The same run reported **8504 individual bound flips over 3000 pivots**. This
  does not reveal how many pivots contained at least one flip, but it means the
  older statement that flip work is negligible cannot be assumed for the new
  numerical trajectory. Instrument `flip_pivots` separately before optimizing
  away the BFRT-RHS FTRAN path. Hyper-sparse row-wise summation changes floating
  point order; on a degenerate LP, tiny coefficient differences can legitimately
  change the pivot path and hence flip/iteration statistics even when every
  computed tableau row agrees within tolerance.

### 20.6 Reproduce the post-change measurements

```sh
# Sparse PRICE + sparse BFRT, strict checked solves
env MIPSOLVERS_DS_PROFILE=1 MIPSOLVERS_DS_TIER_CHECKED_SOLVES=0 \
  ./tests/native_kernel_comparison --full --skip-lp --time-limit 8

# Same kernel with tiered hot-path solve validation
env MIPSOLVERS_DS_PROFILE=1 MIPSOLVERS_DS_TIER_CHECKED_SOLVES=1 \
  ./tests/native_kernel_comparison --full --skip-lp --time-limit 4

# Iteration count without the density/profile scans
env MIPSOLVERS_DS_VERBOSE=1 ./tests/native_kernel_comparison --full --skip-lp \
  --time-limit 4
```

## 21. Optimization step 2 — sparse reduced-cost invariant (done)

The remaining reduced-cost work in `minor_iteration` now uses the structural
PRICE pattern end-to-end on ordinary pivots:

- the analytical BFRT postcondition scans only pattern columns, including the
  leaving column under its **post-pivot** bound move;
- the reduced-cost commit updates only pattern columns in place and explicitly
  zeros the entering column;
- unchanged basic columns retain their exact-zero invariant;
- every periodic audit pivot still materializes the complete dense candidate,
  zeros the new basis, checks all columns, and runs the original residual audit;
- dense PRICE/fallback configurations retain the original dense reduced-cost
  construction.

This avoids an $O(n)$ Eigen expression, an $O(m)$ basic-column zeroing loop,
an $O(n)$ `allFinite`, and the separate $O(n)$ analytical postcondition on 15
of every 16 pivots. No rollback log is needed: all failure-prone checks and the
factor update occur before the sparse reduced-cost mutation; after that point
the pivot commit cannot fail.

Fresh strict profile, same 8 s command as §20:

| metric | sparse BFRT only (§20) | + sparse reduced costs |
|---|---:|---:|
| simplex window | 7.16 s | 7.17 s |
| pivots | 6400 | **6586** |
| minor cost | 1.089 ms/pivot | **1.057 ms/pivot** |
| postcondition | 0.020 ms/pivot | **<0.001 ms/pivot** |
| reduced-cost materialization | hidden in `OTHER` | **0.014 ms/pivot** |
| throughput change | baseline | **+2.9%** |

The small end-to-end change is consistent with §20: after sparse PRICE and
BFRT, the remaining dense reduced-cost work was only a few percent of the
pivot. The audit-only `rcUpdate` timer is now explicit; most of the remaining
1.057 ms is CHUZR/BTRAN, primal and DSE state maintenance, checked solves, the
periodic audit, and the factor update.

Validation:

- `test_dual_simplex`: 1018 assertions / 23 cases;
- dense fallback (`MIPSOLVERS_DS_HYPERSPARSE_PRICE=0 test_dual_simplex`): same;
- `test_lp_solver`, `test_numerical_stability`, `test_netlib_regression`, and
  `test_milp_solver`: all pass;
- `native_kernel_comparison --check`: 0 must-pass failures;
- `git diff --check`: clean.

**Next implementation target:** expose indexed FTRAN/BTRAN results from
`HFactorBackend` and carry those patterns into the primal and DSE updates. The
factor already computes with `HVector`, but its public dense-pointer API scans
and copies all $m$ entries at both boundaries. Removing that boundary is a
prerequisite for sparse `x_basic`/DSE updates and incremental CHUZR; optimizing
another isolated dense loop before it will have another low Amdahl ceiling.

## 22. Optimization step 3 - indexed factor solves (done)

The HFactor boundary now accepts an optional RHS pattern and returns the
structural result pattern for FTRAN and BTRAN. The backend also retains the
pivotal `HVector`s captured by `ftran_for_update` / `btran_for_update`; the FT
update consumes those vectors directly instead of reconstructing two sparse
packs from dense arrays on every pivot. Capture serials prevent a stale vector
from crossing a rebuild or a later pivotal solve.

The native dual driver carries the patterns through CHUZR, PRICE, pivotal
FTRAN, and DSE. A result pattern is allowed to contain cancellation zeros, but
it must contain every nonzero result entry. `test_dual_simplex` now verifies
that contract directly for indexed FTRAN and BTRAN under a permuted,
non-diagonal basis.

Fresh strict 118-bus measurements after this step (before the later sparse
commit and heap changes):

| metric | sparse reduced costs (§21) | + indexed factor solves |
|---|---:|---:|
| pivots in the 7.16 s simplex window | 6586 | **7412** |
| minor cost | 1.057 ms/pivot | **0.934 ms/pivot** |
| leaving | - | 0.185 ms/pivot |
| FTRAN | - | 0.058 ms/pivot |
| DSE | - | 0.078 ms/pivot |
| periodic audit | - | 0.186 ms/pivot |
| `OTHER` | - | 0.339 ms/pivot |

This was an 8.3% throughput increase over §21 and a 33% reduction from the
original 1.40 ms/pivot baseline. The structural supports averaged 24.7 rows
for `row_ep` and 122.9 rows for the pivotal FTRAN result. These are HVector
supports, not numerical `abs(value)>tol` counts.

## 23. Optimization step 4 - sparse state and incremental CHUZR (done)

Three remaining dense state operations have been removed from ordinary
pivots:

1. `compute_dse_weights` returns validated `(row, new_weight)` assignments
   instead of copying all $m$ weights. Devex and steepest-edge recurrences scan
   the pivotal FTRAN pattern when it is known.
2. The primal transaction computes assignments only for the pivotal FTRAN
   support. It validates them before the LU update and applies them only at the
   commit point. Flip, audit, and unknown-pattern pivots retain the dense path.
3. CHUZR uses a versioned lazy max-heap. Only rows touched by the primal/DSE
   update receive new entries; stale entries are discarded at the top. A
   reconstruction or dense flip invalidates the heap, which is rebuilt on the
   next CHUZR. Taboo-row ordering and the lowest-row deterministic tie break are
   unchanged.

These changes preserve the transaction boundary: no primal or edge-weight
state is mutated until every failure-prone postcondition and the factor update
has succeeded. Duplicate structural indices are harmless because commits are
assignments computed from the pre-pivot state, not repeated increments.

The profiling harness now reports on every solver return and
`--lp118-native --time-limit S` passes the cap into the native LP solve. This
exposed an important measurement limitation: the isolated 118-bus cold run
currently spends the full 8 s in primal Phase I (7830 primal pivots) and never
enters this dual hot loop. Therefore there is no honest final 118-bus
ms-per-dual-pivot number for this step yet. A warm-basis snapshot or a dedicated
Phase-II benchmark entry point is required for the next apples-to-apples run.

## 24. Optimization step 5 - tiered large-basis audits (done)

The old stride-16 full audit cost 0.186 ms/pivot by itself, already exceeding
the requested 0.14 ms/pivot total. Large bases (`m >= 4096`) now use stride 128;
smaller bases retain stride 16. Hot-path FTRAN/BTRAN residual checks use the
same cadence by default, including the advisory DSE solve. Finiteness checks,
pivot stability, the analytical BFRT dual-feasibility postcondition, rebuild
audits, interrupted-bound certification, and the fresh terminal original-space
audit remain active.

The safety/performance controls are explicit:

```sh
# Every-pivot diagnostic posture
env MIPSOLVERS_DS_PARANOID=1 MIPSOLVERS_DS_TIER_CHECKED_SOLVES=0 ...

# Override the periodic full-state/checked-solve cadence
env MIPSOLVERS_DS_AUDIT_STRIDE=16 ...
```

On the reproducible 39-bus relaxation, alternating warm-cache runs kept the
same 178 pivots and objective. The median was about 20.0 ms in paranoid/strict
mode and 17.2 ms with the tiered defaults. This medium model does not exercise
the large-basis stride 128, so it is a correctness/sanity measurement rather
than a proxy for the expected 118-bus audit saving.

### 24.1 Status of the 10% target

The 10% target means approximately 0.14 ms/pivot. It is **not yet measured as
achieved**. From the last comparable 118-bus profile, merely changing the audit
amortization from stride 16 to 128 reduces its theoretical contribution from
0.186 to about 0.023 ms/pivot, a saving of 0.163 ms/pivot. Sparse state commits
and incremental CHUZR remove additional $O(m)$ bandwidth, while tiered solves
remove repeated $O(nnz(B))$ residual products. But the previously measured
entering test (0.031), LU update (0.041), sparse factor solves, and remaining
bookkeeping leave little room under 0.14.

The next required work is therefore not another dense loop in the simplex
driver. It is a warm Phase-II 118-bus benchmark followed by an HVector-native
solve result API that avoids allocating/materializing a dense Eigen vector and
running dense `allFinite` scans for every solve. If that fresh profile remains
above 0.14 ms/pivot, the target requires a different factor-update architecture
or using IPM as the primary large-root path; audit tuning alone cannot supply
another order of magnitude.

Final validation for §§22-24:

- `test_dual_simplex`: 1038 assertions / 24 cases in default, paranoid/strict,
  dense-PRICE, and PRICE/signature-verification configurations;
- `test_lp_solver`: 41 assertions / 6 cases;
- `test_numerical_stability`: 394 assertions / 15 cases;
- `test_netlib_regression`: 220 assertions / 2 cases;
- `test_milp_solver`: 131 assertions / 15 cases;
- `native_kernel_comparison --check --time-limit 8`: 0 must-pass failures;
- `git diff --check`: clean.

## 25. Packed pivot kernels - dense-plus-pattern removed

Sections 22-24 describe intermediate implementations and are superseded by
this section. A pattern attached to an `Eigen::VectorXd` was not a genuinely
indexed algorithm: construction, finiteness checks, fallback paths, and state
transactions could still touch all `m` or `n` entries. Those structures and
fallbacks have now been removed from both simplex pivot loops.

The hot solve boundary now uses `IndexedVector`, with parallel index/value
arrays and no full-length numeric payload. Indices are sorted after HFactor
solves, so random coordinate lookup is logarithmic rather than a linear scan of
the support. HFactor's captured pivotal `HVector`s pass directly to the
Forrest-Tomlin update. If HFactor reports `count < 0` (no valid packed index
list), the wrapper rejects the solve; it no longer accepts an empty result and
silently loses a dense solution.

### 25.1 Revised-dual pivot complexity

The ordinary dual pivot now has these support-bounded operations:

| operation | work |
|---|---:|
| CHUZR | lazy heap changes only |
| BTRAN/FTRAN | HFactor indexed solve support |
| PRICE | nonzeros in rows reached by packed `row_ep` |
| BFRT ratio/postcondition | packed pivot-row support plus shifts |
| BFRT primal update | union of BFRT-FTRAN and pivotal-FTRAN supports |
| reduced-cost update | union of PRICE and cost-shift supports |
| DSE/Devex update | pivotal-FTRAN support |
| FT update | captured HFactor vectors |

There is no periodic pivot audit, full primal reconstruction, dense candidate
reduced-cost vector, dense BFRT marker, dense cost-shift vector, or dense PRICE
fallback in `minor_iteration`. The obsolete audit/PRICE switches and dense
hypersparse helper were deleted so the pivot cannot silently select the old
path.

### 25.2 Primal Phase I and cleanup

The cold 118-bus benchmark did not spend time in the revised-dual loop; it was
running primal Phase I. That separate pivot loop contained another complete set
of full scans and dense solve/state temporaries. It now uses:

- an indexed mutable entering heap with at most one entry per column;
- packed pivotal FTRAN and update BTRAN;
- ratio tests over the FTRAN support only;
- packed row-wise PRICE;
- sparse bound-flip, primal, and reduced-cost commits;
- captured-vector FT updates;
- no pre-pivot full-state audit.

Publication still requires a fresh reconstruction and terminal audit. That is
outside the pivot loop. A fixed 256-update reinversion interval controls FT
fill and drift. Measurements rejected 32 (18.8 s), 128 (14.1 s), 512 (9.0 s),
and 320 (numerical reconstruction failure); 256 was the fastest successful
setting.

### 25.3 Fresh 118-bus result and remaining floor

For `UC_118bus_24T-relax` (`m=35,599`, `n=62,786`), the old cold run completed
about 7,830 primal pivots in an 8-second window without solving. The packed
kernel with the 256-update policy solves the relaxation in **7.44 s**, with
37,475 pivots, objective `1.23363614e+05`, row violation about `1.0e-12`, and
bound violation about `2.0e-13`.

The two primal phases measured:

| phase | pivots | time | ms/pivot | FTRAN | PRICE | update |
|---|---:|---:|---:|---:|---:|---:|
| Phase I | 19,854 | 2.765 s | 0.139 | 1.655 s | 0.074 s | 0.014 s |
| Phase II | 17,621 | 4.639 s | 0.263 | 2.723 s | 0.329 s | 0.028 s |

This is a large improvement, but the weighted cost is about 0.198 ms/pivot,
so the 0.14 ms/pivot interpretation of the 10% target is not yet met. Packed
FTRAN alone contributes about 0.117 ms/pivot. The next material improvement is
therefore inside HFactor's triangular/FT solve representation and fill growth,
not another driver-level vector conversion. Reusing output storage may remove
some allocation cost, but it cannot supply the remaining reduction unless the
factor solve itself also becomes cheaper.

Final validation after the packed conversion:

- `test_dual_simplex`: 1038 assertions / 24 cases;
- `test_lp_solver`: 41 assertions / 6 cases;
- `test_numerical_stability`: 394 assertions / 15 cases;
- `test_netlib_regression`: 220 assertions / 2 cases;
- `test_milp_solver`: 131 assertions / 15 cases;
- `native_kernel_comparison --check`: 0 must-pass failures.

## 26. HFactor FTRAN data-structure analysis and implementation

The first HFactor experiment exposed a build-ownership defect: the executable
linked both embedded HiGHS and `mipsolvers_hfactor`, which define the same
global `HFactor` symbols. Archive order selected `highs/util/HFactor.cpp`; edits
to the nominal native copy were not executed. Configurations with embedded
HiGHS now use that single implementation. The standalone mirror is built only
when the full HiGHS library is unavailable, and the two source copies retain
the same indexed-solve changes.

### 26.1 Rejected FT dependency graph

The chronological FT loop originally visits every stored update and ETA entry.
An inverted row-to-update dependency graph measured genuine reach sparsity over
the 118-bus run:

- reached FT updates: 647,423 / 4,691,595 = 13.8%;
- reached ETA entries: 20,585,561 / 50,744,746 = 40.6%;
- dependency edges examined: 6,895,225.

Despite skipping work, a linked row graph plus binary heap increased the solve
from 7.44 s to about 8.0 s. FT application itself was only about 66 ms over
37,000 profiled FTRANs; pointer chasing and heap scheduling cost more than the
short chronological scan over at most 256 updates. The experiment also needed
a model-sized `ft_row_head`, contrary to the packed design. It has been removed
completely. A fused heap frontier for U was also rejected: it changed numerical
accumulation order, increased the run to 48,401 pivots and 14.6 s, and is not in
the final code.

### 26.2 Actual dominant operations

Internal timing showed that FT was not the FTRAN floor. Before the retained
changes, 37,000 FTRAN calls accumulated approximately:

| internal operation | time |
|---|---:|
| lower triangular reach/solve | 0.021 s |
| FT updates | 0.066 s |
| upper triangular reach/solve | 1.91 s |
| `HVector::reIndex` | 0.32 s |

The upper solve expands roughly 115 intermediate entries per call to several
thousand output entries. Its DFS symbolic reach and numerical scatter are the
remaining intrinsic HFactor cost. Only 144 / 37,000 calls selected HFactor's
full U scan, so changing the 5% hyper-sparse switch is not a material answer.

Two avoidable data-structure costs were removed:

1. Indexed FTRAN/BTRAN no longer calls `reIndex()`. Every indexed triangular
   and FT kernel already maintains its support; `reIndex()` discarded that
   fact and scanned all 35,599 rows whenever support exceeded 10%.
2. HFactor results no longer allocate `pair<int,double>` entries and comparison
   sort them. `IndexedVector` carries a flat, support-sized open-address lookup.
   The backend constructs it while exporting HFactor's support, so there is no
   second pass. Iteration remains over parallel packed index/value arrays and
   random `at(row)` lookup is expected O(1). No lookup storage or operation is
   proportional to `m` or `n`.

### 26.3 Measured result

The retained implementation solves `UC_118bus_24T-relax` in **5.54 s** with
36,545 pivots, objective `1.23363614e+05`, row violation about `1.1e-12`, and
bound violation about `2.6e-13`:

| phase | pivots | time | ms/pivot | FTRAN |
|---|---:|---:|---:|---:|
| Phase I | 19,854 | 2.189 s | 0.110 | 1.033 s |
| Phase II | 16,691 | 3.314 s | 0.199 | 1.487 s |

Compared with section 25, wall time fell from 7.44 s to 5.54 s (25.5%) and
profiled FTRAN fell from 4.378 s to 2.520 s (42.4%). The weighted total is about
0.151 ms/pivot and FTRAN is about 0.069 ms/pivot.

The requested 10% cost target is **not achieved**: current wall time is 74.5%
of the section-25 case, and FTRAN is 57.6% of its former cost. The next HFactor
step must reduce the upper triangular symbolic/numeric reach itself without
changing accumulation order. FT scheduling, another density threshold, or
another packed-output micro-optimization cannot provide the remaining factor.

Validation:

- `test_dual_simplex`: 1038 assertions / 24 cases;
- `native_kernel_comparison --check`: 0 must-pass failures;
- `git diff --check`: clean.

### 26.4 Order-preserving U-reach representation

The old U representation stores a physical row in `u_index[k]`. During every
symbolic DFS edge visit, `solveHyper()` translated it to a logical U node with

```cpp
u_pivot_lookup[u_index[k]]
```

The first load is sequential, but the second is an input-dependent access into
the row-to-pivot permutation. The arithmetic pass subsequently traverses the
same U entry range again in stored order. Changing the traversal, merging the
symbolic and arithmetic frontiers, or sorting the frontier is not admissible:
those changes alter DFS postorder and therefore floating-point accumulation
order.

HFactor now maintains a parallel packed symbolic adjacency:

```cpp
u_reach_index[k] == u_pivot_lookup[u_index[k]]
```

`u_reach_index` contains 32-bit logical node IDs even when `HighsInt` is 64
bit. It therefore costs four bytes per stored U entry, is proportional to U
storage rather than the model dimension, and turns the symbolic child access
into one sequential load. Models whose logical node count cannot fit in 32
bits retain the original lookup path.

The ordering proof is direct:

1. RHS roots are still consumed in the same packed order and translated by
   `u_pivot_lookup`.
2. Each node still scans exactly `[u_start[i], u_last_p[i])` in increasing
   physical entry order.
3. For every scanned entry, the cached child equals the child returned by the
   old double lookup.
4. Hence marks, stack pushes, DFS postorder, and reverse-postorder node order
   are identical.
5. The numeric pass is unchanged and still uses `u_index` and `u_value` in the
   original stored order.

The cache is built in `O(stored nnz(U))` during factorization and invert
restore. An FT deletion copies the cached ID with the same last-entry swap as
`u_index`; an FT append records the current lookup of the appended physical
row. This is valid because all incoming entries for the pivotal row are
deleted before that row's lookup moves to its newly appended logical pivot.
Unsupported structural update paths invalidate the cache and use the original
sparse lookup. There is no dense or model-dimension fallback.

Two consecutive 118-bus profiles after the compact representation measured
5.430 s and 5.419 s total, with FTRAN totals of 2.314 s and 2.306 s. Both runs
had 36,545 pivots, objective `1.23363614e+05`, row violation `1.14e-12`, and
bound violation `2.56e-13`. Relative to section 26.3, typical FTRAN time fell
from 2.520 s to about 2.31 s (8.3%), while total time fell from 5.54 s to about
5.42 s (2.2%). Relative to the section-25 baseline, total time is still about
72.8% and FTRAN about 52.8%. Thus this redesign removes the symbolic
row-to-pivot indirection without numerical drift, but the requested 10% total
cost target remains far away; the unchanged upper numeric scatter is now the
next HFactor-level cost to isolate.

### 26.5 FTRAN call topology and factor-age measurement

The first algorithm-level step was to classify the hot solves before trying to
reuse or remove them. `MIPSOLVERS_DS_PROFILE=1` now reports pivotal FTRAN and
basis-update BTRAN call counts, average RHS/result support, and FTRAN time and
result support in 32-update factor-age buckets. The counters reuse the existing
environment-gated primal profiler; they add no scan and execute no accounting
on the normal path.

The 118-bus cold solve contains two primal-simplex passes, not a dual minor
iteration trajectory:

| pass | pivots / pivotal FTRANs | average FTRAN RHS | average FTRAN result | update BTRANs |
|---|---:|---:|---:|---:|
| Phase I | 19,854 | 30.6 | 2,446.3 | 18,928 |
| Phase II | 16,691 | 30.7 | 4,861.9 | 16,675 |

Thus every one of the 36,545 hot FTRANs has a distinct entering column and is
the pivotal direction required by the ratio test and factor update. There is
no repeated BFRT/DSE FTRAN in this trajectory to eliminate algebraically.

Factor age does increase solve cost. In Phase II, age 0--31 accumulated 0.155 s
over 2,113 calls (73 microseconds/call), while age 224--255 accumulated 0.195 s
over 2,081 calls (94 microseconds/call). However, the average result support
only increased from 4,772 to 5,083 entries. A measured 128-update reinversion
experiment changed the degenerate pivot path, increasing the run from 36,545
pivots and about 5.42 s to 42,782 pivots and 8.83 s. It was removed and the
256-update policy restored.

A primal Devex experiment was also removed. It reduced Phase I to 10,704
pivots, but selected fill-heavy Phase-II columns; the run reached 41,058 total
pivots at the 20 s limit without finishing. This demonstrates that minimizing
pivot count or a local norm proxy alone is insufficient. Restricting Devex to
Phase I did not isolate the benefit: the different Phase-I exit basis caused
48,942 Phase-II pivots and 59,646 total pivots, taking 12.4 s. An exact attempt
to hand the Phase-I basis directly to dual simplex also found that the basis
could not be made dual feasible. Both experiments were removed. Any future
pricing or crash change must optimize Phase-II exit-basis quality together with
Phase-I progress and factor reach.

### 26.6 Step 2: sparse IPM-to-simplex basis handoff

The old recovery kept three candidates per row, greedily replaced at most 256
logicals, and repeatedly formed a growing `Eigen::MatrixXd`. `FullPivLU` proved
rank only for that dense corner, not for the complete simplex basis. The
mandatory native-MILP path then ignored the recovered basis and started from
the cold logical basis.

The replacement is sparse throughout:

1. IPM primal, row-dual, and bound-dual values are mapped to scaled standard
   form. Row duals now use the public simplex sign convention rather than the
   opposite legacy crash convention.
2. Basic likelihood uses scaled reduced cost. Interior columns have lexical
   priority; bound columns are eligible only when their reduced cost is
   numerically zero.
3. Each row contributes at most 12 strong structural edges to one bounded
   packed edge array. Scores combine optimal partition, normalized coefficient
   magnitude, row-dual activity, and column sparsity.
4. Retained edges are deduplicated into a candidate-column order: interior
   status first, then reduced-cost/basic score, coefficient quality, and
   sparsity. The valid logical or seeded basis is factorized before the first
   exchange; seeded structural positions are never eligible to leave.
5. Each sparse candidate column is packed directly from `A` and FTRANed by the
   current HFactor. The largest coefficient at a replaceable logical position
   is eligible only when its absolute magnitude is at least `1e-9` and it is at
   least 10% of the largest magnitude in the complete transformed direction.
   BTRAN of the selected unit position is then captured and the exchange is
   committed through `update_captured`. The published basis index changes only
   after that commit succeeds.
6. HFactor requests an early refactor when update fill requires it; otherwise
   the crash refactors after 128 accepted exchanges. One final complete sparse
   factorization is mandatory. There is no post-hoc rank repair: a failed final
   build rejects the recovered basis as a whole, and protected seed columns
   cannot be silently replaced.
7. `allow_warm_primal_phase_one` is off by default and enabled only for a
   screened IPM crossover basis. A dual-infeasible crash basis now enters the
   existing artificial-objective Phase I from that basis instead of being
   discarded at iteration zero for a cold logical restart.

The primal Phase-I driver also applies its one-reinvert confirmation policy to
non-finite pre-commit dual updates and impossible unbounded Phase-I ratio
results. No suspect pivot is committed; a repeated failure from the fresh
factor remains terminal.

On the presolved 118-bus root (`m=26,499`, `n=43,313`), the incremental version
retained 9,890 packed edges and admitted 1,184 structural columns. It performed
nine bounded sparse refactors, required zero rank repairs, and the accepted
relative pivots ranged from 0.117 to 1.0 with mean 0.597. The complete basis
factorized and entered warm Phase I. This is a stronger construction result
than the old batch basis (672 matches followed by 93 repairs): every published
exchange has now been admitted against the actual preceding numerical basis.

Crossover is still not complete. Warm Phase I reaches its existing 5,000-pivot
limit, after which the strict simplex-basis fallback remains necessary. Stable
incremental construction therefore removes the batch rank/conditioning bug but
does not prove that the selected optimal-partition columns give a short primal
Phase-I trajectory. A basis-free IPM return was remeasured and rejected because
downstream cold LP solves expanded the 20-second case to 68 seconds.

Step 2 therefore removes the dense rank test, fixes the dual-sign and warm-start
contract bugs, and now constructs the recovered basis by stable packed HFactor
updates. It does not yet meet the 10% target. The remaining boundary is Phase-I
trajectory quality: candidate admission controls linear independence and local
pivot stability, but not the number of primal pivots needed to reach a vertex.

---

# Progress log — rebuild/boundary round (2026-07-30, later)

## 27. Duplicate INVERT eliminated; 0.14 ms/pivot target met

### 27.1 Diagnosis — every rebuild factorized the basis twice

A `sample` profile of the §26.4 baseline (5.34 s) attributed ~21 % of wall time
to the INVERT path — twice what one build per rebuild should cost. The call
tree showed why: `HFactorBackend::factorize_with_logicals` ran a **complete
HFactor build twice per rebuild** — first the diagnostic build with HFactor's
implicit logicals (rank detection/repair), then, because the caller's logical
columns are Ruiz-scaled multiples of $e_i$, a second full build in the real
column space via `factorize()`. Every one of the ~142 rebuilds in the 118-bus
run is full rank (`rank_repairs = 0`), so the diagnostic build was pure
overhead, ~0.5 s of the run.

### 27.2 Fix — real-space build first, diagnostic build only on deficiency

`factorize_with_logicals` now tries the real-space `factorize()` on the
caller's basis first and returns immediately when it succeeds; the
implicit-logical diagnostic build and repair run only after a real-space build
reports deficiency (a rank-deficient input now costs one extra build — it is
rare and confined to crash bases). In the full-rank case the final factor
state is the same build the old path ended with, so the pivot trajectory is
**bit-identical**: 36,545 pivots, obj `1.23363614e+05`, rowviol `1.14e-12`,
bndviol `2.56e-13` before and after.

**Measured: 5.34 s → 4.61 s (−13.6 %).** This also speeds every node-LP and
warm-start factorization fleet-wide (39-bus/24T MILP row now ~13 ms, was ~35 ms
in §10).

### 27.3 Solve-boundary cleanups (kept; ≈1–3 % combined)

- `ftran_indexed`/`btran_indexed` built an open-address coordinate-lookup
  table over the full result support (~2.4k–4.9k entries) on **every** solve —
  ~130 M hash inserts per 118-bus run — while the primal loop queries the
  direction at exactly one row. The lookup is now built **lazily** by
  `IndexedVector::at()` (mutable `lookup_slot`; supports < 8 use a linear scan
  with no allocation), the backend no longer populates it, and the unsorted
  `lower_bound` hazard is gone because `at()` never binary-searches. The
  primal ratio test additionally captures the selected direction entry in
  `PrimalLeaving::direction_value`, so the hot loop performs no coordinate
  lookups at all.
- The separate `finite()` pass over every exported solve result was fused into
  the backend's export loop; a non-finite solve now rejects at the boundary
  (also closing a latent NaN-acceptance hole in the §26.6 crossover, which
  consumed backend solves without a finiteness check).

Individually these measured near the noise floor on this hardware (the §10.1
lesson again — the eliminated work was memory-cheap); together with the INVERT
fix the run lands at **4.47–4.53 s**.

### 27.4 Rejected: software prefetch in `solveHyper`

Prefetching the next node's pivot slot and entry range (distance 2, reverse
DFS postorder) in the numeric pass made the run **slower** (4.60–4.66 s vs
4.47–4.53 s): Apple-silicon's out-of-order window already hides the latency
and the hint instructions are pure overhead. Removed; `solveHyper` is
unchanged from §26.4.

### 27.5 Result vs the 10 % target, and the benchmark rows

`UC_118bus_24T-relax` (`m=35,599`, `n=62,786`), same 36,545-pivot trajectory:

| round | wall | weighted ms/pivot | Phase I | Phase II |
|---|---:|---:|---:|---:|
| §26.4 baseline | 5.34 s | 0.148 | 0.106 | 0.192 |
| this round | **4.49 s** | **0.123** | 0.089 | 0.168 |

The **0.14 ms/pivot reading of the 10 % target is met** (§24.1/§26.3 said it
required an HFactor-level change; the change turned out to be *not building
the factor twice*). Benchmark rows at 8 s: `NativeBC[natSimplex]` certified
bound improved **64.8k–67.6k → 80,516** (more pivots fit the budget);
`NativeBC[natIPMroot]` unchanged at the exact root bound 123,363.61.

Remaining profile: `solveHyper` ~38 % (upper-solve reach — intrinsic per
§26.2–26.4), single INVERT + reconstruct ~12 %, driver state
maintenance/ratio/PRICE the rest. The next material lever on this LP is not
per-pivot cost: it is the §26.6 crossover Phase-I trajectory (pivot count),
unchanged from that section's conclusion.

### 27.6 Validation

- `native_kernel_comparison --check` — **0 must-pass failures** (exact
  objectives), before and after each change in this round;
- `test_lp_solver` 41/41 · `test_numerical_stability` 394/394 ·
  `test_dual_simplex` 1052/1052 (also under
  `MIPSOLVERS_DS_PARANOID=1 MIPSOLVERS_DS_TIER_CHECKED_SOLVES=0`) ·
  `test_netlib_regression` 220/220 · `test_milp_solver` 131/131;
- 118-bus pivot trajectory bit-identical across the INVERT fix (36,545 pivots,
  identical objective and residuals), confirming no numerical drift.

Files touched this round: `hfactor_backend.cpp` (single-build fast path,
no eager lookup, fused finiteness), `native_dual/model.hpp` (lazy
`IndexedVector` lookup), `native_dual/factor.cpp` (redundant `finite()`
removed), `native_dual/primal.cpp` (`direction_value` capture).

---

# Progress log — bounded crossover and root re-solve removal (2026-07-31)

## 28.1 Explicit basis-free IPM handoff

The sparse recovered basis remained rank-valid but still exhausted its
5,000-pivot primal Phase-I limit. The failure then silently fell through to a
cold simplex and consumed the rest of the 8 s root budget even though IPM had
already converged to the exact `123363.614` bound.

`LPRelaxationResult` now carries an explicit `requires_ipm_nodes` transition.
After a bounded crossover failure, `solve_lp_relaxation` returns the audited
IPM point and bound; the B&C caller disables simplex-node dispatch and enables
IPM-node dispatch before any later LP solve. Large roots with at most 10 s of
LP budget bypass crossover up front, while longer runs retain a capped
1,000-pivot experiment. A failed experiment no longer launches cold simplex.

## 28.2 Redundant second root solve removed

The `2.104e11` warm start was rejected by the incumbent quality gate only at
publication time, after it had already been registered as an objective cutoff
and used for root-domain learning. Cutoff registration now applies the same
quality gate. On IEEE-118 this removes all 3,299 artificial root bound
tightenings and therefore the second full IPM solve.

For legitimate propagation tightenings, the root path now audits the existing
optimal point against every tightened bound. If it remains feasible, the
unchanged constraints/objective plus the retained dual bound certify the same
optimum and the result is reused. A violating point still takes the existing
re-solve path, with a refreshed remaining-time budget.

## 28.3 Measured result and validation

At the 8 s limit, native IPM now performs one root relaxation, retains the
exact `123363.614` bound, performs no crossover/cold-simplex cascade, and does
no propagation re-solve. Measured end-to-end rows are `7.77-7.96 s` (previously
the root LP alone consumed the 8 s budget); the remaining time is spent in
root heuristics and probing.

- `test_milp_solver`: 131/131 assertions.
- `native_kernel_comparison --check --time-limit 8`: 0 must-pass failures.
- `native_kernel_comparison --milp118-native-ipm --time-limit 8`: one root
  relaxation, exact root bound, total 7.77-7.96 s.

---

# Progress log - basis-free root branching and IPM deadlines (2026-07-31)

## 29.1 Remove basis-dependent work after the IPM handoff

Simplex diving is useful as a sequence of warm bound-change reoptimizations;
without `root.basis_hint`, each dive LP instead pays for a new Phase-I solve.
Large basis-free roots now skip that path. Under a short budget (at most 10 s),
they also skip standalone split-bound probing. Those probes produced no
fixings or bound lift on IEEE-118 and duplicated the same two domains needed
by the first real branch.

The root now enters the tree immediately. Reliability probing is disabled for
this short-budget case, and the down and up domains of the selected branch are
processed as the actual children. When longer-budget IPM reliability probes
are used, successful selected-direction results are retained and consumed by
child processing if domain closure has not changed their bounds. The usual
queue logic retains both valid solved children.

## 29.2 Deadline-aware IPM solves

`NativeIPMLPAdapter` now exposes a per-solve wall-clock limit that does not
invalidate its cached matrix structure. Root probes divide their local
remaining budget across unattempted directions. The first real down/up child
solves divide the global remaining budget fairly, and later node, fallback,
cut, proof, pool-separation, and repair IPM solves receive the current global
remainder. A child that reaches this deadline records `Time limit reached`
instead of allowing an empty frontier to be reported as search exhaustion.

Short-budget IPM roots also keep the remaining post-presolve budget instead of
reserving half of the original limit for a much larger unpresolved fallback.
The redundant background analytic-centre IPM is disabled in this mode. A 10%
(at most 1 s) finalization reserve prevents domain closure or a child solve
from being admitted when it cannot finish before the global deadline.

This follows two scheduling principles: speculative solves should be admitted
only when their information cannot be obtained from mandatory tree solves,
and every iterative solve needs a local deadline derived from the enclosing
phase deadline. It reduces redundant factorization/KKT work without changing
the branch-and-bound proof contract.

## 29.3 Measured result and remaining work

On IEEE-118 with an 8 s limit, root diving performs zero LP solves and
standalone probing is skipped. A clean build performs one root IPM, retains
the exact `123363.6142` root bound, and terminates correctly on its deadline
in 7.32 solver-seconds. The admission gate does not start the first branch in
this run because less than the finalization reserve remains. With a longer
budget, the direct branch path solves the actual down/up children, shares the
remaining deadline between them, and retains both valid results.

PaPILO's constant objective contribution from eliminated columns is now
preserved and added at result reporting. The reduced root objective is
`121383.6142`; the eliminated-column offset restores the original-space bound
`123363.6142`. This constant does not affect branch decisions inside the
reduced search.

- `test_milp_solver`: 131/131 assertions.
- `native_kernel_comparison --check --time-limit 8`: 0 must-pass failures.
- `native_kernel_comparison --milp118-native-ipm --time-limit 8`: 0 nodes,
  1 LP solve, 7.32 s, exact root bound, time-limit status.

Batched structure-aware IPM diving remains the next larger step. It should
group sibling/fixing domains that share the same constraint matrix, reuse
symbolic KKT analysis and scaling, and schedule the batch by predicted
information gain per remaining second rather than treating every fixing as an
independent solve.

---

# Progress log - structure-aware IPM batches (2026-07-31)

## 30.1 Bound-change batch contract

`NativeIPMLPAdapter` now accepts a batch of one-column bound changes over a
shared parent domain. The batch reuses one bounds scratch buffer and prepared
matrix structure, divides a single wall deadline over unattempted entries,
and reports whether each LP was actually started. Nonselected reliability
candidates keep only `(variable, direction, result)`; full `n`-column domains
are materialized only for the selected candidate.

The initial cached normal-equations experiment confirmed that structural
reuse alone is insufficient. At 15 s, six directions consumed 4.57 s and all
timed out. Capping admission to one down/up pair still consumed 4.53 s with no
convergence. The cached normal-equations kernel was the wrong linear system
for the dense SCUC coupling pattern.

## 30.2 Augmented-KKT structure selection

Dense-coupling prepared models now route node and batch solves through the
sparse augmented-KKT kernel used by the successful root IPM. Apple Accelerate
symbolic LDLT analysis for that augmented pattern is retained across solves;
numeric factorization remains per iteration because the barrier diagonal
changes. Narrow-band and sparse-normal models keep the existing cached normal
equations path.

With the same one-pair experiment, both directions converged in 2.95-3.03 s
total instead of timing out after 4.53 s. Raw reliability results can still
have a different domain after child closure; such results are never consumed
as child bounds, but their primal points are clamped and retained as warm
starts.

## 30.3 Sibling-first root expansion

Basis-free roots now skip IPM reliability probing at every time limit. The
selected pseudocost branch is moved forward, both actual child domains are
closed, and their primary augmented-KKT solves share the remaining deadline.
Optional proof, pool, cut, and repair re-solves for the first child are
deferred until the sibling has a primary bound. A pair-admission gate stops
before popping another node when the remaining budget is below 2.2 times the
measured root-IPM runtime, preserving the solved frontier.

On IEEE-118 with a 15 s limit, both first children now converge and remain in
the queue: the down bound is `135170.6017`, the up bound is `29526229.34`, and
the original-space frontier lower bound is `137150.6017` after PaPILO's 1980
objective offset. The run explores 1 node, performs 5 LP solves, and stops in
13.93 solver-seconds. Before sibling-first structure selection, the same run
spent 11 LP solves, retained no child, and stopped at the root bound.

The 8 s behavior is unchanged: 1 root LP, 0 nodes, exact original-space bound
`123363.6141`, and 7.22 solver-seconds.

- `test_numerical_stability`: 403/403 assertions, including the cached batch
  equivalence test.
- `test_milp_solver`: 131/131 assertions.
- `native_kernel_comparison --check --time-limit 8`: 0 must-pass failures.

# Structural LP kernel selector (dual-simplex vs interior-point)

Date: 2026-08-11. Scope: `NativeAutoLPAdapter` in
`src/engine/solver/native/native_lp_selector.cpp`. This is the derivation and
calibration record required by `AGENTS.md` (theory leads, references at the
site, quantitative prediction and a validation protocol fixed before results).

## 1. Motivation (measured)

On the 24-instance NETLIB set (macOS M4, Release, `build/macos-release`,
2026-08-11) no single native LP kernel dominates HiGHS-simplex:

- `Native-DualSimplex[ExactDSE]` (full-exact dual steepest edge) wins the
  small/medium instances and the geomean, but loses the large/dense ones.
- `Native-IPM[centrality-step,direct]` wins the large/dense/wide instances
  (`scsd8` 8.6x, `25fv47` 4.2x, `d2q06c` 3.8x) but loses the small ones.
- A per-instance oracle that picks the faster of the two reaches geomean
  **1.60x** vs HiGHS-simplex (min-over-repeats), versus 0.99x (ExactDSE only)
  and 0.90x (ipm-direct only).

The default top-level LP path was IPM-only (`default_priority_for(LP)` led with
`NativeIPMLP`), so it left the ExactDSE wins on the table. The selector recovers
them without changing correctness: both branches are audited kernels, so the
choice only affects wall-clock time.

## 2. Cost model

For an LP with `m` constraint rows, `n` structural columns, `nnz = nnz(A)+nnz(Aeq)`,
let the normal-equations symbolic factor of `M = A_a Θ A_aᵀ` have factor flops
`F` and fill `nnz(L)` (both reported by CHOLMOD's `analyze`, the same quantities
the IPM already uses for its normal-vs-augmented routing — see
`docs/netlib_benchmark.md` §5.2).

- **IPM.** Mehrotra predictor-corrector needs a near-constant iteration count
  `K₀` (independent of `m` up to a log factor), each dominated by one sparse
  `LDLᵀ`:  `T_ipm ≈ K₀ · F`.
- **Dual simplex.** `P ≈ κ·m` pivots (empirically `κ ≈ 1–3` on NETLIB), each
  costing one FTRAN + one BTRAN against the basis factor (`≈ 2·nnz(L)`) plus a
  PRICE scan over the nonbasic columns (`≈ nnz`):
  `T_splx ≈ κ·m · (2·nnz(L) + nnz)`.

The PRICE term is what makes the model aspect-ratio aware: wide LPs (`n ≫ m`,
large `nnz`) inflate `T_splx` and favor the IPM even when the factor is cheap;
tall/narrow LPs favor the simplex.

## 3. Decision rule and the constant

Simplex is predicted faster when `T_splx < T_ipm`:

    κ·m·(2·nnz(L) + nnz) < K₀·F
    ⟺  g := F / ( m · (2·nnz(L) + nnz) )  >  κ/K₀ =: c.

So the selector chooses the dual simplex iff `g > c`, else the IPM.

`c = κ/K₀`. With the NETLIB regime `κ ≈ 2` pivots/row and `K₀ ≈ 50` IPM
iterations (the presolve-free `ipm-direct` path runs more iterations than a
presolved one), `c ≈ 0.04`. This rule was implemented, measured, and then
superseded by the portfolio (§4–5); it is retained here as the analysis that
motivates racing both kernels.

### Mismatch that forced a re-derivation

The first model omitted PRICE and used `φ = F/nnz(L) > c·m`. It mispredicted the
narrow/tall instances (`agg`, `sc205`, `ship04s`) and the wide ones (`fit1d`,
`scsd1`, `scsd8`) — the classic aspect-ratio failure. Per the `AGENTS.md`
mismatch protocol the cost model was corrected (add the `nnz` PRICE term), which
is the `g` above.

## 4. What the structural selector achieved — and why it was not shipped

The rule of §3 was implemented and measured on 2026-08-11 (`build/macos-release`,
repeat 3). The offline calibration on cached timings looked promising (geomean
~1.10x at `c=0.04`), but the **realized in-process selector reached only 0.92x**
vs HiGHS-simplex — better than the prior IPM-only default (0.88x) but below
HiGHS and far below the ~1.5x per-instance oracle. The shortfall is a few
instances (`agg` 0.09x, `sc205` 0.08x, `ship04s`, `stocfor1`) whose runtime is
dominated by the actual pivot/iteration count, which the sparsity pattern does
not predict; each misroute costs 3–16x. This is a fundamental ceiling: a cheap
a-priori structural signal cannot recover the oracle because the winner is not a
function of the sparsity pattern alone. The structural estimator was removed.

## 5. Shipped design — concurrent portfolio

`NativeAutoLPAdapter` runs the dual-simplex-DSE kernel and the IPM
(`ipm-direct`) on separate threads and returns the first successful result,
realizing the per-instance `min(T_DSE, T_IPM)` directly instead of predicting
it. Both kernels are audited, so whichever wins is correct; the adapter
recomputes the user objective `c·x` on the dual-simplex path (that kernel
reports the internal minimize-sense value). The loser is detached and bounded by
the solve's time limit.

Measured on the 24 NETLIB instances (repeat 3, geomean of medians vs
HiGHS-simplex):

| default LP path | geomean vs HiGHS-simplex |
|---|---|
| prior (IPM-only) | 0.88x |
| ExactDSE-only | 0.92x |
| structural selector (§3–4) | 0.92x |
| **concurrent portfolio (shipped)** | **1.26x** |

All 72 solves accurate; the full 18-test suite is green with the portfolio as
the default `solve_lp()` path. The portfolio lands below the 1.5x oracle because
of thread-spawn overhead and losers that run to completion (the kernels have no
cooperative-cancel hook), at roughly 2x CPU.

## 6. Limitations and notes

- ~2x CPU per solve: both kernels run until one finishes; the loser is detached
  and finishes on its own (bounded by the time limit). A cooperative-cancel hook
  or a shared thread pool would cut the wasted work — a future optimization.
- Non-deterministic winner (timing-dependent), but not the answer: both kernels
  are audited to the same tolerances, so the returned objective/solution agree.
- MILP/SCUC/B&C are unaffected: they call `solve_lp_with_basis` /
  `solve_lp_from_sf` directly and never route through `default_priority_for(LP)`.
- The cost model in §2 remains the justification for the portfolio: it shows no
  single native kernel dominates, which is precisely why racing both wins.

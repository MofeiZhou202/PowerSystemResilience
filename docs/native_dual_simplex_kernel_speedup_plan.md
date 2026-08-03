# Native dual-simplex per-pivot kernel speedup — scoped plan (2026-08)

Profiling-grounded plan for closing the remaining per-pivot gap vs HiGHS on the
NETLIB LP suite. Written after establishing that the quick/targeted levers are
exhausted; the remaining work is a genuine multi-week effort.

## Where the time goes (ground truth)

macOS `sample`, native dual-simplex on `d2q06c` (m=1759, n=6423, ~8510 pivots,
45% dense pivot rows), flat top-of-stack self-time (higher = hotter):

| Rank | Symbol | Samples | Improvable? |
|---|---|---:|---|
| 1 | `choose_entering_bfrt` (ratio test) | 1668 | native, HARD (branchy compute) |
| 2 | `multiply_AT_indexed` (PRICE) | 578 | native, HARD (scatter/gather) |
| 3 | `solveHyper` / `HFactor::ftranU`/`btranU`/`ftranL`/`btranL` | ~700 combined | NO — vendored HiGHS `HFactor` |
| 4 | `run_phase` driver | 379 | native, some |
| 5 | `push_leaving_row` + `primal_infeasibility` (CHUZR heap) | 272 | native, some |
| 6 | `compute_dse_weights` (DSE) | 100 | native, inherent |

Phase buckets (`MIPSOLVERS_DS_PROFILE=1`, d2q06c): minor 1.33s of 1.39s wall;
entering(BFRT) 0.40, price 0.33, ftran 0.13, leaving 0.12, postcond 0.08,
OTHER 0.16, rcUpdate 0.06, dse 0.03, validBlock 0.00 (already tiered).
BFRT telemetry: candidates≈1052/pivot, sort 8e-5s, order 8e-3s — so the BFRT
cost is the **scan + per-column certified `dot_error_bound`**, NOT sorting.

## Key structural facts (do not re-discover)

1. **FTRAN/BTRAN are HiGHS's `HFactor`** (`HFactorBackend` wraps the vendored
   factorization; `ftran_indexed`/`btran_indexed` are hypersparse). Already
   optimal — do NOT attempt to rewrite them.
2. **Parallelism is a dead end for medium cases.** PRICE parallel_csc needs
   `nnz(A) >= 65536`; d2q06c is below it → serial. BFRT parallel needs
   `scan >= 8192`; d2q06c scan is 2857 → serial. Measured 1/2/4 threads on
   d2q06c/degen3: identical. The wins must come from serial-path efficiency.
3. **The gap is BOTH pivot count and per-pivot cost.** vs HiGHS on d2q06c:
   ~1.75x more pivots (E1 cost-shift start inflates it: 5618→9854) AND ~2.5x
   slower per pivot (certified-kernel `dot_error_bound`/postcond + kernel
   maturity). Closing it needs BOTH.
4. **Already done (prior sessions):** E1 cost-shifted dual start
   (0.721→0.785x), ratio-test error-bound short-circuit + scan fusion +
   workspace reuse, validBlock tiering. See
   `native_dual_simplex_optimization_2026-07-31.md`.
5. **A serial evaluate+merge fusion in `choose_entering_bfrt` was tried and
   is a WASH** — the intermediate array is cache-resident; the cost is compute,
   not memory. Do not retry.

## Prioritized levers (payoff / risk / effort)

### L1 — Pivot-count reduction: adaptive E1 fallback  [HIGH payoff, MED risk, MED effort]
d2q06c/degen2/degen3/pilot4 get MORE pivots with E1 (cost-shift start) than with
dual Phase I. A runtime, presolve-feature-derived selector (NOT case-name /
size / shift-count — those overfit per the prior doc) chooses E1 vs Phase I.
Candidate features: post-start dual-infeasibility mass, estimated cleanup
pivots (shift count / m ratio + reduced-cost spread), degeneracy proxy (zero
reduced-cost fraction). MUST validate out-of-sample (hold out ≥8 NETLIB cases
+ the SCUC suite) to avoid the overfitting the prior session warned about.
Potential: ~1.5–1.75x on the E1-loss cases.

### L2 — Certified-kernel per-pivot tiering  [LOW-MED payoff, HIGH risk, LOW effort]
Tier `postcond` (0.08) and the BFRT `dot_error_bound` certification to periodic
(every K pivots / on rebuild), like validBlock already is. ~5–8% on dense
cases only; ~0 on the flippable small cases; does NOT flip any case. Weakens
the numerical guarantee that exists for the hard SCUC/118-bus LPs. Only pursue
behind a paranoid-mode flag with `test_numerical_stability` as a hard gate.
NOT recommended as a standalone — poor risk/reward.

### L3 — Serial PRICE for dense pivot rows  [MED payoff, MED risk, MED effort]
For a dense `row_ep` (d2q06c 45% fill), the serial stamped-accumulator
`multiply_AT_indexed` scatters; a serial CSC column-dot (sequential, no scatter
conflicts, SIMD-friendly inner product) may be faster but is currently gated
behind the parallel `nnz>=65536` threshold. Add a serial-CSC path selected by a
density predicate (e.g. `4*row_ep.nnz >= m`). Validate bit-exactness of the
pivotal row (tiny-value truncation must match).

### L4 — SIMD the BFRT/PRICE inner loops  [HIGH payoff, HIGH effort, HARD]
The classification loop has data-dependent branches (basic/direction/phase/
sign); PRICE has gather/scatter. True SIMD needs a branch-free reformulation
(masked lanes) + AVX2/NEON gather. Large, platform-specific, uncertain payoff.
The realistic multi-week core of "deep kernel work".

## Validation gates (every lever)
- `test_dual_simplex` (full suite), `test_lp_solver`, `test_netlib_regression`,
  `test_numerical_stability`.
- `native_kernel_comparison --check` exact objectives on 6/39/118-bus SCUC.
- NETLIB 24-case × 3-repeat geomean must not regress accuracy (69/72) and must
  improve the DS geomean vs HiGHS (currently 0.76x).
- No performance claim without the fixed repeated cohort (audit rule).

## Recommendation
L1 (pivot-count reduction) is the highest-leverage, but requires disciplined
out-of-sample validation to avoid overfitting the 24-case set. L4 is the true
multi-week kernel effort. L2 is not worth its risk. Sequence: L1 → L3 → L4;
treat L2 as optional and gated.

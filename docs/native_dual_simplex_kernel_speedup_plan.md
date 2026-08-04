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
2. **Parallelism is a dead end for medium cases unless measured otherwise.**
   The former PRICE `nnz(A) >= 65536` dispatch rule was an unportable proxy.
   The replacement online model also failed the suite gate and was removed;
   production PRICE is CSR-only. BFRT still has its separate `scan >= 8192`
   rule. Prior 1/2/4-thread measurements on d2q06c/degen3 were identical, so
   serial efficiency remains central.
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
6. **DSE now consumes HFactor's dense backing directly on the captured
   pivotal-column support.** This removes the packed auxiliary export,
   `dense_rho` clear, and scatter with no selector. The fixed 24x3 A/B/A gate
   improved native geomean by about 0.6% with identical non-time run records.
7. **Structural DSE is exact when the assigned basis columns are
   singletons.** The identity \(w_i=\lVert B^{-T}e_i\rVert^2=1/d_i^2\)
   removes all initialization BTRANs without approximating the metric. On the
   fixed 24x3 cohort it reduced mean pivots by 22.52% and bracketed wall time
   by 6.61%. A separate fixed-binary production A/B/A retained the policy at
   6.42% lower wall time; nonstructural uncached bases fall back to Devex.
8. **The structural DSE proof is basis-local, not cold-start-local.** An
   uncached warm assigned-row-singleton basis now takes the same zero-BTRAN
   analytic path; a nonstructural hint still falls back to Devex.
9. **DSE update values reuse the pivotal-column row stream.** Removing the
   duplicated `{row,value}` transaction records saves at least 12 bytes per
   updated nonpivotal row with identical pivots. Its fleet wall effect is below
   stable resolution, so retention is based on strict stream dominance.

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
The stamped CSR accumulator scatters, while a CSC column dot streams the
matrix without stamp branches. Serial and persistent-pool CSC experimental
kernels establish the rounding envelope and a 25%-33% crossover on one
uniform synthetic matrix. Both fixed-density dispatch and threshold-free
online timing dispatch are rejected: the latter regressed the fixed cohort by
4.1%. HiGHS-style row-wise sparse-to-dense result handling driven by structural
output saturation was then tested and regressed the same cohort by 2.4%, due
to its O(n) materialization and export. E3/L3 is closed until a kernel design
strictly removes that dense traffic term; do not retune either switch.
Three threshold-free Class-P reductions also failed: BFRT candidate compaction
regressed `d2q06c` by 2.1% in A/B/A; PRICE output-index reuse regressed the
cohort by 4.1%; and a signed-zero accumulator that eliminated the stamp array
regressed it by 1.2%. A future design must reduce both traffic and the dynamic
instruction/dependency cost; byte count alone is no longer an adequate gate.

### L4 — SIMD the BFRT/PRICE inner loops  [HIGH payoff, HIGH effort, HARD]
The classification loop has data-dependent branches (basic/direction/phase/
sign); PRICE has gather/scatter. True SIMD needs a branch-free reformulation
(masked lanes) + AVX2/NEON gather. Large, platform-specific, uncertain payoff.
The realistic multi-week core of "deep kernel work".

The Phase-II cheap-prefilter kernel is proved and retained in production on the
target AArch64 toolchain. The independently audited two-lane benchmark body
uses mask algebra for all seven classifications, has 80 instructions per two
lanes, one loop branch, and no candidate-dependent branch. A compact 17-byte
result replaces the conservative 48-byte scalar evaluation record. On the
fixed synthetic distribution, exact runtime category counts plus audited
basic blocks prove 27.788% fewer dynamic machine instructions and the array
model proves 33.691% fewer bytes; five serial seven-sample runs give a 1.349x
median speedup. Production subsequently removed an unconsumed output flag, so
the 80-instruction count describes the conservative audited benchmark body,
not an exact disassembly count for the final inlined production body. This
replaces the earlier predicate-instruction proxy.

PRICE now emits an active-only BFRT position stream as an ownership contract,
not a threshold. The full pivotal row remains available to Devex and reduced-
cost commit; each active entry adds only its four-byte packed position and does
not duplicate the column/value. BFRT preserves scalar `dot_error_bound` for
exact-required flags and merges in original PRICE order without materializing
the former full evaluation array. The fixed A/B/A was 5.140 / 4.803 / 5.187 ms
native geomean, a 7.5% improvement against the bracketing-control geometric
mean, with bit-identical non-time records and 72/72 accuracy throughout. The
temporary measurement switch was removed; no selector remains.

### L4a — Cross-owner materialization removal  [LOW payoff, RETAINED]
The first retained instance is the DSE support-intersection extraction:
HFactor's FTRAN `HVector::array` is read directly on the captured direction
pattern instead of exporting and re-scattering `rho`. Future proposals in
this class must identify a producer-owned representation and prove that the
consumer needs only a subset or projection of it. Repacking the same data in
another container is not sufficient.

The analogous BFRT experiment is closed. Stamped (O(L+S\log S)), first-touch
(O(L+S)), and direct HFactor `HVector` collection all passed 72/72 but were
neutral or slower than the same 4.851 ms native control. Even deleting RHS
export and scatter did not move the fleet metric. Do not add a density/size
selector or retry a different accumulator layout; BFRT RHS materialization is
not an evidenced cohort bottleneck.

PRICE-owned exact-error accumulation is also closed. Accumulating an absolute
dot beside every signed PRICE term deleted later CSC reads only for
`needs_exact` columns, but added an RMW and dependency to every PRICE matrix
term. It regressed `d2q06c` by 4.9% in targeted B/A/B and changed 5,609 pivots
to 5,678. Do not retry this fusion unless a representation removes the extra
producer accumulator rather than merely moving consumer work upstream.

### L5 — Reinversion interval  [MEASURED, CLOSED]
The complete-minor estimator measures every indexed factor solve, fits exact
update-age means separately by dual phase, and charges the full scheduled
major rebuild to \(R\). Synthetic work confirms a strong near-linear age
trend, but its build/solve weights are not a machine wall-time calibration.
Three-repeat wall estimates on the target cases differ from the existing
interval by at most 1.8x, below the predeclared 2x gate. Keep
`max(50, min(200, m/4))`; never delay numerical-trouble or HFactor fill-advice
reinversions. Do not tune this clamp from individual cases.

## Validation gates (every lever)
- `test_dual_simplex` (full suite), `test_lp_solver`, `test_netlib_regression`,
  `test_numerical_stability`.
- `native_kernel_comparison --check` exact objectives on 6/39/118-bus SCUC.
- NETLIB 24-case × 3-repeat must remain 72/72 successful and accurate. The
  explicit-Devex three-way baseline is 4.658 ms Native versus 3.587 ms HiGHS
  (0.770x); structural exact DSE measured 4.090 ms (0.877x). The independent
  benchmark A/B/A was 4.677 / 4.341 / 4.621 ms. The later production
  fixed-binary A/B/A was 4.674 / 4.369 / 4.664 ms, a 6.42% win against the
  control geometric mean; all brackets were 72/72 accurate. The former 69/72
  geomean included `grow22` fail-fast time and is retired.
- No performance claim without the fixed repeated cohort (audit rule).

## Recommendation
Structural exact DSE validates L1's premise without a learned selector: exact
cold weights reduced mean pivots by 22.52% and bracketed fleet time by 6.61%.
The fixed-binary production gate retained it at 6.42% lower wall time. Its four
per-case pivot losses remain evidence, not a reason for a fitted fallback:
production selection is solely the exact singleton proof, with Devex outside
that boundary. L4 is the true multi-week kernel effort. L2 is not worth its
risk; L3 and L5 are closed by suite evidence. DSE extraction validates
producer-owned cross-stage reuse but has only low single-digit remaining
scope. The next kernel proposal must strictly reduce memory traffic or
instruction work before implementation, rather than introduce another
dispatch threshold.

### L1 exact-selection follow-up  [MEASURED, REJECTED]

A benchmark-only certified lazy CHUZR closed the remaining logical gap between
recursive heap weights and the exact selected-row BTRAN. It used a
backward-error-derived merit upper bound and no empirical threshold. On the
four DSE pivot-regression cases it selected 806/70/36/1061 pivots versus
Devex's 732/67/35/1041, while requiring 33,444 BTRAN on `grow22` and 125,306
on `stocfor2`. Thus exact selection neither restores the Devex path nor
amortizes its proof cost. Do not tighten this experiment with case, dimension,
density, or error thresholds. Keep it benchmark-only as a falsified theory
probe; do not run the 24x3 gate unless a new structural bound changes the
asymptotic number of solves.

### L4a CHUZR row-stream fusion  [MEASURED, REJECTED]

Following HiGHS' producer-owned infeasibility update suggested deleting the
temporary `changed_primal_rows` projection and refreshing each heap row while
committing `primal_changes`. This removes one four-byte write per changed row,
one later compact-index read, a second loop, and the redundant linear search
for the leaving row. It preserves row order, heap operations, and pivots and
uses no selector. Nevertheless, a fixed `d2q06c` 3-repeat A/B/A measured
1275.619 ms control, 1279.995 ms experiment, and 1278.427 ms control. The
experiment was 0.23% slower than the 1277.022 ms geometric mean of its controls
with 8,510 identical pivots in every run. The likely mechanism is that the old
four-byte projection is a compact, cache-resident consumer stream, whereas the
fused heap update extends the live dependency chain and revisits 16-byte
`pair<int,double>` records. The implementation was removed and the 24x3 gate
was not run. This is another concrete example that deleted byte count alone is
not a sufficient kernel theory.

### L4a redundant leaving-row search  [RETAINED, BELOW WALL RESOLUTION]

The rejected fusion contained one independently dominant subgraph. The
transaction builder unconditionally appends `leaving.row` to `primal_changes`,
and `changed_primal_rows` copies that complete stream in the same order.
Therefore the later `std::find(changed_rows, leaving.row)` always succeeds and
its fallback insertion is unreachable. Removing the `extra_row` argument and
this linear scan deletes dynamic instructions on every committed pivot without
changing either compact data layout or heap operation order.

The full B/A/B cohort measured Native geometric means of 4.612 / 4.716 / 4.685
ms while simultaneous HiGHS means were 3.512 / 3.580 / 3.573 ms. This is too
small and system-correlated for a wall-time speedup claim. All three brackets
were 72/72 accurate, and after deleting timing fields and case load time their
normalized JSON documents were byte-identical. The change is retained on the
static instruction-dominance proof, not on an empirical selector or claimed
fleet gain. Raw data are `reports/netlib_chuzr_{no_extra_scan_repeat3,
extra_scan_control_repeat3,no_extra_scan_repeat3_c}.{csv,json}`.

### L4b derived BFRT range stream  [RETAINED, BELOW WALL RESOLUTION]

The Phase-II prefilter used to write `upper-lower` as an eight-byte value for
every active lane and the ordered merge read it immediately. Since bounds own
this derived value and only positive lanes consume it, production now computes
the subtraction in the positive merge branch. This removes 16 temporary bytes
per active lane unconditionally; a nonpositive lane also avoids both bound
loads and the subtraction. No policy threshold, ordering change, or numerical
approximation is introduced.

The `d2q06c` path remained 5,609 pivots and the full gate was 72/72 accurate,
with 639.0 mean Native pivots. Native/HiGHS geometric means were 4.358/3.502 ms
(0.804x), indistinguishable from the previous 4.355/3.542 ms measurement.
Retain on traffic dominance only. Do not claim fleet speedup from this run.
Raw data are `reports/netlib_bfrt_deferred_range_full_repeat3.{csv,json}`.

### L4b prevalidated reduced-cost stream  [MEASURED, REJECTED]

Reusing `pivot_row.value` for factor-update-safe commit values removed the
second reduced-cost expression and made the profiled commit bucket nearly
zero. It nevertheless moved `d2q06c` from 5,609 to 5,592 pivots: producing the
committed double in the validation loop changed its floating contraction and
rounding context. Accuracy alone is insufficient for a kernel-only rewrite;
the experiment violated the discrete-path contract and was removed.

### L4b minimal BFRT candidate state  [RETAINED, BELOW WALL RESOLUTION]

For every admitted candidate, `alpha=side*move_sign*pivot>0`; hence
`pivot=side*move_sign*alpha` is an exact sign recovery. Margin is fully consumed
when breakpoint is formed. Removing both redundant doubles shrinks the Apple
AArch64 record from 56 to 40 bytes and removes a candidate-only pivotal-row
read in the Phase-II merge. The scalar/Phase-I evaluation record also no longer
copies pivot.

This is not the earlier rejected candidate/taboo compound layout: taboo lookup
is unchanged. The `d2q06c` path stayed at 5,609 pivots, positive and negative
pivot recovery tests are exact, and normalized full reports are identical.
The 24x3 gate was 72/72 accurate with Native/HiGHS geometric means of
4.359/3.568 ms (0.819x). Retain on sufficient-statistic and traffic dominance;
do not claim a fleet wall-time win. Raw data are
`reports/netlib_bfrt_minimal_candidate_full_repeat3.{csv,json}`.

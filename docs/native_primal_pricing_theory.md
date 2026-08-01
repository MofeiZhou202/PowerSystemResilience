# Native primal cleanup pricing: theory and numerical contract

## 1. Scope and acceptance criterion

This document defines the mathematical and numerical contract for the primal
simplex cleanup used after Native Dual Simplex restores the original objective.
It is deliberately independent of benchmark-specific thresholds.

The implementation is accepted in four stages:

1. the formulas below are tested against explicit basis inverses on small LPs;
2. shadow pricing records what Devex/PSE would choose without changing pivots;
3. weighted pricing is enabled only with the pivot-stability contract;
4. difficult NETLIB cases and then the complete repeated suite pass the same
   original-space audit as the baseline.

Performance is not a correctness condition. The project-wide speed target is
reached only by 72/72 accurate runs over 24 NETLIB cases and geometric speedup
greater than 1.0 over one-thread HiGHS Simplex in the repeated benchmark.

## 2. Native standard form and signs

Native solves the bounded equality-form maximization problem

\[
    \max_x c^T x + c_0,
    \qquad Ax=b,
    \qquad l\le x\le u.
\]

The public LP is minimization, but standard-form construction stores its
negated, scaled objective in `StandardFormLP::c_max`. All derivations in this
document use the internal maximization convention.

Let `B=A[:,basis]`, let `N` denote nonbasic columns, and put each nonbasic at a
finite active bound. Native stores its allowed direction as

\[
 s_j=\operatorname{sign}(move_j)\in\{-1,0,+1\},
\]

where `+1` moves upward from the lower bound, `-1` moves downward from the
upper bound, and zero is basic or fixed. With

\[
 B^T y=c_B,\qquad r=c-A^T y,
\]

moving entering variable `q` by `Delta x_q=s_q theta`, `theta>=0`, gives

\[
 h_q=B^{-1}a_q,
 \qquad x_B(\theta)=x_B-s_q h_q\theta,
 \qquad \Delta(c^Tx)=s_qr_q\theta.
\]

Thus primal eligibility and gain are exactly

\[
 g_q=s_qr_q>\tau_d.
\]

This confirms the current Native expression `move * reduced_cost`. A pricing
rule may rank eligible columns, but must never change this eligibility test.

Free nonbasics require two possible directions rather than one stored bound
move. Native standard-form cold cleanup currently has finite lower bounds, so a
future free-variable implementation must represent the two directions
explicitly; treating a free column as lower-only is invalid.

## 3. Harris two-pass primal ratio test

For candidate `q`, define the basic change per nonnegative step

\[
 d_i=-s_q h_{iq}.
\]

For `d_i>0`, an upper bound limits the step; for `d_i<0`, a lower bound limits
it. A boxed entering variable also supplies its own range limit.

Pass 1 computes a relaxed maximum step using primal tolerance `tau_p`:

\[
 \bar\theta=\min\left\{
 \frac{u_i-x_i+\tau_p}{d_i}:d_i>0, u_i<+\infty;
 \frac{x_i-l_i+\tau_p}{-d_i}:d_i<0, l_i>-\infty;
 u_q-l_q+\tau_p
 \right\}.
\]

Only algebraically usable direction entries participate. Pass 2 considers
tight steps no greater than `bar theta` and chooses the most stable pivot, not
the smallest exact ratio. The exact step of the selected row is then used for
the state update. If the boxed entering range is strictly smaller, this is a
bound flip rather than a basis exchange.

An entry is algebraically usable only when it passes Section 7. Ignoring tiny
entries before both Harris passes is necessary: allowing one to set the relaxed
breakpoint and rejecting it only in pass 2 can falsely report no leaving row.

## 4. Exact primal steepest-edge geometry

For a nonbasic column `j`, the full-space feasible edge is

\[
 d_j=e_j-E_BB^{-1}a_j.
\]

Writing `h_j=B^{-1}a_j`, its squared Euclidean length is

\[
 w_j=\lVert d_j\rVert_2^2=1+\lVert h_j\rVert_2^2.
\]

Primal steepest-edge pricing maximizes

\[
 \frac{g_j^2}{w_j},
\]

or equivalently `g_j/sqrt(w_j)`. Squaring avoids a square root and preserves
the ordering because gains and weights are positive.

### 4.1 Goldfarb-Reid basis-exchange recurrence

Let `q` enter and old basic position `p` leave. Put

\[
 h=h_q=B^{-1}a_q,\qquad \alpha=h_p\ne0.
\]

Then `B'=BE`, where column `p` of `E` is `h`. For an old nonbasic `j != q`,

\[
 \lambda_j=\frac{(h_j)_p}{\alpha}
           =\frac{\hat a_{pj}}{\hat a_{pq}},
\]

and

\[
 h'_j=B'^{-1}a_j=h_j-\lambda_jh+\lambda_je_p.
\]

Define

\[
 z=B^{-T}h,\qquad
 \mu_j=h^Th_j=z^Ta_j.
\]

Expanding `1+||h'_j||^2` gives the exact update

\[
 w'_j=w_j-2\lambda_j\mu_j
       +\lambda_j^2\lVert h\rVert_2^2+\lambda_j^2.
\]

The final `lambda^2` is essential: the transformed pivotal component is
`lambda`, although `h_j-lambda h` has a zero pivotal component. Omitting it
systematically underestimates weights.

The old basic variable is a special case and must not use its stored basic
weight. Directly transforming `e_p` gives

\[
 w'_{B_p}=\frac{1+\lVert h\rVert_2^2}{\alpha^2}.
\]

The entering variable becomes basic, so its nonbasic weight is inactive.

Exact PSE therefore needs the pivotal FTRAN `h`, one additional BTRAN
`z=B^{-T}h`, a PRICE of `z^TA`, and transactional updates to every affected
nonbasic weight. It is not the first implementation stage because that extra
solve and PRICE must earn their runtime cost.

## 5. Devex reference framework

Devex approximates steepest-edge geometry with a fixed reference set `R`.
At framework initialization, `R` is exactly the set of currently nonbasic
variables. Define `delta_j=1` when `j in R` and zero otherwise. In a later
basis, the exact squared norm of edge `j` projected onto reference coordinates
is

\[
 \tilde w_j=\delta_j+
   \sum_i \delta_{B_i}(h_j)_i^2.
\]

For the entering column, compute from the current basis before exchange

\[
 \omega_q=\delta_q+\sum_i\delta_{B_i}h_i^2,
 \qquad \bar\omega_q=\frac{\omega_q}{\alpha^2}.
\]

Using tableau-row entry `rho_j=(B^{-T}e_p)^Ta_j`, the conservative Devex
recurrence is

\[
 w'_j=\max\left(w_j,\;\delta_j+\bar\omega_q\rho_j^2\right),
\]

with pivot special cases

\[
 w'_{B_p}=\max(1,\bar\omega_q),\qquad w'_q=1.
\]

The max makes the approximation monotone within a framework and avoids weight
collapse from cancellation. It is an approximation, not the PSE recurrence;
mixing PSE cross terms with Devex reference weights has no valid derivation.

The framework is restarted when repeated entering-column checks show that the
stored approximation exceeds three times the freshly computed reference norm.
Following the standard HiGHS Devex safeguard, four such observations end the
framework; these constants control approximation quality and never admit a
numerically rejected pivot. A
restart means atomically: choose the current nonbasics as a new `R`, set all
weights to one, clear the mismatch counter, and rebuild the pricing heap.
Reinversion alone does not change `R` or the mathematical Devex weights; if
numerical drift invalidates them, restart explicitly rather than silently
partially reinitializing state.

## 6. Pricing heap contract

The heap key is `gain^2/weight`. Its deterministic tie-break is column index.
The heap contains exactly the currently nonbasic, movable columns satisfying
the unweighted eligibility condition `gain > optimality_tol`.

After a pivot, reduced costs change only on the priced tableau-row support.
Devex weights also change only on that support, plus the entering and leaving
columns. Every such column must be refreshed after both reduced-cost and weight
updates. Refreshing before the weight transaction commits leaves stale keys.

Dantzig (`weight=1`) remains a policy fallback only. It cannot be used to
bypass a failed solve, unstable pivot, infeasible ratio test, or failed audit.
Changing pricing never changes the correctness predicates.

## 7. Numerical pivot admission

### 7.1 Backward error of pivotal solves

For computed `h` and `z_p=B^{-T}e_p`, define residuals

\[
 r_c=a_q-Bh,\qquad r_r=e_p-B^Tz_p.
\]

The normwise backward tests are

\[
 \lVert r_c\rVert_\infty\le
 C u\max(1,\lVert B\rVert_\infty\lVert h\rVert_\infty+
                 \lVert a_q\rVert_\infty),
\]

and the analogous transpose expression for `z_p`. Here `u` is machine
epsilon and `C` accounts for sparse arithmetic. Native already uses this form
for dense checked solves; packed pivotal solves must provide equivalent
evidence or be cross-checked before factor update.

### 7.2 Row/column pivot identity

The same pivot is independently available as

\[
 \alpha_c=e_p^Th,\qquad \alpha_r=z_p^Ta_q.
\]

Their discrepancy satisfies, up to dot-product rounding,

\[
 |\alpha_r-\alpha_c|
 \le |z_p|^T|r_c|+|r_r|^T|h|+\gamma_k S.
\]

Native's existing `PivotEvidence` computes this residual envelope in long
double. A pivot is inadmissible when the discrepancy exceeds the envelope.
This cross-check must occur before the irreversible FT update.

### 7.3 Relative pivot and update amplification

The exchange matrix inverse contains divisions by `alpha`; its amplification
is governed by

\[
 \rho=\frac{|\alpha|}{\max(1,\lVert h\rVert_\infty)}.
\]

An error-aware acceptance floor is

\[
 |\alpha| > 8E_\alpha,
 \qquad
 \rho > \sqrt{u},
\]

where `E_alpha` is the pivot-identity residual envelope plus arithmetic guard.
The first condition says the pivot is resolved above its measured uncertainty;
the second prevents one update from amplifying roundoff beyond the square-root
precision budget. Both are scale-relative and derive from arithmetic precision,
not a NETLIB-tuned absolute pivot constant.

Harris pass 2 ranks only candidates satisfying these conditions. If no stable
candidate remains but the factor has updates, INVERT and repeat the iteration.
If a fresh factor still has no stable candidate, report numerical failure or a
separately certified unbounded result; do not force the largest rejected pivot.

## 8. Factor update and transaction protocol

A pivot has prepare, validate, factor-commit, and state-commit phases.

Prepare, without mutating solver state:

1. compute and validate pivotal FTRAN `h`;
2. run Harris and choose `p`;
3. compute and validate `z_p` and tableau row;
4. verify row/column pivot identity and relative pivot;
5. compute prospective primal values, reduced costs, objective, moves, and
   Devex/PSE weights in scratch storage;
6. verify all prospective values are finite and all primal/dual local
   postconditions hold.

Then call the captured Forrest-Tomlin update. Only if it succeeds, commit the
basis membership, moves, primal values, reduced costs, objective, weights, and
heap keys. No failure path may leave only weights or only basis membership
updated. If a captured solve was refined, its stale FT packs must not be used;
exchange the logical basis state and immediately INVERT the new basis instead.

## 9. Reinversion contract

INVERT reconstructs `x_B`, reduced costs, objective, and basis membership from
the current basis. It is triggered by at least one of:

- factor fill/update-limit advice;
- failed backward-error or pivot-identity evidence after any existing update;
- no stable Harris candidate under an updated factor;
- excessive primal, dual, or objective drift;
- a refined pivotal solve whose update capture is invalid.

For exact PSE, reinversion must recompute all active nonbasic weights from the
new factor, or prove the recurrence state remains within a specified audit
error. For Devex, reinversion preserves the reference set and weights because
the exact basis exchange sequence has not changed; a Devex mismatch restart is
a separate atomic operation. Heap keys are always rebuilt after INVERT because
reconstructed reduced costs can change.

## 10. Presolve/postsolve publication contract

Solving a reduced LP is only an internal optimization. A successful result is
publishable only after HiGHS postsolve reconstructs the original primal and the
original LP passes a sentinel-aware audit of:

- every row lower and upper bound;
- every equality residual;
- every variable bound;
- finite original objective recomputed as `c^T x`.

A failed reduced solve or failed postsolve audit may fall through to one direct
solve, but benchmark telemetry must distinguish this double-solve path. A
primal-only postsolve cannot publish an original-space basis or dual
certificate. Any future publication of those objects requires a basis/dual
postsolve and an original-space dual audit.

## 11. Complexity and expected payoff

Let `F` and `T` be pivotal FTRAN/BTRAN cost, `P` tableau PRICE cost, and `k` the
tableau-row support.

- Dantzig iteration: `F + T + P + O(k)` with often high pivot count.
- Devex iteration: the same asymptotic operations plus `O(nnz(h)+k)` arithmetic
  and heap refresh; no extra solve.
- PSE iteration: `F + 2T + 2P + O(k)` in the straightforward formulation,
  unless its `z^TA` work can be fused or kept hyper-sparse.

Devex is therefore the first theory-guided upgrade. It is successful only if
the reduction in cleanup pivots exceeds weight-maintenance overhead without
increasing reinversions or audit failures. Exact PSE becomes justified only
after shadow telemetry demonstrates enough additional pivot reduction to pay
for its extra BTRAN/PRICE and after its recurrence passes exact recomputation
tests over long update chains.

## 12. Required invariant tests

Before weighted pricing is enabled, tests must cover:

1. gain/move signs for lower, upper, boxed, fixed, and (when supported) free
   columns;
2. Harris relaxed/tight passes and boxed bound flips;
3. PSE initialization `1+||B^{-1}a_j||^2`;
4. Goldfarb-Reid recurrence versus fresh inversion after each pivot, including
   the leaving-column special case;
5. Devex reference norm and recurrence versus direct projected norms;
6. entering/leaving special weights and atomic framework restart;
7. row/column pivot discrepancy rejection and relative-pivot rejection;
8. reinversion preserving Devex state and recomputing exact PSE state;
9. no heap-key staleness after reduced-cost and weight changes;
10. original-space postsolve audit rejection on an intentionally corrupted
    reconstructed primal.

These are algorithmic tests. Benchmark wins are evidence about performance,
not substitutes for the invariants.

## 13. Implemented result

The first implementation stage uses Devex by default. It includes atomic
weight transactions, framework restarts, the relative pivot floor, and the
row/column pivot identity check. Shadow mode showed that Devex and Dantzig
disagreed on 89.1% of `d2q06c` cleanup choices and 94.7% of `degen3` choices,
confirming that the change is algorithmic rather than a tie-break adjustment.

Exact PSE is also implemented behind `MIPSOLVERS_PRIMAL_DEVEX=pse`. Its
initial weights and consecutive Goldfarb-Reid updates are checked against
explicit basis inverses, and INVERT recomputes every active nonbasic weight.
It remains experimental because full-suite runtime does not repay the extra
initialization, BTRAN, and PRICE work.

The release-build 24-case by three-repeat result is 72/72 accurate for both
Native and HiGHS. Native records 3.025 ms median and 4.908 ms geometric mean,
versus 3.195 ms and 3.731 ms for HiGHS. Thus this stage improves Native but does
not satisfy the greater-than-1.0 geometric speed target.

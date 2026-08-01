# Native Dual Simplex optimization (2026-07-31)

## 2026-08-01: E1 cost-shifted dual start

### Theory contract

For a fixed cold basis, a nonbasic column with a finite upper bound can select
the endpoint whose move direction satisfies the reduced-cost sign condition.
If a lower-only nonbasic has positive reduced cost in the internal maximization
convention, no endpoint change can make it dual feasible; applying the working
cost shift `-reduced_cost` makes its reduced cost exactly zero. Performing
these two operations for every nonbasic produces a dual-feasible working-cost
state. The shift count is bounded by the number of initially dual-infeasible
lower-only nonbasics. There is deliberately no claimed bound on subsequent
original-cost cleanup pivots.

The production cold-start policy now tries this construction after exact
original-bound endpoint normalization fails. A successful, audited shifted
start enters dual Phase II directly. Failure to construct or audit it retains
the existing Dual Phase I path as a fallback. `MIPSOLVERS_DUAL_SHIFT_START=off`
selects that fallback for diagnosis and A/B measurement; it is not the
production default. Original costs are restored by the existing mandatory
cleanup transaction before an optimum is published.

### Implementation and telemetry

- `initialize_cost_shifted_dual_start` is now on the production cold path;
- shift accounting is committed only after reconstruction and the dual-start
  audit succeed;
- `Statistics::dual_start_cost_shifts` separates E1's initial shift bill from
  BFRT working-cost shifts, and both verbose summaries publish the counter;
- the production contract test verifies that E1 bypasses Dual Phase I,
  performs the mandatory original-cost cleanup, and returns the same audited
  objective as HiGHS;
- the explicit Dual Phase-I pivot-budget test runs with E1 diagnostically
  disabled, preserving coverage of the fallback state machine.

The follow-up diagnosis exposed missing phase accounting on paths that leave
the dual solver and run a full primal fallback. `Statistics` and
`MIPSOLVERS_DS_PROFILE` now report primal Phase I and primal Phase II
iterations and time separately. The terminal accounting contract is
`dualI + dualII + primalI + primalII + primalCleanup == iterations`.

This closes the formerly unexplained `grow22` path with E1 disabled: 66 Dual
Phase-I pivots and 660 Dual Phase-II pivots are followed by 357 primal Phase-I
and 529 primal Phase-II pivots after a canonical residual reconstruction
failure, totaling exactly 1,612. It also closes the planned `grow7` BFRT
diagnosis: E1 changes 485 pivots to 179 working-cost dual plus 173
original-cost cleanup pivots, totaling 352 versus HiGHS's 353. No ratio-test
change is justified by that case. The phase-sum invariant holds on all 24
NETLIB cases both with E1 enabled and with the legacy path selected (48/48
diagnostic solves).

### Validation and retained decision

The complete native dual-simplex suite passes 1,166/1,166 assertions across
39 test cases. The direct NETLIB regression passes 220/220 assertions. The
formal 24-case, three-repeat A/B result is:

| Cold-start policy | Accurate | Median (ms) | Geomean (ms) | Speed vs run-local HiGHS |
|---|---:|---:|---:|---:|
| E1 shifted start, production default | 72/72 | 3.575 | 4.395 | 0.785x |
| Dual Phase I (`MIPSOLVERS_DUAL_SHIFT_START=off`) | 72/72 | 4.427 | 4.840 | 0.718x |

E1 improves Native's A/B geometric mean by 9.2%. Relative to the previous
formal retained result (4.757 ms, 0.721x), the new production result is 7.6%
lower in geometric mean and advances the HiGHS ratio to 0.785x. Important
path changes include `fit1p` 859 -> 541 pivots, `grow7` 485 -> 352, and
`stocfor1` 134 -> 93.

The improvement is not uniform. E1 increases `d2q06c` 5,618 -> 9,854 pivots,
`degen2` 604 -> 902, `degen3` 2,082 -> 3,303, and `pilot4` 517 -> 1,049.
It is nevertheless retained because the declared suite-wide geometric target
moves materially and all accuracy gates pass. No case-name, size-only, or
shift-count-only selector is introduced: the observed wins and losses are not
separated by any theoretically justified monotone predicate. A future hybrid
policy must be derived from pre-solve state features and validated outside the
24-case training set rather than fitted to these instance names.

Raw A/B results are
`reports/netlib_e1_shift_start_repeat3.json` and
`reports/netlib_e1_phase_one_repeat3.json`.

## 2026-08-01: ratio-test error-bound short-circuit, scan fusion, workspace reuse, PRICE noise truncation

### Profile-driven motivation

`MIPSOLVERS_DS_PROFILE` and macOS `sample` on `d2q06c` attributed the largest
per-pivot buckets to the BFRT ratio test (35% of minor time), PRICE (12%), and
the HFactor triangular solves (17%). Inside the ratio test, the dominating
cost was `dot_error_bound` — a column-wise traversal of `A` evaluated for
every nonbasic movable support entry, including entries with a nonpositive
signed pivot, which can never satisfy the certificate `signed_alpha > error`
and whose only remaining effect is the stability flag when the error envelope
reaches `-signed_alpha`.

### Retained change 1: dominated error-bound short-circuit (path-identical)

`State::bfrt_error_coef[j] = 2 * (gamma_nnz(j) + 256*eps) * ||A_j||_1` is
filled once per solve. For any pivotal row, `coef_j * max|row_ep|` provably
dominates the value `dot_error_bound` computes: the analytic bound is
`(gamma + 256 eps) * fl(sum |row_ep_i A_ij|)`, whose accumulated rounding is
below the factor 2 for any support size far under `1/eps`. Since rounding to
nearest is monotone, `fl(signed_alpha + cheap) <= 0` proves
`fl(signed_alpha + error) <= 0`, so the branch outcome — flag, counters, and
control flow — is decided without the column traversal. Nonpositive pivots
skip the traversal when the proof applies (both dual phases); Phase II
positive candidates whose pivot dominates the cheap bound also skip it, which
is valid because `transaction.stable_capacity_error` is consumed only on
dual Phase I paths, and dual Phase I retains the exact evaluation.

### Retained change 2: fused flip/shift and step-interval scans (bit-exact)

The two post-selection traversals of the pivotal-row support were fused into
one pass. A column's step-interval terms depend only on that column's own
flip/shift marks, which are final once its flip/shift determination has run,
and the reductions (`step_lower`/`step_upper`) process columns in the same
order with the same expressions, so no computed value or its order changes.

### Retained change 3: allocation and copy removal (bit-exact)

The priced row is written into a thread-local `IndexedVector` reused across
pivots; the CHUZR BTRAN solution is moved (not copied) into
`leaving.row_ep`; the BFRT FTRAN image is moved into `bfrt_delta`; the CHUZR
unit vector and deferred-heap scratch are thread-local; the empty taboo table
skips the keyed `is_taboo_change` lookup; the BFRT dense `row_ep` scatter is
built lazily on the first exact bound evaluation.

### Retained change 4: PRICE sub-tiny truncation (algorithmic, HiGHS parity)

PRICE now drops accumulated products with `|value| <= 1e-14` from the packed
result, matching HiGHS `HVector::tight` (`kHighsTiny`). Non-finite values are
still exported so the caller's finiteness check keeps rejecting them.
On `d2q06c` this removes 18% of the priced-row support (3,168 -> 2,588
average nonzeros) as pure cancellation noise. Reduced costs of dropped
columns are reconstructed at every INVERT, bounding the drift. The keep-all
behaviour remains selectable with `MIPSOLVERS_PRICE_TIGHT=off`.

### Rejected experiment: Devex dual pricing for cold starts

`MIPSOLVERS_DUAL_PRICING=devex` (new diagnostic selection, kept) replaces the
exact DSE initialization with a Devex framework on cold starts. The complete
24x3 run remained 72/72 accurate but increased pivots almost everywhere
(`d2q06c` 5,618 -> 8,519, `scsd8` 1,027 -> 1,340, geomean 4.963 -> 5.262 ms),
so exact DSE remains the production policy at every model size.

### Validation

- 1,160/1,160 mathematical assertions and 220/220 NETLIB regression
  assertions pass;
- differential path contract: `d2q06c` retains 5,609 dual pivots, 31
  rebuilds, 1,423/4,186 Phase I/II iterations, 9 cleanup pivots, and
  bit-identical BFRT telemetry (957.6 mean candidates, 612,771 prefiltered);
  `scsd8`, `degen2`, `degen3`, `25fv47`, and `pilot4` likewise retain their
  exact pivot paths. Under the PRICE truncation only `stocfor2` changes path
  (988 -> 1,054 iterations, still accurate and net faster); every other case
  keeps its pivot sequence with strictly less work per pivot.

The formal 24-case, three-repeat comparison (same machine, Release, one
thread, 30 s limit):

| Path | Accurate runs | Median (ms) | Geomean (ms) | Geometric speed ratio |
|---|---:|---:|---:|---:|
| HiGHS Simplex | 72/72 | 3.035 | 3.430 | 1.000x |
| Native, this session end | 72/72 | 4.430 | 4.757 | 0.721x |
| Native, session start | 72/72 | 4.447 | 5.017 | 0.677x |

Raw results: `reports/netlib_bfrt_shortcircuit_repeat3.{csv,json}` (session
start), `reports/netlib_bfrt_workspace_repeat3.{csv,json}` (after changes
1-3), `reports/netlib_price_tight_repeat3.{csv,json}` (final). Case-level
gains concentrate where the support is widest: `scsd8` 47.0 -> 39.4 ms,
`d2q06c` 799 -> 759 ms, `degen3` 155 -> 144 ms.

### Remaining gap after this session

The per-pivot buckets are now flat (ratio test 0.16 s, PRICE 0.12 s, solves
~0.25 s, DSE 0.10 s, commit ~0.19 s of 0.70 s minor time on `d2q06c`).
The dominant remaining deficits versus HiGHS are (a) iteration counts on
small degenerate models (`lotfi` 285 vs 103, `sc205` 230 vs 88, `stocfor1`
134 vs 65), which point at cost perturbation / degeneracy policy rather than
kernel cost, and (b) a residual ~1.3-1.5x per-iteration factor on the dense
NETLIB tail. The 1.0x geometric target remains open.

## 2026-08-01: validated reduced-cost transaction

### Motivation

After the BFRT ordering and packed-vector changes, `d2q06c` telemetry still
showed two traversals of the pivotal-row support around every basis update.
The first traversal formed the analytical BFRT postcondition, while the second
traversal recomputed the same reduced-cost expression during commit. This is
not an algorithmic reason for a second evaluation: the first value is already
the value whose finiteness and dual-feasibility sign are certified.

### Derivation

For pivotal-row support column `j`, let

```
theta = leaving_side * entering_theta
s_j   = the accepted working-cost shift, or zero
r'_j  = fl(r_j + fl(theta * a_pj) + s_j).
```

The pre-update basis classification partitions the support into four disjoint
sets:

1. the entering column, whose post-pivot reduced cost is exactly zero;
2. basic columns other than the leaving column, which remain basic and whose
   post-pivot reduced cost is exactly zero;
3. the leaving column, which becomes nonbasic and receives `r'_j`;
4. all other nonbasic columns, which retain nonbasic status and receive
   `r'_j`.

This is exactly the classification performed by the old commit loop after the
basis membership swap. Therefore a packed scratch vector containing zero for
sets 1-2 and the already evaluated `r'_j` for sets 3-4 is extensionally equal
to the old commit result. It is stronger numerically: commit publishes the
exact floating-point value checked by the postcondition instead of evaluating
the expression a second time.

The transaction boundary is unchanged. Scratch values are formed before the
factor update and do not mutate `State`. A failed FTRAN, DSE update, pivot
identity check, or factor update discards them. They are copied into
`state.reduced_costs` only after the basis update succeeds.

### Acceptance contract

The change is retained only if all of the following hold:

- the mathematical-contract suite and NETLIB regression suite pass;
- `d2q06c` retains the same 5,609 dual pivots and direct presolved publication;
- the 24-case, three-repeat benchmark remains 72/72 accurate for both Native
  and HiGHS;
- no new fallback, cleanup, or phase transition appears in telemetry.

### Validation result: rejected

The implementation passed 1,160 mathematical assertions and 220 NETLIB
regression assertions, but failed the differential path contract on `d2q06c`:
dual pivots changed from 5,609 to 5,955 and primal cleanup changed from 9 to 19
pivots. The result remained accurate, but the changed path demonstrates that
moving the expression into a local temporary changed compiler-visible floating
point evaluation/rounding. Real-arithmetic equivalence was insufficient for
the required bitwise transactional equivalence. The implementation was
removed; only this rejection record remains.

## 2026-08-01: allocation-only packed workspace reuse

The replacement change deliberately leaves every numerical expression and
iteration order untouched. Two temporary objects previously allocated backing
storage during every pivot:

1. the packed entering column passed to FTRAN;
2. the `(row, contribution)` list used to materialize the BFRT flip RHS.

Both are now thread-local workspaces. `clear()` resets logical contents while
retaining capacity. Entries are appended in the same order, BFRT terms use the
same comparator, and equal-row contributions are summed in the same loop.
Consequently the generated floating-point operands and their order are
identical; only allocator calls are removed. The workspaces contain no
solver-owned pointers and are overwritten before use, so failure/rebuild paths
cannot observe stale logical contents.

Acceptance requires the same test, pivot-path, phase-transition, and 24x3
accuracy checks specified above. Benchmark results are recorded after the
validation run.

### Validation result: retained as numerically safe, performance neutral

The contract results were:

- 1,160/1,160 mathematical assertions passed;
- 220/220 NETLIB regression assertions passed;
- `d2q06c` retained 5,609 dual pivots, 9 cleanup pivots, 31 rebuilds, and the
  same direct presolved publication path;
- both solvers were accurate on 72/72 formal benchmark runs.

The formal timing comparison was:

| Path | Accurate | Median (ms) | Geomean (ms) | Geometric speed ratio |
|---|---:|---:|---:|---:|
| HiGHS Simplex | 72/72 | 3.028 | 3.448 | 1.000x |
| Native + presolve | 72/72 | 5.097 | 5.449 | 0.633x |

Raw results are `/tmp/netlib_workspace_reuse_24x3.csv` and
`/tmp/netlib_workspace_reuse_24x3.json`. The prior run was 5.351 ms Native and
3.456 ms HiGHS, so this experiment does not establish an end-to-end speedup;
it is recorded as performance neutral rather than credited toward the target.
The allocation-only implementation is retained because it removes allocator
work without changing a numerical operand, ordering, pivot, phase transition,
or publication decision. No claim of beating HiGHS follows from this change.

## Theory-guided primal cleanup follow-up

The primal cleanup has since been upgraded from Dantzig pricing to a
transactional Devex implementation. The derivation, sign convention, Harris
ratio test, exact PSE recurrence, numerical pivot admission, reinversion
contract, and postsolve publication contract are specified in
`docs/native_primal_pricing_theory.md`.

The implementation adds:

1. Devex pricing by `gain^2 / weight`, with the reference set equal to the
   framework's initial nonbasic variables;
2. transactional weight updates and atomic framework restarts using the
   standard HiGHS Devex mismatch safeguard;
3. a relative pivot floor of `sqrt(machine epsilon)` and an independent
   row-pivot/column-pivot identity check before the irreversible FT update;
4. a separately selectable exact PSE implementation with Goldfarb-Reid updates
   and exact reinitialization after INVERT;
5. direct formula tests against explicit basis inverses, including consecutive
   basis exchanges and the leaving-column special case.

Devex is the default. `MIPSOLVERS_PRIMAL_DEVEX=off|shadow|on|pse` retains
diagnostic and experimental selection. PSE is not the default: it reduces
pivots on the largest cleanup paths, but its exact initialization and extra
BTRAN/PRICE make the complete-suite geometric mean worse.

The formal 24-case, three-repeat comparison is:

| Path | Accurate runs | Median (ms) | Geomean (ms) |
|---|---:|---:|---:|
| HiGHS Simplex | 72/72 | 3.195 | 3.731 |
| Native + presolve + Devex | 72/72 | 3.025 | 4.908 |

Raw results are in `reports/netlib_native_theory_devex_repeat3.csv` and
`reports/netlib_native_theory_devex_repeat3.json`. Relative to the prior stable
Native result below, median improves by about 12.9% and geometric mean by about
5.8%. Native's geometric speed ratio versus HiGHS improves from 0.671x to
0.760x. The final goal is therefore still not reached.

## Result

The optimized Native Dual Simplex path is now accurate on all 24 NETLIB
instances. It is substantially faster than the previous implementation, but it
does not yet beat HiGHS Simplex over the complete suite.

The comparison uses the Release build, three repetitions per instance, one
thread for HiGHS, a 30 second limit, and the same independently audited
accuracy rule as `netlib_solver_benchmark`.

| Path | Accurate runs | Median (ms) | Geomean (ms) |
|---|---:|---:|---:|
| HiGHS Simplex | 72/72 | 3.090 | 3.494 |
| Native Dual Simplex, optimized | 72/72 | 3.474 | 5.210 |
| Native Dual Simplex, previous baseline | 66/72 | 8.028 | 7.680 |

Relative to the previous Native baseline, median time fell by 56.7% and
geometric mean time fell by 32.2%. Relative to current HiGHS, Native is 0.671x
as fast geometrically. Native wins on 10 of the 24 per-case medians and ties on
`ship04s`, but the larger difficult instances dominate the aggregate gap.

Raw results:

- `reports/netlib_dual_enhanced_final_repeat3.csv`
- `reports/netlib_dual_enhanced_final_repeat3.json`

## Changes

1. Added a cost-shifted dual crash for cold models larger than 256 rows. Boxed
   nonbasics select the dual-feasible bound; positive reduced costs on
   lower-only columns are shifted exactly to zero. Dual simplex first builds a
   primal-feasible basis, then the original objective is restored for cleanup.
2. Changed the HiGHS presolve acceptance policy to retain every strict NNZ
   reduction. The old 0.85 threshold paid the complete presolve cost and then
   discarded useful reductions such as `d2q06c` (32,417 to 30,524 NNZ).
3. Replaced PRICE's expanded `vector<pair<column, product>>` plus sort/reduce
   with a thread-local stamped accumulator and a unique touched-column list.
   The touched order is deterministic and no ordering contract is required by
   the ratio test.
4. Added a direct unit test for the cost-shifted dual-start invariant.

These changes also remove both deterministic failures from the previous
baseline: `d2q06c` now reaches an audited optimum instead of exhausting primal
Phase I, and `grow22` succeeds through the retained reduced model.

## Remaining gap

The dominant issue is pivot count during original-objective primal cleanup,
not sparse matrix multiplication alone. On `d2q06c`, the optimized PRICE kernel
reduces the run to about 3.5 seconds, but HiGHS completes in about 0.316 seconds.
Other major gaps are `degen2`, `degen3`, `pilot4`, and `scsd8`.

The next algorithmic milestone is a real primal Devex or projected
steepest-edge implementation with incrementally updated weights. Static column
norms and a three-candidate exact-FTRAN experiment were evaluated and rejected:
the former increased `degen3` pivot count by an order of magnitude, while the
latter reduced pivots but increased end-to-end time due to four FTRAN calls per
pivot. Neither experiment is present in the final code.

The performance target should be considered reached only when a repeated full
suite run has 72/72 accurate results and a geometric speedup greater than 1.0x
versus HiGHS, not when Native wins selected small instances.

## 2026-08-01: E6 dependency-sliced INVERT

### Theory and implementation

For a fixed basis, primal reconstruction depends on the nonbasic bound-side
state but not the cost, while reduced-cost reconstruction depends on the cost
but not the bound-side state:

\[
  x_B=B^{-1}(b-A_Nx_N), \qquad
  r=c-A^TB^{-T}c_B.
\]

`major_rebuild` previously ran both maps after INVERT, again after boxed-side
classification, and a third time after a one-sided cost shift. It now runs the
unchanged full reconstruction once, the unchanged primal block after side
classification, and the unchanged dual block only when costs were shifted.
Objective reconstruction and the final audit remain mandatory. No option,
tolerance, iteration limit, model-size threshold, or case-specific condition
changed. The floating-point contract is in
`docs/native_dual_simplex_performance_theory.md`, section 6.4.

The source change is deliberately narrow: `reconstruct` gained two internal
execution-domain flags and two calls in `major_rebuild` select the required
domain. No parallel implementation or new policy object was added.

### Validation

- Release mathematical suite: 1,166/1,166 assertions;
- NETLIB regression: 220/220 assertions;
- repeated benchmark: 72/72 accurate for both Native and HiGHS;
- differential contract: all 72 Native `(case, repeat, iterations, status,
  objective)` records are identical before and after;
- Native geometric mean: 4.287 -> 4.220 ms (1.6% lower);
- Native/HiGHS geometric speed ratio: 0.785x -> 0.791x;
- raw reports: `reports/netlib_e6_before_repeat3.{csv,json}` and
  `reports/netlib_e6_after_repeat3.{csv,json}`.

The general numerical-stability binary has one unrelated MUMPS KKT failure
(`MumpsSolver solves symmetric indefinite KKT systems`, residuals 5 and 9);
the full run passes 401/403 assertions, and excluding that case passes 397/397
assertions in the remaining 15 cases. E6 does not touch the MUMPS/KKT path.
The target of exceeding HiGHS remains unmet, so E6 is retained as a
path-identical structural gain, not declared as completion.

## 2026-08-01: reliability branching and strong-result reuse

The tree pseudocost contract now stores directional unit gain
`gain / branch_distance` consistently for root probes, strong probes, and
normal child solves. Reliability probing is admitted only when the current
provisional winner is unreliable, and candidates use the same priority plus
pseudocost ordering as final selection. Final selection still considers the
complete candidate set. This removes the previous failure mode where LP work
was spent on weak unreliable variables merely because they were closest to
half-integral.

An exact-domain strong result is moved directly into child processing and
counts as an avoided duplicate LP. A propagated-domain mismatch can reuse only
the primal seed and an independently owned vendored-HiGHS basis; objective,
cutoff status, and native matrix-bound factorization are not reused. Local-cut
counts are part of the cache contract. Branching telemetry reports reliability
nodes, probe candidates/LPs, exact and warm hits, duplicate LPs avoided, and
separate pseudocost/observed-strong regret. The MIPLIB JSON also reports first
incumbent node and LP count.

Release tests pass 134/134 MILP assertions and 15/15 B&C assertions, including
the new distance-normalization contract. In the fixed four-case, 10-second,
seed-0 diagnostic, the node geometric mean changed from 51.392 to 41.478
(-19.29%). The active branching case `enlight_hard` changed from 262 nodes and
991 LP solves to 108 nodes and 794 LP solves; the other three cases had no
reliability probes and measured 386, 1, and 71 nodes in the final run. Every
returned incumbent in the four-case run passed row, bound, integrality, and
objective audit.

This diagnostic does not establish the other requested gates. `enlight_hard`
still produced no incumbent, while the other three cases already found their
first incumbent at the root. Their final gaps did not show a controlled
improvement attributable to branching. The result is retained as a structural
node/LP reduction, not as evidence that Native MILP is yet faster than HiGHS or
that first-incumbent/final-gap targets are complete. Raw results are in
`reports/miplib2017_reliability_branch_after_2026-08-01.json` and
`reports/miplib2017_reliability_branch_final_2026-08-01.json`.

## 2026-08-01: persistent incremental MILP LP state

The sequential B&C path now reuses one HiGHS standard-form model across node
bound changes. It validates structural identity in O(1), updates only changed
column bounds and row RHS values, reapplies the immutable node basis when the
workspace currently represents another node, and continues dual simplex on
the same handle. Root cut rounds use direct `addRows`; accepted rounds commit,
while failed or nonmoving rounds restore bounds/basis and `deleteRows` before
the C++ model snapshot is restored. Rollback reruns the restored basis so its
factorization is immediately usable. Parallel dispatch deliberately leaves
persistent mutation disabled, preserving one mutable solver workspace per
thread.

No tolerance, iteration limit, cut policy, model-size threshold, or
case-specific parameter changed. Contract tests cover handle reuse, failed
child-state rollback followed by another successful solve, basis-solve
availability after rollback, and direct cut-row commit/delete rollback.
Release validation passed 1,200/1,200 dual-simplex assertions, 15/15 B&C
assertions, and 131/131 MILP assertions. A 10-second `50v-10` diagnostic kept
the audited incumbent 6195.589992 and roughly the prior 50% gap while
processing 380 nodes and 868 LP solves; this short timed run demonstrates
correct active use but is not evidence that Native is yet competitive with
HiGHS-MIP.

## 2026-08-01: reliability-branching design consolidation

No solver behavior changed in this documentation-only step. The complete
theory and implementation order were consolidated in
`docs/native_milp_reliability_branching_theory_and_plan.md` before further
branching edits. The specification identifies the update-then-select
information loss, replaces scale-dependent fake failure gains with typed proof
states, defines a candidate-driven node-local exact-score overlay, separates
branch-ranking observations from publishable child bounds, requires
one-solve/one-pseudocost-sample accounting, and makes serial/parallel parity a
contract.

Implementation is divided into independently attributable phases: failure
semantics and tests; node-local overlay; selected-state reuse and sample
deduplication; parallel parity; child-direction scheduling; and only then
pseudocost shrinkage or node-estimate work. Each phase has mathematical,
mechanism, search-quality, and paired wall-time acceptance gates. No probe
count, reliability threshold, score weight, model-size threshold, or
instance-specific parameter was changed.

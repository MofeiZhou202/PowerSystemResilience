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

## 2026-08-03: rejected E3 cost-model PRICE

The provisional `4 * row_ep.nnz >= m` selector and the CSC
`cols >= 4096 && nnz >= 65536` parallel gate were removed. They treated row
support density and model size as portable performance laws, which they are
not. The replacement implements the section 6.1 cost model directly: each
pivot computes the exact CSR work
`sum(row_end[row] - row_start[row])`; one per-solve calibration observes CSR,
serial CSC, and (when available) parallel CSC; later pivots select the smallest
predicted cost from cumulative time / structural-work observations. No case
name, density percentage, matrix-size cutoff, shift statistic, or environment
policy enters the selector. The calibration pivot returns CSR output, so the
measurement itself cannot change the first pivot's floating-point path.

The vendored HiGHS 1.14 uses a 0.75 local `row_ep` density cutoff for column
PRICE and `priceByRowWithSwitch` for sparse-to-dense result handling, backed by
running result-density averages. The useful design lesson is local/historical
state and mid-operation adaptation; its literal density constants are
deliberately not copied. The rejected selector improved the structural proxy
by using actual row work and identified hardware costs online.

Kernel tests at 10/25/33/50/100%
uniform support put the serial CSC crossover between 25% and 33%, while also
demonstrating why that percentage must not become policy. The adaptive solve
selected CSC 1032/1411 times on `scsd8` and 2312/2902 times on `25fv47`;
three-repeat medians were 58.97 ms and 207.44 ms, respectively, with identical
pivot counts across repeats and accurate objectives.

The paired fixed NETLIB 24 x 3 gate rejected the change. With identical
sentinel/warm-start code, CSR-only achieved 69/72 accurate results, 4.297 ms
native geometric mean, and 0.757x versus HiGHS. Online adaptive PRICE retained
69/72 but measured 4.472 ms and 0.743x: a 4.1% native regression. The first
pivot's extra kernel evaluations and every-pivot clock reads cost more across
short models than CSC recovered on the dense long paths. Production wiring,
timing state, and selector tests were removed; only the forced CSC kernels,
rounding-envelope test, and opt-in synthetic benchmark remain. Raw paired
results are `reports/netlib_e3_{csr,cost_model}_repeat3.{csv,json}`. This
rejection is theory-driven: it falsifies the assumed
amortization term, rather than adding another case or size threshold.

The threshold-free HiGHS-style follow-up was also rejected. Row-wise PRICE
switched from stamped accumulation when `current_support + next_row_nnz >= n`,
a structural saturation condition rather than a tuned density. Per-column
double results stayed bit-exact because row accumulation order was unchanged,
but switching required O(n) zero materialization and O(n) result export. The
fixed cohort remained 69/72 accurate and measured 4.402 ms native geometric
mean, 2.4% slower than the 4.297 ms CSR-only control. The production switch
was removed. Raw results are
`reports/netlib_e3_row_switch_repeat3.{csv,json}`. E3 is closed until a design
eliminates the dense traffic term; changing its saturation condition would be
threshold retuning, not a new theory.

## 2026-08-03: E4 reinversion measurement, policy unchanged

E4 added opt-in diagnostic telemetry without changing the simplex path when
`MIPSOLVERS_DS_PROFILE` is unset. HFactor now accumulates wall time and
synthetic work for every indexed solve while profiling. The dual driver takes
counter snapshots around a complete minor iteration, so the fitted solve cost
includes unit BTRAN, pivotal-column FTRAN, BFRT FTRAN, and DSE auxiliary
FTRAN. The first version measured only the first two and was rejected as an
incomplete lower bound.

The estimator groups observations by exact `updates_since_rebuild` age, fits
the age means, and separates Dual Phase I from Phase II. Scheduled rebuild
cost uses only `UpdateLimit` events. HFactor `SyntheticWork`, numerical
trouble, and terminal certification are reported as safety reinversions and
remain dominant over any future schedule. Wall \(R\) covers the full major
rebuild. Synthetic \(R\) is explicitly reported as `R_build_tick` because
the dense reconstruction solves do not expose synthetic-clock work. This
asymmetry is another reason synthetic \(T^*\) is corroboration rather than the
wall-time decision metric.

Three-repeat results were stable after warm-up. Phase-II synthetic fits had
\(R^2\) of 0.962 (`scsd8`), 0.970 (`25fv47`), 0.938 (`degen3`), 0.977
(`d2q06c`), and 0.842 (`grow7`). Corresponding
`(scheduled interval, T_wall, T_tick)` values were about `(99, 89, 52)`,
`(168, 94, 54)`, `(200, 141, 93)`, `(200, 133, 72)`, and `(50, 72, 48)`.
Synthetic ticks robustly establish linear update-age growth, but their fixed
operation weights are not calibrated to this machine's relative INVERT and
triangular-solve wall costs. They are therefore structural corroboration, not
the optimization objective.

The predeclared policy gate required wall \(T^*\) to differ from the effective
interval by more than 2x on cases totaling over 10% of suite time. No measured
case met it; the largest ratio was about 1.8x on `25fv47`. The existing
`max(50, min(200, m/4))` update limit is retained. No interval, clamp,
matrix-size condition, or case-specific rule was added.

## 2026-08-04: rejected strict-traffic PRICE/BFRT kernels

Three threshold-free Class-P experiments tested whether a strict reduction in
hot memory operations is sufficient to improve PRICE/BFRT.

First, BFRT candidate records dropped an unused copied margin and moved taboo
lookup from candidate construction to Harris-eligible candidates. Scan
evaluation records also dropped fields used only inside their producing loop.
Candidate order, breakpoint arithmetic, sorting keys, capacity summation, and
taboo selection were unchanged. On `d2q06c`, five-repeat medians were
1397.5 ms control, 1426.4 ms experiment, and 1398.7 ms restored control: a
reproducible 2.1% regression. The layout change was removed.

Second, PRICE used its output index vector directly as the touched list and
compacted it in place. If \(S\) columns are touched and \(K\) survive
tightening, index writes changed from \(S+K\) to \(S+\text{moves}\), with
\(\text{moves}\le K\). Despite this non-increasing traffic proof, the fixed
24x3 cohort retained 69/72 accuracy but moved from 4.277 to 4.453 ms native
geometric mean and from 0.737x to 0.720x versus HiGHS. It was removed.

Third, the stamped CSR accumulator was replaced by a HiGHS-style value-marked
accumulator. Export cleared every touched value, eliminating the stamp array
and one stamp read per visited matrix term. Since a literal `value == 0`
test duplicates an index after exact cancellation and later revival, negative
zero represented 'touched but currently zero' while positive zero represented
'untouched'. A dedicated cancellation/reuse test passed, and all solver paths
remained identical. The cohort nevertheless measured 4.329 ms and 0.733x,
1.2% slower in native time than the same 4.277 ms control. The sign test and
touched-value clear offset the removed stamp traffic; this implementation and
its test were removed.

Raw paired reports are
`reports/netlib_e3_{strict_traffic_control,inplace_index_exp,no_stamp_exp}_repeat3.{csv,json}`.
These results close scalar bookkeeping rearrangements around the existing CSR
kernel. The next proposal must remove traffic and dynamic instructions
together, or establish a locality transformation with a kernel benchmark
before entering the solver.

## 2026-08-04: retained HFactor-backed DSE intersection extraction

The next experiment moved from scalar PRICE bookkeeping to an ownership
boundary already used by HiGHS. `HVector` retains each FTRAN result in a dense
backing array; HiGHS' DSE update reads that array on the pivotal-column
support. Native previously exported the entire packed auxiliary FTRAN result,
cleared an $m$-entry `dense_rho`, scattered the packed result back into it,
and then gathered only the pivotal-column coordinates needed by the
Goldfarb-Reid recurrence.

The new HFactor backend operation solves the same indexed RHS and extracts the
result directly on the previously captured pivotal FTRAN pattern. For
$d=|\operatorname{supp}(B^{-1}a_q)|$ and
$r=|\operatorname{supp}(B^{-1}B^{-T}e_p)|$, it replaces the $m$-entry clear,
packed $r$-entry export, and $r$-entry scatter with one contiguous
$d\le m$ extraction. It introduces no threshold or dispatch policy. FTRAN,
recurrence, rounding, and commit order are unchanged; the indexed-solve wall
and synthetic-work telemetry still records the operation.

A coordinate differential test covers a permuted basis and compares the
extracted sequence with ordinary packed FTRAN lookup. The fixed 24x3 A/B/A
cohort produced native geometric means of 4.256 ms control, 4.234 ms direct
extraction, and 4.263 ms restored control. Accuracy remained 69/72. After
deleting only `runtime_ms`, all native JSON run records were identical across
control and experiment, including iterations, status, objective, and
feasibility audit. The approximately 0.6% fleet improvement is retained
because it follows a strict materialization-elimination proof and passes the
suite gate, not because of any per-case result.

Raw A/B/A results are
`reports/netlib_dse_extract_{control_a,experiment,control_c}_repeat3.{csv,json}`.

## 2026-08-04: grow22 canonical reconstruction fixed, 72/72

The three reported NETLIB failures were three repeats of one deterministic
failure, `grow22`, not three different instances. At the scheduled rebuild
after pivot 220, HFactor's BTRAN residual was (6.7\times10^{-16}), while the
canonical (Ax=b) residual was (6.1467\times10^{-8}) against the fixed
(10^{-8}) limit.

The failure came from an incomplete audit-cache dependency. Primal
reconstruction is (x_B(B,s)=B^{-1}(b-A_Nx_N(s))), but the cached audit key
contained only the factor generation (B). `major_rebuild` audited the old
side vector, changed boxed nonbasics to their dual-feasible sides, then skipped
correction of the new primal reconstruction because the factor generation had
not changed. Bound-side reconstruction now requests an exact residual audit,
so a cached result for the preceding (s) cannot be reused.

One correction reduced the failing residual to (1.4901\times10^{-8}) but did
not cross the unchanged limit because the update to a large basic value was
quantized by its double ULP. The former one-step correction was therefore
completed into monotone iterative refinement. Each accepted step recomputes
the residual in long double, must strictly reduce its infinity norm, and still
fails closed on stagnation or after the machine-precision safety bound. No
feasibility tolerance or benchmark acceptance threshold changed.

The new `grow22` regression requires native success, the published objective,
and audited residual. The full 24x3 cohort is now 72/72 successful and
accurate for both Native and HiGHS. Native geometric mean is 4.788 ms versus
3.589 ms for HiGHS, or 0.750x. The old 69/72 geometric mean counted the early
failure runtime and is retired as a performance baseline. Raw results are
`reports/netlib_correctness_72_repeat3.{csv,json}`.

## 2026-08-04: rejected BFRT RHS materialization removal

The next experiment tested the remaining `(row, contribution)` materialization
in the BFRT flip RHS. For (L=\sum_{j\in F}\operatorname{nnz}(A_j)) and
(S=|\cup_{j\in F}\operatorname{supp}(A_j)|\le L), a stamped row accumulator
first replaced (O(L)) pair writes, (O(L\log L)) sorting, and (O(L))
reduction reads by (O(L)) accumulation and (O(S\log S)) index sorting. A
second variant exported rows in deterministic first-touch order and removed
the remaining sort, giving (O(L+S)). Neither variant used a case, density,
or dimension threshold.

Both variants were Class A because the old `std::sort` did not preserve the
order of equal-row contributions. They passed the complete correctness gates
and the fixed cohort remained 72/72 accurate. The sorted-stamp A/B/A native
geometric means were 4.861 ms experiment, 4.851 ms control, and 4.836 ms
experiment; the experiment mean differed from control by only about 0.05%.
The no-sort variant measured 4.849 ms versus the 4.851 ms control. These are
performance-neutral, while four NETLIB pivot paths changed, so both scalar
implementations were removed.

A stronger ownership-boundary variant mirrored HiGHS `collectAj`: HFactor
collected flip columns directly into its reusable `HVector`, performed the
materialized-coverage projection there, and consumed the same backing storage
for FTRAN. This additionally removed the indexed RHS export and HFactor
scatter. A permuted-basis coordinate differential test and all correctness
gates passed, but the fixed cohort measured 4.858 ms native and 0.740x versus
HiGHS, below the 4.851 ms / 0.744x control. It too was removed.

The result is a negative cost attribution, not permission to add a selector:
BFRT RHS construction is not a fleet-level bottleneck in this cohort even
when all of its avoidable materialization is deleted. Pricing thresholds or
case-specific dispatch cannot repair an absent aggregate payoff. Raw reports
are `reports/netlib_bfrt_{stamped_rhs,stamped_rhs_control,stamped_rhs_experiment_c,first_touch_rhs,hfactor_direct}_repeat3.{csv,json}`.

## 2026-08-04: independent branch-free BFRT prefilter proof

The next L4 experiment isolates the Phase-II cheap BFRT classification before
`dot_error_bound`. The scalar reference follows the production strict
comparison order. The AArch64 NEON kernel processes two packed PRICE entries,
computes signed alpha, range, stability rejection, cheap rejection,
certification, and exact-required masks without candidate-dependent branches,
and writes two doubles plus one flag byte per lane. Two adjacent flag bytes
are committed by one packed 16-bit store. The kernel is standalone and is not
linked into `pricing.cpp`.

Correctness is exact for every flag and bitwise for finite/signed-zero alpha
and range. Independent expectations cover all-basic, all-fixed, odd lengths,
`alpha == 0`, `alpha == stable_tolerance`, `alpha == cheap_error`, negative
`alpha + cheap_error == 0`, and NaN/Inf in every floating input class. Lanes
marked exact-required are explicitly returned to the existing scalar sparse
dot; the experiment does not approximate its error bound.

The Apple Clang 21 `-O3` disassembly gives seven scalar path lengths of
37/40/56/56/57/58/58 instructions and a fixed NEON body of 80 instructions
per two lanes. Weighting those blocks by the measured category counts for
1,048,576 lanes gives 58,083,022 scalar versus 41,943,064 NEON dynamic machine
instructions, a 27.788% reduction. The semantic array model gives 86,974,187
versus 57,671,680 bytes, a 33.691% reduction. This model is compiler-gated and
reports unavailable for an odd scalar tail or an unaudited toolchain; it does
not present predicate counts as total machine instructions or claim hardware
performance-counter data.

Five serial runs of
`./tests/native_dual_bfrt_simd_benchmark 1048576 20 7` all passed. Their
seven-sample median speedups were 1.326x, 1.393x, 1.375x, 1.349x, and 1.332x,
with a 1.349x median across runs. The byte proof is distribution-dependent
while inactive lanes remain in the input: its exact condition is
`31*active > 2*basic + fixed`. The production design must therefore make
PRICE emit active-only packed support, which makes the 31-byte per-lane output
reduction structural and threshold-free. Only after the scalar exact fallback
and deterministic merge are included may this enter the full correctness and
24x3 fleet gates. No complete-solver speedup is claimed yet.

## 2026-08-04: production active PRICE/BFRT SIMD retained

The production integration keeps the complete PRICE pivotal row: Devex needs
basis-reference coefficients and reduced-cost commit needs the full update.
The PRICE producer now records, during that same export loop, the packed
positions whose columns are both nonbasic and movable. This four-byte position
stream preserves the exact PRICE order and does not duplicate pivot values.
BFRT therefore begins with an active-only domain without a second basic/move
scan or any density, size, or case policy.

In Phase II the two-lane NEON prefilter writes signed alpha and one flag byte
per active lane; the later deferred-range change removed its derived range
stream. Stability-prefiltered and cheap-rejected lanes merge without a full
evaluation record; only `needs_exact` lanes call the unchanged sparse
`dot_error_bound`. Cheap-certified lanes retain the same error coefficient
product. Candidate insertion, all capacity sums, taboo lookup, and subsequent
BFRT ordering remain in original PRICE order. Dual Phase I retains the prior
scalar exact path. The former `BfrtScanEvaluation[]` materialization is not
created on Phase II.

The producer regression proves that the complete pivotal row is byte-for-byte
unchanged and that the active positions preserve tiny-drop and PRICE order.
All correctness gates passed: 9,961 dual-simplex assertions across 64 cases,
41 LP assertions, 224 NETLIB assertions, 409 numerical-stability assertions,
and zero SCUC must-pass failures.

A temporary experiment-only off switch compared the original complete scan
against the integrated path in one binary and was removed afterward. The fixed
24x3 A/B/A native geometric means were 5.140 ms control, 4.803 ms experiment,
and 5.187 ms control. Relative to the geometric mean of the two controls, the
retained implementation is 7.5% faster. Every run was 72/72 successful and
accurate; after deleting `runtime_ms`, all three JSON run arrays were exactly
identical. The final paired run measured Native 4.780 ms versus HiGHS 3.727 ms,
or 0.780x. Raw data are
`reports/netlib_bfrt_simd_{control_a,experiment_b,control_c,production}_repeat3.{csv,json}`.

## 2026-08-04: retained production structural exact DSE

The next pivot-count experiment separates three cold edge-weight policies:
explicit Devex, structural exact DSE, and full exact DSE. For a basis whose
column assigned to row \(i\) has exactly one nonzero \(d_i\) in that row,
\(B\) is diagonal in basis order and the exact DSE weight is
\(\lVert B^{-T}e_i\rVert_2^2=1/d_i^2\). The structural policy uses that
identity and otherwise starts Devex. The full policy uses the same identity
when applicable and checked BTRAN weights otherwise. This is a structural
proof boundary with no case, size, density, or timing threshold.

All 144 structural/full NETLIB runs took the analytic path: initialization
solve count was zero, and the two policies were identical on pivots, status,
objective, and audited feasibility in all 72 paired runs. The formal
three-way-plus-HiGHS measurement was:

| Policy | Accurate | GeoMean ms | Mean pivots | Mean init ms | Kernel ms/pivot |
|---|---:|---:|---:|---:|---:|
| HiGHS simplex | 72/72 | 3.587 | - | - | - |
| Native Devex | 72/72 | 4.658 | 824.7 | 0.00063 | 0.02003 |
| Native structural exact DSE | 72/72 | 4.090 | 639.0 | 0.00158 | 0.02007 |
| Native full exact DSE | 72/72 | 3.977 | 639.0 | 0.00159 | 0.01940 |

The structural/full wall-time difference is measurement-order noise because
their mathematical and discrete paths are identical. Across per-case medians,
structural DSE won 19 cases and lost 5; pivots decreased on 14, increased on
4, and were zero under both policies on 5. The largest structural wins were
`lotfi` (0.621x Devex time) and `d2q06c` (0.654x, 8,510 to 5,609 pivots).
The largest regressions were `stocfor2` (1.187x, 1,041 to 1,054 pivots) and
`grow22` (1.035x, 732 to 810 pivots). These losses are evidence, not selectors.

The independent 24x3 A/B/A bracket measured 4.677 ms Devex, 4.341 ms
structural DSE, and 4.621 ms Devex. The control geometric mean was 4.649 ms,
so structural DSE reduced elapsed time by 6.61%; controls differed by 1.19%.
Mean pivots fell 22.52%, while geometric kernel time per pivot rose 3.59%.
Initialization added only 0.00109 ms over the controls. This attributes the
fleet gain to better pivot selection rather than a per-pivot kernel change.

The first formal run exposed three repeated failures on `grow22`, all the same
deterministic terminal residual. Primal cleanup performed a major INVERT and
reconstruction, but its sampled residual audit did not share the long-double
termination contract. Primal major reconstruction now requests the exact
canonical audit and monotone defect correction immediately. The fixed
\(10^{-8}\) feasibility tolerance was not changed; `grow22` became 9/9
accurate in the targeted rerun with the same DSE pivot count, after which the
complete correctness suite and formal cohort passed.

Raw results are `reports/netlib_dse_threeway_repeat3.{csv,json}` and
`reports/netlib_dse_aba_{A1_devex,B_structural,A2_devex}_repeat3.{csv,json}`.
These first measurements justified a production gate, but did not themselves
select a policy.

The production gate used two fixed binaries rather than an in-process switch:
the old production Devex binary bracketed the new production binary. Native
geometric means were 4.674 ms Devex, 4.369 ms structural exact DSE, and 4.664
ms Devex. Their control geometric mean is 4.669 ms, making the retained policy
6.42% faster; the controls differ by 0.21%. Mean pivots reproduced the earlier
824.7 to 639.0 change, and the structural initialization averaged 0.00142 ms
with zero BTRAN. Every native and HiGHS run was accurate (72/72 per bracket),
and every native status was Optimal. The experiment bracket measured HiGHS at
3.559 ms, moving Native/HiGHS from 0.766x in the first control to 0.814x.

Production now uses the analytic weights only when every cold basis column is
an assigned-row singleton. Failure of that proof condition deterministically
falls back to Devex; warm cached-weight handling is unchanged. Explicit
Devex/StructuralExact/FullExact/CertifiedExact modes remain available for
controlled experiments, but CertifiedExact is not a production path. Raw
production-gate data are
`reports/netlib_structural_production_{A_devex,B_exact,C_devex}_repeat3.{csv,json}`.

The structural identity was subsequently extended to uncached warm hints. The
proof depends only on the current assigned-row-singleton basis, not on whether
the basis is cold. Matching cached DSE weights remain first priority;
Production and explicit StructuralExact use analytic weights on a structural
uncached hint, while a nonstructural hint and explicit nonproduction warm modes
retain Devex. Tests cover both branches and zero initialization BTRAN.

## 2026-08-04: rejected certified exact CHUZR

The recursive DSE heap has a genuine proof gap: after its selected row is
recomputed by BTRAN, the implementation does not re-establish that the row is
the global exact-DSE maximizer. A benchmark-only fourth mode closed that gap
without copying HiGHS' empirical `updated_weight >= 0.25 * exact_weight`
decision. From (b_i^TB^{-T}e_i=1), the checked-BTRAN residual contract, and
the residual dot-product rounding envelope, it derives a positive lower bound
on every candidate weight and therefore an upper bound on every candidate
merit. It evaluates candidates by descending upper bound and stops only after
the best checked merit dominates all unvisited bounds.

The brute-force exact-merit oracle and the complete dual-simplex suite passed.
The four-case targeted performance gate did not:

| Case | Devex pivots | Structural pivots | Certified pivots | Certified total ms | Certified BTRAN |
|---|---:|---:|---:|---:|---:|
| `grow22` | 732 | 810 | 806 | 321.59 | 33,444 |
| `fit1d` | 67 | 70 | 70 | 7.06 | 604 |
| `recipe` | 35 | 36 | 36 | 0.25 | 138 |
| `stocfor2` | 1,041 | 1,054 | 1,061 | 1,802.75 | 125,306 |

All 36 results were accurate. The four-case geometric means were 31.460 ms
CertifiedExact, 5.529 ms Devex, and 5.499 ms StructuralExact. Exact selection
did not remove any of the four pivot regressions, while the proof-safe lower
bound was too loose to avoid many BTRANs on large bases. This falsifies the
hypothesis that eager global weight correction is the missing algorithmic
improvement. The mode and its telemetry remain benchmark-only for
reproducibility; production never selects it. Per the staged gate, no
full 24x3 A/B/A was run. Future work must either derive a structural bound that
reduces the solve count asymptotically or target a different source of pivot
quality; empirical selectors are out of scope. Raw data are
`reports/netlib_certified_dse_targeted_repeat3.{csv,json}`.

## 2026-08-04: rejected CHUZR row-stream fusion

The next Class-P experiment removed `changed_primal_rows`: after committing a
`primal_changes` entry, it immediately refreshed that row's lazy CHUZR heap.
This preserved the producer row order and every heap operation while deleting
one index write, one compact-index read, one loop, and the redundant search for
the already-present leaving row. It contained no case, size, or density rule.

On `d2q06c`, a 3-repeat control/experiment/control bracket measured geometric
means of 1275.619 / 1279.995 / 1278.427 ms with exactly 8,510 pivots and accurate
results throughout. Relative to the 1277.022 ms control geometric mean, the
experiment regressed 0.23%. The compact four-byte row stream evidently pays
for itself by shortening the commit dependency chain and improving locality
over revisiting `pair<int,double>` records. The implementation was removed;
the failed targeted gate intentionally stopped before 24x3.

## 2026-08-04: retained redundant CHUZR search removal

The rejected row-stream fusion was decomposed before closing the work. Since
the transaction builder always appends `leaving.row` and the compact row stream
copies every `primal_changes` row in order, the subsequent `std::find` for that
same row and its fallback heap insertion are unreachable. Removing only this
search preserves the compact consumer layout that the fusion lost. It changes
no heap push, merit, row order, tie break, or pivot.

A full Native+HiGHS B/A/B measured Native geometric means of 4.612 / 4.716 /
4.685 ms and simultaneous HiGHS means of 3.512 / 3.580 / 3.573 ms. The effect
is below stable wall-time resolution, so no speedup is claimed. Every bracket
was 72/72 accurate. With timing fields and per-case load time removed, all
three JSON documents were byte-identical. The change is retained because it
strictly removes the per-pivot linear instruction stream without changing the
memory layout; it introduces no threshold or runtime policy. Raw data are
`reports/netlib_chuzr_{no_extra_scan_repeat3,extra_scan_control_repeat3,
no_extra_scan_repeat3_c}.{csv,json}`.

## 2026-08-04: retained DSE value-only transaction stream

The pivotal-column `direction.index` already owns the ordered rows updated by
the Goldfarb-Reid recurrence and remains valid through factor update and
commit. The old DSE transaction duplicated each row in a 16-byte
`{row,value}` record. The retained form materializes only the eight-byte values
for nonpivotal rows plus one pivotal value, validates every value before any
state mutation, and commits against the original direction stream. It deletes
at least 12 bytes of transaction traffic per nonpivotal row without changing
arithmetic, order, rollback boundaries, or pivots.

The complete correctness suite passed. The 24x3 paired gate was 72/72 accurate
with 639.0 mean pivots and measured Native 4.355 ms versus HiGHS 3.542 ms
(0.813x). The preceding production measurement was 4.369 ms, so no wall-time
speedup is claimed; retention follows from strict stream dominance and
identical discrete paths. Raw data are
`reports/netlib_dse_value_stream_full_repeat3.{csv,json}`.

## 2026-08-04: rejected PRICE-owned BFRT absolute dot

A producer-fusion experiment accumulated `sum_i |row_ep_i A_ij|` next to every
signed PRICE accumulator. BFRT could then evaluate the unchanged exact error
formula without materializing a dense row or rereading `needs_exact` CSC
columns. This moved selective consumer work into the universal producer: every
PRICE matrix term gained an absolute-value RMW and dependency.

On `d2q06c`, fixed control/experiment/control geometric means were 857.9 /
898.2 / 855.1 ms. The experiment regressed 4.9% against the 856.5 ms control
geometric mean, increased kernel time per pivot from about 0.150 to 0.156 ms,
and changed the path from 5,609 to 5,678 pivots because the absolute sum used
PRICE rather than CSC accumulation order. All runs were accurate, but the
traffic/instruction gate failed, so all production code was removed. Raw data
are `reports/netlib_price_abs_d2q_{control_a,experiment_b,control_c}.{csv,json}`.

## 2026-08-04: retained deferred BFRT range

The branch-free Phase-II prefilter no longer exports a dense `range` value
stream. `range=upper-lower` is derived from immutable model bounds, and only
positive signed-alpha lanes consume it for capacity and candidates. The
original-order merge now computes it once inside that positive branch. This
unconditionally deletes 16 workspace bytes per active lane and also skips 16
bound bytes plus one subtraction on every nonpositive lane. Scalar
`dot_error_bound`, flags, capacity accumulation, candidate order, and all BFRT
transaction semantics are unchanged.

`d2q06c` kept the 5,609-pivot path. All dual-simplex, LP, NETLIB, numerical-
stability, and SCUC must-pass gates passed. The full 24x3 run was 72/72 accurate
for both Native and HiGHS, with 639.0 mean Native pivots. Native measured 4.358
ms versus 3.502 ms for HiGHS (0.804x), within noise of the previous 4.355/3.542
ms run. The change is retained for strict traffic dominance, not a wall-time
claim. Raw data are
`reports/netlib_bfrt_deferred_range_full_repeat3.{csv,json}`.

An earlier attempt in the same iteration stored prevalidated reduced costs in
`pivot_row.value` so commit could avoid recomputation. It reduced the profiled
`rcUpdate` bucket from about 0.04 seconds to near zero on `d2q06c`, but changed
the path from 5,609 to 5,592 pivots because the committed expression moved to
a different floating contraction/rounding context. It failed the discrete-
path gate and was fully removed.

## 2026-08-04: retained minimal BFRT candidate state

An accepted candidate satisfies `alpha = leaving_side * move_sign * pivot > 0`.
Both signs are exactly +/-1 and remain live in state, so the selected pivot is
recovered bitwise as `leaving_side * move_sign * alpha`. Margin is already
represented by `breakpoint=margin/alpha` and has no later consumer. Candidate
records therefore retain only column, alpha, breakpoint, range, and taboo
metadata.

This reduces the Apple AArch64 candidate layout from 56 to 40 bytes, deletes
the Phase-II merge's candidate-only pivotal-row read, and removes the copied
pivot from the Phase-I evaluation record. It does not repeat the earlier
rejected compound experiment: taboo lookup remains in its established place,
so no hash-probe dependency chain or candidate order changes.

Exact positive/negative pivot tests and the complete correctness suite passed.
`d2q06c` retained 5,609 pivots; the profiled BFRT bucket moved from about 0.16
to 0.15 seconds. The full 24x3 gate was 72/72 accurate for Native and HiGHS,
with 639.0 mean Native pivots. Native/HiGHS geometric means were 4.359/3.568 ms
(0.819x). Normalized non-time reports were identical to the preceding
production gate. Retention is based on the minimal-sufficient-state proof, not
a wall-time claim. Raw data are
`reports/netlib_bfrt_minimal_candidate_full_repeat3.{csv,json}`.

# Native Dual Simplex optimization (2026-07-31)

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

# Native Dual Simplex Stabilization Architecture

> Development-only: production modules use `LpKernelBackend::HiGHS`. This
> document governs explicit `ExperimentalNative` work only.

> Status: algorithm contract. This document supersedes the assumption that a
> correct sparse Forrest-Tomlin factor is sufficient for a production dual
> simplex. The factor is one component of the solver; the major/minor rebuild
> state machine is the numerical stabilization mechanism.

## 1. Evidence and conclusion

The ACTIVSg2000 FMEA regression reached thousands of valid basis exchanges.
At failure, individual HFactor solves often satisfied the unchanged normwise
backward-error bound. The remaining failures were accumulated-state failures:

- an incrementally updated primal component and its adjoint projection no
  longer agreed at the scale assumed by the minor iteration;
- exact DSE weights and recursively updated weights diverged after long update
  chains;
- a fresh factor could be accurate while the incrementally maintained primal,
  reduced-cost, and objective state was no longer fresh;
- possible optimality or infeasibility was being treated as a terminal event
  before rebuilding the state on the current basis.

Therefore replacing LU or tuning an FT threshold cannot complete the solver.
The native implementation must reproduce the stabilization role performed by
the `HEkkDual` control layer around HiGHS' factorization.

## 2. Reference-source map

The implementation is derived from these vendored reference components:

| Reference component | Mechanism to implement natively |
|---|---|
| `highs/simplex/HEkkDual.cpp::solvePhase2` | major rebuild loop around minor iterations |
| `HEkkDual.cpp::rebuild` | recompute dual, primal, infeasibility set and objective |
| `HEkkDual.cpp::cleanup` | remove working-cost perturbations and shifts before termination |
| `HEkkDual.cpp::iterate` | ordered minor-iteration transaction |
| `HEkkDual.cpp::chooseRow` | DSE verification before accepting a leaving row |
| `HEkkDual.cpp::updateVerify` | convert row/column disagreement into a rebuild reason |
| `HEkkDual.cpp::correctDualInfeasibilities` | working-cost correction after rebuild |
| `HEkkDualRHS.cpp` | maintained primal-infeasibility array and candidate set |
| `HEkk.cpp::rebuildRefactor` | reason-driven INVERT policy |
| `HEkk.cpp::updateFactor` | update-count and synthetic-work rebuild triggers |
| `HEkk.cpp::initialiseCost` | perturb a working objective, never the published objective |

The native kernel does not copy HiGHS' parallel slices, PAMI minor iterations,
basis rollback/backtracking, logging framework, or runtime fallback. It does
implement serial cycle detection and scoped taboo basis changes, without
rolling back a committed pivot.

## 3. Two-level state machine

The solver has a major state and a minor state.

```text
MAJOR REBUILD
  INVERT current basis when required
  reconstruct x_B, y, r and objective from the current working problem
  rebuild primal infeasibilities and exact/verified edge weights
  correct working dual infeasibilities by deterministic cost shifts
  record drift against the previous incremental state
  mark state fresh
        |
        v
MINOR ITERATIONS
  CHUZR -> BTRAN -> PRICE/BFRT -> FTRANs -> verify -> atomic pivot commit
  update x_B, r, objective, infeasibility set, weights and factor
  mark state non-fresh
        |
        +-- update limit / synthetic work / numerical evidence --> MAJOR
        +-- possibly optimal / infeasible ----------------------> MAJOR
        +-- no event --------------------------------------------> next minor
```

A `RebuildReason` is not an LP status. It is evidence that the current minor
state cannot publish a conclusion. The initial reasons are:

```text
Initial
UpdateLimit
SyntheticWork
NumericalTrouble
PrimalDrift
DualDrift
ObjectiveDrift
PossiblyOptimal
PossiblyPrimalInfeasible
Cleanup
```

There is no pivot blacklist and no alternative-pivot search. A numerical event
rebuilds the same current basis. If the same algebraic event is present in a
fresh state, the solver returns a numerical status with its residual evidence.

## 4. Freshness contract

`fresh_rebuild` means all of the following were computed from the same current
basis and the same working cost vector:

```text
x_B = B^-1 (b - N x_N)
y   = B^-T c_B
r   = c_work - A^T y
z   = c_work^T x
I_p = {i: x_B[i] violates its active bounds}
```

Every basis exchange or bound flip clears freshness. A factor solve alone does
not restore freshness. Only the major rebuild operation can set it.

The incremental state is compared with the rebuilt state before replacement:

```text
delta_x = ||x_B_incremental - x_B_rebuilt||_inf
delta_r = ||r_incremental - r_rebuilt||_inf
delta_z = |z_incremental - z_rebuilt|
```

These are telemetry and rebuild evidence, not relaxed feasibility tolerances.
The published feasibility and optimality checks continue to use the original
`feasibility_tol` and `optimality_tol`.

## 5. Working objective and original objective

The solver owns two costs:

```text
c_original  immutable canonical objective
c_work      objective used by stabilized minor iterations
sigma       c_work - c_original
```

### 5.1 Deterministic perturbation

Cost perturbation separates large degenerate breakpoint groups. It is applied
once to `c_work`, with a deterministic column-index sequence. It never changes
`c_original`, feasibility bounds, an iteration budget, or a solver tolerance.

The sign follows the bound geometry of the fixed standard-form variable:

```text
lower-only:  c_work[j] = c_original[j] - delta_j
upper-only:  c_work[j] = c_original[j] + delta_j
boxed:       perturb away from zero using sign(c_original[j])
fixed:       no perturbation
```

The magnitude is scale-relative and recorded in statistics. It must not be
used in a published optimality claim.

### 5.2 Deterministic cost shift after rebuild

For a nonbasic variable with move `d_j`, working dual feasibility is

```text
d_j r_j <= tau_d.
```

If reconstruction violates this condition, a working-cost shift changes only
`c_work[j]` so that the reconstructed reduced cost is exactly zero. This is a
stabilized auxiliary objective, not a tolerance relaxation. Free variables
cannot be repaired this way and force a phase/status decision.

### 5.3 Cleanup

Possible optimality on `c_work` triggers cleanup:

1. restore `c_work = c_original` and clear all shifts/perturbations;
2. perform a fresh major rebuild on the unchanged basis;
3. if primal and dual audits pass, publish `Optimal`;
4. otherwise continue with the unperturbed problem or invoke the explicitly
   modelled primal cleanup phase.

The solver must not cycle between perturbation and cleanup. Once cleanup starts,
cost perturbation is disabled for the remainder of that solve.

## 6. Factor and rebuild policy

Forrest-Tomlin updates remain the normal minor-iteration basis representation.
The driver, rather than the factor wrapper, owns reinversion decisions.

An INVERT is required when any of these occurs:

- HFactor reports its update/fill hint;
- the deterministic update-count limit is reached;
- accumulated sparse-solve work exceeds the measured INVERT work after the
  minimum update count;
- a checked FTRAN/BTRAN exceeds the unchanged backward-error bound;
- the row/column pivot residual identity fails;
- a terminal candidate is not based on fresh state.

Every solve returns structured evidence: accepted, needs-rebuild, residual,
error limit, and whether iterative refinement was used. The factor wrapper is
forbidden from calling `factorize`. Iterative refinement is a linear solve
operation, not an alternative simplex attempt, and is allowed for ordinary,
batched, and pivotal solves. A refined pivotal vector no longer matches
HFactor's captured FT pack. The already verified exchange is therefore committed
without that pack, and the driver immediately INVERTs and reconstructs the new
basis before another minor iteration.

Backward stability of `Bx_B=rhs` can still be weaker than the canonical
feasibility contract when `||B|| ||x_B||` is large. Reconstruction therefore
forms `r=b-Ax` with extended-precision accumulation. If the unchanged canonical
limit is missed, it performs one defect-correction solve `B Delta x_B=r` and
rechecks the same limit. Failure is numerical; neither limit is enlarged.

Exact DSE initialization is a factor-level batch: one existing INVERT is reused
for all unit BTRAN right-hand sides. Each right-hand side is checked against the
unchanged backward-error bound and may receive one refinement with the same
factor. No row is allowed to trigger a same-basis INVERT. A failed batch returns
one driver-visible rebuild request.

## 7. Edge-weight frameworks

The framework is selected from owned state, not by a tolerance change:

- a logical/diagonal basis uses exact unit (or scaled diagonal) DSE weights;
- a warm basis carrying matching cached DSE weights continues DSE;
- a nonlogical warm basis without weights starts a Devex framework, avoiding
  `m` startup BTRAN solves.

In DSE mode, each selected leaving row computes `pi=B^-T e_p` and exact weight
`||pi||_2^2`.

An underestimated recursive weight can make a row falsely attractive. Hence a
selected row whose stored weight is materially below the exact weight is put
back into CHUZR with the exact value. This reselects a row, not an entering
pivot, and follows the DSE merit definition.

The DSE update remains the Goldfarb-Reid recurrence with definition-based
reconstruction on cancellation. For Devex, let `R` be the variables basic when
the framework starts. For pivotal row `p`, the exact reference weight is

```text
w_p^R = max(1, sum_{j in R} (e_p^T B^-1 a_j)^2).
```

After a pivot with column `a_q`, `u=B^-1 a_q`, and pivot `alpha=u_p`, the
advisory weights are updated by

```text
w'_p = max(1, w_p^R / alpha^2)
w'_i = max(w_i, w'_p u_i^2), i != p.
```

If the ratio between the advisory and exact pivotal reference weight exceeds
the squared factor `9`, the next basis becomes a new deterministic reference
framework. This changes pricing only; feasibility and optimality tolerances are
untouched.

## 8. Harris BFRT and anti-cycling

For every remaining nonbasic column define signed pivot `alpha_j > 0`, signed
reduced cost `sigma_j=d_j r_j`, and exact breakpoint
`theta_j=max(0,-sigma_j)/alpha_j`. BFRT first consumes complete earlier
finite-range breakpoint groups. Harris pass one is constrained by every
positive signed tableau coefficient, including coefficients too small to be an
entering pivot, and computes

```text
theta_H = min_j (tau_d - sigma_j) / alpha_j
        = min_j (tau_d - d_j r_j) / alpha_j.
```

Pass two selects the largest `alpha_j` among candidates satisfying
`theta_j <= theta_H`, with column order as the deterministic final tie-break.
The published terminal audit still uses `tau_d`; Harris does not relax it.

The cycle signature contains the ordered basis and every nonbasic bound side.
When that state repeats, the previously recorded outgoing `(leaving, entering)`
exchange is taboo for `2m+1` committed pivots in that exact state. A taboo edge
may still participate in a bound flip; it is excluded only as the basis change.
If every Harris-eligible exchange for a leaving row is taboo, CHUZR temporarily
taboos that row instead of releasing an entering edge. If all infeasible rows
are taboo, CHUZR deterministically releases the earliest-expiring row. Thus
taboo metadata cannot create a false infeasibility conclusion, and the solver
does not retry another pivot from the same row. There is no pivot trial,
rollback, or tolerance change.

## 9. Minor-iteration transaction

One minor iteration is atomic:

1. choose a primal-infeasible row from the maintained infeasibility set;
2. recompute and verify its exact edge weight;
3. BTRAN the unit row and PRICE the pivotal row;
4. perform BFRT plus Harris two-pass selection, respecting scoped taboo edges;
5. FTRAN the BFRT RHS, pivotal column and DSE vector;
6. verify backward errors and the primal/adjoint residual identities;
7. materialize the complete candidate primal, reduced-cost and objective state;
8. update the basis factor, unless a refined pivotal solve or an over-limit
   incremental primal residual requires a new-basis INVERT;
9. commit basis membership, bound flips, primal state, dual state, objective,
   infeasibility set and weights together;
10. clear `fresh_rebuild` and, when required by step 8, immediately INVERT and
    reconstruct the committed new basis before further pricing.

No failure after step 6 may leave a partially committed state.

## 10. Terminal protocol

`PossiblyOptimal` and `PossiblyPrimalInfeasible` always return to the major
loop. A result can be published only if:

- the state is fresh;
- the working objective equals the original objective;
- no perturbation or shift is active;
- the canonical primal/dual audit passes unchanged tolerances;
- the original-space publication audit passes;
- infeasibility has a checked Farkas certificate.

Thus the stabilization objective may guide the basis sequence but can never
alter the reported model or certificate.

## 11. Required telemetry and tests

Statistics must expose:

```text
major_rebuilds, reinversions, FT updates, max updates between rebuilds,
rebuild reasons, iterative refinements, canonical primal corrections and
new-basis pivot reinversions, cost perturbation magnitude,
cost shifts, cleanup passes, DSE/Devex frameworks, Harris candidates, BFRT
flips, cycles, taboo insertions/rejections/releases,
max primal drift, max dual drift, max objective drift.
```

Required gates:

1. state-machine unit tests for every rebuild reason;
2. perturbation cleanup returns the exact original optimum;
3. a terminal candidate after updates forces rebuild;
4. incremental and rebuilt states agree within the unchanged audits;
5. HFactor/fresh-HFactor/dense-LU differential tests after update chains;
6. LP, numerical-stability, Netlib and MILP regressions;
7. all 3655 ACTIVSg2000 FMEA contingencies without conservative fallback;
8. same-machine old-native/new-native timing proving the 100x gate.

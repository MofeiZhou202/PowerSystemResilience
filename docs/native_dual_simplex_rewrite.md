# Native Revised Dual Simplex Rewrite

> Development-only: production modules default to `LpKernelBackend::HiGHS`.
> Select `ExperimentalNative` explicitly for the tests and benchmarks in this
> document. There is no production HiGHS-to-native fallback.

> Implementation status: the legacy primal/dual drivers have been removed
> from the native entry point. The cold revised-primal Phase I/II and warm
> revised-dual Harris kernel are active. Checked Phase-I Farkas certificates
> exact-breakpoint BFRT, and sparse Forrest-Tomlin basis updates are active.
> The HySim 3655-contingency gate passes without LP fallback. The 100x
> same-machine performance gate remains open.
> The major/minor stabilization contract is specified separately in
> `docs/native_dual_stabilization.md` and is now mandatory for completion.

## 1. Scope and non-goals

This document is the implementation contract for replacing the legacy native
dual-simplex driver. The new solver must not call the legacy reoptimizer, reset
to a crash basis after a numerical failure, increase an iteration budget to
hide a failed transition, or delegate an LP to HiGHS. HiGHS is a reference
implementation and a differential-test oracle only.

The rewrite is accepted only when every published solution satisfies the
canonical-space invariants below and the case-system regressions contain no
legacy warm path or cold Phase-I recovery.

## 2. Canonical problem and state

The native standard form is

```text
maximize    c^T x
subject to  A x = b
            0 <= x_j <= u_j
```

where `u_j` may be infinite. Artificial columns and zero-width columns have
effective bounds `[0, 0]` in Phase II. They are not ordinary lower-bounded
variables.

For a basis `B = A[:, beta]`, nonbasic variables are represented by a move
`d_j`:

```text
d_j = +1  when x_j is at its lower bound
d_j = -1  when x_j is at its finite upper bound
d_j =  0  when x_j is fixed or not eligible to enter
```

The reconstructed state is

```text
x_B = B^-1 (b - N x_N)
y   = B^-T c_B
r   = c - A^T y
```

For this maximization convention, nonbasic dual feasibility is

```text
d_j r_j <= tau_d.
```

## 3. Mandatory invariants

The following checks are contracts, not recovery heuristics.

1. `beta` has exactly `m` distinct valid columns.
2. FTRAN and BTRAN satisfy a normwise backward-error bound based on machine
   epsilon and the true basis matrix. A factor that fails this check is invalid.
3. Reconstructed values satisfy `||A x - b||_inf` in scaled canonical space.
4. Basic reduced costs are zero to the reconstruction accuracy.
5. Before every Phase-II pivot, all eligible nonbasic columns satisfy
   `d_j r_j <= tau_d`.
6. After every pivot and bound-flip transaction, invariants 1-5 are rechecked
   against an independent reconstruction while the correctness kernel is under
   development.
7. `Optimal` additionally requires primal bounds, artificial value zero, and
   the original-space residual/objective audit.
8. `PrimalInfeasible` requires a checked dual ray. CHUZC failure alone is not a
   certificate.

No result may be published from a shifted objective or Phase-I bound system.

## 4. One revised-dual pivot

Let row `p` be primal infeasible and let

```text
Delta_p = x_B[p] - violated_bound
s       = sign(Delta_p), so s is -1 below lower and +1 above upper
pi      = B^-T e_p
a_bar_j = pi^T A_j
```

A nonbasic column is eligible precisely when

```text
alpha_j = s d_j a_bar_j > 0.
```

Define its dual margin and exact breakpoint by

```text
mu_j    = -d_j r_j >= 0
theta_j = mu_j / alpha_j.
```

For a dual step `theta >= 0`,

```text
y'   = y - s theta pi
r'_j = r_j + s theta a_bar_j
d_j r'_j = -mu_j + theta alpha_j.
```

These equations are the executable pivot oracle. The selected entering column
must have `r'_q = 0`; any candidate with a smaller crossed breakpoint must be
flipped, changing `d_j` to `-d_j`. Candidates at or after the selected
breakpoint stay on their current side.

The leaving column is placed at the violated bound. Its new move is `+1` for a
lower-bound departure and `-1` for an upper-bound departure, which gives
`d_leave r'_leave = -theta <= 0`.

### 4.1 Floating-point eligibility

A fixed absolute pivot threshold is not part of the mathematical algorithm.
For each priced entry, compute the dot-product error bound

```text
eta_j = gamma_k sum_i |pi_i A_ij| + solve_error_contribution,
gamma_k = k eps / (1 - k eps).
```

The sign of `alpha_j` is trusted only when its lower error bound is positive:

```text
s d_j a_bar_j - eta_j > 0.
```

Otherwise the entry is numerically unresolved. The row is repriced after a
fresh factorization; if it remains unresolved, the solver returns a numerical
status. It does not blacklist the column or lower a pivot threshold.

The independently computed row and column pivots are not compared with a
fixed multiple of epsilon. With `v=B^-1 A_q`, `pi=B^-T e_p`, and the measured
solve residuals

```text
r_v  = A_q-Bv,
r_pi = e_p-B^T pi,
```

the exact a-posteriori identity is

```text
pi^T A_q-e_p^T v = pi^T r_v-r_pi^T v.
```

The implementation evaluates both residual terms in extended precision and
accepts the pivot pair only when this identity encloses their discrepancy.
This scales with the actual two solves and does not relax a feasibility or
optimality tolerance.

## 5. CHUZR

CHUZR uses dual steepest-edge merit

```text
merit_p = primal_infeasibility_p^2 / max(weight_p, machine_safe_min).
```

An updated DSE weight is advisory only. Before pricing the selected row, BTRAN
recomputes `pi` and the exact weight `||pi||_2^2`; an underestimated cached
weight cannot select an invalid row.

For entering pivotal column `v = B^-1 A_q`, pivot `alpha = v_p`, and
`rho = B^-1 B^-T e_p`, the exact Goldfarb-Reid update is

```text
w'_p = w_p / alpha^2
w'_i = w_i - 2 (v_i/alpha) rho_i + (v_i/alpha)^2 w_p,  i != p.
```

This follows by writing the exchanged basis as `B' = B E`, where column `p`
of `E` is `v`. Hence `B'^-1 = E^-1 B^-1`, so
`pi'_p = pi_p/alpha` and
`pi'_i = pi_i-(v_i/alpha)pi_p`. The revised-dual driver applies this update,
clamps only negative roundoff to a machine-error lower bound, and recomputes
the exact selected-row weight before the next CHUZC.

When the three-term Goldfarb-Reid recurrence suffers cancellation, a negative
or unresolved cached value is not an LP failure. For that row the driver
computes `pi_i=B^-T e_i` and forms
`||pi_i-(v_i/alpha)pi_p||_2^2` in extended precision. This is the defining
post-exchange DSE weight, not a clipped or relaxed recurrence.

The DSE cache is owned exclusively by the revised-dual phase. Cold Phase I and
Phase II use revised-primal pricing, which does not read a row-edge weight.
Maintaining Goldfarb-Reid weights during those phases would therefore be dead
state: it adds one FTRAN and an `O(m)` recurrence to every primal pivot, and a
cancelled recurrence could incorrectly terminate an otherwise valid primal
pivot sequence. The primal driver still computes the BTRAN required by the
   Forrest-Tomlin basis exchange, but it does not numerically maintain
`edge_weight`; a basis exchange invalidates the cache and the exported basis
omits it. A warm revised-dual entry either reuses a basis-matched DSE cache or
starts the Devex framework defined in `native_dual_stabilization.md`; no stale
primal-phase cache can cross the phase boundary.

## 6. Harris CHUZC correctness kernel

The first correctness milestone uses a two-pass, single-step ratio test:

1. Build every eligible `(j, alpha_j, sigma_j)` without mutating state, where
   `sigma_j=d_j r_j` and `alpha_j>0` is the signed tableau coefficient.
2. Compute the limiting step from `(tau_d-sigma_j)/alpha_j` over every positive
   signed coefficient, including coefficients too small to enter.
3. Among columns admitted by that step, choose the largest stable `alpha_j`,
   with deterministic column ordering for ties.
4. Perform no long-step bound flips.

This was the enabling correctness milestone. Production now composes the same
two Harris passes with the BFRT prefix in Section 7, as specified by the
stabilization contract.

## 7. BFRT long step

BFRT is a separate ratio-test implementation, not an extension of Harris by
an enlarged threshold.  The first implementation uses exact ordered
breakpoints rather than HiGHS' decade expansion and backward large-pivot scan.
Those HiGHS rules are useful performance policies, but neither is required by
the bound-flipping theorem and both enlarge the proof surface.

For every error-certified candidate define

```text
t_j = max(0, -sigma_j) / alpha_j,   range_j = upper_j-lower_j.
```

Sort candidates by `(t_j, column_j)` and form a group only when the computed
`t_j` values are exactly equal.  This is deliberately not a tolerance-based
merge.  If `F` is the set of finite-range candidates in complete groups before
the selected group, the new move is `-d_j` for `j in F`.  Dual feasibility is
equivalent to the interval

```text
max_(j in F)     (-tau_d-sigma_j)/alpha_j <= theta
theta <= min_(j not in F) ( tau_d-sigma_j)/alpha_j.
```

Choosing `theta=t_q` from the selected exact-breakpoint group satisfies both
inequalities by construction: every flipped exact breakpoint is no larger
than `theta`, and every unflipped breakpoint is no smaller.  The implementation
still evaluates the interval explicitly; it never repairs a failed interval
by trying another pivot.

The primal part follows from a bound flip `Delta x_j=d_j range_j`:

```text
Delta x_B = -B^-1 A_j Delta x_j,
Delta x_B[p] = -s alpha_j range_j.
```

Thus complete groups reduce the leaving violation by
`sum alpha_j range_j`.  The transaction is:

1. Accumulate `alpha_j range_j` only for finite-range candidates in complete
   exact-breakpoint groups. An infinite-range candidate cannot be flipped, so
   its first breakpoint is an impassable dual-feasibility boundary.
2. Select the first group containing an infinite-range candidate, or the first
   finite group whose inclusion would cover the leaving-row violation. If all
   ranges are finite and their total change is insufficient, select no pivot
   and require the checked pivotal-row interval certificate.
3. Harris pass one computes the upper bound from every remaining positive
   signed coefficient. Pass two selects the largest error-certified `alpha_j`
   whose exact breakpoint is within that bound, with deterministic column order
   for ties.
4. Flip every finite-range candidate in complete groups strictly before the
   selected group.
5. Form the BFRT RHS `sum_j A_j d_j range_j`, apply one FTRAN, and materialize
   the complete post-pivot vector. If its incremental residual meets the fixed
   `Ax=b` limit, apply the captured FT update. Otherwise commit the verified
   exchange without that pack and immediately INVERT and reconstruct the new
   basis before further pricing.
6. Check the dual-feasibility interval and every analytical
   `d_j r'_j <= tau_d` before committing, then check them again in the
   post-pivot audit.

The ratio test returns a transaction object. Basis membership, bound sides,
primal values, and factor state are committed together or not at all.

Anti-cycling uses a signature of the ordered basis plus all nonbasic bound
sides. A repeated state taboos its recorded outgoing exchange for a finite
lifetime. If all Harris-eligible exchanges for one leaving row are taboo, CHUZR
taboos the row; only when every infeasible row is taboo does it release the
earliest-expiring row. It never searches alternative pivots from a failed row.

The validated state at loop entry is an induction invariant. Each transaction
materializes its complete candidate primal vector, checks every candidate
nonbasic reduced-cost sign, fixes all new basic reduced costs to their defining
zero, and either commits a checked FT transition or commits the basis exchange
followed immediately by new-basis INVERT and reconstruction. An incremental
vector above the fixed `Ax=b` limit is never used at the next loop entry. The
driver performs exact reconstruction plus a full audit before publication.

The first BFRT implementation deliberately omits both HiGHS' relaxed group
expansion and its backward scan to an earlier large-alpha group. They are not
enabled until a separate proof and differential test show that the resulting
shorter dual step and altered flip prefix preserve the interval above.

### 7.1 Pivot transaction pseudocode

```text
pivot(state):
  p, s, violation := CHUZR(state)
  pi               := checked_BTRAN(e_p)
  row              := checked_PRICE(pi, A)
  tx                := CHUZC_or_BFRT(state, p, s, row)
  require tx.dual_postcondition_holds_analytically

  aq                := checked_FTRAN(A[:, tx.entering])
  require row[tx.entering] agrees with aq[p]
  bfrt_delta        := checked_FTRAN(tx.bfrt_rhs) if tx.has_flips else 0

  candidate_state   := apply flips, basis exchange, and leaving bound
  candidate_factor  := update or fresh-factor candidate basis
  require candidate_state satisfies basis, equation, and dual invariants
  commit candidate_state and candidate_factor atomically

termination_candidate(state):
  exact_state := reconstruct(state, current_factor)
  require exact_state satisfies primal, dual, equation, and artificial invariants
  publish exact_state
```

## 8. Cold Phase I and simplex strategy

The earlier design attempted to copy only HiGHS' dual-phase-one bounds and run
the Phase-II dual pivot unchanged. That is not a valid algorithm. HiGHS' dual
Phase I also has a distinct dual-infeasibility objective, phase-specific ratio
logic, cost perturbation/shift cleanup, and rebuild transitions. In isolation,
the bounds do not make the inherited basis dual feasible and primal feasibility
of that auxiliary bound system does not prove original dual feasibility.

The correctness kernel therefore uses the standard deterministic strategy:

```text
warm basis with a dual-feasible bound-side assignment
    -> revised dual Phase II

cold logical basis
    -> certified singleton crash substitutions
    -> exact bound-side dual-feasibility classification
       -> feasible: revised dual Phase II
       -> impossible for this basis: revised primal artificial-objective
          Phase I, then revised primal Phase II with original costs
```

For a fixed basis, this classification is necessary and sufficient. A boxed
nonbasic column can select the endpoint whose move sign is compatible with its
reduced cost. A lower-only nonbasic column has no alternative upper endpoint,
so a positive reduced cost is precisely the obstruction. This check happens
before any pivot. It is not an attempted solve, and a failure after entering
the revised-dual branch is terminal rather than a fallback to primal Phase I.

For primal Phase I, artificial columns have bounds `[0,+inf)` and cost `-1` in
the canonical maximization convention; every other column retains its original
bounds and has zero Phase-I cost. The logical basis is primal feasible because
the standard-form builder orients every row to a nonnegative RHS. At a checked
primal/dual optimum,

```text
sum_j artificial_j = 0  -> original canonical system is feasible
sum_j artificial_j > 0  -> original canonical system is infeasible
```

The transition restores original costs, fixes artificial columns to zero,
reconstructs the same basis exactly, and verifies primal feasibility before
primal Phase II. No shifted objective is ever published.

Inside either revised-primal phase, a pivot is an incremental basis
coordinate transformation, not a request to solve the whole LP again. For
entering move `d`, step `t`, pivotal column `v=B^-1 A_q`, pivotal row
`z=B^-T e_p`, reduced cost `r_q`, and `alpha=v_p`, the committed state is

```text
Delta x_q = d t
x'_B      = x_B - v Delta x_q, with coordinate p replaced by x_q+Delta x_q
r'        = r - (r_q/alpha) A^T z, with r'_q set exactly to zero
objective'= objective + r_q Delta x_q.
```

These identities follow respectively from `Bx_B+A_q x_q=b` and from choosing
`y'=y+(r_q/alpha)z`, which makes the entering reduced cost zero. A bound flip
uses only the first and last identities because the basis and dual vector do
not change. The next invariant audit checks the same `Ax=b`, active bounds,
and basic reduced-cost thresholds as before; no tolerance or pivot rule is
relaxed. Full reconstruction remains a phase-transition and reinversion
operation, not an `O(nnz(A))` pair of solves on every primal pivot.

The singleton crash is not a heuristic search. A structural column may replace
the logical at row `i` only when it has exactly one matrix nonzero, that nonzero
is in row `i`, and `b_i/a_ij` lies in the column's original bounds. Therefore
all replacements are independent diagonal substitutions and preserve full
rank by construction. Candidate order is deterministic.

A warm basis that cannot be made dual feasible by legitimate original bound
sides is rejected as `DualInfeasibleStart`; the solver does not discard it and
silently perform a cold restart. A future warm Phase-I algorithm requires its
own proof and tests rather than reintroducing isolated HiGHS phase-one bounds.

## 9. Certificates

When CHUZC has no eligible column, the pivotal row is only a candidate primal
infeasibility certificate. Let `pi = B^-T e_p`, `h = pi^T b`, and
`a_j = pi^T A_j`. Compute the attainable interval

```text
L = sum_j min(a_j l_j, a_j u_j)
U = sum_j max(a_j l_j, a_j u_j),
```

with the appropriate infinite-endpoint rules. Since every feasible point must
satisfy `pi^T A x = h`, primal infeasibility is certified only if `h < L` or
`h > U` by more than the certified accumulation error. The certificate stores
`pi`, the interval endpoint, its error bound, and the separating margin.

A Phase-I optimum with unresolved original dual infeasibility is not sufficient
by itself to report primal unboundedness or dual infeasibility. Until a checked
dual ray is implemented, that outcome is `NumericalFailure`, never `Optimal`.

### 9.1 Cold and warm share state and factor contracts

Cold and warm paths use the same basis ownership, factorization, reconstruction,
residual audit, iteration budget, and result publication. Their mathematically
required pivot algorithms differ as described in section 8. There is no legacy
dense driver, numerical retry level, Big-M objective, or HiGHS runtime fallback
inside the native path.

## 10. Basis factorization and updates

Fresh rank-revealing factorization remains the reinversion oracle. HFactor
identifies no-pivot rows and minimally replaces those basis columns with their
logicals. Every FTRAN/BTRAN is checked against the explicit current `B`; a
measured backward-error failure receives one refinement with the same factor.
If that is still unacceptable, the factor returns structured evidence and the
major driver owns any INVERT. It never changes a simplex decision or tolerance.

For one exchange, with `v=B^-1 A_q`, the new basis is

```text
B' = B E_p(v),
```

where `E_p(v)` is identity with column `p` replaced by `v`. Forrest-Tomlin is
an algebraically equivalent sparse reorganization of the product-form
operators `E_p(v)^-1` and `E_p(v)^-T`. The update test compares its action
with fresh `B'` solves; update count, fill, or a backend success flag cannot
replace this comparison.

The factor layer represents the current basis by `(A, basis_indices)`; it does
not copy an Eigen sparse `B`, rebuild `B^T`, or rescan all basis nonzeros after
every exchange. Backward residuals multiply directly by the selected columns
of `A`. Row absolute sums are changed by subtracting the leaving column and
adding the entering column, while column sums change at position `p`; the
induced infinity norms used in the unchanged backward-error bound are then
reduced from these arrays. This preserves the exact residual and norm
definitions while removing basis materialization from the pivot path.

All kernel sparse matrix-vector products use a deterministic serial CSC loop.
For the target LPs (about ten thousand columns and only tens of thousands of
nonzeros), dispatching an OpenMP team for every PRICE or residual product costs
more than the arithmetic and introduces scheduling variance. This changes
neither summands nor acceptance bounds.

The active representation is HFactor's sparse Forrest-Tomlin update. HFactor
permutes caller basis positions into pivot-row positions during INVERT. The
wrapper therefore maps the leaving position and the final FTRAN vector into
that internal basis-coordinate space, while the BTRAN result stays in physical
constraint-row space. It also updates HFactor's internal `basic_index` before
the exchange.

FT does not consume only the final vectors `B^-1 A_q` and `B^-T e_p`. Its
packed update operands are intermediate triangular-solve states: the pivotal
column pack is captured after the L solve and before the U solve, and the
pivotal-row pack is captured after the transposed U solve and before the
transposed L solve. Repacking the final vectors is algebraically wrong and was
the cause of the earlier multi-update divergence. The wrapper captures both
intermediate packs during the original solves and binds them to a factor
generation. If a pivotal solve needs iterative refinement, the raw captured
pack is stale. The verified exchange is committed without applying that pack,
then the driver immediately INVERTs and reconstructs the new basis. The same
new-basis protocol is used when the materialized incremental transaction misses
the unchanged canonical `Ax=b` limit. No pivot, bound side, tolerance, or
iteration budget changes, and the over-limit state is never used for pricing.

Reconstruction separately closes the gap between factor backward stability and
the canonical primal contract: it accumulates `r=b-Ax` in extended precision,
applies one defect correction `B Delta x_B=r` only when required, and rechecks
the original limit.

The mandatory differential tests start from a non-diagonal, permuted basis,
perform ten exchanges, and after every exchange compare FT FTRAN/BTRAN with
both a fresh HFactor factorization and dense LU. They also repeatedly reuse a
single factor object across fresh factorizations. These tests distinguish the
three coordinate spaces and the intermediate-pack contract that unit/diagonal
bases cannot exercise.

The private native HFactor port does not apply HiGHS' `kHighsTiny` sparsifying
policy inside triangular solves, FT replay, or LU fill cancellation. A value is
removed only when it is exactly zero. The embedded HiGHS solver is unchanged.
This is required because the native kernel verifies solves against the true
explicit `B`; silently dropping a nonzero changes the represented linear
system and cannot be justified by increasing the backward-error threshold.

## 11. Test gates

The implementation proceeds through these mandatory gates:

1. Algebraic pivot tests compare analytical `r'` and `x_B'` with exact
   reconstruction for lower and upper departures.
2. Harris tests cover ties, degeneracy, free/lower-only/boxed/fixed columns,
   and tiny but valid pivots.
3. BFRT tests contain at least three breakpoint groups and verify the exact
   flip set and BFRT RHS.
4. Phase-I tests cover lower-only dual infeasibility, boxed-variable side
   selection, artificial basics, infeasible LPs, and transition to original
   bounds.
5. Random small LP/property tests compare status, objective, primal residual,
   and dual feasibility against HiGHS across many valid warm bases and bound
   changes.
6. Existing dual-simplex, numerical-stability, LP, MILP, and Netlib suites pass
   with the legacy warm path disabled.
7. HySim reliability case-system tests pass without conservative fallback.
8. `ACTIVSg2000 / 2000 AC + 8 DC` completes all 3655 FMEA contingencies with
   no `Simplex Phase I failed`, no legacy `[PATH A]`, and no cold recovery for
   a warm contingency LP.
9. The active FT update chain remains enabled only while its generation-bound
   intermediate-pack differential tests pass. The final performance gate is
   at least 100x over the old native flagship path, measured on the same build
   and hardware.

## 12. Frozen algorithm decisions

The initial rewrite has the following fixed decisions:

1. Maximization reduced-cost convention and move signs are those in sections
   2-4; conversions occur only at the public API boundary.
2. Exact-breakpoint BFRT is active. Fresh factorization is the correctness
   oracle; the active Forrest-Tomlin update is checked by the same
   explicit-basis backward-error gate.
3. Cold initialization uses the standard artificial-objective revised-primal
   Phase I. Warm revised-dual entry requires a dual-feasible original-bound
   side assignment. Deterministic Phase-II cost perturbation and one-sided cost
   shifts belong to revised-dual stabilization only and must be removed by the
   original-cost cleanup before publication.
4. BFRT is enabled only after the Harris kernel passes gates 1-7, and uses the
   complete-group transaction in section 7.
5. Sparse FT and BFRT are algebraically independent and each retains its own
   transition and differential tests.
6. Every failure status is evidence-bearing. A missing certificate is a
   numerical/unknown outcome, not an infeasibility verdict.

Changes to these decisions require updating this document and adding the test
that justifies the change before implementation.

## 13. Implementation boundaries

The rewrite is split by mathematical ownership rather than accumulated helper
functions:

```text
native_dual/model.*          canonical bounds, moves, basis membership
native_dual/factor.*         rank repair, FTRAN/BTRAN, update verification
native_dual/state.*          exact reconstruction and invariant audit
native_dual/pricing.*        CHUZR, row pricing, Harris and BFRT transactions
native_dual/primal.*         cold primal Phase I/II and Harris ratio test
native_dual/certificate.*    checked primal/dual rays
native_dual/solver.*         phase controller, limits and result publication
```

The public `StandardFormLP`, `SimplexBasis`, `SimplexOptions`, `SimplexResult`,
and `BasisOps` contracts remain in the existing API header. The legacy
`sparse_dual_simplex_reoptimize`, retry/rollback/blacklist controllers, and
duplicate dense warm paths are deleted after the replacement passes the
correctness gates; they are not retained as fallback branches.

The controller state machine is fixed as:

```text
Input
  -> ValidateBasis / BuildLogicalBasis
  -> RankRepairAndFactor
  -> ExactReconstruction
  -> Warm? -> OriginalDualFeasibility -> RevisedDualPhaseII
  -> Cold? -> SingletonCrash -> PrimalPhaseI -> PrimalPhaseII
  -> OriginalSpaceAudit
  -> PublishResult
```

Every arrow has a typed failure result. None points to the legacy driver or
back to `BuildLogicalBasis` as a retry.

## 14. Reference mapping

The corresponding HiGHS sources are:

```text
CHUZR and DSE validation    highs/simplex/HEkkDualRHS.cpp, HEkkDual.cpp
Harris/BFRT CHUZC           highs/simplex/HEkkDualRow.cpp
Primal Phase-I objective    highs/simplex/HEkkPrimal.cpp
Dual Phase-I non-equivalence highs/simplex/HEkk.cpp::initialiseBound,
                             highs/simplex/HEkkDual.cpp::solvePhase1
FTRAN/BTRAN/update          highs/simplex/HSimplexNla.cpp, highs/util/HFactor.cpp
```

The native code may simplify data structures, but it must preserve the
mathematical state transitions and postconditions documented above.

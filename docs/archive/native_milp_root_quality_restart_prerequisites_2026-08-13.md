# Native MILP Root Quality: Restart Prerequisites

## Rationale

Model/algorithm: HiGHS root RENS followed by the inactive-integer restart
fixed point.  RENS fixes a target fraction of root-LP-integral integer
variables, preserves a residual integer frontier, and solves the restricted
MIP with finite node, leaf, stall, and wall-time limits.  A sufficiently good
incumbent activates objective-cutoff and reduced-cost propagation; HiGHS then
re-presolve the root whenever the inactive-integer rate crosses its restart
threshold.

Claim: the current `native-highs-lp` profile omits the first prerequisite.  It
labels the native RENS slot as replaced by `solveSubMip`, but does not invoke
the existing HiGHS-style sub-MIP helper because
`enable_root_low_fractionality_rens` remains false.  Enabling that slot can
improve the root incumbent without changing the root relaxation, branching,
or tree policy.

Cost model: after native separation, `p200x1188c` has 24 fractional binaries
among 1188 binaries.  The existing helper uses at most 200 sub-MIP nodes, 500
leaves, 12 non-improving nodes, and the smaller of 3 seconds or 5% of the
remaining solve time.  With the vendored HiGHS backend this is one restricted
MIP invocation and no new persistent storage.

Prediction: on the fixed 30-second, one-node `p200x1188c` diagnostic, RENS
finds an audited incumbent no worse than the upstream pre-RENS incumbent
`15747` within `0.50 s`; the target is the known optimum `15078`.  The native
root bound remains valid and no worse than `13544.7`.  On the fixed five-second
`neos-1122047` diagnostic, the root phase returns normally, any incumbent
audits at objective `161`, and total solve time is below `3.5 s`.

Assumptions: the native RENS bounds match the HiGHS neighbourhood closely
enough on these low-fractionality roots; the existing vendored sub-MIP helper
honours its limits; and the retained presolve/postsolve audit remains the
authoritative feasibility gate.

References: Achterberg (2007), Section 9.4; Berthold (2006), Section 3.5;
HiGHS `HighsPrimalHeuristics::RENS`, `solveSubMip`, and
`HighsMipSolverData::performRestart`; baseline root traces
`/tmp/p200_highs_root.log`, `/tmp/p200_native_root.log`,
`/tmp/neos_highs_root.log`, and `/tmp/neos_native_root.log`.

## Validation Contract

Baseline commit `1b51f0bd8caa`, macOS ARM64, Release `-O3 -DNDEBUG`, one
thread.  Build and run:

```sh
cmake --build build/macos-release \
  --target test_branch_and_cut test_milp_solver miplib2017_benchmark -j4

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case p200x1188c \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-verbose

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case neos-1122047 \
  --time-limit 5 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-verbose
```

A wrong-sign incumbent result, a root bound below `13544.7`, an incumbent
audit failure, or a runtime beyond either fixed ceiling triggers the mismatch
protocol before another algorithm edit.

## Root-Only Diagnosis

Before this experiment, upstream HiGHS solved `p200x1188c` at one node.  It
found incumbent `15078` through source `L`, then restarted repeatedly as the
inactive-integer fraction crossed 29.2%, 16.4%, 13.9%, 12.0%, 26.5%, 18.6%,
60.2%, and 11.4%.  Its working model shrank from 1188 to 39 binaries.  Native
performed 23 separation rounds on the unchanged 2376-column model, stopped at
root bound `13544.7974832`, and found only incumbent `55424`.

For `neos-1122047`, both solvers obtained root LP bound `161`, equal to the
integer optimum.  Upstream found incumbent `161` at the root through central
rounding.  Native spent `14.639 s` in its objective pump without finding an
incumbent in the 30-second diagnostic.  These measurements locate both
failures in root processing rather than branch selection.

## First Measurement And Re-Derivation

Enabling the existing low-fractionality RENS slot succeeded on
`p200x1188c`.  The restricted HiGHS sub-MIP found the exact incumbent `15078`
in 23 ms, versus the predicted ceiling `15747` and `0.50 s`.  The independent
audit passed.  Incumbent-cutoff propagation fixed or tightened 1915 bounds
and lifted the root lower bound from `13544.7974832` to `13930.54148`, so the
one-node gap fell from 75.6% (against incumbent `55424`) to 7.61%.

The `neos-1122047` result had the wrong sign: it still reached the seven-second
hard watchdog instead of the predicted 3.5-second ceiling.  The trace entered
the RENS slot but never reached the bounded sub-MIP invocation.

Mismatch investigation:

1. Implementation infidelity: confirmed.  The 5%-of-remaining-time budget is
   passed to `solve_root_submip_reusing_parent_state`, but the preceding RENS
   neighbourhood dive can perform up to 12 full root LP solves without using
   that budget as a deadline.  On the 57,611-row `neos-1122047` root, the
   unbudgeted construction consumes the solve allowance first.
2. Machine/cost-model error: the model treated the entire RENS slot as one
   bounded sub-MIP, while the implementation has a separate large-LP dive.
3. Assumption violation: the preliminary LP rounds are cheap on the
   1,388-row `p200x1188c` model but not on `neos-1122047`.
4. Theory error: standard root RENS does not require repeated LP solves when
   enough integer variables are already integral in the root relaxation.  It
   may fix those variables directly to their root values and leave the
   fractional residual for the sub-MIP.

The corrected neighbourhood construction fixes root-LP-integral variables
directly until the 60% target is reached and enters the bounded sub-MIP
without a redundant relaxation solve.  Both cases have enough integral root
binaries for this path: 1164 of 1188 on `p200x1188c` and 87 of 100 on
`neos-1122047`.  The original validation thresholds remain unchanged.

## Second Measurement And Deadline Re-Derivation

The corrected construction removed every preliminary RENS LP.  On
`p200x1188c`, the first restricted sub-MIP again found the exact audited
incumbent `15078`, now in 24 ms.  Cutoff propagation tightened 1915 bounds,
the root lower bound remained `13930.54148`, and the one-node run took
`1.371 s`.

On `neos-1122047`, all three neighbourhoods entered their bounded sub-MIPs:
the 60%, 30%, and 15% fixing attempts took 209, 218, and 198 ms in the timed
trace.  None found an incumbent, and the process still reached the seven-second
watchdog.  This again violates the predicted `3.5 s` ceiling, so no further
root algorithm change is admissible until the mismatch is explained.

Mismatch investigation:

1. Implementation infidelity: confirmed in the phase following RENS.  The
   standalone root domain-probing scope checks the optional-root deadline only
   before entry, then can run two propagation worlds for each of 100 binaries.
   It has no internal deadline poll.  The last trace line before the watchdog
   is the third completed RENS attempt.
2. Machine/cost-model error: the earlier model counted probing as a small
   number of LP solves.  This phase instead initializes an incremental domain
   and costs `O(nnz + W C_closure)`, where `W` is the number of completed probe
   worlds and `C_closure` depends on the active row, clique, and implication
   sources.  The `neos` trace has 444579 such sources after root separation.
3. Assumption violation: the existing `n_unfixed_bins <= 1000` gate controls
   only the number of worlds.  It does not control the cost of a propagation
   closure in a 57611-row root.
4. Theory error: probing deductions remain valid when the phase stops between
   worlds; an incomplete schedule is weaker, not incorrect.  A completed
   `x_j=0` or `x_j=1` world is therefore the correct cancellation granularity.

The corrected budget is the smaller of 5% of the solve limit and the time
remaining before the existing post-root finalization reserve.  The deadline is
polled before each binary and between its down/up worlds, so only completed
worlds export deductions.  For the five-second `neos` gate this gives 0.25 s;
including one in-flight world, probing is predicted below 0.35 s and the full
run below `3.5 s`.  Stateful initialization is assumed below 0.1 s at 163120
matrix nonzeros.  `p200x1188c` already has an incumbent and does not enter this
domain-probing loop, so objective `15078`, root bound at least `13930.5`, and
runtime within 5% of `1.371 s` are predicted.

References: Savelsbergh (1994), Sections 3-4; Achterberg (2007), Section 3.3;
the atomic savepoint/restore and completed-world contract in `BCDomainProbe`;
and the fixed validation commands above.

## Third Measurement And Entry-Gate Re-Derivation

The completed-world deadline removed the hard watchdog failure, but the
`neos-1122047` run took `4.779 s`, still above the fixed `3.5 s` prediction.
The root returned normally at the five-second limit with lower bound `161`, no
incumbent, and no processed tree node.

The trace disproved two assumptions in the preceding cost model.  Constructing
the stateful probing workspace and performing its mandatory baseline closure
took `0.538 s`, already more than the `0.25 s` phase allowance, so zero probe
worlds completed.  The enclosing standalone-probing scope then spent another
`1.951 s` generating, propagating, solving, and rejecting 128 graph implied-
bound rows.  It tightened 19993 trial bounds but left the root lower bound
unchanged at `161`.  Total scope time was `2.489 s`.

Mismatch investigation:

1. Implementation infidelity: the world-level poll is correct, but the scope
   has a mandatory initialization before that poll and owns additional
   implied-bound/reduced-cost stages after the domain loop.
2. Machine/cost-model error: the assumed sub-0.1-second initialization was
   wrong by more than 5x.  The measured baseline closure over 57611 rows and
   163120 nonzeros costs 0.538 seconds on the validation machine.
3. Assumption violation: a phase-local deadline cannot bound work that occurs
   before the first cancellable unit or in coupled tails outside its loop.
4. Theory error: an anytime probing phase should not be entered unless its
   first atomic result fits its allowance.  Skipping it is valid because
   probing only strengthens the relaxation/domain; it is not required for
   correctness.

The corrected entry gate skips the complete standalone-probing scope on roots
with more than 10000 rows when the solve limit is at most 10 seconds.  These
are the existing repository definitions of a large root and a short budget;
the gate applies regardless of basis ownership because domain-workspace
initialization does not consume the simplex basis.  The predicted
`neos-1122047` runtime is `2.9-3.2 s`: the measured pre-scope work was about
`2.29 s`, the bounded RENS attempts about `0.61 s`, and the 2.489-second scope
is removed.  Its root bound remains `161`.  `p200x1188c` has only 1388 rows,
so its prior objective, bound, and 5% runtime prediction remain unchanged.

## Root Primal Conformance: Central Rounding

The corrected standalone-probing gate matched its root-phase prediction:
`neos-1122047` completed root processing in `2.901 s`, inside the predicted
`2.9-3.2 s` range, with lower bound `161`.  The total run nevertheless used
the full five seconds because no root incumbent existed; its one permitted
child consumed the remaining time in domain closure.  This separates the
remaining defect from the deadline correction: the root dual bound is already
the integer optimum, but native is missing HiGHS's root primal certificate.

Upstream reports solution source `C`.  Its implementation computes an analytic
center concurrently with root separation, then calls
`linesearchRounding(firstlpsol, analyticCenter)`.  Each line-search breakpoint
fixes rounded integer variables, propagates the local domain, and solves a
continuous repair LP.  Native has the corresponding implementation, but its
`short_budget_root` gate suppresses analytic-center computation whenever the
limit is at most ten seconds.  The so-called central-rounding driver then uses
a nearest-integer copy of the root LP as its endpoint.  That endpoint has the
same rounded integer assignment as the start, so it does not represent the
HiGHS central-rounding neighborhood.

The corrected algorithm permits the existing background analytic-center solve
on the automatic HiGHS root profile even under a short budget.  Its zero-
objective, no-crossover IPM is still capped at the smaller of 100 iterations
and half of the remaining wall time and runs concurrently with the measured
roughly 2.0 seconds of root LP and separation work.  Prediction: on
`neos-1122047`, it finishes by the post-cut join or adds at most 0.4 seconds of
join time; central rounding finds an audited incumbent at objective `161`,
closes the root, and total runtime is below `3.5 s`.  Propagation is expected
to reject most of the at most 64 line-search breakpoints before a repair LP.
The 30-second `p200x1188c` run already enables this background task, so its
objective, bound, and runtime predictions are unchanged.

References: HiGHS `HighsMipSolverData::startAnalyticCenterComputation`,
`finishAnalyticCenterComputation`, and
`HighsPrimalHeuristics::{centralRounding,linesearchRounding}`; Achterberg
(2007), Section 9.2.

## Central-Rounding Backend Mismatch

Enabling the background task on `neos-1122047` had the wrong sign: the native
IPM did not finish before the seven-second process watchdog, and the mandatory
post-cut join blocked before central rounding ran.  No incumbent or root result
was returned, versus the predicted audited incumbent `161` below 3.5 seconds.

Mismatch investigation:

1. Implementation infidelity: confirmed.  HiGHS computes this point with its
   IPX solver, `run_centring=true`, presolve and crossover disabled, and a
   200-iteration cap.  Native used `NativeIPMLPAdapter`, which is an ordinary
   primal-dual LP solve without IPX's centring mode.  The two procedures do not
   implement the same analytic-center algorithm.
2. Machine/cost-model error: the prediction treated a native-IPM iteration as
   comparable to an IPX centring iteration and assumed the adapter's nominal
   time limit would bound the background task.  Neither assumption held on the
   57611-row system.
3. Assumption violation: concurrency only hides work that completes before
   synchronization; an uncancellable background solve converts the join into
   an unbounded serial tail.
4. Theory error: not indicated.  Upstream IPX central rounding finds the exact
   incumbent on this same reduced model.  The defect is backend fidelity.

The corrected implementation uses the linked vendored HiGHS/IPX LP algorithm
for this background task, with presolve off, zero objective, continuous
integrality, `run_centring=true`, crossover off, at most 200 iterations, and a
wall limit equal to half the solve time.  Prediction remains an audited root
incumbent `161` and total runtime below `3.5 s`; the IPX task must itself return
by 2.5 seconds even if it fails to produce a center.  `p200x1188c` is expected
to retain incumbent `15078` and lower bound at least `13930.5`.

## Central-Rounding Start-Point Mismatch

The IPX correction satisfied its operational prediction: on `neos-1122047`
the analytic center was available with zero join wait, and root processing
finished in `2.933 s`.  The primal prediction still failed.  Central rounding
tried 12 assignments, all rejected during domain propagation before a repair
LP, so there was no incumbent and the one child again consumed the remainder
of the five-second limit.

Mismatch investigation:

1. Implementation infidelity: confirmed in the line-search start point.
   HiGHS calls `linesearchRounding(firstlpsol, analyticCenter)`.  Native retains
   that certified initial point as `root_lp_source_x`, but constructs every
   convex combination from the mutable post-propagation/post-cut
   `root.x_relax`.  Root source processing has replaced that vector several
   times before the heuristic runs.
2. Machine/cost-model error: resolved for analytic-center computation; IPX
   completed concurrently as predicted.
3. Assumption violation: equal LP objectives do not imply equal primal points
   on this highly degenerate relaxation.  Rounding breakpoints depend on the
   coordinates, not only on the bound `161`.
4. Theory error: none.  The HiGHS algorithm explicitly preserves and uses the
   first LP solution while applying each rounded assignment to the current
   propagated domain.

The corrected line search uses `root_lp_source_x` as its first endpoint when
it has the expected dimension and retains the current `root.lb/root.ub` and
`repair_domain` for proof-valid fixing and propagation.  Prediction remains an
audited root incumbent `161` and total runtime below `3.5 s`; no more than the
observed 12 breakpoint assignments should be required.  `p200x1188c` remains
subject to the existing incumbent and bound gates.

## Central-Rounding Feasibility Oracle

Using the preserved first LP point did not change the measurement: all 12
central-rounding assignments were again rejected before a repair LP, root
processing took `2.935 s`, and no incumbent was found.  The start-point
prediction therefore failed, although the IPX center and runtime predictions
continued to hold.

Mismatch investigation:

1. Implementation infidelity: the retained first point was used as derived,
   but native treats an incremental `BCDomain` propagation rejection as the
   final feasibility decision.  HiGHS's theoretical `tryRoundedPoint` contract
   is a fixed-integer continuous LP feasibility test; propagation is a valid
   accelerator only when its infeasibility result is numerically reliable.
2. Machine/cost-model error: no additional error observed; the 12 rejections
   themselves took negligible reported time.
3. Assumption violation: the derivation assumed native and HiGHS domain
   propagation accepted the same rounded assignments.  Native's domain has
   undergone 11358 bound tightenings and uses a different activity arithmetic
   implementation, so this equivalence is not established.
4. Theory error: a direct fixed-integer LP is the authoritative oracle.  If it
   is feasible, the propagation rejection is conclusively a false negative;
   if infeasible, the rounded assignment is legitimately discarded.

The corrected central-rounding path falls back to a presolved vendored-HiGHS
continuous LP when native propagation rejects an assignment.  All integer
columns are fixed to the rounded values, so a feasible LP solution is a valid
MIP incumbent and still passes the existing independent audit before adoption.
The fallback is limited to central rounding in the automatic HiGHS root
profile and to the remaining optional-root budget.  Prediction: the first
upstream-equivalent feasible breakpoint yields audited objective `161` within
0.4 seconds, closing `neos-1122047` below `3.5 s`; at most 12 LP oracles are
attempted.  References: HiGHS
`HighsPrimalHeuristics::{tryRoundedPoint,linesearchRounding}` and Achterberg
(2007), Section 9.2.

## Lock-Direction Re-Derivation

The fixed-integer LP fallback rejected the same 12 assignments, establishing
that native propagation was not falsely pruning a feasible central-rounding
point.  The fallback added about 0.09 seconds; root processing took `3.007 s`
and still found no incumbent.  The feasibility-oracle prediction therefore
failed and the oracle should not remain on the production path.

The next source comparison found an exact algebraic discrepancy in the rounded
assignments.  For a row `l <= a x <= u`, increasing `x_j` is locked by the
upper side when `a_j > 0` and by the lower side when `a_j < 0`; decreasing it
is locked by the opposite sides.  Consequently, every nonzero coefficient in
an equality row contributes one up-lock and one down-lock.  HiGHS implements
these two independent row-side tests in `HighsMipSolverData::setup`.  Native
instead counted each equality coefficient once, assigning only an up-lock for
a positive coefficient or only a down-lock for a negative coefficient.

This implementation infidelity changes `linesearchRounding`: a variable with
native `uplocks[j] == 0` is always rounded upward and one with
`downlocks[j] == 0` always downward, bypassing the analytic-center breakpoint
rule.  On an equality-dominated model such as `neos-1122047`, these false
one-sided classifications explain why every generated assignment differs from
the upstream central-rounding path and is LP-infeasible.

The corrected lock computation treats the finite lower and upper side of each
inequality row independently and counts both sides of every equality.  The
fixed-integer oracle is removed because it proved no false propagation result.
Prediction: central rounding generates the upstream lock-directed assignments,
finds audited objective `161`, and closes the root below `3.5 s`.  Lock counts
remain `O(nnz)` and add no asymptotic or allocation cost.  `p200x1188c` retains
objective `15078` and root bound at least `13930.5`.

References: Achterberg (2007), Section 9.2; HiGHS
`HighsMipSolverData::setup` lock initialization and
`HighsPrimalHeuristics::linesearchRounding`.

## Structural-Lock Ownership

Correcting equality-row lock directions did not change the `neos-1122047`
result: central rounding still produced 12 propagation-infeasible assignments,
root time was `2.937 s`, and no incumbent was found.  Thus equality-side
undercounting was a real conformance defect but not the active cause here.

The remaining lock mismatch is ownership.  HiGHS initializes `uplocks` and
`downlocks` in `HighsMipSolverData::setup` from the presolved structural model,
before root separation.  Generated root cuts live in the cut pool and do not
retroactively alter those rounding locks.  Native computes locks from
`base_lp` after 64 generated root rows have been appended.  A generated row can
turn a structural zero-lock into a positive count, replacing HiGHS's forced
ceil/floor rule with nearest rounding and changing every subsequent line-search
assignment.

The corrected computation uses `root_lp`, the retained presolved structural
model, including its exact row-side bounds, while repair and incumbent audit
continue to use the current `base_lp`.  This is the same separation of
heuristic guidance from feasibility ownership as HiGHS.  Prediction remains an
audited incumbent `161` and root closure below `3.5 s`; `p200x1188c` retains
objective `15078` and bound at least `13930.5`.

## Central-Rounding Domain Ownership

Structural lock ownership also did not change the 12 rejected assignments.
The `neos-1122047` root finished in `2.920 s` without an incumbent, so the
prediction again failed.  Inspection shows the analogous ownership mismatch in
the feasibility accelerator itself: native initializes `repair_domain` from
the cut-augmented `base_lp`, while HiGHS copies its structural `domain` in
`tryRoundedPoint`; generated separation rows remain cut-pool objects and are
not part of that local structural propagation pass.

This distinction does not authorize ignoring cuts.  Structural propagation is
only a cheap rejection/closure stage.  Native's subsequent fixed-integer repair
LP is built from `base_lp`, and `adopt_root_incumbent` independently audits the
same current model, so every retained valid cut is still enforced before a
candidate can become an incumbent.  Using generated rows in the preliminary
domain can create numerically different activity-closure decisions without
strengthening the final feasibility proof.

The corrected `repair_domain` is initialized from `root_lp`, matching HiGHS's
structural-domain ownership, while repair and audit remain cut-augmented.
Prediction: at least one of the 12 central-rounding assignments reaches its
repair LP and yields audited objective `161`; total runtime remains below
`3.5 s`.  References: HiGHS
`HighsPrimalHeuristics::tryRoundedPoint`, `HighsDomain`, and Achterberg (2007),
Section 9.2.

## Immutable First-LP Ownership

Structural-domain ownership did not change the result: all 12 assignments were
still rejected, and root processing took `2.962 s`.  An environment-gated
coordinate trace then compared the algorithms directly.  At `alpha=0`, HiGHS
rounded
`0010100001001001...`, while native rounded
`0000001100000100...`; later breakpoint alphas differed only in the fifth
decimal place.  Thus the IPX analytic center is aligned, but native is not
using HiGHS's first root LP vertex.

The lifecycle audit found the implementation defect.  Native initializes
`root_lp_source_x` from the first LP solution but calls
`refresh_root_lp_source_solution()` after every accepted root resolve.  That
vector intentionally serves current separator/source generation.  Reusing it
as HiGHS `firstlpsol` made central rounding start from the final post-cut LP
vertex.  Identical objective `161` does not make those vertices interchangeable
in a degenerate relaxation.

The corrected ownership keeps `root_lp_source_x` mutable for separation and
stores a distinct immutable `first_root_lp_x` immediately after the certified
initial root solve.  Central rounding alone uses the immutable vector.  The
temporary coordinate traces are removed after identifying the first divergent
assignment.  Prediction: native's `alpha=0` assignment matches upstream,
central rounding finds audited objective `161`, and the root closes below
`3.5 s`.  References: HiGHS
`HighsMipSolverData::{firstlpsol,rootlpsol}` and
`HighsPrimalHeuristics::centralRounding`.

## Immutable First-LP Measurement Mismatch

The immutable first-LP correction did not satisfy its primal prediction.  On
`neos-1122047` with a five-second solve limit and one-node limit, all 12
central-rounding assignments were again rejected, root processing took
`2.946 s`, and no incumbent was found.  Total runtime was `4.858 s` after one
child consumed the remaining budget.  The root dual bound remained the exact
value `161`, so this measurement continues to isolate the defect to root
primal construction rather than branching or dual-bound quality.

Mismatch investigation:

1. Implementation infidelity: unresolved.  The ownership audit proves that
   native now reads an immutable initial-LP vector, but the coordinate trace
   was removed before verifying that this vector produces HiGHS's alpha-zero
   rounded assignment.  Storage lifetime alone does not establish numerical
   equivalence of the two initial LP vertices.
2. Machine/cost-model error: not indicated.  Root processing remained below
   the fixed `3.5 s` runtime gate and the analytic-center join remained free;
   only the predicted incumbent was absent.
3. Assumption violation: unresolved.  Native and HiGHS may obtain different
   optimal vertices on the degenerate root relaxation despite both reporting
   objective `161` and 5256 LP iterations.  Alternatively, the assignments
   may match while the propagated root domains differ.
4. Theory error: not yet indicated.  The next observation must distinguish
   endpoint divergence from domain divergence before another algorithm
   change is justified.

The fixed diagnostic is an environment-gated native trace of the rounded
binary assignment at each central-rounding breakpoint.  If native's
alpha-zero assignment differs from HiGHS's recorded
`0010100001001001...`, compare the unrounded first-LP binary coordinates and
root LP options.  If it matches, compare structural root-domain bounds and
the first propagation conflict for that identical fixing.  This trace is
observational only, emits at most one line for each of the observed 12
breakpoints, and must be removed after localization.  No algorithmic
prediction is changed until this diagnostic resolves the mismatch.

The diagnostic localized the mismatch to the initial LP endpoint.  After the
immutable-vector change, native still rounded at `alpha=0` to
`0000001100000100...`, not HiGHS's `0010100001001001...`.  Its later
breakpoints remained numerically aligned with the IPX endpoint (for example,
native `0.5092721242` versus HiGHS `0.5092630376`).  Therefore the stored
native vector is immutable as intended, but it is not numerically equivalent
to HiGHS's `firstlpsol`; root-domain propagation has not yet participated when
the divergent assignments are formed.

This resolves the mismatch as an assumption violation caused by root-LP
degeneracy: equal objective `161` and equal reported iteration count do not
identify a unique optimal vertex.  The next derivation must compare the first
100 binary coordinates and the LP algorithm/options/model ownership used by
native's direct root LP against HiGHS MIP's root LP.  The endpoint trace is
removed once that comparison identifies the differing contract.

## Pre-LP Domain Fixed-Point Oracle

The root lifecycle exposes a stronger endpoint-ownership difference.  HiGHS
computes structural row activities and calls `HighsDomain::propagate()` before
loading and evaluating the first root LP.  Native solves the presolved root LP
first, then performs its initial structural-domain propagation.  On
`neos-1122047`, that later pass reports 11358 tightenings, but the old optimum
remains inside the tightened box, so native correctly reuses it as an optimum
certificate.  Proof reuse is valid for the bound but does not reproduce the
vertex that a cold simplex solve selects from the tightened model.  Central
rounding is coordinate-sensitive, so these two ownership contracts are not
heuristically equivalent on a degenerate optimal face.

RATIONALE

Model/algorithm: HiGHS root initialization order: structural-domain fixed
point followed by a cold root LP solve, then immutable `firstlpsol` ownership.

Claim: a diagnostic cold solve after native's initial root-domain fixed point
will determine whether solve ordering, rather than simplex options or model
coordinates, causes the divergent central-rounding endpoint.

Cost model: native already pays for the propagation.  The oracle adds one
cold vendored-HiGHS LP solve on 57611 rows and 5080 columns, expected to repeat
approximately the observed 5256 simplex iterations and cost `0.6--0.8 s` on
the baseline macOS ARM64 Release build.

Prediction: the oracle's alpha-zero assignment matches HiGHS's recorded
`0010100001001001...`, central rounding produces the audited incumbent `161`,
and diagnostic root processing remains below `4.0 s`.  A mismatch instead
eliminates lifecycle ordering and requires a direct simplex-option/model
comparison before any production change.

Assumptions: native's structural propagation reaches the same relevant bound
box as HiGHS; the MIP-presolved model, objective, column order, and numerical
tolerances are otherwise equivalent; the cold LP is deterministic at seed 0.

References: HiGHS `HighsMipSolverData::setup` (domain activity computation and
propagation), `HighsMipSolverData::evaluateRootNode` (load/evaluate first root
LP and assign `firstlpsol`), and Achterberg (2007), Section 9.2.

Validation: baseline commit `1b51f0bd8caa`, macOS ARM64, Release
`-O3 -DNDEBUG`; run the fixed `neos-1122047`, five-second, one-node benchmark
with the environment-only post-propagation cold-solve oracle and coordinate
trace.  Compare alpha-zero bits, incumbent, root time, and LP iterations to
the prediction.  The oracle is removed after localization.

## Pre-LP Domain Fixed-Point Oracle Mismatch

The cold post-propagation solve did not reproduce HiGHS's endpoint.  It solved
the tightened native structural model in 5208 simplex iterations and
`0.607 s`, versus the predicted approximately 5256 iterations and
`0.6--0.8 s`, but its alpha-zero assignment remained
`0000001100000100...` (only one later bit changed from the original native
assignment), not HiGHS's `0010100001001001...`.  All 12 assignments were
rejected and no incumbent was found.  Diagnostic root time was `3.459 s`,
inside the predicted `4.0 s` operational gate.

Mismatch investigation:

1. Implementation infidelity: not indicated.  The oracle solved `root_lp`
   after `apply_node_bounds()` committed the completed native structural
   propagation, without a seed or basis.
2. Machine/cost-model error: none material.  Runtime landed inside the fixed
   range; the iteration estimate differed by less than one percent.
3. Assumption violation: confirmed.  Native's propagated root box and/or LP
   configuration is not equivalent to the state from which HiGHS obtains
   `firstlpsol`.  Matching dimensions, objective, and near-matching iteration
   count do not establish equality of bounds, coefficients, scaling, or
   simplex tie-breaking state.
4. Theory error: the proposed solve-order explanation is false for this
   instance.  Re-solving after the native fixed point does not recover the
   HiGHS vertex.

The cold oracle is removed.  The next fixed diagnostic prints, for every
integer column, the unrounded first-LP value and active bounds in native and
HiGHS, together with the effective simplex strategy, scale strategy, and
presolve setting.  This distinguishes model/domain divergence from simplex
tie-breaking without changing either solve.

## First-LP Coordinate Identity

The paired trace falsified both remaining endpoint hypotheses.  Native and a
direct `highs-mip` run produced bit-identical values for all 100 binary
columns, including every fractional coordinate, and both exposed bounds
`[0,1]`.  HiGHS reported effective `simplex_strategy=1`,
`simplex_scale_strategy=2`, and `presolve=off` at `firstlpsol` capture, matching
native's direct LP configuration.  For example, both paths returned
`x[4986]=0.99512666693059415`, `x[5029]=0.29911711449871514`, and
`x[5061]=0.67293320364491394`.

Therefore the alpha-zero assignment discrepancy cannot be attributed to root
LP bound quality, vertex selection, presolve, scaling, or simplex strategy.
At `alpha=0`, `linesearchRounding` differs from ordinary nearest rounding only
for a column with zero up-lock or zero down-lock.  The next fixed diagnostic
compares `(uplocks, downlocks)` for every integer column in the two structural
models.  Prediction: the first differing lock pair explains the first
different rounded bit.  If all pairs match, the earlier HiGHS coordinate trace
was not from the equivalent central-rounding invocation and must be recaptured
with source identity.  References: HiGHS
`HighsMipSolverData::setup` and
`HighsPrimalHeuristics::linesearchRounding`, lines 1216--1230 in the vendored
baseline.

The lock trace showed the same decision class for all 100 binaries: both
implementations have a positive up-lock and down-lock `1` for every column.
Native and HiGHS lock magnitudes matched on 99 columns; column 4986 had native
up-lock 563 versus HiGHS 499, but this cannot affect `linesearchRounding`,
which tests only whether a count is zero.  Thus both implementations apply
nearest rounding to the already bit-identical first-LP coordinates at
`alpha=0`.

This identifies an observational error in the earlier coordinate diagnosis:
HiGHS' `intcols` is sorted by lock/clique score in `setupIntCols()`.  The
recorded HiGHS bit string was emitted in that iteration order, while native's
string was emitted in structural column order.  Comparing those strings by
position was invalid.  The corrected diagnostic emits HiGHS rounded values in
structural column order and includes the heuristic solution-source ID.
Prediction: the corrected HiGHS alpha-zero assignment matches native exactly.
If so, the remaining defect is downstream in local-domain propagation/repair,
not in root LP or rounding construction.

The corrected structural-order trace matched for all eight assignments that
HiGHS attempted.  HiGHS accepted the eighth assignment at
`alpha=0.8887126824267898`; native generated the identical bits at
`alpha=0.88872867922308008`, rejected it, and continued through four more
breakpoints.  Native's root-subsolve ledger reports zero LP solves for all 12
attempts, so the rejection occurs in `BCDomain::fix_col/propagate`, whereas
HiGHS reaches its fixed-integer LP and installs objective `161`.

The next diagnostic compares the current structural-domain bounds against the
structural model bounds in both implementations, reporting changed columns
and aggregate counts before central rounding.  Prediction: a bound delta
present only in native excludes the feasible eighth assignment and identifies
the invalid or differently owned propagation source.  If the bound boxes are
identical, the mismatch is implementation infidelity in `BCDomain` activity
or tolerance arithmetic and must be reduced to the first conflicting row.

The normalized domain ledgers differ substantially.  Before central rounding,
4640 of 5080 columns have different bounds; native has 259 tighter lower
bounds and 4621 tighter upper bounds.  For example, HiGHS keeps column 0 at
`[103.15186,+inf]`, while native uses
`[103.15186,10033.729771424018]`; column 2 changes from HiGHS
`[48.483360000000175,+inf]` to native
`[60.425809899999749,9981.0830121230265]`.  This explains why equal integer
fixings need not follow the same propagation path, but it does not yet prove
whether native's extra bounds are valid.

RATIONALE

Model/algorithm: primal feasibility is a direct oracle for domain-bound
validity.  Every globally valid propagated bound must contain every feasible
solution of the structural model, including HiGHS' independently audited
central-rounding incumbent.

Claim: comparing the accepted HiGHS repair vector against native's pre-repair
box distinguishes an invalid/incorrectly owned native bound from a false
`BCDomain` row-activity conflict.

Cost model: observational output of 5080 primal coordinates and a linear
`O(n)` comparison; no additional LP solve and negligible time relative to the
existing root run.

Prediction: at least one coordinate of the audited HiGHS objective-161 repair
violates native's tighter bound by more than `1e-6`.  That column identifies
the first bound source requiring proof reconstruction.  If there is no
violation, the bound box is not the cause and the first native propagation
conflict row must be traced instead.

Assumptions: both traces use the same reduced-column order and the HiGHS
incumbent passes its existing original-model audit.  Validation is the fixed
paired one-node benchmark with environment-only domain and accepted-repair
coordinate traces.  References: Achterberg (2007), Sections 3.1 and 9.2;
HiGHS `HighsPrimalHeuristics::tryRoundedPoint` and `HighsDomain`.

## Repair-Box Oracle Mismatch

The accepted HiGHS objective-161 repair violates none of native's pre-repair
bounds: all 5080 coordinates lie inside the complete native box to `1e-6`.
Thus the predicted invalid or wrongly owned bound was not observed.  Native's
extra 4640 bound differences are real, but they do not exclude this feasible
solution and cannot by themselves justify rejecting the eighth fixing.

Mismatch investigation:

1. Implementation infidelity: now the leading suspect.  `BCDomain` reports
   infeasibility for a box containing a point independently audited feasible
   for the same structural model.
2. Machine/cost-model error: none; the comparison is `O(n)` and exact to the
   fixed `1e-6` feasibility tolerance.
3. Assumption violation: not observed.  Reduced column counts and ordering are
   identical, and the accepted vector contains all 5080 coordinates.
4. Theory error: the bound-box explanation is false.  A valid interval box
   containing a feasible point cannot be infeasible under exact row-activity
   propagation.

The next diagnostic records the first `BCDomain` infeasibility certificate:
row index, computed minimum/maximum activity, row sides, tolerance, and the
last fixed column.  The same accepted HiGHS vector is then evaluated on that
row as a direct feasibility oracle.  Prediction: the reported activity
interval incorrectly excludes the accepted vector's row activity, localizing
the arithmetic or incremental-update defect.  No repair bypass is authorized
until the certificate is understood.

## Transactional Domain-Restore Defect

The conflict certificate localized every central-rounding rejection to an
incremental upper-bound delta on structural row 0.  At the accepted eighth
assignment, native reported maximum activity `154` below lower side `161`
while processing column 5063.  Earlier failed attempts reported the same row
with maxima 160, 159, 154, 153, and so on.  This monotone decay across restored
savepoints is impossible for independent probes and exposes stale cached
activity.

The implementation audit found the exact state-machine defect.  In
`BCDomain::change_bound`, `apply_lb_delta` and `apply_ub_delta` mutate cached
row activities before `lb_[col]` or `ub_[col]` is assigned the corresponding
new bound.  If the activity update proves infeasibility, control returns before
the bound assignment.  `restore()` later reads the unchanged bound as both the
old and new endpoint, applies a zero reverse delta, and leaves the failed
activity mutation in the cache.  Reusing the domain across heuristic calls
therefore accumulates fictitious restrictions.  The feasible HiGHS incumbent
lying inside the native box is consistent with this diagnosis.

RATIONALE

Model/algorithm: transactional incremental domain propagation.  At every
return boundary, the explicit bound vector and every cached row activity must
describe the same box; rollback reverse-applies exactly the committed bound
deltas.  This is the state invariant of HiGHS `HighsDomain::changeBound` and
`backtrack`.

Claim: committing each scalar bound before applying its cached activity delta
makes an infeasible intermediate state exactly reversible by the existing
trail, eliminating cross-attempt activity corruption without changing any
propagation formula or tolerance.

Cost model: the update remains `O(column nnz)` and memory traffic is unchanged;
the correction only reorders two scalar stores relative to the existing delta
walks.  No allocations or extra row scans are introduced.

Prediction: on `neos-1122047`, failed line-search attempts restore exactly,
the eighth assignment reaches one fixed-integer repair LP, installs an audited
incumbent of `161`, and the solver terminates at one node below `3.5 s`.  The
alpha sequence remains aligned with HiGHS and no more than eight attempts are
needed.  On `p200x1188c`, incumbent `15078` and root lower bound at least
`13930.5` are retained.

Assumptions: each delta helper fully applies the one requested bound change
before setting infeasibility; the existing trail entry is pushed before either
store and records both pre-change endpoints; restore runs with infeasibility
cleared, as currently implemented.

References: HiGHS `HighsDomain::changeBound` and `backtrack`; Achterberg
(2007), Section 3.1; invariant derivation above.

Validation: add a focused restore regression comparing a fresh domain with a
domain after an infeasible fix/restore cycle, then run the fixed one-node
`neos-1122047` and `p200x1188c` gates on baseline commit `1b51f0bd8caa`, macOS
ARM64, Release `-O3 -DNDEBUG`.  A wrong sign or runtime/incumbent miss triggers
the mandatory mismatch protocol before another algorithm change.

## Transactional Restore Measurement Mismatch

The transactional update correction passes 30 assertions across lower- and
upper-bound infeasibility/restore regressions, but it did not change the
`neos-1122047` result.  Native still attempted 12 assignments, reached zero
repair LPs, found no incumbent, and completed root processing in `2.817 s`.
The prediction of an audited incumbent `161` below `3.5 s` therefore failed on
its primal component, while the runtime gate held.

Mismatch investigation:

1. Implementation infidelity: the repaired transaction invariant is exercised
   by the focused tests, and repeated certificate traces are stable per
   assignment.  The active rejection is not explained by cross-attempt cache
   accumulation.
2. Machine/cost-model error: none; the reorder added no measurable work and
   root time improved within normal run variance.
3. Assumption violation: confirmed.  The earlier monotone sequence of row-0
   maxima came from different rounded assignments, not repeated restoration of
   one assignment.  It was invalid evidence of cumulative corruption.
4. Theory error: the general transaction invariant remains necessary, but it
   is not sufficient to explain this instance.  A different cache-fidelity or
   propagation-formula defect remains.

The next fixed diagnostic recomputes the reported row's minimum or maximum
activity from scratch using the current explicit bounds at the instant of the
conflict, and compares it with the cached `StableActivitySum`.  Prediction: for
the accepted eighth assignment, the from-scratch maximum is at least `161`
while the cached maximum is `154`, proving incremental cache drift inside the
same propagation attempt.  If they agree, the explicit propagated box itself
excludes the fixing and the first invalid derived bound must be traced.

## Conflict Activity Oracle Mismatch

The from-scratch row activity equals the cached activity exactly for every
reported conflict.  At the eighth assignment, both are `154` with zero
infinite contributions against equality lower side `161`; the other line
search attempts and the unrelated randomized-rounding conflict also match
exactly.  The predicted within-attempt cache drift is therefore absent.

Mismatch investigation:

1. Implementation infidelity: not in activity caching.  The cached
   `StableActivitySum` faithfully represents the current explicit box.
2. Machine/cost-model error: none; recomputation is a single diagnostic row
   scan and agrees bit-for-bit.
3. Assumption violation: confirmed.  The earlier accepted HiGHS repair was
   checked against native's box *before* applying and propagating the integer
   fixing.  It does not establish that every bound subsequently derived by
   native contains that repair.
4. Theory error: the cache-drift explanation is false.  Either native derives
   an invalid bound during the fixing sequence, or its structural root model
   differs from the model used by HiGHS' repair LP.

The next fixed oracle solves native's own structural `root_lp` with the eighth
integer assignment fixed, using vendored HiGHS and no native propagation.  If
feasible at objective `161`, `BCDomain` has derived an invalid bound and the
first derived bound excluding the oracle solution must be traced.  If
infeasible, the model-ownership difference is real and the native/HiGHS repair
LP row sets must be compared.  This diagnostic is environment-gated and does
not alter ordinary execution.

## Structural LP Oracle Cost Mismatch

The first structural oracle implementation ran on every rejected assignment,
disabled presolve, and exhausted a 30-second solve plus the two-second hard
grace before emitting a central-rounding result.  No feasibility conclusion is
available from that run.

Mismatch investigation:

1. Implementation infidelity: the stated oracle targeted the eighth fixing,
   but the implementation invoked it for all rejected line-search fixings.
2. Machine/cost-model error: confirmed.  Cold fixed-integer LPs on 57611 rows
   with presolve disabled are not negligible and cannot be multiplied by 12
   under the benchmark watchdog.
3. Assumption violation: HiGHS `tryRoundedPoint` enables presolve when integer
   columns are at least one fifth of all columns, otherwise imports the root
   basis.  The diagnostic did neither equivalent operation.
4. Theory error: none.  A single correctly budgeted fixed-integer LP remains
   an authoritative feasibility discriminator.

The corrected diagnostic runs only for the eighth `linesearch_round` call,
uses a two-second internal wall limit, and follows HiGHS' repair presolve/basis
policy.  Prediction: it returns a feasibility status within two seconds; a
feasible result has objective `161`.  The oracle remains observational only.

## Structural Repair Model Divergence

The corrected eighth-assignment oracle returned `Infeasible` immediately with
zero simplex iterations for native's structural `root_lp`.  The equivalent
HiGHS central-rounding call accepts the same structural-order binary assignment
and installs an audited objective-161 incumbent.  Thus native `BCDomain` is
consistent with its own structural LP at this fixing; the active defect is a
repair-model ownership mismatch, not propagation arithmetic.

The same first-LP binary coordinates do not prove structural model identity:
many continuous vertices can support the same binary projection at objective
161, and a row-side or coefficient discrepancy can be inactive at the first
LP but decisive after integer fixing.  The next diagnostic compares canonical
row signatures and row sides of native `root_lp` against HiGHS'
`presolvedModel`/repair LP, then traces the first row violated by HiGHS' accepted
repair in native coordinates.  Prediction: at least one structural row or side
exists only in native or differs numerically.  The environment-only structural
LP oracle is removed after this localization.

## Structural Repair Model Mismatch Resolution

The accepted HiGHS repair coordinates resolve the apparent contradiction.
For structural columns 4980--5079, row 0 has ten consecutive coefficients of
each value 1 through 10.  The eighth pre-propagation line-search proposal has
weighted row activity 78, while the accepted HiGHS repair has weighted
activity exactly 161.  Twelve binary values differ: columns 4999, 5009, 5019,
5029, 5039, 5049, 5059, 5061, 5069, 5072, 5074, and 5075 are zero in the
proposal and one in the accepted repair.  Thus the accepted LP was never
solved with the literal pre-propagation proposal fixed.  The structural-model
divergence prediction was false.

Mismatch investigation:

1. Implementation infidelity: confirmed in the native HiGHS analogue.
   HiGHS processes integers in descending lock score, clamps each rounded
   request to the *current* local-domain interval, fixes it, and propagates
   immediately.  Native processes structural column order, passes the stale
   pre-propagation value directly to `fix_col`, and propagates only after all
   fixes.  These are different sequential projection operators.
2. Machine/cost-model error: none.  The coordinate comparison and row dot
   products are linear scans and exact for binary values and integer
   coefficients.
3. Assumption violation: the earlier oracle assumed the proposal printed by
   `linesearchRounding` was the final fixed vector.  HiGHS' evolving-domain
   clamp invalidates that assumption.
4. Theory error: a rounded proposal is not a simultaneous fixing.  It is a
   preference vector projected sequentially through domain propagation; later
   preferences may be overridden by implications of earlier decisions.

RATIONALE

Model/algorithm: sequential projection of an integral preference vector onto
the propagation closure of a mixed-integer domain.  Let `D_0` be the root
domain and let integer columns be ordered by descending two-sided lock score.
At step `k`, set

```text
v_k = min(ub_{D_k}(j_k), max(lb_{D_k}(j_k), round(xr(j_k))))
D_{k+1} = propagate(D_k intersect {x(j_k) = v_k}).
```

This is the state transition implemented by HiGHS
`HighsPrimalHeuristics::setupIntCols` and `tryRoundedPoint`.

Claim: applying the same lock-priority order, evolving-domain clamp, and
per-fix propagation in native central-rounding repair produces the same
propagation closure as HiGHS when lock classifications and the initial domain
agree.  In particular, an already implied later integer is fixed to its
implied value instead of making the repair domain falsely infeasible.

Cost model: sorting `n_I` integer columns costs `O(n_I log n_I)` comparisons
once per root.  Each repair attempt retains `O(n_I)` fixes and the existing
incremental propagation work; clamping adds two comparisons per fixed
integer.  Propagating after each fix does not add row work asymptotically
because `BCDomain` processes only marked rows and drains the same implication
queue before the next decision.  On the 100-binary `neos-1122047`, sorting and
clamping are negligible relative to the root LP.

Prediction: the eighth `neos-1122047` line-search proposal reaches exactly one
fixed-integer repair LP, yields an audited incumbent and objective 161, and
terminates at one node within 3.5 seconds.  The repair closure has row-0
weighted activity 161.  `p200x1188c` retains incumbent 15078 and root lower
bound at least 13930.5, with one-node wall time no more than 10% above its
1.371-second reference measurement.

Assumptions: native and HiGHS begin repair from equivalent structural domains;
the measured lock classifications agree; implication/clique tie scores are
zero or are not decisive for this instance; and `BCDomain::propagate` reaches
the same row-propagation fixed point after each decision.  Native uses a
deterministic column-index tie break when no clique implication score is
available.

References: Achterberg (2007), Sections 3.1 and 9.2; HiGHS
`HighsPrimalHeuristics::setupIntCols` and `tryRoundedPoint`; the coordinate and
row-activity derivation above.

Validation: first add a focused domain regression in which fixing one integer
propagates a later integer to the opposite of its rounded preference; the
sequential clamp must accept the implied value and restore exactly.  Then run
Release `-O3 -DNDEBUG` tests and the fixed one-node `neos-1122047` and
`p200x1188c` benchmark commands recorded above, on commit baseline
`1b51f0bd8caa`, macOS ARM64.  A missing incumbent, wrong objective, runtime
over 3.5 seconds on `neos`, or more than 10% `p200` regression triggers the
mandatory mismatch protocol before another algorithm change.

## Sequential Projection Measurement Mismatch

The focused projection regression passed as part of 430 assertions in 32
branch-and-cut test cases, and the broader MILP suite passed 2029 assertions in
54 cases.  However, the fixed `neos-1122047` gate still reported
`linesearch_round calls=12 ok=0 lps=0`, found no incumbent, and reached the
five-second limit at one node.  The prediction of one feasible repair LP and
incumbent 161 therefore failed with the wrong primal result.  Root processing
itself completed in 3.106 seconds, inside the predicted 3.5-second root-time
bound, but that does not satisfy the primal-quality claim.

Mismatch investigation:

1. Implementation infidelity: the isolated operator computes the stated
   clamp/fix/propagate transition and its restore contract passes.  End-to-end
   fidelity is not established because native's integer order has only matched
   HiGHS' lock-score formula, not its complete implication tie scores, and the
   resulting fixing sequence has not been compared.
2. Machine/cost-model error: none observed.  Sorting and the added clamps did
   not push root processing beyond 3.5 seconds.
3. Assumption violation: now the leading suspect.  Equivalent raw row-lock
   classifications do not imply identical `intcols` ordering: HiGHS breaks
   lock-score ties using both literal implication counts, while native's
   available clique table may represent a different subset.  Native also
   reconstructs `BCDomain` from a root box after its own propagation pipeline,
   rather than copying HiGHS' live domain state.
4. Theory error: not yet established.  Sequential projection remains the
   observed HiGHS transition, but its result is order-dependent, so matching
   only the transition formula is insufficient.

The next fixed diagnostic uses the existing native repair-conflict certificate
to record the first rejected row and column for each line-search proposal.  It
then compares the eighth attempt's integer order and evolving fixed values with
HiGHS `tryRoundedPoint`.  Prediction: the first divergence occurs before the
native row-0 infeasibility and is caused by a different tie ordering; if the
orders and values agree up to that point, the initial domains or propagation
closures differ and must be compared at that exact step.  No further
algorithmic edit is authorized until this discriminator is resolved.

## Repair-Domain Initialization Discriminator

The existing certificate disproved the predicted ordering conflict.  Every
rejected proposal reported `kind=none`, and the initialization status was

```text
ok=0 infeasible=0 complete=0 rows=461400
```

The row count is exactly the propagation safety budget for the current
structural model.  Therefore no proposal reached its first semantic fixing:
the first per-fix `propagate()` resumed a pre-existing initialization backlog,
hit the safety budget again, and returned `false`; the caller then mislabeled
incomplete propagation as infeasibility.  Integer ordering was not exercised
far enough to explain this measurement.

Mismatch investigation:

1. Implementation infidelity: confirmed.  HiGHS `tryRoundedPoint` copies its
   already-propagated live `HighsDomain`, whose model-row queue is empty before
   heuristic fixings.  Native reconstructs equivalent activity caches from an
   already-propagated root box but marks every structural row pending, ignores
   the incomplete initialization result, and later treats incompleteness as a
   conflict.
2. Machine/cost-model error: the earlier cost model omitted 461400 redundant
   initialization row visits.  This work is not part of HiGHS' copied-domain
   transition and dominates the intended `O(n_I)` preference projection.
3. Assumption violation: confirmed.  The repair domain did not start at
   propagation closure as assumed.
4. Theory error: none in sequential projection.  The state-transfer boundary,
   not the projection recurrence, was implemented incorrectly.

RATIONALE

Model/algorithm: snapshot transfer for an incremental propagation engine.
Given a root box that has already passed the solver's root-domain closure, a
repair-domain snapshot reconstructs exact row activities for that box and
starts with an empty propagation queue.  A subsequent bound change marks its
incident rows and drains only consequences of that change.  This matches the
state copied by HiGHS `auto localdom = mipdata_->domain` in
`tryRoundedPoint`.

Claim: initializing `BCDomain` from the already-propagated root box without
marking all model rows preserves the box and activity invariants while removing
the artificial incomplete backlog.  Every heuristic fixing still marks all
incident rows, so its propagation closure is unchanged relative to a fully
closed initial state.

Cost model: activity construction remains `O(nnz)` once.  Removing the initial
queue avoids 461400 observed row propagations on `neos-1122047`; subsequent
work is proportional to rows incident to changed bounds and their implication
closure.  No allocations or LP solves are added.

Prediction: repair initialization reports complete with zero processed rows;
the eighth line-search proposal reaches one repair LP, installs audited
incumbent 161, and root processing remains below 3.5 seconds.  The focused
regression proves that a no-initial-sweep snapshot still propagates rows touched
by the first fixing.  `p200x1188c` retains incumbent 15078, root bound at least
13930.5, and no more than 10% wall-time regression from 1.371 seconds.

Assumptions: the supplied root box has completed the main root propagation
pipeline and is feasible; activity construction exactly represents that box;
all later bound changes pass through `change_bound`, which marks every incident
row.  This no-initial-sweep mode is restricted to the repair snapshot; generic
`BCDomain` initialization keeps its full initial sweep.

References: HiGHS `HighsPrimalHeuristics::tryRoundedPoint` and
`HighsDomain::propagate`; Achterberg (2007), Section 3.1; state-transfer
derivation above.

Validation: extend the projection regression to use the propagated-snapshot
initialization and assert zero initial row visits plus the same implied later
binary.  Run the fixed focused/MILP tests and both one-node benchmark gates.
Wrong primal outcome, root time above 3.5 seconds, or more than 10% `p200`
regression triggers another documented mismatch before further edits.

## Snapshot Initialization Measurement Mismatch

The snapshot regression passed, and the `neos-1122047` diagnostic matched the
initialization prediction exactly:

```text
ok=1 infeasible=0 complete=1 rows=0
```

Nevertheless, every repair attempt still returned incomplete with
`kind=none`, reached zero repair LPs, and found no incumbent.  Root processing
was 3.068 seconds, within the timing prediction, but the primal prediction
failed again.

Mismatch investigation:

1. Implementation infidelity: the initial copied-state boundary is now
   faithful, but `project_integer_preferences` treats any `propagate()==false`
   as infeasible even though `BCDomain::propagate` also returns false when its
   safety budget expires with a nonempty queue.  The boolean interface loses a
   required third state.
2. Machine/cost-model error: confirmed for post-fixing propagation.  Even from
   an empty queue, the first fixing can generate more than `8*m` row visits.
   The assumption that incremental marking alone bounded the closure below one
   call's budget was false.
3. Assumption violation: confirmed.  Native row propagation lacks HiGHS'
   capacity-threshold scheduling and applies bound changes during a row sweep,
   so an incident-row cascade may be substantially larger than the HiGHS
   closure on the same box.
4. Theory error: the snapshot-transfer claim remains correct, but it is
   insufficient.  A trivalent propagation result and a measured closure cost
   are prerequisites for deciding how repair should handle incomplete native
   propagation.

The next fixed diagnostic records row visits, trail growth, and pending queue
size for the first incomplete projection.  Prediction: the first integer
fixing alone consumes the full 461400-row budget and leaves a nonempty queue,
with no infeasibility certificate.  If so, the next derivation must either
restore HiGHS-style capacity-threshold scheduling or define a proof-safe
incomplete-propagation repair path; increasing the cap without a cost model is
not authorized.

## Propagation Bound-Admission Discriminator

The first fixed integer was column 4986.  Eight additional propagation chunks
each consumed the full 461400-row budget without infeasibility or closure.
Across those chunks the trail grew from 44526 to 186031 changes, while the
pending queue remained between 728 and 1884 rows.  A finite monotone bound
closure on only 5080 columns should not require this many accepted changes.

The implementation comparison found a direct fidelity defect.  HiGHS
`HighsDomain::adjustedLb/adjustedUb` admits an integral bound only when its
movement exceeds `1000 * feastol * abs(bound)`.  For a continuous bound it
requires both an absolute `1000 * feastol` improvement and at least 30% of the
current finite interval (or the analogous magnitude scale for a one-sided
interval).  Native `BCDomain::propagate_row` admits every improvement above
approximately `1e-9`.  Consequently small residual changes are committed,
mark incident rows again, and form the observed long numerical chain before
HiGHS would schedule capacity propagation at all.

RATIONALE

Model/algorithm: numerically significant bound admission for row propagation.
For an integral column, admit candidate `b` only if the rounded bound is
strictly tighter and the movement exceeds `1000 * feastol * |b|`.  For a
continuous column, admit only if the movement exceeds `1000 * feastol` and its
relative improvement is at least 0.3, using the current finite interval when
available and `max(|current|, |candidate|)` for a one-sided interval.  These
are HiGHS `HighsDomain::adjustedLb` and `adjustedUb`.

Claim: applying the HiGHS admission predicate before `change_bound` rejects
numerically insignificant continuous propagation steps without losing any
bound that HiGHS would use in its central-rounding domain.  Since rejected
steps are weaker than HiGHS' own domain closure, this restores heuristic
fidelity rather than weakening the intended repair model.

Cost model: each candidate adds constant-time comparisons and, for finite
two-sided continuous columns, one division.  Rejected changes avoid a column
activity update, trail entry, and incident-row rescheduling.  On the measured
`neos` cascade, the prediction is to reduce accepted trail changes by at least
90% from 186031 and finish the first fixing's propagation within one 461400-row
budget.

Prediction: the first traced projection is complete with fewer than 20000
trail entries and fewer than 461400 row visits.  The eighth line-search
proposal reaches a repair LP and installs audited incumbent 161; root time is
below 3.5 seconds.  `p200x1188c` retains incumbent 15078 and root bound at
least 13930.5 with no more than 10% wall-time regression from 1.371 seconds.

Assumptions: the long cascade is dominated by continuous bound movements that
HiGHS' admission rule rejects; native and HiGHS use `feastol=1e-7` for domain
propagation; candidates are finite before admission; and integral rounding is
already performed by the row formulas.

References: HiGHS `HighsDomain::adjustedLb`, `adjustedUb`, and `boundRange`;
Achterberg (2007), Section 3.1; measurement above.

Validation: add focused continuous and integral admission regressions, rerun
the branch-and-cut and MILP suites, then the two fixed one-node benchmarks.
Failure to meet the trail/row prediction triggers the mismatch protocol before
implementing the separate HiGHS capacity-threshold scheduler.

## Bound-Admission Measurement Mismatch

The focused admission tests passed as part of 440 assertions in 33 cases, but
the end-to-end result had the wrong sign.  After repair initialization reported
zero pending rows, the first `neos-1122047` projection did not return before
the benchmark's seven-second hard process timeout.  It emitted neither an
infeasibility certificate nor an incomplete-chunk measurement.  The prediction
of closure below 461400 row visits and 3.5 seconds therefore failed.

Mismatch investigation:

1. Implementation infidelity: confirmed at the propagation schedule level.
   HiGHS scans a batch of pending rows against one domain snapshot, stores
   proposed bound changes, and applies them only after every row in the batch
   has been evaluated.  It also recomputes per-row capacity thresholds.  Native
   mutates bounds recursively inside each row sweep.  Copying only HiGHS'
   admission predicate into this different schedule does not implement the
   derived HiGHS transition.
2. Machine/cost-model error: confirmed.  The predicted saved rescheduling did
   not account for the cost of evaluating the still-pathological recursive row
   queue under the new predicate.  Wall time exceeded seven seconds before the
   first projection returned.
3. Assumption violation: confirmed.  Small-bound admission was not the sole or
   dominant cause of the propagation explosion.
4. Theory error: the standalone admission rule matches HiGHS, but the claim
   that it would restore central-rounding fidelity independently was false.
   Bound admission, capacity scheduling, and batched application form one
   coupled algorithm.

No further native propagation edit is authorized until that coupled algorithm
is derived as a whole or an existing proven HiGHS domain/root-primal component
is reused through a bounded interface.  The next read-only investigation maps
the repository's existing vendored-HiGHS root-primal adapters and determines
whether central rounding can be invoked without duplicating the full domain
engine.  The failed standalone admission change is not retained as a claimed
improvement.

## Coupled HiGHS Root-Primal Ownership

The repository already contains the required bounded reuse interface.  Under
the strict HiGHS root contract, a retained `Highs` owner executes presolve,
`HighsDomain`, root LP evaluation, separation, analytic-center construction,
and central rounding as one coupled state machine, then stops before tree
search.  `Highs::run()` postsolves any incumbent into original model
coordinates before returning.  Native currently imports the retained root LP,
cuts, basis, and dual bound but discards that postsolved incumbent.

RATIONALE

Model/algorithm: compositional root ownership with audited primal transfer.
HiGHS owns the complete root state transition through
`HighsMipSolverData::evaluateRootNode`; native imports the resulting LP state
and an incumbent certificate.  For each surviving native working column `j`,
the retained presolve metadata defines

```text
x_original(orig(j)) = scale(j) * x_working(j) + constant(j),
x_working(j) = (x_original(orig(j)) - constant(j)) / scale(j).
```

The mapped point is not trusted directly: native recomputes its objective and
passes it through the existing bound, row, integrality, quality, and proof
audit in `adopt_root_incumbent`.

Claim: when the retained HiGHS root owner has a valid postsolved incumbent and
every surviving working column has an exact affine map, importing that point
preserves feasibility and objective value between coordinate systems.  The
native audit rejects any stale, inexact, dimensionally inconsistent, or
non-improving transfer.  This obtains HiGHS root primal quality without a
partial reimplementation of its coupled domain propagation algorithm.

Cost model: incumbent extraction and affine mapping cost `O(n)` time and one
working-space vector.  The strict root owner is already executed to provide
the authoritative root LP/cut/basis state, so the transfer adds no LP or MIP
solve.  Enabling that existing ownership contract for `native-highs-lp`
replaces native's approximate root pipeline rather than running a HiGHS tree.

Prediction: `neos-1122047` imports the HiGHS central-rounding incumbent 161,
passes native audit, closes the zero root gap, and terminates at one root node
within five seconds.  `p200x1188c` retains incumbent 15078 and root lower bound
at least 13930.5; its one-node wall time remains below two seconds.  The
incumbent transfer itself consumes zero additional LP solves.

Assumptions: the retained root owner stops after `evaluateRootNode`; its public
solution is postsolved and marked valid; the strict working LP is the retained
HiGHS presolved coordinate system; all surviving columns expose nonzero exact
affine transforms; and native's original and working objective conventions
match the existing root-state import.

References: HiGHS `HighsMipSolver::run`,
`HighsMipSolverData::evaluateRootNode`, `Highs::run` MIP cleanup/postsolve, and
`HighsPrimalHeuristics::centralRounding`; Achterberg (2007), Sections 3.1 and
9.2; Berthold (2006), Section 3.5; coordinate derivation above.

Validation: add an affine-transfer regression through the existing presolve
mapping helper; enable the existing strict root ownership contract for the
HiGHS-LP native benchmark; run focused/MILP tests and the fixed `neos` and
`p200` one-node gates.  Missing incumbent, audit failure, more than five
seconds on `neos`, more than two seconds on `p200`, or an extra transfer LP
solve triggers the mismatch protocol before further algorithm edits.

### Root-primal transfer validation mismatch

The first `neos-1122047` gate at commit `1b51f0bd8caa` on macOS 26.5.2 arm64,
Release `-O3 -DNDEBUG`, imported the authoritative root LP state at bound 161
and entered `adopt_root_incumbent("highs_root_primal", ...)` with objective
161, but exceeded the seven-second process watchdog instead of terminating
within five seconds.  The transfer added no LP solve, but adoption synchronously
called `run_root_incumbent_propagation` before testing the already exact primal
and dual certificate.  Its repeated objective-artifact closure passes cannot
strengthen a proof once the audited incumbent equals the rigorous root bound.

Mismatch classification, in required order: (1) implementation infidelity.
The ownership model predicted an `O(n)` transfer followed by audit, whereas the
implementation accidentally attached an optional incumbent-driven fixed point
to that transfer.  Machine cost, input assumptions, and the compositional
ownership theory are not implicated because the expected incumbent and bound
were both observed before the unexpected work began.

Corrected claim: after native audit has accepted an incumbent, if
`incumbent_obj - root.bound <= 1e-9 * max(1, abs(incumbent_obj))`, incumbent-
driven propagation has zero certificate value and is skipped.  The exact-gap
predicate is the existing native optimality certificate, so this changes no
feasible set or bound.  Cost changes from the observed repeated closure sweeps
to `O(1)` after the `O(n)` audit.  Prediction: the same `neos` gate retains
objective and bound 161, performs no incumbent-transfer LP, and completes
within five seconds; all focused tests remain unchanged.  Validation commands
and the `p200x1188c` thresholds above remain fixed.

The first corrective placement guarded only the adoption hook.  A repeated
gate again timed out at 7.015 seconds even though the log confirmed
`[B&C-HIGHS-ROOT-PRIMAL] available=1 adopted=1 obj=161`.  Call-graph audit
found a second unconditional invocation,
`run_root_incumbent_propagation("after_root_incumbent")`, after the primal
heuristic phase.  This remains implementation infidelity: the predicate was
placed at one caller instead of at the shared propagation routine.  The
corrected implementation places the exact-root-certificate guard at the entry
of `run_root_incumbent_propagation`, covering adoption, post-heuristic, and
post-dual-proof callers.  The model, cost claim, prediction, and fixed
validation thresholds are otherwise unchanged.

With the shared guard, the next gate reached native optimality in 3.830 seconds
with bound/incumbent 161, zero transfer LPs, and zero tree nodes, but the
benchmark reported `Audit=no`: finalization copied the presolved incumbent to
`BCResult::x` and explicitly skipped original-space objective recomputation
when `strict_highs_presolved_working_space` was true.  This is a third
implementation-infidelity finding.  The retained root `Highs` owner holds the
matching `HighsPostsolveStack`, so finalization must call its primal postsolve
for every incumbent produced in the native working space, then run the existing
original-model validation.  This is HiGHS' standard presolve/postsolve
composition, costs `O(n + nnz)` without an LP solve, and is required for the
claimed coordinate-space preservation.  Prediction: `neos` remains below five
seconds with objective/bound 161 and zero transfer LPs, while benchmark audit
changes from `no` to `ok`; the fixed `p200x1188c` gate remains unchanged.

The attempted finalization replay failed loudly with
`Invalid incumbent after HiGHS postsolve (strict root postsolve unavailable)`
at 4.176 seconds.  `Highs::run()` had already postsolved the root incumbent and
consumed the MIP postsolve lifecycle before native finalization, so invoking
the generic public postsolve API a second time could not recover the solution.
This is implementation infidelity in ownership lifetime, not a failure of the
affine mapping: the original-space solution had already been captured before
the working-space audit.  The corrected composition retains that original
vector paired with its mapped working vector.  If the paired working vector is
the accepted final incumbent, finalization publishes the already postsolved
original vector and audits it against the original model.  A later native-tree
incumbent without an owned postsolve certificate fails loudly rather than
returning presolved coordinates.  Pair comparison is `O(n)` and adds no LP.
The objective, audit, timing, and `p200x1188c` predictions remain unchanged.

The first `p200x1188c` gate exited in 6.2 milliseconds before root processing
with `HiGHS presolved working LP unavailable`.  HiGHS reported
`status=not_reduced`, with the same 2376 columns, 1388 rows, and 4752 nonzeros
as the original model.  Mismatch classification: (3) assumption violation.
The rationale assumed a nonempty reduced model, but HiGHS presolve may validly
return the identity transformation.  In that case the authoritative HiGHS
working space is the original model, every surviving-column transform is the
identity, and the objective offset is zero.  Treating this exact fixed point as
fatal contradicts compositional ownership.  Corrected claim: strict root
ownership accepts both `reduced` and `not_reduced`; the latter retains
`base_lp` unchanged and marks it as an exact HiGHS working space.  This is
`O(1)` and changes no model data.  Prediction and validation remain fixed:
`p200x1188c` must import incumbent 15078, retain root bound at least 13930.5,
audit in original space, and complete below two seconds.

The identity admission exposed a deeper ownership incompatibility.  During
`evaluateRootNode`, HiGHS performed repeated inactive-integer restarts and
changed its live root model from 2376 columns to 29.  Native had already built
its domain, integrality masks, and presolve metadata for 2376 columns.  The
unconditional import reported bound 13389, below the fixed 13930.5 gate, then
`BCDomain` rejected the 29-column preference vector against its 2376-column
domain.  The captured public solution also mapped to a non-finite objective
and was rejected before adoption.

Mismatch classification: (1) implementation infidelity and (3) assumption
violation.  Compositional ownership requires an isomorphism between the
producer's final root coordinates and the consumer's fixed tree coordinates;
the code checked the imported ledger internally but not against the native
working-column dimension.  HiGHS root restarts are allowed to change that
dimension, invalidating the assumption that the initial presolve map remains
the final root map.

Corrected algorithm: root-oracle import is transactional.  If the final
retained HiGHS LP has the native working-column count, import proceeds as on
`neos-1122047`.  Otherwise no oracle LP, bound, basis, or primal state is
committed; strict root-only policy is rolled back and the unchanged native
root pipeline continues.  This is the standard representation-invariant gate
for composing presolve/restart state machines.  The check is `O(1)`; fallback
reuses the already validated native RENS path, previously measured at
incumbent 15078, root bound 13930.54148, and 1.371 seconds.  Prediction:
`p200x1188c` again meets those values below two seconds, while `neos-1122047`
retains the compatible HiGHS root import and its audited 161 result below five
seconds.  The fixed commands and thresholds remain unchanged.

Late transactional rollback prevented the dimension exception, but missed the
quality prediction: `p200x1188c` returned audited incumbent 15531 and root
bound 5678.60709 in 0.358 seconds.  Strict-root policy had already suppressed
the production HiGHS presolve and native cut/heuristic setup before the
incompatibility was detected, so restoring option values after root evaluation
could not reconstruct skipped preprocessing.  This is implementation
infidelity in transaction placement.

The corrected eligibility transaction runs before strict policy
normalization and reuses the cached HiGHS presolve side-state consumed by later
setup.  Root-only ownership is selected only for a genuine `reduced` snapshot,
where the retained affine map supplies the required coordinate isomorphism.
`not_reduced` stays on the established native HiGHS-LP pipeline from entry; it
does not pay a second presolve.  Any unexpected final-column mismatch after an
eligible preflight fails loudly as `HiGHS presolved working LP mismatch`
instead of attempting partial rollback.  The prediction and fixed validation
thresholds remain unchanged.

## Eligibility-Transaction Validation

Baseline commit `1b51f0bd8caa` plus the campaign working-tree diff, macOS
26.5.2 arm64, Release `-O3 -DNDEBUG`, one thread, seed 0, the fixed build and
benchmark commands above.  Every prediction component was met with the right
sign:

- `p200x1188c` (30 s, one node): audited incumbent `15078` (predicted
  `15078`), root bound `13930.54148` (predicted at least `13930.5`), wall time
  `1.293 s` (predicted below `2 s`; reference `1.371 s`, so within the 10%
  band).  The `not_reduced` preflight kept the native pipeline from entry; no
  strict-root state was committed and no dimension exception occurred.
- `neos-1122047` (5 s, one node): audited incumbent `161` imported through
  `highs_root_primal` with zero transfer LP solves (ledger
  `calls=1 ok=1 inc=1 lps=0`), root bound `161`, gap `0`, status
  `Optimal (root gap closed)` at zero tree nodes, wall time `3.726 s`
  (predicted below `5 s`), original-space audit `ok` with maximum row
  violation `5.3e-12`.
- Focused suites: `test_branch_and_cut` passed 436 assertions in 33 cases;
  `test_milp_solver` passed 2034 assertions in 55 cases.

This closes the strict-root ownership chain for this experiment: root primal
parity on `neos-1122047` is obtained by bounded reuse of the coupled HiGHS
root state machine, and identity-presolve roots retain the validated native
RENS pipeline.  The two fixed gates remain the regression contract for any
subsequent root-policy change.

## Thirty-Instance Parity Sweep

With both gates green, a read-only discovery sweep measured the broader
parity state: 30 evenly sampled MIPLIB 2017 instances, `highs-mip` versus
`native-highs-lp`, 10-second limit, 2-second grace, seed 0, same build
(raw results `/tmp/root_parity_sweep_2026-08-13.{csv,json}`).  HiGHS
reported 13 audited feasible runs and one optimum; native reported 2 and 0,
with five distinct native defect classes:

1. `worker terminated by signal 6` on `neos17`, `bnatt400`, and
   `traininstance2` (three of thirty).  The crash-report backtrace shows
   `std::terminate` raised from `~unique_ptr<std::thread>` inside
   `BCSolveState::run()`: the strict-root dimension-mismatch early return
   (`05_root_relaxation_d.inc`) exits scope while the background
   analytic-centre thread is still joinable, and ISO C++
   [thread.thread.destr] then terminates the process.
2. The same three instances prove the eligibility assumption wrong: a
   `reduced` preflight does not imply a stable coordinate system, because
   `evaluateRootNode` performs inactive-integer restarts
   (`HighsMipSolverData.cpp:3003`) that re-presolve into new coordinates
   (`neos17`: 459 final versus preflight-committed native columns).  Ten
   percent of the sample hitting a designed loud-fail is a defect class,
   not an anomaly.
3. `Invalid incumbent after HiGHS postsolve (strict root certificate
   mismatch)` on `glass4`, `ic97_potential`, and `neos-3046615-murg`: the
   native tree improved past the imported root incumbent, and finalization
   owns no postsolve certificate for tree incumbents, so a feasible run is
   reported as a failure.  The strict working space is built from the
   stats-only `cached_highs_presolve_side_state`, while the retained-
   instance path (`highs_presolve_lp`, `HighsLpPresolveResult::impl`,
   `highs_presolve_recover_primal`) that can postsolve arbitrary
   working-space primals is skipped in strict mode.
4. Ten native hard process timeouts versus five for HiGHS: root phases on
   large instances do not honour the wall limit plus grace.
5. `Root relaxation failed` on `neos-3656078-kumeu`, `rail02`, and
   `satellites2-40`.

Classes 1 and 2 are corrected first (smallest closed derivations); classes
3-5 are open, in that order.

## Root Background-Task Ownership

RATIONALE

Model/algorithm: resource-safe structured concurrency for the root
background analytic-centre task.  ISO C++20 [thread.thread.destr]:
destroying a joinable `std::thread` calls `std::terminate`.  The correct
ownership contract is destructor-join (the `std::jthread` invariant,
[thread.jthread]): every control-flow exit of the owning scope joins
exactly once, structurally.

Claim: replacing the manually joined `unique_ptr<std::thread>` with a
join-on-destruction wrapper removes every terminate path, including the
strict-root transactional early return, without moving the normal-path
join site or changing any propagation, LP, or heuristic behaviour.

Cost model: `O(1)` state; on early-return paths the destructor join blocks
at most the analytic-centre wall cap of half the remaining root time,
which already bounds the normal-path join.

Prediction: `neos17`, `bnatt400`, and `traininstance2` no longer die with
signal 6.  With this change alone they return the honest
`kHighsPresolvedWorkingLpMismatch` status; combined with the coordinate
pinning below they complete normally.  Both fixed gates are unchanged.

Assumptions: the analytic-centre HiGHS task honours its wall limit; no
other joinable thread escapes `BCSolveState::run()` (tree explorer and cut
worker threads are joined by the concurrent tree teardown).

References: ISO C++20 [thread.thread.destr], [thread.jthread];
crash report `miplib2017_benchmark-2026-08-13-183554.ips`; sweep record
above.

Validation: a focused regression destroys the wrapper while its thread is
still running and asserts completion without terminate; both suites and
both fixed gates re-run unchanged.

## Strict-Root Coordinate Pinning

RATIONALE

Model/algorithm: compositional root ownership (above) requires an
isomorphism between the owner's final root coordinates and native's fixed
working columns.  Every root re-presolve site in the vendored HiGHS is
gated on `mip_allow_restart` (`HighsMipSolverData.cpp:3003`, `:3432`; the
tree-side site in `HighsMipSolver.cpp:510` is unreachable under
`stop_after_evaluate_root`).  Setting `mip_allow_restart=false` on the
retained root owner makes the preflight coordinate system invariant
through root evaluation by construction.

Claim: with restarts disabled, the final root LP has exactly the preflight
column count for every `reduced`-eligible instance; the loud-fail
dimension branch becomes a genuine invariant violation.  Cut separation,
RENS, analytic-centre computation, and central rounding are unaffected.
Represolve-based restart strength is excluded — an explicit scope cut;
wholesale post-restart state adoption is recorded as the successor
experiment for class-3/4 work.

Cost model: one `O(1)` option assignment.  Root strength on restarting
instances is below upstream HiGHS (no represolve), but those instances
previously returned no result at all.

Prediction: `neos17`, `bnatt400`, and `traininstance2` at the 10-second
limit exit normally — no signal 6, no mismatch status — and any reported
incumbent passes the original-space audit.  `neos-1122047` retains audited
incumbent `161`, bound `161`, zero transfer LPs, below five seconds.
`p200x1188c` is `not_reduced` and keeps its native-pipeline result
(incumbent `15078`, bound at least `13930.5`, below two seconds).

Assumptions: the vendored owner honours `mip_allow_restart` at both root
sites; no other `evaluateRootNode` path changes the model dimension when
it is false; the prior `neos-1122047` import did not depend on a restart
(its evaluated root already matched the native column count).

References: HiGHS `HighsMipSolverData::evaluateRootNode` restart gates;
sweep record and eligibility mismatch above; Achterberg (2007),
Section 10.1.

Validation: the fixed `neos-1122047` and `p200x1188c` gate commands, the
three-instance 10-second reruns, and both suites.  A signal-6 exit, a
mismatch status on the rerun set, a changed gate objective/bound, or a
gate-time regression beyond the recorded ceilings triggers the mismatch
protocol.

## Ownership And Pinning Validation

Both corrections were measured together on the same baseline, build, and
seed.  Every prediction component was met with the right sign:

- Suites: `test_branch_and_cut` passed 441 assertions in 34 cases
  (including the new join-on-destruction regression);
  `test_milp_solver` passed 2034 assertions in 55 cases.
- Former crash set at the 10-second limit, seed 0: no signal 6 and no
  mismatch status on any of the three.  `neos17` reached 39 nodes, gap
  `0.0269`, audit ok (row violation `1.8e-15`); `traininstance2` reached
  46 nodes with an audited incumbent (gap `1`); `bnatt400` timed out
  normally at 19 nodes without an incumbent.  The strict-root import now
  completes on these instances (for example `bnatt400`: 2145 columns
  imported, bound `313.44238432`, `error=none`).
- `p200x1188c` gate: incumbent `15078`, bound `13930.54148`, audit ok,
  `1.282 s` (reference `1.371 s`).  Unchanged, as predicted.
- `neos-1122047` gate: `Optimal (root gap closed)`, audited `161`,
  `3.805 s`, zero tree nodes.  Unchanged, as predicted.

The loud-fail dimension branch is retained as an invariant check but was
not observed after pinning.

Repeating the thirty-instance sweep after both corrections
(`/tmp/root_parity_sweep_postfix_2026-08-13.{csv,json}`): native audited
feasible runs rose from 2 to 4, the signal-6 class is empty, and two
former hard-timeout instances (`ns1208400`, `rocII-5-11`) now return
within the grace window.  HiGHS remains at 13 audited feasible runs on
the same sample, so the parity gap is now owned by the remaining classes:
strict-mode tree-incumbent postsolve ownership (class 3, still rejecting
real incumbents on `glass4`, `ic97_potential`, and `neos-3046615-murg`:
route the strict working space through the retained-instance presolve
pass so `highs_presolve_recover_primal` can certify tree incumbents),
wall-limit compliance on large roots (class 4, ten hard timeouts), and
the three `Root relaxation failed` instances (class 5).

## Strict Tree-Incumbent Postsolve Ownership

RATIONALE

Model/algorithm: HiGHS presolve/postsolve composition for the strict
working space.  The strict working LP is `getPresolvedLp()` of one HiGHS
presolve pass; every point of that space postsolves to original
coordinates through that pass's `HighsPostsolveStack`
(`Highs::postsolve(solution)`, whose MIP path accepts a primal-only
column solution of presolved size, ignores row values, and reconstructs
every eliminated original column).  The lifecycle constraint measured
earlier still holds: `Highs::run()` consumes the MIP postsolve lifecycle,
but a presolve-only instance keeps `can_run_postsolve` true for the
`kReduced` state.  Native already relies on exactly this contract in the
non-strict `highs_mip_model_reduced` finalization path via
`HighsLpPresolveResult::impl` and `highs_postsolve_primal`.

Claim: retaining the side-state pass's `Highs` instance and postsolving
any non-paired strict-mode incumbent through it publishes every native
tree incumbent in original coordinates, with the existing original-model
validation as the audit gate.  The paired root-incumbent fast path is
unchanged; an identity (`not_reduced`) working space needs no transform
because its coordinates are already original.  The loud-fail branch
remains only for a genuinely unavailable or invalid transfer.

Cost model: one `O(n + nnz)` primal postsolve at finalization, no LP
solve.  Memory: the side-state cache entry additionally retains one
presolve-only `Highs` instance (original plus presolved model plus
stack); retention is requested only by strict-contract entry points, so
non-strict solves keep the existing cache footprint.  A mutex guards the
instance because `Highs::postsolve` mutates it and cache entries are
shared.

Prediction: `glass4`, `ic97_potential`, and `neos-3046615-murg` at the
10-second, seed-0 benchmark report audited feasible incumbents
(`Audit=yes`, status `Time limit reached`) with gaps at or below the
previously discarded values (`0.66`, `0.094`, `0.796`).  The
thirty-instance sweep rises from 4 to 7 native audited-feasible runs.
Both fixed gates are unchanged (`neos-1122047` audited `161` below five
seconds through the paired fast path; `p200x1188c` native pipeline
untouched).  A focused regression round-trips a known original solution
through forward map and retained-instance postsolve exactly.

Assumptions: the retained instance's postsolve lifecycle is intact
(presolve-only, `run()` never called on it); primal-only postsolve
reconstructs eliminated columns for MIP reductions (existing
`highs_postsolve_primal` production behaviour); the working-space
objective plus `strict_highs_presolved_objective_offset` equals the
original objective under HiGHS presolve invariants (checked end-to-end
by the benchmark audit); postsolve of the same pass is deterministic.

References: vendored `Highs::postsolve` and `Highs::callRunPostsolve`
(MIP-without-basis branch); `HighsPostsolveStack`; the
"Coupled HiGHS Root-Primal Ownership" lifecycle mismatch above;
`highs_postsolve_primal` in
`src/engine/strategy/highs_presolve_side_state.cpp`.

Validation: baseline commit `1b51f0bd8caa` plus the campaign diff, macOS
26.5.2 arm64, Release `-O3 -DNDEBUG`, seed 0.  Both suites; the focused
round-trip regression; the three class-3 instances at the 10-second
limit; both fixed gates; the thirty-instance sweep.  A missing audit on
any of the three, a changed gate result, or a sweep regression triggers
the mismatch protocol before further edits.

### Postsolve-ownership validation

Test construction required one adjustment before measurement: the first
toy regression model (two binaries plus a doubleton-eliminable
continuous) presolved to `reduced_to_empty`, and on the corrected
knapsack-based model the affine forward map returned empty because a
surviving column was not linearly transformable after presolve fixings.
Neither invalidates the claim — tree incumbents originate natively in
working coordinates, so the regression was restructured to produce the
working-space point exactly as production does (solving the presolved
working model) before postsolving it.  The retained-instance postsolve
itself worked unchanged.

Every prediction component was then met with the right sign:

- Suites: `test_branch_and_cut` passed 490 assertions in 35 cases
  (including the retention-upgrade, round-trip, and wrong-dimension
  rejection checks); `test_milp_solver` passed 2034 assertions in 55
  cases.
- Class-3 instances at the 10-second limit: `glass4` gap `0.66`,
  `ic97_potential` gap `0.0941`, `neos-3046615-murg` gap `0.796`, all
  `Audit=yes` with status `Time limit reached` — exactly the previously
  discarded incumbents, now published and audited in original
  coordinates.
- Fixed gates unchanged: `neos-1122047` audited `161`,
  `Optimal (root gap closed)`, `3.772 s`; `p200x1188c` audited gap
  `0.0761`, `1.281 s`.
- Thirty-instance sweep
  (`/tmp/root_parity_sweep_class3_2026-08-13.{csv,json}`): native
  audited-feasible rose from 4 to 7, matching the predicted count.
  HiGHS remains at 13.

The `strict root certificate mismatch` loud-fail class is empty.  The
remaining parity gap on this sample is owned by class 4 (nine native
hard process timeouts versus five for HiGHS: `n2seq36q`,
`neos-5104907-jarama`, `piperout-27`, `sorrell3`, `supportcase6` are
native-only) and class 5 (`Root relaxation failed` on
`neos-3656078-kumeu`, `rail02`, `satellites2-40`).

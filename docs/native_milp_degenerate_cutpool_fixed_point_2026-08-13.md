# Native MILP Degenerate Cutpool Fixed Point

## Defect

The transformed root cutpool selected rows that were violated, efficacious,
and sufficiently nonparallel, appended them to the live root LP, and obtained
a successful optimal re-solve. When the objective did not move immediately,
the native lifecycle rolled those rows back to the available pool and stopped
the source loop.

That is not a valid cutting-plane fixed-point test on a degenerate LP. A cut
can remove part of the current optimal face without changing the objective.
Keeping the cut changes the basic solution and permits later separation to
cut a different part of that face. A fixed point is reached only after the
separator/cutpool lifecycle makes no progress for its configured stall budget,
not after one accepted batch has zero immediate objective lift.

Reference: T. Achterberg, *Constraint Integer Programming*, PhD thesis, 2007,
Sections 4.1-4.2 (LP-based cutting-plane loop); HiGHS
`HighsCutPool::separate` and `HighsMipSolverData::separationRound` lifecycle.

## Baseline Evidence

Baseline commit: `1b51f0b`, macOS ARM64, Release `-O3 -DNDEBUG`, one thread.

Diagnostic command:

```text
MIPSOLVERS_XTAB_DIAG=1 MIPSOLVERS_BC_TIMELINE=1 \
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-probe-max 0 --native-verbose
```

The transformed source loop lifted the reduced root objective from
`5.136438176` to `64.707795369`. In its next round the cutpool still had 276
violated active rows and selected 13 rows, but the successful re-solve moved
the objective by only `1.421e-14`. The implementation rolled those rows back
and stopped. The following generic Gomory/MIR rounds admitted 52 rows and
reached `65.30500564`; root probing finished at `65.3176477`, corresponding
to approximately `94.3176477` after the retained presolve objective offset.

## Rationale And Fixed Validation Contract

Model/algorithm: a degenerate cutting-plane fixed point over a global
valid-cut pool. A violated cut can leave the objective unchanged while
removing part of the current optimal face; subsequent cuts are separated
against the new face.

Claim: transformed-pool rows that pass validity, efficacy, and diversity
filters remain LP-owned after a successful re-solve even when their immediate
objective lift is zero. The existing three-round smooth-progress stall guard
owns termination.

Cost model: at most the existing three zero-progress rounds, typically 8-13
extra sparse rows and one warm HiGHS resolve per round. On `sp150x300d`, the
predicted added root cost is below 0.10 seconds and must remain below the fixed
0.50-second acceptance ceiling.

Prediction: the final certified reduced root bound is at least `66.305`
(original-space bound at least `95.305`) versus `65.318` / `94.318`. At 500
nodes, best bound is at least `95.3` or relative gap is at most `0.092`, with
runtime at most 3.0 seconds. At 10 seconds the solver processes at least 2000
nodes if the tree remains. Every reported incumbent passes the original-model
audit.

Assumptions: the 276 remaining violated rows contain cuts that constrain
adjacent parts of the degenerate optimal face; accepted rows are globally
valid; and warm row append remains numerically stable.

Validation commands:

```text
./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 1 --seed 0 \
  --native-probe-max 0 --native-verbose

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 30 --hard-timeout-grace 2 --max-nodes 500 --seed 0 \
  --native-probe-max 0

./tests/miplib2017_benchmark \
  --data-dir tests/data/miplib2017/benchmark \
  --solu tests/data/miplib2017/miplib2017-v36.solu \
  --solvers native-highs-lp --case sp150x300d \
  --time-limit 10 --hard-timeout-grace 2 --max-nodes 5000 --seed 0 \
  --native-probe-max 0
```

After acceptance, run `test_branch_and_cut`, `test_milp_solver`, and the fixed
`sp150x300d,p200x1188c,neos-1122047` five-second cohort. A wrong-sign result or
a deviation greater than 50% of the predicted effect triggers the mismatch
protocol before another algorithm change.

## First Measurement And Re-Derivation

Retaining zero-lift rows met the dual-bound prediction but failed the end-to-end
runtime contract. The transformed source loop retained 105 additional rows,
finished at `66.28756699`, and the generic/probing stages finished at
`66.39578305`. This is a `+1.07813535` reduced-space improvement over the
`65.3176477` baseline and exceeds the predicted `+1.0`. Root cut processing
rose from 268 ms to 529 ms, within the fixed 0.50-second added-cost ceiling.

However, the changed root face increased feasibility-pump time from about
65 ms to 25.073 seconds. The one-node run therefore took 26.127 seconds, so
the precommitted 500-node runtime maximum of 3.0 seconds cannot be met. The
change is rejected despite the stronger bound.

Mismatch protocol:

1. Implementation infidelity: not found in the cutpool change. All selected
   batches were retained, every re-solve was successful and monotone, and the
   incumbent passed the original-model audit.
2. Machine/cost-model error: the separator cost model was accurate; added
   cut generation and re-solves cost 261 ms, below the 500 ms ceiling.
3. Assumption violation: confirmed. The model assumed separator overhead was
   the only downstream cost. The feasibility pump is highly sensitive to the
   changed degenerate root point/basis and consumed the remaining 30-second
   solve budget even though separation itself was cheap.
4. Theory error: the fixed-point claim remains valid, but it is insufficient
   as a production policy. A stronger root face is not useful when the primal
   pipeline can consume the tree budget in response.

No cut count, tolerance, or heuristic budget is tuned from this failed run.
The production rollback remains until a new theory jointly bounds dual
separation and downstream root-primal work.

## Shallow-Node Cutpool Separation Contract

The reverted baseline exports 1,529 unadmitted transformed rows to the global
tree cutpool, but a fixed `base_lp.A.rows() <= 500` gate prevents every node LP
separation on `sp150x300d`. The root-augmented LP has approximately 649 rows;
the fixed 500-row boundary therefore produces `pool_sep=0` through 500 nodes
even though the rows remain available for domain propagation.

Model/algorithm: shallow-node re-separation of globally valid root cuts. A
global cut that is violated after branching can be appended to that node LP,
whose optimum is a valid stronger node dual bound.

Claim: admitting the existing cutpool path through 1,000 base rows restores
separation for medium sparse LPs without changing cut validity, root behavior,
or nodes below the existing depth-three limit.

Cost model: at most 14 shallow children, each performing one sparse violation
scan and one approximately 669-row IPM resolve after selecting at most 20
cuts. Predicted added time through 500 nodes is below 0.8 seconds.

Prediction: the fixed 500-node command reports `pool_sep > 0` and either a
reduced best bound of at least `65.98` (approximately `94.98` original space)
or relative gap at most `0.095`, versus baseline `65.864` and `0.09775`.
Runtime remains at most 3.0 seconds and the incumbent passes audit.

Assumptions: at least one of the 1,529 LP-visible pool rows becomes violated
under a shallow branch domain; the existing IPM cut resolve is stable at this
matrix size; and root/tree execution is deterministic at seed zero.

Reference: Achterberg (2007), Sections 4.1-4.2; HiGHS cutpool separation and
LP-row lifecycle. Validation uses the fixed 500-node command above, followed
by the 10-second run and both MILP suites only if this contract is accepted.

## Shallow-Node Measurement And Re-Derivation

The 1,000-row gate activated the intended path and separated 160 pool rows
through 500 nodes. The exact verbose measurement took 2.361 seconds, within
the 3.0-second ceiling, but the reduced best bound fell from `65.864` to
`65.532` and the relative gap worsened from `0.09775` to `0.102`. A separate
nonverbose run took 3.255 seconds and reported gap `0.0968`; it also missed the
fixed `0.095` target and exceeded the runtime ceiling. The change is rejected.

Mismatch protocol:

1. Implementation infidelity: not found. `pool_sep=160` proves that the
   intended global rows were selected and the incumbent audit passed.
2. Machine/cost-model error: the measured wall cost was timing-sensitive and
   ranged from approximately 0.17 to 1.07 seconds above baseline, rather than
   remaining below the predicted 0.8 seconds on every run.
3. Assumption violation: confirmed. Stronger individual shallow-node LPs
   changed branching and queue composition, but did not improve the minimum
   certified frontier at a fixed 500-node count. This is the same distinction
   between local child strength and global best-bound progress established by
   the reliability-branching experiment.
4. Theory error: global validity guarantees monotone bounds only for the node
   receiving a cut, not for a different search frontier produced after 500
   branching decisions. The fixed-node prediction incorrectly conflated the
   two.

The production 500-row gate is restored. No row threshold, depth, or scan
interval is tuned from this rejected experiment.
